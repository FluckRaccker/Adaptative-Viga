#!/usr/bin/env python3
import os
import mmap
import struct
import time
import re
from pathlib import Path

import pandas as pd


# ============================================================
# Endereços da bridge
# ============================================================
BRIDGE = 0xC0000000
BRIDGE_SPAN = 0x100

IN_A       = 0x00
IN_B       = 0x08
OUT_EXPORT = 0x10
OUT_DATA   = 0x18


# ============================================================
# Clock / NCO
# ============================================================
CLK_HZ = 50_000_000
PHASE_BITS = 32


# ============================================================
# Opcodes do simple_cmd_storage
# ============================================================
CMD_NOP             = 0x00

CMD_SET_FREQ_DAC1   = 0x01
CMD_SET_FREQ_DAC2   = 0x02

CMD_SET_OFF_DAC1    = 0x03
CMD_SET_OFF_DAC2    = 0x04

CMD_SET_ASIN_DAC1   = 0x05
CMD_SET_ASIN_DAC2   = 0x06

CMD_SET_ANOISE_DAC1 = 0x07
CMD_SET_ANOISE_DAC2 = 0x08

CMD_SET_AFC_DAC1    = 0x09
CMD_SET_AFC_DAC2    = 0x0A

CMD_SET_CFG_DAC1    = 0x0B
CMD_SET_CFG_DAC2    = 0x0C

CMD_SET_COEFF_WF    = 0x20
CMD_SET_COEFF_WS    = 0x21

CMD_CLEAR_PENDING   = 0x22
CMD_CLEAR_OVERRUN   = 0x23

CMD_CONTROL_START   = 0x30
CMD_CONTROL_STOP    = 0x31

CMD_SET_MITIO       = 0x32
CMD_SET_CORRECTION_S = 0x33


# ============================================================
# Modos do DAC
# ============================================================
DAC_OFF        = 0
DAC_SINE       = 1
DAC_SINE_NOISE = 2
DAC_NOISE      = 3
DAC_FC         = 4

DAC_MODE_NAMES = {
    "off": DAC_OFF,
    "sine": DAC_SINE,
    "sin": DAC_SINE,
    "senoide": DAC_SINE,
    "sine_noise": DAC_SINE_NOISE,
    "sin_noise": DAC_SINE_NOISE,
    "senoide_ruido": DAC_SINE_NOISE,
    "noise": DAC_NOISE,
    "ruido": DAC_NOISE,
    "fc": DAC_FC,
    "adapt": DAC_FC,
    "adaptativo": DAC_FC,
}


# ============================================================
# Acesso básico
# ============================================================
def write_u64(mm, offset, value):
    mm[offset:offset + 8] = struct.pack("<Q", value & 0xFFFFFFFFFFFFFFFF)


def read_u64(mm, offset):
    return struct.unpack("<Q", mm[offset:offset + 8])[0]


def parse_int_auto(text):
    return int(str(text), 0)


def to_signed(x, bits):
    x &= (1 << bits) - 1
    if x & (1 << (bits - 1)):
        return x - (1 << bits)
    return x


def sat_u(value, bits):
    value = int(value)
    maxv = (1 << bits) - 1
    if value < 0:
        return 0
    if value > maxv:
        return maxv
    return value


def hz_to_phase_inc(freq_hz, clk_hz=CLK_HZ, phase_bits=PHASE_BITS):
    return int(round((float(freq_hz) * (1 << phase_bits)) / clk_hz))


def phase_inc_to_hz(phase_inc, clk_hz=CLK_HZ, phase_bits=PHASE_BITS):
    return (int(phase_inc) * clk_hz) / (1 << phase_bits)


def pack4_signed16(c0, c1, c2, c3):
    """
    Empacota 4 coeficientes signed 16 bits em 64 bits.

    coeff_in[15:0]   = c0
    coeff_in[31:16]  = c1
    coeff_in[47:32]  = c2
    coeff_in[63:48]  = c3
    """
    coeffs = [c0, c1, c2, c3]
    payload = 0

    for i, c in enumerate(coeffs):
        payload |= (int(c) & 0xFFFF) << (16 * i)

    return payload & 0xFFFFFFFFFFFFFFFF


def alpha_to_q15(alpha):
    q = int(round(float(alpha) * 32768.0))

    if q < 0:
        q = 0
    elif q > 32767:
        q = 32767

    return q


def q15_to_alpha(q):
    return int(q) / 32768.0


# ============================================================
# Decode OUT_EXPORT
#
# out_export = {8'd0, dac_signal[23:0], accel_z[31:0]}
# dac_signal = dac1[11:0] & dac2[11:0]
# accel_z    = imu1[15:0] & imu2[15:0]
# ============================================================
def unpack_out_export(raw):
    raw &= 0xFFFFFFFFFFFFFFFF

    accel_z = raw & 0xFFFFFFFF

    accel_1_u16 = (accel_z >> 16) & 0xFFFF
    accel_2_u16 = accel_z & 0xFFFF

    accel_1 = to_signed(accel_1_u16, 16)
    accel_2 = to_signed(accel_2_u16, 16)

    dac_pack = (raw >> 32) & 0xFFFFFF
    dac_1 = (dac_pack >> 12) & 0xFFF
    dac_2 = dac_pack & 0xFFF

    return {
        "raw_export": raw,
        "dac_1": dac_1,
        "dac_2": dac_2,
        "accel_1": accel_1,
        "accel_2": accel_2,
        "accel_z_u32": accel_z,
        "dac_pack": dac_pack,
    }


# ============================================================
# Decode OUT_DATA
#
# Novo formato do FPGA:
# [63:56] = reservado, atualmente 0
# [55:48] = last_opcode
# [47:36] = coeff_count_dbg[11:0]
# [35:24] = coeff_count_dbg[23:12]
# [23:16] = status_flags
# [15:8]  = config_status
# [7:0]   = reservado, atualmente 0
#
# status_flags:
#   [7] coeff_pending[1]  ws
#   [6] coeff_pending[0]  wf
#   [5] coeff_ready[1]    ws
#   [4] coeff_ready[0]    wf
#   [3] coeff_overrun[1]  ws
#   [2] coeff_overrun[0]  wf
#   [1] load_config
#   [0] sample_valid
#
# config_status esperado:
#   [7] correction_s
#   [6] control_enable
#   [5:3] config_dac2
#   [2:0] config_dac1
# ============================================================
def unpack_out_data(raw):
    raw &= 0xFFFFFFFFFFFFFFFF

    reserved_hi    = (raw >> 56) & 0xFF
    last_opcode    = (raw >> 48) & 0xFF

    coeff_count_lo = (raw >> 36) & 0xFFF
    coeff_count_hi = (raw >> 24) & 0xFFF

    status_flags   = (raw >> 16) & 0xFF
    config_status  = (raw >> 8)  & 0xFF
    reserved_lo    = raw & 0xFF

    # Atenção:
    # Se no datapath você fez:
    #   coeff_count_dbg <= coeff_count_wf & coeff_count_ws;
    # então:
    #   coeff_count_dbg[23:12] = wf
    #   coeff_count_dbg[11:0]  = ws
    #
    # Como no simple_cmd_storage você colocou [11:0] antes no out_data,
    # o campo "lo" aparece antes no pacote.
    coeff_count_ws = coeff_count_lo
    coeff_count_wf = coeff_count_hi

    return {
        "raw_data": raw,

        "reserved_hi": reserved_hi,
        "reserved_lo": reserved_lo,

        "last_opcode": last_opcode,

        "coeff_count_wf": coeff_count_wf,
        "coeff_count_ws": coeff_count_ws,

        "status_flags": status_flags,
        "config_status": config_status,

        "coeff_pending_wf": (status_flags >> 6) & 1,
        "coeff_pending_ws": (status_flags >> 7) & 1,

        "coeff_ready_wf": (status_flags >> 4) & 1,
        "coeff_ready_ws": (status_flags >> 5) & 1,

        "coeff_overrun_wf": (status_flags >> 2) & 1,
        "coeff_overrun_ws": (status_flags >> 3) & 1,

        "load_config": (status_flags >> 1) & 1,
        "sample_valid": status_flags & 1,

        "config_dac1": config_status & 0x7,
        "config_dac2": (config_status >> 3) & 0x7,

        "control_enable": (config_status >> 6) & 0x1,
        "correction_s": (config_status >> 7) & 0x1,

        # Mantém compatibilidade com partes antigas do código.
        # Você removeu sample_counter do out_data novo.
        "sample_counter": None,
        "command_counter": None,
    }


# ============================================================
# Leitura de coeficientes
# ============================================================
def normalize_coeff_list(values):
    coeffs = []
    for x in values:
        if pd.isna(x):
            continue
        coeffs.append(int(x))
    return coeffs


def read_coeffs_txt(filename):
    """
    Aceita TXT com coeficientes separados por:
      espaço, vírgula, ponto e vírgula, tab ou quebra de linha.

    Exemplo:
      32767 0 0 0
      0 0 0 0

    Ou:
      32767,0,0,0,0,0,0,0
    """
    text = Path(filename).read_text(encoding="utf-8", errors="ignore")

    # Remove comentários iniciados por #
    lines = []
    for line in text.splitlines():
        line = line.split("#", 1)[0]
        lines.append(line)

    text = "\n".join(lines)

    tokens = re.split(r"[,\s;]+", text.strip())
    tokens = [t for t in tokens if t]

    return [parse_int_auto(t) for t in tokens]


def read_coeffs_csv(filename, column=None):
    """
    Aceita CSV com:
      - coluna "coeff";
      - coluna "wf" ou "ws";
      - primeira coluna numérica;
      - ou CSV sem header com uma coluna de números.
    """
    path = Path(filename)

    try:
        df = pd.read_csv(path)
    except Exception:
        df = pd.read_csv(path, header=None)

    if column is not None:
        if column not in df.columns:
            raise ValueError(f"Coluna '{column}' não encontrada no CSV.")
        return normalize_coeff_list(df[column].tolist())

    if "coeff" in df.columns:
        return normalize_coeff_list(df["coeff"].tolist())

    numeric_cols = df.select_dtypes(include="number").columns
    if len(numeric_cols) > 0:
        return normalize_coeff_list(df[numeric_cols[0]].tolist())

    # Tenta CSV sem header
    df2 = pd.read_csv(path, header=None)
    return normalize_coeff_list(df2.iloc[:, 0].tolist())


def extract_coeffs_from_pkl_object(obj, filt=None):
    """
    Aceita:
      1) lista/tupla/array:
           [32767, 0, 0, 0, ...]

      2) dict:
           {"wf": [...], "ws": [...]}

      3) DataFrame:
           coluna "wf" ou "ws"
           coluna "coeff"
           ou primeira coluna numérica
    """
    filt = None if filt is None else filt.lower()

    if isinstance(obj, dict):
        if filt is None:
            result = {}
            if "wf" in obj:
                result["wf"] = normalize_coeff_list(obj["wf"])
            if "ws" in obj:
                result["ws"] = normalize_coeff_list(obj["ws"])
            if not result:
                raise ValueError("Dict precisa ter chave 'wf' e/ou 'ws'.")
            return result

        if filt not in obj:
            raise ValueError(f"Dict não contém a chave '{filt}'.")
        return normalize_coeff_list(obj[filt])

    if isinstance(obj, pd.DataFrame):
        if filt is not None and filt in obj.columns:
            return normalize_coeff_list(obj[filt].tolist())

        if "coeff" in obj.columns:
            return normalize_coeff_list(obj["coeff"].tolist())

        numeric_cols = obj.select_dtypes(include="number").columns
        if len(numeric_cols) == 0:
            raise ValueError("DataFrame não tem coluna numérica para coeficientes.")

        return normalize_coeff_list(obj[numeric_cols[0]].tolist())

    if hasattr(obj, "tolist"):
        obj = obj.tolist()

    if isinstance(obj, (list, tuple)):
        return normalize_coeff_list(obj)

    raise ValueError("Formato de PKL não reconhecido.")


def read_coeffs_pkl(filename, filt=None):
    obj = pd.read_pickle(filename)
    return extract_coeffs_from_pkl_object(obj, filt=filt)


def read_coeffs_file(filename, filt=None, column=None):
    suffix = Path(filename).suffix.lower()

    if suffix == ".pkl":
        return read_coeffs_pkl(filename, filt=filt)

    if suffix == ".txt":
        return read_coeffs_txt(filename)

    if suffix == ".csv":
        return read_coeffs_csv(filename, column=column)

    raise ValueError("Formato não suportado. Use .pkl, .txt ou .csv.")


# ============================================================
# Classe principal
# ============================================================
class BridgeCLI:
    def __init__(self, mm):
        self.mm = mm
        self.seq = 0

    # ------------------------------------------------------------
    # Comando base
    # ------------------------------------------------------------
    def send_cmd(self, opcode, data=0, delay_s=0.0005):
        """
        IN_A = data
        IN_B = (seq << 8) | opcode
        """
        self.seq = (self.seq + 1) & ((1 << 56) - 1)

        cmd_word = (self.seq << 8) | (opcode & 0xFF)

        write_u64(self.mm, IN_A, data)
        write_u64(self.mm, IN_B, cmd_word)

        if delay_s > 0:
            time.sleep(delay_s)

    # ------------------------------------------------------------
    # Leituras
    # ------------------------------------------------------------
    def raw_export(self):
        return read_u64(self.mm, OUT_EXPORT)

    def raw_data(self):
        return read_u64(self.mm, OUT_DATA)

    def read_export(self):
        return unpack_out_export(self.raw_export())

    def read_status(self):
        return unpack_out_data(self.raw_data())

    def read(self):
        ex = self.read_export()

        print(f"OUT_EXPORT = 0x{ex['raw_export']:016X}")
        print(f"dac_1       = {ex['dac_1']}")
        print(f"dac_2       = {ex['dac_2']}")
        print(f"accel_1     = {ex['accel_1']}")
        print(f"accel_2     = {ex['accel_2']}")

        return ex

    def status(self):
        st = self.read_status()

        print(f"OUT_DATA raw       = 0x{st['raw_data']:016X}")
        print(f"last_opcode        = 0x{st['last_opcode']:02X}")
        print(f"coeff_count wf/ws  = {st['coeff_count_wf']} / {st['coeff_count_ws']}")
        print("")
        print(f"pending wf/ws      = {st['coeff_pending_wf']} / {st['coeff_pending_ws']}")
        print(f"ready   wf/ws      = {st['coeff_ready_wf']} / {st['coeff_ready_ws']}")
        print(f"overrun wf/ws      = {st['coeff_overrun_wf']} / {st['coeff_overrun_ws']}")
        print("")
        print(f"config DAC1        = {st['config_dac1']}")
        print(f"config DAC2        = {st['config_dac2']}")
        print(f"sample_valid       = {st['sample_valid']}")
        print(f"load_config        = {st['load_config']}")
        print(f"control_enable     = {st['control_enable']}")
        print(f"correction_s       = {st['correction_s']}")
        return st

    # ------------------------------------------------------------
    # Controle adaptativo interno
    # ------------------------------------------------------------
    def _control_start(self):
        self.send_cmd(CMD_CONTROL_START, 0)
        print("FSM adaptativa ligada.")

    def _control_stop(self):
        self.send_cmd(CMD_CONTROL_STOP, 0)
        print("FSM adaptativa desligada.")

    # ------------------------------------------------------------
    # DAC
    # ------------------------------------------------------------
    def set_dac_mode(self, dac, mode):
        dac = int(dac)

        if isinstance(mode, str):
            key = mode.lower()
            if key not in DAC_MODE_NAMES:
                raise ValueError(f"Modo inválido: {mode}")
            mode_val = DAC_MODE_NAMES[key]
        else:
            mode_val = int(mode)

        if mode_val < 0 or mode_val > 4:
            raise ValueError("Modo do DAC deve estar entre 0 e 4.")

        if dac == 1:
            self.send_cmd(CMD_SET_CFG_DAC1, mode_val)
        elif dac == 2:
            self.send_cmd(CMD_SET_CFG_DAC2, mode_val)
        else:
            raise ValueError("DAC deve ser 1 ou 2.")

        print(f"DAC{dac} modo = {mode_val}")

    def set_freq_hz(self, dac, freq_hz):
        phase_inc = hz_to_phase_inc(freq_hz)

        if int(dac) == 1:
            self.send_cmd(CMD_SET_FREQ_DAC1, phase_inc)
        elif int(dac) == 2:
            self.send_cmd(CMD_SET_FREQ_DAC2, phase_inc)
        else:
            raise ValueError("DAC deve ser 1 ou 2.")

        print(f"DAC{dac}: freq={freq_hz} Hz, phase_inc=0x{phase_inc:08X}")

    def set_offset(self, dac, offset):
        offset = sat_u(offset, 12)

        if int(dac) == 1:
            self.send_cmd(CMD_SET_OFF_DAC1, offset)
        elif int(dac) == 2:
            self.send_cmd(CMD_SET_OFF_DAC2, offset)
        else:
            raise ValueError("DAC deve ser 1 ou 2.")

        print(f"DAC{dac}: offset={offset}")

    def set_amp_sin(self, dac, amp):
        amp = sat_u(amp, 7)

        if int(dac) == 1:
            self.send_cmd(CMD_SET_ASIN_DAC1, amp)
        elif int(dac) == 2:
            self.send_cmd(CMD_SET_ASIN_DAC2, amp)
        else:
            raise ValueError("DAC deve ser 1 ou 2.")

        print(f"DAC{dac}: amp_sin={amp}")

    def set_amp_noise(self, dac, amp):
        amp = sat_u(amp, 7)

        if int(dac) == 1:
            self.send_cmd(CMD_SET_ANOISE_DAC1, amp)
        elif int(dac) == 2:
            self.send_cmd(CMD_SET_ANOISE_DAC2, amp)
        else:
            raise ValueError("DAC deve ser 1 ou 2.")

        print(f"DAC{dac}: amp_noise={amp}")

    def set_amp_fc(self, dac, amp):
        amp = sat_u(amp, 7)

        if int(dac) == 1:
            self.send_cmd(CMD_SET_AFC_DAC1, amp)
        elif int(dac) == 2:
            self.send_cmd(CMD_SET_AFC_DAC2, amp)
        else:
            raise ValueError("DAC deve ser 1 ou 2.")

        print(f"DAC{dac}: amp_fc={amp}")

    def cfg_dac(self, dac, freq_hz, offset, amp_sin, amp_noise, amp_fc=None, mode=None):
        self.set_freq_hz(dac, freq_hz)
        self.set_offset(dac, offset)
        self.set_amp_sin(dac, amp_sin)
        self.set_amp_noise(dac, amp_noise)

        if amp_fc is not None:
            self.set_amp_fc(dac, amp_fc)

        if mode is not None:
            self.set_dac_mode(dac, mode)

    def set_mitio_raw(self, value):
        value = int(value) & 0xFFFF
        self.send_cmd(CMD_SET_MITIO, value)
        print(f"mitio = 0x{value:04X} ({q15_to_alpha(value):.8f} em Q1.15)")

    def set_mitio_alpha(self, alpha):
        q = alpha_to_q15(alpha)
        self.set_mitio_raw(q)

    def set_correction_s(self, enable=True):
        value = 1 if bool(enable) else 0
        self.send_cmd(CMD_SET_CORRECTION_S, value)
        print(f"correction_s = {value}")

    def sweep_amp_sin(
        self,
        dac,
        amp_start,
        amp_stop,
        amp_step,
        dwell_s,
        poll_ms=1.0,
        filename=None,
        set_mode=True,
    ):
        """
        Varre a amplitude da senoide no DAC escolhido.

        Exemplo:
          sweep_amp_sin(
              dac=1,
              amp_start=10,
              amp_stop=100,
              amp_step=10,
              dwell_s=5
          )

        Se filename for fornecido (.pkl ou .csv), salva as leituras
        durante cada patamar da varredura.
        """
        dac = int(dac)
        amp_start = int(amp_start)
        amp_stop = int(amp_stop)
        amp_step = int(amp_step)
        dwell_s = float(dwell_s)
        poll_ms = float(poll_ms)

        if dac not in (1, 2):
            raise ValueError("DAC deve ser 1 ou 2.")

        if dwell_s <= 0:
            raise ValueError("tempo_s deve ser maior que zero.")

        if amp_step == 0:
            raise ValueError("passo não pode ser zero.")

        # Corrige automaticamente o sinal do passo.
        if amp_start < amp_stop and amp_step < 0:
            amp_step = abs(amp_step)
        elif amp_start > amp_stop and amp_step > 0:
            amp_step = -amp_step

        amps = []
        amp = amp_start

        if amp_step > 0:
            while amp <= amp_stop:
                amps.append(amp)
                amp += amp_step
        else:
            while amp >= amp_stop:
                amps.append(amp)
                amp += amp_step

        if not amps:
            raise ValueError("Varredura vazia. Verifique início, fim e passo.")

        if set_mode:
            self.set_dac_mode(dac, DAC_SINE)

        dfs = []
        sweep_t0 = time.perf_counter()

        print(
            f"Iniciando varredura DAC{dac}: "
            f"{amps[0]} -> {amps[-1]}, passo {amp_step}, "
            f"{dwell_s} s por patamar."
        )

        for step_idx, amp in enumerate(amps):
            amp_limited = sat_u(amp, 7)

            print(f"DAC{dac}: amp_sin={amp_limited} por {dwell_s} s")
            self.set_amp_sin(dac, amp_limited)

            if filename is None:
                time.sleep(dwell_s)
            else:
                step_t0 = time.perf_counter() - sweep_t0
                df_step = self.collect_df(
                    duration_s=dwell_s,
                    poll_ms=poll_ms,
                    phase=f"amp_sin_{amp_limited}",
                )

                df_step["sweep_dac"] = dac
                df_step["sweep_step"] = step_idx
                df_step["sweep_amp_sin"] = amp_limited
                df_step["sweep_dwell_s"] = dwell_s
                df_step["sweep_step_t0_s"] = step_t0
                df_step["sweep_t_s"] = df_step["sweep_step_t0_s"] + df_step["t_s"]

                dfs.append(df_step)

        if filename is not None:
            df = pd.concat(dfs, ignore_index=True) if dfs else pd.DataFrame()

            filename = str(filename)
            suffix = Path(filename).suffix.lower()

            if suffix == ".csv":
                df.to_csv(filename, index=False)
            else:
                df.to_pickle(filename)

            print(f"Arquivo salvo: {filename}")
            print(f"Amostras lidas pelo Python: {len(df)}")
            return df

        print("Varredura finalizada.")
        return None

    def sweep_amp_sin_pkl(
        self,
        dac,
        amp_start,
        amp_stop,
        amp_step,
        dwell_s,
        filename="/root/sweep_amp.pkl",
        poll_ms=1.0,
        set_mode=True,
    ):
        return self.sweep_amp_sin(
            dac=dac,
            amp_start=amp_start,
            amp_stop=amp_stop,
            amp_step=amp_step,
            dwell_s=dwell_s,
            poll_ms=poll_ms,
            filename=filename,
            set_mode=set_mode,
        )

    # ------------------------------------------------------------
    # Coeficientes
    # ------------------------------------------------------------
    def clear_overrun(self):
        self.send_cmd(CMD_CLEAR_OVERRUN, 0)
        print("Flags de overrun limpas.")

    def clear_pending(self):
        self.send_cmd(CMD_CLEAR_PENDING, 0)
        print("Pendências de coeficientes limpas.")

    def _pending_and_overrun(self, filt):
        st = self.read_status()
        filt = filt.lower()

        if filt == "wf":
            return st["coeff_pending_wf"], st["coeff_overrun_wf"]

        if filt == "ws":
            return st["coeff_pending_ws"], st["coeff_overrun_ws"]

        raise ValueError("Filtro deve ser 'wf' ou 'ws'.")

    def wait_pending_clear(self, filt, timeout_s=3.0, poll_s=0.001):
        t0 = time.perf_counter()

        while True:
            pending, overrun = self._pending_and_overrun(filt)

            if overrun:
                raise RuntimeError(f"Overrun no carregamento de coeficientes do {filt}.")

            if pending == 0:
                return

            if time.perf_counter() - t0 > timeout_s:
                raise TimeoutError(
                    f"Timeout esperando coeff_pending do {filt} limpar. "
                    "Verifique se a FSM adaptativa foi ligada e se o FIR está carregando."
                )

            time.sleep(poll_s)

    def send_coeff_block(self, filt, c0, c1, c2, c3, timeout_s=3.0):
        filt = filt.lower()
        payload = pack4_signed16(c0, c1, c2, c3)

        if filt == "wf":
            opcode = CMD_SET_COEFF_WF
        elif filt == "ws":
            opcode = CMD_SET_COEFF_WS
        else:
            raise ValueError("Filtro deve ser 'wf' ou 'ws'.")

        self.send_cmd(opcode, payload)
        self.wait_pending_clear(filt, timeout_s=timeout_s)

        print(f"{filt}: bloco enviado [{c0}, {c1}, {c2}, {c3}]")

    def upload_coefficients_from_list(
        self,
        filt,
        coeffs,
        start_fsm=True,
        pad_to_multiple_of_4=True,
        timeout_s=3.0,
    ):
        filt = filt.lower()
        coeffs = [int(x) for x in coeffs]

        if pad_to_multiple_of_4:
            while len(coeffs) % 4 != 0:
                coeffs.append(0)
        elif len(coeffs) % 4 != 0:
            raise ValueError("A lista de coeficientes precisa ter tamanho múltiplo de 4.")

        if start_fsm:
            self._control_start()

        for i in range(0, len(coeffs), 4):
            self.send_coeff_block(
                filt,
                coeffs[i],
                coeffs[i + 1],
                coeffs[i + 2],
                coeffs[i + 3],
                timeout_s=timeout_s,
            )

        print(f"Upload de {len(coeffs)} coeficientes para {filt} finalizado.")

    def upload_coefficients_from_file(
        self,
        filt,
        filename,
        column=None,
        start_fsm=True,
        timeout_s=3.0,
    ):
        filt = filt.lower()
        coeffs = read_coeffs_file(filename, filt=filt, column=column)

        self.upload_coefficients_from_list(
            filt=filt,
            coeffs=coeffs,
            start_fsm=start_fsm,
            timeout_s=timeout_s,
        )

    def upload_coefficients_pkl_all(self, filename, start_fsm=False, timeout_s=3.0):
        data = read_coeffs_pkl(filename, filt=None)

        if not isinstance(data, dict):
            raise ValueError("Para coeff_pkl_all, o .pkl precisa ser dict com chaves 'wf' e/ou 'ws'.")

        if start_fsm:
            self._control_start()

        if "wf" in data:
            self.upload_coefficients_from_list(
                "wf",
                data["wf"],
                start_fsm=False,
                timeout_s=timeout_s,
            )

        if "ws" in data:
            self.upload_coefficients_from_list(
                "ws",
                data["ws"],
                start_fsm=False,
                timeout_s=timeout_s,
            )

    # ------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------
    def collect_df(self, duration_s, poll_ms=1.0, phase=""):
        duration_s = float(duration_s)
        poll_s = float(poll_ms) / 1000.0

        rows = []
        t0 = time.perf_counter()

        while True:
            t = time.perf_counter() - t0
            if t >= duration_s:
                break

            ex = self.read_export()
            st = self.read_status()

            rows.append({
                "t_s": t,
                "phase": phase,

                "accel_1": ex["accel_1"],
                "accel_2": ex["accel_2"],

                "dac_1": ex["dac_1"],
                "dac_2": ex["dac_2"],

                "sample_counter": st.get("sample_counter"),
                "sample_valid": st["sample_valid"],

                "coeff_count_wf": st["coeff_count_wf"],
                "coeff_count_ws": st["coeff_count_ws"],
                "control_enable": st["control_enable"],
                "correction_s": st["correction_s"],

                "config_dac1": st["config_dac1"],
                "config_dac2": st["config_dac2"],

                "pending_wf": st["coeff_pending_wf"],
                "pending_ws": st["coeff_pending_ws"],
                "ready_wf": st["coeff_ready_wf"],
                "ready_ws": st["coeff_ready_ws"],
                "overrun_wf": st["coeff_overrun_wf"],
                "overrun_ws": st["coeff_overrun_ws"],

                "raw_export": ex["raw_export"],
                "raw_data": st["raw_data"],
            })

            if poll_s > 0:
                time.sleep(poll_s)

        return pd.DataFrame(rows)

    def log_pkl(self, filename, duration_s, poll_ms=1.0):
        df = self.collect_df(duration_s, poll_ms)
        df.to_pickle(filename)
        print(f"Arquivo salvo: {filename}")
        print(f"Amostras lidas pelo Python: {len(df)}")

    def log_csv(self, filename, duration_s, poll_ms=1.0):
        df = self.collect_df(duration_s, poll_ms)
        df.to_csv(filename, index=False)
        print(f"Arquivo salvo: {filename}")
        print(f"Amostras lidas pelo Python: {len(df)}")

    # ------------------------------------------------------------
    # Adaptativo
    # ------------------------------------------------------------
    def adapt_on(self, dac=1, amp_fc=None):
        """
        Liga o adaptativo corretamente:

          1) ajusta amp_fc, se fornecido;
          2) coloca o DAC escolhido em modo FC;
          3) liga a FSM adaptativa.

        O modo FC separado não é exposto, porque fc(n) só faz sentido
        com a FSM rodando.
        """
        dac = int(dac)

        if amp_fc is not None:
            self.set_amp_fc(dac, amp_fc)

        self.set_dac_mode(dac, DAC_FC)
        self._control_start()

        print(f"Adaptativo ligado no DAC{dac}.")

    def adapt_off(self, dac=1, dac_off=True):
        """
        Desliga a FSM adaptativa.
        Por padrão também coloca o DAC em OFF.
        """
        dac = int(dac)

        self._control_stop()

        if dac_off:
            self.set_dac_mode(dac, DAC_OFF)
            print(f"DAC{dac} desligado.")

    def adapt_log(
        self,
        duration_s,
        filename,
        poll_ms=1.0,
        dac=1,
        amp_fc=None,
        pre_s=0.0,
        stop_after=False,
        dac_off_after=False,
    ):
        """
        Grava antes e depois de ligar o adaptativo.

        pre_s:
        tempo em segundos para gravar antes de ligar o adaptativo.

        duration_s:
        tempo em segundos para gravar depois de ligar o adaptativo.
        """

        dfs = []

        pre_s = float(pre_s)
        duration_s = float(duration_s)

        if pre_s > 0:
            print(f"Gravando {pre_s} s antes de ligar o adaptativo...")
            df_pre = self.collect_df(pre_s, poll_ms, phase="pre")
            dfs.append(df_pre)

        print("Ligando adaptativo...")
        self.adapt_on(dac=dac, amp_fc=amp_fc)

        print(f"Gravando {duration_s} s com adaptativo ligado...")
        df_adapt = self.collect_df(duration_s, poll_ms, phase="adapt")
        dfs.append(df_adapt)

        df = pd.concat(dfs, ignore_index=True)

        filename = str(filename)
        suffix = Path(filename).suffix.lower()

        if suffix == ".csv":
            df.to_csv(filename, index=False)
        else:
            df.to_pickle(filename)

        print(f"Arquivo salvo: {filename}")
        print(f"Amostras lidas pelo Python: {len(df)}")

        if stop_after:
            self.adapt_off(dac=dac, dac_off=dac_off_after)


# ============================================================
# Help / CLI
# ============================================================
def print_help():
    print("""
Comandos:

  help
  q

Leitura:
  read
  status
  raw

DAC manual:
  mode <dac> <modo>
      modos: off, sine, sine_noise, noise
      Observação: o modo fc é usado automaticamente por adapt_on/adapt_log.

  cfg_dac <dac> <freq_hz> <offset> <amp_sin> <amp_noise> [amp_fc] [modo]
      exemplo:
      cfg_dac 1 80 2048 30 0 10 sine

  freq <dac> <freq_hz>
  offset <dac> <offset>
  amp_sin <dac> <0..100>
  amp_noise <dac> <0..100>
  amp_fc <dac> <0..100>

Varredura de amplitude:
  amp_sweep <dac> <amp_ini> <amp_fim> <passo> <tempo_s> [poll_ms] [arquivo.pkl/csv] [set_mode=1]
      exemplo sem salvar:
      amp_sweep 1 10 100 10 5

      exemplo salvando PKL:
      amp_sweep 1 10 100 10 5 1 /root/sweep_amp.pkl

  sweep_pkl <dac> <amp_ini> <amp_fim> <passo> <tempo_s> [arquivo.pkl] [poll_ms]
      exemplo padrão:
      sweep_pkl 1 10 100 10 5

      exemplo com nome:
      sweep_pkl 1 10 100 10 5 /root/sweep_80hz.pkl

Configuração extra:
  corr <0/1>
      altera correction_s no FPGA

Coeficientes:
  coeffs <wf/ws> <c0> <c1> <c2> ...
      exemplo:
      coeffs wf 32767 0 0 0 0 0 0 0

  coeff_file <wf/ws> <arquivo.pkl/txt/csv> [coluna_csv]
      exemplo:
      coeff_file wf wf_coeffs.txt
      coeff_file ws ws_coeffs.csv coeff

  coeff_txt <wf/ws> <arquivo.txt>
  coeff_csv <wf/ws> <arquivo.csv> [coluna]
  coeff_pkl <wf/ws> <arquivo.pkl>

  coeff_pkl_all <arquivo.pkl>
      espera dict:
      {"wf": [...], "ws": [...]}

  clear_pending
  clear_overrun

Adaptativo:
  adapt_on [dac] [amp_fc]
      liga FSM adaptativa e coloca o DAC escolhido em modo FC

  adapt_log <duracao_adapt_s> <arquivo.pkl/csv> [poll_ms] [dac] [amp_fc] [pre_s]
      grava pre_s segundos antes de ligar o adaptativo
      depois liga o adaptativo e grava duracao_adapt_s segundos

  adapt_off [dac] [off_dac=1]
      desliga FSM adaptativa e opcionalmente desliga DAC

Log sem alterar controle:
  log_pkl <arquivo.pkl> <duracao_s> [poll_ms]
  log_csv <arquivo.csv> <duracao_s> [poll_ms]

Passo adaptativo:
  mitio <valor_int/hex>
      exemplo: mitio 0x0200

  alpha <valor_real>
      exemplo: alpha 0.015625
      exemplo: alpha 0.05
""")


def main():
    fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)

    try:
        mm = mmap.mmap(
            fd,
            BRIDGE_SPAN,
            flags=mmap.MAP_SHARED,
            prot=mmap.PROT_READ | mmap.PROT_WRITE,
            offset=BRIDGE,
        )

        try:
            cli = BridgeCLI(mm)
            write_u64(mm, IN_B, 0)

            print("=== Bridge CLI - Viga / MEMS / DAC / FxNLMS ===")
            print_help()

            while True:
                s = input(">> ").strip()

                if not s:
                    continue

                if s.lower() == "q":
                    print("Saindo...")
                    break

                p = s.split()
                cmd = p[0].lower()

                try:
                    if cmd == "help":
                        print_help()

                    elif cmd == "read":
                        cli.read()

                    elif cmd == "status":
                        cli.status()

                    elif cmd == "raw":
                        print(f"OUT_EXPORT = 0x{cli.raw_export():016X}")
                        print(f"OUT_DATA   = 0x{cli.raw_data():016X}")

                    elif cmd == "mode" and len(p) == 3:
                        cli.set_dac_mode(parse_int_auto(p[1]), p[2])

                    elif cmd == "cfg_dac" and len(p) in (6, 7, 8):
                        dac = parse_int_auto(p[1])
                        freq_hz = float(p[2])
                        offset = parse_int_auto(p[3])
                        amp_sin = parse_int_auto(p[4])
                        amp_noise = parse_int_auto(p[5])
                        amp_fc = parse_int_auto(p[6]) if len(p) >= 7 else None
                        mode = p[7] if len(p) == 8 else None

                        cli.cfg_dac(dac, freq_hz, offset, amp_sin, amp_noise, amp_fc, mode)

                    elif cmd == "freq" and len(p) == 3:
                        cli.set_freq_hz(parse_int_auto(p[1]), float(p[2]))

                    elif cmd == "offset" and len(p) == 3:
                        cli.set_offset(parse_int_auto(p[1]), parse_int_auto(p[2]))

                    elif cmd == "amp_sin" and len(p) == 3:
                        cli.set_amp_sin(parse_int_auto(p[1]), parse_int_auto(p[2]))

                    elif cmd == "amp_noise" and len(p) == 3:
                        cli.set_amp_noise(parse_int_auto(p[1]), parse_int_auto(p[2]))

                    elif cmd == "amp_fc" and len(p) == 3:
                        cli.set_amp_fc(parse_int_auto(p[1]), parse_int_auto(p[2]))

                    elif cmd == "corr" and len(p) == 2:
                        cli.set_correction_s(parse_int_auto(p[1]) != 0)

                    elif cmd == "amp_sweep" and len(p) in (6, 7, 8, 9):
                        dac = parse_int_auto(p[1])
                        amp_start = parse_int_auto(p[2])
                        amp_stop = parse_int_auto(p[3])
                        amp_step = parse_int_auto(p[4])
                        dwell_s = float(p[5])

                        poll_ms = float(p[6]) if len(p) >= 7 else 1.0
                        filename = p[7] if len(p) >= 8 else None
                        set_mode = bool(parse_int_auto(p[8])) if len(p) == 9 else True

                        cli.sweep_amp_sin(
                            dac=dac,
                            amp_start=amp_start,
                            amp_stop=amp_stop,
                            amp_step=amp_step,
                            dwell_s=dwell_s,
                            poll_ms=poll_ms,
                            filename=filename,
                            set_mode=set_mode,
                        )

                    elif cmd == "sweep_pkl" and len(p) in (6, 7, 8):
                        dac = parse_int_auto(p[1])
                        amp_start = parse_int_auto(p[2])
                        amp_stop = parse_int_auto(p[3])
                        amp_step = parse_int_auto(p[4])
                        dwell_s = float(p[5])

                        filename = p[6] if len(p) >= 7 else "/root/sweep_amp.pkl"
                        poll_ms = float(p[7]) if len(p) == 8 else 1.0

                        cli.sweep_amp_sin_pkl(
                            dac=dac,
                            amp_start=amp_start,
                            amp_stop=amp_stop,
                            amp_step=amp_step,
                            dwell_s=dwell_s,
                            filename=filename,
                            poll_ms=poll_ms,
                            set_mode=True,
                        )

                    elif cmd == "coeffs" and len(p) >= 6:
                        filt = p[1].lower()
                        coeffs = [parse_int_auto(x) for x in p[2:]]
                        cli.upload_coefficients_from_list(filt, coeffs, start_fsm=False)

                    elif cmd == "coeff_file" and len(p) in (3, 4):
                        filt = p[1].lower()
                        filename = p[2]
                        column = p[3] if len(p) == 4 else None
                        cli.upload_coefficients_from_file(filt, filename, column=column, start_fsm=False)

                    elif cmd == "coeff_txt" and len(p) == 3:
                        filt = p[1].lower()
                        filename = p[2]
                        coeffs = read_coeffs_txt(filename)
                        cli.upload_coefficients_from_list(filt, coeffs, start_fsm=False)

                    elif cmd == "coeff_csv" and len(p) in (3, 4):
                        filt = p[1].lower()
                        filename = p[2]
                        column = p[3] if len(p) == 4 else None
                        coeffs = read_coeffs_csv(filename, column=column)
                        cli.upload_coefficients_from_list(filt, coeffs, start_fsm=False)

                    elif cmd == "coeff_pkl" and len(p) == 3:
                        filt = p[1].lower()
                        filename = p[2]
                        coeffs = read_coeffs_pkl(filename, filt=filt)
                        cli.upload_coefficients_from_list(filt, coeffs, start_fsm=False)

                    elif cmd == "coeff_pkl_all" and len(p) == 2:
                        filename = p[1]
                        cli.upload_coefficients_pkl_all(filename, start_fsm=False)

                    elif cmd == "clear_pending":
                        cli.clear_pending()

                    elif cmd == "clear_overrun":
                        cli.clear_overrun()

                    elif cmd == "adapt_on" and len(p) in (1, 2, 3):
                        dac = parse_int_auto(p[1]) if len(p) >= 2 else 1
                        amp_fc = parse_int_auto(p[2]) if len(p) == 3 else None
                        cli.adapt_on(dac=dac, amp_fc=amp_fc)

                    elif cmd == "adapt_log" and len(p) in (3, 4, 5, 6, 7):
                        duration_s = float(p[1])
                        filename = p[2]
                        poll_ms = float(p[3]) if len(p) >= 4 else 1.0
                        dac = parse_int_auto(p[4]) if len(p) >= 5 else 1
                        amp_fc = parse_int_auto(p[5]) if len(p) >= 6 else None
                        pre_s = float(p[6]) if len(p) == 7 else 0.0

                        cli.adapt_log(
                            duration_s=duration_s,
                            filename=filename,
                            poll_ms=poll_ms,
                            dac=dac,
                            amp_fc=amp_fc,
                            pre_s=pre_s,
                            stop_after=False,
                            dac_off_after=False,
                        )

                    elif cmd == "adapt_off" and len(p) in (1, 2, 3):
                        dac = parse_int_auto(p[1]) if len(p) >= 2 else 1
                        dac_off = bool(parse_int_auto(p[2])) if len(p) == 3 else True
                        cli.adapt_off(dac=dac, dac_off=dac_off)

                    elif cmd == "log_pkl" and len(p) in (3, 4):
                        filename = p[1]
                        duration_s = float(p[2])
                        poll_ms = float(p[3]) if len(p) == 4 else 1.0
                        cli.log_pkl(filename, duration_s, poll_ms)

                    elif cmd == "log_csv" and len(p) in (3, 4):
                        filename = p[1]
                        duration_s = float(p[2])
                        poll_ms = float(p[3]) if len(p) == 4 else 1.0
                        cli.log_csv(filename, duration_s, poll_ms)

                    elif cmd == "mitio" and len(p) == 2:
                        # Aceita hexadecimal ou decimal inteiro:
                        #   mitio 0x0200
                        #   mitio 512
                        cli.set_mitio_raw(parse_int_auto(p[1]))

                    elif cmd == "alpha" and len(p) == 2:
                        # Aceita valor real:
                        #   alpha 0.01
                        #   alpha 0.05
                        cli.set_mitio_alpha(float(p[1]))

                    else:
                        print("Comando inválido.")
                        print_help()

                except Exception as e:
                    print(f"Erro: {e}")

        finally:
            mm.close()

    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
