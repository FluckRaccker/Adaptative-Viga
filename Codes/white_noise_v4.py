#!/usr/bin/env python3
import os
import mmap
import struct
import time
import csv
import pandas as pd

# ===============================
# Endereços
# ===============================
BRIDGE = 0xC0000000
BRIDGE_SPAN = 0x100

IN_A       = 0x00
IN_B       = 0x08
# OUT_EXPORT = 0x10
OUT_EXPORT = 0x18

# ===============================
# Clock do FPGA / NCO
# ===============================
CLK_HZ = 50_000_000
PHASE_BITS = 32

# ===============================
# Máscaras do simple_cmd_storage
# ===============================
CMD_FREQ       = 1 << 0
CMD_OFFSET     = 1 << 1
CMD_AMP_SIN    = 1 << 2
CMD_AMP_NOISE  = 1 << 3

CMD_ALL_DAC = CMD_FREQ | CMD_OFFSET | CMD_AMP_SIN | CMD_AMP_NOISE

# ===============================
# Acesso básico
# ===============================
def write_u64(mm, offset, value):
    mm[offset:offset + 8] = struct.pack("<Q", value & 0xFFFFFFFFFFFFFFFF)

def read_u64(mm, offset):
    return struct.unpack("<Q", mm[offset:offset + 8])[0]

def parse_int_auto(text):
    return int(text, 0)

def pulse_cmd_mask(mm, cmd_mask, delay_s=0.001):
    write_u64(mm, IN_B, cmd_mask & 0xF)
    time.sleep(delay_s)
    write_u64(mm, IN_B, 0)
    time.sleep(delay_s)

# ===============================
# Conversões
# ===============================
def hz_to_phase_inc(freq_hz, clk_hz=CLK_HZ, phase_bits=PHASE_BITS):
    return int(round((freq_hz * (1 << phase_bits)) / clk_hz))

def phase_inc_to_hz(phase_inc, clk_hz=CLK_HZ, phase_bits=PHASE_BITS):
    return (phase_inc * clk_hz) / (1 << phase_bits)

def to_signed16(x):
    x &= 0xFFFF
    return x if x < 0x8000 else x - 0x10000

def to_signed(x, bits):
    if x & (1 << (bits - 1)):
        return x - (1 << bits)
    return x
# ===============================
# Empacotamento do IN_A
# ===============================
def pack_dac_cfg_from_hz(freq_hz, offset, amp_sin, amp_noise):
    phase_inc = hz_to_phase_inc(freq_hz)

    payload = 0
    payload |= (phase_inc  & 0xFFFFFFFF)
    payload |= (offset     & 0xFFF) << 32
    payload |= (amp_sin    & 0x7F)  << 44
    payload |= (amp_noise  & 0x7F)  << 51

    return payload, phase_inc

def pack_dac_cfg_from_phase(phase_inc, offset, amp_sin, amp_noise):
    payload = 0
    payload |= (phase_inc  & 0xFFFFFFFF)
    payload |= (offset     & 0xFFF) << 32
    payload |= (amp_sin    & 0x7F)  << 44
    payload |= (amp_noise  & 0x7F)  << 51
    return payload

# ===============================
# Desempacotamento dos sensores
# accel_z <= imu_accel_z_1 & imu_accel_z_2
# ===============================

def unpack_out_export(raw):
    packed = raw & 0xFFFFFFFFFFFFFFFF

    dac_signal_u12 = (packed >> 32) & 0xFFF

    accel_z = packed & 0xFFFFFFFF
    accel_1_u16 = (accel_z >> 16) & 0xFFFF
    accel_2_u16 = accel_z & 0xFFFF

    dac_signal = dac_signal_u12
    accel_1 = to_signed(accel_1_u16, 16)
    accel_2 = to_signed(accel_2_u16, 16)

    return dac_signal, accel_1, accel_2, packed

# ===============================
# Comandos de alto nível
# ===============================
class BridgeCLI:
    def __init__(self, mm):
        self.mm = mm

    def cfg_dac(self, freq_hz, offset, amp_sin, amp_noise):
        payload, phase_inc = pack_dac_cfg_from_hz(freq_hz, offset, amp_sin, amp_noise)
        write_u64(self.mm, IN_A, payload)
        pulse_cmd_mask(self.mm, CMD_ALL_DAC)

        print("DAC configurado:")
        print(f"  freq_hz   = {freq_hz}")
        print(f"  phase_inc = {phase_inc} (0x{phase_inc:08X})")
        print(f"  offset    = {offset}")
        print(f"  amp_sin   = {amp_sin}")
        print(f"  amp_noise = {amp_noise}")

    def cfg_phase(self, phase_inc, offset, amp_sin, amp_noise):
        payload = pack_dac_cfg_from_phase(phase_inc, offset, amp_sin, amp_noise)
        write_u64(self.mm, IN_A, payload)
        pulse_cmd_mask(self.mm, CMD_ALL_DAC)

        print("DAC configurado:")
        print(f"  phase_inc = {phase_inc} (0x{phase_inc:08X})")
        print(f"  freq_hz   = {phase_inc_to_hz(phase_inc):.6f}")
        print(f"  offset    = {offset}")
        print(f"  amp_sin   = {amp_sin}")
        print(f"  amp_noise = {amp_noise}")

    def raw(self):
        raw = read_u64(self.mm, OUT_EXPORT)
        print(f"RAW = 0x{raw:016X} ({raw})")
        return raw

    def read(self):
        raw = read_u64(self.mm, OUT_EXPORT)
        dac_signal, accel_1, accel_2, packed = unpack_out_export(raw)

        print(f"dac_signal = {dac_signal}")
        print(f"accel_1 = {accel_1}")
        print(f"accel_2 = {accel_2}")
        print(f"packed  = 0x{packed:016X}")
        print(f"raw     = 0x{raw:016X}")

        return accel_1, accel_2, raw

    def log_csv(self, filename, duration_s, poll_ms=1.0):
        poll_s = poll_ms / 1000.0
        t0 = time.perf_counter()

        with open(filename, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "accel_1", "accel_2", "dac_signal", "packed_u64", "raw_u64"])

            while True:
                now = time.perf_counter() - t0
                if now >= duration_s:
                    break

                raw = read_u64(self.mm, OUT_EXPORT)
                dac_signal, accel_1, accel_2, packed = unpack_out_export(raw)

                w.writerow([
                    f"{now:.6f}",
                    accel_1,
                    accel_2,
                    dac_signal,
                    packed,
                    raw
                ])

                time.sleep(poll_s)

        print(f"CSV salvo em: {filename}")

    def log_pkl(self, filename, duration_s, poll_ms=1.0):
        """
        Salva os dados em formato binário do pandas (.pkl).
        Mais rápido que CSV porque não escreve linha por linha em texto.
        """

        poll_s = poll_ms / 1000.0

        t_list = []
        dac_signal_list = []
        accel_1_list = []
        accel_2_list = []
        packed_list = []
        raw_list = []

        t0 = time.perf_counter()

        while True:
            now = time.perf_counter() - t0
            if now >= duration_s:
                break

            raw = read_u64(self.mm, OUT_EXPORT)
            dac_signal, accel_1, accel_2, packed = unpack_out_export(raw)

            t_list.append(now)
            dac_signal_list.append(dac_signal)
            accel_1_list.append(accel_1)
            accel_2_list.append(accel_2)
            packed_list.append(packed)
            raw_list.append(raw)

            if poll_s > 0:
                time.sleep(poll_s)

        df = pd.DataFrame({
            "t_s": t_list,
            "accel_1": accel_1_list,
            "accel_2": accel_2_list,
            "packed_u64": packed_list,
            "raw_u64": raw_list,
            "dac_signal": dac_signal_list,
        })

        df.to_pickle(filename)

        print(f"Arquivo binário pandas salvo em: {filename}")
        print(f"Amostras salvas: {len(df)}")

    def log_txt(self, filename, duration_s, poll_ms=1.0):
        poll_s = poll_ms / 1000.0
        t0 = time.perf_counter()

        with open(filename, "w") as f:
            f.write("t_s, accel_1, accel_2, dac_signal, packed_u64, raw_u64\n")

            while True:
                now = time.perf_counter() - t0
                if now >= duration_s:
                    break

                raw = read_u64(self.mm, OUT_EXPORT)
                dac_signal, accel_1, accel_2, packed = unpack_out_export(raw)

                f.write(f"{now:.6f}, {accel_1}, {accel_2}, {dac_signal}, {packed}, {raw}\n")

                time.sleep(poll_s)

        print(f"TXT salvo em: {filename}")

def print_help():
    print("\nComandos disponíveis:")
    print("  help")
    print("  cfg_dac   <freq_hz> <offset> <amp_sin> <amp_noise>")
    print("  cfg_phase <phase_inc> <offset> <amp_sin> <amp_noise>")
    print("  read")
    print("  raw")
    print("  log_csv <arquivo.csv> <duracao_s> [poll_ms]")
    print("  log_txt <arquivo.txt> <duracao_s> [poll_ms]")
    print("  log_pkl <arquivo.pkl> <duracao_s> [poll_ms]")
    print("  q\n")

def main():
    fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)

    try:
        mm = mmap.mmap(
            fd,
            BRIDGE_SPAN,
            flags=mmap.MAP_SHARED,
            prot=mmap.PROT_READ | mmap.PROT_WRITE,
            offset=BRIDGE
        )

        try:
            cli = BridgeCLI(mm)
            print("=== Bridge CLI (simple_cmd mode / 2x16-bit accel) ===")
            print_help()

            write_u64(mm, IN_B, 0)

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

                    elif cmd == "cfg_dac" and len(p) == 5:
                        freq_hz   = float(p[1])
                        offset    = parse_int_auto(p[2])
                        amp_sin   = parse_int_auto(p[3])
                        amp_noise = parse_int_auto(p[4])
                        cli.cfg_dac(freq_hz, offset, amp_sin, amp_noise)

                    elif cmd == "cfg_phase" and len(p) == 5:
                        phase_inc = parse_int_auto(p[1])
                        offset    = parse_int_auto(p[2])
                        amp_sin   = parse_int_auto(p[3])
                        amp_noise = parse_int_auto(p[4])
                        cli.cfg_phase(phase_inc, offset, amp_sin, amp_noise)

                    elif cmd == "read":
                        cli.read()

                    elif cmd == "raw":
                        cli.raw()

                    elif cmd == "log_csv" and len(p) in (3, 4):
                        filename   = p[1]
                        duration_s = float(p[2])
                        poll_ms    = float(p[3]) if len(p) == 4 else 1.0
                        cli.log_csv(filename, duration_s, poll_ms)
                    
                    elif cmd == "log_pkl" and len(p) in (3, 4):
                        filename   = p[1]
                        duration_s = float(p[2])
                        poll_ms    = float(p[3]) if len(p) == 4 else 1.0
                        cli.log_pkl(filename, duration_s, poll_ms)

                    elif cmd == "log_txt" and len(p) in (3, 4):
                        filename   = p[1]
                        duration_s = float(p[2])
                        poll_ms    = float(p[3]) if len(p) == 4 else 1.0
                        cli.log_txt(filename, duration_s, poll_ms)

                    else:
                        print("Comando inválido.")
                        print_help()

                except ValueError as e:
                    print(f"Erro de valor: {e}")

        finally:
            mm.close()

    finally:
        os.close(fd)

if __name__ == "__main__":
    main()