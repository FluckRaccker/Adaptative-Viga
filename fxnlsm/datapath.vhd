	library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use WORK.TYPES.ALL;
use work.basic_op.all;

entity datapath is
  port (
    clock : in std_logic;
    reset : in std_logic; -- ativo em '1'

    ------------------------------------------------------------------
    -- UART para leitura/debug dos MEMS
    ------------------------------------------------------------------
    serial_in  : in  std_logic;
    serial_out : out std_logic;

    ------------------------------------------------------------------
    -- I2C dos dois sensores
    ------------------------------------------------------------------
    i2c_scl_mems1 : out   std_logic;
    i2c_sda_mems1 : inout std_logic;

    i2c_scl_mems2 : out   std_logic;
    i2c_sda_mems2 : inout std_logic;

    ------------------------------------------------------------------
    -- I2C dos dois DACs
    ------------------------------------------------------------------
    i2c_scl_dac1 : inout std_logic;
    i2c_sda_dac1 : inout std_logic;

    i2c_scl_dac2 : inout std_logic;
    i2c_sda_dac2 : inout std_logic;

    ------------------------------------------------------------------
    -- Controle vindo da FSM
    ------------------------------------------------------------------
    coeff_in       : in std_logic_vector(4*coeff_width-1 downto 0);
    enable_fir     : in std_logic_vector(2 downto 0);
    mode_fir       : in std_logic;
    coeff_clear_fir : in std_logic_vector(1 downto 0);

    ------------------------------------------------------------------
    -- Configuração vinda do simple_cmd_storage
    ------------------------------------------------------------------
    load_config : in std_logic;

    offset    : in std_logic_vector(23 downto 0); -- [11:0] DAC1, [23:12] DAC2
    freq      : in std_logic_vector(63 downto 0); -- [31:0] DAC1, [63:32] DAC2

    amp_sin   : in std_logic_vector(13 downto 0); -- [6:0] DAC1, [13:7] DAC2
    amp_noise : in std_logic_vector(13 downto 0);
    amp_fc    : in std_logic_vector(13 downto 0);
	 
	 mitio : in std_logic_vector(15 downto 0);
	 
	 correction : in  std_logic;

    config_dac : in std_logic_vector(5 downto 0); -- [2:0] DAC1, [5:3] DAC2

    ------------------------------------------------------------------
    -- Saídas para controle/top/debug
    ------------------------------------------------------------------
    data_out : out std_logic_vector(data_width-1 downto 0);

    valid_fir    : out std_logic_vector(1 downto 0);
    busy_fir     : out std_logic_vector(1 downto 0);
    coeff_ok_fir : out std_logic_vector(1 downto 0);

    accel_z          : out std_logic_vector(31 downto 0);
    sample_valid_imu : out std_logic;
    who_ok           : out std_logic_vector(1 downto 0);

    dac_signal : out std_logic_vector(23 downto 0); -- DAC1 & DAC2

    dac_status_1 : out std_logic_vector(3 downto 0);
    dac_status_2 : out std_logic_vector(3 downto 0);
	 
	 coeff_count_dbg : out std_logic_vector(23 downto 0)
  );
end datapath;

architecture rtl of datapath is
  -- Number of taps is ajusted in fir_configs

  signal reset_n_i : std_logic;

  --------------------------------------------------------------------
  -- MEMS
  --------------------------------------------------------------------
  signal accel_z_s          : std_logic_vector(31 downto 0);
  signal sample_valid_imu_s : std_logic;
  signal who_ok_s           : std_logic_vector(1 downto 0);

  signal xr : std_logic_vector(data_width-1 downto 0);
  signal xe : std_logic_vector(data_width-1 downto 0);

  --------------------------------------------------------------------
  -- Filtros
  --------------------------------------------------------------------
  signal fc  : std_logic_vector(data_width-1 downto 0) := (others => '0');
  signal yf  : signed(data_width-1 downto 0) := (others => '0');
  signal xc  : std_logic_vector(data_width-1 downto 0) := (others => '0');
  signal xfx : signed(data_width-1 downto 0) := (others => '0');

  signal valid_wf : std_logic;
  signal valid_ws : std_logic;
  signal busy_wf  : std_logic;
  signal busy_ws  : std_logic;

  signal coeff_ok_wf : std_logic;
  signal coeff_ok_ws : std_logic;

  signal reset_wf : std_logic;
  signal reset_ws : std_logic;
  
  signal coeff_count_wf : std_logic_vector(11 downto 0);
  signal coeff_count_ws : std_logic_vector(11 downto 0);

  --------------------------------------------------------------------
  -- DAC
  --------------------------------------------------------------------
  signal dac_signal_1 : std_logic_vector(11 downto 0);
  signal dac_signal_2 : std_logic_vector(11 downto 0);

  signal fc_to_dac    : signed(data_width-1 downto 0) := (others => '0');
  signal fc_valid_dac : std_logic := '0';
  signal en_wc_d1     : std_logic := '0';

  --------------------------------------------------------------------
  -- Componentes
  --------------------------------------------------------------------
  component feedback_fir is
    generic (
      FIR_TAPS : positive := taps
    );
    port (
      clock     : in  std_logic;
      reset     : in  std_logic;
      data_in   : in  signed(data_width-1 downto 0);
      coeff_in  : in  std_logic_vector(4*coeff_width-1 downto 0);
      en        : in  std_logic;
      mode      : in  std_logic;
      data_out  : out signed(data_width-1 downto 0);
      valid_out : out std_logic;
      busy      : out std_logic;
      coeff_ok  : out std_logic;
		coeff_count : out std_logic_vector(11 downto 0)
    );
  end component;

  component feedsecondary_fir is
    generic (
      FIR_TAPS : positive := taps
    );
    port (
      clock     : in  std_logic;
      reset     : in  std_logic;
      data_in   : in  signed(data_width-1 downto 0);
      coeff_in  : in  std_logic_vector(4*coeff_width-1 downto 0);
      en        : in  std_logic;
      mode      : in  std_logic;
      data_out  : out signed(data_width-1 downto 0);
      valid_out : out std_logic;
      busy      : out std_logic;
      coeff_ok  : out std_logic;
		coeff_count : out std_logic_vector(11 downto 0)
    );
  end component;

  component adap_fxnlms is
    generic (
		ORDER : positive := taps_adapt
    );
    port (
      xc     : in  std_logic_vector(15 downto 0);
      xfx    : in  std_logic_vector(15 downto 0);
      xe     : in  std_logic_vector(15 downto 0);
      clock  : in  std_logic;
      reset  : in  std_logic;
      en     : in  std_logic;
		mitio_cfg : in std_logic_vector(15 downto 0);
	   correction_cfg : in  std_logic;
      fc     : out std_logic_vector(15 downto 0)
    );
  end component;

  component mems_sensor_read is
    port (
      clk              : in    std_logic;
      serial_in        : in    std_logic;
      rstn             : in    std_logic;
      serial_out       : out   std_logic;

      i2c_scl_1        : out   std_logic;
      i2c_sda_1        : inout std_logic;
      i2c_scl_2        : out   std_logic;
      i2c_sda_2        : inout std_logic;

      accel_z          : out   std_logic_vector(31 downto 0);
      sample_valid_imu : out   std_logic;
      who_ok           : out   std_logic_vector(1 downto 0)
    );
  end component;

  component dac_module is
    generic (
      CLK_HZ    : integer := 50000000;
      SAMPLE_HZ : integer := 2000;
      FC_SHIFT  : integer := 4
    );
    port (
      clk        : in    std_logic;
      reset_n    : in    std_logic;
      load_cfg   : in    std_logic;

      amp_sin    : in    std_logic_vector(6 downto 0);
      amp_noise  : in    std_logic_vector(6 downto 0);
      amp_fc     : in    std_logic_vector(6 downto 0);

      offset     : in    std_logic_vector(11 downto 0);
      freq       : in    std_logic_vector(31 downto 0);

      fc_in      : in    signed(15 downto 0);
      fc_valid   : in    std_logic;

      i2c_scl    : inout std_logic;
      i2c_sda    : inout std_logic;

      dac_status : out   std_logic_vector(3 downto 0);
      dac_signal : out   std_logic_vector(11 downto 0);

      config     : in    std_logic_vector(2 downto 0)
    );
  end component;
  
  component dac_module_2 is
    generic (
      CLK_HZ    : integer := 50000000;
      SAMPLE_HZ : integer := 2000;
      FC_SHIFT  : integer := 4
    );
    port (
      clk        : in    std_logic;
      reset_n    : in    std_logic;
      load_cfg   : in    std_logic;

      amp_sin    : in    std_logic_vector(6 downto 0);
      amp_noise  : in    std_logic_vector(6 downto 0);
      amp_fc     : in    std_logic_vector(6 downto 0);

      offset     : in    std_logic_vector(11 downto 0);
      freq       : in    std_logic_vector(31 downto 0);

      fc_in      : in    signed(15 downto 0);
      fc_valid   : in    std_logic;

      i2c_scl    : inout std_logic;
      i2c_sda    : inout std_logic;

      dac_status : out   std_logic_vector(3 downto 0);
      dac_signal : out   std_logic_vector(11 downto 0);

      config     : in    std_logic_vector(2 downto 0)
    );
  end component;

begin

  reset_n_i <= not reset;

  --------------------------------------------------------------------
  -- MEMS
  --------------------------------------------------------------------
  mems_inst : mems_sensor_read
    port map (
      clk              => clock,
      serial_in        => serial_in,
      rstn             => reset_n_i,
      serial_out       => serial_out,

      i2c_scl_1        => i2c_scl_mems1,
      i2c_sda_1        => i2c_sda_mems1,
      i2c_scl_2        => i2c_scl_mems2,
      i2c_sda_2        => i2c_sda_mems2,

      accel_z          => accel_z_s,
      sample_valid_imu => sample_valid_imu_s,
      who_ok           => who_ok_s
    );

  --------------------------------------------------------------------
  -- Sensor 1 = referência xr
  -- Sensor 2 = erro xe
  --------------------------------------------------------------------
  xr <= accel_z_s(31 downto 16);
  xe <= accel_z_s(15 downto 0);

  accel_z          <= accel_z_s;
  sample_valid_imu <= sample_valid_imu_s;
  who_ok           <= who_ok_s;

  --------------------------------------------------------------------
  -- xc(n) = xr(n) - yf(n)
  --------------------------------------------------------------------
  xc <= std_logic_vector(signed(xr) - yf);

  --------------------------------------------------------------------
  -- Reset individual dos FIRs para recarregar coeficientes
  -- Solução rápida: usa reset do FIR.
  -- Depois, o ideal é trocar por coeff_clear interno no FIR.
  --------------------------------------------------------------------
  reset_wf <= reset or coeff_clear_fir(0);
  reset_ws <= reset or coeff_clear_fir(1);

  --------------------------------------------------------------------
  -- wf: entrada fc, saída yf
  --------------------------------------------------------------------
  wf_inst : feedback_fir
    generic map (
      FIR_TAPS => taps
    )
    port map (
      clock     => clock,
      reset     => reset_wf,
      data_in   => signed(fc),
      coeff_in  => coeff_in,
      en        => enable_fir(0),
      mode      => mode_fir,
      data_out  => yf,
      valid_out => valid_wf,
      busy      => busy_wf,
      coeff_ok  => coeff_ok_wf,
      coeff_count => coeff_count_wf
    );

  --------------------------------------------------------------------
  -- ws: entrada xc, saída xfx
  --------------------------------------------------------------------
  ws_inst : feedsecondary_fir
    generic map (
      FIR_TAPS => taps
    )
    port map (
      clock     => clock,
      reset     => reset_ws,
      data_in   => signed(xc),
      coeff_in  => coeff_in,
      en        => enable_fir(1),
      mode      => mode_fir,
      data_out  => xfx,
      valid_out => valid_ws,
      busy      => busy_ws,
      coeff_ok  => coeff_ok_ws,
      coeff_count => coeff_count_ws
    );

  --------------------------------------------------------------------
  -- wc: filtro adaptativo
  --------------------------------------------------------------------
  wc_inst : adap_fxnlms
    generic map (
		ORDER => taps_adapt
    )
    port map (
      xc     => xc,
      xfx    => std_logic_vector(xfx),
      xe     => xe,
      clock  => clock,
      reset  => reset,
      en     => enable_fir(2),
      mitio_cfg => mitio,
		correction_cfg => correction,
      fc     => fc
    );

  --------------------------------------------------------------------
  -- Saída principal do datapath
  --------------------------------------------------------------------
  data_out <= fc;

  valid_fir(0) <= valid_wf;
  valid_fir(1) <= valid_ws;

  busy_fir(0) <= busy_wf;
  busy_fir(1) <= busy_ws;

  coeff_ok_fir(0) <= coeff_ok_wf;
  coeff_ok_fir(1) <= coeff_ok_ws;
  
  coeff_count_dbg <= coeff_count_wf & coeff_count_ws;

  --------------------------------------------------------------------
  -- Gera fc_valid para os DACs
  -- Usa um atraso de 1 clock depois do enable do wc.
  --------------------------------------------------------------------
  process(clock, reset)
  begin
    if reset = '1' then
      en_wc_d1     <= '0';
      fc_valid_dac <= '0';
      fc_to_dac    <= (others => '0');

    elsif rising_edge(clock) then
      en_wc_d1     <= enable_fir(2);
      fc_valid_dac <= en_wc_d1;

      if en_wc_d1 = '1' then
        fc_to_dac <= signed(fc);
      end if;
    end if;
  end process;

  --------------------------------------------------------------------
  -- DAC 1
  --------------------------------------------------------------------
  dac1_inst : dac_module
    generic map (
      CLK_HZ    => 50000000,
      SAMPLE_HZ => 2000,
      FC_SHIFT  => 4
    )
    port map (
      clk        => clock,
      reset_n    => reset_n_i,
      load_cfg   => load_config,

      amp_sin    => amp_sin(6 downto 0),
      amp_noise  => amp_noise(6 downto 0),
      amp_fc     => amp_fc(6 downto 0),

      offset     => offset(11 downto 0),
      freq       => freq(31 downto 0),

      fc_in      => fc_to_dac,
      fc_valid   => fc_valid_dac,

      i2c_scl    => i2c_scl_dac1,
      i2c_sda    => i2c_sda_dac1,

      dac_status => dac_status_1,
      dac_signal => dac_signal_1,

      config     => config_dac(2 downto 0)
    );

  --------------------------------------------------------------------
  -- DAC 2
  --------------------------------------------------------------------
  dac2_inst : dac_module_2
    generic map (
      CLK_HZ    => 50000000,
      SAMPLE_HZ => 2000,
      FC_SHIFT  => 4
    )
    port map (
      clk        => clock,
      reset_n    => reset_n_i,
      load_cfg   => load_config,

      amp_sin    => amp_sin(13 downto 7),
      amp_noise  => amp_noise(13 downto 7),
      amp_fc     => amp_fc(13 downto 7),

      offset     => offset(23 downto 12),
      freq       => freq(63 downto 32),

      fc_in      => fc_to_dac,
      fc_valid   => fc_valid_dac,

      i2c_scl    => i2c_scl_dac2,
      i2c_sda    => i2c_sda_dac2,

      dac_status => dac_status_2,
      dac_signal => dac_signal_2,

      config     => config_dac(5 downto 3)
    );

  dac_signal <= dac_signal_1 & dac_signal_2;

end rtl;