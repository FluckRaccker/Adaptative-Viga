library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use WORK.TYPES.ALL;
use WORK.BASIC_OP.ALL;

entity viga_circuit is
  port (
    clock : in std_logic;
    reset : in std_logic; -- ativo em '1'


    ------------------------------------------------------------------
    -- UART
    ------------------------------------------------------------------
    serial_in  : in  std_logic;
    serial_out : out std_logic;

    ------------------------------------------------------------------
    -- I2C MEMS
    ------------------------------------------------------------------
    i2c_scl_mems1 : out   std_logic;
    i2c_sda_mems1 : inout std_logic;

    i2c_scl_mems2 : out   std_logic;
    i2c_sda_mems2 : inout std_logic;

    ------------------------------------------------------------------
    -- I2C DACs
    ------------------------------------------------------------------s
    i2c_scl_dac1 : inout std_logic;
    i2c_sda_dac1 : inout std_logic;

    i2c_scl_dac2 : inout std_logic;
    i2c_sda_dac2 : inout std_logic;

    ------------------------------------------------------------------
    -- Interface MMIO / Python
    ------------------------------------------------------------------
    in_a       : in  std_logic_vector(63 downto 0);
    in_b       : in  std_logic_vector(63 downto 0);
    out_export : out std_logic_vector(63 downto 0);
    out_data   : out std_logic_vector(63 downto 0);

    ------------------------------------------------------------------
    -- Debug
    ------------------------------------------------------------------
    LEDR     : out std_logic_vector(7 downto 0)
  );
end viga_circuit;

architecture rtl of viga_circuit is

  signal reset_n_i : std_logic;

  --------------------------------------------------------------------
  -- Simple command storage
  --------------------------------------------------------------------
  signal load_config_s : std_logic;
  signal control_enable_s : std_logic;

  signal coeff_tick_s    : std_logic_vector(1 downto 0);
  signal coeff_pending_s : std_logic_vector(1 downto 0);
  signal coeff_ready_s   : std_logic_vector(1 downto 0);

  signal coeffs_s   : std_logic_vector(63 downto 0);
  signal coeff_in_s : std_logic_vector(4*coeff_width-1 downto 0);

  signal offset_s     : std_logic_vector(23 downto 0);
  signal freq_s       : std_logic_vector(63 downto 0);
  signal amp_noise_s  : std_logic_vector(13 downto 0);
  signal amp_sin_s    : std_logic_vector(13 downto 0);
  signal amp_fc_s     : std_logic_vector(13 downto 0);
  
  signal mitio_s : std_logic_vector(15 downto 0);
  
  signal correction_s : std_logic;
  
  signal config_dac_s : std_logic_vector(5 downto 0);

  --------------------------------------------------------------------
  -- Control <-> Datapath
  --------------------------------------------------------------------
  signal valid_fir_s    : std_logic_vector(1 downto 0);
  signal busy_fir_s     : std_logic_vector(1 downto 0);
  signal coeff_ok_fir_s : std_logic_vector(1 downto 0);

  signal enable_fir_s      : std_logic_vector(2 downto 0);
  signal mode_fir_s        : std_logic;
  signal coeff_clear_fir_s : std_logic_vector(1 downto 0);

  signal configured_s : std_logic;
  signal running_s    : std_logic;

  --------------------------------------------------------------------
  -- Datapath -> Simple command
  --------------------------------------------------------------------
  signal accel_z_s          : std_logic_vector(31 downto 0);
  signal sample_valid_imu_s : std_logic;
  signal who_ok_s           : std_logic_vector(1 downto 0);

  signal dac_signal_s : std_logic_vector(23 downto 0);

  signal dac_status_1_s : std_logic_vector(3 downto 0);
  signal dac_status_2_s : std_logic_vector(3 downto 0);
  
  signal coeff_count_dbg_s : std_logic_vector(23 downto 0);
  
  

  --------------------------------------------------------------------
  -- Components
  --------------------------------------------------------------------
  component blink is
	port(
	clk          : in  std_logic;
	led          : out std_logic
	);
  end component;
  
  component simple_cmd_storage is
    port (
      clk          : in  std_logic;
      reset_n      : in  std_logic;

      sample_valid : in  std_logic;
		
		control_enable : out std_logic;

      in_a         : in  std_logic_vector(63 downto 0);
      in_b         : in  std_logic_vector(63 downto 0);

      out_export   : out std_logic_vector(63 downto 0);
      out_data     : out std_logic_vector(63 downto 0);

      load_config  : out std_logic;

      coeff_tick    : out std_logic_vector(1 downto 0);
      coeff_pending : out std_logic_vector(1 downto 0);
      coeff_ready   : in  std_logic_vector(1 downto 0);

      offset    : out std_logic_vector(23 downto 0);
      freq      : out std_logic_vector(63 downto 0);

      coeffs    : out std_logic_vector(63 downto 0);

      amp_noise : out std_logic_vector(13 downto 0);
      amp_sin   : out std_logic_vector(13 downto 0);
      amp_fc    : out std_logic_vector(13 downto 0);
		
		mitio : out std_logic_vector(15 downto 0);
		
		correction  : out std_logic;

      config_dac : out std_logic_vector(5 downto 0);

      accel_z    : in std_logic_vector(31 downto 0);
      dac_signal : in std_logic_vector(23 downto 0);
		coeff_count_dbg : in std_logic_vector(23 downto 0)
    );
  end component;

  component control is
    port (
      clock : in std_logic;
      reset : in std_logic;
      control_enable : in std_logic;

      sample_tick : in std_logic;

      coeff_tick    : in std_logic_vector(1 downto 0);
      coeff_pending : in std_logic_vector(1 downto 0);

      coeff_ok_fir : in std_logic_vector(1 downto 0);
      busy_fir     : in std_logic_vector(1 downto 0);
      valid_fir    : in std_logic_vector(1 downto 0);

      enable_fir : out std_logic_vector(2 downto 0);
      mode_fir   : out std_logic;

      coeff_ready    : out std_logic_vector(1 downto 0);
      coeff_clear_fir : out std_logic_vector(1 downto 0);

      configured : out std_logic;
      running    : out std_logic
    );
  end component;

  component datapath is
    port (
      clock : in std_logic;
      reset : in std_logic;

      serial_in  : in  std_logic;
      serial_out : out std_logic;

      i2c_scl_mems1 : out   std_logic;
      i2c_sda_mems1 : inout std_logic;

      i2c_scl_mems2 : out   std_logic;
      i2c_sda_mems2 : inout std_logic;

      i2c_scl_dac1 : inout std_logic;
      i2c_sda_dac1 : inout std_logic;

      i2c_scl_dac2 : inout std_logic;
      i2c_sda_dac2 : inout std_logic;

      coeff_in        : in std_logic_vector(4*coeff_width-1 downto 0);
      enable_fir      : in std_logic_vector(2 downto 0);
      mode_fir        : in std_logic;
      coeff_clear_fir : in std_logic_vector(1 downto 0);

      load_config : in std_logic;

      offset    : in std_logic_vector(23 downto 0);
      freq      : in std_logic_vector(63 downto 0);

      amp_sin   : in std_logic_vector(13 downto 0);
      amp_noise : in std_logic_vector(13 downto 0);
      amp_fc    : in std_logic_vector(13 downto 0);
		
		mitio : in std_logic_vector(15 downto 0);
		
		correction : in  std_logic;

      config_dac : in std_logic_vector(5 downto 0);

      data_out : out std_logic_vector(data_width-1 downto 0);

      valid_fir    : out std_logic_vector(1 downto 0);
      busy_fir     : out std_logic_vector(1 downto 0);
      coeff_ok_fir : out std_logic_vector(1 downto 0);

      accel_z          : out std_logic_vector(31 downto 0);
      sample_valid_imu : out std_logic;
      who_ok           : out std_logic_vector(1 downto 0);

      dac_signal : out std_logic_vector(23 downto 0);

      dac_status_1 : out std_logic_vector(3 downto 0);
      dac_status_2 : out std_logic_vector(3 downto 0);
		
		coeff_count_dbg : out std_logic_vector(23 downto 0)
    );
  end component;

begin

  reset_n_i <= not reset;

  --------------------------------------------------------------------
  -- Ajuste do tamanho do pacote de coeficientes
  -- Se coeff_width = 16, usa os 64 bits completos.
  --------------------------------------------------------------------
  coeff_in_s <= coeffs_s(4*coeff_width-1 downto 0);

  --------------------------------------------------------------------
  -- Simple command storage
  --------------------------------------------------------------------
  cmd_inst : simple_cmd_storage
    port map (
      clk          => clock,
      reset_n      => reset_n_i,

      sample_valid => sample_valid_imu_s,
		
		control_enable => control_enable_s,

      in_a         => in_a,
      in_b         => in_b,

      out_export   => out_export,
      out_data     => out_data,

      load_config  => load_config_s,

      coeff_tick    => coeff_tick_s,
      coeff_pending => coeff_pending_s,
      coeff_ready   => coeff_ready_s,

      offset    => offset_s,
      freq      => freq_s,

      coeffs    => coeffs_s,

      amp_noise => amp_noise_s,
      amp_sin   => amp_sin_s,
      amp_fc    => amp_fc_s,
		mitio => mitio_s,
		
		correction => correction_s,

      config_dac => config_dac_s,

      accel_z    => accel_z_s,
      dac_signal => dac_signal_s,
		
		coeff_count_dbg => coeff_count_dbg_s
    );

  --------------------------------------------------------------------
  -- Controller
  --------------------------------------------------------------------
  control_inst : control
    port map (
      clock => clock,
      reset => reset,
      control_enable => control_enable_s,

      sample_tick => sample_valid_imu_s,

      coeff_tick    => coeff_tick_s,
      coeff_pending => coeff_pending_s,

      coeff_ok_fir => coeff_ok_fir_s,
      busy_fir     => busy_fir_s,
      valid_fir    => valid_fir_s,

      enable_fir => enable_fir_s,
      mode_fir   => mode_fir_s,

      coeff_ready     => coeff_ready_s,
      coeff_clear_fir => coeff_clear_fir_s,

      configured => configured_s,
      running    => running_s
    );

  --------------------------------------------------------------------
  -- Datapath
  --------------------------------------------------------------------
  datapath_inst : datapath
    port map (
      clock => clock,
      reset => reset,

      serial_in  => serial_in,
      serial_out => serial_out,

      i2c_scl_mems1 => i2c_scl_mems1,
      i2c_sda_mems1 => i2c_sda_mems1,

      i2c_scl_mems2 => i2c_scl_mems2,
      i2c_sda_mems2 => i2c_sda_mems2,

      i2c_scl_dac1 => i2c_scl_dac1,
      i2c_sda_dac1 => i2c_sda_dac1,

      i2c_scl_dac2 => i2c_scl_dac2,
      i2c_sda_dac2 => i2c_sda_dac2,

      coeff_in        => coeff_in_s,
      enable_fir      => enable_fir_s,
      mode_fir        => mode_fir_s,
      coeff_clear_fir => coeff_clear_fir_s,

      load_config => load_config_s,

      offset    => offset_s,
      freq      => freq_s,

      amp_sin   => amp_sin_s,
      amp_noise => amp_noise_s,
      amp_fc    => amp_fc_s,
		
		correction => correction_s,
		mitio => mitio_s,

      config_dac => config_dac_s,

      data_out => open,

      valid_fir    => valid_fir_s,
      busy_fir     => busy_fir_s,
      coeff_ok_fir => coeff_ok_fir_s,

      accel_z          => accel_z_s,
      sample_valid_imu => sample_valid_imu_s,
      who_ok           => who_ok_s,

      dac_signal => dac_signal_s,

      dac_status_1 => dac_status_1_s,
      dac_status_2 => dac_status_2_s,
		
		coeff_count_dbg => coeff_count_dbg_s
    );

  --------------------------------------------------------------------
  -- LEDs
  --------------------------------------------------------------------
  
--  blink_1 : blink
--	port map(
--		clk => clock,
--		led => LEDR(0)
--	);
--  
  
  LEDR(1 downto 0) <= who_ok_s;
  LEDR(3 downto 2) <= dac_status_1_s(3) & dac_status_2_s(3);
--  LEDR(6 downto 7) <= dac_status_1_s(2) & dac_status_2_s(2);
  LEDR(4) <= coeff_ok_fir_s(0); -- wf carregado
  LEDR(5) <= coeff_ok_fir_s(1); -- ws carregado
  LEDR(6) <= configured_s;
  LEDR(7) <= running_s;

end rtl;