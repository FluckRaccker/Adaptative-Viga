library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity mems_sensor_read is
  port (
    clk        : in    std_logic;
    serial_in  : in    std_logic;
    rstn       : in    std_logic;
    serial_out : out   std_logic;
    i2c_scl_1    : out   std_logic;
    i2c_sda_1    : inout std_logic;
	 i2c_scl_2    : out   std_logic;
    i2c_sda_2    : inout std_logic;
	 accel_z      : out  std_logic_vector(31 downto 0);
	 who_ok       : out   std_logic_vector(1 downto 0)
  );
end entity mems_sensor_read;

architecture str of mems_sensor_read is

  component UART_TX
    port (
      clk            : in  std_logic;
      rstn           : in  std_logic;
      send           : in  std_logic;
      send_data      : in  std_logic_vector(7 downto 0);
      serial_dat_out : out std_logic;
      ready          : out std_logic
    );
  end component;

  component UART_RX
    port (
      clk           : in  std_logic;
      rstn          : in  std_logic;
      serial_dat_in : in  std_logic;
      valid         : out std_logic;
      data          : out std_logic_vector(7 downto 0)
    );
  end component;

  component unpacker_16_8 is
    port (
      clk      : in  std_logic;
      rstn     : in  std_logic;
      in_valid : in  std_logic;
      data_in  : in  std_logic_vector(15 downto 0);
      tx_ready : in  std_logic;
      tx_valid : out std_logic;
      tx_data  : out std_logic_vector(7 downto 0);
      busy     : out std_logic
    );
  end component;

  component blink is
    port (
      clk : in  std_logic;
      led : out std_logic
    );
  end component;

  component i2c_mems is
    port (
      clk        : in    std_logic;
      rstn       : in    std_logic;
      i2c_scl    : out   std_logic;
      i2c_sda    : inout std_logic;
      data       : out   std_logic_vector(7 downto 0);
      status     : out   std_logic;
      who_ok     : out   std_logic;
      config_ok  : out   std_logic;
      reading_ok : out   std_logic;
      accel_z    : out   std_logic_vector(15 downto 0)
    );
  end component;

  signal ready_tx       : std_logic;
  signal unpacker_valid : std_logic;
  signal data_tx        : std_logic_vector(7 downto 0);
  signal unpacker_busy  : std_logic;

  signal valid_in       : std_logic;
  signal data_in        : std_logic_vector(7 downto 0);
  signal serial_rx_test : std_logic;

  signal imu_status_1     : std_logic;
  signal imu_who_ok_1     : std_logic;
  signal imu_config_ok_1  : std_logic;
  signal imu_reading_ok_1 : std_logic;
  signal imu_accel_z_1    : std_logic_vector(15 downto 0);
  
  signal imu_status_2     : std_logic;
  signal imu_who_ok_2     : std_logic;
  signal imu_config_ok_2  : std_logic;
  signal imu_reading_ok_2 : std_logic;
  signal imu_accel_z_2    : std_logic_vector(15 downto 0);
  
  

  signal sample_valid   : std_logic;
  signal sample_word    : std_logic_vector(15 downto 0);
  signal req_pending    : std_logic;
  
  signal imu_to_send : std_logic := '1';

begin

  ----------------------------------------------------------------
  -- RX invertido: foi isso que fez funcionar no seu teste
  ----------------------------------------------------------------
  serial_rx_test <= not serial_in;

  ----------------------------------------------------------------
  -- IMU
  ----------------------------------------------------------------
  imu_i2c_1 : i2c_mems
    port map (
      clk        => clk,
      rstn       => rstn,
      i2c_scl    => i2c_scl_1,
      i2c_sda    => i2c_sda_1,
      data       => open,
      status     => imu_status_1,
      who_ok     => imu_who_ok_1,
      config_ok  => imu_config_ok_1,
      reading_ok => imu_reading_ok_1,
      accel_z    => imu_accel_z_1
    );
	

  imu_i2c_2 : i2c_mems
    port map (
      clk        => clk,
      rstn       => rstn,
      i2c_scl    => i2c_scl_2,
      i2c_sda    => i2c_sda_2,
      data       => open,
      status     => imu_status_2,
      who_ok     => imu_who_ok_2,
      config_ok  => imu_config_ok_2,
      reading_ok => imu_reading_ok_2,
      accel_z    => imu_accel_z_2
    );
	 
  ----------------------------------------------------------------
  -- Sending Acceleration in z axe
  ----------------------------------------------------------------

  accel_z <= imu_accel_z_1 & imu_accel_z_2;
  
  ----------------------------------------------------------------
  -- UART RX
  ----------------------------------------------------------------
  uart_receiver_1 : UART_RX
    port map (
      clk           => clk,
      rstn          => rstn,
      serial_dat_in => serial_rx_test,
      valid         => valid_in,
      data          => data_in
    );

  ----------------------------------------------------------------
  -- Pedido por UART:
  -- manda b'1' no Python -> ASCII x"31"
  -- se quiser usar b'\x01', troque x"31" por x"01"
  ----------------------------------------------------------------
  process(clk, rstn)
  begin
    if rstn = '0' then
      sample_valid <= '0';
      sample_word  <= (others => '0');
      req_pending  <= '0';

    elsif rising_edge(clk) then
      sample_valid <= '0';

      -- recebeu comando do PC
      if valid_in = '1' then
        if data_in = x"31" then
          req_pending <= '1';
        end if;
      end if;

      -- atende pedido quando IMU e UART estiverem prontos
--      if req_pending = '1' then
--        if imu_who_ok_1 = '1' and imu_status_1 = '1' and unpacker_busy = '0' and imu_to_send = '1' then
--          sample_word  <= imu_accel_z_1;
--          sample_valid <= '1';
--          req_pending  <= '0';
--			 imu_to_send  <= '0';
--			 
--		  elsif imu_who_ok_2 = '1' and imu_status_2 = '1' and unpacker_busy = '0' and imu_to_send = '0' then
--			 sample_word  <= imu_accel_z_2;
--          sample_valid <= '1';
--          req_pending  <= '0';
--			 imu_to_send  <= '1';
--			
--			
--        end if;
--      end if;

		if req_pending = '1' and unpacker_busy = '0' then

		  if imu_to_send = '1' then

			 if imu_who_ok_1 = '1' and imu_status_1 = '1' then
				sample_word <= imu_accel_z_1;
			 else
				sample_word <= x"7FFF"; -- erro sensor 1
			 end if;

			 sample_valid <= '1';
			 req_pending  <= '0';
			 imu_to_send  <= '0';

		  else

			 if imu_who_ok_2 = '1' and imu_status_2 = '1' then
				sample_word <= imu_accel_z_2;
			 else
				sample_word <= x"8000"; -- erro sensor 2
			 end if;

			 sample_valid <= '1';
			 req_pending  <= '0';
			 imu_to_send  <= '1';

		  end if;

		end if;
    end if;
  end process;

  ----------------------------------------------------------------
  -- Unpacker 16 -> 8
  ----------------------------------------------------------------
  unpacker_1 : unpacker_16_8
    port map (
      clk      => clk,
      rstn     => rstn,
      in_valid => sample_valid,
      data_in  => sample_word,
      tx_ready => ready_tx,
      tx_valid => unpacker_valid,
      tx_data  => data_tx,
      busy     => unpacker_busy
    );

  ----------------------------------------------------------------
  -- UART TX
  ----------------------------------------------------------------
  uart_transmitter_1 : UART_TX
    port map (
      clk            => clk,
      rstn           => rstn,
      send           => unpacker_valid,
      send_data      => data_tx,
      serial_dat_out => serial_out,
      ready          => ready_tx
    );

  ----------------------------------------------------------------
  -- LEDs de debug
  ----------------------------------------------------------------
  who_ok(0) <= imu_who_ok_1;
  who_ok(1) <= imu_who_ok_2;

end architecture str;