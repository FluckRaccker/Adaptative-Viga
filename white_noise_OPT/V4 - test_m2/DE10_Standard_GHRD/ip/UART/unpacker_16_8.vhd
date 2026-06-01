library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity unpacker_16_8 is
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
end entity;

architecture rtl of unpacker_16_8 is
  type state_t is (
    IDLE,
    SEND_MSB,
    WAIT_MSB_START,
    WAIT_MSB_DONE,
    SEND_LSB,
    WAIT_LSB_START,
    WAIT_LSB_DONE
  );

  signal state      : state_t := IDLE;
  signal data_reg   : std_logic_vector(15 downto 0) := (others => '0');
  signal tx_valid_r : std_logic := '0';
  signal tx_data_r  : std_logic_vector(7 downto 0) := (others => '0');
begin

  process(clk, rstn)
  begin
    if rstn = '0' then
      state      <= IDLE;
      data_reg   <= (others => '0');
      tx_data_r  <= (others => '0');
      tx_valid_r <= '0';

    elsif rising_edge(clk) then
      tx_valid_r <= '0';  -- pulso de 1 ciclo por byte

      case state is
        when IDLE =>
          if in_valid = '1' then
            data_reg <= data_in;
            state    <= SEND_MSB;
          end if;

        when SEND_MSB =>
          if tx_ready = '1' then
            tx_data_r  <= data_reg(15 downto 8);
            tx_valid_r <= '1';
            state      <= WAIT_MSB_START;
          end if;

        when WAIT_MSB_START =>
          if tx_ready = '0' then
            state <= WAIT_MSB_DONE;
          end if;

        when WAIT_MSB_DONE =>
          if tx_ready = '1' then
            state <= SEND_LSB;
          end if;

        when SEND_LSB =>
          if tx_ready = '1' then
            tx_data_r  <= data_reg(7 downto 0);
            tx_valid_r <= '1';
            state      <= WAIT_LSB_START;
          end if;

        when WAIT_LSB_START =>
          if tx_ready = '0' then
            state <= WAIT_LSB_DONE;
          end if;

        when WAIT_LSB_DONE =>
          if tx_ready = '1' then
            state <= IDLE;
          end if;

      end case;
    end if;
  end process;

  tx_valid <= tx_valid_r;
  tx_data  <= tx_data_r;
  busy     <= '1' when state /= IDLE else '0';

end architecture;