library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity packer_8_16 is
  port (
    clk      : in  std_logic;
    rstn     : in  std_logic;
    rx_valid : in  std_logic;                      
    data_in  : in  std_logic_vector(7 downto 0);   
    ready    : out std_logic;
    data_out : out std_logic_vector(15 downto 0)
  );
end entity;

architecture rtl of packer_8_16 is
  signal msb_reg : std_logic_vector(7 downto 0) := (others => '0');
  signal out_reg : std_logic_vector(15 downto 0) := (others => '0');
  signal sel     : std_logic := '0';
  signal ready_r : std_logic := '0';
begin

  process(clk, rstn)
  begin
    if rstn = '0' then
      msb_reg <= (others => '0');
      out_reg <= (others => '0');
      sel     <= '0';
      ready_r <= '0';

    elsif rising_edge(clk) then
      ready_r <= '0';  -- padrão: ready é pulso de 1 ciclo

      if rx_valid = '1' then
        if sel = '0' then
          -- 1º byte chegou
          msb_reg <= data_in;
          sel <= '1';
        else
          -- 2º byte chegou: forma a palavra e "ready" SOBE no MESMO ciclo do rx_valid
          out_reg <= msb_reg & data_in;  -- msb primeiro, depois lsb
          sel <= '0';
          ready_r <= '1';
        end if;
      end if;
    end if;
  end process;

  data_out <= out_reg;
  ready    <= ready_r;

end architecture;
