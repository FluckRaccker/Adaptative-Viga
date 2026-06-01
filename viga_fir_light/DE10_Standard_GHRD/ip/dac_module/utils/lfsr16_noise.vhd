library ieee;
use ieee.std_logic_1164.all;

entity lfsr16_noise is
  port (
    clk   : in  std_logic;
    rst_n : in  std_logic;
    en    : in  std_logic;
    load  : in  std_logic;
    seed  : in  std_logic_vector(15 downto 0);
    q     : out std_logic_vector(15 downto 0)
  );
end entity;

architecture rtl of lfsr16_noise is
  signal r  : std_logic_vector(15 downto 0);
  signal fb : std_logic;
begin
  -- x^16 + x^14 + x^13 + x^11 + 1
  fb <= r(15) xor r(13) xor r(12) xor r(10);

  process(clk, rst_n)
  begin
    if rst_n = '0' then
      r <= x"ACE1";  -- seed não nula
    elsif rising_edge(clk) then
      if load = '1' then
        if seed = x"0000" then
          r <= x"ACE1";
        else
          r <= seed;
        end if;
      elsif en = '1' then
        r <= r(14 downto 0) & fb;
      end if;
    end if;
  end process;

  q <= r;
end architecture;