library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity dc_blocker_iir is
  generic (
    DATA_WIDTH : positive := 16;
    EXTRA_BITS : natural  := 8;
    SHIFT      : positive := 10
  );
  port (
    clk          : in  std_logic;
    reset        : in  std_logic;  -- ativo em '1'
    sample_valid : in  std_logic;

    x_in         : in  signed(DATA_WIDTH-1 downto 0);

    y_out        : out signed(DATA_WIDTH-1 downto 0);
    mean_out     : out signed(DATA_WIDTH-1 downto 0);
    valid_out    : out std_logic
  );
end entity;

architecture rtl of dc_blocker_iir is

  constant ACC_WIDTH : natural := DATA_WIDTH + EXTRA_BITS + 2;

  constant OUT_MAX_I : integer := (2 ** (DATA_WIDTH-1)) - 1;
  constant OUT_MIN_I : integer := -(2 ** (DATA_WIDTH-1));

  signal mean_q      : signed(ACC_WIDTH-1 downto 0) := (others => '0');
  signal initialized : std_logic := '0';

  function sat_acc_to_data(
    x : signed(ACC_WIDTH-1 downto 0)
  ) return signed is
    variable x_int : signed(ACC_WIDTH-1 downto 0);
  begin
    x_int := shift_right(x, EXTRA_BITS);

    if x_int > to_signed(OUT_MAX_I, ACC_WIDTH) then
      return to_signed(OUT_MAX_I, DATA_WIDTH);

    elsif x_int < to_signed(OUT_MIN_I, ACC_WIDTH) then
      return to_signed(OUT_MIN_I, DATA_WIDTH);

    else
      return resize(x_int, DATA_WIDTH);
    end if;
  end function;

begin

  mean_out <= resize(shift_right(mean_q, EXTRA_BITS), DATA_WIDTH);

  process(clk, reset)
    variable x_q       : signed(ACC_WIDTH-1 downto 0);
    variable err_q     : signed(ACC_WIDTH-1 downto 0);
    variable mean_next : signed(ACC_WIDTH-1 downto 0);
    variable y_q       : signed(ACC_WIDTH-1 downto 0);
  begin
    if reset = '1' then
      mean_q      <= (others => '0');
      initialized <= '0';

      y_out     <= (others => '0');
      valid_out <= '0';

    elsif rising_edge(clk) then
      valid_out <= '0';

      if sample_valid = '1' then
        x_q := shift_left(resize(x_in, ACC_WIDTH), EXTRA_BITS);

        -- Na primeira amostra, inicializa a média com o próprio valor.
        -- Assim evita um transiente gigante por causa da gravidade/offset.
        if initialized = '0' then
          mean_q      <= x_q;
          initialized <= '1';

          y_out     <= (others => '0');
          valid_out <= '1';

        else
          err_q     := x_q - mean_q;
          mean_next := mean_q + shift_right(err_q, SHIFT);
          y_q       := x_q - mean_next;

          mean_q <= mean_next;
          y_out  <= sat_acc_to_data(y_q);

          valid_out <= '1';
        end if;
      end if;
    end if;
  end process;

end architecture;