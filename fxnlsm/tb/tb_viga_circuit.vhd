library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use WORK.TYPES.ALL;

entity tb_viga_circuit is
end entity;

architecture sim of tb_viga_circuit is

  constant clock_PERIOD : time := 20 ns;

  signal clock         : std_logic := '0';
  signal reset       : std_logic := '1';

  signal xr          : std_logic_vector(data_width-1 downto 0) := (others => '0');
  signal xe          : std_logic_vector(data_width-1 downto 0) := (others => '0');

  signal start       : std_logic := '0';
  signal sample_tick : std_logic := '0';
  signal coeff_tick  : std_logic := '0';

  signal coeff_in    : std_logic_vector(4*coeff_width-1 downto 0) := (others => '0');

  signal data_out    : std_logic_vector(data_width-1 downto 0);
  signal LEDR        : std_logic_vector(3 downto 0);

  --------------------------------------------------------------------
  -- Convert integer to signed coefficient word
  --------------------------------------------------------------------
  function coeff_word(v : integer) return std_logic_vector is
  begin
    return std_logic_vector(to_signed(v, coeff_width));
  end function;

  --------------------------------------------------------------------
  -- Pack 4 coefficients into coeff_in.
  --
  -- Convention:
  --   coeff_in = c3 & c2 & c1 & c0
  --
  -- c0 is placed in the least significant bits.
  --------------------------------------------------------------------
  function pack4(
    c0 : integer;
    c1 : integer;
    c2 : integer;
    c3 : integer
  ) return std_logic_vector is
  begin
    return coeff_word(c3) & coeff_word(c2) & coeff_word(c1) & coeff_word(c0);
  end function;

  --------------------------------------------------------------------
  -- Wait N clock cycles
  --------------------------------------------------------------------
  procedure wait_cycles(
    signal clock_in : in std_logic;
    constant n    : in natural
  ) is
  begin
    for k in 1 to n loop
      wait until rising_edge(clock_in);
    end loop;
  end procedure;

  --------------------------------------------------------------------
  -- One-clock pulse
  --------------------------------------------------------------------
  procedure pulse_one_clock(
    signal clock_in : in std_logic;
    signal sig    : out std_logic
  ) is
  begin
    wait until rising_edge(clock_in);
    sig <= '1';

    wait until rising_edge(clock_in);
    sig <= '0';
  end procedure;

  --------------------------------------------------------------------
  -- Send one coefficient packet
  --------------------------------------------------------------------
  procedure send_coeff_packet(
    signal clock_in      : in std_logic;
    signal coeff_tick_s: out std_logic;
    signal coeff_in_s  : out std_logic_vector(4*coeff_width-1 downto 0);
    constant packet    : in  std_logic_vector(4*coeff_width-1 downto 0)
  ) is
  begin
    coeff_in_s <= packet;

    wait_cycles(clock_in, 2);

    pulse_one_clock(clock_in, coeff_tick_s);

    -- Wait enough time for the FIR to consume this coefficient packet.
    -- Adjust if your FIR stays busy for more cycles.
    wait_cycles(clock_in, 20);
  end procedure;

  --------------------------------------------------------------------
  -- Send one data sample
  --------------------------------------------------------------------
  procedure send_sample(
    signal clock_in        : in std_logic;
    signal sample_tick_s : out std_logic;
    signal xr_s          : out std_logic_vector(data_width-1 downto 0);
    signal xe_s          : out std_logic_vector(data_width-1 downto 0);
    constant xr_value    : in integer;
    constant xe_value    : in integer
  ) is
  begin
    xr_s <= std_logic_vector(to_signed(xr_value, data_width));
    xe_s <= std_logic_vector(to_signed(xe_value, data_width));

    wait_cycles(clock_in, 2);

    pulse_one_clock(clock_in, sample_tick_s);

    -- Wait enough time for:
    --   wf -> ws -> adaptive filter
    -- Increase this if adap_fxnlms takes more time.
    wait_cycles(clock_in, 300);
  end procedure;

begin

  --------------------------------------------------------------------
  -- Clock generation
  --------------------------------------------------------------------
  clock <= not clock after clock_PERIOD/2;

  --------------------------------------------------------------------
  -- DUT
  --------------------------------------------------------------------
  dut : entity work.viga_circuit
    port map (
      clock         => clock,
      reset       => reset,

      xr          => xr,
      xe          => xe,

      start       => start,
      sample_tick => sample_tick,
      coeff_tick  => coeff_tick,

      coeff_in    => coeff_in,

      data_out    => data_out,
      LEDR        => LEDR
    );

  --------------------------------------------------------------------
  -- Main stimulus
  --------------------------------------------------------------------
  stim_proc : process

    ------------------------------------------------------------------
    -- Coefficients for 8-tap test
    --
    -- wf = [0, 0, 0, 0, 0, 0, 0, 0]
    -- Therefore:
    --   yf = 0
    --   xc = xr
    --
    -- ws = [8192, 0, 0, 0, 0, 0, 0, 0]
    -- If using Q1.15:
    --   8192 = 0.25
    ------------------------------------------------------------------
    constant WF_PACKET_0 : std_logic_vector(4*coeff_width-1 downto 0) :=
      pack4(0, 0, 0, 0);

    constant WF_PACKET_1 : std_logic_vector(4*coeff_width-1 downto 0) :=
      pack4(0, 0, 0, 0);

    constant WS_PACKET_0 : std_logic_vector(4*coeff_width-1 downto 0) :=
      pack4(32767, 0, 0, 0);

    constant WS_PACKET_1 : std_logic_vector(4*coeff_width-1 downto 0) :=
      pack4(0, 0, 0, 0);

  begin

    ------------------------------------------------------------------
    -- Reset
    ------------------------------------------------------------------
    reset       <= '1';
    start       <= '0';
    sample_tick <= '0';
    coeff_tick  <= '0';
    xr          <= (others => '0');
    xe          <= (others => '0');
    coeff_in    <= (others => '0');

    wait_cycles(clock, 10);

    reset <= '0';

    wait_cycles(clock, 5);

    ------------------------------------------------------------------
    -- Start the controller
    ------------------------------------------------------------------
    pulse_one_clock(clock, start);

    wait_cycles(clock, 5);

    ------------------------------------------------------------------
    -- Load wf coefficients
    ------------------------------------------------------------------
    send_coeff_packet(clock, coeff_tick, coeff_in, WF_PACKET_0);
    send_coeff_packet(clock, coeff_tick, coeff_in, WF_PACKET_1);

    ------------------------------------------------------------------
    -- Load ws coefficients
    ------------------------------------------------------------------
    send_coeff_packet(clock, coeff_tick, coeff_in, WS_PACKET_0);
    send_coeff_packet(clock, coeff_tick, coeff_in, WS_PACKET_1);

    ------------------------------------------------------------------
    -- Wait until both FIR filters report coefficient loaded
    ------------------------------------------------------------------
  --  wait until LEDR(1 downto 0) = "11";

    wait_cycles(clock, 20);

--    report "Both wf and ws coefficients loaded." severity note;

    ------------------------------------------------------------------
    -- Send samples
    --
    -- With wf = 0:
    --   xc = xr
    --
    -- With ws = [8192, 0, ...]:
    --   xfx should be proportional to xc.
    --
    -- xe is kept positive first so adaptive coefficients should move
    -- in one direction. Then xe is made negative to check inversion.
--    ------------------------------------------------------------------
--    send_sample(clock, sample_tick, xr, xe, 1000,  500);
--    send_sample(clock, sample_tick, xr, xe, 1000,  500);
--    send_sample(clock, sample_tick, xr, xe, 1000,  500);
--    send_sample(clock, sample_tick, xr, xe, 1000,  500);
--
--    send_sample(clock, sample_tick, xr, xe, 800,   400);
--    send_sample(clock, sample_tick, xr, xe, 600,   300);
--    send_sample(clock, sample_tick, xr, xe, 400,   200);
--    send_sample(clock, sample_tick, xr, xe, 200,   100);
--
--    ------------------------------------------------------------------
--    -- Change error polarity
--    ------------------------------------------------------------------
--    send_sample(clock, sample_tick, xr, xe, 1000, -500);
--    send_sample(clock, sample_tick, xr, xe, 1000, -500);
--    send_sample(clock, sample_tick, xr, xe, 1000, -500);
--    send_sample(clock, sample_tick, xr, xe, 1000, -500);

		send_sample(clock, sample_tick, xr, xe, 4000,  2000);
		send_sample(clock, sample_tick, xr, xe, 4000,  2000);
		send_sample(clock, sample_tick, xr, xe, 4000,  2000);
		send_sample(clock, sample_tick, xr, xe, 4000,  2000);
		send_sample(clock, sample_tick, xr, xe, 4000,  2000);
		send_sample(clock, sample_tick, xr, xe, 4000,  2000);
		send_sample(clock, sample_tick, xr, xe, 4000,  2000);
		send_sample(clock, sample_tick, xr, xe, 4000,  2000);

		send_sample(clock, sample_tick, xr, xe, 4000, -2000);
		send_sample(clock, sample_tick, xr, xe, 4000, -2000);
		send_sample(clock, sample_tick, xr, xe, 4000, -2000);
		send_sample(clock, sample_tick, xr, xe, 4000, -2000);

    wait_cycles(clock, 500);

    report "End of viga_circuit simulation." severity note;

    wait;
  end process;

end architecture;