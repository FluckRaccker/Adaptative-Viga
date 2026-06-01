library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use WORK.TYPES.ALL;

entity tb_control is
end entity;

architecture sim of tb_control is

  constant CLK_PERIOD : time := 20 ns;

  signal clk         : std_logic := '0';
  signal reset       : std_logic := '1';
  signal start       : std_logic := '0';
  signal sample_tick : std_logic := '0';
  signal coeff_tick  : std_logic := '0';

  signal coeff_ok_fir : std_logic_vector(1 downto 0);
  signal busy_fir     : std_logic_vector(1 downto 0);
  signal valid_fir    : std_logic_vector(1 downto 0);

  signal enable_fir   : std_logic_vector(2 downto 0);
  signal mode_fir     : std_logic;
  signal configured   : std_logic;
  signal running      : std_logic;

  -- Individual simulated filter flags
  signal coeff_ok_wf : std_logic := '0';
  signal coeff_ok_ws : std_logic := '0';

  signal busy_wf : std_logic := '0';
  signal busy_ws : std_logic := '0';

  signal valid_wf : std_logic := '0';
  signal valid_ws : std_logic := '0';

  -- Pulse counters for checking
  signal wf_load_pulses : natural := 0;
  signal ws_load_pulses : natural := 0;

  signal wf_run_pulses  : natural := 0;
  signal ws_run_pulses  : natural := 0;
  signal wc_run_pulses  : natural := 0;

  --------------------------------------------------------------------
  -- Wait N clock cycles
  --------------------------------------------------------------------
  procedure wait_cycles(
    signal clk_in : in std_logic;
    constant n    : in natural
  ) is
  begin
    for k in 1 to n loop
      wait until rising_edge(clk_in);
    end loop;
  end procedure;

  --------------------------------------------------------------------
  -- Generate a one-clock pulse
  --------------------------------------------------------------------
  procedure pulse_one_clock(
    signal clk_in : in std_logic;
    signal sig    : out std_logic
  ) is
  begin
    wait until rising_edge(clk_in);
    sig <= '1';

    wait until rising_edge(clk_in);
    sig <= '0';
  end procedure;

begin

  --------------------------------------------------------------------
  -- Clock generation
  --------------------------------------------------------------------
  clk <= not clk after CLK_PERIOD/2;

  --------------------------------------------------------------------
  -- Join individual flags into controller input vectors
  --
  -- Index convention:
  --   bit 0 -> wf
  --   bit 1 -> ws
  --------------------------------------------------------------------
  coeff_ok_fir <= coeff_ok_ws & coeff_ok_wf;
  busy_fir     <= busy_ws     & busy_wf;
  valid_fir    <= valid_ws    & valid_wf;

  --------------------------------------------------------------------
  -- DUT: control
  --------------------------------------------------------------------
  dut : entity work.control
    port map (
      clk          => clk,
      reset        => reset,
      start        => start,
      sample_tick  => sample_tick,
      coeff_tick   => coeff_tick,

      coeff_ok_fir => coeff_ok_fir,
      busy_fir     => busy_fir,
      valid_fir    => valid_fir,

      enable_fir   => enable_fir,
      mode_fir     => mode_fir,

      configured   => configured,
      running      => running
    );

  --------------------------------------------------------------------
  -- Simulated wf behavior
  --
  -- In coefficient loading mode:
  --   each enable_fir(0) pulse consumes one coefficient block.
  --   after 2 blocks, coeff_ok_wf becomes '1'.
  --
  -- In run mode:
  --   enable_fir(0) starts wf.
  --   after a few cycles, valid_wf pulses.
  --------------------------------------------------------------------
  wf_model_proc : process
    variable load_count : natural := 0;
  begin
    busy_wf     <= '0';
    valid_wf    <= '0';
    coeff_ok_wf <= '0';

    wait until reset = '0';

    loop
      wait until enable_fir(0) = '1';

      if mode_fir = '0' then
        --------------------------------------------------------------
        -- Coefficient loading mode
        --------------------------------------------------------------
        busy_wf <= '1';

        wait_cycles(clk, 3);

        busy_wf <= '0';
        load_count := load_count + 1;

        if load_count = 2 then
          coeff_ok_wf <= '1';
        end if;

      else
        --------------------------------------------------------------
        -- Filtering mode
        --------------------------------------------------------------
        busy_wf <= '1';
        valid_wf <= '0';

        wait_cycles(clk, 4);

        busy_wf <= '0';
        valid_wf <= '1';

        wait_cycles(clk, 1);

        valid_wf <= '0';
      end if;
    end loop;
  end process;

  --------------------------------------------------------------------
  -- Simulated ws behavior
  --
  -- Same idea as wf, but using enable_fir(1).
  --------------------------------------------------------------------
  ws_model_proc : process
    variable load_count : natural := 0;
  begin
    busy_ws     <= '0';
    valid_ws    <= '0';
    coeff_ok_ws <= '0';

    wait until reset = '0';

    loop
      wait until enable_fir(1) = '1';

      if mode_fir = '0' then
        --------------------------------------------------------------
        -- Coefficient loading mode
        --------------------------------------------------------------
        busy_ws <= '1';

        wait_cycles(clk, 3);

        busy_ws <= '0';
        load_count := load_count + 1;

        if load_count = 2 then
          coeff_ok_ws <= '1';
        end if;

      else
        --------------------------------------------------------------
        -- Filtering mode
        --------------------------------------------------------------
        busy_ws <= '1';
        valid_ws <= '0';

        wait_cycles(clk, 4);

        busy_ws <= '0';
        valid_ws <= '1';

        wait_cycles(clk, 1);

        valid_ws <= '0';
      end if;
    end loop;
  end process;

  --------------------------------------------------------------------
  -- Count enable pulses
  --------------------------------------------------------------------
  count_proc : process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then
        wf_load_pulses <= 0;
        ws_load_pulses <= 0;

        wf_run_pulses <= 0;
        ws_run_pulses <= 0;
        wc_run_pulses <= 0;

      else
        if mode_fir = '0' then
          if enable_fir = "001" then
            wf_load_pulses <= wf_load_pulses + 1;
          elsif enable_fir = "010" then
            ws_load_pulses <= ws_load_pulses + 1;
          end if;

        else
          if enable_fir = "001" then
            wf_run_pulses <= wf_run_pulses + 1;
          elsif enable_fir = "010" then
            ws_run_pulses <= ws_run_pulses + 1;
          elsif enable_fir = "100" then
            wc_run_pulses <= wc_run_pulses + 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  --------------------------------------------------------------------
  -- Check that enable_fir pulses last only one clock cycle
  --------------------------------------------------------------------
  pulse_width_check_proc : process(clk)
    variable enable_prev : std_logic_vector(2 downto 0) := "000";
  begin
    if rising_edge(clk) then
      if reset = '1' then
        enable_prev := "000";
      else
        if enable_prev /= "000" then
          assert enable_fir = "000"
            report "ERROR: enable_fir stayed active for more than one clock cycle"
            severity error;
        end if;

        enable_prev := enable_fir;
      end if;
    end if;
  end process;

  --------------------------------------------------------------------
  -- Main stimulus
  --------------------------------------------------------------------
  stim_proc : process
  begin

    ------------------------------------------------------------------
    -- Initial reset
    ------------------------------------------------------------------
    reset       <= '1';
    start       <= '0';
    sample_tick <= '0';
    coeff_tick  <= '0';

    wait_cycles(clk, 5);

    reset <= '0';

    wait_cycles(clk, 3);

    ------------------------------------------------------------------
    -- Start controller
    ------------------------------------------------------------------
    pulse_one_clock(clk, start);

    wait_cycles(clk, 2);

    ------------------------------------------------------------------
    -- Send coefficient ticks.
    --
    -- For 8 taps:
    --   wf needs 2 coeff_tick pulses
    --   ws needs 2 coeff_tick pulses
    --
    -- The controller decides whether the pulse goes to wf or ws.
    ------------------------------------------------------------------

    -- wf block 0
    pulse_one_clock(clk, coeff_tick);
    wait_cycles(clk, 8);

    -- wf block 1
    pulse_one_clock(clk, coeff_tick);
    wait_cycles(clk, 8);

    -- ws block 0
    pulse_one_clock(clk, coeff_tick);
    wait_cycles(clk, 8);

    -- ws block 1
    pulse_one_clock(clk, coeff_tick);
    wait_cycles(clk, 8);

    ------------------------------------------------------------------
    -- Wait until controller enters run mode
    ------------------------------------------------------------------
    wait until configured = '1';

    wait_cycles(clk, 2);

    ------------------------------------------------------------------
    -- Check configuration results
    ------------------------------------------------------------------
    assert wf_load_pulses = 2
      report "ERROR: wf did not receive exactly 2 coefficient load pulses"
      severity error;

    assert ws_load_pulses = 2
      report "ERROR: ws did not receive exactly 2 coefficient load pulses"
      severity error;

    assert mode_fir = '1'
      report "ERROR: mode_fir should be '1' after configuration"
      severity error;

    assert running = '1'
      report "ERROR: running should be '1' after configuration"
      severity error;

    ------------------------------------------------------------------
    -- Send one sample tick and check operation sequence:
    --   wf -> ws -> adaptive
    ------------------------------------------------------------------
    pulse_one_clock(clk, sample_tick);

    wait until enable_fir = "001" and mode_fir = '1';
    report "Run pulse detected: wf";

    wait until enable_fir = "010" and mode_fir = '1';
    report "Run pulse detected: ws";

    wait until enable_fir = "100" and mode_fir = '1';
    report "Run pulse detected: adaptive wc";

    wait_cycles(clk, 5);

    ------------------------------------------------------------------
    -- Send another sample tick
    ------------------------------------------------------------------
    pulse_one_clock(clk, sample_tick);

    wait until enable_fir = "001" and mode_fir = '1';
    wait until enable_fir = "010" and mode_fir = '1';
    wait until enable_fir = "100" and mode_fir = '1';

    wait_cycles(clk, 5);

    ------------------------------------------------------------------
    -- Final checks
    ------------------------------------------------------------------
    assert wf_run_pulses = 2
      report "ERROR: wf did not run exactly 2 times"
      severity error;

    assert ws_run_pulses = 2
      report "ERROR: ws did not run exactly 2 times"
      severity error;

    assert wc_run_pulses = 2
      report "ERROR: adaptive filter did not run exactly 2 times"
      severity error;

    report "Simulation finished successfully." severity note;

    wait;
  end process;

end architecture;