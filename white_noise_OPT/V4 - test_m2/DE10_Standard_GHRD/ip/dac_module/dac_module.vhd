library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity dac_module is
  generic (
    CLK_HZ    : integer := 50000000;
    SAMPLE_HZ : integer := 2000
  );
	port (
	  clk        : in    std_logic;
	  reset_n    : in    std_logic;
	  load_cfg   : in    std_logic;
	  amp_sin    : in    std_logic_vector(6 downto 0);
	  amp_noise  : in    std_logic_vector(6 downto 0);
	  offset     : in    std_logic_vector(11 downto 0);
	  freq       : in    std_logic_vector(31 downto 0);
	  i2c_scl    : inout std_logic;
	  i2c_sda    : inout std_logic;
	  dac_status : out   std_logic_vector(3 downto 0);
	  dac_signal : out   std_logic_vector(11 downto 0);
	  SW         : in    std_logic_vector(1 downto 0)
	);
end entity;

architecture rtl of dac_module is

  component waveform_gen is
    port (
      clk       : in  std_logic;
      reset     : in  std_logic;
      en        : in  std_logic;
      phase_inc : in  std_logic_vector(31 downto 0);
      sin_out   : out std_logic_vector(11 downto 0);
      cos_out   : out std_logic_vector(11 downto 0);
      squ_out   : out std_logic_vector(11 downto 0);
      saw_out   : out std_logic_vector(11 downto 0)
    );
  end component;

  component i2c_mcp4725 is
    port (
      clk       : in    std_logic;
      rstn      : in    std_logic;
      i2c_scl   : inout std_logic;
      i2c_sda   : inout std_logic;
      start_wr  : in    std_logic;
      dac_code  : in    std_logic_vector(11 downto 0);
      pd_mode   : in    std_logic_vector(1 downto 0);
      init_done : out   std_logic;
      busy      : out   std_logic;
      done      : out   std_logic;
      ack_error : out   std_logic
    );
  end component;

  
  component lfsr16_noise is
    port (
      clk   : in  std_logic;
      rst_n : in  std_logic;
      en    : in  std_logic;
      load  : in  std_logic;
      seed  : in  std_logic_vector(15 downto 0);
      q     : out std_logic_vector(15 downto 0)
    );
  end component;

  constant SAMPLE_DIV : integer := CLK_HZ / SAMPLE_HZ;

  -- 1 kHz com waveform_gen rodando continuamente a 50 MHz:
  -- phase_inc = (1000 / 50_000_000) * 2^32 ≈ 85899 = 0x00014F8B  -> 2HZ 0hAC
	constant DEFAULT_PHASE_INC  : std_logic_vector(31 downto 0) := x"00000102"; -- 3hz
	constant DEFAULT_OFFSET     : unsigned(11 downto 0) := to_unsigned(2048, 12);
	constant DEFAULT_AMP_SIN    : unsigned(6 downto 0)  := to_unsigned(90, 7);
	constant DEFAULT_AMP_NOISE  : unsigned(6 downto 0)  := to_unsigned(100, 7);

	signal phase_inc_reg  : std_logic_vector(31 downto 0);
	signal offset_reg     : unsigned(11 downto 0);
	signal amp_sin_reg    : unsigned(6 downto 0);
	signal amp_noise_reg  : unsigned(6 downto 0);

  signal noise_q       : std_logic_vector(15 downto 0);
  signal selected_u12  : std_logic_vector(11 downto 0);

  signal sin_raw  : std_logic_vector(11 downto 0);
  signal cos_raw  : std_logic_vector(11 downto 0);
  signal squ_raw  : std_logic_vector(11 downto 0);
  signal saw_raw  : std_logic_vector(11 downto 0);

  signal sin_s    : signed(11 downto 0);
  signal sin_off  : signed(12 downto 0);
  signal sin_amp  : integer;
  signal sin_u12  : std_logic_vector(11 downto 0);

  signal dac_start     : std_logic := '0';
  signal dac_code_reg  : std_logic_vector(11 downto 0) := (others => '0');
  signal dac_init_done : std_logic;
  signal dac_busy      : std_logic;
  signal dac_done      : std_logic;
  signal dac_ack_error : std_logic;

  signal sample_cnt    : integer range 0 to SAMPLE_DIV-1 := 0;
  signal sample_tick   : std_logic := '0';

begin

	process(clk, reset_n)
		begin
		  if reset_n = '0' then
			 phase_inc_reg <= DEFAULT_PHASE_INC;
			 offset_reg    <= DEFAULT_OFFSET;
			 amp_sin_reg   <= DEFAULT_AMP_SIN;
			 amp_noise_reg <= DEFAULT_AMP_NOISE;

		  elsif rising_edge(clk) then
			 if load_cfg = '1' then
				phase_inc_reg <= freq;
				offset_reg    <= unsigned(offset);
				amp_sin_reg   <= unsigned(amp_sin);
				amp_noise_reg <= unsigned(amp_noise);
			 end if;
		  end if;
	end process;

  --------------------------------------------------------------------
  -- Debug
  --------------------------------------------------------------------
	dac_status(0) <= dac_init_done;
	dac_status(1) <= dac_busy;
	dac_status(2) <= dac_done;
	dac_status(3) <= dac_ack_error;

  --------------------------------------------------------------------
  -- NCO sempre habilitado
  --------------------------------------------------------------------
  u_waveform_gen : waveform_gen
    port map (
      clk       => clk,
      reset     => reset_n,       -- reset ativo em '0' no waveform_gen
      en        => '1',
      phase_inc => phase_inc_reg,
      sin_out   => sin_raw,
      cos_out   => cos_raw,
      squ_out   => squ_raw,
      saw_out   => saw_raw
    );

  --------------------------------------------------------------------
  -- LFSR de ruído
  -- en = sample_tick -> 1 novo valor pseudoaleatório por amostra do DAC
  --------------------------------------------------------------------
  u_lfsr : lfsr16_noise
    port map (
      clk   => clk,
      rst_n => reset_n,
      en    => sample_tick,
      load  => '0',
      seed  => x"ACE1",
      q     => noise_q
    );

  --------------------------------------------------------------------
  -- signed -> unsigned para o MCP4725
  -- MCP4725 recebe código unsigned 0..4095
  --------------------------------------------------------------------
  sin_s   <= signed(sin_raw);
  sin_amp <= ((to_integer(signed(sin_raw)) * to_integer(amp_sin_reg)) / 100);
  sin_off <= to_signed(sin_amp, 13) + to_signed(to_integer(offset_reg), 13);
  sin_u12 <= std_logic_vector(unsigned(sin_off(11 downto 0)));
  
  process(sin_raw, noise_q, SW, offset_reg, amp_sin_reg, amp_noise_reg)
    variable sine_i  : integer;
    variable noise_i : integer;
    variable out_i   : integer;
	 
  begin
  
		sine_i  := (to_integer(signed(sin_raw)) * to_integer(amp_sin_reg)) / 100;
			
		noise_i := ((to_integer(unsigned(noise_q(11 downto 0))) - 2048) * to_integer(amp_noise_reg)) / 100;

	 
		case SW is
		when "00" =>  -- senoidal limpa
		  out_i := to_integer(offset_reg) + sine_i;

		when "01" =>  -- senoidal com ruído
		  out_i := to_integer(offset_reg) + sine_i + noise_i;

		when "10" =>  -- somente ruído
		  --out_i := to_integer(offset_reg) + (to_integer(unsigned(noise_q(11 downto 0))) - 2048);
		  out_i := to_integer(offset_reg) + noise_i;

		when others => -- reservado / DC médio
		  out_i := to_integer(offset_reg) + sine_i;
		end case;
		
--		if out_i < 0 then
--		  out_i := 0;
--		elsif out_i > 4095 then
--		  out_i := 4095;
--		end if;
		
		selected_u12 <= std_logic_vector(to_unsigned(out_i, 12));
		
  end process;
		

  --------------------------------------------------------------------
  -- Driver do DAC
  --------------------------------------------------------------------
  u_dac : i2c_mcp4725
    port map (
      clk       => clk,
      rstn      => reset_n,
      i2c_scl   => i2c_scl,
      i2c_sda   => i2c_sda,
      start_wr  => dac_start,
      dac_code  => dac_code_reg,
      pd_mode   => "00",
      init_done => dac_init_done,
      busy      => dac_busy,
      done      => dac_done,
      ack_error => dac_ack_error
    );
	
  dac_signal <= dac_code_reg;

  --------------------------------------------------------------------
  -- Tick de amostragem para atualizar o DAC
  --------------------------------------------------------------------
  process(clk, reset_n)
  begin
    if reset_n = '0' then
      sample_cnt  <= 0;
      sample_tick <= '0';
    elsif rising_edge(clk) then
      sample_tick <= '0';

      if sample_cnt = SAMPLE_DIV - 1 then
        sample_cnt  <= 0;
        sample_tick <= '1';
      else
        sample_cnt <= sample_cnt + 1;
      end if;
    end if;
  end process;

  --------------------------------------------------------------------
  -- A cada tick, se o DAC estiver livre, envia a amostra atual
  --------------------------------------------------------------------
  process(clk, reset_n)
  begin
    if reset_n = '0' then
      dac_start    <= '0';
      dac_code_reg <= (others => '0');
    elsif rising_edge(clk) then
      dac_start <= '0';

      if (sample_tick = '1') and (dac_init_done = '1') and (dac_busy = '0') then
        dac_code_reg <= selected_u12;
        dac_start    <= '1';
      end if;
    end if;
  end process;

end architecture;