library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity dac_module is
  generic (
    CLK_HZ    : integer := 50000000;
    SAMPLE_HZ : integer := 2000;

    -- Se fc_in vier como signed 16 bits normalizado tipo Q15,
    -- dividir por 16 transforma aproximadamente em escala de 12 bits:
    -- -32768..32767 -> -2048..2047
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

    -- Sinal vindo do filtro adaptativo
    -- Idealmente centrado em zero e com sinal.
    fc_in      : in    signed(15 downto 0);
    fc_valid   : in    std_logic;

    i2c_scl    : inout std_logic;
    i2c_sda    : inout std_logic;

    dac_status : out   std_logic_vector(3 downto 0);
    dac_signal : out   std_logic_vector(11 downto 0);

    -- SW:
    -- "00" -> senoide
    -- "01" -> senoide + ruído
    -- "10" -> ruído
    -- "11" -> filtro adaptativo fc(n)
    config         : in    std_logic_vector(2 downto 0)
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
		address   : in    std_logic_vector(6 downto 0);
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

  constant DEFAULT_PHASE_INC  : std_logic_vector(31 downto 0) := x"000001AE";
  constant DEFAULT_OFFSET     : unsigned(11 downto 0) := to_unsigned(2048, 12);
  constant DEFAULT_AMP_SIN    : unsigned(6 downto 0)  := to_unsigned(80, 7);
  constant DEFAULT_AMP_NOISE  : unsigned(6 downto 0)  := to_unsigned(10, 7);
  constant DEFAULT_AMP_FC     : unsigned(6 downto 0)  := to_unsigned(10, 7);

  signal phase_inc_reg  : std_logic_vector(31 downto 0);
  signal offset_reg     : unsigned(11 downto 0);
  signal amp_sin_reg    : unsigned(6 downto 0);
  signal amp_noise_reg  : unsigned(6 downto 0);
  signal amp_fc_reg     : unsigned(6 downto 0);

  signal noise_q       : std_logic_vector(15 downto 0);
  signal selected_u12  : std_logic_vector(11 downto 0);

  signal sin_raw  : std_logic_vector(11 downto 0);
  signal cos_raw  : std_logic_vector(11 downto 0);
  signal squ_raw  : std_logic_vector(11 downto 0);
  signal saw_raw  : std_logic_vector(11 downto 0);

  signal dac_start     : std_logic := '0';
  signal dac_code_reg  : std_logic_vector(11 downto 0) := (others => '0');
  signal dac_init_done : std_logic;
  signal dac_busy      : std_logic;
  signal dac_done      : std_logic;
  signal dac_ack_error : std_logic;

  signal sample_cnt    : integer range 0 to SAMPLE_DIV-1 := 0;
  signal sample_tick   : std_logic := '0';
  
    function sat_u12(x : integer) return std_logic_vector is
    variable y : unsigned(11 downto 0);
	  begin
		 if x <= 0 then
			y := (others => '0');

		 elsif x >= 4095 then
			y := (others => '1');

		 else
			y := to_unsigned(x, 12);
		 end if;

		 return std_logic_vector(y);
	  end function;


begin


  --------------------------------------------------------------------
  -- Registradores de configuração
  --------------------------------------------------------------------
  process(clk, reset_n)
  begin
    if reset_n = '0' then
      phase_inc_reg <= DEFAULT_PHASE_INC;
      offset_reg    <= DEFAULT_OFFSET;
      amp_sin_reg   <= DEFAULT_AMP_SIN;
      amp_noise_reg <= DEFAULT_AMP_NOISE;
      amp_fc_reg    <= DEFAULT_AMP_FC;

    elsif rising_edge(clk) then
      if load_cfg = '1' then
        phase_inc_reg <= freq;
        offset_reg    <= unsigned(offset);
        amp_sin_reg   <= unsigned(amp_sin);
        amp_noise_reg <= unsigned(amp_noise);
        amp_fc_reg    <= unsigned(amp_fc);
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
  -- NCO
  --------------------------------------------------------------------
  u_waveform_gen : waveform_gen
    port map (
      clk       => clk,
      reset     => reset_n,
      en        => '1',
      phase_inc => phase_inc_reg,
      sin_out   => sin_raw,
      cos_out   => cos_raw,
      squ_out   => squ_raw,
      saw_out   => saw_raw
    );

  --------------------------------------------------------------------
  -- Ruído
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
  -- Seleção do sinal enviado ao DAC
  --------------------------------------------------------------------
process(sin_raw, noise_q, config, offset_reg,
        amp_sin_reg, amp_noise_reg, amp_fc_reg, fc_in)

  variable sine_i    : integer;
  variable noise_i   : integer;
  variable fc_base_i : integer;
  variable fc_i      : integer;
  variable out_i     : integer;

begin

  -- Senoide com controle percentual
  sine_i :=
    (to_integer(signed(sin_raw)) * to_integer(amp_sin_reg)) / 100;

  -- Ruído centralizado em zero e com controle percentual
  noise_i :=
    ((to_integer(unsigned(noise_q(11 downto 0))) - 2048)
     * to_integer(amp_noise_reg)) / 100;

  -- fc_in normalmente vem com 16 bits signed.
  -- O shift reduz para uma escala próxima de 12 bits.
  fc_base_i := to_integer(fc_in) / (2 ** FC_SHIFT);

  -- Controle percentual independente para o fc
  fc_i := (fc_base_i * to_integer(amp_fc_reg)) / 100;

  case config is

    when "000" =>  -- desligado / repouso do atuador
      out_i := to_integer(offset_reg);

    when "001" =>  -- senoide limpa
      out_i := to_integer(offset_reg) + sine_i;

    when "010" =>  -- senoide com ruído
      out_i := to_integer(offset_reg) + sine_i + noise_i;

    when "011" =>  -- somente ruído
      out_i := to_integer(offset_reg) + noise_i;

    when "100" =>  -- filtro adaptativo fc(n)
      out_i := to_integer(offset_reg) + fc_i;

    when others =>
      out_i := to_integer(offset_reg);

  end case;

  selected_u12 <= sat_u12(out_i);

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
		address   => "1100001",
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
  -- Tick de amostragem para os modos internos
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
  -- Envio para o DAC
  --------------------------------------------------------------------
	process(clk, reset_n)
	begin
	  if reset_n = '0' then
		 dac_start    <= '0';
		 dac_code_reg <= std_logic_vector(DEFAULT_OFFSET);

	  elsif rising_edge(clk) then
		 dac_start <= '0';

		 if (dac_init_done = '1') and (dac_busy = '0') then

			-- Modos internos: desligado, senoide, senoide+ruído, ruído
			if (config /= "100") and (sample_tick = '1') then
			  dac_code_reg <= selected_u12;
			  dac_start    <= '1';

			-- Modo adaptativo: só atualiza quando chega novo fc
			elsif (config = "100") and (fc_valid = '1') then
			  dac_code_reg <= selected_u12;
			  dac_start    <= '1';

			end if;

		 end if;
	  end if;
	end process;

end architecture;