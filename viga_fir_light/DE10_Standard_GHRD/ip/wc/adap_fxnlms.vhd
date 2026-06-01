-- Adaptive FIR filter using FxNLMS
-- Based on the original NLMS implementation by Edson Manoel da Silva
-- Modified for FxNLMS:
--   xc  -> input of the adaptive control filter wc
--   xfx -> filtered-x signal, i.e., xfx = ws * xc
--   xe  -> error signal measured at the error sensor
--   fc  -> adaptive control output

library ieee;
use work.basic_op.all;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_signed.all;

entity adap_fxnlms is
  generic (
    ORDER : positive := 300
  );
  port (
    xc        : in  std_logic_vector(15 downto 0);
    xfx       : in  std_logic_vector(15 downto 0);
    xe        : in  std_logic_vector(15 downto 0);
    clock     : in  std_logic;
    reset     : in  std_logic;
    en        : in  std_logic;

    mitio_cfg : in  std_logic_vector(15 downto 0);
	 
	 correction_cfg : in  std_logic;
	 
	 valid_out : out std_logic;
	 
	 

    fc        : out std_logic_vector(15 downto 0)
  );
end entity;

architecture Behavioral of adap_fxnlms is

    --------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------
--    constant ORDER      : natural := 3000;
--    constant ORDER      : natural := 8;
--    constant ORDER      : natural := 300;
    constant ADDR_WIDTH : natural := 12;

    constant LAST_ADDR : unsigned(ADDR_WIDTH-1 downto 0) :=
        to_unsigned(ORDER-1, ADDR_WIDTH);

--    constant mitio : std_logic_vector(15 downto 0) := X"0200";
    constant gama  : std_logic_vector(15 downto 0) := X"0001";

    --------------------------------------------------------------------
    -- State machine declaration
    --------------------------------------------------------------------
    type state is (
        limpa_ram,
        limpa_fim,

        s0,
        espera_amostra,
        s1, s2, s3, s4, s5,
        aguarda,
        s6,
        div,
        saturate,
        aguarda2,
        for_div,
        if_div,
        s7, s8, s9, s10, s11, s12, s13, s14, s15, s16
    );

    --------------------------------------------------------------------
    -- RAM output signals
    --------------------------------------------------------------------
    signal q_dados1 : std_logic_vector(15 downto 0) := (others => '0');
    signal q_dados2 : std_logic_vector(15 downto 0) := (others => '0');
    signal q_coef   : std_logic_vector(15 downto 0) := (others => '0');

    --------------------------------------------------------------------
    -- RAM write-enable signals
    --------------------------------------------------------------------
    signal controle  : std_logic := '0';
    signal controle2 : std_logic := '0';
    signal controle3 : std_logic := '0';

    --------------------------------------------------------------------
    -- RAM data input muxes
    --------------------------------------------------------------------
    signal data_coef_ram : std_logic_vector(15 downto 0) := (others => '0');
    signal data_xfx_ram  : std_logic_vector(15 downto 0) := (others => '0');
    signal data_xc_ram   : std_logic_vector(15 downto 0) := (others => '0');

    --------------------------------------------------------------------
    -- Address and loop counters
    --------------------------------------------------------------------
    signal endereco  : unsigned(ADDR_WIDTH-1 downto 0) := (others => '0');
    signal endereco2 : unsigned(ADDR_WIDTH-1 downto 0) := (others => '0');
    signal endereco3 : unsigned(ADDR_WIDTH-1 downto 0) := (others => '0');

    signal clear_addr : unsigned(ADDR_WIDTH-1 downto 0) := (others => '0');

    signal p : unsigned(ADDR_WIDTH-1 downto 0) := (others => '0');

    signal i : unsigned(ADDR_WIDTH-1 downto 0) := (others => '0');
    signal j : unsigned(ADDR_WIDTH-1 downto 0) := (others => '0');

    signal cont_div : natural range 0 to 15 := 0;

    --------------------------------------------------------------------
    -- Arithmetic signals
    --------------------------------------------------------------------
    signal num          : std_logic_vector(31 downto 0) := (others => '0');
    signal denum        : std_logic_vector(31 downto 0) := (others => '0');
    signal mi           : std_logic_vector(15 downto 0) := (others => '0');

    signal coef         : std_logic_vector(15 downto 0) := (others => '0');
    signal saida_adap   : std_logic_vector(15 downto 0) := (others => '0');

    signal pre_filtrado : std_logic_vector(31 downto 0) := (others => '0');
    signal energ        : std_logic_vector(31 downto 0) := (others => '0');
    signal energ16      : std_logic_vector(15 downto 0) := (others => '0');

    signal fator        : std_logic_vector(31 downto 0) := (others => '0');
    signal fator16      : std_logic_vector(15 downto 0) := (others => '0');

    signal mult32       : std_logic_vector(31 downto 0) := (others => '0');
    signal mult16       : std_logic_vector(15 downto 0) := (others => '0');

    signal erro         : std_logic_vector(15 downto 0) := (others => '0');

    signal fc_reg       : std_logic_vector(15 downto 0) := (others => '0');

    signal estado       : state := limpa_ram;

    --------------------------------------------------------------------
    -- RAM component
    --------------------------------------------------------------------
    component ram is
        port (
            address : in  std_logic_vector(ADDR_WIDTH-1 downto 0);
            clock   : in  std_logic := '1';
            data    : in  std_logic_vector(15 downto 0);
            wren    : in  std_logic;
            q       : out std_logic_vector(15 downto 0)
        );
    end component;

begin

    --------------------------------------------------------------------
    -- Output register
    --------------------------------------------------------------------
    fc <= fc_reg;

    --------------------------------------------------------------------
    -- RAM data muxes
    -- Durante a limpeza, as três RAMs recebem zero.
    --------------------------------------------------------------------
    data_coef_ram <= (others => '0') when (estado = limpa_ram or estado = limpa_fim) else coef;
    data_xfx_ram  <= (others => '0') when (estado = limpa_ram or estado = limpa_fim) else xfx;
    data_xc_ram   <= (others => '0') when (estado = limpa_ram or estado = limpa_fim) else xc;

    --------------------------------------------------------------------
    -- Coefficient memory: wc coefficients
    --------------------------------------------------------------------
    m1_inst : ram
        port map (
            address => std_logic_vector(endereco),
            clock   => clock,
            data    => data_coef_ram,
            wren    => controle,
            q       => q_coef
        );

    --------------------------------------------------------------------
    -- Data memory 1: filtered-x samples
    --------------------------------------------------------------------
    m2_inst : ram
        port map (
            address => std_logic_vector(endereco2),
            clock   => clock,
            data    => data_xfx_ram,
            wren    => controle2,
            q       => q_dados1
        );

    --------------------------------------------------------------------
    -- Data memory 2: xc samples
    --------------------------------------------------------------------
    m3_inst : ram
        port map (
            address => std_logic_vector(endereco3),
            clock   => clock,
            data    => data_xc_ram,
            wren    => controle3,
            q       => q_dados2
        );

    --------------------------------------------------------------------
    -- Main state machine
    --------------------------------------------------------------------
    process(clock)
    begin
        if rising_edge(clock) then

            ----------------------------------------------------------------
            -- Reset síncrono ativo em '1'
            ----------------------------------------------------------------
            if reset = '1' then

                controle  <= '0';
                controle2 <= '0';
                controle3 <= '0';

                endereco  <= (others => '0');
                endereco2 <= (others => '0');
                endereco3 <= (others => '0');
                clear_addr <= (others => '0');

                p <= (others => '0');
                i <= (others => '0');
                j <= (others => '0');

                cont_div <= 0;

                num          <= (others => '0');
                denum        <= (others => '0');
                mi           <= (others => '0');
                coef         <= (others => '0');
                saida_adap   <= (others => '0');
                pre_filtrado <= (others => '0');
                energ        <= (others => '0');
                energ16      <= (others => '0');
                fator        <= (others => '0');
                fator16      <= (others => '0');
                mult32       <= (others => '0');
                mult16       <= (others => '0');
                erro         <= (others => '0');
					 
					 
					 valid_out <= '0';

                fc_reg <= (others => '0');

                estado <= limpa_ram;

            else

                case estado is

                    ----------------------------------------------------------------
                    -- Limpa as RAMs após o reset.
                    -- Isso garante:
                    --   wc = 0
                    --   histórico de xfx = 0
                    --   histórico de xc = 0
                    ----------------------------------------------------------------
                    when limpa_ram =>
                        endereco  <= clear_addr;
                        endereco2 <= clear_addr;
                        endereco3 <= clear_addr;

                        controle  <= '1';
                        controle2 <= '1';
                        controle3 <= '1';

                        fc_reg <= (others => '0');

                        if clear_addr = LAST_ADDR then
                            clear_addr <= (others => '0');
                            estado <= limpa_fim;
                        else
                            clear_addr <= clear_addr + 1;
                            estado <= limpa_ram;
                        end if;

                    ----------------------------------------------------------------
                    -- Um ciclo extra para garantir a escrita do último endereço.
                    ----------------------------------------------------------------
                    when limpa_fim =>
                        controle  <= '0';
                        controle2 <= '0';
                        controle3 <= '0';

                        endereco  <= (others => '0');
                        endereco2 <= (others => '0');
                        endereco3 <= (others => '0');

                        p <= (others => '0');

                        estado <= s0;

                    ----------------------------------------------------------------
                    -- Wait for sample strobe
                    ----------------------------------------------------------------
                    when s0 =>
                        controle  <= '0';
                        controle2 <= '0';
                        controle3 <= '0';

                        if en = '0' then
                            estado <= espera_amostra;
                        end if;

                    ----------------------------------------------------------------
                    -- Store the newest xc and xfx samples at the same circular address
                    ----------------------------------------------------------------
                    when espera_amostra =>
                        if en = '1' then
                            endereco2 <= p;
                            endereco3 <= p;

                            controle2 <= '1';
                            controle3 <= '1';

                            estado <= s1;
                        end if;

                    ----------------------------------------------------------------
                    -- Initialize accumulators and prepare RAMs for reading
                    ----------------------------------------------------------------
                    when s1 =>
                        energ        <= (others => '0');
                        pre_filtrado <= (others => '0');

                        endereco  <= (others => '0');
                        endereco2 <= p;
                        endereco3 <= p;

                        controle  <= '0';
                        controle2 <= '0';
                        controle3 <= '0';

                        i <= (others => '0');

                        estado <= s2;

                    ----------------------------------------------------------------
                    -- Wait one clock cycle for synchronous RAM outputs
                    ----------------------------------------------------------------
                    when s2 =>
                        estado <= s3;

                    ----------------------------------------------------------------
                    -- FIR output and filtered-x energy calculation
                    ----------------------------------------------------------------
                    when s3 =>
                        pre_filtrado <= L_mac(pre_filtrado, q_coef, q_dados2);
                        energ        <= L_mac(energ, q_dados1, q_dados1);

                        estado <= s4;

                    ----------------------------------------------------------------
                    -- Circular buffer address update for the FIR loop
                    ----------------------------------------------------------------
                    when s4 =>
                        if i = LAST_ADDR then
                            i <= (others => '0');
                            estado <= s5;
                        else
                            endereco <= endereco + 1;

                            if endereco2 = LAST_ADDR then
                                endereco2 <= (others => '0');
                            else
                                endereco2 <= endereco2 + 1;
                            end if;

                            if endereco3 = LAST_ADDR then
                                endereco3 <= (others => '0');
                            else
                                endereco3 <= endereco3 + 1;
                            end if;

                            i <= i + 1;
                            estado <= s2;
                        end if;

                    ----------------------------------------------------------------
                    -- Round accumulated values
                    ----------------------------------------------------------------
                    when s5 =>
                        energ16    <= round(energ);
                        saida_adap <= round(pre_filtrado);

                        fc_reg <= round(pre_filtrado);

                        estado <= aguarda;

                    ----------------------------------------------------------------
                    -- Wait one clock cycle for signal update
                    ----------------------------------------------------------------
                    when aguarda =>
								valid_out <= '1'; --Here
                        estado <= s6;

                    ----------------------------------------------------------------
                    -- Use xe directly as the FxNLMS error signal
                    ----------------------------------------------------------------
                    when s6 =>
								valid_out <= '0'; --Here
                        erro <= xe;
                        num  <= (others => '0');

                        estado <= div;

                    ----------------------------------------------------------------
                    -- Prepare division:
                    --   mi = mitio_cfg / (energ16 + gama)
                    ----------------------------------------------------------------
                    when div =>
--                        num   <= X"0000" & mitio_cfg;
								num   <= L_mult(mitio_cfg, erro);
                        denum <= std_logic_vector(
                                    resize(unsigned(energ16), 32) +
                                    resize(unsigned(gama), 32)
                                 );

                        estado <= saturate;

                    ----------------------------------------------------------------
                    -- Saturate denominator to the positive 16-bit signed range
                    ----------------------------------------------------------------
                    when saturate =>
                        if denum > X"00007FFF" then
                            denum <= X"00007FFF";
                        elsif denum < X"FFFF8000" then
                            denum <= X"FFFF8000";
                        end if;

                        estado <= aguarda2;

                    ----------------------------------------------------------------
                    -- Initialize binary divider
                    ----------------------------------------------------------------
                    when aguarda2 =>
                        mi       <= X"0000";
                        cont_div <= 0;

                        estado <= for_div;

                    ----------------------------------------------------------------
                    -- Binary restoring division loop
                    ----------------------------------------------------------------
                    when for_div =>
                        if cont_div < 15 then
                            cont_div <= cont_div + 1;

                            mi  <= SHL(mi, X"0001");
                            num <= SHL(num, X"0001");

                            estado <= if_div;
                        else
                            estado <= s7;
                        end if;

                    ----------------------------------------------------------------
                    -- Divider compare and subtract stage
                    ----------------------------------------------------------------
                    when if_div =>
                        if num >= denum then
                            num <= num - denum;
                            mi  <= mi + X"0001";
                        end if;

                        estado <= for_div;

                    ----------------------------------------------------------------
                    -- Calculate:
                    --   fator = mi * xe
                    ----------------------------------------------------------------
                    when s7 =>
--                        fator <= L_mult(mi, erro);
--								fator <= mi;
                        estado <= s8;

                    ----------------------------------------------------------------
                    -- Round factor
                    ----------------------------------------------------------------
                    when s8 =>
--                        fator16 <= round(fator);
								fator16 <= mi;
                        estado  <= s9;

                    ----------------------------------------------------------------
                    -- Prepare coefficient update loop
                    ----------------------------------------------------------------
                    when s9 =>
                        endereco  <= (others => '0');
                        endereco2 <= p;

                        controle  <= '0';
                        controle2 <= '0';

                        j <= (others => '0');

                        estado <= s10;

                    ----------------------------------------------------------------
                    -- Wait one clock cycle for RAM outputs
                    ----------------------------------------------------------------
                    when s10 =>
                        estado <= s11;

                    ----------------------------------------------------------------
                    -- Coefficient adaptation:
                    --   wc_i(n+1) = wc_i(n) + fator * xfx(n-i)
                    ----------------------------------------------------------------
                    when s11 =>
                        mult32 <= L_mult(fator16, q_dados1);
                        estado <= s12;

                    ----------------------------------------------------------------
                    -- Round multiplication result
                    ----------------------------------------------------------------
                    when s12 =>
                        mult16 <= round(mult32);
                        estado <= s13;

                    ----------------------------------------------------------------
                    -- Add correction to current coefficient and write it back
                    ----------------------------------------------------------------
                    when s13 =>
						  
								if correction_cfg = '0' then
									coef     <= add(q_coef, mult16); --verificar se é soma ou subtração
								else
									coef     <= sub(q_coef, mult16); --verificar se é soma ou subtração
									
								end if;
								
                        controle <= '1';

                        estado <= s14;

                    ----------------------------------------------------------------
                    -- Wait for RAM write
                    ----------------------------------------------------------------
                    when s14 =>
                        estado <= s15;

                    ----------------------------------------------------------------
                    -- Move to next coefficient and next xfx sample
                    ----------------------------------------------------------------
                    when s15 =>
                        controle <= '0';

                        if j = LAST_ADDR then
                            j <= (others => '0');
                            estado <= s16;
                        else
                            endereco <= endereco + 1;

                            if endereco2 = LAST_ADDR then
                                endereco2 <= (others => '0');
                            else
                                endereco2 <= endereco2 + 1;
                            end if;

                            j <= j + 1;
                            estado <= s10;
                        end if;

                    ----------------------------------------------------------------
                    -- Update circular buffer pointer
                    ----------------------------------------------------------------
                    when s16 =>
                        if p = to_unsigned(0, ADDR_WIDTH) then
                            p <= LAST_ADDR;
                        else
                            p <= p - 1;
                        end if;

                        estado <= s0;

                    when others =>
                        estado <= s0;

                end case;
            end if;
        end if;
    end process;

end architecture;