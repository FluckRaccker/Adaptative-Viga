library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use WORK.TYPES.ALL;

entity control is
  port (
    clock          : in  std_logic;
    reset          : in  std_logic;

    -- Habilita apenas a operação adaptativa em run_mode.
    -- O carregamento de coeficientes pode ocorrer mesmo com este sinal em '0'.
    control_enable : in  std_logic;

    sample_tick    : in  std_logic;

    -- bit 0 -> wf
    -- bit 1 -> ws
    coeff_tick     : in  std_logic_vector(1 downto 0);
    coeff_pending  : in  std_logic_vector(1 downto 0);

    coeff_ok_fir   : in  std_logic_vector(1 downto 0);
    busy_fir       : in  std_logic_vector(1 downto 0);
    valid_fir      : in  std_logic_vector(1 downto 0);

    enable_fir     : out std_logic_vector(2 downto 0);
    mode_fir       : out std_logic;

    coeff_ready    : out std_logic_vector(1 downto 0);

    -- bit 0 -> clear wf
    -- bit 1 -> clear ws
    coeff_clear_fir : out std_logic_vector(1 downto 0);

    configured     : out std_logic;
    running        : out std_logic
  );
end entity;

architecture rtl of control is

  type state_t is (
    idle,

    clear_wf,
    load_wf,
    wait_wf,
    wait_wf_2,

    clear_ws,
    load_ws,
    wait_ws,
    wait_ws_2,

    run_mode,

    pulse_wf,
    wait_wf_valid,

    pulse_ws,
    wait_ws_valid,

    pulse_adapt
  );

  signal state : state_t := idle;

  --------------------------------------------------------------------
  -- Decide para onde ir depois que um carregamento de wf terminou.
  --------------------------------------------------------------------
  procedure next_after_wf_loaded(
    signal state_s          : out state_t;
    signal configured_s     : out std_logic;
    constant control_en     : in  std_logic;
    constant coeff_pending_v : in  std_logic_vector(1 downto 0);
    constant coeff_ok_v      : in  std_logic_vector(1 downto 0)
  ) is
  begin
    -- Se chegou coeficiente de ws, carrega ws mesmo com control_enable = '0'.
    if coeff_pending_v(1) = '1' then
      state_s <= clear_ws;

    -- Se o adaptativo foi ligado e os dois filtros estão prontos, roda.
    elsif control_en = '1' and coeff_ok_v(1) = '1' then
      state_s <= run_mode;

    -- Se o adaptativo foi ligado, mas ws ainda não está pronto, vai carregar ws.
    elsif control_en = '1' and coeff_ok_v(1) = '0' then
      state_s <= clear_ws;

    -- Caso contrário, carregou coeficientes e volta parado para idle.
    else
      state_s <= idle;
    end if;

    configured_s <= coeff_ok_v(0) and coeff_ok_v(1);
  end procedure;

  --------------------------------------------------------------------
  -- Decide para onde ir depois que um carregamento de ws terminou.
  --------------------------------------------------------------------
  procedure next_after_ws_loaded(
    signal state_s          : out state_t;
    signal configured_s     : out std_logic;
    constant control_en     : in  std_logic;
    constant coeff_pending_v : in  std_logic_vector(1 downto 0);
    constant coeff_ok_v      : in  std_logic_vector(1 downto 0)
  ) is
  begin
    -- Se chegou coeficiente novo de wf, dá prioridade ao wf.
    if coeff_pending_v(0) = '1' then
      state_s <= clear_wf;

    -- Se o adaptativo foi ligado e os dois filtros estão prontos, roda.
    elsif control_en = '1' and coeff_ok_v(0) = '1' then
      state_s <= run_mode;

    -- Se o adaptativo foi ligado, mas wf ainda não está pronto, vai carregar wf.
    elsif control_en = '1' and coeff_ok_v(0) = '0' then
      state_s <= clear_wf;

    -- Caso contrário, carregou coeficientes e volta parado para idle.
    else
      state_s <= idle;
    end if;

    configured_s <= coeff_ok_v(0) and coeff_ok_v(1);
  end procedure;

begin

  process(clock)
  begin
    if rising_edge(clock) then

      if reset = '1' then
        state            <= idle;

        enable_fir       <= "000";
        mode_fir         <= '0';
        coeff_ready      <= "00";
        coeff_clear_fir  <= "00";

        configured       <= '0';
        running          <= '0';

      else

        ----------------------------------------------------------------
        -- Defaults: pulsos de 1 clock
        ----------------------------------------------------------------
        enable_fir      <= "000";
        coeff_ready     <= "00";
        coeff_clear_fir <= "00";

        case state is

          --------------------------------------------------------------
          -- Estado parado.
          --
          -- Importante:
          --   coeff_pending acorda a FSM apenas para carregar coeficientes.
          --   control_enable habilita somente a operação adaptativa.
          --------------------------------------------------------------
          when idle =>
            mode_fir   <= '0';
            running    <= '0';
            configured <= coeff_ok_fir(0) and coeff_ok_fir(1);

            -- Carrega wf mesmo com control_enable = '0'.
            if coeff_pending(0) = '1' then
              state <= clear_wf;

            -- Carrega ws mesmo com control_enable = '0'.
            elsif coeff_pending(1) = '1' then
              state <= clear_ws;

            -- Só entra em run_mode se o Python mandar adapt_on/control_start.
            elsif control_enable = '1' then

              if coeff_ok_fir(0) = '1' and coeff_ok_fir(1) = '1' then
                state <= run_mode;

              elsif coeff_ok_fir(0) = '0' then
                state <= clear_wf;

              else
                state <= clear_ws;
              end if;

            end if;

          --------------------------------------------------------------
          -- Limpa contador de coeficientes do wf
          --------------------------------------------------------------
          when clear_wf =>
            mode_fir   <= '0';
            configured <= '0';
            running    <= '0';

            coeff_clear_fir <= "01";

            state <= load_wf;

          --------------------------------------------------------------
          -- Carrega coeficientes do wf
          --------------------------------------------------------------
          when load_wf =>
            mode_fir   <= '0';
            configured <= '0';
            running    <= '0';

            if coeff_ok_fir(0) = '1' then
              next_after_wf_loaded(
                state,
                configured,
                control_enable,
                coeff_pending,
                coeff_ok_fir
              );

            elsif busy_fir(0) = '0' then
              coeff_ready(0) <= '1';

              if coeff_tick(0) = '1' then
                enable_fir <= "001"; -- pulso para wf
                state      <= wait_wf;
              end if;
            end if;

          --------------------------------------------------------------
          -- Espera wf consumir bloco
          --------------------------------------------------------------
          when wait_wf =>
            mode_fir   <= '0';
            configured <= '0';
            running    <= '0';

            state <= wait_wf_2;

          when wait_wf_2 =>
            mode_fir   <= '0';
            configured <= '0';
            running    <= '0';

            if busy_fir(0) = '0' then
              if coeff_ok_fir(0) = '1' then
                next_after_wf_loaded(
                  state,
                  configured,
                  control_enable,
                  coeff_pending,
                  coeff_ok_fir
                );
              else
                state <= load_wf;
              end if;
            end if;

          --------------------------------------------------------------
          -- Limpa contador de coeficientes do ws
          --------------------------------------------------------------
          when clear_ws =>
            mode_fir   <= '0';
            configured <= '0';
            running    <= '0';

            coeff_clear_fir <= "10";

            state <= load_ws;

          --------------------------------------------------------------
          -- Carrega coeficientes do ws
          --------------------------------------------------------------
          when load_ws =>
            mode_fir   <= '0';
            configured <= '0';
            running    <= '0';

            if coeff_ok_fir(1) = '1' then
              next_after_ws_loaded(
                state,
                configured,
                control_enable,
                coeff_pending,
                coeff_ok_fir
              );

            elsif busy_fir(1) = '0' then
              coeff_ready(1) <= '1';

              if coeff_tick(1) = '1' then
                enable_fir <= "010"; -- pulso para ws
                state      <= wait_ws;
              end if;
            end if;

          --------------------------------------------------------------
          -- Espera ws consumir bloco
          --------------------------------------------------------------
          when wait_ws =>
            mode_fir   <= '0';
            configured <= '0';
            running    <= '0';

            state <= wait_ws_2;

          when wait_ws_2 =>
            mode_fir   <= '0';
            configured <= '0';
            running    <= '0';

            if busy_fir(1) = '0' then
              if coeff_ok_fir(1) = '1' then
                next_after_ws_loaded(
                  state,
                  configured,
                  control_enable,
                  coeff_pending,
                  coeff_ok_fir
                );
              else
                state <= load_ws;
              end if;
            end if;

          --------------------------------------------------------------
          -- Operação normal
          --------------------------------------------------------------
          when run_mode =>
            mode_fir   <= '1';
            configured <= '1';
            running    <= '1';

            if control_enable = '0' then
              state <= idle;

            elsif coeff_pending(0) = '1' then
              state <= clear_wf;

            elsif coeff_pending(1) = '1' then
              state <= clear_ws;

            elsif sample_tick = '1' then
              state <= pulse_wf;
            end if;

          --------------------------------------------------------------
          -- Pulsa wf
          --------------------------------------------------------------
          when pulse_wf =>
            mode_fir   <= '1';
            configured <= '1';
            running    <= '1';

            if control_enable = '0' then
              state <= idle;

            elsif busy_fir(0) = '0' then
              enable_fir <= "001";
              state      <= wait_wf_valid;
            end if;

          --------------------------------------------------------------
          -- Espera yf válido
          --------------------------------------------------------------
          when wait_wf_valid =>
            mode_fir   <= '1';
            configured <= '1';
            running    <= '1';

            if control_enable = '0' then
              state <= idle;

            elsif valid_fir(0) = '1' and busy_fir(0) = '0' then
              state <= pulse_ws;
            end if;

          --------------------------------------------------------------
          -- Pulsa ws
          --------------------------------------------------------------
          when pulse_ws =>
            mode_fir   <= '1';
            configured <= '1';
            running    <= '1';

            if control_enable = '0' then
              state <= idle;

            elsif busy_fir(1) = '0' then
              enable_fir <= "010";
              state      <= wait_ws_valid;
            end if;

          --------------------------------------------------------------
          -- Espera xfx válido
          --------------------------------------------------------------
          when wait_ws_valid =>
            mode_fir   <= '1';
            configured <= '1';
            running    <= '1';

            if control_enable = '0' then
              state <= idle;

            elsif valid_fir(1) = '1' and busy_fir(1) = '0' then
              state <= pulse_adapt;
            end if;

          --------------------------------------------------------------
          -- Pulsa filtro adaptativo wc
          --------------------------------------------------------------
          when pulse_adapt =>
            mode_fir   <= '1';
            configured <= '1';
            running    <= '1';

            if control_enable = '0' then
              state <= idle;
            else
              enable_fir <= "100";
              state      <= run_mode;
            end if;

          when others =>
            state <= idle;

        end case;
      end if;
    end if;
  end process;

end architecture;
