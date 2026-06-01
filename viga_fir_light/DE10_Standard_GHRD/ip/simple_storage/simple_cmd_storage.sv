module simple_cmd_storage
(
    input  logic        clk,
    input  logic        reset_n,        // ativo em 0

    input  logic        sample_valid,
	 output logic        control_enable,

    input  logic [63:0] in_a,           // Data In
    input  logic [63:0] in_b,           // Command In

    output logic [63:0] out_export,     // resposta rápida: DAC + IMU
    output logic [63:0] out_data,       // status/config para Python

    output logic        load_config,

    // Coeficientes separados por filtro:
    // bit 0 -> wf
    // bit 1 -> ws
    output logic [1:0]  coeff_tick,
    output logic [1:0]  coeff_pending,
    input  logic [1:0]  coeff_ready,

    output logic [23:0] offset,         // [11:0] DAC1, [23:12] DAC2
    output logic [63:0] freq,           // [31:0] DAC1, [63:32] DAC2

    output logic [63:0] coeffs,         // bloco entregue ao FIR selecionado

    output logic [13:0] amp_noise,      // [6:0] DAC1, [13:7] DAC2
    output logic [13:0] amp_sin,        // [6:0] DAC1, [13:7] DAC2
    output logic [13:0] amp_fc,         // [6:0] DAC1, [13:7] DAC2
	 output logic [15:0] mitio,
	 
	 output  logic correction,

    // config_dac:
    // [2:0] -> DAC1
    // [5:3] -> DAC2
    output logic [5:0]  config_dac,

    input  logic [31:0] accel_z,
    input  logic [23:0] dac_signal,
	 
	 input  logic [23:0] coeff_count_dbg
);

    // ============================================================
    // Protocolo:
    //
    // in_b[7:0]   = opcode
    // in_b[63:8]  = seq/counter vindo do Python
    //
    // O Python deve mudar in_b a cada comando.
    // Exemplo:
    // in_b = (seq << 8) | opcode
    // ============================================================
    logic [7:0]  opcode;
    logic [63:0] in_b_prev;

    assign opcode = in_b[7:0];

    // ============================================================
    // OpCodes
    // ============================================================
    localparam logic [7:0] CMD_NOP             = 8'h00;

    localparam logic [7:0] CMD_SET_FREQ_DAC1   = 8'h01;
    localparam logic [7:0] CMD_SET_FREQ_DAC2   = 8'h02;

    localparam logic [7:0] CMD_SET_OFF_DAC1    = 8'h03;
    localparam logic [7:0] CMD_SET_OFF_DAC2    = 8'h04;

    localparam logic [7:0] CMD_SET_ASIN_DAC1   = 8'h05;
    localparam logic [7:0] CMD_SET_ASIN_DAC2   = 8'h06;

    localparam logic [7:0] CMD_SET_ANOISE_DAC1 = 8'h07;
    localparam logic [7:0] CMD_SET_ANOISE_DAC2 = 8'h08;

    localparam logic [7:0] CMD_SET_AFC_DAC1    = 8'h09;
    localparam logic [7:0] CMD_SET_AFC_DAC2    = 8'h0A;

    localparam logic [7:0] CMD_SET_CFG_DAC1    = 8'h0B;
    localparam logic [7:0] CMD_SET_CFG_DAC2    = 8'h0C;

    // Coeficientes separados
    localparam logic [7:0] CMD_SET_COEFF_WF    = 8'h20;
    localparam logic [7:0] CMD_SET_COEFF_WS    = 8'h21;

    localparam logic [7:0] CMD_CLEAR_PENDING   = 8'h22;
    localparam logic [7:0] CMD_CLEAR_OVERRUN   = 8'h23;
	 
	 localparam logic [7:0] CMD_CONTROL_START = 8'h30;
	 localparam logic [7:0] CMD_CONTROL_STOP  = 8'h31;
	 
	 localparam logic [7:0] CMD_SET_MITIO = 8'h32;
	 localparam logic [7:0] CMD_SET_CORRECTION_S = 8'h33;

    // ============================================================
    // Configuração dos DACs
    //
    // 0 -> desligado
    // 1 -> senoide
    // 2 -> senoide + ruído
    // 3 -> ruído
    // 4 -> fc adaptativo
    // ============================================================
    localparam logic [2:0] DAC_OFF        = 3'd0;
    localparam logic [2:0] DAC_SINE       = 3'd1;
    localparam logic [2:0] DAC_SINE_NOISE = 3'd2;
    localparam logic [2:0] DAC_NOISE      = 3'd3;
    localparam logic [2:0] DAC_FC         = 3'd4;

    // Buffers separados de coeficientes
    logic [63:0] coeffs_wf;
    logic [63:0] coeffs_ws;

    // Status
    logic [31:0] sample_counter;
    logic [7:0]  command_counter;
    logic [7:0]  last_opcode;
    logic [1:0]  coeff_overrun;

    logic [7:0]  status_flags;
    logic [7:0]  config_status;
	 
	 logic [7:0] wf_blocks_rx;
  	 logic [7:0] ws_blocks_rx;
	 logic [7:0] wf_blocks_sent;
	 logic [7:0] ws_blocks_sent;

    // ============================================================
    // Limita configuração DAC para valores válidos
    // ============================================================
    function automatic logic [2:0] clamp_dac_cfg(input logic [2:0] x);
        begin
            if (x <= DAC_FC)
                clamp_dac_cfg = x;
            else
                clamp_dac_cfg = DAC_OFF;
        end
    endfunction

    // ============================================================
    // Saídas para o Python
    // ============================================================
    assign out_export = {8'd0, dac_signal, accel_z};

    // status_flags:
    // [7:6] coeff_pending
    // [5:4] coeff_ready
    // [3:2] coeff_overrun
    // [1]   load_config
    // [0]   sample_valid
    assign status_flags = {
        coeff_pending,
        coeff_ready,
        coeff_overrun,
        load_config,
        sample_valid
    };

//    assign config_status = {2'b00, config_dac};
		assign config_status = {1'b0, control_enable, config_dac};

//    assign out_data = {
//        command_counter,     // [63:56]
//        last_opcode,         // [55:48]
//        status_flags,        // [47:40]
//        config_status,       // [39:32]
//        sample_counter       // [31:0]
//    };

		assign out_data = {
			 8'd0,
			 last_opcode,
			 coeff_count_dbg[11:0],    // [47:36]
			 coeff_count_dbg[23:12],    // [35:24]
			 status_flags,
			 config_status,
			 8'd0
		};

    // ============================================================
    // Registradores
    // ============================================================
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            in_b_prev       <= 64'd0;

            load_config     <= 1'b0;

            coeff_tick      <= 2'b00;
            coeff_pending   <= 2'b00;
            coeff_overrun   <= 2'b00;

            coeffs          <= 64'd0;
            coeffs_wf       <= 64'd0;
            coeffs_ws       <= 64'd0;

            freq            <= {32'h000001AE, 32'h000001AE};
            offset          <= {12'd2048, 12'd2048};

            amp_sin         <= {7'd80, 7'd80};
            amp_noise       <= {7'd10, 7'd10};
            amp_fc          <= {7'd10, 7'd10};

            config_dac      <= {DAC_OFF, DAC_OFF};

            sample_counter  <= 32'd0;
            command_counter <= 8'd0;
            last_opcode     <= CMD_NOP;
				control_enable <= 1'b0;
				
				wf_blocks_rx   <= 8'd0;
				ws_blocks_rx   <= 8'd0;
				wf_blocks_sent <= 8'd0;
				ws_blocks_sent <= 8'd0;
				
				mitio <= 16'h0021;
				correction <= 1'b1;
        end
        else begin
            load_config <= 1'b0;
            coeff_tick  <= 2'b00;

            // Conta amostras
            if (sample_valid) begin
                sample_counter <= sample_counter + 32'd1;
            end

            // ====================================================
            // Entrega coeficiente para WF
            // ====================================================
            if (coeff_pending[0] && coeff_ready[0]) begin
                coeffs           <= coeffs_wf;
                coeff_tick[0]    <= 1'b1;
                coeff_pending[0] <= 1'b0;
            end

            // ====================================================
            // Entrega coeficiente para WS
            // ====================================================
            else if (coeff_pending[1] && coeff_ready[1]) begin
                coeffs           <= coeffs_ws;
                coeff_tick[1]    <= 1'b1;
                coeff_pending[1] <= 1'b0;
            end

            // ====================================================
            // Processa comando novo
            // ====================================================
            if (in_b != in_b_prev) begin
                in_b_prev <= in_b;

                if (opcode != CMD_NOP) begin
                    command_counter <= command_counter + 8'd1;
                    last_opcode     <= opcode;

                    case (opcode)

                        // ----------------------------------------
                        // Frequência
                        // ----------------------------------------
                        CMD_SET_FREQ_DAC1: begin
                            freq[31:0] <= in_a[31:0];
                            load_config <= 1'b1;
                        end

                        CMD_SET_FREQ_DAC2: begin
                            freq[63:32] <= in_a[31:0];
                            load_config <= 1'b1;
                        end

                        // ----------------------------------------
                        // Offset
                        // ----------------------------------------
                        CMD_SET_OFF_DAC1: begin
                            offset[11:0] <= in_a[11:0];
                            load_config <= 1'b1;
                        end

                        CMD_SET_OFF_DAC2: begin
                            offset[23:12] <= in_a[11:0];
                            load_config <= 1'b1;
                        end

                        // ----------------------------------------
                        // Amplitude senoide
                        // ----------------------------------------
                        CMD_SET_ASIN_DAC1: begin
                            amp_sin[6:0] <= in_a[6:0];
                            load_config <= 1'b1;
                        end

                        CMD_SET_ASIN_DAC2: begin
                            amp_sin[13:7] <= in_a[6:0];
                            load_config <= 1'b1;
                        end

                        // ----------------------------------------
                        // Amplitude ruído
                        // ----------------------------------------
                        CMD_SET_ANOISE_DAC1: begin
                            amp_noise[6:0] <= in_a[6:0];
                            load_config <= 1'b1;
                        end

                        CMD_SET_ANOISE_DAC2: begin
                            amp_noise[13:7] <= in_a[6:0];
                            load_config <= 1'b1;
                        end

                        // ----------------------------------------
                        // Amplitude fc
                        // ----------------------------------------
                        CMD_SET_AFC_DAC1: begin
                            amp_fc[6:0] <= in_a[6:0];
                            load_config <= 1'b1;
                        end

                        CMD_SET_AFC_DAC2: begin
                            amp_fc[13:7] <= in_a[6:0];
                            load_config <= 1'b1;
                        end

                        // ----------------------------------------
                        // Configuração DAC
                        // in_a[2:0]:
                        // 0 off
                        // 1 senoide
                        // 2 senoide + ruído
                        // 3 ruído
                        // 4 fc
                        // ----------------------------------------
                        CMD_SET_CFG_DAC1: begin
                            config_dac[2:0] <= clamp_dac_cfg(in_a[2:0]);
                            load_config <= 1'b1;
                        end

                        CMD_SET_CFG_DAC2: begin
                            config_dac[5:3] <= clamp_dac_cfg(in_a[2:0]);
                            load_config <= 1'b1;
                        end

                        // ----------------------------------------
                        // Coeficientes WF
                        // ----------------------------------------
                        CMD_SET_COEFF_WF: begin
                            if (!coeff_pending[0]) begin
                                coeffs_wf        <= in_a;
                                coeff_pending[0] <= 1'b1;
                            end
                            else begin
                                coeff_overrun[0] <= 1'b1;
                            end
                        end

                        // ----------------------------------------
                        // Coeficientes WS
                        // ----------------------------------------
                        CMD_SET_COEFF_WS: begin
                            if (!coeff_pending[1]) begin
                                coeffs_ws        <= in_a;
                                coeff_pending[1] <= 1'b1;
                            end
                            else begin
                                coeff_overrun[1] <= 1'b1;
                            end
                        end

                        // ----------------------------------------
                        // Limpa pendências
                        // ----------------------------------------
                        CMD_CLEAR_PENDING: begin
                            coeff_pending <= 2'b00;
                        end

                        // ----------------------------------------
                        // Limpa flags de overrun
                        // ----------------------------------------
                        CMD_CLEAR_OVERRUN: begin
                            coeff_overrun <= 2'b00;
                        end
								
								CMD_CONTROL_START: begin
									 control_enable <= 1'b1;
								end

								CMD_CONTROL_STOP: begin
									 control_enable <= 1'b0;
								end
																
								CMD_SET_MITIO: begin
									 mitio <= in_a[15:0];
								end
								
								CMD_SET_CORRECTION_S: begin
									 correction <= in_a[0];
								end

                        default: begin
                            // comando desconhecido: ignora
                        end


                    endcase
                end
            end
        end
    end

endmodule