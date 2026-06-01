module i2c_mcp4725 (
    input  wire        clk,
    input  wire        rstn,

    inout  wire        i2c_scl,
    inout  wire        i2c_sda,

    input  wire        start_wr,        // pulso de 1 clk para iniciar escrita
    input  wire [11:0] dac_code,        // valor 0..4095
    input  wire [1:0]  pd_mode,         // 00=normal, 01=1k, 10=100k, 11=500k

    output reg         init_done,
    output reg         busy,
    output reg         done,
    output reg         ack_error
);

    // =========================================================
    // MCP4725
    // =========================================================
    // Endereço padrão:
    // A0 = 0 -> 7'h60
    // A0 = 1 -> 7'h61
    localparam [6:0] MCP4725_ADDR = 7'h60;

    // =========================================================
    // OpenCores I2C
    // fórmula aproximada: SCL = clk / (5 * (PRER + 1))
    // para 50 MHz e 100 kHz -> PRER = 99
    // =========================================================
    localparam [15:0] PRER_VALUE = 16'd99;

    localparam [2:0] REG_PRER_LO = 3'b000;
    localparam [2:0] REG_PRER_HI = 3'b001;
    localparam [2:0] REG_CTR     = 3'b010;
    localparam [2:0] REG_TXR     = 3'b011;
    localparam [2:0] REG_RXR     = 3'b011;
    localparam [2:0] REG_CR      = 3'b100;
    localparam [2:0] REG_SR      = 3'b100;

    localparam [7:0] CMD_START_WRITE = 8'h90; // STA + WR
    localparam [7:0] CMD_WRITE       = 8'h10; // WR
    localparam [7:0] CMD_WRITE_STOP  = 8'h50; // WR + STO
    localparam [7:0] CTR_ENABLE      = 8'h80; // enable core

    wire rst;
    assign rst = ~rstn;

    // =========================================================
    // Interface Wishbone
    // =========================================================
    reg  [2:0] wb_adr_i;
    reg  [7:0] wb_dat_i;
    wire [7:0] wb_dat_o;
    reg        wb_we_i;
    reg        wb_stb_i;
    reg        wb_cyc_i;
    wire       wb_ack_o;
    wire       wb_inta_o;

    // =========================================================
    // Linhas open-drain
    // =========================================================
    wire scl_pad_o;
    wire scl_padoen_o;
    wire sda_pad_o;
    wire sda_padoen_o;

    assign i2c_scl = (scl_padoen_o) ? 1'bz : scl_pad_o;
    assign i2c_sda = (sda_padoen_o) ? 1'bz : sda_pad_o;

    // =========================================================
    // Core I2C
    // =========================================================
    i2c_master_top uut_i2c_master_top (
        .wb_clk_i      (clk),
        .wb_rst_i      (rst),
        .arst_i        (rstn),   // mantido igual ao seu código
        .wb_adr_i      (wb_adr_i),
        .wb_dat_i      (wb_dat_i),
        .wb_dat_o      (wb_dat_o),
        .wb_we_i       (wb_we_i),
        .wb_stb_i      (wb_stb_i),
        .wb_cyc_i      (wb_cyc_i),
        .wb_ack_o      (wb_ack_o),
        .wb_inta_o     (wb_inta_o),

        .scl_pad_i     (i2c_scl),
        .scl_pad_o     (scl_pad_o),
        .scl_padoen_o  (scl_padoen_o),
        .sda_pad_i     (i2c_sda),
        .sda_pad_o     (sda_pad_o),
        .sda_padoen_o  (sda_padoen_o)
    );

    // =========================================================
    // Engine Wishbone simples
    // =========================================================
    reg        wb_start;
    reg        wb_start_we;
    reg  [2:0] wb_start_adr;
    reg  [7:0] wb_start_data;

    reg        wb_done;
    reg  [7:0] wb_rd_data;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            wb_adr_i   <= 3'd0;
            wb_dat_i   <= 8'd0;
            wb_we_i    <= 1'b0;
            wb_stb_i   <= 1'b0;
            wb_cyc_i   <= 1'b0;
            wb_done    <= 1'b0;
            wb_rd_data <= 8'd0;
        end else begin
            wb_done <= 1'b0;

            if (wb_cyc_i) begin
                if (wb_ack_o) begin
                    wb_rd_data <= wb_dat_o;
                    wb_cyc_i   <= 1'b0;
                    wb_stb_i   <= 1'b0;
                    wb_we_i    <= 1'b0;
                    wb_done    <= 1'b1;
                end
            end else if (wb_start) begin
                wb_adr_i <= wb_start_adr;
                wb_dat_i <= wb_start_data;
                wb_we_i  <= wb_start_we;
                wb_cyc_i <= 1'b1;
                wb_stb_i <= 1'b1;
            end
        end
    end

    // =========================================================
    // Dados latched para transmissão
    // =========================================================
    reg [11:0] dac_code_latched;
    reg [1:0]  pd_mode_latched;

    reg [7:0] tx_byte1;
    reg [7:0] tx_byte2;
    reg [7:0] tx_byte3;

    // =========================================================
    // FSM
    // =========================================================
    localparam [4:0]
        ST_CFG_PRER_LO_REQ  = 5'd0,
        ST_CFG_PRER_LO_WAIT = 5'd1,
        ST_CFG_PRER_HI_REQ  = 5'd2,
        ST_CFG_PRER_HI_WAIT = 5'd3,
        ST_CFG_CTR_REQ      = 5'd4,
        ST_CFG_CTR_WAIT     = 5'd5,
        ST_IDLE             = 5'd6,

        ST_W_TXR1_REQ       = 5'd7,
        ST_W_TXR1_WAIT      = 5'd8,
        ST_W_CMD1_REQ       = 5'd9,
        ST_W_CMD1_WAIT      = 5'd10,
        ST_W_POLL1_REQ      = 5'd11,
        ST_W_POLL1_WAIT     = 5'd12,

        ST_W_TXR2_REQ       = 5'd13,
        ST_W_TXR2_WAIT      = 5'd14,
        ST_W_CMD2_REQ       = 5'd15,
        ST_W_CMD2_WAIT      = 5'd16,
        ST_W_POLL2_REQ      = 5'd17,
        ST_W_POLL2_WAIT     = 5'd18,

        ST_W_TXR3_REQ       = 5'd19,
        ST_W_TXR3_WAIT      = 5'd20,
        ST_W_CMD3_REQ       = 5'd21,
        ST_W_CMD3_WAIT      = 5'd22,
        ST_W_POLL3_REQ      = 5'd23,
        ST_W_POLL3_WAIT     = 5'd24;

    reg [4:0] state;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            state            <= ST_CFG_PRER_LO_REQ;

            wb_start         <= 1'b0;
            wb_start_we      <= 1'b0;
            wb_start_adr     <= 3'd0;
            wb_start_data    <= 8'd0;

            dac_code_latched <= 12'd0;
            pd_mode_latched  <= 2'b00;

            tx_byte1         <= 8'd0;
            tx_byte2         <= 8'd0;
            tx_byte3         <= 8'd0;

            init_done        <= 1'b0;
            busy             <= 1'b1;
            done             <= 1'b0;
            ack_error        <= 1'b0;
        end else begin
            wb_start <= 1'b0;
            done     <= 1'b0;

            case (state)
                // -----------------------------------------------------
                // Inicialização do core I2C
                // -----------------------------------------------------
                ST_CFG_PRER_LO_REQ: begin
                    busy          <= 1'b1;
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b1;
                    wb_start_adr  <= REG_PRER_LO;
                    wb_start_data <= PRER_VALUE[7:0];
                    state         <= ST_CFG_PRER_LO_WAIT;
                end

                ST_CFG_PRER_LO_WAIT: begin
                    if (wb_done)
                        state <= ST_CFG_PRER_HI_REQ;
                end

                ST_CFG_PRER_HI_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b1;
                    wb_start_adr  <= REG_PRER_HI;
                    wb_start_data <= PRER_VALUE[15:8];
                    state         <= ST_CFG_PRER_HI_WAIT;
                end

                ST_CFG_PRER_HI_WAIT: begin
                    if (wb_done)
                        state <= ST_CFG_CTR_REQ;
                end

                ST_CFG_CTR_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b1;
                    wb_start_adr  <= REG_CTR;
                    wb_start_data <= CTR_ENABLE;
                    state         <= ST_CFG_CTR_WAIT;
                end

                ST_CFG_CTR_WAIT: begin
                    if (wb_done) begin
                        init_done <= 1'b1;
                        busy      <= 1'b0;
                        state     <= ST_IDLE;
                    end
                end

                // -----------------------------------------------------
                // Espera comando start_wr
                // -----------------------------------------------------
                ST_IDLE: begin
                    busy <= 1'b0;

                    if (start_wr) begin
                        busy             <= 1'b1;
                        ack_error        <= 1'b0;

                        dac_code_latched <= dac_code;
                        pd_mode_latched  <= pd_mode;

                        // byte1 = endereço + W
                        tx_byte1         <= {MCP4725_ADDR, 1'b0};

                        // Fast Mode:
                        // byte2 = 00 PD1 PD0 D11 D10 D9 D8
                        tx_byte2         <= {2'b00, pd_mode, dac_code[11:8]};

                        // byte3 = D7..D0
                        tx_byte3         <= dac_code[7:0];

                        state            <= ST_W_TXR1_REQ;
                    end
                end

                // -----------------------------------------------------
                // Byte 1: endereço + write
                // -----------------------------------------------------
                ST_W_TXR1_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b1;
                    wb_start_adr  <= REG_TXR;
                    wb_start_data <= tx_byte1;
                    state         <= ST_W_TXR1_WAIT;
                end

                ST_W_TXR1_WAIT: begin
                    if (wb_done)
                        state <= ST_W_CMD1_REQ;
                end

                ST_W_CMD1_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b1;
                    wb_start_adr  <= REG_CR;
                    wb_start_data <= CMD_START_WRITE;
                    state         <= ST_W_CMD1_WAIT;
                end

                ST_W_CMD1_WAIT: begin
                    if (wb_done)
                        state <= ST_W_POLL1_REQ;
                end

                ST_W_POLL1_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b0;
                    wb_start_adr  <= REG_SR;
                    wb_start_data <= 8'h00;
                    state         <= ST_W_POLL1_WAIT;
                end

                ST_W_POLL1_WAIT: begin
                    if (wb_done) begin
                        if (wb_rd_data[1]) begin
                            state <= ST_W_POLL1_REQ;   // TIP=1
                        end else begin
                            if (wb_rd_data[7])
                                ack_error <= 1'b1;     // RXACK=1
                            state <= ST_W_TXR2_REQ;
                        end
                    end
                end

                // -----------------------------------------------------
                // Byte 2: modo + nibble alto
                // -----------------------------------------------------
                ST_W_TXR2_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b1;
                    wb_start_adr  <= REG_TXR;
                    wb_start_data <= tx_byte2;
                    state         <= ST_W_TXR2_WAIT;
                end

                ST_W_TXR2_WAIT: begin
                    if (wb_done)
                        state <= ST_W_CMD2_REQ;
                end

                ST_W_CMD2_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b1;
                    wb_start_adr  <= REG_CR;
                    wb_start_data <= CMD_WRITE;
                    state         <= ST_W_CMD2_WAIT;
                end

                ST_W_CMD2_WAIT: begin
                    if (wb_done)
                        state <= ST_W_POLL2_REQ;
                end

                ST_W_POLL2_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b0;
                    wb_start_adr  <= REG_SR;
                    wb_start_data <= 8'h00;
                    state         <= ST_W_POLL2_WAIT;
                end

                ST_W_POLL2_WAIT: begin
                    if (wb_done) begin
                        if (wb_rd_data[1]) begin
                            state <= ST_W_POLL2_REQ;
                        end else begin
                            if (wb_rd_data[7])
                                ack_error <= 1'b1;
                            state <= ST_W_TXR3_REQ;
                        end
                    end
                end

                // -----------------------------------------------------
                // Byte 3: nibble baixo + STOP
                // -----------------------------------------------------
                ST_W_TXR3_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b1;
                    wb_start_adr  <= REG_TXR;
                    wb_start_data <= tx_byte3;
                    state         <= ST_W_TXR3_WAIT;
                end

                ST_W_TXR3_WAIT: begin
                    if (wb_done)
                        state <= ST_W_CMD3_REQ;
                end

                ST_W_CMD3_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b1;
                    wb_start_adr  <= REG_CR;
                    wb_start_data <= CMD_WRITE_STOP;
                    state         <= ST_W_CMD3_WAIT;
                end

                ST_W_CMD3_WAIT: begin
                    if (wb_done)
                        state <= ST_W_POLL3_REQ;
                end

                ST_W_POLL3_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b0;
                    wb_start_adr  <= REG_SR;
                    wb_start_data <= 8'h00;
                    state         <= ST_W_POLL3_WAIT;
                end

                ST_W_POLL3_WAIT: begin
                    if (wb_done) begin
                        if (wb_rd_data[1]) begin
                            state <= ST_W_POLL3_REQ;
                        end else begin
                            if (wb_rd_data[7])
                                ack_error <= 1'b1;

                            busy <= 1'b0;
                            done <= 1'b1;
                            state <= ST_IDLE;
                        end
                    end
                end

                default: begin
                    state <= ST_CFG_PRER_LO_REQ;
                end
            endcase
        end
    end

endmodule