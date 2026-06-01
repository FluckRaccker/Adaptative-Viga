module i2c_mems (
    input  wire              clk,
    input  wire              rstn,

    inout  wire              i2c_scl,
    inout  wire              i2c_sda,

    output reg               status,
    output reg               who_ok,
	 output reg               config_ok,
	 output reg               reading_ok,
    output reg [7:0]         data,

    output reg signed [15:0] gyro_x,
    output reg signed [15:0] gyro_y,
    output reg signed [15:0] gyro_z,
    output reg signed [15:0] accel_x,
    output reg signed [15:0] accel_y,
    output reg signed [15:0] accel_z
);

    // =========================================================
    // Parâmetros do sensor
    // =========================================================
    localparam [6:0] LSM6DS3_ADDR  = 7'h6B;   // troque para 7'h6A se SA0=0

    localparam [7:0] REG_WHO_AM_I  = 8'h0F;
    localparam [7:0] REG_CTRL1_XL  = 8'h10;
    localparam [7:0] REG_CTRL2_G   = 8'h11;
    localparam [7:0] REG_CTRL3_C   = 8'h12;
    localparam [7:0] REG_STATUS    = 8'h1E;

    localparam [7:0] REG_OUTX_L_G  = 8'h22;
    localparam [7:0] REG_OUTX_H_G  = 8'h23;
    localparam [7:0] REG_OUTY_L_G  = 8'h24;
    localparam [7:0] REG_OUTY_H_G  = 8'h25;
    localparam [7:0] REG_OUTZ_L_G  = 8'h26;
    localparam [7:0] REG_OUTZ_H_G  = 8'h27;

    localparam [7:0] REG_OUTX_L_XL = 8'h28;
    localparam [7:0] REG_OUTX_H_XL = 8'h29;
    localparam [7:0] REG_OUTY_L_XL = 8'h2A;
    localparam [7:0] REG_OUTY_H_XL = 8'h2B;
    localparam [7:0] REG_OUTZ_L_XL = 8'h2C;
    localparam [7:0] REG_OUTZ_H_XL = 8'h2D;

    localparam [7:0] WHO_AM_I_OK   = 8'h69;

    // CTRL3_C = 0x44 -> BDU=1, IF_INC=1
    // CTRL1_XL = 0x70 -> 833 Hz, +/-2g
    // CTRL2_G  = 0x70 -> 833 Hz, 250 dps
    localparam [7:0] VAL_CTRL3_C   = 8'h44;
    localparam [7:0] VAL_CTRL1_XL  = 8'h70;
    localparam [7:0] VAL_CTRL2_G   = 8'h70;

    // =========================================================
    // I2C core OpenCores
    // =========================================================
    localparam [15:0] PRER_VALUE   = 16'd99;      // ~100 kHz para 50 MHz
    localparam [23:0] WAIT_CYCLES  = 24'd15000;   // ~300 us para 50 MHz

    localparam [2:0] REG_PRER_LO = 3'b000;
    localparam [2:0] REG_PRER_HI = 3'b001;
    localparam [2:0] REG_CTR     = 3'b010;
    localparam [2:0] REG_TXR     = 3'b011;
    localparam [2:0] REG_RXR     = 3'b011;
    localparam [2:0] REG_CR      = 3'b100;
    localparam [2:0] REG_SR      = 3'b100;

    // Command register do OpenCores I2C
    localparam [7:0] CMD_START_WRITE   = 8'h90; // STA + WR
    localparam [7:0] CMD_WRITE         = 8'h10; // WR
    localparam [7:0] CMD_WRITE_STOP    = 8'h50; // WR + STO
    localparam [7:0] CMD_READ_ACK      = 8'h20; // RD + ACK(0)
    localparam [7:0] CMD_READ_NACK_STO = 8'h68; // RD + ACK(1=NACK) + STO
    localparam [7:0] CTR_ENABLE        = 8'h80; // enable core

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
    // Linhas I2C open-drain
    // =========================================================
    wire scl_pad_o;
    wire scl_padoen_o;
    wire sda_pad_o;
    wire sda_padoen_o;

    assign i2c_scl = (scl_padoen_o) ? 1'bz : scl_pad_o;
    assign i2c_sda = (sda_padoen_o) ? 1'bz : sda_pad_o;

    i2c_master_top uut_i2c_master_top (
        .wb_clk_i      (clk),
        .wb_rst_i      (rst),
        .arst_i        (rstn),
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
    // FSM principal
    // =========================================================
    localparam [5:0]
        ST_CFG_PRER_LO_REQ  = 6'd0,
        ST_CFG_PRER_LO_WAIT = 6'd1,
        ST_CFG_PRER_HI_REQ  = 6'd2,
        ST_CFG_PRER_HI_WAIT = 6'd3,
        ST_CFG_CTR_REQ      = 6'd4,
        ST_CFG_CTR_WAIT     = 6'd5,

        ST_LOAD_CFG         = 6'd6,
        ST_AFTER_CFG_WRITE  = 6'd7,
        ST_AFTER_WHO        = 6'd8,
        ST_IDLE_WAIT        = 6'd9,
        ST_AFTER_STATUS     = 6'd10,
        ST_LOAD_DATA_READ   = 6'd11,
        ST_STORE_DATA_BYTE  = 6'd12,

        // generic write op
        ST_W_TXR1_REQ       = 6'd13,
        ST_W_TXR1_WAIT      = 6'd14,
        ST_W_CMD1_REQ       = 6'd15,
        ST_W_CMD1_WAIT      = 6'd16,
        ST_W_POLL1_REQ      = 6'd17,
        ST_W_POLL1_WAIT     = 6'd18,
        ST_W_TXR2_REQ       = 6'd19,
        ST_W_TXR2_WAIT      = 6'd20,
        ST_W_CMD2_REQ       = 6'd21,
        ST_W_CMD2_WAIT      = 6'd22,
        ST_W_POLL2_REQ      = 6'd23,
        ST_W_POLL2_WAIT     = 6'd24,
        ST_W_TXR3_REQ       = 6'd25,
        ST_W_TXR3_WAIT      = 6'd26,
        ST_W_CMD3_REQ       = 6'd27,
        ST_W_CMD3_WAIT      = 6'd28,
        ST_W_POLL3_REQ      = 6'd29,
        ST_W_POLL3_WAIT     = 6'd30,

        // generic read op
        ST_R_TXR1_REQ       = 6'd31,
        ST_R_TXR1_WAIT      = 6'd32,
        ST_R_CMD1_REQ       = 6'd33,
        ST_R_CMD1_WAIT      = 6'd34,
        ST_R_POLL1_REQ      = 6'd35,
        ST_R_POLL1_WAIT     = 6'd36,
        ST_R_TXR2_REQ       = 6'd37,
        ST_R_TXR2_WAIT      = 6'd38,
        ST_R_CMD2_REQ       = 6'd39,
        ST_R_CMD2_WAIT      = 6'd40,
        ST_R_POLL2_REQ      = 6'd41,
        ST_R_POLL2_WAIT     = 6'd42,
        ST_R_TXR3_REQ       = 6'd43,
        ST_R_TXR3_WAIT      = 6'd44,
        ST_R_CMD3_REQ       = 6'd45,
        ST_R_CMD3_WAIT      = 6'd46,
        ST_R_POLL3_REQ      = 6'd47,
        ST_R_POLL3_WAIT     = 6'd48,
        ST_R_CMD4_REQ       = 6'd49,
        ST_R_CMD4_WAIT      = 6'd50,
        ST_R_POLL4_REQ      = 6'd51,
        ST_R_POLL4_WAIT     = 6'd52,
        ST_R_RX_REQ         = 6'd53,
        ST_R_RX_WAIT        = 6'd54;

    reg [5:0]  state;
    reg [5:0]  return_state;

    reg [23:0] wait_cnt;
    reg [1:0]  cfg_index;
    reg [2:0]  byte_index;

    reg [7:0]  op_reg_addr;
    reg [7:0]  op_wr_data;
    reg [7:0]  rd_byte;
    reg [7:0]  status_reg_byte;

    reg        nack_error;
    reg        pending_gyro;
    reg        pending_accel;
    reg        reading_gyro;

    reg [7:0] gx_l, gx_h, gy_l, gy_h, gz_l, gz_h;
    reg [7:0] ax_l, ax_h, ay_l, ay_h, az_l, az_h;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            state          <= ST_CFG_PRER_LO_REQ;
            return_state   <= ST_CFG_PRER_LO_REQ;
            wait_cnt       <= 24'd0;
            cfg_index      <= 2'd0;
            byte_index     <= 3'd0;

            wb_start       <= 1'b0;
            wb_start_we    <= 1'b0;
            wb_start_adr   <= 3'd0;
            wb_start_data  <= 8'd0;

            op_reg_addr    <= 8'd0;
            op_wr_data     <= 8'd0;
            rd_byte        <= 8'd0;
            status_reg_byte<= 8'd0;

            nack_error     <= 1'b0;
            pending_gyro   <= 1'b0;
            pending_accel  <= 1'b0;
            reading_gyro   <= 1'b0;

            status         <= 1'b0;
            who_ok         <= 1'b0;
				reading_ok     <= 1'b0;
				config_ok      <= 1'b0;
            data           <= 8'd0;

            gx_l <= 8'd0; gx_h <= 8'd0; gy_l <= 8'd0; gy_h <= 8'd0; gz_l <= 8'd0; gz_h <= 8'd0;
            ax_l <= 8'd0; ax_h <= 8'd0; ay_l <= 8'd0; ay_h <= 8'd0; az_l <= 8'd0; az_h <= 8'd0;

            gyro_x         <= 16'sd0;
            gyro_y         <= 16'sd0;
            gyro_z         <= 16'sd0;
            accel_x        <= 16'sd0;
            accel_y        <= 16'sd0;
            accel_z        <= 16'sd0;
        end else begin
            wb_start <= 1'b0;

            case (state)
                // =====================================================
                // Inicialização do core I2C
                // =====================================================
                ST_CFG_PRER_LO_REQ: begin
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
                        cfg_index  <= 2'd0;
                        nack_error <= 1'b0;
                        state      <= ST_LOAD_CFG;
                    end
                end

                // =====================================================
                // Configuração do sensor
                // =====================================================
                ST_LOAD_CFG: begin
                    nack_error <= 1'b0;

                    case (cfg_index)
                        2'd0: begin
                            op_reg_addr  <= REG_CTRL3_C;
                            op_wr_data   <= VAL_CTRL3_C;
                            return_state <= ST_AFTER_CFG_WRITE;
                            state        <= ST_W_TXR1_REQ;
                        end

                        2'd1: begin
                            op_reg_addr  <= REG_CTRL1_XL;
                            op_wr_data   <= VAL_CTRL1_XL;
                            return_state <= ST_AFTER_CFG_WRITE;
                            state        <= ST_W_TXR1_REQ;
                        end

                        2'd2: begin
                            op_reg_addr  <= REG_CTRL2_G;
                            op_wr_data   <= VAL_CTRL2_G;
                            return_state <= ST_AFTER_CFG_WRITE;
                            state        <= ST_W_TXR1_REQ;
									 config_ok    <= 1'b1;
                        end

                        default: begin
                            op_reg_addr  <= REG_WHO_AM_I;
                            return_state <= ST_AFTER_WHO;
                            state        <= ST_R_TXR1_REQ;
                        end
                    endcase
                end

                ST_AFTER_CFG_WRITE: begin
                    if (cfg_index < 2'd2) begin
                        cfg_index <= cfg_index + 2'd1;
                        state     <= ST_LOAD_CFG;
                    end else begin
                        cfg_index    <= 2'd3;
                        nack_error   <= 1'b0;
                        op_reg_addr  <= REG_WHO_AM_I;
                        return_state <= ST_AFTER_WHO;
                        state        <= ST_R_TXR1_REQ;
                    end
                end

                ST_AFTER_WHO: begin
                    who_ok   <= (!nack_error) && (rd_byte == WHO_AM_I_OK);
                    status   <= (!nack_error);
                    wait_cnt <= 24'd0;
                    state    <= ST_IDLE_WAIT;
                end

                // =====================================================
                // Laço principal
                // =====================================================
                ST_IDLE_WAIT: begin
                    if (wait_cnt < (WAIT_CYCLES - 1)) begin
                        wait_cnt <= wait_cnt + 24'd1;
                    end else begin
                        wait_cnt      <= 24'd0;
                        nack_error    <= 1'b0;
                        op_reg_addr   <= REG_STATUS;
                        return_state  <= ST_AFTER_STATUS;
                        state         <= ST_R_TXR1_REQ;
                    end
                end

                ST_AFTER_STATUS: begin
                    status_reg_byte <= rd_byte;
                    pending_gyro    <= rd_byte[1]; // GDA
                    pending_accel   <= rd_byte[0]; // XLDA
                    status          <= (!nack_error);

                    if (!nack_error && rd_byte[1]) begin
                        reading_gyro <= 1'b1;
                        byte_index   <= 3'd0;
                        state        <= ST_LOAD_DATA_READ;
                    end else if (!nack_error && rd_byte[0]) begin
                        reading_gyro <= 1'b0;
                        byte_index   <= 3'd0;
                        state        <= ST_LOAD_DATA_READ;
                    end else begin
                        state <= ST_IDLE_WAIT;
                    end
                end

                ST_LOAD_DATA_READ: begin
                    nack_error   <= 1'b0;
                    return_state <= ST_STORE_DATA_BYTE;

                    if (reading_gyro) begin
                        case (byte_index)
                            3'd0: op_reg_addr <= REG_OUTX_L_G;
                            3'd1: op_reg_addr <= REG_OUTX_H_G;
                            3'd2: op_reg_addr <= REG_OUTY_L_G;
                            3'd3: op_reg_addr <= REG_OUTY_H_G;
                            3'd4: op_reg_addr <= REG_OUTZ_L_G;
                            default: op_reg_addr <= REG_OUTZ_H_G;
                        endcase
                    end else begin
                        case (byte_index)
                            3'd0: op_reg_addr <= REG_OUTX_L_XL;
                            3'd1: op_reg_addr <= REG_OUTX_H_XL;
                            3'd2: op_reg_addr <= REG_OUTY_L_XL;
                            3'd3: op_reg_addr <= REG_OUTY_H_XL;
                            3'd4: op_reg_addr <= REG_OUTZ_L_XL;
                            default: op_reg_addr <= REG_OUTZ_H_XL;
                        endcase
                    end

                    state <= ST_R_TXR1_REQ;
                end

                ST_STORE_DATA_BYTE: begin
                    if (reading_gyro) begin
                        case (byte_index)
                            3'd0: gx_l <= rd_byte;
                            3'd1: gx_h <= rd_byte;
                            3'd2: gy_l <= rd_byte;
                            3'd3: gy_h <= rd_byte;
                            3'd4: gz_l <= rd_byte;
                            3'd5: begin
                                gz_h   <= rd_byte;
                                gyro_x <= {gx_h, gx_l};
                                gyro_y <= {gy_h, gy_l};
                                gyro_z <= {rd_byte, gz_l};
                            end
                        endcase
                    end else begin
                        case (byte_index)
                            3'd0: ax_l <= rd_byte;
                            3'd1: ax_h <= rd_byte;
                            3'd2: ay_l <= rd_byte;
                            3'd3: ay_h <= rd_byte;
                            3'd4: az_l <= rd_byte;
                            3'd5: begin
                                az_h    <= rd_byte;
                                accel_x <= {ax_h, ax_l};
                                accel_y <= {ay_h, ay_l};
                                accel_z <= {rd_byte, az_l};
                            end
                        endcase
                    end

                    if (byte_index < 3'd5) begin
                        byte_index <= byte_index + 3'd1;
                        state      <= ST_LOAD_DATA_READ;
                    end else begin
                        if (reading_gyro) begin
                            pending_gyro <= 1'b0;
                            if (pending_accel) begin
                                reading_gyro <= 1'b0;
                                byte_index   <= 3'd0;
                                state        <= ST_LOAD_DATA_READ;
                            end else begin
                                state <= ST_IDLE_WAIT;
                            end
                        end else begin
                            pending_accel <= 1'b0;
                            state         <= ST_IDLE_WAIT;
                        end
                    end
                end

                // =====================================================
                // Generic WRITE op: [START + SLA+W] [REG] [DATA+STOP]
                // =====================================================
                ST_W_TXR1_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b1;
                    wb_start_adr  <= REG_TXR;
                    wb_start_data <= {LSM6DS3_ADDR, 1'b0};
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
                            state <= ST_W_POLL1_REQ;
                        end else begin
                            if (wb_rd_data[7])
                                nack_error <= 1'b1;
                            state <= ST_W_TXR2_REQ;
                        end
                    end
                end

                ST_W_TXR2_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b1;
                    wb_start_adr  <= REG_TXR;
                    wb_start_data <= op_reg_addr;
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
                                nack_error <= 1'b1;
                            state <= ST_W_TXR3_REQ;
                        end
                    end
                end

                ST_W_TXR3_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b1;
                    wb_start_adr  <= REG_TXR;
                    wb_start_data <= op_wr_data;
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
                                nack_error <= 1'b1;
                            status <= (!nack_error);
                            state  <= return_state;
                        end
                    end
                end

                // =====================================================
                // Generic READ op: [START + SLA+W] [REG] [SR + SLA+R] [READ+STOP]
                // =====================================================
                ST_R_TXR1_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b1;
                    wb_start_adr  <= REG_TXR;
                    wb_start_data <= {LSM6DS3_ADDR, 1'b0};
                    state         <= ST_R_TXR1_WAIT;
                end

                ST_R_TXR1_WAIT: begin
                    if (wb_done)
                        state <= ST_R_CMD1_REQ;
                end

                ST_R_CMD1_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b1;
                    wb_start_adr  <= REG_CR;
                    wb_start_data <= CMD_START_WRITE;
                    state         <= ST_R_CMD1_WAIT;
                end

                ST_R_CMD1_WAIT: begin
                    if (wb_done)
                        state <= ST_R_POLL1_REQ;
                end

                ST_R_POLL1_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b0;
                    wb_start_adr  <= REG_SR;
                    wb_start_data <= 8'h00;
                    state         <= ST_R_POLL1_WAIT;
                end

                ST_R_POLL1_WAIT: begin
                    if (wb_done) begin
                        if (wb_rd_data[1]) begin
                            state <= ST_R_POLL1_REQ;
                        end else begin
                            if (wb_rd_data[7])
                                nack_error <= 1'b1;
                            state <= ST_R_TXR2_REQ;
                        end
                    end
                end

                ST_R_TXR2_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b1;
                    wb_start_adr  <= REG_TXR;
                    wb_start_data <= op_reg_addr;
                    state         <= ST_R_TXR2_WAIT;
                end

                ST_R_TXR2_WAIT: begin
                    if (wb_done)
                        state <= ST_R_CMD2_REQ;
                end

                ST_R_CMD2_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b1;
                    wb_start_adr  <= REG_CR;
                    wb_start_data <= CMD_WRITE;
                    state         <= ST_R_CMD2_WAIT;
                end

                ST_R_CMD2_WAIT: begin
                    if (wb_done)
                        state <= ST_R_POLL2_REQ;
                end

                ST_R_POLL2_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b0;
                    wb_start_adr  <= REG_SR;
                    wb_start_data <= 8'h00;
                    state         <= ST_R_POLL2_WAIT;
                end

                ST_R_POLL2_WAIT: begin
                    if (wb_done) begin
                        if (wb_rd_data[1]) begin
                            state <= ST_R_POLL2_REQ;
                        end else begin
                            if (wb_rd_data[7])
                                nack_error <= 1'b1;
                            state <= ST_R_TXR3_REQ;
                        end
                    end
                end

                ST_R_TXR3_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b1;
                    wb_start_adr  <= REG_TXR;
                    wb_start_data <= {LSM6DS3_ADDR, 1'b1};
                    state         <= ST_R_TXR3_WAIT;
                end

                ST_R_TXR3_WAIT: begin
                    if (wb_done)
                        state <= ST_R_CMD3_REQ;
                end

                ST_R_CMD3_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b1;
                    wb_start_adr  <= REG_CR;
                    wb_start_data <= CMD_START_WRITE;
                    state         <= ST_R_CMD3_WAIT;
                end

                ST_R_CMD3_WAIT: begin
                    if (wb_done)
                        state <= ST_R_POLL3_REQ;
                end

                ST_R_POLL3_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b0;
                    wb_start_adr  <= REG_SR;
                    wb_start_data <= 8'h00;
                    state         <= ST_R_POLL3_WAIT;
                end

                ST_R_POLL3_WAIT: begin
                    if (wb_done) begin
                        if (wb_rd_data[1]) begin
                            state <= ST_R_POLL3_REQ;
                        end else begin
                            if (wb_rd_data[7])
                                nack_error <= 1'b1;
                            state <= ST_R_CMD4_REQ;
                        end
                    end
                end

                ST_R_CMD4_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b1;
                    wb_start_adr  <= REG_CR;
                    wb_start_data <= CMD_READ_NACK_STO;
                    state         <= ST_R_CMD4_WAIT;
                end

                ST_R_CMD4_WAIT: begin
                    if (wb_done)
                        state <= ST_R_POLL4_REQ;
                end

                ST_R_POLL4_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b0;
                    wb_start_adr  <= REG_SR;
                    wb_start_data <= 8'h00;
                    state         <= ST_R_POLL4_WAIT;
                end

                ST_R_POLL4_WAIT: begin
                    if (wb_done) begin
                        if (wb_rd_data[1]) begin
                            state <= ST_R_POLL4_REQ;
                        end else begin
                            state <= ST_R_RX_REQ;
                        end
                    end
                end

                ST_R_RX_REQ: begin
                    wb_start      <= 1'b1;
                    wb_start_we   <= 1'b0;
                    wb_start_adr  <= REG_RXR;
                    wb_start_data <= 8'h00;
                    state         <= ST_R_RX_WAIT;
                end

                ST_R_RX_WAIT: begin
                    if (wb_done) begin
                        rd_byte <= wb_rd_data;
                        data    <= wb_rd_data;
                        status  <= (!nack_error);
                        state   <= return_state;
                    end
                end

                default: begin
                    state <= ST_CFG_PRER_LO_REQ;
                end
            endcase
        end
    end

endmodule