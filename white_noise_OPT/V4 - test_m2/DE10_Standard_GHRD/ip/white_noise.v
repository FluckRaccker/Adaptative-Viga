module white_noise
(
    /******* clock input *******/
    input  wire        CLOCK_50,

    /****** LEDs *******/
    output wire [6:0]  LEDR,
	 
	 /****** SWITCHs *******/
	 input wire  [1:0]  sw,

    /******** UART ********/
    input  wire        serial_in,
    output wire        serial_out,

    /******** I2C ********/
    output wire        i2c_scl_mems1,
    output wire        i2c_scl_mems2,
    inout  wire        i2c_scl_dac,

    inout  wire        i2c_sda_mems1,
    inout  wire        i2c_sda_mems2,
    inout  wire        i2c_sda_dac,

    /******** General purpose ********/
    input  wire        reset,       // reset externo ativo em 1
    input  wire [63:0] in_a,
    input  wire [63:0] in_b,
    output wire [63:0] out_export
);

    wire clk_50 = CLOCK_50;

    /******** initial reset ********/
    wire rst_initial_50;
    reset_init_blk reset_init_blk_50 (
        .clk    (clk_50),
        .rst_out(rst_initial_50)
    );

    // rst  ativo em 1
    // rstn ativo em 0
//    wire rst  = rst_initial_50 | reset;
    wire rstn = reset;

    /****************** config wires ******************/
    wire        load_config;
    wire [11:0] offset_cfg;
	 wire [11:0] dac_signal;
    wire [31:0] freq_cfg;
    wire [6:0]  amp_noise_cfg;
    wire [6:0]  amp_sin_cfg;


    /****************** blink confirmation ******************/
    blink blk1 (
        .clk(clk_50),
        .led(LEDR[0])
    );

    /****************** mems_sensor ******************/
	 wire [31:0] accel_z;
	 
    mems_sensor_read mems_1 (
        .clk      (clk_50),
        .serial_in(serial_in),
        .rstn     (rstn),
        .serial_out(serial_out),
        .i2c_scl_1(i2c_scl_mems1),
        .i2c_sda_1(i2c_sda_mems1),
        .i2c_scl_2(i2c_scl_mems2),
        .i2c_sda_2(i2c_sda_mems2),
		  .accel_z  (accel_z),
        .who_ok   (LEDR[2:1])
    );

    /****************** command storage ******************/
    simple_cmd_storage cmd_storage (
        .clk        (clk_50),
        .reset_n    (rstn),
        .in_a       (in_a),
        .in_b       (in_b),
        .out_export (out_export),
        .load_config(load_config),
        .offset     (offset_cfg),
        .freq       (freq_cfg),
        .amp_noise  (amp_noise_cfg),
        .amp_sin    (amp_sin_cfg),
		  .accel_z    (accel_z),
		  .dac_signal (dac_signal)
    );

    /****************** DAC ******************/
	 
    dac_module dac1 (
        .clk       (clk_50),
        .reset_n   (rstn),
        .load_cfg  (load_config),
        .amp_sin   (amp_sin_cfg),
        .amp_noise (amp_noise_cfg),
        .offset    (offset_cfg),
        .freq      (freq_cfg),
        .i2c_scl   (i2c_scl_dac),
        .i2c_sda   (i2c_sda_dac),
        .dac_status(LEDR[6:3]),
		  .dac_signal(dac_signal),
        .SW        (sw)
    );

//    /****************** configuração fixa ******************/
//    wire        load_config  = 1'b0;          // ou 1'b0 se seu dac_module só usa defaults
//    wire [11:0] offset_cfg   = 12'd2048;
//    wire [31:0] freq_cfg     = 32'h000001AE;  // exemplo atual
//    wire [6:0]  amp_noise_cfg = 7'd10;
//    wire [6:0]  amp_sin_cfg   = 7'd80;



endmodule