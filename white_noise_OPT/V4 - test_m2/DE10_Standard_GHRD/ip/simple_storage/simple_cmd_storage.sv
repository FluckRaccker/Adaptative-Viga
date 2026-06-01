module simple_cmd_storage
(
    input  logic        clk,
    input  logic        reset_n,     // ativo em 0
    input  logic [63:0] in_a,
    input  logic [63:0] in_b,
    output logic [63:0] out_export,
    output logic        load_config,
    output logic [11:0] offset,
    output logic [31:0] freq,
    output logic [6:0]  amp_noise,
    output logic [6:0]  amp_sin,
	 input logic [31:0]  accel_z,
	 input logic [11:0]  dac_signal
);


    logic [3:0]  cmd;
    logic [63:0] in_b_prev;

    assign cmd = in_b[3:0];

    // out_export sempre mostra o acelerômetro
    assign out_export = {20'd0, dac_signal, accel_z};

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            in_b_prev    <= 64'd0;
            load_config  <= 1'b0;

            // defaults
            freq         <= 32'h000001AE; // ajuste se quiser outro default
            offset       <= 12'd2048;
            amp_sin      <= 7'd80;
            amp_noise    <= 7'd10;
        end
        else begin
            load_config <= 1'b0;

            // só processa quando in_b mudar
            if (in_b != in_b_prev) begin
                in_b_prev <= in_b;

                // ignora quando o Python limpa IN_B para zero
                if (cmd != 4'b0000) begin
                    if (cmd[0])
                        freq <= in_a[31:0];

                    if (cmd[1])
                        offset <= in_a[43:32];

                    if (cmd[2])
                        amp_sin <= in_a[50:44];

                    if (cmd[3])
                        amp_noise <= in_a[57:51];

                    load_config <= 1'b1;
                end
            end
        end
    end

endmodule