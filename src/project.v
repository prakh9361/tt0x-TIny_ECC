/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_ecc_gf2_8 (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // 1. Configure Bidirectional Pins
    // uio[4:0] are inputs (control signals)
    // uio[7:5] are outputs (status flags)
    assign uio_oe = 8'b1110_0000;

    // 2. Map Inputs
    wire [7:0] data_in   = ui_in;
    wire       load_k    = uio_in[0];
    wire       load_x    = uio_in[1];
    wire       load_y    = uio_in[2];
    wire       start     = uio_in[3];
    wire       read_sel  = uio_in[4];

    // 3. Map Outputs
    wire [7:0] result_x;
    wire [7:0] result_y;
    wire       status_done;
    wire       status_busy;
    wire       status_error;

    // Output Mux: Choose X or Y based on read_sel
    assign uo_out       = (read_sel) ? result_y : result_x;
    
    // Status Flags mapping
    assign uio_out[4:0] = 5'b00000; // Tie off unused output paths
    assign uio_out[5]   = status_done;
    assign uio_out[6]   = status_busy;
    assign uio_out[7]   = status_error;

    // 4. Instantiate the Core ECC Logic
    ecc_core_gf2_8 ecc_inst (
        .clk(clk),
        .rst_n(rst_n),
        .data_in(data_in),
        .load_k(load_k),
        .load_x(load_x),
        .load_y(load_y),
        .start(start),
        .result_x(result_x),
        .result_y(result_y),
        .done(status_done),
        .busy(status_busy),
        .error(status_error)
    );

endmodule
