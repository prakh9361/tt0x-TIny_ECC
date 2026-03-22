`default_nettype none

// ============================================================
// ECC Core GF(2^8) — LUT-based inversion
// Curve: y^2 + xy = x^3 + 0x20*x^2 + 0x01
// Field: GF(2^8), poly = 0x11B
// n=113, G=(143,41), h=2
// ============================================================

module ecc_core_gf2_8 (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data_in,
    input  wire       load_k,
    input  wire       load_x,
    input  wire       load_y,
    input  wire       start,
    output reg  [7:0] result_x,
    output reg  [7:0] result_y,
    output reg        done,
    output reg        busy,
    output reg        error
);

    localparam CURVE_A = 8'h20;
    localparam CURVE_B = 8'h01;

    // --- Registers ---
    reg [7:0] k;
    reg [7:0] xg, yg;
    reg [7:0] xr, yr;
    reg [7:0] lam, temp;
    reg [2:0] bit_idx;
    reg [7:0] x1_saved;
    reg       r_is_inf;
    reg       dbl_only;   // skip ADD_CHK after doubling when R==G

    // --- FSM States ---
    localparam IDLE       = 5'd0;
    localparam SCAN_BIT   = 5'd18;
    localparam NEXT_BIT   = 5'd16;
    localparam DONE_STATE = 5'd17;

    // Doubling
    localparam DBL_LAM_1  = 5'd1;
    localparam DBL_LAM_2  = 5'd2;
    localparam DBL_X3_1   = 5'd3;
    localparam DBL_X3_2   = 5'd4;
    localparam DBL_Y3_1   = 5'd5;
    localparam DBL_Y3_2   = 5'd6;
    localparam DBL_Y3_3   = 5'd7;

    // Addition
    localparam ADD_CHK    = 5'd8;
    localparam ADD_LAM_1  = 5'd9;
    localparam ADD_LAM_2  = 5'd10;
    localparam ADD_X3_1   = 5'd11;
    localparam ADD_X3_2   = 5'd12;
    localparam ADD_Y3_1   = 5'd13;
    localparam ADD_Y3_2   = 5'd14;
    localparam ADD_Y3_3   = 5'd15;

    reg [4:0] state;

    // --- ALU ---
    reg  [7:0] alu_a, alu_b;
    reg  [1:0] alu_op;
    wire [7:0] alu_out;

    alu_gf2_8 alu (
        .a(alu_a), .b(alu_b), .op(alu_op), .result(alu_out)
    );

    // --- Inverse ROM ---
    reg  [7:0] rom_addr;
    wire [7:0] inv_out;

    inv_rom rom (
        .addr(rom_addr), .inv_out(inv_out)
    );

    // --- Combinational Routing ---
    always @(*) begin
        alu_a = 8'b0; alu_b = 8'b0; alu_op = 2'b00; rom_addr = 8'b0;

        case (state)
            // Doubling
            DBL_LAM_1: begin
                rom_addr = xr;
                alu_a    = inv_out;
                alu_b    = yr;
                alu_op   = 2'b10;
            end
            DBL_X3_1: begin alu_a = lam;        alu_op = 2'b01; end
            DBL_X3_2: begin alu_a = temp; alu_b = lam ^ CURVE_A; alu_op = 2'b00; end
            DBL_Y3_1: begin alu_a = x1_saved;   alu_op = 2'b01; end
            DBL_Y3_2: begin alu_a = lam;  alu_b = xr;           alu_op = 2'b10; end
            DBL_Y3_3: begin alu_a = temp; alu_b = lam ^ xr;     alu_op = 2'b00; end

            // Addition
            ADD_LAM_1: begin alu_a = xr;   alu_b = xg;          alu_op = 2'b00; end
            ADD_LAM_2: begin
                rom_addr = temp;
                alu_a    = inv_out;
                alu_b    = yr ^ yg;
                alu_op   = 2'b10;
            end
            ADD_X3_1: begin alu_a = lam;         alu_op = 2'b01; end
            ADD_X3_2: begin alu_a = temp; alu_b = lam ^ xr ^ xg ^ CURVE_A; alu_op = 2'b00; end
            ADD_Y3_1: begin alu_a = x1_saved; alu_b = xr;       alu_op = 2'b00; end
            ADD_Y3_2: begin alu_a = lam;  alu_b = temp;         alu_op = 2'b10; end
            ADD_Y3_3: begin alu_a = temp; alu_b = xr ^ yr;      alu_op = 2'b00; end

            default: ;
        endcase
    end

    // --- Synchronous FSM ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= IDLE;
            k        <= 0; xg <= 0; yg <= 0; xr <= 0; yr <= 0;
            result_x <= 0; result_y <= 0;
            done     <= 0; busy <= 0; error <= 0;
            bit_idx  <= 0; r_is_inf <= 0;
            lam      <= 0; temp <= 0; x1_saved <= 0;
            dbl_only <= 0;
        end else begin
            if (state == IDLE) begin
                if (load_k) k  <= data_in;
                if (load_x) xg <= data_in;
                if (load_y) yg <= data_in;
            end

            case (state)

                IDLE: begin
                    done <= 0; error <= 0;
                    if (start) begin busy <= 1; bit_idx <= 7; state <= SCAN_BIT; end
                end

                // ---------------------------------------------------
                // Find MSB of k, initialise R = G
                SCAN_BIT: begin
                    if (k[bit_idx] == 1'b1) begin
                        xr <= xg; yr <= yg; r_is_inf <= 0;
                        state <= NEXT_BIT;
                    end else begin
                        if (bit_idx == 0) begin
                            error <= 1; done <= 1; busy <= 0; state <= IDLE;
                        end else begin
                            bit_idx <= bit_idx - 1;
                        end
                    end
                end

                // ---------------------------------------------------
                // Point Doubling

                DBL_LAM_1: begin
                    x1_saved <= xr;
                    temp     <= alu_out;
                    state    <= DBL_LAM_2;
                end

                DBL_LAM_2: begin
                    lam   <= temp ^ xr;
                    state <= DBL_X3_1;
                end

                DBL_X3_1: begin temp <= alu_out; state <= DBL_X3_2; end
                DBL_X3_2: begin xr   <= alu_out; state <= DBL_Y3_1; end
                DBL_Y3_1: begin temp <= alu_out; state <= DBL_Y3_2; end
                DBL_Y3_2: begin lam  <= alu_out; state <= DBL_Y3_3; end
                DBL_Y3_3: begin
                    yr <= alu_out;
                    if (dbl_only) begin
                        dbl_only <= 1'b0;
                        state    <= NEXT_BIT;
                    end else begin
                        state    <= ADD_CHK;
                    end
                end

                // ---------------------------------------------------
                // Point Addition

                ADD_CHK: begin
                    x1_saved <= xr;
                    if (k[bit_idx] == 1'b1) begin
                        if (r_is_inf) begin
                            xr <= xg; yr <= yg; r_is_inf <= 1'b0;
                            state <= NEXT_BIT;
                        end else if (xr == xg) begin
                            if (yr == (xg ^ yg)) begin
                                // R = -G, result is infinity
                                r_is_inf <= 1'b1; state <= NEXT_BIT;
                            end else begin
                                // R == G, double only — do not add again
                                dbl_only <= 1'b1;
                                state    <= DBL_LAM_1;
                            end
                        end else begin
                            state <= ADD_LAM_1;
                        end
                    end else begin
                        state <= NEXT_BIT;
                    end
                end

                ADD_LAM_1: begin temp <= alu_out; state <= ADD_LAM_2; end
                ADD_LAM_2: begin lam  <= alu_out; state <= ADD_X3_1;  end
                ADD_X3_1:  begin temp <= alu_out; state <= ADD_X3_2;  end
                ADD_X3_2:  begin xr   <= alu_out; state <= ADD_Y3_1;  end
                ADD_Y3_1:  begin temp <= alu_out; state <= ADD_Y3_2;  end
                ADD_Y3_2:  begin temp <= alu_out; state <= ADD_Y3_3;  end
                ADD_Y3_3:  begin yr   <= alu_out; state <= NEXT_BIT;  end

                // ---------------------------------------------------
                NEXT_BIT: begin
                    if (bit_idx == 0) begin
                        state <= DONE_STATE;
                    end else begin
                        bit_idx  <= bit_idx - 1;
                        x1_saved <= xr;
                        if (r_is_inf) begin
                            state <= ADD_CHK;
                        end else if (xr == 8'b0) begin
                            r_is_inf <= 1'b1; state <= ADD_CHK;
                        end else begin
                            state <= DBL_LAM_1;
                        end
                    end
                end

                DONE_STATE: begin
                    busy <= 0; done <= 1;
                    if (r_is_inf) begin
                        error <= 1; result_x <= 0; result_y <= 0;
                    end else begin
                        result_x <= xr; result_y <= yr;
                    end
                    state <= IDLE;
                end

                default: state <= IDLE;

            endcase
        end
    end

endmodule
