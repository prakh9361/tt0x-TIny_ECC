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

    // --- FSM States ---
    localparam IDLE       = 5'd0;
    localparam SCAN_BIT   = 5'd18;
    localparam NEXT_BIT   = 5'd16;
    localparam DONE_STATE = 5'd17;

    // Doubling: lambda = xr + yr*inv(xr)
    //           x3 = lam^2 + lam + CURVE_A
    //           y3 = x1^2 + lam*x3 + x3
    localparam DBL_LAM_1  = 5'd1;   // rom_addr=xr; temp = inv(xr)*yr
    localparam DBL_LAM_2  = 5'd2;   // lam  = temp ^ xr
    localparam DBL_X3_1   = 5'd3;   // temp = lam^2
    localparam DBL_X3_2   = 5'd4;   // xr   = lam^2 ^ lam ^ CURVE_A
    localparam DBL_Y3_1   = 5'd5;   // temp = x1_saved^2
    localparam DBL_Y3_2   = 5'd6;   // lam  = lam * xr(new x3)
    localparam DBL_Y3_3   = 5'd7;   // yr   = temp ^ lam ^ xr

    // Addition: lambda = (yr^yg) * inv(xr^xg)
    //           x3 = lam^2 + lam + xr + xg + CURVE_A
    //           y3 = lam*(x1+x3) + x3 + y1
    localparam ADD_CHK    = 5'd8;
    localparam ADD_LAM_1  = 5'd9;   // temp = xr^xg (denominator)
    localparam ADD_LAM_2  = 5'd10;  // rom_addr=temp; lam = inv(xr^xg)*(yr^yg)
    localparam ADD_X3_1   = 5'd11;  // temp = lam^2
    localparam ADD_X3_2   = 5'd12;  // xr   = new x3
    localparam ADD_Y3_1   = 5'd13;  // temp = x1_saved ^ xr(new x3)
    localparam ADD_Y3_2   = 5'd14;  // temp = lam * temp
    localparam ADD_Y3_3   = 5'd15;  // yr   = temp ^ xr ^ yr(old y1)

    // Validation
    localparam VAL_Y2     = 5'd19;  // temp = y^2
    localparam VAL_XY     = 5'd20;  // lam  = x*y
    localparam VAL_LEFT   = 5'd21;  // temp = y^2 + xy  (left side)
    localparam VAL_X2     = 5'd22;  // temp = x^2
    localparam VAL_AX2    = 5'd23;  // lam  = CURVE_A * x^2
    localparam VAL_X3     = 5'd24;  // check temp == x^3 ^ lam ^ CURVE_B

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
            // DBL_LAM_1: inv(xr) from ROM, multiply by yr
            DBL_LAM_1: begin
                rom_addr = xr;
                alu_a    = inv_out;
                alu_b    = yr;
                alu_op   = 2'b10;
            end
            // DBL_LAM_2: lam = temp ^ xr  (XOR is free, no ALU needed)
            // handled entirely in FSM register assignment
            DBL_X3_1: begin alu_a = lam;        alu_op = 2'b01; end
            DBL_X3_2: begin alu_a = temp; alu_b = lam ^ CURVE_A; alu_op = 2'b00; end
            DBL_Y3_1: begin alu_a = x1_saved;   alu_op = 2'b01; end
            DBL_Y3_2: begin alu_a = lam;  alu_b = xr;           alu_op = 2'b10; end
            // DBL_Y3_3: yr = x1^2 ^ lam*x3 ^ x3 = temp ^ lam ^ xr
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
            ADD_Y3_3: begin alu_a = alu_out; alu_b = xr ^ yr;   alu_op = 2'b00; end

            // Validation
            VAL_Y2:   begin alu_a = yg;          alu_op = 2'b01; end
            VAL_XY:   begin alu_a = xg;  alu_b = yg;            alu_op = 2'b10; end
            VAL_LEFT: begin alu_a = temp; alu_b = lam;           alu_op = 2'b00; end
            VAL_X2:   begin alu_a = xg;          alu_op = 2'b01; end
            VAL_AX2:  begin alu_a = CURVE_A; alu_b = temp;       alu_op = 2'b10; end
            VAL_X3:   begin alu_a = xg;  alu_b = temp;           alu_op = 2'b10; end

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
        end else begin
            if (load_k) k  <= data_in;
            if (load_x) xg <= data_in;
            if (load_y) yg <= data_in;

            case (state)

                IDLE: begin
                    done <= 0; error <= 0;
                    if (start) begin busy <= 1; state <= VAL_Y2; end
                end

                // ---------------------------------------------------
                // Validation
                // left  = y^2 + xy
                // right = x^3 + CURVE_A*x^2 + CURVE_B
                VAL_Y2:   begin temp <= alu_out; state <= VAL_XY;   end
                VAL_XY:   begin lam  <= alu_out; state <= VAL_LEFT; end
                VAL_LEFT: begin temp <= alu_out; state <= VAL_X2;   end // temp=left
                VAL_X2:   begin temp <= alu_out; state <= VAL_AX2;  end // temp=x^2
                VAL_AX2:  begin lam  <= alu_out; state <= VAL_X3;   end // lam=CURVE_A*x^2
                VAL_X3: begin
                    // alu_out = xg * temp = x^3  (VAL_X3 routing uses temp=x^2)
                    // right = x^3 ^ CURVE_A*x^2 ^ CURVE_B = alu_out ^ lam ^ CURVE_B
                    // temp currently holds left side (y^2+xy)
                    if ((temp ^ alu_out ^ lam ^ CURVE_B) == 8'b0) begin
                        bit_idx <= 7; state <= SCAN_BIT;
                    end else begin
                        error <= 1; done <= 1; busy <= 0; state <= IDLE;
                    end
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
                    // alu_out = inv(xr) * yr  (combinational this cycle)
                    // save x1 before xr is overwritten
                    x1_saved <= xr;
                    temp     <= alu_out;
                    state    <= DBL_LAM_2;
                end

                DBL_LAM_2: begin
                    // lam = inv(xr)*yr ^ xr  (XOR with saved xr)
                    lam   <= temp ^ xr;
                    state <= DBL_X3_1;
                end

                DBL_X3_1: begin temp <= alu_out; state <= DBL_X3_2; end  // temp = lam^2
                DBL_X3_2: begin xr   <= alu_out; state <= DBL_Y3_1; end  // xr   = new x3
                DBL_Y3_1: begin temp <= alu_out; state <= DBL_Y3_2; end  // temp = x1^2
                DBL_Y3_2: begin lam  <= alu_out; state <= DBL_Y3_3; end  // lam  = lam*x3
                // yr = x1^2 ^ lam*x3 ^ x3 = temp ^ lam ^ xr
                DBL_Y3_3: begin yr   <= alu_out; state <= ADD_CHK;  end

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
                                // R = G in addition — shouldn't happen in L-R d&a
                                error <= 1'b1; done <= 1'b1; busy <= 1'b0;
                                state <= IDLE;
                            end
                        end else begin
                            state <= ADD_LAM_1;
                        end
                    end else begin
                        state <= NEXT_BIT;
                    end
                end

                ADD_LAM_1: begin temp <= alu_out; state <= ADD_LAM_2; end // temp = xr^xg
                ADD_LAM_2: begin lam  <= alu_out; state <= ADD_X3_1;  end // lam  = inv(xr^xg)*(yr^yg)
                ADD_X3_1:  begin temp <= alu_out; state <= ADD_X3_2;  end // temp = lam^2
                ADD_X3_2:  begin xr   <= alu_out; state <= ADD_Y3_1;  end // xr   = new x3
                ADD_Y3_1:  begin temp <= alu_out; state <= ADD_Y3_2;  end // temp = x1^x3
                ADD_Y3_2:  begin temp <= alu_out; state <= ADD_Y3_3;  end // temp = lam*(x1^x3)
                // yr = lam*(x1^x3) ^ x3 ^ y1
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
