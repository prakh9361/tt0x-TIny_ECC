`default_nettype none

// ============================================================
// ECC Core GF(2^8)
// Curve: y^2 + xy = x^3 + 0x20*x^2 + 0x01
// Field: GF(2^8), poly = 0x11B (x^8+x^4+x^3+x+1)
// n=113, G=(143,41), h=2
//
// Inversion via Itoh-Tsujii:
//   x^-1 = x^(2^8-2) = (x^(2^7-1))^2
//   Chain: x^2, x^4, x^6, x^7, x^3, x^12, x^15, x^112, x^127, x^254
//   Cost: 4 muls + 8 squarings = 12 ALU states (no ROM needed)
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

    // --- Datapath Registers ---
    reg [7:0] k;
    reg [7:0] xg, yg;
    reg [7:0] xr, yr;
    reg [7:0] lam, temp;
    reg [2:0] bit_idx;
    reg [7:0] x1_saved;
    reg       r_is_inf;

    // Itoh-Tsujii intermediate registers
    reg [7:0] inv_in;           // value to invert
    reg [7:0] inv_x2;           // a^2
    reg [7:0] inv_x7;           // a^7  = a^(2^3-1)
    reg [7:0] inv_x15;          // a^15 = a^(2^4-1)
    reg [7:0] inv_x3_saved;     // a^3  (needed for x^15 step)
    reg [5:0] inv_return;       // return address after inversion

    // --- FSM State Encoding (6-bit) ---

    // Main flow
    localparam IDLE       = 6'd0;
    localparam SCAN_BIT   = 6'd18;
    localparam NEXT_BIT   = 6'd16;
    localparam DONE_STATE = 6'd17;

    // Validation
    localparam VAL_Y2     = 6'd19;
    localparam VAL_XY     = 6'd20;
    localparam VAL_LEFT   = 6'd21;
    localparam VAL_X2     = 6'd22;
    localparam VAL_AX2    = 6'd23;
    localparam VAL_X3     = 6'd24;

    // Point doubling
    // lambda = xr + yr*inv(xr),  x3 = lam^2+lam+a,  y3 = x1^2+lam*x3+x3
    localparam DBL_INV    = 6'd1;   // setup inversion of xr
    localparam DBL_LAM_2  = 6'd2;   // lam = temp(=inv)*yr ^ xr
    localparam DBL_X3_1   = 6'd3;   // temp = lam^2
    localparam DBL_X3_2   = 6'd4;   // xr   = lam^2 ^ lam ^ CURVE_A
    localparam DBL_Y3_1   = 6'd5;   // temp = x1_saved^2
    localparam DBL_Y3_2   = 6'd6;   // lam  = lam * xr(new x3)
    localparam DBL_Y3_3   = 6'd7;   // yr   = temp ^ lam ^ xr

    // Point addition
    // lambda = (yr^yg)*inv(xr^xg),  x3 = lam^2+lam+xr+xg+a,  y3 = lam*(x1+x3)+x3+y1
    localparam ADD_CHK    = 6'd8;
    localparam ADD_INV    = 6'd9;   // setup inversion of (xr^xg)
    localparam ADD_LAM_2  = 6'd10;  // lam = temp(=inv)*(yr^yg)
    localparam ADD_X3_1   = 6'd11;  // temp = lam^2
    localparam ADD_X3_2   = 6'd12;  // xr   = new x3
    localparam ADD_Y3_1   = 6'd13;  // temp = x1_saved ^ xr(new x3)
    localparam ADD_Y3_2   = 6'd14;  // temp = lam * temp
    localparam ADD_Y3_3   = 6'd15;  // yr   = temp ^ xr ^ yr(old y1)

    // Itoh-Tsujii inversion chain
    // After INV_DONE, result (a^254 = a^-1) is in temp
    localparam INV_1      = 6'd32;   // temp = a^2          [sq inv_in]
    localparam INV_2      = 6'd33;   // temp = a^4          [sq inv_x2]
    localparam INV_3      = 6'd34;   // temp = a^6          [mul temp * inv_x2]
    localparam INV_4      = 6'd35;   // temp = a^7          [mul temp * inv_in]  -> save inv_x7
    localparam INV_5      = 6'd36;   // temp = a^3          [mul inv_in * inv_x2] -> save inv_x3_saved
    localparam INV_6      = 6'd37;   // temp = a^6          [sq temp]
    localparam INV_7      = 6'd38;   // temp = a^12         [sq temp]
    localparam INV_8      = 6'd39;   // temp = a^15         [mul temp * inv_x3_saved] -> save inv_x15
    localparam INV_9      = 6'd40;   // temp = a^14         [sq inv_x7]
    localparam INV_10     = 6'd41;   // temp = a^28         [sq temp]
    localparam INV_11     = 6'd42;   // temp = a^56         [sq temp]
    localparam INV_12     = 6'd43;   // temp = a^112        [sq temp]
    localparam INV_13     = 6'd44;   // temp = a^127        [mul temp * inv_x15]
    localparam INV_14     = 6'd45;   // temp = a^254 = a^-1 [sq temp]
    localparam INV_DONE   = 6'd46;   // jump to inv_return

    reg [5:0] state;

    // --- ALU Instantiation ---
    reg  [7:0] alu_a, alu_b;
    reg  [1:0] alu_op;
    wire [7:0] alu_out;

    alu_gf2_8 alu (
        .a(alu_a), .b(alu_b), .op(alu_op), .result(alu_out)
    );

    // --- Combinational ALU Routing ---
    always @(*) begin
        alu_a = 8'b0; alu_b = 8'b0; alu_op = 2'b00;

        case (state)
            // Doubling
            DBL_LAM_2: begin alu_a = temp;        alu_b = yr;             alu_op = 2'b10; end // inv*yr
            DBL_X3_1:  begin alu_a = lam;                                 alu_op = 2'b01; end // lam^2
            DBL_X3_2:  begin alu_a = temp;        alu_b = lam ^ CURVE_A;  alu_op = 2'b00; end // lam^2^lam^a
            DBL_Y3_1:  begin alu_a = x1_saved;                            alu_op = 2'b01; end // x1^2
            DBL_Y3_2:  begin alu_a = lam;         alu_b = xr;             alu_op = 2'b10; end // lam*x3
            DBL_Y3_3:  begin alu_a = temp;        alu_b = lam ^ xr;       alu_op = 2'b00; end // x1^2^lam*x3^x3

            // Addition
            ADD_LAM_2: begin alu_a = temp;        alu_b = yr ^ yg;        alu_op = 2'b10; end // inv*(yr^yg)
            ADD_X3_1:  begin alu_a = lam;                                 alu_op = 2'b01; end // lam^2
            ADD_X3_2:  begin alu_a = temp;        alu_b = lam^xr^xg^CURVE_A; alu_op = 2'b00; end
            ADD_Y3_1:  begin alu_a = x1_saved;    alu_b = xr;             alu_op = 2'b00; end // x1^x3
            ADD_Y3_2:  begin alu_a = lam;         alu_b = temp;           alu_op = 2'b10; end // lam*(x1^x3)
            ADD_Y3_3:  begin alu_a = alu_out;     alu_b = xr ^ yr;        alu_op = 2'b00; end // ^x3^y1

            // Validation
            VAL_Y2:    begin alu_a = yg;                                  alu_op = 2'b01; end // y^2
            VAL_XY:    begin alu_a = xg;          alu_b = yg;             alu_op = 2'b10; end // x*y
            VAL_LEFT:  begin alu_a = temp;        alu_b = lam;            alu_op = 2'b00; end // y^2+xy
            VAL_X2:    begin alu_a = xg;                                  alu_op = 2'b01; end // x^2
            VAL_AX2:   begin alu_a = CURVE_A;     alu_b = temp;           alu_op = 2'b10; end // a*x^2
            VAL_X3:    begin alu_a = xg;          alu_b = temp;           alu_op = 2'b10; end // x^3

            // Itoh-Tsujii
            INV_1:  begin alu_a = inv_in;                                 alu_op = 2'b01; end // a^2
            INV_2:  begin alu_a = inv_x2;                                 alu_op = 2'b01; end // a^4
            INV_3:  begin alu_a = temp;           alu_b = inv_x2;         alu_op = 2'b10; end // a^6
            INV_4:  begin alu_a = temp;           alu_b = inv_in;         alu_op = 2'b10; end // a^7
            INV_5:  begin alu_a = inv_in;         alu_b = inv_x2;         alu_op = 2'b10; end // a^3
            INV_6:  begin alu_a = temp;                                   alu_op = 2'b01; end // a^6
            INV_7:  begin alu_a = temp;                                   alu_op = 2'b01; end // a^12
            INV_8:  begin alu_a = temp;           alu_b = inv_x3_saved;   alu_op = 2'b10; end // a^15
            INV_9:  begin alu_a = inv_x7;                                 alu_op = 2'b01; end // a^14
            INV_10: begin alu_a = temp;                                   alu_op = 2'b01; end // a^28
            INV_11: begin alu_a = temp;                                   alu_op = 2'b01; end // a^56
            INV_12: begin alu_a = temp;                                   alu_op = 2'b01; end // a^112
            INV_13: begin alu_a = temp;           alu_b = inv_x15;        alu_op = 2'b10; end // a^127
            INV_14: begin alu_a = temp;                                   alu_op = 2'b01; end // a^254

            default: ;
        endcase
    end

    // --- Synchronous FSM ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            k            <= 0; xg <= 0; yg <= 0; xr <= 0; yr <= 0;
            result_x     <= 0; result_y <= 0;
            done         <= 0; busy <= 0; error <= 0;
            bit_idx      <= 0; r_is_inf <= 0;
            lam          <= 0; temp <= 0; x1_saved <= 0;
            inv_in       <= 0; inv_x2 <= 0; inv_x7 <= 0;
            inv_x15      <= 0; inv_x3_saved <= 0;
            inv_return   <= IDLE;
        end else begin
            if (load_k) k  <= data_in;
            if (load_x) xg <= data_in;
            if (load_y) yg <= data_in;

            case (state)

                // ===========================================================
                IDLE: begin
                    done <= 0; error <= 0;
                    if (start) begin busy <= 1; state <= VAL_Y2; end
                end

                // ===========================================================
                // Point validation
                VAL_Y2:   begin temp <= alu_out; state <= VAL_XY;   end  // temp = y^2
                VAL_XY:   begin lam  <= alu_out; state <= VAL_LEFT; end  // lam  = x*y
                VAL_LEFT: begin temp <= alu_out; state <= VAL_X2;   end  // temp = y^2+xy (left)
                VAL_X2:   begin temp <= alu_out; state <= VAL_AX2;  end  // temp = x^2
                VAL_AX2:  begin lam  <= alu_out; state <= VAL_X3;   end  // lam  = CURVE_A*x^2
                VAL_X3: begin
                    // alu_out=x*temp=x^3, lam=CURVE_A*x^2
                    // right = x^3 ^ CURVE_A*x^2 ^ CURVE_B
                    // temp holds left side (y^2+xy)
                    if ((temp ^ alu_out ^ lam ^ CURVE_B) == 8'b0) begin
                        bit_idx <= 7; state <= SCAN_BIT;
                    end else begin
                        error <= 1; done <= 1; busy <= 0; state <= IDLE;
                    end
                end

                // ===========================================================
                // Scan for MSB of k
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

                // ===========================================================
                // Point Doubling

                // Trigger inversion of xr, then continue at DBL_LAM_2
                DBL_INV: begin
                    x1_saved   <= xr;
                    inv_in     <= xr;
                    inv_return <= DBL_LAM_2;
                    state      <= INV_1;
                end

                // temp = inv(xr), now compute lam = inv(xr)*yr ^ xr
                DBL_LAM_2: begin
                    // alu routing: temp*yr -> result in alu_out next cycle
                    // but alu_out is already valid this cycle (combinational)
                    // so we capture: lam = alu_out ^ xr
                    lam   <= alu_out ^ xr;
                    state <= DBL_X3_1;
                end

                DBL_X3_1: begin temp <= alu_out; state <= DBL_X3_2; end  // temp = lam^2
                DBL_X3_2: begin
                    xr    <= alu_out;   // xr = lam^2 ^ lam ^ CURVE_A
                    state <= DBL_Y3_1;
                end

                DBL_Y3_1: begin temp <= alu_out; state <= DBL_Y3_2; end  // temp = x1_saved^2
                DBL_Y3_2: begin lam  <= alu_out; state <= DBL_Y3_3; end  // lam  = lam*x3(new xr)

                // yr = x1^2 ^ lam*x3 ^ x3  = temp ^ lam ^ xr
                DBL_Y3_3: begin yr <= alu_out; state <= ADD_CHK; end

                // ===========================================================
                // Point Addition

                ADD_CHK: begin
                    x1_saved <= xr;
                    if (k[bit_idx] == 1'b1) begin
                        if (r_is_inf) begin
                            xr <= xg; yr <= yg; r_is_inf <= 1'b0;
                            state <= NEXT_BIT;
                        end else if (xr == xg) begin
                            if (yr == (xg ^ yg)) begin
                                r_is_inf <= 1'b1; state <= NEXT_BIT;
                            end else begin
                                // R == G in add — should not happen in L-to-R d&a
                                error <= 1'b1; done <= 1'b1; busy <= 1'b0;
                                state <= IDLE;
                            end
                        end else begin
                            state <= ADD_INV;
                        end
                    end else begin
                        state <= NEXT_BIT;
                    end
                end

                // Trigger inversion of (xr^xg), continue at ADD_LAM_2
                ADD_INV: begin
                    inv_in     <= xr ^ xg;
                    inv_return <= ADD_LAM_2;
                    state      <= INV_1;
                end

                // temp = inv(xr^xg), lam = temp*(yr^yg)
                ADD_LAM_2: begin lam  <= alu_out; state <= ADD_X3_1; end
                ADD_X3_1:  begin temp <= alu_out; state <= ADD_X3_2; end  // temp = lam^2
                ADD_X3_2:  begin xr   <= alu_out; state <= ADD_Y3_1; end  // xr   = x3
                ADD_Y3_1:  begin temp <= alu_out; state <= ADD_Y3_2; end  // temp = x1^x3
                ADD_Y3_2:  begin temp <= alu_out; state <= ADD_Y3_3; end  // temp = lam*(x1^x3)
                // yr = lam*(x1^x3) ^ x3 ^ y1  — alu_out computed combinationally
                ADD_Y3_3:  begin yr   <= alu_out; state <= NEXT_BIT; end

                // ===========================================================
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
                            state <= DBL_INV;
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

                // ===========================================================
                // Itoh-Tsujii Inversion
                // Input:  inv_in  (set by caller before jumping to INV_1)
                // Output: temp    (holds a^-1 when INV_DONE fires)
                // Return: jumps to inv_return after INV_DONE
                //
                // Exponent chain (verified):
                //   INV_1 : a^2                        [sq]
                //   INV_2 : a^4                        [sq]
                //   INV_3 : a^6  = a^4 * a^2           [mul]
                //   INV_4 : a^7  = a^6 * a              [mul]  -> inv_x7
                //   INV_5 : a^3  = a   * a^2            [mul]  -> inv_x3_saved
                //   INV_6 : a^6  = a^3 ^ 2              [sq]
                //   INV_7 : a^12 = a^6 ^ 2              [sq]
                //   INV_8 : a^15 = a^12 * a^3           [mul]  -> inv_x15
                //   INV_9 : a^14 = a^7  ^ 2             [sq]
                //   INV_10: a^28 = a^14 ^ 2             [sq]
                //   INV_11: a^56 = a^28 ^ 2             [sq]
                //   INV_12: a^112= a^56 ^ 2             [sq]
                //   INV_13: a^127= a^112 * a^15         [mul]
                //   INV_14: a^254= a^127 ^ 2            [sq]  -> a^-1

                INV_1:  begin
                    inv_x2 <= alu_out;
                    temp   <= alu_out;
                    state  <= INV_2;
                end
                INV_2:  begin temp <= alu_out; state <= INV_3;  end
                INV_3:  begin temp <= alu_out; state <= INV_4;  end
                INV_4:  begin
                    inv_x7 <= alu_out;
                    temp   <= alu_out;
                    state  <= INV_5;
                end
                INV_5:  begin
                    inv_x3_saved <= alu_out;
                    temp         <= alu_out;
                    state        <= INV_6;
                end
                INV_6:  begin temp <= alu_out; state <= INV_7;  end
                INV_7:  begin temp <= alu_out; state <= INV_8;  end
                INV_8:  begin
                    inv_x15 <= alu_out;
                    temp    <= alu_out;
                    state   <= INV_9;
                end
                INV_9:  begin temp <= alu_out; state <= INV_10; end
                INV_10: begin temp <= alu_out; state <= INV_11; end
                INV_11: begin temp <= alu_out; state <= INV_12; end
                INV_12: begin temp <= alu_out; state <= INV_13; end
                INV_13: begin temp <= alu_out; state <= INV_14; end
                INV_14: begin temp <= alu_out; state <= INV_DONE; end

                INV_DONE: begin
                    // temp = inv_in^-1, return to caller
                    state <= inv_return;
                end

                default: state <= IDLE;

            endcase
        end
    end

endmodule
