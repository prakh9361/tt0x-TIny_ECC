`default_nettype none

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

    // Curve Parameter 'a' for y^2 + xy = x^3 + ax^2 + b
    localparam CURVE_A = 8'h01; 

    // --- Registers ---
    reg [7:0] k;            // Private Key
    reg [7:0] xg, yg;       // Base Point G
    reg [7:0] xr, yr;       // Working Point R (Accumulator)
    reg [7:0] lam, temp;    // Slope and temporary storage
    reg [2:0] bit_idx;      // Tracks which bit of K we are on (6 down to 0)
    reg [7:0] x1_saved;

    // --- FSM States ---
    localparam IDLE       = 5'd0;
    localparam SCAN_BIT   = 5'd18;
    // Doubling States (R = 2R)
    localparam DBL_LAM_1  = 5'd1;
    localparam DBL_LAM_2  = 5'd2;
    localparam DBL_X3_1   = 5'd3;
    localparam DBL_X3_2   = 5'd4;
    localparam DBL_Y3_1   = 5'd5;
    localparam DBL_Y3_2   = 5'd6;
    localparam DBL_Y3_3   = 5'd7;
    // Addition States (R = R + G)
    localparam ADD_CHK    = 5'd8;
    localparam ADD_LAM_1  = 5'd9;
    localparam ADD_LAM_2  = 5'd10;
    localparam ADD_X3_1   = 5'd11;
    localparam ADD_X3_2   = 5'd12;
    localparam ADD_Y3_1   = 5'd13;
    localparam ADD_Y3_2   = 5'd14;
    localparam ADD_Y3_3   = 5'd15;
    // Loop Control
    localparam NEXT_BIT   = 5'd16;
    localparam DONE_STATE = 5'd17;

    reg [4:0] state;

    // --- ALU & ROM Instantiation ---
    reg  [7:0] alu_a, alu_b;
    reg  [1:0] alu_op;
    wire [7:0] alu_out;
    
    alu_gf2_8 alu (
        .a(alu_a), .b(alu_b), .op(alu_op), .result(alu_out)
    );

    reg  [7:0] rom_addr;
    wire [7:0] inv_out;
    
    inv_rom rom (
        .addr(rom_addr), .inv_out(inv_out)
    );

    // --- ALU & ROM Combinational Routing ---
    // Instead of instantiating multiple ALUs, we multiplex inputs to the single ALU based on the state.
    always @(*) begin
        // Default assignments to prevent latches
        alu_a = 8'b0; alu_b = 8'b0; alu_op = 2'b00; rom_addr = 8'b0;

        case (state)
            // Doubling Routing: lambda = x1 + (y1 * x1^-1)
            DBL_LAM_1: begin rom_addr = xr; alu_a = inv_out; alu_b = yr; alu_op = 2'b10; end // inv(x)*y
            DBL_LAM_2: begin alu_a = temp; alu_b = xr; alu_op = 2'b00; end                   // (inv(x)*y) + x
            DBL_X3_1:  begin alu_a = lam; alu_op = 2'b01; end                                // lam^2
            DBL_X3_2:  begin alu_a = temp; alu_b = lam ^ CURVE_A; alu_op = 2'b00; end        // lam^2 + lam + a
            DBL_Y3_1:  begin alu_a = x1_saved; alu_op = 2'b01; end                           // x1^2 <- was xr
            DBL_Y3_2:  begin alu_a = lam; alu_b = xr; alu_op = 2'b10; end                    // lam * x3 (using xr as x3 is updated later)
            // [FIXED] Removed the erroneous "^ yr" from the doubling Y3 calculation
            DBL_Y3_3:  begin alu_a = temp; alu_b = xr; alu_op = 2'b00; end                   // x1^2 + lam*x3 + x3
            
            // Addition Routing: lambda = (y1 + y2) * (x1 + x2)^-1
            ADD_LAM_1: begin alu_a = xr; alu_b = xg; alu_op = 2'b00; end                     // x1 + x2
            ADD_LAM_2: begin rom_addr = temp; alu_a = inv_out; alu_b = yr ^ yg; alu_op = 2'b10; end // inv(x1+x2) * (y1+y2)
            ADD_X3_1:  begin alu_a = lam; alu_op = 2'b01; end                                // lam^2
            ADD_X3_2:  begin alu_a = temp; alu_b = lam ^ xr ^ xg ^ CURVE_A; alu_op = 2'b00; end
            // [FIXED] Changed alu_a from 'xr' to 'x1_saved' since xr gets overwritten by x3 in ADD_X3_2
            ADD_Y3_1:  begin alu_a = x1_saved; alu_b = temp; alu_op = 2'b00; end             // x1 + x3
            ADD_Y3_2:  begin alu_a = lam; alu_b = temp; alu_op = 2'b10; end                  // lam * (x1 + x3)
            ADD_Y3_3:  begin alu_a = temp; alu_b = xr ^ yr; alu_op = 2'b00; end              // lam*(x1+x3) + x3 + y1 (xr holds x3 here)

            default: ;
        endcase
    end

    // --- Main Synchronous FSM ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            k <= 0; xg <= 0; yg <= 0; xr <= 0; yr <= 0;
            result_x <= 0; result_y <= 0;
            done <= 0; busy <= 0; error <= 0;
            bit_idx <= 0;
        end else begin
            // Data Loading (Independent of state)
            if (load_k) k <= data_in;
            if (load_x) xg <= data_in;
            if (load_y) yg <= data_in;

            case (state)
                IDLE: begin
                    done <= 0; error <= 0;
                    if (start) begin
                        busy <= 1;
                        bit_idx <= 7;      // Start scanning from the absolute top bit
                        state <= SCAN_BIT; // Go to our new scanning state
                    end
                end

                // --- New State: Find the first '1' ---
                SCAN_BIT: begin
                    if (k[bit_idx] == 1'b1) begin
                        // Found the MSB! Initialize R = G here.
                        xr <= xg;
                        yr <= yg;
                        // Now drop into the normal loop for the REST of the bits
                        state <= NEXT_BIT; 
                    end else begin
                        // It was a 0. Keep scanning.
                        if (bit_idx == 0) begin
                            // The entire key was 0. This results in the Point at Infinity.
                            // We don't have infinity hardware, so throw an error.
                            error <= 1; 
                            done <= 1;
                            busy <= 0;
                            state <= IDLE;
                        end else begin
                            bit_idx <= bit_idx - 1;
                        end
                    end
                end

                // --- Point Doubling Sequence ---
                DBL_LAM_1: begin temp <= alu_out; state <= DBL_LAM_2; end
                DBL_LAM_2: begin lam <= alu_out; state <= DBL_X3_1; end
                DBL_X3_1:  begin temp <= alu_out; state <= DBL_X3_2; end
                DBL_X3_2:  begin xr <= alu_out; state <= DBL_Y3_1; end // x3 stored in xr
                DBL_Y3_1:  begin temp <= alu_out; state <= DBL_Y3_2; end
                DBL_Y3_2:  begin temp <= temp ^ alu_out; state <= DBL_Y3_3; end // x1^2 + lam*x3
                DBL_Y3_3:  begin yr <= alu_out; state <= ADD_CHK; end           // y3 stored in yr

                // --- Point Addition Sequence ---
                ADD_CHK: begin
                    // [FIXED] Save the old x1 before it gets permanently overwritten in ADD_X3_2
                    x1_saved <= xr; 
                    if (k[bit_idx] == 1'b1) state <= ADD_LAM_1;
                    else state <= NEXT_BIT;
                end
                ADD_LAM_1: begin temp <= alu_out; state <= ADD_LAM_2; end
                ADD_LAM_2: begin lam <= alu_out; state <= ADD_X3_1; end
                ADD_X3_1:  begin temp <= alu_out; state <= ADD_X3_2; end
                ADD_X3_2:  begin xr <= alu_out; temp <= alu_out; state <= ADD_Y3_1; end // temp holds x3 for Y calc
                ADD_Y3_1:  begin temp <= alu_out; state <= ADD_Y3_2; end
                ADD_Y3_2:  begin temp <= alu_out; state <= ADD_Y3_3; end
                ADD_Y3_3:  begin yr <= alu_out; state <= NEXT_BIT; end

                // --- Loop Logic ---
                NEXT_BIT: begin
                    if (bit_idx == 0) begin
                        state <= DONE_STATE;
                    end else begin
                        bit_idx <= bit_idx - 1;
                        x1_saved <= xr;
                        state <= DBL_LAM_1;
                    end
                end

                DONE_STATE: begin
                    busy <= 0;
                    done <= 1;
                    result_x <= xr;
                    result_y <= yr;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule
