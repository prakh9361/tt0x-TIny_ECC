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

    // Curve Parameter 'a' , 'b' or y^2 + xy = x^3 + ax^2 + b
    localparam CURVE_A = 8'h20;
    localparam CURVE_B = 8'h01;

    // --- Registers ---
    reg [7:0] k;            // Private Key
    reg [7:0] xg, yg;       // Base Point G
    reg [7:0] xr, yr;       // Working Point R (Accumulator)
    reg [7:0] lam, temp;    // Slope and temporary storage
    reg [2:0] bit_idx;      // Tracks which bit of K we are on (6 down to 0)
    reg [7:0] x1_saved;
    reg       r_is_inf;     // Tracks if the working point R is at Infinity

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

    // verification states
    localparam VAL_Y2     = 5'd19;
    localparam VAL_XY     = 5'd20;
    localparam VAL_LEFT   = 5'd21;
    localparam VAL_X2     = 5'd22;
    localparam VAL_X3     = 5'd23;

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
    always @(*) begin
        alu_a = 8'b0; alu_b = 8'b0; alu_op = 2'b00; rom_addr = 8'b0;

        case (state)
            // Doubling Routing: lambda = x1 + (y1 * x1^-1)
            DBL_LAM_1: begin rom_addr = xr; alu_a = inv_out; alu_b = yr; alu_op = 2'b10; end 
            DBL_LAM_2: begin alu_a = temp; alu_b = xr; alu_op = 2'b00; end                   
            DBL_X3_1:  begin alu_a = lam; alu_op = 2'b01; end                                
            DBL_X3_2:  begin alu_a = temp; alu_b = lam ^ CURVE_A; alu_op = 2'b00; end        
            DBL_Y3_1:  begin alu_a = x1_saved; alu_op = 2'b01; end                           
            DBL_Y3_2:  begin alu_a = lam; alu_b = xr; alu_op = 2'b10; end                    
            DBL_Y3_3:  begin alu_a = temp; alu_b = xr; alu_op = 2'b00; end                   
            
            // Addition Routing: lambda = (y1 + y2) * (x1 + x2)^-1
            ADD_LAM_1: begin alu_a = xr; alu_b = xg; alu_op = 2'b00; end                     
            ADD_LAM_2: begin rom_addr = temp; alu_a = inv_out; alu_b = yr ^ yg; alu_op = 2'b10; end 
            ADD_X3_1:  begin alu_a = lam; alu_op = 2'b01; end                                
            ADD_X3_2:  begin alu_a = temp; alu_b = lam ^ xr ^ xg ^ CURVE_A; alu_op = 2'b00; end
            ADD_Y3_1:  begin alu_a = x1_saved; alu_b = temp; alu_op = 2'b00; end             
            ADD_Y3_2:  begin alu_a = lam; alu_b = temp; alu_op = 2'b10; end                  
            ADD_Y3_3:  begin alu_a = temp; alu_b = xr ^ yr; alu_op = 2'b00; end 

            // Point validation routing 
            VAL_Y2:    begin alu_a = yg; alu_op = 2'b01; end                      // y^2
            VAL_XY:    begin alu_a = xg; alu_b = yg; alu_op = 2'b10; end          // x * y
            VAL_LEFT:  begin alu_a = temp; alu_b = lam; alu_op = 2'b00; end       // y^2 + xy
            VAL_X2:    begin alu_a = xg; alu_op = 2'b01; end                      // x^2
            VAL_X3:    begin alu_a = xg; alu_b = lam; alu_op = 2'b10; end         // x^3

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
            r_is_inf <= 0; // Reset infinity flag
        end else begin
            if (load_k) k <= data_in;
            if (load_x) xg <= data_in;
            if (load_y) yg <= data_in;

            case (state)
                IDLE: begin
                    done <= 0;
                    error <= 0;
                    if (start) begin
                        busy <= 1;
                        state <= VAL_Y2; // Jump to validation first
                    end
                end

                // --- Point Validation Sequence ---
                VAL_Y2:   begin temp <= alu_out; state <= VAL_XY; end
                VAL_XY:   begin lam <= alu_out; state <= VAL_LEFT; end
                VAL_LEFT: begin temp <= alu_out; state <= VAL_X2; end   // temp now holds left side (y^2 + xy)
                VAL_X2:   begin lam <= alu_out; state <= VAL_X3; end    // lam now holds x^2
                VAL_X3: begin 
                    // alu_out is x^3. Since a=1, ax^2 = lam. 
                    // Right side: x^3 + ax^2 + b = alu_out ^ lam ^ CURVE_B
                    // Compare Left (temp) == Right
                    if ((temp ^ alu_out ^ lam ^ CURVE_B) == 8'b0) begin
                        bit_idx <= 7;      
                        state <= SCAN_BIT; // Point is valid, start multiplication
                    end else begin
                        error <= 1;        // Point is NOT on the curve
                        done <= 1;
                        busy <= 0;
                        state <= IDLE;
                    end
                end

                SCAN_BIT: begin
                    if (k[bit_idx] == 1'b1) begin
                        xr <= xg;
                        yr <= yg;
                        r_is_inf <= 0; // Initialize to a valid point
                        state <= NEXT_BIT; 
                    end else begin
                        if (bit_idx == 0) begin
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
                DBL_X3_2:  begin xr <= alu_out; state <= DBL_Y3_1; end 
                DBL_Y3_1:  begin temp <= alu_out; state <= DBL_Y3_2; end
                DBL_Y3_2:  begin temp <= temp ^ alu_out; state <= DBL_Y3_3; end 
                DBL_Y3_3:  begin yr <= alu_out; state <= ADD_CHK; end           

                // --- Point Addition Sequence ---
                ADD_CHK: begin
                    x1_saved <= xr; 
                    if (k[bit_idx] == 1'b1) begin
                        // [NEW] Handle Infinity Rules for Addition
                        if (r_is_inf) begin
                            // Infinity + G = G
                            xr <= xg;
                            yr <= yg;
                            r_is_inf <= 1'b0;
                            state <= NEXT_BIT;
                        end else if (xr == xg) begin
                            // Prevent Division by Zero
                            if (yr == (xg ^ yg)) begin
                                // R == -G -> Result is Infinity
                                r_is_inf <= 1'b1;
                                state <= NEXT_BIT;
                            end else begin
                                // R == G -> Requires doubling, which shouldn't happen 
                                // naturally in Left-to-Right Double-and-Add. 
                                // Halt and throw error to prevent corruption.
                                error <= 1'b1;
                                done <= 1'b1;
                                busy <= 1'b0;
                                state <= IDLE;
                            end
                        end else begin
                            state <= ADD_LAM_1; // Safe to do normal math
                        end
                    end else begin
                        state <= NEXT_BIT;
                    end
                end
                
                ADD_LAM_1: begin temp <= alu_out; state <= ADD_LAM_2; end
                ADD_LAM_2: begin lam <= alu_out; state <= ADD_X3_1; end
                ADD_X3_1:  begin temp <= alu_out; state <= ADD_X3_2; end
                ADD_X3_2:  begin xr <= alu_out; temp <= alu_out; state <= ADD_Y3_1; end 
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
                        
                        // [NEW] Handle Infinity Rules for Doubling
                        if (r_is_inf) begin
                            // 2 * Infinity = Infinity. Skip ALU.
                            state <= ADD_CHK;
                        end else if (xr == 8'b0) begin
                            // Tangent is vertical, 2R = Infinity. Skip ALU.
                            r_is_inf <= 1'b1;
                            state <= ADD_CHK;
                        end else begin
                            state <= DBL_LAM_1; // Safe to do normal math
                        end
                    end
                end

                DONE_STATE: begin
                    busy <= 0;
                    done <= 1;
                    // [NEW] If the final mathematical result is Infinity, output 0 and flag error
                    if (r_is_inf) begin
                        error <= 1;
                        result_x <= 0;
                        result_y <= 0;
                    end else begin
                        result_x <= xr;
                        result_y <= yr;
                    end
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule
