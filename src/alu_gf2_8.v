`default_nettype none

module alu_gf2_8 (
    input  wire [7:0] a,
    input  wire [7:0] b,
    input  wire [1:0] op,     // 00: Add, 01: Square, 10: Mult
    output reg  [7:0] result
);

    // Irreducible polynomial: x^8 + x^4 + x^3 + x + 1 (AES standard)
    localparam POLY = 8'h1B; // The 9th bit (x^8) is implicit in the reduction

    integer i;
    reg [7:0] v;
    reg [7:0] p;

    always @(*) begin
        case (op)
            2'b00: begin
                // Addition and Subtraction in GF(2^m) are identical: just XOR
                result = a ^ b;
            end
            
            2'b01, 2'b10: begin
                // Combinational Shift-and-XOR Multiplication
                // If squaring (01), we multiply 'a' by 'a'. Otherwise, 'a' by 'b'.
                v = (op == 2'b01) ? a : b; 
                p = 8'b0;
                
                for (i = 0; i < 8; i = i + 1) begin
                    if (a[i]) p = p ^ v;
                    
                    // Shift v left. If the MSB was 1, reduce it using the polynomial
                    if (v[7]) v = (v << 1) ^ POLY;
                    else      v = v << 1;
                end
                result = p;
            end
            
            default: result = 8'b0;
        endcase
    end

endmodule
