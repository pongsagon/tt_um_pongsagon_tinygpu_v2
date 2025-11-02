//`timescale 1ns / 1ps

// white, pink, cyan, green
// color[5:0] R1 R2 G1 G2 B1 B2

module color_palette (
    input wire [3:0]  spi_data,
    output wire [5:0] color
);

  reg [5:0] mem;
  assign color = mem;

  always @(*) begin
    case(spi_data)
      0: mem = 6'b000000;
      1: mem = 6'b010101;
      2: mem = 6'b101010;
      3: mem = 6'b111111;

      4: mem = 6'b000000;
      5: mem = 6'b010001;
      6: mem = 6'b100010;
      7: mem = 6'b110011;

      8: mem =  6'b000000;
      9: mem =  6'b000101;
      10: mem = 6'b001010;
      11: mem = 6'b001111;

      12: mem = 6'b000000;
      13: mem = 6'b000100;
      14: mem = 6'b001000;
      15: mem = 6'b001100;
      default: mem = 6'b000000;
    endcase
  end


endmodule
