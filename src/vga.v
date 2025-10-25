//`timescale 1ns / 1ps
//`default_nettype none

// 640x480 @60Hz
// input clk 25Mhz
module vga(
	input clk, 		
	input reset,
	output HS, VS,
	output [9:0] x,
	output [9:0] y,
	output blank
	);
	
	// Counter Registers, two each for buffering to avoid glitches
	reg [9:0] xc, yc;
	
	// Horizontal 640 + fp 16 + HS 96 + bp 48 = 800 pixel clocks
	// Vertical, 480 + fp 10 lines + VS 2 lines + bp 33 lines = 525 lines
	assign blank = ((xc > 639) | (yc > 479));
	assign HS = ~ (xc > (655) && xc < (752));
	assign VS = ~ ((yc > 489) & (yc < 492));
	
	assign x = xc;
	assign y = yc;
	//assign newframe = ((xc == 0) && (yc == 1) && (prescaler == 0));
	//assign endframe = ((xc == 0) && (yc == 480) && (prescaler == 0));
	
	always @(posedge clk or posedge reset) begin
		if (reset) begin
			xc <= 0;
			yc <= 0;
		end
		else begin
			if (xc == 799) begin
				xc <= 0;
				yc <= yc + 1;
			end else begin
				xc <= xc + 1;
			end
			if ((yc == 524) && (xc == 799)) begin
				yc <= 0;
			end
		end
	end
	
endmodule


