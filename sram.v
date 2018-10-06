`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    23:56:11 03/19/2007 
// Design Name: 
// Module Name:    sram 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////

//A 64Kx16B synchronus SRAM module
module sram(WE, A, Din, Dout, CLK);
	input WE;
	input CLK;
	input [15:0] A;
	input [15:0] Din;
	output [15:0] Dout;
	reg [15:0] mem [0:65535];

	assign Dout = mem[A];

	always @(posedge CLK)
	begin
		if (WE)
			mem[A] <= Din;
	end
endmodule
