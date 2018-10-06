`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   20:39:04 04/01/2007
// Design Name:   processor
// Module Name:   /home/tom/Classes/EE480/project2/processor_test.v
// Project Name:  project2
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: processor
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module processor_test_v;
	integer fd;
	integer maddr;
	
	// Inputs
	reg reset;
	reg clk;
	wire [15:0] code_mem_data;
	wire [15:0] data_mem_out;

	// Outputs
	wire [15:0] code_mem_addr;
	wire data_mem_write;
	wire [15:0] data_mem_addr;
	wire [15:0] data_mem_in;
	wire halt;

	sram code_ram (
		.WE(1'b0),
		.A(code_mem_addr), 
		.Din(16'b0), 
		.Dout(code_mem_data),
		.CLK(clk)
	);
	
	sram data_ram (
		.WE(data_mem_write),
		.A(data_mem_addr), 
		.Din(data_mem_in), 
		.Dout(data_mem_out),
		.CLK(clk)
	);

	// Instantiate the Unit Under Test (UUT)
	processor uut (
		.code_mem_addr(code_mem_addr), 
		.code_mem_data(code_mem_data),
		.data_mem_write(data_mem_write),
		.data_mem_addr(data_mem_addr),
		.data_mem_in(data_mem_in),
		.data_mem_out(data_mem_out),
		.halt(halt),
		.reset(reset), 
		.clk(clk)
	);

	always begin
		#25 clk = 1'b1;
		#25 clk = 1'b0;
	end

	initial begin
		$readmemh("code.mem", code_ram.mem);
		$readmemh("data.mem", data_ram.mem);

		// Initialize Inputs
		reset = 1;
		clk = 0;

		// Wait 40 ns for global reset to finish (one rising edge, plus offset)
		#40;
      reset = 0;
		
		@(posedge halt)
		
		@(negedge halt)
			fd = $fopen("result.mem");
			for(maddr = 0 ; maddr < 65536 ; maddr = maddr + 1)
				$fwrite(fd, "%04x\n", data_ram.mem[maddr]);
			$fclose(fd);
			
			$stop;
	end
endmodule
