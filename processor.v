`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    00:06:14 03/20/2007 
// Design Name: 
// Module Name:    processor 
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

module processor(code_mem_addr, code_mem_data, data_mem_write, data_mem_addr,
					  data_mem_in, data_mem_out, halt, reset, clk);
	parameter n = 16;
	input clk;
	input reset;
	input [n-1:0] code_mem_data;	//The instruction
	input [n-1:0] data_mem_out;	//The word from data memory at data_mem_addr
	
	output [n-1:0] code_mem_addr;	//Address at which we fetch the next instruction
	output data_mem_write;			//Signal to write to memory
	output [n-1:0] data_mem_addr;	//Address to data memory
	output [n-1:0] data_mem_in;	//Word to write to data memory
	output halt;

	//Signals entering IF stage
	wire squash_if, stall_if;

	//Signals entering ID stage
	wire [n-1:0] reg_rs_in_id;
	wire [n-1:0] rs_in_id;
	wire [n-1:0] reg_rd_in_id;
	wire [n-1:0] rd_in_id;
	
	wire [n-1:0] instr_id;
	wire [n-1:0] pc_id;
	
	wire ext_ld_id, stall_id, squash_id;
	
	//Signals leaving ID stage
	wire [n-1:0] id_target;
	wire id_is_extended, id_is_control, id_is_taken_control;
	wire id_is_halt;
	
	wire id_alu_en;
	wire [3:0] id_alu_op;
	
	wire id_mem_ld, id_mem_st, id_wb_en;

	wire [3:0] id_rd_read_sel;
	wire [3:0] id_rd_write_sel;
	wire [3:0] id_rs_sel;
	
	wire [n-1:0] id_rs_out;
	wire [n-1:0] id_rd_out;
	
	//Signals entering EX stage
	wire alu_en_ex;
	wire [3:0] alu_op_ex;
	
	wire [n-1:0] id_rs_ex;
	wire [n-1:0] id_rd_ex;
		
	wire [n-1:0] rd_ex;
	wire [n-1:0] rs_ex;
	
	wire [3:0] rd_sel_ex;
	wire [3:0] rs_sel_ex;
	
	wire mem_ld_ex, mem_st_ex, wb_en_ex;
	
	wire squash_ex;
	
	//Signals leaving EX stage
	wire [n-1:0] ex_rd;
	
	//Signals entering MEM stage
	wire [n-1:0] ex_rs_mem;
	wire [n-1:0] ex_rd_mem;
	wire [n-1:0] rd_mem;
	wire [n-1:0] rs_mem;
	
	wire [3:0] rd_sel_mem;
	wire [3:0] rs_sel_mem;
	
	wire mem_ld_mem, mem_st_mem, wb_en_mem;
	
	//Signals leaving MEM stage
	wire [n-1:0] mem_rd;
	
	//Signals entering WB stage
	wire wb_en_wb;
	wire [3:0] rd_sel_wb;
	wire [n-1:0] rd_wb;
	
	//Stuff for value forwarding
	wire vfu_stall_at_id, vfu_stall_at_ex;

	wire vf_id_rd_from_ex, vf_id_rd_from_mem, vf_id_rs_from_wb, vf_id_rd_from_wb;
	wire vf_ex_rs_from_mem, vf_ex_rd_from_mem, vf_ex_rs_from_wb, vf_ex_rd_from_wb;
	wire vf_mem_rs_from_wb, vf_mem_rd_from_wb;

	register_file reg_file(id_rs_sel, id_rd_read_sel, rd_sel_wb, reg_rs_in_id,
												 reg_rd_in_id, rd_wb, wb_en_wb, clk);

	//Value forwarding unit. This thing is HUGE
	value_forwarding_control VFU (
        .ctrl_in_id(id_is_control),
        .li_in_ex(wb_en_ex & ~alu_en_ex & ~mem_ld_ex),
        .li_in_mem(wb_en_mem & ~mem_ld_mem),
        .li_in_wb(wb_en_wb),
        .alu_in_ex(alu_en_ex),
        .ld_in_ex(mem_ld_ex),
        .ld_in_mem(mem_ld_mem),
        .st_in_mem(mem_st_mem),
		  .rs_sel_id(id_rs_sel),
        .rs_sel_ex(rs_sel_ex),
        .rs_sel_mem(rs_sel_mem),
        .rd_sel_id(id_rd_read_sel),
        .rd_sel_ex(rd_sel_ex),
        .rd_sel_mem(rd_sel_mem),
        .rd_sel_wb(rd_sel_wb),
        .id_rd_from_ex(vf_id_rd_from_ex),
        .id_rd_from_mem(vf_id_rd_from_mem),
        .id_rs_from_wb(vf_id_rs_from_wb),
        .id_rd_from_wb(vf_id_rd_from_wb),
        .ex_rs_from_mem(vf_ex_rs_from_mem),
        .ex_rd_from_mem(vf_ex_rd_from_mem),
        .ex_rs_from_wb(vf_ex_rs_from_wb),
        .ex_rd_from_wb(vf_ex_rd_from_wb),
        .mem_rs_from_wb(vf_mem_rs_from_wb),
        .mem_rd_from_wb(vf_mem_rd_from_wb),
        .stall_at_id(vfu_stall_at_id),
        .stall_at_ex(vfu_stall_at_ex)
	);

	//The control unit for the beginning of the pipeline
	flow_control pipeline_control(id_is_extended, id_is_taken_control, id_is_halt, 
										   vfu_stall_at_id, vfu_stall_at_ex, stall_if,
										   stall_id, squash_if, squash_id, squash_ex, 
										   ext_ld_id, halt, reset, clk);

	//==========================================================================
	//		INSTRUCTION FETCH
	//==========================================================================
	//IF stage. Load the pc with the target address only if the control_is_taken
	// signal is generated with a real instruction in ID. i.e. not one that is 
	// waiting for a value, or the second byte of an extended
	IF instr_fetch(code_mem_addr, stall_if, id_is_taken_control & ~vfu_stall_at_id & ~ext_ld_id,
					   id_target, reset, clk);
	
	//Interstage buffers. The instruction is always loaded, but is zeroed out if IF
	// needs to stall. The PC is always forwarded
	interstage_buffer if_id_instr(~stall_if, squash_if, code_mem_data, reset, instr_id, clk);
	interstage_buffer if_id_pc(~stall_if, squash_if, code_mem_addr, reset, pc_id, clk);

	//==========================================================================
	//		INSTRUCTION DECODE
	//==========================================================================	
	//If forwarding from wb, select that, else, if forwarding from mem.... etc
	assign rd_in_id = vf_id_rd_from_wb ? rd_wb : (vf_id_rd_from_mem ? rd_mem : 
												(vf_id_rd_from_ex ? rd_ex : reg_rd_in_id));
	assign rs_in_id = vf_id_rs_from_wb ? rd_wb : reg_rs_in_id;
	
	ID instr_decode(instr_id, pc_id, rs_in_id, rd_in_id, id_is_extended, id_is_halt,
						 id_is_control, id_is_taken_control, id_target, id_alu_en, id_alu_op, id_mem_ld,
						 id_mem_st, id_wb_en, id_rs_sel, id_rd_read_sel, id_rd_write_sel,
						 id_rs_out, id_rd_out);

	//Interstage buffers. If there is an extended 32 bit instruction in ID, then
	// nothing but the RD buffer is loaded, sort of a partial stall. At the same
	// time, the control unit also squashes the instruction coming from ID so that
	// it is not forwarded to EX without an operand.	
	//In addition, if the ext_ld_id signal is high, it is the second cycle of an 
	// extended instruction fetch, so just bypass the entire ID stage and load 
	// the instruction straight from the IF/ID buffer	
	interstage_buffer id_ex_rd(~stall_id | ext_ld_id, squash_id, ext_ld_id ? instr_id : id_rd_out, reset, id_rd_ex, clk);
	interstage_buffer id_ex_rs(~(stall_id | ext_ld_id), squash_id, id_rs_out, reset, id_rs_ex, clk);
	
	defparam id_ex_rd_sel.n = 4;
	interstage_buffer id_ex_rd_sel(~(stall_id | ext_ld_id), reset | squash_id, id_rd_write_sel, reset, rd_sel_ex, clk);

	defparam id_ex_rs_sel.n = 4;
	interstage_buffer id_ex_rs_sel(~(stall_id | ext_ld_id), reset | squash_id, id_rs_sel, reset, rs_sel_ex, clk);

	defparam id_ex_alu_en.n = 1;
	interstage_buffer id_ex_alu_en(~(stall_id | ext_ld_id), reset | squash_id, id_alu_en, reset, alu_en_ex, clk);
	
	defparam id_ex_alu_op.n = 4;
	interstage_buffer id_ex_alu_op(~(stall_id | ext_ld_id), reset | squash_id, id_alu_op, reset, alu_op_ex, clk);

	defparam id_ex_mem_ld.n = 1;
	interstage_buffer id_ex_mem_ld(~(stall_id | ext_ld_id), reset | squash_id, id_mem_ld, reset, mem_ld_ex, clk);
	
	defparam id_ex_mem_st.n = 1;
	interstage_buffer id_ex_mem_st(~(stall_id | ext_ld_id), reset | squash_id, id_mem_st, reset, mem_st_ex, clk);
	
	defparam id_ex_wb_en.n = 1;
	interstage_buffer id_ex_wb_en(~(stall_id | ext_ld_id), reset | squash_id, id_wb_en, reset, wb_en_ex, clk);

	//==========================================================================
	//		EXECUTE
	//==========================================================================
	assign rs_ex = vf_ex_rs_from_wb ? rd_wb : (vf_ex_rs_from_mem ? rd_mem : id_rs_ex);
	assign rd_ex = vf_ex_rd_from_wb ? rd_wb : (vf_ex_rd_from_mem ? rd_mem : id_rd_ex);
	
	EX execute(rs_ex, rd_ex, alu_en_ex, alu_op_ex, ex_rd);
	
	interstage_buffer ex_mem_rd(1'b1, reset | squash_ex, ex_rd, reset, ex_rd_mem, clk);
	interstage_buffer ex_mem_rs(1'b1, reset | squash_ex, rs_ex, reset, ex_rs_mem, clk);
	
	defparam ex_mem_rd_sel.n = 4;
	interstage_buffer ex_mem_rd_sel(1'b1, reset | squash_ex, rd_sel_ex, reset, rd_sel_mem, clk);

	defparam ex_mem_rs_sel.n = 4;
	interstage_buffer ex_mem_rs_sel(1'b1, reset | squash_ex, rs_sel_ex, reset, rs_sel_mem, clk);
		
	defparam ex_mem_mem_ld.n = 1;
	interstage_buffer ex_mem_mem_ld(1'b1, reset | squash_ex, mem_ld_ex, reset, mem_ld_mem, clk);
	
	defparam ex_mem_mem_st.n = 1;
	interstage_buffer ex_mem_mem_st(1'b1, reset | squash_ex, mem_st_ex, reset, mem_st_mem, clk);
	
	defparam ex_mem_wb_en.n = 1;
	interstage_buffer ex_mem_wb_en(1'b1, reset | squash_ex, wb_en_ex, reset, wb_en_mem, clk);
	
	//==========================================================================
	//		MEMORY
	//==========================================================================
	assign rs_mem = vf_mem_rs_from_wb ? rd_wb : ex_rs_mem;
	assign rd_mem = vf_mem_rd_from_wb ? rd_wb : ex_rd_mem;

	assign data_mem_write = mem_st_mem;
	assign data_mem_addr  = mem_st_mem ? rd_mem : rs_mem;
	assign data_mem_in	 = rs_mem;
	assign mem_rd			 = mem_ld_mem ? data_mem_out : rd_mem;

	interstage_buffer mem_wb_rd(1'b1, 1'b0, mem_rd, reset, rd_wb, clk);
	
	defparam mem_wb_rd_sel.n = 4;
	interstage_buffer mem_wb_rd_sel(1'b1, 1'b0, rd_sel_mem, reset, rd_sel_wb, clk);
	
	defparam mem_wb_wb_en.n = 1;
	interstage_buffer mem_wb_wb_en(1'b1, 1'b0, wb_en_mem, reset, wb_en_wb, clk);
	
	//==========================================================================
	//		WRITEBACK
	//==========================================================================

	//Theres not really anything here... just three wires that go to the 
	// register file

endmodule

//A simple register with discrete input and output ports. The contents of the
//register is alway asserted on the output port when oe is high, and the value
//on the input port is loaded into the register if the load signal is asserted
//on the positive edge of the clock.
module two_port_register(ld, in, reset, out, clk);
	parameter n = 16;
	input clk;
	input ld;
	input [n-1:0] in;
	input reset;
	output [n-1:0] out;
	
	reg [n-1:0] contents;
	
	assign out = contents;

	always@(posedge clk)
		if(reset) contents <= 'b0;
		else if(ld) contents <= in;
		
endmodule

//Same as the above, except that there is a signal which causes the register to
//output zeroes instead of its contents.
module interstage_buffer(ld, zero, in, reset, out, clk);
	parameter n = 16;
	input clk;
	input ld;
	input zero;
	input [n-1:0] in;
	input reset;
	output [n-1:0] out;
	
	reg [n-1:0] contents;
	
	assign out = zero ? 0 : contents;

	always@(posedge clk)
		if(reset) contents <= 0;
		else if(ld) contents <= in;

endmodule

//Register file with input and 2 output points
//R0 is tied to 0, R1 to -1
module register_file(sel_out1, sel_out2, sel_in, out1, out2, in, ld, clk);
	parameter n = 16;
	input [3:0] sel_out1;
	input [3:0] sel_out2;
	input [3:0] sel_in;
	input clk;
	input ld;
	input [n-1:0] in;
	output [n-1:0] out1;
	output [n-1:0] out2;
	
	reg [n-1:0] contents [2:15];
	
	//R0 returns all zeroes, R1 returns all ones.
	assign out1 = (sel_out1 == 0) ? 'b0 : ((sel_out1 == 1) ? -1 : contents[sel_out1]);
	assign out2 = (sel_out2 == 0) ? 'b0 : ((sel_out2 == 1) ? -1 : contents[sel_out2]);

	always@(posedge clk)
		if(ld & ~((sel_in == 0) | (sel_in == 1))) contents[sel_in] <= in;
endmodule

//16-bit ALU
module alu(x, y, z, sel);
	parameter n = 16;
	input [n-1:0] x;
	input [n-1:0] y;
	output [n-1:0] z;
	input [3:0] sel;
	
	reg [n-1:0] z;
	
	//Hopefully, Xilinx will be smart enough to recognize that this is
	// just a big multiplexer
   always @(sel, x, y)
		case (sel)
			4'b0000: z = x + y;
			4'b0001: z = x - y;
			4'b0010: z = x & y;
			4'b0011: z = x | y;
			4'b0100: z = x ^ y;
			4'b0101: z = y[0] + y[1] + y[2] + y[3] + y[4] + y[5] + y[6] + y[7] + y[8]
							 + y[9] + y[10] + y[11] + y[12] + y[13] + y[14] + y[15];
			default: z = 'bx;
		endcase
	
endmodule

//The instruction fetch stage: the output address indicates where the
// next instruction should be fetched.
//The most common behavior of is to increment the addr every clock cycle. However,
// if stall is high, the address will not be incremented. If pc_ld is high, the
// value on target_addr is loaded into the PC instead of PC+1. The PC can be
// loaded when the stall signal is high. This way, if a later stage stalls while
// a control instruction is in ID, the target address can still be loaded, 
// which overlaps the stalls.
module IF(addr, stall, pc_ld, target_addr, reset, clk);
	parameter n = 16;
	input stall;
	input clk;
	input pc_ld;	//If high, loads the value at target_addr into the PC
	input [n-1:0] target_addr;
	input reset;
	output [n-1:0] addr;	//The current PC, where we should be fetching
								// the next instruction

	wire [n-1:0] pc_next;		//The input to the PC
	wire [n-1:0] pc_out_addr;	//The output from the PC
	
	assign addr = pc_out_addr;

	//The next PC is either the target address, or the instruction after
	// the current PC
	assign pc_next = pc_ld ? target_addr : pc_out_addr + 1;
	
	//Load the PC when there is no stall, or if there is a stall, when
	// a load is requested
	two_port_register pc(~stall | pc_ld, pc_next, reset, pc_out_addr, clk);
endmodule

//Instruction decode module. Decodes the opcode and outputs a set of control
// signals. The source register is loaded from rs_sel, if the instruction does
// not use rs, then the values in rs and rs_sel are junk and the other control
// signals will cause them to be ignored. The destination register is what is
// written back to the register file in WB, so it is always loaded with something.
// For ALU instructions, it is loaded with ID, for memory-immediate, the immediate
// value, for JAL, the PC. This will make value forwarding easier: only rd_sel
// needs to be checked.
module ID(instr, pc, rs_in, rd_in, is_extended, is_halt, is_control, is_taken_control, target_addr, 
			 alu_en, alu_op, mem_ld, mem_st, wb_en, rs_sel, rd_read_sel, rd_write_sel, 
			 rs_out, rd_out);
	parameter n=16;
	input [n-1:0] instr;
	input [n-1:0] pc;
	
	output is_extended;
	output is_halt;
	output is_control;
	output is_taken_control;
	output [n-1:0] target_addr;

	output alu_en;
	output [3:0] alu_op;
	output mem_ld, mem_st;
	output wb_en;
	output [3:0] rs_sel;
	output [3:0] rd_read_sel;
	output [3:0] rd_write_sel;
	output [n-1:0] rs_out;
	output [n-1:0] rd_out;

	input [n-1:0] rd_in;
	input [n-1:0] rs_in;

	wire mem_ild;
	wire is_jump, is_jal, is_branch, branch_true;
	wire [16:0] imm_val;

	assign is_halt = (~|instr[15:9]) & instr[8];

	assign alu_en = ~instr[15] & instr[13];
	assign alu_op = instr[11:8];

	assign mem 		= ~instr[15] & instr[14];
	assign mem_ld	= mem & (~|instr[11:8]);
	assign mem_st	= mem & (~|instr[11:9]) & instr[8];
	assign mem_ild	= instr[15] & instr[14];

	assign imm_val	= {{8{instr[7]}},instr[7:0]};

	assign wb_en	= is_jal | mem_ld | mem_ild | alu_en;

	//rd_read_sel is the value of rd_sel used to read the register file
	//rd_write_sel is the value of rd_sel which should be written to in WB.
	//This is a hack. Since JAL always writes to $15, and we dont have a clock,
	// we have to generate a second signal.
	assign rs_sel			= instr[3:0];
	assign rd_read_sel	= instr[15] ? instr[11:8] : instr[7:4];
	assign rd_write_sel 	= is_jal ? 15 : rd_read_sel; 

	assign is_jump	= ~instr[15] & instr[12];
	assign is_jal	= is_jump & (~|instr[11:9]) & instr[8];

	assign is_branch		= instr[15] & instr[13];
	assign branch_true	= instr[12] ? rd_in[15] : ~|rd_in[15:0];
	assign target_addr	= is_branch ? imm_val + pc : rd_in;

	assign is_control 	= is_branch | is_jump;
	assign is_taken_control = (is_branch & branch_true) | is_jump; 
	
	assign is_extended	= mem_ild & instr[12];
	
	//If the instruction is a load immediate, rd is the sign extended immediate
	// value. If its a JAL, store the PC in rd for writeback, otherwise, use
	// whatever came into the module, ostensibly from the register file. If the
	// instruction doesn't have an rd, the other control signals should prevent
	// its use. Extended instructions are handled outside this module
	assign rd_out = mem_ild ? imm_val : (is_jal ? pc + 1 : rd_in);
	
	assign rs_out = rs_in;
endmodule

//The EX stage... entirely combinatorial. Passes rd through unaltered if the
// instruction is not a register-register ALU instruction
module EX(rs_in, rd_in, alu_en, alu_op, rd_out);
	parameter n=16;
	input [n-1:0] rs_in;
	input [n-1:0] rd_in;
	input alu_en;
	input [3:0] alu_op;
	output [n-1:0] rd_out;
	
	wire [n-1:0] alu_out;
	
	assign rd_out = alu_en ? alu_out : rd_in;
	
	alu executor(rd_in, rs_in, alu_out, alu_op);
	
endmodule
