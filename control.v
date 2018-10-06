`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    21:31:55 04/10/2007 
// Design Name: 
// Module Name:    control 
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

//Control system for the IF and ID modules. Deals with extended 32-bit
// instructions, as well as control instructions by providing memory to the
// pipeline stages.
//An extended instruction causes a stall and a squash for the next cycle in ID 
// while the extended 16 bit value is fetched.
//A control instruction causes a squash in IF during the next cycle. This
// squashes the instruction fetched before the target of the control instruction 
// was loaded. By squashing the instruction in IF, we prevent any side effects
//A halt instruction causes IF to be squashed until the instruction before the halt can
// complete, then brings a halt signal high
module flow_control(id_is_extended, id_is_control, id_is_halt, vfu_stall_at_id, 
						  vfu_stall_at_ex, stall_if, stall_id, squash_if, squash_id, 
						  squash_ex, extended_ld, halt, 
						  reset, clk);
	input id_is_extended, id_is_control, id_is_halt;
	input vfu_stall_at_id, vfu_stall_at_ex;
	output stall_if, stall_id;
	output squash_if, squash_id, squash_ex;
	output extended_ld;
	output halt;
	input reset;
	input clk;
	
	reg vfu_squash_id, squash_ex;
	
	parameter start 	 = 3'b000,
				 extended = 3'b001,
				 control  = 3'b010,
				 halt1	 = 3'b100,
				 halt2	 = 3'b101,
				 halt3    = 3'b110;
	
	reg [2:0] state;
	reg [2:0] next_state;
	
	//next state logic
	always @(state or id_is_extended or id_is_control or id_is_halt or 
				vfu_stall_at_ex or vfu_stall_at_id or reset)
		case(state)
			start:	if(id_is_extended & 
								~vfu_stall_at_ex)	next_state = extended;
						else if(id_is_control 
							 & ~vfu_stall_at_id)	next_state = control;
						else if(id_is_halt &
							   ~vfu_stall_at_ex)	next_state = halt1;
						else							next_state = start;
			extended:								next_state = start;
			control:									next_state = start;
			halt1:									next_state = halt2;
			halt2:									next_state = halt3;
			halt3:									next_state = start;
			default:									next_state = 'bx;
		endcase
	
	//move to next state 
	always @(posedge clk or posedge reset)
		if(reset)
			state <= start;
		else
			state <= next_state;
	
	assign squash_if = (state == control)|(state == halt1)|(state == halt2)|(state == halt3);
	assign squash_id = (state == extended)|vfu_squash_id;
	assign extended_ld = (state == extended);
	assign halt = (state == halt3);
	
	assign stall_id = vfu_stall_at_ex;
	assign stall_if = (vfu_stall_at_ex & ~id_is_control) | vfu_stall_at_id;
	
	always @(posedge clk)
	begin
		squash_ex <= vfu_stall_at_ex;
		vfu_squash_id <= vfu_stall_at_id & ~vfu_stall_at_ex;
	end

endmodule

//value forwarding control unit
module value_forwarding_control(ctrl_in_id, li_in_ex, li_in_mem,
										  li_in_wb, alu_in_ex,
										  ld_in_ex, ld_in_mem, st_in_mem, rs_sel_id, 
										  rs_sel_ex, rs_sel_mem, rd_sel_id, rd_sel_ex,
										  rd_sel_mem, rd_sel_wb, id_rd_from_ex,
										  id_rd_from_mem, id_rs_from_wb,
										  id_rd_from_wb, ex_rs_from_mem, ex_rd_from_mem, 
										  ex_rs_from_wb, ex_rd_from_wb, mem_rs_from_wb, 
										  mem_rd_from_wb, stall_at_id, stall_at_ex);
	input ctrl_in_id;
	input li_in_ex, li_in_mem, li_in_wb;
	input alu_in_ex;
	input ld_in_ex, ld_in_mem;
	input st_in_mem;
	input [3:0]	rs_sel_id;
	input [3:0] rs_sel_ex;
	input [3:0] rs_sel_mem;
	input [3:0] rd_sel_id;
	input [3:0] rd_sel_ex;
	input [3:0] rd_sel_mem;
	input [3:0] rd_sel_wb;

	output id_rd_from_ex, id_rd_from_mem, id_rs_from_wb, id_rd_from_wb;
	output ex_rs_from_mem, ex_rd_from_mem, ex_rs_from_wb, ex_rd_from_wb;
	output mem_rs_from_wb, mem_rd_from_wb;
	output stall_at_id, stall_at_ex;

	//Filter out $0 and $1, which are special reserved registers
	assign rs_valid_id = (rs_sel_id != 0) & (rs_sel_id != 1);
	assign rd_valid_id = (rd_sel_id != 0) & (rd_sel_id != 1);

	assign rs_valid_ex = (rs_sel_ex != 0) & (rs_sel_ex != 1);
	assign rd_valid_ex = (rd_sel_ex != 0) & (rd_sel_ex != 1);

	assign rs_valid_mem = (rs_sel_mem != 0) & (rs_sel_mem != 1);
	assign rd_valid_mem = (rd_sel_mem != 0) & (rd_sel_mem != 1);

	assign rd_valid_wb = (rd_sel_wb != 0) & (rd_sel_wb != 1);

	//Determine matching values
	assign match_rd_id_ex  = (rd_sel_id == rd_sel_ex) & rd_valid_id;
	assign match_rd_id_mem = (rd_sel_id == rd_sel_mem) & rd_valid_id;
	assign match_rs_id_wb  = (rs_sel_id == rd_sel_wb) & rs_valid_id;
	assign match_rd_id_wb  = (rd_sel_id == rd_sel_wb) & rd_valid_id;
	
	assign match_rs_ex_mem = (rs_sel_ex == rd_sel_mem) & rs_valid_ex;
	assign match_rd_ex_mem = (rd_sel_ex == rd_sel_mem) & rd_valid_ex;
	assign match_rs_ex_wb  = (rs_sel_ex == rd_sel_wb) & rs_valid_ex;
	assign match_rd_ex_wb  = (rd_sel_ex == rd_sel_wb) & rd_valid_ex;

	assign match_rs_mem_wb = (rs_sel_mem == rd_sel_wb) & rs_valid_mem;
	assign match_rd_mem_wb = (rd_sel_mem == rd_sel_wb) & rd_valid_mem;

	//Determine if the instructions will write back to the register file
	assign wb_en_ex  = (li_in_ex | alu_in_ex | ld_in_ex) & rd_valid_ex;
	assign wb_en_mem = (li_in_mem | ld_in_mem) & rd_valid_mem;
	assign wb_en_wb  = li_in_wb & rd_valid_wb;

	//Determine which stages are ready to forward values
	assign ex_ready = li_in_ex;
	assign mem_ready = li_in_mem;

	//Determine which stages need values and which are needed
	//These signals are all parallel, so conflicting pairs may be high, i.e.
	// ID could 'need' values from mem and wb. The next level of logic will make
	// the priority determination between those signals.
	assign need_rd_id_ex  = match_rd_id_ex & ctrl_in_id & wb_en_ex;
	assign need_rd_id_mem = match_rd_id_mem & ctrl_in_id & wb_en_mem;
	assign need_rs_id_wb  = match_rs_id_wb & wb_en_wb;
	assign need_rd_id_wb  = match_rd_id_wb & wb_en_wb;
	
	assign need_rs_ex_mem = match_rs_ex_mem & alu_in_ex & wb_en_mem;
	assign need_rd_ex_mem = match_rd_ex_mem & alu_in_ex & wb_en_mem;
	assign need_rs_ex_wb  = match_rs_ex_wb & ~li_in_ex & wb_en_wb;
	assign need_rd_ex_wb  = match_rd_ex_wb & ~li_in_ex & wb_en_wb;

	assign need_rs_mem_wb = match_rs_mem_wb & (ld_in_mem | st_in_mem) & wb_en_wb;
	assign need_rd_mem_wb = match_rd_mem_wb & (ld_in_mem | st_in_mem) & wb_en_wb;

	//Forwarding to the MEM stage
	assign mem_rs_from_wb = need_rs_mem_wb;
	assign mem_rd_from_wb = need_rd_mem_wb;

	//Forwarding to the EX stage	
	assign ex_rs_from_mem = need_rs_ex_mem & mem_ready;
	assign ex_rd_from_mem = need_rd_ex_mem & mem_ready;
	assign ex_rs_from_wb  = need_rs_ex_wb & ~need_rs_ex_mem;
	assign ex_rd_from_wb  = need_rd_ex_wb & ~need_rd_ex_mem;
	
	assign ex_must_wait = (need_rs_ex_mem | need_rd_ex_mem) & ~mem_ready;
	
	//Forwarding to ID stage
	assign id_rd_from_ex  = need_rd_id_ex & ex_ready;
	assign id_rd_from_mem = need_rd_id_mem & mem_ready & ~need_rd_id_ex;
	assign id_rs_from_wb  = need_rs_id_wb;
	assign id_rd_from_wb  = need_rd_id_wb & ~need_rd_id_mem & ~need_rd_ex_mem;
	
	assign id_must_wait = (need_rd_id_ex & ~ex_ready) | (need_rd_id_mem & ~mem_ready);

	assign stall_at_id = id_must_wait;
	assign stall_at_ex = ex_must_wait;
endmodule
