


// 小指令进来，大指令出去

`default_nettype none

module ominicore_instr_decompress #(
`include "ominicore_config.vh"
) (
	input  wire        clk,
	input  wire        rst_n,

	input  wire [31:0] instr_in,

	output reg         instr_is_32bit,
	output reg  [31:0] instr_out,

	// 如果指令是非末尾的 uop，需要抑制 PC 更新，并把
	// 第 3 级中 mepc 地址的 PC 偏移清零。
	output wire        instr_out_is_uop,
	output wire        instr_out_is_final_uop,
	output wire        instr_out_uop_no_pc_update,
	// 指示 instr_out 是 uop 序列中不可中断部分的 uop。
	// 只要有一个 uop 不可中断，那么直到序列结束
	// 之前的所有后续 uop 也都不可中断。
	output wire        instr_out_uop_atomic,
	// 当前微码序列因下游执行而停顿
	input  wire        instr_out_uop_stall,
	input  wire        instr_out_uop_clear,

	// 送往前端的寄存器号译码器
	output wire [3:0]  df_uop_step,

	output reg         invalid
);

`include "rv_opcodes.vh"

localparam W_REGADDR = 5;
localparam PASSTHROUGH = ~|EXTENSION_C;

// 长寄存器格式: cr, ci, css
// 短寄存器格式: ciw, cl, cs, cb, cj
wire [W_REGADDR-1:0] rd_l  = instr_in[11:7];
wire [W_REGADDR-1:0] rs1_l = instr_in[11:7];
wire [W_REGADDR-1:0] rs2_l = instr_in[6:2];
wire [W_REGADDR-1:0] rd_s  = {2'b01, instr_in[4:2]};
wire [W_REGADDR-1:0] rs1_s = {2'b01, instr_in[9:7]};
wire [W_REGADDR-1:0] rs2_s = {2'b01, instr_in[4:2]};

// cx -> x 立即数格式的映射（我们是在*展开*指令，
// 而不是译码它们）:

wire [31:0] imm_ci = {
	{7{instr_in[12]}},
	instr_in[6:2],
	20'h00000
};

wire [31:0] imm_cj = {
	instr_in[12],
	instr_in[8],
	instr_in[10:9],
	instr_in[6],
	instr_in[7],
	instr_in[2],
	instr_in[11],
	instr_in[5:3],
	{9{instr_in[12]}},
	12'h000
};

wire [31:0] imm_cb ={
	{4{instr_in[12]}},
	instr_in[6:5],
	instr_in[2],
	13'h0000,
	instr_in[11:10],
	instr_in[4:3],
	instr_in[12],
	7'h00
};

wire [31:0] imm_c_lb = {
	10'h0,
	instr_in[5],
	instr_in[6],
	20'h00000
};

wire [31:0] imm_c_lh = {
	10'h000,
	instr_in[5],
	1'b0,
	20'h00000
};

function [31:0] rfmt_rd;  input [4:0] rd;  begin rfmt_rd  = {20'h00000, rd, 7'h00};   end endfunction
function [31:0] rfmt_rs1; input [4:0] rs1; begin rfmt_rs1 = {12'h000, rs1, 15'h0000}; end endfunction
function [31:0] rfmt_rs2; input [4:0] rs2; begin rfmt_rs2 = {7'h00, rs2, 20'h00000};  end endfunction

// ----------------------------------------------------------------------------
// push/pop 及其同类指令

// 最长的 uop 序列是最大配置的 cm.popretz:
//
// - 13 条 lw                    (counter = 0..12)
// - 1 条 addi 将 a0 清零        (counter = 13   ) < 原子段
// - 1 条 jalr 经 ra 跳转        (counter = 14   ) < 原子段
// - 1 条 addi 调整 sp           (counter = 15   ) < 原子段

wire [3:0] uop_ctr;
reg  [3:0] uop_ctr_nxt_in_seq;
reg        in_uop_seq;
reg        uop_no_pc_update;

wire zcmp_is_pushpop = instr_in[12];
wire uop_seq_end      = |EXTENSION_ZCMP && (zcmp_is_pushpop ? uop_ctr == 4'hf : uop_ctr[0]);
wire uop_atomic       = |EXTENSION_ZCMP && (zcmp_is_pushpop ? uop_ctr >= 4'he : uop_ctr[0]);

wire [3:0] uop_ctr_nxt =
	instr_out_uop_clear ? 4'h0    :
	instr_out_uop_stall ? uop_ctr : uop_ctr_nxt_in_seq;

assign instr_out_is_uop = in_uop_seq;
assign instr_out_is_final_uop = uop_seq_end;
assign instr_out_uop_atomic = uop_atomic;
assign instr_out_uop_no_pc_update = uop_no_pc_update;
assign df_uop_step = uop_ctr;

// 从当前 sp 值到最低地址被保存寄存器的偏移量，+64。
wire [3:0] zcmp_rlist = instr_in[7:4];
wire [3:0] zcmp_n_regs = zcmp_rlist == 4'hf ? 4'hd : zcmp_rlist - 4'h3;
wire       zcmp_rlist_invalid = zcmp_rlist < 4'h4 || (|EXTENSION_E && zcmp_rlist > 4'h6);

wire [11:0] zcmp_stack_adj_base =
	zcmp_rlist == 4'hf ? 12'h040 :
	zcmp_rlist >= 4'hc ? 12'h030 :
	zcmp_rlist >= 4'h8 ? 12'h020 : 12'h010;

wire [11:0] zcmp_stack_adj = zcmp_stack_adj_base + {6'h00, instr_in[3:2], 4'h0};

// 注意：我们先执行全部 load/store，然后才移动栈指针。
wire [11:0] zcmp_stack_lw_offset = -{6'h00, {zcmp_n_regs - uop_ctr}, 2'h0} + zcmp_stack_adj;
wire [11:0] zcmp_stack_sw_offset = -{6'h00, {zcmp_n_regs - uop_ctr}, 2'h0};

wire [4:0] zcmp_ls_reg =
	uop_ctr == 4'h0 ? 5'd01 : // ra
	uop_ctr == 4'h1 ? 5'd08 : // s0
	uop_ctr == 4'h2 ? 5'd09 : // s1
	5'd15 + {1'b0, uop_ctr};  // s2-s11 (s2 == x18)

wire [31:0] zcmp_push_sw_instr = `RVOPC_NOZ_SW | rfmt_rs1(5'd2) | rfmt_rs2(zcmp_ls_reg) | {
	zcmp_stack_sw_offset[11:5], 13'h0000, zcmp_stack_sw_offset[4:0], 7'h00
};

wire [31:0] zcmp_pop_lw_instr = `RVOPC_NOZ_LW | rfmt_rd(zcmp_ls_reg) | rfmt_rs1(5'd2)| {
	zcmp_stack_lw_offset[11:0], 20'h00000
};

wire [31:0] zcmp_push_stack_adj_instr = `RVOPC_NOZ_ADDI | rfmt_rd(5'd2) | rfmt_rs1(5'd2) | {
	-zcmp_stack_adj,
	20'h00000
};

wire [31:0] zcmp_pop_stack_adj_instr = `RVOPC_NOZ_ADDI | rfmt_rd(5'd2) | rfmt_rs1(5'd2) | {
	zcmp_stack_adj,
	20'h00000
};

wire [4:0] zcmp_sa01_r1s = {|instr_in[9:8], ~|instr_in[9:8], instr_in[9:7]};
wire [4:0] zcmp_sa01_r2s = {|instr_in[4:3], ~|instr_in[4:3], instr_in[4:2]};

wire       zcmp_sa01_invalid = |EXTENSION_E && |{instr_in[9:8], instr_in[4:3]};

// ----------------------------------------------------------------------------

generate
if (PASSTHROUGH) begin: instr_passthrough
	always @ (*) begin
		instr_is_32bit = 1'b1;
		instr_out = instr_in;
		invalid = 1'b0;
	end
end else begin: instr_decompress
	always @ (*) begin
		if (instr_in[1:0] == 2'b11) begin
			instr_is_32bit = 1'b1;
			instr_out = instr_in;
			invalid = 1'b0;
			in_uop_seq = 1'b0;
			uop_no_pc_update = 1'b0;
			uop_ctr_nxt_in_seq = uop_ctr;
		end else begin
			instr_is_32bit = 1'b0;
			instr_out = 32'd0;
			invalid = 1'b0;
			in_uop_seq = 1'b0;
			uop_no_pc_update = 1'b0;
			uop_ctr_nxt_in_seq = uop_ctr;
			casez (instr_in[15:0])
			`RVOPC_C_ADDI4SPN: begin
				instr_out = `RVOPC_NOZ_ADDI | rfmt_rd(rd_s) | rfmt_rs1(5'd2)
					| {2'h0, instr_in[10:7], instr_in[12:11], instr_in[5], instr_in[6], 2'b00, 20'h00000};
				invalid   = ~|instr_in[12:2]; // 恒为非法的全零指令
			end
			`RVOPC_C_LW:   instr_out = `RVOPC_NOZ_LW | rfmt_rd(rd_s) | rfmt_rs1(rs1_s)
				| {5'h00, instr_in[5], instr_in[12:10], instr_in[6], 2'b00, 20'h00000};
			`RVOPC_C_SW:   instr_out = `RVOPC_NOZ_SW | rfmt_rs2(rs2_s) | rfmt_rs1(rs1_s)
				| {5'h00, instr_in[5], instr_in[12], 13'h0000, instr_in[11:10], instr_in[6], 2'b00, 7'h00};
			`RVOPC_C_ADDI: instr_out = `RVOPC_NOZ_ADDI | rfmt_rd(rd_l) | rfmt_rs1(rs1_l) | imm_ci;
			`RVOPC_C_JAL:  instr_out = `RVOPC_NOZ_JAL  | rfmt_rd(5'd1) | imm_cj;
			`RVOPC_C_J:    instr_out = `RVOPC_NOZ_JAL  | rfmt_rd(5'd0) | imm_cj;
			`RVOPC_C_LI:   instr_out = `RVOPC_NOZ_ADDI | rfmt_rd(rd_l) | imm_ci;
			`RVOPC_C_LUI: begin
				if (rd_l == 5'd2) begin
					// addi16sp
					instr_out = `RVOPC_NOZ_ADDI | rfmt_rd(5'd2) | rfmt_rs1(5'd2) |
						{{3{instr_in[12]}}, instr_in[4:3], instr_in[5], instr_in[2], instr_in[6], 24'h000000};
				end else begin
					instr_out = `RVOPC_NOZ_LUI | rfmt_rd(rd_l) | {{15{instr_in[12]}}, instr_in[6:2], 12'h000};
				end
				invalid = ~|{instr_in[12], instr_in[6:2]}; // imm == 0 时为 RESERVED
			end
			`RVOPC_C_SLLI: instr_out = `RVOPC_NOZ_SLLI | rfmt_rd(rs1_l) | rfmt_rs1(rs1_l) | imm_ci;
			`RVOPC_C_SRAI: instr_out = `RVOPC_NOZ_SRAI | rfmt_rd(rs1_s) | rfmt_rs1(rs1_s) | imm_ci;
			`RVOPC_C_SRLI: instr_out = `RVOPC_NOZ_SRLI | rfmt_rd(rs1_s) | rfmt_rs1(rs1_s) | imm_ci;
			`RVOPC_C_ANDI: instr_out = `RVOPC_NOZ_ANDI | rfmt_rd(rs1_s) | rfmt_rs1(rs1_s) | imm_ci;
			`RVOPC_C_AND:  instr_out = `RVOPC_NOZ_AND  | rfmt_rd(rs1_s) | rfmt_rs1(rs1_s) | rfmt_rs2(rs2_s);
			`RVOPC_C_OR:   instr_out = `RVOPC_NOZ_OR   | rfmt_rd(rs1_s) | rfmt_rs1(rs1_s) | rfmt_rs2(rs2_s);
			`RVOPC_C_XOR:  instr_out = `RVOPC_NOZ_XOR  | rfmt_rd(rs1_s) | rfmt_rs1(rs1_s) | rfmt_rs2(rs2_s);
			`RVOPC_C_SUB:  instr_out = `RVOPC_NOZ_SUB  | rfmt_rd(rs1_s) | rfmt_rs1(rs1_s) | rfmt_rs2(rs2_s);
			`RVOPC_C_ADD: begin
				if (|rs2_l) begin
					instr_out = `RVOPC_NOZ_ADD | rfmt_rd(rd_l) | rfmt_rs1(rs1_l) | rfmt_rs2(rs2_l);
				end else if (|rs1_l) begin // jalr
					instr_out = `RVOPC_NOZ_JALR | rfmt_rd(5'd1) | rfmt_rs1(rs1_l);
				end else begin // ebreak
					instr_out = `RVOPC_NOZ_EBREAK;
				end
			end
			`RVOPC_C_MV: begin
				if (|rs2_l) begin // mv
					instr_out = `RVOPC_NOZ_ADD | rfmt_rd(rd_l) | rfmt_rs2(rs2_l);
				end else begin // jr
					instr_out = `RVOPC_NOZ_JALR | rfmt_rs1(rs1_l);
					invalid = ~|rs1_l; // RESERVED
				end
			end
			`RVOPC_C_LWSP: begin
				instr_out = `RVOPC_NOZ_LW | rfmt_rd(rd_l) | rfmt_rs1(5'd2) |
					{4'h0, instr_in[3:2], instr_in[12], instr_in[6:4], 2'b00, 20'h00000};
				invalid = ~|rd_l; // RESERVED
			end
			`RVOPC_C_SWSP:    instr_out = `RVOPC_NOZ_SW | rfmt_rs2(rs2_l) | rfmt_rs1(5'd2)
				| {4'h0, instr_in[8:7], instr_in[12], 13'h0000, instr_in[11:9], 2'b00, 7'h00};
			`RVOPC_C_BEQZ:     instr_out = `RVOPC_NOZ_BEQ | rfmt_rs1(rs1_s) | imm_cb;
			`RVOPC_C_BNEZ:     instr_out = `RVOPC_NOZ_BNE | rfmt_rs1(rs1_s) | imm_cb;

			// 可选的 Zcb 指令:
			`RVOPC_C_LBU: begin
				instr_out = `RVOPC_NOZ_LBU    | rfmt_rd(rd_s)  | rfmt_rs1(rs1_s) | imm_c_lb;
				invalid = ~|EXTENSION_ZCB;
			end
			`RVOPC_C_LHU: begin
				instr_out = `RVOPC_NOZ_LHU    | rfmt_rd(rd_s)  | rfmt_rs1(rs1_s) | imm_c_lh;
				invalid = ~|EXTENSION_ZCB;
			end
			`RVOPC_C_LH: begin
				instr_out = `RVOPC_NOZ_LH     | rfmt_rd(rd_s)  | rfmt_rs1(rs1_s) | imm_c_lh;
				invalid = ~|EXTENSION_ZCB;
			end
			`RVOPC_C_SB: begin
				instr_out = `RVOPC_NOZ_SB     | rfmt_rs2(rd_s) | rfmt_rs1(rs1_s) | imm_c_lb >> 13;
				invalid = ~|EXTENSION_ZCB;
			end
			`RVOPC_C_SH: begin
				instr_out = `RVOPC_NOZ_SH     | rfmt_rs2(rd_s) | rfmt_rs1(rs1_s) | imm_c_lh >> 13;
				invalid = ~|EXTENSION_ZCB;
			end
			`RVOPC_C_ZEXT_B: begin
				instr_out = `RVOPC_NOZ_ANDI   | rfmt_rd(rs1_s) | rfmt_rs1(rs1_s) | 32'h0ff00000;
				invalid = ~|EXTENSION_ZCB;
			end
			`RVOPC_C_SEXT_B: begin
				instr_out = `RVOPC_NOZ_SEXT_B | rfmt_rd(rs1_s) | rfmt_rs1(rs1_s);
				invalid = ~|EXTENSION_ZCB || ~|EXTENSION_ZBB;
			end
			`RVOPC_C_ZEXT_H: begin
				instr_out = `RVOPC_NOZ_ZEXT_H | rfmt_rd(rs1_s) | rfmt_rs1(rs1_s);
				invalid = ~|EXTENSION_ZCB || ~|EXTENSION_ZBB;
			end
			`RVOPC_C_SEXT_H: begin
				instr_out = `RVOPC_NOZ_SEXT_H | rfmt_rd(rs1_s) | rfmt_rs1(rs1_s);
				invalid = ~|EXTENSION_ZCB || ~|EXTENSION_ZBB;
			end
			`RVOPC_C_NOT: begin
				instr_out = `RVOPC_NOZ_XORI   | rfmt_rd(rs1_s) | rfmt_rs1(rs1_s) | 32'hfff00000;
				invalid = ~|EXTENSION_ZCB;
			end
			`RVOPC_C_MUL: begin
				instr_out = `RVOPC_NOZ_MUL    | rfmt_rd(rs1_s) | rfmt_rs1(rs1_s) | rfmt_rs2(rs2_s);
				invalid = ~|EXTENSION_ZCB || ~|EXTENSION_M;
			end

			// 可选的 Zclsd 指令:
			`RVOPC_C_LD: begin
				instr_out = `RVOPC_NOZ_LD | rfmt_rd(rd_s) | rfmt_rs1(rs1_s)
					| {4'h0, instr_in[6:5], instr_in[12:10], 3'b000, 20'h00000};
				invalid = ~|EXTENSION_ZILSD || ~|EXTENSION_ZCLSD;
			end
			`RVOPC_C_SD: begin
				instr_out = `RVOPC_NOZ_SD | rfmt_rs2(rs2_s) | rfmt_rs1(rs1_s)
					| {4'h0, instr_in[6:5], instr_in[12], 13'h0000, instr_in[11:10], 3'b000, 7'h00};
				invalid = ~|EXTENSION_ZILSD || ~|EXTENSION_ZCLSD;
			end
			`RVOPC_C_LDSP: begin
				instr_out = `RVOPC_NOZ_LD | rfmt_rd(rd_l) | rfmt_rs1(5'd2) |
					{3'h0, instr_in[4:2], instr_in[12], instr_in[6:5], 3'b000, 20'h00000};
				invalid = ~|EXTENSION_ZILSD || ~|EXTENSION_ZCLSD || ~|rd_l; // RESERVED
			end
			`RVOPC_C_SDSP: begin
				instr_out = `RVOPC_NOZ_SD | rfmt_rs2(rs2_l) | rfmt_rs1(5'd2)
					| {3'h0, instr_in[9:7], instr_in[12], 13'h0000, instr_in[11:10], 3'b000, 7'h00};
				invalid = ~|EXTENSION_ZILSD || ~|EXTENSION_ZCLSD;
			end

			// 可选的 Zcmp 指令:
			`RVOPC_CM_PUSH: if (~|EXTENSION_ZCMP || zcmp_rlist_invalid) begin
				invalid = 1'b1;
			end else if (uop_ctr == 4'hf) begin
				in_uop_seq = 1'b1;
				uop_ctr_nxt_in_seq = 4'h0;
				instr_out = zcmp_push_stack_adj_instr;
			end else begin
				in_uop_seq = 1'b1;
				uop_ctr_nxt_in_seq = uop_ctr + 4'h1;
				instr_out = zcmp_push_sw_instr;
				uop_no_pc_update = 1'b1;
				if (uop_ctr_nxt_in_seq == zcmp_n_regs) begin
					uop_ctr_nxt_in_seq = 4'hf;
				end
			end

			`RVOPC_CM_POP: if (~|EXTENSION_ZCMP || zcmp_rlist_invalid) begin
				invalid = 1'b1;
			end else if (uop_ctr == 4'hf) begin
				in_uop_seq = 1'b1;
				uop_ctr_nxt_in_seq = 4'h0;
				instr_out = zcmp_pop_stack_adj_instr;
			end else begin
				in_uop_seq = 1'b1;
				uop_ctr_nxt_in_seq = uop_ctr + 4'h1;
				uop_no_pc_update = 1'b1;
				instr_out = zcmp_pop_lw_instr;
				if (uop_ctr_nxt_in_seq == zcmp_n_regs) begin
					uop_ctr_nxt_in_seq = 4'hf;
				end
			end

			`RVOPC_CM_POPRET: if (~|EXTENSION_ZCMP || zcmp_rlist_invalid) begin
				invalid = 1'b1;
			end else if (uop_ctr == 4'he) begin
				// 注意：虽然这只是不可中断序列中的第一条指令，
				// 我们仍将本条指令标记为不可中断：这里有一些特殊情况的逻辑，
				// 让这条跳转执行时不冲刷最后的栈调整 uop —— 若该 uop 被中断，
				// 可能会采样到错误的异常 PC。
				in_uop_seq = 1'b1;
				uop_ctr_nxt_in_seq = uop_ctr + 4'h1;
				instr_out = `RVOPC_NOZ_JALR | rfmt_rs1(5'd1);
			end else if (uop_ctr == 4'hf) begin
				in_uop_seq = 1'b1;
				uop_ctr_nxt_in_seq = 4'h0;
				uop_no_pc_update = 1'b1;
				instr_out = zcmp_pop_stack_adj_instr;
			end else begin
				in_uop_seq = 1'b1;
				uop_ctr_nxt_in_seq = uop_ctr + 4'h1;
				instr_out = zcmp_pop_lw_instr;
				uop_no_pc_update = 1'b1;
				if (uop_ctr_nxt_in_seq == zcmp_n_regs) begin
					uop_ctr_nxt_in_seq = 4'he;
				end
			end

			`RVOPC_CM_POPRETZ: if (~|EXTENSION_ZCMP || zcmp_rlist_invalid) begin
				invalid = 1'b1;
			end else if (uop_ctr == 4'hd) begin
				in_uop_seq = 1'b1;
				uop_ctr_nxt_in_seq = uop_ctr + 4'h1;
				uop_no_pc_update = 1'b1;
				instr_out = `RVOPC_NOZ_ADDI | rfmt_rd(5'd10); // li a0, 0
			end else if (uop_ctr == 4'he) begin
				in_uop_seq = 1'b1;
				uop_ctr_nxt_in_seq = uop_ctr + 4'h1;
				instr_out = `RVOPC_NOZ_JALR | rfmt_rs1(5'd1);
			end else if (uop_ctr == 4'hf) begin
				in_uop_seq = 1'b1;
				uop_ctr_nxt_in_seq = 4'h0;
				uop_no_pc_update = 1'b1;
				instr_out = zcmp_pop_stack_adj_instr;
			end else begin
				in_uop_seq = 1'b1;
				uop_ctr_nxt_in_seq = uop_ctr + 4'h1;
				uop_no_pc_update = 1'b1;
				instr_out = zcmp_pop_lw_instr;
				if (uop_ctr_nxt_in_seq == zcmp_n_regs) begin
					uop_ctr_nxt_in_seq = 4'hd;
				end
			end

			`RVOPC_CM_MVSA01: if (~|EXTENSION_ZCMP || zcmp_sa01_invalid) begin
				invalid = 1'b1;
			end else if (uop_ctr == 4'h0) begin
				in_uop_seq = 1'b1;
				uop_ctr_nxt_in_seq = uop_ctr + 4'h1;
				uop_no_pc_update = 1'b1;
				instr_out = `RVOPC_NOZ_ADDI | rfmt_rd(zcmp_sa01_r1s) | rfmt_rs1(5'd10);
			end else begin
				in_uop_seq = 1'b1;
				uop_ctr_nxt_in_seq = 4'h0;
				instr_out = `RVOPC_NOZ_ADDI | rfmt_rd(zcmp_sa01_r2s) | rfmt_rs1(5'd11);
			end

			`RVOPC_CM_MVA01S: if (~|EXTENSION_ZCMP || zcmp_sa01_invalid) begin
				invalid = 1'b1;
			end else if (uop_ctr == 4'h0) begin
				in_uop_seq = 1'b1;
				uop_ctr_nxt_in_seq = uop_ctr + 4'h1;
				uop_no_pc_update = 1'b1;
				instr_out = `RVOPC_NOZ_ADDI | rfmt_rd(5'd10) | rfmt_rs1(zcmp_sa01_r1s);
			end else begin
				in_uop_seq = 1'b1;
				uop_ctr_nxt_in_seq = 4'h0;
				instr_out = `RVOPC_NOZ_ADDI | rfmt_rd(5'd11) | rfmt_rs1(zcmp_sa01_r2s);
			end

			default: invalid = 1'b1;
			endcase
		end
	end
end
endgenerate

generate
if (EXTENSION_ZCMP) begin: have_uop_ctr
	reg [3:0] uop_ctr_r;
	assign uop_ctr = uop_ctr_r;
	always @ (posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			uop_ctr_r <= 4'h0;
		end else begin
			uop_ctr_r <= uop_ctr_nxt;
`ifdef OMNICORE_ASSERTIONS
			assert(in_uop_seq || uop_ctr_r == 4'h0);
			assert(in_uop_seq || zcmp_ls_reg == 5'h01);
			assert(in_uop_seq || !uop_atomic);
			assert(in_uop_seq || !uop_no_pc_update);
			if (uop_seq_end) begin
				assert(in_uop_seq);
				assert(instr_out_uop_stall || uop_ctr_nxt == 4'h0);
			end
`endif
		end
	end
end else begin: no_uop_ctr
	assign uop_ctr = 4'h0;
end
endgenerate

endmodule

`ifndef YOSYS
`default_nettype wire
`endif
