



// 仅 MUL（配置：MUL_FAST）与 MUL/MULH/MULHU/MULHSU（配置：MUL_FAST &&
// MULH_FAST）分别由不同电路处理。两种情况都只是简单的行为级乘法，
// 依靠推断（inference）在 FPGA 上获得良好性能。

`default_nettype none

module ominicore_mul_fast #(
`include "ominicore_config.vh"
,
`include "ominicore_width_const.vh"
) (
	input  wire               clk,
	input  wire               rst_n,

	input  wire [W_MULOP-1:0] op,
	input  wire               op_vld,
	input  wire [W_DATA-1:0]  op_a,
	input  wire [W_DATA-1:0]  op_b,

	output wire [W_DATA-1:0] result,
	output reg               result_vld
);

`include "ominicore_ops.vh"

localparam XLEN = W_DATA;

//synthesis translate_off
generate if (MULH_FAST && !MUL_FAST) begin: err_require_mul_fast_for_mulh
	initial $fatal("%m: MULH_FAST requires that MUL_FAST is also set.");
end endgenerate
generate if (MUL_FASTER && !MUL_FAST) begin: err_require_mul_fast_for_faster
	initial $fatal("%m: MUL_FASTER requires that MUL_FAST is also set.");
end endgenerate
//synthesis translate_on

// 延迟为 1 拍：
always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		result_vld <= 1'b0;
	end else begin
		result_vld <= op_vld;
	end
end

// ----------------------------------------------------------------------------
// 仅快速 MUL

generate
if (!MULH_FAST) begin: mul_only

// 该流水级在 UP5k 上被折叠进 DSP 硬核前端。注意设计意图是：在 X 级末尾
// 寄存旁路的内核寄存器（因为旁路相当慢），随后在 M 级组合地完成乘法，
// 再多路选择进 MW 结果寄存器。

reg [XLEN-1:0] op_a_r;
reg [XLEN-1:0] op_b_r;

if (MUL_FASTER) begin: op_passthrough
	always @ (*) begin
		op_a_r = op_a;
		op_b_r = op_b;
	end
end else begin: op_register
	always @ (posedge clk) begin
		if (op_vld) begin
			op_a_r <= op_a;
			op_b_r <= op_b;
		end
	end
end

// 在 UP5k 上应能推断为 3 个 DSP 硬核：
//
// 1. 先寄存再相乘 a[15: 0] 与 b[15: 0]
// 2. 先寄存再相乘 a[31:16] 与 b[15: 0]，然后直接加上 1 的输出
// 3. 先寄存再相乘 a[15: 0] 与 b[31:16]，然后直接加上 2 的输出
//
// 因此路径相当长（1 次 16 位乘法，再 2 次 16 位加法）。在其他平台上
// 则可能只会得到一堆门电路。

`ifndef RISCV_FORMAL_ALTOPS

assign result = op_a_r * op_b_r;

`else

// riscv-formal 可以用更简单的函数，因为它只是确认结果已正确连接。
assign result = result_vld ? (op_a_r + op_b_r) ^ 32'h5876063e : 32'hdeadbeef;

`endif

// ----------------------------------------------------------------------------
// 快速 MUL/MULH/MULHU/MULHSU

end else begin: mul_and_mulh

reg [XLEN-1:0]    op_a_r;
reg [XLEN-1:0]    op_b_r;
reg [W_MULOP-1:0] op_r;

if (MUL_FASTER) begin: op_passthrough
	always @ (*) begin
		op_a_r = op_a;
		op_b_r = op_b;
		op_r = op;
	end
end else begin: op_register
	always @ (posedge clk) begin
		if (op_vld) begin
			op_a_r <= op_a;
			op_b_r <= op_b;
			op_r <= op;
		end
	end
end

wire op_a_signed = op_r == M_OP_MULH || op_r == M_OP_MULHSU;
wire op_b_signed = op_r == M_OP_MULH;

wire [2*XLEN-1:0] op_a_sext = {
	{XLEN{op_a_r[XLEN - 1] && op_a_signed}},
	op_a_r
};

wire [2*XLEN-1:0] op_b_sext = {
	{XLEN{op_b_r[XLEN - 1] && op_b_signed}},
	op_b_r
};

wire [2*XLEN-1:0] result_full = op_a_sext * op_b_sext;

`ifndef RISCV_FORMAL_ALTOPS

assign result = op_r == M_OP_MUL ? result_full[0 +: XLEN] : result_full[XLEN +: XLEN];

`else

assign result =
	op_r == M_OP_MULH   ? (op_a_r + op_b_r) ^ 32'hf6583fb7 :
	op_r == M_OP_MULHSU ? (op_a_r - op_b_r) ^ 32'hecfbe137 :
	op_r == M_OP_MULHU  ? (op_a_r + op_b_r) ^ 32'h949ce5e8 :
	op_r == M_OP_MUL    ? (op_a_r + op_b_r) ^ 32'h5876063e : 32'hdeadbeef;

`endif

end
endgenerate

endmodule

`ifndef YOSYS
`default_nettype wire
`endif
