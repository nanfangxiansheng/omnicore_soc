


`default_nettype none

// 经过 ALU 的分支判决路径较慢，原因如下：
//
// - 输入除了寄存器外还挂着立即数和 PC
// - 要做加/减而不只是加法（减法条件的译码较复杂）
// - 若启用 Zba 扩展，加法器前还要多 2 层 mux
//
// 因此使用专用的分支比较器有时能带来时序收益。

module ominicore_branchcmp #(
`include "ominicore_config.vh"
,
`include "ominicore_width_const.vh"
) (
	input  wire [31:0]        cir,
	input  wire [W_DATA-1:0]  op_a,
	input  wire [W_DATA-1:0]  op_b,
	output wire               cmp
);

`include "ominicore_ops.vh"

wire [W_DATA-1:0] diff = op_a - op_b;

// funct3    指令
// ------------------
// 000    BEQ
// 001    BNE
// 100    BLT
// 101    BGE
// 110    BLTU
// 111    BGEU

wire cmp_is_unsigned = cir[13];

wire lt = op_a[W_DATA-1] == op_b[W_DATA-1] ? diff[W_DATA-1] :
          cmp_is_unsigned                  ? op_b[W_DATA-1] :
                                             op_a[W_DATA-1] ;

assign cmp = cir[14] ? lt : op_a != op_b;

endmodule

`ifndef YOSYS
`default_nettype wire
`endif
