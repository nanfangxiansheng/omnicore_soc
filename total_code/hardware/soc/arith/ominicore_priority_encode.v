


// req: 位图
// gnt: 最低置位位（HIGHEST_WINS=0）或最高置位位（HIGHEST_WINS=1）的索引

`default_nettype none

module ominicore_priority_encode #(
	parameter W_REQ        = 16,
	parameter HIGHEST_WINS = 0,
	parameter W_GNT        = $clog2(W_REQ) // 勿改动
) (
	input  wire [W_REQ-1:0] req,
	output wire [W_GNT-1:0] gnt
);

wire [W_REQ-1:0] gnt_onehot;

ominicore_onehot_priority #(
	.W_REQ        (W_REQ),
	.HIGHEST_WINS (HIGHEST_WINS)
) priority_u (
	.req (req),
	.gnt (gnt_onehot)
);

ominicore_onehot_encode #(
	.W_REQ (W_REQ)
) encode_u (
	.req (gnt_onehot),
	.gnt (gnt)
);

endmodule

`ifndef YOSYS
`default_nettype wire
`endif
