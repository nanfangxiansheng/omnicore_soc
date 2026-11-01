



// req: 独热位图
// idx: req 中唯一置位位的索引

`default_nettype none

module ominicore_onehot_encode #(
	parameter W_REQ = 16,
	parameter W_GNT = $clog2(W_REQ) // 勿改动
) (
	input  wire [W_REQ-1:0] req,
	output reg  [W_GNT-1:0] gnt
);

always @ (*) begin: encode
	reg [W_GNT:0] i;
	gnt = {W_GNT{1'b0}};
	for (i = 0; i < W_REQ; i = i + 1) begin
		gnt = gnt | ({W_GNT{req[i[W_GNT-1:0]]}} & i[W_GNT-1:0]);
	end
end

endmodule

`ifndef YOSYS
`default_nettype wire
`endif
