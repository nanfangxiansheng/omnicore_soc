



// req: 位图
// idx: 除 req 中最低（HIGHEST_WINS=0）或最高（HIGHEST_WINS=1）的
//      置位位外全部清零的位图。

`default_nettype none

module ominicore_onehot_priority #(
	parameter W_REQ        = 16,
	parameter HIGHEST_WINS = 0
) (
	input  wire [W_REQ-1:0] req,
	output reg  [W_REQ-1:0] gnt
);

always @ (*) begin: select
	integer i;
	for (i = 0; i < W_REQ; i = i + 1) begin
		gnt[i] = req[i] && ~|(req & (
			HIGHEST_WINS ? ~({W_REQ{1'b1}} >> (W_REQ - 1 - i)) : ~({W_REQ{1'b1}} << i)
		));
	end
end

endmodule

`ifndef YOSYS
`default_nettype wire
`endif
