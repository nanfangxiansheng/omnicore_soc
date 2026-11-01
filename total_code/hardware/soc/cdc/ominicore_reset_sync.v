


// 输入置起时输出异步置起，输入撤销后则随时钟同步撤销。
// 输入与输出均为低电平有效。
//
// 这是一个基线实现——应替换为目标 FPGA/工艺专属的复位单元

`ifndef OMNICORE_REG_KEEP_ATTRIBUTE
`define OMNICORE_REG_KEEP_ATTRIBUTE (* keep = 1'b1 *)
`endif

`default_nettype none

module ominicore_reset_sync #(
	parameter N_STAGES = 2 // 应 >= 2
) (
	input  wire clk,
	input  wire rst_n_in,
	output wire rst_n_out
);

`OMNICORE_REG_KEEP_ATTRIBUTE reg [N_STAGES-1:0] delay;

always @ (posedge clk or negedge rst_n_in)
	if (!rst_n_in)
		delay <= {N_STAGES{1'b0}};
	else
		delay <= {delay[N_STAGES-2:0], 1'b1};

assign rst_n_out = delay[N_STAGES-1];

endmodule

`ifndef YOSYS
`default_nettype wire
`endif
