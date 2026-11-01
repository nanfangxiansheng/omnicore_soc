


// req:      请求位图
// priority: 各请求动态优先级的紧凑（packed）数组
// gnt:      含最高优先级请求的独热位图。

`default_nettype none

module ominicore_onehot_priority_dynamic #(
	parameter W_REQ                 = 8,
	parameter N_PRIORITIES          = 2,
	parameter PRIORITY_HIGHEST_WINS = 1, // 若为 1，数值最高的级具有最高优先权。
	                                     // 否则数值最低者胜出。
	parameter TIEBREAK_HIGHEST_WINS = 0, // 若为 1，最高优先级平局时编号最大的请求胜出。
	                                     // 否则编号最小者胜出。
	// 勿改动：
	parameter W_PRIORITY            = $clog2(N_PRIORITIES)
) (
	input  wire [W_REQ*W_PRIORITY-1:0] pri,
	input  wire [W_REQ-1:0]            req,
	output wire [W_REQ-1:0]            gnt
);

// 1. 按优先级对请求分层
reg [W_REQ-1:0]        req_stratified [0:N_PRIORITIES-1];
reg [N_PRIORITIES-1:0] level_has_req;

always @ (*) begin: stratify
	reg signed [31:0] i, j;
	for (i = 0; i < N_PRIORITIES; i = i + 1) begin
		for (j = 0; j < W_REQ; j = j + 1) begin
			req_stratified[i][j] = req[j] &&
				pri[W_PRIORITY * j +: W_PRIORITY] == i[W_PRIORITY-1:0];
		end
		level_has_req[i] = |req_stratified[i];
	end
end

// 2. 选出有活跃请求的最高层级
wire [N_PRIORITIES-1:0] active_layer_sel;

ominicore_onehot_priority #(
	.W_REQ        (N_PRIORITIES),
	.HIGHEST_WINS (PRIORITY_HIGHEST_WINS)
) prisel_layer (
	.req (level_has_req),
	.gnt (active_layer_sel)
);

// 3. 仅保留该层级上的请求
reg [W_REQ-1:0] reqs_from_highest_layer;

always @ (*) begin: mux_reqs_by_layer
	integer i;
	reqs_from_highest_layer = {W_REQ{1'b0}};
	for (i = 0; i < N_PRIORITIES; i = i + 1)
		reqs_from_highest_layer = reqs_from_highest_layer |
			(req_stratified[i] & {W_REQ{active_layer_sel[i]}});
end

// 4. 对这些请求做标准优先选择以打破平局
ominicore_onehot_priority #(
	.W_REQ        (W_REQ),
	.HIGHEST_WINS (TIEBREAK_HIGHEST_WINS)
) prisel_tiebreak (
	.req (reqs_from_highest_layer),
	.gnt (gnt)
);

endmodule

`ifndef YOSYS
`default_nettype wire
`endif
