



// 用单个对数型桶形移位器实现三种移位（逻辑左移、逻辑右移、算术右移）。
// 32 位约需 240 个 LUT。
// （7 层每层 32 个 2 输入 mux，另有一些额外的 LUT 与 LUT 输入用于算术移位）

`default_nettype none

module ominicore_shift_barrel #(
`include "ominicore_config.vh"
,
`include "ominicore_width_const.vh"
) (
	input wire [W_DATA-1:0]  din,
	input wire [W_SHAMT-1:0] shamt,
	input wire               right_nleft,
	input wire               rotate,
	input wire               arith,
	output reg [W_DATA-1:0]  dout
);

reg [W_DATA-1:0] din_rev;
reg [W_DATA-1:0] shift_accum;
reg sext; // 哈哈

always @ (*) begin: shift
	integer i;

	for (i = 0; i < W_DATA; i = i + 1)
		din_rev[i] = right_nleft ? din[W_DATA - 1 - i] : din[i];

	sext = arith && din_rev[0];

	shift_accum = din_rev;
	for (i = 0; i < W_SHAMT; i = i + 1) begin
		if (shamt[i]) begin
			shift_accum = (shift_accum << (1 << i)) |
				({W_DATA{sext}} & ~({W_DATA{1'b1}} << (1 << i))) |
				({W_DATA{rotate && |EXTENSION_ZBB}} & (shift_accum >> (W_DATA - (1 << i))));
		end
	end

	for (i = 0; i < W_DATA; i = i + 1)
		dout[i] = right_nleft ? shift_accum[W_DATA - 1 - i] : shift_accum[i];
end

`ifdef OMNICORE_ASSERTIONS
always @ (*) begin
	if (right_nleft && arith && !rotate) begin: asr
		assert($signed(dout) == $signed(din) >>> $signed(shamt));
	end else if (right_nleft && !arith && !rotate) begin
		assert(dout == din >> shamt);
	end else if (!right_nleft && !arith && !rotate) begin
		assert(dout == din << shamt);
	end
end
`endif

endmodule

`ifndef YOSYS
`default_nettype wire
`endif
