



// 乘法/除法/取模组合电路。所有运算每时钟推进 1 位；目标是在 iCE40 FPGA 上
// 将资源占用降到最低。可选地将电路展开（unroll）以获得稍高的性能。
//
// op_kill 为高时，当前计算立即停止。op_vld 可在同一周期置位，无论 op_rdy
// 状态如何，新计算都会无延迟地开始。处理器可在误预测或陷入（trap）等
// 情况下使用该机制。
//
// 实际的乘/除硬件是无符号的，符号在输入/输出端处理。

`default_nettype none

module ominicore_muldiv_seq #(
`include "ominicore_config.vh"
,
`include "ominicore_width_const.vh"
) (
	input  wire               clk,
	input  wire               rst_n,
	input  wire [W_MULOP-1:0] op,
	input  wire               op_vld,
	output wire               op_rdy,
	input  wire               op_kill,
	input  wire [W_DATA-1:0]  op_a,
	input  wire [W_DATA-1:0]  op_b,

	output wire [W_DATA-1:0]  result_h, // mulh* 或 rem*
	output wire [W_DATA-1:0]  result_l, // mul   或 div*
	output wire               result_vld
);

`include "ominicore_ops.vh"

//synthesis translate_off
generate if (|(MULDIV_UNROLL & (MULDIV_UNROLL - 1)) || ~|MULDIV_UNROLL) begin: err_pow2
	initial $fatal("%m: MULDIV_UNROLL must be a positive power of 2");
end endgenerate
//synthesis translate_on

localparam XLEN = W_DATA;
parameter W_CTR = $clog2(XLEN + 1);

// ----------------------------------------------------------------------------
// 操作译码、操作数符号调整

// 第一周期 op_a 和 op_b 直接送入累加器与除数/被乘数寄存器，
// 下一周期再就地调整。这样可在输出前复用同一电路做符号调整
// （同时也有利于输入时序）。

reg [W_MULOP-1:0] op_r;
reg [2*XLEN-1:0]  accum;
reg [XLEN-1:0]    op_b_r;
reg               op_a_neg_r;
reg               op_b_neg_r;

wire op_a_signed =
	op_r == M_OP_MULH ||
	op_r == M_OP_MULHSU ||
	op_r == M_OP_DIV ||
	op_r == M_OP_REM;

wire op_b_signed =
	op_r == M_OP_MULH ||
	op_r == M_OP_DIV ||
	op_r == M_OP_REM;

wire op_a_neg = op_a_signed && accum[XLEN-1];
wire op_b_neg = op_b_signed && op_b_r[XLEN-1];

// 若所有 MUL 运算都由快速乘法器处理，电路中非除法的部分应被常量折叠掉

wire is_div = op_r[2] || (MUL_FAST && MULH_FAST);

// 用于修改累加器全部/部分符号的控制信号
wire accum_neg_l;
wire accum_inv_h;
wire accum_incr_h;

// ----------------------------------------------------------------------------
// 运算电路

// 组合逻辑：
reg [2*XLEN-1:0] accum_next;
reg [2*XLEN-1:0] addend;
reg [2*XLEN-1:0] shift_tmp;
reg [2*XLEN-1:0] addsub_tmp;
reg              neg_l_borrow;

always @ (*) begin: alu
	integer i;
	// 乘/除迭代层
	accum_next = accum;
	addend = {2*XLEN{1'b0}};
	addsub_tmp = {2*XLEN{1'b0}};
	neg_l_borrow = 1'b0;
	for (i = 0; i < MULDIV_UNROLL; i = i + 1) begin
		addend = {is_div && |op_b_r, op_b_r, {XLEN-1{1'b0}}};
		shift_tmp = is_div ? accum_next : accum_next >> 1;
		addsub_tmp = shift_tmp + addend;
		accum_next = (is_div ? !addsub_tmp[2 * XLEN - 1] : accum_next[0]) ?
			addsub_tmp : shift_tmp;
		if (is_div)
			accum_next = {accum_next[2*XLEN-2:0], !addsub_tmp[2 * XLEN - 1]};
	end
	// 累加器全部/部分取负的备用路径
	if (accum_neg_l)
		{neg_l_borrow, accum_next[XLEN-1:0]} = {~accum[XLEN-1:0]} + 1'b1;
	if (accum_incr_h || accum_inv_h)
		accum_next[XLEN +: XLEN] = (accum[XLEN +: XLEN] ^ {XLEN{accum_inv_h}})
			+ {{XLEN-1{1'b0}}, accum_incr_h};
end

// ----------------------------------------------------------------------------
// 主状态机

reg sign_preadj_done;
reg [W_CTR-1:0] ctr;
reg sign_postadj_done;
reg sign_postadj_carry;

localparam CTR_TOP = XLEN[W_CTR-1:0];

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		ctr <= {W_CTR{1'b0}};
		sign_preadj_done <= 1'b1;
		sign_postadj_done <= 1'b1;
		sign_postadj_carry <= 1'b0;
		op_r <= {W_MULOP{1'b0}};
		op_a_neg_r <= 1'b0;
		op_b_neg_r <= 1'b0;
		op_b_r <= {XLEN{1'b0}};
		accum <= {XLEN*2{1'b0}};
	end else if (op_kill || (op_vld && op_rdy)) begin
		// 用操作数 + 状态初始化电路
		ctr <= op_vld ? CTR_TOP : {W_CTR{1'b0}};
		sign_preadj_done <= !op_vld;
		sign_postadj_done <= !op_vld;
		sign_postadj_carry <= 1'b0;
		op_r <= op;
		op_b_r <= op_b;
		accum <= {{XLEN{1'b0}}, op_a};
	end else if (!sign_preadj_done) begin
		// 必要时先做符号预调整，否则立即执行第一次迭代
		op_a_neg_r <= op_a_neg;
		op_b_neg_r <= op_b_neg;
		sign_preadj_done <= 1'b1;
		if (accum_neg_l || (op_b_neg ^ is_div)) begin
			if (accum_neg_l)
				accum[0 +: XLEN] <= accum_next[0 +: XLEN];
			if (op_b_neg ^ is_div)
				op_b_r <= -op_b_r;
		end else begin
			ctr <= ctr - MULDIV_UNROLL[W_CTR-1:0];
			accum <= accum_next;
		end
	end else if (|ctr) begin
		ctr <= ctr - MULDIV_UNROLL[W_CTR-1:0];
		accum <= accum_next;
	end else if (!sign_postadj_done || sign_postadj_carry) begin
		sign_postadj_done <= 1'b1;
		if (accum_inv_h || accum_incr_h)
			accum[XLEN +: XLEN] <= accum_next[XLEN +: XLEN];
		if (accum_neg_l) begin
			accum[0 +: XLEN] <= accum_next[0 +: XLEN];
			if (!is_div) begin
				sign_postadj_carry <= neg_l_borrow;
				sign_postadj_done <= !neg_l_borrow;
			end
		end
	end
end

// ----------------------------------------------------------------------------
// 符号调整控制

// 预调整：对任意 a、b 取 |a|、|b|。注意任何 32 位有符号整数的幅值
// 都可用 32 位无符号整数表示。

// 除法的后调整：
// 求 q、r 满足 a = b * q + r，其中 a、b 已知，且 |r| < |b|。
// 一种做法是令
// sgn(r) = sgn(a)
// sgn(q) = sgn(a) ^ sgn(b)
// 这还带来一些良好性质，例如
// -(a / b) = (-a) / b = a / (-b)

// 乘法的后调整：
// 已算出 |a| * |b| 的 2*XLEN 位结果。
// 若 sgn(a) ^ sgn(b) 则将整个累加器取负。
// 分两步完成（以复用除/模电路，并避免 64 位进位）：
// - 累加器低半部取负，高半部取反
// - 若低半部有进位则高半部加 1

wire do_postadj = ~|{ctr, sign_postadj_done};
wire op_signs_differ = op_a_neg_r ^ op_b_neg_r;

assign accum_neg_l =
	!sign_preadj_done && op_a_neg ||
	do_postadj && !sign_postadj_carry && op_signs_differ && !(is_div && ~|op_b_r);

assign {accum_incr_h, accum_inv_h} =
	do_postadj &&  is_div && op_a_neg_r                             ? 2'b11 :
	do_postadj && !is_div && op_signs_differ && !sign_postadj_carry ? 2'b01 :
	do_postadj && !is_div && op_signs_differ &&  sign_postadj_carry ? 2'b10 :
	                                                                  2'b00 ;

// ----------------------------------------------------------------------------
// 输出

assign op_rdy = ~|{ctr, accum_neg_l, accum_incr_h, accum_inv_h};
assign result_vld = op_rdy;

`ifndef RISCV_FORMAL_ALTOPS

assign {result_h, result_l} = accum;

`else

// 提供算术上更简单的替代运算，以加速形式验证
always assert(XLEN == 32);

reg [XLEN-1:0] fml_a_saved;
reg [XLEN-1:0] fml_b_saved;

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		fml_a_saved <= {XLEN{1'b0}};
		fml_b_saved <= {XLEN{1'b0}};
	end else if (op_vld && op_rdy) begin
		fml_a_saved <= op_a;
		fml_b_saved <= op_b;
	end
end

assign result_h =
	op_r == M_OP_MULH   ? (fml_a_saved + fml_b_saved) ^ 32'hf6583fb7 :
	op_r == M_OP_MULHSU ? (fml_a_saved - fml_b_saved) ^ 32'hecfbe137 :
	op_r == M_OP_MULHU  ? (fml_a_saved + fml_b_saved) ^ 32'h949ce5e8 :
	op_r == M_OP_REM    ? (fml_a_saved - fml_b_saved) ^ 32'h8da68fa5 :
	op_r == M_OP_REMU   ? (fml_a_saved - fml_b_saved) ^ 32'h3138d0e1 : 32'hdeadbeef;

assign result_l =
	op_r == M_OP_MUL    ? (fml_a_saved + fml_b_saved) ^ 32'h5876063e :
	op_r == M_OP_DIV    ? (fml_a_saved - fml_b_saved) ^ 32'h7f8529ec :
	op_r == M_OP_DIVU   ? (fml_a_saved - fml_b_saved) ^ 32'h10e8fd70 : 32'hdeadbeef;

`endif

// ----------------------------------------------------------------------------
// 接口性质（断言）

`ifdef OMNICORE_ASSERTIONS

always @ (posedge clk) if (rst_n && $past(rst_n)) begin: properties
	integer i;
	reg alive;

	if ($past(op_rdy && !op_vld))
		assert(op_rdy);

	if (result_vld && $past(result_vld) && !$past(op_kill))
		assert($stable({result_h, result_l}));

	// kill 会中止进行中的运算，但新运算可以与 kill 同时发起。
	if ($past(op_kill))
		assert(op_rdy == !$past(op_vld));

	// 应当周期性地就绪（活性属性），除非新运算与 kill 同时被强行插入，
	// 这种情况下不存在中间的就绪状态。
	alive = op_rdy || (op_kill && op_vld);
	for (i = 1; i <= XLEN / MULDIV_UNROLL + 3; i = i + 1)
		alive = alive || $past(op_rdy || (op_kill && op_vld), i);
	assert(alive);
end

`endif

endmodule

`ifndef YOSYS
`default_nettype wire
`endif
