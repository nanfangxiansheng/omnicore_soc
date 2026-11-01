


`default_nettype none

// OmniCore 中断控制器。支持最多 512 条外部中断
// 输入线，以及最多 16 级抢占。

module ominicore_irq_ctrl #(
`include "ominicore_config.vh"
) (
	input  wire                clk,
	input  wire                clk_always_on,
	input  wire                rst_n,

	// CSR 接口
	input  wire [11:0]         addr,
	input  wire [1:0]          wtype,
	input  wire                wen_m_mode,
	input  wire                ren_m_mode,
	input  wire [W_DATA-1:0]   wdata_raw,
	input  wire [W_DATA-1:0]   wdata,
	output reg  [W_DATA-1:0]   rdata,

	// 用于上下文更新的陷阱进入/退出信号
	input  wire                trapreg_update_enter,
	input  wire                trapreg_update_exit,
	input  wire                trap_entry_is_eirq,

	// 通过 meicontext 清除与保存 mie.mtie/msie 的接口
	output wire                meicontext_clearts,
	input  wire                mie_mtie,
	input  wire                mie_msie,

	// 外部 IRQ 输入:
	input  wire [NUM_IRQS-1:0] irq,

	// mip.meip:
	output wire                external_irq_pending
);

`include "ominicore_ops.vh"
`include "ominicore_csr_addr.vh"

localparam MAX_IRQS = 512;
localparam [3:0] IRQ_PRIORITY_MASK = ~(4'hf >> IRQ_PRIORITY_BITS);
localparam W_IRQ_INDEX = $clog2(MAX_IRQS);

// ----------------------------------------------------------------------------
// IRQ 输入触发器

// 对外部 IRQ 信号做寄存（主要是避免 IRQ 到
// 总线请求信号的直通路径）。始终有时钟，因为它用于产生唤醒。
// 输入寄存器可以按 IRQ 逐条移除，但这样做
// 须谨慎，因为这会在 IRQ 与总线之间形成直通路径。

wire [NUM_IRQS-1:0] irq_r;

genvar g;
generate
for (g = 0; g < NUM_IRQS; g = g + 1) begin: irq_reg_loop
	if (IRQ_INPUT_BYPASS[g]) begin: no_reg
		assign irq_r[g] = irq[g];
	end else begin: have_reg
		reg q;
		always @ (posedge clk_always_on or negedge rst_n) begin
			if (!rst_n) begin
				q <= 1'b0;
			end else begin
				q <= irq[g];
			end
		end
		assign irq_r[g] = q;
	end
end
endgenerate

// ----------------------------------------------------------------------------
// CSR 写

// 稍后赋值:
wire [W_IRQ_INDEX-1:0] meinext_irq;
wire                   meinext_noirq;
reg  [3:0]             eirq_highest_priority;

// 中断阵列寄存器:
reg  [NUM_IRQS-1:0]   meiea;
reg  [NUM_IRQS-1:0]   meifa;
reg  [4*NUM_IRQS-1:0] meipra;

// 用于 CSR 读出的补零向量
wire [MAX_IRQS-1:0]   meiea_rdata  = {{MAX_IRQS-NUM_IRQS{1'b0}}, meiea};
wire [MAX_IRQS-1:0]   meifa_rdata  = {{MAX_IRQS-NUM_IRQS{1'b0}}, meifa};
wire [4*MAX_IRQS-1:0] meipra_rdata = {{4*(MAX_IRQS-NUM_IRQS){1'b0}}, meipra};

always @ (posedge clk or negedge rst_n) begin: update_irq_reg_arrays
	reg signed [31:0] i;
	if (!rst_n) begin
		meiea <= {NUM_IRQS{1'b0}};
		meifa <= {NUM_IRQS{1'b0}};
		meipra <= {4*NUM_IRQS{1'b0}};
	end else begin
		for (i = 0; i < NUM_IRQS; i = i + 1) begin
			// CSR 写更新。注意阵列索引用的是原始 wdata ——
			// 这是正确性所需，同时避免与 rdata 构成环路。
			if (wen_m_mode && addr == MEIEA  && $signed(wdata_raw[4:0]) == i[W_IRQ_INDEX-1:4]) begin
				meiea[i]  <= wdata[16 + (i % 16)];
			end
			if (wen_m_mode && addr == MEIFA  && $signed(wdata_raw[4:0]) == i[W_IRQ_INDEX-1:4]) begin
				meifa[i]  <= wdata[16 + (i % 16)];
			end
			if (wen_m_mode && addr == MEIPRA && $signed(wdata_raw[6:0]) == i[W_IRQ_INDEX-1:2]) begin
				meipra[4 * i +: 4] <= wdata[16 + 4 * (i % 4) +: 4] & IRQ_PRIORITY_MASK;
			end
			// 当对应 IRQ 从 meinext 被取样后清除 IRQ 强制（force）位
			// （这样一条 IRQ 可以*只*注入一次，而无需修改 ISR 源）
			if (meinext_irq == i[W_IRQ_INDEX-1:0] && ren_m_mode && addr == MEINEXT && !meinext_noirq) begin
				meifa[i[$clog2(NUM_IRQS)-1:0]] <= 1'b0;
			end
		end
	end
end

reg [3:0]             meicontext_pppreempt;
reg [3:0]             meicontext_ppreempt;
reg [4:0]             meicontext_preempt;
reg                   meicontext_noirq;
reg [W_IRQ_INDEX-1:0] meicontext_irq;
reg                   meicontext_mreteirq;

wire [4:0] preempt_level_next = meinext_noirq ? 5'h10 : (
	(5'd1 << (4 - IRQ_PRIORITY_BITS)) + {1'b0, eirq_highest_priority}
);

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		meicontext_pppreempt <= 4'h0;
		meicontext_ppreempt <= 4'h0;
		meicontext_preempt <= 5'h0;
		meicontext_noirq <= 1'b1;
		meicontext_irq <= {W_IRQ_INDEX{1'b0}};
		meicontext_mreteirq <= 1'b0;
	end else if (trapreg_update_enter) begin
		if (trap_entry_is_eirq) begin
			// 优先级保存。注意 preempt 的最高位无需保存，
			// 因为该位置起时不可能发生抢占，我们不会走到这里。
			meicontext_pppreempt <= meicontext_ppreempt & IRQ_PRIORITY_MASK;
			meicontext_ppreempt <= meicontext_preempt[3:0] & IRQ_PRIORITY_MASK;
			// 设置 preempt 并非严格必要，因为在通过
			// mstatus.mie 重新使能 IRQ 之前，本应先做一次
			// meinext 的更新读，但这样做似乎最不会令人意外:
			meicontext_preempt <= preempt_level_next & {1'b1, IRQ_PRIORITY_MASK};
			meicontext_mreteirq <= 1'b1;
		end else begin
			meicontext_mreteirq <= 1'b0;
		end
	end else if (trapreg_update_exit) begin
		meicontext_mreteirq <= 1'b0;
		if (meicontext_mreteirq) begin
			// 优先级恢复
			meicontext_pppreempt <= 4'h0;
			meicontext_ppreempt <= meicontext_pppreempt & IRQ_PRIORITY_MASK;
			meicontext_preempt <= {1'b0, meicontext_ppreempt & IRQ_PRIORITY_MASK};
		end
	end else if (wen_m_mode && addr == MEICONTEXT) begin
		meicontext_pppreempt <= wdata[31:28] & IRQ_PRIORITY_MASK;
		meicontext_ppreempt <= wdata[27:24] & IRQ_PRIORITY_MASK;
		meicontext_preempt <= wdata[20:16] & {1'b1, IRQ_PRIORITY_MASK};
		meicontext_noirq <= wdata[15];
		meicontext_irq <= wdata[12:4];
		meicontext_mreteirq <= wdata[0];
	end else if (wen_m_mode && addr == MEINEXT && wdata[0]) begin
		// 中断已被取样且更新请求置起，因此相应地
		// 更新上下文（包括抢占级别）。
		meicontext_preempt <= preempt_level_next & {1'b1, IRQ_PRIORITY_MASK};
		meicontext_noirq <= meinext_noirq;
		meicontext_irq <= meinext_irq;
	end
end

assign meicontext_clearts = wen_m_mode && wtype != CSR_WTYPE_C && addr == MEICONTEXT && wdata_raw[1];

// ----------------------------------------------------------------------------
// 外部中断逻辑

// 当存在不低于当前抢占级别的中断时，发出陷阱请求。
// meinext 显示的是不低于*前一*抢占级别的中断:
// 这种屏蔽有助于避免在你已经抢占的
// 帧中重复进入同一 IRQ。

wire [NUM_IRQS-1:0] meipa = irq_r | meifa;
wire [MAX_IRQS-1:0] meipa_rdata = {{MAX_IRQS-NUM_IRQS{1'b0}}, meipa};

reg [NUM_IRQS-1:0] eirq_active_above_preempt;
reg [NUM_IRQS-1:0] eirq_active_above_ppreempt;

always @ (*) begin: eirq_compare
	integer i;
	for (i = 0; i < NUM_IRQS; i = i + 1) begin
		eirq_active_above_preempt[i]  = meipa[i] && meiea[i] && {1'b0, meipra[i * 4 +: 4]} >= meicontext_preempt;
		eirq_active_above_ppreempt[i] = meipa[i] && meiea[i] &&        meipra[i * 4 +: 4]  >= meicontext_ppreempt;
	end
end

assign external_irq_pending =  |eirq_active_above_preempt;
assign meinext_noirq        = ~|eirq_active_above_ppreempt;

// 还剩两件事要计算:
//
// - 高于 meicontext.ppreempt 的最高优先级未决（pending）IRQ
//   的 IRQ 编号是多少
// - 该 IRQ 的优先级是多少
//
// 第二种情况可以放宽计算、忽略 ppreempt，因为
// 只有当这样的 IRQ 确实存在时结果才需要有效。目前我们选择复用
// 同一个优先级选择器（关键路径可能更长，但节省面积），
// 也可以改用第二个忽略 ppreempt 屏蔽的优先级选择器。

wire [NUM_IRQS-1:0]    highest_eirq_onehot;
wire [W_IRQ_INDEX-1:0] meinext_irq_unmasked;

ominicore_onehot_priority_dynamic #(
	.W_REQ                 (NUM_IRQS),
	.N_PRIORITIES          (16),
	.PRIORITY_HIGHEST_WINS (1),
	.TIEBREAK_HIGHEST_WINS (0)
) eirq_priority_u (
	.pri (meipra[4*NUM_IRQS-1:0] & {NUM_IRQS{IRQ_PRIORITY_MASK}}),
	.req (eirq_active_above_ppreempt),
	.gnt (highest_eirq_onehot)
);

always @ (*) begin: get_highest_eirq_priority
	integer i;
	eirq_highest_priority = 4'h0;
	for (i = 0; i < NUM_IRQS; i = i + 1) begin
		eirq_highest_priority = eirq_highest_priority | (
			meipra[4 * i +: 4] & {4{highest_eirq_onehot[i]}}
		);
	end
end

wire [$clog2(NUM_IRQS)-1:0] meinext_irq_unmasked_nopad;

ominicore_onehot_encode #(
	.W_REQ (NUM_IRQS)
) eirq_encode_u (
	.req (highest_eirq_onehot),
	.gnt (meinext_irq_unmasked_nopad)
);

generate
if ($clog2(NUM_IRQS) == $clog2(MAX_IRQS)) begin: encode_eirq_no_padding
	assign meinext_irq_unmasked = meinext_irq_unmasked_nopad;
end else begin: encode_eirq_padded
	assign meinext_irq_unmasked = {
		{$clog2(MAX_IRQS) - $clog2(NUM_IRQS){1'b0}},
		meinext_irq_unmasked_nopad
	};
end
endgenerate

// 无需基于 meinext_noirq 对 meinext_irq 进行屏蔽，原因是:
// - noirq 置起时，CSR 字段的值并不重要
// - 没有任何 IRQ 时，优先级选择器
//   没有任何输入，结果本来就是 0。
assign meinext_irq = meinext_irq_unmasked;

// ----------------------------------------------------------------------------
// CSR 读

always @ (*) begin
	rdata = {W_DATA{1'b0}};
	case (addr)

	MEIEA: rdata = {
		meiea_rdata[wdata_raw[4:0] * 16 +: 16],
		16'h0
	};

	MEIPA: rdata = {
		meipa_rdata[wdata_raw[4:0] * 16 +: 16],
		16'h0
	};

	MEIFA: rdata = {
		meifa_rdata[wdata_raw[4:0] * 16 +: 16],
		16'h0
	};

	MEIPRA: rdata = {
		meipra_rdata[wdata_raw[6:0] * 16 +: 16],
		16'h0
	};

	MEINEXT: rdata = {
		meinext_noirq,
		20'h0,
		meinext_irq,
		2'h0
	};

	MEICONTEXT: rdata = {
		meicontext_pppreempt,
		meicontext_ppreempt,
		3'h0,
		meicontext_preempt,
		meicontext_noirq,
		2'h0,
		meicontext_irq,
		mie_mtie && meicontext_clearts,
		mie_msie && meicontext_clearts,
		1'b0,
		meicontext_mreteirq
	};

	default: rdata = {W_DATA{1'b0}};
	endcase
end

endmodule

`ifndef YOSYS
`default_nettype wire
`endif
