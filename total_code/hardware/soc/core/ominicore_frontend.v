



`default_nettype none

module ominicore_frontend #(
`include "ominicore_config.vh"
) (
	input  wire              clk,
	input  wire              rst_n,

	// 取指接口
	// addr_vld 可在任意时刻置起，但置起之后，
	// 在 addr_rdy 的下一拍之前，addr 与 addr_vld 都不得变化。
	// 数据接口没有反压；前端必须确保
	// 不请求自己无法接收的数据。
	// addr_rdy 与 dat_vld 可以是 hready 的函数，
	// 但不得用于计算组合逻辑输出。
	output wire              mem_size, // 1'b1 -> 32 位访问
	output wire [W_ADDR-1:0] mem_addr,
	output wire              mem_priv,
	output wire              mem_addr_vld,
	input  wire              mem_addr_rdy,
	input  wire [W_DATA-1:0] mem_data,
	input  wire              mem_data_err,
	input  wire              mem_data_vld,

	// 跳转/冲刷接口
	// 处理器可在任意时刻置起 vld。请求只有在
	// rdy 为高时才会被受理。在此期间处理器*可以*修改请求。
	// 输入不得是 hready 的函数。
	input  wire [W_ADDR-1:0] jump_target,
	input  wire              jump_priv,
	input  wire              jump_target_vld,
	output wire              jump_target_rdy,

	// 到分支目标缓冲器的接口。`src_addr` 是发生跳转的后向分支
	// 最后一个半字的地址。前端据此重定向取指，
	// 使得 `src_addr` 看起来顺序紧跟着 `target`。
	input  wire              btb_set,
	input  wire [W_ADDR-1:0] btb_set_src_addr,
	input  wire              btb_set_src_size,
	input  wire [W_ADDR-1:0] btb_set_target_addr,
	input  wire              btb_clear,
	input  wire              btb_flush,   // ZIGUANG (BP-06): fence.i/陷阱整体清空
	output wire [W_ADDR-1:0] btb_target_addr_out,
	// ZIGUANG: 预测器统计抽头（送往 ominicore_perf）
	output wire              zg_evt_br_alloc,
	output wire              zg_evt_br_flush,
	output wire              zg_evt_br_ctrdec,

	// 到译码级的接口
	output reg  [31:0]       cir,                  // 当前指令寄存器; 已预先展开为 32 位
	output wire [31:0]       cir_raw,              // 未展开的指令数据
	output reg  [1:0]        cir_vld,              // CIR 中有效半字数
	input  wire [1:0]        cir_use,              // D 级打算消耗的半字数
	                                               // *可以*是 hready 的函数
	output wire [1:0]        cir_err,              // CIR 上/下半字上的总线错误。
	output wire [1:0]        cir_predbranch,       // 预测跳转分支的最后一个半字置起
	output wire              cir_break_any,        // CIR 低位半字精确命中断点地址时置起
	output wire              cir_break_d_mode,     // 同上，但专指打断进入调试模式
	output reg               cir_is_32bit,         // 因预先展开而无法从 CIR 译码得到
	output reg               cir_invalid_16bit,    // 展开得到的是非法 32 位指令
	output reg               cir_is_uop,           // 当前指令是微操作序列的一部分
	output reg               cir_uop_nonfinal,     // ……且本条指令还有后续 uop
	output reg               cir_uop_no_pc_update, // 抑制 PC 自增或跳转（注意 cm.popret 中的跳转不是最后一个 uop！）
	output reg               cir_uop_atomic,       // 阻止进入 IRQ，使中间状态不被观察到
	input  wire              uop_stall,
	input  wire              uop_clear,

	// "flush_behind": 接受跳转请求时不冲刷最老的指令
	// （但仍冲刷更年轻的指令）。有时一条停顿中的指令
	// 也可能发出跳转请求，例如停顿
	// 依赖于总线停顿信号而无法对请求进行门控。
	input  wire              cir_flush_behind,
	// 使能 Zilsd 时寄存器号预译码所需:
	input  wire              df_lspair_phase_next,

	// 向电源控制器指示可以断电的信号。（进入休眠时，
	// 流水线先停顿，然后电源控制器
	// 等待前端自然停止后，
	// 才释放其电源请求。这避免了手动暂停前端。）
	output wire              pwrdown_ok,
	// 用于延迟复位后第一次取指的信号，
	// 因为上电尚未协商完成。
	input  wire              delay_first_fetch,

	// 提供下一拍将出现在 CIR 中的 rs1/rs2 寄存器号。
	// 粗粒度: 只要本指令有非零寄存器操作数即有效。
	// （适合寄存器堆读）
	output reg  [4:0]        predecode_rs1_coarse,
	output reg  [4:0]        predecode_rs2_coarse,
	// 细粒度: 类似粗粒度，但当操作数是隐式的时准确清零。
	// （适合旁路。对停顿逻辑仍不够精确。）
	output reg  [4:0]        predecode_rs1_fine,
	output reg  [4:0]        predecode_rs2_fine,

	// 调试器指令注入: 调试停顿状态下取指被抑制，
	// 然后 DM 可以通过 vld/rdy 握手把指令注入
	// 预取队列的最后一个表项。
	input  wire              debug_mode,
	input  wire [W_DATA-1:0] dbg_instr_data,
	input  wire              dbg_instr_data_vld,
	output wire              dbg_instr_data_rdy,

	// PMP 查询->击杀接口，用于 X 权限检查
	output wire [W_ADDR-1:0] pmp_i_addr,
	output wire              pmp_i_m_mode,
	input  wire              pmp_i_kill,

	// 触发单元查询->打断接口，用于断点
	output wire [W_ADDR-1:0] trigger_addr,
	output wire              trigger_m_mode,
	input  wire [1:0]        trigger_break_any,
	input  wire [1:0]        trigger_break_d_mode

);

`include "rv_opcodes.vh"

localparam W_BUNDLE = 16;
// 这是满足满吞吐的最小深度（足以在译码停顿时
// 不丢数据），再大也没有显著好处。
localparam FIFO_DEPTH = 2;

// ZIGUANG (C1): 半字槽布局 -- 元数据 4 位 + W_ADDR 预测目标字段 + 16 位指令。
// 槽字段位偏移在模块前部声明 (btb_target_addr_out 前向引用)。
localparam W_SLOT                = 4 + W_ADDR + W_BUNDLE;
wire btb_match_next_addr;            // (2026-10-09 前移: vlog 严格先声明后使用)
reg  [3*W_SLOT-1:0] buf_contents;
localparam SLOT_TARGET_BIT       = 4 + W_BUNDLE;
localparam SLOT_BREAK_ANY_BIT    = 3 + W_BUNDLE;
localparam SLOT_BREAK_D_MODE_BIT = 2 + W_BUNDLE;
localparam SLOT_ERR_BIT          = 1 + W_BUNDLE;
localparam SLOT_PREDBRANCH_BIT   = 0 + W_BUNDLE;

// ----------------------------------------------------------------------------
// 取指队列

wire jump_now = jump_target_vld && jump_target_rdy;
reg [1:0] mem_data_hwvld;

// PMP X 权限错误与取指并行检查（只要可执行内存
// 满足读幂等即可），失败会升级为总线错误:
wire pmp_kill_fetch_dph;
wire mem_or_pmp_err = mem_data_err || pmp_kill_fetch_dph;

// 类似地，断点匹配在取指数据相检查。它们命名为
// mem_xxx，因为它们是本 dphase 中
// 从内存返回数据的断点元数据。
wire [1:0] mem_break_any;
wire [1:0] mem_break_d_mode;

// 将数据标记为含预测跳转的分支指令，以便
// 恢复误预测 —— 需要同时跟踪两个半字，才能
// 标记整条指令、且仅该指令:
reg [1:0] mem_data_predbranch;

// ZIGUANG (C1): 预测目标随指令穿透 -- 与 mem_data_predbranch 同拍锁存的
// 预测目标快照 (取指受理拍的 BTB 查找结果), 经 FIFO 与半字槽元数据
// 携带到译码 PC 多选器, 取代原 "译码读活动表项 a_target"。
reg [W_ADDR-1:0] mem_data_target;

// 总线错误（及其他元数据）与数据同行。若内核译码了
// 该指令则产生异常，在那之前可以
// 无害地冲刷掉。

reg  [W_DATA-1:0]    fifo_mem          [0:FIFO_DEPTH];
reg                  fifo_err          [0:FIFO_DEPTH];
reg  [1:0]           fifo_break_any    [0:FIFO_DEPTH];
reg  [1:0]           fifo_break_d_mode [0:FIFO_DEPTH];
reg  [1:0]           fifo_predbranch   [0:FIFO_DEPTH];
reg  [W_ADDR-1:0]    fifo_target       [0:FIFO_DEPTH];
reg  [1:0]           fifo_valid_hw     [0:FIFO_DEPTH];
reg                  fifo_valid        [0:FIFO_DEPTH];
reg                  fifo_valid_m1     [0:FIFO_DEPTH];

wire [W_DATA-1:0] fifo_rdata       = fifo_mem[0];
wire              fifo_full        = fifo_valid[FIFO_DEPTH - 1];
wire              fifo_empty       = !fifo_valid[0];
wire              fifo_almost_full = fifo_valid[FIFO_DEPTH - 2];

wire              fifo_push;
wire              fifo_pop;
wire              fifo_dbg_inject = DEBUG_SUPPORT && dbg_instr_data_vld && dbg_instr_data_rdy;

always @ (*) begin: boundary_conditions
	integer i;
	fifo_mem[FIFO_DEPTH] = mem_data;
	fifo_predbranch[FIFO_DEPTH] = 2'b00;
	fifo_target[FIFO_DEPTH] = {W_ADDR{1'b0}};
	fifo_err[FIFO_DEPTH] = 1'b0;
	fifo_break_any[FIFO_DEPTH] = 2'b00;
	fifo_break_d_mode[FIFO_DEPTH] = 2'b00;
	for (i = 0; i <= FIFO_DEPTH; i = i + 1) begin
		fifo_valid[i] = |EXTENSION_C ? |fifo_valid_hw[i] : fifo_valid_hw[i][0];
		// valid-to-right 条件: i == 0 || fifo_valid[i - 1]，但不能
		// 使用负数组下界（在 Yosys 中似乎是坏的?）或短路情形下的
		// 越界（虽然结果良定义，但会报 lint）
		if (i == 0) begin
			fifo_valid_m1[i] = 1'b1;
		end else begin
			fifo_valid_m1[i] = fifo_valid[i - 1];
		end
	end
end

always @ (posedge clk or negedge rst_n) begin: fifo_update
	integer i;
	if (!rst_n) begin
		for (i = 0; i < FIFO_DEPTH; i = i + 1) begin
			fifo_valid_hw[i] <= 2'b00;
			fifo_mem[i] <= 32'd0;
			fifo_err[i] <= 1'b0;
			fifo_break_any[i] <= 2'b00;
			fifo_break_d_mode[i] <= 2'b00;
			fifo_predbranch[i] <= 2'b00;
			fifo_target[i] <= {W_ADDR{1'b0}};
		end
		// 它只为循环边界条件而存在，但在这个同步
		// 过程中绑定，以规避一个 Verilator 调度
		// 问题（见 issue #21）
		fifo_valid_hw[FIFO_DEPTH] <= 2'b00;
	end else begin
		for (i = 0; i < FIFO_DEPTH; i = i + 1) begin
			if (fifo_pop || (fifo_push && !fifo_valid[i])) begin
				fifo_mem[i]          <= fifo_valid[i + 1] ? fifo_mem[i + 1]          : mem_data;
				fifo_err[i]          <= fifo_valid[i + 1] ? fifo_err[i + 1]          : mem_or_pmp_err;
				fifo_break_any[i]    <= fifo_valid[i + 1] ? fifo_break_any[i + 1]    : mem_break_any;
				fifo_break_d_mode[i] <= fifo_valid[i + 1] ? fifo_break_d_mode[i + 1] : mem_break_d_mode;
				fifo_predbranch[i]   <= fifo_valid[i + 1] ? fifo_predbranch[i + 1]   : mem_data_predbranch;
				fifo_target[i]       <= fifo_valid[i + 1] ? fifo_target[i + 1]       : mem_data_target;
			end
			fifo_valid_hw[i] <=
				jump_now                                   ? 2'h0                            :
				fifo_valid[i + 1] && fifo_pop              ? fifo_valid_hw[i + 1]            :
				fifo_valid[i]     && fifo_pop              ? mem_data_hwvld & {2{fifo_push}} :
				fifo_valid[i]                              ? fifo_valid_hw[i]                :
				fifo_push && !fifo_pop && fifo_valid_m1[i] ? mem_data_hwvld                  : 2'h0;
		end
		// 允许 DM 把指令直接注入编号最小的
		// 队列表项。该多选器不应延长关键路径，
		// 因为它与队列旁路路径上的指令组装多选器
		// 相互平衡。注意冲刷优先于调试注入
		// （调试模块的设计必须考虑这一点）
		if (fifo_dbg_inject) begin
			fifo_mem[0] <= dbg_instr_data;
			fifo_err[0] <= 1'b0;
			fifo_predbranch[0] <= 2'b00;
			fifo_target[0] <= {W_ADDR{1'b0}};
			fifo_break_any[0] <= 2'b00;
			fifo_break_d_mode[0] <= 2'b00;
			fifo_valid_hw[0] <= jump_now ? 2'b00 : 2'b11;
		end
		fifo_valid_hw[FIFO_DEPTH] <= 2'b00;
	end
end

`ifdef OMNICORE_ASSERTIONS
always @ (posedge clk) if (rst_n) begin
	// FIFO 有效位必须紧凑，这样我们总能从末端消费
	if (!fifo_valid[0]) begin
		assert(!fifo_valid[1]);
	end
end
`endif

assign pwrdown_ok = (fifo_full && !jump_target_vld) || debug_mode;

// ----------------------------------------------------------------------------
// 分支目标缓冲器

wire [W_ADDR-1:0] btb_src_addr;
wire              btb_src_size;
wire [W_ADDR-1:0] btb_target_addr;
wire              btb_valid;

// 取指地址超前于 PC，按字递增。(模块级前置声明: default_nettype none 下
// ADS 单遍解析要求先声明后引用 -- 上方 generate 内的 BTB 实例与下方
// 取指地址寄存块都引用它, 不能放进 generate 分支作用域)
reg [W_ADDR-1:0] fetch_addr;

generate
if (BRANCH_PREDICTOR) begin: have_btb
	// ---------------------------------------------------------------------------
	// ZIGUANG 升级 (紫光同创赛题 高阶任务②: 动态分支预测器)
	// 用 N 表项、直接映射、带 2 位饱和方向计数器的 BTB，
	// 替换上游的单表项 BTB 寄存器块。对外
	// 连线语义（btb_valid/src_addr/src_size/target_addr）不变，
	// 且下方的 btb_match_word 仍然有效，因为查找命中
	// 在构造上就意味着字地址匹配。
	// 表项数来自配置头文件（ZG_BTB_ENTRIES）。
	// ---------------------------------------------------------------------------
	// N 表项阵列 + 计数器。阵列查找供下方的
	// 晋升（PROMOTION）逻辑使用; 预测路径则使用活动表项寄存器，
	// 使呈现给译码的目标在预测在途期间保持稳定。
	// （上游单表项 BTB 天然具有这种稳定性:
	// 其寄存器只从 X 级分配更新，
	// 而 X 级分配之前总会冲刷更年轻的指令。）
	wire              arr_hit;
	wire              arr_taken;
	wire [W_ADDR-1:0] arr_src_addr;
	wire              arr_src_size;
	wire [W_ADDR-1:0] arr_target;

	ominicore_btb #(
		.W_ADDR      (W_ADDR),
		.NUM_ENTRIES (ZG_BTB_ENTRIES)
	) btb (
		.clk                 (clk),
		.rst_n               (rst_n),
		.btb_set             (btb_set),
		.btb_set_src_addr    (btb_set_src_addr),
		.btb_set_src_size    (btb_set_src_size),
		.btb_set_target_addr (btb_set_target_addr),
		.btb_clear           (btb_clear),
		.btb_flush           (btb_flush),
		.lookup_addr         (fetch_addr),
		.lookup_hit          (arr_hit),
		.lookup_taken        (arr_taken),
		.lookup_src_addr     (arr_src_addr),
		.lookup_src_size     (arr_src_size),
		.lookup_target_addr  (arr_target),
		.evt_alloc           (zg_evt_br_alloc),
		.evt_flush_all       (zg_evt_br_flush),
		.evt_ctr_dec         (zg_evt_br_ctrdec)
	);

	// 活动预测（与上游等价的单表项）
	// ZIGUANG (C1, BP-02 多表项预测穿透): 预测路径改为数组查找直驱 --
	// 每个取指字按 fetch_addr 组合查找 BTB, 命中且计数器 taken 偏置即采用,
	// 目标在取指受理拍随预测元数据一起锁存, 经 FIFO/半字槽携带到译码
	// (btb_target_addr_out = CIR 槽 0 携带目标)。替换原单活动表项 a_*:
	// a_* 一次只能跟踪一个分支, 两个热分支交错时相互驱逐 (X 级 btb_set
	// 重分配), 造成高误预测率; 数组查找使全部表项同时参与预测。
	// 稳定性论证: 译码消费的是取指受理拍锁存的逐指令目标快照 (与取指
	// 重定向同拍同源), 不再依赖 "活动表项在途不变" 的上游不变量;
	// 误预测恢复仍由 X 级反相比较 + fall-through 跳转机制完成 (不变)。
	// 衰减语义由数组内计数器承担 (btb.v lookup_taken): 分配置 11, 每次
	// fall-through 递减, 10 仍预测 (单次不 taken 不驱逐热循环), 01 起停用,
	// 00 出表; fence.i/陷阱 btb_flush 清全部有效位 (BP-06)。
	assign btb_valid       = arr_hit && arr_taken;
	assign btb_src_addr    = arr_src_addr;
	assign btb_src_size    = arr_src_size;
	assign btb_target_addr = arr_target;
end else begin: no_btb
	assign btb_src_addr = {W_ADDR{1'b0}};
	assign btb_src_size = 1'b0;
	assign btb_target_addr = {W_ADDR{1'b0}};
	assign btb_valid = 1'b0;
	assign zg_evt_br_alloc = 1'b0;
	assign zg_evt_br_flush = 1'b0;
	assign zg_evt_br_ctrdec = 1'b0;
end
endgenerate

// 译码级利用目标地址，在预测跳转的分支之后把 PC 设为正确的
// 分支目标值（否则 PC 通常会跟随 X 级跳转请求更新，
// 而这种情形下并没有跳转请求）。
//
// ZIGUANG (C1): 目标 = CIR 槽 0 携带的预测目标快照 (取指受理拍锁存, 随
// FIFO/缓冲槽携带), 不再是 BTB 活动表项的活值 -- 逐指令稳定, 且天然支持
// 多个预测分支同时在途。32 位跨字分支两半字槽携带同一目标快照 (第二字
// 由 btb_prev_overhang_target 寄存传递)。
assign btb_target_addr_out = buf_contents[SLOT_TARGET_BIT +: W_ADDR];

// ----------------------------------------------------------------------------
// 取指请求生成

reg              fetch_priv;
reg              btb_prev_start_of_overhanging;
// ZIGUANG (C1): 跨字悬挂 32 位分支的第二个字 -- 预测标记沿用首字的目标
// 快照 (第二字按其自身 PC 查找必然 miss, 目标必须从首字寄存携带)。
reg [W_ADDR-1:0] btb_prev_overhang_target;
reg [1:0]        mem_aph_hwvld;
reg              mem_addr_hold;

// 悬挂目标寄存 (模块作用域: fetch_addr 块与取指标记块共同引用)
always @ (posedge clk or negedge rst_n) begin
	if (!rst_n)
		btb_prev_overhang_target <= {W_ADDR{1'b0}};
	else if (btb_match_next_addr)
		btb_prev_overhang_target <= btb_target_addr;
end

wire btb_match_word = |BRANCH_PREDICTOR && btb_valid && (
	fetch_addr[W_ADDR-1:2] == btb_src_addr[W_ADDR-1:2]
);

// 捕捉预测跳转的分支指令延伸到下一个字的情况:
wire btb_src_overhanging    = btb_src_size && btb_src_addr[1];

// 抑制以下情况: 在字对齐的半字大小分支之后立即跳转，
// 且因地址相保持使跳转目标进入了 fetch_addr:
wire btb_jumped_beyond      = !btb_src_size && !btb_src_addr[1] && !mem_aph_hwvld[0];

wire btb_match_current_addr = btb_match_word && !btb_src_overhanging && !btb_jumped_beyond;
assign btb_match_next_addr    = btb_match_word && btb_src_overhanging;

wire btb_match_now = btb_match_current_addr || btb_prev_start_of_overhanging;

// ZIGUANG (C1): 重定向目标选择。跨字悬挂分支的第二个字上 (prev_overhang=1)
// 逐字查找按 fa=第二个字 索引, 命中的是别的表项 (或 miss) -- 目标必须取
// 首字寄存的 btb_prev_overhang_target; a_* 时代靠活动表项驻留掩盖。
wire [W_ADDR-1:0] btb_redirect_target =
	btb_prev_start_of_overhanging ? btb_prev_overhang_target : btb_target_addr;

// 跳转请求直接穿透时地址后递增
wire [W_ADDR-1:0] jump_target_post_increment =
	{jump_target[W_ADDR-1:2],                          2'b00} +
	{{W_ADDR-3{1'b0}}, mem_addr_rdy && !mem_addr_hold, 2'b00};

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		fetch_addr <= RESET_VECTOR;
		// 复位时的 M 模式:
		fetch_priv <= 1'b1;
		btb_prev_start_of_overhanging <= 1'b0;
	end else begin
		if (jump_now) begin
			fetch_addr <= jump_target_post_increment;
			fetch_priv <= jump_priv || !U_MODE;
			btb_prev_start_of_overhanging <= 1'b0;
		end else if (mem_addr_vld && mem_addr_rdy) begin
			if (btb_match_now && |BRANCH_PREDICTOR) begin
				fetch_addr <= {btb_redirect_target[W_ADDR-1:2], 2'b00};
			end else begin
				fetch_addr <= fetch_addr + 32'd4;
			end
			btb_prev_start_of_overhanging <= btb_match_next_addr;
		end
	end
end

// 以组合逻辑生成地址相请求

reg reset_holdoff;
always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		reset_holdoff <= 1'b1;
	end else begin
		reset_holdoff <= (|EXTENSION_XH3POWER && delay_first_fetch) ? reset_holdoff : 1'b0;
		// 这本不应发生，但仍用断言确认，因为它*确实*
		// 会改变取指地址（若能证明它不会发生，
		// 就不该在硬件中检查它）
	end
end

`ifdef OMNICORE_ASSERTIONS
always @ (posedge clk) if (rst_n) begin
	assert(!(jump_target_vld && reset_holdoff));
end
`endif

reg [W_ADDR-1:0] mem_addr_r;
reg              mem_priv_r;
reg              mem_addr_vld_r;

// 下游访问始终是字大小、字对齐的。
assign mem_addr = mem_addr_r;
assign mem_priv = mem_priv_r;
assign mem_addr_vld = mem_addr_vld_r && !reset_holdoff;
assign mem_size = 1'b1;

wire fetch_stall;

always @ (*) begin
	mem_addr_r = fetch_addr;
	mem_priv_r = fetch_priv;
	mem_addr_vld_r = 1'b1;
	case (1'b1)
		mem_addr_hold                    : begin mem_addr_r = fetch_addr; end
		jump_target_vld || reset_holdoff : begin
		                                         mem_addr_r = {jump_target[W_ADDR-1:2], 2'b00};
		                                         mem_priv_r = jump_priv || !U_MODE;
		end
		DEBUG_SUPPORT && debug_mode      : begin mem_addr_vld_r = 1'b0; end
		!fetch_stall                     : begin mem_addr_r = fetch_addr; end
		default                          : begin mem_addr_vld_r = 1'b0; end
	endcase
end

assign jump_target_rdy = !mem_addr_hold;

// ----------------------------------------------------------------------------
// 总线流水线跟踪

// 记录内存接口的一些有用状态

reg  [1:0] pending_fetches;
reg  [1:0] ctr_flush_pending;

wire [1:0] pending_fetches_next = pending_fetches + (mem_addr_vld && !mem_addr_hold) - mem_data_vld;

// 使用未寄存的 pending_fetches 版本可以提高 FIFO
// 利用率，但会形成从 hready 到地址相的组合路径！
// 这意味着满取指吞吐至少需要 2 字深的 FIFO。
assign fetch_stall = fifo_full
	|| fifo_almost_full && |pending_fetches
	|| pending_fetches > 2'h1;

// 调试器只在前端静止且为空时注入指令。
assign dbg_instr_data_rdy = DEBUG_SUPPORT && !fifo_valid[0] && ~|ctr_flush_pending;

wire cir_room_for_fetch;
// 若取指数据旁路绕过 FIFO，需确保它不同时被写入 FIFO。
assign fifo_push = mem_data_vld && ~|ctr_flush_pending && !(cir_room_for_fetch && fifo_empty)
	&& !(DEBUG_SUPPORT && debug_mode);

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		mem_addr_hold <= 1'b0;
		pending_fetches <= 2'h0;
		ctr_flush_pending <= 2'h0;
	end else begin
		mem_addr_hold <= mem_addr_vld && !mem_addr_rdy;
		pending_fetches <= pending_fetches_next;
		if (jump_now) begin
			ctr_flush_pending <= pending_fetches - mem_data_vld;
		end else if (|ctr_flush_pending && mem_data_vld) begin
			ctr_flush_pending <= ctr_flush_pending - 1'b1;
		end
	end
end

`ifdef OMNICORE_ASSERTIONS
always @ (posedge clk) if (rst_n) begin
	assert(ctr_flush_pending <= pending_fetches);
	assert(pending_fetches < 2'd3);
	assert(!(mem_data_vld && !pending_fetches));
end
`endif

`ifdef B2_DEBUG
	always @ (posedge clk) begin
		if (mem_addr_vld && mem_addr_rdy)
			$display("[FE] fa=%08x mw=%b mn=%b valid=%b src=%08x sz=%b tgt=%08x po=%b ph=%b t=%08x",
				fetch_addr, btb_match_word, btb_match_now, btb_valid,
				btb_src_addr, btb_src_size, btb_target_addr,
				btb_src_overhanging, btb_prev_start_of_overhanging, mem_data_target);
		if (jump_now)
			$display("[FJMP] tgt=%08x", jump_target);
	end
`endif

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		mem_data_hwvld <= 2'b11;
		mem_aph_hwvld <= 2'b11;
		mem_data_predbranch <= 2'b00;
		mem_data_target <= {W_ADDR{1'b0}};
	end else begin
		if (jump_now) begin
			mem_data_target <= {W_ADDR{1'b0}};
			if (|EXTENSION_C) begin
				if (mem_addr_rdy) begin
					mem_aph_hwvld <= 2'b11;
					mem_data_hwvld <= {1'b1, !jump_target[1]};
				end else begin
					mem_aph_hwvld <= {1'b1, !jump_target[1]};
				end
			end
			mem_data_predbranch <= 2'b00;
		end else if (mem_addr_vld && mem_addr_rdy) begin
			if (|EXTENSION_C) begin
				// 若预测跳转的分支指令只占据
				// 一个字的前一半，需要把后一半标记为无效。
				// ZIGUANG (C1): 悬挂拍 (prev_overhang) 下本字上半字 = 跨字分支
				// 第二半字, 必须无效化 -- src[1]==src_size 事实来自首字表项,
				// 本字逐字查找是 miss (垃圾 src), 不能采信。
				mem_data_hwvld <= mem_aph_hwvld & {
					!(|BRANCH_PREDICTOR && btb_match_now &&
						(btb_prev_start_of_overhanging ? 1'b1 : (btb_src_addr[1] == btb_src_size))),
					1'b1
				};
				// 还需要考虑目标的地址对齐。
				mem_aph_hwvld <= {
					1'b1,
					!(|BRANCH_PREDICTOR && btb_match_now && btb_redirect_target[1])
				};
			end
			mem_data_predbranch <=
				|BRANCH_PREDICTOR && btb_match_word ? (
					btb_src_addr[1] ? 2'b10 :
					btb_src_size    ? 2'b11 : 2'b01
				) :
				|BRANCH_PREDICTOR && btb_prev_start_of_overhanging ? (
					2'b01
				) : 2'b00;
			// ZIGUANG (C1): 目标与预测元数据同拍锁存 -- 取指受理拍的
			// 查找结果即为取指重定向所用目标 (同源), 随 FIFO 携带到译码,
			// 替代原 "译码读活动表项" 的在途稳定依赖。
			mem_data_target <=
				|BRANCH_PREDICTOR && btb_match_word ? btb_target_addr :
				|BRANCH_PREDICTOR && btb_prev_start_of_overhanging ? btb_prev_overhang_target :
				{W_ADDR{1'b0}};
		end
	end
end

// ----------------------------------------------------------------------------
// PMP 与触发单元对接: 查询 -> 击杀/打断

wire [W_ADDR-1:0] pmp_trigger_check_dph_addr;
wire              pmp_trigger_check_dph_m_mode;

// 将取指地址寄存进 F 级，使 PMP 能与总线数据相
// 并行检查。单独设一个寄存器似乎浪费，
// 但直接用 fetch_addr 计数器风险很大，因为新地址
// 进入它还是绕过它取决于地址相是否保持。

generate
if (PMP_REGIONS > 0 || DEBUG_SUPPORT != 0) begin: have_check_reg

	reg [W_ADDR-1:0] check_addr_dph;
	reg              check_m_mode_dph;
	always @ (posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			check_addr_dph <= {W_ADDR{1'b0}};
			check_m_mode_dph <= 1'b0;
		end else if (mem_addr_vld && mem_addr_rdy) begin
			check_addr_dph <= mem_addr;
			check_m_mode_dph <= mem_priv;
		end
	end

	assign pmp_trigger_check_dph_addr = check_addr_dph;
	assign pmp_trigger_check_dph_m_mode = check_m_mode_dph;

end else begin: no_check_reg

	assign pmp_trigger_check_dph_addr = {W_ADDR{1'b0}};
	assign pmp_trigger_check_dph_m_mode = 1'b0;

end
endgenerate

generate
if (PMP_REGIONS == 0) begin: no_pmp

	assign pmp_i_addr = {W_ADDR{1'b0}};
	assign pmp_i_m_mode = 1'b0;
	assign pmp_kill_fetch_dph = 1'b0;

end else begin: have_pmp

	assign pmp_i_addr = pmp_trigger_check_dph_addr;
	assign pmp_i_m_mode = pmp_trigger_check_dph_m_mode;
	assign pmp_kill_fetch_dph = pmp_i_kill && !debug_mode;

end
endgenerate

generate
if (DEBUG_SUPPORT == 0) begin: no_triggers

	assign trigger_addr = {W_ADDR{1'b0}};
	assign trigger_m_mode = 1'b0;
	assign mem_break_any = 2'b00;
	assign mem_break_d_mode = 2'b00;

end else begin: have_triggers

	assign trigger_addr = pmp_trigger_check_dph_addr;
	assign trigger_m_mode = pmp_trigger_check_dph_m_mode;
	assign mem_break_any = trigger_break_any & {|EXTENSION_C, 1'b1};
	assign mem_break_d_mode = trigger_break_d_mode & {|EXTENSION_C, 1'b1};

end
endgenerate

// ----------------------------------------------------------------------------
// 指令缓冲

// 指令缓冲是一个 3 x ~16 位的移位寄存器:
//
// * 2 x 16 位表项构成 32 位当前指令寄存器（CIR），
//   即处理器的译码窗口
//
// * 1 x 16 位表项使译码窗口可以不按 32 位对齐，
//   相对于 2 x 32 位预取队列表项而言 —— 后者在内存中
//   始终自然对齐（若全部有效）。
//
// 对非 RVC 配置，第三个表项应被修剪，
// 这是 EXTENSION_C 常量折叠的效果; 在该配置下
// 指令始终 32 位对齐，第三表项并不需要。

// 各表项（"槽"）略大于 16 位，因为它们还包含
// 总线错误之类的元数据:
// ZIGUANG (C1): 布局声明见模块前部 (W_SLOT/SLOT_TARGET_BIT 等)。

reg  [1:0]          buf_level;

wire                fetch_data_vld   = !fifo_empty || (mem_data_vld && ~|ctr_flush_pending && !debug_mode);

wire [W_DATA-1:0]   fetch_data         = fifo_empty ? mem_data            : fifo_rdata;
wire [1:0]          fetch_data_hwvld   = fifo_empty ? mem_data_hwvld      : fifo_valid_hw[0];
wire                fetch_bus_err      = fifo_empty ? mem_or_pmp_err      : fifo_err[0];
wire [1:0]          fetch_break_any    = fifo_empty ? mem_break_any       : fifo_break_any[0];
wire [1:0]          fetch_break_d_mode = fifo_empty ? mem_break_d_mode    : fifo_break_d_mode[0];
wire [1:0]          fetch_predbranch   = fifo_empty ? mem_data_predbranch : fifo_predbranch[0];
wire [W_ADDR-1:0]   fetch_target       = fifo_empty ? mem_data_target     : fifo_target[0];

wire [W_SLOT-1:0] fetch_contents_hw1 = {
	fetch_target,
	fetch_break_any[1],
	fetch_break_d_mode[1],
	fetch_bus_err,
	fetch_predbranch[1],
	fetch_data[W_BUNDLE +: W_BUNDLE]
};

wire [W_SLOT-1:0] fetch_contents_hw0 = {
	fetch_target,
	fetch_break_any[0],
	fetch_break_d_mode[0],
	fetch_bus_err,
	fetch_predbranch[0],
	fetch_data[0 +: W_BUNDLE]
};

wire [2*W_SLOT-1:0] fetch_contents_aligned = {
	fetch_contents_hw1,
	fetch_data_hwvld[0] || ~|EXTENSION_C ? fetch_contents_hw0 : fetch_contents_hw1
};

// 把尚未使用的内容下移，回填 D 级的消耗。我们不关心
// 无效的、或将被新数据覆盖的内容，因此
// 以最小化多选器的方式选取这些值。
wire [3*W_SLOT-1:0] buf_shifted =
	cir_use[1]                ? {buf_contents[W_SLOT +: 2 * W_SLOT], buf_contents[2 * W_SLOT +: W_SLOT]} :
	cir_use[0] && EXTENSION_C ? {buf_contents[2 * W_SLOT +: W_SLOT], buf_contents[W_SLOT +: 2 * W_SLOT]} :
	                            buf_contents;

wire [1:0] level_next_no_fetch = buf_level - cir_use;

// 将新取指数据覆盖到移位/回收后的缓冲内容上。同样，
// 不会被看到的内容就生成最廉价的垃圾值。
assign cir_room_for_fetch = level_next_no_fetch <= (|EXTENSION_C && ~&fetch_data_hwvld ? 2'h2 : 2'h1);
assign fifo_pop = cir_room_for_fetch && !fifo_empty;

wire [3*W_SLOT-1:0] buf_shifted_plus_fetch =
	!cir_room_for_fetch                    ? buf_shifted :
	level_next_no_fetch[1] && |EXTENSION_C ? {fetch_contents_aligned[0 +: W_SLOT], buf_shifted[0 +: 2 * W_SLOT]} :
	level_next_no_fetch[0] && |EXTENSION_C ? {fetch_contents_aligned, buf_shifted[0 +: W_SLOT]} :
	                                         {buf_shifted[2 * W_SLOT +: W_SLOT], fetch_contents_aligned};

wire [1:0] fetch_fill_amount = cir_room_for_fetch && fetch_data_vld ? (
	&fetch_data_hwvld || ~|EXTENSION_C ? 2'h2 : 2'h1
) : 2'h0;

wire [1:0] buf_level_next = {1'b1, |EXTENSION_C} & (
	jump_now && cir_flush_behind ? (cir_is_32bit ? 2'h2 : 2'h1) :
	jump_now                     ? 2'h0 : level_next_no_fetch + fetch_fill_amount
);

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		buf_level <= 2'h0;
		cir_vld <= 2'h0;
		// 神秘的复位值是为了保证复位期间地址总线为零
		// （见 ominicore_decode 中 d_addr_offs 的定义）
		buf_contents <= {{3 * W_SLOT - 2{1'b0}}, 2'b11};
	end else begin
		buf_level <= buf_level_next;
		cir_vld <= buf_level_next & ~(buf_level_next >> 1'b1);
		buf_contents <= buf_shifted_plus_fetch;
	end
end

`ifdef OMNICORE_ASSERTIONS
reg [1:0] prop_past_buf_level; // 规避 $past 复位值非常量的怪问题
always @ (posedge clk) begin
	if (!rst_n) begin
		prop_past_buf_level <= 2'h0;
	end else begin
		prop_past_buf_level <= buf_level;

		assert(cir_vld <= 2);
		assert(cir_use <= cir_vld);
		if (!jump_now) assert(buf_level_next >= level_next_no_fetch);
		// 每拍最多取 32 位。若发生此情况，是向下溢出所致。
		if (prop_past_buf_level == 2'h0)
			assert(buf_level != 2'h3);
	end
end
`endif

assign cir_err = {
	buf_contents[1 * W_SLOT + SLOT_ERR_BIT],
	buf_contents[0 * W_SLOT + SLOT_ERR_BIT]
};

assign cir_predbranch = {
	buf_contents[1 * W_SLOT + SLOT_PREDBRANCH_BIT],
	buf_contents[0 * W_SLOT + SLOT_PREDBRANCH_BIT]
};

assign cir_break_any = buf_contents[0 * W_SLOT + SLOT_BREAK_ANY_BIT] && |cir_vld;

assign cir_break_d_mode = buf_contents[0 * W_SLOT + SLOT_BREAK_D_MODE_BIT] && |cir_vld;

// ----------------------------------------------------------------------------
// 寄存器号预译码

wire [31:0] next_instr = {
	buf_shifted_plus_fetch[1 * W_SLOT +: W_BUNDLE],
	buf_shifted_plus_fetch[0 * W_SLOT +: W_BUNDLE]
};

wire next_instr_is_32bit = next_instr[1:0] == 2'b11 || ~|EXTENSION_C;

wire [3:0] decomp_uop_step;
wire [3:0] uop_ctr = decomp_uop_step & {4{|EXTENSION_ZCMP}};

wire [4:0] zcmp_pushpop_rs2 =
	uop_ctr == 4'h0 ? 5'd01                   : // ra
	uop_ctr == 4'h1 ? 5'd08                   : // s0
	uop_ctr == 4'h2 ? 5'd09                   : // s1
	                  5'd15 + {1'b0, uop_ctr} ; // s2-s11

wire [4:0] zcmp_pushpop_rs1 =
	uop_ctr <  4'hd ? 5'd02 :                   // sp   (地址基址寄存器)
	uop_ctr == 4'hd ? 5'd00 :                   // zero (清零 a0)
	uop_ctr == 4'he ? 5'd01 :                   // ra   (用于 ret)
	                  5'd02 ;                   // sp   (栈调整)

wire [4:0] zcmp_sa01_r1s  = {|next_instr[9:8], ~|next_instr[9:8], next_instr[9:7]};
wire [4:0] zcmp_sa01_r2s  = {|next_instr[4:3], ~|next_instr[4:3], next_instr[4:2]};

wire [4:0] zcmp_mvsa01_rs1 = {4'h5, uop_ctr[0]};
wire [4:0] zcmp_mva01s_rs1 = uop_ctr[0] ? zcmp_sa01_r2s : zcmp_sa01_r1s;

// 称"粗粒度"是因为 (x0, x1) -> (x0, x0) 的映射尚未施加
wire [4:0] zilsd_rs2_coarse      = {       next_instr[24:21], df_lspair_phase_next ^ ~next_instr[15]};
wire [4:0] zclsd_sd_rs2_coarse   = {2'b01, next_instr[4:3],   df_lspair_phase_next ^ ~next_instr[7] };
wire [4:0] zclsd_sdsp_rs2_coarse = {       next_instr[6:3],   df_lspair_phase_next ^ 1'b1           };

always @ (*) begin

	casez ({next_instr_is_32bit, |EXTENSION_ZCMP, next_instr[15:0]})
	{1'b1, 1'bz, 16'bzzzzzzzzzzzzzzzz}: predecode_rs1_coarse = next_instr[19:15]; // 32 位 R、S、B 格式
	{1'b0, 1'bz, 16'b00zzzzzzzzzzzz00}: predecode_rs1_coarse = 5'd2;              // c.addi4spn + 无关项
	{1'b0, 1'bz, 16'b0zzzzzzzzzzzzz01}: predecode_rs1_coarse = next_instr[11:7];  // c.addi, c.addi16sp + 无关项 (jal, li)
	{1'b0, 1'bz, 16'bz1zzzzzzzzzzzz10}: predecode_rs1_coarse = 5'd2;              // c.lwsp, c.swsp, c.ldsp, c.sdsp
	{1'b0, 1'bz, 16'bz00zzzzzzzzzzz10}: predecode_rs1_coarse = next_instr[11:7];  // c.slli, c.mv, c.add
	{1'b0, 1'b1, 16'b1011zzzzzzzzzz10}: predecode_rs1_coarse = zcmp_pushpop_rs1;  // cm.push, cm.pop*
	{1'b0, 1'b1, 16'b1010zzzzz0zzzz10}: predecode_rs1_coarse = zcmp_mvsa01_rs1;   // cm.mvsa01
	{1'b0, 1'b1, 16'b1010zzzzz1zzzz10}: predecode_rs1_coarse = zcmp_mva01s_rs1;   // cm.mva01s
	default:                            predecode_rs1_coarse = {2'b01, next_instr[9:7]};
	endcase

	casez ({next_instr_is_32bit, |EXTENSION_ZCMP, |EXTENSION_ZILSD, |EXTENSION_ZCLSD, next_instr[15:0]})
	{1'b1, 1'bz, 1'b1, 1'bz, 16'bzz11zzzzz0z0zzzz}: predecode_rs2_coarse = zilsd_rs2_coarse;   // ld, sd (Zilsd)
	{1'b1, 1'bz, 1'b0, 1'bz, 16'bzz11zzzzz0z0zzzz}: predecode_rs2_coarse = next_instr[24:20];  // ld, sd（无 Zilsd）

	{1'b1, 1'bz, 1'bz, 1'bz, 16'bzz0zzzzzzzzzzzzz}: predecode_rs2_coarse = next_instr[24:20];  // （覆盖剩余的 32 位
	{1'b1, 1'bz, 1'bz, 1'bz, 16'bzz10zzzzzzzzzzzz}: predecode_rs2_coarse = next_instr[24:20];  //  模式，且互不重叠）
	{1'b1, 1'bz, 1'bz, 1'bz, 16'bzz11zzzzz1zzzzzz}: predecode_rs2_coarse = next_instr[24:20];
	{1'b1, 1'bz, 1'bz, 1'bz, 16'bzz11zzzzz0z1zzzz}: predecode_rs2_coarse = next_instr[24:20];

	{1'b0, 1'bz, 1'b1, 1'b1, 16'bzz1zzzzzzzzzzz00}: predecode_rs2_coarse = zclsd_sd_rs2_coarse;
	{1'b0, 1'bz, 1'bz, 1'bz, 16'bzz0zzzzzzzzzzz10}: predecode_rs2_coarse = next_instr[6:2];    // c.add, c.swsp
	{1'b0, 1'b1, 1'bz, 1'bz, 16'bz01zzzzzzzzzzz10}: predecode_rs2_coarse = zcmp_pushpop_rs2;   // cm.push
	{1'b0, 1'bz, 1'b1, 1'b1, 16'bz11zzzzzzzzzzz10}: predecode_rs2_coarse = zclsd_sdsp_rs2_coarse;
	default:                                        predecode_rs2_coarse = {2'b01, next_instr[4:2]};
	endcase

	// "细粒度"预译码面向以下指令:
	// - 展开形式中含隐式零寄存器操作数者（如 c.beqz）
	// - 该端口本身没有寄存器操作数、但依赖端口值为 0 者
	// 至于无视寄存器端口的指令（如 ebreak）则无需关心

	casez ({|EXTENSION_C, next_instr})
	// -> addi rd, x0, imm:
	{1'b1, 16'hzzzz, `RVOPC_C_LI}: predecode_rs1_fine = 5'd0;
	{1'b1, 16'hzzzz, `RVOPC_C_MV}: begin
		if (next_instr[6:2] == 5'd0) begin
			// c.jr 的 rs1 与常规一致
			predecode_rs1_fine = predecode_rs1_coarse;
		end else begin
			// -> add rd, x0, rs2:
			predecode_rs1_fine = 5'd0;
		end
	end
	default: predecode_rs1_fine = predecode_rs1_coarse;
	endcase

	casez ({|EXTENSION_C, |EXTENSION_ZILSD, |EXTENSION_ZCLSD, next_instr})
	{1'b1, 1'bz, 1'bz, 16'hzzzz, `RVOPC_C_BEQZ}: predecode_rs2_fine = 5'd0;    // -> beq rs1, x0, label
	{1'b1, 1'bz, 1'bz, 16'hzzzz, `RVOPC_C_BNEZ}: predecode_rs2_fine = 5'd0;    // -> bne rs1, x0, label
	{1'b1, 1'b1, 1'bz,           `RVOPC_SD    }: predecode_rs2_fine = predecode_rs2_coarse & {5{|next_instr[24:21]}};
	{1'b1, 1'b1, 1'b1, 16'hzzzz, `RVOPC_C_SDSP}: predecode_rs2_fine = predecode_rs2_coarse & {5{|next_instr[ 6: 3]}};
	default:                                     predecode_rs2_fine = predecode_rs2_coarse;
	endcase

	if (|EXTENSION_E) begin
		predecode_rs1_coarse[4] = 1'b0;
		predecode_rs2_coarse[4] = 1'b0;
		predecode_rs1_fine[4] = 1'b0;
		predecode_rs2_fine[4] = 1'b0;
	end

end

// ----------------------------------------------------------------------------
// 指令解压

// 指令在第 1 级末尾（取指数据相）解压。在 ASIC 上，
// 寄存器堆用多选器综合而成，解压与寄存器堆读
// 在一定程度上并行进行 —— 后者使用
// 近似译码的寄存器号。

generate
if (~|EXTENSION_C) begin: no_decompress

	// 无解压; 直接从预取缓冲译码指令
	always @ (*) begin
		cir = {
			buf_contents[1 * W_SLOT +: W_BUNDLE],
			buf_contents[0 * W_SLOT +: W_BUNDLE]
		} | 32'd3;
		cir_is_32bit         = 1'b1;
		cir_invalid_16bit    = ~&buf_contents[1:0];
		cir_is_uop           = 1'b0;
		cir_uop_nonfinal     = 1'b0;
		cir_uop_no_pc_update = 1'b0;
		cir_uop_atomic       = 1'b0;
	end	

	assign decomp_uop_step = 4'h0;

end else begin: have_decompress

	wire        decomp_instr_is_32bit;
	wire [31:0] decomp_instr_out;
	wire        decomp_is_uop;
	wire        decomp_is_final_uop;
	wire        decomp_uop_no_pc_update;
	wire        decomp_uop_atomic;
	wire        decomp_invalid;

	wire        first_uop           = ~|decomp_uop_step;
	// 确保第一个 uop 直接穿透，因为它要被寄存进 CIR:
	wire        uop_stall_non_first = first_uop ? ~|buf_level_next : uop_stall;
	// 确保 uop 计数器回绕一次后停在 0:
	wire        uop_stall_on_repeat = cir_is_uop && !cir_uop_nonfinal && ~|cir_use;

	ominicore_instr_decompress #(
		`include "ominicore_config_inst.vh"
	) decomp (
		.clk                        (clk),
		.rst_n                      (rst_n),

		.instr_in                   (next_instr),

		.instr_is_32bit             (decomp_instr_is_32bit),
		.instr_out                  (decomp_instr_out),

		.instr_out_is_uop           (decomp_is_uop),
		.instr_out_is_final_uop     (decomp_is_final_uop),
		.instr_out_uop_no_pc_update (decomp_uop_no_pc_update),
		.instr_out_uop_atomic       (decomp_uop_atomic),
		.instr_out_uop_stall        (uop_stall_non_first || uop_stall_on_repeat),
		.instr_out_uop_clear        (uop_clear),
		.df_uop_step                (decomp_uop_step),

		.invalid                    (decomp_invalid)
	);

	wire cir_clken =
		~|cir_vld || (!cir_vld[1] && &buf_contents[1:0]) ||
		|cir_use  || (|EXTENSION_ZCMP && cir_is_uop && !uop_stall) ||
		(|EXTENSION_ZCMP && uop_clear);

	wire cir_is_uop_next = decomp_is_uop && |buf_level_next;
	wire cir_uop_nonfinal_next = cir_is_uop_next && !decomp_is_final_uop;

	always @ (posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			cir                  <= 32'd3;
			cir_is_32bit         <= 1'b0;
			cir_invalid_16bit    <= 1'b0;
			cir_is_uop           <= 1'b0;
			cir_uop_nonfinal     <= 1'b0;
			cir_uop_no_pc_update <= 1'b0;
			cir_uop_atomic       <= 1'b0;
		end else if (cir_clken) begin
			cir                  <= decomp_instr_out | 32'd3;
			cir_is_32bit         <= decomp_instr_is_32bit;
			cir_invalid_16bit    <= decomp_invalid;
			cir_is_uop           <= |EXTENSION_ZCMP && !uop_clear && cir_is_uop_next;
			cir_uop_nonfinal     <= |EXTENSION_ZCMP && !uop_clear && cir_uop_nonfinal_next;
			cir_uop_no_pc_update <= |EXTENSION_ZCMP && !uop_clear && decomp_uop_no_pc_update;
			cir_uop_atomic       <= |EXTENSION_ZCMP && !uop_clear && decomp_uop_atomic;
		end
	end

end
endgenerate

assign cir_raw = {
	buf_contents[1 * W_SLOT +: W_BUNDLE],
	buf_contents[0 * W_SLOT +: W_BUNDLE]
};

endmodule

`ifndef YOSYS
`default_nettype wire
`endif
