

`ifdef OMNICORE_RVFI_STANDALONE
`include "ominicore_rvfi_standalone_defs.vh"
`endif

`default_nettype none

module ominicore_core #(
`include "ominicore_config.vh"
,
`include "ominicore_width_const.vh"
) (
	// 全局信号
	input wire                 clk,
	input wire                 clk_always_on,
	input wire                 rst_n,

	// 电源控制信号
	output wire                pwrup_req,
	input  wire                pwrup_ack,
	output wire                clk_en,
	output reg                 unblock_out,
	input  wire                unblock_in,

	`ifdef RISCV_FORMAL
	`RVFI_OUTPUTS ,
	`endif

	// 取指端口
	output wire                bus_aph_req_i,
	output wire                bus_aph_panic_i, // 例如分支误预测 + 冲刷
	input  wire                bus_aph_ready_i,
	input  wire                bus_dph_ready_i,
	input  wire                bus_dph_err_i,

	output wire [W_ADDR-1:0]   bus_haddr_i,
	output wire [2:0]          bus_hsize_i,
	output wire                bus_priv_i,
	input  wire [W_DATA-1:0]   bus_rdata_i,

	// load/store 端口
	output reg                 bus_aph_req_d,
	output wire                bus_aph_excl_d,
	input  wire                bus_aph_ready_d,
	input  wire                bus_dph_ready_d,
	input  wire                bus_dph_err_d,
	input  wire                bus_dph_exokay_d,

	output reg  [W_ADDR-1:0]   bus_haddr_d,
	output reg  [2:0]          bus_hsize_d,
	output reg                 bus_priv_d,
	output reg                 bus_hwrite_d,
	output reg  [W_DATA-1:0]   bus_wdata_d,
	input  wire [W_DATA-1:0]   bus_rdata_d,

	// 内存序信号
	output wire                fence_i_vld,
	output wire                fence_d_vld,
	input  wire                fence_rdy,

	// 调试器运行/停机控制
	input  wire                dbg_req_halt,
	input  wire                dbg_req_halt_on_reset,
	input  wire                dbg_req_resume,
	output wire                dbg_halted,
	output wire                dbg_running,
	// 调试器对 data0 CSR 的访问
	input  wire [W_DATA-1:0]   dbg_data0_rdata,
	output wire [W_DATA-1:0]   dbg_data0_wdata,
	output wire                dbg_data0_wen,
	// 调试器指令注入
	input  wire [W_DATA-1:0]   dbg_instr_data,
	input  wire                dbg_instr_data_vld,
	output wire                dbg_instr_data_rdy,
	output wire                dbg_instr_caught_exception,
	output wire                dbg_instr_caught_ebreak,

	// 标识（ID）CSR 值
	input  wire [W_DATA-1:0]   mhartid_val,
	input  wire [3:0]          eco_version,

	// 电平触发中断源
	input  wire [NUM_IRQS-1:0] irq,       // -> mip.meip
	input  wire                soft_irq,  // -> mip.msip
	input  wire                timer_irq,  // -> mip.mtip

	// ZIGUANG: 分支预测器统计抽头（自前端透传）
	output wire                zg_evt_br_alloc,
	output wire                zg_evt_br_flush,
	output wire                zg_evt_br_ctrdec,

	// ZIGUANG (规格 §10 PMU-01~04): cache/总线事件脉冲, 由 cache 化顶层接入
	input  wire                zg_pmu_icaccess,
	input  wire                zg_pmu_icmiss,
	input  wire                zg_pmu_dcaccess,
	input  wire                zg_pmu_dcmiss,
	input  wire                zg_pmu_buswait
);

wire                 zg_evt_br_alloc_w;   // (2026-10-09 前移: vlog 严格先声明后使用)
wire                 zg_evt_br_flush_w;
wire                 zg_evt_br_ctrdec_w;

assign zg_evt_br_alloc  = zg_evt_br_alloc_w;
`ifdef ZG_PCTRACE
// CIR PC 序列轨迹: d_pc 变化时打印（用于上游/优化版指令流 diff）
reg [31:0] zg_pc_prev = 32'hffff_ffff;
always @ (posedge clk) if (rst_n)
	zg_pc_prev <= d_pc;
always @ (posedge clk) begin
	if (rst_n && d_pc != zg_pc_prev)
		$display("[PC %08x]", d_pc);
end
`endif


`ifdef ZG_TRACE_PC
// 野跳转检测：PC 离开 .text 进入数据区时打印最近 8 个 PC
reg [31:0] zg_hist [0:7];
reg [31:0] zg_ctr = 32'h0;
always @ (posedge clk) if (rst_n) begin
	zg_ctr <= zg_ctr + 1;
	if (d_pc >= 32'h80012000 && d_pc < 32'h800fe000) begin
		$display("[WILD %0d] pc=%08x hist=%08x %08x %08x %08x %08x %08x %08x %08x",
			zg_ctr, d_pc, zg_hist[0], zg_hist[1], zg_hist[2], zg_hist[3],
			zg_hist[4], zg_hist[5], zg_hist[6], zg_hist[7]);
	end
	zg_hist[7] <= zg_hist[6];
	zg_hist[6] <= zg_hist[5];
	zg_hist[5] <= zg_hist[4];
	zg_hist[4] <= zg_hist[3];
	zg_hist[3] <= zg_hist[2];
	zg_hist[2] <= zg_hist[1];
	zg_hist[1] <= zg_hist[0];
	zg_hist[0] <= d_pc;
end
// 心跳: 每 2^14 拍打印当前 d_pc, 用于定位死循环
reg [31:0] zg_hb = 32'h0;
always @ (posedge clk) if (rst_n) begin
	zg_hb <= zg_hb + 32'h1;
	if (zg_hb[13:0] == 14'h3fff)
		$display("[HB %08x] pc=%08x", zg_hb, d_pc);
end
`endif

`ifdef ZG_WATCH_REGW
// 寄存器写回监视：打印指定周期窗口内所有寄存器堆写回（排查写回索引串扰）
reg [31:0] zg_rw_ctr = 32'h0;
always @ (posedge clk) begin
	zg_rw_ctr <= zg_rw_ctr + 32'h1;
	if (m_reg_wen && zg_rw_ctr >= 32'h2c0 && zg_rw_ctr <= 32'h380)
		$display("[REGW ctr=%08x] x%02d <= %08x",
			zg_rw_ctr, xm_rd, m_result);
end
`endif

`ifdef ZG_WATCH_X
initial $display("XBLOCK-ALIVE");
// 首个 X 追踪: 找到 X 进入核心的第一拍
reg zg_x_found = 1'b0;
reg [31:0] zg_xcyc = 32'h0;
always @ (posedge clk) begin
	zg_xcyc <= zg_xcyc + 32'h1;
	if ((zg_xcyc > 32'd1000 && !zg_x_found && (
		^bus_rdata_d === 1'bx ||
		^fd_cir === 1'bx ||
		^bus_aph_req_d === 1'bx ||
		^d_pc === 1'bx ||
		(^frontend.buf_contents === 1'bx) ||
		(^frontend.mem_data === 1'bx))) || zg_xcyc[19:0] == 20'h00000) begin
		zg_x_found <= 1'b1;
		$display("[FIRSTX cyc=%0d] rdata=%h cir=%h req=%b pc=%h mem_data=%h faddr=%h fa_vld=%b",
			zg_xcyc, bus_rdata_d, fd_cir, bus_aph_req_d, d_pc,
			frontend.mem_data, frontend.buf_contents, frontend.buf_level);
	end
end
`endif

`ifdef ZG_WATCH_BYPASS
// 旁路决策探针：跟踪 __libc_init_array 栈帧建立（0x80000932 ~ 0x80000960）
reg [31:0] zg_by_ctr = 32'h0;
reg        zg_by_arm = 1'b0;
always @ (posedge clk) begin
	zg_by_ctr <= zg_by_ctr + 32'h1;
	if (d_pc == 32'h80000932)
		zg_by_arm <= 1'b1;
	if (zg_by_arm)
		$display("[BYP %0d] pc=%08x sel2=%b d_rs2p=%02d d_rs2=%02d xm_rd=%02d mw_rd=%02d xm_res=%08x mw_res=%08x xstall=%b dstarv=%b wen=%b rd=%02d res=%08x req_d=%b rdy_d=%b f_co=%02d xr2=%08x rfr2=%08x",
			zg_by_ctr, d_pc, x_rs2_select_regs_xm_mw, d_rs2_predecoded, d_rs2,
			xm_rd, mw_rd, xm_result, mw_result, x_stall, d_starved,
			m_reg_wen, m_reg_wen ? xm_rd : 5'd31, m_result,
			bus_aph_req_d, bus_aph_ready_d,
			f_rs2_coarse, x_rdata2, regs.rdata2);
	if (zg_by_arm && d_pc == 32'h8000095c)
		zg_by_arm <= 1'b0;
end
`endif

assign zg_evt_br_flush  = zg_evt_br_flush_w;
assign zg_evt_br_ctrdec = zg_evt_br_ctrdec_w;

`include "ominicore_ops.vh"
`include "rv_opcodes.vh"

wire x_stall;
wire m_stall;

localparam HSIZE_WORD  = 3'd2;
localparam HSIZE_HWORD = 3'd1;
localparam HSIZE_BYTE  = 3'd0;

localparam [4:0] REGADDR_MASK = {~|EXTENSION_E, 4'hf};

wire debug_mode;
assign dbg_halted = DEBUG_SUPPORT && debug_mode;
assign dbg_running = DEBUG_SUPPORT && !debug_mode;

// ----------------------------------------------------------------------------
// 流水线 F 级

wire                 f_jump_req;
wire [W_ADDR-1:0]    f_jump_target;
wire                 f_jump_priv;
wire                 f_jump_rdy;
wire                 f_jump_now = f_jump_req && f_jump_rdy;

wire                 f_frontend_pwrdown_ok;

// 预译码的寄存器号，供寄存器堆访问使用
wire [W_REGADDR-1:0] f_rs1_coarse;
wire [W_REGADDR-1:0] f_rs2_coarse;
wire [W_REGADDR-1:0] f_rs1_fine;
wire [W_REGADDR-1:0] f_rs2_fine;

wire [31:0]          fd_cir;
wire [31:0]          fd_cir_raw;
wire [1:0]           fd_cir_err;
wire                 fd_cir_break_any;
wire                 fd_cir_break_d_mode;
wire [1:0]           fd_cir_predbranch;
wire [1:0]           fd_cir_vld;
wire                 fd_cir_is_32bit;
wire                 fd_cir_invalid_16bit;
wire                 fd_cir_is_uop;
wire                 fd_cir_uop_nonfinal;
wire                 fd_cir_uop_no_pc_update;
wire                 fd_cir_uop_atomic;

wire [1:0]           df_cir_use;
wire                 df_cir_flush_behind;

wire                 df_uop_stall;
wire                 df_uop_clear;
wire                 df_lspair_phase_next;

wire                 x_btb_set;
wire [W_ADDR-1:0]    x_btb_set_src_addr;
wire                 x_btb_set_src_size;
wire [W_ADDR-1:0]    x_btb_set_target_addr;
wire                 x_btb_clear;
wire                 x_btb_flush;

wire [W_ADDR-1:0]    d_btb_target_addr;

wire [W_ADDR-1:0]    f_pmp_i_addr;
wire                 f_pmp_i_m_mode;
wire                 f_pmp_i_kill;

wire [W_ADDR-1:0]    f_trigger_addr;
wire                 f_trigger_m_mode;
wire [1:0]           f_trigger_break_any;
wire [1:0]           f_trigger_break_d_mode;

assign bus_aph_panic_i = 1'b0;

wire f_mem_size;
assign bus_hsize_i = f_mem_size ? HSIZE_WORD : HSIZE_HWORD;

ominicore_frontend #(
`include "ominicore_config_inst.vh"
) frontend (
	.clk                  (clk),
	.rst_n                (rst_n),

	.mem_size             (f_mem_size),
	.mem_addr             (bus_haddr_i),
	.mem_priv             (bus_priv_i),
	.mem_addr_vld         (bus_aph_req_i),
	.mem_addr_rdy         (bus_aph_ready_i),

	.mem_data             (bus_rdata_i),
	.mem_data_err         (bus_dph_err_i),
	.mem_data_vld         (bus_dph_ready_i),

	.jump_target          (f_jump_target),
	.jump_priv            (f_jump_priv),
	.jump_target_vld      (f_jump_req),
	.jump_target_rdy      (f_jump_rdy),

	.btb_set              (x_btb_set),
	.btb_set_src_addr     (x_btb_set_src_addr),
	.btb_set_src_size     (x_btb_set_src_size),
	.btb_set_target_addr  (x_btb_set_target_addr),
	.btb_clear            (x_btb_clear),
		.btb_flush           (x_btb_flush),
	.btb_target_addr_out  (d_btb_target_addr),
	.zg_evt_br_alloc      (zg_evt_br_alloc_w),
	.zg_evt_br_flush      (zg_evt_br_flush_w),
	.zg_evt_br_ctrdec     (zg_evt_br_ctrdec_w),

	.cir                  (fd_cir),
	.cir_raw              (fd_cir_raw),
	.cir_err              (fd_cir_err),
	.cir_predbranch       (fd_cir_predbranch),
	.cir_break_any        (fd_cir_break_any),
	.cir_break_d_mode     (fd_cir_break_d_mode),
	.cir_vld              (fd_cir_vld),
	.cir_is_32bit         (fd_cir_is_32bit),
	.cir_invalid_16bit    (fd_cir_invalid_16bit),
	.cir_is_uop           (fd_cir_is_uop),
	.cir_uop_nonfinal     (fd_cir_uop_nonfinal),
	.cir_uop_no_pc_update (fd_cir_uop_no_pc_update),
	.cir_uop_atomic       (fd_cir_uop_atomic),

	.cir_use              (df_cir_use),
	.cir_flush_behind     (df_cir_flush_behind),
	.uop_stall            (df_uop_stall),
	.uop_clear            (df_uop_clear),
	.df_lspair_phase_next (df_lspair_phase_next),

	.pwrdown_ok           (f_frontend_pwrdown_ok),
	.delay_first_fetch    (!pwrup_ack),

	.predecode_rs1_coarse (f_rs1_coarse),
	.predecode_rs2_coarse (f_rs2_coarse),
	.predecode_rs1_fine   (f_rs1_fine),
	.predecode_rs2_fine   (f_rs2_fine),

	.debug_mode           (debug_mode),
	.dbg_instr_data       (dbg_instr_data),
	.dbg_instr_data_vld   (dbg_instr_data_vld),
	.dbg_instr_data_rdy   (dbg_instr_data_rdy),

	.pmp_i_addr           (f_pmp_i_addr),
	.pmp_i_m_mode         (f_pmp_i_m_mode),
	.pmp_i_kill           (f_pmp_i_kill),

	.trigger_addr         (f_trigger_addr),
	.trigger_m_mode       (f_trigger_m_mode),
	.trigger_break_any    (f_trigger_break_any),
	.trigger_break_d_mode (f_trigger_break_d_mode)
);

// ----------------------------------------------------------------------------
// 流水线 X 级（译码逻辑）

// 对前端声称有效的指令片段做 X 检查
`ifdef OMNICORE_X_CHECKS
always @ (posedge clk) begin
	if (rst_n) begin
		if (|fd_cir_vld && (^fd_cir[15:0] === 1'bx)) begin
			$display("CIR LSBs are X, should be valid!");
			$finish;
		end
		if (fd_cir_vld[1] && (^fd_cir === 1'bX)) begin
			$display("CIR contains X, should be fully valid!");
			$finish;
		end
	end
end
`endif

// 送往 X 级
wire                 d_starved;
wire [W_DATA-1:0]    d_imm;
wire [W_REGADDR-1:0] d_rs1;
wire [W_REGADDR-1:0] d_rs2;
wire [W_REGADDR-1:0] d_rd;
wire [2:0]           d_funct3_32b;
wire [6:0]           d_funct7_32b;
wire [W_ALUSRC-1:0]  d_alusrc_a;
wire [W_ALUSRC-1:0]  d_alusrc_b;
wire [W_ALUOP-1:0]   d_aluop;
wire [W_MEMOP-1:0]   d_memop;
wire [W_MULOP-1:0]   d_mulop;
wire [W_BCOND-1:0]   d_branchcond;
wire [W_ADDR-1:0]    d_addr_offs;
wire                 d_addr_is_regoffs;
wire [W_ADDR-1:0]    d_pc;
wire [W_EXCEPT-1:0]  d_except;
wire                 d_sleep_wfi;
wire                 d_sleep_block;
wire                 d_sleep_unblock;
wire                 d_no_pc_increment;
wire                 d_uninterruptible;
wire                 d_fence_i;
wire                 d_fence_d;
wire                 d_csr_ren;
wire                 d_csr_wen;
wire [1:0]           d_csr_wtype;
wire                 d_csr_w_imm;
wire [W_ADDR-1:0]    d_lspair_offset;

wire                 x_jump_not_except;
wire                 x_mmode_execution;
wire                 x_trap_wfi;

wire [W_ADDR-1:0]    debug_dpc_wdata;
wire                 debug_dpc_wen;
wire [W_ADDR-1:0]    debug_dpc_rdata;

ominicore_decode #(
`include "ominicore_config_inst.vh"
) decode_u (
	.clk                     (clk),
	.rst_n                   (rst_n),

	.fd_cir                  (fd_cir),
	.fd_cir_err              (fd_cir_err),
	.fd_cir_predbranch       (fd_cir_predbranch),
	.fd_cir_vld              (fd_cir_vld),
	.fd_cir_is_32bit         (fd_cir_is_32bit),
	.fd_cir_invalid_16bit    (fd_cir_invalid_16bit),
	.fd_cir_is_uop           (fd_cir_is_uop),
	.fd_cir_uop_nonfinal     (fd_cir_uop_nonfinal),
	.fd_cir_uop_no_pc_update (fd_cir_uop_no_pc_update),
	.fd_cir_uop_atomic       (fd_cir_uop_atomic),

	.df_cir_use              (df_cir_use),
	.df_cir_flush_behind     (df_cir_flush_behind),
	.df_uop_stall            (df_uop_stall),
	.df_uop_clear            (df_uop_clear),
	.df_lspair_phase_next    (df_lspair_phase_next),

	.d_pc                    (d_pc),
	.x_jump_not_except       (x_jump_not_except),

	.debug_mode              (debug_mode),
	.m_mode                  (x_mmode_execution),
	.trap_wfi                (x_trap_wfi),

	.debug_dpc_wdata         (debug_dpc_wdata),
	.debug_dpc_wen           (debug_dpc_wen),
	.debug_dpc_rdata         (debug_dpc_rdata),

	.d_starved               (d_starved),
	.x_stall                 (x_stall),
	.f_jump_now              (f_jump_now),
	.f_jump_target           (f_jump_target),
	.d_btb_target_addr       (d_btb_target_addr),

	.d_imm                   (d_imm),
	.d_rs1                   (d_rs1),
	.d_rs2                   (d_rs2),
	.d_rd                    (d_rd),
	.d_funct3_32b            (d_funct3_32b),
	.d_funct7_32b            (d_funct7_32b),
	.d_alusrc_a              (d_alusrc_a),
	.d_alusrc_b              (d_alusrc_b),
	.d_aluop                 (d_aluop),
	.d_memop                 (d_memop),
	.d_mulop                 (d_mulop),
	.d_csr_ren               (d_csr_ren),
	.d_csr_wen               (d_csr_wen),
	.d_csr_wtype             (d_csr_wtype),
	.d_csr_w_imm             (d_csr_w_imm),
	.d_branchcond            (d_branchcond),
	.d_addr_offs             (d_addr_offs),
	.d_addr_is_regoffs       (d_addr_is_regoffs),
	.d_except                (d_except),
	.d_sleep_wfi             (d_sleep_wfi),
	.d_sleep_block           (d_sleep_block),
	.d_sleep_unblock         (d_sleep_unblock),
	.d_no_pc_increment       (d_no_pc_increment),
	.d_uninterruptible       (d_uninterruptible),
	.d_lspair_offset         (d_lspair_offset),
	.d_fence_i               (d_fence_i),
	.d_fence_d               (d_fence_d)
);

// ----------------------------------------------------------------------------
// 流水线 X 级（执行逻辑）

// 寄存上一拍发生在寄存器堆上的写，并做旁路。
// 这是寄存器堆内部"写->读"旁路的一种替代方案，
// 后者若要保持 BRAM 推断兼容性（iCE40）就无法实现。
reg  [W_REGADDR-1:0] mw_rd;
reg  [W_DATA-1:0]    mw_result;

// 来自寄存器堆:
wire [W_DATA-1:0]    x_rdata1;
wire [W_DATA-1:0]    x_rdata2;

// 用于多选的组合寄存器
reg   [W_DATA-1:0]   x_rs1_bypass;
reg   [W_DATA-1:0]   x_rs2_bypass;
reg   [W_DATA-1:0]   x_op_a;
reg   [W_DATA-1:0]   x_op_b;
wire  [W_DATA-1:0]   x_alu_result;
wire                 x_alu_cmp;

wire [W_DATA-1:0]    m_trap_addr;
wire                 m_trap_is_irq;
wire                 m_trap_enter_vld;
wire                 m_trap_enter_soon;
wire                 m_trap_enter_rdy = f_jump_rdy;

wire                 x_mmode_loadstore;
wire                 m_mmode_trap_entry;

reg  [W_REGADDR-1:0] xm_rs1;
reg  [W_REGADDR-1:0] xm_rs2;
reg  [W_REGADDR-1:0] xm_rd;
reg  [W_DATA-1:0]    xm_result;
reg  [1:0]           xm_addr_align;
reg  [W_MEMOP-1:0]   xm_memop;
reg  [W_EXCEPT-1:0]  xm_except;
reg                  xm_except_to_d_mode;
reg                  xm_no_pc_increment;
reg                  xm_sleep_wfi;
reg                  xm_sleep_block;
reg                  xm_delay_irq_entry_on_ls_stagex;

// ----------------------------------------------------------------------------
// 停顿逻辑

// IRQ 会插在 X 级与 M 级指令之间，因此这种情况下
// X 停顿而 M 可以继续。-> M 陷入陷阱时 X 必然停顿，M *可能*停顿。
wire x_stall_on_trap = m_trap_enter_vld && !m_trap_enter_rdy ||
	m_trap_enter_soon && !m_trap_enter_vld;

// 插入停顿以避免总线上 exclusive 访问被非法流水化
// （同时也为直接 lr.w -> sc.w 指令序列
// 更新本地监视器争取时间）。注意 M 级不检查 AMO，
// 因为 AMO 自身完成时会完全栅栏隔离后才下流。

wire d_memop_is_amo = |EXTENSION_A && d_memop == MEMOP_AMO;

wire x_stall_on_exclusive_overlap = |EXTENSION_A && (
	(d_memop_is_amo || d_memop == MEMOP_SC_W || d_memop == MEMOP_LR_W) &&
	(xm_memop == MEMOP_SC_W || xm_memop	== MEMOP_LR_W)
);

// AMO 完全从 X 级发出。我们让 X 保持停顿，
// 并向 M 级送入气泡。否则异常处理会更加混乱。相位
// 0-3 是读/写的地址/数据相。相位 4 是错误相，由 HRESP
// 或读操作的 HEXOKAY 低电平响应引起。

// 若 AMO 跟在一条会产生异常的指令之后，也需清除 AMO。注意
// 当 hresp 为高时，即使 hready 为高我们在相位 3 仍要停顿，
// 因为接下来要进入相位 4 处理错误响应。

reg [2:0] x_amo_phase;
wire x_stall_on_amo = |EXTENSION_A && d_memop_is_amo && !m_trap_enter_soon && (
	x_amo_phase < 3'h3 || (x_amo_phase == 3'h3 && (!bus_dph_ready_d || !bus_dph_exokay_d || bus_dph_err_d))
);

// 写后读（RAW）冒险检测（例如 load-use）

wire m_fast_mul_result_vld;
wire m_generating_result =
	(xm_memop < MEMOP_SW) ||
	(|EXTENSION_A && xm_memop == MEMOP_LR_W) ||
	(|EXTENSION_A && xm_memop == MEMOP_SC_W) || // sc.w 的成功结果是数据相
	(|EXTENSION_M && m_fast_mul_result_vld);

reg x_stall_on_raw;

always @ (*) begin
	x_stall_on_raw = 1'b0;
	if (REDUCED_BYPASS) begin
		x_stall_on_raw =
			|xm_rd && (xm_rd == d_rs1 || xm_rd == d_rs2) ||
			|mw_rd && (mw_rd == d_rs1 || mw_rd == d_rs2);
	end else if (m_generating_result) begin
		// 有了完整旁路网络后，load-use（或快速乘法-use）是唯一的 RAW 停顿
		if (|xm_rd && xm_rd == d_rs1) begin
			// store 地址无法在后面再旁路，因此这里没有例外。
			x_stall_on_raw = 1'b1;
		end else if (|xm_rd && xm_rd == d_rs2) begin
			// store 数据可以在 M 级旁路。其他指令必须停顿。
			x_stall_on_raw = !(d_memop == MEMOP_SW || d_memop == MEMOP_SH || d_memop == MEMOP_SB);
		end
	end
end

wire x_stall_on_fence;
wire x_stall_muldiv;
wire x_jump_req;

assign x_stall =
	m_stall ||
	x_stall_on_trap ||
	x_stall_on_exclusive_overlap ||
	x_stall_on_amo ||
	x_stall_on_raw ||
	x_stall_on_fence ||
	x_stall_muldiv ||
	bus_aph_req_d && !bus_aph_ready_d ||
	x_jump_req && !f_jump_rdy;

wire m_sleep_stall_release;

wire x_loadstore_pmp_fail;
wire x_trig_step_break;
wire x_trig_break = fd_cir_break_any || x_trig_step_break;
wire x_trig_break_d_mode = fd_cir_break_d_mode;

// ----------------------------------------------------------------------------
// 执行逻辑

// ALU、操作数多选器与旁路

// 近似寄存器号已在第 1 级预译码，供寄存器堆读。
// （"近似"指当指令在该端口上*没有*寄存器操作数时
// 它们是无效的。）这些寄存器号不能用于
// 冒险检查，但用于旁路没问题，并使旁路多选器
// 不依赖第 2 级译码。

reg [W_REGADDR-1:0] d_rs1_predecoded;
reg [W_REGADDR-1:0] d_rs2_predecoded;

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		d_rs1_predecoded <= {W_REGADDR{1'b0}};
		d_rs2_predecoded <= {W_REGADDR{1'b0}};
	end else if (d_starved || !x_stall) begin
		d_rs1_predecoded <= f_rs1_fine & REGADDR_MASK;
		d_rs2_predecoded <= f_rs2_fine & REGADDR_MASK;
	end
end

wire [W_REGADDR-1:0] d_rs1_predecoded_nxt = REGADDR_MASK & (
	d_starved || !x_stall ? f_rs1_fine : d_rs1_predecoded
);
wire [W_REGADDR-1:0] d_rs2_predecoded_nxt = REGADDR_MASK & (
	d_starved || !x_stall ? f_rs2_fine : d_rs2_predecoded
);

wire [W_REGADDR-1:0] xm_rd_nxt;
wire [W_REGADDR-1:0] mw_rd_nxt;

reg [2:0] x_rs1_select_regs_xm_mw;
reg [2:0] x_rs2_select_regs_xm_mw;

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		x_rs1_select_regs_xm_mw <= 3'b000;
		x_rs2_select_regs_xm_mw <= 3'b000;
	end else begin
		x_rs1_select_regs_xm_mw <=
			5'h00     == d_rs1_predecoded_nxt ? 3'b000 :
			xm_rd_nxt == d_rs1_predecoded_nxt ? 3'b010 :
			mw_rd_nxt == d_rs1_predecoded_nxt ? 3'b001 : 3'b100;
		x_rs2_select_regs_xm_mw <=
			5'h00     == d_rs2_predecoded_nxt ? 3'b000 :
			xm_rd_nxt == d_rs2_predecoded_nxt ? 3'b010 :
			mw_rd_nxt == d_rs2_predecoded_nxt ? 3'b001 : 3'b100;
	end
end

`ifdef OMNICORE_ASSERTIONS
always @ (posedge clk) if (rst_n && !x_stall) begin
	// 若第 2 级看到寄存器操作数，那它必定也已被正确预译码。
	if (|d_rs1)
		assert(d_rs1_predecoded == d_rs1);
	if (|d_rs2)
		assert(d_rs2_predecoded == d_rs2);
	// 若没有预译码出寄存器，第 2 级译码必须一致认为没有寄存器操作数。
	if (~|d_rs1_predecoded)
		assert(~|d_rs1);
	if (~|d_rs2_predecoded)
		assert(~|d_rs2);
	// 确保旁路掩码的更新能跟踪更早的在途指令
	if      (|d_rs1 && d_rs1 == xm_rd) assert(x_rs1_select_regs_xm_mw[1]);
	else if (|d_rs1 && d_rs1 == mw_rd) assert(x_rs1_select_regs_xm_mw[0]);
	else if (|d_rs1                  ) assert(x_rs1_select_regs_xm_mw[2]);
	else if (~|d_rs1_predecoded      ) assert(~|x_rs1_select_regs_xm_mw);
	if      (|d_rs2 && d_rs2 == xm_rd) assert(x_rs2_select_regs_xm_mw[1]);
	else if (|d_rs2 && d_rs2 == mw_rd) assert(x_rs2_select_regs_xm_mw[0]);
	else if (|d_rs2                  ) assert(x_rs2_select_regs_xm_mw[2]);
	else if (~|d_rs2_predecoded      ) assert(~|x_rs2_select_regs_xm_mw);
end
`endif


always @ (*) begin

	x_rs1_bypass =
		(x_rdata1  & {W_DATA{x_rs1_select_regs_xm_mw[2]}}) |
		(xm_result & {W_DATA{x_rs1_select_regs_xm_mw[1]}}) |
		(mw_result & {W_DATA{x_rs1_select_regs_xm_mw[0]}});

	x_rs2_bypass =
		(x_rdata2  & {W_DATA{x_rs2_select_regs_xm_mw[2]}}) |
		(xm_result & {W_DATA{x_rs2_select_regs_xm_mw[1]}}) |
		(mw_result & {W_DATA{x_rs2_select_regs_xm_mw[0]}});

	// AMO 在读数据相结束时把 rdata 捕获进 mw_result，
	// 以便回馈送入 ALU。
	if (|EXTENSION_A && x_amo_phase == 3'h2)
		x_op_a = mw_result;
	else if (d_alusrc_a)
		x_op_a = d_pc;
	else
		x_op_a = x_rs1_bypass;

	if (d_alusrc_b)
		x_op_b = d_imm;
	else
		x_op_b = x_rs2_bypass;
end

ominicore_alu #(
`include "ominicore_config_inst.vh"
) alu (
	.aluop      (d_aluop),
	.funct3_32b (d_funct3_32b),
	.funct7_32b (d_funct7_32b),
	.op_a       (x_op_a),
	.op_b       (x_op_b),
	.result     (x_alu_result),
	.cmp        (x_alu_cmp)
);

// load/store 总线请求

wire x_unaligned_addr = d_memop != MEMOP_NONE && (
	bus_hsize_d == HSIZE_WORD && |bus_haddr_d[1:0] ||
	bus_hsize_d == HSIZE_HWORD && bus_haddr_d[0]
);

reg mw_local_exclusive_reserved;

wire x_memop_vld = d_memop != MEMOP_NONE && !(
	|EXTENSION_A && d_memop == MEMOP_SC_W && !mw_local_exclusive_reserved ||
	|EXTENSION_A && d_memop_is_amo && x_amo_phase != 3'h0 && x_amo_phase != 3'h2
);

wire x_memop_write =
	d_memop == MEMOP_SW ||
	d_memop == MEMOP_SH ||
	d_memop == MEMOP_SB ||
	(|EXTENSION_A && d_memop == MEMOP_SC_W) ||
	(|EXTENSION_A && d_memop_is_amo && x_amo_phase == 3'h2);

// 始终查询全局监视器，除非 store-conditional 被本地监视器抑制。
assign bus_aph_excl_d = |EXTENSION_A && (
	d_memop == MEMOP_LR_W ||
	d_memop == MEMOP_SC_W ||
	d_memop_is_amo
);

// AMO 使流水线停顿，然后每 4 拍迭代产生两次总线传输，
// 除非它因总线错误或 load 保留失败
// 而中途放弃。
always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		x_amo_phase <= 3'h0;
	end else if (|EXTENSION_A && d_memop_is_amo && (
		bus_aph_ready_d || bus_dph_ready_d ||
		m_trap_enter_vld || x_unaligned_addr || x_loadstore_pmp_fail ||
		x_amo_phase == 3'h4
	)) begin
		if (m_trap_enter_vld) begin
			// 中途放弃，取消进行中的 AMO。
			x_amo_phase <= 3'h0;
		end else if (x_stall_on_raw || x_stall_on_exclusive_overlap || m_trap_enter_soon) begin
			// 第一个地址相因依赖前一条 load/mul 等的结果而停顿。
			// 后续相位不应出现这种情况。
			x_amo_phase <= 3'h0;
		end else if (x_amo_phase == 3'h4) begin
			// 错误相进入第 3 级并产生异常后即清除
			if (!x_stall)
				x_amo_phase <= 3'h0;
		end else if (x_unaligned_addr || x_loadstore_pmp_fail) begin
			x_amo_phase <= 3'h4;
		end else if (x_amo_phase == 3'h1 && !bus_dph_exokay_d) begin
			// load 保留失败表明该内存区域不支持
			// exclusive 访问，store 永远不会成功。异常。
			x_amo_phase <= 3'h4;
		end else if ((x_amo_phase == 3'h1 || x_amo_phase == 3'h3) && bus_dph_err_d) begin
			// 总线错误。异常。
			x_amo_phase <= 3'h4;
		end else if (x_amo_phase == 3'h3) begin
			// 要么已完成，要么写失败。无论哪种，回到起点。
			x_amo_phase <= 3'h0;
		end else begin
			// 默认推进: 读地址 -> 读数据 -> 写地址 -> 写数据
			x_amo_phase <= x_amo_phase + 3'h1;
		end
	end
end

`ifdef OMNICORE_ASSERTIONS
// 上述块的断言，单独提取是为了避免断言
// 对复位敏感（yosys-smtbmc 的限制）
always @ (posedge clk) if (rst_n) begin
	if (|EXTENSION_A && d_memop_is_amo && (
		bus_aph_ready_d || bus_dph_ready_d ||
		m_trap_enter_vld || x_unaligned_addr ||
		x_amo_phase == 3'h4
	)) begin
		if (m_trap_enter_vld) begin
			// 应只发生在地址相*或*错误相。
			assert(x_amo_phase == 3'h0 || x_amo_phase == 3'h2 || x_amo_phase == 3'h4);
			// 错误相仅在如下情况保持: 一条非对齐 AMO 紧随
			// 一条随后产生异常的普通内存访问，且该 AMO
			// 因非对齐直接进入错误相。
			if (x_amo_phase == 3'h4)
				assert(x_unaligned_addr);
		end else if (x_stall_on_raw || x_stall_on_exclusive_overlap || m_trap_enter_soon) begin
			assert(x_amo_phase == 3'h0);
		end else if (x_amo_phase == 3'h4) begin
			// 这只应发生在我们因更早的 load/store 等而停顿时
			assert(!(x_stall && !m_stall));
		end
	end
end
`endif

`ifdef OMNICORE_ASSERTIONS
// AMO 通用设计断言
reg prop_dph_is_excl = 1'b0;
always @ (posedge clk) if (rst_n) begin
	if (bus_aph_ready_d && bus_aph_req_d) begin
		prop_dph_is_excl <= bus_aph_excl_d;
	end else if (bus_dph_ready_d) begin
		// 数据相有可能先于地址相结束，
		// 例如 SBA 仲裁所致
		prop_dph_is_excl <= 1'b0;
	end
	// 其他状态应不可达
	assert(x_amo_phase <= 3'h4);
	// 首状态应为 0 —— 不希望上一条 AMO 的状态带到下一条。
	if (x_stall_on_amo && !$past(x_stall_on_amo))
		assert(x_amo_phase == 3'h0);
	// AMO 之间应处于静息状态
	if (!d_memop_is_amo)
		assert(x_amo_phase == 3'h0);
	// 错误相不应有第 2 级阻塞，以便进入第 3 级
	// 触发异常进入。允许被更年轻的指令阻塞，但是..
	if (x_amo_phase == 3'h4)
		assert(!x_stall || m_stall);
	// ..在第 3 级未清空时能到达 AMO 错误相的唯一途径，
	// 应当是非对齐的 AMO 地址 —— 它直接进入错误相。
	if (x_amo_phase == 3'h4 && m_stall)
		assert(x_unaligned_addr);
	// 写地址相不应出现非对齐地址，
	// 因为在读地址相就会被发现
	if (x_amo_phase == 3'h2)
		assert(!x_unaligned_addr);
	// 错误相要么源于总线响应，要么源于非对齐地址。
	// 两者都不是写地址相。
	if (x_amo_phase == 3'h4)
		assert($past(x_amo_phase) != 3'h2);
	// 确保相位 2 传递 store 数据时 M 级未停顿
	if (x_amo_phase == 3'h2)
		assert(!m_stall);
	// 确保不会把 AMO 地址相与其他 exclusive 数据相流水化
	if (d_memop_is_amo && bus_aph_req_d)
		assert(!prop_dph_is_excl);
	// 只有相位 0（读地址相）和相位 2（写地址相）发出新的总线传输
	if (d_memop_is_amo && bus_aph_req_d)
		assert(x_amo_phase == 3'h0 || x_amo_phase == 3'h2);
end
`endif

// 该加法器同时用于分支目标和 load/store 地址。
// 支持全部分支类型已经需要 rs1 + I 型与 pc + B 型，
// B 型与 S 型几乎相同，因此 rs1 + S 型几乎免费。

wire [W_ADDR-1:0] x_addr_sum =
	(d_lspair_offset & {W_ADDR{|EXTENSION_ZILSD}}) +
	(d_addr_is_regoffs ? x_rs1_bypass : d_pc) +
	d_addr_offs;

always @ (*) begin
	// 注意切勿用任何来自 hready 的信号来门控 htrans！
	bus_haddr_d = x_addr_sum;
	bus_hwrite_d = x_memop_write;
	bus_priv_d = x_mmode_loadstore;
	case (d_memop)
		MEMOP_LW:  bus_hsize_d = HSIZE_WORD;
		MEMOP_SW:  bus_hsize_d = HSIZE_WORD;
		MEMOP_LH:  bus_hsize_d = HSIZE_HWORD;
		MEMOP_LHU: bus_hsize_d = HSIZE_HWORD;
		MEMOP_SH:  bus_hsize_d = HSIZE_HWORD;
		MEMOP_LB:  bus_hsize_d = HSIZE_BYTE;
		MEMOP_LBU: bus_hsize_d = HSIZE_BYTE;
		MEMOP_SB:  bus_hsize_d = HSIZE_BYTE;
		default:   bus_hsize_d = HSIZE_WORD;
	endcase
	bus_aph_req_d = x_memop_vld && !(
		x_stall_on_raw ||
		x_stall_on_exclusive_overlap ||
		x_loadstore_pmp_fail ||
		x_trig_break ||
		x_unaligned_addr ||
		m_trap_enter_soon ||
		((xm_sleep_wfi || xm_sleep_block) && !m_sleep_stall_release)
	);
end

// 栅栏指令要等到相关总线安静后才发出。
wire m_dphase_in_flight;
assign x_stall_on_fence =
	(d_fence_i && !(fence_rdy && !m_dphase_in_flight && f_frontend_pwrdown_ok)) ||
	(d_fence_d && !(fence_rdy && !m_dphase_in_flight));

wire x_fence_mask =
	x_trig_break ||
	m_dphase_in_flight ||
	m_trap_enter_soon ||
	((xm_sleep_wfi || xm_sleep_block) && !m_sleep_stall_release);

assign fence_i_vld = d_fence_i && !x_fence_mask && f_frontend_pwrdown_ok;
assign fence_d_vld = d_fence_d && !x_fence_mask;

// 乘法/除法

wire [W_DATA-1:0] x_muldiv_result;
wire [W_DATA-1:0] m_fast_mul_result;

wire x_use_fast_mul = d_aluop == ALUOP_MULDIV && (
	(MUL_FAST  && d_mulop == M_OP_MUL   ) ||
	(MULH_FAST && d_mulop == M_OP_MULH  ) ||
	(MULH_FAST && d_mulop == M_OP_MULHU ) ||
	(MULH_FAST && d_mulop == M_OP_MULHSU)
);

generate
if (EXTENSION_M) begin: has_muldiv
	wire              x_muldiv_op_vld;
	wire              x_muldiv_op_rdy;
	wire              x_muldiv_result_vld;
	wire [W_DATA-1:0] x_muldiv_result_h;
	wire [W_DATA-1:0] x_muldiv_result_l;

	reg x_muldiv_posted;
	always @ (posedge clk or negedge rst_n)
		if (!rst_n)
			x_muldiv_posted <= 1'b0;
		else
			x_muldiv_posted <= (x_muldiv_posted || (x_muldiv_op_vld && x_muldiv_op_rdy)) && x_stall;

	wire x_muldiv_kill = m_trap_enter_soon;

	assign x_muldiv_op_vld = (d_aluop == ALUOP_MULDIV && !x_use_fast_mul)
		&& !(x_muldiv_posted || x_stall_on_raw || x_muldiv_kill);

	ominicore_muldiv_seq #(
	`include "ominicore_config_inst.vh"
	) muldiv (
		.clk        (clk),
		.rst_n      (rst_n),
		.op         (d_mulop),
		.op_vld     (x_muldiv_op_vld),
		.op_rdy     (x_muldiv_op_rdy),
		.op_kill    (x_muldiv_kill),
		.op_a       (x_rs1_bypass),
		.op_b       (x_rs2_bypass),

		.result_h   (x_muldiv_result_h),
		.result_l   (x_muldiv_result_l),
		.result_vld (x_muldiv_result_vld)
	);

	wire x_muldiv_result_is_high =
		d_mulop == M_OP_MULH ||
		d_mulop == M_OP_MULHSU ||
		d_mulop == M_OP_MULHU ||
		d_mulop == M_OP_REM ||
		d_mulop == M_OP_REMU;
	assign x_muldiv_result = x_muldiv_result_is_high ? x_muldiv_result_h : x_muldiv_result_l;
	assign x_stall_muldiv = x_muldiv_op_vld || !x_muldiv_result_vld;

	if (MUL_FAST) begin: has_fast_mul

		// 若置起 MUL_FASTER，乘法器组合地产生结果，
		// 因此从不需要把乘法结果递交到第 3 级。
		wire x_issue_fast_mul = x_use_fast_mul && |d_rd && !x_stall && !MUL_FASTER;

		ominicore_mul_fast #(
		`include "ominicore_config_inst.vh"
		) mul_fast (
			.clk        (clk),
			.rst_n      (rst_n),

			.op_vld     (x_issue_fast_mul),
			.op         (d_mulop),
			.op_a       (x_rs1_bypass),
			.op_b       (x_rs2_bypass),

			.result     (m_fast_mul_result),
			.result_vld (m_fast_mul_result_vld)
		);

	end else begin: no_fast_mul

		assign m_fast_mul_result = {W_DATA{1'b0}};
		assign m_fast_mul_result_vld = 1'b0;

	end

`ifdef OMNICORE_ASSERTIONS
	always @ (posedge clk) if (d_aluop != ALUOP_MULDIV) assert(!x_stall_muldiv);
`endif

end else begin: no_muldiv

	assign x_muldiv_result = {W_DATA{1'b0}};
	assign m_fast_mul_result = {W_DATA{1'b0}};
	assign m_fast_mul_result_vld = 1'b0;
	assign x_stall_muldiv = 1'b0;

end
endgenerate

// 分支处理

// 对 JALR，结果的最低位必须由硬件清除
wire [W_ADDR-1:0] x_jump_target = x_addr_sum & ~32'd1;
wire              x_jump_misaligned = ~|EXTENSION_C && x_addr_sum[1];
wire              x_branch_cmp_noinvert;

wire              x_branch_was_predicted = |BRANCH_PREDICTOR && fd_cir_predbranch[0];
wire              x_branch_cmp = x_branch_cmp_noinvert ^ x_branch_was_predicted;

generate
if (~|FAST_BRANCHCMP) begin: alu_branchcmp

	assign x_branch_cmp_noinvert = x_alu_cmp;

end else begin: fast_branchcmp

	ominicore_branchcmp #(
	`include "ominicore_config_inst.vh"
	) branchcmp_u (
		.cir   (fd_cir),
		.op_a  (x_rs1_bypass),
		.op_b  (x_rs2_bypass),
		.cmp   (x_branch_cmp_noinvert)
	);

end
endgenerate

// 注意不要执行比较值依赖 load 结果的分支
wire x_jump_req_unchecked = !x_stall_on_raw && (
	d_branchcond == BCOND_ALWAYS ||
	d_branchcond == BCOND_ZERO && !x_branch_cmp ||
	d_branchcond == BCOND_NZERO && x_branch_cmp
);

assign x_jump_req = x_jump_req_unchecked && !x_jump_misaligned && !x_trig_break;

// ZIGUANG (BP-OPT 评估结论, B2-DESIGN §十三): 保持向后过滤。放宽分配
// + 方向感知实测 mbpmiss -24% 但 mcycle +0.28% (前向分支学习税与恢复
// 跳冲刷), 按评分标准 (CoreMark/MHz, CoreMark/LUT) 保留 mcycle 最优;
// 实验矩阵与方案留档, 未来分支预测轮次可直接复用。
assign x_btb_set = |BRANCH_PREDICTOR && (
	x_jump_req_unchecked && d_addr_offs[W_ADDR - 1] && !x_branch_was_predicted &&
	(d_branchcond == BCOND_NZERO || d_branchcond == BCOND_ZERO)
);

// ZIGUANG (BP-06/BP-03): flush (fence.i/陷阱, 整体清空) 与误预测衰减
// (方向翻转+计数器递减) 分离 —— 前者不得被表项巧合匹配劫持为衰减
assign x_btb_flush = d_fence_i || (m_trap_enter_vld && m_trap_enter_rdy);
assign x_btb_clear = |BRANCH_PREDICTOR && (
	x_branch_was_predicted && x_jump_req_unchecked
);

assign x_btb_set_src_addr = d_pc;
assign x_btb_set_target_addr = x_jump_target;
assign x_btb_set_src_size = fd_cir_is_32bit;

// 内存保护

wire [11:0]       x_pmp_cfg_addr;
wire              x_pmp_cfg_wen;
wire [W_DATA-1:0] x_pmp_cfg_wdata;
wire [W_DATA-1:0] x_pmp_cfg_rdata;

generate
if (PMP_REGIONS > 0) begin: have_pmp

	wire x_loadstore_pmp_badperm;
	assign x_loadstore_pmp_fail = x_loadstore_pmp_badperm && x_memop_vld && !debug_mode;

	// R 与 W 在 load/store 地址生成处强制执行，
	// 及时屏蔽地址相请求。X 则通过在取指数据相
	// （F 级）检查取指地址来强制执行，并把 PMP
	// 失败升级为总线错误。

	ominicore_pmp #(
	`include "ominicore_config_inst.vh"
	) pmp (
		.clk                       (clk),
		.rst_n                     (rst_n),

		.cfg_addr                  (x_pmp_cfg_addr),
		.cfg_wen                   (x_pmp_cfg_wen),
		.cfg_wdata                 (x_pmp_cfg_wdata),
		.cfg_rdata                 (x_pmp_cfg_rdata),

		.i_addr                    (f_pmp_i_addr),
		.i_m_mode                  (f_pmp_i_m_mode),
		.i_kill                    (f_pmp_i_kill),

		.d_addr                    (bus_haddr_d),
		.d_addr_addend_rs1         (x_rs1_bypass),
		.d_addr_addend_imm         (d_addr_offs),
		.d_addr_addend_lspair_offs (d_lspair_offset),
		.d_m_mode                  (x_mmode_loadstore),
		.d_write                   (bus_hwrite_d),
		.d_kill                    (x_loadstore_pmp_badperm)
	);

end else begin: no_pmp

	assign x_pmp_cfg_rdata = 32'd0;
	assign x_loadstore_pmp_fail = 1'b0;
	assign f_pmp_i_kill = 1'b0;

end
endgenerate

// 触发器单元（trigger）

wire [11:0]       x_trig_cfg_addr;
wire              x_trig_cfg_wen;
wire [W_DATA-1:0] x_trig_cfg_wdata;
wire [W_DATA-1:0] x_trig_cfg_rdata;
wire              x_trig_m_en;

wire              m_trig_event_interrupt;
wire              m_trig_event_exception;
wire [3:0]        m_trig_event_trap_cause;

wire              x_instr_ret;

generate
if (DEBUG_SUPPORT != 0) begin: have_triggers

	// .fetch_d_mode 的连线来自错误的级: 之所以安全，
	// 是因为进入/退出调试模式都经由异常，异常会冲刷前端。

	ominicore_triggers #(
	`include "ominicore_config_inst.vh"
	) triggers (
		.clk              (clk),
		.rst_n            (rst_n),

		.cfg_addr         (x_trig_cfg_addr),
		.cfg_wen          (x_trig_cfg_wen),
		.cfg_wdata        (x_trig_cfg_wdata),
		.cfg_rdata        (x_trig_cfg_rdata),
		.trig_m_en        (x_trig_m_en),

		.fetch_addr       (f_trigger_addr),
		.fetch_m_mode     (f_trigger_m_mode),
		.fetch_d_mode     (debug_mode),

		.event_instr_ret  (x_instr_ret),

		.event_interrupt  (m_trig_event_interrupt),
		.event_exception  (m_trig_event_exception),
		.event_trap_cause (m_trig_event_trap_cause),
		.event_trap_enter (m_trap_enter_vld && m_trap_enter_rdy),

		.break_any        (f_trigger_break_any),
		.break_d_mode     (f_trigger_break_d_mode),

		.break_m_step     (x_trig_step_break),

		.x_d_mode         (debug_mode),
		.x_m_mode         (x_mmode_execution)
	);

end else begin: no_triggers

	assign x_trig_cfg_rdata = {W_DATA{1'b0}};
	assign f_trigger_break_any = 2'b00;
	assign f_trigger_break_d_mode = 2'b00;
	assign x_trig_step_break = 1'b0;

end
endgenerate

// CSR 与陷阱处理

// 直接使用操作码位，因为使能 RVE 时 d_rs1 会被屏蔽:
wire [W_DATA-1:0] x_csr_wdata = d_csr_w_imm ?
	{{W_DATA-5{1'b0}}, fd_cir[RV_RS1_LSB +: 5]} : x_rs1_bypass;

wire [W_DATA-1:0] x_csr_rdata;
wire              x_csr_illegal_access;
wire              x_csr_write_is_fetch_ordered;

// "前一条"指上一个进入 D/X 的指令，即
// 最近到达 M 级的指令（可能仍在 M，也可能已离开）。
reg prev_instr_was_32_bit;

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		xm_delay_irq_entry_on_ls_stagex <= 1'b0;
		prev_instr_was_32_bit <= 1'b0;
	end else begin
		// 若处于地址相的第二拍或更晚，必须推迟 IRQ，
		// 因为此时 load/store 已无法撤回。IRQ 会在
		// 该 load/store 移入下一级时进入: 如果流水线上还有另一条
		// load/store 紧随其后，它会在地址相开始前
		// 立即被 IRQ 进入所抑制。

		// AMO 同样要推迟，除非 AMO 正在转入地址相
		// 或正在完成。（"完成"不包括转入错误相。）

		xm_delay_irq_entry_on_ls_stagex <= bus_aph_req_d && !bus_aph_ready_d ||
			d_memop_is_amo && !(
				x_amo_phase == 3'h3 && bus_dph_ready_d && !bus_dph_err_d ||
				// 读保留失败同样会产生错误
				x_amo_phase == 3'h1 && bus_dph_ready_d && !bus_dph_err_d && bus_dph_exokay_d
			) || (
				(fence_i_vld || fence_d_vld) && !fence_rdy
			);

		if (!x_stall)
			prev_instr_was_32_bit <= df_cir_use == 2'd2;
	end
end

wire [W_ADDR-1:0] m_exception_return_addr;

wire [W_EXCEPT-1:0] x_except =
	x_trig_break                                             ? EXCEPT_EBREAK         :
	x_jump_req_unchecked && x_jump_misaligned                ? EXCEPT_INSTR_MISALIGN :
	x_csr_illegal_access                                     ? EXCEPT_INSTR_ILLEGAL  :
	|EXTENSION_A && x_unaligned_addr &&  d_memop_is_amo      ? EXCEPT_STORE_ALIGN    :
	|EXTENSION_A && x_amo_phase == 3'h4 && x_unaligned_addr  ? EXCEPT_STORE_ALIGN    :
	|EXTENSION_A && x_amo_phase == 3'h4                      ? EXCEPT_STORE_FAULT    :
	x_unaligned_addr &&  x_memop_write                       ? EXCEPT_STORE_ALIGN    :
	x_unaligned_addr && !x_memop_write                       ? EXCEPT_LOAD_ALIGN     :
	x_loadstore_pmp_fail && (d_memop_is_amo || bus_hwrite_d) ? EXCEPT_STORE_FAULT    :
	x_loadstore_pmp_fail                                     ? EXCEPT_LOAD_FAULT     :
	x_csr_write_is_fetch_ordered                             ? EXCEPT_REFETCH        : d_except;

// 若一条指令引发异常条件，则不认为它已退休。
wire x_except_counts_as_retire =
	x_except == EXCEPT_EBREAK  ||
	x_except == EXCEPT_REFETCH ||
	x_except == EXCEPT_MRET    ||
	x_except == EXCEPT_ECALL_M ||
	x_except == EXCEPT_ECALL_U;

assign x_instr_ret = |df_cir_use && (x_except == EXCEPT_NONE || x_except_counts_as_retire);

assign m_dphase_in_flight = xm_memop != MEMOP_NONE && xm_memop != MEMOP_AMO;

// 休眠退出时需要延迟 IRQ 进入，因为对深度休眠状态，
// 在电源握手完成前无法访问总线。
wire m_delay_irq_entry = xm_delay_irq_entry_on_ls_stagex ||
	((xm_sleep_wfi || xm_sleep_block) && !m_sleep_stall_release);

// 抑制 CSR 读写的副作用; 注意大多数 X 级异常源
// 已被排除，因为它们只适用于 CSR 指令。
wire m_csr_access_mask_early = m_trap_enter_soon || fd_cir_break_any;
wire m_csr_access_mask = m_csr_access_mask_early || x_stall;

// ZIGUANG (PMU-01): 退休级分支事件与数据/结构冲突停顿 (唯一计数点):
// - 条件分支 = d_branchcond 为 ZERO/NZERO (decode 默认 NEVER, ALWAYS 为跳转)
// - 误预测 = 有条件跳转在 X 级发出 (x_jump_req_unchecked)。推导: 本预测器
//   仅做 taken 预测 (BP-03), 预测分支的 jump 仅在实际 NT 恢复时发出,
//   未预测分支的 jump 仅在实际 taken 时发出 -- 两者皆为方向误预测;
//   旧公式 (x_jump_req_unchecked != x_branch_was_predicted) 把
//   "预测正确且实际 taken" (C 类, 热循环主力) 错计为误预测、漏计
//   "预测后 fall-through" (B 类), 口径完全扭曲 (实测 A+C≈旧值)。
// - mstall  = X 级停顿且当拍非总线等待 (mbuswait 另计)、非 wfi 休眠/停机
wire x_cond_branch_pm  = (d_branchcond == BCOND_ZERO) || (d_branchcond == BCOND_NZERO);
wire zg_pmu_branch_ret = x_instr_ret && x_cond_branch_pm;
wire zg_pmu_bp_miss    = x_instr_ret && x_cond_branch_pm && x_jump_req_unchecked;
wire zg_pmu_stall      = x_stall && !zg_pmu_buswait &&
                         !(xm_sleep_wfi || xm_sleep_block) && !debug_mode;


wire m_pwr_allow_clkgate;
wire m_pwr_allow_power_down;
wire m_pwr_allow_sleep_on_block;
wire m_wfi_wakeup_req;
wire m_clear_excl_on_trap_exit;

ominicore_csr #(
	.XLEN            (W_DATA),
`include "ominicore_config_inst.vh"
) csr_u (
	.clk                        (clk),
	.clk_always_on              (clk_always_on),
	.rst_n                      (rst_n),

	// 调试器信号
	.debug_mode                 (debug_mode),
	.dbg_req_halt               (dbg_req_halt),
	.dbg_req_halt_on_reset      (dbg_req_halt_on_reset),
	.dbg_req_resume             (dbg_req_resume),

	.dbg_instr_caught_exception (dbg_instr_caught_exception),
	.dbg_instr_caught_ebreak    (dbg_instr_caught_ebreak),

	.dbg_data0_rdata            (dbg_data0_rdata),
	.dbg_data0_wdata            (dbg_data0_wdata),
	.dbg_data0_wen              (dbg_data0_wen),

	.debug_dpc_wdata            (debug_dpc_wdata),
	.debug_dpc_wen              (debug_dpc_wen),
	.debug_dpc_rdata            (debug_dpc_rdata),

	// CSR 访问端口
	// *en_soon 是早期访问选通，不是总线停顿的函数。
	// 它们可以产生访问错误（进而陷阱），但并不真正执行访问。
	.addr                       (fd_cir[31:20]), // 始终是 I 型立即数
	.wdata                      (x_csr_wdata),
	.wen_raw                    (d_csr_wen),
	.wen_soon                   (d_csr_wen && !m_csr_access_mask_early),
	.wen                        (d_csr_wen && !m_csr_access_mask),
	.wtype                      (d_csr_wtype),
	.rdata                      (x_csr_rdata),
	.ren_soon                   (d_csr_ren && !m_csr_access_mask_early),
	.ren                        (d_csr_ren && !m_csr_access_mask),
	.illegal                    (x_csr_illegal_access),
	.write_is_fetch_ordered     (x_csr_write_is_fetch_ordered),

	// 陷阱信号
	.trap_addr                  (m_trap_addr),
	.trap_is_irq                (m_trap_is_irq),
	.trap_enter_soon            (m_trap_enter_soon),
	.trap_enter_vld             (m_trap_enter_vld),
	.trap_enter_rdy             (m_trap_enter_rdy),
	.loadstore_dphase_pending   (m_dphase_in_flight),
	.delay_irq_entry            (m_delay_irq_entry || d_uninterruptible),
	.mepc_in                    (m_exception_return_addr),

	.pwr_allow_clkgate          (m_pwr_allow_clkgate),
	.pwr_allow_power_down       (m_pwr_allow_power_down),
	.pwr_allow_sleep_on_block   (m_pwr_allow_sleep_on_block),
	.pwr_wfi_wakeup_req         (m_wfi_wakeup_req),

	.m_mode_execution           (x_mmode_execution),
	.m_mode_loadstore           (x_mmode_loadstore),
	.m_mode_trap_entry          (m_mmode_trap_entry),

	// IRQ 与异常请求
	.irq                        (irq),
	.irq_software               (soft_irq),
	.irq_timer                  (timer_irq),
	.except                     (xm_except),
	.except_to_d_mode           (xm_except_to_d_mode),

	.pmp_cfg_addr               (x_pmp_cfg_addr),
	.pmp_cfg_wen                (x_pmp_cfg_wen),
	.pmp_cfg_wdata              (x_pmp_cfg_wdata),
	.pmp_cfg_rdata              (x_pmp_cfg_rdata),

	.trig_cfg_addr              (x_trig_cfg_addr),
	.trig_cfg_wen               (x_trig_cfg_wen),
	.trig_cfg_wdata             (x_trig_cfg_wdata),
	.trig_cfg_rdata             (x_trig_cfg_rdata),
	.trig_m_en                  (x_trig_m_en),

	.trig_event_interrupt       (m_trig_event_interrupt),
	.trig_event_exception       (m_trig_event_exception),
	.trig_event_trap_cause      (m_trig_event_trap_cause),

	// 其他 CSR 专属信号
	.clear_excl_on_trap_exit    (m_clear_excl_on_trap_exit),
	.trap_wfi                   (x_trap_wfi),
	.instr_ret                  (x_instr_ret),

	// ZIGUANG (PMU-01~04)
	.zg_pmu_branch_ret          (zg_pmu_branch_ret),
	.zg_pmu_bp_miss             (zg_pmu_bp_miss),
	.zg_pmu_icaccess            (zg_pmu_icaccess),
	.zg_pmu_icmiss              (zg_pmu_icmiss),
	.zg_pmu_dcaccess            (zg_pmu_dcaccess),
	.zg_pmu_dcmiss              (zg_pmu_dcmiss),
	.zg_pmu_stall               (zg_pmu_stall),
	.zg_pmu_buswait             (zg_pmu_buswait),
	.mhartid_val                (mhartid_val),
	.eco_version                (eco_version)
);

// 流水线寄存器

assign xm_rd_nxt = REGADDR_MASK & (
	m_stall                                   ? xm_rd :
	x_stall || d_starved || m_trap_enter_soon ? 5'h0  : d_rd
);

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		xm_memop <= MEMOP_NONE;
		xm_except <= EXCEPT_NONE;
		xm_except_to_d_mode <= 1'b0;
		xm_sleep_wfi <= 1'b0;
		xm_sleep_block <= 1'b0;
		unblock_out <= 1'b0;
		xm_rs1 <= {W_REGADDR{1'b0}};
		xm_rs2 <= {W_REGADDR{1'b0}};
		xm_rd <= {W_REGADDR{1'b0}};
		xm_no_pc_increment <= 1'b0;
	end else begin
		unblock_out <= 1'b0;
		if (!m_stall) begin
			xm_rs1 <= REGADDR_MASK & d_rs1;
			xm_rs2 <= REGADDR_MASK & d_rs2;
			xm_rd  <= REGADDR_MASK & d_rd;
			// 非末尾微操作抑制 PC 自增，只有 Zcmp 需要:
			xm_no_pc_increment <= d_no_pc_increment && |EXTENSION_ZCMP;
			// 若某个源自 X 级的异常已取消地址相，数据相也要一并取消。
			xm_memop            <= x_except != EXCEPT_NONE ? MEMOP_NONE : d_memop;
			xm_except           <= x_except;
			xm_except_to_d_mode <= x_trig_break_d_mode;
			xm_sleep_wfi        <= x_except == EXCEPT_NONE && d_sleep_wfi;
			xm_sleep_block      <= x_except == EXCEPT_NONE && d_sleep_block;
			unblock_out         <= x_except == EXCEPT_NONE && d_sleep_unblock;
			// 注意需要 d_starved 项，因为前端饥饿时
			// 也可能产生异常（例如 PMP X 权限失败），
			// 若放任该跳转抢先，会得到错误的 mepc:
			if (x_stall || d_starved || m_trap_enter_soon) begin
				// 插入气泡
				xm_rd               <= {W_REGADDR{1'b0}};
				xm_memop            <= MEMOP_NONE;
				xm_except           <= EXCEPT_NONE;
				xm_except_to_d_mode <= 1'b0;
				xm_sleep_wfi        <= 1'b0;
				xm_sleep_block      <= 1'b0;
				unblock_out         <= 1'b0;
			end
		end else if (bus_dph_err_d) begin
			// 两拍式 AHB5 错误响应的第一拍。本拍传递异常，
			// 下一拍陷阱进入即置起，
			// 从而抑制当前可能处于 X 级的任何 load/store。
			xm_except <=
				|EXTENSION_A && xm_memop == MEMOP_LR_W ? EXCEPT_LOAD_FAULT :
				                xm_memop <= MEMOP_LBU  ? EXCEPT_LOAD_FAULT : EXCEPT_STORE_FAULT;
		end
	end
end

`ifdef OMNICORE_ASSERTIONS
// 上一块的属性
always @ (posedge clk) if (rst_n) begin
	if (!m_stall) begin
		// 假定下游总线符合规范，err && !stalled 是
		// 错误响应的最后一拍，此时它必定
		// 已在此前被捕获为异常。
		if (bus_dph_err_d && x_amo_phase == 3'h0) begin
			assert(xm_except == EXCEPT_LOAD_FAULT || xm_except == EXCEPT_STORE_FAULT);
		end
	end else if (bus_dph_err_d) begin
		// 第 3 级中应有一条与该总线错误对应的总线传输
		assert(xm_memop != MEMOP_NONE);
		// 该错误应恰好捕获一次; 若当时还有
		// 其他异常源，那么这条 load/store 本来
		// 就不该发生。唯一的例外是 load/store
		// 的陷阱进入停顿在前端上。
		assert(xm_except == EXCEPT_NONE || (!m_trap_enter_rdy && (
			xm_except == EXCEPT_LOAD_FAULT || xm_except == EXCEPT_STORE_FAULT
		)));
		// 确保无需清除第 3 级的其他任何标志
		assert(!xm_except_to_d_mode);
		assert(!xm_sleep_wfi);
		assert(!xm_sleep_block);
		assert(!unblock_out);
	end
	// 某些异常->ls 路径被切断了，因为它们被认为
	// 不可能发生 —— 要确保不存在未取消的
	// load/store。（这包括调试进入！）
	if (m_trap_enter_vld) begin
		assert(!bus_aph_req_d);
	end
end
`endif

`ifdef OMNICORE_ASSERTIONS
always @ (posedge clk) if (rst_n) begin
	// D 级总线错误必须始终取消更年轻的 load/store
	if ($past(bus_dph_err_d && !bus_dph_ready_d))
		assert(!bus_aph_req_d);
	// 第 3 级有活跃的访存操作却对齐了第 2 级产生的异常，
	// 是荒谬的 —— 本应被取消。例外是 M 级产生的异常，
	// 即数据相总线错误。AMO 则另当别论。
	if (xm_except != EXCEPT_NONE && !(bus_dph_err_d || $past(!m_trap_enter_rdy)))
		assert(xm_memop == MEMOP_NONE || xm_memop == MEMOP_AMO);
	assert(xm_rd == $past(xm_rd_nxt));
end
`endif

// 数据通路触发器
always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		xm_result <= {W_DATA{1'b0}};
		xm_addr_align <= 2'b00;
	end else if (!m_stall && !(|EXTENSION_A && x_amo_phase == 3'h3 && !bus_dph_ready_d)) begin
		// AMO 需要特别关照（当然）:
		// - AMO 结束时把 mw_result 中捕获的读相数据经 xm_result 回送
		// - 确保 xm_result（store 数据）在停顿的写数据相期间不发生变化
		xm_result <=
			d_csr_ren                               ? x_csr_rdata       :
			|EXTENSION_A && x_amo_phase == 3'h3     ? mw_result         :
			|MUL_FASTER  && x_use_fast_mul          ? m_fast_mul_result :
			|EXTENSION_M && d_aluop == ALUOP_MULDIV ? x_muldiv_result   :
			                                          x_alu_result;
		xm_addr_align <= x_addr_sum[1:0];
	end
end

// ----------------------------------------------------------------------------
//                               流水线 M 级

reg [W_DATA-1:0] m_rdata_pick_sext;
reg [W_DATA-1:0] m_wdata;
reg [W_DATA-1:0] m_result;

assign f_jump_req = x_jump_req || m_trap_enter_vld;
assign f_jump_target = m_trap_enter_vld	? m_trap_addr : x_jump_target;
assign f_jump_priv = m_trap_enter_vld ? m_mmode_trap_entry : x_mmode_execution;
assign x_jump_not_except = !m_trap_enter_vld;

// 停顿与休眠控制

// 以下时序需要 EXCEPT_NONE 子句:
// - 第 0 拍: hresp 置起、hready 为低。我们设置异常以取消身后的指令。总线停顿为高。
// - 第 1 拍: hready 为高。由于某种原因，前端本拍无法接受陷阱地址。
// - 第 2 拍: 数据相已结束，bus_dph_ready_d 不会再脉冲。m_bus_stall 卡在高位。
wire m_bus_stall = m_dphase_in_flight && !bus_dph_ready_d && xm_except == EXCEPT_NONE && !(
	|EXTENSION_A && xm_memop == MEMOP_SC_W && !mw_local_exclusive_reserved
);

assign m_stall = m_bus_stall ||
	(m_trap_enter_vld && !m_trap_enter_rdy && !m_trap_is_irq) ||
	((xm_sleep_wfi || xm_sleep_block) && !m_sleep_stall_release);

// 异常针对当前位于 M 级的指令进入，因此要把 PC
// 回退。IRQ 插在 M 级指令与 X 级指令"之间"，
// 因此返回地址设为 X 级程序计数器。注意，进入
// 异常时可以断定，上一个位于 X 级的指令（现位于 M 级）
// *不是*发生跳转的分支，这正是可以直接回退 PC 的原因。
assign m_exception_return_addr = d_pc - (
	m_trap_is_irq         ? 32'd0 :
	xm_no_pc_increment    ? 32'd0 :
	prev_instr_was_32_bit ? 32'd4 : 32'd2
);

ominicore_power_ctrl power_ctrl (
	.clk_always_on          (clk_always_on),
	.rst_n                  (rst_n),

	.pwrup_req              (pwrup_req),
	.pwrup_ack              (pwrup_ack),
	.clk_en                 (clk_en),

	.allow_clkgate          (m_pwr_allow_clkgate),
	.allow_power_down       (m_pwr_allow_power_down),
	.allow_sleep_on_block   (m_pwr_allow_sleep_on_block),

	.frontend_pwrdown_ok    (f_frontend_pwrdown_ok),

	.sleeping_on_wfi        (xm_sleep_wfi),
	.wfi_wakeup_req         (m_wfi_wakeup_req),
	.sleeping_on_block      (xm_sleep_block),
	.block_wakeup_req_pulse (unblock_in),
	.stall_release          (m_sleep_stall_release)
);

// load/store 数据处理

always @ (*) begin
	// store 数据的本地前递
	if (|mw_rd && xm_rs2 == mw_rd && !REDUCED_BYPASS) begin
		m_wdata = mw_result;
	end else begin
		m_wdata = xm_result;
	end
	// 复制 store 数据以确保驱动正确的字节通道
	case (xm_memop)
		MEMOP_SH: bus_wdata_d = {2{m_wdata[15:0]}};
		MEMOP_SB: bus_wdata_d = {4{m_wdata[7:0]}};
		default:  bus_wdata_d = m_wdata;
	endcase

	casez ({xm_memop, xm_addr_align[1:0]})
		{MEMOP_LH  , 2'b0z}: m_rdata_pick_sext = {{16{bus_rdata_d[15]}}, bus_rdata_d[15: 0]};
		{MEMOP_LH  , 2'b1z}: m_rdata_pick_sext = {{16{bus_rdata_d[31]}}, bus_rdata_d[31:16]};
		{MEMOP_LHU , 2'b0z}: m_rdata_pick_sext = {{16{1'b0           }}, bus_rdata_d[15: 0]};
		{MEMOP_LHU , 2'b1z}: m_rdata_pick_sext = {{16{1'b0           }}, bus_rdata_d[31:16]};
		{MEMOP_LB  , 2'b00}: m_rdata_pick_sext = {{24{bus_rdata_d[ 7]}}, bus_rdata_d[ 7: 0]};
		{MEMOP_LB  , 2'b01}: m_rdata_pick_sext = {{24{bus_rdata_d[15]}}, bus_rdata_d[15: 8]};
		{MEMOP_LB  , 2'b10}: m_rdata_pick_sext = {{24{bus_rdata_d[23]}}, bus_rdata_d[23:16]};
		{MEMOP_LB  , 2'b11}: m_rdata_pick_sext = {{24{bus_rdata_d[31]}}, bus_rdata_d[31:24]};
		{MEMOP_LBU , 2'b00}: m_rdata_pick_sext = {{24{1'b0           }}, bus_rdata_d[ 7: 0]};
		{MEMOP_LBU , 2'b01}: m_rdata_pick_sext = {{24{1'b0           }}, bus_rdata_d[15: 8]};
		{MEMOP_LBU , 2'b10}: m_rdata_pick_sext = {{24{1'b0           }}, bus_rdata_d[23:16]};
		{MEMOP_LBU , 2'b11}: m_rdata_pick_sext = {{24{1'b0           }}, bus_rdata_d[31:24]};
		{MEMOP_LW  , 2'bzz}: m_rdata_pick_sext = bus_rdata_d;
		{MEMOP_LR_W, 2'bzz}: m_rdata_pick_sext = bus_rdata_d;
		default:             m_rdata_pick_sext = 32'hxxxx_xxxx;
	endcase

	if (|EXTENSION_A && x_amo_phase == 3'h1) begin
		// 把 AMO 读数据捕获进 mw_result，供回馈送入 ALU。
		m_result = bus_rdata_d;
	end else if (|EXTENSION_A && xm_memop == MEMOP_SC_W) begin
		// sc.w 可能因本地或全局监视器的否定响应而失败。
		// 注意结果的极性: 0 为成功，1 为失败。
		m_result = {31'h0, !(mw_local_exclusive_reserved && bus_dph_exokay_d)};
	end else if (xm_memop != MEMOP_NONE && xm_memop != MEMOP_AMO) begin
		m_result = m_rdata_pick_sext;
	end else if (MUL_FAST && m_fast_mul_result_vld) begin
		m_result = m_fast_mul_result;
	end else begin
		m_result = xm_result;
	end
end

// 本地监视器更新。
// - 全局监视器响应良好的 load-reserved 时置起
// - 任何 store-conditional 清除
// - 与调试无关的陷阱退出清除

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		mw_local_exclusive_reserved <= 1'b0;
	end else begin
		if (|EXTENSION_A && (!m_stall || bus_dph_err_d)) begin
			if (d_memop_is_amo) begin
				mw_local_exclusive_reserved <= 1'b0;
			end else if (xm_memop == MEMOP_SC_W && (bus_dph_ready_d || bus_dph_err_d)) begin
				mw_local_exclusive_reserved <= 1'b0;
			end else if (xm_memop == MEMOP_LR_W && bus_dph_ready_d) begin
				// 理论上，HRESP 置起时总线不应报告 HEXOKAY。
				// 但仍可能发生（例如 HEXOKAY 被接高电平），因此用
				// HRESP 屏蔽 HEXOKAY，确保失败的 lr.w 会清除监视器。
				mw_local_exclusive_reserved <= bus_dph_exokay_d && !bus_dph_err_d;
			end
		end
		if (m_clear_excl_on_trap_exit) begin
			mw_local_exclusive_reserved <= 1'b0;
		end
	end
end

// 注意异常进入会阻止写回，因为异常进入
// 会取代 M 级中的指令。中断进入不阻止写回，
// 因为中断在概念上插在 M 级指令
// 与 X 级指令之间。

wire m_reg_wen_if_nonzero = !m_bus_stall && xm_except == EXCEPT_NONE;
wire m_reg_wen = |xm_rd && m_reg_wen_if_nonzero;

`ifdef OMNICORE_X_CHECKS
always @ (posedge clk) begin
	if (rst_n) begin
		if (m_reg_wen && (^m_result === 1'bX)) begin
			$display("Writing X to register file!");
			$finish;
		end
	end
end
`endif

`ifdef OMNICORE_ASSERTIONS
// AMO 期间借用 mw_result 捕获 rdata 并回馈 ALU，
// 因为它已具备正确的路径。要确保这样做是安全的。
// （排在 AMO 之前、位于 M 级的指令，到 AMO
// 到达读数据相时应当已经通过）
always @ (posedge clk) if (rst_n) begin
	if (x_amo_phase == 3'h1)
		assert(m_reg_wen_if_nonzero);
	if (x_amo_phase == 3'h1)
		assert(~|xm_rd);
end
`endif

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		mw_result <= {W_DATA{1'b0}};
	end else if (m_reg_wen_if_nonzero && !(|EXTENSION_A && x_amo_phase[1])) begin
		// （AMO 的第 2/3 级期间不要破坏捕获的 AMO 读相数据 —— 我们需要它！）
		mw_result <= m_result;
	end
end

assign mw_rd_nxt = REGADDR_MASK & (
	m_reg_wen_if_nonzero ? xm_rd : mw_rd
);

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		mw_rd <= {W_REGADDR{1'b0}};
	end else begin
`ifdef OMNICORE_X_CHECKS
		if (!m_stall && ^bus_wdata_d === 1'bX) begin
			$display("Writing Xs to memory! pc=%h m_wdata=%h rs2sel=%b xm_rs2=%02d mw_rd=%02d mw_res=%h xm_res=%h",
				d_pc, m_wdata,
				(|mw_rd && xm_rs2 == mw_rd && !REDUCED_BYPASS), xm_rs2, mw_rd, mw_result, xm_result);
			$finish;
		end
`endif
		if (m_reg_wen_if_nonzero) begin
			mw_rd <= REGADDR_MASK & xm_rd;
		end
	end
end

ominicore_regfile_1w2r #(
`include "ominicore_config_inst.vh"
) regs (
	.clk    (clk),
	.rst_n  (rst_n),
	// 下游停顿时，把 D 级的地址回馈进寄存器堆，
	// 使输出保持不变。
	.raddr1 (x_stall && !d_starved ? d_rs1 : f_rs1_coarse),
	.rdata1 (x_rdata1),
	.raddr2 (x_stall && !d_starved ? d_rs2 : f_rs2_coarse),
	.rdata2 (x_rdata2),

	.waddr  (xm_rd),
	.wdata  (m_result),
	.wen    (m_reg_wen)
);

`ifdef RISCV_FORMAL
`include "ominicore_rvfi_monitor.vh"
`endif

`ifdef OMNICORE_FORMAL_REGRESSION
// 每次形式化回归各自提供具有下述名称的文件:
`include "ominicore_formal_regression.vh"
`endif

endmodule

`ifndef YOSYS
`default_nettype wire
`endif
