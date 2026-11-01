

/*****************************************************************************\
|    ominicore_core (ziguang 副本) + N 项 BTB 前端 (ziguang 副本)               |
|    + 2 路组相联 I$ / D$ + AHB5 master 对                                    |
|                                                                             |
|  对外接口是上游 ominicore_cpu_2port 的超集兼容替换:                            |
|  相同的 AHB5 端口对, 另加性能事件引出点。                                       |
|  与上游的差异（见 README）:                                                   |
|   - dbg_sbus（调试器系统总线访问）未布线；可另用                              |
|     AHB 适配层或调试器抽象命令。                                              |
|   - fence_i 使 I$ 失效; fence_d 触发 D$ 写回排空,                               |
|     且 fence_rdy 以排空完成为门控。                                           |
|                                                                             |
|  可缓存窗口为配置 include 中的 [ZG_CACHE_BASE, ZG_CACHE_LIMIT);               |
|  窗口之外按非缓存直通                                                          |
|  （TCM BRAM、外设、DMA 缓冲区）。                                              |
\*****************************************************************************/

`default_nettype none

module ominicore_cpu_2port_cached #(
`include "ominicore_config.vh"
	// I$/D$ 使能开关（ZG_ICACHE_EN/ZG_DCACHE_EN, 默认值见配置文件）。
	// 关闭时该侧 generate 剪除 cache 模块, 恢复与上游 cpu_2port 同构直通;
	// 配置文件同时提供 ZG_* 参数, 此处不再重复声明 (避免重复定义)。
) (
	// 全局信号
	input  wire                clk,
	input  wire                clk_always_on,
	input  wire                rst_n,

	// 电源控制信号
	output wire                pwrup_req,
	input  wire                pwrup_ack,
	output wire                clk_en,
	output wire                unblock_out,
	input  wire                unblock_in,

	// 取指端口（AHB5 master）
	output wire [W_ADDR-1:0]   i_haddr,
	output wire                i_hwrite,
	output wire [1:0]          i_htrans,
	output wire [2:0]          i_hsize,
	output wire [2:0]          i_hburst,
	output wire [3:0]          i_hprot,
	output wire                i_hmastlock,
	output wire [7:0]          i_hmaster,
	input  wire                i_hready,
	input  wire                i_hresp,
	output wire [W_DATA-1:0]   i_hwdata,
	input  wire [W_DATA-1:0]   i_hrdata,

	// load/store 端口（AHB5 master）
	output wire [W_ADDR-1:0]   d_haddr,
	output wire                d_hwrite,
	output wire [1:0]          d_htrans,
	output wire [2:0]          d_hsize,
	output wire [2:0]          d_hburst,
	output wire [3:0]          d_hprot,
	output wire                d_hmastlock,
	output wire                d_hexcl,
	output wire [7:0]          d_hmaster,
	input  wire                d_hready,
	input  wire                d_hresp,
	input  wire                d_hexokay,
	output wire [W_DATA-1:0]   d_hwdata,
	input  wire [W_DATA-1:0]   d_hrdata,

	// 内存排序信号
	output wire                fence_i_vld,
	output wire                fence_d_vld,
	input  wire                fence_rdy,

	// 调试器运行/暂停控制
	input  wire                dbg_req_halt,
	input  wire                dbg_req_halt_on_reset,
	input  wire                dbg_req_resume,
	output wire                dbg_halted,
	output wire                dbg_running,
	// 调试器访问 data0 CSR
	input  wire [W_DATA-1:0]   dbg_data0_rdata,
	output wire [W_DATA-1:0]   dbg_data0_wdata,
	output wire                dbg_data0_wen,
	// 调试器指令注入
	input  wire [W_DATA-1:0]   dbg_instr_data,
	input  wire                dbg_instr_data_vld,
	output wire                dbg_instr_data_rdy,
	output wire                dbg_instr_caught_exception,
	output wire                dbg_instr_caught_ebreak,

	// 标识 CSR 值
	input  wire [W_DATA-1:0]   mhartid_val,
	input  wire [3:0]          eco_version,

	// 电平触发中断源
	input  wire [NUM_IRQS-1:0] irq,
	input  wire                soft_irq,
	input  wire                timer_irq,

	// D 侧核心请求挂起 (板级仲裁用: D$ 默认拥有总线, 见 boardsoc_top)
	output wire                d_req_pending,

	// 性能事件引出点（接到 ominicore_perf slave）
	output wire                perf_ic_req,
	output wire                perf_ic_hit,
	output wire                perf_ic_miss,
	output wire                perf_ic_err,
	output wire                perf_dc_req,
	output wire                perf_dc_hit,
	output wire                perf_dc_miss,
	output wire                perf_dc_writeback,
	output wire                perf_dc_bypass,
	output wire                perf_br_alloc,
	output wire                perf_br_mispredict,
	output wire                perf_cpu_stall
);

	// ------------------------------------------------------------------
	// 核心（ziguang 副本：模块名 ominicore_core，由文件列表选择）
	// ------------------------------------------------------------------
	wire              core_aph_req_i;
	wire              core_aph_ready_i;
	wire              core_dph_ready_i;
	wire              core_dph_err_i;

	wire [W_ADDR-1:0] core_haddr_i;
	wire [2:0]        core_hsize_i;
	wire              core_priv_i;
	wire [W_DATA-1:0] core_rdata_i;

	wire              core_aph_req_d;
	wire              core_aph_excl_d;
	wire              core_aph_ready_d;
	wire              core_dph_ready_d;
	wire              core_dph_err_d;
	wire              core_dph_exokay_d;

	wire [W_ADDR-1:0] core_haddr_d;
	wire [2:0]        core_hsize_d;
	wire              core_priv_d;
	wire              core_hwrite_d;
	wire [W_DATA-1:0] core_wdata_d;
	wire [W_DATA-1:0] core_rdata_d;

	wire              ic_flush;
	wire              dc_drain_done;

	// fence.d 排序：核心的 fence_rdy 以 D$ 排空完成为条件，
	// 软件可确信脏数据已到达总线。
	wire core_fence_rdy = fence_rdy && dc_drain_done;

	// ZIGUANG (PMU): 前置声明 -- default_nettype none 下 ADS 单遍解析要求
	// 先声明后引用 (下方核心实例端口引用这三个事件, assign 在 PMU 小节)。
	wire zg_pmu_dcaccess;
	wire zg_pmu_dcmiss;
	wire zg_pmu_buswait;

	ominicore_core #(
	`include "ominicore_config_inst.vh"
	) core (
		.clk                        (clk),
		.clk_always_on              (clk_always_on),
		.rst_n                      (rst_n),

		.pwrup_req                  (pwrup_req),
		.pwrup_ack                  (pwrup_ack),
		.clk_en                     (clk_en),
		.unblock_out                (unblock_out),
		.unblock_in                 (unblock_in),

		.bus_aph_req_i              (core_aph_req_i),
		.bus_aph_panic_i            (),
		.bus_aph_ready_i            (core_aph_ready_i),
		.bus_dph_ready_i            (core_dph_ready_i),
		.bus_dph_err_i              (core_dph_err_i),
		.bus_haddr_i                (core_haddr_i),
		.bus_hsize_i                (core_hsize_i),
		.bus_priv_i                 (core_priv_i),
		.bus_rdata_i                (core_rdata_i),

		.bus_aph_req_d              (core_aph_req_d),
		.bus_aph_excl_d             (core_aph_excl_d),
		.bus_aph_ready_d            (core_aph_ready_d),
		.bus_dph_ready_d            (core_dph_ready_d),
		.bus_dph_err_d              (core_dph_err_d),
		.bus_dph_exokay_d           (core_dph_exokay_d),
		.bus_haddr_d                (core_haddr_d),
		.bus_hsize_d                (core_hsize_d),
		.bus_priv_d                 (core_priv_d),
		.bus_hwrite_d               (core_hwrite_d),
		.bus_wdata_d                (core_wdata_d),
		.bus_rdata_d                (core_rdata_d),

		.fence_i_vld                (fence_i_vld),
		.fence_d_vld                (fence_d_vld),
		.fence_rdy                  (core_fence_rdy),

		.dbg_req_halt               (dbg_req_halt),
		.dbg_req_halt_on_reset      (dbg_req_halt_on_reset),
		.dbg_req_resume             (dbg_req_resume),
		.dbg_halted                 (dbg_halted),
		.dbg_running                (dbg_running),
		.dbg_data0_rdata            (dbg_data0_rdata),
		.dbg_data0_wdata            (dbg_data0_wdata),
		.dbg_data0_wen              (dbg_data0_wen),
		.dbg_instr_data             (dbg_instr_data),
		.dbg_instr_data_vld         (dbg_instr_data_vld),
		.dbg_instr_data_rdy         (dbg_instr_data_rdy),
		.dbg_instr_caught_exception (dbg_instr_caught_exception),
		.dbg_instr_caught_ebreak    (dbg_instr_caught_ebreak),

		.mhartid_val                (mhartid_val),
		.eco_version                (eco_version),

		.irq                        (irq),
		.soft_irq                   (soft_irq),
		.timer_irq                  (timer_irq),

		.zg_evt_br_alloc            (perf_br_alloc),
		.zg_evt_br_flush            (perf_br_mispredict),
		.zg_evt_br_ctrdec           (),

		// ZIGUANG (规格 §10 PMU-01~04): cache/总线事件接入内核
		.zg_pmu_icaccess            (perf_ic_req),
		.zg_pmu_icmiss              (perf_ic_miss),
		.zg_pmu_dcaccess            (zg_pmu_dcaccess),
		.zg_pmu_dcmiss              (zg_pmu_dcmiss),
		.zg_pmu_buswait             (zg_pmu_buswait)
	);

	// ------------------------------------------------------------------
	// ZIGUANG (PMU-01): D 侧 cacheable 查询/load-miss 与总线等待事件。
	// D$=0 (bypass) 分支: cacheable 地址相受理即查询, cacheable load 即 miss
	// (无缓存可命中); D$=1 分支: 接 dcache 事件口 (待 D$ 启用时补齐, 现记 0)。
	// mbuswait = I$ 下游等待 || D 口数据相等待 (同拍只计一次)。
	// ------------------------------------------------------------------
	wire d_aph_accept_pm = core_aph_req_d && core_aph_ready_d;
	wire d_cacheable_pm  = (core_haddr_d >= ZG_CACHE_BASE) && (core_haddr_d < ZG_CACHE_LIMIT);
	assign zg_pmu_dcaccess = d_aph_accept_pm && d_cacheable_pm;
	// ZIGUANG (PMU 修正): mdcmiss 原实现 = cacheable load 受理数 (D$=0 占位
	// 语义未随 D$ 启用更新) -- "miss 79%" 实为 load 计数, 与命中无关, 曾误导
	// 出 "容量 miss" 定性 (探针实测真实 miss 率 ~0.2%, 见 B2-DESIGN §十一)。
	// D$=1 时按 dcache evt_miss (cacheable miss 受理拍, 载荷无关) 计数。
	assign zg_pmu_dcmiss   = zg_pmu_dcaccess && !core_hwrite_d &&
		(ZG_DCACHE_EN ? perf_dc_miss : 1'b1);
	wire zg_pmu_ic_buswait;
	wire zg_pmu_dc_buswait;
	assign zg_pmu_buswait  = zg_pmu_ic_buswait || zg_pmu_dc_buswait;

	// ------------------------------------------------------------------
	// 指令 cache
	// ------------------------------------------------------------------
	localparam HTRANS_IDLE = 2'b00;
	localparam HTRANS_NSEQ = 2'b10;

	generate if (ZG_ICACHE_EN) begin : icache_gen

	ominicore_icache #(
		.W_ADDR      (W_ADDR),
		.W_DATA      (W_DATA),
		.NUM_SETS    (ZG_ICACHE_SETS),
		.LINE_WORDS  (ZG_CACHE_LINE_WORDS),
		.CACHE_BASE  (ZG_CACHE_BASE),
		.CACHE_LIMIT (ZG_CACHE_LIMIT),
		.WRAP_FILL   (ZG_WRAP_FILL)
	) icache (
		.clk            (clk),
		.rst_n          (rst_n),

		.core_aph_req   (core_aph_req_i),
		.core_haddr     (core_haddr_i),
		.core_aph_ready (core_aph_ready_i),
		.core_dph_ready (core_dph_ready_i),
		.core_dph_err   (core_dph_err_i),
		.core_rdata     (core_rdata_i),

		.flush_req      (fence_i_vld),

		.evt_req        (perf_ic_req),
		.evt_hit        (perf_ic_hit),
		.evt_miss       (perf_ic_miss),
		.evt_err        (perf_ic_err),
		.evt_buswait    (zg_pmu_ic_buswait),

		.m_haddr        (i_haddr),
		.m_htrans       (i_htrans),
		.m_hsize        (i_hsize),
		.m_hburst       (i_hburst),
		.m_hwrite       (i_hwrite),
		.m_hmastlock    (i_hmastlock),
		.m_hwdata       (i_hwdata),
		.m_hready       (i_hready),
		.m_hresp        (i_hresp),
		.m_hrdata       (i_hrdata)
	);

	end else begin : icache_bypass
		// 上游 ominicore_cpu_2port 取指侧原样直通
		reg dphase_active_i;
		always @ (posedge clk or negedge rst_n) begin
			if (!rst_n)
				dphase_active_i <= 1'b0;
			else if (i_hready)
				dphase_active_i <= core_aph_req_i;
		end

		assign i_haddr         = core_haddr_i;
		assign i_htrans        = core_aph_req_i ? HTRANS_NSEQ : HTRANS_IDLE;
		assign i_hsize         = core_hsize_i;
		assign i_hwrite        = 1'b0;
		assign i_hburst        = 3'h0;
		assign i_hmastlock     = 1'b0;
		assign i_hwdata        = {W_DATA{1'b0}};

		assign core_aph_ready_i = i_hready && core_aph_req_i;
		assign core_dph_ready_i = i_hready && dphase_active_i;
		assign core_dph_err_i   = i_hready && dphase_active_i && i_hresp;
		assign core_rdata_i     = i_hrdata;

		assign perf_ic_req  = 1'b0;
		assign perf_ic_hit  = 1'b0;
		assign perf_ic_miss = 1'b0;
		assign perf_ic_err  = 1'b0;
		assign zg_pmu_ic_buswait = 1'b0;
	end endgenerate

	// ------------------------------------------------------------------
	// 数据 cache
	// ------------------------------------------------------------------
	generate if (ZG_DCACHE_EN) begin : dcache_gen

	ominicore_dcache #(
		.W_ADDR      (W_ADDR),
		.W_DATA      (W_DATA),
		.NUM_SETS    (ZG_DCACHE_SETS),
		.LINE_WORDS  (ZG_CACHE_LINE_WORDS),
		.CACHE_BASE  (ZG_CACHE_BASE),
		.CACHE_LIMIT (ZG_CACHE_LIMIT),
		.WRAP_FILL   (ZG_WRAP_FILL)
	) dcache (
		.clk            (clk),
		.rst_n          (rst_n),

		.core_aph_req   (core_aph_req_d),
		.core_aph_excl  (core_aph_excl_d),
		.core_haddr     (core_haddr_d),
		.core_hsize     (core_hsize_d),
		.core_hwrite    (core_hwrite_d),
		.core_wdata     (core_wdata_d),
		.core_aph_ready (core_aph_ready_d),
		.core_dph_ready (core_dph_ready_d),
		.core_dph_err   (core_dph_err_d),
		.core_dph_exokay(core_dph_exokay_d),
		.core_rdata     (core_rdata_d),

		.drain_req      (fence_d_vld),
		.drain_done     (dc_drain_done),

		.evt_req        (perf_dc_req),
		.evt_hit        (perf_dc_hit),
		.evt_miss       (perf_dc_miss),
		.evt_writeback  (perf_dc_writeback),
		.evt_bypass     (perf_dc_bypass),
		.evt_err        (),

		.m_haddr        (d_haddr),
		.m_htrans       (d_htrans),
		.m_hsize        (d_hsize),
		.m_hburst       (d_hburst),
		.m_hwrite       (d_hwrite),
		.m_hmastlock    (d_hmastlock),
		.m_hexcl        (d_hexcl),
		.m_hwdata       (d_hwdata),
		.m_hready       (d_hready),
		.m_hresp        (d_hresp),
		.m_hexokay      (d_hexokay),
		.m_hrdata       (d_hrdata)
	);

	end else begin : dcache_bypass
		// 上游 ominicore_cpu_2port load/store 侧原样直通（无 SBA 仲裁, 本顶层
		// 不路由 dbg_sbus, 与 tb 的 sbus 停用约定一致）
		wire bus_gnt_d = core_aph_req_d;
		reg  bus_active_dph_d;
		always @ (posedge clk_always_on or negedge rst_n) begin
			if (!rst_n)
				bus_active_dph_d <= 1'b0;
			else if (d_hready)
				bus_active_dph_d <= bus_gnt_d;
		end

		assign d_haddr     = core_haddr_d;
		assign d_hwrite    = core_hwrite_d;
		assign d_hsize     = core_hsize_d;
		assign d_hexcl     = core_aph_excl_d;
		assign d_hwdata    = core_wdata_d;
		assign d_htrans    = bus_gnt_d ? HTRANS_NSEQ : HTRANS_IDLE;
		assign d_hburst    = 3'h0;
		assign d_hmastlock = 1'b0;

		// D 侧错误即使在非 ready 拍也上报, 供核心用两相错误响应干净地
		// 压掉在流水线中追赶故障访问的第二笔 load/store（上游注释原样保留）
		assign core_aph_ready_d   = d_hready && bus_gnt_d;
		assign core_dph_ready_d   = bus_active_dph_d && d_hready;
		assign core_dph_err_d     = bus_active_dph_d && d_hresp;
		assign core_dph_exokay_d  = bus_active_dph_d && d_hexokay;
		assign core_rdata_d       = d_hrdata;

		// 无脏数据, fence.d 瞬时完成
		assign dc_drain_done = 1'b1;

		assign perf_dc_req       = 1'b0;
		assign perf_dc_hit       = 1'b0;
		assign perf_dc_miss      = 1'b0;
		assign perf_dc_writeback = 1'b0;
		assign perf_dc_bypass    = 1'b0;
		// ZIGUANG (PMU-01): D 口数据相在途且 hready 未到 = 下游等待一拍
		assign zg_pmu_dc_buswait = bus_active_dph_d && !d_hready;
	end endgenerate

	// ------------------------------------------------------------------
	// AHB5 辅助信号
	// ------------------------------------------------------------------
	assign d_req_pending = core_aph_req_d;

	assign i_hprot = {2'b00, core_priv_i, 1'b0};   // 取指访问
	assign d_hprot = {2'b00, core_priv_d, 1'b1};   // 数据访问
	assign d_hmaster = 8'h00;
	assign i_hmaster = 8'h00;

`ifdef ZG_WATCH_STACK
	// 栈帧监视：观察核心侧对栈顶 ra 槽（0x800ffffc）的每次读/写
	reg [31:0] watch_addr_r;
	reg        watch_wr_r;
	reg [31:0] watch_wd_r;
	reg        watch_hit_r, watch_miss_r, watch_byp_r;
	reg [31:0] zg_watch_ctr = 32'h0;
	always @ (posedge clk) begin
		zg_watch_ctr <= zg_watch_ctr + 32'h1;
		if (core_aph_req_d && core_aph_ready_d) begin
			watch_addr_r <= core_haddr_d;
			watch_wr_r   <= core_hwrite_d;
			watch_wd_r   <= core_wdata_d;
			watch_hit_r  <= perf_dc_hit;
			watch_miss_r <= perf_dc_miss;
			watch_byp_r  <= perf_dc_bypass;
		end
		if (core_dph_ready_d && watch_addr_r >= 32'h800fff00)
			$display("[WATCH] ctr=%08x wr=%01x hit=%01x miss=%01x byp=%01x addr=%08x aph_wdata=%08x rdata=%08x",
				zg_watch_ctr, {31'h0, watch_wr_r}, {31'h0, watch_hit_r}, {31'h0, watch_miss_r},
				{31'h0, watch_byp_r}, watch_addr_r, watch_wd_r, core_rdata_d);
	end
`endif

	// ------------------------------------------------------------------
	// CPU stall 引出点：等待 cache 的访存请求即是 stall 来源
	// ------------------------------------------------------------------
	reg stall_ic, stall_dc;
	always @ (posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			stall_ic <= 1'b0;
			stall_dc <= 1'b0;
		end else begin
			stall_ic <= core_aph_req_i && !core_aph_ready_i;
			stall_dc <= core_aph_req_d && !core_aph_ready_d;
		end
	end
	assign perf_cpu_stall = stall_ic || stall_dc;

endmodule

`default_nettype wire
