

/*****************************************************************************\
|  ominicore_dcache.v — OmniCore 2 路组相联写回式数据 cache                       |
|                                                                             |
|  紫光同创赛题 高阶任务①: 存储系统优化（2 路组相联 D-Cache + 突发优化）              |
|                                                                             |
|  策略:                                                                       |
|   - 写回 + 写分配。脏的受害行先以 4 beat INCR4 突发写向下游                     |
|     （地址由其存储的 tag 重构），然后该行才被重填。                              |
|   - 行填充在 WRAP_FILL=1 时用以缺失字为锚点的 WRAP4                             |
|     （关键字优先 critical-word-first），否则从行基址起 INCR4。                  |
|   - 独占访问（lr.w / sc.w / amo*）旁路 cache，与下游全局                        |
|     监视器完成交互；随后目标行的缓存副本被失效。                                 |
|   - [CACHE_BASE, CACHE_LIMIT) 内的地址走 cache；其余按                          |
|     单次访问直通。DMA 缓冲区（摄像头、加速器）请放在本窗口                       |
|     之外，或在 DMA 传输前后排空 cache。                                        |

\*****************************************************************************/

`default_nettype none

module ominicore_dcache #(
	parameter W_ADDR           = 32,
	parameter W_DATA           = 32,
	parameter NUM_SETS         = 256,            // 2 的幂。
	parameter LINE_WORDS       = 4,              // ZIGUANG (B3/CACHE-01): 每行字数 (4=16B, 8=32B)
	parameter CACHE_BASE       = 32'h00000000,
	parameter CACHE_LIMIT      = 32'h40000000,
	parameter WRAP_FILL        = 1
) (
	input  wire                clk,
	input  wire                rst_n,

	// 处理器侧 load/store 接口
	input  wire                core_aph_req,
	input  wire                core_aph_excl,
	input  wire [W_ADDR-1:0]   core_haddr,
	input  wire [2:0]          core_hsize,
	input  wire                core_hwrite,
	input  wire [W_DATA-1:0]   core_wdata,
	output wire                core_aph_ready,
	output wire                core_dph_ready,
	output wire                core_dph_err,
	output wire                core_dph_exokay,
	output wire [W_DATA-1:0]   core_rdata,

	// cache 维护
	input  wire                drain_req,        // 脉冲：写回所有脏行（fence.d）
	output wire                drain_done,       // 无挂起或进行中的排空

	// 统计事件脉冲（送往 ominicore_perf）
	output wire                evt_req,
	output wire                evt_hit,
	output wire                evt_miss,
	output wire                evt_writeback,
	output wire                evt_bypass,
	output wire                evt_err,

	// 下游 AHB5 master 端口（独占持有）
	output wire [W_ADDR-1:0]   m_haddr,
	output wire [1:0]          m_htrans,
	output wire [2:0]          m_hsize,
	output wire [2:0]          m_hburst,
	output wire                m_hwrite,
	output wire                m_hmastlock,
	output wire                m_hexcl,
	output wire [W_DATA-1:0]   m_hwdata,
	input  wire                m_hready,
	input  wire                m_hresp,
	input  wire                m_hexokay,
	input  wire [W_DATA-1:0]   m_hrdata
);

	function integer clog2;
		input integer value;
		begin
			for (clog2 = 0; value > 1; clog2 = clog2 + 1)
				value = value >> 1;
		end
	endfunction

	wire hit_accept;   // (2026-10-09 前移: vlog 严格先声明后使用)
	localparam OFF_BITS   = clog2(LINE_WORDS);        // 每行字数
	localparam IDX_BITS   = clog2(NUM_SETS);
	localparam BEAT_BITS  = clog2(LINE_WORDS);        // AHB 突发 beat 计数位宽
	localparam TAG_BITS   = W_ADDR - OFF_BITS - 2 - IDX_BITS;
	localparam AWR_BITS   = OFF_BITS + IDX_BITS;      // 阵列字索引: {组, 行内字}

	localparam HTRANS_IDLE = 2'b00;
	localparam HTRANS_NSEQ = 2'b10;
	localparam HTRANS_SEQ  = 2'b11;
	localparam HBURST_SINGLE = 3'b000;
	localparam HBURST_WRAP4  = 3'b010;
	localparam HBURST_INCR4  = 3'b011;
	localparam HBURST_INCR8  = 3'b111;   // ZIGUANG (B3): 8-beat 突发 (32B 行)
	localparam HBURST_FILL   = (LINE_WORDS == 8) ? HBURST_INCR8 :
	                           (WRAP_FILL       ? HBURST_WRAP4  : HBURST_INCR4);
	// LINE_WORDS 为 2 的幂 (4/8): LINE_WORDS-1 即 BEAT_BITS 位全 1
	localparam [BEAT_BITS-1:0] BEAT_LAST = {BEAT_BITS{1'b1}};

	// ------------------------------------------------------------------
	// 存储：字节通道阵列（利于 BRAM 字节写），tag 用触发器
	// ------------------------------------------------------------------
	reg [7:0] way0_b0 [0:NUM_SETS*LINE_WORDS-1];
	reg [7:0] way0_b1 [0:NUM_SETS*LINE_WORDS-1];
	reg [7:0] way0_b2 [0:NUM_SETS*LINE_WORDS-1];
	reg [7:0] way0_b3 [0:NUM_SETS*LINE_WORDS-1];
	reg [7:0] way1_b0 [0:NUM_SETS*LINE_WORDS-1];
	reg [7:0] way1_b1 [0:NUM_SETS*LINE_WORDS-1];
	reg [7:0] way1_b2 [0:NUM_SETS*LINE_WORDS-1];
	reg [7:0] way1_b3 [0:NUM_SETS*LINE_WORDS-1];

	reg [TAG_BITS-1:0] tag0 [0:NUM_SETS-1];
	reg [TAG_BITS-1:0] tag1 [0:NUM_SETS-1];
	reg [NUM_SETS-1:0] valid0, valid1, dirty0, dirty1;

	integer i;

	// ------------------------------------------------------------------
	// 请求译码
	// ------------------------------------------------------------------
	// verilator lint_off UNSIGNED
	wire addr_cacheable = (core_haddr >= CACHE_BASE) && (core_haddr < CACHE_LIMIT);
	// verilator lint_on UNSIGNED

	wire [IDX_BITS-1:0] req_idx = core_haddr[OFF_BITS + 2 +: IDX_BITS];
	wire [TAG_BITS-1:0] req_tag = core_haddr[OFF_BITS + 2 + IDX_BITS +: TAG_BITS];
	wire [OFF_BITS-1:0] req_off = core_haddr[OFF_BITS+1:2];   // 行内字序号 (B3: 随 LINE_WORDS)

	wire hit0 = valid0[req_idx] && (tag0[req_idx] == req_tag);
	wire hit1 = valid1[req_idx] && (tag1[req_idx] == req_tag);
	wire req_hit = hit0 || hit1;

	// ZIGUANG (B4, CACHE-05): 真 LRU —— 2-way 下每组 1 bit 即精确 LRU。
	// lru_way[i]=0: way0 是最近使用 (下次逐出 way1); =1: way1 最近使用。
	reg [NUM_SETS-1:0] lru_way;
	always @ (posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			lru_way <= {NUM_SETS{1'b0}};
		end else begin
			// 命中即更新 LRU 状态 (load/store 命中同路)
			if (hit_accept && hit0)
				lru_way[req_idx] <= 1'b0;
			else if (hit_accept && hit1)
				lru_way[req_idx] <= 1'b1;
		end
	end

	wire victim_way_sel =
		(!valid0[req_idx] && valid1[req_idx]) ? 1'b0 :
		(!valid1[req_idx] && valid0[req_idx]) ? 1'b1 :
		                                        lru_way[req_idx];  // LRU: 0=逐 way1, 1=逐 way0

	// 被逐出行存储的 tag（用于重构其总线地址）
	wire [TAG_BITS-1:0] victim_tag = victim_way_sel ? tag1[req_idx] : tag0[req_idx];
	wire victim_dirty = victim_way_sel ? dirty1[req_idx] : dirty0[req_idx];

	// store 字节通道译码（非对齐访问由核心在上游拆分）
	reg [3:0] st_lane_mask;
	always @ (*) begin
		case (core_hsize[1:0])
			2'b00:   st_lane_mask = 4'b0001 << core_haddr[1:0];        // 字节
			2'b01:   st_lane_mask = 4'b0011 << {core_haddr[1], 1'b0};  // 半字
			default: st_lane_mask = 4'b1111;                            // 字
		endcase
	end

	// ------------------------------------------------------------------
	// FSM 状态
	// ------------------------------------------------------------------
	localparam S_IDLE        = 4'd0;
	localparam S_EXCL_APH    = 4'd2;
	localparam S_EXCL_DPH    = 4'd3;
	localparam S_BYP_APH     = 4'd4;
	localparam S_BYP_DPH     = 4'd5;
	localparam S_WB_EXTRACT  = 4'd6;  // 读脏受害行入 wb_buf
	localparam S_WB_WRITE    = 4'd7;  // 向下游写受害/排空突发
	localparam S_FILL        = 4'd8;  // 重填突发
	localparam S_STORE_APPLY = 4'd9;  // store 缺失：填充后施加 store
	localparam S_DRAIN       = 4'd10; // fence.d 扫描：逐组查找脏行
	localparam S_ERR_DPH     = 4'd11; // 为失败的填充交付总线错误

	reg [3:0] state;

	// 在途事务跟踪（上游只允许一个在途操作）
	reg dph_pending;
	reg hit_dph_armed;
	reg excl_pending;
	reg byp_pending;
	reg byp_is_write;
	reg [2:0] byp_hsize;   // ZIGUANG: 锁存访问尺寸, 旁路子字 store 按真实 hsize 上总线
	reg [W_ADDR-1:0] byp_addr;    // 锁存：数据相期间 core_haddr 已前进
	reg        fill_alloc_req;    // 旁路 load 缺失后需要后台填充
	reg [IDX_BITS-1:0] fill_alloc_idx;
	reg [TAG_BITS-1:0] fill_alloc_tag;
	reg        fill_alloc_way;
	reg [W_ADDR-1:0] excl_addr;   // 锁存的独占操作地址
	reg [1:0]        excl_hitway; // 0 = 未缓存, 1 = way0, 2 = way1
	reg [IDX_BITS-1:0] excl_idx;

	reg excl_aph_done;
	reg byp_aph_done;

	// 填充记账
	reg       fill_is_store;
	reg       fill_no_dph;   // ZIGUANG: 背景填充 (核心数据相已经旁路交付)。
	                       // 此类填充出错必须静默放弃——核心无挂起数据相,
	                       // 上报会触发伪异常 (zcmp_pop_bus_fault 挂死根因)
	// ZG_DEBUG_FILL 门控占位
	reg       fill_fwd_kill;
	reg       fill_err;
	reg       fill_flushed;
	reg [BEAT_BITS-1:0] aph_beat;    // 本周期呈现地址的 beat（突发）
	reg [BEAT_BITS-1:0] dph_beat;    // 本周期总线上数据的 beat（突发）
	reg       dph_valid;   // m_hrdata/m_hwdata 属于 beat dph_beat
	reg       aph_done;    // 全部 4 个地址相均已受理
	reg [BEAT_BITS-1:0] fill_beat;   // 通用突发 beat（写回路径）
	reg [OFF_BITS-1:0] fill_off;
	reg       fill_way;
	reg [IDX_BITS-1:0] fill_idx;
	reg [TAG_BITS-1:0] fill_tag;
	reg       fill_err_pending; // 填充中止后上报总线错误

	// 受害行写回记账
	reg       wb_from_drain;
	reg       wb_from_excl;   // 先写回独占目标行，再执行独占操作
	reg       e_write;        // 独占操作写内存（sc / amo*）
	reg       wb_way;
	reg [IDX_BITS-1:0] wb_idx;
	reg [TAG_BITS-1:0] wb_tag;
	reg [BEAT_BITS-1:0] wb_ext_beat;
	reg [W_DATA-1:0] wb_buf [0:LINE_WORDS-1];

	// 排空
	reg     drain_latch;
	reg     drain_req_prev;
	reg     drain_active;
	reg [IDX_BITS-1:0] drain_idx;

	// 命中读数据
	reg [W_DATA-1:0] rd_rdata;

	// ------------------------------------------------------------------
	// 受理逻辑（组合逻辑）
	// ------------------------------------------------------------------
	wire hit_dph_deliver_now = dph_pending && hit_dph_armed;

	// 写策略: 写直通 + 写不分配（store 全部经总线旁路完成）。
	// 交付策略: 所有访问的交付形状与上游零等待包装器严格同构——
	//   命中: 接受拍上阵列读, 下一拍交付;
	//   缺失/旁路: 接受拍上总线, 下一拍交付总线数据;
	//   load 缺失另在交付后启动后台行填充（分配）, 核心只见 hready 拉伸。
	// ZIGUANG (A2/fence.d): 已锁存的 drain 会阻塞新的受理，因此核心
	// 正在呈现请求时扫描也能启动。若等待 !core_aph_req 反而死锁：
	// fence 停住 M 级的同时下一条 load 停在 X，使 core_aph_req 永远为高。
	wire drain_blocks_accept = drain_latch && state == S_IDLE;
	assign hit_accept = core_aph_req && !core_aph_excl && addr_cacheable && req_hit
		&& !core_hwrite
		&& (state == S_IDLE) && (!dph_pending || hit_dph_deliver_now)
		&& !drain_blocks_accept;

	wire excl_accept = core_aph_req && core_aph_excl
		&& (state == S_IDLE) && !dph_pending && !drain_blocks_accept;

	wire bypass_accept = core_aph_req && !core_aph_excl
		&& (state == S_IDLE) && !dph_pending && m_hready && !drain_blocks_accept;

	assign core_aph_ready = hit_accept || excl_accept || bypass_accept;

	// ------------------------------------------------------------------
	// 控制
	// ------------------------------------------------------------------
	always @ (posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state            <= S_IDLE;
			dph_pending      <= 1'b0;
			hit_dph_armed    <= 1'b0;
			excl_pending     <= 1'b0;
			byp_pending      <= 1'b0;
			excl_aph_done    <= 1'b0;
			byp_aph_done     <= 1'b0;
			fill_alloc_req   <= 1'b0;
			fill_is_store    <= 1'b0;
			fill_err         <= 1'b0;
			fill_flushed     <= 1'b0;
			aph_beat <= {BEAT_BITS{1'b0}};
			dph_beat <= {BEAT_BITS{1'b0}};
			dph_valid        <= 1'b0;
			aph_done         <= 1'b0;
			fill_beat <= {BEAT_BITS{1'b0}};
			fill_off         <= {OFF_BITS{1'b0}};
			fill_way         <= 1'b0;
			fill_idx         <= {IDX_BITS{1'b0}};
			fill_tag         <= {TAG_BITS{1'b0}};
			fill_err_pending <= 1'b0;
			wb_from_drain    <= 1'b0;
			wb_from_excl     <= 1'b0;
			e_write          <= 1'b0;
			wb_way           <= 1'b0;
			wb_idx           <= {IDX_BITS{1'b0}};
			wb_tag           <= {TAG_BITS{1'b0}};
			wb_ext_beat <= {BEAT_BITS{1'b0}};
			drain_latch      <= 1'b0;
			drain_req_prev   <= 1'b0;
			drain_active     <= 1'b0;
			drain_idx        <= {IDX_BITS{1'b0}};
			valid0           <= {NUM_SETS{1'b0}};
			valid1           <= {NUM_SETS{1'b0}};
			dirty0           <= {NUM_SETS{1'b0}};
			dirty1           <= {NUM_SETS{1'b0}};
		end else begin
			// ZIGUANG (A2/fence.d): fence.d 扫描请求仅在上升沿锁存。
			// fence_d_vld 在核心看到 fence_rdy 前一直为高，若用电平敏感
			// 锁存，扫描一结束就会重新启动，fence_rdy 将永远无法置起
			// （S_DRAIN 中的活锁）。扫描从 S_IDLE 附近的延迟块启动，
			// 并失效每一行，使后续 load 重新到达总线。
			if (drain_req && !drain_req_prev && !drain_latch && !drain_active)
				drain_latch <= 1'b1;
			drain_req_prev <= drain_req;

			// --- 地址相受理 ---
			if (hit_accept) begin
				dph_pending   <= 1'b1;
				hit_dph_armed <= 1'b1;
				if (core_hwrite) begin
					// 命中 store：标记该行为脏（dirty 阵列
					// 仅由本块驱动）
					if (hit0) dirty0[req_idx] <= 1'b1;
					if (hit1) dirty1[req_idx] <= 1'b1;
				end
			end else if (excl_accept) begin
				excl_pending  <= 1'b1;
				excl_aph_done <= 1'b0;
				e_write       <= core_hwrite;
				excl_addr     <= core_haddr;
				excl_idx      <= req_idx;
				excl_hitway   <= hit0 ? 2'd1 : hit1 ? 2'd2 : 2'd0;
				if (req_hit) begin
					// 目标行已缓存：若为脏，其数据必须先到达内存，
					// 独占操作才能绕过 cache 触碰该地址。
					// 先写回，再执行独占操作。
					wb_from_drain <= 1'b0;
					wb_from_excl  <= 1'b1;
					wb_way        <= hit0 ? 1'b0 : 1'b1;
					wb_idx        <= req_idx;
					wb_tag        <= hit0 ? tag0[req_idx] : tag1[req_idx];
					wb_ext_beat <= {BEAT_BITS{1'b0}};
					aph_beat <= {BEAT_BITS{1'b0}};
					dph_beat <= {BEAT_BITS{1'b0}};
					dph_valid     <= 1'b0;
					aph_done      <= 1'b0;
					fill_beat <= {BEAT_BITS{1'b0}};
					state         <= (hit0 && dirty0[req_idx]) || (hit1 && dirty1[req_idx]) ?
					                S_WB_EXTRACT : S_EXCL_APH;
				end else begin
					state <= S_EXCL_APH;
				end
			end else if (bypass_accept) begin
				byp_pending     <= 1'b1;
				byp_aph_done    <= 1'b0;
				byp_hsize       <= core_hsize;
				byp_is_write    <= core_hwrite;
				byp_addr        <= core_haddr;
				// 锁存写数据: 旁路写的总线写在 dph 拍用锁存值驱动。
				// 核心的 core_wdata 只保证在数据相前有效, 而旁路 dph 在
				// 接受后一拍——期间下一条指令已推进流水线, 活采
				// core_wdata 会把错误数据写入内存（曾导致栈上数据被清零）。
				byp_aph_done    <= 1'b1;
				// ZIGUANG (B2'): 写分配 —— store 缺失也填充行 (直通写已落
				// 内存, fill 读到的内存含本 store 值), 后续同地址 load 命中,
				// 消除 store->load 失效抖动 (miss 率 79% 的根源)。
				fill_alloc_req  <= addr_cacheable && !req_hit;
				fill_alloc_idx  <= req_idx;
				fill_alloc_tag  <= req_tag;
				fill_alloc_way  <= victim_way_sel;
				state           <= S_BYP_DPH;
			end

			// --- 命中数据相在受理后一拍送达 ---
			// 让位于同周期的再次受理（背靠背命中）
			if (hit_dph_armed && !hit_accept && !excl_accept && !bypass_accept) begin
				dph_pending   <= 1'b0;
				hit_dph_armed <= 1'b0;
			end

			// --- 待交付的总线错误 ---
			if (fill_err_pending) begin
				fill_err_pending <= 1'b0;
				dph_pending      <= 1'b0;
			end

			case (state)
				S_EXCL_APH: begin
					if (m_hready) begin
						excl_aph_done <= 1'b1;
						state         <= S_EXCL_DPH;
					end
				end

				S_EXCL_DPH: begin
					if (m_hready) begin
						// 写型独占（sc / amo*）绕过 cache 改动了内存：
						// 失效缓存副本，使下一次 load 重新取数。
						// 干净的 lr.w 可以保留它。
						if (e_write && excl_hitway != 2'd0) begin
							if (excl_hitway == 2'd1) valid0[excl_idx] <= 1'b0;
							else                     valid1[excl_idx] <= 1'b0;
						end
						excl_pending <= 1'b0;
						dph_pending  <= 1'b0;
						state        <= S_IDLE;
					end
				end

				S_BYP_DPH: begin
					if (m_hready) begin
						byp_pending <= 1'b0;
						dph_pending <= 1'b0;
						if (fill_alloc_req) begin
							// 后台行填充: WRAP4 关键字优先, 只写阵列提交
							// 标签, 不向核心交付任何数据相。
							fill_err      <= 1'b0;
							aph_beat <= {BEAT_BITS{1'b0}};
							dph_beat <= {BEAT_BITS{1'b0}};
							dph_valid     <= 1'b0;
							aph_done      <= 1'b0;
							fill_off      <= byp_addr[OFF_BITS+1:2];   // B3: 位段随 LINE_WORDS
							fill_idx      <= fill_alloc_idx;
							fill_tag      <= fill_alloc_tag;
							fill_way      <= fill_alloc_way;
							wb_from_drain <= 1'b0;
							wb_way        <= fill_alloc_way;
							wb_idx        <= fill_alloc_idx;
							wb_tag        <= fill_alloc_tag;
							wb_ext_beat <= {BEAT_BITS{1'b0}};
							fill_alloc_req<= 1'b0;
							fill_no_dph   <= 1'b1;
							state         <= S_FILL;
						end else begin
							state       <= S_IDLE;
						end
					end
				end

				S_WB_EXTRACT: begin
					// wb_buf[wb_ext_beat] 由下方阵列读块锁存
					if (wb_ext_beat == BEAT_LAST) begin
						aph_beat <= {BEAT_BITS{1'b0}};
						dph_beat <= {BEAT_BITS{1'b0}};
						dph_valid  <= 1'b0;
						aph_done   <= 1'b0;
						fill_beat <= {BEAT_BITS{1'b0}};
						state      <= S_WB_WRITE;
					end else begin
						wb_ext_beat <= wb_ext_beat + 1'b1;
					end
				end

				S_WB_WRITE: begin
					// 写突发：消费 beat dph_beat 的数据相，
					// 呈现 beat aph_beat 的地址相。
					if (m_hready && !dph_valid) begin
						// beat 0 的地址刚被受理；其数据相现在开始
						dph_valid <= 1'b1;
						dph_beat <= {BEAT_BITS{1'b0}};
						aph_beat  <= 1;
					end else if (dph_valid && m_hready) begin
						if (dph_beat == BEAT_LAST) begin
							dph_valid <= 1'b0;
							if (wb_from_drain) begin
								// ZIGUANG (A2/fence.d): drain 是完全失效，
								// 而不只是清洗——此处保持 valid 的行会把陈旧
								// 数据供给后续 load，而 test/bus master 已无法
								// 观测到它们。清洗的同时失效并继续扫描。
								if (wb_way) begin
									dirty1[wb_idx] <= 1'b0;
									valid1[wb_idx] <= 1'b0;
								end else begin
									dirty0[wb_idx] <= 1'b0;
									valid0[wb_idx] <= 1'b0;
								end
								drain_idx <= drain_idx + {{(IDX_BITS-1){1'b0}}, 1'b1};
								if (&drain_idx) begin
									drain_active <= 1'b0;
									state        <= S_IDLE;
								end else begin
									state <= S_DRAIN;
								end
							end else if (wb_from_excl) begin
								// 行已在内存中清洗：执行独占操作
								state <= S_EXCL_APH;
							end else begin
								state <= S_FILL;
							end
						end else begin
							dph_valid <= 1'b1;
							dph_beat <= dph_beat + 1'b1;
						end
					end
					if (m_hready && !aph_done) begin
						if (aph_beat == BEAT_LAST)
							aph_done <= 1'b1;
						else
							aph_beat <= aph_beat + 1'b1;
					end
				end

				S_FILL: begin
					// Read burst: 消费数据相, 关键字优先转发给核心
					if (m_hready && !dph_valid) begin
						// beat 0 的地址刚被受理；其数据相现在开始
						dph_valid <= 1'b1;
						dph_beat <= {BEAT_BITS{1'b0}};
						aph_beat  <= 1;
					end else if (dph_valid && m_hready) begin
						if (m_hresp)
							fill_err <= 1'b1;
						if (dph_beat == BEAT_LAST) begin
							if (!fill_err && !m_hresp) begin   // B3: 末拍 hresp 同拍可见

								if (fill_way) begin
									tag1[fill_idx]   <= fill_tag;
									valid1[fill_idx] <= 1'b1;
									dirty1[fill_idx] <= 1'b0;
								end else begin
									tag0[fill_idx]   <= fill_tag;
									valid0[fill_idx] <= 1'b1;
									dirty0[fill_idx] <= 1'b0;
								end
								dph_pending <= 1'b0;
								fill_no_dph <= 1'b0;
								state       <= S_IDLE;
							end else if (fill_no_dph) begin
								// 背景填充出错: 静默放弃 (不提交标签、不上报)
								fill_no_dph <= 1'b0;
								state       <= S_IDLE;
							end else begin
								fill_err_pending <= 1'b1;
								state            <= S_ERR_DPH;
							end
						end else begin
							dph_valid <= 1'b1;
							dph_beat <= dph_beat + 1'b1;
						end
					end
					if (m_hready && !aph_done) begin
						if (aph_beat == BEAT_LAST)
							aph_done <= 1'b1;
						else
							aph_beat <= aph_beat + 1'b1;
					end
				end

				S_STORE_APPLY: begin
					if (fill_way) begin
						dirty1[fill_idx] <= 1'b1;
					end else begin
						dirty0[fill_idx] <= 1'b1;
					end
					dph_pending <= 1'b0;
					state       <= S_IDLE;
				end

				S_ERR_DPH: begin
					state <= S_IDLE;
				end

				S_DRAIN: begin
					if (valid0[drain_idx] && dirty0[drain_idx]) begin
						wb_from_drain <= 1'b1;
						wb_way        <= 1'b0;
						wb_idx        <= drain_idx;
						wb_tag        <= tag0[drain_idx];
						wb_ext_beat <= {BEAT_BITS{1'b0}};
						aph_beat <= {BEAT_BITS{1'b0}};
						dph_beat <= {BEAT_BITS{1'b0}};
						dph_valid     <= 1'b0;
						aph_done      <= 1'b0;
						fill_beat <= {BEAT_BITS{1'b0}};
						state         <= S_WB_EXTRACT;
					end else if (valid1[drain_idx] && dirty1[drain_idx]) begin
						wb_from_drain <= 1'b1;
						wb_way        <= 1'b1;
						wb_idx        <= drain_idx;
						wb_tag        <= tag1[drain_idx];
						wb_ext_beat <= {BEAT_BITS{1'b0}};
						aph_beat <= {BEAT_BITS{1'b0}};
						dph_beat <= {BEAT_BITS{1'b0}};
						dph_valid     <= 1'b0;
						aph_done      <= 1'b0;
						fill_beat <= {BEAT_BITS{1'b0}};
						state         <= S_WB_EXTRACT;
					end else begin
						// ZIGUANG (A2/fence.d): fence.d 是完全的 cache
						// 失效。写直通使行保持干净，仅清洗的扫描就是
						// 空操作，陈旧的 valid 行会在无总线事务的情况下
						// 继续供数给 load（zcmp *_bus_fault 测试依赖每个
						// 下毒的 load 真正到达总线）。干净行也一并失效；
						// 脏行在其上方写回完成时被失效。
						valid0[drain_idx] <= 1'b0;
						valid1[drain_idx] <= 1'b0;
						drain_idx <= drain_idx + {{(IDX_BITS-1){1'b0}}, 1'b1};
						if (&drain_idx) begin
							drain_active <= 1'b0;
							state        <= S_IDLE;
						end
					end
				end

				default: ;
			endcase

			// 引擎完全空闲时启动已锁存的 drain。有了上面的
			// drain_blocks_accept，锁存一旦置起同周期受理即被抑制，
			// 因此不会在此丢弃任何已受理的请求。（ZIGUANG A2/fence.d：
			// 切勿等待 !core_aph_req——fence 停住 M 级时停在 X 的
			// 请求永远不会撤销，会使 fence 死锁。）
			if (drain_latch && state == S_IDLE && !dph_pending) begin
				drain_latch  <= 1'b0;
				drain_active <= 1'b1;
				drain_idx    <= {IDX_BITS{1'b0}};
				state        <= S_DRAIN;
			end
		end
	end

`ifdef ZG_ACCEPT_WATCH
	// 丢受理检测: 核心 D 口请求高电平期间若从未出现 aph_ready 脉冲而
	// 请求撤销(核心退休该访存), 即为一次被丢弃的事务 —— 立即打印快照。
	reg aw_seen;
	reg aw_done;
	always @(posedge clk) begin
		if (!rst_n) begin
			aw_seen <= 1'b0; aw_done <= 1'b0;
		end else begin
			if (!aw_seen && core_aph_req && !core_aph_ready) begin
				aw_seen <= 1'b1;
				aw_done <= 1'b0;
			end else if (aw_seen && core_aph_ready) begin
				aw_seen <= 1'b0;
				aw_done <= core_hwrite;
			end else if (aw_seen && !core_aph_req) begin
				if (!aw_done)
					$display("[ADROP] state=%0d dph=%b armed=%b byp=%b hready=%b excl=%b",
						state, dph_pending, hit_dph_armed, byp_pending, m_hready, core_aph_excl);
				aw_seen <= 1'b0;
			end
		end
	end
`endif

	// ------------------------------------------------------------------
	// 核心侧输出
	// ------------------------------------------------------------------
	wire fill_critical_now = (state == S_FILL) && dph_valid && m_hready && !fill_fwd_kill &&
		!fill_is_store &&
		(WRAP_FILL ? (dph_beat == {BEAT_BITS{1'b0}}) : (dph_beat == fill_off));

	assign core_dph_ready =
		(dph_pending && hit_dph_armed)                                   // 命中
		|| (state == S_EXCL_DPH && excl_aph_done && m_hready)            // 独占
		|| (state == S_BYP_DPH && byp_aph_done && m_hready)              // 旁路
		|| fill_critical_now;                                            // load 缺失关键字 beat

`ifdef ZG_LOG_DL
	reg [31:0] zg_ld_addr;
	reg        zg_ld_vld;
	reg [2:0]  zg_ld_size;
	always @ (posedge clk) begin
		zg_ld_vld <= 1'b0;
		if (core_aph_req && core_aph_ready) begin
			zg_ld_addr <= core_haddr;
			zg_ld_size <= core_hsize;
			zg_ld_vld  <= !core_hwrite;
		end
		if (core_dph_ready && zg_ld_vld)
			$display("[LDL %08x %08x %01x]", zg_ld_addr, core_rdata, zg_ld_size);
		if (core_aph_req && core_aph_ready && core_hwrite)
			$display("[STL %08x %08x %01x]", core_haddr, core_wdata, core_hsize);
	end
`endif
`ifdef B2_DEBUG
	reg [31:0] dbg_acc_addr;
	reg        dbg_acc_vld;
	always @ (posedge clk) begin
		dbg_acc_vld <= 1'b0;
		if (core_aph_ready && core_aph_req) begin
			dbg_acc_addr <= core_haddr;
			dbg_acc_vld  <= 1'b1;
			$display("[DACC] a=%08x wr=%0d hit=%b st=%0d dphp=%b arm=%b",
				core_haddr, core_hwrite, req_hit, state, dph_pending, hit_dph_armed);
		end
		if (core_dph_ready)
			$display("[DDLV] a=%08x rd=%08x arr=%08x bus=%08x dphp=%b arm=%b",
				dbg_acc_addr, core_rdata, rd_rdata, m_hrdata, dph_pending, hit_dph_armed);
	end
`endif

	// ZIGUANG: 错误上报不门控 hready —— 上游 wrapper 语义为两相错误的第一相
	// (hready=0, hresp=1) 即上报, 供核心提前锁存; 若等 hready=1 才上报,
	// 核心在第一相已按 err=0 采样, 第二相的错误会被整体错过 (吞异常)。
	assign core_dph_err =
		(state == S_EXCL_DPH && excl_aph_done && m_hresp)
		|| (state == S_BYP_DPH && byp_aph_done && m_hresp)
		|| (state == S_ERR_DPH)
		|| (fill_critical_now && m_hresp && !fill_no_dph);

	assign core_dph_exokay =
		(state == S_EXCL_DPH && excl_aph_done && m_hready && m_hexokay);

	assign core_rdata =
		(dph_pending && hit_dph_armed) ? rd_rdata : m_hrdata;

	// 关键字 beat 一旦报错即停止前递
	always @ (posedge clk or negedge rst_n) begin
		if (!rst_n)
			fill_fwd_kill <= 1'b0;
		else if (fill_critical_now && m_hresp)
			fill_fwd_kill <= 1'b1;
	end

	// ------------------------------------------------------------------
	// 阵列读：命中 load + 受害/排空提取
	// ZGUANG-BRAM: 两读源互斥 (hit_accept 受理 vs S_WB_EXTRACT 状态),
	// 归并为每路单读口 (1R), 配合下方单写口 (1W) 使 8 个 bank 阵列
	// 满足 SDP 结构, PDS 可推断 DRM (原 2R+3W 结构触发 Sdm-2010 多写告警,
	// bank 被展开成 LUTRAM)。
	// ------------------------------------------------------------------
	wire [AWR_BITS-1:0] hit_ridx = {req_idx, req_off};
	wire [AWR_BITS-1:0] wb_ridx  = {wb_idx, wb_ext_beat};

	wire hit_read = hit_accept;
	wire wb_read  = (state == S_WB_EXTRACT);

	wire                way0_re  = (hit_read &&  hit0) || (wb_read && !wb_way);
	wire                way1_re  = (hit_read && !hit0) || (wb_read &&  wb_way);
	wire [AWR_BITS-1:0] way_raddr = hit_read ? hit_ridx : wb_ridx;

	wire [7:0] way0_b0_r = way0_b0[way_raddr];
	wire [7:0] way0_b1_r = way0_b1[way_raddr];
	wire [7:0] way0_b2_r = way0_b2[way_raddr];
	wire [7:0] way0_b3_r = way0_b3[way_raddr];
	wire [7:0] way1_b0_r = way1_b0[way_raddr];
	wire [7:0] way1_b1_r = way1_b1[way_raddr];
	wire [7:0] way1_b2_r = way1_b2[way_raddr];
	wire [7:0] way1_b3_r = way1_b3[way_raddr];
	wire [31:0] way0_rdata = {way0_b3_r, way0_b2_r, way0_b1_r, way0_b0_r};
	wire [31:0] way1_rdata = {way1_b3_r, way1_b2_r, way1_b1_r, way1_b0_r};

	always @ (posedge clk) begin
		if (hit_read)
			rd_rdata <= hit0 ? way0_rdata : way1_rdata;
		// 受害行提取：在与读相同的块中把阵列字捕获进缓冲，
		// 使缓冲获得本拍内容（中间寄存器会引入一拍偏移）。
		if (wb_read)
			wb_buf[wb_ext_beat] <= wb_way ? way1_rdata : way0_rdata;
	end

	// ------------------------------------------------------------------
	// 阵列写: 命中 store（数据相采样）、重填充 beat、store-miss 补写
	// ------------------------------------------------------------------
	// AHB 契约: hwdata 在数据相（接受后一拍）才有效。命中 store 的阵列
	// 写入必须在数据相拍采样 core_wdata——在接受拍采样会读到上一条 M 级
	// 指令的结果（背靠背访存流中的典型错值, 曾导致栈上 ra 被写坏）。
	// 背靠背情形: 命中 N 的数据相拍 = 命中 N+1 的接受拍, 阵列写用本拍
	// 标志值组合求值, 标志在沿上被 N+1 重新锁存, 两者无冲突。
	// ZIGUANG (B2' WT+WA): store 命中 = 旁路直通写 + 同步写行。
	// store 一律经 bypass_accept 上总线 (写直通), 行更新挂在 bypass 受理拍
	// 锁存 (way/idx/lane), 在该 store 的总线数据相完成拍写行。
	// hit_store_write 原定义 (hit_accept && core_hwrite) 恒为 0 -- hit_accept
	// 含 !core_hwrite -- 行更新死代码 + 行失效已删 => load hit 交付陈旧行
	// (hellow 栈 ra 读回 0, ret 跳 0 trap, 2008 拍 exit -1)。
	wire hit_store_write = bypass_accept && core_hwrite && addr_cacheable && req_hit;

	reg                hit_wr_pending;
	reg                hit_wr_way0;
	reg [AWR_BITS-1:0] hit_wr_widx;
	reg [3:0]          hit_wr_lane;

	// Apply 拍 = 该 store 的 bypass DPH 完成拍: 与总线写 (m_hwdata 活采)
	// 同拍同源, core_wdata 此刻仍为本 store 数据 (该拍 dph_ready 同拍退休)。
	// 总线错误响应拍不写行 (DC-03: store fault 不得修改 cache line)。
	wire hit_store_apply = hit_wr_pending && (state == S_BYP_DPH) && byp_aph_done
		&& m_hready && !m_hresp;

	always @ (posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			hit_wr_pending <= 1'b0;
			hit_wr_way0    <= 1'b0;
			hit_wr_widx    <= {AWR_BITS{1'b0}};
			hit_wr_lane    <= 4'h0;
		end else begin
			// ZIGUANG (板级等待拍修复): 旁路写有从机等待拍时 (hready 在
			// 受理拍之后拉低), hit_wr_pending 不能按拍无条件跟随
			// hit_store_write -- 否则等待拍清 0, apply 拍 (hready 恢复) 行写
			// 丢失, load hit 永远读旧行 (板级自旋标志读回旧值死循环)。
			// 规则: 新 store 受理即置位; 否则 apply 拍清零; 等待拍保持。
			if (hit_store_write)
				hit_wr_pending <= 1'b1;
			else if (hit_store_apply)
				hit_wr_pending <= 1'b0;
			if (hit_store_write) begin
				hit_wr_way0 <= hit0;
				hit_wr_widx <= {req_idx, req_off};
				hit_wr_lane <= st_lane_mask;
			end
		end
	end

	wire [AWR_BITS-1:0] hit_widx = hit_wr_widx;

	// 重填 beat 写：以总线上数据 beat 为索引
	wire fill_beat_write = (state == S_FILL) && dph_valid && m_hready;
	wire [OFF_BITS-1:0] wr_line_off = WRAP_FILL ? (fill_off + dph_beat) : dph_beat;
	wire [AWR_BITS-1:0] fill_widx = {fill_idx, wr_line_off};

	wire store_apply_write = (state == S_STORE_APPLY);
	wire [AWR_BITS-1:0] apply_widx = {fill_idx, fill_off};

	// ZGUANG-BRAM: 三个写源按 FSM 状态互斥 (S_BYP_DPH / S_FILL / S_STORE_APPLY),
	// 归并为每路单写口。命中 store 只写命中 lane (字节使能), fill/store-apply
	// 写全 4 lane。写地址/数据/字节使能三选一与原逐 if 写语义一致。
	wire way0_we = (hit_store_apply &&  hit_wr_way0)
	             || (fill_beat_write  && !fill_way)
	             || (store_apply_write && !fill_way);
	wire way1_we = (hit_store_apply && !hit_wr_way0)
	             || (fill_beat_write  &&  fill_way)
	             || (store_apply_write &&  fill_way);
	// verilator lint_off WIDTH
	wire [AWR_BITS-1:0] way_waddr = hit_store_apply ? hit_widx     :
	                                fill_beat_write  ? fill_widx   :
	                                                   apply_widx;
	// verilator lint_on WIDTH
	wire [31:0] way_wdata = hit_store_apply ? core_wdata  :
	                        fill_beat_write  ? m_hrdata    :
	                                           core_wdata;
	wire [3:0]  way0_be = hit_store_apply ? hit_wr_lane : 4'b1111;
	wire [3:0]  way1_be = hit_store_apply ? hit_wr_lane : 4'b1111;

	always @ (posedge clk) begin
`ifdef ZG_TRACE_FILL
		if (fill_beat_write)
			$display("[FWR] slot={%0d,%0d} data=%08x", fill_idx, wr_line_off, m_hrdata);
`endif
		if (way0_we && way0_be[0]) way0_b0[way_waddr] <= way_wdata[7:0];
		if (way0_we && way0_be[1]) way0_b1[way_waddr] <= way_wdata[15:8];
		if (way0_we && way0_be[2]) way0_b2[way_waddr] <= way_wdata[23:16];
		if (way0_we && way0_be[3]) way0_b3[way_waddr] <= way_wdata[31:24];
		if (way1_we && way1_be[0]) way1_b0[way_waddr] <= way_wdata[7:0];
		if (way1_we && way1_be[1]) way1_b1[way_waddr] <= way_wdata[15:8];
		if (way1_we && way1_be[2]) way1_b2[way_waddr] <= way_wdata[23:16];
		if (way1_we && way1_be[3]) way1_b3[way_waddr] <= way_wdata[31:24];
	end

	// ------------------------------------------------------------------
	// 下游 AHB5 master
	// ------------------------------------------------------------------
	wire in_excl = (state == S_EXCL_APH) || (state == S_EXCL_DPH);
	wire in_byp  = (state == S_BYP_DPH);
	wire in_wb   = (state == S_WB_WRITE);
	wire in_fill = (state == S_FILL);

	// 本周期呈现地址相的 beat 的地址
	wire [OFF_BITS-1:0] fill_present_off = WRAP_FILL ? (fill_off + aph_beat) : aph_beat;
	wire [W_ADDR-1:0] fill_addr = {fill_tag, fill_idx, fill_present_off, 2'b00};
	wire [W_ADDR-1:0] wb_addr   = {wb_tag, wb_idx, aph_beat, 2'b00};

	assign m_haddr =
		in_wb   ? wb_addr :
		in_fill ? fill_addr :
		in_excl ? excl_addr :
		          core_haddr;

	// 旁路接受拍直接把核心地址相呈上总线（上游直通形状）
	// 复位期间强制 IDLE: 未复位的内部状态不得驱动总线 (4 态仿真 X 防护)
	assign m_htrans = !rst_n ? HTRANS_IDLE :
		(in_excl && state == S_EXCL_APH)          ? HTRANS_NSEQ :
		bypass_accept                             ? HTRANS_NSEQ :
		(in_wb  && !aph_done)                     ? (aph_beat != 0 ? HTRANS_SEQ : HTRANS_NSEQ) :
		(in_fill && !aph_done)                    ? (aph_beat != {BEAT_BITS{1'b0}} ? HTRANS_SEQ : HTRANS_NSEQ) :
		                                            HTRANS_IDLE;

	assign m_hburst =
		in_wb   ? HBURST_INCR8 :
		in_fill ? HBURST_FILL :
		          HBURST_SINGLE;

	// ZIGUANG: 尺寸按事务驱动 - 旁路子字 store 必须以真实 hsize 上总线
	// (tb 字节使能由 hsize+haddr[1:0] 解码, 硬编码 word 会覆盖同字相邻字节)
	assign m_hsize = bypass_accept ? core_hsize :
		(in_byp && byp_is_write) ? byp_hsize : 3'd2;
	assign m_hwrite    = in_wb || (in_byp && byp_is_write) || (bypass_accept && core_hwrite);
	assign m_hmastlock = 1'b0;
	assign m_hexcl     = in_excl;
	// ZIGUANG: 旁路写活采 core_wdata 是正确语义 ——
//  a) tb 在数据相拍 (受理后一拍) 采样 hwdata, 此时该 store 仍在 M 级
//     (dph_ready 同拍 retirement), wdata 保持有效;
//  b) 受理拍锁存 (byp_wdata) 反而抓到上一条指令的数据 (实测 hellow
//     1753 拍无退出回归, PROGRESS-07 §五)。
// 历史注释 "活采会写下一条指令的数据" 不成立, 已移除。
assign m_hwdata    = in_wb ? wb_buf[dph_beat] : core_wdata;

	// ------------------------------------------------------------------
	// 维护状态
	// ------------------------------------------------------------------
	// ZIGUANG (A2/fence.d): fence.d 现在执行完全失效扫描
	// （S_DRAIN 清除 valid 位），因此完成必须如实上报——核心的
	// fence_rdy 等待扫描结束，而扫描的开始会在 M 级还有在途请求时
	// 被推迟（见 drain_latch 启动条件），这正是从前 fence 死锁的原因
	// （PROGRESS-07）。drain_done 如实上报后，fence 只有在扫描真正
	// 结束后才会退休。
	assign drain_done = !drain_latch && !drain_active;

	// ------------------------------------------------------------------
	// 统计
	// ------------------------------------------------------------------
	assign evt_req       = core_aph_ready && core_aph_req;
	assign evt_hit       = evt_req && !core_aph_excl && addr_cacheable && req_hit;
	assign evt_miss      = evt_req && !core_aph_excl && addr_cacheable && !req_hit;
	assign evt_writeback = in_wb && dph_valid && m_hready && (dph_beat == BEAT_LAST);
	assign evt_bypass    = evt_req && (!addr_cacheable || core_aph_excl);
	assign evt_err       = core_dph_err;

	reg [31:0] cnt_req, cnt_hit, cnt_miss, cnt_wb, cnt_byp, cnt_err;
	always @ (posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			cnt_req <= 32'd0; cnt_hit <= 32'd0; cnt_miss <= 32'd0;
			cnt_wb  <= 32'd0; cnt_byp  <= 32'd0; cnt_err  <= 32'd0;
		end else begin
			if (evt_req)       cnt_req  <= cnt_req  + 32'd1;
			if (evt_hit)       cnt_hit  <= cnt_hit  + 32'd1;
			if (evt_miss)      cnt_miss <= cnt_miss + 32'd1;
			if (evt_writeback) cnt_wb   <= cnt_wb   + 32'd1;
			if (evt_bypass)    cnt_byp  <= cnt_byp  + 32'd1;
			if (evt_err)       cnt_err  <= cnt_err  + 32'd1;
		end
	end

	wire [31:0] stats [0:5];
	assign stats[0] = cnt_req;
	assign stats[1] = cnt_hit;
	assign stats[2] = cnt_miss;
	assign stats[3] = cnt_wb;
	assign stats[4] = cnt_byp;
	assign stats[5] = cnt_err;

endmodule

`default_nettype wire
