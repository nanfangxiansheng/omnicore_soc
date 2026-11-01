/*****************************************************************************\


/*****************************************************************************\
|  ominicore_icache.v — OmniCore 2 路组相联指令 cache                           |
|                                                                             |
|  紫光同创赛题 高阶任务①: 存储系统优化（组相联 Cache + AHB5 突发传输优化）           |
|                                                                             |
|  位于处理器的简单取指接口（一次只允许一个在途取指，                             |
|  即上游 cpu_1port/cpu_2port 封装所产生的形态）                                 |
|  与通往 SoC 互连的完整 AHB5 master 端口之间。                                 |
|                                                                             |

\*****************************************************************************/

`default_nettype none

module ominicore_icache #(
	parameter W_ADDR           = 32,
	parameter W_DATA           = 32,
	parameter NUM_SETS         = 256,            // 2 的幂。
	parameter LINE_WORDS       = 4,              // ZIGUANG (B3/CACHE-01): 每行字数
	parameter CACHE_BASE       = 32'h00000000,   // 最低可缓存地址
	parameter CACHE_LIMIT      = 32'h40000000,   // 最高可缓存地址加一
	parameter WRAP_FILL        = 1               // 1: WRAP4 关键字优先, 0: 从行基址起 INCR4
) (
	input  wire                clk,
	input  wire                rst_n,

	// 处理器侧取指接口
	input  wire                core_aph_req,
	input  wire [W_ADDR-1:0]   core_haddr,
	output wire                core_aph_ready,
	output wire                core_dph_ready,
	output wire                core_dph_err,
	output wire [W_DATA-1:0]   core_rdata,

	// cache 维护
	input  wire                flush_req,        // 脉冲：使所有行失效

	// 统计事件脉冲（送往 ominicore_perf）
	output wire                evt_req,
	output wire                evt_hit,
	output wire                evt_miss,
	output wire                evt_err,
	// ZIGUANG (PMU-01/mbuswait): 下游总线数据相等待拍 (突发 beat 或旁路
	// 数据相在途且 hready 未到)。唯一计数点, 与 mstall 事件互斥定义见
	// PROGRESS-06 PMU 文档
	output wire                evt_buswait,

	// 下游 AHB5 master 端口
	output wire [W_ADDR-1:0]   m_haddr,
	output wire [1:0]          m_htrans,
	output wire [2:0]          m_hsize,
	output wire [2:0]          m_hburst,
	output wire                m_hwrite,
	output wire                m_hmastlock,
	output wire [W_DATA-1:0]   m_hwdata,
	input  wire                m_hready,
	input  wire                m_hresp,
	input  wire [W_DATA-1:0]   m_hrdata
);

	function integer clog2;
		input integer value;
		begin
			for (clog2 = 0; value > 1; clog2 = clog2 + 1)
				value = value >> 1;
		end
	endfunction

	wire hit_accept;          // (2026-10-09 前移: vlog 严格先声明后使用)
	wire fill_critical_now;
	localparam OFF_BITS   = clog2(LINE_WORDS);
	localparam BEAT_BITS  = clog2(LINE_WORDS);                        // 每行字数
	localparam IDX_BITS   = clog2(NUM_SETS);
	localparam TAG_BITS   = W_ADDR - OFF_BITS - 2 - IDX_BITS; // 字 + 字节偏移
	localparam ARR_BITS   = OFF_BITS + IDX_BITS;      // 阵列索引: {组, 行内字}

	localparam HTRANS_IDLE = 2'b00;
	localparam HTRANS_NSEQ = 2'b10;
	localparam HTRANS_SEQ  = 2'b11;
	localparam HBURST_SINGLE = 3'b000;
	localparam HBURST_WRAP4  = 3'b010;
	localparam HBURST_INCR4  = 3'b011;
	localparam HBURST_INCR8  = 3'b111;   // ZIGUANG (B3)
	localparam HBURST_FILL   = (LINE_WORDS == 8) ? HBURST_INCR8 :
	                           (WRAP_FILL       ? HBURST_WRAP4  : HBURST_INCR4);
	// LINE_WORDS 为 2 的幂 (4/8): LINE_WORDS-1 即 BEAT_BITS 位全 1
	localparam [BEAT_BITS-1:0] BEAT_LAST = {BEAT_BITS{1'b1}};
	localparam [BEAT_BITS-1:0] BEAT_ONE  = {{(BEAT_BITS-1){1'b0}}, 1'b1};

	// ------------------------------------------------------------------
	// 存储：数据阵列（同步读以便推断 BRAM），tag 用触发器
	// ------------------------------------------------------------------
	reg [W_DATA-1:0] way0_data [0:NUM_SETS*LINE_WORDS-1];
	reg [W_DATA-1:0] way1_data [0:NUM_SETS*LINE_WORDS-1];

	reg [TAG_BITS-1:0] tag0 [0:NUM_SETS-1];
	reg [TAG_BITS-1:0] tag1 [0:NUM_SETS-1];
	reg [NUM_SETS-1:0] valid0, valid1;

	integer i;

	// ------------------------------------------------------------------
	// 请求译码（组合逻辑，基于当前核心地址）
	// ------------------------------------------------------------------
	// verilator lint_off UNSIGNED
	wire addr_cacheable = (core_haddr >= CACHE_BASE) && (core_haddr < CACHE_LIMIT);
	// verilator lint_on UNSIGNED

	wire [IDX_BITS-1:0] req_idx = core_haddr[OFF_BITS + 2 +: IDX_BITS];
	wire [TAG_BITS-1:0] req_tag = core_haddr[OFF_BITS + 2 + IDX_BITS +: TAG_BITS];
	wire [OFF_BITS-1:0] req_off = core_haddr[OFF_BITS+1:2];   // ZIGUANG (B3): 行内字序号 (随 LINE_WORDS)

	wire hit0 = valid0[req_idx] && (tag0[req_idx] == req_tag);
	wire hit1 = valid1[req_idx] && (tag1[req_idx] == req_tag);
	wire req_hit = hit0 || hit1;

	// ZIGUANG (B4, CACHE-05): 真 LRU —— 每组 1 bit (2-way 精确 LRU)。
	// lru_way[i]=0: way0 最近使用 (下次逐出 way1); =1: way1 最近使用。
	reg [NUM_SETS-1:0] lru_way;
	always @ (posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			lru_way <= {NUM_SETS{1'b0}};
		end else begin
			if (hit_accept && hit0)
				lru_way[req_idx] <= 1'b0;
			else if (hit_accept && hit1)
				lru_way[req_idx] <= 1'b1;
		end
	end

	// ------------------------------------------------------------------
	// 在途事务跟踪
	// ------------------------------------------------------------------
	reg dph_pending;         // 已受理的取指等待其数据相
	reg hit_dph_armed;       // 是命中：数据下一拍有效
	reg [W_DATA-1:0] rd_rdata;

	// 填充引擎
	reg       miss_in_progress;
	reg       fill_fwd_kill; // 关键字 beat 已送达（带错误）
	reg       fill_err;      // 本次填充期间出现过 hresp 错误
	reg       fill_flushed;  // 填充期间 flush 到来：不提交 tag
	reg [BEAT_BITS-1:0] aph_beat;      // 本周期呈现地址的 beat
	reg [BEAT_BITS-1:0] dph_beat;      // 本周期总线上数据的 beat
	reg       dph_valid;     // 本周期 m_hrdata 属于 beat dph_beat
	reg       aph_done;      // 全部 4 个地址相均已受理
	reg [OFF_BITS-1:0] fill_off;
	reg       fill_way;
	reg [IDX_BITS-1:0] fill_idx;
	reg [TAG_BITS-1:0] fill_tag;

	// 旁路引擎
	reg       bypass_in_progress;
	reg       bypass_aph_done;
	reg [W_ADDR-1:0] byp_addr;   // 锁存：数据相期间取指地址已前进

	// ------------------------------------------------------------------
	// 受理逻辑（组合逻辑）
	// ------------------------------------------------------------------
	wire hit_dph_deliver_now = dph_pending && hit_dph_armed;

	assign hit_accept = core_aph_req && addr_cacheable && req_hit
		&& !miss_in_progress && !bypass_in_progress
		&& (!dph_pending || hit_dph_deliver_now);

	wire miss_accept = core_aph_req && addr_cacheable && !req_hit
		&& !miss_in_progress && !bypass_in_progress
		&& (!dph_pending || hit_dph_deliver_now);

	wire bypass_accept = core_aph_req && !addr_cacheable
		&& !miss_in_progress && !bypass_in_progress && !dph_pending;

	assign core_aph_ready = hit_accept || miss_accept || bypass_accept;

	// ------------------------------------------------------------------
	// 控制
	// ------------------------------------------------------------------
	localparam S_IDLE      = 3'd0;
	localparam S_BYP_APH   = 3'd1;
	localparam S_BYP_DPH   = 3'd2;
	localparam S_FILL_APH  = 3'd3;
	localparam S_FILL_BEAT = 3'd4;

	reg [2:0] state;

	always @ (posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state              <= S_IDLE;
			dph_pending        <= 1'b0;
			hit_dph_armed      <= 1'b0;
			miss_in_progress   <= 1'b0;
			bypass_in_progress <= 1'b0;
			bypass_aph_done    <= 1'b0;
			fill_err           <= 1'b0;
			fill_flushed       <= 1'b0;
			aph_beat <= {BEAT_BITS{1'b0}};
			dph_beat <= {BEAT_BITS{1'b0}};
			dph_valid          <= 1'b0;
			aph_done           <= 1'b0;
			fill_off           <= {OFF_BITS{1'b0}};
			fill_way           <= 1'b0;
			fill_idx           <= {IDX_BITS{1'b0}};
			fill_tag           <= {TAG_BITS{1'b0}};
			valid0             <= {NUM_SETS{1'b0}};
			valid1             <= {NUM_SETS{1'b0}};
		end else begin
			// fence.i / flush：丢弃所有行。在途填充不提交
			// 其 tag（其数据可能早于 fence）。
			if (flush_req) begin
				valid0 <= {NUM_SETS{1'b0}};
				valid1 <= {NUM_SETS{1'b0}};
				if (miss_in_progress)
					fill_flushed <= 1'b1;
			end

			// --- 地址相受理 ---
			if (hit_accept) begin
				dph_pending   <= 1'b1;
				hit_dph_armed <= 1'b1;
			end else if (miss_accept) begin
				dph_pending      <= 1'b1;
				hit_dph_armed    <= 1'b0;
				miss_in_progress <= 1'b1;
				fill_err         <= 1'b0;
				fill_flushed     <= 1'b0;
				aph_beat <= {BEAT_BITS{1'b0}};
				dph_beat <= {BEAT_BITS{1'b0}};
				dph_valid        <= 1'b0;
				aph_done         <= 1'b0;
				fill_off         <= req_off;
				fill_idx         <= req_idx;
				fill_tag         <= req_tag;
				// ZIGUANG (B4): LRU 逐出 (invalid 优先)
				fill_way         <= (valid0[req_idx] && !valid1[req_idx]) ? 1'b1 :
				                    (valid1[req_idx] && !valid0[req_idx]) ? 1'b0 :
				                    lru_way[req_idx];
				state            <= S_FILL_APH;
			end else if (bypass_accept) begin
				dph_pending        <= 1'b1;
				hit_dph_armed      <= 1'b0;
				bypass_in_progress <= 1'b1;
				bypass_aph_done    <= 1'b0;
				byp_addr           <= core_haddr;
			end

			// --- 命中数据相在受理后一拍送达 ---
			// 清除必须让位于同周期的再次受理（背靠背流中
			// 它会为下一次命中重新置起 pending）。
			if (hit_dph_armed && !hit_accept && !miss_accept && !bypass_accept) begin
				dph_pending   <= 1'b0;
				hit_dph_armed <= 1'b0;
			end

			// --- 旁路引擎 ---
			if (bypass_in_progress) begin
				if (!bypass_aph_done) begin
					if (m_hready)
						bypass_aph_done <= 1'b1;
				end else if (m_hready) begin
					bypass_in_progress <= 1'b0;
					dph_pending        <= 1'b0;
				end
			end

			// --- 填充引擎 ---
			case (state)
				S_FILL_APH: begin
					// 呈现 beat 0（NSEQ）。被受理后其数据相
					// 在下一拍运行。
					if (m_hready) begin
						dph_valid <= 1'b1;
						dph_beat <= {BEAT_BITS{1'b0}};
						aph_beat <= BEAT_ONE;
						state     <= S_FILL_BEAT;
					end
				end

				S_FILL_BEAT: begin
					// 消费本周期总线上的数据相
					if (dph_valid && m_hready) begin
						if (m_hresp)
							fill_err <= 1'b1;
						if (fill_critical_now)
							dph_pending <= 1'b0;
						if (dph_beat == BEAT_LAST) begin
							// 最后一个数据相完成：提交行 tag
							if (!fill_err && !m_hresp && !fill_flushed) begin   // B3: 末拍 hresp 同拍可见 (INCR8 毒词末拍吞错修复)
								if (fill_way) begin
									tag1[fill_idx]  <= fill_tag;
									valid1[fill_idx]<= 1'b1;
								end else begin
									tag0[fill_idx]  <= fill_tag;
									valid0[fill_idx]<= 1'b1;
								end
							end
							miss_in_progress <= 1'b0;
							dph_valid        <= 1'b0;
							state            <= S_IDLE;
						end else begin
							dph_valid <= 1'b1;
							dph_beat <= dph_beat + 1'b1;
						end
					end
					// 推进地址相（共 4 beat）
					if (m_hready && !aph_done) begin
						if (aph_beat == BEAT_LAST)
							aph_done <= 1'b1;
						else
							aph_beat <= aph_beat + 1'b1;
					end
				end

				default: ;
			endcase
		end
	end

	// 关键 beat：缺失字的数据相
	wire hit_dph_now = dph_pending && hit_dph_armed;
	wire fill_data_beat_now = miss_in_progress && dph_valid && m_hready;
	assign fill_critical_now = fill_data_beat_now && !fill_fwd_kill &&
		(WRAP_FILL ? (dph_beat == {BEAT_BITS{1'b0}}) : (dph_beat == fill_off));

	// ------------------------------------------------------------------
	// 核心侧输出
	// ------------------------------------------------------------------
	assign core_dph_ready =
		hit_dph_now
		|| (bypass_in_progress && bypass_aph_done && m_hready)
		|| fill_critical_now;

	assign core_dph_err =
		(bypass_in_progress && bypass_aph_done && m_hready && m_hresp)
		|| (fill_critical_now && m_hresp);

	assign core_rdata =
		hit_dph_now ? rd_rdata : m_hrdata;  // 命中用寄存器，否则用总线

	// fill_fwd_kill：关键字 beat 一旦报错即停止前递
	// （单一用途块；dph_pending 在主控制块中清除）
	// 注意：本块是 fill_fwd_kill 的唯一驱动源（ADS Verilog-4086
	// 禁止两个 always 块驱动同一变量）。复位与新 fill 受理的清零
	// 都由本块覆盖：miss_accept 隐含 !miss_in_progress（旧值），
	// 故受理边沿下方 !miss_in_progress 分支必然清零。
	always @ (posedge clk or negedge rst_n) begin
		if (!rst_n)
			fill_fwd_kill <= 1'b0;
		else begin
			if (fill_critical_now && m_hresp)
				fill_fwd_kill <= 1'b1;
			if (!miss_in_progress)
				fill_fwd_kill <= 1'b0;
		end
	end

	// ------------------------------------------------------------------
	// 数据阵列
	// ------------------------------------------------------------------
	// 读端口：命中受理时启动，下一拍消费数据。
	always @ (posedge clk) begin
		if (hit_accept)
			rd_rdata <= hit0 ? way0_data[{req_idx, req_off}]
			                 : way1_data[{req_idx, req_off}];
	end

	// 写端口：重填 beat，以总线上数据相 beat 为索引。
	// WRAP4 时突发按偏移序 fill_off, fill_off+1, ...（模 4）交付行字，
	// INCR4 时为 0,1,2,3；阵列保持自然行序，因此写索引做补偿。
	wire fill_beat_write = miss_in_progress && dph_valid && m_hready;
	wire [OFF_BITS-1:0] wr_line_off = WRAP_FILL ? (fill_off + dph_beat) : dph_beat;
	wire [ARR_BITS-1:0] wr_idx = {fill_idx, wr_line_off};

	always @ (posedge clk) begin
		if (fill_beat_write) begin
			if (fill_way)
				way1_data[wr_idx] <= m_hrdata;
			else
				way0_data[wr_idx] <= m_hrdata;
		end
	end

	// ------------------------------------------------------------------
	// 下游 AHB5 master
	// ------------------------------------------------------------------
	wire in_fill = (state == S_FILL_APH) || (state == S_FILL_BEAT);

	// 本周期呈现地址相的 beat 的地址
	// ZIGUANG (B3): 位段宽 OFF_BITS -- 32B 行下 beat 字段 3 位, 拼接总宽必须
	// 保持 32 位, 否则 tag 整体右移一位 (bit31 丢失, fill 地址错).
	wire [OFF_BITS-1:0] present_beat = (state == S_FILL_APH) ? {OFF_BITS{1'b0}} : aph_beat;
	wire [W_ADDR-1:0] fill_beat_addr =
		WRAP_FILL ? {fill_tag, fill_idx, fill_off + present_beat, 2'b00}
		          : {fill_tag, fill_idx, present_beat, 2'b00};

	assign m_haddr =
		bypass_in_progress ? byp_addr :
		in_fill            ? fill_beat_addr
		                   : core_haddr;

	// 复位期间强制 IDLE: 未复位的内部状态不得驱动总线 (4 态仿真 X 防护)
	assign m_htrans = !rst_n ? HTRANS_IDLE :
		bypass_in_progress && !bypass_aph_done ? HTRANS_NSEQ :
		bypass_in_progress                     ? HTRANS_IDLE :
		state == S_FILL_APH                    ? HTRANS_NSEQ :
		(state == S_FILL_BEAT && !aph_done)    ? HTRANS_SEQ :
		                                         HTRANS_IDLE;

	assign m_hsize     = 3'd2;   // 取指总是字大小、字对齐
	assign m_hburst    = in_fill ? HBURST_FILL
	                          : HBURST_SINGLE;
	assign m_hwrite    = 1'b0;
	assign m_hmastlock = 1'b0;
	assign m_hwdata    = {W_DATA{1'b0}};

	// ------------------------------------------------------------------
	// 统计
	// ------------------------------------------------------------------
	assign evt_req  = core_aph_ready && core_aph_req && addr_cacheable; // ZIGUANG (CACHE-06): bypass 不计入查询
	assign evt_hit  = evt_req && addr_cacheable && req_hit;
	assign evt_miss = evt_req && addr_cacheable && !req_hit;
	assign evt_err  = core_dph_err;

	// ZIGUANG (PMU-01): 数据相在途且当拍 hready 未到 = 下游等待一拍
	assign evt_buswait = !m_hready && (
		(miss_in_progress && dph_valid) ||
		(bypass_in_progress && bypass_aph_done)
	);

	// ZIGUANG (PMU-01): 数据相在途且当拍 hready 未到 = 下游等待一拍
		reg [31:0] cnt_req, cnt_hit, cnt_miss, cnt_err;
	always @ (posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			cnt_req  <= 32'd0;
			cnt_hit  <= 32'd0;
			cnt_miss <= 32'd0;
			cnt_err  <= 32'd0;
		end else begin
			if (evt_req)  cnt_req  <= cnt_req  + 32'd1;
			if (evt_hit)  cnt_hit  <= cnt_hit  + 32'd1;
			if (evt_miss) cnt_miss <= cnt_miss + 32'd1;
			if (evt_err)  cnt_err  <= cnt_err  + 32'd1;
		end
	end

	wire [31:0] stats [0:3];
	assign stats[0] = cnt_req;
	assign stats[1] = cnt_hit;
	assign stats[2] = cnt_miss;
	assign stats[3] = cnt_err;

endmodule

`default_nettype wire
