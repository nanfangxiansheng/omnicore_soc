// ============================================================================
// ominicore_dvp_rx.v — OV5640 DVP 采集 → 帧缓冲 AHB 写主机 (APP-PROFILE S2)
// ----------------------------------------------------------------------------
// 结构:
//   cam_pclk 域 : VSYNC 帧界 + HREF 选通字节流 → 字节配对 RGB565 → 两像素拼
//                 32bit 字 → 异步 FIFO 写侧; 上电跳帧(图像稳定)
//   clk 域      : FIFO 读侧 → 4 字 skid → AHB 主机 INCR4/SINGLE 突发写帧缓冲
//                 写指针计数到帧尾 → frame_done_sticky IRQ + 乒乓缓冲切换
//   从机口      : 5 个字寄存器 (CTRL/BASE0/BASE1/STATUS/IRQ_W1C)
// ============================================================================
`default_nettype none

module ominicore_dvp_rx #(
	parameter H_PIXELS    = 640,     // 每行像素(偶数)
	parameter V_LINES     = 480,     // 每帧行数
	parameter FIFO_DEPTH  = 1024,    // 异步 FIFO 深度(32bit 字, 2 的幂)
	parameter SKIP_FRAMES = 3        // 上电跳帧数(等图像稳定)
) (
	input  wire        clk,            // 系统钟(AHB)
	input  wire        rst_n,          // 系统复位(低有效)
	// ---- OV5640 DVP (cam_pclk 域) ----
	input  wire        cam_pclk,
	input  wire        cam_vsync,      // 帧消隐指示(极性由 CTRL[1] 选)
	input  wire        cam_href,       // 行选通(高=字节有效)
	input  wire [7:0]  cam_data,       // 8bit 字节流(每像素 2 字节, 高字节在前)
	// ---- AHB 主机 (帧缓冲写, clk 域) ----
	output reg  [31:0] m_haddr,
	output wire        m_hwrite,
	output wire [2:0]  m_hsize,        // 恒 WORD
	output wire [2:0]  m_hburst,       // SINGLE / INCR4
	output reg  [1:0]  m_htrans,
	output reg  [31:0] m_hwdata,
	input  wire        m_hready,
	input  wire        m_hresp,        // DDR 窗口不应出错(占位监听)
	// ---- AHB 从机 (配置寄存器, clk 域, 零等待) ----
	input  wire        s_hsel,
	input  wire        s_hreadyin,
	input  wire [31:0] s_haddr,
	input  wire        s_hwrite,
	input  wire [1:0]  s_htrans,
	input  wire [31:0] s_hwdata,
	output reg         s_hreadyout,
	output reg         s_hresp,
	output reg  [31:0] s_hrdata,
	// ---- 中断 ----
	output wire        irq_frame_done  // 帧完成粘滞(CTRL[2] 门控)
);

	// ---------------- 参数派生 ----------------
	localparam WORDS_PER_FRAME = H_PIXELS * V_LINES / 2;   // 2 像素/字
	localparam WP_BITS  = $clog2(WORDS_PER_FRAME);
	localparam FB_BITS  = $clog2(FIFO_DEPTH);
	localparam [WP_BITS-1:0] WPF_W = WORDS_PER_FRAME[WP_BITS-1:0];

	// ================= 从机寄存器 =================
	localparam [2:0] R_CTRL   = 3'd0;  // [0]=en [1]=vsync_pol [2]=irq_en
	localparam [2:0] R_BASE0  = 3'd1;  // 缓冲 A 基址(字对齐)
	localparam [2:0] R_BASE1  = 3'd2;  // 缓冲 B 基址
	localparam [2:0] R_STATUS = 3'd3;  // [0]=done_sticky [1]=完成帧缓冲 [31:16]=已完成帧数
	localparam [2:0] R_IRQCLR = 3'd4;  // 写 1 清 done_sticky (W1C)

	reg         ctrl_en, ctrl_vsync_pol, ctrl_irq_en;
	reg  [31:0] base0_q, base1_q;
	reg         frame_done_sticky;
	reg         cur_buf;               // 当前写入缓冲: 0=A 1=B
	reg         last_buf;              // 刚完成帧的缓冲号(固件读数依据)
	reg  [15:0] frame_cnt;

	wire        s_wr = s_hsel && s_hreadyin && s_hwrite && s_htrans[1];
	wire        s_rd = s_hsel && s_hreadyin && !s_hwrite && s_htrans[1];
	wire [2:0]  s_reg = s_haddr[4:2];

	// 主机段驱动的前向声明
	wire        frame_done_set;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			ctrl_en <= 1'b0; ctrl_vsync_pol <= 1'b0; ctrl_irq_en <= 1'b0;
			base0_q <= 32'h0; base1_q <= 32'h0;
			frame_done_sticky <= 1'b0; cur_buf <= 1'b0;
			last_buf <= 1'b0; frame_cnt <= 16'h0;
		end else begin
			if (s_wr) begin
				case (s_reg)
					R_CTRL: begin
						ctrl_en        <= s_hwdata[0];
						ctrl_vsync_pol <= s_hwdata[1];
						ctrl_irq_en    <= s_hwdata[2];
						if (!s_hwdata[0]) begin    // 关闭即复位帧跟踪
							frame_cnt <= 16'h0;
							cur_buf   <= 1'b0;
						end
					end
					R_BASE0: base0_q <= s_hwdata;
					R_BASE1: base1_q <= s_hwdata;
					R_IRQCLR: frame_done_sticky <= 1'b0;
					default: ;
				endcase
			end
			if (frame_done_set) begin
				frame_done_sticky <= 1'b1;
				frame_cnt         <= frame_cnt + 16'd1;
				last_buf          <= cur_buf;   // 完成帧缓冲(翻转前取值)
				cur_buf           <= !cur_buf;  // 乒乓切换
			end
		end
	end

	assign irq_frame_done = frame_done_sticky & ctrl_irq_en;

	// ================= pclk 域: 采集 + 字节配对 =================
	reg        vsync_d, vsync_dd;
	reg [3:0]  blank_cnt;              // 已历消隐期计数(跳帧)
	reg        skip_done;
	reg        frame_active;           // 本帧采集窗口
	reg [1:0]  bcnt;                   // 行内字节相位 0..3
	reg [7:0]  bs0, bs1, bs2;          // 前 3 字节缓存
	reg [31:0] word_q;
	reg        word_vld;               // FIFO 写请求(一拍)
	reg        fifo_full_d;            // 满标志对齐 push 判定时刻

	wire vsync_eff = ctrl_vsync_pol ? cam_vsync : !cam_vsync;  // 帧消隐电平
	wire vsync_blank_end = vsync_d & !vsync_eff;               // 消隐结束→帧首行前

	// 字节配对: 帧窗口内 HREF 高电平期间, 4 字节拼 2 像素成一字
	// p0={b0,b1} 低半字, p1={b2,b3} 高半字 (首采样字节=像素高字节)
	// (H_PIXELS 为偶数 → 每行字节流 4 的倍数, 无半像素残留)
	always @(posedge cam_pclk or negedge rst_n) begin
		if (!rst_n) begin
			vsync_d <= 1'b0; vsync_dd <= 1'b0;
			blank_cnt <= 4'd0; skip_done <= 1'b0; frame_active <= 1'b0;
			bcnt <= 2'd0; bs0 <= 8'h0; bs1 <= 8'h0; bs2 <= 8'h0;
			word_q <= 32'h0; word_vld <= 1'b0;
		end else begin
			vsync_d  <= vsync_eff;
			vsync_dd <= vsync_d;
			word_vld <= 1'b0;
			if (vsync_blank_end) begin
				// 新帧开始: 跳帧计数推进, 达额后开启采集
				frame_active <= skip_done;
				if (!skip_done) begin
					blank_cnt <= blank_cnt + 4'd1;
					if (blank_cnt + 4'd1 >= SKIP_FRAMES[3:0])
						skip_done <= 1'b1;
				end
			end
			if (frame_active && cam_href) begin
				case (bcnt)
					2'd0: bs0 <= cam_data;
					2'd1: bs1 <= cam_data;
					2'd2: bs2 <= cam_data;
					2'd3: begin
						word_q   <= {bs2, cam_data, bs0, bs1};
						word_vld <= 1'b1;
					end
				endcase
				bcnt <= bcnt + 2'd1;
			end else if (!cam_href) begin
				bcnt <= 2'd0;                 // 行尾对齐兜底(行字节数为 4 的倍数)
			end
		end
	end

	// ================= 异步 FIFO (pclk 写 / clk 读, 格雷指针) =================
	reg [31:0] fifo_mem [0:FIFO_DEPTH-1];
	reg [FB_BITS:0] wbin, rbin;                    // 二进制(多 1 位判满)
	reg [FB_BITS:0] wgray, rgray;                  // 本域格雷输出
	reg [FB_BITS:0] rgray_w1, rgray_w2;            // 读指针 → 写域 (2FF)
	reg [FB_BITS:0] wgray_r1, wgray_r2;            // 写指针 → 读域 (2FF)

	wire fifo_push = word_vld && !fifo_full_d;
	wire [FB_BITS:0] wbin_n  = wbin + {{FB_BITS{1'b0}},1'b1};
	wire [FB_BITS:0] wgray_n = (wbin_n>>1) ^ wbin_n;
	// 格雷判满: 高 2 位相反 + 余位相同
	wire fifo_full = (wgray_n[FB_BITS -: 2] == ~rgray_w2[FB_BITS -: 2]) &&
	                 (wgray_n[FB_BITS-2:0] == rgray_w2[FB_BITS-2:0]);

	always @(posedge cam_pclk or negedge rst_n) begin
		if (!rst_n) begin
			wbin <= {(FB_BITS+1){1'b0}}; wgray <= {(FB_BITS+1){1'b0}}; rgray_w1 <= {(FB_BITS+1){1'b0}}; rgray_w2 <= {(FB_BITS+1){1'b0}};
		end else begin
			rgray_w1 <= rgray;
			rgray_w2 <= rgray_w1;
			if (fifo_push) begin
				fifo_mem[wbin[FB_BITS-1:0]] <= word_q;
				wbin  <= wbin_n;
				wgray <= wgray_n;
			end
		end
	end
	// 满标志打一拍, 与 push 生效拍对齐(防上溢的保守判定)
	always @(posedge cam_pclk) fifo_full_d <= fifo_full;

	// 读域
	wire [FB_BITS:0] rbin_n  = rbin + {{FB_BITS{1'b0}},1'b1};
	wire [FB_BITS:0] rgray_n = (rbin_n>>1) ^ rbin_n;

	wire       fifo_pop;               // 前向声明(定义在 skid 装填段; default_nettype none 下须先声明后引用)

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			rbin <= {(FB_BITS+1){1'b0}}; rgray <= {(FB_BITS+1){1'b0}}; wgray_r1 <= {(FB_BITS+1){1'b0}}; wgray_r2 <= {(FB_BITS+1){1'b0}};
		end else begin
			wgray_r1 <= wgray;
			wgray_r2 <= wgray_r1;
			if (fifo_pop) begin
				rbin  <= rbin_n;
				rgray <= rgray_n;
			end
		end
	end
	wire fifo_empty = (rgray == wgray_r2);   // 空: 读格雷 == 同步写格雷(标准判据)
	wire [31:0] fifo_rdata = fifo_mem[rbin[FB_BITS-1:0]];

	// ================= sys 域: AHB 写主机 =================
	localparam [2:0] HB_WORD = 3'b010, HB_INCR4 = 3'b011, HB_SINGLE = 3'b000;
	localparam [1:0] HT_IDLE = 2'b00, HT_NSEQ = 2'b10, HT_SEQ = 2'b01;
	localparam [1:0] M_IDLE = 2'd0, M_ADDR = 2'd1, M_DATA = 2'd2;

	reg [1:0]  m_state;
	reg [31:0] m_addr;                 // 当前突发起址
	reg [31:0] skid [0:3];             // 4 字突发缓冲
	reg [2:0]  skid_cnt;               // 0..4
	reg        m_incr4;                // 本突发 INCR4(否则 SINGLE)
	reg [2:0]  m_beat;                 // 本突发已完成数据拍(1..m_beats)
	reg [2:0]  m_beats;                // 本突发总拍数
	reg [WP_BITS-1:0] wp;              // 帧内已写字指针

	// 帧尾剩余字
	wire [WP_BITS:0] words_left = WORDS_PER_FRAME[WP_BITS-1:0] - wp;
	// 起突发条件: 满 4 字, 或帧尾剩余 ≤ 手头字数(SINGLE 收尾)
	wire burst_go = ctrl_en && (skid_cnt == 3'd4 ||
	                (skid_cnt > 3'd0 && (words_left <= {{(WP_BITS+1-3){1'b0}}, skid_cnt})));

	// skid 装填: 仅 M_IDLE 拍出栈一格(突发期间不装填, 每突发回 IDLE 补给)
	assign fifo_pop = (m_state == M_IDLE) && (skid_cnt < 3'd4) && !fifo_empty;

	// ---- 时序: skid 装填 + 突发推进 ----
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			m_state <= M_IDLE; m_addr <= 32'h0;
			skid_cnt <= 3'd0; skid[0] <= 32'h0; skid[1] <= 32'h0;
			skid[2] <= 32'h0; skid[3] <= 32'h0;
			m_incr4 <= 1'b0; m_beat <= 3'd0; m_beats <= 3'd0;
			wp <= {WP_BITS{1'b0}};
		end else begin
			// skid 装填(单拍一格)
			if (fifo_pop) begin
				skid[skid_cnt[1:0]] <= fifo_rdata;
				skid_cnt <= skid_cnt + 3'd1;
			end
			case (m_state)
				M_IDLE: begin
					if (burst_go && skid_cnt > 3'd0 && !fifo_pop) begin
						// 突发形态: 满 4 且帧余量≥4 → INCR4; 否则 SINGLE
						m_incr4 <= (skid_cnt == 3'd4) && (words_left >= 4);
						m_beats <= (skid_cnt == 3'd4) && (words_left >= 4) ? 3'd4 : 3'd1;
						m_beat  <= 3'd1;
						m_addr  <= (cur_buf ? base1_q : base0_q) + (wp32 << 2);
						m_state <= M_ADDR;
					end
				end
				M_ADDR: begin
					if (m_hready)
						m_state <= M_DATA;
				end
				M_DATA: begin
					if (m_hready) begin
						// 数据拍完成: skid 前移
						skid[0] <= skid[1]; skid[1] <= skid[2]; skid[2] <= skid[3];
						skid_cnt <= skid_cnt - 3'd1;
						wp       <= wp + {{(WP_BITS-1){1'b0}},1'b1};
						if (m_beat == m_beats) begin
							if (wp + {{(WP_BITS-1){1'b0}},1'b1} == WPF_W)
								wp <= {WP_BITS{1'b0}};            // 帧尾: 指针归零(frame_done_set 组合置位)
							m_state <= M_IDLE;
						end else begin
							m_beat <= m_beat + 3'd1;
						end
					end
				end
				default: m_state <= M_IDLE;
			endcase
		end
	end

	// 帧完成组合判定: 末数据拍被接受且写指针命中帧尾
	assign frame_done_set = (m_state == M_DATA) && m_hready &&
	                        (m_beat == m_beats) &&
	                        (wp + {{(WP_BITS-1){1'b0}},1'b1} == WPF_W);

	// ---- 总线输出 ----
	always @(*) begin
		case (m_state)
			M_ADDR: m_htrans = HT_NSEQ;
			M_DATA: m_htrans = (m_beat < m_beats) ? HT_SEQ : HT_IDLE;
			default: m_htrans = HT_IDLE;
		endcase
	end
	// 地址组合驱动: 地址相=突发起址; 数据相=下一拍地址(INCR 突发要求逐拍给出)
	always @(*) begin
		case (m_state)
			M_ADDR: m_haddr = m_addr;
			M_DATA: m_haddr = m_addr + (m_beat32 << 2);   // 已完成拍数×4
			default: m_haddr = m_addr;
		endcase
	end
	// hwdata 流水: 地址相预置 skid[0]; 之后每完成一拍 skid 前移, 展示下一字
	always @(*) begin
		m_hwdata = skid[0];
	end
	assign m_hwrite = 1'b1;
	assign m_hsize  = HB_WORD;
	assign m_hburst = m_incr4 ? HB_INCR4 : HB_SINGLE;

	// ================= 从机口时序(零等待, 与主机段不冲突) =================
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			s_hreadyout <= 1'b1;
			s_hresp     <= 1'b0;
			s_hrdata    <= 32'h0;
		end else begin
			s_hreadyout <= 1'b1;
			s_hresp     <= 1'b0;
			if (s_rd)
				s_hrdata <= s_hrdata_comb;
		end
	end

	// 从机读数据组合源(独立于时序段, 便于综合复用)
	reg  [31:0] s_hrdata_comb;
	wire [31:0] wp32    = {14'd0, wp};                 // 32bit 对齐(消 WIDTH 警)
	wire [31:0] m_beat32 = {29'd0, m_beat};
	always @(*) begin
		case (s_reg)
			R_CTRL:   s_hrdata_comb = {29'h0, ctrl_irq_en, ctrl_vsync_pol, ctrl_en};
			R_BASE0:  s_hrdata_comb = base0_q;
			R_BASE1:  s_hrdata_comb = base1_q;
			R_STATUS: s_hrdata_comb = {frame_cnt, 14'd0, last_buf, frame_done_sticky};
			default:  s_hrdata_comb = 32'h0;
		endcase
	end

	// hresp 占位监听: DDR 窗口不应出错; 出错即视为系统异常(由上层探针捕获)
	wire unused_tie = m_hresp;

endmodule

`default_nettype wire
