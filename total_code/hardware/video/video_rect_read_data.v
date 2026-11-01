module video_rect_read_data
#(
	parameter DATA_WIDTH = 16                       // Video data one clock data width
)
(
	input                       video_clk,          // Video pixel clock
	input                       rst,
	input[11:0]                 video_left_offset,
	input[11:0]                 video_top_offset,
	input[11:0]                 video_width,
	input[11:0]                 video_height,
	/* v52: 像素延迟级数（运行时寄存器，0..127）。读 FIFO 的 rd_en→rd_data 有若干拍
	 * 延迟，而原设计只补了 2 拍；差多少就会让整行横向偏多少像素（表现为一条竖缝）。
	 * 这里做一条 128 级像素延迟线，选出第 x_shift 级输出 ⇒ 等价于把图像左移 x_shift 像素，
	 * 运行时扫 0..127 即可让缝消失，不用反复出片。 */
	input[6:0]                  x_shift,
	/* v56 (2026-10-26): 读窗口前移量 (0..31)。
	 *   v55 的做法(推迟 de)会让窗口末端 16px 采样时 read_en 已停 ⇒ 黑带, 已废弃。
	 *   现在改为: 消隐期先预取 N 个字, 窗口末端对应少取 N 个 ⇒ 每行仍消耗 640 个,
	 *   不漂移; de/hs/vs 保持基线 ⇒ 既不缺像素也不产生黑带。
	 *   机理: 数据比 de 晚 Δ 像素(实测 16), 提前 Δ 拍请求即可对齐。
	 *
	 * 历史注释 (v55): 显示时序延迟。实测竖缝 = 每行最左 16px 是上一行尾部,
	 *   即"数据比 de 晚 16 像素"(读 FIFO 的 rd_en->rd_data 延迟没补够)。
	 *   数据延迟(x_shift)方向相反, 所以这里延迟 hs/vs/de 本身:
	 *   de 晚 N 像素 => 同一显示位置取到 N 像素之后的数据 => 图像右移 N, 缝闭合。
	 *   范围内 0..31, 出片后运行期扫描定值。 */
	input[4:0]                  x_delay,
	/* ==== v63: 行缓存（按地址对齐）====
	 * lb_en=1 时显示数据来自行缓存（与 FIFO 弹出时机无关）; lb_en=0 保持老路径，用于 A/B。
	 * 写侧从 DDR 读 beat 流直接灌入（256b = 16 像素/拍）。 */
	input                       lb_en,
	input                       lb_wr_clk,
	input                       lb_wr_rst,
	input[255:0]                lb_wr_data,
	input                       lb_wr_valid,
	/* v64: 行缓存槽位线性偏移 (0..319, 一档 16px)。上板实测行首仍差 1 个 beat,
	 * 由 overlay 0x64[9:1] 给, 运行时可扫, 不用重烧。 */
	input[8:0]                  lb_ofs,
	output reg                  read_req,           // Start reading a frame of data     
	input                       read_req_ack,       // Read request response
	(*mark_debug = "true"*)output                      read_en,            // Read data enable
	input[DATA_WIDTH - 1:0]     read_data,          // Read data
	input                       timing_hs,    
	input                       timing_vs,   
    output    reg[11:0]            x_cnt    ,
    output    reg[11:0]            y_cnt    ,
	(*mark_debug = "true"*)input                       timing_de,    
	(*mark_debug = "true"*)input[DATA_WIDTH - 1:0]     timing_data, 
	output                      hs,                 // horizontal synchronization
	output                      vs,                 // vertical synchronization
	(*mark_debug = "true"*)output                      de,                 // video valid
	output[DATA_WIDTH - 1:0]    vout_data           // video data
);
reg [11:0]    x_cnt_d0;
reg [11:0]    x_cnt_d1;
reg [11:0]    y_cnt_d0;
reg [11:0]    y_cnt_d1;

/* v57b: 声明前移（iverilog 严格检查"先声明后使用"）*/
(*mark_debug = "true"*)wire[11:0]             pos_x;
(*mark_debug = "true"*)wire[11:0]             pos_y;

always@(posedge video_clk or posedge rst)    begin
	if(rst == 1'b1)
	begin
        x_cnt    <=    12'd0;
        x_cnt_d0 <=    12'd0;
        x_cnt_d1 <=    12'd0;
        y_cnt    <=    12'd0;
        y_cnt_d0 <=    12'd0;
        y_cnt_d1 <=    12'd0;
    end
    else
    begin
        x_cnt_d0     <=    pos_x;
        x_cnt_d1     <=    x_cnt_d0;
        x_cnt        <=    x_cnt_d1;
        y_cnt_d0     <=    pos_y;
        y_cnt_d1     <=    y_cnt_d0;
        y_cnt        <=    y_cnt_d1;

    end
end
wire                   pos_hs;
wire                   pos_vs;
(*mark_debug = "true"*)wire                   pos_de;
wire[DATA_WIDTH - 1:0] pos_data;
reg[DATA_WIDTH - 1:0]  pos_data_d0;
reg[DATA_WIDTH - 1:0]  pos_data_d1;



//delay video_hs video_vs  video_de 2 clock cycles
reg                    pos_hs_d0;
reg                    pos_vs_d0;
reg                    pos_de_d0;
reg                    pos_hs_d1;
reg                    pos_vs_d1;
reg                    pos_de_d1;
/* v55: 时序延迟链 (32 级)。tq[0] 等价于原来的 pos_*_d2 三级延迟,
 *   tq[n] = 三级 + n 级; 输出取 tq[x_delay]。
 *   只有 {hs,vs,de} 三根被延迟, 数据通路(dl/vout_data_r)不动 —— 这正是
 *   "把显示节拍推后 N 像素"的效果, 与数据延迟(x_shift)方向相反。 */
reg [2:0]  tq [0:31];
integer    ti;
reg                    read_en_r;
reg                    read_en_r_d0;

reg[DATA_WIDTH - 1:0]  vout_data_r;
assign hs = tq[0][2];
assign vs = tq[0][1];
assign de = tq[0][0];
/* ==== v61 (2026-10-26 夜): 读流相位补偿 —— 最终形态 ====
 * 症状: 每行最左 ~16px 是上一行尾部（"右边截一段搬到左边"），左右半屏都有。
 * 判读: FIFO 的 rd_en→rd_data 延迟 L 对**每一行**都存在 ⇒ 每行开头都要补 L 个字。
 *   —— 数据总线在 H 消隐期没有弹出, 只能"保持"上一行最后一个字,
 *      所以这不是"每帧一次"（v58 的错误假设）, 而是"每行一次"。
 * 做法: ① 每行窗口开启前 x_delay 拍预取 x_delay 个字（把读流整体提前）;
 *       ② 窗口内少取 x_delay 个 ⇒ 每行总弹出恒 640, 不产生逐行漂移;
 *       ③ 采样/移位门控用 win_disp（v57 改）—— 否则补出来的字会被丢弃。
 * ★ v61 新增: ②/①的"预取位置"必须紧贴**本实例自己的窗口**, 不是固定放消隐末端:
 *   left_offset==0（左半屏 inst0）: 窗口从行首起 ⇒ 放 H/V 消隐末端 ✓
 *   left_offset==640（右半屏 inst1）: 窗口在行内第 640 像素 ⇒ 必须在行内
 *       pos_x = 640-x_delay 处预取。放消隐末端会隔 640 拍, 补齐全部作废。
 *   sim/tb_rect_read.v +LO=640 实测: 修前右半屏每行仍错 16px（= 用户看到的缝）,
 *   修后 20 行只剩 19 处（≈1px/行）。左半屏 20 行 53 处（第0行15 + 2px/行）。
 *   两半都不漂移 ⇒ 说明每行总弹出确实恒 640。
 * 板上用法: 0x68 (x_delay) 扫 0..24, 只有一档会"咔"一下对齐; 经验值 = L + 1。
 * 残留: 每行前 1~2 像素（dl(1)+vout(1) 两级输出寄存器的空档）;
 *   第 0 行前 x_delay 个像素（V 消隐那次预取离得太远）, 每帧一行, 不可见。
 * 历史: v58 的"每帧预取"是错的（每行仍错 16px）, 源码留 .bak_v58;
 *      v57b 是正确结构但只对左半屏有效（.bak_v57b）。 */
localparam [11:0] GAP_LEN = 12'd370;
reg [11:0] gap_cnt;
reg        pre_en;
/* 预取判据: LO==0 走消隐, LO!=0 走行内; pre_en 打一拍才成为 read_en */
wire pre_blank = (x_delay != 5'd0) && (video_left_offset == 12'd0) &&
                 (gap_cnt >= (GAP_LEN - {7'd0, x_delay})) && (gap_cnt < GAP_LEN);
wire pre_line  = (x_delay != 5'd0) && (video_left_offset != 12'd0) && pos_de &&
                 (pos_x >= (video_left_offset - {7'd0, x_delay})) &&
                 (pos_x <  (video_left_offset));
always @(posedge video_clk or posedge rst) begin
	if (rst == 1'b1) begin
		gap_cnt <= 12'd0;
		pre_en  <= 1'b0;
	end else begin
		gap_cnt <= (pos_de == 1'b1) ? 12'd0
		         : ((gap_cnt < GAP_LEN) ? (gap_cnt + 12'd1) : gap_cnt);
		pre_en  <= pre_blank | pre_line;
	end
end

always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		pos_hs_d0 <= 1'b0;
		pos_vs_d0 <= 1'b0;
		pos_de_d0 <= 1'b0;
		pos_hs_d1 <= 1'b0;
		pos_vs_d1 <= 1'b0;
		pos_de_d1 <= 1'b0;
		pos_data_d0 <= {DATA_WIDTH{1'b0}};
		pos_data_d1 <= {DATA_WIDTH{1'b0}};
		read_en_r_d0 <= 1'b0;
		for(ti=0; ti<32; ti=ti+1) tq[ti] <= 3'd0;
	end
	else
	begin
		//delay pos hs vs de 3 clock cycles
		pos_hs_d0   <= pos_hs;
		pos_vs_d0   <= pos_vs;
		pos_de_d0   <= pos_de;
		pos_hs_d1   <= pos_hs_d0;
		pos_vs_d1   <= pos_vs_d0;
		pos_de_d1   <= pos_de_d0;	
		/* v55: 三级基线 -> tq[0], 再串 31 级 */
		tq[0] <= {pos_hs_d1, pos_vs_d1, pos_de_d1};
		for(ti=1; ti<32; ti=ti+1) tq[ti] <= tq[ti-1];
		pos_data_d0 <= pos_data;
		pos_data_d1 <= pos_data_d0;
		read_en_r_d0 <= read_en_r;
	end
end

assign read_en = read_en_r | pre_en;
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		read_en_r <= 1'b0;
	else if(pos_de == 1'b1 && pos_x >= video_left_offset
	        && (pos_x + {7'd0, x_delay}) < (video_left_offset + video_width)
	        && pos_y >= video_top_offset && pos_y < video_top_offset + video_height)
		read_en_r <= 1'b1;
	else
		read_en_r <= 1'b0;
end
/*always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		read_en_r <= 1'b0;
	else if( pos_de && pos_x >= 0 && pos_x <= 511  &&
	        pos_y >= 0 && pos_y <=383)
		read_en_r <= 1'b1;
	else
		read_en_r <= 1'b0;
end*/
/* ---- v52 像素延迟线（补偿读 FIFO 的输出延迟）---- */
/* v57b: win_disp 前移定义；用 x_cnt_d1 与 de 对齐（de=pos_de_d2，x_cnt_d1=pos_x 延 2 拍）
 * 原写法用输出 x_cnt（延 3 拍）⇒ 采样窗口与 de 差 1 拍，首/末像素会采错。*/
wire win_disp = de && (x_cnt_d1 >= video_left_offset)
                   && (x_cnt_d1 <  video_left_offset + video_width);

reg [DATA_WIDTH-1:0] dl [0:127];
integer di;
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		for(di=0; di<128; di=di+1) dl[di] <= {DATA_WIDTH{1'b0}};
	end
	else if(win_disp == 1'b1)		// 只在显示窗口内推进（与采样同条件, 保证一一对应）
	begin
		dl[0] <= read_data;
		for(di=1; di<128; di=di+1) dl[di] <= dl[di-1];
	end
end
wire [DATA_WIDTH-1:0] dl_tap = dl[x_shift];

/* v57 (2026-10-26): 采样门控改用"显示窗口"而不是 read_en。
 *   v56 的教训: 消隐预取把 16 个字从 FIFO 弹出, 但 dl 只在 read_en_r_d0 时移位
 *   ⇒ 弹出的 16 个字被丢弃(起点没前移), 同时窗口末端 16 拍又无采样(黑带)。
 *   改成: 只要"显示窗口内"(de 有效且 x 在窗口)就采样, 且 dl 同步移位。
 *   x_delay=16 时数据总线在第 i 拍正好是第 i 个字 ⇒ 窗口内逐拍采样即对齐;
 *   x_delay=0 时行为与改动前完全一致(无回归)。 */
/* ---- v63: 行缓存实例 ----
 * 读侧用 de(整行) 计行号、用 x_cnt_d1-LO 作为行内像素号（与 de 同拍对齐）。
 * 写侧由顶层把 DDR 读 beat（256b）直接接进来。 */
wire [DATA_WIDTH-1:0] lb_px;
video_line_buf #(.DW(DATA_WIDTH), .BEATS(40), .BANKS_LOG(3)) u_line_buf (
	.wr_clk   ( lb_wr_clk   ),
	.wr_rst   ( lb_wr_rst   ),
	.wr_data  ( lb_wr_data  ),
	.wr_valid ( lb_wr_valid ),
	.rd_clk   ( video_clk   ),
	.rd_vs    ( pos_vs      ),
	.rd_de    ( de          ),
	.rd_x     ( x_cnt - video_left_offset ),   /* de 是 pos_de 延 3 拍(tq[0] 又打一拍), 用延 3 的 x_cnt 才对 */
	.beat_ofs ( lb_ofs       ),                /* v64: 槽位线性偏移, 一档 16px */
	.rd_px    ( lb_px       )
);

/* 新路径专用窗口门控: 用延 3 拍的 x_cnt, 与 de 同拍（win_disp 用的是延 2 的 x_cnt_d1,
 * 那是老路径的历史写法, 差 1 拍 —— 老路径靠 x_delay 吸收掉了, 新路径按地址取不能差）。*/
wire win_disp_lb = de && (x_cnt >= video_left_offset)
                      && (x_cnt <  video_left_offset + video_width);

/* lb_en=1: 行缓存（按地址，与弹出时机无关）; lb_en=0: 老路径（A/B 用） */
assign vout_data = (lb_en == 1'b1) ? (win_disp_lb ? lb_px : pos_data_d1)
                                   : vout_data_r;
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		vout_data_r <= {DATA_WIDTH{1'b0}};
	else if(win_disp == 1'b1)
		vout_data_r <= dl_tap;
	else
		vout_data_r <= pos_data_d1;
end

always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		read_req <= 1'b0;
	else if(pos_vs_d0 == 1'b1 & pos_vs == 1'b0) //vertical synchronization edge (the rising or falling edges are OK)
		read_req <= 1'b1;
	else if(read_req_ack == 1'b1)
		read_req <= 1'b0;
end

timing_gen_xy#(.DATA_WIDTH(DATA_WIDTH)) timing_gen_xy_m0(
	.rst_n    (~rst          ),
	.clk      (video_clk     ),
	.i_hs     (timing_hs     ),
	.i_vs     (timing_vs     ),
	.i_de     (timing_de     ),
	.i_data   (timing_data   ),
	.o_hs     (pos_hs        ),
	.o_vs     (pos_vs        ),
	.o_de     (pos_de        ),
	.o_data   (pos_data      ),
	.x        (pos_x         ),
	.y        (pos_y         )
);
endmodule 