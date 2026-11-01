// ============================================================================
// video_line_buf.v — 显示读侧"按地址对齐"的行缓存（v63）
//
// 为什么要这个模块（一段历史）:
//   原来的读侧是 "read_en 弹出 → 等固定 L 拍 → 采样"，是【按时间】对齐。
//   H 消隐期间没有弹出，FIFO 输出只能保持 ⇒ 每行行首都欠 L 个字，
//   表现成一条 L 像素宽的竖缝（实测 L=16），而且 DDR 抢占引起的抖动会让 L 变动。
//   本模块把对齐改成【按地址】: burst 第 k 拍写进第 k 个槽位，显示第 p 个像素
//   读第 p>>4 槽的第 p&15 个字。两边各数各的，不存在"迟到几拍"这件事。
// ============================================================================
`default_nettype none

module video_line_buf #(
    parameter DW        = 16,      // 像素位宽
    parameter BEATS     = 40,      // 每行 beat 数 = H_NUM*DW/256 = 640*16/256
    parameter BANKS_LOG = 3        // 银行数 = 2^BANKS_LOG（必须能整除 720: 8 可以）
)(
    // ---- 写侧: DDR 读 beat ----
    input  wire          wr_clk,
    input  wire          wr_rst,       // 高有效, 帧起点
    input  wire [255:0]  wr_data,      // 一拍 16 个像素
    input  wire          wr_valid,
    // ---- 读侧: 显示 ----
    input  wire          rd_clk,
    input  wire          rd_vs,        // 场同步(上升沿清零行计数)
    input  wire          rd_de,        // 行有效
    input  wire [11:0]   rd_x,         // 行内像素号 0..H_NUM-1
    input  wire [8:0]    beat_ofs,     // v64: 槽位线性偏移 0..319 (一档 16px), 运行时可调
    output wire [DW-1:0] rd_px
);
    localparam BANKS = (1 << BANKS_LOG);
    localparam DEPTH = BANKS * BEATS;

    // ---------------- 写侧 ----------------
    reg [255:0]          mem [0:DEPTH-1];
    reg [7:0]            wbeat;        // 0..BEATS-1
    reg [BANKS_LOG-1:0]  wbank;
    wire                 wlast = (wbeat == (BEATS-1));

    always @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) begin
            wbeat <= 8'd0;
            wbank <= {BANKS_LOG{1'b0}};
        end else if (wr_valid) begin
            mem[wbank*BEATS + wbeat] <= wr_data;
            if (wlast) begin
                wbeat <= 8'd0;
                wbank <= wbank + 1'b1;
            end else begin
                wbeat <= wbeat + 8'd1;
            end
        end
    end

    // ---------------- 读侧 ----------------
    reg [BANKS_LOG-1:0]  rbank;
    reg                  rd_vs_d, rd_de_d;

    always @(posedge rd_clk or posedge wr_rst) begin
        if (wr_rst) begin
            rbank  <= {BANKS_LOG{1'b0}};
            rd_vs_d <= 1'b0;
            rd_de_d <= 1'b0;
        end else begin
            rd_vs_d <= rd_vs;
            rd_de_d <= rd_de;
            if (rd_vs & ~rd_vs_d)            rbank <= {BANKS_LOG{1'b0}};
            else if (~rd_de & rd_de_d)       rbank <= rbank + 1'b1;   // de 下降沿 = 本行结束
            /* 用下降沿而不是上升沿: 复位后 rbank=0 对应第 0 行, 行末才 ++, 不会差一行 */
        end
    end

    /* v64: 槽位线性偏移 —— 一档 = 1 个 beat = 16 px。 */
    wire [8:0]  ofs_c = (beat_ofs > 9'd319) ? 9'd319 : beat_ofs;
    /* 钳位: de 覆盖整条 1280 的合成流，窗口外 rd_x 会 >639（右半屏还会下溢），
     * 不钳位会算出越界索引。 */
    wire [9:0]  rx    = (rd_x > 12'd639) ? 10'd639 : rd_x[9:0];
    wire [10:0] lin_r = rbank*11'd40 + {5'd0, rx[9:4]} + {2'd0, ofs_c};
    wire [10:0] lin   = (lin_r >= 11'd320) ? (lin_r - 11'd320) : lin_r;

    (* ram_style = "distributed" *)
    reg [255:0] rd_word;
    always @(*) rd_word = mem[lin];

    wire [3:0] sub = rx[3:0];
    assign rd_px = rd_word[sub*DW +: DW];

endmodule
`default_nettype wire
