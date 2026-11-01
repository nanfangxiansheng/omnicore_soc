// ============================================================================
// async_fifo_dc.v — 双时钟异步 FIFO（格雷指针, Cummings 风格）
// 融合工程通用 CDC 组件: ddr_bridge 的 32b(AHB)/256b(DDR) 位宽转换两端使用。
// DEPTH 必须为 2 的幂; OUTREG=0 时读数据组合有效 (供流式突发供给)。
// ============================================================================
`default_nettype none

module async_fifo_dc #(
    parameter WIDTH      = 32,
    parameter DEPTH_LOG2 = 9,     // 深度 = 2^DEPTH_LOG2
    parameter OUTREG     = 1      // 1=读出口寄存一拍; 0=组合读 (mem[rptr])
) (
    // 写域
    input  wire             wr_clk,
    input  wire             wr_rst_n,
    input  wire             wr_en,
    input  wire [WIDTH-1:0] wr_data,
    output wire             full,
    // ZIGUANG (2026-10-18): 写域当前存量 (0..2^DEPTH_LOG2)。
    //   为什么需要: wload_dma 的 DDR 读突发从"单拍"改成"多拍 burst"后,
    //   一个 burst 会在几拍内连续塞进多个字。若中途 FIFO 满, 原来的
    //   `wr_en && !full` 会**静默丢字**(权重错但现场看不出), 所以必须在
    //   **发 burst 之前**就知道还剩多少空间, 空间不够就不发。
    //   注意方向: rptr 经 2FF 同步到写域有延迟, 期间读域已弹但这里还不知道
    //   ⇒ wr_data_count **偏大** ⇒ 估算空间偏小 ⇒ 保守, 不会溢出。
    output wire [DEPTH_LOG2:0] wr_data_count,
    // 读域
    input  wire             rd_clk,
    input  wire             rd_rst_n,
    input  wire             rd_en,
    output wire [WIDTH-1:0] rd_data,
    output wire             empty
);

    localparam AW = DEPTH_LOG2;

    reg [WIDTH-1:0] mem [0:(1<<AW)-1];
    reg [AW:0]      wptr_bin, rptr_bin;
    reg [AW:0]      wptr_gray, rptr_gray;
    reg [AW:0]      wptr_gray_r1, wptr_gray_r2;   // 写指针 → 读域 (2FF)
    reg [AW:0]      rptr_gray_r1, rptr_gray_r2;   // 读指针 → 写域 (2FF)
    reg [WIDTH-1:0] rd_data_r;

    wire [AW:0] wptr_bin_next  = wptr_bin + {{AW{1'b0}}, 1'b1};
    wire [AW:0] rptr_bin_next  = rptr_bin + {{AW{1'b0}}, 1'b1};
    wire [AW:0] wptr_gray_next = (wptr_bin_next >> 1) ^ wptr_bin_next;
    wire [AW:0] rptr_gray_next = (rptr_bin_next >> 1) ^ rptr_bin_next;

    // ZIGUANG 修复 (2026-10-09): 原 empty 用【下一个】读指针比较
    // (rptr_gray_next), 语义变成"剩 1 个即报空、真空时报非空"(复位态即
    // 报非空) —— 排水侧被假空挡拍, beat 永久滞留 FIFO (tb_bridge 实测:
    // 8 push / 7 pop, T1 泄 1 beat, 后续全部事务流错位)。组合 empty 必须
    // 用当前 rptr 对同步后的 wptr; full 保留 next 口径 (Cummings 注册式
    // 语义, 滞后 2 拍的 rptr_r2 使其保守偏严, 不会溢出)。
    wire full_next  = (wptr_gray_next == rptr_gray_r2);
    wire empty_flag = (rptr_gray == wptr_gray_r2);

    reg [WIDTH-1:0] rd_data_mux;
    always @(*) rd_data_mux = mem[rptr_bin[AW-1:0]];
    assign rd_data = (OUTREG != 0) ? rd_data_r : rd_data_mux;

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wptr_bin <= 0; wptr_gray <= 0;
            rptr_gray_r1 <= 0; rptr_gray_r2 <= 0;
        end else begin
            rptr_gray_r1 <= rptr_gray;
            rptr_gray_r2 <= rptr_gray_r1;
            if (wr_en && !full) begin
                mem[wptr_bin[AW-1:0]] <= wr_data;
                wptr_bin  <= wptr_bin_next;
                wptr_gray <= wptr_gray_next;
            end
        end
    end

    assign full  = full_next;

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rptr_bin <= 0; rptr_gray <= 0;
            wptr_gray_r1 <= 0; wptr_gray_r2 <= 0;
            rd_data_r <= 0;
        end else begin
            wptr_gray_r1 <= wptr_gray;
            wptr_gray_r2 <= wptr_gray_r1;
            if (rd_en && !empty) begin
                if (OUTREG) rd_data_r <= mem[rptr_bin[AW-1:0]];
                rptr_bin  <= rptr_bin_next;
                rptr_gray <= rptr_gray_next;
            end
        end
    end

    assign empty = empty_flag;

    // ------------------------------------------------------------------
    // 写域存量: wptr_bin - rptr_bin(已同步到写域)
    //   rptr_gray_r2 是写域里的读指针(gray), gray→bin: bin[i] = ^(gray >> i)
    //   —— 与 ddr_bridge_ahb.v 里 g_gray2bin 同一套写法。
    // ------------------------------------------------------------------
    reg [AW:0] rptr_bin_w;
    integer    gi;
    always @(*) begin
        for (gi = 0; gi <= AW; gi = gi + 1)
            rptr_bin_w[gi] = ^(rptr_gray_r2 >> gi);
    end
    assign wr_data_count = wptr_bin - rptr_bin_w;

endmodule

`default_nettype wire
