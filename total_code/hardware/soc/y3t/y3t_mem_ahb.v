// ============================================================================
// y3t_mem_ahb.v — Y3T 引擎存储器 AHB 桥（M3.1）
// 真双口 256b RAM：
//   A 口 = 引擎专用，时序逐拍复刻 mem_model.v（LAT=1、读/写 burst）——
//         engine_y3t 无需任何改动即可接入（tb_y3t 单元台口径不变）。
//   B 口 = CPU 专用，经 AHB-Lite 从机（boardsoc 约定）访问。时序与全体从机
//         一致（bram_slave/cfg 口同款，实测偏离会触发 CPU 侧故障，见 REPORT-02）：
//         读  = 零等待（接受拍寄存读出，次拍 hrdata 有效）；
//         写  = 1 等待拍（S_WR 采样 hwdata，读-改-写合并 32b lane 提交）。
// 固件纪律：引擎 job 执行期间 CPU 不得访问 B 口（done 轮询走 cfg 口），
//   双口同字同拍冲突因此结构性不存在。
// ============================================================================
`default_nettype none

module y3t_mem_ahb #(
    parameter WORDS   = 131072,          // 2^17 × 32B = 4MB（仿真全容量）
    parameter PRELOAD = "y3t_mem.hex",   // 行为级路径预载文件; yosys 在 AST 前端
                                         // 用默认参数解析 readmemh 文件名, 默认不可为空
    parameter USE_IP  = 1,               // 1 = 例化 y3tram IP（综合板级路径, 需 WORDS==32768）
    parameter DO_PRELOAD = 0             // 1 = 行为级路径执行 $readmemh 预载; IP 路径置 0
) (
    input  wire        clk,
    input  wire        rst_n,
    // ---- AHB-Lite 从（boardsoc 约定口） ----
    input  wire        hsel,
    input  wire        hready,
    input  wire        htrans_nseq,
    input  wire [21:2] haddr,
    input  wire        hwrite,
    input  wire [2:0]  hsize,   // AHB HSIZE 标准 3 位; 本从机仅字访问 (hsize[1:0]==10)
    input  wire [31:0] hwdata,
    output wire        hreadyout,
    output wire [31:0] hrdata,
    output wire        hresp,
    // ---- 引擎存储口（= mem_model 口，LAT=1 语义） ----
    input  wire        eng_req,
    input  wire        eng_we,
    input  wire [7:0]  eng_blen,
    input  wire [31:0] eng_addr,
    input  wire [255:0] eng_wdata,
    output wire [255:0] eng_rdata,
    output wire        eng_ack,
    /* 【2026-10-21 20:20】观测输出：存储器真实写入次数与最后地址。
     *   板上已确认：DDR 读侧 1152 burst 全收、FIFO 满丢 0、done_words=9216（请求全发），
     *   但 y3tram 只有 ~50% 正确。所以必须知道 ram_we 到底跳了几次：
     *     ram_we ≈ 9216  -> 写都发生了，问题在地址或 wdata
     *     ram_we ≈ 4608  -> 写被吞，问题在 a_wr_r / a_ack 链 */
    output reg  [15:0] dbg_wr_cnt,
    output reg  [14:0] dbg_last_addr
);

    function integer clog2;
        input integer value;
        begin
            for (clog2 = 0; value > 1; clog2 = clog2 + 1)
                value = value >> 1;
        end
    endfunction

    localparam AW = clog2(WORDS);        // 256b 字地址位宽

    /* 写计数: 独立 always 块。绝不能并进下面那个存储器写块 ——
     *   文件头注释已记录: 存储器写必须收敛在单个 always 块,
     *   夹带其它逻辑会让 memory 展开成寄存器阵列(综合 OOM)。 */
    always @(posedge clk) begin
        if (!rst_n) begin
            dbg_wr_cnt    <= 16'd0;
            dbg_last_addr <= 15'd0;
        end else if (ram_we) begin
            dbg_wr_cnt    <= dbg_wr_cnt + 16'd1;
            dbg_last_addr <= ram_waddr[14:0];
        end
    end

    // ======================================================================
    // B 口：AHB 从机 FSM（bram_slave/cfg 口同款时序）
    //   读:  接受拍寄存读出 → S_RD (hready=1, hrdata 有效) → IDLE   [零等待]
    //   写:  接受拍寄存读出 → S_WR (采样 hwdata, RMW 合并提交) → IDLE [1 等待]
    //   错:  两周期 ERROR
    // ======================================================================
    localparam [2:0] S_IDLE = 3'd0, S_RD = 3'd1, S_WR = 3'd2,
                     S_ERR1 = 3'd3, S_ERR2 = 3'd4;
    reg [2:0]  state;
    reg [16:0] addr_r;
    reg [2:0]  lane_r;

    wire word_ok = (hsize[1:0] == 2'b10);
    wire accept  = hsel && htrans_nseq && hready &&
                   (state == S_IDLE || state == S_RD || state == S_ERR2);
    wire accept_bad = accept && !word_ok;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state  <= S_IDLE;
            addr_r <= 17'd0;
            lane_r <= 3'd0;
        end else begin
            if (accept) begin
                addr_r <= haddr[21:5];
                lane_r <= haddr[4:2];
                state  <= accept_bad ? S_ERR1 :
                          hwrite     ? S_WR  : S_RD;
            end else begin
                case (state)
                S_RD:  state <= S_IDLE;
                S_WR:  state <= S_IDLE;
                S_ERR1: state <= S_ERR2;
                S_ERR2: state <= S_IDLE;
                default: state <= S_IDLE;
                endcase
            end
        end
    end

    // 读零等待: S_RD 拍 hready=1; 写 1 等待: S_WR 拍 hready=0
    assign hreadyout = (state == S_WR)   ? 1'b0 :
                       (state == S_ERR1) ? 1'b0 : 1'b1;
    assign hresp = (state == S_ERR1) || (state == S_ERR2);

    // ======================================================================
    // 统一读数据（两实现均为"地址呈现后一拍有效"）:
    //   rd_rdata_v ↔ 原 rd_rdata_r (B 口读出/RMW 旧值)
    //   a_rdata_v  ↔ 原 a_rdata    (引擎 LAT=1 读出)
    // 固件纪律下两者无需同拍并存, 共享存储读口。
    // ======================================================================
    wire [255:0] rd_rdata_v;
    wire [255:0] a_rdata_v;

    // RMW 合并: 读回字 + master 保持的 hwdata 嵌入目标 lane
    // (rd_rdata_v 在接受拍后一拍有效, 与原实现 rd_rdata_r 同拍)
    reg [255:0] merge_c;
    always @(*) begin
        merge_c = rd_rdata_v;
        merge_c[lane_r*32 +: 32] = hwdata;
    end

    // ======================================================================
    // A 口引擎协议（mem_model.v LAT=1 逻辑原样移植, 仅搬出存储访问）
    // ======================================================================
    reg         a_wr_r;
    reg  [7:0]  a_b_left;
    reg  [31:0] a_b_addr;
    reg         a_ack;

    always @(posedge clk) begin
        if (!rst_n) begin
            a_wr_r   <= 1'b0;
            a_ack    <= 1'b0;
            a_b_left <= 8'd0;
            a_b_addr <= 32'd0;
        end else begin
            if (eng_req && !eng_we) begin
                a_b_left <= eng_blen;
                a_b_addr <= eng_addr + 32'd1;
                a_wr_r   <= 1'b0;
            end
            if (eng_req && eng_we) begin
                a_b_left <= eng_blen;
                a_b_addr <= eng_addr;
                a_wr_r   <= 1'b1;
            end
            a_ack <= eng_req;                 // LAT=1：首 ack
            if (a_ack && a_wr_r) begin        // 写 burst：每 ack 拍提交一个字
                a_b_addr <= a_b_addr + 32'd1;
                if (a_b_left != 0) begin
                    a_ack    <= 1'b1;
                    a_b_left <= a_b_left - 8'd1;
                end else begin
                    a_wr_r <= 1'b0;
                end
            end else if (a_ack && a_b_left != 0) begin   // 读 burst 续拍
                a_ack    <= 1'b1;
                a_b_left <= a_b_left - 8'd1;
                a_b_addr <= a_b_addr + 32'd1;
            end
        end
    end
    assign eng_ack = a_ack;

    // ======================================================================
    // 双口归并（固件纪律: A/B 不同时活跃 → 各归并成单口, 可映射 SDP BRAM）
    //   写: 引擎写 burst beat 优先（与原实现的语句序优先级一致）
    //   读: 引擎首拍 eng_addr / 续拍 a_b_addr（续拍优先, 与原覆盖序一致）
    // ======================================================================
    wire eng_act = eng_req || a_ack;

    wire         a_rd_first = eng_req && !eng_we;
    wire         a_rd_cont  = a_ack && !a_wr_r && (a_b_left != 8'd0);
    wire         a_rd_port  = a_rd_first || a_rd_cont;
    // verilator lint_off WIDTH
    wire [AW-1:0] a_rd_idx  = a_rd_cont ? a_b_addr[AW-1:0] : eng_addr[AW-1:0];
    // verilator lint_on WIDTH

    wire          ram_we    = eng_act ? (a_ack && a_wr_r) : (state == S_WR);
    // verilator lint_off WIDTH
    wire [AW-1:0] ram_waddr = eng_act ? a_b_addr[AW-1:0]  : addr_r[AW-1:0];
    wire [255:0]  ram_wdata = eng_act ? eng_wdata         : merge_c;
    wire          ram_re    = eng_act ? a_rd_port          : (accept && !accept_bad);
    wire [AW-1:0] ram_raddr = eng_act ? a_rd_idx           : haddr[AW+4:5];
    // verilator lint_on WIDTH

    assign eng_rdata = a_rdata_v;
    assign hrdata    = rd_rdata_v[lane_r*32 +: 32];

    generate
    if (USE_IP) begin : g_ip
        // ---- 综合路径: Pango DRM SDPRAM IP（ipcore/y3tram, 256b×32768）----
        wire [255:0] ip_rdata;
        y3tram u_y3tram (
            .wr_clk   (clk),
            .wr_rst   (~rst_n),
            .wr_en    (ram_we),
            .wr_addr  (ram_waddr),
            .wr_data  (ram_wdata),
            .rd_clk   (clk),
            .rd_rst   (~rst_n),
            .rd_addr  (ram_raddr),
            .rd_data  (ip_rdata)
        );
        assign rd_rdata_v = ip_rdata;
        assign a_rdata_v  = ip_rdata;
    end else begin : g_behav
        // ---- 仿真/推断路径: 行为级阵列（单 always 块、单写口、同步风格）----
        reg [255:0] mem [0:WORDS-1];

        integer i, j;
        initial begin
`ifndef YOSYS
            // 分块清零: 单层 WORDS 次循环超出解析器/综合引擎的
            // 2000 次循环展开上限 (Verilog-4092), 拆为外层 ceil(WORDS/1024)
            // 次 × 内层 1024 次, 两级均 < 2000, 对任意 WORDS 值均正确。
            for (i = 0; i < (WORDS + 1023) / 1024; i = i + 1)
                for (j = 0; j < 1024; j = j + 1)
                    if (i * 1024 + j < WORDS)
                        mem[i * 1024 + j] = 256'd0;
`endif
            if (DO_PRELOAD)
                $readmemh(PRELOAD, mem);     // DO_PRELOAD=0 时跳过 (板上综合路径)
        end

        reg [255:0] mem_rdata_q;
        // 存储器与读寄存口不进异步复位进程 (BRAM 推断要求; CODE-05)。
        always @(posedge clk) begin
            if (ram_we)
                mem[ram_waddr] <= ram_wdata;
            if (ram_re)
                mem_rdata_q <= mem[ram_raddr];
        end
        assign rd_rdata_v = mem_rdata_q;
        assign a_rdata_v  = mem_rdata_q;
    end
    endgenerate

endmodule

`default_nettype wire
