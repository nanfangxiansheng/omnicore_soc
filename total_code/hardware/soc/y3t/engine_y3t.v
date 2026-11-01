// ============================================================================
// engine_y3t.v — Y3T 卷积引擎
// 源与裁剪依据：doc/npu develop/DESIGN-M2-引擎移植.md §二
//
// 模式（cfg_mode）：
//   0 CONV：ICH tile × OCH tile 循环，K=1/3，S=1/2，pad，2 输入区（concat 零拷贝，
//     split = g0 的 ich tile 数，ng2 两级尺度组分组 requant）。tile-major 打包：
//     addr = base + tile*H*W + y*W + x（每字 256b = 16ch int16，通道 pad 到 16）。
//   2 UPSAMPLE：2×2 最近邻复制（写侧），cfg_tiles = 通道 tile 数。
//   3 POOL2：2×2 maxpool s2（无 pad，H/W 全偶；lane 级有符号取大）。
//   其他：err（母本 ADD/ATTN/POOL5 已裁剪）。
// 量化契约 = requant_pipe_y3t.v（= requant_pipe.v，act=ReLU），对拍基准
//   tools/y3t/golden_np.py conv_job_np（G3 门）。
// 相对母本的机制裁剪（硬编码 WGROUP=1）：ATTN/ADD 全删；POOL5→POOL2；OT4 组连发、
//   kx burst、行缓冲 lbuf、预取 pf、生产者/消费者 FIFO、acc_ram RMW 全删——
//   这些路径在 WGROUP=1 下本就不可达；CONV 主路径（CW_LDW→CT→C_ACCSAVE/C_ACCWAIT→
//   C_RQ→C_WR）与母本 134 门验证过的遗产流逐拍一致。写回 burst（P8-A3 攒字）
//   与 WGROUP 无关，原样保留。
// 时序约定：REQ 拍发请求（单拍脉冲），CAP 拍等 ack；mac/rq 控制信号同拍寄存。
// ============================================================================
module engine_y3t (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    output reg         done,
    output reg         err,
    input  wire [2:0]  cfg_mode,
    input  wire [1:0]  cfg_k,
    input  wire [1:0]  cfg_s,
    input  wire [1:0]  cfg_pad,
    input  wire [9:0]  cfg_H,
    input  wire [9:0]  cfg_W,
    input  wire [9:0]  cfg_Ho,
    input  wire [9:0]  cfg_Wo,
    input  wire [6:0]  cfg_cit,
    input  wire [6:0]  cfg_cot,
    input  wire [6:0]  cfg_split,
    input  wire [5:0]  cfg_shM0,
    input  wire [5:0]  cfg_shM1,
    input  wire [5:0]  cfg_shY,
    input  wire [31:0] cfg_multY,
    input  wire        cfg_act,
    input  wire        cfg_bits16,
    input  wire [6:0]  cfg_tiles,
    input  wire [31:0] cfg_in0,
    input  wire [31:0] cfg_in1,
    input  wire [31:0] cfg_wb,
    input  wire [31:0] cfg_out,
    // ---- 配置写口：0..2047 multm_rom；2048..3071 bias_rom ----
    input  wire        cfg_we,
    input  wire [11:0] cfg_waddr,
    input  wire [31:0] cfg_wdata,
    // ---- 存储口（256b 字地址；读单字 blen=0；写 burst ≤15） ----
    output wire        mem_req,
    output wire        mem_we,
    output wire [31:0] mem_addr,
    output wire [7:0]  mem_blen,
    output wire [255:0] mem_wdata,
    input  wire [255:0] mem_rdata,
    input  wire        mem_ack,
    // ---- busy-cycle 统计 ----
    output reg  [31:0] cyc_mac,
    output reg  [31:0] cyc_tot,
    output wire [5:0]  dbg_state
);
    // ---------------- 状态（先于引用点声明） ----------------
    localparam M_CONV = 3'd0, M_UPS = 3'd2, M_POOL = 3'd3;
    localparam E_IDLE = 6'd0,
               CW_LDW_REQ = 6'd1,  CW_LDW_CAP = 6'd2,
               CT_REQ = 6'd3,      CT_CAP = 6'd4,
               C_ACCSAVE = 6'd5,   C_ACCSAVE2 = 6'd6,
               C_ACCWAIT = 6'd7,
               C_RQ = 6'd8,        C_RQ_DRAIN = 6'd9,
               C_WR_REQ = 6'd10,   C_WR_WAIT = 6'd11,
               E_DONE = 6'd12,
               U_RD_REQ = 6'd13,   U_RD_CAP = 6'd14,
               U_WR_REQ = 6'd15,   U_WR_WAIT = 6'd16,
               P2_RD = 6'd17,      P2_CAP = 6'd18,
               P2_WR_REQ = 6'd19,  P2_WR_WAIT = 6'd20,
               C_ACCSAVE3 = 6'd21, C_ACCSAVE4 = 6'd22, C_ACCSAVE5 = 6'd23,
               C_RQ_WAIT = 6'd24,  C_RQ_WAIT2 = 6'd25,
               C_WR_NEXT = 6'd26;
    reg [5:0]  state;

    // ---------------- 存储口脉冲寄存 ----------------
    reg         mq_req, mq_we;
    reg  [31:0] mq_addr;
    reg  [7:0]  mq_blen;
    reg  [255:0] mq_wdata;
    assign mem_req   = mq_req;
    assign mem_we    = mq_we;
    assign mem_addr  = mq_addr;
    assign mem_blen  = mq_blen;
    assign mem_wdata = mq_wdata;

    // ---------------- 尺度 ROM（配置写口加载） ----------------
    reg signed [31:0] multm_rom [0:2047];   // {group,och tile[5:0],lane[3:0]}
    reg signed [31:0] bias_rom  [0:1023];   // {och tile[5:0],lane[3:0]}
    always @(posedge clk) begin
        if (cfg_we) begin
            if (cfg_waddr < 12'd2048)      multm_rom[cfg_waddr[10:0]] <= cfg_wdata;
            else if (cfg_waddr < 12'd3072) bias_rom[cfg_waddr[9:0]]   <= cfg_wdata;
        end
    end

    // ---------------- 权重缓冲（BRAM：wfill 拼 2048b 整字后同步写；同步读） ----------------
    (* ram_style = "block" *) reg [2047:0] wbuf [0:575];    // (it,tap) → 16 OCH×16 ICH int8
    reg [2047:0] wfill;
    reg [2047:0] wbuf_q;

    // ---------------- 循环/暂存寄存 ----------------
    reg  [6:0]  ot, it, tile;
    reg  [9:0]  oy, ox;
    reg  [12:0] cnt;
    reg  [3:0]  tap;
    reg  [4:0]  rq_odx;
    reg         rq_active;
    reg  [5:0]  fc;
    reg  [255:0] out_word, word_a;      // word_a：UPS 读缓存
    reg  [1:0]  sub;                    // UPS 写侧 2×2 子象素
    reg  [3:0]  wb_n, wb_i;             // 写回攒字
    reg  [255:0] out_buf [0:15];
    reg  [1:0]  q_i;                    // POOL2 窗口 4 字计数
    reg  [11:0] cit_taps_r;             // start 装载：cfg_cit*taps
    reg  [13:0] wjob_r;                 // start 装载：cfg_cit*taps*8
    reg  [31:0] u_tile_base, u_row_base;
    reg  [31:0] p2_tile_ib, p2_tile_ob;
    reg  [767:0] acc_g0;                // ng2 组 0 部分和（C_ACCSAVE 保存）

    // ---------------- 地址/循环组合量 ----------------
    wire [7:0]  taps = (cfg_k == 1) ? 8'd1 : 8'd9;
    // 【修 BUG-Y3T-NG2 (2026-10-17)】原式 `ng2 = (cfg_split < cfg_cit)`：
    //   cfg_split=0（常规单组模式，固件 y3t_job 的默认写法、y3t.h 注释亦如此）
    //   时仍判为"两级尺度"——而组 0 部分和的保存条件在 C_ACCSAVE 分支里是
    //   `it == cfg_split - 1`，split=0 时等于 `it == -1`（it 无符号）永不成立
    //   ⇒ acc_g0 恒 0 ⇒ requant 的 g0 拍读到 0 ⇒ 输出全 0（现象: [8] S2
    //   bad=全部、out_word 全 0、cyc_tot 虚高 30 拍/像素）。
    //   split 的真实语义: 0 = 单组；1..cit-1 = 两组（g0 = split 个 ich tile）。
    wire        ng2  = (cfg_split != 7'd0) && (cfg_split < cfg_cit);
    wire [19:0] plane_in  = cfg_H  * cfg_W;
    wire [19:0] plane_out = cfg_Ho * cfg_Wo;
    wire [13:0] wjob  = cfg_cit * taps * 8;       // 每 OCH tile 的权重 256b 字数
    wire [31:0] ldw_addr = cfg_wb + ot * wjob + cnt;

    wire [1:0] ky = (tap >= 6) ? 2'd2 : (tap >= 3) ? 2'd1 : 2'd0;
    wire [1:0] kx = tap - {ky, 1'b0} - ky;
    wire signed [11:0] iy = $signed({2'b0, oy}) * cfg_s + ((cfg_k == 1) ? 2'd0 : ky) - cfg_pad;
    wire signed [11:0] ix = $signed({2'b0, ox}) * cfg_s + ((cfg_k == 1) ? 2'd0 : kx) - cfg_pad;
    wire in_bounds = (iy >= 0) && (iy < cfg_H) && (ix >= 0) && (ix < cfg_W);
    wire        g1 = (it >= cfg_split);
    wire [6:0]  local_tile = g1 ? (it - cfg_split) : it;
    wire [31:0] act_base = g1 ? cfg_in1 : cfg_in0;
    wire [31:0] act_addr = act_base + local_tile * plane_in + iy * cfg_W + ix;

    // ---------------- mac_array（3 级流水，48bit 驻留累加，母本原样） ----------------
    reg  [255:0] mac_act;
    reg          mac_valid, mac_first;
    reg  [11:0]  mac_wsel;
    reg  [255:0] mac_act_r;
    reg          mac_valid_r, mac_first_r;
    wire         wbuf_we = (state == CW_LDW_CAP) && mem_ack && (cnt[2:0] == 3'd7);
    wire [11:0]  wbuf_waddr = cnt[12:3];
    wire [767:0] mac_acc;
    always @(posedge clk) begin
        if (wbuf_we) wbuf[wbuf_waddr] <= {mem_rdata, wfill[1791:0]};
        wbuf_q      <= wbuf[mac_wsel];
        mac_valid_r <= mac_valid;
        mac_first_r <= mac_first;
        mac_act_r   <= mac_act;
    end
    mac_array #(.OCH(16), .ICH(16), .NGROUP(1)) u_mac (
        .clk(clk), .rst_n(rst_n),
        .in_valid(mac_valid_r), .in_first(mac_first_r), .in_group(2'b00),
        .in_act(mac_act_r), .in_w(wbuf_q),
        .out_acc(mac_acc), .out_valid()
    );

    // ---------------- requant_pipe_y3t ----------------
    reg          rq_valid, rq_first, rq_last, rq_g;
    reg  [3:0]   rq_ch;
    wire         rq_out_valid;
    wire signed [15:0] rq_out_data;
    wire [6:0]   rq_ot = ot;                    // WGROUP=1：ot 即全局 OCH tile
    // ng2：g0 拍读 C_ACCSAVE 保存的组 0 部分和，g1 拍读 mac 驻留累加（母本 acc_src 语义）
    wire [767:0] acc_src = (ng2 && !rq_g) ? acc_g0 : mac_acc;
    wire signed [47:0] rq_acc = $signed(acc_src[rq_ch*48 +: 48]);
    wire [10:0]  midx = {rq_g, rq_ot[5:0], rq_ch};     // group*1024 + ot*16 + ch
    requant_pipe_y3t u_rq (
        .clk(clk), .rst_n(rst_n),
        .in_valid(rq_valid), .in_first(rq_first), .in_last(rq_last),
        .in_acc(rq_acc),
        .in_multM(multm_rom[midx]),
        .in_shiftM(rq_g ? cfg_shM1 : cfg_shM0),
        .in_bias_q13(bias_rom[{rq_ot[5:0], rq_ch}]),
        .in_multY(cfg_multY),
        .in_shiftY(cfg_shY),
        .in_bits16(cfg_bits16), .in_act(cfg_act),
        .out_valid(rq_out_valid), .out_data(rq_out_data)
    );

    // ---------------- POOL2：lane 级有符号取大 ----------------
    function signed [15:0] smax2;
        input signed [15:0] a;
        input signed [15:0] b;
        smax2 = (a > b) ? a : b;
    endfunction
    function [255:0] lane_max2;
        input [255:0] a;
        input [255:0] b;
        integer i;
        begin
            for (i = 0; i < 16; i = i + 1)
                lane_max2[i*16 +: 16] = smax2(a[i*16 +: 16], b[i*16 +: 16]);
        end
    endfunction
    // 窗口第 q_i 字输入地址：(2oy+q_i[1], 2ox+q_i[0])，同输入通道 tile
    wire [9:0]  p2_iy = {oy, 1'b0} + {9'b0, q_i[1]};
    wire [9:0]  p2_ix = {ox, 1'b0} + {9'b0, q_i[0]};
    wire [31:0] p2_raddr = cfg_in0 + p2_tile_ib + p2_iy * cfg_W + p2_ix;
    wire [31:0] p2_waddr = cfg_out + p2_tile_ob + oy * cfg_Wo + ox;

    // ---------------- 写回 ----------------
    wire [31:0] wr_addr_w = cfg_out + rq_ot * plane_out + oy * cfg_Wo + ox;
    wire        wr_segf_w = (wr_addr_w[3:0] == 4'd15);   // 16 字对齐段末 → 整包发
    wire        wr_eol_w  = (ox == cfg_Wo - 1);          // 行尾 → 短包

    // ---------------- 主 FSM ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= E_IDLE; done <= 1'b0; err <= 1'b0;
            mq_req <= 0; mq_we <= 0; mq_addr <= 0; mq_blen <= 0; mq_wdata <= 0;
            ot <= 0; it <= 0; tile <= 0; oy <= 0; ox <= 0; cnt <= 0; tap <= 0;
            rq_odx <= 0; rq_active <= 0; fc <= 0; sub <= 0; q_i <= 0;
            mac_valid <= 0; mac_act <= 0; mac_first <= 0; mac_wsel <= 0;
            wb_n <= 0; wb_i <= 0;
            rq_valid <= 0; rq_first <= 0; rq_last <= 0; rq_g <= 0; rq_ch <= 0;
            out_word <= 0; word_a <= 0; acc_g0 <= 0;
            cyc_mac <= 0; cyc_tot <= 0;
            cit_taps_r <= 0; wjob_r <= 0;
            u_tile_base <= 0; u_row_base <= 0;
            p2_tile_ib <= 0; p2_tile_ob <= 0;
        end else begin
            mac_valid <= 0;
            rq_valid  <= 0;
            mq_req    <= 0;
            mq_we     <= 0;
            mq_blen   <= 0;              // 单拍脉冲（同 mem_req）：Y3T 恒单字读

            if (state != E_IDLE) cyc_tot <= cyc_tot + 32'd1;
            if (mac_valid)       cyc_mac <= cyc_mac + 32'd1;

            // requant 输出流逐通道入 out_word（rq_active 窗口内）
            if (rq_active && rq_out_valid) begin
                out_word[rq_odx*16 +: 16] <= rq_out_data;
                rq_odx <= rq_odx + 1;
            end

            case (state)
            E_IDLE: begin
                done <= 0; err <= 0;
                if (start) begin
                    ot <= 0; it <= 0; tile <= 0; oy <= 0; ox <= 0;
                    cnt <= 0; tap <= 0; sub <= 0; q_i <= 0;
                    wb_n <= 0; wb_i <= 0;
                    cyc_mac <= 0; cyc_tot <= 0;
                    cit_taps_r  <= cfg_cit * taps;
                    wjob_r      <= cfg_cit * taps * 8;
                    u_tile_base <= 0; u_row_base <= 0;
                    p2_tile_ib <= 0; p2_tile_ob <= 0;
                    case (cfg_mode)
                    M_CONV: state <= CW_LDW_REQ;
                    M_UPS:  state <= U_RD_REQ;
                    M_POOL: state <= P2_RD;
                    default: begin err <= 1; state <= E_DONE; end
                    endcase
                end
            end
            E_DONE: begin
                done  <= 1;
                state <= E_IDLE;
            end
            // ==================================================== CONV
            CW_LDW_REQ: begin                // 权重 LDW：单字（Y3T 无 burst）
                mq_req  <= 1;
                mq_addr <= ldw_addr;
                state   <= CW_LDW_CAP;
            end
            CW_LDW_CAP: begin
                if (mem_ack) begin
                    wfill[cnt[2:0]*256 +: 256] <= mem_rdata;   // 拼 8 字；第 8 字整字写 RAM
                    if (cnt == wjob_r - 1) begin               // 本 ot tile 权重读完
                        cnt <= 0; it <= 0; tap <= 0;
                        state <= CT_REQ;
                    end else begin
                        cnt   <= cnt + 1;
                        state <= CW_LDW_REQ;
                    end
                end
            end
            CT_REQ: begin                    // 激活 tap 字请求（pad 不发 req）
                if (in_bounds) begin
                    mq_req  <= 1;
                    mq_addr <= act_addr;
                end
                state <= CT_CAP;
            end
            CT_CAP: begin
                if (!in_bounds || mem_ack) begin
                    mac_valid <= 1;
                    mac_first <= (tap == 0) && ((it == 0) || (it == cfg_split));
                    mac_wsel  <= it * taps + tap;
                    mac_act   <= in_bounds ? mem_rdata : 256'd0;
                    if (tap == taps - 1) begin
                        tap <= 0;
                        if (ng2 && it == cfg_split - 1) begin
                            state <= C_ACCSAVE;
                        end else if (it == cfg_cit - 1) begin
                            state <= C_ACCWAIT;
                        end else begin
                            it    <= it + 1;
                            state <= CT_REQ;
                        end
                    end else begin
                        tap   <= tap + 1;
                        state <= CT_REQ;
                    end
                end
            end
            C_ACCSAVE: state <= C_ACCSAVE2;  // mac 3 级流水：等 out_acc 稳定（母本 v5 配平）
            C_ACCSAVE2: state <= C_ACCSAVE3;
            C_ACCSAVE3: state <= C_ACCSAVE4;
            C_ACCSAVE4: state <= C_ACCSAVE5;
            C_ACCSAVE5: begin                // g0 部分和保存 → 组 1（in_first）重新累加
                acc_g0 <= mac_acc;
                it     <= it + 1;
                state  <= CT_REQ;
            end
            C_ACCWAIT: begin                 // 末 it mac 已发射；等沉降后进 RQ
                fc        <= 0;
                rq_odx    <= 0;
                rq_active <= 1;
                state     <= C_RQ_WAIT;
            end
            C_RQ_WAIT:  state <= C_RQ_WAIT2;
            C_RQ_WAIT2: state <= C_RQ;
            C_RQ: begin                      // 每输出字 16 拍（ng2：32 拍 (g,ch) 交错）
                rq_valid <= 1;
                rq_g     <= ng2 ? fc[0] : 1'b0;
                rq_ch    <= ng2 ? fc[4:1] : fc[3:0];
                rq_first <= ng2 ? ~fc[0] : 1'b1;
                rq_last  <= ng2 ?  fc[0] : 1'b1;
                if (fc == (ng2 ? 6'd31 : 6'd15)) state <= C_RQ_DRAIN;
                fc <= fc + 1;
            end
            C_RQ_DRAIN: begin
                if (rq_odx == 16) begin
                    rq_active <= 0;
                    state     <= C_WR_REQ;
                end
            end
            // P8-A3：写回按 16 字对齐段攒 burst（恒不跨 4KB）；行尾/段满/不可攒时发送
            C_WR_REQ: begin
                out_buf[wb_n] <= out_word;
                if (wr_segf_w || wr_eol_w) begin
                    mq_req   <= 1;
                    mq_we    <= 1;
                    mq_addr  <= wr_addr_w - {28'd0, wb_n};   // 本包首字地址
                    mq_blen  <= {4'b0, wb_n};
                    mq_wdata <= (wb_n == 4'd0) ? out_word : out_buf[0];
                    wb_i      <= 4'd0;
                    state     <= C_WR_WAIT;
                end else begin
                    wb_n  <= wb_n + 4'd1;
                    state <= C_WR_NEXT;
                end
            end
            C_WR_WAIT: begin
                if (mem_ack) begin
                    if (wb_i == wb_n) begin
                        wb_n  <= 4'd0;            // 末 ack → 推进像素
                        state <= C_WR_NEXT;
                    end else begin
                        wb_i      <= wb_i + 4'd1; // W beat ack → 换下一字
                        mq_wdata <= out_buf[wb_i + 4'd1];
                    end
                end
            end
            C_WR_NEXT: begin
                it <= 0;                       // 像素切换：输入 tile 归零
                if (ox != cfg_Wo - 1) begin
                    ox    <= ox + 1;
                    state <= CT_REQ;
                end else begin
                    ox <= 0;
                    if (oy == cfg_Ho - 1) begin
                        oy <= 0;
                        if (ot + 1 >= cfg_cot) state <= E_DONE;
                        else begin
                            ot    <= ot + 1;
                            cnt   <= 0;
                            state <= CW_LDW_REQ;
                        end
                    end else begin
                        oy    <= oy + 1;
                        state <= CT_REQ;
                    end
                end
            end
            // ==================================================== UPSAMPLE
            U_RD_REQ: begin
                mq_req  <= 1;
                mq_addr <= cfg_in0 + u_tile_base + u_row_base + ox;
                state   <= U_RD_CAP;
            end
            U_RD_CAP: if (mem_ack) begin word_a <= mem_rdata; sub <= 0; state <= U_WR_REQ; end
            U_WR_REQ: begin
                mq_req   <= 1;
                mq_we    <= 1;
                // tile*(plane*4)+(oy*2+sub1)*(W*2)+ox*2+sub0 = 基址<<2 + 分量（母本恒等式）
                mq_addr  <= cfg_out + ((u_tile_base + u_row_base) << 2)
                            + (sub[1] ? {cfg_W, 1'b0} : 32'd0) + {ox, 1'b0} + sub[0];
                mq_wdata <= word_a;
                state    <= U_WR_WAIT;
            end
            U_WR_WAIT: begin
                if (mem_ack) begin
                    if (sub == 3) begin
                        if (ox == cfg_W - 1) begin
                            ox <= 0;
                            if (oy == cfg_H - 1) begin
                                oy <= 0; u_row_base <= 0;
                                if (tile == cfg_tiles - 1) state <= E_DONE;
                                else begin tile <= tile + 1; u_tile_base <= u_tile_base + plane_in; state <= U_RD_REQ; end
                            end else begin oy <= oy + 1; u_row_base <= u_row_base + cfg_W; state <= U_RD_REQ; end
                        end else begin ox <= ox + 1; state <= U_RD_REQ; end
                    end else begin
                        sub   <= sub + 1;
                        state <= U_WR_REQ;
                    end
                end
            end
            // ==================================================== POOL2（2×2 s2）
            P2_RD: begin
                mq_req  <= 1;
                mq_addr <= p2_raddr;
                state   <= P2_CAP;
            end
            P2_CAP: begin
                if (mem_ack) begin
                    out_word <= (q_i == 2'd0) ? mem_rdata
                                              : lane_max2(out_word, mem_rdata);
                    if (q_i == 2'd3) begin
                        q_i   <= 2'd0;
                        state <= P2_WR_REQ;
                    end else begin
                        q_i   <= q_i + 2'd1;
                        state <= P2_RD;
                    end
                end
            end
            P2_WR_REQ: begin
                mq_req   <= 1;
                mq_we    <= 1;
                mq_addr  <= p2_waddr;
                mq_wdata <= out_word;
                state    <= P2_WR_WAIT;
            end
            P2_WR_WAIT: begin
                if (mem_ack) begin
                    if (ox == cfg_Wo - 1) begin
                        ox <= 0;
                        if (oy == cfg_Ho - 1) begin
                            oy <= 0;
                            if (tile == cfg_tiles - 1) state <= E_DONE;
                            else begin
                                tile <= tile + 1;
                                p2_tile_ib <= p2_tile_ib + plane_in;
                                p2_tile_ob <= p2_tile_ob + plane_out;
                                state <= P2_RD;
                            end
                        end else begin
                            oy <= oy + 1;
                            state <= P2_RD;
                        end
                    end else begin
                        ox    <= ox + 1;
                        state <= P2_RD;
                    end
                end
            end
            default: state <= E_IDLE;
            endcase
        end
    end

    assign dbg_state = state;
endmodule
