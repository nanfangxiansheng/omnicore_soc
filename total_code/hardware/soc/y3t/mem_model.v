// ============================================================================
// mem_model.v — 256bit 总线存储器模型（行为级，仅仿真）
// req 拉高一拍发起事务；LAT 拍后 ack=1，读时 rdata 有效，写时数据已落 mem。
// 预载由 testbench 用 $readmemh 层次化写入 mem 数组。
// Phase 1a：blen 突发读（0=单拍兼容；N>0 时首 ack 后连续 N 拍、每拍地址+1 一字）。
//   未接 blen 的旧实例化点（悬空 x）按 0 处理，行为与原完全一致。
// P8-3：blen 突发写（与读镜像）：req&&we 锁存 blen/地址，ack 每拍采样 wdata
//   写入 b_addr 并 +1；ack 总数恒 = blen+1（与 master "换字 ack + B ack" 计数一致）。
// ============================================================================
module mem_model #(
    parameter WORDS = 18720,
    parameter LAT   = 1
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         req,
    input  wire         we,
    input  wire [7:0]   blen,       // burst 长度-1（0=单拍；悬空按 0）
    input  wire [31:0]  addr,       // 256bit 字地址
    input  wire [255:0] wdata,
    output reg  [255:0] rdata,
    output reg          ack
);
    reg [255:0] mem [0:WORDS-1];

    reg [LAT:0] ack_sr;
    reg         rd_r;
    reg         wr_r;               // P8-3：写 burst 标记（req&&we 置位，读事务/复位清）
    reg  [7:0]  b_left;             // burst 首拍后剩余 ack 拍
    reg  [31:0] b_addr;             // burst 下一字地址
    wire [7:0]  blen_safe = (^blen === 1'bx) ? 8'd0 : blen;   // 悬空兼容

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ack_sr <= {(LAT+1){1'b0}};
            rdata  <= 256'd0;
            rd_r   <= 1'b0;
            wr_r   <= 1'b0;
            ack    <= 1'b0;
            b_left <= 8'd0;
            b_addr <= 32'd0;
        end else begin
            ack_sr <= {ack_sr[LAT-1:0], req};
            if (req && !we) begin
                rdata  <= mem[addr];
                b_left <= blen_safe;
                b_addr <= addr + 1;
                rd_r   <= 1'b1;
                wr_r   <= 1'b0;
            end else begin
                rd_r   <= 1'b0;
            end
            if (req && we) begin
                // P8-3 修正：首字也推迟到首 ack 拍写——engine 见首 ack 后才开始逐拍换字，
                // req 拍即写会采到 word0、续拍错位（mem[addr+k]=word k-1、末字丢失）。
                b_left <= blen_safe;   // P8-3：写 burst 同锁存（原单字行为 blen=0 不变）
                b_addr <= addr;
                wr_r   <= 1'b1;
            end
            if (LAT == 1) ack <= req;
            else          ack <= ack_sr[LAT-1];
            if (ack && wr_r) begin          // P8-3 写：每个 ack 拍采样 wdata 写当前字
                mem[b_addr] <= wdata;       // （engine 在上个 ack 后已换字，时序对齐）
                b_addr <= b_addr + 1;
                if (b_left != 0) begin
                    ack    <= 1'b1;         // 续拍 ack（总数恒 = blen+1）
                    b_left <= b_left - 1;
                end else begin
                    wr_r <= 1'b0;           // 末字写完，ack 下拍回落（ack<=req=0）
                end
            end else if (ack && b_left != 0) begin   // 读 burst 续拍（原逻辑不变）
                ack    <= 1'b1;
                b_left <= b_left - 1;
                b_addr <= b_addr + 1;
                rdata  <= mem[b_addr];
            end
        end
    end
endmodule
