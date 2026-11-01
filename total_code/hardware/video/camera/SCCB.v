`timescale 1ns / 1ps

module SCCB (
    input  wire         clk         ,   //时钟
    input  wire         rst_n       ,   //复位
    input  wire         sccb_start  ,   //开始信号
    input  wire [23:0]  data_in     ,   //高位地址 + 低位地址 + 数据
    output reg          SCL         ,   //
    output reg          SDA         ,   //
    output reg          sccb_done       //结束信号
);
parameter DEV_ADDR = 8'h78; //器件地址写   器件地址：7'h3c

parameter SYS_CLK  = 27_000_000;
parameter SCCB_CLK = 250_000   ;

localparam delay = SYS_CLK / SCCB_CLK;

localparam IDLE   = 7'b000_0001;    //空闲
localparam START  = 7'b000_0010;    //开始
localparam DEVICE = 7'b000_0100;    //器件地址写
localparam ADDR_H = 7'b000_1000;    //高位寄存器地址
localparam ADDR_L = 7'b001_0000;    //低位寄存器地址
localparam DATA   = 7'b010_0000;    //数据
localparam STOP   = 7'b100_0000;    //停止

reg [6:0] c_state,n_state;

reg [31:0] cnt_clk;
reg [7:0]  cnt_bit;


always @(posedge clk ) begin
    if(!rst_n)
        c_state <= IDLE;
    else
        c_state <= n_state;
end

always @(*) begin
    if(!rst_n)
        n_state = IDLE;
    else begin
        case (c_state)
                IDLE   :begin
                    if(sccb_start)
                        n_state = START;
                    else
                        n_state = IDLE;
                end
                START  :begin
                    if(cnt_clk == delay - 1)
                        n_state = DEVICE;
                    else
                        n_state = START;
                end
                DEVICE :begin
                    if(cnt_clk == delay - 1 && cnt_bit == 8)    //无应答机制，但是需要应答位，用来站位
                        n_state = ADDR_H;
                    else
                        n_state = DEVICE;
                end
                ADDR_H :begin
                    if(cnt_clk == delay - 1 && cnt_bit == 8)
                        n_state = ADDR_L;
                    else
                        n_state = ADDR_H;
                end
                ADDR_L :begin
                    if(cnt_clk == delay - 1 && cnt_bit == 8)
                        n_state = DATA;
                    else
                        n_state = ADDR_L;
                end
                DATA   :begin
                    if(cnt_clk == delay - 1 && cnt_bit == 8)
                        n_state = STOP;
                    else
                        n_state = DATA;
                end
                STOP   :begin
                    if(cnt_clk == delay - 1)
                        n_state = IDLE;
                    else
                        n_state = STOP;
                end
            default: n_state = IDLE;
        endcase
    end
end


always @(posedge clk ) begin
    if(!rst_n)begin
        cnt_clk <= 0;
        cnt_bit <= 0;
    end
    else begin
        case (c_state)
                START ,STOP  :begin
                    cnt_bit <= 0;
                    if(cnt_clk == delay - 1)
                        cnt_clk <= 0;
                    else
                        cnt_clk <= cnt_clk + 1;
                end
                DEVICE ,ADDR_H ,ADDR_L ,DATA:begin
                    if(cnt_clk == delay - 1)begin
                        cnt_clk <= 0;
                        if(cnt_bit == 8)
                            cnt_bit <= 0;
                        else
                            cnt_bit <= cnt_bit + 1;
                    end
                    else
                        cnt_clk <= cnt_clk + 1;
                end
            default: begin
                cnt_clk <= 0;
                cnt_bit <= 0;
            end
        endcase
    end
end

always @(posedge clk ) begin
    if(!rst_n)
        SCL <= 1;
    else begin
        case (c_state)
                START  :begin
                    if(cnt_clk < delay / 4 * 3 - 1)
                        SCL <= 1;
                    else
                        SCL <= 0;
                end
                DEVICE ,ADDR_H ,ADDR_L ,DATA:begin
                    if(cnt_clk > delay / 4 -1 && cnt_clk < delay / 4 * 3 - 1)
                        SCL <= 1;
                    else
                        SCL <= 0;
                end
                STOP   :begin
                    if(cnt_clk > delay / 4 - 1)
                        SCL <= 1;
                    else
                        SCL <= 0;
                end
            default: SCL <= 1;
        endcase
    end
end

always @(posedge clk ) begin
    if(!rst_n)
        SDA <= 1;
    else begin
        case (c_state)
                START  :begin
                    if(cnt_clk > delay / 4 - 1)
                        SDA <= 0;
                    else
                        SDA <= 1;
                end
                DEVICE :begin
                    if(cnt_bit < 8)
                        SDA <= DEV_ADDR[7 - cnt_bit];
                    else
                        SDA <= 0;
                end
                ADDR_H :begin
                    if(cnt_bit < 8)
                        SDA <= data_in[23 - cnt_bit];
                    else
                        SDA <= 0;
                end
                ADDR_L :begin
                    if(cnt_bit < 8)
                        SDA <= data_in[15 - cnt_bit];
                    else
                        SDA <= 0;
                end
                DATA   :begin
                    if(cnt_bit < 8)
                        SDA <= data_in[7 - cnt_bit];
                    else
                        SDA <= 0;
                end
                STOP   :begin
                    if(cnt_clk > delay / 4 * 3 - 1)
                        SDA <= 1;
                    else
                        SDA <= 0;
                end
            default: SDA <= 1;
        endcase
    end
end

always @(posedge clk ) begin
    if(!rst_n)
        sccb_done <= 0;
    else if(c_state == STOP && cnt_clk == delay - 2)
        sccb_done <= 1;
    else
        sccb_done <= 0;
end

    
endmodule