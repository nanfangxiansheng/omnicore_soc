// ============================================================================
// fusion_top.v — OmniCore SoC + 双目相机 + DDR3 + Y3T 推理 + HDMI 叠框显示
//                (ZIGUANG 融合工程顶层)
//
// 组成 (参考 demo_ov5640 已验证视频链 + ominicore boardsoc SoC):
//   clk_hdmi : 27M → pclk(74.25M) + pclkx5(371M)   [原样沿用, PnR 已验证]
//   clk_soc  : 27M → soc_clk(54M)                  [新增, CPU/Y3T 域]
//   DDR_200H : DDR3 控制器 (125M 差分参考), AXI256 用户口, core_clk
//   双目     : camera×2 (官方 reg_config 配置 RGB565) → video_scale_down (1280×720→640×720)
//              → frame_read_write ×2 (乒乓 4 帧槽) → mem_arbi ch0/ch1
//   SoC 桥   : boardsoc_top 内 ddr_bridge_ahb @0x8000_0000 → mem_arbi ch2
//   显示     : sync_vg(720p@60) → video_rect_read_data ×2 (左右半屏) →
//              overlay_boxes (检测框叠画, 寄存器@0x4000_6000) → rgb2tmds
//   SoC      : boardsoc_top (CPU+Y3T+外设) @54MHz
//
// CPU 每帧流程 (固件): vsync 同步 → 经 0x8000_0000 桥读相机帧 ROI →
//   量化 → Y3T 29 job (引擎专用口) → 检测框坐标/颜色写 0x4000_6000 → HDMI 叠框
//
// 帧缓冲布局 (256bit 字地址, 同官方 demo 立体缩放版): 8 个连续槽, 帧长 28800 块,
//   槽距 57600 (一整帧余量, 防 vsync 竞态越界):
//   cam0 帧 n@0/57600/115200/172800, cam1 帧 n@230400/288000/345600/403200。
//   总占用 432000 块 = 13.8MB, 远在 DDR 的 256MB (AXI 28bit) 寻址窗内。
//   注: 早期误用 2073600/8294400 起步的槽距 —— 既非官方口径, 且 cam1 三个槽
//   越过 256MB 发生地址回卷 (见 §八f / BOARD_DEBUG_2026-10-15.md)。
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module fusion_top #(
	parameter MEM_ROW_ADDR_WIDTH   = 15         ,
	parameter MEM_COL_ADDR_WIDTH   = 10         ,
	parameter MEM_BADDR_WIDTH      = 3          ,
	parameter MEM_DQ_WIDTH         =  32        ,
	parameter MEM_DQS_WIDTH        =  32/8
)(
	input   wire                                sys_clk        ,   // 27M
	input   wire                                clk_p          ,   // 125M 差分 (DDR 参考)
	input   wire                                clk_n          ,
	input   wire                                rst_in         ,
	// DDR3
	output  wire                                mem_rst_n      ,
	output  wire                                mem_ck         ,
	output  wire                                mem_ck_n       ,
	output  wire                                mem_cke        ,
	output  wire                                mem_cs_n       ,
	output  wire                                mem_ras_n      ,
	output  wire                                mem_cas_n      ,
	output  wire                                mem_we_n       ,
	output  wire                                mem_odt        ,
	output  wire    [MEM_ROW_ADDR_WIDTH-1:0]    mem_a          ,
	output  wire    [MEM_BADDR_WIDTH-1:0]       mem_ba         ,
	inout   wire    [MEM_DQ_WIDTH/8-1:0]        mem_dqs        ,
	inout   wire    [MEM_DQ_WIDTH/8-1:0]        mem_dqs_n      ,
	inout   wire    [MEM_DQ_WIDTH-1:0]          mem_dq         ,
	output  wire    [MEM_DQ_WIDTH/8-1:0]        mem_dm         ,
	output  reg                                 heart_beat_led ,
	// 双目相机
	input   wire                                camera_clk_0   ,
	input   wire    [7:0]                       camera_data_0  ,
	input   wire                                camera_href_0  ,
	input   wire                                camera_vsync_0 ,
	output  wire                                SCL_0          ,
	inout   wire                                SDA_0          ,
	output  wire                                cam_rst_0      ,
	input   wire                                camera_clk_1   ,
	input   wire    [7:0]                       camera_data_1  ,
	input   wire                                camera_href_1  ,
	input   wire                                camera_vsync_1 ,
	output  wire                                SCL_1          ,
	inout   wire                                SDA_1          ,
	output  wire                                cam_rst_1      ,
	// HDMI (MS7210: 并行 RGB888 + PCLK 输入, 芯片内串化; I2C 初始化)
	output  wire                                pixclk_out     ,
	output  wire                                vs_out         ,
	output  wire                                hs_out         ,
	output  wire                                de_out         ,
	output  wire    [7:0]                       r_out          ,
	output  wire    [7:0]                       g_out          ,
	output  wire    [7:0]                       b_out          ,
	output  wire                                hd_scl         ,
	inout   wire                                hd_sda         ,
	// ZIGUANG 修复 (2026-10-14): MS7210 硬件复位 (G25, 低有效) —— 原工程
	// 漏接, MS7210 可能滞留复位态导致 HDMI 无信号。对齐 demo_ov5640。
	output  wire                                hd_rstn        ,
	output  reg                                 cam_led0       ,
	output  reg                                 cam_led1       ,
	// SoC 外设
	input   wire                                uart_rxd       ,
	output  wire                                uart_txd       ,
	/* v65 (2026-10-27) GPIO 引脚绑定 —— 依据《MES2L676-200HP 硬件使用手册 v1.2》
	 *   §3.4.1 按键: KEY0=C22(rst_in, 已占) KEY1=C23 KEY2=B22 KEY3=A22 KEY4=B20
	 *   §3.4.2 LED : LED0=A20(cam_led0) LED1=C18(hb) LED2=C19(cam_led1)
	 *                LED3=E18 LED4=A17 LED5=A18 LED6=C17 LED7=B17  ← 全空, 本次取用
	 * 按键低有效(未按=高, 按下=低), 所以"按下"是**下降沿**; gpio_in[0] 用下降沿中断。
	 * gpio_out 只引出 5 位 = 类别码[3:0] + det_valid, 对应 LED3..LED7。 */
	input   wire                                gpio_key1      ,   /* C23: 外部触发 (下降沿中断) */
	input   wire                                gpio_key2      ,   /* B22: 阈值模式 (0=0.25 1=0.15) */
	output  wire    [4:0]                       gpio_out       ,   /* E18/A17/A18/C17/B17 = LED3..LED7 */
	/* v65: 原 led[15:0] 端口的 5 个引脚(LED3..LED7)已让给 GPIO 输出，端口随之删除。
	 *   bring-up 诊断(phy_pll_lock/gpll_lock/ddrphy_cpd_lock/ddr_init_done)改由 UART 自检行查看。*/
	output  wire                                exit_req       ,
	output  wire [7:0]                          exit_code
);

	parameter CTRL_ADDR_WIDTH = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH;
	parameter TH_1S = 27'd33_000_000;   // ≈1s @ core_clk

	wire ddr_init_done;                 // DDR3 校准完成
	wire core_clk;                      // DDR 用户时钟

	//=====================================================================================
	// 时钟与复位
	//=====================================================================================
	wire pclk, pclkx5, soc_clk, soc_lock, hdmi_lock, rst_n;
	/* v65 (2026-10-27): 复位分域 -------------------------------------------------
	 *   rst_pll_n : 只等 PLL 锁定 —— 给 **DDR 控制器 / 相机 / 整条显示链** 用。
	 *   rst_n     : rst_pll_n & rst_in(复位键) —— 只给 SoC(CPU/外设) 用。
	 * 原来两者是同一个信号, 按一下复位键会连带复位 DDR_200H 与两路相机：
	 *   DDR 重新校准 -> ddr_init_done 掉低 -> frame_read_write 被复位 -> 显示停止取数;
	 *   相机重新跑 I2C 配置(5~8s) -> 写侧停止 -> 屏幕停在最后一帧（用户看到"图像暂停"）。
	 * 分域后: 复位键只重启软件, 采集与显示不停, 画面不冻结。 */
	wire rst_pll_n;

	clk_hdmi clk_hdmi_inst (
		.clkout0 (pclk      ),   // ≈74.25M
		.clkout1 (pclkx5    ),   // ≈371M (TMDS 串化)
		.lock    (hdmi_lock ),
		.clkin1  (sys_clk   )
	);

	clk_soc clk_soc_inst (
		.clkout0 (soc_clk    ),   // ≈54.05M
		.lock    (soc_lock   ),
		.clkin1  (sys_clk    )
	);

	// ZIGUANG (2026-10-14): 恢复正常复位表达式 —— 根因是 clk_soc PLL 倍率配错
	// (见 ipcore/clk_soc/clk_soc.v 修复注释), soc_lock 假 0 曾把全系统钳死在复位。
		assign rst_pll_n = hdmi_lock & soc_lock;      /* v65: 视频/DDR 域, 不按键 */
		assign rst_n     = rst_pll_n & rst_in;         /* v65: SoC 域, 受复位键控制 */

	// cam0 vsync -> soc_clk 2FF 同步 + 上升沿脉冲 (帧同步中断源)
	reg cam_vs_s1, cam_vs_s2, cam_vs_s3;
	always @(posedge soc_clk) begin
		if (!rst_n) begin
			cam_vs_s1 <= 1'b0; cam_vs_s2 <= 1'b0; cam_vs_s3 <= 1'b0;
		end else begin
			cam_vs_s1 <= camera_vsync_0;
			cam_vs_s2 <= cam_vs_s1;
			cam_vs_s3 <= cam_vs_s2;
		end
	end
	wire cam_frame_evt = cam_vs_s2 & ~cam_vs_s3;

	//=====================================================================================
	// 相机 (双目, 官方 reg_config 配置引擎, RGB565 输出)
	//=====================================================================================
	wire            cam_vsync_0, cam_href_0, cam_write_en_0, cam_init_done_0;
	wire    [15:0]  cam_data_0_raw;
	wire            cam_vsync_1, cam_href_1, cam_write_en_1, cam_init_done_1;
	wire    [15:0]  cam_data_1_raw;

	/* ================= v47 ★ 相机 R/B 通道修正 =================
	 * 板上实测：相机输出的 16bit 数据虽然在 RTL 里按 {R[4:0],G[5:0],B[4:0]}
	 * 解释，但实际字节顺序是 BGR565 —— 表现为
	 *   ① HDMI 显示：现实红色显示成蓝色；
	 *   ② 模型输入：PP 把"B"当 R 写进 lane0 ⇒ 网络吃到 BGR，彩色物体判错
	 *      （原先靠交换 L0 权重 ich0/ich2 补偿，属于软件绕行）。
	 * 这里在**唯一源头**交换 R/B，使显示与预处理同时得到正确 RGB。
	 * 回退：把 CAM_SWAP_RB 改 0 即可（同时要用原始权重、不用 BGR 版）。
	 */
	localparam CAM_SWAP_RB = 1'b1;
	wire [15:0] cam_data_0 = CAM_SWAP_RB ?
		{ cam_data_0_raw[4:0], cam_data_0_raw[10:5], cam_data_0_raw[15:11] } : cam_data_0_raw;
	wire [15:0] cam_data_1 = CAM_SWAP_RB ?
		{ cam_data_1_raw[4:0], cam_data_1_raw[10:5], cam_data_1_raw[15:11] } : cam_data_1_raw;
	/* ============================================================ */

	camera camera_inst0(
		.clk           ( sys_clk           )  ,
		.rst_n         ( rst_pll_n         )  ,   // v65: 相机不随复位键重启(否则停 5~8s)
		.SCL           ( SCL_0             )  ,
		.SDA           ( SDA_0             )  ,
		.camera_clk    ( camera_clk_0      )  ,
		.camera_data   ( camera_data_0     )  ,
		.camera_herf   ( camera_href_0     )  ,
		.camera_vsync  ( camera_vsync_0    )  ,
		.ddr_init      ( ddr_init_done     )  ,
		.camera_rstn   ( cam_rst_0         )  ,
		.camera_pwnd   (                   )  ,
		.init_done     ( cam_init_done_0   )  ,
		.wf_wr_data    ( cam_data_0_raw    )  ,   // RGB565 (v47 后在下面交换 R/B)
		.wf_wr_en      ( cam_write_en_0    )  ,
		.vs            ( cam_vsync_0       )  ,
		.hs            ( cam_href_0        )  ,
		.sop           (                   )  ,
		.eop           (                   )
	);

	camera camera_inst1(
		.clk           ( sys_clk              )  ,
		.rst_n         ( rst_pll_n & cam_init_done_0 )  ,   // v65
		.SCL           ( SCL_1                )  ,
		.SDA           ( SDA_1                )  ,
		.camera_clk    ( camera_clk_1         )  ,
		.camera_data   ( camera_data_1        )  ,
		.camera_herf   ( camera_href_1        )  ,
		.camera_vsync  ( camera_vsync_1       )  ,
		.ddr_init      ( ddr_init_done        )  ,
		.camera_rstn   ( cam_rst_1            )  ,
		.camera_pwnd   (                      )  ,
		.init_done     ( cam_init_done_1      )  ,
		.wf_wr_data    ( cam_data_1_raw       )  ,   // RGB565 (v47 后在下面交换 R/B)
		.wf_wr_en      ( cam_write_en_1       )  ,
		.vs            ( cam_vsync_1          )  ,
		.hs            ( cam_href_1           )  ,
		.sop           (                      )  ,
		.eop           (                      )
	);

	// 相机心跳 LED (24M 计数翻转)
	reg [27:0] cam_cnt_0, cam_cnt_1;
	reg        cam_cnt_flag_0, cam_cnt_flag_1;
	always @(posedge camera_clk_0) begin
		if (!rst_n) begin cam_cnt_0 <= 28'd0; cam_cnt_flag_0 <= 1'b0; end
		else if (cam_cnt_0 == 28'd24_000_000) begin cam_cnt_0 <= 28'd0; cam_cnt_flag_0 <= 1'b1; end
		else begin cam_cnt_0 <= cam_cnt_0 + 28'd1; cam_cnt_flag_0 <= 1'b0; end
	end
	always @(posedge camera_clk_1) begin
		if (!rst_n) begin cam_cnt_1 <= 28'd0; cam_cnt_flag_1 <= 1'b0; end
		else if (cam_cnt_1 == 28'd24_000_000) begin cam_cnt_1 <= 28'd0; cam_cnt_flag_1 <= 1'b1; end
		else begin cam_cnt_1 <= cam_cnt_1 + 28'd1; cam_cnt_flag_1 <= 1'b0; end
	end
	always @(posedge camera_clk_0) begin
		if (!rst_n) cam_led0 <= 1'b1;
		else if (cam_cnt_flag_0) cam_led0 <= ~cam_led0;
	end
	always @(posedge camera_clk_1) begin
		if (!rst_n) cam_led1 <= 1'b1;
		else if (cam_cnt_flag_1) cam_led1 <= ~cam_led1;
	end

	//=====================================================================================
	// 缩放: 1280×720 → 640×720 (左右半屏)
	//=====================================================================================
	wire [15:0] write_data_0, write_data_1;
	wire        write_en_0, write_en_1;

	video_scale_down #(.PIX_DATA_WIDTH(16)) video_scale_down_inst0(
		.video_clk            ( camera_clk_0    ),
		.rst_n                ( rst_pll_n       ),   // v65
		.frame_sync_n         ( ~cam_vsync_0    ),
		.video_data_in        ( cam_data_0      ),
		.video_data_valid     ( cam_write_en_0  ),
		.video_data_out       ( write_data_0    ),
		.video_data_out_valid ( write_en_0      ),
		.video_ready          ( 1'b1            ),
		.video_width_in       ( 1280            ),
		.video_height_in      ( 720             ),
		.video_width_out      ( 640             ),
		.video_height_out     ( 720             )
	);
	video_scale_down #(.PIX_DATA_WIDTH(16)) video_scale_down_inst1(
		.video_clk            ( camera_clk_1    ),
		.rst_n                ( rst_pll_n       ),   // v65
		.frame_sync_n         ( ~cam_vsync_1    ),
		.video_data_in        ( cam_data_1      ),
		.video_data_valid     ( cam_write_en_1  ),
		.video_data_out       ( write_data_1    ),
		.video_data_out_valid ( write_en_1      ),
		.video_ready          ( 1'b1            ),
		.video_width_in       ( 1280            ),
		.video_height_in      ( 720             ),
		.video_width_out      ( 640             ),
		.video_height_out     ( 720             )
	);

	// 乒乓帧槽序号
	wire        write_req_0, write_req_ack_0;
	wire [1:0]  write_addr_index_0, read_addr_index_0;
	wire        write_req_1, write_req_ack_1;
	wire [1:0]  write_addr_index_1, read_addr_index_1;

	cmos_write_req_gen cmos_write_req_gen_inst0(
		.rst             ( ~rst_pll_n             ),
		.pclk            ( camera_clk_0       ),
		.cmos_vsync      ( cam_vsync_0        ),
		.write_req       ( write_req_0        ),
		.write_addr_index( write_addr_index_0 ),
		.read_addr_index ( read_addr_index_0  ),
		.write_req_ack   ( write_req_ack_0    )
	);
	cmos_write_req_gen cmos_write_req_gen_inst1(
		.rst             ( ~rst_pll_n             ),
		.pclk            ( camera_clk_1       ),
		.cmos_vsync      ( cam_vsync_1        ),
		.write_req       ( write_req_1        ),
		.write_addr_index( write_addr_index_1 ),
		.read_addr_index ( read_addr_index_1  ),
		.write_req_ack   ( write_req_ack_1    )
	);

	//=====================================================================================
	// 720p@60 时序
	//=====================================================================================
	wire            vs_out_sync, hs_out_sync, de_out_sync;
	wire            read_req_0, read_req_ack_0, read_en_0;
	wire [15:0]     read_data_0;
	wire            read_req_1, read_req_ack_1, read_en_1;
	wire [15:0]     read_data_1;
	wire            vs_out_0, hs_out_0, de_out_0;
	wire            vs_out_1, hs_out_1, de_out_1;
	wire [15:0]     vout_data_0, vout_data_1;

	sync_vg #(
		.X_BITS(4'd12), .Y_BITS(4'd12),
		.V_TOTAL(12'd750), .V_FP(12'd5), .V_BP(12'd20), .V_SYNC(12'd5), .V_ACT(12'd720),
		.H_TOTAL(12'd1650), .H_FP(12'd110), .H_BP(12'd220), .H_SYNC(12'd40), .H_ACT(12'd1280)
	) sync_vg_inst (
		.clk    ( pclk        ),
		.rstn   ( rst_pll_n   ),   // v65
		.vs_out ( vs_out_sync ),
		.hs_out ( hs_out_sync ),
		.de_out ( de_out_sync ),
		.x_act  (             ),
		.y_act  (             )
	);

	/* v52: 读侧像素延迟级数（由 overlay 寄存器 0x6C 提供，必须在实例化前声明）*/
	wire [6:0]  ovl_xshift;      /* v52: overlay 寄存器 0x6C 给出的读侧像素延迟级数 */
/* v63: 行缓存要在读实例处就接 DDR 读 beat, 这 4 根网线必须在这里先声明
 *     （本文件 default_nettype=none, 不能先用后声明）*/
wire                        rd_burst_data_valid_0, rd_burst_data_valid_1;
wire [255:0]                rd_burst_data_0,       rd_burst_data_1;   /* 位宽 = MEM_DATA_BITS(256), 参数在本行之后才定义, 故用字面值 */
	wire [4:0]  ovl_xdelay;      /* v55: overlay 寄存器 0x68 给出的显示时序延迟级数 */
	wire        ovl_lb_en;       /* v63: overlay 寄存器 0x64 给出的行缓存使能(默认1) */
	wire [8:0]  ovl_lb_ofs;      /* v64: overlay 寄存器 0x64[9:1] 的槽位线性偏移(一档16px) */
	video_rect_read_data video_rect_read_data_inst0(
		.video_clk        ( pclk           ),
		.rst              ( ~rst_pll_n         ),
		.video_left_offset( 0              ),
		/* v63: 行缓存 —— 直接把 DDR 读 beat(256b) 灌进去, 按地址对齐显示 */
		.lb_en            ( ovl_lb_en                ),
		.lb_ofs           ( ovl_lb_ofs               ),   // v64: 槽位偏移
		.lb_wr_clk        ( core_clk                 ),
		.lb_wr_rst        ( read_req_0 | ~ddr_init_done ),
		.lb_wr_data       ( rd_burst_data_0          ),
		.lb_wr_valid      ( rd_burst_data_valid_0    ),
		.video_top_offset ( 0              ),
		.video_width      ( 640            ),
		.video_height     ( 720            ),
		.x_shift          ( ovl_xshift     ),
		.x_delay          ( ovl_xdelay     ),
		.read_req         ( read_req_0     ),
		.read_req_ack     ( read_req_ack_0 ),
		.read_en          ( read_en_0      ),
		.read_data        ( read_data_0    ),
		.timing_hs        ( hs_out_sync    ),
		.timing_vs        ( vs_out_sync    ),
		.timing_de        ( de_out_sync    ),
		.timing_data      ( 16'd0          ),
		.hs               ( hs_out_0       ),
		.vs               ( vs_out_0       ),
		.de               ( de_out_0       ),
		.vout_data        ( vout_data_0    )
	);
	video_rect_read_data video_rect_read_data_inst1(
		.video_clk        ( pclk           ),
		.rst              ( ~rst_pll_n         ),
		.video_left_offset( 640            ),
		.lb_en            ( ovl_lb_en                ),
		.lb_ofs           ( ovl_lb_ofs               ),   // v64: 槽位偏移
		.lb_wr_clk        ( core_clk                 ),
		.lb_wr_rst        ( read_req_1 | ~ddr_init_done ),
		.lb_wr_data       ( rd_burst_data_1          ),
		.lb_wr_valid      ( rd_burst_data_valid_1    ),
		.video_top_offset ( 0              ),
		.video_width      ( 640            ),
		.video_height     ( 720            ),
		.x_shift          ( ovl_xshift     ),
		.x_delay          ( ovl_xdelay     ),
		.read_req         ( read_req_1     ),
		.read_req_ack     ( read_req_ack_1 ),
		.read_en          ( read_en_1      ),
		.read_data        ( read_data_1    ),
		.timing_hs        ( hs_out_0       ),
		.timing_vs        ( vs_out_0       ),
		.timing_de        ( de_out_0       ),
		.timing_data      ( vout_data_0    ),
		.hs               ( hs_out_1       ),
		.vs               ( vs_out_1       ),
		.de               ( de_out_1       ),
		.vout_data        ( vout_data_1    )
	);

	//=====================================================================================
	// 叠框 (检测框, 寄存器由 SoC @0x4000_6000 配置)
	//=====================================================================================
	wire        ovl_hs, ovl_vs, ovl_de;
	wire [15:0] ovl_data;
	wire        ovl_sel, ovl_hready, ovl_nseq, ovl_write;
	wire [5:0]  ovl_addr;
	wire [31:0] ovl_wdata, ovl_hrdata;

	overlay_boxes #(
		.NBOX      ( 8  ),
		.BORDER_PX ( 3'd2 )
	) overlay_boxes_inst (
		.ahb_clk  ( soc_clk     ),
		.ahb_rst_n( rst_n       ),
		.hsel     ( ovl_sel     ),
		.hready   ( ovl_hready  ),
		.htrans_nseq ( ovl_nseq ),
		// ZIGUANG 修复 (2026-10-08): 原写法 {2'b00, ovl_addr} 拼接后截到 6 位
		// 端口会丢掉 ovl_addr[1:0] (=s_haddr[3:2]), overlay 寄存器选址全部
		// 错位。ovl_addr 本身就是 s_haddr[7:2], 直连即可。
		.haddr    ( ovl_addr ),
		.hwrite   ( ovl_write   ),
		.hsize    ( 3'b010      ),
		.hwdata   ( ovl_wdata   ),
		.hreadyout(             ),   // 恒 1, 已在 SoC mux 处理
		.hrdata   ( ovl_hrdata  ),
		.hresp    (             ),
		.pclk     ( pclk        ),
		.rst_n    ( rst_n       ),
		.in_hs    ( hs_out_1    ),
		.in_vs    ( vs_out_1    ),
		.in_de    ( de_out_1    ),
		.in_data  ( vout_data_1 ),
		.frame_idx( {read_addr_index_1, read_addr_index_0} ),
		.x_shift_o( ovl_xshift    ),
		.x_delay_o( ovl_xdelay    ),   // v55: 显示时序延迟 (竖缝主旋钮)
		.lb_en_o  ( ovl_lb_en     ),   // v63: 行缓存使能 (0x64, 默认 1)
		.lb_ofs_o ( ovl_lb_ofs    ),   // v64: 槽位偏移 (0x64[9:1])
		.out_hs   ( ovl_hs      ),
		.out_vs   ( ovl_vs      ),
		.out_de   ( ovl_de      ),
		.out_data ( ovl_data    )
	);

	//=====================================================================================
	// 视频输出: 并行 RGB888 + 同步 + PCLK → MS7210 (板卡 HDMI TX 为芯片串化,
	// 手册 §3.3; 引脚 N21/T25/P19... = HD_TX_*。TMDS 直出不适用本板!)
	//=====================================================================================
	assign vs_out    = ovl_vs;
	assign hs_out    = ovl_hs;
	assign de_out    = ovl_de;
	// RGB565 → RGB888
	assign r_out = {ovl_data[15:11], ovl_data[15:13]};
	assign g_out = {ovl_data[10:5],  ovl_data[10:9]};
	assign b_out = {ovl_data[4:0],   ovl_data[4:2]};
	assign pixclk_out = pclk;

	// MS7210 初始化 (官方 iic_dri + ms7210_ctl, 53 条表, 器件 0xB2, soc_clk 域)
	// hd_rstn: 上电延迟 ~37ms 释放 (sys_clk 27M x 1e6), 同 demo_ov5640;
	// 同时驱动 G25 引脚 (MS7210 硬件复位) 与本工程配置逻辑复位。
	reg  [26:0] hdmi_rst_cnt = 27'd0;
	always @(posedge sys_clk) begin
		if (hdmi_rst_cnt != 27'd1_000_000)
			hdmi_rst_cnt <= hdmi_rst_cnt + 1'b1;
	end
	assign hd_rstn = (hdmi_rst_cnt == 27'd1_000_000);

	wire        ms_init_over, ms_iic_trig, ms_w_r, ms_busy, ms_byte_over;
	wire [15:0] ms_addr;
	wire [7:0]  ms_data_in, ms_data_out;
	wire        ms_sda_in, ms_sda_out, ms_sda_out_en;

	ms7210_ctl ms7210_ctl_inst (
		.clk        ( soc_clk      ),
		.rstn       ( hd_rstn      ),
		.init_over  ( ms_init_over ),
		.device_id  (              ),   // 模块内固定 8'hB2
		.iic_trig   ( ms_iic_trig  ),
		.w_r        ( ms_w_r       ),
		.addr       ( ms_addr      ),
		.data_in    ( ms_data_in   ),
		.busy       ( ms_busy      ),
		.data_out   ( ms_data_out  ),
		.byte_over  ( ms_byte_over )
	);

	iic_dri #(
		.CLK_FRE   ( 27'd54_000_000 ),
		.IIC_FREQ  ( 20'd400_000    ),
		.T_WR      ( 10'd1          ),
		.ADDR_BYTE ( 2'd2           ),
		.LEN_WIDTH ( 8'd3           ),
		.DATA_BYTE ( 2'd1           )
	) u_iic_dri (
		.clk        ( soc_clk      ),
		.rstn       ( hd_rstn      ),
		.pluse      ( ms_iic_trig  ),
		.device_id  ( 8'hB2        ),
		.w_r        ( ms_w_r       ),
		.byte_len   ( 4'd1         ),
		.addr       ( ms_addr      ),
		.data_in    ( ms_data_in   ),
		.busy       ( ms_busy      ),
		.byte_over  ( ms_byte_over ),
		.data_out   ( ms_data_out  ),
		.scl        ( hd_scl       ),
		.sda_in     ( hd_sda       ),
		.sda_out    ( ms_sda_out   ),
		.sda_out_en ( ms_sda_out_en )
	);
	assign hd_sda = ms_sda_out_en ? ms_sda_out : 1'bz;


	//=====================================================================================
	// DDR 读写通道 (256bit 突发用户口)
	//=====================================================================================
	parameter MEM_DATA_BITS = 256;
	parameter ADDR_BITS     = 25;
	parameter BUSRT_BITS    = 10;

	wire                        wr_burst_req;
	wire [BUSRT_BITS-1:0]       wr_burst_len;
	wire [ADDR_BITS-1:0]        wr_burst_addr;
	wire                        wr_burst_data_req;
	wire [MEM_DATA_BITS-1:0]    wr_burst_data;
	wire                        wr_burst_finish;

	wire                        rd_burst_req;
	wire [BUSRT_BITS-1:0]       rd_burst_len;
	wire [ADDR_BITS-1:0]        rd_burst_addr;
	wire                        rd_burst_data_valid;
	wire [MEM_DATA_BITS-1:0]    rd_burst_data;
	wire                        rd_burst_finish;

	wire                        wr_burst_req_0, wr_burst_finish_0, wr_burst_data_req_0;
	wire [BUSRT_BITS-1:0]       wr_burst_len_0;
	wire [ADDR_BITS-1:0]        wr_burst_addr_0;
	wire [MEM_DATA_BITS-1:0]    wr_burst_data_0;
	wire                        rd_burst_req_0, rd_burst_finish_0;
	wire [BUSRT_BITS-1:0]       rd_burst_len_0;
	wire [ADDR_BITS-1:0]        rd_burst_addr_0;

	wire                        wr_burst_req_1, wr_burst_finish_1, wr_burst_data_req_1;
	wire [BUSRT_BITS-1:0]       wr_burst_len_1;
	wire [ADDR_BITS-1:0]        wr_burst_addr_1;
	wire [MEM_DATA_BITS-1:0]    wr_burst_data_1;
	wire                        rd_burst_req_1, rd_burst_finish_1;
	wire [BUSRT_BITS-1:0]       rd_burst_len_1;
	wire [ADDR_BITS-1:0]        rd_burst_addr_1;

	// SoC DDR 桥通道 (ch2)
	wire                        soc_rd_burst_req, soc_rd_burst_finish, soc_rd_burst_data_valid;
	wire [BUSRT_BITS-1:0]       soc_rd_burst_len;
	wire [ADDR_BITS-1:0]        soc_rd_burst_addr;
	wire [MEM_DATA_BITS-1:0]    soc_rd_burst_data;
	wire                        soc_wr_burst_req, soc_wr_burst_finish, soc_wr_burst_data_req;
	wire [BUSRT_BITS-1:0]       soc_wr_burst_len;
	wire [ADDR_BITS-1:0]        soc_wr_burst_addr;
	wire [MEM_DATA_BITS-1:0]    soc_wr_burst_data;

	// ZIGUANG 修复 (2026-10-08): ch3 原本悬空 —— 接 boardsoc 权重加载 DMA
	// (wload) 的 core 侧读通道; 原先该组端口在例化处未连接, 权重加载永远
	// 无法完成。eng 侧 (wld_eng_*) 为调试引出, 悬空即可。
	/* 【2026-10-20 21:10】给 wload 读通道 5 根线加 syn_keep。
	 *   原因: 综合后只有 req/len/addr/finish 保留了名字, 唯独 `data_valid`
	 *   因为"只连 arbiter 输出 → wload 输入两点"被直接合并、名字丢失
	 *   (同样用途的 ch2 那根因为还有别的负载才留下来)。
	 *   Fabric Inserter 按名字找不到它 ⇒ ILA 抓不到"每 burst 收到几个数据",
	 *   而这一条正是判断"控制器少给/多给"的唯一依据。
	 *   syn_keep 让综合保留这 5 根线的名字与实体, 供 Inserter 观测。
	 *   (Verilator 会忽略该属性, 不影响仿真。) */
	(* syn_keep = 1 *) wire                        ch3_rd_burst_req;
	(* syn_keep = 1 *) wire [BUSRT_BITS-1:0]       ch3_rd_burst_len;
	(* syn_keep = 1 *) wire [ADDR_BITS-1:0]        ch3_rd_burst_addr;
	(* syn_keep = 1 *) wire                        ch3_rd_burst_data_valid;
	(* syn_keep = 1 *) wire                        ch3_rd_burst_finish;
	wire [MEM_DATA_BITS-1:0]    ch3_rd_burst_data;   /* 256 位, 不抓, 不加 keep */
	/* ZIGUANG (2026-10-18 21:40): ch3 写通道原来是常量 0 —— CMD 直通需要
	 *   "DDR→y3tram 的同时把同一份数据写回 DDR", 所以这一路现在接
	 *   boardsoc 里 wload_dma 的写回口 (wld_wr_burst_*)。
	 *   写通道与读通道共用 ch3 的仲裁槽, mem_write_arbi 各自轮询, 互不冲突。 */
	wire                        ch3_wr_burst_req;
	wire [BUSRT_BITS-1:0]       ch3_wr_burst_len;
	wire [ADDR_BITS-1:0]        ch3_wr_burst_addr;
	wire                        ch3_wr_burst_data_req;
	wire [MEM_DATA_BITS-1:0]    ch3_wr_burst_data;
	wire                        ch3_wr_burst_finish;

	frame_read_write #(
		.MEM_DATA_BITS  (256  ),
		.READ_DATA_BITS (16   ),
		.WRITE_DATA_BITS(16   ),
		.ADDR_BITS      (25   ),
		.BUSRT_BITS     (10   ),
		.BURST_SIZE     (16   )
	) frame_read_write_inst0(
		.rst                 ( ~ddr_init_done       ),
		.mem_clk             ( core_clk             ),
		.rd_burst_req        ( rd_burst_req_0       ),
		.rd_burst_len        ( rd_burst_len_0       ),
		.rd_burst_addr       ( rd_burst_addr_0      ),
		.rd_burst_data_valid ( rd_burst_data_valid_0),
		.rd_burst_data       ( rd_burst_data_0      ),
		.rd_burst_finish     ( rd_burst_finish_0    ),
		.read_clk            ( pclk                 ),
		.read_req            ( read_req_0           ),
		.read_req_ack        ( read_req_ack_0       ),
		.read_finish         (                      ),
		// ZIGUANG 修复 (2026-10-15): 帧槽地址改回官方 demo 立体缩放版口径。
		// 原 2073600/4147200/6220800 是从 demo 注释
		// "The second frame address is 24'd2073600" 误抄的示例值(非真实步进),
		// 造成 4 个帧槽被 66MB 空洞隔开; 且 cam1 的槽(见下)越过 256MB AXI
		// 地址窗 → 地址回卷 → 花屏。官方 Top.v.720p.bak 立体缩放版为
		// 槽距 57600 = 一整帧余量, 帧长 28800(640x720x2B/32B)。
		.read_addr_0         ( 25'd0                ),
		.read_addr_1         ( 25'd57600            ),
		.read_addr_2         ( 25'd115200           ),
		.read_addr_3         ( 25'd172800           ),
		.read_addr_index     ( read_addr_index_0    ),
		.read_len            ( 25'd28800            ),   // ZIGUANG: 帧长=28800 块 (demo 720p 官方值); 原 129600 越界写致花屏
		.read_en             ( read_en_0            ),
		.read_data           ( read_data_0          ),
		.wr_burst_req        ( wr_burst_req_0       ),
		.wr_burst_len        ( wr_burst_len_0       ),
		.wr_burst_addr       ( wr_burst_addr_0      ),
		.wr_burst_data_req   ( wr_burst_data_req_0  ),
		.wr_burst_data       ( wr_burst_data_0      ),
		.wr_burst_finish     ( wr_burst_finish_0    ),
		.write_clk           ( camera_clk_0         ),
		.write_req           ( write_req_0          ),
		.write_req_ack       ( write_req_ack_0      ),
		.write_finish        (                      ),
		.write_addr_0        ( 25'd0                ),
		.write_addr_1        ( 25'd57600            ),
		.write_addr_2        ( 25'd115200           ),
		.write_addr_3        ( 25'd172800           ),
		.write_addr_index    ( write_addr_index_0   ),
		.write_len           ( 25'd28800            ),   // ZIGUANG: 帧长=28800 块 (demo 720p 官方值); 原 129600 越界写致花屏
		.write_en            ( write_en_0           ),
		.write_data          ( write_data_0         )
	);

	frame_read_write #(
		.MEM_DATA_BITS  (256  ),
		.READ_DATA_BITS (16   ),
		.WRITE_DATA_BITS(16   ),
		.ADDR_BITS      (25   ),
		.BUSRT_BITS     (10   ),
		.BURST_SIZE     (16   )
	) frame_read_write_inst1(
		.rst                 ( ~ddr_init_done       ),
		.mem_clk             ( core_clk             ),
		.rd_burst_req        ( rd_burst_req_1       ),
		.rd_burst_len        ( rd_burst_len_1       ),
		.rd_burst_addr       ( rd_burst_addr_1      ),
		.rd_burst_data_valid ( rd_burst_data_valid_1),
		.rd_burst_data       ( rd_burst_data_1      ),
		.rd_burst_finish     ( rd_burst_finish_1    ),
		.read_clk            ( pclk                 ),
		.read_req            ( read_req_1           ),
		.read_req_ack        ( read_req_ack_1       ),
		.read_finish         (                      ),
		// ZIGUANG 修复 (2026-10-15): cam1 帧槽同理改连续口径 (230400 = 4x57600,
		// 与 cam0 共 8 个连续槽, 总占用 432000 块 = 13.8MB)。
		// 原 8294400 起步、槽距 2073600: frame1/2/3 (10368000/12441600/14515200)
		// 的字节地址 331/398/464MB 全部越过 DDR IP 的 28 位地址口(上限
		// 268435456B = 8388608 块) → 回卷到 1.9M/4.0M/6.1M 块, cam1 图像
		// 落到错误区域, 是花屏的直接来源。
		.read_addr_0         ( 25'd230400           ),
		.read_addr_1         ( 25'd288000           ),
		.read_addr_2         ( 25'd345600           ),
		.read_addr_3         ( 25'd403200           ),
		.read_addr_index     ( read_addr_index_1    ),
		.read_len            ( 25'd28800            ),   // ZIGUANG: 帧长=28800 块 (demo 720p 官方值); 原 129600 越界写致花屏
		.read_en             ( read_en_1            ),
		.read_data           ( read_data_1          ),
		.wr_burst_req        ( wr_burst_req_1       ),
		.wr_burst_len        ( wr_burst_len_1       ),
		.wr_burst_addr       ( wr_burst_addr_1      ),
		.wr_burst_data_req   ( wr_burst_data_req_1  ),
		.wr_burst_data       ( wr_burst_data_1      ),
		.wr_burst_finish     ( wr_burst_finish_1    ),
		.write_clk           ( camera_clk_1         ),
		.write_req           ( write_req_1          ),
		.write_req_ack       ( write_req_ack_1      ),
		.write_finish        (                      ),
		.write_addr_0        ( 25'd230400           ),
		.write_addr_1        ( 25'd288000           ),
		.write_addr_2        ( 25'd345600           ),
		.write_addr_3        ( 25'd403200           ),
		.write_addr_index    ( write_addr_index_1   ),
		.write_len           ( 25'd28800            ),   // ZIGUANG: 帧长=28800 块 (demo 720p 官方值); 原 129600 越界写致花屏
		.write_en            ( write_en_1           ),
		.write_data          ( write_data_1         )
	);

	//=====================================================================================
	// 读/写通道仲裁 (ch0=cam0, ch1=cam1, ch2=SoC 桥, ch3=空闲)
	//=====================================================================================
	mem_read_arbi #(
		.MEM_DATA_BITS(MEM_DATA_BITS),
		.ADDR_BITS    (ADDR_BITS    ),
		.BUSRT_BITS   (BUSRT_BITS   )
	) mem_read_arbi_inst(
		.rst_n                   ( ddr_init_done ),
		.mem_clk                 ( core_clk      ),
		.ch0_rd_burst_req        ( rd_burst_req_0        ),
		.ch0_rd_burst_len        ( rd_burst_len_0        ),
		.ch0_rd_burst_addr       ( rd_burst_addr_0       ),
		.ch0_rd_burst_data_valid ( rd_burst_data_valid_0 ),
		.ch0_rd_burst_data       ( rd_burst_data_0       ),
		.ch0_rd_burst_finish     ( rd_burst_finish_0     ),
		.ch1_rd_burst_req        ( rd_burst_req_1        ),
		.ch1_rd_burst_len        ( rd_burst_len_1        ),
		.ch1_rd_burst_addr       ( rd_burst_addr_1       ),
		.ch1_rd_burst_data_valid ( rd_burst_data_valid_1 ),
		.ch1_rd_burst_data       ( rd_burst_data_1       ),
		.ch1_rd_burst_finish     ( rd_burst_finish_1     ),
		.ch2_rd_burst_req        ( soc_rd_burst_req      ),
		.ch2_rd_burst_len        ( soc_rd_burst_len      ),
		.ch2_rd_burst_addr       ( soc_rd_burst_addr     ),
		.ch2_rd_burst_data_valid ( soc_rd_burst_data_valid ),
		.ch2_rd_burst_data       ( soc_rd_burst_data     ),
		.ch2_rd_burst_finish     ( soc_rd_burst_finish   ),
		.ch3_rd_burst_req        ( ch3_rd_burst_req      ),
		.ch3_rd_burst_len        ( ch3_rd_burst_len      ),
		.ch3_rd_burst_addr       ( ch3_rd_burst_addr     ),
		.ch3_rd_burst_data_valid ( ch3_rd_burst_data_valid ),
		.ch3_rd_burst_data       ( ch3_rd_burst_data     ),
		.ch3_rd_burst_finish     ( ch3_rd_burst_finish   ),
		.rd_burst_req            ( rd_burst_req          ),
		.rd_burst_len            ( rd_burst_len          ),
		.rd_burst_addr           ( rd_burst_addr         ),
		.rd_burst_data_valid     ( rd_burst_data_valid   ),
		.rd_burst_data           ( rd_burst_data         ),
		.rd_burst_finish         ( rd_burst_finish       )
	);

	mem_write_arbi #(
		.MEM_DATA_BITS(MEM_DATA_BITS),
		.ADDR_BITS    (ADDR_BITS    ),
		.BUSRT_BITS   (BUSRT_BITS   )
	) mem_write_arbi_inst(
		.rst_n                 ( ddr_init_done ),
		.mem_clk               ( core_clk      ),
		.ch0_wr_burst_req      ( wr_burst_req_0      ),
		.ch0_wr_burst_len      ( wr_burst_len_0      ),
		.ch0_wr_burst_addr     ( wr_burst_addr_0     ),
		.ch0_wr_burst_data_req ( wr_burst_data_req_0 ),
		.ch0_wr_burst_data     ( wr_burst_data_0     ),
		.ch0_wr_burst_finish   ( wr_burst_finish_0   ),
		.ch1_wr_burst_req      ( wr_burst_req_1      ),
		.ch1_wr_burst_len      ( wr_burst_len_1      ),
		.ch1_wr_burst_addr     ( wr_burst_addr_1     ),
		.ch1_wr_burst_data_req ( wr_burst_data_req_1 ),
		.ch1_wr_burst_data     ( wr_burst_data_1     ),
		.ch1_wr_burst_finish   ( wr_burst_finish_1   ),
		.ch2_wr_burst_req      ( soc_wr_burst_req      ),
		.ch2_wr_burst_len      ( soc_wr_burst_len      ),
		.ch2_wr_burst_addr     ( soc_wr_burst_addr     ),
		.ch2_wr_burst_data_req ( soc_wr_burst_data_req ),
		.ch2_wr_burst_data     ( soc_wr_burst_data     ),
		.ch2_wr_burst_finish   ( soc_wr_burst_finish   ),
		.ch3_wr_burst_req      ( ch3_wr_burst_req      ),
		.ch3_wr_burst_len      ( ch3_wr_burst_len      ),
		.ch3_wr_burst_addr     ( ch3_wr_burst_addr     ),
		.ch3_wr_burst_data_req ( ch3_wr_burst_data_req ),
		.ch3_wr_burst_data     ( ch3_wr_burst_data     ),
		.ch3_wr_burst_finish   ( ch3_wr_burst_finish   ),
		.wr_burst_req          ( wr_burst_req          ),
		.wr_burst_len          ( wr_burst_len          ),
		.wr_burst_addr         ( wr_burst_addr         ),
		.wr_burst_data_req     ( wr_burst_data_req     ),
		.wr_burst_data         ( wr_burst_data         ),
		.wr_burst_finish       ( wr_burst_finish       )
	);

	//=====================================================================================
	// AXI 主 (突发用户口 → DDR_200H AXI256)
	//=====================================================================================
	wire [CTRL_ADDR_WIDTH-1:0] axi_awaddr;
	wire [3:0]                 axi_awid;
	wire [3:0]                 axi_awlen;
	wire                       axi_awready, axi_awvalid;
	wire [MEM_DQ_WIDTH*8-1:0]  axi_wdata;
	wire [MEM_DQ_WIDTH*8/8-1:0] axi_wstrb;
	wire                       axi_wready;
	wire [CTRL_ADDR_WIDTH-1:0] axi_araddr;
	wire [3:0]                 axi_arid;
	wire [3:0]                 axi_arlen;
	wire                       axi_arready, axi_arvalid;
	wire [MEM_DQ_WIDTH*8-1:0]  axi_rdata;
	wire [3:0]                 axi_rid;
	wire                       axi_rlast, axi_rvalid;
	wire axi_bvalid;
	assign axi_bvalid = 1'b1;

	aq_axi_master_256 u_aq_axi_master (
		.ARESETN       ( ddr_init_done ),
		.ACLK          ( core_clk      ),
		.M_AXI_AWID    ( axi_awid   ),
		.M_AXI_AWADDR  ( axi_awaddr ),
		.M_AXI_AWLEN   ( axi_awlen  ),
		.M_AXI_AWSIZE  (            ),
		.M_AXI_AWBURST (            ),
		.M_AXI_AWLOCK  (            ),
		.M_AXI_AWCACHE (            ),
		.M_AXI_AWPROT  (            ),
		.M_AXI_AWQOS   (            ),
		.M_AXI_AWUSER  (            ),
		.M_AXI_AWVALID ( axi_awvalid ),
		.M_AXI_AWREADY ( axi_awready ),
		.M_AXI_WDATA   ( axi_wdata  ),
		.M_AXI_WSTRB   ( axi_wstrb  ),
		.M_AXI_WLAST   (            ),
		.M_AXI_WUSER   (            ),
		.M_AXI_WVALID  (            ),
		.M_AXI_WREADY  ( axi_wready ),
		.M_AXI_BID     (            ),
		.M_AXI_BRESP   (            ),
		.M_AXI_BUSER   (            ),
		.M_AXI_BVALID  ( axi_bvalid ),
		.M_AXI_BREADY  (            ),
		.M_AXI_ARID    ( axi_arid   ),
		.M_AXI_ARADDR  ( axi_araddr ),
		.M_AXI_ARLEN   ( axi_arlen  ),
		.M_AXI_ARSIZE  (            ),
		.M_AXI_ARBURST (            ),
		.M_AXI_ARLOCK  (            ),
		.M_AXI_ARCACHE (            ),
		.M_AXI_ARPROT  (            ),
		.M_AXI_ARQOS   (            ),
		.M_AXI_ARUSER  (            ),
		.M_AXI_ARVALID ( axi_arvalid ),
		.M_AXI_ARREADY ( axi_arready ),
		.M_AXI_RID     ( axi_rid    ),
		.M_AXI_RDATA   ( axi_rdata  ),
		.M_AXI_RRESP   (            ),
		.M_AXI_RLAST   ( axi_rlast  ),
		.M_AXI_RUSER   (            ),
		.M_AXI_RVALID  ( axi_rvalid ),
		.M_AXI_RREADY  (            ),
		.MASTER_RST    ( 1'b0         ),
		.WR_START      ( wr_burst_req ),
		.WR_ADRS       ( {wr_burst_addr, 5'd0} ),
		.WR_LEN        ( {wr_burst_len, 5'd0}  ),
		.WR_READY      (              ),
		.WR_FIFO_RE    ( wr_burst_data_req ),
		.WR_FIFO_EMPTY ( 1'b0         ),
		.WR_FIFO_AEMPTY( 1'b0         ),
		.WR_FIFO_DATA  ( wr_burst_data ),
		.WR_DONE       ( wr_burst_finish ),
		.RD_START      ( rd_burst_req ),
		.RD_ADRS       ( {rd_burst_addr, 5'd0} ),
		.RD_LEN        ( {rd_burst_len, 5'd0}  ),
		.RD_READY      (              ),
		.RD_FIFO_WE    ( rd_burst_data_valid ),
		.RD_FIFO_FULL  ( 1'b0         ),
		.RD_FIFO_AFULL ( 1'b0         ),
		.RD_FIFO_DATA  ( rd_burst_data ),
		.RD_DONE       ( rd_burst_finish ),
		.DEBUG         (              )
	);

	//=====================================================================================
	// DDR3 控制器/PHY
	//=====================================================================================
	wire pll_lock, phy_pll_lock, gpll_lock, rst_gpll_lock, ddrphy_cpd_lock;
	wire clk_125Mhz;
	// ZIGUANG 上板诊断 (2026-10-13): DDR init 链状态 LED 化, 板上直读卡点
	wire dbg_ddr_init_fail;
	wire [15:0] soc_led_dummy;

	/* v65: GPIO 与板级引脚的连接 (高位补 0, 避免未使用位被综合成悬空 IO) */
	wire [15:0] gpio_in_w  = {14'd0, gpio_key2, gpio_key1};
	wire [15:0] gpio_out_w;
	assign gpio_out = gpio_out_w[4:0];

	GTP_INBUFGDS #(
		.IOSTANDARD("DEFAULT"),
		.TERM_DIFF("ON")
	) u_gtp (
		.O (clk_125Mhz),
		.I (clk_p),
		.IB(clk_n)
	);

	DDR_200H DDR_200H_inst (
		.ref_clk       ( clk_125Mhz    ),
		.resetn        ( rst_pll_n     ),   // v65: DDR 不再被复位键复位(否则重校准导致画面冻结)
		.ddr_init_done ( ddr_init_done ),
		.pll_lock      ( pll_lock      ),
		.core_clk      ( core_clk      ),
		.phy_pll_lock  ( phy_pll_lock  ),
		.gpll_lock     ( gpll_lock     ),
		.rst_gpll_lock ( rst_gpll_lock ),
		.ddrphy_cpd_lock( ddrphy_cpd_lock ),
		.axi_awaddr    ( axi_awaddr    ),
		.axi_awuser_ap ( 1'b0          ),
		.axi_awuser_id ( axi_awid      ),
		.axi_awlen     ( axi_awlen     ),
		.axi_awready   ( axi_awready   ),
		.axi_awvalid   ( axi_awvalid   ),
		.axi_wdata     ( axi_wdata     ),
		.axi_wstrb     ( axi_wstrb     ),
		.axi_wready    ( axi_wready    ),
		.axi_wusero_id (               ),
		.axi_wusero_last(              ),
		.axi_araddr    ( axi_araddr    ),
		.axi_aruser_ap ( 1'b0          ),
		.axi_aruser_id ( axi_arid      ),
		.axi_arlen     ( axi_arlen     ),
		.axi_arready   ( axi_arready   ),
		.axi_arvalid   ( axi_arvalid   ),
		.axi_rdata     ( axi_rdata     ),
		.axi_rid       ( axi_rid       ),
		.axi_rlast     ( axi_rlast     ),
		.axi_rvalid    ( axi_rvalid    ),
		.apb_clk       ( 1'b0          ),
		.apb_rst_n     ( 1'b1          ),
		.apb_sel       ( 1'b0          ),
		.apb_enable    ( 1'b0          ),
		.apb_addr      ( 8'b0          ),
		.apb_write     ( 1'b0          ),
		.apb_ready     (               ),
		.apb_wdata     ( 16'b0         ),
		.apb_rdata     (               ),
		.mem_rst_n     ( mem_rst_n     ),
		.mem_ck        ( mem_ck        ),
		.mem_ck_n      ( mem_ck_n      ),
		.mem_cke       ( mem_cke       ),
		.mem_cs_n      ( mem_cs_n      ),
		.mem_ras_n     ( mem_ras_n     ),
		.mem_cas_n     ( mem_cas_n     ),
		.mem_we_n      ( mem_we_n      ),
		.mem_odt       ( mem_odt       ),
		.mem_a         ( mem_a         ),
		.mem_ba        ( mem_ba        ),
		.mem_dqs       ( mem_dqs       ),
		.mem_dqs_n     ( mem_dqs_n     ),
		.mem_dq        ( mem_dq        ),
		.mem_dm        ( mem_dm        ),
		.dbg_gate_start         ( 1'b0 ),
		.dbg_cpd_start          ( 1'b0 ),
		.dbg_ddrphy_rst_n       ( 1'b1 ),
		.dbg_gpll_scan_rst      ( 1'b0 ),
		.samp_position_dyn_adj  ( 1'b0 ),
		.init_samp_position_even( 32'd0 ),
		.init_samp_position_odd ( 32'd0 ),
		.wrcal_position_dyn_adj ( 1'b0 ),
		.init_wrcal_position    ( 32'd0 ),
		.force_read_clk_ctrl    ( 1'b0 ),
		.init_slip_step         ( 16'd0 ),
		.init_read_clk_ctrl     ( 12'd0 ),
		.debug_calib_ctrl       ( ),
		.dbg_slice_status       ( ),
		.dbg_slice_state        ( ),
		.debug_data             ( ),
		.dbg_dll_upd_state      ( ),
		.debug_gpll_dps_phase   ( ),
		.dbg_rst_dps_state      ( ),
		.dbg_tran_err_rst_cnt   ( ),
		.dbg_ddrphy_init_fail   ( dbg_ddr_init_fail ),
		.debug_cpd_offset_adj   ( 1'b0 ),
		.debug_cpd_offset_dir   ( 1'b0 ),
		.debug_cpd_offset       ( 10'd0 ),
		.debug_dps_cnt_dir0     ( ),
		.debug_dps_cnt_dir1     ( ),
		.ck_dly_en              ( 1'b0 ),
		.init_ck_dly_step       ( 8'h0 ),
		.ck_dly_set_bin         ( ),
		.align_error            ( ),
		.debug_rst_state        ( ),
		.debug_cpd_state        ( )
	);

	//=====================================================================================
	// OmniCore SoC (CPU + Y3T + 外设)
	//=====================================================================================
	ominicore_boardsoc_top #(
		.CLK_HZ    ( 54_054_054 ),   // clk_soc 实际频率 (27.027M × 2)
		.UART_BAUD ( 460800     )   // 2026-10-22: 115200→460800, 上传提速 4 倍
		                             //   (54.054M/460800 = 117.3 → 固件整数分频 117,
		                             //    实际 462000, 误差 0.26% < 2% 可接受)
		                             //   上限受 USB-UART 桥限制: 最高 460800''
	) boardsoc (
		.clk            ( soc_clk    ),
		.rst_n          ( rst_n      ),
		.uart_rxd       ( uart_rxd   ),
		.uart_txd       ( uart_txd   ),
		.gpio_in        ( gpio_in_w  ),
		.gpio_out       ( gpio_out_w ),
		.led            ( soc_led_dummy ),
		.exit_req       ( exit_req   ),
		.exit_code      ( exit_code  ),
		.npu_hready     ( 1'b0       ),
		.npu_htrans     ( 2'b00      ),
		.npu_haddr      ( 32'h0      ),
		.npu_hwrite     ( 1'b0       ),
		.npu_hsize      ( 3'b0       ),
		.npu_hburst     ( 3'b0       ),
		.npu_hwdata     ( 32'h0      ),
		.npu_hreadyout  (            ),
		.npu_hrdata     (            ),
		.npu_hresp      (            ),
		.accel_irq      ( 1'b0       ),
		.ddr_core_clk   ( core_clk   ),
		.ddr_core_rst_n ( rst_n & ddr_init_done ),
		.ddr_rd_req     ( soc_rd_burst_req      ),
		.ddr_rd_len     ( soc_rd_burst_len      ),
		.ddr_rd_addr    ( soc_rd_burst_addr     ),
		.ddr_rd_valid   ( soc_rd_burst_data_valid ),
		.ddr_rd_data    ( soc_rd_burst_data     ),
		.ddr_rd_finish  ( soc_rd_burst_finish   ),
		.ddr_wr_req     ( soc_wr_burst_req      ),
		.ddr_wr_len     ( soc_wr_burst_len      ),
		.ddr_wr_addr    ( soc_wr_burst_addr     ),
		.ddr_wr_data_req( soc_wr_burst_data_req ),
		.ddr_wr_data    ( soc_wr_burst_data     ),
		.ddr_wr_finish  ( soc_wr_burst_finish   ),
		.ovl_sel        ( ovl_sel    ),
		.ovl_hready     ( ovl_hready ),
		.ovl_nseq       ( ovl_nseq   ),
		.ovl_addr       ( ovl_addr   ),
		.ovl_write      ( ovl_write  ),
		.ovl_wdata      ( ovl_wdata  ),
	.ovl_hrdata     ( ovl_hrdata ),
	.cam_frame_evt  ( cam_frame_evt ),

	// ZIGUANG (2026-10-19): 相机预处理 → y3tram IN 区
	//   直连 cam0 像素流 (video_scale_down 之前的 1280x720 原始流)
	.pp_cam_clk     ( camera_clk_0   ),
	.pp_vsync       ( cam_vsync_0    ),
	.pp_de          ( cam_write_en_0 ),
	.pp_pix         ( cam_data_0     ),
	// ZIGUANG 修复 (2026-10-08): 权重 DMA core 侧通道接读仲裁 ch3 (原先悬空)
	.wld_core_clk   ( core_clk    ),
	.wld_core_rst_n ( rst_n & ddr_init_done ),
	.wld_rd_burst_req  ( ch3_rd_burst_req  ),
	.wld_rd_burst_len  ( ch3_rd_burst_len  ),
	.wld_rd_burst_addr ( ch3_rd_burst_addr ),
	.wld_rd_burst_data_valid ( ch3_rd_burst_data_valid ),
	.wld_rd_burst_data ( ch3_rd_burst_data ),
	.wld_rd_burst_finish ( ch3_rd_burst_finish ),
	/* 2026-10-18: CMD 直通的 DDR 写回 (同 ch3 写通道; 见 wload_dma 头注释) */
	.wld_wr_burst_req      ( ch3_wr_burst_req      ),
	.wld_wr_burst_len      ( ch3_wr_burst_len      ),
	.wld_wr_burst_addr     ( ch3_wr_burst_addr     ),
	.wld_wr_burst_data_req ( ch3_wr_burst_data_req ),
	.wld_wr_burst_data     ( ch3_wr_burst_data     ),
	.wld_wr_burst_finish   ( ch3_wr_burst_finish   )
);

	//=====================================================================================
	// DDR 心跳
	//=====================================================================================
	// ZIGUANG 上板诊断 (2026-10-13): led[0..4] -> LED3..LED7 (高电平亮)
	//   LED3(E18)=init_fail  LED4(A17)=init_done  LED5(A18)=cpd_lock
	//   LED6(C17)=gpll_lock  LED7(B17)=phy_pll_lock
	// 2026-10-14: LED3 改接 CPU 活性 (boardsoc led[14]=dbg_running) —— 串口
	// 排障: LED3 常亮=CPU 在取指执行(问题在 UART 外围/波特率); 灭=CPU 死(查 TCM)。
	/* v65: led 诊断输出已下线（引脚让给 GPIO）；soc_led_dummy 仅保留给 boardsoc 的调试口 */
	reg [26:0] cnt;
	always @(posedge core_clk) begin
		if (!ddr_init_done) cnt <= 27'd0;
		else if (cnt >= TH_1S) cnt <= 27'd0;
		else cnt <= cnt + 27'd1;
	end
	always @(posedge core_clk) begin
		if (!ddr_init_done) heart_beat_led <= 1'd1;
		else if (cnt >= TH_1S) heart_beat_led <= ~heart_beat_led;
	end

endmodule

`default_nettype wire
