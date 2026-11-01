

/*****************************************************************************
|  ominicore_btb.v — 具有 2 位饱和计数器的 N 项分支目标缓冲器（BTB）                |
|                                                                             |
|  用于替换 ominicore_frontend 内单表项 BTB 的比赛升级版                          |

\*****************************************************************************/

`default_nettype none

module ominicore_btb #(
	parameter W_ADDR      = 32,
	parameter NUM_ENTRIES = 64,   // 2 的幂
	parameter W_CTR       = 2     // 每个表项一个 2 位饱和方向计数器
) (
	input  wire              clk,
	input  wire              rst_n,

	// 更新接口（由处理器 X 级驱动）
	input  wire              btb_set,
	input  wire [W_ADDR-1:0] btb_set_src_addr,
	input  wire              btb_set_src_size,
	input  wire [W_ADDR-1:0] btb_set_target_addr,
	input  wire              btb_clear,
	input  wire              btb_flush,   // ZIGUANG (BP-06): fence.i/陷阱 → 清全部有效位并重置计数器

	// 查找接口（组合逻辑，来自前端的取指地址）
	input  wire [W_ADDR-1:0] lookup_addr,
	output wire              lookup_hit,
	output wire              lookup_taken,  // ZIGUANG (C1): 方向 = 计数器 MSB (BP-03)
	output wire [W_ADDR-1:0] lookup_src_addr,
	output wire              lookup_src_size,
	output wire [W_ADDR-1:0] lookup_target_addr,

	// 统计抽头（连接到性能计数器单元）
	output wire              evt_alloc,
	output wire              evt_flush_all,
	output wire              evt_ctr_dec
);

	function integer clog2;
		input integer value;
		begin
			for (clog2 = 0; value > 1; clog2 = clog2 + 1)
				value = value >> 1;
		end
	endfunction

	localparam IDX_BITS = clog2(NUM_ENTRIES);
	localparam TAG_BITS = W_ADDR - 2 - IDX_BITS;

	// 索引与标签均以字为粒度（上游版本比较 fetch_addr[W_ADDR-1:2]）
	wire [IDX_BITS-1:0] lu_idx  = lookup_addr[2 +: IDX_BITS];
	wire [TAG_BITS-1:0] lu_tag  = lookup_addr[2 + IDX_BITS +: TAG_BITS];

	wire [IDX_BITS-1:0] upd_idx = btb_set_src_addr[2 +: IDX_BITS];
	wire [TAG_BITS-1:0] upd_tag = btb_set_src_addr[2 + IDX_BITS +: TAG_BITS];

	reg                  ent_valid  [0:NUM_ENTRIES-1];
	reg [TAG_BITS-1:0]   ent_tag    [0:NUM_ENTRIES-1];
	reg                  ent_lsb1   [0:NUM_ENTRIES-1]; // src_addr[1]: 跨越字边界的 32 位分支指令
	reg                  ent_size   [0:NUM_ENTRIES-1];
	reg [W_ADDR-1:0]     ent_target [0:NUM_ENTRIES-1];
	reg [W_CTR-1:0]      ent_ctr    [0:NUM_ENTRIES-1];

	integer i;

	// ------------------------------------------------------------------
	// 查找（组合逻辑，与取指地址相同拍）
	// ------------------------------------------------------------------
	assign lookup_hit = ent_valid[lu_idx] && (ent_tag[lu_idx] == lu_tag);
	// ZIGUANG (C1): 采用条件 = 计数器 taken 偏置 (BP-03)。衰减语义: 分配置
	// 11, 每次 fall-through 递减, 10 仍预测 (单次不 taken 不驱逐热循环分支),
	// 01 起停止预测, 00 出表 -- 原 a_dir 快照语义的数组内实现。
	assign lookup_taken = ent_ctr[lu_idx][W_CTR-1];

	// 重建完整的源半字地址。下游逻辑只消费
	// bit 1（悬出位）和字号（隐含在标签/索引中）。
	assign lookup_src_addr    = {ent_tag[lu_idx], lu_idx, ent_lsb1[lu_idx], 1'b0};
	assign lookup_src_size    = lookup_hit && ent_size[lu_idx];
	assign lookup_target_addr = ent_target[lu_idx];

	// ------------------------------------------------------------------
	// 更新
	// ------------------------------------------------------------------
	// 若 btb_clear 与误预测分支自身的 PC 同时出现（clear 期间 set_src_addr
	// 携带 d_pc）-> 仅做逐表项计数器递减。
	wire entry_exists       = ent_valid[upd_idx] && (ent_tag[upd_idx] == upd_tag);
	wire clear_is_mispredict = btb_clear && entry_exists;

	assign evt_alloc     = btb_set;
	assign evt_flush_all = btb_clear && !clear_is_mispredict;
	assign evt_ctr_dec   = clear_is_mispredict && (ent_ctr[upd_idx] != {W_CTR{1'b0}});

	always @ (posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			// 所有字段都复位，而不仅是有效位：查找数据通路是
			// 组合逻辑，未使用表项中的 X 会泄漏到
			// 前端的 btb_jumped_beyond（该信号并未按 hit 门控）。
			// ZIGUANG (BP-OPT): 复位/清空 for 循环统一用非阻塞赋值 --
			// PDS ADS 综合器(Verilog-4042)禁止同一变量混用两种赋值。
			// 纯复位写无块内读后写冒险, 语义等价; 若 verilator 4.038
			// 对 for 内非阻塞数组赋值报 BLKLOOPINIT, 需升级 verilator。
			for (i = 0; i < NUM_ENTRIES; i = i + 1) begin
				ent_valid[i]    <= 1'b0;
				ent_ctr[i]      <= 2'b01;   // ZIGUANG (BP-06): weakly-not-taken（弱不跳转）
				ent_tag[i]      <= {TAG_BITS{1'b0}};
				ent_lsb1[i]     <= 1'b0;
				ent_size[i]     <= 1'b0;
				ent_target[i]   <= {W_ADDR{1'b0}};
			end
		end else if (btb_flush) begin
			// ZIGUANG (BP-06): fence.i / 陷阱 → 清有效位并将计数器置
			// weakly-not-taken (2'b01); 与误预测衰减 (btb_clear) 区分
			for (i = 0; i < NUM_ENTRIES; i = i + 1) begin
				ent_valid[i] <= 1'b0;
				ent_ctr[i]   <= 2'b01;
			end
		end else begin
			if (btb_set) begin
				ent_valid[upd_idx]  <= 1'b1;
				ent_tag[upd_idx]    <= upd_tag;
				ent_lsb1[upd_idx]   <= btb_set_src_addr[1];
				ent_size[upd_idx]   <= btb_set_src_size;
				ent_target[upd_idx] <= btb_set_target_addr;
				ent_ctr[upd_idx]    <= {W_CTR{1'b1}};  // （重新）分配时置为强跳转
			end
			if (clear_is_mispredict && !btb_set) begin
				// 预测跳转的分支实际顺序执行了：施加迟滞（hysteresis）。
				// 只有计数器耗尽才淘汰表项，这样单次
				// 不跳转迭代不会搅掉热循环分支。
				if (ent_ctr[upd_idx] == {W_CTR{1'b0}})
					ent_valid[upd_idx] <= 1'b0;
				else
					ent_ctr[upd_idx] <= ent_ctr[upd_idx] - {{(W_CTR-1){1'b0}}, 1'b1};
			end
		end
	end

`ifdef OMNICORE_ASSERTIONS
	always @ (posedge clk) if (rst_n && ent_valid[lu_idx]) assert(ent_tag[lu_idx] === ent_tag[lu_idx]);
`endif

endmodule

`default_nettype wire
