python fw_gen.py \
    --params demo_hw/y3t_params.json --bin demo_hw/y3t_int8.bin \
    --image /home/hcy/ziguang/ziguang/images/IMG_20260925_143535.jpg

# fw_gen.py — M3 固件工件生成器（CPU 预处理 → Y3T 引擎 → CPU 后处理）
#
# 输入: demo_hw/y3t_params.json + y3t_int8.bin（demo_hw_sim 产出的导出层表/权重）
#       + 一张源图（固件内置的"相机帧"，CPU 预处理输入）
# 输出:
#   sim/board/y3t_mem.hex   引擎 RAM 预载（仅权重区, @addr 稀疏; IN 帧区留零——
#                           帧由固件预处理后经 y3t_mem_ahb 写入）
#   sw/board/y3t_net.h      cfg 写流（非 ROM 对 + ROM 段表/值表, ROM_DATA 自增压缩）
#                           + job 边界 + 头输出解码元数据 + sigmoid 定点表
#   sw/board/y3t_img.h      源图 RGB565（SW_SRC_W×SW_SRC_H）