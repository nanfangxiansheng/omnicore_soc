# software —— 软件部分交付源码

## 目录结构

```
software/
├── firmware/          片上固件 + arena 执行器源码（C, RV32IMAC, -nostdlib）
│   ├── boot.S / demo.c / uart_cmd.c / y3t_exec.c / y3t_detect.c / y3t_auto.c
│   ├── blob_auto.c            arena 执行器（串口下装 0x8000 独立运行）
│   ├── y3t_*.h                三张表：整网执行表 / 尺度偏移(对齐) / 检测头阈值
│   ├── gpio_ctl.h / cmd_proto.h / perf_probe.h / trap_emu.h / layer_table.h
│   ├── board.ld / netrun_ld.ld（链接脚本）/ blob_syms.h（借用层地址）
│   └── Makefile               make demo.elf / autob / hex
├── host/              上位机（Python38 + pyserial；net_station 另需 numpy/PIL）
│   ├── net_station.py         数据采集 / 权重尺度输入上传 / 整网启动（总控 GUI）
│   ├── blob_station.py        blob 部署 + 运行中阈值热调（十档 0.15~0.80）
│   ├── coremark_station.py    CoreMark 一键测试取分
│   ├── eval_bench.py          验证集发图评测（P/R/F1 + 叠加图）
│   └── board_lib.py           共享串口协议库
├── scripts/           构建与部署辅助
│   ├── split_hex.py / merge_lanes.py / check_fw_hex.py   # hex 链与门禁
│   ├── gen_net_table.py / export_engine_weights.py /
│   │   export_scales.py / pack_scales_aligned.py          # 模型产物生成
│   └── upload_weights.py / switch_fw.sh
└── release/           当前构建产物快照（v72+blobfeat）
    ├── omnicore_fw32.hex (+ 4 lane)   # PDS Compile $readmemh 烤入比特流
    ├── blob_auto.bin                   # arena 执行器镜像（串口下装）
    └── engine_weights.bin / engine_scales(_aligned).bin
```

## 尺度文件配对（重要——传反不报错，只是结果错）

| 执行体 | DDR 580000 必须放 | 偏移表 |
|---|---|---|
| 固件整网（0x10/0x15） | engine_scales.bin（对齐版） | y3t_scale_table_blob.h |
| arena blob | engine_scales_aligned.bin（对齐版） | y3t_scale_table_blob.h |

本仓库固件与 blob 均使用**对齐版**（同一张 y3t_scale_table_blob.h），直接配套。


## 编译与部署顺序

1. 固件：`gcc ... -T board.ld boot.S demo.c ... -o demo.elf` →
   `split_hex.py demo.elf omnicore_fw 24576 65536` → 4 lane 拷入 ipcore →
   `merge_lanes.py` → fw32.hex → `check_fw_hex.py` 门禁 → PDS 出片
2. arena blob：`gcc ... -T netrun_ld.ld blob_auto.c -o blob_auto.elf` → `blob_auto.bin`
3. 上位机：net_station 一键全传（权重+尺度+act_in）→ AUTO_RUN；
   或 blob_station 部署 blob（复位后首用需全量——**复位会破坏 DDR**）

## 已知口径

* 检测类别（classes.txt 新序）：0=bottle 1=mouse 2=cup 3=toy 4=screen
* 阈值默认 conf≥0.30（逐头 thr_q 内嵌检测表），blob 运行中可热调
* E_MTIME 时基 54.054 MHz（mtime 与 CPU 同频，实测与帧率计数交叉验证）
