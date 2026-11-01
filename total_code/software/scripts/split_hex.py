#!/usr/bin/env python3
# split_hex.py — 把 ELF 按 boardsoc 的 BRAM 字节 lane 拆成 4 个 $readmemh 文件
# 用法: split_hex.py <elf> <hex_prefix> <total_words> <boot_bytes>
#   boot 区位于 0x0, 主区位于 0x1000_0000 (spec §7.1);
#   输出: <prefix>_l0.hex .. <prefix>_l3.hex (每行一字节, word 顺序)
import subprocess, sys, tempfile, os

def main():
    elf, prefix, total_words, boot_bytes = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
    objcopy = os.environ.get("CROSS_PREFIX", "riscv32-unknown-elf-") + "objcopy"
    with tempfile.NamedTemporaryFile("w+", suffix=".vlog", delete=False) as tf:
        vlog_path = tf.name
    subprocess.run([objcopy, "-O", "verilog", "--verilog-data-width=1", elf, vlog_path], check=True)
    mem = {}
    addr = None
    with open(vlog_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("@"):
                addr = int(line[1:], 16)
            else:
                for tok in line.split():
                    mem[addr] = int(tok, 16)
                    addr += 1
    os.unlink(vlog_path)
    main_off = 0x10000000
    boot_words = boot_bytes // 4
    assert total_words >= boot_words
    for lane in range(4):
        out = []
        for w in range(total_words):
            if w < boot_words:
                a = w * 4 + lane
            else:
                a = main_off + (w - boot_words) * 4 + lane
            out.append("%02x" % mem.get(a, 0))
        with open("%s_l%d.hex" % (prefix, lane), "w") as f:
            f.write("\n".join(out) + "\n")
    print("wrote %s_l0..3.hex (%d words)" % (prefix, total_words))

if __name__ == "__main__":
    main()
