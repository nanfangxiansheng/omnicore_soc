# -*- coding: utf-8 -*-
"""pack_scales_aligned.py — 把尺度 blob 重排成每层段 64 字（256B）对齐。

背景：DDR 桥的多 beat 读要求起始块 8 块（256B）对齐，非对齐起始返回
0xffffffff（板上实测，2026-10-24）。原 engine_scales.bin 的层段紧排，
bias 段偏移天然非对齐 ⇒ 固件 rom_stream 读尺度必坏。
本脚本生成：
  sw/engine_scales_aligned.bin   重排后的 blob（每层段 64 字对齐，0 填充）
  sw/y3t_scale_table_blob.h      配套的偏移表（blob 编译用，偏移已更新）
"""
import os, re, struct, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
HDR = os.path.join(ROOT, 'sw', 'y3t_scale_table.h')
BIN = os.path.join(ROOT, 'sw', 'engine_scales.bin')
OUT_BIN = os.path.join(ROOT, 'sw', 'engine_scales_aligned.bin')
OUT_HDR = os.path.join(ROOT, 'sw', 'y3t_scale_table_blob.h')

txt = open(HDR, encoding='utf-8').read()
sbin = open(BIN, 'rb').read()
sw = list(struct.unpack('<%dI' % (len(sbin) // 4), sbin))

# 解析 g_scale 的每一行: { n, shM0, shM1, shY, w_g0, w_g1, w_bias, multY }, /* 名字 */
rows = re.findall(r'\{\s*(\d+)u,\s*(\d+)u,\s*(\d+)u,\s*(\d+)u,\s*(\d+)u,\s*(\d+)u,\s*(\d+)u,\s*(\d+)u\s*\},\s*/\*\s*(\S+)\s*\*/', txt)
entries = []
for n, shM0, shM1, shY, w_g0, w_g1, w_bias, multY, name in rows:
    entries.append({'name': name, 'n': int(n), 'shM0': int(shM0), 'shM1': int(shM1),
                    'shY': int(shY), 'w_g0': int(w_g0), 'w_g1': int(w_g1),
                    'w_bias': int(w_bias), 'multY': int(multY)})
assert len(entries) == 29, len(entries)

ALIGN = 64            # 字（= 8 块 = 256B）
out = []
cur = 0
new_entries = []
def place(words):
    """把一段字放到下一个 64 字对齐位置，返回起始偏移"""
    global cur, out
    if cur % ALIGN:
        pad = ALIGN - (cur % ALIGN)
        out += [0] * pad
        cur += pad
    start = cur
    out += words
    cur += len(words)
    return start

for e in entries:
    if e['n'] == 0:                     # pool/upsample 层：无尺度段
        new_entries.append(dict(e, w_g0=0, w_g1=0, w_bias=0))
        continue
    n = e['n']
    g1 = 1 if e['w_g1'] else 0
    ng0 = place(sw[e['w_g0']:e['w_g0'] + n])                     # group0 multM
    ng1 = place(sw[e['w_g1']:e['w_g1'] + n]) if g1 else 0        # group1 multM
    nb = place(sw[e['w_bias']:e['w_bias'] + n])                  # bias
    new_entries.append(dict(e, w_g0=ng0, w_g1=ng1, w_bias=nb))

while len(out) % 8:
    out.append(0)                       # 末尾补齐到整块

open(OUT_BIN, 'wb').write(struct.pack('<%dI' % len(out), *out))
print('原 blob %d 字 → 对齐 blob %d 字（%d 块）' % (len(sbin), len(out) * 4, len(out) // 8))

# 生成配套头文件
hdr = []
hdr.append('/* y3t_scale_table_blob.h —— 对齐版尺度偏移（pack_scales_aligned.py 生成）')
hdr.append(' *   每层段起始 64 字对齐，配合 rom_stream 的对齐读。数值与 y3t_scale_table.h 同源。 */')
hdr.append('#ifndef Y3T_SCALE_TABLE_BLOB_H')
hdr.append('#define Y3T_SCALE_TABLE_BLOB_H')
hdr.append('#include <stdint.h>')
hdr.append('#define Y3T_SCALE_LAYERS   %du' % len(entries))
hdr.append('#define Y3T_SCALE_WORDS    %du' % len(out))
hdr.append('typedef struct {')
hdr.append('\tuint16_t n;')
hdr.append('\tuint16_t shM0, shM1, shY;')
hdr.append('\tuint32_t w_g0, w_g1, w_bias, multY;')
hdr.append('} y3t_scale_t;')
hdr.append('static const y3t_scale_t g_scale[Y3T_SCALE_LAYERS] = {')
for e in new_entries:
    hdr.append('\t{%5du, %3du, %3du, %3du, %6du, %6du, %6du, %uu }, /* %s */' % (
        e['n'], e['shM0'], e['shM1'], e['shY'], e['w_g0'], e['w_g1'],
        e['w_bias'], e['multY'], e['name']))
hdr.append('};')
hdr.append('#endif')
open(OUT_HDR, 'w', encoding='utf-8').write('\n'.join(hdr) + '\n')
print('已生成', OUT_BIN, '和', OUT_HDR)
