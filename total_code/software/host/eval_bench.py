#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""eval_bench.py —— 静态图评测台 (blob 通道版, 2026-10-30)

链路: PC 发 'D' + 96x96x3 RGB u8(27648B) + 异或校验
      → blob 量化写 y3tram IN → 撤 arm 跑整网 → 板上解码(Q8+NMS, 与实时路同一份代码)
      → 回传 "[DET] n=.. c/q/x/y/w/h .."  (Q8 网格值, PC 除 256 得像素)
      PC 只做: 发图 / 解析 [DET] / 与 YOLO 真值 IoU 匹配 / 汇总准确率

按钮: ①『部署+评测』= 复位后首用(传权重+对齐尺度+blob, ~2min)
      ②『仅评测』= blob 已在跑时直接发图
真值: labels/val/<name>.txt 每行 "cls cx cy w h" 归一化(图 96x96 直接 ×96)
类别: 0=bottle 1=mouse 2=cup 3=toy 4=screen

用法:  C:\\Python38\\python.exe tools\\pc\\eval_bench.py
"""
import json
import math
import os
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext

import numpy as np
import serial
from serial.tools import list_ports
from PIL import Image, ImageDraw, ImageTk

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
SW = os.path.join(PROJ, 'sw')
VAL_IMG = r'C:\Users\26871\Desktop\ziguang2\ziguang2\images\val'
VAL_LBL = r'C:\Users\26871\Desktop\ziguang2\ziguang2\labels\val'
PARAMS = r'C:\Users\26871\Desktop\export_y3t\y3t_params.json'
CLASSES = ['bottle', 'mouse', 'cup', 'toy', 'screen']
N_EVAL = 0                                    # 0 = 整个 val 文件夹全量评测
IOU_MATCH = 0.30
W_BLK, S_BLK = 480000, 580000
OUT_ROOT = os.path.join(PROJ, 'eval_out')     # 每次评测建时间戳子目录存叠加图+报告


# ---- 协议（部署阶段用, 与 deploy_newmodel.py 一致）----
def _cksum(cmd, payload):
    ck = cmd ^ (len(payload) & 0xFF) ^ ((len(payload) >> 8) & 0xFF)
    for b in payload:
        ck ^= b
    return ck & 0xFF


def send(ser, cmd, payload=b''):
    ser.write(bytes([0xA5, cmd, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF])
              + payload + bytes([_cksum(cmd, payload)]))


def recv_rsp(ser, timeout=3.0):
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(64)
        if chunk:
            buf += chunk
        while True:
            i = buf.find(b'\x5a')
            if i < 0:
                buf.clear()
                break
            if i:
                del buf[:i]
            if len(buf) < 5:
                break
            cmd, status = buf[1], buf[2]
            plen = buf[3] | (buf[4] << 8)
            if plen > 4096:
                del buf[:1]
                continue
            if len(buf) < 5 + plen + 1:
                break
            del buf[:5 + plen + 1]
            return (cmd, status)
    return None


def ping_ok(ser):
    for _ in range(4):
        ser.reset_input_buffer()
        send(ser, 0x01)
        r = recv_rsp(ser, 1.5)
        if r and r[1] == 0:
            return True
        time.sleep(0.4)
    return False


def bulk_upload(ser, data, base_blk, log, label):
    n = (len(data) + 255) // 256
    log('[上传] %s %d B → DDR %d' % (label, len(data), base_blk))
    blk = base_blk
    t0 = time.time()
    for i in range(n):
        body = data[i * 256:(i + 1) * 256]
        if len(body) % 32:
            body += b'\x00' * (32 - len(body) % 32)
        send(ser, 0x0B, struct.pack('<IH', blk, len(body)) + body)
        blk += len(body) // 32
    ser.flush()
    time.sleep(0.3)
    log('[上传] %s 完成 %.0fs' % (label, time.time() - t0))


def load_gt(path):
    gts = []
    if os.path.exists(path):
        for ln in open(path):
            p = ln.split()
            if len(p) == 5:
                c, cx, cy, w, h = (float(v) for v in p)
                gts.append([int(c), cx * 96, cy * 96, w * 96, h * 96])
    return gts


def iou(p, g):
    """pred [cls,conf,cx,cy,w,h] × GT [cls,cx,cy,w,h]"""
    px0, px1 = p[2] - p[4] / 2, p[2] + p[4] / 2
    py0, py1 = p[3] - p[5] / 2, p[3] + p[5] / 2
    gx0, gx1 = g[1] - g[3] / 2, g[1] + g[3] / 2
    gy0, gy1 = g[2] - g[4] / 2, g[2] + g[4] / 2
    iw = max(0.0, min(px1, gx1) - max(px0, gx0))
    ih = max(0.0, min(py1, gy1) - max(py0, gy0))
    inter = iw * ih
    uni = p[4] * p[5] + g[3] * g[4] - inter
    return inter / uni if uni > 0 else 0.0


class App(object):
    def __init__(self, root):
        self.root = root
        self.ser = None
        self.busy = False
        root.title(' Y3T 静态图评测台（blob 通道）  IoU≥%.2f 判 TP' % IOU_MATCH)
        root.geometry('1150x760')
        self._build()

    def log(self, s):
        def _p():
            self.txt.insert('end', s + '\n')
            self.txt.see('end')
        self.root.after(0, _p)

    def _build(self):
        top = ttk.Frame(self.root)
        top.pack(fill='x', padx=6, pady=4)
        ttk.Label(top, text='串口').pack(side='left')
        self.cb = ttk.Combobox(top, width=8, values=[p.device for p in list_ports.comports()])
        self.cb.pack(side='left', padx=3)
        if self.cb['values']:
            self.cb.set(self.cb['values'][0])
        ttk.Button(top, text='部署+评测（复位后首用 ~2min）',
                   command=lambda: self._go(True)).pack(side='left', padx=6)
        ttk.Button(top, text='仅评测（blob 已在跑）',
                   command=lambda: self._go(False)).pack(side='left', padx=6)
        ttk.Button(top, text='连接测试', command=self._test_conn).pack(side='left', padx=6)
        ttk.Button(top, text='刷新串口', command=self._refresh).pack(side='left', padx=6)
        self.lb = ttk.Label(top, text='未开始', foreground='#b00')
        self.lb.pack(side='left', padx=10)

        mid = ttk.Frame(self.root)
        mid.pack(fill='both', expand=True, padx=6, pady=4)
        cols = ('图片', '真值', '检出', 'TP', 'FP', 'FN', '最佳IoU', '检出conf')
        self.tv = ttk.Treeview(mid, columns=cols, show='headings', height=8)
        for c in cols:
            self.tv.heading(c, text=c)
            self.tv.column(c, width=110 if c not in ('真值', '检出') else 220)
        self.tv.pack(side='top', fill='x')
        self.lb_sum = ttk.Label(mid, text='', font=('Consolas', 11, 'bold'))
        self.lb_sum.pack(side='top', anchor='w', pady=4)
        self.pv = ttk.Label(mid)
        self.pv.pack(side='left', padx=(0, 6))
        self.txt = scrolledtext.ScrolledText(mid, width=70, height=16, font=('Consolas', 9))
        self.txt.pack(side='left', fill='both', expand=True)

    def _go(self, deploy):
        if self.busy:
            return
        def _safe():
            self.busy = True
            try:
                self._eval(deploy)
            except Exception as e:
                self.log('[错误] %s' % e)
                self.lb.config(text='失败: %s' % e, foreground='#b00')
            finally:
                self.busy = False
        threading.Thread(target=_safe, daemon=True).start()

    def _refresh(self):
        self.cb['values'] = [p.device for p in list_ports.comports()]
        if self.cb['values'] and not self.cb.get():
            self.cb.set(self.cb['values'][0])

    def _test_conn(self):
        """连接测试: 先试固件命令循环(PING), 再试 blob(发 r 看 [THR])"""
        if self.busy:
            return
        def _safe():
            self.busy = True
            try:
                ser = serial.Serial(self.cb.get(), 460800, timeout=0.8)
            except Exception as e:
                self.log('[连接] %s 打不开: %s' % (self.cb.get(), e))
                self.lb.config(text='端口打不开', foreground='#b00')
                return
            try:
                if ping_ok(ser):
                    self.lb.config(text='已连接 · 固件命令循环（可「部署+评测」）', foreground='#080')
                    self.log('[连接] PING OK —— 固件循环在位')
                elif self._probe_blob(ser):
                    self.lb.config(text='已连接 · blob 运行中（可直接「仅评测」）', foreground='#080')
                    self.log('[连接] blob 存活（[THR] 已回应）')
                else:
                    self.lb.config(text='已连接 · 无响应（既非固件也非 blob）', foreground='#b00')
                    self.log('[连接] 串口通但无响应')
            finally:
                ser.close()
                self.busy = False
        threading.Thread(target=_safe, daemon=True).start()

    def _probe_blob(self, ser):
        ser.reset_input_buffer()
        ser.write(b'r')
        t0 = time.time()
        while time.time() - t0 < 1.5:
            d = ser.read(128)
            if d and b'[THR]' in d:
                return True
        return False

    def _deploy(self, ser):
        if not ping_ok(ser):
            raise RuntimeError('PING 失败 —— 需固件命令循环, 请先按板上复位')
        self.log('[0] PING OK, 开始部署')
        bulk_upload(ser, open(os.path.join(SW, 'engine_weights.bin'), 'rb').read(),
                    W_BLK, self.log, '权重')
        bulk_upload(ser, open(os.path.join(SW, 'engine_scales_aligned.bin'), 'rb').read(),
                    S_BLK, self.log, '尺度(对齐, blob 路)')
        blob = open(os.path.join(SW, 'blob_auto.bin'), 'rb').read()
        n = (len(blob) + 3) // 4
        data = blob + b'\x00' * (n * 4 - len(blob))
        for off in range(0, n, 120):
            cw = min(120, n - off)
            send(ser, 0x0C, struct.pack('<II', 0x8000 + off * 4, cw)
                 + data[off * 4:(off + cw) * 4])
            recv_rsp(ser, 3.0)
        self.log('[下装] blob %d B 完成' % len(blob))
        send(ser, 0x15, struct.pack('<III', 0, W_BLK, S_BLK))
        recv_rsp(ser, 2.0)
        send(ser, 0x03, struct.pack('<IH', 0xFC00, 12)
             + struct.pack('<IIi', W_BLK, S_BLK, 0))
        recv_rsp(ser, 2.0)
        ser.reset_input_buffer()
        send(ser, 0x0D, struct.pack('<I', 0x8000))
        time.sleep(1.0)
        self.log('[EXEC] blob 已启动')

    def _eval(self, deploy):
        imgs = sorted(f for f in os.listdir(VAL_IMG) if f.lower().endswith('.png'))
        if N_EVAL > 0:
            imgs = imgs[:N_EVAL]
        outdir = os.path.join(OUT_ROOT, time.strftime('%m%d_%H%M%S'))
        os.makedirs(outdir, exist_ok=True)
        self.log('输出目录: %s' % outdir)
        report = []                                   # 汇总报告行缓存
        self.log('评测集(%d): %s' % (len(imgs), ', '.join(imgs)))
        ser = serial.Serial(self.cb.get(), 460800, timeout=0.8)
        pj = json.load(open(PARAMS, encoding='utf-8'))
        # 只取该 conv job 的 s_out（jobs 里还有 pool/up/concat, 无此字段）
        so = next(j['s_out'] for j in pj['jobs'] if j['name'] == 'H_cv30_k2')

        if deploy:
            self.lb.config(text='部署中…', foreground='#008')
            self._deploy(ser)
        elif not self._probe_blob(ser):
            raise RuntimeError('blob 无响应 —— 先按复位, 再点「部署+评测」')

        TP = FP = FN = 0
        per_cls = {c: [0, 0, 0] for c in range(5)}
        for k, name in enumerate(imgs):
            self.lb.config(text='评测 %d/%d: %s' % (k + 1, len(imgs), name), foreground='#008')
            im = Image.open(os.path.join(VAL_IMG, name)).convert('RGB')
            raw = np.asarray(im, np.uint8).reshape(-1).tobytes()      # 27648B
            xchk = 0
            for b in raw:
                xchk ^= b
            ser.reset_input_buffer()
            ser.write(bytes([0x44, 0xDE, 0xAD]))   # D + magic DE AD
            time.sleep(0.05)
            ser.write(raw)
            ser.write(bytes([xchk]))
            det_line, buf, t0 = None, b'', time.time()
            while time.time() - t0 < 15.0:
                d = ser.read(256)
                if d:
                    buf += d
                    for ln in buf.decode('utf-8', 'replace').splitlines():
                        if ln.startswith('[DET]'):
                            det_line = ln
                    if det_line:
                        break
            if not det_line:
                raise RuntimeError('%s 未收到 [DET]（尾: %r）' % (name, buf[-80:]))
            if 'ERR' in det_line:
                raise RuntimeError('%s 板端校验失败: %s' % (name, det_line))
            preds = []
            for tok in det_line.split()[2:]:
                f = tok.split('/')
                if len(f) == 6:
                    # token 形如 c0/q1463/x13750/.. —— 首字符是类型字母, 去掉再转 int
                    c, q, xq, yq, wq, hq = (int(v[1:]) for v in f)
                    conf = 1.0 / (1.0 + math.exp(-max(-30.0, min(30.0, q * so))))
                    preds.append([c, conf, xq / 256.0, yq / 256.0, wq / 256.0, hq / 256.0])
            gts = load_gt(os.path.join(VAL_LBL, name.replace('.png', '.txt')))

            used = [False] * len(preds)
            tp = fp = fn = 0
            best_iou = 0.0
            det_desc = []
            for g in gts:
                bi, bv = -1, 0.0
                for pi, p in enumerate(preds):
                    if used[pi] or p[0] != g[0]:
                        continue
                    v = iou(p, g)
                    if v > bv:
                        bv, bi = v, pi
                if bv >= IOU_MATCH:
                    used[bi] = True
                    tp += 1
                    best_iou = max(best_iou, bv)
                    det_desc.append('%s✓%.0f%%' % (CLASSES[g[0]], preds[bi][1] * 100))
                    per_cls[g[0]][0] += 1
                else:
                    fn += 1
                    per_cls[g[0]][2] += 1
            for pi, p in enumerate(preds):
                if not used[pi]:
                    fp += 1
                    per_cls[p[0]][1] += 1
            TP += tp; FP += fp; FN += fn
            gt_desc = ', '.join(CLASSES[g[0]] for g in gts) or '(空)'
            self.tv.insert('', 'end', values=(
                name[:14], gt_desc, ', '.join(det_desc) or '(无)',
                tp, fp, fn, '%.2f' % best_iou,
                ' '.join('%.0f%%' % (p[1] * 100) for p in preds[:3]) or '-'))
            self.log('  %s: GT=%d 检出=%d TP=%d FP=%d FN=%d' % (name, len(gts), len(preds), tp, fp, fn))
            save_p = os.path.join(outdir, name.replace('.png', '_gtpred.png'))
            self._preview(os.path.join(VAL_IMG, name), gts, preds, save_p)
            report.append('%-16s GT[%s] 检出[%s] TP=%d FP=%d FN=%d IoU=%.2f'
                          % (name, gt_desc, ', '.join(det_desc) or '无', tp, fp, fn, best_iou))
        ser.close()
        P = TP / (TP + FP) if TP + FP else 0.0
        R = TP / (TP + FN) if TP + FN else 0.0
        F1 = 2 * P * R / (P + R) if P + R else 0.0
        cls_txt = '  '.join('%s:%d✓/%d✗' % (CLASSES[c], per_cls[c][0], per_cls[c][2])
                            for c in range(5) if per_cls[c][0] + per_cls[c][2])
        self.lb_sum.config(
            text='准确率: Precision=%.1f%%  Recall=%.1f%%  F1=%.1f%%   (TP=%d FP=%d FN=%d)\n%s'
                 % (P * 100, R * 100, F1 * 100, TP, FP, FN, cls_txt), foreground='#080')
        self.lb.config(text='完成', foreground='#080')
        self.log('=== 评测完成: P=%.1f%% R=%.1f%% F1=%.1f%% ===' % (P * 100, R * 100, F1 * 100))
        # ---- 汇总报告落盘 ----
        with open(os.path.join(outdir, 'summary.txt'), 'w', encoding='utf-8') as f:
            f.write('Y3T 静态图评测  %s  (conf≥%.2f 判定, IoU≥%.2f 记 TP)\n'
                    % (time.strftime('%Y-%m-%d %H:%M:%S'), 0.30, IOU_MATCH))
            f.write('Precision=%.1f%%  Recall=%.1f%%  F1=%.1f%%   TP=%d FP=%d FN=%d\n'
                    % (P * 100, R * 100, F1 * 100, TP, FP, FN))
            f.write(cls_txt + '\n\n')
            f.write('\n'.join(report) + '\n')
        self.log('[保存] %d 张叠加图 + summary.txt → %s' % (len(imgs), outdir))

    def _preview(self, path, gts, preds, save_path=None):
        im = Image.open(path).convert('RGB').resize((384, 384), Image.NEAREST)
        d = ImageDraw.Draw(im)
        S = 4.0
        for g in gts:
            x0 = (g[1] - g[3] / 2) * S
            y0 = (g[2] - g[4] / 2) * S
            d.rectangle([x0, y0, x0 + g[3] * S, y0 + g[4] * S], outline=(0, 255, 0), width=3)
            d.text((x0 + 2, y0 + 2), 'GT ' + CLASSES[g[0]], fill=(0, 255, 0))
        for p in preds:
            x0 = (p[2] - p[4] / 2) * S
            y0 = (p[3] - p[5] / 2) * S
            d.rectangle([x0, y0, x0 + p[4] * S, y0 + p[5] * S], outline=(255, 60, 60), width=2)
            d.text((x0 + 2, y0 + 14), '%s %.0f%%' % (CLASSES[p[0]], p[1] * 100), fill=(255, 60, 60))
        if save_path:
            im.save(save_path)                        # 叠加图落盘（绿=真值 红=板上检出）
        self._tkimg = ImageTk.PhotoImage(im)
        self.pv.config(image=self._tkimg)


if __name__ == '__main__':
    root = tk.Tk()
    App(root)
    root.mainloop()
