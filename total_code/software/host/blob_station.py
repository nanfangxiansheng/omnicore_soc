#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""blob_station.py —— arena blob 部署 + 运行中阈值热调 上位机 (2026-10-30)

把 deploy_newmodel.py 的部署链 (权重/尺度上传 → blob 下装 → EXEC) 做成按钮,
并在 blob 运行期间通过**单字节串口命令**实时调整置信度阈值:
    '0'..'9' 十档 conf 0.15~0.80   '+'/'-' ±128 微调   'r' 恢复默认   'q' 停止
(需要配套 2026-10-30 版 blob_auto.bin —— 旧版任意字节即退出, 不认这些命令)

运行:  C:\\Python38\\python.exe tools\\pc\\blob_station.py
依赖:  pyserial (C:\\Python38 已装) + numpy (S4 校验用)
"""
import json
import math
import os
import queue
import struct
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext

import serial
from serial.tools import list_ports

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
SW = os.path.join(PROJ, 'sw')
EXPORT = r'C:\Users\26871\Desktop\export_y3t'

# ---- 协议常量（与 deploy_newmodel.py 一致）----
SYNC_REQ, SYNC_RSP = 0xA5, 0x5A
CMD_PING, CMD_MEM_READ, CMD_MEM_WRITE = 0x01, 0x02, 0x03
CMD_WLOAD_RAW, CMD_LOAD_IMG, CMD_EXEC = 0x0B, 0x0C, 0x0D
CMD_AUTO_RUN = 0x15
W_BLK, S_BLK = 480000, 580000
ARENA, MAILBOX = 0x8000, 0xFC00
BULK_BODY, LOAD_WORDS = 256, 120

# 十档置信度（与 blob 内 LOGIT_Q12 表一一对应）
CONF_LEVELS = [0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.50, 0.60, 0.70, 0.80]


def _cksum(cmd, payload):
    ck = cmd ^ (len(payload) & 0xFF) ^ ((len(payload) >> 8) & 0xFF)
    for b in payload:
        ck ^= b
    return ck & 0xFF


def send(ser, cmd, payload=b''):
    ser.write(bytes([SYNC_REQ, cmd, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF])
              + payload + bytes([_cksum(cmd, payload)]))


def recv_rsp(ser, timeout=2.0, max_len=4096):
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
            if plen > max_len:
                del buf[:1]
                continue
            if len(buf) < 5 + plen + 1:
                break
            pay = bytes(buf[5:5 + plen])
            del buf[:5 + plen + 1]
            return (cmd, status, pay)
    return None


def ping(ser, tries=4):
    for _ in range(tries):
        ser.reset_input_buffer()
        send(ser, CMD_PING)
        r = recv_rsp(ser, 1.5)
        if r and r[1] == 0:
            return True
        time.sleep(0.5)
    return False


def open_port(port, log):
    for b in (460800, 115200):
        try:
            s = serial.Serial(port, b, timeout=0.8)
        except Exception as e:
            raise RuntimeError('打不开 %s: %s' % (port, e))
        if ping(s):
            log('[0] PING OK  %s @ %d' % (port, b))
            return s, b
        s.close()
    raise RuntimeError('%s 无响应（板子在固件命令循环吗? blob 循环中需先复位）' % port)


def bulk_upload(ser, data, base_blk, log, label=''):
    n = (len(data) + BULK_BODY - 1) // BULK_BODY
    log('[上传] %s %d B → DDR %d（%d 帧）' % (label, len(data), base_blk, n))
    blk = base_blk
    t0 = time.time()
    for i in range(n):
        body = data[i * BULK_BODY:(i + 1) * BULK_BODY]
        if len(body) % 32:
            body += b'\x00' * (32 - len(body) % 32)
        pay = struct.pack('<IH', blk, len(body)) + body
        send(ser, CMD_WLOAD_RAW, pay)
        blk += len(body) // 32
        if i % 512 == 511:
            log('  ... %d/%d (%.0fs)' % (min((i + 1) * BULK_BODY, len(data)), len(data),
                                         time.time() - t0))
    ser.flush()
    log('[上传] %s 完成 %.0fs %.1f KB/s' % (label, time.time() - t0,
        len(data) / (time.time() - t0) / 1024))


def mem_read(ser, addr, n):
    out = b''
    off = 0
    while off < n:
        m = min(480, n - off)
        for _ in range(3):
            ser.reset_input_buffer()
            send(ser, CMD_MEM_READ, struct.pack('<IH', addr + off, m))
            r = recv_rsp(ser, 3.0)
            if r and len(r[2]) >= m:
                out += r[2][:m]
                off += m
                break
        else:
            return None
    return out


def load_blob(ser, blob, log):
    n = (len(blob) + 3) // 4
    data = blob + b'\x00' * (n * 4 - len(blob))
    t0 = time.time()
    for off in range(0, n, LOAD_WORDS):
        cw = min(LOAD_WORDS, n - off)
        chunk = data[off * 4:(off + cw) * 4]
        ser.reset_input_buffer()
        send(ser, CMD_LOAD_IMG, struct.pack('<II', ARENA + off * 4, cw) + chunk)
        r = recv_rsp(ser, 3.0)
        back = mem_read(ser, ARENA + off * 4, cw * 4)
        if not (r and r[1] == 0) or back != chunk:
            raise RuntimeError('blob 段 @0x%x 回读不一致' % (ARENA + off * 4))
    log('[下装] %d 字校验通过 (%.1fs)' % (n, time.time() - t0))


def check_blob_tables(blob, log):
    """S4: blob 内必须含本模型 4 个头的 thr_q/s_out_q24"""
    pj = json.load(open(os.path.join(EXPORT, 'y3t_params.json'), encoding='utf-8'))
    by = {j['name']: j for j in pj['jobs']}
    ok = True
    for nm in ('H_cv20_k2', 'H_cv30_k2', 'H_cv21_k2', 'H_cv31_k2'):
        s = by[nm]['s_out']
        tq = int(round(math.log(0.3 / 0.7) / s))
        sq = int(round(s * (1 << 24)))
        hit = struct.pack('<i', tq) in blob and struct.pack('<i', sq) in blob
        ok &= hit
        log('  S4 %-11s thr_q=%-6d %s' % (nm, tq, 'Y' if hit else 'N!'))
    if not ok:
        raise RuntimeError('blob 不是本模型配套的（S4 FAIL）')


class App(object):
    def __init__(self, root):
        self.root = root
        self.ser = None
        self.q = queue.Queue()
        self.rx_thread = None
        self.rx_pause = threading.Event()   # 部署/收发期间挂起日志线程, 防抢响应帧
        self.stat = {'fps': '-', 'det': '-', 'last': ''}
        root.title(' blob 部署/阈值热调台  (arena · 一类一灯 LED)')
        root.geometry('980x720')
        self._build()
        self.root.after(80, self._drain)

    # ---------------- 界面 ----------------
    def _build(self):
        top = ttk.LabelFrame(self.root, text='① 连接（需固件命令循环：blob 运行中请先按复位）')
        top.pack(fill='x', padx=6, pady=4)
        self.cb_port = ttk.Combobox(top, width=10, values=[p.device for p in list_ports.comports()])
        self.cb_port.pack(side='left', padx=(8, 2))
        if self.cb_port['values']:
            self.cb_port.set(self.cb_port['values'][0])
        ttk.Button(top, text='刷新', width=6, command=self._refresh).pack(side='left', padx=3)
        ttk.Button(top, text='连接 PING', command=lambda: self._task(self._connect)).pack(side='left', padx=6)
        self.lb_conn = ttk.Label(top, text='未连接', foreground='#b00')
        self.lb_conn.pack(side='left', padx=8)

        mid = ttk.Frame(self.root)
        mid.pack(fill='x', padx=6, pady=4)
        f2 = ttk.LabelFrame(mid, text='② 部署（blob 路 · 对齐尺度）')
        f2.pack(side='left', fill='both', expand=True, padx=(0, 4))
        ttk.Button(f2, text='一键全量（权重+尺度+blob+启动）约 90s',
                   command=lambda: self._task(self._deploy_full)).pack(fill='x', padx=6, pady=(6, 2))
        ttk.Button(f2, text='快速重跑（跳过上传, 仅重下装 blob）约 40s',
                   command=lambda: self._task(self._deploy_fast)).pack(fill='x', padx=6, pady=2)
        ttk.Label(f2, text='权重→DDR480000 尺度(对齐)→580000\nblob→arena 0x8000',
                  foreground='#666').pack(padx=6, pady=2)

        f3 = ttk.LabelFrame(mid, text='③ 运行中阈值热调（单字节命令, ~100ms 生效）')
        f3.pack(side='left', fill='both', expand=True)
        row = ttk.Frame(f3)
        row.pack(fill='x', padx=6, pady=(6, 2))
        ttk.Label(row, text='十档:').pack(side='left')
        for i, c in enumerate(CONF_LEVELS):
            ttk.Button(row, text='%.2f' % c, width=5,
                       command=lambda i=i: self._send_cmd(chr(ord('0') + i))).pack(side='left', padx=1)
        row2 = ttk.Frame(f3)
        row2.pack(fill='x', padx=6, pady=2)
        ttk.Button(row2, text='阈值 +128', command=lambda: self._send_cmd('+')).pack(side='left', padx=2)
        ttk.Button(row2, text='阈值 −128', command=lambda: self._send_cmd('-')).pack(side='left', padx=2)
        ttk.Button(row2, text='恢复默认 0.30', command=lambda: self._send_cmd('r')).pack(side='left', padx=2)
        ttk.Button(row2, text='停止 blob（q, 回固件）', command=lambda: self._send_cmd('q')).pack(side='left', padx=10)
        self.lb_stat = ttk.Label(f3, text='fps=-  det=-', font=('Consolas', 10, 'bold'))
        self.lb_stat.pack(anchor='w', padx=6, pady=(2, 6))

        f4 = ttk.LabelFrame(self.root, text='④ 板端日志')
        f4.pack(fill='both', expand=True, padx=6, pady=4)
        self.txt = scrolledtext.ScrolledText(f4, font=('Consolas', 9), height=22)
        self.txt.pack(fill='both', expand=True)

    def _refresh(self):
        self.cb_port['values'] = [p.device for p in list_ports.comports()]
        if self.cb_port['values'] and not self.cb_port.get():
            self.cb_port.set(self.cb_port['values'][0])

    # ---------------- 基础 ----------------
    def log(self, s):
        self.q.put(s)

    def _drain(self):
        try:
            while True:
                line = self.q.get_nowait()
                self.txt.insert('end', line + '\n')
                self.txt.see('end')
                self._parse(line)
        except queue.Empty:
            pass
        self.lb_stat.config(text='fps=%(fps)s  det=%(det)s' % self.stat)
        self.root.after(80, self._drain)

    def _parse(self, line):
        if '[AUTO]' in line:
            d = dict(p.split('=') for p in line.split() if '=' in p)
            if 'tot' in d:
                try:
                    fx = 54054000.0 / max(int(d['tot']), 1)
                    self.stat['fps'] = '%.1f' % fx
                except Exception:
                    pass
            self.stat['det'] = d.get('det', '-')
            self.stat['last'] = line

    def _task(self, fn):
        threading.Thread(target=self._safe, args=(fn,), daemon=True).start()

    def _safe(self, fn):
        try:
            fn()
        except Exception as e:
            self.log('[错误] %s' % e)

    # ---------------- 动作 ----------------
    def _connect(self):
        self.rx_pause.set()                    # 收发独占串口
        try:
            self._connect_inner()
        finally:
            self.rx_pause.clear()

    def _connect_inner(self):
        if self.ser:
            self.ser.close()
            self.ser = None
        port = self.cb_port.get()
        if not port:
            raise RuntimeError('先选串口')
        self.ser, baud = open_port(port, self.log)
        self.lb_conn.config(text='%s @ %d 已连接' % (port, baud), foreground='#080')
        self._start_rx()

    def _ensure_open(self):
        if not self.ser:
            self._connect()
        if not ping(self.ser, 2):
            raise RuntimeError('PING 失败 —— blob 循环中？按板上复位再点连接')

    def _start_rx(self):
        if self.rx_thread:
            return
        def rx():
            while True:
                try:
                    if self.rx_pause.is_set():   # 部署线程在收发, 日志线程让路
                        time.sleep(0.05)
                        continue
                    if self.ser and self.ser.is_open:
                        d = self.ser.read(256)
                        if d:
                            for line in d.decode('utf-8', 'replace').splitlines():
                                if line.strip():
                                    self.log('  | ' + line.strip()[:150])
                    else:
                        time.sleep(0.3)
                except Exception:
                    time.sleep(0.5)
        self.rx_thread = threading.Thread(target=rx, daemon=True)
        self.rx_thread.start()

    def _deploy(self, upload):
        self.rx_pause.set()                    # 部署期间日志线程必须让路
        try:
            self._deploy_inner(upload)
        finally:
            self.rx_pause.clear()

    def _deploy_inner(self, upload):
        blob = open(os.path.join(SW, 'blob_auto.bin'), 'rb').read()
        self.log('blob = %d B (arena 上限 32768)' % len(blob))
        check_blob_tables(blob, self.log)
        self._ensure_open()
        if upload:
            bulk_upload(self.ser, open(os.path.join(SW, 'engine_weights.bin'), 'rb').read(),
                        W_BLK, self.log, '权重')
            bulk_upload(self.ser, open(os.path.join(SW, 'engine_scales_aligned.bin'), 'rb').read(),
                        S_BLK, self.log, '尺度(对齐)')
        load_blob(self.ser, blob, self.log)
        # 停固件循环 → 写邮箱(默认阈值 0=表内) → EXEC
        send(self.ser, CMD_AUTO_RUN, struct.pack('<III', 0, W_BLK, S_BLK))
        recv_rsp(self.ser, 2.0)
        time.sleep(0.2)
        send(self.ser, CMD_MEM_WRITE, struct.pack('<IH', MAILBOX, 12)
             + struct.pack('<IIi', W_BLK, S_BLK, 0))
        recv_rsp(self.ser, 2.0)
        self.ser.reset_input_buffer()
        send(self.ser, CMD_EXEC, struct.pack('<I', ARENA))
        self.log('[EXEC] 0x8000 —— blob 自治循环已启动（日志见下, 阈值随时调）')

    def _deploy_full(self):
        self.log('=== 一键全量部署 ===')
        self._deploy(upload=True)

    def _deploy_fast(self):
        self.log('=== 快速重跑（DDR 已有数据）===')
        self._deploy(upload=False)

    def _send_cmd(self, ch):
        if not (self.ser and self.ser.is_open):
            self.log('[提示] 未连接（阈值命令需要 blob 已在运行）')
            return
        data = ch.encode('ascii')
        if ch == 'q':
            data += bytes([0x55, 0xAA])   # q 退出魔数确认(防图像流碰撞)
        self.ser.write(data)
        self.log('[发] %r' % data)


if __name__ == '__main__':
    root = tk.Tk()
    App(root)
    root.mainloop()
