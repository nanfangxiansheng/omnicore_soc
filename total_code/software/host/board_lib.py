#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""board_lib.py — 板卡上位机公共库（v65, 2026-10-27）

两个上位机程序（net_station.py / coremark_station.py）共用本文件，避免各自
复制一份串口协议、上传校验、整网解码逻辑。

设计要点
--------
1) **命令与日志共用一条串口**：板子每秒打一行 `t=Ns detect=..`，而命令响应
   也是同一 UART。这里用"忙标志 + 请求前清空缓冲"的做法：命令执行期间暂停
   后台监听线程，命令结束后继续收，两边不抢字节。
2) **上传必须逐帧应答 + 全量校验**（2026-10-26 定案）：
   盲发不读 ACK 会让固件卡在 TX 上空转 → RX 溢出 → DDR 随机坏块；
   单次连续写超过 ~1MB 也会丢。所以按 512KB 分块写、每块全量校验、坏块定点重传。
3) CoreMark 会把 UART_BAUDDIV 改成 468（115200）且不恢复 —— 所以跑完要
   `reopen(115200)` 才能读结果；复位或重下比特流才回到 460800。
"""
import os
import re
import struct
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))          # tools/pc -> tools -> 项目根
TOOLS = os.path.join(PROJ, 'tools')
TOOLS2 = os.path.join(PROJ, 'tools', 'tools')
for _p in (TOOLS, TOOLS2):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import serial                                        # noqa: E402
import serial.tools.list_ports                        # noqa: E402  (必须显式导入)
import upload_weights as uw                          # noqa: E402

# ---- 命令号（upload_weights 只导出了其中几个，这里按 sw/cmd_proto.h 补齐） ----
CMD_PING      = 0x01
CMD_MEM_READ  = 0x02
CMD_MEM_WRITE = 0x03
CMD_Y3T_JOB   = 0x04
CMD_WLOAD_RAW = 0x0B
CMD_LOAD_IMG  = 0x0C
CMD_EXEC      = 0x0D
CMD_DDR2TCM   = 0x0E
CMD_NET_RUN   = 0x10
CMD_W3T       = 0x11
CMD_OVL_MODE  = 0x14
CMD_AUTO_RUN  = 0x15
CMD_SET_THR   = 0x16
CMD_GPIO      = 0x17
CMD_GPIO_IRQ  = 0x18

BAUD_DEFAULT = 460800
BAUD_COREMARK = 115200                 # CoreMark 之后固件会停在这个波特率

BLK = 32                               # DDR/TCM 一块 = 32 B = 256 bit
TMP = 0x9000                           # DDR->TCM 中转（arena 无程序时可用）
READ_MAX = 480                          # 固件 MEM_READ 单条上限
SPAN_BLOCKS = 512                      # 一次 0x0E 搬运块数 = 16 KB
CHUNK_BLOCKS = 16384                   # 写分块 = 512 KB

# 外设/寄存器地址
OVL_BASE = 0x40006000
OVL_EN = OVL_BASE + 0x60
OVL_LB = OVL_BASE + 0x64              # {bit0 lb_en, bit9:1 ofs}
OVL_XD = OVL_BASE + 0x68
PP_CSR = 0x40005000                   # 预处理 CSR（见 layer_run）
GPIO_BASE = 0x40001000
EXTIRQ_BASE = 0x40003000
TIMER_BASE = 0x40002000
CM_RESULT_ADDR = 0x0000FFE0
CM_RESULT_MAGIC = 0x434D5253          # "CMRS"


def list_ports():
    out = []
    try:
        for p in serial.tools.list_ports.comports():
            out.append(p.device)
    except Exception:
        pass
    return out


class Board(object):
    """一个板子连接。所有方法都是阻塞的，调用方自己决定放在哪个线程里跑。"""

    def __init__(self, port, baud=BAUD_DEFAULT, on_log=None):
        self.port = port
        self.baud = baud
        self.ser = None
        self._on_log = on_log or (lambda s: None)
        self._busy = threading.Event()
        self._pause = False           # 长任务期间暂停后台监听, 免得抢命令响应
        self._run = False
        self._th = None
        self._txtbuf = b''
        self._blob_ok = False        # arena 里是否已下装 netrun_blob.bin

    # ---------------- 基础 ----------------
    def log(self, s):
        self._on_log(s)

    def open(self, baud=None):
        if baud:
            self.baud = baud
        s = serial.Serial()
        s.port = self.port
        s.baudrate = self.baud
        # ★ 4ms（与 up_verified 一致）。50ms 时 recv_rsp 每条命令都白等满一拍，
        #   1.2 万帧的上传会凭空多出 ~10 分钟（2026-10-27 实测 754s -> 修复后 ~140s）
        s.timeout = 0.004
        # 不拉 DTR/RTS：会复位板子（相机要 5~8 s 才出帧）
        s.dtr = False
        s.rts = False
        s.open()
        s.reset_input_buffer()
        self.ser = s
        self._txtbuf = b''
        self._blob_ok = False        # 换了连接/波特率, arena 状态未知
        return self.ping()

    def close(self):
        self.stop_listen()
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def reopen(self, baud, do_ping=True):
        """换波特率（CoreMark 用）。"""
        self.close()
        time.sleep(0.15)
        self.baud = baud
        s = serial.Serial()
        s.port = self.port
        s.baudrate = baud
        # ★ 4ms（与 up_verified 一致）。50ms 时 recv_rsp 每条命令都白等满一拍，
        #   1.2 万帧的上传会凭空多出 ~10 分钟（2026-10-27 实测 754s -> 修复后 ~140s）
        s.timeout = 0.004
        s.dtr = False
        s.rts = False
        s.open()
        s.reset_input_buffer()
        self.ser = s
        self._txtbuf = b''
        self._blob_ok = False        # 换了连接/波特率, arena 状态未知
        # do_ping=False 给 CoreMark 用：ping 的 recv_rsp 会把板端打印整段吞掉，
        # 连 [CoreMark finished] 都会被吃掉，只能干等到超时。
        return self.ping() if do_ping else True

    def ping(self, tries=5, timeout=1.2):
        if not self.ser:
            return False
        for _ in range(tries):
            uw.send(self.ser, CMD_PING, b'')
            if uw.recv_rsp(self.ser, timeout):
                return True
            time.sleep(0.3)
        return False

    # ---------------- 命令 ----------------
    def cmd(self, code, payload=b'', timeout=2.0):
        """发一条命令并取响应；返回 (status, bytes) 或 (None, b'')。"""
        if not self.ser:
            return None, b''
        self._busy.set()
        try:
            time.sleep(0.002)
            self.ser.reset_input_buffer()
            uw.send(self.ser, code, payload)
            r = uw.recv_rsp(self.ser, timeout)
            if not r:
                return None, b''
            return r[1], r[2]
        finally:
            self._busy.clear()

    def wr32(self, addr, val):
        st, _ = self.cmd(CMD_MEM_WRITE,
                         struct.pack('<IH', addr, 4) + struct.pack('<I', val))
        return st == 0

    def rd32(self, addr):
        st, pl = self.cmd(CMD_MEM_READ, struct.pack('<IH', addr, 4))
        if st != 0 or len(pl) < 4:
            return None
        return struct.unpack('<I', pl[:4])[0]

    def rdblk(self, addr, nbytes):
        out = bytearray()
        off = 0
        while off < nbytes:
            k = min(READ_MAX, nbytes - off)
            st, pl = self.cmd(CMD_MEM_READ, struct.pack('<IH', addr + off, k), 5.0)
            if st != 0 or len(pl) < k:
                return None
            out += pl[:k]
            off += k
        return bytes(out)

    # ---------------- 后台监听（板子打印） ----------------
    def start_listen(self):
        if self._run:
            return
        self._run = True
        self._th = threading.Thread(target=self._listen_loop, daemon=True)
        self._th.start()

    def pause_listen(self, on=True):
        """长任务(上传/整网/CoreMark)期间调用: 后台监听让出串口, 避免和命令响应抢字节。"""
        self._pause = bool(on)

    def stop_listen(self):
        self._run = False
        t = self._th
        self._th = None
        if t and t.is_alive():
            t.join(timeout=0.5)

    def _listen_loop(self):
        while self._run and self.ser:
            if self._busy.is_set() or self._pause:
                time.sleep(0.02)
                continue
            try:
                d = self.ser.read(256)
            except Exception:
                break
            if not d:
                continue
            self._txtbuf += d
            while b'\n' in self._txtbuf:
                line, self._txtbuf = self._txtbuf.split(b'\n', 1)
                self.log(line.decode('utf-8', 'replace').rstrip('\r'))

    # ---------------- 竖条（显示对齐） ----------------
    SEAM_OFS = 1          # 板上实测的唯一正确档：ofs=1 时左右两边都不欠

    def fix_seam(self, ofs=None):
        """写 0x64 = (ofs<<1)|1 修掉每行最左的 16px 竖条，返回 (ok, ofs_readback)。

        原理: 行缓存按地址读，写侧有 1 个 beat(16px) 延迟 ⇒ 第 40 槽(下一 bank 第 0 槽)
        存的正是本行最后一个 beat。所以 ofs=1 时左右都不欠；ofs=0 左边 16px 是上一行
        尾部（竖条），ofs=2 右边 16px 变成下一行开头（右缝）。
        ★ V65 比特流里把 lb_ofs_r 的复位值写成 1 了，出片后无需再调；
          当前 v64 片每次上电/复位后都要重写一次。
        """
        v = self.SEAM_OFS if ofs is None else ofs
        ok = self.wr32(OVL_LB, ((v & 0x1FF) << 1) | 1)
        rb = self.rd32(OVL_LB) or 0
        got = (rb >> 2) & 0x1FF          # v64 读回 = ofs<<2|en（33 位截断）
        self.log('[竖条] 0x64 <- 0x%X (ofs=%d)  回读 0x%08X -> ofs=%d %s'
                 % (((v & 0x1FF) << 1) | 1, v, rb, got, 'OK' if got == v else 'MISMATCH'))
        return ok, got

    # ---------------- 上传（逐帧应答 + 全量校验） ----------------
    def _wr_frame(self, blk, data, retries=3):
        for _ in range(retries):
            self.ser.reset_input_buffer()
            uw.send(self.ser, CMD_WLOAD_RAW, struct.pack('<IH', blk, len(data)) + data)
            r = uw.recv_rsp(self.ser, 3.0)
            if r and r[1] == 0:
                return True
        return False

    def _copy_span(self, blk, nblk, tmp=TMP):
        self.ser.reset_input_buffer()
        uw.send(self.ser, CMD_DDR2TCM, struct.pack('<III', blk, tmp, nblk * BLK))
        r = uw.recv_rsp(self.ser, 20.0)
        return bool(r and r[1] == 0)

    def _check_range(self, src, base_blk, lo, hi, span=SPAN_BLOCKS, prog=None):
        bad = []
        b = lo
        while b < hi:
            n = min(span, hi - b)
            if not self._copy_span(b, n):
                bad += list(range(b, b + n))
                b += n
                continue
            d = self.rdblk(TMP, n * BLK)
            if d is None:
                bad += list(range(b, b + n))
            else:
                for j in range(n):
                    off = (b + j - base_blk) * BLK
                    if d[j * BLK:(j + 1) * BLK] != src[off:off + BLK]:
                        bad.append(b + j)
            b += n
            if prog:
                prog(min(hi, b) - lo, hi - lo)
        return bad

    def _check_samples(self, src, base_blk, nblk, nsamples=128, prog=None):
        """抽验：每个采样点用【对齐的 16 块窗口】读（单块 0x0E 读不可信，
        见 tools/tools/up_verified.py 的结论）。返回坏块列表。"""
        step = max(1, nblk // max(1, nsamples))
        blks = [k for k in range(0, nblk, step)][:nsamples]
        bad = []
        for i, k in enumerate(blks):
            win = k & ~15
            n = min(16, nblk - win)
            if not self._copy_span(base_blk + win, n):
                bad.append(base_blk + k)
                continue
            d = self.rdblk(TMP, n * BLK)
            if d is None:
                bad.append(base_blk + k)
                continue
            o = (k - win) * BLK
            if d[o:o + BLK] != src[k * BLK:(k + 1) * BLK]:
                bad.append(base_blk + k)
            if prog:
                prog(i + 1, len(blks))
        return bad

    def upload(self, path, base_blk, chunk_blocks=CHUNK_BLOCKS, rounds=4,
               prog=None, tag=None, verify='full', samples=128):
        """把文件写进 DDR 并全量校验；返回 (ok, 说明)。prog(done, total)。"""
        src = open(path, 'rb').read()
        if len(src) % BLK:
            src += b'\x00' * (BLK - len(src) % BLK)
        nblk = len(src) // BLK
        name = tag or os.path.basename(path)
        self.log('[上传] %s  %d B = %d 块 @%d  校验=%s'
                 % (name, len(src), nblk, base_blk, verify))

        for clo in range(0, nblk, chunk_blocks):
            chi = min(clo + chunk_blocks, nblk)
            nfr = (chi - clo + 7) // 8
            noack = 0
            for i in range(nfr):
                b = clo + i * 8
                d = src[b * BLK:min((b + 8) * BLK, len(src))]
                if not self._wr_frame(base_blk + b, d):
                    noack += 1
                if prog:
                    prog(clo + i * 8, nblk * 1.0)
            self.log('  段 [%d,%d) 写 %d 帧, 无应答 %d' % (clo, chi, nfr, noack))

            lo, hi = base_blk + clo, base_blk + chi
            if verify == 'full':
                bad = []
                for rnd in range(rounds):
                    bad = self._check_range(src, base_blk, lo, hi)
                    self.log('  第 %d 轮校验: %d/%d 正确%s'
                             % (rnd + 1, (hi - lo) - len(bad), hi - lo,
                                '' if not bad else '  **坏 %d**' % len(bad)))
                    if not bad:
                        break
                    ok = 0
                    for b in bad:
                        off = (b - base_blk) * BLK
                        if self._wr_frame(b, src[off:off + BLK]):
                            ok += 1
                    self.log('    重传 %d/%d 块' % (ok, len(bad)))
                if bad:
                    if prog:
                        prog(nblk, nblk)
                    return False, '段 [%d,%d) 仍有 %d 坏块' % (clo, chi, len(bad))
            elif verify == 'none':
                self.log('  （跳过校验 —— 若整网结果异常，优先怀疑数据）')
        if verify == 'sample':
            bad = self._check_samples(src, base_blk, nblk, samples,
                                      prog=lambda d, t: self.q_put_prog(prog, d, t, nblk))
            self.log('  抽验 %d 点: 坏 %d%s' % (samples, len(bad),
                                                '' if not bad else '  ' + str(bad[:8])))
            if bad:
                return False, '抽验发现 %d 个坏块: %s' % (len(bad), bad[:8])
        if prog:
            prog(nblk, nblk)
        if verify == 'full':
            return True, '全量校验通过（%d 块）' % nblk
        if verify == 'sample':
            return True, '抽验 %d 点通过（%d 块）' % (samples, nblk)
        return True, '未校验（%d 块）' % nblk

    def q_put_prog(self, prog, done, total, nblk):
        """抽验进度并进主进度条（写阶段已占 nblk 中的比例由调用方保证）"""
        if prog:
            prog(nblk + done, nblk + total)

    # ---------------- arena 程序 ----------------
    def load_img(self, path, addr=0x8000, chunk_words=120, prog=None):
        data = open(path, 'rb').read()
        n = (len(data) + 3) // 4
        data += b'\x00' * (n * 4 - len(data))
        for off in range(0, n, chunk_words):
            cw = min(chunk_words, n - off)
            okc = False
            # 逐块重试: 板子在打心跳文本时偶尔会吃掉一条响应（实测"块 0 下装失败"）
            for _try in range(3):
                self.ser.reset_input_buffer()
                uw.send(self.ser, CMD_LOAD_IMG,
                        struct.pack('<II', addr + off * 4, cw) + data[off * 4:(off + cw) * 4])
                r = uw.recv_rsp(self.ser, 3.0)
                if r and r[1] == 0:
                    okc = True
                    break
                time.sleep(0.15)
            if not okc:
                return False, '块 %d 下装失败（重试 3 次）' % off
            if prog:
                prog(off + cw, n)
        return True, '%d 字' % n

    def exec_no_wait(self, addr=0x8000):
        """0x0D 执行但**不等应答**。

        给 CoreMark 用：它一进来就把 UART_BAUDDIV 改成 468(115200)，
        如果我们还按 460800 等应答，只会白等 2 秒并把日志起点推后。
        """
        self.ser.reset_input_buffer()
        uw.send(self.ser, CMD_EXEC, struct.pack('<I', addr))

    def exec_at(self, addr=0x8000):
        self.ser.reset_input_buffer()
        uw.send(self.ser, CMD_EXEC, struct.pack('<I', addr))
        uw.recv_rsp(self.ser, 2.0)

    # ---------------- 整网 ----------------
    def start_auto(self, wblk=480000, romblk=580000, timeout=3.0):
        """0x15 en=1: 板上自主识别循环 (来帧→PP→整网→解码→画框→GPIO 发布)。

        与 PC 驱动的"连续识别"区别: 不再逐帧走 UART, 帧率只受整网耗时限制;
        PC 侧只收串口日志和看屏幕画框。前提: 权重/尺度已在 DDR (先"一键全传")。
        注意: 固件循环会与 PC 驱动的整网抢 y3tram 检测头, 混用前先 stop_auto。
        """
        self.ser.reset_input_buffer()
        uw.send(self.ser, CMD_AUTO_RUN, struct.pack('<III', 1, wblk, romblk))
        r = uw.recv_rsp(self.ser, timeout)
        if not r:
            return False, '0x15 无响应'
        if r[1] != 0:
            return False, '0x15 状态=%d' % r[1]
        en = struct.unpack('<I', r[2][:4])[0] if len(r[2]) >= 4 else -1
        return (en == 1), '板上自动循环 en=%d (wblk=%d romblk=%d)' % (en, wblk, romblk)

    def stop_auto(self):
        """停板上识别循环（0x15 en=0）。"""
        self.ser.reset_input_buffer()
        uw.send(self.ser, CMD_AUTO_RUN, struct.pack('<III', 0, 480000, 580000))
        r = uw.recv_rsp(self.ser, 2.0)
        if not r:
            return False, '0x15 off 无响应'
        return (r[1] == 0), '已发 0x15 off (st=%d)' % r[1]

    def set_thr(self, conf):
        """conf(0~1) -> 两个尺度的 int16 阈值。thr_q = ln(c/(1-c))/s_out。"""
        import math
        s0, s1 = 3.3176e-4, 3.7205e-4
        q0 = int(round(math.log(conf / (1.0 - conf)) / s0))
        q1 = int(round(math.log(conf / (1.0 - conf)) / s1))
        q0 = max(-32768, min(32767, q0))
        q1 = max(-32768, min(32767, q1))
        st, _ = self.cmd(CMD_SET_THR, struct.pack('<ii', q0, q1))
        return (st == 0), (q0, q1)

    def set_settle(self, cyc=1080000):
        """0x19 (v66): 自动循环"撤 arm 后落盘等待"拍数, 在线扫参压帧率用。
        1080000=20ms(默认); 钳制 108000(2ms)..5405400(100ms)。
        用法: 开自动循环后逐步调小, [AUTO] 行 det 变差即回退一档。"""
        cyc = max(108000, min(5405400, int(cyc)))
        self.ser.reset_input_buffer()
        uw.send(self.ser, 0x19, struct.pack('<I', cyc))
        r = uw.recv_rsp(self.ser, 2.0)
        if not r or r[1] != 0:
            return False, '0x19 无响应/出错'
        v = struct.unpack('<I', r[2][:4])[0]
        return True, 'settle=%d cyc (%.1f ms)' % (v, v / 54054.0)

    # ---------------- 相机抓图 (v67) ----------------
    def mem_read_bulk(self, base, total, prog=None):
        """0x02 分块连读 (480B/次), 返回 bytes; 失败返回 None。"""
        buf = bytearray()
        while len(buf) < total:
            off = len(buf)
            cnt = min(480, total - off)
            self.ser.reset_input_buffer()
            uw.send(self.ser, CMD_MEM_READ, struct.pack('<IH', base + off, cnt))
            r = uw.recv_rsp(self.ser, 3.0)
            if not r or r[1] != 0 or len(r[2]) < cnt:
                return None
            buf += r[2][:cnt]
            if prog:
                prog(len(buf), total)
        return bytes(buf)

    def cam_freeze(self):
        """停自动循环 + 撤 PP arm —— 防止抓到 30fps 轮转覆盖的撕裂帧。"""
        self.stop_auto()
        uw.send(self.ser, CMD_MEM_WRITE,
                struct.pack('<IH', 0x40004000, 4) + struct.pack('<I', 0))
        uw.recv_rsp(self.ser, 2.0)

    def cam_resume(self):
        """恢复自动循环 en=1 (抓图后用; PP 由循环自愈重新 arm)。"""
        uw.send(self.ser, CMD_AUTO_RUN, struct.pack('<III', 1, 480000, 580000))
        uw.recv_rsp(self.ser, 2.0)

    # ---------------- 相机帧快照 (v68, 消除撕裂) ----------------
    def cam_snapshot(self, src_blk=0, dst_blk=500000, nblk=28800, timeout=10.0):
        """0x1B (v68): 经 DDR 桥把相机帧整帧拷到 DDR 暂存区 (CPU 速度 ~130ms,
        带采样校验 + 自动重试)。之后用 get_frame09 读暂存区 => 无撕裂。
        dst 默认 500000 blk (权重区 491328 之后 / 程序暂存 600000 之前的空闲带)。"""
        self.ser.reset_input_buffer()
        uw.send(self.ser, 0x1B, struct.pack('<III', src_blk, dst_blk, nblk))
        r = uw.recv_rsp(self.ser, timeout)
        if not r or r[1] != 0 or len(r[2]) < 8:
            return None
        return {'attempts': r[2][0], 'ok': r[2][1],
                'fp': struct.unpack('<I', r[2][4:8])[0]}

    def get_frame09(self, base_blk=500000, nb=3600, prog=None):
        """0x09 × nb (256B/次) 从暂存区读帧 (无撕裂)。返回 bytes 或 None。"""
        raw = bytearray()
        fail = 0
        for b in range(nb):
            self.ser.reset_input_buffer()
            uw.send(self.ser, 0x09, struct.pack('<I', base_blk + b * 8))
            r = uw.recv_rsp(self.ser, 3.0)
            if not r or r[1] != 0 or len(r[2]) < 256:
                fail += 1
                if fail > 10:
                    return None
                time.sleep(0.2)
                continue
            raw += r[2][:256]
            if prog:
                prog(b * 256, nb * 256)
        if fail:
            self._on_log('[抓图] %d 块读取失败(已跳过)' % fail)
        return bytes(raw[:640 * 720 * 2])

    def ovl_mask(self, mask):
        st, _ = self.cmd(0x14, bytes([mask & 0xFF]))
        return st == 0

    def load_in(self, blk=590000):
        """把 DDR 590000 的静态图输入 DMA 到 y3tram 块 0。
        ★ 注意: 这里只做 DMA, **不会**把 act_in.bin 写进 DDR（要在界面上先"上传 act_in"）。"""
        self._busy.set()
        try:
            from layer_run import Cmd, load_in as _load_in
            c = Cmd(self.ser)
            return bool(_load_in(c, self.ser, blk))
        except Exception as e:
            self.log('[load_in] %s' % e)
            return False
        finally:
            self._busy.clear()

    def ensure_blob(self, force=False):
        """把 arena 整网执行器 sw/netrun_blob.bin 下装到 0x8000。

        ★ 必须做：上电/复位后 arena 是空的（或残留上一个程序）。
          0x03+0x0D 只是"设参数并执行 0x8000"，**不会**自己下装程序；
          早期脚本每次都先跑一遍 demo_full_net，所以这个前提没暴露出来。
        """
        if self._blob_ok and not force:
            return True
        path = os.path.join(PROJ, 'sw', 'netrun_blob.bin')
        if not os.path.exists(path):
            self.log('[执行器] 找不到 %s' % path)
            return False
        ok, msg = self.load_img(path, 0x8000)
        self.log('[执行器] netrun_blob.bin 下装 %s（%s）' % ('OK' if ok else 'FAIL', msg))
        self._blob_ok = ok
        return ok

    def net_run(self, wblk=480000, romblk=580000, timeout=10.0):
        """跑一次整网（0x03 设参数 + 0x0D 执行 blob），返回 dict。"""
        if not self.ensure_blob():
            return {'ok': False, 'r': 'no-blob', 'layers': 0, 'cyc': 0}
        self._busy.set()
        buf = bytearray()
        try:
            self.ser.reset_input_buffer()
            uw.send(self.ser, 0x03, struct.pack('<IH', 0x9F80, 8)
                    + struct.pack('<II', wblk, romblk))
            uw.send(self.ser, CMD_EXEC, struct.pack('<I', 0x8000))
            t0 = time.time()
            while time.time() - t0 < timeout:
                d = self.ser.read(1024)
                if d:
                    buf += d
                    if b'[BLOB] done' in bytes(buf[-300:]):
                        break
        finally:
            self._busy.clear()
        txt = bytes(buf).decode('utf-8', 'replace')
        for ln in txt.splitlines():
            t = ln.strip()
            if not t:
                continue
            # blob 每次都会刷几十行 [RS] 寄存器配置, 对使用者没意义, 过滤掉
            if t.startswith('[RS]'):
                continue
            self.log('  ' + t)
        m = re.search(r'\[BLOB\] done r=([0-9a-fA-F]+).*?layers=(\d+).*?cyc=([0-9a-fA-F]+)', txt)
        if m:
            # r 是 8 位十六进制（如 00000000），必须按数值判 0 —— 早期写成
            # 字符串比较 '0'，导致成功也被标成失败
            rv = int(m.group(1), 16)
            return {'ok': rv == 0, 'r': '%08X' % rv,
                    'layers': int(m.group(2)), 'cyc': int(m.group(3), 16)}
        m = re.search(r'\[BLOB\] done r=([0-9a-fA-F]+)', txt)
        return {'ok': False, 'r': (m.group(1) if m else '?'), 'layers': 0, 'cyc': 0}

    def det_read(self, conf=0.25):
        import det_read as dr
        self._busy.set()
        try:
            import json
            P = json.load(open(dr.DEF_PARAMS, encoding='utf-8'))
            tm = {k: tuple(v) for k, v in P['tmeta'].items()}
            byname = {j['name']: j for j in P['jobs']}
            tbl = dr.parse_table(dr.DEF_TABLE)
            for _ in range(3):
                try:
                    dets, _ = dr.read_heads(self.ser, tbl, byname, tm, conf)
                    return dets, tbl, byname, tm
                except SystemExit:
                    time.sleep(0.15)
            return None, tbl, byname, tm
        finally:
            self._busy.clear()

    def draw(self, dets, tbl, byname, tm):
        import det_read as dr
        self._busy.set()
        try:
            dr.clear_ovl(self.ser)
            if dets:
                dr.draw(self.ser, dets[:8], 0)
        finally:
            self._busy.clear()

    def pp_grab(self, wait_ms=1500):
        """让 PP 处理一帧并等 letterbox+pad 完成（pad_done, bit8）。
        返回 (ok, frame_cnt, pad_done)。"""
        self._busy.set()
        try:
            self.wr32(PP_CSR, 0)
            time.sleep(0.06)
            fc0 = (self.rd32(PP_CSR) or 0) >> 10 & 0xFF
            self.wr32(PP_CSR, 1)
            t0 = time.time()
            hit = False
            while (time.time() - t0) * 1000 < wait_ms:
                st = self.rd32(PP_CSR) or 0
                if ((st >> 10) & 0xFF) != fc0 and (st >> 8) & 1:
                    hit = True
                    break
                time.sleep(0.003)
            self.wr32(PP_CSR, 0)
            time.sleep(0.05)          # 等 PP 在途写入落盘
            st = self.rd32(PP_CSR) or 0
            return hit, (st >> 10) & 0xFF, (st >> 8) & 1
        finally:
            self._busy.clear()

    # ---------------- CoreMark ----------------
    def read_cm_result(self):
        d = self.rdblk(CM_RESULT_ADDR, 32)
        if not d or len(d) < 32:
            return None
        magic, iters, ticks, mcycle, minstret, freq, milli, traps = \
            struct.unpack('<8I', d[:32])
        if magic != CM_RESULT_MAGIC:
            return {'bad_magic': magic}
        # ticks 的单位是**时钟周期**（EE_TICKS_PER_SEC = 频率），不是微秒。
        #   迭代/秒      = iters × freq / ticks          (4000×54.054e6/1.0566e9 = 204.63)
        #   CoreMark/MHz = iters × 1e6  / ticks          (4000e6/1.0566e9 = 3.786)
        # 历史上 cm_result.py 把前者写成 iters×1e6/ticks 并再除一次频率，得出 0.0700，
        # 这次在 GUI 里复发了 —— 两个式子分开算，不要再互相推导。
        score  = (iters * float(freq) / ticks) if ticks else 0.0
        cm_mhz = (iters * 1e6 / ticks) if ticks else 0.0
        secs   = (ticks / float(freq)) if freq else 0.0
        return {'iters': iters, 'ticks': ticks, 'mcycle': mcycle,
                'minstret': minstret, 'freq': freq, 'milli': milli,
                'traps': traps, 'score': score, 'cm_mhz': cm_mhz,
                'secs': secs,
                'cpi': (mcycle / minstret) if minstret else 0.0}
