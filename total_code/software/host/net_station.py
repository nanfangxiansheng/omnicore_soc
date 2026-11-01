#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""net_station.py — Y3T 整网上位机（一键上传 / 推理 / 画框 / 看板端日志）

用法
    /e/anconda/python.exe tools/pc/net_station.py

界面分区
    ① 连接     选串口 + 波特率(默认 460800)，连接后开始接收板子打印
    ② 数据准备  权重(2.9MB) / 尺度(22KB) / 静态图输入(288KB) 上传，逐帧应答 + 全量校验
    ③ 推理      「静态图整网」走 act_in；「一帧相机」让 PP 预处理一帧再推理；「连续」按次数循环
    ④ 显示      框使能、清框、阈值(0.15~0.9)
    ⑤ 日志      板子打印（自检/识别/t=Ns 心跳）+ 上位机操作记录

注意
    · 静态图必须**先上传 act_in**：固件的 load_in 只做 DMA（DDR→y3tram），
      不会把 act_in.bin 写进 DDR。漏了这一步 L0 就吃到旧数据，检出框全错。
    · 相机帧要等 PP 的 pad_done（letterbox 完成，约 127 ms）；只等"帧号变化"
      会拿到半成品，置信度整体偏低（历史踩坑）。
"""
import os
import queue
import re
import struct
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import board_lib as bl                                        # noqa: E402

WEIGHTS = os.path.join(bl.PROJ, 'sw', 'engine_weights.bin')
# ★ 尺度文件配对（传反不报错、只是结果错）：
#   当前 v65 片：固件(y3t_exec) 与 arena blob 都用【对齐版 aligned】——保持此配置；
#   将来烧 v71 片（固件改用紧排表）时，才把这行换回 'engine_scales.bin'（紧排）。
SCALES = os.path.join(bl.PROJ, 'sw', 'engine_scales_aligned.bin')
ACTIN = os.path.join(bl.PROJ, 'sw', 'act_in.bin')

# 存盘目录固定在工程根下(绝对路径), 不随启动目录漂移
DATASET = os.path.join(bl.PROJ, 'dataset')              # 训练帧 / 截图
CAPS = os.path.join(bl.PROJ, 'caps')                    # 640×720 整幅抓图

WBLK, ROMBLK, INBLK = 480000, 580000, 590000


class App(object):
    def __init__(self, root):
        self.root = root
        self.board = None
        self.q = queue.Queue()
        self.busy = False
        self.auto_on = False          # 板上自动循环状态 (抓图后决定是否恢复)
        self.stop_collect = False     # 采集停止标志
        self._collect_n = 0
        self._collect_tag = ''
        root.title(' 整网上位机  v65  (FPGA PG2L200H · 29 层 )')
        root.geometry('980x760')
        self._build()
        self.root.after(80, self._drain)

    # ---------------- 界面 ----------------
    def _build(self):
        pad = dict(padx=6, pady=4)

        top = ttk.LabelFrame(self.root, text='① 连接')
        top.pack(fill='x', **pad)
        ttk.Label(top, text='串口').pack(side='left', padx=(8, 2))
        self.cb_port = ttk.Combobox(top, width=10, values=bl.list_ports())
        self.cb_port.pack(side='left')
        if self.cb_port['values']:
            self.cb_port.set(self.cb_port['values'][0])
        ttk.Button(top, text='刷新', width=6, command=self._refresh_ports).pack(side='left', padx=4)
        ttk.Label(top, text='波特率').pack(side='left', padx=(10, 2))
        self.cb_baud = ttk.Combobox(top, width=9, values=['460800', '115200'],
                                    state='readonly')
        self.cb_baud.set('460800')
        self.cb_baud.pack(side='left')
        self.btn_conn = ttk.Button(top, text='连接', width=10, command=self._toggle_conn)
        self.btn_conn.pack(side='left', padx=8)
        self.lb_state = ttk.Label(top, text='未连接', foreground='#b00')
        self.lb_state.pack(side='left', padx=6)

        mid = ttk.Frame(self.root)
        mid.pack(fill='both', expand=False, **pad)

        # ② 数据准备
        f2 = ttk.LabelFrame(mid, text='② 数据准备（上传到 DDR，逐帧应答 + 全量校验）')
        f2.pack(side='left', fill='both', expand=True, padx=(0, 6))
        ttk.Button(f2, text='一键全传（权重+尺度+act_in）　约 3.5 分钟', width=38,
                   command=lambda: self._task(self._upload_all)).pack(fill='x', padx=8, pady=(8, 3))
        ttk.Button(f2, text='只传权重  engine_weights.bin  → DDR 480000',
                   command=lambda: self._task(self._up_weights)).pack(fill='x', padx=8, pady=3)
        ttk.Button(f2, text='只传尺度  engine_scales_aligned.bin（对齐版）→ DDR 580000',
                   command=lambda: self._task(self._up_scales)).pack(fill='x', padx=8, pady=3)
        ttk.Button(f2, text='只传 act_in  act_in.bin → DDR 590000（静态图必传）',
                   command=lambda: self._task(self._up_actin)).pack(fill='x', padx=8, pady=3)
        rowv = ttk.Frame(f2)
        rowv.pack(fill='x', padx=8, pady=(3, 8))
        ttk.Label(rowv, text='校验方式').pack(side='left')
        self.cb_verify = ttk.Combobox(rowv, width=30, state='readonly', values=[
            '全量校验',
            '抽验',
            '不校验'])
        self.cb_verify.current(1)          # 默认抽验：比全量快得多，又兜得住大范围损坏
        self.cb_verify.pack(side='left', padx=6)

        # ③ 推理
        f3 = ttk.LabelFrame(mid, text='③ 推理')
        f3.pack(side='left', fill='both', expand=True, padx=(0, 6))
        ttk.Button(f3, text='静态图整网（act_in 静态回归，随机激励出 0 框属正常）', width=38,
                   command=lambda: self._task(self._run_static)).pack(fill='x', padx=8, pady=(8, 3))
        ttk.Button(f3, text='相机帧 × 1（PP 预处理后推理）',
                   command=lambda: self._task(self._run_cam_once)).pack(fill='x', padx=8, pady=3)
        row = ttk.Frame(f3)
        row.pack(fill='x', padx=8, pady=3)
        ttk.Label(row, text='连续帧数').pack(side='left')
        self.ed_n = ttk.Entry(row, width=6)
        self.ed_n.insert(0, '10')
        self.ed_n.pack(side='left', padx=4)
        ttk.Label(row, text='间隔(ms)').pack(side='left', padx=(8, 0))
        self.ed_gap = ttk.Entry(row, width=6)
        self.ed_gap.insert(0, '100')
        self.ed_gap.pack(side='left', padx=4)
        ttk.Button(f3, text='开始连续识别', command=lambda: self._task(self._run_cam_loop)
                   ).pack(fill='x', padx=8, pady=(3, 3))
        # v65: 板上自主循环 —— 固件自己"来帧→推理→解码→画框→GPIO 发布", PC 只看日志
        rowa = ttk.Frame(f3)
        rowa.pack(fill='x', padx=8, pady=(3, 8))
        ttk.Button(rowa, text='板上自动识别 开始（0x15 en=1）',
                   command=lambda: self._task(self._auto_on)).pack(side='left',
                   fill='x', expand=True, padx=(0, 4))
        ttk.Button(rowa, text='停止自动', command=lambda: self._task(self._auto_off)
                   ).pack(side='left', fill='x', expand=True, padx=(4, 0))

        # ④ 显示
        f4 = ttk.LabelFrame(mid, text='④ 显示 / 阈值')
        f4.pack(side='left', fill='both', expand=True)
        ttk.Button(f4, text='打开检测框（0x14 = 0x04）',
                   command=lambda: self._task(self._ovl_on)).pack(fill='x', padx=8, pady=(8, 3))
        ttk.Button(f4, text='清掉所有框',
                   command=lambda: self._task(self._ovl_clear)).pack(fill='x', padx=8, pady=3)
        ttk.Button(f4, text='修竖条（显示对齐 ofs=1）',
                   command=lambda: self._task(self._fix_seam)).pack(fill='x', padx=8, pady=3)
        row2 = ttk.Frame(f4)
        row2.pack(fill='x', padx=8, pady=3)
        ttk.Label(row2, text='置信度阈值').pack(side='left')
        self.ed_thr = ttk.Entry(row2, width=7)
        self.ed_thr.insert(0, '0.25')
        self.ed_thr.pack(side='left', padx=4)
        ttk.Button(row2, text='应用', width=6,
                   command=lambda: self._task(self._apply_thr)).pack(side='left')
        ttk.Button(f4, text='停板上识别循环（0x15 off）',
                   command=lambda: self._task(self._stop_auto)).pack(fill='x', padx=8, pady=(3, 8))

        # ⑤ 相机抓图 (v67): DDR 帧槽 / y3tram IN 区 → PNG
        f5 = ttk.LabelFrame(mid, text='⑤ 相机抓图')
        f5.pack(side='left', fill='both', expand=True, padx=(6, 0))
        ttk.Button(f5, text='抓相机帧 640×720（待 RTL 写门控）',
                   command=lambda: self._task(self._grab_frame)).pack(fill='x', padx=8, pady=(8, 3))
        ttk.Button(f5, text='读网络输入 96×96 (~18s)',
                   command=lambda: self._task(self._grab_in96)).pack(fill='x', padx=8, pady=3)
        # 训练数据批量采集 (v69): 每帧 arm PP -> 等 400ms -> 撤 arm -> 读 IN -> 存盘
        rowc = ttk.Frame(f5)
        rowc.pack(fill='x', padx=8, pady=(6, 3))
        ttk.Label(rowc, text='帧数').pack(side='left')
        self.ed_ncol = ttk.Entry(rowc, width=5)
        self.ed_ncol.insert(0, '20')
        self.ed_ncol.pack(side='left', padx=(2, 6))
        ttk.Label(rowc, text='标签').pack(side='left')
        self.ed_tag = ttk.Entry(rowc, width=8)
        self.ed_tag.pack(side='left', padx=2)
        ttk.Button(f5, text='开始采集训练帧 (18s/帧)',
                   command=self._collect_start).pack(fill='x', padx=8, pady=3)
        self.btn_stop_collect = ttk.Button(f5, text='停止采集',
                                           command=self._collect_stop)
        self.btn_stop_collect.pack(fill='x', padx=8, pady=(3, 8))
        ttk.Label(f5, text='存入 dataset/ (PNG+npy+manifest)\n序号自动接续, 重复采集不覆盖;\n采集中可随时点"停止采集"。').pack(
            fill='x', padx=8, pady=(0, 8))
        ttk.Label(f5, text='抓取前自动冻结防撕裂;\nPNG 存 caps/ 目录;\n若之前在跑自动循环,\n抓完自动恢复。').pack(
            fill='x', padx=8, pady=(3, 8))

        # 进度
        pr = ttk.Frame(self.root)
        pr.pack(fill='x', padx=8)
        self.pb = ttk.Progressbar(pr, mode='determinate', maximum=100.0)
        self.pb.pack(side='left', fill='x', expand=True)
        self.lb_stat = ttk.Label(pr, text='就绪', width=42, anchor='w')
        self.lb_stat.pack(side='left', padx=8)

        # ⑤ 日志
        f5 = ttk.LabelFrame(self.root, text='⑤ 板端输出 / 操作日志')
        f5.pack(fill='both', expand=True, padx=6, pady=6)
        self.txt = scrolledtext.ScrolledText(f5, height=18, font=('Consolas', 9),
                                            background='#111', foreground='#d0f0c0',
                                            insertbackground='#d0f0c0')
        self.txt.pack(fill='both', expand=True, padx=6, pady=(6, 0))
        bar = ttk.Frame(f5)
        bar.pack(fill='x', padx=6, pady=6)
        ttk.Button(bar, text='清空日志', command=lambda: self.txt.delete('1.0', 'end')
                   ).pack(side='left')
        ttk.Button(bar, text='保存日志', command=self._save_log).pack(side='left', padx=6)
        ttk.Label(bar, text='提示：静态图前务必先传 act_in；相机帧需等 pad_done(≈127ms)',
                  foreground='#555').pack(side='right')

    # ---------------- 基础设施 ----------------
    def _refresh_ports(self):
        v = bl.list_ports()
        self.cb_port['values'] = v
        if v and not self.cb_port.get():
            self.cb_port.set(v[0])

    def _log(self, s):
        self.q.put(('log', s))

    def _drain(self):
        try:
            while True:
                item = self.q.get_nowait()
                if item[0] == 'log':
                    self.txt.insert('end', str(item[1]) + '\n')
                    self.txt.see('end')
                elif item[0] == 'stat':
                    self.lb_stat.config(text=item[1])
                elif item[0] == 'prog':
                    d, t = item[1], max(1e-9, item[2])
                    self.pb['value'] = 100.0 * d / t
                elif item[0] == 'done':
                    self.busy = False
                    self._enable(True)
        except queue.Empty:
            pass
        self.root.after(80, self._drain)

    def _enable(self, on):
        state = 'normal' if on else 'disabled'
        def walk(w):
            for c in w.winfo_children():
                if isinstance(c, (ttk.Button,)):
                    try:
                        c.config(state=state)
                    except Exception:
                        pass
                walk(c)
        walk(self.root)

    def _task(self, fn):
        if self.busy:
            return
        if not self.board:
            messagebox.showwarning('未连接', '请先在①里连接串口')
            return
        self.busy = True
        self._enable(False)
        if self.board:
            self.board.pause_listen(True)      # 长任务期间不抢串口
        def run():
            try:
                fn()
            except Exception as e:
                self._log('[异常] %r' % (e,))
            finally:
                if self.board:
                    self.board.pause_listen(False)
                self.q.put(('prog', 0, 1))
                self.q.put(('stat', '就绪'))
                self.q.put(('done',))
        threading.Thread(target=run, daemon=True).start()

    def _toggle_conn(self):
        if self.board:
            self.board.close()
            self.board = None
            self.btn_conn.config(text='连接')
            self.lb_state.config(text='未连接', foreground='#b00')
            self._log('[连接] 已断开')
            return
        port = self.cb_port.get().strip()
        baud = int(self.cb_baud.get())
        b = bl.Board(port, baud, on_log=lambda s: self._log(s))
        try:
            ok = b.open()
        except Exception as e:
            messagebox.showerror('打开失败', '%s\n%s' % (port, e))
            return
        if not ok:
            b.close()
            messagebox.showerror('无响应',
                                 '%s 上没有 PING 应答。\n'
                                 '串口助手是否占着该端口？或板子需要复位/重下比特流。' % port)
            return
        self.board = b
        b.start_listen()
        # v64 片上电/复位后 ofs 会回 0，竖条就出来了 —— 连上就自动修一次
        try:
            b.fix_seam()
        except Exception as e:
            self._log('[竖条] 自动修正失败: %r' % (e,))
        self.btn_conn.config(text='断开')
        self.lb_state.config(text='已连接 %s @%d' % (port, baud), foreground='#070')
        self._log('[连接] %s @%d  PING OK，开始接收板端输出' % (port, baud))

    def _save_log(self):
        p = os.path.join(bl.PROJ, 'log', 'net_station_%s.txt'
                         % time.strftime('%Y%m%d_%H%M%S'))
        os.makedirs(os.path.dirname(p), exist_ok=True)
        open(p, 'w', encoding='utf-8').write(self.txt.get('1.0', 'end'))
        self._log('[日志] 已保存 %s' % p)

    @staticmethod
    def _unique_path(directory, stem, ext):
        """目标文件已存在时追加 _1/_2…, 杜绝同秒/同名覆盖"""
        p = os.path.join(directory, stem + ext)
        k = 1
        while os.path.exists(p):
            p = os.path.join(directory, '%s_%d%s' % (stem, k, ext))
            k += 1
        return p

    # ---------------- 数据准备 ----------------
    def _up_weights(self):
        self._upload(WEIGHTS, WBLK, '权重')
    def _up_scales(self):
        self._upload(SCALES, ROMBLK, '尺度')
    def _up_actin(self):
        self._upload(ACTIN, INBLK, 'act_in')

    def _verify_mode(self):
        """按前缀识别，不依赖完整文案 —— 标签随便改也不会 KeyError
        （2026-10-27 踩坑：值 '抽验 ' 末尾多个空格，精确匹配直接 KeyError）"""
        v = self.cb_verify.get().strip()
        if v.startswith('不校验'):
            return 'none'
        if v.startswith('全量'):
            return 'full'
        return 'sample'                      # 默认/抽验

    def _upload(self, path, base, tag):
        if not os.path.exists(path):
            self._log('[上传] 找不到 %s' % path)
            return
        mode = self._verify_mode()
        self.q.put(('stat', '上传 %s（%s）…' % (tag, mode)))
        ok, msg = self.board.upload(path, base, verify=mode,
                                    prog=lambda d, t: self.q.put(('prog', d, t)),
                                    tag=tag)
        self._log('[上传] %s: %s' % (tag, msg))
        self.q.put(('stat', '%s %s' % (tag, 'OK' if ok else 'FAIL')))

    def _upload_all(self):
        self._up_weights()
        self._up_scales()
        self._up_actin()
        self._log('[上传] 三样齐全，可以跑静态图整网')

    # ---------------- 推理 ----------------
    def _report(self, tag, r, dets):
        # 类别序号 = 训练侧 classes.txt 顺序（新模型 2026-10-28：bottle,mouse,cup,toy,screen）
        names = ['bottle', 'mouse', 'cup', 'toy', 'screen']
        if r:
            self._log('[整网] %s  r=%s layers=%d cyc=0x%X %s'
                      % (tag, r.get('r'), r.get('layers', 0), r.get('cyc', 0),
                         'OK' if r.get('ok') else 'FAIL'))
        if dets is None:
            self._log('[解码] 读检测头失败（整网可能没跑成）')
            return
        self._log('[检出] %s: %d 框' % (tag, len(dets)))
        for d in dets:
            c = names[d['cls']] if d['cls'] < 5 else str(d['cls'])
            self._log('    %-6s conf=%.2f 中心=(%.0f,%.0f) 尺寸=%.0fx%.0f'
                      % (c, d['conf'], d['xywh96'][0], d['xywh96'][1],
                         d['xywh96'][2], d['xywh96'][3]))

    def _thr(self):
        try:
            return float(self.ed_thr.get())
        except Exception:
            return 0.25

    def _pre_infer(self):
        """推理前必须先停掉固件自己的识别循环（0x15 off）。
        否则固件循环在跑：它会打印 [AUTO]/[LOOP] 干扰 blob 的完成标记解析，
        还会和我们的整网抢 y3tram 检测头 —— 实测会导致 r=? 且框高度为 0。"""
        self.board.stop_auto()
        self._log('[准备] 已发 0x15 off（固件循环若在跑会污染整网）')
        time.sleep(0.15)

    def _run_static(self):
        self.q.put(('stat', '静态图整网…'))
        self._pre_infer()
        if not self.board.load_in(INBLK):
            self._log('[整网] load_in 失败（DDR 590000 有 act_in 吗？）')
        r = self.board.net_run(WBLK, ROMBLK)
        dets, tbl, byname, tm = self.board.det_read(self._thr())
        if dets is not None:                 # 0 个也要画(=清掉上一帧的旧框)
            self.board.draw(dets, tbl, byname, tm)
        self._report('静态图', r, dets)

    def _run_cam_once(self):
        self.q.put(('stat', '抓相机帧…'))
        self._pre_infer()
        ok, fc, pd = self.board.pp_grab()
        self._log('[PP] 抓帧=%s frame=%d pad_done=%d' % (ok, fc, pd))
        if not ok:
            self._log('[PP] 没等到 pad_done（超时）—— 结果可能是半成品，置信度会偏低')
        r = self.board.net_run(WBLK, ROMBLK)
        dets, tbl, byname, tm = self.board.det_read(self._thr())
        if dets is not None:                 # 0 个也要画(=清掉上一帧的旧框)
            self.board.draw(dets, tbl, byname, tm)
        self._report('相机帧', r, dets)

    def _run_cam_loop(self):
        try:
            n = int(self.ed_n.get())
            gap = int(self.ed_gap.get()) / 1000.0
        except Exception:
            n, gap = 10, 0.1
        self._pre_infer()
        for i in range(n):
            self.q.put(('stat', '连续识别 %d/%d' % (i + 1, n)))
            self.q.put(('prog', i + 1, n))
            ok, fc, pd = self.board.pp_grab()
            r = self.board.net_run(WBLK, ROMBLK)
            dets, tbl, byname, tm = self.board.det_read(self._thr())
            if dets is not None:
                self.board.draw(dets, tbl, byname, tm)
            self._report('第%d帧' % (i + 1), r, dets)
            time.sleep(max(0.0, gap))

    # ---------------- 板上自动识别 (0x15) ----------------
    def _auto_on(self):
        self.q.put(('stat', '启动板上自动识别…'))
        ok, msg = self.board.start_auto(WBLK, ROMBLK)
        self._log('[自动] %s' % msg)
        self.auto_on = bool(ok)
        if ok:
            self._log('[自动] 板子自主运行中: 来帧→PP→整网→解码→画框→GPIO 发布。')
            self._log('[自动] 结果只体现在: ①屏幕画框 ②LED 类别码 ③本窗口串口日志;')
            self._log('[自动] 前提: 权重/尺度已在 DDR（漏了就先"一键全传"再重开）。')
            self._log('[自动] 提示: 混用"连续识别"前务必先点"停止自动"。')
        else:
            self._log('[自动] 启动失败 —— 检查是否已上传权重/尺度')

    # ---------------- 相机抓图 (v67) ----------------
    def _grab_frame(self):
        """v68: 0x1B 快照 (消除撕裂) → 0x09 读暂存区 → 640×720 RGB565 → PNG。
        自动循环无需暂停 (快照与引擎/PP 通路无关)。"""
        import numpy as np
        from PIL import Image
        NB = 3600                                   # 3600 × 8块 × 32B = 921,600B
        self.q.put(('stat', '快照+读帧 (~40s)...'))
        self._log('[抓图] 快照 (DDR 桥整帧拷贝, 消除撕裂)...')
        snap = self.board.cam_snapshot()
        if not snap or not snap['ok']:
            self._log('[抓图] 快照失败 %s' % (snap,))
            self._log('[抓图] 原因: 相机 30fps 下帧槽覆盖周期 ≤66ms, 而 CPU 拷贝 ~186ms,')
            self._log('[抓图] 必然被覆盖 —— 全幅抓图需 RTL 加 DVP 写门控后才能用。')
            self._log('[抓图] 现阶段训练数据请用「读网络输入 96×96」(干净无撕裂)')
            return
        self._log('[抓图] 快照完成 (尝试 %d 次, fp=%08X)' % (snap['attempts'], snap['fp']))

        def prog(c, t):
            self.q.put(('prog', c, t))

        raw = self.board.get_frame09(500000, NB, prog)
        if raw is None:
            self._log('[抓图] 暂存区读回失败')
            return
        px = np.frombuffer(raw, dtype='<u2').reshape(720, 640)
        r = (px >> 11) & 0x1F
        g = (px >> 5) & 0x3F
        b2 = px & 0x1F
        rgb = np.dstack([(r << 3) | (r >> 2), (g << 2) | (g >> 4),
                         (b2 << 3) | (b2 >> 2)]).astype(np.uint8)
        os.makedirs(CAPS, exist_ok=True)
        p = self._unique_path(CAPS, time.strftime('cam_%Y%m%d_%H%M%S'), '.png')
        Image.fromarray(rgb, 'RGB').save(p)
        self._log('[抓图] 已保存 %s (640x720, 无撕裂)' % p)
        self.q.put(('stat', '就绪'))

    def _grab_in96(self):
        """y3tram IN 区 → 模型实际看到的 96×96 输入 (HWC 帧填充布局)。
        v69: 自己 arm PP 抓一帧再读 (不依赖自动循环是否在跑; 否则复位后
        IN 区是上电残留 -> 纯黑)。"""
        import numpy as np
        from PIL import Image
        D, C = 96, 16
        total = 294912
        self.q.put(('stat', '读网络输入 96×96 (~18s)...'))
        self._log('[抓图] 冻结自动循环...')
        self.board.stop_auto()
        time.sleep(0.1)
        self._log('[抓图] arm PP 抓一帧 (等 400ms)...')
        self.board.cmd(0x03, struct.pack('<IH', 0x40004000, 4) + struct.pack('<I', 1))
        time.sleep(0.4)
        self.board.cmd(0x03, struct.pack('<IH', 0x40004000, 4) + struct.pack('<I', 0))
        time.sleep(0.05)

        def prog(c, t):
            self.q.put(('prog', c, t))

        raw = self.board.mem_read_bulk(0x60000000, total, prog)
        if raw is None:
            self._log('[抓图] 读 y3tram IN 失败')
            return
        i16 = np.frombuffer(raw[:total], dtype='<i2').reshape(D * D, C)
        q = i16[:, :3].astype(np.float64)
        qmax = max(q.max(), 1)
        u8 = np.clip(q * 255.0 / qmax, 0, 255).astype(np.uint8).reshape(D, D, 3)
        if qmax < 64:
            self._log('[抓图] 警告: qmax=%d 极小 —— IN 区可能没被 PP 写 (相机没出流?)' % qmax)
        os.makedirs(CAPS, exist_ok=True)
        p = self._unique_path(CAPS, time.strftime('netin_%Y%m%d_%H%M%S'), '.png')
        Image.fromarray(u8, 'RGB').save(p)
        np.save(os.path.splitext(p)[0] + '.npy', i16.reshape(D, D, C))
        self._log('[抓图] 已保存 %s (96x96, qmax=%d, 亮度自动拉伸; npy 同存)' % (p, qmax))
        if self.auto_on:
            self.board.cam_resume()
            self._log('[抓图] 自动循环已恢复')
        self.q.put(('stat', '就绪'))

    # ---------------- 训练数据批量采集 (v69) ----------------
    def _collect_stop(self):
        self.stop_collect = True
        self._log('[采集] 已请求停止 (当前帧读完后退出)')

    def _collect_start(self):
        try:
            n = int(self.ed_ncol.get())
        except Exception:
            n = 20
        n = max(1, min(2000, n))
        tag = self.ed_tag.get().strip()
        self.stop_collect = False
        self._collect_n = n
        self._collect_tag = tag
        self._log('[采集] 开始 %d 帧 -> dataset/ (标签=%s, 约 %.0f 分钟)'
                  % (n, tag or '无', n * 18 / 60.0))
        self.q.put(('prog', 0, n))
        self._task(self._collect_run)
        # _task 会把所有按钮 disable, 这里把"停止采集"重新放开
        self.root.after(200, lambda: self.btn_stop_collect.config(state='normal'))

    def _collect_run(self):
        import numpy as np
        from PIL import Image
        D, C = 96, 16
        total = 294912
        n, tag = self._collect_n, self._collect_tag
        prefix = (tag + '_') if tag else ''
        self.board.stop_auto()
        time.sleep(0.1)
        os.makedirs(DATASET, exist_ok=True)
        # 序号接续: 接着该标签已有帧的最大序号往下编, 重复采集不再覆盖旧数据
        # (png/npy 都计入, 避免只删了 png 留下 npy 时撞名)
        start = 0
        for fn in os.listdir(DATASET):
            m = re.match(r'^frame_%s(\d+)\.(?:png|npy)$' % re.escape(prefix), fn)
            if m:
                start = max(start, int(m.group(1)) + 1)
        if start:
            self._log('[采集] 检测到已有 %d 帧, 序号从 %04d 接续 (不覆盖)' % (start, start))
        mf = open(os.path.join(DATASET, 'manifest.txt'), 'a', encoding='utf-8')
        done = 0
        try:
            while done < n and not self.stop_collect:
                self.q.put(('stat', '采集 %d/%d: arm PP' % (done + 1, n)))
                self.board.cmd(0x03, struct.pack('<IH', 0x40004000, 4) + struct.pack('<I', 1))
                time.sleep(0.4)
                self.board.cmd(0x03, struct.pack('<IH', 0x40004000, 4) + struct.pack('<I', 0))
                time.sleep(0.03)
                self.q.put(('stat', '采集 %d/%d: 读 IN 295KB' % (done + 1, n)))
                raw = self.board.mem_read_bulk(0x60000000, total)
                if raw is None:
                    self._log('[采集] 第 %d 帧读失败, 重试' % (done + 1))
                    continue
                i16 = np.frombuffer(raw, dtype='<i2').reshape(D * D, C)
                q = i16[:, :3].astype(np.float64)
                qmax = max(q.max(), 1)
                u8 = np.clip(q * 255.0 / qmax, 0, 255).astype(np.uint8).reshape(D, D, 3)
                name = 'frame_%s%04d' % (prefix, start + done)
                Image.fromarray(u8, 'RGB').save(os.path.join(DATASET, name + '.png'))
                np.save(os.path.join(DATASET, name + '.npy'), i16.reshape(D, D, C))
                mf.write('%s %s qmax=%d\n' % (name, time.strftime('%H:%M:%S'), qmax))
                mf.flush()
                done += 1
                self.q.put(('prog', done, n))
                self._log('[采集] %s 已存 (qmax=%d)%s'
                          % (name, qmax, '' if qmax >= 64 else '   ⚠ 极小, 检查相机/光照'))
        finally:
            mf.close()
        self._log('[采集] 结束: %d/%d 帧 -> %s (序号 %04d~%04d)'
                  % (done, n, DATASET, start, start + done - 1))

    def _auto_off(self):
        ok, msg = self.board.stop_auto()
        self.auto_on = False
        self._log('[自动] %s%s' % (msg, '' if ok else '（可重试或复位）'))
        self.q.put(('stat', '就绪'))

    # ---------------- 显示 ----------------
    def _ovl_on(self):
        ok = self.board.ovl_mask(0x04)
        # 0x14 只改固件的 g_ovl_mask；OVL_EN 由画框时那一帧才点亮，所以此刻读到 0 是正常的
        self._log('[显示] 框掩码 -> 0x04 %s（下次画框时生效）' % ('OK' if ok else 'FAIL'))

    def _ovl_clear(self):
        self.board.draw([], None, None, None)
        self._log('[显示] 已清框')

    def _fix_seam(self):
        ok, got = self.board.fix_seam()
        self._log('[竖条] %s（回读 ofs=%d）' % ('已修正' if ok else '写失败', got))

    def _apply_thr(self):
        conf = self._thr()
        ok, q = self.board.set_thr(conf)
        self._log('[阈值] conf=%.2f -> thr_q=%s  %s' % (conf, q, 'OK' if ok else 'FAIL'))

    def _stop_auto(self):
        self.board.stop_auto()
        self._log('[循环] 已发送 0x15 off')


def main():
    root = tk.Tk()
    try:
        root.call('tk', 'scaling', 1.15)
    except Exception:
        pass
    App(root)
    root.mainloop()


if __name__ == '__main__':
    main()
