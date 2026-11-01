#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""coremark_station.py — CoreMark 上位机（一键下装 + 运行 + 取分 + 看板端日志）

用法
    /e/anconda/python.exe tools/pc/coremark_station.py

流程（点一次按钮全做完）
    1. 0x15 en=0        停掉板上识别循环，避免抢 DDR
    2. 0x0C 分块下装     sw/coremark_port/coremark_arena.bin → arena 0x8000
    3. 0x0D 执行         CoreMark 开始跑（4000 iterations，约 19.5 s）
    4. 收日志           v65 起 CoreMark 写 BAUDDIV=0（回落 460800，与固件一致），
                        上位机全程 460800，不再切波特率
    5. 读结果块 0xFFE0   8×u32: magic/iters/ticks/mcycle/minstret/freq/milli/traps
                        迭代/秒 = iters×freq/ticks ; CoreMark/MHz = iters×1e6/ticks

注意
    · v65 起 CoreMark 全程 460800，跑完直接可以继续用整网上位机，不需要复位。
    · 串口文本里的 "Total ticks" 与 0xFFE0 的内存块互为备份；内存路径不受板端
      心跳打印串扰，是可靠判据（文本只用来给人看）。
"""
import os
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import board_lib as bl                                        # noqa: E402

CM_BIN = os.path.join(bl.PROJ, 'sw', 'coremark_port', 'coremark_arena.bin')
DONE_MARK = '[CoreMark finished]'
LABELS = [('iters', '迭代次数 Iterations'), ('ticks', '总时钟 ticks'),
          ('secs', '运行时长 (s)'), ('score', '迭代/秒 iterations/s'),
          ('cm_mhz', 'CoreMark/MHz'), ('cpi', 'CPI'),
          ('mcycle', 'mcycle'), ('minstret', 'minstret'),
          ('traps', '陷阱次数 trap_total')]


class App(object):
    def __init__(self, root):
        self.root = root
        self.board = None
        self.q = queue.Queue()
        self.busy = False
        root.title('CoreMark 上位机  v65  (OmniCore fusion · RV32IMAC @54.054MHz)')
        root.geometry('900x700')
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
        ttk.Button(top, text='刷新', width=6, command=self._refresh).pack(side='left', padx=4)
        ttk.Label(top, text='波特率').pack(side='left', padx=(10, 2))
        self.cb_baud = ttk.Combobox(top, width=9, values=['460800', '115200'],
                                    state='readonly')
        self.cb_baud.set('460800')
        self.cb_baud.pack(side='left')
        self.btn_conn = ttk.Button(top, text='连接', width=10, command=self._toggle_conn)
        self.btn_conn.pack(side='left', padx=8)
        self.lb_state = ttk.Label(top, text='未连接', foreground='#b00')
        self.lb_state.pack(side='left', padx=6)
        ttk.Label(top, text='omini',
                  foreground='#555').pack(side='right', padx=8)

        run = ttk.LabelFrame(self.root, text='② 运行')
        run.pack(fill='x', **pad)
        ttk.Button(run, text='一键跑 CoreMark（下装 → 执行 → 自动切波特率 → 取分）',
                   width=58, command=lambda: self._task(self._run_all)
                   ).pack(side='left', padx=8, pady=8)
        ttk.Button(run, text='只读结果（已跑完）', command=lambda: self._task(self._read_only)
                   ).pack(side='left', padx=6)
        ttk.Button(run, text='停识别循环 0x15 off', command=lambda: self._task(self._stop_auto)
                   ).pack(side='left', padx=6)

        res = ttk.LabelFrame(self.root, text='③ 结果')
        res.pack(fill='x', **pad)
        self.vars = {}
        for i, (k, name) in enumerate(LABELS):
            r, c = divmod(i, 4)
            ttk.Label(res, text=name, width=22, anchor='e').grid(row=r, column=c * 2,
                                                                padx=(10, 4), pady=5, sticky='e')
            v = tk.StringVar(value='—')
            self.vars[k] = v
            ttk.Label(res, textvariable=v, width=16, anchor='w',
                      font=('Consolas', 11, 'bold'), foreground='#036').grid(
                row=r, column=c * 2 + 1, padx=(0, 18), pady=5, sticky='w')

        pr = ttk.Frame(self.root)
        pr.pack(fill='x', padx=8)
        self.pb = ttk.Progressbar(pr, mode='determinate', maximum=100.0)
        self.pb.pack(side='left', fill='x', expand=True)
        self.lb_stat = ttk.Label(pr, text='就绪', width=40, anchor='w')
        self.lb_stat.pack(side='left', padx=8)

        f5 = ttk.LabelFrame(self.root, text='④ 板端输出 / 操作日志')
        f5.pack(fill='both', expand=True, padx=6, pady=6)
        self.txt = scrolledtext.ScrolledText(f5, height=16, font=('Consolas', 9),
                                            background='#111', foreground='#d0f0c0',
                                            insertbackground='#d0f0c0')
        self.txt.pack(fill='both', expand=True, padx=6, pady=(6, 0))
        bar = ttk.Frame(f5)
        bar.pack(fill='x', padx=6, pady=6)
        ttk.Button(bar, text='清空日志', command=lambda: self.txt.delete('1.0', 'end')
                   ).pack(side='left')
        ttk.Button(bar, text='保存日志', command=self._save_log).pack(side='left', padx=6)

    # ---------------- 基础设施 ----------------
    def _refresh(self):
        v = bl.list_ports()
        self.cb_port['values'] = v
        if v and not self.cb_port.get():
            self.cb_port.set(v[0])

    def _log(self, s):
        self.q.put(('log', s))

    def _drain(self):
        try:
            while True:
                it = self.q.get_nowait()
                if it[0] == 'log':
                    self.txt.insert('end', str(it[1]) + '\n')
                    self.txt.see('end')
                elif it[0] == 'stat':
                    self.lb_stat.config(text=it[1])
                elif it[0] == 'prog':
                    self.pb['value'] = 100.0 * it[1] / max(1e-9, it[2])
                elif it[0] == 'res':
                    for k, v in it[1].items():
                        if k in self.vars:
                            self.vars[k].set(v)
                elif it[0] == 'done':
                    self.busy = False
                    self._enable(True)
        except queue.Empty:
            pass
        self.root.after(80, self._drain)

    def _enable(self, on):
        st = 'normal' if on else 'disabled'
        def walk(w):
            for c in w.winfo_children():
                if isinstance(c, ttk.Button):
                    try:
                        c.config(state=st)
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
                                 '串口助手是否占着？或板子需要复位/重下比特流。' % port)
            return
        self.board = b
        self.btn_conn.config(text='断开')
        self.lb_state.config(text='已连接 %s @%d' % (port, baud), foreground='#070')
        self._log('[连接] %s @%d  PING OK' % (port, baud))

    def _save_log(self):
        p = os.path.join(bl.PROJ, 'log', 'coremark_%s.txt' % time.strftime('%Y%m%d_%H%M%S'))
        os.makedirs(os.path.dirname(p), exist_ok=True)
        open(p, 'w', encoding='utf-8').write(self.txt.get('1.0', 'end'))
        self._log('[日志] 已保存 %s' % p)

    # ---------------- 动作 ----------------
    def _stop_auto(self):
        self.board.stop_auto()
        self._log('[循环] 已发 0x15 off')

    def _run_all(self):
        b = self.board
        if not os.path.exists(CM_BIN):
            self._log('[CoreMark] 找不到 %s' % CM_BIN)
            return

        # 1) 停识别循环
        self.q.put(('stat', '停板上识别循环…'))
        b.stop_auto()
        self._log('[1/5] 0x15 off（避免 DDR 被识别循环抢）')

        # 2) 下装 arena 程序
        self.q.put(('stat', '下装 coremark_arena.bin…'))
        t0 = time.time()
        ok, msg = b.load_img(CM_BIN, 0x8000,
                             prog=lambda d, t: self.q.put(('prog', d, t)))
        self._log('[2/5] 下装 %s  %s  (%.1fs)' % ('OK' if ok else 'FAIL', msg, time.time() - t0))
        if not ok:
            return

        # 3) 执行 —— v65 起 CoreMark 不再改波特率（写 0 回落到 460800，与固件一致），
        #    所以上位机全程 460800，不需要切波特率、也不需要 reopen。
        self._log('[3/4] 0x0D exec @0x8000，CoreMark 开始（理论 19.55 s）')
        b.exec_no_wait(0x8000)

        # 4) 收日志：见到完成标记 / 固件心跳恢复即停
        self.q.put(('stat', 'CoreMark 运行中…'))
        buf = b''
        t0 = time.time()
        finished = False
        why = ''
        timeout = 45.0
        while time.time() - t0 < timeout:
            try:
                d = b.ser.read(512)
            except Exception:
                break
            if d:
                buf += d
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    t = line.decode('utf-8', 'replace').rstrip('\r')
                    if t:
                        self._log(t)
                    if DONE_MARK in t:
                        finished, why = True, '收到完成标记'
                    elif ('detect=' in t) or ('rx=' in t and t.startswith('t=')):
                        finished, why = True, '固件心跳已恢复（CoreMark 已返回）'
                if finished:
                    break
            el = time.time() - t0
            self.q.put(('prog', min(el, timeout * 0.98), timeout))
            self.q.put(('stat', 'CoreMark 运行中… %.1fs' % el))
        el = time.time() - t0
        self._log('[4/4] %s（从 exec 起 %.1fs）'
                  % (why if finished else '未检测到结束标志（按 %.0fs 超时）' % timeout, el))

        # 5) 读结果块（板子仍在 460800，直接读）
        r = b.read_cm_result()
        self._show(r)

    def _read_only(self):
        b = self.board
        self.q.put(('stat', '读结果块 0xFFE0…'))
        r = b.read_cm_result()
        self._show(r)

    def _show(self, r):
        if r is None:
            self._log('[结果] 读 0xFFE0 失败（PORT 无响应）')
            return
        if 'bad_magic' in r:
            self._log('[结果] 魔数不符 0x%08X（期望 0x%08X）—— CoreMark 还没跑过，'
                      '或结果块被覆盖' % (r['bad_magic'], bl.CM_RESULT_MAGIC))
            return
        self.q.put(('res', {
            'iters': '%d' % r['iters'],
            'ticks': '%d' % r['ticks'],
            'mcycle': '%d' % r['mcycle'],
            'minstret': '%d' % r['minstret'],
            'cpi': '%.4f' % r['cpi'],
            'traps': '%d' % r['traps'],
            'score': '%.2f' % r['score'],
            'cm_mhz': '%.4f' % r['cm_mhz'],
            'secs': '%.2f' % r['secs'],
        }))
        self._log('[结果] 结果块 @0x%X:' % bl.CM_RESULT_ADDR)
        self._log('   iterations=%d  total_ticks=%d' % (r['iters'], r['ticks']))
        self._log('   mcycle=%d  minstret=%d  CPI=%.4f  trap_total=%d'
                  % (r['mcycle'], r['minstret'], r['cpi'], r['traps']))
        self._log('   freq=%d Hz  运行时长=%.2fs  →  迭代/秒=%.2f  →  **CoreMark/MHz=%.4f**'
                  % (r['freq'], r['secs'], r['score'], r['cm_mhz']))
        self._log('[提醒] 板子仍在 460800，可直接继续使用整网上位机。')


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
