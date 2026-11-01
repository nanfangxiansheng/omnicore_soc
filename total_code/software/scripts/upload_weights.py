#!/usr/bin/env python3
"""upload_weights.py — 把引擎权重经 UART 上传到板上的 y3tram 权重区

三种模式 (2026-10-19 重写, 目标: 3 MB 从 ~9 小时降到 5 分钟以内):

  --mode ddr     旧路径。0x0B 一条一条写 DDR, 每条 256B, **每条等 ACK**。
                 板上实测 347 ms/条 ⇒ 3 MB 约 9 小时。留着只为对照与救急。

  --mode bulk    大包。0x0B 一条写 4 KB (128 个 256b 块), 固件**静默不回包**
                 (RFC_BULK), 主机连续灌不等待。慢了再用 0x01 PING 探活。
                 这一档已经把 stop-and-wait 的往返彻底去掉, 是纯比特率上限:
                 460800bps ⇒ 3 MB ≈ 270 s; 460800bps ⇒ ≈ 34 s。

  --mode stream  推荐。大包 0x0B 把每批 (默认 64 KB) 灌进 DDR 暂存块
                 W3T_SRC_BLK, 再发一条 0x11 W3T 让 wload_dma 走 CMD 直通
                 同时写 y3tram 和权重正式区。相比 bulk 多做一步校验:
                 0x11 回 {ok_words, err}, 主机逐批核对字数, 少一个字就停。

用法:
  python tools/upload_weights.py --port COM5                      # 默认 stream
  python tools/upload_weights.py --port COM5 --mode bulk --baud 460800
  python tools/upload_weights.py --port COM5 --verify 480000 64   # 抽验 DDR

解释器: pyserial 3.5 装在 **anaconda** 里, 托管 python 没有 ⇒ 必须用
  E:/anconda/python.exe tools/upload_weights.py ...
  直接敲 `python` 可能落到没有 pyserial 的那个, 报 "pip install pyserial"。

板上串口: COM18 = Silicon Labs CP210x USB-UART 桥 (VID:PID=10C4:EA60)。
  其余 COMx 都是系统蓝牙虚拟串口, 别选错。

关于波特率: 板上 UART 的 BAUDDIV 由固件按固定值写入。改 --baud 前先确认
  固件里 BAUDDIV 与之匹配 (见 sw/demo.c 的 UART_BAUDDIV), 否则一定收错。
  拿不准就先 460800 试 PING, 通了再往上提。
"""
import argparse
import json
import os
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("pip install pyserial")
    sys.exit(1)

# ---- 协议常量 (与 sw/cmd_proto.h 一致) ----
SYNC_REQ   = 0xA5
SYNC_RSP   = 0x5A
CMD_PING       = 0x01
CMD_MEM_READ   = 0x02
CMD_WLOAD_RAW  = 0x0B
CMD_W3T        = 0x11
CMD_WLOAD_POLL = 0x06
CMD_GET_FRAME  = 0x09   # {u32 blk} -> 8 块 256B (固件经 DDR 桥读, 不会越界)

W3T_SRC_BLK = 620000       # 与 cmd_proto.h 的 CMD_W3T_SRC_BLK 一致
SCALES_BLK  = 580000       # 逐通道尺度镜像落点 (权重区 480000..574912 之后)
                           # 必须与 sw/y3t_scale_table.h 的 Y3T_SCALE_BLK_DEF 一致
WDDR_BASE   = 480000       # 权重在 DDR 的正式区起点 (避开 0..460800 相机帧槽)

BLK_BYTES   = 32           # 1 个 256bit 块 = 32 字节
BULK_BYTES  = 4096         # RFC_BULK: 单帧最大载荷
BULK_HDR    = 6            # 0x0B 载荷头: u32 blk + u16 len

# stream 模式每批的字节数。受三方面约束:
#   · DDR 暂存区 W3T_SRC_BLK 起要有这么多空间;
#   · 0x11 的 nwords 建议 <= 2048 (见 cmd_proto.h), 2048*32 = 64 KB;
#   · 批太小 → 0x11 的往返次数多; 批太大 → 失败重传代价高。
STREAM_BATCH = 64 * 1024


# --------------------------------------------------------------------------
# 链路层
# --------------------------------------------------------------------------
def _cksum(cmd, payload):
    ck = cmd ^ (len(payload) & 0xFF) ^ (len(payload) >> 8)
    for b in payload:
        ck ^= b
    return ck & 0xFF


def send(ser, cmd, payload):
    ser.write(bytes([SYNC_REQ, cmd, len(payload) & 0xFF, len(payload) >> 8])
              + payload + bytes([_cksum(cmd, payload)]))


def send_blk(ser, cmd, payload, ck):
    """发送一条命令; ck 是**已经算好**的校验字节。

    为什么要单独一个函数: bytes()+payload 拼接在大包时会把 4KB 再拷一份,
    而 pyserial 的 write 一次调用也远快于多次小 write (每次 write 都可能
    触发一个 USB CDC 报文)。这里保证一整帧只用一次 write 发出。
    """
    hdr = bytes([SYNC_REQ, cmd, len(payload) & 0xFF, len(payload) >> 8])
    ser.write(hdr + payload + bytes([ck]))


def recv_rsp(ser, timeout=2.0, max_len=4096):
    """读一条响应帧; 自动按 0x5A 重同步, 抗板端周期性打印串扰。

    为什么必须重同步 (2026-10-19 上板实测):
      固件 bring-up 后每秒打一行 "t=Ns detect=.. rx=.. last=.." (~40B/s),
      这些文本和响应帧在同一 UART 上交织, 会把响应帧从中间切开。旧实现的
      "逐字节状态机 + ser.read(need)" 撞上假 5A 就去 read 一个超长载荷,
      0.5s 超时后整帧作废 → PING 时好时坏 (3 次里挂 1 次)。
      这里改成: 批量收 → 扫 5A → 长度不合理就当假同步丢 1 字节继续扫。
    """
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(64)
        if chunk:
            buf += chunk
        while True:
            i = buf.find(bytes([SYNC_RSP]))
            if i < 0:
                buf.clear()               # 没有 5A, 全是杂散文本
                break
            if i:
                del buf[:i]               # 丢掉 5A 之前的杂散字节
            if len(buf) < 5:
                break                     # 头不完整, 回去再读
            cmd, status = buf[1], buf[2]
            plen = buf[3] | (buf[4] << 8)
            if plen > max_len:
                del buf[:1]               # 假同步: 丢这个 5A, 继续往后扫
                continue
            # 【BUG-FIX 2026-10-23】响应帧**没有**独立校验字节:
            #   固件 rsp_head 只发 5 字节头 (5A cmd|80 st len_lo len_hi) + 载荷,
            #   之后直接是下一帧 —— 见 uart_cmd.c ping_fast 注释"9 字节, 无独立校验字节按协议"。
            #   旧代码 here 写 need = plen + 1 (按"载荷+1 校验"算), 每次都会多吞 1 字节:
            #     · 单条命令时那一字节靠 1 秒一次的 "t=Ns ..." 心跳文本补上
            #       ⇒ 每条命令固定耗时 ~1.02s, 一直被误判成"主循环一轮相机帧差";
            #     · 批量连发时多吞的正是**下一帧的 5A** ⇒ 下一帧整条丢失 ⇒
            #       32 条只回 16 条、8 条只回 4 条 —— 表现为"恰好一半", 触发逐字重发。
            need = plen
            if len(buf) < 5 + need:
                break
            pay = bytes(buf[5:5 + plen])
            del buf[:5 + need]
            return (cmd, status, pay)
    return None


def rsp_ok(ser, timeout=2.0):
    r = recv_rsp(ser, timeout)
    return bool(r) and r[1] == 0


# --------------------------------------------------------------------------
# 上传: 0x0B 单条大包
# --------------------------------------------------------------------------
def chunk_bulk(data, start_blk):
    """把 data 切成 0x0B 的帧载荷列表 [(blk, payload), ...]。

    每条载荷 = u32 blk + u16 dlen + data, 总长不能超过 RFC_BULK (4096)。
    data 长度必须是 32 的倍数 (固件按 256b 块写), 所以切分按 32 对齐。
    """
    payload_max = BULK_BYTES - BULK_HDR                # 4090
    # 【BUG-WORKAROUND 2026-10-22 14:45】帧大小 4064 → 256（8 块）。
    #   板上实测: 一条 0x0B 内部拆成 >=2 批就坏(数据确定性丢失), 单批(8 块)全对。
    #   固件 DDRB_MAX_BEATS 已是 8, 但 4064B/帧 仍会拆成 16 批 ⇒ 仍然坏。
    #   这里把帧压到 256B = 刚好 1 批, 彻底绕开多批问题。
    #   配合 UART 460800, 2.9MB 权重约 1.5 分钟, 可接受。
    #   根因(多批为何坏)仍未定位, 属于绕行, 不是修复。
    body_max = 256
    out = []
    blk = start_blk
    off = 0
    n = len(data)
    while off < n:
        body = data[off:off + body_max]
        if len(body) % BLK_BYTES:
            body = body + b"\x00" * (BLK_BYTES - len(body) % BLK_BYTES)
        pay = struct.pack("<IH", blk, len(body)) + body
        out.append((blk, pay))
        blk += len(body) // BLK_BYTES
        off += len(body)
    return out


def mode_ddr(ser, data, args):
    """旧路径: 每条 256B + 等 ACK。仅作对照。"""
    n = (len(data) + 255) // 256
    print(f"[ddr] {len(data)} bytes / {n} 条 (每条 256B, 等 ACK)")
    blk = args.base_blk
    t0 = time.time()
    for i in range(n):
        chunk = data[i * 256:(i + 1) * 256]
        pay = struct.pack("<IH", blk, len(chunk)) + chunk
        ok = False
        for _ in range(3):
            send(ser, CMD_WLOAD_RAW, pay)
            if rsp_ok(ser, 1.0):
                ok = True
                break
        if not ok:
            print(f"\n[FAIL] chunk {i} (blk {blk})")
            return False
        blk += 8
        if i % 200 == 0:
            el = time.time() - t0
            eta = el / (i + 1) * (n - i - 1)
            print(f"\r  {i+1}/{n}  {el:.0f}s  ETA {eta:.0f}s", end="")
    print(f"\n[ddr] 完成 {time.time()-t0:.0f}s")
    return True


def mode_bulk(ser, data, args, base_blk, quiet_ack=True, pace=0.0):
    """大包: 0x0B 连续灌, 不等回包。返回是否全发出去。

    quiet_ack 依赖固件 bulk_ack==0 (默认)。若固件被设成 1, 每条都会回 4 字节,
    不收就会把 RX 缓冲灌满反压 —— 那种情况下应该走 mode_stream 或先关掉 bulk_ack。
    """
    frames = chunk_bulk(data, base_blk)
    print(f"[bulk] {len(data)} bytes / {len(frames)} 帧 (每帧 <= {BULK_BYTES}B)")
    t0 = time.time()
    # 【2026-10-19 方案C】固件 0x0B 是同步写 DDR (ddrb_write 期间不收 UART),
    #   PC 全速灌时 RX FIFO 溢出丢字节 (板端 rx 计数比发送量少 ~35%)。
    #   这里每帧后强制留 pace 秒, 让固件有空把 UART 排空。
    for i, (blk, pay) in enumerate(frames):
        send_blk(ser, CMD_WLOAD_RAW, pay, _cksum(CMD_WLOAD_RAW, pay))
        if pace > 0:
            time.sleep(pace)
        if i % 64 == 63:
            el = time.time() - t0
            sent = min((i + 1) * (BULK_BYTES - BULK_HDR), len(data))
            eta = el / sent * (len(data) - sent) if sent else 0
            print(f"\r  {sent}/{len(data)}  {el:.0f}s  ETA {eta:.0f}s", end="")
    ser.flush()
    # 大包静默模式没有逐条 ACK, 靠最后一条 PING 确认链路与固件还活着。
    # 【2026-10-19】重试 3 次: 板端 bring-up 后每秒打一行状态文本, 与响应帧
    #   在 UART 上交织, 一次 PING 偶发解析失败不代表链路死了 —— 4.5 分钟的
    #   权重都灌完了, 因为一次抖动就报 FAIL 太亏。
    time.sleep(0.05)
    alive = False
    for _ in range(3):
        ser.reset_input_buffer()
        send(ser, CMD_PING, b"")
        if rsp_ok(ser, 3.0):
            alive = True
            break
        time.sleep(0.3)
    el = time.time() - t0
    rate = len(data) / el if el > 0 else 0
    print(f"\n[bulk] 完成 {el:.0f}s  {rate/1024:.1f} KB/s  链路={'OK' if alive else '无响应!'}")
    return alive


def mode_stream(ser, data, args):
    """推荐路径: 0x0B 灌暂存区 -> 0x11 触发 wload_dma CMD 直通搬进 y3tram。

    每批做一次字数核对。核对失败立刻停 —— 权重少一个字, 后面算出来的全是
    垃圾, 继续灌只会浪费更多时间。
    """
    total_words = (len(data) + BLK_BYTES - 1) // BLK_BYTES
    batch_bytes = args.batch
    batch_words = batch_bytes // BLK_BYTES
    print(f"[stream] {len(data)} bytes = {total_words} 词, "
          f"每批 {batch_words} 词 / {batch_bytes}B")

    t0 = time.time()
    word_off = 0
    sent_bytes = 0
    while sent_bytes < len(data):
        body = data[sent_bytes:sent_bytes + batch_bytes]
        nw = (len(body) + BLK_BYTES - 1) // BLK_BYTES
        if len(body) % BLK_BYTES:
            body += b"\x00" * (BLK_BYTES - len(body) % BLK_BYTES)

        # 1) 灌 DDR 暂存区 (静默大包)
        for blk, pay in chunk_bulk(body, W3T_SRC_BLK):
            send_blk(ser, CMD_WLOAD_RAW, pay, _cksum(CMD_WLOAD_RAW, pay))
        ser.flush()

        # 2) 触发 wload_dma: 目标 y3tram 字地址 = args.wb_base + word_off
        dst = args.wb_base + word_off
        pay = struct.pack("<II", dst, nw)
        send(ser, CMD_W3T, pay)
        r = recv_rsp(ser, args.w3t_timeout)
        if not r or r[1] != 0:
            print(f"\n[FAIL] 0x11 在 word {word_off} 失败: {r}")
            return False
        if len(r[2]) >= 4:
            okw = struct.unpack("<I", r[2][:4])[0]
            if okw != nw:
                print(f"\n[FAIL] word {word_off}: 固件只送进 {okw} 词, 期望 {nw}")
                return False

        word_off += nw
        sent_bytes += len(body)
        el = time.time() - t0
        eta = el / sent_bytes * (len(data) - sent_bytes) if sent_bytes else 0
        print(f"\r  {sent_bytes}/{len(data)}  {el:.0f}s  ETA {eta:.0f}s", end="")

    el = time.time() - t0
    print(f"\n[stream] 完成 {el:.0f}s  {len(data)/1024/max(el,1e-9):.1f} KB/s  "
          f"{total_words} 词")
    return True


# --------------------------------------------------------------------------
# 抽验
# --------------------------------------------------------------------------
def verify(ser, data, base_blk, nblk):
    """从 DDR 读回 nblk 个块与源数据比对 (0x09 GET_FRAME, 走 DDR 桥)。

    【为什么绝不能用 0x02 MEM_READ 按 blk*32 当地址读】(2026-10-19 上板教训):
      DDR 不在 CPU 内存窗口里, y3tram 才是 0x60000000。把块号当裸地址读,
      blk=0 读到的是固件代码, blk 大一点 (759264) 直接踩进未映射区 ->
      TRAP#1 mcause=5 (load access fault) -> 固件挂死, 只能复位。
      0x09 GET_FRAME 由固件经 DDR 桥 (0x8000_0000) 取块, 物理上不可能越界。
      一次读 8 块 = 256B, 所以 nblk 按 8 对齐。
    """
    print(f"[verify] 读回 {nblk} 块 @ blk {base_blk} (经 0x09 GET_FRAME)")
    bad = 0
    i = 0
    while i < nblk:
        blk = base_blk + i
        send(ser, CMD_GET_FRAME, struct.pack("<I", blk))
        r = recv_rsp(ser, 3.0)
        if not r or r[1] != 0 or len(r[2]) < 256:
            print(f"  [FAIL] blk {blk}..{blk+7} 读回失败 (r={r})")
            bad += 1
            i += 8
            continue
        got = r[2][:256]
        exp = data[i * BLK_BYTES:(i + 8) * BLK_BYTES]
        if got != exp:
            nd = sum(1 for a, b in zip(got, exp) if a != b)
            print(f"  [FAIL] blk {blk}..{blk+7} 不符 {nd}/256 B")
            print(f"        got {got[:16].hex()}")
            print(f"        exp {exp[:16].hex()}")
            bad += 1
        i += 8
    print(f"[verify] {'PASS' if bad == 0 else f'{bad} 组不符'}")
    return bad == 0


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--bin", default=os.path.join(root, "sw", "engine_weights.bin"))
    ap.add_argument("--layout", default=os.path.join(root, "sw", "engine_layout.json"))
    ap.add_argument("--mode", choices=["stream", "bulk", "ddr"], default="stream")
    ap.add_argument("--base-blk", type=lambda x: int(x, 0), default=WDDR_BASE,
                    help="权重在 DDR 的正式区起点 (ddr/bulk 模式用)")
    ap.add_argument("--wb-base", type=lambda x: int(x, 0), default=0,
                    help="stream 模式: y3tram 权重起始字地址 (默认 0, 与 layout 一致)")
    ap.add_argument("--pace", type=float, default=0.02,
                    help="bulk 模式每帧后的等待秒数。固件 0x0B 是同步写 DDR, "
                         "期间不收 UART; 全速灌会让 RX FIFO 溢出丢字节。"
                         "0.02 秒/帧 ⇒ 2.9MB 约 5 分钟且不丢")
    ap.add_argument("--batch", type=int, default=STREAM_BATCH,
                    help="stream 模式每批字节数 (默认 64KB = 2048 词)")
    ap.add_argument("--w3t-timeout", type=float, default=30.0)
    ap.add_argument("--scales", action="store_true",
                    help="顺带把 sw/engine_scales.bin 传到块 580000 (NET_RUN 的 romblk 用它)")
    ap.add_argument("--scales-blk", type=lambda x: int(x, 0), default=SCALES_BLK)
    ap.add_argument("--verify", type=int, default=0,
                    help="完后抽验 N 个块 (0 = 不验)")
    args = ap.parse_args()

    data = open(args.bin, "rb").read()
    if len(data) % BLK_BYTES:
        print(f"[warn] {args.bin} 不是 {BLK_BYTES} 的整数倍, 末尾补零")
        data += b"\x00" * (BLK_BYTES - len(data) % BLK_BYTES)

    # 用 layout 交叉核对 bin 大小, 防止拿错文件 (比如把 demo.bin 当权重传上去)
    if os.path.exists(args.layout):
        lay = json.load(open(args.layout))
        want = lay.get("total_bytes")
        if want and want != len(data) and os.path.basename(args.bin) == 'engine_weights.bin':
            print(f"[warn] layout.total_bytes={want}, 实际 {len(data)} —— 可能拿错 bin")
        total_words = lay.get("total_words", len(data) // BLK_BYTES)
    else:
        total_words = len(data) // BLK_BYTES

    print(f"bin={args.bin}  {len(data)} B = {total_words} 词  mode={args.mode}")
    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    ser.reset_input_buffer()

    # 先 PING 确认链路通, 免得白灌几 MB 才发现端口错
    send(ser, CMD_PING, b"")
    if not rsp_ok(ser, 2.0):
        print("[FAIL] PING 无响应 —— 检查端口/波特率/板子是否起来")
        return 1
    print("PING OK")

    t0 = time.time()
    if args.mode == "ddr":
        ok = mode_ddr(ser, data, args)
    elif args.mode == "bulk":
        ok = mode_bulk(ser, data, args, args.base_blk, pace=args.pace)
    else:
        ok = mode_stream(ser, data, args)
    if not ok:
        return 1

    el = time.time() - t0
    print(f"总耗时 {el:.1f}s ({len(data)/1024/el:.1f} KB/s)")

    # 尺度镜像: 只传 18.9 KB, 但它是"数值是不是网络输出"的唯一开关
    if args.scales:
        sp = os.path.join(root, "sw", "engine_scales.bin")
        if not os.path.exists(sp):
            print(f"[FAIL] 找不到 {sp} —— 先跑 /e/anconda/python.exe tools/export_scales.py")
            return 1
        sd = open(sp, "rb").read()
        print(f"[scales] {len(sd)} B -> DDR 块 {args.scales_blk} "
              f"(0x{args.scales_blk*32:08X})")
        if not mode_bulk(ser, sd, args, args.scales_blk, pace=args.pace):
            return 1

    if args.verify:
        blk = args.base_blk if args.mode != "stream" else args.base_blk
        if not verify(ser, data, blk, args.verify):
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
