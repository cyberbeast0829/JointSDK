#!/usr/bin/env python3
"""
f18_seq_probe.py — 真机验证固件 d10883f5 的 F18 修复（响应回显请求 Seq）

旧实现：`send_mit_response` 忽略 seq 形参, 填本地滚动计数 `tx_seq_++`
        ⇒ 主站无法用 (SRC,DEST,MsgType,Seq) 配对请求与响应。
新实现：`txmsg.id = make_can_id(..., seq & SEQ_MASK)` —— **原样回显请求的 seq**。

验证方法（零依赖 termios）：
  连发 4 条 MIT 请求，seq 依次 0,1,2,3，
  检查每条**应答的 seq** 是否 == 请求的 seq（而不是 0,1,2,3 的本地滚动）。
  为了排除"碰巧等于本地滚动"的歧义，再用**乱序** 3,2,1,0 发一遍：
  - 若应答 seq 跟随请求 → 是 F18 修复后的行为
  - 若应答 seq 恒为 0,1,2,3 递增 → 是旧行为
"""
import argparse
import os
import sys
import termios
import time

DLC = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8,
       12: 9, 16: 10, 20: 11, 24: 12, 32: 13, 48: 14, 64: 15}
MSG_MIT = 0x00


def slcan_fd(cid, data):
    n = len(data)
    if n not in DLC:
        raise ValueError("FD 无 %d 字节 DLC" % n)
    return ("B%08X%d%s\r" % (cid, DLC[n],
            "".join("%02X" % c for c in data))).encode()


def mkid(pri, mt, dest, src, seq):
    return ((pri & 7) << 26) | ((mt & 0xFF) << 18) | ((dest & 0xFF) << 10) | \
           ((src & 0xFF) << 2) | (seq & 3)


def mit_zero_pos():
    """pos=0 (0x8000), vel=0 (0x800), kp=kd=tau=0 (0/0/0x800) — 严格按位打包。

    布局（pack_mit_command 同构）：
      [0..1] pos，[2..3] vel(高8)+kp 高4，[3..4] kp 低8，
      [5..6] kd(高8)+tau 高4，[6..7] tau 低8
    """
    p, v, a, b, t = 0x8000, 0x800, 0x000, 0x000, 0x800
    o = bytearray(8)
    o[0] = (p >> 8) & 0xFF
    o[1] = p & 0xFF
    o[2] = (v >> 4) & 0xFF
    o[3] = (((v & 0x0F) << 4) | ((a >> 8) & 0x0F)) & 0xFF
    o[4] = a & 0xFF
    o[5] = (b >> 4) & 0xFF
    o[6] = (((b & 0x0F) << 4) | ((t >> 8) & 0x0F)) & 0xFF
    o[7] = t & 0xFF
    return bytes(o)


class Raw:
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        at = termios.tcgetattr(self.fd)
        at[0] = at[1] = at[3] = 0
        at[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        at[4] = at[5] = termios.B115200
        at[6][termios.VMIN] = 0
        at[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, at)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def w(self, b):
        os.write(self.fd, b)

    def r(self, wait=0.06, n=16384):
        time.sleep(wait)
        buf = bytearray()
        try:
            while True:
                c = os.read(self.fd, n)
                if not c:
                    break
                buf += c
                if len(buf) >= n:
                    break
        except (BlockingIOError, OSError):
            pass
        return bytes(buf)

    def lines(self, wait=0.06):
        out = []
        for ln in self.r(wait).decode("ascii", "replace").split("\r"):
            if len(ln) < 5 or ln[0] not in "tTdDbB":
                continue
            idlen = 8 if ln[0] in "TDB" else 3
            try:
                cid = int(ln[1:1 + idlen], 16)
                dlc = int(ln[1 + idlen], 16)
                nb = DLC.get(dlc)
                if nb is None:
                    continue
                hx = ln[2 + idlen:2 + idlen + 2 * nb]
                if len(hx) < 2 * nb:
                    continue
                out.append((cid, bytes.fromhex(hx)))
            except (ValueError, IndexError):
                continue
        return out

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--node", type=int, default=1)
    ap.add_argument("--master-id", type=int, default=1)
    a = ap.parse_args()

    ser = Raw(a.port)
    fails = 0
    try:
        ser.w(b"C\r"); time.sleep(0.15); ser.r(0.1)
        ser.w(b"Y5\r"); time.sleep(0.15); ser.r(0.1)
        ser.w(b"O\r"); time.sleep(0.2); ser.r(0.2)

        def one(seq, label):
            cid = mkid(0, MSG_MIT, a.node, a.master_id, seq)
            ser.w(slcan_fd(cid, mit_zero_pos()))
            got = None
            for c2, d in ser.lines(0.10):
                if ((c2 >> 18) & 0xFF) == MSG_MIT and ((c2 >> 2) & 0xFF) == a.master_id:
                    got = c2 & 3
                    break
            ok = (got == seq)
            print("  请求 seq=%d  ->  应答 seq=%s   %s"
                  % (seq, "?" if got is None else str(got),
                     "OK 回显一致" if ok else "✗ 不一致"))
            return ok

        print("=" * 74)
        print("F18：MIT 响应是否回显请求的 Seq")
        print("=" * 74)

        print("\n--- 顺序 0,1,2,3 ---")
        for s in (0, 1, 2, 3):
            if not one(s, "seq%d" % s):
                fails += 1

        print("\n--- 乱序 3,2,1,0（排除“本地滚动恰好相同”的歧义）---")
        for s in (3, 2, 1, 0):
            if not one(s, "seq%d" % s):
                fails += 1

        print("\n--- 重复同一 seq 三次（检验是否被本地计数器污染）---")
        for _ in range(3):
            if not one(2, "seq2"):
                fails += 1

        print()
        if fails == 0:
            print("结论：✅ F18 已生效 —— 应答原样回显请求 seq（顺序/乱序/重复均一致）")
        else:
            print("结论：❌ 有 %d 处不一致 —— F18 可能未生效或部分生效" % fails)
        return 0 if fails == 0 else 1
    finally:
        try:
            ser.w(b"C\r")
        except OSError:
            pass
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
