#!/usr/bin/env python3
"""
f13_write_probe.py — 真机验证固件 d10883f5 的 F13 修复（分段写末块长度校验）

固件（cmd_param_write_segmented）关键约定：
  · 块帧布局（8 B）：[Flags][EpID u16 BE][TotalLen][Chunk 4 B]
  · Flags bit7(0x80) = More（1 = 还有后续块，0 = 末块）
  · TotalLen **只允许 5..8**，否则中止装配
  · 末块（More=0）必须 `offset == total_len`，否则**丢弃且不发 ACK**（F13 新增）
  · 只有走完上述校验才 `send_param_write_ack()` → 0x21 静默确认（Flags 原样）

判据 = **是否收到 0x21 确认**：
  ① 合法：TotalLen=8，两块 4+4（More 然后末块）      → 应有 ACK
  ② 不完整：TotalLen=8，但先 4B+More、末块只给…（同 ①）
     真正的"不完整"要构造 offset != total_len：
        - **单块**带 TotalLen=8 且 More=0（只贡献 4 字节）→ offset=4 != 8 → 丢弃（F13）
  ③ TotalLen=8，两块，但第二块把 More 置 0 且第一块也置 0
     （即 offset=4 就宣告结束）→ 丢弃（F13）

安全：用 `config.custom_serial_number`（端点 78，uint64 rw，当前 = 0）。
      "合法"用例**写回原值 0**，幂等无害。
"""
import argparse
import os
import struct
import sys
import termios
import time

DLC = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8,
       12: 9, 16: 10, 20: 11, 24: 12, 32: 13, 48: 14, 64: 15}
MSG_PARAM_READ = 0x20
MSG_PARAM_WRITE = 0x21
FLAG_MORE = 0x80


def mkid(pri, mt, dest, src, seq):
    return ((pri & 7) << 26) | ((mt & 0xFF) << 18) | ((dest & 0xFF) << 10) | \
           ((src & 0xFF) << 2) | (seq & 3)


def slcan_fd(cid, data):
    n = len(data)
    if n not in DLC:
        raise ValueError("FD 无 %d 字节 DLC" % n)
    return ("B%08X%d%s\r" % (cid, DLC[n],
            "".join("%02X" % c for c in data))).encode()


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

    def r(self, wait=0.3, n=32768):
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

    def frames(self, wait=0.3):
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
    ap.add_argument("--ep", type=int, default=78,
                    help="8 字节 rw 端点（默认 78 = config.custom_serial_number）")
    a = ap.parse_args()

    ep = a.ep
    ser = Raw(a.port)
    try:
        ser.w(b"C\r"); time.sleep(0.15); ser.r(0.1)
        ser.w(b"Y5\r"); time.sleep(0.15); ser.r(0.1)
        ser.w(b"O\r"); time.sleep(0.2); ser.r(0.2)
        print("=" * 74)
        print("F13：分段写末块长度校验（判据 = 是否收到 0x21 确认）")
        print("  端点 0x%04X (%d)，TotalLen 合法范围 5..8" % (ep, ep))
        print("=" * 74)

        def read_val():
            body = bytes([0x00, (ep >> 8) & 0xFF, ep & 0xFF, 0x08,
                          0, 0, 0, 0])          # ReqLen=8, Offset=0
            cid = mkid(0, MSG_PARAM_READ, a.node, a.master_id, 0)
            ser.w(slcan_fd(cid, body))
            for c, d in ser.frames(0.35):
                if ((c >> 18) & 0xFF) == MSG_PARAM_READ and \
                   ((d[1] << 8) | d[2]) == ep:
                    dl = d[3]
                    return d[4:4 + dl]
            return None

        def seg(blocks, label, total=8):
            """blocks = [(flags, chunk4), ...]"""
            for flags, ch in blocks:
                body = bytes([flags, (ep >> 8) & 0xFF, ep & 0xFF, total]) + \
                    ch[:4].ljust(4, b"\x00")
                cid = mkid(0, MSG_PARAM_WRITE, a.node, a.master_id, 0)
                ser.w(slcan_fd(cid, body))
                time.sleep(0.15)
            got = [(c, d) for c, d in ser.frames(0.35)
                   if ((c >> 18) & 0xFF) == MSG_PARAM_WRITE and
                   ((d[1] << 8) | d[2]) == ep]
            for c, d in got:
                print("       ACK flags=0x%02X ep=0x%04X len=%d data=%s"
                      % (d[0], (d[1] << 8) | d[2], d[3], d.hex()))
            print("    => ACK=%d  %s" % (len(got),
                  "写入被接受" if got else "无 ACK ⇒ 装配被丢弃"))
            return len(got)

        orig = read_val()
        print("\n  写前读回 = %s" % (orig.hex() if orig else "?"))
        orig8 = (orig or b"\x00" * 8).ljust(8, b"\x00")

        print("\n--- ① 合法：TotalLen=8，两块 4+4（写回原值，幂等安全）---")
        n_ok = seg([(FLAG_MORE, orig8[0:4]), (0x00, orig8[4:8])], "合法", 8)

        print("\n--- ② 单块带 TotalLen=8 且 More=0（offset=4 != 8）→ F13 应丢弃 ---")
        n_bad1 = seg([(0x00, b"\x11\x22\x33\x44")], "单块不足", 8)

        print("\n--- ③ 两块但第一块就不带 More（offset=4 即宣告结束）→ 应丢弃 ---")
        n_bad2 = seg([(0x00, b"\x11\x22\x33\x44")], "提前结束", 8)

        print("\n--- ④ 边界：TotalLen=4（<5，非法）→ 应丢弃 ---")
        n_bad3 = seg([(0x00, b"\x11\x22\x33\x44")], "TotalLen=4", 4)

        after = read_val()
        print("\n  写后读回 = %s" % (after.hex() if after else "?"))
        intact = (after == orig)

        print()
        print("预期：① >=1；② 0；③ 0；④ 0；且值未被改动")
        ok = (n_ok >= 1 and n_bad1 == 0 and n_bad2 == 0 and n_bad3 == 0
              and intact)
        print("结果：%s" % ("✅ F13 生效（合法通过；三种非法装配全被丢弃且未破坏值）"
                          if ok else "❌ 与预期不符，见上"))
        return 0 if ok else 1
    finally:
        try:
            ser.w(b"C\r")
        except OSError:
            pass
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
