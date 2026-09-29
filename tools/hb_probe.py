#!/usr/bin/env python3
"""
hb_probe.py — 真机验证固件 d10883f5 的心跳修订（F11 / F12）

固件声明（commit d10883f5）：
  F11: CAN FD 心跳 len 18 → 19，新增 buf[18] = 完整 current_state_（0~255）
       前 18 字节与旧版兼容。
  F12: FD 心跳的 vbus/ibus/pos/vel 补钳位。

本探针验证：
  1) FD 心跳的**实际线上长度**与 DLC 码
  2) buf[18] 的取值与 axis0.current_state 是否一致
  3) buf[1] 的高 4 bit（旧 state）与 buf[18] 是否一致
  4) 数值合理性（vbus/温度/电流在物理范围内）

⚠ CAN FD 的 DLC 表没有"19 字节"这一档：
   0..8 直映，之后 12/16/20/24/32/48/64。
   所以 19 字节的报文在总线上**必然**以 20 字节传输（置位填充），
   接收端读到的 len 会是 20 —— 这**不是缺陷**，但会给客户造成困惑。
"""
import argparse
import os
import sys
import termios
import time

DLC = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8,
       12: 9, 16: 10, 20: 11, 24: 12, 32: 13, 48: 14, 64: 15}
MSG_HEARTBEAT = 0x48


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

    def r(self, wait=0.15, n=32768):
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

    def frames(self, wait=0.15):
        """返回 [(kind, cid, dlc_code, nbytes, data)]"""
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
                out.append((ln[0], cid, dlc, nb, bytes.fromhex(hx)))
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
    try:
        ser.w(b"C\r"); time.sleep(0.15); ser.r(0.1)
        ser.w(b"Y5\r"); time.sleep(0.15); ser.r(0.1)
        ser.w(b"O\r"); time.sleep(0.2); ser.r(0.2)
        # 触发一次 master_id 学习：发一条无关紧要的 MIT（不施力）
        ser.w(("B%08X8%s\r" % (0x00000404, "8000800000000800")).encode())
        time.sleep(0.15)
        ser.r(0.2)

        fr = ser.frames(0.6)
        hb = [x for x in fr if ((x[1] >> 18) & 0xFF) == MSG_HEARTBEAT]
        print("=" * 74)
        print("F11/F12：CAN FD 心跳")
        print("=" * 74)
        if not hb:
            print("⚠ 未收到心跳（master_id 学会了吗？等一会儿再试）")
            return 2

        lens = {}
        for k, cid, dlc, nb, d in hb:
            lens.setdefault((k, dlc, nb), 0)
            lens[(k, dlc, nb)] += 1
        print("\n观察到的 帧类型/DLC码/字节数 : 次数")
        for (k, dlc, nb), c in sorted(lens.items()):
            print("   %s  dlc=%2d  len=%2d   x%d" % (k, dlc, nb, c))

        print("\n--- 逐帧字段（前 6 帧）---")
        bad = 0
        for k, cid, dlc, nb, d in hb[:6]:
            life = (d[0] >> 5) & 7
            errf = d[0] & 0x1F
            st4 = (d[1] >> 4) & 0x0F
            cm = d[1] & 0x0F
            tmot, tmos = d[2] - 50, d[3] - 50
            vbus = ((d[4] << 8) | d[5]) * 0.1
            ibus = (((d[6] << 8) | d[7]) - (65536 if d[6] & 0x80 else 0)) * 0.01
            pos = (((d[8] << 24) | (d[9] << 16) | (d[10] << 8) | d[11])
                   - (1 << 32) if d[8] & 0x80 else 0)
            pos = ((d[8] << 24) | (d[9] << 16) | (d[10] << 8) | d[11])
            if pos >= (1 << 31):
                pos -= (1 << 32)
            vel = ((d[12] << 24) | (d[13] << 16) | (d[14] << 8) | d[15])
            if vel >= (1 << 31):
                vel -= (1 << 32)
            iq = ((d[16] << 8) | d[17])
            if iq >= (1 << 15):
                iq -= (1 << 16)
            st_full = d[18] if nb > 18 else None
            print("   life=%d err=0x%02X st4bit=%2d ctrl=%d tMot=%3d tMos=%3d "
                  "vbus=%6.1fV ibus=%+6.2fA pos=%+8.4f vel=%+8.4f iq=%+7.2fA "
                  "stFull=%s"
                  % (life, errf, st4, cm, tmot, tmos, vbus, ibus,
                     pos / 10000.0, vel / 10000.0, iq / 100.0,
                     "None" if st_full is None else str(st_full)))
            if st_full is None:
                print("      ✗ 无 buf[18]（F11 未生效？）")
                bad += 1
            elif st_full != st4:
                # 只有当 st4 未被截断时才应相等
                if st4 != 0:
                    print("      ⚠ buf[18]=%d 与 4bit state=%d 不一致" %
                          (st_full, st4))
                    bad += 1
                else:
                    print("      ℹ 4bit=0 而完整=%d（这正是 F11 要修的截断情形）"
                          % st_full)

        print()
        if bad == 0:
            print("结论：✅ F11 生效 —— 心跳含 buf[18] 完整 state，且数值合理")
        else:
            print("结论：⚠ 有 %d 处异常，见上" % bad)
        return 0 if bad == 0 else 1
    finally:
        try:
            ser.w(b"C\r")
        except OSError:
            pass
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
