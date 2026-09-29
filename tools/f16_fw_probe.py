#!/usr/bin/env python3
"""
f16_fw_probe.py — 真机验证固件 d10883f5 的 F16 修复（越界饱和 vs 回绕）

零依赖（POSIX termios）。

判定原理
--------
`float_to_uint()` 只用于**编码**（固件发响应 / 主站组帧）。
设备**解码**主站帧用的是 `uint_to_float()`（未变）。

所以 F16 的现场事故机制是：
  主站想发 +12.625 rad（仅超量程 0.47%）
  - 若主站按「旧语义」编码 → 0x0097 → 设备解码 **−12.508 rad** ⇒ 关节反向跑
  - 若主站按「新语义」编码 → 0xFFFF → 设备解码 **+12.566 rad** ⇒ 饱和在正向端点

固件本次修复的是 **编码侧**（`float_to_uint` 改 round+clamp），
并把旧语义保留为 `float_to_uint_legacy_unclamped` 供对拍。

本探针做两件事：
  1) **语义对拍**（无需真机）：证明两种编码解码出的目标值符号相反。
  2) **真机**：把两种编码都发出去，读 MIT 应答，确认设备**接受帧并且未报错**；
     同时验证应答的 pos 字段可用（作为后续方向实验的判据）。

⚠ 本探针不做施力实验（kp=kd=tau=0），因此不会驱动电机。
"""
import argparse
import os
import sys
import termios
import time

# ---------------------------------------------------------------- codecs
def f2u_legacy(x, x_min, x_max, bits):
    """旧固件语义：向零截断 + 不钳位"""
    span = x_max - x_min
    return int((x - x_min) * float((1 << bits) - 1) / span)


def f2u_new(x, x_min, x_max, bits):
    """新固件语义：round-half-away-from-zero + 钳位"""
    span = x_max - x_min
    maxv = (1 << bits) - 1
    x = max(x_min, min(x_max, x))
    s = (x - x_min) * maxv / span
    s = int(s + 0.5) if s >= 0 else int(s - 0.5)
    return max(0, min(maxv, s))


def u2f(v, x_min, x_max, bits):
    return float(v) * (x_max - x_min) / float((1 << bits) - 1) + x_min


def pack_mit(pos, vel, kp, kd, tau, pmax, vmax, kpmax, kdmax, tmax, enc):
    p = enc(pos, -pmax, pmax, 16) & 0xFFFF
    v = enc(vel, -vmax, vmax, 12) & 0xFFF
    a = enc(kp, 0.0, kpmax, 12) & 0xFFF
    b = enc(kd, 0.0, kdmax, 12) & 0xFFF
    t = enc(tau, -tmax, tmax, 12) & 0xFFF
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


DLC = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8,
       12: 9, 16: 10, 20: 11, 24: 12, 32: 13, 48: 14, 64: 15}


def slcan_fd(cid, data):
    n = len(data)
    if n not in DLC:
        raise ValueError("FD 无 %d 字节 DLC" % n)
    return ("B%08X%d%s\r" % (cid, DLC[n],
            "".join("%02X" % c for c in data))).encode()


def mkid(pri, mt, dest, src, seq):
    return ((pri & 7) << 26) | ((mt & 0xFF) << 18) | ((dest & 0xFF) << 10) | \
           ((src & 0xFF) << 2) | (seq & 3)


MSG_MIT = 0x00


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
                out.append((ln[0], cid, bytes.fromhex(hx)))
            except (ValueError, IndexError):
                continue
        return out

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass


def decode_mit_resp(d, pmax, vmax, maxtau, tc):
    if len(d) < 8:
        return None
    p = (d[0] << 8) | d[1]
    v = (d[2] << 4) | ((d[3] >> 4) & 0x0F)
    err = d[3] & 0x0F
    c = ((d[4] << 4) & 0xFFF) | ((d[5] >> 4) & 0x0F)
    mode = d[5] & 0x0F
    maxcur = 40.0
    if tc > 0.001:
        maxcur = min(80.0, maxtau / tc)
    return {"pos": u2f(p, -pmax, pmax, 16), "vel": u2f(v, -vmax, vmax, 12),
            "cur": u2f(c, -maxcur, maxcur, 12), "err": err, "mode": mode,
            "t_mot": d[6] - 50, "t_mos": d[7] - 50}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--node", type=int, default=1)
    ap.add_argument("--master-id", type=int, default=1)
    ap.add_argument("--pmax", type=float, default=12.56637)
    ap.add_argument("--vmax", type=float, default=30.0)
    ap.add_argument("--kpmax", type=float, default=500.0)
    ap.add_argument("--kdmax", type=float, default=5.0)
    ap.add_argument("--tmax", type=float, default=18.0)
    ap.add_argument("--tc", type=float, default=0.0385)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    P, V, KP, KD, T = a.pmax, a.vmax, a.kpmax, a.kdmax, a.tmax

    print("=" * 74)
    print("F16 语义对拍（编码侧决定设备收到的目标值）")
    print("=" * 74)
    for name, val, lo, hi, bits in [("pos", 12.625, -P, P, 16),
                                    ("pos", -12.625, -P, P, 16),
                                    ("tau", 18.009, -T, T, 12),
                                    ("tau", -18.009, -T, T, 12)]:
        leg = f2u_legacy(val, lo, hi, bits) & ((1 << bits) - 1)
        new = f2u_new(val, lo, hi, bits) & ((1 << bits) - 1)
        dl, dn = u2f(leg, lo, hi, bits), u2f(new, lo, hi, bits)
        print("  %s=%+8.3f  量程 %+.3f..%+.3f" % (name, val, lo, hi))
        print("     旧语义 0x%0*X -> 设备解码 %+8.3f%s"
              % (bits // 4, leg, dl,
                 "   <-- 符号反转（事故）" if (val > 0) != (dl > 0) else ""))
        print("     新语义 0x%0*X -> 设备解码 %+8.3f%s"
              % (bits // 4, new, dn,
                 "   <-- 饱和（正确）" if abs(dn) >= abs(val) - 1e-9 else ""))
    print("\n  结论：固件修复点在**编码侧**（round+clamp）；解码侧 uint_to_float 未变。")
    print("        => 主站按固件保证的语义编码时，越界命令会**饱和**而非反向。")

    if a.dry_run:
        print("\n[--dry-run] 不做真机交互")
        return 0

    ser = Raw(a.port)
    try:
        ser.w(b"C\r"); print("\n  C  -> %r" % ser.r(0.15)[:40])
        ser.w(b"Y5\r"); print("  Y5 -> %r" % ser.r(0.15)[:40])
        ser.w(b"O\r"); print("  O  -> %r" % ser.r(0.2)[:40])

        def send(data, label, seq=0, wait=0.09):
            cid = mkid(0, MSG_MIT, a.node, a.master_id, seq)
            ser.w(slcan_fd(cid, data))
            ls = ser.lines(wait)
            resp = None
            for k, c2, d in ls:
                if ((c2 >> 18) & 0xFF) == MSG_MIT and (c2 & 3) == (seq & 3):
                    resp = d
                    break
            print("\n  [%s]" % label)
            print("     发 pos=0x%04X vel=0x%03X kp=0x%03X kd=0x%03X tau=0x%03X"
                  % ((data[0] << 8) | data[1],
                     (data[2] << 4) | (data[3] >> 4),
                     ((data[3] & 0x0F) << 8) | data[4],
                     ((data[5] << 4) | (data[6] >> 4)),
                     ((data[6] & 0x0F) << 8) | data[7]))
            if resp:
                r = decode_mit_resp(resp, P, V, T, a.tc)
                print("     应答 pos=%+.4f vel=%+.4f cur=%+.3fA err=%d mode=%d "
                      "tMot=%d tMos=%d"
                      % (r["pos"], r["vel"], r["cur"], r["err"], r["mode"],
                         r["t_mot"], r["t_mos"]))
                ok = (r["err"] == 0)
                print("     %s" % ("OK: 帧被接受且 err=0" if ok else
                                   "⚠ err=0x%X" % r["err"]))
            else:
                print("     ⚠ 未收到 seq 匹配的 MIT 应答")
            return resp

        print("\n--- 1) 越界 pos 旧语义编码（设备会当成负向目标）---")
        send(pack_mit(12.625, 0, 0, 0, 0, P, V, KP, KD, T, f2u_legacy),
             "legacy +12.625 -> 0x0097")
        print("\n--- 2) 越界 pos 新语义编码（设备饱和到 +12.566）---")
        send(pack_mit(12.625, 0, 0, 0, 0, P, V, KP, KD, T, f2u_new),
             "new    +12.625 -> 0xFFFF")
        print("\n--- 3) 负向越界 旧/新 ---")
        send(pack_mit(-12.625, 0, 0, 0, 0, P, V, KP, KD, T, f2u_legacy),
             "legacy -12.625")
        send(pack_mit(-12.625, 0, 0, 0, 0, P, V, KP, KD, T, f2u_new),
             "new    -12.625")
        print("\n  安全收尾：回零（kp=kd=tau=0，不施力）")
        send(pack_mit(0.0, 0, 0, 0, 0, P, V, KP, KD, T, f2u_new), "回零")
    finally:
        try:
            ser.w(b"C\r")
        except OSError:
            pass
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
