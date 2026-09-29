#!/usr/bin/env python3
"""
f1_stiffness_probe.py — 真机判定 MIT kp 的输出端等效刚度（限位法）

为什么必须用"限位法"
--------------------
位置阶跃法在**自由关节**上不可用：实测给 0.02 rad 阶跃后关节冲到
1.45 rad / 5.7 rad/s，采到的是闭环暂态，iq 被动力学项主导。

限位法：把目标设在**机械限位之外**，关节被挡死 ⇒ 静止。
  此时 e_out = 目标 − 实际位置 是**已知且稳定**的，iq = 纯静态保持电流。
  由  K_out = (iq·tc/g) / e_out  得到输出端刚度。
    · K_out/kp ≈ 1.000 ⇒ 固件注释正确（K=kp）
    · K_out/kp ≈ 1/g   ⇒ 我们的推导正确（K=kp/g）

安全
----
  · 必须先手动确认限位位置（脚本有 --read-pos 模式）
  · 目标只超限位一点点（--travel，默认 0.05 rad），电流很小
  · 每次拉住 ≤1.0 s，结束立刻卸力 + STOP_MOTOR
  · 需要 --unsafe-force
"""
import argparse
import os
import sys
import termios
import time

DLC = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8,
       12: 9, 16: 10, 20: 11, 24: 12, 32: 13, 48: 14, 64: 15}
MSG_MIT = 0x00
MSG_START_MOTOR = 0x62
MSG_STOP_MOTOR = 0x63


def f2u(x, lo, hi, bits):
    span = hi - lo
    maxv = (1 << bits) - 1
    x = max(lo, min(hi, x))
    s = (x - lo) * maxv / span
    s = int(s + 0.5) if s >= 0 else int(s - 0.5)
    return max(0, min(maxv, s))


def u2f(v, lo, hi, bits):
    return float(v) * (hi - lo) / float((1 << bits) - 1) + lo


def pack_mit(pos, vel, kp, kd, tau, P, V, KP, KD, T):
    p = f2u(pos, -P, P, 16)
    v = f2u(vel, -V, V, 12)
    a = f2u(kp, 0.0, KP, 12)
    b = f2u(kd, 0.0, KD, 12)
    t = f2u(tau, -T, T, 12)
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

    def r(self, wait=0.04, n=32768):
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

    def frames(self, wait=0.04):
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
    ap.add_argument("--gear", type=float, default=7.75)
    ap.add_argument("--tc", type=float, default=0.0385)
    ap.add_argument("--pmax", type=float, default=12.56637)
    ap.add_argument("--vmax", type=float, default=30.0)
    ap.add_argument("--kpmax", type=float, default=500.0)
    ap.add_argument("--kdmax", type=float, default=5.0)
    ap.add_argument("--tmax", type=float, default=18.0)
    ap.add_argument("--read-pos", action="store_true",
                    help="只读当前位置（用于手动推到限位后取值）")
    ap.add_argument("--limit-pos", type=float, default=None)
    ap.add_argument("--travel", type=float, default=0.05)
    ap.add_argument("--kp", type=float, default=30.0)
    ap.add_argument("--unsafe-force", action="store_true")
    a = ap.parse_args()

    P, V, KP, KD, T = a.pmax, a.vmax, a.kpmax, a.kdmax, a.tmax

    ser = Raw(a.port)
    try:
        ser.w(b"C\r"); time.sleep(0.15); ser.r(0.1)
        ser.w(b"Y5\r"); time.sleep(0.15); ser.r(0.1)
        ser.w(b"O\r"); time.sleep(0.2); ser.r(0.2)

        def mit(pos, vel, kp, kd, tau, wait=0.04):
            cid = mkid(0, MSG_MIT, a.node, a.master_id, 0)
            ser.w(slcan_fd(cid, pack_mit(pos, vel, kp, kd, tau, P, V, KP, KD, T)))
            for c, d in ser.frames(wait):
                if ((c >> 18) & 0xFF) == MSG_MIT and len(d) >= 8:
                    p = (d[0] << 8) | d[1]
                    v = (d[2] << 4) | ((d[3] >> 4) & 0x0F)
                    maxcur = min(80.0, T / a.tc)
                    ci = ((d[4] << 4) & 0xFFF) | ((d[5] >> 4) & 0x0F)
                    return (u2f(p, -P, P, 16), u2f(v, -V, V, 12),
                            u2f(ci, -maxcur, maxcur, 12), d[3] & 0x0F)
            return None

        if a.read_pos:
            print("=" * 74)
            print("当前位置读取（把关节手动推到限位后运行，取回的值即 --limit-pos）")
            print("=" * 74)
            for i in range(20):
                r = mit(0.0, 0.0, 0.0, 0.0, 0.0, wait=0.05)
                if r:
                    print("  [%2d] pos=%+.5f rad  vel=%+.5f  iq=%+.4fA  err=%d"
                          % (i, r[0], r[1], r[2], r[3]))
            return 0

        if not a.unsafe_force:
            print("警告：本实验会使能关节并施加静态保持力矩。加 --unsafe-force 才执行。")
            print("流程：① 手动推到限位 → ② --read-pos 取值 → ③ --limit-pos <值> --unsafe-force")
            return 0
        if a.limit_pos is None:
            print("必须给 --limit-pos（用 --read-pos 取）")
            return 2

        print("=" * 74)
        print("F1 限位法静态刚度判定  limit=%.4f travel=%.4f kp=%.1f gear=%.2f tc=%.4f"
              % (a.limit_pos, a.travel, a.kp, a.gear, a.tc))
        print("=" * 74)

        base = mit(0.0, 0.0, 0.0, 0.0, 0.0)
        print("  基线: pos=%+.5f vel=%+.5f iq=%+.4f" % base[:3])

        cid = mkid(0, MSG_START_MOTOR, a.node, a.master_id, 0)
        ser.w(slcan_fd(cid, b"\x00" * 8))
        time.sleep(0.4); ser.frames(0.2)
        print("  已使能")

        # 目标 = 限位 + travel（两个方向都试，取关节真被挡住的那些）
        results = []
        try:
            for sign in (+1.0, -1.0):
                for tv in (a.travel, a.travel * 2.0):
                    target = a.limit_pos + sign * tv
                    if abs(target) >= P * 0.99:
                        continue
                    samples = []
                    for i in range(60):     # 0.6 s 持续发帧
                        r = mit(target, 0.0, a.kp, 0.0, 0.0, wait=0.01)
                        if r:
                            samples.append(r)
                    tail = samples[len(samples) // 2:]
                    if not tail:
                        continue
                    pos_avg = sum(x[0] for x in tail) / len(tail)
                    vel_avg = sum(abs(x[1]) for x in tail) / len(tail)
                    iq_avg = sum(x[2] for x in tail) / len(tail)
                    e_out = target - pos_avg
                    tau_out = iq_avg * a.tc / a.gear
                    K = (tau_out / e_out) if abs(e_out) > 1e-5 else float("nan")
                    results.append((sign, tv, pos_avg, vel_avg, iq_avg, e_out,
                                    tau_out, K))
                    print("\n  目标=%+.4f (sign%+d tv=%.3f)" % (target, sign, tv))
                    print("    稳态 pos=%+.5f  |vel|=%.5f  iq=%+.5fA"
                          % (pos_avg, vel_avg, iq_avg))
                    print("    e_out=%+.5f  tau_out=%+.5f  K_out=%+.5f  K/kp=%.4f"
                          % (e_out, tau_out, K, K / a.kp if a.kp else 0))
                    # 回位
                    for _ in range(3):
                        mit(a.limit_pos, 0.0, 0.0, 0.0, 0.0, 0.04)
        finally:
            for _ in range(5):
                mit(a.limit_pos, 0.0, 0.0, 0.0, 0.0, 0.03)
            cid = mkid(0, MSG_STOP_MOTOR, a.node, a.master_id, 0)
            ser.w(slcan_fd(cid, b"\x00" * 8))
            time.sleep(0.2); ser.frames(0.2)
            print("\n  已卸力并停机")

        # 汇总：只取"确实静止且被挡住"的点（|vel| 小、|e_out| 明显）
        good = [r for r in results if r[3] < 0.5 and abs(r[5]) > 0.01
                and r[7] == r[7]]
        print("\n  --- 合格样本（|vel|<0.5 rad/s 且 |e_out|>0.01 rad）---")
        for r in good:
            print("    sign%+d tv=%.3f  K/kp = %.4f" % (r[0], r[1], r[7] / a.kp))
        if good:
            import statistics
            ratio = statistics.median([r[7] / a.kp for r in good])
            print("\n  中位 K_out/kp = %.4f" % ratio)
            print("  参考：kp/g = %.4f ; kp =  1.0000" % (1.0 / a.gear))
            if abs(ratio - 1.0) < abs(ratio - 1.0 / a.gear):
                print("  ⇒ 更接近 **K_out = kp**（固件注释成立）")
            else:
                print("  ⇒ 更接近 **K_out = kp/g**（我们的推导成立）")
        else:
            print("    ⚠ 无合格样本：关节可能未被挡死（还是自由转动）")
            print("      ⇒ 请确认限位位置，或改用其他加载方式")
    finally:
        try:
            ser.w(b"C\r")
        except OSError:
            pass
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
