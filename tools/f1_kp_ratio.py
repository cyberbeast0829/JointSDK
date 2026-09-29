#!/usr/bin/env python3
"""
f1_kp_ratio.py — 用「同一外部力矩 + 两组不同 kp」直接测 K_out/kp

核心思想（单变量，消除换算路径未知性）
--------------------------------------
静态平衡（输出端）：  K_out * e = -M_ext
  ⇒ e = -M_ext / K_out

对**同一个** M_ext（重力＋摩擦，未知但恒定），用两组不同的 kp 测出 e1、e2：
    e1 = -M_ext / K_out(kp1)
    e2 = -M_ext / K_out(kp2)
  ⇒ e1/e2 = K_out(kp2)/K_out(kp1)

若 K_out = kp      (固件主张)  ⇒ e1/e2 = kp2/kp1
若 K_out = kp/g    (我们主张)  ⇒ e1/e2 = (kp2/g)/(kp1/g) = kp2/kp1
  ⚠ 两者给出**相同**的 e1/e2！ ⇒ 这个比值不能区分。

⇒ 必须用**绝对**量。已知绝对量只有 tau_ff（CAN 侧给的输出端 N·m）。

更直接的判据：**kp 项 vs tau_ff 项 的相对增益**
  静态平衡（输出端）：
       K_out * e  =  -tau_ff_effective
  · 若两项经过**同一**换算，则 K_out = kp，且 tau_ff_effective = tau_ff_can
       ⇒ e = -tau_ff_can / kp
  · 我们需要**独立**知道 M_ext，才能定 K_out。

⇒ 真正单变量且绝对的方法：**用 tau_ff 直接抵消重力**
   步骤：
     1) kp 取大、tau_ff 扫描，找到使 e≈0 的 tau_ff*  →  该 tau_ff* 正好抵消 M_ext
     2) 记录该点的 iq*（纯保持电流）
     3) 与"kp 项"无关 ⇒ 可单独标定 tau_ff→iq 的换算！
   由 (1) 得 M_ext（以 CAN 侧 tau_ff 单位计），
   由 (2) 得 iq 对应的力矩 ⇒ tau_ff_can 与 iq 的关系 K_tf = iq/tau_ff。
   若 tau_ff_can 是**输出端** N·m，则 tau_motor = tau_ff_can*g，
      iq = tau_motor/tc = tau_ff_can*g/tc
      ⇒ iq/tau_ff = g/tc = 7.75/0.0385 = 201.3
   若 tau_ff 被 /g（即被当成电机端但还除了一次），则 iq/tau_ff = 1/tc = 25.97

   ★ 这个 K_tf = iq/tau_ff 是**绝对标定**，能直接判定 tau_ff 的端别！
   接着：既然 kp 项与 tau_ff 项在 (kp*mit_p_err) 里是**同一行相加**，
         只要知道 mit_p_err 是输出端 rad（已由符号推导确认），
         就能定出 kp 的等效刚度。

本脚本只做第 (1) 步的高精度版本：**扫 tau_ff 找 e≈0**，并记录 iq。
"""
import argparse
import os
import sys
import termios
import time

DLC = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]
P, V, KP, KD, T = 12.56637, 30.0, 500.0, 5.0, 18.0
TC = 0.0385
GEAR = 7.75


def f2u(x, lo, hi, b):
    s = (max(lo, min(hi, x)) - lo) * ((1 << b) - 1) / (hi - lo)
    return max(0, min((1 << b) - 1, int(s + 0.5) if s >= 0 else int(s - 0.5)))


def u2f(v, lo, hi, b):
    return v * (hi - lo) / float((1 << b) - 1) + lo


def pk(pos, vel, kp, kd, tau):
    p = f2u(pos, -P, P, 16)
    v = f2u(vel, -V, V, 12)
    a = f2u(kp, 0, KP, 12)
    b = f2u(kd, 0, KD, 12)
    t = f2u(tau, -T, T, 12)
    return bytes([(p >> 8) & 255, p & 255, (v >> 4) & 255,
                  (((v & 15) << 4) | ((a >> 8) & 15)) & 255, a & 255,
                  (b >> 4) & 255,
                  (((b & 15) << 4) | ((t >> 8) & 15)) & 255, t & 255])


def mk(pri, mt, d, s, q):
    return ((pri & 7) << 26) | ((mt & 255) << 18) | ((d & 255) << 10) | \
           ((s & 255) << 2) | (q & 3)


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

    def r(self, t=0.02, n=32768):
        time.sleep(t)
        o = bytearray()
        try:
            while True:
                c = os.read(self.fd, n)
                if not c:
                    break
                o += c
                if len(o) >= n:
                    break
        except (BlockingIOError, OSError):
            pass
        return bytes(o)

    def frames(self, t=0.02):
        out = []
        for ln in self.r(t).decode("ascii", "replace").split("\r"):
            if len(ln) > 5 and ln[0] in "TDB":
                try:
                    cid = int(ln[1:9], 16)
                    nb = DLC[int(ln[9], 16)]
                    hx = ln[10:10 + 2 * nb]
                    if len(hx) >= 2 * nb:
                        out.append((cid, bytes.fromhex(hx)))
                except (ValueError, IndexError):
                    pass
        return out

    def mit(self, pos, vel, kp, kd, tau, wait=0.02):
        self.w(("B%08X8%s\r" % (mk(0, 0, 1, 1, 0),
                                pk(pos, vel, kp, kd, tau).hex())).encode())
        for c, d in self.frames(wait):
            if ((c >> 18) & 255) == 0 and len(d) >= 8:
                p = (d[0] << 8) | d[1]
                v = (d[2] << 4) | ((d[3] >> 4) & 15)
                mc = min(80.0, T / TC)
                ci = ((d[4] << 4) & 0xFFF) | ((d[5] >> 4) & 15)
                return u2f(p, -P, P, 16), u2f(v, -V, V, 12), u2f(ci, -mc, mc, 12)
        return None

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--target", type=float, default=-3.03675)
    ap.add_argument("--kp", type=float, default=60.0)
    ap.add_argument("--kd", type=float, default=0.8)
    ap.add_argument("--unsafe-force", action="store_true")
    a = ap.parse_args()

    if not a.unsafe_force:
        print("需要 --unsafe-force")
        return 0

    ser = Raw(a.port)
    try:
        ser.w(b"C\r"); ser.r(0.15)
        ser.w(b"Y5\r"); ser.r(0.15)
        ser.w(b"O\r"); ser.r(0.2)

        base = ser.mit(0, 0, 0, 0, 0)
        print("基线: pos=%+.5f vel=%+.5f iq=%+.4f" % base)

        ser.w(("B%08X8%s\r" % (mk(0, 0x62, 1, 1, 0), "0" * 16)).encode())
        time.sleep(0.4); ser.r(0.2)
        print("已使能")

        TGT = a.target
        print("\n扫 tau_ff（kp=%.0f kd=%.1f），找 e≈0 的点" % (a.kp, a.kd))
        print("  tau_ff     pos        e         |vel|      iq")
        rows = []
        try:
            for tau in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.5,
                        -0.2, -0.4, -0.6):
                sm = []
                for _ in range(120):
                    x = ser.mit(TGT, 0.0, a.kp, a.kd, tau, 0.008)
                    if x:
                        sm.append(x)
                tail = sm[int(len(sm) * 0.6):]
                if not tail:
                    continue
                pa = sum(x[0] for x in tail) / len(tail)
                va = sum(abs(x[1]) for x in tail) / len(tail)
                ia = sum(x[2] for x in tail) / len(tail)
                e = pa - TGT
                rows.append((tau, pa, e, va, ia))
                print("  %+6.2f  %+.6f  %+.6f  %.4f  %+.5f"
                      % (tau, pa, e, va, ia))
                for _ in range(4):
                    ser.mit(TGT, 0, 0, 0, 0, 0.02)

            # 线性拟合 e = A*tau + B，找 e=0 的 tau*
            n = len(rows)
            if n >= 3:
                sx = sum(r[0] for r in rows)
                sy = sum(r[2] for r in rows)
                sxx = sum(r[0] ** 2 for r in rows)
                sxy = sum(r[0] * r[2] for r in rows)
                den = n * sxx - sx * sx
                if abs(den) > 1e-12:
                    A = (n * sxy - sx * sy) / den
                    B = (sy - A * sx) / n
                    tau_zero = -B / A if abs(A) > 1e-12 else float("nan")
                    print("\n  拟合 e(tau_ff) = %.6f*tau_ff %+.6f" % (A, B))
                    print("  抵消重力所需 tau_ff* = %+.4f N·m (使 e=0)" % tau_zero)
                    print("  ⇒ M_ext（以 CAN 侧 tau_ff 单位计）= %+.4f" % tau_zero)

                    # 该点附近的 iq（用插值近似）
                    if rows:
                        # 找最接近 e=0 的行
                        best = min(rows, key=lambda r: abs(r[2]))
                        print("  最接近 e=0 的实测点：tau=%.2f  e=%+.6f  iq=%+.5f"
                              % (best[0], best[2], best[4]))
                        if abs(best[1]) > 1e-9 or True:
                            # iq/tau 的比值（用 |tau| 较大的点更可靠）
                            cand = [r for r in rows if abs(r[0]) > 0.3
                                    and abs(r[4]) > 1e-4]
                            for r in cand[:6]:
                                print("    tau=%+.2f iq=%+.5f  iq/tau=%+.2f"
                                      % (r[0], r[4], r[4] / r[0]))
                            if cand:
                                ktf = sum(r[4] for r in cand) / sum(r[0] for r in cand)
                                print("\n  ★ K_tf = iq / tau_ff_can = %+.2f" % ktf)
                                print("    参考：g/tc = %.2f   (tau_ff 是输出端 N·m 时)")
                                print("          1/tc = %.2f   (tau_ff 被当成电机端时)"
                                      % (1 / TC))
                                print("          1/(g*tc) = %.2f"
                                      % (1 / (GEAR * TC)))
                                for nm, v in (("g/tc", GEAR / TC),
                                              ("1/tc", 1 / TC),
                                              ("1/(g*tc)", 1 / (GEAR * TC))):
                                    print("          与 %-9s 偏差 %.2f"
                                          % (nm, abs(ktf - v)))
        finally:
            for _ in range(8):
                ser.mit(TGT, 0, 0, 0, 0, 0.02)
            ser.w(("B%08X8%s\r" % (mk(0, 0x63, 1, 1, 0), "0" * 16)).encode())
            time.sleep(0.2); ser.r(0.2)
            print("\n已卸力并停机")
    finally:
        try:
            ser.w(b"C\r")
        except OSError:
            pass
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
