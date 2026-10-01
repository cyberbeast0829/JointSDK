#!/usr/bin/env python3
"""F32 现场验证：心跳 pos 量程是否已修复（不再被错误钳位到 21.4748 turns）。

背景
----
固件 d10883f5 起，心跳 FD pos/vel 被错误地按 ±214748.0f（= 21.4748 turns，
因为 214748/10000）的**物理量域**钳位，而不是按 int32 原始计数域
（±2147483648/10000 = ±214748.3648 turns）。表现：电机端位置一旦超过
21.4748 turns，心跳 pos 就静默饱和到 214748（0x000346DC），位置信息全错。

固件 2f72ea09 用 ``scale_clamped()`` 根治：按**原始计数域**钳位，
并转 int64 精钳（避免 2^31 的 float32 舍入 UB）。

本脚本
------
位置模式的 ``mit_max_pos = 12.5 rad``（输出端）够不到 21.4748 turns
（= 17.41 rad 输出端），所以必须用 **CSV/速度模式**转过去。

流程：
  1. 后台 candump 抓原始心跳帧（解码 pos/vel 原始计数）
  2. 使能 → CSV 速度模式 → 转到 >21.4748 电机端 turns
  3. 停机，回看样本

判定
----
* 修复后：越过 21.4748 turns 后 pos 原始计数**继续增长**，不卡在 214748。
* 未修复：立即卡死在 214748（0x000346DC）。

运行（在试验机上）
------------------
    JSDK_LIB_PATH=/tmp/bsh-linux PYTHONPATH=/tmp/bindings/python/src \\
        python3 /tmp/f32_hb_probe.py --arm --target-turns 60

⚠ 会真的转动关节！确认关节在**自由空间**、有人值守、随时可断电。
"""

from __future__ import annotations

import argparse
import re
import struct
import subprocess
import sys
import threading
import time

HB_MSG_TYPE = 0x48
SCALE = 10000.0
POS_SAT_OLD = 214748          # 0x000346DC —— 旧 bug 的饱和值
OLD_LIMIT_TURNS = 21.4748     # 214748/10000


# --------------------------------------------------------------------------
# 原始帧解析
# --------------------------------------------------------------------------
def parse_candump_line(line: str):
    m = re.match(r"\s*\S+\s+([0-9A-Fa-f]+)\s+\[(\d+)\]\s+(.*)$", line)
    if not m:
        return None
    can_id = int(m.group(1), 16)
    n = int(m.group(2))
    toks = m.group(3).split()
    if len(toks) < n:
        return None
    payload = bytes(int(t, 16) for t in toks[:n])
    return can_id, payload


def hb_pos_raw(payload: bytes):
    return struct.unpack(">i", payload[8:12])[0] if len(payload) >= 12 else None


def hb_vel_raw(payload: bytes):
    return struct.unpack(">i", payload[12:16])[0] if len(payload) >= 16 else None


class HbSniffer(threading.Thread):
    """后台 candump，只留本节点的心跳帧。"""

    def __init__(self, channel: str, node: int):
        super().__init__(daemon=True)
        self.channel, self.node = channel, node
        self.samples: list[tuple[float, int, int]] = []   # (t, pos_raw, vel_raw)
        self._halt = threading.Event()      # 不能叫 _stop：会遮蔽 Thread._stop()
        self.proc = None

    def run(self):
        self.proc = subprocess.Popen(
            ["candump", self.channel],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
        )
        for line in self.proc.stdout:
            if self._halt.is_set():
                break
            p = parse_candump_line(line)
            if not p:
                continue
            can_id, payload = p
            if (can_id >> 18) & 0xFF != HB_MSG_TYPE:
                continue
            if (can_id >> 2) & 0xFF != self.node:
                continue
            pos = hb_pos_raw(payload)
            if pos is None:
                continue
            self.samples.append((time.time(), pos, hb_vel_raw(payload)))

    @property
    def latest(self):
        return self.samples[-1] if self.samples else None

    def stop(self):
        self._halt.set()
        if self.proc:
            self.proc.terminate()
        self.join(timeout=2)


# --------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description="F32 心跳量程现场验证")
    ap.add_argument("--iface", default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--node", type=int, default=1)
    ap.add_argument("--target-turns", type=float, default=60.0,
                    help="目标电机端 turns（必须远超 21.4748）")
    ap.add_argument("--speed", type=float, default=-8.0,
                    help="CSV 速度模式目标（输出端 rad/s）；先用小值确认方向")
    ap.add_argument("--cur-lim-a", type=float, default=15.0,
                    help="CSV 帧携带的**电机端电流限值(A)**。⚠ 固件 `cmd_vel_control()`\n"
                         "会把 `torque_lim = cur_limit × torque_constant` **无条件覆盖**，\n"
                         "所以这个值若为 0（SDK 的默认！）电流环就不出力 ⇒ 关节不动。")
    ap.add_argument("--vel-lim-rad-s", type=float, default=20.0,
                    help="CSV 的速限（输出端 rad/s），防止失控")
    ap.add_argument("--max-seconds", type=float, default=45.0)
    ap.add_argument("--no-move", action="store_true",
                    help="只观测当前位置，不运动")
    ap.add_argument("--arm", action="store_true", help="确认会让电机出力")
    args = ap.parse_args()

    print("=" * 74)
    print(" F32 现场验证：心跳 pos 量程是否已修复")
    print("=" * 74)
    print(f" 通道={args.channel} 节点={args.node} 目标=±{args.target_turns} 电机端 turns")
    print(f" 旧 bug 阈值 = {OLD_LIMIT_TURNS} turns，饱和值 = {POS_SAT_OLD} "
          f"(0x{POS_SAT_OLD:08X})")
    print()

    sniff = HbSniffer(args.channel, args.node)
    sniff.start()
    time.sleep(0.6)

    from jsdk_can import Context, Mode, SocketCanHal

    with Context(SocketCanHal(args.channel, 1_000_000, 5_000_000),
                 period_ns=5_000_000, auto_keepalive=True) as ctx:
        j = ctx.add_joint(args.node)
        ctx.configure()

        # ---- 基线 ----
        time.sleep(0.5)
        if sniff.latest:
            _, pos, vel = sniff.latest
            print(f"[基线] 心跳 pos = {pos:>12d} ({pos/SCALE: .5f} turns)  "
                  f"vel = {vel}")
        else:
            print("[基线] ✗ 没抓到心跳帧 —— 检查 can0/candump")
            sniff.stop()
            return 2

        if args.no_move:
            rc = report(sniff)
            sniff.stop()
            return rc
        if not args.arm:
            print("\n拒绝：真机运动请加 --arm")
            sniff.stop()
            return 2

        # ---- 使能 + 速度模式 ----
        print(f"\n[动作] 使能 → CSV 速度模式 → {args.speed} rad/s（输出端）")
        print(f"       限流 {args.cur_lim_a} A（电机端）—— **必须非 0**，"
              f"否则固件把 torque_lim 打成 0，关节不出力")
        j.enable(Mode.CSV)
        ctx.activate()                       # 等使能序列 + 安全首帧走完
        print(f"  enabled={j.is_enabled()}  axis_state={j.can_state()}")

        # ⚠ **关键**：CSV/CSP 帧里的 cur_limit 会被固件用来覆盖 `torque_lim`
        #   （`can_cyberbeast.cpp: cmd_vel_control()`：
        #    `axis.motor_.config_.torque_lim = cur_limit_a * torque_constant`）。
        #   不调 set_limits() 时 `j->tgt.cur_lim_A` 是 0（零初始化，**没有**自动默认值），
        #   于是每帧都发 0 A ⇒ torque_lim = 0 ⇒ 电流环被钳到 0 ⇒ 转不动。
        #   这正是本探测首次跑“关节纹丝不动”的原因。
        j.set_limits(args.vel_lim_rad_s, args.cur_lim_a)
        time.sleep(0.1)

        t0 = time.time()
        reached = None
        worst_turns = 0.0
        n = int(args.max_seconds / 0.005)
        for i in range(n):
            ctx.cycle_begin()
            j.set_velocity(args.speed)
            ctx.cycle_end()
            ctx.pace()
            s = sniff.latest
            if s:
                turns = s[1] / SCALE
                if abs(turns) > abs(worst_turns):
                    worst_turns = turns
                if abs(turns) >= args.target_turns:
                    reached = turns
                    break
            if i % 200 == 0 and s:
                print(f"   t={time.time()-t0:5.1f}s  pos={s[1]/SCALE: .4f} turns  "
                      f"(原始 {s[1]})", flush=True)

        # ---- 停机 ----
        print("[动作] 减速停止 + 失能 ...")
        for _ in range(40):
            ctx.cycle_begin()
            j.set_velocity(0.0)
            ctx.cycle_end()
            ctx.pace()
        # ⚠ 停止帧也必须带非 0 限流：若带 0，会把 torque_lim 又打回 0，
        #   之后任何 MIT 都在“无电流”状态下假死。
        j.set_limits(args.vel_lim_rad_s, args.cur_lim_a)
        j.disable()
        for _ in range(20):
            ctx.cycle_begin()
            ctx.cycle_end()
            ctx.pace()
        time.sleep(0.5)

        if reached is not None:
            print(f"[到达] pos = {reached: .4f} turns（已越界 {OLD_LIMIT_TURNS}）")
        else:
            print(f"[警告] 未到达目标，最远 {worst_turns: .4f} turns")

        rc = report(sniff)

    sniff.stop()
    return rc


def report(sniff: HbSniffer) -> int:
    print()
    print("=" * 74)
    print(" 结果判定")
    print("=" * 74)
    if not sniff.samples:
        print("✗ 没有心跳样本")
        return 2

    beyond = [s for s in sniff.samples if abs(s[1] / SCALE) > OLD_LIMIT_TURNS]
    sats = [s for s in sniff.samples if s[1] == POS_SAT_OLD]

    print(f" 样本总数            : {len(sniff.samples)}")
    print(f" |pos| > 21.4748 turns: {len(beyond)}   ← 旧 bug 区间")
    print(f" pos 恰为 214748      : {len(sats)}   ← 旧 bug 特征值")
    print()

    if not beyond:
        print("⚠ 未进入旧 bug 区间 —— 本次**不能**证明 F32 已修。")
        print("  请增大 --target-turns / --max-seconds。")
        return 3

    if sats:
        print("✗ F32 **仍未修复**：位置已越界，但 pos 饱和到 214748。")
        for t, p, v in sats[:5]:
            print(f"      pos={p} vel={v}")
        return 1

    print("✓ F32 **已修复**：越过 21.4748 turns 后 pos 原始计数继续增长，")
    print("  没有卡在 214748（旧 bug 下必然饱和）。")
    print()
    print(" 越界区样本：")
    for t, p, v in beyond[:3]:
        print(f"      pos_raw={p:>12d}  = {p/SCALE: .4f} turns  vel_raw={v}")
    print("      ...")
    for t, p, v in beyond[-3:]:
        print(f"      pos_raw={p:>12d}  = {p/SCALE: .4f} turns  vel_raw={v}")

    peak = max(sniff.samples, key=lambda s: abs(s[1]))
    print()
    print(f" 峰值 |pos| = {abs(peak[1])/SCALE:.4f} turns (原始 {peak[1]})，"
          f"占 int32 满量程 {abs(peak[1])/2147483648*100:.4f}%")
    return 0


if __name__ == "__main__":
    sys.exit(main())
