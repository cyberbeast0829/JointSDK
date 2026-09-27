#!/usr/bin/env python3
"""真机实验：**闭环运行中**能不能用非阻塞状态请求拿到新鲜的位置/速度。

## 要回答的三个问题（这是 JointROS 那个需求的**前置未知**）

1. **应答吗**：关节在闭环、1 kHz 控制帧在流时，设备还答不答 `QUERY_POS_VEL(0x41)`？
2. **值新吗**：答的话，值是不是真值（还是像主动上报帧那样冻在 `0/0`）？
3. **代价多少**：轮询要占多少总线/多少时间（`req_timeouts`、每周期墙钟耗时）？

三种结果对应三种方案（见 `docs/BACKLOG.zh-CN.md` §1.6）：
答且新鲜 → 阶段 1 的方案成立；答但也是 0/0 → 退回 `0x20` 端点读 + 更严限速；
不答 → **做不到**，直接写结论（不提供假 API）。

## 为什么用 Python 而不是 C

要答的是“**设备行为**”，不是“微秒级抖动”（那是 JointROS 的 RT 自己的事）。
Python 侧还能顺带验证：新绑定 `Joint.request_state()` 在真机上确实能跑。

## 两阶段

* **阶段 A（默认，零风险）**：关节**失能**。先读端点真值
  （`axis0.encoder.pos_estimate`/`vel_estimate`，阻塞读，这里允许），再发 5 次
  `0x41` 对比 —— 这同时验证了帧格式、解码与“0x41 == 端点真值”这条等价性。
* **阶段 B（`--armed`）**：**会真的使能电机**。关节以 **零力矩偏置**保持当前位置
  （`kp`/`kd` 可调，默认纯阻尼 `kp=0, kd=0.5`），控制周期 = `--period-ms`，
  期间每 `--poll-ms` 发一次 `request_state(POS_VEL|CURRENT)`。
  ⚠ **风险**：即使零力矩，重力下也可能**缓慢溜车**。脚本自带漂移保护：
  `|pos - pos0| > --max-drift`（默认 0.05 rad）立即退出并失能。
  请确认关节处于“可以小幅移动/已有支撑或抱闸”的状态再跑。

用法::

    sudo env JSDK_LIB_PATH=$PWD/bsh-linux PYTHONPATH=$PWD/bindings/python/src \\
         python3 tools/state_poll_probe.py --if slcan --channel /dev/ttyACM0 --node 1
    # 加上 --armed 才是使能实验

退出码：0 = 三个问题都有结论；1 = 有失败/被安全保护中断；2 = 用法或环境问题。
"""
from __future__ import annotations

import argparse
import os
import statistics
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(os.path.dirname(_HERE), "bindings", "python", "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

try:                                    # 重定向时 Windows 控制台默认 GBK → 中文乱码
    sys.stdout.reconfigure(encoding="utf-8")      # type: ignore[union-attr]
    sys.stderr.reconfigure(encoding="utf-8")      # type: ignore[union-attr]
except Exception:                                   # pragma: no cover
    pass

import jsdk_can  # noqa: E402
from jsdk_can import Context, Mode, VirtualHal  # noqa: E402
from jsdk_can.joint import STATE_ALL, STATE_CURRENT, STATE_POS_VEL  # noqa: E402

MT_QUERY_POS_VEL = 0x41
MT_QUERY_CURRENT = 0x44

EP_POS = "axis0.encoder.pos_estimate"
EP_VEL = "axis0.encoder.vel_estimate"

# ⚠ 判据的关键事实（JointROS 真机实测）：这版固件**根本不给反馈帧** ——
#   `age_ms = 0xFFFFFFFF`、`FEEDBACK_STALE` 置位、`unicast_poll` 也不刷新。
#   所以 `feedback().valid` 为真这件事**只可能**来自我们自己的 0x41/0x44 应答。
#   （如果哪天固件开始回 MIT 响应，这条判据会变弱 —— 到时要改成按“应答值 vs 端点真值”判。）

_ok = 0
_bad: list[str] = []
_notes: list[str] = []


def check(name: str, cond: bool, detail: str = "") -> bool:
    global _ok
    if cond:
        _ok += 1
        print(f"  ok   {name} {detail}")
    else:
        _bad.append(name)
        print(f"  FAIL {name} {detail}")
    return cond


def note(text: str) -> None:
    _notes.append(text)
    print(f"  ..   {text}")


def build_hal(args):
    """按 `--if` 建后端。

    ⚠ slcan 的 `data_bitrate` 传 **0**（= 不碰适配器配置）：真机实测过 ——
      1 Mbps Classic 的适配器先发 `Y5` 会被配成 FD，之后帧**发不出去**。
    """
    if args.iface == "slcan":
        return jsdk_can.SlcanHal(args.channel, args.baud, 0)
    if args.iface == "socketcan":
        return jsdk_can.SocketCanHal(args.channel, args.bitrate, args.data_bitrate)
    if args.iface == "pcan":
        return jsdk_can.PcanHal(args.channel, args.bitrate, args.data_bitrate)
    if args.iface == "virtual":
        # ⚠ 虚拟后端要的是**设备规格**（不是设备路径）——与 `tools/py_hw_smoke.py` 同一条约定。
        return VirtualHal(args.spec)
    raise SystemExit(f"未知后端 --if {args.iface}")


def read_truth(j, path_pos: str, path_vel: str) -> tuple[float, float, float]:
    """阻塞读端点真值（**只在失能时做**：这条路要独占总线窗口）。"""
    t0 = time.perf_counter()
    pos = float(j.param_get_f32(path_pos))   # type: ignore[arg-type]
    vel = float(j.param_get_f32(path_vel))   # type: ignore[arg-type]
    return pos, vel, (time.perf_counter() - t0) * 1e3


def phase_a(ctx, j, args) -> tuple[float, float]:
    """失能：端点真值 vs `0x41`（这是整条链路的值正确性交叉对拍）。"""
    print("\n=== 阶段 A：失能态 —— 端点真值 vs 0x41 ===")
    fails = 0
    deltas: list[float] = []
    pos0 = vel0 = 0.0

    for i in range(5):
        # ⚠ 每轮**各自**取一次真值：关节失能时是自由的，两次采样之间的
        #    微小移动（没抱闸时的滑移）会破坏“同一个瞬间”的比较。
        pos, vel, ms = read_truth(j, EP_POS, EP_VEL)
        if i == 0:
            pos0, vel0 = pos, vel
            note(f"端点真值（阻塞读，2 个参数共 {ms:.2f} ms）："
                 f"pos={pos:.9f} rad  vel={vel:.9f} rad/s")
            if ms > 50.0:
                note(f"⚠ 单次端点读很慢（{ms:.2f} ms）—— 这正是“不能放进 1 kHz tick”的原因")

        j.request_state(STATE_POS_VEL)
        ctx.cycle_begin()
        fb = j.feedback()
        ctx.cycle_end()
        dpos = abs(fb.pos - pos)
        note(f"  第 {i + 1} 次：valid={int(fb.valid)} age={fb.age_ms} ms "
             f"0x41.pos={fb.pos:.9f} 端点={pos:.9f} Δ={dpos:.3e} rad")
        if not fb.valid:
            fails += 1
        deltas.append(dpos)
        time.sleep(0.05)

    check("0x41 有应答（5/5）", fails == 0, f"未应答 {fails}/5")
    if deltas:
        worst = max(deltas)
        check("0x41 的值 == 端点真值（Δpos < 1e-3 rad）", worst < 1e-3,
              f"最大 Δ={worst:.3e} rad")
    return pos0, vel0


def phase_b(ctx, j, args, pos0: float) -> None:
    """使能 + 零力矩保持：闭环运行中轮询（**本脚本唯一会动设备的阶段**）。"""
    period_s = args.period_ms / 1000.0
    poll_s   = args.poll_ms / 1000.0
    print(f"\n=== 阶段 B：使能（MIT，kp={args.kp} kd={args.kd} tau=0）"
          f"周期 {args.period_ms} ms，每 {args.poll_ms} ms 轮询一次 ===")
    note(f"保持目标 = 当前位置 {pos0:.9f} rad；漂移保护 {args.max_drift} rad")

    j.enable(Mode.MIT)
    ctx.activate()
    if not check("使能成功", j.is_enabled()):
        return

    t_end   = time.perf_counter() + args.seconds
    next_p  = time.perf_counter()
    next_q  = time.perf_counter() + poll_s
    sent = replied = cycles = overruns = pending = 0
    latencies: list[float] = []
    ages: list[int] = []
    pos_seen: list[float] = []
    cur_seen: list[float] = []
    drift_abort = False

    try:
        while time.perf_counter() < t_end:
            t0 = time.perf_counter()
            ctx.cycle_begin()
            fb = j.feedback()          # 上一轮 0x41 的应答在这里已被收帧解出
            j.set_mit(pos=pos0, vel=0.0, kp=args.kp, kd=args.kd, tau=0.0)
            ctx.cycle_end()
            cycles += 1
            latencies.append((time.perf_counter() - t0) * 1e3)

            # 结算上一次请求：这版固件不给其它反馈帧 ⇒ valid 只可能来自它
            if pending:
                if fb.valid:
                    replied += 1
                    ages.append(fb.age_ms)
                    pos_seen.append(fb.pos)
                    cur_seen.append(fb.current_A)
                pending = 0

            if fb.valid and abs(fb.pos - pos0) > args.max_drift:
                print(f"  !! 漂移 {abs(fb.pos - pos0):.4f} rad > {args.max_drift}"
                      f" ⇒ 立即退出并失能")
                drift_abort = True
                break

            now = time.perf_counter()
            if now >= next_q:
                next_q = now + poll_s
                sent += 1
                j.request_state(STATE_ALL)      # 在 tick 间隙发；下个 cycle 收应答
                pending = 1

            next_p += period_s
            slack = next_p - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            else:
                overruns += 1
                next_p = time.perf_counter()
    finally:
        ctx.deactivate()                        # hold_position + STOP + 等 IDLE

    check("退出后关节已失能", not j.is_enabled())
    check("未被安全保护中断（漂移）", not drift_abort)

    print(f"\n  轮询 {sent} 次，其中拿到有效应答 {replied} 次"
          f"（{100.0 * replied / sent if sent else 0:.1f}%）")
    check("0x41 在闭环中也应答", sent > 0 and replied >= max(1, int(0.95 * sent)),
          f"{replied}/{sent}")
    check("闭环中反馈不再陈旧（age_ms 合理）",
          bool(ages) and statistics.median(ages) <= 3 * args.poll_ms,
          f"age 中位数 {statistics.median(ages) if ages else -1:.0f} ms")

    if pos_seen:
        span = max(pos_seen) - min(pos_seen)
        note(f"闭环期间 pos 范围 {min(pos_seen):.6f} … {max(pos_seen):.6f} rad"
             f"（跨度 {span:.6f}）")
    if cur_seen:
        note(f"闭环期间 current_A 范围 {min(cur_seen):.4f} … {max(cur_seen):.4f} A"
             f"（零力矩下应当接近 0）")

    if latencies:
        lat = sorted(latencies)
        p99 = lat[min(len(lat) - 1, int(len(lat) * 0.99))]
        note(f"每周期墙钟：中位 {statistics.median(lat):.3f} ms，p99 {p99:.3f} ms，"
             f"最大 {lat[-1]:.3f} ms；超期 {overruns}/{cycles}（{100.0 * overruns / cycles:.2f}%）")

    # 失能后再读一次真值：与闭环期间最后一次 0x41 的结果对拍
    pos_t, vel_t, _ = read_truth(j, EP_POS, EP_VEL)
    note(f"失能后端点真值：pos={pos_t:.9f} vel={vel_t:.9f}")
    if pos_seen:
        delta = abs(pos_seen[-1] - pos_t)
        # ⚠ 这个 Δ **必然**包含“最后一次轮询 → 失能完成”之间关节的移动（零增益下会溜车，
        #   仿真的玩具模型也会），所以它只能当**量级**判据，不能当等值判据。
        note(f"最后一次轮询值 vs 随后的真值：Δ={delta:.3e} rad"
             f"（含失能过渡期间的移动）")
        check("闭环期间 0x41 的值在**漂移量级**内（不是垃圾值）",
              delta <= args.max_drift, f"Δ={delta:.3e} rad > 漂移上限 {args.max_drift}")


def main() -> int:
    ap = argparse.ArgumentParser(description="闭环中的非阻塞状态请求实验")
    ap.add_argument("--if", dest="iface", default="slcan",
                    choices=["slcan", "socketcan", "pcan", "virtual"])
    ap.add_argument("--channel", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--data-bitrate", type=int, default=0)
    ap.add_argument("--node", type=int, default=1)
    ap.add_argument("--spec", default=None,
                    help="仅 --if virtual 用：设备规格字符串（默认单节点 FD）")
    ap.add_argument("--armed", action="store_true",
                    help="真的使能电机（先确认关节可以小幅移动/有支撑）")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--period-ms", type=float, default=2.0)
    ap.add_argument("--poll-ms", type=float, default=100.0)
    ap.add_argument("--kp", type=float, default=0.0)
    ap.add_argument("--kd", type=float, default=0.5)
    ap.add_argument("--max-drift", type=float, default=0.05)
    args = ap.parse_args()
    if args.spec is None:
        args.spec = ("0:id=1,gear=16.5,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                     "kpmax=500,kdmax=5,hb=10,timeout=30000,fd")

    hal = build_hal(args)
    ctx = Context(hal)
    try:
        j = ctx.add_joint(args.node)
        print(f"=== 非阻塞状态请求真机实验（{'armed' if args.armed else '只读'}）===")
        ctx.configure()
        info = ctx.device_info()
        note(f"设备：hw={info.hw_version} fw={info.fw_version} serial={info.serial}")
        note(f"总线状态：{ctx.bus_state()}")

        # 看门狗：控制周期必须远小于 break_timeout（0 = 设备侧超时检测被禁用）
        try:
            bt = int(j.param_get_u32("can.config.break_timeout"))
        except Exception as exc:                    # pragma: no cover - 端点可能不在
            bt = -1
            note(f"（读 can.config.break_timeout 失败：{exc}）")
        note(f"设备 break_timeout = {bt} ms（0 = 禁用）")
        if bt > 0 and args.period_ms > bt / 4.0:
            print(f"  FAIL 控制周期 {args.period_ms} ms 对 break_timeout {bt} ms 太慢")
            _bad.append("period vs break_timeout")
            return 1

        ctx.warmup()                                 # 会话预热（slcan 首帧丢失）
        pos0, _ = phase_a(ctx, j, args)

        if args.armed:
            phase_b(ctx, j, args, pos0)
        else:
            note("未加 --armed ⇒ 不使能、不运动（阶段 B 未跑）")

        print(f"\n=== 结论：{_ok} 项通过，{len(_bad)} 项失败 ===")
        for b in _bad:
            print(f"  - {b}")
        return 0 if not _bad else 1
    finally:
        try:
            ctx.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
