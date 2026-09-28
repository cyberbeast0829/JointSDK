#!/usr/bin/env python3
"""CAN FD **5 Mbps 数据段** 真机测试（原生 slcan 串口协议）。

## 为什么要单独一个脚本

`slcand` 只能把串口适配器挂成 SocketCAN，且**不支持 FD 的数据段速率** ——
所以测 5M FD **不能**用 SocketCAN，只能直接用 SDK 自带的 `SlcanHal`
（CANable 2.0 固件的 `b/B`/`d/D` 帧前缀 + `Y<n>` 配置数据段速率）。

## 本脚本验证什么（按“能不能用”的顺序，全部只读，除非加 --armed）

1. **链路**：打开（`C` → `Y5` → `O`，每条等 ACK）、能否听到 FD 心跳；
2. **完整性**：描述符下载（FD 单帧 64 B，最吃带宽也最容易丢）字节数/CRC；
3. **正确性**：端点读（f32/多字节）与心跳解出的物理量对拍；
4. **FD 专属**：`fd_frames` 显示**收发都是 FD**（`rx_fd == 0` 而 `tx_fd > 0`
   就意味着对端在按 Classic 回，或适配器没进 FD 模式）；
5. **限速轮询**（v0.37 的调度器）在 5M FD 下的行为与计数；
6. （`--armed`）闭环中轮询：应答率 / 值是否新鲜 / 是否恒 stale。

## 用法

    export JSDK_LIB_PATH=$PWD/bsh-linux PYTHONPATH=$PWD/bindings/python/src
    python3 tools/canfd_5m_probe.py --port /dev/ttyACM0
    python3 tools/canfd_5m_probe.py --port /dev/ttyACM0 --armed --seconds 8

⚠ `--armed` 会**真的使能电机**（零力矩 + 轻阻尼保持），先确认关节可以小幅移动。
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

import jsdk_can
from jsdk_can import Mode
from jsdk_can.joint import STATE_ALL

# 板载端点（本固件上用来对拍真值）
EP_POS = "axis0.encoder.pos_estimate"
EP_VEL = "axis0.encoder.vel_estimate"

_checks = [0, 0]   # [通过, 失败]


def check(what: str, ok: bool, extra: str = "") -> bool:
    _checks[0 if ok else 1] += 1
    mark = "ok  " if ok else "FAIL"
    print(f"  {mark} {what}" + (f"  {extra}" if extra else ""))
    return ok


def note(msg: str) -> None:
    print(f"  ..   {msg}")


def main() -> int:
    ap = argparse.ArgumentParser(description="CAN FD 5 Mbps 真机测试（原生 slcan）")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--serial-baud", type=int, default=115200,
                    help="串口波特率（不是 CAN 段波特率）")
    ap.add_argument("--data-bitrate", type=int, default=5_000_000,
                    help="FD 数据段速率；0 = 不碰适配器配置（默认 5 Mbps）")
    ap.add_argument("--node", type=int, default=1)
    ap.add_argument("--armed", action="store_true",
                    help="**会动设备**：使能 + 零力矩阻尼保持下轮询")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--period-ms", type=float, default=2.0)
    ap.add_argument("--poll-ms", type=float, default=100.0)
    ap.add_argument("--kp", type=float, default=0.0)
    ap.add_argument("--kd", type=float, default=0.5)
    ap.add_argument("--max-drift", type=float, default=0.05)
    ap.add_argument("--measure-inbound", action="store_true", default=True,
                    help="使能后量一次入站帧率并给出饱和告警（默认开，见 BACKLOG §2.15）")
    ap.add_argument("--no-measure-inbound", dest="measure_inbound", action="store_false",
                    help="跳过入站速率测量")
    args = ap.parse_args()

    db = None if args.data_bitrate else 0
    print(f"=== CAN FD 5 Mbps 测试（原生 slcan，非 SocketCAN）===")
    note(f"端口 {args.port}，串口 {args.serial_baud}，"
         f"数据段 {args.data_bitrate or '不碰（沿用适配器）'} bps")

    lib = jsdk_can._abi.load_library()
    hal = jsdk_can.SlcanHal(args.port, args.serial_baud, db)
    hal.attach(lib)

    # ---- 1) 打开链路（打开序列 C -> Y5 -> O，每条等 ACK）----
    try:
        hal._open()
    except Exception as e:                              # noqa: BLE001
        check("打开 slcan 串口", False, f"{type(e).__name__}: {e}")
        print(f"     原因：{hal.last_open_error()}")
        return 1
    en, rate = hal.fd_config()
    check("打开成功，并向适配器配置了 FD 数据段速率",
          en and rate == args.data_bitrate,
          f"fd_config=({en}, {rate})")

    fd_supported = bool(lib.jsdk_hal_slcan_supports_fd())
    check("后端自报支持 CAN FD", fd_supported)

    ctx = jsdk_can.Context(hal, is_fd=True, is_fd_explicit=True)
    j = ctx.add_joint(args.node)

    try:
        # ---- 2) 只收不发：能不能听到 FD 心跳 ----
        t0 = time.time()
        online = False
        while time.time() - t0 < 2.0:
            ctx.cycle_begin()
            ctx.cycle_end()
            if j.feedback().online:
                online = True
                break
        tx_fd, rx_fd = hal.fd_frames()
        check("2 秒内听到设备（只收不发）", online,
              f"tx_fd={tx_fd} rx_fd={rx_fd}")
        check("收到的是 **FD** 帧（不是 Classic）", rx_fd > 0,
              f"rx_fd={rx_fd}（0 意味着对端在按 Classic 回或适配器没进 FD 模式）")

        # ---- 3) 配置 + 描述符完整性（最吃带宽的一步）----
        t_cfg = time.perf_counter()
        ctx.configure()
        dt_cfg = (time.perf_counter() - t_cfg) * 1e3
        di = ctx.desc_info()
        note(f"configure 用了 {dt_cfg:.0f} ms")
        check("describe 下载完整（crc 非 0）", di.crc != 0,
              f"total_len={di.total_len} crc=0x{di.crc:04X} eps={di.endpoint_count}")
        check("描述符长度在协议上限内（u16 chunkOffset）", di.total_len <= 65535,
              f"{di.total_len} B")
        dev = ctx.device_info()
        check("读到设备信息", dev.hw_version != 0,
              f"hw={dev.hw_version} fw={dev.fw_version} serial={dev.serial} "
              f"classic={dev.classic}")
        check("设备自报 **非 Classic**（FD 链路）", not dev.classic)

        snap = j.config_snapshot()
        gear = float(snap.gear_ratio)
        check("标定值读回（gear_ratio > 0）", gear > 0, f"gear_ratio={gear}")

        # ---- 4) 端点读 vs 心跳：同一物理量的两种来源 ----
        pos_t = float(j.param_get_f32(EP_POS))
        vel_t = float(j.param_get_f32(EP_VEL))
        note(f"端点真值：pos={pos_t:.6f} turns  vel={vel_t:.6f} turns/s")
        check("端点值合理（非 NaN/Inf）",
              all(math.isfinite(v) for v in (pos_t, vel_t)))

        fb = j.feedback()
        note(f"心跳解出：pos={fb.pos:.6f} rad vel={fb.vel:.6f} rad/s "
             f"vbus={fb.vbus_V:.1f}V t_motor={fb.t_motor_C:.1f}C "
             f"axis_state={fb.axis_state} err={fb.err_code}")
        check("心跳里的温度/母线可读且合理",
              0 <= fb.t_motor_C <= 150 and 5 <= fb.vbus_V <= 60,
              f"t={fb.t_motor_C}C vbus={fb.vbus_V}V")

        s = hal.stats()
        note(f"slcan 统计：tx={s['tx']} rx={s['rx']} malformed={s['malformed']} "
             f"acks={s['acks']} nacks={s['nacks']}")
        check("没有适配器 NACK（NACK = 线接反/无终端电阻/波特率不符）",
              s["nacks"] == 0, f"nacks={s['nacks']}")

        # 打开序列会留下一两条启动垃圾，属于**已知无害**；增长才是问题
        mal0 = s["malformed"]

        # ---- 5) 阶段 A：失能态 + SDK 侧限速轮询（v0.37 调度器）----
        print(f"\n=== 阶段 A：失能态，SDK 调度器每 {args.poll_ms:g} ms 轮询 ===")
        # ⚠⚠ 关键：**在途超时必须 ≥ 2 个 tick**。真机实测（5M FD 与 Classic 都一样）：
        #    0x41/0x44 的应答**到达于第 2 拍**（请求发出后，下一次 cycle_begin 才收得到）。
        #    如果这里的 tick 周期 ≫ 在途超时（默认 50 ms），会看到
        #    “state_sent 一直涨、state_ok 恒 0、state_timeout 也跟着涨”的假象 ——
        #    那不是固件不答，而是我们**没收就判了超时**。
        poll_timeout = 50
        tick_ms = max(args.period_ms, args.poll_ms / 2.0)
        if tick_ms * 2 > poll_timeout:
            poll_timeout = int(tick_ms * 4) + 50
            note(f"⚠ tick≈{tick_ms:.0f} ms，在途超时提到 {poll_timeout} ms"
                 f"（默认 50 ms 会不足 2 个 tick ⇒ 全是假超时）")
        ctx.set_state_poll(int(args.poll_ms), fields=STATE_ALL, timeout_ms=poll_timeout)
        n_poll = int(args.seconds / (args.poll_ms / 1000.0))
        for _ in range(n_poll):
            # 每轮泵**两拍**：应答在第 2 拍才到（见上），只泵一拍会测出“0 应答”。
            ctx.cycle_begin()
            ctx.cycle_end()
            time.sleep(min(args.poll_ms, 20) / 1000.0)
            ctx.cycle_begin()
            ctx.cycle_end()
            time.sleep(max(0.0, args.poll_ms / 1000.0 - min(args.poll_ms, 20) / 1000.0))
        bs = ctx.bus_state()
        note(f"调度器计数：state_sent={bs.state_sent} state_ok={bs.state_ok} "
             f"state_timeout={bs.state_timeout}")
        expect = n_poll * 2      # POS_VEL + CURRENT 各一帧
        check("调度器没有超时", bs.state_timeout == 0, f"to={bs.state_timeout}")
        check("调度器按周期发（帧数接近期望，没超发）",
              0.5 * expect <= bs.state_sent <= 2.0 * expect + 4,
              f"sent={bs.state_sent} 期望≈{expect}")
        check("每个请求都拿到了应答（ok 与 sent 相当）",
              bs.state_ok >= bs.state_sent - 2,
              f"ok={bs.state_ok} sent={bs.state_sent}")
        fb = j.feedback()
        # ⚠ 这里**不能**断言 `valid`：它是“**本周期**有没有新帧”的旗标，每次
        #   `cycle_begin()` 先清零。阶段 A 的泵法是“cycle, sleep20, cycle, sleep80”，
        #   第二拍的 `cycle_begin()` 把应答收掉，而**下一次** `cycle_begin()` 又会把
        #   `valid` 清零 ⇒ 在任意时刻采样，`valid` 本来就只有 ~50% 概率为真
        #   （真机实测 sent/ok 全对，但 valid 时真时假）。
        #   新鲜的**正确判据**是 `age_ms` 与阈值（见头文件）。
        check("轮询让反馈保持新鲜（age_ms 远小于阈值）",
              fb.age_ms <= max(50, int(args.poll_ms) * 3),
              f"valid={fb.valid} age={fb.age_ms}ms stale_ms={j.stale_ms()} "
              f"pos={fb.pos:.6f}")
        check("阈值把轮询周期算进去了（不再恒 stale）",
              j.stale_ms() >= int(args.poll_ms),
              f"stale_ms={j.stale_ms()} poll={args.poll_ms:g}")
        # ⚠ 关调度器前**先把在途请求结掉**（多泵两拍收应答）——否则
        #   `set_state_poll(0)` 会把“还有在途”记成一次 `state_timeout`，
        #   并留下一个陈旧的槽位，污染阶段 B 的计数（脚本第一版就踩了：
        #   阶段 B 一开局就看到 to=116、sent 虚高）。
        ctx.cycle_begin(); ctx.cycle_end()
        time.sleep(0.02)
        ctx.cycle_begin(); ctx.cycle_end()

        s = hal.stats()
        check("全程没有新增 malformed（打开序列那两条是启动垃圾）",
              s["malformed"] <= mal0 + 2,
              f"malformed={s['malformed']}（基线 {mal0}）")
        ctx.set_state_poll(0)

        # ---- 6) 阶段 B：闭环中轮询（会动设备）----
        if not args.armed:
            note("未加 --armed ⇒ 不使能、不运动（阶段 B 未跑）")
        else:
            print(f"\n=== 阶段 B：使能（MIT kp={args.kp} kd={args.kd} tau=0），"
                  f"周期 {args.period_ms:g} ms，调度器 {args.poll_ms:g} ms ===")
            fb = j.feedback()
            pos0 = fb.pos
            note(f"保持目标 = {pos0:.9f} rad；漂移保护 {args.max_drift} rad")
            j.enable(Mode.MIT)
            ctx.activate()
            if not check("使能成功", j.is_enabled()):
                ctx.close(); hal.close(); return _summarize()

            # ⚠⚠ 使能后设备会**持续**以 ~475 帧/s 推流（失能时只有 10 帧/s），而
            #     `/dev/ttyACM*` 是 **USB-CDC**：串口 `baud` 对吞吐没有影响，真正的限制
            #     是 USB 的 1 ms 帧节拍 ⇒ 这条链在闭环下会**长期饱和**，`pump_rx()`
            #     追不上入站速率，状态轮询的应答被埋在积压里 ⇒ 记成 `state_timeout`。
            #     这里把入站速率**显式量出来并报出结论**，而不是让后面莫名其妙失败。
            #     （详见 docs/BACKLOG.zh-CN.md §2.15 发现 2）
            if args.measure_inbound:
                b0 = ctx.bus_state()
                t0 = time.perf_counter()
                while time.perf_counter() - t0 < 0.5:
                    ctx.cycle_begin()
                    ctx.cycle_end()
                    time.sleep(0.001)
                dt = time.perf_counter() - t0
                b1 = ctx.bus_state()
                in_rate = (b1.rx_frames - b0.rx_frames) / dt
                note(f"使能后入站速率 ≈{in_rate:.0f} 帧/s"
                     f"（失能时 ~10 帧/s）")
                if in_rate > 300:
                    note("⚠ 该链路在闭环下**长期饱和**：slcan/USB-CDC 撑不住使能后的上报率。")
                    note("  闭环请用 SocketCAN/PCAN；若必须用 slcan，把控制周期放到 ≥20 ms，")
                    note("  并调大 rx_burst_limit —— 否则状态轮询会看似“全是超时”。")


            #   会被插进来的查询帧抢走应答（真机报 activate: timeout 且无故障）。
            ctx.set_state_poll(int(args.poll_ms), fields=STATE_ALL)
            # 阶段 B 的计数按**差值**判断：阶段 A 的轮询也记在同一个累计计数器里，
            # 不减掉基线就会把“阶段 A 的 80 条”算进阶段 B（脚本第一版就是这么虚高的）。
            bs_base = ctx.bus_state()

            pos_seen, ages, lats, cycles, overruns = [], [], [], 0, 0
            next_p = time.perf_counter()
            t_end = next_p + args.seconds
            drift = False
            try:
                while time.perf_counter() < t_end:
                    t0 = time.perf_counter()
                    ctx.cycle_begin()
                    fb = j.feedback()
                    j.set_mit(pos=pos0, vel=0.0, kp=args.kp, kd=args.kd, tau=0.0)
                    ctx.cycle_end()
                    cycles += 1
                    lats.append((time.perf_counter() - t0) * 1e3)
                    if fb.valid:
                        ages.append(fb.age_ms)
                        pos_seen.append(fb.pos)
                        if abs(fb.pos - pos0) > args.max_drift:
                            print(f"  !! 漂移 {abs(fb.pos - pos0):.4f} rad > "
                                  f"{args.max_drift} ⇒ 立即退出并失能")
                            drift = True
                            break
                    next_p += args.period_ms / 1000.0
                    slack = next_p - time.perf_counter()
                    if slack > 0:
                        time.sleep(slack)
                    else:
                        overruns += 1
                        next_p = time.perf_counter()
            finally:
                ctx.set_state_poll(0)
                ctx.deactivate()

            check("退出后关节已失能", not j.is_enabled())
            check("未被漂移保护中断", not drift)
            bs = ctx.bus_state()
            sent_b = bs.state_sent - bs_base.state_sent
            ok_b = bs.state_ok - bs_base.state_ok
            to_b = bs.state_timeout - bs_base.state_timeout
            note(f"阶段 B 调度器计数（已减基线）：state_sent={sent_b} "
                 f"state_ok={ok_b} state_timeout={to_b}")
            expect = max(1.0, args.seconds / (args.poll_ms / 1000.0) * 2.0)
            # ⚠ 这里**不能**要求 `to == 0`：使能后设备以 ~475 帧/s 持续上报，而
            #   USB-CDC 的 1 ms 帧节拍下这条链长期在饱和边缘 ⇒ **偶发**一两次超时
            #   是链路的正常表现（实测 5 次里 4 次是 `to=1`）。真正要判的是
            #   **系统性的垮掉**：`ok` 基本为 0、`to` 与 `sent` 同量级（见 BACKLOG §2.15）。
            check("调度器超时是偶发而非系统性",
                  to_b <= max(2, int(0.05 * sent_b)),
                  f"to={to_b} sent={sent_b}（>5% 就算系统性饱和）")
            check("调度器按周期发（没超发）",
                  0.5 * expect <= sent_b <= 2.0 * expect + 4,
                  f"sent={sent_b} 期望≈{expect:.0f}")
            check("闭环中反馈新鲜（age_ms 合理）",
                  bool(ages) and statistics.median(ages) <= 3 * args.poll_ms,
                  f"age 中位 {statistics.median(ages) if ages else -1:.0f} ms")
            if pos_seen:
                note(f"闭环期间 pos 范围 {min(pos_seen):.6f} … {max(pos_seen):.6f} rad"
                     f"（跨度 {max(pos_seen) - min(pos_seen):.6f}）")
            if lats:
                lat = sorted(lats)
                p99 = lat[min(len(lat) - 1, int(len(lat) * 0.99))]
                note(f"每周期墙钟：中位 {statistics.median(lat):.3f} ms，"
                     f"p99 {p99:.3f} ms，最大 {lat[-1]:.3f} ms；"
                     f"超期 {overruns}/{cycles}")
            pos_after = _read_pos_nonblocking(ctx, j, args.max_drift)
            if pos_seen:
                # 两边同单位：端点 = 电机端 turns，feedback = 输出端 rad
                delta = abs(pos_seen[-1] - pos_after * (2.0 * math.pi / gear))
                note(f"最后一次轮询 vs 失能后真值：Δ={delta:.3e} rad"
                     f"（含失能过渡期间的移动）")
                check("轮询值与真值在漂移量级内（不是垃圾值）",
                      delta <= args.max_drift, f"Δ={delta:.3e} rad")
    finally:
        try:
            ctx.close()
        except Exception:                               # noqa: BLE001
            pass
        try:
            hal.close()
        except Exception:                               # noqa: BLE001
            pass

    return _summarize()


def _read_pos_nonblocking(ctx, j, timeout_s: float = 1.0) -> float:
    """用**非阻塞**路径读一次位置（电机端 turns → 调用方自己换算）。

    ⚠ 为什么不用 ``j.param_get_f32()``：那条路是**同步**的（发出请求后在调用者线程里
      等应答），而控制循环一旦在跑，循环的 ``cycle_begin()`` 会**先**把应答收走 ⇒
      同步读永远等不到。真机实测（5M FD）：闭环在跑时同步 ``param_get`` **0/10 成功**
      （失能态同样代码 10/10 成功）。这正是阶段 1/2 要解决的问题，所以这里也用轮询。
    """
    ctx.set_state_poll(100, fields=STATE_ALL, timeout_ms=500)
    t0 = time.perf_counter()
    v = None
    while time.perf_counter() - t0 < timeout_s:
        ctx.cycle_begin()
        fb = j.feedback()
        ctx.cycle_end()
        if fb.valid:
            v = fb.pos
            break
        time.sleep(0.005)
    ctx.set_state_poll(0)
    if v is None:
        raise RuntimeError("非阻塞读位置超时（1 s 内没有有效反馈）")
    return v


def _summarize() -> int:
    ok, bad = _checks
    print(f"\n=== 结论：{ok} 项通过，{bad} 项失败 ===")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
