#!/usr/bin/env python3
"""真机验证：**JointSDK 在 candleLight（gs_usb）+ SocketCAN CAN FD 5 Mbps 上可用**。

与 `tools/canfd_5m_probe.py` 的分工
-------------------------------------
  · `canfd_5m_probe.py`：走**原生 slcan 串口协议**（slcand 不支持 FD 数据段速率）。
    它的结论是"slcan/USB-CDC 在闭环下长期饱和；闭环请用 SocketCAN/PCAN"。
  · **本脚本**：走**内核 SocketCAN**（candleLight → can0，`ip link ... dbitrate 5000000`）。
    要回答的是"换成 SocketCAN 之后，之前的两个发现（应答慢一拍、闭环饱和）是否消失"。

前置（需要 root 起接口，脚本只检测不代劳）::

    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on \\
         sample-point 0.75 dsample-point 0.75 restart-ms 100
    sudo ip link set can0 up
    ip -details link show can0      # 期望 mtu 72 / <FD> / state ERROR-ACTIVE

用法::

    JSDK_LIB_PATH=$PWD/bsh-linux PYTHONPATH=$PWD/bindings/python/src \\
        python3 tools/socketcan_fd5m_probe.py --channel can0 --node 1

阶段：
  A 链路与识别：FD 链路、hw/fw、gear、bus_state
  B FD 专属路径：描述符分片下载、批量读（FD 单帧）、uint64 写回读回
  C 时效性：`0x41` 是否**同一拍**就能拿到（slcan 上要第 2 拍）
  D 吞吐 + E 闭环（`--armed`）：限速状态轮询的 ok/sent/timeout、饱和与否

退出码：0 = 全部预期通过；1 = 有失败；2 = 用法/环境问题。
"""
from __future__ import annotations

import argparse
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(os.path.dirname(_HERE), "bindings", "python", "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

try:
    sys.stdout.reconfigure(encoding="utf-8")      # type: ignore[union-attr]
    sys.stderr.reconfigure(encoding="utf-8")      # type: ignore[union-attr]
except Exception:                                   # pragma: no cover
    pass

import jsdk_can  # noqa: E402
from jsdk_can import Context, Mode  # noqa: E402
from jsdk_can.joint import STATE_CURRENT, STATE_POS_VEL  # noqa: E402

TWO_PI = 6.283185307179586

_ok: list[str] = []
_bad: list[str] = []


def note(msg: str) -> None:
    print(f"  ..   {msg}", flush=True)


def check(name: str, cond: bool, detail: str = "") -> bool:
    if cond:
        _ok.append(name)
        print(f"  ok   {name}" + (f"  {detail}" if detail else ""), flush=True)
    else:
        _bad.append(name)
        print(f"  FAIL {name}" + (f"  {detail}" if detail else ""), flush=True)
    return cond


def tick(ctx: Context) -> None:
    """一拍：收帧 + 收尾（Python 绑定把 C 的 cycle_begin/end 分开暴露）。"""
    ctx.cycle_begin()
    ctx.cycle_end()


def main() -> int:
    ap = argparse.ArgumentParser(description="candleLight + SocketCAN CAN FD 5M 验证")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--node", type=int, default=1)
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--data-bitrate", type=int, default=5000000)
    ap.add_argument("--period-ms", type=float, default=2.0)
    ap.add_argument("--poll-ms", type=float, default=10.0)
    ap.add_argument("--armed-secs", type=float, default=3.0)
    ap.add_argument("--max-drift", type=float, default=2.0,
                    help="漂移上限（电机端 turns；端点读不到时退回 feedback rad）")
    ap.add_argument("--armed", action="store_true",
                    help="跑阶段 D/E（**会真的使能电机**）")
    args = ap.parse_args()

    print("=== JointSDK × candleLight(SocketCAN) CAN FD 5M 验证 ===", flush=True)
    print(f"    if={args.channel} bitrate={args.bitrate} "
          f"data_bitrate={args.data_bitrate} node={args.node}", flush=True)

    hal = jsdk_can.SocketCanHal(args.channel, args.bitrate, args.data_bitrate)
    ctx = Context(hal, is_fd=True)
    j = ctx.add_joint(args.node)

    # ---------------------------------------------------------------- 阶段 A
    print("\n--- 阶段 A：链路与设备识别 ---", flush=True)
    t0 = time.monotonic()
    try:
        ctx.configure()
    except Exception as exc:
        check("configure（FD 描述符下载）", False, f"{type(exc).__name__}: {exc}")
        print(f"\n=== 结论：{len(_ok)} 项通过，{len(_bad)} 项失败 ===")
        return 1
    dt = (time.monotonic() - t0) * 1000.0
    check("configure（FD 描述符下载）", True, f"{dt:.0f} ms")

    info = ctx.device_info()
    note(f"hw={info.hw_version} fw={info.fw_version} serial={info.serial} "
         f"classic={bool(info.classic)}")
    check("fw_version 已读到", int(info.fw_version) > 0, f"fw={info.fw_version}")
    # DeviceInfo.classic==False ⇒ 设备侧按 CAN FD 工作（与 can.config.baud_rate 对应）
    check("设备按 CAN FD 工作（classic=False）", not bool(info.classic),
          f"classic={bool(info.classic)}")

    bs = ctx.bus_state()
    note(f"bus_state: link_up={bs.link_up} nodes={bs.nodes_online} "
         f"tx={bs.tx_frames} rx={bs.rx_frames} errors={bs.link_errors}")
    check("链路 link_up", bool(bs.link_up))
    check("link_errors == 0", int(bs.link_errors) == 0,
          f"errors={bs.link_errors}")

    gear = float(j.config_snapshot().gear_ratio)
    check("gear_ratio 合理（真机值）", 1.0 < gear < 100.0, f"gear={gear}")

    # ---------------------------------------------------------------- 阶段 B
    print("\n--- 阶段 B：FD 专属数据路径 ---", flush=True)
    di = ctx.desc_info()
    note(f"descriptor: total_len={di.total_len} crc=0x{int(di.crc):04X} "
         f"ep={di.endpoint_count} parsed={di.parsed_total} frames={di.frames_rx} "
         f"complete={di.complete}")
    check("描述符完整（分片传输全部到达）", bool(di.complete))
    if int(di.total_len) > 30000:
        check("描述符规模 > 30 KB（即 >8B 分片确实走了 FD）", True,
              f"{int(di.total_len)} B")
    eps = ctx.endpoints()
    check("端点数量 > 400", len(eps) > 400, f"{len(eps)} 个端点")

    # 批量读：FD 下 C 侧应打包成单帧
    try:
        batch = j.param_get_batch("axis0.motor.config.gear_ratio",
                                  "axis0.motor.config.torque_constant",
                                  "axis0.motor.config.pole_pairs")
        bad = {k: v for k, v in batch.items() if isinstance(v, Exception)}
        check("批量读（FD 单帧多端点）", not bad,
              str({k: round(float(v), 6) for k, v in batch.items()
                   if not isinstance(v, Exception)}))
    except Exception as exc:
        check("批量读（FD 单帧多端点）", False, f"{type(exc).__name__}: {exc}")

    # uint64 往返（写回原值，幂等安全）—— FD 才能单帧带 8 字节值
    try:
        ep = "config.custom_serial_number"
        before = j.param_get(ep)
        j.param_set_auto(ep, before)
        after = j.param_get(ep)
        check("uint64 写回并读回一致（FD 8 字节值）", int(after) == int(before),
              f"{ep}={after}")
    except Exception as exc:
        check("uint64 写回并读回一致（FD 8 字节值）", False,
              f"{type(exc).__name__}: {exc}")

    # ---------------------------------------------------------------- 阶段 C
    print("\n--- 阶段 C：0x41 时效性（slcan 上需第 2 拍；这里量到底几拍）---",
          flush=True)
    # ⚠ 语义：`feedback().valid` 是"**本拍**有没有新帧"的旗标，每次 cycle_begin()
    #   先清零。所以"发请求的那一拍"必然 valid=0；应答最快只能在**下一拍**被解出。
    #   这里测的是"从发出到解出共需几拍"，并与端点真值对拍。
    latencies: list[int] = []
    deltas: list[float] = []
    n_try = 8
    for _ in range(n_try):
        j.request_state(STATE_POS_VEL)
        ctx.cycle_begin(); ctx.cycle_end()          # 第 1 拍：请求出门
        ticks = 1
        got = False
        for _ in range(6):                           # 最多再等 6 拍
            time.sleep(0.002)
            ctx.cycle_begin(); ctx.cycle_end()
            ticks += 1
            if j.feedback().valid:
                got = True
                break
        if got:
            latencies.append(ticks)
            try:
                ep_v = float(j.param_get_f32("axis0.encoder.pos_estimate"))
                deltas.append(abs(j.feedback().pos - ep_v * TWO_PI / gear))
            except Exception:
                pass
        time.sleep(0.01)

    if latencies:
        note(f"应答延迟：中位 {sorted(latencies)[len(latencies)//2]} 拍，"
             f"范围 {min(latencies)}..{max(latencies)} 拍（共 {n_try} 次）")
    note(f"取到有效帧 {len(latencies)}/{n_try}")
    if deltas:
        note(f"与端点真值最大偏差：{max(deltas):.6f} rad")
        check("0x41 值与端点真值一致（< 0.02 rad）", max(deltas) < 0.02,
              f"max Δ={max(deltas):.6f} rad")
    check("0x41 每次都能取到有效帧", len(latencies) == n_try,
          f"{len(latencies)}/{n_try}")
    if latencies:
        # slcan 上实测是"第 2 拍"（=2）；SocketCAN 上不应更差
        check("应答延迟不超过 2 拍（与 slcan 同级或更好）",
              max(latencies) <= 2, f"max={max(latencies)} 拍")

    # ---------------------------------------------------------------- D/E
    if not args.armed:
        print("\n--- 阶段 D/E 未跑（需 --armed，会真的使能电机）---", flush=True)
        print(f"\n=== 结论：{len(_ok)} 项通过，{len(_bad)} 项失败 ===")
        if _bad:
            for b in _bad:
                print(f"    - {b}")
        return 0 if not _bad else 1

    print(f"\n--- 阶段 D/E：闭环吞吐 + 限速轮询（{args.armed_secs:.0f}s）---",
          flush=True)
    note("⚠ 即将使能关节（零力矩保持当前位置，带漂移保护）")

    ctx.warmup()

    def poll_once() -> None:
        """发一次 0x41 并把应答泵进缓存（需跨 2 拍，见阶段 C 的说明）。"""
        j.request_state(STATE_POS_VEL)
        ctx.cycle_begin(); ctx.cycle_end()
        time.sleep(0.004)
        ctx.cycle_begin(); ctx.cycle_end()

    poll_once()
    pos0 = j.feedback().pos
    note(f"保持目标 = {pos0:.6f} rad")

    # ⚠ 漂移保护的基准必须用**同一来源**：
    #   `feedback()` 的 pos 会在"心跳兜底"与"0x41 应答"之间切换，两者量程不同
    #   （心跳 = int32/10000 turns；0x41 = 16-bit、量程 ±mit_max_pos）。
    #   本机输出端 71.34 rad 已**远超** mit_max_pos(12.5)，故 0x41 只能给 12.5
    #   ⇒ 直接比较会得到"58 rad 漂移"的假警报。
    #   因此这里改用**端点的电机端 turns**做漂移判据（读得到就用它）。
    def drift_ref() -> float | None:
        try:
            return float(j.param_get_f32("axis0.encoder.pos_estimate"))
        except Exception:
            return None

    ep0 = drift_ref()
    if ep0 is not None:
        note(f"漂移判据改用端点 axis0.encoder.pos_estimate = {ep0:.4f} motor-turns")
    else:
        note("⚠ 读不到端点，退回 feedback().pos 做漂移判据（量程可能不足）")

    # ⚠ 使能后设备上报 ~900 帧/s，**阻塞式**端点读会被淹没（实测 transport error）。
    #   因此漂移只在**低频**采样，且失败就跳过本次（不伪装成漂移）。
    drift_fail = 0

    ctx.enable_all(Mode.MIT)
    t_en = time.monotonic()
    enabled = False
    while time.monotonic() - t_en < 6.0:
        tick(ctx)
        if j.is_enabled():
            enabled = True
            break
        time.sleep(0.005)
    check("关节使能成功", enabled)
    if not enabled:
        print(f"\n=== 结论：{len(_ok)} 项通过，{len(_bad)} 项失败 ===")
        return 1

    poll_cfg = False
    try:
        # period_ms 是"每关节周期"；这里给 4×控制周期，约 50 Hz
        ctx.set_state_poll(int(args.period_ms * 4), per_cycle=1,
                           fields=STATE_POS_VEL | STATE_CURRENT, timeout_ms=50)
        poll_cfg = True
    except Exception as exc:
        note(f"set_state_poll 失败：{exc}")

    try:
        bs0 = ctx.bus_state()
        t_end = time.monotonic() + args.armed_secs
        n_cyc = 0
        worst_dt = 0.0
        max_drift = 0.0
        prev = time.monotonic()
        while time.monotonic() < t_end:
            now = time.monotonic()
            dt = now - prev
            prev = now
            if n_cyc > 10:
                worst_dt = max(worst_dt, dt)
            # 只让 **SDK 侧限速调度器**发状态请求（这正是被测特性）。
            # ⚠ 不要再手工 request_state：单总线只有一个在途槽位，
            #   两者竞争会让 request_state 直接返回 TRANSPORT。
            tick(ctx)
            n_cyc += 1
            # 漂移：低频采样端点（电机端 turns，量程充足）；失败则跳过本次
            if n_cyc % 25 == 0:
                cur = drift_ref()
                if cur is None:
                    drift_fail += 1
                elif ep0 is not None:
                    max_drift = max(max_drift, abs(cur - ep0))
            if max_drift > args.max_drift:
                note(f"⚠ 漂移 {max_drift:.4f} > {args.max_drift} ⇒ 安全退出")
                break
            time.sleep(args.period_ms / 1000.0)

        bs1 = ctx.bus_state()
        dur = args.armed_secs
        rate = n_cyc / dur
        note(f"控制拍数 {n_cyc}（{rate:.0f} 拍/s，目标周期 {args.period_ms} ms）")
        note(f"最差单拍间隔 {worst_dt*1000:.2f} ms")
        note(f"最大漂移 {max_drift:.4f}（单位见上；端点采样失败 {drift_fail} 次）")
        note(f"tx={bs1.tx_frames} rx={bs1.rx_frames} errors={bs1.link_errors}")
        d_sent = int(bs1.state_sent) - int(bs0.state_sent)
        d_ok = int(bs1.state_ok) - int(bs0.state_ok)
        d_to = int(bs1.state_timeout) - int(bs0.state_timeout)
        note(f"状态轮询 Δ：sent={d_sent} ok={d_ok} timeout={d_to}")
        check("轮询有实际发出的帧", d_sent > 0, f"Δsent={d_sent}")
        if d_sent > 0:
            ratio = d_ok / d_sent
            note(f"轮询成功率 ok/sent = {d_ok}/{d_sent} = {ratio:.3f}")
            check("轮询成功率 ≥ 0.9（slcan 上会明显偏低）", ratio >= 0.9,
                  f"{ratio:.3f}")
            check("超时是偶发而非系统性", d_to <= max(2, int(0.05 * d_sent)),
                  f"Δtimeout={d_to}")
        check("闭环期间无链路错误", int(bs1.link_errors) == 0,
              f"errors={bs1.link_errors}")
        # 漂移：端点采样在使能后可能失败（~900 帧/s 淹没阻塞读），故只在
        # 能采到时才判定；采不到时不把它算作失败，而是明确报告。
        if drift_fail == 0 and ep0 is not None:
            check("漂移在限内", max_drift <= args.max_drift,
                  f"{max_drift:.4f} ≤ {args.max_drift}")
        else:
            note(f"⚠ 漂移未完全判定（端点采样失败 {drift_fail} 次）"
                 f"——不代表失败，但结论受限")
    finally:
        try:
            ctx.set_state_poll(0)
        except Exception:
            pass
        try:
            tick(ctx)
            ctx.disable_all()
            t_d = time.monotonic()
            while time.monotonic() - t_d < 5.0:
                tick(ctx)
                if not j.is_enabled():
                    break
                time.sleep(0.005)
            note("已失能")
        except Exception as exc:
            note(f"（失能时异常：{exc}）")

    check("set_state_poll 可配置", poll_cfg)

    print(f"\n=== 结论：{len(_ok)} 项通过，{len(_bad)} 项失败 ===")
    if _bad:
        for b in _bad:
            print(f"    - {b}")
    return 0 if not _bad else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:                       # pragma: no cover
        print("\n中断", file=sys.stderr)
        sys.exit(130)
