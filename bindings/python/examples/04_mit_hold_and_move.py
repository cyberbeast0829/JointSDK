"""示例 4：使能 → MIT 保持/正弦运动 → 断电（**会动电机**）。

    python examples/04_mit_hold_and_move.py                 # 虚拟后端，安全
    python examples/04_mit_hold_and_move.py --if socketcan --channel can0 --arm

真机上必须显式 ``--arm``：这个示例会让电机出力。上真机前请读
``docs/PORTING.zh-CN.md`` 的"§7.5.3 现场接线核对表"。

关键安全点（SDK 强制，绕不过去）：
  * ``enable()`` 之后的**第一帧 MIT 一定是全零目标** —— 让电机"待在原地"，而不是
    跳到上一次残留的目标；
  * 每个 MIT 周期都要显式给 ``pos``，控制器按 ``kp·e_p + kd·(−v) + τ_ff`` 解算，
    不给就等于把 kp 乘到 "(目标=0) − 当前位置" 上 → 飞车；
  * 使能前**必须**已 ``configure()``（标定量未知时拒绝出力）。

另外注意 ``ctx.pace()``：``cycle_end()`` **不会**替你等周期。裸机上节拍来自 RTOS
定时器；Python 里不睡就会"CPU 多快就发多快"，看门狗判定和帧间隔全失去意义。
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from jsdk_can import JsdkError, Context, PcanHal, SlcanHal, SocketCanHal, VirtualHal


def make_hal(args):
    """按命令行选项选后端。**每个选项都要真的传到后端**。"""
    if args.iface == "virtual":
        return VirtualHal(args.channel)
    if args.iface == "socketcan":
        return SocketCanHal(args.channel or "can0",
                            args.bitrate, args.data_bitrate)
    if args.iface == "pcan":
        return PcanHal(args.channel or "PCAN_USBBUS1",
                       args.bitrate, args.data_bitrate)
    if args.iface == "slcan":
        return SlcanHal(args.channel or ("COM3" if sys.platform.startswith("win")
                                         else "/dev/ttyACM0"),
                        args.baud, args.data_bitrate)
    raise SystemExit(f"未知后端 {args.iface}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--if", dest="iface", default="virtual")
    ap.add_argument("--channel", default=None)
    ap.add_argument("--bitrate", type=int, default=1_000_000)
    ap.add_argument("--data-bitrate", type=int, default=5_000_000,
                    help="CAN FD 数据段速率；0 = 不碰适配器配置（slcan 用）")
    ap.add_argument("--baud", type=int, default=115_200,
                    help="slcan 的**串口**波特率（不是 CAN 段速率）")
    ap.add_argument("--node", type=int, default=1)
    ap.add_argument("--period-ms", type=float, default=5.0)
    ap.add_argument("--amp", type=float, default=0.2, help="正弦幅值 (rad)")
    ap.add_argument("--freq", type=float, default=0.25, help="正弦频率 (Hz)")
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--arm", action="store_true",
                    help="真机必须显式给出，确认会让电机出力")
    args = ap.parse_args()

    if args.iface != "virtual" and not args.arm:
        print("拒绝：真机运行请加 --arm（本示例会让电机出力）")
        return 2

    # 广播控制（一帧喂多个关节）由 ctx 级 API 提供，见示例 05。
    with Context(make_hal(args),
                 period_ns=int(args.period_ms * 1e6),
                 auto_keepalive=True) as ctx:
        j = ctx.add_joint(args.node)
        ctx.configure()

        # 用**实测**标定量算一个保守的 kp：输出端刚度 ~ 5 N·m/rad
        snap = ctx.dump_config()
        kp = 5.0 * 2 * math.pi / snap.gear_ratio
        print(f"gear={snap.gear_ratio}  tau_max={snap.mit_max_torque} N·m  "
              f"-> kp={kp:.4f}（线上值）")

        j.enable()
        ctx.activate()          # 阻塞等到"使能序列 + 安全首帧"真正走完
        print(f"使能完成: enabled={j.is_enabled()}  mode_state={j.mode_state()}  "
              f"axis_state={j.can_state()}")

        # ---- 阶段 1：保持当前位置（第一帧全零 = 待在原地） ----
        fb = j.feedback()
        hold = fb.pos
        print(f"\n阶段 1 保持 pos={hold:+.4f} rad，30 个周期")
        for _ in range(30):
            ctx.cycle_begin()
            j.set_mit(pos=hold, vel=0.0, tau=0.0, kp=kp, kd=kp * 0.05)
            ctx.cycle_end()
            ctx.pace()
        print(f"  -> pos={j.feedback().pos:+.4f} rad (漂移 "
              f"{j.feedback().pos - hold:+.4f})")

        # ---- 阶段 2：正弦摆动 ----
        n = int(args.seconds / (args.period_ms / 1000.0))
        print(f"\n阶段 2 正弦 {args.amp} rad @ {args.freq} Hz，{n} 个周期")
        worst = 0.0
        for i in range(n):
            # 相位由**周期序号**推进，而不是墙上时钟：这样虚拟后端（时间不走）
            # 和真机（pace() 提供节拍）跑出来的轨迹是同一条。
            t = i * args.period_ms / 1000.0
            want = hold + args.amp * math.sin(2 * math.pi * args.freq * t)
            ctx.cycle_begin()
            j.set_mit(pos=want, vel=0.0, tau=0.0, kp=kp, kd=kp * 0.05)
            ctx.cycle_end()
            ctx.pace()
            err = abs(j.feedback().pos - want)
            worst = max(worst, err)
            if i % max(1, n // 8) == 0:
                print(f"  t={t:5.2f}s want={want:+.4f} now={j.feedback().pos:+.4f} "
                      f"(|e|={err:.4f}) tau={j.feedback().torque_Nm:+.3f} N·m")
        print(f"  跟踪最大偏差 {worst:.4f} rad —— 偏大说明 kp 太小 / 有负载 / 限幅生效")
        print(f"  节拍统计: {ctx.pace_stats()[0]} 个周期，最大落后 "
              f"{ctx.pace_stats()[1]:.2f} ms（明显 >0 说明本机调度不过来）")

        # ---- 阶段 3：退出保持，然后断电 ----
        print("\n阶段 3 回到保持位，再 deactivate()（阻塞到确认断电）")
        for _ in range(20):
            ctx.cycle_begin()
            j.set_mit(pos=hold, vel=0.0, tau=0.0, kp=kp, kd=kp * 0.05)
            ctx.cycle_end()
            ctx.pace()

        # ⚠ `j.disable()` 只是"请求失能"：调用返回后 is_enabled() 还会真好几拍
        #   （实测 5 个控制周期）。想让"已经断电"成为**事实**再返回，用阻塞的
        #   deactivate()：内部走 安全帧 → 等 2 周期 → STOP_MOTOR → 等 IDLE。
        #   直接停发帧不行 —— 设备会在 break_timeout 后报看门狗故障。
        ctx.deactivate()
        print(f"  enabled={j.is_enabled()}  mode_state={j.mode_state()}")

        if j.is_fault():
            print(f"  ⚠ 故障: {j.describe_fault()}  -> j.fault_reset()")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except JsdkError as exc:
        # 示例报错就一行 —— 一大段调用栈对"第一次跑"的人是噪音
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(1)
