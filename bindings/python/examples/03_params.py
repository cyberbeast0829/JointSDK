"""示例 3：读参数 / 写参数 / 批量读 / 配置持久化。

    python examples/03_params.py

⚠ 写参数**会真的改设备**。本示例在虚拟后端上跑，所以安全；换到真机时请先在
   ``--json`` 模式确认量程，并且知道 ``save_config()`` 会写 Flash。
"""

from __future__ import annotations

import sys

from jsdk_can import JsdkError, Context, VirtualHal


def main() -> int:
    with Context(VirtualHal()) as ctx:
        j = ctx.add_joint(1)
        ctx.configure()

        # --- 读：类型随端点自动决定 ---
        gear = j.param_get("axis0.motor.config.gear_ratio")
        node = j.param_get_u32("axis0.config.can.node_id")
        wd = j.param_get_bool("axis0.config.enable_watchdog")
        print(f"gear_ratio={gear}  node_id={node}  enable_watchdog={wd}")

        # --- 批量读：FD 下打包到**一帧**（Classic 自动逐条退化） ---
        got = j.param_get_batch(
            "axis0.motor.config.gear_ratio",
            "axis0.motor.config.torque_constant",
            "axis0.controller.config.mit_max_torque",
            "axis0.config.can.node_id",
        )
        print("\n批量读:")
        for path, value in got.items():
            print(f"  {path:<46} = {value}")

        # --- 写：先查类型，再按类型写（超范围会被拒，绝不静默截断） ---
        j.param_set("axis0.controller.config.vel_limit", 5.0)
        print(f"\nvel_limit -> {j.param_get('axis0.controller.config.vel_limit')}")

        j.param_set_u32("axis0.config.can.heartbeat_rate_ms", 20)
        print(f"heartbeat_rate_ms -> "
              f"{j.param_get_u32('axis0.config.can.heartbeat_rate_ms')}")

        # --- 看门狗（注意 0 的语义） ---
        j.set_watchdog_ms(250)
        print(f"break_timeout -> "
              f"{j.param_get_u32('can.config.break_timeout')} ms")
        print("  注意：0 = 关闭设备侧协议级超时检测（最新固件语义，不是 100 ms）")

        # --- 持久化 ---
        j.save_config()
        print("\n已 CONFIG_SAVE 到 Flash（SDK 会读回校验）")

        # --- 错误路径：类型不符会被明确拒绝 ---
        try:
            j.param_set("axis0.motor.config.gear_ratio", 7)     # int 写 float 端点
        except Exception as exc:                                # noqa: BLE001
            print(f"\n类型不符被拒（这是对的）: {exc}")

        return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except JsdkError as exc:
        # 示例报错就一行 —— 一大段调用栈对"第一次跑"的人是噪音
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(1)
