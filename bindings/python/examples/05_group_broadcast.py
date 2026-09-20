"""示例 5：一条 CAN FD 帧同时驱动多个关节（广播 MIT）。

    python examples/05_group_broadcast.py

（``ctx.pace()`` 让循环按 ``period_ns`` 节拍走；``cycle_end()`` 本身不等待。）

为什么值得用：4 个关节每周期从 4 帧降到 1 帧。FD 数据段 64 字节刚好装下 7 个
关节的紧凑目标（每关节 8 字节：pos/vel/kp/kd/tau 的定点编码）。带宽省下来给
反馈和状态，是几十 Hz 以上控制环路的关键。

设备侧约束（SDK 会在本地先拦一道，不靠设备报错）：
  * ``node_id`` 必须 **1..7** —— 广播帧用**位图**寻址，最高 bit 是第 7 个；
  * 一帧最多 **7** 个关节；
  * 成员必须已使能、已标定、处于 MIT 模式；
  * Classic CAN 下只支持"全员同一目标"，目标不一致会**自动降级为单播**
    （返回成功但 ``last_error()`` 会说明原因 —— 不静默）。
"""

from __future__ import annotations

import math
import sys

from jsdk_can import JsdkError, Context, VirtualHal

SPEC = ("0:id=1,gear=16.5,pmax=12.5,vmax=65,tmax=50,kpmax=500,kdmax=5,hb=10,fd;"
        "1:id=2,gear=16.5,pmax=12.5,vmax=65,tmax=50,kpmax=500,kdmax=5,hb=10,fd;"
        "2:id=3,gear=16.5,pmax=12.5,vmax=65,tmax=50,kpmax=500,kdmax=5,hb=10,fd")


def main() -> int:
    with Context(VirtualHal(SPEC)) as ctx:
        ids = [1, 2, 3]
        joints = [ctx.add_joint(i) for i in ids]
        ctx.configure()

        kp = 0.25
        print(f"group kp={kp}（线上值），成员={ids}")

        # 群组使能也走广播（内部逐节点入队，同周期一次 cycle_begin/end 完成）
        ctx.enable_all()
        for j in joints:
            print(f"  node {j.node_id}: enabled={j.is_enabled()} "
                  f"{j.mode_state()}")
        assert all(j.is_enabled() for j in joints)

        # ---- 1 帧 → 3 个关节 ----
        for step in range(40):
            want = 0.1 * math.sin(2 * math.pi * 0.5 * step * 0.005)
            ctx.cycle_begin()
            ctx.group_set_mit({
                1: {"pos": want, "vel": 0.0, "tau": 0.0, "kp": kp, "kd": kp * 0.05},
                2: {"pos": want, "vel": 0.0, "tau": 0.0, "kp": kp, "kd": kp * 0.05},
                3: {"pos": want, "vel": 0.0, "tau": 0.0, "kp": kp, "kd": kp * 0.05},
            })
            ctx.cycle_end()
            ctx.pace()

        for j in joints:
            fb = j.feedback()
            print(f"  node {j.node_id}: pos={fb.pos:+.4f} tau={fb.torque_Nm:+.3f} "
                  f"age={fb.age_ms}ms")
        print(f"总线: tx={ctx.bus_state().tx_frames} 帧 / rx="
              f"{ctx.bus_state().rx_frames} 帧 （3 个关节用了 1 条目标帧/周期）")

        # ---- 本地前置校验：>7 个成员会被挡下来，不发到总线 ----
        try:
            ctx.cycle_begin()
            ctx.group_set_mit({n: {"pos": 0.0, "kp": 0.0, "kd": 0.0}
                               for n in range(1, 9)})
        except Exception as exc:                             # noqa: BLE001
            print(f"\n8 个成员被本地拒绝: {exc}")
        finally:
            ctx.cycle_end()

        ctx.disable_all()
        ctx.cycle_begin()
        ctx.cycle_end()
        print(f"已停电：{[j.is_enabled() for j in joints]}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except JsdkError as exc:
        # 示例报错就一行 —— 一大段调用栈对"第一次跑"的人是噪音
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(1)
