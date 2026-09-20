"""示例 2：读回标定量 + 端点浏览（**只读**）。

展示三件事：
  1. ``dump_config()``：一次拿到全部关键标定量（配置阶段已读回校验）；
  2. ``endpoints()`` / ``endpoints_matching()``：枚举运行时从设备下载的端点表；
  3. ``lookup()``：路径 → 端点 ID/类型/权限。

**为什么要"运行时"端点表**：端点 ID 按 JSON 声明顺序分配，实测跨固件版本
**86% 的端点 ID 会漂移**。所以 SDK 不内置任何静态表，全部现读现用 —— 换固件不会
读错参数。
"""

from __future__ import annotations

from jsdk_can import JsdkError, Context, VirtualHal


def main() -> int:
    with Context(VirtualHal()) as ctx:
        ctx.add_joint(1)
        ctx.configure()

        # --- 1. 标定量 ---
        snap = ctx.dump_config()
        print(f"配置快照（valid={snap.valid}）:")
        for k, v in snap.as_dict().items():
            if k != "valid":
                print(f"  {k:<20} {v}")

        if not snap.valid:
            print("⚠ valid=False：标定参数没读全，物理量 API 会被拒绝（绝不猜量程）")
            return 1

        # 顺手算一下"实际输出端刚度系数"：固件把 kp 作用在电机端 turns 误差上
        import math
        print(f"\n注意：kp 是线上值，输出端实际刚度 = kp × gear / 2π "
              f"= kp × {snap.gear_ratio / (2 * math.pi):.3f}")

        # --- 2. 端点表 ---
        eps = ctx.endpoints()
        print(f"\n端点表：{len(eps)} 条")
        by_type: dict[str, int] = {}
        for e in eps:
            by_type[e.type_name] = by_type.get(e.type_name, 0) + 1
        print("  按类型统计:", ", ".join(f"{k}={v}" for k, v in sorted(by_type.items())))

        print("\nmit_max 家族（现场最常用的一族）:")
        for e in ctx.endpoints_matching("mit_max_"):
            print(f"  {e.ep_id:>6}  {e.access_str}  {e.type_name:<8} {e.path}")

        # --- 3. 路径 → 端点 ---
        ep = ctx.lookup("axis0.motor.config.torque_constant")
        print(f"\nlookup: {ep.path} → id={ep.ep_id} type={ep.type_name} "
              f"access={ep.access_str}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except JsdkError as exc:
        # 示例报错就一行 —— 一大段调用栈对"第一次跑"的人是噪音
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(1)
