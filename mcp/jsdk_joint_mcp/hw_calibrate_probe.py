"""真机**标定**验证。

⚠⚠ **本脚本会驱动电机** —— 全标定会转十几圈电气角，真机实测约 30 s。
   运行前请确认：关节固定牢、周围无人无障碍、电源/急停随手可及。

它验证的是 `AGENTS.md` §3.3 要求的**三样收尾证据**（缺一样就会"谎称成功"）：
  ① `state`          —— 是否回到静息态
  ② `fault`          —— 跑完 ≠ 成功（失败路径正是"进子态→出错→回静息"）
  ③ `pre_calibrated` —— 跑完 ≠ 生效（不会自动置位，要人工写 + save）

⚠ 特别地，② 里"**返回时轴必须已在静息态**"这一条抓到过一个真缺陷：
   `state_is_busy()` 漏了 `MOTOR_DEADTIME_CALIBRATION(16)`，导致 `calibrate`
   在**电机仍在转**的时候就返回 OK（修复前 elapsed 13.4 s，修复后 30.9 s）。

跑法::

    python -m jsdk_joint_mcp.hw_calibrate_probe --channel COM4 --node 1 --yes
"""

from __future__ import annotations

import argparse
import json
import sys
import time

from ._mcp_client import McpClient, add_common_args, require_channel

try:  # pragma: no cover
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001
    pass

PRE_CAL = ["axis0.motor.config.pre_calibrated",
           "axis0.encoder.config.pre_calibrated"]


def main() -> int:
    ap = add_common_args(argparse.ArgumentParser(
        description="真机标定验证（会驱动电机）"))
    ap.add_argument("--yes", action="store_true",
                    help="必须显式给出：确认知道电机会转")
    ap.add_argument("--state-timeout-ms", type=int, default=0,
                    help="等序列跑完的预算；0 = 内置默认 120 s")
    args = ap.parse_args()
    rc = require_channel(args)
    if rc is not None:
        return rc
    if not args.yes:
        print("拒绝执行：标定会驱动电机。确认后请加 --yes", file=sys.stderr)
        return 2

    # 标定 ~30 s + 描述符下载 ⇒ 给足客户端超时
    cli = McpClient(args.iface, args.channel, args.node, timeout_s=300)

    print("#" * 66)
    print("# 真机标定验证  ——  ⚠ 电机会转，约 30 s")
    print("#" * 66)
    print()

    print("=" * 66)
    print("标定前")
    print("=" * 66)
    pre = cli.call("jsdk_get_state")
    print(f"  状态       : {pre.get('current_state_name')} (raw={pre.get('current_state')})")
    print(f"  静息态     : {pre.get('resting')}")
    for k in ("error_raw", "motor_error", "encoder_error", "controller_error"):
        print(f"  {k:10} : 0x{pre.get(k, 0):X}" if isinstance(pre.get(k), int) else "")
    pre_cal = cli.call("jsdk_read_param", paths=PRE_CAL)
    print(f"  pre_calibrated: {json.dumps(pre_cal.get('values'), ensure_ascii=False)}")

    if not pre.get("resting"):
        print("\n⚠ 轴不在静息态 → 标定要求先 disable。中止。")
        return 2

    print()
    print("=" * 66)
    print("执行标定（confirm=true）")
    print("=" * 66)
    t0 = time.monotonic()
    r = cli.call("jsdk_calibrate", confirm=True,
                 state_timeout_ms=args.state_timeout_ms)
    dt = time.monotonic() - t0

    print(f"  墙钟耗时 : {dt:.1f} s（含描述符下载）")
    print(f"  ok       : {r.get('ok')}")
    print(f"  elapsed  : {r.get('elapsed_s')} s  (server 侧计时)")
    for k in ("error", "message", "hint"):
        if r.get(k):
            print(f"  {k:8} : " + str(r[k]).replace("\n", "\n             "))

    print()
    print("=" * 66)
    print("收尾证据（AGENTS.md §3.3 第 5 条：缺一样就会'谎称成功'）")
    print("=" * 66)

    st = r.get("state") or {}
    print("\n① state —— 是否回到静息态")
    print(f"   current_state_name : {st.get('current_state_name')}")
    print(f"   resting            : {st.get('resting')}")
    print(f"   error_raw          : {st.get('error_raw')}")

    subs = (r.get("fault") or {}).get("subsystems") or {}
    nz = {k: v for k, v in subs.items() if isinstance(v, dict) and v.get("value")}
    print("\n② fault —— 跑完 ≠ 成功")
    if nz:
        print("   ⚠ **有非零故障位**:")
        for k, v in nz.items():
            print(f"      {k}: {v.get('value')} ({v.get('hex')})")
    else:
        print("   （全部为 0 —— 无故障）")

    pc = r.get("pre_calibrated") or {}
    print("\n③ pre_calibrated —— 跑完 ≠ 生效")
    for k, v in pc.items():
        if k != "_read_errors":
            print(f"   {k} = {v}")
    print(f"   pre_calibrated_both : {r.get('pre_calibrated_both')}")
    if r.get("note"):
        print("\n   note:")
        for line in str(r["note"]).splitlines():
            print(f"     {line}")

    print()
    print("=" * 66)
    print("标定后（独立再读一次，不复用上面的返回）")
    print("=" * 66)
    post = cli.call("jsdk_get_state")
    print(f"  状态       : {post.get('current_state_name')} (raw={post.get('current_state')})")
    print(f"  resting    : {post.get('resting')}")
    post_cal = cli.call("jsdk_read_param", paths=PRE_CAL)
    print(f"  pre_calibrated: {json.dumps(post_cal.get('values'), ensure_ascii=False)}")

    print()
    print("=" * 66)
    print("判读")
    print("=" * 66)
    at_rest = bool(st.get("resting"))
    print(f"  ① 序列返回且**轴已停** : {at_rest}")
    print(f"  ② 无故障位            : {not nz}")
    print(f"  ③ 标定已生效          : {r.get('pre_calibrated_both')}")
    print()
    if r.get("ok") and at_rest and not nz and not r.get("pre_calibrated_both"):
        print("  ⇒ 序列跑完、设备无故障、轴已停，但**标定未生效**。")
        print("    这是**正常行为**（AGENTS.md §2.1）：pre_calibrated 不会自动置位，")
        print("    必须人工写入两个标志 + save 到 Flash。")
    if not at_rest:
        print("  ⇒ ⚠ **返回时轴不在静息态** —— 不要立即下发其他指令。")
    return 0 if (r.get("ok") and at_rest) else 1


if __name__ == "__main__":
    sys.exit(main())
