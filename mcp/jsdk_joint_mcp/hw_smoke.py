"""真机冒烟 —— 通过 MCP 协议层调用真实关节（**只读**，不动电机）。

为什么要在 MCP 层做（而不是直接调 Python 绑定）
-------------------------------------------------
因为要验证的是 **MCP server 本身**在路上有没有丢东西：
  - JSON-RPC 信封对不对
  - 工具返回是不是**结构化字段**（而不是 repr 字符串）
  - 编码有没有把中文/特殊字符搞崩（cp936 真机崩过）
  - 交叉校验（单读 vs 批量读）数值一致
  - 自动 configure（不要求调用者先手动走一遍）

跑法 / Usage::

    python -m jsdk_joint_mcp.hw_smoke --channel COM4 --node 1
    python -m jsdk_joint_mcp.hw_smoke --if virtual          # 无硬件也能跑

⚠ 本脚本**只做只读操作**（device_info / get_state / read / probe_ranges /
  telemetry）。它**不会**调用 calibrate / home / set_zero / write_param。
"""

from __future__ import annotations

import argparse
import json
import sys

from ._mcp_client import McpClient, add_common_args, require_channel

try:  # pragma: no cover - 平台相关
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001
    pass

_results: list[tuple[bool, str]] = []


def check(cond: bool, label: str, detail: str = "") -> bool:
    _results.append((bool(cond), label))
    print(f"[{'ok  ' if cond else 'FAIL'}] {label}")
    if detail and not cond:
        for line in str(detail).splitlines()[:12]:
            print(f"         {line}")
    return bool(cond)


def main() -> int:
    ap = add_common_args(argparse.ArgumentParser(
        description="MCP 真机冒烟（只读，不驱动电机）"))
    args = ap.parse_args()
    rc = require_channel(args)
    if rc is not None:
        return rc

    cli = McpClient(args.iface, args.channel, args.node)
    print(f"=== MCP 真机冒烟: {args.iface} {args.channel} node={args.node} ===")
    print("（只读；不会驱动电机）\n")

    # ---------------------------------------------------------- 1 设备信息
    print("--- [1] device_info ---")
    r = cli.call("jsdk_device_info")
    check(r.get("ok") is True, "device_info 成功", r)
    info = r.get("info")
    # ⚠ 必须是**结构化 dict**，不是 "DeviceInfo(...)" 这样的 repr 字符串
    check(isinstance(info, dict), "info 是结构化字段（不是 repr 字符串）",
          f"got {type(info).__name__}: {info!r}")
    if isinstance(info, dict):
        check("hw_version" in info, "含 hw_version")
        check("fw_version" in info, "含 fw_version")
        print(f"         hw={info.get('hw_version')} fw={info.get('fw_version')} "
              f"serial={info.get('serial')} classic={info.get('classic')}")

    # ---------------------------------------------------------- 2 状态
    print("\n--- [2] get_state（自动 configure）---")
    r = cli.call("jsdk_get_state")
    check(r.get("ok") is True, "get_state 成功（**无需**调用者先手动 configure）", r)
    check("error_raw" in r, "给**原始**错误位（不是 err_name）")
    check("current_state_name" in r, "给翻译后的状态名")
    check("session" in r, "附会话状态（可核查帧数）")
    s = r.get("session") or {}
    check(s.get("descriptor_loaded") is True, "描述符已从设备下载")
    check(s.get("configured") is True, "会话已配置")
    print(f"         error_raw={r.get('error_raw')} "
          f"({r.get('motor_error_hex')}/{r.get('encoder_error_hex')}/"
          f"{r.get('controller_error_hex')}) "
          f"state={r.get('current_state_name')} resting={r.get('resting')}")
    print(f"         tx={s.get('tx_frames')} rx={s.get('rx_frames')} "
          f"retries={s.get('tx_retries')} timeouts={s.get('req_timeouts')}")

    # ---------------------------------------------------- 3 交叉校验
    print("\n--- [3] 交叉校验：单读 vs 批量读 ---")
    GEAR = "axis0.motor.config.gear_ratio"
    TC = "axis0.motor.config.torque_constant"
    a = cli.call("jsdk_read_param", paths=[GEAR])
    b = cli.call("jsdk_read_param", paths=[GEAR, TC])
    check(a.get("ok") is True, "单读成功", a)
    check(b.get("ok") is True, "批量读成功", b)
    v1 = a.get("value")
    v2 = (b.get("values") or {}).get(GEAR)
    consistent = (v1 is not None and v2 is not None
                  and abs(float(v1) - float(v2)) <= 1e-6 * max(1.0, abs(float(v1))))
    check(consistent, "单读与批量读**数值一致**（不一致=链路丢帧）",
          f"single={v1!r} batch={v2!r}")
    print(f"         gear_ratio={v1}  (id={a.get('id')}, type={a.get('type')})")
    print(f"         torque_constant={(b.get('values') or {}).get(TC)}")

    # ---------------------------------------------------- 4 量程
    print("\n--- [4] probe_ranges ---")
    r = cli.call("jsdk_probe_ranges")
    check(r.get("ok") is True, "probe_ranges 成功", r)
    rng = r.get("ranges") or {}
    check("mit_max_torque" in rng, "含 mit_max_torque")
    check("torque_constant" in rng, "含 torque_constant")
    # ⚠ 交叉校验：ranges 里的值和单独读出来的必须一致
    cross = r.get("cross_check") or {}
    if cross.get(GEAR) is not None and v1 is not None:
        check(abs(float(cross[GEAR]) - float(v1)) <= 1e-6,
              "ranges 与单读的 gear_ratio 一致", f"{cross.get(GEAR)} vs {v1}")
    check("how_to_use" in r, "给出使用说明（含'越界回绕'的警告）")
    print(f"         mit_max_pos={rng.get('mit_max_pos')} "
          f"vel={rng.get('mit_max_vel')} torque={rng.get('mit_max_torque')} "
          f"kp={rng.get('mit_max_kp')} kd={rng.get('mit_max_kd')}")
    print(f"         torque_constant={rng.get('torque_constant')} "
          f"break_timeout={rng.get('break_timeout_ms')} "
          f"snapshot_valid={r.get('snapshot_valid')}")

    # ---------------------------------------------------- 5 遥测
    print("\n--- [5] get_telemetry（只读，抓 500ms）---")
    r = cli.call("jsdk_get_telemetry", duration_ms=500, rate_hz=10)
    check(r.get("ok") is True, "telemetry 成功（**server 没崩** = 编码没问题）", r)
    summ = r.get("summary") or {}
    check(summ.get("usable_samples", 0) > 0, "有可用样本",
          f"usable={summ.get('usable_samples')} errors={summ.get('errors')}")
    check("age_ms" in summ, "给 age_ms（判新鲜度用它，不是 valid）")
    print(f"         count={r.get('count')} usable={summ.get('usable_samples')} "
          f"errors={summ.get('errors')}")
    for f in ("t_motor_C", "t_fet_C", "vbus_V"):
        if f in summ:
            print(f"         {f}: {summ[f]['min']} .. {summ[f]['max']}")

    # ---------------------------------------------------- 6 故障明细
    print("\n--- [6] get_fault_detail ---")
    r = cli.call("jsdk_get_fault_detail")
    check(r.get("ok") is True, "fault_detail 成功", r)
    check("subsystems" in r, "给六类分项（不是只有名字）")
    subs = r.get("subsystems") or {}
    nz = {k: v for k, v in subs.items()
          if isinstance(v, dict) and v.get("value")}
    print(f"         非零子系统: {nz if nz else '（全部为 0）'}")

    # ---------------------------------------------------- 7 安全闸
    print("\n--- [7] 安全闸（未确认必须拒绝）---")
    r = cli.call("jsdk_calibrate")
    check(r.get("ok") is False and r.get("error") == "SAFETY_REFUSED",
          "calibrate 未确认 → 被拒绝（不会转电机）", r)

    # ---------------------------------------------------- 汇总
    total = len(_results)
    bad = sum(1 for ok, _ in _results if not ok)
    print(f"\n{'=' * 60}")
    print(f"真机冒烟: {total - bad}/{total} 通过")
    print(f"{'=' * 60}")
    if bad:
        print("\n失败项:")
        for ok, label in _results:
            if not ok:
                print(f"  - {label}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
