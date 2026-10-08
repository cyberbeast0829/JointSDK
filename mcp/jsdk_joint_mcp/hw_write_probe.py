"""真机**写操作**验证（宽/窄类型）。

⚠ 本脚本会**写设备参数**，但每个参数都会在测完**恢复原值**（无论成败）。
它**不驱动电机**。

设计要点（都来自 `AGENTS.md`）：
  §1.1 写后必读回，分 `requested` / `value` / `verified` 三字段
  §1.2 窄类型（`bool`/`u8`/`u16`）若未补齐到 ≥8 字节会被固件静默丢弃
       ⇒ 所以**必须写一个不同的值再改回**。**只写回原值是弱测试**：
         写入被丢弃时读回仍是原值，会**假装成功**。
  §1.5 真机结论要交叉校验（这里用"新进程重新读"独立确认）

跑法::

    python -m jsdk_joint_mcp.hw_write_probe --channel COM4 --node 1
"""

from __future__ import annotations

import argparse
import json
import sys

from ._mcp_client import McpClient, add_common_args, require_channel

try:  # pragma: no cover - 平台相关（中文 Windows 控制台是 cp936）
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001
    pass

#: 测试用例：(路径, 说明)
#: ⚠ 只挑**当前流程未生效**的参数，避免改变设备行为。
#:   `axis0.config.can.is_extended` 之类**绝不能碰**（会改 CAN 帧格式、断通信）。
CASES = [
    ("axis0.config.can.heartbeat_rate_ms", "uint32（宽类型，帧长本来就是 8 B）"),
    ("axis0.config.general_lockin.finish_on_distance",
     "bool（窄类型，**必须补齐到 8 B**，历史 bug 所在）"),
    ("axis0.config.dir_gpio_pin", "uint16（窄类型）"),
]


def show(tag: str, r: dict) -> None:
    print(f"  {tag}")
    print(f"    requested = {r.get('requested')!r}   <- 我请求写入的（**不是结果**）")
    print(f"    value     = {r.get('value')!r}   <- 设备读回的")
    print(f"    verified  = {r.get('verified')!r}   <- 两者是否一致")
    if r.get("error"):
        print(f"    error     = {r['error']}")
    if r.get("hint"):
        print(f"    hint      = {r['hint']}")


def roundtrip(cli: McpClient, path: str, label: str) -> bool:
    """读基线 → 写**不同值** → 恢复 → 独立确认。返回"翻转确实生效"。"""
    print(f"=== {label} ===")
    print(f"路径: {path}")

    b = cli.call("jsdk_read_param", paths=[path])
    if not b.get("ok"):
        print(f"  ❌ 读基线失败: {json.dumps(b, ensure_ascii=False)}")
        return False
    base = b["value"]
    print(f"  基线 = {base!r}  (type={b.get('type')}, id={b.get('id')})")

    if isinstance(base, bool):
        target = not base
    elif isinstance(base, int):
        target = base + 1 if base < 1000 else base - 1
    else:
        print("  (非 bool/int，跳过翻转测试)")
        return False
    lit = str(target).lower() if isinstance(target, bool) else str(target)
    base_lit = str(base).lower() if isinstance(base, bool) else str(base)

    print(f"\n  步骤 1: 写**不同值** {lit!r}（若被静默丢弃，读回仍是 {base!r}）")
    r = cli.call("jsdk_write_param", path=path, value=lit)
    show("写不同值", r)
    ok = (r.get("value") == target) and (r.get("verified") is True)

    print(f"\n  步骤 2: 恢复 {base!r}")
    show("恢复", cli.call("jsdk_write_param", path=path, value=base_lit))

    print("\n  步骤 3: 独立确认（新进程重新读）")
    now = cli.call("jsdk_read_param", paths=[path]).get("value")
    print(f"    现在读到 = {now!r}  (应为 {base!r})")
    restored = (now == base)

    print(f"\n  结论: 翻转生效={ok}  已恢复={restored}\n")
    return bool(ok and restored)


def main() -> int:
    ap = add_common_args(argparse.ArgumentParser(
        description="真机写操作验证（不驱动电机；测完恢复原值）"))
    args = ap.parse_args()
    rc = require_channel(args)
    if rc is not None:
        return rc

    cli = McpClient(args.iface, args.channel, args.node)
    print("#" * 62)
    print(f"# 真机写操作验证  {args.iface} {args.channel or '(virtual)'} node={args.node}")
    print("#" * 62)
    print()

    results = [(f"{label}", roundtrip(cli, path, label)) for path, label in CASES]

    print("=" * 62)
    print("汇总")
    print("=" * 62)
    for label, ok in results:
        print(f"  [{'OK  ' if ok else 'FAIL'}] {label}")
    bad = sum(1 for _, ok in results if not ok)
    print(f"\n{len(results) - bad}/{len(results)} 通过")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
