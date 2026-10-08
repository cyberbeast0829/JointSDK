"""轮询真机状态轨迹（**只读**，不驱动电机）。

用途：判断"标定/索引搜索是否真的在推进"，还是卡住了。
⚠ **每条请求起一个新进程 + 新会话**，所以每次都要重下描述符（FD 约 0.4 s）；
   观察间隔因此不能设得太小，否则"读的耗时"会盖过"状态变化"。

跑法::

    python -m jsdk_joint_mcp.hw_state_poll --channel COM4 --node 1 --count 6
"""

from __future__ import annotations

import argparse
import sys
import time

from ._mcp_client import McpClient, add_common_args, require_channel

try:  # pragma: no cover
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001
    pass


def main() -> int:
    ap = add_common_args(argparse.ArgumentParser(
        description="轮询真机状态轨迹（只读）"))
    ap.add_argument("--count", type=int, default=6, help="轮询次数（默认 6）")
    ap.add_argument("--interval", type=float, default=3.0,
                    help="间隔秒数（默认 3）")
    args = ap.parse_args()
    rc = require_channel(args)
    if rc is not None:
        return rc

    cli = McpClient(args.iface, args.channel, args.node)
    print(f"轮询真机状态 {args.count} 次（间隔 {args.interval}s）")
    print()
    print(f"{'#':>2}  {'wall':>6}  {'state':>26}  {'resting':>7}  errors(m/e/c)")
    print("-" * 74)

    prev = None
    for i in range(args.count):
        t0 = time.monotonic()
        r = cli.call("jsdk_get_state")
        dt = time.monotonic() - t0
        st = r.get("current_state")
        errs = (f"{r.get('motor_error')}/{r.get('encoder_error')}"
                f"/{r.get('controller_error')}")
        marker = "  <== 变了" if (prev is not None and st != prev) else ""
        print(f"{i:>2}  {dt:>5.1f}s  {str(r.get('current_state_name')):>26}  "
              f"{str(r.get('resting')):>7}  {errs}{marker}")
        prev = st
        if i < args.count - 1:
            time.sleep(args.interval)
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
