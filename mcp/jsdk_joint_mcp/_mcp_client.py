"""真机脚本共用的 MCP stdio 客户端与参数解析。

⚠ 为什么单独一个模块：`hw_smoke` / `hw_write_probe` / `hw_calibrate_probe` /
`hw_state_poll` 都要"起一个 MCP 进程发一条 tools/call 再把 JSON 解回来"。
以前每个脚本各写一份，且**通道写死成 `COM4`** —— 那种脚本进了共享仓库就只能在
作者机器上跑。这里把参数与调用集中一份。
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from typing import Any

#: 起一个 MCP 进程的开销 = 重开总线 + 重下描述符（FD 约 0.4 s / Classic 数秒）。
#: 标定要等 30 s 以上，所以默认给足。
DEFAULT_TIMEOUT_S = 120


def add_common_args(ap: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """给脚本加 `--if/--channel/--node`（与 `hw_smoke` 同名同义）。"""
    ap.add_argument("--if", dest="iface", default="slcan",
                    choices=["slcan", "socketcan", "pcan", "virtual"],
                    help="传输后端（默认 slcan）")
    ap.add_argument("--channel", default="",
                    help="通道名（slcan: COM4 或 /dev/ttyACM0；socketcan: can0）")
    ap.add_argument("--node", type=int, default=1, help="目标 node_id（默认 1）")
    return ap


def require_channel(args: argparse.Namespace) -> int | None:
    """非 virtual 后端必须给 `--channel`。返回非 None 表示应直接退出。"""
    if args.iface != "virtual" and not args.channel:
        print("错误: 非 virtual 后端必须给 --channel（如 --channel COM4）",
              file=sys.stderr)
        return 2
    return None


class McpClient:
    """通过 stdio 调 MCP server 的最小客户端。

    ⚠ **每条请求起一个新进程**：简单、无状态泄漏（真机冒烟要验的正是"冷启动也
    能用"）。代价是每次都要重开总线 + 重下描述符 —— 所以**不适合**跑大量往返；
    生产用法是一个长寿命进程 + 复用会话。
    """

    def __init__(self, iface: str = "slcan", channel: str = "",
                 node: int = 1, timeout_s: int = DEFAULT_TIMEOUT_S) -> None:
        self.iface = iface
        self.channel = channel
        self.node = node
        self.timeout_s = timeout_s
        self._id = 0

    def call(self, name: str, **args: Any) -> dict[str, Any]:
        """调用一个工具，返回**解析后的结构化结果**（不是 MCP 信封）。"""
        self._id += 1
        payload: dict[str, Any] = {"node": self.node, "iface": self.iface}
        if self.channel:
            payload["channel"] = self.channel
        payload.update(args)

        req = json.dumps({
            "jsonrpc": "2.0", "id": self._id, "method": "tools/call",
            "params": {"name": name, "arguments": payload},
        })
        try:
            proc = subprocess.run(
                [sys.executable, "-m", "jsdk_joint_mcp"],
                input=req + "\n", capture_output=True, text=True,
                encoding="utf-8", errors="replace", timeout=self.timeout_s,
            )
        except subprocess.TimeoutExpired:
            return {"ok": False, "error": "CLIENT_TIMEOUT",
                    "message": f"{name} 超过 {self.timeout_s}s 未返回"}

        if proc.returncode != 0:
            return {"ok": False, "error": "SERVER_CRASHED",
                    "message": (proc.stderr or "")[-800:]}

        lines = (proc.stdout or "").strip().splitlines()
        if not lines:
            return {"ok": False, "error": "EMPTY_OUTPUT",
                    "message": (proc.stderr or "")[-400:]}
        try:
            env = json.loads(lines[-1])
        except json.JSONDecodeError as exc:
            return {"ok": False, "error": "BAD_ENVELOPE",
                    "message": f"{exc}: {lines[-1][:300]}"}
        if "error" in env:
            return {"ok": False, "error": "RPC_ERROR",
                    "message": str(env["error"])}
        try:
            return json.loads(env["result"]["content"][0]["text"])
        except Exception as exc:                            # noqa: BLE001
            return {"ok": False, "error": "BAD_PAYLOAD", "message": str(exc)}
