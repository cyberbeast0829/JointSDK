"""MCP server 主体 — stdio JSON-RPC 传输。

为什么不绑定某个 MCP SDK
--------------------------
`mcp` 官方 Python SDK 的 API 在 0.x → 1.x 之间改过；本 server 只需要
`initialize` / `tools/list` / `tools/call` 三个方法 + stdio 传输，
自己实现反而更稳、依赖更少。

⚠ 关键：工具调用的**结果里没有 "error" 字段**时才是"工具没崩溃"；
工具内部的失败（写丢了、读不回来）用 `ok:false` + `error` 字段在
**正常结果**里返回 —— 这样 agent 能看到结构化解剖，而不是 "tool failed"。
"""

from __future__ import annotations

import json
import sys
import traceback
from typing import Any

from . import __version__
from .session import SessionManager
from .tools import ToolRegistry, build_tools

PROTOCOL_VERSION = "2024-11-05"
SERVER_NAME = "jsdk-joint-mcp"


class McpServer:
    def __init__(self) -> None:
        self.mgr = SessionManager()
        self.reg = ToolRegistry(self.mgr)
        self.tools = build_tools(self.reg)
        self._by_name = {t["name"]: t for t in self.tools}

    # ------------------------------------------------------------------ 分发

    def handle(self, req: dict[str, Any]) -> dict[str, Any] | None:
        method = req.get("method")
        rid = req.get("id")
        params = req.get("params") or {}

        # 通知（无 id）不需要响应
        if rid is None and method and method.startswith("notifications/"):
            return None

        try:
            if method == "initialize":
                return self._ok(rid, {
                    "protocolVersion": PROTOCOL_VERSION,
                    "capabilities": {"tools": {}},
                    "serverInfo": {"name": SERVER_NAME, "version": __version__},
                })
            if method == "tools/list":
                return self._ok(rid, {"tools": self.tools})
            if method == "tools/call":
                return self._ok(rid, self._call_tool(params))
            if method in ("ping", "logging/setLevel"):
                return self._ok(rid, {})
            return self._err(rid, -32601, f"Method not found: {method}")
        except Exception as exc:                            # noqa: BLE001
            return self._err(rid, -32603, "Internal error", {
                "message": str(exc),
                "traceback": traceback.format_exc(),
            })

    # ------------------------------------------------------------- 工具调用

    def _call_tool(self, params: dict[str, Any]) -> dict[str, Any]:
        name = params.get("name", "")
        args = params.get("arguments") or {}

        tool = self._by_name.get(name)
        if tool is None:
            return {
                "content": [{"type": "text", "text": json.dumps({
                    "ok": False,
                    "error": "UNKNOWN_TOOL",
                    "message": f"没有这个工具: {name}",
                    "available": sorted(self._by_name),
                }, ensure_ascii=False, indent=2)}],
                "isError": True,
            }

        fn = getattr(self.reg, name.removeprefix("jsdk_"), None)
        if fn is None or not callable(fn):
            return {
                "content": [{"type": "text", "text": json.dumps({
                    "ok": False,
                    "error": "NOT_IMPLEMENTED",
                    "message": f"工具 {name} 没有实现",
                }, ensure_ascii=False, indent=2)}],
                "isError": True,
            }

        # ⚠ 把异常在 registry 层转成结构化响应（见 ToolRegistry._wrap 注释）
        try:
            result = fn(**args)
        except TypeError as exc:
            result = {
                "ok": False,
                "error": "BAD_ARGUMENTS",
                "kind": "usage",
                "message": f"参数不匹配: {exc}",
                "hint": "检查工具 schema 里的必填字段",
                "detail": {"got": sorted(args), "tool": name},
            }
        except Exception as exc:                            # noqa: BLE001
            result = {
                "ok": False,
                "error": "INTERNAL",
                "kind": "internal",
                "message": str(exc),
                "detail": {"type": type(exc).__name__, "tool": name},
            }

        # ⚠ 工具内部失败（ok:false）**不**设 isError ——
        #    那是"设备告诉我一件事"，不是"这个工具坏了"。
        #    agent 应该拿到解剖后的结果继续推理。
        return {
            "content": [{
                "type": "text",
                "text": json.dumps(result, ensure_ascii=False, indent=2,
                                   default=str),
            }],
        }

    # ---------------------------------------------------------------- 信封

    @staticmethod
    def _ok(rid: Any, result: dict[str, Any]) -> dict[str, Any]:
        return {"jsonrpc": "2.0", "id": rid, "result": result}

    @staticmethod
    def _err(rid: Any, code: int, message: str,
             data: dict[str, Any] | None = None) -> dict[str, Any]:
        err: dict[str, Any] = {"code": code, "message": message}
        if data:
            err["data"] = data
        return {"jsonrpc": "2.0", "id": rid, "error": err}

    # ------------------------------------------------------------------ 主循环

    def serve_stdio(self) -> int:
        """stdin 读一行一个 JSON-RPC，stdout 写一行一个响应。

        ⚠ **stdout 只能有协议消息**。任何调试输出必须走 stderr，
        否则会污染 MCP 流（agent 会看到解析失败）。
        """
        for raw in sys.stdin:
            raw = raw.strip()
            if not raw:
                continue
            try:
                req = json.loads(raw)
            except json.JSONDecodeError as exc:
                self._emit(self._err(None, -32700, "Parse error", {
                    "message": str(exc)
                }))
                continue

            resp = self.handle(req)
            if resp is not None:
                self._emit(resp)
        return 0

    @staticmethod
    def _emit(obj: dict[str, Any]) -> None:
        """写一条 JSON-RPC 响应。

        ⚠ 双重保险：即使 stdout 仍是 cp936（上面 reconfigure 失败/被绕过），
        也**绝不允许**一个字符把整个 server 搞崩 —— 崩了的表现是"连接断开"，
        比"这个字段少了个字符"难查得多。所以编码失败时降级成 ensure_ascii=True
        （纯 ASCII 转义，一定能写出）。
        """
        text = json.dumps(obj, ensure_ascii=False)
        try:
            sys.stdout.write(text + "\n")
        except UnicodeEncodeError:
            # 降级：\uXXXX 转义，任何编码都能表示
            sys.stdout.write(json.dumps(obj, ensure_ascii=True) + "\n")
        sys.stdout.flush()


def main() -> int:
    # ⚠⚠ 必须在**任何输出之前**把 stdout/stderr 切成 UTF-8。
    #
    # 中文 Windows 的默认编码是 cp936，而 MCP 协议输出里会出现 cp936 表示不了的字符
    # （实测：`⇒` U+21D2 出现在工具返回的文本里）→ `sys.stdout.write` 直接抛
    # UnicodeEncodeError ⇒ **server 进程崩掉**，客户端看到的是"连接断开"，
    # 而不是"某个工具返回了一个特殊字符"。
    #
    # 这类崩溃的特点是：**只在真机上、只在某个特定返回内容时才出现** ——
    # 无硬件自检永远测不到，因为那些返回里没有这些字符。
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:                                       # noqa: BLE001, S110
        pass

    server = McpServer()
    try:
        return server.serve_stdio()
    finally:
        server.mgr.close_all()
