"""结构化错误 — 让 agent 能按字段推理，而不是解析人话。

⚠ 这是本 server 的核心设计点之一。
`AGENTS.md` §5: "不要把'人话'当作机器可解析的输出；结构化字段和退出码才是结论的载体。"

所以每个错误都带：
- ``code``     机器可读的错误类（agent 用它分支）
- ``kind``     错误性质（transport / protocol / usage / safety / not_found）
- ``hint``     给 agent 的**下一步该查什么**（不是给用户看的话术）
- ``detail``   原始证据（原始错误位、帧长、读回值…）
"""

from __future__ import annotations

from typing import Any


class McpToolError(Exception):
    """所有工具错误的基类。会被转换成 MCP 的结构化错误响应。"""

    #: 机器可读的错误类，agent 按它分支
    code = "UNKNOWN"
    #: transport / protocol / usage / safety / not_found / device
    kind = "device"

    def __init__(self, message: str, *, hint: str = "", **detail: Any) -> None:
        super().__init__(message)
        self.message = message
        #: 给 agent 的下一步排查建议（不是给终端用户的话术）
        self.hint = hint
        #: 原始证据，原样透传（原始位图、帧长、读回值…）
        self.detail: dict[str, Any] = detail

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "ok": False,
            "error": self.code,
            "kind": self.kind,
            "message": self.message,
        }
        if self.hint:
            out["hint"] = self.hint
        if self.detail:
            out["detail"] = self.detail
        return out


class JointNotFound(McpToolError):
    code = "JOINT_NOT_FOUND"
    kind = "not_found"


class WriteNotVerified(McpToolError):
    """写成功发出回，但读回值不一致 —— 这是**必须**让 agent 看到的情况。

    ⚠ 历史缺陷：CLI 曾经对用户说 ``written:true``，其实什么都不知道。
    这里绝不允许那样：``requested`` 与 ``value`` 必须分开呈现。
    """

    code = "WRITE_NOT_VERIFIED"
    kind = "device"

    def __init__(self, path: str, requested: Any, value: Any, **detail: Any) -> None:
        super().__init__(
            f"写 {path} 后读回值不一致",
            hint=(
                "先按 AGENTS.md §1.2 数一下写帧长度：值宽 <4 的端点（bool/u8/u16）"
                "若未补齐到 >=8 字节会被固件整帧静默丢弃。"
            ),
            path=path,
            requested=requested,
            value=value,
            **detail,
        )


class TransportError(McpToolError):
    code = "TRANSPORT"
    kind = "transport"


class ProtocolError(McpToolError):
    code = "PROTOCOL"
    kind = "protocol"


class UsageError(McpToolError):
    code = "USAGE"
    kind = "usage"


class SafetyRefused(McpToolError):
    """安全闸拒绝执行（会驱动电机的操作没给确认，或状态不允许）。

    ⚠ 这是**特性不是缺陷**：本 server 涉及真实运动控制硬件，
    "拒绝执行"永远优于"猜一下试试"。
    """

    code = "SAFETY_REFUSED"
    kind = "safety"
