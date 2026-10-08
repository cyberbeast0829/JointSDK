"""MCP server for CyberBeast joint modules — talk to the joint, not to the SDK.

设计目标 / Design goal
-----------------------
用户的目标是"**把关节模组调好**"，不是"用 JointSDK"。所以本 server 暴露的是
**关节的物理概念**（读参数、读错误位、看遥测、标定），而不是 SDK 的 API 形状。

关键约定（与 `AGENTS.md` 一致）：
- 写后必读回：所有写工具返回 ``requested`` / ``value`` / ``verified`` 三个字段，
  **绝不把请求值当结果**。
- 端点 ID 绝不硬编码：一律从设备运行时描述符查。
- 量程/单位绝不写死：写探针前先读。
- 错误给**原始位图**，不给只有名字的摘要。
"""

from .session import JointSession, SessionManager
from .errors import McpToolError, JointNotFound, WriteNotVerified

__all__ = [
    "JointSession",
    "SessionManager",
    "McpToolError",
    "JointNotFound",
    "WriteNotVerified",
]

__version__ = "0.1.0"
