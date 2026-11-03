"""工具注册表 — 每个工具的描述本身就是给 agent 的指令。

⚠ 设计要点：MCP 工具的 description 不是"文档"，是**agent 唯一的说明书**。
所以每个描述都要回答：
- 什么时候用（when to use）
- 什么时候**别**用（when NOT to use）
- 返回的字段里哪个是"我看到的"、哪个是"我推断的"

这与 `AGENTS.md` §3.4 的汇报格式一一对应。
"""

from __future__ import annotations

from typing import Any, Callable

from .errors import McpToolError, SafetyRefused, UsageError
from .session import SessionConfig, SessionManager


class ToolRegistry:
    def __init__(self, mgr: SessionManager) -> None:
        self.mgr = mgr

    # ------------------------------------------------------------- 辅助构建

    @staticmethod
    def _cfg(args: dict[str, Any]) -> SessionConfig:
        """从工具入参构造会话配置。只接受显式给出的字段。"""
        known = {f for f in SessionConfig.__dataclass_fields__}
        return SessionConfig(**{k: v for k, v in args.items() if k in known})

    def _session(self, args: dict[str, Any], *, connect: bool = False):
        """取会话。``connect=True`` 时允许隐式建立（用于只读工具）。"""
        cfg = self._cfg(args)
        if connect:
            s = self.mgr.get(cfg)
            s.open()
            return s
        return self.mgr.require(cfg)

    @staticmethod
    def _wrap(fn: Callable[..., dict[str, Any]], *args: Any, **kw: Any) -> dict[str, Any]:
        """把异常统一转换成结构化响应（**不抛给 MCP 框架**）。

        ⚠ 为什么捕获而不抛出：agent 需要"失败的结构化解剖"，
        而不是一句 traceback。这样它能自己决定下一步。
        """
        try:
            return fn(*args, **kw)
        except McpToolError as exc:
            return exc.to_dict()
        except Exception as exc:                            # noqa: BLE001
            return {
                "ok": False,
                "error": "INTERNAL",
                "kind": "internal",
                "message": str(exc),
                "hint": "这是本 server 未预料的异常，请报告；先检查会话状态。",
                "detail": {"type": type(exc).__name__},
            }

    # =====================================================================
    # 工具实现
    # =====================================================================

    # -------- 会话 --------

    def connect(self, node: int, iface: str = "slcan", channel: str = "",
                master_id: int = 1, bitrate: int = 1_000_000,
                data_bitrate: int = 0, baud: int = 115_200,
                classic: bool | None = None, timeout_ms: int = 3_000,
                state_timeout_ms: int = 0,
                load_descriptor: bool = True) -> dict[str, Any]:
        """建立会话并（可选）下载描述符。

        ⚠ **安全起步**：不传 `classic` 时按 Classic 起步，让设备告诉我们它是什么。
        协议**没有运行时协商** —— "心跳收得到、请求没人应" = 猜错了格式。
        """
        cfg = SessionConfig(
            iface=iface, channel=channel, node=node, master_id=master_id,
            bitrate=bitrate, data_bitrate=data_bitrate, baud=baud,
            classic=classic, timeout_ms=timeout_ms,
            state_timeout_ms=state_timeout_ms,
            load_descriptor=load_descriptor,
        )
        s = self.mgr.get(cfg)
        st = s.open()
        if load_descriptor:
            st = s.configure()
        out = {"ok": True, "session": st.to_dict()}
        if not load_descriptor:
            out["note"] = (
                "描述符未下载：按路径访问参数需要它（端点 ID 跨固件漂移 86%，"
                "绝不硬编码）。要读参数请用 load_descriptor=true 重新连接。"
            )
        return out

    def disconnect(self, node: int, iface: str = "slcan", channel: str = "") -> dict[str, Any]:
        cfg = SessionConfig(iface=iface, channel=channel, node=node)
        return {"ok": True, "closed": self.mgr.drop(cfg)}

    def list_sessions(self) -> dict[str, Any]:
        items = self.mgr.list()
        return {"ok": True, "count": len(items), "sessions": items}

    def session_state(self, node: int, iface: str = "slcan", channel: str = "") -> dict[str, Any]:
        cfg = SessionConfig(iface=iface, channel=channel, node=node)
        return {"ok": True, "session": self.mgr.require(cfg).state.to_dict()}

    # -------- 只读 --------

    def device_info(self, node: int, iface: str = "slcan", channel: str = "",
                    timeout_ms: int = 3_000) -> dict[str, Any]:
        """读设备标识（hw/fw/serial）。**不驱动电机，最安全的第一步。**"""
        return self._session(
            {"node": node, "iface": iface, "channel": channel,
             "timeout_ms": timeout_ms}, connect=True
        ).device_info()

    def get_state(self, node: int, iface: str = "slcan", channel: str = "",
                  timeout_ms: int = 3_000) -> dict[str, Any]:
        """读状态：**原始错误位图**（不是 err_name）+ 当前轴状态。

        ⚠ 返回的 `*_error` 是原始位（可核查）；`current_state_name` 是翻译。
        `err_name` 会把看门狗超时与急停混成同一个名字，**不要**用它下结论。
        """
        return self._session(
            {"node": node, "iface": iface, "channel": channel,
             "timeout_ms": timeout_ms}, connect=True
        ).get_state()

    def get_fault_detail(self, node: int, iface: str = "slcan", channel: str = "",
                         timeout_ms: int = 3_000) -> dict[str, Any]:
        """六类故障的**分项位图**。用于区分"看门狗超时"与"急停"。"""
        return self._session(
            {"node": node, "iface": iface, "channel": channel,
             "timeout_ms": timeout_ms}, connect=True
        ).fault_info()

    def get_health(self, node: int, iface: str = "slcan", channel: str = "",
                   timeout_ms: int = 3_000) -> dict[str, Any]:
        """心跳/新鲜度/链路统计快照。含 `tx_retries` / `req_timeouts`。"""
        return self._session(
            {"node": node, "iface": iface, "channel": channel,
             "timeout_ms": timeout_ms}, connect=True
        ).health()

    def read_param(self, node: int, paths: list[str], iface: str = "slcan",
                   channel: str = "", timeout_ms: int = 3_000) -> dict[str, Any]:
        """按路径读参数。单条走 `read`，多条走批量读。

        ⚠ **交叉校验技巧**：对同一个参数分别用 `path=[X]` 和 `path=[X,Y]`
        读两次，数值应一致。不一致说明链路在丢帧。
        """
        s = self._session(
            {"node": node, "iface": iface, "channel": channel,
             "timeout_ms": timeout_ms}, connect=True
        )
        if len(paths) == 1:
            return s.read(paths[0])
        return s.read_many(paths)

    def get_descriptor(self, node: int, path: str | None = None,
                       filter_substr: str | None = None, limit: int = 200,
                       iface: str = "slcan", channel: str = "",
                       timeout_ms: int = 3_000) -> dict[str, Any]:
        """查端点表（路径 → ID / 类型 / 权限 / 是否可写）。

        ⚠ **端点 ID 只能从这里来**。跨固件版本约 86% 的 ID 会漂移，
        所以任何硬编码的 ID 都是错的。
        """
        return self._session(
            {"node": node, "iface": iface, "channel": channel,
             "timeout_ms": timeout_ms}, connect=True
        ).descriptor(path=path, filter_substr=filter_substr, limit=limit)

    # -------- 写 --------

    def write_param(self, node: int, path: str, value: str,
                    iface: str = "slcan", channel: str = "",
                    timeout_ms: int = 3_000) -> dict[str, Any]:
        """写参数 + **强制读回**。

        返回 `requested` / `value` / `verified` 三个字段：
        - `requested` = 我请求写入的值
        - `value`     = 设备读回的值
        - `verified`  = 两者是否一致

        ⚠ **绝不把 requested 当结果看**。若 `verified=false`，
        先按 AGENTS.md §1.2 数写帧长度（值宽 <4 的端点不补齐到 >=8 字节会被静默丢弃）。

        对 `axis0.requested_state` 这类"写进去就被状态机消费"的端点，
        `verified` 返回 `null`（无法用读回值判定，**这不算失败**）。
        """
        return self._session(
            {"node": node, "iface": iface, "channel": channel,
             "timeout_ms": timeout_ms}, connect=True
        ).write(path, value)

    # -------- 动作（需确认） --------

    def fault_reset(self, node: int, confirm: bool = False,
                    iface: str = "slcan", channel: str = "",
                    timeout_ms: int = 3_000) -> dict[str, Any]:
        """清故障（STOP_MOTOR → CLEAR_ERRORS → 等错误位归零）。需要 `confirm=true`。

        ⚠ **原因还在，故障会立刻回来 —— 这是正确行为，不是失败。**
        清完要重新 `get_state`；若又出现，去查根因（温度/编码器/看门狗）。
        """
        # ⚠ 安全闸必须在**碰设备之前**判定：
        #   否则未确认的请求会先去开会话，把一个无关的连接/导入错误
        #   当成对用户的答复（agent 会沿着错的方向排查）。
        if not confirm:
            return SafetyRefused(
                "fault-reset 需要显式确认",
                hint="这是会改变设备状态的操作，调用时请传 confirm=true",
                action="fault-reset",
            ).to_dict()
        return self._session(
            {"node": node, "iface": iface, "channel": channel,
             "timeout_ms": timeout_ms}, connect=True
        ).fault_reset(confirm=confirm)

    def set_zero(self, node: int, confirm: bool = False,
                 iface: str = "slcan", channel: str = "",
                 timeout_ms: int = 3_000) -> dict[str, Any]:
        """把当前位置设为零点。**会改写设备状态**，需要 `confirm=true`。"""
        # ⚠ 同上：先判安全闸，再碰设备。
        if not confirm:
            return SafetyRefused(
                "set-zero 需要显式确认",
                hint="这会把当前位置改写为零点，调用时请传 confirm=true",
                action="set-zero",
            ).to_dict()
        return self._session(
            {"node": node, "iface": iface, "channel": channel,
             "timeout_ms": timeout_ms}, connect=True
        ).set_zero(confirm=confirm)

    # -------- 标定 / 回零 --------

    def calibrate(self, node: int, confirm: bool = False,
                  iface: str = "slcan", channel: str = "",
                  timeout_ms: int = 3_000,
                  state_timeout_ms: int = 0) -> dict[str, Any]:
        """全标定。**电机会转十几圈电气角，需要 `confirm=true`。**

        何时用：装好的关节第一次上电、或换了编码器之后。
        何时**别**用：轴正在使能（闭环）时 —— 会与闭环控制打架，必须先在 IDLE。
        """
        if not confirm:
            return SafetyRefused(
                "calibrate 需要显式确认",
                hint="这会驱动电机转动，调用时请传 confirm=true",
                action="calibrate",
            ).to_dict()
        return self._session(
            {"node": node, "iface": iface, "channel": channel,
             "timeout_ms": timeout_ms, "state_timeout_ms": state_timeout_ms},
            connect=True
        ).calibrate(confirm=confirm)

    def home(self, node: int, confirm: bool = False,
             iface: str = "slcan", channel: str = "",
             timeout_ms: int = 3_000,
             state_timeout_ms: int = 0) -> dict[str, Any]:
        """回零。**会驱动电机，需要 `confirm=true`。**"""
        if not confirm:
            return SafetyRefused(
                "home 需要显式确认",
                hint="这会驱动电机寻零，调用时请传 confirm=true",
                action="home",
            ).to_dict()
        return self._session(
            {"node": node, "iface": iface, "channel": channel,
             "timeout_ms": timeout_ms, "state_timeout_ms": state_timeout_ms},
            connect=True
        ).home(confirm=confirm)

    # -------- 遥测 / 探针 --------

    def get_telemetry(self, node: int, duration_ms: int = 1000,
                      rate_hz: int = 10, iface: str = "slcan",
                      channel: str = "", timeout_ms: int = 3_000) -> dict[str, Any]:
        """抓一段时间的反馈样本（位置/速度/力矩/温度/母线/错误位）。

        **这是只读的，不会驱动电机。**

        何时用：判断"数值是否合理"（温度是否爬升、误差是否收敛、是否有丢样）。
        返回 `summary` 是**可核查的统计**（min/max/漂移/丢样数），不是结论 ——
        你自己判断，别听我们说"看起来正常"。
        """
        return self._session(
            {"node": node, "iface": iface, "channel": channel,
             "timeout_ms": timeout_ms}, connect=True
        ).telemetry(duration_ms=duration_ms, rate_hz=rate_hz)

    def probe_ranges(self, node: int, iface: str = "slcan", channel: str = "",
                     timeout_ms: int = 3_000) -> dict[str, Any]:
        """读**量程与换常量**（gear_ratio / torque_constant / mit_max_* / 限值）。

        ⚠ **写任何探针之前必须先调用这个。** 这些值都会变
        （实测：同一台关节 `torque_constant` 从 0.0824464 变到 0.0864797；
        `mit_max_torque` 曾被当成 18 而实际是 50 ⇒ 实发值差 2.78 倍）。

        ⚠ 越界**不钳位而是回绕**（+12.6 rad → −12.4 rad）⇒ 满力矩反向。
        所以量程只能**读**，绝不能按经验值打包。
        """
        return self._session(
            {"node": node, "iface": iface, "channel": channel,
             "timeout_ms": timeout_ms}, connect=True
        ).probe_ranges()


# =========================================================================
# MCP 工具清单（name → (callable, schema)）
# =========================================================================

#: 复用给几乎所有工具的会话参数
_SESSION_PROPS = {
    "node": {"type": "integer", "description": "目标关节的 node_id"},
    "iface": {
        "type": "string",
        "enum": ["slcan", "socketcan", "pcan", "gsusb", "virtual"],
        "default": "slcan",
        "description": "传输后端",
    },
    "channel": {
        "type": "string",
        "description": "通道名（slcan: COM3 或 /dev/ttyACM0；socketcan: can0）",
    },
    "timeout_ms": {
        "type": "integer", "default": 3000,
        "description": "单次操作超时（毫秒）",
    },
}


def _props(*extra: dict[str, Any]) -> dict[str, Any]:
    out = dict(_SESSION_PROPS)
    for e in extra:
        out.update(e)
    return out


def build_tools(reg: ToolRegistry) -> list[dict[str, Any]]:
    """返回 MCP 工具定义列表。

    ⚠ 工具数量刻意控制。工具太多会让 agent 选错；
    宁可少而精，并在描述里写清"什么时候用/什么时候别用"。
    """
    return [
        # ---------------- 会话 ----------------
        {
            "name": "jsdk_connect",
            "description": (
                "建立到一台关节模组的会话，并（默认）下载 JSON 描述符。\n\n"
                "何时用：任何其他操作之前。\n"
                "何时别用：已经有会话且没改过通道时，直接用其他工具即可（会话会复用）。\n\n"
                "⚠ 安全起步：不传 classic 时按 Classic 起步 —— 协议没有运行时协商，"
                "'心跳收得到、请求没人应' 就是格式猜错了。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": _props(
                    {
                        "master_id": {"type": "integer", "default": 1,
                                      "description": "主站源地址；多会话共用总线时必须各不相同"},
                        "bitrate": {"type": "integer", "default": 1000000},
                        "data_bitrate": {
                            "type": "integer", "default": 0,
                            "description": "0 = 不碰适配器配置（安全起步）；5000000 = 显式要 FD",
                        },
                        "baud": {"type": "integer", "default": 115200,
                                 "description": "**串口**波特率（仅 slcan）"},
                        "classic": {
                            "type": ["boolean", "null"], "default": None,
                            "description": "true=强制 Classic；false=强制 FD；null=先按 Classic 起步",
                        },
                        "load_descriptor": {
                            "type": "boolean", "default": True,
                            "description": "是否下载描述符（按路径读写参数的前提）",
                        },
                        "state_timeout_ms": {
                            "type": "integer", "default": 0,
                            "description": (
                                "状态序列预算（calibrate/home 等“跑一会”的操作）。"
                                "0 = SDK 内置默认（标定 120 s / 回零 5 s）。"
                                "⚠ 与 timeout_ms（单次传输超时）**不是同一回事**："
                                "真机全标定实测 ~30 s，拿 3000 去当预算会在 3 s 就误报超时。"
                            ),
                        },
                    }
                ),
                "required": ["node"],
            },
        },
        {
            "name": "jsdk_disconnect",
            "description": "关闭会话并释放总线。何时用：换通道/换帧格式之前。",
            "inputSchema": {
                "type": "object",
                "properties": _props(),
                "required": ["node"],
            },
        },
        {
            "name": "jsdk_list_sessions",
            "description": "列出当前所有会话及其状态（含 tx/rx 帧数、帧格式学习结果）。",
            "inputSchema": {"type": "object", "properties": {}},
        },
        # ---------------- 只读 ----------------
        {
            "name": "jsdk_device_info",
            "description": (
                "读设备标识（hw_version / fw_version / serial）。\n\n"
                "⚠ 固件版本会变、行为也会变 —— 拿到版本号后再决定其他假设。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": _props(),
                "required": ["node"],
            },
        },
        {
            "name": "jsdk_get_state",
            "description": (
                "读关节状态：**原始错误位图** + 当前轴状态 + 是否处于静息态。\n\n"
                "何时用：任何诊断的第一步；写完 requested_state 之后看结果。\n\n"
                "⚠ 返回的 `*_error` 是**原始位**（可核查），"
                "`current_state_name` 是按固件枚举翻译的。\n"
                "⚠ 不要用 err_name 下结论 —— 它把看门狗超时和急停混成同一个名字。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": _props(),
                "required": ["node"],
            },
        },
        {
            "name": "jsdk_get_fault_detail",
            "description": (
                "读六类故障的**分项位图**（motor / encoder / controller / …）。\n\n"
                "何时用：看到错误位非零、需要区分'看门狗超时 vs 急停'时。\n"
                "这是回答'为什么它报 CAN_TIMEOUT'的正确工具。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": _props(),
                "required": ["node"],
            },
        },
        {
            "name": "jsdk_get_health",
            "description": (
                "心跳/新鲜度/链路统计快照。\n\n"
                "含 `tx_retries`（预热+幂等请求重发次数）与 `req_timeouts`。\n"
                "⚠ 重发次数 >0 说明链路上确实丢过帧 —— 这是**正常现象**，"
                "但若持续增长说明链路质量差。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": _props(),
                "required": ["node"],
            },
        },
        {
            "name": "jsdk_read_param",
            "description": (
                "按路径读参数。传 1 个路径走单读，多个走批量读（FD 单帧）。\n\n"
                "⚠ **交叉校验技巧**：同一参数分别用单读和批量读各读一次，"
                "数值应一致；不一致说明链路在丢帧。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": _props({
                    "paths": {
                        "type": "array", "items": {"type": "string"},
                        "description": "端点路径，如 axis0.motor.config.gear_ratio",
                    },
                }),
                "required": ["node", "paths"],
            },
        },
        {
            "name": "jsdk_get_descriptor",
            "description": (
                "查端点表：路径 → ID / 类型 / 权限 / 是否可写。\n\n"
                "何时用：写参数之前必须查（要知道**类型宽度**）；"
                "想知道某个路径是否存在时。\n\n"
                "⚠ **端点 ID 只能从这里来** —— 跨固件版本约 86% 的 ID 会漂移，"
                "任何硬编码的 ID 都是错的。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": _props({
                    "path": {"type": "string", "description": "精确查某个路径"},
                    "filter_substr": {"type": "string", "description": "子串过滤"},
                    "limit": {"type": "integer", "default": 200},
                }),
                "required": ["node"],
            },
        },
        # ---------------- 写 ----------------
        {
            "name": "jsdk_write_param",
            "description": (
                "写参数 + **强制读回**。这是本 server 最需要小心的工具。\n\n"
                "返回三个字段，必须分开看：\n"
                "  `requested` = 我请求写入的值（**不是结果**）\n"
                "  `value`     = 设备读回的值\n"
                "  `verified`  = 两者是否一致\n\n"
                "⚠ 若 `verified=false`：**第一步不是怀疑设备，是先数写帧长度**。\n"
                "值宽 <4 的端点（bool/u8/u16）若不补齐到 >=8 字节，"
                "会被固件 `if (msg.len < 8) return;` 整帧静默丢弃 —— "
                "症状是'有些参数能写、有些写不动'。\n\n"
                "⚠ 对 `axis0.requested_state` 这类写进去就被状态机消费的端点，"
                "`verified` 返回 null，**这不算失败**；要用 get_state 看 current_state。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": _props({
                    "path": {"type": "string", "description": "端点路径"},
                    "value": {
                        "type": "string",
                        "description": (
                            "文本形式的值。会按端点**声明类型**装箱，"
                            "超值域直接报错（不静默截断）。"
                        ),
                    },
                }),
                "required": ["node", "path", "value"],
            },
        },
        # ---------------- 动作 ----------------
        {
            "name": "jsdk_fault_reset",
            "description": (
                "清故障：STOP_MOTOR → CLEAR_ERRORS → 等错误位归零。**需要 confirm=true**。\n\n"
                "⚠ **原因还在的话故障会立刻回来 —— 这是正确行为，不是失败。**\n"
                "清完重新 get_state；若又出现，去查根因（温度/编码器/看门狗/急停）。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": _props({
                    "confirm": {
                        "type": "boolean", "default": False,
                        "description": "必须显式传 true（会改变设备状态）",
                    },
                }),
                "required": ["node"],
            },
        },
        {
            "name": "jsdk_set_zero",
            "description": (
                "把当前位置设为零点。**需要 confirm=true**（会改写设备状态）。\n\n"
                "何时别用：关节未静止、或不确定机械状态时。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": _props({
                    "confirm": {"type": "boolean", "default": False},
                }),
                "required": ["node"],
            },
        },
        {
            "name": "jsdk_calibrate",
            "description": (
                "全标定（写 requested_state=3 并等序列跑完）。"
                "**电机会转十几圈电气角，需要 confirm=true。**\n\n"
                "何时用：关节第一次上电、或换了编码器之后。\n"
                "何时**别**用：轴正在使能（闭环）时 —— 会与闭环控制打架，必须先在 IDLE。\n\n"
                "返回里会包含三样收尾证据（缺一样就会'谎称成功'）：\n"
                "  `state`          —— 当前状态（是否回到静息态）\n"
                "  `fault`          —— 错误明细；**跑完≠成功**，失败路径正是'进子态→出错→回静息'\n"
                "  `pre_calibrated` —— 两个标定标志\n\n"
                "⚠ `pre_calibrated_both=false` 是**正常行为，不是缺陷** —— "
                "标定跑完不会自动置位，必须人工写两个标志 + save。\n\n"
                "⚠ 若报 'never left idle'：最常见的是**锁存故障**让状态机拒绝启动"
                "（写被接受、值也对，但状态就是不动）→ 先 fault_reset。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": _props({
                    "confirm": {
                        "type": "boolean", "default": False,
                        "description": "必须显式传 true（会驱动电机）",
                    },
                    "state_timeout_ms": {
                        "type": "integer", "default": 0,
                        "description": (
                            "等序列跑完的预算。0 = 内置默认 120 s（真机全标定实测 ~30 s）。"
                            "不要传 3000 之类的小值 —— 那是单次传输超时的量级。"
                        ),
                    },
                }),
                "required": ["node"],
            },
        },
        {
            "name": "jsdk_home",
            "description": (
                "回零（写 requested_state=11 并等）。"
                "**会驱动电机，需要 confirm=true。**\n\n"
                "返回结构与 calibrate 一致（state / fault）。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": _props({
                    "confirm": {"type": "boolean", "default": False},
                    "state_timeout_ms": {
                        "type": "integer", "default": 0,
                        "description": "等序列跑完的预算。0 = 内置默认 5 s。",
                    },
                }),
                "required": ["node"],
            },
        },
        {
            "name": "jsdk_get_telemetry",
            "description": (
                "抓一段时间的反馈样本（位置/速度/力矩/温度/母线/错误位）。"
                "**只读，不驱动电机。**\n\n"
                "何时用：判断'数值是否合理' —— 温度是否爬升、误差是否收敛、是否丢样。\n\n"
                "返回的 `summary` 是**可核查的统计**（min/max/span/age_ms/丢样数），"
                "**不是结论** —— 你自己判断，不要因为这里是'ok:true'就认为正常。\n"
                "⚠ 判新鲜度看 `age_ms`，不要看 `valid`（后者是每周期旗标，约 50% 为真）。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": _props({
                    "duration_ms": {
                        "type": "integer", "default": 1000,
                        "description": "采样时长（毫秒）",
                    },
                    "rate_hz": {
                        "type": "integer", "default": 10,
                        "description": "采样率（1..200）。⚠ 过高会把 slcan 挤爆",
                    },
                }),
                "required": ["node"],
            },
        },
        {
            "name": "jsdk_probe_ranges",
            "description": (
                "读**量程与换常量**（gear_ratio / torque_constant / mit_max_* / 限值）。\n\n"
                "⚠ **写任何探针之前必须先调用这个。**\n\n"
                "这些值都会变：实测同一台关节 `torque_constant` 从 0.0824464 变到 0.0864797；"
                "`mit_max_torque` 曾被当成 18 而实际是 50 ⇒ 实发值差 2.78 倍。\n\n"
                "⚠ 越界**不钳位而是回绕**（+12.6 rad → −12.4 rad）⇒ **满力矩反向**。"
                "所以量程只能读，绝不能按经验值打包。\n\n"
                "⚠ `snapshot_valid=false` 时里面的数值**不可用**，不要当 0 用。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": _props(),
                "required": ["node"],
            },
        },
    ]
