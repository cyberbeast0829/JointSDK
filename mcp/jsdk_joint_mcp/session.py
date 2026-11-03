"""会话管理 — 把"有状态的物理设备"建模成 agent 可用的对象。

为什么 MCP 需要会话层，而不是每次工具调用都开一次总线
--------------------------------------------------------
⚠ 这是本骨架最重要的设计决策，来自真机事实：

1. **slcan 打开端口后头 1~2 帧上不了总线**（`AGENTS.md` §1.6）。
   每次调用都 `open()` 一次 = 每次都在踩首帧丢失，而且每次都要重下 38 KB 描述符。
   实测代价：FD 1+662 帧 / Classic 1+6906 帧，整份流 ~3.5 s。

2. **描述符必须从设备下载**（端点 ID 跨固件漂移 86%），
   会话内缓存是刚需，不然每个工具调用都要等 3.5 s。

3. **标定/回零是长过程**（真机 29.5 s），必须能被轮询观察，不能阻塞在一个调用里。

4. **`node_id` 可以变** —— 会话必须容忍寻址改变。

因此：一个进程内维护一组**长寿命** Context，工具调用是"往会话里发动作"。
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from dataclasses import fields as dataclasses_fields
from dataclasses import is_dataclass as dataclasses_is_dataclass
from typing import Any

from .errors import (
    JointNotFound,
    ProtocolError,
    SafetyRefused,
    TransportError,
    UsageError,
)


# --------------------------------------------------------------------------
# 设备事实：这些**不能**写死，只在这里记录"如何取得"
# --------------------------------------------------------------------------

#: 这些端点路径是**稳定**的（不是 ID！ID 会漂移，路径相对稳定）。
#: AGENTS.md §2.2：值必须现场读，但路径名本身是协议的一部分。
#:
#: ⚠⚠ **路径也不是凭记忆写的** —— 实测发现 `can.heartbeat_rate_ms` 与 `can.node_id`
#: 都不存在，真实路径是 `axis0.config.can.*`。所以 `configure()` 之后会跑
#: `_validate_ep_paths()`，把缺失的路径**明确报出来**（而不是等到调用时报
#: 一个困惑的 `not_found`）。
#:
#: 本机实测（fw 1547 / 576 端点）的 ID 列在注释里，**仅供对照，绝不用于寻址**。
EP = {
    "error_raw":        "axis0.error",                          # id 138
    "motor_error":      "axis0.motor.error",                    # id 193 (uint64!)
    "encoder_error":    "axis0.encoder.error",                  # id 365 (uint16)
    "controller_error": "axis0.controller.error",               # id 268 (uint8)
    "current_state":    "axis0.current_state",                  # id 142
    "requested_state":  "axis0.requested_state",                # id 143
    "pos_estimate":     "axis0.encoder.pos_estimate",           # id 372
    "gear_ratio":       "axis0.motor.config.gear_ratio",        # id 242
    "torque_constant":  "axis0.motor.config.torque_constant",   # id 247
    "pre_cal_motor":    "axis0.motor.config.pre_calibrated",    # id 240
    "pre_cal_encoder":  "axis0.encoder.config.pre_calibrated",  # id 394
    "enable_watchdog":  "axis0.config.enable_watchdog",         # id 154
    "heartbeat_rate":   "axis0.config.can.heartbeat_rate_ms",   # id 182  ← 曾写错
    "break_timeout":    "can.config.break_timeout",             # id 73
    "node_id":          "axis0.config.can.node_id",             # id 180  ← 曾写错
}

#: 标定/回零等**关键路径**的这个子集必须在，否则要明确告警。
EP_REQUIRED = (
    "error_raw", "motor_error", "encoder_error", "controller_error",
    "current_state", "requested_state", "gear_ratio", "torque_constant",
)

#: 静息态（`AGENTS.md` §2.1：判据是"离开静息态"，不是"等于请求值"）
RESTING_STATES = {1, 8}   # IDLE, CLOSED_LOOP_CONTROL

#: ⚠⚠ **不要自己维护 AxisState 名字表** —— 实测踩过：
#: 我按记忆写了一份（6/7 写反、14 名字错、15/16 缺失），
#: 于是把设备的 `ENCODER_OFFSET_CALIBRATION(7)` 报成 `ENCODER_INDEX_SEARCH`，
#: 把 `MOTOR_DEADTIME_CALIBRATION(16)` 报成 `UNKNOWN(16)`。
#:
#: 权威来源是 SDK 自己的 `jsdk_axis_state_string()`（`src/core/jsdk_fault.c`
#: 的 `k_can_axis_state[17]`）。本模块**直接调它**，本地表只作 fallback。
AXIS_STATES_FALLBACK = {
    0: "UNDEFINED", 1: "IDLE", 2: "STARTUP_SEQUENCE",
    3: "FULL_CALIBRATION_SEQUENCE", 4: "MOTOR_CALIBRATION",
    6: "ENCODER_INDEX_SEARCH", 7: "ENCODER_OFFSET_CALIBRATION",
    8: "CLOSED_LOOP_CONTROL", 9: "LOCKIN_SPIN", 10: "ENCODER_DIR_FIND",
    11: "HOMING", 12: "ENCODER_HALL_POLARITY_CALIBRATION",
    13: "ENCODER_HALL_PHASE_CALIBRATION", 14: "INERTIA_CALIBRATION",
    15: "ENCODER_LINEARIZATION", 16: "MOTOR_DEADTIME_CALIBRATION",
}


def axis_state_name(raw: int) -> str:
    """固件原始 `AxisState` 号 → 名字。**用 SDK 的 `jsdk_can_axis_state_name()`**。

    ⚠⚠ 别用 `jsdk_axis_state_string()` —— 那是 SDK **自己归一化**的枚举
    （`jsdk_axis_state_t`，如 `switch-on-disabled(idle)`），不是固件的原始号。
    实测踩过：拿它去翻译 `axis0.current_state` 的原始值，7 得到
    `unknown-axis-state`、16 得到 `unknown-axis-state` —— **看着像"状态未知"，
    其实是把两种枚举搞混了**。

    权威表是 `src/core/jsdk_fault.c` 的 `k_can_axis_state[17]`
    （0..16，5 空缺，16 = MOTOR_DEADTIME_CALIBRATION）。
    """
    try:
        from jsdk_can import _abi                        # noqa: PLC0415
        fn = getattr(_abi.lib(), "jsdk_can_axis_state_name", None)
        if fn is not None:
            s = fn(int(raw))
            if isinstance(s, (bytes, bytearray)):
                s = s.decode("utf-8", "replace")
            if s and s != "unknown":
                return str(s)
    except Exception:                                      # noqa: BLE001
        pass
    return AXIS_STATES_FALLBACK.get(int(raw), f"unknown({raw})")


@dataclass
class SessionConfig:
    """建会话所需的最小配置。默认值刻意保守（AGENTS.md §2.1）。"""

    iface: str = "slcan"
    channel: str = ""
    node: int = 1
    master_id: int = 1
    bitrate: int = 1_000_000
    #: 0 = 不碰适配器配置（**安全起步**：先按 Classic，让设备告诉我们它是什么）
    data_bitrate: int = 0
    baud: int = 115_200
    #: None = 自动探测（先听 500 ms 再定格式）；True/False = 显式指定
    classic: bool | None = None
    timeout_ms: int = 3_000
    #: ⚠ **状态序列预算**（calibrate / home 等“跑一会”的操作），与 `timeout_ms`
    #: （单次传输超时）**不是同一回事**。
    #:
    #: 把它们混用会出一个很能误导人的错：真机全标定实测 ~30 s，
    #: 若把 3000 ms 的传输超时当预算传进去，会在 3 s 就报
    #: "state 3 did not finish within 3000 ms" —— **看起来像设备不动了**，
    #: 实际只是我们的参数用错了。
    #:
    #: `0` = 用 SDK 内置默认（标定 120 s / 回零 5 s）。
    state_timeout_ms: int = 0
    #: 是否立刻下载描述符（只读的 info/health 不需要，下 38 KB 很贵）
    load_descriptor: bool = True
    #: 仅 virtual：节点规格（None = 内置默认）。用于离线演练多节点/故障场景
    virtual_spec: str = ""


@dataclass
class SessionState:
    """会话的可观测状态 — 每个工具都返回它，让 agent 知道"我在什么处境"。"""

    iface: str = ""
    channel: str = ""
    node: int = 0
    opened: bool = False
    configured: bool = False
    descriptor_loaded: bool = False
    #: 0 未学 / 1 改学 Classic / 2 改学 FD / 3 一致 / 4 显式配置与对端冲突
    framing_learned: int = 0
    #: 首次发帧时实际用的格式（排"到底发出去了没有"的关键字段）
    first_tx_framing: str = ""
    tx_frames: int = 0
    rx_frames: int = 0
    tx_retries: int = 0
    req_timeouts: int = 0
    last_error: str = ""
    notes: list[str] = field(default_factory=list)
    #: 内置路径表里**不在这台设备上**的项（路径表过时的直接证据）
    missing_endpoints: dict[str, str] = field(default_factory=dict)
    #: 其中属“必需”子集的（缺了相关功能就会失败）
    missing_required: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "iface": self.iface,
            "channel": self.channel,
            "node": self.node,
            "opened": self.opened,
            "configured": self.configured,
            "descriptor_loaded": self.descriptor_loaded,
            "framing_learned": self.framing_learned,
            "framing_learned_name": {
                0: "not_learned", 1: "learned_classic", 2: "learned_fd",
                3: "consistent", 4: "explicit_conflict",
            }.get(self.framing_learned, "unknown"),
            "first_tx_framing": self.first_tx_framing,
            "tx_frames": self.tx_frames,
            "rx_frames": self.rx_frames,
            "tx_retries": self.tx_retries,
            "req_timeouts": self.req_timeouts,
            "last_error": self.last_error,
            "notes": list(self.notes),
            "missing_endpoints": dict(self.missing_endpoints),
            "missing_required": list(self.missing_required),
        }


class JointSession:
    """一台关节的一个会话。

    ⚠ **线程安全**：同一 `(node, msgtype)` 的请求必须串行
    （`AGENTS.md` §2.1：响应关联不能靠 Seq，只能靠 source/dest/msgtype）。
    所以整个会话用一把锁，宁可串行也不并发。
    """

    def __init__(self, cfg: SessionConfig) -> None:
        self.cfg = cfg
        self.state = SessionState(
            iface=cfg.iface, channel=cfg.channel, node=cfg.node
        )
        self._lock = threading.RLock()
        self._ctx: Any = None
        #: 缓存的"参数读回值"，避免同一个值反复往返（也用于交叉校验）
        self._param_cache: dict[str, Any] = {}
        #: 已解析的端点路径 → (id, type, access)
        self._endpoints: dict[str, tuple[int, str, int]] = {}

    # ---------------------------------------------------------------- 生命周期

    def open(self) -> SessionState:
        """打开总线。**只读**，不发送任何帧。

        ⚠ 与 `AGENTS.md` §2.1 一致：不指定帧格式时按 Classic 起步，
        而且要"**发任何帧之前先只收不发探测**"。
        """
        from . import paths                            # noqa: PLC0415

        # ⚠ 先把 jsdk_can 解析出来。不这样做的话，用户会看到
        #   `No module named 'jsdk_can'` —— 看起来像 SDK 坏了，
        #   实际只是 mcp/ 与 bindings/python/ 是两个独立的包、路径没配对。
        if not paths.ensure_jsdk_can():
            raise TransportError(
                "无法加载 jsdk_can（Python 绑定）",
                hint=paths.import_error_message(),
                diagnosis=paths.diagnosis(),
            )

        from jsdk_can import Context, Mode          # noqa: PLC0415
        from jsdk_can.hal import (GsusbHal, PcanHal, SlcanHal,  # noqa: PLC0415
                                  SocketCanHal, VirtualHal)

        with self._lock:
            if self.state.opened:
                return self.state

            hal = self._build_hal(
                SlcanHal, SocketCanHal, PcanHal, VirtualHal, GsusbHal
            )

            # ⚠⚠ 帧格式的**两层**归属（之前写错过，这里记清楚）：
            #   - `data_bitrate` → **HAL 层**（打开适配器时就定了；slcan 靠 `Y<n>`）
            #   - `is_fd` / `is_fd_explicit` → **Context 层**（cfg 字段，不是 add_joint 的）
            #   `add_joint()` 只接受 `mode` / `gear_ratio` / `mit_max_*` / `torque_constant`。
            #
            # 语义（真机踩过）：
            #   显式指定 → 不允许被对端对齐（否则调用者看到的 cfg 与实际发出的帧不一致，
            #              而 8 字节参数分块读正是按 is_fd 切片的）
            #   未指定   → 允许对端把我们"教会"（先按 Classic 起步是安全方向）
            explicit = self.cfg.classic is not None
            is_fd: bool | None = (not self.cfg.classic) if explicit else None

            ctx = Context(
                hal,
                master_id=self.cfg.master_id,
                is_fd=is_fd,
                # ⚠ 用**独立的**状态预算（不是传输超时）；0 = SDK 内置默认
                #   （标定 120 s / 回零 5 s）
                state_timeout_ms=self.cfg.state_timeout_ms,
            )
            ctx.add_joint(self.cfg.node, mode=Mode.MIT)
            self._ctx = ctx
            self.state.opened = True
            if explicit:
                self.state.notes.append(
                    "帧格式已显式指定；对端不一致时会报 conflict（不会静默改学）"
                )
            else:
                self.state.notes.append(
                    "帧格式未指定 → 先按 Classic 起步，允许被对端对齐"
                )
            return self.state

    def _build_hal(self, SlcanHal, SocketCanHal, PcanHal, VirtualHal, GsusbHal=None):
        """构造 HAL —— **帧格式在这里定**（不是 add_joint 层）。

        ⚠ `data_bitrate` 的真实语义（按 HAL 不同）：
        - **slcan**：`0` = 不碰适配器配置（**安全起步**）；`5000000` → 适配器发 `Y5`。
          ⚠ 真机踩过：slcan 的数据段速率是在**打开序列**里发的（`C`→`Y<n>`→`O`），
          所以一旦按 FD 打开，之后即使 SDK 改学成 Classic，**帧也发不出去了**
          —— 所以"未指定时先用 0 / Classic 起步"是必须的。
        - **socketcan / pcan**：FD 能力是**打开时**的句柄属性；`0` 表示不开 FD。
        """
        iface = self.cfg.iface
        if iface == "virtual":
            # ⚠ spec=None → 用内置默认规格；`fd`/`classic` 是 spec 里的无值键
            spec = self.cfg.virtual_spec
            return VirtualHal(spec) if spec else VirtualHal()

        if iface == "slcan":
            if not self.cfg.channel:
                raise UsageError(
                    "slcan 需要 channel（如 COM3 / /dev/ttyACM0）",
                    hint="用 list_channels 工具看看系统上有什么",
                )
            kw: dict[str, Any] = {"baud": self.cfg.baud}
            # ⚠ 只有**显式**要 FD 时才给 data_bitrate；否则保持 0（不碰适配器）
            if self.cfg.classic is False:
                kw["data_bitrate"] = self.cfg.data_bitrate or 5_000_000
            return SlcanHal(self.cfg.channel, **kw)

        if iface == "socketcan":
            if not self.cfg.channel:
                raise UsageError("socketcan 需要 channel（如 can0）")
            # socketcan 的 data_bitrate：0 表示不开 FD
            if self.cfg.classic is True:
                return SocketCanHal(self.cfg.channel, bitrate=self.cfg.bitrate,
                                    data_bitrate=0)
            return SocketCanHal(
                self.cfg.channel, bitrate=self.cfg.bitrate,
                data_bitrate=self.cfg.data_bitrate or 5_000_000,
            )

        if iface == "pcan":
            if self.cfg.classic is True:
                return PcanHal(self.cfg.channel or "PCAN_USBBUS1",
                               bitrate=self.cfg.bitrate, data_bitrate=0)
            return PcanHal(self.cfg.channel or "PCAN_USBBUS1",
                           bitrate=self.cfg.bitrate,
                           data_bitrate=self.cfg.data_bitrate or 5_000_000)

        if iface == "gsusb":
            # gs_usb（candleLight）USB-CAN：走 python-can 的 gs_usb 接口。
            # ⚠ 该接口是 CAN 2.0（无 FD），且**需要设备已绑到 WinUSB/libusbK/
            #   libusb-win32**（厂商驱动打不开）；与 usbipd（交给 WSL）互斥。
            if GsusbHal is None:  # 旧库没有这个后端
                raise UsageError(
                    "当前 jsdk_can 没有 GsusbHal（需要带插件式 HAL 的版本）")
            return GsusbHal(self.cfg.channel or "0",
                            bitrate=self.cfg.bitrate or 1_000_000)

        raise UsageError(
            f"未知传输后端: {iface}",
            hint="可选 gsusb / socketcan / pcan / slcan / virtual",
        )

    def configure(self, *, load_descriptor: bool | None = None) -> SessionState:
        """下载描述符并解析端点表。**这是所有按路径访问的前提。**"""
        want = self.cfg.load_descriptor if load_descriptor is None else load_descriptor
        with self._lock:
            self.open()
            if self.state.configured and (self.state.descriptor_loaded or not want):
                return self.state

            try:
                self._ctx.configure()
            except Exception as exc:                       # noqa: BLE001
                self.state.last_error = str(exc)
                raise self._classify(exc, "configure()") from exc

            self.state.configured = True
            if want:
                self._index_endpoints()
                # ⚠ 校验内置路径表 —— 这一步抓到过 2 个写错的路径
                self._validate_ep_paths()
                self.state.descriptor_loaded = True
            self._sync_bus_state()
            return self.state

    def close(self) -> None:
        with self._lock:
            if self._ctx is not None:
                try:
                    self._ctx.close()
                except Exception:                          # noqa: BLE001, S110
                    pass
            self.state.opened = False
            self.state.configured = False
            self.state.descriptor_loaded = False
            self._ctx = None

    # ------------------------------------------------------------ 错误归一化

    def _classify(self, exc: Exception, op: str) -> Exception:
        """把 SDK 异常翻译成**带排查方向**的结构化错误。

        ⚠ 关键是区分"通道不通"与"请求丢了"——这两者的下一步完全不同。
        `AGENTS.md` §3.1 的判据：看 rx 帧数。
        """
        msg = str(exc)
        low = msg.lower()
        got_zero_bytes = "0/0 bytes" in msg
        rx = self.state.rx_frames

        if "timed out" in low or "timeout" in low:
            if got_zero_bytes or rx == 0:
                return TransportError(
                    f"{op} 超时，且**一帧都没收到**",
                    hint=(
                        "0 帧 = 链路/格式问题。先确认端口/权限/波特率，"
                        "再怀疑 Classic/FD 猜错（心跳收得到但请求没人应 = 格式错）。"
                    ),
                    op=op, rx_frames=rx, raw=msg,
                )
            return TransportError(
                f"{op} 超时，但收到了 {rx} 帧",
                hint=(
                    ">0 帧 = **通道通、我们的请求丢了**。这不是线缆问题；"
                    "用 warmup（幂等请求重发）吸收首帧丢失。"
                ),
                op=op, rx_frames=rx, raw=msg,
            )
        if "descriptor" in low or "parse" in low:
            return ProtocolError(
                f"{op} 描述符解析失败",
                hint="若报 total_len must be 1..65535，可能是把数据帧误判成元数据帧",
                op=op, raw=msg,
            )
        return ProtocolError(f"{op} 失败: {msg}", op=op, raw=msg)

    # ---------------------------------------------------------------- 内部工具

    def _index_endpoints(self) -> None:
        """从**设备下载的描述符**建索引。绝不内置静态表。

        ⚠ 真实接口是 `Context.endpoints()`（不是 `enumerate_endpoints()`）——
        这一步靠真机跑出来的 AttributeError 才发现，不是靠记忆。
        """
        eps = self._ctx.endpoints()
        for ep in eps:
            # ⚠ `type_name` 是 **property**（不是方法）—— 靠真机跑出来才发现
            self._endpoints[ep.path] = (
                ep.ep_id, ep.type_name, ep.access,
            )

    def _validate_ep_paths(self) -> None:
        """核对 `EP` 表里每个路径**在这台设备上是否真的存在**。

        ⚠ 为什么需要这一步：路径虽然比 ID 稳定，但**也会变/也会写错**。
        实测踩过：`can.heartbeat_rate_ms` 与 `can.node_id` 都不存在，
        真实路径是 `axis0.config.can.*`。

        如果在调用时才暴露，报的是 `lookup(x): not_found` —— 看起来像
        "设备没有这个参数"，而不是"我们的路径表过时了"，排查方向会跑偏。
        所以在这里一次核完，并把缺失项**记进会话状态**（`missing_endpoints`），
        每个工具返回时都会带上。
        """
        missing = [k for k, p in EP.items() if p not in self._endpoints]
        self.state.missing_endpoints = {k: EP[k] for k in missing}
        required_missing = [k for k in missing if k in EP_REQUIRED]
        self.state.missing_required = required_missing
        if missing:
            self.state.notes.append(
                f"⚠ 有 {len(missing)} 个内置路径不在这台设备上："
                + ", ".join(f"{k}={EP[k]}" for k in missing)
                + "（路径表可能过时；用 get_descriptor 查真实路径）"
            )
        if required_missing:
            self.state.notes.append(
                f"⚠⚠ 其中**必需**路径缺失: {', '.join(required_missing)}"
                " —— 相关功能会失败"
            )

    def _sync_bus_state(self) -> None:
        try:
            bs = self._ctx.bus_state()
            self.state.tx_frames = bs.tx_frames
            self.state.rx_frames = bs.rx_frames
            self.state.tx_retries = bs.tx_retries
            self.state.req_timeouts = bs.req_timeouts
        except Exception:                                  # noqa: BLE001, S110
            pass

    def _joint(self):
        if not self.state.configured:
            raise UsageError(
                "会话尚未 configure（描述符未下载）",
                hint="先调用 connect 工具；只读的 info/health 也需要 configure",
            )
        return self._ctx.joint(self.cfg.node)

    def _ensure_configured(self, *, descriptor: bool = True) -> None:
        """按需完成 configure —— 工具不该要求调用者先手动走一遍。

        ⚠ 设计原因（真机踩到）：`get_state` 是**任何诊断的第一步**
        （`AGENTS.md` §3.1 第 4 步），如果它要求"你先单独 configure"，
        那 agent 会先撞一个 `会话尚未 configure` 的错 —— 而这个错
        **不是设备的问题**，是本 server 的门槛放错了位置。

        所以：需要端点的操作（读参数/状态/写）自动带描述符；
        只要基础配置的操作（如 info）可以传 `descriptor=False`（省 38 KB 下载）。
        """
        self.configure(load_descriptor=descriptor)

    def _lookup(self, path: str) -> tuple[int, str, int]:
        """查端点元数据。**端点 ID 绝不硬编码**（AGENTS.md §2.1）。

        ⚠ 用 `Context.lookup()`（不是 `Joint.endpoint_lookup()`）。
        未命中会抛 `JsdkDeviceNotFoundError`（**不猜、不近似**）—— 这正好，
        因为猜一个端点 ID 比报错危险得多。
        """
        if path in self._endpoints:
            return self._endpoints[path]
        ep = self._ctx.lookup(path)
        entry = (ep.ep_id, ep.type_name, ep.access)   # type_name 是 property
        self._endpoints[path] = entry
        return entry

    # --------------------------------------------------------- 读（安全）

    def read(self, path: str) -> dict[str, Any]:
        with self._lock:
            self._ensure_configured(descriptor=True)
            ep_id, type_name, access = self._lookup(path)
            try:
                value = self._joint().param_get(path)
            except Exception as exc:                       # noqa: BLE001
                raise self._classify(exc, f"read {path}") from exc
            self._param_cache[path] = value
            self._sync_bus_state()
            return {
                "ok": True,
                "path": path,
                "id": ep_id,
                "type": type_name,
                "access": _access_str(access),
                "value": _jsonable(value),
            }

    def read_many(self, paths: list[str]) -> dict[str, Any]:
        """批量读（FD 单帧；Classic 自动退化）。

        ⚠ 用于**交叉校验**：拿 `read` 的结果和 `read_many` 的对拍
        （`AGENTS.md` §1.5：数值要交叉校验）。
        """
        with self._lock:
            self._ensure_configured(descriptor=True)
            try:
                got = self._joint().param_get_batch(*paths)
            except Exception as exc:                       # noqa: BLE001
                raise self._classify(exc, "read_many") from exc
            self._sync_bus_state()
        return {
            "ok": True,
            "values": {k: _jsonable(v) for k, v in got.items()},
            "count": len(got),
        }

    # --------------------------------------------------------- 写（**危险**）

    def write(self, path: str, value: str) -> dict[str, Any]:
        """写参数 + **强制读回**。

        ⚠⚠ 这是本 server 最需要小心的工具。三条规矩：
        1. 先查描述符拿**类型/宽度**，按真实位宽装箱，超值域直接拒（不静默截断）
        2. 写完**必须读回**，`requested` / `value` / `verified` 分字段返回
        3. "写进去就被状态机消费"的端点（`requested_state`）不谎称已验证
        """
        with self._lock:
            self._ensure_configured(descriptor=True)
            ep_id, type_name, access = self._lookup(path)
            if not (access & 0x02):   # JSDK_EP_ACCESS_W
                raise UsageError(
                    f"{path} 不可写（access={_access_str(access)}）",
                    hint="用 get_descriptor 工具查这个路径的权限",
                    path=path, access=_access_str(access),
                )

            # ---- 按端点**声明类型**装箱（不是按 Python 类型猜） ----
            try:
                boxed = _box_value(value, type_name)
            except ValueError as exc:
                raise UsageError(
                    f"值 {value!r} 不是合法的 {type_name}",
                    hint=f"该端点类型是 {type_name}；超值域不会被截断",
                    path=path, type=type_name, raw=str(exc),
                ) from exc

            joint = self._joint()
            try:
                if hasattr(joint, "param_set_auto"):
                    joint.param_set_auto(path, boxed)
                else:                                      # pragma: no cover
                    joint.param_set(path, boxed)
            except Exception as exc:                       # noqa: BLE001
                raise self._classify(exc, f"write {path}") from exc

            self._sync_bus_state()

            # ---- 消费型端点：读回必然不等，**不谎称已验证** ----
            if path == EP["requested_state"]:
                return {
                    "ok": True,
                    "path": path,
                    "id": ep_id,
                    "type": type_name,
                    "requested": _jsonable(boxed),
                    "value": None,
                    "verified": None,
                    "note": (
                        "该端点写进去就被状态机立刻消费回 0（固件行为），"
                        "读回值无法判生效。要看结果用 get_state / calibrate_and_wait。"
                    ),
                }

            # ---- 一般端点：读回 ----
            try:
                back = joint.param_get(path)
                have = True
            except Exception:                              # noqa: BLE001
                back, have = None, False

            same = have and _values_equal(boxed, back)
            result: dict[str, Any] = {
                "ok": True,
                "path": path,
                "id": ep_id,
                "type": type_name,
                "requested": _jsonable(boxed),
                "value": _jsonable(back) if have else None,
                "verified": same,
            }
            if not have:
                result["note"] = (
                    "写入已发出，但**读不回来**，无法确认生效。"
                    "若改的是 node_id 类参数，这是正常的（寻址变了）。"
                )
            elif not same:
                # ⚠ **不抛异常**：MCP 层要把"写丢了"如实返回给 agent，
                #    让它自己决定怎么办。但 ok 字段要能区分。
                result["ok"] = False
                result["error"] = "not_accepted"
                result["hint"] = (
                    "先数写帧长度：值宽 <4 的端点（bool/u8/u16）若未补齐到 >=8 字节，"
                    "会被固件 `if (msg.len < 8) return;` 整帧静默丢弃。"
                )
            self._param_cache[path] = back if have else None
            return result

    # --------------------------------------------------------- 状态 / 诊断

    def get_state(self) -> dict[str, Any]:
        """读状态：**原始错误位图**，不是 `err_name`（AGENTS.md §1.4）。"""
        with self._lock:
            # ⚠ 这是诊断的第一步，必须自己能跑起来（不能要求调用者先 configure）
            self._ensure_configured(descriptor=True)
            joint = self._joint()
            out: dict[str, Any] = {"ok": True}

            for key in ("error_raw", "motor_error", "encoder_error",
                        "controller_error", "current_state"):
                path = EP[key]
                try:
                    val = joint.param_get(path)
                    out[key] = _jsonable(val)
                    if key == "current_state":
                        out["current_state_name"] = axis_state_name(int(val))
                        out["resting"] = int(val) in RESTING_STATES
                    if key.endswith("error"):
                        out[key + "_hex"] = (
                            f"0x{int(val):X}" if isinstance(val, int) else None
                        )
                except Exception as exc:                   # noqa: BLE001
                    out[key] = None
                    out.setdefault("_read_errors", {})[path] = str(exc)

            # ⚠ 告诉 agent 该怎么读这个错误，而不是替它下结论
            out["how_to_read"] = (
                "err_name 会把看门狗超时与急停混成同一个名字；"
                "要查具体来源请用 fault_info 工具看子系统明细。"
            )
            self._sync_bus_state()
            out["session"] = self.state.to_dict()
            return out

    def fault_info(self) -> dict[str, Any]:
        """六类错误明细分项（子系统位图）。"""
        with self._lock:
            self._ensure_configured(descriptor=True)
            joint = self._joint()
            try:
                fi = joint.fault_info()
            except Exception as exc:                       # noqa: BLE001
                raise self._classify(exc, "fault_info") from exc
            return {
                "ok": True,
                "mit_error_name": getattr(fi, "mit_name", "") or "",
                "subsystems": {
                    k: {"value": v, "hex": f"0x{v:X}"}
                    for k, v in (fi.subsystems() or {}).items()
                },
            }

    def health(self) -> dict[str, Any]:
        with self._lock:
            self.configure(load_descriptor=False)
            joint = self._joint()
            out: dict[str, Any] = {"ok": True}
            for attr in ("feedback",):
                if hasattr(joint, attr):
                    try:
                        fb = getattr(joint, attr)()
                        out[attr] = _jsonable(fb)
                    except Exception as exc:               # noqa: BLE001
                        out[attr] = None
                        out.setdefault("_read_errors", {})[attr] = str(exc)
            self._sync_bus_state()
            out["session"] = self.state.to_dict()
            return out

    def device_info(self) -> dict[str, Any]:
        """⚠ 只需要基础配置（**不下载描述符** —— 省 38 KB / 3.5 s）。"""
        with self._lock:
            self.configure(load_descriptor=False)
            try:
                info = self._joint().device_info()
            except Exception as exc:                       # noqa: BLE001
                raise self._classify(exc, "device_info") from exc
            return {"ok": True, "info": _jsonable(info)}

    def descriptor(self, path: str | None = None,
                   filter_substr: str | None = None,
                   limit: int = 200) -> dict[str, Any]:
        """查端点表。**端点 ID 只能从这里来，绝不硬编码。**"""
        with self._lock:
            self._ensure_configured(descriptor=True)
            items = []
            for p, (ep_id, type_name, access) in sorted(self._endpoints.items()):
                if path and p != path:
                    continue
                if filter_substr and filter_substr not in p:
                    continue
                items.append({
                    "path": p, "id": ep_id, "type": type_name,
                    "access": _access_str(access),
                    "writable": bool(access & 0x02),
                })
                if len(items) >= limit:
                    break
            return {
                "ok": True,
                "count": len(items),
                "truncated": len(items) >= limit,
                "items": items,
                "total_indexed": len(self._endpoints),
            }

    # --------------------------------------------------------- 运动（最高风险）

    def set_zero(self, *, confirm: bool = False) -> dict[str, Any]:
        self._require_confirm(confirm, "set-zero（会改写位置零点）")
        with self._lock:
            self._ensure_configured(descriptor=True)
            try:
                self._joint().set_zero_here()
            except Exception as exc:                       # noqa: BLE001
                raise self._classify(exc, "set_zero") from exc
            return {"ok": True, "action": "set_zero"}

    def fault_reset(self, *, confirm: bool = False) -> dict[str, Any]:
        self._require_confirm(confirm, "fault-reset")
        with self._lock:
            self._ensure_configured(descriptor=True)
            try:
                # ⚠ 用阻塞版：`CLEAR_ERRORS(0x65)` 清不掉某些锁存位（真机
                #   `ERROR_CAN_BUS_FAILED`），而 `feedback().axis_error` 是 0x45
                #   快照、不刷新 —— 只“请求”会让工具一直报“仍有故障”。
                j = self._joint()
                fn = getattr(j, "fault_reset_wait", None)
                if fn is not None:
                    fn(0)
                else:                                   # 旧库没这个 API
                    j.fault_reset()
            except Exception as exc:                       # noqa: BLE001
                raise self._classify(exc, "fault_reset") from exc
            return self.get_state()

    # ------------------------------------------------- 标定 / 回零（会转电机）

    def calibrate(self, *, confirm: bool = False) -> dict[str, Any]:
        """全标定（写 `requested_state=3` 并等序列跑完）。**电机会动。**
        ⚠⚠ 本方法**不重新实现轮询** —— 那是 C 侧 `request_state_and_wait()` 的职责，
        且它已满足 `AGENTS.md` §3.3 的全部四条约束：

        | 约束 | C 侧实现 |
        |---|---|
        | 限速轮询 | 首段 20 ms（不漏瞬时态）→ 确认开始后转 200 ms |
        | 判据 = 离开静息态 | `state_is_busy()` 置 `seen_transient` |
        | 空闲只收不发 | 直接 `hal.recv()`，**不**调 `context.poll()`（那会补发控制帧） |
        | 超时来自时钟 | `jsdk_elapsed(now, t0) >= timeout_ms` |

        在 MCP 层重写一遍 = 造出第二套实现（违反 §5）。
        这里的职责是**如实转译语义**，并补上 C 侧不做的那一步：
        结束时按 §3.3 第 5 条读回两个 `pre_calibrated`。
        """
        self._require_confirm(confirm, "calibrate（会转电机，十几圈电气角）")
        with self._lock:
            self._ensure_configured(descriptor=True)
            t0 = time.monotonic()
            try:
                self._joint().calibrate()
            except Exception as exc:                       # noqa: BLE001
                return self._state_op_result(
                    "calibrate", ok=False, t0=t0, error=str(exc),
                    extra={"error_class": self._classify(exc, "calibrate").code},
                )
            return self._state_op_result("calibrate", ok=True, t0=t0)

    def home(self, *, confirm: bool = False) -> dict[str, Any]:
        """回零（写 `requested_state=11` 并等）。**电机会动。**"""
        self._require_confirm(confirm, "home（会转电机寻零）")
        with self._lock:
            self._ensure_configured(descriptor=True)
            t0 = time.monotonic()
            try:
                self._joint().home()
            except Exception as exc:                       # noqa: BLE001
                return self._state_op_result(
                    "home", ok=False, t0=t0, error=str(exc),
                    extra={"error_class": self._classify(exc, "home").code},
                )
            return self._state_op_result("home", ok=True, t0=t0)

    def _state_op_result(self, action: str, *, ok: bool, t0: float,
                         error: str = "",
                         extra: dict[str, Any] | None = None) -> dict[str, Any]:
        """标定/回零结束后的收尾（`AGENTS.md` §3.3 第 5 条）。

        ⚠ 三件事缺一个就会"谎称成功"：
        1. 读**当前状态**（是否回到静息态）
        2. 读**错误明细**（"回到静息"≠成功 —— 失败路径正是"进子态→出错→回静息"）
        3. 读**两个 `pre_calibrated`**（标定跑完 ≠ 标定生效）
        """
        out: dict[str, Any] = {
            "ok": ok,
            "action": action,
            "elapsed_s": round(time.monotonic() - t0, 3),
        }
        if error:
            out["error"] = "STATE_OP_FAILED"
            out["message"] = error
            out["hint"] = (
                "若提示 'never left idle' / 'requires the axis to be disabled'：\n"
                "  - 'never left idle' 最常见的是**锁存故障**让状态机拒绝启动"
                "（写被接受、值也对，但状态就是不动）→ 先 fault_reset\n"
                "  - 标定要求轴处于 IDLE（未使能）；使能中调用与闭环控制打架"
            )
            if extra:
                out.update(extra)

        out["state"] = self._read_state_quiet()
        out["fault"] = self._read_fault_quiet()

        # ---- 防线：C 侧说“跑完”不代表轴真的停了 ----
        # ⚠ 真机踩过：`state_is_busy()` 曾经漏了 15/16，于是全标定走到
        #   `MOTOR_DEADTIME_CALIBRATION(16)` 时被当成“已回静息” ⇒ calibrate
        #   在**电机仍在转**的时候就返回 OK。SDK 侧已修，这里仍保留一道
        #   独立检查：不信单点结论，要看轴**现在**在哪个状态。
        st = out["state"]
        if isinstance(st, dict) and st.get("ok"):
            if not st.get("resting"):
                out["ok"] = False
                out.setdefault("warnings", []).append(
                    f"序列返回时轴**不在静息态**（current_state="
                    f"{st.get('current_state')}/{st.get('current_state_name')}）"
                    " —— 电机可能仍在动作，不要立即下发其他指令。"
                )

        if action == "calibrate":
            flags: dict[str, Any] = {}
            for key in ("pre_cal_motor", "pre_cal_encoder"):
                try:
                    flags[EP[key]] = _jsonable(self._joint().param_get(EP[key]))
                except Exception as exc:                   # noqa: BLE001
                    flags[EP[key]] = None
                    flags.setdefault("_read_errors", {})[EP[key]] = str(exc)
            out["pre_calibrated"] = flags
            both = (flags.get(EP["pre_cal_motor"]) is True
                    and flags.get(EP["pre_cal_encoder"]) is True)
            out["pre_calibrated_both"] = both
            if ok and not both:
                # ⚠ **正常行为，不是缺陷** —— 必须说清，否则用户会一直重试
                out["note"] = (
                    "标定序列跑完 ≠ 标定生效。`pre_calibrated`（电机与编码器**两个都要**）"
                    "不会自动变 true，必须人工写入 + save 到 Flash 才持久。要生效请依次执行：\n"
                    "  write axis0.motor.config.pre_calibrated 1\n"
                    "  write axis0.encoder.config.pre_calibrated 1\n"
                    "  save"
                )

        # ---- "跑完了" ≠ "成功了" ----
        subs = (out.get("fault") or {}).get("subsystems") or {}
        nonzero = {k: v for k, v in subs.items()
                   if isinstance(v, dict) and v.get("value")}
        if nonzero:
            out["ok"] = False
            out["error"] = "DEVICE_FAULT"
            out["hint"] = (
                "状态序列跑完，但设备报了故障位。按 AGENTS.md，"
                "'回到静息态'不等于成功 —— 失败路径正是'进子态→出错→回静息'。"
            )
            out["active_faults"] = nonzero
        return out

    def _read_state_quiet(self) -> dict[str, Any]:
        try:
            return self.get_state()
        except Exception as exc:                           # noqa: BLE001
            return {"ok": False, "error": "READ_FAILED", "message": str(exc)}

    def _read_fault_quiet(self) -> dict[str, Any]:
        try:
            return self.fault_info()
        except Exception as exc:                           # noqa: BLE001
            return {"ok": False, "error": "READ_FAILED", "message": str(exc)}

    # ---------------------------------------------------------------- 遥测

    def telemetry(self, duration_ms: int = 1000, rate_hz: int = 10) -> dict[str, Any]:
        """抓一段时间的反馈样本（**只读**）。"""
        if duration_ms <= 0:
            raise UsageError("duration_ms 必须 > 0")
        rate_hz = max(1, min(200, rate_hz))
        interval = 1.0 / rate_hz

        with self._lock:
            self._ensure_configured(descriptor=True)
            joint = self._joint()
            samples: list[dict[str, Any]] = []
            t0 = time.monotonic()
            deadline = t0 + duration_ms / 1000.0

            while True:
                # 必须泵周期，否则反馈不更新（阻塞 API 在冻结时钟上永远完不成）
                try:
                    self._ctx.cycle_begin(0)
                    self._ctx.cycle_end()
                except Exception:                          # noqa: BLE001, S110
                    pass

                now = time.monotonic()
                if now >= deadline:
                    break
                try:
                    fb = joint.feedback()
                    samples.append({
                        "t_ms": round((now - t0) * 1000.0, 2),
                        "pos": fb.pos,
                        "vel": fb.vel,
                        "torque_Nm": fb.torque_Nm,
                        "current_A": fb.current_A,
                        "t_motor_C": fb.t_motor_C,
                        "t_fet_C": fb.t_fet_C,
                        "vbus_V": fb.vbus_V,
                        # ⚠ 判新鲜度用 age_ms，**不要**用 valid（每周期旗标，约 50% 为真）
                        "age_ms": fb.age_ms,
                        "online": bool(fb.online),
                        "err_code": int(fb.err_code),
                        "axis_error": int(fb.axis_error),
                        "axis_state": int(fb.axis_state),
                        "status_flags": int(fb.status_flags),
                    })
                except Exception as exc:                   # noqa: BLE001
                    samples.append({
                        "t_ms": round((now - t0) * 1000.0, 2),
                        "error": str(exc),
                    })
                sleep = interval - (time.monotonic() - now)
                if sleep > 0:
                    time.sleep(sleep)

            self._sync_bus_state()

        return {
            "ok": True,
            "duration_ms": duration_ms,
            "rate_hz": rate_hz,
            "count": len(samples),
            "samples": samples,
            "summary": _summarize(samples),
            "session": self.state.to_dict(),
        }

    # ---------------------------------------------------------------- 探针

    def probe_ranges(self) -> dict[str, Any]:
        """读**量程与换常量** —— 写探针前必须先调用。

        ⚠ `AGENTS.md` §2.2：这些值**都会变**（不同关节、同关节不同固件版本、
        有的每次测量都不同）。实测教训：
        - `torque_constant` 在同一台关节上从 `0.0824464` 变到 `0.0864797`
        - `mit_max_torque` 曾被当成 18（实际 **50**）⇒ 打包 MIT 帧实发值差 2.78 倍

        ⚠ **越界不钳位，是回绕**：超量程的值绕到反方向（+12.6 rad → −12.4 rad）
        ⇒ 满力矩反向。所以量程只能读，不能按经验值打包。
        """
        with self._lock:
            self._ensure_configured(descriptor=True)
            out: dict[str, Any] = {"ok": True, "ranges": {}, "source": "device"}

            # 优先用 config_snapshot（一次读回全部量程）
            try:
                snap = self._joint().config_snapshot()
                out["ranges"] = snap.as_dict()
                out["snapshot_valid"] = bool(snap.valid)
                if not snap.valid:
                    out["note"] = (
                        "config_snapshot.valid=false ⇒ 里面的数值**不可用**，"
                        "不要当 0 用。回退到逐项读端点。"
                    )
            except Exception as exc:                       # noqa: BLE001
                out["_snapshot_error"] = str(exc)

            # 逐项读端点交叉校验（§1.5：真机结论要交叉校验）
            cross: dict[str, Any] = {}
            for key in ("gear_ratio", "torque_constant"):
                try:
                    cross[EP[key]] = _jsonable(self._joint().param_get(EP[key]))
                except Exception as exc:                   # noqa: BLE001
                    cross[EP[key]] = None
                    cross.setdefault("_read_errors", {})[EP[key]] = str(exc)
            out["cross_check"] = cross

            out["how_to_use"] = (
                "写 MIT 帧/探针前用这里读到的量程打包。"
                "MIT 的 pos/vel/tau 是对称量程 ±mit_max_*；kp/kd 是无符号 [0, mit_max_*]。"
                "越过量程**不钳位而是回绕**（会反向满力矩）。"
            )
            self._sync_bus_state()
            out["session"] = self.state.to_dict()
            return out

    def _require_confirm(self, confirm: bool, what: str) -> None:
        """⚠ 安全闸 —— 会改设备状态/驱动电机的操作必须显式确认。

        宁可拒绝执行，也不"猜一下试试"（AGENTS.md §5）。
        """
        if not confirm:
            raise SafetyRefused(
                f"{what} 需要显式确认",
                hint="这是会改变设备状态的操作，调用时请传 confirm=true",
                action=what,
            )


# --------------------------------------------------------------------------
# 会话池
# --------------------------------------------------------------------------

class SessionManager:
    """管理多个会话（多台关节 / 多条总线）。

    ⚠ 多会话共用一个接口时**必须给不同的 master_id**
    （协议按 SRC/DEST/MsgType 关联请求响应）。
    """

    def __init__(self) -> None:
        self._sessions: dict[str, JointSession] = {}
        self._lock = threading.RLock()

    @staticmethod
    def key(cfg: SessionConfig) -> str:
        return f"{cfg.iface}:{cfg.channel}:{cfg.node}"

    def get(self, cfg: SessionConfig) -> JointSession:
        with self._lock:
            k = self.key(cfg)
            if k not in self._sessions:
                self._sessions[k] = JointSession(cfg)
            return self._sessions[k]

    def require(self, cfg: SessionConfig) -> JointSession:
        with self._lock:
            s = self._sessions.get(self.key(cfg))
        if s is None:
            raise JointNotFound(
                f"没有到 {self.key(cfg)} 的会话",
                hint="先调用 connect 工具建立会话",
                key=self.key(cfg),
            )
        return s

    def drop(self, cfg: SessionConfig) -> bool:
        with self._lock:
            s = self._sessions.pop(self.key(cfg), None)
        if s is not None:
            s.close()
            return True
        return False

    def list(self) -> list[dict[str, Any]]:
        with self._lock:
            return [s.state.to_dict() for s in self._sessions.values()]

    def close_all(self) -> None:
        with self._lock:
            for s in self._sessions.values():
                s.close()
            self._sessions.clear()


# --------------------------------------------------------------------------
# 装箱 / 序列化辅助
# --------------------------------------------------------------------------

#: 端点类型名 → (Python 转换, 值域) —— 与描述符的 type 全集对齐
def _box_value(text: str, type_name: str) -> Any:
    """按**端点声明类型**装箱文本值。

    ⚠ 关键：按类型宽度装箱，而不是按 Python 类型猜（真机踩过：
    `param_set(path, 250)` 按普通 int 猜成 u32 → u16 端点被拒）。
    超值域**直接报错，不静默截断**。
    """
    t = type_name.lower()
    s = text.strip()
    if t == "bool":
        if s.lower() in ("1", "true", "yes", "on"):
            return True
        if s.lower() in ("0", "false", "no", "off"):
            return False
        raise ValueError(f"不是布尔字面量: {text!r}")

    if t in ("float", "float32"):
        try:
            return float(s)
        except ValueError as exc:
            raise ValueError(f"不是浮点数: {text!r}") from exc

    if t.startswith("uint"):
        bits = _bits(t, 32)
        v = int(s, 0)
        if not (0 <= v < (1 << bits)):
            raise ValueError(f"{v} 超出 {t} 值域 [0, {1 << bits})")
        return v

    if t.startswith("int"):
        bits = _bits(t, 32)
        v = int(s, 0)
        lo, hi = -(1 << (bits - 1)), (1 << (bits - 1))
        if not (lo <= v < hi):
            raise ValueError(f"{v} 超出 {t} 值域 [{lo}, {hi})")
        return v

    raise ValueError(f"不支持的端点类型: {type_name}")


def _bits(type_name: str, default: int) -> int:
    for n in (64, 32, 16, 8):
        if type_name.endswith(str(n)):
            return n
    return default


def _access_str(access: int) -> str:
    return ("r" if access & 0x01 else "-") + ("w" if access & 0x02 else "-")


def _summarize(samples: list[dict[str, Any]]) -> dict[str, Any]:
    """把样本序列压成"哪里不正常"的判据。

    ⚠ 给 agent 的是**可核查的统计**，不是结论：
    它拿到 min/max/漂移/丢样次数后自己判断，而不是听我们说"看起来正常"。
    """
    good = [s for s in samples if "error" not in s]
    if not good:
        return {
            "usable_samples": 0,
            "errors": len(samples) - len(good),
            "note": "**一个可用样本都没有** —— 先查链路，不要解读数值。",
        }

    out: dict[str, Any] = {
        "usable_samples": len(good),
        "errors": len(samples) - len(good),
    }

    # ⚠ 新鲜度必须用 age_ms 判（valid 是每周期旗标，约 50% 为真）
    ages = [s["age_ms"] for s in good if isinstance(s.get("age_ms"), int)]
    if ages:
        out["age_ms"] = {
            "min": min(ages), "max": max(ages),
            "stale_count": sum(1 for a in ages if a > 500),
            "note": "age_ms 远大于预期上报周期 ⇒ 反馈不新鲜（先看链路，别解读数值）",
        }

    for field in ("pos", "vel", "torque_Nm", "current_A",
                  "t_motor_C", "t_fet_C", "vbus_V"):
        vals = [s[field] for s in good if isinstance(s.get(field), (int, float))]
        if vals:
            out[field] = {
                "min": min(vals), "max": max(vals),
                "span": max(vals) - min(vals),
                "first": vals[0], "last": vals[-1],
            }

    # 事故信号：任何非零错误位都要浮出来
    errs = sorted({s["axis_error"] for s in good
                   if isinstance(s.get("axis_error"), int) and s["axis_error"]})
    if errs:
        out["axis_error_values"] = [f"0x{v:X}" for v in errs]
        out["axis_error_note"] = "出现过非零错误位 —— 用 get_fault_detail 查分项"
    return out


def _values_equal(a: Any, b: Any) -> bool:
    """按端点类型的容差比较（float32 往返不可能精确相等）。"""
    if isinstance(a, float) or isinstance(b, float):
        try:
            fa, fb = float(a), float(b)
        except (TypeError, ValueError):
            return False
        scale = max(1.0, abs(fa), abs(fb))
        return abs(fa - fb) <= 1e-6 * scale
    return a == b


def _jsonable(v: Any) -> Any:
    """把 SDK 对象转成可 JSON 序列化的**结构化**形式。

    ⚠ 关键：优先取对象的 `as_dict()`。
    有些 SDK 数据类（如 `DeviceInfo`）**只有 dataclass 的 `repr`**，
    没有 `as_dict()` —— 直接 `str(v)` 会得到
    `"DeviceInfo(hw_version=65539, ...)"` 这种**人话**，
    违反 `AGENTS.md` §5（"不要把'人话'当作机器可解析的输出"）。

    所以这里对 dataclass 走 `dataclasses.asdict()` 兜底。
    """
    if v is None or isinstance(v, (bool, int, float, str)):
        return v
    if isinstance(v, (bytes, bytearray)):
        return {"hex": bytes(v).hex(), "len": len(v)}

    # 1) 首选显式的 as_dict()
    if hasattr(v, "as_dict"):
        try:
            return _jsonable(v.as_dict())
        except Exception:                                  # noqa: BLE001
            pass

    # 2) dataclass → 结构化字段（**不是** repr 字符串）
    if dataclasses_is_dataclass(v):
        try:
            return {
                f.name: _jsonable(getattr(v, f.name))
                for f in dataclasses_fields(v)
                if not f.name.startswith("_")
            }
        except Exception:                                  # noqa: BLE001
            pass

    # 3) __dict__ 兜底（普通对象）
    if hasattr(v, "__dict__") and v.__dict__:
        try:
            return {
                k: _jsonable(x) for k, x in vars(v).items()
                if not k.startswith("_")
            }
        except Exception:                                  # noqa: BLE001
            pass

    if isinstance(v, dict):
        return {k: _jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]

    # 最后才退回 str（并标明这是"只能给字符串"的情况）
    return {"repr": str(v), "_note": "该类型没有结构化表示，只能给 repr"}
