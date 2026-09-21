"""``Joint``：单个关节的 Python 封装。

约定
----

* **控制函数不返回错误**（与 C 侧一致）：目标值先缓存，越界/非法在周期结束时按
  §6.10 策略处理（默认"拒绝并改发安全帧"），事后通过
  ``feedback.status_flags`` / ``feedback.tx_rejected`` 查。
  这是 RT 路径的设计，Python 侧不额外包装成异常 —— 否则每个周期都可能抛异常，
  反而掩盖真正的链路故障。
* **配置/运维函数返回即抛异常**（阻塞、非 RT）。
* 单位全是**输出端物理量**：rad / rad/s / N·m。
"""

from __future__ import annotations

import ctypes
import struct
import time
from dataclasses import dataclass

from . import _abi
from .enums import AxisState, EpType, Mode, ModeState, Status, StatusFlag
from .errors import raise_for_status

__all__ = ["Joint", "Feedback", "ConfigSnapshot", "DeviceInfo", "FaultInfo",
           "Sdo"]


def _c_str(value: object) -> str:
    """把 C 侧返回的 `const char*` 变成 `str`（None/空都当空串）。"""
    if not value:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", "replace")
    return str(value)


@dataclass
class Feedback:
    """``jsdk_joint_feedback_t`` 的 Python 视图。"""

    pos: float = 0.0
    vel: float = 0.0
    current_A: float = 0.0
    torque_Nm: float = 0.0
    t_motor_C: float = 0.0
    t_fet_C: float = 0.0
    vbus_V: float = 0.0
    ibus_A: float = 0.0
    axis_state: AxisState = AxisState.UNKNOWN
    mode: Mode | int = Mode.MIT
    mode_state: ModeState | int = ModeState.RESET
    err_code: int = 0
    hb_error: int = 0
    axis_error: int = 0
    age_ms: int = 0
    tx_rejected: int = 0
    tx_frames: int = 0
    status_flags: int = 0
    online: bool = False
    valid: bool = False

    @classmethod
    def _from_c(cls, c: _abi.JointFeedback) -> Feedback:
        def _enum(typ, value):
            try:
                return typ(value)
            except ValueError:
                return value

        return cls(
            pos=c.pos, vel=c.vel, current_A=c.current_A, torque_Nm=c.torque_Nm,
            t_motor_C=c.t_motor_C, t_fet_C=c.t_fet_C,
            vbus_V=c.vbus_V, ibus_A=c.ibus_A,
            axis_state=_enum(AxisState, c.axis_state),
            mode=_enum(Mode, c.mode),
            mode_state=_enum(ModeState, c.mode_state),
            err_code=c.err_code, hb_error=c.hb_error, axis_error=c.axis_error,
            age_ms=c.age_ms, tx_rejected=c.tx_rejected, tx_frames=c.tx_frames,
            status_flags=c.status_flags,
            online=bool(c.online), valid=bool(c.valid),
        )

    @property
    def has_flag(self) -> "StatusFlagQuery":
        """``fb.has_flag(StatusFlag.FEEDBACK_STALE)``。"""
        return StatusFlagQuery(self.status_flags)

    def axis_state_name(self) -> str:
        """字面名字，**取自 C 侧的 `jsdk_axis_state_string()`**。

        ⚠ 别用 ``axis_state.name`` 代替：C 版 CLI 打的是这个函数的文本
        （如 ``switch-on-disabled(idle)``），两版输出要能逐字节对拍。
        """
        raw = self.axis_state
        num = int(raw) if isinstance(raw, int) else -1
        return _c_str(_abi.lib().jsdk_axis_state_string(num))

    def mode_name(self) -> str:
        """同 :meth:`axis_state_name`，取自 `jsdk_mode_string()`（如 ``mit(0x00)``）。"""
        raw = self.mode
        num = int(raw) if isinstance(raw, int) else -1
        return _c_str(_abi.lib().jsdk_mode_string(num))


@dataclass
class StatusFlagQuery:
    """粘滞标志的查询辅助（``if fb.has_flag(FEEDBACK_STALE): ...``）。"""

    raw: int

    def __call__(self, flag: StatusFlag) -> bool:
        return bool(self.raw & int(flag))

    def names(self) -> list[str]:
        return [f.name for f in StatusFlag
                if f is not StatusFlag.NONE and (self.raw & int(f))]

    def __repr__(self) -> str:  # pragma: no cover - 人读
        return f"StatusFlags(0x{self.raw:04X}: {', '.join(self.names()) or '-'})"


@dataclass
class ConfigSnapshot:
    """``jsdk_joint_config_snapshot_t``。``valid=False`` 时数值**不可用**。"""

    gear_ratio: float = 0.0
    mit_max_pos: float = 0.0
    mit_max_vel: float = 0.0
    mit_max_torque: float = 0.0
    mit_max_kp: float = 0.0
    mit_max_kd: float = 0.0
    torque_constant: float = 0.0
    node_id: int = 0
    heartbeat_rate_ms: int = 0
    break_timeout_ms: int = 0
    valid: bool = False

    @classmethod
    def _from_c(cls, c: _abi.JointConfigSnapshot) -> ConfigSnapshot:
        return cls(
            gear_ratio=c.gear_ratio, mit_max_pos=c.mit_max_pos,
            mit_max_vel=c.mit_max_vel, mit_max_torque=c.mit_max_torque,
            mit_max_kp=c.mit_max_kp, mit_max_kd=c.mit_max_kd,
            torque_constant=c.torque_constant, node_id=c.node_id,
            heartbeat_rate_ms=c.heartbeat_rate_ms,
            break_timeout_ms=c.break_timeout_ms, valid=bool(c.valid),
        )

    def as_dict(self) -> dict:
        return {
            "valid": self.valid,
            "gear_ratio": self.gear_ratio,
            "mit_max_pos": self.mit_max_pos,
            "mit_max_vel": self.mit_max_vel,
            "mit_max_torque": self.mit_max_torque,
            "mit_max_kp": self.mit_max_kp,
            "mit_max_kd": self.mit_max_kd,
            "torque_constant": self.torque_constant,
            "node_id": self.node_id,
            "heartbeat_rate_ms": self.heartbeat_rate_ms,
            "break_timeout_ms": self.break_timeout_ms,
        }


@dataclass
class DeviceInfo:
    """``jsdk_device_info_t``。``serial`` 仅 CAN FD 返回（Classic 为 0）。"""

    hw_version: int = 0
    fw_version: int = 0
    serial: int = 0
    classic: bool = False

    @classmethod
    def _from_c(cls, c: _abi.DeviceInfo) -> DeviceInfo:
        return cls(hw_version=c.hw_version, fw_version=c.fw_version,
                   serial=c.serial, classic=bool(c.classic))


@dataclass
class FaultInfo:
    """``jsdk_fault_info_t``。

    ⚠ 三套错误编码**语义层级不同**，不要互相比对：
    ``mit_err`` 是 4-bit 摘要、``hb_flags`` 是 5-bit 子系统位图、
    ``*_error`` 是 32-bit 明细位图。
    """

    valid: bool = False
    mit_err: int = 0
    hb_flags: int = 0
    motor_error: int = 0
    encoder_error: int = 0
    sensorless_error: int = 0
    controller_error: int = 0
    system_error: int = 0
    axis_error: int = 0
    #: ``mit_err`` 的名字（由 C 侧错误表给出）
    mit_err_name: str = ""

    @classmethod
    def _from_c(cls, c: _abi.FaultInfo, mit_name: str = "") -> FaultInfo:
        return cls(valid=bool(c.valid), mit_err=c.mit_err, hb_flags=c.hb_flags,
                   motor_error=c.motor_error, encoder_error=c.encoder_error,
                   sensorless_error=c.sensorless_error,
                   controller_error=c.controller_error,
                   system_error=c.system_error, axis_error=c.axis_error,
                   mit_err_name=mit_name)

    def subsystems(self) -> dict[str, int]:
        return {
            "motor": self.motor_error,
            "encoder": self.encoder_error,
            "sensorless": self.sensorless_error,
            "controller": self.controller_error,
            "system": self.system_error,
            "axis": self.axis_error,
        }


class Sdo:
    """SDO 风格端点槽位：**裸字节** + 非阻塞状态机。

    与 `jsdk-cli` 的 ``read``/``write`` 等价，但拿到的是**线上字节**：

    * ``data`` 就是线上字节，参数值是**小端**（`PROTOCOL_NOTES` §3.1）；
    * 长度恒等于端点**真实类型宽度**（``size``），不需要也不允许自己凑；
    * ``read()`` / ``write()`` 是**阻塞便利**（内部自己跑控制周期直到状态机离开
      ``busy``）。控制回路里请用 :meth:`start_read` / :attr:`state` 自己轮询；
    * 想按物理量读写就别用 SDO：用 :meth:`Joint.param_get` / :meth:`Joint.param_set`
      （内部做小端编解码）。

    ⚠ **槽位有限且不可释放**：每个关节最多 8 个，其中 0 号保留给故障详情
      ⇒ 实际可用 7 个。:meth:`Joint.sdo` 对**同一个端点**返回同一个对象，
      所以正常用法不会耗尽；但不要用循环去创建一堆不同端点的槽位。

    状态（``jsdk_sdo_state_t``）：``0 idle`` → ``1 busy`` → ``2 success`` / ``3 error``。
    """

    __slots__ = ("_j", "_lib", "handle", "path", "ep_id", "size", "_ptype")

    def __init__(self, joint: "Joint", handle: int, path: str | None,
                 ep_id: int, size: int, ptype) -> None:
        self._j = joint
        self._lib = joint._lib
        self.handle = int(handle)
        self.path = path
        self.ep_id = int(ep_id)
        self.size = int(size)
        self._ptype = ptype

    # --- 状态 -------------------------------------------------------------

    @property
    def state(self) -> int:
        """0 idle / 1 busy / 2 success / 3 error。"""
        return int(self._lib.jsdk_joint_sdo_state(self._j._ptr, self.handle))

    @property
    def state_name(self) -> str:
        return {0: "idle", 1: "busy", 2: "success",
                3: "error"}.get(self.state, "?")

    @property
    def data(self) -> bytes:
        """槽位的**裸字节**（副本；长度 = :attr:`size`）。"""
        ptr = self._lib.jsdk_joint_sdo_data(self._j._ptr, self.handle)
        if not ptr:
            raise ValueError("SDO 句柄无效或数据不可取")
        return ctypes.string_at(ptr, self.size)

    def _data_ptr(self):
        return self._lib.jsdk_joint_sdo_data(self._j._ptr, self.handle)

    # --- 非阻塞 -----------------------------------------------------------

    def start_read(self) -> None:
        """发起读（**不等待**）。之后用 :attr:`state` 轮询。"""
        self._j._chk(self._lib.jsdk_joint_sdo_read(self._j._ptr, self.handle),
                     "sdo_read")

    def start_write(self, data: bytes | bytearray) -> None:
        """把 ``data`` 写进槽位并**发起写**（不等待）。

        ``data`` 长度必须等于 :attr:`size`（否则设备按长度校验会拒收）。
        """
        b = bytes(data)
        if len(b) != self.size:
            raise ValueError(f"{self.path or self.ep_id}: 需要 {self.size} 字节，"
                             f"给了 {len(b)}")
        ptr = self._data_ptr()
        if not ptr:
            raise ValueError("SDO 句柄无效")
        ctypes.memmove(ptr, b, self.size)
        self._j._chk(self._lib.jsdk_joint_sdo_write(self._j._ptr, self.handle),
                     "sdo_write")

    # --- 阻塞便利（内部自己跑周期）----------------------------------------

    def _pump_until(self, done, timeout_ms: int) -> int:
        """跑控制周期直到 ``done(state)`` 为真；返回最终 state。超时抛异常。"""
        ctx = self._j._ctx
        deadline = time.monotonic() + max(1, timeout_ms) / 1000.0
        while True:
            ctx.cycle_begin(0)
            ctx.cycle_end()
            st = self.state
            if done(st):
                return st
            if time.monotonic() >= deadline:
                raise_for_status(int(Status.TIMEOUT), "sdo_wait",
                                 f"SDO 状态={self.state_name}（{timeout_ms} ms 内没完成）")
            # 与真实控制回路同节奏；period_ns=0 时退化成很短的小睡
            if ctx.period_ns > 0:
                ctx.pace()
            else:  # pragma: no cover - 非默认配置
                time.sleep(0.0005)

    def read(self, timeout_ms: int = 1000) -> bytes:
        """读并返回裸字节（阻塞，内部跑周期）。"""
        self.start_read()
        self._pump_until(lambda st: st in (2, 3), timeout_ms)
        if self.state == 3:
            raise_for_status(int(Status.PROTOCOL), "sdo_read",
                             f"{self.path or self.ep_id}: 设备拒绝（state=error）")
        return self.data

    def write(self, data: bytes | bytearray, timeout_ms: int = 1000) -> None:
        """写裸字节（阻塞，内部跑周期）。"""
        self.start_write(data)
        self._pump_until(lambda st: st in (2, 3), timeout_ms)
        if self.state == 3:
            raise_for_status(int(Status.PROTOCOL), "sdo_write",
                             f"{self.path or self.ep_id}: 设备拒绝（state=error）")

    # --- 便利：按端点类型解成 Python 值（小端，与 C 侧 cb_le_* 一致）-------

    #: 端点类型 → struct 格式（**小端、无填充**，与线上字节一一对应）
    _FMT = {
        EpType.U8: "B", EpType.I8: "b", EpType.BOOL: "B",
        EpType.U16: "H", EpType.I16: "h",
        EpType.U32: "I", EpType.I32: "i",
        EpType.U64: "Q", EpType.I64: "q",
        EpType.F32: "f", EpType.F64: "d",
    }

    def read_value(self) -> object:
        """读并按端点类型解成 Python 值（⚠ 线上是小端）。

        与 ``Joint.param_get(path)`` 读同一个端点时必须得到同一个值 ——
        测试里就是这么对拍的（两条解析路径互相验证）。
        """
        fmt = self._FMT.get(self._ptype)
        if fmt is None:
            raise ValueError(f"{self.path or self.ep_id}: 类型 {self._ptype} 不是标量")
        b = self.read()
        return struct.unpack("<" + fmt, b[:struct.calcsize(fmt)])[0]

    def write_value(self, value: object, timeout_ms: int = 1000) -> None:
        """按端点类型编码并写（**小端**）。"""
        fmt = self._FMT.get(self._ptype)
        if fmt is None:
            raise ValueError(f"{self.path or self.ep_id}: 类型 {self._ptype} 不是标量")
        if isinstance(value, bool):
            value = int(value)
        packed = struct.pack("<" + fmt, value)
        if len(packed) < self.size:
            packed = packed + b"\x00" * (self.size - len(packed))
        self.write(packed, timeout_ms=timeout_ms)

    def __repr__(self) -> str:  # pragma: no cover - 人读
        return (f"<Sdo h={self.handle} {self.path or self.ep_id} "
                f"size={self.size} state={self.state_name}>")


class Joint:
    """一个关节。

    不要自己构造：用 :meth:`Context.add_joint`。
    """

    def __init__(self, ctx, ptr, index: int, node_id: int) -> None:
        self._ctx = ctx
        self._ptr = ptr
        self.index = index
        self.node_id = node_id
        self._lib = ctx._lib
        self._sdos: dict[int, Sdo] = {}

    # --- 内部 -------------------------------------------------------------

    def _chk(self, st: int, op: str, allow: tuple[int, ...] = ()) -> int:
        if int(st) != 0 and int(st) not in allow:
            raise_for_status(int(st), op, self._ctx.last_error())
        return int(st)

    # --- 反馈 -------------------------------------------------------------

    def feedback(self) -> Feedback:
        """读反馈（**不**发起总线交互，取最近一次解码结果）。

        ``age_ms`` 是距上次有效反馈的毫秒数；心跳停了它就会涨。
        """
        c = _abi.JointFeedback()
        self._chk(self._lib.jsdk_joint_get_feedback(self._ptr, ctypes.byref(c)),
                  "get_feedback")
        return Feedback._from_c(c)

    def is_enabled(self) -> bool:
        return bool(self._lib.jsdk_joint_is_enabled(self._ptr))

    def is_fault(self) -> bool:
        return bool(self._lib.jsdk_joint_is_fault(self._ptr))

    def mode_state(self) -> ModeState | int:
        """固件原始 ModeState nibble（排障用；**不是** :class:`AxisState`）。"""
        v = int(self._lib.jsdk_joint_get_mode_state(self._ptr))
        try:
            return ModeState(v)
        except ValueError:  # pragma: no cover
            return v

    def can_state(self) -> int:
        """固件 ``axis0.current_state``（AxisState 0..16）；未读到过返回 -1。

        ⚠ 与 :meth:`mode_state` 不是同一个枚举。3 = 标定中、11 = 回零中。
        """
        return int(self._lib.jsdk_joint_get_can_state(self._ptr))

    def clear_status_flags(self, mask: int = 0xFFFF) -> None:
        """清除粘滞状态标志（``JSDK_JF_*``）。"""
        self._lib.jsdk_joint_clear_status_flags(self._ptr, int(mask))

    # --- 使能与模式 -------------------------------------------------------

    def enable(self, mode: Mode = Mode.MIT) -> None:
        """请求使能（非阻塞）。

        真正的"已使能"要等**使能序列（含安全首帧）走完**；期间
        :meth:`is_enabled` 仍为 False。用 :meth:`Context.activate` 等它完成。
        """
        self._lib.jsdk_joint_request_enable(self._ptr, int(mode))

    def disable(self) -> None:
        """请求失能（非阻塞）。"""
        self._lib.jsdk_joint_request_disable(self._ptr)

    def fault_reset(self) -> None:
        """请求清故障（非阻塞）。"""
        self._lib.jsdk_joint_request_fault_reset(self._ptr)

    def set_mode(self, mode: Mode) -> None:
        """设置本关节的控制模式（下一周期生效）。"""
        self._lib.jsdk_joint_set_mode(self._ptr, int(mode))

    # --- 控制（单位：输出端 rad / rad/s / N·m） ---------------------------

    def set_mit(self, pos: float = 0.0, vel: float = 0.0,
                kp: float = 0.0, kd: float = 0.0, tau: float = 0.0) -> None:
        """MIT 力位混合指令。``kp``/``kd`` 为**线上值、原样透传**。

        ⚠ 固件把 kp/kd 作用在**电机端 turns 误差**上，所以输出端实际刚度
        ``= kp × gear_ratio / 2π``（gear 16.5 时约 2.63 倍）。要按真实刚度给值
        请用 :meth:`set_mit_stiffness`。
        """
        self._lib.jsdk_joint_set_mit(self._ptr, float(pos), float(vel),
                                     float(kp), float(kd), float(tau))

    def set_mit_stiffness(self, pos: float = 0.0, vel: float = 0.0,
                          stiffness: float = 0.0, damping: float = 0.0,
                          tau: float = 0.0) -> None:
        """以**输出端真实刚度/阻尼**为输入（SDK 内部换算 kp = 刚度 × 2π / gear）。"""
        self._lib.jsdk_joint_set_mit_stiffness(
            self._ptr, float(pos), float(vel),
            float(stiffness), float(damping), float(tau))

    def set_position(self, rad: float) -> None:
        """CSP/位置模式的目标位置（rad，输出端）。"""
        self._lib.jsdk_joint_set_target_position_rad(self._ptr, float(rad))

    def set_velocity(self, rad_s: float) -> None:
        """CSV/速度模式的目标速度（rad/s，输出端）。"""
        self._lib.jsdk_joint_set_target_velocity_rad_s(self._ptr, float(rad_s))

    def set_torque(self, Nm: float) -> None:
        """CST/力矩模式的目标力矩（N·m，输出端）。"""
        self._lib.jsdk_joint_set_target_torque_Nm(self._ptr, float(Nm))

    def set_limits(self, vel_lim_rad_s: float, cur_lim_A: float) -> None:
        """CSP/CSV/CURRENT 模式的限制量。"""
        self._lib.jsdk_joint_set_limits(self._ptr, float(vel_lim_rad_s),
                                        float(cur_lim_A))

    def hold_position(self) -> None:
        """按当前模式发**最小能量**指令（MIT 下 = 零增益，电机泄力）。"""
        self._lib.jsdk_joint_hold_position(self._ptr)

    def hold_position_pd(self, kp: float, kd: float) -> None:
        """主动抱持（PD 锁位）；kp/kd 为线上值。"""
        self._lib.jsdk_joint_hold_position_pd(self._ptr, float(kp), float(kd))

    # --- 配置 / 运维（阻塞，返回即抛异常） --------------------------------

    def config_snapshot(self) -> ConfigSnapshot:
        """读回关键标定量。``valid=False`` 时**不要**用这些数。"""
        c = _abi.JointConfigSnapshot()
        self._chk(self._lib.jsdk_joint_read_config_snapshot(self._ptr,
                                                            ctypes.byref(c)),
                  "read_config_snapshot")
        return ConfigSnapshot._from_c(c)

    def device_info(self) -> DeviceInfo:
        """``QUERY_DEVICE_INFO(0x46)``。"""
        c = _abi.DeviceInfo()
        self._chk(self._lib.jsdk_joint_get_device_info(self._ptr, ctypes.byref(c)),
                  "get_device_info")
        return DeviceInfo._from_c(c)

    def fault_info(self) -> FaultInfo:
        """读**缓存**的故障信息（不发起总线交互）。"""
        c = _abi.FaultInfo()
        self._chk(self._lib.jsdk_joint_get_fault_info(self._ptr, ctypes.byref(c)),
                  "get_fault_info")
        return FaultInfo._from_c(c)

    def query_error_detail(self) -> FaultInfo:
        """主动 ``QUERY_ERROR(0x45)`` 六次查询（阻塞，配置阶段用）。"""
        c = _abi.FaultInfo()
        self._chk(self._lib.jsdk_joint_query_error_detail(self._ptr,
                                                          ctypes.byref(c)),
                  "query_error_detail")
        name = self._lib.jsdk_joint_error_string(c.mit_err)
        return FaultInfo._from_c(c, mit_name=name.decode() if name else "")

    def describe_fault(self) -> str:
        """一行可读的故障描述（含 32-bit 位名）。"""
        buf = ctypes.create_string_buffer(192)
        n = int(self._lib.jsdk_joint_describe_fault(self._ptr, buf, len(buf)))
        return buf.value.decode("utf-8", "replace") if n > 0 else ""

    def set_watchdog_ms(self, ms: int) -> None:
        """写设备看门狗（``can.config.break_timeout``）。

        ``ms == 0`` = **关闭**设备侧的协议级超时检测（最新固件的语义：
        ``auto_stop_if_timeout()`` 首句就是 ``if (timeout_ms == 0) return;``，
        且配置项默认值就是 0）。早期固件把 0 当成 100 ms 且无法关闭，
        所以旧文档写的“0 ≠ 关闭”**已过时**。

        ``ms > 0`` 时才真正开启；而该保护**只在设备收到过控制类帧后才武装**
        （纯 ``CURRENT_CONTROL`` 客户端武装不了 —— 固件 F19）。

        ⚠ 真机（fw 1545）实测：该端点的**读回恒为 0**（F28）——写 250 立刻读也是 0。
        因此写非 0 值时若读不回来，会置 :attr:`StatusFlag.WATCHDOG_UNVERIFIED`，
        并且**不抛异常**（SDK 按写入值保守处理：继续喂狗是安全方向）。
        调用方要判断“到底确认了没有”，读这个标志位，或看 ``Context.last_error()``。

        写入不落 Flash，需要持久化请再 :meth:`save_config`。
        """
        self._chk(self._lib.jsdk_joint_set_watchdog_ms(self._ptr, int(ms)),
                  "set_watchdog_ms")

    def set_zero_here(self) -> None:
        """``SET_ZERO(0x61)``：把当前位置设为零点（不落 Flash）。"""
        self._chk(self._lib.jsdk_joint_set_zero_here(self._ptr), "set_zero_here")

    def calibrate(self) -> None:
        """全标定（写 ``requested_state=3`` 并等状态跳转）。**电机会动**，耗时数秒。"""
        self._chk(self._lib.jsdk_joint_calibrate(self._ptr), "calibrate")

    def home(self) -> None:
        """回零（写 ``requested_state=11`` 并等状态跳转）。"""
        self._chk(self._lib.jsdk_joint_home(self._ptr), "home")

    def save_config(self) -> None:
        """``CONFIG_SAVE(0x22)``：写 Flash（写后读回校验）。"""
        self._chk(self._lib.jsdk_joint_save_config(self._ptr), "save_config")

    def reset_device(self) -> None:
        """``RESET_DEVICE(0x64)`` 软复位；之后需要重新握手。"""
        self._chk(self._lib.jsdk_joint_reset_device(self._ptr), "reset_device")

    def set_node_id(self, new_id: int, persist: bool = True) -> None:
        """``SET_NODE_ID(0x60)``。``persist=True`` 时追加 ``CONFIG_SAVE``。

        成功后本对象的 :attr:`node_id` 会更新；**调用方**也该更新自己的记录
        （SDK 不会自动改 Context 里的关节映射，因为那份映射由 node_id 索引）。
        """
        if not 1 <= int(new_id) <= 254:
            raise ValueError("node_id 必须在 1..254")
        self._chk(self._lib.jsdk_joint_set_node_id(self._ptr, int(new_id),
                                                   1 if persist else 0),
                  "set_node_id")
        self.node_id = int(new_id)

    # --- 参数（SDO 风格，按路径名） ---------------------------------------

    def param_get(self, path: str) -> object:
        """按路径读参数，返回 Python 原生值。

        :raises JsdkUsageError: 路径不存在
        :raises JsdkProtocolError: 类型与端点不符
        """
        v = _abi.Value()
        self._chk(self._lib.jsdk_joint_param_get(self._ptr, path.encode("utf-8"),
                                                 ctypes.byref(v)),
                  f"param_get({path})")
        return _value_to_python(v)

    def param_set(self, path: str, value: object) -> None:
        """按路径写参数。``value`` 的类型要与端点匹配（否则 ``PROTOCOL``）。

        ⚠ **严格版**：``int`` 一律按 u32/i32 发。要写 u8/u16/i8/i16 端点，
          用 :meth:`param_set_auto`（按端点声明类型装箱）或
          :meth:`param_set_u32`（C 侧按真实位宽收窄）。
        """
        v = _python_to_value(value)
        self._chk(self._lib.jsdk_joint_param_set(self._ptr, path.encode("utf-8"),
                                                 ctypes.byref(v)),
                  f"param_set({path})")

    def param_set_auto(self, path: str, value: object) -> None:
        """写参数，**按端点声明的类型**给值装箱（``param_set`` 的便利版）。

        :param value: ``bool`` / ``int`` / ``float``（字符串由调用方解析）
        :raises JsdkProtocolError: 值超出该类型值域（**不静默截断**）、
                                   或端点类型不是标量

        为什么需要它：``param_set()`` 靠 **Python 类型**猜线上宽度，
        ``param_set(path, 250)`` 会被当成 u32 发；端点其实是 u16 时，C 侧会以
        “descriptor=uint16 given=uint32” 拒绝 —— 于是 u8/u16/i8/i16 端点
        **从 Python 根本写不进去**（看着像“写了没反应”）。
        这里改成先查描述符、再按端点的真实类型打包，于是：

        >>> j.param_set_auto("can.config.break_timeout", 250)   # u16 端点，直接可用
        >>> j.param_set_auto("axis0.config.enable_watchdog", True)   # bool
        >>> j.param_set_auto("axis0.motor.config.gear_ratio", 8)     # f32 端点给 int
        """
        ep = self._ctx.lookup(path)                  # 未命中抛 NOT_FOUND
        t = EpType(int(ep.type))

        if t in (EpType.F32, EpType.F64):
            v = _abi.Value()
            v.type = int(t)
            _VALUE_SETTERS[t](v, float(value))
        elif t is EpType.BOOL:
            if not isinstance(value, (bool, int)):
                raise_for_status(int(Status.PROTOCOL), "param_set_auto",
                                 f"{path}: bool 端点只接受 0/1/true/false")
            v = _abi.Value()
            v.type = int(t)
            _VALUE_SETTERS[t](v, value)
        elif t in _INT_RANGES:
            if isinstance(value, bool) or not isinstance(value, int):
                raise_for_status(
                    int(Status.PROTOCOL), "param_set_auto",
                    f"{path}: {ep.type_name} 端点需要整数（收到 {type(value).__name__}）")
            lo, hi = _INT_RANGES[t]
            if not lo <= value <= hi:
                raise_for_status(
                    int(Status.PROTOCOL), "param_set_auto",
                    f"{path}: {value} 超出 {ep.type_name} 值域（{lo}..{hi}）")
            v = _abi.Value()
            v.type = int(t)
            _VALUE_SETTERS[t](v, value)
        else:
            raise_for_status(int(Status.UNSUPPORTED), "param_set_auto",
                             f"{path}: 类型 {ep.type_name} 不是标量")

        self._chk(self._lib.jsdk_joint_param_set(self._ptr, path.encode("utf-8"),
                                                 ctypes.byref(v)),
                  f"param_set_auto({path})")

    def param_get_f32(self, path: str) -> float:
        out = _abi.c_float()
        self._chk(self._lib.jsdk_joint_param_get_f32(self._ptr, path.encode("utf-8"),
                                                     ctypes.byref(out)),
                  f"param_get_f32({path})")
        return float(out.value)

    def param_set_f32(self, path: str, value: float) -> None:
        self._chk(self._lib.jsdk_joint_param_set_f32(self._ptr, path.encode("utf-8"),
                                                     float(value)),
                  f"param_set_f32({path})")

    def param_get_u32(self, path: str) -> int:
        out = _abi.c_uint32()
        self._chk(self._lib.jsdk_joint_param_get_u32(self._ptr, path.encode("utf-8"),
                                                     ctypes.byref(out)),
                  f"param_get_u32({path})")
        return int(out.value)

    def param_set_u32(self, path: str, value: int) -> None:
        """写 u32；SDK 会按端点**真实位宽**自动收窄（如 u8 端点）。"""
        self._chk(self._lib.jsdk_joint_param_set_u32(self._ptr, path.encode("utf-8"),
                                                     int(value)),
                  f"param_set_u32({path})")

    def param_get_i32(self, path: str) -> int:
        out = _abi.c_int32()
        self._chk(self._lib.jsdk_joint_param_get_i32(self._ptr, path.encode("utf-8"),
                                                     ctypes.byref(out)),
                  f"param_get_i32({path})")
        return int(out.value)

    def param_get_bool(self, path: str) -> bool:
        out = _abi.c_int()
        self._chk(self._lib.jsdk_joint_param_get_bool(self._ptr, path.encode("utf-8"),
                                                      ctypes.byref(out)),
                  f"param_get_bool({path})")
        return bool(out.value)

    def param_get_batch(self, *paths: str) -> dict[str, object]:
        """批量读。FD 下打包成单帧；Classic 下 C 侧自动退化为逐条。

        :return: ``{path: value}``；某条失败时该条的值是 :class:`JsdkError` 实例
                 （不抛异常，让调用方一次拿到全部结果）。
        """
        if not paths:
            raise ValueError("至少给一个路径")
        if len(paths) > 8:
            raise ValueError("一次最多 8 条（超了请分批）")

        arr = (_abi.ParamReq * len(paths))()
        for i, p in enumerate(paths):
            arr[i].path = p.encode("utf-8")

        self._chk(self._lib.jsdk_joint_param_get_batch(self._ptr, arr, len(paths)),
                  "param_get_batch")

        from .errors import error_for_status

        out: dict[str, object] = {}
        for i, p in enumerate(paths):
            if int(arr[i].status) == 0:
                out[p] = _value_to_python(arr[i].value)
            else:
                out[p] = error_for_status(int(arr[i].status), f"param_get({p})")
        return out

    # --- SDO 槽位（裸字节 / 轮询状态机） ----------------------------------

    #: 端点类型 → 字节宽度（用于创建 SDO 槽位时的显式长度校验）
    _WIDTHS = {
        EpType.U8: 1, EpType.I8: 1, EpType.BOOL: 1,
        EpType.U16: 2, EpType.I16: 2,
        EpType.U32: 4, EpType.I32: 4, EpType.F32: 4,
        EpType.U64: 8, EpType.I64: 8, EpType.F64: 8,
    }

    @property
    def sdo_slots_used(self) -> int:
        """本关节已占用的 SDO 槽位数（每关节最多 8 个，0 号保留给故障详情）。"""
        return len(self._sdos)

    def sdo(self, path_or_ep: str | int, *, subindex: int = 0,
            size: int = 0) -> Sdo:
        """取一个 SDO 槽位（**同一个端点复用同一个对象**）。

        :param path_or_ep: 端点路径（str）或端点 ID（int）
        :param subindex: 本协议恒为 0（端点 ID 是平铺的）
        :param size: 0 = 从描述符推断（推荐）
        :raises JsdkNotFoundError: 端点不存在 / 端点没有标量宽度 / 槽位用尽

        ⚠ 槽位**不可释放**（C 侧没有 free），所以这里按端点 ID 复用对象；
          真正的边界是"不同端点的个数"：一个关节最多 7 个（0 号留给故障详情）。
        """
        ctx = self._ctx
        ptype = None

        if isinstance(path_or_ep, str):
            ep = ctx.lookup(path_or_ep)          # 未命中会抛 NOT_FOUND
            ep_id = int(ep.ep_id)
            ptype = ep.type
            if size == 0:
                w = self._WIDTHS.get(ep.type)
                if w is None:
                    raise_for_status(int(Status.UNSUPPORTED), "sdo",
                                     f"{path_or_ep}: 类型 {ep.type_name} 不是标量"
                                     "（SDO 只访问标量端点）")
                size = w
            key = ep_id
            path = path_or_ep
        else:
            ep_id = int(path_or_ep)
            key = ep_id
            path = None
            for e in ctx.endpoints():            # 反过来找路径（为了取类型名）
                if int(e.ep_id) == ep_id:
                    path = e.path
                    ptype = e.type
                    break

        cached = self._sdos.get(key)
        if cached is not None:
            return cached

        h = int(self._lib.jsdk_joint_sdo_create(self._ptr, ep_id, int(subindex),
                                                int(size)))
        if h <= 0:
            raise_for_status(int(Status.NOT_FOUND), f"sdo({path or ep_id})",
                             self._ctx.last_error())
        width = int(self._lib.jsdk_joint_sdo_data_size(self._ptr, h))
        obj = Sdo(self, h, path, ep_id, width, ptype)
        self._sdos[key] = obj
        return obj

    # --- 逃生通道：协议原始量 / 电流模式 / 手工换算 ------------------------

    def set_position_raw(self, raw: int) -> None:
        """目标位置 = ``raw / 1000`` **度**（输出端；CSP/POS 模式的协议原始量）。

        ⚠ 这是**逃生通道**：SDK 不做任何换算，单位也不是 rad。
        常规用法请用 :meth:`set_position`（rad，自动按标定换算）。
        """
        self._lib.jsdk_joint_set_target_position(self._ptr, int(raw))

    def set_velocity_raw(self, raw: int) -> None:
        """目标速度 = ``raw / 1000`` **RPM**（输出端；CSV/VEL 模式）。"""
        self._lib.jsdk_joint_set_target_velocity(self._ptr, int(raw))

    def set_torque_raw(self, raw: int) -> None:
        """目标力矩 = ``raw / 1000`` **N·m**（输出端；CST 模式）。"""
        self._lib.jsdk_joint_set_target_torque(self._ptr, int(raw))

    def set_current(self, amp: float) -> None:
        """CURRENT 模式的**电机端**电流（A）。

        ⚠⚠ 本调用**不喂看门狗**（与固件一致：``0x04`` 不刷新 ``last_cmd_time_``）。
        也就是说：只发电流指令时设备的协议级超时保护**永远不会武装**，
        掉线也不会自动停机（`FIRMWARE_ISSUES` 的 **F19**）。
        必须周期性另发一条 MIT 帧（随便什么目标值）才能把保护维持住 ——
        用 :meth:`set_mit` 或 :meth:`hold_position`。
        """
        self._lib.jsdk_joint_set_current_A(self._ptr, float(amp))

    def set_scale(self, scale) -> None:
        """手工覆盖输出端换算系数（:class:`~jsdk_can.units.UnitScale`）。

        ⚠ 只在"设备标定不可用、但你手里有权威换算表"时用：覆盖之后 SDK 不再用
        从设备读回的 ``gear_ratio`` / ``torque_constant`` 做换算。
        """
        from .units import UnitScale

        s = scale if isinstance(scale, UnitScale) else UnitScale.from_dict(scale)
        self._lib.jsdk_joint_set_scale(self._ptr, ctypes.byref(s._c))

    def get_scale(self):
        """当前生效的换算系数（:class:`~jsdk_can.units.UnitScale`）。"""
        from .units import UnitScale

        c = _abi.UnitScale()
        self._lib.jsdk_joint_get_scale(self._ptr, ctypes.byref(c))
        return UnitScale._from_c(c)

    def __repr__(self) -> str:  # pragma: no cover - 人读
        return f"<Joint node_id={self.node_id} index={self.index}>"


# ==========================================================================
# 值转换辅助
# ==========================================================================


#: 各整数类型的值域（用于 param_set_auto 的**显式**范围校验：
#: 超出就报 PROTOCOL，不静默截断 —— 与 C 版 CLI 的 text_to_value 行为一致）。
_INT_RANGES = {
    EpType.U8:  (0, 255),
    EpType.I8:  (-128, 127),
    EpType.U16: (0, 65535),
    EpType.I16: (-32768, 32767),
    EpType.U32: (0, 4294967295),
    EpType.I32: (-2147483648, 2147483647),
    EpType.U64: (0, 18446744073709551615),
    EpType.I64: (-9223372036854775808, 9223372036854775807),
}

_VALUE_SETTERS = {
    EpType.U8: lambda v, x: setattr(v.v, "u8", int(x)),
    EpType.I8: lambda v, x: setattr(v.v, "i8", int(x)),
    EpType.U16: lambda v, x: setattr(v.v, "u16", int(x)),
    EpType.I16: lambda v, x: setattr(v.v, "i16", int(x)),
    EpType.U32: lambda v, x: setattr(v.v, "u32", int(x)),
    EpType.I32: lambda v, x: setattr(v.v, "i32", int(x)),
    EpType.U64: lambda v, x: setattr(v.v, "u64", int(x)),
    EpType.I64: lambda v, x: setattr(v.v, "i64", int(x)),
    EpType.F32: lambda v, x: setattr(v.v, "f32", float(x)),
    EpType.F64: lambda v, x: setattr(v.v, "f64", float(x)),
    EpType.BOOL: lambda v, x: setattr(v.v, "boolean", 1 if x else 0),
}

_VALUE_GETTERS = {
    EpType.U8: lambda v: int(v.v.u8),
    EpType.I8: lambda v: int(v.v.i8),
    EpType.U16: lambda v: int(v.v.u16),
    EpType.I16: lambda v: int(v.v.i16),
    EpType.U32: lambda v: int(v.v.u32),
    EpType.I32: lambda v: int(v.v.i32),
    EpType.U64: lambda v: int(v.v.u64),
    EpType.I64: lambda v: int(v.v.i64),
    EpType.F32: lambda v: float(v.v.f32),
    EpType.F64: lambda v: float(v.v.f64),
    EpType.BOOL: lambda v: bool(v.v.boolean),
}


def _python_to_value(value: object) -> _abi.Value:
    """按 Python 类型推断端点类型（``bool`` 必须先判，它是 ``int`` 的子类）。"""
    v = _abi.Value()
    if isinstance(value, bool):
        v.type = int(EpType.BOOL)
        v.v.boolean = 1 if value else 0
    elif isinstance(value, int):
        # 用 i32 兜底；C 侧会按端点真实位宽校验/收窄，不匹配则报 PROTOCOL
        v.type = int(EpType.I32) if value < 0 else int(EpType.U32)
        _VALUE_SETTERS[EpType(v.type)](v, value)
    elif isinstance(value, float):
        v.type = int(EpType.F32)
        v.v.f32 = value
    else:
        raise TypeError(f"不支持的参数值类型：{type(value).__name__}")
    return v


def _value_to_python(v: _abi.Value) -> object:
    try:
        ep = EpType(int(v.type))
    except ValueError:  # pragma: no cover
        return None
    getter = _VALUE_GETTERS.get(ep)
    return getter(v) if getter else None
