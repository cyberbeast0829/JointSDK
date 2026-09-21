"""枚举与常量（数值与 ``joint_sdk.h`` **逐值一致**）。

用 ``IntEnum`` 而不是裸常量：这样 ``print(Mode.MIT)`` 给出 ``Mode.MIT``，
而比较/传参仍然就是整数。
"""

from __future__ import annotations

from enum import IntEnum

from ._abi import (
    JSDK_ABI_VERSION_CAN,
    JSDK_BACKEND_TAG_CAN,
    JSDK_FRAME_BRS,
    JSDK_FRAME_EXT,
    JSDK_FRAME_FD,
    JSDK_HAL_BUS_ERROR_PASS,
    JSDK_HAL_BUS_ERROR_WARN,
    JSDK_HAL_BUS_OFF,
    JSDK_HAL_BUS_OK,
    JSDK_HAL_LISTEN_ONLY,
)

__all__ = [
    "Status",
    "Mode",
    "AxisState",
    "ModeState",
    "EpType",
    "EpAccess",
    "DescMode",
    "DescRetain",
    "BusFlag",
    "FrameFlag",
    "StatusFlag",
    "BACKEND_TAG",
    "ABI_VERSION",
]


BACKEND_TAG = JSDK_BACKEND_TAG_CAN
ABI_VERSION = JSDK_ABI_VERSION_CAN


class Status(IntEnum):
    """``jsdk_status_t``。0 = 成功，负数 = 错误。"""

    OK = 0
    INVALID_ARG = -1
    NO_MEMORY = -2
    NOT_FOUND = -3
    BAD_STATE = -4
    TRANSPORT = -5
    UNSUPPORTED = -6
    TIMEOUT = -7
    PROTOCOL = -8
    BUSY = -9
    PARSE = -10

    #: 与 EtherCAT 家族同义（源码兼容别名）
    @property
    def is_error(self) -> bool:
        return int(self) != 0


class Mode(IntEnum):
    """控制模式。

    MIT(4) 与固件 ModeState nibble 对齐；CSP/CSV/CST 沿用 EtherCAT 家族数值，
    CAN 侧分别映射到固件 POS/VEL/TORQUE_CONTROL。**线上单位不同，SDK 负责换算。**
    """

    MIT = 4
    CSP = 8
    CSV = 9
    CST = 10
    CURRENT = 11


class AxisState(IntEnum):
    """归一化关节状态（**不是**固件 ``current_state``）。"""

    UNKNOWN = 0
    SWITCH_ON_DISABLED = 1
    READY_TO_SWITCH_ON = 2
    SWITCHED_ON = 3
    OPERATION_ENABLED = 4
    FAULT = 5


class ModeState(IntEnum):
    """固件 ModeState nibble 原始值（排障用）。"""

    RESET = 0
    CALIBRATING = 1
    IDLE = 2
    CLOSED_LOOP = 3
    MIT = 4
    POSITION = 5
    VELOCITY = 6
    TORQUE = 7


class EpType(IntEnum):
    """``jsdk_ep_type_t``（与描述符 JSON 的 type 对应）。"""

    U8 = 1
    I8 = 2
    U16 = 3
    I16 = 4
    U32 = 5
    I32 = 6
    U64 = 7
    I64 = 8
    F32 = 9
    F64 = 10
    BOOL = 11
    # 以下为不可直接读写的不透明类型（仅用于枚举）
    OBJECT = 12
    ENDPOINT_REF = 13
    JSON = 14
    FUNCTION = 15


EP_TYPE_STRINGS = {
    EpType.U8: "uint8",
    EpType.I8: "int8",
    EpType.U16: "uint16",
    EpType.I16: "int16",
    EpType.U32: "uint32",
    EpType.I32: "int32",
    EpType.U64: "uint64",
    EpType.I64: "int64",
    EpType.F32: "float",
    EpType.F64: "double",
    EpType.BOOL: "bool",
    EpType.OBJECT: "object",
    EpType.ENDPOINT_REF: "endpoint_ref",
    EpType.JSON: "json",
    EpType.FUNCTION: "function",
}


class EpAccess(IntEnum):
    """端点权限位（可组合）。"""

    R = 0x01
    W = 0x02


class DescMode(IntEnum):
    """``jsdk_desc_mode_t``。"""

    #: 描述符不存在时向设备下载并解析（默认）
    DYNAMIC = 0
    #: 强制不下载：configure() 时若仍不存在就报错（保证启动时间/离线）
    FROM_CACHE = 1


class DescRetain(IntEnum):
    """``jsdk_desc_retain_t``。"""

    #: 保留全部端点（≈24 KB；桌面）
    ALL = 0
    #: 只保留 ``filter_paths`` 命中的端点（MCU，典型 <1 KB）
    FILTERED = 1


class BusFlag(IntEnum):
    """``JSDK_HAL_BUS_*``（``bus_state.hal_bus_flags``）。"""

    OK = JSDK_HAL_BUS_OK
    ERROR_WARN = JSDK_HAL_BUS_ERROR_WARN
    ERROR_PASS = JSDK_HAL_BUS_ERROR_PASS
    BUS_OFF = JSDK_HAL_BUS_OFF
    LISTEN_ONLY = JSDK_HAL_LISTEN_ONLY


class FrameFlag(IntEnum):
    """``JSDK_FRAME_*``。"""

    FD = JSDK_FRAME_FD
    BRS = JSDK_FRAME_BRS
    EXT = JSDK_FRAME_EXT


class StatusFlag(IntEnum):
    """``JSDK_JF_*`` 粘滞标志（``feedback.status_flags``）。

    **粘滞**：置位后一直保持，直到显式 ``Joint.clear_status_flags()``。
    数值逐位取自 ``joint_sdk.h``（不凭记忆写）。
    """

    NONE = 0x0000
    #: 曾因越界拒绝目标位置
    TARGET_REJECTED = 0x0001
    #: 曾在本周期改发安全帧
    SAFE_FRAME_SENT = 0x0002
    #: 控制周期接近 break_timeout
    WATCHDOG_RISK = 0x0004
    #: 反馈超时（age_ms 超阈值）
    FEEDBACK_STALE = 0x0008
    #: HAL send 连续失败
    TX_FAILED = 0x0010
    #: 未取得标定参数，物理量 API 不可用
    SCALE_INVALID = 0x0020
    #: 写 ``can.config.break_timeout`` 后设备读回不符（真机 F28：该端点读回恒 0）
    #: —— 已按写入值保守处理，但**不能**当成“已武装”
    WATCHDOG_UNVERIFIED = 0x0040
