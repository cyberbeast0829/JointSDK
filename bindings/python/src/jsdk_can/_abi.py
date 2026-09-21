"""ctypes ABI 绑定层 —— 与 ``joint_sdk.h`` 一一对应。

这一层是**唯一的** ctypes 代码所在处；上层（``context`` / ``joint``）只写
Python 语义，不碰指针。

设计要点
--------

1. **不使用 ctypes 回调拼 ``jsdk_can_hal_t``**。
   Python 回调进 C 再回到 Python 的开销与 GIL 抖动都不可控，且一旦在 RT 路径上
   被调用就会破坏 SDK 的非阻塞约定。改为：由 C 侧的内置后端工厂
   （``jsdk_hal_virtual_open`` / ``..._socketcan_open`` / ...）把回调**填进**我们
   提供的存储里 —— Python 只选后端名。这也是设计文档 §7.3 的做法。

2. **导入时做 ABI 自检**（``_check_abi``）。
   ctypes 结构体是"猜"出来的布局；猜错不会报错，而是**直接踩内存**。C 侧导出
   ``jsdk_abi_types()`` 给出每个公开结构体的 ``sizeof``/对齐，这里逐项比对，
   不一致立刻抛 :class:`AbiMismatchError` 并指名是哪个类型。

3. **对齐分配**。
   ``jsdk_context_t`` 与描述符 arena 都含指针/双精度成员，必须 8 字节对齐。
   ``ctypes.create_string_buffer()`` 只保证 1 字节对齐，直接拿它当上下文用就是
   未定义行为（Windows 上会因为未对齐的 SSE 访问直接崩）。所以统一走
   :func:`alloc_aligned`。
"""

from __future__ import annotations

import ctypes
import glob
import os
import sys
from ctypes import (
    POINTER,
    byref,
    c_char_p,
    c_double,
    c_float,
    c_int,
    c_int8,
    c_int16,
    c_int32,
    c_int64,
    c_size_t,
    c_uint8,
    c_uint16,
    c_uint32,
    c_uint64,
    c_void_p,
)

# ⚠ 这里**不能**在模块级 import .errors：enums 依赖本模块的常量，会成环。
#   异常类改在 raises 处按需 import（模块已缓存，无额外开销）。

# ==========================================================================
# 常量（与 joint_sdk.h 保持一致）
# ==========================================================================

JSDK_BACKEND_TAG_CAN = 3
JSDK_ABI_VERSION_CAN = 0x00010000

JSDK_FRAME_FD = 0x01
JSDK_FRAME_BRS = 0x02
JSDK_FRAME_EXT = 0x04

JSDK_HAL_BUS_OK = 0x00000001
JSDK_HAL_BUS_ERROR_WARN = 0x00000002
JSDK_HAL_BUS_ERROR_PASS = 0x00000004
JSDK_HAL_BUS_OFF = 0x00000008
JSDK_HAL_LISTEN_ONLY = 0x00000010

# §6.2 关节配置里的 0 表示"从设备读取"
JSDK_MAX_JOINTS_STATIC = 8
JSDK_CONTEXT_MAX_SIZE = 6144


# ==========================================================================
# 结构体
# ==========================================================================


class CanFrame(ctypes.Structure):
    """``jsdk_can_frame_t``。"""

    _fields_ = [
        ("id", c_uint32),
        ("len", c_uint8),
        ("flags", c_uint8),
        ("data", c_uint8 * 64),
    ]


class CanHal(ctypes.Structure):
    """``jsdk_can_hal_t``。

    Python **从不调用**这些回调（见模块文档第 1 点），所以函数指针统一声明成
    ``c_void_p``：既不引入 CFUNCTYPE 的样板，也不会因为调用约定写错而在运行期
    崩。字段数量必须与 C 一致 —— ``_check_abi`` 会核对总尺寸。
    """

    _fields_ = [
        ("user", c_void_p),
        ("send", c_void_p),
        ("recv", c_void_p),
        ("now_ms", c_void_p),
        ("on_error", c_void_p),
        ("bus_status", c_void_p),
    ]


class DescConfig(ctypes.Structure):
    """``jsdk_desc_config_t``。"""

    _fields_ = [
        ("mode", c_uint8),
        ("retain", c_uint8),
        ("share_by_crc", c_uint8),
        ("stop_when_satisfied", c_uint8),
        ("max_endpoints", c_uint16),
        ("max_path_len", c_uint16),
        ("timeout_ms", c_uint32),
        ("filter_paths", POINTER(c_char_p)),
        ("filter_count", c_uint32),
        ("arena", c_void_p),
        ("arena_size", c_size_t),
        ("arena_used", c_size_t),
    ]


class ContextConfig(ctypes.Structure):
    """``jsdk_context_config_t``。"""

    _fields_ = [
        ("magic", c_uint32),
        ("hal", CanHal),
        ("master_id", c_uint8),
        ("is_fd", c_uint8),
        ("period_ns", c_uint32),
        ("state_timeout_ms", c_uint32),
        ("auto_keepalive", c_uint8),
        ("clamp_target_position", c_uint8),
        ("enable_watchdog_hint", c_uint8),
        ("max_joints", c_uint8),
        ("rx_burst_limit", c_uint8),
        ("desc", DescConfig),
    ]


class JointConfig(ctypes.Structure):
    """``jsdk_joint_config_t``。"""

    _fields_ = [
        ("magic", c_uint32),
        ("node_id", c_uint8),
        ("axis", c_uint8),
        ("gear_ratio", c_float),
        ("mit_max_pos", c_float),
        ("mit_max_vel", c_float),
        ("mit_max_torque", c_float),
        ("mit_max_kp", c_float),
        ("mit_max_kd", c_float),
        ("torque_constant", c_float),
        ("initial_mode", c_int),
    ]


class JointConfigSnapshot(ctypes.Structure):
    """``jsdk_joint_config_snapshot_t``。"""

    _fields_ = [
        ("gear_ratio", c_float),
        ("mit_max_pos", c_float),
        ("mit_max_vel", c_float),
        ("mit_max_torque", c_float),
        ("mit_max_kp", c_float),
        ("mit_max_kd", c_float),
        ("torque_constant", c_float),
        ("node_id", c_uint32),
        ("heartbeat_rate_ms", c_uint32),
        ("break_timeout_ms", c_uint32),
        ("valid", c_int),
    ]


class JointFeedback(ctypes.Structure):
    """``jsdk_joint_feedback_t``。"""

    _fields_ = [
        ("pos", c_double),
        ("vel", c_double),
        ("current_A", c_double),
        ("torque_Nm", c_double),
        ("t_motor_C", c_double),
        ("t_fet_C", c_double),
        ("vbus_V", c_double),
        ("ibus_A", c_double),
        ("axis_state", c_int),
        ("mode", c_int),
        ("mode_state", c_int),
        ("err_code", c_uint8),
        ("hb_error", c_uint8),
        ("axis_error", c_uint32),
        ("age_ms", c_uint32),
        ("tx_rejected", c_uint32),
        ("tx_frames", c_uint32),
        ("status_flags", c_uint16),
        ("online", c_int),
        ("valid", c_int),
    ]


class BusState(ctypes.Structure):
    """``jsdk_bus_state_t``。"""

    _fields_ = [
        ("tx_frames", c_uint32),
        ("rx_frames", c_uint32),
        ("tx_failed", c_uint32),
        ("rx_dropped", c_uint32),
        ("keepalive_sent", c_uint32),
        ("last_rx_age_ms", c_uint32),
        ("hal_bus_flags", c_uint32),
        ("link_errors", c_uint32),
        ("nodes_online", c_uint8),
        ("link_up", c_uint8),
    ]


class DeviceInfo(ctypes.Structure):
    """``jsdk_device_info_t``。"""

    _fields_ = [
        ("hw_version", c_uint32),
        ("fw_version", c_uint32),
        ("serial", c_uint64),
        ("classic", c_uint8),
    ]


class FaultInfo(ctypes.Structure):
    """``jsdk_fault_info_t``。"""

    _fields_ = [
        ("valid", c_int),
        ("mit_err", c_uint8),
        ("hb_flags", c_uint8),
        ("motor_error", c_uint32),
        ("encoder_error", c_uint32),
        ("sensorless_error", c_uint32),
        ("controller_error", c_uint32),
        ("system_error", c_uint32),
        ("axis_error", c_uint32),
    ]


class _ValueUnion(ctypes.Union):
    _fields_ = [
        ("u8", c_uint8),
        ("i8", c_int8),
        ("u16", c_uint16),
        ("i16", c_int16),
        ("u32", c_uint32),
        ("i32", c_int32),
        ("u64", c_uint64),
        ("i64", c_int64),
        ("f32", c_float),
        ("f64", c_double),
        ("boolean", c_int),
    ]


class Value(ctypes.Structure):
    """``jsdk_value_t``。"""

    _fields_ = [("type", c_int), ("v", _ValueUnion)]


class UnitScale(ctypes.Structure):
    """``jsdk_unit_scale_t``。"""

    _fields_ = [
        ("pos_counts_to_rad", c_double),
        ("vel_counts_to_rad_s", c_double),
        ("trq_to_Nm", c_double),
        ("valid", c_int),
    ]


class ParamReq(ctypes.Structure):
    """``jsdk_param_req_t``。"""

    _fields_ = [
        ("path", c_char_p),
        ("value", Value),
        ("status", c_int),
    ]


class GroupTarget(ctypes.Structure):
    """``jsdk_group_target_t``。"""

    _fields_ = [
        ("node_id", c_uint8),
        ("pos_rad", c_double),
        ("vel_rad_s", c_double),
        ("kp", c_double),
        ("kd", c_double),
        ("tau_Nm", c_double),
    ]


class DescInfo(ctypes.Structure):
    """``jsdk_desc_info_t``。"""

    _fields_ = [
        ("total_len", c_uint32),
        ("crc", c_uint16),
        ("fw_version", c_uint32),
        ("hw_version", c_uint32),
        ("endpoint_count", c_int),
        ("parsed_total", c_int),
        ("frames_rx", c_int),
        ("bytes_scanned", c_uint32),
        ("complete", c_uint8),
        ("mode_used", c_uint8),
        ("shared_hit", c_uint8),
        ("raw_sink_failed", c_uint8),
    ]


class AbiType(ctypes.Structure):
    """``jsdk_abi_type_t``（自检用）。"""

    _fields_ = [
        ("name", c_char_p),
        ("size", c_uint32),
        ("align", c_uint32),
    ]


class DescHint(ctypes.Structure):
    """``jsdk_desc_hint_t`` —— 原始 JSON 缓存（路线 B）的失效键。

    ⚠ C 的 ``jsdk_abi_types()`` **不报**这个类型的尺寸，所以它不在 :data:`TYPE_MAP`
    里；布局由测试显式断言（``sizeof == 8``：u16 + 2 字节填充 + u32）。
    """

    _fields_ = [
        ("crc", c_uint16),
        ("fw_version", c_uint32),
    ]


#: ``jsdk_endpoint_visit_fn``。**唯一**允许的 ctypes 回调：它只在配置阶段
#: （`enumerate`）被逐条调用，不在控制回路上，因此 GIL 开销可接受。
ENDPOINT_VISIT_FN = ctypes.CFUNCTYPE(
    c_int, c_void_p, c_char_p, c_uint16, c_int, c_uint8
)

#: ``jsdk_desc_raw_sink_fn``：下载时把**解析前**的原始 JSON 字节 tee 给应用。
#: 只在配置阶段调用，**允许阻塞**（典型用法是写 Flash）；返回非 0 = 放弃 tee。
#: 参数：``(ctx, data, len, offset, user)``。
#: ⚠ ``data`` 只在回调期间有效 —— 必须立即复制，不能存地址。
DESC_RAW_SINK_FN = ctypes.CFUNCTYPE(c_int, c_void_p, c_void_p, c_size_t,
                                    c_uint32, c_void_p)

#: ``jsdk_desc_progress_fn``：下载进度；配置阶段调用。
DESC_PROGRESS_FN = ctypes.CFUNCTYPE(None, c_void_p, c_uint32, c_uint32, c_void_p)

#: ``jsdk_fault_callback_t``：故障**边沿**回调（0→1 只报一次）。
#: ⚠ 它在 ``poll()`` / ``cycle_end()``（控制路径）里被调用：回调里不要做重活，
#:   且**不得**重入 SDK。硬实时场景请改用轮询 ``is_fault()`` / ``fault_info()``。
FAULT_CALLBACK_FN = ctypes.CFUNCTYPE(None, c_void_p, POINTER(FaultInfo), c_void_p)


# 名称 → ctypes 类型的映射，用于自检
TYPE_MAP: dict[str, type] = {
    "jsdk_can_frame_t": CanFrame,
    "jsdk_can_hal_t": CanHal,
    "jsdk_context_config_t": ContextConfig,
    "jsdk_desc_config_t": DescConfig,
    "jsdk_joint_config_t": JointConfig,
    "jsdk_joint_config_snapshot_t": JointConfigSnapshot,
    "jsdk_joint_feedback_t": JointFeedback,
    "jsdk_bus_state_t": BusState,
    "jsdk_device_info_t": DeviceInfo,
    "jsdk_fault_info_t": FaultInfo,
    "jsdk_value_t": Value,
    "jsdk_unit_scale_t": UnitScale,
    "jsdk_param_req_t": ParamReq,
    "jsdk_group_target_t": GroupTarget,
    "jsdk_desc_info_t": DescInfo,
}


class _AlignedProbe(ctypes.Structure):
    """``char`` 后跟一个 ``T``：``offsetof(t)`` 即 ``T`` 的对齐。"""


def _alignment_of(cls: type) -> int:
    probe = type(
        "_Probe_" + cls.__name__,
        (ctypes.Structure,),
        {"_fields_": [("c", ctypes.c_char), ("t", cls)]},
    )
    return getattr(probe, "t").offset


# ==========================================================================
# 库定位与加载
# ==========================================================================

_LIB_CANDIDATES = {
    "windows": ["jsdk_can.dll", "libjsdk_can.dll"],
    "darwin": ["libjsdk_can.dylib", "jsdk_can.dylib"],
    "linux": ["libjsdk_can.so", "libjsdk_can.so.0", "jsdk_can.so"],
}


def _platform_key() -> str:
    if sys.platform.startswith("win"):
        return "windows"
    if sys.platform == "darwin":
        return "darwin"
    return "linux"


def library_search_path() -> list[str]:
    """返回按优先级排列的候选路径（含目录与文件名混合）。

    顺序：
      1. ``JSDK_LIB_PATH``（**最高优先级**）：可以是文件，也可以是目录；
      2. 本包自带的 ``lib/`` 子目录（wheel 内随包分发）；
      3. 仓库内的 CMake 构建目录（开发用：``build/`` / ``build-shared/`` …）；
      4. 空串 —— 交给操作系统加载器（PATH / LD_LIBRARY_PATH / DYLD_*）。
    """
    names = _LIB_CANDIDATES[_platform_key()]
    out: list[str] = []

    env = os.environ.get("JSDK_LIB_PATH", "").strip()
    if env:
        if os.path.isdir(env):
            out.extend(os.path.join(env, n) for n in names)
        else:
            out.append(env)

    here = os.path.dirname(os.path.abspath(__file__))
    lib = os.path.join(here, "lib")
    out.extend(os.path.join(lib, n) for n in names)

    # ⚠ **兜底 glob**（B9）：包内 lib/ 里到底叫什么名字，不能假设。
    #   真实事故：CMake 在 Linux 上拷进去的是 `libjsdk_can.so.0.1.0`
    #   （`$<TARGET_FILE:>` 给的是带完整版本号的实体文件），而候选表里只有
    #   `libjsdk_can.so` / `.so.0` → **包自带库却找不到**，Windows 因为 dll 不带
    #   版本号而一直没暴露。这里把 lib/ 下所有 `libjsdk_can.*` / `jsdk_can.*`
    #   都列上（已存在的才列），于是“换个版本号就找不到”这类问题不再可能。
    if os.path.isdir(lib):
        out.extend(sorted(glob.glob(os.path.join(lib, "libjsdk_can.*"))
                          + glob.glob(os.path.join(lib, "jsdk_can.*"))))

    # 开发布局：bindings/python/src/jsdk_can/_abi.py → 仓库根
    root = os.path.abspath(os.path.join(here, "..", "..", "..", ".."))
    for sub in ("build", "build-shared", os.path.join("build", "Release")):
        out.extend(os.path.join(root, sub, n) for n in names)

    out.extend(names)
    # 去重但**保持顺序**（glob 可能把显式候选又列一遍；重复尝试只是浪费时间）
    seen: set[str] = set()
    uniq: list[str] = []
    for p in out:
        if p not in seen:
            seen.add(p)
            uniq.append(p)
    return uniq


_CACHED_LIB: ctypes.CDLL | None = None


def lib() -> ctypes.CDLL:
    """进程内**缓存**的库句柄。

    给不需要上下文的纯函数用（例如 :func:`jsdk_can.units.unit_scale_calc`）——
    ``load_library()`` 本身不缓存，每次都 dlopen 一遍；在热路径上调用它的
    包装函数会白白重复加载。
    """
    global _CACHED_LIB
    if _CACHED_LIB is None:
        _CACHED_LIB = load_library()
    return _CACHED_LIB


def load_library(path: str | None = None) -> ctypes.CDLL:
    """加载共享库。``path`` 为 None 时按 :func:`library_search_path` 依次尝试。"""
    from .errors import LibraryNotFoundError

    tried: list[str] = []
    candidates = [path] if path else library_search_path()

    for cand in candidates:
        try:
            lib = ctypes.CDLL(cand)
        except OSError as exc:  # pragma: no cover - 取决于机器
            tried.append(f"{cand or '(系统加载器)'}: {exc}")
            continue
        _initialise(lib)
        return lib

    raise LibraryNotFoundError(
        "找不到 libjsdk_can（共享库）。请用 -DJSDK_BUILD_SHARED=ON 构建，"
        "或用 JSDK_LIB_PATH 指向库文件/目录。\n已尝试：\n  "
        + "\n  ".join(tried)
    )


# ==========================================================================
# 函数签名绑定
# ==========================================================================

_FUNCS: dict[str, tuple[list, object]] = {
    # --- ABI ---
    "jsdk_backend_name": ([], c_char_p),
    "jsdk_abi_version": ([], c_uint32),
    "jsdk_abi_types": ([POINTER(c_size_t)], POINTER(AbiType)),
    "jsdk_context_size": ([c_void_p], c_size_t),
    "jsdk_desc_arena_size": ([POINTER(DescConfig)], c_size_t),
    # --- HAL 工厂 ---
    "jsdk_hal_virtual_open": ([POINTER(CanHal), POINTER(c_void_p), c_char_p], c_int),
    "jsdk_hal_virtual_inject": ([c_void_p, POINTER(CanFrame)], c_int),
    "jsdk_hal_virtual_capture": ([c_void_p, POINTER(CanFrame)], c_int),
    "jsdk_hal_virtual_set_tx_fail": ([c_void_p, c_int], None),
    "jsdk_hal_virtual_dropped": ([c_void_p], c_uint32),
    "jsdk_hal_virtual_advance_ms": ([c_void_p, c_uint32], None),
    "jsdk_hal_virtual_set_autotick": ([c_void_p, c_int], None),
    "jsdk_hal_socketcan_open": ([POINTER(CanHal), POINTER(c_void_p), c_char_p,
                                 c_uint32, c_uint32], c_int),
    "jsdk_hal_pcan_open": ([POINTER(CanHal), POINTER(c_void_p), c_char_p,
                            c_uint32, c_uint32], c_int),
    "jsdk_hal_slcan_open": ([POINTER(CanHal), POINTER(c_void_p), c_char_p,
                             c_uint32, c_uint32], c_int),
    "jsdk_hal_slcan_supports_fd": ([], c_int),
    "jsdk_hal_slcan_fd_config": ([c_void_p, POINTER(c_int),
                                  POINTER(c_uint32)], None),
    "jsdk_hal_slcan_fd_frames": ([c_void_p, POINTER(c_uint32),
                                  POINTER(c_uint32)], None),
    "jsdk_hal_slcan_stats": ([c_void_p, POINTER(c_uint32), POINTER(c_uint32),
                              POINTER(c_uint32), POINTER(c_uint32),
                              POINTER(c_uint32)], None),
    "jsdk_hal_close": ([c_void_p], c_int),
    # --- 生命周期 ---
    "jsdk_context_config_default": ([POINTER(ContextConfig)], None),
    "jsdk_context_init": ([c_void_p, POINTER(ContextConfig)], c_int),
    "jsdk_context_create": ([POINTER(ContextConfig)], c_void_p),   # 堆模式（需库开了 heap）
    "jsdk_context_free": ([c_void_p], None),
    "jsdk_context_add_joint": ([c_void_p, POINTER(JointConfig),
                                POINTER(c_void_p)], c_int),
    "jsdk_context_configure": ([c_void_p], c_int),
    "jsdk_context_activate": ([c_void_p], c_int),
    "jsdk_context_deactivate": ([c_void_p], None),
    "jsdk_context_destroy": ([c_void_p], None),
    # --- 循环 ---
    "jsdk_context_cycle_begin": ([c_void_p, c_uint64], c_int),
    "jsdk_context_cycle_end": ([c_void_p], c_int),
    "jsdk_context_poll": ([c_void_p, c_uint64], c_int),
    # --- 总线 ---
    "jsdk_context_discover": ([c_void_p, POINTER(c_uint8), c_int,
                               POINTER(c_int), c_uint8], c_int),
    "jsdk_context_get_bus_state": ([c_void_p, POINTER(BusState)], c_int),
    "jsdk_context_last_error": ([c_void_p], c_char_p),
    "jsdk_context_estop": ([c_void_p], None),
    # --- 关节反馈/控制 ---
    "jsdk_joint_get_feedback": ([c_void_p, POINTER(JointFeedback)], c_int),
    "jsdk_joint_is_enabled": ([c_void_p], c_int),
    "jsdk_joint_is_fault": ([c_void_p], c_int),
    "jsdk_joint_get_mode_state": ([c_void_p], c_int),
    "jsdk_joint_get_can_state": ([c_void_p], c_int),
    "jsdk_joint_clear_status_flags": ([c_void_p, c_uint16], None),
    "jsdk_joint_request_enable": ([c_void_p, c_int], None),
    "jsdk_joint_request_disable": ([c_void_p], None),
    "jsdk_joint_request_fault_reset": ([c_void_p], None),
    "jsdk_joint_set_mode": ([c_void_p, c_int], None),
    "jsdk_joint_set_target_position_rad": ([c_void_p, c_double], None),
    "jsdk_joint_set_target_velocity_rad_s": ([c_void_p, c_double], None),
    "jsdk_joint_set_target_torque_Nm": ([c_void_p, c_double], None),
    "jsdk_joint_set_mit": ([c_void_p, c_double, c_double, c_double,
                            c_double, c_double], None),
    "jsdk_joint_set_mit_stiffness": ([c_void_p, c_double, c_double,
                                      c_double, c_double, c_double], None),
    "jsdk_joint_set_limits": ([c_void_p, c_double, c_double], None),
    "jsdk_joint_set_current_A": ([c_void_p, c_double], None),
    "jsdk_joint_set_target_position": ([c_void_p, c_int32], None),
    "jsdk_joint_set_target_velocity": ([c_void_p, c_int32], None),
    "jsdk_joint_set_target_torque": ([c_void_p, c_int16], None),
    "jsdk_joint_set_scale": ([c_void_p, POINTER(UnitScale)], None),
    "jsdk_joint_get_scale": ([c_void_p, POINTER(UnitScale)], None),
    "jsdk_unit_scale_default": ([POINTER(UnitScale), c_uint32], None),
    "jsdk_unit_scale_calc": ([POINTER(UnitScale), c_uint32, c_uint32, c_uint32,
                              c_uint32], None),
    "jsdk_joint_hold_position": ([c_void_p], None),
    "jsdk_joint_hold_position_pd": ([c_void_p, c_double, c_double], None),
    # --- 运维 ---
    "jsdk_context_set_fault_callback": ([c_void_p, c_void_p, c_void_p], None),
    "jsdk_joint_set_zero_here": ([c_void_p], c_int),
    "jsdk_joint_calibrate": ([c_void_p], c_int),
    "jsdk_joint_home": ([c_void_p], c_int),
    "jsdk_joint_save_config": ([c_void_p], c_int),
    "jsdk_joint_reset_device": ([c_void_p], c_int),
    "jsdk_joint_set_node_id": ([c_void_p, c_uint8, c_int], c_int),
    "jsdk_joint_set_watchdog_ms": ([c_void_p, c_uint32], c_int),
    "jsdk_joint_get_device_info": ([c_void_p, POINTER(DeviceInfo)], c_int),
    "jsdk_joint_read_config_snapshot": ([c_void_p, POINTER(JointConfigSnapshot)],
                                        c_int),
    # --- 故障 ---
    "jsdk_joint_get_fault_info": ([c_void_p, POINTER(FaultInfo)], c_int),
    "jsdk_joint_query_error_detail": ([c_void_p, POINTER(FaultInfo)], c_int),
    "jsdk_joint_error_string": ([c_uint8], c_char_p),
    "jsdk_axis_error_first": ([c_uint32, POINTER(c_int)], c_char_p),
    "jsdk_axis_error_bit_name": ([c_int], c_char_p),
    "jsdk_hb_error_bit_name": ([c_int], c_char_p),
    "jsdk_can_axis_state_name": ([c_uint8], c_char_p),
    "jsdk_joint_describe_fault": ([c_void_p, POINTER(ctypes.c_char), c_size_t],
                                  c_int),
    # --- 参数 ---
    "jsdk_joint_param_get": ([c_void_p, c_char_p, POINTER(Value)], c_int),
    "jsdk_joint_param_set": ([c_void_p, c_char_p, POINTER(Value)], c_int),
    "jsdk_joint_param_get_batch": ([c_void_p, POINTER(ParamReq), c_int], c_int),
    "jsdk_joint_param_get_f32": ([c_void_p, c_char_p, POINTER(c_float)], c_int),
    "jsdk_joint_param_set_f32": ([c_void_p, c_char_p, c_float], c_int),
    "jsdk_joint_param_get_u32": ([c_void_p, c_char_p, POINTER(c_uint32)], c_int),
    "jsdk_joint_param_set_u32": ([c_void_p, c_char_p, c_uint32], c_int),
    "jsdk_joint_param_get_i32": ([c_void_p, c_char_p, POINTER(c_int32)], c_int),
    "jsdk_joint_param_get_bool": ([c_void_p, c_char_p, POINTER(c_int)], c_int),
    # --- 参数：SDO 风格槽位（可拿裸字节 + 轮询状态机）---
    "jsdk_joint_sdo_create": ([c_void_p, c_uint16, c_uint8, c_size_t], c_int),
    "jsdk_joint_sdo_create_by_name": ([c_void_p, c_char_p], c_int),
    "jsdk_joint_sdo_state": ([c_void_p, c_int], c_int),
    "jsdk_joint_sdo_data": ([c_void_p, c_int], c_void_p),
    "jsdk_joint_sdo_data_size": ([c_void_p, c_int], c_size_t),
    "jsdk_joint_sdo_read": ([c_void_p, c_int], c_int),
    "jsdk_joint_sdo_write": ([c_void_p, c_int], c_int),
    # --- 分组 ---
    "jsdk_group_set_mit": ([c_void_p, POINTER(GroupTarget), c_int], c_int),
    "jsdk_group_enable": ([c_void_p, POINTER(c_uint8), c_int], c_int),
    "jsdk_group_disable": ([c_void_p, POINTER(c_uint8), c_int], c_int),
    # --- 描述符 ---
    "jsdk_context_get_desc_info": ([c_void_p, POINTER(DescInfo)], c_int),
    "jsdk_context_desc_fetch": ([c_void_p], c_int),
    "jsdk_context_desc_poll": ([c_void_p, c_uint64], c_int),
    "jsdk_context_desc_import_raw": ([c_void_p, c_void_p, c_size_t,
                                       POINTER(DescHint)], c_int),
    # ⚠ 回调参数声明成 `c_void_p`（而不是 CFUNCTYPE 类）：C API 允许传 NULL 取消注册，
    #   而 ctypes 对 CFUNCTYPE 类声明的参数**不接受 None**（ArgumentError:
    #   expected CFunctionType instance instead of NoneType）。调用方用
    #   `ctypes.cast(fn, c_void_p)` 传真实回调 —— 类型检查少一点，但“能取消”
    #   对这几个 setter 是硬需求。
    "jsdk_context_set_desc_raw_sink": ([c_void_p, c_void_p, c_void_p], None),
    "jsdk_context_set_desc_progress": ([c_void_p, c_void_p, c_void_p], None),
    "jsdk_endpoint_lookup": ([c_void_p, c_char_p, POINTER(c_uint16),
                              POINTER(c_int), POINTER(c_uint8)], c_int),
    "jsdk_endpoint_enumerate": ([c_void_p, ENDPOINT_VISIT_FN, c_void_p], c_int),
    "jsdk_desc_export_max_size": ([c_void_p], c_size_t),
    "jsdk_context_desc_export": ([c_void_p, c_void_p, c_size_t,
                                  POINTER(c_size_t)], c_int),
    "jsdk_context_desc_import": ([c_void_p, c_void_p, c_size_t], c_int),
    # --- 文本 ---
    "jsdk_status_string": ([c_int], c_char_p),
    "jsdk_axis_state_string": ([c_int], c_char_p),
    "jsdk_mode_string": ([c_int], c_char_p),
    "jsdk_ep_type_string": ([c_int], c_char_p),
}


#: 绑定**必须**能找到的函数。缺任何一个都说明"库不是这套绑定对应的库"
#: （例如指向了旧版本的 DLL）。其余函数按需使用，缺失不影响基本功能。
_REQUIRED_FUNCS = (
    "jsdk_backend_name",       # 后端自报家门
    "jsdk_abi_version",        # 版本
    "jsdk_abi_types",          # 结构体布局探针（WP8 新增）
    "jsdk_context_size",
    "jsdk_context_init",
    "jsdk_context_destroy",
    "jsdk_context_cycle_begin",
    "jsdk_context_cycle_end",
    "jsdk_context_add_joint",
    "jsdk_context_configure",
    # --- A13：以下符号也是 0.1.0 公开 ABI 的一部分（无条件编译进共享库），
    #     所以“库比绑定旧”必须当场报出来，而不是等用户调到才 AttributeError。
    #     ⚠ **不包含**需要特殊构建开关的少数：`jsdk_context_create/free`
    #     （JSDK_ENABLE_HEAP）与 `jsdk_hal_slcan_*`（要编 slcan 后端）——
    #     它们在各自的封装里按需探测。
    "jsdk_context_desc_fetch",
    "jsdk_context_desc_poll",
    "jsdk_context_desc_import_raw",
    "jsdk_context_set_desc_raw_sink",
    "jsdk_context_set_desc_progress",
    "jsdk_context_set_fault_callback",
    "jsdk_joint_sdo_create",
    "jsdk_joint_sdo_create_by_name",
    "jsdk_joint_sdo_state",
    "jsdk_joint_sdo_data",
    "jsdk_joint_sdo_data_size",
    "jsdk_joint_sdo_read",
    "jsdk_joint_sdo_write",
    "jsdk_unit_scale_default",
    "jsdk_unit_scale_calc",
    "jsdk_joint_set_scale",
    "jsdk_joint_get_scale",
)


def _initialise(lib: ctypes.CDLL) -> None:
    """给库对象装上签名。

    ⚠ 缺失的函数不能一律静默跳过：老库（早于 ABI 探针的版本）少了
    ``jsdk_abi_types``，静默跳过后会在 ``check_abi()`` 里变成一句莫名其妙的
    ``AttributeError``，把"版本不匹配"误报成"绑定有 bug"。
    这里显式列举必需符号，缺了就报出**缺了哪些**。
    """
    missing: list[str] = []
    for name, (argtypes, restype) in _FUNCS.items():
        try:
            fn = getattr(lib, name)
        except AttributeError:
            missing.append(name)
            continue
        fn.argtypes = argtypes
        fn.restype = restype

    hard = [n for n in _REQUIRED_FUNCS if n in missing]
    if hard:
        from .errors import AbiMismatchError

        raise AbiMismatchError(
            "加载到的库缺少本绑定必需的函数：\n  - " + "\n  - ".join(hard)
            + "\n这几乎总是因为库版本比绑定旧（或指错了文件）。"
            "请用 -DJSDK_BUILD_SHARED=ON 重新构建，或用 JSDK_LIB_PATH "
            "指向配套的 libjsdk_can。"
        )


# ==========================================================================
# ABI 自检
# ==========================================================================


def check_abi(lib: ctypes.CDLL) -> dict[str, tuple[int, int]]:
    """比对 ctypes 布局与 C 侧 ``jsdk_abi_types()``。

    :raises AbiMismatchError: 任一类型尺寸或对齐不一致
    :return: ``{类型名: (size, align)}``
    """
    from .errors import AbiMismatchError

    n = c_size_t()
    table = lib.jsdk_abi_types(byref(n))
    if not table:
        raise AbiMismatchError("jsdk_abi_types() 返回空指针")

    result: dict[str, tuple[int, int]] = {}
    problems: list[str] = []

    for i in range(n.value):
        entry = table[i]
        name = entry.name.decode() if entry.name else "<unnamed>"
        result[name] = (entry.size, entry.align)

        py = TYPE_MAP.get(name)
        if py is None:
            problems.append(f"{name}: C 侧有此类型，ctypes 未定义（绑定过旧）")
            continue
        ssize, salign = ctypes.sizeof(py), _alignment_of(py)
        if ssize != entry.size or salign != entry.align:
            problems.append(
                f"{name}: C 侧 size={entry.size} align={entry.align}，"
                f"ctypes size={ssize} align={salign}"
            )

    if problems:
        raise AbiMismatchError(
            "libjsdk_can 与 Python 绑定的结构体布局不一致（ABI 不匹配）。\n"
            "这**不能**忽略：布局不同会直接踩内存。请用与绑定同版本的库，"
            "或用 -DJSDK_BUILD_SHARED=ON 重新构建。\n  - " + "\n  - ".join(problems)
        )

    backend = lib.jsdk_backend_name()
    if backend and backend != b"cyberbeast-can":
        raise AbiMismatchError(
            f"加载到的库是另一个 jsdk 后端：{backend!r}（本绑定只支持 "
            "'cyberbeast-can'）。家族里三个后端符号同名、互斥链接，"
            "混用会导致静默的内存错配。"
        )
    return result


# ==========================================================================
# 对齐分配
# ==========================================================================

_ALIGN = 16  # 足够覆盖所有公开结构体（含 8 字节指针/双精度）


def alloc_aligned(nbytes: int) -> tuple[ctypes.Array, ctypes.c_void_p]:
    """分配至少 ``nbytes`` 字节、**16 字节对齐**的内存。

    ``ctypes.create_string_buffer()`` 只保证 1 字节对齐，拿它当 ``jsdk_context_t``
    或描述符 arena 用是未定义行为（arena 里存指针，上下文里有 double）。
    这里用 ``c_uint64`` 数组实现对齐，再返回 ``(buffer, void_ptr)``。
    """
    if nbytes <= 0:
        raise ValueError("nbytes 必须为正")
    words = (nbytes + 7) // 8
    buf = (ctypes.c_uint64 * words)()
    ptr = ctypes.cast(buf, c_void_p)
    addr = ctypes.addressof(ctypes.cast(ptr, POINTER(ctypes.c_char)).contents)
    if addr % 8 != 0:  # pragma: no cover - 正常平台不会发生
        raise RuntimeError(f"分配到的内存没有 8 字节对齐（addr={addr:#x}）")
    return buf, ptr
