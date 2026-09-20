"""slcan 后端的 CAN FD 支持。

背景（这条曾经写错过，值得留个用例）：slcan 的 FD 表述来自 **CANable 2.0 固件**
对 Lawicel 协议的扩展 —— 帧前缀 `d/D`（BRS=0）与 `b/B`（BRS=1），DLC 位是 FD
**长度码**。本 SDK 早期版本把 slcan 判定为"不支持 FD"，**是错的**。

Python 侧只能测到"参数校验"与"能力自报"（真机收发要硬件）：
  1. `SlcanHal` 默认按 FD 5 Mbps 配置，且能报出"已配置 / 已设置的速率"；
  2. 表外的数据段速率必须在**打开串口之前**就被拒（所以没有硬件也能断言）；
  3. `data_bitrate=0` = 不碰适配器配置，此时不报"不支持"；
  4. C 侧 `jsdk_hal_slcan_supports_fd()` 必须为 1。
"""

from __future__ import annotations

import ctypes

import pytest

import jsdk_can
from jsdk_can import JsdkError, JsdkUnsupportedError, SlcanHal
from jsdk_can import _abi

#: 一定不存在的串口：本模块只验证"参数校验发生在打开串口之前"，
#: 所以这些用例不需要（也不该需要）真硬件。
NO_SUCH_PORT = "JSDK_NO_SUCH_PORT"


def _open(hal: SlcanHal):
    """走真实的 C 路径（`attach` = 加载库 + 打开后端）。"""
    hal.attach(jsdk_can.load_library())
    return hal


def test_c_side_reports_fd_support():
    """C 侧能力自报：slcan **支持** FD。"""
    lib = jsdk_can.load_library()
    assert lib.jsdk_hal_slcan_supports_fd() == 1


def test_fd_functions_are_exported():
    """绑定要调的三个诊断函数必须在导出表里（`JSDK_API` 别漏）。"""
    lib = jsdk_can.load_library()
    for name in ("jsdk_hal_slcan_supports_fd", "jsdk_hal_slcan_fd_config",
                 "jsdk_hal_slcan_fd_frames", "jsdk_hal_slcan_stats"):
        assert hasattr(lib, name), f"{name} 未导出"


def test_slcan_hal_default_is_fd_5m():
    """默认就是协议默认的 FD 数据段速率 5 Mbps（不是"不支持 FD"）。"""
    hal = SlcanHal("COM_TEST")
    assert hal.supports_fd is True
    assert hal.data_bitrate == 5_000_000


def test_unsupported_data_bitrate_is_rejected_before_port_open():
    """表外速率 → `JsdkUnsupportedError`，且**不提串口**。

    这条能在无硬件机器上跑，正是因为它证明了校验顺序：
    如果先开串口，我们会得到"端口不存在"（INVALID_ARG），
    那现场遇到"速率不支持"就会被误导去查线。
    """
    hal = SlcanHal(NO_SUCH_PORT, data_bitrate=3_000_000)
    with pytest.raises(JsdkUnsupportedError) as ei:
        _open(hal)
    assert ei.value.op.startswith("slcan_open")


def test_only_known_data_rates_are_accepted():
    """2 Mbps / 5 Mbps 是公认的两项；其余（含 1 Mbps、8 Mbps）都不接受。"""
    for bad in (1_000_000, 4_000_000, 8_000_000, 12_500_000):
        hal = SlcanHal(NO_SUCH_PORT, data_bitrate=bad)
        with pytest.raises(JsdkUnsupportedError):
            _open(hal)


def test_zero_data_bitrate_does_not_touch_adapter():
    """`data_bitrate=0` = 不碰适配器配置 → 不该报"不支持"，
    只会在真正打开串口时因端口不存在而失败。"""
    hal = SlcanHal(NO_SUCH_PORT, data_bitrate=0)
    with pytest.raises(JsdkError) as ei:
        _open(hal)
    assert not isinstance(ei.value, JsdkUnsupportedError)
    assert ei.value.status in (jsdk_can.Status.INVALID_ARG,
                               jsdk_can.Status.NOT_FOUND,
                               jsdk_can.Status.TRANSPORT)


def test_fd_config_and_frame_counters_are_callable():
    """诊断函数能在句柄上跑（虚拟端口开不了，这里用假句柄验证签名不会炸）。

    真正的计数要在真机上才有意义；这里只保证 ctypes 签名与 C 侧一致
    （签名写错会在这里段错误或报 TypeError，而不是等到客户现场）。
    """
    enabled = ctypes.c_int(-1)
    bitrate = ctypes.c_uint32(0xFFFFFFFF)
    tx = ctypes.c_uint32(0xFFFFFFFF)
    rx = ctypes.c_uint32(0xFFFFFFFF)

    lib = jsdk_can.load_library()
    # NULL 句柄：C 侧必须安静地什么都不做（不是崩溃）
    lib.jsdk_hal_slcan_fd_config(None, ctypes.byref(enabled),
                                 ctypes.byref(bitrate))
    lib.jsdk_hal_slcan_fd_frames(None, ctypes.byref(tx), ctypes.byref(rx))
    lib.jsdk_hal_slcan_stats(None, ctypes.byref(tx), ctypes.byref(rx),
                             ctypes.byref(tx), ctypes.byref(tx),
                             ctypes.byref(tx))
    # 未被改写 = 函数真的判断了句柄，而不是写出垃圾
    assert enabled.value == -1
    assert bitrate.value == 0xFFFFFFFF
    assert tx.value == 0xFFFFFFFF


def test_signature_matches_five_args():
    """`jsdk_hal_slcan_open` 现在收 5 个参数（port/baud/data_bitrate）。

    ctypes 的 `argtypes` 里少一个参数时，调用会**静默**把栈上的垃圾当第五个
    实参 —— 于是 `data_bitrate` 变成随机值、适配器被设成随机速率。
    这里直接核对 `_FUNCS` 里的声明。
    """
    args, _restype = _abi._FUNCS["jsdk_hal_slcan_open"]
    assert len(args) == 5
    assert args[-1] is ctypes.c_uint32
