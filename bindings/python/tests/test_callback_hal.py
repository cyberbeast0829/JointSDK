"""插件式（回调）HAL 与 gs_usb 后端。

背景
----
`jsdk_can` 过去**没有**“自己实现传输”这条路：内置后端全是 C 侧实现，Python 只调
`jsdk_hal_*_open()`。而 gs_usb（candleLight）在 C 侧没有后端 ⇒ 在 Windows 上
完全没法用。`CallbackHal` 就是为此加的那条路（把 **Python 侧**的传输用 ctypes
回调接进 `jsdk_can_hal_t`）。

本模块**不需要硬件**（真的收发用 `tools/gsusb_selftest.py`）。它盯的是几件
容易出事、且出事就很难查的事：

  1. 函数表真的被填了（四个回调 + `on_error` 故意留 NULL）；
  2. 两个方向的**编解码**（C `jsdk_can_frame_t` ↔ Python）；
  3. **生命周期**：回调对象必须被实例持有 —— `jsdk_context_init()` 只复制结构体，
     回调被 GC 之后 C 侧下一次调用就是跳转到已释放内存（本仓已有同类事故）；
  4. 端到端：SDK 真的会**经回调式 HAL 发帧**（用 `estop`：它同步走 `send`，
     不排队、不需要设备应答）；
  5. `supports_fd=False` 的后端，`Context` 必须默认按 Classic —— 否则 SDK 会按
     FD 打包，而 gs_usb 的 Python 通路根本发不出去。
"""

from __future__ import annotations

import collections
import ctypes

import pytest

import jsdk_can
from jsdk_can import Context, JsdkError, Mode
from jsdk_can import _abi
from jsdk_can.hal import CallbackHal, GsusbHal, _hal_lookup


class FakeHal(CallbackHal):
    """最小可用的“自实现传输”：发出去的帧记下来，收不下任何东西。"""

    name = "fake"
    supports_fd = False

    def __init__(self) -> None:
        super().__init__()
        self.sent: list = []
        self.inbox: collections.deque = collections.deque()
        self.bus_flags = _abi.JSDK_HAL_BUS_OK

    def _py_open(self) -> None:
        pass

    def _py_close(self) -> None:
        pass

    def py_send(self, frame) -> int:
        self.sent.append((int(frame.id), int(frame.len),
                          bytes(int(frame.data[i]) for i in range(int(frame.len)))))
        return 0

    def py_recv(self, frame) -> int:
        if not self.inbox:
            return 0
        fid, flen, fdata = self.inbox.popleft()
        frame.id = fid
        frame.len = flen
        frame.flags = _abi.JSDK_FRAME_EXT
        for i, b in enumerate(fdata):
            frame.data[i] = b
        return 1

    def py_bus_flags(self) -> int:
        return self.bus_flags


def _opened(hal: CallbackHal) -> CallbackHal:
    """走真实的 C 路径（加载库 + `attach` 填函数表）。"""
    hal.attach(jsdk_can.load_library())
    return hal


# ==========================================================================
# 1. 函数表
# ==========================================================================

def test_function_table_is_filled():
    hal = FakeHal()
    assert not hal.storage.send, "attach 之前不该有函数表"
    _opened(hal)
    for name in ("send", "recv", "now_ms", "bus_status"):
        assert getattr(hal.storage, name), f"storage.{name} 没被填上"
    # on_error 故意留空：后端拿不到调用方的 on_error（结构体在 init 时被复制），
    # 所以一律用“锁存 + bus_status 上报”，与 hal_socketcan.c 同一套做法。
    assert not hal.storage.on_error
    assert hal.storage.user, "user token 必须非 0（回调靠它找回 Python 对象）"
    hal.close()


def test_registry_maps_token_to_instance():
    hal = FakeHal()
    assert _hal_lookup(hal.storage.user) is hal
    assert _hal_lookup(0) is None
    assert _hal_lookup(999999) is None


def test_callback_objects_are_held_by_instance():
    """防的是“CFUNCTYPE 被 GC → C 侧跳转到已释放内存”那类事故。"""
    hal = FakeHal()
    _opened(hal)
    cbs = hal._cbs
    assert len(cbs) == 4
    assert all(cb is not None for cb in cbs)
    hal.close()


# ==========================================================================
# 2. 两个方向的编解码
# ==========================================================================

def test_send_callback_reads_c_frame():
    hal = FakeHal()
    _opened(hal)
    frame = _abi.CanFrame()
    frame.id = 0x1234567
    frame.len = 3
    frame.flags = _abi.JSDK_FRAME_EXT
    frame.data[0], frame.data[1], frame.data[2] = 0xAA, 0xBB, 0xCC

    rc = hal._cb_send(hal.storage.user, ctypes.pointer(frame))
    assert rc == 0
    assert hal.sent == [(0x1234567, 3, bytes([0xAA, 0xBB, 0xCC]))]
    hal.close()


def test_recv_callback_writes_c_frame():
    hal = FakeHal()
    _opened(hal)
    hal.inbox.append((0x0ABCDEF, 2, bytes([0x11, 0x22])))

    frame = _abi.CanFrame()
    rc = hal._cb_recv(hal.storage.user, ctypes.pointer(frame))
    assert rc == 1
    assert frame.id == 0x0ABCDEF
    assert frame.len == 2
    assert frame.flags == _abi.JSDK_FRAME_EXT
    assert bytes(frame.data[:2]) == bytes([0x11, 0x22])
    # 队列空了必须报 0（契约：0 = 当前无帧，不是错误）
    assert hal._cb_recv(hal.storage.user, ctypes.pointer(frame)) == 0
    hal.close()


def test_now_ms_callback_is_monotonic():
    hal = FakeHal()
    a = hal._cb_now_ms(None)
    b = hal._cb_now_ms(None)
    assert isinstance(a, int) and isinstance(b, int)
    assert 0 <= a <= b
    assert 0 <= b < 2 ** 32


def test_bus_status_callback_writes_flags():
    hal = FakeHal()
    _opened(hal)
    hal.bus_flags = _abi.JSDK_HAL_BUS_OK | _abi.JSDK_HAL_BUS_ERROR_WARN
    out = ctypes.c_uint32(0)
    assert hal._cb_bus_status(hal.storage.user, ctypes.pointer(out)) == 0
    assert int(out.value) == hal.bus_flags
    hal.close()


def test_callback_exception_does_not_propagate():
    """回调里抛异常会**穿过 C 栈** —— 必须被吞掉并如实返回失败。"""

    class Boom(FakeHal):
        def py_send(self, frame) -> int:
            raise RuntimeError("boom")

    hal = Boom()
    _opened(hal)
    frame = _abi.CanFrame()
    assert hal._cb_send(hal.storage.user, ctypes.pointer(frame)) == -1
    hal.close()


# ==========================================================================
# 3. 端到端：SDK → 回调 → Python
# ==========================================================================

def test_estop_reaches_the_python_hal():
    """`jsdk_context_estop()` 同步走 `hal.send`，是最确定的“SDK 真的发了”证据。"""
    hal = FakeHal()
    ctx = Context(hal, master_id=1, is_fd=False)
    ctx.add_joint(1, mode=Mode.MIT)

    ctx.estop()

    assert hal.sent, "SDK 没有经回调式 HAL 发出任何帧"
    fid, flen, fdata = hal.sent[0]
    # 29-bit 布局：[28:26] pri | [25:18] msgtype | [17:10] dest | [9:2] src | [1:0] seq
    msgtype = (fid >> 18) & 0xFF
    dest = (fid >> 10) & 0xFF
    assert msgtype == 0xC0, f"estop 的 MsgType 应为 0xC0，实际 0x{msgtype:02x}"
    assert dest == 0xFF, f"estop 是广播，Dest 应为 0xFF，实际 0x{dest:02x}"
    assert fid <= 0x1FFFFFFF
    assert flen <= 8
    ctx.close()


def test_context_honours_supports_fd_false():
    """`supports_fd=False` 的后端，`is_fd` 未指定时必须落到 Classic。"""
    classic_only = FakeHal()          # supports_fd = False
    ctx = Context(classic_only, master_id=1)
    assert ctx.cfg.is_fd == 0, "不支持 FD 的后端不该默认按 FD 打包"
    ctx.close()

    class FdHal(FakeHal):
        supports_fd = True

    fd = FdHal()
    ctx2 = Context(fd, master_id=1)
    assert ctx2.cfg.is_fd == 1
    ctx2.close()


def test_explicit_is_fd_still_wins():
    hal = FakeHal()
    ctx = Context(hal, master_id=1, is_fd=True)   # 调用方显式要 FD
    assert ctx.cfg.is_fd == 1
    ctx.close()


# ==========================================================================
# 4. GsusbHal：参数解析与“没有硬件时也要说人话”
# ==========================================================================

def test_gsusb_channel_parsing():
    assert GsusbHal._parse_channel("0") == (0, None, None)
    assert GsusbHal._parse_channel("2") == (2, None, None)
    assert GsusbHal._parse_channel("1:4") == (None, 1, 4)
    assert GsusbHal._parse_channel("") == (0, None, None)


def test_gsusb_is_classic_only():
    """python-can 的 gs_usb 通路是 CAN 2.0 —— 别对外宣称 FD。"""
    assert GsusbHal.supports_fd is False
    assert GsusbHal.name == "gsusb"


def test_gsusb_open_failure_is_actionable():
    """没有设备（或被 usbipd 交给 WSL）时，错误必须说清“先查什么”。"""
    if GsusbHal.devices():
        pytest.skip("本机有 gs_usb 设备，跳过“没有设备”的分支")
    hal = GsusbHal("0", 1_000_000)
    with pytest.raises(JsdkError) as ei:
        hal.attach(jsdk_can.load_library())
    detail = str(ei.value)
    assert detail, "错误信息不能是空的"
    assert "WinUSB" in detail or "usbipd" in detail or "libusb" in detail
