"""传输后端（HAL）选择。

Python 侧**不**用 ctypes 回调拼 ``jsdk_can_hal_t``（脆弱、慢、且会在 RT 路径上
引入 GIL 抖动）。改为直接调用 C 侧的内置后端工厂，由它把回调填进我们提供的
存储里 —— 见 ``_abi.CanHal``。

每个 :class:`Hal` 负责：
  * 一个 ``CanHal`` 存储（**必须**与上下文同寿命：``jsdk_context_init()`` 只
    复制结构体，厂商后端句柄指针仍指向它，提前释放就是悬垂指针）；
  * 一个后端句柄（``jsdk_hal_close()`` 释放）。
"""

from __future__ import annotations

import collections
import ctypes
import itertools
import threading
import time
from dataclasses import dataclass

from . import _abi
from .enums import Status
from .errors import JsdkUsageError, raise_for_status

__all__ = [
    "Hal",
    "CallbackHal",
    "VirtualHal",
    "SocketCanHal",
    "PcanHal",
    "SlcanHal",
    "GsusbHal",
    "DEFAULT_VIRTUAL_SPEC",
]


#: 虚拟后端默认节点规格：单节点、FD、与仿真设备默认量程一致。
#: 虚拟设备**自带描述符**，所以无需任何测试夹具就能跑通全部只读操作。
DEFAULT_VIRTUAL_SPEC = (
    "0:id=1,gear=16.5,pmax=12.5,vmax=65,tmax=50,kpmax=500,kdmax=5,"
    "hb=10,timeout=30000,fd"
)


class Hal:
    """后端基类。子类只需实现 :meth:`_open`。"""

    #: 后端名（与 ``jsdk-cli --if`` 一致）
    name = "none"
    #: 该后端是否支持 CAN FD
    supports_fd = True

    def __init__(self) -> None:
        self.storage = _abi.CanHal()
        self.handle: ctypes.c_void_p | None = None
        self._lib: ctypes.CDLL | None = None
        #: 由 :meth:`attach` 注入的库（Context 创建时调用）
        self._opened = False

    # --- 生命周期 ---------------------------------------------------------

    def attach(self, lib: ctypes.CDLL) -> None:
        """绑定到具体库并打开后端。由 :class:`~jsdk_can.context.Context` 调用。"""
        self._lib = lib
        self._open()
        self._opened = True

    def _open(self) -> None:  # pragma: no cover - 抽象
        raise NotImplementedError

    def close(self) -> None:
        """关闭后端（幂等）。"""
        if self.handle is not None and self._lib is not None:
            self._lib.jsdk_hal_close(self.handle)
        self.handle = None
        self._opened = False

    # --- 上下文使用 -------------------------------------------------------

    @property
    def hal(self) -> _abi.CanHal:
        """可直接放进 ``jsdk_context_config_t.hal`` 的结构体。"""
        return self.storage

    def require(self, what: str):  # pragma: no cover - 便利断言
        if not self._opened:
            raise JsdkUsageError(-1, what, "HAL 未打开（先创建 Context）")
        return self


class VirtualHal(Hal):
    """虚拟总线 + 驱动器行为模拟器（CI / 离线演示）。

    :param spec: 节点规格，``;`` 分隔多个节点。**起始数字是数组下标，不是
        node_id**（node_id 用 ``id=`` 设，缺省为下标+1）。
        支持的键：``id`` ``gear`` ``tconst`` ``pmax`` ``vmax`` ``kpmax``
        ``kdmax`` ``tmax`` ``hb`` ``timeout`` ``vb`` ``temp``；
        无值键：``fd`` / ``classic`` / ``arm`` / ``disarm`` / ``enabled`` / ``disabled``。
        ``None`` 用 :data:`DEFAULT_VIRTUAL_SPEC`。
    """

    name = "virtual"

    def __init__(self, spec: str | None = None) -> None:
        super().__init__()
        self.spec = spec if spec is not None else DEFAULT_VIRTUAL_SPEC

    def _open(self) -> None:
        handle = ctypes.c_void_p()
        st = self._lib.jsdk_hal_virtual_open(
            ctypes.byref(self.storage), ctypes.byref(handle),
            self.spec.encode("utf-8"))
        raise_for_status(st, "virtual_open")
        self.handle = handle

        # **关键**：让仿真时钟像真实 HAL 一样自由运行。
        # 否则 SDK 里那些"等时间流逝"的阻塞 API（configure / discover /
        # calibrate / home）会在冻结时钟下一直等到内部自旋上限，
        # 报 "descriptor download stalled (frozen clock?)"。
        # 真实后端不需要这个开关（它们的 now_ms 本来就是自由运行计数器）。
        self._lib.jsdk_hal_virtual_set_autotick(handle, 1)
        self._autotick = True

    # --- 仅供仿真/测试 -----------------------------------------------------

    def advance_ms(self, ms: int) -> None:
        """手动推进仿真时钟。

        打开自动推进（:meth:`_open` 里已做）后一般**不需要**手动调用；
        它的用途是精确控制时间轴（例如断言看门狗在 N ms 后触发）。
        """
        self._lib.jsdk_hal_virtual_advance_ms(self.handle, int(ms))

    def inject(self, frame: _abi.CanFrame) -> None:
        """冒充设备向主站注入一帧（构造异常场景用）。"""
        raise_for_status(self._lib.jsdk_hal_virtual_inject(self.handle,
                                                           ctypes.byref(frame)),
                         "virtual_inject")

    def capture(self) -> _abi.CanFrame | None:
        """取出 SDK 刚发出的一帧（字节级断言用）；无帧返回 None。"""
        frame = _abi.CanFrame()
        if self._lib.jsdk_hal_virtual_capture(self.handle, ctypes.byref(frame)):
            return frame
        return None

    def set_tx_fail(self, count: int) -> None:
        """令后续 ``count`` 次发送失败（-1 = 永久失败）。"""
        self._lib.jsdk_hal_virtual_set_tx_fail(self.handle, int(count))

    def dropped(self) -> int:
        """因 RX 环满而丢弃的注入帧数。"""
        return int(self._lib.jsdk_hal_virtual_dropped(self.handle))


class SocketCanHal(Hal):
    """Linux SocketCAN。

    位定时由**内核**负责，链路必须先 up：

    .. code-block:: bash

        ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on

    SDK 不改链路参数。若要求 FD 而链路是 Classic，打开会失败（``INVALID_ARG``）
    —— 这比之后每条帧都 EINVAL 且看不出原因要好。
    """

    name = "socketcan"

    def __init__(self, ifname: str = "can0",
                 bitrate: int = 1_000_000, data_bitrate: int = 5_000_000) -> None:
        super().__init__()
        self.ifname = ifname
        self.bitrate = bitrate
        self.data_bitrate = data_bitrate
        self.supports_fd = data_bitrate != 0

    def _open(self) -> None:
        handle = ctypes.c_void_p()
        st = self._lib.jsdk_hal_socketcan_open(
            ctypes.byref(self.storage), ctypes.byref(handle),
            self.ifname.encode("utf-8"), int(self.bitrate), int(self.data_bitrate))
        raise_for_status(st, f"socketcan_open({self.ifname})")
        self.handle = handle


class PcanHal(Hal):
    """PEAK PCAN-Basic（Windows / macOS）。

    PCAN-Basic 由**运行期**加载（``LoadLibrary`` / ``dlopen``），所以没装驱动
    只影响这一个后端，不会让整个包起不来。没装驱动 → ``NOT_FOUND``（异常文本
    里会说装什么）。

    :param channel: ``PCAN_USBBUS1``，或简写 ``can0``/``can1``，或 ``0x51``。
    """

    name = "pcan"

    def __init__(self, channel: str = "PCAN_USBBUS1",
                 bitrate: int = 1_000_000, data_bitrate: int = 5_000_000) -> None:
        super().__init__()
        self.channel = channel
        self.bitrate = bitrate
        self.data_bitrate = data_bitrate
        self.supports_fd = data_bitrate != 0

    def _open(self) -> None:
        handle = ctypes.c_void_p()
        st = self._lib.jsdk_hal_pcan_open(
            ctypes.byref(self.storage), ctypes.byref(handle),
            self.channel.encode("utf-8"), int(self.bitrate), int(self.data_bitrate))
        raise_for_status(st, f"pcan_open({self.channel})")
        self.handle = handle


class SlcanHal(Hal):
    """串口 slcan（CANable / 兼容 USB-CAN 适配器）。

    :param port: ``COM5``（Windows）/ ``/dev/ttyACM0``（Linux）
    :param baud: **串口**波特率（不是 CAN 段波特率！）。适配器通常是 115200。
    :param data_bitrate: CAN FD **数据段**速率（bps）。

        - ``None``（默认）→ 用协议默认的 **5 Mbps**，打开时会向适配器发 ``Y5``；
        - ``0`` → **不碰适配器配置**，只打开通道（适配器保持它自己的设置）；
        - 其它值 → 只有 ``2000000``（``Y2``）与 ``5000000``（``Y5``）有公认的
          命令码，别的速率会抛 :class:`~jsdk_can.JsdkUnsupportedError`
          —— ``Y<n>`` 的 n 是固件私有表索引，**不猜**。
    :param bitrate: CAN **仲裁段**速率（bps）。

        - ``None``（默认）→ **不碰适配器已有设置**（发 0，厂家工具配的值保留）；
        - 非 None → 打开序列里发 ``S<n>`` 把适配器切到该速率。只支持
          Lawicel 标准 9 档（10k/20k/50k/100k/125k/250k/500k/800k/1M），
          其它值抛 ``JsdkUnsupportedError``（``S9`` 及以后是固件私有扩展，
          **不猜**）。

          ⚠ 这是**改适配器配置**的动作，所以默认不做；现场的适配器往往已经
          被厂家工具配好，默认去改它反而会把一条本来能跑的链路弄断。

    ⚠ slcan 的 FD 支持来自 **CANable 2.0 固件**（``b/B/d/D`` 帧前缀）。
    单帧载荷最大 64 B，但 ASCII 展开 + 串口带宽限制使实际帧率很低
    （Classic 约 100~500 fps，FD 更低），只适合配置、监控与低速控制。

    ⚠ 如果手上的适配器是**只支持 Classic 的旧型号**（CANable 1.x 等），
    请用 ``data_bitrate=0`` 并让 ``Context(is_fd=False)`` —— 否则帧会以 FD
    前缀发出而适配器不认，现场表现为 NACK 而收不到任何回复。
    """

    name = "slcan"
    supports_fd = True

    def __init__(self, port: str = "COM3", baud: int = 115_200,
                 data_bitrate: int | None = 5_000_000,
                 bitrate: int | None = None) -> None:
        super().__init__()
        self.port = port
        self.baud = baud
        self.data_bitrate = 5_000_000 if data_bitrate is None else data_bitrate
        self.bitrate = 0 if bitrate is None else int(bitrate)

    def _open(self) -> None:
        handle = ctypes.c_void_p()
        # 新库走 `_ex`（带仲裁段速率）；旧库回退到不带它的入口，
        # 但此时 **bitrate 参数无法生效** —— 不静默忽略，直接报错。
        fn = getattr(self._lib, "jsdk_hal_slcan_open_ex", None)
        if fn is not None:
            st = fn(ctypes.byref(self.storage), ctypes.byref(handle),
                    self.port.encode("utf-8"), int(self.baud),
                    int(self.bitrate), int(self.data_bitrate))
        else:
            if self.bitrate:
                raise_for_status(-6, f"slcan_open({self.port})",
                                 "这个 libjsdk_can 没有 jsdk_hal_slcan_open_ex()，"
                                 "无法下发仲裁段速率（--bitrate）；请升级 SDK 或改为 bitrate=None")
            st = self._lib.jsdk_hal_slcan_open(
                ctypes.byref(self.storage), ctypes.byref(handle),
                self.port.encode("utf-8"), int(self.baud), int(self.data_bitrate))
        raise_for_status(st, f"slcan_open({self.port})")
        self.handle = handle

    # --- 诊断 -------------------------------------------------------------

    def last_open_error(self) -> str:
        """上一次 slcan 打开失败的**原因**（人可读，含 errno 与建议）。

        ⚠ 打开失败时**没有句柄**可查（见 C 侧 `jsdk_hal_slcan_last_open_error()`
        的说明）：所以它是静态缓冲、在每个后端实例上都能问：
        “invalid-argument” 这三个字分不清**权限 / 设备不存在 / 被占用**
        （Ubuntu 上忘了 sudo 就是 EACCES，现场为此查很久）。
        """
        fn = getattr(self._lib, "jsdk_hal_slcan_last_open_error", None) if self._lib else None
        if fn is None:
            return ""
        return (fn() or b"").decode("utf-8", "replace")

    def fd_config(self) -> tuple[bool, int]:
        """``(是否已向适配器发过 Y<n>, 已设置的数据段速率)``。"""
        if not hasattr(self._lib, "jsdk_hal_slcan_fd_config"):
            return (False, 0)          # 旧库没这个诊断函数：不该因此挂掉
        enabled = ctypes.c_int(0)
        bitrate = ctypes.c_uint32(0)
        self._lib.jsdk_hal_slcan_fd_config(self.handle, ctypes.byref(enabled),
                                          ctypes.byref(bitrate))
        return bool(enabled.value), int(bitrate.value)

    def nominal_config(self) -> tuple[bool, int]:
        """``(是否已向适配器发过 S<n>, 已设置的仲裁段速率)``。

        ``enabled=False`` 不等于“速率未知”，而是“**我们没有改它**” ——
        适配器可能有一个自己存着的速率（厂家工具设的）。这两件事必须分开讲。
        """
        if not hasattr(self._lib, "jsdk_hal_slcan_nominal_config"):
            return (False, 0)          # 旧库没这个诊断函数：不该因此挂掉
        enabled = ctypes.c_int(0)
        bitrate = ctypes.c_uint32(0)
        self._lib.jsdk_hal_slcan_nominal_config(self.handle,
                                                ctypes.byref(enabled),
                                                ctypes.byref(bitrate))
        return bool(enabled.value), int(bitrate.value)

    def fd_frames(self) -> tuple[int, int]:
        """``(发出的 FD 帧数, 收到的 FD 帧数)``。

        收到的恒为 0 往往说明**对端在按 Classic 回**（或适配器没进 FD 模式），
        这比“反愃写不进去”早一步指出现场问题。
        """
        if not hasattr(self._lib, "jsdk_hal_slcan_fd_frames"):
            return (0, 0)
        tx = ctypes.c_uint32(0)
        rx = ctypes.c_uint32(0)
        self._lib.jsdk_hal_slcan_fd_frames(self.handle, ctypes.byref(tx),
                                          ctypes.byref(rx))
        return int(tx.value), int(rx.value)

    def stats(self) -> dict:
        """线级计数：``tx`` / ``rx`` / ``malformed`` / ``acks`` / ``nacks``。

        排障顺序（真机经验）：``acks == 0`` 说明**适配器连命令都没应答**
        （打开序列没生效 —— 那会让"打开后第一个请求"永远丢，见 `BACKLOG` L7）；
        ``malformed`` 持续增长说明串口上有非 slcan 行（波特率不对/别的工具在抢）；
        ``nacks`` 说明适配器拒绝了某条命令（例如它不认识 ``Y``）。
        """
        out = {"tx": 0, "rx": 0, "malformed": 0, "acks": 0, "nacks": 0}
        if not hasattr(self._lib, "jsdk_hal_slcan_stats"):
            return out                # 旧库/未编 slcan 后端：不该因此挂掉
        vals = [ctypes.c_uint32(0) for _ in range(5)]
        self._lib.jsdk_hal_slcan_stats(self.handle, *[ctypes.byref(v) for v in vals])
        for k, v in zip(out, vals):
            out[k] = int(v.value)
        return out


# ==========================================================================
# 插件式（回调）HAL —— “没有 C 实现”的传输在 Python 侧的接入点
# ==========================================================================

#: C 侧只把 `hal.user` 原样回传，所以拿它当 token 找回 Python 对象。
_HAL_REGISTRY: dict = {}
_HAL_TOKEN_SEQ = itertools.count(1)


def _hal_lookup(user) -> "CallbackHal | None":
    if not user:
        return None
    return _HAL_REGISTRY.get(int(user))


class CallbackHal(Hal):
    """用 **ctypes 回调**把 Python 侧实现的传输接进 SDK。

    为什么要有它
    ------------
    内置后端（virtual/socketcan/pcan/slcan）都是 C 侧实现，Python 只调
    ``jsdk_hal_*_open()`` 拿现成的函数表。**没有 C 实现**的传输
    （python-can 的 gs_usb、EtherCAT 网关、UDP 桥、测试替身……）过去在
    Python 侧无路可走 —— 本类就是那条路。

    ⚠ 与 ``docs/DESIGN.zh-CN.md`` §7.2 的关系
      那里写的是“Python 侧**不**用 ctypes 回调拼 ``jsdk_can_hal_t``
      （脆弱且慢）”。那条针对的是**内置后端**（它们有 C 实现，走回调纯属
      自找麻烦）；本类只服务“**没有** C 实现”的后端，并且把这个代价摆在
      明处：

      * **慢**：每次 ``send`` / ``recv`` / ``now_ms`` 都是一次 C→Python 调用。
        桌面非实时场景可以接受（Python 绑定本身就明确非实时）；
        **不要拿它做硬实时控制**。
      * **脆弱**：``jsdk_context_init()`` 只**复制结构体**，函数指针指向的
        ``CFUNCTYPE`` 对象一旦被 GC，C 侧下一次回调就是跳转到已释放内存。
        本仓已为描述符回调踩过一次（``tests/test_full_surface.py`` A13），
        所以这里：① 回调对象**存在实例上**（``self._cbs``）；
        ② 实例挂在模块级注册表里，C 侧只回传一个整数 token；
        ③ ``Context`` 持有本实例 ⇒ 只要上下文活着，回调就一定活着。

    子类实现这几个 Python 方法即可（都在**非 RT** 的 Python 线程上被调用）：

      * :meth:`_py_open` / :meth:`_py_close` —— 打开/关闭真实传输
      * :meth:`py_send` → 0 成功 / 非 0 失败（SDK 会计入 ``tx_failed``
        并在下一周期重试，不中止循环）
      * :meth:`py_recv` → 1 = 已填好一帧 / 0 = 当前无帧
      * :meth:`py_bus_flags` → ``JSDK_HAL_BUS_*`` 位（可省，默认 OK）
    """

    #: 后端名（子类覆盖）
    name = "callback"
    #: 回调式后端的 FD 能力由子类决定
    supports_fd = False

    def __init__(self) -> None:
        super().__init__()
        self._token = next(_HAL_TOKEN_SEQ)
        _HAL_REGISTRY[self._token] = self
        self.storage.user = ctypes.c_void_p(self._token)
        # ⚠ 必须持引用（理由见类文档）
        self._cbs = (
            _abi.HAL_SEND_FN(self._cb_send),
            _abi.HAL_RECV_FN(self._cb_recv),
            _abi.HAL_NOW_MS_FN(self._cb_now_ms),
            _abi.HAL_BUS_STATUS_FN(self._cb_bus_status),
        )

    # --- Hal 生命周期 -----------------------------------------------------

    def attach(self, lib) -> None:
        """**不走** C 侧工厂：自己把函数表填进 ``storage``。"""
        self._lib = lib
        self._py_open()
        send, recv, now_ms, bus_status = self._cbs
        self.storage.send = ctypes.cast(send, ctypes.c_void_p)
        self.storage.recv = ctypes.cast(recv, ctypes.c_void_p)
        self.storage.now_ms = ctypes.cast(now_ms, ctypes.c_void_p)
        self.storage.bus_status = ctypes.cast(bus_status, ctypes.c_void_p)
        self._opened = True

    def close(self) -> None:
        """关闭传输。

        ⚠ **不**清 ``storage`` 里的函数指针：若调用方在 ``close()`` 之后还
        用着 Context，清掉就变成“调用空指针”；留着最坏也只是回调返回失败。
        回调对象随本实例存活，而 ``Context`` 持有本实例。
        """
        if not self._opened:
            return
        try:
            self._py_close()
        finally:
            _HAL_REGISTRY.pop(self._token, None)
            self._opened = False

    # --- 子类实现（Python 侧传输） ----------------------------------------

    def _py_open(self) -> None:  # pragma: no cover - 抽象
        raise NotImplementedError

    def _py_close(self) -> None:
        pass

    def py_send(self, frame) -> int:  # pragma: no cover - 抽象
        raise NotImplementedError

    def py_recv(self, frame) -> int:  # pragma: no cover - 抽象
        raise NotImplementedError

    def py_bus_flags(self) -> int:
        return _abi.JSDK_HAL_BUS_OK

    # --- ctypes 回调（C → Python） ----------------------------------------

    def _cb_send(self, user, fptr) -> int:
        h = _hal_lookup(user)
        if h is None or not fptr:
            return -1
        try:
            return int(h.py_send(fptr.contents))
        except Exception:
            # ⚠ 回调里抛出去会**穿过 C 栈**，只能吞掉并如实报失败
            return -1

    def _cb_recv(self, user, fptr) -> int:
        h = _hal_lookup(user)
        if h is None or not fptr:
            return 0
        try:
            return int(h.py_recv(fptr.contents))
        except Exception:
            return 0

    def _cb_now_ms(self, user) -> int:
        return (time.perf_counter_ns() // 1_000_000) & 0xFFFFFFFF

    def _cb_bus_status(self, user, flags_ptr) -> int:
        h = _hal_lookup(user)
        if h is None or not flags_ptr:
            return -1
        try:
            flags_ptr[0] = int(h.py_bus_flags()) & 0xFFFFFFFF
            return 0
        except Exception:
            return -1


def _libusb_warm_up() -> bool:
    """让 pyusb 在 Windows 上能找到 `libusb-1.0.dll`。

    ⚠ 真机踩过：pyusb 的 ``libusb1.get_backend()`` 在 Windows 上**找不到**
       `libusb-package` 装在 site-packages 里的那份 DLL，返回 ``None``；
       python-can 的 gs_usb 接口内部就用这句，于是
       ``GsUsb.scan()`` 直接返回 **0 个设备** —— 症状是“设备明明插着，
       程序说找不到”。

    先按**绝对路径**把 DLL 载进进程（``libusb_package.get_libusb1_backend()``），
    之后 pyusb 再按基名 ``LoadLibrary`` 就会命中已加载模块。
    实测：不预热 = 0 个设备；预热后 = 1 个设备。
    """
    try:
        from usb.backend import libusb1 as _libusb1
        if _libusb1.get_backend() is not None:
            return True
    except Exception:
        pass
    try:
        import libusb_package
        getter = getattr(libusb_package, "get_libusb1_backend", None)
        if getter is None:
            return False
        getter()
        return True
    except Exception:
        return False


class GsusbHal(CallbackHal):
    """**gs_usb**（candleLight 协议）USB-CAN 适配器，走 python-can。

    为什么需要它
    ------------
    SDK 的 C 侧没有 gs_usb 后端，而 gs_usb **不是串口**（slcan 够不着）、
    也不是 PCAN ⇒ 过去在 Windows 上完全没法用它。本类用 python-can 的
    ``gs_usb`` 接口把它接进来。

    依赖：``pip install "python-can[gs_usb]"``（带上 ``gs-usb`` + ``pyusb``）。

    ⚠ **Windows 驱动**：设备必须绑到 **WinUSB / libusbK / libusb-win32**
      （Zadig 或设备管理器改）。绑在厂商驱动上（实测某机是 VirtualBox 的
      ``VBoxUSB``）libusb **打不开**，而报出来的可能是“找不到设备”。
      另外它与 **usbipd**（把设备交给 WSL）**互斥**：同一条 USB 通路只能开一条。

    ⚠ **Classic only**：python-can 的 gs_usb 传输是 CAN 2.0（8 字节）实现，
      没有 FD 路径 ⇒ ``supports_fd = False``，``Context`` 会因此默认按经典帧。
      需要 FD 请用 ``SocketCanHal``（Linux 内核驱动支持 FD）。

    ⚠ 设备会把**我们发出去的帧回显**回来，回显不是“收到”——本类按
      python-can 的 ``is_rx`` 过滤掉（不过滤的话 SDK 会把自己的请求当对端帧）。

    :param channel: 设备序号（``"0"`` 起），或 ``"bus:address"``（如 ``"1:4"``）
    :param bitrate: 仲裁段波特率（bps）
    :param rx_queue: 接收缓冲深度（帧）；满了丢**最旧**的并计数
    """

    name = "gsusb"
    supports_fd = False

    _HINT = ('pip install "python-can[gs_usb]"；并确认设备已用 Zadig/设备管理器'
             '绑到 WinUSB（不是厂商驱动），且没有被 usbipd 交给 WSL')

    def __init__(self, channel: str = "0", bitrate: int = 1_000_000,
                 rx_queue: int = 4096) -> None:
        super().__init__()
        self.channel = str(channel)
        self.bitrate = int(bitrate)
        self.rx_queue = int(rx_queue)
        self.bus = None
        self._rx: collections.deque = collections.deque(maxlen=max(16, self.rx_queue))
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._latched = 0
        self._stats = {"tx": 0, "tx_err": 0, "rx": 0, "echo": 0,
                       "err_frame": 0, "dropped": 0}

    # --- 便利：列出设备 ---------------------------------------------------

    @classmethod
    def devices(cls) -> list:
        """枚举当前可见的 gs_usb 适配器（排障用）。

        ⚠ 返回 **0 个** 有两类原因，别混为一谈：① 设备不在（或被 usbipd
        交给了 WSL）；② libusb 后端/DLL 没找到（见 :func:`_libusb_warm_up`）。
        """
        if not _libusb_warm_up():
            raise_for_status(Status.NOT_FOUND, "gsusb_devices",
                             "没有可用的 libusb 后端。" + cls._HINT)
        from gs_usb.gs_usb import GsUsb
        out = []
        for i, dev in enumerate(GsUsb.scan()):
            try:
                serial = dev.serial_number
            except Exception:
                serial = "<读不到>"
            out.append({"index": i, "bus": dev.bus, "address": dev.address,
                        "serial": serial})
        return out

    # --- 打开 / 关闭 ------------------------------------------------------

    @staticmethod
    def _parse_channel(spec: str):
        s = (spec or "0").strip()
        if ":" in s:
            a, b = s.split(":", 1)
            return None, int(a, 0), int(b, 0)
        return int(s, 0), None, None

    def _py_open(self) -> None:
        try:
            import can
        except ImportError as exc:
            raise_for_status(Status.NOT_FOUND, "gsusb_open",
                             f"需要 python-can（{exc}）。{self._HINT}")
        if not _libusb_warm_up():
            raise_for_status(Status.NOT_FOUND, "gsusb_open",
                             "找不到可用的 libusb 后端（装 libusb-package，或把 "
                             "libusb-1.0.dll 放到程序目录）。" + self._HINT)
        from gs_usb.gs_usb import GsUsb

        found = len(GsUsb.scan())
        idx, bus_no, addr = self._parse_channel(self.channel)
        kwargs = {"interface": "gs_usb", "bitrate": self.bitrate}
        if bus_no is None:
            kwargs["channel"] = idx
        else:
            kwargs["channel"] = self.channel
            kwargs["bus"] = bus_no
            kwargs["address"] = addr
        try:
            self.bus = can.Bus(**kwargs)
        except Exception as exc:
            raise_for_status(
                Status.TRANSPORT, "gsusb_open",
                f"打开 gs_usb 失败（枚举到 {found} 个设备；channel={self.channel}，"
                f"bitrate={self.bitrate}）：{exc}。{self._HINT}")

        self._stop.clear()
        self._thread = threading.Thread(target=self._rx_loop,
                                        name="jsdk-gsusb-rx", daemon=True)
        self._thread.start()

    def _py_close(self) -> None:
        self._stop.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=1.0)
        self._thread = None
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None

    # --- 接收线程 ---------------------------------------------------------

    def _rx_loop(self) -> None:
        """常驻接收线程。

        为什么必须有它：gs_usb 是**设备主动推**的，而 HAL 的 ``recv`` 契约要求
        **非阻塞** —— 若在 ``recv`` 里同步读，每个空轮询都要等一个 USB 往返
        （python-can 内部还会把 timeout 夹到 ≥1 ms），控制周期会被它吃掉。
        这里让线程一直挂着读，轮询只是从队列取。
        """
        while not self._stop.is_set():
            bus = self.bus
            if bus is None:
                time.sleep(0.01)
                continue
            try:
                msg = bus.recv(timeout=0.05)
            except Exception:
                # 拔线/关设备时各种 USB 错都可能冒出来；线程不能就这么死掉
                self._latched |= _abi.JSDK_HAL_BUS_ERROR_WARN
                time.sleep(0.05)
                continue
            if msg is None:
                continue
            if getattr(msg, "is_error_frame", False):
                self._stats["err_frame"] += 1
                self._latched |= _abi.JSDK_HAL_BUS_ERROR_WARN
                continue
            if not getattr(msg, "is_rx", True):
                # 自己发出去的回显（echo），不是“收到”
                self._stats["echo"] += 1
                continue
            with self._lock:
                if len(self._rx) >= self._rx.maxlen:
                    self._stats["dropped"] += 1
                self._rx.append((int(msg.arbitration_id), int(msg.dlc),
                                 bytes(msg.data)))
            self._stats["rx"] += 1

    # --- HAL 契约（C 侧调用） --------------------------------------------

    def py_recv(self, frame) -> int:
        with self._lock:
            if not self._rx:
                return 0
            fid, flen, fdata = self._rx.popleft()
        frame.id = fid
        frame.len = min(int(flen), 8)          # 本后端是 CAN 2.0
        frame.flags = _abi.JSDK_FRAME_EXT
        for i in range(frame.len):
            frame.data[i] = fdata[i]
        return 1

    def py_send(self, frame) -> int:
        import can
        bus = self.bus
        if bus is None:
            return -1
        n = min(int(frame.len), 8)
        data = bytes(int(frame.data[i]) for i in range(n))
        msg = can.Message(arbitration_id=int(frame.id) & 0x1FFFFFFF,
                          is_extended_id=True, data=data)
        try:
            bus.send(msg)
        except Exception:
            self._stats["tx_err"] += 1
            return -1
        self._stats["tx"] += 1
        return 0

    def py_bus_flags(self) -> int:
        if self.bus is None:
            return 0
        return _abi.JSDK_HAL_BUS_OK | self._latched

    def stats(self) -> dict:
        """线级计数（排障用）：``tx`` / ``tx_err`` / ``rx`` / ``echo`` /
        ``err_frame`` / ``dropped``，外加队列深度与锁存标志。

        ⚠ ``rx == 0`` 且 ``echo > 0`` 说明**发得出去、收不到任何东西** ——
        先查对端是否在发（而不是先怀疑自己）；``err_frame`` 增长指向链路错误。
        """
        out = dict(self._stats)
        out.update({"queued": len(self._rx), "channel": self.channel,
                    "bitrate": self.bitrate, "latched_flags": self._latched})
        return out
