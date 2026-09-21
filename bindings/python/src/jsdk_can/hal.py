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

import ctypes
from dataclasses import dataclass

from . import _abi
from .errors import JsdkUsageError, raise_for_status

__all__ = [
    "Hal",
    "VirtualHal",
    "SocketCanHal",
    "PcanHal",
    "SlcanHal",
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
                 data_bitrate: int | None = 5_000_000) -> None:
        super().__init__()
        self.port = port
        self.baud = baud
        self.data_bitrate = 5_000_000 if data_bitrate is None else data_bitrate

    def _open(self) -> None:
        handle = ctypes.c_void_p()
        st = self._lib.jsdk_hal_slcan_open(
            ctypes.byref(self.storage), ctypes.byref(handle),
            self.port.encode("utf-8"), int(self.baud), int(self.data_bitrate))
        raise_for_status(st, f"slcan_open({self.port})")
        self.handle = handle

    # --- 诊断 -------------------------------------------------------------

    def fd_config(self) -> tuple[bool, int]:
        """``(是否已向适配器发过 Y<n>, 已设置的数据段速率)``。"""
        if not hasattr(self._lib, "jsdk_hal_slcan_fd_config"):
            return (False, 0)          # 旧库没这个诊断函数：不该因此挂掉
        enabled = ctypes.c_int(0)
        bitrate = ctypes.c_uint32(0)
        self._lib.jsdk_hal_slcan_fd_config(self.handle, ctypes.byref(enabled),
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
