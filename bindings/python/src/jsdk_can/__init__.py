"""``jsdk_can`` —— CyberBeast 关节 SDK 的 Python 绑定。

用法
----

.. code-block:: python

    from jsdk_can import Context, VirtualHal, Mode

    # 没有硬件时用虚拟后端（自带仿真设备与描述符，全部流程可跑通）
    with Context(VirtualHal()) as ctx:
        j = ctx.add_joint(1)
        ctx.configure()
        print(ctx.dump_config())
        j.enable(Mode.MIT)
        ctx.activate()
        for _ in range(10):
            ctx.cycle_begin()
            j.set_mit(pos=0.0, kp=2.0, kd=0.2)
            ctx.cycle_end()
        print(j.feedback())

真机：

.. code-block:: python

    from jsdk_can import Context, SocketCanHal   # 或 PcanHal / SlcanHal

    with Context(SocketCanHal("can0")) as ctx:
        ...

设计边界（**请先看这三条**）
----------------------------

1. **非实时**。``Context`` 不创建线程；``cycle_begin/end`` 由调用者驱动。Python 侧
   的调度抖动远大于 CAN 周期，所以绑定适合**配置、采集、可视化、产测**，不适合
   硬实时控制。硬实时请用 C/C++ 或 C++ 包装。
2. **不隐藏 C 的语义**。控制函数（``set_mit`` 等）不抛异常、缓存目标，越界在周期
   结束按 §6.10 策略处理 —— 与 C 一致。配置/运维函数才阻塞并抛异常。
3. **导入即 ABI 自检**。共享库与绑定的结构体布局不一致会直接抛
   :class:`~jsdk_can._abi.AbiMismatchError`，而不是等你踩内存。
"""

from __future__ import annotations

from ._abi import (
    JSDK_ABI_VERSION_CAN as ABI_VERSION,
    CanFrame,
    alloc_aligned,
    check_abi,
    library_search_path,
    load_library,
)
from .context import BusState, Context, DescInfo, Endpoint
from .errors import AbiMismatchError, LibraryNotFoundError
from .enums import (
    AxisState,
    BACKEND_TAG,
    BusFlag,
    DescMode,
    DescRetain,
    EpAccess,
    EpType,
    FrameFlag,
    Mode,
    ModeState,
    Status,
    StatusFlag,
)
from .errors import (
    JsdkBusyError,
    JsdkDeviceNotFoundError,
    JsdkError,
    JsdkMemoryError,
    JsdkParseError,
    JsdkProtocolError,
    JsdkStateError,
    JsdkTimeoutError,
    JsdkTransportError,
    JsdkUnsupportedError,
    JsdkUsageError,
)
from .hal import (
    DEFAULT_VIRTUAL_SPEC,
    Hal,
    PcanHal,
    SlcanHal,
    SocketCanHal,
    VirtualHal,
)
from .joint import ConfigSnapshot, DeviceInfo, FaultInfo, Feedback, Joint

__version__ = "0.1.0"

__all__ = [
    # 上下文与关节
    "Context",
    "Joint",
    "Feedback",
    "ConfigSnapshot",
    "DeviceInfo",
    "FaultInfo",
    "BusState",
    "DescInfo",
    "Endpoint",
    # 传输后端
    "Hal",
    "VirtualHal",
    "SocketCanHal",
    "PcanHal",
    "SlcanHal",
    "DEFAULT_VIRTUAL_SPEC",
    # 枚举
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
    # 异常
    "JsdkError",
    "JsdkUsageError",
    "JsdkStateError",
    "JsdkTransportError",
    "JsdkTimeoutError",
    "JsdkProtocolError",
    "JsdkDeviceNotFoundError",
    "JsdkUnsupportedError",
    "JsdkMemoryError",
    "JsdkParseError",
    "JsdkBusyError",
    # 低层（给需要直接摸 ABI 的人）
    "CanFrame",
    "load_library",
    "check_abi",
    "library_search_path",
    "alloc_aligned",
    "AbiMismatchError",
    "LibraryNotFoundError",
    "__version__",
]
