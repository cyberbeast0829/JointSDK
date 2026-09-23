"""``Context``：一个 CAN 口上的若干关节。

生命周期（与 C 侧一致，顺序不能变）
----------------------------------

.. code-block:: text

    Context(...)                 # 加载库 + ABI 自检 + 打开 HAL
      .add_joint(1)              # 必须在 configure() 之前
      .configure()               # 握手 + 下载 JSON 描述符 + 读回 gear/mit_max_*
      .enable_all()              # 或逐个 j.enable(Mode.MIT)
      while True:                # 控制回路
          ctx.cycle_begin()
          j.set_mit(...)
          ctx.cycle_end()
      ctx.deactivate()           # 安全停车：hold_position → STOP_MOTOR → 等 IDLE
      ctx.close()                # 释放上下文与 HAL

``with Context(...) as ctx:`` 会在退出时自动 :meth:`close`（内部先
:meth:`deactivate`，只有真的使能过才发停车帧）。

关于阻塞
--------

``configure()`` / ``discover()`` / ``calibrate()`` / ``home()`` 都是**阻塞**的
配置阶段 API：它们靠"等时间流逝"来轮询响应。

* 真实后端（socketcan/pcan/slcan）：`now_ms` 是自由运行计数器，天然可用；
* **虚拟后端**：时钟默认只在 ``advance_ms()`` 时前进 —— 冻结时钟下这些 API 会一直
  等到内部自旋上限。所以 :class:`~jsdk_can.hal.VirtualHal` 打开时会调用 C 侧的
  ``jsdk_hal_virtual_set_autotick()``，让 ``now_ms`` 自己前进 1 ms/次，语义与真实
  HAL 一致。**这是 C 侧的能力**（不是 Python 包装的 hack），因此 CLI 与离线脚本
  也走同一条路。
"""

from __future__ import annotations

import ctypes
import time
from typing import Iterator

from . import _abi
from .enums import DescMode, DescRetain, EpType, Mode, Status
from .errors import error_for_status, raise_for_status
from .hal import Hal, VirtualHal
from .joint import ConfigSnapshot, DeviceInfo, FaultInfo, Joint

__all__ = ["Context", "BusState", "DescInfo", "Endpoint"]


class BusState:
    """``jsdk_bus_state_t`` 的 Python 视图。"""

    __slots__ = ("tx_frames", "rx_frames", "tx_failed", "rx_dropped",
                 "keepalive_sent", "last_rx_age_ms", "hal_bus_flags",
                 "link_errors", "nodes_online", "link_up")

    def __init__(self, c: _abi.BusState) -> None:
        self.tx_frames = c.tx_frames
        self.rx_frames = c.rx_frames
        self.tx_failed = c.tx_failed
        self.rx_dropped = c.rx_dropped
        self.keepalive_sent = c.keepalive_sent
        self.last_rx_age_ms = c.last_rx_age_ms
        self.hal_bus_flags = c.hal_bus_flags
        self.link_errors = c.link_errors
        self.nodes_online = c.nodes_online
        self.link_up = bool(c.link_up)

    def as_dict(self) -> dict:
        return {k: getattr(self, k) for k in self.__slots__}

    def __repr__(self) -> str:  # pragma: no cover - 人读
        return (f"BusState(link_up={self.link_up}, nodes={self.nodes_online}, "
                f"tx={self.tx_frames}, rx={self.rx_frames}, "
                f"errors={self.link_errors})")


class DescInfo:
    """``jsdk_desc_info_t`` 的 Python 视图。"""

    __slots__ = ("total_len", "crc", "fw_version", "hw_version",
                 "endpoint_count", "parsed_total", "frames_rx", "bytes_scanned",
                 "complete", "mode_used", "shared_hit", "raw_sink_failed")

    def __init__(self, c: _abi.DescInfo) -> None:
        self.total_len = c.total_len
        self.crc = c.crc
        self.fw_version = c.fw_version
        self.hw_version = c.hw_version
        self.endpoint_count = c.endpoint_count
        self.parsed_total = c.parsed_total
        self.frames_rx = c.frames_rx
        self.bytes_scanned = c.bytes_scanned
        self.complete = bool(c.complete)
        self.mode_used = c.mode_used
        self.shared_hit = bool(c.shared_hit)
        self.raw_sink_failed = bool(c.raw_sink_failed)

    def as_dict(self) -> dict:
        return {k: getattr(self, k) for k in self.__slots__}

    def __repr__(self) -> str:  # pragma: no cover - 人读
        return (f"DescInfo(len={self.total_len}, crc=0x{self.crc:04X}, "
                f"endpoints={self.endpoint_count}, complete={self.complete})")


class Endpoint:
    """一个端点（路径 / ID / 类型 / 权限）。"""

    __slots__ = ("path", "ep_id", "type", "access")

    def __init__(self, path: str, ep_id: int, type_: EpType | int, access: int):
        self.path = path
        self.ep_id = ep_id
        self.type = type_
        self.access = access

    @property
    def readable(self) -> bool:
        return bool(self.access & 0x01)

    @property
    def writable(self) -> bool:
        return bool(self.access & 0x02)

    @property
    def access_str(self) -> str:
        return ("r" if self.readable else "-") + ("w" if self.writable else "-")

    def __repr__(self) -> str:  # pragma: no cover - 人读
        return (f"Endpoint({self.ep_id}, {self.type_name}, {self.access_str}, "
                f"{self.path!r})")

    @property
    def type_name(self) -> str:
        from .enums import EP_TYPE_STRINGS

        return EP_TYPE_STRINGS.get(self.type, str(self.type))


class Context:
    """一个上下文 = 一个 CAN 口 + 若干关节。

    :param hal: 传输后端（:class:`~jsdk_can.hal.VirtualHal` /
        :class:`~jsdk_can.hal.SocketCanHal` / ...）。默认用虚拟后端 —— 没有硬件
        也能把全部流程跑通。
    :param master_id: 主站源地址（1..254）。**禁止 0**：设备会完全不回复。
    :param is_fd: 帧格式。``True`` = CAN FD（1M/5M BRS）、``False`` = Classic，
        **显式给定后 SDK 不会自动改它**；``None``（默认）= 先按 FD 试，
        收到本关节第一帧时自动对齐到对端的实际格式并报告
        （见 :attr:`framing_learned`）。“设备是 Classic 还是 FD”协议无法协商，
        真机上报 ``desc-info`` 超时（`0/0 bytes, N frames received`）多半就是它。
    :param desc_retain: 端点保留策略。桌面用 ``ALL``；MCU 用 ``FILTERED``。
    :param desc_filter: ``FILTERED`` 时的路径过滤器（精确 / ``前缀*`` / ``段前缀.`` / ``*``）。
    :param arena_size: 描述符解析区大小；``None`` 用 C 侧推荐值。
    :param period_ns: 期望控制周期（ns），用于 keepalive 与超时判定。
    :param lib_path: 覆盖共享库路径（默认按 ``JSDK_LIB_PATH`` → 包内 → 构建目录）。
    """

    def __init__(self, hal: Hal | None = None, *,
                 master_id: int = 1,
                 is_fd: bool | None = None,
                 desc_retain: DescRetain | int = DescRetain.ALL,
                 desc_filter: list[str] | None = None,
                 desc_mode: DescMode | int = DescMode.DYNAMIC,
                 arena_size: int | None = None,
                 period_ns: int = 1_000_000,
                 state_timeout_ms: int = 0,
                 auto_keepalive: bool = True,
                 max_joints: int = 8,
                 lib_path: str | None = None) -> None:
        self._lib = _abi.load_library(lib_path)
        self.abi = _abi.check_abi(self._lib)

        if not 1 <= int(master_id) <= 254:
            raise ValueError("master_id 必须在 1..254（0 会让设备完全不回复）")

        self.hal = hal if hal is not None else VirtualHal()
        self.hal.attach(self._lib)

        # --- 上下文存储：必须 8 字节对齐（内部有指针与 double） ---
        size = int(self._lib.jsdk_context_size(None))
        self._ctx_buf, self._ctx_ptr = _abi.alloc_aligned(size)

        # --- 配置 ---
        self.cfg = _abi.ContextConfig()
        self._lib.jsdk_context_config_default(ctypes.byref(self.cfg))
        self.cfg.hal = self.hal.hal
        self.cfg.master_id = int(master_id)
        self.cfg.is_fd = 1 if (is_fd is None or is_fd) else 0
        # None = “没指定，猜 FD”，允许 SDK 自动对齐；给定值 = 明确要求，不许被改
        # （见 joint_sdk.h 的 is_fd_explicit：改掉显式配置会连带弄错 8 字节参数的分块读）
        self.cfg.is_fd_explicit = 0 if is_fd is None else 1
        self.cfg.period_ns = int(period_ns)
        # 等状态序列跑完的预算（calibrate/home）：0 = SDK 内置默认（标定 120 s / 回零 5 s）
        self.cfg.state_timeout_ms = int(state_timeout_ms)
        self.cfg.auto_keepalive = 1 if auto_keepalive else 0
        self.cfg.max_joints = int(max_joints)

        self.cfg.desc.mode = int(desc_mode)
        self.cfg.desc.retain = int(desc_retain)

        # filter_paths 的生命周期必须覆盖 configure()：把 Python 字符串的
        # bytes 与 c_char_p 数组都留在 self 上（否则 GC 之后就是悬垂指针）。
        self._filter_arr = None
        if desc_filter:
            self._filter_bytes = [p.encode("utf-8") for p in desc_filter]
            self._filter_arr = (ctypes.c_char_p * len(self._filter_bytes))(
                *self._filter_bytes)
            self.cfg.desc.filter_paths = self._filter_arr
            self.cfg.desc.filter_count = len(self._filter_bytes)

        # arena：同样要 8 字节对齐（里面存 char* 路径指针）
        if arena_size is None:
            arena_size = int(self._lib.jsdk_desc_arena_size(ctypes.byref(self.cfg.desc)))
        self._arena_buf, self._arena_ptr = _abi.alloc_aligned(arena_size)
        self.cfg.desc.arena = self._arena_ptr
        self.cfg.desc.arena_size = arena_size

        self._joints: list[Joint] = []
        self._configured = False
        self._closed = False
        #: 存活的 ctypes 回调对象。**必须**持有引用：CFUNCTYPE 对象被 GC 之后，
        #: C 侧还留着那个函数指针 —— 下一次回调就是跳转到已释放内存。
        self._callbacks: dict[str, object] = {}
        self._next_deadline_ns = 0
        self._paced_cycles = 0
        self._pace_worst_late_ms = 0.0

        raise_for_status(self._lib.jsdk_context_init(self._ctx_ptr,
                                                     ctypes.byref(self.cfg)),
                         "context_init")

    # --- 生命周期 ---------------------------------------------------------

    def add_joint(self, node_id: int, *, mode: Mode = Mode.MIT,
                  gear_ratio: float = 0.0, mit_max_pos: float = 0.0,
                  mit_max_vel: float = 0.0, mit_max_torque: float = 0.0,
                  mit_max_kp: float = 0.0, mit_max_kd: float = 0.0,
                  torque_constant: float = 0.0) -> Joint:
        """添加关节。必须在 :meth:`configure` 之前。

        标定参数留 0 = **从设备读取**（推荐）。读取失败不会静默取默认值，
        而是把 ``unit_scale.valid`` 置 0 并拒绝物理量 API —— **绝不猜量程**。
        """
        if self._configured:
            raise error_for_status(-4, "add_joint",
                                   "必须在 configure() 之前添加关节")
        if not 1 <= int(node_id) <= 254:
            raise ValueError("node_id 必须在 1..254")

        jc = _abi.JointConfig()
        ctypes.memset(ctypes.byref(jc), 0, ctypes.sizeof(jc))
        jc.node_id = int(node_id)
        jc.initial_mode = int(mode)
        jc.gear_ratio = float(gear_ratio)
        jc.mit_max_pos = float(mit_max_pos)
        jc.mit_max_vel = float(mit_max_vel)
        jc.mit_max_torque = float(mit_max_torque)
        jc.mit_max_kp = float(mit_max_kp)
        jc.mit_max_kd = float(mit_max_kd)
        jc.torque_constant = float(torque_constant)

        ptr = ctypes.c_void_p()
        raise_for_status(self._lib.jsdk_context_add_joint(
            self._ctx_ptr, ctypes.byref(jc), ctypes.byref(ptr)),
            f"add_joint({node_id})", self.last_error())

        joint = Joint(self, ptr, len(self._joints), int(node_id))
        self._joints.append(joint)
        return joint

    def configure(self) -> None:
        """握手 → 下载并解析 JSON 描述符 → 读回标定量。

        ⚠ **禁止在关节使能时调用**：描述符帧会挤满 TX 并挤掉控制帧，触发设备的
        ``break_timeout`` 保护（此时返回 ``BAD_STATE``）。
        ⚠ 这是**阻塞**调用（最长 ``desc.timeout_ms``）：它在配置阶段跑，不在控制
        回路上 —— 所以不会破坏"控制回路无阻塞"的承诺。
        """
        if self._configured:
            return
        raise_for_status(self._lib.jsdk_context_configure(self._ctx_ptr),
                         "configure", self.last_error())
        self._configured = True

    def activate(self) -> None:
        """按各关节 ``initial_mode`` 使能，并等到使能序列（含安全首帧）走完。"""
        raise_for_status(self._lib.jsdk_context_activate(self._ctx_ptr),
                         "activate", self.last_error())

    def deactivate(self) -> None:
        """**安全停车**：hold_position → 等 2 周期 → STOP_MOTOR → 等 IDLE。

        顺序不可颠倒 —— 先停发控制帧会直接触发设备的 ``break_timeout`` 保护。
        只有真的使能过才需要调用（未使能时它不做任何总线动作）。
        """
        if self._closed:
            return
        self._lib.jsdk_context_deactivate(self._ctx_ptr)

    def close(self) -> None:
        """释放上下文与 HAL（幂等）。"""
        if self._closed:
            return
        if any(j.is_enabled() for j in self._joints):
            self.deactivate()
        self._lib.jsdk_context_destroy(self._ctx_ptr)
        self.hal.close()
        self._closed = True
        self.reset_pace()

    def __enter__(self) -> Context:
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # --- 控制回路 ---------------------------------------------------------

    @property
    def framing_learned(self) -> int:
        """对端帧格式的学习结果（0 未知 / 1 已改 Classic / 2 已改 FD / 3 一致 / 4 冲突）。

        ⚠ 协议没有运行时协商：设备用 Classic 还是 FD 由它自己的配置决定。
        我们在收到本关节的第一帧时会把发送格式对齐过去 —— 否则发出去的帧
        它根本不收，现场只看到“有心跳、但我的请求没人应”。

        ⚠ 构造时**显式**传了 ``is_fd=`` 的话，自动对齐不会覆盖它；
        对端与它不同时返回 **4**（“你写的与对端冲突”），此时命令很可能无人应答。
        """
        return int(self._lib.jsdk_context_framing_learned(self._ctx_ptr))

    def cycle_begin(self, app_time_ns: int = 0) -> int:
        """周期开始：接收并解码全部待处理帧。**RT 安全**。"""
        return int(self._lib.jsdk_context_cycle_begin(self._ctx_ptr,
                                                      int(app_time_ns)))

    def cycle_end(self) -> int:
        """周期结束：编码并发送本周期指令、按需补喂狗、推进超时状态。"""
        return int(self._lib.jsdk_context_cycle_end(self._ctx_ptr))

    def poll(self, app_time_ns: int = 0) -> int:
        """``cycle_begin() + cycle_end()``（裸机/MCU 便利入口）。"""
        return int(self._lib.jsdk_context_poll(self._ctx_ptr, int(app_time_ns)))

    # --- 桌面节拍 ---------------------------------------------------------

    @property
    def period_ns(self) -> int:
        """``configure()`` 时登记的期望周期（ns）。"""
        return int(self.cfg.period_ns)

    def pace(self) -> None:
        """睡到下一个周期边界（**桌面/非 RT 场景**，MCU 上请用硬件定时器）。

        为什么需要它：``cycle_end()`` 只做"编帧 + 发帧"，**从不主动等待**。裸机上
        节拍由 RTOS 定时器提供；Python 里如果循环里不睡，控制回路会以
        "CPU 有多快就发多快"的速度跑 —— 真机上这不是"更跟手"，而是把总线塞满、
        帧间隔抖动、看门狗超时判定失去意义。

        实现要点：按**绝对时间**累加 deadline，而不是每次 ``sleep(周期)``。
        后者会把每周期的工作耗时**累加**成漂移（跑 1 kHz 时几毫秒的抖动很常见）。
        落后超过一个周期时**重锚**而不是连续补发 —— 补发会在落后的瞬间产生一波
        密集指令，对电机不友好。

        用法::

            while running:
                ctx.cycle_begin()
                j.set_mit(pos=..., ...)
                ctx.cycle_end()
                ctx.pace()          # 放到循环最后
        """
        period = self.period_ns
        if period <= 0:
            return
        now = time.perf_counter_ns()
        if self._next_deadline_ns == 0:
            self._next_deadline_ns = now
        self._next_deadline_ns += period

        remaining = self._next_deadline_ns - time.perf_counter_ns()
        if remaining > 0:
            time.sleep(remaining / 1e9)
            # sleep 之后再看一次：操作系统调度抖动也算"落后"
            late_ms = (time.perf_counter_ns() - self._next_deadline_ns) / 1e6
        else:
            # 本周期的工作还没做完就已经超出 deadline 了。**这里测到的落后量才是
            # 真正要报的值**（旧实现重锚后直接把 0 记进统计，于是"根本跟不上周期"
            # 这件事被指标自己掩盖成"最大落后 0.1 ms"）。
            late_ms = -remaining / 1e6
            # 重锚，不追赶：连续补发会在落后的瞬间产生一波密集指令，对电机和
            # 总线都不友好
            self._next_deadline_ns = time.perf_counter_ns()

        self._paced_cycles += 1
        if late_ms > self._pace_worst_late_ms:
            self._pace_worst_late_ms = late_ms

    def reset_pace(self) -> None:
        """丢弃累计的节拍基准（例如刚做完阻塞调用、或从暂停恢复时）。"""
        self._next_deadline_ns = 0

    def pace_stats(self) -> tuple[int, float]:
        """``(已走周期数, 最大落后 ms)``。

        第二个值包含两种情况：``sleep`` 之后被系统调度耽误的时间，以及
        **本周期工作本身就没在 deadline 前做完**的时间。后者才是要紧的 ——
        如果它稳定地大于 0，说明周期定得太短（或循环里有阻塞调用），
        此时 ``pace()`` 会每个周期都重锚、实际周期变成"工作耗时"，
        而不是你以为的 ``period_ns``。
        """
        return self._paced_cycles, self._pace_worst_late_ms

    # --- 总线级 -----------------------------------------------------------

    def discover(self, max_probe: int = 16) -> list[int]:
        """节点发现：被动听心跳（200 ms）+ 主动 ``QUERY_STATUS`` 探测 1..``max_probe``。

        ⚠ 不要在 :meth:`activate` 之后、控制回路运行期间调用（会争用响应）。
        ⚠ 阻塞。``max_probe=0`` 表示只被动听 —— 首次连接真机时请用默认值：
        主动探测的第一帧就足以让设备学到主站地址，纯被动可能什么都听不到。
        """
        ids = (ctypes.c_uint8 * 256)()
        found = ctypes.c_int(0)
        raise_for_status(self._lib.jsdk_context_discover(
            self._ctx_ptr, ids, 256, ctypes.byref(found), int(max_probe)),
            "discover", self.last_error())
        return [int(ids[i]) for i in range(found.value)]

    def bus_state(self) -> BusState:
        """链路与收发统计。"""
        c = _abi.BusState()
        raise_for_status(self._lib.jsdk_context_get_bus_state(self._ctx_ptr,
                                                              ctypes.byref(c)),
                         "get_bus_state")
        return BusState(c)

    def last_error(self) -> str:
        """最近一次错误的可读文本（含关节/给定量/量程）。"""
        msg = self._lib.jsdk_context_last_error(self._ctx_ptr)
        return msg.decode("utf-8", "replace") if msg else ""

    def estop(self) -> None:
        """广播 ``ESTOP(0xC0)``（最高仲裁优先级）。**不需要额外确认** —— 拒绝执行更危险。"""
        self._lib.jsdk_context_estop(self._ctx_ptr)

    # --- 描述符 / 端点 ----------------------------------------------------

    def desc_info(self) -> DescInfo:
        """描述符元信息。"""
        c = _abi.DescInfo()
        raise_for_status(self._lib.jsdk_context_get_desc_info(self._ctx_ptr,
                                                             ctypes.byref(c)),
                         "get_desc_info")
        return DescInfo(c)

    def lookup(self, path: str) -> Endpoint:
        """路径 → 端点。未命中抛 :class:`~jsdk_can.errors.JsdkDeviceNotFoundError`
        （**不猜、不近似**）。"""
        ep_id = ctypes.c_uint16(0)
        type_ = ctypes.c_int(0)
        access = ctypes.c_uint8(0)
        raise_for_status(self._lib.jsdk_endpoint_lookup(
            self._ctx_ptr, path.encode("utf-8"), ctypes.byref(ep_id),
            ctypes.byref(type_), ctypes.byref(access)),
            f"lookup({path})", self.last_error())
        try:
            type_enum: EpType | int = EpType(type_.value)
        except ValueError:  # pragma: no cover
            type_enum = type_.value
        return Endpoint(path, int(ep_id.value), type_enum, int(access.value))

    def endpoints(self) -> list[Endpoint]:
        """枚举已保留的全部端点。

        ⚠ 这是唯一使用 ctypes 回调的地方：``jsdk_endpoint_enumerate`` 需要逐条
        回调。它只在配置阶段（非 RT）运行，GIL 开销可接受。
        """
        out: list[Endpoint] = []

        def _visit(_ctx, path, ep_id, type_, access) -> int:
            try:
                t: EpType | int = EpType(type_)
            except ValueError:  # pragma: no cover
                t = type_
            out.append(Endpoint(path.decode("utf-8", "replace"),
                                int(ep_id), t, int(access)))
            return 0

        cb = _abi.ENDPOINT_VISIT_FN(_visit)
        st = int(self._lib.jsdk_endpoint_enumerate(self._ctx_ptr, cb, None))
        raise_for_status(st, "endpoint_enumerate", self.last_error())
        return out

    def endpoints_matching(self, substring: str) -> list[Endpoint]:
        """按**子串**过滤端点（现场最常见：``"mit_max_"``）。"""
        return [e for e in self.endpoints() if substring in e.path]

    def dump_config(self) -> ConfigSnapshot:
        """读回关键标定量（多关节时取第一个）。

        :raises JsdkUsageError: 尚未 :meth:`configure`
        """
        if not self._joints:
            raise error_for_status(-1, "dump_config", "没有关节")
        return self._joints[0].config_snapshot()

    def device_info(self) -> DeviceInfo:
        """``QUERY_DEVICE_INFO(0x46)``（多关节时取第一个）。"""
        if not self._joints:
            raise error_for_status(-1, "device_info", "没有关节")
        return self._joints[0].device_info()

    def query_error_detail(self) -> FaultInfo:
        """``QUERY_ERROR(0x45)`` 六类错误明细（多关节时取第一个）。"""
        if not self._joints:
            raise error_for_status(-1, "query_error_detail", "没有关节")
        return self._joints[0].query_error_detail()

    def desc_export(self) -> bytes:
        """导出描述符缓存（路线 A：已解析并按 retain 裁剪的结果）。

        ⚠ 缓存与 ``retain`` / ``filter_paths`` / SDK 内部格式三者绑定；改任一项
        都要重新下载。要跨版本共用请缓存**原始 JSON**（C 侧 ``raw_sink``）。
        """
        cap = int(self._lib.jsdk_desc_export_max_size(self._ctx_ptr))
        if cap <= 0:
            cap = 64 * 1024
        buf, ptr = _abi.alloc_aligned(cap)
        out_len = ctypes.c_size_t(0)
        raise_for_status(self._lib.jsdk_context_desc_export(
            self._ctx_ptr, ptr, cap, ctypes.byref(out_len)),
            "desc_export", self.last_error())
        return ctypes.string_at(ptr, out_len.value)

    def desc_import(self, blob: bytes) -> None:
        """导入 :meth:`desc_export` 的产物（**不下载**）。

        ⚠ 导入的只是**描述符**，不会代替 :meth:`configure`：后者仍要跑握手与标定
        （C 侧发现描述符已在手就不会再下载）。早期版本这里把“已配置”标记置上了，
        于是 `desc_import()` 之后的 `configure()` **静默什么都不做** —— 没握手、
        没标定，之后所有涉及物理量的操作都拿到 0（见回归用例）。
        """
        buf = ctypes.create_string_buffer(bytes(blob), len(blob))
        raise_for_status(self._lib.jsdk_context_desc_import(
            self._ctx_ptr, ctypes.cast(buf, ctypes.c_void_p), len(blob)),
            "desc_import", self.last_error())

    # --- 描述符：显式下载 / 非阻塞推进 / 原始 JSON 缓存 -------------------

    def desc_fetch(self) -> None:
        """下载并解析描述符（**阻塞**，= ``configure()`` 里的第一步）。

        ⚠ 禁止在任一关节使能时调用（描述符帧会挤掉控制帧 → 触发设备的
        ``break_timeout``）。要那一步的细节请直接用 :meth:`configure`。
        """
        raise_for_status(self._lib.jsdk_context_desc_fetch(self._ctx_ptr),
                         "desc_fetch", self.last_error())

    def desc_poll(self, app_time_ns: int = 0) -> bool:
        """**非阻塞**推进描述符下载（裸机/MCU 主循环用）。

        :return: ``True`` = 已完成；``False`` = 还在进行中（``BUSY``）
        :raises JsdkError: 真错误（超时/解析/arena 不足）

        用法::

            ctx.desc_fetch_start()            # 或直接反复调 desc_poll()
            while not ctx.desc_poll():
                ...                            # 你自己的主循环
        """
        st = int(self._lib.jsdk_context_desc_poll(self._ctx_ptr, int(app_time_ns)))
        if st == int(Status.BUSY):
            return False
        raise_for_status(st, "desc_poll", self.last_error())
        return True

    def desc_import_raw(self, json_bytes: bytes, *, crc: int,
                        fw_version: int) -> None:
        """用**原始 JSON** 建表（路线 B：从 Flash/磁盘缓存恢复，不经 CAN）。

        :param crc: 描述符 ``VersionCRC``（下载时从 ``desc_info().crc`` 取得）
        :param fw_version: 设备固件版本（``device_info().fw_version``）

        与 :meth:`desc_import` 的区别：那个导入 SDK 自己的紧凑格式（已解析、
        已按 retain 裁剪）；本方法导入设备原始 JSON，**按当前 cfg 重新解析**，
        因此改 ``retain`` / ``filter`` 不用重新下载。

        ⚠ 与 :meth:`desc_import` 一样：**不会**代替 :meth:`configure`（仍需握手与标定）。
        """
        b = bytes(json_bytes)
        buf = ctypes.create_string_buffer(b, len(b))
        hint = _abi.DescHint()
        hint.crc = int(crc)
        hint.fw_version = int(fw_version)
        raise_for_status(self._lib.jsdk_context_desc_import_raw(
            self._ctx_ptr, ctypes.cast(buf, ctypes.c_void_p), len(b),
            ctypes.byref(hint)), "desc_import_raw", self.last_error())

    def desc_raw_sink(self, cb) -> None:
        """安装/取消**原始 JSON 流出**回调（路线 B 的写侧）。``cb=None`` 取消。

        Python 回调形如 ``cb(data: bytes, offset: int) -> bool | None``：
        返回 ``True`` 以外的非 0 值表示放弃（下载继续，但 raw 缓存不完整）。

        ⚠ ``data`` **只在回调期间有效**（这里已经替你复制成 bytes）；不要把
          它缓存成 memoryview 到回调之外用。
        ⚠ 必须配合 ``desc_remain_all``/``stop_when_satisfied = 0``，否则流会被
          提前终止 → 缓存不完整（``desc_info().complete == 0`` 时不要写缓存）。
        """
        if cb is None:
            self._callbacks.pop("raw_sink", None)
            self._lib.jsdk_context_set_desc_raw_sink(self._ctx_ptr, None, None)
            return

        def _trampoline(_ctx, data, length, offset, _user):
            try:
                r = cb(ctypes.string_at(data, length), int(offset))
            except Exception:        # 回调里抛异常不能穿过 C 栈
                return 1
            return 0 if r is None else (0 if r else 1)

        fn = _abi.DESC_RAW_SINK_FN(_trampoline)
        self._callbacks["raw_sink"] = fn
        self._lib.jsdk_context_set_desc_raw_sink(self._ctx_ptr,
                                                 ctypes.cast(fn, ctypes.c_void_p),
                                                 None)

    def desc_progress(self, cb) -> None:
        """安装/取消下载进度回调（``cb(done, total)``）；``cb=None`` 取消。"""
        if cb is None:
            self._callbacks.pop("progress", None)
            self._lib.jsdk_context_set_desc_progress(self._ctx_ptr, None, None)
            return

        def _trampoline(_ctx, done, total, _user):
            try:
                cb(int(done), int(total))
            except Exception:
                pass                     # 进度回调不该影响下载

        fn = _abi.DESC_PROGRESS_FN(_trampoline)
        self._callbacks["progress"] = fn
        self._lib.jsdk_context_set_desc_progress(self._ctx_ptr,
                                                 ctypes.cast(fn, ctypes.c_void_p),
                                                 None)

    def on_fault(self, cb) -> None:
        """安装/取消故障**边沿**回调（0→1 只报一次）；``cb=None`` 取消。

        Python 回调形如 ``cb(joint, info: FaultInfo)``。

        ⚠ 它在 ``cycle_begin/end`` 里被调用（控制路径）：回调里**不要**做重活，
          也**不得**重入 SDK（会踩到正在使用中的内部状态）。硬实时场景请改用
          轮询 ``joint.is_fault()`` / ``joint.fault_info()``。
        """
        if cb is None:
            self._callbacks.pop("fault", None)
            self._lib.jsdk_context_set_fault_callback(self._ctx_ptr, None, None)
            return

        def _trampoline(jptr, info_ptr, _user):
            try:
                cb(self._joint_by_ptr(jptr), FaultInfo._from_c(info_ptr.contents))
            except Exception:
                pass                     # 异常不能穿过 C 栈

        fn = _abi.FAULT_CALLBACK_FN(_trampoline)
        self._callbacks["fault"] = fn
        self._lib.jsdk_context_set_fault_callback(
            self._ctx_ptr, ctypes.cast(fn, ctypes.c_void_p), None)

    def _joint_by_ptr(self, jptr):
        """把 C 传来的 ``jsdk_joint_t*`` 换回 Python 对象（找不到就返回 None）。

        ⚠ 两侧的指针形态不一样：C 回调给的参数是 **int**，而 ``Joint._ptr`` 存的是
          ``c_void_p`` 实例 —— 直接 ``==`` 永远不相等（真踩过：回调里 joint 恒为
          None）。统一用 ``ctypes.cast(...).value`` 归一化再比。
        """
        def _addr(p):
            try:
                return int(ctypes.cast(p, ctypes.c_void_p).value or 0)
            except (TypeError, ValueError):
                return 0

        want = _addr(jptr)
        if want == 0:
            return None
        for j in self._joints:
            if _addr(j._ptr) == want:
                return j
        return None

    # --- 关节集合 ---------------------------------------------------------

    @property
    def joints(self) -> list[Joint]:
        return list(self._joints)

    def joint(self, node_id: int) -> Joint:
        """按 node_id 取关节。"""
        for j in self._joints:
            if j.node_id == int(node_id):
                return j
        raise error_for_status(-3, f"joint({node_id})", "该节点不在本上下文里")

    def __iter__(self) -> Iterator[Joint]:
        return iter(self._joints)

    def __len__(self) -> int:
        return len(self._joints)

    # --- 便捷批量操作 -----------------------------------------------------

    def enable_all(self, mode: Mode = Mode.MIT) -> None:
        """逐个使能并等到全部就绪（等价于 ``activate()``，但可指定模式）。"""
        for j in self._joints:
            j.enable(mode)
        self.activate()

    def disable_all(self) -> None:
        """安全失能：先 hold_position，再逐个 disable，最后等 IDLE。

        顺序与 :meth:`deactivate` 一致 —— 直接停发控制帧会触发设备看门狗。
        """
        for j in self._joints:
            j.hold_position()
        self.poll()
        for j in self._joints:
            j.disable()
        for _ in range(50):
            self.poll()
            if not any(j.is_enabled() for j in self._joints):
                return
            if isinstance(self.hal, VirtualHal):
                self.hal.advance_ms(1)

    def group_set_mit(self, targets: dict[int, dict]) -> None:
        """一条 CAN FD 帧同时驱动多个关节。

        :param targets: ``{node_id: {"pos":…, "vel":…, "kp":…, "kd":…, "tau":…}}``
            （键可省略，缺省 0）。**node_id 必须 ∈ 1..7**（位图寻址上限）。

        ⚠ 必须在 ``cycle_begin()`` 与 ``cycle_end()`` 之间调用**一次**；
        成员必须已使能、已标定、且当前为 MIT 模式，否则抛 ``BAD_STATE``。
        ⚠ Classic 模式下仅支持"全员同一目标"，目标不一致会自动降级为单播
        （返回值仍成功，原因在 :meth:`last_error`）。
        """
        if not targets:
            raise ValueError("targets 不能为空")
        if len(targets) > 7:
            raise error_for_status(-6, "group_set_mit",
                                   "位图寻址最多 7 个关节（node_id 1..7）")

        n = len(targets)
        arr = (_abi.GroupTarget * n)()
        for i, (node, spec) in enumerate(sorted(targets.items())):
            arr[i].node_id = int(node)
            arr[i].pos_rad = float(spec.get("pos", 0.0))
            arr[i].vel_rad_s = float(spec.get("vel", 0.0))
            arr[i].kp = float(spec.get("kp", 0.0))
            arr[i].kd = float(spec.get("kd", 0.0))
            arr[i].tau_Nm = float(spec.get("tau", 0.0))

        raise_for_status(self._lib.jsdk_group_set_mit(self._ctx_ptr, arr, n),
                         "group_set_mit", self.last_error())

    def group_enable(self, node_ids: list[int]) -> None:
        """逐个下单播使能请求（协议**没有**广播版 START_MOTOR）。"""
        self._group_fan_out(node_ids, self._lib.jsdk_group_enable, "group_enable")

    def group_disable(self, node_ids: list[int]) -> None:
        """逐个下单播失能请求。"""
        self._group_fan_out(node_ids, self._lib.jsdk_group_disable, "group_disable")

    def _group_fan_out(self, node_ids: list[int], fn, op: str) -> None:
        if not node_ids:
            raise ValueError("node_ids 不能为空")
        arr = (ctypes.c_uint8 * len(node_ids))(*[int(n) for n in node_ids])
        raise_for_status(fn(self._ctx_ptr, arr, len(node_ids)), op,
                         self.last_error())

    def __repr__(self) -> str:  # pragma: no cover - 人读
        return (f"<Context hal={self.hal.name} joints={len(self._joints)} "
                f"configured={self._configured}>")
