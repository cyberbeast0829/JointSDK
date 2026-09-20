# Python 绑定（`jsdk_can`）使用与实现说明

> 面向：用 Python 做配置、采集、可视化、原型验证的同事。
> 方案与取舍见 `DESIGN.zh-CN.md` §7.2；CLI 见 `CLI.zh-CN.md`。
> 本文只讲**怎么用**与**为什么这么设计**，不复述 C API。

---

## 1. 定位与边界

| | |
|---|---|
| 做什么 | 参数配置、标定、状态采集、端点浏览、原型控制回路、离线仿真 |
| 不做什么 | 硬实时控制（GIL + 调度抖动，实测 1 kHz 下落后 1~3 ms）、多线程并发访问同一 `Context` |
| 依赖 | Python ≥ 3.9、`ctypes`。**没有** numpy 之类的硬依赖 |
| 后端 | 与 C 侧同一套：`virtual` / `socketcan` / `pcan` / `slcan` |

**为什么用 ctypes 而不是 CFFI/扩展模块**：客户侧不需要编译器；而且共享库
（`-DJSDK_BUILD_SHARED=ON`）本来就是给 FFI 用的，导出面只有公共 ABI。
代价是结构体布局要自己对齐 —— 所以有了 §3 的 ABI 自检。

---

## 2. 安装与库定位

```bash
# 开发（源码树内）
pip install -e "bindings/python[test]"

# 构建共享库（同时把库复制进包内 lib/）
cmake -S . -B bsh -G "MinGW Makefiles" -DJSDK_BUILD_SHARED=ON -DJSDK_BUILD_PYTHON=ON
cmake --build bsh
```

库查找顺序（`jsdk_can.library_search_path()` 可打印实际候选）：

1. `JSDK_LIB_PATH`（文件或目录）
2. 包内 `jsdk_can/lib/`
3. 仓库 `build/`、`build-shared/`、`build/Release/`

找不到时抛 `LibraryNotFoundError`，异常文本里列出**每个候选与失败原因**
（Windows 上常见的是"找不到依赖项"而不是"文件不存在"，所以原文要留着）。

---

## 3. 加载即自检：为什么值得多花这一步

`Context` 一构造就做四件事，任何一件不通过就**直接抛异常，不允许继续**：

| 检查 | 不做的后果 |
|---|---|
| 逐类型比对 `sizeof` / 对齐（C 侧 `jsdk_abi_types()` ↔ ctypes） | 结构体错位 → `ctypes` 写越界/读到垃圾，症状是"数字莫名其妙" |
| 后端自报家门必须 `cyberbeast-can` | 家族里三个后端符号同名，混用 = 静默内存错配 |
| `JSDK_API` 必需符号齐全 | 老库缺少新符号，报出来的是 `AttributeError` 而不是"版本不对" |
| 每个 `Status` ↔ 异常类自洽（参数化测试） | 文档承诺 `except JsdkStateError`，实际抛的是 `JsdkUsageError` |

**残留风险（诚实说明）**：探针只覆盖 `sizeof`/对齐/名字，**不覆盖字段偏移**。
字段顺序被改而总尺寸恰好不变时，自检发现不了。兜底是行为测试：`tests/` 会对
`gear_ratio=16.5`、`node_id=180`、`mit_max_torque=50.0`、反馈各字段等逐项断言 ——
这类断言实际上就是在验证偏移。**改公共结构体字段时要同时跑 Python 套件。**

表本身是"**只增不改**"契约：新增结构体 = 往 `k_abi_types[]` 追加；改名/改序
= 破坏 ABI，必须同时改 `JSDK_ABI_VERSION_CAN`。

---

## 4. 生命周期

```python
from jsdk_can import Context, VirtualHal, SocketCanHal, DescRetain

ctx = Context(
    hal=VirtualHal(),            # 或 SocketCanHal("can0") / PcanHal("PCAN_USBBUS1") / SlcanHal("COM3")
    master_id=1,                 # 主站源地址
    is_fd=True,                  # False = 强制 Classic（描述符下载会从 662 帧涨到 6839 帧）
    desc_retain=DescRetain.ALL,  # 保留全部端点（约 24.9 KB RAM）；只留关键路径可降到 <1 KB
    desc_filter=["axis0.motor.config.gear_ratio", "axis0.config.can.node_id"],
    period_ns=1_000_000,         # 期望周期（keepalive 与超时判定用）
    auto_keepalive=True,
    max_joints=8,
)

j = ctx.add_joint(1)             # 必须在 configure() 之前
ctx.configure()                  # 下载描述符 → 解析 → 读回标定量并校验
ctx.activate()                   # 阻塞：使能 + 安全首帧；全有或全无
...
ctx.deactivate()                 # 阻塞：安全关机
ctx.close()                      # 或 with Context(...) as ctx:
```

**顺序约束（SDK 会拒绝而不是猜）**：

- `add_joint()` 在 `configure()` 之后调用 → `JsdkStateError`；
- 未 `configure()` 就 `activate()` → `JsdkStateError`，且**不改任何状态**；
- `discover()` 不要在控制回路运行期间调（会争用响应队列）。

---

## 5. 控制回路

```python
ctx.cycle_begin()                # 收帧 + 解码（RT 安全）
j.set_mit(pos=..., vel=..., kp=..., kd=..., tau=...)
ctx.cycle_end()                  # 编帧 + 发帧 + 补喂狗 + 推进超时
ctx.pace()                       # 睡到下一个周期边界
```

### 5.1 `pace()` 为什么必要

C 侧的 `cycle_end()` 是**裸机语义**：它只做"编帧 + 发帧"，周期由 RTOS 定时器提供。
Python 里如果不睡，循环会以"CPU 多快就发多快"的速度跑 —— 真机上这不是更跟手，
而是塞满总线、帧间隔抖动、看门狗判定失去意义。

`pace()` 按**绝对时间**累加 deadline（不是每周期 `sleep(周期)`，那会把工作耗时累加成
漂移），落后超过一个周期时**重锚而不追赶**（连续补发会在落后瞬间产生一波密集指令）。

`pace_stats()` 返回 `(周期数, 最大落后 ms)`。第二个值包含两种情况：`sleep` 后被系统
调度耽误的时间，以及**工作本身没在 deadline 前做完**的时间。后者稳定大于 0 就说明
周期定得太短，此时实际周期会变成"工作耗时"而不是你以为的 `period_ns`。

### 5.2 三种"使能"的区别（最容易踩）

| 调用 | 语义 | 返回后设备状态 |
|---|---|---|
| `j.enable()` | **请求**使能（非阻塞） | 还没变；要跑若干周期 |
| `ctx.activate()` | **阻塞**直到就绪（含安全首帧） | 确实已使能 |
| `j.disable()` | **请求**失能（非阻塞） | 还没变，实测 **5 个周期**才落地 |
| `ctx.deactivate()` | **阻塞**到确认断电 | `is_enabled()` 为 `False` |

⚠ **使能序列的第 4 步是"安全首帧"**：pos 取当前反馈位置（客户没设过目标时）、
vel/tau/kp/kd 取 0 —— 所以"使能瞬间"电机既不跳向残留目标也不出力。绕过序列直接发
运动指令就会跳，这是 WP3 修过的缺陷类别。

⚠ **`deactivate()` 会先掐掉所有排队中的请求**再做安全关机。否则一个陈旧的
`enable` 请求会在返回后的下一个周期把电机**重新使能** —— "我明明关机了"。
（这条是 WP8 自审实测复现的缺陷，已修并有回归用例。）

---

## 6. 主站节拍与看门狗
- `auto_keepalive=True`（默认）时，`cycle_end()` 会按 `period_ns` 自动补喂狗帧；
- 设备的 `break_timeout` 由 `configure()` 按 `period_ns` 设定（`enable_watchdog_hint`）；
- `j.set_watchdog_ms(0)` **不是关闭**：固件把 0 解释为 100 ms。要放宽就写大值；
- 纯 CURRENT 模式的客户端**不会**武装设备的超时保护（固件问题 F19，见 `FIRMWARE_ISSUES.zh-CN.md`），别把它当安全兜底。

---

## 6.1 平台支持与库定位（先看这一节）

| 平台 | 支持 | 加载器找的名字 | 实测 |
|---|---|---|---|
| Windows | ✅ | `jsdk_can.dll` / `libjsdk_can.dll` | 全套装 142 项通过 |
| Linux | ✅ | `libjsdk_can.so` / `.so.0` / `jsdk_can.so`（另对包内 `lib/` glob 兜底） | **全套 142 项通过**（Ubuntu 20.04 / Python 3.8.10） |
| macOS | ✅（未实测） | `libjsdk_can.dylib` / `jsdk_can.dylib` | 本仓无 macOS 机器 |

**Python 版本**：`>=3.8`（3.8 与 3.12 都跑通了全套装）。
包内只用到注解层面的新语法（`list[str]` / `X | None`），各模块靠
`from __future__ import annotations` 让 3.8 也能 import。

**库从哪来**（按优先级，见 `_abi.py` 的 `library_search_path()`）：

1. `JSDK_LIB_PATH`（环境变量，可指向**文件或目录**）—— 开发时指向 `build-shared/` 最方便；
2. **包内 `lib/`**：先试平台候选名，再对目录做一次 `libjsdk_can.*` / `jsdk_can.*` **glob 兜底**；
3. 仓库内的 CMake 构建目录（`build/`、`build-shared/`、`build/Release/`）；
4. 交给操作系统加载器（`PATH` / `LD_LIBRARY_PATH` / `DYLD_*`）。

构建：`cmake -S . -B bsh -DJSDK_BUILD_SHARED=ON -DJSDK_BUILD_PYTHON=ON && cmake --build bsh`
——`JSDK_BUILD_PYTHON=ON` 会把共享库拷进包内 `lib/`，并**只保留本平台那一份**（别的平台的名字会被清掉）。

> ⚠ **包内库名必须是不带版本号的名字**。Linux/macOS 的共享库默认带完整版本号
> （`libjsdk_can.so.0.1.0` / `libjsdk_can.0.1.0.dylib`），早期 CMake 用
> `$<TARGET_FILE:>` 直接拷过去，于是**包自带库却找不到** —— 而 Windows 因为 dll
> 不带版本号一直没暴露，外表看起来就像"绑定只支持 Windows"。
> 现在两道防线都在：CMake 拷成规范名 + 加载器 glob 兜底；
> 回归用例 `tests/test_review_regressions.py::test_bundled_library_is_discoverable`。
> 完整经过见 `BACKLOG.zh-CN.md` 的 **B9**。

---

## 7. 传输后端（HAL）

```python
from jsdk_can import VirtualHal, SocketCanHal, PcanHal, SlcanHal

VirtualHal("0:id=1,gear=16.5,fd")          # 内置驱动器模型，无需硬件
SocketCanHal("can0", 1_000_000, 5_000_000) # 位定时由内核管，先用 ip link 把链路 up
PcanHal("PCAN_USBBUS1", 1_000_000, 5_000_000)
SlcanHal("COM5", 115_200, 5_000_000)        # 串口速率 **不是** CAN 段速率
```

### 7.1 slcan 的 CAN FD

slcan **支持 CAN FD**（CANable 2.0 固件对 Lawicel 的扩展）。四个帧前缀：
`d/D`（BRS=0）、`b/B`（BRS=1），小写 = 标准帧。⚠ **`b/B` 是“带 BRS”**，字母与
BRS 的对应是反直觉的；DLC 位是 **FD 长度码**（`0..8` 直映，`9`→12 … `F`→64），
所以 9/10/11 字节在 FD 里**无法表示**（SDK 明确拒绝，不会静默按 12 字节发）。

`SlcanHal` 的第二个参数与第三个参数**不是**同一件事：

```python
SlcanHal("COM5", baud=115200, data_bitrate=5_000_000)
#                  ^ 串口波特率       ^ CAN FD 数据段速率
```

`data_bitrate` 的三种取值：

| 值 | 行为 |
|---|---|
| `5_000_000`（默认） | 打开时向适配器发 `Y5`（协议默认的 FD 数据段速率） |
| `2_000_000` | 发 `Y2` |
| `0` | **不碰适配器配置**（与 SocketCAN 的“内核管链路”同一姿态）；适配器保持自有设置 |
| 其它 | 抛 `JsdkUnsupportedError`：`Y<n>` 的索引是固件私有表，**不猜** |

诊断：`hal.fd_config()` → `(是否发过 Y<n>, 数据段速率)`；
`hal.fd_frames()` → `(发出 FD 帧, 收到 FD 帧)`。
**`rx_fd == 0` 而 `tx_fd > 0`** 通常说明对端在按 Classic 回（或适配器没进 FD 模式）
—— 比“反馈解析不出来”早一步指出现场问题。

> 真机验证请在**先**跑 `jsdk-cli --if slcan ... scan`（见 `PORTING.zh-CN.md` §7.5.3）。
> slcan 吞吐只有 100~500 fps，不适合高频控制。

## 8. 参数访问

```python
j.param_get("axis0.motor.config.gear_ratio")          # 类型由端点决定
j.param_get_u32("axis0.config.can.node_id")
j.param_get_bool("axis0.config.enable_watchdog")
j.param_set("axis0.controller.config.vel_limit", 5.0)  # 类型不符 → 明确拒绝
j.param_get_batch(p1, p2, p3, p4)                      # FD 下打包成一帧；Classic 自动逐条
```

设计原则：

- **类型不符宁可报错**（`JsdkProtocolError`），不做隐式转换、不静默截断、不猜量程；
- 标定量读不全时 `unit_scale.valid = 0`，此时物理量 API 会被拒绝 —— 因为"猜一个
  gear_ratio"比报错危险得多；
- `param_get_batch` 在 Classic 上退化为逐条读，但**返回值语义不变**。

---

## 9. 错误处理

```python
from jsdk_can import JsdkTransportError, JsdkStateError, JsdkError

try:
    ctx.configure()
except JsdkTransportError as e:      # 链路/总线不行了 → 重连
    print(e.status, e.op, e.detail)  # detail 是 C 侧原文（写给现场工程师的那句）
except JsdkStateError as e:          # 调用顺序不对、设备状态不允许
    raise
```

- 每个异常都带 `status`（`Status` 枚举）、`op`（操作名）、`detail`（C 侧可读文本，
  含关节号/给定量/量程）；
- 异常类 ↔ 状态码由**唯一入口** `error_for_status()` 决定，并有参数化用例遍历
  每个 `Status` 断言自洽；
- 加载期问题（库找不到、ABI 不匹配）也在 `JsdkError` 家族里，不会被
  `except JsdkError` 兜底时漏掉。

---

## 9.1 **未暴露的 C API**（绑定不完整的地方，读之前先知道）

Python 绑定覆盖了 112 个公共 C 函数里的 **89 个**，**23 个未绑定**。
不列出来会让人以为"绑的跟 C 一样全"，然后在需要时白白查一圈。

### 真实缺口（约 20 个，按需补）

| 类别 | 函数 | 现在怎么办 |
|---|---|---|
| **SDO 风格端点访问**（7） | `jsdk_joint_sdo_create` / `_by_name` / `_state` / `_data` / `_data_size` / `_read` / `_write` | 用 `Joint.param_get/param_set`（按路径，够用；拿不到句柄/状态机那套） |
| **单位与标度**（4） | `jsdk_unit_scale_default` / `_calc`、`jsdk_joint_set_scale` / `get_scale` | **CAN 上基本不需要**：线上量已是物理量，`jsdk_unit_scale_default()` 是恒等映射（见 `BACKLOG.zh-CN.md` §1.2 / A12） |
| **描述符缓存与回调**（5） | `jsdk_context_set_desc_raw_sink`、`jsdk_context_desc_import_raw`、`jsdk_context_set_desc_progress`、`jsdk_context_set_fault_callback`、`jsdk_context_desc_fetch` | `Context.configure()` 内部会取描述符（所以能跑）；但 **Python 侧用不了路线 B 原始缓存、也注册不了故障回调** —— 这两个是后续要补的重点 |
| **raw 逃生通道**（3） | `jsdk_joint_set_target_position` / `_velocity` / `_torque` | 用物理量入口（`set_target_position_rad` 等）；raw 入口在 Python 里**故意不绑**（语义逐模式不同，容易误用） |
| **诊断**（1） | `jsdk_hal_slcan_stats` | 用 `SlcanHal.fd_config()` / `fd_frames()`（统计接口未绑） |

### 有意不暴露（符合设计，不需要补）

| 函数 | 原因 |
|---|---|
| `jsdk_context_create` / `jsdk_context_free` | 堆模式（`-DJSDK_ENABLE_HEAP=ON`）；Python 自己管 `jsdk_context_storage_t` 那块内存 |
| `jsdk_context_desc_poll` | 裸机/MCU 的轮询路径；Python 用阻塞式 `configure()` |

> 完整清单与修法排序见 **`BACKLOG.zh-CN.md` §1.3（A13）**。
> 本节的内容是"补文档"而不是"补功能" —— 但**不写出来就是缺陷**：
> 读者会默认绑定是完整的。

---

## 10. 命令行入口（只读）

```bash
python -m jsdk_can scan --json
python -m jsdk_can health
python -m jsdk_can mon --duration 5 --json        # NDJSON
python -m jsdk_can read axis0.motor.config.gear_ratio --json
python -m jsdk_can dump-config --json
python -m jsdk_can desc-info --json
python -m jsdk_can ep-list --filter mit_max_ --json
python -m jsdk_can ep-lookup axis0.config.can.node_id --json
```

**只读**：写类子命令（`write`/`mit`/`save`/`set-zero`/`calibrate`）**根本没注册** ——
不是"有但被禁"，用 `--help` 也看不到。要让设备动作请用 `jsdk-cli`（C 实现，带
`--yes`/`--hold` 安全闸）。全局选项写在子命令之前或之后都可以。

---

## 11. 测试

```bash
JSDK_LIB_PATH=/path/to/bsh python -m pytest bindings/python/tests -q
```

| 模块 | 内容 |
|---|---|
| `test_abi.py` | ABI 自检、枚举数值与 C 侧逐项比对、库查找 |
| `test_lifecycle.py` | 生命周期、`configure`、`activate`/`deactivate`、描述符 |
| `test_joint.py` | 反馈解析、MIT 编码回读、参数类型、运维命令 |
| `test_errors_and_cli.py` | 状态码↔异常、真实错误映射、`python -m jsdk_can` 子进程冒烟 |
| `test_review_regressions.py` | 自审发现的回归（`pace()` 指标、必需符号、加载异常家族） |

全部使用 `virtual` 后端，**不需要硬件**。`JSDK_BUILD_SHARED=ON` 且装了 pytest 时，
CMake 会把它注册成 ctest 目标 `python_bindings`，于是 `ctest` 一条命令跑完 C + Python。

---

## 12. 实现要点（改绑定前先读）

1. **对齐分配**：`ctypes.create_string_buffer` 只有 1 字节对齐，拿它当 `jsdk_context_t`
   或 arena 是 UB（上下文里有 `double`，arena 里存指针）。用 `alloc_aligned()`（`c_uint64`
   数组 + 地址校验）。
2. **生命周期**：`cfg.hal = hal.hal` 是**结构体复制**，但 `user` 指针仍指向 Python 侧对象 ——
   `Context` 必须持有 HAL 对象引用；arena/描述符过滤字符串必须活到 `jsdk_context_destroy()`。
3. **导入环**：`enums` 依赖 `_abi` 的常量，所以 `_abi` **不能**在模块级 import `errors`
   （异常类因此放在 `errors.py`，`_abi` 按需 import）。
4. **`_FUNCS` 与头文件同步**：改 C ABI 后必须同步 `_abi.py` 的结构体、签名与
   `TYPE_MAP`，并跑 Python 套件 —— ABI 自检会先替你挡住大部分错误。
5. **不要用 `subprocess(text=True)` 收子进程输出**：它按本机 locale（Windows 上是 GBK）
   解码，而子进程按 UTF-8 写 → 中文变乱码、断言假失败。显式 `encoding="utf-8"`。
