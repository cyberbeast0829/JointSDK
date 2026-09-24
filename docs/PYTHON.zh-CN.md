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
| 依赖 | Python ≥ 3.8、`ctypes`。**没有** numpy 之类的硬依赖 |
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
    is_fd=None,                  # None（默认）= 先按 FD 试、收到本关节第一帧时自动对齐并在 stderr 说明；
                                 # True/False = **明确指定**，冲突时不改（`ctx.framing_learned == 4`）
    is_fd_explicit=None,         # 高级用法：单独控制“能不能被自动对齐覆盖”。
                                 #   想“先按 Classic 起步、听准了再对齐”：is_fd=False,
                                 #   is_fd_explicit=False（两个 CLI 的自动模式就是这么干的）
    desc_retain=DescRetain.ALL,  # 保留全部端点（约 24.9 KB RAM）；只留关键路径可降到 <1 KB
    desc_filter=["axis0.motor.config.gear_ratio", "axis0.config.can.node_id"],
    period_ns=1_000_000,         # 期望周期（keepalive 与超时判定用）
    state_timeout_ms=0,          # 等状态序列跑完的预算；0 = 内置默认（标定 120 s / 回零 5 s）
    auto_keepalive=True,
    max_joints=8,
)

j = ctx.add_joint(1)             # 必须在 configure() 之前
ctx.warmup()                     # 会话预热（幂等重发）：挡掉 slcan "首帧丢失"
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

**会话预热（`ctx.warmup()`）**：真机上**第一条命令随机超时**（`0/0 bytes` + 心跳正常，
再敲一次就好）的根因是 **slcan 的“首帧丢失”** —— 适配器打开端口时重置输入缓冲，
主站头一两帧上不了总线，而 Lawicel 对帧行**不回报结果**（`acks/nacks` 恒 0）⇒
主机侧**没有任何可观测信号**。唯一可验证的解法是幂等请求 + 重发，
也就是 `ctx.warmup()`：反复发一条只读的 `QUERY_DEVICE_INFO(0x46)`，每轮等 50 ms、
默认总预算 500 ms；返回 `True` = 设备应答了，`False` = 预算内没应答
（**不是致命错误**，只收不发的命令照样能用；但别当成功 —— 设备掉电/线松最先在这里暴露）。

```python
if not ctx.warmup():
    print("链路没应答：", ctx.last_error())
print(ctx.bus_state().tx_retries)      # 总共重发了几次（会话预热 + 幂等请求）
print(ctx.bus_state().tx_retries_req)  # 其中“**运行中途**丢帧”那部分（持续非 0 = 链路有问题）
print(ctx.bus_state().tx_retries_warm) # 会话预热那部分（开头丢帧，已知无害）
print(ctx.bus_state().req_timeouts)    # 等超时的次数（含被重发救回的 —— 那才是丢过帧的证据）
print(ctx.bus_state().last_retry_what, ctx.bus_state().last_retry_age_ms)  # 最近一次重发
print(ctx.desc_info().retries)         # 描述符下载重发 0x24 的次数（请求丢了的直接证据）
```

⚠ SDK 已经把它**自动挂在发帧之前**（`Context` 内部，覆盖库用户），所以不调用也不会
再“莫名其妙失败”；显式调用是为了**尽早知道链路坏了**，以及让只收不发的命令也先确认链路。
⚠ 同一个会话里成功后**再调是空操作**；失败后自动预热不会反复花时间，但**显式**调用仍可重试。
⚠ 需要至少一个关节（用它定 `node_id`），否则 `JsdkStateError`（而不是乱发一帧）。

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
  ⚠ **只在设备侧超时 > 0 时**才补 —— `break_timeout = 0`（= 禁用）时一帧都不补；
- 设备的 `break_timeout` 由 `configure()` 在**需要时**按 `period_ns` 设定
  （`enable_watchdog_hint`：当前为 0 或比周期还短时才写，写完读回确认）；
- `j.set_watchdog_ms(0)` = **关闭**设备侧协议级超时检测（最新固件语义：
  `auto_stop_if_timeout()` 首句就是 `if (timeout_ms == 0) return;`）。
  注意保护只在该设备**收到过控制帧后**才武装 —— 纯电流模式武装不了（固件 F19）；
- **写后读回校验**：`set_watchdog_ms()` 把读回分成三类：等于写入值（含写 0）→ 校验通过；
  **读不回来**（超时）→ 置 `StatusFlag.WATCHDOG_UNVERIFIED`、保留写入值（继续喂狗是安全方向）、但返回成功；
  其它值 → 报 `PROTOCOL`。CLI 两版都会**独立再读一次设备**并输出 `device_reports_ms` + `verified`，不回显写入值。
  真机实测（COM3 / fw 1545）：`watchdog 300` → `{"ms":300,"device_reports_ms":300,"verified":true}` ✓
  （历史上“该端点读回恒为 0”的结论是**误判**，根因见 `FIRMWARE_ISSUES` F28：当时 SDK 发的参数
  写帧不足 8 字节，被固件 `if (msg.len < 8) return;` 静默丢弃）
- 纯 CURRENT 模式的客户端**不会**武装设备的超时保护（固件问题 F19，见 `FIRMWARE_ISSUES.zh-CN.md`），别把它当安全兜底。

---

## 6.1 平台支持与库定位（先看这一节）

| 平台 | 支持 | 加载器找的名字 | 实测 |
|---|---|---|---|
| Windows | ✅ | `jsdk_can.dll` / `libjsdk_can.dll` | **212 passed / 2 skipped**（Python 3.12） |
| Linux | ✅ | `libjsdk_can.so` / `.so.0` / `jsdk_can.so`（另对包内 `lib/` glob 兜底） | **212 passed / 2 skipped**（Ubuntu 20.04 / Python 3.8.10） |
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

**输出编码（B10）**：`python -m jsdk_can` 在**控制台**上由 Python 自己按 UTF-16 写，
中文一直是好的；但**重定向**（管道/文件）时 Python 用的是 locale 编码（中文 Windows 上
是 cp936），与 C 版 `jsdk-cli`（重定向时写 UTF-8）**不一致**，而且输出里一旦出现
CP936 表示不了的字符（如 `⚠`）会直接 `UnicodeEncodeError` 崩掉。
现在 `__main__.py` 在**重定向时把 stdout/stderr 钉成 UTF-8**（控制台保持 Python 的处理）。
回归用例：`tests/test_errors_and_cli.py::test_redirected_output_is_utf8`
（它**故意不设** `PYTHONIOENCODING` —— 设了就把这个坑盖回去了）。

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
j.param_set_auto("can.config.break_timeout", 250)       # ✅ 按**端点声明的类型**装箱
j.param_get_batch(p1, p2, p3, p4)                      # FD 下打包成一帧；Classic 自动逐条
```

`param_set` 与 `param_set_auto` 的分工（**两个都要有**）：

| | 装箱依据 | 适用 |
|---|---|---|
| `param_set(path, v)` | **Python 类型**（`bool`→bool、`float`→f32、`int`→u32/i32） | 你确实知道端点类型；类型不符要**报错**（防止“靠猜写错值”） |
| `param_set_auto(path, v)` | **端点声明的类型**（先查描述符：u8/u16/i8/i16/u32/i32/u64/i64/bool/f32/f64 全覆盖，超值域报 `PROTOCOL`） | 命令行/脚本的便利入口：`param_set(path, 250)` 写 u16 端点会被当成 u32 发而被 C 侧拒（`descriptor=uint16 given=uint32`）——用这个就没坑 |

> ⚠ `param_set_auto` 修的是真发生过的事：Python 版 **u8/u16/i8/i16 端点根本写不进去**
> （看着像“写了没反应”），而 C 版 CLI 却可以。`python -m jsdk_can write` 现在就是走它。

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

## 9.1 与 C 公共 API 的对齐（**A13：已全覆盖，114/114**）

Python 绑定覆盖了公共 C API 的 **114/114 个函数，0 个未绑定**（`tools/_abi_gap.py` 可复核）：

```console
$ python tools/_abi_gap.py
公共 API 总数        : 114
已绑定函数总数        : 114
未绑定（缺口）        : 0
```

> 上一版这份文档写的是“112 个里 89 个”（**23 个未绑定**）。那份清单已经**全部关掉**：
> 7 个 SDO + 4 个单位/标度 + 5 个描述符缓存/回调 + 3 个 raw 逃生通道 + 1 个 slcan 统计
> + 上下文创建/销毁 + 轮询。行为与数字都由 `bindings/python/tests/test_full_surface.py`
> 的两个**双向**用例钉死：
>
> | 用例 | 做什么 | 为什么必须双向 |
> |---|---|---|
> | `test_all_public_functions_bound` | 扫头文件里的公共函数 → 逐个断言在 `_abi._FUNCS` 里 | 只扫源码文本会**漏掉已删除的签名**（实测：把参数删掉仍能过）—— 所以改为与 `_FUNCS` 对拍 |
> | `test_bound_functions_are_all_public_api` | 反向：`_FUNCS` 里每个名字都必须在头文件里 | 防止绑定里留着“已经不在 C API 里”的僵尸封装 |

### 新可用的能力（原来是缺口）

| 能力 | Python 入口 | 说明 |
|---|---|---|
| **链路质量观测** | `Context.bus_state()`（`tx_retries*` / `req_timeouts` / `last_retry_*`）、`Context.desc_info().retries` | 丢帧/重发按类别计数，`health` 的 `bus` 里也全有 —— 现场排“链路稳不稳”不用再靠猜 |
| **会话预热** | `Context.warmup(timeout_ms=0)` → `bool` | 幂等重发，挡掉 slcan 的**首帧丢失**（现场：第一条命令随机超时、再敲一次就好）。`False` = 预算内没应答（不致命）；次数在 `ctx.bus_state().tx_retries`。SDK 已自动挂在发帧之前，显式调用是为了**尽早**发现链路坏 |
| **SDO 风格端点访问** | `Joint.sdo(path_or_ep, *, subindex, size)` → `Sdo` 对象：`.state/.data/.size/.start_read()/.start_write()/.read()/.write()/.read_value()/.write_value()` | 非阻塞启动 + 阻塞等（内部抽 `cycle_begin/end`）；`.data` 是**裸线上字节**，参数值**小端**（见 `PROTOCOL_NOTES` §3.1）；`.read_value()/.write_value()` 按 `size` 帮你解成 int/float。同一个端点会**复用**句柄（`sdo_slots_used` 可查） |
| **单位与标度** | `units.UnitScale`、`unit_scale_default()`、`unit_scale_calc(enc, motor_rev, shaft_rev, rated)`、`Joint.set_scale()/get_scale()` | CAN 上一般不需要（线上量已是物理量，默认标度是恒等映射）——但工具类/换算场景现在不用自己手算 |
| **描述符缓存与回调** | `Context.desc_fetch()` / `desc_poll()` / `desc_import_raw(json, crc=, fw_version=)` / `desc_raw_sink(cb)` / `desc_progress(cb)` / `on_fault(cb)` | 路线 B 全开：可拿**原始描述符字节**、可看进度、可注册故障回调（边缘触发，只变沿时回调）。回调异常会被吞掉并保留引用，不会因 GC 丢 |
| **raw 逃生通道** | `Joint.set_position_raw()` / `set_velocity_raw()` / `set_torque_raw()` | 与物理量入口（`set_target_position_rad()` 等）并存；raw 是**逐模式语义不同**的那一层，文档里标注了风险 |
| **诊断** | `SlcanHal.stats()` → `dict(tx, rx, malformed, acks, nacks)` | 真机掉帧/适配器问题排查用（仅 slcan 后端） |
| **电流** | `Joint.set_current(A)` | ⚠ **不喂看门狗**（`CURRENT_CONTROL` 不在 `is_ctrl` 里，见 `FIRMWARE_ISSUES` F19）→ 需要自己周期性重发 |

### 仍然没有“必需保证”的少数（不是缺口，是构建开关）

`_REQUIRED_FUNCS`（加载时**必须**找到的符号）**不含**这两类，因为它们依赖构建选项；
绑定会按需探测，库里有就能用：

| 函数 | 原因 |
|---|---|
| `jsdk_context_create` / `jsdk_context_free` | 堆模式（`-DJSDK_ENABLE_HEAP=ON`）；用 `Context()` 时 Python 自己管 `jsdk_context_storage_t` 那块内存 |
| `jsdk_hal_slcan_*`（含 `stats`） | 需要编 slcan 后端；纯虚拟/socketcan 构建里没有 |

用途很明确：**“库比绑定旧”要在加载那一刻就报错**，而不是等用户调到某个函数才
`AttributeError`（报错文本会直接告诉你重新 `-DJSDK_BUILD_SHARED=ON` 构建，
或用 `JSDK_LIB_PATH` 指对文件）。

---

## 10. 命令行入口（与 `jsdk-cli` 对齐，**25 个子命令**）

```bash
# 只读类
python -m jsdk_can scan --json
python -m jsdk_can health
python -m jsdk_can mon --duration 5 --json        # NDJSON
python -m jsdk_can mon --csv > run1.csv           # CSV 到 stdout（与 C 版逐字节同列）
python -m jsdk_can mon --csv-file run1.csv        # 额外落文件
python -m jsdk_can read axis0.motor.config.gear_ratio --json
python -m jsdk_can batch-read a b c --json
python -m jsdk_can dump-config --json
python -m jsdk_can desc-info --json
python -m jsdk_can ep-list --filter mit_max_ --json
python -m jsdk_can ep-lookup axis0.config.can.node_id --json
python -m jsdk_can desc-export desc.bin

# 写类：**需要 --yes**
python -m jsdk_can write --yes axis0.config.can.heartbeat_rate_ms 100 --json
python -m jsdk_can watchdog --yes 250 --json
python -m jsdk_can save --yes
python -m jsdk_can set-node-id --yes 2
python -m jsdk_can reset --yes
python -m jsdk_can set-zero --yes
python -m jsdk_can calibrate --yes                  # 会动电机
python -m jsdk_can home --yes                       # 会动电机

# 驱动：--yes **且** --hold（秒，1~60）
python -m jsdk_can mit --yes --hold 2 --pos 0.5 --kp 1 --kd 0.1 --tau 0

# 急停：广播、**不需要** --yes（安全动作不能因为少个参数而失败）
python -m jsdk_can estop
```

### 与 C 版 `jsdk-cli` 的契约（同一份，不是“差不多”）

| 项 | 规则 |
|---|---|
| 退出码 | `0` 成功 / `1` 运行时错误 / `2` 用法错误 / `3` 被安全闸拦住 |
| 安全闸 | `write`/`watchdog`/`save`/`set-node-id`/`reset`/`set-zero`/`calibrate`/`home`/`mit` 需要 `--yes`；`mit` 还要 `--hold ∈ [1,60]`；**`estop` 不需要** |
| 描述符档位 | 无描述符即可跑：`scan`/`info`/`err`/`hb-dump`/`estop`/`desc-import`；仅需描述符：`read`/`batch-read`/`write`/`save`/`set-node-id`/`reset`/`desc-info`/`desc-export`/`ep-list`/`ep-lookup`；需要完整配置（含标定才能解析的端点）：`health`/`dump-config`/`watchdog`/`mon`/`set-zero`/`calibrate`/`home`/`mit` |
| `--json` 字段 | 与 C 版**逐字段对齐**（有测试：`test_cli_json_field_parity_with_c_cli`） |
| 闸的顺序 | **先判闸、后开总线** —— 少了 `--yes` 的写命令在虚拟后端/没接硬件时也要得到 `3`，而不是“连不上” |

两版入口都对**同一份** `_COMMAND_SMOKE`（24 条，每个子命令一条）跑虚拟后端冒烟：
`test_every_subcommand_runs_on_virtual`（Python）与
`test_every_subcommand_runs_on_virtual_c_cli`（C）。

> 已知差异（都有测试注明）：① C 版 `mon` 在虚拟后端**不受墙钟约束**（虚拟时钟由循环驱动），所以 C 侧冒烟跳过 `mon`；
> ② `hb-dump` 的“原始字节”输出只在虚拟后端有（真机后端拿不到原始帧）。

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
| `test_wide_params.py` | 8 字节参数读（分段）、线上原始字节按小端断言 |
| `test_slcan_fd.py` | slcan 编码器（四个帧前缀、FD DLC 码）与真机后端构造 |
| `test_errors_and_cli.py` | 状态码↔异常、真实错误映射、`python -m jsdk_can` 子进程冒烟与**安全闸** |
| `test_review_regressions.py` | 自审发现的回归（`pace()` 指标、必需符号、加载异常家族） |
| `test_full_surface.py` | **A13 覆盖面守卫**：114/114 绑定（双向核对 `_abi._FUNCS` ↔ 头文件）、SDO/单位标度/描述符回调/故障回调、**24 个子命令的虚拟后端冒烟**（Python 与 C 各一遍）、与 C 版的 JSON 字段对拍 |

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
