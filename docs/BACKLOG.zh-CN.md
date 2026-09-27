# 待办与审计台账（BACKLOG）

> **本文件是"未完成项"与"历史盘点结论"的唯一出处。**
> `DESIGN.zh-CN.md`、`MIGRATION.zh-CN.md`、`PYTHON.zh-CN.md` 里出现的 `A*` / `B*` 编号，
> 定义都在这里。
>
> ⚠ 为什么需要它：`A11`/`A12`/`A13` 这几个编号一度**只在状态行里被提到、却没有定义**
> （来自一次口头盘点，结论没落到文档）—— 这种"有编号没出处"的状态比不编号更糟，
> 因为它看起来像已经记录在案了。本文件就是为了终结这种情况。

---

## 0. 编号规则

| 前缀 | 含义 | 现状 |
|---|---|---|
| `A*` | **审计（Audit）项**：功能/交付面缺口、需要补的东西 | **A1~A13 全部完成**（含 A12，见 §1.2） |
| `B*` | **缺陷（Bug）项**：实现里的真缺陷 | **B1~B4、B6、B8 已修复**（B6 于 v0.34，见 §3.2）；**B5/B7 记录缺失**（见 §3.2） |
| `F*` | **固件侧问题**（不是本 SDK 的待办） | 独立编号，见 `FIRMWARE_ISSUES.zh-CN.md` |

编号**只增不复用**；完成的条目不删除，移到 §2/§3 保留可追溯性。

---

## 1. 未完成项（P1）

> ✅ **上一轮（v0.34）曾经清空**：`A1~A13` 与 `B1~B4 / B6 / B8~B10` 全部完成（A11/A12/A13 见 §1.1/§1.2/§1.3）。
> **v0.35 又开了三条**（JointROS 的“非阻塞读路径”需求，见 §1.4〜§1.6），其中 §1.6 是整个需求的**前置未知**。
> 下面保留各条的**原始记录**（当时的证据与结论），完成的在标题上标 ✅ 并给出结果。

### 1.4 阶段 2：SDK 侧**限速状态轮询**（`jsdk_context_set_state_poll()`）+ 关节级状态 + 计数器

**背景**：阶段 1（§2.11）只给了“发一帧、不等”的**原语**，相位与限速留给调用者。
JointROS 的需求里明确要 SDK 提供“限速上限/每 tick 预算/多关节顺序保证”。

**要做**（默认**关**，不改变现有行为）：

| 项 | 做法 |
|---|---|
| 配置 | `jsdk_context_set_state_poll(ctx, {period_ms, per_cycle, fields, timeout_ms})`；`period_ms = 0`（默认）就是现在的行为 |
| 调度 | 每总线**最多一个在途**请求；按 `node_id` **升序轮转**（确定性）；`per_cycle` 上限（默认 1）；`period_ms` 是**每关节**周期（文档给出总线负载算式） |
| 失效 | 在途超时（默认 50 ms）→ 计数 + 让该关节状态**失效**，**不拿旧值冒充当前值**（与 JointROS `polled_usable()` 同一条原则） |
| 关节级 | 需要时再考虑“路径解析一次并缓存”；⚠ 先确认真的比现有 `jsdk_joint_get_feedback()`（已含 pos/vel/current/torque/age/status_flags）多给什么，**不要**凭空造第二套缓存 |
| 可观 | 计数进 `jsdk_bus_state_t`（`state_sent/state_ok/state_timeout`）+ `-v`/`health` + Python（ABI 三处同步：头、`jsdk_text.c` 表、`_abi.py`） |
| ⚠ 新鲜度阈值 | 现在 `jsdk_joint_stale_ms()` = max(50 ms, `heartbeat_rate_ms×3`, 控制周期×6)（`src/core/jsdk_fault.c:233`）—— **只盯着心跳**。一旦反馈源改成“按需轮询”，客户端会看到“帧是新的但 `FEEDBACK_STALE` 仍置位”（真机上已出现：心跳 10 Hz 而阈值退到 50 ms ⇒ 恒 stale）。阶段 2 必须把**轮询周期也计入阈值**，并且把阈值变成**可读**（建议公开 `jsdk_joint_get_stale_ms()`，让 `age_ms` 与阈值一眼可比） |

**优先级**：中（阶段 1 已可让 JointROS 自己排；但需求明确要 SDK 给限速与顺序）。

### 1.5 `jsdk_joint_sdo_read()`：文档说“非阻塞”，实现是**阻塞**的（用户裁定：真做成非阻塞）

**证据**：头文件写“发起读/写（**非阻塞**，返回 0 表示已发起）。用 `sdo_state()` 轮询结果”，
而 `src/core/jsdk_param.c` 的实现直接 `jsdk_ctx_read_param_exact()` —— **阻塞到应答或超时**，
`state` 在返回前就被置成 `SUCCESS`/`ERROR`，调用者**永远看不到 `BUSY`**。

**影响**：想找“非阻塞读路径”的人（正是本次 JointROS 需求）会踩它；文档承诺的能力不存在。
**修法**（用户已选）：真做成非阻塞 —— 需要一个在途请求槽 + 收帧时完成 + 超时失效，
与 §1.4 的在途状态机制**共用**（别做两套）。
**优先级**：中高（承诺与实现不符本身是真缺陷；且与 §1.4 天然合并）。

### 1.6 ⚠ **前置未知（下一步就做）**：闭环运行中设备到底答不答 `0x41`/`0x44`，值新不新

**为什么它排在最前面**：阶段 1 的 API 已经能用，但“**闭环 + 1 kHz 控制帧在流**时设备是否
应答查询帧”**两边的文档都没有记录**（JointROS 只在失能态读过）。这决定了整个需求能不能成：

| 实测结果 | 后续方案 |
|---|---|
| 答 + 值新鲜 | 方案成立，沿用 `0x41`（2 帧/次，10 Hz × 7 关节 ≈ 1.5% 总线） |
| 答 + 值也是 0/0（与上报帧同病） | 退回 `0x20` 端点读（Classic 上 4 帧/次）+ 更严的限速 |
| **不答** | **做不到**，直接写结论（不提供假 API） |

**实验**（已获用户批准）：用阶段 1 的原语，在关节使能（阻尼保持/零增益，按用户选定的安全姿势）
下跑 30 s、1 kHz tick、10 Hz 发 `0x41`，记录：应答率 / `fb.pos`·`vel` vs 失能态端点真值 /
`tick_overruns` / `req_timeouts`。⚠ 风险已告知（使能后重力可能缓慢溜车）。

**优先级**：**P0（挡住阶段 2 的验收口径）**。

---

### 1.1 A11 — `heartbeat_rate_ms` 快照字段恒为 0 ✅ **已完成（v0.23，见 §2.5）**

> 已修：`jsdk_joint_read_config_snapshot()` 直接用标定阶段读回的设备真值；
> 真机实测 `dump-config --json` → `heartbeat_rate_ms:100`（此前恒为 0）。
> 语义已按用户确认钉住：**`0` = 固件关闭了心跳**（不是“没读到”）。以下为原始记录。

**证据**

| 位置 | 内容 |
|---|---|
| `include/joint_sdk/joint_sdk.h:476-484` | `jsdk_joint_config_snapshot_t` 的注释：**"字段均为设备实际生效值（configure() 已读回并校验）"**，其中含 `uint32_t heartbeat_rate_ms;` |
| `src/core/jsdk_config.c:810` | `out->heartbeat_rate_ms = 0u;               /* 由 P1 的心跳配置 API 补齐 */` |

**影响（为什么不是无关紧要）**

- `jsdk-cli dump-config` 与 Python `Context.dump_config()` 都直接暴露这个字段；
- 在**本协议里 `heartbeat_rate_ms = 0` 是有含义的**：它表示"设备不发心跳"。
  于是客户做心跳排障时读到 `0`，会得出"心跳被关了"的结论 —— 而设备实际可能每 100 ms 在发。
  **一个错误的 0 比一个诚实的"未知"更有害。**
- 抓不到的原因：测试只断言了字段**存在**，没断言它**等于设备值**。

**建议修法**

1. 在 `jsdk_joint_read_config_snapshot()` 里真正读端点
   `axis0.config.can.heartbeat_rate_ms`（u32；该路径已被 `examples/04_param_rw.c` 验证可读）；
2. 读失败时**不要**静默填 0：要么保留 0 但把 `valid` 语义说清楚，要么加一个
   `heartbeat_known` 位（推荐后者，与 `jsdk_unit_scale_t.valid` 的做法一致）；
3. 补回归用例：断言快照值 == 设备规格里给的值（例如 spec `hb=10` → 断言 10），
   并在设备关闭心跳时断言能区分"0 = 真的关了"与"读不到"。

**优先级**：中（会误导现场排障，但不影响控制）。

---

### 1.2 A12 — 单位/标度 API 在公共头里没有注释，而它在 CAN 版语义与 EtherCAT 版不同

**证据**

| 位置 | 内容 |
|---|---|
| `include/joint_sdk/joint_sdk.h:1009-1015` | `jsdk_unit_scale_t` 结构体**有**字段注释 |
| `include/joint_sdk/joint_sdk.h:1017-1025` | `jsdk_unit_scale_default()` / `jsdk_unit_scale_calc()` / `jsdk_joint_set_scale()` / `jsdk_joint_get_scale()` —— **四行声明，零注释** |
| `src/core/jsdk_units.c:135-160` | 说明与公式其实都写了，但写在 **`.c` 里**（客户看不到）：`jsdk_unit_scale_default()` 在 CAN 版是**恒等映射**（`pos_counts_to_rad = 1.0`），`valid` 只表示"调用方是否真知道这台电机（`rated_trq > 0`）" |

**影响**

- 从 EtherCAT 版迁过来的人会**合理地**假设 `pos_counts_to_rad` 是"计数 → rad"的换算
  （EtherCAT 版确实如此），于是在 CAN 版拿到 `1.0` 后要么以为标定失败、要么写出错误的换算。
- `MIGRATION.zh-CN.md` §5 已经把它列成"迁移软肋"，但那边也只能说"见头文件注释" —— 而头文件里没有。

**建议修法**：把 `jsdk_units.c` 里的说明**搬到头文件**（每个函数 3~5 行 Doxygen：
CAN 版为什么是恒等映射、`valid` 的含义、`jsdk_unit_scale_calc()` 的公式与参数单位），
并在 `UNITS.zh-CN.md` 加一小节"什么时候需要 scale（答案：CAN 上基本不需要）"。

**优先级**：中（迁移者会踩；纯粹是文档工作，半天内可完成）。

---

### 1.2.1 A12 ✅ **已完成（v0.34）**

**做了什么**

| 项 | 内容 |
|---|---|
| 头文件 §16 | 用一张 **CAN vs EtherCAT 对照表**开头（线上量是否已是物理量 / `_default()` 是不是恒等映射 / 什么时候才需要 `_calc()`），再逐个函数写 Doxygen（含 `pos_counts_to_rad` 公式、`trq_to_Nm = rated/1000`、`valid` 的两种来路） |
| 顺带补齐 | 审计脚本还揪出 **18 个**其它**完全没有 Doxygen** 的公共声明（`get_bus_state`/`get_feedback`/`group_enable`/四个 `*_string()`/`desc_import` …）—— 一并补全，共 **19 处** |
| `docs/UNITS.zh-CN.md` | 新增 §5「`unit_scale_*`：什么时候**真的**需要它」：结论表（物理量 API → 不要 / raw 计数驱动 → 要）、EtherCAT 迁移对照、`valid == 0` 的**两种**来路区分、`> 1e12` 防呆上界的理由、三条自查 |
| **防复发** | 新增 `tools/check_api_docs.py`：扫公共头，任何 `JSDK_API` 声明缺少相邻 Doxygen 就**失败**（支持"一行注释带一组声明"），并注册进 ctest（`api_docs_lint`） |

**为什么加脚本而不是只改文档**：这 18 处缺注释**不是同行漏写**，而是"新加了公共 API
却没有文档"这个动作**没有被任何东西拦住**。只补内容，下一个人还会犯。
守卫本身也自检过：补之前 `--list` 报 **18 处**，补之后 **0 处**；且它**不是**靠"有没有 `/**`"
糊弄 —— 分组声明（四个 `*_string()`）算一处、`ALLOWED` 例外要写明理由。

```console
$ python tools/check_api_docs.py
check_api_docs: OK（所有 JSDK_API 声明都有文档）
$ ctest --test-dir build -R api_docs_lint   # 已进 ctest，不会再烂掉
```

**过程中的一个真错（记下来，同类操作会再遇到）**：批量插文档块时，脚本用"锚点字符串 → 新内容"
做替换，其中 4 处**锚点本身就是被替换掉的代码**（3 个 `typedef` + `jsdk_context_desc_export` 的声明）
⇒ 编译期报 `unknown type name`、链接期报 `undefined reference`。
**自检办法**：`git show HEAD:...h | grep -c JSDK_API` 与 `grep -c JSDK_API 现文件`
必须**相等**（文档插入不该改变声明条数）—— 这一步立刻定位到唯一缺失的那一条。

---

### 1.3 A13 — Python 绑定曾缺 23 个公共 C 函数 ✅ **已完成（v0.24）**

**结果（可复现，与下面“当时的状态”对照）：**

```console
$ python tools/_abi_gap.py
公共 API 总数        : 114
已绑定函数总数        : 114
未绑定（缺口）        : 0
```

**当时的状态（本文件首次写作时用脚本实测，不是估算）**

```text
公共头声明的 jsdk_* 函数：112
Python 绑定里出现过的：    89
Python 未绑定的：          23
```

**分四类**（类型不同，处理优先级也不同）：

| 类别 | 函数 | 判断 |
|---|---|---|
| **SDO 风格端点访问**（7） | `jsdk_joint_sdo_create` / `_by_name` / `_state` / `_data` / `_data_size` / `_read` / `_write` | **真实缺口**：Python 只能用 `param_get/set`，拿不到"句柄 + 状态机"那套 |
| **单位与标度**（4） | `jsdk_unit_scale_default` / `_calc`、`jsdk_joint_set_scale` / `get_scale` | **真实缺口**（与 A12 同源：连注释都没有，更别说绑定） |
| **描述符缓存与回调**（5） | `jsdk_context_set_desc_raw_sink`、`jsdk_context_desc_import_raw`、`jsdk_context_set_desc_progress`、`jsdk_context_set_fault_callback`、`jsdk_context_desc_fetch` | **真实缺口**：Python 侧用不了"路线 B 原始缓存"、也注册不了故障回调；`desc_fetch` 稍轻（`Context.configure()` 内部会取描述符），但没有"只取不配"的入口 |
| **raw 逃生通道**（3） | `jsdk_joint_set_target_position` / `_velocity` / `_torque` | **真实缺口**：Python 拿不到协议原始量入口 |
| **有意不暴露**（3） | `jsdk_context_create` / `jsdk_context_free`（堆模式；Python 自己管存储）、`jsdk_context_desc_poll`（MCU 轮询路径） | **符合设计**，不需要绑 |
| **诊断**（1） | `jsdk_hal_slcan_stats` | 小缺口（`fd_config`/`fd_frames` 已绑，只差统计） |

**怎么修的（按实际发生的顺序）**

1. 在 `PYTHON.zh-CN.md` 列出“未暴露清单”（先止血，因为“没有说明”本身就是缺陷）—— 已**撤销**：清单已变成“0 未绑定”；
2. `_abi.py` 的 `_FUNCS` 补全到 **114** 个，并把其中无构建开关依赖的符号加进 `_REQUIRED_FUNCS`
   （**正是这一条把“库比绑定旧”当场报出来**——作者自己就在真机上撞到过：Python CLI 用的是包内**旧的** DLL）；
3. 封装层：`Joint.sdo()` + `Sdo` 类、`units.UnitScale` + `unit_scale_*`、`Context.desc_fetch/desc_poll/desc_import_raw/desc_raw_sink/desc_progress/on_fault`、`Joint.set_*_raw()`、`SlcanHal.stats()`、`Joint.set_current()`；
4. `python -m jsdk_can` 从“只读”扩到 **24 个子命令**，与 `jsdk-cli` 同款安全闸/退出码/JSON 字段；
5. 守卫用例：`bindings/python/tests/test_full_surface.py`（双向核对 `_FUNCS` ↔ 头文件、SDO 跨路径对拍、两个入口各 24 条子命令冒烟）。

**过程中查出的真问题（都已修 + 有回归）**

| 问题 | 后果 | 修法 |
|---|---|---|
| `desc_import()` 后把 `_configured = True` | 紧接着的 `configure()` 变成**静默 no-op**（不握手、不标定）—— 最危险的那种错 | 不再设那个标志 + 回归用例 |
| ctypes 回调参数用 `CFUNCTYPE` 类型声明 | 传 `None`（取消注册）直接被拒；`Joint` 查找因 `int` vs `c_void_p` 失败 | 参数声明为 `c_void_p` + `ctypes.cast`；新增 `_joint_by_ptr()` |
| `watchdog` 在**两个** CLI 里都被归到“只读描述符”档 | 该命令**必然失败**（`ep_break_timeout` 是标定时才解析的）—— C 侧发现后同步修 Python 侧 | 两版都改为“需完整配置”档 |
| ABI 守卫最初只 grep 源码文本 | **测不出已删除的签名**（作者做了变异：把参数删掉仍然通过） | 改为与 `_abi._FUNCS` **对拍**（变异后必失败） |
| Python 测试硬编码 `build/jsdk-cli.exe` | WSL 里 Linux 能 binfmt 直跑 `.exe` → 用例“真跑起来了”但写不了 `/tmp/...`，看着像 SDK 的 bug | 按平台**只选原生二进制**（`_find_c_cli()`） |

**优先级**：中（不影响现有用户，影响交付面的“完整感”与迁移评估）。

---

## 2. 已完成（保留可追溯）

### 2.1 审计项 A1~A10

| # | 内容 | 完成于 |
|---|---|---|
| A1 | 7 个 C 示例（+1 个 C++ 示例） | v0.19（`examples/`，全部注册 ctest） |
| A2 | pkg-config（`Libs.private: -lm -ldl`，**无 pthread**） | v0.19（`cmake/joint-sdk-can.pc.in`） |
| A3 | 单文件 amalgamation | v0.19（`tools/amalgamate.py` → `dist/jsdk_can_amalgam.{h,c}`） |
| A4 | C++ header 包装 | v0.19（`include/joint_sdk/joint_group.hpp`） |
| A5 | Arduino / PlatformIO | v0.19（`arduino/`；⚠ 真实 AVR 工具链未验证） |
| A6 | `install(EXPORT)` → `find_package(jsdk_can)` | v0.19（目标 `jsdk::can`） |
| A7 | `UNITS.zh-CN.md` + `MIGRATION.zh-CN.md` | v0.19 |
| A8 | 真链路冒烟（能自动化的部分） | v0.17（`tools/live_can_smoke.sh`，`vcan` 44 项断言） |
| A9 | Linux 后端首次编译/执行并修缺陷 | v0.17 |
| A10 | 固件问题清单汇总 | v0.18（`FIRMWARE_ISSUES.zh-CN.md`） |

### 2.2 缺陷项 B1~B4、B8~B10

| # | 内容 | 完成于 |
|---|---|---|
| B1 | 8 字节参数读（`ReqLen` 写死 4） | v0.16 |
| B2 | Classic 链路分段读 | v0.16 |
| B3 | wheel 标签 `py3-none-any`（内含 dll） | v0.19（`bindings/python/setup.py`） |
| B4 | CLI `estop`/`reset`/`set-node-id` 执行路径无测试 | v0.19（并因此挖出 B8） |
| B8 | `set-node-id` 会静默造出重号（"验证新地址可应答"被别的设备满足） | v0.19（新增 `jsdk_ctx_probe_node()`） |
| B9 | **Python 包内的 Linux/macOS 共享库名带版本号，而加载器只认无版本号的名字** → "包自带库却找不到"，外表看起来像"只支持 Windows" | v0.21（CMake 拷成无版本号名字 + 加载器对包内 `lib/` glob 兜底） |
| B10 | **Windows 控制台上中文全是乱码**（`jsdk-cli` 输出 UTF-8，而控制台是 CP936）—— 代码里**没有任何代码页处理**，而且是"通病"：任何把本 SDK 的中文打到控制台上的程序都一样 | v0.22（`tools/jsdk_cli/cli_text.{h,c}`：控制台转码、管道原样 UTF-8；Python 入口重定向时钉 UTF-8） |

#### B9 详情（用户报的"只看到一个 DLL"）

**现象**：`bindings/python/src/jsdk_can/lib/` 里只有一个 `libjsdk_can.dll`，看起来 Python 绑定只支持 Windows。

**真实情况**：绑定本身是**跨平台**的（加载器有 `.dll` / `.so` / `.dylib` 三套候选，`package-data` 也配了三种），
`lib/` 里只有 DLL 只是因为"最后一次带 `-DJSDK_BUILD_PYTHON=ON` 的构建是在 Windows 上做的"。
**但确实有一个真缺陷**（否则在 Linux 上本应开箱可用）：

| 位置 | 内容 |
|---|---|
| `CMakeLists.txt` 的拷贝步骤（原） | `cmake -E copy_if_different "$<TARGET_FILE:jsdk_can_shared>" "…/lib/"` |
| 实际结果 | Linux 上 `$<TARGET_FILE:>` 解析为**带完整版本号**的实体文件 → 拷进去的是 **`libjsdk_can.so.0.1.0`** |
| `_abi.py` 的候选表（原） | `libjsdk_can.so` / `libjsdk_can.so.0` / `jsdk_can.so` —— **没有带完整版本号的那一个** |
| 后果 | Linux 上 `pip install .` 装出来的包**找不到自己带的库**（除非用户自己设 `JSDK_LIB_PATH`）。Windows 因为 `libjsdk_can.dll` 本来就不带版本号，一直没暴露 |

**两道防线（都做了）**

1. **CMake 拷成本平台加载器认识的名字**：Linux `libjsdk_can.so`、macOS `libjsdk_can.dylib`、
   Windows 保持工具链原本的名字；顺便**清掉别的平台的库**（`lib/` 里全是构建产物，
   混一份用不上的 DLL 进去只会让 wheel 变脏）。
2. **加载器对包内 `lib/` 做 glob 兜底**（`libjsdk_can.*` / `jsdk_can.*`，只列存在的），
   于是"以后换个命名/手动扔一份带版本号的库进去"也不会又炸一次。

**验证**

- **Linux（Python 3.8.10）上跑完整 pytest：142 passed / 1 skipped** —— 与 Windows 数字**完全一致**；
  这同时证明了两件事：绑定在 Linux 上真的能用，且 **Python 3.8 是被实测过的下限**
  （因此 `requires-python` 从 `>=3.9` 下移到 `>=3.8`）。
- **回归用例** `tests/test_review_regressions.py::test_bundled_library_is_discoverable`：
  包内只要有库，候选路径里就必须有一条**真实存在**（无库的纯源码树自动 skip）。
- **变异测试**（三道）：① 库名带版本号 + 去掉 glob → **用例变红**（即原缺陷重现）；
  ② 恢复 glob（名字仍带版本号）→ **变绿**（证明兜底真有效）；③ 全部还原 → 142 通过。
#### B10 详情（用户报的"`jsdk-cli` 输出乱码"）

**现象**：本机挂了 CANable（slcan）后跑 `./build/jsdk-cli --if slcan scan`，屏幕上是
`鍙戠幇 0 涓�鑺傜偣锛堣��鍔� 200 ms + 涓诲姩鎺㈡祴 1..16锛�` 这类乱码。

**根因（字节级实测，不是猜的）**：

| 事实 | 证据 |
|---|---|
| CLI 的输出是**合法 UTF-8 且带 CRLF** | 捕获字节 383 B，`decode('utf-8')` 成功 |
| 这台机器的控制台代码页是 **936** | `cmd /c chcp` → `活动代码页: 936` |
| 同一个字节串按 CP936 解读 = **恰好是用户看到的乱码** | `decode('gbk')` → `鍙戠幇 0 涓�...` |
| 代码里**完全没有**代码页处理 | `grep -rn "SetConsoleOutputCP\|GetConsoleOutputCP\|setlocale\|_setmode"` → 0 命中 |

所以不是"某条消息写错了"，而是**输出端的字节不符合接收端的约定**。
范围（即"通病"）：`jsdk-cli`、示例、**客户自己的程序**（SDK 只返回 UTF-8 中文，
打印由客户做）、以及测试程序在 CP936 控制台上打印的中文。

**修法**（与 git for Windows 的 `mingw_ansi_fputs()` 同思路）：

1. 新增 `tools/jsdk_cli/cli_text.{h,c}`，把 CLI 的 `printf/fprintf/fputs/fputc/puts`
   机械替换为**同名同签名**的 `cli_*` 版本（参数顺序完全一致，`-Wformat` 检查
   靠 `__attribute__((format(printf,...)))` 保留）；
2. 判则：**是控制台** → 转成 `GetConsoleOutputCP()`；**是管道/文件** → 原样 UTF-8；
3. 按**完整 UTF-8 码点**转（`cli_utf8_complete_prefix()` + 跨调用残尾状态），
   绝不把 3 字节汉字从中间切开；纯 ASCII 块走快路径不转；转码不可行时降级为原样写，
   **绝不丢内容**；
4. 环境变量 `JSDK_CLI_TEXT`（`utf8` / 代码页数字）做逃生门，同时让 CI 能在没有
   真控制台的机器上验证转码路径；
5. 不用 `SetConsoleOutputCP(65001)`：那改的是**共享的控制台状态**，被 kill 就回不去，
   还要求控制台字体有 CJK 字形。

**同类修正（一并做掉）**：

- 输出字符串里**不得出现 CP936 表示不了的字符**：`cli_cmd.c` 的一个 `⚠`（U+26A0）
  改为 `注意：`，`examples/04_param_rw.c` 的 `✓/✗` 改为 `[OK]/[FAIL]`，
  `examples/05_desc_cache.c` 与 `bindings/python/examples/03_params.py` 的 `⚠` 同样处理；
- **Python 入口**：`python -m jsdk_can` 重定向时 Python 用 locale 编码（本机 cp936），
  与 C 版 CLI（UTF-8）**不一致**，且碰到 CP936 表示不了的字符会直接 `UnicodeEncodeError`
  崩掉 → 在 `__main__.py` 里"重定向时钉 UTF-8"（控制台不动，Python 本来就正确）。
  注：原测试助手一直带着 `PYTHONIOENCODING=utf-8`，把这个坑盖住了 —— 新用例**故意不带**。

**验证**：

- 新增 `tests/test_cli_text.c`（ctest `cli_text`）：**60 项断言 / 0 失败**，覆盖纯逻辑、
  真转换器（CP936 `发现` → `b7 a2 cf d6`）、跨调用被切开的汉字、逐字节写、
  >512 B 分块不丢字节、非控制台原样 UTF-8、转码失败降级不丢内容；
- **端到端**：① `JSDK_CLI_TEXT=936` 跑整个 CLI → 前 4 字节 `b7 a2 cf d6`，按 CP936 解码是
  "发现 0 个节点"（即"CP936 控制台会正确显示"）；② **真控制台 A/B**（集成终端就是真控制台，
  探针实测 `_isatty≠0`、`GetConsoleMode` 成功、`GetConsoleOutputCP()=936`）：
  默认 → `发现 1 个节点（被动 200 ms + 主动探测 1..16）`**正确**；
  加 `JSDK_CLI_TEXT=utf8` → `鍙戠幇 1 涓�鑺傜偣锛�`**精确复现用户报的乱码**。
  ⚠ "检测分支"目前**没有自动化**用例（需要真 ConPTY）；CI 里用 `JSDK_CLI_TEXT=<cp>`
  覆盖转码路径，检测分支靠上面这次人工 A/B 与后续复现；
- **变异测试**（三道，都先断言"变异串唯一命中"）：① 不做转码 → 单测 **20 项失败** +
  端到端变乱码；② 不按码点边界切 → **1 项失败**；③ 转码失败就丢内容 → **1 项失败**；
  还原后**字节级一致**且全绿；
- **防复发（静态检查）**：`tools/check_cli_text.py`（ctest `cli_text_lint`）
  挡两类回退：CLI 里再出现裸 `printf` 家族（绕过 `cli_text`），以及输出字符串里
  出现 CP936 表示不了的字符（含 `\u26A0` 这种通用字符名写法）。
  它本身也验过：五道假问题（裸 printf / 真实 ⚠ / `\u26A0` / 注释里的 printf / 字符串里的
  `http://`）该报的报、不该报的不报；
- Python 用例 `test_redirected_output_is_utf8`（**故意不带** `PYTHONIOENCODING`）：
  去掉修复 → **失败**，加回 → **通过**。

**剩下的范围（未做，已写进文档）**：示例程序与**测试程序**仍然直接 `printf` 中文
（前者由 `PORTING.zh-CN.md` §7.5.4 给出可抄的 20 行骨架；后者的中文只在开发者本地
控制台上不好看，ctest 抓的是文件/管道，不影响结论）。
### 2.3 本次（写 BACKLOG 时）顺手修掉的文档缺口

| 项 | 说明 |
|---|---|
| 2.3.1 | `A11/A12/A13` 此前**只有编号没有定义** → 本文件 §1 补齐（头号问题） |
| 2.3.2 | `DESIGN` 状态行与 `MIGRATION` §5 的引用改为指向本文件 |
| 2.3.3 | `PYTHON.zh-CN.md` 补"未暴露的 C API"清单（原先读起来像绑定是完整的） |
| 2.3.4 | 本文件 §3.2 明确记录"B5/B7 记录缺失"，而不是假装它们不存在 |

### 2.4 真机联调（slcan + CANable）—— 发现与修复

环境：CANable（slcan，115200）+ 一台关节（node 1，`hw_version 262711`、
`fw_version 1544`、FD）。**全程只用只读命令**（`scan`/`info`/`read`/`batch-read`/
`health`/`desc-*`）；写命令（`write`/`set-node-id`/看门狗）**未在真机上执行**。

| # | 现象 | 根因 | 处理 |
|---|---|---|---|
| C1 | `desc-info` / `read` / `info` 等**首次尝试**常失败，重跑就好 | ① 发 `0x24` 前**不排水**，上次中断传输的残留帧吃掉超时预算；② 请求发出去后**不重试**，3 s 一到就放弃 | 发请求前 `desc_drain_rx()` 排到静默；250 ms 内一帧未收到则**重试一次**；把"自旋计数判死"换成**冻结时钟判死**（旧判据在真机上误报 `stalled (no frames; frozen clock?)` —— 真机要 100 ms 以上才回帧，而自旋只转了几十 ms） |
| C2 | 描述符下载报 `total_len must be 1..65535` | 元数据帧校验只接受 `{` 开头的 JSON，而**本机固件的描述符根是数组**（首字符 `[`）→ offset=0 的**数据帧**被误判成元数据帧 | 两种首字符都接受；未开始传输时**跳过**非元数据帧（上限 4096，超限报 `CB_DESC_ERR_META_SKIPPED`），并补针对性用例 |
| C3 | `read` / `batch-read` 报"未标定"而拒绝执行 | CLI 把它们归入"完整模式"（下载描述符**并标定**），而 `jsdk_joint_param_get/set()` 本身**没有标定门槛** —— 只读参数不该被挡 | 命令表拆成三档 `CLI_DESC_NONE` / `ONLY` / `FULL`；`read`/`batch-read`/`dump-config`/`write`/`watchdog` 等只要描述符 |
| C4 | `info` / `health` / `dump-config` 全部失败，报"周期 ≥ 设备看门狗" | CLI 给**所有**命令都设了 100 ms 周期，而设备 `break_timeout = 100 ms` → 撞上"周期必须小于看门狗"的校验（该校验只对**跑循环**的命令有意义） | 只有 `mon`/`calibrate`/`home`/`mit` 设周期（`needs_loop=1`），其余命令周期为 0 → 跳过该门 |
| C5 | `read axis0.motor.config.gear_ratio` → `8.90553e-41`；`batch-read` → `node_id = 16777216`、`heartbeat_rate_ms = 1677721600`；`health` 报"标定值超出范围" | **参数值在线上是小端，而 SDK 按大端解码**（照协议文档 P7/§4 实现）。症状却看着像"描述符与固件版本对不上"或"标定值异常"，把人往错方向带 | **SDK 侧适配**（不改固件）：值编解码统一走 `cb_le_get_*()` / `cb_le_put_*()`，帧字段继续 `cb_be_*()`。文档修订见 `PROTOCOL_NOTES` §3.1、固件侧登记为 `FIRMWARE_ISSUES` 的 **F27** |
| C6 | 单次请求命令在**打开端口后的第一个**请求仍可能丢（约 1/4） | slcan 的 CDC 端口刚打开时命令会丢（`C`/`Y5`/`O` 未生效）→ 通道从未真正打开 | ✅ **已修（v0.22 续）**：① `hal_slcan.c` 的 `C`/`Y<n>`/`O` 改成 **`sl_cmd_sync()`：发完等适配器 ACK（`\r`），没等到就重发**（最多 3 次× 250 ms）—— 盲等固定时间无法区分“已生效”与“整段丢掉”；② 描述符请求的重发从“250 ms 一次”改为 **0.25 / 0.6 / 1.2 s 递增、最多 3 次**（设备对 `0x24` 幂等）。**修后真机实测：`desc-info` 20/20、`health --node 1` 12/12（修前约 1/10 失败）**；离线回归用例 `test_ops.c` 的 `[7]` 在 HAL 层注入“丢掉主站前 N 帧”，验证 N=1/3 能自愈、全丢时**恰好 4 次尝试**；变异测试（把重发改回 1 次）→ **6 项用例失败** |

**C5 的真机复测**（同一台设备、同一组只读命令，改前 / 改后）：

| 命令 | 改前 | 改后 |
|---|---|---|
| `read axis0.config.can.node_id` | `16777216` | **`1`** ✓ |
| `read axis0.config.can.heartbeat_rate_ms` | `1677721600` | **`100`** ✓ |
| `read axis0.motor.config.torque_constant` | 垃圾值 | **`0.0904141`** N·m/A ✓ |
| `read axis0.motor.config.gear_ratio` | `8.90553e-41` | **`7.75`** ✓ |
| `health` | 失败（`calibration values out of range`） | **通过**：`fault no`、`err_code 0`、`pos_rad`/`vel_rad_s`/`current_A`/`torque_Nm` 均可读（`current_A = -0.088`、`torque_Nm = -0.000`） |
| `batch-read`（上述三个参数） | `16777216` / `1677721600` / `8.9e-41` | **`1` / `100` / `7.75`** ✓ |

> ⚠ **注意**：`gear_ratio` 的真机值是 **7.75**，而项目夹具里一直用 16.5、
> 文档里也把 16.5 当参照 —— 那个数字是**夹具的**，不是真机的。凡是"物理量换算"类
> 结论，最终必须以真机读回的标定值为准。

**C5 的验证链**（三层，缺一不可）：

1. **线上原始字节**被测试钉死：`tests/test_hal_virtual.c` 的 `[7] param access` 断言
   `node_id` 回 `01 00 00 00`、批量值流 `64 00` / `00 20 00 00`、分段写分块字节；
   Python `test_wide_params.py` 的 `SERIAL_BYTES` 改成小端构造；
2. **反向证据**：同一台设备的 `0x46` 响应 `fw_version` 只有按**大端**解才是 1544
   （小端解是 134610944）→ 证明是"参数通路与帧字段两套字节序"，不是"协议整体反了"；
3. **真机复测**：上表。

> 附注（~~不是缺陷，但会误导~~ **已在 §2.5 修掉**）：`dump-config` 原先属于"只需描述符"的
> 命令，因此打印的快照 `valid=0`（本次调用没跑标定），多数字段是 0。已改为"完整配置"档。

---

### 2.5 A11 完成 + `dump-config` 档位修正（v0.23）

**用户对语义的裁定（这条很关键）**：`heartbeat_rate_ms == 0` 表示**固件关掉了心跳**。
因此不需要"valid 位"那套：直接把**设备真值**报出来即可，0 就是 0。

| # | 改动 | 证据 |
|---|---|---|
| 1 | `jsdk_joint_read_config_snapshot()` 不再写死 0，改用标定阶段读回的 `j->heartbeat_rate_ms` | `src/core/jsdk_config.c`（原 `:809` 的 `= 0u; /* 由 P1 补齐 */`） |
| 2 | 头文件写明字段语义：**0 = 固件已关闭心跳**（不是"没读到"）；并说明快照需要**已标定**的关节，否则除 `node_id` 外为 0 且 `valid == 0` | `include/joint_sdk/joint_sdk.h` 的 `jsdk_joint_config_snapshot_t` |
| 3 | `dump-config` 从 `CLI_DESC_ONLY` 改为 `CLI_DESC_FULL` —— 它打印的就是**标定后的快照**，放在 ONLY 档只能打出一堆 0，看着像"设备没配好" | `tools/jsdk_cli/cli_cmd.c` 命令表 |

**真机复测**（同一台设备）：

| 命令 | 改前 | 改后 |
|---|---|---|
| `dump-config --json` | `{"valid":false, …, "heartbeat_rate_ms":0}` | `{"valid":true,"gear_ratio":7.75,"mit_max_pos":12.5,…,"torque_constant":0.0904141,"node_id":1,"heartbeat_rate_ms":100,"break_timeout_ms":0}` |

（仿真后端也一样：夹具 `hb=10` → 快照报 `10`，不再是 0。）

> ⚠ 那张表里的 `break_timeout_ms` **后来改成了 0**：当时显示 100 是 `jsdk_watchdog_device_ms()`
> 把 0 归一化的产物（错得根子在于以为“0 = 100 ms”）。真机 `read` 一直说的是 **0 = 禁用**，
> 见 §2.8。

---

### 2.8 `break_timeout == 0` = 禁用（用户裁定后的语义修正，v0.25）

**用户裁定**：“最新固件里 `can.config.break_timeout == 0` 代表 watchdog 失效（禁用），
需要修改相关代码逻辑、注释与文档。”

**回源复核**（`ODrive/Firmware/communication/can/`）：

```cpp
// can_cyberbeast.cpp: auto_stop_if_timeout()
uint32_t timeout_ms = odrv.can_.config_.break_timeout;
if (timeout_ms == 0) return;             // 超时检测被禁用
```
```cpp
// odrive_can.hpp: struct Config_t { … uint16_t break_timeout = 0; … }
```

⇒ `0` = 禁用，而且**是默认值**（设备出厂即无协议超时保护）。此前 SDK 与两份文档都写着
“0 → 按 100 ms 处理”，那是**旧固件的行为**（也是 F28 一开始被误判的根源之一：当时
“写 250 却读回 0”被解读成“端点不落地”，真因见 §2.5 末尾 —— 是我们的写帧太短）。

**改了什么**（每一处都能被下面列出的用例发现）：

| 位置 | 旧 | 新 |
|---|---|---|
| `jsdk_watchdog_device_ms()` | `0 → JSDK_WD_DEFAULT_MS(100)` | 原样返回 `0`（新增 `JSDK_WD_DISABLED_MS = 0` 具名常量） |
| `keepalive_joint()` | 任何情况下都按 `wd` 补喂 | `wd == 0` → **一帧不补、不置 RISK** |
| `configure()` 周期闸 | `period_ms >= wd`（wd=0 时**恒真** → 每个循环命令都被拒） | `wd != 0` 时才检查 |
| `configure()` 的 hint | 无条件写 `2×period` | **需要时**才写（当前为 0 或比周期短）+ **读回确认** |
| `set_watchdog_ms(0)` | 警告“0 ≠ 关闭” | **真的关闭**（写 0 读回 0 = 校验通过） |
| 读回不一致 | 只写进 `last_error` | 新增公共标志 `JSDK_JF_WATCHDOG_UNVERIFIED` |
| `dump-config` | 显示 100（与 `read` 的 0 矛盾） | 显示 **0**，与 `read` 一致 |
| 仿真设备 | `0 → SIM_BREAK_TIMEOUT_DEFAULT_MS(100)` | `0` = 禁用；默认值也改成 0（对齐固件） |
| 两个 CLI | `break_timeout_ms` = 回显写入值 | `device_reports_ms` = **独立再读一次设备** + `verified` |

**回归与变异**（本项目要求“新增产物必须有能自动发现它坏掉的检查”）：

- `tests/test_ops.c [9]`：读回 0（置位但保留写入值）/ 读回 999（`PROTOCOL`）/ **写 0 = 关闭且校验通过**；
- `tests/test_ops.c [9b]`：`timeout=0` 时 **100 ms 周期照样能 configure**、跑 40 个周期设备不被停、
  切 CURRENT 后**0 帧 keepalive**；
- `tests/test_hal_virtual.c`：`timeout=0` 时“武装后停发 5 s 也不停机”；
- `tests/test_joint.c`：快照 `break_timeout_ms == 0`（= 禁用，不再被归一成 100）；
- **变异 4/4 全部被检出**：keepalive 照补 / `0` 当门限比 / 模型 `0 → 100` / `device_ms` 归一。

**真机验证**（fw 1545，COM3；**v0.27 复测，此前那组“读回恒 0”的结论已推翻**）：

```console
$ jsdk-cli --if slcan --channel COM3 --node 1 --json read can.config.break_timeout
{"path":"can.config.break_timeout","type":"uint16","value":0,"value_text":"0"}
$ jsdk-cli … --json dump-config        → "break_timeout_ms":0      （与 read 一致）
$ jsdk-cli … --json --yes watchdog 300 → {"ms":300,"device_reports_ms":300,"verified":true,…}   ✓
$ jsdk-cli … --json --yes watchdog 0   → {"ms":0,"device_reports_ms":0,"verified":true,"disabled":true}
$ python -m jsdk_can … --json --yes watchdog 300 → 与 C 版**逐字段一致**
```

> 📌 **F28 已结案：是我们自己的 bug，不是固件缺陷**。
> 本项当初因为“写 250 之后同进程立刻读仍是 0”而判为“端点不落地”，但真因是
> **SDK 发出的参数写帧不足 8 字节**（`bool`/`u8` = 5 B、`u16` = 6 B），
> 被固件 `cmd_param_write()` 首句 `if (msg.len < 8) return;` **整帧静默丢弃**
> （不回 ACK、不改值）。这就是现场那两句症状的**同一个根因**：
> “写 `pre_calibrated 1` 报成功但读回 `false`”、“`calibrate --yes` 进不去状态 3”。
> 修复：`cb_param_pack_write_req()` 一律补齐到 ≥ 8 B（`dst[3]` 仍写真实值宽），
> 仿真设备**复刻**同一门限（否则离线测试永远抓不到），两版 CLI 的 `write` 一律**写后读回**
> 并在不一致时 `rc=1`。


---

### 2.5b 参数写帧**最短 8 字节**（v0.27，现场故障的根因）

现场两句话：“`write axis0.motor.config.pre_calibrated 1` 报成功但读回 `false`”、
“`calibrate --yes` 报 `device never entered state 3`，电机不动”。**同一个根因**：

```
固件 cmd_param_write() 第一句：  if (msg.len < 8) return;      // 整帧静默丢弃，不回 ACK
SDK 按 4 + 值宽 打包：           bool/u8 = 5 B、u16 = 6 B、u32/f32 = 8 B
                                → 短帧全被吞，宽帧恰好正常（于是“有些参数能写、有些写不动”）
```

`axis0.requested_state` 是 **u8** ⇒ `calibrate` / `home` 在真机上**从未成功过**；`break_timeout` 是
**u16** ⇒ 见 §2.5（已被误判成固件 F28）。

| 层 | 修复 | 能发现它坏掉的检查 |
|---|---|---|
| 打包 | `cb_param_pack_write_req()` 补齐到 ≥ 8 B（`CB_PARAM_WRITE_REQ_MIN`；`dst[3]` 仍写**真实**值宽） | `tests/test_ops.c [11] test_write_short_values`（u16/bool/u8 必须到达设备）+ 变异 |
| 仿真设备 | **复刻同一门限**（`f->len < 8` → 丢弃） | 不复制门限时离线测试**永远绿** —— 这正是它藏这么久的原因 |
| CLI | `write` 一律**写后读回**：`requested/value/verified`，不一致 → `rc=1` | `tests/test_cli.c [11]`（含 `dropwrite` 故障注入 → 必须 `rc=1`）；Python 侧 6 条对拍 |
| Python 库 | 新增 `param_set_auto()`：按端点声明类型装箱 | `test_joint.py` 的 u16 往返 + 值域拒绝 |
| 工具 | `hw_verify.sh` L8 断言 `verified`（旧断言 `written` 已废） | 真机 `--write-probe` 一轮不通过即红 |

**真机结果**（COM3 / fw 1545）：`hw_verify.sh --runs 3 --write-probe` **48/48 全绿**；
`pre_calibrated`、`break_timeout` 0→250→0、`enable_watchdog` 全部写→读回 `verified:true`。

> ⚠ 教训：**怀疑固件之前先数一下自己发出的帧长**。同一个 bug 在仿真上永远是绿的，
> 因为仿真器当时没有复制固件的早期返回 —— “模型必须复刻固件的门限”这条已写进
> `sim_device.c` 的注释与本表。

### 2.5c “关节一直进故障”与 `fault-reset`（v0.27，含一次误判的订正）

现场现象：两次 CLI 调用之间，`axis0.error` 就是 `0x800`，`health` 报 `err_code 8 / err_name CAN_TIMEOUT`；
写 `axis0.error 0` / 发 `CLEAR_ERRORS` 都能让它**瞬时变 0**，但 ~120 ms 后又回到 `0x800`。

**我一开始认错了根因**（以为是 `estop` 锁存，实为）：对照固件枚举，`0x800` 是
**`ERROR_WATCHDOG_TIMER_EXPIRED`**（esstop 是 `0x4000`，本次**从未出现过**）。真因是设备配置：

```
axis0.config.enable_watchdog = true
axis0.config.watchdog_timeout = 0.0        // ← get_watchdog_reset() = 0
get_watchdog_reset() = clamp(watchdog_timeout,0,…) * current_meas_hz = 0
watchdog_check():  value > 0 ? value-- : (error_ |= WATCHDOG_TIMER_EXPIRED)
                     ↑ 0 就是“零容忍”：只要这一周期没收到帧（只有 do_command() 才 watchdog_feed）就置位
```

⇒ 任何**CAN 静默空隙**（进程退出/命令之间）都会锁存 0x800，而 `detect_error_code()` 把它映射成
`ERR_CAN_TIMEOUT`，把人往波特率/线缆上带。修复/应对：

| 项 | 内容 |
|---|---|
| 设备侧（用户可改） | `axis0.config.watchdog_timeout = 0.1`（或 `enable_watchdog = false`）；⚠ `watchdog_timeout = 0` **不是禁用** |
| 新增命令 | `fault-reset`（`STOP_MOTOR` → `CLEAR_ERRORS` → 等错误位归零，不动电机）；能清锁存位，**但原因还在就会立刻回来 —— 这是正确行为** |
| 固件问题 | 记为 **F29**（含两个不同根因共用一个 `ERR_CAN_TIMEOUT` 报错名） |

### 2.5d `calibrate` 的等待判据错了（v0.27，真机实测暴露）

`request_state_and_wait()` 原判据是“`current_state` **等于**请求值”（例如 3）才算“已进入”，
只见“离开”就返回成功。真机实测两个都错：

| 事实（实测） | 后果 |
|---|---|
| 跑 `FULL_CALIBRATION_SEQUENCE` 时 `current_state` 报的是**子状态**（4 MOTOR_CALIBRATION → 6 ENCODER_OFFSET_CALIBRATION → 7 INDEX_SEARCH），**从不等于 3** | 一次**真的跑起来了**（电机都转了）的标定被判成“device never entered state 3”并超时 ← 害我把排查方向全押在“写没写进去” |
| 回到空闲**也不等于成功**（标定失败 = 进子状态 → 出错 → 回空闲） | 标定失败也报 OK |

已改为：① 判据是“**离开静息态**”（静息 = IDLE(1) / CLOSED_LOOP_CONTROL(8)，后者是**回零**成功后的定格）；
② 结束时**抓一次 QUERY_ERROR 六类明细**，有故障位就报 `PROTOCOL` 并把位名写进 `last_error`；
③ 超时路径也带上故障明细（“状态完全没动”最常见的原因就是**锁存故障让状态机拒绝启动**）。

真机现在的输出（同一台设备、同一个配置）：

```console
$ jsdk-cli --if slcan --channel COM3 --node 1 --json --yes calibrate
{"action":"calibrate","warning":"motor moves during full calibration"}
jsdk-cli: calibrate 失败：protocol-error
         joint 0(node 1): state 3 ran but the device reports a fault:
         detail_err=0x00000102(ENCODER_FAILED) axis_error=0x00000000
         mit_err=ENCODER hb=axis|encoder can_state=IDLE
```

`0x102` = 轴 `ENCODER_FAILED`(0x100) + 编码器 `CPR_POLEPAIRS_MISMATCH`(0x2)：编码器**偏置标定的扫描**
实测位移与期望值（cpr 16384 / pole_pairs 14 / `calib_range` 2% → 期望 16384 counts）偏差超限。
设备侧要查：转子能否自由转动（抱闸/机械卡滞）、`calibration_current`(5 A) 是否够、编码器方向。

---

### 2.6 “首帧丢失”链条的完整修复（v0.23）

§2.4 的 C6 修掉之后，真机还残留 **1/20** 的 `desc-info` 失败。**是新加的诊断信息把它抓出来的**：
报错从 `(0/0 bytes)` 变成 `(0/0 bytes, 680 frames received)` —— "收到了 680 帧，却一个字节都没解析"，
说明**通道是通的、我们的请求丢了**，而重发**从来没触发**：

| 环节 | 原来 | 现在 |
|---|---|---|
| 重发判据 | `ctx->fetch.frames_rx == 0u`（"一帧都没收到"） | `!cb_desc_fetch_started(&ctx->fetch)`（"**本次传输还没开始**"） |
| 为什么原来错 | RX 里常有上一次被中断传输的**残留帧**（真机实测 680 帧）：帧数 ≠ 0，于是永不重发，最后超时 | 新增 `cb_desc_fetch_started()`（`cb_jsondesc_fetch.[ch]`）：只有"收到并接受元数据帧"才算本次传输开始 |
| 握手 | 单发 + 一次等 3 s（丢了就 `no response to handshake`） | `handshake_verified()`：发 → 等 50 ms → **重发（最多 6 次）**；失败信息写明尝试次数与"适配器可能丢首帧" |

**回归与变异（都做过）**：

- `tests/test_ops.c` 的 `[7]`：HAL 层注入"丢掉主站前 N 帧" → N=1/3 自愈、全丢时**恰好 4 次尝试**；
  新增"**注入残留帧 + 丢掉第一个请求**"用例（残留帧只在**请求发出之后、设备开口之前**注入 ——
  早了会被 `desc_drain_rx()` 排掉、晚了会插进本次流里，都不算复现真机）；
  变异（判据改回 `frames_rx == 0`）→ **4 项失败** ✓
- `tests/test_ops.c` 的 `[8]`：按 MsgType 丢 `QUERY_STATUS` → 丢 3 次自愈、全丢时**恰好 6 次尝试**；
  变异（握手改回单发）→ **5 项失败** ✓
- ⚠ 做变异时踩到自己一个坑：`cp` 恢复备份后 `make` 可能认为**无需重建**（跑的还是突变版二进制）
  → 必须 `touch` 再 build，并用测试结果确认。第一次的 `[7]` 变异结论就是这样被推翻的。

**真机复测**：`desc-info` **30/30**（此前 1/20 失败）；分层自检 3 轮 **51/51**（含写路径）。

#### 2.6b 第三轮（v0.31）：**会话预热** —— 从“事后补救”到“压根不发生”

前两轮都是在**具体命令**上加重发（描述符下载、握手）。但用户点出的真问题范围更大：
**任何**只发一帧就等回包的命令（`err` / `info` / `read` / `batch-read` …）都会踩到同一个坑，
而它们**没有**描述符下载那种自带重发的兜底 —— 真机实测 `err`（隔一会儿的第一条）
报 `timeout / not configured`，紧接着连跑 4 次全成功。

v0.31 的做法是把它变成一个**库级保证**：

| 环节 | 做法 |
|---|---|
| 探针 | 幂等的 `QUERY_DEVICE_INFO(0x46)`（只读设备信息，重发无副作用） |
| 节奏 | 每轮等 `JSDK_WARMUP_ATTEMPT_MS`(50 ms)，默认预算 `JSDK_WARMUP_TIMEOUT_MS`(500 ms)，**外加轮次上限**（防 `now_ms` 冻住时死循环） |
| 挂点 | `jsdk_ctx_send()` —— **发帧之前**（库 / Python / 两个 CLI 全覆盖）；控制路径与急停走 `jsdk_ctx_send_raw()` 绕开（急停不能被 500 ms 预热拖住） |
| 语义 | 成功记 `tx_retries`；同会话再调是**空操作**；失败只报 `TIMEOUT`（**不致命**，只收不发的命令照用）；无关节 → `BAD_STATE` |
| 不参与 | ⚠ 预热的收发**不参与帧格式学习**：设备对 0x46 的回包会按**请求**格式回（镜像），学到“一致”就永远发现不了对端是 FD ⇒ 两个 CLI 的顺序固定为「**先只听一耳朵（定格式）→ 再预热**」 |

⚠ 副产物（两个必须一起改的地方）：**仿真器原本让查询回包跟随请求格式**（描述符流却按节点格式），
那正是把这个镜像问题藏起来的地方 → 已改为按节点自己的 `is_fd` 回包；
另新增故障注入 **`drophead=N`**（丢掉主站最初 N 帧），否则 CI 里无法端到端复现现场。

**真机验收（2026-09-24）**：36 个独立进程（`err`/`info`/`scan` 各 12 次）**失败 0**，
其中第一条 `err` 如实报出 `重发=1（预热）` —— **真丢了一帧，并被预热吸收**，
而用户侧看到的是一条正常结果（修前这个场景就是失败）。

#### 2.6c 第四轮（v0.32）：**幂等请求重发**（用户裁定：只做这一档）

预热只挡住“会话开头丢帧”。**运行中途**丢帧时，`err`/`info`/`read` 这类“单发即等”的命令
以前仍然是一条超时（用户现场复现过：`err` 隔一会儿第一条失败、紧接着连跑 4 次全成功）。
本轮只做**能证明安全**的那档：

| 类别 | 处理 |
|---|---|
| **幂等**（读、等 ACK 的同值写） | 超时后重发同一帧，最多 1 次额外尝试 |
| **可证未生效才重发**（`requested_state` 3/11） | **本轮不做**（留待下一步） |
| **绝不重发**（控制帧/急停、`SET_NODE_ID`、`requested_state`） | 由 `timeout_ms = 0` 天然排除 + 用例钉住 |
| **轮询读** | 用 `jsdk_ctx_read_param_once()` 只发一次（节奏优先，重发会把 200 ms 节奏拖成 6 s） |

⚠ 实现中踩到并修掉的一个真错：助手最初只有一个 `msgtype` 参数（请求与应答同号），
而 `QUERY_STATUS(0x40)` 的应答是 **MIT 响应（0x00）** ⇒ `set-node-id` 的“目标号已占用”探测
直接失效 —— 是既有的 `test_cli`/`test_ops` 用例当场变红把它顶出来的。现在 `rsp_msgtype` 显式传。

观测：`tx_retries_req` + `-v` 的 `重发=N（预热 X + 幂等请求 Y）`。
**真机**：36 个独立进程 0 失败之外，本轮再跑 `hw_verify.sh --runs 3 --write-probe` 与 `py_hw_smoke.py` 回归。

---

---

#### 2.6d 第五轮（v0.33）：**链路质量观测** —— 以及它当场照出的两个"假数字"

前四轮修的是行为，但没有一个**客户可见的数字**能回答"这条链路现在稳不稳"。
本轮按**调用意图**给补偿分类计数（不是按"发生了什么"）：

| 计数器 | 含义 |
|---|---|
| `tx_retries` | 所有重发（= `_warm` + `_req`） |
| `tx_retries_warm` | 会话预热的（会话开头，正常现象） |
| `tx_retries_req` | 幂等请求的（**运行中途丢帧**，值得关注） |
| `req_timeouts` | "等超时"的次数（含被重发救回的） |
| `last_retry_what` / `last_retry_age_ms` | 最近一次重发的类别与年龄（`-v` / health 里直接看） |
| `desc_retries` | 描述符 `0x24` 请求的重发次数（`desc-info`） |

**真机抓到的两个"假数字"**（都是**统计口径**错，不是功能错）——这轮的真正价值：

| 现象 | 根因 | 修法 |
|---|---|---|
| `scan` 每次都报 `重发=15（幂等请求 15）`，且耗时翻倍 | 扫描**本来就**要探 15 个空地址，探不到是**预期结果**，却被当成"丢帧重发" | 主动探测（`probe`）**不重发**：`jsdk_ctx_probe_node()` 传 `flags = 0` |
| 修完上一条，`scan` 改报 `超时=15` | 同一个错误换了个计数器：`flags` 只有一个"要不要重发"的位，**重发**与**计数**是两件事 | 拆成两个位：`JSDK_REQ_RETRY`（重发，判据="幂等吗"）与 `JSDK_REQ_COUNT`（计数，判据="这次空等值得上报吗"）；**探测与轮询读都传 `flags = 0`** |

**真机复验**（同一台设备）：`scan` → `重发=0（预热 0 + 幂等请求 0）；超时=0；tx=15 rx=75`；
12/12 行 `超时=0`。**端点**：`info` 在真机上如实报出
`重发=1（预热 0 + 幂等请求 1）；超时=1` —— 即**运行中途真丢了一帧、并被重发救回**（v0.32 的证据）。

**规律（三个实例，值得单独记）**：每一个"补偿"或"统计"都必须按**调用意图**分类，
否则它会把**预期行为**报成故障：`scan 重发=15`（探测当成了请求）、`scan 超时=15`（同一错换计数器）、
预热镜像帧（把"收到回包"当成"格式已确认"）。**信号的含义取决于它是在哪种调用里产生的**。

---

### 2.7 写路径真机验证（v0.23，已脚本化）

用户批准后新增 `tools/hw_verify.sh` 的 **L8/L8b**：

| 层 | 做法 | 断言 |
|---|---|---|
| **L8**（默认） | `read` 原值 → `write` 原值 → `read` | 读回必须等于原值（幂等；**不会改设备状态**） |
| **L8b**（`--write-probe`） | 写 `原值+Δ` → 校验 → **无论成败都写回原值** → 校验 | 既证明"写真的生效了"，又证明能恢复；恢复失败会**大声报警** |

默认目标 `axis0.config.can.heartbeat_rate_ms`（u32、非安全关键、不落 Flash；`--write-path` 可换）。

- **真机实测**：`100 → 150 → 已恢复 100` ✓（3 轮 51 项全过）
- 顺带价值：这条同时验证了**写方向的字节序** —— 若按大端写，设备会把 150 存成 `0x96000000`，
  读回即不符 → 用例会红。此前"写路径未验证"的缺口到此补上（**针对 u32 参数**；
  `set-node-id`/`reset`/`save` 这类**不可幂等重放**的命令仍然只做人工确认）。
- ⚠ 虚拟后端上 L8/L8b **显式跳过**：每个 CLI 进程都是新仿真总线，写入不跨进程保留，
  在仿真上跑这两层只会给出"通过"的假象。

---

### 2.9 MSVC 支持 + 两版 CLI 的四处契约偏差（用户提 README 改进时挖出，v0.26）

**起因**：用户要求 README 把“各环境的编译选项/准备工作”写清。按本项目规矩，
**写进文档的命令必须真跑过** —— 于是先试了一遍 Visual Studio 生成器，结果一连串问题。

| # | 问题 | 症状 | 修法 |
|---|---|---|---|
| 1 | **MSVC 的 C 默认是 C89 + 扩展** | `cb_ctrl.c` 里 `if (classic) { int16_t vel_raw = …; }`（语句后声明）→ `error C2059: 语法错误:"if"`，整个库编不过 | CMake 的 MSVC 分支加 `/std:c11` |
| 2 | 源码是 UTF-8（注释含中文），中文 Windows 代码页 936 | 逐文件 `warning C4819` | 加 `/utf-8` |
| 3 | `JSDK_WERROR=ON` 在 MSVC 下**被静默忽略**（只加 `/W4`，没有 `/WX`） | “0 告警”在 VS 下不受约束 | 补 `/WX`，并把三类噪声处理掉：`C4996`（`fopen`/`strcpy`）→ `_CRT_SECURE_NO_WARNINGS`；`M_PI` 未定义 → `_USE_MATH_DEFINES`；`C4127`（`do{}while(0)` 宏误报）→ `/wd4127` 并注明理由 |
| 4 | `JSDK_ALIGNOF` 用 `offsetof(struct { char c; T t; }, t)` 求对齐 | MSVC `C4116`（宏里的匿名结构体） | 就地 `#pragma warning(push/disable:4116)` + 注释（C11 允许的写法，只在本文件用） |
| 5 | `(uint8_t)~CB_PARAM_FLAG_MORE` | MSVC `C4310`（截断常量）| 先掩到 8 位再转，语义不变 |
| 6 | 测试里写常量 `0.0/0.0`、`1.0/0.0` 造 NaN/Inf | MSVC **编译期**直接 `error C2124` | 改用 C99 `NAN` / `INFINITY`（带运行期兜底宏） |
| 7 | ⚠⚠ **静态库与 DLL 导入库同名**（都叫 `jsdk_can.lib`） | `-DJSDK_BUILD_SHARED=ON` 时两者互相覆盖 → 链接到导入库 → 内部符号全部 `LNK2019`（**只有 MSVC 会这样**：MinGW 的 `libjsdk_can.a` 与 `libjsdk_can.dll.a` 名字不同） | Windows 下给共享库的导入库改名 `jsdk_can_dll.lib`（Python 按路径加载 DLL，不依赖它） |

**结果**：Windows + MSVC 19.44 / VS 2022 生成器 **ctest 21/21**（共享库配置 22/22），
`/W4 /WX` **0 告警**；同一份 CMake 在 gcc 上的行为完全不变（MinGW 21/21 + 18→22/22）。

**顺带查出的 CLI 契约偏差（都是“文档说两版一致，其实不一致”）**：

| # | 问题 | 以前 | 现在 |
|---|---|---|---|
| 8 | `mon --csv` 的**语义**不同 | C：`--csv FILE`（同时落盘）；Python：`--csv` 是开关 | 统一 `--csv` = 开关（写 stdout）、`--csv-file FILE` = 落盘；且取值以 `-` 开头**当场报用法错**（以前 `mon --csv --duration 1` 会静默写出一个叫 `--duration` 的 CSV —— 在仓库根目录躺了几天才被发现） |
| 9 | `mon --csv` 的**列数**不同 | C 17 列 / Python 12 列（文档却写“方便两边对着看”） | 统一 17 列 + 数值格式，两版输出**逐字节一致**（有对拍用例） |
| 10 | `write` 的值错分类不同 | C：非数字与超范围**都是 1**；Python：2 / 1 | 统一为 非数字 = **2**、超范围 = **1**（有对拍用例） |
| 11 | ⚠ **C 版写不了任何负数** | `write vel_limit -5.0` → “未知选项 -5.0”（`-` 开头一律当选项）| 负数按**位置参数**处理（判定规则同 argparse：`-` 后紧跟数字/`.`），并加回归用例 |

**验证**：真机上把 README 里要写的命令逐条跑过（14 条只读 + 9 条安全闸 + 用法闸 + `estop`）；
新增用例 `test_cli.c [9]/[10]` 与 `test_full_surface.py` 的两条对拍；变异/对拍逻辑见
`§2.8` 的同款做法。

---


### 2.10 v0.34：交付面收尾（A12 ✅ + B6 ✅ + 一键回归）

A12 与 B6 是 BACKLOG 里最后两个**未完成项**（B5/B7 属"记录缺失"，见 §3.2）。

| 项 | 结果 |
|---|---|
| **A12** | 头文件 §16 重写 + **19 处**缺文档声明补全 + `docs/UNITS.zh-CN.md` §5 + `tools/check_api_docs.py`（已进 ctest）—— 详见 §1.2.1 |
| **B6** | 矛盾构建组合改为 configure 期 `FATAL_ERROR`（"宁可拒绝也不静默"）—— 详见 §3.2 |
| **一键回归** | 新增 `tools/check_all.sh`：一条命令跑完 Windows（MinGW 堆+CLI、共享库、MSVC）+ WSL（Linux、ASan、Linux 共享库）+ Python（两平台）+ 三个冒烟 + 三个静态守卫，末尾汇总表，任一失败即非 0 退出 |
| **状态同步** | README「当前状态」、本文件 §1/§3 与 DESIGN 版本行按 v0.33/v0.34 实际数字同步 |
| **CI（只读验证型）** | `.github/workflows/verify.yml`：① `full-matrix` 在 **Windows 自托管 runner** 上跑 `./tools/check_all.sh` 全 14 步；② `linux-native` 在 `ubuntu-latest` 上跑 **同一个** `tools/_check_all_body.sh`（Linux 四步）+ `check_all.sh --only` 的三个守卫。接法、runner 前置条件、以及**哪些永不进 CI**（真机：`hw_verify.sh`/`py_hw_smoke.py`）见 `docs/CI.zh-CN.md` |

**结果（可复现，`./tools/check_all.sh` 的汇总表）**

```text
win-build        通过   22/22      win-bsh   通过   23/23     msvc   通过   23/23
py-win           通过   242 passed / 2 skipped                smoke-* ×3     通过
lint-cli-text    通过   OK         lint-api-docs 通过 OK      abi-gap 通过   缺口 0
linux            通过   22/22      asan      通过   22/22     bsh-linux 通过 构建 OK
py-linux         通过   242 passed / 2 skipped                ---------------------------------
                                                  14 步：14 通过 / 0 失败（退出码 0）
```

**为什么要脚本**：v0.31~v0.33 每轮都要手工敲十几条命令（两个平台、三套工具链、
Python 两平台、冒烟与静态守卫），漏一条就会得出"全绿"的错误结论 ——
本项目已经吃过 **"MSVC 构建失败却仍报 22/22"（陈旧结果）** 的亏。
脚本因此默认**先 touch 源码**（内容改了但 mtime 没变时 `make` 会静默跳过，跑的是旧二进制），
`--no-touch` 可关。

**脚本第一次跑就抓到两处真问题**（都不是它自己写错了）：

| # | 现象 | 根因 | 处理 |
|---|---|---|---|
| D1 | `smoke-amalgam` 报 `dist/ 已过期`；`smoke-arduino` 报 `arduino/src/jsdk_can_amalgam.h` 与 `dist/` 不一致 | 本轮改了**公开头**（A12 的 Doxygen）却没有重新生成合并件与 Arduino 副本 ⇒ **客户拿到的单文件版会落后一版** | 按提示 `python tools/amalgamate.py && cp dist/jsdk_can_amalgam.* arduino/src/`；两个冒烟转绿 |
| D2 | `amalgamate.py` 报 `jsdk_can_amalgam.h（70964 字节）`，实际文件 **99976** 字节 | `len(text)` 是**字符数**不是字节数；文件里有中文，写出去是 UTF-8 ⇒ 报小了 40%，而客户会拿这个数字做 Flash/内存预算 | 改成 `len(text.encode('utf-8'))`；重跑后报告值与 `stat` **完全一致** |

⚠ 顺带记一个自己踩的坑（值得留给下一个写守卫的人）：**守卫必须用“负向测试”验它真的会红**。
`check_api_docs.py` 的第一版把"紧邻的声明可以共用一份文档"做成了启发式 —— 于是把一条
**故意没文档**的新声明贴在已文档声明旁边时，它仍然报 OK（假阴性），而那正是 A12 要防的事。
改成 **显式 `GROUPS` 白名单**（共用文档的 11 组逐条登记并写明理由）+ 多声明组的"蹭文档"检查后，
同一条破坏会被准确点名。

---

### 2.11 v0.35：非阻塞状态请求（阶段 1）—— 响应 JointROS 的“读不能与 tick 互斥”

**需求**（JointROS 测试工程师，原文要点）：设备主动上报的帧在真机上**冻结**
（`pos/vel=0/0`、`valid=false`、`FEEDBACK_STALE`、`age_ms=0xFFFFFFFF），而
`axis0.encoder.pos_estimate`/`vel_estimate` 能读到真值；但它们的**所有**读路径都要进
安全暂停窗口（`TickGroup::pause()` = **先安全失能**）⇒ 驱动中拿不到新鲜反馈。
需求：① 不与 tick 互斥的读；② 非阻塞取结果；③ 明确限速/每 tick 预算与多关节顺序；
④ 最好有关节级状态（免得硬编码端点）。他们提供的输入：单次读 **1.4〜5.5 ms**（真机
Classic/slcan）、一次独占窗口 ≈ **9 个 1 kHz tick**，建议 **≤10 Hz**。

**阶段 1 做了什么（本版交付）**

| 项 | 内容 |
|---|---|
| 新 API | `jsdk_joint_request_state(j, fields)`，`fields = JSDK_STATE_POS_VEL(0x41) \| JSDK_STATE_CURRENT(0x44)` |
| 语义 | **发出即返回，不等应答**；应答由 `cycle_begin()` 里**已有的**收帧路径解码回填 `jsdk_joint_feedback_t`（0x41 → `pos`/`vel`；0x44 → `current_A`/`torque_Nm`）⇒ **没有新增缓存、也没有新增解复用**，可信度判据仍是客户已经在用的 `age_ms` |
| 不重发/不记账 | 它是空闲查询：丢了就丢（用 `age_ms` 判），所以**不碰** `req_timeouts`、不加重发预算 |
| 硬约束 | 0x41/0x44 **不喂设备看门狗** ⇒ 只能额外发、**不能顶替控制帧**；RT 路径走 `jsdk_ctx_send_raw()`（不带预热，否则首次最多白等 500 ms） |
| 为什么不是端点读 | `0x41` 一次轮询 **2 帧**（Classic 上 pos+vel 端点读要 4 帧，不能批量），且应答是 **f32**（心跳只有 ×100 定点） |

**验证**：`test_ops [18]`、Python 新增 4 项（242→246）、矩阵 14/14（含 MSVC `/W4 /WX` 与 ASan）。

**过程中的两个教训（自审留下来）**

| # | 现象 | 根因 |
|---|---|---|
| S1 | 测试里“注入速度/电流再读回”会读到 0 | 仿真的**应答是在下一个 sim_tick 里才生成**的，而 `vel_estimate`/`iq_measured` 是**每 tick 重算**的派生量（未 armed 时被清零）⇒ 注入活不过一拍。**位置是积分量**，所以只有它能直接注入。改法：位置的**数值**用注入钉死（精确断言），速度/电流的数值改在“已使能 + 纯力矩”场景里验**同一条反馈内部的自洽关系** `torque_Nm == current_A × Kt × gear` |
| S2 | “应答丢了就不假装新鲜”这条断言差点变成**概率性**的 | 心跳（默认 10 ms）可能刚好在那一拍到达并置 `valid` ⇒ 全部子用例改用 `hb=0`（同一手法：`tests/test_joint.py` 的 `test_feedback_age_grows_when_bus_goes_quiet`） |

---

## 3. 已知限制与记录缺失

### 3.1 已知限制（不是缺陷；**无法在本环境消除**，交付时要一并说明）

| # | 限制 | 出处 |
|---|---|---|
| L1 | **`CANFD_FDF` 那一处修复无法自动化验证**：`vcan` 不做收发器层校验，缺 FDF 的 FD 帧照样能收发 | `PORTING.zh-CN.md` §7.5.3 人工清单第 12 项、`tools/live_can_mutation_test.sh` 的 M3 说明 |
| L2 | 真硬件项：位定时/终端电阻、error-frame/bus-off、真适配器对 `Y`/`O` 的反应、真驱动器运动 | `PORTING.zh-CN.md` §7.5.3 B 段（12 项） |
| L3 | **Arduino 未在真实工具链编译过**（本环境无 `arduino-cli`）：只做了 `src/` 新鲜度 + sketch 逻辑编译 | `arduino/README.md`、`tools/arduino_smoke.sh` 第 4 步 |
| L4 | PCAN 后端只有 stub 级测试（真通道名无法在 Windows 上校验） | `tests/test_hal.c` 的平台无关断言 |
| L5 | 描述符 > 65535 B 会撞 `chunkOffset` 的 u16 上限（当前 41029 B，占 63%） | 设计 §6.6、`FIRMWARE_ISSUES` 的 F10 条目 |
| L6 | `test_unreadable_type_is_rejected` 恒跳过（夹具里只有标量端点） | `tests/` 内的注释 |
| L7 | ~~**slcan：打开串口后的第一个请求可能丢**~~（见 §2.4 的 C6）→ **已修三轮**：① 打开序列等 ACK + 重发；② 描述符请求重发的**判据从“一帧都没收到”改成“本次传输还没开始”**（`cb_desc_fetch_started()`）—— 后者才是残留帧场景的正解，见 §2.6；③ **v0.31 根治：会话预热**（`jsdk_context_warmup()`，幂等 `QUERY_DEVICE_INFO` + 重发，默认 500 ms 预算/每轮 50 ms），**挂在发帧之前**（`jsdk_ctx_send()`）⇒ 丢的帧发生在**预热**阶段，用户可见的第一条命令不再失败（急停等不能等帧走 `jsdk_ctx_send_raw()` 绕开）。次数见 `tx_retries` / `-v` 的 `重发=N（预热）`。真机实测：`desc-info` **30/30**、`health` 12/12、分层自检 10 轮 160 项全过；**36 个独立进程 0 失败**（其中一次 `err` 如实报出 `重发=1（预热）` = 真丢了一帧且被吸收）。⚠ **仍有前提**：预热只能保证“**请求最终到达**”，它自己也会丢帧（靠重发兜住）；若适配器固件同样不 ACK 命令、且丢帧是**持续**而非“只在打开后最初几帧”，则重发次数用尽仍会失败 | `src/core/jsdk_ops.c`、`src/core/jsdk_context.c`、`src/hal/hal_slcan.c`、`src/core/jsdk_desc.c`、`src/proto_cyberbeast/cb_jsondesc_fetch.c` |

### 3.2 记录缺失（诚实交代）

| # | 情况 |
|---|---|
| **B5 / B7** | 这两个编号出现在早期盘点里，但**结论没有落到任何文档**，本次已无法复核它们原本指什么。**处理**：保留编号不复用；若这两个问题仍存在，请重新提一条（新编号）并附证据 —— 不要凭记忆去"恢复"它们 |
| **B6** | ✅ **已修复（v0.34）**。`-DJSDK_BUILD_HAL_VIRTUAL=OFF` 且 `JSDK_BUILD_TESTS=ON` 时，`test_desc_fetch` 因为调用 `sim_set_desc()`（定义在 `sim_device.c`，属于虚拟后端）**链接失败**。**处理**：取"显式拒绝"而不是"加 `if()` 守卫"—— 理由：测试套件里 **11 套 C 测试全都**用 `sim_*` 注入故障（丢帧/不回包/镜像帧），虚拟后端是唯一**不需要硬件**的 HAL；用 `if()` 守卫会得到"少跑一半、却报 100% 通过"的 ctest，那正是本项目已经踩过的假绿灯。现在在 `CMakeLists.txt` 的测试段开头 `message(FATAL_ERROR)` 并给出两条出路（`-DJSDK_BUILD_TESTS=OFF` 或 `-DJSDK_BUILD_HAL_VIRTUAL=ON`）。**验证**：矛盾组合 → configure 期报错（不再是链接末端的 `undefined reference`）；`TESTS=OFF + HAL_VIRTUAL=OFF + CLI=OFF` → 配置+构建 **0 错 0 警** |

---

## 4. 如何使用本文件

- **评审/验收**：看 §1（未完成）与 §3（限制）—— 这两节就是"还差什么"的完整答案。
- **回答"XX 修了没有"**：查 §2 的对照表（含完成于哪个设计版本）。
- **新增待办**：追加到 §1 并**附证据（`文件:行号` 或可复现命令）**；
  纯口头结论不要进这个文件 —— 那正是 A11/A12/A13 一度不可追溯的原因。
