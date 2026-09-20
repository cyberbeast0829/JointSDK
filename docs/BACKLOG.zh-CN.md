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
| `A*` | **审计（Audit）项**：功能/交付面缺口、需要补的东西 | A1~A10 已完成；A11~A13 未完成 |
| `B*` | **缺陷（Bug）项**：实现里的真缺陷 | B1~B4、B8 已修复；B6 未修；**B5/B7 记录缺失**（见 §3.2） |
| `F*` | **固件侧问题**（不是本 SDK 的待办） | 独立编号，见 `FIRMWARE_ISSUES.zh-CN.md` |

编号**只增不复用**；完成的条目不删除，移到 §2/§3 保留可追溯性。

---

## 1. 未完成项（P1）

### 1.1 A11 — `heartbeat_rate_ms` 快照字段恒为 0，但头文件声称是"设备实际生效值"

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

### 1.3 A13 — Python 绑定缺 23 个公共 C 函数（其中约 20 个是真实缺口）

**证据（本文件写作时用脚本实测，不是估算）**

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

**影响**：功能不缺，但 Python 客户做不到"C 能做的全部事"，
而 `docs/PYTHON.zh-CN.md` **一个字都没提这件事** —— 读者会默认绑定是完整的。
**这个"没有说明"本身是缺陷**（已在本次修复：见 §2.4）。

**建议修法（按性价比排序）**

1. 先在 `PYTHON.zh-CN.md` 列出"未暴露清单"（**已完成**）；
2. 补 `set_fault_callback` + `set_desc_raw_sink` / `desc_import_raw`（Python 客户做缓存与故障上报最常用的两个）；
3. 补 SDO 家族与 `set_scale/get_scale`（用 ctypes 包一层，工作量小但要补测试）；
4. raw 三件套：显式标记为"C 专用逃生通道"，**故意不绑**并把理由写进文档（比静默缺失好）。

**优先级**：中（不影响现有用户，影响交付面的"完整感"与迁移评估）。

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

### 2.2 缺陷项 B1~B4、B8

| # | 内容 | 完成于 |
|---|---|---|
| B1 | 8 字节参数读（`ReqLen` 写死 4） | v0.16 |
| B2 | Classic 链路分段读 | v0.16 |
| B3 | wheel 标签 `py3-none-any`（内含 dll） | v0.19（`bindings/python/setup.py`） |
| B4 | CLI `estop`/`reset`/`set-node-id` 执行路径无测试 | v0.19（并因此挖出 B8） |
| B8 | `set-node-id` 会静默造出重号（"验证新地址可应答"被别的设备满足） | v0.19（新增 `jsdk_ctx_probe_node()`） |
| B9 | **Python 包内的 Linux/macOS 共享库名带版本号，而加载器只认无版本号的名字** → "包自带库却找不到"，外表看起来像"只支持 Windows" | v0.21（CMake 拷成无版本号名字 + 加载器对包内 `lib/` glob 兜底） |

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

### 2.3 本次（写 BACKLOG 时）顺手修掉的文档缺口

| 项 | 说明 |
|---|---|
| 2.3.1 | `A11/A12/A13` 此前**只有编号没有定义** → 本文件 §1 补齐（头号问题） |
| 2.3.2 | `DESIGN` 状态行与 `MIGRATION` §5 的引用改为指向本文件 |
| 2.3.3 | `PYTHON.zh-CN.md` 补"未暴露的 C API"清单（原先读起来像绑定是完整的） |
| 2.3.4 | 本文件 §3.2 明确记录"B5/B7 记录缺失"，而不是假装它们不存在 |

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

### 3.2 记录缺失（诚实交代）

| # | 情况 |
|---|---|
| **B5 / B7** | 这两个编号出现在早期盘点里，但**结论没有落到任何文档**，本次已无法复核它们原本指什么。**处理**：保留编号不复用；若这两个问题仍存在，请重新提一条（新编号）并附证据 —— 不要凭记忆去"恢复"它们 |
| **B6** | 唯一还活着的 B 项：`-DJSDK_BUILD_HAL_VIRTUAL=OFF` 且 `JSDK_BUILD_TESTS=ON` 时，`test_desc_fetch` 因为调用 `sim_set_desc()`（定义在 `sim_device.c`，属于虚拟后端）**链接失败**。**性质**：开发期构建组合问题，不影响交付产物（客户不会这么配）。**建议**：给 `test_desc_fetch` 加 `if(JSDK_BUILD_HAL_VIRTUAL)` 守卫，或把这个组合设为显式 `FATAL_ERROR`（更符合"宁可拒绝也不静默"的项目风格） |

---

## 4. 如何使用本文件

- **评审/验收**：看 §1（未完成）与 §3（限制）—— 这两节就是"还差什么"的完整答案。
- **回答"XX 修了没有"**：查 §2 的对照表（含完成于哪个设计版本）。
- **新增待办**：追加到 §1 并**附证据（`文件:行号` 或可复现命令）**；
  纯口头结论不要进这个文件 —— 那正是 A11/A12/A13 一度不可追溯的原因。
