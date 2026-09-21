# 从 EtherCAT 版迁移到 CAN 版

> 参考基线：`SOEM/joint-sdk`、`EtherCAT_Master/joint-sdk`（本文的对照表是**读两边源码**
> 得出的，不是按记忆写的）。本仓的 CAN 版见 `../include/joint_sdk/`。

---

## 0. 先读这三条

1. **三个库同名同义、互斥链接**（设计决策 ADR-1）：`SOEM` 版、`EtherCAT_Master` 版、
   本 CAN 版都导出 `jsdk_*` 符号。**同一个进程里不能同时链接两个** —— 会得到
   "重复定义"或者更糟的"静默链到另一个实现"（同名不同语义）。
2. **概念一致、API 允许不同**（ADR-2）：绝大多数函数名可以直接沿用，
   真正要改的是**寻址方式、反馈单位、应用数据来源**这三件事。
3. **错误码数值完全对齐**：`JSDK_OK / JSDK_ERR_INVALID_ARG / …` 取值相同，
   所以 `switch (st)` 与日志分析习惯不用改。
4. **重合度比想象的高**：`jsdk_joint_sdo_*()`（SDO 风格端点访问）、
   `jsdk_unit_scale_*()`、`jsdk_joint_get_fault_info()`、`jsdk_joint_set_scale()`
   在 CAN 版**同名存在** —— 真正要改的只有 §3 里那六件事。（这三条都是读两边
   源码逐个符号对出来的，不是按印象写的。）

---

## 1. 三个库的分工

| 库 | 传输 | 寻址 | 应用数据来源 | 上下文内存 |
|---|---|---|---|---|
| `SOEM/joint-sdk` | EtherCAT（SOEM 主站栈） | `alias` + `position`（从站拓扑序号，1 起） | **ESI XML** profile（`jsdk_profile_load_from_esi`） | **堆**：`jsdk_context_create()` |
| `EtherCAT_Master/joint-sdk` | EtherCAT（IgH/EtherCAT Master） | 同上 | 同上（IgH 兼容路径也支持） | 同上 |
| **本仓（CAN）** | CAN / CAN-FD（SocketCAN / PCAN / slcan / 自实现 HAL） | **`node_id`**（1..254） | **JSON 端点描述符**（设备自己服务，41 KB / 594 端点） | **零 malloc**（默认）；堆模式是可选文件 |

---

## 2. API 对照表

### 2.1 可以直接沿用（同名同义）

| 函数 | 备注 |
|---|---|
| `jsdk_context_config_default()` | 两边都先调用它拿默认配置 |
| `jsdk_context_create()` / `jsdk_context_destroy()` | **注意**：CAN 版默认**不编译**堆模式，需 `-DJSDK_ENABLE_HEAP=ON`；推荐改用下面的零 malloc 方式 |
| `jsdk_context_add_joint()` | 参数结构体不同（见 §3.1） |
| `jsdk_context_activate()` / `deactivate()` | 语义一致（都含"安全首帧 → 等 → 停机"） |
| `jsdk_context_configure()` | EtherCAT 版读 ESI/PDO 映射；CAN 版下 JSON 描述符并读量程 |
| `jsdk_context_cycle_begin()` / `cycle_end()` | 调用者拥有循环，两边一致 |
| `jsdk_context_get_bus_state()` / `last_error()` | 一致 |
| `jsdk_context_set_fault_callback()` | 回调约束一致：**rt 上下文里不许 sleep/malloc/阻塞** |
| `jsdk_joint_request_enable/disable/fault_reset()` | 一致（非阻塞请求，要跑周期才生效） |
| `jsdk_joint_is_enabled()` / `is_fault()` / `get_feedback()` | **反馈结构体差异很大**（见 §3.2） |
| `jsdk_joint_set_mode()` | 模式枚举见 §3.3 |
| `jsdk_joint_set_target_position_rad()` / `..._velocity_rad_s()` | 一致 |
| `jsdk_joint_set_target_position()` / `..._velocity()` / `..._torque()` | **raw 语义不同**（见 §3.4） |
| `jsdk_joint_set_scale()` / `jsdk_joint_get_scale()` | 一致（CAN 版同样有；见 §5 的 A12） |
| `jsdk_unit_scale_default()` / `jsdk_unit_scale_calc()` | **同名存在**（换算表来源不同：ESI vs JSON 描述符，但调用方式一样） |
| `jsdk_joint_sdo_create()` / `sdo_state()` / `sdo_data()` / `sdo_data_size()` / `sdo_read()` / `sdo_write()` | **同名存在** —— 差别只在 `create` 的入参：EtherCAT 版给 ESI 对象索引，CAN 版给**端点 ID**（另提供 `jsdk_joint_sdo_create_by_name(j, "路径")` 按路径创建） |
| `jsdk_joint_get_fault_info()` | **同名存在**（CAN 版会读 `0x45` 的 32-bit 子系统位图 + 心跳 5-bit 位图） |
| `jsdk_status_string()` / `jsdk_axis_state_string()` | 一致 |

### 2.2 只有一边有

| EtherCAT 版 | CAN 版对应物 | 说明 |
|---|---|---|
| `jsdk_profile_load_from_esi()` / `jsdk_profile_get_name()` / `jsdk_profile_destroy()` | **无**（不需要） | CAN 版从设备的 JSON 描述符 + 路径字符串拿一切；`jsdk_endpoint_lookup()` 替代 profile 查询 |
| `jsdk_joint_actual_position_rad()` / `..._velocity_rad_s()` | `jsdk_joint_get_feedback()`（已是物理单位） | CAN 版反馈**天生就是 rad**，不需要再换算 |
| — | `jsdk_hal_*_open()`（socketcan/pcan/slcan/virtual） | CAN 版多一层"内置 HAL 工厂"（可选，MCU 客户自己实现 vtable） |
| — | `jsdk_desc_fetch()` / `jsdk_endpoint_lookup/enumerate()` / `jsdk_context_desc_export/import[_raw]()` | CAN 版独有：描述符获取与缓存 |
| — | `jsdk_joint_param_*()` / `jsdk_joint_param_get_batch()` | CAN 版独有：按**路径**读写端点（SDO 的简化版） |
| — | `jsdk_group_set_mit()` / `group_enable/disable()` | CAN 版独有：一条 FD 帧驱动最多 7 个关节（EtherCAT 靠 PDO 天然同步） |

---

## 3. 六处**必须**改的语义

### 3.1 寻址：从站序号 → `node_id`

```c
/* EtherCAT 版 */
jsdk_joint_config_t jc = { .alias = 0, .position = 3, .profile_name = NULL };

/* CAN 版 */
jsdk_joint_config_t jc = { 0 };
jc.node_id      = 1;                  /* 设备自己配置的节点号（1..254） */
jc.initial_mode = JSDK_MODE_MIT;
```

⚠ CAN 上没有"拓扑序号"——设备是靠 `node_id` 认领帧的（`Dest` 字段）。
建议给每台设备写死一个 `node_id` 并记录到机柜标签上，别依赖发现顺序。

### 3.2 反馈：CiA402 原始量 → 物理量

```c
/* EtherCAT 版：statusword + counts */
fb.statusword; fb.actual_position;    /* counts */ fb.actual_torque; /* int16 */

/* CAN 版：已经是物理量 */
fb.pos;        /* 输出端 rad */
fb.vel;        /* 输出端 rad/s */
fb.current_A;  /* 电机端 A */
fb.torque_Nm;  /* 估算：current × torque_constant × gear */
fb.age_ms; fb.tx_rejected; fb.tx_frames; fb.status_flags;   /* CAN 版新增的运维字段 */
```

**不要**把 `fb.pos` 当 counts 用（单位不同，症状是差 2.6 倍或毫无意义的数）。
单位细节见 `UNITS.zh-CN.md`。

### 3.3 模式枚举：`CSP/CSV/CST` 的落点

CAN 版的 `jsdk_mode_t` 保留 `CSP=8 / CSV=9 / CST=10` 这三个**名字**，
但线上对应的帧是 `POS_CONTROL(0x01)` / `VEL_CONTROL(0x02)` / `TORQUE_CONTROL(0x03)`；
另有 `MIT=4` 与 `CURRENT=11`（CAN 独有）。所以：

- 迁过来的 `set_mode(JSDK_MODE_CSP)` 仍可用，但**单位与限制量的语义要按 §3.4 检查**；
- 原来依赖 CiA402 的 `mode_display` 判断状态的代码，改用
  `fb.axis_state` / `fb.mode_state`（后者是固件原始 nibble）。

### 3.4 raw 入口的语义

| | EtherCAT 版 | CAN 版 |
|---|---|---|
| `set_target_position(raw)` | counts | `raw / 1000` = 输出端**度** |
| `set_target_velocity(raw)` | counts/s | `raw / 1000` = 输出端 **RPM** |
| `set_target_torque(raw)` | ‰ 额定力矩 | `raw / 1000` = **输出端 N·m** |

**这是最危险的迁移点**：同一个 `set_target_torque(5000)` 在两个库里含义完全不同。
如果原来用 raw 入口调参，请改成物理量入口（`..._torque_Nm`）并重新整定。

### 3.5 应用数据：ESI → JSON 描述符

EtherCAT 版靠 ESI XML 得到对象字典；CAN 版靠设备服务的 JSON 描述符。差异是**时机**：

```c
/* CAN 版：先拿描述符，再 configure */
jsdk_context_desc_fetch(ctx);        /* 41 KB / FD 662 帧 / Classic 6839 帧 */
jsdk_context_configure(ctx);
```

MCU 上不要每次上电都下载 41 KB —— 用 `desc_import_raw()` + 常量 JSON，
或 `desc_export/import()` 缓存（见 `PORTING.zh-CN.md` §3）。

### 3.6 内存模型：堆 → 零 malloc（可选）

```c
/* EtherCAT 版（堆） */
jsdk_context_t *ctx = jsdk_context_create(&cfg);

/* CAN 版（推荐：调用者给存储） */
static jsdk_context_storage_t store;                 /* 6144 B */
jsdk_context_t *ctx = (jsdk_context_t *)&store;
jsdk_context_init(ctx, &cfg);                        /* cfg 必须与 arena 同寿命 */
/* 销毁：不需要 free()，但必须先 deactivate() */
```

想保留堆用法：`-DJSDK_ENABLE_HEAP=ON` 后 `jsdk_context_create/destroy` 仍然可用
（名字都没变）。

---

## 4. 迁移检查表

| # | 项 | ✔ |
|---|---|---|
| 1 | 全工程搜索 `jsdk_`，确认**只链接一个**实现（构建系统里删掉旧的） | |
| 2 | `jsdk_joint_config_t` 的 `alias/position/profile_name` → `node_id` | |
| 3 | 所有 `fb.actual_*` / `statusword` 的用法 → `fb.pos/vel/current_A/...` | |
| 4 | 用了 raw 入口的地方 → 改物理量入口并重新整定（§3.4） | |
| 5 | 依赖 ESI/profile 的代码 → 改成路径字符串（`jsdk_endpoint_lookup`） | |
| 6 | `jsdk_context_create` → `jsdk_context_init` + 静态存储（或开 `JSDK_ENABLE_HEAP`） | |
| 7 | 新增 `desc_fetch()` 步骤（或导入缓存） | |
| 8 | 控制周期与**设备 `break_timeout`**（CAN 独有！**设备默认 0 = 禁用**，主站需主动武装并读回确认）对齐 | |
| 9 | 多关节同步：PDO 同步 → `jsdk_group_set_mit()`（**仅 node_id 1..7 可广播**） | |
| 10 | 故障恢复：`jsdk_joint_request_fault_reset()` + `jsdk_joint_get_fault_info()`（同名，不用改） | |
| 11 | 单位自检（`UNITS.zh-CN.md` §7 的三件套）在**不接电机**时先跑一遍 | |
| 12 | 真机冒烟按 `PORTING.zh-CN.md` §7.5.3 走一遍（位定时/终端电阻/error-frame 是 CAN 独有的坑） | |

---

## 5. 已知的迁移"软肋"（别踩）

| 坑 | 说明 |
|---|---|
| **两个库混链** | 符号同名 → 重复定义或静默错链。双跑期请**分进程**，不要分库 |
| `jsdk_joint_set_target_torque()` 语义变了 | 见 §3.4，这是最可能"编过但行为错"的一处 |
| EtherCAT 有分布式时钟，CAN 没有 | 多关节的**时间一致性**只能靠广播帧（一条帧同时到达）+ 各自的本地时基 |
| EtherCAT 周期是"总线周期"，CAN 是"帧" | 周期越短帧越多；1 kHz × 7 关节若各自单播 = 7000 帧/s，请用广播同步 |
| 设备掉线语义不同 | EtherCAT 靠 AL state / 工作计数器；CAN 靠 `age_ms` + `link_errors` + 设备自己的 `break_timeout` |
| `jsdk_joint_set_scale/get_scale` 在 CAN 版**语义不同且没有文档** | CAN 上是**恒等映射**（线上已是物理量），而 EtherCAT 版是 counts→rad 换算；两者同名 —— 直接照搬会写出错的换算。细节与修法见 **`BACKLOG.zh-CN.md` §1.2（A12）** |

---

## 6. 建议的迁移顺序

1. **先换传输、后调参**：把 EtherCAT 的 `context_create/add_joint/cycle` 换成 CAN 版，
   在**虚拟后端**（`jsdk_hal_virtual_open()`）上把所有逻辑跑通 —— 不需要硬件；
2. **接一台真设备**，只跑 `health/scan`（用 `jsdk-cli` 更快：见 `CLI.zh-CN.md`）；
3. **单关节使能 + 零增益**，确认 `fb.age_ms`、`link_errors` 正常；
4. 最后才加控制增益，并用 `UNITS.zh-CN.md` §7 的自检确认量纲没换错。
