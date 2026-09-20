# CYBERBEAST CAN 协议 — 固件问题清单（交付固件团队）

> **交付对象**：CYBERBEAST 固件维护者（`ODrive/Firmware`）
> **复核基线**：`ODrive` @ **`4ff46135`**（2026-09-20）。**所有行号都锚定在这个 commit 上**，
> 换版本后请按函数名重新定位（本清单**每条都给到函数名**，不依赖行号）。
> **来源**：JointSDK（`jsdk_can`）实现过程中**逐行核对固件源码**得出，不是推测。
> **本文件是固件问题的唯一权威清单**；`DESIGN.zh-CN.md` §10 与
> `PROTOCOL_NOTES.zh-CN.md` §14 已改为指向本文件（避免两份编号各自漂移）。

---

## 0. 怎么读这份清单

### 0.1 编号规则

| 规则 | 说明 |
|---|---|
| 编号 | `F1` … `F26`，**全局唯一、只增不复用**；一个号对应一件事 |
| 作废 | **F3 / F10 已作废**（号位保留，不复用），见 §6 |
| 合并 | **F4 已并入 F19**（同一件事的两种描述），见 §6 |
| 新增 | 本次汇总新发现的问题从 **F23** 起编号（不插队、不重排已有编号） |
| 引用 | 源码注释与测试里已存在 `F11`/`F13`/`F19`/`F22` 等引用，**编号保持不变**，所以这些引用全部继续有效 |

### 0.2 路径约定

- **固件路径**（`Firmware/...`、`docs/cyberbeast-protocol.md`、`tools/can/cyberbeast_tool.py`）
  均相对于 **`ODrive` 仓库根**。
- **SDK 路径**（`src/...`、`tests/...`、`include/...`）均相对于本仓库（`JointSDK`）根。
- 行号只在**同一个 commit 内**有意义，因此每条都同时给出**函数名**——换版本后按函数名定位。

### 0.3 列的含义

| 列 | 取值 | 含义 |
|---|---|---|
| 类型 | **缺陷** / **文档** / **需求** | 缺陷 = 实现与设计意图不符；文档 = 文档错/缺；需求 = 希望新增能力 |
| 严重度 | **致命** / **高** / **中** / **低** | 判据见下 |
| 状态 | **待修** / **建议修** / **仅文档澄清** / **不提出** | 我们建议的处理方式 |
| SDK 侧应对 | — | 我们在 `jsdk_can` 里已做的规避。**列出来不是为了让问题降级**，而是让你判断影响面与修复优先级 |

**严重度判据**

- **致命**：可能导致人身/设备损伤，或在正常使用下**静默**进入危险状态
- **高**：正常配置下会产生错误行为，且症状难以定位（安全阀失效、单位差一个齿比、越界反向满力矩）
- **中**：特定配置/特定操作下出错，或产生难以排查的静默错误
- **低**：一致性问题、文档/工具问题、极窄窗口

---

## 1. 汇总表（一页看完）

### 1.1 缺陷（实现与设计意图不符）

| # | 严重度 | 位置 | 现象 | 影响 | 建议 | SDK 侧应对 |
|---|---|---|---|---|---|---|
| **F19** | **致命** | `do_command()` `:259` / `auto_stop_if_timeout()` `:1420` | `is_ctrl` 不含 `0x04`，而超时函数首句是 `if (last_cmd_time_[i] == 0) return;` | 只用 `CURRENT_CONTROL` 的客户端**超时保护永远不武装**（CAN 掉线不 auto-stop）；反之先发过 `is_ctrl` 帧再只发 CURRENT 会被 `ERROR_CAN_BUS_FAILED` + `disarm()` **误停** | 把 `0x04` 纳入 `is_ctrl`，并用独立 `bool armed` 区分"未武装/已过期"（见 F20） | `jsdk_msgtype_feeds_watchdog()` + 客户端自动补喂 keepalive |
| **F22** | **致命** | `mit_control_cmd()` `:340` / `is_message_for_me()` `:113` | 槽位号 = `node_id`；`Dest=0xFF` 时任何 node_id 0~7 的设备都接受帧 | 若"位图里被置位、但槽位没填"（例如改成 `Dest=0xFF` 而只填了部分槽），该设备读到**全零槽** = `pos/vel/tau` 均为最小值 = **满力矩反向** | (a) 文档写明"接收端按位图逐位校验"这条安全网；(b) 文档把"0 = 字段最小值"显著标出并给零增益示例；(c) 建议"全员广播"独立 `MsgType`，避免与"8 位全置位"混淆 | 广播路径**两遍**写入：先给每个未使用槽位写显式零增益指令，再填真实目标 |
| **F5** | **高** | `odrive_can.cpp:20` + `:112` | `axis = &odrv.get_axis(0);` → `service_stack(*axis)` 只传 axis0 | 多轴板上**只有 axis0 做超时检查**，其余 axis 的 CAN 掉线**永不 auto-stop** | 在 `service_stack` 内对 `axes[0..AXIS_COUNT)` 全部调用 `auto_stop_if_timeout()`（心跳循环已是全轴） | 单轴目标平台暂不受影响；已记为"多轴部署前必须确认" |
| **F14** | **高** | `cmd_torque_control()` `:517` / `mit_control_cmd()` `:359` / `cmd_current_control()` `:537` | `TORQUE_CONTROL` 直接写 `input_torque_ = 目标值`（**电机端**），MIT 却做 `motor_torque = torque / gear_ratio`（**输出端**）；`CURRENT_CONTROL` 又乘 `torque_constant` | 同一物理量在不同帧单位不同：等价命令的**实际输出差一个 `gear_ratio` 倍**（16.5 → 16.5 倍力矩差异）；电流模式依赖 `torque_constant` 标定 | 统一为输出端；或至少把三个帧的单位在文档里写成表、函数改名（`..._MOTOR`/`..._OUTPUT`） | 对外只给**输出端**语义（`jsdk_joint_set_target_torque_Nm()`），发 `0x03` 前在 `jsdk_joint.c:651` 除以 `gear_ratio`；`PROTOCOL_NOTES.zh-CN.md` §4.7 逐帧列端别 |
| **F16** | **高** | `can_simple.cpp:9` `float_to_uint()` | `(int)((x-offset)*(2^bits-1)/span)`：**截断 + 不钳位** | 越界命令被掩码回绕：`+12.625 rad`（仅超 1%）→ `−12.376 rad`；`+50.5 N·m` → `−49.5 N·m`（**反向满力矩**） | 四舍五入 + 钳位 | 客户端发帧前钳位（`cb_mit_pack_command`），并断言越界必被拦 |
| **F11** | 中 | `send_heartbeat()` `:1319` | `(uint8_t)(state << 4) \| (control_mode & 0x0F)`，而 `AXIS_STATE_MOTOR_DEADTIME_CALIBRATION = 16`（`autogen/interfaces.hpp:351`） | **状态 16 被静默截断为 0**（UNDEFINED），上位机看到"未定义状态"却查不出原因（控制模式 nibble 正常） | Classic 心跳 8 B 已排满且 4 bit 放不下 16 → 建议**文档限定 state 仅对 0~15 有效**，并给 FD 心跳补一个完整 `state` 字节（详见 §3.1） | 不信任心跳 state；状态判定以 `axis0.current_state` 端点为准 |
| **F12** | 中 | `send_heartbeat()` FD 分支 `:1380`–`:1397` | `vbus/ibus/pos/vel` 直接 `static_cast` **未钳位**（Classic 分支 `:1347`/`:1351` 的 pos/vel、以及两分支的 Iq 都钳了） | 越界值行为未定义（实际表现为回绕变号）：例如 `ibus_ = 400 A` → `(int16_t)(40000)` 回绕成负数 | 对齐 Classic 分支的 `std::max/min` 钳位 | 心跳只当作"快照"用；精确值走 QUERY_* / 端点 |
| **F13** | 中 | `cmd_param_write_segmented()` `:835` | 末块用 `size_t(total_len)` 交给 `endpoint_handler`，**不校验实际已填字节数**（`param_write_asm_.offset`） | 主站若把"单块声明 `TotalLen=8` 且 `More=0`"发出去，后 4 字节来自清零缓冲 → **静默写入错误值** | 写入前校验 `offset == total_len`，否则丢弃并回 ERR | `cb_param_pack_write_chunk()` 三条拒绝规则，保证 SDK 永不生成这种块 |
| **F15** | 中 | `cmd_query_pos_vel()` `:1030`–`:1031` vs `send_mit_response()` `:391` | QUERY_POS_VEL 透传 `pos_estimate_linear_src_`（**电机端 turns**），MIT 响应做 `pos_out = pos × 2π/gear_ratio`（**输出端 rad**） | 同一物理量在两个帧里单位不同，客户端极易写错（差 2π/gear_ratio ≈ 0.38） | 统一单位；或改名（`MOTOR_POS_TURNS`）避免误用 | 单位换算集中在 `src/core/jsdk_units.c`；`PROTOCOL_NOTES.zh-CN.md` §4.7 逐帧列端别 |
| **F18** | 中 | `send_mit_response()` `:402` / `handle_can_message` | 形参 `seq` **从未使用**；CAN ID 用设备本地滚动计数 `tx_seq_++` | 上位机**无法用 `Seq` 关联请求与响应**；同一 `(node, msgtype)` 上并发多个待响应请求会串味 | 回显请求的 `Seq`；文档明确"按 `(source, dest, msgtype)` 匹配" | 按 `(SRC, DEST, MsgType)` 匹配（与厂商工具 `request_response()` 一致）；多上下文要求不同 `master_id` |
| **F23** | 中 | `init()` `:196` + `docs/cyberbeast-protocol.md:156` | `rx_seq_` **只被初始化为 0xFF，此后从不读取**；而协议文档明确写"接收方检测 Seq 是否连续，`(cur-last) mod 4 ≠ 1` 则丢包"，并把"丢包检测 ✅"列入协议对比表 | **文档声称的能力未实现**（配合 F18，`Seq` 整套机制目前实际无效） | 二选一：(a) 实现接收侧校验并统计丢包；(b) 文档改为"`Seq` 目前仅保留字段、无功能" | 客户端不依赖 `Seq` 判断丢包（按超时/新鲜度判） |
| **F6** | 中 | `cmd_query_current()` `:1053` | 第 2 项用 `Idq_setpoint_->first`（**Id 设定值**），但注释与文档写 "Id Measured" | 客户把设定值当测量值用（例如做电流环诊断）→ 结论错 | 返回 `Id_measured_`，或文档明确写"第 2 项为 Id 设定值" | 文档标注"第 2 项为设定值" |
| **F24** | 低 | `can_cyberbeast.hpp` 成员 `active_report_enabled_`；`init()` `:191` | 该成员**只被赋过一次 `false`，从不读取** | 头注释写着"广播后是否主动上报响应"，即**曾计划的功能未实现**（与 F8 相关） | 实现或删除该成员，别留死代码误导 | 不依赖"广播后主动上报" |
| **F25** | 低 | `cmd_current_control()` `:530`–`:538` | 该函数**不发任何响应帧**（`0x01`/`0x02`/`0x03` 都会 `send_mit_response`） | 纯电流模式下主站拿不到设备应答（无法确认已生效），也让 CURRENT 路径更难诊断 | 与其他控制帧一致地回一帧；或文档写明"0x04 无响应" | 文档写明 0x04 无响应；客户端不等应答 |
| **F20** | 低 | `auto_stop_if_timeout()` `:1420` | `last_cmd_time_ == 0` 同时表示"从未收到控制帧"和"开机第 0 ms 收到" | 开机瞬间的控制帧无法武装超时保护（窗口极窄，但语义有歧义） | 用独立 `bool armed`，或哨兵取 `UINT32_MAX` | 仿真设备时钟从 1 ms 起（规避同款歧义） |
| **F17** | 低 | `cmd_param_read()` | 批量请求在 Classic 上正确回 `ERR`，但**单读的重试/回退没有节流** | 主站实现不当时可能形成重试风暴（设备侧无错，风险在主站） | 文档写明"主站应限速重试"即可 | 按类型拆分重发 + 单次读失败即报错，不做无界重试 |

### 1.2 文档问题（固件方拥有的协议文档）

| # | 严重度 | 位置 | 现状 | 应改为 | 影响 |
|---|---|---|---|---|---|
| **F26** | 中 | `docs/cyberbeast-protocol.md:112/124/131/138` | `Dest=0xFF` 被描述为"全局广播（所有设备**响应**/接收）"；`Dest` 语义表未区分"广播类型 / 点对点类型" | 分两句写清：① 对**广播类型**（`MsgType ≥ 0x80`）`0xFF` = 全部 8 位；② 对**点对点类型**（`MsgType < 0x80`）`Dest` 必须等于目标 `node_id`，`0xFF` **无人接收**；③ 广播帧**从不产生响应**（"响应"一词删除） | 客户照文档发 `Dest=0xFF` 的点对点请求会"石沉大海"而查不出原因 |
| **F1** | 中 | `docs/cyberbeast-protocol.md:353/366` | `KP (刚度) … N·m/rad`；`mit_max_kp = 500 N·m/rad` | 写明 kp 的作用点：固件把 kp **原样**交给 MIT 控制器，而 `input_pos` 是**电机端 turns**（`mit_control_cmd()` `:356`–`:359`）→ **输出端等效刚度 = kp × gear_ratio / (2π)**（gear=16.5 时 ≈ 2.63×）。或新增 `mit_kp_unit` 能力标志 | 客户按文档理解 kp 会**系统性调小 2.63 倍增益**（串级回路下可能振荡） |
| **F21** | 中 | `Firmware/docs/cyberbeast-json-descriptor-protocol.md` | 0x24/0x25 是**全量流式**传输（`0x24` 只带 `Offset`，`Firmware/.../can_cyberbeast.cpp:879`–`:893`），文档**没有说明这一点** | 明确写出：设备端**永远全量发送**、无服务端 filter、无"传输完成"信号；客户端若按自己的 filter"凑够就提前停"，必须自行保证不会丢同族路径（前缀/通配 filter 会被**首个**匹配项误判为已满足） | 客户端自行提前终止时会**静默丢字段**，且丢哪些取决于 JSON 字段顺序 → 不同固件版本得到不同的残缺端点表，极难复现 |
| **F6** | 中 | `docs/cyberbeast-protocol.md`（QUERY_CURRENT 一节） | 第 2 项写作 Id **测量值** | 改为"Id **设定值**"（代码是 `Idq_setpoint_`，见 F6） | 同 F6 |
| **F23** | 中 | `docs/cyberbeast-protocol.md:156`–`164`、`:982` | 声称接收侧做 Seq 连续性检测、对比表标"丢包检测 ✅" | 与 F23 的实现决定保持一致（实现 or 降级为"保留字段"） | 客户以为有丢包检测，实际没有 |

> **说明**：F21 原先被记为"文档与工具把提前终止当成无条件优化"，本次**回源复核后修正**——
> 固件仓库（含 `tools/can/cyberbeast_tool.py`）**没有任何"提前终止"的实现或描述**，
> 该优化是**客户端（我们 SDK）自己的设计选择**。所以这条的性质是"文档**缺**一条警告"，
> 不是"文档写错了"。详见 §6.1。

### 1.3 能力需求（希望新增/放宽，不是缺陷）

| # | 严重度 | 需求 | 收益 | SDK 侧应对 |
|---|---|---|---|---|
| **F1** | 中 | 统一 `kp/kd` 量纲，或在 JSON 描述符增加 `mit_kp_unit` / `mit_kd_unit` 能力标志 | 消除 §1.2 的歧义，客户调参可预期 | 客户端按"输出端刚度"语义换算，并在文档里给出换算公式 |
| **F2** | 低 | 增加 `protocol_version`（如 `can.config.protocol_version`）或 `capabilities u32` 端点 | 能力发现不再依赖"猜端点 ID + 比版本号"（端点 ID 跨版本漂移率实测 **86%**） | 全动态 JSON 描述符（不依赖任何静态端点表） |
| **F7** | 中 | 位掩码寻址与 MIT 槽位扩展到 `node_id ≥ 8` | `MAX_BROADCAST_DEVICES = 8`（`can_cyberbeast.hpp:47`）→ 12 自由度机器人**无法一帧广播同步** | `node_id ≥ 8` 时自动降级单播 + `configure()` 提前告警 |
| **F8** | 中 | 广播帧支持"回复聚合"，或提供一个**广播类型**的状态请求（可分时回复） | 目前广播后完全无反馈；而 `MSG_STATUS_FEEDBACK = 0x49` 是**点对点类型**，无法组播（`is_message_for_me` 对非广播类型要求 `Dest == node_id`） | 广播后不期待反馈；需要反馈时逐个单播 `0x49` |
| **F9** | 中 | `break_timeout = 0` 语义明确为"关闭"（或新增 `enable_break_timeout`） | 目前 0 被强制当作 100 ms（`auto_stop_if_timeout()` `:1423`–`:1426`），**无法关闭**该保护 | 文档写明"0 = 100 ms，不是关闭"；客户端显式设非零值 |

---

## 2. 致命 / 高危条目详述

### 2.1 F19 — 纯 `CURRENT_CONTROL` 客户端的超时保护永不武装（致命）

**现象**：`do_command()` 第一条语句是 `axis.watchdog_feed()`（**无条件**，任何帧都喂 ODrive 轴看门狗），
但决定 CAN **auto-stop** 的 `is_ctrl` 是：

```cpp
// can_cyberbeast.cpp:259
bool is_ctrl = (msgtype <= MSG_TORQUE_CONTROL)          // 0x00~0x03
            || (msgtype >= MSG_MIT_CONTROL_BCAST && msgtype <= MSG_TORQUE_CONTROL_BCAST);  // 0x80~0x83
if (is_ctrl && axis_idx < AXIS_COUNT) {
    last_cmd_time_[axis_idx] = xTaskGetTickCount();     // :262
}
```

`MSG_CURRENT_CONTROL = 0x04` 不在其中。而超时函数：

```cpp
// can_cyberbeast.cpp:1420
if (last_cmd_time_[axis_idx] == 0) return;              // 从未武装 → 直接返回
```

**触发条件**：客户端**所有**控制帧都用 `CURRENT_CONTROL(0x04)`。

**后果（两个方向都不安全）**：

1. **安全阀失效**：`last_cmd_time_` 永远是 0 → `auto_stop_if_timeout()` 立即返回 →
   **CAN 线拔掉/主站崩溃后设备不会 auto-stop**，会一直按最后一个电流指令驱动（`disarm()` 不会被调用）。
2. **误停**：若客户端**先**发过任意 `0x00~0x03`/`0x80~0x83` 帧（例如使能序列用了 MIT 或 `0x63 STOP_MOTOR`），
   之后只发 `0x04` → `last_cmd_time_` 不再刷新 → **100 ms 后被判 `ERROR_CAN_BUS_FAILED` + `disarm()`**，
   正常控制流被自己打断。

**建议**：把 `0x04` 纳入 `is_ctrl`（一行），并把"未武装"与"已过期"用不同哨兵表示（见 F20）。

**SDK 侧应对**：`jsdk_msgtype_feeds_watchdog()` 精确复刻固件的 `is_ctrl` 语义
（`src/core/jsdk_watchdog.c:13` 注明"纯 `CURRENT_CONTROL` 的客户端**永远不武装**这个保护（安全缺口 F19）"），
并在客户端**自动补喂** `MIT` keepalive，确保走 CURRENT 也不会被误停。
但**安全阀失效这一半我们无法在固件侧规避**——这正是本条的 P0 理由。

**回归验证建议**：新增用例 —— 只发 `0x04` 若干帧后停发，断言 2×`break_timeout` 内必须出现
`ERROR_CAN_BUS_FAILED` 且 `motor_.disarm()` 被调用。

---

### 2.2 F22 — 广播槽位语义与 `Dest=0xFF` 的满力矩风险（致命）

**现象**：广播 MIT 的槽位号就是 `node_id`，且帧长必须覆盖到该槽：

```cpp
// can_cyberbeast.cpp:340 (mit_control_cmd)
slot = device_id;                                        // 槽位 = node_id
if (msg.len < (slot + 1) * 8) return;                    // 太短就安全地忽略
```

`Dest=0xFF` 在接收侧被视为"全部 8 位"（`is_message_for_me()` `:122`）：
`if (dest == ADDR_BROADCAST) return true;` —— **任何 node_id 0~7 的设备都会接受这一帧**。

**触发条件**：一次广播里**位图置位了、但对应槽位没写有效内容**。最容易出现的两种写法：

- 稀疏分组（只控 1 号与 5 号）却把 `Dest` 改成 `0xFF`（"反正都一样，图省事"）；
- "把 64 字节缓冲清零，只填自己关心的槽位"（**这是最自然的实现方式，也是错的**）。

**后果**：全零槽位经 `uint_to_float` 解码 = 各字段的**最小值**：

| 字段 | 全零码解码结果 |
|---|---|
| `pos` | `−mit_max_pos` |
| `vel` | `−mit_max_vel` |
| `tau` | `−mit_max_torque` |

即 **满力矩反向**。设备侧没有任何"零值保护"（0 在 MIT 编码里是有意义的正值端点，不是"无效"）。

**建议**（三条，按性价比排序）：

1. **文档**：把"0 = 字段最小值"在协议文档 §4.1 显著标出，并给一个"零增益指令"的 8 字节示例；
2. **文档**：写明接收端**已经**用 `Dest` 位图逐位校验（`(dest & (1 << my_id)) != 0`，见 `is_message_for_me()` `:125`）——
   这是客户端唯一的安全网，值得显式声明；
3. **协议**：把"全员广播"拆成独立 `MsgType`（例如 `0x84`），避免 `Dest=0xFF` 与"8 位全置位"语义重叠，
   让客户端无法"顺手"写出危险帧。

**SDK 侧应对**（`jsdk_group_set_mit()`，见 `assert` 于 `tests/test_group.c`）：
FD 路径**两遍**构造 —— 先求本轮最大槽号 `max_slot`，对 `0..max_slot` 中**每一个未使用的槽位**
显式调用 `cb_mit_pack_command(dst, 0,0,0,0,0)`（零增益指令），再填真实目标；
帧长取 `(max_slot+1)*8`。测试断言"空闲槽位字节**不是**全零"，并与逐槽手拼结果**逐字节相同**。
另：`cb_make_broadcast_mask()` 要求所有 `node_id ∈ 1..7`，否则返回 −1 让上层降级单播。

---

### 2.3 F5 — 只有 axis0 做 CAN 超时检查（高）

```cpp
// odrive_can.cpp:20
axis = &odrv.get_axis(0);
// odrive_can.cpp:112
next_service_time = std::min(can_cyberbeast_.service_stack(*axis), next_service_time);
```

`CANCyberBeast::service_stack(axis)` 内部**心跳**循环遍历了全部 axis
（`:1465`–`:1468`：`for (size_t i = 0; i < AXIS_COUNT; ++i) send_heartbeat(ax, isClassic);`），
但**超时检查**只对传入的那一个 axis：

```cpp
// can_cyberbeast.cpp:1440 (service_stack 开头)
auto_stop_if_timeout(axis);          // :1442 —— 只有这一个 axis
```

**后果**：多轴板（`AXIS_COUNT > 1`）上，除 axis0 外的轴 **CAN 掉线后永不 auto-stop**，
而心跳照常上报（看起来"设备在线"）→ 现场表现为"某一关节不听使唤地保持最后指令"。

**建议**：把超时检查改成全轴循环（与心跳循环一致），并加一个"多轴各自独立武装/超时"的用例。

**SDK 侧应对**：在单 axis（`AXIS_COUNT == 1`）平台不受影响；已把"多轴部署前必须确认此条已修"
写进交付注意事项。**建议固件侧优先修这一条**——改动量小、风险低、收益明确。

---

### 2.4 F14 / F16 — 单位差一个齿比、越界回绕反向满力矩（高）

**F14**（力矩端别）：

| 帧 | 代码 | 实际单位 |
|---|---|---|
| `MIT_CONTROL (0x00)` | `motor_torque = torque / gear_ratio`（`:359`） | **输出端** N·m |
| `TORQUE_CONTROL (0x03)` | `input_torque_ = target_torque_nm`（`:517`） | **电机端** N·m |
| `CURRENT_CONTROL (0x04)` | `input_torque_ = target_current_a * torque_constant`（`:537`） | 电机端 N·m（且依赖 `torque_constant` 标定） |

同一个数值 `10` 在 MIT 下是"输出端 10 N·m"，在 TORQUE 下是"电机端 10 N·m"——
**输出差 `gear_ratio` 倍**（16.5）或**反向差 16.5²**（若客户按输出端调参再切模式）。

**F16**（定点回绕）：

```cpp
// can_simple.cpp:9
int float_to_uint(float x, float x_min, float x_max, int bits){
  float span = x_max - x_min;
  float offset = x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);   // 截断；无钳位
}
```

`int` 结果随后被 `& 0x0FFF` 之类截成 12 bit（`pack_mit_command()` `:49`–`:53`）→
**越界输入不会饱和，而是回绕**。实测：`+12.625 rad`（超量程 1%）→ `−12.376 rad`；
`+50.5 N·m` → `−49.5 N·m`。**只超一点点就反向满力矩**，且上位机无从察觉。

**SDK 侧应对**：`cb_mit_pack_command()` 采用"四舍五入 + 钳位"（与固件不同，已与用户确认保留），
并有回归用例复刻固件的截断/回绕语义，证明"若 SDK 不钳位就会复现飞车"
（`tests/test_group.c`、`tests/test_proto.c`）。

**建议**：`float_to_uint` 改为四舍五入 + 钳位（一行 `std::max/min`）。
**注意**：这是**破坏性变更**吗？不是——越界输入本来就没有正确行为可言。

---

## 3. 中/低缺陷要点

### 3.1 F11 — 心跳状态 16 被截断为 0

```cpp
// can_cyberbeast.cpp:1319
uint8_t state_mode = (static_cast<uint8_t>(axis.current_state_) << 4)
                   | (static_cast<uint8_t>(axis.controller_.config_.control_mode) & 0x0F);
```

`AXIS_STATE_MOTOR_DEADTIME_CALIBRATION = 16`（`Firmware/autogen/interfaces.hpp:351`）→
`16 << 4 = 256` → `(uint8_t)` 后为 **0** → 上位机读到 `state = UNDEFINED`。
**建议**：低 4 bit 放 `state` 的低 4 位不行（16 也放不下）→ 需要重排字段：
建议 **bit7~bit5 = `life`（已有）、bit4 = `state` 高位** 这类方案，或干脆用 FD 心跳多出的字节单列 `state`。
（Classic 心跳只有 8 字节且已排满，故建议**在协议文档里明确"state 字段仅对 0~15 有效，
16 会被截断"**，并在 FD 心跳里补一个完整 `state` 字节。）

### 3.2 F12 — FD 心跳未钳位

Classic 分支对 pos/vel（`:1355`–`:1365`）和 Iq（两分支都有）用了 `std::max/min` 钳位，
FD 分支的 `vbus`（`:1380`，uint16）、`ibus`（`:1385`，int16）、`pos`（`:1390`，int32）、
`vel`（`:1395`，int32）**直接 `static_cast`**。
`ibus_ = 400 A` 时 `(int16_t)(40000)` 回绕为负 → 上位机看到的母线电流符号错误。
**建议**：对齐 Classic 分支的写法。

### 3.3 F13 — 分段写末块不校验已填长度

```cpp
// can_cyberbeast.cpp:835  （末块）
fibre::cbufptr_t input_buffer{param_write_asm_.buf, size_t(total_len)};
```

装配过程只保证 `offset <= total_len`（`:815`/`:827`），**不保证 `offset == total_len` 就写入**。
于是"单块声明 `TotalLen=8`、`More=0`"会把后 4 个字节（清零缓冲）一起写进端点 → **静默写错值**。
**建议**：`if (param_write_asm_.offset != total_len) { active = false; return; }`（或回 ERR）。
**SDK 侧应对**：`cb_param_pack_write_chunk()` 三条拒绝规则（非末块不满 4 B / 非末块声明 More 却已填满 / 末块未刚好补齐），
保证 SDK 作为主站永不生成这种块。

### 3.4 F15 — QUERY_POS_VEL 与 MIT 响应单位不同

| 帧 | 字段 | 代码 | 单位 |
|---|---|---|---|
| `QUERY_POS_VEL (0x41)` | pos / vel | `:1026`–`:1029` | **电机端 turns / turns/s** |
| `MIT` 响应 (`0x00` 回帧) | pos / vel | `:391`–`:392` | **输出端 rad / rad/s** |

比值 `2π/gear_ratio ≈ 0.381`。客户把两个来源的数据混用（例如用 0x41 做位置环反馈、
用 MIT 响应做显示）会得到相差 2.6 倍的位置。
**建议**：统一，或改名（`MOTOR_POS_TURNS`）。

### 3.5 F18 / F23 — `Seq` 机制目前实际无效

- **F18**：`send_mit_response(axis, master_id, seq)` 收了 `seq` 但**没用**，
  CAN ID 里填的是本地滚动计数 `tx_seq_++`（`:402`）。→ 客户端**不能用 `Seq` 关联请求与响应**。
- **F23**：`rx_seq_[]` 只在 `init()` 里被置 `0xFF`（`:196`），**从不读取**。→ 协议文档
  `docs/cyberbeast-protocol.md:156`–`164` 所述的"接收侧 Seq 连续性检测"**没有实现**，
  对比表里的"丢包检测 ✅"（`:982`）与实现不符。

**建议**：二选一，但必须一致 —— 实现（接收侧校验 + 丢包计数），或把文档降级为
"`Seq` 为保留字段，当前不参与丢包判定"。**我们建议后者**（成本低、且现有客户端已按
`(SRC, DEST, MsgType)` 匹配，不依赖 `Seq`）。

### 3.6 F24 / F25 — 两处"半成品"

- **F24**：`active_report_enabled_`（头注释："广播后是否主动上报响应"）**只赋值不读取**
  → 该功能未实现。与 **F8**（广播无反馈）是同一件事的两面。建议实现或删除，避免误导。
- **F25**：`cmd_current_control()` 是**唯一不发响应帧**的控制命令（`0x01`/`0x02`/`0x03` 都回一帧）。
  纯电流模式本来就缺安全网（F19），再拿不到应答会让现场排障更难。建议补一帧。

### 3.7 F20 — `last_cmd_time_ == 0` 哨兵歧义

`0` 既表示"从未武装"，又表示"开机第 0 ms 收到控制帧"，超时触发后也用它复位（`:1432`）。
后果是"开机瞬间"与"超时后"两种情形无法区分。
**建议**：独立 `bool armed`，或哨兵改用 `UINT32_MAX`。

---

## 4. 建议的修复顺序

| 优先级 | 条目 | 理由 |
|---|---|---|
| **P0** | **F19**、**F22（文档部分）** | 安全相关，且改动小：F19 是编码 `is_ctrl` 的一行；F22 先把"0 = 字段最小值""接收端按位图校验"写进文档，成本几乎为零 |
| **P1** | **F5**、**F16**、**F14** | 多轴安全阀、越界飞车、单位差一个齿比 —— 都是"正常使用下会出错且难定位" |
| **P2** | F11、F12、F13、F15、F6、F23/F18、F26、F1(文档侧)、F21 | 静默错误与文档误导；多数可以只改文档 |
| **P3** | F17、F20、F24、F25、F2、F7、F8、F9 | 一致性、体验、能力扩展 |

> **改动成本提示**：P0/P1 里真正要改代码的只有 **F19（1 行）、F5（1 个循环）、F16（1 行钳位）**；
> **F14** 若选择"只改文档"也可立即消除歧义（把三个帧的端别写成表）。
> 其余多数条目"改文档"即可闭环，且能显著减少现场排障时间。

---

## 5. 作废 / 合并条目

| # | 原因 |
|---|---|
| **F3** | ~~端点 ID 稳定性~~ → **不提出**。SDK 已改为**全动态 JSON 描述符解析**（`jsdk_context_configure()` 内下载并增量解析），不依赖任何静态端点表，因此 ID 漂移不再是问题 |
| **F10** | ~~元数据帧标志字节~~ / ~~`chunkOffset` 扩为 u32~~ → **不再提需求，SDK 自行规避**：靠"一次请求的第一帧必为元数据帧"的时序约定，并对 `total_len > 65535` 提前返回 `JSDK_ERR_UNSUPPORTED`。**仅提示**：当前描述符 **41029 字节**（占 65535 的 63%），端点表继续增长时会撞上限 |
| **F4** | 已并入 **F19**。原描述"`CURRENT_CONTROL(0x04)` 也刷新看门狗计时"用词不准（`watchdog_feed()` 本来就是无条件调用），真正缺的是 `last_cmd_time_` 刷新 —— 即 F19 |

---

## 6. 复核方法与证据锚点

### 6.1 我们是怎么得出这些结论的

1. **逐个函数读源码**（不是读文档）：`can_cyberbeast.cpp`（1473 行）、`can_cyberbeast.hpp`（328 行）、
   `can_simple.cpp`（`float_to_uint`）、`odrive_can.cpp`（服务线程与 axis 传递）。
2. **交叉核对文档**：`Firmware/docs/cyberbeast-json-descriptor-protocol.md`、
   `docs/cyberbeast-protocol.md`、`Firmware/autogen/interfaces.hpp`、`Firmware/autogen/endpoints.hpp`。
3. **以测试固化**：每条"SDK 侧应对"都有对应用例（见下表），并在必要时做**变异测试**
   （把修复改回缺陷 → 用例必须失败）证明用例有效。
4. **回源修正过本清单**：F21 的第一版结论（"文档与工具把提前终止当成无条件优化"）在
   **全仓搜索后不成立**（固件与工具均无此描述），已改写为"文档缺一条警告"。
   —— 这类修正也说明：**本清单里的每一条都可以被独立复核**，请放心质疑。

### 6.2 SDK 侧对应实现（便于交叉验证）

| 条目 | SDK 侧位置 |
|---|---|
| F19 / F4 | `src/core/jsdk_watchdog.c`、`jsdk_msgtype_feeds_watchdog()`、`tests/test_joint.c:[6]` |
| F22 | `src/core/jsdk_group.c`（`jsdk_group_set_mit()` 两遍构造）、`tests/test_group.c`（空闲槽位非全零、逐字节对拍） |
| F13 | `src/proto_cyberbeast/cb_param.c`（`cb_param_pack_write_chunk()` 三条拒绝规则）、`tests/test_proto.c:[10]` |
| F16 | `src/proto_cyberbeast/cb_mit.c`（四舍五入 + 钳位）、`tests/test_group.c`（复刻固件回绕语义的对照用例） |
| F14 / F15 | `src/core/jsdk_joint.c:651`（发 `0x03` 前 ÷ `gear_ratio`）、`src/core/jsdk_units.c`、`docs/PROTOCOL_NOTES.zh-CN.md` §4.7（逐帧单位表） |
| F11 | 状态判定改读 `axis0.current_state` 端点，不信任心跳 state |
| F1 | `jsdk_joint_set_mit_stiffness()`（以**真实输出端刚度**为入参，内部 `kp = stiffness × 2π / gear_ratio`）、`include/joint_sdk/joint_sdk.h` 的 `@warning` 给出换算公式、`docs/DESIGN.zh-CN.md` §6.2 |
| F18 | 按 `(SRC, DEST, MsgType)` 匹配响应 |
| F2 / F3 | 全动态 JSON 描述符解析（`src/proto_cyberbeast/cb_jsondesc_parse.c`） |
| F9 | 文档写明"0 = 100 ms，不是关闭" |
| F23 | 客户端不依赖 `Seq` 判丢包（按反馈新鲜度/超时判） |

### 6.3 复核锚点

| 项 | 值 |
|---|---|
| 仓库 | `ODrive` |
| commit | **`4ff46135`**（`feat(Ecat): 支持通过 CoE 读取 JSON 端点描述符并读写任意端点参数`） |
| 复核日期 | 2026-09-20 |
| 主文件 | `Firmware/communication/can/can_cyberbeast.cpp` @ 1473 行 |
| 端点表 | `Firmware/autogen/endpoints.hpp`（描述符 **41029 B / 594 端点**） |
| 协议文档 | `docs/cyberbeast-protocol.md`（v2.4，1124 行）、`Firmware/docs/cyberbeast-json-descriptor-protocol.md`（v1.0，212 行） |

---

## 7. 变更记录（本清单自身）

| 版本 | 日期 | 说明 |
|---|---|---|
| v1.0 | 2026-09-20 | 首次汇总。合并 `DESIGN.zh-CN.md` §10（原 F1~F10 需求）与 `PROTOCOL_NOTES.zh-CN.md` §14（原 F11~F22 缺陷），**统一编号并回源复核全部条目**；新增 F23（`rx_seq_` 未实现丢包检测）、F24（`active_report_enabled_` 死成员）、F25（0x04 无响应）、F26（`Dest` 文档语义）；修正 F21 的证据；F3/F10 作废、F4 并入 F19；补严重度、SDK 侧应对与修复顺序 |
