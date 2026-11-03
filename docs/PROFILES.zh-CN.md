# 设备 profile（多固件词汇兼容）

> 目标：**L3 只依赖语义，不依赖某一家的端点名。**  
> 同一份 JointSDK 可以同时接 ODrive 词汇的关节和 MIT 扁平词汇的关节；以后加新厂商
> （例如 moteus）不需要再改 `jsdk_config.c` / `jsdk_ops.c`。

---

## 1. 它解决什么问题

`jsdk_context_configure()` 早期把 7 条 ODrive 路径写死在 `jsdk_config.c` 里：

```text
axis0.motor.config.gear_ratio
axis0.motor.config.torque_constant
axis0.controller.config.mit_max_pos / vel / torque / kp / kd
```

任何换了词汇的固件（例如 `config.gr`、`config.kt`、`config.p_max` …）都会在
“描述符已下好、握手也通了”之后死在 `required endpoint missing`。

现在这些名字只存在于 **`src/core/jsdk_profile.c` 的候选路径表**里。

---

## 2. 解析流程

```text
jsdk_context_configure()
        │
        ├── 下载 / 导入 JSON 描述符（端点名 → ID，仍全部来自设备）
        │
        └── jsdk_profile_resolve()
                │  1. family_hint（默认 AUTO）
                │  2. 按 g_profiles[] 顺序试：ODRIVE → MIT → （未来）MOTEUS
                │  3. 必需语义齐全 → 写回 j->ep_* + j->profile_flags
                └── 失败：JSDK_ERR_NOT_FOUND，错误串列出两族候选名
```

`AUTO` **先试 ODrive**，所以老设备的行为逐字节不变。

端到端解析结果可用：

```c
jsdk_joint_get_device_family(j);   /* 0=AUTO/未解析, 1=ODRIVE, 2=CYBERBEAST_MIT */
```

需要强制选路（多套词汇共存 / 测试 / 新厂商）时：

```c
jsdk_joint_set_device_family(j, JSDK_DEVFAMILY_ODRIVE);   /* configure() 之前 */
```

---

## 3. 语义键与能力位

| 语义键 | ODrive 词汇 | MIT 扁平词汇 |
|---|---|---|
| `GEAR` | `axis0.motor.config.gear_ratio` | `config.gr` |
| `TCONST` | `axis0.motor.config.torque_constant` | `config.kt` |
| `CURLIM` | `axis0.motor.config.current_lim` | `config.i_max` |
| `POS_MAX` | `axis0.controller.config.mit_max_pos` | `config.p_max` |
| `VEL_MAX` | `axis0.controller.config.mit_max_vel` | `config.v_max` |
| `TAU_MAX` | `axis0.controller.config.mit_max_torque` | **无 → 派生** |
| `KP_MAX` / `KD_MAX` | `…mit_max_kp` / `…mit_max_kd` | `config.kp_max` / `config.kd_max` |
| `NODE_ID` | `axis0.config.can.node_id` | `config.can_id` |
| `HB_RATE` | `axis0.config.can.heartbeat_rate_ms` | `config.can_hb_ms` |
| `WATCHDOG` | `can.config.break_timeout`（u16） | `config.can_timeout`（int32） |
| `CURRENT_STATE` | `axis0.current_state` | 无（用 nibble） |
| `REQUESTED_STATE` | `axis0.requested_state` | 无 |
| `ACTION_CALIBRATE` | 无 | `action.calibrate`（function） |

能力位（`JSDK_PF_*`）决定 L3 的行为：

| 能力位 | 含义 | 谁有 |
|---|---|---|
| `TAU_MAX_EXPLICIT` | 有显式力矩量程端点 | ODrive |
| `TAU_MAX_DERIVED` | `tau_max = cur_lim × kt × gear` | MIT |
| `WD_FEEDS_CURRENT` | `0x04 CURRENT` 也刷新固件超时 | MIT |
| `HAS_HOME` | `requested_state` 支持 HOMING(11) | ODrive |
| `ACTION_CALIBRATE` | 标定走 `action.calibrate` | MIT |
| `STATE_FROM_NIBBLE` | 状态由 MIT 响应 / 心跳 nibble 提供 | MIT |
| `ZERO_REQUIRES_IDLE` | `SET_ZERO` 只在静息态生效 | MIT |
| `ZERO_NEEDS_BOOT` | `SET_ZERO` 只写**零点存储**（并落 Flash）；报告位置**不保证**立刻为 0（多圈位置连续累计） | MIT |

---

## 4. 语义差异（不是名字差异）

| 能力 | ODrive | MIT 扁平词汇 |
|---|---|---|
| 标定 | 写 `requested_state = 3`，等 AxisState 回静息 | 调 `action.calibrate`，等 MIT 响应 nibble `1 → 2` |
| 回零 | `requested_state = 11` | **不支持** → `JSDK_ERR_UNSUPPORTED` |
| 置零 | 闭环里也可；位置**当场**归 0，需 `save` 才持久 | 必须 `MENU_MODE`；SDK 先查一次 nibble，不满足返回 `BAD_STATE`。零点**连 Flash 一起写**；报告位置是否立刻为 0 取决于设备内部多圈累计（实测两种都出现过）⇒ 判据用零点端点，位置不巧不是 0 时置 `JSDK_JF_ZERO_PENDING_REBOOT` |
| 当前状态 | `axis0.current_state` | 心跳 / MIT 响应 nibble（映射成 1/4/8） |
| 协议超时 | u16 | int32（按描述符宽度读写） |
| `0x04` 喂狗 | 不喂（F19） | **喂**（`cb_torque_like()` 清 `controller.timeout`） |

---

## 5. 新增一个厂商（以 moteus 为例）

1. **公共头**：`jsdk_devfamily_t` 追加 `JSDK_DEVFAMILY_MOTEUS = 3`。
2. **候选路径表**：在 `src/core/jsdk_profile.c` 的 `g_profiles[]` 追加一行
   （每个语义键最多 `JSDK_PROFILE_MAX_CANDIDATES=3` 个候选名）；
   如果该厂商的词汇里有 `TAU_MAX` 就填上，没有就让 `CURLIM` 派生。
3. **能力位**：按它的真实语义选 `JSDK_PF_*`；若需要新的行为（例如标定不是
   function 而是命令帧），再加一个新能力位并在 `jsdk_ops.c` 按位分支。
4. **传输层**：若它连帧格式都不同（moteus 是 11-bit register 协议），在
   `src/proto_*` 增加一个 L2 实现，由 `jsdk_context_config_t` 选择 —— 这一层
   与 profile 解耦，profile 仍然只负责“语义 → 端点名”。
5. **测试**：照 `tests/data/endpoints_mit.json` 再放一份夹具，
   在 `test_ops.c` 的 `test_mit_profile()` 旁边加一个同形状的用例。

---

## 6. 注意事项

- **端点 ID 仍然不内置**：profile 只存路径名，ID 一律运行时从描述符解析。
- **数值仍然必须现场读**：`gear_ratio` / `torque_constant` / 量程都不写死；
  `tau_max` 派生也必须三项都真的读到，否则 `JSDK_ERR_NOT_FOUND`。
- **MIT 固件的 `config.kt` 默认是 1.0（占位值）**：拓扑能跑，但物理力矩数值不可信，
  上机前要标定或写入真实值。
- **MIT 的 CAN 超时不锁存**：超时只 `zero_commands()`，轴仍在闭环，但 MIT 响应的
  err nibble 会报 `9 = CAN_TIMEOUT` → SDK 会判成 fault。把 `config.can_timeout`
  设成 ≥ 2~4 倍控制周期，或保持 0（关闭）。
