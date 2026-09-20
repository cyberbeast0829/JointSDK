# 单位与 kp/kd 公式速查

> 这份文档只解决一件事：**同一个名字的物理量，在这套协议里可能有两个不同的坐标系**。
> 所有结论都对着固件源码逐条核实过（基线 `ODrive` @ `4ff46135`，
> 行号见 `FIRMWARE_ISSUES.zh-CN.md` 与 `PROTOCOL_NOTES.zh-CN.md` §4.7）。

---

## 1. 为什么会"有两套坐标系"

驱动器内部按**电机端**（turns / turns·s⁻¹）做 MIT 控制点的计算
（`mit_control_cmd()` 把输出端 rad 换算成电机端 turns 再交给控制器），
而**上位机的机器人模型在输出端**（关节角度 rad、力矩 N·m）。

于是协议作者在**不同帧上**分别做了这两套换算，而且**没有统一**：

- MIT 命令/响应：**输出端** rad / rad·s⁻¹ / N·m
- `TORQUE_CONTROL(0x03)`：**电机端** N·m
- `QUERY_POS_VEL(0x41)`、心跳：**电机端** turns
- `CURRENT_CONTROL(0x04)`：**电机端** A

**SDK 不会帮你猜**：它把这套映射逐帧固化在 `src/core/jsdk_units.c` 与各模式的编码
路径里，并对客户只暴露**输出端**语义（`jsdk_joint_set_target_*_rad/Nm`）；
想拿协议原始量就用 `jsdk_joint_set_target_*()`（raw 入口，见 §4）。

---

## 2. 逐帧单位表（已对固件核实）

| 帧 | 位置 | 速度 | 力矩 / 电流 | 固件换算 |
|---|---|---|---|---|
| `MIT_CONTROL (0x00)` | **输出端** rad | **输出端** rad/s | 力矩 **输出端** N·m | `× gear/2π`、`÷ gear`（`mit_control_cmd()`） |
| `MIT` 响应（含 0x40 与所有控制帧的应答） | **输出端** rad | **输出端** rad/s | 电流 **电机端** A | `× 2π/gear`（`send_mit_response()`） |
| `POS_CONTROL (0x01)` | **输出端** 度 | **输出端** RPM | 电流限制 **电机端** A | 见 `cmd_pos_control()` |
| `VEL_CONTROL (0x02)` | — | **输出端** RPM | 电流限制 **电机端** A | 见 `cmd_vel_control()` |
| `TORQUE_CONTROL (0x03)` | — | — | 力矩 **电机端** N·m ⚠ | **不换算**（`input_torque_ = 原值`） |
| `CURRENT_CONTROL (0x04)` | — | — | 电流 **电机端** A（内部 `× torque_constant`） | 见 `cmd_current_control()` |
| `QUERY_POS_VEL (0x41)` | **电机端** turns | **电机端** turns/s | — | 直接透传估算器值 |
| 心跳 `0x48` | **电机端** turns | **电机端** turns/s | 电流 **电机端** A | Classic: ×100 定点；FD: ×10000 |

> ⚠ `TORQUE_CONTROL(0x03)` 是**唯一**端别与 MIT 相反的力矩帧。同一数值 `10`
> 在 MIT 下是"输出端 10 N·m"，在 `0x03` 下是"电机端 10 N·m" —— 对 gear=16.5 的设备，
> 实际输出差 **16.5 倍**。详见 `FIRMWARE_ISSUES.zh-CN.md` 的 F14。

---

## 3. SDK 面向客户的单位（推荐用这些）

| API | 入参单位 | 说明 |
|---|---|---|
| `jsdk_joint_set_target_position_rad(j, rad)` | 输出端 rad | CSP/POS：内部换算成度 |
| `jsdk_joint_set_target_velocity_rad_s(j, r)` | 输出端 rad/s | CSV/VEL：内部换算成 RPM |
| `jsdk_joint_set_target_torque_Nm(j, Nm)` | **输出端** N·m | CST：内部 `÷ gear_ratio` 后发 `0x03` |
| `jsdk_joint_set_mit(j, pos, vel, kp, kd, tau)` | 输出端 rad / rad·s⁻¹ / — / — / 输出端 N·m | `kp/kd` 是**线上值**（见 §5） |
| `jsdk_joint_set_mit_stiffness(j, pos, vel, K, D, tau)` | 输出端 rad、**N·m/rad**、**N·m·s/rad** | 内部 `kp = K × 2π / gear` |

反馈结构体 `jsdk_joint_feedback_t`：

| 字段 | 单位 | 来源 |
|---|---|---|
| `pos` / `vel` | 输出端 rad、rad/s | MIT 响应（已换算） |
| `current_A` | **电机端** A | MIT 响应的 Iq |
| `torque_Nm` | **输出端** N·m（**估算值**） | `current_A × torque_constant × gear_ratio` |
| `t_motor_C` / `t_fet_C` / `vbus_V` / `ibus_A` | °C / V / A | 心跳（`temp = raw − 50`） |
| `age_ms` | ms | 距上次有效反馈 |

> `torque_Nm` 是**估算**：它依赖设备里标定的 `torque_constant`。
> 要精确定力矩，请读设备端参数或用外部力矩传感器。

---

## 4. Raw 入口（协议原始量，SDK 不换算）

`jsdk_joint_set_target_position/velocity/torque(raw)` 的语义**逐模式不同**，
单位一律是"协议原始定点值 ÷ 1000"：

| 模式 | raw 语义 |
|---|---|
| `CSP/POS` | `raw / 1000` = 目标位置【**度**】（输出端） |
| `CSV/VEL` | `raw / 1000` = 目标速度【**RPM**】（输出端） |
| `CST` | `raw / 1000` = 目标力矩【**N·m**】（输出端） |
| `CURRENT` | `raw / 1000` = 目标电流【**A**】（**电机端**） |
| `MIT` | 不适用（请用 `jsdk_joint_set_mit`） |

也就是说：**raw 入口仍然是"输出端"语义**（除 CURRENT），只是去掉了物理量换算、
只做定点化。真正"协议原始量"的逃生通道是直接拼帧（`cb_ctrl_*` 系列，非公开 ABI）。

---

## 5. kp / kd：最容易误解的一对参数

### 事实

固件把 `kp` **原样**交给 MIT 控制器，而控制器的位置误差是**电机端 turns**：

```cpp
// can_cyberbeast.cpp: pack_mit_command()
int kp_int = float_to_uint(kp, 0, mit_max_kp, 12);   // 原样编码，不换算
// mit_control_cmd()
float motor_pos = pos * gear_ratio / (2.0f * M_PI);   // 输出端 rad → 电机端 turns
axis.controller_.input_mit_kp_ = kp;                  // 原样使用
```

于是**输出端等效刚度**不是 `kp`：

$$
\text{stiffness}_{\text{out}}\ [\mathrm{N\cdot m/rad}]
= kp \times \frac{\text{gear\_ratio}}{2\pi}
$$

### 数字例子（`gear_ratio = 16.5`）

| 你给的 `kp` | 实际输出端刚度 | 说明 |
|---|---|---|
| 10 | 26.3 N·m/rad | `16.5 / (2π) = 2.626` |
| 100 | 262.6 N·m/rad | 手册上限 `mit_max_kp = 500` → 等效 1313 N·m/rad |
| 500 | 1313 N·m/rad | 上限值 |

> 协议文档（`docs/cyberbeast-protocol.md:353`）把 `KP` 的单位写成 **N·m/rad**，
> 与实现相差 **2.63 倍** —— 这一条已经作为 **F1** 提给固件侧
> （见 `FIRMWARE_ISSUES.zh-CN.md`）。SDK 的处理是**两端都暴露**：
> `set_mit()` 给"线上 kp"，`set_mit_stiffness()` 给"真实刚度"。

### 怎么选

- 想**和厂商工具/别人给的调参值一致** → `jsdk_joint_set_mit()`（线上值，逐字节一致）
- 想**按物理意义整定**（"我要 30 N·m/rad 的刚度"）→ `jsdk_joint_set_mit_stiffness()`
- `kd` 同理：`实际阻尼 = kd × gear_ratio / (2π)`

---

## 6. 常见错误与症状

| 错误 | 症状 | 怎么发现 |
|---|---|---|
| 把 `QUERY_POS_VEL(0x41)` 的位置当输出端 rad | 位置差 2.6 倍（`2π/gear` ≈ 0.381） | 与 MIT 响应的 `fb.pos` 对不上 |
| 把 `kp` 当"真实刚度"填 | 增益**偏小 2.63 倍**，串级回路下可能振荡或"软绵绵" | 用 `set_mit_stiffness()` 复现同一个数再对比 |
| 切到 `TORQUE_CONTROL` 时沿用 MIT 的力矩值 | 输出差 `gear_ratio` 倍（可能瞬间飞车） | 看 `FIRMWARE_ISSUES` F14；SDK 的物理量入口已代为换算 |
| 拿心跳位置做闭环反馈 | 精度只有 1/100 turn（Classic）或 1/10000（FD） | 心跳是"快照"，精确值走 MIT 响应 / `0x41` |
| 认为 `torque_Nm` 是实测值 | 标定不准时误差可达数十 % | 它是 `Iq × Kt × gear` 的估算 |

---

## 7. 自查三件套

1. **打印一次量程**：`jsdk_joint_read_config_snapshot()` 的 `gear_ratio` / `mit_max_*`
   是否与你设备铭牌一致（不一致说明描述符或设备不对，先别动电机）。
2. **量纲自检**：给 `set_mit_stiffness(pos, 0, K, 0, 0)` 与
   `set_mit(pos, 0, K×2π/gear, 0, 0)`，两者的位置响应应当**一致**
   （不一致就是这里讲的换算被改坏了）。
3. **端别自检**：静止时读 `fb.pos` 与 `0x41` 的位置，比值应约等于
   `2π / gear_ratio`（`16.5` 时 ≈ `0.381`）。

---

## 8. 相关文档

- `PROTOCOL_NOTES.zh-CN.md` §4.7：逐帧单位对照（协议视角，含固件函数名）
- `FIRMWARE_ISSUES.zh-CN.md`：F1（kp/kd 量纲）、F14（力矩端别）、F15（位置端别）
- `PORTING.zh-CN.md`：自己实现 HAL 时时间与单位的关系
- `include/joint_sdk/joint_sdk.h`：每个 setter 的头文件注释里都写了单位
