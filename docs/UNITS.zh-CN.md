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
| `POS_CONTROL (0x01)` | **输出端** 度 | **输出端** RPM | **力矩上限**（线上以**电机端** A 表达）⚠ | 见下方 §2.1 与 `cmd_pos_control()` |
| `VEL_CONTROL (0x02)` | — | **输出端** RPM | **力矩上限**（线上以**电机端** A 表达）⚠ | 见下方 §2.1 与 `cmd_vel_control()` |
| `TORQUE_CONTROL (0x03)` | — | — | 力矩 **电机端** N·m ⚠ | **不换算**（`input_torque_ = 原值`） |
| `CURRENT_CONTROL (0x04)` | — | — | 电流 **电机端** A（内部 `× torque_constant`） | 见 `cmd_current_control()` |
| `QUERY_POS_VEL (0x41)` | **电机端** turns | **电机端** turns/s | — | 直接透传估算器值 |
| 心跳 `0x48` | **电机端** turns | **电机端** turns/s | 电流 **电机端** A | Classic: ×100 定点；FD: ×10000 |

> ⚠ `TORQUE_CONTROL(0x03)` 是**唯一**端别与 MIT 相反的力矩帧。同一数值 `10`
> 在 MIT 下是"输出端 10 N·m"，在 `0x03` 下是"电机端 10 N·m" —— 对 gear=16.5 的设备，
> 实际输出差 **16.5 倍**。详见 `FIRMWARE_ISSUES.zh-CN.md` 的 F14。

---

### 2.1 ⚠ CSP/CSV 的那个字段是「力矩上限」，不是「过流告警门限」

设备里有两个长得很像、**作用完全不同**的上限（真机核实）：

| 设备参数 | 作用 | 越界后果 |
|---|---|---|
| `motor.config.current_lim` | **过流告警门限**：`Itrip = current_lim + current_lim_margin` | **`disarm_with_error(ERROR_CURRENT_LIMIT_VIOLATION)`** —— 报错并失能 |
| `motor.config.torque_lim` | **正常工作力矩上限**：`max_torque = clamp(电流限值 × torque_constant, 0, torque_lim)` | 静默钳位，**不报错** |

CSP/CSV 帧的第 5..8 字节在线上**以电机端 A 为单位**，但固件的用法是：

```cpp
// cmd_pos_control() / cmd_vel_control()
axis.motor_.config_.torque_lim = cur_limit_a * axis.motor_.config_.torque_constant;  // A → N·m
```

即它设的是 **`torque_lim`（力矩上限）**。固件源码注释明确写着
「**不修改 current_lim 避免误触发告警**」，所以这两个概念**不能混为一谈**：
客户的 `current_lim` 本来就应当**大于** `torque_lim / torque_constant`。

**SDK 侧对应关系：**

| SDK API | 参数单位 | 写入的设备参数 |
|---|---|---|
| `jsdk_joint_set_torque_limit_Nm(j, vel_lim, tau_Nm)` ← **推荐** | **电机端 N·m** | `torque_lim` |
| `jsdk_joint_set_limits(j, vel_lim, cur_lim_A)`（deprecated） | 电机端 A（线上原值） | `torque_lim` |

**⚠ 静默陷阱**：固件对每帧 CSP/CSV 都**无条件覆盖** `torque_lim`。若这个值发成
`0`，`torque_lim` 就变 0 ⇒ 电流环被钳到 0 ⇒ **电机不出力，但 `is_enabled()`
仍为 1、无 fault、`tx_rejected == 0`**，现场表现为“使能成功却完全不转”。
为避免这个坑，`configure()` 会用设备读回的
`current_lim × torque_constant` 作为**默认上限**（不调上面两个 API 也能工作）。

---

## 3. SDK 面向客户的单位（推荐用这些）

| API | 入参单位 | 说明 |
|---|---|---|
| `jsdk_joint_set_target_position_rad(j, rad)` | 输出端 rad | CSP/POS：内部换算成度 |
| `jsdk_joint_set_target_velocity_rad_s(j, r)` | 输出端 rad/s | CSV/VEL：内部换算成 RPM |
| `jsdk_joint_set_target_torque_Nm(j, Nm)` | **输出端** N·m | CST：内部 `÷ gear_ratio` 后发 `0x03` |
| `jsdk_joint_set_mit(j, pos, vel, kp, kd, tau)` | 输出端 rad / rad·s⁻¹ / — / — / 输出端 N·m | `kp/kd` 是**线上值**（见 §6） |
| `jsdk_joint_set_mit_stiffness(j, pos, vel, K, D, tau)` | 输出端 rad、**N·m/rad**、**N·m·s/rad** | 与 `kp/kd` 数值相同（不换算） |

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

> ⚠⚠ **端点值的单位（真机实测核实，最容易踩）**：`axis0.encoder.pos_estimate` /
> `vel_estimate` 这些**端点值就是设备内存里的原样值**（**电机端 turns / turns·s⁻¹**），
> SDK **不做**任何换算。而 `feedback().pos/vel` 是**输出端 rad / rad·s⁻¹**。
> 两者直接相比会得到“同一个位置两个值、差一个 `2π/gear`”的假象（实测：
> 端点 1.8475 turns ↔ `feedback().pos` 1.4979 rad ↔ ×2π/7.75 ✓）。
> 同理：**心跳 `0x48` 的 pos 是电机端 turns（Classic ×100 定点，1 LSB = 0.01 turn）** ——
> 它在刷新且正确，但精度只有 0.0081 rad（本关节）⇒ 不能当控制反馈。

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

## 4.1 function 端点调用（Fibre 方法）

描述符里除了属性（可读写的值），还有一类 **function**（`"type":"function"`）——
它没有 `access` 字段、也没有线宽，语义是"写它 = 执行"。
实测 v8 描述符里有 **30 个**，例如：

| function | 后果 |
|---|---|
| `save_configuration` | 写 Flash |
| `clear_errors` | 清错误位 |
| `reboot` / `enter_dfu_mode` / `enter_bootloader_mode` | **设备重启/断开** |
| `erase_configuration` | **擦除全部配置** |
| `oscilloscope.get_val` | 采样（有入参/出参） |
| `axis0.controller.move_incremental` | **让电机运动** |
| `axis0.controller.start_anticogging_calibration` | 长时间标定，电机会动 |

### 调用序列

```
1. 写 inputs   —— 每个入参在描述符里都有自己的端点 ID，
                  路径 = "<function_path>.<input_name>"
2. 写 function —— 无值（DataLen = 0），**这一步才触发执行**
3. 读 outputs  —— 同样各有自己的端点 ID
```

以 `axis0.controller.move_incremental` 为例（描述符实测）：

| 端点 | ID | 类型 | 权限 |
|---|---|---|---|
| `axis0.controller.move_incremental` | 349 | function | — |
| ├ `.displacement` | 350 | float | rw ← **input** |
| └ `.from_input_pos` | 351 | bool | rw ← **input** |

**SDK 用法：**

```c
jsdk_value_t in[2] = {
    { .type = JSDK_EP_F32,  .v.f32     = 0.5f },   /* → .displacement */
    { .type = JSDK_EP_BOOL, .v.boolean = 1     },   /* → .from_input_pos */
};
jsdk_joint_ep_invoke(j, "axis0.controller.move_incremental", in, 2, NULL, 0, NULL);
```

有出参的例子 `oscilloscope.get_val`（input `index` u32 → output `val` float）：

```c
jsdk_value_t in[1]  = { { .type = JSDK_EP_U32, .v.u32 = 3 } };
jsdk_value_t out[1];
unsigned got = 0;
jsdk_joint_ep_invoke(j, "oscilloscope.get_val", in, 1, out, 1, &got);
/* got == 1, out[0].v.f32 = 采样值 */
```

CLI：`jsdk-cli invoke --yes oscilloscope.get_val 3`

Python：`j.ep_invoke("oscilloscope.get_val", 3)` → `[1.25]`

> **CLI 的参数按描述符类型解析**（不是猜）：`oscilloscope.get_val` 的入参是
> `uint32`、`move_incremental.displacement` 是 `float` —— 两者在命令行上都是
> "一个数字"，CLI 先查描述符拿到真实类型再解析，超出值域即拒绝（**不静默截断**）。
> 真机实测教训：早期版本用"字面量像整数就当 i32"的猜法，
> `invoke oscilloscope.get_val 0` 必然以 `descriptor=uint32 given=int32` 失败。

### 几点须知

* **入参顺序必须与描述符 `inputs` 一致**；类型也必须一致（不符 → `JSDK_ERR_PROTOCOL`）。
* **参数校验在发帧之前完成** —— 数量/类型/路径任一不对就一帧都不发，
  不会把设备的 input 改成"半新半旧"。
* **function 端点不能用 `jsdk_joint_param_set()`** 调用（它会以 "read-only" 拒绝，
  因为 function 没有 access 字段）。`ep_invoke()` 是**唯一**通路。
* 有专用帧的功能**并存**，两条路等价：

  | function | 专用帧 |
  |---|---|
  | `save_configuration` | `CONFIG_SAVE(0x22)` / CLI `save` |
  | `clear_errors` | `CLEAR_ERRORS(0x65)` / CLI `fault-reset` |
  | `reboot` | `RESET_DEVICE(0x64)` / CLI `reset` |
  | `set_current_pos_zero` | `SET_ZERO(0x61)` / CLI `set-zero` |

* ⚠ **没有白名单**（这是刻意的）。`erase_configuration` / `reboot` / `enter_dfu_mode`
  会立刻改变设备状态或使其失联；CLI 对 `invoke` 强制要 `--yes`。

---

## 5. `unit_scale_*`：什么时候**真的**需要它

### 先给结论

| 你打算怎么驱动电机 | 要不要 `unit_scale_*` |
|---|---|
| 用物理量 API（`set_target_position_rad()` / `_velocity_rad_s()` / `_torque_Nm()` / `set_mit*()`） | **不要**。线上量在模式映射之后**已经是物理量**，默认就是恒等映射 |
| 用 raw 入口（`set_target_position(raw)`）传"自己约定的**编码器计数**" | **要** `jsdk_unit_scale_calc()` 拿到 `counts → rad` 的比例 |
| 从 EtherCAT 版迁移过来（那里 `unit_scale_default()` 是 counts→rad 的真实比例） | 注意**同名不同义**，见下表 |

### 与 EtherCAT 版的关键差异

| | CAN / CyberBeast（本库） | EtherCAT 版 |
|---|---|---|
| 线上量 | 模式映射之后**已经是物理量**（MIT = 输出端 rad / rad·s⁻¹ / N·m） | 原始**编码器计数** |
| `jsdk_unit_scale_default()` | **恒等映射**（三个比例都是 `1.0`） | counts → rad 的真实比例 |
| 什么时候需要 `jsdk_unit_scale_calc()` | 只有你**按计数驱动**时，且换来的是给**你自己**用的比例 | 总是需要 |

> ⚠ **本库内部不会用 `scale` 做任何换算** —— 逐帧换算在 `src/core/jsdk_units.c` 里
> 已经按模式写死了（见 §2/§3）。`scale` 的实际作用只有两个：
> 1. **报给调用方**：`jsdk_joint_get_scale()` 读回这台关节当前的 scale；
> 2. **驱动一个可信度标志**：`valid` 控制 `JSDK_JF_SCALE_INVALID` 粘滞位
>    （`valid == 0` 置位，`!= 0` 清位）—— 语义是"这台电机的标定**可信吗**"，
>    不是"数据非法"。

### 两个构造函数

```c
/* 默认：恒等映射。rated_trq 只用来判断"调用方是否真的知道这台电机" */
void jsdk_unit_scale_default(jsdk_unit_scale_t *s, uint32_t rated_trq);
/*   rated_trq > 0 → valid = 1；否则 valid = 0。三个比例恒为 1.0 */

/* 按编码器计数换算（面向"按计数驱动"的场景） */
void jsdk_unit_scale_calc(jsdk_unit_scale_t *s,
                          uint32_t encoder_resolution,  /* CPR，正交后 */
                          uint32_t motor_rev, uint32_t shaft_rev,  /* gear = shaft/motor */
                          uint32_t rated_torque);
```

$$pos_{\text{counts}\to\text{rad}} = \frac{2\pi \times \text{motor\_rev}}{\text{encoder\_resolution} \times \text{shaft\_rev}}$$

```
vel_counts_to_rad_s = pos_counts_to_rad          （同一个位置/时间的比例）
trq_to_Nm           = rated_torque / 1000        （线力矩单位 = 0.1% 额定）
```

> 公式假设编码器装在**电机**侧（`CYT` 系列如此）。装在输出侧时把 `shaft_rev` 设成 1、
> `motor_rev` 设成 1，等价于"CPR 就是输出端 CPR"。

### `valid == 0` 的两种来路（别混）

| 来路 | 含义 |
|---|---|
| `jsdk_unit_scale_calc()` 任一入参为 0，或算出的比例不合理（`≤0` 或 `> 1e12`） | **算不出来**：三个比例**全部置 0**，绝不猜一个近似值 |
| `jsdk_unit_scale_default(s, 0)` | 调用方**没给额定力矩**，即"不知道这是哪台电机" |

两者都置位 `JSDK_JF_SCALE_INVALID`。若你手里有权威换算表（铭牌/出厂报告），
直接填好 `jsdk_unit_scale_t` 再 `jsdk_joint_set_scale()` 即可解除该位 —— 这条路径
是给"设备标定读不出来、但你知道正确值"的场景准备的（Python 侧即 `Joint.set_scale`）。

> `> 1e12` 这条上界是**防呆**：位数写错一位（例如 CPR 写成 1 而不是 4096 的倒数）
> 会得到一个天文数字比例，那比 `valid = 0` 危险得多 —— 它会被当成"有效"。
> 与其给一个错的换算，不如让你看见"无效"。

### 自查

1. 打印 `jsdk_joint_get_scale()`：物理量驱动下应当是 `1.0 / 1.0 / 1.0` 且 `valid = 1`。
2. 若你**没有**按计数驱动，却在代码里乘了 `pos_counts_to_rad` → 位置会差几千倍，
   这是最容易复现的误用症状。
3. `jsdk_joint_is_fault()` 里看到 `SCALE_INVALID` 粘滞位置起时，先确认
   `configure()` 是否读到了可信的标定（见 §7 的第 1 条）。

---

## 6. kp / kd：最容易误解的一对参数

### 事实

固件把 `kp` **原样**交给 MIT 控制器，而控制器的位置误差是**电机端 turns**：

```cpp
// can_cyberbeast.cpp: pack_mit_command()
int kp_int = float_to_uint(kp, 0, mit_max_kp, 12);   // 原样编码，不换算
// mit_control_cmd()
float motor_pos = pos * gear_ratio / (2.0f * M_PI);   // 输出端 rad → 电机端 turns
axis.controller_.input_mit_kp_ = kp;                  // 原样使用
```

于是**输出端等效刚度就等于 `kp`**（不做齿比换算）：

$$
\text{stiffness}_{\text{out}}\ [\mathrm{N\cdot m/rad}] = kp
$$

### 数字例子

| 你给的 `kp` | 实际输出端刚度 |
|---|---|
| 10 | 10 N·m/rad |
| 100 | 100 N·m/rad |
| 500 | 500 N·m/rad（即 `mit_max_kp`） |

> ✅ **2026-09-29 真机实测定案**：本节先后写过 $kp\times\text{gear}/(2\pi)$ 与 $kp/\text{gear}$，
> **两者都被实测推翻**。方法：静态平衡 $e=-\tau_{ff}/K_{out}$，用 MIT 的 `tau_ff` 制造**已知**的
> 输出端力矩，用 MIT 应答的 `pos`（固件已 `×2π/g` 换成输出端 rad）测偏移。
> 三组独立测量 $K/kp$ = **1.019 / 1.000 / 1.008** ⇒ $K=kp$。
> 见 `tools/f1_kp_ratio.py` 与 `src/core/jsdk_units.c`。

> 协议文档（`docs/cyberbeast-protocol.md`）把 `KP` 的单位写成 **N·m/rad**，
> **这与实际行为一致**（我们曾以为不一致，已实测纠正）。
> SDK 仍**两端都暴露**：`set_mit()` 给"线上 kp"，`set_mit_stiffness()` 给"真实刚度" ——
> 两者现在数值相同，保留双入口是为了让客户代码的语义清晰。

### 怎么选

- 想**和厂商工具/别人给的调参值一致** → `jsdk_joint_set_mit()`（线上值，逐字节一致）
- 想**按物理意义整定**（"我要 30 N·m/rad 的刚度"）→ `jsdk_joint_set_mit_stiffness()`
- `kd` 同理：`实际阻尼 = kd / gear_ratio`

---

## 7. 常见错误与症状

| 错误 | 症状 | 怎么发现 |
|---|---|---|
| 把 `QUERY_POS_VEL(0x41)` 的位置当输出端 rad | 位置差 2.6 倍（`2π/gear` ≈ 0.381） | 与 MIT 响应的 `fb.pos` 对不上 |
| 把 `kp` 当"真实刚度"填 | 增益**偏小 2.63 倍**，串级回路下可能振荡或"软绵绵" | 用 `set_mit_stiffness()` 复现同一个数再对比 |
| 切到 `TORQUE_CONTROL` 时沿用 MIT 的力矩值 | 输出差 `gear_ratio` 倍（可能瞬间飞车） | 看 `FIRMWARE_ISSUES` F14；SDK 的物理量入口已代为换算 |
| 拿心跳位置做闭环反馈 | 精度只有 1/100 turn（Classic）或 1/10000（FD） | 心跳是"快照"，精确值走 MIT 响应 / `0x41` |
| 认为 `torque_Nm` 是实测值 | 标定不准时误差可达数十 % | 它是 `Iq × Kt × gear` 的估算 |

---

## 8. 自查三件套

1. **打印一次量程**：`jsdk_joint_read_config_snapshot()` 的 `gear_ratio` / `mit_max_*`
   是否与你设备铭牌一致（不一致说明描述符或设备不对，先别动电机）。
2. **量纲自检**：给 `set_mit_stiffness(pos, 0, K, 0, 0)` 与
   `set_mit(pos, 0, K×2π/gear, 0, 0)`，两者的位置响应应当**一致**
   （不一致就是这里讲的换算被改坏了）。
3. **端别自检**：静止时读 `fb.pos` 与 `0x41` 的位置，比值应约等于
   `2π / gear_ratio`（`16.5` 时 ≈ `0.381`）。

---

## 9. 相关文档

- `PROTOCOL_NOTES.zh-CN.md` §4.7：逐帧单位对照（协议视角，含固件函数名）
- `FIRMWARE_ISSUES.zh-CN.md`：F1（kp/kd 量纲）、F14（力矩端别）、F15（位置端别）
- `PORTING.zh-CN.md`：自己实现 HAL 时时间与单位的关系
- `include/joint_sdk/joint_sdk.h`：每个 setter 的头文件注释里都写了单位
