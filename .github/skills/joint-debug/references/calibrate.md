# 标定 / 回零 —— "过不去"

> ⚠⚠ **会驱动电机**。全标定会转**十几圈电气角**，真机实测 **~30 s**。
> 调用 `jsdk_calibrate` / `jsdk_home` 必须显式 `confirm=true`，
> 且运行前确认：**关节固定牢、周围无人无障碍、电源/急停随手可及**。

## 第 0 步：先查两件事（比什么都重要）

### ① 有没有**锁存故障**挡着？

⚠ **有故障时，写 `requested_state` 只会被消耗掉、不会启动** ——
写被接受、值也对，但状态就是**不动**。

⇒ 必须**先告诉用户这一点**，否则他会一直重试。

```
jsdk_get_state(node=...)          # 看 error_raw 是否非零
jsdk_get_fault_detail(node=...)   # 看是哪一类
```
有故障 ⇒ `jsdk_fault_reset(confirm=true)` 清掉再标定。

### ② 轴在不在 **IDLE**（未使能）？

标定要求轴**未使能**；使能中调用会与闭环控制打架。

```
jsdk_get_state(node=...)   # current_state_name 应为 idle，resting=true
```

## 标定的四条约束（SDK 已内建，你要知道为什么）

`jsdk_calibrate` **不重新实现轮询** —— 那是 SDK 内部 `request_state_and_wait()`
的职责，它已满足以下全部四条。**在 MCP 层重写一遍 = 造出第二套实现。**

| 约束 | 为什么 |
|---|---|
| **限速轮询**：首段 20 ms，确认开始后转 200 ms | 全速刷会把 115200 的 slcan 挤爆（上限约 380 次交换/秒），后 2/3 的读全超时 → 状态永远读不回来。而首段必须快，否则会**整个错过短的瞬时状态** → 误报 "never left idle" |
| **判据 = "离开静息态"**，不是"等于请求值" | 全标定运行期间报的是**子状态**，**从不等于 3** |
| **空闲期间只收不发** | 往正在被标定的轴送控制帧是危险动作 |
| **超时来自时钟**，不是迭代次数 | 真机上循环会因为"读不到数据立即返回"而瞬间跑完（提示写着 120 秒、墙钟只过了 4 秒） |

⚠ **不要传小的 `state_timeout_ms`**：它的量级是**分钟**，不是毫秒。
默认 `0` = SDK 内置（标定 120 s / 回零 5 s）。
（曾把 3000 ms 的传输超时误当预算传进去 → 3 秒就报超时，**看着像设备不动了**。）

## 真实的子状态轨迹

固件 `axis.cpp:487-493` 的全标定 task_chain：

```
MOTOR_CALIBRATION(4) → MOTOR_DEADTIME_CALIBRATION(16)
→ [ENCODER_INDEX_SEARCH(6)] → ENCODER_OFFSET_CALIBRATION(7) → IDLE(1)
```

回零则**确实**停在 `HOMING(11)`，跑完回 `CLOSED_LOOP_CONTROL(8)`。

⚠ **扩展枚举**（别记反，这是 SDK 曾出错的地方）：

| 值 | 名称 |
|---|---|
| 6 | `ENCODER_INDEX_SEARCH` |
| 7 | `ENCODER_OFFSET_CALIBRATION` |
| 14 | `INERTIA_CALIBRATION` |
| 15 | `ENCODER_LINEARIZATION` |
| 16 | `MOTOR_DEADTIME_CALIBRATION` |

## 三样收尾证据（缺一样就会"谎称成功"）

`jsdk_calibrate` 返回里必须**三样都看**：

```jsonc
{
  "ok": true,
  "elapsed_s": 30.9,
  "state":  { "current_state_name": "idle", "resting": true },   // ①
  "fault":  { "subsystems": { ... } },                          // ②
  "pre_calibrated": { "…motor…": false, "…encoder…": false },    // ③
  "pre_calibrated_both": false
}
```

### ① `state` —— 是否**回到静息态**

⚠ **"跑完了"≠"轴停了"**。历史上 SDK 曾漏判 `MOTOR_DEADTIME_CALIBRATION(16)`，
于是**在电机仍在转的时候就返回 OK**：
修复前 `elapsed = 13.4 s` 且返回时 `current_state == 16`；
修复后 `elapsed = 30.9 s` 且返回时 `IDLE`。

⇒ 如果返回里 `state.resting == false`，**不要立即下发其他指令**。

### ② `fault` —— **跑完 ≠ 成功**

失败路径正是"**进子态 → 出错 → 回静息**"。所以结束时**必须**再查一次错误明细，
有故障位就报失败。

### ③ `pre_calibrated` —— **跑完 ≠ 生效**

⚠ 这是**正常行为，不是缺陷**，必须跟用户说清楚，否则他会一直重试：

> `pre_calibrated`（**电机与编码器两个都要**）**不会**因为跑完标定序列自动变 true。
> 必须人工写入 + `save` 到 Flash 才持久。

要让标定生效，依次执行：

```
jsdk_write_param(node, "axis0.motor.config.pre_calibrated", "1")
jsdk_write_param(node, "axis0.encoder.config.pre_calibrated", "1")
jsdk_save / invoke("save")        # 写 Flash
```

⚠ **两个都要写**；只写一个仍是 `false`。

## 一键验证

```
python -m jsdk_joint_mcp.hw_calibrate_probe --channel COM4 --node 1 --yes
```

它会打印标定前/后状态、三样收尾证据，并给出判读。
**不加 `--yes` 会被拒绝**（不动电机）。

## 常见失败信息怎么读

| 信息 | 含义 | 下一步 |
|---|---|---|
| `device never left idle after requesting state 3` | 状态机**拒绝启动** | **最常见是锁存故障** → `jsdk_fault_reset` |
| `state 3 did not finish within N ms` | 开始了但没跑完 | 看是否 N 太小；查 `fault` |
| `requires the axis to be disabled first` | 轴在使能中 | 先 disable，等回到 IDLE |
| `ran but the device reports a fault: …` | 跑完但设备有故障 | 看分项，查根因（编码器/温度/机械） |

⚠ 编码器类故障（如 `CPR_POLEPAIRS_MISMATCH`）通常是**设备侧**问题：
转子能否自由转动、`calibration_current`、编码器方向 —— **不是 SDK 的问题**。
