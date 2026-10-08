---
name: joint-bringup
description: "Bring up, commission and debug CyberBeast joint modules over CAN using the jsdk-joint MCP tools. Use when: bringing a joint up from power-on, calibrating or homing it, or when a joint won't move, reports an error or fault, parameters won't write or won't take effect, calibration or homing fails, or you need to read joint state / raw error bits / telemetry / endpoint descriptors. 适用于：把关节从首次上电调通、标定/回零，以及关节不动、报错、参数写不进去、标定或回零过不去、要读状态/原始错误位/遥测/端点表。"
---

# 关节模组调测（CyberBeast / JointSDK）

## 目标

**把关节调好**，不是"用 SDK"。SDK（含 `jsdk-joint` MCP 工具）只是你手里的一件工具；
真正的对象是**一台会动、会发热、会锁存故障、会静默丢帧的物理设备**。

⚠ 这是**运动控制硬件**。一个错误的写入可能让关节**满力矩飞车**。
本 skill 里所有"先读后写""必须读回""不许猜"的规定，都是为了这一条。

## 何时用

- **把一台关节从首次上电调通**（建链 → 读状态 → 写参数 → 标定）
- 关节**不动** / 报错 / 行为异常
- **参数写不进去**，或写成功但"没生效"
- **标定 / 回零过不去**
- 要读**状态、原始错误位、故障分项、遥测、端点表**
- 判断"这个现象是设备的问题，还是我们的问题"

## 三条铁律

违反任何一条，你的结论都不可信。

1. **写后必读回，且分字段看**：`requested`（我请求的）/ `value`（设备读回的）/
   `verified`（是否一致）。**绝不能把 requested 当结果**。
2. **先读后写**：任何写之前，先确认链路、读状态。**没有任何一步可以跳过**。
3. **分清"看到的"与"推断的"**：报结论时**分开写**（见下方汇报格式）。

## 标准三查（几乎每个问题都从这里开始）

### 第 1 查：建链 —— `jsdk_connect`

```
jsdk_connect(iface="slcan", channel="COM4", node=1)
```

- ⚠ **不传 `classic`** → 按 Classic 起步。协议**没有运行时协商**，
  "心跳收得到、请求没人应" = 猜错了 Classic/FD。
- 报错怎么读（`rx_frames` 是关键判据）：

  | 报错 | 含义 | 下一步 |
  |---|---|---|
  | `0/0 bytes` + **0 帧** | 链路/端口/格式 | 查端口、权限、Classic/FD |
  | `0/0 bytes` + **N 帧** | **通道通、我们的请求丢了** | 正常现象（首帧丢失），重试即可 |
  | 超时且心跳正常 | **Classic/FD 猜错** | 见 `./references/connect.md` |

### 第 2 查：状态 —— `jsdk_get_state`

```
jsdk_get_state(node=1, iface="slcan", channel="COM4")
```

- 读的是**原始错误位图**（`error_raw` + 各子系统 `*_hex`），**不是 `err_name`**。
  ⚠ `err_name = "CAN_TIMEOUT"` 把**看门狗超时**和**急停**混成同一个名字 ——
  看到它就去查波特率/线缆，会白费几个小时。
- 同时看 `current_state_name` 与 `resting`。

### 第 3 查：故障分项 —— `jsdk_get_fault_detail`

```
jsdk_get_fault_detail(node=1, ...)
```

拿到六类（motor / encoder / controller / sensorless / system / axis）的**分项位图**，
才能回答"到底是哪一类"。

## 症状 → 去哪

| 症状 | 先看 | 详见 |
|---|---|---|
| 连不上 / 收不到帧 / 心跳有但请求没人应 | 第 1 查的 `rx_frames` | `./references/connect.md` |
| 有错误位 / 报 CAN_TIMEOUT / 找不出哪一类 | `jsdk_get_fault_detail` | `./references/diagnose.md` |
| **参数写不进去** / 写成功但读回还是旧值 | 端点**类型宽度** | `./references/write-param.md` |
| 标定 / 回零过不去 | 锁存故障 + 是否在 IDLE | `./references/calibrate.md` |
| 读数"看着像数但是错的" | **字节序** | `./references/diagnose.md` |
| 数值是否合理（温度/误差/丢样） | `jsdk_get_telemetry` | `./references/diagnose.md` |
| 要写 MIT 帧 / 探针（涉及量程） | `jsdk_probe_ranges` | `./references/write-param.md` |

## 汇报格式

```text
【结论】 一句话
【我看到的】 原始读数 / 位图 / 帧计数（可核查，不要转述）
【我推断的】 基于上面事实的推理
【下一步】 一个具体的、可执行的检查
```

⚠ 不要把**推断**写成**事实**。
⚠ 不要断言"输出里应该有 X"而不先确认那个命令真的会打印 X。

## 参考

- `./references/connect.md` —— 建链、帧格式、首帧丢失
- `./references/write-param.md` —— 安全写参数、窄类型陷阱、量程
- `./references/calibrate.md` —— 标定 / 回零的四条约束与三样证据
- `./references/diagnose.md` —— 错误位、字节序、单位、遥测判读

本仓库根目录的 **`AGENTS.md`** 是完整的领域规则（物理事实 vs 必须现场读取的值、
单位陷阱、看门狗语义……），需要时一并读。**`CONTRIBUTING.md`** 是改 SDK 代码时的陷阱，
与本 skill 无关。
