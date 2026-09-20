# `jsdk-cli` 使用说明

> CyberBeast 关节 SDK 的 **PC 诊断命令行工具**（WP7）。
> 目标：客户在**没有自研上位机**的情况下也能完成扫描 / 监控 / 参数读写 / 排障，
> 并且所有输出都能被脚本和 CI 消费。

---

## 0. 一句话定位

| 想做的事 | 用哪个命令 |
|---|---|
| 总线接对了没？设备在哪？ | `scan` |
| 现在健康吗？为什么不动？ | `health`、`err`、`hb-dump` |
| 参数表长什么样？某个参数叫什么？ | `ep-list`、`ep-lookup`、`desc-info` |
| 标定量对不对？ | `dump-config` |
| 改参数 / 存 Flash / 改节点号 | `write`、`save`、`set-node-id`、`watchdog` |
| 让它转一下（**唯一会驱动电机**） | `mit --yes --hold N` |
| 采集一段数据 | `mon --csv out.csv` |
| 省掉每次上电的 41 KB 下载 | `desc-export` / `desc-import` |

**默认只读。** 凡会写设备或让电机动的子命令都要显式加 `--yes`。

---

## 1. 构建

CLI 需要堆模式（上下文与描述符 arena 都是分配的），因此：

```bash
cmake -S . -B build -G "MinGW Makefiles" -DJSDK_WERROR=ON -DJSDK_ENABLE_HEAP=ON
cmake --build build
# → build/jsdk-cli.exe  （或 Linux 上 build/jsdk-cli）
```

`JSDK_BUILD_CLI` 默认**跟随** `JSDK_ENABLE_HEAP`：堆关着时不会冒然打开 CLI，
免得默认 `configure` 就报错。显式 `-DJSDK_BUILD_CLI=ON` 而不给堆模式会直接
`FATAL_ERROR` 并说明原因（而不是给出一个"能编但一跑就崩"的目标）。

MCU 构建不需要 CLI，也就不会付这部分体积。

### Windows 控制台的中文显示

输出是 UTF-8。旧版 `cmd.exe` 默认用 GBK 代码页，会显示成乱码。两种解法：

```cmd
chcp 65001
```
或用 `--json`（机器可读，脚本里本来也不该解析中文）。

---

## 2. 全局选项

| 选项 | 说明 |
|---|---|
| `--if socketcan｜pcan｜slcan｜virtual` | 传输后端。默认：Windows/macOS → `pcan`，其它 → `socketcan` |
| `--channel NAME` | `can0` / `PCAN_USBBUS1` / `COM5` / `/dev/ttyACM0`；`virtual` 时为节点规格 |
| `--bitrate N` | CAN **仲裁段**波特率（默认 1000000，仅用于校验/初始化） |
| `--data-bitrate N` | CAN FD **数据段**波特率（默认 5000000；写 `0` 或加 `--classic` 用 Classic） |
| `--baud N` | **串口**波特率（**仅 slcan**，默认 115200）。与 `--bitrate` 不是同一个量 |
| `--classic` | 强制 Classic CAN |
| `--master-id N` | 主站源地址（默认 1；**禁止 0** —— 设备完全不回复） |
| `--node N` | 目标节点 ID（默认 1） |
| `--probe N` | `scan` 的主动探测上限（默认 16，`0` = 仅被动听心跳） |
| `--timeout MS` | 单次操作超时（默认 3000） |
| `--json` | 机器可读输出（字段名见 §5，Python 绑定与 CI 依赖它们） |
| `--rate-hz N` | `mon` 的采样率（默认 10） |
| `--duration S` | 运行时长，`0` = 直到 Ctrl-C |
| `--yes` | 确认执行写/动类命令 |
| `--hold S` | `mit` 的持续时间（1..60 秒，**必需**） |
| `--csv FILE` | `mon` 同时写 CSV |
| `--filter P` | `ep-list` 的过滤：**子串**匹配；以 `*` 结尾则按前缀 |
| `--pos --vel --kp --kd --tau --stiffness` | `mit` 的目标量 |
| `-v` / `-q` | 日志级别 |

选项可以出现在子命令**之前或之后**（`jsdk-cli scan --if virtual` 与
`jsdk-cli --if virtual scan` 等价）。

### 2.1 `virtual` 后端的节点规格

`--channel` 的格式与测试一致：`;` 分隔多个节点，每个节点 `下标:键=值,...`。
**起始数字是数组下标，不是 node_id**（node_id 用 `id=` 设，缺省为下标+1）。

```
--if virtual --channel "0:gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=10,fd"
--if virtual --channel "0:id=1,fd;1:id=2,gear=8,classic"
```

无值键：`fd` / `classic` / `arm` / `disarm` / `enabled` / `disabled`。
省略 `--channel` 时用内置默认单节点规格。

**虚拟设备自带描述符**（由它的端点表生成），所以 `scan`/`health`/`read`/
`ep-list` 等全部子命令**开箱即用** —— 没有硬件时可以用它自检命令与脚本。

---

## 3. 子命令

### 3.1 只读（不需要 `--yes`）

| 命令 | 说明 |
|---|---|
| `scan` | 节点发现：被动听心跳 200 ms + 主动 `QUERY_STATUS` 探测 1..`--probe`。**不下载描述符**（省 41 KB 流量） |
| `info` | `QUERY_DEVICE_INFO(0x46)`：hw / fw / serial |
| `health` | 健康快照：模式、轴状态、错误码、心跳标志、温度、母线、`age_ms`、链路统计 |
| `mon` | 周期监控；`--csv` 同时落文件 |
| `read <path>` | 按名读参数（类型随描述符） |
| `batch-read <path>...` | 批量读：FD 下打包成单帧，Classic 下自动逐条 |
| `dump-config` | 关键配置快照（`gear_ratio` / `mit_max_*` / `torque_constant` / `node_id` / `break_timeout_ms`…） |
| `err` | `QUERY_ERROR(0x45)` 六类 32-bit 错误字明细 |
| `hb-dump` | 最近心跳的**原始字节** + 解码对照。会**自己等**一个心跳（最多 2 s） |
| `desc-info` | 描述符元信息（长度 / CRC / 帧数 / 完整 / 端点数） |
| `ep-list` | 枚举端点（`--filter` 过滤） |
| `ep-lookup <path>` | 路径 → 端点 ID / 类型 / 权限 |
| `desc-export <file>` | 导出描述符缓存 |
| `desc-import <file>` | 导入描述符缓存（**不下载**，导入后直接可用） |

### 3.2 写（需要 `--yes`）

| 命令 | 说明 |
|---|---|
| `write <path> <value>` | 参数写。值按端点**真实类型/位宽**解析，超范围**直接拒绝**（不静默截断） |
| `save` | `CONFIG_SAVE(0x22)`，写后读回校验 |
| `set-node-id N` | 改节点地址（含冲突检查、验证新地址有应答，可持久化） |
| `watchdog MS` | 写 `can.config.break_timeout`。⚠ 固件把 `0` 解释为 **100 ms**，`0` ≠ 关闭 |
| `set-zero` | `SET_ZERO(0x61)`（当前位置设为零点，不落 Flash） |
| `reset` | `RESET_DEVICE(0x64)` |

> **为什么 `write` 也要 `--yes`？**
> `write axis0.motor.config.gear_ratio 8` 会让同一条 MIT 指令的实际输出差一倍 ——
> 足以让电机跳。与其维护"哪些参数危险"的清单，不如规则统一。

### 3.3 动作（需要 `--yes`）

| 命令 | 说明 |
|---|---|
| `calibrate` | 写 `requested_state = 3` 并等待状态跳转（电机会动） |
| `home` | 写 `requested_state = 11` 并等待 |
| `estop` | 广播 `ESTOP(0xC0)`，最高仲裁优先级。**不需要 `--yes`** —— 拒绝执行反而更危险 |
| `mit` | **唯一会驱动电机的命令**，见 §4 |

---

## 4. `mit` 的三道闸

`mit` 是本工具里唯一会驱动电机的东西，因此约束最紧：

1. **必须 `--yes`**，缺了直接拒绝（退出码 3）。
2. **必须 `--hold <秒>`，范围 1..60**。到期自动 `hold_position()` → `disable()`，
   即使客户忘了 Ctrl-C 也不会一直转。
3. **执行前先把将发送的量与设备量程打印到 stderr**，让日志里有据可查：

```
jsdk-cli: MIT 将发送 pos=0.1000 vel=0.0000 kp=2.0000 kd=0.2000 tau=0.0000，持续 1 s
          量程: pos=±12.5000 vel=±65.0000 tau=±50.0000 kp=500.0000 kd=5.0000 gear=16.5000
```

收到 `SIGINT`/`SIGTERM` 时，主循环会**先 `hold_position()` 再 `disable()`** 再退出。
顺序不可颠倒：先停发控制帧会直接触发设备的 `break_timeout` 保护。

```bash
# 零位置、零增益（最温和的"通电抱持"验证）
jsdk-cli --if socketcan --channel can0 --node 1 --yes --hold 3 mit --kp 0 --kd 0

# 用"输出端真实刚度"给值（SDK 按 kp = 刚度 × 2π / gear 换算）
jsdk-cli --if socketcan --channel can0 --yes --hold 5 mit --pos 0.0 --stiffness 20
```

`--kp` 是**线上值、原样透传**；固件把它作用在电机端 turns 误差上，因此输出端
实际刚度 = `kp × gear_ratio / 2π`（gear 16.5 时约 2.63 倍）。要按真实刚度给值
请用 `--stiffness`。

---

## 5. `--json` 的字段名

字段名是**契约**（Python 绑定、CI 脚本会按这些名字断言）。当前保证存在：

| 命令 | 顶层字段 |
|---|---|
| `health` | `joint{ node, pos_rad, vel_rad_s, current_A, torque_Nm, t_motor_C, t_fet_C, vbus_V, ibus_A, axis_state, mode, mode_state_nibble, err_code, err_name, hb_error, axis_error, age_ms, tx_frames, tx_rejected, status_flags, online, enabled, fault }`、`bus{ tx_frames, rx_frames, tx_failed, rx_dropped, keepalive_sent, last_rx_age_ms, link_errors, hal_bus_flags, nodes_online, link_up }` |
| `dump-config` | `valid, gear_ratio, mit_max_pos, mit_max_vel, mit_max_torque, mit_max_kp, mit_max_kd, torque_constant, node_id, heartbeat_rate_ms, break_timeout_ms` |
| `read` | `path, type, value, value_text` |
| `batch-read` | `values[]{ path, value｜error, type }, status, single_frame` |
| `ep-list` | `endpoints[]{ path, id, type, access }, count, status` |
| `ep-lookup` | `path, id, type, readable, writable` |
| `scan` | `nodes[], count` |
| `desc-info` | `total_len, crc, fw_version, hw_version, endpoint_count, parsed_total, frames_rx, bytes_scanned, complete, mode_used, shared_hit, raw_sink_failed` |
| `hb-dump` | `heartbeats[]{ src, len, bytes, life, err_flags, state, control_mode }, count, waited_ms` |
| `desc-export` | `file, bytes` |
| `desc-import` | `file, bytes, imported, endpoint_count, downloaded:false` |
| `mon` | NDJSON：每行一个对象（`t_ms, node, pos_rad, …`） |
| 被拒绝 | `refused, reason, hint{ rerun }` |
| 出错 | `error{ op, status, status_code, message }` |

`mon` 在 `--json` 下输出 **NDJSON**（每行一个对象），便于流式管道消费。

---

## 6. 退出码

| 码 | 含义 |
|---|---|
| 0 | 成功 |
| 1 | 运行时失败（总线、超时、设备拒绝…）；原因在 stderr 与 `last_error` |
| 2 | 用法错误（缺子命令/未知选项/取值非法/未知后端/未知子命令） |
| 3 | 被安全闸拒绝（缺 `--yes`，或 `mit` 缺 `--hold`） |

`--help` 退出码是 **0**（它不是错误）。

脚本里因此可以区分"配错了"（2）、"跑失败了"（1）与"被安全闸拦住了"（3）。

---

## 7. 典型演练

```bash
# 1) 先确认总线和节点（不下载描述符，很快）
jsdk-cli --if socketcan --channel can0 scan
# 发现 1 个节点（被动 200 ms + 主动探测 1..16）:
#   node 1

# 2) 看健康
jsdk-cli --if socketcan --channel can0 --node 1 health

# 3) 端点表：先看规模，再找 mit_max 家族
jsdk-cli --if socketcan --channel can0 desc-info
jsdk-cli --if socketcan --channel can0 --filter mit_max_ ep-list

# 4) 标定量
jsdk-cli --if socketcan --channel can0 dump-config

# 5) 采 10 秒数据（CSV 供后续画图）
jsdk-cli --if socketcan --channel can0 --duration 10 --rate-hz 50 \
         --csv run1.csv mon

# 6) 危险动作：显式确认 + 自限时
jsdk-cli --if socketcan --channel can0 --node 1 --yes --hold 3 \
         mit --pos 0 --kp 2 --kd 0.2
```

CI 里冒烟（不需要硬件）：把 `--if` 换成 `virtual` 即可跑通全部只读子命令。

```bash
jsdk-cli --if virtual scan --json
```

---

## 8. 已知限制（不隐藏）

| 限制 | 说明 |
|---|---|
| 真机收发未进 CI | `tests/test_cli.c` 在 `virtual` 后端断言全部子命令与安全闸；真机需人工冒烟（见 `PORTING.zh-CN.md` §7.5.3） |
| `dump-config` 的 `heartbeat_rate_ms` 目前为 0 | 配置快照暂未包含该字段（P1） |
| 读参数 | 标量端点（≤ 8 字节，含 `u64`/`double`）都读得到：FD 一次请求，**Classic 自动分两块**。`object`/`json`/`endpoint_ref` 这类非标量端点没有标量尺寸，`read` 会明确报 `UNSUPPORTED`（不做“读一半”） |
| `mon` 的实时性 | 墙钟节奏 + 非实时线程，抖动取决于操作系统。硬实时请写自己的 C 循环 |
| Windows 控制台中文 | 见 §1（`chcp 65001` 或 `--json`） |
| slcan 吞吐 | 约 100~500 fps（ASCII 展开 + USB 帧调度），不适合高频控制 |
| slcan 数据段速率 | 只有 `2000000`（`Y2`）与 `5000000`（`Y5`）有公认命令码；表外值报 `UNSUPPORTED`，传 `--data-bitrate 0` 则不碰适配器配置 |
| `hb-dump` 的留存深度 | 只保留**本次调用**收到的最近 16 帧（进程级，不跨调用） |
