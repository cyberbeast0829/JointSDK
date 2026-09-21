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
| 采集一段数据 | `mon --csv > run1.csv` |
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

### Windows 控制台的中文显示（v0.22 起自动处理）

**背景**：本工程的中文全是 **UTF-8** 字面量，而 Windows 控制台的默认代码页是
**CP936(GBK)**（中文机器上 `chcp` 会告诉你 `936`）。C 运行时不看代码页，只把
字节原样交给控制台，控制台按**自己的**代码页解释 → 于是现场看到的是

```
$ jsdk-cli --if slcan scan
鍙戠幇 0 涓�鑺傜偣锛堣��鍔� 200 ms + 涓诲姩鎺㈡祴 1..16锛�:
```

（实际想显示"发现 0 个节点（被动 200 ms + 主动探测 1..16）"。）

**现在的规则**（`tools/jsdk_cli/cli_text.h`，与 git for Windows 的做法同思路）：

| 输出去哪 | 怎么发 |
|---|---|
| **控制台** | UTF-8 先转成 `GetConsoleOutputCP()` 再发 → 中文正常 |
| **管道/重定向到文件** | 原样发 UTF-8 → 脚本、`| jq`、编辑器都拿到 UTF-8 |

也就是说：**`jsdk-cli ... > out.txt` 与 `jsdk-cli ... | grep x` 里是 UTF-8，
屏幕上是控制台代码页**，两边都不用你操心。

**逃生门**：`JSDK_CLI_TEXT` 环境变量（排查或特殊管道用）

| 值 | 含义 |
|---|---|
| 不设 | 自动（上表的默认行为） |
| `utf8`（或 `off`） | 一律按 UTF-8 原样写。给**本来就吃 UTF-8** 的终端（MinTTY、Windows Terminal）或管道另一端期待 UTF-8 的场合 |
| 代码页数字，如 `936` | 一律按"控制台 + 该代码页"处理。CI 里用它验证转码路径，不必造真控制台 |

**怎么自查**（三行，直接看字节，不靠眼睛）：

```bash
# 1) 屏幕上直接看：应当是中文（不是 鍙戠幇 ...）
./build/jsdk-cli.exe --if virtual scan

# 1b) A/B 对照：加 JSDK_CLI_TEXT=utf8 就会**复现**修复前的乱码
JSDK_CLI_TEXT=utf8 ./build/jsdk-cli.exe --if virtual scan

# 2) 重定向后必须是 UTF-8（能被 python 以 utf-8 解码）
./build/jsdk-cli.exe --if virtual scan > out.txt
python -c "import io;print(io.open('out.txt',encoding='utf-8').read()[:40])"

# 3) 强制走"控制台 + 936"路径，直接看字节（应为 b7 a2 cf d6 = 发现）
JSDK_CLI_TEXT=936 ./build/jsdk-cli.exe --if virtual scan > gbk.bin
python -c "d=open('gbk.bin','rb').read();print(d[:4].hex(' '), d.decode('gbk')[:16])"
```

**⚠ 两个副作用要知道**

1. 控制台代码页里**表示不了**的字符会变成 `?`（不是悄悄换成别的字）。所以
   输出字符串里不要用 `⚠`（U+26A0）、`✓`（U+2713）这类字符 —— 这条已经写成了
   静态检查 `tools/check_cli_text.py`（ctest 里的 `cli_text_lint`），
   它同时挡住"绕过 `cli_text` 直接用 `printf`"的回退。
2. **不要**用 `chcp 65001` 解决：那改的是**共享的控制台状态**（程序被 kill 就
   回不去、同控制台的其它进程跟着遭殃），还要求控制台字体有 CJK 字形。
   本工程改成"按接收端的约定发字节"，不改环境。

**为什么 `--json` 也走同一条路**：`--json` 里可能有中文（错误消息）。重定向时它
是 UTF-8（机器读没问题），打在屏幕上时按控制台代码页显示（人读没问题）。

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
| `--csv` | `mon` 用 **CSV** 而不是 NDJSON/表格（写 stdout） |
| `--csv-file FILE` | `mon` **额外**把同一份 CSV 写到文件（与 `--csv` 同列，可只写文件） |
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
| `mon` | 周期监控；`--csv` 换格式，`--csv-file` 同时落文件 |
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

> **只读命令的前置条件（真机联调后修正）**：命令按“需要多少前置”分三档 ——
> ① `scan` / `info` / `err` / `hb-dump` / `estop`：**连描述符都不下载**（省流量）；
> ② `read` / `batch-read` / `desc-*` / `ep-*` / `write` / `watchdog` /
> `save` / `set-node-id` / `reset`：需要描述符，**不需要标定**（`jsdk_joint_param_get()`
> 本来就没有标定门槛）；
> ③ `health` / **`dump-config`** / `mon` / `set-zero` / `calibrate` / `home` / `mit`：完整配置（含标定）。
> 另外只有**跑循环**的命令（`mon`/`calibrate`/`home`/`mit`）会设控制周期，因此
> “周期必须小于设备 `break_timeout`”这条校验**不会**挡住单次请求类命令。
> 细节与踩坑记录见 `docs/BACKLOG.zh-CN.md` §2.4。
>
> ⚠ `dump-config` 打印的是**标定后的快照**（`gear_ratio` / `mit_max_*` / `torque_constant` /
> `heartbeat_rate_ms`），所以它属于第③档。放在第②档时它只能打出一堆 0 与 `valid=0`，
> 看上去像“设备没配好” —— 实际上只是那一次调用没跑标定。
>
> ⚠ **参数值在线上是小端**（`read`/`batch-read` 打印的数值已由 SDK 正确解码，但若要
> 自己解析 `--json` 的裸字节或 SDO 缓冲区，必须知道这一点）：见
> `docs/PROTOCOL_NOTES.zh-CN.md` §3.1。

### 3.2 写（需要 `--yes`）

| 命令 | 说明 |
|---|---|
| `write <path> <value>` | 参数写。值按端点**真实类型/位宽**解析，超范围**直接拒绝**（不静默截断） |
| `save` | `CONFIG_SAVE(0x22)`，写后读回校验 |
| `set-node-id N` | 改节点地址（含冲突检查、验证新地址有应答，可持久化） |
| `watchdog MS` | 写 `can.config.break_timeout`。**`0` = 关闭**设备侧协议级超时检测（最新固件语义；旧固件把 0 当 100 ms）。⚠ 真机上该端点读回恒为 0（F28），所以输出里 `device_reports_ms` 是**独立再读**的结果，`verified:false` 表示“没确认到” |
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
         --csv-file run1.csv mon

# 6) 危险动作：显式确认 + 自限时
jsdk-cli --if socketcan --channel can0 --node 1 --yes --hold 3 \
         mit --pos 0 --kp 2 --kd 0.2
```

CI 里冒烟（不需要硬件）：把 `--if` 换成 `virtual` 即可跑通全部只读子命令。

```bash
jsdk-cli --if virtual scan --json
```

---

## 8. 自检：怎么确认"链路是好的"（而不是"跑了一次看到成功"）

**为什么不能只跑一次**：本项目踩过两次"开发机跑通、现场报错"，两次都不是幻觉 ——

1. **slcan 适配器打开端口后的头几帧会丢**（实测约 **1/10 次进程**），单跑一次很容易
   恰好成功 → 于是"已真机验证"的结论掩盖了 10% 的失败率。症状是
   `configure() 失败：timeout` / `descriptor download ... (0/0 bytes)`，**再敲一次就好了**；
2. **参数值字节序错时 `read` 依然"成功返回"**（只是数值是垃圾，如 `8.9e-41`）——
   只看退出码永远发现不了。

所以自检的规矩是：**每层独立起进程 + 重复 N 轮 + 对数值做交叉校验**。

```bash
# 真机（默认 slcan + COM3 + node 1，跑 3 轮）
./tools/hw_verify.sh --channel COM3 --node 1 --runs 5 \
    --expect axis0.config.can.node_id=1 \
    --expect axis0.config.can.heartbeat_rate_ms=100 \
    --expect axis0.motor.config.gear_ratio=7.75

# 连**写路径**一起验（会真的写：先把读到的原值原样回写，再改一下数值并恢复）
./tools/hw_verify.sh --channel COM3 --node 1 --runs 3 --write-probe

# 没有硬件时自检脚本本身（走虚拟后端；L8/L8b 会显式跳过）
./tools/hw_verify.sh --if virtual --runs 1
```

脚本的输出（真机示例，5 轮；这是**只读**的 8 层。加 `--write-probe` 会多出 L8/L8b 两行）：

```
=== 汇总（5 轮）===
  层            通过 失败
  L1-scan             5      0
  L2-info             5      0
  L3-desc             5      0
  L4-hb               5      0
  L5-read            20      0
  L6-batch           20      0
  L6b-expect         15      0
  L7-health           5      0

结论：全部通过（80 项检查）
```

### 8.1 层次含义（失败在哪一层，就说明哪一层的问题）

| 层 | 命令 | 判据 | 失败说明 |
|---|---|---|---|
| L1 | `scan` | 报出目标节点 | 适配器/端口/波特率/接线 |
| L2 | `info` | `fw_version > 0` | 单请求收发不通（0x46 不通） |
| L3 | `desc-info` | `complete=true` 且 `bytes_scanned==total_len` | 38 KB 流式下载失败 —— **最常暴露"首帧丢失"** |
| L4 | `hb-dump` | 2 s 内收到心跳 | **只收不发**：它失败 = 通道层面（CRX 没打开/终端电阻/bitrate/上电） |
| L5 | `read` ×4 | 退出码 0 + 数值在合理范围 | 值荒谬 = 字节序/类型解析（见 `PROTOCOL_NOTES` §3.1） |
| L6 | `batch-read` | **与 L5 的同一个参数完全一致** | 两条解析路径不一致，说明其中一条错 |
| L6b | `--expect` | 与设备真值相符 | 你已知真值时用它钉死（换设备后记得更新） |
| L7 | `health` | `online=true` 且 `fault=false` | 标定/配置问题，看 stderr 第一条原因 |
| L8 | `read` + `write` + `read` | 原值回写后读回必须相符 | 写路径：请求打包 / 设备拒绝 / 值不对（**顺带验证写方向的字节序**） |
| L8b | 同上 + `--write-probe` | 写原值+Δ 后读回 = 新值，**且已恢复原值** | 先看是不是“恢复失败”（那种要手动 `--yes write` 回写） |

⚠ **L8/L8b 在虚拟后端会显式跳过**：每个 CLI 进程都是一条新仿真总线，写入不会跨进程保留，
在仿真上跑这两层只会给出“通过”的假象。它们的真值在真机上 —— 实测 `100 → 150 → 恢复 100`，
这同时证明了**写方向的字节序也对**（若按大端写，设备会把 150 存成 `0x96000000`）。

**关键判读**：`L3` 失败而 `L4` 通过 ⇒ 链路是通的、**只是请求没到达设备**（首帧丢失类）；
`L3` 与 `L4` 同时失败 ⇒ 通道层面，与"请求"无关。

### 8.2 命令行逐条手敲也可以

```bash
./build/jsdk-cli --if slcan --channel COM3 scan                 # ① 有没有节点
./build/jsdk-cli --if slcan --channel COM3 hb-dump              # ② 收到了什么（只收不发）
./build/jsdk-cli --if slcan --channel COM3 --json desc-info     # ③ 描述符
./build/jsdk-cli --if slcan --channel COM3 --json \
        read axis0.motor.config.gear_ratio                      # ④ 数值是否荒谬
./build/jsdk-cli --if slcan --channel COM3 --json health        # ⑤ 完整配置 + 健康
```

⚠ 每次都加 `--json`：纯文本在 Windows 控制台上可能被编码问题干扰，而 JSON 是 ASCII
键名 + 稳定字段（见 §5）。

### 8.3 已知的"会自愈"的失败

| 现象 | 原因 | 现在的处理 |
|---|---|---|
| 第一次跑 `configure()/desc-info` 超时，再跑一次就好 | 适配器打开端口后头几帧被丢 | ① `hal_slcan` 的 `C`/`Y<n>`/`O` **等适配器 ACK，没 ACK 就重发**；② 描述符请求按 **0.25/0.6/1.2 s** 递增间隔**重发 3 次**（设备对 `0x24` 幂等）。修后实测：`desc-info` **20/20**、`health` **12/12**（修前约 1/10 失败） |
| 超时信息里有 `0/0 bytes, 0 frames received` | 通道层面：一帧都没收到 | 这是**明确诊断**，不是"设备慢"：查端口/终端电阻/bitrate/上电 |
| 超时信息里有 `0/0 bytes, N frames received`（N>0） | 通道是通的，但请求没到达设备 | 重发已用尽：查适配器固件、或设备是否在过滤该 MsgType |

---

## 9. 已知限制（不隐藏）

| 限制 | 说明 |
|---|---|
| 真机收发未进 CI | `tests/test_cli.c` 在 `virtual` 后端断言全部子命令与安全闸；真机需人工冒烟（见 `PORTING.zh-CN.md` §7.5.3），**已脚本化**：`tools/hw_verify.sh`（L1~L8b 分层，含写路径） |
| `watchdog` 的读回校验 | 真机上 `can.config.break_timeout` **读回恒为 0**（写 250 立刻读也是 0，而 `0` 的含义是**禁用**，见 `FIRMWARE_ISSUES` **F28**）→ 写后判定分三类：相等（含 `0 == 0`，即真的关上了）= 通过；**读回 0 而写入非 0 = “已接受但无法校验”**（保留写入值 + `verified:false`，退出码仍是 0）；其它值 = `PROTOCOL` |
| 另一个入口 | `python -m jsdk_can` 是**同一份契约**的 Python 实现：**24 个子命令**、同款安全闸（`--yes` / `mit` 的 `--hold` / `estop` 免确认）、同款退出码 0/1/2/3、同款 JSON 字段（有对拍用例）。已知差异：C 版 `mon` 在**虚拟后端**不受墙钟约束（虚拟时钟由循环驱动），Python 版用墙钟 |
| 读参数 | 标量端点（≤ 8 字节，含 `u64`/`double`）都读得到：FD 一次请求，**Classic 自动分两块**。`object`/`json`/`endpoint_ref` 这类非标量端点没有标量尺寸，`read` 会明确报 `UNSUPPORTED`（不做“读一半”） |
| `mon` 的实时性 | 墙钟节奏 + 非实时线程，抖动取决于操作系统。硬实时请写自己的 C 循环 |
| Windows 控制台中文 | 见 §1（`chcp 65001` 或 `--json`） |
| slcan 吞吐 | 约 100~500 fps（ASCII 展开 + USB 帧调度），不适合高频控制 |
| slcan 数据段速率 | 只有 `2000000`（`Y2`）与 `5000000`（`Y5`）有公认命令码；表外值报 `UNSUPPORTED`，传 `--data-bitrate 0` 则不碰适配器配置 |
| `hb-dump` 的留存深度 | 只保留**本次调用**收到的最近 16 帧（进程级，不跨调用） |
| `--csv` / `--csv-file` | `--csv` 是**格式开关**（写 stdout），`--csv-file F` 才落盘。以前 `--csv` 要一个文件名，于是 `mon --csv --duration 1` 会把 `--duration` 当文件名、**静默写出一个叫 `--duration` 的 CSV**（真发生过）—— 现在取值以 `-` 开头会**当场报用法错**（rc=2）。⚠ 两版 CLI 的 CSV 表头（17 列）与数值格式**逐字节一致**，有对拍用例 |
