# CyberBeast Joint SDK（CAN / CAN-FD 后端）

包装关节电机驱动器的 **CYBERBEAST 协议**的纯 C99 SDK，
目标是 **Windows / Linux / 嵌入式 MCU 同一套 API**。

与 `jsdk_*` 家族（`SOEM/joint-sdk`、`EtherCAT_Master/joint-sdk`）同名同义，是第三个后端。

---

## 核心设计约束

| 约束 | 做法 |
|---|---|
| 跨平台，含 MCU | 纯 C99；无 VLA、无 C11 原子/线程、无编译器扩展、**零 malloc**（arena 由调用者提供） |
| 隔离协议细节 | 客户只看到 `jsdk_joint_*` 物理量 API；帧格式、端点 ID、字节序全在内部 |
| 不内置端点表 | 端点 **完全来自运行时下载的 JSON 描述符**（实测跨固件版本 86% 的端点 ID 会漂移） |
| 不猜 | 参数缺失/超范围/类型不符 → 明确报错；拒绝静默截断、拒绝猜量程 |
| 不静默丢字段 | 描述符提前终止只在 filter **全为精确路径**时启用 |
| RT 安全 | 控制循环无阻塞 / 无分配 / 无日志；阻塞 API 只在配置阶段 |

---

## 快速开始

### 0. 环境准备（各平台推荐做法）

| 平台 / 工具链 | 推荐装什么 | 说明 |
|---|---|---|
| **Windows + MinGW**（本项目的长期验证组合） | MSYS2 → `pacman -S --needed mingw-w64-x86_64-gcc mingw-w64-x86_64-cmake mingw-w64-x86_64-make`，并把 `C:\msys64\mingw64\bin` 加进 `PATH` | **必须显式 `-G "MinGW Makefiles"`**，否则 CMake 会挑到 Visual Studio/Ninja。另外建议装 **Git for Windows**（`tools/*.sh` 验证脚本用 bash 跑） |
| **Windows + MSVC** | Visual Studio 2022（或 Build Tools）勾选 **“使用 C++ 的桌面开发”** | 多配置生成器：构建要 `--config Release`，测试要 `ctest -C Release`。本项目已为 MSVC 加好 `/std:c11 /utf-8 /W4 /WX`（否则默认的 C89 语言模式会直接编不过） |
| **Linux / WSL（Ubuntu 20.04+）** | `sudo apt install build-essential cmake python3 python3-pytest`（`pytest` 只为 Python 绑定套件；SocketCAN 调试再装 `can-utils iproute2`） | 用系统默认生成器即可（Unix Makefiles）；想要更快的构建可另装 `ninja-build` |
| **macOS** | `xcode-select --install` + `brew install cmake` | **未实测**（本仓没有 macOS 机器） |
| **MCU / 裸机** | 不需要 CMake | 用 `dist/jsdk_can_amalgam.{h,c}`（单文件版）或把 `src/**` 加进你的工程；三个回调（`send`/`recv`/`now_ms`）自己实现，见 `docs/PORTING.zh-CN.md` |

> ⚠ **CMake 版本**：`ctest --test-dir <dir>` 需要 **CMake ≥ 3.20**。
> Ubuntu 20.04 自带的 3.16 没有这个选项 —— 那就 `cd build && ctest`（本仓的 `tools/wsl_build.sh` 就是这么做的）。

### 1. 构建（按环境选一条）

```bash
# ── Windows / MinGW Makefiles ────────────────────────────────────────
cmake -S . -B build -G "MinGW Makefiles" -DJSDK_WERROR=ON -DJSDK_ENABLE_HEAP=ON -DJSDK_BUILD_CLI=ON
cmake --build build
ctest --test-dir build                     # CMake ≥3.20；否则 cd build && ctest

# ── Windows / Visual Studio（多配置）────────────────────────────────
cmake -S . -B build-vs -G "Visual Studio 17 2022" -A x64 -DJSDK_WERROR=ON -DJSDK_ENABLE_HEAP=ON -DJSDK_BUILD_CLI=ON
cmake --build build-vs --config Release
ctest --test-dir build-vs -C Release

# ── Windows / Ninja（要另外装 ninja；用 MSVC 时先从 “Developer PowerShell” 起）──
cmake -S . -B build-ninja -G Ninja -DCMAKE_BUILD_TYPE=Release -DJSDK_WERROR=ON -DJSDK_ENABLE_HEAP=ON -DJSDK_BUILD_CLI=ON
cmake --build build-ninja

# ── Linux / macOS（默认 Unix Makefiles；也可 -G Ninja）──────────────
cmake -S . -B build -DJSDK_WERROR=ON -DJSDK_ENABLE_HEAP=ON -DJSDK_BUILD_CLI=ON
cmake --build build -j4
cd build && ctest

# ── 只跑一遍“无硬件自检”（库 + 测试，不要 CLI/堆）────────────────────
cmake -S . -B build -DJSDK_WERROR=ON && cmake --build build && cd build && ctest
```

### 2. 跑起来（无硬件也能验证）

```bash
./build/jsdk-cli --if virtual scan          # 内置虚拟总线 + 驱动器模型
./build/jsdk-cli --if virtual health
```

### 3. 共享库 / Python 绑定（可选）

```bash
# 共享库（DLL/.so）+ 把库拷进 Python 包
cmake -S . -B bsh -G "MinGW Makefiles" -DJSDK_WERROR=ON -DJSDK_ENABLE_HEAP=ON \
      -DJSDK_BUILD_SHARED=ON -DJSDK_BUILD_PYTHON=ON
cmake --build bsh && ctest --test-dir bsh

pip install -e "bindings/python[test]"
python -m pytest bindings/python/tests -q
```

> ⚠ **只保留本平台那一份库**：`JSDK_BUILD_PYTHON=ON` 会把共享库拷进
> `bindings/python/src/jsdk_can/lib/` 并**清掉别的平台的文件名**。所以
> “在 WSL 里验 Linux、在 Windows 里验 Windows”时别互相覆盖 —— Linux 侧建议
> 不带 `JSDK_BUILD_PYTHON`，改用 `JSDK_LIB_PATH=/path/to/bsh-linux`。

### 4. 本项目已在哪些环境验证过

| 环境 | 工具链实测版本 | 生成器 | 结果 |
|---|---|---|---|
| Windows | gcc 13.2.0（MSYS2）+ CMake 4.4.3 | `MinGW Makefiles` | ctest **21/21**（共享库构建 22/22）、11 套 C 测试 **30504 项 0 失败**、`-Werror` 0 告警 |
| Windows | MSVC 19.44（VS 2022 Build Tools）+ CMake 4.4.3 | `Visual Studio 17 2022` | ctest **21/21**（共享库构建 22/22）、`/W4 /WX` **0 告警** |
| Linux / WSL | gcc 9.4.0 + CMake 3.16（Ubuntu 20.04） | 默认 | ctest **21/21**；ASan + UBSan **21/21** |
| Python 绑定 | 3.12（Windows）/ 3.8.10（Linux） | — | 两平台各 **217 通过 / 2 跳过** |
| macOS | — | — | **未实测** |
| 真机 | CANable（slcan）+ 一台关节 | — | 见下文「真机（硬件）常用命令」 |

---

## CMake 选项

| 选项 | 默认 | 说明 |
|---|---|---|
| `JSDK_BUILD_TESTS` | ON | 单元测试（**11 套 / 30504 项断言**） |
| `JSDK_WERROR` | OFF | 把告警当错误（MSVC 下是 `/WX`） |
| `JSDK_BUILD_HAL_VIRTUAL` | ON | 虚拟总线 + 驱动器模型（自带描述符，CI/离线用） |
| `JSDK_BUILD_HAL_SOCKETCAN` | Linux ON | Linux SocketCAN（CAN FD + BRS） |
| `JSDK_BUILD_HAL_PCAN` | Win/macOS ON | PEAK PCAN-Basic（**运行期**加载，不需链接 `.lib`） |
| `JSDK_BUILD_HAL_SLCAN` | ON | 串口 slcan（CANable 等；支持 CAN FD，见下文） |
| `JSDK_ENABLE_HEAP` | OFF | 堆模式（`jsdk_context_create/free`） |
| `JSDK_BUILD_CLI` | 跟随堆模式 | `jsdk-cli` 诊断工具 |
| `JSDK_BUILD_SHARED` | OFF | 共享库（Python 绑定/FFI 用；只导出公共 ABI） |
| `JSDK_BUILD_PYTHON` | OFF | 把共享库拷进 `bindings/python/src/jsdk_can/lib/` |

MCU 构建：`-DJSDK_BUILD_HAL_*=OFF -DJSDK_BUILD_CLI=OFF -DJSDK_ENABLE_HEAP=OFF`，
自己实现 `jsdk_can_hal_t`（三个回调：`send` / `recv` / `now_ms`）。

### 构建脚本（可选，都是“一条命令跑一层”）

| 脚本 | 用途 |
|---|---|
| `tools/wsl_build.sh {configure\|build\|ctest\|all\|asan\|pcan}` | 在 WSL 里配置/构建/跑测试/跑 ASan/强制编 PCAN 后端 |
| `tools/hw_verify.sh --if slcan --channel COM3` | **真机分层自检**（L1~L8b，含写路径探针；`--if virtual` 可无硬件自检脚本本身） |
| `tools/packaging_smoke.sh` | 真装到临时前缀，用 pkg-config / 静态链接 / `find_package` 三个真实消费者验一遍 |
| `tools/amalgam_smoke.sh`、`tools/arduino_smoke.sh` | 单文件版与 Arduino 包装的冒烟 |

---

## 客户侧最小用法

```c
#include "joint_sdk.h"

static uint8_t g_ctx[6144];      /* 或 jsdk_context_size(NULL) */
static uint8_t g_arena[1024];    /* RETAIN_FILTERED：见 PORTING §2 */

static const char *const k_paths[] = {
    "axis0.motor.config.gear_ratio",
    "axis0.motor.config.torque_constant",
    "axis0.controller.config.mit_max_pos",
    "axis0.controller.config.mit_max_vel",
    "axis0.controller.config.mit_max_torque",
    "axis0.controller.config.mit_max_kp",
    "axis0.controller.config.mit_max_kd"
};

jsdk_context_config_t cfg;
jsdk_context_config_default(&cfg);
cfg.hal               = my_hal;          /* 你的 send/recv/now_ms */
cfg.master_id         = 1;
cfg.is_fd             = 1;
cfg.desc.retain       = JSDK_DESC_RETAIN_FILTERED;
cfg.desc.filter_paths = k_paths;
cfg.desc.filter_count = sizeof k_paths / sizeof k_paths[0];
cfg.desc.arena        = g_arena;
cfg.desc.arena_size   = sizeof g_arena;

jsdk_context_init((jsdk_context_t *)g_ctx, &cfg);
jsdk_context_add_joint(...);              /* 填 node_id / initial_mode */
jsdk_context_configure(...);              /* 握手 + 下载描述符 + 读回量程 */
jsdk_context_activate(...);               /* 使能（含安全首帧） */

for (;;) {                                /* 控制循环：RT 安全 */
    jsdk_context_cycle_begin(ctx, app_ns);
    jsdk_joint_set_mit(j, pos, vel, kp, kd, tau);
    jsdk_context_cycle_end(ctx);
}
```

---

## 真机（硬件）常用命令：`jsdk-cli`

> 下面每一条都在**真机上实跑过**：CANable（slcan）+ 一台关节（`node 1`，`fw 1545`，FD）。
> 跑之前只需确认三件事：**后端（`--if`）→ 通道（`--channel`）→ 节点（`--node`）**。
> 任何命令都能叠 `--json`（机器可读，字段是契约）和 `-v` / `-q`（日志级别）。
>
> ⚠ 还有第四件事：**设备是 Classic 还是 FD**（`can.config.baud_rate`）——协议**没有**
> 运行时协商，猜错的现场表现就是“心跳收得到、但我的请求没人应”
> （`desc-info` 报 `0/0 bytes, N frames received`）。同一台 CyberBeast USB2CAN 在不同的
> 适配器/固件配置下可以是 **1 Mbps Classic**，所以：**不确定就别传** `--classic` /
> `--data-bitrate` —— SDK 会按对端第一帧自动对齐并打一行提示；传了就是明确指定，
> 冲突时只会警告（不会偷偷改你的值）。

### 0. 通道怎么填

| 硬件 | `--if` | `--channel` | 还要给什么 |
|---|---|---|---|
| CANable / 串口（slcan） | `slcan` | `COM3`（Win）/ `/dev/ttyACM0`（Linux） | `--baud 115200`（**串口**速率，不是 CAN 速率）；FD 数据段速率用 `--data-bitrate 2000000\|5000000`，或 `--classic` 强制 Classic。⚠ Linux 下 `/dev/ttyACM0` 属 `root:dialout`，用户不在 `dialout` 组里会报 `invalid-argument` + 权限原因提示（`--channel` 前加 `sudo` 或 `usermod -aG dialout`）|
| PEAK PCAN | `pcan` | `PCAN_USBBUS1` | 运行期加载 PCAN-Basic，无需链接 `.lib` |
| Linux SocketCAN | `socketcan` | `can0` | 链路速率用 `ip link` 设，SDK 不碰 |
| 无硬件自检 | `virtual` | 可省；或 `"0:id=1,timeout=30000,fd"` | 内置驱动器模型（自带描述符） |

**默认值**：Windows/macOS → `pcan` + `PCAN_USBBUS1`；Linux → `socketcan` + `can0`；`slcan` 通道默认 `COM3` / `/dev/ttyACM0`。

```bash
# 本仓自检脚本里的真机组合（Windows）
./build/jsdk-cli --if slcan --channel COM3 --node 1 <子命令>
```

### 1. 只读巡检（安全，不动设备）

```bash
# 总线上有谁（被动听 200 ms + 主动探测 1..16）
./build/jsdk-cli --if slcan --channel COM3 scan
# 发现 1 个节点（被动 200 ms + 主动探测 1..16）:  node 1

# 这台是不是我要的板子：hw / fw / 序列号
./build/jsdk-cli --if slcan --channel COM3 --node 1 info
# {"hw_version":262711,"fw_version":1545,"serial":169659955819600,"classic":false,"node":1}

# 一眼看健康：模式/错误码/温度/母线/反馈新鲜度/链路统计
./build/jsdk-cli --if slcan --channel COM3 --node 1 health

# 六类 32-bit 错误明细（axis/motor/encoder/controller/sensor/其它）
./build/jsdk-cli --if slcan --channel COM3 --node 1 err

# 最近心跳的**原始字节** + 解码对照（查“心跳对不对”最快）
./build/jsdk-cli --if slcan --channel COM3 --node 1 hb-dump

# 关键配置快照（标定读回值；valid=1 才可信）
./build/jsdk-cli --if slcan --channel COM3 --node 1 dump-config
# {"valid":true,"gear_ratio":7.75,"mit_max_pos":12.5,"mit_max_vel":65,"mit_max_torque":50,
#  "mit_max_kp":500,"mit_max_kd":5,"torque_constant":0.086432,"node_id":1,
#  "heartbeat_rate_ms":100,"break_timeout_ms":0}
#   ↑ break_timeout_ms = 0 表示**设备侧的协议级超时检测是关着的**（0 = 禁用）

# 读单个参数（类型随端点自动决定）
./build/jsdk-cli --if slcan --channel COM3 --node 1 read axis0.motor.config.gear_ratio
./build/jsdk-cli --if slcan --channel COM3 --node 1 read serial_number      # u64 也能读

# 批量读：FD 下打进**一帧**（Classic 自动逐条退化）
./build/jsdk-cli --if slcan --channel COM3 --node 1 --json batch-read \
    axis0.config.can.node_id axis0.motor.config.gear_ratio serial_number
# {"values":[{"path":"axis0.config.can.node_id","value":1,"type":"uint32"}, …],"single_frame":true}
```

### 2. 实时采集

```bash
# 默认：给人看的表格
./build/jsdk-cli --if slcan --channel COM3 --node 1 mon --duration 5 --rate-hz 20

# CSV 到 stdout（列与 Python 版**逐字节一致**，可直接喂 pandas/Excel）
./build/jsdk-cli --if slcan --channel COM3 --node 1 mon --csv --duration 30 --rate-hz 50 > run.csv

# CSV 同时落盘 + 屏幕仍是 NDJSON（`--duration 0` = 直到 Ctrl-C）
./build/jsdk-cli --if slcan --channel COM3 --node 1 --json mon --csv-file run.csv --duration 0
```

### 3. 描述符与端点（换固件后“路径→ID”变了就靠这几个）

```bash
./build/jsdk-cli --if slcan --channel COM3 --node 1 desc-info
./build/jsdk-cli --if slcan --channel COM3 --node 1 ep-lookup axis0.controller.config.mit_max_torque
./build/jsdk-cli --if slcan --channel COM3 --node 1 ep-list --filter mit_max_     # 子串匹配
./build/jsdk-cli --if slcan --channel COM3 --node 1 ep-list --filter "axis0.config.*"  # 前缀匹配
./build/jsdk-cli --if slcan --channel COM3 --node 1 desc-export desc.bin   # 缓存下来，下次免下载
```

### 4. 写参数（**需要 `--yes`**；不落 Flash，要持久化再跑 `save`）

```bash
# 写之前先看类型与量程（超范围/非数字会被拒，绝不静默截断）
./build/jsdk-cli --if slcan --channel COM3 --node 1 ep-lookup axis0.config.can.heartbeat_rate_ms

./build/jsdk-cli --if slcan --channel COM3 --node 1 --yes write axis0.config.can.heartbeat_rate_ms 100
./build/jsdk-cli --if slcan --channel COM3 --node 1 --yes write axis0.controller.config.vel_limit -5.0
#   ↑ 负数、0x 前缀、小数都支持

# 看门狗超时：**0 = 关闭**设备侧协议级超时检测；非 0 = 毫秒数
./build/jsdk-cli --if slcan --channel COM3 --node 1 --json --yes watchdog 300
# {"ms":300,"device_reports_ms":300,"verified":true,"disabled":false}
#   ↑ 真机实测（COM3 / fw 1545）：写 300 → 读回 300，verified=true。
#     注意：**非 0 就是真的武装**——之后若停发控制帧，设备会在 break_timeout 后
#     置 ERROR_CAN_BUS_FAILED 并 disarm（实测过）。测试完记得写回 0 或继续喂心跳。

./build/jsdk-cli --if slcan --channel COM3 --node 1 --json --yes write axis0.config.enable_watchdog 0
# {"path":"axis0.config.enable_watchdog","id":154,"type":"bool",
#  "requested":false,"value":false,"verified":true,"persisted":"no (use save to persist)"}
#   ↑ 每个 write 都**读回验证**：`verified:false` / 退出码 1 表示设备没接受这一帧
#     （曾经的坑：SDK 发的参数写帧不足 8 字节，被固件静默丢弃 —— 见 PROTOCOL_NOTES §5.4）

./build/jsdk-cli --if slcan --channel COM3 --node 1 --yes save            # 落 Flash
./build/jsdk-cli --if slcan --channel COM3 --node 1 --yes set-node-id 2   # 改地址（会先探测冲突）
```

### 5. 运动与安全（**会动电机**，务必先确认关节是自由的）

```bash
# 零点 / 标定 / 回零
./build/jsdk-cli --if slcan --channel COM3 --node 1 --yes set-zero
./build/jsdk-cli --if slcan --channel COM3 --node 1 --yes calibrate       # 写 requested_state=3 并等待
#   ↑ 真机实测整条序列 **29.5 s**（子状态 4 电机标定 → 7 索引搜索 → 1），
#     默认预算 120 s；要改就加 `--timeout-ms 300000`。
#     跑完会读回两个 pre_calibrated 标志：true 才算真的落上了（状态跑完 ≠ 生效）。
./build/jsdk-cli --if slcan --channel COM3 --node 1 --yes home            # 写 requested_state=11 并等待

# 唯一会驱动电机的命令：**必须**同时给 --yes 与 --hold（秒，1..60 自限时）
./build/jsdk-cli --if slcan --channel COM3 --node 1 --yes --hold 2 mit --pos 0.5 --kp 2 --kd 0.1 --tau 0
#   --pos/--vel 单位 rad、rad/s；--kp/--kd/--tau 的作用点见 docs/UNITS.zh-CN.md

# 急停：广播 ESTOP，**不需要** --yes（安全动作不该因为少个参数而失败）
./build/jsdk-cli --if slcan --channel COM3 estop
```

> ⚠ `mit` 结束后 SDK 会走“失能序列”（停电机 → 等模式落 IDLE → 再停发），
> 直接 Ctrl-C 掉进程会留下看门狗故障码 —— 用 `--hold` 让它自己收尾。

### 6. 一键真机自检（推荐发给现场）

```bash
# 10 层（scan→info→描述符→心跳→read→batch-read→health→写路径→写探针），
# 每层独立进程；写探针“写原值+Δ → 校验 → **无论成败都恢复** → 再校验”
./tools/hw_verify.sh --if slcan --channel COM3 --node 1 --runs 3 --write-probe
# 结论：全部通过（45 项检查）

# 只想断一条已知量（例如齿轮比）
./tools/hw_verify.sh --if slcan --channel COM3 --node 1 --expect axis0.motor.config.gear_ratio=7.75

# 脚本本身无硬件也能验：./tools/hw_verify.sh --if virtual --runs 1
```

### 7. 退出码与常见现象

| 退出码 | 含义 | 典型场景 |
|---|---|---|
| `0` | 成功 | 含“写入已接受但无法读回校验”（`verified:false`） |
| `1` | 运行期错误 | 超时、设备拒绝、值超出端点值域（如给 u16 写 99999） |
| `2` | 用法错误 | 选项/参数写错（如给 u16 端点写 `abc`、`--csv-file` 后面跟了另一个选项） |
| `3` | **被安全闸拦住** | 写/动类命令少了 `--yes`（或 `mit` 少了 `--hold`）：先判闸、后才开总线 |

| 现象 | 怎么办 |
|---|---|
| `configure() 失败：timeout` + `descriptor download … (0/0 bytes, 0 frames)` | slcan 打开端口后头几帧可能被适配器丢掉（实测约 1/10 进程）→ **重跑一次**；仍失败再查通道名/波特率/终端电阻 |
| `… (0/0 bytes, 680 frames received)` | 通道是通的，是我们的请求丢了 → SDK 会自动重发（0.25/0.6/1.2 s）；连续失败说明适配器配置有问题 |
| 周期类命令报 `control period ≥ break_timeout` | 设备侧超时太短（非 0）；缩短周期或 `watchdog` 写大一点 / 写 `0` 关掉 |
| 中文在 Windows 控制台变乱码 | `chcp 65001`，或一律用 `--json` |

---

## 文档

| 文档 | 内容 |
|---|---|
| [`docs/DESIGN.zh-CN.md`](docs/DESIGN.zh-CN.md) | 总体设计、ADR、分层、API 骨架、工作包与验收、变更记录 |
| [`docs/FIRMWARE_ISSUES.zh-CN.md`](docs/FIRMWARE_ISSUES.zh-CN.md) | **固件问题清单（交付固件团队）**：统一编号 F1~F29、类型/严重度/修复顺序、每条附固件源码锚点与 SDK 侧应对（**F28 已结案：是我们自己的 bug**，已在清单里订正） |
| [`docs/UNITS.zh-CN.md`](docs/UNITS.zh-CN.md) | **单位与 kp/kd 公式速查**：逐帧端别表、真实刚度换算、常见错误与自查三件套 |
| [`docs/MIGRATION.zh-CN.md`](docs/MIGRATION.zh-CN.md) | **从 EtherCAT 版迁移**：逐符号对照、六处必改语义、迁移检查表 |
| `docs/BACKLOG.zh-CN.md`](docs/BACKLOG.zh-CN.md) | **待办与审计台账**：未完成项（A12、B6）、已完成项对照、已知限制与“记录缺失”的诚实交代 |
| [`examples/README.md`](examples/README.md) | 8 个可跑示例（虚拟后端，无需硬件）与“换真机要改什么” |
| [`docs/PROTOCOL_NOTES.zh-CN.md`](docs/PROTOCOL_NOTES.zh-CN.md) | 帧级协议手册 + JSON 描述符解析算法 + 实现检查表（固件问题已移至 `FIRMWARE_ISSUES.zh-CN.md`） |
| [`docs/CLI.zh-CN.md`](docs/CLI.zh-CN.md) | `jsdk-cli` 全部子命令、`--json` 字段契约、安全闸、退出码、演练 |
| [`docs/PORTING.zh-CN.md`](docs/PORTING.zh-CN.md) | HAL 实现契约、arena 选型、Flash 缓存两条路线、桌面后端与冒烟清单 |

---

## 目录结构

```
include/joint_sdk/    公共头（joint_sdk.h = 唯一需要包含的头；jsdk_hal_builtin.h 仅桌面后端）
src/proto_cyberbeast/ L2 协议层（cb_*：帧编解码 / MIT / 控制 / 查询 / 心跳 / 参数 / 描述符 / 缓存）
src/core/             L3 关节层（jsdk_*：上下文 / 关节 / 看门狗 / 描述符 / 运维 / 参数 / 分组）
src/hal/              内置 HAL 后端（virtual / socketcan / pcan / slcan）
tools/jsdk_cli/       PC 诊断 CLI
tests/                11 套 C 单元与集成测试（含字节级黄金向量对拍）
tools/                夹具/黄金向量生成、构建与验证脚本（`wsl_build.sh`、`hw_verify.sh`、
                      packaging/amalgam/arduino/live_can 冒烟 + 变异测试）
```

---

## 当前状态

**WP1 ~ WP9 全部实现并通过回归**，交付面 **A1~A13**、审计项 **B1~B4 / B8~B10** 已完成。

| 维度 | 现状（可复现） |
|---|---|
| 构建 | **两套工具链都干净**：gcc（`-Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wstrict-prototypes` + `-Werror`）与 MSVC（`/W4 /WX /std:c11 /utf-8`） |
| C 测试 | **11 套 / 30504 项断言**，0 失败（Windows MinGW gcc 13；Windows MSVC 19.44；Linux WSL gcc 9；ASan+UBSan 同样全过） |
| ctest | **21 项**（11 套 C + 8 个示例 + `cli_text_lint` + `hw_verify_virtual`；共享库构建为 22 项，多一个 `python_bindings`） |
| Python | **224 passed / 2 skipped**（Windows 3.12 与 Linux 3.8 结果一致）；公共 C API **114/114 已绑定，0 缺口**；`python -m jsdk_can` 与 `jsdk-cli` **25 个子命令对齐**（同款安全闸、同款退出码、同款 JSON 字段与 CSV 列） |
| 真机 | slcan + CANable + 一台关节（node 1，fw 1545）：`scan`/`info`/`desc-*`/`read`/`batch-read`/`health`/`dump-config`/`mon` **与写路径**（原值回写 / 写探针后恢复，实测 `100 → 150 → 恢复 100`）逐条验证；`tools/hw_verify.sh` 多轮全过（3 轮 **45/45**） |

**未完成与已知限制的完整清单见 [`docs/BACKLOG.zh-CN.md`](docs/BACKLOG.zh-CN.md)**
（§1 未完成项 A12、B6；§3 已知限制 L1~L7 与记录缺失）。
真机层面**必须人工**的部分（终端电阻、error-frame/bus-off、真驱动器运动、写路径）见
[`docs/PORTING.zh-CN.md`](docs/PORTING.zh-CN.md) §7.5.3 B 段；只读自检步骤见
[`docs/CLI.zh-CN.md`](docs/CLI.zh-CN.md) §8。
