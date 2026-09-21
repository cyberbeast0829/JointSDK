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

```bash
# 库 + 测试（默认配置：无堆、无 CLI）
cmake -S . -B build -G "MinGW Makefiles" -DJSDK_WERROR=ON
cmake --build build && ctest --test-dir build

# 库 + 测试 + PC 诊断 CLI（CLI 需要堆模式）
cmake -S . -B build -G "MinGW Makefiles" -DJSDK_WERROR=ON -DJSDK_ENABLE_HEAP=ON -DJSDK_BUILD_CLI=ON
cmake --build build
./build/jsdk-cli --if virtual scan          # 无硬件也能跑
```

### CMake 选项

| 选项 | 默认 | 说明 |
|---|---|---|
| `JSDK_BUILD_TESTS` | ON | 单元测试（**10 套 / 25624 项断言**） |
| `JSDK_WERROR` | OFF | 把告警当错误 |
| `JSDK_BUILD_HAL_VIRTUAL` | ON | 虚拟总线 + 驱动器模型（自带描述符，CI/离线用） |
| `JSDK_BUILD_HAL_SOCKETCAN` | Linux ON | Linux SocketCAN（CAN FD + BRS） |
| `JSDK_BUILD_HAL_PCAN` | Win/macOS ON | PEAK PCAN-Basic（**运行期**加载，不需链接 `.lib`） |
| `JSDK_BUILD_HAL_SLCAN` | ON | 串口 slcan（CANable 等；支持 CAN FD，见下文） |
| `JSDK_ENABLE_HEAP` | OFF | 堆模式（`jsdk_context_create/free`） |
| `JSDK_BUILD_CLI` | 跟随堆模式 | `jsdk-cli` 诊断工具 |

MCU 构建：`-DJSDK_BUILD_HAL_*=OFF -DJSDK_BUILD_CLI=OFF -DJSDK_ENABLE_HEAP=OFF`，
自己实现 `jsdk_can_hal_t`（三个回调：`send` / `recv` / `now_ms`）。

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

## 文档

| 文档 | 内容 |
|---|---|
| [`docs/DESIGN.zh-CN.md`](docs/DESIGN.zh-CN.md) | 总体设计、ADR、分层、API 骨架、工作包与验收、变更记录 |
| [`docs/FIRMWARE_ISSUES.zh-CN.md`](docs/FIRMWARE_ISSUES.zh-CN.md) | **固件问题清单（交付固件团队）**：统一编号 F1~F26、类型/严重度/修复顺序、每条附固件源码锚点与 SDK 侧应对 |
| [`docs/UNITS.zh-CN.md`](docs/UNITS.zh-CN.md) | **单位与 kp/kd 公式速查**：逐帧端别表、真实刚度换算、常见错误与自查三件套 |
| [`docs/MIGRATION.zh-CN.md`](docs/MIGRATION.zh-CN.md) | **从 EtherCAT 版迁移**：逐符号对照、六处必改语义、迁移检查表 |
| [`docs/BACKLOG.zh-CN.md`](docs/BACKLOG.zh-CN.md) | **待办与审计台账**：未完成项（A11/A12/A13、B6）、已完成项对照、已知限制与“记录缺失”的诚实交代 |
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
| 构建 | `-Werror` 干净（`-Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wstrict-prototypes`） |
| C 测试 | **11 套 / 30486 项断言**，0 失败（Windows MinGW gcc 13；Linux WSL gcc 9；ASan+UBSan 同样全过） |
| ctest | **21 项**（11 套 C + 8 个示例 + `cli_text_lint` + `hw_verify_virtual`；共享库构建 `bsh` 为 18 项） |
| Python | **213 passed / 2 skipped**（Windows 3.12 与 Linux 3.8 结果一致）；公共 C API **114/114 已绑定，0 缺口**；`python -m jsdk_can` 与 `jsdk-cli` **24 个子命令对齐**（同款安全闸、同款退出码、同款 JSON 字段） |
| 真机 | slcan + CANable + 一台关节（node 1，fw 1545）：`scan`/`info`/`desc-*`/`read`/`batch-read`/`health`/`dump-config` **与写路径**（原值回写 / 写探针后恢复，实测 `100 → 150 → 恢复 100`）均已逐条验证；自检脚本 10 轮 **160 项**、含写路径 3 轮 **51 项**全过 |

**未完成与已知限制的完整清单见 [`docs/BACKLOG.zh-CN.md`](docs/BACKLOG.zh-CN.md)**
（§1 未完成项 A12、B6；§3 已知限制 L1~L7 与记录缺失）。
真机层面**必须人工**的部分（终端电阻、error-frame/bus-off、真驱动器运动、写路径）见
[`docs/PORTING.zh-CN.md`](docs/PORTING.zh-CN.md) §7.5.3 B 段；只读自检步骤见
[`docs/CLI.zh-CN.md`](docs/CLI.zh-CN.md) §8。
