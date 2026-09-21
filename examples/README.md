# 示例（7 个）

每个示例都是**独立可编译可运行**的最小程序，同时也是 `ctest` 的一部分
（构建后 `ctest -R example` 会全部跑一遍，退出码非 0 即失败）。

| # | 文件 | 学到什么 | 需要硬件 |
|---|---|---|---|
| 01 | `01_hello_virtual.c` | ABI 自检、虚拟后端、句柄关闭 | 否 |
| 02 | `02_enable_mit.c` | 使能 → MIT 定点 → 失能（安全动作序列） | 否 |
| 03 | `03_control_loop.c` | 1 kHz 控制循环骨架 + 节拍统计 | 否 |
| 04 | `04_param_rw.c` | 按路径读参数（含 **8 字节**端点）、批量读、写回读、`save` | 否 |
| 05 | `05_desc_cache.c` | 描述符下载、`enumerate`、**两条缓存路线** | 否 |
| 06 | `06_group_broadcast.c` | 4 关节一条 FD 帧同步、**何时会自动降级** | 否 |
| 07 | `07_custom_hal.c` | **自己实现 `jsdk_can_hal_t`**（MCU 客户的入口，零 malloc） | 否 |

01–06 用内置的**虚拟后端**（库自带一个简化固件模型），所以不需要任何硬件、不需要
CAN 卡，插上就能跑。07 反过来：它**故意不用**任何内置后端，只用 `joint_sdk.h`，
把 SDK 当纯协议栈用 —— 那才是 MCU 客户要走的路。

## 构建与运行

```bash
cmake -S . -B build -DJSDK_BUILD_EXAMPLES=ON
cmake --build build -j
ctest --test-dir build -R example --output-on-failure   # 一次跑完 7 个

./build/examples/01_hello_virtual
./build/examples/02_enable_mit
./build/examples/06_group_broadcast
```

单文件（amalgamation）用户：先生成，再用同一份源码对照
（见 `tools/amalgam_smoke.sh`，它就是这么验的）：

```bash
python tools/amalgamate.py --with virtual
gcc examples/01_hello_virtual.c dist/jsdk_can_amalgam.c -lm -o hello
```

## 真机上跑要改什么

示例全部跑虚拟后端，所以"换成真机"只有三处：

1. 打开后端：把 `jsdk_hal_virtual_open(...)` 换成 `jsdk_hal_socketcan_open(...)` /
   `jsdk_hal_slcan_open(...)` / `jsdk_hal_pcan_open(...)`，或者（MCU）换成你自己的
   `jsdk_can_hal_t`（照抄示例 07）；
2. **看门狗**：`configure()` 会检查"控制周期能不能喂得动设备的 `break_timeout`"
   （设备侧 `0` = 禁用，此时不检查）。周期大于设备的超时时，要么把周期改小，要么用
   `jsdk_joint_set_watchdog_ms()` 放宽（示例为了跑得快，把 `timeout` 设成了 30 s）；
3. **量程**：`configure()` 从设备读 `mit_max_*` / `gear_ratio`。真机上这些是设备的实况值，
   示例里的 spec 只是仿真设备的参数。

⚠ 真机动作前请先读 `docs/PORTING.zh-CN.md` §7.5.3 的人工冒烟清单：位定时、终端电阻、
error-frame/bus-off 这些只有真硬件才能验。
