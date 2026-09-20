# CyberBeast Joint SDK — Arduino / PlatformIO 包装

这个目录是一个**可以整目录拷进 `~/Arduino/libraries/`（或 PlatformIO 的 `lib/`）**
的库：

```
arduino/
├─ library.properties            # Arduino 库元数据（PlatformIO 也读它）
├─ src/
│   └─ jsdk_can_amalgam.{h,c}    # ⚠ **生成物**（见下），Arduino 只编译 src/ 下的源文件
├─ examples/
│   └─ 01_mit_move/              # 一个完整 sketch：使能 + MIT 周期控制
└─ extras/host_shim/             # 在 PC 上做语法检查用的 Arduino.h 替身（Arduino 不编译它）
```

## `src/` 里的文件是生成的 —— 不要手改

```bash
python tools/amalgamate.py --out-dir dist   # 重新生成单文件版
cp dist/jsdk_can_amalgam.h dist/jsdk_can_amalgam.c arduino/src/
```

`tools/arduino_smoke.sh` 会检查"`arduino/src/` 与 `dist/` 是否一致"，所以不会出现
"库里的代码比 src/ 旧"这种静默漂移。

**为什么是"纯核心"而不是全部源码**：Arduino 只编译 `src/` 下的源文件，
而 PC 后端（socketcan/pcan）与 slcan 都依赖操作系统接口，MCU 上没意义。
MCU 客户本来就该自己实现 HAL 三回调（见下面 sketch）。

生成的单文件版依赖 `<string.h>` / `<math.h>` / `<stdio.h>` / `<stdarg.h>` /
`<stdint.h>` / `<stddef.h>`。AVR 上 `snprintf` 会带来约 1.5 KB 代码体积 ——
如果 Flash 紧张，可以在你的工程里用 `-Wl,--wrap` 或自备精简实现替换掉。

## 用法（Arduino IDE）

1. 把整个 `arduino/` 拷成 `~/Arduino/libraries/CyberBeastJointSDK/`；
2. 打开 `examples/01_mit_move`；
3. 按你的板子填 `can_hal_impl.h` 里的三个回调（MCP2515 / ESP32 TWAI / FDCAN …），
   以及 `kJson`（端点描述符）；
4. 编译、烧写。

## 端点描述符怎么办

MCU 上**不建议**通过 CAN 下载 41 KB 描述符（慢、占 RAM）。推荐：

```bash
# 从固件仓库导出权威端点表（含 mit_max_*），再按需裁剪
python tools/extract_endpoints_json.py --hpp ../ODrive/Firmware/autogen/endpoints.hpp
# 然后用 RETAIN_FILTERED + 显式 filter_paths 只留自己用到的十几条路径
```

把结果作为常量数组（`const char kJson[] = "...";`）用
`jsdk_context_desc_import_raw()` 喂进去即可 —— 示例 07（`examples/07_custom_hal.c`）
演示的就是这条路线，实测 arena 只需 **33 字节**（3 条路径）。

## PlatformIO

`library.properties` 会被 PlatformIO 读取，或直接写进 `platformio.ini`：

```ini
lib_deps = file:///path/to/JointSDK/arduino
```

⚠ **还没在真 Arduino 工具链上编译过**（本仓库的 CI 没有 arduino-cli）。
`tools/arduino_smoke.sh` 做的是：`src/` 新鲜度检查 + 用 PC 上的 g++（配
`extras/host_shim/`）把 sketch 的**逻辑**编过一遍，能挡住拼写/API 误用这类错误，
但**挡不住 AVR 特有的问题**（`int` 宽度、`snprintf` 体积、中断上下文里的调用约定）。
第一次上板请预留调试时间。
