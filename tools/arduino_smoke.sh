#!/usr/bin/env bash
# Arduino 包装冒烟（A5）：把"没编过的示例"这类风险降到最低。
#
# 做三件事：
#   1. `arduino/src/` 与 `dist/`（amalgamation 生成物）**逐字节一致** ——
#      否则库里的代码会悄悄落后于 src/；
#   2. `library.properties` 必需字段齐全（Arduino IDE/PlatformIO 靠它识别库）；
#   3. 用 PC 上的 g++ + `arduino/extras/host_shim/Arduino.h` 把 sketch 的**逻辑**编过 ——
#      挡住拼写/API 误用这类错误。
#
# ⚠ 明确**不**做的事：不用 arduino-cli 做真实目标编译（本环境没有），
#   所以 AVR/ESP32 特有问题（int 宽度、PROGMEM、snprintf 体积）仍要靠第一次上板验。
#
# 用法（Windows bash，工作目录 = 仓库根）：
#   ./tools/arduino_smoke.sh
set -uo pipefail

SRC="${JSDK_WSL_SRC:-/mnt/d/projects/cheetah/JointSDK}"
DISTRO="${JSDK_WSL_DISTRO:-Ubuntu-20.04}"

MSYS_NO_PATHCONV=1 MSYS2_ARG_CONV_EXCL='*' \
wsl.exe -d "$DISTRO" -u root -e bash "$SRC/tools/_arduino_smoke_body.sh"
