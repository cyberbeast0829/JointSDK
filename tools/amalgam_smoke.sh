#!/usr/bin/env bash
# amalgamation 冒烟（A3）：证明"客户只加两个文件"这条路真的能编译、能跑。
#
# 验的几件事（每一件都对应一个真会踩的坑）：
#   1. **纯核心**变体在 gcc 9 的严格告警 + `-Werror` 下能编过（MCU 客户拿到的是这个）；
#   2. **带虚拟后端**的变体能让**同一个客户程序**（tests/packaging/consumer.c）
#      编过并跑通 —— 即"合并件 = 公开头的即插即用替代"（靠 amalgam_shim/ 伪装）；
#   3. 生成物**没有过期**：重新生成后与 dist/ 里的逐字节相同
#      （否则仓库里的 dist/ 会悄悄落后于 src/，客户拿到的是旧代码）。
#
# 用法（Windows bash，工作目录 = 仓库根）：
#   ./tools/amalgam_smoke.sh
set -uo pipefail

SRC="${JSDK_WSL_SRC:-/mnt/d/projects/cheetah/JointSDK}"
DISTRO="${JSDK_WSL_DISTRO:-Ubuntu-20.04}"

MSYS_NO_PATHCONV=1 MSYS2_ARG_CONV_EXCL='*' \
wsl.exe -d "$DISTRO" -u root -e bash "$SRC/tools/_amalgam_smoke_body.sh"
