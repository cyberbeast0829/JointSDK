#!/usr/bin/env bash
# 打包冒烟：验证**安装出去**的东西能被正常使用（A2 pkg-config + A6 find_package）。
#
# 为什么必须"安装到临时前缀再编译一个消费者"：
#   构建树里的 include/ 与 src/ 都在同一个目录下，`-Iinclude/joint_sdk` 随便写都不会
#   出错；真正会出错的是**装完之后**的布局（`$<INSTALL_INTERFACE>` 拼错、
#   EXPORT_NAME 与 NAMESPACE 对不上、`.pc` 少了 `-lm`/`-ldl`、
#   公开头之间用引号互相引用却只给了一个 -I …）。这些只有装一遍才看得见。
#
# 用法（Windows bash，工作目录 = 仓库根，需要 WSL + root）：
#   ./tools/packaging_smoke.sh
set -uo pipefail

SRC="${JSDK_WSL_SRC:-/mnt/d/projects/cheetah/JointSDK}"
DISTRO="${JSDK_WSL_DISTRO:-Ubuntu-20.04}"

# ⚠ Git-Bash 会把 /mnt/... 当 Windows 路径去"翻译" → 关掉路径转换
MSYS_NO_PATHCONV=1 MSYS2_ARG_CONV_EXCL='*' \
wsl.exe -d "$DISTRO" -u root -e bash "$SRC/tools/_packaging_smoke_body.sh"
