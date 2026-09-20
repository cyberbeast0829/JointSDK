#!/usr/bin/env bash
# SocketCAN **真链路**冒烟：在 WSL 里建 vcan 接口，跑 tests/test_socketcan_live。
#
# 为什么需要它：src/hal/hal_socketcan.c 只在 Linux 上编译，而开发机是 Windows ——
# 在拿到真硬件之前，这是唯一能让这个后端**真的收发一次**的途径。它覆盖了离线
# 测试（test_hal.c）碰不到的东西：
#   - `jsdk_hal_socketcan_open()` 是否真能打开一个接口（socket/ifindex/MTU/bind）；
#   - CAN_RAW_FD_FRAMES 是否真的设上了、FD 帧双向是否字节一致；
#   - "要求 FD 但链路是 Classic(MTU=16)" 是否真的被拒（INVALID_ARG）。
# 实测价值：它抓到了 `struct ifreq` union 复用导致 bind 用了错误 ifindex、
# 整个后端**从未成功打开过任何接口**的缺陷（见 DESIGN v0.17）。
#
# ⚠ 必须在**同一次** WSL 调用里做完 modprobe + 建接口 + 跑测试：
#   WSL2 空闲后会关掉虚拟机，上一次调用加载的模块不会保留（表现是"刚才 modprobe
#   成功，这次 ip link add 却说 Unknown device type"）。
#
# 用法（Windows bash，工作目录 = 仓库根）：
#   ./tools/live_can_smoke.sh            # 用 vcan0 / vcan1
#   JSDK_LIVE_IFACE=can0 ./tools/live_can_smoke.sh   # 真硬件（需自行 ifconfig up + 终端电阻）
set -uo pipefail

SRC="${JSDK_WSL_SRC:-/mnt/d/projects/cheetah/JointSDK}"
DISTRO="${JSDK_WSL_DISTRO:-Ubuntu-20.04}"
IFACE="${JSDK_LIVE_IFACE:-vcan0}"
IFACE_CLASSIC="${JSDK_LIVE_IFACE_CLASSIC:-vcan1}"

# ⚠ 输出管道：wsl.exe 的启动提示里有 NUL/UTF-16 片段，直接 grep 会变成
#   "Binary file matches" —— 先 tr -d 掉 NUL，再用 grep -a 当文本处理。
wsl.exe -d "$DISTRO" -u root -e bash -lc "
set -e
cd '$SRC'

echo '--- 1) 准备接口 ---'
# ⚠ 三个模块缺一不可（顺序无关）：
#   can      : 注册 PF_CAN 地址族（缺它 socket(PF_CAN,...) 直接 EAFNOSUPPORT）
#   can-raw  : 注册 CAN_RAW 协议
#   vcan     : 提供链路类型（ip link add type vcan）
#   只 modprobe vcan 时，ip 能把 vcan0 建出来、但 socket() 一律失败 —— 本脚本
#   最初就是这么踩的（\"Address family not supported by protocol\"）。
modprobe can
modprobe can-raw
modprobe vcan

# vcan0 = FD 能力（MTU 72 = CANFD_MTU），vcan1 = 降成 Classic（MTU 16）
ip link del vcan0 2>/dev/null || true
ip link add dev vcan0 type vcan mtu 72
ip link set vcan0 up

ip link del vcan1 2>/dev/null || true
ip link add dev vcan1 type vcan mtu 72
ip link set vcan1 mtu 16 || echo '(注意：该内核不允许把 vcan 降到 MTU 16，用例 6 会被跳过)'
ip link set vcan1 up

ip -br link show | grep -E '^vcan' || true

echo
echo '--- 2) 构建 ---'
rm -rf build-live
cmake -S . -B build-live -DJSDK_WERROR=ON -DJSDK_BUILD_HAL_SOCKETCAN=ON \
      -DJSDK_LIVE_CAN_IFACE='$IFACE' > /dev/null
cmake --build build-live -j4 --target test_socketcan_live > /dev/null

echo
echo '--- 3) 真链路冒烟 ---'
JSDK_SC_IFACE='$IFACE' JSDK_SC_IFACE_CLASSIC='$IFACE_CLASSIC' \
    ./build-live/test_socketcan_live

echo
echo '--- 4) 清理 ---'
ip link del vcan0 2>/dev/null || true
ip link del vcan1 2>/dev/null || true
echo done
" 2>&1 | tr -d '\000' | grep -av '^wsl:'

exit "${PIPESTATUS[0]}"
