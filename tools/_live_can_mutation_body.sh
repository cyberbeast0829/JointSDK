#!/usr/bin/env bash
# 见 tools/live_can_mutation_test.sh 的说明。这个文件在 **WSL 里** 执行（不是 Windows bash），
# 拆成两个文件是为了避免 `wsl.exe bash -lc '...'` 的引号地狱（里层没法用单引号）。
set -uo pipefail

cd "${JSDK_WSL_SRC:-/mnt/d/projects/cheetah/JointSDK}"
SRC_FILE=src/hal/hal_socketcan.c
BAK=/tmp/hal_socketcan.bak
BUILD=build-live
LOG=/tmp/mut_build.log

# --- 1) 真链路准备（必须与测试在同一次调用里；WSL2 会关掉 VM） ---
modprobe can 2>/dev/null
modprobe can-raw 2>/dev/null
modprobe vcan 2>/dev/null
ip link del vcan0 2>/dev/null || true
ip link del vcan1 2>/dev/null || true
# ⚠ MTU 必须**在建接口时**就给（`ip link set vcan0 mtu 72` 在 up 之后会 EBUSY /
#   静默保留 16 → FD 用例会以"链路不支持 FD"的形式失败，看着像代码坏了）
ip link add dev vcan0 type vcan mtu 72
ip link set vcan0 up
ip link add dev vcan1 type vcan mtu 72
ip link set vcan1 mtu 16 || echo '  (注意：本内核不允许 vcan 降到 MTU 16，用例 6 会被跳过)'
ip link set vcan1 up
echo "内核：$(uname -r)"
ip -br link show | grep -E '^vcan'

cp "$SRC_FILE" "$BAK"

# --- 2) 工具函数 ---
# 用 python 做**字面量**替换：`sed` 的替换段里 `&` 有特殊含义（`&&` = 两遍匹配文本），
# 在 `wsl.exe bash -lc '...'` 的双层引用里极易被静默改坏 —— 实测让变异变成空操作，
# 于是"未被检出"的假结论。python 的 str.replace 没有这些陷阱，并且能断言"恰好命中一次"。
apply_mutation() {   # $1 = 旧串  $2 = 新串
    python3 - "$SRC_FILE" "$1" "$2" <<'PY'
import io, sys
path, old, new = sys.argv[1], sys.argv[2], sys.argv[3]
s = io.open(path, encoding='utf-8').read()
n = s.count(old)
if n != 1:
    sys.stderr.write('变异无效：旧串命中 %d 次（应为 1）\n' % n)
    sys.exit(3)
io.open(path, 'w', encoding='utf-8').write(s.replace(old, new))
PY
}

run_case() {         # $1 = 标签
    cp "$BAK" "$SRC_FILE"
    if ! apply_mutation "$2" "$3"; then
        printf '  %-34s %s\n' "$1" "跳过（变异无效）"
        return
    fi
    if ! cmake --build "$BUILD" -j4 --target test_socketcan_live >"$LOG" 2>&1; then
        printf '  %-34s %s\n' "$1" "构建失败（变异语法错误，无效）"
        grep -m2 -E 'error' "$LOG"
        return
    fi
    local out
    out=$(JSDK_SC_IFACE=vcan0 JSDK_SC_IFACE_CLASSIC=vcan1 ./"$BUILD"/test_socketcan_live 2>&1)
    printf '  %-34s %s\n' "$1" "$(echo "$out" | tail -1)"
    echo "$out" | grep -E '^  FAIL' | head -3 | sed 's/^/      /'
}

# --- 3) 基线 + 4 个变异 ---
cp "$BAK" "$SRC_FILE"
cmake --build "$BUILD" -j4 --target test_socketcan_live >"$LOG" 2>&1
printf '  %-34s %s\n' "基线（期望 0 失败）" \
    "$(JSDK_SC_IFACE=vcan0 JSDK_SC_IFACE_CLASSIC=vcan1 ./"$BUILD"/test_socketcan_live 2>&1 | tail -1)"

run_case "M1 不发 CAN_RAW_FD_FRAMES" \
    '    if (want_fd) {
        int on = 1;
        if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES' \
    '    if (0 && want_fd) {
        int on = 1;
        if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES'

run_case "M2 bind 读 union（原缺陷）" \
    'addr.can_ifindex = ifindex;' \
    'addr.can_ifindex = ifindex * 0 + ifr.ifr_ifindex;'

run_case "M3 不置 CANFD_FDF" \
    'cf.flags = (uint8_t)(CANFD_FDF' \
    'cf.flags = (uint8_t)(0u * CANFD_FDF'

run_case "M4 LOOPBACK=0 代替 RECV_OWN_MSGS" \
    'setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &own, sizeof own)' \
    'setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &own, sizeof own)'

# --- 4) 还原并确认 ---
cp "$BAK" "$SRC_FILE"
cmake --build "$BUILD" -j4 --target test_socketcan_live >"$LOG" 2>&1
printf '  %-34s %s\n' "还原后（期望 0 失败）" \
    "$(JSDK_SC_IFACE=vcan0 JSDK_SC_IFACE_CLASSIC=vcan1 ./"$BUILD"/test_socketcan_live 2>&1 | tail -1)"
ip link del vcan0 2>/dev/null || true
ip link del vcan1 2>/dev/null || true
if diff -q "$BAK" "$SRC_FILE" >/dev/null; then echo "  源文件已完整还原"; else echo "  ⚠ 源文件未还原！"; fi
