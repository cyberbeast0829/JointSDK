#!/usr/bin/env bash
# 变异测试：证明 tests/test_socketcan_live.c **真的有牙齿**。
#
# 为什么需要它：
#   `test_socketcan_live` 是唯一能验证 Linux 后端"真的收发过一帧"的测试，但它跑在
#   **内核**上 —— 所以必须先证明"故意把修复改回缺陷实现后它确实会失败"。否则它可能
#   只是打印了一堆 CHECK 而从未真正断言（这个项目已经踩过"用例恒真"的坑）。
#
# 用法（Windows bash，工作目录 = 仓库根，需要 WSL + root）：
#   ./tools/live_can_mutation_test.sh
#
# 每个变异 = 把当初的真实缺陷**原样改回去**，然后看冒烟是否失败：
#   M1 不设 CAN_RAW_FD_FRAMES      → FD 帧应发不出（✅ 实测检出，3 项失败）
#   M2 bind 读 union（用 MTU 当索引） → 应根本打不开接口（✅ 实测检出，8 项失败）
#   M3 不置 CANFD_FDF              → **vcan 测不出来**（❌ 见下）
#   M4 用 CAN_RAW_LOOPBACK=0       → 本地对端应看不见我们发的帧（✅ 实测检出，2 项失败）
#
# ⚠ **M3 的不可检出是一个必须记住的结论**：vcan 只把 skb 回环，不做真实收发器那层的
#   校验，所以"缺 `CANFD_FDF` 的 FD 帧"在 vcan 上照样能发/能收。也就是说
#   **`CANFD_FDF` 那一处修复无法用自动化证明** —— 它依赖内核 UAPI 契约（FD 帧必须置
#   FDF，Linux 5.11 起在发送侧强制），只能靠真硬件/真控制器验证。这条已写进
#   PORTING §7.5.3 的人工清单，不要因为"冒烟绿了"就以为它被覆盖了。
#
# ⚠ 变异用 python 做**字面量**替换而不是 `sed`：`sed` 替换段里的 `&` 有特殊含义
#   （`&` = 匹配文本、`&&` = 两遍匹配文本），在 `wsl.exe bash -lc '...'` 的双层引用里
#   会被静默改坏 → 变异变成空操作 → 拿到"未被检出"的**假结论**（本脚本第一版就这么错过）。
#   python 版还会断言"旧串恰好命中一次"，于是**同一份代码被重复插入**这类事故也会当场报出来
#   （实测就靠这条抓到了 FD_FRAMES 块被插了两遍）。
set -uo pipefail

SRC="${JSDK_WSL_SRC:-/mnt/d/projects/cheetah/JointSDK}"
DISTRO="${JSDK_WSL_DISTRO:-Ubuntu-20.04}"

# ⚠ Git-Bash / MSYS 会把 `/mnt/...` 这种参数当成 Windows 路径去"翻译"
#   （报 "D:/Program Files/Git/mnt/..." 找不到）→ 必须关掉路径转换。
MSYS_NO_PATHCONV=1 MSYS2_ARG_CONV_EXCL='*' \
    wsl.exe -d "$DISTRO" -u root -e bash "$SRC/tools/_live_can_mutation_body.sh"
