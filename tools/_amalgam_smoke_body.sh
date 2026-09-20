#!/usr/bin/env bash
# 见 tools/amalgam_smoke.sh 的说明。在 **WSL 里**执行。
set -uo pipefail

cd "${JSDK_WSL_SRC:-/mnt/d/projects/cheetah/JointSDK}"

WARN="-std=c99 -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Werror"
WORK=/tmp/jsdk-amalgam
FAIL=0

note() { printf '\n--- %s ---\n' "$1"; }

rm -rf "$WORK"; mkdir -p "$WORK"

note "1) 重新生成并检查 dist/ 是否过期"
python3 tools/amalgamate.py --out-dir "$WORK/core" > /dev/null || { echo "生成失败"; exit 1; }
python3 tools/amalgamate.py --with virtual --out-dir "$WORK/virt" > /dev/null || { echo "生成失败"; exit 1; }

if [ -d dist ]; then
    if diff -q "$WORK/core/jsdk_can_amalgam.c" dist/jsdk_can_amalgam.c > /dev/null 2>&1 \
    && diff -q "$WORK/core/jsdk_can_amalgam.h" dist/jsdk_can_amalgam.h > /dev/null 2>&1; then
        echo "  ✓ dist/ 与当前 src/ 一致（未过期）"
    else
        echo "  ✗ dist/ 已过期 —— 客户的单文件版是旧代码。请重跑："
        echo "      python tools/amalgamate.py"
        FAIL=1
    fi
else
    echo "  ⚠ 仓库里没有 dist/（跳过）；生成命令：python tools/amalgamate.py"
fi

note "2) 纯核心变体：严格告警 + -Werror（MCU 客户拿到的是这个）"
if gcc $WARN -c "$WORK/core/jsdk_can_amalgam.c" -o "$WORK/core.o" 2> "$WORK/core.log"; then
    echo "  ✓ 编译通过（$(stat -c%s "$WORK/core.o") 字节目标文件）"
    echo "    依赖的系统头：$(grep -o '^#include <[a-z.]*>' "$WORK/core/jsdk_can_amalgam.c" | tr '\n' ' ')"
    if grep -q "stdlib.h\|unistd.h\|sys/" "$WORK/core/jsdk_can_amalgam.c"; then
        echo "  ✗ 纯核心里出现了操作系统头（MCU 侧会有问题）"; FAIL=1
    else
        echo "  ✓ 不含任何 OS 头（stdlib/unistd/sys/*）"
    fi
else
    echo "  ✗ 编译失败："; tail -20 "$WORK/core.log"; FAIL=1
fi

note "3) 带虚拟后端的变体 + 同一个客户程序（即插即用替代）"
gcc -std=c99 -Wall -Wextra -Werror -c "$WORK/virt/jsdk_can_amalgam.c" \
    -o "$WORK/virt/jsdk_can_amalgam.o" || { echo "  ✗ 变体编译失败"; exit 1; }
# 把单文件版伪装成原来的两个公开头（见 tests/packaging/amalgam_shim/）：
# 客户程序一个字都不用改，只调整 include 顺序。
if gcc -std=c99 -Wall -Wextra -Werror \
        -Itests/packaging/amalgam_shim -I"$WORK/virt" \
        tests/packaging/consumer.c "$WORK/virt/jsdk_can_amalgam.c" \
        -lm -o "$WORK/consumer" 2> "$WORK/consumer.log"; then
    echo "  ✓ 编译/链接通过（只用了两个文件 + -lm）"
    if "$WORK/consumer" > "$WORK/consumer.out" 2>&1; then
        sed 's/^/    /' "$WORK/consumer.out"
    else
        echo "  ✗ 运行失败："; sed 's/^/    /' "$WORK/consumer.out"; FAIL=1
    fi
else
    echo "  ✗ 编译失败："; tail -20 "$WORK/consumer.log"; FAIL=1
fi

note "4) C++ 客户（真实用法：.cpp 编成 C++，实现仍由 C 编译器编）"
#
# ⚠ 不要"把实现文件按 C++ 编" —— 那不是受支持的用法：实现是 C99，里面用了
#   `offsetof(struct { char c; T t; }, t)` 这类 C 专用技巧（C++ 里非法，
#   见 src/core/jsdk_text.c 的 JSDK_ALIGNOF）。真实客户是：自己的 .cpp 包含
#   头文件（靠 extern "C" 守卫），SDK 的 .c 照旧由 C 编译器编成目标文件，
#   最后用 C++ 链接。下面复刻的就是它。
if g++ -std=c++14 -Wall -Wextra -Werror -x c++ \
        -Itests/packaging/amalgam_shim -I"$WORK/virt" \
        -c tests/packaging/consumer.c -o "$WORK/consumer_cpp.o" 2> "$WORK/cpp.log"; then
    echo "  ✓ 客户 TU 按 C++ 编过（头文件的 extern \"C\" 守卫有效）"
    if g++ "$WORK/consumer_cpp.o" "$WORK/virt/jsdk_can_amalgam.o" -lm \
            -o "$WORK/consumer_cpp" 2> "$WORK/cpp_link.log"; then
        if "$WORK/consumer_cpp" > "$WORK/cpp.out" 2>&1; then
            echo "  ✓ C++ 客户链接 + 运行通过：$(tail -1 "$WORK/cpp.out")"
        else
            echo "  ✗ C++ 客户运行失败："; sed 's/^/    /' "$WORK/cpp.out"; FAIL=1
        fi
    else
        echo "  ✗ C++ 链接失败（extern \"C\" 没生效？）："; tail -12 "$WORK/cpp_link.log"; FAIL=1
    fi
else
    echo "  ✗ 客户 TU 按 C++ 编译失败："; tail -20 "$WORK/cpp.log"; FAIL=1
fi

rm -rf "$WORK"
printf '\n'
if [ "$FAIL" = 0 ]; then echo "=== amalgamation 冒烟：全部通过 ==="; else echo "=== amalgamation 冒烟：有失败 ==="; fi
exit "$FAIL"
