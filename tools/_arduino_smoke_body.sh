#!/usr/bin/env bash
# 见 tools/arduino_smoke.sh 的说明。在 **WSL 里**执行。
set -uo pipefail

cd "${JSDK_WSL_SRC:-/mnt/d/projects/cheetah/JointSDK}"

WORK=/tmp/jsdk-arduino
FAIL=0

note() { printf '\n--- %s ---\n' "$1"; }
rm -rf "$WORK"; mkdir -p "$WORK"

note "1) arduino/src/ 与 dist/ 是否一致（防陈旧）"
python3 tools/amalgamate.py --out-dir "$WORK/dist" > /dev/null || { echo "生成失败"; exit 1; }
for f in jsdk_can_amalgam.h jsdk_can_amalgam.c; do
    if [ ! -f "arduino/src/$f" ]; then
        echo "  ✗ arduino/src/$f 不存在（跑：cp dist/$f arduino/src/）"; FAIL=1
    elif cmp -s "arduino/src/$f" "$WORK/dist/$f"; then
        echo "  ✓ $f 一致（$(stat -c%s "arduino/src/$f") 字节）"
    else
        echo "  ✗ arduino/src/$f 与 dist/ 不一致 —— 库里的代码比 src/ 旧。请："
        echo "      python tools/amalgamate.py && cp dist/jsdk_can_amalgam.* arduino/src/"
        FAIL=1
    fi
done

note "2) library.properties 必需字段"
for k in name version author maintainer sentence paragraph category url architectures; do
    if grep -q "^$k=" arduino/library.properties; then
        printf '  ✓ %-14s %s\n' "$k" "$(grep "^$k=" arduino/library.properties | head -c 60)"
    else
        echo "  ✗ 缺字段：$k"; FAIL=1
    fi
done

note "3) sketch 逻辑语法检查（g++ + host_shim，不是真实目标编译）"
# 把 .ino 当成 C++ 编（Arduino 就是这么干的：先拼接成 .cpp）。
cat > "$WORK/sketch.cpp" <<'EOF'
#include "Arduino.h"
#include "jsdk_can_amalgam.h"
EOF

# 把 .ino 的内容拼进去（去掉 .ino 特有的东西：本例没有），并给 setup/loop 声明
cat arduino/examples/01_mit_move/01_mit_move.ino >> "$WORK/sketch.cpp"

# sketch 里用到的 kJson 定义在 .ino 末尾；setup/loop 需要前置声明（Arduino 会自动做）
cat >> "$WORK/sketch.cpp" <<'EOF'

int main()
{
    setup();
    for (int i = 0; i < 5; ++i) loop();
    return 0;
}
EOF

if g++ -std=c++14 -Wall -Wextra \
        -Iarduino/extras/host_shim \
        -Iarduino/examples/01_mit_move \
        -I"$WORK/dist" \
        -c "$WORK/sketch.cpp" -o "$WORK/sketch.o" 2> "$WORK/sketch.log"; then
    echo "  ✓ sketch 编过（host_shim + 单文件版）"
else
    echo "  ✗ sketch 编译失败："
    grep -E "error" "$WORK/sketch.log" | head -12 | sed 's/^/    /'
    FAIL=1
fi
# 注意：这里用 `-c`（不链接）—— host_shim 的 millis()/Serial 只是替身，
# 真实目标是"逻辑能不能编过"，链接与上板行为属于真实工具链。

if command -v arduino-cli > /dev/null 2>&1; then
    note "4) arduino-cli 存在：尝试真实目标编译"
    arduino-cli compile --library arduino --fqbn arduino:avr:uno \
        arduino/examples/01_mit_move > /dev/null 2>&1 \
        && echo "  ✓ arduino-cli 编译通过（avr:uno）" \
        || { echo "  ✗ arduino-cli 编译失败（AVR 特有问题？）"; FAIL=1; }
else
    note "4) 没有 arduino-cli → 跳过真实目标编译"
    echo "  ⚠ 这意味着 AVR/ESP32 特有的问题**未验证**（见 arduino/README.md 的说明）"
fi

rm -rf "$WORK"
printf '\n'
if [ "$FAIL" = 0 ]; then echo "=== Arduino 冒烟：全部通过 ==="; else echo "=== Arduino 冒烟：有失败 ==="; fi
exit "$FAIL"
