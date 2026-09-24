#!/usr/bin/env bash
# check_all.sh 的 **WSL 侧主体**（不要在 Windows 的 bash 里直接跑）。
#
# 为什么拆成 `_*_body.sh`：Windows 的 bash 直接 `wsl.exe ... | grep` 会被 wsl.exe
# 的 UTF-16 提示信息 + GBK 解码弄成 "Binary file matches"，输出完全不可读
# （与 tools/wsl_build.sh 同一个坑）。所以这里把结果写成**机器可读的一行行**，
# 由 Windows 侧读文件汇总；详细输出留在各自的日志里。
#
# 输出：$SRC/build/check_all/wsl-summary.txt —— 每行 `名称|rc|详情`
# 日志：$SRC/build/check_all/wsl-<名称>.log
#
# 四个步骤（都是**纯离线**，不需要任何硬件）：
#   linux      gcc + -Werror 的常规构建与 ctest（socketcan 后端只在这里被编译器看到）
#   asan       ASan + UBSan 全套（内存问题的唯一可靠防线，见 wsl_build.sh 的说明）
#   bsh-linux  Linux 共享库（**故意不带 JSDK_BUILD_PYTHON**：那条路会把包里的
#              .dll/.so 互相清掉，Linux 侧靠 JSDK_LIB_PATH 指到 bsh-linux）
#   py-linux   Python 绑定在 Linux 上跑同一套用例（与 Windows 结果必须一致）
set -uo pipefail

# 参数由 Windows 侧的 check_all.sh 以 **argv** 传入（不能用环境变量：Windows 的环境
# 变量不会自动进入 WSL 的 Linux 环境，除非走 WSLENV）。
SRC="${1:-/mnt/d/projects/cheetah/JointSDK}"
JOBS="${2:-4}"
NO_TOUCH="${3:-0}"
OUT="$SRC/build/check_all"

mkdir -p "$OUT"
SUM="$OUT/wsl-summary.txt"
: > "$SUM"

# ⚠ 防"构建被静默跳过"：文件内容改了但 mtime 没变时，make 会认为无需重建，
#   于是脚本报"通过"而跑的还是旧二进制（本项目真踩过：cp 恢复备份后 make 什么都没做）。
touch_src() {
    [ "$NO_TOUCH" = "1" ] && return 0
    find "$SRC/src" "$SRC/include" "$SRC/tests" "$SRC/examples" "$SRC/tools" \
         -type f \( -name '*.c' -o -name '*.h' -o -name '*.cpp' -o -name '*.hpp' \) \
         -exec touch {} + 2>/dev/null
    touch "$SRC/CMakeLists.txt"
}

detail_ctest() {   # ctest 日志 → "100% tests passed, 0 tests failed out of 21"
    grep -E "tests passed" "$1" 2>/dev/null | tail -1 | sed -E 's/^[[:space:]]*//'
}
detail_pytest() {
    grep -E "^[0-9]+ (passed|failed)|passed|failed|error" "$1" 2>/dev/null | tail -1 | sed -E 's/^[[:space:]]*//'
}
detail_err() {     # 失败时给一条最有信息量的行
    grep -m1 -E "error:|Error|undefined reference|No rule|FAILED" "$1" 2>/dev/null \
        | cut -c1-160 || echo ""
}

run() {            # 名称 命令...
    local name="$1"; shift
    local log="$OUT/wsl-$name.log"
    "$@" > "$log" 2>&1
    local rc=$?
    local d
    if [ $rc -eq 0 ]; then
        case "$name" in
            py-linux)  d="$(detail_pytest "$log")" ;;
            bsh-linux) d="构建通过（只有构建，无 ctest）" ;;
            *)         d="$(detail_ctest "$log")" ;;
        esac
    else
        d="$(detail_err "$log")"
    fi
    printf '%s|%s|%s\n' "$name" "$rc" "${d//|//}" >> "$SUM"
    return 0
}

step_linux() {
    cmake -S "$SRC" -B "$SRC/build-linux" -DJSDK_WERROR=ON -DJSDK_ENABLE_HEAP=ON || return 1
    cmake --build "$SRC/build-linux" -j"$JOBS" || return 1
    # ⚠ Ubuntu 20.04 自带 cmake 3.16，**没有** `ctest --test-dir`（3.20 才有）→ 必须 cd 进去
    ( cd "$SRC/build-linux" && ctest --output-on-failure -j"$JOBS" )
}

step_asan() {
    rm -rf "$SRC/build-asan"
    cmake -S "$SRC" -B "$SRC/build-asan" -DCMAKE_BUILD_TYPE=Debug \
          -DCMAKE_C_FLAGS='-fsanitize=address,undefined -fno-omit-frame-pointer -g' \
          -DJSDK_WERROR=ON -DJSDK_ENABLE_HEAP=ON || return 1
    cmake --build "$SRC/build-asan" -j"$JOBS" || return 1
    ( cd "$SRC/build-asan" && ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=print_stacktrace=1 \
        ctest --output-on-failure -j"$JOBS" )
}

step_bsh_linux() {
    cmake -S "$SRC" -B "$SRC/bsh-linux" -DJSDK_WERROR=ON -DJSDK_ENABLE_HEAP=ON \
          -DJSDK_BUILD_SHARED=ON -DJSDK_BUILD_PYTHON=OFF || return 1
    cmake --build "$SRC/bsh-linux" -j"$JOBS"
}

step_py_linux() {
    ( cd "$SRC/bindings/python" && JSDK_LIB_PATH="$SRC/bsh-linux" \
        python3 -m pytest tests/ -q )
}

touch_src
run linux     step_linux
run asan      step_asan
run bsh-linux step_bsh_linux
run py-linux  step_py_linux

echo "wsl 侧完成，汇总：$SUM"
