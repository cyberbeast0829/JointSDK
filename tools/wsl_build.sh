#!/usr/bin/env bash
# 在 WSL Ubuntu 里构建/测试 JointSDK，并把输出整理成可读形式。
#
# 为什么需要这个脚本：
#   1. **Linux 是首要目标平台之一，但开发机是 Windows** —— `src/hal/hal_socketcan.c`
#      只在 Linux 上编译，历史上"从未被编译器看过"（DESIGN v0.17 记了它藏了什么）。
#   2. 从 Windows 的 bash 直接 `wsl.exe ... | grep` 会被 wsl.exe 的 UTF-16 提示
#      信息 + GBK 解码弄成 "Binary file matches"，输出完全不可读 —— 所以统一
#      「WSL 内跑 → 写日志 → Python 按 UTF-8 读」。
#
# 用法（Windows bash，工作目录 = 仓库根）：
#   ./tools/wsl_build.sh configure [额外 cmake 参数...]
#   ./tools/wsl_build.sh build
#   ./tools/wsl_build.sh ctest
#   ./tools/wsl_build.sh all              # configure + build + ctest
#   ./tools/wsl_build.sh asan             # ASAN+UBSan 全套（**内存问题的唯一可靠防线**）
#   ./tools/wsl_build.sh pcan             # 强制编 PCAN 后端（Linux 下默认 OFF，否则永不编译；见下）
#
# ⚠ 为什么需要 `pcan`：`hal_pcan.c` 在 Linux 上**默认不编**（默认 OFF），而 Windows 上
#   只会编 `LoadLibrary` 那半（`#ifdef _WIN32`）—— 于是那个 `dlopen` 分支（含
#   `libPCBUSB.dylib` 等 macOS 名字）**在哪都没被编译器看过**。与 A9 同一个坑，
#   一条命令就能堵上：编一遍、链一遍（`-ldl` 传递）、跑一遍。
#
# 可用环境变量覆盖：JSDK_WSL_SRC / JSDK_WSL_BUILD / JSDK_WSL_LOG / JSDK_WSL_DISTRO
set -uo pipefail

ACTION="${1:-all}"
shift || true

SRC="${JSDK_WSL_SRC:-/mnt/d/projects/cheetah/JointSDK}"
BUILD="${JSDK_WSL_BUILD:-$SRC/build-linux}"
LOG="${JSDK_WSL_LOG:-/tmp/jsdk_wsl.log}"
DISTRO="${JSDK_WSL_DISTRO:-Ubuntu-20.04}"

run_in_wsl() {          # $1 = 要执行的 bash 片段
    wsl.exe -d "$DISTRO" -u root -e bash -lc "$1"
}

# ⚠ Ubuntu 20.04 自带 cmake 3.16，**没有** `ctest --test-dir`（3.20 才有）→ 必须 cd 进构建目录
do_configure() {
    run_in_wsl "cd '$SRC' && cmake -S . -B '$BUILD' -DJSDK_WERROR=ON -DJSDK_ENABLE_HEAP=ON $*"
}

do_build() {
    run_in_wsl "cd '$SRC' && cmake --build '$BUILD' -j4"
}

do_ctest() {
    run_in_wsl "cd '$BUILD' && ctest --output-on-failure"
}

# ASAN/UBSan 构建：这是发现**内存问题**（悬垂指针 / 越界 / 用后释放）的唯一可靠手段。
# 实测它抓到了 CLI 里"配置结构体放在局部变量、SDK 之后回写 arena_used"造成的
# stack-buffer-underflow（见 DESIGN v0.17）—— 那种问题在 Windows/普通构建下完全静默。
do_asan() {
    run_in_wsl "cd '$SRC' && rm -rf build-asan && \
        cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug \
              -DCMAKE_C_FLAGS='-fsanitize=address,undefined -fno-omit-frame-pointer -g' \
              -DJSDK_WERROR=ON -DJSDK_ENABLE_HEAP=ON > /dev/null && \
        cmake --build build-asan -j4 && \
        cd build-asan && ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=print_stacktrace=1 \
        ctest --output-on-failure"
}

do_pcan() {
    run_in_wsl "cd '$SRC' && rm -rf build-pcan && \
        cmake -S . -B build-pcan -DJSDK_BUILD_HAL_PCAN=ON -DJSDK_WERROR=ON \
              -DJSDK_BUILD_TESTS=ON -DJSDK_BUILD_HAL_VIRTUAL=ON \
              -DJSDK_ENABLE_HEAP=ON > /dev/null && \
        cmake --build build-pcan -j4 && \
        cd build-pcan && ctest --output-on-failure"
}

{
    case "$ACTION" in
        configure) do_configure "$@" ;;
        build)     do_build ;;
        ctest)     do_ctest ;;
        asan)      do_asan ;;
        pcan)      do_pcan ;;
        all)       do_configure "$@" && do_build && do_ctest ;;
        *)         echo "未知动作：$ACTION（可用：configure/build/ctest/asan/pcan/all）" >&2; exit 2 ;;
    esac
    echo "rc=$?"
} > "$LOG" 2>&1

python - "$LOG" <<'PY'
import io, sys
text = io.open(sys.argv[1], encoding='utf-8', errors='replace').read()
keep = [l for l in text.splitlines() if not l.startswith('wsl:')]
out = '\n'.join(l.rstrip() for l in keep if l.strip() != '')
# ⚠ 必须**直接写 UTF-8 字节**：
#   Windows 控制台默认编码是 GBK，而 WSL 的输出里可能出现 U+FFFD（替换字符）——
#   `print()` 要把它编码成 GBK 时会抛 UnicodeEncodeError，整个脚本挂掉
#   （现象是"构建结果一个字都看不到"，看着像构建失败，其实是输出管道崩了）。
sys.stdout.buffer.write(out.encode('utf-8', 'replace'))
sys.stdout.buffer.write(b'\n')
PY
