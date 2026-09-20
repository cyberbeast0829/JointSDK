#!/usr/bin/env bash
# 见 tools/packaging_smoke.sh 的说明。这个文件在 **WSL 里**执行（不是 Windows bash），
# 拆成两个文件是为了避免 `wsl.exe bash -lc '...'` 的引号问题（里层没法用单引号）。
set -uo pipefail

cd "${JSDK_WSL_SRC:-/mnt/d/projects/cheetah/JointSDK}"

BUILD=build-install
PREFIX=/tmp/jsdk-prefix
WORK=/tmp/jsdk-consumer
FAIL=0

note() { printf '\n--- %s ---\n' "$1"; }

note "1) 构建并按前缀安装"
rm -rf "$BUILD" "$PREFIX" "$WORK"
cmake -S . -B "$BUILD" -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DJSDK_WERROR=ON -DJSDK_ENABLE_HEAP=ON -DJSDK_BUILD_SHARED=ON \
      -DCMAKE_INSTALL_PREFIX="$PREFIX" > /tmp/pkg_cfg.log 2>&1 || {
    echo "configure 失败"; tail -20 /tmp/pkg_cfg.log; exit 1; }
cmake --build "$BUILD" -j4 > /tmp/pkg_build.log 2>&1 || {
    echo "构建失败"; grep -m5 -E "error|warning" /tmp/pkg_build.log; exit 1; }
cmake --install "$BUILD" > /tmp/pkg_install.log 2>&1 || {
    echo "安装失败"; tail -20 /tmp/pkg_install.log; exit 1; }
echo "安装树："
find "$PREFIX" \( -type f -o -type l \) | sed "s|$PREFIX|<prefix>|" | sort

note "2) pkg-config 路径"
export PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig"
if ! pkg-config --exists joint-sdk-can; then
    echo "  ✗ pkg-config 找不到 joint-sdk-can"; FAIL=1
else
    echo "  version : $(pkg-config --modversion joint-sdk-can)"
    echo "  cflags  : $(pkg-config --cflags joint-sdk-can)"
    echo "  libs    : $(pkg-config --libs joint-sdk-can)"
    echo "  static  : $(pkg-config --static --libs joint-sdk-can)"
fi

mkdir -p "$WORK"
cp tests/packaging/consumer.c "$WORK/"

note "3a) 消费者 A：gcc + pkg-config（动态链接）"
# 动态链接：`Libs:` 就够（-L + -ljsdk_can）。运行时需要 LD_LIBRARY_PATH —— 这是
# 共享库的常识，不是打包缺陷；写在这里是为了让下一个人不用再猜。
if gcc -std=c99 -Wall -Wextra -Werror "$WORK/consumer.c" \
        $(pkg-config --cflags joint-sdk-can) \
        $(pkg-config --libs joint-sdk-can) \
        -o "$WORK/consumer_dyn" 2> /tmp/pkg_cc.log; then
    if LD_LIBRARY_PATH="$PREFIX/lib" "$WORK/consumer_dyn" > "$WORK/dyn.out" 2>&1; then
        echo "  运行输出："; sed 's/^/    /' "$WORK/dyn.out"
    else
        echo "  ✗ 运行失败："; sed 's/^/    /' "$WORK/dyn.out"; FAIL=1
    fi
    # 确认拿到的确实是那个共享库（而不是静态库被偷偷选中）
    if readelf -d "$WORK/consumer_dyn" 2>/dev/null | grep -q 'NEEDED.*libjsdk_can'; then
        echo "  ✓ 动态依赖 libjsdk_can.so"
    else
        echo "  ⚠ 没有动态依赖 libjsdk_can（链接器可能选了静态库）"
    fi
else
    echo "  ✗ 编译/链接失败："; tail -15 /tmp/pkg_cc.log; FAIL=1
fi

note "3b) 消费者 A：gcc + pkg-config（静态链接，验证 Libs.private）"
# 静态链接：`-Wl,-Bstatic` 钉住 libjsdk_can，再补 Libs.private 给的 -lm/-ldl。
# 这一步专门抓"静态库漏声明依赖"（v0.17 的 -lm 缺失就是这么漏了很久的）。
if gcc -std=c99 -Wall -Wextra -Werror "$WORK/consumer.c" \
        $(pkg-config --cflags joint-sdk-can) \
        -L"$PREFIX/lib" -Wl,-Bstatic -ljsdk_can -Wl,-Bdynamic \
        $(pkg-config --static --libs joint-sdk-can | sed 's/.*-ljsdk_can//') \
        -o "$WORK/consumer_static" 2> /tmp/pkg_cc_static.log; then
    if "$WORK/consumer_static" > "$WORK/static.out" 2>&1; then
        echo "  ✓ 静态链接的消费者无需 LD_LIBRARY_PATH 即可运行"
        tail -3 "$WORK/static.out" | sed 's/^/    /'
    else
        echo "  ✗ 运行失败："; sed 's/^/    /' "$WORK/static.out"; FAIL=1
    fi
else
    echo "  ✗ 静态链接失败（Libs.private 缺东西？）："; tail -15 /tmp/pkg_cc_static.log; FAIL=1
fi

note "4) 消费者 B：独立 CMake 工程 + find_package(jsdk_can)"
if cmake -S tests/packaging/cmake-consumer -B "$WORK/cmake-build" \
        -DCMAKE_PREFIX_PATH="$PREFIX" > /tmp/pkg_cm.log 2>&1; then
    grep -m1 "interface includes" /tmp/pkg_cm.log || true
    if cmake --build "$WORK/cmake-build" > /tmp/pkg_cm_build.log 2>&1; then
        if "$WORK/cmake-build/consumer" > "$WORK/cm.out" 2>&1; then
            echo "  运行输出："; sed 's/^/    /' "$WORK/cm.out"
        else
            echo "  ✗ 运行失败："; sed 's/^/    /' "$WORK/cm.out"; FAIL=1
        fi
    else
        echo "  ✗ 构建失败："; tail -15 /tmp/pkg_cm_build.log; FAIL=1
    fi
else
    echo "  ✗ find_package 失败："; tail -15 /tmp/pkg_cm.log; FAIL=1
fi

note "5) 负向检查：版本不兼容必须被拒（SameMinorVersion）"
if cmake -S tests/packaging/cmake-consumer -B "$WORK/cmake-bad" \
        -DCMAKE_PREFIX_PATH="$PREFIX" \
        -DJSDK_CONSUMER_REQUIRE_VERSION=0.2 > /tmp/pkg_bad.log 2>&1; then
    echo "  ✗ 竟然接受了 0.2（当前 0.1.0）—— SameMinorVersion 没生效？"; FAIL=1
else
    echo "  ✓ 0.2 被拒：$(grep -m1 -i 'version' /tmp/pkg_bad.log | head -c 120)"
fi

rm -rf "$WORK"
printf '\n'
if [ "$FAIL" = 0 ]; then echo "=== 打包冒烟：全部通过 ==="; else echo "=== 打包冒烟：有失败 ==="; fi
exit "$FAIL"
