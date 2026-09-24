#!/usr/bin/env bash
# 一键回归（BACKLOG 任务 2）：一条命令跑完**整个矩阵**，末尾给一张表 + 非 0 退出。
#
# 为什么需要它：
#   v0.31~v0.33 每轮都要手工敲十来条命令 —— Windows 三套工具链（MinGW 堆+CLI、
#   MinGW 共享库、MSVC）、WSL 三套（常规、ASan+UBSan、共享库）、Python 两个平台、
#   三个冒烟、三个静态守卫。**漏跑一条就会得出"全绿"的错误结论**，而本项目已经
#   吃过两次这个亏：
#     ① MSVC 构建失败，但之前跑过的 `ctest` 仍报 22/22（陈旧结果）；
#     ② 源文件内容改了却没变 mtime，`cmake --build` 静默跳过，跑的是旧二进制。
#   所以这里默认**先 touch 源码**（宁可全量重编，也不要假绿灯），`--no-touch` 可关。
#
# 用法（Windows 的 bash / Git-Bash，工作目录任意；不要在 WSL 里跑本脚本）：
#   ./tools/check_all.sh                 # 全部步骤（含 WSL 与 MSVC）
#   ./tools/check_all.sh --no-wsl        # 只跑 Windows 侧
#   ./tools/check_all.sh --no-msvc
#   ./tools/check_all.sh --only win-build,py-win
#   ./tools/check_all.sh --skip smoke-arduino
#   ./tools/check_all.sh --list          # 只列出步骤
#   ./tools/check_all.sh --no-touch      # 增量构建（快，但可能跑旧二进制）
#
# 环境变量：JSDK_JOBS（并行度，默认 4）、JSDK_WSL_DISTRO（默认 Ubuntu-20.04）
#
# 退出码：0 = 所有步骤通过；1 = 有步骤失败（汇总表里逐个标出并给日志路径）；2 = 用法/环境问题
#
# ⚠ 为什么结果写在文件里再读：Windows 的 bash 直接 `wsl.exe | grep` 会被 wsl.exe 的
#   UTF-16 提示信息 + GBK 解码弄成 "Binary file matches"（见 tools/wsl_build.sh）。
#   所以每一步都写日志，最后的表格由 Python 按 UTF-8 读日志/汇总文件生成。
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 2

LOG_DIR="$ROOT/build/check_all"
WIN_SUM="$LOG_DIR/win-summary.txt"
WSL_SUM="$LOG_DIR/wsl-summary.txt"
JOBS="${JSDK_JOBS:-4}"
DISTRO="${JSDK_WSL_DISTRO:-Ubuntu-20.04}"

ONLY=""; SKIP=""; DO_WSL=1; DO_MSVC=1; NO_TOUCH=0; LIST=0

while [ $# -gt 0 ]; do
    case "$1" in
        --only)      ONLY="${2:-}"; shift 2 ;;
        --skip)      SKIP="${2:-}"; shift 2 ;;
        --no-wsl)    DO_WSL=0; shift ;;
        --no-msvc)   DO_MSVC=0; shift ;;
        --no-touch)  NO_TOUCH=1; shift ;;
        --jobs|-j)   JOBS="${2:-4}"; shift 2 ;;
        --list)      LIST=1; shift ;;
        -h|--help)   sed -n '2,30p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "未知参数：$1（--help 看用法）" >&2; exit 2 ;;
    esac
done

ALL_STEPS=(win-build win-bsh msvc py-win smoke-amalgam smoke-arduino smoke-packaging
           lint-cli-text lint-api-docs abi-gap wsl)

step_desc() {
    case "$1" in
        win-build)       echo "MinGW：堆+CLI+tests（build）+ ctest" ;;
        win-bsh)         echo "MinGW：共享库+Python 绑定（bsh）+ ctest" ;;
        msvc)            echo "MSVC/VS2022：Release 构建 + ctest" ;;
        py-win)          echo "Python 用例（Windows，JSDK_LIB_PATH=bsh）" ;;
        smoke-amalgam)   echo "amalgamation 冒烟（客户只加两个文件那条路）" ;;
        smoke-arduino)   echo "Arduino 包装冒烟" ;;
        smoke-packaging) echo "安装/打包冒烟（临时前缀 + 3 个真实消费者）" ;;
        lint-cli-text)   echo "静态守卫：CLI 输出必须走 cli_text 且 CP936 可表示" ;;
        lint-api-docs)   echo "静态守卫：每个 JSDK_API 声明都要有 Doxygen" ;;
        abi-gap)         echo "静态守卫：Python 绑定覆盖全部公共 C API" ;;
        wsl)             echo "WSL：常规 + ASan + Linux 共享库 + Python（4 步）" ;;
        *)               echo "(未知)" ;;
    esac
}

selected() {
    local s="$1"
    if [ -n "$ONLY" ]; then
        case ",$ONLY," in *",$s,"*) ;; *) return 1 ;; esac
    fi
    if [ -n "$SKIP" ]; then
        case ",$SKIP," in *",$s,"*) return 1 ;; esac
    fi
    [ "$s" = "msvc" ] && [ "$DO_MSVC" = "0" ] && return 1
    [ "$s" = "wsl" ]  && [ "$DO_WSL"  = "0" ] && return 1
    return 0
}

if [ "$LIST" = "1" ]; then
    for s in "${ALL_STEPS[@]}"; do
        printf '%-16s %s%s\n' "$s" "$(step_desc "$s")" \
               "$(selected "$s" && echo '' || echo '   [本次不跑]')"
    done
    exit 0
fi

mkdir -p "$LOG_DIR"
# ⚠ 两份汇总都要**每次清空**：否则没跑 wsl 的那次会把上一轮的 WSL 行读进来，
#   表格里就会混进“上次的结果”，而它们可能早就不是当前代码的结论了（假绿灯）。
: > "$WIN_SUM"
: > "$WSL_SUM"

# ── 防"构建被静默跳过" ──────────────────────────────────────────────────────
touch_src() {
    [ "$NO_TOUCH" = "1" ] && return 0
    find src include tests examples tools -type f \
         \( -name '*.c' -o -name '*.h' -o -name '*.cpp' -o -name '*.hpp' \) \
         -exec touch {} + 2>/dev/null
    touch CMakeLists.txt 2>/dev/null
    return 0
}

# ── 配置：**沿用已有构建目录的生成器**（否则会得到 "generator does not match"） ──
configure_mingw() {
    local d="$1"; shift
    if [ -f "$d/CMakeCache.txt" ]; then
        cmake -S . -B "$d" "$@"
    else
        cmake -S . -B "$d" -G "MinGW Makefiles" "$@"
    fi
}
configure_vs() {
    local d="$1"; shift
    if [ -f "$d/CMakeCache.txt" ]; then
        cmake -S . -B "$d" "$@"
    else
        cmake -S . -B "$d" -G "Visual Studio 17 2022" -A x64 "$@"
    fi
}

detail_ctest() { grep -E "tests passed" "$1" 2>/dev/null | tail -1 | sed -E 's/^[[:space:]]*//'; }
detail_pytest() { grep -E "[0-9]+ passed|[0-9]+ failed" "$1" 2>/dev/null | tail -1 | sed -E 's/^[[:space:]]*//'; }
detail_err() {
    grep -m1 -E "error:|Error|undefined reference|No rule|FAILED|失败" "$1" 2>/dev/null \
        | cut -c1-160 || echo "(见日志)"
}

record() {  # 名称 rc 详情
    printf '%s|%s|%s\n' "$1" "$2" "${3//|//}" >> "$WIN_SUM"
}

run_step() {  # 名称 命令...
    local name="$1"; shift
    local log="$LOG_DIR/$name.log"
    printf '[%s] %s ... ' "$name" "$(step_desc "$name")"
    "$@" > "$log" 2>&1
    local rc=$?
    local d
    if [ $rc -eq 0 ]; then
        case "$name" in
            py-win)     d="$(detail_pytest "$log")" ;;
            abi-gap)    # ⚠ 别用 grep 判“缺口 0”：Windows 的 Python 默认按 cp936 写 stdout，
                        #   用 UTF-8 的中文去 grep 会**匹配不到**（而“匹配不到”看起来像失败）。
                        d="$(python - "$log" <<'PY2'
import io, re, sys
# ⚠ 直接写 UTF-8 字节：Windows 控制台/管道的默认编码是 cp936，走 print() 会把中文
#   变成乱码（tools/wsl_build.sh 里同一个坑，当时现象是“结果一个字都看不到”）。
import io as _io
t = _io.open(sys.argv[1], encoding='utf-8', errors='replace').read()
m = re.search(r'\u672a\u7ed1\u5b9a\uff08\u7f3a\u53e3\uff09\s*:\s*(\d+)', t)
s = '\u672a\u7ed1\u5b9a\uff08\u7f3a\u53e3\uff09: %s' % (m.group(1) if m else '?')
sys.stdout.buffer.write(s.encode('utf-8'))
PY2
)"
                        case "$d" in *": 0") ;; *) rc=1 ;; esac ;;
            lint-*)     d="$(tail -1 "$log")" ;;
            smoke-*)    d="$(detail_ctest "$log")"; [ -z "$d" ] && d="OK" ;;
            *)          d="$(detail_ctest "$log")" ;;
        esac
    else
        d="$(detail_err "$log")"
    fi
    [ $rc -eq 0 ] && echo "通过" || echo "失败"
    record "$name" "$rc" "$d"
    return 0
}
# ── 各步骤 ──────────────────────────────────────────────────────────────────
step_win_build() {
    touch_src
    configure_mingw build -DJSDK_WERROR=ON -DJSDK_ENABLE_HEAP=ON || return 1
    cmake --build build -j "$JOBS" || return 1
    ctest --test-dir build --output-on-failure -j "$JOBS"
}
step_win_bsh() {
    touch_src
    configure_mingw bsh -DJSDK_WERROR=ON -DJSDK_ENABLE_HEAP=ON \
        -DJSDK_BUILD_SHARED=ON -DJSDK_BUILD_PYTHON=ON || return 1
    cmake --build bsh -j "$JOBS" || return 1
    ctest --test-dir bsh --output-on-failure -j "$JOBS"
}
step_msvc() {
    touch_src
    configure_vs build-msvc -DJSDK_WERROR=ON -DJSDK_ENABLE_HEAP=ON \
        -DJSDK_BUILD_SHARED=ON -DJSDK_BUILD_PYTHON=ON || return 1
    cmake --build build-msvc --config Release -j "$JOBS" || return 1
    ctest --test-dir build-msvc -C Release --output-on-failure -j "$JOBS"
}
step_py_win() {
    ( cd bindings/python && JSDK_LIB_PATH=../../bsh PYTHONIOENCODING=utf-8 \
        python -m pytest tests/ -q )
}

echo "=== check_all：一键回归（日志在 $LOG_DIR/）==="

selected win-build       && run_step win-build       step_win_build
selected win-bsh         && run_step win-bsh         step_win_bsh
selected msvc            && run_step msvc            step_msvc
selected py-win          && run_step py-win          step_py_win
selected smoke-amalgam   && run_step smoke-amalgam   ./tools/amalgam_smoke.sh
selected smoke-arduino   && run_step smoke-arduino   ./tools/arduino_smoke.sh
selected smoke-packaging && run_step smoke-packaging ./tools/packaging_smoke.sh
selected lint-cli-text   && run_step lint-cli-text   env PYTHONIOENCODING=utf-8 python tools/check_cli_text.py .
selected lint-api-docs   && run_step lint-api-docs   env PYTHONIOENCODING=utf-8 python tools/check_api_docs.py
selected abi-gap         && run_step abi-gap         env PYTHONIOENCODING=utf-8 python tools/_abi_gap.py

# ── WSL 侧：一次 wsl.exe 调用里跑 4 步，结果写成机器可读文件 ────────────────
if selected wsl; then
    # WSL 里的源码路径。⚠ 三种写法都要认：
    #   /d/projects/...   （Windows 的 Git-Bash 给的 $PWD 就是这种 MSYS 形式）→ 加前缀 /mnt
    #   D:\projects\...    （若从别处传来原生形式）→ /mnt/d/projects/...
    #   /mnt/...          （已经在 WSL 里）→ 原样
    if [ -n "${JSDK_WSL_SRC:-}" ]; then
        WSL_SRC="$JSDK_WSL_SRC"
    else
        case "$ROOT" in
            /[A-Za-z]/*)      WSL_SRC="/mnt${ROOT}" ;;
            [A-Za-z]:[/\\]*)  WSL_SRC="/mnt/$(printf '%s' "$ROOT" | sed -E 's#^([A-Za-z]):#\L\1#; s#\\#/#g')" ;;
            *)                WSL_SRC="$ROOT" ;;
        esac
    fi
    printf '[%s] %s ... ' "wsl" "$(step_desc wsl)"
    : > "$WSL_SUM"
    # ⚠ 参数必须走 **argv**：Windows 的环境变量不会自动进入 WSL 的 Linux 环境
    #   （WSLENV 才行），所以不能靠 `VAR=x wsl.exe ...` 传进去。
    MSYS_NO_PATHCONV=1 MSYS2_ARG_CONV_EXCL='*' \
      wsl.exe -d "$DISTRO" -u root -e bash "$WSL_SRC/tools/_check_all_body.sh" \
      "$WSL_SRC" "$JOBS" "$NO_TOUCH" \
      > "$LOG_DIR/wsl.log" 2>&1
    # wsl.exe 的退出码只说明"主体跑完了"；每一步的真实结果在 wsl-summary.txt 里
    if [ -s "$WSL_SUM" ]; then echo "完成（4 步见下表）"; else echo "失败（见 wsl.log）"; fi
fi

# ── 汇总表 ─────────────────────────────────────────────────────────────────
python - "$WIN_SUM" "$WSL_SUM" "$LOG_DIR" <<'PY'
import io, os, sys
win_sum, wsl_sum, log_dir = sys.argv[1], sys.argv[2], sys.argv[3]
rows = []
for path in (win_sum, wsl_sum):
    if not os.path.exists(path):
        continue
    for line in io.open(path, encoding='utf-8', errors='replace').read().splitlines():
        parts = line.split('|', 2)
        if len(parts) == 3:
            rows.append(parts)

print()
print('=' * 78)
print('一键回归汇总')
print('=' * 78)
bad = 0
lines = []
for name, rc, detail in rows:
    ok = (rc == '0')
    bad += 0 if ok else 1
    lines.append('%-16s %-6s %s' % (name, '通过' if ok else '失败', detail[:110]))
lines.append('-' * 78)
lines.append('%d 步：%d 通过 / %d 失败' % (len(rows), len(rows) - bad, bad))
if bad:
    lines.append('')
    lines.append('失败步骤的日志：')
    for name, rc, detail in rows:
        if rc != '0':
            for cand in (os.path.join(log_dir, name + '.log'),
                         os.path.join(log_dir, 'wsl-' + name + '.log')):
                if os.path.exists(cand):
                    lines.append('  %-16s %s' % (name, cand))
    lines.append('')
    lines.append('提示：Windows 侧的 MINGW/MSVC 失败先看日志里第一条 error；')
    lines.append('      WSL 侧的四步共用 build/check_all/wsl.log 与 wsl-<步骤>.log。')

# ⚠ 直接写 UTF-8 字节：默认 stdout 在 Windows 上是 cp936（见 tools/wsl_build.sh）
sys.stdout.buffer.write(('\n'.join(lines) + '\n').encode('utf-8', 'replace'))
sys.exit(1 if bad else 0)
PY
exit $?
