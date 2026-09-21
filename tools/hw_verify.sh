#!/usr/bin/env bash
# 关节链路自检：按**层次**逐条验证 jsdk-cli 与设备的链路，并给出可复现的结论。
#
# 为什么需要它（而不是"跑一次看到成功"）：
#   本项目踩过两次"我这边跑通了、用户那边报错"的坑，两次都不是幻觉：
#   ① slcan 适配器**打开端口后的头几帧会丢**（实测约 1/10 次进程），单跑一次
#      很容易恰好成功，于是"已真机验证"的结论掩盖了 1/10 的失败率；
#   ② 参数值字节序错时，`read` 仍然"成功返回"（只是数值是垃圾）——
#      只看退出码永远发现不了。
#   所以这里的规矩是：**每层独立起进程、重复 N 轮、并且对数值做交叉校验**。
#
# 层次（每层失败只说明那一层的问题，不会互相掩盖）：
#   L1 scan        节点发现（不下描述符）
#   L2 info        0x46 设备信息（单请求）
#   L3 desc-info   描述符下载（38 KB 流式；"首帧会不会丢"最先在这里暴露）
#   L4 hb-dump     **只收不发**的心跳——与请求通路无关，用来区分"通道没开"与"请求丢了"
#   L5 read        单参数读（范围 + 可选 --expect 精确值）
#   L6 batch-read  批量读，**必须与 L5 的同一个参数完全一致**（跨路径对拍）
#   L7 health      完整配置（含标定）+ 健康快照#   L8 write       **写路径**：读原值 → 写回同一个值 → 再读回必须相符（不动设备）
#   L8b write-probe 仅 `--write-probe`：写原值+Δ → 校验 → **无论成败都恢复原值**并校验
#
# ⚠ L8 会**真的往设备写**（默认目标 `axis0.config.can.heartbeat_rate_ms`，u32、非安全
#   关键、不落 Flash）。它默认只把读到的原值原样回写（幂等）；`--write-probe` 会真的
#   改一下数值再恢复 —— 那种情况下任何一步失败都会**立即尝试恢复**，恢复失败会大声报警。#
# 用法（仓库根目录，bash）：
#   ./tools/hw_verify.sh                                  # 默认 slcan + COM3 + node 1，跑 3 轮
#   ./tools/hw_verify.sh --channel COM5 --node 2 --runs 5
#   ./tools/hw_verify.sh --expect axis0.motor.config.gear_ratio=7.75   # 设备已知真值时钉死它
#   ./tools/hw_verify.sh --if virtual                      # 没有硬件时自检脚本本身
#
# ⚠ 依赖：**bash 4+**（用了关联数组 `declare -A`；macOS 自带 bash 3.2 不行，用 `brew install bash`）
#   与 python3/python（解析 `--json`；会**试跑一次**再选，避开 Git-Bash 里 Microsoft Store 的桩）。
#   `--expect` 的值是**具体设备的真值**，换设备要改（不传则只做范围与跨路径自洽检查）。
#
# 退出码：0 = 所有轮次所有层都通过；1 = 有失败（并打印各层失败详情）；2 = 环境/用法问题
set -uo pipefail

CLI="${JSDK_CLI:-build/jsdk-cli.exe}"
IFACE="slcan"
CHANNEL="COM3"
NODE="1"
RUNS=3
QUIET=0
EXPECTS=()
WRITE_PATH="axis0.config.can.heartbeat_rate_ms"
WRITE_DELTA=50
WRITE_PROBE=0

usage() {
    sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'
}

while [ $# -gt 0 ]; do
    case "$1" in
        --if)      IFACE="$2";   shift 2 ;;
        --channel) CHANNEL="$2"; shift 2 ;;
        --node)    NODE="$2";    shift 2 ;;
        --runs)    RUNS="$2";    shift 2 ;;
        --expect)  EXPECTS+=("$2"); shift 2 ;;
        --write-path)  WRITE_PATH="$2";  shift 2 ;;
        --write-delta) WRITE_DELTA="$2"; shift 2 ;;
        --write-probe) WRITE_PROBE=1; shift ;;
        --quiet)   QUIET=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "未知参数: $1"; echo; usage; exit 2 ;;
    esac
done

if [ ! -x "$CLI" ]; then
    echo "找不到可执行文件：$CLI"
    echo "先构建：cmake -S . -B build -G \"MinGW Makefiles\" -DJSDK_WERROR=ON -DJSDK_ENABLE_HEAP=ON && cmake --build build"
    echo "（或用 JSDK_CLI=/path/to/jsdk-cli 指定）"
    exit 2
fi

# ⚠ `--runs 0` 会让下面的循环一次都不跑，然后打印“全部通过（0 项检查）”——
#   那种“空跑成功”比报错更危险（本项目已经两次被“看着成功”骗到）。
case "$RUNS" in
    ''|*[!0-9]*) echo "--runs 需要正整数（收到：$RUNS）"; exit 2 ;;
esac
if [ "$RUNS" -lt 1 ]; then
    echo "--runs 至少为 1（收到：$RUNS）"; exit 2
fi

# python：Windows/Git-Bash 是 `python`，Ubuntu 只有 `python3`（可用 JSDK_PYTHON 覆盖）。
# ⚠ 必须**真的试跑一次**再选：Git-Bash 的 PATH 里 `python3` 常命中 Microsoft Store 的
#   桩程序（存在、但一跑就失败）—— 只查 `command -v` 会选中它，然后所有 JSON 解析
#   静默变空（表现为"所有值都读不到"，看着像 SDK 坏了）。
PY="${JSDK_PYTHON:-}"
if [ -z "$PY" ]; then
    for cand in python python3; do
        if command -v "$cand" >/dev/null 2>&1 && "$cand" -c 'import sys' >/dev/null 2>&1; then
            PY="$cand"; break
        fi
    done
fi
if [ -z "$PY" ]; then
    echo "需要可用的 python3 或 python（解析 --json 输出）"; exit 2
fi

BASE=("$CLI" --if "$IFACE")
if [ -n "$CHANNEL" ] && [ "$IFACE" != "virtual" ]; then
    BASE+=(--channel "$CHANNEL")
fi
BASE+=(--node "$NODE" --json)

#: 参与 L5/L6 对拍的参数（都在真机与仿真设备上存在）
PATHS=(
    "axis0.config.can.node_id"
    "axis0.config.can.heartbeat_rate_ms"
    "axis0.motor.config.gear_ratio"
    "axis0.motor.config.torque_constant"
)

TMP_OUT=$(mktemp)
TMP_ERR=$(mktemp)
trap 'rm -f "$TMP_OUT" "$TMP_ERR"' EXIT

declare -A LAYER_OK=()
declare -A LAYER_BAD=()
FAIL_LINES=()

ok()   { LAYER_OK[$1]=$(( ${LAYER_OK[$1]:-0} + 1 )); }
bad()  { LAYER_BAD[$1]=$(( ${LAYER_BAD[$1]:-0} + 1 ));
         FAIL_LINES+=("[$1] $2");
         [ "$QUIET" -eq 0 ] && printf '      [FAIL] %s: %s\n' "$1" "$2"; }

# 跑一条命令：RC / 输出分别落到 TMP_OUT、TMP_ERR（都不经过管道，保留真实退出码）
run_cli() {
    "${BASE[@]}" "$@" >"$TMP_OUT" 2>"$TMP_ERR"
    RC=$?
    ERR1=$(head -1 "$TMP_ERR")
    [ -z "$ERR1" ] && ERR1=$(head -1 "$TMP_OUT")
}

# 从 TMP_OUT 的 JSON 里取值：jget '<python 表达式，d 是解析后的 dict>'
jget() {
    "$PY" -c "
import json,sys
try:
    d = json.load(open(sys.argv[1], encoding='utf-8'))
except Exception:
    print('__PARSE_FAIL__'); sys.exit(0)
try:
    print(eval(sys.argv[2]))
except Exception:
    print('__MISSING__')
" "$TMP_OUT" "$1" 2>/dev/null
}

# 数值四则（python 代劳）
num_add() {
    "$PY" -c "
import sys
a, b = float(sys.argv[1]), float(sys.argv[2])
v = a + b
print(int(v) if v == int(v) else repr(v))" "$1" "$2"
}

# 数值比较（允许浮点容差）；用 python 免去 bash 里的小数运算
num_eq() {
    "$PY" -c "
import sys
a, b = float(sys.argv[1]), float(sys.argv[2])
print(1 if abs(a - b) <= 1e-6 * max(1.0, abs(b)) else 0)
" "$1" "$2"
}

echo "=== 关节链路自检：if=$IFACE channel=$CHANNEL node=$NODE runs=$RUNS ==="
echo "    cli=$CLI"
echo

for r in $(seq 1 "$RUNS"); do
    echo "--- 第 $r/$RUNS 轮 ---"

    # ---------- L1 节点发现 ----------
    run_cli scan
    if [ "$RC" -ne 0 ]; then
        bad L1-scan "rc=$RC  ${ERR1}"
    else
        n=$(jget "d['count']")
        has=$(jget "$NODE in d['nodes']")
        if [ "$has" = "True" ] && [ "$n" != "0" ]; then ok L1-scan
        else bad L1-scan "期望 nodes 含 $NODE，实际 count=$n nodes=$(jget "d['nodes']")"; fi
    fi

    # ---------- L2 设备信息（单请求） ----------
    run_cli info
    if [ "$RC" -ne 0 ]; then
        bad L2-info "rc=$RC  ${ERR1}"
    else
        fw=$(jget "d['fw_version']")
        if [ "$fw" != "__MISSING__" ] && [ "$fw" != "0" ]; then ok L2-info
        else bad L2-info "fw_version 异常：$fw  ${ERR1}"; fi
    fi

    # ---------- L3 描述符下载（38 KB 流式，最容易暴露"首帧丢"） ----------
    run_cli desc-info
    if [ "$RC" -ne 0 ]; then
        bad L3-desc "rc=$RC  ${ERR1}"
    else
        comp=$(jget "d['complete']")
        eps=$(jget "d['endpoint_count']")
        scanned=$(jget "d['bytes_scanned']")
        total=$(jget "d['total_len']")
        if [ "$comp" = "True" ] && [ "$eps" != "0" ] && [ "$scanned" = "$total" ]; then ok L3-desc
        else bad L3-desc "complete=$comp endpoints=$eps scanned=$scanned/$total  ${ERR1}"; fi
    fi

    # ---------- L4 心跳（只收不发；证明 RX 通路本身是通的） ----------
    run_cli hb-dump
    if [ "$RC" -ne 0 ]; then
        bad L4-hb "rc=$RC  ${ERR1}"
    else
        hb=$(jget "d['count']")
        if [ "$hb" != "0" ] && [ "$hb" != "__MISSING__" ]; then ok L4-hb
        else bad L4-hb "2 s 内一个心跳都没收到（通道/适配器层面）  ${ERR1}"; fi
    fi

    # ---------- L5 单读 ----------
    declare -A SOLO=()
    for p in "${PATHS[@]}"; do
        run_cli read "$p"
        if [ "$RC" -ne 0 ]; then
            bad L5-read "read $p  rc=$RC  ${ERR1}"
            continue
        fi
        v=$(jget "d['value']")
        SOLO[$p]="$v"
        if [ "$v" = "__MISSING__" ]; then
            bad L5-read "read $p 没有 value 字段"
        else
            # 量级合理性：字节序错时数值会荒谬（8.9e-41 / 16777216）
            sane=$("$PY" -c "
import sys
v=float(sys.argv[1]); p=sys.argv[2]
lim = {'axis0.config.can.node_id':(1,254),
       'axis0.config.can.heartbeat_rate_ms':(1,65535),
       'axis0.motor.config.gear_ratio':(0.01,10000),
       'axis0.motor.config.torque_constant':(0.0,100)}.get(p,( -1e12,1e12))
print(1 if lim[0] <= v <= lim[1] else 0)" "$v" "$p")
            if [ "$sane" = "1" ]; then ok L5-read
            else bad L5-read "read $p = $v，超出合理范围（字节序/单位问题？）"; fi
        fi
    done

    # ---------- L6 批量读，必须与 L5 一致 ----------
    run_cli batch-read "${PATHS[@]}"
    if [ "$RC" -ne 0 ]; then
        bad L6-batch "rc=$RC  ${ERR1}"
    else
        for p in "${PATHS[@]}"; do
            bv=$(jget "[x['value'] for x in d['values'] if x['path']=='$p'][0]")
            sv="${SOLO[$p]:-__MISSING__}"
            if [ "$bv" = "__MISSING__" ] || [ "$sv" = "__MISSING__" ]; then
                bad L6-batch "batch-read 缺少 $p 的值"
            elif [ "$(num_eq "$bv" "$sv")" = "1" ]; then ok L6-batch
            else bad L6-batch "$p 单读=$sv 批量=$bv **不一致**（两条解析路径有一处错）"; fi
        done
    fi

    # ---------- L6b --expect：设备已知真值时钉死 ----------
    for e in "${EXPECTS[@]:-}"; do
        [ -z "$e" ] && continue
        ep="${e%%=*}"; ev="${e#*=}"
        got="${SOLO[$ep]:-}"
        if [ -z "$got" ]; then
            run_cli read "$ep"; got=$(jget "d['value']")
        fi
        if [ "$(num_eq "${got:-__MISSING__}" "$ev" 2>/dev/null || echo 0)" = "1" ]; then ok L6b-expect
        else bad L6b-expect "$ep 期望 $ev，实际 $got"; fi
    done

    # ---------- L7 完整配置 + 健康快照 ----------
    run_cli health
    if [ "$RC" -ne 0 ]; then
        bad L7-health "rc=$RC  ${ERR1}"
    else
        online=$(jget "d['joint']['online']")
        fault=$(jget "d['joint']['fault']")
        if [ "$online" = "True" ] && [ "$fault" = "False" ]; then ok L7-health
        else bad L7-health "online=$online fault=$fault  ${ERR1}"; fi
    fi

    # ---------- L8 写路径（默认：原值回写；不改变设备状态）----------
    # ⚠ 虚拟后端**跳过**：每个 CLI 进程都会新建一条仿真总线，写入不可能被
    #   另一个进程读回 → 那两层在这里既测不出问题，又容易给出“通过”的假象。
    if [ "$IFACE" = "virtual" ]; then
        [ "$QUIET" -eq 0 ] && printf '      [skip] L8/L8b：虚拟后端下写入不跨进程保留（需真机）\n'
        write_ok=0
        orig=""
    else
    write_ok=0
    orig=""
    run_cli read "$WRITE_PATH"
    if [ "$RC" -ne 0 ]; then
        bad L8-write "先读 $WRITE_PATH 就失败了（写路径不测：没有已知原值就不写）rc=$RC  ${ERR1}"
    else
        orig=$(jget "d['value']")
        if [ "$orig" = "__MISSING__" ] || [ -z "$orig" ]; then
            bad L8-write "$WRITE_PATH 没有 value 字段"
        else
            # 写回原值（幂等）：这同时验证了请求帧的打包、设备接受与 ACK/回读
            "${BASE[@]}" --yes write "$WRITE_PATH" "$orig" >"$TMP_OUT" 2>"$TMP_ERR"
            RC=$?
            ERR1=$(head -1 "$TMP_ERR"); [ -z "$ERR1" ] && ERR1=$(head -1 "$TMP_OUT")
            if [ "$RC" -ne 0 ]; then
                bad L8-write "write $WRITE_PATH $orig rc=$RC  ${ERR1}"
            # ⚠ 断言 **verified**（写后读回确认），不是旧的 `written`：
            #   命令现在会**读回**再下结论；设备静默丢帧时 `verified=false` 且 rc=1。
            elif [ "$(jget "d['verified']")" != "True" ]; then
                bad L8-write "write 未报 verified=true（写入没被设备确认）  ${ERR1}"
            else
                run_cli read "$WRITE_PATH"
                got=$(jget "d['value']")
                if [ "$(num_eq "${got:-__MISSING__}" "$orig" 2>/dev/null || echo 0)" = "1" ]; then
                    ok L8-write; write_ok=1
                else
                    bad L8-write "原值回写后读回 $got ≠ 原值 $orig"
                fi
            fi
        fi
    fi

    # ---------- L8b 写探针（仅 --write-probe）：改一下再恢复 ----------
    if [ "$WRITE_PROBE" = "1" ] && [ "$write_ok" = "1" ]; then
        want=$(num_add "$orig" "$WRITE_DELTA")
        probe_ok=1

        "${BASE[@]}" --yes write "$WRITE_PATH" "$want" >"$TMP_OUT" 2>"$TMP_ERR"
        [ $? -ne 0 ] && probe_ok=0
        if [ "$probe_ok" = "1" ]; then
            run_cli read "$WRITE_PATH"
            got=$(jget "d['value']")
            [ "$(num_eq "${got:-__MISSING__}" "$want" 2>/dev/null || echo 0)" = "1" ] && [ "$(num_eq "$got" "$orig")" = "0" ] || probe_ok=0
        fi

        # **无论成败都恢复原值**（这是本层最重要的行为）
        "${BASE[@]}" --yes write "$WRITE_PATH" "$orig" >"$TMP_OUT" 2>"$TMP_ERR"
        restore_rc=$?
        run_cli read "$WRITE_PATH"
        back=$(jget "d['value']")
        if [ "$restore_rc" -ne 0 ] || [ "$(num_eq "${back:-__MISSING__}" "$orig" 2>/dev/null || echo 0)" != "1" ]; then
            bad L8b-write-probe "⚠⚠ **恢复失败**：$WRITE_PATH 现在是 ${back:-?}，原值 $orig（用 --yes write 手动回写）"
        elif [ "$probe_ok" = "1" ]; then
            ok L8b-write-probe
            [ "$QUIET" -eq 0 ] && printf '      写探针：%s → %s → 已恢复 %s\n' "$orig" "$want" "$orig"
        else
            bad L8b-write-probe "写 $want 后读回不符（但已恢复 $orig）"
        fi
    fi
    fi                              # / 虚拟后端跳过的 else
    echo
done

echo "=== 汇总（$RUNS 轮）==="
printf '  %-14s %6s %6s\n' "层" "通过" "失败"
total_ok=0; total_bad=0
for key in L1-scan L2-info L3-desc L4-hb L5-read L6-batch L6b-expect L7-health L8-write L8b-write-probe; do
    ko=${LAYER_OK[$key]:-0}; kb=${LAYER_BAD[$key]:-0}
    [ "$ko" -eq 0 ] && [ "$kb" -eq 0 ] && continue
    printf '  %-14s %6d %6d\n' "$key" "$ko" "$kb"
    total_ok=$((total_ok + ko)); total_bad=$((total_bad + kb))
done
echo
if [ "$total_bad" -eq 0 ]; then
    echo "结论：全部通过（$total_ok 项检查）"
    exit 0
fi

echo "结论：失败 $total_bad 项（共 $((total_ok + total_bad)) 项）。失败详情："
printf '  - %s\n' "${FAIL_LINES[@]}"
echo
echo "怎么读这些失败（按层定位）："
echo "  L1/L2 失败            → 适配器/端口/波特率/接线；先确认别的工具能收到帧"
echo "  L3 失败而 L4 通过     → 通道是通的，但**请求**没到达设备（首帧丢失类问题）"
echo "  L3 与 L4 同时失败     → 通道层面：端口没打开、终端电阻、bitrate、设备没上电"
echo "  L5/L6 数值荒谬/不一致 → 字节序或类型解析问题（见 docs/PROTOCOL_NOTES §3.1）"
echo "  L7 失败               → 标定/配置问题：看 stderr 里第一条具体原因"
echo "  L8/L8b 失败           → 写路径：请求打包 / 设备拒绝 / 原值恢复；**先看是不是恢复失败**（那种要手动回写）"
exit 1
