#!/usr/bin/env python3
"""Python 绑定的**真机冒烟**（非运动：只读 + 一次"回写原值"）。

与 `tools/hw_verify.sh`（C CLI）互补：那个查的是链路分层，这个查的是
**Python 侧那几条最容易悄悄错的路**：
  * Classic 链路上 u64 参数的**分块读**（FD 一次 8 B / Classic 一次 4 B）；
  * 单读 vs `param_get_batch()` 交叉对拍（v0.22 的教训：真机验证必须对拍）；
  * `param_set_auto()` 按端点声明类型装箱 + "写后读回"契约（**回写原值，不改状态**）；
  * 帧格式自动对齐（`framing_learned`）与 `python -m jsdk_can` 的一致行为。

用法（Linux 上串口属 root:dialout，所以要 root）::

    cmake -S . -B bsh-linux -DJSDK_BUILD_SHARED=ON -DJSDK_BUILD_PYTHON=OFF
    cmake --build bsh-linux -j
    sudo env JSDK_LIB_PATH=$PWD/bsh-linux PYTHONPATH=$PWD/bindings/python/src \\
         python3 tools/py_hw_smoke.py --if slcan --channel /dev/ttyACM0 --node 1

退出码：0 = 全通过；1 = 有失败；2 = 用法/环境问题。
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(os.path.dirname(_HERE), "bindings", "python", "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

# 重定向时 Windows 控制台默认用 GBK 编码 → 中文会变乱码（与 `python -m jsdk_can` 同一处理）
try:
    sys.stdout.reconfigure(encoding="utf-8")      # type: ignore[union-attr]
    sys.stderr.reconfigure(encoding="utf-8")      # type: ignore[union-attr]
except Exception:                                   # pragma: no cover - 老 Python/包装流
    pass

import jsdk_can  # noqa: E402
from jsdk_can import Context  # noqa: E402

_ok = 0
_bad: list[str] = []


def check(name: str, cond: bool, detail: str = "") -> None:
    global _ok
    if cond:
        _ok += 1
        print(f"  ok   {name} {detail}")
    else:
        _bad.append(name)
        print(f"  FAIL {name} {detail}")


def build_hal(args):
    """按 `--if` 建后端。

    ⚠ slcan 的 `data_bitrate` 传 **0**（= 不碰适配器配置）：真机实测过
      （1 Mbps Classic 的适配器），先发 `Y5` 会把适配器配成 FD，
      之后即使 SDK 改学成 Classic，帧也**发不出去**。
    """
    if args.iface == "slcan":
        return jsdk_can.SlcanHal(args.channel, args.baud, 0)
    if args.iface == "socketcan":
        return jsdk_can.SocketCanHal(args.channel, args.bitrate, args.data_bitrate)
    if args.iface == "pcan":
        return jsdk_can.PcanHal(args.channel, args.bitrate, args.data_bitrate)
    if args.iface == "virtual":
        return jsdk_can.VirtualHal(args.channel or None)
    raise SystemExit(f"未知后端 --if {args.iface}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Python 绑定真机冒烟（非运动）")
    ap.add_argument("--if", dest="iface", default="slcan",
                    choices=["slcan", "socketcan", "pcan", "virtual"])
    ap.add_argument("--channel", default="/dev/ttyACM0")
    ap.add_argument("--node", type=int, default=1)
    ap.add_argument("--baud", type=int, default=115_200)
    ap.add_argument("--bitrate", type=int, default=1_000_000)
    ap.add_argument("--data-bitrate", type=int, default=5_000_000)
    ap.add_argument("--lib-path", default=None, help="共享库目录（= JSDK_LIB_PATH）")
    args = ap.parse_args()

    if args.lib_path:
        os.environ["JSDK_LIB_PATH"] = args.lib_path

    print(f"== A) 高层 API（{args.iface} {args.channel} node {args.node}）==")
    # `is_fd=False` + `is_fd_explicit=False` = “先按 Classic 发，但允许被对端帧对齐”
    # —— 与两个 CLI 的自动模式同一组合（FD 控制器也收经典帧，反之不成立）。
    ctx = Context(build_hal(args), master_id=1, is_fd=False, is_fd_explicit=False,
                  period_ns=0, state_timeout_ms=30_000)
    try:
        j = ctx.add_joint(args.node)
        t0 = time.time()
        ctx.desc_fetch()                       # 阻塞式；返回 None
        di = ctx.desc_info()
        dt = time.time() - t0
        print(f"       desc: {di.total_len} B / {di.endpoint_count} 端点 / "
              f"{di.frames_rx} 帧 / {dt:.1f} s")
        check("desc 完整（bytes_scanned == total_len）",
              int(di.bytes_scanned) == int(di.total_len) and bool(di.complete))
        check("帧格式已对齐/一致", ctx.framing_learned in (1, 2, 3),
              f"framing_learned={ctx.framing_learned}")

        paths = ["serial_number", "axis0.motor.config.gear_ratio",
                 "axis0.motor.error", "axis0.config.can.node_id"]
        vals = {p: j.param_get(p) for p in paths}
        for p, v in vals.items():
            print(f"       {p} = {v}")
        check("serial_number 读得出来（u64：Classic 下要分块）",
              isinstance(vals["serial_number"], int) and vals["serial_number"] > 0)
        check("gear_ratio 合理（>0）", float(vals["axis0.motor.config.gear_ratio"]) > 0.0)
        check("node_id 与 --node 一致", int(vals["axis0.config.can.node_id"]) == args.node)

        # 单读 vs 批量读**必须完全一致**（v0.22 的教训：只看"成功"发现不了错值）
        b = j.param_get_batch(*paths)
        check("单读 / batch-read 完全一致",
              all(b[p] == vals[p] for p in paths), str(b))

        # 写路径：把**原值**回写一遍（不改设备状态），验证装箱类型与"写后读回"
        before = j.param_get("can.config.break_timeout")
        j.param_set_auto("can.config.break_timeout", int(before))
        after = j.param_get("can.config.break_timeout")
        check("param_set_auto 回写原值后读回一致", int(after) == int(before),
              f"{before} -> {after}")
    finally:
        ctx.close()

    print("== B) 子命令 CLI（python -m jsdk_can，非运动）==")
    env = dict(os.environ)
    env["PYTHONPATH"] = _SRC
    env["PYTHONIOENCODING"] = "utf-8"

    def run_cli(*extra: str):
        return subprocess.run([sys.executable, "-m", "jsdk_can", *extra],
                              capture_output=True, text=True, encoding="utf-8",
                              errors="replace", env=env, timeout=300)

    base = ["--if", args.iface, "--channel", args.channel, "--node", str(args.node)]
    r = run_cli(*base, "--json", "read", "serial_number")
    check("CLI read serial_number", r.returncode == 0 and str(vals["serial_number"]) in r.stdout,
          (r.stdout or r.stderr).strip()[:90])

    r = run_cli(*base, "--json", "--timeout-ms", "30000", "health")
    check("CLI health", r.returncode == 0 and '"online"' in r.stdout,
          (r.stderr or "").strip()[:90])

    r = run_cli(*base, "--json", "scan")
    check("CLI scan 发现节点", r.returncode == 0 and f'[{args.node}]' in r.stdout,
          (r.stdout or r.stderr).strip()[:90])

    print()
    print(f"=== Python 真机冒烟：{_ok} 项通过，{len(_bad)} 项失败 ===")
    for n in _bad:
        print("  失败：", n)
    return 1 if _bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
