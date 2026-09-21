"""``python -m jsdk_can`` —— 与 ``jsdk-cli`` 对齐的完整诊断/运维入口。

覆盖 `jsdk-cli` 的全部子命令，**安全闸也一模一样**：

* 只读：``scan`` / ``info`` / ``err`` / ``hb-dump`` / ``health`` / ``mon`` /
  ``read`` / ``batch-read`` / ``dump-config`` / ``desc-info`` / ``ep-list`` /
  ``ep-lookup`` / ``desc-export``；
* 写设备（**需要 ``--yes``**）：``write`` / ``watchdog`` / ``save`` /
  ``set-node-id`` / ``reset``；
* 动作（**需要 ``--yes``**）：``set-zero`` / ``calibrate`` / ``home``；
  ``estop`` **不需要** ``--yes``（拒绝执行反而更危险）；
* ``mit`` 是**唯一会驱动电机**的命令：需要 ``--yes`` **且** ``--hold 秒``（自限时）。

退出码与 ``jsdk-cli`` 一致：``0`` 成功 / ``1`` 运行期失败 / ``2`` 用法错误 /
``3`` 被安全闸拒绝。输出字段与 ``jsdk-cli --json`` 一致，脚本可以互换。

⚠ 已知差异（写在这里，不隐藏）：``hb-dump`` 在**真实后端**上只能给出解码后的
心跳字段（Python 侧拿不到原始帧队列）；虚拟后端会额外给出捕获到的原始字节。
需要真机原始字节时用 C 版 ``jsdk-cli hb-dump``。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any

from . import __version__
from .context import Context
from .enums import Mode, StatusFlag
from .errors import JsdkError
from .hal import PcanHal, SlcanHal, SocketCanHal, VirtualHal

__all__ = ["main"]

#: 只读子命令（不需要 --yes）
READ_CMDS = {
    "scan", "info", "err", "hb-dump", "health", "mon", "read", "batch-read",
    "dump-config", "desc-info", "ep-list", "ep-lookup", "desc-export",
}

#: 会**写设备**的子命令（需要 --yes）
WRITE_CMDS = {"write", "watchdog", "save", "set-node-id", "reset",
              "set-zero", "calibrate", "home"}

#: ``estop`` 特殊：拒绝执行比误停更危险，所以**不要** --yes（与 jsdk-cli 一致）
NO_CONFIRM_CMDS = {"estop"}

#: 需要 --yes **且** --hold（自限时）的命令
HOLD_CMDS = {"mit"}

#: 连描述符都不下载的命令（省 38~41 KB 流量）
NO_DESC_CMDS = {"scan", "info", "err", "hb-dump", "estop", "desc-import"}

#: 只需要描述符、**不需要标定**的命令（与 jsdk-cli 的第二档一致）
#: ⚠ `watchdog` **不在**这里：它写的是 `can.config.break_timeout`，而那个端点 ID 是
#:   `configure()` 的标定阶段解析出来的 —— 只下描述符的话 `ep_break_timeout` 还是 0，
#:   调用会以 “endpoint unavailable” 失败（两版 CLI 都踩过，已加逐命令回归用例）。
DESC_ONLY_CMDS = {"read", "batch-read", "write", "save", "set-node-id",
                  "reset", "desc-info", "desc-export", "ep-list", "ep-lookup"}

#: 会跑控制周期的命令（只有它们需要设 period_ns）
LOOP_CMDS = {"mon", "mit", "calibrate", "home"}

ALL_CMDS = READ_CMDS | WRITE_CMDS | NO_CONFIRM_CMDS | HOLD_CMDS

#: 退出码（与 jsdk-cli 对齐）
EXIT_OK = 0
EXIT_RUNTIME = 1
EXIT_USAGE = 2
EXIT_GATE = 3


def _build_hal(args: argparse.Namespace):
    if args.iface == "virtual":
        return VirtualHal(args.channel or None)
    if args.iface == "socketcan":
        return SocketCanHal(args.channel or "can0", args.bitrate, args.data_bitrate)
    if args.iface == "pcan":
        return PcanHal(args.channel or "PCAN_USBBUS1", args.bitrate, args.data_bitrate)
    if args.iface == "slcan":
        return SlcanHal(args.channel or "COM3", args.baud, args.data_bitrate)
    raise SystemExit(f"未知后端：{args.iface}")


#: 全局选项的默认值。parent parser 用 SUPPRESS，解析完再统一补齐。
_DEFAULTS = {
    "iface": "virtual",
    "channel": None,
    "baud": 115_200,
    "bitrate": 1_000_000,
    "data_bitrate": 5_000_000,
    "master_id": 1,
    "node": 1,
    "classic": False,
    "json": False,
    "duration": 0,
    "rate_hz": 10,
    "yes": False,
    "hold": 0.0,
    "csv": False,
    "csv_file": None,
}


def _common_parser() -> argparse.ArgumentParser:
    """全局选项（主 parser 与每个子命令都继承一份，两种顺序都能写）。"""
    c = argparse.ArgumentParser(add_help=False, argument_default=argparse.SUPPRESS)
    c.add_argument("--if", dest="iface", choices=["virtual", "socketcan", "pcan",
                                                  "slcan"],
                   help="传输后端（默认 virtual：无硬件也能跑）")
    c.add_argument("--channel",
                   help="can0 / PCAN_USBBUS1 / COM3 / 虚拟节点规格")
    c.add_argument("--baud", type=int,
                   help="**串口**波特率（仅 slcan；与 --bitrate 不是一个量）")
    c.add_argument("--bitrate", type=int)
    c.add_argument("--data-bitrate", type=int)
    c.add_argument("--master-id", type=int)
    c.add_argument("--node", type=int)
    c.add_argument("--classic", action="store_true", help="强制 Classic CAN")
    c.add_argument("--json", action="store_true", help="机器可读输出")
    c.add_argument("--duration", type=int, help="mon 的时长（秒，0=直到 Ctrl-C）")
    c.add_argument("--rate-hz", type=int)
    c.add_argument("--csv", action="store_true", help="mon 用 CSV 而不是 NDJSON")
    # ⚠ 同样**不能**写 `default=None`（见 `--hold` 上的注释）：写在子命令之前的
    #   `--csv-file x.csv` 会被子解析器重新写回 None —— 文件根本不生成、而且不报错。
    c.add_argument("--csv-file", metavar="FILE",
                   help="mon 同时把 CSV 写到文件（与 --csv 同列）")
    c.add_argument("--yes", action="store_true",
                   help="确认执行会写设备/会动电机的命令（安全闸）")
    # ⚠ 不要写 `default=None`：本 parser 用 `argument_default=SUPPRESS`，
    #   显式给 default 会让**子解析器**再次写这个属性（真机踩过：`--hold 1`
    #   被覆盖成 None），默认值统一由 `_DEFAULTS` 在解析后补。
    c.add_argument("--hold", type=float,
                   help="mit 的自限时（秒）；**必须 > 0**")
    return c


def _parser() -> argparse.ArgumentParser:
    common = _common_parser()
    p = argparse.ArgumentParser(
        prog="python -m jsdk_can",
        parents=[common],
        description="CyberBeast 关节 SDK 的完整诊断/运维入口（与 jsdk-cli 对齐）",
        epilog="输出字段与 `jsdk-cli --json` 一致。全局选项可写在子命令之前或之后。",
    )
    p.add_argument("--version", action="version", version=f"jsdk_can {__version__}")

    sub = p.add_subparsers(dest="cmd", required=True)
    kw = {"parents": [common]}

    # --- 只读 -------------------------------------------------------------
    sub.add_parser("scan", help="节点发现", **kw)
    sub.add_parser("info", help="QUERY_DEVICE_INFO(0x46)", **kw)
    sub.add_parser("err", help="QUERY_ERROR(0x45) 六类错误明细", **kw)
    sub.add_parser("hb-dump", help="心跳（虚拟后端附带原始字节）", **kw)
    sub.add_parser("health", help="健康快照", **kw)
    sub.add_parser("mon", help="周期监控（NDJSON 或 --csv）", **kw)
    r = sub.add_parser("read", help="按名读参数", **kw)
    r.add_argument("path")
    br = sub.add_parser("batch-read", help="批量读（FD 单帧；Classic 自动逐条）", **kw)
    br.add_argument("paths", nargs="+")
    sub.add_parser("dump-config", help="关键配置快照", **kw)
    sub.add_parser("desc-info", help="描述符元信息", **kw)
    e = sub.add_parser("ep-list", help="枚举端点", **kw)
    e.add_argument("--filter", default=None, help="子串过滤")
    lk = sub.add_parser("ep-lookup", help="路径 → 端点", **kw)
    lk.add_argument("path")
    de = sub.add_parser("desc-export", help="导出描述符缓存到文件", **kw)
    de.add_argument("file")
    di = sub.add_parser("desc-import", help="从文件导入描述符缓存（不下载）", **kw)
    di.add_argument("file")

    # --- 写（需要 --yes）--------------------------------------------------
    w = sub.add_parser("write", help="写参数（需要 --yes）", **kw)
    w.add_argument("path")
    w.add_argument("value")
    wd = sub.add_parser("watchdog", help="写 can.config.break_timeout（需要 --yes）", **kw)
    wd.add_argument("ms", type=int)
    sub.add_parser("save", help="CONFIG_SAVE(0x22)，读回校验（需要 --yes）", **kw)
    sn = sub.add_parser("set-node-id", help="改节点地址（需要 --yes）", **kw)
    sn.add_argument("new_id", type=int)
    sub.add_parser("reset", help="RESET_DEVICE(0x64)（需要 --yes）", **kw)
    sub.add_parser("set-zero", help="当前位置设为零点（需要 --yes）", **kw)
    sub.add_parser("calibrate", help="标定：会动电机（需要 --yes）", **kw)
    sub.add_parser("home", help="回零：会动电机（需要 --yes）", **kw)

    # --- 动作 -------------------------------------------------------------
    sub.add_parser("estop", help="广播急停（**不需要** --yes）", **kw)
    mt = sub.add_parser("mit", help="驱动电机（需要 --yes 与 --hold）", **kw)
    mt.add_argument("--pos", type=float, default=0.0, help="目标位置 rad")
    mt.add_argument("--vel", type=float, default=0.0, help="目标速度 rad/s")
    mt.add_argument("--kp", type=float, default=0.0)
    mt.add_argument("--kd", type=float, default=0.0)
    mt.add_argument("--tau", type=float, default=0.0, help="前馈力矩 N·m")
    return p


def _apply_defaults(args: argparse.Namespace) -> argparse.Namespace:
    for k, v in _DEFAULTS.items():
        if not hasattr(args, k):
            setattr(args, k, v)
    return args


def _emit(args: argparse.Namespace, payload: dict[str, Any], human: list[str]) -> None:
    if args.json:
        print(json.dumps(payload, ensure_ascii=False, default=str))
    else:
        for line in human:
            print(line)


def _cmd_scan(ctx: Context, args) -> int:
    ids = ctx.discover()
    _emit(args, {"nodes": ids, "count": len(ids)},
          [f"发现 {len(ids)} 个节点: {ids}" if ids else
           "发现 0 个节点 —— 检查总线是否 up、波特率/FD 是否与设备一致、"
           "是否有 120Ω 终端；设备要先收到过一帧才学到主站地址"])
    return 0


def _cmd_health(ctx: Context, args) -> int:
    j = ctx.joint(args.node)
    fb = j.feedback()
    bs = ctx.bus_state()
    payload = {
        "joint": {
            "node": args.node,
            "pos_rad": fb.pos, "vel_rad_s": fb.vel,
            "current_A": fb.current_A, "torque_Nm": fb.torque_Nm,
            "t_motor_C": fb.t_motor_C, "t_fet_C": fb.t_fet_C,
            "vbus_V": fb.vbus_V, "ibus_A": fb.ibus_A,
            "axis_state": getattr(fb.axis_state, "name", fb.axis_state),
            "mode": getattr(fb.mode, "name", fb.mode),
            "mode_state_nibble": int(getattr(fb.mode_state, "value", fb.mode_state)),
            "err_code": fb.err_code, "hb_error": fb.hb_error,
            "axis_error": fb.axis_error, "age_ms": fb.age_ms,
            "tx_frames": fb.tx_frames, "tx_rejected": fb.tx_rejected,
            "status_flags": fb.status_flags, "online": fb.online,
            "enabled": j.is_enabled(), "fault": j.is_fault(),
        },
        "bus": bs.as_dict(),
    }
    human = [
        f"节点 {args.node}: online={fb.online} enabled={j.is_enabled()} "
        f"fault={j.is_fault()}",
        f"  axis_state={fb.axis_state} mode={fb.mode} err_code={fb.err_code} "
        f"hb=0x{fb.hb_error:02X} axis_err=0x{fb.axis_error:08X}",
        f"  pos={fb.pos:.4f} vel={fb.vel:.4f} tau={fb.torque_Nm:.3f} "
        f"tMot={fb.t_motor_C:.1f} vbus={fb.vbus_V:.2f} age={fb.age_ms}ms",
        f"  flags={fb.has_flag!r}",
        f"总线: link_up={bs.link_up} nodes={bs.nodes_online} "
        f"tx={bs.tx_frames} rx={bs.rx_frames} errors={bs.link_errors}",
    ]
    if j.is_fault():
        human.append("  故障: " + (j.describe_fault() or "(无描述)"))
    _emit(args, payload, human)
    return 0


#: `mon` 的 CSV 表头。
#: ⚠ **与 C 版 `jsdk-cli` 共用同一份契约**：列名、列数、列序、数值格式都会
#:   被跨版本对拍用例比对（`test_full_surface.py::test_mon_csv_header_matches_c_cli`）。
#:   以前两版**列数都不一样**（12 vs 17），“可以从两边对着看”其实是空话。
_MON_CSV_HEADER = (
    "t_ms,node,pos_rad,vel_rad_s,current_A,torque_Nm,"
    "t_motor_C,t_fet_C,vbus_V,ibus_A,axis_state,mode,err_code,"
    "hb_error,age_ms,tx_frames,tx_rejected"
)


def _mon_csv_row(fb, elapsed: int, node: int) -> str:
    """一行 CSV。格式必须与 C 版 `mon_csv_row()` 逐字节一致（%.6f/%.2f/%.3f…）。"""
    return (
        f"{elapsed},"
        f"{node},"
        f"{fb.pos:.6f},{fb.vel:.6f},{fb.current_A:.6f},{fb.torque_Nm:.6f},"
        f"{fb.t_motor_C:.2f},{fb.t_fet_C:.2f},{fb.vbus_V:.3f},{fb.ibus_A:.3f},"
        f"{fb.axis_state_name()},{fb.mode_name()},{fb.err_code},{fb.hb_error},"
        f"{fb.age_ms},{fb.tx_frames},{fb.tx_rejected}"
    )


def _cmd_mon(ctx: Context, args) -> int:
    period = max(1, int(1000 / max(1, args.rate_hz)))
    t0 = time.monotonic()
    elapsed = 0
    j = ctx.joint(args.node)
    csv_file = None
    if args.csv:
        print(_MON_CSV_HEADER, flush=True)
    if args.csv_file:
        csv_file = open(args.csv_file, "w", encoding="utf-8", newline="")
        csv_file.write(_MON_CSV_HEADER + "\n")

    try:
        while args.duration <= 0 or elapsed < args.duration * 1000:
            ctx.poll()
            fb = j.feedback()
            state = getattr(fb.axis_state, "name", fb.axis_state)
            mode = getattr(fb.mode, "name", fb.mode)
            if args.csv:
                print(_mon_csv_row(fb, elapsed, args.node), flush=True)
            else:
                print(json.dumps({
                    "t_ms": elapsed, "node": args.node,
                    "pos_rad": fb.pos, "vel_rad_s": fb.vel,
                    "torque_Nm": fb.torque_Nm, "t_motor_C": fb.t_motor_C,
                    "vbus_V": fb.vbus_V,
                    "axis_state": state, "mode": mode,
                    "err_code": fb.err_code, "age_ms": fb.age_ms,
                    "valid": fb.valid,
                }, ensure_ascii=False), flush=True)
            if csv_file is not None:
                csv_file.write(_mon_csv_row(fb, elapsed, args.node) + "\n")
                csv_file.flush()
            time.sleep(period / 1000.0)
            elapsed = int((time.monotonic() - t0) * 1000)
    except KeyboardInterrupt:
        print("（mon 被中断）", file=sys.stderr)
    finally:
        if csv_file is not None:
            csv_file.close()
    return EXIT_OK


def _cmd_read(ctx: Context, args) -> int:
    j = ctx.joint(args.node)
    value = j.param_get(args.path)
    ep = ctx.lookup(args.path)
    _emit(args, {"path": args.path, "type": ep.type_name, "value": value,
                 "value_text": str(value)},
          [f"{args.path} = {value} ({ep.type_name})"])
    return EXIT_OK


def _cmd_info(ctx: Context, args) -> int:
    info = ctx.device_info()
    _emit(args, {"hw_version": info.hw_version, "fw_version": info.fw_version,
                 "serial": info.serial, "classic": info.classic,
                 "node": args.node},
          [f"节点 {args.node}:",
           f"  hw_version             {info.hw_version}",
           f"  fw_version             {info.fw_version}",
           f"  serial                 {info.serial}",
           f"  device_mode            {'Classic' if info.classic else 'FD'}"])
    return EXIT_OK


def _cmd_err(ctx: Context, args) -> int:
    """``QUERY_ERROR(0x45)`` 六类错误字明细（与 C 版字段名一致）。"""
    fi = ctx.joint(args.node).query_error_detail()
    payload = {
        "mit_err": fi.mit_err, "mit_err_name": fi.mit_err_name,
        "hb_flags": fi.hb_flags,
        "motor_error": fi.motor_error, "encoder_error": fi.encoder_error,
        "sensorless_error": fi.sensorless_error,
        "controller_error": fi.controller_error,
        "system_error": fi.system_error, "axis_error": fi.axis_error,
    }
    human = [f"节点 {args.node} 错误明细:", f"  mit_err={fi.mit_err_name} "
             f"hb_flags=0x{fi.hb_flags:02X}"]
    if fi.subsystems():
        human += [f"  {k:<18} 0x{v:08X}" for k, v in fi.subsystems().items()]
    else:
        human.append("  （六个子系统全为 0）")
    _emit(args, payload, human)
    return EXIT_OK


def _cmd_hb_dump(ctx: Context, args) -> int:
    """等一个心跳（最多 2 s）并打印解码字段。

    ⚠ 与 C 版的差异：Python 侧拿不到"原始帧队列"（HAL 回调在 C 里），所以**真实
    后端**上这里只有解码后的字段；虚拟后端会额外给出捕获到的原始字节。
    """
    j = ctx.joint(args.node)
    t0 = time.monotonic()
    got = None
    while (time.monotonic() - t0) < 2.0:
        ctx.cycle_begin(0)
        ctx.cycle_end()
        fb = j.feedback()
        if fb.valid and fb.age_ms < 500:
            got = fb
            break
        time.sleep(0.002)

    if got is None:
        print(f"{args.node}: 2 s 内没有收到心跳（检查 node_id/接线/波特率）",
              file=sys.stderr)
        return EXIT_RUNTIME

    waited = int((time.monotonic() - t0) * 1000)
    hb: dict[str, Any] = {"src": args.node, "life": None,
                          "err_flags": got.hb_error,
                          "state": int(getattr(got.axis_state, "value",
                                               got.axis_state)),
                          "control_mode": int(getattr(got.mode, "value",
                                                      got.mode))}
    payload: dict[str, Any] = {"heartbeats": [hb], "count": 1,
                               "waited_ms": waited}
    human = [f"节点 {args.node} 心跳:",
             f"  err_flags={got.hb_error} axis_state={hb['state']} "
             f"mode={hb['control_mode']} age_ms={got.age_ms}"]

    # 虚拟后端能给原始字节（真机需要 C 版 jsdk-cli hb-dump）
    cap = getattr(ctx.hal, "capture", None)
    if callable(cap):
        last = None
        while True:
            fr = cap()
            if fr is None:
                break
            last = fr
        if last is not None:
            raw = " ".join(f"{b:02X}" for b in bytes(last.data[:last.len]))
            hb["len"] = int(last.len)
            hb["bytes"] = raw + " "
            human.append(f"  原始字节: {hb['bytes']}")
    _emit(args, payload, human)
    return EXIT_OK


def _cmd_batch_read(ctx: Context, args) -> int:
    paths = list(args.paths)
    if len(paths) > 8:
        print("一次最多 8 条（超了请分批）", file=sys.stderr)
        return EXIT_USAGE
    res = ctx.joint(args.node).param_get_batch(*paths)
    values, human, ok = [], [], True
    for p in paths:
        v = res[p]
        if isinstance(v, JsdkError):
            ok = False
            values.append({"path": p, "error": v.name(), "message": v.detail})
            human.append(f"  {p:<48} 失败：{v.name()} — {v.detail}")
        else:
            ep = ctx.lookup(p)
            values.append({"path": p, "value": v, "type": ep.type_name})
            human.append(f"  {p:<48} = {v}")
    _emit(args, {"values": values, "status": "ok" if ok else "partial"},
          [f"节点 {args.node} 批量读（{len(paths)} 条）:"] + human)
    return EXIT_OK if ok else EXIT_RUNTIME


def _cmd_desc_export(ctx: Context, args) -> int:
    blob = ctx.desc_export()
    with open(args.file, "wb") as fh:
        fh.write(blob)
    import os

    _emit(args, {"file": os.path.abspath(args.file), "bytes": len(blob)},
          [f"已导出 {len(blob)} 字节 → {args.file}"])
    return EXIT_OK


def _cmd_desc_import(ctx: Context, args) -> int:
    with open(args.file, "rb") as fh:
        blob = fh.read()
    ctx.desc_import(blob)
    info = ctx.desc_info()
    import os

    _emit(args, {"file": os.path.abspath(args.file), "bytes": len(blob),
                 "imported": True, "endpoint_count": info.endpoint_count,
                 "downloaded": False},
          [f"已导入 {len(blob)} 字节（{info.endpoint_count} 端点），未下载"])
    return EXIT_OK


# ==========================================================================
# 写设备（需要 --yes）
# ==========================================================================


def _parse_cli_value(text: str, type_name: str):
    """按端点类型解析命令行里的值（与 C 版一样：类型不符/超范围直接拒绝）。"""
    if type_name == "bool":
        low = text.strip().lower()
        if low in ("1", "true", "on", "yes"):
            return True
        if low in ("0", "false", "off", "no"):
            return False
        raise ValueError(f"bool 端点只接受 0/1/true/false（收到 {text!r}）")
    if type_name.startswith("float"):
        return float(text)
    return int(text, 0)          # 允许 0x 前缀，方便写位掩码类参数


def _cmd_write(ctx: Context, args) -> int:
    j = ctx.joint(args.node)
    ep = ctx.lookup(args.path)
    try:
        value = _parse_cli_value(args.value, ep.type_name)
    except ValueError as exc:
        print(f"{args.path}: {exc}", file=sys.stderr)
        return EXIT_USAGE

    j.param_set(args.path, value)
    got = j.param_get(args.path)
    _emit(args, {"path": args.path, "id": ep.ep_id, "type": ep.type_name,
                 "value": got, "written": True,
                 "persisted": "no (use save to persist)"},
          [f"{args.path} = {got} ({ep.type_name}) 已写入；"
           "⚠ 不落 Flash（需要 save）"])
    return EXIT_OK


def _cmd_watchdog(ctx: Context, args) -> int:
    j = ctx.joint(args.node)
    ms = int(args.ms)
    j.set_watchdog_ms(ms)

    # ⚠ 校验结论只认标志位；而报告给用户的“设备现值”**独立再读一次设备**，
    #    而不是回显 SDK 内部记住的写入值 —— 真机上会写 250 却读回 0（F28），
    #    回显就会变成“说得比知道的多”。
    unverified = j.feedback().has_flag(StatusFlag.WATCHDOG_UNVERIFIED)
    try:
        back = int(j.param_get("can.config.break_timeout"))
    except JsdkError:
        back = None                      # 读不到就如实说读不到（JSON 里是 null）

    human = [f"节点 {args.node} 写入 break_timeout = {ms} ms"]
    if back is not None:
        human.append(f"  设备独立读回：{back} ms")
    if ms == 0:
        human.append("  0 = **关闭**设备侧协议级超时检测"
                     "（最新固件语义，不等于 100 ms）")
    elif unverified:
        human.append("  写入已接受，但**读回校验未成功**：无法确认保护已武装")
        human.append("  原因：本固件该端点读回恒为 0（= 禁用）→ FIRMWARE_ISSUES F28")
    else:
        human.append("  已读回确认")
    human.append("保护只在设备收到过控制类帧后武装；"
                 "纯 CURRENT(0x04) 客户端武装不了（F19）")

    _emit(args, {"ms": ms, "device_reports_ms": back, "verified": not unverified,
                 "disabled": ms == 0},
          human)
    return EXIT_OK


def _cmd_save(ctx: Context, args) -> int:
    ctx.joint(args.node).save_config()
    _emit(args, {"saved": True}, ["配置已保存（写后读回校验通过）"])
    return EXIT_OK


def _cmd_set_node_id(ctx: Context, args) -> int:
    j = ctx.joint(args.node)
    old = j.node_id
    j.set_node_id(int(args.new_id))
    _emit(args, {"old_node": old, "new_node": int(args.new_id),
                 "persisted": True},
          [f"节点号 {old} → {args.new_id}（新地址已验证有应答）"])
    return EXIT_OK


def _cmd_reset(ctx: Context, args) -> int:
    ctx.joint(args.node).reset_device()
    _emit(args, {"reset": True}, ["RESET_DEVICE(0x64) 已发送（设备会重启）"])
    return EXIT_OK


def _cmd_set_zero(ctx: Context, args) -> int:
    ctx.joint(args.node).set_zero_here()
    _emit(args, {"zeroed": True}, ["当前位置已设为零点（未落 Flash）"])
    return EXIT_OK


def _cmd_calibrate(ctx: Context, args) -> int:
    ctx.joint(args.node).calibrate()
    _emit(args, {"calibrated": True}, ["标定完成（电机会动）"])
    return EXIT_OK


def _cmd_home(ctx: Context, args) -> int:
    ctx.joint(args.node).home()
    _emit(args, {"homed": True}, ["回零完成（电机会动）"])
    return EXIT_OK


def _cmd_estop(ctx: Context, args) -> int:
    ctx.estop()
    _emit(args, {"estop_sent": True},
          ["已广播 ESTOP(0xC0)：所有关节立即进入安全态"])
    return EXIT_OK


def _cmd_mit(ctx: Context, args) -> int:
    """唯一会驱动电机的命令：使能 MIT → 跑 ``--hold`` 秒 → 安全停车。"""
    j = ctx.joint(args.node)
    ctx.activate()
    if not j.is_enabled():
        print("使能失败（关节未就绪）", file=sys.stderr)
        return EXIT_RUNTIME

    print(f"驱动节点 {args.node}：pos={args.pos} vel={args.vel} kp={args.kp} "
          f"kd={args.kd} tau={args.tau}，持续 {args.hold} 秒", file=sys.stderr)
    t0 = time.monotonic()
    try:
        while (time.monotonic() - t0) < float(args.hold):
            ctx.cycle_begin(0)
            j.set_mit(pos=args.pos, vel=args.vel, tau=args.tau,
                      kp=args.kp, kd=args.kd)
            ctx.cycle_end()
            ctx.pace()
    except KeyboardInterrupt:
        print("（被 Ctrl-C 中断，正在安全停车）", file=sys.stderr)
    finally:
        ctx.deactivate()          # hold_position → STOP_MOTOR → 等 IDLE

    fb = j.feedback()
    _emit(args, {"node": args.node, "drove_s": float(args.hold),
                 "pos_rad": fb.pos, "vel_rad_s": fb.vel,
                 "torque_Nm": fb.torque_Nm, "enabled": j.is_enabled()},
          [f"已停：pos={fb.pos:.4f} rad vel={fb.vel:.4f} rad/s "
           f"tau={fb.torque_Nm:.3f} N·m enabled={j.is_enabled()}"])
    return EXIT_OK


def _cmd_dump_config(ctx: Context, args) -> int:
    snap = ctx.joint(args.node).config_snapshot()
    _emit(args, snap.as_dict(),
          [f"节点 {args.node} 配置快照 (valid={snap.valid}):"]
          + [f"  {k:<20} {v}" for k, v in snap.as_dict().items() if k != "valid"])
    return 0 if snap.valid else 1


def _cmd_desc_info(ctx: Context, args) -> int:
    info = ctx.desc_info()
    _emit(args, info.as_dict(),
          [f"描述符: {info.total_len} 字节, crc=0x{info.crc:04X}, "
           f"{info.endpoint_count} 端点, complete={info.complete}"])
    return 0


def _cmd_ep_list(ctx: Context, args) -> int:
    eps = ctx.endpoints()
    if args.filter:
        eps = [e for e in eps if args.filter in e.path]
    _emit(args, {"endpoints": [{"path": e.path, "id": e.ep_id,
                               "type": e.type_name, "access": e.access_str}
                              for e in eps],
                 "count": len(eps), "status": "ok"},
          [f"{e.ep_id:>6}  {e.access_str}  {e.type_name:<12} {e.path}" for e in eps]
          + [f"共 {len(eps)} 个端点"])
    return 0


def _cmd_ep_lookup(ctx: Context, args) -> int:
    ep = ctx.lookup(args.path)
    _emit(args, {"path": args.path, "id": ep.ep_id, "type": ep.type_name,
                 "readable": ep.readable, "writable": ep.writable},
          [f"{args.path}\n  id={ep.ep_id} type={ep.type_name} "
           f"access={ep.access_str}"])
    return 0


_HANDLERS = {
    # 只读
    "scan": _cmd_scan,
    "info": _cmd_info,
    "err": _cmd_err,
    "hb-dump": _cmd_hb_dump,
    "health": _cmd_health,
    "mon": _cmd_mon,
    "read": _cmd_read,
    "batch-read": _cmd_batch_read,
    "dump-config": _cmd_dump_config,
    "desc-info": _cmd_desc_info,
    "ep-list": _cmd_ep_list,
    "ep-lookup": _cmd_ep_lookup,
    "desc-export": _cmd_desc_export,
    "desc-import": _cmd_desc_import,
    # 写设备（--yes）
    "write": _cmd_write,
    "watchdog": _cmd_watchdog,
    "save": _cmd_save,
    "set-node-id": _cmd_set_node_id,
    "reset": _cmd_reset,
    "set-zero": _cmd_set_zero,
    "calibrate": _cmd_calibrate,
    "home": _cmd_home,
    # 动作
    "estop": _cmd_estop,
    "mit": _cmd_mit,
}


def _force_utf8_when_redirected() -> None:
    """
    被**重定向**（管道/文件）时，把 stdout/stderr 钉成 UTF-8。

    为什么只管重定向：Windows 下 Python 对**控制台**用的是 UTF-16（PEP 528），
    中文一直是好的；但重定向时用的是 **locale 编码**（中文机器上就是 cp936）。
    于是同一个流水线里会出现两种编码 —— C 版 ``jsdk-cli`` 重定向时写 UTF-8，
    本入口写 GBK；而且输出里一旦出现 CP936 表示不了的字符（如 ``⚠`` U+26A0），
    直接 ``UnicodeEncodeError`` **崩掉**，而不是"显示不好看"。

    ⚠ 不要指望用户设 ``PYTHONIOENCODING``：测试里能设，现场不会。
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            if stream is not None and not stream.isatty():
                stream.reconfigure(encoding="utf-8")  # 3.7+
        except (AttributeError, ValueError, OSError):
            # reconfigure 不存在（被包装过的流）或流不可重配 —— 不影响后续逻辑
            pass


def main(argv: list[str] | None = None) -> int:
    """``python -m jsdk_can`` 的入口。返回进程退出码（与 ``jsdk-cli`` 一致）。"""
    _force_utf8_when_redirected()
    args = _apply_defaults(_parser().parse_args(argv))

    # --- 安全闸：**在碰总线之前**判完 ---
    # 顺序很重要：不满足闸门时不能打开后端（否则“被拒绝”的命令还是会碰设备）。
    if args.cmd in WRITE_CMDS and not args.yes:
        print(f"`{args.cmd}` 会写设备，必须显式加 --yes"
              "（先读一遍确认当前值，再决定写什么）", file=sys.stderr)
        return EXIT_GATE
    if args.cmd in HOLD_CMDS:
        if not args.yes:
            print("`mit` 会驱动电机，必须显式加 --yes", file=sys.stderr)
            return EXIT_GATE
        hold = float(args.hold or 0.0)
        if not 1.0 <= hold <= 60.0:
            print(f"--hold 必须在 1..60 秒之间（给的是 {args.hold}）",
                  file=sys.stderr)
            return EXIT_GATE

    is_fd = not args.classic and args.data_bitrate != 0
    hal = _build_hal(args)
    period_ns = int(1e9 / max(1, args.rate_hz)) if args.cmd in LOOP_CMDS else 0

    try:
        with Context(hal, master_id=args.master_id, is_fd=is_fd,
                     period_ns=period_ns) as ctx:
            ctx.add_joint(args.node, mode=Mode.MIT)
            # 三档前置（与 jsdk-cli 一致）：
            #   ① 不下载描述符 ② 只下描述符（不标定）③ 完整配置（含标定）
            if args.cmd not in NO_DESC_CMDS:
                if args.cmd in DESC_ONLY_CMDS:
                    ctx.desc_fetch()
                else:
                    ctx.configure()
            return _HANDLERS[args.cmd](ctx, args)
    except JsdkError as exc:
        if args.json:
            print(json.dumps({"error": {"op": exc.op, "status": exc.name(),
                                        "status_code": int(exc.status),
                                        "message": exc.detail}},
                             ensure_ascii=False))
        print(f"jsdk_can: {exc}", file=sys.stderr)
        return EXIT_RUNTIME


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
