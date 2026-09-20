"""``python -m jsdk_can`` —— **只读**诊断子集。

为什么只做只读：这一入口是给"手边没有编译器、又想知道设备怎么了"的场景用的。
凡是会写设备或让电机动的操作一律不做 —— 那些请用 C 实现的 ``jsdk-cli``，它有
完整的 ``--yes`` / ``--hold`` 安全闸。

子命令：``scan`` / ``health`` / ``mon`` / ``read`` / ``dump-config`` /
``desc-info`` / ``ep-list`` / ``ep-lookup``。

输出与 ``jsdk-cli`` 的 ``--json`` 字段名保持一致，脚本可以互换使用。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any

from . import __version__
from .context import Context
from .enums import Mode
from .errors import JsdkError
from .hal import PcanHal, SlcanHal, SocketCanHal, VirtualHal

__all__ = ["main"]

_READ_ONLY = {"scan", "health", "mon", "read", "dump-config", "desc-info",
              "ep-list", "ep-lookup"}


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
    return c


def _parser() -> argparse.ArgumentParser:
    common = _common_parser()
    p = argparse.ArgumentParser(
        prog="python -m jsdk_can",
        parents=[common],
        description="CyberBeast 关节 SDK 的**只读**诊断入口（写/动请用 jsdk-cli）",
        epilog="输出字段与 `jsdk-cli --json` 一致。全局选项可写在子命令之前或之后。",
    )
    p.add_argument("--version", action="version", version=f"jsdk_can {__version__}")

    sub = p.add_subparsers(dest="cmd", required=True)
    kw = {"parents": [common]}
    sub.add_parser("scan", help="节点发现", **kw)
    sub.add_parser("health", help="健康快照", **kw)
    sub.add_parser("mon", help="周期监控（NDJSON）", **kw)
    r = sub.add_parser("read", help="按名读参数", **kw)
    r.add_argument("path")
    sub.add_parser("dump-config", help="关键配置快照", **kw)
    sub.add_parser("desc-info", help="描述符元信息", **kw)
    e = sub.add_parser("ep-list", help="枚举端点", **kw)
    e.add_argument("--filter", default=None, help="子串过滤")
    lk = sub.add_parser("ep-lookup", help="路径 → 端点", **kw)
    lk.add_argument("path")
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


def _cmd_mon(ctx: Context, args) -> int:
    period = max(1, int(1000 / max(1, args.rate_hz)))
    t0 = time.monotonic()
    elapsed = 0
    j = ctx.joint(args.node)

    try:
        while args.duration <= 0 or elapsed < args.duration * 1000:
            ctx.poll()
            fb = j.feedback()
            print(json.dumps({
                "t_ms": elapsed, "node": args.node,
                "pos_rad": fb.pos, "vel_rad_s": fb.vel,
                "torque_Nm": fb.torque_Nm, "t_motor_C": fb.t_motor_C,
                "vbus_V": fb.vbus_V,
                "axis_state": getattr(fb.axis_state, "name", fb.axis_state),
                "mode": getattr(fb.mode, "name", fb.mode),
                "err_code": fb.err_code, "age_ms": fb.age_ms,
                "valid": fb.valid,
            }, ensure_ascii=False), flush=True)
            time.sleep(period / 1000.0)
            elapsed = int((time.monotonic() - t0) * 1000)
    except KeyboardInterrupt:
        print("（mon 被中断）", file=sys.stderr)
    return 0


def _cmd_read(ctx: Context, args) -> int:
    j = ctx.joint(args.node)
    value = j.param_get(args.path)
    ep = ctx.lookup(args.path)
    _emit(args, {"path": args.path, "type": ep.type_name, "value": value,
                 "value_text": str(value)},
          [f"{args.path} = {value} ({ep.type_name})"])
    return 0


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
    "scan": _cmd_scan,
    "health": _cmd_health,
    "mon": _cmd_mon,
    "read": _cmd_read,
    "dump-config": _cmd_dump_config,
    "desc-info": _cmd_desc_info,
    "ep-list": _cmd_ep_list,
    "ep-lookup": _cmd_ep_lookup,
}


def main(argv: list[str] | None = None) -> int:
    """``python -m jsdk_can`` 的入口。返回进程退出码。"""
    args = _apply_defaults(_parser().parse_args(argv))

    if args.cmd not in _READ_ONLY:  # pragma: no cover - argparse 已限制
        print(f"本入口只支持只读子命令：{sorted(_READ_ONLY)}", file=sys.stderr)
        return 2

    is_fd = not args.classic and args.data_bitrate != 0
    hal = _build_hal(args)

    try:
        with Context(hal, master_id=args.master_id, is_fd=is_fd) as ctx:
            ctx.add_joint(args.node, mode=Mode.MIT)
            # scan 只需要一个能收发帧的上下文（描述符要 41 KB 流量，不必要）
            if args.cmd != "scan":
                ctx.configure()
            return _HANDLERS[args.cmd](ctx, args)
    except JsdkError as exc:
        if args.json:
            print(json.dumps({"error": {"op": exc.op, "status": exc.name(),
                                        "status_code": int(exc.status),
                                        "message": exc.detail}},
                             ensure_ascii=False))
        print(f"jsdk_can: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
