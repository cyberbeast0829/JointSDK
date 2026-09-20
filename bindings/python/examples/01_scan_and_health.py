"""示例 1：扫描总线 + 健康检查（**只读**，真机与虚拟后端都能跑）。

    python examples/01_scan_and_health.py
    python examples/01_scan_and_health.py --if socketcan --channel can0
"""

from __future__ import annotations

import argparse
import sys

import jsdk_can
from jsdk_can import JsdkError, Context, PcanHal, SlcanHal, SocketCanHal, VirtualHal


def make_hal(args):
    """按命令行选项选后端。**每个选项都要真的传到后端**（静默忽略比报错更难查）。"""
    if args.iface == "virtual":
        return VirtualHal(args.channel)
    if args.iface == "socketcan":
        return SocketCanHal(args.channel or "can0",
                            args.bitrate, args.data_bitrate)
    if args.iface == "pcan":
        return PcanHal(args.channel or "PCAN_USBBUS1",
                       args.bitrate, args.data_bitrate)
    if args.iface == "slcan":
        default_port = "COM3" if sys.platform.startswith("win") else "/dev/ttyACM0"
        # ⚠ slcan：第 2 个参数是**串口**波特率，第 3 个才是 FD 数据段速率。
        #   数据段速率是适配器私有表：0 = 不碰适配器配置，非 0 时会主动发 Y<n>
        #   （只认 2000000 与 5000000，其它值会明确报“不支持”）。
        return SlcanHal(args.channel or default_port, args.baud, args.data_bitrate)
    raise SystemExit(f"未知后端 {args.iface}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--if", dest="iface", default="virtual")
    ap.add_argument("--channel", default=None)
    ap.add_argument("--bitrate", type=int, default=1_000_000,
                    help="CAN 仲裁段速率（socketcan/pcan）")
    ap.add_argument("--data-bitrate", type=int, default=5_000_000,
                    help="CAN FD 数据段速率；0 = 不碰适配器配置（slcan 用）")
    ap.add_argument("--baud", type=int, default=115_200,
                    help="slcan 的**串口**波特率（不是 CAN 段速率）")
    ap.add_argument("--node", type=int, default=1)
    args = ap.parse_args()

    with Context(make_hal(args)) as ctx:
        # --- 1. 谁在总线上？（不下载描述符，很快） ---
        nodes = ctx.discover()
        print(f"发现 {len(nodes)} 个节点：{nodes}")
        if not nodes:
            print("提示：检查总线是否 up、波特率/FD 是否与设备一致、是否有 120Ω 终端")
            return 1

        # --- 2. 加关节 + configure（下载 JSON 描述符并读回量程） ---
        j = ctx.add_joint(args.node)
        ctx.configure()
        info = ctx.desc_info()
        print(f"描述符：{info.total_len} 字节 / {info.endpoint_count} 端点 / "
              f"crc=0x{info.crc:04X} / complete={info.complete}")

        # --- 3. 健康快照 ---
        fb = j.feedback()
        print(f"节点 {args.node}: online={fb.online} enabled={j.is_enabled()} "
              f"fault={j.is_fault()}")
        print(f"  pos={fb.pos:+.4f} rad  vel={fb.vel:+.4f} rad/s  "
              f"tau={fb.torque_Nm:+.3f} N·m")
        print(f"  vbus={fb.vbus_V:.2f} V  ibus={fb.ibus_A:.2f} A  "
              f"tMot={fb.t_motor_C:.1f} °C  tFet={fb.t_fet_C:.1f} °C")
        print(f"  axis_state={fb.axis_state}  mode={fb.mode}  "
              f"err_code={fb.err_code}  age={fb.age_ms} ms")

        if fb.has_flag.names():
            print(f"  粘滞标志：{fb.has_flag.names()}")
        if j.is_fault():
            print(f"  故障：{j.describe_fault()}")

        bs = ctx.bus_state()
        print(f"总线：link_up={bs.link_up} tx={bs.tx_frames} rx={bs.rx_frames} "
              f"errors={bs.link_errors}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except JsdkError as exc:
        # 示例报错就一行 —— 一大段调用栈对"第一次跑"的人是噪音
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(1)
