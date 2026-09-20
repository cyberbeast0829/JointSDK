#!/usr/bin/env python3
"""从 ODrive 固件的 autogen/endpoints.hpp 提取设备实际服务的 JSON 描述符。

用途
----
`Firmware/autogen/endpoints.hpp` 把整份 JSON 描述符拆成多个 C 字符串字面量存储
（设备通过 JSON_DESC_DATA(0x25) 原样回送这些字节）。本工具把它们拼回完整 JSON，
写成测试夹具，供 SDK 的解析器做黄金向量回归。

用法
----
    python tools/extract_endpoints_json.py \
        --hpp  ../ODrive/Firmware/autogen/endpoints.hpp \
        --out  tests/data/endpoints_v8.json

    # 也可直接指向任何 ODrive checkout
    python tools/extract_endpoints_json.py --hpp /path/to/ODrive/Firmware/autogen/endpoints.hpp

输出
----
夹具文件 + 一份统计摘要（端点数、路径字节数、arena 预算），用于核对文档里的数字。
"""

import argparse
import json
import os
import re
import sys

C_STRING_RE = re.compile(r'^\s*"(.*)"\s*,?\s*$')


def extract_fragments(hpp_path):
    """取出文件里所有顶层 C 字符串字面量的内容（已反转义）。"""
    frags = []
    with open(hpp_path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = C_STRING_RE.match(line)
            if not m:
                continue
            frags.append(m.group(1).encode().decode("unicode_escape"))
    return frags


def build_json(frags):
    """拼接分片并补上顶层数组括号。"""
    body = "".join(frags).rstrip().rstrip(",")
    return "[" + body + "]"


def flatten(doc):
    """递归展开 members / inputs / outputs，返回 [(path, id, type, access)]。"""
    out = []

    def walk(node, prefix):
        name = node.get("name", "")
        full = (prefix + "." + name) if (prefix and name) else (prefix or name)
        if "id" in node:
            out.append((full, node["id"], node.get("type"), node.get("access")))
        for member in node.get("members") or []:
            walk(member, full)
        for key in ("inputs", "outputs"):
            for member in node.get(key) or []:
                walk(member, full)

    for element in doc:
        walk(element, "")
    return out


def arena_budget(paths, entry_bytes=8, paths_only=None):
    """arena = N × entry_bytes + 路径池（每条含 NUL）。"""
    pool = sum(len(p) + 1 for p in paths)
    return len(paths) * entry_bytes + pool, pool


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hpp", required=True, help="路径：Firmware/autogen/endpoints.hpp")
    ap.add_argument("--out", default="tests/data/endpoints_v8.json", help="输出夹具路径")
    ap.add_argument("--indent", type=int, default=None,
                    help="夹具缩进（默认不缩进，保证与设备回送的字节完全一致）")
    args = ap.parse_args()

    if not os.path.isfile(args.hpp):
        print(f"error: 找不到 {args.hpp}", file=sys.stderr)
        return 2

    frags = extract_fragments(args.hpp)
    if not frags:
        print("error: 未从文件里解析出任何字符串字面量", file=sys.stderr)
        return 2

    raw = build_json(frags)
    doc = json.loads(raw)                      # 顺带验证结构合法
    eps = flatten(doc)

    # 夹具必须与设备回送的字节一致 → 默认原样保存（不缩进、不重排）
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "wb") as fh:
        fh.write(raw.encode("utf-8"))

    paths = sorted(p for p, _, _, _ in eps)
    budget_all, pool = arena_budget(paths)
    max_path = max(len(p) for p in paths)

    req = [
        "axis0.motor.config.gear_ratio",
        "axis0.motor.config.torque_constant",
        "axis0.controller.config.mit_max_pos",
        "axis0.controller.config.mit_max_vel",
        "axis0.controller.config.mit_max_torque",
        "axis0.controller.config.mit_max_kp",
        "axis0.controller.config.mit_max_kd",
        "axis0.requested_state",
        "axis0.current_state",
        "axis0.config.can.node_id",
        "axis0.config.can.heartbeat_rate_ms",
        "can.config.break_timeout",
    ]
    missing = [p for p in req if p not in set(paths)]
    budget_req, _ = arena_budget(req)

    types = {}
    for _, _, t, _ in eps:
        types[t] = types.get(t, 0) + 1

    print(f"fragments            : {len(frags)}")
    print(f"json bytes           : {len(raw)}")
    print(f"endpoints (flattened): {len(eps)}")
    print(f"path pool bytes      : {pool}")
    print(f"max path len         : {max_path}")
    print(f"arena RETAIN_ALL     : {budget_all} B ({budget_all/1024:.1f} KB)")
    print(f"arena required 12    : {budget_req} B")
    print(f"required set missing : {missing if missing else 'none'}")
    print(f"types                : {dict(sorted(types.items(), key=lambda kv: str(kv[0])))}")
    print(f"written              : {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
