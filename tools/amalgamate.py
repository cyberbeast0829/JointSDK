#!/usr/bin/env python3
"""把 SDK 合并成"两个文件"（`jsdk_can_amalgam.h` + `jsdk_can_amalgam.c`）。

为什么需要它
------------
MCU / Keil / IAR / 单文件导入的场景下，把 20 多个 `.c` 逐个加进工程既麻烦又易错
（漏一个就是 `undefined reference`，而且报错点在客户那边）。合并后客户只要加两个文件。

只合并"可移植核心"
------------------
默认生成的 `.c` **只依赖 `<string.h>` 与 `<math.h>`**：不含任何操作系统后端。

    python tools/amalgamate.py                     # MCU：纯核心
    python tools/amalgamate.py --with virtual      # 主机自测：加虚拟后端 + 设备模型
    python tools/amalgamate.py --with slcan        # 串口：slcan（POSIX）
    python tools/amalgamate.py --with heap         # 加 jsdk_context_create/free（需要 malloc）
    python tools/amalgamate.py --with virtual,heap

`socketcan` / `pcan` 两个桌面后端**不参与合并**：它们要么是 Linux 内核接口、
要么要 `dlopen` 平台库，MCU 用不上；要用它们请走 CMake（`find_package` / pkg-config）。

陈旧守卫（重要）
----------------
文件清单**不是手写的**，而是 glob 出 `src/core/` 与 `src/proto_cyberbeast/` 下的全部
`.c`，再与"已知可选件"白名单对照。将来有人加了新文件而忘了分类，这个脚本会**报错**
而不是静默漏掉 —— 否则客户会拿到一个缺文件的 amalgamation，且症状是链接错误。
同时检查跨文件重复的 `static` 函数名（合并后它们共享一个作用域）。
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# --- 公开头（进 amalgam.h） ---
PUBLIC_HEADERS = [
    "include/joint_sdk/joint_sdk.h",
    "include/joint_sdk/jsdk_hal_builtin.h",
]

# --- 内部头（进 amalgam.c 最前面；顺序 = 依赖顺序） ---
INTERNAL_HEADERS = [
    "src/jsdk_internal.h",
    "src/proto_cyberbeast/cb_frame.h",
    "src/proto_cyberbeast/cb_mit.h",
    "src/proto_cyberbeast/cb_ctrl.h",
    "src/proto_cyberbeast/cb_query.h",
    "src/proto_cyberbeast/cb_heartbeat.h",
    "src/proto_cyberbeast/cb_param.h",
    "src/proto_cyberbeast/cb_jsondesc_fetch.h",
    "src/proto_cyberbeast/cb_desc_cache.h",
    "src/jsdk_core_internal.h",
    "src/hal/hal_handle.h",
    "src/hal/hal_slcan_codec.h",
    "src/hal/sim_device.h",
    "src/hal/hal_virtual_internal.h",
]

# --- 核心源（永远合并） ---
CORE_DIRS = ["src/proto_cyberbeast", "src/core"]

# --- 可选（必须显式 --with 打开）；不在白名单里的 core 文件会让脚本报错 ---
OPTIONAL = {
    "heap":    ["src/hal/heap_optional.c"],
    "virtual": ["src/hal/hal_virtual.c", "src/hal/sim_device.c"],
    "slcan":   ["src/hal/hal_slcan.c", "src/hal/hal_slcan_codec.c", "src/hal/hal_common.c"],
}

# 任何 --with 打开时都要带上 hal_common.c（jsdk_hal_close + 未编译后端的占位）
OPTIONAL_COMMON = "src/hal/hal_common.c"

# 这些文件**故意**不参与合并（桌面后端；见文件头说明）
EXCLUDED = [
    "src/hal/hal_socketcan.c",
    "src/hal/hal_pcan.c",
]

# ⚠ 用**前缀**匹配而不是整行匹配：真实代码里存在带行尾注释的写法，例如
#     #include "jsdk_internal.h"   /* jsdk_jsondesc_t / jsdk_ep_store_t */
# 早期版本的整行正则（要求引号后就是行尾）
# 漏掉了这一行，于是生成的 .c 里留着一个指向不存在文件的 include。
LOCAL_INCLUDE_RE = re.compile(r'^\s*#\s*include\s+"([^"]+)"')
SYS_INCLUDE_RE = re.compile(r'^\s*#\s*include\s+<([^>]+)>')
STATIC_FUNC_RE = re.compile(
    r'^static\s+(?:[A-Za-z_][A-Za-z0-9_]*\s+|\*+\s*)*([A-Za-z_][A-Za-z0-9_]*)\s*\(')


def read(path: str) -> str:
    with open(os.path.join(ROOT, path), encoding="utf-8") as f:
        return f.read()


def strip_local_includes(text: str):
    """返回 (去掉本地 include 的正文, 收集到的系统 include 集合)。"""
    sys_includes = []
    out_lines = []
    for line in text.splitlines():
        m = SYS_INCLUDE_RE.match(line)
        if m:
            if m.group(1) not in sys_includes:
                sys_includes.append(m.group(1))
            continue
        if LOCAL_INCLUDE_RE.match(line):
            continue          # 已经并进来了，不需要再 include
        out_lines.append(line)
    return "\n".join(out_lines), sys_includes


def relpath(p: str) -> str:
    """相对仓库根、**统一用正斜杠**。

    Windows 上 `os.path.relpath()` 给的是 `src\\hal\\x.c`，而下面白名单写的是
    正斜杠 —— 不归一化的话，陈旧守卫会把**每一个**文件都报成"未分类"。
    """
    return os.path.relpath(p, ROOT).replace(os.sep, "/")


def collect_sources(with_opts):
    # 核心：按目录 glob（这样"新加了文件却忘了分类"会被下面发现）
    core = []
    for d in CORE_DIRS:
        core.extend(sorted(relpath(p) for p in glob.glob(os.path.join(ROOT, d, "*.c"))))

    known_optional = {f for files in OPTIONAL.values() for f in files} | {OPTIONAL_COMMON}
    misc = {relpath(p) for p in glob.glob(os.path.join(ROOT, "src", "hal", "*.c"))}

    unknown = misc - known_optional - set(EXCLUDED)
    if unknown:
        sys.stderr.write(
            "✗ src/hal/ 里有未分类的源文件（不知道要不要合并）：\n"
            + "".join("    %s\n" % u for u in sorted(unknown))
            + "  → 请在 tools/amalgamate.py 的 OPTIONAL 或 EXCLUDED 里明确归类。\n")
        sys.exit(2)

    chosen = list(core)
    chosen += [f for f in OPTIONAL["heap"] if "heap" in with_opts]
    chosen += [f for f in OPTIONAL["virtual"] if "virtual" in with_opts]
    chosen += [f for f in OPTIONAL["slcan"] if "slcan" in with_opts]
    if with_opts:
        chosen.append(OPTIONAL_COMMON)
    return core, chosen


def check_duplicate_statics(files):
    seen = {}
    dupes = []
    for path in files:
        for line in read(path).splitlines():
            m = STATIC_FUNC_RE.match(line)
            if m:
                name = m.group(1)
                if name in seen and seen[name] != path:
                    dupes.append((name, seen[name], path))
                seen.setdefault(name, path)
    return dupes


def build(with_opts, out_dir):
    core, chosen = collect_sources(with_opts)
    for path in PUBLIC_HEADERS + INTERNAL_HEADERS + chosen:
        if not os.path.exists(os.path.join(ROOT, path)):
            sys.stderr.write("✗ 缺文件：%s\n" % path)
            sys.exit(2)

    dupes = check_duplicate_statics(chosen)
    if dupes:
        sys.stderr.write("✗ 合并后会撞名（同一作用域里的 static 函数）：\n")
        for name, a, b in dupes:
            sys.stderr.write("    %s: %s ↔ %s\n" % (name, a, b))
        sys.stderr.write("  → 合并成一个 .c 之后它们共享文件作用域，必须改名。\n")
        sys.exit(2)

    # ---------- .h ----------
    h_parts = []
    h_parts.append("""\
/* ==========================================================================
 * jsdk_can_amalgam.h — CyberBeast CAN/CAN-FD 关节 SDK 的单文件头
 *
 * ⚠ **本文件由 tools/amalgamate.py 生成，请勿手工修改**。
 *    改代码请改 include/joint_sdk/ 下的原始头，然后重新生成：
 *        python tools/amalgamate.py <同样的 --with 选项>
 *
 * 用法（MCU / 单文件工程）：
 *     #include "jsdk_can_amalgam.h"   // 本文件
 *     再把 jsdk_can_amalgam.c 加进工程即可。
 *
 * 合并进来的公开头（顺序即原文件顺序）：
""")
    for p in PUBLIC_HEADERS:
        h_parts.append(" *   - %s\n" % p)
    h_parts.append("""\
 *
 * 许可证：专有（仅授权客户随产品分发）—— 与 include/ 下的原件一致。
 * ========================================================================== */

#ifndef JSDK_CAN_AMALGAM_H
#define JSDK_CAN_AMALGAM_H

""")

    h_sys = []
    for path in PUBLIC_HEADERS:
        body, sys_inc = strip_local_includes(read(path))
        for s in sys_inc:
            if s not in h_sys:
                h_sys.append(s)
        h_parts.append("/* ---------- %s ---------- */\n" % path)
        h_parts.append(body.strip("\n") + "\n\n")

    # 系统头提到最前（在 guard 之后）
    head = "".join(h_parts)
    guard_end = head.index("#define JSDK_CAN_AMALGAM_H\n") + len("#define JSDK_CAN_AMALGAM_H\n")
    sys_block = "".join("#include <%s>\n" % s for s in sorted(h_sys))
    h_text = head[:guard_end] + "\n" + sys_block + "\n" + head[guard_end:] + \
        "\n#endif /* JSDK_CAN_AMALGAM_H */\n"

    # ---------- .c ----------
    c_parts = []
    c_sys = []
    c_parts.append("""\
/* ==========================================================================
 * jsdk_can_amalgam.c — CyberBeast CAN/CAN-FD 关节 SDK 的单文件实现
 *
 * ⚠ **本文件由 tools/amalgamate.py 生成，请勿手工修改**（改代码请改 src/ 下原件后重跑）。
 *
 * 合并选项（生成时确定，改选项要重新生成并**重新编译**）：
""")
    for opt in ("heap", "virtual", "slcan"):
        c_parts.append(" *   %-8s %s\n" % (opt, "已包含" if opt in with_opts else "未包含"))
    c_parts.append("""\
 *
 * 合并进来的文件：
""")
    for path in INTERNAL_HEADERS + chosen:
        c_parts.append(" *   - %s\n" % path)
    c_parts.append("""\
 * ========================================================================== */

#include "jsdk_can_amalgam.h"

""")

    # ⚠ 先把所有文件过一遍（去本地 include、收集系统 include），**再**拼输出：
    #   早先的写法是"边拼边收集"，结果 `#include <string.h>` 只写进了 .h、
    #   没写进 .c → 生成的 .c 自己在 `strlen`/`memcpy` 上编译失败。
    stripped = []
    for path in INTERNAL_HEADERS + chosen:
        body, sys_inc = strip_local_includes(read(path))
        for s in sys_inc:
            if s not in c_sys:
                c_sys.append(s)
        stripped.append((path, body))

    # 后端存在性宏：hal_common.c 用它决定是否提供"未编译"占位
    if "virtual" in with_opts:
        c_parts.append("#define JSDK_HAL_HAVE_VIRTUAL 1\n")
    if "slcan" in with_opts:
        c_parts.append("#define JSDK_HAL_HAVE_SLCAN 1\n")
    if with_opts:
        c_parts.append("\n")

    for s in sorted(c_sys):
        c_parts.append("#include <%s>\n" % s)
    c_parts.append("\n")

    for path, body in stripped:
        c_parts.append("\n/* ======================== %s ======================== */\n" % path)
        c_parts.append(body.strip("\n") + "\n")

    c_text = "".join(c_parts)

    os.makedirs(out_dir, exist_ok=True)
    h_out = os.path.join(out_dir, "jsdk_can_amalgam.h")
    c_out = os.path.join(out_dir, "jsdk_can_amalgam.c")
    with open(h_out, "w", encoding="utf-8", newline="\n") as f:
        f.write(h_text)
    with open(c_out, "w", encoding="utf-8", newline="\n") as f:
        f.write(c_text)

    print("核心源 %d 个（%s）+ 可选 %d 个" % (len(core), ", ".join(CORE_DIRS),
                                            len(chosen) - len(core)))
    print("内部头 %d 个，公开头 %d 个" % (len(INTERNAL_HEADERS), len(PUBLIC_HEADERS)))
    print("选项：%s" % (", ".join(sorted(with_opts)) if with_opts else "（无，纯核心）"))
    print("系统头：%s" % ", ".join(sorted(set(c_sys) | set(h_sys))))
    print("写出 %s（%d 字节）" % (h_out, len(h_text)))
    print("写出 %s（%d 字节）" % (c_out, len(c_text)))


def main():
    ap = argparse.ArgumentParser(description="生成单文件 amalgamation")
    ap.add_argument("--with", default="", dest="with_opts",
                    help="逗号分隔的可选件：heap,virtual,slcan")
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "dist"),
                    help="输出目录（默认 dist/）")
    args = ap.parse_args()

    opts = {s.strip() for s in args.with_opts.split(",") if s.strip()}
    bad = opts - set(OPTIONAL)
    if bad:
        sys.stderr.write("✗ 未知选项：%s（可用：%s）\n" % (", ".join(sorted(bad)),
                                                        ", ".join(sorted(OPTIONAL))))
        return 2
    build(opts, args.out_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
