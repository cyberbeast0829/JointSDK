#!/usr/bin/env python3
"""
CLI / 示例的**文本输出**静态检查（防"乱码"复发）。

背景：工程里的中文都是 UTF-8 字面量，而 Windows 控制台默认 CP936。修法是
把 CLI 的输出统一走 `tools/jsdk_cli/cli_text.h`（控制台转码、管道原样 UTF-8）。
本脚本挡住两类回退：

1. **绕过 `cli_text`**：`tools/jsdk_cli/*.c` 里再出现裸的
   `printf/fprintf/puts/fputs/fputc`（`cli_text.c` 自己实现这些包装，豁免）。
   绕过去就等于"控制台上又乱了"，而且**只在中文控制台上能看出来**。

2. **输出里含 CP936 表示不了的字符**：这类字符在中文控制台上会变成 `?`
   （例子：`⚠` U+26A0、`✓` U+2713）。注释里随便写，**字符串字面量里不行**。

用法：`python tools/check_cli_text.py [仓库根]`，退出码非 0 = 发现问题。
由 ctest 的目标 `cli_text_lint` 调用（见 CMakeLists.txt）。
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

# 目标控制台代码页。选 CP936 的理由：本项目的现场环境就是中文 Windows；
# 要求"输出字符在 CP936 里可表示"能挡住 ⚠/✓/emoji 这类会变成 '?' 的东西。
TARGET_CODEPAGE = "cp936"

CLI_DIR = "tools/jsdk_cli"
EXAMPLE_GLOB = "examples/*.[ch]"

# cli_text.c 自己就是要包装这些函数，豁免
EXEMPT_C = {"cli_text.c"}

BARE_CALL = re.compile(r"(?<![A-Za-z0-9_])(printf|fprintf|puts|fputs|fputc)\s*\(")

# C 字符串字面量（含转义），够用即可：不处理原始字符串/多行拼接的边界情况
C_STRING = re.compile(r'"((?:[^"\\]|\\.)*)"')
# C 的"通用字符名"转义 `\uXXXX` / `\UXXXXXXXX`：源码里是 ASCII，运行期却是真字符
C_UCN = re.compile(r"\\[uU]([0-9a-fA-F]{4,8})")


def _unescape_probe(ch: str, where: str, lineno: int, problems: list[str]) -> None:
    """单个字符是否能在目标代码页里表示（不能则记一条）。"""
    if ord(ch) < 128:
        return
    try:
        ch.encode(TARGET_CODEPAGE)
    except UnicodeEncodeError:
        problems.append(
            f"{where}:{lineno}: 输出字符串里的 {ch!r} (U+{ord(ch):04X}) 在 "
            f"{TARGET_CODEPAGE} 里表示不了 → 中文控制台上会变成 '?'"
        )


def strip_comments(code: str) -> str:
    """
    去掉 `//` 与 `/* */` 注释，但**保留字符串字面量里的内容**。

    ⚠ 不能简单地按 `//` 切行：`printf("http://x")` 里的 `//` 在字面量内部，
      切了就漏检（还会把行尾真实代码当成注释）。所以用一个小状态机。
    """
    out: list[str] = []
    i = 0
    n = len(code)
    in_str = False
    while i < n:
        c = code[i]
        if in_str:
            out.append(c)
            if c == "\\" and i + 1 < n:
                out.append(code[i + 1])
                i += 2
                continue
            if c == '"':
                in_str = False
            i += 1
            continue
        if c == '"':
            in_str = True
            out.append(c)
            i += 1
            continue
        if c == "/" and i + 1 < n and code[i + 1] == "/":
            break                                   # 行注释：后面都不用看了
        if c == "/" and i + 1 < n and code[i + 1] == "*":
            j = code.find("*/", i + 2)
            if j < 0:
                break
            i = j + 2
            continue
        out.append(c)
        i += 1
    return "".join(out)


def check_bare_calls(root: Path) -> list[str]:
    problems: list[str] = []
    for path in sorted((root / CLI_DIR).glob("*.c")):
        if path.name in EXEMPT_C:
            continue
        for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            for match in BARE_CALL.finditer(strip_comments(line)):
                problems.append(
                    f"{path.relative_to(root)}:{lineno}: 裸 `{match.group(1)}` "
                    f"→ 应改用 cli_text.h 的对应包装（控制台上不会乱码）"
                )
    return problems


def check_charset(root: Path) -> list[str]:
    problems: list[str] = []
    targets = sorted((root / CLI_DIR).glob("*.c")) + sorted(root.glob(EXAMPLE_GLOB))
    for path in targets:
        where = str(path.relative_to(root))
        for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            for literal in C_STRING.findall(strip_comments(line)):
                for ch in literal:
                    _unescape_probe(ch, where, lineno, problems)
                # `"\u26a0"` 这种写法源码里全是 ASCII，运行期才是 ⚠
                for hexcode in C_UCN.findall(literal):
                    try:
                        _unescape_probe(chr(int(hexcode, 16)), where, lineno, problems)
                    except ValueError:
                        pass
    return problems


def main(argv: list[str]) -> int:
    # ⚠ 报告里可能含 cp936 表示不了的字符（正是本检查要找的东西）——
    #   控制台按 GBK 解码时 print() 会直接 **UnicodeEncodeError**，于是
    #   检查器在“找到问题”的那一刻自己崩掉（真发生过，看不出结论）。
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

    root = Path(argv[1]) if len(argv) > 1 else Path(__file__).resolve().parent.parent
    problems = check_bare_calls(root) + check_charset(root)

    if problems:
        print("check_cli_text: 发现 %d 个问题" % len(problems))
        for p in problems:
            print("  " + p)
        return 1
    print("check_cli_text: OK（CLI 输出都走 cli_text；字符串字面量都在 "
          f"{TARGET_CODEPAGE} 里可表示）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
