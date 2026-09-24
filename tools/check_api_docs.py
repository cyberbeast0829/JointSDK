#!/usr/bin/env python3
"""公共 API 文档守卫：**每一个 `JSDK_API` 声明都必须有紧邻的 Doxygen 注释**。

为什么需要它（v0.34，来自 BACKLOG §1.2 的 A12）：
`jsdk_unit_scale_*` 四个函数在公共头里**零注释**，而它们的语义与 EtherCAT 版**同名不同义**
（CAN 版是恒等映射、EtherCAT 版是 counts→rad）。解释只写在 `src/core/jsdk_units.c` 里 ——
**客户看不到的东西等于没有**，于是从 EtherCAT 版迁过来的人会照搬并写出错的换算。
一句话：*文档缺口不是“少写了几行”，而是会把调用者引向错误结论*。
（同类先例：`heartbeat_rate_ms` 恒为 0 会让现场以为“心跳被关了”。）

判定规则（刻意简单、可复现）：
  1. 只看 `include/joint_sdk/*.h`（客户可见的头文件）；
  2. 一行里出现 `JSDK_API` 且含 `(` 即视为一条声明（多行声明由续行归并）；
  3. **一组紧邻的声明**可以共用一份文档：只要这一组之前有一份以 `*/` 结尾的注释块；
  4. 组与组之间被空行/其它代码隔开，则重新要求文档。

用法：
  python tools/check_api_docs.py            # 检查（有缺口 → 退出码 1）
  python tools/check_api_docs.py --list     # 只列缺口（不用于 CI 判定）
"""

from __future__ import annotations

import io
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HEADERS = ["include/joint_sdk/joint_sdk.h", "include/joint_sdk/jsdk_hal_builtin.h"]

#: 有意不要求逐条文档的声明（必须写清理由；宁可留空也不要“凑一句废话”）。
ALLOWED: set[str] = set()

#: 允许**共用同一份文档块**的声明组：必须**显式列出、可审计**。
#:
#: ⚠ 为什么不用“紧邻的声明都算有文档”这种启发式：那样把一个**新函数**贴在已有文档
#:   旁边，守卫会静默放过它 —— 本脚本的负向测试就是这么漏掉了一条故意没文档的声明的
#:   （插入 `jsdk_sabotage_probe` 后仍然报 OK）。而“新加的 API 没有文档”正是 A12 要防的
#:   那件事，所以这里宁可多写几行白名单。
#:
#: 每条 = (声明名集合, 为什么可以共用一份文档)。集合**不允许**在事后被“悄悄加一条”：
#: 加成员后这组不再等于登记的那组，多出来的那条会被报出来。
GROUPS: list[tuple[frozenset[str], str]] = [
    (frozenset({"jsdk_context_create", "jsdk_context_free"}),
     "堆模式成对构造/释放（heap_optional.c，同一份说明）"),
    (frozenset({"jsdk_joint_request_enable", "jsdk_joint_request_disable",
                "jsdk_joint_request_fault_reset", "jsdk_joint_set_mode"}),
     "请求类四个入口共用同一份“排队语义 + 关机序列”说明"),
    (frozenset({"jsdk_joint_set_target_position_rad", "jsdk_joint_set_target_velocity_rad_s",
                "jsdk_joint_set_target_torque_Nm"}),
     "物理量入口三件套（同一份模式/单位说明）"),
    (frozenset({"jsdk_joint_set_target_position", "jsdk_joint_set_target_velocity",
                "jsdk_joint_set_target_torque"}),
     "raw 入口三件套（同一份“/1000 定点化”说明）"),
    (frozenset({"jsdk_joint_set_limits", "jsdk_joint_set_current_A"}),
     "限制量两个入口（共用一段说明；后者自带行内注释）"),
    (frozenset({"jsdk_joint_sdo_create_by_name", "jsdk_joint_sdo_state"}),
     "SDO 建槽 / 查状态（同一段说明）"),
    (frozenset({"jsdk_joint_sdo_data", "jsdk_joint_sdo_data_size"}),
     "裸字节缓冲区与长度（成对）"),
    (frozenset({"jsdk_joint_sdo_read", "jsdk_joint_sdo_write"}),
     "发起读写（成对，非阻塞）"),
    (frozenset({"jsdk_joint_param_get", "jsdk_joint_param_set"}),
     "通用类型化读写（成对）"),
    (frozenset({"jsdk_joint_param_get_f32", "jsdk_joint_param_set_f32",
                "jsdk_joint_param_get_u32", "jsdk_joint_param_set_u32",
                "jsdk_joint_param_get_i32", "jsdk_joint_param_get_bool"}),
     "类型化便利包装（同一份“实际端点多为整型”说明）"),
    (frozenset({"jsdk_group_enable", "jsdk_group_disable"}),
     "成组使能/失能（同一份“无广播”语义说明）"),
    (frozenset({"jsdk_status_string", "jsdk_axis_state_string",
                "jsdk_mode_string", "jsdk_ep_type_string"}),
     "四个“枚举 → 文本”函数（同一份“永不返回 NULL”说明）"),
]
GROUPED: set[str] = set().union(*[g for g, _ in GROUPS]) if GROUPS else set()
GROUP_REASONS = {g: why for g, why in GROUPS}


def declared_name(line: str) -> str:
    """从一行声明里取函数名（`JSDK_API <ret> <name>(` 中的 name）。"""
    m = re.search(r"JSDK_API\b.*?(\bjsdk_[A-Za-z0-9_]+)\s*\(", line)
    if m:
        return m.group(1)
    m = re.search(r"\bjsdk_[A-Za-z0-9_]+\s*\(", line)
    return m.group(0)[:-1].strip() if m else line.strip()[:48]


def scan(path: str) -> tuple[list[tuple[int, str]], list[tuple[int, list[str]]]]:
    """返回 (缺文档的声明, 被相邻文档“顺带覆盖”的多声明组)。

    第二条是本脚本最关键的一处：一份文档块覆盖了 **>1** 条声明时，只有显式登记在
    `GROUPS` 里的那几条算合法；其余的说明它们是“贴在别人旁边混过检查”的新声明。
    """
    lines = io.open(path, encoding="utf-8").read().splitlines()
    missing: list[tuple[int, str]] = []
    adopted: list[tuple[int, list[str]]] = []
    documented = False          # 当前这一组声明是否已被文档覆盖
    in_block = False            # 正在一份 /* ... */ 注释里
    group: list[tuple[int, str]] = []   # 当前这一组声明（行号, 名字）

    def flush() -> None:
        """一组声明结束 → 判定"""
        if not group:
            return
        first = group[0][0]
        names = [n for _, n in group]
        if not documented:
            missing.extend(group)
        elif len(group) > 1:
            extra = [n for n in names if n not in GROUPED and n not in ALLOWED]
            if extra:
                adopted.append((first, extra))
        group.clear()

    for idx, raw in enumerate(lines, start=1):
        line = raw.strip()

        if in_block:
            if line.endswith("*/"):
                in_block = False
                documented = True      # 注释块结束 → 覆盖紧随的声明组
            continue
        if line.startswith("/*"):
            flush()
            # 单行 `/* ... */` 立即成文档；多行则等 `*/`（在 in_block 分支里置位）
            documented = line.endswith("*/")
            in_block = not line.endswith("*/")
            continue
        if line.startswith("//") or line.startswith("#"):
            continue

        if not line:
            flush()
            documented = False         # 空行 = 组结束
            continue

        if "JSDK_API" in line and "(" in line:
            group.append((idx, declared_name(line)))
            continue

        # 续行（多行声明/参数表）：不结束组
        if (not line.endswith(";") and line.endswith((",", ")"))) or line.startswith(")"):
            continue

        # 其它代码（typedef、宏、结构体成员……）→ 组结束
        flush()
        documented = False

    flush()
    return missing, adopted


def main(argv: list[str]) -> int:
    list_only = "--list" in argv
    problems: list[tuple[str, int, str]] = []
    adopted: list[tuple[str, int, list[str]]] = []

    for rel in HEADERS:
        path = os.path.join(REPO, rel)
        if not os.path.exists(path):
            continue
        miss, adopt = scan(path)
        for line_no, name in miss:
            if name in ALLOWED:
                continue
            problems.append((rel, line_no, name))
        for line_no, names in adopt:
            adopted.append((rel, line_no, names))

    if problems:
        print(f"check_api_docs: {len(problems)} \u5904 JSDK_API 缺文档")
        for rel, line_no, name in problems:
            print(f"  {rel}:{line_no}  {name}")
        if not list_only:
            print("（公共头里没有注释的声明，对客户等于不存在；见 docs/BACKLOG.zh-CN.md §1.2）")
            return 1
    if adopted:
        print(f"check_api_docs: {len(adopted)} \u5904声明“蹭”了相邻声明\u7684文档块")
        for rel, line_no, names in adopted:
            print(f"  {rel}:{line_no}  {', '.join(names)}")
        if not list_only:
            print("（多声明共一份文档**只允许 GROUPS 里显式登记的那几组**，"
                  "每条都写明了理由；"
                  "新声明要么自己写文档，要么把整组加进 GROUPS 并说明为什么可以共用）")
            return 1

    print("check_api_docs: OK（所有 JSDK_API 声明都有文档）")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
