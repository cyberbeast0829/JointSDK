"""无硬件自检 — 在 virtual 后端上验证 MCP server 的协议层与安全语义。

⚠ 这个脚本的定位与 `tools/check_all.sh` 的一致：
**能在没有关节模组的情况下，证明 server 本身是通的**。

它验证的不是"关节能动了"，而是：
- JSON-RPC 信封正确（initialize / tools/list / tools/call）
- 工具 schema 与实现一一对应（不会调用不存在的工具）
- **写操作返回 requested/value/verified 三字段**（不许把请求值当结果）
- 安全闸真的会拒绝（没给 confirm 时必须 refuse）
- 错误是结构化的（ok:false + error + hint），不是 traceback

跑法 / Usage::

    python -m jsdk_joint_mcp.selftest
"""

from __future__ import annotations

import json
import sys
from typing import Any

from .server import McpServer

# ⚠ 本仓库在 Windows 控制台默认是 cp936，输出含非 cp936 字符（如 ↔）
#   会直接 UnicodeEncodeError 崩掉 —— 而且崩在**打印点**，看起来像测试失败。
#   统一把 stdout/stderr 换成 UTF-8（errors=replace 保证不再崩）。
try:  # pragma: no cover - 平台相关
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001
    pass

PASS, FAIL = "ok  ", "FAIL"
_results: list[tuple[bool, str]] = []


def check(cond: bool, label: str, detail: str = "") -> bool:
    _results.append((bool(cond), label))
    mark = PASS if cond else FAIL
    line = f"[{mark}] {label}"
    if detail and not cond:
        line += f"\n         {detail}"
    print(line)
    return bool(cond)


def as_json(result: dict[str, Any]) -> dict[str, Any]:
    """把 MCP 工具结果信封里的 JSON 文本解回来。"""
    try:
        text = result["content"][0]["text"]
        return json.loads(text)
    except Exception as exc:                                # noqa: BLE001
        return {"_unparsable": str(exc), "_raw": result}


def main() -> int:
    srv = McpServer()

    # ---------------------------------------------------------------- 1 信封
    print("\n=== [1] JSON-RPC 信封 ===")
    r = srv.handle({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {}})
    check(r is not None and r.get("id") == 1, "initialize 回带同一个 id")
    res = (r or {}).get("result", {})
    check(res.get("protocolVersion") == "2024-11-05", "protocolVersion 正确")
    check(res.get("serverInfo", {}).get("name") == "jsdk-joint-mcp", "serverInfo.name")

    r = srv.handle({"jsonrpc": "2.0", "id": 2, "method": "tools/list"})
    tools = (r or {}).get("result", {}).get("tools", [])
    check(len(tools) >= 12, f"tools/list 返回 {len(tools)} 个工具")

    r = srv.handle({"jsonrpc": "2.0", "id": 3, "method": "no/such/method"})
    check("error" in (r or {}), "未知方法返回 JSON-RPC error")

    # 通知不响应
    n = srv.handle({"jsonrpc": "2.0", "method": "notifications/initialized"})
    check(n is None, "通知（无 id）不产生响应")

    # ------------------------------------------------------- 2 schema 一致性
    print("\n=== [2] 工具 schema 与实现一致性 ===")
    names = [t["name"] for t in tools]
    check(len(names) == len(set(names)), "工具名无重复")
    missing = []
    for n in names:
        attr = n.removeprefix("jsdk_")
        if not callable(getattr(srv.reg, attr, None)):
            missing.append(n)
    check(not missing, "每个声明的工具都有实现", f"缺实现: {missing}")
    for t in tools:
        if not t.get("description"):
            check(False, f"{t['name']} 有 description")
        sch = t.get("inputSchema") or {}
        if sch.get("type") != "object":
            check(False, f"{t['name']} inputSchema 是 object")

    # -------------------------------------------------- 3 未知工具 / 坏参数
    print("\n=== [3] 错误是结构化的 ===")
    r = as_json(srv._call_tool({"name": "no_such_tool", "arguments": {}}))
    check(r.get("ok") is False and r.get("error") == "UNKNOWN_TOOL",
          "未知工具 → ok:false / UNKNOWN_TOOL", str(r))

    r = as_json(srv._call_tool({"name": "jsdk_read_param", "arguments": {"node": 1}}))
    check(r.get("ok") is False and r.get("error") == "BAD_ARGUMENTS",
          "缺参数 → ok:false / BAD_ARGUMENTS", str(r))
    check(bool(r.get("hint")), "错误带 hint（给 agent 的下一步）")

    # ------------------------------------------------------ 4 安全闸（关键）
    print("\n=== [4] 安全闸：没确认必须拒绝 ===")
    r = as_json(srv._call_tool({
        "name": "jsdk_set_zero",
        "arguments": {"node": 1, "iface": "virtual"},
    }))
    check(r.get("ok") is False and r.get("error") == "SAFETY_REFUSED",
          "set_zero 未确认 → SAFETY_REFUSED", str(r))
    check(r.get("kind") == "safety", "SAFETY_REFUSED 的 kind=safety")

    r = as_json(srv._call_tool({
        "name": "jsdk_fault_reset",
        "arguments": {"node": 1, "iface": "virtual"},
    }))
    check(r.get("ok") is False and r.get("error") == "SAFETY_REFUSED",
          "fault_reset 未确认 → SAFETY_REFUSED", str(r))

    # ------------------------------------------- 5 缺失会话（不该崩）
    print("\n=== [5] 没有会话时给结构化错误 ===")
    srv2 = McpServer()
    r = as_json(srv2._call_tool({
        "name": "jsdk_get_state",
        "arguments": {"node": 9, "iface": "virtual"},
    }))
    check("ok" in r, "get_state 无硬件时返回结构化结果（不抛异常）", str(r)[:300])

    # ---------------------------------------------------------- 6 装箱语义
    print("\n=== [6] 按端点类型装箱（不猜 Python 类型）===")
    from .session import _box_value

    check(_box_value("1", "bool") is True, "bool: '1' → True")
    check(_box_value("true", "bool") is True, "bool: 'true' → True")
    check(_box_value("0", "bool") is False, "bool: '0' → False")
    try:
        _box_value("2", "bool")
        check(False, "bool: '2' 应被拒")
    except ValueError:
        check(True, "bool: '2' 被拒（不是静默成 True）")

    check(_box_value("250", "uint16") == 250, "uint16: 250")
    try:
        _box_value("65536", "uint16")
        check(False, "uint16 超值域应被拒")
    except ValueError:
        check(True, "uint16: 65536 被拒（不静默截断成 0）")

    try:
        _box_value("-1", "uint8")
        check(False, "uint8 负值应被拒")
    except ValueError:
        check(True, "uint8: -1 被拒")

    check(abs(_box_value("7.75", "float") - 7.75) < 1e-9, "float: 7.75")
    check(_box_value("0.0824464", "float32") == 0.0824464, "float32 字面量")
    check(_box_value("-5", "int32") == -5, "int32: 负数（CLI 曾经写不了）")

    # ------------------------------------------------------- 7 数值比较容差
    print("\n=== [7] 读回比较按类型容差 ===")
    from .session import _values_equal

    check(_values_equal(7.75, 7.7500001), "float32 往返误差视为相等")
    check(not _values_equal(7.75, 7.8), "明显不同的浮点不等")
    check(_values_equal(250, 250), "整数精确相等")
    check(not _values_equal(250, 0), "整数不等")

    # --------------------------------------------------- 8 遥测统计（新）
    print("\n=== [8] 遥测统计（可核查，不是结论）===")
    from .session import _summarize

    s = _summarize([])
    check(s.get("usable_samples") == 0, "空样本 → usable_samples=0（不是假装正常）")
    check("note" in s, "空样本给 note 叫你先查链路")

    s = _summarize([
        {"t_ms": 0, "pos": 0.0, "age_ms": 5, "axis_error": 0},
        {"t_ms": 100, "pos": 0.5, "age_ms": 6, "axis_error": 0},
        {"t_ms": 200, "pos": 1.0, "age_ms": 7, "axis_error": 0},
    ])
    check(s["usable_samples"] == 3, "3 个样本")
    check(abs(s["pos"]["span"] - 1.0) < 1e-9, "pos span = 1.0")
    check(abs(s["pos"]["first"]) < 1e-9, "pos first 保留")
    check(abs(s["pos"]["last"] - 1.0) < 1e-9, "pos last 保留")
    check("axis_error_values" not in s, "无错误位时不报错误值")

    s = _summarize([
        {"t_ms": 0, "age_ms": 5, "axis_error": 0x800},
        {"t_ms": 100, "age_ms": 6000, "axis_error": 0x800},
        {"t_ms": 200, "error": "timeout"},
    ])
    check(s["errors"] == 1, "数得清有几个坏样本")
    check(s["axis_error_values"] == ["0x800"], "非零错误位浮出来（hex）")
    check(s["age_ms"]["stale_count"] == 1, "age_ms 判新鲜度（6000ms 算 stale）")

    # --------------------------------------------------- 9 装箱边界（新）
    print("\n=== [9] 装箱边界 ===")
    check(_box_value("18446744073709551615", "uint64")
          == 18446744073709551615, "uint64 最大值")
    try:
        _box_value("18446744073709551616", "uint64")
        check(False, "uint64 溢出应被拒")
    except ValueError:
        check(True, "uint64 溢出被拒")
    check(_box_value("0xFA", "uint8") == 250, "支持 0x 十六进制字面量")
    check(_box_value("1e-3", "float") == 0.001, "支持科学计数法")

    # ------------------------------------------- 10 新工具 schema 一致性
    print("\n=== [10] 新增工具的 schema ↔ 实现 ===")
    for name in ("jsdk_calibrate", "jsdk_home", "jsdk_get_telemetry",
                 "jsdk_probe_ranges"):
        attr = name.removeprefix("jsdk_")
        check(callable(getattr(srv.reg, attr, None)), f"{name} 有实现")
        t = next((x for x in tools if x["name"] == name), None)
        check(t is not None and bool(t.get("description")), f"{name} 有 description")

    # ---- 标定/回零的安全闸也必须在碰设备之前 ----
    for name in ("jsdk_calibrate", "jsdk_home"):
        r = as_json(srv._call_tool({
            "name": name, "arguments": {"node": 1, "iface": "virtual"},
        }))
        check(r.get("ok") is False and r.get("error") == "SAFETY_REFUSED",
              f"{name} 未确认 → SAFETY_REFUSED", str(r)[:200])

    # ------------------------------ 11 "跑完 ≠ 成功" 的判定（关键新逻辑）
    print("\n=== [11] 标定收尾：跑完 ≠ 成功 ===")
    from .session import JointSession, SessionConfig

    # ⚠ 这里**不碰设备**：把 _read_* 换成假的，专测判定逻辑。
    js = JointSession(SessionConfig(iface="virtual", node=1))
    js._joint = lambda: object()          # 绕过"必须 configure"的检查

    # 场景 A：一切正常，但 pre_calibrated 还是 false（**正常行为**）
    js._read_state_quiet = lambda: {"ok": True, "current_state": 1, "resting": True}
    js._read_fault_quiet = lambda: {"ok": True, "subsystems": {
        "motor_error": {"value": 0, "hex": "0x0"},
        "encoder_error": {"value": 0, "hex": "0x0"},
    }}
    orig_param_get = js._joint

    class _FakeJoint:
        def param_get(self, path):
            if "pre_calibrated" in path:
                return False
            return 0
    js._joint = lambda: _FakeJoint()

    r = js._state_op_result("calibrate", ok=True, t0=0.0)
    check(r["ok"] is True, "无故障 → ok 保持 true")
    check(r["pre_calibrated_both"] is False, "两个标志都 false")
    # ⚠ 断言要盯**语义关键字**，不要盯整句 —— 文案里的换行/缩进会把整句断言弄假红。
    check("note" in r and "标定生效" in r["note"],
          "明确说明'跑完≠生效'（否则用户会一直重试）")
    check("save" in r["note"], "给出 save 的完整操作步骤")
    check("state" in r and "fault" in r, "收尾证据齐全（state + fault）")

    # 场景 B：跑完但设备有故障位 → 必须翻成 ok=False
    js._read_fault_quiet = lambda: {"ok": True, "subsystems": {
        "motor_error": {"value": 0, "hex": "0x0"},
        "encoder_error": {"value": 2, "hex": "0x2"},
    }}
    r = js._state_op_result("calibrate", ok=True, t0=0.0)
    check(r["ok"] is False, "**有故障位 → ok 翻 false**（回到静息≠成功）")
    check(r["error"] == "DEVICE_FAULT", "错误码 DEVICE_FAULT")
    check("encoder_error" in r["active_faults"], "列出具体是哪一类故障")
    check("不等于成功" in r["hint"], "hint 解释为什么这不算成功")

    # 场景 C：状态完全没动（锁存故障）→ 给"先清故障"的提示
    # ⚠ 场景 B 把 _read_fault_quiet 换成了"有故障"，这里必须**显式重置** ——
    #   否则测的是场景 B 的残留状态（顺序耦合会让断言与代码对不上，
    #   排查时会误以为是被测代码的问题）。
    js._read_fault_quiet = lambda: {"ok": True, "subsystems": {
        "motor_error": {"value": 0, "hex": "0x0"},
        "encoder_error": {"value": 0, "hex": "0x0"},
    }}
    r = js._state_op_result("calibrate", ok=False, t0=0.0,
                            error="device never left idle after requesting state 3")
    check(r["ok"] is False, "失败被如实报告（ok=False）")
    check(r["error"] == "STATE_OP_FAILED",
          "错误码是 STATE_OP_FAILED（不是 DEVICE_FAULT）", str(r.get("error")))
    hint = r["hint"]
    check("锁存故障" in hint, "hint 点出'锁存故障'这个最常见原因")
    check("fault_reset" in hint, "hint 给出可执行的下一步")
    check(r["message"].startswith("device never left idle"),
          "原始错误文本保留（可核查）")

    # 场景 D：回零不该报 pre_calibrated
    r = js._state_op_result("home", ok=True, t0=0.0)
    check("pre_calibrated" not in r, "回零不报 pre_calibrated（与标定区分）")

    # ---------------- 12 AxisState 枚举（真机踩过，必须有牙齿） ----------------
    print("\n=== [12] AxisState 枚举翻译（真机踩过）===")
    from .session import axis_state_name

    # ⚠ 这几个值曾经被写错：6/7 互换、16 缺失 ⇒ 把设备的
    #   ENCODER_OFFSET_CALIBRATION(7) 报成 INDEX_SEARCH，
    #   把 MOTOR_DEADTIME_CALIBRATION(16) 报成 UNKNOWN。
    #   权威来源是 SDK 的 jsdk_can_axis_state_name() / k_can_axis_state[17]。
    check(axis_state_name(1) == "IDLE", "1 = IDLE")
    check(axis_state_name(4) == "MOTOR_CALIBRATION", "4 = MOTOR_CALIBRATION")
    check(axis_state_name(6) == "ENCODER_INDEX_SEARCH",
          "6 = ENCODER_INDEX_SEARCH（不是 OFFSET）", axis_state_name(6))
    check(axis_state_name(7) == "ENCODER_OFFSET_CALIBRATION",
          "7 = ENCODER_OFFSET_CALIBRATION（不是 INDEX_SEARCH）",
          axis_state_name(7))
    check(axis_state_name(14) == "INERTIA_CALIBRATION",
          "14 = INERTIA_CALIBRATION（不是 ANTICOGGING）", axis_state_name(14))
    check(axis_state_name(15) == "ENCODER_LINEARIZATION",
          "15 = ENCODER_LINEARIZATION", axis_state_name(15))
    check(axis_state_name(16) == "MOTOR_DEADTIME_CALIBRATION",
          "**16 = MOTOR_DEADTIME_CALIBRATION**（全标定经过它，缺了会误判跑完）",
          axis_state_name(16))
    check("unknown" in axis_state_name(99).lower(),
          "越界值给 unknown（不崩）", axis_state_name(99))

    # ---------------- 13 防线：返回时轴不在静息态 ----------------
    print("\n=== [13] 防线：C 侧说'跑完'但轴没停 ===")
    js2 = JointSession(SessionConfig(iface="virtual", node=1))
    js2._joint = lambda: type("F", (), {"param_get": lambda s, p: False})()
    # C 侧返回 ok，但状态是 16（非静息）—— 这就是真机踩到的场景
    js2._read_state_quiet = lambda: {
        "ok": True, "current_state": 16,
        "current_state_name": "MOTOR_DEADTIME_CALIBRATION", "resting": False,
    }
    js2._read_fault_quiet = lambda: {"ok": True, "subsystems": {}}
    r = js2._state_op_result("calibrate", ok=True, t0=0.0)
    check(r["ok"] is False, "**轴不在静息态 → ok 必须翻 false**（不信单点结论）", str(r)[:200])
    check(any("静息" in w for w in r.get("warnings", [])),
          "warnings 里点明'不在静息态'")
    check(any("仍在动作" in w for w in r.get("warnings", [])),
          "warnings 提醒不要立即下发指令")

    # ---------------------------------------------------------- 14 汇总
    total = len(_results)
    bad = sum(1 for ok, _ in _results if not ok)
    print(f"\n{'=' * 56}")
    print(f"自检结果: {total - bad}/{total} 通过")
    print(f"{'=' * 56}")
    if bad:
        print("\n失败项:")
        for ok, label in _results:
            if not ok:
                print(f"  - {label}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
