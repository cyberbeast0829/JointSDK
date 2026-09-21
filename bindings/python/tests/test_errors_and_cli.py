"""异常映射与命令行入口。

覆盖设计文档 §8「Python 绑定」的"异常映射"与"``python -m jsdk_can`` 只读子集冒烟"。
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

import jsdk_can
from jsdk_can import Status, VirtualHal
from jsdk_can.errors import _MAP, error_for_status, raise_for_status

from conftest import SRC, lib_dir, run_module, virtual_spec  # noqa: F401



# ==========================================================================
# 状态码 ⇔ 异常类：映射必须自洽
# ==========================================================================


@pytest.mark.parametrize("status", list(Status))
def test_status_to_exception_mapping_is_self_consistent(status):
    """``error_for_status(st)`` 造出来的异常，其 ``status`` 必须还是 ``st``。

    这条防的是"手工抛异常时写错类"：``raise JsdkUsageError(-4, ...)`` 看着没问题，
    但 -4 是 BAD_STATE，调用方按文档 ``except JsdkStateError`` 捕不到。
    """
    if status is Status.OK:
        return
    exc = error_for_status(int(status), "op", "detail")
    assert exc.status == status
    assert isinstance(exc, jsdk_can.JsdkError)
    assert exc.op == "op"
    assert "detail" in str(exc)
    # 映射表里登记的类必须真的被用上
    if status in _MAP:
        assert type(exc) is _MAP[status]


def test_raise_for_status_ok_is_noop():
    assert raise_for_status(0) is None


@pytest.mark.parametrize("status,expected", [
    (Status.INVALID_ARG, jsdk_can.JsdkUsageError),
    (Status.NOT_FOUND, jsdk_can.JsdkDeviceNotFoundError),
    (Status.BAD_STATE, jsdk_can.JsdkStateError),
    (Status.TRANSPORT, jsdk_can.JsdkTransportError),
    (Status.TIMEOUT, jsdk_can.JsdkTimeoutError),
    (Status.PROTOCOL, jsdk_can.JsdkProtocolError),
    (Status.UNSUPPORTED, jsdk_can.JsdkUnsupportedError),
    (Status.NO_MEMORY, jsdk_can.JsdkMemoryError),
    (Status.PARSE, jsdk_can.JsdkParseError),
    (Status.BUSY, jsdk_can.JsdkBusyError),
])
def test_specific_exception_classes(status, expected):
    assert type(error_for_status(int(status))) is expected
    assert issubclass(expected, jsdk_can.JsdkError)


def test_unknown_status_falls_back_to_base():
    exc = error_for_status(-999)
    assert type(exc) is jsdk_can.JsdkError
    assert "unknown" in exc.name()


def test_error_message_contains_detail():
    exc = error_for_status(int(Status.TIMEOUT), "configure", "node 1 无应答")
    assert "configure" in str(exc)
    assert "node 1 无应答" in str(exc)
    assert "timeout" in str(exc).lower()


# ==========================================================================
# 真实错误的映射（走虚拟后端构造）
# ==========================================================================


def test_lookup_failure_is_not_found(ctx_joint):
    ctx, _j = ctx_joint
    with pytest.raises(jsdk_can.JsdkDeviceNotFoundError) as ei:
        ctx.lookup("nope")
    assert ei.value.op.startswith("lookup")
    assert ei.value.status == Status.NOT_FOUND


def test_joint_not_in_context(ctx_joint):
    ctx, _j = ctx_joint
    with pytest.raises(jsdk_can.JsdkDeviceNotFoundError):
        ctx.joint(7)


def test_add_joint_after_configure(ctx_joint):
    ctx, _j = ctx_joint
    with pytest.raises(jsdk_can.JsdkStateError):
        ctx.add_joint(2)


def test_unsupported_hal_backend_is_not_crash():
    """没编译的后端 → JsdkUnsupportedError（或本机没有该硬件对应的 NotFound），
    但必须是 JsdkError 且句柄不泄漏。"""
    try:
        ctx = jsdk_can.SocketCanHal("can0")
        ctx.attach(jsdk_can.load_library())
    except jsdk_can.JsdkError as exc:
        assert exc.status in (Status.UNSUPPORTED, Status.NOT_FOUND,
                              Status.TRANSPORT, Status.INVALID_ARG)
    else:                                   # pragma: no cover - 仅在 Linux 真机上
        ctx.close()


def test_last_error_is_attached_to_exception(ctx_joint):
    """异常里必须带上 C 侧的可读文本 —— 那句话是写给现场工程师的。"""
    ctx, j = ctx_joint
    with pytest.raises(jsdk_can.JsdkError) as ei:
        j.param_set("axis0.no.such.path", 1)
    assert ei.value.detail, "detail 不该为空"


# ==========================================================================
# python -m jsdk_can（只读子集）
# ==========================================================================


def test_module_help(lib_dir):
    """帮助必须把**安全闸**写在明面上（本入口现在与 jsdk-cli 功能对齐）。

    早期这里断言的是帮助里的"只读"两个字 —— 那时这一入口只做只读子集。
    现在写/动作命令都在（这是有意的扩展），所以改钉**真正的契约**：
    ① 帮助里出现 `--yes`（写类命令的闸）；② 列出会动电机的 `mit`。
    """
    r = run_module("--help", lib_dir=lib_dir)
    assert r.returncode == 0
    assert "--yes" in r.stdout
    assert "mit" in r.stdout
    assert "estop" in r.stdout


def test_module_scan_json(lib_dir):
    r = run_module("scan", "--json", lib_dir=lib_dir)
    assert r.returncode == 0, r.stderr
    payload = json.loads(r.stdout)
    assert payload["nodes"] == [1]
    assert payload["count"] == 1


def test_module_health(lib_dir):
    r = run_module("health", lib_dir=lib_dir)
    assert r.returncode == 0, r.stderr
    assert "节点 1" in r.stdout or "joint" in r.stdout


def test_module_dump_config_json(lib_dir):
    r = run_module("dump-config", "--json", lib_dir=lib_dir)
    assert r.returncode == 0, r.stderr
    payload = json.loads(r.stdout)
    assert payload["valid"] is True
    assert payload["gear_ratio"] == pytest.approx(16.5)
    assert payload["mit_max_torque"] == pytest.approx(50.0)


def test_module_read(lib_dir):
    r = run_module("read", "axis0.motor.config.gear_ratio", "--json",
                   lib_dir=lib_dir)
    assert r.returncode == 0, r.stderr
    payload = json.loads(r.stdout)
    assert payload["type"] == "float"
    assert payload["value"] == pytest.approx(16.5)


def test_module_desc_info_and_ep_list(lib_dir):
    r = run_module("desc-info", "--json", lib_dir=lib_dir)
    assert r.returncode == 0, r.stderr
    assert json.loads(r.stdout)["endpoint_count"] > 0

    r = run_module("ep-list", "--json", "--filter", "mit_max_", lib_dir=lib_dir)
    assert r.returncode == 0, r.stderr
    payload = json.loads(r.stdout)
    assert payload["count"] == 5
    assert any(e["path"].endswith("mit_max_torque") for e in payload["endpoints"])


def test_module_ep_lookup(lib_dir):
    r = run_module("ep-lookup", "axis0.config.can.node_id", "--json",
                   lib_dir=lib_dir)
    assert r.returncode == 0, r.stderr
    payload = json.loads(r.stdout)
    assert payload["id"] == 180
    assert payload["type"] == "uint32"


def test_module_mon_bounded(lib_dir):
    r = run_module("mon", "--duration", "1", "--rate-hz", "50", lib_dir=lib_dir)
    assert r.returncode == 0, r.stderr
    lines = [ln for ln in r.stdout.splitlines() if ln.startswith("{")]
    assert len(lines) >= 2, f"mon 应当输出多行 NDJSON（实际 {len(lines)} 行）"
    assert "pos_rad" in lines[0]


def test_module_gates_write_subcommands(lib_dir):
    """写类子命令**必须存在**，但不加 ``--yes`` 时必须被安全闸拒绝（退出码 3）。

    ⚠ 本条与它的前身（``test_module_rejects_write_subcommands``）是**相反的契约**：
    那时这一入口只做只读子集，写类命令必须“不存在”。现在与 `jsdk-cli` 功能对齐，
    它们存在 —— 所以这条用例改钉“存在、但默认拒绝”。
    """
    for cmd in (("write", "axis0.config.can.node_id", "1"),
                ("mit", "--pos", "0"),
                ("save",), ("set-zero",), ("calibrate",), ("home",),
                ("watchdog", "100"), ("reset",)):
        r = run_module("--if", "virtual", *cmd, lib_dir=lib_dir)
        assert r.returncode == 3, \
            f"{' '.join(cmd)} 不加 --yes 必须被拒（rc={r.returncode}）"
        assert "--yes" in r.stderr

    # estop 是例外：拒绝执行比误停更危险
    r = run_module("--if", "virtual", "estop", lib_dir=lib_dir)
    assert r.returncode == 0, r.stderr


def test_module_bad_backend(lib_dir):
    r = run_module("--if", "nosuchbus", "scan", lib_dir=lib_dir)
    assert r.returncode != 0


def test_redirected_output_is_utf8(lib_dir):
    """
    重定向时必须是 UTF-8 —— 不能靠用户设 `PYTHONIOENCODING`。

    Windows 下 Python 重定向时默认用 **locale 编码**（中文机器是 cp936），于是：
      - 与 C 版 `jsdk-cli`（重定向时写 UTF-8）**不一致**，同一条流水线两种编码；
      - 输出里出现 CP936 表示不了的字符（`⚠` U+26A0 之类）会直接
        `UnicodeEncodeError` 崩掉。

    ⚠ 这里**故意**不带 `PYTHONIOENCODING`：带上它这个用例就永远是绿的，
      等于把这个坑重新盖回去（原来的 `run_module()` 就一直带着它）。
    """
    import os

    env = dict(os.environ)
    env.pop("PYTHONIOENCODING", None)
    env["PYTHONPATH"] = str(SRC)
    env["JSDK_LIB_PATH"] = lib_dir
    r = subprocess.run([sys.executable, "-m", "jsdk_can", "--help"],
                       capture_output=True, env=env, timeout=120)
    assert r.returncode == 0, r.stderr
    text = r.stdout.decode("utf-8")          # 不是 UTF-8 → UnicodeDecodeError
    assert any("\u4e00" <= ch <= "\u9fff" for ch in text), "帮助文本里应当有中文"
