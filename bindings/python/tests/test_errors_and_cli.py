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

from conftest import virtual_spec

REPO_PY = Path(__file__).resolve().parents[1]
SRC = REPO_PY / "src"


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


def run_module(*args: str, lib_dir: str) -> subprocess.CompletedProcess:
    env = {
        "JSDK_LIB_PATH": lib_dir,
        "PYTHONPATH": str(SRC),
        # Windows 下控制台默认 GBK，子进程里统一 UTF-8
        "PYTHONIOENCODING": "utf-8",
    }
    import os
    full_env = dict(os.environ)
    full_env.update(env)
    # ⚠ 必须显式 encoding="utf-8"：`text=True` 会按**本机 locale**（Windows 上是
    #   GBK）解码子进程输出，而子进程按 PYTHONIOENCODING 写的是 UTF-8 ——
    #   中文会变成乱码，断言随之中文匹配失败（看着像功能坏了，其实是解码问题）。
    return subprocess.run([sys.executable, "-m", "jsdk_can", *args],
                          capture_output=True, text=True,
                          encoding="utf-8", errors="replace",
                          env=full_env, timeout=120)


@pytest.fixture(scope="module")
def lib_dir():
    """
    子进程要能找到共享库。优先级：环境变量 → **包内自带的 lib/**。

    ⚠ 原先这里**硬要求** `JSDK_LIB_PATH`，于是"刚 clone 下来跑 `pytest`"会得到
    10 个 error（而不是 fail），而且提示看起来像配置问题 —— 但包内其实**就有**
    一份 `src/jsdk_can/lib/libjsdk_can.{dll,so}`（CMake 的 JSDK_BUILD_PYTHON
    会把库拷进去，wheel 里也随包分发）。既然包自己带着库，测试就不该要求
    调用者先导出环境变量。环境变量仍然优先（用于指向别的构建）。
    """
    import os
    from pathlib import Path

    d = os.environ.get("JSDK_LIB_PATH", "").strip()
    if d:
        return d

    bundled = Path(__file__).resolve().parents[1] / "src" / "jsdk_can" / "lib"
    if bundled.is_dir() and any(bundled.iterdir()):
        return str(bundled)

    pytest.fail(
        "找不到共享库：包内 lib/ 是空的，也没设 JSDK_LIB_PATH。\n"
        "先构建：cmake -S . -B bsh -DJSDK_BUILD_SHARED=ON -DJSDK_BUILD_PYTHON=ON "
        "&& cmake --build bsh\n"
        f"（已查找：{bundled}）"
    )


def test_module_help(lib_dir):
    r = run_module("--help", lib_dir=lib_dir)
    assert r.returncode == 0
    assert "只读" in r.stdout


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


def test_module_rejects_write_subcommands(lib_dir):
    """本入口只做只读：写类子命令必须不存在。"""
    for cmd in ("write", "mit", "save", "set-zero", "calibrate"):
        r = run_module(cmd, lib_dir=lib_dir)
        assert r.returncode != 0, f"{cmd} 不该被接受"


def test_module_bad_backend(lib_dir):
    r = run_module("--if", "nosuchbus", "scan", lib_dir=lib_dir)
    assert r.returncode != 0
