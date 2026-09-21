"""供 pytest 使用的公共夹具。

**关键约定**：测试默认跑在**虚拟后端**上（``--if virtual``），因此
``pytest`` 不需要任何 CAN 硬件 —— 这也是 CI 里唯一能跑的方式。真机用例一律用
``@pytest.mark.hardware`` 标记并默认跳过。

共享库位置：优先环境变量 ``JSDK_LIB_PATH``；否则 ``jsdk_can._abi`` 会依次找
包内 ``lib/`` 与仓库的 ``build/`` 目录。
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

# 让 pytest 直接跑源码树（无需 pip install -e）
_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

import jsdk_can  # noqa: E402  （必须在 sys.path 调整之后）


def pytest_configure(config):  # pragma: no cover - pytest 钩子
    config.addinivalue_line("markers", "hardware: 需要真实 CAN 硬件")


@pytest.fixture(scope="session")
def lib():
    """共享库句柄（会话级）。加载失败就直接把原因暴露出来。"""
    return jsdk_can.load_library()


#: 仿真设备支持的节点数上限（C 侧 SIM_MAX_NODES）
SIM_MAX_NODES = 4


def virtual_spec(nodes: int, *, hb_ms: int = 10) -> str:
    """构造 n 个节点的虚拟总线规格（FD）。``nodes`` 最大 4。"""
    assert 1 <= nodes <= SIM_MAX_NODES, "仿真设备最多 4 个节点"
    return ";".join(
        f"{i}:id={i + 1},gear=16.5,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
        f"kpmax=500,kdmax=5,hb={hb_ms},timeout=30000,fd"
        for i in range(nodes))


@pytest.fixture
def ctx():
    """一个配好的 Context（单节点、虚拟后端），退出时自动安全停车并释放。"""
    c = jsdk_can.Context(jsdk_can.VirtualHal())
    try:
        yield c
    finally:
        c.close()


@pytest.fixture
def ctx_joint(ctx):
    """配好且**已 configure** 的 Context + 关节（最常见的前置条件）。"""
    j = ctx.add_joint(1)
    ctx.configure()
    return ctx, j


@pytest.fixture
def ctx_enabled(ctx_joint):
    """使能好的 Context + 关节。"""
    ctx, j = ctx_joint
    j.enable(jsdk_can.Mode.MIT)
    ctx.activate()
    return ctx, j


@pytest.fixture
def ctx_joint_classic():
    """**Classic 链路**上配好的 Context + 关节（单次响应最多 4 字节）。

    用于验证需要跨块的东西（例如 8 字节参数的读），因为 FD 与 Classic
    在“一次能拿多少字节”上不同（FD ≤ 8 / Classic ≤ 4）。
    """
    c = jsdk_can.Context(jsdk_can.VirtualHal(), is_fd=False)
    try:
        j = c.add_joint(1)
        c.configure()
        yield c, j
    finally:
        c.close()


def spin(ctx, joint, *, n: int = 20, **mit) -> None:
    """跑若干周期并下发 MIT（测试里高频使用，避免到处写循环）。"""
    for _ in range(n):
        ctx.cycle_begin()
        if mit:
            joint.set_mit(**mit)
        ctx.cycle_end()


# ==========================================================================
# 子进程跑 `python -m jsdk_can` 的共享助手（原本在 test_errors_and_cli.py 里）
# ==========================================================================

REPO_PY = Path(__file__).resolve().parents[1]
SRC = REPO_PY / "src"


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
