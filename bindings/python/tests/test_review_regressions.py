"""WP8 self-review 的回归测试：pacer 指标 + 必需符号检查。

这两条都是 review 里发现"看着能工作、其实在骗人"的地方：

* ``pace()`` 的"最大落后"在主循环跟不上周期时恒为 ~0 —— 把最该报警的情况
  抹成了最健康的读数；
* 老库缺少 ``jsdk_abi_types`` 时 ``_initialise`` 静默跳过，用户在
  ``check_abi`` 里收到 ``AttributeError``，"版本不匹配"被误报成"绑定有 bug"。
"""

from __future__ import annotations

import ctypes
import time

import pytest

import jsdk_can
from jsdk_can import _abi
from jsdk_can import AbiMismatchError, JsdkError


# --- pace() ---------------------------------------------------------------


def test_pace_reports_real_overrun(ctx):
    """周期被故意设成比工作耗时还短：pace_stats 必须如实报出落后量。"""
    ctx.cfg.period_ns = 200_000            # 0.2 ms —— 比一个周期的工作还短
    ctx.reset_pace()
    for _ in range(5):
        t0 = time.perf_counter_ns()
        while time.perf_counter_ns() - t0 < 1_000_000:   # 故意干 1 ms 的活
            pass
        ctx.pace()

    cycles, worst_ms = ctx.pace_stats()
    assert cycles == 5
    # 每个周期超时 ~0.8 ms；旧实现会报成 ~0.0x ms
    assert worst_ms > 0.4, f"worst_late={worst_ms} —— pacer 指标把超时掩盖了"


def test_pace_no_overrun_when_period_is_ample(ctx):
    ctx.cfg.period_ns = 20_000_000           # 20 ms，远超工作耗时
    ctx.reset_pace()
    for _ in range(3):
        ctx.pace()
    cycles, worst_ms = ctx.pace_stats()
    assert cycles == 3
    assert worst_ms < 20.0, f"20 ms 周期却报出 {worst_ms} ms 落后？"


def test_pace_is_noop_without_period(ctx):
    ctx.cfg.period_ns = 0
    ctx.reset_pace()
    t0 = time.perf_counter()
    ctx.pace()
    assert time.perf_counter() - t0 < 0.05     # 不该睡
    assert ctx.pace_stats()[0] == 0            # 也不算一个周期


# --- 必需符号检查 ---------------------------------------------------------


def _foreign_library() -> ctypes.CDLL:
    """一个**不是** jsdk 的库（系统 C 库），用来模拟"老版本/拿错文件"。"""
    import sys as _sys
    if _sys.platform.startswith("win"):
        import os
        return ctypes.CDLL(os.path.join(os.environ.get("WINDIR", r"C:\Windows"),
                                        "System32", "kernel32.dll"))
    return ctypes.CDLL("libc.so.6")


def test_missing_required_symbol_is_reported_clearly():
    """用一个不是 jsdk 的库模拟"老版本 DLL / 拿错文件"。"""
    lib = _foreign_library()
    with pytest.raises(AbiMismatchError) as ei:
        _abi._initialise(lib)
    msg = str(ei.value)
    assert "jsdk_abi_types" in msg or "jsdk_context_init" in msg
    assert "旧" in msg                # 提示语要说清是版本问题


def test_real_library_passes_required_check(ctx):
    """真实库必须一个不缺（顺带确认 _REQUIRED_FUNCS 里没有写错的函数名）。"""
    lib = jsdk_can.load_library()
    missing = [n for n in _abi._REQUIRED_FUNCS if not hasattr(lib, n)]
    assert missing == [], f"正式库缺符号：{missing}"


def test_load_errors_are_jsdk_errors():
    """库缺失/ABI 不匹配必须也在 JsdkError 家族里 —— 否则用户写
    ``except JsdkError`` 兜底时会被最外层的部署错误穿过去。"""
    assert issubclass(AbiMismatchError, JsdkError)
    assert issubclass(jsdk_can.LibraryNotFoundError, JsdkError)
    assert issubclass(jsdk_can.LibraryNotFoundError, OSError)   # 仍是文件类错误


def test_abi_check_verifies_backend(ctx):
    table = _abi.check_abi(ctx._lib)
    # 表的键是 C 侧 typedef 名（jsdk_*_t），不是 Python 类名
    assert table["jsdk_can_frame_t"][0] == ctypes.sizeof(_abi.CanFrame)
    assert table["jsdk_joint_feedback_t"][0] == ctypes.sizeof(
        _abi.JointFeedback)


def test_abi_table_has_no_unknown_entries(ctx):
    """C 侧新增结构体而绑定没跟上时，check_abi 必须报出来（这里是反向确认：
    当前版本两边完全对齐）。"""
    table = _abi.check_abi(ctx._lib)
    unknown = [n for n in table if n not in _abi.TYPE_MAP]
    assert unknown == []


# ==========================================================================
# 包内库的发现（B9）
# ==========================================================================

def test_bundled_library_is_discoverable():
    """包内 ``lib/`` 里只要有库，加载器的候选路径里就必须有一条**真实存在**。

    这条用例的存在理由是一个真实缺陷（B9）：Linux 上 CMake 把共享库拷进包内时
    用的是 ``$<TARGET_FILE:>``，于是文件名是带完整版本号的 ``libjsdk_can.so.0.1.0``，
    而 ``_LIB_CANDIDATES["linux"]`` 只有 ``libjsdk_can.so`` / ``.so.0`` / ``jsdk_can.so``
    → **包自带库却找不到**。Windows 因为 ``libjsdk_can.dll`` 不带版本号，一直没暴露，
    于是"看起来只支持 Windows"。

    修法有两道：CMake 拷成无版本号的名字（首选），加载器再对包内 ``lib/`` 做一次
    glob 兜底（防下次换个命名又炸）。这条用例同时覆盖两者：
    **只要包内有库却找不到，它就红**。
    """
    import pathlib

    lib_dir = pathlib.Path(jsdk_can.__file__).resolve().parent / "lib"
    bundled = sorted(p for p in lib_dir.iterdir()) if lib_dir.is_dir() else []
    if not bundled:
        pytest.skip("包内没有捆绑库（纯源码树，正常）")

    resolvable = [p for p in _abi.library_search_path()
                  if p and pathlib.Path(p).is_file()]
    assert resolvable, (
        "包内 lib/ 有 %s，但加载器的候选路径里没有任何**存在**的文件：\n  %s"
        % ([p.name for p in bundled], "\n  ".join(_abi.library_search_path()))
    )


def test_library_search_path_is_deduplicated():
    """候选路径不该重复（glob 兜底 + 显式候选会撞车；重复只是白白多试几次）。"""
    paths = _abi.library_search_path()
    assert len(paths) == len(set(paths))
