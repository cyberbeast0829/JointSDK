"""ABI 与库加载的自检 —— 这一层必须最先跑通，否则后面全是踩内存。

对应设计文档 §8「ABI 守卫」一行。
"""

from __future__ import annotations

import ctypes

import pytest

import jsdk_can
from jsdk_can import _abi


def test_backend_name(lib):
    assert lib.jsdk_backend_name() == b"cyberbeast-can"


def test_abi_version(lib):
    assert lib.jsdk_abi_version() == jsdk_can.ABI_VERSION


def test_abi_types_all_match(lib):
    """C 侧导出的每个类型都必须在 ctypes 里定义且尺寸/对齐一致。"""
    table = jsdk_can.check_abi(lib)          # 不一致会抛 AbiMismatchError
    assert "jsdk_context_config_t" in table
    assert "jsdk_joint_feedback_t" in table
    assert len(table) >= 15


def test_abi_table_sizes_are_sane():
    """抽查几个尺寸：如果哪天有人改了字段顺序而没同步表，这里会先响。"""
    # 144 = 136（v0.27 的 state_timeout_ms）+ 状态轮询四个字段（v0.37）
    assert ctypes.sizeof(_abi.ContextConfig) == 144
    assert ctypes.sizeof(_abi.JointFeedback) == 112
    assert ctypes.sizeof(_abi.CanHal) == 48
    assert ctypes.sizeof(_abi.CanFrame) == 72


def test_alignment_probe_matches_ctypes():
    """``_alignment_of`` 用的是 offsetof 技巧，别让它算错。"""
    assert _abi._alignment_of(_abi.ContextConfig) == 8
    assert _abi._alignment_of(_abi.CanFrame) == 4


def test_alloc_aligned_really_aligned():
    buf, ptr = jsdk_can.alloc_aligned(100)
    addr = ctypes.cast(ptr, ctypes.c_void_p).value
    assert addr % 8 == 0
    assert ctypes.sizeof(buf) >= 100


def test_alloc_aligned_rejects_zero():
    with pytest.raises(ValueError):
        jsdk_can.alloc_aligned(0)


def test_context_size_matches_exported_bounds(lib):
    """上下文实际尺寸必须落在公开上限内（否则客户按文档分配会溢出）。"""
    assert 0 < lib.jsdk_context_size(None) <= jsdk_can._abi.JSDK_CONTEXT_MAX_SIZE


def test_search_path_mentions_env(monkeypatch):
    monkeypatch.setenv("JSDK_LIB_PATH", "/some/dir")
    paths = jsdk_can.library_search_path()
    assert any("/some/dir" in p for p in paths)


def test_load_missing_library_raises():
    with pytest.raises(jsdk_can.LibraryNotFoundError):
        jsdk_can.load_library("/definitely/not/here/libjsdk_can.so")


def test_enums_match_c(lib):
    """枚举数值是 Python 侧手写的 —— 必须与 C 侧一致。"""
    for mode in jsdk_can.Mode:
        assert lib.jsdk_mode_string(int(mode)), f"mode {mode} 在 C 侧无名字"
    for st in jsdk_can.Status:
        assert lib.jsdk_status_string(int(st))
    for ep in jsdk_can.EpType:
        assert lib.jsdk_ep_type_string(int(ep))
