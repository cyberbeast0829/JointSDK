"""阶段 2：SDK 侧限速状态轮询（``Context.set_state_poll``）。

对应 C 侧 ``tests/test_ops.c`` 的 ``[19]``。这里只验 Python 侧能正常配/读，
真正的调度语义（每总线最多一个在途、轮转、超时、记账）由 C 用例负责 ——
Python 层再抄一份只会得到两份会不同步的断言。
"""

from __future__ import annotations

import ctypes

import jsdk_can
from jsdk_can import _abi


def _ctx(**kw):
    hal = jsdk_can.VirtualHal()
    ctx = jsdk_can.Context(hal, is_fd=False, **kw)
    return ctx


def test_default_is_off(lib):
    """默认关闭：不调 ``set_state_poll`` 时一帧都不发，且计数器恒为 0。"""
    ctx = _ctx()
    try:
        for _ in range(20):
            ctx.cycle_begin()
            ctx.cycle_end()
        bs = ctx.bus_state()
        assert bs.state_sent == 0
        assert bs.state_ok == 0
        assert bs.state_timeout == 0
    finally:
        ctx.close()


def test_set_and_read_back(lib):
    """配置后能生效，并且 ``stale_ms()`` 把轮询周期算进去。"""
    ctx = _ctx()
    try:
        j = ctx.add_joint(1)
        ctx.configure()

        base = j.stale_ms()
        assert base >= 50          # 硬下限

        ctx.set_state_poll(100)
        # 100 ms × 4 = 400 > 50 ⇒ 阈值必须跟着涨
        assert j.stale_ms() == 400

        ctx.set_state_poll(0)
        assert j.stale_ms() == base
    finally:
        ctx.close()


def test_bad_field_mask_rejected(lib):
    """未知字段位必须被拒（不是静默忽略）。"""
    ctx = _ctx()
    try:
        ctx.add_joint(1)
        ctx.configure()
        try:
            ctx.set_state_poll(100, fields=0x80)
            raised = False
        except jsdk_can.JsdkError:
            raised = True
        assert raised, "未知字段位应当报错"
    finally:
        ctx.close()


def test_abi_size_updated(lib):
    """ABI 探针：结构体尺寸变了就说明有人在 C 侧加了字段而没同步这里。

    数字取自 C 侧实测（``sizeof(jsdk_context_config_t)`` / ``sizeof(jsdk_bus_state_t)``），
    不靠手算 padding —— 本项目**已经**因为“手算 padding”错过一次。
    """
    assert ctypes.sizeof(_abi.ContextConfig) == 144, ctypes.sizeof(_abi.ContextConfig)
    assert ctypes.sizeof(_abi.BusState) == 72, ctypes.sizeof(_abi.BusState)
