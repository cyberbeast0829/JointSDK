"""非阻塞状态请求（``Joint.request_state()``）：驱动中的新鲜反馈。

真机背景（JointROS 的 F11）：某些固件上**主动上报的帧不更新**
（``pos``/``vel`` 恒 0 + ``FEEDBACK_STALE``），而 ``param_get()`` / SDO 这类读都
**在调用者线程里等应答**（真机单次 1.4〜5.5 ms，而 1 kHz 的 tick 只有 1 ms）
⇒ RT 循环里用不了。``request_state()`` 只把请求帧发出去，应答由
``cycle_begin()`` 的收帧路径回填到 ``feedback()`` —— 于是"读"不再与 tick 互斥。

⚠ 这些用例都关掉设备心跳（``hb_ms=0``）：否则一个周期里可能"碰巧"收到心跳而让
``feedback().valid`` 变真，"应答丢了就不假装新鲜"那条断言会变成**概率性**的。
"""

from __future__ import annotations

import pytest

import jsdk_can
from jsdk_can import Context, VirtualHal
from jsdk_can.joint import STATE_ALL, STATE_CURRENT, STATE_POS_VEL

from conftest import virtual_spec

MT_QUERY_POS_VEL = "0x41"
MT_QUERY_CURRENT = "0x44"


def test_request_state_lands_in_feedback_next_cycle():
    """发一次 0x41 → 下一个 ``cycle_begin()`` 里 ``feedback()`` 就是新鲜的。"""
    with Context(VirtualHal(virtual_spec(1, hb_ms=0))) as ctx:
        j = ctx.add_joint(1)
        ctx.configure()

        tx0 = ctx.bus_state().tx_frames
        j.request_state(STATE_POS_VEL)
        assert ctx.bus_state().tx_frames == tx0 + 1, "只发一帧请求，不等应答"
        assert ctx.bus_state().req_timeouts == 0

        ctx.cycle_begin()
        fb = j.feedback()
        assert fb.valid, "应答应当已被 cycle_begin() 的收帧路径解码回填"
        assert fb.age_ms == 0
        ctx.cycle_end()


def test_request_state_does_not_pretend_when_reply_is_lost():
    """应答丢了：调用仍**立刻**成功（它根本没等）、不记账，且 ``valid`` 保持假。"""
    with Context(VirtualHal(virtual_spec(1, hb_ms=0) + f",dropmsg={MT_QUERY_POS_VEL}:1")) as ctx:
        j = ctx.add_joint(1)
        ctx.configure()

        to0 = ctx.bus_state().req_timeouts
        j.request_state(STATE_POS_VEL)              # 不抛异常 = 没有阻塞等待
        assert ctx.bus_state().req_timeouts == to0, "非阻塞路径不记超时"

        ctx.cycle_begin()                            # 没有新帧：不能卡住
        assert not j.feedback().valid, "丢了就是丢了，不能假装新鲜"
        ctx.cycle_end()


def test_request_state_current_and_combined_mask():
    """``STATE_CURRENT`` 走 0x44；``STATE_ALL`` = 两帧（各一次往返）。"""
    with Context(VirtualHal(virtual_spec(1, hb_ms=0))) as ctx:
        j = ctx.add_joint(1)
        ctx.configure()

        tx0 = ctx.bus_state().tx_frames
        j.request_state(STATE_CURRENT)
        assert ctx.bus_state().tx_frames == tx0 + 1

        tx1 = ctx.bus_state().tx_frames
        j.request_state(STATE_ALL)
        assert ctx.bus_state().tx_frames == tx1 + 2

        ctx.cycle_begin()
        assert j.feedback().valid
        ctx.cycle_end()


def test_request_state_rejects_bad_mask():
    """掩码为 0 或含未知位：抛异常且**一帧都不发**（参数错不该上总线）。"""
    with Context(VirtualHal(virtual_spec(1, hb_ms=0))) as ctx:
        j = ctx.add_joint(1)
        ctx.configure()
        tx0 = ctx.bus_state().tx_frames

        for bad in (0, 0x80, STATE_POS_VEL | 0x40):
            with pytest.raises(jsdk_can.JsdkError):
                j.request_state(bad)

        assert ctx.bus_state().tx_frames == tx0
