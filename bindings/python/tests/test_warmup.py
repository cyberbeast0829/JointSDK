"""会话预热（幂等重发）：把"首帧丢失"挡在第一条真命令之前。

真机（slcan）症状：适配器打开端口时重置输入缓冲，主站**头一两帧上不了总线**，
而 Lawicel 对帧行**不回报结果**（``acks/nacks`` 恒 0）⇒ 主机侧没有任何可观测
信号，于是第一条命令莫名超时（``0/0 bytes`` + 心跳正常），再敲一次又好了。

仿真器侧用 ``drophead=N``（丢掉主站最初的 N 帧）复刻这个现场行为 ——
这是唯一能在 CI 里端到端验证"预热真的自救"的手段。
"""

from __future__ import annotations

import pytest

import jsdk_can
from jsdk_can import Context, VirtualHal

from conftest import virtual_spec


def test_warmup_succeeds_and_is_idempotent():
    """健康链路：一次就通（零重发）；成功后**再调是空操作**（不再发帧）。"""
    with Context(VirtualHal(virtual_spec(1))) as ctx:
        ctx.add_joint(1)

        assert ctx.warmup() is True
        bs = ctx.bus_state()
        assert bs.tx_retries == 0

        tx = bs.tx_frames
        assert ctx.warmup() is True
        assert ctx.bus_state().tx_frames == tx
        assert ctx.bus_state().tx_retries == 0


def test_warmup_absorbs_dropped_first_frames():
    """丢 1..5 帧都必须自愈，且 ``tx_retries`` 正好等于丢掉的帧数。"""
    for n in (1, 2, 3, 5):
        with Context(VirtualHal(virtual_spec(1) + f",drophead={n}")) as ctx:
            ctx.add_joint(1)
            assert ctx.warmup() is True
            assert ctx.bus_state().tx_retries == n, f"drophead={n}"


def test_warmup_absorbs_drops_before_the_first_real_command():
    """懒预热：库用户不显式调用也受保护 —— 第一条真命令前自动预热。

    ``device_info()`` 只发一条 0x46 就等回包（**没有**描述符下载那种自带重发的
    兜底），所以它是"首帧丢失"最锋利的一条探针：没有预热时这里必超时。
    """
    with Context(VirtualHal(virtual_spec(1) + ",drophead=2")) as ctx:
        ctx.add_joint(1)
        info = ctx.device_info()
        assert info.hw_version != 0
        assert ctx.bus_state().tx_retries == 2


def test_warmup_timeout_is_not_fatal_but_is_reported():
    """全丢：``warmup()`` 返回 False（**不抛异常**），错误串说清是预热失败。"""
    with Context(VirtualHal(virtual_spec(1) + ",drophead=100000")) as ctx:
        ctx.add_joint(1)
        assert ctx.warmup(timeout_ms=200) is False
        assert "warm-up" in ctx.last_error()
        # ⚠ 不是致命错误：只收不发的命令照样能跑（这里只断言状态没被破坏）
        assert ctx.bus_state().tx_retries >= 1


def test_warmup_needs_a_joint():
    """没有关节就没法确定 node_id → ``BAD_STATE``（而不是乱发一帧）。"""
    with Context(VirtualHal()) as ctx:
        with pytest.raises(jsdk_can.JsdkStateError):
            ctx.warmup()
