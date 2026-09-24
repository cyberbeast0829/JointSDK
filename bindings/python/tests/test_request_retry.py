"""幂等的“请求 → 响应”重发（运行**中途**丢帧）。

会话预热（见 ``test_warmup.py``）只保护“会话开头那几帧”；适配器中途抽一下时，
``err``/``info``/``read`` 这类**单发即等**的命令以前没有任何兜底 —— 丢了就是一条超时
（与首帧丢失同症状）。现在这类命令在超时后会**重发一次同一帧**（幂等 ⇒ 安全），
并且只有“等 ACK 的写”才有资格重发（``axis0.requested_state`` 那类不等 ACK 的写
**绍不**重发，否则等于重复触发标定/回零）。

仿真器侧用 ``dropmsg=<MsgType>:<N>``（丢掉主站某个 MsgType 的前 N 帧）复刻现场行为；
计数从 ``ctx.bus_state().tx_retries_req`` 读（与“会话预热”那种开头丢帧分开统计）。
"""

from __future__ import annotations

import pytest

from jsdk_can import Context, VirtualHal

from conftest import virtual_spec

#: 参数读（0x20，`configure()` 与 `param_get()` 都用它）。
MT_PARAM_READ = 0x20
#: 故障明细查询（0x45，只有 `err` / `query_error_detail()` 用它）。
MT_QUERY_ERROR = 0x45


def test_dropped_param_read_is_retried_once():
    """丢一条参数读（0x20）→ 自动重发一次；`configure()` 里的标定读因此没失败。"""
    with Context(VirtualHal(virtual_spec(1) + f",dropmsg={MT_PARAM_READ}:1")) as ctx:
        j = ctx.add_joint(1)
        ctx.configure()                       # 第一条 0x20 被丢掉 → 必须靠重发救回

        assert ctx.bus_state().tx_retries_req == 1

        # 已经投过的丢帧不会再来：后续读不应再产生重发
        assert j.param_get("axis0.motor.config.gear_ratio") == pytest.approx(16.5)
        assert ctx.bus_state().tx_retries_req == 1


def test_dropped_error_query_is_retried_after_configure():
    """典型的“单发即等”命令：`query_error_detail()` 丢一帧也必须自愈。"""
    with Context(VirtualHal(virtual_spec(1) + f",dropmsg={MT_QUERY_ERROR}:1")) as ctx:
        ctx.add_joint(1)
        ctx.configure()                       # 0x45 在这条路径上不会被用到

        assert ctx.bus_state().tx_retries_req == 0
        ctx.query_error_detail()
        assert ctx.bus_state().tx_retries_req == 1


def test_all_dropped_fails_bounded():
    """全丢：**有界失败**（只多试一次），不是无限重试。"""
    with Context(VirtualHal(virtual_spec(1) + f",dropmsg={MT_PARAM_READ}:100000")) as ctx:
        ctx.add_joint(1)

        with pytest.raises(Exception):
            ctx.configure()
        assert ctx.bus_state().tx_retries_req == 1


# ---------------------------------------------------------------------------
# 观测（v0.33）：计数分类 + 不变式 + 最近一次重发 + 描述符请求重发次数
# ---------------------------------------------------------------------------


def test_retry_counters_and_invariant():
    """`tx_retries == tx_retries_warm + tx_retries_req`，且“运行中途丢帧”单独计量。"""
    with Context(VirtualHal(virtual_spec(1) + f",dropmsg={MT_PARAM_READ}:1")) as ctx:
        ctx.add_joint(1)
        ctx.configure()

        bs = ctx.bus_state()
        assert bs.tx_retries == bs.tx_retries_warm + bs.tx_retries_req
        assert bs.tx_retries_req == 1
        assert bs.req_timeouts >= 1          # 救回来了也要记账（那才是“丢过帧”的证据）
        assert bs.last_retry_what == 2       # 2 = 幂等请求
        assert bs.last_retry_age_ms >= 0


def test_warmup_retries_are_classified_as_warmup():
    """会话开头丢帧归到 `tx_retries_warm`，不算“运行中途丢帧”。"""
    with Context(VirtualHal(virtual_spec(1) + ",drophead=2")) as ctx:
        ctx.add_joint(1)
        ctx.warmup()
        bs = ctx.bus_state()
        assert bs.tx_retries == bs.tx_retries_warm + bs.tx_retries_req
        assert bs.tx_retries_warm >= 2
        assert bs.tx_retries_req == 0
        assert bs.last_retry_what == 1       # 1 = 会话预热


def test_desc_info_reports_request_resends():
    """`desc_info().retries` = 0x24 请求重发次数（“请求丢了”的直接证据）。"""
    with Context(VirtualHal(virtual_spec(1) + ",dropmsg=0x24:1")) as ctx:
        ctx.add_joint(1)
        ctx.configure()
        info = ctx.desc_info()
        assert info.retries >= 1
        assert info.complete


def test_probes_and_polling_do_not_pollute_timeouts():
    """扫描探测与轮询读**不计**超时：15 个空地址的“没人应答”不是链路故障。

    （真机实测：一条正常的 `scan` 曾报 `超时=15` —— 全是 2..16 号空地址。）
    """
    with Context(VirtualHal(virtual_spec(1))) as ctx:
        ctx.add_joint(1)
        ctx.configure()

        before = ctx.bus_state().req_timeouts
        assert ctx.discover(max_probe=16)          # 只 node 1 在
        ctx.joint(1).feedback()                    # 轮询读（内部走 _once）
        assert ctx.bus_state().req_timeouts == before
