"""8 字节参数的读（WP9 修复）+ 分块读的请求形状。

背景：固件按**设备侧归一化**的 `ReqLen` 切片 —— FD 一次 ≤ 8 B、Classic 一次 ≤ 4 B，
超出部分靠 `offset` 续读（`docs/PROTOCOL_NOTES.zh-CN.md` §5.2 早就写明了）。
SDK 侧原先只实现了"写死 4 字节请求"，于是 `serial_number` / `axis0.motor.error`
这类 u64 端点**永远读不出来**，报的还是"设备只回了 4 字节，而描述符说 8 字节"
—— 把矛头指向固件/描述符，真正的原因在主站自己的请求里。

本模块同时钉住**请求帧形状**（第一块旧式 4 B、后续块带 offset），因为仿真设备
两种形式都接受：只看"读回来的值对不对"是发现不了写错的。
"""

from __future__ import annotations

import pytest

import jsdk_can

#: 虚拟设备里 serial_number 是固定常量
SERIAL = 0x1122334455667788
SERIAL_BYTES = SERIAL.to_bytes(8, "big")

#: MsgType 在 CAN ID 里的位置（见 PROTOCOL_NOTES §1）
_MSGTYPE_SHIFT = 18
_MSG_PARAM_READ = 0x20


def _take_param_requests(ctx) -> list[bytes]:
    """取出捕获队列里的 `PARAM_READ(0x20)` **请求**帧载荷（会清空队列）。

    绑定层是唯一能看到"实际发出去的字节"的地方 —— 用它验证分块读的**帧形状**。
    """
    payloads: list[bytes] = []
    while True:
        f = ctx.hal.capture()
        if f is None:
            break
        if ((int(f.id) >> _MSGTYPE_SHIFT) & 0xFF) != _MSG_PARAM_READ:
            continue
        payloads.append(bytes(f.data[:f.len]))
    return payloads


# ==========================================================================
# 值正确性
# ==========================================================================


def test_wide_param_read_fd(ctx_joint):
    """FD：一次请求读满 8 字节，请求帧用旧式 4 B 形式但 ReqLen=8。"""
    ctx, j = ctx_joint
    _take_param_requests(ctx)                     # 清空 configure 期残留

    tx0 = ctx.bus_state().tx_frames
    assert j.param_get("serial_number") == SERIAL
    assert ctx.bus_state().tx_frames - tx0 == 1, "FD 下一次请求就该拿满"

    rq = _take_param_requests(ctx)
    assert len(rq) == 1
    assert len(rq[0]) == 4, "第一块用旧式 4 B 形式（兼容性最好）"
    assert rq[0][3] == 8, "ReqLen 必须是 8，否则设备只回 4 B"


def test_wide_param_read_classic_is_chunked(ctx_joint_classic):
    """Classic：设备一次只回 4 字节 → 必须分两块，值要和 FD 一致。"""
    ctx, j = ctx_joint_classic
    _take_param_requests(ctx)

    tx0 = ctx.bus_state().tx_frames
    assert j.param_get("serial_number") == SERIAL
    assert ctx.bus_state().tx_frames - tx0 == 2, "8 字节值在 Classic 下要两块"

    rq = _take_param_requests(ctx)
    assert len(rq) == 2
    assert len(rq[0]) == 4 and rq[0][3] == 4
    assert len(rq[1]) == 8, "后续块必须带 offset 字段"
    assert rq[1][3] == 4
    assert int.from_bytes(rq[1][4:8], "big") == 4, "第二块的 offset 必须是 4"


def test_motor_error_64bit_is_readable(ctx_joint):
    """`axis0.motor.error` 是 u64 —— 排障最常用的字段之一，必须读得到。"""
    ctx, j = ctx_joint
    value = j.param_get("axis0.motor.error")
    assert value == 0
    assert isinstance(value, int)


def test_narrow_param_read_is_still_one_request(ctx_joint_classic):
    """4 字节/1 字节的值不能被"统一分块"拖成两次请求。"""
    ctx, j = ctx_joint_classic
    for path, expect in (("axis0.motor.config.gear_ratio", 16.5),
                         ("axis0.config.can.node_id", 1)):
        tx0 = ctx.bus_state().tx_frames
        assert j.param_get(path) == pytest.approx(expect)
        assert ctx.bus_state().tx_frames - tx0 == 1, path


def test_wide_param_read_matches_batch(ctx_joint):
    """单读与批量读必须给出同一个值。

    修复前：批量（自己按类型长度装箱）对、单读（写死 4 字节）错 ——
    这个用例把两条路径绑在一起，以后哪条退化了都会立刻现形。
    """
    ctx, j = ctx_joint
    solo = j.param_get("serial_number")
    batch = j.param_get_batch("serial_number")["serial_number"]
    assert solo == batch == SERIAL


def test_classic_wide_value_is_not_truncated(ctx_joint_classic):
    """⚠ 最关键的一条：分块读**不能**把前 4 字节复制两遍当成功。

    只检查"没报错"是不够的：offset 不推进时旧实现会得到
    `0x11223344_11223344` 这种"看着像成功"的错值。
    """
    ctx, j = ctx_joint_classic
    raw = j.param_get("serial_number")
    assert raw != int.from_bytes(SERIAL_BYTES[:4] * 2, "big")
    assert raw == SERIAL


# ==========================================================================
# 类型与错误路径
# ==========================================================================


def test_wide_param_read_is_type_checked(ctx_joint):
    """读到 8 字节值 ≠ 能按 4 字节类型读：类型不符必须被拒。"""
    ctx, j = ctx_joint
    with pytest.raises(jsdk_can.JsdkProtocolError):
        j.param_get_u32("serial_number")


def test_unreadable_type_is_rejected(ctx_joint):
    """`object`/`json` 这类类型没有标量尺寸 → 明确拒绝，而不是读一半。

    ⚠ 会跳过：**测试夹具 `endpoints_v8.json` 是精简过的、只有标量端点**
    （真实描述符里有 object/json/endpoint_ref）。保留这条是为了将来把夹具换成
    完整描述符时能立刻覆盖到这个分支；C 侧同样有这个空缺。
    """
    ctx, j = ctx_joint
    eps = [e for e in ctx.endpoints() if e.type_name in ("object", "json",
                                                        "endpoint_ref")]
    if not eps:                                   # pragma: no cover
        pytest.skip("测试夹具只有标量端点（见 docstring）")
    with pytest.raises(jsdk_can.JsdkUnsupportedError):
        j.param_get(eps[0].path)


def test_unknown_endpoint_is_not_found(ctx_joint):
    ctx, j = ctx_joint
    with pytest.raises(jsdk_can.JsdkDeviceNotFoundError):
        j.param_get("axis0.motor.config.no_such_thing")
