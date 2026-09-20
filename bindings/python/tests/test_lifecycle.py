"""上下文生命周期：加载 → 加关节 → configure → 使能 → 循环 → 安全停车。

对应设计文档 §8「Python 绑定」一行："pytest + virtual HAL：上下文生命周期、
5 种模式、反馈新鲜度、异常映射、dump_config() 数值"。
"""

from __future__ import annotations

import pytest

import jsdk_can
from jsdk_can import Context, Mode, Status, VirtualHal

from conftest import SIM_MAX_NODES, spin, virtual_spec


# ==========================================================================
# 构造与参数校验
# ==========================================================================


def test_construct_and_close():
    ctx = Context(VirtualHal())
    assert len(ctx) == 0
    assert ctx.hal.name == "virtual"
    ctx.close()
    ctx.close()          # 幂等


def test_context_manager_closes():
    with Context(VirtualHal()) as ctx:
        ctx.add_joint(1)
        assert len(ctx) == 1
    assert ctx._closed


def test_master_id_zero_rejected():
    """主站 0 → 设备完全不回复。必须在 Python 侧就拦住，别让它上总线。"""
    with pytest.raises(ValueError, match="master_id"):
        Context(VirtualHal(), master_id=0)


def test_node_id_range_checked():
    with Context(VirtualHal()) as ctx:
        with pytest.raises(ValueError, match="node_id"):
            ctx.add_joint(0)
        with pytest.raises(ValueError, match="node_id"):
            ctx.add_joint(255)


def test_add_joint_after_configure_refused(ctx):
    ctx.add_joint(1)
    ctx.configure()
    with pytest.raises(jsdk_can.JsdkStateError):
        ctx.add_joint(2)


def test_bad_virtual_spec():
    from jsdk_can import JsdkUsageError

    with pytest.raises(JsdkUsageError):
        Context(VirtualHal("this is not a spec"))


def test_missing_backend_symbol_is_clear():
    """没编译的后端要给出明确错误，而不是 AttributeError。"""
    with Context(VirtualHal()) as ctx:
        # 本机（Windows）没编译 socketcan → 打开必须失败且是 JsdkError 子类
        with pytest.raises(jsdk_can.JsdkError):
            Context(jsdk_can.SocketCanHal("can0"))


# ==========================================================================
# configure / 描述符 / 端点
# ==========================================================================


def test_configure_and_desc_info(ctx_joint):
    ctx, _j = ctx_joint
    info = ctx.desc_info()
    assert info.complete, "描述符必须完整（complete=0 时不能缓存原始 JSON）"
    assert info.endpoint_count > 0
    assert info.total_len > 0
    assert info.crc != 0


def test_configure_is_idempotent(ctx_joint):
    ctx, _j = ctx_joint
    before = ctx.bus_state().tx_frames
    ctx.configure()                       # 再来一次应当是空操作
    assert ctx.bus_state().tx_frames == before


def test_descriptor_covers_calibration_set(ctx_joint):
    """标定必需的那批端点必须在端点表里 —— 少一个 configure() 就会失败。"""
    ctx, _j = ctx_joint
    paths = {e.path for e in ctx.endpoints()}
    for required in (
        "axis0.motor.config.gear_ratio",
        "axis0.motor.config.torque_constant",
        "axis0.controller.config.mit_max_pos",
        "axis0.controller.config.mit_max_vel",
        "axis0.controller.config.mit_max_torque",
        "axis0.controller.config.mit_max_kp",
        "axis0.controller.config.mit_max_kd",
    ):
        assert required in paths, f"缺少必需端点 {required}"


def test_lookup_unknown_path_is_not_found(ctx_joint):
    ctx, _j = ctx_joint
    with pytest.raises(jsdk_can.JsdkDeviceNotFoundError):
        ctx.lookup("axis0.no.such.thing")


def test_endpoint_access_and_type(ctx_joint):
    ctx, _j = ctx_joint
    ep = ctx.lookup("axis0.motor.config.gear_ratio")
    assert ep.type is jsdk_can.EpType.F32
    assert ep.readable and ep.writable
    assert ep.access_str == "rw"

    ro = ctx.lookup("axis0.current_state")
    assert ro.readable and not ro.writable


def test_endpoints_matching_substring(ctx_joint):
    ctx, _j = ctx_joint
    eps = ctx.endpoints_matching("mit_max_")
    assert len(eps) == 5
    assert all("mit_max_" in e.path for e in eps)


def test_desc_export_import_roundtrip():
    """路线 A 缓存：导出 → 导入。导入**不下载**。"""
    with Context(VirtualHal()) as ctx:
        ctx.add_joint(1)
        ctx.configure()
        blob = ctx.desc_export()
        assert len(blob) > 0
        n_before = len(ctx.endpoints())

    # 新上下文：只导入，不 configure
    with Context(VirtualHal()) as ctx2:
        ctx2.add_joint(1)
        ctx2.desc_import(blob)
        assert len(ctx2.endpoints()) == n_before
        assert ctx2.lookup("axis0.motor.config.gear_ratio").ep_id > 0


def test_desc_import_garbage_rejected(ctx):
    with pytest.raises(jsdk_can.JsdkError):
        ctx.desc_import(b"not a descriptor cache at all")


# ==========================================================================
# 使能 / 循环 / 停车
# ==========================================================================


def test_activate_enables_and_sends_first_frame(ctx_joint):
    ctx, j = ctx_joint
    assert not j.is_enabled()
    j.enable(Mode.MIT)
    ctx.activate()
    assert j.is_enabled()
    assert j.feedback().online, "使能后应已收到设备响应"


def test_enable_then_activate_does_not_deadlock(ctx_joint):
    """``request_enable`` 与 ``activate`` 必须能混用 —— 头文件承诺两者等价。

    这两条路径曾经是**两套实现**，混用会让 ``enable_pending`` 永远清不掉，
    ``activate()`` 只会一路超时（v0.14 修复）。
    """
    ctx, j = ctx_joint
    j.enable(Mode.MIT)          # 走 L3 序列
    ctx.activate()              # 走"等价于 request_enable + 等待"
    assert j.is_enabled()


def test_enable_all_then_disable_all():
    # ⚠ 虚拟总线默认只有 1 个节点；要两个关节就必须显式给 2 节点规格，
    #   否则第二个关节永远没有应答，configure() 会超时。
    with Context(VirtualHal(virtual_spec(2))) as ctx:
        ctx.add_joint(1)
        ctx.add_joint(2)
        ctx.configure()
        ctx.enable_all(Mode.MIT)
        assert all(j.is_enabled() for j in ctx.joints)

        ctx.disable_all()
        assert not any(j.is_enabled() for j in ctx.joints)


def test_cycle_moves_joint(ctx_enabled):
    ctx, j = ctx_enabled
    spin(ctx, j, n=30, pos=0.1, kp=5.0, kd=0.5)
    fb = j.feedback()
    assert fb.valid
    assert fb.tx_frames > 0
    assert fb.pos > 0.0, "kp>0 且给了正目标，位置应朝正方向走"


def test_close_deactivates_when_enabled():
    ctx = Context(VirtualHal())
    j = ctx.add_joint(1)
    ctx.configure()
    j.enable(Mode.MIT)
    ctx.activate()
    assert j.is_enabled()
    ctx.close()                 # 先 deactivate 再释放
    assert ctx._closed


def test_estop_does_not_raise(ctx_enabled):
    ctx, _j = ctx_enabled
    ctx.estop()                 # 广播 ESTOP，不需要确认


# ==========================================================================
# 总线级
# ==========================================================================


def test_discover_finds_the_node():
    with Context(VirtualHal()) as ctx:
        ctx.add_joint(1)
        assert ctx.discover() == [1]


def test_discover_passive_only():
    """``max_probe=0`` = 仅被动听心跳（虚拟设备会主动发心跳，所以仍能发现）。"""
    with Context(VirtualHal()) as ctx:
        ctx.add_joint(1)
        assert 1 in ctx.discover(max_probe=0)


def test_bus_state(ctx_joint):
    ctx, _j = ctx_joint
    bs = ctx.bus_state()
    d = bs.as_dict()
    assert d["link_up"] is True
    assert d["tx_frames"] > 0
    assert d["rx_frames"] > 0
    assert d["link_errors"] == 0


def test_last_error_is_string(ctx_joint):
    ctx, _j = ctx_joint
    assert isinstance(ctx.last_error(), str)


# ==========================================================================
# 分组广播同步
# ==========================================================================


def test_group_set_mit_single_frame():
    """4 关节由**一条**帧驱动（FD 槽位 = node_id）。"""
    with Context(VirtualHal(virtual_spec(4))) as ctx:
        for n in range(1, 5):
            ctx.add_joint(n)
        ctx.configure()
        ctx.enable_all(Mode.MIT)

        tx_before = ctx.bus_state().tx_frames
        ctx.cycle_begin()
        ctx.group_set_mit({n: {"pos": 0.1 * n, "kp": 5.0} for n in range(1, 5)})
        ctx.cycle_end()

        # 一条广播帧 + 可能的 keepalive；关键是不能退化成 4 条单播
        used = ctx.bus_state().tx_frames - tx_before
        assert used <= 2, f"广播应当只占一条帧（实际 {used}）"


def test_group_set_mit_rejects_over_seven(ctx_enabled):
    """超过 7 个成员必须在**客户端**就被拒（位图只有 7 位）。

    仿真设备最多 4 个节点，所以这里用 8 个"目标"来触发客户端的成员数校验 ——
    它发生在任何总线交互之前。
    """
    ctx, _j = ctx_enabled
    with pytest.raises(jsdk_can.JsdkUnsupportedError):
        ctx.group_set_mit({n: {"pos": 0.0} for n in range(1, 9)})


def test_group_set_mit_requires_enabled_members():
    """未使能的关节不能进组：否则等于替客户驱动一台 IDLE 设备。"""
    with Context(VirtualHal(virtual_spec(2))) as ctx:
        ctx.add_joint(1)
        ctx.add_joint(2)
        ctx.configure()
        with pytest.raises(jsdk_can.JsdkStateError):
            ctx.group_set_mit({1: {"pos": 0.0}})
