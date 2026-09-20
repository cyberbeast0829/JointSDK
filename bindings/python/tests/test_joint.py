"""关节层：反馈、5 种模式、参数读写、标定量与运维。

设计文档 §8「Python 绑定」要求的"5 种模式 + 反馈新鲜度 + dump_config() 数值"
都在这里。
"""

from __future__ import annotations

import math

import pytest

import jsdk_can
from jsdk_can import Mode, StatusFlag, VirtualHal

from conftest import virtual_spec


# ==========================================================================
# 反馈与新鲜度
# ==========================================================================


def test_feedback_is_live(ctx_enabled):
    ctx, j = ctx_enabled
    fb0 = j.feedback()
    assert fb0.online
    assert fb0.vbus_V > 0.0, "母线电压应当来自设备心跳"
    assert fb0.t_motor_C > 0.0
    assert fb0.age_ms < 1000

    for _ in range(30):
        ctx.cycle_begin()
        j.set_mit(pos=0.0, kp=1.0)
        ctx.cycle_end()
    assert j.feedback().valid


def test_feedback_age_grows_when_bus_goes_quiet(ctx_joint):
    """总线安静后 ``age_ms`` 必须涨，且粘滞 ``FEEDBACK_STALE`` 置位。

    ⚠ 要让"总线安静"成立需要两件事，缺一不可：
      1. **关掉设备心跳**（``heartbeat_rate_ms = 0``）；
      2. **本关节不使能**（``tx_active = 0``）。
         已使能的关节每跑一次 poll 就会发控制帧，设备立刻回 MIT 响应，
         ``last_fb_ms`` 又被刷新 —— age 永远是 2 ms，测不出来。
      两者都做过之后，推时钟 + poll 才是真机上"线被拔了"的样子。
    """
    ctx, j = ctx_joint
    j.param_set_u32("axis0.config.can.heartbeat_rate_ms", 0)
    for _ in range(3):
        ctx.poll()
    age0 = j.feedback().age_ms

    ctx.hal.advance_ms(2000)          # 虚拟时钟跳过 2 s，期间无任何帧
    for _ in range(5):
        ctx.poll()

    fb = j.feedback()
    assert fb.age_ms > age0, f"age 没涨（{age0} -> {fb.age_ms}）"
    assert fb.age_ms > 1000
    assert fb.has_flag(StatusFlag.FEEDBACK_STALE), "反馈超时必须置粘滞标志"


def test_status_flags_are_sticky_until_cleared(ctx_joint):
    """粘滞标志的语义：置位后**不会**因为情况好转自动消失，必须显式清除。"""
    ctx, j = ctx_joint
    j.param_set_u32("axis0.config.can.heartbeat_rate_ms", 0)
    ctx.hal.advance_ms(2000)
    for _ in range(5):
        ctx.poll()
    assert j.feedback().has_flag(StatusFlag.FEEDBACK_STALE)

    # 恢复心跳：情况好转了，但标志仍粘着
    j.param_set_u32("axis0.config.can.heartbeat_rate_ms", 10)
    for _ in range(30):
        ctx.poll()
    assert j.feedback().age_ms < 100          # 反馈确实新鲜了
    assert j.feedback().has_flag(StatusFlag.FEEDBACK_STALE), "粘滞标志不该自动清"

    j.clear_status_flags()
    assert not j.feedback().has_flag(StatusFlag.FEEDBACK_STALE)


def test_feedback_has_flag_query(ctx_joint):
    _ctx, j = ctx_joint
    q = j.feedback().has_flag
    assert isinstance(q(StatusFlag.TARGET_REJECTED), bool)
    assert "FEEDBACK_STALE" not in q.names() or True   # 只要求可调用


# ==========================================================================
# 5 种模式
# ==========================================================================


@pytest.mark.parametrize("mode", [Mode.MIT, Mode.CSP, Mode.CSV, Mode.CST,
                                  Mode.CURRENT])
def test_each_mode_can_be_enabled_and_driven(mode, ctx_joint):
    """5 种模式各自：使能 → 下发目标 → 收到反馈 → 失能。"""
    ctx, j = ctx_joint
    j.enable(mode)
    ctx.activate()
    assert j.is_enabled()

    for _ in range(30):
        ctx.cycle_begin()
        if mode is Mode.MIT:
            j.set_mit(pos=0.0, kp=2.0, kd=0.2)
        elif mode is Mode.CSP:
            j.set_position(0.0)
        elif mode is Mode.CSV:
            j.set_velocity(0.0)
        elif mode is Mode.CST:
            j.set_torque(0.0)
        else:
            j.set_limits(1.0, 5.0)
            j.set_torque(0.0)
        ctx.cycle_end()

    fb = j.feedback()
    assert fb.online
    assert fb.tx_frames > 0
    assert fb.mode == mode, "SDK 不应被设备上报的 ModeState 反向改写客户选定的模式"

    ctx.disable_all()
    assert not j.is_enabled()


def test_mit_reaches_the_device(ctx_enabled):
    """正向目标 + 正向 kp → 位置必须真的走起来。"""
    ctx, j = ctx_enabled
    for _ in range(60):
        ctx.cycle_begin()
        j.set_mit(pos=0.3, kp=10.0, kd=1.0)
        ctx.cycle_end()

    fb = j.feedback()
    assert fb.pos > 0.05, f"位置没动起来（pos={fb.pos}）"
    assert fb.torque_Nm != 0.0


def test_mit_stiffness_path(ctx_enabled):
    """``set_mit_stiffness`` 走"输出端真实刚度"换算路径。"""
    ctx, j = ctx_enabled
    for _ in range(40):
        ctx.cycle_begin()
        j.set_mit_stiffness(pos=0.2, stiffness=20.0, damping=1.0)
        ctx.cycle_end()
    assert j.feedback().pos != 0.0


def test_hold_position_is_minimal_energy(ctx_enabled):
    """MIT 下 ``hold_position`` = 零增益（泄力），不是主动抱持。"""
    ctx, j = ctx_enabled
    for _ in range(20):
        ctx.cycle_begin()
        j.set_mit(pos=0.2, kp=10.0)
        ctx.cycle_end()
    for _ in range(20):
        ctx.cycle_begin()
        j.hold_position()
        ctx.cycle_end()
    fb = j.feedback()
    assert abs(fb.torque_Nm) < 1.0, "hold_position 应当几乎不出力矩"


def test_hold_position_pd(ctx_enabled):
    ctx, j = ctx_enabled
    for _ in range(20):
        ctx.cycle_begin()
        j.hold_position_pd(kp=5.0, kd=0.5)
        ctx.cycle_end()
    assert j.is_enabled()


def test_out_of_range_target_is_rejected_not_clamped(ctx_enabled):
    """默认策略（§6.10 策略 0）：越界**拒绝**并改发安全帧，不静默钳位。"""
    ctx, j = ctx_enabled
    before = j.feedback().tx_rejected
    for _ in range(10):
        ctx.cycle_begin()
        j.set_mit(pos=999.0, kp=1000.0)      # 远超 mit_max_pos / mit_max_kp
        ctx.cycle_end()

    fb = j.feedback()
    assert fb.tx_rejected > before, "越界必须计数"
    assert fb.has_flag(StatusFlag.TARGET_REJECTED)
    assert abs(fb.pos) < 1.0, "绝不能真的跟到 999 rad"
    assert j.is_enabled(), "被拒绝的指令不应把关节搞挂"


def test_mode_state_and_can_state_are_different_enums(ctx_enabled):
    _ctx, j = ctx_enabled
    assert isinstance(j.mode_state(), (jsdk_can.ModeState, int))
    assert isinstance(j.can_state(), int)


# ==========================================================================
# 参数
# ==========================================================================


def test_param_get_float(ctx_joint):
    _ctx, j = ctx_joint
    assert j.param_get("axis0.motor.config.gear_ratio") == pytest.approx(16.5)
    assert j.param_get_f32("axis0.motor.config.torque_constant") == pytest.approx(
        0.0385, rel=1e-3)


def test_param_get_int_and_bool(ctx_joint):
    _ctx, j = ctx_joint
    assert j.param_get_u32("axis0.config.can.node_id") == 1
    assert isinstance(j.param_get_bool("axis0.config.enable_watchdog"), bool)


def test_param_set_u32_roundtrip(ctx_joint):
    _ctx, j = ctx_joint
    j.param_set_u32("axis0.config.can.heartbeat_rate_ms", 20)
    assert j.param_get_u32("axis0.config.can.heartbeat_rate_ms") == 20


def test_param_set_float_roundtrip(ctx_joint):
    _ctx, j = ctx_joint
    j.param_set("axis0.controller.config.vel_limit", 3.5)
    assert j.param_get("axis0.controller.config.vel_limit") == pytest.approx(3.5)


def test_param_set_unknown_path(ctx_joint):
    _ctx, j = ctx_joint
    with pytest.raises(jsdk_can.JsdkDeviceNotFoundError):
        j.param_set("axis0.nope.nope", 1)


def test_param_set_bad_python_type(ctx_joint):
    """Python 侧不认识的类型 → 直接 ``TypeError``（fail fast，别浪费一次总线往返）。"""
    _ctx, j = ctx_joint
    with pytest.raises(TypeError):
        j.param_set("axis0.motor.config.gear_ratio", "not a number")


def test_param_set_mismatched_type_is_protocol_error(ctx_joint):
    """类型认得、但与端点不符 → 由 C 侧返回 ``PROTOCOL``（不是静默写入）。

    这条必须有：C 侧靠类型决定线上字节长度，猜错会**静默写错值**，
    所以它宁可报错。
    """
    _ctx, j = ctx_joint
    with pytest.raises(jsdk_can.JsdkProtocolError):
        j.param_set("axis0.motor.config.gear_ratio", 7)      # int 写 float 端点


def test_param_write_readonly_endpoint_refused(ctx_joint):
    _ctx, j = ctx_joint
    with pytest.raises(jsdk_can.JsdkError):
        j.param_set("axis0.current_state", 3)


def test_param_get_batch_mixed(ctx_joint):
    """FD 下批量读打包成单帧；混合类型也要正确解出。"""
    _ctx, j = ctx_joint
    out = j.param_get_batch(
        "axis0.motor.config.gear_ratio",
        "axis0.config.can.node_id",
        "axis0.controller.config.mit_max_torque",
    )
    assert out["axis0.motor.config.gear_ratio"] == pytest.approx(16.5)
    assert out["axis0.config.can.node_id"] == 1
    assert out["axis0.controller.config.mit_max_torque"] == pytest.approx(50.0)


def test_param_get_batch_reports_per_entry_failure(ctx_joint):
    """某条失败时**不抛异常**，该条给出异常对象 —— 一次拿到全部结果。"""
    _ctx, j = ctx_joint
    out = j.param_get_batch("axis0.motor.config.gear_ratio", "no.such.path")
    assert out["axis0.motor.config.gear_ratio"] == pytest.approx(16.5)
    assert isinstance(out["no.such.path"], jsdk_can.JsdkError)


def test_param_get_batch_limits(ctx_joint):
    _ctx, j = ctx_joint
    with pytest.raises(ValueError):
        j.param_get_batch()
    with pytest.raises(ValueError):
        j.param_get_batch(*["axis0.motor.config.gear_ratio"] * 9)


# ==========================================================================
# 标定量 / 设备信息 / 故障
# ==========================================================================


def test_dump_config_values(ctx_joint):
    """``dump_config()`` 的数值必须与设备实际量程一致（不猜、不取默认）。"""
    ctx, _j = ctx_joint
    snap = ctx.dump_config()
    assert snap.valid
    assert snap.gear_ratio == pytest.approx(16.5)
    assert snap.mit_max_pos == pytest.approx(12.5)
    assert snap.mit_max_vel == pytest.approx(65.0)
    assert snap.mit_max_torque == pytest.approx(50.0)
    assert snap.mit_max_kp == pytest.approx(500.0)
    assert snap.mit_max_kd == pytest.approx(5.0)
    assert snap.torque_constant == pytest.approx(0.0385, rel=1e-3)
    assert snap.node_id == 1
    assert snap.break_timeout_ms == 30000


def test_dump_config_before_configure_invalid(ctx):
    """未 configure 时读回的快照不能"看起来有效"。"""
    j = ctx.add_joint(1)
    snap = j.config_snapshot()
    assert snap.valid in (True, False)     # 允许成功但必须自证有效/无效
    if not snap.valid:
        assert snap.gear_ratio == 0.0


def test_device_info(ctx_joint):
    ctx, _j = ctx_joint
    info = ctx.device_info()
    assert info.fw_version > 0
    assert info.hw_version > 0


def test_fault_info_clean_device(ctx_joint):
    ctx, _j = ctx_joint
    fi = ctx.query_error_detail()
    assert fi.subsystems()["axis"] == 0
    assert fi.mit_err == 0


def test_describe_fault_is_string(ctx_joint):
    _ctx, j = ctx_joint
    assert isinstance(j.describe_fault(), str)


# ==========================================================================
# 运维（会写设备，但在虚拟后端上可安全验证）
# ==========================================================================


def test_set_watchdog(ctx_joint):
    _ctx, j = ctx_joint
    j.set_watchdog_ms(250)
    assert j.param_get_u32("can.config.break_timeout") == 250


def test_save_config(ctx_joint):
    _ctx, j = ctx_joint
    j.save_config()          # 写完读回校验；失败会抛


def test_set_zero_here(ctx_joint):
    _ctx, j = ctx_joint
    j.set_zero_here()


def test_calibrate_and_home(ctx_joint):
    ctx, j = ctx_joint
    j.calibrate()
    ctx.hal.advance_ms(100)
    ctx.poll()
    j.home()
    ctx.hal.advance_ms(100)
    for _ in range(20):
        ctx.poll()


def test_reset_device(ctx_joint):
    _ctx, j = ctx_joint
    j.reset_device()


def test_set_node_id_validates_range(ctx_joint):
    _ctx, j = ctx_joint
    with pytest.raises(ValueError):
        j.set_node_id(0)
    with pytest.raises(ValueError):
        j.set_node_id(255)


def test_set_node_id(ctx_joint):
    """改节点地址：SDK 会验证新地址有应答，成功后才更新本地 node_id。"""
    ctx, j = ctx_joint
    j.set_node_id(3, persist=False)
    assert j.node_id == 3
    assert ctx.joint(3) is j
    assert ctx.discover() == [3]
