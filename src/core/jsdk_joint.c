/**
 * @file    jsdk_joint.c
 * @brief   关节层：反馈解码/合并、状态归一化、5 种模式编码、越界策略、安全首帧
 *
 * @par 单位（**最容易错的地方**，逐条对着 PROTOCOL_NOTES §4.7 实现）
 *
 * | 出口 | 位置 | 速度 | 力矩/电流 |
 * |---|---|---|---|
 * | MIT 命令 | 输出端 rad | 输出端 rad/s | 输出端 N·m |
 * | POS 命令 | 输出端 **度** | 输出端 **RPM** | 电流限制 电机端 A |
 * | VEL 命令 | — | 输出端 **RPM** | 电流限制 电机端 A |
 * | TORQUE 命令 | — | — | **电机端** N·m ⚠ 与 MIT 差一个 gear |
 * | CURRENT 命令 | — | — | 电机端 A |
 * | MIT 响应 | 输出端 rad | 输出端 rad/s | 电流 电机端 A |
 * | QUERY_POS_VEL / 心跳 | **电机端** turns | 电机端 turns/s | 电机端 A |
 *
 * `set_target_torque_Nm()` 对外声明的是**输出端** N·m，因此在 TORQUE 模式下
 * 必须除以 gear_ratio 才能下发（DESIGN §6.1 的表格在这一点上不完整，
 * 已按协议事实实现并在 §12.3 记录）。
 */

#include "jsdk_core_internal.h"

#include <math.h>
#include <string.h>

/* ==========================================================================
 * 量程与标定
 * ======================================================================== */

/**
 * 从描述符读回的标定量程做**合理性校验**（DESIGN §6.6 强制校验第 4 条）。
 * 任一项不满足 → calibrated = 0 且物理量 API 全程拒绝发帧（§6.10）。
 */
static int calib_is_sane(const jsdk_joint_t *j)
{
    const cb_mit_range_t *r = &j->range;

    if (!(j->gear_ratio >= 1.0f && j->gear_ratio <= 1000.0f)) return 0;
    /* 力矩常数：>0 且 ≤ 1（电机 N·m/A 不会超过 1） */
    if (!(j->torque_constant > 0.0f && j->torque_constant <= 1.0f)) return 0;

    if (!(r->pos_max > 0.0f) || !(r->vel_max > 0.0f)) return 0;
    if (!(r->tau_max > 0.0f)) return 0;
    if (!(r->kp_max > 0.0f) || !(r->kd_max > 0.0f)) return 0;

    return 1;
}

void jsdk_joint__apply_calibration(jsdk_joint_t *j)
{
    if (!jsdk_joint_check(j)) return;
    j->max_current_a = cb_mit_response_max_current(j->range.tau_max,
                                                   j->torque_constant);

    if (calib_is_sane(j)) {
        j->calibrated = 1u;
        j->status_flags = (uint16_t)(j->status_flags & ~(uint16_t)JSDK_JF_SCALE_INVALID);
        /* CAN 线上量已是物理量 → 恒等；仍走一次 calc 以便客户用 get_scale 看到"有效" */
        jsdk_unit_scale_default(&j->scale, 1u);
    } else {
        j->calibrated = 0u;
        j->scale.valid = 0;
        jsdk_unit_scale_default(&j->scale, 0u);
        j->status_flags |= (uint16_t)JSDK_JF_SCALE_INVALID;
        jsdk_joint_seterr(j,
            "calibration rejected: gear=");
        {
            char a[24], b[24], c[24];
            jsdk_fmt_f(a, sizeof a, (double)j->gear_ratio, 3u);
            jsdk_fmt_f(b, sizeof b, (double)j->torque_constant, 6u);
            jsdk_fmt_f(c, sizeof c, (double)j->range.pos_max, 3u);
            jsdk_joint_seterr(j, "calibration rejected: gear=%s tconst=%s pos_max=%s", a, b, c);
        }
    }
}

/* ==========================================================================
 * 标志位
 * ======================================================================== */

void jsdk_joint_set_flags(jsdk_joint_t *j, uint16_t flags)
{
    if (!j) return;
    j->status_flags |= flags;
    j->fb.status_flags = j->status_flags;
}

void jsdk_joint_clear_status_flags(jsdk_joint_t *j, uint16_t mask)
{
    if (!jsdk_joint_check(j)) return;
    j->status_flags = (uint16_t)(j->status_flags & (uint16_t)~mask);
    j->fb.status_flags = j->status_flags;
}

/* ==========================================================================
 * 状态归一化
 * ------------------------------------------------------------------------
 * 固件只报两个来源：MIT 响应的 4-bit ModeState，与心跳的 4-bit AxisState。
 * 两者语义不同（ModeState 更细），SDK 统一成 jsdk_axis_state_t：
 *   IDLE(2)          → SWITCH_ON_DISABLED
 *   CALIBRATING(1)   → READY_TO_SWITCH_ON
 *   CLOSED_LOOP(3)   → 还没发过控制帧 → SWITCHED_ON；否则 OPERATION_ENABLED
 *   MIT(4)/POS(5)/VEL(6)/TORQUE(7) → OPERATION_ENABLED
 *   任一错误位/错误码置位       → FAULT（优先级最高）
 * ======================================================================== */

void jsdk_joint__refresh_state(jsdk_joint_t *j)
{
    jsdk_axis_state_t st;
    uint8_t ms = j->fb.mode_state;

    if (!jsdk_joint_check(j)) return;

    if (!j->fb.online) {
        st = JSDK_AXIS_UNKNOWN;
    } else if (jsdk_joint_is_fault(j)) {
        st = JSDK_AXIS_FAULT;
    } else {
        switch (ms) {
        case JSDK_MODESTATE_MIT:
        case JSDK_MODESTATE_POSITION:
        case JSDK_MODESTATE_VELOCITY:
        case JSDK_MODESTATE_TORQUE:
            st = JSDK_AXIS_OPERATION_ENABLED;
            break;
        case JSDK_MODESTATE_CLOSED_LOOP:
            st = j->first_frame_done ? JSDK_AXIS_OPERATION_ENABLED
                                     : JSDK_AXIS_SWITCHED_ON;
            break;
        case JSDK_MODESTATE_CALIBRATING:
            st = JSDK_AXIS_READY_TO_SWITCH_ON;
            break;
        default:                       /* RESET / IDLE / 未知 */
            st = JSDK_AXIS_SWITCH_ON_DISABLED;
            break;
        }
    }

    /* ⚠ **不**用设备上报覆盖客户选择的模式。
       使能过渡期设备会先报 POSITION/CLOSED_LOOP 等 nibble，若据此改 j->mode，
       刚 request_enable(MIT) 的客户会在下一帧被静默改成 POS 指令。
       设备真实的模式看 `fb.mode_state`（原始 nibble）与 `axis_state`。 */

    /* ⚠ `enabled` 必须等到**使能序列走完（含安全首帧）**才为真。
       否则会出现这个静默陷阱：设备一进闭环就报 ModeState = POSITION
       （因为此时还没收到过任何 MIT 帧，input_mode 仍是设备默认值），
       客户看到 is_enabled() == 1 就立即下发生运动指令，而
       §6.4 要求的安全首帧还没发出去 —— “使能瞬间大跳变”就这么发生。 */
    j->enabled = (uint8_t)(((st == JSDK_AXIS_OPERATION_ENABLED
                             || st == JSDK_AXIS_SWITCHED_ON)
                            && !j->enable_pending) ? 1u : 0u);
    j->fb.axis_state = st;
    j->fb.mode       = j->mode;

    /* 故障回调：仅在**边沿**触发（DESIGN §6.8） */
    {
        uint8_t now = jsdk_joint_is_fault(j) ? 1u : 0u;
        if (now && !j->fault_prev) {
            /* WP4：边沿时写一条**带恢复路径**的错误串 —— 现场工程师
               最需要的就是“接下来该干什么”，而不是只有一个错误码。 */
            jsdk_joint_seterr(j,
                "fault detected (can_state=%s err=%s hb=0x%X); recovery: "
                "request_fault_reset() then request_enable()",
                jsdk_can_axis_state_name(j->current_state_raw),
                jsdk_joint_error_string(j->fb.err_code),
                (unsigned)j->fb.hb_error);
            if (j->ctx && j->ctx->fault_cb) {
                j->ctx->fault_cb(j, &j->fault, j->ctx->fault_user);
            }
        }
        j->fault_prev = now;
    }
}

/* ==========================================================================
 * 反馈入口
 * ======================================================================== */

static void touch_feedback(jsdk_joint_t *j, uint32_t now_ms)
{
    j->last_fb_ms = now_ms;
    j->fb.online  = 1;
    j->fb.valid   = 1;
    j->fb.age_ms  = 0u;
}

void jsdk_joint__on_mit_response(jsdk_joint_t *j, const uint8_t *data, uint8_t len)
{
    cb_mit_response_t r;

    if (!jsdk_joint_check(j) || !data || len < 8u) return;

    /* 量程未知时仍解出错误码/模式/温度（这些不依赖量程） */
    cb_mit_unpack_response_raw(data, &r);
    if (j->calibrated) {
        cb_mit_unpack_response(data, &j->range, j->max_current_a, &r);
    }

    j->fb.pos        = (double)r.pos;
    j->fb.vel        = (double)r.vel;
    j->fb.current_A  = (double)r.current;
    j->fb.torque_Nm  = (double)r.current * (double)j->torque_constant
                     * (double)j->gear_ratio;   /* 估算：电流→电机端力矩→输出端 */
    j->fb.t_motor_C  = (double)r.motor_temp_c;
    j->fb.t_fet_C    = (double)r.mos_temp_c;
    j->fb.err_code   = r.err_code;
    j->fb.mode_state = r.mode;
    j->fb.axis_error = j->fault.axis_error;

    touch_feedback(j, j->ctx ? j->ctx->now_ms : 0u);
    jsdk_joint__refresh_state(j);
}

void jsdk_joint__on_heartbeat(jsdk_joint_t *j, const uint8_t *data, uint8_t len)
{
    cb_heartbeat_t hb;

    if (!jsdk_joint_check(j) || !data) return;
    if (cb_heartbeat_decode(data, len, &hb) != 0) return;

    j->hb_seen = 1u;
    j->hb_seen_ms = j->ctx ? j->ctx->now_ms : 0u;
    j->fb.hb_error = hb.err_flags;

    /* 心跳是**电机端 turns**；只在本次周期还没有更新位置时才用它兜底，
       避免用粗分辨率（Classic 下 turns×100）覆盖 MIT 响应的输出端 rad。 */
    if (!j->fb.valid) {
        j->fb.pos = jsdk_units_turns_to_rad((double)hb.pos_turns, (double)j->gear_ratio);
        j->fb.vel = jsdk_units_turns_to_rad((double)hb.vel_turns_per_s,
                                           (double)j->gear_ratio);
    }
    j->fb.t_motor_C = (double)hb.motor_temp_c;
    if (hb.have_mos_temp) j->fb.t_fet_C = (double)hb.mos_temp_c;
    if (hb.have_vbus) {
        j->fb.vbus_V = (double)hb.vbus_v;
        j->fb.ibus_A = (double)hb.ibus_a;
    }

    /*
     * 心跳 byte[1] 的高 4 bit **就是** `axis.current_state_`（固件
     * `send_heartbeat()` 用 `(state << 4) | control_mode` 打包），所以心跳是
     * **不需要额外请求**就能拿到设备真实 AxisState 的低成本通道。
     *
     * ⚠ 以前这里不写回 `current_state_raw`，于是 `jsdk_joint_get_can_state()`
     *   只在 `configure()` 时读到一次（那一刻通常是 IDLE=1）就**永远停在 1** ——
     *   实测：`activate()` 后关节已在闭环（心跳 state=8），而 `can_state()` 仍报 1。
     *   这不只是显示问题：客户按 `can_state()` 判断"是否已使能"会得到错误结论，
     *   与 `is_enabled()`/`mode_state()` 互相矛盾。
     *
     * 只在**状态真的变了**时才更新，避免把 `state_known`/seq 相关的语义搅乱；
     * 标定/回零期间的 `read_current_state()` 轮询仍是更权威的来源，会被它覆盖。
     */
    if (j->current_state_raw != hb.state) {
        j->current_state_raw = hb.state;
        j->state_known = 1u;
    }

    touch_feedback(j, j->ctx ? j->ctx->now_ms : 0u);
    jsdk_joint__refresh_state(j);
}

void jsdk_joint__on_pos_vel_turns(jsdk_joint_t *j, float pos_turns, float vel_turns_s)
{
    if (!jsdk_joint_check(j)) return;
    j->fb.pos = jsdk_units_turns_to_rad((double)pos_turns, (double)j->gear_ratio);
    j->fb.vel = jsdk_units_turns_to_rad((double)vel_turns_s, (double)j->gear_ratio);
    touch_feedback(j, j->ctx ? j->ctx->now_ms : 0u);
    jsdk_joint__refresh_state(j);
}

void jsdk_joint__on_current_a(jsdk_joint_t *j, double iq_a)
{
    if (!jsdk_joint_check(j)) return;
    j->fb.current_A = iq_a;
    j->fb.torque_Nm = iq_a * (double)j->torque_constant * (double)j->gear_ratio;
    touch_feedback(j, j->ctx ? j->ctx->now_ms : 0u);
    jsdk_joint__refresh_state(j);
}

void jsdk_joint__on_temps(jsdk_joint_t *j, double motor_c, double fet_c)
{
    if (!jsdk_joint_check(j)) return;
    j->fb.t_motor_C = motor_c;
    j->fb.t_fet_C   = fet_c;
    touch_feedback(j, j->ctx ? j->ctx->now_ms : 0u);
}

void jsdk_joint__on_bus_volts(jsdk_joint_t *j, double vbus_v, double ibus_a)
{
    if (!jsdk_joint_check(j)) return;
    j->fb.vbus_V = vbus_v;
    j->fb.ibus_A = ibus_a;
    touch_feedback(j, j->ctx ? j->ctx->now_ms : 0u);
}

void jsdk_joint__on_fault_alert(jsdk_joint_t *j)
{
    if (!jsdk_joint_check(j)) return;
    j->fault.hb_flags = j->fb.hb_error;
    jsdk_joint__refresh_state(j);
}

/* ==========================================================================
 * 反馈读取
 * ======================================================================== */

jsdk_status_t jsdk_joint_get_feedback(const jsdk_joint_t *j, jsdk_joint_feedback_t *fb)
{
    if (!jsdk_joint_check(j) || !fb) return JSDK_ERR_INVALID_ARG;

    *fb = j->fb;
    fb->age_ms       = jsdk_elapsed(j->ctx->now_ms, j->last_fb_ms);
    fb->status_flags = j->status_flags;
    fb->tx_frames    = j->fb.tx_frames;
    return JSDK_OK;
}

/**
 * 非阻塞状态请求（0x41 / 0x44）：只发帧，不等应答。
 *
 * ⚠ 三个刻意为之的点：
 *   ① 用 `jsdk_ctx_send_raw()` 而不是 `jsdk_ctx_send()`：后者会在会话开头
 *      触发**预热**（预算最长 500 ms）—— RT 调用者会被拖住，而正是为了
 *      “不阻塞 tick” 才有这个 API。
 *   ② 不记账、不重发：丢了就丢（0x41 是空闲查询，不值得为它加重发预算；
 *      可信度用 `fb.age_ms` 判）。所以本函数不会碰 `req_timeouts`。
 *   ③ 请求帧**不顶替**控制帧：`jsdk_msgtype_feeds_watchdog()` 只认 ≤0x03 与
 *      0x80〜0x83，0x41/0x44 不喂狗 ⇒ 调用者必须继续照常发控制帧。
 */
jsdk_status_t jsdk_joint_request_state(jsdk_joint_t *j, uint32_t fields)
{
    jsdk_context_t *ctx;

    if (!jsdk_joint_check(j) || !j->ctx) return JSDK_ERR_INVALID_ARG;
    if (fields == 0u
        || (fields & ~(uint32_t)(JSDK_STATE_POS_VEL | JSDK_STATE_CURRENT)) != 0u) {
        jsdk_joint_seterr(j, "request_state: bad field mask 0x%08x", (unsigned)fields);
        return JSDK_ERR_INVALID_ARG;
    }

    ctx = j->ctx;

    if ((fields & JSDK_STATE_POS_VEL) != 0u) {
        if (jsdk_ctx_send_raw(ctx, CB_PRI_QUERY, CB_MSG_QUERY_POS_VEL,
                              j->cfg.node_id, NULL, 0u) != 0) {
            jsdk_joint_seterr(j, "request_state: QUERY_POS_VEL send failed");
            return JSDK_ERR_TRANSPORT;
        }
        ctx->tx_seq = cb_seq_next(ctx->tx_seq);
    }

    if ((fields & JSDK_STATE_CURRENT) != 0u) {
        if (jsdk_ctx_send_raw(ctx, CB_PRI_QUERY, CB_MSG_QUERY_CURRENT,
                              j->cfg.node_id, NULL, 0u) != 0) {
            jsdk_joint_seterr(j, "request_state: QUERY_CURRENT send failed");
            return JSDK_ERR_TRANSPORT;
        }
        ctx->tx_seq = cb_seq_next(ctx->tx_seq);
    }

    return JSDK_OK;
}

int jsdk_joint_is_enabled(const jsdk_joint_t *j)
{
    if (!jsdk_joint_check(j)) return 0;
    return j->enabled ? 1 : 0;
}

int jsdk_joint_is_fault(const jsdk_joint_t *j)
{
    if (!jsdk_joint_check(j)) return 0;
    if (j->fb.err_code != 0u) return 1;
    if (j->fb.hb_error != 0u) return 1;
    if (j->fb.axis_error != 0u) return 1;
    return 0;
}

jsdk_modestate_t jsdk_joint_get_mode_state(const jsdk_joint_t *j)
{
    if (!jsdk_joint_check(j)) return JSDK_MODESTATE_RESET;
    return (jsdk_modestate_t)j->fb.mode_state;
}

/* ==========================================================================
 * 请求（非阻塞；由 cycle_end 的状态机推进）
 * ======================================================================== */

void jsdk_joint_request_enable(jsdk_joint_t *j, jsdk_mode_t mode)
{
    if (!jsdk_joint_check(j)) return;
    switch (mode) {
    case JSDK_MODE_MIT: case JSDK_MODE_CSP: case JSDK_MODE_CSV:
    case JSDK_MODE_CST: case JSDK_MODE_CURRENT:
        break;
    default:
        jsdk_joint_seterr(j, "request_enable: unknown mode %d", (int)mode);
        return;
    }
    j->enable_mode     = mode;
    j->mode            = mode;
    j->enable_pending  = 1u;
    /*
     * ⚠ 必须同时清 disable_pending（与 request_disable 清 enable_pending 对称）。
     *   否则 `disable(); enable();` 连写会留下"两个请求都在排队"的状态：
     *   advance_seq 先跑使能序列（enable 优先），使能完成后 disable_pending 还在，
     *   紧接着又跑失能序列 —— 表现为"使能成功了，过十几拍自己又断了"，
     *   而且没有任何错误码。实测确认过这个序列。
     */
    j->disable_pending = 0u;
    j->faultreset_pending = 0u;
    j->seq_step        = 0u;
}

void jsdk_joint_request_disable(jsdk_joint_t *j)
{
    if (!jsdk_joint_check(j)) return;
    j->disable_pending = 1u;
    j->enable_pending  = 0u;
    j->seq_step        = 0u;
}

void jsdk_joint_request_fault_reset(jsdk_joint_t *j)
{
    if (!jsdk_joint_check(j)) return;
    j->faultreset_pending = 1u;
    j->seq_step           = 0u;
}

void jsdk_joint_set_mode(jsdk_joint_t *j, jsdk_mode_t mode)
{
    if (!jsdk_joint_check(j)) return;
    j->mode = mode;
}

/* ==========================================================================
 * 目标 setter
 * ------------------------------------------------------------------------
 * 全部为 `void`：RT 路径无分支、不返回错误。越界在编码阶段按 §6.10 处理。
 * ======================================================================== */

void jsdk_joint_set_target_position_rad(jsdk_joint_t *j, double rad)
{
    if (!jsdk_joint_check(j)) return;
    j->tgt.pos_rad = rad;
    j->tgt.have_pos = 1u;
}

void jsdk_joint_set_target_velocity_rad_s(jsdk_joint_t *j, double rad_s)
{
    if (!jsdk_joint_check(j)) return;
    j->tgt.vel_rad_s = rad_s;
    j->tgt.have_vel = 1u;
}

void jsdk_joint_set_target_torque_Nm(jsdk_joint_t *j, double nm)
{
    if (!jsdk_joint_check(j)) return;
    j->tgt.tau_Nm = nm;
    j->tgt.have_tau = 1u;
}

void jsdk_joint_set_mit(jsdk_joint_t *j, double pos_rad, double vel_rad_s,
                        double kp, double kd, double tau_Nm)
{
    if (!jsdk_joint_check(j)) return;
    j->tgt.pos_rad   = pos_rad;
    j->tgt.vel_rad_s = vel_rad_s;
    j->tgt.kp        = kp;
    j->tgt.kd        = kd;
    j->tgt.tau_Nm    = tau_Nm;
    j->tgt.have_pos  = 1u;
    j->tgt.have_vel  = 1u;
    j->tgt.have_tau  = 1u;
    j->tgt.have_mit  = 1u;
}

void jsdk_joint_set_mit_stiffness(jsdk_joint_t *j, double pos_rad, double vel_rad_s,
                                  double stiffness, double damping, double tau_Nm)
{
    double g;

    if (!jsdk_joint_check(j)) return;
    g = (double)j->gear_ratio;
    jsdk_joint_set_mit(j, pos_rad, vel_rad_s,
                       jsdk_units_stiffness_to_kp(stiffness, g),
                       jsdk_units_stiffness_to_kp(damping, g),
                       tau_Nm);
}

void jsdk_joint_set_target_position(jsdk_joint_t *j, int32_t raw)
{
    if (!jsdk_joint_check(j)) return;
    j->tgt.raw_pos = raw; j->tgt.have_raw_pos = 1u;
}

void jsdk_joint_set_target_velocity(jsdk_joint_t *j, int32_t raw)
{
    if (!jsdk_joint_check(j)) return;
    j->tgt.raw_vel = raw; j->tgt.have_raw_vel = 1u;
}

void jsdk_joint_set_target_torque(jsdk_joint_t *j, int16_t raw)
{
    if (!jsdk_joint_check(j)) return;
    j->tgt.raw_tau = raw; j->tgt.have_raw_tau = 1u;
}

void jsdk_joint_set_limits(jsdk_joint_t *j, double vel_lim_rad_s, double cur_lim_A)
{
    if (!jsdk_joint_check(j)) return;
    j->tgt.vel_lim_rad_s = vel_lim_rad_s;
    j->tgt.have_limits    = 1u;

    /*
     * 兼容入口：旧 API 给的是**电机端 A**，这里换算成力矩上限（N·m）存起来。
     *
     * ⚠ 线上字段确实是 A（固件：`torque_lim = cur_limit_a * torque_constant`），
     *   但它设的是**torque_lim（正常工作力矩上限）**，**不是** `current_lim`
     *   （过流告警门限）。旧名字 `cur_lim_A` 容易让人以为在改告警门限 ——
     *   新代码请用 `jsdk_joint_set_torque_limit_Nm()`，量纲与语义都不用猜。
     */
    if (j->torque_constant > 0.0) {
        j->tgt.tau_lim_Nm = cur_lim_A * j->torque_constant;
    } else {
        /* 还没标定出 torque_constant：先原样存 A，等标定完再换算是不可以的
           （会静默错量纲），所以这里只记下“客户显式给过 A”并用 0 占位，
           由 configure() 之后的默认值兵底。 */
        j->tgt.tau_lim_Nm = 0.0;
    }
}

void jsdk_joint_set_torque_limit_Nm(jsdk_joint_t *j, double vel_lim_rad_s,
                                    double tau_lim_Nm)
{
    if (!jsdk_joint_check(j)) return;
    j->tgt.vel_lim_rad_s = vel_lim_rad_s;
    j->tgt.tau_lim_Nm    = tau_lim_Nm;
    j->tgt.have_limits    = 1u;
}

void jsdk_joint_set_current_A(jsdk_joint_t *j, double amp)
{
    if (!jsdk_joint_check(j)) return;
    j->tgt.cur_A = amp;
    j->tgt.have_cur = 1u;
}

/* ==========================================================================
 * 安全帧
 * ======================================================================== */

/** 把目标改成"最小能量"（§6.10 / §5.7）。 */
static void make_safe_target(jsdk_joint_t *j)
{
    switch (j->mode) {
    case JSDK_MODE_MIT:
        j->tgt.pos_rad   = j->fb.pos;      /* 实际位置 */
        j->tgt.vel_rad_s = 0.0;
        j->tgt.kp = 0.0; j->tgt.kd = 0.0; j->tgt.tau_Nm = 0.0;
        j->tgt.have_mit = 1u;
        break;
    case JSDK_MODE_CSP:
        j->tgt.pos_rad = j->fb.pos;        /* 保留 vel_lim / cur_lim */
        break;
    case JSDK_MODE_CSV:
        j->tgt.vel_rad_s = 0.0;
        break;
    case JSDK_MODE_CST:
        j->tgt.tau_Nm = 0.0;
        break;
    case JSDK_MODE_CURRENT:
        j->tgt.cur_A = 0.0;
        break;
    default:
        break;
    }
}

void jsdk_joint_hold_position(jsdk_joint_t *j)
{
    if (!jsdk_joint_check(j)) return;
    if (!j->fb.online) return;       /* 位置未知 → 本周期退回 keepalive 语义 */
    make_safe_target(j);
    j->hold_requested = 1u;
}

void jsdk_joint_hold_position_pd(jsdk_joint_t *j, double kp, double kd)
{
    if (!jsdk_joint_check(j)) return;
    if (!j->fb.online) return;
    j->tgt.pos_rad   = j->fb.pos;
    j->tgt.vel_rad_s = 0.0;
    j->tgt.kp = kp; j->tgt.kd = kd; j->tgt.tau_Nm = 0.0;
    j->tgt.have_mit = 1u;
    j->hold_requested = 1u;
}

/* ==========================================================================
 * 编码并发送
 * ======================================================================== */

/** 该模式发出的帧会不会刷新固件的协议级超时计时器（见 jsdk_core_internal.h）。 */
static int mode_feeds_watchdog(uint8_t msgtype)
{
    return jsdk_msgtype_feeds_watchdog(msgtype);
}

/**
 * POS/VEL 帧里那个「限流」字段要填的**电机端 A**。
 *
 * 线上确实以 A 为单位（固件：`torque_lim = cur_limit_a * torque_constant`），
 * 但它驱动的是固件 `motor.config.torque_lim`（**正常工作力矩上限**），
 * **不是** `current_lim`（过流告警门限）。所以这里从 `tau_lim_Nm` 反推 A。
 *
 * ⚠ 必须保证**永不为 0**（除非设备真的禁用了力矩）。
 *   固件每收到一帧 CSP/CSV 都会**无条件覆盖** `torque_lim`：
 *       axis.motor_.config_.torque_lim = cur_limit_a * torque_constant;
 *   一旦这里发出 0，`torque_lim` 就变 0 ⇒ 电流环被钳到 0 ⇒ 电机**不出力但
 *   不报任何错**（`is_enabled()` 仍为 1、无 fault），现场表现为“使能成功却
 *   转不动”。这正是 F32 探测首次失败的原因（脚本忘了 set_limits()）。
 *
 *   因此当 `tau_lim_Nm` 还是 0（客户从未设置、且标定也没给出默认值）时，
 *   退回设备的 `min(current_lim, mit_max_torque)` 语义等价值；
 *   两者都拿不到才用固件自带的 `torque_lim` 默认量级兑底并置 sticky flag。
 */
static double jsdk_joint__tau_lim_to_wire_a(jsdk_joint_t *j)
{
    double tau = j->tgt.tau_lim_Nm;

    if (!(tau > 0.0)) {                       /* 0 或 NaN */
        double fallback = j->tau_lim_default_Nm;   /* configure() 从 current_lim 读回 */
        if (!(fallback > 0.0)) {
            /* 连默认值都没读回来（描述符里没有 current_lim）：用固件
               `motor.hpp` 的 torque_lim 默认量级（2.58 N·m）当保底，
               宁可给出一个保守的非 0 值，也不要让电机静默假死。 */
            fallback = 2.58;
            jsdk_joint_set_flags(j, (uint16_t)JSDK_JF_TORQUE_LIM_UNSET);
        }
        tau = fallback;
    }
    if (j->torque_constant > 0.0) return tau / j->torque_constant;
    return 0.0;
}

/** 越界处理：返回 1 = 已按安全帧替换目标，0 = 目标可用。 */
static int reject_or_clamp(jsdk_joint_t *j)
{
    double lim;
    int    bad = 0;
    char   a[24], b[24];

    /* 只有 MIT 的 pos/vel/tau 与 CSP/CSV 有明确量程；CURRENT/TORQUE 用量程表 */
    if (j->mode == JSDK_MODE_MIT || j->mode == JSDK_MODE_CSP) {
        lim = (double)j->range.pos_max;
        if (j->tgt.pos_rad > lim || j->tgt.pos_rad < -lim) bad = 1;
    }
    if (j->mode == JSDK_MODE_MIT || j->mode == JSDK_MODE_CSV) {
        lim = (double)j->range.vel_max;
        if (j->tgt.vel_rad_s > lim || j->tgt.vel_rad_s < -lim) bad = 1;
    }
    if (j->mode == JSDK_MODE_MIT || j->mode == JSDK_MODE_CST) {
        lim = (double)j->range.tau_max;
        if (j->tgt.tau_Nm > lim || j->tgt.tau_Nm < -lim) bad = 1;
    }
    if (j->mode == JSDK_MODE_MIT) {
        if (j->tgt.kp < 0.0 || j->tgt.kp > (double)j->range.kp_max) bad = 1;
        if (j->tgt.kd < 0.0 || j->tgt.kd > (double)j->range.kd_max) bad = 1;
    }
    if (j->mode == JSDK_MODE_CURRENT) {
        lim = (double)j->max_current_a;
        if (j->tgt.cur_A > lim || j->tgt.cur_A < -lim) bad = 1;
    }
    if (!bad) return 0;

    j->tx_rejected++;
    j->fb.tx_rejected = j->tx_rejected;
    jsdk_joint_set_flags(j, (uint16_t)(JSDK_JF_TARGET_REJECTED | JSDK_JF_SAFE_FRAME_SENT));

    if (j->ctx->cfg.clamp_target_position) {
        /* 策略 1：静默钳位，不记错误串 */
        if (j->mode == JSDK_MODE_MIT || j->mode == JSDK_MODE_CSP) {
            lim = (double)j->range.pos_max;
            if (j->tgt.pos_rad > lim)       j->tgt.pos_rad = lim;
            else if (j->tgt.pos_rad < -lim) j->tgt.pos_rad = -lim;
        }
        if (j->mode == JSDK_MODE_MIT || j->mode == JSDK_MODE_CSV) {
            lim = (double)j->range.vel_max;
            if (j->tgt.vel_rad_s > lim)       j->tgt.vel_rad_s = lim;
            else if (j->tgt.vel_rad_s < -lim) j->tgt.vel_rad_s = -lim;
        }
        if (j->mode == JSDK_MODE_MIT || j->mode == JSDK_MODE_CST) {
            lim = (double)j->range.tau_max;
            if (j->tgt.tau_Nm > lim)       j->tgt.tau_Nm = lim;
            else if (j->tgt.tau_Nm < -lim) j->tgt.tau_Nm = -lim;
        }
        if (j->mode == JSDK_MODE_MIT) {
            if (j->tgt.kp < 0.0) j->tgt.kp = 0.0;
            if (j->tgt.kp > (double)j->range.kp_max) j->tgt.kp = (double)j->range.kp_max;
            if (j->tgt.kd < 0.0) j->tgt.kd = 0.0;
            if (j->tgt.kd > (double)j->range.kd_max) j->tgt.kd = (double)j->range.kd_max;
        }
        if (j->mode == JSDK_MODE_CURRENT) {
            lim = (double)j->max_current_a;
            if (j->tgt.cur_A > lim)       j->tgt.cur_A = lim;
            else if (j->tgt.cur_A < -lim) j->tgt.cur_A = -lim;
        }
        return 0;
    }

    /* 策略 0（默认）：拒绝客户目标，改发安全帧。
       ⚠ 必须**仍然发帧**：看门狗只认控制类帧，不发就会触发 ERROR_CAN_BUS_FAILED。 */
    jsdk_fmt_f(a, sizeof a, j->tgt.pos_rad, 3u);
    jsdk_fmt_f(b, sizeof b, (double)j->range.pos_max, 3u);
    jsdk_joint_seterr(j,
        "target rejected (pos=%s rad, limit=%s rad, mode=%s) -> safe frame sent",
        a, b, jsdk_mode_string(j->mode));
    make_safe_target(j);
    return 1;
}

/**
 * 编码并发送本周期指令。@return 1 = 已发送控制帧；0 = 未发送（调用方决定是否补喂狗）
 */
static int send_one_frame(jsdk_joint_t *j)
{
    jsdk_context_t *ctx = j->ctx;
    uint8_t  payload[CB_CTRL_POS_LEN_FD];
    uint8_t  flags = 0u;
    size_t   n = 0u;
    uint8_t  msgtype;
    int      classic = !ctx->cfg.is_fd;
    int      replaced;

    msgtype = (uint8_t)((j->mode == JSDK_MODE_MIT)     ? CB_MSG_MIT_CONTROL
                      : (j->mode == JSDK_MODE_CSP)     ? CB_MSG_POS_CONTROL
                      : (j->mode == JSDK_MODE_CSV)     ? CB_MSG_VEL_CONTROL
                      : (j->mode == JSDK_MODE_CST)     ? CB_MSG_TORQUE_CONTROL
                                                       : CB_MSG_CURRENT_CONTROL);

    replaced = reject_or_clamp(j);

    switch (j->mode) {
    case JSDK_MODE_MIT: {
        uint8_t clamped = 0u;
        cb_mit_pack_command(payload, &j->range,
                            (float)j->tgt.pos_rad, (float)j->tgt.vel_rad_s,
                            (float)j->tgt.kp, (float)j->tgt.kd,
                            (float)j->tgt.tau_Nm, &clamped);
        n = 8u;
        if (clamped & (uint8_t)(CB_MIT_CLAMP_KP | CB_MIT_CLAMP_KD)) {
            char a[24], b[24];
            jsdk_fmt_f(a, sizeof a, (double)j->range.kp_max, 1u);
            jsdk_fmt_f(b, sizeof b, (double)j->range.kd_max, 1u);
            jsdk_joint_seterr(j, "MIT kp/kd out of range, encoded as clamped "
                                 "(kp_max=%s kd_max=%s)", a, b);
            jsdk_joint_set_flags(j, (uint16_t)JSDK_JF_TARGET_REJECTED);
        }
        break;
    }
    case JSDK_MODE_CSP: {
        float pos_deg = (float)(j->tgt.pos_rad * 180.0 / 3.14159265358979323846);
        float vel_rpm = (float)jsdk_units_rad_s_to_rpm(j->tgt.vel_lim_rad_s);
        float cur_a   = (float)jsdk_joint__tau_lim_to_wire_a(j);
        n = cb_ctrl_pos_pack(payload, classic, pos_deg, vel_rpm, cur_a, &flags);
        break;
    }
    case JSDK_MODE_CSV: {
        float vel_rpm = (float)jsdk_units_rad_s_to_rpm(j->tgt.vel_rad_s);
        n = cb_ctrl_vel_pack(payload, vel_rpm, (float)jsdk_joint__tau_lim_to_wire_a(j),
                             &flags);
        break;
    }
    case JSDK_MODE_CST: {
        /* ⚠ TORQUE 线上是**电机端** N·m；对外给的是输出端 → 除以 gear */
        double g = (j->gear_ratio > 0.0f) ? (double)j->gear_ratio : 1.0;
        n = cb_ctrl_torque_pack(payload, (float)(j->tgt.tau_Nm / g), &flags);
        break;
    }
    case JSDK_MODE_CURRENT:
    default:
        n = cb_ctrl_current_pack(payload, (float)j->tgt.cur_A, &flags);
        break;
    }

    if (n == 0u) return 0;

    {
        int rc = jsdk_ctx_send(ctx, CB_PRI_HIGH_CTRL, msgtype, j->cfg.node_id,
                               payload, (uint8_t)n);
        if (rc != 0) {
            jsdk_joint_set_flags(j, (uint16_t)JSDK_JF_TX_FAILED);
            return 0;
        }
    }

    j->tx_frames++;
    j->fb.tx_frames = j->tx_frames;
    j->sent_cycle = 1u;          /* 本周期已给这个关节发过帧 */
    ctx->tx_seq = cb_seq_next(ctx->tx_seq);

    if (mode_feeds_watchdog(msgtype)) {
        j->last_ctrl_tx_ms = ctx->now_ms;
        j->first_frame_done = 1u;
    }
    if (replaced) {
        jsdk_joint_set_flags(j, (uint16_t)JSDK_JF_SAFE_FRAME_SENT);
    }
    (void)flags;
    return 1;
}

/* ==========================================================================
 * 使能 / 失能 / 复位序列（非阻塞步进，每个周期推进一步）
 * ======================================================================== */

static int mode_is_closed_loop(uint8_t ms)
{
    return (ms == JSDK_MODESTATE_MIT || ms == JSDK_MODESTATE_POSITION
            || ms == JSDK_MODESTATE_VELOCITY || ms == JSDK_MODESTATE_TORQUE
            || ms == JSDK_MODESTATE_CLOSED_LOOP) ? 1 : 0;
}

/** 发一帧"模式轮询"请求：QUERY_STATUS 的应答就是 MIT 响应（含 ModeState）。 */
static void poll_mode(jsdk_joint_t *j)
{
    (void)jsdk_ctx_send(j->ctx, CB_PRI_QUERY, CB_MSG_QUERY_STATUS,
                        j->cfg.node_id, NULL, 0u);
    j->ctx->tx_seq = cb_seq_next(j->ctx->tx_seq);
}

/** @return 1 = 序列完成；0 = 继续进行中；-1 = 超时 */
static int advance_seq(jsdk_joint_t *j)
{
    jsdk_context_t *ctx = j->ctx;
    uint8_t ms = j->fb.mode_state;

    if (j->seq_step == 0u) {
        j->seq_start_ms = ctx->now_ms;
    }

    if (j->enable_pending) {
        switch (j->seq_step) {
        case 0u:
            (void)jsdk_ctx_send(ctx, CB_PRI_CTRL, CB_MSG_CLEAR_ERRORS, j->cfg.node_id, NULL, 0u);
            j->seq_step = 1u;
            return 0;
        case 1u:
            (void)jsdk_ctx_send(ctx, CB_PRI_CTRL, CB_MSG_START_MOTOR, j->cfg.node_id, NULL, 0u);
            j->seq_step = 2u;
            return 0;
        case 2u:
            if (mode_is_closed_loop(ms)) {
                j->seq_step = 3u;
                return 0;
            }
            poll_mode(j);
            break;
        case 3u:
            /* 安全首帧：CSP 用当前位置，其余模式用 0（§6.4）。
               ⚠ 用 **enable_mode**，不能用 `j->mode`：客户可能在序列进行中
                  调了 set_mode()，那会让“使能首帧”发成另一种模式。 */
            j->mode = j->enable_mode;
            j->first_frame_done = 0u;
            if (!j->tgt.have_pos) {
                j->tgt.have_pos  = 1u;
                j->tgt.pos_rad   = j->fb.pos;
            }
            if (!j->tgt.have_vel) { j->tgt.have_vel = 1u; j->tgt.vel_rad_s = 0.0; }
            if (!j->tgt.have_tau) { j->tgt.have_tau = 1u; j->tgt.tau_Nm = 0.0; }
            if (!j->tgt.have_mit) {
                j->tgt.kp = 0.0; j->tgt.kd = 0.0; j->tgt.have_mit = 1u;
            }
            (void)send_one_frame(j);
            j->enable_pending = 0u;
            j->seq_step = 0u;
            j->tx_active = 1u;      /* 从现在起本关节才由 SDK 驱动 */
            return 1;
        default:
            break;
        }
    } else if (j->disable_pending) {
        switch (j->seq_step) {
        case 0u:
            /* 先发一帧零/保持指令，避免"停发即故障" */
            make_safe_target(j);
            (void)send_one_frame(j);
            j->seq_step = 1u;
            return 0;
        case 1u:
            if (jsdk_elapsed(ctx->now_ms, j->seq_start_ms) < 2u * JSDK_CFG_PERIOD_FALLBACK_MS) {
                return 0;    /* 等 2 个周期 */
            }
            j->seq_step = 2u;
            return 0;
        case 2u:
            (void)jsdk_ctx_send(ctx, CB_PRI_CTRL, CB_MSG_STOP_MOTOR, j->cfg.node_id, NULL, 0u);
            j->seq_step = 3u;
            return 0;
        case 3u:
            if (ms == JSDK_MODESTATE_IDLE || ms == JSDK_MODESTATE_RESET) {
                j->disable_pending = 0u;
                j->seq_step = 0u;
                j->enabled = 0u;
                j->tx_active = 0u;   /* 停止发帧 → 也让协议级超时保护自然失效 */
                return 1;
            }
            poll_mode(j);
            break;
        default:
            break;
        }
    } else if (j->faultreset_pending) {
        switch (j->seq_step) {
        case 0u:
            (void)jsdk_ctx_send(ctx, CB_PRI_CTRL, CB_MSG_STOP_MOTOR, j->cfg.node_id, NULL, 0u);
            j->seq_step = 1u;
            return 0;
        case 1u:
            (void)jsdk_ctx_send(ctx, CB_PRI_CTRL, CB_MSG_CLEAR_ERRORS, j->cfg.node_id, NULL, 0u);
            j->seq_step = 2u;
            return 0;
        case 2u:
            if (j->fb.err_code == 0u && j->fb.hb_error == 0u && j->fb.axis_error == 0u) {
                j->faultreset_pending = 0u;
                j->seq_step = 0u;
                j->fault_prev = 0u;
                j->status_flags &= (uint16_t)~(uint16_t)(JSDK_JF_TARGET_REJECTED
                                                        | JSDK_JF_SAFE_FRAME_SENT
                                                        | JSDK_JF_TX_FAILED);
                j->fb.status_flags = j->status_flags;
                return 1;
            }
            poll_mode(j);
            break;
        default:
            break;
        }
    } else {
        j->seq_step = 0u;
        return 1;
    }

    /* 统一超时判定（配置阶段用 200 ms；控制循环里同样适用） */
    if (jsdk_elapsed(ctx->now_ms, j->seq_start_ms) > JSDK_CFG_TIMEOUT_MS) {
        jsdk_joint_seterr(j, "enable/disable sequence timed out (mode_state=%u)",
                          (unsigned)ms);
        j->enable_pending = j->disable_pending = j->faultreset_pending = 0u;
        j->seq_step = 0u;
        return -1;
    }
    return 0;
}

int jsdk_joint__send_now(jsdk_joint_t *j)
{
    if (!jsdk_joint_check(j)) return 0;
    if (!j->calibrated) return 0;
    if (j->sent_cycle) return 0;      /* 本周期已经发过（例如广播帧） */
    return send_one_frame(j);
}

/* ==========================================================================
 * cycle_end
 * ======================================================================== */

void jsdk_joint__cycle_end_all(jsdk_context_t *ctx)
{
    unsigned i;

    if (!jsdk_ctx_check(ctx)) return;

    for (i = 0u; i < ctx->nj; ++i) {
        jsdk_joint_t *j = &ctx->joints[i];

        /* 量程未标定 → 不发任何控制帧（§6.10：绝不用错误量程算出错误力矩） */
        if (!j->calibrated) continue;

        /* 还没使能过 → 绝不替客户发帧（见 tx_active 的说明） */
        if (!j->tx_active && !j->enable_pending) { j->hold_requested = 0u; continue; }

        /* 本周期已经发过（例如已经用广播帧驱动过这个关节） → 不重复发单播 */
        if (j->sent_cycle) { j->hold_requested = 0u; continue; }

        /* 标定/回零进行中 → 设备在转电机，客户指令会打架（§6.4） */
        if (j->ctrl_blocked) { j->hold_requested = 0u; continue; }

        /* 序列进行中（含**本周期刚完成**）：本周期不再额外发控制帧。
           enable 的最后一步自己会发安全首帧；disable 的最后一步之后
           再发控制帧会把设备重新拉回闭环（刚发的 STOP_MOTOR 就白发了）。 */
        if (j->enable_pending || j->disable_pending || j->faultreset_pending) {
            (void)advance_seq(j);
            j->hold_requested = 0u;
            continue;
        }

        (void)send_one_frame(j);
        j->hold_requested = 0u;
    }
}
