/**
 * @file    sim_device.c
 * @brief   虚拟驱动器行为模型实现（见 sim_device.h）
 *
 * 逐条对照固件 Firmware/communication/can/can_cyberbeast.cpp 实现；
 * 所有帧编解码都复用 L2 的 cb_* 模块，**不重复实现协议**——
 * 这样模拟器与 SDK 共享同一套协议理解，黄金向量测试也能反过来约束模拟器。
 */

#include "sim_device.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cb_frame.h"
#include "cb_mit.h"
#include "cb_ctrl.h"
#include "cb_query.h"
#include "cb_heartbeat.h"
#include "cb_param.h"

#define SIM_PI 3.14159265358979323846f

/** sim_tick 单次推进的毫秒上限（防止一次跳太远导致巨量循环） */
#define SIM_MAX_TICK_MS 20000u

/* 前向声明（send_mit_response 早于它们使用） */
static uint8_t detect_error_code(const sim_node_t *n);
static uint8_t detect_mode_state(const sim_node_t *n);
static void node_defaults(sim_node_t *n);

/* ==========================================================================
 * 端点表（ID 与类型取自固件 v8 真实 JSON 描述符）
 * ======================================================================== */

#define EP(id, path, type, acc, field)                                        \
    { (uint16_t)(id), (path), (uint8_t)(type), (uint8_t)(acc),                \
      offsetof(sim_node_t, field) }

static const sim_ep_def_t k_eps[] = {
    EP(  1, "error",                                        SIM_T_U8,  SIM_ACC_READ | SIM_ACC_WRITE, error_board),
    EP(  2, "vbus_voltage",                                 SIM_T_F32, SIM_ACC_READ,                 vbus_voltage),
    EP(  3, "ibus",                                         SIM_T_F32, SIM_ACC_READ,                 ibus),
    EP(  5, "serial_number",                                SIM_T_U64, SIM_ACC_READ,                 serial_number),
    EP(  6, "hw_version_major",                             SIM_T_U8,  SIM_ACC_READ,                 hw_version_major),
    EP(  9, "fw_version_major",                             SIM_T_U8,  SIM_ACC_READ,                 fw_version_major),
    EP( 73, "can.config.break_timeout",                     SIM_T_U16, SIM_ACC_READ | SIM_ACC_WRITE, break_timeout),
    EP(138, "axis0.error",                                  SIM_T_U32, SIM_ACC_READ | SIM_ACC_WRITE, error_axis),
    EP(142, "axis0.current_state",                          SIM_T_U8,  SIM_ACC_READ,                 current_state),
    EP(143, "axis0.requested_state",                        SIM_T_U8,  SIM_ACC_READ | SIM_ACC_WRITE, requested_state),
    EP(153, "axis0.config.watchdog_timeout",                SIM_T_F32, SIM_ACC_READ | SIM_ACC_WRITE, watchdog_timeout),
    EP(154, "axis0.config.enable_watchdog",                 SIM_T_BOOL,SIM_ACC_READ | SIM_ACC_WRITE, enable_watchdog),
    EP(180, "axis0.config.can.node_id",                     SIM_T_U32, SIM_ACC_READ | SIM_ACC_WRITE, node_id),
    EP(181, "axis0.config.can.is_extended",                 SIM_T_BOOL,SIM_ACC_READ | SIM_ACC_WRITE, is_extended),
    EP(182, "axis0.config.can.heartbeat_rate_ms",           SIM_T_U32, SIM_ACC_READ | SIM_ACC_WRITE, heartbeat_rate_ms),
    EP(193, "axis0.motor.error",                            SIM_T_U64, SIM_ACC_READ | SIM_ACC_WRITE, error_motor),
    EP(207, "axis0.motor.fet_thermistor.temperature",       SIM_T_F32, SIM_ACC_READ,                 fet_temp),
    EP(211, "axis0.motor.motor_thermistor.temperature",     SIM_T_F32, SIM_ACC_READ,                 motor_temp),
    EP(232, "axis0.motor.current_control.Iq_measured",      SIM_T_F32, SIM_ACC_READ,                 iq_measured),
    EP(242, "axis0.motor.config.gear_ratio",                SIM_T_F32, SIM_ACC_READ | SIM_ACC_WRITE, gear_ratio),
    EP(247, "axis0.motor.config.torque_constant",           SIM_T_F32, SIM_ACC_READ | SIM_ACC_WRITE, torque_constant),
    EP(249, "axis0.motor.config.current_lim",               SIM_T_F32, SIM_ACC_READ | SIM_ACC_WRITE, current_lim),
    EP(251, "axis0.motor.config.torque_lim",                SIM_T_F32, SIM_ACC_READ | SIM_ACC_WRITE, torque_lim),
    EP(268, "axis0.controller.error",                       SIM_T_U8,  SIM_ACC_READ | SIM_ACC_WRITE, error_controller),
    EP(270, "axis0.controller.input_pos",                   SIM_T_F32, SIM_ACC_READ | SIM_ACC_WRITE, input_pos),
    EP(271, "axis0.controller.input_vel",                   SIM_T_F32, SIM_ACC_READ | SIM_ACC_WRITE, input_vel),
    EP(272, "axis0.controller.input_torque",                SIM_T_F32, SIM_ACC_READ | SIM_ACC_WRITE, input_torque),
    EP(287, "axis0.controller.config.control_mode",         SIM_T_U8,  SIM_ACC_READ | SIM_ACC_WRITE, control_mode),
    EP(288, "axis0.controller.config.input_mode",           SIM_T_U8,  SIM_ACC_READ | SIM_ACC_WRITE, input_mode),
    EP(301, "axis0.controller.config.vel_limit",            SIM_T_F32, SIM_ACC_READ | SIM_ACC_WRITE, vel_limit),
    EP(335, "axis0.controller.config.mit_max_pos",          SIM_T_F32, SIM_ACC_READ | SIM_ACC_WRITE, mit_max_pos),
    EP(336, "axis0.controller.config.mit_max_vel",          SIM_T_F32, SIM_ACC_READ | SIM_ACC_WRITE, mit_max_vel),
    EP(337, "axis0.controller.config.mit_max_torque",       SIM_T_F32, SIM_ACC_READ | SIM_ACC_WRITE, mit_max_torque),
    EP(338, "axis0.controller.config.mit_max_kp",           SIM_T_F32, SIM_ACC_READ | SIM_ACC_WRITE, mit_max_kp),
    EP(339, "axis0.controller.config.mit_max_kd",           SIM_T_F32, SIM_ACC_READ | SIM_ACC_WRITE, mit_max_kd),
    EP(365, "axis0.encoder.error",                          SIM_T_U16, SIM_ACC_READ | SIM_ACC_WRITE, error_encoder),
    EP(372, "axis0.encoder.pos_estimate",                   SIM_T_F32, SIM_ACC_READ,                 pos_estimate),
    EP(378, "axis0.encoder.vel_estimate",                   SIM_T_F32, SIM_ACC_READ,                 vel_estimate),
    EP(390, "axis0.encoder.config.cpr",                     SIM_T_I32, SIM_ACC_READ | SIM_ACC_WRITE, cpr),
};
#undef EP

#define K_EP_COUNT (sizeof k_eps / sizeof k_eps[0])

const sim_ep_def_t *sim_default_endpoints(size_t *count_out)
{
    if (count_out) *count_out = K_EP_COUNT;
    return k_eps;
}

uint8_t sim_ep_width(uint8_t type)
{
    switch (type) {
    case SIM_T_U8: case SIM_T_I8: case SIM_T_BOOL: return 1u;
    case SIM_T_U16: case SIM_T_I16:               return 2u;
    case SIM_T_U32: case SIM_T_I32: case SIM_T_F32: return 4u;
    case SIM_T_U64: case SIM_T_I64: case SIM_T_F64: return 8u;
    default: return 0u;
    }
}

const sim_ep_def_t *sim_find_ep(const sim_node_t *n, uint16_t ep_id)
{
    size_t i;
    (void)n;
    for (i = 0u; i < K_EP_COUNT; ++i) {
        if (k_eps[i].id == ep_id) return &k_eps[i];
    }
    return NULL;
}

/* 端点读写用**参数值字节序 = 小端**，与真固件一致：
   固件的参数通路是把端点值原样 memcpy 进/出 CAN 载荷（`endpoint_handler`\+
   `memcpy(&txmsg.buf[4], &value_buf[offset], n)`），所以线上就是主机序 LE。
   ⚠ 旧的注释说"与固件 float_to_big_endian_bytes 一致"是**错的**：
     那些 BE 辅助函数只用于控制帧/查询响应，参数通路根本不经过它们。
     实测依据见 `cb_frame.h` 的 `cb_le_*` 说明。 */

uint8_t sim_ep_read(const sim_node_t *n, const sim_ep_def_t *def,
                    uint8_t *out, uint8_t cap)
{
    const uint8_t *p;

    if (!n || !def || !out) return 0u;

    {
        uint8_t w = sim_ep_width(def->type);
        if (w == 0u || cap < w) return 0u;
        p = (const uint8_t *)n + def->offset;

        switch (def->type) {
        case SIM_T_U8: case SIM_T_BOOL: out[0] = p[0]; break;
        case SIM_T_I8:   out[0] = p[0]; break;
        case SIM_T_U16:  cb_le_put_u16(out, *(const uint16_t *)p); break;
        case SIM_T_I16:  cb_le_put_i16(out, *(const int16_t *)p); break;
        case SIM_T_U32:  cb_le_put_u32(out, *(const uint32_t *)p); break;
        case SIM_T_I32:  cb_le_put_i32(out, *(const int32_t *)p); break;
        case SIM_T_U64:  cb_le_put_u64(out, *(const uint64_t *)p); break;
        case SIM_T_I64:  cb_le_put_u64(out, (uint64_t)(*(const int64_t *)p)); break;
        case SIM_T_F32:  cb_le_put_f32(out, *(const float *)p); break;
        /* ⚠ F64 暂时无端点使用；保留分支以免 sim_ep_width 与读写不一致 */
        case SIM_T_F64:  return 0u;
        default:         return 0u;
        }
        return w;
    }
}

int sim_ep_write(sim_node_t *n, const sim_ep_def_t *def,
                 const uint8_t *in, uint8_t len)
{
    uint8_t *p;
    uint8_t w;

    if (!n || !def || !in) return -1;
    if (!(def->access & SIM_ACC_WRITE)) return -1;

    w = sim_ep_width(def->type);
    if (w == 0u || len != w) return -1;

    p = (uint8_t *)n + def->offset;
    switch (def->type) {
    case SIM_T_U8: case SIM_T_BOOL: p[0] = in[0]; break;
    case SIM_T_I8:   p[0] = in[0]; break;
    case SIM_T_U16:  *(uint16_t *)p = cb_le_get_u16(in); break;
    case SIM_T_I16:  *(int16_t *)p  = cb_le_get_i16(in); break;
    case SIM_T_U32:  *(uint32_t *)p = cb_le_get_u32(in); break;
    case SIM_T_I32:  *(int32_t *)p  = cb_le_get_i32(in); break;
    case SIM_T_U64:  *(uint64_t *)p = cb_le_get_u64(in); break;
    case SIM_T_I64:  *(int64_t *)p  = (int64_t)cb_le_get_u64(in); break;
    case SIM_T_F32:  *(float *)p    = cb_le_get_f32(in); break;
    default:         return -1;      /* F64 未使用，与 sim_ep_width 保持一致 */
    }
    return 0;
}

/* ==========================================================================
 * 出站队列
 * ======================================================================== */

static void sim_emit(sim_bus_t *b, uint32_t id, const uint8_t *payload,
                     uint8_t len, int is_fd)
{
    uint32_t next;
    jsdk_can_frame_t *f;

    if (b->force_txq_full > 0u) {
        b->force_txq_full--;
        b->txq_dropped++;
        return;                       /* 模拟总线/队列满 */
    }

    next = (b->txq_tail + 1u) % SIM_TX_QUEUE;
    if (next == b->txq_head) {
        b->txq_dropped++;
        return;
    }

    f = &b->txq[b->txq_tail];
    memset(f, 0, sizeof *f);
    f->id    = id;
    f->len   = len;
    f->flags = (uint8_t)(JSDK_FRAME_EXT | (is_fd ? (JSDK_FRAME_FD | JSDK_FRAME_BRS) : 0u));
    if (payload && len) memcpy(f->data, payload, len);

    b->txq_tail = next;
    b->tx_frames++;
}

int sim_tx_pop(sim_bus_t *b, jsdk_can_frame_t *f)
{
    if (!b || !f) return 0;
    if (b->txq_head == b->txq_tail) return 0;
    *f = b->txq[b->txq_head];
    b->txq_head = (b->txq_head + 1u) % SIM_TX_QUEUE;
    return 1;
}

/* ==========================================================================
 * 应答构造
 * ======================================================================== */

/** 设备 → 主站的状态上报（0x40 与各控制帧的应答都用它）*/
static void send_mit_response(sim_bus_t *b, sim_node_t *n, uint8_t master_id)
{
    cb_mit_range_t r;
    cb_mit_response_t resp;
    float max_current;
    uint8_t payload[8];
    uint8_t txseq;

    r.pos_max = n->mit_max_pos; r.vel_max = n->mit_max_vel;
    r.kp_max  = n->mit_max_kp;  r.kd_max  = n->mit_max_kd;
    r.tau_max = n->mit_max_torque;

    /* 固件把**输出端** rad / rad/s 发出（× 2π / gear_ratio） */
    resp.pos      = n->pos_estimate * (2.0f * SIM_PI) / n->gear_ratio;
    resp.vel      = n->vel_estimate * (2.0f * SIM_PI) / n->gear_ratio;
    resp.current  = n->iq_measured;
    resp.err_code = (uint8_t)detect_error_code(n);
    resp.mode     = (uint8_t)detect_mode_state(n);
    resp.motor_temp_c = (int16_t)n->motor_temp;
    resp.mos_temp_c   = (int16_t)n->fet_temp;

    max_current = cb_mit_response_max_current(n->mit_max_torque, n->torque_constant);
    cb_mit_pack_response(payload, &r, &resp, max_current);

    /* ⚠ 复刻固件：响应的 Seq 用的是**设备本地滚动计数器**，
       不是请求的 Seq（固件 `send_mit_response` 的 seq 参数是死参数）。
       因此上位机**不能**用 Seq 关联请求与响应，只能靠
       `(source, dest, msgtype)`。 */
    txseq = n->tx_seq;
    n->tx_seq = (uint8_t)((n->tx_seq + 1u) & 0x03u);

    sim_emit(b, cb_make_id(CB_PRI_HIGH_CTRL, CB_MSG_MIT_CONTROL,
                           master_id, (uint8_t)n->node_id, txseq),
             payload, 8u, (int)n->is_fd);
}

static uint8_t detect_error_code(const sim_node_t *n)
{
    if (n->error_axis == 0u && n->error_motor == 0u && n->error_encoder == 0u
        && n->error_controller == 0u && n->error_board == 0u) {
        return CB_ERR_NONE;
    }
    /* 与固件同样：先看轴级/CAN 超时，再按子系统 */
    if (n->error_axis & SIM_ERR_CAN_BUS_FAILED) return CB_ERR_CAN_TIMEOUT;
    if (n->error_motor != 0u)    return CB_ERR_MOTOR;
    if (n->error_encoder != 0u)  return CB_ERR_ENCODER;
    if (n->error_controller != 0u) return CB_ERR_CONTROLLER;
    return CB_ERR_MULTIPLE;
}

static uint8_t detect_mode_state(const sim_node_t *n)
{
    if (!n->armed) return CB_MODE_IDLE;
    if (n->input_mode == SIM_INPUT_MODE_MIT) return CB_MODE_MIT;
    switch (n->control_mode) {
    case SIM_CM_VELOCITY: return CB_MODE_VELOCITY;
    case SIM_CM_TORQUE:   return CB_MODE_TORQUE;
    default:              return CB_MODE_POSITION;
    }
}

/** 0x41..0x44 / 0x47 的响应：同一 MsgType、seq = 0 */
static void send_query_f32x2(sim_bus_t *b, sim_node_t *n, uint8_t msgtype,
                             uint8_t master_id, float a, float c)
{
    uint8_t payload[8];
    cb_be_put_f32(payload, a);
    cb_be_put_f32(payload + 4, c);
    sim_emit(b, cb_make_id(CB_PRI_QUERY, msgtype, master_id,
                           (uint8_t)n->node_id, 0u),
             payload, 8u, (int)n->is_fd);
}

static void send_heartbeat(sim_bus_t *b, sim_node_t *n, uint32_t master_id)
{
    cb_heartbeat_t hb;
    uint8_t payload[CB_HB_LEN_FD];

    memset(&hb, 0, sizeof hb);
    hb.life      = n->life;
    n->life      = (uint8_t)((n->life + 1u) & 0x07u);

    hb.err_flags = 0u;
    if (n->error_axis != 0u)       hb.err_flags |= CB_HB_ERR_AXIS;
    if (n->error_motor != 0u)      hb.err_flags |= CB_HB_ERR_MOTOR;
    if (n->error_encoder != 0u)    hb.err_flags |= CB_HB_ERR_ENCODER;
    if (n->error_controller != 0u) hb.err_flags |= CB_HB_ERR_CONTROLLER;
    if (n->error_board != 0u)      hb.err_flags |= CB_HB_ERR_BOARD;

    hb.state        = (uint8_t)(n->armed ? SIM_AS_CLOSED_LOOP : SIM_AS_IDLE);
    hb.control_mode = n->control_mode;
    hb.motor_temp_c = (int16_t)n->motor_temp;
    hb.mos_temp_c   = (int16_t)n->fet_temp;
    hb.vbus_v       = n->vbus_voltage;
    hb.ibus_a       = n->ibus;
    hb.pos_turns    = n->pos_estimate;
    hb.vel_turns_per_s = n->vel_estimate;
    hb.iq_a         = n->iq_measured;
    /* ⚠ 必须置位：FD 编码器靠它决定 MOS 温度是否有效，
       忘了置位会把 MOS 温度静默写成 0（本模型确实撑过这个 bug） */
    hb.have_mos_temp = 1;
    hb.have_vbus     = 1;

    /* 与固件一致：`state << 4` 会截断 16 —— 本模型只产生 1/8，无此问题 */
    if (n->is_fd) {
        uint8_t len = (uint8_t)cb_heartbeat_encode_fd(payload, &hb);
        sim_emit(b, cb_make_id(CB_PRI_STATUS, CB_MSG_HEARTBEAT,
                               (uint8_t)master_id, (uint8_t)n->node_id, 0u),
                 payload, len, 1);
    } else {
        uint8_t len = (uint8_t)cb_heartbeat_encode_classic(payload, &hb);
        sim_emit(b, cb_make_id(CB_PRI_STATUS, CB_MSG_HEARTBEAT,
                               (uint8_t)master_id, (uint8_t)n->node_id, 0u),
                 payload, len, 0);
        (void)len;
    }
    n->last_heartbeat_ms = b->now_ms;
}

/* ==========================================================================
 * 控制帧
 * ======================================================================== */

static void handle_mit(sim_bus_t *b, sim_node_t *n, const jsdk_can_frame_t *f,
                       int is_bcast)
{
    cb_mit_range_t r;
    float pos = 0.0f, vel = 0.0f, kp = 0.0f, kd = 0.0f, tau = 0.0f;
    uint8_t slot;
    uint8_t master_id = (uint8_t)cb_id_source(f->id);

    r.pos_max = n->mit_max_pos; r.vel_max = n->mit_max_vel;
    r.kp_max  = n->mit_max_kp;  r.kd_max  = n->mit_max_kd;
    r.tau_max = n->mit_max_torque;

    if (is_bcast) {
        if (n->node_id >= CB_MAX_BROADCAST_DEVICES) return;
        if (!n->is_fd) {
            /* Classic：只有槽位 0，被掩码命中的所有设备执行**同一条**命令 */
            if (f->len < 8u) { b->bad_len_drops++; return; }
            slot = 0u;
        } else {
            slot = (uint8_t)n->node_id;
            if (f->len < (uint8_t)((slot + 1u) * CB_MIT_SLOT_BYTES)) {
                b->bad_len_drops++;
                return;
            }
        }
        n->bcast_seen = 1u;
    } else {
        slot = 0u;
        if (f->len < 8u) { b->bad_len_drops++; return; }
    }

    cb_mit_unpack_command(&f->data[(size_t)slot * CB_MIT_SLOT_BYTES], &r,
                          &pos, &vel, &kp, &kd, &tau);

    /* 单位换算（与固件逐式对应） */
    n->cmd_pos_out_rad    = pos;
    n->cmd_vel_out_rad_s  = vel;
    n->cmd_kp             = kp;
    n->cmd_kd             = kd;
    n->cmd_tau_out_nm     = tau;
    n->cmd_torque_motor_nm = tau / n->gear_ratio;
    n->pos_target_motor   = pos * n->gear_ratio / (2.0f * SIM_PI);
    n->vel_target_motor   = vel * n->gear_ratio / (2.0f * SIM_PI);

    n->control_mode = SIM_CM_POSITION;
    n->input_mode   = SIM_INPUT_MODE_MIT;
    n->input_pos    = n->pos_target_motor;
    n->input_vel    = n->vel_target_motor;
    n->input_torque = n->cmd_torque_motor_nm;
    n->last_control_msgtype = CB_MSG_MIT_CONTROL;
    n->cmd_count++;

    if (master_id != 0u && !is_bcast) {
        send_mit_response(b, n, master_id);
    }
}

/* ==========================================================================
 * 参数访问
 * ======================================================================== */

static void handle_param_read(sim_bus_t *b, sim_node_t *n,
                              const jsdk_can_frame_t *f, int classic)
{
    uint8_t master_id = (uint8_t)cb_id_source(f->id);
    /* ⚠ 批量响应最大可达 64 B（FD 上限），不能按单读的 12 B 开缓冲 */
    uint8_t rsp[CB_PARAM_FD_FRAME_MAX];
    size_t  n_rsp;

    if (f->len < 4u) { b->bad_len_drops++; return; }

    if (f->data[0] & CB_PARAM_FLAG_BATCH) {
        /* ---- 批量读（仅 FD） ---- */
        uint16_t eps[CB_PARAM_MAX_BATCH];
        cb_param_batch_req_t rq;
        uint8_t  vals[CB_PARAM_MAX_BATCH * CB_PARAM_MAX_VALUE];
        uint8_t  valid[4] = { 0u, 0u, 0u, 0u };
        uint32_t data_len = 0u;
        uint32_t budget;
        int fits = 1;
        uint8_t i;

        if (classic) {
            /* 固件行为：Classic 收到批量请求 → 回 2 字节 ERR（Count = 0） */
            cb_param_pack_batch_err(rsp, sizeof rsp);
            n_rsp = 2u;
            n->param_err_count++;
            sim_emit(b, cb_make_id(CB_PRI_CONFIG, CB_MSG_PARAM_READ, master_id,
                                   (uint8_t)n->node_id, 0u),
                     rsp, (uint8_t)n_rsp, 0);
            return;
        }

        if (cb_param_unpack_batch_req(f->data, f->len, eps,
                                      (uint8_t)CB_PARAM_MAX_BATCH, &rq) != 0) {
            return;
        }

        budget = 64u - 2u - cb_param_bitmap_bytes(rq.count);

        for (i = 0u; i < rq.count; ++i) {
            const sim_ep_def_t *d = sim_find_ep(n, eps[i]);
            uint8_t tmp[CB_PARAM_MAX_VALUE];
            uint8_t w = 0u;

            if (d) w = sim_ep_read(n, d, tmp, sizeof tmp);
            if ((uint32_t)w > budget - data_len) { fits = 0; break; }
            if (w > 0u) {
                valid[i / 8u] = (uint8_t)(valid[i / 8u] | (uint8_t)(1u << (i % 8u)));
                memcpy(&vals[data_len], tmp, w);
                data_len += w;
            }
        }

        if (!fits) {
            cb_param_pack_batch_err(rsp, sizeof rsp);
            n->param_err_count++;
            sim_emit(b, cb_make_id(CB_PRI_CONFIG, CB_MSG_PARAM_READ, master_id,
                                   (uint8_t)n->node_id, 0u),
                     rsp, 2u, 1);
            return;
        }

        rsp[0] = CB_PARAM_FLAG_BATCH;
        rsp[1] = rq.count;
        memcpy(&rsp[2], valid, cb_param_bitmap_bytes(rq.count));
        memcpy(&rsp[2 + cb_param_bitmap_bytes(rq.count)], vals, data_len);
        sim_emit(b, cb_make_id(CB_PRI_CONFIG, CB_MSG_PARAM_READ, master_id,
                               (uint8_t)n->node_id, 0u),
                 rsp, (uint8_t)(2u + cb_param_bitmap_bytes(rq.count) + data_len), 1);
        return;
    }

    /* ---- 单读 ---- */
    {
        cb_param_read_req_t rq;
        const sim_ep_def_t *d;
        uint8_t full[CB_PARAM_MAX_VALUE];
        uint8_t full_len = 0u;

        if (cb_param_unpack_read_req(f->data, f->len, classic, &rq) != 0) return;

        d = sim_find_ep(n, rq.ep_id);
        if (d) full_len = sim_ep_read(n, d, full, sizeof full);

        n_rsp = cb_param_build_read_rsp(rsp, sizeof rsp, f->data[0], rq.ep_id,
                                        full, full_len, rq.req_len, rq.offset);
        if (n_rsp == 0u) return;

        sim_emit(b, cb_make_id(CB_PRI_CONFIG, CB_MSG_PARAM_READ, master_id,
                               (uint8_t)n->node_id, 0u),
                 rsp, (uint8_t)n_rsp, (int)n->is_fd);
    }
}


/**
 * 写 `axis0.requested_state` 的状态机跳转（复刻固件 `Axis::requested_state_` 的语义）。
 *
 * | 写入 | 瞬时状态（current_state） | 结束后回到 |
 * |---|---|---|
 * | 1 IDLE | — | IDLE（并解除使能） |
 * | 3 FULL_CALIBRATION_SEQUENCE | 3，持续 `SIM_TRANSIENT_MS` | IDLE |
 * | 8 CLOSED_LOOP_CONTROL | — | CLOSED_LOOP（并使能） |
 * | 11 HOMING | 11，持续 `SIM_TRANSIENT_MS` | CLOSED_LOOP |
 * | 其它（4/6/7/9/10/12…） | 原值，持续 `SIM_TRANSIENT_MS` | IDLE |
 *
 * 关键点：**瞬时期间 `current_state` != `requested_state`** —— 上位机就是靠这个
 * 差异判断“标定还在跑”。
 */
static void apply_requested_state(sim_bus_t *b, sim_node_t *n, uint8_t want)
{
    n->requested_state = want;
    n->requested_hits++;

    switch (want) {
    case SIM_AS_IDLE:
        n->armed = 0u;
        n->current_state = SIM_AS_IDLE;
        n->transient_until_ms = 0u;
        break;

    case SIM_AS_CLOSED_LOOP:
        n->armed = 1u;
        n->estop = 0u;
        n->current_state = SIM_AS_CLOSED_LOOP;
        n->transient_until_ms = 0u;
        break;

    case SIM_AS_FULL_CALIB:
        n->armed = 0u;
        n->current_state = SIM_AS_FULL_CALIB;
        n->transient_until_ms = b->now_ms + SIM_TRANSIENT_MS;
        n->settle_to = SIM_AS_IDLE;
        break;

    case SIM_AS_HOMING:
        n->armed = 1u;
        n->current_state = SIM_AS_HOMING;
        n->transient_until_ms = b->now_ms + SIM_TRANSIENT_MS;
        n->settle_to = SIM_AS_CLOSED_LOOP;
        break;

    default:
        n->current_state = want;
        n->transient_until_ms = b->now_ms + SIM_TRANSIENT_MS;
        n->settle_to = SIM_AS_IDLE;
        break;
    }
}

static void handle_param_write(sim_bus_t *b, sim_node_t *n,
                               const jsdk_can_frame_t *f, int classic)
{
    uint8_t master_id = (uint8_t)cb_id_source(f->id);
    uint8_t flags;
    uint16_t ep;
    uint8_t vlen = 0u;
    uint8_t value[CB_PARAM_MAX_VALUE];
    size_t  slot = (size_t)(n - b->nodes);

    if (f->len < 4u) { b->bad_len_drops++; return; }

    /* Classic 且声明写 >4 字节 → 分段装配 */
    if (classic && f->data[3] > 4u) {
        uint8_t got[CB_PARAM_MAX_VALUE];
        uint8_t got_len = 0u;
        int rc;

        if (f->len < CB_PARAM_CLASSIC_FRAME) { b->bad_len_drops++; return; }

        rc = cb_param_write_asm_feed(&b->asm_state[slot], master_id,
                                     cb_be_get_u16(f->data + 1), f->data[3],
                                     f->data[0], f->data + 4, got, &got_len);
        if (rc != 0) return;          /* 1 = 等后续块；-1 = 中止，不写入 */

        {
            const sim_ep_def_t *d = sim_find_ep(n, cb_be_get_u16(f->data + 1));
            if (d) (void)sim_ep_write(n, d, got, got_len);
        }
        {
            uint8_t ack[CB_PARAM_ACK_LEN];
            cb_param_pack_write_ack(ack, sizeof ack, f->data[0],
                                    cb_be_get_u16(f->data + 1));
            sim_emit(b, cb_make_id(CB_PRI_CONFIG, CB_MSG_PARAM_WRITE, master_id,
                                   (uint8_t)n->node_id, 0u),
                     ack, (uint8_t)CB_PARAM_ACK_LEN, (int)n->is_fd);
        }
        return;
    }

    if (cb_param_unpack_write_req(f->data, f->len, &flags, &ep, value, &vlen) != 0) {
        return;
    }

    {
        const sim_ep_def_t *d = sim_find_ep(n, ep);
        if (d) (void)sim_ep_write(n, d, value, vlen);
    }

    /* ⚠ 固件写 `axis0.requested_state` 会**触发状态机跳转**，不是单纯存一个字节。
       很多标定/回零流程就靠“写 requested_state → 等 current_state 变化”判定完成，
       因此模型必须复刻这一点，否则 SDK 的 calibrate/home 在仿真上前不通。 */
    if (ep == 143u && vlen >= 1u) {
        apply_requested_state(b, n, value[0]);
    }

    /* 写确认：Flags 原样返回、DataLen = 0（静默确认） */
    {
        uint8_t ack[CB_PARAM_ACK_LEN];
        cb_param_pack_write_ack(ack, sizeof ack, flags, ep);
        sim_emit(b, cb_make_id(CB_PRI_CONFIG, CB_MSG_PARAM_WRITE, master_id,
                               (uint8_t)n->node_id, 0u),
                 ack, (uint8_t)CB_PARAM_ACK_LEN, (int)n->is_fd);
    }
}

/* ==========================================================================
 * JSON 描述符
 * ======================================================================== */

static void handle_desc_read(sim_bus_t *b, sim_node_t *n,
                            const jsdk_can_frame_t *f)
{
    if (f->len < 4u) { b->bad_len_drops++; return; }

    b->desc.active        = 1;
    b->desc.metadata_sent = 0;
    b->desc.offset        = (uint32_t)f->data[0]
                          | ((uint32_t)f->data[1] << 8)
                          | ((uint32_t)f->data[2] << 16)
                          | ((uint32_t)f->data[3] << 24);
    b->desc.master_id     = (uint32_t)cb_id_source(f->id);
    b->desc.my_id         = n->node_id;
    b->desc.is_classic    = !n->is_fd;
    /* 后续帧由 sim_tick 按每毫秒预算推送 */
}

/** 推一批描述符帧；@return 1 = 本次把队列塞满或传完了 */
static void desc_pump(sim_bus_t *b, uint32_t budget_in)
{
    uint8_t frame_len;
    uint8_t per_frame;
    uint32_t budget = budget_in;

    if (!b->desc.active || !b->desc.json) return;

    frame_len = (uint8_t)(b->desc.is_classic ? CB_PARAM_CLASSIC_FRAME : CB_PARAM_FD_FRAME_MAX);
    per_frame = (uint8_t)(frame_len - 2u);

    if (!b->desc.metadata_sent) {
        uint8_t meta[CB_PARAM_FD_FRAME_MAX];
        memset(meta, 0, sizeof meta);
        meta[0] = 0u;  meta[1] = 0u;
        meta[2] = (uint8_t)(b->desc.len);
        meta[3] = (uint8_t)(b->desc.len >> 8);
        meta[4] = (uint8_t)(b->desc.len >> 16);
        meta[5] = (uint8_t)(b->desc.len >> 24);
        meta[6] = (uint8_t)(b->desc.crc);
        meta[7] = (uint8_t)(b->desc.crc >> 8);

        sim_emit(b, cb_make_id(CB_PRI_CONFIG, CB_MSG_JSON_DESC_DATA,
                               (uint8_t)b->desc.master_id,
                               (uint8_t)b->desc.my_id, 0u),
                 meta, frame_len, b->desc.is_classic ? 0 : 1);
        b->desc.metadata_sent = 1;
        if (budget == 0u) return;
        budget--;
    }

    while (b->desc.offset < b->desc.len && budget > 0u) {
        uint8_t chunk[CB_PARAM_FD_FRAME_MAX];
        uint32_t remaining = b->desc.len - b->desc.offset;
        uint8_t n_copy = (remaining < per_frame) ? (uint8_t)remaining : per_frame;
        uint16_t off = (uint16_t)b->desc.offset;

        memset(chunk, 0, sizeof chunk);
        chunk[0] = (uint8_t)(off);
        chunk[1] = (uint8_t)(off >> 8);
        memcpy(&chunk[2], &b->desc.json[b->desc.offset], n_copy);

        sim_emit(b, cb_make_id(CB_PRI_CONFIG, CB_MSG_JSON_DESC_DATA,
                               (uint8_t)b->desc.master_id,
                               (uint8_t)b->desc.my_id, 0u),
                 chunk, frame_len, b->desc.is_classic ? 0 : 1);

        b->desc.offset += n_copy;
        budget--;
    }

    if (b->desc.offset >= b->desc.len) {
        b->desc.active        = 0;
        b->desc.metadata_sent = 0;
        b->desc.offset        = 0u;
    }
}

/* ==========================================================================
 * 内建描述符：由端点表**生成**（不是手写常量）
 * ==========================================================================
 *
 * @par 为什么需要
 *  仿真设备原本**不自带** JSON：测试自己 `load_fixture()` 再 `sim_set_desc()`
 *  塞进去。这对测试没问题，但 `jsdk-cli --if virtual` 会直接卡在
 *  "descriptor download timed out (0/0 bytes)" —— 设备根本没东西可发。
 *  对一个**诊断工具**来说，"现场没有硬件时也能起来自检/演示"是第一位的。
 *
 * @par 为什么是"生成"而不是"手写一段 JSON 常量"
 *  设备**行为**依据 `k_eps`（端点 id → 字段偏移/类型/权限），而 SDK 的
 *  端点**元数据**来自 JSON。如果手写一份 JSON，就有两份清单要同步维护，
 *  漂移的第一个症状会是"读到的值类型不对"——极难查。
 *  这里让 JSON 从 `k_eps` 直接生成，**只有一个数据源**。
 *
 * @par 为什么是扁平结构
 *  描述符里路径靠容器名逐级拼接。把整条路径直接写成一个 `name` 是等价的
 *  （解析器拼出来的仍然是 `axis0.motor.config.gear_ratio`），
 *  这样就不必为树形结构写递归拼装代码。
 */

/** sim_val_type_t → 描述符 type 字符串。 */
static const char *sim_type_name(uint8_t t)
{
    switch (t) {
    case SIM_T_U8:   return "uint8";
    case SIM_T_I8:   return "int8";
    case SIM_T_U16:  return "uint16";
    case SIM_T_I16:  return "int16";
    case SIM_T_U32:  return "uint32";
    case SIM_T_I32:  return "int32";
    case SIM_T_U64:  return "uint64";
    case SIM_T_I64:  return "int64";
    case SIM_T_F32:  return "float";
    case SIM_T_F64:  return "double";
    case SIM_T_BOOL: return "bool";
    default:         return NULL;
    }
}

/**
 * 生成内建描述符（calloc 一块刚好够用的缓冲）。
 * @return 缓冲区（调用方用 sim_set_desc 之后即可 free）；失败返回 NULL
 */
static char *sim_build_builtin_desc(size_t *len_out)
{
    size_t need = 2u;                     /* '[' + ']' */
    size_t i;
    char  *buf;
    size_t off = 0u;

    /* 先算长度：每个端点一个对象，字段名固定 */
    for (i = 0u; i < K_EP_COUNT; ++i) {
        need += 64u + 2u * strlen(k_eps[i].path) + 24u + 1u;
    }

    buf = (char *)malloc(need);
    if (!buf) return NULL;

    buf[off++] = '[';
    for (i = 0u; i < K_EP_COUNT; ++i) {
        const char *tn = sim_type_name((uint8_t)k_eps[i].type);
        int n;
        if (!tn) continue;                /* 未知类型：跳过而不是发坏描述符 */

        n = snprintf(&buf[off], need - off,
                     "%s{\"name\":\"%s\",\"id\":%u,\"type\":\"%s\","
                     "\"access\":\"%s%s\"}",
                     (off > 1u) ? "," : "",
                     k_eps[i].path,
                     (unsigned)k_eps[i].id,
                     tn,
                     (k_eps[i].access & SIM_ACC_READ) ? "r" : "",
                     (k_eps[i].access & SIM_ACC_WRITE) ? "w" : "");
        if (n < 0 || (size_t)n >= need - off) { free(buf); return NULL; }
        off += (size_t)n;
    }
    buf[off++] = ']';

    *len_out = off;
    return buf;
}

/**
 * 装入内建描述符（幂等：已有描述符就不动）。
 * @return 0 = 成功（或已有）；-1 = 分配失败
 */
static int sim_load_builtin_desc(sim_bus_t *b)
{
    char  *js;
    size_t len = 0u;
    int    rc;

    if (!b || b->desc.json) return 0;

    js = sim_build_builtin_desc(&len);
    if (!js) return -1;

    rc = sim_set_desc(b, js, (uint32_t)len, 0u);   /* crc=0 → 用字节和占位 */
    free(js);
    return rc;
}

int sim_set_desc(sim_bus_t *b, const char *json, uint32_t len, uint16_t crc)
{
    if (!b || !json || len == 0u) return -1;

    if (b->desc.owned && b->desc.json) {
        free(b->desc.json);
        b->desc.json  = NULL;
        b->desc.owned = 0;
    }

    b->desc.json = (uint8_t *)malloc(len);
    if (!b->desc.json) return -1;
    memcpy(b->desc.json, json, len);
    b->desc.len   = len;
    b->desc.owned = 1;

    if (crc == 0u) {                        /* 简单字节和占位（非固件 CRC） */
        uint32_t i; uint16_t s = 0u;
        for (i = 0u; i < len; ++i) s = (uint16_t)(s + b->desc.json[i]);
        b->desc.crc = s;
    } else {
        b->desc.crc = crc;
    }
    return 0;
}

int sim_set_desc_file(sim_bus_t *b, const char *path, uint16_t crc)
{
    FILE *fh;
    long  sz;
    char *buf;
    int   rc;

    if (!b || !path) return -1;

    fh = fopen(path, "rb");
    if (!fh) return -1;
    if (fseek(fh, 0L, SEEK_END) != 0) { fclose(fh); return -1; }
    sz = ftell(fh);
    if (sz <= 0) { fclose(fh); return -1; }
    rewind(fh);

    buf = (char *)malloc((size_t)sz);
    if (!buf) { fclose(fh); return -1; }
    if (fread(buf, 1u, (size_t)sz, fh) != (size_t)sz) {
        free(buf); fclose(fh); return -1;
    }
    fclose(fh);

    rc = sim_set_desc(b, buf, (uint32_t)sz, crc);
    free(buf);
    return rc;
}

int sim_desc_active(const sim_bus_t *b)
{
    return (b && b->desc.active) ? 1 : 0;
}

/* ==========================================================================
 * 帧分发
 * ======================================================================== */

static void handle_frame_for_node(sim_bus_t *b, sim_node_t *n,
                                  const jsdk_can_frame_t *f)
{
    uint8_t msgtype = (uint8_t)cb_id_msgtype(f->id);
    uint8_t master_id = (uint8_t)cb_id_source(f->id);
    int is_bcast = cb_id_is_broadcast(f->id);
    int classic = !(f->flags & JSDK_FRAME_FD);
    int is_ctrl;

    n->last_control_msgtype = msgtype;

    /* `is_ctrl`：与固件一致，只含 0x00~0x03 与 0x80~0x83 */
    is_ctrl = (msgtype <= CB_MSG_TORQUE_CONTROL)
           || (msgtype >= CB_MSG_MIT_CONTROL_BCAST
               && msgtype <= CB_MSG_TORQUE_CONTROL_BCAST);
    if (is_ctrl) n->last_cmd_ms = b->now_ms;

    /* ESTOP 优先级最高：任何节点都会停机 */
    if (msgtype == CB_MSG_ESTOP) {
        n->armed = 0u;
        n->estop = 1u;
        n->error_axis |= SIM_ERR_ESTOP_REQUESTED;
        n->iq_measured = 0.0f;
        n->vel_estimate = 0.0f;
        return;
    }
    if (msgtype == CB_MSG_FAULT_ALERT) {
        n->armed = 0u;
        n->error_axis |= SIM_ERR_CAN_BUS_FAILED;
        return;
    }

    switch (msgtype) {
    /* ---- 实时控制 ---- */
    case CB_MSG_MIT_CONTROL:
    case CB_MSG_MIT_CONTROL_BCAST:
        handle_mit(b, n, f, is_bcast);
        break;

    case CB_MSG_POS_CONTROL:
    case CB_MSG_POS_CONTROL_BCAST: {
        float pos_deg = 0.0f, vel_rpm = 0.0f, cur_a = 0.0f;
        if (f->len < (uint8_t)(classic ? CB_CTRL_POS_LEN_CLASSIC : CB_CTRL_POS_LEN_FD)) {
            b->bad_len_drops++;
            break;
        }
        if (cb_ctrl_pos_unpack(f->data, f->len, classic, &pos_deg, &vel_rpm, &cur_a) != 0) {
            break;
        }
        n->pos_target_motor  = pos_deg / 360.0f * n->gear_ratio;
        n->vel_target_motor  = vel_rpm / 60.0f * n->gear_ratio;
        n->torque_lim        = cur_a * n->torque_constant;   /* ⚠ 固件持久改写 */
        n->vel_limit         = n->vel_target_motor;
        n->control_mode      = SIM_CM_POSITION;
        n->input_mode        = 3u;                            /* POS_FILTER */
        n->input_pos         = n->pos_target_motor;
        n->cmd_pos_out_rad   = pos_deg * SIM_PI / 180.0f;
        n->cmd_vel_out_rad_s = vel_rpm * 2.0f * SIM_PI / 60.0f;
        n->cmd_count++;
        if (master_id != 0u && !is_bcast) {
            send_mit_response(b, n, master_id);
        }
        break;
    }

    case CB_MSG_VEL_CONTROL:
    case CB_MSG_VEL_CONTROL_BCAST: {
        float vel_rpm = 0.0f, cur_a = 0.0f;
        if (f->len < CB_CTRL_VEL_LEN) { b->bad_len_drops++; break; }
        if (cb_ctrl_vel_unpack(f->data, f->len, &vel_rpm, &cur_a) != 0) break;
        n->vel_target_motor  = vel_rpm / 60.0f * n->gear_ratio;
        n->torque_lim        = cur_a * n->torque_constant;
        n->control_mode      = SIM_CM_VELOCITY;
        n->input_mode        = 2u;                            /* VEL_RAMP */
        n->input_vel         = n->vel_target_motor;
        n->cmd_vel_out_rad_s = vel_rpm * 2.0f * SIM_PI / 60.0f;
        n->cmd_count++;
        if (master_id != 0u && !is_bcast) {
            send_mit_response(b, n, master_id);
        }
        break;
    }

    case CB_MSG_TORQUE_CONTROL:
    case CB_MSG_TORQUE_CONTROL_BCAST: {
        float tau = 0.0f;
        if (f->len < CB_CTRL_TORQUE_LEN) { b->bad_len_drops++; break; }
        if (cb_ctrl_torque_unpack(f->data, f->len, &tau) != 0) break;
        /* ⚠ 固件**不做** gear_ratio 换算：值直接当电机端 N·m */
        n->cmd_torque_motor_nm = tau;
        n->input_torque        = tau;
        n->control_mode        = SIM_CM_TORQUE;
        n->input_mode          = 6u;                          /* TORQUE_RAMP */
        n->cmd_count++;
        if (master_id != 0u && !is_bcast) {
            send_mit_response(b, n, master_id);
        }
        break;
    }

    case CB_MSG_CURRENT_CONTROL: {
        float cur_a = 0.0f;
        if (f->len < CB_CTRL_CURRENT_LEN) { b->bad_len_drops++; break; }
        if (cb_ctrl_current_unpack(f->data, f->len, &cur_a) != 0) break;
        n->cmd_torque_motor_nm = cur_a * n->torque_constant;
        n->input_torque        = n->cmd_torque_motor_nm;
        n->control_mode        = SIM_CM_TORQUE;
        n->input_mode          = 6u;
        n->cmd_count++;
        /* 固件不回任何应答 */
        break;
    }

    /* ---- 参数访问 ---- */
    case CB_MSG_PARAM_READ:
        handle_param_read(b, n, f, classic);
        break;
    case CB_MSG_PARAM_WRITE:
        handle_param_write(b, n, f, classic);
        break;

    case CB_MSG_CONFIG_SAVE:
        n->cmd_count++;
        break;

    /* ---- 状态查询 ---- */
    case CB_MSG_QUERY_STATUS:
        if (master_id != 0u) send_mit_response(b, n, master_id);
        break;
    case CB_MSG_QUERY_POS_VEL:
        /* ⚠ 固件**不做**单位换算：直接透传电机端 turns */
        if (master_id != 0u) {
            send_query_f32x2(b, n, msgtype, master_id,
                             n->pos_estimate, n->vel_estimate);
        }
        break;
    case CB_MSG_QUERY_CURRENT:
        if (master_id != 0u) {
            send_query_f32x2(b, n, msgtype, master_id, n->iq_measured, 0.0f);
        }
        break;
    case CB_MSG_QUERY_TEMPERATURE:
        if (master_id != 0u) {
            send_query_f32x2(b, n, msgtype, master_id, n->motor_temp, n->fet_temp);
        }
        break;
    case CB_MSG_QUERY_BUS:
        if (master_id != 0u) {
            send_query_f32x2(b, n, msgtype, master_id, n->vbus_voltage, n->ibus);
        }
        break;
    case CB_MSG_QUERY_POWER:
        if (master_id != 0u) {
            send_query_f32x2(b, n, msgtype, master_id,
                             n->vbus_voltage * n->ibus,
                             n->cmd_torque_motor_nm * n->vel_estimate
                                 * 2.0f * SIM_PI);
        }
        break;
    case CB_MSG_QUERY_ERROR: {
        cb_query_error_t e;
        uint8_t payload[CB_QUERY_ERROR_LEN];
        uint32_t v = 0u;
        if (f->len < 1u || master_id == 0u) break;
        e.err_type = f->data[0];
        switch (e.err_type) {
        case CB_ET_MOTOR:      v = (uint32_t)n->error_motor; break;
        case CB_ET_ENCODER:    v = (uint32_t)n->error_encoder; break;
        case CB_ET_SENSORLESS: v = 0u; break;
        case CB_ET_CONTROLLER: v = (uint32_t)n->error_controller; break;
        case CB_ET_SYSTEM:     v = (uint32_t)n->error_board; break;
        case CB_ET_AXIS:       v = n->error_axis; break;
        default: break;
        }
        e.err_value = v;
        cb_query_encode_error(payload, &e);
        sim_emit(b, cb_make_id(CB_PRI_QUERY, msgtype, master_id,
                               (uint8_t)n->node_id, 0u),
                 payload, (uint8_t)CB_QUERY_ERROR_LEN, (int)n->is_fd);
        break;
    }
    case CB_MSG_QUERY_DEVICE_INFO: {
        cb_query_device_info_t d;
        uint8_t payload[CB_QUERY_DEVLEN_FD];
        if (master_id == 0u) break;
        d.hw_ver = CB_QUERY_VER_PACK(1u, 0u, 3u);
        d.fw_ver = CB_QUERY_VER_PACK(0u, 5u, 6u);
        d.serial = n->serial_number;
        d.has_serial = 1;
        cb_be_put_u32(payload, d.hw_ver);
        cb_be_put_u32(payload + 4, d.fw_ver);
        cb_be_put_u64(payload + 8, d.serial);
        sim_emit(b, cb_make_id(CB_PRI_QUERY, msgtype, master_id,
                               (uint8_t)n->node_id, 0u),
                 payload,
                 (uint8_t)(classic ? CB_QUERY_DEVLEN_CLASSIC : CB_QUERY_DEVLEN_FD),
                 classic ? 0 : 1);
        break;
    }

    /* ---- 系统管理 ---- */
    case CB_MSG_SET_NODE_ID:
        if (f->len >= 1u) n->node_id = f->data[0];
        break;
    case CB_MSG_SET_ZERO:
        n->pos_estimate = 0.0f;
        n->input_pos = 0.0f;
        break;
    case CB_MSG_START_MOTOR:
        n->armed = 1u;
        n->estop = 0u;
        n->requested_state = SIM_AS_CLOSED_LOOP;
        n->current_state   = SIM_AS_CLOSED_LOOP;
        n->last_cmd_ms = b->now_ms;
        break;
    case CB_MSG_STOP_MOTOR:
        n->armed = 0u;
        n->requested_state = SIM_AS_IDLE;
        n->current_state   = SIM_AS_IDLE;
        n->transient_until_ms = 0u;
        n->iq_measured = 0.0f;
        n->vel_estimate = 0.0f;
        break;
    case CB_MSG_RESET_DEVICE:
    case CB_MSG_CONFIG_RESET:
        /* 简化：固件是整机重启/擦除配置；本模型只复位该节点的易失状态。
           （不再在遍历 nodes[] 的过程中重建数组，避免迭代语义混乱） */
        node_defaults(n);
        break;
    case CB_MSG_CLEAR_ERRORS:
        n->error_axis = 0u;
        n->error_motor = 0u;
        n->error_encoder = 0u;
        n->error_controller = 0u;
        n->error_board = 0u;
        break;

    /* ---- JSON 描述符 ---- */
    case CB_MSG_JSON_DESC_READ:
        handle_desc_read(b, n, f);
        break;

    default:
        b->unhandled++;
        break;
    }
}

void sim_rx(sim_bus_t *b, const jsdk_can_frame_t *f)
{
    size_t i;

    if (!b || !f) return;
    b->rx_frames++;

    for (i = 0u; i < b->n_nodes; ++i) {
        sim_node_t *n = &b->nodes[i];
        if (n->node_id == 0u) continue;         /* 节点被禁用 */
        if (!cb_id_is_for_me((uint8_t)n->node_id, f->id)) continue;
        b->rx_for_me++;
        handle_frame_for_node(b, n, f);
    }
}

/* ==========================================================================
 * 时间推进
 * ======================================================================== */

void sim_tick(sim_bus_t *b, uint32_t now_ms)
{
    uint32_t t;
    uint32_t elapsed;

    if (!b) return;
    if (now_ms < b->now_ms) return;             /* 单调 */

    /* 逐毫秒推进，与固件 `service_stack()`（1 ms 周期）一致：
       每毫秒最多发 kMaxJsonFramesPerCycle 帧描述符、最多发一次心跳、
       并对每个节点做一次物理步进与超时检查。
       一次跳很久时截断，避免巨量循环（也避免队列撑爆）。 */
    elapsed = (uint32_t)(now_ms - b->last_tick_ms);
    if (elapsed > SIM_MAX_TICK_MS) {
        b->last_tick_ms = now_ms - SIM_MAX_TICK_MS;
    }

    for (t = b->last_tick_ms + 1u; t <= now_ms; ++t) {
        size_t i;

        b->now_ms = t;

        /* 描述符传输优先于心跳（与固件 service_stack 的次序一致） */
        desc_pump(b, SIM_JSON_FRAMES_PER_CYCLE);

        for (i = 0u; i < b->n_nodes; ++i) {
            sim_node_t *n = &b->nodes[i];
            uint32_t timeout;
            const float dt = 0.001f;            /* 固定 1 ms 步长 */

            if (n->node_id == 0u) continue;

            /* ---- 玩具级一阶物理模型 ---- */
            if (n->armed && !n->estop) {
                float v;
                if (n->control_mode == SIM_CM_POSITION) {
                    float err = n->pos_target_motor - n->pos_estimate;
                    v = n->vel_target_motor + 2.0f * err;
                } else {
                    v = n->vel_target_motor;
                }
                if (v > n->mit_max_vel)  v = n->mit_max_vel;
                if (v < -n->mit_max_vel) v = -n->mit_max_vel;
                n->vel_estimate  = v;
                n->pos_estimate += v * dt;
                n->iq_measured   = n->cmd_torque_motor_nm / n->torque_constant;
                if (n->iq_measured > n->current_lim)  n->iq_measured  = n->current_lim;
                if (n->iq_measured < -n->current_lim) n->iq_measured  = -n->current_lim;
                if (n->vel_limit > 0.0f) {
                    if (n->vel_estimate > n->vel_limit)  n->vel_estimate = n->vel_limit;
                    if (n->vel_estimate < -n->vel_limit) n->vel_estimate = -n->vel_limit;
                }
            } else {
                n->vel_estimate = 0.0f;
                n->iq_measured  = 0.0f;
            }
            n->ibus = n->iq_measured * 0.1f;

            /* ---- 瞬时状态（标定/回零）到期 → 落到 settle_to ---- */
            if (n->transient_until_ms != 0u
                && (int32_t)(t - n->transient_until_ms) >= 0) {
                n->current_state      = n->settle_to;
                n->state_change_ms    = t;      /* 上位机靠它确认“跳转已完成” */
                n->transient_until_ms = 0u;
                if (n->settle_to == SIM_AS_IDLE) n->armed = 0u;
            }

            /* ---- break_timeout：仅对已武装（收到过 is_ctrl 帧）的节点生效 ----
               ⚠ `0` = **超时检测被禁用**（与最新固件 `auto_stop_if_timeout()` 一致：
                  `if (timeout_ms == 0) return;`）。旧模型把 0 当成 100 ms 是错的。 */
            if (n->last_cmd_ms != 0u && n->break_timeout != 0u) {
                timeout = (uint32_t)n->break_timeout;
                if ((uint32_t)(t - n->last_cmd_ms) > timeout) {
                    n->error_axis |= SIM_ERR_CAN_BUS_FAILED;
                    n->armed = 0u;
                    n->iq_measured = 0.0f;
                    n->vel_estimate = 0.0f;
                    n->last_cmd_ms = 0u;        /* 防重复触发 */
                }
            }

            /* ---- 心跳（每毫秒最多一次） ---- */
            if (n->heartbeat_rate_ms != 0u
                && (uint32_t)(t - n->last_heartbeat_ms) >= n->heartbeat_rate_ms) {
                send_heartbeat(b, n, 1u);
            }
        }
    }

    b->now_ms       = now_ms;
    b->last_tick_ms = now_ms;
}

/* ==========================================================================
 * 初始化与配置
 * ======================================================================== */

static void node_defaults(sim_node_t *n)
{
    memset(n, 0, sizeof *n);

    n->error_board        = 0u;
    n->vbus_voltage       = SIM_DEFAULT_VBUS;
    n->ibus               = 0.0f;
    n->serial_number      = 0x1122334455667788ull;
    n->hw_version_major   = 1u;
    n->fw_version_major   = 0u;
    n->break_timeout      = (uint16_t)SIM_BREAK_TIMEOUT_DEFAULT_MS;
    n->error_axis         = 0u;
    n->requested_state    = SIM_AS_IDLE;
    n->current_state      = SIM_AS_IDLE;
    n->transient_until_ms = 0u;
    n->settle_to          = SIM_AS_IDLE;
    n->watchdog_timeout   = 0.5f;
    n->enable_watchdog    = 1u;
    n->node_id            = SIM_NODE_ID_DEFAULT;
    n->is_extended        = 1u;
    n->heartbeat_rate_ms  = 0u;              /* 缺省不发，需显式配置 */
    n->error_motor        = 0u;
    n->fet_temp           = SIM_DEFAULT_FET_TEMP;
    n->motor_temp         = SIM_DEFAULT_MOTOR_TEMP;
    n->iq_measured        = 0.0f;
    n->gear_ratio         = SIM_GEAR_RATIO_DEFAULT;
    n->torque_constant    = SIM_TORQUE_CONST_DEFAULT;
    n->current_lim        = 40.0f;
    n->torque_lim         = SIM_MIT_TAU_DEFAULT;
    n->error_controller   = 0u;
    n->input_pos          = 0.0f;
    n->input_vel          = 0.0f;
    n->input_torque       = 0.0f;
    n->control_mode       = SIM_CM_POSITION;
    n->input_mode         = 3u;
    n->vel_limit          = SIM_MIT_VEL_DEFAULT;
    n->mit_max_pos        = SIM_MIT_POS_DEFAULT;
    n->mit_max_vel        = SIM_MIT_VEL_DEFAULT;
    n->mit_max_torque     = SIM_MIT_TAU_DEFAULT;
    n->mit_max_kp         = SIM_MIT_KP_DEFAULT;
    n->mit_max_kd         = SIM_MIT_KD_DEFAULT;
    n->error_encoder      = 0u;
    n->pos_estimate       = 0.0f;
    n->vel_estimate       = 0.0f;
    n->cpr                = 8192;

    n->is_fd              = 1u;              /* 缺省 CAN FD */
    n->armed              = 0u;
    n->estop              = 0u;
    n->life               = 0u;
}

void sim_bus_init(sim_bus_t *b, size_t n)
{
    size_t i;

    if (!b) return;
    memset(b, 0, sizeof *b);

    /* 分段写装配器已随总线（`b->asm_state`），上面的 memset 顺带清干净了。
       早期它是模块级静态，导致同一进程里两条虚拟总线共用槽位 →
       多 CAN 口测试会出现跨总线串味。 */

    b->n_nodes = (n == 0u) ? 1u : ((n > SIM_MAX_NODES) ? SIM_MAX_NODES : n);
    for (i = 0u; i < b->n_nodes; ++i) {
        node_defaults(&b->nodes[i]);
        if (i > 0u) b->nodes[i].node_id = (uint32_t)(i + 1u);
    }

    /* 内建描述符：让仿真设备"出厂就有端点表"。测试随后可以 sim_set_desc()
       换成真实固件夹具（那会覆盖这一份）。 */
    (void)sim_load_builtin_desc(b);
}

void sim_bus_free(sim_bus_t *b)
{
    if (!b) return;
    if (b->desc.owned && b->desc.json) {
        free(b->desc.json);
    }
    b->desc.json  = NULL;
    b->desc.owned = 0;
    b->desc.len   = 0u;
    b->desc.active = 0;
}

/* ---- node_spec 解析 ---- */

static int spec_match(const char *s, const char *key, size_t *adv)
{
    size_t k = strlen(key);
    /* 合法后继：'=' 带值、',' 下一项、';' 下一节点、'\0' 结尾 */
    if (strncmp(s, key, k) == 0
        && (s[k] == '=' || s[k] == ',' || s[k] == ';' || s[k] == '\0')) {
        *adv = k;
        return 1;
    }
    return 0;
}

int sim_configure(sim_bus_t *b, const char *spec)
{
    const char *p;

    if (!b || !spec) return -1;

    p = spec;
    while (*p) {
        size_t idx;
        char *end;
        long v;

        /* 节点下标 */
        v = strtol(p, &end, 10);
        if (end == p) return -1;
        if (v < 0 || (size_t)v >= b->n_nodes) return -1;
        idx = (size_t)v;
        p = end;

        if (*p == ':') ++p;

        while (*p && *p != ';') {
            size_t adv = 0;
            int matched = 0;

            #define TRY_KEY(k) do { if (spec_match(p, k, &adv)) matched = 1; } while (0)

            if (spec_match(p, "id", &adv)) {
                matched = 1;
                if (p[adv] == '=') {
                    b->nodes[idx].node_id = (uint32_t)strtoul(p + adv + 1, &end, 10);
                    p = end;
                } else { p += adv; }
            }
            if (!matched) { TRY_KEY("fd");      if (matched) { b->nodes[idx].is_fd = 1u; p += adv; } }
            if (!matched) { TRY_KEY("classic"); if (matched) { b->nodes[idx].is_fd = 0u; p += adv; } }
            if (!matched) { TRY_KEY("arm");     if (matched) { b->nodes[idx].armed = 1u; p += adv; } }
            if (!matched) { TRY_KEY("disarm");  if (matched) { b->nodes[idx].armed = 0u; p += adv; } }
            if (!matched) { TRY_KEY("enabled"); if (matched) { p += adv; } }
            if (!matched) { TRY_KEY("disabled");if (matched) { b->nodes[idx].node_id = 0u; p += adv; } }

            #define TRY_F32(k, field)                                              \
                if (!matched) {                                                    \
                    TRY_KEY(k);                                                    \
                    if (matched) {                                                 \
                        if (p[adv] != '=') return -1;                              \
                        b->nodes[idx].field = strtof(p + adv + 1, &end);           \
                        if (end == p + adv + 1) return -1;                         \
                        p = end;                                                    \
                    }                                                              \
                }
            #define TRY_U32(k, field)                                              \
                if (!matched) {                                                    \
                    TRY_KEY(k);                                                    \
                    if (matched) {                                                 \
                        if (p[adv] != '=') return -1;                              \
                        b->nodes[idx].field = (uint32_t)strtoul(p + adv + 1, &end, 10);\
                        if (end == p + adv + 1) return -1;                         \
                        p = end;                                                    \
                    }                                                              \
                }
            #define TRY_U16(k, field)                                              \
                if (!matched) {                                                    \
                    TRY_KEY(k);                                                    \
                    if (matched) {                                                 \
                        unsigned long _v;                                           \
                        if (p[adv] != '=') return -1;                              \
                        _v = strtoul(p + adv + 1, &end, 10);                        \
                        if (end == p + adv + 1) return -1;                         \
                        if (_v > 65535ul) return -1;                                \
                        b->nodes[idx].field = (uint16_t)_v;                        \
                        p = end;                                                    \
                    }                                                              \
                }

            TRY_F32("gear",   gear_ratio)
            TRY_F32("tconst", torque_constant)
            TRY_F32("pmax",   mit_max_pos)
            TRY_F32("vmax",   mit_max_vel)
            TRY_F32("kpmax",  mit_max_kp)
            TRY_F32("kdmax",  mit_max_kd)
            TRY_F32("tmax",   mit_max_torque)
            TRY_F32("vb",     vbus_voltage)
            TRY_F32("temp",   motor_temp)
            TRY_U32("hb",      heartbeat_rate_ms)
            TRY_U16("timeout", break_timeout)

            if (!matched) return -1;            /* 未知键 */

            if (*p == ',') ++p;
        }
        if (*p == ';') ++p;
    }
    return 0;
}

/* ==========================================================================
 * 便捷查询
 * ======================================================================== */

sim_node_t *sim_find_node(sim_bus_t *b, uint32_t node_id)
{
    size_t i;
    if (!b) return NULL;
    for (i = 0u; i < b->n_nodes; ++i) {
        if (b->nodes[i].node_id == node_id) return &b->nodes[i];
    }
    return NULL;
}

void sim_clear_stats(sim_bus_t *b)
{
    if (!b) return;
    b->rx_frames = 0u;
    b->rx_for_me = 0u;
    b->tx_frames = 0u;
    b->bad_len_drops = 0u;
    b->unhandled = 0u;
    b->txq_dropped = 0u;
    b->force_txq_full = 0u;
    b->txq_head = 0u;
    b->txq_tail = 0u;
}
