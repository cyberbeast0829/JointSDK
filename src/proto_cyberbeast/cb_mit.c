/**
 * @file    cb_mit.c
 * @brief   CYBERBEAST MIT 紧凑编解码实现（见 cb_mit.h）
 */

#include "cb_mit.h"
#include "cb_frame.h"   /* CB_MAX_BROADCAST_DEVICES */

/* ==========================================================================
 * 定点原语
 * ======================================================================== */

int32_t cb_mit_f2u_raw(float x, float x_min, float x_max, int bits)
{
    /* 精确复刻固件： (int)((x-offset)*((float)((1<<bits)-1))/span)
       语义：**向零截断，且不钳位** → 越界输入会回绕（见 cb_mit.h 的危寄说明）。
       本函数仅供诊断/向量对拍使用，SDK 实际发包走 cb_mit_f2u()。 */
    float span   = x_max - x_min;
    float offset = x_min;

    if (bits <= 0 || bits > 30) return 0;
    if (span == 0.0f)           return 0;   /* 固件此处除零，行为未定义 */

    return (int32_t)((x - offset) * (float)((1 << bits) - 1) / span);
}

/* 对称四舍五入（远离零），不依赖 libm。
 * 传入值已钳位在 [0, maxv]，故只需处理非负分支。 */
static int32_t round_nonneg(float x)
{
    return (int32_t)(x + 0.5f);
}

int32_t cb_mit_f2u(float x, float x_min, float x_max, int bits)
{
    float span;
    int32_t maxv;

    if (bits <= 0 || bits > 30) return 0;
    if (x_min > x_max) { float t = x_min; x_min = x_max; x_max = t; }

    span = x_max - x_min;
    maxv = (int32_t)((1u << (unsigned)bits) - 1u);
    if (span == 0.0f) return 0;

    if (x != x) {                       /* NaN：按最小值处理（调用方会置 INVALID） */
        x = x_min;
    } else if (x < x_min) {
        x = x_min;
    } else if (x > x_max) {
        x = x_max;
    }

    {
        /* 四舍五入而非截断：
         *   1) 量化误差从 1.0 LSB 降到 0.5 LSB；
         *   2) 使 pack(unpack(bytes)) == bytes 幂等 —— 否则“回读再下发”
         *      的控制环每周期会因截断而漂移 1 LSB。
         *   3) 与生态里的 cyberbeast_tool.py（用 round()）一致。
         * 注意：固件自身的 float_to_uint 用截断，但那只用于**响应编码**，
         *       命令编码由主机侧决定，因此这里选择精度更高的做法。 */
        int32_t v = round_nonneg((x - x_min) * (float)maxv / span);
        if (v < 0)     v = 0;
        if (v > maxv)  v = maxv;        /* 浮点误差下的最后一道保险 */
        return v;
    }
}

float cb_mit_u2f(int32_t x_int, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;

    if (bits <= 0 || bits > 30) return 0.0f;
    return (float)x_int * span / (float)((1u << (unsigned)bits) - 1u) + offset;
}

int cb_mit_range_valid(const cb_mit_range_t *r)
{
    if (!r) return 0;
    if (!(r->pos_max > 0.0f)) return 0;
    if (!(r->vel_max > 0.0f)) return 0;
    if (!(r->kp_max  > 0.0f)) return 0;
    if (!(r->kd_max  > 0.0f)) return 0;
    if (!(r->tau_max > 0.0f)) return 0;
    return 1;
}

/* ==========================================================================
 * 钳位辅助
 * ======================================================================== */

/**
 * 把对称量钳到 [-lim, +lim]。
 * @param invalid 出现 NaN 时置 1（该字段按 0 处理）。±Inf 走边界钳位，不算 invalid。
 * @return 钳位后的值；*clamped 置 1 表示发生了变化
 */
static float clamp_sym(float x, float lim, uint8_t *clamped, uint8_t *invalid)
{
    if (x != x) {                       /* NaN */
        *clamped = 1u;
        *invalid = 1u;
        return 0.0f;
    }
    if (x > lim)  { *clamped = 1u; return  lim; }   /* +Inf 也走这里 */
    if (x < -lim) { *clamped = 1u; return -lim; }   /* -Inf 也走这里 */
    return x;
}

/** 把非负量钳到 [0, lim]。NaN 视为 invalid，±Inf 走边界钳位。 */
static float clamp_pos(float x, float lim, uint8_t *clamped, uint8_t *invalid)
{
    if (x != x) {                       /* NaN */
        *clamped = 1u;
        *invalid = 1u;
        return 0.0f;
    }
    if (x < 0.0f) { *clamped = 1u; return 0.0f; }
    if (x > lim)  { *clamped = 1u; return lim; }
    return x;
}

/* ==========================================================================
 * 命令帧
 * ======================================================================== */

void cb_mit_pack_command(uint8_t *dst8, const cb_mit_range_t *range,
                         float pos, float vel, float kp, float kd, float tau,
                         uint8_t *clamped)
{
    uint8_t flags = 0u, inv = 0u;
    uint8_t c = 0u;
    int32_t p_int, v_int, kp_int, kd_int, t_int;

    if (!dst8 || !range) {
        if (clamped) *clamped = CB_MIT_INVALID;
        if (dst8) { unsigned i; for (i = 0; i < CB_MIT_SLOT_BYTES; ++i) dst8[i] = 0u; }
        return;
    }

    pos = clamp_sym(pos, range->pos_max, &c, &inv);
    if (c) flags |= CB_MIT_CLAMP_POS;
    c = 0u;
    vel = clamp_sym(vel, range->vel_max, &c, &inv);
    if (c) flags |= CB_MIT_CLAMP_VEL;
    c = 0u;
    kp  = clamp_pos(kp,  range->kp_max,  &c, &inv);
    if (c) flags |= CB_MIT_CLAMP_KP;
    c = 0u;
    kd  = clamp_pos(kd,  range->kd_max,  &c, &inv);
    if (c) flags |= CB_MIT_CLAMP_KD;
    c = 0u;
    tau = clamp_sym(tau, range->tau_max, &c, &inv);
    if (c) flags |= CB_MIT_CLAMP_TAU;

    if (inv) flags |= CB_MIT_INVALID;

    p_int  = cb_mit_f2u(pos, -range->pos_max, range->pos_max, 16);
    v_int  = cb_mit_f2u(vel, -range->vel_max, range->vel_max, 12);
    kp_int = cb_mit_f2u(kp,   0.0f,           range->kp_max,  12);
    kd_int = cb_mit_f2u(kd,   0.0f,           range->kd_max,  12);
    t_int  = cb_mit_f2u(tau, -range->tau_max, range->tau_max, 12);

    dst8[0] = (uint8_t)(p_int >> 8);
    dst8[1] = (uint8_t)(p_int);
    dst8[2] = (uint8_t)(v_int >> 4);
    dst8[3] = (uint8_t)(((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F));
    dst8[4] = (uint8_t)(kp_int);
    dst8[5] = (uint8_t)(kd_int >> 4);
    dst8[6] = (uint8_t)(((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F));
    dst8[7] = (uint8_t)(t_int);

    if (clamped) *clamped = flags;
}

void cb_mit_unpack_command(const uint8_t *src8, const cb_mit_range_t *range,
                           float *pos, float *vel, float *kp, float *kd, float *tau)
{
    int32_t p_int, v_int, kp_int, kd_int, t_int;

    if (!src8 || !range) {
        if (pos) *pos = 0.0f;
        if (vel) *vel = 0.0f;
        if (kp)  *kp  = 0.0f;
        if (kd)  *kd  = 0.0f;
        if (tau) *tau = 0.0f;
        return;
    }

    p_int  = ((int32_t)src8[0] << 8) | (int32_t)src8[1];
    v_int  = ((int32_t)src8[2] << 4) | ((int32_t)src8[3] >> 4);
    kp_int = ((int32_t)(src8[3] & 0x0F) << 8) | (int32_t)src8[4];
    kd_int = ((int32_t)src8[5] << 4) | ((int32_t)src8[6] >> 4);
    t_int  = ((int32_t)(src8[6] & 0x0F) << 8) | (int32_t)src8[7];

    if (pos) *pos = cb_mit_u2f(p_int,  -range->pos_max, range->pos_max, 16);
    if (vel) *vel = cb_mit_u2f(v_int,  -range->vel_max, range->vel_max, 12);
    if (kp)  *kp  = cb_mit_u2f(kp_int,  0.0f,           range->kp_max,  12);
    if (kd)  *kd  = cb_mit_u2f(kd_int,  0.0f,           range->kd_max,  12);
    if (tau) *tau = cb_mit_u2f(t_int,  -range->tau_max, range->tau_max, 12);
}

size_t cb_mit_bcast_frame_len(uint8_t max_slot_used)
{
    if (max_slot_used >= CB_MAX_BROADCAST_DEVICES) return 0;   /* 超出槽位上限 */
    return ((size_t)max_slot_used + 1u) * CB_MIT_SLOT_BYTES;
}

/* ==========================================================================
 * 响应帧
 * ======================================================================== */

float cb_mit_response_max_current(float tau_max, float torque_constant)
{
    float mc = 40.0f;                       /* 固件回退值 */

    if (torque_constant > 0.001f) {
        mc = tau_max / torque_constant;
        if (mc > 80.0f) mc = 80.0f;         /* 固件钳位 */
    }
    return mc;
}

void cb_mit_unpack_response(const uint8_t *src8, const cb_mit_range_t *range,
                            float max_current, cb_mit_response_t *out)
{
    int32_t p_int, v_int, c_int;

    if (!src8 || !out) return;
    if (!range) { cb_mit_unpack_response_raw(src8, out); return; }

    p_int = ((int32_t)src8[0] << 8) | (int32_t)src8[1];
    v_int = ((int32_t)src8[2] << 4) | ((int32_t)src8[3] >> 4);
    c_int = ((int32_t)src8[4] << 4) | ((int32_t)src8[5] >> 4);

    out->pos      = cb_mit_u2f(p_int, -range->pos_max, range->pos_max, 16);
    out->vel      = cb_mit_u2f(v_int, -range->vel_max, range->vel_max, 12);
    out->current  = cb_mit_u2f(c_int, -max_current,    max_current,    12);
    out->err_code = (uint8_t)(src8[3] & 0x0F);
    out->mode     = (uint8_t)(src8[5] & 0x0F);
    /* ⚠ 用 int16：值域为 -50..205，int8 会溢出（205 > 127） */
    out->motor_temp_c = (int16_t)((int16_t)src8[6] - 50);
    out->mos_temp_c   = (int16_t)((int16_t)src8[7] - 50);
}

void cb_mit_unpack_response_raw(const uint8_t *src8, cb_mit_response_t *out)
{
    out->pos      = 0.0f;
    out->vel      = 0.0f;
    out->current  = cb_mit_u2f(((int32_t)src8[4] << 4) | ((int32_t)src8[5] >> 4),
                               -40.0f, 40.0f, 12);     /* 固件回退量程 */
    out->err_code = (uint8_t)(src8[3] & 0x0F);
    out->mode     = (uint8_t)(src8[5] & 0x0F);
    out->motor_temp_c = (int16_t)((int16_t)src8[6] - 50);
    out->mos_temp_c   = (int16_t)((int16_t)src8[7] - 50);
}

const char *cb_mit_error_name(uint8_t err_code)
{
    switch (err_code) {
    case CB_ERR_NONE:          return "NONE";
    case CB_ERR_MOTOR:         return "MOTOR";
    case CB_ERR_ENCODER:       return "ENCODER";
    case CB_ERR_CONTROLLER:    return "CONTROLLER";
    case CB_ERR_VOLTAGE:       return "VOLTAGE";
    case CB_ERR_OVER_TEMP:     return "OVER_TEMP";
    case CB_ERR_OVER_CURRENT:  return "OVER_CURRENT";
    case CB_ERR_STALL:         return "STALL";
    case CB_ERR_OVERLOAD:      return "OVERLOAD";
    case CB_ERR_CAN_TIMEOUT:   return "CAN_TIMEOUT";
    case CB_ERR_MULTIPLE:      return "MULTIPLE";
    default:                   return "unknown";
    }
}

const char *cb_mit_mode_name(uint8_t mode)
{
    switch (mode) {
    case CB_MODE_RESET:       return "RESET";
    case CB_MODE_CALIBRATING: return "CALIBRATING";
    case CB_MODE_IDLE:        return "IDLE";
    case CB_MODE_CLOSED_LOOP: return "CLOSED_LOOP";
    case CB_MODE_MIT:         return "MIT";
    case CB_MODE_POSITION:    return "POSITION";
    case CB_MODE_VELOCITY:    return "VELOCITY";
    case CB_MODE_TORQUE:      return "TORQUE";
    default:                  return "unknown";
    }
}

/* ==========================================================================
 * 响应编码（虚拟设备 / 上位机仿真用）
 *
 * ⚠ 数值语义**故意**与 pack_command 不同：
 *   pack_command（主站发命令）→ 四舍五入 + 钳位
 *   pack_response（设备上报）  → **截断** + 钳位
 * 后者对应固件 float_to_uint 的行为（固件缺钳位，这里补上），
 * 目的是逐字节重现真设备发出的帧。
 * ======================================================================== */

/** 固件语义的定点转换 + 钳位（截断） */
static int32_t resp_f2u(float x, float x_min, float x_max, int bits)
{
    float span;
    int32_t maxv;
    int32_t v;

    if (bits <= 0 || bits > 30) return 0;
    span = x_max - x_min;
    if (span == 0.0f) return 0;
    maxv = (int32_t)((1u << (unsigned)bits) - 1u);

    if (x != x) x = x_min;              /* NaN → 取下界 */
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;

    v = (int32_t)((x - x_min) * (float)maxv / span);   /* 截断 */
    if (v < 0)    v = 0;
    if (v > maxv) v = maxv;
    return v;
}

/** 温度 °C → 线上 u8（固件：clamp(t + 50, 0, 255) 后截断） */
static uint8_t resp_temp_u8(int16_t celsius)
{
    int32_t v = (int32_t)celsius + 50;
    if (v < 0)   v = 0;
    if (v > 255) v = 255;
    return (uint8_t)v;
}

size_t cb_mit_pack_response(uint8_t *dst8, const cb_mit_range_t *range,
                            const cb_mit_response_t *r, float max_current)
{
    int32_t p_int, v_int, c_int;

    if (!dst8 || !range || !r) return 0u;

    p_int = resp_f2u(r->pos,     -range->pos_max, range->pos_max, 16);
    v_int = resp_f2u(r->vel,     -range->vel_max, range->vel_max, 12);
    c_int = resp_f2u(r->current, -max_current,    max_current,    12);

    dst8[0] = (uint8_t)(p_int >> 8);
    dst8[1] = (uint8_t)(p_int);
    dst8[2] = (uint8_t)(v_int >> 4);
    /* 低 4 位与邻字段拼一个字节：两个操作数都先转成 `unsigned`，
       否则 `int | unsigned` 会触发 `-Wsign-conversion`（GCC 9 会报，见
       cb_heartbeat_make_life_flags 的注释）。 */
    dst8[3] = (uint8_t)((((unsigned)v_int & 0x0Fu) << 4)
                        | ((unsigned)r->err_code & 0x0Fu));
    dst8[4] = (uint8_t)(c_int >> 4);
    dst8[5] = (uint8_t)((((unsigned)c_int & 0x0Fu) << 4)
                        | ((unsigned)r->mode & 0x0Fu));
    dst8[6] = resp_temp_u8(r->motor_temp_c);
    dst8[7] = resp_temp_u8(r->mos_temp_c);

    return CB_MIT_SLOT_BYTES;
}
