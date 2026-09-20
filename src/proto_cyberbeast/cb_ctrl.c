/**
 * @file    cb_ctrl.c
 * @brief   CYBERBEAST 实时控制帧编解码实现（见 cb_ctrl.h）
 *
 * 逐字节与固件 Firmware/communication/can/can_cyberbeast.cpp 的
 * cmd_pos_control / cmd_vel_control / cmd_torque_control / cmd_current_control
 * 对齐；打包侧额外做钳位与标志上报（固件不钳位）。
 */

#include "cb_ctrl.h"
#include "cb_frame.h"
#include "cb_mit.h"    /* CB_MIT_SLOT_BYTES（cb_ctrl_min_len 里判断 MIT 帧长） */

/* ==========================================================================
 * 局部工具：钳位 + 修正标志（与 cb_mit.c 同一套语义）
 * ======================================================================== */

/** 对称四舍五入（远离零），不依赖 libm */
static int32_t ctl_round(float x)
{
    if (x >= 0.0f) return (int32_t)(x + 0.5f);
    return -(int32_t)(0.5f - x);
}

/**
 * 浮点 → int16（钳位到**完整** int16 值域 + 四舍五入）。
 *
 * ⚠ 钳位区间是 [-32768, 32767]而不是对称的 ±32767：后者会白白丢掉
 * -32768 这个合法码（参考工具 cyberbeast_tool.py 也是用完整区间）。
 * 固件此处的写法是裸 `static_cast<int16_t>(...)`，越界时行为是实现定义的；
 * SDK 必须钳位，否则会出现“越界回绕变号”的危害（见 cb_mit.h 说明）。
 */
static int16_t ctl_f2i16(float x, uint8_t bit, uint8_t *flags, uint8_t *invalid)
{
    float c;
    int32_t r;

    if (x != x) {                       /* NaN */
        if (flags)   *flags |= bit;
        if (invalid) *invalid = 1u;
        return 0;
    }

    c = x;
    if (c > (float)CB_CTRL_I16_MAX) {
        c = (float)CB_CTRL_I16_MAX;
        if (flags) *flags |= bit;
    } else if (c < (float)CB_CTRL_I16_MIN) {
        c = (float)CB_CTRL_I16_MIN;     /* ±Inf 也走这里 */
        if (flags) *flags |= bit;
    }

    r = ctl_round(c);
    if (r > CB_CTRL_I16_MAX) r = CB_CTRL_I16_MAX;
    if (r < CB_CTRL_I16_MIN) r = CB_CTRL_I16_MIN;
    return (int16_t)r;
}

static void ctl_finish_flags(uint8_t *flags, uint8_t value)
{
    /* ⚠ 必须是**赋值**而不是 `|=`：调用方常把一个变量在多次调用间复用，
       累加式写入会把上一帧的钳位标志残留到下一帧上（已由测试的哨兵值暴露）。
       与 cb_mit_pack_command 的 `*clamped = flags;` 保持一致。 */
    if (flags) *flags = value;
}

/* ==========================================================================
 * POS_CONTROL (0x01 / 0x81)
 *
 * Classic (8 B): [0..3] pos_deg f32 BE | [4..5] vel_limit i16 BE | [6..7] cur_limit i16 BE(0.1A)
 * FD     (12 B): [0..3] pos_deg f32 BE | [4..7] vel_limit f32 BE | [8..11] cur_limit f32 BE
 * ======================================================================== */

size_t cb_ctrl_pos_pack(uint8_t *dst, int classic,
                        float pos_deg, float vel_limit_rpm, float cur_limit_a,
                        uint8_t *flags)
{
    uint8_t f = 0u, inv = 0u;

    if (!dst) return 0u;

    /* pos_deg 的合法域由设备的 mit_max_pos 决定，此处无法得知 → 不做钳位，
       只做 NaN 防护（NaN 会被固件当作 0 以外的垃圾值参与运算）。*/
    if (pos_deg != pos_deg) {
        f |= CB_CTRL_POS_F_POS;
        inv = 1u;
        pos_deg = 0.0f;
    }

    if (classic) {
        /* vel_limit：int16 RPM */
        int16_t vel_raw = ctl_f2i16(vel_limit_rpm, CB_CTRL_POS_F_VEL, &f, &inv);
        /* cur_limit：int16 的 0.1 A/bit → 量程 −3276.8..3276.7 A。
           先把物理量乘到定点标度，再按 int16 值域钳位。 */
        int16_t cur_raw = ctl_f2i16(cur_limit_a * 10.0f, CB_CTRL_POS_F_CUR,
                                    &f, &inv);

        cb_be_put_f32(dst, pos_deg);
        cb_be_put_i16(dst + 4, vel_raw);
        cb_be_put_i16(dst + 6, cur_raw);

        ctl_finish_flags(flags, (uint8_t)(f | (inv ? CB_CTRL_F_INVALID : 0u)));
        return CB_CTRL_POS_LEN_CLASSIC;
    }

    /* FD：三个 float32，无需量化；仅做 NaN 防护 */
    if (vel_limit_rpm != vel_limit_rpm) { f |= CB_CTRL_POS_F_VEL; inv = 1u; vel_limit_rpm = 0.0f; }
    if (cur_limit_a    != cur_limit_a)  { f |= CB_CTRL_POS_F_CUR; inv = 1u; cur_limit_a    = 0.0f; }

    cb_be_put_f32(dst,      pos_deg);
    cb_be_put_f32(dst + 4,  vel_limit_rpm);
    cb_be_put_f32(dst + 8,  cur_limit_a);

    ctl_finish_flags(flags, (uint8_t)(f | (inv ? CB_CTRL_F_INVALID : 0u)));
    return CB_CTRL_POS_LEN_FD;
}

int cb_ctrl_pos_unpack(const uint8_t *src, size_t len, int classic,
                       float *pos_deg, float *vel_limit_rpm, float *cur_limit_a)
{
    if (!src) return -1;

    /* 与固件一致：先按 isClassic 选分支，再校验长度 */
    if (classic) {
        if (len < CB_CTRL_POS_LEN_CLASSIC) return -1;
        if (pos_deg)        *pos_deg        = cb_be_get_f32(src);
        if (vel_limit_rpm)  *vel_limit_rpm  = (float)cb_be_get_i16(src + 4);
        if (cur_limit_a)    *cur_limit_a    = (float)cb_be_get_i16(src + 6)
                                              * CB_CTRL_CLASSIC_CUR_LSB;
        return 0;
    }

    if (len < CB_CTRL_POS_LEN_FD) return -1;
    if (pos_deg)       *pos_deg       = cb_be_get_f32(src);
    if (vel_limit_rpm) *vel_limit_rpm = cb_be_get_f32(src + 4);
    if (cur_limit_a)   *cur_limit_a   = cb_be_get_f32(src + 8);
    return 0;
}

/* ==========================================================================
 * VEL_CONTROL (0x02 / 0x82) — 8 B，Classic 与 FD 同布局
 * ======================================================================== */

size_t cb_ctrl_vel_pack(uint8_t *dst, float target_vel_rpm, float cur_limit_a,
                        uint8_t *flags)
{
    uint8_t f = 0u, inv = 0u;

    if (!dst) return 0u;

    if (target_vel_rpm != target_vel_rpm) { f |= CB_CTRL_VEL_F_VEL; inv = 1u; target_vel_rpm = 0.0f; }
    if (cur_limit_a    != cur_limit_a)    { f |= CB_CTRL_VEL_F_CUR; inv = 1u; cur_limit_a    = 0.0f; }

    cb_be_put_f32(dst,     target_vel_rpm);
    cb_be_put_f32(dst + 4, cur_limit_a);

    ctl_finish_flags(flags, (uint8_t)(f | (inv ? CB_CTRL_F_INVALID : 0u)));
    return CB_CTRL_VEL_LEN;
}

int cb_ctrl_vel_unpack(const uint8_t *src, size_t len,
                       float *target_vel_rpm, float *cur_limit_a)
{
    if (!src || len < CB_CTRL_VEL_LEN) return -1;
    if (target_vel_rpm) *target_vel_rpm = cb_be_get_f32(src);
    if (cur_limit_a)    *cur_limit_a    = cb_be_get_f32(src + 4);
    return 0;
}

/* ==========================================================================
 * TORQUE_CONTROL (0x03 / 0x83) — ≥4 B，电机端 N·m
 * ======================================================================== */

size_t cb_ctrl_torque_pack(uint8_t *dst, float target_torque_nm, uint8_t *flags)
{
    uint8_t f = 0u, inv = 0u;

    if (!dst) return 0u;

    /* 力矩是 float32，**没有**线上量程可钳 —— 实际限制由设备的
       `torque_lim` / `current_lim` 决定，L2 无从得知。因此这里只防 NaN。 */
    if (target_torque_nm != target_torque_nm) {
        f |= CB_CTRL_TAU_F_TAU;
        inv = 1u;
        target_torque_nm = 0.0f;
    }

    cb_be_put_f32(dst, target_torque_nm);

    ctl_finish_flags(flags, (uint8_t)(f | (inv ? CB_CTRL_F_INVALID : 0u)));
    return CB_CTRL_TORQUE_LEN;
}

int cb_ctrl_torque_unpack(const uint8_t *src, size_t len, float *target_torque_nm)
{
    if (!src || len < CB_CTRL_TORQUE_LEN) return -1;
    if (target_torque_nm) *target_torque_nm = cb_be_get_f32(src);
    return 0;
}

/* ==========================================================================
 * CURRENT_CONTROL (0x04) — ≥4 B，电机端 A，无广播版本
 * ======================================================================== */

size_t cb_ctrl_current_pack(uint8_t *dst, float target_current_a, uint8_t *flags)
{
    uint8_t f = 0u, inv = 0u;

    if (!dst) return 0u;

    /* 同 TORQUE：float32 无线上量程，只防 NaN。 */
    if (target_current_a != target_current_a) {
        f |= CB_CTRL_CUR_F_CUR;
        inv = 1u;
        target_current_a = 0.0f;
    }

    cb_be_put_f32(dst, target_current_a);

    ctl_finish_flags(flags, (uint8_t)(f | (inv ? CB_CTRL_F_INVALID : 0u)));
    return CB_CTRL_CURRENT_LEN;
}

int cb_ctrl_current_unpack(const uint8_t *src, size_t len, float *target_current_a)
{
    if (!src || len < CB_CTRL_CURRENT_LEN) return -1;
    if (target_current_a) *target_current_a = cb_be_get_f32(src);
    return 0;
}

/* ==========================================================================
 * 辅助
 * ======================================================================== */

int cb_ctrl_is_control_msgtype(uint8_t msgtype)
{
    switch (msgtype) {
    case CB_MSG_MIT_CONTROL:
    case CB_MSG_POS_CONTROL:
    case CB_MSG_VEL_CONTROL:
    case CB_MSG_TORQUE_CONTROL:
    case CB_MSG_CURRENT_CONTROL:
    /* 广播变体（注意：没有 CURRENT 广播） */
    case CB_MSG_MIT_CONTROL_BCAST:
    case CB_MSG_POS_CONTROL_BCAST:
    case CB_MSG_VEL_CONTROL_BCAST:
    case CB_MSG_TORQUE_CONTROL_BCAST:
        return 1;
    default:
        return 0;
    }
}

size_t cb_ctrl_min_len(uint8_t msgtype, int classic)
{
    switch (msgtype) {
    case CB_MSG_POS_CONTROL:
    case CB_MSG_POS_CONTROL_BCAST:
        return classic ? CB_CTRL_POS_LEN_CLASSIC : CB_CTRL_POS_LEN_FD;
    case CB_MSG_VEL_CONTROL:
    case CB_MSG_VEL_CONTROL_BCAST:
        return CB_CTRL_VEL_LEN;
    case CB_MSG_TORQUE_CONTROL:
    case CB_MSG_TORQUE_CONTROL_BCAST:
        return CB_CTRL_TORQUE_LEN;
    case CB_MSG_CURRENT_CONTROL:
        return CB_CTRL_CURRENT_LEN;
    case CB_MSG_MIT_CONTROL:
    case CB_MSG_MIT_CONTROL_BCAST:
        return CB_MIT_SLOT_BYTES;
    default:
        return 0u;
    }
}

int cb_ctrl_expects_response(uint8_t msgtype)
{
    switch (msgtype) {
    case CB_MSG_POS_CONTROL:
    case CB_MSG_VEL_CONTROL:
    case CB_MSG_TORQUE_CONTROL:
        return 1;
    default:
        /* CURRENT_CONTROL：固件不回任何应答；
           广播帧：`!is_bcast` 条件使其也不应答。 */
        return 0;
    }
}
