/**
 * @file    cb_heartbeat.c
 * @brief   CYBERBEAST 心跳帧编解码实现（见 cb_heartbeat.h）
 *
 * 与固件 can_cyberbeast.cpp 的 send_heartbeat() 逐字节对齐。
 */

#include "cb_heartbeat.h"
#include "cb_frame.h"

/* ==========================================================================
 * 定标工具
 * ======================================================================== */

static int32_t hb_round(float x)
{
    if (x >= 0.0f) return (int32_t)(x + 0.5f);
    return -(int32_t)(0.5f - x);
}

static float hb_clamp_f(float x, float lo, float hi)
{
    if (x != x) return 0.0f;            /* NaN → 0（固件此处行为未定义） */
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static int hb_clamp_i32(float x, int32_t lo, int32_t hi)
{
    float v = hb_clamp_f(x, (float)lo, (float)hi);
    int32_t r = hb_round(v);
    if (r < lo) r = lo;
    if (r > hi) r = hi;
    return r;
}

/** 温度 °C → 线上 u8（固件用截断，此处用四舍五入；两者都先钳到 0..255） */
static uint8_t hb_temp_to_u8(float celsius)
{
    float v = hb_clamp_f(celsius, (float)CB_HB_TEMP_MIN, (float)CB_HB_TEMP_MAX);
    int32_t r = hb_round(v) + CB_HB_TEMP_OFFSET;
    if (r < 0) r = 0;
    if (r > 255) r = 255;
    return (uint8_t)r;
}

static int16_t hb_u8_to_temp(uint8_t raw)
{
    /* −50..205，必须用 int16（int8 会溢出） */
    return (int16_t)((int16_t)raw - CB_HB_TEMP_OFFSET);
}

/* ==========================================================================
 * 解码
 * ======================================================================== */

int cb_heartbeat_decode_classic(const uint8_t *src, size_t len, cb_heartbeat_t *out)
{
    int16_t pos_raw, vel_raw;

    if (!src || len < CB_HB_LEN_CLASSIC) return -1;
    if (!out) return 0;

    out->is_fd     = 0;
    out->life      = cb_heartbeat_life(src[0]);
    out->err_flags = cb_heartbeat_flags(src[0]);
    out->state     = cb_heartbeat_state(src[1]);
    out->control_mode = cb_heartbeat_control_mode(src[1]);

    out->motor_temp_c = hb_u8_to_temp(src[2]);
    out->mos_temp_c   = 0;
    out->have_mos_temp = 0;

    pos_raw = cb_be_get_i16(src + 3);
    vel_raw = cb_be_get_i16(src + 5);
    out->pos_turns       = (float)pos_raw / CB_HB_CLASSIC_POS_SCALE;
    out->vel_turns_per_s = (float)vel_raw / CB_HB_CLASSIC_VEL_SCALE;

    /* iq：int8，0.5 A/bit */
    out->iq_a = (float)(int8_t)src[7] * CB_HB_CLASSIC_IQ_LSB;

    out->vbus_v  = 0.0f;
    out->ibus_a  = 0.0f;
    out->have_vbus = 0;
    return 0;
}

int cb_heartbeat_decode_fd(const uint8_t *src, size_t len, cb_heartbeat_t *out)
{
    int32_t pos_raw, vel_raw;

    if (!src || len < CB_HB_LEN_FD) return -1;
    if (!out) return 0;

    out->is_fd     = 1;
    out->life      = cb_heartbeat_life(src[0]);
    out->err_flags = cb_heartbeat_flags(src[0]);
    out->state     = cb_heartbeat_state(src[1]);
    out->control_mode = cb_heartbeat_control_mode(src[1]);

    out->motor_temp_c  = hb_u8_to_temp(src[2]);
    out->mos_temp_c    = hb_u8_to_temp(src[3]);
    out->have_mos_temp = 1;

    /* vbus 是 **无符号** u16（0.1 V）；ibus 是**有符号** i16（0.01 A） */
    out->vbus_v   = (float)cb_be_get_u16(src + 4) * CB_HB_FD_VBUS_LSB;
    out->ibus_a   = (float)cb_be_get_i16(src + 6) * CB_HB_FD_IBUS_LSB;
    out->have_vbus = 1;

    pos_raw = cb_be_get_i32(src + 8);
    vel_raw = cb_be_get_i32(src + 12);
    out->pos_turns       = (float)pos_raw / CB_HB_FD_POS_SCALE;
    out->vel_turns_per_s = (float)vel_raw / CB_HB_FD_VEL_SCALE;

    out->iq_a = (float)cb_be_get_i16(src + 16) * CB_HB_FD_IQ_LSB;
    return 0;
}

int cb_heartbeat_decode(const uint8_t *src, size_t len, cb_heartbeat_t *out)
{
    if (!src) return -1;
    if (len == CB_HB_LEN_CLASSIC) return cb_heartbeat_decode_classic(src, len, out);
    if (len >= CB_HB_LEN_FD)      return cb_heartbeat_decode_fd(src, len, out);
    return -1;
}

/* ==========================================================================
 * 编码
 * ======================================================================== */

size_t cb_heartbeat_encode_classic(uint8_t *dst, const cb_heartbeat_t *v)
{
    int32_t pos_raw, vel_raw;

    if (!dst || !v) return 0u;

    dst[0] = cb_heartbeat_make_life_flags(v->life, v->err_flags);
    {
        int sm = cb_heartbeat_make_state_mode(v->state, v->control_mode);
        dst[1] = (uint8_t)(sm < 0 ? 0 : sm);
    }
    dst[2] = hb_temp_to_u8((float)v->motor_temp_c);

    /* Classic 是 int16，量程 ±32767 → ±327.67 turns。固件同样钳位。 */
    pos_raw = hb_clamp_i32(v->pos_turns * CB_HB_CLASSIC_POS_SCALE,
                           -32768, 32767);
    vel_raw = hb_clamp_i32(v->vel_turns_per_s * CB_HB_CLASSIC_VEL_SCALE,
                           -32768, 32767);
    cb_be_put_i16(dst + 3, (int16_t)pos_raw);
    cb_be_put_i16(dst + 5, (int16_t)vel_raw);

    /* iq：int8，0.5 A/bit → ±64 A */
    {
        int32_t iq_raw = hb_clamp_i32(v->iq_a / CB_HB_CLASSIC_IQ_LSB, -128, 127);
        dst[7] = (uint8_t)(int8_t)iq_raw;
    }
    return CB_HB_LEN_CLASSIC;
}

size_t cb_heartbeat_encode_fd(uint8_t *dst, const cb_heartbeat_t *v)
{
    int32_t pos_raw, vel_raw, iq_raw;

    if (!dst || !v) return 0u;

    dst[0] = cb_heartbeat_make_life_flags(v->life, v->err_flags);
    {
        int sm = cb_heartbeat_make_state_mode(v->state, v->control_mode);
        dst[1] = (uint8_t)(sm < 0 ? 0 : sm);
    }
    dst[2] = hb_temp_to_u8((float)v->motor_temp_c);
    dst[3] = hb_temp_to_u8(v->have_mos_temp ? (float)v->mos_temp_c : 0.0f);

    /* vbus 无符号 u16，0..65535 → 0..6553.5 V。
       固件不钳位；此处钳位以免负值/超大值产生垃圾。 */
    cb_be_put_u16(dst + 4, (uint16_t)hb_clamp_i32(v->vbus_v / CB_HB_FD_VBUS_LSB,
                                                  0, 65535));
    cb_be_put_i16(dst + 6, (int16_t)hb_clamp_i32(v->ibus_a / CB_HB_FD_IBUS_LSB,
                                                 -32768, 32767));

    /* FD 是 int32，固件**不钳位**；此处钳到 int32 值域等价范围 */
    pos_raw = hb_clamp_i32(v->pos_turns * CB_HB_FD_POS_SCALE,
                           -2147483000, 2147483000);
    vel_raw = hb_clamp_i32(v->vel_turns_per_s * CB_HB_FD_VEL_SCALE,
                           -2147483000, 2147483000);
    cb_be_put_i32(dst + 8,  pos_raw);
    cb_be_put_i32(dst + 12, vel_raw);

    iq_raw = hb_clamp_i32(v->iq_a / CB_HB_FD_IQ_LSB, -32768, 32767);
    cb_be_put_i16(dst + 16, (int16_t)iq_raw);

    return CB_HB_LEN_FD;
}

/* ==========================================================================
 * 字段级辅助
 * ======================================================================== */

uint8_t cb_heartbeat_life(uint8_t b0)
{
    return (uint8_t)((b0 >> 5) & 0x07u);
}

uint8_t cb_heartbeat_flags(uint8_t b0)
{
    return (uint8_t)(b0 & CB_HB_ERR_MASK);
}

uint8_t cb_heartbeat_make_life_flags(uint8_t life, uint8_t err_flags)
{
    /*
     * 全程用 `unsigned` 运算：`uint8_t` 参与运算时会整型提升为 `int`，
     * 再与无符号掩码（`CB_HB_ERR_MASK` 带 `u` 后缀）相遇就会发生
     * `int → unsigned` 的隐式符号转换。GCC 13 靠值域分析看得出"掩码后必然非负"
     * 而不报，**GCC 9（Ubuntu 20.04 LTS，正是目标平台）会报 `-Wsign-conversion`**
     * —— 显式转换既消除告警，也把意图写清楚。
     */
    return (uint8_t)((((unsigned)life & 0x07u) << 5)
                     | ((unsigned)err_flags & CB_HB_ERR_MASK));
}

uint8_t cb_heartbeat_state(uint8_t b1)
{
    return (uint8_t)((b1 >> 4) & 0x0Fu);
}

uint8_t cb_heartbeat_control_mode(uint8_t b1)
{
    return (uint8_t)(b1 & 0x0Fu);
}

int cb_heartbeat_make_state_mode(uint8_t state, uint8_t control_mode)
{
    if (state > 0x0Fu) return -1;       /* 4 bit 装不下：固件会静默截断 */
    return (int)(((unsigned)state << 4) | (unsigned)(control_mode & 0x0Fu));
}

int cb_heartbeat_life_is_next(uint8_t prev_life, uint8_t cur_life)
{
    return (((prev_life + 1u) & 0x07u) == (cur_life & 0x07u)) ? 1 : 0;
}

int cb_heartbeat_len_valid(size_t len)
{
    return (len == CB_HB_LEN_CLASSIC || len >= CB_HB_LEN_FD) ? 1 : 0;
}

/* ==========================================================================
 * 名称
 * ======================================================================== */

const char *cb_heartbeat_state_name(uint8_t state)
{
    /* 值与 Firmware/autogen/interfaces.hpp 的 AxisState 逐值对应 */
    switch (state) {
    case 0:  return "UNDEFINED";
    case 1:  return "IDLE";
    case 2:  return "STARTUP_SEQUENCE";
    case 3:  return "FULL_CALIBRATION_SEQUENCE";
    case 4:  return "MOTOR_CALIBRATION";
    case 5:  return "reserved";              /* 固件跳过 5 */
    case 6:  return "ENCODER_INDEX_SEARCH";
    case 7:  return "ENCODER_OFFSET_CALIBRATION";
    case 8:  return "CLOSED_LOOP_CONTROL";
    case 9:  return "LOCKIN_SPIN";
    case 10: return "ENCODER_DIR_FIND";
    case 11: return "HOMING";
    case 12: return "ENCODER_HALL_POLARITY_CALIBRATION";
    case 13: return "ENCODER_HALL_PHASE_CALIBRATION";
    case 14: return "INERTIA_CALIBRATION";
    case 15: return "ENCODER_LINEARIZATION";
    default:
        /* ⚠ 16 = MOTOR_DEADTIME_CALIBRATION **无法**出现在心跳里（4 bit 溢出，
           见 cb_heartbeat.h 文件头问题 1）——落到这里说明帧被伪造或固件已改。 */
        return "unrepresentable";
    }
}

const char *cb_heartbeat_control_mode_name(uint8_t control_mode)
{
    /* 名称取自固件 autogen/interfaces.hpp 的 ControlMode 枚举（去掉 CONTROL_MODE_ 前缀），
       而非参考工具 cyberbeast_tool.py 的短名（"VOLTAGE" 等）—— 后者是工具自造。 */
    switch (control_mode) {
    case CB_HB_CM_VOLTAGE:  return "VOLTAGE_CONTROL";
    case CB_HB_CM_TORQUE:   return "TORQUE_CONTROL";
    case CB_HB_CM_VELOCITY: return "VELOCITY_CONTROL";
    case CB_HB_CM_POSITION: return "POSITION_CONTROL";
    default:                return "unknown";
    }
}
