/**
 * @file    cb_query.c
 * @brief   CYBERBEAST 状态查询帧编解码实现（见 cb_query.h）
 *
 * 与固件 can_cyberbeast.cpp 的 cmd_query_* 系列逐字节对齐。
 */

#include "cb_query.h"
#include "cb_frame.h"

/* ==========================================================================
 * 请求
 * ======================================================================== */

int cb_query_request_len(uint8_t msgtype)
{
    switch (msgtype) {
    case CB_MSG_QUERY_STATUS:
    case CB_MSG_QUERY_POS_VEL:
    case CB_MSG_QUERY_CURRENT:
    case CB_MSG_QUERY_TEMPERATURE:
    case CB_MSG_QUERY_BUS:
    case CB_MSG_QUERY_DEVICE_INFO:
    case CB_MSG_QUERY_POWER:
        return 0;                       /* 空载荷请求 */
    case CB_MSG_QUERY_ERROR:
        return 1;                       /* [ErrorType] */
    default:
        return -1;
    }
}

size_t cb_query_build_request(uint8_t *dst, size_t cap, uint8_t msgtype,
                              uint8_t err_type)
{
    int need = cb_query_request_len(msgtype);

    if (need < 0) return 0u;
    if (need == 0) return 0u;           /* 合法空载荷 */

    if (!dst || cap < (size_t)need) return 0u;
    dst[0] = err_type;
    return (size_t)need;
}

/* ==========================================================================
 * 响应长度
 * ======================================================================== */

size_t cb_query_response_len(uint8_t msgtype, int classic)
{
    switch (msgtype) {
    case CB_MSG_QUERY_STATUS:           /* 响应就是 MIT 响应帧 */
        return CB_QUERY_MIT_RESP_LEN;
    case CB_MSG_QUERY_POS_VEL:
    case CB_MSG_QUERY_CURRENT:
    case CB_MSG_QUERY_TEMPERATURE:
    case CB_MSG_QUERY_BUS:
        return CB_QUERY_F32X2_LEN;
    case CB_MSG_QUERY_ERROR:
        return CB_QUERY_ERROR_LEN;
    case CB_MSG_QUERY_DEVICE_INFO:
        return classic ? CB_QUERY_DEVLEN_CLASSIC : CB_QUERY_DEVLEN_FD;
    case CB_MSG_QUERY_POWER:
        return CB_QUERY_F32X2_LEN;
    default:
        return 0u;
    }
}

/* ==========================================================================
 * 解码：统一的 “两个 f32 BE” 形状
 * ======================================================================== */

static int decode_f32x2(const uint8_t *src, size_t len, float *a, float *b)
{
    if (!src || len < CB_QUERY_F32X2_LEN) return -1;
    if (a) *a = cb_be_get_f32(src);
    if (b) *b = cb_be_get_f32(src + 4);
    return 0;
}

int cb_query_decode_pos_vel(const uint8_t *src, size_t len, cb_query_pos_vel_t *out)
{
    float a = 0.0f, b = 0.0f;
    if (decode_f32x2(src, len, &a, &b) != 0) return -1;
    if (out) { out->pos_turns = a; out->vel_turns_per_s = b; }
    return 0;
}

int cb_query_decode_current(const uint8_t *src, size_t len, cb_query_current_t *out)
{
    float a = 0.0f, b = 0.0f;
    if (decode_f32x2(src, len, &a, &b) != 0) return -1;
    if (out) { out->iq_a = a; out->id_a = b; }
    return 0;
}

int cb_query_decode_temp(const uint8_t *src, size_t len, cb_query_temp_t *out)
{
    float a = 0.0f, b = 0.0f;
    if (decode_f32x2(src, len, &a, &b) != 0) return -1;
    if (out) { out->motor_c = a; out->fet_c = b; }
    return 0;
}

int cb_query_decode_bus(const uint8_t *src, size_t len, cb_query_bus_t *out)
{
    float a = 0.0f, b = 0.0f;
    if (decode_f32x2(src, len, &a, &b) != 0) return -1;
    if (out) { out->vbus_v = a; out->ibus_a = b; }
    return 0;
}

int cb_query_decode_power(const uint8_t *src, size_t len, cb_query_power_t *out)
{
    float a = 0.0f, b = 0.0f;
    if (decode_f32x2(src, len, &a, &b) != 0) return -1;
    if (out) { out->elec_w = a; out->mech_w = b; }
    return 0;
}

/* ==========================================================================
 * 0x45 QUERY_ERROR
 *   b0 = ErrorType 回显 · b1..b3 保留 · b4..b7 = u32 BE 错误位图
 * ======================================================================== */

int cb_query_decode_error(const uint8_t *src, size_t len, cb_query_error_t *out)
{
    if (!src || len < CB_QUERY_ERROR_LEN) return -1;
    if (out) {
        out->err_type  = src[0];
        out->err_value = cb_be_get_u32(src + 4);
    }
    return 0;
}

size_t cb_query_encode_error(uint8_t *dst, const cb_query_error_t *v)
{
    if (!dst || !v) return 0u;
    dst[0] = v->err_type;
    dst[1] = 0u;
    dst[2] = 0u;
    dst[3] = 0u;
    cb_be_put_u32(dst + 4, v->err_value);
    return CB_QUERY_ERROR_LEN;
}

/* ==========================================================================
 * 0x46 QUERY_DEVICE_INFO
 *   Classic 8 B : hw u32 BE + fw u32 BE
 *   FD     16 B : 再追加 serial u64 BE
 * ======================================================================== */

int cb_query_decode_device(const uint8_t *src, size_t len,
                           cb_query_device_info_t *out)
{
    if (!src || len < CB_QUERY_DEVLEN_CLASSIC) return -1;

    if (out) {
        out->hw_ver = cb_be_get_u32(src);
        out->fw_ver = cb_be_get_u32(src + 4);
        if (len >= CB_QUERY_DEVLEN_FD) {
            out->serial     = cb_be_get_u64(src + 8);
            out->has_serial = 1;
        } else {
            out->serial     = 0u;
            out->has_serial = 0;        /* Classic：sn 需另用 PARAM_READ 读 */
        }
    }
    return 0;
}

size_t cb_query_encode_device(uint8_t *dst, const cb_query_device_info_t *v,
                              int classic)
{
    if (!dst || !v) return 0u;

    cb_be_put_u32(dst, v->hw_ver);
    cb_be_put_u32(dst + 4, v->fw_ver);

    if (classic) return CB_QUERY_DEVLEN_CLASSIC;

    cb_be_put_u64(dst + 8, v->serial);
    return CB_QUERY_DEVLEN_FD;
}

/* ==========================================================================
 * 编码（其余 f32×2 形状）
 * ======================================================================== */

static size_t encode_f32x2(uint8_t *dst, float a, float b)
{
    if (!dst) return 0u;
    cb_be_put_f32(dst, a);
    cb_be_put_f32(dst + 4, b);
    return CB_QUERY_F32X2_LEN;
}

size_t cb_query_encode_pos_vel(uint8_t *dst, const cb_query_pos_vel_t *v)
{
    if (!v) return 0u;
    return encode_f32x2(dst, v->pos_turns, v->vel_turns_per_s);
}

size_t cb_query_encode_current(uint8_t *dst, const cb_query_current_t *v)
{
    if (!v) return 0u;
    return encode_f32x2(dst, v->iq_a, v->id_a);
}

size_t cb_query_encode_temp(uint8_t *dst, const cb_query_temp_t *v)
{
    if (!v) return 0u;
    return encode_f32x2(dst, v->motor_c, v->fet_c);
}

size_t cb_query_encode_bus(uint8_t *dst, const cb_query_bus_t *v)
{
    if (!v) return 0u;
    return encode_f32x2(dst, v->vbus_v, v->ibus_a);
}

size_t cb_query_encode_power(uint8_t *dst, const cb_query_power_t *v)
{
    if (!v) return 0u;
    return encode_f32x2(dst, v->elec_w, v->mech_w);
}

/* ==========================================================================
 * 名称
 * ======================================================================== */

const char *cb_error_type_name(uint8_t err_type)
{
    switch (err_type) {
    case CB_ET_MOTOR:      return "MOTOR";
    case CB_ET_ENCODER:    return "ENCODER";
    case CB_ET_SENSORLESS: return "SENSORLESS";
    case CB_ET_CONTROLLER: return "CONTROLLER";
    case CB_ET_SYSTEM:     return "SYSTEM";
    case CB_ET_AXIS:       return "AXIS";
    default:               return "unknown";
    }
}
