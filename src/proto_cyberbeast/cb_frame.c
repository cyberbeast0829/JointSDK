/**
 * @file    cb_frame.c
 * @brief   CYBERBEAST 帧层原语实现（见 cb_frame.h）
 */

#include "cb_frame.h"

#include <string.h>

/* 浮点编码依赖 IEEE-754 单精度 4 字节；不满足则编译期失败。 */
typedef char cb_assert_float_is_4_bytes[(sizeof(float) == 4u) ? 1 : -1];
typedef char cb_assert_u64_is_8_bytes[(sizeof(uint64_t) == 8u) ? 1 : -1];

/* ==========================================================================
 * CAN ID
 * ======================================================================== */

uint32_t cb_make_id(uint8_t priority, uint8_t msgtype, uint8_t dest,
                    uint8_t source, uint8_t seq)
{
    return ((uint32_t)(priority & CB_PRI_MASK)          << CB_PRI_SHIFT)
         | ((uint32_t)(msgtype)                         << CB_MSGTYPE_SHIFT)
         | ((uint32_t)(dest & CB_DEST_MASK)             << CB_DEST_SHIFT)
         | ((uint32_t)(source & CB_SOURCE_MASK)         << CB_SOURCE_SHIFT)
         | ((uint32_t)(seq & CB_SEQ_MASK)               << CB_SEQ_SHIFT);
}

uint8_t cb_id_priority(uint32_t id) { return (uint8_t)((id >> CB_PRI_SHIFT)     & CB_PRI_MASK); }
uint8_t cb_id_msgtype (uint32_t id) { return (uint8_t)((id >> CB_MSGTYPE_SHIFT) & CB_MSGTYPE_MASK); }
uint8_t cb_id_dest    (uint32_t id) { return (uint8_t)((id >> CB_DEST_SHIFT)    & CB_DEST_MASK); }
uint8_t cb_id_source  (uint32_t id) { return (uint8_t)((id >> CB_SOURCE_SHIFT)  & CB_SOURCE_MASK); }
uint8_t cb_id_seq     (uint32_t id) { return (uint8_t)((id >> CB_SEQ_SHIFT)     & CB_SEQ_MASK); }

int cb_id_is_broadcast(uint32_t id)
{
    return cb_id_msgtype(id) >= CB_MSGTYPE_BROADCAST_THRESHOLD;
}

uint8_t cb_seq_next(uint8_t seq)
{
    return (uint8_t)((seq + 1u) & CB_SEQ_MASK);
}

int cb_id_is_for_me(uint8_t my_node_id, uint32_t id)
{
    uint8_t dest = cb_id_dest(id);

    if (my_node_id == 0u) return 0;                   /* 节点被禁用 */

    if (cb_id_is_broadcast(id)) {
        if (dest == CB_ADDR_BROADCAST) return 1;      /* 全局广播 */
        if (my_node_id >= CB_MAX_BROADCAST_DEVICES) return 0;  /* ⚠ ≥8 不可位寻址 */
        return (dest & (1u << my_node_id)) ? 1 : 0;
    }
    return (dest == my_node_id) ? 1 : 0;
}

int cb_make_broadcast_mask(const uint8_t *node_ids, unsigned count)
{
    unsigned i;
    int mask = 0;

    if (!node_ids || count == 0u) return -1;

    for (i = 0; i < count; ++i) {
        uint8_t n = node_ids[i];
        if (n == 0u || n >= CB_MAX_BROADCAST_DEVICES) return -1;  /* 不可位掩码寻址 */
        mask |= (int)(1u << n);
    }
    return mask;
}

/* ==========================================================================
 * 字节序访问
 * ------------------------------------------------------------------------
 * 全部逐字节处理：不要求指针对齐，也不依赖平台字节序。
 * ======================================================================== */

uint16_t cb_be_get_u16(const uint8_t *b)
{
    return (uint16_t)(((uint16_t)b[0] << 8) | (uint16_t)b[1]);
}

int16_t cb_be_get_i16(const uint8_t *b)
{
    /* 显式二补数解释：避免“无符号转有符号”的实现定义行为 */
    uint16_t u = cb_be_get_u16(b);
    return (u & 0x8000u) ? (int16_t)((int32_t)u - 0x10000) : (int16_t)u;
}

uint32_t cb_be_get_u32(const uint8_t *b)
{
    return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16)
         | ((uint32_t)b[2] <<  8) | (uint32_t)b[3];
}

int32_t cb_be_get_i32(const uint8_t *b)
{
    uint32_t u = cb_be_get_u32(b);
    if (u & 0x80000000u) {
        return (int32_t)((int64_t)u - 0x100000000LL);
    }
    return (int32_t)u;
}

uint64_t cb_be_get_u64(const uint8_t *b)
{
    uint64_t hi = (uint64_t)cb_be_get_u32(b);
    uint64_t lo = (uint64_t)cb_be_get_u32(b + 4);
    return (hi << 32) | lo;
}

void cb_be_put_u16(uint8_t *b, uint16_t v)
{
    b[0] = (uint8_t)(v >> 8);
    b[1] = (uint8_t)(v);
}

void cb_be_put_i16(uint8_t *b, int16_t v)
{
    cb_be_put_u16(b, (uint16_t)v);
}

void cb_be_put_u32(uint8_t *b, uint32_t v)
{
    b[0] = (uint8_t)(v >> 24);
    b[1] = (uint8_t)(v >> 16);
    b[2] = (uint8_t)(v >>  8);
    b[3] = (uint8_t)(v);
}

void cb_be_put_i32(uint8_t *b, int32_t v)
{
    cb_be_put_u32(b, (uint32_t)v);      /* 负数转无符号是良定义的（模 2^32） */
}

void cb_be_put_u64(uint8_t *b, uint64_t v)
{
    cb_be_put_u32(b, (uint32_t)(v >> 32));
    cb_be_put_u32(b + 4, (uint32_t)(v & 0xFFFFFFFFu));
}

float cb_be_get_f32(const uint8_t *b)
{
    uint32_t w = cb_be_get_u32(b);
    float f;
    memcpy(&f, &w, sizeof f);      /* 位模式搬移，避免对齐/别名问题 */
    return f;
}

void cb_be_put_f32(uint8_t *b, float v)
{
    uint32_t w;
    memcpy(&w, &v, sizeof w);
    cb_be_put_u32(b, w);
}

/* --------------------------------------------------------------------------
 * 小端存取器：**只用于参数值** —— 理由见 cb_frame.h（固件 memcpy 主机序）。
 * ------------------------------------------------------------------------ */

uint16_t cb_le_get_u16(const uint8_t *b);   /* 定义在文件后面（原有实现） */

int16_t cb_le_get_i16(const uint8_t *b)
{
    return (int16_t)cb_le_get_u16(b);
}

uint32_t cb_le_get_u32(const uint8_t *b);   /* 定义在文件后面（原有实现） */

int32_t cb_le_get_i32(const uint8_t *b)
{
    return (int32_t)cb_le_get_u32(b);
}

uint64_t cb_le_get_u64(const uint8_t *b)
{
    return (uint64_t)cb_le_get_u32(b)
         | ((uint64_t)cb_le_get_u32(b + 4) << 32);
}

float cb_le_get_f32(const uint8_t *b)
{
    uint32_t w = cb_le_get_u32(b);
    float    f;
    memcpy(&f, &w, sizeof f);      /* 位模式搬移，避免对齐/别名问题 */
    return f;
}

void cb_le_put_u16(uint8_t *b, uint16_t v);  /* 定义在文件后面（原有实现） */

void cb_le_put_i16(uint8_t *b, int16_t v)
{
    cb_le_put_u16(b, (uint16_t)v);
}

void cb_le_put_u32(uint8_t *b, uint32_t v);  /* 定义在文件后面（原有实现） */

void cb_le_put_i32(uint8_t *b, int32_t v)
{
    cb_le_put_u32(b, (uint32_t)v);
}

void cb_le_put_u64(uint8_t *b, uint64_t v)
{
    cb_le_put_u32(b, (uint32_t)(v));
    cb_le_put_u32(b + 4, (uint32_t)(v >> 32));
}

void cb_le_put_f32(uint8_t *b, float v)
{
    uint32_t w;
    memcpy(&w, &v, sizeof w);
    cb_le_put_u32(b, w);
}

uint16_t cb_le_get_u16(const uint8_t *b)
{
    return (uint16_t)(((uint16_t)b[1] << 8) | (uint16_t)b[0]);
}

uint32_t cb_le_get_u32(const uint8_t *b)
{
    return ((uint32_t)b[3] << 24) | ((uint32_t)b[2] << 16)
         | ((uint32_t)b[1] <<  8) | (uint32_t)b[0];
}

void cb_le_put_u16(uint8_t *b, uint16_t v)
{
    b[0] = (uint8_t)(v);
    b[1] = (uint8_t)(v >> 8);
}

void cb_le_put_u32(uint8_t *b, uint32_t v)
{
    b[0] = (uint8_t)(v);
    b[1] = (uint8_t)(v >>  8);
    b[2] = (uint8_t)(v >> 16);
    b[3] = (uint8_t)(v >> 24);
}

/* ==========================================================================
 * 平台自检
 * ======================================================================== */

int cb_frame_selfcheck(void)
{
    /* 1.0f 的 IEEE-754 单精度位模式必须是 0x3F800000 */
    uint32_t one_bits;
    float one = 1.0f;

    memcpy(&one_bits, &one, sizeof one_bits);
    if (one_bits != 0x3F800000u) return -1;

    /* 往返检查：-12.5f 的 BE 字节应为 C1 48 00 00 */
    {
        uint8_t b[4];
        cb_be_put_f32(b, -12.5f);
        if (b[0] != 0xC1u || b[1] != 0x48u || b[2] != 0x00u || b[3] != 0x00u) return -1;
        if (cb_be_get_f32(b) != -12.5f) return -1;
    }

    /* 整型往返 */
    {
        uint8_t b[8];
        cb_be_put_u64(b, 0x0102030405060708ull);
        if (b[0] != 0x01u || b[7] != 0x08u) return -1;
        if (cb_be_get_u64(b) != 0x0102030405060708ull) return -1;
        cb_le_put_u32(b, 0x01020304u);
        if (b[0] != 0x04u || b[3] != 0x01u) return -1;
        if (cb_le_get_u32(b) != 0x01020304u) return -1;
    }
    return 0;
}

/* ==========================================================================
 * 文本
 * ======================================================================== */

const char *cb_msgtype_name(uint8_t msgtype)
{
    switch (msgtype) {
    case CB_MSG_MIT_CONTROL:       return "MIT_CONTROL";
    case CB_MSG_POS_CONTROL:       return "POS_CONTROL";
    case CB_MSG_VEL_CONTROL:       return "VEL_CONTROL";
    case CB_MSG_TORQUE_CONTROL:    return "TORQUE_CONTROL";
    case CB_MSG_CURRENT_CONTROL:   return "CURRENT_CONTROL";
    case CB_MSG_PARAM_READ:        return "PARAM_READ";
    case CB_MSG_PARAM_WRITE:       return "PARAM_WRITE";
    case CB_MSG_CONFIG_SAVE:       return "CONFIG_SAVE";
    case CB_MSG_CONFIG_RESET:      return "CONFIG_RESET";
    case CB_MSG_JSON_DESC_READ:    return "JSON_DESC_READ";
    case CB_MSG_JSON_DESC_DATA:    return "JSON_DESC_DATA";
    case CB_MSG_QUERY_STATUS:      return "QUERY_STATUS";
    case CB_MSG_QUERY_POS_VEL:     return "QUERY_POS_VEL";
    case CB_MSG_QUERY_CURRENT:     return "QUERY_CURRENT";
    case CB_MSG_QUERY_TEMPERATURE: return "QUERY_TEMPERATURE";
    case CB_MSG_QUERY_BUS:         return "QUERY_BUS";
    case CB_MSG_QUERY_ERROR:       return "QUERY_ERROR";
    case CB_MSG_QUERY_DEVICE_INFO: return "QUERY_DEVICE_INFO";
    case CB_MSG_QUERY_POWER:       return "QUERY_POWER";
    case CB_MSG_HEARTBEAT:         return "HEARTBEAT";
    case CB_MSG_STATUS_FEEDBACK:   return "STATUS_FEEDBACK";
    case CB_MSG_SET_NODE_ID:       return "SET_NODE_ID";
    case CB_MSG_SET_ZERO:          return "SET_ZERO";
    case CB_MSG_START_MOTOR:       return "START_MOTOR";
    case CB_MSG_STOP_MOTOR:        return "STOP_MOTOR";
    case CB_MSG_RESET_DEVICE:      return "RESET_DEVICE";
    case CB_MSG_CLEAR_ERRORS:      return "CLEAR_ERRORS";
    case CB_MSG_MIT_CONTROL_BCAST: return "MIT_CONTROL_BCAST";
    case CB_MSG_POS_CONTROL_BCAST: return "POS_CONTROL_BCAST";
    case CB_MSG_VEL_CONTROL_BCAST: return "VEL_CONTROL_BCAST";
    case CB_MSG_TORQUE_CONTROL_BCAST: return "TORQUE_CONTROL_BCAST";
    case CB_MSG_ESTOP:             return "ESTOP";
    case CB_MSG_FAULT_ALERT:       return "FAULT_ALERT";
    default:                       return "unknown";
    }
}

const char *cb_priority_name(uint8_t priority)
{
    switch (priority) {
    case CB_PRI_CRITICAL:  return "CRITICAL";
    case CB_PRI_EMERGENCY: return "EMERGENCY";
    case CB_PRI_HIGH_CTRL: return "HIGH_CTRL";
    case CB_PRI_CTRL:      return "CTRL";
    case CB_PRI_CONFIG:    return "CONFIG";
    case CB_PRI_QUERY:     return "QUERY";
    case CB_PRI_STATUS:    return "STATUS";
    case CB_PRI_LOW:       return "LOW";
    default:               return "unknown";
    }
}
