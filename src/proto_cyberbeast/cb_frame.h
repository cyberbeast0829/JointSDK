/**
 * @file    cb_frame.h
 * @brief   CYBERBEAST 帧层原语：CAN ID 编解码、字节序访问、寻址判定
 *
 * 本文件是协议层（WP2）的地基，**不含任何 I/O**，可独立单测。
 * 权威定义：`ODrive/Firmware/communication/can/can_cyberbeast.hpp`。
 *
 * CAN ID 布局（29-bit 扩展帧，帧内无协议头）：
 * @verbatim
 *   bit  28..26   25..18     17..10       9..2       1..0
 *       [Priority][MsgType][Dest/Group][Source  ][ Seq ]
 * @endverbatim
 */

#ifndef JSDK_CB_FRAME_H
#define JSDK_CB_FRAME_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 位域常量
 * ======================================================================== */

#define CB_PRI_SHIFT      26u
#define CB_MSGTYPE_SHIFT  18u
#define CB_DEST_SHIFT     10u
#define CB_SOURCE_SHIFT    2u
#define CB_SEQ_SHIFT       0u

#define CB_PRI_MASK       0x07u
#define CB_MSGTYPE_MASK   0xFFu
#define CB_DEST_MASK      0xFFu
#define CB_SOURCE_MASK    0xFFu
#define CB_SEQ_MASK       0x03u

#define CB_ID_MAX         0x1FFFFFFFu   /**< 29-bit 掩码 */

/** MsgType ≥ 此值为广播（Dest 解释为位图）。 */
#define CB_MSGTYPE_BROADCAST_THRESHOLD 0x80u

/** 广播地址：Dest = 0xFF 表示全局（所有节点，忽略 node_id）。 */
#define CB_ADDR_BROADCAST 0xFFu

/** 位掩码寻址与 MIT 广播槽位的设备数上限（node_id 1..7）。 */
#define CB_MAX_BROADCAST_DEVICES 8u

/** 主站默认源地址（设备在学到 master_id 之前把心跳发往该地址）。 */
#define CB_DEFAULT_MASTER_ID 1u

/** 节点 ID 有效范围。0 = 禁用（设备不回复、不动作）。 */
#define CB_NODE_ID_MIN 1u
#define CB_NODE_ID_MAX 254u

/* ==========================================================================
 * 优先级 / 消息类型（与固件枚举数值一致）
 * ======================================================================== */

typedef enum {
    CB_PRI_CRITICAL  = 0,   /**< 生命安全/硬件保护（ESTOP、过流故障） */
    CB_PRI_EMERGENCY = 1,
    CB_PRI_HIGH_CTRL = 2,   /**< 高频实时控制（MIT/POS/VEL/TORQUE） */
    CB_PRI_CTRL      = 3,
    CB_PRI_CONFIG    = 4,   /**< 参数读写、JSON 描述符 */
    CB_PRI_QUERY     = 5,   /**< 状态查询 0x40..0x47 */
    CB_PRI_STATUS    = 6,   /**< 周期上报（心跳） */
    CB_PRI_LOW       = 7
} cb_priority_t;

typedef enum {
    /* 点对点实时控制 */
    CB_MSG_MIT_CONTROL       = 0x00,
    CB_MSG_POS_CONTROL       = 0x01,
    CB_MSG_VEL_CONTROL       = 0x02,
    CB_MSG_TORQUE_CONTROL    = 0x03,
    CB_MSG_CURRENT_CONTROL   = 0x04,
    /* 点对点配置管理 */
    CB_MSG_PARAM_READ        = 0x20,
    CB_MSG_PARAM_WRITE       = 0x21,
    CB_MSG_CONFIG_SAVE       = 0x22,
    CB_MSG_CONFIG_RESET      = 0x23,
    CB_MSG_JSON_DESC_READ    = 0x24,
    CB_MSG_JSON_DESC_DATA    = 0x25,
    /* 点对点状态查询 */
    CB_MSG_QUERY_STATUS      = 0x40,
    CB_MSG_QUERY_POS_VEL     = 0x41,
    CB_MSG_QUERY_CURRENT     = 0x42,
    CB_MSG_QUERY_TEMPERATURE = 0x43,
    CB_MSG_QUERY_BUS         = 0x44,
    CB_MSG_QUERY_ERROR       = 0x45,
    CB_MSG_QUERY_DEVICE_INFO = 0x46,
    CB_MSG_QUERY_POWER       = 0x47,
    CB_MSG_HEARTBEAT         = 0x48,
    CB_MSG_STATUS_FEEDBACK   = 0x49,
    /* 点对点系统管理 */
    CB_MSG_SET_NODE_ID       = 0x60,
    CB_MSG_SET_ZERO          = 0x61,
    CB_MSG_START_MOTOR       = 0x62,
    CB_MSG_STOP_MOTOR        = 0x63,
    CB_MSG_RESET_DEVICE      = 0x64,
    CB_MSG_CLEAR_ERRORS      = 0x65,
    /* 广播实时控制 */
    CB_MSG_MIT_CONTROL_BCAST    = 0x80,
    CB_MSG_POS_CONTROL_BCAST    = 0x81,
    CB_MSG_VEL_CONTROL_BCAST    = 0x82,
    CB_MSG_TORQUE_CONTROL_BCAST = 0x83,
    /* 广播紧急/系统 */
    CB_MSG_ESTOP             = 0xC0,
    CB_MSG_FAULT_ALERT       = 0xC1
} cb_msgtype_t;

/* ==========================================================================
 * CAN ID 编解码
 * ======================================================================== */

/** 组装 29-bit ID（各字段越界部分按掩码截断）。 */
uint32_t cb_make_id(uint8_t priority, uint8_t msgtype, uint8_t dest,
                    uint8_t source, uint8_t seq);

uint8_t cb_id_priority(uint32_t id);
uint8_t cb_id_msgtype (uint32_t id);
uint8_t cb_id_dest    (uint32_t id);
uint8_t cb_id_source  (uint32_t id);
uint8_t cb_id_seq     (uint32_t id);

/** MsgType ≥ 0x80 → 1（Dest 是位图而非节点 ID）。 */
int cb_id_is_broadcast(uint32_t id);

/** 广播序列号滚动（模 4）。 */
uint8_t cb_seq_next(uint8_t seq);

/**
 * 该帧是否发给我（等价固件 `is_message_for_me`）。
 *
 * 规则：
 *   - node_id == 0 → 永远不是（设备被禁用）；
 *   - 广播 + Dest == 0xFF → 是（全局广播对所有 node_id 有效）；
 *   - 广播 + 位图 → 仅 **node_id 1..7** 可被寻址（固件对 node_id ≥ 8 直接返回 0）；
 *   - 单播 → Dest == node_id。
 */
int cb_id_is_for_me(uint8_t my_node_id, uint32_t id);

/**
 * 组装给某个节点的位图广播 Dest。
 * @return 位图；若 node_ids 含 ≥ 8 的节点则返回 -1（该调用不可用位掩码寻址）。
 */
int cb_make_broadcast_mask(const uint8_t *node_ids, unsigned count);

/* ==========================================================================
 * 字节序访问
 * ------------------------------------------------------------------------
 * 全协议 Big-Endian；**唯一例外**是 JSON 描述符（offset/len/crc/chunkOffset）为 LE。
 * 所有访问都是字节级的，**不要求指针对齐**。
 * ======================================================================== */

uint16_t cb_be_get_u16(const uint8_t *b);
int16_t  cb_be_get_i16(const uint8_t *b);
uint32_t cb_be_get_u32(const uint8_t *b);
int32_t  cb_be_get_i32(const uint8_t *b);
uint64_t cb_be_get_u64(const uint8_t *b);
float    cb_be_get_f32(const uint8_t *b);

void cb_be_put_u16(uint8_t *b, uint16_t v);
void cb_be_put_i16(uint8_t *b, int16_t v);
void cb_be_put_u32(uint8_t *b, uint32_t v);
void cb_be_put_i32(uint8_t *b, int32_t v);
void cb_be_put_u64(uint8_t *b, uint64_t v);
void cb_be_put_f32(uint8_t *b, float v);

uint16_t cb_le_get_u16(const uint8_t *b);
uint32_t cb_le_get_u32(const uint8_t *b);
void     cb_le_put_u16(uint8_t *b, uint16_t v);
void     cb_le_put_u32(uint8_t *b, uint32_t v);

/**
 * 平台自检：验证 float 为 IEEE-754 单精度且字节序访问正确。
 * @return 0 = 正常；-1 = 平台不满足前提（此时 SDK 的浮点编解码不可用）。
 * @note 建议在 jsdk_context_init() 里调用一次。
 */
int cb_frame_selfcheck(void);

/* ==========================================================================
 * 工具
 * ======================================================================== */

/** MsgType 名称（静态字符串，用于日志/CLI）；未知返回 "unknown"。 */
const char *cb_msgtype_name(uint8_t msgtype);
const char *cb_priority_name(uint8_t priority);

#ifdef __cplusplus
}
#endif

#endif /* JSDK_CB_FRAME_H */
