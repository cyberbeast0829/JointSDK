/* ==========================================================================
 * jsdk_can_amalgam.c — CyberBeast CAN/CAN-FD 关节 SDK 的单文件实现
 *
 * ⚠ **本文件由 tools/amalgamate.py 生成，请勿手工修改**（改代码请改 src/ 下原件后重跑）。
 *
 * 合并选项（生成时确定，改选项要重新生成并**重新编译**）：
 *   heap     未包含
 *   virtual  未包含
 *   slcan    未包含
 *
 * 合并进来的文件：
 *   - src/jsdk_internal.h
 *   - src/proto_cyberbeast/cb_frame.h
 *   - src/proto_cyberbeast/cb_mit.h
 *   - src/proto_cyberbeast/cb_ctrl.h
 *   - src/proto_cyberbeast/cb_query.h
 *   - src/proto_cyberbeast/cb_heartbeat.h
 *   - src/proto_cyberbeast/cb_param.h
 *   - src/proto_cyberbeast/cb_jsondesc_fetch.h
 *   - src/proto_cyberbeast/cb_desc_cache.h
 *   - src/jsdk_core_internal.h
 *   - src/hal/hal_handle.h
 *   - src/hal/hal_slcan_codec.h
 *   - src/hal/sim_device.h
 *   - src/hal/hal_virtual_internal.h
 *   - src/proto_cyberbeast/cb_ctrl.c
 *   - src/proto_cyberbeast/cb_desc_cache.c
 *   - src/proto_cyberbeast/cb_frame.c
 *   - src/proto_cyberbeast/cb_heartbeat.c
 *   - src/proto_cyberbeast/cb_jsondesc_fetch.c
 *   - src/proto_cyberbeast/cb_jsondesc_parse.c
 *   - src/proto_cyberbeast/cb_mit.c
 *   - src/proto_cyberbeast/cb_param.c
 *   - src/proto_cyberbeast/cb_query.c
 *   - src/core/jsdk_config.c
 *   - src/core/jsdk_context.c
 *   - src/core/jsdk_desc.c
 *   - src/core/jsdk_fault.c
 *   - src/core/jsdk_group.c
 *   - src/core/jsdk_joint.c
 *   - src/core/jsdk_ops.c
 *   - src/core/jsdk_param.c
 *   - src/core/jsdk_text.c
 *   - src/core/jsdk_units.c
 *   - src/core/jsdk_watchdog.c
 * ========================================================================== */

#include "jsdk_can_amalgam.h"

#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


/* ======================== src/jsdk_internal.h ======================== */
/**
 * @file    jsdk_internal.h
 * @brief   Joint SDK 内部接口（**不安装、不对外**）
 *
 * 本头文件只被 src/ 下的实现与 tests/ 使用。对外 ABI 见 joint_sdk.h。
 */

#ifndef JSDK_INTERNAL_H
#define JSDK_INTERNAL_H


/* ==========================================================================
 * 编译期上限
 * ======================================================================== */

/**
 * JSON 括号嵌套深度上限。
 * 实测（v8 描述符）最大嵌套 = **10**：最深端点为
 *   axis0.motor.motor_thermistor.config.temp_limit_upper
 * 每层 = 1 个 object + 1 个 members 数组，故帧深度 = 1(根数组) + 2×4 = 9。
 * 取 16 留出余量（固件新增一层嵌套仍可解析）。
 */
#define JSDK_JSON_MAX_DEPTH      16u

/** filter_paths 条数上限（受 filter_hit 位图宽度限制，见 jsdk_desc_config_t）。 */
#define JSDK_DESC_MAX_FILTERS    64u

/** 单条路径长度硬上限（实测最长 68，默认 max_path_len = 128）。 */
#define JSDK_EP_PATH_MAX_HARD    160u

/** JSON 键名缓冲上限（最长键为 "endpoint_ref" / "members"）。 */
#define JSDK_EP_KEY_MAX          16u

/** type / access 字段值缓冲上限（最长 "endpoint_ref" = 12）。 */
#define JSDK_EP_VAL_MAX          16u

/** RETAIN_ALL 下的保守推荐 arena 大小（无法预知端点数，见 jsdk_desc_arena_size）。 */
#define JSDK_DESC_ARENA_RECOMMEND_ALL   32768u

/* ==========================================================================
 * 端点存储（arena）
 * ======================================================================== */

/** arena 条目：8 字节。条目区从 arena 头部向后增长，路径池从尾部向前增长。 */
typedef struct {
    uint32_t path_off;   /**< 相对 arena 基址的偏移，指向 NUL 结尾的路径 */
    uint16_t ep_id;
    uint8_t  type;       /**< jsdk_ep_type_t */
    uint8_t  access;     /**< JSDK_EP_ACCESS_* 位掩码 */
} jsdk_ep_entry_t;

typedef struct {
    uint8_t *base;         /**< 调用者提供的 arena 基址 */
    size_t   size;         /**< arena 字节数 */
    uint32_t entry_count;  /**< 已写入的条目数 */
    size_t   blob_top;     /**< 路径池当前顶部（向下增长） */
    size_t   blob_used;    /**< 路径池已用字节 */
} jsdk_arena_t;

typedef struct {
    jsdk_arena_t arena;
    unsigned     parsed_total;   /**< 解析到的叶子总数（含未保留） */
    unsigned     max_endpoints;
} jsdk_ep_store_t;

/* ==========================================================================
 * 过滤器
 * ======================================================================== */

typedef struct {
    const char *const *paths;
    unsigned           count;
    uint64_t           hit;      /**< 第 i 条是否已命中（count ≤ 64） */
} jsdk_desc_filter_t;

/* ==========================================================================
 * 增量 JSON 解析器
 * ------------------------------------------------------------------------
 * 设计要点（见 docs/PROTOCOL_NOTES.zh-CN.md §9.2）：
 *   - 逐字节状态机，无递归、无 malloc，不需要缓存完整 41 KB；
 *   - 路径在解析 name 字段时**增量拼接到 cur_base**，因此不需要按层保存
 *     name 副本（key 与 id/type 的先后顺序无关）；
 *   - 叶子判定：对象闭合时同时见过 id 与 type。
 * ======================================================================== */

typedef struct {
    uint16_t restore_len;  /**< 弹出时把 cur_base 截断回的长度 */
    uint32_t id;
    uint8_t  have_id;
    uint8_t  type;         /**< jsdk_ep_type_t */
    uint8_t  have_type;
    uint8_t  access;
    uint8_t  have_access;
    uint8_t  have_name;    /**< 见过 name 字段（含空串） */
    uint8_t  name_appended;/**< 名字已追加到 cur_base（空名字时为 0） */
    uint8_t  is_array;
    uint8_t  cur_key;      /**< 当前对象的 key（见 JSDK_KEY_*） */
} jsdk_json_frame_t;

enum {
    JSDK_KEY_NONE = 0,
    JSDK_KEY_NAME,
    JSDK_KEY_ID,
    JSDK_KEY_TYPE,
    JSDK_KEY_ACCESS,
    JSDK_KEY_OTHER
};

enum {
    JSDK_JS_BEGIN = 0,
    JSDK_JS_EXPECT,      /**< 期待 key（对象内）或 value（数组内）或闭合符 */
    JSDK_JS_IN_KEY,
    JSDK_JS_AFTER_KEY,
    JSDK_JS_IN_STR,      /**< 字符串值 */
    JSDK_JS_IN_NUM,
    JSDK_JS_IN_LIT,
    JSDK_JS_AFTER_VAL,
    JSDK_JS_DONE,
    JSDK_JS_FAIL
};

typedef struct jsdk_jsondesc {
    /* 配置（不拥有） */
    const jsdk_desc_config_t *cfg;
    jsdk_ep_store_t          *store;

    jsdk_desc_filter_t filter;
    uint32_t           bytes_fed;
    uint16_t           max_path_len;

    /* 状态机 */
    uint8_t  state;
    uint8_t  depth;
    uint8_t  retain_all;
    uint8_t  esc;
    uint8_t  cap;          /**< 当前字符串是否要捕获 */
    uint8_t  in_key;       /**< 当前字符串是键（而非值） */
    uint8_t  expect_key;   /**< 下一字符串是键；由 '{' / ',' 判定，与帧类型无关 */
    uint8_t  key_overflow;
    uint8_t  neg;
    uint32_t num;
    uint8_t  num_digits;

    uint16_t cur_base_len;
    char     cur_base[JSDK_EP_PATH_MAX_HARD];
    uint16_t key_len;
    char     key[JSDK_EP_KEY_MAX];
    uint16_t val_len;
    char     val[JSDK_EP_VAL_MAX];

    jsdk_json_frame_t f[JSDK_JSON_MAX_DEPTH + 1u];

    const char *err;       /**< 失败原因（静态字符串） */
    int         fail_code; /**< 粘性失败码：JSDK_ERR_PARSE 或 JSDK_ERR_NO_MEMORY */
} jsdk_jsondesc_t;

/* ==========================================================================
 * 解析器 API
 * ======================================================================== */

int  jsdk_jsondesc_init(jsdk_jsondesc_t *p, const jsdk_desc_config_t *cfg,
                        jsdk_ep_store_t *store);
int  jsdk_jsondesc_feed(jsdk_jsondesc_t *p, const void *data, size_t len);
int  jsdk_jsondesc_finish(jsdk_jsondesc_t *p);

/** 一次跑完（等价 init + feed + finish），用于测试与 desc_import_raw()。 */
int  jsdk_jsondesc_run(const jsdk_desc_config_t *cfg, jsdk_ep_store_t *store,
                       const void *json, size_t len);

/** 已命中的 filter 条数。 */
unsigned jsdk_jsondesc_filter_hits(const jsdk_jsondesc_t *p);

/** filter 是否已全部命中（用于 stop_when_satisfied）。 */
int jsdk_jsondesc_satisfied(const jsdk_jsondesc_t *p);

/** 最后一次失败原因（可读文本）；未失败返回 NULL。 */
const char *jsdk_jsondesc_error(const jsdk_jsondesc_t *p);

/* ==========================================================================
 * 端点存储 API
 * ======================================================================== */

int          jsdk_ep_arena_put(jsdk_arena_t *a, const char *path, size_t len,
                               uint16_t ep_id, uint8_t type, uint8_t access);
unsigned     jsdk_ep_store_count(const jsdk_ep_store_t *s);
const char  *jsdk_ep_store_path(const jsdk_ep_store_t *s, unsigned index);
int          jsdk_ep_store_at(const jsdk_ep_store_t *s, unsigned index,
                              const char **path, uint16_t *ep_id,
                              jsdk_ep_type_t *type, uint8_t *access);
int          jsdk_ep_store_lookup(const jsdk_ep_store_t *s, const char *path,
                                  uint16_t *ep_id, jsdk_ep_type_t *type,
                                  uint8_t *access);
void         jsdk_ep_store_reset(jsdk_ep_store_t *s);

/** 名称 → 类型枚举；未知返回 0。 */
uint8_t jsdk_ep_type_from_name(const char *name, size_t len);

/** 类型枚举 → 名称（静态字符串）；未知返回 "?"。 */
const char *jsdk_ep_type_name(jsdk_ep_type_t t);

/** 该类型是否可直接读写（JSON / FUNCTION / ENDPOINT_REF 为不透明类型）。 */
int jsdk_ep_type_is_scalar(jsdk_ep_type_t t);

/** 该类型在协议上的字节长度；不透明类型返回 0。 */
unsigned jsdk_ep_type_size(jsdk_ep_type_t t);

#endif /* JSDK_INTERNAL_H */

/* ======================== src/proto_cyberbeast/cb_frame.h ======================== */
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

/* ======================== src/proto_cyberbeast/cb_mit.h ======================== */
/**
 * @file    cb_mit.h
 * @brief   CYBERBEAST MIT 紧凑编解码（8 字节/设备，Big-Endian 位打包）
 *
 * 权威实现：`ODrive/Firmware/communication/can/can_cyberbeast.cpp`
 *           `pack_mit_command` / `unpack_mit_command` / `pack_mit_response`
 * 定点原语：`ODrive/Firmware/communication/can/can_simple.cpp`
 *           `float_to_uint` / `uint_to_float`
 *
 * @par 关键语义（与固件必须位级一致）
 * 固件的 `float_to_uint` 是
 * @code
 *   return (int)((x - x_min) * ((float)((1<<bits)-1)) / (x_max - x_min));
 * @endcode
 * 即 **C 截断取整**（不是四舍五入），且 **不做钳位**。
 * ⚠ 不钳位意味着越界输入会让定点值超出位宽，在打包时被掩码**回绕**，
 * 对端解出的将是完全不同的值（例如 +12.6 rad 可能变成接近 -12.5 rad）。
 * 因此本 SDK 的编码器**先钳位再截断**，并通过 `clamped` 位图回报，
 * 让上层能按 docs/DESIGN.zh-CN.md §6.10 的策略处理（默认拒绝并改发安全帧）。
 *
 * @note 参考的 Python 工具 `cyberbeast_tool.py` 用的是 `round()` + 钳位，
 *       与固件**差 ±1 LSB**，不能用作位级黄金向量（见 tools/gen_golden_vectors.py）。
 */

#ifndef JSDK_CB_MIT_H
#define JSDK_CB_MIT_H


#ifdef __cplusplus
extern "C" {
#endif

/** 单个 MIT 槽位的字节数。 */
#define CB_MIT_SLOT_BYTES 8u

/* ==========================================================================
 * 定点原语
 * ======================================================================== */

/**
 * 浮点 → 无符号定点。**完全复刻固件语义**：C 截断、不钳位、不防 NaN。
 * 仅供位级对拍与理解固件行为使用；编码器请用 cb_mit_f2u()。
 *
 * @note x_min == x_max（span 为 0）时返回 0（固件此处会除零，行为未定义）。
 */
int32_t cb_mit_f2u_raw(float x, float x_min, float x_max, int bits);

/**
 * 浮点 → 无符号定点，**先钳位到 [x_min, x_max] 再截断**。
 * NaN 与 Inf 视为越界，按边界钳位。这是编码器实际使用的版本。
 */
int32_t cb_mit_f2u(float x, float x_min, float x_max, int bits);

/** 无符号定点 → 浮点（与固件一致）。 */
float cb_mit_u2f(int32_t x_int, float x_min, float x_max, int bits);

/* ==========================================================================
 * 量程
 * ------------------------------------------------------------------------
 * 全部来自设备 JSON 描述符（v8 端点 335..339），**不得硬编码**。
 * 出厂默认值仅作参考：pos 12.5 / vel 65.0 / kp 500.0 / kd 5.0 / tau 50.0。
 * ======================================================================== */

typedef struct {
    float pos_max;   /**< 位置：±pos_max，单位 rad（输出端），端点 335 */
    float vel_max;   /**< 速度：±vel_max，单位 rad/s（输出端），端点 336 */
    float kp_max;    /**< Kp：0..kp_max（无符号），端点 338 */
    float kd_max;    /**< Kd：0..kd_max（无符号），端点 339 */
    float tau_max;   /**< 力矩：±tau_max，单位 N·m（输出端），端点 337 */
} cb_mit_range_t;

/** 量程是否可用（各最大值必须 > 0 且非 NaN）。 */
int cb_mit_range_valid(const cb_mit_range_t *r);

/* ==========================================================================
 * 命令帧编解码
 * ======================================================================== */

/* 钳位标志位（cb_mit_pack_command 的 clamped 输出）
 *
 * 分层语义（两者可同时置位，不会互相屏蔽）：
 *   CB_MIT_CLAMP_xxx — 该字段的输入值**被修改过**，原因是：
 *                       (a) 超出 [lo, hi] 量程 → 饱和到边界（±Inf 也走这条）；
 *                       (b) 该字段是 NaN → 按 0 处理。
 *   CB_MIT_INVALID   — 至少有一个字段的输入是 **NaN**（已按 0 处理）。
 *                       这是调用方代码 bug 的信号，比单纯钳位更严重。
 *
 * ⚠ 与固件的区别：固件的 float_to_uint 既不报错也不钳位，越界值会被掩码回绕
 *   （+12.625 rad 会变成 -12.38 rad）。SDK 必须钳位；本处的标志就是告知调用方
 *   “你给的命令已经被修正，实际下发值不等于请求值”。
 *
 * 用法：只关心“是否被修正” → `if (flags != 0)`；
 *       需定位具体字段 → 查对应 bit；
 *       只有 INVALID 置位才说明输入是 NaN（不是量程问题）。
 */
#define CB_MIT_CLAMP_POS   0x01u
#define CB_MIT_CLAMP_VEL   0x02u
#define CB_MIT_CLAMP_KP    0x04u
#define CB_MIT_CLAMP_KD    0x08u
#define CB_MIT_CLAMP_TAU   0x10u
#define CB_MIT_INVALID     0x80u  /**< 输入含 NaN，相关字段已按 0 处理（±Inf 不算） */

/**
 * 打包一条 MIT 命令到 8 字节。
 *
 * 广播时把整帧的 `slot * CB_MIT_SLOT_BYTES` 偏移处作为 @p dst8 传入
 * （槽位号 == node_id，见 docs/PROTOCOL_NOTES.zh-CN.md §4.1）。
 *
 * @param clamped 可选（可传 NULL）。输出钳位/无效标志位，见 CB_MIT_CLAMP_*。
 */
void cb_mit_pack_command(uint8_t *dst8, const cb_mit_range_t *range,
                         float pos, float vel, float kp, float kd, float tau,
                         uint8_t *clamped);

/** 解包一条 MIT 命令（8 字节），用于测试与诊断。 */
void cb_mit_unpack_command(const uint8_t *src8, const cb_mit_range_t *range,
                           float *pos, float *vel, float *kp, float *kd, float *tau);

/**
 * 广播帧所需长度：`(max_node_id_in_mask + 1) * 8`。
 *
 * @param max_slot_used  位掩码中**最大的 node_id**（不是“设备个数”）。
 * @return 字节数；若 `max_slot_used >= CB_MAX_BROADCAST_DEVICES` 返回 0（不可位寻址）。
 *
 * ⚠ 三个容易踩的坑（均来自固件 `mit_control_cmd`）：
 *   1. **FD 广播的槽位号就是 node_id**，即设备 1 读 `[8,16)`、设备 2 读 `[16,24)`……
 *      发送 7 台设备的广播必须凑够 **64 字节**（头 8 字节属于不存在的设备 0）。
 *   2. 固件对 `msg.len < (slot + 1) * 8` 的帧**整帧丢弃**，所以末尾空槽位不能省——
 *      必须用安全值（如 kp=kd=0、tau=0）填充，否则后段设备会静默不动。
 *   3. **Classic CAN 广播恒用 slot 0**，所有被掩码命中的设备执行**同一条**命令
 *      （长度只需 8）。想让各设备执行不同指令必须用 CAN FD。
 *      → 所以 Classic 广播下本函数无意义，直接用 8 即可。
 */
size_t cb_mit_bcast_frame_len(uint8_t max_slot_used);

/* ==========================================================================
 * 响应帧解码
 * ======================================================================== */

typedef enum {
    CB_ERR_NONE          = 0x0,
    CB_ERR_MOTOR         = 0x1,
    CB_ERR_ENCODER       = 0x2,
    CB_ERR_CONTROLLER    = 0x3,
    CB_ERR_UNDER_VOLTAGE = 0x4,   /**< ⚠ 过压（DC_BUS_OVER_VOLTAGE）也映射到此处 */
    CB_ERR_OVER_TEMP     = 0x5,
    CB_ERR_OVER_CURRENT  = 0x6,
    CB_ERR_STALL         = 0x7,
    CB_ERR_CAN_TIMEOUT   = 0x8,   /**< ⚠ ESTOP_REQUESTED 与 CAN_BUS_FAILED 都映射到此处 */
    CB_ERR_MULTIPLE      = 0xF
} cb_mit_error_t;

typedef enum {
    CB_MODE_RESET       = 0,
    CB_MODE_CALIBRATING = 1,
    CB_MODE_IDLE        = 2,
    CB_MODE_CLOSED_LOOP = 3,
    CB_MODE_MIT         = 4,
    CB_MODE_POSITION    = 5,
    CB_MODE_VELOCITY    = 6,
    CB_MODE_TORQUE      = 7
} cb_mit_mode_t;

typedef struct {
    float   pos;           /**< 输出端 rad */
    float   vel;           /**< 输出端 rad/s */
    float   current;       /**< 电机端 A（量程见 cb_mit_response_max_current） */
    uint8_t err_code;      /**< cb_mit_error_t */
    uint8_t mode;          /**< cb_mit_mode_t */
    int16_t motor_temp_c;  /**< −50..205 °C（⚠ 必须 16 位：205 超出 int8 范围） */
    int16_t mos_temp_c;
} cb_mit_response_t;

/**
 * 响应帧里 current 字段的满量程（A）。
 *
 * 固件：`max_current = mit_max_torque / torque_constant`，并钳到 80 A；
 * 若 `torque_constant <= 0.001` 则回退 40 A。
 *
 * @warning 必须传设备真实 `torque_constant`（v8 端点 247），否则电流/力矩全错。
 */
float cb_mit_response_max_current(float tau_max, float torque_constant);

/**
 * 解包 MIT 响应（8 字节）。
 *
 * 布局：`pos16 | vel12 | err4 | cur12 | mode4 | motor_temp u8(-50) | mos_temp u8(-50)`
 */
void cb_mit_unpack_response(const uint8_t *src8, const cb_mit_range_t *range,
                            float max_current, cb_mit_response_t *out);

/** 响应帧的最小安全解码：量程未知时使用（pos/vel 返回 0 并置 range_valid=0）。 */
void cb_mit_unpack_response_raw(const uint8_t *src8, cb_mit_response_t *out);

/**
 * 打包 MIT 响应（8 字节）。供**虚拟设备/上位机仿真**使用。
 *
 * @param range       用于 pos/vel 的对称量程（kp/kd 不参与响应）
 * @param max_current 电流满量程，用 cb_mit_response_max_current() 算
 * @return 写入字节数（恒为 8）；指针为空返回 0
 *
 * ⚠ **本函数故意与 cb_mit_pack_command 的数值语义不同**：
 *   - `pack_command` 是**主站**发命令，用四舍五入 + 钳位（SDK 的选择）；
 *   - `pack_response` 是**设备**产生上报，用**截断 + 钳位**，即固件
 *     `float_to_uint` 的行为（只额外补上固件缺的钳位）。
 *   这样才能逐字节重现真设备发出的帧，让上层解码路径得到真实训练。
 *   温度按固件的 `clamp(temp + 50, 0, 255)` 后截断。
 *
 * @note 错误码 / 模式只取低 4 位（线宽所限），与固件一致。
 */
size_t cb_mit_pack_response(uint8_t *dst8, const cb_mit_range_t *range,
                            const cb_mit_response_t *r, float max_current);

const char *cb_mit_error_name(uint8_t err_code);
const char *cb_mit_mode_name(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* JSDK_CB_MIT_H */

/* ======================== src/proto_cyberbeast/cb_ctrl.h ======================== */
/**
 * @file    cb_ctrl.h
 * @brief   CYBERBEAST 实时控制帧编解码（POS / VEL / TORQUE / CURRENT）
 *
 * **单位约定：本层一律使用“线上原生单位”，不做任何单位换算。**
 * 单位归一（deg→rad、RPM→rad/s、输出端↔电机端）属于 L3 关节逻辑的职责。
 * 这样 L2 可以被逐字节对拍，也能被虚拟设备直接复用。
 *
 * | 帧 | MsgType | 长度 | 线上单位 |
 * |---|---|---|---|
 * | POS_CONTROL | 0x01 / 0x81 | Classic 8B ・ FD 12B | **输出端**度、**输出端**RPM、**电机端**A |
 * | VEL_CONTROL | 0x02 / 0x82 | 8B（Classic/FD 同布局） | **输出端**RPM、**电机端**A |
 * | TORQUE_CONTROL | 0x03 / 0x83 | ≥4B | **电机端**N·m |
 * | CURRENT_CONTROL | 0x04 | ≥4B（**无广播版本**） | **电机端**A |
 *
 * ⚠ 两个容易踩的坑（均由固件源码核实）：
 *
 * 1. **力矩的方向端不一致**。`MIT_CONTROL` 的力矩按**输出端**处理
 *    （固件 `motor_torque = torque / gear_ratio`），而 `TORQUE_CONTROL` 的力矩
 *    直接赋给 `input_torque_`，是**电机端** N·m。两者相差一个 `gear_ratio`！
 *    本模块只忠实转发，不做补偿；L3 必须分别按对端换算。
 *
 * 2. **CURRENT_CONTROL 没有广播版本**。固件的 `do_command()` 只注册了
 *    `MSG_CURRENT_CONTROL (0x04)`，**不存在 0x84**。因此 `cb_msgtype_t`
 *    里也没有 0x84 —— 需要多轴同步电流控制时只能用 MIT 广播。
 *
 * 定点字段的量化：
 *   - Classic POS 的 `vel_limit` 用 int16 RPM（±32767），`cur_limit` 用 int16 的
 *     0.1 A/bit（±3276.7 A）。固件对越界值**不钳位直接 `static_cast`**，行为未定义；
 *     本模块钳位并通过 @c flags 上报（位约定与 `CB_MIT_*` 一致）。
 */

#ifndef CB_CTRL_H
#define CB_CTRL_H


#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * 线上长度与量纲常量
 * ------------------------------------------------------------------------ */

#define CB_CTRL_POS_LEN_CLASSIC 8u    /**< POS: 4B f32 + 2B i16 + 2B i16 */
#define CB_CTRL_POS_LEN_FD      12u   /**< POS: 3 × f32 */
#define CB_CTRL_VEL_LEN         8u    /**< VEL: 2 × f32（Classic 与 FD 同布局） */
#define CB_CTRL_TORQUE_LEN      4u    /**< TORQUE: 1 × f32 */
#define CB_CTRL_CURRENT_LEN     4u    /**< CURRENT: 1 × f32 */

/** Classic POS 的 `cur_limit` 每 bit 代表的安培数 */
#define CB_CTRL_CLASSIC_CUR_LSB 0.1f

#define CB_CTRL_I16_MIN (-32768)
#define CB_CTRL_I16_MAX 32767

/* --------------------------------------------------------------------------
 * 修正标志（“请求值 != 实际下发值”）
 *
 * 位约定与 cb_mit.h 的 CB_MIT_CLAMP_* 一致：从 0x01 起按**线上字段顺序**逐位分配，
 * 0x80 恒为 INVALID（输入含 NaN）。分层语义也一致：NaN 同时置 INVALID 与该字段标志。
 * ------------------------------------------------------------------------ */

/** POS：第 1 字段 pos_deg 被修正 */
#define CB_CTRL_POS_F_POS  0x01u
/** POS：第 2 字段 vel_limit 被修正 */
#define CB_CTRL_POS_F_VEL  0x02u
/** POS：第 3 字段 cur_limit 被修正 */
#define CB_CTRL_POS_F_CUR  0x04u

/** VEL：第 1 字段目标速度被修正 */
#define CB_CTRL_VEL_F_VEL  0x01u
/** VEL：第 2 字段电流限制被修正 */
#define CB_CTRL_VEL_F_CUR  0x02u

/** TORQUE：目标力矩被修正 */
#define CB_CTRL_TAU_F_TAU  0x01u

/** CURRENT：目标电流被修正 */
#define CB_CTRL_CUR_F_CUR  0x01u

/** 任一帧：输入含 NaN（已按 0 处理，±Inf 只算钳位） */
#define CB_CTRL_F_INVALID  0x80u

/* --------------------------------------------------------------------------
 * POS_CONTROL
 * ------------------------------------------------------------------------ */

/**
 * 打包 POS_CONTROL 载荷。
 *
 * @param dst     输出缓冲；Classic 需 ≥8 B，FD 需 ≥12 B
 * @param classic 非 0 → Classic 8 B 变体；0 → FD 12 B 变体
 * @param pos_deg        目标位置（**输出端**度）
 * @param vel_limit_rpm  速度限制（**输出端**RPM）
 * @param cur_limit_a    电流限制（**电机端**A）
 * @param flags   可选（可传 NULL）。输出修正标志，见 CB_CTRL_POS_F_* / CB_CTRL_F_INVALID
 * @return 写入的字节数（8 或 12）；参数非法时返回 0
 */
size_t cb_ctrl_pos_pack(uint8_t *dst, int classic,
                        float pos_deg, float vel_limit_rpm, float cur_limit_a,
                        uint8_t *flags);

/**
 * 解包 POS_CONTROL 载荷。
 * @param src   载荷首字节
 * @param len   实际长度（决定按 Classic 还是 FD 解析）
 * @param classic 非 0 → 强制按 Classic 解析；此参数仅为明确意图，实际以 @p len 为准
 * @return 0 成功；-1 长度不足以解析
 */
int cb_ctrl_pos_unpack(const uint8_t *src, size_t len, int classic,
                       float *pos_deg, float *vel_limit_rpm, float *cur_limit_a);

/* --------------------------------------------------------------------------
 * VEL_CONTROL
 * ------------------------------------------------------------------------ */

size_t cb_ctrl_vel_pack(uint8_t *dst, float target_vel_rpm, float cur_limit_a,
                        uint8_t *flags);

int cb_ctrl_vel_unpack(const uint8_t *src, size_t len,
                       float *target_vel_rpm, float *cur_limit_a);

/* --------------------------------------------------------------------------
 * TORQUE_CONTROL
 *
 * ⚠ 力矩按**电机端** N·m 解释（与 MIT 的输出端语义不同，见文件头）。
 * ------------------------------------------------------------------------ */

size_t cb_ctrl_torque_pack(uint8_t *dst, float target_torque_nm, uint8_t *flags);

int cb_ctrl_torque_unpack(const uint8_t *src, size_t len, float *target_torque_nm);

/* --------------------------------------------------------------------------
 * CURRENT_CONTROL（无广播版本）
 *
 * ⚠ 电流按**电机端** A 解释。
 * ------------------------------------------------------------------------ */

size_t cb_ctrl_current_pack(uint8_t *dst, float target_current_a, uint8_t *flags);

int cb_ctrl_current_unpack(const uint8_t *src, size_t len, float *target_current_a);

/* --------------------------------------------------------------------------
 * 辅助
 * ------------------------------------------------------------------------ */

/**
 * 该 MsgType 是否属于本模块的实时控制帧（含广播变体）。
 *
 * ⚠ **这不是固件的 `is_ctrl`。** 两者只差一个取值，但恰好是安全关键的那个：
 *   本函数**包含** `CURRENT_CONTROL(0x04)`，而固件的 `is_ctrl`
 *   （决定是否刷新 `last_cmd_time_`、即协议级 CAN 超时保护）**不含** 0x04。
 *   需要固件语义时请用 `jsdk_msgtype_feeds_watchdog()`（在 src/jsdk_core_internal.h）。
 */
int cb_ctrl_is_control_msgtype(uint8_t msgtype);

/**
 * 该 MsgType 的载荷最小长度要求（0 表示不是控制帧）。
 * 用于接收侧丢帧判断，与固件的 `if (msg.len < N) return;` 逐条对应。
 */
size_t cb_ctrl_min_len(uint8_t msgtype, int classic);

/**
 * 单播控制帧的预期应答：固件对 POS/VEL/TORQUE 会回一条 MIT 响应，
 * 但 **CURRENT 不应答**。此函数返回 1 表示“会应答”。
 *
 * ⚠ **不能用 `Seq` 关联请求与响应**（已核实）：固件 `send_mit_response()` 的
 *   `seq` 参数是**死参数**，实际 ID 用的是设备本地计数器 `tx_seq_++`，
 *   并不回显请求的 Seq。因此响应只能靠 `(source, dest, msgtype)` 匹配
 *   ——厂商自己的工具 `cyberbeast_tool.py` 也是这么做的。
 *   推论：同一 `(node, msgtype)` 上必须**串行**发请求，不能并发多个待响应请求。
 *
 * ⚠ **CURRENT_CONTROL 的自动停机保护缺失**（已核实，比“会被停机”更危险）：
 *   固件的控制类超时计时靠 `is_ctrl = (msgtype <= 0x03) || (0x80..0x83)`；
 *   0x04 不在其中，因此**只用 CURRENT 的客户端永远不会把 `last_cmd_time_` 置位**，
 *   而 `auto_stop_if_timeout()` 开头就是 `if (last_cmd_time_[i] == 0) return;`
 *   → **CAN 掉线也不会自动停机**（安全阀根本没武装）。
 *   反之，若先发过 MIT/POS/VEL/TORQUE 再只发 CURRENT，计时器会过期并
 *   `ERROR_CAN_BUS_FAILED` + `disarm()` —— 尽管 CURRENT 一直在发。
 *   两种都不好；SDK 的 `auto_keepalive` 必须周期插入一条 `is_ctrl` 帧来两头兼顾。
 *
 * @note 任何帧都会喂 ODrive 自身的 `axis.watchdog_feed()`（`do_command()` 开头
 *       无条件调用），那是**另一套机制**，不要与上面这个协议级超时混淆。
 */
int cb_ctrl_expects_response(uint8_t msgtype);

#ifdef __cplusplus
}
#endif

#endif /* CB_CTRL_H */

/* ======================== src/proto_cyberbeast/cb_query.h ======================== */
/**
 * @file    cb_query.h
 * @brief   CYBERBEAST 状态查询帧编解码（0x40 ~ 0x47）
 *
 * **关键协议特性：请求与响应使用同一个 MsgType。** 不存在独立的“响应类型”，
 * 区分靠 CAN ID 的 Source/Dest 方向：
 *
 * ```
 * 请求  M→D : dest = 设备 node_id, source = 主站, seq = 任意
 * 响应  D→M : dest = 主站, source = 设备 node_id, **seq = 0**
 * ```
 *
 * 因此 SDK 的接收路径必须靠 `cb_id_source()` 是否等于目标 node_id 来判定方向，
 * 而不是靠 MsgType。
 *
 * | MsgType | 请求载荷 | 响应载荷 |
 * |---|---|---|
 * | 0x40 QUERY_STATUS | 无 | **MIT 响应帧（8 B）**，用 `cb_mit_unpack_response()` 解 |
 * | 0x41 QUERY_POS_VEL | 无 | f32 BE pos(turns) + f32 BE vel(turns/s) |
 * | 0x42 QUERY_CURRENT | 无 | f32 BE Iq(A) + f32 BE Id(A) |
 * | 0x43 QUERY_TEMPERATURE | 无 | f32 BE motor(°C) + f32 BE fet(°C) |
 * | 0x44 QUERY_BUS | 无 | f32 BE vbus(V) + f32 BE ibus(A) |
 * | 0x45 QUERY_ERROR | 1 B：ErrorType | b0=ErrorType 回显 · b1..3 保留 · b4..7 u32 BE 错误位图 |
 * | 0x46 QUERY_DEVICE_INFO | 无 | Classic 8 B：hw u32 BE + fw u32 BE；FD 16 B：再 + serial u64 BE |
 * | 0x47 QUERY_POWER | 无 | f32 BE 电功率(W) + f32 BE 机械功率(W) |
 *
 * ⚠ **单位陷阱（已由固件源码核实，务必当心）**：同一个物理量在不同帧里单位不同！
 *
 * | 帧 | 位置 | 速度 |
 * |---|---|---|
 * | MIT 响应（含 0x40 与所有控制帧的应答） | **输出端 rad**（固件已 ×2π/gear_ratio） | **输出端 rad/s** |
 * | 0x41 QUERY_POS_VEL | **电机端 turns**（固件**未**转换） | **电机端 turns/s** |
 * | 0x48 心跳 | **电机端 turns** | **电机端 turns/s** |
 *
 * 根源：MIT 响应走 `pos_out = pos * 2π / gear_ratio`，而 0x41 / 心跳直接透传
 * `pos_estimate_linear_src_`（电机端 turns）。所以 L3 归一化时对这两类帧
 * 必须用**不同**的公式，不能只写一个转换函数。
 *
 * 其余字段：
 *   - 0x42 的电流是 **电机端** A（电流与齿轮比无关）。
 *   - 0x46 的版本号打包为 `(MAJOR << 16) | (MINOR << 8) | PATCH`（只占低 24 位）；
 *     **同一个 u32 在 hw 里第三段是 VARIANT，在 fw 里是 REVISION**，语义不同。
 */

#ifndef CB_QUERY_H
#define CB_QUERY_H


#ifdef __cplusplus
extern "C" {
#endif

#define CB_QUERY_F32X2_LEN 8u    /**< 0x41..0x44、0x47 的响应长度 */
#define CB_QUERY_ERROR_LEN 8u    /**< 0x45 响应固定 8 B */
#define CB_QUERY_MIT_RESP_LEN 8u /**< 0x40 的响应就是 MIT 响应帧，8 B */
#define CB_QUERY_DEVLEN_CLASSIC 8u
#define CB_QUERY_DEVLEN_FD      16u

/* --------------------------------------------------------------------------
 * 0x45 QUERY_ERROR 的 ErrorType（与固件 ErrorType 枚举逐值对应）
 * ------------------------------------------------------------------------ */

typedef enum {
    CB_ET_MOTOR      = 0,   /**< Motor::Error */
    CB_ET_ENCODER    = 1,   /**< Encoder::Error */
    CB_ET_SENSORLESS = 2,   /**< SensorlessEstimator::Error */
    CB_ET_CONTROLLER = 3,   /**< Controller::Error */
    CB_ET_SYSTEM     = 4,   /**< ODrive::Error（系统级） */
    CB_ET_AXIS       = 5    /**< Axis::Error */
} cb_error_type_t;

/** 版本号打包/拆解（hw 与 fw 共用同一打包方式，第三段语义不同） */
#define CB_QUERY_VER_MAJOR(v)  (((uint32_t)(v) >> 16) & 0xFFu)
#define CB_QUERY_VER_MINOR(v)  (((uint32_t)(v) >> 8)  & 0xFFu)
#define CB_QUERY_VER_THIRD(v)  ((uint32_t)(v) & 0xFFu)
#define CB_QUERY_VER_PACK(maj, min, third)                                     \
    ((((uint32_t)(maj) & 0xFFu) << 16) | (((uint32_t)(min) & 0xFFu) << 8) |    \
     ((uint32_t)(third) & 0xFFu))

/* --------------------------------------------------------------------------
 * 响应结构体
 * ------------------------------------------------------------------------ */

typedef struct { float pos_turns;   float vel_turns_per_s; } cb_query_pos_vel_t;
typedef struct { float iq_a;        float id_a;            } cb_query_current_t;
typedef struct { float motor_c;     float fet_c;           } cb_query_temp_t;
typedef struct { float vbus_v;      float ibus_a;          } cb_query_bus_t;
typedef struct { float elec_w;      float mech_w;          } cb_query_power_t;

typedef struct {
    uint8_t  err_type;      /**< 回显的请求 ErrorType */
    uint32_t err_value;     /**< 该子系统的错误位图（语义取决于 err_type） */
} cb_query_error_t;

typedef struct {
    uint32_t hw_ver;        /**< 打包值，用 CB_QUERY_VER_* 拆（第三段 = VARIANT） */
    uint32_t fw_ver;        /**< 打包值，用 CB_QUERY_VER_* 拆（第三段 = REVISION） */
    uint64_t serial;        /**< 仅 FD 变体有效，否则为 0 */
    int      has_serial;    /**< 1 = serial 有效（FD 16 B 响应） */
} cb_query_device_info_t;

/* --------------------------------------------------------------------------
 * 请求构造（主站侧）
 * ------------------------------------------------------------------------ */

/**
 * 构造查询请求载荷。
 *
 * @param dst      输出缓冲
 * @param cap      缓冲容量
 * @param msgtype  CB_MSG_QUERY_* （只接受 0x40..0x47）
 * @param err_type 仅 0x45 使用（cb_error_type_t）；其余忽略
 * @return 写入字节数；msgtype 非法或 cap 不足返回 0
 *
 * 除 0x45 需要 1 字节外，其余请求载荷长度为 0（返回 0 且不写任何字节，
 * 但此时 0 是**合法**结果，需用 cb_query_request_len() 区分“合法空载荷”与“失败”）。
 */
size_t cb_query_build_request(uint8_t *dst, size_t cap, uint8_t msgtype,
                              uint8_t err_type);

/**
 * 查询请求的合法载荷长度。
 * @return ≥0 表示合法长度（0 表示空载荷请求）；-1 表示不是查询 MsgType
 */
int cb_query_request_len(uint8_t msgtype);

/**
 * 查询响应的期望长度（用于接收侧校验）。
 * @param classic 非 0 → Classic 变体（仅 0x46 有差异）
 * @return 期望长度；0 表示该 MsgType 不是查询帧（0x40 返回 8，即 MIT 响应帧）
 */
size_t cb_query_response_len(uint8_t msgtype, int classic);

/* --------------------------------------------------------------------------
 * 响应解码（主站侧）
 *
 * 所有函数：成功返回 0，长度不足/指针为空返回 -1。
 * out 指针可为 NULL（表示不关心该字段）。
 * ------------------------------------------------------------------------ */

int cb_query_decode_pos_vel(const uint8_t *src, size_t len, cb_query_pos_vel_t *out);
int cb_query_decode_current(const uint8_t *src, size_t len, cb_query_current_t *out);
int cb_query_decode_temp   (const uint8_t *src, size_t len, cb_query_temp_t    *out);
int cb_query_decode_bus    (const uint8_t *src, size_t len, cb_query_bus_t     *out);
int cb_query_decode_power  (const uint8_t *src, size_t len, cb_query_power_t   *out);
int cb_query_decode_error  (const uint8_t *src, size_t len, cb_query_error_t   *out);
int cb_query_decode_device (const uint8_t *src, size_t len,
                            cb_query_device_info_t *out);

/* --------------------------------------------------------------------------
 * 响应编码（虚拟设备/测试用，与固件逐字节对齐）
 * ------------------------------------------------------------------------ */

size_t cb_query_encode_pos_vel(uint8_t *dst, const cb_query_pos_vel_t *v);
size_t cb_query_encode_current(uint8_t *dst, const cb_query_current_t *v);
size_t cb_query_encode_temp   (uint8_t *dst, const cb_query_temp_t    *v);
size_t cb_query_encode_bus    (uint8_t *dst, const cb_query_bus_t     *v);
size_t cb_query_encode_power  (uint8_t *dst, const cb_query_power_t   *v);
size_t cb_query_encode_error  (uint8_t *dst, const cb_query_error_t   *v);
size_t cb_query_encode_device (uint8_t *dst, const cb_query_device_info_t *v,
                               int classic);

/** ErrorType 名称（未知返回 "unknown"） */
const char *cb_error_type_name(uint8_t err_type);

#ifdef __cplusplus
}
#endif

#endif /* CB_QUERY_H */

/* ======================== src/proto_cyberbeast/cb_heartbeat.h ======================== */
/**
 * @file    cb_heartbeat.h
 * @brief   CYBERBEAST 心跳帧编解码（0x48，D→M 周期上报，无请求）
 *
 * SDK 侧用途：**不需要主动查询就能拿到电机核心状态**，是低成本保活/监控通道，
 * 也是判断“设备是否还在线、母线是否正常”的第一手依据。
 *
 * | | Classic | CAN FD |
 * |---|---|---|
 * | 长度 | **8 B** | **18 B**（⚠ 不是 64，需 HAL 正确映射 DLC） |
 * | 位置/速度 | int16，turns×100 / turns/s×100 | int32，turns×10000 / turns/s×10000 |
 * | 温度 | 仅电机温度 | 电机温度 + MOS 温度 |
 * | 母线 | **无** | vbus u16(0.1 V) + ibus i16(0.01 A) |
 * | 电流 | iq int8（**0.5 A/bit**） | iq int16（**0.01 A/bit**） |
 *
 * 共同字段：
 * ```
 * [0]  Life(高 3 bit) | ErrorFlags(低 5 bit)
 * [1]  AxisState(高 4 bit) | ControlMode(低 4 bit)
 * ```
 *
 * ⚠ 已核实的三个固件侧问题（本模块只做忠实解码，问题记录在此以便上报修复）：
 *
 * 1. **`AxisState == 16` 会溢出 4 位字段**。固件写法是
 *    `(uint8_t)(state << 4) | (control_mode & 0x0F)`；`AXIS_STATE_MOTOR_DEADTIME_
 *    CALIBRATION = 16`，`16 << 4 = 256` 截断为 `0x00`，于是**状态被上报成 0
 *    （UNDEFINED）且 control_mode 被静默丢弃**。解码侧无法区分，只能知道拿到 0。
 *
 * 2. **FD 的 vbus/ibus/pos/vel 没有钳位**（固件直接 `static_cast`），越界时行为
 *    未定义；而 Classic 的 pos/vel、两种 iq、两种温度**有**钳位。不一致。
 *
 * 3. `Life` 是 3 bit 单调计数器（0..7 循环），用于检测丢帧；主机应在两次
 *    相邻心跳间校验 `life` 是否恰好 +1（mod 8）。
 *
 * 4. **FD 的位置/速度分辨率超出 float32 精度**。FD 的 pos 是 int32 的
 *    `turns × 10000`（LSB = 0.0001 turns），但解码后存成 float32；而 float32 在
 *    `|x| > ~840 turns` 时 ulp 已大于 0.0001，**无法逐位重现原始整数**。
 *    因此“解码→重新编码得到相同字节”只在 `|pos| ≲ 840 turns` 成立；
 *    超出后只能保证误差 ≤ 1 个 int32 LSB 量级。Classic 的 ±327.67 turns
 *    分辨率较粗（0.01 turns），不存在该问题。
 */

#ifndef CB_HEARTBEAT_H
#define CB_HEARTBEAT_H


#ifdef __cplusplus
extern "C" {
#endif

#define CB_HB_LEN_CLASSIC 8u
#define CB_HB_LEN_FD      18u

/** 温度字段的偏移（线上 u8 = 实际温度 + 50） */
#define CB_HB_TEMP_OFFSET 50
#define CB_HB_TEMP_MIN    (-50)   /**< u8=0   → −50 °C */
#define CB_HB_TEMP_MAX    205     /**< u8=255 → +205 °C（⚠ 必须按 int16 存放） */

/* ---- ErrorFlags（低 5 bit，按子系统压缩） ---- */
#define CB_HB_ERR_AXIS       0x01u
#define CB_HB_ERR_MOTOR      0x02u
#define CB_HB_ERR_ENCODER    0x04u
#define CB_HB_ERR_CONTROLLER 0x08u
#define CB_HB_ERR_BOARD      0x10u
#define CB_HB_ERR_MASK       0x1Fu
/** 任一子系统有错误（快捷判断） */
#define CB_HB_ERR_ANY        0x1Fu

/* ---- 各变体的定点比例 ---- */
#define CB_HB_CLASSIC_POS_SCALE 100.0f     /**< turns × 100 */
#define CB_HB_CLASSIC_VEL_SCALE 100.0f
#define CB_HB_CLASSIC_IQ_LSB    0.5f       /**< iq int8，0.5 A/bit → ±64 A */
#define CB_HB_FD_POS_SCALE      10000.0f   /**< turns × 10000 */
#define CB_HB_FD_VEL_SCALE      10000.0f
#define CB_HB_FD_VBUS_LSB       0.1f       /**< u16，0.1 V → 0..6553.5 V */
#define CB_HB_FD_IBUS_LSB       0.01f      /**< i16，0.01 A */
#define CB_HB_FD_IQ_LSB         0.01f      /**< i16，0.01 A */

/** ControlMode（字节 1 低 4 bit） */
typedef enum {
    CB_HB_CM_VOLTAGE  = 0,
    CB_HB_CM_TORQUE   = 1,
    CB_HB_CM_VELOCITY = 2,
    CB_HB_CM_POSITION = 3
} cb_hb_control_mode_t;

/**
 * 解码结果。
 *
 * 所有字段都已换算成物理量（turns / turns·s⁻¹ / A / V / °C）。
 * Classic 变体没有的字段置 0，并用 @c have_* 区分“真的是 0”与“本变体无此字段”。
 */
typedef struct {
    int      is_fd;          /**< 1 = 来自 18 B FD 帧；0 = 来自 8 B Classic 帧 */
    uint32_t node_id;        /**< 由调用方从 CAN ID 填入；解码器不碰 */

    uint8_t  life;           /**< 0..7，用于丢帧检测 */
    uint8_t  err_flags;      /**< CB_HB_ERR_* 位或 */
    uint8_t  state;          /**< AxisState 高 4 bit（0..15，见文件头问题 1） */
    uint8_t  control_mode;   /**< cb_hb_control_mode_t */

    int16_t  motor_temp_c;   /**< −50..205 */
    int16_t  mos_temp_c;     /**< 仅 FD 有效，见 have_mos_temp */

    float    pos_turns;      /**< 电机端 turns */
    float    vel_turns_per_s;/**< 电机端 turns/s */
    float    iq_a;           /**< 电机端 A（Classic 分辨率较粗：0.5 A） */

    float    vbus_v;         /**< 仅 FD */
    float    ibus_a;         /**< 仅 FD */

    int      have_mos_temp;  /**< 1 = mos_temp_c 有效 */
    int      have_vbus;      /**< 1 = vbus_v / ibus_a 有效 */
} cb_heartbeat_t;

/* --------------------------------------------------------------------------
 * 解码
 * ------------------------------------------------------------------------ */

/** 解 8 B Classic 心跳 */
int cb_heartbeat_decode_classic(const uint8_t *src, size_t len, cb_heartbeat_t *out);

/** 解 18 B FD 心跳 */
int cb_heartbeat_decode_fd(const uint8_t *src, size_t len, cb_heartbeat_t *out);

/**
 * 按长度自动分派：8 → Classic，18 → FD。
 * @return 0 成功；-1 长度不是 8 或 18
 */
int cb_heartbeat_decode(const uint8_t *src, size_t len, cb_heartbeat_t *out);

/* --------------------------------------------------------------------------
 * 编码（虚拟设备/测试用，与固件逐字节对齐，含钳位）
 * ------------------------------------------------------------------------ */

size_t cb_heartbeat_encode_classic(uint8_t *dst, const cb_heartbeat_t *v);
size_t cb_heartbeat_encode_fd(uint8_t *dst, const cb_heartbeat_t *v);

/* --------------------------------------------------------------------------
 * 字段级辅助（心跳是周期帧，调用方常有“只看 life / 只看有无故障”的需求）
 * ------------------------------------------------------------------------ */

/** 从字节 0 提取 Life（0..7） */
uint8_t cb_heartbeat_life(uint8_t b0);
/** 从字节 0 提取 ErrorFlags（0..31） */
uint8_t cb_heartbeat_flags(uint8_t b0);
/** 组装字节 0 */
uint8_t cb_heartbeat_make_life_flags(uint8_t life, uint8_t err_flags);

/** 从字节 1 提取 AxisState（高 4 bit，0..15） */
uint8_t cb_heartbeat_state(uint8_t b1);
/** 从字节 1 提取 ControlMode（低 4 bit，0..3） */
uint8_t cb_heartbeat_control_mode(uint8_t b1);
/**
 * 组装字节 1。
 * ⚠ 与固件同样存在“state > 15 无法表示”的限制：传入 > 15 会被截断到低 4 位，
 *   并返回 -1 提示调用方（固件不报错，直接静默丢失）。
 */
int cb_heartbeat_make_state_mode(uint8_t state, uint8_t control_mode);

/**
 * 判断两个连续 life 值是否连续（用于丢帧检测）。
 * @return 1 = 连续（+1 mod 8）；0 = 有丢帧或乱序
 */
int cb_heartbeat_life_is_next(uint8_t prev_life, uint8_t cur_life);

/** len 是否是一个合法的心跳帧长度 */
int cb_heartbeat_len_valid(size_t len);

/** AxisState 名称（0..15；5 为固件跳过的空号，返回 "reserved"） */
const char *cb_heartbeat_state_name(uint8_t state);
/** ControlMode 名称 */
const char *cb_heartbeat_control_mode_name(uint8_t control_mode);

#ifdef __cplusplus
}
#endif

#endif /* CB_HEARTBEAT_H */

/* ======================== src/proto_cyberbeast/cb_param.h ======================== */
/**
 * @file    cb_param.h
 * @brief   CYBERBEAST 参数访问编解码（PARAM_READ 0x20 / PARAM_WRITE 0x21）
 *
 * 这是“端点（Endpoint）”访问层：用 u16 端点 ID 读写设备的任意配置/状态项。
 * 端点 ID → 名称/类型 的映射来自 JSON 描述符（见 cb_jsondesc_parse.h）。
 *
 * 本模块提供四种形式的**纯编解码**，外加两个状态机/规划器：
 *   - `cb_param_write_asm_t`：Classic 分段写的**接收侧装配器**（虚拟设备用）
 *   - `cb_param_plan_batches()`：批量读的**发送侧装箱器**（主站用）
 *
 * ============================================================================
 * 一、单参数读（0x20，Flags bit6 = 0）
 * ============================================================================
 * ```
 * 请求（4 B 或 8 B）:  [Flags][EpID u16 BE][ReqLen][Offset u32 BE]
 *   - ReqLen: 期望字节数 1..8；**0 视为 4**（旧客户端兼容）
 *   - Offset: 起始偏移；**请求帧短于 8 B 时视为 0**（旧客户端兼容）
 *   - Classic: ReqLen 被强制 ≤4（响应帧 = 4 B 头 + 数据，必须 ≤8 B）
 *
 * 响应:  [Flags][EpID u16 BE][DataLen][Value(DataLen B)]
 *   - Flags bit7 (0x80) = More：1 表示还有数据未返回
 *   - 设备**总是先按 8 字节读满整个值**再按 offset 切片，因此
 *     `Offset + DataLen < FullLen` 时置 More。
 *   - Offset ≥ 值长度 → DataLen = 0（合法，表示越界读完）
 * ```
 * 用途：读取 uint64 序列号等大参数。Classic 下需两块（offset 0 和 4）。
 *
 * ============================================================================
 * 二、批量读（0x20，Flags bit6 = 1，**仅 CAN FD**）
 * ============================================================================
 * ```
 * 请求:  [0x40][Count N][N × EpID u16 BE]          N ∈ 1..31
 * 响应:  [0x40][Count][ValidBitmap ⌈N/8⌉ B][ValueStream]
 *        - 不回显 EpID / DataLen：主站按 JSON 描述符的类型长度依次切分值流
 *        - ValidBitmap bit i = 1 表示第 i 条有效
 *        - 无分页：设备**永远全量返回**
 * 错误响应: [0x40|0x20][0x00]  ← 仅 2 B
 *        - 触发条件：① Classic 收到批量请求（不支持）
 *                    ② 整批值流装不下 64 B（主站应按类型拆分重发）
 * ```
 * 值流预算：`64 − 2 − ⌈N/8⌉` 字节。
 *
 * ============================================================================
 * 三、参数写（0x21）
 * ============================================================================
 * ```
 * 请求:  [Flags][EpID u16 BE][DataLen][Value(≤8 B)]
 * 确认:  [Flags 原样][EpID u16 BE][0][4 × 0x00]    固定 8 B
 * ```
 * ⚠ 确认帧**不回传是否写入成功**，也不回传错误码 —— 是静默确认。
 *   要确认写入生效必须回读一次（`PARAM_READ`）。
 *
 * ============================================================================
 * 四、分段写（Classic，DataLen > 4）
 * ============================================================================
 * Classic 8 B 帧装不下「4 B 头 + 8 B 值」，故退化为 4 B/块：
 * ```
 * 块请求（8 B）: [Flags][EpID u16 BE][TotalLen][Chunk 4 B]
 *   - Flags bit7 (0x80) = More：1 还有后续块，0 末块
 *   - TotalLen: 参数完整字节数，**只允许 5..8**；每块一致
 *   - 末块不足 4 B 的部分用 0 填充，设备按 TotalLen 截断
 * ```
 * 设备端装配的中止条件（丢块防护，本模块的装配器逐条复刻）：
 *   ① TotalLen 不在 5..8      ② offset 已 ≥ TotalLen
 *   ③ 已声明 More 但缓冲已填满  ④ EpID 或 master_id 中途变化 → 重新开始装配
 *
 * ⚠ 装配器在**中止时返回 -1，且不写入任何数据**——避免把半截值写进设备。
 */

#ifndef CB_PARAM_H
#define CB_PARAM_H


#ifdef __cplusplus
extern "C" {
#endif

/* ---- 标志位 ---- */
#define CB_PARAM_FLAG_MORE  0x80u   /**< 通用：还有后续数据 */
#define CB_PARAM_FLAG_BATCH 0x40u   /**< PARAM_READ：批量模式 */
#define CB_PARAM_FLAG_ERR   0x20u   /**< PARAM_READ 批量：整批无法完成 */

/* ---- 容量上限 ---- */
#define CB_PARAM_MAX_VALUE      8u    /**< 单个参数值最大字节数 */
#define CB_PARAM_MAX_BATCH      31u   /**< 单批最大条目数（2 + 2×31 = 64） */
#define CB_PARAM_FD_FRAME_MAX   64u   /**< CAN FD 单帧上限 */
#define CB_PARAM_CLASSIC_FRAME  8u    /**< Classic 单帧上限 */
#define CB_PARAM_CHUNK_BYTES    4u    /**< 分段写每块的字节数 */
#define CB_PARAM_SEG_MIN_LEN    5u    /**< 分段写的 TotalLen 下限 */
#define CB_PARAM_READ_REQ_MIN   4u    /**< 单读请求最短（无 offset） */
#define CB_PARAM_READ_REQ_FULL  8u    /**< 单读请求带 offset */
#define CB_PARAM_ACK_LEN        8u    /**< 写确认固定 8 B */

/* ==========================================================================
 * 一、单参数读
 * ======================================================================== */

/** 单读请求（解析结果；字段已按固件规则归一化） */
typedef struct {
    uint8_t  flags;
    uint16_t ep_id;
    uint8_t  req_len;       /**< 已归一化：0→4，>8→8，Classic>4→4 */
    uint32_t offset;        /**< 无 offset 字段时为 0 */
    int      has_offset;    /**< 请求帧是否真的带了 offset 字段 */
} cb_param_read_req_t;

/** 单读响应 */
typedef struct {
    uint8_t  flags;         /**< bit7 = More */
    uint16_t ep_id;         /**< 请求 EpID 回显 */
    uint8_t  data_len;
    uint8_t  value[CB_PARAM_MAX_VALUE];
} cb_param_read_rsp_t;

/**
 * 打包单读请求。
 * @param with_offset 非 0 → 发出完整 8 B（含 offset）；0 → 发 4 B 旧式请求
 * @return 写入字节数（4 或 8）；参数非法返回 0
 *
 * ⚠ 这里**不**接受 `classic` 参数：Classic 的 `ReqLen ≤ 4` 限制是**设备侧**
 *   归一化的（见 cb_param_normalize_req_len），主站发 8 设备也只会回 4 字节。
 *   主站若要提前知道实际能拿到多少字节，应自己调 cb_param_normalize_req_len()
 *   或 cb_param_read_chunks()，而不是让编码函数默默改动请求内容。
 */
size_t cb_param_pack_read_req(uint8_t *dst, size_t cap, uint16_t ep_id,
                              uint8_t req_len, uint32_t offset,
                              int with_offset);

/**
 * 固件在**设备侧**对 ReqLen 做的归一化（导出供主站预估切片）。
 *   0 → 4（旧客户端兼容）；> 8 → 8；Classic 且 > 4 → 4
 */
uint8_t cb_param_normalize_req_len(uint8_t req_len, int classic);

/**
 * 读一个值需要几次请求/响应（主站用来预分配循环次数）。
 * @param value_len 该端点在 JSON 描述符里的类型长度
 * @return 需要的请求次数；value_len = 0 时返回 0
 */
uint32_t cb_param_read_chunks(uint8_t value_len, uint8_t req_len, int classic);

/** 解析单读请求（应用与固件相同的归一化） */
int cb_param_unpack_read_req(const uint8_t *src, size_t len, int classic,
                             cb_param_read_req_t *out);

/** 打包单读响应 */
size_t cb_param_pack_read_rsp(uint8_t *dst, size_t cap,
                              const cb_param_read_rsp_t *v);

/** 解析单读响应 */
int cb_param_unpack_read_rsp(const uint8_t *src, size_t len,
                             cb_param_read_rsp_t *out);

/**
 * **设备侧**：按请求对完整值做切片并生成响应帧（虚拟设备用，逐字节复刻固件）。
 *
 * @param full_value 设备的完整值（长度 = full_len，≤8）
 * @param full_len   完整值长度
 * @param req_flags  请求的 Flags（响应会回显其低 7 位）
 * @return 响应帧长度（4 + DataLen）；参数非法返回 0
 */
size_t cb_param_build_read_rsp(uint8_t *dst, size_t cap,
                               uint8_t req_flags, uint16_t ep_id,
                               const uint8_t *full_value, uint8_t full_len,
                               uint8_t req_len, uint32_t offset);

/** 响应是否还有后续块（More 位） */
int cb_param_rsp_has_more(const cb_param_read_rsp_t *rsp);

/* ==========================================================================
 * 二、批量读
 * ======================================================================== */

/** 批量读请求的解析结果 */
typedef struct {
    uint8_t        count;      /**< 实际解析出的条目数（≤31） */
    const uint8_t *ep_bytes;   /**< 指向 src[2]，每个条目 2 B BE */
} cb_param_batch_req_t;

/** 批量读响应的解析结果（值流需由调用方按 JSON 类型长度切分） */
typedef struct {
    uint8_t        flags;
    uint8_t        count;         /**< 正常 = N；ERR 时为 0 */
    uint8_t        bitmap_bytes;  /**< ⌈N/8⌉；ERR 时为 0 */
    const uint8_t *bitmap;        /**< 指向 src[2]；ERR 时为 NULL */
    const uint8_t *values;        /**< 值流起始；ERR 时为 NULL */
    size_t         values_len;    /**< 值流字节数 */
    int            is_err;        /**< 1 = 设备报 ERR，需拆分重发 */
} cb_param_batch_rsp_t;

/**
 * 打包批量读请求（仅 CAN FD）。
 * @param n 条目数，须在 1..31
 * @return 写入字节数（2 + 2n）；非法返回 0
 */
size_t cb_param_pack_batch_req(uint8_t *dst, size_t cap,
                               const uint16_t *eps, uint8_t n);

/**
 * 解析批量读请求。
 * @param ep_cap @p out_eps 的容量（条目数）
 * @return 0 成功（*out 已填）；-1 不是批量请求或长度非法
 */
int cb_param_unpack_batch_req(const uint8_t *src, size_t len,
                              uint16_t *out_eps, uint8_t ep_cap,
                              cb_param_batch_req_t *out);

/**
 * 解析批量读响应。
 * @param n_req 请求时的条目数 N（用于校验 bitmap 大小）
 * @return 0 成功；-1 格式非法
 */
int cb_param_unpack_batch_rsp(const uint8_t *src, size_t len, uint8_t n_req,
                              cb_param_batch_rsp_t *out);

/** 打包批量读的 ERR 响应（2 B） */
size_t cb_param_pack_batch_err(uint8_t *dst, size_t cap);

/** 测试 ValidBitmap 的第 i 位 */
int cb_param_bitmap_test(const uint8_t *bitmap, uint8_t n_bytes, uint8_t index);

/** 计算 N 个条目需要的 bitmap 字节数 */
uint8_t cb_param_bitmap_bytes(uint8_t n);

/** 批量装箱用的条目描述（value_len 来自 JSON 描述符的类型长度） */
typedef struct {
    uint16_t ep_id;
    uint8_t  value_len;     /**< 该端点值的序列化字节数（0 表示无效端点） */
} cb_param_batch_item_t;

/**
 * **主站侧**：把条目序列贪心装箱成若干批，每批满足
 * `2 + ⌈n/8⌉ + Σvalue_len ≤ budget`（budget 通常取 64）。
 *
 * @param out_counts      输出：每批的条目数
 * @param out_offsets     输出：每批在 items 中的起始下标（可为 NULL）
 * @param max_batches     out_counts 的容量
 * @return 批次数；若条目数为 0 或任一单条装不下则返回 0
 *
 * 注意：装箱只保证**响应**装得下；请求帧要求 `2 + 2n ≤ 64` 即 n ≤ 31，
 *       本函数同时满足该约束。
 */
size_t cb_param_plan_batches(const cb_param_batch_item_t *items, size_t n_items,
                             size_t budget,
                             size_t *out_counts, size_t *out_offsets,
                             size_t max_batches);

/* ==========================================================================
 * 三、参数写
 * ======================================================================== */

/** 打包写请求；value_len 须 ≤8 */
size_t cb_param_pack_write_req(uint8_t *dst, size_t cap, uint16_t ep_id,
                               const uint8_t *value, uint8_t value_len);

/** 解析写请求 */
int cb_param_unpack_write_req(const uint8_t *src, size_t len,
                              uint8_t *out_flags, uint16_t *out_ep_id,
                              uint8_t *out_value, uint8_t *out_value_len);

/** 打包写确认（8 B，Flags 原样返回） */
size_t cb_param_pack_write_ack(uint8_t *dst, size_t cap,
                               uint8_t flags, uint16_t ep_id);

/** 解析写确认（仅校验格式，确认帧不含成败信息） */
int cb_param_unpack_write_ack(const uint8_t *src, size_t len,
                              uint8_t *out_flags, uint16_t *out_ep_id);

/* ==========================================================================
 * 四、分段写
 * ======================================================================== */

/**
 * 打包一个分段写块（Classic，8 B）。
 * @param offset 该块在完整值中的起始偏移（须为 4 的倍数，最后一块可不足 4）
 * @param more   非 0 → 置 More 位
 * @return 8；参数非法返回 0
 */
size_t cb_param_pack_write_chunk(uint8_t *dst, size_t cap, uint16_t ep_id,
                                 uint8_t total_len, uint32_t offset,
                                 const uint8_t *value, uint8_t value_len,
                                 int more);

/** 分段写装配器（接收侧，复刻固件的丢块防护） */
typedef struct {
    int      active;
    uint16_t ep_id;
    uint8_t  master_id;
    uint8_t  total_len;
    uint8_t  offset;
    uint8_t  buf[CB_PARAM_MAX_VALUE];
} cb_param_write_asm_t;

/** 重置装配器 */
void cb_param_write_asm_init(cb_param_write_asm_t *a);

/**
 * 喂入一个分段写块。
 *
 * @param value_out     组装完成时输出完整值（至少 8 B）
 * @param value_len_out 组装完成时输出 total_len
 * @return
 *    1  已接收，**还需后续块**
 *    0  组装完成（*value_out / *value_len_out 有效，应执行写入并回 ACK）
 *   -1  中止（TotalLen 非法 / 块序错误 / 声明 More 却已填满），**不写入**
 */
int cb_param_write_asm_feed(cb_param_write_asm_t *a, uint8_t master_id,
                            uint16_t ep_id, uint8_t total_len, uint8_t flags,
                            const uint8_t *chunk,
                            uint8_t *value_out, uint8_t *value_len_out);

#ifdef __cplusplus
}
#endif

#endif /* CB_PARAM_H */

/* ======================== src/proto_cyberbeast/cb_jsondesc_fetch.h ======================== */
/**
 * @file    cb_jsondesc_fetch.h
 * @brief   JSON 端点描述符下载（0x24 请求 / 0x25 数据流）—— 传输状态机
 *
 * 本模块负责“把 41 KB 的描述符从设备搬到解析器里”，**不含任何上下文/关节逻辑**：
 * 它只认识 `(0x25 载荷) → (解析器 + raw sink + 进度回调)` 这条链路，
 * 因此可以在虚拟 HAL 上独立做端到端回归。
 *
 * @par 协议时序（已对固件 `send_json_desc_chunk()` 逐行核实）
 *  - 主站发 **一帧** `0x24`，载荷 = `[offset u32 LE]`（4 B）即触发**自主流式发送**；
 *    不需要逐块请求。
 *  - 设备回 **`0x25`**，每帧长度**恒定**（Classic 8 B / FD 64 B，末尾补 0）：
 *      · 元数据帧（整次传输的**第一帧**）：`[0..1]=00 00`、`[2..5]=totalLen u32 LE`、
 *        `[6..7]=crc u16 LE`
 *      · 数据帧：`[0..1]=chunkOffset u16 LE`、`[2..]=JSON 文本`
 *  - 设备每毫秒 ≤ `kMaxJsonFramesPerCycle = 50` 帧，发完后自动清理状态。
 *
 * @par ⚠ 帧分类必须“有状态”，不能逐帧瞎猜
 *  `chunkOffset` 与元数据帧的前两字节都是 `00 00` 时无法靠前两字节区分。
 *  真正可靠的做法是：**第一帧必为元数据帧**（设备就是这么实现的），
 *  之后全部是数据帧；`buf[2] == '{'` 只用来**交叉校验**第一帧——
 *  若“元数据帧”的第 3 字节竟然是 JSON 起始符，说明设备跳过了元数据，
 *  此时无法得知 total_len，必须**整体失败**（绝不接受部分结果）。
 *
 * @par 两个必须的硬约束
 *  1. **`total_len > 65535` 一律拒绝。** `chunkOffset` 只有 u16，超出会静默回绕。
 *  2. **`stop_when_satisfied` 与 raw sink 互斥。** 提前终止会让原始字节不完整，
 *     缓存下来下次解析必然失败。用 `cb_desc_fetch_cache_safe()` 做后置校验。
 *
 * @par ⚠ 提前终止只对**全部为精确路径**的 filter 生效（重要）
 *  前缀/通配 filter（`mit_max_*`）会被它的**第一个**匹配项“满足”——
 *  于是 `satisfied` 为真、扫描立即停止，同前缀的其余路径（`mit_max_torque`…）
 *  就**静默丢失**，而且丢失哪些取决于 JSON 的字段顺序。
 *  这是无法在流式扫描下可靠判定的（除非下完整份描述符），
 *  因此本模块的规则是：
 *
 *  > **只要 filter 列表里存在任何通配/前缀（`*` 结尾）或段前缀（`.` 结尾）条目，
 *  > 就禁用提前终止**，宁可多下几百帧，也不交付一个依赖字段顺序的残缺端点表。
 *
 *  想把 arena 压到最小，请**显式列出全部需要的精确路径**（如逐条写出 5 个
 *  `mit_max_*`），此时提前终止会正常生效（实测省 40% 帧数）。
 *  实际是否启用过提前终止由 `cb_desc_fetch_result_t.stop_allowed` 告知。
 */

#ifndef CB_JSONDESC_FETCH_H
#define CB_JSONDESC_FETCH_H



#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * 常量
 * ------------------------------------------------------------------------ */

#define CB_DESC_REQ_LEN        4u      /**< 0x24 请求载荷长度：offset u32 LE */
#define CB_DESC_META_MIN_LEN   8u      /**< 元数据帧至少需要 8 字节 */
#define CB_DESC_META_HDR_BYTES 8u      /**< 元数据帧的有效字段长度 */
#define CB_DESC_DATA_HDR_BYTES 2u      /**< 数据帧头：chunkOffset u16 LE */

/**
 * `total_len` 上限。
 * ⚠ 不是随意取的：`chunkOffset` 是 **u16**，固件在 64 KB 处会静默回绕，
 *   因此 SDK 侧必须在 65535 处硬拒绝（见 PROTOCOL_NOTES §5.6）。
 */
#define CB_DESC_MAX_TOTAL_LEN  65535u

/** 合法帧长（固件恒发整帧：Classic 8 / FD 64） */
#define CB_DESC_FRAME_LEN_CLASSIC 8u
#define CB_DESC_FRAME_LEN_FD      64u

/** JSON 文本的第一个字节（用于第一帧的交叉校验） */
#define CB_DESC_JSON_FIRST_BYTE  0x7Bu   /* '{' */

/* --------------------------------------------------------------------------
 * 失败原因（静态字符串，便于日志与测试断言）
 * ------------------------------------------------------------------------ */

#define CB_DESC_ERR_NONE          ((const char *)0)
#define CB_DESC_ERR_FRAME_LEN     "0x25 frame length must be 8 (Classic) or 64 (FD)"
#define CB_DESC_ERR_META_EXPECTED "first 0x25 frame must be the metadata frame"
#define CB_DESC_ERR_META_TOTAL    "metadata total_len must be 1..65535"
#define CB_DESC_ERR_OFFSET        "chunkOffset out of sequence"
#define CB_DESC_ERR_PARSE         "JSON parser rejected the descriptor"
#define CB_DESC_ERR_ARENA         "endpoint arena exhausted"

/* 注：截断不会产生自己的错误码——它表现为 `cb_desc_fetch_is_done() == 0`
   （调用方靠自己给的超时终止）。raw sink 报错也不使下载失败，
   而是置 `raw_sink_failed` 并继续，因此同样没有独立的错误码。 */

/* --------------------------------------------------------------------------
 * 状态机
 * ------------------------------------------------------------------------ */

typedef struct {
    /* --- 注入的依赖（均不拥有） --- */
    jsdk_context_t        *ctx;        /**< 不透明透传给回调，可 NULL */
    const jsdk_desc_config_t *cfg;     /**< 提供 retain / filter / 上限 */
    jsdk_ep_store_t       *store;      /**< arena + 计数 */

    jsdk_desc_raw_sink_fn  sink;       /**< 可 NULL */
    void                  *sink_user;
    jsdk_desc_progress_fn  progress;   /**< 可 NULL */
    void                  *progress_user;

    /* --- 解析器 --- */
    jsdk_jsondesc_t parser;

    /* --- 传输状态 --- */
    uint32_t total_len;      /**< 来自元数据帧 */
    uint16_t crc;            /**< 来自元数据帧（VersionCRC） */
    uint32_t next_offset;    /**< 期望的下一个 chunkOffset */
    uint32_t bytes_scanned;  /**< 已喂给解析器的字节数 */
    uint32_t frames_rx;      /**< 收到的 0x25 帧数（含元数据帧） */
    uint32_t last_report;    /**< 上次回调进度时的字节数（节流用） */

    uint8_t  started;        /**< 已经收到过元数据帧 */
    uint8_t  done;           /**< 传输已结束（成功或失败） */
    uint8_t  failed;         /**< 1 = 失败 */
    uint8_t  complete;       /**< 1 = 收满 total_len 且解析器正常收尾 */
    uint8_t  stopped_early;  /**< 1 = filter 满足后主动终止（数据不完整） */
    uint8_t  stop_allowed;   /**< 1 = 本次配置允许提前终止（全部 filter 均为精确路径） */
    uint8_t  raw_sink_failed;/**< 1 = sink 报错，已停止回调 */
    uint8_t  sink_called;    /**< 至少成功调用过 sink 一次 */
    uint8_t  arena_full;     /**< 曾因 arena 不足被拒 */

    int      fail_rc;        /**< 首次失败返回的错误码（重复调用返回同一值） */
    const char *err;         /**< 失败原因（CB_DESC_ERR_*） */
    const char *err_detail;  /**< 解析器的细粒度原因（可 NULL） */
} cb_desc_fetch_t;

/** 传输结束后的汇总（供 desc_info / 缓存决策使用） */
typedef struct {
    uint32_t total_len;
    uint16_t crc;
    uint32_t bytes_scanned;
    unsigned frames_rx;
    unsigned endpoint_count;   /**< 已入 arena 的端点数 */
    unsigned parsed_total;     /**< 实际解析到的叶子总数（含未保留） */
    size_t   arena_used;       /**< arena 实际占用字节数 */
    uint8_t  complete;
    uint8_t  stopped_early;
    uint8_t  stop_allowed;     /**< 本次是否允许提前终止（见文件头说明） */
    uint8_t  raw_sink_failed;
    uint8_t  failed;
} cb_desc_fetch_result_t;

/* --------------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------------ */

/**
 * 初始化一次下载。
 * @param cfg   描述符配置（必需；提供 retain / filter_paths / max_* / arena）
 * @param store 端点存储（必需；其 arena 必须是调用者提供的有效内存）
 * @return JSDK_OK / JSDK_ERR_INVALID_ARG
 */
int cb_desc_fetch_init(cb_desc_fetch_t *f, jsdk_context_t *ctx,
                       const jsdk_desc_config_t *cfg, jsdk_ep_store_t *store);

/** 安装回调（可随时调用；sink = NULL 表示不 tee）。 */
void cb_desc_fetch_set_raw_sink(cb_desc_fetch_t *f, jsdk_desc_raw_sink_fn fn,
                                void *user);
void cb_desc_fetch_set_progress(cb_desc_fetch_t *f, jsdk_desc_progress_fn fn,
                                void *user);

/**
 * 构造 0x24 请求载荷。
 * @param offset 起始字节偏移（断点续传用；正常为 0）
 * @return 写入字节数（恒为 4）；cap 不足返回 0
 */
size_t cb_desc_build_request(uint8_t *dst, size_t cap, uint32_t offset);

/**
 * 喂入一帧 `0x25` 载荷。
 *
 * 必须在**同一线程**内按到达顺序调用；帧可能是元数据帧也可能是数据帧，
 * 由内部状态判定（见文件头说明）。
 *
 * @return JSDK_OK 已接受；JSDK_ERR_PARSE / JSDK_ERR_NO_MEMORY / JSDK_ERR_PROTOCOL
 */
int cb_desc_fetch_frame(cb_desc_fetch_t *f, const uint8_t *payload, size_t len);

/** 传输是否已结束（成功或失败）。 */
int cb_desc_fetch_is_done(const cb_desc_fetch_t *f);

/**
 * 传输是否**可用**：未失败，且（完整收到 或 按 filter 提前终止且 filter 全命中）。
 */
int cb_desc_fetch_is_ok(const cb_desc_fetch_t *f);

/**
 * 原始字节是否可信（三条后置校验同时成立）：
 *   ① `complete == 1`（没被提前终止）
 *   ② `raw_sink_failed == 0`
 *   ③ `bytes_scanned == total_len`
 * 只有本函数返回 1 时才允许把 raw sink 收到的字节写进 Flash 当缓存。
 */
int cb_desc_fetch_cache_safe(const cb_desc_fetch_t *f);

const char *cb_desc_fetch_error(const cb_desc_fetch_t *f);

/**
 * 失败时解析器给出的**更细**原因（如 "path longer than max_path_len"），
 * 无更细原因时返回 NULL。仅用于日志，**不要**用它做判定。
 */
const char *cb_desc_fetch_detail(const cb_desc_fetch_t *f);

void cb_desc_fetch_result(const cb_desc_fetch_t *f, cb_desc_fetch_result_t *out);

/** 已扫描比例（0..100）；尚未收到元数据帧时返回 0。 */
unsigned cb_desc_fetch_pct(const cb_desc_fetch_t *f);

/** 期望的帧长（按 cfg 的 is_fd 推断不了，由调用方传；此处仅做常量导出）。 */
int cb_desc_frame_len_valid(size_t len);

/** 生效的下载超时（ms）：`cfg->timeout_ms` 为 0 时取 5000。 */
uint32_t cb_desc_timeout_ms(const jsdk_desc_config_t *cfg);

/**
 * filter 列表是否**全部为精确路径**（无 `*` / `.` 结尾的通配）。
 * 只有为真时才允许提前终止，理由见文件头。filter_count = 0 时返回 1。
 */
int cb_desc_filters_all_exact(const jsdk_desc_config_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* CB_JSONDESC_FETCH_H */

/* ======================== src/proto_cyberbeast/cb_desc_cache.h ======================== */
/**
 * @file    cb_desc_cache.h
 * @brief   描述符缓存：已解析结果的导出/导入（PORTING §3 路线 A）
 *
 * 目的：免掉每次上电重下 41 KB / 662 帧。两条路线：
 *
 *   **路线 A（本模块）**：缓存**已解析的端点表**。体积小
 *   （`RETAIN_FILTERED` + 12 条路径约 0.5 KB），但结果被
 *   **retain / filter_paths / SDK 内部格式**三者绑定。
 *   → 导出文件自带这三项的失效键，导入时逐项校验，不匹配就明确报错。
 *
 *   **路线 B**：缓存**原始 JSON**（`jsdk_desc_raw_sink_fn` + `cb_desc_import_raw()`），
 *   只与 `(fw_version, desc_crc)` 绑定，改 filter 无需重下。约 41 KB。
 *
 * @par 格式设计要点
 *  - **不依赖 arena 布局**：导出的是 `(ep_id, type, access, path)` 记录序列，
 *    导入时逐条重新 `jsdk_ep_arena_put()`。因此**导入侧的 arena 可以比导出侧小或大**
 *    —— 只要装得下即可，这对现场缩容很重要。
 *  - **自带完整性校验**：`body_crc` 覆盖全部记录，防 Flash 位翻转。
 *  - **失败不留部分结果**：任一步校验失败都 `jsdk_ep_store_reset()`。
 *  - **字节序固定小端**，与 CPU 无关（缓存要能跨平台读）。
 */

#ifndef CB_DESC_CACHE_H
#define CB_DESC_CACHE_H



#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * 格式常量
 * ------------------------------------------------------------------------ */

/** 魔数：ASCII "JSDK" 的小端表示 */
#define CB_DESC_CACHE_MAGIC    0x4B44534Au
/** 格式版本。**改动线格式时必须 +1**，旧缓存会被判为不兼容而不是误读。 */
#define CB_DESC_CACHE_VERSION  1u
/** 固定头长度 */
#define CB_DESC_CACHE_HDR_LEN  48u
/** 单条记录的头：ep_id u16 + type u8 + access u8 + path_len u16 */
#define CB_DESC_CACHE_REC_HDR  6u

/** `max_endpoints` 的生效值（0 → 2048） */
uint32_t cb_desc_eff_max_endpoints(const jsdk_desc_config_t *cfg);
/** `max_path_len` 的生效值（0 → 128） */
uint32_t cb_desc_eff_max_path_len(const jsdk_desc_config_t *cfg);

/** 头里 `flags` 的位 */
#define CB_DESC_CACHE_F_FILTERED 0x0001u  /**< 1 = RETAIN_FILTERED */
#define CB_DESC_CACHE_F_EARLY    0x0002u  /**< 1 = 下载时被 stop_when_satisfied 提前终止 */

/* --------------------------------------------------------------------------
 * 失效键
 * ------------------------------------------------------------------------ */

/**
 * 计算 `cfg` 的失效键。覆盖：retain 模式、完整 filter 列表（顺序敏感）、
 * `max_endpoints`、`max_path_len`，以及本文件的线格式版本。
 *
 * 用途：① 写进导出头，导入时校验；② 应用也可用它做自己缓存头的键。
 */
uint32_t cb_desc_filter_hash(const jsdk_desc_config_t *cfg);

/* --------------------------------------------------------------------------
 * 导出
 * ------------------------------------------------------------------------ */

/** 导出所需字节数（`header + body`）。store 为空时返回头长度。 */
size_t cb_desc_cache_size(const jsdk_ep_store_t *store);

/**
 * 把已解析的端点表导出到 `buf`。
 *
 * @param cfg 必须与解析时**同一份**（用于写失效键）
 * @param early 下载是否被 `stop_when_satisfied` 提前终止（仅记录，不参与校验）
 * @param out_len 输出实际长度（可为 NULL）
 * @return JSDK_OK / JSDK_ERR_INVALID_ARG
 *         / JSDK_ERR_NO_MEMORY（cap 不足，需扩容后重试）
 */
int cb_desc_cache_export(const jsdk_desc_config_t *cfg, const jsdk_ep_store_t *store,
                         int early, void *buf, size_t cap, size_t *out_len);

/* --------------------------------------------------------------------------
 * 导入
 * ------------------------------------------------------------------------ */

/**
 * 从 `buf` 恢复端点表。
 *
 * 校验顺序：长度 → 魔数 → 版本 → 头长 → retain → filter_hash →
 *           max_endpoints → max_path_len → 逐条记录边界 → body_crc。
 * 全部通过后才开始写 arena；任一步失败都返回错误并**清空 store**。
 *
 * @return JSDK_OK
 *         JSDK_ERR_INVALID_ARG  指针/长度非法
 *         JSDK_ERR_PROTOCOL     魔数/版本/头长/记录越界/CRC 不符
 *         JSDK_ERR_BAD_STATE    失效键不匹配（retain / filter / 上限 变了）
 *         JSDK_ERR_NO_MEMORY    arena 装不下（**无部分结果**）
 */
int cb_desc_cache_import(const jsdk_desc_config_t *cfg, jsdk_ep_store_t *store,
                         const void *buf, size_t len);

/** 从导出缓冲里读出头部的只读信息（用于应用做 (fw_version, crc) 比对）。 */
typedef struct {
    uint16_t desc_crc;      /**< 描述符 VersionCRC */
    uint16_t flags;         /**< CB_DESC_CACHE_F_* */
    uint32_t fw_version;    /**< 导出时记录的固件版本（应用自己填的） */
    uint32_t endpoint_count;
} cb_desc_cache_meta_t;

/**
 * 只解析头部，不碰 store。用于“先看键对不对，再决定是否导入”。
 * @return JSDK_OK / JSDK_ERR_INVALID_ARG / JSDK_ERR_PROTOCOL
 */
int cb_desc_cache_peek(const void *buf, size_t len, cb_desc_cache_meta_t *out);

/** 覆盖导出头里的 fw_version（导出后再补写，避免导出时还要查设备）。 */
int cb_desc_cache_set_fw_version(void *buf, size_t len, uint32_t fw_version);

/**
 * 覆盖导出头里的描述符 VersionCRC（来自 `cb_desc_fetch_result_t.crc`）。
 * 应用把两个键写进自己的缓存头，启动时就能在导入前先比对。
 */
int cb_desc_cache_set_desc_crc(void *buf, size_t len, uint16_t crc);

/* --------------------------------------------------------------------------
 * 路线 B：从原始 JSON 重建
 * ------------------------------------------------------------------------ */

/**
 * 用**设备原始 JSON** 重建端点表（`retain` / `filter_paths` 按当前 cfg 生效）。
 *
 * 与 `cb_desc_cache_import()` 的区别：不依赖 SDK 内部格式，也不需要
 * `(fw_version, crc)` 之外的键；但要求 JSON 是**完整**的
 * （即下载时必须 `stop_when_satisfied = 0`，否则 `JSDK_ERR_PARSE`）。
 *
 * @return JSDK_OK / JSDK_ERR_INVALID_ARG / JSDK_ERR_PARSE / JSDK_ERR_NO_MEMORY
 */
int cb_desc_import_raw(const jsdk_desc_config_t *cfg, jsdk_ep_store_t *store,
                       const void *json, size_t len);

/** CRC-16/CCITT-FALSE（导出体校验用；也可供应用校验 Flash 内容）。 */
uint16_t cb_desc_cache_crc16(const void *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* CB_DESC_CACHE_H */

/* ======================== src/jsdk_core_internal.h ======================== */
/**
 * @file    jsdk_core_internal.h
 * @brief   L3 关节层的内部结构（**不安装、不对外**）
 *
 * 分层：L1 HAL → L2 协议（cb_*）→ **L3 本文件** → L4 公共 C ABI（joint_sdk.h）。
 * 本层是纯逻辑：**零 malloc、零平台 #ifdef、零阻塞**（配置阶段 API 例外，见
 * `jsdk_joint_read_config_snapshot()` 等）。
 *
 * @par 为什么不用指针指向公共结构体
 *  `jsdk_context_t` / `jsdk_joint_t` 对外是不透明句柄，但**存储由调用者提供**
 *  （`jsdk_context_storage_t`）。因此本文件里的定义就是实际布局，其大小必须
 *  塞进 `JSDK_CONTEXT_MAX_SIZE`；`jsdk_context.c` 里有编译期断言守住这条。
 */

#ifndef JSDK_CORE_INTERNAL_H
#define JSDK_CORE_INTERNAL_H



/* ==========================================================================
 * 编译期常量
 * ======================================================================== */

/** 上下文/关节魔数（ABI 守卫；`jsdk_context_init` 与每个公开入口校验）。 */
#define JSDK_CTX_MAGIC    0x4B43544Au  /**< "TCK" 变体 */
#define JSDK_JOINT_MAGIC  0x4B4E4A54u

/** `last_error` 缓冲长度（DESIGN §6.8：192 字节，可读给工程师）。 */
#define JSDK_ERRSTR_LEN   192u

/** 每个关节的参数槽位数（DESIGN §6.9）。槽位 0 保留给故障详情自动读取。 */
#define JSDK_SDO_SLOTS    8u

/** 关节容量硬上限（与公共头 `JSDK_MAX_JOINTS_STATIC` 一致）。 */
#define JSDK_MAX_JOINTS   JSDK_MAX_JOINTS_STATIC

/** 单周期接收帧数默认上限（防总线风暴阻塞控制循环）。 */
#define JSDK_RX_BURST_DEFAULT 32u

/** 控制帧「近超时」预警阈值下限（ms）。 */
#define JSDK_WD_RISK_MIN_MS  10u

/** 配置阶段等待单次响应/状态跳转的默认超时（ms）。 */
#define JSDK_CFG_TIMEOUT_MS  200u

/**
 * `jsdk_context_activate()` 的**整个过程**超时（ms）。
 *
 * ⚠ 不能复用 JSDK_CFG_TIMEOUT_MS(200)：那是"等单次响应"的尺度，而这里是
 *   "CLEAR_ERRORS → START_MOTOR → 进闭环 → 发安全首帧"整条使能链，
 *   真机上通常几百 ms，带制动/大惯量时可达数秒。用 200 ms 会在真机上
 *   必然失败（而仿真里因为时间推进得快，反而看不出来）。
 */
#define JSDK_ACTIVATE_TIMEOUT_MS 5000u

/** `can.config.break_timeout == 0` 在固件里按 **100 ms** 处理（0 ≠ 关闭）。 */
#define JSDK_WD_DEFAULT_MS   100u

/** 未提供 period_ns 时，式微序列用这个周期估算（ms）。 */
#define JSDK_CFG_PERIOD_FALLBACK_MS  1u

/** 节点发现“被动阶段”的静置时长：2 个默认心跳周期（§6.7）。 */
#define JSDK_DISCOVER_PASSIVE_MS     200u

/**
 * 该 MsgType 的帧会不会刷新固件的**协议级超时计时器**（`last_cmd_time_`）。
 *
 * 与固件 `is_ctrl` 逐字对应：`msgtype <= 0x03 || (0x80 <= msgtype <= 0x83)`。
 *
 * ⚠ **不要**用 `cb_ctrl_is_control_msgtype()` 代替：那是"属于 ctrl 模块的实时
 *   控制帧"，**包含 0x04 CURRENT_CONTROL**，而固件的 `is_ctrl` 恰好**不含** 0x04。
 *   两者正好在唯一关键的那一个取值上不同 —— 用错就会让 SDK 以为电流指令能喂狗，
 *   于是永远不补 keepalive，把固件侧的 F19（安全阀不武装 / 误停）原样复制到
 *   SDK 内部。（本项目确实踩过：`test_joint` 的 `[6] watchdog` 用例暴露出来。）
 */
static inline int jsdk_msgtype_feeds_watchdog(uint8_t msgtype)
{
    return (msgtype <= (uint8_t)CB_MSG_TORQUE_CONTROL
            || (msgtype >= (uint8_t)CB_MSG_MIT_CONTROL_BCAST
                && msgtype <= (uint8_t)CB_MSG_TORQUE_CONTROL_BCAST)) ? 1 : 0;
}

/* ==========================================================================
 * 目标缓存
 * ------------------------------------------------------------------------
 * setter 是 `void`（RT 路径无分支），越界判定与编码统一放到 cycle_end()。
 * 每个量独立记 `have_*`，这样切换模式不会把别的模式的量一起清掉。
 * ======================================================================== */

typedef struct {
    double pos_rad;        /**< 输出端 rad（MIT / CSP） */
    double vel_rad_s;      /**< 输出端 rad/s（MIT / CSV） */
    double tau_Nm;         /**< 输出端 N·m（MIT / CST） */
    double kp, kd;         /**< MIT 线上值（原样透传，见 §6.2） */
    double cur_A;          /**< 电机端 A（CURRENT） */
    double vel_lim_rad_s;  /**< POS/VEL 模式的限速（输出端） */
    double cur_lim_A;      /**< POS/VEL 模式的限流（电机端） */

    uint8_t have_pos;      /**< set_target_position_rad / set_mit 调用过 */
    uint8_t have_vel;
    uint8_t have_tau;
    uint8_t have_mit;      /**< set_mit / set_mit_stiffness 调用过 */
    uint8_t have_cur;
    uint8_t have_raw_pos;  /**< set_target_position(raw) 调用过：原样透传 */
    uint8_t have_raw_vel;
    uint8_t have_raw_tau;

    int32_t raw_pos, raw_vel;  /**< 协议原始量（/1000 定点，见头文件 §11 说明） */
    int16_t raw_tau;
} jsdk_target_t;

/* ==========================================================================
 * 参数槽位（SDO 风格）
 * ======================================================================== */

typedef struct {
    uint16_t ep_id;
    uint16_t size;       /**< 缓冲字节数（描述符给出的类型长度） */
    uint8_t  in_use;
    uint8_t  state;      /**< jsdk_sdo_state_t */
    uint8_t  data[8];    /**< 端点值最大 8 字节（u64） */
} jsdk_sdo_slot_t;

/* ==========================================================================
 * 关节
 * ======================================================================== */

struct jsdk_joint {
    uint32_t magic;
    struct jsdk_context *ctx;
    uint8_t  index;        /**< 在 ctx->joints[] 中的下标 */
    uint8_t  reserved[3];

    jsdk_joint_config_t cfg;   /**< 用户给的配置副本（0 = 自动发现） */

    /* ---- 标定量程（configure() 从描述符读回，全部为设备实际值） ---- */
    cb_mit_range_t    range;
    float             gear_ratio;
    float             torque_constant;
    float             max_current_a;  /**< cb_mit_response_max_current() */
    jsdk_unit_scale_t scale;
    uint8_t           calibrated;     /**< 1 = 量程/齿比/力矩常数均有效 */
    uint8_t           shared_desc;    /**< 1 = 端点表复用自同总线其它节点 */

    /* ---- 端点 ID（由运行时描述符解析，无内置表） ---- */
    uint16_t ep_gear_ratio, ep_torque_constant;
    uint16_t ep_mit_pos, ep_mit_vel, ep_mit_tau, ep_mit_kp, ep_mit_kd;
    uint16_t ep_requested_state, ep_current_state, ep_node_id, ep_break_timeout;

    /* ---- 设备侧配置读回 ---- */
    uint32_t break_timeout_ms;    /**< can.config.break_timeout（0 → 固件当 100 ms） */
    uint32_t node_id_readback;    /**< axis0.config.can.node_id */
    uint32_t heartbeat_rate_ms;   /**< axis0.config.can.heartbeat_rate_ms（0 = 设备不发心跳） */
    uint8_t  current_state_raw;   /**< axis0.current_state（固件 AxisState 0..16） */
    uint8_t  state_known;         /**< 1 = 至少读到过一次 current_state */

    /* ---- 目标 ---- */
    jsdk_target_t tgt;

    /* ---- 模式与状态机 ---- */
    jsdk_mode_t mode;
    jsdk_mode_t enable_mode;      /**< request_enable 携带的模式 */
    uint8_t  enabled;             /**< 归一化结论：闭环执行中（**且使能序列已走完**） */
    uint8_t  enable_pending;
    uint8_t  disable_pending;
    uint8_t  faultreset_pending;
    uint8_t  first_frame_done;    /**< 使能后的安全首帧已发（§6.4） */
    uint8_t  seq_step;            /**< 使能/失能序列步号（非阻塞步进） */
    uint8_t  ctrl_blocked;        /**< 1 = 标定/回零进行中，禁止发控制帧（§6.4） */
    /**
     * 1 = 本关节已在**发控制帧**（使能序列走完后置位，失能/复位后清除）。
     *
     * ⚠ 不能用「使能序列走完」以外的条件代替：没使能的关节如果也发 MIT，
     *   就等于替客户把设备的**协议级超时保护**武装了（`is_ctrl` 会刷新
     *   `last_cmd_time_`）——设备本来是 IDLE 安全的，现在“客户不再调用循环”
     *   反而会让它 `CAN_BUS_FAILED`。
     */
    uint8_t  tx_active;

    /**
     * 1 = 本周期已经给这个关节发过控制帧（单播或广播都算）。
     *
     * 由 `cycle_begin()` 清零；发送成功后置位。
     * 用途：`jsdk_group_set_mit()` 已经用一条广播帧驱动了 N 个关节，
     * `cycle_end()` 就不应再给它们各发一条单播 —— 否则“广播同步”
     * 反而比逐个单播还多一条帧，失去意义。
     */
    uint8_t  sent_cycle;
    uint8_t  hold_requested;      /**< 本周期调用过 hold_position*（用于诊断） */
    uint32_t seq_start_ms;        /**< 当前序列步的起始时刻（超时判定） */

    /* ---- 反馈（直接就是对外结构，避免二次转换） ---- */
    jsdk_joint_feedback_t fb;
    uint32_t last_fb_ms;      /**< 最近一次有效反馈时刻 */
    uint32_t hb_seen_ms;      /**< 最近一次心跳时刻 */
    uint8_t  hb_seen;

    /* ---- 记账 ---- */
    uint32_t last_ctrl_tx_ms; /**< 最近一次发出**控制类**帧（喂狗有效） */
    uint32_t keepalive_sent;  /**< 本关节被自动补喂狗的次数 */
    uint32_t tx_frames;       /**< 本关节发出的控制帧总数 */
    uint32_t tx_rejected;     /**< 因越界被拒绝（改发安全帧/钳位）的指令数 */
    uint16_t status_flags;    /**< JSDK_JF_* 粘滞位 */
    uint8_t  fault_prev;      /**< 上一次 is_fault（故障回调边沿检测） */
    jsdk_fault_info_t fault;

    jsdk_sdo_slot_t sdo[JSDK_SDO_SLOTS];
};

/* ==========================================================================
 * 上下文
 * ======================================================================== */

struct jsdk_context {
    uint32_t magic;
    jsdk_context_config_t cfg;   /**< 配置副本（hal 已被拷贝，可安全丢弃原结构） */

    /* ---- 循环状态 ---- */
    uint32_t now_ms;             /**< 本周期 HAL 时钟 */
    uint8_t  tx_seq;             /**< 广播/命令用的滚动 Seq（模 4） */
    uint8_t  in_cycle;
    uint8_t  destroyed;

    jsdk_joint_t joints[JSDK_MAX_JOINTS];
    unsigned     nj;

    /* ---- 描述符（端点表放在 jsdk_desc_config_t.arena，不占本结构） ---- */
    jsdk_ep_store_t   store;
    cb_desc_fetch_t   fetch;
    int               fetch_active;   /**< cb_desc_fetch 已被 init（需避免二次 init） */
    uint32_t          fetch_deadline_ms;
    uint8_t           desc_present;   /**< 1 = 端点表可用（configure 不再下载） */
    jsdk_desc_info_t  desc;           /**< 对外元信息；crc/fw 同时是缓存键 */

    /**
     * 回写 `jsdk_desc_config_t.arena_used` 的槽位。
     *
     * ⚠ 公共头把 `arena_used` 声明为**输出**（“实际用量，可按实测缩容”），
     *   但 init 时配置是被**复制**的 —— 只写自己的副本的话调用方永远看不到。
     *   因此这里存一个槽位指针直接写回。
     *
     * @warning 因此**调用方的 `jsdk_context_config_t` 必须与 arena 同寿命**
     *          （只要 arena 还活着，配置结构体通常也在）。
     */
    size_t           *arena_used_slot;
    jsdk_desc_raw_sink_fn raw_sink;
    void                 *raw_sink_user;
    jsdk_desc_progress_fn progress;
    void                 *progress_user;

    /* ---- 记账 ---- */
    jsdk_bus_state_t bus;
    uint32_t         last_rx_ms;      /**< 最近一次收到与本主站相关帧的时刻 */

    /**
     * 堆模式（`heap_optional.c`）下由 SDK 自己分配的 arena；零 malloc 模式下为 NULL。
     * 记在这里是为了让 `jsdk_context_free()` 知道该释放什么。
     */
    void                 *owned_arena;

    /* ---- 错误与回调 ---- */
    char                  last_error[JSDK_ERRSTR_LEN];
    jsdk_fault_callback_t fault_cb;
    void                 *fault_user;
};

/* ==========================================================================
 * 内部工具（跨文件共享）
 * ======================================================================== */

/** 上下文/关节魔数校验。 */
int jsdk_ctx_check(const jsdk_context_t *ctx);
int jsdk_joint_check(const jsdk_joint_t *j);

/** 写 `ctx->last_error`（printf 风格的最小子集：仅 %s / %u / %d / %%）。 */
void jsdk_ctx_seterr(jsdk_context_t *ctx, const char *fmt, ...);

/** `now_ms` 无符号差值（天然处理 32 位回绕）。 */
static inline uint32_t jsdk_elapsed(uint32_t now, uint32_t then)
{
    return (uint32_t)(now - then);
}

/**
 * 发送一帧并记账。@return HAL send 的返回值（0 = 成功）。
 * @note 失败时置 `JSDK_JF_TX_FAILED` 不在此处做（需要 joint 上下文），
 *       由调用方处理。
 */
int jsdk_ctx_send(jsdk_context_t *ctx, uint8_t pri, uint8_t msgtype,
                  uint8_t dest, const uint8_t *payload, uint8_t len);

/**
 * 阻塞等待某个 `(msgtype, source)` 的响应，同时把所有收到的帧分派给
 * 反馈解复用器（否则会丢掉期间的心跳）。
 *
 * @param want_seen 可选输出：1 = 收到目标响应
 * @return JSDK_OK / JSDK_ERR_TIMEOUT / JSDK_ERR_TRANSPORT
 * @note **仅配置阶段可用**（会阻塞并调用 HAL 的 now_ms/recv）。
 */
int jsdk_ctx_wait_response(jsdk_context_t *ctx, uint8_t msgtype, uint8_t source,
                           jsdk_can_frame_t *out, uint32_t timeout_ms);

/**
 * 目标地址上是否**已经有人在应答**（定向探测，只发一帧 `QUERY_STATUS`）。
 *
 * 用途：`set-node-id` 的前置检查 —— 目标号被别的设备占用时，
 * 「验证新地址可应答」会被那台设备满足，于是静默造出两个同号设备。
 *
 * @return 1 = 有设备应答；0 = 无应答（含发送失败）
 * @note **仅配置阶段可用**（会阻塞）。
 */
int jsdk_ctx_probe_node(jsdk_context_t *ctx, uint8_t node_id);

/**
 * 接收并解复用**一帧**（不推进时钟、不记账 rx 计数以外的状态）。
 * @return 1 = 已处理；0 = 队列空；< 0 = 链路错误
 */
int jsdk_ctx_handle_frame(jsdk_context_t *ctx, const jsdk_can_frame_t *f);

/** 至少有一个关节已被使能（描述符下载与发现都要拒绝这种状态）。 */
int jsdk_ctx_any_enabled(const jsdk_context_t *ctx);

/** 按 (source, dest) 找关节；找不到返回 NULL。 */
jsdk_joint_t *jsdk_ctx_find_joint(jsdk_context_t *ctx, uint8_t node_id);

/** 更新 `JSDK_JF_*` 粘滞位（内部用 `|=`）。 */
void jsdk_joint_set_flags(jsdk_joint_t *j, uint16_t flags);

/** 把 arena 实际用量写回调用方的 `desc.arena_used`（未提供时无操作）。 */
void jsdk_ctx_publish_arena_used(jsdk_context_t *ctx);

/** 记录一条可读错误串（含关节号）。 */
void jsdk_joint_seterr(jsdk_joint_t *j, const char *fmt, ...);

/** 单个关节结构的字节数（用于验证 `JSDK_CONTEXT_MAX_SIZE` 的预算）。 */
size_t jsdk_context_joint_size(void);

/* ---- 单位换算（jsdk_units.c） ---- */

/** 输出端 rad → 电机 turns（心跳/QUERY 用）。 */
double jsdk_units_rad_to_turns(double rad, double gear_ratio);

/** 电机 turns → 输出端 rad。齿比非法（≤0）时返回 0。 */
double jsdk_units_turns_to_rad(double turns, double gear_ratio);

/** rad/s → RPM（POS/VEL 线上单位）。 */
double jsdk_units_rad_s_to_rpm(double rad_s);

/** RPM → rad/s。 */
double jsdk_units_rpm_to_rad_s(double rpm);

/** 输出端真实刚度 → 线上 kp（§6.2：kp = stiffness × 2π / gear）。 */
double jsdk_units_stiffness_to_kp(double stiffness_nm_per_rad, double gear_ratio);

/** 线上 kp → 输出端真实刚度（kp × gear / 2π）。 */
double jsdk_units_kp_to_stiffness(double kp, double gear_ratio);

/**
 * 多圈位置展开（§6.1）。默认**不启用**：MIT 响应是 16-bit 定点，
 * 满量程 ±mit_max_pos，超出会被设备钳位；盲目累加会掩盖钳位。
 *
 * @param prev_rad  上次展开值
 * @param raw_rad   本次单圈读数
 * @param range_rad 量程（= mit_max_pos）
 * @param out_turns 输出：累计圈数（可 NULL）
 * @return 展开后的绝对位置（rad）
 *
 * @note 仅在调用方明确知道"越过 ±range 是真实运动而非钳位"时使用。
 */
double jsdk_units_pos_unwrap(double prev_rad, double raw_rad, double range_rad,
                             long *out_turns);

/* ---- 文本（jsdk_text.c） ---- */

/**
 * 把浮点写成定点文本（如 `-12.500`）。
 *
 * 为什么不直接用 `%f`：很多 MCU 工具链**默认不链接浮点 printf**，一旦用了
 * `%f` 要么增大 20 KB 代码，要么在运行期打印出 `%f` 字面量。错误串只在
 * 出错路径用，不值得为此付出代价。
 *
 * @return 写入的字符数（不含结尾 NUL）；缓冲不足时截断并仍返回 NUL 结尾。
 */
size_t jsdk_fmt_f(char *dst, size_t cap, double v, unsigned decimals);

/* ==========================================================================
 * 关节钩子（jsdk_joint.c）
 * ------------------------------------------------------------------------
 * 这些函数只由 jsdk_context.c（循环边界/解复用）与配置流程调用。
 * ======================================================================== */

/** 收到 MIT 响应（所有控制帧的应答都是这个格式）。 */
void jsdk_joint__on_mit_response(jsdk_joint_t *j, const uint8_t *data, uint8_t len);

/** 收到心跳 0x48。 */
void jsdk_joint__on_heartbeat(jsdk_joint_t *j, const uint8_t *data, uint8_t len);

/** QUERY_POS_VEL(0x41)：**电机端 turns**，需按 gear 换算成输出端 rad。 */
void jsdk_joint__on_pos_vel_turns(jsdk_joint_t *j, float pos_turns, float vel_turns_s);

void jsdk_joint__on_current_a(jsdk_joint_t *j, double iq_a);
void jsdk_joint__on_temps(jsdk_joint_t *j, double motor_c, double fet_c);
void jsdk_joint__on_bus_volts(jsdk_joint_t *j, double vbus_v, double ibus_a);

/** 收到 0xC1 FAULT_ALERT 广播。 */
void jsdk_joint__on_fault_alert(jsdk_joint_t *j);

/** cycle_end()：推进所有关节的状态机并发送本周期指令。 */
void jsdk_joint__cycle_end_all(jsdk_context_t *ctx);

/**
 * 立即编码并发送**一个**关节的本周期指令。
 * @return 1 = 已发；0 = 未发（未标定 / 本周期已发 / 发送失败）。
 * @note 用于广播同步的降级路径；正常路径由 `cycle_end()` 统一做。
 */
int jsdk_joint__send_now(jsdk_joint_t *j);

/** cycle_end()：看门狗/keepalive（jsdk_watchdog.c）。 */
void jsdk_watchdog__cycle_end(jsdk_context_t *ctx);

/**
 * 设备侧协议级超时（`can.config.break_timeout`，单位 ms）。
 * @note 读回的 0 在固件里按 **100 ms** 处理 —— 0 **不是**“关闭”（PROTOCOL_NOTES §2.5）。
 */
uint32_t jsdk_watchdog_device_ms(const jsdk_joint_t *j);

/** 刷新一个关节的归一化状态（在线/故障/使能）并触发故障回调边沿。 */
void jsdk_joint__refresh_state(jsdk_joint_t *j);

/** WP4：刷新 `age_ms` 并在反馈超时后置 `JSDK_JF_FEEDBACK_STALE`（jsdk_fault.c）。 */
void jsdk_joint__refresh_freshness(jsdk_joint_t *j);

/** WP4：反馈超时阈值（由心跳周期/控制周期观测推导）。 */
uint32_t jsdk_joint_stale_ms(const jsdk_joint_t *j);

/** 把描述符读到的标定量程应用到关节（由 configure() 调用）。 */
void jsdk_joint__apply_calibration(jsdk_joint_t *j);

/**
 * 阻塞读一个参数值（配置期）—— **只发一次请求，最多拿回一次能给的字节**。
 *
 * ⚠ 这是给“值已知 ≤ 4 B”的调用方（标定量程、`current_state`、`break_timeout`）
 *   用的**探索式**读：请求里写死 `ReqLen = 4`，返回多少字节全看设备。
 *   要读满一个值的全部字节（`u64/i64/f64` 这些 8 字节类型）**必须**用
 *   @ref jsdk_ctx_read_param_exact —— 否则设备只会回 4 字节，调用方再拿
 *   `len < 8` 去报错，就会得到一个指向错误方向的“描述符不符”消息。
 *
 * @param out     输出缓冲（≥ 8 字节）
 * @param out_len 输出实际字节数（可 NULL）
 */
int jsdk_ctx_read_param(jsdk_context_t *ctx, uint8_t node_id, uint16_t ep_id,
                        uint8_t *out, uint8_t *out_len, uint32_t timeout_ms);

/**
 * 阻塞读一个参数值：**精确读满 `want` 字节**，必要时分块。
 *
 * 分块规则完全按固件的切分行为来（不猜）：
 *  - 一次请求能拿多少由**设备侧**归一化决定：FD 下 `≤ 8`，Classic 下 `≤ 4`；
 *  - FD 且 `want ≤ 8` → **一次请求**（请求帧里 `ReqLen = want`，用 4 B 旧式形式，
 *    兼容性最好）；
 *  - Classic 且 `want > 4` → 存 4 字节一块：第一块用 4 B 旧式形式（offset 隐含 0），
 *    后续块用 8 B 形式带 offset（`cb_param_pack_read_req` 的 `with_offset`）。
 *
 * 每一块都要求设备确实推进（`data_len > 0`）；循环因为“每块至少 1 字节”而必然
 * 终止（`want ≤ CB_PARAM_MAX_VALUE = 8`），不存在卡死风险。
 *
 * @param out      输出缓冲，至少 @p want 字节
 * @param want     需要的字节数（1..CB_PARAM_MAX_VALUE）
 * @param out_len  输出：实际读到的字节数（成功时等于 @p want，可 NULL）
 * @return JSDK_OK；值比 @p want 短 → `JSDK_ERR_PROTOCOL`（并写入一句话说明）；
 *         端点不存在 → `JSDK_ERR_NOT_FOUND`；设备回了 0 字节 → 同上
 */
int jsdk_ctx_read_param_exact(jsdk_context_t *ctx, uint8_t node_id, uint16_t ep_id,
                              uint8_t *out, uint8_t want, uint8_t *out_len,
                              uint32_t timeout_ms);

/**
 * 阻塞写一个参数值（配置阶段）；写完等 8 字节静默 ACK。
 *
 * @param val 值字节，**必须是线上大端序**（与全协议一致）。
 *            直接传主机序的 `uint16_t *` 会得到字节交换后的值 —— 本项目胉过
 *            （250 变成 64000）。要写数值请用 `jsdk_joint_param_set*()`，
 *            它内部会做 `cb_be_put_*()`。
 */
int jsdk_ctx_write_param(jsdk_context_t *ctx, uint8_t node_id, uint16_t ep_id,
                         const void *val, uint8_t len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* JSDK_CORE_INTERNAL_H */

/* ======================== src/hal/hal_handle.h ======================== */
/**
 * @file    hal_handle.h
 * @brief   内置 HAL 后端的句柄布局（**内部头，不安装**）
 *
 * @par 为什么需要它
 *  `jsdk_hal_handle_t` 对客户是不透明的，但**每个后端要有自己的资源**
 *  （socket fd / 串口 fd / PCAN 句柄 / 虚拟模型）。早先的版本只有虚拟后端，
 *  `jsdk_hal_close()` 直接 `free()` 就够了；一旦有真实后端，就必须让
 *  "关闭"分派到各后端 —— 否则 **fd 泄漏**，而且是那种跑一晚上才发现的泄漏。
 *
 *  因此所有后端句柄都以一个公共前缀开头：
 *  @code
 *      typedef struct { struct jsdk_hal_handle base; ... } sc_handle_t;
 *  @endcode
 *  `base.destroy` 由各后端的 `open()` 填好，`jsdk_hal_close()` 只负责转调。
 *  由于 `base` 是首个成员，`(T *)(void *)h` 是合法的反向转换。
 */

#ifndef JSDK_HAL_HANDLE_H
#define JSDK_HAL_HANDLE_H


#ifdef __cplusplus
extern "C" {
#endif

/** 后端类别（`jsdk_hal_close()` 与诊断输出都用它）。 */
typedef enum {
    JSDK_HAL_KIND_NONE      = 0,
    JSDK_HAL_KIND_VIRTUAL   = 1,
    JSDK_HAL_KIND_SOCKETCAN = 2,
    JSDK_HAL_KIND_PCAN      = 3,
    JSDK_HAL_KIND_SLCAN     = 4
} jsdk_hal_kind_t;

/** 所有后端句柄的公共前缀。`destroy` 必须释放 @p h 自身。 */
struct jsdk_hal_handle {
    jsdk_hal_kind_t kind;
    void          (*destroy)(struct jsdk_hal_handle *h);
};

/** 由公共前缀取回具体后端结构体（`base` 是首成员，转换合法）。 */
#define JSDK_HAL_CAST(type, h) ((type *)(void *)(h))

/** 取后端类别（允许 NULL）。 */
jsdk_hal_kind_t jsdk_hal_kind_of(const struct jsdk_hal_handle *h);

/** 后端类别名（"virtual" / "socketcan" / "pcan" / "slcan" / "none"）。 */
const char *jsdk_hal_kind_name(jsdk_hal_kind_t k);

#ifdef __cplusplus
}
#endif

#endif /* JSDK_HAL_HANDLE_H */

/* ======================== src/hal/hal_slcan_codec.h ======================== */
/**
 * @file    hal_slcan_codec.h
 * @brief   slcan（ASCII 行协议）编解码 —— **纯逻辑，无平台依赖**
 *
 * @par 为什么单独一个文件
 *  slcan 的编解码是整个后端里唯一"有判断逻辑"的部分（帧格式、转义、DL、
 *  RTR/EFF 标志、错误行）。把它和串口 I/O 分开，就能**在没有硬件的机器上**
 *  用单元测试把格式钉死；剩下的 open/read/write 只是系统调用搬运。
 *
 * @par 协议要点（Lawicel slcan + CANable 2.0 的 FD 扩展）
 *  - CAN 2.0 数据帧：`t` + 3 位十六进制 ID（标准帧）
 *                     `T` + 8 位十六进制 ID（扩展帧）
 *  - 可选 DLC 字符 `0..8`，随后是十六进制载荷。**DLC 省略 = 0 字节**
 *    （Lawicel 规范里 DLC 可省略；要发 8 字节必须写 `8` 并跟上 16 个十六进制字符）
 *  - 行尾必须是 `\r`（部分固件收 `\n` 也能跑，但**发送**要用 `\r`）
 *  - 远端 RTR 帧：`r`/`R` 前缀；本 SDK 不使用，但**必须能解析并拒绝**，
 *    否则会把 RTR 帧当成数据帧交给协议层
 *  - 应答/错误行：`\r`（仅回车，表示 OK）、`\a`（BELL，表示失败）
 *
 * @par CAN FD 帧（**CANable 2.0 固件扩展**，Lawicel 原版没有）
 *  四个前缀字母，**小写 = 标准帧，大写 = 扩展帧**：
 *
 *  | 前缀 | ID   | BRS |
 *  |------|------|-----|
 *  | `d`  | 标准 | 0   |
 *  | `D`  | 扩展 | 0   |
 *  | `b`  | 标准 | 1   |
 *  | `B`  | 扩展 | 1   |
 *
 *  ⚠ **`b/B` 是“带 BRS”，不是“不带”** —— 这一点反直觉，但以 CANable 2.0 固件
 *    与 python-can 的 `can/interfaces/slcan.py` 为准（本 SDK 早期版本据此把
 *    slcan 判定为“不支持 FD”，是错的）。
 *
 *  FD 帧的载荷长度由紧跟 ID 的**一位十六进制 FD DLC 码**决定：
 *  `0..8` → 0..8 字节，`9`→12、`A`→16、`B`→20、`C`→24、`D`→32、`E`→48、`F`→64。
 *  即 **9/10/11 字节在 FD 里无法表示**（下一个可用长度直接跳到 12）。
 *  载荷**必须写满** DLC 码对应的字节数（不写满 = 语法错误）。
 *
 *  @note **已知有损点（与 python-can 相同）**：slcan 的字母表里没有 ESI（错误状态
 *        指示位）与 FD 的远程帧，所以这两样信息在收发时**丢弃**。本协议不使用它们
 *        （MIT/参数/描述符帧均为无 ESI 的数据帧）。
 *  @note 长度 9/10/11 字节在 FD 里无对应码：编码**返回 0**、解码报 `MALFORMED`。
 *        SDK 自己的帧长（MIT 8 B、描述符 64 B）都在表内。
 */

#ifndef JSDK_HAL_SLCAN_CODEC_H
#define JSDK_HAL_SLCAN_CODEC_H



#ifdef __cplusplus
extern "C" {
#endif

/**
 * 一条 slcan 文本行最坏情况长度（含 `\r` 与结尾 NUL）。
 *
 * 最坏是 FD 扩展帧：`D` + 8 位 ID + 1 位 DLC + 128 个十六进制字符 + `\r` = 139。
 * 取 144 留余量，且是 8 的倍数（便于在结构体里排布）。
 */
#define JSDK_SLCAN_LINE_MAX 144u

/** 解码结果。 */
typedef enum {
    JSDK_SLCAN_OK        = 0,   /**< 解出一帧（数据帧；FD 与 Classic 都可能） */
    JSDK_SLCAN_ACK       = 1,   /**< `\r`：固件确认 */
    JSDK_SLCAN_NACK      = 2,   /**< `\a`：固件/总线错误 */
    JSDK_SLCAN_RTR       = 3,   /**< 合法但是远端请求帧（本协议不用） */
    JSDK_SLCAN_MALFORMED = 4,   /**< 语法错误 */
    JSDK_SLCAN_NEED_MORE = 5    /**< 行未结束（未见到 `\r`），继续喂 */
} jsdk_slcan_rc_t;

/**
 * CAN FD 的 DLC 码 → 字节数。
 * @param code 0..15
 * @return 字节数（0..64）；@p code 越界返回 -1
 */
int jsdk_slcan_fd_len_from_dlc(unsigned code);

/**
 * 字节数 → CAN FD 的 DLC 码（上一函数的逆）。
 * @param len 0..64
 * @return DLC 码 0..15；**无法表示的长度（9/10/11 与 >64）返回 -1**
 */
int jsdk_slcan_fd_dlc_from_len(unsigned len);

/**
 * 把一帧编码成 slcan 行（含 `\r`，**不含** NUL）。
 *
 * 帧类型完全由 @p f 的标志位决定：
 *  - `JSDK_FRAME_EXT` → 8 位 ID 与大写前缀，否则 3 位 ID 与小写前缀；
 *  - `JSDK_FRAME_FD`  → 用 `d`/`D`（BRS=0）或 `b`/`B`（`JSDK_FRAME_BRS`，BRS=1）；
 *  - 否则用 `t`/`T` 走 Classic（此时 `len` 必须 ≤ 8）。
 *
 * @param f       待编码帧
 * @param out     输出缓冲，建议 `char out[JSDK_SLCAN_LINE_MAX]`
 * @param cap     输出缓冲容量
 * @return 写入的字节数（含 `\r`）；0 = 无法表示（FD 的 9/10/11 字节、Classic 的
 *         >8 字节）或缓冲不足 —— **绝不静默降级成另一条帧**
 */
size_t jsdk_slcan_encode(const jsdk_can_frame_t *f, char *out, size_t cap);

/**
 * 解码一行 slcan 文本。
 *
 * @param line    输入（**可以**包含尾部 `\r`；不需要 NUL 结尾，靠 @p len）
 * @param len     输入长度（解析到 `\r` 或 @p len 为止）
 * @param f       解出的帧（仅 @ref JSDK_SLCAN_OK 时有效）；FD 帧会带上
 *                `JSDK_FRAME_FD` / `JSDK_FRAME_BRS` 标志
 * @param consumed 已消费的字节数（含 `\r`）；可为 NULL
 * @return 见 @ref jsdk_slcan_rc_t
 *
 * @note `MALFORMED` 时也会设置 @p consumed，调用方据此丢弃整行后继续。
 *
 * @warning **载荷长度必须与 DLC 码严格对应**（FD 帧不补齐就不合法）。
 *  写短了会返回 `MALFORMED` 而不是“解出一条短帧”：把一帧截断的 JSON 交给
 *  描述符解析器，比当场报语法错误难查得多。
 */
jsdk_slcan_rc_t jsdk_slcan_decode(const char *line, size_t len,
                                  jsdk_can_frame_t *f, size_t *consumed);

#ifdef __cplusplus
}
#endif

#endif /* JSDK_HAL_SLCAN_CODEC_H */

/* ======================== src/hal/sim_device.h ======================== */
/**
 * @file    sim_device.h
 * @brief   虚拟驱动器行为模型（内置，仅桌面平台）
 *
 * 这是 `jsdk_hal_virtual` 内部的“固件替身”：接收主站发来的帧，按固件语义
 * 更新状态并生成应答。目的是让 SDK 在没有硬件时也能做**端到端**回归
 * （而不只是单帧编解码测试），并让客户在没有驱动器时就能开发上位机。
 *
 * @par 设计边界（重要）
 *  本模型**不是**固件的精确仿真，也不用于验证控制性能。它只保证：
 *    ① 帧级行为正确（寻址、长度校验、应答类型与 seq、错误语义、超时）
 *    ② 单位换算与固件一致（输出端/电机端、gear_ratio、torque_constant）
 *    ③ 描述符传输时序（元数据帧 + 数据帧、每毫秒帧数上限）
 *  物理模型是**玩具级**一阶模型，不可用于调参或性能评估。
 *
 * @par 端点表用真实 ID
 *  ID 与类型全部取自固件 v8 的 JSON 描述符（`tests/data/endpoints_v8.json`），
 *  因此模拟器可以直接服务真实描述符，SDK 的动态端点解析路径能被完整测到。
 *  其中 `axis0.motor.error` 是 **uint64（8 字节）**——用于测试 Classic 分段读写。
 *
 * @par 与零 malloc 核心的关系
 *  本模块位于零 malloc 核心**之外**，允许 malloc（JSON 描述符缓冲）。
 *  只在内置 HAL 被选择编译时参与构建。
 *
 * @par 行为基准
 *  `ODrive @ CyberBeast`，`can_cyberbeast.cpp`。已复刻：
 *  `is_message_for_me` 判定顺序、广播槽位 = node_id、`is_ctrl` 只含 0x00~0x03
 *  与 0x80~0x83、`break_timeout` 缺省 100 ms、`kMaxJsonFramesPerCycle = 50`、
 *  超时置 `CAN_BUS_FAILED` 后 `disarm()`、描述符元数据帧判别（看 `buf[2]`）。
 */

#ifndef JSDK_SIM_DEVICE_H
#define JSDK_SIM_DEVICE_H



#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * 容量与缺省常量
 * ------------------------------------------------------------------------ */

#define SIM_MAX_NODES     4u     /**< 同一条虚拟总线上的节点数上限 */
#define SIM_MAX_ENDPOINTS 48u    /**< 端点表容量 */
#define SIM_TX_QUEUE      1024u  /**< 出站（D→M）帧队列容量 */

#define SIM_GEAR_RATIO_DEFAULT       16.0f
#define SIM_TORQUE_CONST_DEFAULT     0.0385f
#define SIM_MIT_POS_DEFAULT          12.5f
#define SIM_MIT_VEL_DEFAULT          65.0f
#define SIM_MIT_KP_DEFAULT           500.0f
#define SIM_MIT_KD_DEFAULT           5.0f
#define SIM_MIT_TAU_DEFAULT          50.0f
#define SIM_BREAK_TIMEOUT_DEFAULT_MS 100u
#define SIM_JSON_FRAMES_PER_CYCLE    50u     /**< 固件 kMaxJsonFramesPerCycle */
#define SIM_NODE_ID_DEFAULT          1u
#define SIM_DEFAULT_VBUS             48.0f
#define SIM_DEFAULT_MOTOR_TEMP       25.0f
#define SIM_DEFAULT_FET_TEMP         30.0f

/** `Axis::ERROR_CAN_BUS_FAILED`（固件值，见 autogen/interfaces.hpp） */
#define SIM_ERR_CAN_BUS_FAILED       0x00100000u
/** `Axis::ERROR_ESTOP_REQUESTED` */
#define SIM_ERR_ESTOP_REQUESTED      0x00004000u
/** `InputMode::INPUT_MODE_MIT`（MIT 帧会把 input_mode 设为它） */
#define SIM_INPUT_MODE_MIT           9u
/** `Controller::ControlMode` */
#define SIM_CM_VOLTAGE  0u
#define SIM_CM_TORQUE   1u
#define SIM_CM_VELOCITY 2u
#define SIM_CM_POSITION 3u
/** `Axis::AxisState` */
#define SIM_AS_IDLE              1u
#define SIM_AS_CLOSED_LOOP       8u
#define SIM_AS_FULL_CALIB        3u   /* AXIS_STATE_FULL_CALIBRATION_SEQUENCE */
#define SIM_AS_HOMING           11u   /* AXIS_STATE_HOMING */
#define SIM_AS_UNDEFINED         0u

/** 标定/回零瞬时状态持续多久（ms）—— 真实固件是几百 ms 到数秒。 */
#define SIM_TRANSIENT_MS        50u

/** 端点值类型（线宽，与 JSON 描述符的 type 字符串一致） */
typedef enum {
    SIM_T_U8 = 0, SIM_T_I8, SIM_T_U16, SIM_T_I16,
    SIM_T_U32, SIM_T_I32, SIM_T_U64, SIM_T_I64,
    SIM_T_F32, SIM_T_F64, SIM_T_BOOL
} sim_val_type_t;

#define SIM_ACC_READ  0x01u
#define SIM_ACC_WRITE 0x02u

/** 端点定义：值放在 sim_node_t 内，用 offset 定位（避免逐字段写代码）。 */
typedef struct {
    uint16_t    id;
    const char *path;
    uint8_t     type;      /**< sim_val_type_t */
    uint8_t     access;    /**< SIM_ACC_* */
    size_t      offset;    /**< offsetof(sim_node_t, 字段) */
} sim_ep_def_t;

/**
 * 一个节点（轴）的状态。
 *
 * ⚠ 前一段字段被端点表按 offset 引用，**类型必须与描述符完全一致**
 *   （例如 `axis0.motor.error` 是 uint64，就不能写成 uint32）。
 */
typedef struct {
    /* ---- 端点可见字段 ---- */
    uint8_t  error_board;          /*   1  error                             u8  rw */
    float    vbus_voltage;         /*   2  vbus_voltage                      f32 r  */
    float    ibus;                 /*   3  ibus                              f32 r  */
    uint64_t serial_number;        /*   5  serial_number                     u64 r  */
    uint8_t  hw_version_major;     /*   6  hw_version_major                  u8  r  */
    uint8_t  fw_version_major;     /*   9  fw_version_major                  u8  r  */
    uint16_t break_timeout;        /*  73  can.config.break_timeout          u16 rw */
    uint32_t error_axis;           /* 138  axis0.error                       u32 rw */
    uint8_t  requested_state;      /* 143  axis0.requested_state            u8  rw */
    uint8_t  current_state;        /* 142  axis0.current_state              u8  r  实际状态 */
    float    watchdog_timeout;     /* 153  axis0.config.watchdog_timeout     f32 rw */
    uint8_t  enable_watchdog;      /* 154  axis0.config.enable_watchdog      bool rw */
    uint32_t node_id;              /* 180  axis0.config.can.node_id          u32 rw */
    uint8_t  is_extended;          /* 181  axis0.config.can.is_extended      bool rw */
    uint32_t heartbeat_rate_ms;    /* 182  axis0.config.can.heartbeat_rate_ms u32 rw */
    uint64_t error_motor;          /* 193  axis0.motor.error                 u64 rw ⚠8B */
    float    fet_temp;             /* 207  ...fet_thermistor.temperature     f32 r  */
    float    motor_temp;           /* 211  ...motor_thermistor.temperature   f32 r  */
    float    iq_measured;          /* 232  ...current_control.Iq_measured    f32 r  */
    float    gear_ratio;           /* 242  axis0.motor.config.gear_ratio     f32 rw */
    float    torque_constant;      /* 247  axis0.motor.config.torque_constant f32 rw */
    float    current_lim;          /* 249  axis0.motor.config.current_lim    f32 rw */
    float    torque_lim;           /* 251  axis0.motor.config.torque_lim     f32 rw */
                                     /*      ↳ POS/VEL 帧会持久改写它（固件副作用） */
    uint8_t  error_controller;     /* 268  axis0.controller.error            u8  rw */
    float    input_pos;            /* 270  axis0.controller.input_pos        f32 rw */
    float    input_vel;            /* 271  axis0.controller.input_vel        f32 rw */
    float    input_torque;         /* 272  axis0.controller.input_torque     f32 rw */
    uint8_t  control_mode;         /* 287  ...config.control_mode            u8  rw */
    uint8_t  input_mode;           /* 288  ...config.input_mode              u8  rw */
    float    vel_limit;            /* 301  ...config.vel_limit               f32 rw */
    float    mit_max_pos;          /* 335  ...config.mit_max_pos             f32 rw */
    float    mit_max_vel;          /* 336  ...config.mit_max_vel             f32 rw */
    float    mit_max_torque;       /* 337  ...config.mit_max_torque          f32 rw */
    float    mit_max_kp;           /* 338  ...config.mit_max_kp              f32 rw */
    float    mit_max_kd;           /* 339  ...config.mit_max_kd              f32 rw */
    uint16_t error_encoder;        /* 365  axis0.encoder.error               u16 rw */
    float    pos_estimate;         /* 372  axis0.encoder.pos_estimate        f32 r  电机端 turns */
    float    vel_estimate;         /* 378  axis0.encoder.vel_estimate        f32 r  电机端 turns/s */
    int32_t  cpr;                  /* 390  axis0.encoder.config.cpr          i32 rw */

    /* ---- 非端点字段 ---- */
    uint32_t is_fd;                /* 该节点用 CAN FD 通信 */
    uint8_t  armed;
    uint8_t  estop;
    uint8_t  life;
    uint8_t  bcast_seen;

    uint32_t last_cmd_ms;          /* is_ctrl 帧上次到达（0 = 从未收到） */
    uint32_t last_heartbeat_ms;
    uint32_t cmd_count;
    uint32_t state_change_ms;      /* current_state 最近一次**由瞬时态转定态**的时刻 */
    uint32_t transient_until_ms;   /* > now_ms 时 current_state 停在瞬时态 */
    uint8_t  settle_to;            /* 瞬时态结束后回到哪个状态 */
    uint32_t requested_hits;       /* 通过端点写入 requested_state 的次数 */
    uint32_t param_err_count;      /* 因装不下而回 ERR 的批量请求次数 */
    uint8_t  tx_seq;               /* ⚠ 设备本地滚动计数器（固件 tx_seq_[axis]++）
                                      响应帧用它，**不回显请求的 Seq** */

    /* 最近一条控制帧的解码结果（供测试断言单位换算；非端点） */
    uint8_t  last_control_msgtype;
    float    cmd_pos_out_rad;
    float    cmd_vel_out_rad_s;
    float    cmd_kp;
    float    cmd_kd;
    float    cmd_tau_out_nm;
    float    cmd_torque_motor_nm;
    float    pos_target_motor;
    float    vel_target_motor;
} sim_node_t;

/** 虚拟总线（多节点） */
typedef struct {
    sim_node_t nodes[SIM_MAX_NODES];
    size_t     n_nodes;
    uint32_t   now_ms;
    uint32_t   last_tick_ms;

    /* 出站帧队列（模型 → HAL → SDK 接收路径） */
    jsdk_can_frame_t txq[SIM_TX_QUEUE];
    uint32_t         txq_head;
    uint32_t         txq_tail;
    uint32_t         txq_dropped;

    /* 统计 */
    uint32_t rx_frames;
    uint32_t rx_for_me;
    uint32_t tx_frames;
    uint32_t bad_len_drops;
    uint32_t unhandled;

    /* JSON 描述符（0x24 / 0x25） */
    struct {
        uint8_t *json;
        uint32_t len;
        uint16_t crc;
        int      owned;
        int      active;
        int      metadata_sent;
        uint32_t offset;
        uint32_t master_id;
        uint32_t my_id;
        int      is_classic;
    } desc;

    /* 故障注入 */
    uint32_t force_txq_full;

    /**
     * 分段写（Classic `PARAM_WRITE` 多块）的装配器，**每个总线一份**。
     *
     * ⚠ 早期版本是文件级 `static g_asm[SIM_MAX_NODES]`，那样**同一进程里两条
     *   虚拟总线会共用同一组槽位**（节点下标相同就碰撞）→ 多 CAN 口测试
     *   会出现“A 总线写入的数据拼进了 B 总线”的假象。装配器属于总线状态，
     *   必须跟着总线走。
     */
    cb_param_write_asm_t asm_state[SIM_MAX_NODES];
} sim_bus_t;

/* --------------------------------------------------------------------------
 * 生命周期
 * ------------------------------------------------------------------------ */

/** 初始化总线；`n` = 0 时创建缺省单节点（node_id = 1）。 */
void sim_bus_init(sim_bus_t *b, size_t n);

/** 释放动态内存（幂等）。 */
void sim_bus_free(sim_bus_t *b);

/**
 * 按 `node_spec` 配置节点。
 *
 * 语法：`<index>[:k=v[,k=v...]][;...]`（index 从 0 开始，对应 nodes[]）
 * 键：`id` `gear` `tconst` `pmax` `vmax` `kpmax` `kdmax` `tmax` `hb`
 *     `timeout` `vb` `temp`；无值键：`fd` / `classic` / `arm` / `disarm`
 * 例：`"0:gear=16.5,pmax=12.5,vmax=65,tmax=50,fd;1:gear=8,id=2,classic"`
 *
 * @return 0 成功；-1 语法错误
 */
int sim_configure(sim_bus_t *b, const char *node_spec);

/* --------------------------------------------------------------------------
 * 收发与时间
 * ------------------------------------------------------------------------ */

/** 把一帧交给模型处理（等价于“总线上出现了一帧”）。 */
void sim_rx(sim_bus_t *b, const jsdk_can_frame_t *f);

/** 取出模型生成的下一帧。@return 1 = 取到；0 = 空。 */
int sim_tx_pop(sim_bus_t *b, jsdk_can_frame_t *f);

/**
 * 推进时间到 `now_ms` 并执行周期任务：
 * 逐毫秒推进物理模型 → 检查 `break_timeout` → 到期发心跳 →
 * 继续未完成的描述符传输（每毫秒 ≤ SIM_JSON_FRAMES_PER_CYCLE 帧）。
 */
void sim_tick(sim_bus_t *b, uint32_t now_ms);

/* --------------------------------------------------------------------------
 * JSON 描述符
 * ------------------------------------------------------------------------ */

/** 设置描述符内容（内部拷贝）。crc = 0 时用字节和占位。 */
int sim_set_desc(sim_bus_t *b, const char *json, uint32_t len, uint16_t crc);

/** 从文件读入描述符（stdio，仅桌面平台）。crc 语义同上。 */
int sim_set_desc_file(sim_bus_t *b, const char *path, uint16_t crc);

/** 描述符传输是否仍在进行。 */
int sim_desc_active(const sim_bus_t *b);

/* --------------------------------------------------------------------------
 * 端点表
 * ------------------------------------------------------------------------ */

const sim_ep_def_t *sim_default_endpoints(size_t *count_out);
const sim_ep_def_t *sim_find_ep(const sim_node_t *n, uint16_t ep_id);

/** 读端点值（线上字节序）。@return 实际字节数；未知端点返回 0。 */
uint8_t sim_ep_read(const sim_node_t *n, const sim_ep_def_t *def,
                    uint8_t *out, uint8_t cap);

/** 写端点值（线上字节序）。@return 0 成功；-1 只读或长度不符。 */
int sim_ep_write(sim_node_t *n, const sim_ep_def_t *def,
                 const uint8_t *in, uint8_t len);

/** 端点值的线宽（字节）；未知返回 0。 */
uint8_t sim_ep_width(uint8_t type);

/* --------------------------------------------------------------------------
 * 便捷查询
 * ------------------------------------------------------------------------ */

/** 按当前 node_id 查找节点（注意 node_id 可能被主站改写）。 */
sim_node_t *sim_find_node(sim_bus_t *b, uint32_t node_id);

/** 清空统计与故障注入（不清节点状态）。 */
void sim_clear_stats(sim_bus_t *b);

#ifdef __cplusplus
}
#endif

#endif /* JSDK_SIM_DEVICE_H */

/* ======================== src/hal/hal_virtual_internal.h ======================== */
/**
 * @file    hal_virtual_internal.h
 * @brief   `jsdk_hal_virtual` 的内部接口（**仅供本工程测试使用**，不安装）
 *
 * 设备模型 `sim_device.h` 并不适合塞进公共头文件（它是桌面测试专用的实现细节），
 * 但测试需要直接检视/操纵模型内部状态（例如断言单位换算结果、检查超时后的
 * 位、配置心跳周期）。因此用这个内部头把桥接函数暴露出来。
 */

#ifndef JSDK_HAL_VIRTUAL_INTERNAL_H
#define JSDK_HAL_VIRTUAL_INTERNAL_H


#ifdef __cplusplus
extern "C" {
#endif

/**
 * 取出虚拟后端内部的设备模型。仅在本工程测试中使用。
 * @return 模型指针；@p h 为空时返回 NULL
 */
sim_bus_t *jsdk_hal_virtual_sim(jsdk_hal_handle_t *h);

#ifdef __cplusplus
}
#endif

#endif /* JSDK_HAL_VIRTUAL_INTERNAL_H */

/* ======================== src/proto_cyberbeast/cb_ctrl.c ======================== */
/**
 * @file    cb_ctrl.c
 * @brief   CYBERBEAST 实时控制帧编解码实现（见 cb_ctrl.h）
 *
 * 逐字节与固件 Firmware/communication/can/can_cyberbeast.cpp 的
 * cmd_pos_control / cmd_vel_control / cmd_torque_control / cmd_current_control
 * 对齐；打包侧额外做钳位与标志上报（固件不钳位）。
 */


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

/* ======================== src/proto_cyberbeast/cb_desc_cache.c ======================== */
/**
 * @file    cb_desc_cache.c
 * @brief   描述符缓存导出/导入实现（见 cb_desc_cache.h）
 */



/* ==========================================================================
 * 字节序工具（缓存固定小端，与 CPU 无关）
 * ======================================================================== */

static uint16_t rd16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
}

static void wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

/* ==========================================================================
 * CRC-16/CCITT-FALSE（poly 0x1021，init 0xFFFF）
 * 逐位实现，不占 ROM 表；描述符缓存只有几十 KB，速度不成问题。
 * ======================================================================== */

uint16_t cb_desc_cache_crc16(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint16_t crc = 0xFFFFu;
    size_t i;
    int b;

    if (!p) return 0u;

    /*
     * 内部全程用 `unsigned` 运算，每轮显式截回 16 位（与原来的 `(uint16_t)` 转换
     * 逐位等价）。
     *
     * ⚠ 为什么必须写显式转换：`uint16_t` 参与移位/异或时会整型提升 `int`，
     *   之后与无符号常量相遇就产生 `int → unsigned` 的隐式符号转换。
     *   GCC 的 `-Wsign-conversion` 在这条上**跟优化级别有关**：
     *   `-O2` 的值域分析能证明"掩码后必然非负"而不报，加上
     *   `-fsanitize=undefined`（为插桩降低优化）后就证不出来、开始报错。
     *   写死 `unsigned` 后，任何优化级别、任何 sanitizer 组合都零告警
     *   —— 这直接决定了 `-Werror` 的 sanitizer 构建能不能跑起来。
     */
    for (i = 0u; i < len; ++i) {
        unsigned c = (unsigned)crc;

        c ^= ((unsigned)p[i] << 8);
        for (b = 0; b < 8; ++b) {
            c = ((c & 0x8000u) != 0u) ? ((c << 1) ^ 0x1021u) : (c << 1);
            c &= 0xFFFFu;                       /* 逐位 16 位截断（与原实现一致） */
        }
        crc = (uint16_t)c;
    }
    return crc;
}

/* ==========================================================================
 * 失效键
 * ======================================================================== */

/* FNV-1a 32 位 */
#define FNV_OFFSET 2166136261u
#define FNV_PRIME  16777619u

static uint32_t fnv_bytes(uint32_t h, const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    size_t i;
    for (i = 0u; i < len; ++i) {
        h ^= (uint32_t)p[i];
        h *= FNV_PRIME;
    }
    return h;
}

static uint32_t fnv_u32(uint32_t h, uint32_t v)
{
    uint8_t b[4];
    wr32(b, v);
    return fnv_bytes(h, b, 4u);
}

/* 配置上限的“生效值”：0 表示使用内置默认值（见 joint_sdk.h）。
   哈希与缓存头都必须用生效值，否则“用默认值导出、用**显式**默认值导入”
   会被误判为配置不符（这是一个很容易踩的坑）。 */
uint32_t cb_desc_eff_max_endpoints(const jsdk_desc_config_t *cfg)
{
    return cfg->max_endpoints ? (uint32_t)cfg->max_endpoints : 2048u;
}

uint32_t cb_desc_eff_max_path_len(const jsdk_desc_config_t *cfg)
{
    return cfg->max_path_len ? (uint32_t)cfg->max_path_len : 128u;
}

uint32_t cb_desc_filter_hash(const jsdk_desc_config_t *cfg)
{
    uint32_t h = FNV_OFFSET;
    unsigned i;

    if (!cfg) return 0u;

    /* 线格式版本：SDK 升级导致内部表示变化时，缓存必须失效 */
    h = fnv_u32(h, CB_DESC_CACHE_VERSION);
    h = fnv_u32(h, (uint32_t)cfg->retain);
    h = fnv_u32(h, cb_desc_eff_max_endpoints(cfg));
    h = fnv_u32(h, cb_desc_eff_max_path_len(cfg));
    h = fnv_u32(h, (uint32_t)cfg->filter_count);

    /* filter 列表按**顺序**参与：顺序变了就视为不同的键（保守但简单） */
    if (cfg->filter_paths) {
        for (i = 0u; i < cfg->filter_count; ++i) {
            const char *s = cfg->filter_paths[i];
            size_t n = s ? strlen(s) : 0u;
            h = fnv_u32(h, (uint32_t)n);
            if (n) h = fnv_bytes(h, s, n);
        }
    }
    return h;
}

/* ==========================================================================
 * 导出
 * ======================================================================== */

static size_t path_len_in_arena(const jsdk_ep_store_t *store, unsigned idx)
{
    const char *p = jsdk_ep_store_path(store, idx);
    return p ? (strlen(p) + 1u) : 0u;
}

size_t cb_desc_cache_size(const jsdk_ep_store_t *store)
{
    size_t total = CB_DESC_CACHE_HDR_LEN;
    unsigned n, i;

    if (!store) return CB_DESC_CACHE_HDR_LEN;

    n = jsdk_ep_store_count(store);
    for (i = 0u; i < n; ++i) {
        total += CB_DESC_CACHE_REC_HDR + path_len_in_arena(store, i);
    }
    return total;
}

int cb_desc_cache_export(const jsdk_desc_config_t *cfg, const jsdk_ep_store_t *store,
                         int early, void *buf, size_t cap, size_t *out_len)
{
    uint8_t *b = (uint8_t *)buf;
    size_t need, body_len = 0u;
    unsigned n, i;
    uint8_t *body;
    size_t off;
    uint32_t path_bytes = 0u;

    if (out_len) *out_len = 0u;
    if (!cfg || !store || !buf) return JSDK_ERR_INVALID_ARG;

    need = cb_desc_cache_size(store);
    if (cap < need) return JSDK_ERR_NO_MEMORY;   /* 调用方扩容后重试 */

    n = jsdk_ep_store_count(store);

    /* --- 头 --- */
    wr32(b + 0u, CB_DESC_CACHE_MAGIC);
    wr16(b + 4u, (uint16_t)CB_DESC_CACHE_VERSION);
    wr16(b + 6u, (uint16_t)CB_DESC_CACHE_HDR_LEN);
    wr16(b + 8u, 0u);                                   /* desc_crc：由 set_fw/后续补写 */
    wr16(b + 10u, (uint16_t)((cfg->retain == JSDK_DESC_RETAIN_FILTERED
                              ? CB_DESC_CACHE_F_FILTERED : 0u)
                             | (early ? CB_DESC_CACHE_F_EARLY : 0u)));
    wr32(b + 12u, 0u);                                  /* fw_version：由 set 补写 */
    wr32(b + 16u, cb_desc_filter_hash(cfg));
    wr32(b + 20u, cb_desc_eff_max_endpoints(cfg));
    wr16(b + 24u, (uint16_t)cb_desc_eff_max_path_len(cfg));
    wr16(b + 26u, 0u);
    wr32(b + 28u, (uint32_t)n);
    wr32(b + 36u, 0u);                                  /* body_len：下面填 */
    wr32(b + 40u, 0u);                                  /* body_crc：下面填 */
    wr32(b + 44u, 0u);

    /* --- 体 --- */
    body = b + CB_DESC_CACHE_HDR_LEN;
    off = 0u;
    for (i = 0u; i < n; ++i) {
        const char *path = jsdk_ep_store_path(store, i);
        uint16_t ep_id = 0u;
        jsdk_ep_type_t type = JSDK_EP_OBJECT;
        uint8_t access = 0u;
        size_t plen;

        if (jsdk_ep_store_at(store, i, NULL, &ep_id, &type, &access) != JSDK_OK) {
            return JSDK_ERR_INVALID_ARG;
        }
        plen = path ? (strlen(path) + 1u) : 0u;
        if (plen == 0u || plen > 0xFFFFu) return JSDK_ERR_INVALID_ARG;

        wr16(body + off + 0u, ep_id);
        body[off + 2u] = (uint8_t)type;
        body[off + 3u] = access;
        wr16(body + off + 4u, (uint16_t)plen);
        if (plen) memcpy(body + off + CB_DESC_CACHE_REC_HDR, path, plen);
        off += CB_DESC_CACHE_REC_HDR + plen;
        path_bytes += (uint32_t)plen;
    }
    body_len = off;

    wr32(b + 32u, path_bytes);
    wr32(b + 36u, (uint32_t)body_len);
    wr32(b + 40u, (uint32_t)cb_desc_cache_crc16(body, body_len));

    if (out_len) *out_len = CB_DESC_CACHE_HDR_LEN + body_len;
    return JSDK_OK;
}

/* ==========================================================================
 * 头部查看 / 补写
 * ======================================================================== */

/** 头部合法性（不含失效键与体校验）。 */
static int hdr_check(const uint8_t *b, size_t len)
{
    if (!b || len < CB_DESC_CACHE_HDR_LEN)             return JSDK_ERR_INVALID_ARG;
    if (rd32(b + 0u) != CB_DESC_CACHE_MAGIC)           return JSDK_ERR_PROTOCOL;
    if (rd16(b + 4u) != (uint16_t)CB_DESC_CACHE_VERSION) return JSDK_ERR_PROTOCOL;
    if (rd16(b + 6u) != (uint16_t)CB_DESC_CACHE_HDR_LEN) return JSDK_ERR_PROTOCOL;
    return JSDK_OK;
}

int cb_desc_cache_peek(const void *buf, size_t len, cb_desc_cache_meta_t *out)
{
    const uint8_t *b = (const uint8_t *)buf;
    int rc = hdr_check(b, len);

    if (rc != JSDK_OK) return rc;
    if (out) {
        out->desc_crc       = rd16(b + 8u);
        out->flags          = rd16(b + 10u);
        out->fw_version     = rd32(b + 12u);
        out->endpoint_count = rd32(b + 28u);
    }
    return JSDK_OK;
}

int cb_desc_cache_set_fw_version(void *buf, size_t len, uint32_t fw_version)
{
    uint8_t *b = (uint8_t *)buf;
    int rc = hdr_check(b, len);

    if (rc != JSDK_OK) return rc;
    wr32(b + 12u, fw_version);
    return JSDK_OK;
}

/** 设置描述符 VersionCRC（导出后再补写，避免导出时还要查设备）。 */
int cb_desc_cache_set_desc_crc(void *buf, size_t len, uint16_t crc);
int cb_desc_cache_set_desc_crc(void *buf, size_t len, uint16_t crc)
{
    uint8_t *b = (uint8_t *)buf;
    int rc = hdr_check(b, len);

    if (rc != JSDK_OK) return rc;
    wr16(b + 8u, crc);
    return JSDK_OK;
}

/* ==========================================================================
 * 导入
 * ======================================================================== */

int cb_desc_cache_import(const jsdk_desc_config_t *cfg, jsdk_ep_store_t *store,
                         const void *buf, size_t len)
{
    const uint8_t *b = (const uint8_t *)buf;
    const uint8_t *body;
    uint32_t path_bytes, body_len, body_crc, n, i;
    size_t off;
    uint32_t got_path_bytes = 0u;
    int rc;

    if (!cfg || !store) return JSDK_ERR_INVALID_ARG;

    /* 先清空：任何失败都不留部分结果 */
    if (store->arena.base && store->arena.size) jsdk_ep_store_reset(store);

    rc = hdr_check(b, len);
    if (rc != JSDK_OK) return rc;

    /* --- 失效键：三者任一不匹配都拒绝（这是路线 A 的固有限制） --- */
    {
        uint16_t want = (uint16_t)((cfg->retain == JSDK_DESC_RETAIN_FILTERED
                                    ? CB_DESC_CACHE_F_FILTERED : 0u)
                                   | (rd16(b + 10u) & CB_DESC_CACHE_F_EARLY));
        if (rd16(b + 10u) != want)               return JSDK_ERR_BAD_STATE;
    }
    if (rd32(b + 16u) != cb_desc_filter_hash(cfg))  return JSDK_ERR_BAD_STATE;
    if (rd32(b + 20u) != cb_desc_eff_max_endpoints(cfg)) return JSDK_ERR_BAD_STATE;
    if (rd16(b + 24u) != (uint16_t)cb_desc_eff_max_path_len(cfg)) return JSDK_ERR_BAD_STATE;

    n          = rd32(b + 28u);
    path_bytes = rd32(b + 32u);
    body_len   = rd32(b + 36u);
    body_crc   = rd32(b + 40u);

    if (n > 0xFFFFu) return JSDK_ERR_PROTOCOL;                 /* ep_id 是 u16 */
    if ((size_t)CB_DESC_CACHE_HDR_LEN + body_len > len) return JSDK_ERR_PROTOCOL;

    body = b + CB_DESC_CACHE_HDR_LEN;
    if (cb_desc_cache_crc16(body, body_len) != (uint16_t)body_crc) {
        return JSDK_ERR_PROTOCOL;                              /* Flash 位翻转 */
    }

    /* --- 第一遍：只做边界校验，先不碰 arena --- */
    off = 0u;
    for (i = 0u; i < n; ++i) {
        uint16_t plen;
        if (off + CB_DESC_CACHE_REC_HDR > body_len) return JSDK_ERR_PROTOCOL;
        plen = rd16(body + off + 4u);
        if (plen == 0u || plen > JSDK_EP_PATH_MAX_HARD) return JSDK_ERR_PROTOCOL;
        if (off + CB_DESC_CACHE_REC_HDR + plen > body_len) return JSDK_ERR_PROTOCOL;
        got_path_bytes += (uint32_t)plen;
        off += CB_DESC_CACHE_REC_HDR + plen;
    }
    if (off != body_len)          return JSDK_ERR_PROTOCOL;
    if (got_path_bytes != path_bytes) return JSDK_ERR_PROTOCOL;

    /* --- 第二遍：写 arena --- */
    off = 0u;
    for (i = 0u; i < n; ++i) {
        uint16_t ep_id = rd16(body + off + 0u);
        uint8_t  type  = body[off + 2u];
        uint8_t  access= body[off + 3u];
        uint16_t plen  = rd16(body + off + 4u);
        const char *path = (const char *)(body + off + CB_DESC_CACHE_REC_HDR);

        if (jsdk_ep_arena_put(&store->arena, path, plen, ep_id, type, access)
                != JSDK_OK) {
            jsdk_ep_store_reset(store);
            return JSDK_ERR_NO_MEMORY;
        }
        store->parsed_total++;      /* 缓存里的条目都算“解析到过” */
        off += CB_DESC_CACHE_REC_HDR + plen;
    }

    /* 恢复上限：导入后继续解析新数据时应沿用当前 cfg（而非缓存里的旧值） */
    store->max_endpoints = cfg->max_endpoints ? cfg->max_endpoints : 2048u;
    return JSDK_OK;
}

/* ==========================================================================
 * 路线 B
 * ======================================================================== */

int cb_desc_import_raw(const jsdk_desc_config_t *cfg, jsdk_ep_store_t *store,
                       const void *json, size_t len)
{
    if (!cfg || !store || !json || len == 0u) return JSDK_ERR_INVALID_ARG;
    return jsdk_jsondesc_run(cfg, store, json, len);
}

/* ======================== src/proto_cyberbeast/cb_frame.c ======================== */
/**
 * @file    cb_frame.c
 * @brief   CYBERBEAST 帧层原语实现（见 cb_frame.h）
 */



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

/* ======================== src/proto_cyberbeast/cb_heartbeat.c ======================== */
/**
 * @file    cb_heartbeat.c
 * @brief   CYBERBEAST 心跳帧编解码实现（见 cb_heartbeat.h）
 *
 * 与固件 can_cyberbeast.cpp 的 send_heartbeat() 逐字节对齐。
 */


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

/* ======================== src/proto_cyberbeast/cb_jsondesc_fetch.c ======================== */
/**
 * @file    cb_jsondesc_fetch.c
 * @brief   JSON 端点描述符下载传输状态机（见 cb_jsondesc_fetch.h）
 *
 * 与固件 `cmd_json_desc_read()` / `send_json_desc_chunk()` 逐字段对齐。
 */



/* ==========================================================================
 * 内部工具
 * ======================================================================== */

/** 小端 u32（描述符的 offset / totalLen 都用 LE，与其它帧的 BE 相反） */
static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

/** 记录失败并结束传输（首个原因优先）。 */
static int dfail(cb_desc_fetch_t *f, const char *msg, int code)
{
    if (!f->failed) {
        f->failed   = 1;
        f->err      = msg;
        f->fail_rc  = code;
        f->done     = 1;
        f->complete = 0;
    }
    return code;
}

/** 进度上报（节流：每 ≥4 KB 或按调用方要求立即上报） */
static void report_progress(cb_desc_fetch_t *f, int force)
{
    if (!f->progress) return;
    if (!force && (f->bytes_scanned - f->last_report) < 4096u) return;
    f->last_report = f->bytes_scanned;
    f->progress(f->ctx, f->bytes_scanned, f->total_len, f->progress_user);
}

/* ==========================================================================
 * 生命周期
 * ======================================================================== */

int cb_desc_fetch_init(cb_desc_fetch_t *f, jsdk_context_t *ctx,
                       const jsdk_desc_config_t *cfg, jsdk_ep_store_t *store)
{
    int rc;

    if (!f || !cfg || !store) return JSDK_ERR_INVALID_ARG;

    memset(f, 0, sizeof *f);
    f->ctx   = ctx;
    f->cfg   = cfg;
    f->store = store;

    rc = jsdk_jsondesc_init(&f->parser, cfg, store);
    if (rc != JSDK_OK) {
        (void)dfail(f, CB_DESC_ERR_PARSE, rc);
    }

    /* 提前终止只对“全部精确路径”的 filter 安全，见头文件说明。
       含通配时宁可多扫几百帧，也不交付依赖 JSON 字段顺序的残缺端点表。 */
    f->stop_allowed = (uint8_t)(cfg->stop_when_satisfied
                                && cb_desc_filters_all_exact(cfg));
    return rc;
}

void cb_desc_fetch_set_raw_sink(cb_desc_fetch_t *f, jsdk_desc_raw_sink_fn fn,
                                void *user)
{
    if (!f) return;
    f->sink      = fn;
    f->sink_user = user;
    if (!fn) f->raw_sink_failed = 1u;      /* 未安装 = raw 缓存不可用 */
}

void cb_desc_fetch_set_progress(cb_desc_fetch_t *f, jsdk_desc_progress_fn fn,
                                void *user)
{
    if (!f) return;
    f->progress      = fn;
    f->progress_user = user;
}

uint32_t cb_desc_timeout_ms(const jsdk_desc_config_t *cfg)
{
    if (cfg && cfg->timeout_ms) return cfg->timeout_ms;
    return 5000u;
}

int cb_desc_filters_all_exact(const jsdk_desc_config_t *cfg)
{
    unsigned i;

    if (!cfg || cfg->filter_count == 0u || !cfg->filter_paths) return 1;

    for (i = 0u; i < cfg->filter_count; ++i) {
        const char *s = cfg->filter_paths[i];
        size_t n;
        if (!s || s[0] == '\0') continue;
        n = strlen(s);
        /* `prefix*`（含全文通配 "*"）或 `segment.` 都算通配 → 不允许提前终止 */
        if (s[n - 1u] == '*' || s[n - 1u] == '.') return 0;
    }
    return 1;
}

/* ==========================================================================
 * 请求构造
 * ======================================================================== */

size_t cb_desc_build_request(uint8_t *dst, size_t cap, uint32_t offset)
{
    if (!dst || cap < CB_DESC_REQ_LEN) return 0u;
    put_le32(dst, offset);
    return CB_DESC_REQ_LEN;
}

int cb_desc_frame_len_valid(size_t len)
{
    return (len == CB_DESC_FRAME_LEN_CLASSIC || len == CB_DESC_FRAME_LEN_FD) ? 1 : 0;
}

/* ==========================================================================
 * 帧处理
 * ======================================================================== */

/** 元数据帧：解析 total_len / crc 并做全部合法性校验。 */
static int take_metadata(cb_desc_fetch_t *f, const uint8_t *payload, size_t len)
{
    uint32_t total;

    if (len < CB_DESC_META_MIN_LEN) {
        return dfail(f, CB_DESC_ERR_FRAME_LEN, JSDK_ERR_PROTOCOL);
    }

    /* 元数据帧的头两字节恒为 0 */
    if (payload[0] != 0u || payload[1] != 0u) {
        return dfail(f, CB_DESC_ERR_META_EXPECTED, JSDK_ERR_PROTOCOL);
    }

    /* 交叉校验：若第 3 字节是 JSON 起始符，说明这其实是 offset = 0 的数据帧，
       即设备跳过了元数据帧。此时无法得知 total_len，只能整体失败。 */
    if (payload[2] == CB_DESC_JSON_FIRST_BYTE) {
        return dfail(f, CB_DESC_ERR_META_EXPECTED, JSDK_ERR_PROTOCOL);
    }

    total = le32(payload + 2);
    if (total == 0u || total > CB_DESC_MAX_TOTAL_LEN) {
        /* ⚠ > 65535 必须硬拒绝：chunkOffset 只有 u16，固件会静默回绕 */
        return dfail(f, CB_DESC_ERR_META_TOTAL, JSDK_ERR_PROTOCOL);
    }

    f->total_len   = total;
    f->crc         = le16(payload + 6);
    f->next_offset = 0u;
    f->started     = 1u;
    f->last_report = 0u;
    report_progress(f, 1);
    return JSDK_OK;
}

/** 数据帧：tee → 解析 → 推进偏移 → 判定结束。 */
static int take_data(cb_desc_fetch_t *f, const uint8_t *payload, size_t len)
{
    uint32_t off;
    uint32_t avail;
    uint32_t n;
    int rc;

    off   = (uint32_t)le16(payload);
    avail = f->total_len - f->next_offset;

    if (off != f->next_offset) {
        return dfail(f, CB_DESC_ERR_OFFSET, JSDK_ERR_PROTOCOL);
    }

    n = (uint32_t)(len - CB_DESC_DATA_HDR_BYTES);
    if (n > avail) n = avail;              /* 末帧尾部是补零 */

    if (n > 0u) {
        /* ① 先 tee 再解析（公共 API 承诺：sink 拿到的是**解析之前**的字节） */
        if (f->sink && !f->raw_sink_failed) {
            if (f->sink(f->ctx, payload + CB_DESC_DATA_HDR_BYTES, n,
                        f->next_offset, f->sink_user) != 0) {
                f->raw_sink_failed = 1u;   /* 放弃 tee，但下载继续 */
            } else {
                f->sink_called = 1u;
            }
        }

        /* ② 增量解析 */
        rc = jsdk_jsondesc_feed(&f->parser, payload + CB_DESC_DATA_HDR_BYTES, n);
        if (rc != JSDK_OK) {
            if (rc == JSDK_ERR_NO_MEMORY) {
                f->arena_full = 1u;
                return dfail(f, CB_DESC_ERR_ARENA, JSDK_ERR_NO_MEMORY);
            }
            /* 详细原因由 jsdk_jsondesc_error() 给出（如“路径超过上限”）*/
            if (jsdk_jsondesc_error(&f->parser)) f->err_detail = jsdk_jsondesc_error(&f->parser);
            return dfail(f, CB_DESC_ERR_PARSE, JSDK_ERR_PARSE);
        }

        f->next_offset   += n;
        f->bytes_scanned += n;
    }

    /* ③ 收满 → 解析器必须能正常收尾，否则视为描述符损坏。
       ⚠ 必须先于“提前终止”判定：若最后一个 filter 恰好在**末帧**才满足，
          先判提前终止会把一次**完整**下载误标成 stopped_early（complete=0），
          进而让 `cache_safe()` 无谓地否掉一份可用的 raw 缓存。 */
    if (f->next_offset >= f->total_len) {
        rc = jsdk_jsondesc_finish(&f->parser);
        if (rc != JSDK_OK) {
            if (rc == JSDK_ERR_NO_MEMORY) {
                f->arena_full = 1u;
                return dfail(f, CB_DESC_ERR_ARENA, JSDK_ERR_NO_MEMORY);
            }
            return dfail(f, CB_DESC_ERR_PARSE, JSDK_ERR_PARSE);
        }
        f->complete = 1u;
        f->done     = 1u;
        if (!f->sink) f->raw_sink_failed = 1u;   /* 没装 sink → raw 缓存不可用 */
        report_progress(f, 1);
        return JSDK_OK;
    }

    /* ④ 提前终止：filter 全部命中就不再等剩下的帧（设备会继续发，由调用方丢弃）。
       ⚠ 仅在全部 filter 为精确路径时允许——前缀 filter 会被首个匹配项“满足”，
          提前停止会静默丢掉同前缀家族的其余路径。 */
    if (f->stop_allowed && jsdk_jsondesc_satisfied(&f->parser)) {
        f->stopped_early = 1u;
        f->done          = 1u;
        f->complete      = 0u;
        report_progress(f, 1);
    }

    return JSDK_OK;
}

int cb_desc_fetch_frame(cb_desc_fetch_t *f, const uint8_t *payload, size_t len)
{
    if (!f || !payload) return JSDK_ERR_INVALID_ARG;

    if (f->failed) return f->fail_rc ? f->fail_rc : JSDK_ERR_PARSE;
    /* 已提前终止后设备还会继续发帧；静默忽略，不算错 */
    if (f->done) return JSDK_OK;

    if (!cb_desc_frame_len_valid(len)) {
        return dfail(f, CB_DESC_ERR_FRAME_LEN, JSDK_ERR_PROTOCOL);
    }

    f->frames_rx++;

    if (!f->started) {
        return take_metadata(f, payload, len);
    }
    if (len < CB_DESC_DATA_HDR_BYTES) {
        return dfail(f, CB_DESC_ERR_FRAME_LEN, JSDK_ERR_PROTOCOL);
    }
    return take_data(f, payload, len);
}

/* ==========================================================================
 * 查询
 * ======================================================================== */

int cb_desc_fetch_is_done(const cb_desc_fetch_t *f)
{
    return (f && f->done) ? 1 : 0;
}

int cb_desc_fetch_is_ok(const cb_desc_fetch_t *f)
{
    if (!f || f->failed) return 0;
    if (f->complete) return 1;
    if (f->stopped_early) return jsdk_jsondesc_satisfied(&f->parser);
    return 0;
}

int cb_desc_fetch_cache_safe(const cb_desc_fetch_t *f)
{
    if (!f || f->failed)             return 0;
    if (!f->complete)                return 0;   /* 被提前终止 → 数据不完整 */
    if (f->raw_sink_failed)          return 0;   /* sink 报错或未安装 */
    if (!f->sink_called)             return 0;   /* 一次都没成功写出 */
    if (f->bytes_scanned != f->total_len) return 0;   /* 字节数对不上 */
    return 1;
}

const char *cb_desc_fetch_error(const cb_desc_fetch_t *f)
{
    if (!f) return CB_DESC_ERR_PARSE;
    return f->err;
}

const char *cb_desc_fetch_detail(const cb_desc_fetch_t *f)
{
    return f ? f->err_detail : (const char *)0;
}

void cb_desc_fetch_result(const cb_desc_fetch_t *f, cb_desc_fetch_result_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof *out);
    if (!f) return;

    out->total_len     = f->total_len;
    out->crc           = f->crc;
    out->bytes_scanned = f->bytes_scanned;
    out->frames_rx     = f->frames_rx;
    out->complete      = f->complete;
    out->stopped_early = f->stopped_early;
    out->stop_allowed  = f->stop_allowed;
    out->raw_sink_failed = f->raw_sink_failed;
    out->failed        = f->failed;

    if (f->store) {
        out->endpoint_count = jsdk_ep_store_count(f->store);
        out->parsed_total   = f->store->parsed_total;
        out->arena_used = (size_t)f->store->arena.entry_count * sizeof(jsdk_ep_entry_t)
                        + f->store->arena.blob_used;
    }
}

unsigned cb_desc_fetch_pct(const cb_desc_fetch_t *f)
{
    if (!f || f->total_len == 0u) return 0u;
    if (f->bytes_scanned >= f->total_len) return 100u;
    return (unsigned)(((uint64_t)f->bytes_scanned * 100u) / f->total_len);
}

/* ======================== src/proto_cyberbeast/cb_jsondesc_parse.c ======================== */
/**
 * @file    cb_jsondesc_parse.c
 * @brief   CYBERBEAST JSON 端点描述符的**增量解析器** + 端点存储（arena）
 *
 * 为什么不用现成 JSON 库：描述符实测 41029 字节 / 594 个端点，
 * 受限 MCU 既拿不出 41 KB 缓冲，也不该为它引入动态分配与递归解析器。
 * 本实现：
 *   - 逐字节状态机（可 62 字节/帧地喂入），**无递归、无 malloc**；
 *   - 解析结果写进调用者提供的 arena（条目区向前、路径池向后增长）；
 *   - 路径在解析 name 字段时增量拼接，故 key 与 id/type 的先后顺序无关；
 *   - 任何异常（截断/非法字符/未知 type/超限）→ 整体失败，**不留部分结果**。
 *
 * 协议细节见 docs/PROTOCOL_NOTES.zh-CN.md §5.6 / §9.2。
 */


/* ==========================================================================
 * 小工具
 * ======================================================================== */

static int is_ws(uint8_t c)
{
    return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

static int is_digit(uint8_t c) { return c >= '0' && c <= '9'; }
static int is_alpha(uint8_t c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); }

static int jfail(jsdk_jsondesc_t *p, const char *msg)
{
    if (p->state != JSDK_JS_FAIL) {
        p->state = JSDK_JS_FAIL;
        p->err = msg;
        p->fail_code = JSDK_ERR_PARSE;
        /* 保证“失败不留部分结果”：调用者即使误用 store 也读不到任何东西 */
        if (p->store) {
            p->store->arena.entry_count = 0;
            p->store->arena.blob_top    = p->store->arena.size;
            p->store->arena.blob_used   = 0;
        }
    }
    return p->fail_code;
}

/**
 * 与 jfail 相同，但错误码用 JSDK_ERR_NO_MEMORY。
 *
 * 公共 API 明确承诺「arena 不足 → JSDK_ERR_NO_MEMORY」（见 joint_sdk.h 的
 * jsdk_desc_config_t.arena 与 jsdk_context_desc_import_raw 的说明）。
 * 这个区分很实在：调用方的补救动作完全不同——
 *   NO_MEMORY → 扩容 arena 或收紧 filter；PARSE → 固件发来的 JSON 有问题。
 */
static int jfail_mem(jsdk_jsondesc_t *p, const char *msg)
{
    (void)jfail(p, msg);
    p->fail_code = JSDK_ERR_NO_MEMORY;
    return JSDK_ERR_NO_MEMORY;
}

static int base_append(jsdk_jsondesc_t *p, char c)
{
    if ((size_t)p->cur_base_len + 2u > JSDK_EP_PATH_MAX_HARD)
        return jfail(p, "path exceeds JSDK_EP_PATH_MAX_HARD");
    p->cur_base[p->cur_base_len++] = c;
    p->cur_base[p->cur_base_len] = '\0';
    return JSDK_OK;
}

/* ==========================================================================
 * 类型映射
 * ======================================================================== */

uint8_t jsdk_ep_type_from_name(const char *name, size_t len)
{
    static const struct { const char *n; uint8_t v; } tbl[] = {
        {"uint8",        JSDK_EP_U8},
        {"int8",         JSDK_EP_I8},
        {"uint16",       JSDK_EP_U16},
        {"int16",        JSDK_EP_I16},
        {"uint32",       JSDK_EP_U32},
        {"int32",        JSDK_EP_I32},
        {"uint64",       JSDK_EP_U64},
        {"int64",        JSDK_EP_I64},
        {"float",        JSDK_EP_F32},
        {"double",       JSDK_EP_F64},
        {"bool",         JSDK_EP_BOOL},
        {"object",       JSDK_EP_OBJECT},
        {"endpoint_ref", JSDK_EP_ENDPOINT_REF},
        {"json",         JSDK_EP_JSON},
        {"function",     JSDK_EP_FUNCTION}
    };
    size_t i;
    for (i = 0; i < sizeof tbl / sizeof tbl[0]; ++i) {
        if (strlen(tbl[i].n) == len && memcmp(tbl[i].n, name, len) == 0)
            return tbl[i].v;
    }
    return 0;
}

const char *jsdk_ep_type_name(jsdk_ep_type_t t)
{
    switch (t) {
    case JSDK_EP_U8:  return "uint8";
    case JSDK_EP_I8:  return "int8";
    case JSDK_EP_U16: return "uint16";
    case JSDK_EP_I16: return "int16";
    case JSDK_EP_U32: return "uint32";
    case JSDK_EP_I32: return "int32";
    case JSDK_EP_U64: return "uint64";
    case JSDK_EP_I64: return "int64";
    case JSDK_EP_F32: return "float";
    case JSDK_EP_F64: return "double";
    case JSDK_EP_BOOL: return "bool";
    case JSDK_EP_OBJECT: return "object";
    case JSDK_EP_ENDPOINT_REF: return "endpoint_ref";
    case JSDK_EP_JSON: return "json";
    case JSDK_EP_FUNCTION: return "function";
    default: return "?";
    }
}

int jsdk_ep_type_is_scalar(jsdk_ep_type_t t)
{
    return (t >= JSDK_EP_U8 && t <= JSDK_EP_BOOL);
}

unsigned jsdk_ep_type_size(jsdk_ep_type_t t)
{
    switch (t) {
    case JSDK_EP_U8:
    case JSDK_EP_I8:
    case JSDK_EP_BOOL:  return 1u;
    case JSDK_EP_U16:
    case JSDK_EP_I16:   return 2u;
    case JSDK_EP_U32:
    case JSDK_EP_I32:
    case JSDK_EP_F32:   return 4u;
    case JSDK_EP_U64:
    case JSDK_EP_I64:
    case JSDK_EP_F64:   return 8u;
    default:            return 0u;
    }
}

/* ==========================================================================
 * arena / 端点存储
 * ======================================================================== */

void jsdk_ep_store_reset(jsdk_ep_store_t *s)
{
    if (!s) return;
    s->arena.entry_count = 0;
    s->arena.blob_top    = s->arena.size;
    s->arena.blob_used   = 0;
    s->parsed_total      = 0;
}

int jsdk_ep_arena_put(jsdk_arena_t *a, const char *path, size_t len,
                      uint16_t ep_id, uint8_t type, uint8_t access)
{
    jsdk_ep_entry_t e;
    size_t gap;

    if (!a || !a->base || !path || len == 0) return JSDK_ERR_INVALID_ARG;

    /* 条目区（前→后）与路径池（后→前）之间的空隙 */
    gap = a->blob_top - (size_t)a->entry_count * sizeof(jsdk_ep_entry_t);
    if (gap < len + sizeof(jsdk_ep_entry_t)) return JSDK_ERR_NO_MEMORY;

    a->blob_top -= len;
    memcpy(a->base + a->blob_top, path, len);

    e.path_off = (uint32_t)a->blob_top;
    e.ep_id    = ep_id;
    e.type     = type;
    e.access   = access;
    /* 用 memcpy 写条目：arena 由用户提供，不保证 4 字节对齐 */
    memcpy(a->base + (size_t)a->entry_count * sizeof e, &e, sizeof e);

    a->entry_count++;
    a->blob_used += len;
    return JSDK_OK;
}

static int entry_get(const jsdk_ep_store_t *s, unsigned index, jsdk_ep_entry_t *out)
{
    if (!s || index >= s->arena.entry_count) return JSDK_ERR_NOT_FOUND;
    memcpy(out, s->arena.base + (size_t)index * sizeof *out, sizeof *out);
    return JSDK_OK;
}

unsigned jsdk_ep_store_count(const jsdk_ep_store_t *s)
{
    return s ? s->arena.entry_count : 0u;
}

const char *jsdk_ep_store_path(const jsdk_ep_store_t *s, unsigned index)
{
    jsdk_ep_entry_t e;
    if (entry_get(s, index, &e) != JSDK_OK) return NULL;
    return (const char *)(s->arena.base + e.path_off);
}

int jsdk_ep_store_at(const jsdk_ep_store_t *s, unsigned index, const char **path,
                     uint16_t *ep_id, jsdk_ep_type_t *type, uint8_t *access)
{
    jsdk_ep_entry_t e;
    int rc = entry_get(s, index, &e);
    if (rc != JSDK_OK) return rc;
    if (path)   *path   = (const char *)(s->arena.base + e.path_off);
    if (ep_id)  *ep_id  = e.ep_id;
    if (type)   *type   = (jsdk_ep_type_t)e.type;
    if (access) *access = e.access;
    return JSDK_OK;
}

int jsdk_ep_store_lookup(const jsdk_ep_store_t *s, const char *path,
                         uint16_t *ep_id, jsdk_ep_type_t *type, uint8_t *access)
{
    unsigned i, n;
    if (!s || !path) return JSDK_ERR_INVALID_ARG;

    /* 线性查找：RETAIN_ALL 下最多 594 条，配置期使用足够。
       若将来成为热点，可换成按 path 哈希的小索引（不需要额外内存分配）。 */
    n = s->arena.entry_count;
    for (i = 0; i < n; ++i) {
        jsdk_ep_entry_t e;
        const char *cand;
        memcpy(&e, s->arena.base + (size_t)i * sizeof e, sizeof e);
        cand = (const char *)(s->arena.base + e.path_off);
        if (strcmp(cand, path) == 0) {
            if (ep_id)  *ep_id  = e.ep_id;
            if (type)   *type   = (jsdk_ep_type_t)e.type;
            if (access) *access = e.access;
            return JSDK_OK;
        }
    }
    return JSDK_ERR_NOT_FOUND;
}

/* ==========================================================================
 * 过滤器
 * ======================================================================== */

static unsigned popcount64(uint64_t v)
{
    unsigned n = 0;
    while (v) { v &= (v - 1u); n++; }
    return n;
}

unsigned jsdk_jsondesc_filter_hits(const jsdk_jsondesc_t *p)
{
    return p ? popcount64(p->filter.hit) : 0u;
}

int jsdk_jsondesc_satisfied(const jsdk_jsondesc_t *p)
{
    if (!p || p->filter.count == 0) return 0;
    return jsdk_jsondesc_filter_hits(p) >= p->filter.count;
}

/** 命中则返回 1，并记录到 hit 位图（供 stop_when_satisfied 使用）。 */
static int filter_match(jsdk_jsondesc_t *p, const char *path)
{
    unsigned i;
    size_t plen = strlen(path);
    int keep = 0;

    for (i = 0; i < p->filter.count; ++i) {
        const char *f = p->filter.paths[i];
        size_t flen;
        int hit = 0;

        if (!f) continue;
        flen = strlen(f);
        if (flen == 1u && f[0] == '*') {
            hit = 1;                                    /* 全保留 */
        } else if (flen > 0u && f[flen - 1u] == '*') {
            /* 前缀匹配："axis0.controller.config.mit_max_*" */
            size_t pl = flen - 1u;
            hit = (pl == 0u) ? 1 : ((plen > pl) && (memcmp(path, f, pl) == 0));
        } else if (flen > 0u && f[flen - 1u] == '.') {
            /* 段前缀：路径必须更长且以该段前缀开头 */
            hit = (plen > flen) && (memcmp(path, f, flen) == 0);
        } else if (flen == 0u) {
            hit = (plen == 0u);
        } else {
            hit = (plen == flen) && (memcmp(path, f, flen) == 0);
        }

        if (hit) {
            keep = 1;
            p->filter.hit |= (uint64_t)1u << i;
        }
    }
    return keep;
}

/* ==========================================================================
 * 解析器
 * ======================================================================== */

static jsdk_json_frame_t *top(jsdk_jsondesc_t *p) { return &p->f[p->depth]; }

static int push_frame(jsdk_jsondesc_t *p, int is_array)
{
    jsdk_json_frame_t *f;
    if (p->depth + 1u > JSDK_JSON_MAX_DEPTH)
        return jfail(p, "nesting deeper than JSDK_JSON_MAX_DEPTH");
    f = &p->f[++p->depth];
    memset(f, 0, sizeof *f);
    f->is_array    = (uint8_t)is_array;
    f->restore_len = p->cur_base_len;
    /* 新帧内的第一个字符串：对象内是键，数组内是值 */
    p->expect_key  = (uint8_t)(is_array ? 0u : 1u);
    return JSDK_OK;
}

/** 发出一个叶子端点。path 即 cur_base。 */
static int emit_leaf(jsdk_jsondesc_t *p, uint8_t type, uint16_t id, uint8_t access)
{
    size_t len = (size_t)p->cur_base_len;
    int keep;

    p->store->parsed_total++;
    if (p->store->parsed_total > p->store->max_endpoints)
        return jfail(p, "endpoint count exceeds max_endpoints");

    keep = p->retain_all ? 1 : filter_match(p, p->cur_base);
    if (!keep) return JSDK_OK;

    if (len + 1u > (size_t)p->max_path_len)
        return jfail(p, "path longer than max_path_len");

    /* 长度不足 → NO_MEMORY（由调用者扩容 arena 后重试，会整体重解析） */
    if (jsdk_ep_arena_put(&p->store->arena, p->cur_base, len + 1u, id, type, access)
            != JSDK_OK)
        return jfail_mem(p, "arena exhausted");

    return JSDK_OK;
}

static int pop_object(jsdk_jsondesc_t *p)
{
    jsdk_json_frame_t *f = top(p);

    if (f->have_id && f->have_type) {
        int rc;
        if (!f->have_name)
            return jfail(p, "object has id but no name");
        rc = emit_leaf(p, f->type, (uint16_t)f->id, f->access);
        if (rc != JSDK_OK) return rc;      /* ⚠ 不能一律转成 PARSE，会吞掉 NO_MEMORY */
    }

    p->cur_base_len = f->restore_len;
    p->cur_base[p->cur_base_len] = '\0';
    p->depth--;
    return JSDK_OK;
}

static int pop_array(jsdk_jsondesc_t *p)
{
    jsdk_json_frame_t *f = top(p);
    if (!f->is_array) return jfail(p, "closing ']' without matching '['");

    p->cur_base_len = f->restore_len;
    p->cur_base[p->cur_base_len] = '\0';
    p->depth--;

    if (p->depth == 0) p->state = JSDK_JS_DONE;
    return JSDK_OK;
}

/* ---------- 字段终结 ---------- */

static int finish_key(jsdk_jsondesc_t *p)
{
    jsdk_json_frame_t *f = top(p);

    p->key[p->key_len] = '\0';
    if (!p->key_overflow) {
        if      (p->key_len == 4u && memcmp(p->key, "name",   4) == 0) f->cur_key = JSDK_KEY_NAME;
        else if (p->key_len == 2u && memcmp(p->key, "id",     2) == 0) f->cur_key = JSDK_KEY_ID;
        else if (p->key_len == 4u && memcmp(p->key, "type",   4) == 0) f->cur_key = JSDK_KEY_TYPE;
        else if (p->key_len == 6u && memcmp(p->key, "access", 6) == 0) f->cur_key = JSDK_KEY_ACCESS;
        else                                                          f->cur_key = JSDK_KEY_OTHER;
    } else {
        f->cur_key = JSDK_KEY_OTHER;
    }
    p->state = JSDK_JS_AFTER_KEY;
    return JSDK_OK;
}

static int finish_number(jsdk_jsondesc_t *p)
{
    jsdk_json_frame_t *f = top(p);

    if (f->cur_key == JSDK_KEY_ID) {
        if (p->neg || p->num_digits == 0u) return jfail(p, "invalid id");
        if (p->num > 0xFFFFu)              return jfail(p, "id exceeds 65535");
        f->id      = p->num;
        f->have_id = 1u;
    }
    /* 其它键的数值一律忽略 */
    p->state = JSDK_JS_AFTER_VAL;
    return JSDK_OK;
}

static int finish_literal(jsdk_jsondesc_t *p)
{
    p->val[p->val_len] = '\0';
    if (!(strcmp(p->val, "true") == 0 || strcmp(p->val, "false") == 0 ||
          strcmp(p->val, "null") == 0))
        return jfail(p, "invalid literal");
    p->state = JSDK_JS_AFTER_VAL;
    return JSDK_OK;
}

static int finish_string_value(jsdk_jsondesc_t *p)
{
    jsdk_json_frame_t *f = top(p);

    p->val[p->val_len] = '\0';

    switch (f->cur_key) {
    case JSDK_KEY_TYPE: {
        uint8_t t = jsdk_ep_type_from_name(p->val, p->val_len);
        if (t == 0) return jfail(p, "unknown endpoint type in descriptor");
        f->type      = t;
        f->have_type = 1u;
        break;
    }
    case JSDK_KEY_ACCESS: {
        uint8_t acc = 0, i;
        for (i = 0; i < p->val_len; ++i) {
            if      (p->val[i] == 'r') acc |= JSDK_EP_ACCESS_R;
            else if (p->val[i] == 'w') acc |= JSDK_EP_ACCESS_W;
            else return jfail(p, "invalid access value");
        }
        f->access      = acc;
        f->have_access = 1u;
        break;
    }
    default:
        /* name 已增量写入 cur_base；其余键忽略 */
        break;
    }

    p->state = JSDK_JS_AFTER_VAL;
    return JSDK_OK;
}

static int finish_string_key(jsdk_jsondesc_t *p)
{
    (void)p;
    return finish_key(p);
}

/* ---------- 字符串字符处理 ---------- */

static int string_char(jsdk_jsondesc_t *p, uint8_t c)
{
    if (p->in_key) {
        if (p->key_len + 1u < JSDK_EP_KEY_MAX) {
            p->key[p->key_len++] = (char)c;
        } else {
            p->key_overflow = 1u;   /* 键名过长：按“其它键”处理，不失败 */
        }
        return JSDK_OK;
    }

    {
        jsdk_json_frame_t *f = top(p);

        if (f->cur_key == JSDK_KEY_NAME) {
            if (!f->name_appended) {
                f->have_name = 1u;
                if (p->cur_base_len > 0u) {
                    if (base_append(p, '.') != JSDK_OK) return JSDK_ERR_PARSE;
                }
                f->name_appended = 1u;
            }
            return base_append(p, (char)c);
        }

        if (f->cur_key == JSDK_KEY_TYPE || f->cur_key == JSDK_KEY_ACCESS) {
            if (p->val_len + 1u >= JSDK_EP_VAL_MAX)
                return jfail(p, "type/access value too long");
            p->val[p->val_len++] = (char)c;
        }
        /* 其余键的字符串值：跳过 */
        return JSDK_OK;
    }
}

/** 处理字符串内的转义；返回 1 表示该字符已被消费，0 表示需按普通字符处理。 */
static int escape_char(jsdk_jsondesc_t *p, uint8_t c, uint8_t *out)
{
    if (!p->esc) {
        if (c != '\\') return 0;
        p->esc = 1u;
        return 1;
    }
    p->esc = 0u;
    switch (c) {
    case '"':  *out = '"';  break;
    case '\\': *out = '\\'; break;
    case '/':  *out = '/';  break;
    case 'b':  *out = '\b'; break;
    case 'f':  *out = '\f'; break;
    case 'n':  *out = '\n'; break;
    case 'r':  *out = '\r'; break;
    case 't':  *out = '\t'; break;
    default:
        /* \uXXXX 等：本描述符不含，直接失败以免静默产生错误路径 */
        jfail(p, "unsupported escape sequence in descriptor");
        return 1;
    }
    return 1;
}

/* ---------- 主循环 ---------- */

int jsdk_jsondesc_feed(jsdk_jsondesc_t *p, const void *data, size_t len)
{
    const uint8_t *d = (const uint8_t *)data;
    size_t i = 0;

    if (!p || (!d && len)) return JSDK_ERR_INVALID_ARG;
    if (p->state == JSDK_JS_FAIL) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */

    while (i < len) {
        uint8_t c = d[i];

        p->bytes_fed++;

        if (p->state == JSDK_JS_FAIL) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */

        switch (p->state) {

        case JSDK_JS_BEGIN:
            if (is_ws(c)) { i++; break; }
            if (c != '[') return jfail(p, "root must be an array");
            if (push_frame(p, 1) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
            p->state = JSDK_JS_EXPECT;
            i++;
            break;

        case JSDK_JS_EXPECT: {
            jsdk_json_frame_t *f = top(p);
            if (is_ws(c)) { i++; break; }

            if (c == '}') {
                if (f->is_array) return jfail(p, "'}' inside array");
                if (pop_object(p) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
                p->state = JSDK_JS_AFTER_VAL;
                i++;
                break;
            }
            if (c == ']') {
                if (!f->is_array) return jfail(p, "']' inside object");
                if (pop_array(p) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
                if (p->state != JSDK_JS_DONE) p->state = JSDK_JS_AFTER_VAL;
                i++;
                break;
            }
            if (c == '{') {
                if (push_frame(p, 0) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
                i++;
                break;
            }
            if (c == '[') {
                if (push_frame(p, 1) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
                i++;
                break;
            }
            if (c == '"') {
                p->in_key        = p->expect_key;
                p->key_len       = 0u;
                p->key_overflow  = 0u;
                p->val_len       = 0u;
                p->esc           = 0u;
                /* name 字段：在这里就标记“见过”，否则空名字的节点（如根节点
                   {"name":"","id":0,"type":"json"}）会被误判为缺 name */
                if (!p->in_key && f->cur_key == JSDK_KEY_NAME) {
                    f->have_name     = 1u;
                    f->name_appended = 0u;
                }
                p->state = JSDK_JS_IN_STR;
                i++;
                break;
            }
            if (c == '-' || is_digit(c)) {
                p->neg        = (uint8_t)(c == '-');
                p->num        = is_digit(c) ? (uint32_t)(c - '0') : 0u;
                p->num_digits = is_digit(c) ? 1u : 0u;
                p->state      = JSDK_JS_IN_NUM;
                i++;
                break;
            }
            if (is_alpha(c)) {
                p->val_len = 0u;
                p->state   = JSDK_JS_IN_LIT;
                break;              /* 不消费：交给 IN_LIT */
            }
            return jfail(p, "unexpected character where key/value expected");
        }

        case JSDK_JS_IN_KEY:
            /* 不会进入：键与字符串值共用 IN_STR */
            return jfail(p, "internal state error");

        case JSDK_JS_IN_STR: {
            uint8_t out = 0;
            if (c == '"' && !p->esc) {
                i++;
                if (p->in_key) {
                    p->in_key = 0u;
                    if (finish_string_key(p) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
                } else {
                    if (finish_string_value(p) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
                    p->state = JSDK_JS_AFTER_VAL;
                }
                break;
            }
            if (escape_char(p, c, &out)) {
                i++;
                if (p->state == JSDK_JS_FAIL) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
                if (p->esc) break;      /* 刚吃掉反斜杠 */
                if (string_char(p, out) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
                break;
            }
            if (string_char(p, c) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
            i++;
            break;
        }

        case JSDK_JS_AFTER_KEY:
            if (is_ws(c)) { i++; break; }
            if (c != ':') return jfail(p, "expected ':' after key");
            p->expect_key = 0u;          /* ':' 之后是值 */
            p->state = JSDK_JS_EXPECT;
            i++;
            break;

        case JSDK_JS_IN_NUM:
            if (is_digit(c)) {
                if (p->num_digits < 9u) {
                    p->num = p->num * 10u + (uint32_t)(c - '0');
                    p->num_digits++;
                } else {
                    p->num = 0xFFFFFFFFu;   /* 溢出：后续校验会拒绝 */
                }
                i++;
                break;
            }
            if (c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-') {
                /* 浮点/指数形式：本描述符的 id 不会是浮点 → 交给校验拒绝 */
                if (top(p)->cur_key == JSDK_KEY_ID) return jfail(p, "id must be an integer");
                p->num_digits = 0u;         /* 标记为非整数，忽略其值 */
                p->num = 0u;
                i++;
                break;
            }
            if (finish_number(p) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
            break;                          /* 不消费终止符 */

        case JSDK_JS_IN_LIT:
            if (is_alpha(c) && p->val_len + 1u < JSDK_EP_VAL_MAX) {
                p->val[p->val_len++] = (char)c;
                i++;
                if (p->val_len == 5u) {     /* "false" 已完整 */
                    if (finish_literal(p) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
                }
                break;
            }
            if (finish_literal(p) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
            break;                          /* 不消费终止符 */

        case JSDK_JS_AFTER_VAL:
            if (is_ws(c)) { i++; break; }
            if (c == ',') {
                p->expect_key = (uint8_t)(top(p)->is_array ? 0u : 1u);
                p->state = JSDK_JS_EXPECT;
                i++;
                break;
            }
            if (c == '}' || c == ']') { p->state = JSDK_JS_EXPECT; break; }  /* 不消费 */
            return jfail(p, "expected ',' or closing bracket");

        case JSDK_JS_DONE:
            if (is_ws(c)) { i++; break; }
            return jfail(p, "trailing data after root array");

        default:
            return jfail(p, "internal state error");
        }
    }

    return (p->state == JSDK_JS_FAIL) ? p->fail_code : JSDK_OK;
}

int jsdk_jsondesc_init(jsdk_jsondesc_t *p, const jsdk_desc_config_t *cfg,
                       jsdk_ep_store_t *store)
{
    if (!p || !cfg || !store)                        return JSDK_ERR_INVALID_ARG;
    if (!store->arena.base || store->arena.size == 0) return JSDK_ERR_INVALID_ARG;
    if (cfg->filter_count > JSDK_DESC_MAX_FILTERS)    return JSDK_ERR_INVALID_ARG;
    if (!cfg->filter_paths && cfg->filter_count)      return JSDK_ERR_INVALID_ARG;

    memset(p, 0, sizeof *p);
    p->cfg        = cfg;
    p->store      = store;
    p->retain_all = (uint8_t)(cfg->retain == JSDK_DESC_RETAIN_ALL);
    p->max_path_len = (uint16_t)(cfg->max_path_len ? cfg->max_path_len : 128u);
    if (p->max_path_len > JSDK_EP_PATH_MAX_HARD)      return JSDK_ERR_INVALID_ARG;

    if (!p->retain_all) {
        if (cfg->filter_count == 0u) return JSDK_ERR_INVALID_ARG;
        p->filter.paths = cfg->filter_paths;
        p->filter.count = cfg->filter_count;
    } else if (cfg->filter_paths && cfg->filter_count) {
        /* RETAIN_ALL 下仍记录 filter，用于 satisfied 统计；不影响保留策略 */
        p->filter.paths = cfg->filter_paths;
        p->filter.count = cfg->filter_count;
    }

    jsdk_ep_store_reset(store);
    store->max_endpoints = cfg->max_endpoints ? cfg->max_endpoints : 2048u;

    p->fail_code = JSDK_ERR_PARSE;   /* 默认码；jfail_mem 会覆盖为 NO_MEMORY */
    p->state = JSDK_JS_BEGIN;
    p->depth = 0;
    p->cur_base[0] = '\0';
    return JSDK_OK;
}

int jsdk_jsondesc_finish(jsdk_jsondesc_t *p)
{
    if (!p) return JSDK_ERR_INVALID_ARG;
    /* 粘性失败码：若前面已因 arena 不足失败，这里必须原样返回 NO_MEMORY，
       否则 run()（init+feed+finish）会把真实原因掩盖成 PARSE。 */
    if (p->state == JSDK_JS_FAIL) return p->fail_code;
    if (p->state != JSDK_JS_DONE) return jfail(p, "truncated descriptor (no closing ']')");
    if (p->depth != 0u)           return jfail(p, "unbalanced brackets");
    return JSDK_OK;
}

int jsdk_jsondesc_run(const jsdk_desc_config_t *cfg, jsdk_ep_store_t *store,
                      const void *json, size_t len)
{
    jsdk_jsondesc_t p;
    int rc = jsdk_jsondesc_init(&p, cfg, store);
    if (rc != JSDK_OK) return rc;
    rc = jsdk_jsondesc_feed(&p, json, len);
    if (rc != JSDK_OK) return rc;
    return jsdk_jsondesc_finish(&p);
}

const char *jsdk_jsondesc_error(const jsdk_jsondesc_t *p)
{
    return p ? p->err : NULL;
}

/* ==========================================================================
 * 公开 API（§18）：arena 尺寸估算
 * ======================================================================== */

size_t jsdk_desc_arena_size(const jsdk_desc_config_t *cfg)
{
    size_t need = 0;
    unsigned i;

    if (!cfg) return 0;

    if (cfg->retain == JSDK_DESC_RETAIN_FILTERED && cfg->filter_paths) {
        for (i = 0; i < cfg->filter_count; ++i) {
            const char *f = cfg->filter_paths[i];
            if (!f) continue;
            need += sizeof(jsdk_ep_entry_t) + strlen(f) + 1u;
        }
        if (need == 0) return 0;
        /* 估算：精确匹配时即为准确值；前缀匹配的实际路径更长，故加 25% + 128 余量 */
        return need + need / 4u + 128u;
    }

    /* RETAIN_ALL：端点数未知，返回保守推荐值 */
    return JSDK_DESC_ARENA_RECOMMEND_ALL;
}

/* ======================== src/proto_cyberbeast/cb_mit.c ======================== */
/**
 * @file    cb_mit.c
 * @brief   CYBERBEAST MIT 紧凑编解码实现（见 cb_mit.h）
 */


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
    case CB_ERR_UNDER_VOLTAGE: return "UNDER_VOLTAGE";
    case CB_ERR_OVER_TEMP:     return "OVER_TEMP";
    case CB_ERR_OVER_CURRENT:  return "OVER_CURRENT";
    case CB_ERR_STALL:         return "STALL";
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

/* ======================== src/proto_cyberbeast/cb_param.c ======================== */
/**
 * @file    cb_param.c
 * @brief   CYBERBEAST 参数访问编解码实现（见 cb_param.h）
 *
 * 与固件 can_cyberbeast.cpp 的 cmd_param_read / cmd_param_read_batch /
 * cmd_param_write / cmd_param_write_segmented / send_param_write_ack
 * 逐字节对齐。
 */


/* ==========================================================================
 * 一、单参数读
 * ======================================================================== */

/**
 * 固件在**设备侧**对 ReqLen 做的归一化（cmd_param_read 里逐条对应）。
 * 导出供主站预估实际会拿到多少字节。
 */
uint8_t cb_param_normalize_req_len(uint8_t req_len, int classic)
{
    if (req_len == 0u)                     req_len = 4u;   /* 旧客户端 */
    if (req_len > CB_PARAM_MAX_VALUE)      req_len = CB_PARAM_MAX_VALUE;
    if (classic && req_len > 4u)           req_len = 4u;   /* 8-4 = 4 */
    return req_len;
}

uint32_t cb_param_read_chunks(uint8_t value_len, uint8_t req_len, int classic)
{
    uint32_t eff = (uint32_t)cb_param_normalize_req_len(req_len, classic);

    if (value_len == 0u) return 0u;
    if (eff == 0u)       return 0u;
    return ((uint32_t)value_len + eff - 1u) / eff;
}

size_t cb_param_pack_read_req(uint8_t *dst, size_t cap, uint16_t ep_id,
                              uint8_t req_len, uint32_t offset,
                              int with_offset)
{
    size_t need = with_offset ? CB_PARAM_READ_REQ_FULL : CB_PARAM_READ_REQ_MIN;

    if (!dst || cap < need) return 0u;
    if (req_len == 0u)     return 0u;      /* 0 是“未指定”，发送侧不猜 */
    if (req_len > CB_PARAM_MAX_VALUE) return 0u;

    dst[0] = 0u;                            /* 单读：Flags 全 0 */
    cb_be_put_u16(dst + 1, ep_id);
    dst[3] = req_len;

    if (with_offset) {
        cb_be_put_u32(dst + 4, offset);
    }
    return need;
}

int cb_param_unpack_read_req(const uint8_t *src, size_t len, int classic,
                             cb_param_read_req_t *out)
{
    if (!src || len < CB_PARAM_READ_REQ_MIN) return -1;
    if (!out) return 0;

    out->flags      = src[0];
    out->ep_id      = cb_be_get_u16(src + 1);
    out->has_offset = (len >= CB_PARAM_READ_REQ_FULL) ? 1 : 0;
    out->offset     = out->has_offset ? cb_be_get_u32(src + 4) : 0u;
    out->req_len    = cb_param_normalize_req_len(src[3], classic);
    return 0;
}

size_t cb_param_pack_read_rsp(uint8_t *dst, size_t cap,
                              const cb_param_read_rsp_t *v)
{
    size_t need;

    if (!dst || !v) return 0u;
    if (v->data_len > CB_PARAM_MAX_VALUE) return 0u;

    need = 4u + (size_t)v->data_len;
    if (cap < need) return 0u;

    dst[0] = v->flags;
    cb_be_put_u16(dst + 1, v->ep_id);
    dst[3] = v->data_len;
    if (v->data_len > 0u) {
        uint8_t i;
        for (i = 0u; i < v->data_len; ++i) dst[4u + i] = v->value[i];
    }
    return need;
}

int cb_param_unpack_read_rsp(const uint8_t *src, size_t len,
                             cb_param_read_rsp_t *out)
{
    uint8_t i;

    if (!src || len < 4u) return -1;
    if (src[3] > CB_PARAM_MAX_VALUE) return -1;
    if (len < 4u + (size_t)src[3]) return -1;

    if (out) {
        out->flags    = src[0];
        out->ep_id    = cb_be_get_u16(src + 1);
        out->data_len = src[3];
        /* 未使用的尾部清零，避免调用方读到上一次的残留 */
        for (i = 0u; i < CB_PARAM_MAX_VALUE; ++i) out->value[i] = 0u;
        for (i = 0u; i < out->data_len; ++i) out->value[i] = src[4u + i];
    }
    return 0;
}

size_t cb_param_build_read_rsp(uint8_t *dst, size_t cap,
                               uint8_t req_flags, uint16_t ep_id,
                               const uint8_t *full_value, uint8_t full_len,
                               uint8_t req_len, uint32_t offset)
{
    uint8_t actual_len = 0u;
    uint8_t resp_flags;
    size_t  need;

    if (!dst) return 0u;
    if (full_len > CB_PARAM_MAX_VALUE) return 0u;

    resp_flags = (uint8_t)(req_flags & (uint8_t)~CB_PARAM_FLAG_MORE);

    if (offset < (uint32_t)full_len) {
        uint32_t available = (uint32_t)full_len - offset;
        actual_len = (req_len < available) ? req_len : (uint8_t)available;

        /* 还有剩余 → 置 More */
        if ((uint32_t)offset + (uint32_t)actual_len < (uint32_t)full_len) {
            resp_flags |= CB_PARAM_FLAG_MORE;
        }
    }
    /* offset ≥ full_len → actual_len 保持 0（合法：越界读完） */

    need = 4u + (size_t)actual_len;
    if (cap < need) return 0u;

    dst[0] = resp_flags;
    cb_be_put_u16(dst + 1, ep_id);
    dst[3] = actual_len;
    if (actual_len > 0u && full_value) {
        uint8_t i;
        for (i = 0u; i < actual_len; ++i) {
            dst[4u + i] = full_value[(size_t)offset + i];
        }
    }
    return need;
}

int cb_param_rsp_has_more(const cb_param_read_rsp_t *rsp)
{
    if (!rsp) return 0;
    return (rsp->flags & CB_PARAM_FLAG_MORE) ? 1 : 0;
}

/* ==========================================================================
 * 二、批量读
 * ======================================================================== */

uint8_t cb_param_bitmap_bytes(uint8_t n)
{
    return (uint8_t)(((unsigned)n + 7u) / 8u);
}

int cb_param_bitmap_test(const uint8_t *bitmap, uint8_t n_bytes, uint8_t index)
{
    uint8_t byte_idx = (uint8_t)(index / 8u);
    uint8_t bit_idx  = (uint8_t)(index % 8u);

    if (!bitmap || byte_idx >= n_bytes) return 0;
    return (bitmap[byte_idx] & (uint8_t)(1u << bit_idx)) ? 1 : 0;
}

size_t cb_param_pack_batch_req(uint8_t *dst, size_t cap,
                               const uint16_t *eps, uint8_t n)
{
    uint8_t i;
    size_t  need;

    if (!dst || !eps) return 0u;
    if (n == 0u || n > CB_PARAM_MAX_BATCH) return 0u;

    need = 2u + 2u * (size_t)n;
    if (cap < need) return 0u;

    dst[0] = CB_PARAM_FLAG_BATCH;
    dst[1] = n;
    for (i = 0u; i < n; ++i) {
        cb_be_put_u16(dst + 2u + 2u * i, eps[i]);
    }
    return need;
}

int cb_param_unpack_batch_req(const uint8_t *src, size_t len,
                              uint16_t *out_eps, uint8_t ep_cap,
                              cb_param_batch_req_t *out)
{
    size_t  avail;
    uint8_t n;

    if (!src || len < 2u) return -1;
    if (!(src[0] & CB_PARAM_FLAG_BATCH)) return -1;   /* 不是批量请求 */

    n = src[1];
    avail = (len - 2u) / 2u;                          /* 帧内完整条目数 */

    /* 固件行为：帧内条目不足时**按实际可用数**处理，而不是报错 */
    if ((size_t)n > avail) n = (uint8_t)avail;
    if (n == 0u || n > CB_PARAM_MAX_BATCH) return -1;

    if (out_eps) {
        uint8_t i;
        uint8_t lim = (n < ep_cap) ? n : ep_cap;
        for (i = 0u; i < lim; ++i) {
            out_eps[i] = cb_be_get_u16(src + 2u + 2u * i);
        }
    }

    if (out) {
        out->count    = n;
        out->ep_bytes = src + 2;
    }
    return 0;
}

int cb_param_unpack_batch_rsp(const uint8_t *src, size_t len, uint8_t n_req,
                              cb_param_batch_rsp_t *out)
{
    uint8_t expected_bm;

    if (!src || len < 2u) return -1;
    if (!(src[0] & CB_PARAM_FLAG_BATCH)) return -1;   /* 不是批量响应 */

    if (!out) return 0;

    out->flags        = src[0];
    out->bitmap       = NULL;
    out->values       = NULL;
    out->values_len   = 0u;
    out->bitmap_bytes = 0u;

    if (src[0] & CB_PARAM_FLAG_ERR) {
        /* ERR：长度必须恰为 2，Count = 0 */
        out->is_err = 1;
        out->count  = src[1];
        return 0;
    }

    out->is_err = 0;
    out->count  = src[1];

    expected_bm = cb_param_bitmap_bytes(n_req);
    if (out->count != n_req) return -1;               /* 设备应全量返回 */
    if (len < 2u + (size_t)expected_bm) return -1;

    out->bitmap_bytes = expected_bm;
    out->bitmap       = src + 2;
    out->values       = src + 2 + expected_bm;
    out->values_len   = len - 2u - (size_t)expected_bm;
    return 0;
}

size_t cb_param_pack_batch_err(uint8_t *dst, size_t cap)
{
    if (!dst || cap < 2u) return 0u;
    dst[0] = (uint8_t)(CB_PARAM_FLAG_BATCH | CB_PARAM_FLAG_ERR);
    dst[1] = 0u;
    return 2u;
}

size_t cb_param_plan_batches(const cb_param_batch_item_t *items, size_t n_items,
                             size_t budget,
                             size_t *out_counts, size_t *out_offsets,
                             size_t max_batches)
{
    size_t n_batches = 0u;
    size_t i = 0u;

    if (!items || !out_counts || n_items == 0u || max_batches == 0u) return 0u;
    if (budget > CB_PARAM_FD_FRAME_MAX) budget = CB_PARAM_FD_FRAME_MAX;

    while (i < n_items) {
        size_t batch_start = i;
        size_t sum = 0u;
        size_t n   = 0u;

        /* 贪心：尽量多装，但不超过 N 上限与字节预算。
           请求帧约束 `2 + 2n ≤ 64` 即 n ≤ 31，由 CB_PARAM_MAX_BATCH 保证。 */
        while (i < n_items && n < CB_PARAM_MAX_BATCH) {
            size_t len = (items[i].value_len > CB_PARAM_MAX_VALUE)
                       ? CB_PARAM_MAX_VALUE : (size_t)items[i].value_len;
            size_t try_n  = n + 1u;
            size_t try_bm = ((try_n + 7u) / 8u);

            if (2u + try_bm + sum + len > budget) break;   /* 装不下 */
            sum += len;
            n   = try_n;
            ++i;
        }

        if (n == 0u) return 0u;      /* 单条就装不下 → 整体失败 */

        if (n_batches >= max_batches) return 0u;
        out_counts[n_batches] = n;
        if (out_offsets) out_offsets[n_batches] = batch_start;
        ++n_batches;
    }

    return n_batches;
}

/* ==========================================================================
 * 三、参数写
 * ======================================================================== */

size_t cb_param_pack_write_req(uint8_t *dst, size_t cap, uint16_t ep_id,
                               const uint8_t *value, uint8_t value_len)
{
    size_t need;
    uint8_t i;

    if (!dst) return 0u;
    if (value_len == 0u || value_len > CB_PARAM_MAX_VALUE) return 0u;
    if (!value) return 0u;

    need = 4u + (size_t)value_len;
    if (cap < need) return 0u;

    dst[0] = 0u;
    cb_be_put_u16(dst + 1, ep_id);
    dst[3] = value_len;
    for (i = 0u; i < value_len; ++i) dst[4u + i] = value[i];
    return need;
}

int cb_param_unpack_write_req(const uint8_t *src, size_t len,
                              uint8_t *out_flags, uint16_t *out_ep_id,
                              uint8_t *out_value, uint8_t *out_value_len)
{
    uint8_t data_len;
    uint8_t i;

    if (!src || len < 4u) return -1;

    data_len = src[3];
    if (data_len > CB_PARAM_MAX_VALUE) return -1;
    if (len < 4u + (size_t)data_len) return -1;

    if (out_flags)     *out_flags     = src[0];
    if (out_ep_id)     *out_ep_id     = cb_be_get_u16(src + 1);
    if (out_value_len) *out_value_len = data_len;
    if (out_value) {
        for (i = 0u; i < data_len; ++i) out_value[i] = src[4u + i];
    }
    return 0;
}

size_t cb_param_pack_write_ack(uint8_t *dst, size_t cap,
                               uint8_t flags, uint16_t ep_id)
{
    if (!dst || cap < CB_PARAM_ACK_LEN) return 0u;

    dst[0] = flags;
    cb_be_put_u16(dst + 1, ep_id);
    dst[3] = 0u;
    dst[4] = 0u;
    dst[5] = 0u;
    dst[6] = 0u;
    dst[7] = 0u;
    return CB_PARAM_ACK_LEN;
}

int cb_param_unpack_write_ack(const uint8_t *src, size_t len,
                              uint8_t *out_flags, uint16_t *out_ep_id)
{
    if (!src || len < CB_PARAM_ACK_LEN) return -1;
    /* DataLen 必须为 0：确认帧不携带数据 */
    if (src[3] != 0u) return -1;

    if (out_flags) *out_flags = src[0];
    if (out_ep_id) *out_ep_id = cb_be_get_u16(src + 1);
    return 0;
}

/* ==========================================================================
 * 四、分段写
 * ======================================================================== */

size_t cb_param_pack_write_chunk(uint8_t *dst, size_t cap, uint16_t ep_id,
                                 uint8_t total_len, uint32_t offset,
                                 const uint8_t *value, uint8_t value_len,
                                 int more)
{
    uint8_t i;

    if (!dst || cap < CB_PARAM_CLASSIC_FRAME) return 0u;
    if (total_len < CB_PARAM_SEG_MIN_LEN) return 0u;
    if (total_len > CB_PARAM_MAX_VALUE)   return 0u;
    if (value_len > CB_PARAM_CHUNK_BYTES) return 0u;
    if (offset % CB_PARAM_CHUNK_BYTES != 0u) return 0u;
    if (offset + (uint32_t)value_len > (uint32_t)total_len) return 0u;
    if (value_len > 0u && !value) return 0u;

    /* 加固：**绝不生成固件无法正确装配的块**。
       固件 `cmd_param_write_segmented()` 对末块不做完整性校验：
       若末块（More=0）携带的字节数不足，它会直接写入一个尾部被 0 填充的
       “完整值”（见 `docs/FIRMWARE_ISSUES.zh-CN.md` 的 F13）。因此发送侧必须保证：
         ① 非末块必须满 4 字节（否则中间的洞会被静默补 0）；
         ② 末块必须刚好把值补齐（offset + value_len == total_len）。 */
    if (more) {
        if (value_len != CB_PARAM_CHUNK_BYTES) return 0u;
        if (offset + (uint32_t)value_len >= (uint32_t)total_len) return 0u;
    } else {
        if (offset + (uint32_t)value_len != (uint32_t)total_len) return 0u;
    }

    dst[0] = more ? CB_PARAM_FLAG_MORE : 0u;
    cb_be_put_u16(dst + 1, ep_id);
    dst[3] = total_len;

    /* 末块不足 4 B：用 0 填充（固件按 TotalLen 截断，填充位被忽略） */
    for (i = 0u; i < CB_PARAM_CHUNK_BYTES; ++i) {
        dst[4u + i] = (i < value_len) ? value[i] : 0u;
    }
    return CB_PARAM_CLASSIC_FRAME;
}

void cb_param_write_asm_init(cb_param_write_asm_t *a)
{
    uint8_t i;

    if (!a) return;
    a->active    = 0;
    a->ep_id     = 0u;
    a->master_id = 0u;
    a->total_len = 0u;
    a->offset    = 0u;
    for (i = 0u; i < CB_PARAM_MAX_VALUE; ++i) a->buf[i] = 0u;
}

int cb_param_write_asm_feed(cb_param_write_asm_t *a, uint8_t master_id,
                            uint16_t ep_id, uint8_t total_len, uint8_t flags,
                            const uint8_t *chunk,
                            uint8_t *value_out, uint8_t *value_len_out)
{
    uint8_t remaining;
    uint8_t i;

    if (!a || !chunk) return -1;

    /* ① TotalLen 必须在 5..8（固件：否则中止装配） */
    if (total_len < CB_PARAM_SEG_MIN_LEN || total_len > CB_PARAM_MAX_VALUE) {
        cb_param_write_asm_init(a);
        return -1;
    }

    /* ④ 首块 / 换端点 / 换主站 → 重置装配（固件行为，非错误） */
    if (!a->active || a->ep_id != ep_id || a->master_id != master_id) {
        cb_param_write_asm_init(a);
        a->active    = 1;
        a->master_id = master_id;
        a->ep_id     = ep_id;
        a->total_len = total_len;
    }

    /* ② offset 已越界（块序错乱 / 重复末块）→ 中止 */
    if (a->offset >= a->total_len) {
        cb_param_write_asm_init(a);
        return -1;
    }

    remaining = (uint8_t)(a->total_len - a->offset);
    if (remaining > CB_PARAM_CHUNK_BYTES) remaining = CB_PARAM_CHUNK_BYTES;

    for (i = 0u; i < remaining; ++i) {
        a->buf[(size_t)a->offset + i] = chunk[i];
    }
    a->offset = (uint8_t)(a->offset + remaining);

    if (flags & CB_PARAM_FLAG_MORE) {
        /* ③ 已声明 More 但缓冲已填满 → 协议错误，中止 */
        if (a->offset >= a->total_len) {
            cb_param_write_asm_init(a);
            return -1;
        }
        return 1;                       /* 还需后续块 */
    }

    /* 末块：一次交付完整值 */
    if (value_out) {
        for (i = 0u; i < CB_PARAM_MAX_VALUE; ++i) value_out[i] = a->buf[i];
    }
    if (value_len_out) *value_len_out = a->total_len;

    cb_param_write_asm_init(a);
    return 0;
}

/* ======================== src/proto_cyberbeast/cb_query.c ======================== */
/**
 * @file    cb_query.c
 * @brief   CYBERBEAST 状态查询帧编解码实现（见 cb_query.h）
 *
 * 与固件 can_cyberbeast.cpp 的 cmd_query_* 系列逐字节对齐。
 */


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

/* ======================== src/core/jsdk_config.c ======================== */
/**
 * @file    jsdk_config.c
 * @brief   配置阶段：configure / activate / deactivate / discover / 参数读写
 *
 * 这些都是**阻塞** API（DESIGN §7 的约定）：内部用 `jsdk_ctx_read_param()` /
 * `jsdk_ctx_wait_response()` 轮询，允许最长 `JSDK_CFG_TIMEOUT_MS`。
 * 绝不可在控制循环运行期间调用（会与响应争用并阻塞周期）。
 */



/* ==========================================================================
 * 必需的端点路径（**不内置 ID**，只内置"名字"；ID 由描述符解析而来）
 * ======================================================================== */

#define P_GEAR        "axis0.motor.config.gear_ratio"
#define P_TCONST      "axis0.motor.config.torque_constant"
#define P_MIT_POS     "axis0.controller.config.mit_max_pos"
#define P_MIT_VEL     "axis0.controller.config.mit_max_vel"
#define P_MIT_TAU     "axis0.controller.config.mit_max_torque"
#define P_MIT_KP      "axis0.controller.config.mit_max_kp"
#define P_MIT_KD      "axis0.controller.config.mit_max_kd"
#define P_REQUESTED   "axis0.requested_state"
#define P_CURRENT_ST  "axis0.current_state"
#define P_NODE_ID     "axis0.config.can.node_id"
#define P_BREAK       "can.config.break_timeout"

/* ==========================================================================
 * 参数读写的阻塞实现
 * ======================================================================== */

/**
 * 读一个参数（探索式单发）。
 *
 * 用 4 字节请求（不带 offset）：设备把它归一化为 `ReqLen = 4`，一次拿回
 * 前 4 字节。适合值 ≤ 4 字节的场景（float/u32/u16/u8/bool，即全部标定量程）。
 * 要读满 8 字节的类型用 @ref jsdk_ctx_read_param_exact。
 */
int jsdk_ctx_read_param(jsdk_context_t *ctx, uint8_t node_id, uint16_t ep_id,
                        uint8_t *out, uint8_t *out_len, uint32_t timeout_ms)
{
    uint8_t  req[CB_PARAM_READ_REQ_MIN];
    jsdk_can_frame_t rsp;
    size_t   n;
    int      rc;

    if (!jsdk_ctx_check(ctx) || !out) return JSDK_ERR_INVALID_ARG;

    n = cb_param_pack_read_req(req, sizeof req, ep_id, 4u, 0u, 0);
    if (n == 0u) return JSDK_ERR_INVALID_ARG;

    rc = jsdk_ctx_send(ctx, CB_PRI_CONFIG, CB_MSG_PARAM_READ, node_id,
                       req, (uint8_t)n);
    if (rc != 0) return JSDK_ERR_TRANSPORT;
    ctx->tx_seq = cb_seq_next(ctx->tx_seq);

    rc = jsdk_ctx_wait_response(ctx, CB_MSG_PARAM_READ, node_id, &rsp,
                               timeout_ms ? timeout_ms : JSDK_CFG_TIMEOUT_MS);
    if (rc != JSDK_OK) return rc;

    {
        cb_param_read_rsp_t r;
        if (cb_param_unpack_read_rsp(rsp.data, rsp.len, &r) != 0) {
            return JSDK_ERR_PROTOCOL;
        }
        if (r.ep_id != ep_id) return JSDK_ERR_PROTOCOL;   /* 应答串味 */
        if (r.data_len == 0u) return JSDK_ERR_NOT_FOUND;  /* 设备不认识该端点 */
        memcpy(out, r.value, r.data_len);
        if (out_len) *out_len = r.data_len;
    }
    return JSDK_OK;
}

/**
 * 读一个参数，**精确读满 want 字节**（必要时分块）。
 *
 * 背景（WP9 查实）：固件按“设备侧归一化”的 `ReqLen` 切片：
 *   FD 一次 ≤ 8 B、Classic 一次 ≤ 4 B；超出的部分靠 `offset` 续读。
 * 本函数之前只有“写死 4 B 请求”的实现，而 `jsdk_joint_param_get()` 却按
 * **描述符里的类型长度**（`u64/i64/f64` = 8）去校验，于是那两个 8 字节端点
 * （`serial_number`、`axis0.motor.error`）**永远读不出来**，而且报的是
 * “设备只回了 4 字节，而描述符说 8 字节”——把矛头指向固件/描述符，
 * 实际原因在主站自己的请求里。
 */
int jsdk_ctx_read_param_exact(jsdk_context_t *ctx, uint8_t node_id, uint16_t ep_id,
                              uint8_t *out, uint8_t want, uint8_t *out_len,
                              uint32_t timeout_ms)
{
    const int classic = (ctx && ctx->cfg.is_fd == 0u) ? 1 : 0;
    uint8_t  cap;          /* 一次请求最多能拿回多少字节（设备侧归一化后） */
    uint32_t offset = 0u;
    uint8_t  got   = 0u;

    if (!jsdk_ctx_check(ctx) || !out) return JSDK_ERR_INVALID_ARG;
    if (want == 0u || want > CB_PARAM_MAX_VALUE) return JSDK_ERR_INVALID_ARG;
    if (out_len) *out_len = 0u;

    cap = cb_param_normalize_req_len(CB_PARAM_MAX_VALUE, classic);

    while (got < want) {
        uint8_t  need = (uint8_t)(want - got);          /* 还差多少字节 */
        uint8_t  req[CB_PARAM_READ_REQ_FULL];
        jsdk_can_frame_t rsp;
        cb_param_read_rsp_t r;
        uint8_t  req_len;
        int      with_offset;
        size_t   n;
        int      rc;

        /* 本块最多能要多少：不能超过设备一次能给的上限 */
        req_len = (need < cap) ? need : cap;

        /*
         * 第一块用 4 B 旧式形式（offset 隐含 0）—— 兼容性最好；
         * 后续块必须带 offset（8 B 形式），否则设备会从 0 重新开始。
         */
        with_offset = (offset > 0u) ? 1 : 0;

        n = cb_param_pack_read_req(req, sizeof req, ep_id, req_len, offset,
                                   with_offset);
        if (n == 0u) return JSDK_ERR_INVALID_ARG;

        rc = jsdk_ctx_send(ctx, CB_PRI_CONFIG, CB_MSG_PARAM_READ, node_id,
                           req, (uint8_t)n);
        if (rc != 0) return JSDK_ERR_TRANSPORT;
        ctx->tx_seq = cb_seq_next(ctx->tx_seq);

        rc = jsdk_ctx_wait_response(ctx, CB_MSG_PARAM_READ, node_id, &rsp,
                                   timeout_ms ? timeout_ms : JSDK_CFG_TIMEOUT_MS);
        if (rc != JSDK_OK) return rc;

        if (cb_param_unpack_read_rsp(rsp.data, rsp.len, &r) != 0) {
            return JSDK_ERR_PROTOCOL;
        }
        if (r.ep_id != ep_id) return JSDK_ERR_PROTOCOL;      /* 应答串味 */

        if (r.data_len == 0u) {
            /* 第一块就 0 字节 = 设备不认识该端点；中途 0 字节 = 对端行为不一致 */
            if (got == 0u) return JSDK_ERR_NOT_FOUND;
            jsdk_ctx_seterr(ctx, "param read stopped at byte %u (device returned 0 "
                                  "bytes, value truncated)", (unsigned)got);
            return JSDK_ERR_PROTOCOL;
        }
        if (r.data_len > need) {
            /* 设备给了比我们要的更多：宁可报错也不截断后当成“读成功” */
            jsdk_ctx_seterr(ctx, "param read got %u bytes, asked for %u",
                            (unsigned)r.data_len, (unsigned)need);
            return JSDK_ERR_PROTOCOL;
        }

        memcpy(out + got, r.value, r.data_len);
        got    = (uint8_t)(got + r.data_len);
        offset = offset + (uint32_t)r.data_len;

        /* 已经拿够；若设备还想继续给，那是它的 full_len 比描述符长，与我们无关 */
        if (got >= want) break;

        if (!cb_param_rsp_has_more(&r)) {
            /* 没有 More 但还没读满 → 设备里的值比描述符声明的短 */
            jsdk_ctx_seterr(ctx, "param read got %u bytes, but %u were expected "
                                  "(value shorter than the descriptor declares)",
                            (unsigned)got, (unsigned)want);
            return JSDK_ERR_PROTOCOL;
        }
    }

    if (out_len) *out_len = got;
    return JSDK_OK;
}

/**
 * 写一个参数值。
 *
 * 三种走法（固件对写入长度有硬约束，选错会**静默写不进去**）：
 *
 * | 值长度 | 帧类型 | 做法 |
 * |---|---|---|
 * | ≤ 4 B | 任意 | 一帧写完（4 + 4 = 8 B 载荷） |
 * | 5..8 B | **FD** | 一帧写完（4 + 8 = 12 B ≤ 64） |
 * | 5..8 B | **Classic** | **必须分段**：每块 4 B，末块补齐（固件在 Classic 下不接受单帧长值） |
 *
 * @param val 值字节，**必须是线上大端序**（见头文件说明）。
 */
int jsdk_ctx_write_param(jsdk_context_t *ctx, uint8_t node_id, uint16_t ep_id,
                         const void *val, uint8_t len, uint32_t timeout_ms)
{
    const uint8_t *p = (const uint8_t *)val;
    uint8_t  req[CB_PARAM_READ_REQ_FULL + CB_PARAM_MAX_VALUE];
    size_t   n;
    int      rc;

    if (!jsdk_ctx_check(ctx) || !val) return JSDK_ERR_INVALID_ARG;
    if (len == 0u || len > CB_PARAM_MAX_VALUE) return JSDK_ERR_INVALID_ARG;

    /* --- 单帧可达（≤ 4 B，或 FD 下 ≤ 8 B）--- */
    if (len <= 4u || ctx->cfg.is_fd) {
        n = cb_param_pack_write_req(req, sizeof req, ep_id, p, len);
        if (n == 0u) return JSDK_ERR_INVALID_ARG;

        rc = jsdk_ctx_send(ctx, CB_PRI_CONFIG, CB_MSG_PARAM_WRITE, node_id,
                           req, (uint8_t)n);
        if (rc != 0) return JSDK_ERR_TRANSPORT;
        ctx->tx_seq = cb_seq_next(ctx->tx_seq);

        if (timeout_ms != 0u) {
            /* 写确认（8 B 静默 ACK）；等一等能立刻发现"端点不存在" */
            jsdk_can_frame_t ack;
            return jsdk_ctx_wait_response(ctx, CB_MSG_PARAM_WRITE, node_id,
                                          &ack, timeout_ms);
        }
        return JSDK_OK;
    }

    /* --- Classic 且 > 4 B：分块写，末块必须恰好补齐值 --- */
    {
        uint8_t off = 0u;

        while (off < len) {
            uint8_t chunk = (uint8_t)(len - off);
            int     more;

            if (chunk > CB_PARAM_CHUNK_BYTES) chunk = CB_PARAM_CHUNK_BYTES;
            more = (off + chunk < len) ? 1 : 0;

            n = cb_param_pack_write_chunk(req, sizeof req, ep_id, len, off,
                                          p + off, chunk, more);
            if (n == 0u) return JSDK_ERR_INVALID_ARG;
            if (jsdk_ctx_send(ctx, CB_PRI_CONFIG, CB_MSG_PARAM_WRITE, node_id,
                              req, (uint8_t)n) != 0) {
                return JSDK_ERR_TRANSPORT;
            }
            ctx->tx_seq = cb_seq_next(ctx->tx_seq);
            off = (uint8_t)(off + chunk);
        }
    }

    if (timeout_ms != 0u) {
        jsdk_can_frame_t ack;      /* 装配完成时设备回一次 ACK */
        return jsdk_ctx_wait_response(ctx, CB_MSG_PARAM_WRITE, node_id,
                                      &ack, timeout_ms);
    }
    return JSDK_OK;
}

/** 写一个参数并等待设备的 8 字节静默 ACK（用于需要确认的场景）。 */
static int write_param_sync(jsdk_context_t *ctx, uint8_t node_id, uint16_t ep_id,
                            const void *val, uint8_t len, uint32_t timeout_ms)
{
    jsdk_can_frame_t ack;
    int rc = jsdk_ctx_write_param(ctx, node_id, ep_id, val, len, timeout_ms);
    if (rc != JSDK_OK) return rc;

    /* 设备对写入回 8 B 的 `[0]=0, [1]=0, [2..3]=ep_id`；不等它也不会出错，
       但等一等能立刻发现"端点不存在"。 */
    return jsdk_ctx_wait_response(ctx, CB_MSG_PARAM_WRITE, node_id, &ack,
                                 timeout_ms ? timeout_ms : JSDK_CFG_TIMEOUT_MS);
}

/* ==========================================================================
 * 端点解析（名字 → ID）
 * ======================================================================== */

static int resolve_ep(jsdk_context_t *ctx, const char *path, uint16_t *out_id,
                      jsdk_ep_type_t *out_type, uint8_t *out_access)
{
    return jsdk_ep_store_lookup(&ctx->store, path, out_id, out_type, out_access);
}

/** 读一个 float 端点并做有限性检查。 */
static int read_f32(jsdk_context_t *ctx, uint8_t node, uint16_t ep_id, float *out)
{
    uint8_t buf[8];
    uint8_t len = 0u;

    if (jsdk_ctx_read_param(ctx, node, ep_id, buf, &len, 0u) != JSDK_OK) return -1;
    if (len < 4u) return -1;
    *out = cb_be_get_f32(buf);
    return 0;
}

/** 读一个整数端点（1/2/4 字节，小端无关：协议是 BE，但数值宽度按类型）。 */
static int read_int(jsdk_context_t *ctx, uint8_t node, uint16_t ep_id,
                    jsdk_ep_type_t type, uint32_t *out)
{
    uint8_t buf[8];
    uint8_t len = 0u;
    uint8_t need = (uint8_t)jsdk_ep_type_size(type);

    if (need == 0u || need > 8u) return -1;
    if (jsdk_ctx_read_param(ctx, node, ep_id, buf, &len, 0u) != JSDK_OK) return -1;
    if (len < need) return -1;

    /* 设备回的是**大端**的原始字节；按类型宽度取 */
    switch (type) {
    case JSDK_EP_U8:  case JSDK_EP_BOOL: *out = (uint32_t)buf[0]; break;
    case JSDK_EP_U16: *out = (uint32_t)cb_be_get_u16(buf); break;
    case JSDK_EP_U32: *out = cb_be_get_u32(buf); break;
    default: return -1;
    }
    return 0;
}

/* ==========================================================================
 * configure()
 * ======================================================================== */

/** 握手：发一帧让设备学到 master_id（否则心跳发往 0x01）。 */
static void handshake(jsdk_context_t *ctx, uint8_t node)
{
    (void)jsdk_ctx_send(ctx, CB_PRI_QUERY, CB_MSG_QUERY_STATUS, node, NULL, 0u);
    ctx->tx_seq = cb_seq_next(ctx->tx_seq);
}

/**
 * 从描述符解析出该关节的全部必需端点并读回标定值。
 *
 * @note 端点缺失 → `JSDK_ERR_NOT_FOUND`（**不猜、不近似**）。
 *       数值不合理 → `JSDK_ERR_PROTOCOL` 且 `calibrated = 0`。
 */
static jsdk_status_t calibrate_joint(jsdk_context_t *ctx, jsdk_joint_t *j)
{
    unsigned i;
    uint16_t ep;

    /* --- 1. 端点解析（缺失即失败，绝不猜） --- */
    {
        static const char *const required_paths[] = {
            P_GEAR, P_TCONST, P_MIT_POS, P_MIT_VEL, P_MIT_TAU, P_MIT_KP, P_MIT_KD,
        };
        uint16_t *const slots[] = {
            &j->ep_gear_ratio, &j->ep_torque_constant, &j->ep_mit_pos,
            &j->ep_mit_vel, &j->ep_mit_tau, &j->ep_mit_kp, &j->ep_mit_kd,
        };
        for (i = 0u; i < sizeof required_paths / sizeof required_paths[0]; ++i) {
            if (resolve_ep(ctx, required_paths[i], &ep, NULL, NULL) != JSDK_OK) {
                jsdk_joint_seterr(j, "required endpoint missing: %s", required_paths[i]);
                return JSDK_ERR_NOT_FOUND;
            }
            *slots[i] = ep;
        }
    }

    /* 非必需端点：缺失只影响相应功能（不阻塞 configure） */
    (void)resolve_ep(ctx, P_REQUESTED,  &j->ep_requested_state, NULL, NULL);
    (void)resolve_ep(ctx, P_CURRENT_ST, &j->ep_current_state, NULL, NULL);
    (void)resolve_ep(ctx, P_NODE_ID,    &j->ep_node_id, NULL, NULL);
    (void)resolve_ep(ctx, P_BREAK,      &j->ep_break_timeout, NULL, NULL);

    /* --- 2. 读回标定值（客户端显式给的非 0 值优先，便于离线/异常固件兜底） --- */
    if (j->cfg.gear_ratio != 0.0f) {
        j->gear_ratio = j->cfg.gear_ratio;
    } else if (read_f32(ctx, j->cfg.node_id, j->ep_gear_ratio, &j->gear_ratio) != 0) {
        jsdk_joint_seterr(j, "read %s failed", P_GEAR);
        return JSDK_ERR_TRANSPORT;
    }
    if (j->cfg.torque_constant != 0.0f) {
        j->torque_constant = j->cfg.torque_constant;
    } else if (read_f32(ctx, j->cfg.node_id, j->ep_torque_constant,
                        &j->torque_constant) != 0) {
        jsdk_joint_seterr(j, "read %s failed", P_TCONST);
        return JSDK_ERR_TRANSPORT;
    }

    if (j->cfg.mit_max_pos != 0.0f)        j->range.pos_max = j->cfg.mit_max_pos;
    else if (read_f32(ctx, j->cfg.node_id, j->ep_mit_pos, &j->range.pos_max) != 0) {
        jsdk_joint_seterr(j, "read %s failed", P_MIT_POS);
        return JSDK_ERR_TRANSPORT;
    }
    if (j->cfg.mit_max_vel != 0.0f)        j->range.vel_max = j->cfg.mit_max_vel;
    else if (read_f32(ctx, j->cfg.node_id, j->ep_mit_vel, &j->range.vel_max) != 0) {
        jsdk_joint_seterr(j, "read %s failed", P_MIT_VEL);
        return JSDK_ERR_TRANSPORT;
    }
    if (j->cfg.mit_max_torque != 0.0f)     j->range.tau_max = j->cfg.mit_max_torque;
    else if (read_f32(ctx, j->cfg.node_id, j->ep_mit_tau, &j->range.tau_max) != 0) {
        jsdk_joint_seterr(j, "read %s failed", P_MIT_TAU);
        return JSDK_ERR_TRANSPORT;
    }
    if (j->cfg.mit_max_kp != 0.0f)         j->range.kp_max = j->cfg.mit_max_kp;
    else if (read_f32(ctx, j->cfg.node_id, j->ep_mit_kp, &j->range.kp_max) != 0) {
        jsdk_joint_seterr(j, "read %s failed", P_MIT_KP);
        return JSDK_ERR_TRANSPORT;
    }
    if (j->cfg.mit_max_kd != 0.0f)         j->range.kd_max = j->cfg.mit_max_kd;
    else if (read_f32(ctx, j->cfg.node_id, j->ep_mit_kd, &j->range.kd_max) != 0) {
        jsdk_joint_seterr(j, "read %s failed", P_MIT_KD);
        return JSDK_ERR_TRANSPORT;
    }

    /* --- 3. 设备侧杂项（缺失不致命） --- */
    if (j->ep_break_timeout != 0u) {
        uint32_t v = 0u;
        jsdk_ep_type_t t = JSDK_EP_U16;
        if (resolve_ep(ctx, P_BREAK, NULL, &t, NULL) == JSDK_OK
            && read_int(ctx, j->cfg.node_id, j->ep_break_timeout, t, &v) == 0) {
            j->break_timeout_ms = v;
        }
    }
    if (j->ep_node_id != 0u) {
        uint32_t v = 0u;
        jsdk_ep_type_t t = JSDK_EP_U32;
        if (resolve_ep(ctx, P_NODE_ID, NULL, &t, NULL) == JSDK_OK
            && read_int(ctx, j->cfg.node_id, j->ep_node_id, t, &v) == 0) {
            j->node_id_readback = v;
        }
    }

    /* 心跳周期：WP4 的反馈超时阈值靠它推导，读不到就用保守下限 */
    {
        uint16_t hb_ep = 0u;
        jsdk_ep_type_t hb_t = JSDK_EP_U32;
        if (resolve_ep(ctx, "axis0.config.can.heartbeat_rate_ms", &hb_ep, &hb_t, NULL)
            == JSDK_OK) {
            uint32_t v = 0u;
            if (read_int(ctx, j->cfg.node_id, hb_ep, hb_t, &v) == 0) {
                j->heartbeat_rate_ms = v;
            }
        }
    }

    /* 当前状态（固件 AxisState）：标定/回零的“是否完成”就靠它 */
    if (resolve_ep(ctx, "axis0.current_state", &j->ep_current_state, NULL, NULL)
        == JSDK_OK) {
        uint8_t buf[8];
        uint8_t len = 0u;
        if (jsdk_ctx_read_param(ctx, j->cfg.node_id, j->ep_current_state,
                                buf, &len, 0u) == JSDK_OK && len >= 1u) {
            j->current_state_raw = buf[0];
            j->state_known = 1u;
        }
    }

    /* --- 4. 数值合理性 + 派生量 --- */
    jsdk_joint__apply_calibration(j);

    /* --- 5. 周期 vs 看门狗（§6.3）--- */
    if (ctx->cfg.period_ns != 0u) {
        uint32_t period_ms = (uint32_t)(ctx->cfg.period_ns / 1000000u);
        uint32_t wd = jsdk_watchdog_device_ms(j);

        if (ctx->cfg.enable_watchdog_hint && j->ep_break_timeout != 0u) {
            uint16_t want = (uint16_t)(period_ms * 2u);
            uint8_t  want_be[2];
            if (want < 2u) want = 2u;
            cb_be_put_u16(want_be, want);      /* ⚠ 线上一律大端 */
            if (write_param_sync(ctx, j->cfg.node_id, j->ep_break_timeout,
                                 want_be, 2u, 0u) == JSDK_OK) {
                j->break_timeout_ms = want;
                wd = want;
            }
        }
        if (period_ms != 0u && period_ms >= wd) {
            jsdk_joint_seterr(j,
                "control period %u ms >= device break_timeout %u ms: the loop "
                "cannot feed the protocol watchdog", (unsigned)period_ms,
                (unsigned)wd);
            return JSDK_ERR_BAD_STATE;
        }
    }

    if (!j->calibrated) {
        jsdk_joint_seterr(j, "calibration values out of range; physical-unit API "
                             "is disabled for this joint");
        return JSDK_ERR_PROTOCOL;
    }
    return JSDK_OK;
}

jsdk_status_t jsdk_context_configure(jsdk_context_t *ctx)
{
    unsigned i;
    jsdk_status_t st;

    if (!jsdk_ctx_check(ctx)) return JSDK_ERR_INVALID_ARG;
    if (ctx->nj == 0u) return JSDK_ERR_BAD_STATE;
    if (!ctx->cfg.desc.arena || ctx->cfg.desc.arena_size == 0u) {
        jsdk_ctx_seterr(ctx, "desc.arena is required (this SDK never allocates)");
        return JSDK_ERR_INVALID_ARG;
    }
    if (jsdk_ctx_any_enabled(ctx)) {
        jsdk_ctx_seterr(ctx, "configure() refused while a joint is enabled");
        return JSDK_ERR_BAD_STATE;
    }

    /* --- 1. 描述符（三条路线：已在手 → 不下载；FROM_CACHE → 必须有；否则下载） --- */
    if (!ctx->desc_present) {
        if (ctx->cfg.desc.mode == JSDK_DESC_FROM_CACHE) {
            jsdk_ctx_seterr(ctx, "descriptor not supplied but mode is FROM_CACHE; "
                                 "import a cache (or raw JSON) before configure()");
            return JSDK_ERR_BAD_STATE;
        }
        st = jsdk_context_desc_fetch(ctx);
        if (st != JSDK_OK) {
            if (st == JSDK_ERR_NO_MEMORY) {
                jsdk_ctx_seterr(ctx, "descriptor arena too small: needs >= %u bytes "
                                     "(given %u); see jsdk_desc_arena_size()",
                                (unsigned)ctx->cfg.desc.arena_size,
                                (unsigned)ctx->cfg.desc.arena_size);
            }
            return st;
        }
    }

    /* --- 2. 逐个关节：握手 + 标定 --- */
    for (i = 0u; i < ctx->nj; ++i) {
        jsdk_joint_t *j = &ctx->joints[i];

        handshake(ctx, j->cfg.node_id);
        {
            /* 握手帧的应答顺手确认设备在不在 */
            jsdk_can_frame_t rsp;
            if (jsdk_ctx_wait_response(ctx, CB_MSG_MIT_CONTROL, j->cfg.node_id,
                                       &rsp, JSDK_CFG_TIMEOUT_MS) == JSDK_OK) {
                jsdk_joint__on_mit_response(j, rsp.data, rsp.len);
            } else {
                jsdk_joint_seterr(j, "no response to handshake (node %u): check "
                                     "node_id / wiring / master_id",
                                  (unsigned)j->cfg.node_id);
                return JSDK_ERR_TIMEOUT;
            }
        }

        st = calibrate_joint(ctx, j);
        if (st != JSDK_OK) return st;
    }

    jsdk_ctx_seterr(ctx, "configured: %u joint(s), %u endpoints",
                    ctx->nj, ctx->desc.endpoint_count);

    /* --- 组/广播能力的前置告警（§5.10）---
       位图只有 node_id 1..7 这 7 个可用位；≥8 的关节**不会被广播命中**
       （固件对 node_id ≥ 8 直接判 `is_for_me == 0`），只能单播。
       这不是错误（单播完全可用），但客户若打算用 group 同步，必须提前知道。 */
    {
        unsigned n_big = 0u;
        unsigned k;
        for (k = 0u; k < ctx->nj; ++k) {
            if (ctx->joints[k].cfg.node_id >= CB_MAX_BROADCAST_DEVICES) n_big++;
        }
        if (n_big > 0u) {
            /* ⚠ 不能用 %s 拼中间缓冲：seterr 本身就是 printf 风格，
               再套一层只会让格式化字符串的实参顺序更难核对。
               ⚠ 缓冲要够大、文案要够短：这条消息会和后面的 "configured: …"
               一起塞进 `last_error`（`JSDK_ERRSTR_LEN` = 192 B）。
               原先 `warn[128]` 在最坏情况下（%u 各 10 位）需要 134~152 B，
               **GCC 9 的 `-Wformat-truncation` 当场算了出来** —— 也就是说这
               条警告有可能被截断，而截掉的正是最有用的那半句 "(use unicast)"。
               现在文案压到最坏 105 B，加上前缀 42 B 仍不到 192 B。 */
            char warn[JSDK_ERRSTR_LEN];
            (void)snprintf(warn, sizeof warn,
                           "WARNING: %u/%u joint(s) have node_id >= %u "
                           "(no bitmap addressing); use unicast",
                           n_big, ctx->nj, (unsigned)CB_MAX_BROADCAST_DEVICES);
            jsdk_ctx_seterr(ctx, "configured: %u joint(s), %u endpoints; %s",
                            ctx->nj, ctx->desc.endpoint_count, warn);
        }
    }
    return JSDK_OK;
}

/* ==========================================================================
 * activate() / deactivate()
 * ======================================================================== */

jsdk_status_t jsdk_context_activate(jsdk_context_t *ctx)
{
    unsigned i;
    uint32_t t0;

    if (!jsdk_ctx_check(ctx)) return JSDK_ERR_INVALID_ARG;
    if (ctx->nj == 0u) return JSDK_ERR_BAD_STATE;

    /* --- 1. 先做完整前置校验，任何一个不合格就整体拒绝（不改状态） --- */
    for (i = 0u; i < ctx->nj; ++i) {
        if (!ctx->joints[i].calibrated) {
            jsdk_ctx_seterr(ctx, "activate refused: joint %u (node %u) has no valid "
                                 "calibration", i, (unsigned)ctx->joints[i].cfg.node_id);
            return JSDK_ERR_BAD_STATE;
        }
    }

    /*
     * --- 2. 逐关节**发使能请求**，把实际动作交给 L3 的使能序列 ---
     *
     * ⚠ 这里刻意不再自己重放 CLEAR_ERRORS / START_MOTOR / 等闭环。
     *   早期版本那样做，于是"使能"有两套实现：
     *     A) 客户 `jsdk_joint_request_enable()` + 自己的循环（走 advance_seq）
     *     B) `jsdk_context_activate()`（自己一套阻塞循环）
     *   而 `enabled` 被 `!enable_pending` 门控，A 会置 `enable_pending = 1`，
     *   B 的等待循环又**从不调用 cycle_end**，所以 pending 永远清不掉 ——
     *   两种用法一混，activate() 必然超时（Python 里
     *   `j.enable(MIT); ctx.activate()` 就是这样，而纯 C 只调 activate()
     *   反而成功，因为没人置 pending）。
     *   头文件承诺 activate() "等价于逐个 request_enable + 等待就绪"，
     *   所以可混用是**契约**。现在只有一份实现，混用不可能再出问题；
     *   使能序列里的安全首帧（§6.4）也照旧由 advance_seq 发出。
     */
    for (i = 0u; i < ctx->nj; ++i) {
        jsdk_joint_t *j = &ctx->joints[i];
        if (!j->enable_pending && !jsdk_joint_is_enabled(j)) {
            jsdk_joint_request_enable(j, j->cfg.initial_mode);
        }
    }

    /* --- 3. 推进周期（收响应 + 跑序列）直到全部就绪 --- */
    t0 = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);

    for (;;) {
        unsigned ready = 0u;

        for (i = 0u; i < ctx->nj; ++i) {
            if (jsdk_joint_is_enabled(&ctx->joints[i])) ready++;
        }
        if (ready == ctx->nj) break;

        (void)jsdk_context_cycle_begin(ctx, 0u);   /* 收帧 → 更新 mode_state */
        (void)jsdk_context_cycle_end(ctx);         /* 推进使能序列 + 安全首帧 */

        ctx->now_ms = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);
        if (jsdk_elapsed(ctx->now_ms, t0) > JSDK_ACTIVATE_TIMEOUT_MS) {
            /* 报出**哪一个**关节卡住、卡在哪一步 —— 现场就靠这条信息定位 */
            for (i = 0u; i < ctx->nj; ++i) {
                jsdk_joint_t *j = &ctx->joints[i];
                if (!jsdk_joint_is_enabled(j)) {
                    jsdk_ctx_seterr(ctx,
                        "activate timed out after %u ms: joint %u (node %u) "
                        "mode_state=%u (step %u, %s)",
                        (unsigned)JSDK_ACTIVATE_TIMEOUT_MS, i,
                        (unsigned)j->cfg.node_id, (unsigned)j->fb.mode_state,
                        (unsigned)j->seq_step,
                        jsdk_joint_is_fault(j) ? "device reports a fault"
                                               : "no fault reported");
                    break;
                }
            }
            return JSDK_ERR_TIMEOUT;
        }
    }

    jsdk_ctx_seterr(ctx, "activated %u joint(s)", ctx->nj);
    return JSDK_OK;
}

void jsdk_context_deactivate(jsdk_context_t *ctx)
{
    unsigned i;

    if (!jsdk_ctx_check(ctx)) return;

    for (i = 0u; i < ctx->nj; ++i) {
        jsdk_joint_t *j = &ctx->joints[i];
        uint32_t t0;

        /*
         * ⚠ 先掐掉**所有**排队中的请求，再考虑标定与否。
         *
         *   enable_pending 必须清：advance_seq 里的使能分支是无条件执行的，只要
         *   它还是 1，本函数返回后的**下一个 cycle_end 就会把电机重新使能** ——
         *   也就是"安全关闭"被一个陈旧的使能请求悄悄撤销。这个序列实测复现过
         *   （request_enable() 后不跑周期，直接 deactivate()，再跑 10 个周期
         *   关节又回来了）。deactivate() 的调用方（含 jsdk-cli stop、Python
         *   Context.close()）都把它当成"断电已完成"，所以这里不能留尾巴。
         *
         *   未标定的关节也必须清：否则跳过本轮循环会把悬空请求留到下次。
         */
        j->enable_pending     = 0u;
        j->faultreset_pending = 0u;
        j->seq_step           = 0u;

        if (!j->calibrated) { j->disable_pending = 0u; continue; }

        /* 顺序不可颠倒（§6.3）：先发安全帧 → 等 2 周期 → STOP_MOTOR → 等 IDLE。
           "停发控制帧"会直接触发协议级超时故障。 */
        jsdk_joint_hold_position(j);
        ctx->now_ms = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);
        jsdk_joint__cycle_end_all(ctx);

        (void)jsdk_ctx_send(ctx, CB_PRI_CTRL, CB_MSG_STOP_MOTOR, j->cfg.node_id, NULL, 0u);
        ctx->tx_seq = cb_seq_next(ctx->tx_seq);

        t0 = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);
        for (;;) {
            jsdk_can_frame_t rsp;

            (void)jsdk_ctx_send(ctx, CB_PRI_QUERY, CB_MSG_QUERY_STATUS,
                                j->cfg.node_id, NULL, 0u);
            ctx->tx_seq = cb_seq_next(ctx->tx_seq);
            if (jsdk_ctx_wait_response(ctx, CB_MSG_MIT_CONTROL, j->cfg.node_id,
                                       &rsp, JSDK_CFG_TIMEOUT_MS) != JSDK_OK) {
                break;
            }
            jsdk_joint__on_mit_response(j, rsp.data, rsp.len);
            if (j->fb.mode_state == JSDK_MODESTATE_IDLE
                || j->fb.mode_state == JSDK_MODESTATE_RESET) {
                break;
            }
            ctx->now_ms = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);
            if (jsdk_elapsed(ctx->now_ms, t0) > JSDK_CFG_TIMEOUT_MS) break;
        }
        j->enabled = 0u;
        j->disable_pending = 0u;
        j->seq_step = 0u;
        j->tx_active = 0u;
    }
    jsdk_ctx_seterr(ctx, "deactivated");
}

/* ==========================================================================
 * 设备信息 / 故障详情 / 配置快照
 * ======================================================================== */

jsdk_status_t jsdk_joint_get_device_info(jsdk_joint_t *j, jsdk_device_info_t *info)
{
    jsdk_can_frame_t rsp;
    cb_query_device_info_t d;
    int rc;

    if (!jsdk_joint_check(j) || !info) return JSDK_ERR_INVALID_ARG;
    memset(info, 0, sizeof *info);

    rc = jsdk_ctx_send(j->ctx, CB_PRI_QUERY, CB_MSG_QUERY_DEVICE_INFO,
                       j->cfg.node_id, NULL, 0u);
    if (rc != 0) return JSDK_ERR_TRANSPORT;
    j->ctx->tx_seq = cb_seq_next(j->ctx->tx_seq);

    rc = jsdk_ctx_wait_response(j->ctx, CB_MSG_QUERY_DEVICE_INFO, j->cfg.node_id,
                                &rsp, JSDK_CFG_TIMEOUT_MS);
    if (rc != JSDK_OK) return (jsdk_status_t)rc;

    if (cb_query_decode_device(rsp.data, rsp.len, &d) != 0) {
        return JSDK_ERR_PROTOCOL;
    }
    info->hw_version = d.hw_ver;
    info->fw_version = d.fw_ver;
    info->serial     = d.has_serial ? d.serial : 0u;
    info->classic    = (uint8_t)(j->ctx->cfg.is_fd ? 0u : 1u);
    return JSDK_OK;
}

jsdk_status_t jsdk_joint_query_error_detail(jsdk_joint_t *j, jsdk_fault_info_t *info)
{
    static const uint8_t types[6] = {
        CB_ET_MOTOR, CB_ET_ENCODER, CB_ET_SENSORLESS,
        CB_ET_CONTROLLER, CB_ET_SYSTEM, CB_ET_AXIS
    };
    uint32_t vals[6];
    unsigned i;

    if (!jsdk_joint_check(j) || !info) return JSDK_ERR_INVALID_ARG;

    memset(vals, 0, sizeof vals);
    for (i = 0u; i < 6u; ++i) {
        uint8_t req[1];
        jsdk_can_frame_t rsp;
        cb_query_error_t e;
        int rc;

        req[0] = types[i];
        if (jsdk_ctx_send(j->ctx, CB_PRI_QUERY, CB_MSG_QUERY_ERROR, j->cfg.node_id,
                          req, 1u) != 0) return JSDK_ERR_TRANSPORT;
        j->ctx->tx_seq = cb_seq_next(j->ctx->tx_seq);

        rc = jsdk_ctx_wait_response(j->ctx, CB_MSG_QUERY_ERROR, j->cfg.node_id,
                                    &rsp, JSDK_CFG_TIMEOUT_MS);
        if (rc != JSDK_OK) return (jsdk_status_t)rc;
        if (cb_query_decode_error(rsp.data, rsp.len, &e) != 0) return JSDK_ERR_PROTOCOL;
        vals[i] = e.err_value;
    }

    info->valid            = 1;
    info->mit_err          = j->fb.err_code;
    info->hb_flags         = j->fb.hb_error;
    info->motor_error      = vals[0];
    info->encoder_error    = vals[1];
    info->sensorless_error = vals[2];
    info->controller_error = vals[3];
    info->system_error     = vals[4];
    info->axis_error       = vals[5];

    j->fault = *info;
    return JSDK_OK;
}

jsdk_status_t jsdk_joint_get_fault_info(const jsdk_joint_t *j, jsdk_fault_info_t *info)
{
    if (!jsdk_joint_check(j) || !info) return JSDK_ERR_INVALID_ARG;
    *info = j->fault;
    info->mit_err  = j->fb.err_code;
    info->hb_flags = j->fb.hb_error;
    return JSDK_OK;
}

jsdk_status_t jsdk_joint_read_config_snapshot(jsdk_joint_t *j,
                                              jsdk_joint_config_snapshot_t *out)
{
    if (!jsdk_joint_check(j) || !out) return JSDK_ERR_INVALID_ARG;

    memset(out, 0, sizeof *out);
    out->gear_ratio       = j->gear_ratio;
    out->mit_max_pos      = j->range.pos_max;
    out->mit_max_vel      = j->range.vel_max;
    out->mit_max_torque   = j->range.tau_max;
    out->mit_max_kp       = j->range.kp_max;
    out->mit_max_kd       = j->range.kd_max;
    out->torque_constant  = j->torque_constant;
    out->node_id          = j->node_id_readback ? j->node_id_readback : j->cfg.node_id;
    out->heartbeat_rate_ms = 0u;               /* 由 P1 的心跳配置 API 补齐 */
    out->break_timeout_ms = jsdk_watchdog_device_ms(j);
    out->valid            = j->calibrated ? 1 : 0;
    return JSDK_OK;
}

/* ==========================================================================
 * 节点发现
 * ======================================================================== */

/**
 * 目标地址上是否**已经有人在应答**（定向探测，不扫全总线）。
 *
 * 为什么需要它：`jsdk_joint_set_node_id()` 在发完 `SET_NODE_ID` 之后会「验证新地址
 * 能应答」，但那个验证在目标号**已经被别的设备占用**时是假的 —— 答应的其实是
 * 原来那台设备，于是明明改号了、还是有两个同号设备在总线上（行为不确定：谁先答
 * 谁被认）。所以在**改号之前**必须先问一句。
 *
 * 成本：`QUERY_STATUS` 一帧 + 最多 `JSDK_CFG_TIMEOUT_MS / 8`（实测 25 ms）。
 *
 * @return 1 = 有设备应答；0 = 无应答（包含发送失败 —— 发不出去时后续步骤会自己报错）
 */
int jsdk_ctx_probe_node(jsdk_context_t *ctx, uint8_t node_id)
{
    jsdk_can_frame_t rsp;

    if (!jsdk_ctx_check(ctx) || node_id == 0u) return 0;
    if (jsdk_ctx_send(ctx, CB_PRI_QUERY, CB_MSG_QUERY_STATUS, node_id,
                      NULL, 0u) != 0) {
        return 0;
    }
    ctx->tx_seq = cb_seq_next(ctx->tx_seq);

    /* 回复一律以 MsgType 0x00 回来，靠 Source 区分设备（见协议手册） */
    return (jsdk_ctx_wait_response(ctx, CB_MSG_MIT_CONTROL, node_id, &rsp,
                                   JSDK_CFG_TIMEOUT_MS / 8u) == JSDK_OK) ? 1 : 0;
}

jsdk_status_t jsdk_context_discover(jsdk_context_t *ctx, uint8_t *ids, unsigned cap,
                                    unsigned *found, uint8_t max_probe)
{
    uint8_t  seen[256];
    unsigned n = 0u;
    unsigned i;

    if (!jsdk_ctx_check(ctx) || !found) return JSDK_ERR_INVALID_ARG;
    *found = 0u;
    if (!ids || cap == 0u) return JSDK_ERR_INVALID_ARG;
    if (jsdk_ctx_any_enabled(ctx)) {
        jsdk_ctx_seterr(ctx, "discover() refused while a joint is enabled "
                             "(it would contend for responses)");
        return JSDK_ERR_BAD_STATE;
    }

    memset(seen, 0, sizeof seen);

    /* --- 1. 被动：**静置** 2 个心跳周期收集心跳的 Source ---
       ⚠ 必须真的“等”：只排空一次接收队列的话，心跳还没到就已经扫完了，
         结果是“什么都没发现”（而现场设备明明是好的）。
         这里用 HAL 的 `now_ms` 计时 —— 真实 HAL 的自由运行计数器会前进，
         仿真 HAL 则需要调用方/包装层推进时钟。 */
    {
        uint32_t t0 = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);
        uint32_t budget = JSDK_DISCOVER_PASSIVE_MS;
        unsigned spin = 0u;
        const unsigned spin_cap = 4000000u;

        for (;;) {
            jsdk_can_frame_t f;
            int nrc = ctx->cfg.hal.recv(ctx->cfg.hal.user, &f);

            if (nrc > 0) {
                ctx->bus.rx_frames++;
                ctx->last_rx_ms = ctx->now_ms;
                ctx->bus.link_up = 1u;
                if (cb_id_msgtype(f.id) == CB_MSG_HEARTBEAT) {
                    uint8_t src = (uint8_t)cb_id_source(f.id);
                    if (src != 0u && !seen[src] && n < cap) {
                        seen[src] = 1u;
                        ids[n++] = src;
                    }
                } else {
                    (void)jsdk_ctx_handle_frame(ctx, &f);
                }
                continue;
            }
            if (nrc < 0) {
                ctx->bus.link_errors++;
                ctx->bus.link_up = 0u;
                break;
            }

            ctx->now_ms = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);
            if (jsdk_elapsed(ctx->now_ms, t0) >= budget) break;
            if (++spin >= spin_cap) break;      /* 时钟冻结的仿真 HAL 兜底 */
        }
    }

    /* --- 2. 主动：1..max_probe 逐个 QUERY_STATUS --- */
    for (i = 1u; i <= (unsigned)max_probe && n < cap; ++i) {
        if (seen[i]) continue;
        /* 与 set-node-id 的前置探测同一实现，避免两份“定向问一句”的写法漂移 */
        if (jsdk_ctx_probe_node(ctx, (uint8_t)i)) {
            seen[i] = 1u;
            ids[n++] = (uint8_t)i;
        }
    }

    *found = n;
    jsdk_ctx_seterr(ctx, "discovered %u node(s)", n);
    return JSDK_OK;
}

/* ======================== src/core/jsdk_context.c ======================== */
/**
 * @file    jsdk_context.c
 * @brief   上下文：生命周期、循环边界、描述符集成、总线级操作
 *
 * 本文件是 L3 的“外壳”：它把 HAL（L1）与协议编解码（L2）串起来，并对外提供
 * `jsdk_context_*`。关节相关逻辑在 `jsdk_joint.c`，看门狗在 `jsdk_watchdog.c`。
 *
 * @par 两个循环边界的分工
 *   cycle_begin()：读时钟 → 排空 RX（上限 rx_burst_limit）→ 解复用（反馈/响应）
 *   cycle_end()  ：推进各关节状态机 → 编码并发送控制帧/keepalive → 刷新记账
 *
 * @par 配置阶段 API（configure / activate / deactivate / discover / *_sdo_read）
 * 允许阻塞：内部用 `jsdk_ctx_wait_response()` 轮询。**不得**在控制循环运行期间调用。
 */



/* ==========================================================================
 * 编译期守卫：上下文必须塞进调用者提供的存储
 * ======================================================================== */

/* 编译期守卫：上下文必须塞进调用者提供的存储。
 *
 * ⚠ 若这里编译失败，说明结构体长大了而 `JSDK_CONTEXT_MAX_SIZE`（公共头）
 *   没同步 —— 那会造成**调用者提供的静态存储被静默写溢出**，是最危险的一类
 *   回归。修法只有两个：缩小结构体，或按实测值上调公共头里的常量。
 *
 * （用“数组大小为负”这一 C99 惯用法，不依赖 C11 的 _Static_assert。） */
typedef char jsdk_ctx_size_ok[(sizeof(jsdk_context_t) <= JSDK_CONTEXT_MAX_SIZE) ? 1 : -1];
typedef char jsdk_joint_size_ok[
    (sizeof(jsdk_joint_t) * JSDK_MAX_JOINTS + 256u <= JSDK_CONTEXT_MAX_SIZE) ? 1 : -1];

/* ==========================================================================
 * 魔数校验
 * ======================================================================== */

int jsdk_ctx_check(const jsdk_context_t *ctx)
{
    return (ctx && ctx->magic == JSDK_CTX_MAGIC) ? 1 : 0;
}

int jsdk_joint_check(const jsdk_joint_t *j)
{
    if (!j || j->magic != JSDK_JOINT_MAGIC) return 0;
    return jsdk_ctx_check(j->ctx);
}

/* ==========================================================================
 * 错误串
 * ------------------------------------------------------------------------
 * 只用 %s / %u / %d；浮点走 jsdk_fmt_f()，避免依赖 libc 的浮点 printf。
 * 永远 NUL 结尾；不是 RT 关键路径的日志，但 cycle 内也会被调用
 * （越界拒绝），因此**不分配、不阻塞**。
 * ======================================================================== */

void jsdk_ctx_seterr(jsdk_context_t *ctx, const char *fmt, ...)
{
    va_list ap;

    if (!ctx) return;

    va_start(ap, fmt);
    (void)vsnprintf(ctx->last_error, sizeof ctx->last_error, fmt, ap);
    va_end(ap);
    ctx->last_error[sizeof ctx->last_error - 1u] = '\0';
}

void jsdk_joint_seterr(jsdk_joint_t *j, const char *fmt, ...)
{
    va_list ap;
    size_t  n;

    if (!j || !j->ctx) return;
    n = (size_t)snprintf(j->ctx->last_error, sizeof j->ctx->last_error,
                         "joint %u(node %u): ", (unsigned)j->index,
                         (unsigned)j->cfg.node_id);
    if (n >= sizeof j->ctx->last_error) n = sizeof j->ctx->last_error - 1u;

    va_start(ap, fmt);
    (void)vsnprintf(j->ctx->last_error + n, sizeof j->ctx->last_error - n, fmt, ap);
    va_end(ap);
    j->ctx->last_error[sizeof j->ctx->last_error - 1u] = '\0';
}

/* ==========================================================================
 * 配置默认值
 * ======================================================================== */

void jsdk_context_config_default(jsdk_context_config_t *cfg)
{
    if (!cfg) return;

    memset(cfg, 0, sizeof *cfg);
    /* hal 全 0（调用者必须填 send/recv/now_ms） */
    cfg->master_id              = CB_DEFAULT_MASTER_ID;
    cfg->is_fd                  = 1u;
    cfg->period_ns              = 0u;
    cfg->auto_keepalive         = 1u;
    cfg->clamp_target_position  = 0u;   /* 默认：拒绝并改发安全帧（§6.10） */
    cfg->enable_watchdog_hint   = 0u;   /* 默认：不擅自改客户设备配置 */
    cfg->max_joints             = 0u;
    cfg->rx_burst_limit         = 0u;

    cfg->desc.mode                = JSDK_DESC_DYNAMIC;
    cfg->desc.retain              = JSDK_DESC_RETAIN_ALL;
    cfg->desc.share_by_crc        = 1u;
    cfg->desc.stop_when_satisfied = 1u;
    cfg->desc.timeout_ms          = 0u;
}

/* ==========================================================================
 * 尺寸查询
 * ======================================================================== */

size_t jsdk_context_size(const jsdk_context_config_t *cfg)
{
    (void)cfg;                    /* 容量固定（关节内嵌），与配置无关 */
    return sizeof(jsdk_context_t);
}

/* 注：jsdk_desc_arena_size() 已在 src/proto_cyberbeast/cb_jsondesc_parse.c 实现
   （它按 filter 字符串长度估算，比在上下文层重复一份更准）。 */

/* ==========================================================================
 * 生命周期
 * ======================================================================== */

jsdk_status_t jsdk_context_init(jsdk_context_t *ctx, const jsdk_context_config_t *cfg)
{
    if (!ctx || !cfg) return JSDK_ERR_INVALID_ARG;

    /* ABI 守卫（DESIGN §9「ABI 守卫」验收项）：
         magic == 0        → 全新存储（静态零初始化），正常
         magic == 本后端   → 已初始化，重复 init 属调用方 bug
         其它任意值        → 极可能是**另一个后端**的存储，必须拒绝 */
    if (ctx->magic != 0u && ctx->magic != JSDK_CTX_MAGIC) {
        return JSDK_ERR_INVALID_ARG;
    }
    if (ctx->magic == JSDK_CTX_MAGIC) {
        return JSDK_ERR_BAD_STATE;
    }

    if (!cfg->hal.send || !cfg->hal.recv || !cfg->hal.now_ms) {
        return JSDK_ERR_INVALID_ARG;
    }
    if (cfg->master_id == 0u) {
        /* 0 会让设备**完全不回复**（见 cb_frame.h）——静默失效，必须早拦 */
        return JSDK_ERR_INVALID_ARG;
    }
    if (cfg->is_fd > 1u) return JSDK_ERR_INVALID_ARG;

    if (!cfg->desc.arena || cfg->desc.arena_size == 0u) {
        /* 零 malloc 由调用者提供解析区；没有它连端点都解析不了 */
        return JSDK_ERR_INVALID_ARG;
    }
    if (cfg->desc.filter_count > JSDK_DESC_MAX_FILTERS) {
        return JSDK_ERR_INVALID_ARG;
    }

    /* 平台前提：float 必须是 IEEE-754 单精度，且字节序访问可用 */
    if (cb_frame_selfcheck() != 0) return JSDK_ERR_UNSUPPORTED;

    memset(ctx, 0, sizeof *ctx);
    ctx->cfg = *cfg;                       /* hal 一并拷贝；调用者之后可丢弃 */
    ctx->cfg.magic = JSDK_CTX_MAGIC;
    ctx->nj      = 0u;
    ctx->tx_seq  = 0u;
    ctx->now_ms  = cfg->hal.now_ms(cfg->hal.user);

    /* `desc.arena` 指向调用者的内存，arena_used 也要写回调用者（不能只写副本）。
       生命周期要求：配置结构体与 arena 同寿命（见 jsdk_core_internal.h）。 */
    ctx->arena_used_slot = &((jsdk_context_config_t *)cfg)->desc.arena_used;

    jsdk_ctx_seterr(ctx, "not configured");
    ctx->magic = JSDK_CTX_MAGIC;
    return JSDK_OK;
}

jsdk_status_t jsdk_context_add_joint(jsdk_context_t *ctx,
                                     const jsdk_joint_config_t *jc,
                                     jsdk_joint_t **out_joint)
{
    unsigned cap, i;
    jsdk_joint_t *j;

    if (out_joint) *out_joint = NULL;
    if (!jsdk_ctx_check(ctx) || !jc) return JSDK_ERR_INVALID_ARG;

    /* 关节只能在 configure() 之前加入：configure 会按 nj 逐个标定 */
    if (ctx->desc_present || ctx->fetch_active) return JSDK_ERR_BAD_STATE;

    cap = ctx->cfg.max_joints ? ctx->cfg.max_joints : JSDK_MAX_JOINTS;
    if (cap > JSDK_MAX_JOINTS) cap = JSDK_MAX_JOINTS;
    if (ctx->nj >= cap) return JSDK_ERR_NO_MEMORY;

    if (jc->node_id < CB_NODE_ID_MIN || jc->node_id > CB_NODE_ID_MAX) {
        return JSDK_ERR_INVALID_ARG;
    }
    if (jc->axis != 0u) {
        /* 固件 AXIS_COUNT = 1；提前拒绝优于静默忽略 */
        return JSDK_ERR_UNSUPPORTED;
    }
    for (i = 0u; i < ctx->nj; ++i) {
        if (ctx->joints[i].cfg.node_id == jc->node_id) {
            return JSDK_ERR_INVALID_ARG;   /* 同总线不允许重复 node_id */
        }
    }

    j = &ctx->joints[ctx->nj];
    memset(j, 0, sizeof *j);
    j->ctx   = ctx;
    j->index = (uint8_t)ctx->nj;
    j->cfg   = *jc;
    j->cfg.magic = JSDK_JOINT_MAGIC;
    j->mode  = jc->initial_mode;
    j->enable_mode = jc->initial_mode;
    j->status_flags = (uint16_t)JSDK_JF_SCALE_INVALID;   /* 标定前禁止物理量 API */
    j->magic = JSDK_JOINT_MAGIC;

    ctx->nj++;
    if (out_joint) *out_joint = j;
    return JSDK_OK;
}

jsdk_joint_t *jsdk_ctx_find_joint(jsdk_context_t *ctx, uint8_t node_id)
{
    unsigned i;

    if (!jsdk_ctx_check(ctx)) return NULL;
    for (i = 0u; i < ctx->nj; ++i) {
        if (ctx->joints[i].cfg.node_id == node_id) return &ctx->joints[i];
    }
    return NULL;
}

int jsdk_ctx_any_enabled(const jsdk_context_t *ctx)
{
    unsigned i;

    if (!jsdk_ctx_check(ctx)) return 0;
    for (i = 0u; i < ctx->nj; ++i) {
        if (ctx->joints[i].enabled) return 1;
    }
    return 0;
}

void jsdk_context_destroy(jsdk_context_t *ctx)
{
    unsigned i;

    if (!jsdk_ctx_check(ctx)) return;

    for (i = 0u; i < ctx->nj; ++i) {
        ctx->joints[i].magic = 0u;      /* 使残留句柄立即失效 */
    }
    ctx->nj = 0u;
    ctx->magic = 0u;                    /* 最后清魔数：之后 init 可复用该存储 */
}

void jsdk_context_set_fault_callback(jsdk_context_t *ctx,
                                     jsdk_fault_callback_t cb, void *user)
{
    if (!jsdk_ctx_check(ctx)) return;
    ctx->fault_cb   = cb;
    ctx->fault_user = user;
}

/**
 * 把 arena 实际用量回写到调用方的 `desc.arena_used`。
 *
 * 公共头把它声明为**输出**，而 `jsdk_context_init()` 会复制配置，
 * 所以必须经 `arena_used_slot` 直接写回（见结构体里的说明）。
 */
void jsdk_ctx_publish_arena_used(jsdk_context_t *ctx)
{
    size_t used;

    if (!jsdk_ctx_check(ctx)) return;

    used = (size_t)ctx->store.arena.entry_count * sizeof(jsdk_ep_entry_t)
         + ctx->store.arena.blob_used;
    ctx->cfg.desc.arena_used = used;
    if (ctx->arena_used_slot) *ctx->arena_used_slot = used;
}

/* ==========================================================================
 * 发送
 * ======================================================================== */

int jsdk_ctx_send(jsdk_context_t *ctx, uint8_t pri, uint8_t msgtype,
                  uint8_t dest, const uint8_t *payload, uint8_t len)
{
    jsdk_can_frame_t f;
    int rc;

    if (!jsdk_ctx_check(ctx)) return -1;
    if (len > 64u) return -1;

    memset(&f, 0, sizeof f);
    f.id    = cb_make_id(pri, msgtype, dest, ctx->cfg.master_id, ctx->tx_seq);
    f.len   = len;
    f.flags = (uint8_t)(JSDK_FRAME_EXT | (ctx->cfg.is_fd ? (JSDK_FRAME_FD | JSDK_FRAME_BRS) : 0u));
    if (len > 0u && payload) memcpy(f.data, payload, len);

    ctx->bus.tx_frames++;
    ctx->now_ms = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);   /* 发送前刷新时钟 */

    rc = ctx->cfg.hal.send(ctx->cfg.hal.user, &f);
    if (rc != 0) ctx->bus.tx_failed++;
    return rc;
}

/**
 * 编译期探针：把上下文/关节的真实大小暴露给测试与工具链诊断。
 * （`jsdk_context_size()` 只是个运行时包装，这里给出各部分的分解。）
 */
size_t jsdk_context_joint_size(void) { return sizeof(jsdk_joint_t); }

/* ==========================================================================
 * 接收解复用
 * ======================================================================== */

/**
 * 处理一帧设备→主站的帧。
 *
 * @return 1 = 已消费；0 = 与本主站无关（丢弃）
 */
int jsdk_ctx_handle_frame(jsdk_context_t *ctx, const jsdk_can_frame_t *f)
{
    uint8_t msgtype = (uint8_t)cb_id_msgtype(f->id);
    uint8_t src     = (uint8_t)cb_id_source(f->id);
    uint8_t dst     = (uint8_t)cb_id_dest(f->id);
    jsdk_joint_t *j;

    if (!jsdk_ctx_check(ctx) || !f) return 0;

    ctx->bus.rx_frames++;
    ctx->bus.last_rx_age_ms = 0u;
    ctx->bus.link_up = 1u;
    ctx->last_rx_ms  = ctx->now_ms;

    /* 寻址：设备只会以 dest = master_id 单播回复；广播告警除外 */
    if (dst != ctx->cfg.master_id && !(cb_id_is_broadcast(f->id) && dst == CB_ADDR_BROADCAST)) {
        ctx->bus.rx_dropped++;
        return 0;
    }

    j = jsdk_ctx_find_joint(ctx, src);

    switch (msgtype) {
    /* ---- 所有控制帧的应答都是 MIT 响应格式（0x00） ---- */
    case CB_MSG_MIT_CONTROL:
        if (!j) break;
        jsdk_joint__on_mit_response(j, f->data, f->len);
        return 1;

    /* ---- 心跳 ---- */
    case CB_MSG_HEARTBEAT:
        if (!j) break;
        jsdk_joint__on_heartbeat(j, f->data, f->len);
        return 1;

    /* ---- 查询响应（配置阶段由 wait_response 消费；此处尽力更新反馈） ---- */
    case CB_MSG_QUERY_POS_VEL: {
        cb_query_pos_vel_t pv;
        if (!j) break;
        if (cb_query_decode_pos_vel(f->data, f->len, &pv) != 0) break;
        jsdk_joint__on_pos_vel_turns(j, pv.pos_turns, pv.vel_turns_per_s);
        return 1;
    }
    case CB_MSG_QUERY_CURRENT: {
        cb_query_current_t c;
        if (!j) break;
        if (cb_query_decode_current(f->data, f->len, &c) != 0) break;
        jsdk_joint__on_current_a(j, (double)c.iq_a);
        return 1;
    }
    case CB_MSG_QUERY_TEMPERATURE: {
        cb_query_temp_t t;
        if (!j) break;
        if (cb_query_decode_temp(f->data, f->len, &t) != 0) break;
        jsdk_joint__on_temps(j, (double)t.motor_c, (double)t.fet_c);
        return 1;
    }
    case CB_MSG_QUERY_BUS: {
        cb_query_bus_t bu;
        if (!j) break;
        if (cb_query_decode_bus(f->data, f->len, &bu) != 0) break;
        jsdk_joint__on_bus_volts(j, (double)bu.vbus_v, (double)bu.ibus_a);
        return 1;
    }
    case CB_MSG_FAULT_ALERT:
        /* 设备主动告警广播：只置位，不做业务处理（细节靠 QUERY_ERROR） */
        if (j) jsdk_joint__on_fault_alert(j);
        return 1;

    default:
        break;
    }

    /* 0x20/0x45/0x46 等由等待者消费；走到这里说明没人要它 */
    ctx->bus.rx_dropped++;
    return 0;
}

static int jsdk_ctx_pump_rx(jsdk_context_t *ctx, unsigned limit)
{
    jsdk_can_frame_t f;
    unsigned i;

    for (i = 0u; i < limit; ++i) {
        int n = ctx->cfg.hal.recv(ctx->cfg.hal.user, &f);
        if (n == 0) return 0;
        if (n < 0) {
            /* WP4 链路健康：recv 报错就认为链路不可用，直到下一次成功收到帧。
               不断言“永久 down”—— 总线抖动会自行恢复。 */
            ctx->bus.link_errors++;
            ctx->bus.link_up = 0u;
            return -1;
        }
        (void)jsdk_ctx_handle_frame(ctx, &f);
    }
    return 1;                 /* 达到上限：本周期还有帧没处理完 */
}

/**
 * 阻塞等待某条响应（配置阶段专用）。
 *
 * @par 为什么还要一个自旋上限
 *  虚拟/仿真 HAL 的 `now_ms` 只在测试推进时间轴时变化，`recv` 也可能长期返回 0。
 *  若只靠时间差判断超时，会在这个环境下**死循环**。因此除了时间预算，再加一个
 *  自旋上限兜底（真实 HAL 下时间预算先生效）。
 */
int jsdk_ctx_wait_response(jsdk_context_t *ctx, uint8_t msgtype, uint8_t source,
                           jsdk_can_frame_t *out, uint32_t timeout_ms)
{
    uint32_t t0;
    unsigned spin = 0u;
    const unsigned spin_cap = 200000u;

    if (!jsdk_ctx_check(ctx)) return JSDK_ERR_INVALID_ARG;

    t0 = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);

    for (;;) {
        jsdk_can_frame_t f;
        int n = ctx->cfg.hal.recv(ctx->cfg.hal.user, &f);

        if (n < 0) return JSDK_ERR_TRANSPORT;

        if (n > 0) {
            ctx->bus.rx_frames++;
            ctx->bus.link_up = 1u;
            ctx->last_rx_ms  = ctx->now_ms;
            if (cb_id_msgtype(f.id) == msgtype && cb_id_source(f.id) == source) {
                if (out) *out = f;
                return JSDK_OK;
            }
            (void)jsdk_ctx_handle_frame(ctx, &f);
            continue;
        }

        if (jsdk_elapsed(ctx->cfg.hal.now_ms(ctx->cfg.hal.user), t0) >= timeout_ms) {
            return JSDK_ERR_TIMEOUT;
        }
        if (++spin >= spin_cap) return JSDK_ERR_TIMEOUT;
    }
}

/* ==========================================================================
 * 循环边界
 * ======================================================================== */

jsdk_status_t jsdk_context_cycle_begin(jsdk_context_t *ctx, uint64_t app_time_ns)
{
    unsigned i, limit;
    int rc;

    if (!jsdk_ctx_check(ctx)) return JSDK_ERR_INVALID_ARG;

    (void)app_time_ns;      /* 预留：周期抖动统计（P1） */

    ctx->now_ms = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);
    ctx->in_cycle = 1u;

    for (i = 0u; i < ctx->nj; ++i) {
        ctx->joints[i].fb.valid = 0;      /* 本周期尚无新数据 */
        ctx->joints[i].sent_cycle = 0u;   /* 本周期还没发过控制帧 */
        jsdk_joint__refresh_freshness(&ctx->joints[i]);   /* WP4：age_ms + STALE 位 */
    }

    ctx->bus.last_rx_age_ms = jsdk_elapsed(ctx->now_ms, ctx->last_rx_ms);

    limit = ctx->cfg.rx_burst_limit ? ctx->cfg.rx_burst_limit : JSDK_RX_BURST_DEFAULT;
    rc = jsdk_ctx_pump_rx(ctx, limit);
    if (rc < 0) {
        jsdk_ctx_seterr(ctx, "HAL recv() reported a link error");
        return JSDK_ERR_TRANSPORT;
    }
    return JSDK_OK;
}

jsdk_status_t jsdk_context_cycle_end(jsdk_context_t *ctx)
{
    if (!jsdk_ctx_check(ctx)) return JSDK_ERR_INVALID_ARG;

    ctx->now_ms = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);

    jsdk_joint__cycle_end_all(ctx);
    jsdk_watchdog__cycle_end(ctx);

    ctx->in_cycle = 0u;
    return JSDK_OK;
}

jsdk_status_t jsdk_context_poll(jsdk_context_t *ctx, uint64_t app_time_ns)
{
    jsdk_status_t st = jsdk_context_cycle_begin(ctx, app_time_ns);
    if (st != JSDK_OK) return st;
    return jsdk_context_cycle_end(ctx);
}

/* ==========================================================================
 * 总线状态 / 错误 / ESTOP
 * ======================================================================== */

jsdk_status_t jsdk_context_get_bus_state(jsdk_context_t *ctx, jsdk_bus_state_t *state)
{
    uint8_t online = 0u;
    unsigned i;

    if (!jsdk_ctx_check(ctx) || !state) return JSDK_ERR_INVALID_ARG;

    for (i = 0u; i < ctx->nj; ++i) {
        if (ctx->joints[i].fb.online) online++;
    }

    *state = ctx->bus;
    state->nodes_online = online;
    state->last_rx_age_ms = jsdk_elapsed(ctx->now_ms, ctx->last_rx_ms);
    if (ctx->cfg.hal.bus_status) {
        uint32_t flags = 0u;
        if (ctx->cfg.hal.bus_status(ctx->cfg.hal.user, &flags) == 0) {
            state->hal_bus_flags = flags;
        }
    }
    return JSDK_OK;
}

const char *jsdk_context_last_error(jsdk_context_t *ctx)
{
    if (!jsdk_ctx_check(ctx)) return "invalid context";
    return ctx->last_error;
}

void jsdk_context_estop(jsdk_context_t *ctx)
{
    if (!jsdk_ctx_check(ctx)) return;
    /* MsgType 0xC0 是全局广播（Dest = 0xFF），载荷被固件忽略 */
    (void)jsdk_ctx_send(ctx, CB_PRI_CRITICAL, CB_MSG_ESTOP, CB_ADDR_BROADCAST, NULL, 0u);
    ctx->tx_seq = cb_seq_next(ctx->tx_seq);
    jsdk_ctx_seterr(ctx, "ESTOP broadcast sent");
}

/* ======================== src/core/jsdk_desc.c ======================== */
/**
 * @file    jsdk_desc.c
 * @brief   描述符集成：0x24/0x25 收发、节点间共享（share_by_crc）、缓存三条出口
 *
 * 单帧级传输状态机在 `src/proto_cyberbeast/cb_jsondesc_fetch.c`（已独立测试），
 * 本文件只负责"把它接到上下文上"：谁来发请求、排空 RX、何时算超时、
 * 以及从哪个节点下载、能不能复用别的节点已经下好的结果。
 */



/* ==========================================================================
 * 内部工具
 * ======================================================================== */

/** 单帧能装多少描述符字节（帧长 − 2 字节头）。 */
static uint32_t desc_payload_per_frame(int is_fd)
{
    return (uint32_t)((is_fd ? CB_DESC_FRAME_LEN_FD : CB_DESC_FRAME_LEN_CLASSIC)
                      - CB_DESC_DATA_HDR_BYTES);
}

/** 完整描述符需要多少帧（1 个元数据帧 + N 个数据帧）。 */
static uint32_t desc_frames_needed(uint32_t total_len, int is_fd)
{
    uint32_t ppf = desc_payload_per_frame(is_fd);
    if (ppf == 0u) return 0u;
    return 1u + (total_len + ppf - 1u) / ppf;
}

static void store_init(jsdk_context_t *ctx)
{
    memset(&ctx->store, 0, sizeof ctx->store);
    ctx->store.arena.base     = (uint8_t *)ctx->cfg.desc.arena;
    ctx->store.arena.size     = ctx->cfg.desc.arena_size;
    ctx->store.arena.blob_top = ctx->cfg.desc.arena_size;
    ctx->store.max_endpoints  = ctx->cfg.desc.max_endpoints
                                ? ctx->cfg.desc.max_endpoints : 2048u;
}

/** 至少有一个关节已被使能 → 禁止下载（DESIGN §6.6 硬约束 1）。 */

/**
 * 从 @p node 完整下载并解析描述符（**阻塞**）。
 *
 * 排空 RX 时**不受** `rx_burst_limit` 限制（硬约束 2）：662 帧连续到达，
 * 按默认 32 帧/周期处理会丢帧导致 JSON 断裂。
 */
static jsdk_status_t desc_fetch_from(jsdk_context_t *ctx, uint8_t node)
{
    uint8_t req[CB_DESC_REQ_LEN];
    uint32_t deadline;
    size_t   req_len;
    unsigned spin = 0u;
    const unsigned spin_cap = 4000000u;

    store_init(ctx);

    if (cb_desc_fetch_init(&ctx->fetch, ctx, &ctx->cfg.desc, &ctx->store) != JSDK_OK) {
        return JSDK_ERR_INVALID_ARG;
    }
    ctx->fetch_active = 1;
    cb_desc_fetch_set_raw_sink(&ctx->fetch, ctx->raw_sink, ctx->raw_sink_user);
    cb_desc_fetch_set_progress(&ctx->fetch, ctx->progress, ctx->progress_user);

    req_len = cb_desc_build_request(req, sizeof req, 0u);
    if (req_len == 0u) return JSDK_ERR_INVALID_ARG;

    if (jsdk_ctx_send(ctx, CB_PRI_CONFIG, CB_MSG_JSON_DESC_READ, node,
                      req, (uint8_t)req_len) != 0) {
        jsdk_ctx_seterr(ctx, "descriptor request to node %u could not be sent",
                        (unsigned)node);
        return JSDK_ERR_TRANSPORT;
    }
    ctx->tx_seq = cb_seq_next(ctx->tx_seq);

    deadline = cb_desc_timeout_ms(&ctx->cfg.desc);
    {
        uint32_t t0 = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);

        while (!cb_desc_fetch_is_done(&ctx->fetch)) {
            jsdk_can_frame_t f;
            int n = ctx->cfg.hal.recv(ctx->cfg.hal.user, &f);

            if (n < 0) return JSDK_ERR_TRANSPORT;
            if (n > 0) {
                ctx->bus.rx_frames++;
                ctx->last_rx_ms = ctx->now_ms;
                ctx->bus.link_up = 1u;
                if (cb_id_msgtype(f.id) == CB_MSG_JSON_DESC_DATA
                    && cb_id_source(f.id) == node) {
                    (void)cb_desc_fetch_frame(&ctx->fetch, f.data, f.len);
                } else {
                    (void)jsdk_ctx_handle_frame(ctx, &f);
                }
                continue;
            }

            ctx->now_ms = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);
            if (jsdk_elapsed(ctx->now_ms, t0) >= deadline) {
                jsdk_ctx_seterr(ctx, "descriptor download from node %u timed out "
                                     "after %u ms (%u/%u bytes)",
                                (unsigned)node, (unsigned)deadline,
                                (unsigned)ctx->fetch.bytes_scanned,
                                (unsigned)ctx->fetch.total_len);
                return JSDK_ERR_TIMEOUT;
            }
            if (++spin >= spin_cap) {
                jsdk_ctx_seterr(ctx, "descriptor download from node %u stalled "
                                     "(no frames; frozen clock?)", (unsigned)node);
                return JSDK_ERR_TIMEOUT;
            }
        }
    }

    /* 把传输结果搬进 desc_info */
    {
        cb_desc_fetch_result_t r;
        cb_desc_fetch_result(&ctx->fetch, &r);

        ctx->desc.total_len     = r.total_len;
        ctx->desc.crc           = r.crc;
        ctx->desc.endpoint_count = r.endpoint_count;
        ctx->desc.parsed_total  = r.parsed_total;
        ctx->desc.frames_rx     = r.frames_rx;
        ctx->desc.bytes_scanned = r.bytes_scanned;
        ctx->desc.complete      = r.complete;
        ctx->desc.raw_sink_failed = r.raw_sink_failed;
        ctx->cfg.desc.arena_used = r.arena_used;
    }

    if (cb_desc_fetch_error(&ctx->fetch) != CB_DESC_ERR_NONE) {
        jsdk_ctx_seterr(ctx, "descriptor from node %u rejected: %s (%s)",
                        (unsigned)node, cb_desc_fetch_error(&ctx->fetch),
                        cb_desc_fetch_detail(&ctx->fetch)
                            ? cb_desc_fetch_detail(&ctx->fetch) : "-");
        return (strcmp(cb_desc_fetch_error(&ctx->fetch), CB_DESC_ERR_ARENA) == 0)
                   ? JSDK_ERR_NO_MEMORY : JSDK_ERR_PARSE;
    }
    return JSDK_OK;
}

/**
 * 只读一个节点的描述符**元数据帧**（total_len / crc），随后排空其余帧。
 *
 * 用途：`share_by_crc`。设备不提供"只问 CRC"的接口，但元数据帧恒定是每次请求的
 * 第一帧，所以代价 = 1 次请求 + 1 次整份排空（不解析）。
 *
 * @note 必须在 RX 队列里没有残留 0x25 帧时调用（`desc_fetch_from` 会排空）。
 */
static jsdk_status_t desc_probe_meta(jsdk_context_t *ctx, uint8_t node,
                                     uint16_t *out_crc, uint32_t *out_total)
{
    uint8_t req[CB_DESC_REQ_LEN];
    uint32_t t0, deadline;
    uint32_t seen = 0u;
    unsigned spin = 0u;
    const unsigned spin_cap = 4000000u;
    size_t   req_len;
    uint32_t total = 0u;
    uint16_t crc   = 0u;

    req_len = cb_desc_build_request(req, sizeof req, 0u);
    if (req_len == 0u) return JSDK_ERR_INVALID_ARG;
    if (jsdk_ctx_send(ctx, CB_PRI_CONFIG, CB_MSG_JSON_DESC_READ, node,
                      req, (uint8_t)req_len) != 0) {
        return JSDK_ERR_TRANSPORT;
    }
    ctx->tx_seq = cb_seq_next(ctx->tx_seq);

    deadline = cb_desc_timeout_ms(&ctx->cfg.desc);
    t0 = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);

    for (;;) {
        jsdk_can_frame_t f;
        int n = ctx->cfg.hal.recv(ctx->cfg.hal.user, &f);

        if (n < 0) return JSDK_ERR_TRANSPORT;
        if (n > 0) {
            ctx->bus.rx_frames++;
            ctx->last_rx_ms = ctx->now_ms;
            if (cb_id_msgtype(f.id) == CB_MSG_JSON_DESC_DATA
                && cb_id_source(f.id) == node) {
                seen++;
                if (seen == 1u) {
                    /* 元数据帧：[0..1]=0 + totalLen u32 LE @2 + crc u16 LE @6 */
                    if (f.len < CB_DESC_META_MIN_LEN
                        || f.data[0] != 0u || f.data[1] != 0u) {
                        return JSDK_ERR_PROTOCOL;
                    }
                    total = cb_le_get_u32(f.data + 2);
                    crc   = cb_le_get_u16(f.data + 6);
                    if (out_crc)   *out_crc   = crc;
                    if (out_total) *out_total = total;
                }
                if (seen >= desc_frames_needed(total != 0u ? total : 1u,
                                               ctx->cfg.is_fd)) {
                    return JSDK_OK;      /* 排空完毕 */
                }
            } else {
                (void)jsdk_ctx_handle_frame(ctx, &f);
            }
            continue;
        }

        ctx->now_ms = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);
        if (jsdk_elapsed(ctx->now_ms, t0) >= deadline) return JSDK_ERR_TIMEOUT;
        if (++spin >= spin_cap) return JSDK_ERR_TIMEOUT;
    }
}

/* ==========================================================================
 * 公共：下载
 * ======================================================================== */

jsdk_status_t jsdk_context_desc_fetch(jsdk_context_t *ctx)
{
    uint8_t node;
    jsdk_status_t st;

    if (!jsdk_ctx_check(ctx)) return JSDK_ERR_INVALID_ARG;
    if (ctx->nj == 0u) return JSDK_ERR_BAD_STATE;
    if (jsdk_ctx_any_enabled(ctx)) {
        jsdk_ctx_seterr(ctx, "descriptor download refused: a joint is enabled "
                             "(JSON frames would starve the control frames)");
        return JSDK_ERR_BAD_STATE;
    }

    node = ctx->joints[0].cfg.node_id;
    st = desc_fetch_from(ctx, node);
    if (st != JSDK_OK) {
        ctx->desc_present = 0u;
        ctx->fetch_active = 0;      /* 传输已死：别让 desc_poll 继续推进它 */
        return st;
    }

    ctx->desc_present  = 1u;
    ctx->desc.mode_used = ctx->cfg.desc.mode;
    jsdk_ctx_publish_arena_used(ctx);

    /* 设备信息（fw/hw）同时是缓存与共享的键 */
    {
        jsdk_device_info_t info;
        if (jsdk_joint_get_device_info(&ctx->joints[0], &info) == JSDK_OK) {
            ctx->desc.fw_version = info.fw_version;
            ctx->desc.hw_version = info.hw_version;
        }
    }

    /* --- 其余节点：CRC 相同则复用（share_by_crc） --- */
    if (ctx->cfg.desc.share_by_crc) {
        unsigned i;
        for (i = 1u; i < ctx->nj; ++i) {
            jsdk_joint_t *j = &ctx->joints[i];
            uint16_t crc = 0u;
            uint32_t total = 0u;

            if (desc_probe_meta(ctx, j->cfg.node_id, &crc, &total) == JSDK_OK
                && crc == ctx->desc.crc) {
                j->shared_desc = 1u;
                continue;
            }
            /* 不一致（或探测失败）→ 老老实实重下一份，以最后一个为准 */
            {
                jsdk_status_t s2 = desc_fetch_from(ctx, j->cfg.node_id);
                if (s2 != JSDK_OK) {
                    jsdk_ctx_seterr(ctx, "descriptor mismatch on node %u and "
                                         "re-download failed",
                                    (unsigned)j->cfg.node_id);
                    return s2;
                }
            }
        }
        ctx->desc.shared_hit = (ctx->joints[0].shared_desc
                                || (ctx->nj > 1u && ctx->joints[1].shared_desc))
                               ? 1u : 0u;
    }

    if (ctx->desc.complete == 0u) {
        /* 提前终止（精确 filter）→ 端点表本身可用，但原始字节不完整 */
        jsdk_ctx_seterr(ctx, "descriptor scanned %u of %u bytes (early stop; "
                             "raw caching is NOT safe for this run)",
                        (unsigned)ctx->desc.bytes_scanned, (unsigned)ctx->desc.total_len);
    } else {
        jsdk_ctx_seterr(ctx, "descriptor ok: %u endpoints, %u bytes, crc=0x%04X",
                        ctx->desc.endpoint_count, (unsigned)ctx->desc.total_len,
                        (unsigned)ctx->desc.crc);
    }
    return JSDK_OK;
}

jsdk_status_t jsdk_context_desc_poll(jsdk_context_t *ctx, uint64_t app_time_ns)
{
    (void)app_time_ns;

    if (!jsdk_ctx_check(ctx)) return JSDK_ERR_INVALID_ARG;
    if (ctx->nj == 0u) return JSDK_ERR_BAD_STATE;

    if (!ctx->fetch_active) {
        uint8_t req[CB_DESC_REQ_LEN];
        size_t  req_len;
        uint8_t node = ctx->joints[0].cfg.node_id;

        if (ctx->desc_present) return JSDK_OK;
        if (jsdk_ctx_any_enabled(ctx)) return JSDK_ERR_BAD_STATE;

        store_init(ctx);
        if (cb_desc_fetch_init(&ctx->fetch, ctx, &ctx->cfg.desc, &ctx->store) != JSDK_OK) {
            return JSDK_ERR_INVALID_ARG;
        }
        cb_desc_fetch_set_raw_sink(&ctx->fetch, ctx->raw_sink, ctx->raw_sink_user);
        cb_desc_fetch_set_progress(&ctx->fetch, ctx->progress, ctx->progress_user);
        ctx->fetch_active = 1;

        req_len = cb_desc_build_request(req, sizeof req, 0u);
        if (req_len == 0u) return JSDK_ERR_INVALID_ARG;
        if (jsdk_ctx_send(ctx, CB_PRI_CONFIG, CB_MSG_JSON_DESC_READ, node,
                          req, (uint8_t)req_len) != 0) {
            ctx->fetch_active = 0;
            return JSDK_ERR_TRANSPORT;
        }
        ctx->tx_seq = cb_seq_next(ctx->tx_seq);
        ctx->fetch_deadline_ms = ctx->cfg.hal.now_ms(ctx->cfg.hal.user)
                               + cb_desc_timeout_ms(&ctx->cfg.desc);
        return JSDK_ERR_BUSY;
    }

    /* 推进：每周期最多处理 64 帧（非阻塞路径，调用者自己控制节奏） */
    for (unsigned i = 0u; i < 64u && !cb_desc_fetch_is_done(&ctx->fetch); ++i) {
        jsdk_can_frame_t f;
        int n = ctx->cfg.hal.recv(ctx->cfg.hal.user, &f);
        if (n <= 0) break;
        ctx->bus.rx_frames++;
        ctx->last_rx_ms = ctx->now_ms;
        if (cb_id_msgtype(f.id) == CB_MSG_JSON_DESC_DATA
            && cb_id_source(f.id) == ctx->joints[0].cfg.node_id) {
            (void)cb_desc_fetch_frame(&ctx->fetch, f.data, f.len);
        } else {
            (void)jsdk_ctx_handle_frame(ctx, &f);
        }
    }

    ctx->now_ms = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);
    if (!cb_desc_fetch_is_done(&ctx->fetch)) {
        if ((int32_t)(ctx->now_ms - ctx->fetch_deadline_ms) >= 0) {
            ctx->fetch_active = 0;
            jsdk_ctx_seterr(ctx, "descriptor poll timed out");
            return JSDK_ERR_TIMEOUT;
        }
        return JSDK_ERR_BUSY;
    }

    ctx->fetch_active = 0;
    {
        cb_desc_fetch_result_t r;
        cb_desc_fetch_result(&ctx->fetch, &r);
        ctx->desc.total_len      = r.total_len;
        ctx->desc.crc            = r.crc;
        ctx->desc.endpoint_count = r.endpoint_count;
        ctx->desc.parsed_total   = r.parsed_total;
        ctx->desc.frames_rx      = r.frames_rx;
        ctx->desc.bytes_scanned  = r.bytes_scanned;
        ctx->desc.complete       = r.complete;
        ctx->desc.raw_sink_failed= r.raw_sink_failed;
        ctx->desc.mode_used      = ctx->cfg.desc.mode;
        ctx->cfg.desc.arena_used = r.arena_used;
    }
    if (cb_desc_fetch_error(&ctx->fetch) != CB_DESC_ERR_NONE) {
        ctx->desc_present = 0u;
        return (strcmp(cb_desc_fetch_error(&ctx->fetch), CB_DESC_ERR_ARENA) == 0)
                   ? JSDK_ERR_NO_MEMORY : JSDK_ERR_PARSE;
    }
    ctx->desc_present = 1u;
    jsdk_ctx_publish_arena_used(ctx);
    return JSDK_OK;
}

/* ==========================================================================
 * 公共：进度 / tee / 元信息
 * ======================================================================== */

void jsdk_context_set_desc_progress(jsdk_context_t *ctx,
                                    jsdk_desc_progress_fn fn, void *user)
{
    if (!jsdk_ctx_check(ctx)) return;
    ctx->progress = fn;
    ctx->progress_user = user;
}

void jsdk_context_set_desc_raw_sink(jsdk_context_t *ctx,
                                    jsdk_desc_raw_sink_fn fn, void *user)
{
    if (!jsdk_ctx_check(ctx)) return;
    ctx->raw_sink = fn;
    ctx->raw_sink_user = user;
}

jsdk_status_t jsdk_context_get_desc_info(jsdk_context_t *ctx, jsdk_desc_info_t *info)
{
    if (!jsdk_ctx_check(ctx) || !info) return JSDK_ERR_INVALID_ARG;
    if (!ctx->desc_present) return JSDK_ERR_BAD_STATE;
    *info = ctx->desc;
    info->endpoint_count = jsdk_ep_store_count(&ctx->store);
    info->parsed_total   = ctx->store.parsed_total;
    return JSDK_OK;
}

/* ==========================================================================
 * 公共：查询 / 遍历
 * ======================================================================== */

jsdk_status_t jsdk_endpoint_lookup(jsdk_context_t *ctx, const char *path,
                                   uint16_t *ep_id, jsdk_ep_type_t *type,
                                   uint8_t *access)
{
    if (!jsdk_ctx_check(ctx) || !path) return JSDK_ERR_INVALID_ARG;
    if (!ctx->desc_present) return JSDK_ERR_BAD_STATE;
    return jsdk_ep_store_lookup(&ctx->store, path, ep_id, type, access);
}

jsdk_status_t jsdk_endpoint_enumerate(jsdk_context_t *ctx,
                                      jsdk_endpoint_visit_fn fn, void *user)
{
    unsigned i, n;

    if (!jsdk_ctx_check(ctx) || !fn) return JSDK_ERR_INVALID_ARG;
    if (!ctx->desc_present) return JSDK_ERR_BAD_STATE;

    n = jsdk_ep_store_count(&ctx->store);
    for (i = 0u; i < n; ++i) {
        const char *path = NULL;
        uint16_t ep_id = 0u;
        jsdk_ep_type_t type = JSDK_EP_JSON;
        uint8_t access = 0u;

        if (jsdk_ep_store_at(&ctx->store, i, &path, &ep_id, &type, &access) != JSDK_OK) {
            continue;
        }
        if (fn(user, path, ep_id, type, access) != 0) break;
    }
    return JSDK_OK;
}

/* ==========================================================================
 * 公共：缓存（路线 A / 路线 B）
 * ======================================================================== */

size_t jsdk_desc_export_max_size(const jsdk_context_t *ctx)
{
    if (!jsdk_ctx_check(ctx)) return 0u;
    return cb_desc_cache_size(&ctx->store);
}

jsdk_status_t jsdk_context_desc_export(jsdk_context_t *ctx, void *buf, size_t cap,
                                       size_t *out_len)
{
    int early;

    if (!jsdk_ctx_check(ctx)) return JSDK_ERR_INVALID_ARG;
    if (!ctx->desc_present) return JSDK_ERR_BAD_STATE;

    early = (ctx->desc.complete == 0u) ? 1 : 0;
    {
        /* 导出内容与运行时 arena 无关，但头里的失效键必须用**同一份配置** */
        jsdk_desc_config_t c = ctx->cfg.desc;
        return cb_desc_cache_export(&c, &ctx->store, early, buf, cap, out_len);
    }
}

jsdk_status_t jsdk_context_desc_import(jsdk_context_t *ctx, const void *buf, size_t len)
{
    cb_desc_cache_meta_t meta;
    jsdk_status_t st;

    if (!jsdk_ctx_check(ctx) || !buf) return JSDK_ERR_INVALID_ARG;
    if (len == 0u) return JSDK_ERR_INVALID_ARG;
    if (!ctx->cfg.desc.arena) return JSDK_ERR_INVALID_ARG;

    st = cb_desc_cache_peek(buf, len, &meta);
    if (st != JSDK_OK) return st;

    store_init(ctx);
    st = cb_desc_cache_import(&ctx->cfg.desc, &ctx->store, buf, len);
    if (st != JSDK_OK) {
        ctx->desc_present = 0u;
        jsdk_ctx_seterr(ctx, "descriptor import rejected (%s); cache slot should "
                             "be invalidated and re-downloaded", jsdk_status_string(st));
        return st;
    }

    memset(&ctx->desc, 0, sizeof ctx->desc);
    ctx->desc.total_len      = 0u;
    ctx->desc.crc            = meta.desc_crc;
    ctx->desc.fw_version     = meta.fw_version;
    ctx->desc.endpoint_count = jsdk_ep_store_count(&ctx->store);
    ctx->desc.parsed_total   = ctx->store.parsed_total;
    ctx->desc.complete       = (uint8_t)((meta.flags & CB_DESC_CACHE_F_EARLY) ? 0u : 1u);
    ctx->desc.mode_used      = (uint8_t)JSDK_DESC_FROM_CACHE;
    ctx->desc.raw_sink_failed = 0u;
    ctx->desc_present = 1u;
    jsdk_ctx_publish_arena_used(ctx);
    return JSDK_OK;
}

jsdk_status_t jsdk_context_desc_import_raw(jsdk_context_t *ctx,
                                           const void *json, size_t len,
                                           const jsdk_desc_hint_t *hint)
{
    int rc;

    if (!jsdk_ctx_check(ctx) || !json || !hint) return JSDK_ERR_INVALID_ARG;
    if (len == 0u) return JSDK_ERR_INVALID_ARG;
    if (!ctx->cfg.desc.arena) return JSDK_ERR_INVALID_ARG;

    store_init(ctx);
    rc = jsdk_jsondesc_run(&ctx->cfg.desc, &ctx->store, json, len);
    if (rc != JSDK_OK) {
        ctx->desc_present = 0u;
        jsdk_ctx_seterr(ctx, "raw descriptor import failed (%s)", jsdk_status_string(rc));
        return rc;
    }

    memset(&ctx->desc, 0, sizeof ctx->desc);
    ctx->desc.total_len      = (uint32_t)len;
    ctx->desc.crc            = hint->crc;
    ctx->desc.fw_version     = hint->fw_version;
    ctx->desc.endpoint_count = jsdk_ep_store_count(&ctx->store);
    ctx->desc.parsed_total   = ctx->store.parsed_total;
    ctx->desc.complete       = 1u;      /* 整份 JSON 都在手里 */
    ctx->desc.mode_used      = (uint8_t)JSDK_DESC_FROM_CACHE;
    ctx->desc_present = 1u;
    jsdk_ctx_publish_arena_used(ctx);
    return JSDK_OK;
}

/* ======================== src/core/jsdk_fault.c ======================== */
/**
 * @file    jsdk_fault.c
 * @brief   WP4：故障码映射、链路健康、反馈新鲜度（诊断侧）
 *
 * 三套错误编码的层级关系（详见 joint_sdk.h §13.1）：
 *
 * @verbatim
 *   固件内部：  32-bit 子系统位图（motor/encoder/controller/… 各自一份）
 *                   │  固件 detect_error_code() 归约
 *                   ▼
 *   线上摘要：  4-bit ErrorCode（MIT 响应的 err 半字节）
 *
 *   另有一条独立的快速通道：心跳 5-bit 子系统位图（只表示"哪个子系统有错"）
 * @endverbatim
 *
 * 因此 `err_code != 0` 与 `hb_error != 0` 是**两个不同粒度的信号**，
 * 而 `axis_error` 的第 20 位（CAN_BUS_FAILED）专指协议级超时。
 */



/* ==========================================================================
 * 32-bit 子系统错误位（固件 Firmware/autogen/interfaces.hpp 的 Axis::Error）
 *
 * ⚠ 位号不连续：固件只定义了这些取值。未定义的位返回 NULL，
 *   而不是编一个名字出来（"不猜"原则）。
 * ======================================================================== */

typedef struct {
    unsigned    bit;
    const char *name;
} bit_name_t;

static const bit_name_t k_axis_err[] = {
    {  0u, "INVALID_STATE" },
    {  6u, "MOTOR_FAILED" },
    {  7u, "SENSORLESS_ESTIMATOR_FAILED" },
    {  8u, "ENCODER_FAILED" },
    {  9u, "CONTROLLER_FAILED" },
    { 11u, "WATCHDOG_TIMER_EXPIRED" },
    { 12u, "MIN_ENDSTOP_PRESSED" },
    { 13u, "MAX_ENDSTOP_PRESSED" },
    { 14u, "ESTOP_REQUESTED" },
    { 17u, "HOMING_WITHOUT_ENDSTOP" },
    { 18u, "OVER_TEMP" },
    { 19u, "UNKNOWN_POSITION" },
    { 20u, "CAN_BUS_FAILED" },
};

static const char *const k_hb_err[5] = {
    "axis",        /* CB_HB_ERR_AXIS       0x01 */
    "motor",       /* CB_HB_ERR_MOTOR      0x02 */
    "encoder",     /* CB_HB_ERR_ENCODER    0x04 */
    "controller",  /* CB_HB_ERR_CONTROLLER 0x08 */
    "board"        /* CB_HB_ERR_BOARD      0x10 */
};

/** 固件 `AxisState` 0..16（**注意** 5 是空缺，16 超出心跳的 4 bit）。 */
static const char *const k_can_axis_state[17] = {
    "UNDEFINED",                    /*  0 */
    "IDLE",                         /*  1 */
    "STARTUP_SEQUENCE",             /*  2 */
    "FULL_CALIBRATION_SEQUENCE",    /*  3 */
    "MOTOR_CALIBRATION",            /*  4 */
    "reserved",                     /*  5 —— 固件未定义该值 */
    "ENCODER_INDEX_SEARCH",         /*  6 */
    "ENCODER_OFFSET_CALIBRATION",   /*  7 */
    "CLOSED_LOOP_CONTROL",          /*  8 */
    "LOCKIN_SPIN",                  /*  9 */
    "ENCODER_DIR_FIND",             /* 10 */
    "HOMING",                       /* 11 */
    "ENCODER_HALL_POLARITY_CALIBRATION", /* 12 */
    "ENCODER_HALL_PHASE_CALIBRATION",    /* 13 */
    "INERTIA_CALIBRATION",          /* 14 */
    "ENCODER_LINEARIZATION",        /* 15 */
    "MOTOR_DEADTIME_CALIBRATION"    /* 16 */
};

/* ==========================================================================
 * 名称查询
 * ======================================================================== */

const char *jsdk_joint_error_string(uint8_t mit_err_code)
{
    /* 复用协议层表，保证与 MIT 响应解码永远一致 */
    return cb_mit_error_name(mit_err_code);
}

const char *jsdk_hb_error_bit_name(unsigned bit)
{
    if (bit >= (unsigned)(sizeof k_hb_err / sizeof k_hb_err[0])) return NULL;
    return k_hb_err[bit];
}

const char *jsdk_axis_error_bit_name(unsigned bit)
{
    unsigned i;

    for (i = 0u; i < (unsigned)(sizeof k_axis_err / sizeof k_axis_err[0]); ++i) {
        if (k_axis_err[i].bit == bit) return k_axis_err[i].name;
    }
    return NULL;   /* 固件未定义的位：不编名字 */
}

const char *jsdk_axis_error_first(uint32_t value, unsigned *bit_out)
{
    unsigned i;

    /* 按**固件位号从小到大**找第一个置位，保证同一 value 永远给同一答案 */
    for (i = 0u; i < (unsigned)(sizeof k_axis_err / sizeof k_axis_err[0]); ++i) {
        if (value & (1u << k_axis_err[i].bit)) {
            if (bit_out) *bit_out = k_axis_err[i].bit;
            return k_axis_err[i].name;
        }
    }
    if (bit_out) *bit_out = 0u;
    return NULL;
}

const char *jsdk_can_axis_state_name(uint8_t can_axis_state)
{
    if (can_axis_state >= (uint8_t)(sizeof k_can_axis_state / sizeof k_can_axis_state[0])) {
        return "unknown";
    }
    return k_can_axis_state[can_axis_state];
}

/* ==========================================================================
 * 组合描述
 * ======================================================================== */

/** 在 @p n 之后追加，返回新的长度（含长度上限保护）。 */
static size_t app(char *buf, size_t cap, size_t n, const char *s)
{
    size_t l = strlen(s);
    size_t i;

    if (n >= cap) return n;
    for (i = 0u; i < l && n + 1u < cap; ++i) buf[n++] = s[i];
    buf[n] = '\0';
    return n;
}

static size_t app_u32_hex(char *buf, size_t cap, size_t n, uint32_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    int shift;

    if (n + 1u >= cap) return n;
    buf[n++] = '0'; buf[n++] = 'x'; buf[n] = '\0';
    for (shift = 28; shift >= 0; shift -= 4) {
        if (n + 1u < cap) buf[n++] = hex[(v >> (unsigned)shift) & 0xFu];
    }
    buf[n] = '\0';
    return n;
}

int jsdk_joint_describe_fault(const jsdk_joint_t *j, char *buf, size_t cap)
{
    size_t n = 0u;
    unsigned b;

    if (!j || !buf || cap == 0u) return 0;
    buf[0] = '\0';

    if (!jsdk_joint_check(j)) {
        return (int)app(buf, cap, n, "invalid joint");
    }

    /* 32-bit 明细（来自 0x45；未查询时为 0） */
    if (j->fault.valid) {
        uint32_t v = j->fault.motor_error | j->fault.encoder_error
                   | j->fault.sensorless_error | j->fault.controller_error
                   | j->fault.system_error | j->fault.axis_error;
        n = app(buf, cap, n, "detail_err=");
        n = app_u32_hex(buf, cap, n, v);
        if (v != 0u) {
            const char *nm = jsdk_axis_error_first(j->fault.axis_error, &b);
            if (nm) { n = app(buf, cap, n, "("); n = app(buf, cap, n, nm);
                      n = app(buf, cap, n, ")"); }
        }
        n = app(buf, cap, n, " ");
    }

    n = app(buf, cap, n, "axis_error=");
    n = app_u32_hex(buf, cap, n, j->fb.axis_error);
    {
        const char *nm = jsdk_axis_error_first(j->fb.axis_error, &b);
        if (nm) { n = app(buf, cap, n, "("); n = app(buf, cap, n, nm);
                  n = app(buf, cap, n, ")"); }
    }

    n = app(buf, cap, n, " mit_err=");
    n = app(buf, cap, n, jsdk_joint_error_string(j->fb.err_code));

    n = app(buf, cap, n, " hb=");
    if (j->fb.hb_error == 0u) {
        n = app(buf, cap, n, "none");
    } else {
        unsigned i;
        int first = 1;
        for (i = 0u; i < 5u; ++i) {
            if (j->fb.hb_error & (uint8_t)(1u << i)) {
                if (!first) n = app(buf, cap, n, "|");
                n = app(buf, cap, n, jsdk_hb_error_bit_name(i));
                first = 0;
            }
        }
    }

    n = app(buf, cap, n, " can_state=");
    n = app(buf, cap, n, jsdk_can_axis_state_name(j->current_state_raw));

    return (int)n;
}

/* ==========================================================================
 * WP4：反馈新鲜度
 * ======================================================================== */

/**
 * 反馈超时阈值（ms）。
 *
 * 三个来源取最保守的一个（**越大越保守**，因为误报 stale 会让客户以为设备挂了）：
 *   1. 心跳周期 × 3（心跳是唯一无请求的周期反馈）
 *   2. 控制周期 × 3
 *   3. 硬下限 50 ms
 *
 * @note 该阈值只能由**观测**推导，不引入新配置项 —— 客户不需要为它调参。
 */
uint32_t jsdk_joint_stale_ms(const jsdk_joint_t *j)
{
    uint32_t ms = 50u;

    if (!jsdk_joint_check(j)) return ms;

    if (j->heartbeat_rate_ms != 0u) {
        uint32_t v = j->heartbeat_rate_ms * 3u;
        if (v > ms) ms = v;
    }
    if (j->ctx->cfg.period_ns != 0u) {
        uint32_t v = (uint32_t)(j->ctx->cfg.period_ns / 1000000u) * 6u;
        if (v > ms) ms = v;
    }
    return ms;
}

/**
 * 每个周期调用：刷新 `age_ms`，并在超时后置 `JSDK_JF_FEEDBACK_STALE`。
 *
 * ⚠ 置位是**粘滞**的（与其余 `JSDK_JF_*` 一致）：恢复后要客户显式
 *   `jsdk_joint_clear_status_flags()`，否则现场会看不到"曾经掉过反馈"。
 */
void jsdk_joint__refresh_freshness(jsdk_joint_t *j)
{
    uint32_t age;

    if (!jsdk_joint_check(j)) return;
    if (!j->fb.online) return;          /* 从没收到过 → 不算 stale，是 offline */

    age = jsdk_elapsed(j->ctx->now_ms, j->last_fb_ms);
    j->fb.age_ms = age;

    if (age > jsdk_joint_stale_ms(j)) {
        jsdk_joint_set_flags(j, (uint16_t)JSDK_JF_FEEDBACK_STALE);
    }
}

/* ======================== src/core/jsdk_group.c ======================== */
/**
 * @file    jsdk_group.c
 * @brief   WP6：分组与广播同步（一条帧驱动多个关节）
 *
 * @par 广播的寻址与槽位（两条都来自固件，踩错就“静默不动”）
 *
 * **FD（0x80 MIT 广播）**：`Dest` 是位图（bit n = node_id n），**槽位号就是 node_id**，
 * 即设备 n 读 `[n*8, n*8+8)`。因此：
 *   - 帧长必须是 `(max_node_id + 1) * 8` —— 驱动 1..4 号就是 40 字节，
 *     **前 8 字节属于不存在的设备 0**，不能省；
 *   - 固件对 `len < (slot+1)*8` 的帧**整帧丢弃**，所以末尾空槽位必须用安全值
 *     （kp = kd = tau = 0）填充，否则后段设备静默不动。
 *
 * **Classic（0x81 MIT 广播）**：**恒用槽位 0**，被位图命中的所有设备执行
 * 同一条命令（帧长只需 8）。想让各设备执行不同指令**只能**用 CAN FD。
 *
 * @par 广播不回复
 * 所以 `jsdk_group_set_mit()` 不产生任何反馈。反馈仍来自心跳与后续单播查询。
 * 但广播帧**确实**刷新固件的协议级超时计时器（0x80 属于 `is_ctrl`），
 * 因此组内关节的 `last_ctrl_tx_ms` 会跟着更新。
 *
 * @par 为什么 enable/disable 是单播
 * 协议**没有**广播版的 `START_MOTOR`/`STOP_MOTOR`（MsgType 只有 0x80~0x83 是广播，
 * 且全是实时控制）。所以 `jsdk_group_enable/disable()` 是逐个下发请求的便利封装 ——
 * 真正的"同步"只存在于实时控制帧。这些请求随后由调用者的循环推进（非阻塞）。
 */



/* ==========================================================================
 * 内部工具
 * ======================================================================== */

/** 组内允许的最大成员数（位图只有 node_id 1..7 这 7 个可用位）。 */
#define JSDK_GROUP_MAX_MEMBERS 7u

/** 收集并校验一组关节。 */
static jsdk_status_t collect(jsdk_context_t *ctx, const jsdk_group_target_t *targets,
                             unsigned n, jsdk_joint_t **out, uint8_t *mask_out)
{
    unsigned i, k;
    uint8_t  ids[JSDK_GROUP_MAX_MEMBERS];
    int      mask;

    if (!jsdk_ctx_check(ctx) || !targets) return JSDK_ERR_INVALID_ARG;
    if (n == 0u) return JSDK_ERR_INVALID_ARG;
    if (n > JSDK_GROUP_MAX_MEMBERS) {
        /* 位图只有 7 个可用位；再多就必须分多条帧 */
        jsdk_ctx_seterr(ctx, "group size %u exceeds the bitmap limit (%u devices, "
                             "node_id 1..7)", n, (unsigned)JSDK_GROUP_MAX_MEMBERS);
        return JSDK_ERR_UNSUPPORTED;
    }

    for (i = 0u; i < n; ++i) {
        uint8_t id = targets[i].node_id;

        if (id < CB_NODE_ID_MIN) {
            jsdk_ctx_seterr(ctx, "group member %u has invalid node_id 0", i);
            return JSDK_ERR_INVALID_ARG;
        }
        if (id >= CB_MAX_BROADCAST_DEVICES) {
            /* node_id ≥ 8 无法位掩码寻址（固件直接判 is_for_me == 0） */
            jsdk_ctx_seterr(ctx, "node_id %u cannot be addressed by the broadcast "
                                 "bitmap (only 1..7); use unicast for this joint",
                            (unsigned)id);
            return JSDK_ERR_UNSUPPORTED;
        }
        for (k = 0u; k < i; ++k) {
            if (ids[k] == id) {
                jsdk_ctx_seterr(ctx, "duplicate node_id %u in one group", (unsigned)id);
                return JSDK_ERR_INVALID_ARG;
            }
        }
        out[i] = jsdk_ctx_find_joint(ctx, id);
        if (!out[i]) {
            jsdk_ctx_seterr(ctx, "node %u is not one of this context's joints", (unsigned)id);
            return JSDK_ERR_NOT_FOUND;
        }
        /* 没使能过的关节不能进组：否则等于替客户驱动一台 IDLE 设备 */
        if (!out[i]->tx_active) {
            jsdk_ctx_seterr(ctx, "joint %u (node %u) is not enabled; call "
                                 "request_enable() first", (unsigned)out[i]->index,
                            (unsigned)id);
            return JSDK_ERR_BAD_STATE;
        }
        if (!out[i]->calibrated) {
            jsdk_ctx_seterr(ctx, "joint %u (node %u) has no valid calibration",
                            (unsigned)out[i]->index, (unsigned)id);
            return JSDK_ERR_BAD_STATE;
        }
        /* 这个函数发的是 **MIT** 帧；如果关节正被另一种模式驱动，
           一条 MIT 广播会把设备悄悄切到 MIT 输入模式 —— 必须拦住。 */
        if (out[i]->mode != JSDK_MODE_MIT) {
            jsdk_ctx_seterr(ctx, "joint %u (node %u) is in %s mode; "
                                 "group_set_mit() only drives MIT joints",
                            (unsigned)out[i]->index, (unsigned)id,
                            jsdk_mode_string(out[i]->mode));
            return JSDK_ERR_BAD_STATE;
        }
        ids[i] = id;
    }

    mask = cb_make_broadcast_mask(ids, n);
    if (mask < 0) return JSDK_ERR_UNSUPPORTED;   /* 理论上到不了这里 */
    *mask_out = (uint8_t)mask;
    return JSDK_OK;
}

/** 把目标写进关节的缓存，并记账（广播帧同样喂看门狗）。 */
static void accept_target(jsdk_joint_t *j, const jsdk_group_target_t *t, uint32_t now_ms)
{
    j->tgt.pos_rad   = t->pos_rad;
    j->tgt.vel_rad_s = t->vel_rad_s;
    j->tgt.kp        = t->kp;
    j->tgt.kd        = t->kd;
    j->tgt.tau_Nm    = t->tau_Nm;
    j->tgt.have_pos  = 1u;
    j->tgt.have_vel  = 1u;
    j->tgt.have_mit  = 1u;
    j->tgt.have_tau  = 1u;

    j->tx_frames++;
    j->fb.tx_frames = j->tx_frames;
    j->last_ctrl_tx_ms = now_ms;      /* 0x80 属于 is_ctrl → 刷新协议级超时 */

    /* ⚠ 必须记上“本周期已发”：否则 cycle_end() 还会给这些关节各补一条单播，
       于是“一条广播驱动 N 个关节”变成“N+1 条帧”，广播同步完全失去意义。 */
    j->sent_cycle = 1u;
}

/** 判断两个目标是否完全相同（Classic 广播只能发同一条命令）。 */
static int same_target(const jsdk_group_target_t *a, const jsdk_group_target_t *b)
{
    return (a->pos_rad == b->pos_rad
            && a->vel_rad_s == b->vel_rad_s
            && a->kp == b->kp
            && a->kd == b->kd
            && a->tau_Nm == b->tau_Nm) ? 1 : 0;
}

/** 降级路径：逐个关节发单播 MIT（复用 L3 的编码与越界策略）。 */
static jsdk_status_t unicast_fallback(jsdk_context_t *ctx,
                                      const jsdk_group_target_t *targets,
                                      jsdk_joint_t *const *joints, unsigned n)
{
    unsigned i;

    /* 先把"本周期已发过"这类调用顺序问题一次查清，免得改到一半才发现 */
    for (i = 0u; i < n; ++i) {
        if (joints[i]->sent_cycle) {
            jsdk_ctx_seterr(ctx, "node %u already sent a control frame in this cycle; "
                                 "each joint may be commanded only once per cycle",
                            (unsigned)targets[i].node_id);
            return JSDK_ERR_BAD_STATE;
        }
    }

    for (i = 0u; i < n; ++i) {
        jsdk_joint_t *j = joints[i];

        jsdk_joint_set_mit(j, targets[i].pos_rad, targets[i].vel_rad_s,
                           targets[i].kp, targets[i].kd, targets[i].tau_Nm);
        /* 走与单播完全相同的编码/越界路径：越界时发安全帧而不是静默丢帧 */
        if (jsdk_joint__send_now(j) == 0) {
            jsdk_ctx_seterr(ctx, "group degraded to unicast but node %u could not be sent",
                            (unsigned)targets[i].node_id);
            return JSDK_ERR_TRANSPORT;
        }
    }
    jsdk_ctx_seterr(ctx, "group command degraded to %u unicast frame(s) "
                         "(%s cannot carry per-node targets)",
                    n, ctx->cfg.is_fd ? "range check failed"
                                      : "Classic CAN broadcast");
    return JSDK_OK;
}

/* ==========================================================================
 * jsdk_group_set_mit
 * ======================================================================== */

jsdk_status_t jsdk_group_set_mit(jsdk_context_t *ctx,
                                 const jsdk_group_target_t *targets, unsigned n)
{
    jsdk_joint_t *joints[JSDK_GROUP_MAX_MEMBERS];
    uint8_t  mask = 0u;
    uint8_t  payload[CB_MIT_SLOT_BYTES * CB_MAX_BROADCAST_DEVICES];
    uint8_t  max_slot = 0u;
    size_t   len;
    unsigned i;
    jsdk_status_t st;

    st = collect(ctx, targets, n, joints, &mask);
    if (st != JSDK_OK) return st;

    /* --- Classic：只有槽位 0。全员同目标才发一条广播，否则降级单播 --- */
    if (!ctx->cfg.is_fd) {
        for (i = 1u; i < n; ++i) {
            if (!same_target(&targets[0], &targets[i])) {
                return unicast_fallback(ctx, targets, joints, n);
            }
        }
        memset(payload, 0, sizeof payload);
        {
            uint8_t clamped = 0u;
            cb_mit_pack_command(payload, &joints[0]->range,
                                (float)targets[0].pos_rad, (float)targets[0].vel_rad_s,
                                (float)targets[0].kp, (float)targets[0].kd,
                                (float)targets[0].tau_Nm, &clamped);
        }
        len = CB_MIT_SLOT_BYTES;
        if (jsdk_ctx_send(ctx, CB_PRI_HIGH_CTRL, CB_MSG_MIT_CONTROL_BCAST, mask,
                          payload, (uint8_t)len) != 0) {
            return JSDK_ERR_TRANSPORT;
        }
        ctx->tx_seq = cb_seq_next(ctx->tx_seq);
        for (i = 0u; i < n; ++i) accept_target(joints[i], &targets[i], ctx->now_ms);
        jsdk_ctx_seterr(ctx, "group MIT (Classic, slot 0): %u devices share one command",
                        n);
        return JSDK_OK;
    }

    memset(payload, 0, sizeof payload);

    /* 先确定最大槽位（帧长依赖它） */
    for (i = 0u; i < n; ++i) {
        if (targets[i].node_id > max_slot) max_slot = targets[i].node_id;
    }

    /* ⚠⚠ 未使用的槽位**不能留全零**：MIT 命令的 64 bit 全零 → 解出来是
       pos = -pos_max、vel = -vel_max、tau = -tau_max（0 是各字段的最小码），
       等于“满速反向 + 满力矩反向”。位图会让未被寻址的设备忽略该帧，但一旦
       掩码写错、或将来改用 Dest=0xFF 全局广播，那就是**飞车指令**。
       ⇒ 每个未使用的槽位都显式写“零目标 + 零增益”。 */
    for (i = 0u; i <= (unsigned)max_slot; ++i) {
        jsdk_joint_t *ref = NULL;
        cb_mit_range_t range;
        unsigned k;

        for (k = 0u; k < n; ++k) {
            if (targets[k].node_id == i) { ref = joints[k]; break; }
        }
        if (ref) continue;                       /* 这个槽位稍后填真目标 */

        /* 量程用第 0 个关节的（只为编码 0，任何量程都等价），
           但 kp/kd/tau 三段的位宽是固定的，所以结果与量程无关地安全。 */
        range = joints[0]->range;
        cb_mit_pack_command(&payload[(size_t)i * CB_MIT_SLOT_BYTES], &range,
                            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, NULL);
    }

    for (i = 0u; i < n; ++i) {
        jsdk_joint_t *j = joints[i];
        uint8_t slot = targets[i].node_id;
        uint8_t clamped = 0u;

        cb_mit_pack_command(&payload[(size_t)slot * CB_MIT_SLOT_BYTES], &j->range,
                            (float)targets[i].pos_rad, (float)targets[i].vel_rad_s,
                            (float)targets[i].kp, (float)targets[i].kd,
                            (float)targets[i].tau_Nm, &clamped);

        if (clamped != 0u) {
            /* 越界不能在组路径里静默钳位：改用单播，让 §6.10 的策略生效 */
            jsdk_ctx_seterr(ctx, "node %u target out of range in group command",
                            (unsigned)slot);
            return unicast_fallback(ctx, targets, joints, n);
        }
    }

    /* 帧长 = (最大槽位 + 1) × 8；槽位 0 属于不存在的设备 0，但**必须存在** */
    len = cb_mit_bcast_frame_len(max_slot);
    if (len == 0u) return JSDK_ERR_UNSUPPORTED;

    if (jsdk_ctx_send(ctx, CB_PRI_HIGH_CTRL, CB_MSG_MIT_CONTROL_BCAST, mask,
                      payload, (uint8_t)len) != 0) {
        return JSDK_ERR_TRANSPORT;
    }
    ctx->tx_seq = cb_seq_next(ctx->tx_seq);
    for (i = 0u; i < n; ++i) accept_target(joints[i], &targets[i], ctx->now_ms);

    jsdk_ctx_seterr(ctx, "group MIT (FD): mask=0x%02X len=%u B, %u joints in one frame",
                    (unsigned)mask, (unsigned)len, n);
    return JSDK_OK;
}

/* ==========================================================================
 * jsdk_group_enable / jsdk_group_disable
 * ======================================================================== */

/** 把一组 node_id 变成关节指针（enable/disable 用；不限制 1..7，因为是单播）。 */
static jsdk_status_t collect_joints(jsdk_context_t *ctx, const uint8_t *node_ids,
                                    unsigned n, jsdk_joint_t **out)
{
    unsigned i, k;

    if (!jsdk_ctx_check(ctx) || !node_ids) return JSDK_ERR_INVALID_ARG;
    if (n == 0u || n > JSDK_MAX_JOINTS) return JSDK_ERR_INVALID_ARG;

    for (i = 0u; i < n; ++i) {
        if (node_ids[i] < CB_NODE_ID_MIN) return JSDK_ERR_INVALID_ARG;
        for (k = 0u; k < i; ++k) {
            if (node_ids[k] == node_ids[i]) return JSDK_ERR_INVALID_ARG;
        }
        out[i] = jsdk_ctx_find_joint(ctx, node_ids[i]);
        if (!out[i]) {
            jsdk_ctx_seterr(ctx, "node %u is not one of this context's joints",
                            (unsigned)node_ids[i]);
            return JSDK_ERR_NOT_FOUND;
        }
    }
    return JSDK_OK;
}

jsdk_status_t jsdk_group_enable(jsdk_context_t *ctx, const uint8_t *node_ids,
                                unsigned n)
{
    jsdk_joint_t *joints[JSDK_MAX_JOINTS];
    unsigned i;
    jsdk_status_t st = collect_joints(ctx, node_ids, n, joints);

    if (st != JSDK_OK) return st;

    /* 协议没有广播版 START_MOTOR → 逐个下单播请求，由调用者的循环推进 */
    for (i = 0u; i < n; ++i) {
        jsdk_joint_request_enable(joints[i], joints[i]->mode);
    }
    jsdk_ctx_seterr(ctx, "group enable requested for %u joint(s) "
                         "(unicast: the protocol has no broadcast START_MOTOR)",
                    n);
    return JSDK_OK;
}

jsdk_status_t jsdk_group_disable(jsdk_context_t *ctx, const uint8_t *node_ids,
                                 unsigned n)
{
    jsdk_joint_t *joints[JSDK_MAX_JOINTS];
    unsigned i;
    jsdk_status_t st = collect_joints(ctx, node_ids, n, joints);

    if (st != JSDK_OK) return st;

    for (i = 0u; i < n; ++i) {
        jsdk_joint_request_disable(joints[i]);
    }
    jsdk_ctx_seterr(ctx, "group disable requested for %u joint(s) (unicast)", n);
    return JSDK_OK;
}

/* ======================== src/core/jsdk_joint.c ======================== */
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
    if (hb.control_mode <= (uint8_t)CB_HB_CM_POSITION) {
        /* 心跳的 control_mode 只有 4 个值，粒度比 ModeState 粗；
           仅在还没有 ModeState 时用它粗略同步 */
        if (!j->fb.online) { /* 保留：不做更细映射，避免误判模式 */ }
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
    j->tgt.cur_lim_A     = cur_lim_A;
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
        float cur_a   = (float)j->tgt.cur_lim_A;
        n = cb_ctrl_pos_pack(payload, classic, pos_deg, vel_rpm, cur_a, &flags);
        break;
    }
    case JSDK_MODE_CSV: {
        float vel_rpm = (float)jsdk_units_rad_s_to_rpm(j->tgt.vel_rad_s);
        n = cb_ctrl_vel_pack(payload, vel_rpm, (float)j->tgt.cur_lim_A, &flags);
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

/* ======================== src/core/jsdk_ops.c ======================== */
/**
 * @file    jsdk_ops.c
 * @brief   WP5 运维：零点、标定、回零、保存配置、复位、改节点号、看门狗超时
 *
 * @par 为什么这些 API 可以阻塞
 * 它们都是**配置阶段**调用（§7 与 §12 的约定）：现场工程师按下按钮、
 * 等几百 ms 到几秒。**绝不可**在控制循环运行期间调用 —— 会阻塞周期并
 * 与响应争用。
 *
 * @par 为什么标定/回零要写端点而不是发命令
 * 固件没有“开始标定”的 CAN 命令，只能写 `axis0.requested_state`（端点 143）。
 * 写完必须**等 `axis0.current_state` 离开瞬时值**才算完成，否则客户会在
 * 电机还在转动时开始下发控制帧。这两个端点 ID 来自运行时描述符，不硬编码。
 */



/* ==========================================================================
 * 内部工具
 * ======================================================================== */

#define P_CURRENT_ST  "axis0.current_state"

/** 读设备当前状态（固件 AxisState 0..16）；失败返回 -1。 */
static int read_current_state(jsdk_joint_t *j, uint8_t *out)
{
    uint8_t buf[8];
    uint8_t len = 0u;

    if (j->ep_current_state == 0u) {
        if (jsdk_ep_store_lookup(&j->ctx->store, P_CURRENT_ST,
                                 &j->ep_current_state, NULL, NULL) != JSDK_OK) {
            return -1;
        }
    }
    if (jsdk_ctx_read_param(j->ctx, j->cfg.node_id, j->ep_current_state,
                            buf, &len, 0u) != JSDK_OK) {
        return -1;
    }
    if (len < 1u) return -1;
    *out = buf[0];
    j->current_state_raw = buf[0];
    j->state_known = 1u;
    return 0;
}

/** 写一个 u8 端点（`requested_state` 用）。 */
static int write_u8_ep(jsdk_joint_t *j, uint16_t ep, uint8_t v)
{
    if (ep == 0u) return -1;
    return jsdk_ctx_write_param(j->ctx, j->cfg.node_id, ep, &v, 1u, 0u);
}

/**
 * 请求一次状态跳转并等到设备**真正离开**该瞬时状态。
 *
 * 判定依据（三段式，缺一不可）：
 *   1. 写完之后必须**观测到** `current_state == transient`（否则是"根本没开始"，例如
 *      端点写失败或设备不接受该状态）；
 *   2. 随后必须观测到 `current_state != transient`（否则是"还在跑"）；
 *   3. 超时仍未离开 → `JSDK_ERR_TIMEOUT`。
 *
 * 把①单独列出来很重要：只看"最终不等于 3"会把"压根没开始"当成"标定完成"。
 */
static jsdk_status_t request_state_and_wait(jsdk_joint_t *j, uint8_t transient,
                                            uint32_t timeout_ms)
{
    uint32_t t0;
    unsigned spin = 0u;
    int seen_transient = 0;

    if (j->ep_requested_state == 0u) {
        jsdk_joint_seterr(j, "endpoint axis0.requested_state unavailable "
                             "(descriptor not parsed?)");
        return JSDK_ERR_NOT_FOUND;
    }

    /* 期间不允许发控制帧（标定会转电机，客户指令会打架） */
    j->ctrl_blocked = 1u;

    if (write_u8_ep(j, j->ep_requested_state, transient) != 0) {
        j->ctrl_blocked = 0u;
        jsdk_joint_seterr(j, "failed to write axis0.requested_state = %u",
                          (unsigned)transient);
        return JSDK_ERR_TRANSPORT;
    }

    t0 = j->ctx->cfg.hal.now_ms(j->ctx->cfg.hal.user);

    for (;;) {
        uint8_t st = 0u;

        if (read_current_state(j, &st) == 0) {
            j->current_state_raw = st;
            if ((uint8_t)st == transient) {
                seen_transient = 1;
            } else if (seen_transient) {
                j->ctx->now_ms = j->ctx->cfg.hal.now_ms(j->ctx->cfg.hal.user);
                j->ctrl_blocked = 0u;
                jsdk_ctx_seterr(j->ctx, "node %u left state %u (now %u)",
                                (unsigned)j->cfg.node_id, (unsigned)transient,
                                (unsigned)st);
                return JSDK_OK;
            }
        }

        j->ctx->now_ms = j->ctx->cfg.hal.now_ms(j->ctx->cfg.hal.user);
        if (jsdk_elapsed(j->ctx->now_ms, t0) >= timeout_ms) break;
        if (++spin >= 200000u) break;      /* 时钟冻结的仿真 HAL 兜底 */
    }

    j->ctrl_blocked = 0u;
    jsdk_joint_seterr(j, seen_transient
        ? "state %u did not finish within %u ms"
        : "device never entered state %u (write accepted but state unchanged?)",
        (unsigned)transient, (unsigned)timeout_ms);
    return JSDK_ERR_TIMEOUT;
}

/* ==========================================================================
 * 零点 / 标定 / 回零
 * ======================================================================== */

jsdk_status_t jsdk_joint_set_zero_here(jsdk_joint_t *j)
{
    uint32_t t0;
    unsigned spin = 0u;

    if (!jsdk_joint_check(j)) return JSDK_ERR_INVALID_ARG;

    /* 0x61 无应答；写完后**读回位置**来确认零点确实动了 —— 这比"发出去了"强得多 */
    if (jsdk_ctx_send(j->ctx, CB_PRI_CTRL, CB_MSG_SET_ZERO, j->cfg.node_id,
                      NULL, 0u) != 0) {
        jsdk_joint_seterr(j, "SET_ZERO could not be sent");
        return JSDK_ERR_TRANSPORT;
    }
    j->ctx->tx_seq = cb_seq_next(j->ctx->tx_seq);

    t0 = j->ctx->cfg.hal.now_ms(j->ctx->cfg.hal.user);
    for (;;) {
        float turns = 0.0f;
        jsdk_can_frame_t rsp;

        if (jsdk_ctx_send(j->ctx, CB_PRI_QUERY, CB_MSG_QUERY_POS_VEL,
                          j->cfg.node_id, NULL, 0u) != 0) {
            return JSDK_ERR_TRANSPORT;
        }
        j->ctx->tx_seq = cb_seq_next(j->ctx->tx_seq);

        if (jsdk_ctx_wait_response(j->ctx, CB_MSG_QUERY_POS_VEL, j->cfg.node_id,
                                   &rsp, JSDK_CFG_TIMEOUT_MS) == JSDK_OK) {
            cb_query_pos_vel_t pv;
            if (cb_query_decode_pos_vel(rsp.data, rsp.len, &pv) == 0) {
                turns = pv.pos_turns;
                if (turns > -0.001f && turns < 0.001f) {
                    jsdk_ctx_seterr(j->ctx, "SET_ZERO verified: pos_estimate = 0");
                    return JSDK_OK;
                }
            }
        }

        j->ctx->now_ms = j->ctx->cfg.hal.now_ms(j->ctx->cfg.hal.user);
        if (jsdk_elapsed(j->ctx->now_ms, t0) >= JSDK_CFG_TIMEOUT_MS) break;
        if (++spin >= 40000u) break;
    }

    jsdk_joint_seterr(j, "SET_ZERO sent but position did not become 0 "
                         "(is the axis in closed loop?)");
    return JSDK_ERR_TIMEOUT;
}

jsdk_status_t jsdk_joint_calibrate(jsdk_joint_t *j)
{
    if (!jsdk_joint_check(j)) return JSDK_ERR_INVALID_ARG;

    /* 固件要求标定前处于 IDLE；使能中调用会与闭环控制打架 */
    if (jsdk_joint_is_enabled(j)) {
        jsdk_joint_seterr(j, "calibrate() requires the axis to be disabled first "
                             "(call request_disable + poll until IDLE)");
        return JSDK_ERR_BAD_STATE;
    }

    return request_state_and_wait(j, 3u, 20000u);   /* AXIS_STATE_FULL_CALIBRATION_SEQUENCE */
}

jsdk_status_t jsdk_joint_home(jsdk_joint_t *j)
{
    if (!jsdk_joint_check(j)) return JSDK_ERR_INVALID_ARG;
    if (!j->calibrated) return JSDK_ERR_BAD_STATE;
    return request_state_and_wait(j, 11u, 5000u);
}

/* ==========================================================================
 * 保存配置 / 复位 / 改节点号 / 看门狗
 * ======================================================================== */

jsdk_status_t jsdk_joint_save_config(jsdk_joint_t *j)
{
    if (!jsdk_joint_check(j)) return JSDK_ERR_INVALID_ARG;

    /* CONFIG_SAVE(0x22) 无应答 → 只能用"读回校验"确认真的落盘了 */
    if (jsdk_ctx_send(j->ctx, CB_PRI_CONFIG, CB_MSG_CONFIG_SAVE, j->cfg.node_id,
                      NULL, 0u) != 0) {
        jsdk_joint_seterr(j, "CONFIG_SAVE could not be sent");
        return JSDK_ERR_TRANSPORT;
    }
    j->ctx->tx_seq = cb_seq_next(j->ctx->tx_seq);

    /* 校验点：读回 break_timeout（标定量程里唯一不该被我们改过的可写值之一） */
    if (j->ep_break_timeout != 0u) {
        uint8_t buf[8];
        uint8_t len = 0u;
        if (jsdk_ctx_read_param(j->ctx, j->cfg.node_id, j->ep_break_timeout,
                                buf, &len, 0u) != JSDK_OK || len < 2u) {
            jsdk_joint_seterr(j, "CONFIG_SAVE verification failed: cannot read "
                                 "back can.config.break_timeout");
            return JSDK_ERR_TIMEOUT;
        }
    }
    jsdk_ctx_seterr(j->ctx, "configuration saved and verified on node %u",
                    (unsigned)j->cfg.node_id);
    return JSDK_OK;
}

jsdk_status_t jsdk_joint_reset_device(jsdk_joint_t *j)
{
    if (!jsdk_joint_check(j)) return JSDK_ERR_INVALID_ARG;

    if (jsdk_ctx_send(j->ctx, CB_PRI_CTRL, CB_MSG_RESET_DEVICE, j->cfg.node_id,
                      NULL, 0u) != 0) {
        return JSDK_ERR_TRANSPORT;
    }
    j->ctx->tx_seq = cb_seq_next(j->ctx->tx_seq);

    /* 设备在重启 → 必须重新握手，否则心跳仍发往旧 master_id 之外的地址。
       同时清掉本地的状态缓存（它们关于复位前的设备）。 */
    j->fb = (jsdk_joint_feedback_t){ 0 };
    j->current_state_raw = 0u;
    j->state_known = 0u;
    j->fault_prev = 0u;
    j->enabled = 0u;
    j->first_frame_done = 0u;
    j->tx_active = 0u;
    j->last_ctrl_tx_ms = 0u;
    j->status_flags = (uint16_t)JSDK_JF_SCALE_INVALID;
    j->fb.status_flags = j->status_flags;

    (void)jsdk_ctx_send(j->ctx, CB_PRI_QUERY, CB_MSG_QUERY_STATUS,
                        j->cfg.node_id, NULL, 0u);
    j->ctx->tx_seq = cb_seq_next(j->ctx->tx_seq);

    jsdk_ctx_seterr(j->ctx, "node %u reset; local state cleared, re-handshake sent",
                    (unsigned)j->cfg.node_id);
    return JSDK_OK;
}

jsdk_status_t jsdk_joint_set_node_id(jsdk_joint_t *j, uint8_t new_id, int persist)
{
    uint8_t  payload[1];
    uint32_t t0;
    unsigned spin = 0u;
    uint8_t  old_id;

    if (!jsdk_joint_check(j)) return JSDK_ERR_INVALID_ARG;
    if (new_id < CB_NODE_ID_MIN || new_id > CB_NODE_ID_MAX) {
        jsdk_joint_seterr(j, "node id %u out of range 1..254", (unsigned)new_id);
        return JSDK_ERR_INVALID_ARG;
    }
    /* 改节点号会让本总线上已有的其它关节冲突 → 提前拦，而不是让总线打架 */
    {
        unsigned i;
        for (i = 0u; i < j->ctx->nj; ++i) {
            const jsdk_joint_t *o = &j->ctx->joints[i];
            if (o != j && o->cfg.node_id == new_id) {
                jsdk_joint_seterr(j, "node id %u already used by joint %u",
                                  (unsigned)new_id, (unsigned)o->index);
                return JSDK_ERR_INVALID_ARG;
            }
        }
    }

    old_id = j->cfg.node_id;
    payload[0] = new_id;

    /*
     * ⚠ **改号前**必须先问一句「目标地址上是否已经有人应答」。
     *
     * 没有这一步时的真实故障（v0.19 查到）：总线上若已存在一个同号设备，
     * 下面那段「验证新地址可应答」会被**原来那个设备**满足 → SDK 报改号成功，
     * 而实际上总线上现在有两个同号设备（谁先答谁被认，行为不确定）。
     * 多轴机柜上这是很容易踩到的：CLI/Python 只往上下文里加自己关心的那个关节，
     * 因此本函数开头那个「本上下文内冲突」检查对总线上的其它设备一无所知。
     */
    if (new_id != old_id && jsdk_ctx_probe_node(j->ctx, new_id)) {
        jsdk_joint_seterr(j, "node %u already answers on the bus; renaming %u -> %u "
                             "would create a duplicate id",
                          (unsigned)new_id, (unsigned)old_id, (unsigned)new_id);
        return JSDK_ERR_INVALID_ARG;
    }
    if (jsdk_ctx_send(j->ctx, CB_PRI_CTRL, CB_MSG_SET_NODE_ID,
                      (uint8_t)old_id, payload, 1u) != 0) {
        return JSDK_ERR_TRANSPORT;
    }
    j->ctx->tx_seq = cb_seq_next(j->ctx->tx_seq);

    /* 协议不落 Flash；persist=1 时补一次 CONFIG_SAVE（**在旧地址上**，还没验证新号） */
    if (persist) {
        (void)jsdk_ctx_send(j->ctx, CB_PRI_CONFIG, CB_MSG_CONFIG_SAVE,
                            old_id, NULL, 0u);
        j->ctx->tx_seq = cb_seq_next(j->ctx->tx_seq);
    }

    /* 验证：新地址必须能应答，否则不要把本地 node_id 改过去（否则失联） */
    t0 = j->ctx->cfg.hal.now_ms(j->ctx->cfg.hal.user);
    for (;;) {
        jsdk_can_frame_t rsp;
        if (jsdk_ctx_send(j->ctx, CB_PRI_QUERY, CB_MSG_QUERY_STATUS,
                          new_id, NULL, 0u) == 0) {
            j->ctx->tx_seq = cb_seq_next(j->ctx->tx_seq);
            if (jsdk_ctx_wait_response(j->ctx, CB_MSG_MIT_CONTROL, new_id, &rsp,
                                       JSDK_CFG_TIMEOUT_MS) == JSDK_OK) {
                j->cfg.node_id = new_id;
                j->node_id_readback = new_id;
                jsdk_ctx_seterr(j->ctx, "node id changed %u -> %u%s",
                                (unsigned)old_id, (unsigned)new_id,
                                persist ? " (persisted)" : " (RAM only)");
                return JSDK_OK;
            }
        }
        j->ctx->now_ms = j->ctx->cfg.hal.now_ms(j->ctx->cfg.hal.user);
        if (jsdk_elapsed(j->ctx->now_ms, t0) >= JSDK_CFG_TIMEOUT_MS) break;
        if (++spin >= 40000u) break;
    }

    jsdk_joint_seterr(j, "node %u did not answer after SET_NODE_ID -> %u; "
                         "local id left unchanged (use the old id to retry)",
                      (unsigned)old_id, (unsigned)new_id);
    return JSDK_ERR_TIMEOUT;
}

jsdk_status_t jsdk_joint_set_watchdog_ms(jsdk_joint_t *j, uint32_t ms)
{
    uint8_t be[2];
    int     warn_zero;

    if (!jsdk_joint_check(j)) return JSDK_ERR_INVALID_ARG;
    if (j->ep_break_timeout == 0u) {
        jsdk_joint_seterr(j, "can.config.break_timeout endpoint unavailable");
        return JSDK_ERR_NOT_FOUND;
    }
    if (ms > 65535u) {
        /* 端点只有 u16；静默截断会让客户以为设成了更大的值 */
        jsdk_joint_seterr(j, "break_timeout max is 65535 ms (got %u)", (unsigned)ms);
        return JSDK_ERR_INVALID_ARG;
    }
    warn_zero = (ms == 0u);

    /* ⚠ 参数值是**大端**（与全协议一致）。直接传主机序 u16 会字节交换，
       设备把 250 读成 64000 —— 本项目确实胉过这个坑。 */
    cb_be_put_u16(be, (uint16_t)ms);
    if (jsdk_ctx_write_param(j->ctx, j->cfg.node_id, j->ep_break_timeout,
                             be, 2u, 0u) != JSDK_OK) {
        jsdk_joint_seterr(j, "failed to write can.config.break_timeout");
        return JSDK_ERR_TRANSPORT;
    }

    /* 读回校验（端点可读，没必要盲信写入） */
    {
        uint8_t buf[8];
        uint8_t len = 0u;
        if (jsdk_ctx_read_param(j->ctx, j->cfg.node_id, j->ep_break_timeout,
                                buf, &len, 0u) == JSDK_OK && len >= 2u) {
            j->break_timeout_ms = cb_be_get_u16(buf);
            if (j->break_timeout_ms != ms) {
                jsdk_joint_seterr(j, "break_timeout read back as %u, expected %u",
                                  (unsigned)j->break_timeout_ms, (unsigned)ms);
                return JSDK_ERR_PROTOCOL;
            }
        } else {
            j->break_timeout_ms = ms;   /* 读不回来就用写入值 */
        }
    }

    /* ⚠ 0 **不是**关闭：固件按 100 ms 处理。这条警告必须活到最后 ——
       早先的写法是 write 前发警告、成功后又被成功串覆盖，客户就看不到了。 */
    jsdk_ctx_seterr(j->ctx,
        "break_timeout set to %u ms on node %u%s (not persisted; call save_config())",
        (unsigned)ms, (unsigned)j->cfg.node_id,
        warn_zero ? "; WARNING: 0 means 100 ms, NOT disabled" : "");
    return JSDK_OK;
}

/* ==========================================================================
 * 只读便捷量
 * ======================================================================== */

int jsdk_joint_get_can_state(const jsdk_joint_t *j)
{
    if (!jsdk_joint_check(j)) return -1;
    if (!j->state_known) return -1;
    return (int)j->current_state_raw;
}

/* ======================== src/core/jsdk_param.c ======================== */
/**
 * @file    jsdk_param.c
 * @brief   WP5 参数访问：类型化 get/set、批量读、SDO 风格槽位
 *
 * @par 为什么“批量读”必须能自动退化
 *  固件对 **Classic 帧**的批量请求**恒回 2 字节 ERR**（`cb_param_pack_batch_err`）——
 *  这是固件行为，不是我们没实现。所以 SDK 的策略是：
 *    - FD   → 按各端点类型长度装箱，尽量一帧装完（最多 31 条 / 62 字节预算）
 *    - Classic → 直接逐条单读，**不让客户看到 ERR**
 *  客户只调一次 `jsdk_joint_param_get_batch()`，由 SDK 决定走哪条路。
 *
 * @par 参数的字节序
 *  参数值是**大端**（与全协议一致，JSON 描述符是唯一例外）。读到调用方缓冲后
 *  由本文件负责按端点类型宽度解成主机序的 `jsdk_value_t`。
 */



/* ==========================================================================
 * 名字 → 端点信息
 * ======================================================================== */

/** 解析路径；失败时写错误串并返回状态码。 */
static jsdk_status_t resolve(jsdk_joint_t *j, const char *path,
                             uint16_t *ep_id, jsdk_ep_type_t *type, uint8_t *access)
{
    jsdk_status_t st;

    if (!jsdk_joint_check(j) || !path) return JSDK_ERR_INVALID_ARG;
    if (!j->ctx->desc_present) {
        jsdk_joint_seterr(j, "descriptor not available: configure() first");
        return JSDK_ERR_BAD_STATE;
    }

    st = jsdk_ep_store_lookup(&j->ctx->store, path, ep_id, type, access);
    if (st != JSDK_OK) {
        /* 不猜、不近似：未命中就是未命中 */
        jsdk_joint_seterr(j, "endpoint not found: %s", path);
    }
    return st;
}

/* ==========================================================================
 * 大端窄宽度 ↔ 主机序
 * ======================================================================== */

static void be_to_value(jsdk_ep_type_t t, const uint8_t *b, jsdk_value_t *out)
{
    memset(out, 0, sizeof *out);
    out->type = t;

    switch (t) {
    case JSDK_EP_U8:  out->v.u8  = b[0]; break;
    case JSDK_EP_I8:  out->v.i8  = (int8_t)b[0]; break;
    case JSDK_EP_BOOL: out->v.boolean = (b[0] != 0u) ? 1 : 0; break;
    case JSDK_EP_U16: out->v.u16 = cb_be_get_u16(b); break;
    case JSDK_EP_I16: out->v.i16 = cb_be_get_i16(b); break;
    case JSDK_EP_U32: out->v.u32 = cb_be_get_u32(b); break;
    case JSDK_EP_I32: out->v.i32 = cb_be_get_i32(b); break;
    case JSDK_EP_U64: out->v.u64 = cb_be_get_u64(b); break;
    case JSDK_EP_I64: out->v.i64 = (int64_t)cb_be_get_u64(b); break;
    case JSDK_EP_F32: out->v.f32 = cb_be_get_f32(b); break;
    case JSDK_EP_F64: {
        /* 协议里没有 f64 线格式；按两个 u32 拼（保留位，仅用于透传） */
        uint64_t hi = cb_be_get_u32(b);
        uint64_t lo = cb_be_get_u32(b + 4);
        uint64_t raw = (hi << 32) | lo;
        double d;
        memcpy(&d, &raw, sizeof d);      /* 位模式搬运，不做数值转换 */
        out->v.f64 = d;
        break;
    }
    default:
        break;                            /* 不透明类型：调用方不该走到这里 */
    }
}

static int value_to_be(jsdk_ep_type_t t, const jsdk_value_t *in,
                       uint8_t *b, uint8_t *out_len)
{
    unsigned w = jsdk_ep_type_size(t);

    if (w == 0u) return -1;
    memset(b, 0, 8u);

    switch (t) {
    case JSDK_EP_U8:   b[0] = in->v.u8; break;
    case JSDK_EP_I8:   b[0] = (uint8_t)in->v.i8; break;
    case JSDK_EP_BOOL: b[0] = (uint8_t)(in->v.boolean ? 1 : 0); break;
    case JSDK_EP_U16:  cb_be_put_u16(b, in->v.u16); break;
    case JSDK_EP_I16:  cb_be_put_i16(b, in->v.i16); break;
    case JSDK_EP_U32:  cb_be_put_u32(b, in->v.u32); break;
    case JSDK_EP_I32:  cb_be_put_i32(b, in->v.i32); break;
    case JSDK_EP_U64:  cb_be_put_u64(b, in->v.u64); break;
    case JSDK_EP_I64:  cb_be_put_u64(b, (uint64_t)in->v.i64); break;
    case JSDK_EP_F32:  cb_be_put_f32(b, in->v.f32); break;
    case JSDK_EP_F64: {
        uint64_t raw;
        memcpy(&raw, &in->v.f64, sizeof raw);
        cb_be_put_u32(b, (uint32_t)(raw >> 32));
        cb_be_put_u32(b + 4, (uint32_t)raw);
        break;
    }
    default:
        return -1;
    }

    *out_len = (uint8_t)w;
    return 0;
}

/* ==========================================================================
 * 类型化 get / set
 * ======================================================================== */

jsdk_status_t jsdk_joint_param_get(jsdk_joint_t *j, const char *path,
                                   jsdk_value_t *out)
{
    uint16_t ep = 0u;
    jsdk_ep_type_t t = JSDK_EP_JSON;
    uint8_t  access = 0u;
    uint8_t  buf[8];
    uint8_t  len = 0u;
    uint8_t  need;
    jsdk_status_t st;

    if (!out) return JSDK_ERR_INVALID_ARG;
    st = resolve(j, path, &ep, &t, &access);
    if (st != JSDK_OK) return st;
    if (!(access & JSDK_EP_ACCESS_R)) {
        jsdk_joint_seterr(j, "endpoint is write-only: %s", path);
        return JSDK_ERR_UNSUPPORTED;
    }
    need = (uint8_t)jsdk_ep_type_size(t);
    if (need == 0u) {
        jsdk_joint_seterr(j, "endpoint type is not directly readable: %s (%s)",
                          path, jsdk_ep_type_string(t));
        return JSDK_ERR_UNSUPPORTED;
    }

    st = (jsdk_status_t)jsdk_ctx_read_param_exact(j->ctx, j->cfg.node_id, ep,
                                                  buf, need, &len, 0u);
    if (st != JSDK_OK) {
        jsdk_joint_seterr(j, "read %s failed (%s)", path, jsdk_status_string(st));
        return st;
    }
    if (len != need) {
        /* read_param_exact 成功时保证 len == need；走到这里说明契约被破了 */
        jsdk_joint_seterr(j, "read %s: got %u bytes, expected %u",
                          path, (unsigned)len, (unsigned)need);
        return JSDK_ERR_PROTOCOL;
    }

    be_to_value(t, buf, out);
    return JSDK_OK;
}

jsdk_status_t jsdk_joint_param_set(jsdk_joint_t *j, const char *path,
                                   const jsdk_value_t *in)
{
    uint16_t ep = 0u;
    jsdk_ep_type_t t = JSDK_EP_JSON;
    uint8_t  access = 0u;
    uint8_t  buf[8];
    uint8_t  len = 0u;
    jsdk_status_t st;

    if (!in) return JSDK_ERR_INVALID_ARG;
    st = resolve(j, path, &ep, &t, &access);
    if (st != JSDK_OK) return st;
    if (!(access & JSDK_EP_ACCESS_W)) {
        jsdk_joint_seterr(j, "endpoint is read-only: %s", path);
        return JSDK_ERR_UNSUPPORTED;
    }
    if (in->type != t) {
        /* 类型不匹配必须报错：猜宽度会静默写坏邻近字段 */
        jsdk_joint_seterr(j, "type mismatch for %s: descriptor=%s given=%s",
                          path, jsdk_ep_type_string(t),
                          jsdk_ep_type_string(in->type));
        return JSDK_ERR_PROTOCOL;
    }
    if (value_to_be(t, in, buf, &len) != 0) {
        return JSDK_ERR_UNSUPPORTED;
    }

    st = (jsdk_status_t)jsdk_ctx_write_param(j->ctx, j->cfg.node_id, ep,
                                             buf, len, 0u);
    if (st != JSDK_OK) {
        jsdk_joint_seterr(j, "write %s failed (%s)", path, jsdk_status_string(st));
    }
    return st;
}

/* ==========================================================================
 * 便捷包装
 * ======================================================================== */

/** 共用的“读 + 按目标类型取值”。 */
static jsdk_status_t get_scalar(jsdk_joint_t *j, const char *path,
                                jsdk_value_t *v)
{
    return jsdk_joint_param_get(j, path, v);
}

jsdk_status_t jsdk_joint_param_get_f32(jsdk_joint_t *j, const char *path, float *out)
{
    jsdk_value_t v;
    jsdk_status_t st;

    if (!out) return JSDK_ERR_INVALID_ARG;
    st = get_scalar(j, path, &v);
    if (st != JSDK_OK) return st;
    switch (v.type) {
    case JSDK_EP_F32: *out = v.v.f32; return JSDK_OK;
    case JSDK_EP_U8:  *out = (float)v.v.u8;  return JSDK_OK;
    case JSDK_EP_I8:  *out = (float)v.v.i8;  return JSDK_OK;
    case JSDK_EP_U16: *out = (float)v.v.u16; return JSDK_OK;
    case JSDK_EP_I16: *out = (float)v.v.i16; return JSDK_OK;
    case JSDK_EP_U32: *out = (float)v.v.u32; return JSDK_OK;
    case JSDK_EP_I32: *out = (float)v.v.i32; return JSDK_OK;
    case JSDK_EP_BOOL: *out = (float)v.v.boolean; return JSDK_OK;
    default:
        jsdk_joint_seterr(j, "%s is %s, not a float", path, jsdk_ep_type_string(v.type));
        return JSDK_ERR_PROTOCOL;
    }
}

jsdk_status_t jsdk_joint_param_set_f32(jsdk_joint_t *j, const char *path, float v)
{
    jsdk_value_t in;

    memset(&in, 0, sizeof in);
    in.type = JSDK_EP_F32;
    in.v.f32 = v;
    return jsdk_joint_param_set(j, path, &in);
}

jsdk_status_t jsdk_joint_param_get_u32(jsdk_joint_t *j, const char *path, uint32_t *out)
{
    jsdk_value_t v;
    jsdk_status_t st;

    if (!out) return JSDK_ERR_INVALID_ARG;
    st = get_scalar(j, path, &v);
    if (st != JSDK_OK) return st;
    switch (v.type) {
    case JSDK_EP_U32: *out = v.v.u32; return JSDK_OK;
    case JSDK_EP_U16: *out = v.v.u16; return JSDK_OK;
    case JSDK_EP_U8:  *out = v.v.u8;  return JSDK_OK;
    case JSDK_EP_BOOL: *out = (uint32_t)v.v.boolean; return JSDK_OK;
    default:
        jsdk_joint_seterr(j, "%s is %s, not an unsigned integer",
                          path, jsdk_ep_type_string(v.type));
        return JSDK_ERR_PROTOCOL;
    }
}

jsdk_status_t jsdk_joint_param_set_u32(jsdk_joint_t *j, const char *path, uint32_t v)
{
    jsdk_value_t in;
    uint16_t ep = 0u;
    jsdk_ep_type_t t = JSDK_EP_JSON;
    uint8_t access = 0u;

    /* 目标端点的**真实宽度**由描述符决定：给 u32 值写 u16 端点要按宽度降级，
       但绝不能溢出（超出范围由固件截断） */
    if (resolve(j, path, &ep, &t, &access) != JSDK_OK) return JSDK_ERR_NOT_FOUND;

    memset(&in, 0, sizeof in);
    in.type = t;
    switch (t) {
    case JSDK_EP_U8:  in.v.u8  = (uint8_t)v;  break;
    case JSDK_EP_U16: in.v.u16 = (uint16_t)v; break;
    case JSDK_EP_U32: in.v.u32 = v;           break;
    case JSDK_EP_BOOL: in.v.boolean = (v != 0u) ? 1 : 0; break;
    default:
        jsdk_joint_seterr(j, "%s is %s; use the typed param_set() instead",
                          path, jsdk_ep_type_string(t));
        return JSDK_ERR_PROTOCOL;
    }
    return jsdk_joint_param_set(j, path, &in);
}

jsdk_status_t jsdk_joint_param_get_i32(jsdk_joint_t *j, const char *path, int32_t *out)
{
    jsdk_value_t v;
    jsdk_status_t st;

    if (!out) return JSDK_ERR_INVALID_ARG;
    st = get_scalar(j, path, &v);
    if (st != JSDK_OK) return st;
    switch (v.type) {
    case JSDK_EP_I32: *out = v.v.i32; return JSDK_OK;
    case JSDK_EP_I16: *out = v.v.i16; return JSDK_OK;
    case JSDK_EP_I8:  *out = v.v.i8;  return JSDK_OK;
    case JSDK_EP_U32: *out = (int32_t)v.v.u32; return JSDK_OK;
    default:
        jsdk_joint_seterr(j, "%s is %s, not a signed integer",
                          path, jsdk_ep_type_string(v.type));
        return JSDK_ERR_PROTOCOL;
    }
}

jsdk_status_t jsdk_joint_param_get_bool(jsdk_joint_t *j, const char *path, int *out)
{
    jsdk_value_t v;
    jsdk_status_t st;

    if (!out) return JSDK_ERR_INVALID_ARG;
    st = get_scalar(j, path, &v);
    if (st != JSDK_OK) return st;
    if (v.type == JSDK_EP_BOOL) { *out = v.v.boolean; return JSDK_OK; }
    if (v.type == JSDK_EP_U8)   { *out = (v.v.u8 != 0u) ? 1 : 0; return JSDK_OK; }
    jsdk_joint_seterr(j, "%s is %s, not a bool", path, jsdk_ep_type_string(v.type));
    return JSDK_ERR_PROTOCOL;
}

/* ==========================================================================
 * 批量读
 * ======================================================================== */

/** Classic：自动退化为逐条单读（固件对 Classic 批量请求恒回 ERR）。 */
static void batch_serial(jsdk_joint_t *j, jsdk_param_req_t *reqs, unsigned n)
{
    unsigned i;

    for (i = 0u; i < n; ++i) {
        if (!reqs[i].path) { reqs[i].status = JSDK_ERR_INVALID_ARG; continue; }
        reqs[i].status = jsdk_joint_param_get(j, reqs[i].path, &reqs[i].value);
    }
}

jsdk_status_t jsdk_joint_param_get_batch(jsdk_joint_t *j, jsdk_param_req_t *reqs,
                                        unsigned n)
{
    cb_param_batch_item_t plan[CB_PARAM_MAX_BATCH];
    unsigned  idx[CB_PARAM_MAX_BATCH];
    jsdk_ep_type_t types[CB_PARAM_MAX_BATCH];
    unsigned  n_plan = 0u;
    unsigned  i;

    if (!jsdk_joint_check(j) || (!reqs && n > 0u)) return JSDK_ERR_INVALID_ARG;
    if (n == 0u) return JSDK_OK;
    if (!j->ctx->desc_present) return JSDK_ERR_BAD_STATE;

    for (i = 0u; i < n; ++i) reqs[i].status = JSDK_ERR_NOT_FOUND;

    /* Classic：固件对批量请求**恒回 ERR**（不是我们没实现），
       所以别浪费一次往返 —— 直接逐条单读，让客户看不到 ERR。 */
    if (!j->ctx->cfg.is_fd) {
        batch_serial(j, reqs, n);
        return JSDK_OK;
    }

    /* --- 1. 解析：能打包的进 plan，不能的（未命中/只写/不透明）留在原地 --- */
    for (i = 0u; i < n && n_plan < CB_PARAM_MAX_BATCH; ++i) {
        uint16_t ep = 0u;
        jsdk_ep_type_t t = JSDK_EP_JSON;
        uint8_t access = 0u;
        uint8_t w;

        if (!reqs[i].path) { reqs[i].status = JSDK_ERR_INVALID_ARG; continue; }
        if (resolve(j, reqs[i].path, &ep, &t, &access) != JSDK_OK) continue;
        if (!(access & JSDK_EP_ACCESS_R)) {
            reqs[i].status = JSDK_ERR_UNSUPPORTED;
            continue;
        }
        w = (uint8_t)jsdk_ep_type_size(t);
        if (w == 0u) { reqs[i].status = JSDK_ERR_UNSUPPORTED; continue; }

        plan[n_plan].ep_id     = ep;
        plan[n_plan].value_len = w;
        idx[n_plan]   = i;
        types[n_plan] = t;
        n_plan++;
    }

    /* --- 2. 装箱（budget = 单帧 64 B）并逐批收发 --- */
    if (n_plan > 0u) {
        size_t counts[CB_PARAM_MAX_BATCH];
        size_t offsets[CB_PARAM_MAX_BATCH];
        size_t n_batches;
        size_t b;
        size_t base = 0u;

        n_batches = cb_param_plan_batches(plan, n_plan, CB_PARAM_FD_FRAME_MAX,
                                          counts, offsets, CB_PARAM_MAX_BATCH);
        if (n_batches == 0u) {
            /* 有条目连单帧都装不下（值 ≤ 8 B，正常不会发生）→ 老实逐条 */
            batch_serial(j, reqs, n);
            return JSDK_OK;
        }

        for (b = 0u; b < n_batches; ++b) {
            uint8_t  req[CB_PARAM_FD_FRAME_MAX];
            jsdk_can_frame_t frame;
            uint16_t eps[CB_PARAM_MAX_BATCH];   /* ep_id 是 u16，别收窄成 u8 */
            size_t   cnt = counts[b];
            size_t   k;
            size_t   rq_len;

            for (k = 0u; k < cnt; ++k) eps[k] = plan[base + k].ep_id;

            rq_len = cb_param_pack_batch_req(req, sizeof req, eps, (uint8_t)cnt);
            if (rq_len == 0u) { batch_serial(j, reqs, n); return JSDK_OK; }

            if (jsdk_ctx_send(j->ctx, CB_PRI_CONFIG, CB_MSG_PARAM_READ,
                              j->cfg.node_id, req, (uint8_t)rq_len) != 0) {
                batch_serial(j, reqs, n);
                return JSDK_OK;
            }
            j->ctx->tx_seq = cb_seq_next(j->ctx->tx_seq);

            if (jsdk_ctx_wait_response(j->ctx, CB_MSG_PARAM_READ, j->cfg.node_id,
                                       &frame, JSDK_CFG_TIMEOUT_MS) != JSDK_OK) {
                batch_serial(j, reqs, n);
                return JSDK_OK;
            }

            /* --- 3. 解析响应；形状不对就整批逐条兜底 ---
               ⚠ 绝不把“错位的字节”当成值用：批量响应是**值流 + 位图**，
                  一旦位图或长度对不上，后续所有值都会整体滑动错位。 */
            {
                cb_param_batch_rsp_t rsp;
                size_t off = 0u;
                int usable = 1;

                if (cb_param_unpack_batch_rsp(frame.data, frame.len,
                                              (uint8_t)cnt, &rsp) != 0
                    || rsp.is_err) {
                    usable = 0;
                }

                if (usable) {
                    for (k = 0u; k < cnt; ++k) {
                        unsigned w = plan[base + k].value_len;
                        unsigned slot = idx[base + k];

                        if (!cb_param_bitmap_test(rsp.bitmap, rsp.bitmap_bytes,
                                                  (uint8_t)k)) {
                            reqs[slot].status = JSDK_ERR_NOT_FOUND;  /* 设备说没取到 */
                            continue;
                        }
                        if (off + w > rsp.values_len) { usable = 0; break; }
                        be_to_value(types[base + k], rsp.values + off,
                                    &reqs[slot].value);
                        reqs[slot].status = JSDK_OK;
                        off += w;
                    }
                }

                if (!usable) {
                    for (k = 0u; k < cnt; ++k) {
                        unsigned slot = idx[base + k];
                        reqs[slot].status = jsdk_joint_param_get(j, reqs[slot].path,
                                                                 &reqs[slot].value);
                    }
                }
            }

            base += cnt;
        }
    }

    return JSDK_OK;
}

/* ==========================================================================
 * SDO 风格槽位
 * ======================================================================== */

jsdk_sdo_handle_t jsdk_joint_sdo_create(jsdk_joint_t *j, uint16_t ep_id,
                                        uint8_t subindex, size_t size)
{
    unsigned i;
    jsdk_ep_type_t t = JSDK_EP_JSON;
    uint8_t access = 0u;

    if (!jsdk_joint_check(j)) return -1;
    /* 本协议的 subindex 恒为 0（端点 ID 是平铺的，没有子索引概念） */
    if (subindex != 0u) {
        jsdk_joint_seterr(j, "subindex must be 0 for CYBERBEAST (flat endpoint ids)");
        return -1;
    }
    if (size == 0u) {
        /* 从描述符推断宽度 */
        unsigned n = jsdk_ep_store_count(&j->ctx->store);
        for (i = 0u; i < n; ++i) {
            uint16_t id = 0u;
            if (jsdk_ep_store_at(&j->ctx->store, i, NULL, &id, &t, &access) != JSDK_OK) {
                continue;
            }
            if (id == ep_id) break;
        }
        if (i >= n) {
            jsdk_joint_seterr(j, "endpoint %u not in descriptor", (unsigned)ep_id);
            return -1;
        }
        size = jsdk_ep_type_size(t);
        if (size == 0u || size > 8u) {
            jsdk_joint_seterr(j, "endpoint %u has no scalar width", (unsigned)ep_id);
            return -1;
        }
    }
    if (size > 8u) return -1;

    /* 槽位 0 保留给故障详情自动读取 */
    for (i = 1u; i < JSDK_SDO_SLOTS; ++i) {
        if (!j->sdo[i].in_use) {
            j->sdo[i].in_use = 1u;
            j->sdo[i].ep_id  = ep_id;
            j->sdo[i].size   = (uint16_t)size;
            j->sdo[i].state  = (uint8_t)JSDK_SDO_IDLE;
            memset(j->sdo[i].data, 0, sizeof j->sdo[i].data);
            return (jsdk_sdo_handle_t)i;
        }
    }
    jsdk_joint_seterr(j, "no free SDO slot (max %u per joint)", (unsigned)JSDK_SDO_SLOTS);
    return -1;
}

jsdk_sdo_handle_t jsdk_joint_sdo_create_by_name(jsdk_joint_t *j, const char *path)
{
    uint16_t ep = 0u;
    jsdk_ep_type_t t = JSDK_EP_JSON;
    uint8_t access = 0u;

    if (resolve(j, path, &ep, &t, &access) != JSDK_OK) return -1;
    return jsdk_joint_sdo_create(j, ep, 0u, jsdk_ep_type_size(t));
}

static jsdk_sdo_slot_t *sdo_slot(jsdk_joint_t *j, jsdk_sdo_handle_t h)
{
    if (!jsdk_joint_check(j)) return NULL;
    if (h <= 0 || (unsigned)h >= JSDK_SDO_SLOTS) return NULL;
    if (!j->sdo[h].in_use) return NULL;
    return &j->sdo[h];
}

jsdk_sdo_state_t jsdk_joint_sdo_state(jsdk_joint_t *j, jsdk_sdo_handle_t h)
{
    jsdk_sdo_slot_t *s = sdo_slot(j, h);
    return s ? (jsdk_sdo_state_t)s->state : JSDK_SDO_ERROR;
}

uint8_t *jsdk_joint_sdo_data(jsdk_joint_t *j, jsdk_sdo_handle_t h)
{
    jsdk_sdo_slot_t *s = sdo_slot(j, h);
    return s ? s->data : NULL;
}

size_t jsdk_joint_sdo_data_size(jsdk_joint_t *j, jsdk_sdo_handle_t h)
{
    jsdk_sdo_slot_t *s = sdo_slot(j, h);
    return s ? (size_t)s->size : 0u;
}

int jsdk_joint_sdo_read(jsdk_joint_t *j, jsdk_sdo_handle_t h)
{
    jsdk_sdo_slot_t *s = sdo_slot(j, h);
    uint8_t buf[8];
    uint8_t len = 0u;
    int rc;

    if (!s) return JSDK_ERR_INVALID_ARG;
    s->state = (uint8_t)JSDK_SDO_BUSY;

    /* SDO 的 size ≤ 8（见 jsdk_joint_sdo_create），所以精确读一定能读满；
       Classic 下这段会自己分两块（每块 4 B），不需要调用方关心。 */
    rc = jsdk_ctx_read_param_exact(j->ctx, j->cfg.node_id, s->ep_id,
                                   buf, (uint8_t)s->size, &len, 0u);
    if (rc != JSDK_OK || len < (uint8_t)s->size) {
        s->state = (uint8_t)JSDK_SDO_ERROR;
        return (rc != JSDK_OK) ? rc : JSDK_ERR_PROTOCOL;
    }
    memcpy(s->data, buf, s->size);
    s->state = (uint8_t)JSDK_SDO_SUCCESS;
    return JSDK_OK;
}

int jsdk_joint_sdo_write(jsdk_joint_t *j, jsdk_sdo_handle_t h)
{
    jsdk_sdo_slot_t *s = sdo_slot(j, h);
    jsdk_ep_type_t t = JSDK_EP_JSON;
    uint8_t access = 0u;
    unsigned n, i;
    int rc;

    if (!s) return JSDK_ERR_INVALID_ARG;

    /* 写之前必须确认端点可写（描述符里带 w 权限） */
    n = jsdk_ep_store_count(&j->ctx->store);
    for (i = 0u; i < n; ++i) {
        uint16_t id = 0u;
        if (jsdk_ep_store_at(&j->ctx->store, i, NULL, &id, &t, &access) != JSDK_OK) {
            continue;
        }
        if (id == s->ep_id) break;
    }
    if (i >= n || !(access & JSDK_EP_ACCESS_W)) {
        s->state = (uint8_t)JSDK_SDO_ERROR;
        jsdk_joint_seterr(j, "endpoint %u is not writable", (unsigned)s->ep_id);
        return JSDK_ERR_UNSUPPORTED;
    }

    s->state = (uint8_t)JSDK_SDO_BUSY;
    rc = jsdk_ctx_write_param(j->ctx, j->cfg.node_id, s->ep_id,
                              s->data, (uint8_t)s->size, 0u);
    s->state = (uint8_t)((rc == JSDK_OK) ? JSDK_SDO_SUCCESS : JSDK_SDO_ERROR);
    return rc;
}

/* ======================== src/core/jsdk_text.c ======================== */
/**
 * @file    jsdk_text.c
 * @brief   文本与 ABI 标识（纯 rodata，零依赖，可在任何上下文调用）
 *
 * 全部函数都接受**任意**输入：未知枚举返回 "unknown(...)" 而不是 NULL，
 * 这样日志/CLI 不必到处判空。
 */


/* ==========================================================================
 * ABI 守卫
 * ======================================================================== */

const char *jsdk_backend_name(void)
{
    return JSDK_BACKEND_NAME_CAN;
}

uint32_t jsdk_abi_version(void)
{
    return JSDK_ABI_VERSION_CAN;
}

/* --------------------------------------------------------------------------
 * 结构体尺寸/对齐表（FFI 自检）
 *
 * 为什么需要：Python(ctypes)/C#/Rust 等 FFI 绑定必须**复刻**这些结构体的
 * 字段顺序与对齐。复刻错了不会报错，而是**直接踩内存** —— 症状是"值偶尔不对"
 * 或随机崩溃，现场几乎查不出来。
 *
 * 有了这张表，绑定方可以在导入时逐项比对，不一致就立刻抛出
 * "ABI 不匹配"（并指出是哪个类型），而不是等到跑起来才出事。
 *
 * 表**只增不改**：改字段顺序必须同时改这里，而这会让所有绑定在导入时立刻失败 ——
 * 这正是我们想要的信号（ABI 变了）。
 * ------------------------------------------------------------------------ */

/** 用"结构体里紧跟在 char 之后的成员偏移"求对齐（C99 可移植做法）。 */
#define JSDK_ALIGNOF(T) ((uint32_t)offsetof(struct { char c; T t; }, t))

static const jsdk_abi_type_t k_abi_types[] = {
    { "jsdk_can_frame_t",     (uint32_t)sizeof(jsdk_can_frame_t),     JSDK_ALIGNOF(jsdk_can_frame_t) },
    { "jsdk_can_hal_t",       (uint32_t)sizeof(jsdk_can_hal_t),       JSDK_ALIGNOF(jsdk_can_hal_t) },
    { "jsdk_context_config_t",(uint32_t)sizeof(jsdk_context_config_t),JSDK_ALIGNOF(jsdk_context_config_t) },
    { "jsdk_desc_config_t",   (uint32_t)sizeof(jsdk_desc_config_t),   JSDK_ALIGNOF(jsdk_desc_config_t) },
    { "jsdk_joint_config_t",  (uint32_t)sizeof(jsdk_joint_config_t),  JSDK_ALIGNOF(jsdk_joint_config_t) },
    { "jsdk_joint_config_snapshot_t",
      (uint32_t)sizeof(jsdk_joint_config_snapshot_t), JSDK_ALIGNOF(jsdk_joint_config_snapshot_t) },
    { "jsdk_joint_feedback_t",(uint32_t)sizeof(jsdk_joint_feedback_t),JSDK_ALIGNOF(jsdk_joint_feedback_t) },
    { "jsdk_bus_state_t",     (uint32_t)sizeof(jsdk_bus_state_t),     JSDK_ALIGNOF(jsdk_bus_state_t) },
    { "jsdk_device_info_t",   (uint32_t)sizeof(jsdk_device_info_t),   JSDK_ALIGNOF(jsdk_device_info_t) },
    { "jsdk_fault_info_t",    (uint32_t)sizeof(jsdk_fault_info_t),    JSDK_ALIGNOF(jsdk_fault_info_t) },
    { "jsdk_value_t",         (uint32_t)sizeof(jsdk_value_t),         JSDK_ALIGNOF(jsdk_value_t) },
    { "jsdk_unit_scale_t",    (uint32_t)sizeof(jsdk_unit_scale_t),    JSDK_ALIGNOF(jsdk_unit_scale_t) },
    { "jsdk_param_req_t",     (uint32_t)sizeof(jsdk_param_req_t),     JSDK_ALIGNOF(jsdk_param_req_t) },
    { "jsdk_group_target_t",  (uint32_t)sizeof(jsdk_group_target_t),  JSDK_ALIGNOF(jsdk_group_target_t) },
    { "jsdk_desc_info_t",     (uint32_t)sizeof(jsdk_desc_info_t),     JSDK_ALIGNOF(jsdk_desc_info_t) }
};

const jsdk_abi_type_t *jsdk_abi_types(size_t *count_out)
{
    if (count_out) *count_out = sizeof k_abi_types / sizeof k_abi_types[0];
    return k_abi_types;
}

/* ==========================================================================
 * 状态码
 * ======================================================================== */

const char *jsdk_status_string(jsdk_status_t status)
{
    switch (status) {
    case JSDK_OK:              return "ok";
    case JSDK_ERR_INVALID_ARG: return "invalid-argument";
    case JSDK_ERR_NO_MEMORY:   return "no-memory";
    case JSDK_ERR_NOT_FOUND:   return "not-found";
    case JSDK_ERR_BAD_STATE:   return "bad-state";
    case JSDK_ERR_TRANSPORT:   return "transport-error";
    case JSDK_ERR_UNSUPPORTED: return "unsupported";
    case JSDK_ERR_TIMEOUT:     return "timeout";
    case JSDK_ERR_PROTOCOL:    return "protocol-error";
    case JSDK_ERR_BUSY:        return "busy";
    case JSDK_ERR_PARSE:       return "parse-error";
    default:                   return "unknown-status";
    }
}

/* ==========================================================================
 * 关节状态
 * ------------------------------------------------------------------------
 * CAN 侧只有 IDLE / CLOSED_LOOP 等少数状态；枚举名沿用 EtherCAT 家族。
 * 名称里附上 CAN 的真实含义，避免现场按 EtherCAT 的经验误读。
 * ======================================================================== */

const char *jsdk_axis_state_string(jsdk_axis_state_t state)
{
    switch (state) {
    case JSDK_AXIS_UNKNOWN:              return "unknown";
    case JSDK_AXIS_SWITCH_ON_DISABLED:   return "switch-on-disabled(idle)";
    case JSDK_AXIS_READY_TO_SWITCH_ON:   return "ready-to-switch-on(calibrating)";
    case JSDK_AXIS_SWITCHED_ON:          return "switched-on(closed-loop,no-cmd)";
    case JSDK_AXIS_OPERATION_ENABLED:    return "operation-enabled";
    case JSDK_AXIS_FAULT:                return "fault";
    default:                             return "unknown-axis-state";
    }
}

/* ==========================================================================
 * 控制模式
 * ------------------------------------------------------------------------
 * 名字同时给出 SDK 模式名与线上 MsgType，便于对着总线日志排查。
 * ======================================================================== */

const char *jsdk_mode_string(jsdk_mode_t m)
{
    switch (m) {
    case JSDK_MODE_MIT:     return "mit(0x00)";
    case JSDK_MODE_CSP:     return "csp/pos(0x01)";
    case JSDK_MODE_CSV:     return "csv/vel(0x02)";
    case JSDK_MODE_CST:     return "cst/torque(0x03)";
    case JSDK_MODE_CURRENT: return "current(0x04)";
    default:                return "unknown-mode";
    }
}

/* ==========================================================================
 * 端点类型
 * ======================================================================== */

const char *jsdk_ep_type_string(jsdk_ep_type_t type)
{
    /* 复用协议层的名称表，保证与描述符解析器永远一致 */
    return jsdk_ep_type_name(type);
}

/* ==========================================================================
 * 浮点 → 定点文本
 * ------------------------------------------------------------------------
 * 纯整数实现，不依赖 libc 的浮点 printf（见头文件说明）。
 * ======================================================================== */

size_t jsdk_fmt_f(char *dst, size_t cap, double v, unsigned decimals)
{
    size_t   n = 0u;
    uint64_t scale = 1u;
    uint64_t mag;
    unsigned i;
    int      neg = 0;

    if (!dst || cap == 0u) return 0u;

    if (decimals > 9u) decimals = 9u;
    for (i = 0u; i < decimals; ++i) scale *= 10u;

    if (!(v == v)) {                       /* NaN：不能走 (int) 转换 */
        return jsdk_fmt_f(dst, cap, 0.0, decimals);   /* 统一写 0.000 */
    }
    if (v < 0.0) { neg = 1; v = -v; }

    /* ±1e15 以上不再保证精度；这里直接饱和，绝不产生 UB */
    if (v > 1.0e15) v = 1.0e15;

    mag = (uint64_t)(v * (double)scale + 0.5);

    /* 整数部分（手工十进制，避免依赖 %llu 的 libc 支持） */
    {
        char     tmp[24];
        unsigned t = 0u;
        uint64_t ip = mag / scale;
        uint64_t fp = mag % scale;

        if (neg && cap > n + 1u) dst[n++] = '-';

        do {
            tmp[t++] = (char)('0' + (int)(ip % 10u));
            ip /= 10u;
        } while (ip != 0u && t < sizeof tmp);
        while (t > 0u && n + 1u < cap) dst[n++] = tmp[--t];

        if (decimals > 0u && n + 1u < cap) {
            dst[n++] = '.';
            for (i = 0u; i < decimals; ++i) {
                scale /= 10u;
                if (n + 1u < cap) dst[n++] = (char)('0' + (int)((fp / scale) % 10u));
            }
        }
    }

    dst[n] = '\0';
    return n;
}

/* ======================== src/core/jsdk_units.c ======================== */
/**
 * @file    jsdk_units.c
 * @brief   单位换算与多圈展开（纯函数，无状态）
 *
 * @par CyberBeast 的线上单位（**先说清楚，否则下面全是错的**）
 *
 * | 传输路径 | 位置单位 | 速度单位 | 力矩/电流 |
 * |---|---|---|---|
 * | MIT 命令/响应 | **输出端 rad** | 输出端 rad/s | N·m（输出端） |
 * | POS_CONTROL | **度**（输出端，i16 ×0.001 / f32） | RPM | A（电机端） |
 * | VEL_CONTROL | — | **RPM** | A |
 * | TORQUE_CONTROL | — | — | N·m（**电机端**） |
 * | CURRENT_CONTROL | — | — | A（电机端） |
 * | 心跳 / QUERY_POS_VEL | **电机端 turns** | 电机端 turns/s | A（电机端） |
 *
 * 因此 L3 的换算只有三类：`rad↔turns`（乘除 gear_ratio）、`rad/s↔RPM`、
 * 以及 kp/kd 的量纲修正（§6.2）。
 *
 * @par `jsdk_unit_scale_t` 在本后端的含义
 *  CAN 侧线上量在模式映射之后**已经是物理量**，所以默认换算是恒等映射
 *  （`1.0 / 1.0 / 1.0`）。`jsdk_unit_scale_calc()` 的用途是给
 *  **按编码器计数驱动**的场景（例如客户用 `set_target_position(raw)` 直接下发
 *  自己约定的计数），此时需要 `counts → rad` 的比例。
 */



#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#define JSDK_TWO_PI (2.0 * M_PI)

/* ==========================================================================
 * 基础换算
 * ======================================================================== */

/**
 * rad → 电机 turns。
 * ⚠ 这是**输出端 rad**（MIT 语义）到**电机 turns**（心跳/QUERY 语义）的转换，
 *   也是两套坐标系唯一的桥。`gear_ratio` 非法时返回 0（绝不做除零）。
 */
double jsdk_units_rad_to_turns(double rad, double gear_ratio)
{
    if (!(gear_ratio > 0.0)) return 0.0;
    return rad * gear_ratio / JSDK_TWO_PI;
}

double jsdk_units_turns_to_rad(double turns, double gear_ratio)
{
    if (!(gear_ratio > 0.0)) return 0.0;
    return turns * JSDK_TWO_PI / gear_ratio;
}

/** 输出端 rad/s → 输出端 RPM。 */
double jsdk_units_rad_s_to_rpm(double rad_s)
{
    return rad_s * 60.0 / JSDK_TWO_PI;
}

double jsdk_units_rpm_to_rad_s(double rpm)
{
    return rpm * JSDK_TWO_PI / 60.0;
}

/* ==========================================================================
 * kp / kd 量纲修正（DESIGN §6.2）
 *
 * 固件：`torque_motor = tau + kp×(pos_sp − pos_est) + kd×(vel_des − vel_est)`，
 * 其中 pos 为**电机 turns**，而 kp/kd 由主站原样透传。线上下发 pos 时 SDK 已做
 * `turns = rad × gear / 2π`，于是误差被放大了 `gear/2π` 倍：
 *
 *      力矩 ≈ kp × (gear/2π) × Δrad      ⇒   刚度_输出端 = kp × gear / 2π
 *
 * 反解即 `kp = 刚度 × 2π / gear`。以 gear = 16.5 计，客户直接填 kp 会得到
 * 约 2.63 倍于直觉的刚度 —— 这是最容易让调参现场翻车的一条。
 * ======================================================================== */

double jsdk_units_stiffness_to_kp(double stiffness_nm_per_rad, double gear_ratio)
{
    if (!(gear_ratio > 0.0)) return 0.0;
    return stiffness_nm_per_rad * JSDK_TWO_PI / gear_ratio;
}

double jsdk_units_kp_to_stiffness(double kp, double gear_ratio)
{
    if (!(gear_ratio > 0.0)) return 0.0;
    return kp * gear_ratio / JSDK_TWO_PI;
}

/* ==========================================================================
 * 多圈展开
 * ------------------------------------------------------------------------
 * MIT 响应的 pos 是 16-bit 定点，满量程 ±range。因此：
 *   - **单圈内不会回绕**，但超出量程会被设备**钳位**（不是回绕）；
 *   - 想得到绝对多圈位置必须外部累加，而累加会掩盖钳位 → 默认关闭。
 * 这里只做纯函数形式的展开，是否启用在调用方（§6.1）。
 * ======================================================================== */

double jsdk_units_pos_unwrap(double prev_rad, double raw_rad, double range_rad,
                             long *out_turns)
{
    long   turns = 0;
    double out;

    if (!(range_rad > 0.0)) {
        if (out_turns) *out_turns = 0;
        return raw_rad;          /* 无量程信息 → 不做任何推断 */
    }

    /* 把 prev 归一到 (-range, +range] 后的同圈位置，再与 raw 比较跳变 */
    if (prev_rad != 0.0) {
        double w = prev_rad / (2.0 * range_rad);
        double f = w - floor(w);                 /* [0,1) */
        double p = f * 2.0 * range_rad;
        if (p > range_rad) p -= 2.0 * range_rad; /* → (-range, range] */
        turns = (long)floor(w);

        /* 跳变超过半圈 → 认为跨圈（只在"真实越过 ±range"时才会发生） */
        if (raw_rad - p > range_rad)       turns -= 1;
        else if (p - raw_rad > range_rad)  turns += 1;
    }

    out = raw_rad + 2.0 * range_rad * (double)turns;
    if (out_turns) *out_turns = turns;
    return out;
}

/* ==========================================================================
 * jsdk_unit_scale_*
 * ======================================================================== */

void jsdk_unit_scale_default(jsdk_unit_scale_t *scale, uint32_t rated_trq)
{
    if (!scale) return;

    /* CAN 线上量已是物理量 → 恒等映射。
       rated_trq 只用于判断"调用方是否真的知道这台电机"，避免把 0 当成有效标定。 */
    scale->pos_counts_to_rad   = 1.0;
    scale->vel_counts_to_rad_s = 1.0;
    scale->trq_to_Nm           = 1.0;
    scale->valid               = (rated_trq > 0u) ? 1 : 0;
}

/**
 * 按编码器计数换算（**面向计数驱动的场景**，见文件头说明）。
 *
 * @param encoder_resolution 编码器每**电机**转的计数（CPR，正交后）
 * @param motor_rev          齿轮箱电机侧转数
 * @param shaft_rev          齿轮箱输出侧转数   → gear = shaft_rev / motor_rev
 * @param rated_torque       额定力矩（N·m）
 *
 * 公式（编码器装在**电机**侧）：
 * @verbatim
 *   pos_counts_to_rad = 2π × motor_rev / (encoder_resolution × shaft_rev)
 *   trq_to_Nm         = rated_torque / 1000      （线力矩单位 = 0.1% 额定）
 * @endverbatim
 *
 * 任一参数为 0 → `valid = 0` 且比例置 0（**绝不猜**，DESIGN §6.10）。
 */
void jsdk_unit_scale_calc(jsdk_unit_scale_t *scale,
                          uint32_t encoder_resolution,
                          uint32_t motor_rev,
                          uint32_t shaft_rev,
                          uint32_t rated_torque)
{
    if (!scale) return;

    scale->pos_counts_to_rad   = 0.0;
    scale->vel_counts_to_rad_s = 0.0;
    scale->trq_to_Nm           = 0.0;
    scale->valid               = 0;

    if (encoder_resolution == 0u || motor_rev == 0u || shaft_rev == 0u
        || rated_torque == 0u) {
        return;
    }

    {
        double denom = (double)encoder_resolution * (double)shaft_rev;
        double r = JSDK_TWO_PI * (double)motor_rev / denom;

        /* 上界兜底：编码器极小时 r 会大到失去意义（且 NaN 也过不了两个比较）。
           ⚠ 这里刻意不用 isfinite()：某些 libc 把它展开成带 float 中间值的
           宏，在 -Wconversion 下会报 double→float 收窄。 */
        if (!(r > 0.0) || r > 1.0e12) return;

        scale->pos_counts_to_rad   = r;
        scale->vel_counts_to_rad_s = r;           /* counts/s → rad/s 同比例 */
        scale->trq_to_Nm           = (double)rated_torque / 1000.0;
        scale->valid               = 1;
    }
}

void jsdk_joint_set_scale(jsdk_joint_t *j, const jsdk_unit_scale_t *scale)
{
    if (!jsdk_joint_check(j) || !scale) return;
    j->scale = *scale;
    if (scale->valid) {
        j->status_flags = (uint16_t)(j->status_flags & ~(uint16_t)JSDK_JF_SCALE_INVALID);
    } else {
        j->status_flags |= (uint16_t)JSDK_JF_SCALE_INVALID;
    }
}

void jsdk_joint_get_scale(const jsdk_joint_t *j, jsdk_unit_scale_t *scale)
{
    if (!scale) return;
    if (!jsdk_joint_check(j)) {
        jsdk_unit_scale_default(scale, 0u);
        return;
    }
    *scale = j->scale;
}

/* ======================== src/core/jsdk_watchdog.c ======================== */
/**
 * @file    jsdk_watchdog.c
 * @brief   看门狗 / keepalive（DESIGN §6.3、PROTOCOL_NOTES §4.6）
 *
 * @par 这里其实有**两套**互不相干的机制，必须同时照顾
 *
 * 1. **ODrive 自身的看门狗**（`axis.config.watchdog_timeout`）：
 *    `do_command()` 开头**无条件** `axis.watchdog_feed()` —— 任何发往该设备的帧
 *    都喂它。所以"帧够不够多"不是问题，"有没有帧"才是。
 *
 * 2. **协议级 CAN 超时**（`can.config.break_timeout`，默认 100 ms）：
 *    `last_cmd_time_` **只**由 `is_ctrl` 帧（MsgType ≤ 0x03 或 0x80..0x83）更新。
 *    → 纯 `CURRENT_CONTROL` 的客户端**永远不武装**这个保护（安全缺口 F19，
 *      见 `docs/FIRMWARE_ISSUES.zh-CN.md`）；
 *    → 已经发过 MIT 的客户端若只发 CURRENT，会被**误停**。
 *
 * 因此 `auto_keepalive` 的策略是：只要"距上次控制类帧"接近 `break_timeout`，
 * 就补发一帧 **MIT**（`is_ctrl`），既武装保护又不被误停，且不改变运动状态
 * （kp = kd = 0、tau = 0 → 电机泄力；比"保持位置"更安全）。
 */


/* ==========================================================================
 * 设备侧超时值的获取
 * ------------------------------------------------------------------------
 * `break_timeout` 由 configure() 从端点读出（0 → 固件按 100 ms 处理）。
 * ======================================================================== */

uint32_t jsdk_watchdog_device_ms(const jsdk_joint_t *j)
{
    if (!jsdk_joint_check(j)) return JSDK_WD_DEFAULT_MS;
    if (j->break_timeout_ms == 0u) return JSDK_WD_DEFAULT_MS;   /* ⚠ 0 ≠ 关闭 */
    return j->break_timeout_ms;
}

/**
 * 补喂狗。
 *
 * @return 1 = 已补发；0 = 不需要或发不出去
 */
static int keepalive_joint(jsdk_joint_t *j)
{
    jsdk_context_t *ctx = j->ctx;
    uint32_t wd, since, lead;
    jsdk_target_t save;
    uint8_t payload[8];
    uint8_t clamped = 0u;
    uint32_t period_ms;

    if (!j->calibrated) return 0;

    wd = jsdk_watchdog_device_ms(j);

    /* 首个控制帧之前不需要补喂：`last_cmd_time_ == 0` 时固件直接跳过检查，
       而且此时补喂反而会**武装**保护，让刚启动的客户莫名被停。 */
    if (j->last_ctrl_tx_ms == 0u) return 0;

    period_ms = (uint32_t)(ctx->cfg.period_ns / 1000000u);
    lead = period_ms * 2u;
    if (lead < JSDK_WD_RISK_MIN_MS) lead = JSDK_WD_RISK_MIN_MS;

    since = jsdk_elapsed(ctx->now_ms, j->last_ctrl_tx_ms);
    if (since + lead < wd) {
        /* 还很宽裕；但接近时置一次风险位，便于现场定位"周期比超时还长"的配置 */
        if (wd - since < 4u * lead + lead) {
            jsdk_joint_set_flags(j, (uint16_t)JSDK_JF_WATCHDOG_RISK);
        }
        return 0;
    }

    /* --- 补一帧 MIT：pos = 实际位置, vel = 0, kp = kd = 0, tau = 0 ---
       直接构造临时目标，不改动客户设过的目标（发送后还原）。 */
    save = j->tgt;
    j->tgt.pos_rad   = j->fb.pos;
    j->tgt.vel_rad_s = 0.0;
    j->tgt.kp = 0.0; j->tgt.kd = 0.0; j->tgt.tau_Nm = 0.0;

    cb_mit_pack_command(payload, &j->range,
                        (float)j->tgt.pos_rad, 0.0f, 0.0f, 0.0f, 0.0f, &clamped);

    j->tgt = save;

    if (jsdk_ctx_send(ctx, CB_PRI_HIGH_CTRL, CB_MSG_MIT_CONTROL, j->cfg.node_id,
                      payload, 8u) != 0) {
        jsdk_joint_set_flags(j, (uint16_t)JSDK_JF_TX_FAILED);
        return 0;
    }
    ctx->tx_seq = cb_seq_next(ctx->tx_seq);

    j->last_ctrl_tx_ms = ctx->now_ms;
    j->keepalive_sent++;
    ctx->bus.keepalive_sent++;
    jsdk_joint_set_flags(j, (uint16_t)JSDK_JF_WATCHDOG_RISK);

    /* 补喂帧不是客户指令，不计入 tx_frames（那是"客户下发的控制帧"计数） */
    return 1;
}

void jsdk_watchdog__cycle_end(jsdk_context_t *ctx)
{
    unsigned i;

    if (!jsdk_ctx_check(ctx)) return;
    if (!ctx->cfg.auto_keepalive) return;

    for (i = 0u; i < ctx->nj; ++i) {
        jsdk_joint_t *j = &ctx->joints[i];
        if (j->enable_pending || j->disable_pending || j->faultreset_pending) {
            continue;      /* 序列期间本来就在持续发帧 */
        }
        (void)keepalive_joint(j);
    }
}
