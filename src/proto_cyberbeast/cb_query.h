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

#include <stddef.h>
#include <stdint.h>

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
