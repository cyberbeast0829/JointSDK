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

#include <stdint.h>
#include <stddef.h>

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
