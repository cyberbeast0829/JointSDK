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

#include <stddef.h>
#include <stdint.h>

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
