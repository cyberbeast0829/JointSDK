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

#include <stddef.h>
#include <stdint.h>

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
 * @note 任何帧都会喂驱动器自身的 `axis.watchdog_feed()`（`do_command()` 开头
 *       无条件调用），那是**另一套机制**，不要与上面这个协议级超时混淆。
 */
int cb_ctrl_expects_response(uint8_t msgtype);

#ifdef __cplusplus
}
#endif

#endif /* CB_CTRL_H */
