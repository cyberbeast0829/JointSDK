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

#include "jsdk_core_internal.h"

#include <math.h>

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
 * 固件 `Controller::update()` 的 MIT 分支（controller.cpp:410-418, tag v4.2.55）：
 *
 *      mit_p_err = (pos_setpoint_ − pos_est) / gear_ratio * 2π
 *      torque    = tau*torque_setpoint_*gear_ratio + kp*mit_p_err + kd*mit_v_err
 *      torque_output_ = torque / gear_ratio                  (:450)
 *
 * ⚠ 关键：MIT 入口 `set_input_pos_and_steps()` 收到的 pos **已经是电机端 turns**
 *    （CAN 侧 :371 做了 `pos*g/(2π)`），`pos_setpoint_`/`pos_estimate_linear`
 *    两边同为电机端 turns，故 `pos_err` 是电机端 turns。`:415` 的
 *    `/gear*2π` 作用是把**电机端 turns 换回输出端 rad**。
 *
 *    于是：mit_p_err = (Δrad_out / gear * 2π) 代入（其中 Δturns = Δrad*g/2π）
 *        => mit_p_err = Δrad_out
 *        => torque_motor = kp * Δrad_out
 *        => torque_out   = kp * Δrad_out / gear      (:450)
 *
 *          ⇒ 输出端等效刚度 = kp / gear
 *
 *    反解：kp = 刚度_输出端 × gear。
 *
 * 历史备注：本函数早期版本按 “kp 作用在电机 turns 误差上” 推导，
 * 写成 `刚度 × 2π / gear`，与固件实际链路差 2π 倍（偏高约 6.28×）。
 * 以 gear = 16.5、刚度 100 为例：旧式给 kp ≈ 38.1，正确应给 kp = 1650。
 * 详见 docs/FIRMWARE_REPLY_DRAFT.zh-CN.md §1。
 * ======================================================================== */

double jsdk_units_stiffness_to_kp(double stiffness_nm_per_rad, double gear_ratio)
{
    if (!(gear_ratio > 0.0)) return 0.0;
    return stiffness_nm_per_rad * gear_ratio;
}

double jsdk_units_kp_to_stiffness(double kp, double gear_ratio)
{
    if (!(gear_ratio > 0.0)) return 0.0;
    return kp / gear_ratio;
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
