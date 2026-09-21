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
 * 2. **协议级 CAN 超时**（`can.config.break_timeout`）：
 *    `last_cmd_time_` **只**由 `is_ctrl` 帧（MsgType ≤ 0x03 或 0x80..0x83）更新。
 *    → 纯 `CURRENT_CONTROL` 的客户端**永远不武装**这个保护（安全缺口 F19，
 *      见 `docs/FIRMWARE_ISSUES.zh-CN.md`）；
 *    → 已经发过 MIT 的客户端若只发 CURRENT，会被**误停**。
 *
 *    ⚠⚠ **`0` = 这个超时检测被禁用**（最新固件 `auto_stop_if_timeout()` 首句
 *      `if (timeout_ms == 0) return;`，且配置项默认值就是 0）。
 *      早期版本把 0 当成 100 ms，导致上位机把“没武装”读成“已武装 100 ms”。
 *      因此：**只有 > 0 时才需要补喂**；0 时既不补喂也不报风险。
 *
 * 因此 `auto_keepalive` 的策略是：当设备侧超时 > 0，且“距上次控制类帧”接近
 * `break_timeout` 时，补发一帧 **MIT**（`is_ctrl`）—— 既武装保护又不被误停，
 * 且不改变运动状态（kp = kd = 0、tau = 0 → 电机泄力；比“保持位置”更安全）。
 */

#include "jsdk_core_internal.h"

/* ==========================================================================
 * 设备侧超时值的获取
 * ------------------------------------------------------------------------
 * `break_timeout` 由 configure() 从端点读出。
 *
 * ⚠ **0 = 设备侧超时检测被禁用**（最新固件语义；不是“默认 100 ms”）。
 *   句柄无效时也返回 0（“无法判定”按“不巡喂”处理，永不会因为未知值乱发帧）。
 * ======================================================================== */

uint32_t jsdk_watchdog_device_ms(const jsdk_joint_t *j)
{
    if (!jsdk_joint_check(j)) return JSDK_WD_DISABLED_MS;
    return j->break_timeout_ms;      /* 0 = 禁用；> 0 = 超时毫秒数 */
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

    /* ⚠ **0 = 设备侧超时检测被禁用** → 无狗可喂：既不补帧也不置风险位。
       补帧在这里毫无意义（没有门限要满足），只会自白增加总线流量 ——
       而且会掩盖“你没在发控制帧”这个事实。 */
    if (wd == JSDK_WD_DISABLED_MS) return 0;

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
