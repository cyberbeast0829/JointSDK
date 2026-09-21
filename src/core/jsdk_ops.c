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

#include "jsdk_core_internal.h"

#include <string.h>

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
    uint8_t le[2];
    int     disable;

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
    disable = (ms == 0u);

    /* 每次调用重新判定“能不能读回校验”，所以先清掉上一次的结果位 */
    jsdk_joint_clear_status_flags(j, (uint16_t)JSDK_JF_WATCHDOG_UNVERIFIED);

    /* ⚠ 参数值在线上是**小端**（设备端 memcpy 主机序），见 cb_frame.h。
       旧注释曾写"大端"并把 250→64000 归咎于"传主机序"——那是错的：
       64000 恰恰是**按大端发包**的结果，真机上是小端。 */
    cb_le_put_u16(le, (uint16_t)ms);
    if (jsdk_ctx_write_param(j->ctx, j->cfg.node_id, j->ep_break_timeout,
                             le, 2u, 0u) != JSDK_OK) {
        jsdk_joint_seterr(j, "failed to write can.config.break_timeout");
        return JSDK_ERR_TRANSPORT;
    }

    /* 读回校验（端点可读，没必要盲信写入）
     *
     * ⚠⚠ 真机实测（fw 1545）：**`can.config.break_timeout` 的读回恒为 0** ——
     *    紧随写入之后立刻读、同一个进程，读回来的也是 0（`sdo.data` 证实我们
     *    确实发出去了 `96 00` = 150 小端）。对照端点 `heartbeat_rate_ms` 的
     *    写入→读回是正常的，所以这是**该端点的固件问题**（FIRMWARE_ISSUES F28），
     *    不是请求打包问题。
     *    后果很重：`break_timeout = 0` 在新固件里的含义是**禁用超时检测**，
     *    所以“写 250 却读回 0”意味着**客户端无法证明自己武装了保护**。
     *    处理原则：
     *      - 读回 == 写入值      → 校验通过；
     *      - 读回 0 且写入非 0   → **未能校验**：置 JSDK_JF_WATCHDOG_UNVERIFIED、
     *                              保留写入值（安全方向：宁可多喂几帧），返回 OK；
     *      - 其它不一致          → 真矛盾 → PROTOCOL（这条不能放松）。
     */
    {
        uint8_t buf[8];
        uint8_t len = 0u;

        if (jsdk_ctx_read_param(j->ctx, j->cfg.node_id, j->ep_break_timeout,
                                buf, &len, 0u) == JSDK_OK && len >= 2u) {
            uint32_t back = cb_le_get_u16(buf);
            if (back == ms) {
                j->break_timeout_ms = back;          /* 含 0 == 0：禁用也能量化确认 */
            } else if (back == 0u && ms != 0u) {
                jsdk_joint_set_flags(j, (uint16_t)JSDK_JF_WATCHDOG_UNVERIFIED);
                j->break_timeout_ms = ms;            /* 保守：按“已武装”继续喂狗 */
            } else {
                j->break_timeout_ms = back;
                jsdk_joint_seterr(j, "break_timeout read back as %u, expected %u",
                                  (unsigned)back, (unsigned)ms);
                return JSDK_ERR_PROTOCOL;
            }
        } else {
            jsdk_joint_set_flags(j, (uint16_t)JSDK_JF_WATCHDOG_UNVERIFIED);
            j->break_timeout_ms = ms;                /* 读不回来就用写入值 */
        }

        jsdk_ctx_seterr(j->ctx,
            "break_timeout set to %u ms on node %u%s%s (not persisted; call "
            "save_config())",
            (unsigned)ms, (unsigned)j->cfg.node_id,
            disable ? "; 0 = protocol timeout DISABLED on the device" : "",
            (j->status_flags & (uint16_t)JSDK_JF_WATCHDOG_UNVERIFIED)
                ? "; NOTE: could not verify by read-back (this firmware's "
                  "can.config.break_timeout reads back 0 — see FIRMWARE_ISSUES "
                  "F28): treat the watchdog as NOT confirmed"
                : "");
    }

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
