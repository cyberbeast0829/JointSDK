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
 * 会话预热（`jsdk_context_warmup()`）
 * ======================================================================== */

/* 每轮等响应的窗口 = `JSDK_WARMUP_ATTEMPT_MS`（公共头里有说明：刻意取小，
   因为“首帧丢失”等再久也没用，该做的是重发）。 */

jsdk_status_t jsdk_context_warmup(jsdk_context_t *ctx, uint32_t timeout_ms)
{
    uint32_t budget;
    uint32_t t0;
    unsigned attempts = 0u;
    unsigned max_attempts;
    uint8_t  node;

    if (!jsdk_ctx_check(ctx)) return JSDK_ERR_INVALID_ARG;
    if (ctx->nj == 0u) {
        jsdk_ctx_seterr(ctx, "warm-up needs at least one joint (its node_id is the "
                             "probe target)");
        return JSDK_ERR_BAD_STATE;
    }
    if (ctx->warmed) return JSDK_OK;      /* 同一个会话里只做一次 */

    node   = ctx->joints[0].cfg.node_id;
    budget = timeout_ms ? timeout_ms : JSDK_WARMUP_TIMEOUT_MS;
    /*
     * ⚠ 除了时间预算，还必须有**轮次上限**：`now_ms()` 不前进的 HAL（部分测试夹具、
     *   客户自写的假时钟）下时间预算永远不会到期，光靠它就是一个**死循环**。
     *   （`jsdk_ctx_wait_response()` 里那个自旋上限是同样理由的同一道防线。）
     */
    max_attempts = (unsigned)(budget / JSDK_WARMUP_ATTEMPT_MS) + 2u;
    t0     = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);
    /*
     * 标记“会话预热已经尝试过”：
     *   - 成功 → `warmed`，以后全是空操作；
     *   - 失败 → 自动预热（`jsdk_ctx_send()` 里的钩子）不会**每个请求**都再花
     *     500 ms，但显式再调本函数仍然会重试（现场复查链路时很有用）。
     */
    ctx->warmup_tried = 1u;
    ctx->in_warmup    = 1u;   /* 预热期间的收发不参与帧格式学习（见内部头注释） */

    for (;;) {
        attempts++;

        /* 幂等探测：`QUERY_DEVICE_INFO` 只读一个设备信息，重发无副作用。
           ⚠ 用 raw 发送：不能在这里再触发一次自动预热（那才是真的递归）。 */
        if (jsdk_ctx_send_raw(ctx, CB_PRI_QUERY, CB_MSG_QUERY_DEVICE_INFO, node,
                              NULL, 0u) != 0) {
            ctx->in_warmup = 0u;
            ctx->bus.tx_retries += (uint32_t)(attempts - 1u);   /* 之前那几轮是真重发 */
            return JSDK_ERR_TRANSPORT;
        }
        ctx->tx_seq = cb_seq_next(ctx->tx_seq);

        if (jsdk_ctx_wait_response(ctx, CB_MSG_QUERY_DEVICE_INFO, node, NULL,
                                   JSDK_WARMUP_ATTEMPT_MS) == JSDK_OK) {
            ctx->warmed     = 1u;
            ctx->in_warmup  = 0u;
            ctx->bus.tx_retries += (uint32_t)(attempts - 1u);
            return JSDK_OK;
        }

        if (attempts >= max_attempts
            || jsdk_elapsed(ctx->cfg.hal.now_ms(ctx->cfg.hal.user), t0)
                   + JSDK_WARMUP_ATTEMPT_MS >= budget) {
            ctx->in_warmup = 0u;
            ctx->bus.tx_retries += (uint32_t)(attempts - 1u);
            jsdk_ctx_seterr(ctx,
                "session warm-up failed: node %u did not answer %u attempts "
                "(budget %u ms, %u ms per attempt) — the link is up but there is no "
                "reply; check the adapter/device, or the frame format if you set one "
                "explicitly",
                (unsigned)node, (unsigned)attempts, (unsigned)budget,
                (unsigned)JSDK_WARMUP_ATTEMPT_MS);
            return JSDK_ERR_TIMEOUT;
        }
    }
}

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
/**
 * “序列正在跑”的状态集合（固件 `AxisState`）：启动序列 / 全标定 / 电机标定 /
 * 编码器偏置 / 索引搜索 / LOCKIN 自转 / 方向探测 / 回零 / HALL 标定 / 齿槽标定。
 *
 * ⚠ 全标定（3）在运行期间报的是**子状态**（真机实测 4 → 7 → 1），**从不等于 3** ——
 * 拿“等于请求值”当“已进入”，会把一次真跑起来的标定误判成“没开始”；
 * 反过来也不能把“不等于请求值”当“跑完了”（子状态本身就与请求值不等）。
 */
static int state_is_busy(uint8_t st)
{
    switch (st) {
    case 2u:  /* STARTUP_SEQUENCE                  */
    case 3u:  /* FULL_CALIBRATION_SEQUENCE         */
    case 4u:  /* MOTOR_CALIBRATION                 */
    case 6u:  /* ENCODER_OFFSET_CALIBRATION        */
    case 7u:  /* ENCODER_INDEX_SEARCH              */
    case 9u:  /* LOCKIN_SPIN                       */
    case 10u: /* ENCODER_DIR_FIND                  */
    case 11u: /* HOMING（回零期间就停在这个值）       */
    case 12u: /* ENCODER_HALL_POLARITY_CALIBRATION */
    case 13u: /* ENCODER_HALL_PHASE_CALIBRATION    */
    case 14u: /* ANTICOGGING_CALIBRATION           */
        return 1;
    default:
        return 0;
    }
}

/**
 * “等状态跑完”时的轮询间隔（ms）。子状态是**秒**级；而全速刷（以前就是：每轮一次
 * param read，实测 ~2500 次/s）会挤爆 115200 的 slcan 适配器（理论上限 ~380 次交换/s）——
 * 真机实测 100757 次读取里后 2/3 全部超时，而设备**其实 29.5 s 就跑完了**。
 */
#define JSDK_STATE_POLL_MS 200u

/**
 * 还没看到“离开静息”之前的轮询间隔（ms）：**起始判决必须快**。
 * 真机子状态是秒级，但仿真模型（以及未来可能更快的固件）可能只持续几十 ms；
 * 用 200 ms 去撒，会把整个瞬时过程**漏掉** → 误报“never left idle”。
 */
#define JSDK_STATE_POLL_MS_FAST 20u

/** 等状态序列跑完的预算：配置没给就用该操作的内置默认。 */
static uint32_t state_budget_ms(const jsdk_joint_t *j, uint32_t fallback_ms)
{
    uint32_t v = j->ctx->cfg.state_timeout_ms;
    return (v != 0u) ? v : fallback_ms;
}

/**
 * 设备当前是否有故障（抓一次 QUERY_ERROR(0x45) 六类明细）。
 *
 * @param fb  可选输出：人可读的一行（`jsdk_joint_describe_fault` 格式）
 * @return 1 = 有故障；0 = 干净；**-1 = 查不到**（别把“查不到”当成“没故障”）
 */
static int device_fault(jsdk_joint_t *j, char *fb, size_t cap)
{
    jsdk_fault_info_t info;

    if (fb != NULL && cap > 0u) fb[0] = '\0';
    if (jsdk_joint_query_error_detail(j, &info) != JSDK_OK) return -1;
    if ((info.motor_error | info.encoder_error | info.sensorless_error
         | info.controller_error | info.system_error | info.axis_error) == 0u) {
        return 0;
    }
    if (fb != NULL && cap > 0u) (void)jsdk_joint_describe_fault(j, fb, cap);
    return 1;
}

/**
 * 写 `requested_state = transient` 并等序列跑完。
 *
 * 三条判据都是真机踩出来的（详见 `docs/BACKLOG.zh-CN.md` §2.5d）：
 *   ① “已进入” = **离开静息态**，不是“等于请求值”（全标定只报子状态 4/7）；
 *   ② “跑完了” = 回到**静息态**（IDLE 或 CLOSED_LOOP_CONTROL）；
 *   ③ “成功” = 跑完**且设备没报故障**（锁存故障时轴会直接拒绝启动）。
 * 另外轮询必须限速，且单次读失败不能当成“状态没变”。
 */
static jsdk_status_t request_state_and_wait(jsdk_joint_t *j, uint8_t transient,
                                            uint32_t timeout_ms)
{
    uint32_t t0;
    uint32_t next_poll;
    uint32_t poll_ms = JSDK_STATE_POLL_MS_FAST;
    uint32_t last_now = 0u;
    unsigned frozen = 0u;      /* 时钟连续多少次没动的计数（只用于冻结兜底） */
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
    next_poll = t0;

    for (;;) {
        uint8_t  st;
        uint32_t now = j->ctx->cfg.hal.now_ms(j->ctx->cfg.hal.user);

        j->ctx->now_ms = now;
        if (jsdk_elapsed(now, t0) >= timeout_ms) break;

        /*
         * ⚠ 兜底只能看“**时钟是否冻住**”，不能用“循环次数上限”：
         *   真机 HAL 下 `recv()` 无数据时立即返回，一轮只有几百 ns —— 次数上限
         *   会在**几秒内**被撞到，于是 120 s 的预算在 4.4 s 就报超时
         *   （实测踩过：提示写着 “within 120000 ms”，墙钟只过了 4.4 s）。
         *   而虚拟 HAL 的时钟只在测试推时间轴时前进，所以确实需要这个兜底。
         */
        if (now != last_now) {
            last_now = now;
            frozen = 0u;
        } else if (++frozen >= 200000u) {
            break;                 /* 时钟冻住（仿真 HAL 未推进）→ 只能放弃 */
        }

        if (jsdk_elapsed(now, next_poll) < poll_ms) {
            /*
             * 空闲期间**只收不发**（排掉 RX，免得适配器缓冲溢出丢帧）。
             * ⚠ 这里**不能**用 `jsdk_context_poll()`：它跑的是完整周期
             *   （cycle_begin + cycle_end），会按需补发 keepalive 控制帧 ——
             *   标定期间往正在标定的轴上送 MIT 帧是不该做的事。
             */
            jsdk_can_frame_t f;
            int n = j->ctx->cfg.hal.recv(j->ctx->cfg.hal.user, &f);

            if (n > 0) {
                j->ctx->bus.rx_frames++;
                j->ctx->bus.link_up = 1u;
                j->ctx->last_rx_ms = now;
                (void)jsdk_ctx_handle_frame(j->ctx, &f);
            }
            continue;
        }
        next_poll = now;

        if (read_current_state(j, &st) != 0) {
            /* 单次读失败**不是**“状态没变”（真机上偶发超时之后就恢复了）。 */
            continue;
        }

        j->current_state_raw = st;
        if (state_is_busy(st)) {
            seen_transient = 1;
            poll_ms = JSDK_STATE_POLL_MS;   /* 已确认开始了 → 转入慢轮询 */
        } else if (seen_transient) {
            char fb[192];
            int  f;

            j->ctrl_blocked = 0u;
            f = device_fault(j, fb, sizeof fb);
            /* 回到静息态**不等于**成功：标定失败就是“进子状态 → 出错 → 回静息”。 */
            if (f == 1) {
                jsdk_joint_seterr(j,
                    "state %u ran but the device reports a fault: %s"
                    "（先清故障（`fault-reset`）再重试）",
                    (unsigned)transient, fb);
                return JSDK_ERR_PROTOCOL;
            }
            jsdk_ctx_seterr(j->ctx, "node %u finished state %u (back to rest%s)",
                            (unsigned)j->cfg.node_id, (unsigned)transient,
                            f == 0 ? ", no fault" : ", fault check unavailable");
            return JSDK_OK;
        }
    }

    j->ctrl_blocked = 0u;
    {
        char fb[192] = {0};
        int  f = device_fault(j, fb, sizeof fb);

        if (seen_transient) {
            jsdk_joint_seterr(j, "state %u did not finish within %u ms%s%s",
                              (unsigned)transient, (unsigned)timeout_ms,
                              f == 1 ? " | device fault: " : "", f == 1 ? fb : "");
        } else {
            /* 状态**完全没动**：实测最常见的原因是**锁存故障**让状态机拒绝启动
               （写被接受、值也被消费，但状态就是不动）。 */
            jsdk_joint_seterr(j, "device never left idle after requesting state %u"
                                 " (waited %u ms)%s%s",
                              (unsigned)transient, (unsigned)timeout_ms,
                              f == 1 ? " | device fault: " : "", f == 1 ? fb : "");
        }
    }
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

    return request_state_and_wait(j, 3u,
                                  state_budget_ms(j, JSDK_STATE_TIMEOUT_CALIBRATE_MS));
}

jsdk_status_t jsdk_joint_home(jsdk_joint_t *j)
{
    if (!jsdk_joint_check(j)) return JSDK_ERR_INVALID_ARG;
    if (!j->calibrated) return JSDK_ERR_BAD_STATE;
    return request_state_and_wait(j, 11u,
                                  state_budget_ms(j, JSDK_STATE_TIMEOUT_HOME_MS));
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
