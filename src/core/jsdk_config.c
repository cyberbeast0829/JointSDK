/**
 * @file    jsdk_config.c
 * @brief   配置阶段：configure / activate / deactivate / discover / 参数读写
 *
 * 这些都是**阻塞** API（DESIGN §7 的约定）：内部用 `jsdk_ctx_read_param()` /
 * `jsdk_ctx_wait_response()` 轮询，允许最长 `JSDK_CFG_TIMEOUT_MS`。
 * 绝不可在控制循环运行期间调用（会与响应争用并阻塞周期）。
 */

#include "jsdk_core_internal.h"

#include <stdio.h>
#include <string.h>

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
