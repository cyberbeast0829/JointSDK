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
int jsdk_ctx_read_param_ex(jsdk_context_t *ctx, uint8_t node_id, uint16_t ep_id,
                           uint8_t *out, uint8_t *out_len, uint32_t timeout_ms,
                           int allow_retry)
{
    uint8_t  req[CB_PARAM_READ_REQ_MIN];
    jsdk_can_frame_t rsp;
    size_t   n;
    int      rc;

    if (!jsdk_ctx_check(ctx) || !out) return JSDK_ERR_INVALID_ARG;

    n = cb_param_pack_read_req(req, sizeof req, ep_id, 4u, 0u, 0);
    if (n == 0u) return JSDK_ERR_INVALID_ARG;

    rc = jsdk_ctx_request(ctx, CB_PRI_CONFIG, CB_MSG_PARAM_READ, node_id,
                            req, (uint8_t)n, CB_MSG_PARAM_READ, &rsp,
                            timeout_ms ? timeout_ms : JSDK_CFG_TIMEOUT_MS,
                            allow_retry
                                ? (JSDK_REQ_RETRY | JSDK_REQ_COUNT) : 0u);
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

/** 读参数 + 幂等重发（面向用户的命令走这个）。 */
int jsdk_ctx_read_param(jsdk_context_t *ctx, uint8_t node_id, uint16_t ep_id,
                        uint8_t *out, uint8_t *out_len, uint32_t timeout_ms)
{
    return jsdk_ctx_read_param_ex(ctx, node_id, ep_id, out, out_len, timeout_ms, 1);
}

/** 读参数，只发一次（轮询路径用，见内部头注释）。 */
int jsdk_ctx_read_param_once(jsdk_context_t *ctx, uint8_t node_id, uint16_t ep_id,
                             uint8_t *out, uint8_t *out_len, uint32_t timeout_ms)
{
    return jsdk_ctx_read_param_ex(ctx, node_id, ep_id, out, out_len, timeout_ms, 0);
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

        rc = jsdk_ctx_request_retry(ctx, CB_PRI_CONFIG, CB_MSG_PARAM_READ, node_id,
                                    req, (uint8_t)n, &rsp,
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
 * @param val 值字节，**必须是线上小端序**（参数值小端；见 `cb_frame.h`）。
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

        /*
         * ⚠ **等 ACK 时才谈得上重发**，而“等 ACK 的写”都是“同一个值再写一遍”
         *   （幂等）—— 所以这里可以放心走 `jsdk_ctx_request_retry()`。
         *   `axis0.requested_state` 那类“写一下就跳转状态机”的端点传的是
         *   `timeout_ms = 0`（不等 ACK），因此**永远**走不到重发分支；
         *   将来改调用方时**不能**给它加非 0 超时（重发 = 重复触发标定/回零）。
         */
        if (timeout_ms != 0u) {
            jsdk_can_frame_t ack;
            return jsdk_ctx_request_retry(ctx, CB_PRI_CONFIG, CB_MSG_PARAM_WRITE,
                                          node_id, req, (uint8_t)n, &ack,
                                          timeout_ms);
        }

        rc = jsdk_ctx_send(ctx, CB_PRI_CONFIG, CB_MSG_PARAM_WRITE, node_id,
                           req, (uint8_t)n);
        if (rc != 0) return JSDK_ERR_TRANSPORT;
        ctx->tx_seq = cb_seq_next(ctx->tx_seq);
        return JSDK_OK;
    }

    /*
     * --- Classic 且 > 4 B：分块写（末块必须恰好补齐值）---
     *
     * 重发时重发的是**整条写**（从 offset 0 再走一遍），不是只补最后一块：
     * 设备的分段装配器可能已经把状态收尾了，从头发一遍才是幂等且语义明确的
     * （同一个值覆盖写）。
     */
    {
        /* ⚠ 用 `attempt < max_attempts`（而不是 `attempt >= 常量`）：后者在
           `JSDK_REQ_RETRY_MAX` 被改成 0 时会触发 `-Wtype-limits`，
           让“把重试关掉”这种调试/变异变成编译错误。 */
        const unsigned max_attempts = 1u + (unsigned)JSDK_REQ_RETRY_MAX;
        unsigned attempt;

        for (attempt = 0u; attempt < max_attempts; ++attempt) {
            uint8_t off = 0u;
            int     st;

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

            if (timeout_ms == 0u) return JSDK_OK;      /* 不等 ACK：发完就算 */

            {
                jsdk_can_frame_t ack;      /* 装配完成时设备回一次 ACK */

                st = jsdk_ctx_wait_response(ctx, CB_MSG_PARAM_WRITE, node_id, &ack,
                                           timeout_ms);
            }
            if (st == JSDK_OK) return JSDK_OK;
            if (st != JSDK_ERR_TIMEOUT) return st;
            ctx->bus.req_timeouts++;                    /* 应用写：等不到也要记账 */
            if (!ctx->bus.link_up) return st;           /* 链路本就不通：别再耗时 */
            if (attempt + 1u < max_attempts) {          /* 真的还会再试一次 */
                ctx->bus.tx_retries++;
                ctx->bus.tx_retries_req++;
            }
        }
        return JSDK_ERR_TIMEOUT;
    }
}

/**
 * 写一个参数并等待设备确认（用于“写完必须确认”的场景）。
 *
 * 现在直接复用 `jsdk_ctx_write_param()` 的等待/重发（同一个值覆盖写 ⇒ 幂等），
 * 不再自己再等一遍 ACK —— 以前那两次等待里第二次等的是**下一条** ACK，
 * 既多花一个超时，又可能被上一次写的残留 ACK 骗过。
 */
static int write_param_sync(jsdk_context_t *ctx, uint8_t node_id, uint16_t ep_id,
                            const void *val, uint8_t len, uint32_t timeout_ms)
{
    return jsdk_ctx_write_param(ctx, node_id, ep_id, val, len,
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
    *out = cb_le_get_f32(buf);      /* ⚠ 参数值小端，见 cb_frame.h */
    return 0;
}

/** 读一个整数端点（1/2/4 字节）；⚙ 参数值是**小端**（设备端 memcpy 主机序）。 */
static int read_int(jsdk_context_t *ctx, uint8_t node, uint16_t ep_id,
                    jsdk_ep_type_t type, uint32_t *out)
{
    uint8_t buf[8];
    uint8_t len = 0u;
    uint8_t need = (uint8_t)jsdk_ep_type_size(type);

    if (need == 0u || need > 8u) return -1;
    if (jsdk_ctx_read_param(ctx, node, ep_id, buf, &len, 0u) != JSDK_OK) return -1;
    if (len < need) return -1;

    switch (type) {
    case JSDK_EP_U8:  case JSDK_EP_BOOL: *out = (uint32_t)buf[0]; break;
    case JSDK_EP_U16: *out = (uint32_t)cb_le_get_u16(buf); break;
    case JSDK_EP_U32: *out = cb_le_get_u32(buf); break;
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
 * 握手尝试次数 / 每次等应答的上限。
 *
 * ⚠ 为什么是"短超时 × 多次重发"而不是"一次等 3 s"：
 *   设备的 master_id 是从**收到的帧**里学来的（`source_id == 0` 时它完全不回复），
 *   所以**握手帧丢了就等于整个会话哑掉**——真机（slcan）实测丢帧率约 1/10，
 *   而且丢的就是"打开端口后的头几帧"。中位应答时间只有几毫秒，
 *   把 3 s 预算花在"干等一帧永不到来的帧"上没有任何好处；拆成 6×50 ms
 *   既能把丢失的首帧补上，又把失败时的等待从 3 s 降到 ~300 ms。
 */
#define JSDK_HANDSHAKE_TRIES   6u
#define JSDK_HANDSHAKE_WAIT_MS 50u

/**
 * 握手并**确认它真的生效**：发 `QUERY_STATUS`，等它的应答（MsgType 0x00），
 * 没等到就重发（最多 `JSDK_HANDSHAKE_TRIES` 次）。
 *
 * 为什么不能只发一次：设备的 `master_id` 是从**收到的帧**里学来的
 * （`source_id == 0` 时它不回复任何东西）。所以握手帧丢了 = 整个会话哑掉，
 * 现场表现为 `configure()` 超时、而描述符/心跳看起来都正常。
 * 握手本身是幂等的（`QUERY_STATUS` 不改任何状态），重发无副作用。
 *
 * 成功后顺手把这一帧当作首次反馈吃进去（少一次等待）。
 */
static jsdk_status_t handshake_verified(jsdk_context_t *ctx, jsdk_joint_t *j)
{
    unsigned i;

    for (i = 0u; i < JSDK_HANDSHAKE_TRIES; ++i) {
        jsdk_can_frame_t rsp;

        handshake(ctx, j->cfg.node_id);
        if (jsdk_ctx_wait_response(ctx, CB_MSG_MIT_CONTROL, j->cfg.node_id,
                                   &rsp, JSDK_HANDSHAKE_WAIT_MS) == JSDK_OK) {
            jsdk_joint__on_mit_response(j, rsp.data, rsp.len);
            return JSDK_OK;
        }
    }

    jsdk_joint_seterr(j,
        "no response to handshake after %u attempts (node %u, %u ms each): "
        "check node_id / wiring / master_id; if the port was just opened, the "
        "adapter may be dropping the first frames",
        (unsigned)JSDK_HANDSHAKE_TRIES, (unsigned)j->cfg.node_id,
        (unsigned)JSDK_HANDSHAKE_WAIT_MS);
    return JSDK_ERR_TIMEOUT;
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

        if (ctx->cfg.enable_watchdog_hint && j->ep_break_timeout != 0u
            && (wd == 0u || (period_ms != 0u && period_ms >= wd))) {
            /* 主动武装：只在**需要**时才改设备 —— 当前是禁用（0），或者现有的超时
               比我们的控制周期还短（回路本来就喂不住）。设备本来设得很宽松时
               不动它：把 30000 ms 改成 2×周期只会让设备**更容易**被误停。
               写完必须**读回确认** —— 真机上这个端点的读回恒为 0（F28），
               也就是说“写成功”并不能证明“已经武装”。 */
            uint16_t want = (uint16_t)(period_ms * 2u);
            uint8_t  want_le[2];
            if (want < 2u) want = 2u;
            cb_le_put_u16(want_le, want);      /* ⚠ 参数值小端，见 cb_frame.h */
            if (write_param_sync(ctx, j->cfg.node_id, j->ep_break_timeout,
                                 want_le, 2u, 0u) == JSDK_OK) {
                uint8_t  buf[8];
                uint8_t  len = 0u;
                uint32_t back = 0u;
                if (jsdk_ctx_read_param(ctx, j->cfg.node_id, j->ep_break_timeout,
                                        buf, &len, 0u) == JSDK_OK && len >= 2u) {
                    back = cb_le_get_u16(buf);
                }
                if (back == want) {
                    j->break_timeout_ms = back;             /* 读回一致 = 真的武装了 */
                } else {
                    /* 读回不是我们要的值：**保守地认为“未武装”**。
                       ⚠ 这里不能报致命错误 —— 设备完全可以合法地把该功能关着；
                         但必须让客户知道“这一步没成”，否则他会以为有保护。 */
                    j->break_timeout_ms = jsdk_watchdog_device_ms(j);
                    jsdk_joint_seterr(j,
                        "watchdog hint: wrote break_timeout = %u ms but the device "
                        "reads back %u (see FIRMWARE_ISSUES F28): treat the protocol "
                        "watchdog as NOT armed", (unsigned)want, (unsigned)back);
                }
            }
            wd = jsdk_watchdog_device_ms(j);
        }
        /* ⚠ 只有**确实开着**超时（wd > 0）时，“周期必须小于超时”才有意义。
           0 = 禁用 ⇒ 没有门限要满足，绝不能拿 0 去比较（那会让每个循环命令都被拒）。 */
        if (wd != JSDK_WD_DISABLED_MS && period_ms != 0u && period_ms >= wd) {
            jsdk_joint_seterr(j,
                "control period %u ms >= device break_timeout %u ms: the loop "
                "cannot feed the protocol watchdog (set break_timeout = 0 to "
                "disable it on the device, or shorten the period)",
                (unsigned)period_ms, (unsigned)wd);
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

        jsdk_status_t hst = handshake_verified(ctx, j);

        if (hst != JSDK_OK) return hst;

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

    rc = jsdk_ctx_request_retry(j->ctx, CB_PRI_QUERY, CB_MSG_QUERY_DEVICE_INFO,
                                j->cfg.node_id, NULL, 0u, &rsp,
                                JSDK_CFG_TIMEOUT_MS);
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
        rc = jsdk_ctx_request_retry(j->ctx, CB_PRI_QUERY, CB_MSG_QUERY_ERROR,
                                    j->cfg.node_id, req, 1u, &rsp,
                                    JSDK_CFG_TIMEOUT_MS);
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
    /* 设备真值（标定阶段读回）。⚠ 语义已确认：**0 = 固件关闭了心跳**，不是"没读到" */
    out->heartbeat_rate_ms = j->heartbeat_rate_ms;
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
/**
 * 目标地址上是否有人在应答 —— `allow_retry` 决定“丢一帧要不要补一次”。
 *
 * ⚠ 两个调用场景对重发的需求**正好相反**，所以必须分开：
 *   - `discover()` 的主动扫描（`1..max_probe` 逐个问）：**绝大多数地址本来就没人**，
 *     “等不到应答”是**正常结果**。给它重发等于把不存在节点的等待时间翻倍
 *     （真机实测：16 个地址 × 2 次 × 375 ms ⇒ `scan` 里一半的重发都是这种，
 *     `重发=15` 全是假信号）。
 *   - `set_node_id()` 改号前的“这个号是否已被占用”：**假阴性会造出两个同号设备**，
 *     所以这里宁可多等一个超时也要补一次。
 */
static int probe_node_ex(jsdk_context_t *ctx, uint8_t node_id, int allow_retry)
{
    jsdk_can_frame_t rsp;

    if (!jsdk_ctx_check(ctx) || node_id == 0u) return 0;

    /*
     * 回复一律以 MsgType 0x00 回来，靠 Source 区分设备（见协议手册）。
     * ⚠ 扫描（`allow_retry == 0`）走 `flags = 0`：**既不重发也不记账** ——
     *   “这个号上没人”是正常结果，算进超时统计只会让健康扫描看起来像链路坏了
     *   （真机实测：一条正常的 `scan` 曾报 `超时=15`）。
     */
    return (jsdk_ctx_request(ctx, CB_PRI_QUERY, CB_MSG_QUERY_STATUS, node_id,
                             NULL, 0u, CB_MSG_MIT_CONTROL, &rsp,
                             JSDK_CFG_TIMEOUT_MS / 8u,
                             allow_retry
                                 ? (JSDK_REQ_RETRY | JSDK_REQ_COUNT) : 0u)
            == JSDK_OK) ? 1 : 0;
}

/** 扫描用：**不重发**（“没人应答”就是正常结果，见 `probe_node_ex()`）。 */
int jsdk_ctx_probe_node(jsdk_context_t *ctx, uint8_t node_id)
{
    return probe_node_ex(ctx, node_id, 0);
}

/** 改号前的安全检查用：**允许重发**（假阴性会造出两个同号设备）。 */
int jsdk_ctx_probe_node_strict(jsdk_context_t *ctx, uint8_t node_id)
{
    return probe_node_ex(ctx, node_id, 1);
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
