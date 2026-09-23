/**
 * @file    jsdk_context.c
 * @brief   上下文：生命周期、循环边界、描述符集成、总线级操作
 *
 * 本文件是 L3 的“外壳”：它把 HAL（L1）与协议编解码（L2）串起来，并对外提供
 * `jsdk_context_*`。关节相关逻辑在 `jsdk_joint.c`，看门狗在 `jsdk_watchdog.c`。
 *
 * @par 两个循环边界的分工
 *   cycle_begin()：读时钟 → 排空 RX（上限 rx_burst_limit）→ 解复用（反馈/响应）
 *   cycle_end()  ：推进各关节状态机 → 编码并发送控制帧/keepalive → 刷新记账
 *
 * @par 配置阶段 API（configure / activate / deactivate / discover / *_sdo_read）
 * 允许阻塞：内部用 `jsdk_ctx_wait_response()` 轮询。**不得**在控制循环运行期间调用。
 */

#include "jsdk_core_internal.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ==========================================================================
 * 编译期守卫：上下文必须塞进调用者提供的存储
 * ======================================================================== */

/* 编译期守卫：上下文必须塞进调用者提供的存储。
 *
 * ⚠ 若这里编译失败，说明结构体长大了而 `JSDK_CONTEXT_MAX_SIZE`（公共头）
 *   没同步 —— 那会造成**调用者提供的静态存储被静默写溢出**，是最危险的一类
 *   回归。修法只有两个：缩小结构体，或按实测值上调公共头里的常量。
 *
 * （用“数组大小为负”这一 C99 惯用法，不依赖 C11 的 _Static_assert。） */
typedef char jsdk_ctx_size_ok[(sizeof(jsdk_context_t) <= JSDK_CONTEXT_MAX_SIZE) ? 1 : -1];
typedef char jsdk_joint_size_ok[
    (sizeof(jsdk_joint_t) * JSDK_MAX_JOINTS + 256u <= JSDK_CONTEXT_MAX_SIZE) ? 1 : -1];

/* ==========================================================================
 * 魔数校验
 * ======================================================================== */

int jsdk_ctx_check(const jsdk_context_t *ctx)
{
    return (ctx && ctx->magic == JSDK_CTX_MAGIC) ? 1 : 0;
}

int jsdk_joint_check(const jsdk_joint_t *j)
{
    if (!j || j->magic != JSDK_JOINT_MAGIC) return 0;
    return jsdk_ctx_check(j->ctx);
}

/* ==========================================================================
 * 错误串
 * ------------------------------------------------------------------------
 * 只用 %s / %u / %d；浮点走 jsdk_fmt_f()，避免依赖 libc 的浮点 printf。
 * 永远 NUL 结尾；不是 RT 关键路径的日志，但 cycle 内也会被调用
 * （越界拒绝），因此**不分配、不阻塞**。
 * ======================================================================== */

void jsdk_ctx_seterr(jsdk_context_t *ctx, const char *fmt, ...)
{
    va_list ap;

    if (!ctx) return;

    va_start(ap, fmt);
    (void)vsnprintf(ctx->last_error, sizeof ctx->last_error, fmt, ap);
    va_end(ap);
    ctx->last_error[sizeof ctx->last_error - 1u] = '\0';
}

void jsdk_joint_seterr(jsdk_joint_t *j, const char *fmt, ...)
{
    va_list ap;
    size_t  n;

    if (!j || !j->ctx) return;
    n = (size_t)snprintf(j->ctx->last_error, sizeof j->ctx->last_error,
                         "joint %u(node %u): ", (unsigned)j->index,
                         (unsigned)j->cfg.node_id);
    if (n >= sizeof j->ctx->last_error) n = sizeof j->ctx->last_error - 1u;

    va_start(ap, fmt);
    (void)vsnprintf(j->ctx->last_error + n, sizeof j->ctx->last_error - n, fmt, ap);
    va_end(ap);
    j->ctx->last_error[sizeof j->ctx->last_error - 1u] = '\0';
}

/* ==========================================================================
 * 配置默认值
 * ======================================================================== */

void jsdk_context_config_default(jsdk_context_config_t *cfg)
{
    if (!cfg) return;

    memset(cfg, 0, sizeof *cfg);
    /* hal 全 0（调用者必须填 send/recv/now_ms） */
    cfg->master_id              = CB_DEFAULT_MASTER_ID;
    cfg->is_fd                  = 1u;
    cfg->period_ns              = 0u;
    cfg->state_timeout_ms       = 0u;   /* 0 = 各操作用自己的内置默认（标定 120 s / 回零 5 s） */
    cfg->auto_keepalive         = 1u;
    cfg->clamp_target_position  = 0u;   /* 默认：拒绝并改发安全帧（§6.10） */
    cfg->enable_watchdog_hint   = 0u;   /* 默认：不擅自改客户设备配置 */
    cfg->max_joints             = 0u;
    cfg->rx_burst_limit         = 0u;

    cfg->desc.mode                = JSDK_DESC_DYNAMIC;
    cfg->desc.retain              = JSDK_DESC_RETAIN_ALL;
    cfg->desc.share_by_crc        = 1u;
    cfg->desc.stop_when_satisfied = 1u;
    cfg->desc.timeout_ms          = 0u;
}

/* ==========================================================================
 * 尺寸查询
 * ======================================================================== */

size_t jsdk_context_size(const jsdk_context_config_t *cfg)
{
    (void)cfg;                    /* 容量固定（关节内嵌），与配置无关 */
    return sizeof(jsdk_context_t);
}

/* 注：jsdk_desc_arena_size() 已在 src/proto_cyberbeast/cb_jsondesc_parse.c 实现
   （它按 filter 字符串长度估算，比在上下文层重复一份更准）。 */

/* ==========================================================================
 * 生命周期
 * ======================================================================== */

jsdk_status_t jsdk_context_init(jsdk_context_t *ctx, const jsdk_context_config_t *cfg)
{
    if (!ctx || !cfg) return JSDK_ERR_INVALID_ARG;

    /* ABI 守卫（DESIGN §9「ABI 守卫」验收项）：
         magic == 0        → 全新存储（静态零初始化），正常
         magic == 本后端   → 已初始化，重复 init 属调用方 bug
         其它任意值        → 极可能是**另一个后端**的存储，必须拒绝 */
    if (ctx->magic != 0u && ctx->magic != JSDK_CTX_MAGIC) {
        return JSDK_ERR_INVALID_ARG;
    }
    if (ctx->magic == JSDK_CTX_MAGIC) {
        return JSDK_ERR_BAD_STATE;
    }

    if (!cfg->hal.send || !cfg->hal.recv || !cfg->hal.now_ms) {
        return JSDK_ERR_INVALID_ARG;
    }
    if (cfg->master_id == 0u) {
        /* 0 会让设备**完全不回复**（见 cb_frame.h）——静默失效，必须早拦 */
        return JSDK_ERR_INVALID_ARG;
    }
    if (cfg->is_fd > 1u) return JSDK_ERR_INVALID_ARG;

    if (!cfg->desc.arena || cfg->desc.arena_size == 0u) {
        /* 零 malloc 由调用者提供解析区；没有它连端点都解析不了 */
        return JSDK_ERR_INVALID_ARG;
    }
    if (cfg->desc.filter_count > JSDK_DESC_MAX_FILTERS) {
        return JSDK_ERR_INVALID_ARG;
    }

    /* 平台前提：float 必须是 IEEE-754 单精度，且字节序访问可用 */
    if (cb_frame_selfcheck() != 0) return JSDK_ERR_UNSUPPORTED;

    memset(ctx, 0, sizeof *ctx);
    ctx->cfg = *cfg;                       /* hal 一并拷贝；调用者之后可丢弃 */
    ctx->cfg.magic = JSDK_CTX_MAGIC;
    ctx->nj      = 0u;
    ctx->tx_seq  = 0u;
    ctx->now_ms  = cfg->hal.now_ms(cfg->hal.user);

    /* `desc.arena` 指向调用者的内存，arena_used 也要写回调用者（不能只写副本）。
       生命周期要求：配置结构体与 arena 同寿命（见 jsdk_core_internal.h）。 */
    ctx->arena_used_slot = &((jsdk_context_config_t *)cfg)->desc.arena_used;

    jsdk_ctx_seterr(ctx, "not configured");
    ctx->magic = JSDK_CTX_MAGIC;
    return JSDK_OK;
}

jsdk_status_t jsdk_context_add_joint(jsdk_context_t *ctx,
                                     const jsdk_joint_config_t *jc,
                                     jsdk_joint_t **out_joint)
{
    unsigned cap, i;
    jsdk_joint_t *j;

    if (out_joint) *out_joint = NULL;
    if (!jsdk_ctx_check(ctx) || !jc) return JSDK_ERR_INVALID_ARG;

    /* 关节只能在 configure() 之前加入：configure 会按 nj 逐个标定 */
    if (ctx->desc_present || ctx->fetch_active) return JSDK_ERR_BAD_STATE;

    cap = ctx->cfg.max_joints ? ctx->cfg.max_joints : JSDK_MAX_JOINTS;
    if (cap > JSDK_MAX_JOINTS) cap = JSDK_MAX_JOINTS;
    if (ctx->nj >= cap) return JSDK_ERR_NO_MEMORY;

    if (jc->node_id < CB_NODE_ID_MIN || jc->node_id > CB_NODE_ID_MAX) {
        return JSDK_ERR_INVALID_ARG;
    }
    if (jc->axis != 0u) {
        /* 固件 AXIS_COUNT = 1；提前拒绝优于静默忽略 */
        return JSDK_ERR_UNSUPPORTED;
    }
    for (i = 0u; i < ctx->nj; ++i) {
        if (ctx->joints[i].cfg.node_id == jc->node_id) {
            return JSDK_ERR_INVALID_ARG;   /* 同总线不允许重复 node_id */
        }
    }

    j = &ctx->joints[ctx->nj];
    memset(j, 0, sizeof *j);
    j->ctx   = ctx;
    j->index = (uint8_t)ctx->nj;
    j->cfg   = *jc;
    j->cfg.magic = JSDK_JOINT_MAGIC;
    j->mode  = jc->initial_mode;
    j->enable_mode = jc->initial_mode;
    j->status_flags = (uint16_t)JSDK_JF_SCALE_INVALID;   /* 标定前禁止物理量 API */
    j->magic = JSDK_JOINT_MAGIC;

    ctx->nj++;
    if (out_joint) *out_joint = j;
    return JSDK_OK;
}

jsdk_joint_t *jsdk_ctx_find_joint(jsdk_context_t *ctx, uint8_t node_id)
{
    unsigned i;

    if (!jsdk_ctx_check(ctx)) return NULL;
    for (i = 0u; i < ctx->nj; ++i) {
        if (ctx->joints[i].cfg.node_id == node_id) return &ctx->joints[i];
    }
    return NULL;
}

int jsdk_ctx_any_enabled(const jsdk_context_t *ctx)
{
    unsigned i;

    if (!jsdk_ctx_check(ctx)) return 0;
    for (i = 0u; i < ctx->nj; ++i) {
        if (ctx->joints[i].enabled) return 1;
    }
    return 0;
}

void jsdk_context_destroy(jsdk_context_t *ctx)
{
    unsigned i;

    if (!jsdk_ctx_check(ctx)) return;

    for (i = 0u; i < ctx->nj; ++i) {
        ctx->joints[i].magic = 0u;      /* 使残留句柄立即失效 */
    }
    ctx->nj = 0u;
    ctx->magic = 0u;                    /* 最后清魔数：之后 init 可复用该存储 */
}

void jsdk_context_set_fault_callback(jsdk_context_t *ctx,
                                     jsdk_fault_callback_t cb, void *user)
{
    if (!jsdk_ctx_check(ctx)) return;
    ctx->fault_cb   = cb;
    ctx->fault_user = user;
}

/**
 * 把 arena 实际用量回写到调用方的 `desc.arena_used`。
 *
 * 公共头把它声明为**输出**，而 `jsdk_context_init()` 会复制配置，
 * 所以必须经 `arena_used_slot` 直接写回（见结构体里的说明）。
 */
void jsdk_ctx_publish_arena_used(jsdk_context_t *ctx)
{
    size_t used;

    if (!jsdk_ctx_check(ctx)) return;

    used = (size_t)ctx->store.arena.entry_count * sizeof(jsdk_ep_entry_t)
         + ctx->store.arena.blob_used;
    ctx->cfg.desc.arena_used = used;
    if (ctx->arena_used_slot) *ctx->arena_used_slot = used;
}

/* ==========================================================================
 * 发送
 * ======================================================================== */

int jsdk_ctx_send(jsdk_context_t *ctx, uint8_t pri, uint8_t msgtype,
                  uint8_t dest, const uint8_t *payload, uint8_t len)
{
    jsdk_can_frame_t f;
    int rc;

    if (!jsdk_ctx_check(ctx)) return -1;
    if (len > 64u) return -1;

    memset(&f, 0, sizeof f);
    f.id    = cb_make_id(pri, msgtype, dest, ctx->cfg.master_id, ctx->tx_seq);
    f.len   = len;
    f.flags = (uint8_t)(JSDK_FRAME_EXT | (ctx->cfg.is_fd ? (JSDK_FRAME_FD | JSDK_FRAME_BRS) : 0u));
    if (len > 0u && payload) memcpy(f.data, payload, len);

    ctx->bus.tx_frames++;
    ctx->now_ms = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);   /* 发送前刷新时钟 */

    rc = ctx->cfg.hal.send(ctx->cfg.hal.user, &f);
    if (rc != 0) ctx->bus.tx_failed++;
    return rc;
}

/**
 * 编译期探针：把上下文/关节的真实大小暴露给测试与工具链诊断。
 * （`jsdk_context_size()` 只是个运行时包装，这里给出各部分的分解。）
 */
size_t jsdk_context_joint_size(void) { return sizeof(jsdk_joint_t); }

/* ==========================================================================
 * 接收解复用
 * ======================================================================== */

/**
 * 对端帧格式（Classic / FD）的学习结果。
 *
 * 协议**没有**运行时协商：设备用哪种格式完全由它自己的 `can.config.baud_rate`
 * 决定。主站猜错时，发出去的帧设备**根本不收**，现场只表现为
 * “收到一堆帧（心跳），但我的请求没人应”。所以 SDK 在第一次收到本关节的帧时
 * 会把格式对齐过去（见 `jsdk_ctx_handle_frame()`），并把这个结果暴露出来
 * 让 CLI / 客户如实报告“我改了你的格式”。
 *
 * @return 0 = 还没收到过本关节的帧；1 = 已从 FD 改为 **Classic**；
 *         2 = 已从 Classic 改为 **FD**；3 = 与配置一致（无需调整）；
 *         4 = **显式配置与对端冲突**（格式未改动）
 */
int jsdk_context_framing_learned(const jsdk_context_t *ctx)
{
    if (!ctx) return 0;
    return (int)ctx->framing_learned;
}

/**
 * 处理一帧设备→主站的帧。
 *
 * @return 1 = 已消费；0 = 与本主站无关（丢弃）
 */
int jsdk_ctx_handle_frame(jsdk_context_t *ctx, const jsdk_can_frame_t *f)
{
    uint8_t msgtype = (uint8_t)cb_id_msgtype(f->id);
    uint8_t src     = (uint8_t)cb_id_source(f->id);
    uint8_t dst     = (uint8_t)cb_id_dest(f->id);
    jsdk_joint_t *j;

    if (!jsdk_ctx_check(ctx) || !f) return 0;

    ctx->bus.rx_frames++;
    ctx->bus.last_rx_age_ms = 0u;
    ctx->bus.link_up = 1u;
    ctx->last_rx_ms  = ctx->now_ms;

    /*
     * 学一次对端的**帧格式**（Classic vs FD）。
     *
     * 协议**没有**运行时协商：设备用哪种格式完全由它自己的 `can.config.baud_rate`
     * 决定，主站猜错就是“发出去的帧它根本不收” —— 现场只表现为
     * “收到一堆帧（心跳），但我的请求没人应”（真机实测：1 Mbps Classic 的设备 +
     * 我们默认发 FD ⇒ `desc-info` 报 `0/0 bytes, 198 frames received`）。
     *
     * 所以在**第一次**收到本关节发来的帧时，把主站的格式对齐过去，并记下“已学习”
     * （`jsdk_context_framing_learned()` 供 CLI / 客户报告，见 joint_sdk.h）。
     * ⚠ 只学一次，而且只认**我们关节的 node_id**：总线上别人的帧不该改我们的格式。
     *
     * ⚠⚠ 但**显式配置优先**：`cfg.is_fd_explicit` 置位时（CLI `--classic` /
     * `--data-bitrate`、Python `is_fd=`）绝不动调用者写的值，只报告“冲突”（返回 4）。
     * 理由：自动对齐只是个“猜错补救”，不能变成“你说了不算”——
     * 若悄悄改掉显式配置，调用者看到的 cfg 与实际发出的帧不一致，
     * 而且 8 字节参数的分块读也依赖 `is_fd`（FD 一次 8 B / Classic 一次 4 B）。
     */
    if (ctx->framing_learned == 0u) {
        unsigned k;

        for (k = 0u; k < ctx->nj; ++k) {
            if (ctx->joints[k].cfg.node_id == src) {
                uint8_t peer_fd = (f->flags & JSDK_FRAME_FD) ? 1u : 0u;
                uint8_t cfg_fd  = ctx->cfg.is_fd ? 1u : 0u;

                if (peer_fd == cfg_fd) {
                    ctx->framing_learned = 3u;                  /* 3 = 与配置一致 */
                } else if (ctx->cfg.is_fd_explicit) {
                    ctx->framing_learned = 4u;   /* 4 = 冲突：保留调用者的选择 */
                } else {
                    ctx->cfg.is_fd       = peer_fd;
                    ctx->framing_learned = peer_fd ? 2u : 1u;   /* 1 = 改学 Classic */
                }
                break;
            }
        }
    }

    /* 寻址：设备只会以 dest = master_id 单播回复；广播告警除外 */
    if (dst != ctx->cfg.master_id && !(cb_id_is_broadcast(f->id) && dst == CB_ADDR_BROADCAST)) {
        ctx->bus.rx_dropped++;
        return 0;
    }

    j = jsdk_ctx_find_joint(ctx, src);

    switch (msgtype) {
    /* ---- 所有控制帧的应答都是 MIT 响应格式（0x00） ---- */
    case CB_MSG_MIT_CONTROL:
        if (!j) break;
        jsdk_joint__on_mit_response(j, f->data, f->len);
        return 1;

    /* ---- 心跳 ---- */
    case CB_MSG_HEARTBEAT:
        if (!j) break;
        jsdk_joint__on_heartbeat(j, f->data, f->len);
        return 1;

    /* ---- 查询响应（配置阶段由 wait_response 消费；此处尽力更新反馈） ---- */
    case CB_MSG_QUERY_POS_VEL: {
        cb_query_pos_vel_t pv;
        if (!j) break;
        if (cb_query_decode_pos_vel(f->data, f->len, &pv) != 0) break;
        jsdk_joint__on_pos_vel_turns(j, pv.pos_turns, pv.vel_turns_per_s);
        return 1;
    }
    case CB_MSG_QUERY_CURRENT: {
        cb_query_current_t c;
        if (!j) break;
        if (cb_query_decode_current(f->data, f->len, &c) != 0) break;
        jsdk_joint__on_current_a(j, (double)c.iq_a);
        return 1;
    }
    case CB_MSG_QUERY_TEMPERATURE: {
        cb_query_temp_t t;
        if (!j) break;
        if (cb_query_decode_temp(f->data, f->len, &t) != 0) break;
        jsdk_joint__on_temps(j, (double)t.motor_c, (double)t.fet_c);
        return 1;
    }
    case CB_MSG_QUERY_BUS: {
        cb_query_bus_t bu;
        if (!j) break;
        if (cb_query_decode_bus(f->data, f->len, &bu) != 0) break;
        jsdk_joint__on_bus_volts(j, (double)bu.vbus_v, (double)bu.ibus_a);
        return 1;
    }
    case CB_MSG_FAULT_ALERT:
        /* 设备主动告警广播：只置位，不做业务处理（细节靠 QUERY_ERROR） */
        if (j) jsdk_joint__on_fault_alert(j);
        return 1;

    default:
        break;
    }

    /* 0x20/0x45/0x46 等由等待者消费；走到这里说明没人要它 */
    ctx->bus.rx_dropped++;
    return 0;
}

static int jsdk_ctx_pump_rx(jsdk_context_t *ctx, unsigned limit)
{
    jsdk_can_frame_t f;
    unsigned i;

    for (i = 0u; i < limit; ++i) {
        int n = ctx->cfg.hal.recv(ctx->cfg.hal.user, &f);
        if (n == 0) return 0;
        if (n < 0) {
            /* WP4 链路健康：recv 报错就认为链路不可用，直到下一次成功收到帧。
               不断言“永久 down”—— 总线抖动会自行恢复。 */
            ctx->bus.link_errors++;
            ctx->bus.link_up = 0u;
            return -1;
        }
        (void)jsdk_ctx_handle_frame(ctx, &f);
    }
    return 1;                 /* 达到上限：本周期还有帧没处理完 */
}

/**
 * 阻塞等待某条响应（配置阶段专用）。
 *
 * @par 为什么还要一个自旋上限
 *  虚拟/仿真 HAL 的 `now_ms` 只在测试推进时间轴时变化，`recv` 也可能长期返回 0。
 *  若只靠时间差判断超时，会在这个环境下**死循环**。因此除了时间预算，再加一个
 *  自旋上限兜底（真实 HAL 下时间预算先生效）。
 */
int jsdk_ctx_wait_response(jsdk_context_t *ctx, uint8_t msgtype, uint8_t source,
                           jsdk_can_frame_t *out, uint32_t timeout_ms)
{
    uint32_t t0;
    unsigned spin = 0u;
    const unsigned spin_cap = 200000u;

    if (!jsdk_ctx_check(ctx)) return JSDK_ERR_INVALID_ARG;

    t0 = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);

    for (;;) {
        jsdk_can_frame_t f;
        int n = ctx->cfg.hal.recv(ctx->cfg.hal.user, &f);

        if (n < 0) return JSDK_ERR_TRANSPORT;

        if (n > 0) {
            ctx->bus.rx_frames++;
            ctx->bus.link_up = 1u;
            ctx->last_rx_ms  = ctx->now_ms;
            if (cb_id_msgtype(f.id) == msgtype && cb_id_source(f.id) == source) {
                if (out) *out = f;
                return JSDK_OK;
            }
            (void)jsdk_ctx_handle_frame(ctx, &f);
            continue;
        }

        if (jsdk_elapsed(ctx->cfg.hal.now_ms(ctx->cfg.hal.user), t0) >= timeout_ms) {
            return JSDK_ERR_TIMEOUT;
        }
        if (++spin >= spin_cap) return JSDK_ERR_TIMEOUT;
    }
}

/* ==========================================================================
 * 循环边界
 * ======================================================================== */

jsdk_status_t jsdk_context_cycle_begin(jsdk_context_t *ctx, uint64_t app_time_ns)
{
    unsigned i, limit;
    int rc;

    if (!jsdk_ctx_check(ctx)) return JSDK_ERR_INVALID_ARG;

    (void)app_time_ns;      /* 预留：周期抖动统计（P1） */

    ctx->now_ms = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);
    ctx->in_cycle = 1u;

    for (i = 0u; i < ctx->nj; ++i) {
        ctx->joints[i].fb.valid = 0;      /* 本周期尚无新数据 */
        ctx->joints[i].sent_cycle = 0u;   /* 本周期还没发过控制帧 */
        jsdk_joint__refresh_freshness(&ctx->joints[i]);   /* WP4：age_ms + STALE 位 */
    }

    ctx->bus.last_rx_age_ms = jsdk_elapsed(ctx->now_ms, ctx->last_rx_ms);

    limit = ctx->cfg.rx_burst_limit ? ctx->cfg.rx_burst_limit : JSDK_RX_BURST_DEFAULT;
    rc = jsdk_ctx_pump_rx(ctx, limit);
    if (rc < 0) {
        jsdk_ctx_seterr(ctx, "HAL recv() reported a link error");
        return JSDK_ERR_TRANSPORT;
    }
    return JSDK_OK;
}

jsdk_status_t jsdk_context_cycle_end(jsdk_context_t *ctx)
{
    if (!jsdk_ctx_check(ctx)) return JSDK_ERR_INVALID_ARG;

    ctx->now_ms = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);

    jsdk_joint__cycle_end_all(ctx);
    jsdk_watchdog__cycle_end(ctx);

    ctx->in_cycle = 0u;
    return JSDK_OK;
}

jsdk_status_t jsdk_context_poll(jsdk_context_t *ctx, uint64_t app_time_ns)
{
    jsdk_status_t st = jsdk_context_cycle_begin(ctx, app_time_ns);
    if (st != JSDK_OK) return st;
    return jsdk_context_cycle_end(ctx);
}

/* ==========================================================================
 * 总线状态 / 错误 / ESTOP
 * ======================================================================== */

jsdk_status_t jsdk_context_get_bus_state(jsdk_context_t *ctx, jsdk_bus_state_t *state)
{
    uint8_t online = 0u;
    unsigned i;

    if (!jsdk_ctx_check(ctx) || !state) return JSDK_ERR_INVALID_ARG;

    for (i = 0u; i < ctx->nj; ++i) {
        if (ctx->joints[i].fb.online) online++;
    }

    *state = ctx->bus;
    state->nodes_online = online;
    state->last_rx_age_ms = jsdk_elapsed(ctx->now_ms, ctx->last_rx_ms);
    if (ctx->cfg.hal.bus_status) {
        uint32_t flags = 0u;
        if (ctx->cfg.hal.bus_status(ctx->cfg.hal.user, &flags) == 0) {
            state->hal_bus_flags = flags;
        }
    }
    return JSDK_OK;
}

const char *jsdk_context_last_error(jsdk_context_t *ctx)
{
    if (!jsdk_ctx_check(ctx)) return "invalid context";
    return ctx->last_error;
}

void jsdk_context_estop(jsdk_context_t *ctx)
{
    if (!jsdk_ctx_check(ctx)) return;
    /* MsgType 0xC0 是全局广播（Dest = 0xFF），载荷被固件忽略 */
    (void)jsdk_ctx_send(ctx, CB_PRI_CRITICAL, CB_MSG_ESTOP, CB_ADDR_BROADCAST, NULL, 0u);
    ctx->tx_seq = cb_seq_next(ctx->tx_seq);
    jsdk_ctx_seterr(ctx, "ESTOP broadcast sent");
}
