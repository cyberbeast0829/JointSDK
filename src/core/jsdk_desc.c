/**
 * @file    jsdk_desc.c
 * @brief   描述符集成：0x24/0x25 收发、节点间共享（share_by_crc）、缓存三条出口
 *
 * 单帧级传输状态机在 `src/proto_cyberbeast/cb_jsondesc_fetch.c`（已独立测试），
 * 本文件只负责"把它接到上下文上"：谁来发请求、排空 RX、何时算超时、
 * 以及从哪个节点下载、能不能复用别的节点已经下好的结果。
 */

#include "jsdk_core_internal.h"

#include <string.h>

/* ==========================================================================
 * 内部工具
 * ======================================================================== */

/** 单帧能装多少描述符字节（帧长 − 2 字节头）。 */
static uint32_t desc_payload_per_frame(int is_fd)
{
    return (uint32_t)((is_fd ? CB_DESC_FRAME_LEN_FD : CB_DESC_FRAME_LEN_CLASSIC)
                      - CB_DESC_DATA_HDR_BYTES);
}

/** 完整描述符需要多少帧（1 个元数据帧 + N 个数据帧）。 */
static uint32_t desc_frames_needed(uint32_t total_len, int is_fd)
{
    uint32_t ppf = desc_payload_per_frame(is_fd);
    if (ppf == 0u) return 0u;
    return 1u + (total_len + ppf - 1u) / ppf;
}

/**
 * 整条下载的**总时长兜底**（ms）。
 *
 * 真正的判据是“静默了多久”（`cb_desc_timeout_ms()`，可被 `desc.timeout_ms` /
 * CLI 的 `--timeout` 覆盖）；这条只是防止“每帧都慢、但一直在动”的病态流
 * 把配置阶段挂死。取值宽松：Classic 实测整条流 ~3.5 s。
 */
#define JSDK_DESC_TOTAL_MAX_MS 120000u

static void store_init(jsdk_context_t *ctx)
{
    memset(&ctx->store, 0, sizeof ctx->store);
    ctx->store.arena.base     = (uint8_t *)ctx->cfg.desc.arena;
    ctx->store.arena.size     = ctx->cfg.desc.arena_size;
    ctx->store.arena.blob_top = ctx->cfg.desc.arena_size;
    ctx->store.max_endpoints  = ctx->cfg.desc.max_endpoints
                                ? ctx->cfg.desc.max_endpoints : 2048u;
}

/**
 * 丢掉 RX 里**已有的**帧，直到总线安静下来（连续 @p quiet_ms 没有新帧）。
 *
 * ⚠ 请求 `0x24` 之前必须做，真机（slcan）实测的现场：
 *   上一条被中断的传输（超时 / Ctrl-C / 报错退出）会让设备停在半路、继续把
 *   剩下的帧发完；我们重新打开适配器（`C`→`Y5`→`O`）时它们就涌出来。
 *   不清掉的话，这些残留会：
 *     1. 被 fetch 当成"本次的元数据帧"（fetch 自己也会跳过，但那是有界的）；
 *     2. **吃掉 `desc.timeout_ms` 的预算** —— 真机上残留尾巴可以有 1.5 s 以上，
 *        而一次完整的 38 KB 传输本身也要 ~1.3 s，3 s 默认超时直接不够，
 *        表现为"第一次连不上、第二次就好了"（实测）。
 *   所以这里要**等它发完**再请求：请求前清干净，超时预算就全留给自己的传输。
 *
 * 这里只读"已经排好的"帧，不主动等待数据；唯一的等待是"静默窗口"，
 * 而且有 @p max_frames 与 @p quiet_ms 双重上限，不会把调用者卡死。
 *
 * @param max_frames 最多丢多少帧（硬上限）
 * @param quiet_ms   连续多久没有帧就算"安静"（典型 100 ms）
 */
static void desc_drain_rx(jsdk_context_t *ctx, unsigned max_frames, uint32_t quiet_ms)
{
    jsdk_can_frame_t junk;
    uint32_t         last;
    unsigned         i;

    if (!ctx->cfg.hal.recv) return;

    last = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);
    for (i = 0u; i < max_frames; ++i) {
        if (ctx->cfg.hal.recv(ctx->cfg.hal.user, &junk) == 1) {
            last = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);
            continue;
        }
        if ((uint32_t)(ctx->cfg.hal.now_ms(ctx->cfg.hal.user) - last) >= quiet_ms) break;
    }
}

/* --------------------------------------------------------------------------
 * “首个请求丢了”的重发策略
 *
 * ⚠ 真机实测（slcan + CANable）：打开端口之后的**第一条请求概率性丢失**
 *   （适配器在 `C`/`Y5`/`O` 之后还要配置自己的 CAN 控制器）。丢的次数不确定，
 *   单次 250 ms 重试会撞上“还没就绪”，表现就是“第一次跑失败、再跑一次成功”。
 *   设备对 `0x24` 幂等（重复请求会从头重发），所以多试几次无副作用。
 *
 * 递增间隔：0.25 s → 0.6 s → 1.2 s（共 4 次尝试）。仅当**一帧都没收到**时启用。
 * ------------------------------------------------------------------------ */

#define JSDK_DESC_REQUEST_RETRIES 3u

static uint32_t jsdk_desc_retry_delay_ms(unsigned retry_index)
{
    static const uint16_t k_delay_ms[JSDK_DESC_REQUEST_RETRIES] = { 250u, 600u, 1200u };

    return (retry_index < JSDK_DESC_REQUEST_RETRIES) ? k_delay_ms[retry_index]
                                                    : 0xFFFFFFFFu;
}

/** 至少有一个关节已被使能 → 禁止下载（DESIGN §6.6 硬约束 1）。 */
/**
 * 从 @p node 完整下载并解析描述符（**阻塞**）。
 *
 * 排空 RX 时**不受** `rx_burst_limit` 限制（硬约束 2）：662 帧连续到达，
 * 按默认 32 帧/周期处理会丢帧导致 JSON 断裂。
 */
static jsdk_status_t desc_fetch_from(jsdk_context_t *ctx, uint8_t node)
{
    uint8_t req[CB_DESC_REQ_LEN];
    uint32_t deadline;
    size_t   req_len;
    uint32_t spin_clock = 0u;   /* 与 frozen 一起做"时钟冻结"检测 */
    unsigned frozen = 0u;

    store_init(ctx);

    if (cb_desc_fetch_init(&ctx->fetch, ctx, &ctx->cfg.desc, &ctx->store) != JSDK_OK) {
        return JSDK_ERR_INVALID_ARG;
    }
    ctx->fetch_active = 1;
    ctx->desc_retries = 0u;                  /* 本次下载的重发计数从 0 开始 */
    cb_desc_fetch_set_raw_sink(&ctx->fetch, ctx->raw_sink, ctx->raw_sink_user);
    cb_desc_fetch_set_progress(&ctx->fetch, ctx->progress, ctx->progress_user);

    req_len = cb_desc_build_request(req, sizeof req, 0u);
    if (req_len == 0u) return JSDK_ERR_INVALID_ARG;

    /*
     * 上限取 4096 帧 / 100 ms 静默：真机上一条被中断的流的尾巴可以有几百帧、
     * 持续 1 s 以上（实测），要等它发完再请求，否则残留会占掉超时预算。
     */
    desc_drain_rx(ctx, 4096u, 100u);

    if (jsdk_ctx_send(ctx, CB_PRI_CONFIG, CB_MSG_JSON_DESC_READ, node,
                      req, (uint8_t)req_len) != 0) {
        jsdk_ctx_seterr(ctx, "descriptor request to node %u could not be sent",
                        (unsigned)node);
        return JSDK_ERR_TRANSPORT;
    }
    ctx->tx_seq = cb_seq_next(ctx->tx_seq);

    deadline = cb_desc_timeout_ms(&ctx->cfg.desc);
    {
        uint32_t t0 = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);
        uint32_t req_at = t0;      /* 上一次发请求的时刻（用于重试判定） */
        uint32_t last_progress = t0;   /* 上一次**有新字节**的时刻（静默预算的起点） */
        uint32_t bytes_prev = 0u;
        unsigned retries = 0u;     /* 已重发次数（见下面的递增间隔） */

        while (!cb_desc_fetch_is_done(&ctx->fetch)) {
            jsdk_can_frame_t f;
            int n = ctx->cfg.hal.recv(ctx->cfg.hal.user, &f);

            if (n < 0) return JSDK_ERR_TRANSPORT;
            if (n > 0) {
                ctx->bus.rx_frames++;
                ctx->last_rx_ms = ctx->now_ms;
                ctx->bus.link_up = 1u;
                if (cb_id_msgtype(f.id) == CB_MSG_JSON_DESC_DATA
                    && cb_id_source(f.id) == node) {
                    (void)cb_desc_fetch_frame(&ctx->fetch, f.data, f.len);
                } else {
                    (void)jsdk_ctx_handle_frame(ctx, &f);
                }
                continue;
            }

            ctx->now_ms = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);

            /*
             * ⚠⚠ 预算算的是“**多久没有新字节**”，不是“总共多久”。
             *
             * 描述符是**流式**的：同一个 38433 字节的描述符，FD 是 1+662 帧，
             * 而 **Classic 是 1+6906 帧（10.4 倍）**。真机实测（CyberBeast USB2CAN
             * / 1 Mbps Classic）：连续有进展时整条流只要 ~3.5 s；但按“总预算
             * 3000 ms”算就会在 85% 处被掐断 —— 报出来的却是
             * `timed out after 3000 ms (32982/38433 bytes, 6057 frames received)`，
             * 看上去像设备或线缆有问题（实际上是量纲错了）。
             */
            if (ctx->fetch.bytes_scanned != bytes_prev) {
                bytes_prev    = ctx->fetch.bytes_scanned;
                last_progress = ctx->now_ms;
            }

            /*
             * 请求重发（最多 3 次，共 4 次尝试）。
             *
             * ⚠ 真机实测：**刚打开适配器后的第一条请求可能丢**（`C`/`Y5`/`O`
             *   之后适配器还要配置自己的 CAN 控制器），现场表现就是"第一次连不上、
             *   再敲一次就好了"。丢失概率与延迟**不确定**，所以按递增间隔多试几次；
             *   设备对 `0x24` 幂等（会从头重发），重试无副作用。
             *
             * ⚠⚠ 判据只能是"**本次传输还没开始**"（没收到元数据帧），**不能**是
             *   "一帧都没收到"：RX 里常常有上一次被中断传输的**残留帧**，
             *   真机实测过 680 帧残留、而我们的请求根本没到达设备 ——
             *   用帧数当判据就永远不重发，然后超时（那次的报错正是
             *   `(0/0 bytes, 680 frames received)`）。
             */
            if (retries < JSDK_DESC_REQUEST_RETRIES
                && !cb_desc_fetch_started(&ctx->fetch)
                && jsdk_elapsed(ctx->now_ms, req_at) >= jsdk_desc_retry_delay_ms(retries)) {
                retries++;
                ctx->desc_retries++;
                req_at = ctx->now_ms;
                if (jsdk_ctx_send(ctx, CB_PRI_CONFIG, CB_MSG_JSON_DESC_READ, node,
                                  req, (uint8_t)req_len) == 0) {
                    ctx->tx_seq = cb_seq_next(ctx->tx_seq);
                }
            }

            if (jsdk_elapsed(ctx->now_ms, last_progress) >= deadline) {
                /* `rx` = 一共收到过多少帧：区分两种完全不同的故障
                   （通道没开 → 0；请求丢了但心跳在流 → >0）。
                   ⚠ 把“静默了多久”“已经收了多久”都写出来：否则用户分不清
                     “设备卡住”与“预算量纲写错”（后者真发生过，见上面的注释）。 */
                jsdk_ctx_seterr(ctx, "descriptor download from node %u stalled: "
                                     "no new bytes for %u ms "
                                     "(%u/%u bytes, %u frames received, total %u ms)",
                                (unsigned)node, (unsigned)deadline,
                                (unsigned)ctx->fetch.bytes_scanned,
                                (unsigned)ctx->fetch.total_len,
                                (unsigned)ctx->bus.rx_frames,
                                (unsigned)jsdk_elapsed(ctx->now_ms, t0));
                return JSDK_ERR_TIMEOUT;
            }
            /* 总时长兜底：防“每帧都慢但一直在动”的病态流把这里挂死 */
            if (jsdk_elapsed(ctx->now_ms, t0) >= JSDK_DESC_TOTAL_MAX_MS) {
                jsdk_ctx_seterr(ctx, "descriptor download from node %u exceeded the "
                                     "overall cap of %u ms (%u/%u bytes)",
                                (unsigned)node, (unsigned)JSDK_DESC_TOTAL_MAX_MS,
                                (unsigned)ctx->fetch.bytes_scanned,
                                (unsigned)ctx->fetch.total_len);
                return JSDK_ERR_TIMEOUT;
            }
            /*
             * "卡死"只认**时钟不动**这一种情况。
             *
             * ⚠ 旧实现看的是**迭代次数**（`spin >= spin_cap`，400 万次）。那条
             *   判断在真机上是**误报**：空缓冲上的 `recv` 是**立即返回**的，
             *   400 万次循环只需几十毫秒 —— 而真实设备从收到 `0x24` 到开口
             *   可能要 100 ms 以上（slcan 上还要排队），于是第一次连接会随机
             *   报 `stalled (no frames; frozen clock?)`（真机实测）。
             *   它本来的用途只是"虚拟 HAL 没开 autotick、时钟冻结"时不要死循环，
             *   那就直接看时钟有没有走：走了就交给上面的超时负责。
             */
            if (ctx->now_ms == spin_clock) {
                if (++frozen >= 4000000u) {
                    jsdk_ctx_seterr(ctx, "descriptor download from node %u stalled "
                                         "(no frames and the clock is frozen?)",
                                    (unsigned)node);
                    return JSDK_ERR_TIMEOUT;
                }
            } else {
                spin_clock = ctx->now_ms;
                frozen     = 0u;
            }
        }
    }

    /* 把传输结果搬进 desc_info */
    {
        cb_desc_fetch_result_t r;
        cb_desc_fetch_result(&ctx->fetch, &r);

        ctx->desc.total_len     = r.total_len;
        ctx->desc.crc           = r.crc;
        ctx->desc.endpoint_count = r.endpoint_count;
        ctx->desc.parsed_total  = r.parsed_total;
        ctx->desc.frames_rx     = r.frames_rx;
        ctx->desc.bytes_scanned = r.bytes_scanned;
        ctx->desc.complete      = r.complete;
        ctx->desc.raw_sink_failed = r.raw_sink_failed;
        ctx->cfg.desc.arena_used = r.arena_used;
    }

    if (cb_desc_fetch_error(&ctx->fetch) != CB_DESC_ERR_NONE) {
        jsdk_ctx_seterr(ctx, "descriptor from node %u rejected: %s (%s)",
                        (unsigned)node, cb_desc_fetch_error(&ctx->fetch),
                        cb_desc_fetch_detail(&ctx->fetch)
                            ? cb_desc_fetch_detail(&ctx->fetch) : "-");
        return (strcmp(cb_desc_fetch_error(&ctx->fetch), CB_DESC_ERR_ARENA) == 0)
                   ? JSDK_ERR_NO_MEMORY : JSDK_ERR_PARSE;
    }
    return JSDK_OK;
}

/**
 * 只读一个节点的描述符**元数据帧**（total_len / crc），随后排空其余帧。
 *
 * 用途：`share_by_crc`。设备不提供"只问 CRC"的接口，但元数据帧恒定是每次请求的
 * 第一帧，所以代价 = 1 次请求 + 1 次整份排空（不解析）。
 *
 * @note 必须在 RX 队列里没有残留 0x25 帧时调用（`desc_fetch_from` 会排空）。
 */
static jsdk_status_t desc_probe_meta(jsdk_context_t *ctx, uint8_t node,
                                     uint16_t *out_crc, uint32_t *out_total)
{
    uint8_t req[CB_DESC_REQ_LEN];
    uint32_t t0, deadline;
    uint32_t seen = 0u;
    unsigned spin = 0u;
    const unsigned spin_cap = 4000000u;
    size_t   req_len;
    uint32_t total = 0u;
    uint16_t crc   = 0u;

    req_len = cb_desc_build_request(req, sizeof req, 0u);
    if (req_len == 0u) return JSDK_ERR_INVALID_ARG;
    if (jsdk_ctx_send(ctx, CB_PRI_CONFIG, CB_MSG_JSON_DESC_READ, node,
                      req, (uint8_t)req_len) != 0) {
        return JSDK_ERR_TRANSPORT;
    }
    ctx->tx_seq = cb_seq_next(ctx->tx_seq);

    deadline = cb_desc_timeout_ms(&ctx->cfg.desc);
    t0 = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);

    for (;;) {
        jsdk_can_frame_t f;
        int n = ctx->cfg.hal.recv(ctx->cfg.hal.user, &f);

        if (n < 0) return JSDK_ERR_TRANSPORT;
        if (n > 0) {
            ctx->bus.rx_frames++;
            ctx->last_rx_ms = ctx->now_ms;
            if (cb_id_msgtype(f.id) == CB_MSG_JSON_DESC_DATA
                && cb_id_source(f.id) == node) {
                seen++;
                if (seen == 1u) {
                    /* 元数据帧：[0..1]=0 + totalLen u32 LE @2 + crc u16 LE @6 */
                    if (f.len < CB_DESC_META_MIN_LEN
                        || f.data[0] != 0u || f.data[1] != 0u) {
                        return JSDK_ERR_PROTOCOL;
                    }
                    total = cb_le_get_u32(f.data + 2);
                    crc   = cb_le_get_u16(f.data + 6);
                    if (out_crc)   *out_crc   = crc;
                    if (out_total) *out_total = total;
                }
                if (seen >= desc_frames_needed(total != 0u ? total : 1u,
                                               ctx->cfg.is_fd)) {
                    return JSDK_OK;      /* 排空完毕 */
                }
            } else {
                (void)jsdk_ctx_handle_frame(ctx, &f);
            }
            continue;
        }

        ctx->now_ms = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);
        if (jsdk_elapsed(ctx->now_ms, t0) >= deadline) return JSDK_ERR_TIMEOUT;
        if (++spin >= spin_cap) return JSDK_ERR_TIMEOUT;
    }
}

/* ==========================================================================
 * 公共：下载
 * ======================================================================== */

jsdk_status_t jsdk_context_desc_fetch(jsdk_context_t *ctx)
{
    uint8_t node;
    jsdk_status_t st;

    if (!jsdk_ctx_check(ctx)) return JSDK_ERR_INVALID_ARG;
    if (ctx->nj == 0u) return JSDK_ERR_BAD_STATE;
    if (jsdk_ctx_any_enabled(ctx)) {
        jsdk_ctx_seterr(ctx, "descriptor download refused: a joint is enabled "
                             "(JSON frames would starve the control frames)");
        return JSDK_ERR_BAD_STATE;
    }

    node = ctx->joints[0].cfg.node_id;

    /*
     * 会话预热（幂等 + 重发）由 `jsdk_ctx_send()` 自动完成 —— 也就是下面
     * `desc_fetch_from()` 发的**第一个**请求之前。不做的话，丢的会是我们这
     * 38 KB 流的第一个请求（下载本身也重发，所以能自愈 —— 但那是“事后补救”；
     * 预热让它压根别发生，而且顺带把这条链是否活着先确认掉）。
     *
     * ⚠ 这里**不**显式调用：只收不发的场景（`hb-dump` 这类）压根不需要设备先
     *   应答，而自动钩子挂在“发帧之前”，天然跳过它们。
     */
    st = desc_fetch_from(ctx, node);
    if (st != JSDK_OK) {
        ctx->desc_present = 0u;
        ctx->fetch_active = 0;      /* 传输已死：别让 desc_poll 继续推进它 */
        return st;
    }

    ctx->desc_present  = 1u;
    ctx->desc.mode_used = ctx->cfg.desc.mode;
    jsdk_ctx_publish_arena_used(ctx);

    /* 设备信息（fw/hw）同时是缓存与共享的键 */
    {
        jsdk_device_info_t info;
        if (jsdk_joint_get_device_info(&ctx->joints[0], &info) == JSDK_OK) {
            ctx->desc.fw_version = info.fw_version;
            ctx->desc.hw_version = info.hw_version;
        }
    }

    /* --- 其余节点：CRC 相同则复用（share_by_crc） --- */
    if (ctx->cfg.desc.share_by_crc) {
        unsigned i;
        for (i = 1u; i < ctx->nj; ++i) {
            jsdk_joint_t *j = &ctx->joints[i];
            uint16_t crc = 0u;
            uint32_t total = 0u;

            if (desc_probe_meta(ctx, j->cfg.node_id, &crc, &total) == JSDK_OK
                && crc == ctx->desc.crc) {
                j->shared_desc = 1u;
                continue;
            }
            /* 不一致（或探测失败）→ 老老实实重下一份，以最后一个为准 */
            {
                jsdk_status_t s2 = desc_fetch_from(ctx, j->cfg.node_id);
                if (s2 != JSDK_OK) {
                    jsdk_ctx_seterr(ctx, "descriptor mismatch on node %u and "
                                         "re-download failed",
                                    (unsigned)j->cfg.node_id);
                    return s2;
                }
            }
        }
        ctx->desc.shared_hit = (ctx->joints[0].shared_desc
                                || (ctx->nj > 1u && ctx->joints[1].shared_desc))
                               ? 1u : 0u;
    }

    if (ctx->desc.complete == 0u) {
        /* 提前终止（精确 filter）→ 端点表本身可用，但原始字节不完整 */
        jsdk_ctx_seterr(ctx, "descriptor scanned %u of %u bytes (early stop; "
                             "raw caching is NOT safe for this run)",
                        (unsigned)ctx->desc.bytes_scanned, (unsigned)ctx->desc.total_len);
    } else {
        jsdk_ctx_seterr(ctx, "descriptor ok: %u endpoints, %u bytes, crc=0x%04X",
                        ctx->desc.endpoint_count, (unsigned)ctx->desc.total_len,
                        (unsigned)ctx->desc.crc);
    }
    return JSDK_OK;
}

jsdk_status_t jsdk_context_desc_poll(jsdk_context_t *ctx, uint64_t app_time_ns)
{
    (void)app_time_ns;

    if (!jsdk_ctx_check(ctx)) return JSDK_ERR_INVALID_ARG;
    if (ctx->nj == 0u) return JSDK_ERR_BAD_STATE;

    if (!ctx->fetch_active) {
        uint8_t req[CB_DESC_REQ_LEN];
        size_t  req_len;
        uint8_t node = ctx->joints[0].cfg.node_id;

        if (ctx->desc_present) return JSDK_OK;
        if (jsdk_ctx_any_enabled(ctx)) return JSDK_ERR_BAD_STATE;

        store_init(ctx);
        if (cb_desc_fetch_init(&ctx->fetch, ctx, &ctx->cfg.desc, &ctx->store) != JSDK_OK) {
            return JSDK_ERR_INVALID_ARG;
        }
        cb_desc_fetch_set_raw_sink(&ctx->fetch, ctx->raw_sink, ctx->raw_sink_user);
        cb_desc_fetch_set_progress(&ctx->fetch, ctx->progress, ctx->progress_user);
        ctx->fetch_active = 1;
        ctx->desc_retries = 0u;              /* 非阻塞路径同理 */

        req_len = cb_desc_build_request(req, sizeof req, 0u);
        if (req_len == 0u) return JSDK_ERR_INVALID_ARG;
        /* 与阻塞路径同理：先等总线安静下来再请求 */
        desc_drain_rx(ctx, 1024u, 50u);
        if (jsdk_ctx_send(ctx, CB_PRI_CONFIG, CB_MSG_JSON_DESC_READ, node,
                          req, (uint8_t)req_len) != 0) {
            ctx->fetch_active = 0;
            return JSDK_ERR_TRANSPORT;
        }
        ctx->tx_seq = cb_seq_next(ctx->tx_seq);
        ctx->fetch_bytes_seen  = 0u;
        ctx->fetch_deadline_ms = ctx->cfg.hal.now_ms(ctx->cfg.hal.user)
                               + cb_desc_timeout_ms(&ctx->cfg.desc);
        return JSDK_ERR_BUSY;
    }

    /* 推进：每周期最多处理 64 帧（非阻塞路径，调用者自己控制节奏） */
    for (unsigned i = 0u; i < 64u && !cb_desc_fetch_is_done(&ctx->fetch); ++i) {
        jsdk_can_frame_t f;
        int n = ctx->cfg.hal.recv(ctx->cfg.hal.user, &f);
        if (n <= 0) break;
        ctx->bus.rx_frames++;
        ctx->last_rx_ms = ctx->now_ms;
        if (cb_id_msgtype(f.id) == CB_MSG_JSON_DESC_DATA
            && cb_id_source(f.id) == ctx->joints[0].cfg.node_id) {
            (void)cb_desc_fetch_frame(&ctx->fetch, f.data, f.len);
        } else {
            (void)jsdk_ctx_handle_frame(ctx, &f);
        }
    }

    ctx->now_ms = ctx->cfg.hal.now_ms(ctx->cfg.hal.user);
    if (!cb_desc_fetch_is_done(&ctx->fetch)) {
        /* 与阻塞路径同一套语义：**有新字节就续命**（静默预算，不是总预算）。
           Classic 的描述符是 6906 帧（FD 的 10.4 倍），按总预算算必被误判。 */
        if (ctx->fetch.bytes_scanned != ctx->fetch_bytes_seen) {
            ctx->fetch_bytes_seen  = ctx->fetch.bytes_scanned;
            ctx->fetch_deadline_ms = ctx->now_ms + cb_desc_timeout_ms(&ctx->cfg.desc);
        }
        if ((int32_t)(ctx->now_ms - ctx->fetch_deadline_ms) >= 0) {
            ctx->fetch_active = 0;
            jsdk_ctx_seterr(ctx, "descriptor poll stalled: no new bytes for %u ms "
                                 "(%u/%u bytes)",
                            (unsigned)cb_desc_timeout_ms(&ctx->cfg.desc),
                            (unsigned)ctx->fetch.bytes_scanned,
                            (unsigned)ctx->fetch.total_len);
            return JSDK_ERR_TIMEOUT;
        }
        return JSDK_ERR_BUSY;
    }

    ctx->fetch_active = 0;
    {
        cb_desc_fetch_result_t r;
        cb_desc_fetch_result(&ctx->fetch, &r);
        ctx->desc.total_len      = r.total_len;
        ctx->desc.crc            = r.crc;
        ctx->desc.endpoint_count = r.endpoint_count;
        ctx->desc.parsed_total   = r.parsed_total;
        ctx->desc.frames_rx      = r.frames_rx;
        ctx->desc.bytes_scanned  = r.bytes_scanned;
        ctx->desc.complete       = r.complete;
        ctx->desc.raw_sink_failed= r.raw_sink_failed;
        ctx->desc.mode_used      = ctx->cfg.desc.mode;
        ctx->cfg.desc.arena_used = r.arena_used;
    }
    if (cb_desc_fetch_error(&ctx->fetch) != CB_DESC_ERR_NONE) {
        ctx->desc_present = 0u;
        return (strcmp(cb_desc_fetch_error(&ctx->fetch), CB_DESC_ERR_ARENA) == 0)
                   ? JSDK_ERR_NO_MEMORY : JSDK_ERR_PARSE;
    }
    ctx->desc_present = 1u;
    jsdk_ctx_publish_arena_used(ctx);
    return JSDK_OK;
}

/* ==========================================================================
 * 公共：进度 / tee / 元信息
 * ======================================================================== */

void jsdk_context_set_desc_progress(jsdk_context_t *ctx,
                                    jsdk_desc_progress_fn fn, void *user)
{
    if (!jsdk_ctx_check(ctx)) return;
    ctx->progress = fn;
    ctx->progress_user = user;
}

void jsdk_context_set_desc_raw_sink(jsdk_context_t *ctx,
                                    jsdk_desc_raw_sink_fn fn, void *user)
{
    if (!jsdk_ctx_check(ctx)) return;
    ctx->raw_sink = fn;
    ctx->raw_sink_user = user;
}

jsdk_status_t jsdk_context_get_desc_info(jsdk_context_t *ctx, jsdk_desc_info_t *info)
{
    if (!jsdk_ctx_check(ctx) || !info) return JSDK_ERR_INVALID_ARG;
    if (!ctx->desc_present) return JSDK_ERR_BAD_STATE;
    *info = ctx->desc;
    info->endpoint_count = jsdk_ep_store_count(&ctx->store);
    info->parsed_total   = ctx->store.parsed_total;
    info->retries        = ctx->desc_retries;
    return JSDK_OK;
}

/* ==========================================================================
 * 公共：查询 / 遍历
 * ======================================================================== */

jsdk_status_t jsdk_endpoint_lookup(jsdk_context_t *ctx, const char *path,
                                   uint16_t *ep_id, jsdk_ep_type_t *type,
                                   uint8_t *access)
{
    if (!jsdk_ctx_check(ctx) || !path) return JSDK_ERR_INVALID_ARG;
    if (!ctx->desc_present) return JSDK_ERR_BAD_STATE;
    return jsdk_ep_store_lookup(&ctx->store, path, ep_id, type, access);
}

jsdk_status_t jsdk_endpoint_enumerate(jsdk_context_t *ctx,
                                      jsdk_endpoint_visit_fn fn, void *user)
{
    unsigned i, n;

    if (!jsdk_ctx_check(ctx) || !fn) return JSDK_ERR_INVALID_ARG;
    if (!ctx->desc_present) return JSDK_ERR_BAD_STATE;

    n = jsdk_ep_store_count(&ctx->store);
    for (i = 0u; i < n; ++i) {
        const char *path = NULL;
        uint16_t ep_id = 0u;
        jsdk_ep_type_t type = JSDK_EP_JSON;
        uint8_t access = 0u;

        if (jsdk_ep_store_at(&ctx->store, i, &path, &ep_id, &type, &access) != JSDK_OK) {
            continue;
        }
        if (fn(user, path, ep_id, type, access) != 0) break;
    }
    return JSDK_OK;
}

/* ==========================================================================
 * 公共：缓存（路线 A / 路线 B）
 * ======================================================================== */

size_t jsdk_desc_export_max_size(const jsdk_context_t *ctx)
{
    if (!jsdk_ctx_check(ctx)) return 0u;
    return cb_desc_cache_size(&ctx->store);
}

jsdk_status_t jsdk_context_desc_export(jsdk_context_t *ctx, void *buf, size_t cap,
                                       size_t *out_len)
{
    int early;

    if (!jsdk_ctx_check(ctx)) return JSDK_ERR_INVALID_ARG;
    if (!ctx->desc_present) return JSDK_ERR_BAD_STATE;

    early = (ctx->desc.complete == 0u) ? 1 : 0;
    {
        /* 导出内容与运行时 arena 无关，但头里的失效键必须用**同一份配置** */
        jsdk_desc_config_t c = ctx->cfg.desc;
        return cb_desc_cache_export(&c, &ctx->store, early, buf, cap, out_len);
    }
}

jsdk_status_t jsdk_context_desc_import(jsdk_context_t *ctx, const void *buf, size_t len)
{
    cb_desc_cache_meta_t meta;
    jsdk_status_t st;

    if (!jsdk_ctx_check(ctx) || !buf) return JSDK_ERR_INVALID_ARG;
    if (len == 0u) return JSDK_ERR_INVALID_ARG;
    if (!ctx->cfg.desc.arena) return JSDK_ERR_INVALID_ARG;

    st = cb_desc_cache_peek(buf, len, &meta);
    if (st != JSDK_OK) return st;

    store_init(ctx);
    st = cb_desc_cache_import(&ctx->cfg.desc, &ctx->store, buf, len);
    if (st != JSDK_OK) {
        ctx->desc_present = 0u;
        jsdk_ctx_seterr(ctx, "descriptor import rejected (%s); cache slot should "
                             "be invalidated and re-downloaded", jsdk_status_string(st));
        return st;
    }

    memset(&ctx->desc, 0, sizeof ctx->desc);
    ctx->desc.total_len      = 0u;
    ctx->desc.crc            = meta.desc_crc;
    ctx->desc.fw_version     = meta.fw_version;
    ctx->desc.endpoint_count = jsdk_ep_store_count(&ctx->store);
    ctx->desc.parsed_total   = ctx->store.parsed_total;
    ctx->desc.complete       = (uint8_t)((meta.flags & CB_DESC_CACHE_F_EARLY) ? 0u : 1u);
    ctx->desc.mode_used      = (uint8_t)JSDK_DESC_FROM_CACHE;
    ctx->desc.raw_sink_failed = 0u;
    ctx->desc_present = 1u;
    jsdk_ctx_publish_arena_used(ctx);
    return JSDK_OK;
}

jsdk_status_t jsdk_context_desc_import_raw(jsdk_context_t *ctx,
                                           const void *json, size_t len,
                                           const jsdk_desc_hint_t *hint)
{
    int rc;

    if (!jsdk_ctx_check(ctx) || !json || !hint) return JSDK_ERR_INVALID_ARG;
    if (len == 0u) return JSDK_ERR_INVALID_ARG;
    if (!ctx->cfg.desc.arena) return JSDK_ERR_INVALID_ARG;

    store_init(ctx);
    rc = jsdk_jsondesc_run(&ctx->cfg.desc, &ctx->store, json, len);
    if (rc != JSDK_OK) {
        ctx->desc_present = 0u;
        jsdk_ctx_seterr(ctx, "raw descriptor import failed (%s)", jsdk_status_string(rc));
        return rc;
    }

    memset(&ctx->desc, 0, sizeof ctx->desc);
    ctx->desc.total_len      = (uint32_t)len;
    ctx->desc.crc            = hint->crc;
    ctx->desc.fw_version     = hint->fw_version;
    ctx->desc.endpoint_count = jsdk_ep_store_count(&ctx->store);
    ctx->desc.parsed_total   = ctx->store.parsed_total;
    ctx->desc.complete       = 1u;      /* 整份 JSON 都在手里 */
    ctx->desc.mode_used      = (uint8_t)JSDK_DESC_FROM_CACHE;
    ctx->desc_present = 1u;
    jsdk_ctx_publish_arena_used(ctx);
    return JSDK_OK;
}
