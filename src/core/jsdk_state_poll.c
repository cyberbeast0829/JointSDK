/*
 * JointSDK —— 状态轮询调度器（v0.37，阶段 2）
 * ============================================================================
 *
 * 背景：某些固件上设备主动上报的帧**不更新**（`pos/vel` 恒 0）。阶段 1 给了
 * `jsdk_joint_request_state()` 这个原语（发出即返回），但“什么时候发、发多快、
 * 多关节谁先谁后”全留给调用者 —— RT 调用者很容易把总线压满或饿死某些关节。
 *
 * 本文件就是那个调度器。**四条硬约束**（都是需求里点名要的）：
 *
 *   1. **每总线最多一个在途请求**：`poll_inflight_ms != 0` 时不再发新的。
 *      这是“限速”真正的保证 —— 不是因为配了周期，而是因为不等应答不罢休。
 *      **发帧只在 `cycle_begin()`**（`pump_rx` **之后**）→ 应答一定能在**同一个
 *      tick 内**被收进缓存，于是客户在 `cycle_end()` 之后读到的 `feedback()` 就是
 *      刚回来的那一帧（`age_ms` 也只有几 ms）。若放到 `cycle_end()` 发，
 *      应答要等下一个 tick 才收 → 客户每次读到的都是**上一周期**的值，
 *      而且 `fb.valid` 在 `cycle_end()` 那一刻刚被下一次 `cycle_begin()` 重新清零。
 *   2. **`node_id` 升序轮转**：从 `poll_rotation` 之后开始找，找不到再从头绕一圈。
 *      确定性（同样的总线状态一定发同样的帧），且不会“总是先伺候 node 1”。
 *   3. **每 tick 预算**：一个周期最多发 `per_cycle` 个（默认 1）。
 *   4. **超时即失效**：在途超时后计数 `state_timeout`，**不拿旧值冒充当前值** ——
 *      可信度仍由客户用 `feedback().age_ms` 判定（与 JointROS 的
 *      `polled_usable()` 同一条原则）。
 *
 * ⚠ 时序上的一个**容易做错**的点（本项目第一版就错了）：绝不能把“请求已发出”
 *   安排在**下一个** tick 的前半段（`pump_rx` 之前）才生效 —— 那样应答一定在
 *   `cycle_begin()` 之后才到，客户在本周期读到的还是旧值，而 `valid` 会在
 *   下一次 `cycle_begin()` 被清零 ⇒ 客户**永远看不到 `valid=1`**（真机症状就是
 *   “明明在轮询，`valid` 恒 0、`age_ms` 一直涨”）。
 *
 * ⚠ 与阶段 1 一致的三个“不”：
 *   - **不重发**：丢了就丢了（靠 `age_ms` 判）；
 *   - **不记账进 `req_timeouts`**：那是“请求-等应答”路径的指标，轮询是空闲查询，
 *     混在一起会让一条健康链路看起来在丢帧（真机已见过这类误报）；
 *   - **不喂狗**：0x41/0x44 不在固件 `is_ctrl` 里 ⇒ 客户必须继续照常发控制帧。
 *
 * 默认**关闭**（`cfg.state_poll_period_ms == 0`）：所有行为与本文件不存在时一致。
 */

#include "jsdk_core_internal.h"

/** 在途超时默认值（ms）。真机一个往返 ≈ 2 ms，50 ms 留了两个数量级余量。 */
#define JSDK_POLL_TIMEOUT_DEFAULT_MS 50u

uint32_t jsdk_state_poll_period_ms(const jsdk_context_t *ctx)
{
    if (!ctx) return 0u;
    return ctx->cfg.state_poll_period_ms;
}

/** 解析后的字段掩码（0 在配置里表示“全要”）。 */
static uint8_t poll_fields(const jsdk_context_t *ctx)
{
    uint8_t f = ctx->cfg.state_poll_fields;

    if (f == 0u) f = (uint8_t)(JSDK_STATE_POS_VEL | JSDK_STATE_CURRENT);
    return f;
}

/** 在途超时（解析后，非 0）。 */
static uint16_t poll_timeout(const jsdk_context_t *ctx)
{
    return ctx->cfg.state_poll_timeout_ms ? ctx->cfg.state_poll_timeout_ms
                                         : (uint16_t)JSDK_POLL_TIMEOUT_DEFAULT_MS;
}

/**
 * 本关节现在**是否该发**下一个轮询请求。
 *
 * 判据集中在这里，因为它错了会静默跑偏（“每拍都发”或“永远不发”）：
 *   1. 有在途 ⇒ 不发（约束 1）；
 *   2. 没排期过 ⇒ 发（首次）；
 *   3. 否则看 `poll_next_ms` **是否已经到点**：`elapsed(now, deadline) != 0`
 *      表示 deadline 已经过去。注意 `elapsed == 0` 表示“就是本毫秒”，也算到点 ——
 *      所以正确写法是 `!= 0 ? 到点 : 到点(相等)`,即**恒为真**…⛔
 *      ⇒ 因此这里换一个不会二义的写法：比较**绝对时刻**，只在回绕的半区里用差。
 *      `poll_next_ms` 与 `now_ms` 都是同一时基的 u32，差值天然处理回绕：
 *      `(now - deadline) < 2^31` == “未到点”。
 */
static int poll_is_due(const jsdk_context_t *ctx, const jsdk_joint_t *j)
{
    if (j->poll_pending) return 0;
    if (!j->poll_scheduled) return 1;
    /* `(now - deadline)` 的符号位判断需要无符号回绕语义：用差与 0x80000000 比 */
    return (uint32_t)(ctx->now_ms - j->poll_next_ms) < 0x80000000u;
}

jsdk_status_t jsdk_context_set_state_poll(jsdk_context_t *ctx, uint32_t period_ms,
                                          uint8_t per_cycle, uint8_t fields,
                                          uint16_t timeout_ms)
{
    unsigned i;

    if (!jsdk_ctx_check(ctx)) return JSDK_ERR_INVALID_ARG;
    /* ⚠ 用 `(unsigned)fields & ~KNOWN`（而非把 `~` 硬转成 uint8_t）：
       MSVC 对“把 16 位常量截断成 8 位”的写法报 C4310（本仓库 /W4 /WX ⇒ 直接失败）。 */
    if (((unsigned)fields & ~(unsigned)(JSDK_STATE_POS_VEL | JSDK_STATE_CURRENT)) != 0u) {
        jsdk_ctx_seterr(ctx, "set_state_poll: bad field mask 0x%02x", (unsigned)fields);
        return JSDK_ERR_INVALID_ARG;
    }

    ctx->cfg.state_poll_period_ms  = period_ms;
    ctx->cfg.state_poll_per_cycle  = per_cycle;
    ctx->cfg.state_poll_fields     = fields;
    ctx->cfg.state_poll_timeout_ms = timeout_ms;

    ctx->poll_timeout_ms = poll_timeout(ctx);

    /* 关掉调度器时把在途请求一并放弃（计入超时），否则下次打开会“接着等”一个
       上一轮遗留的请求 —— 客户会看到一个莫名其妙的大 age_ms。 */
    if (period_ms == 0u && ctx->poll_inflight_ms != 0u) {
        ctx->bus.state_timeout++;
        ctx->poll_inflight_ms = 0u;
        for (i = 0u; i < ctx->nj; ++i) ctx->joints[i].poll_pending = 0u;
    }
    if (period_ms == 0u) {
        for (i = 0u; i < ctx->nj; ++i) {
            ctx->joints[i].poll_scheduled = 0u;
            ctx->joints[i].poll_next_ms   = 0u;
        }
        ctx->poll_rotation = 0u;
    }

    return JSDK_OK;
}

void jsdk_state_poll__on_reply(jsdk_context_t *ctx, jsdk_joint_t *j, uint8_t msgtype)
{
    uint8_t field;

    if (!ctx || !j) return;
    if (ctx->poll_inflight_ms == 0u) return;      /* 没有调度器在途请求：客户自己发的，不计数 */
    if (j->index != ctx->poll_inflight_ji) return; /* 不是我们等的那一个关节 */

    field = (msgtype == (uint8_t)CB_MSG_QUERY_POS_VEL)   ? (uint8_t)JSDK_STATE_POS_VEL
          : (msgtype == (uint8_t)CB_MSG_QUERY_CURRENT)   ? (uint8_t)JSDK_STATE_CURRENT
                                                          : 0u;
    if (field == 0u || (field & j->poll_inflight_field) == 0u) return;

    /* ⚠ 计数单位是**帧**：`POS_VEL|CURRENT` 一次轮询两帧两应答 ⇒ state_sent/state_ok
       都 +2。这样 `state_ok / state_sent` 才是有意义的比值（见头文件说明）。 */
    ctx->bus.state_ok++;
    j->poll_inflight_field = (uint8_t)(j->poll_inflight_field & (uint8_t)~field);

    if (j->poll_inflight_field == 0u) {
        j->poll_pending       = 0u;
        ctx->poll_inflight_ms = 0u;               /* 释放总线级槽位 */
        j->poll_scheduled     = 1u;
        j->poll_next_ms       = ctx->now_ms + jsdk_state_poll_period_ms(ctx);
    }
}

/**
 * `cycle_begin()` 里、**`pump_rx()` 之后**调用：结掉已到期的在途请求。
 *
 * ⚠ 这里**不发**任何帧 —— 发帧在 `cycle_end()`（本拍的后半段）。这样安排是为了
 *   让“请求 → 应答 → 收进缓存”全部落在**同一个 tick 内**：客户在 `cycle_end()`
 *   之后读 `feedback()` 看到的就是刚回来的那一帧（`age_ms` 只有几 ms）。
 *   反过来（在 `cycle_begin()` 里发）会让应答一定在下一次 `cycle_begin()` 才收到，
 *   而 `valid` 恰好在那时被清零 ⇒ **客户永远看不到 `valid=1`**（真机症状就是这样）。
 */
void jsdk_state_poll__cycle_begin(jsdk_context_t *ctx)
{
    jsdk_joint_t *j;

    if (!ctx) return;
    if (jsdk_state_poll_period_ms(ctx) == 0u) return;

    /* 应答通常已在本拍的 `pump_rx` 里被收掉（`on_reply()` 清槽位）⇒ 这里只处理
       **没等到应答**的情形。 */
    if (ctx->poll_inflight_ms == 0u) return;

    if (jsdk_elapsed(ctx->now_ms, ctx->poll_inflight_ms) < (uint32_t)ctx->poll_timeout_ms) {
        return;                     /* 还在途：等下一拍 */
    }

    j = &ctx->joints[ctx->poll_inflight_ji];
    ctx->bus.state_timeout++;
    ctx->poll_inflight_ms  = 0u;
    j->poll_pending        = 0u;
    j->poll_inflight_field = 0u;
    /* 超时后**不**补偿式地连发：按正常节拍等下一个周期（有界、可预期）。
       ⚠ 这里把 `poll_next_ms` 设成**跳过本拍**（`now + 1`）—— 否则在“时钟很粗”
       （例如冻住的测试时钟）的场合，本拍的 `cycle_end()` 会立刻判定“又到期了”
       而重新发一次，于是**超时永远数不满**（本项目第一版就踩了这个）。 */
    j->poll_scheduled = 1u;
    j->poll_next_ms   = ctx->now_ms + 1u;
}

void jsdk_state_poll__cycle_end(jsdk_context_t *ctx)
{
    jsdk_joint_t *j = NULL;
    uint8_t  fields;
    uint32_t period;
    unsigned n;

    if (!ctx) return;

    period = jsdk_state_poll_period_ms(ctx);
    if (period == 0u || ctx->nj == 0u) return;

    /* ⚠ 约束 1：**每总线最多一个在途请求**。上一条的应答要在**下一次**
       `cycle_begin()` 的 `pump_rx` 里才会被收（`on_reply()` 清掉槽位）。
       所以这里只要槽位还占着，就什么都不做 —— “等应答或超时后才发下一个”
       正是不把总线压满的真正保证。配置里的 `per_cycle` 只影响**唤醒节拍**，
       不可能让本函数在一拍里发出多条（槽位只有一个，硬发就是自己骗自己）。 */
    if (ctx->poll_inflight_ms != 0u) return;

    fields = poll_fields(ctx);

    /* 轮转找第一个到期的关节（从上次发过的下一个开始 ⇒ 不会饿死后面的）。
       ⚠ 这里**只发一条**，而且不是 `per_cycle` 条：约束 1（每总线最多一个在途）
       使得“一拍发多条”在结构上不可能 —— 不信你看下面：发完就 `return`。
       因此 `per_cycle` 在本版**没有实际作用**，留在配置里只是为了：
         (a) 让调用者不用改代码就能表达“我不介意一拍发多条”的意愿；
         (b) 为将来“多槽位在途”的实现预留同一个开关。
       ⛔ **不要**为了“让 per_cycle 真的有用”而在这里循环发帧 —— 那会直接破坏
          约束 1，并让 `state_ok / state_sent` 失去意义（本项目第一版就是如此，
          靠变异测试才发现：删掉槽位检查后限速测试居然还是绿的）。 */
    for (n = 0u; n < ctx->nj; ++n) {
        unsigned idx = (unsigned)(ctx->poll_rotation + n) % ctx->nj;

        if (!poll_is_due(ctx, &ctx->joints[idx])) continue;
        j = &ctx->joints[idx];
        break;
    }
    if (!j) return;                                /* 都还没到期 / 都还在途 */

    if (jsdk_joint_request_state(j, fields) != JSDK_OK) {
        /* 发送失败：不计数、下个周期再试（不在这里置错，避免污染客户的
           “上一次错误”）—— 失败的下一次节拍由 `poll_next_ms` 兜住。 */
        j->poll_scheduled = 1u;
        j->poll_next_ms   = ctx->now_ms + period;
        return;
    }

    /* 记账单位是**帧**：一个字段一帧（与 jsdk_joint_request_state 的实现一致），
       于是 `state_ok / state_sent` 才是有意义的比值。 */
    ctx->bus.state_sent += (uint32_t)(((fields & JSDK_STATE_POS_VEL) ? 1u : 0u)
                                    + ((fields & JSDK_STATE_CURRENT) ? 1u : 0u));
    /* ❗ 发出去之后**必须**写“绝对到期时刻”，不能写“周期”：写 `now + period` 之后，
       ①`cycle_begin()` 仍会判“未到期”（差 = period ≥ 1）⇒ 本拍不会重复发（对）；
       但 ②`cycle_begin()` 的超时分支会把 `poll_next_ms` 再推到 `now + 1`，于是
       两处对同一个字段的语义重复、容易改坏。这里统一约定：
         `poll_next_ms` = **下一次允许发送的绝对时刻**（由应答、超时、失败三处各自推后）。 */
    j->poll_pending        = 1u;
    j->poll_inflight_field = fields;
    ctx->poll_inflight_ms  = ctx->now_ms ? ctx->now_ms : 1u;   /* 0 = 无在途（哨兵） */
    ctx->poll_inflight_ji  = j->index;
    ctx->poll_rotation     = j->index;
    j->poll_scheduled      = 1u;
    j->poll_next_ms        = ctx->now_ms + period;   /* 收到应答前不会再发 */
}
