/**
 * @file    jsdk_group.c
 * @brief   WP6：分组与广播同步（一条帧驱动多个关节）
 *
 * @par 广播的寻址与槽位（两条都来自固件，踩错就“静默不动”）
 *
 * **FD（0x80 MIT 广播）**：`Dest` 是位图（bit n = node_id n），**槽位号就是 node_id**，
 * 即设备 n 读 `[n*8, n*8+8)`。因此：
 *   - 帧长必须是 `(max_node_id + 1) * 8` —— 驱动 1..4 号就是 40 字节，
 *     **前 8 字节属于不存在的设备 0**，不能省；
 *   - 固件对 `len < (slot+1)*8` 的帧**整帧丢弃**，所以末尾空槽位必须用安全值
 *     （kp = kd = tau = 0）填充，否则后段设备静默不动。
 *
 * **Classic（0x81 MIT 广播）**：**恒用槽位 0**，被位图命中的所有设备执行
 * 同一条命令（帧长只需 8）。想让各设备执行不同指令**只能**用 CAN FD。
 *
 * @par 广播不回复
 * 所以 `jsdk_group_set_mit()` 不产生任何反馈。反馈仍来自心跳与后续单播查询。
 * 但广播帧**确实**刷新固件的协议级超时计时器（0x80 属于 `is_ctrl`），
 * 因此组内关节的 `last_ctrl_tx_ms` 会跟着更新。
 *
 * @par 为什么 enable/disable 是单播
 * 协议**没有**广播版的 `START_MOTOR`/`STOP_MOTOR`（MsgType 只有 0x80~0x83 是广播，
 * 且全是实时控制）。所以 `jsdk_group_enable/disable()` 是逐个下发请求的便利封装 ——
 * 真正的"同步"只存在于实时控制帧。这些请求随后由调用者的循环推进（非阻塞）。
 */

#include "jsdk_core_internal.h"

#include <string.h>

/* ==========================================================================
 * 内部工具
 * ======================================================================== */

/** 组内允许的最大成员数（位图只有 node_id 1..7 这 7 个可用位）。 */
#define JSDK_GROUP_MAX_MEMBERS 7u

/** 收集并校验一组关节。 */
static jsdk_status_t collect(jsdk_context_t *ctx, const jsdk_group_target_t *targets,
                             unsigned n, jsdk_joint_t **out, uint8_t *mask_out)
{
    unsigned i, k;
    uint8_t  ids[JSDK_GROUP_MAX_MEMBERS];
    int      mask;

    if (!jsdk_ctx_check(ctx) || !targets) return JSDK_ERR_INVALID_ARG;
    if (n == 0u) return JSDK_ERR_INVALID_ARG;
    if (n > JSDK_GROUP_MAX_MEMBERS) {
        /* 位图只有 7 个可用位；再多就必须分多条帧 */
        jsdk_ctx_seterr(ctx, "group size %u exceeds the bitmap limit (%u devices, "
                             "node_id 1..7)", n, (unsigned)JSDK_GROUP_MAX_MEMBERS);
        return JSDK_ERR_UNSUPPORTED;
    }

    for (i = 0u; i < n; ++i) {
        uint8_t id = targets[i].node_id;

        if (id < CB_NODE_ID_MIN) {
            jsdk_ctx_seterr(ctx, "group member %u has invalid node_id 0", i);
            return JSDK_ERR_INVALID_ARG;
        }
        if (id >= CB_MAX_BROADCAST_DEVICES) {
            /* node_id ≥ 8 无法位掩码寻址（固件直接判 is_for_me == 0） */
            jsdk_ctx_seterr(ctx, "node_id %u cannot be addressed by the broadcast "
                                 "bitmap (only 1..7); use unicast for this joint",
                            (unsigned)id);
            return JSDK_ERR_UNSUPPORTED;
        }
        for (k = 0u; k < i; ++k) {
            if (ids[k] == id) {
                jsdk_ctx_seterr(ctx, "duplicate node_id %u in one group", (unsigned)id);
                return JSDK_ERR_INVALID_ARG;
            }
        }
        out[i] = jsdk_ctx_find_joint(ctx, id);
        if (!out[i]) {
            jsdk_ctx_seterr(ctx, "node %u is not one of this context's joints", (unsigned)id);
            return JSDK_ERR_NOT_FOUND;
        }
        /* 没使能过的关节不能进组：否则等于替客户驱动一台 IDLE 设备 */
        if (!out[i]->tx_active) {
            jsdk_ctx_seterr(ctx, "joint %u (node %u) is not enabled; call "
                                 "request_enable() first", (unsigned)out[i]->index,
                            (unsigned)id);
            return JSDK_ERR_BAD_STATE;
        }
        if (!out[i]->calibrated) {
            jsdk_ctx_seterr(ctx, "joint %u (node %u) has no valid calibration",
                            (unsigned)out[i]->index, (unsigned)id);
            return JSDK_ERR_BAD_STATE;
        }
        /* 这个函数发的是 **MIT** 帧；如果关节正被另一种模式驱动，
           一条 MIT 广播会把设备悄悄切到 MIT 输入模式 —— 必须拦住。 */
        if (out[i]->mode != JSDK_MODE_MIT) {
            jsdk_ctx_seterr(ctx, "joint %u (node %u) is in %s mode; "
                                 "group_set_mit() only drives MIT joints",
                            (unsigned)out[i]->index, (unsigned)id,
                            jsdk_mode_string(out[i]->mode));
            return JSDK_ERR_BAD_STATE;
        }
        ids[i] = id;
    }

    mask = cb_make_broadcast_mask(ids, n);
    if (mask < 0) return JSDK_ERR_UNSUPPORTED;   /* 理论上到不了这里 */
    *mask_out = (uint8_t)mask;
    return JSDK_OK;
}

/** 把目标写进关节的缓存，并记账（广播帧同样喂看门狗）。 */
static void accept_target(jsdk_joint_t *j, const jsdk_group_target_t *t, uint32_t now_ms)
{
    j->tgt.pos_rad   = t->pos_rad;
    j->tgt.vel_rad_s = t->vel_rad_s;
    j->tgt.kp        = t->kp;
    j->tgt.kd        = t->kd;
    j->tgt.tau_Nm    = t->tau_Nm;
    j->tgt.have_pos  = 1u;
    j->tgt.have_vel  = 1u;
    j->tgt.have_mit  = 1u;
    j->tgt.have_tau  = 1u;

    j->tx_frames++;
    j->fb.tx_frames = j->tx_frames;
    j->last_ctrl_tx_ms = now_ms;      /* 0x80 属于 is_ctrl → 刷新协议级超时 */

    /* ⚠ 必须记上“本周期已发”：否则 cycle_end() 还会给这些关节各补一条单播，
       于是“一条广播驱动 N 个关节”变成“N+1 条帧”，广播同步完全失去意义。 */
    j->sent_cycle = 1u;
}

/** 判断两个目标是否完全相同（Classic 广播只能发同一条命令）。 */
static int same_target(const jsdk_group_target_t *a, const jsdk_group_target_t *b)
{
    return (a->pos_rad == b->pos_rad
            && a->vel_rad_s == b->vel_rad_s
            && a->kp == b->kp
            && a->kd == b->kd
            && a->tau_Nm == b->tau_Nm) ? 1 : 0;
}

/** 降级路径：逐个关节发单播 MIT（复用 L3 的编码与越界策略）。 */
static jsdk_status_t unicast_fallback(jsdk_context_t *ctx,
                                      const jsdk_group_target_t *targets,
                                      jsdk_joint_t *const *joints, unsigned n)
{
    unsigned i;

    /* 先把"本周期已发过"这类调用顺序问题一次查清，免得改到一半才发现 */
    for (i = 0u; i < n; ++i) {
        if (joints[i]->sent_cycle) {
            jsdk_ctx_seterr(ctx, "node %u already sent a control frame in this cycle; "
                                 "each joint may be commanded only once per cycle",
                            (unsigned)targets[i].node_id);
            return JSDK_ERR_BAD_STATE;
        }
    }

    for (i = 0u; i < n; ++i) {
        jsdk_joint_t *j = joints[i];

        jsdk_joint_set_mit(j, targets[i].pos_rad, targets[i].vel_rad_s,
                           targets[i].kp, targets[i].kd, targets[i].tau_Nm);
        /* 走与单播完全相同的编码/越界路径：越界时发安全帧而不是静默丢帧 */
        if (jsdk_joint__send_now(j) == 0) {
            jsdk_ctx_seterr(ctx, "group degraded to unicast but node %u could not be sent",
                            (unsigned)targets[i].node_id);
            return JSDK_ERR_TRANSPORT;
        }
    }
    jsdk_ctx_seterr(ctx, "group command degraded to %u unicast frame(s) "
                         "(%s cannot carry per-node targets)",
                    n, ctx->cfg.is_fd ? "range check failed"
                                      : "Classic CAN broadcast");
    return JSDK_OK;
}

/* ==========================================================================
 * jsdk_group_set_mit
 * ======================================================================== */

jsdk_status_t jsdk_group_set_mit(jsdk_context_t *ctx,
                                 const jsdk_group_target_t *targets, unsigned n)
{
    jsdk_joint_t *joints[JSDK_GROUP_MAX_MEMBERS];
    uint8_t  mask = 0u;
    uint8_t  payload[CB_MIT_SLOT_BYTES * CB_MAX_BROADCAST_DEVICES];
    uint8_t  max_slot = 0u;
    size_t   len;
    unsigned i;
    jsdk_status_t st;

    st = collect(ctx, targets, n, joints, &mask);
    if (st != JSDK_OK) return st;

    /* --- Classic：只有槽位 0。全员同目标才发一条广播，否则降级单播 --- */
    if (!ctx->cfg.is_fd) {
        for (i = 1u; i < n; ++i) {
            if (!same_target(&targets[0], &targets[i])) {
                return unicast_fallback(ctx, targets, joints, n);
            }
        }
        memset(payload, 0, sizeof payload);
        {
            uint8_t clamped = 0u;
            cb_mit_pack_command(payload, &joints[0]->range,
                                (float)targets[0].pos_rad, (float)targets[0].vel_rad_s,
                                (float)targets[0].kp, (float)targets[0].kd,
                                (float)targets[0].tau_Nm, &clamped);
        }
        len = CB_MIT_SLOT_BYTES;
        if (jsdk_ctx_send(ctx, CB_PRI_HIGH_CTRL, CB_MSG_MIT_CONTROL_BCAST, mask,
                          payload, (uint8_t)len) != 0) {
            return JSDK_ERR_TRANSPORT;
        }
        ctx->tx_seq = cb_seq_next(ctx->tx_seq);
        for (i = 0u; i < n; ++i) accept_target(joints[i], &targets[i], ctx->now_ms);
        jsdk_ctx_seterr(ctx, "group MIT (Classic, slot 0): %u devices share one command",
                        n);
        return JSDK_OK;
    }

    memset(payload, 0, sizeof payload);

    /* 先确定最大槽位（帧长依赖它） */
    for (i = 0u; i < n; ++i) {
        if (targets[i].node_id > max_slot) max_slot = targets[i].node_id;
    }

    /* ⚠⚠ 未使用的槽位**不能留全零**：MIT 命令的 64 bit 全零 → 解出来是
       pos = -pos_max、vel = -vel_max、tau = -tau_max（0 是各字段的最小码），
       等于“满速反向 + 满力矩反向”。位图会让未被寻址的设备忽略该帧，但一旦
       掩码写错、或将来改用 Dest=0xFF 全局广播，那就是**飞车指令**。
       ⇒ 每个未使用的槽位都显式写“零目标 + 零增益”。 */
    for (i = 0u; i <= (unsigned)max_slot; ++i) {
        jsdk_joint_t *ref = NULL;
        cb_mit_range_t range;
        unsigned k;

        for (k = 0u; k < n; ++k) {
            if (targets[k].node_id == i) { ref = joints[k]; break; }
        }
        if (ref) continue;                       /* 这个槽位稍后填真目标 */

        /* 量程用第 0 个关节的（只为编码 0，任何量程都等价），
           但 kp/kd/tau 三段的位宽是固定的，所以结果与量程无关地安全。 */
        range = joints[0]->range;
        cb_mit_pack_command(&payload[(size_t)i * CB_MIT_SLOT_BYTES], &range,
                            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, NULL);
    }

    for (i = 0u; i < n; ++i) {
        jsdk_joint_t *j = joints[i];
        uint8_t slot = targets[i].node_id;
        uint8_t clamped = 0u;

        cb_mit_pack_command(&payload[(size_t)slot * CB_MIT_SLOT_BYTES], &j->range,
                            (float)targets[i].pos_rad, (float)targets[i].vel_rad_s,
                            (float)targets[i].kp, (float)targets[i].kd,
                            (float)targets[i].tau_Nm, &clamped);

        if (clamped != 0u) {
            /* 越界不能在组路径里静默钳位：改用单播，让 §6.10 的策略生效 */
            jsdk_ctx_seterr(ctx, "node %u target out of range in group command",
                            (unsigned)slot);
            return unicast_fallback(ctx, targets, joints, n);
        }
    }

    /* 帧长 = (最大槽位 + 1) × 8；槽位 0 属于不存在的设备 0，但**必须存在** */
    len = cb_mit_bcast_frame_len(max_slot);
    if (len == 0u) return JSDK_ERR_UNSUPPORTED;

    if (jsdk_ctx_send(ctx, CB_PRI_HIGH_CTRL, CB_MSG_MIT_CONTROL_BCAST, mask,
                      payload, (uint8_t)len) != 0) {
        return JSDK_ERR_TRANSPORT;
    }
    ctx->tx_seq = cb_seq_next(ctx->tx_seq);
    for (i = 0u; i < n; ++i) accept_target(joints[i], &targets[i], ctx->now_ms);

    jsdk_ctx_seterr(ctx, "group MIT (FD): mask=0x%02X len=%u B, %u joints in one frame",
                    (unsigned)mask, (unsigned)len, n);
    return JSDK_OK;
}

/* ==========================================================================
 * jsdk_group_enable / jsdk_group_disable
 * ======================================================================== */

/** 把一组 node_id 变成关节指针（enable/disable 用；不限制 1..7，因为是单播）。 */
static jsdk_status_t collect_joints(jsdk_context_t *ctx, const uint8_t *node_ids,
                                    unsigned n, jsdk_joint_t **out)
{
    unsigned i, k;

    if (!jsdk_ctx_check(ctx) || !node_ids) return JSDK_ERR_INVALID_ARG;
    if (n == 0u || n > JSDK_MAX_JOINTS) return JSDK_ERR_INVALID_ARG;

    for (i = 0u; i < n; ++i) {
        if (node_ids[i] < CB_NODE_ID_MIN) return JSDK_ERR_INVALID_ARG;
        for (k = 0u; k < i; ++k) {
            if (node_ids[k] == node_ids[i]) return JSDK_ERR_INVALID_ARG;
        }
        out[i] = jsdk_ctx_find_joint(ctx, node_ids[i]);
        if (!out[i]) {
            jsdk_ctx_seterr(ctx, "node %u is not one of this context's joints",
                            (unsigned)node_ids[i]);
            return JSDK_ERR_NOT_FOUND;
        }
    }
    return JSDK_OK;
}

jsdk_status_t jsdk_group_enable(jsdk_context_t *ctx, const uint8_t *node_ids,
                                unsigned n)
{
    jsdk_joint_t *joints[JSDK_MAX_JOINTS];
    unsigned i;
    jsdk_status_t st = collect_joints(ctx, node_ids, n, joints);

    if (st != JSDK_OK) return st;

    /* 协议没有广播版 START_MOTOR → 逐个下单播请求，由调用者的循环推进 */
    for (i = 0u; i < n; ++i) {
        jsdk_joint_request_enable(joints[i], joints[i]->mode);
    }
    jsdk_ctx_seterr(ctx, "group enable requested for %u joint(s) "
                         "(unicast: the protocol has no broadcast START_MOTOR)",
                    n);
    return JSDK_OK;
}

jsdk_status_t jsdk_group_disable(jsdk_context_t *ctx, const uint8_t *node_ids,
                                 unsigned n)
{
    jsdk_joint_t *joints[JSDK_MAX_JOINTS];
    unsigned i;
    jsdk_status_t st = collect_joints(ctx, node_ids, n, joints);

    if (st != JSDK_OK) return st;

    for (i = 0u; i < n; ++i) {
        jsdk_joint_request_disable(joints[i]);
    }
    jsdk_ctx_seterr(ctx, "group disable requested for %u joint(s) (unicast)", n);
    return JSDK_OK;
}
