/**
 * @file    jsdk_profile.c
 * @brief   设备 profile：把「SDK 语义」映射到具体固件的端点词汇
 *
 * @par 为什么需要这一层
 *  L3 需要的是**语义**（齿比、力矩常数、MIT 量程、状态、标定方式），而不是
 *  某一家的端点名。早期实现把 7 个 ODrive 路径写死在 `jsdk_config.c` 里，于是
 *  任何换了词汇的固件（例如 mini-cheetah 派生的扁平 `config.*` / `action.*`）
 *  连 `configure()` 都过不去。
 *
 *  本文件只做两件事：
 *    1. **候选路径表**（名字 → 语义键），AUTO 时按表顺序试；
 *    2. **能力位**（`JSDK_PF_*`），告诉 L3 "这个设备的状态从哪来 / 标定怎么走 /
 *       看门狗是不是吃 0x04"。
 *  端点 ID 仍然**只**来自运行时描述符（本文件不内置任何 ID）。
 *
 * @par 扩展新厂商（例如 moteus）
 *  1. 公共头 `jsdk_devfamily_t` 追加一个值；
 *  2. 下面 `g_profiles[]` 追加一行（候选路径 + 基础能力位）；
 *  3. 若状态/标定机制与现有两族都不同，在 `jsdk_ops.c` 按能力位加分支。
 *  其余代码不需要改 —— 它们只认语义键和能力位。
 *
 * @par 解析顺序
 *  `AUTO` 时按 `g_profiles[]` 的顺序试：**ODrive 在前**（保持既有行为完全不变），
 *  然后才是 MIT 扁平词汇。调用方可用 `jsdk_joint_set_device_family()` 显式指定。
 */

#include "jsdk_core_internal.h"

#include <string.h>

/** 每个语义键最多几个候选路径（新增厂商时可加，但别忘了同步数组长度）。 */
#define JSDK_PROFILE_MAX_CANDIDATES 3u

typedef struct {
    jsdk_epkey_t key;
    const char *cand[JSDK_PROFILE_MAX_CANDIDATES];
} jsdk_profile_path_t;

typedef struct {
    uint8_t  family;       /**< jsdk_devfamily_t */
    const char *name;      /**< 日志用短名 */
    const jsdk_profile_path_t *paths;
    unsigned n_paths;
    uint8_t  base_flags;   /**< 该家族的固有语义（JSDK_PF_*） */
} jsdk_profile_def_t;

/* ==========================================================================
 * 候选路径表
 * ======================================================================== */

/*
 * ODrive / CYBERBEAST 原生词汇（= v0.40 之前写死的那 7 条 + 可选端点）。
 * ⚠ 顺序不能变：AUTO 先试这一族，保证老设备行为逐字节不变。
 */
static const jsdk_profile_path_t k_odrive_paths[] = {
    { JSDK_EPKEY_GEAR,          { "axis0.motor.config.gear_ratio", NULL, NULL } },
    { JSDK_EPKEY_TCONST,        { "axis0.motor.config.torque_constant", NULL, NULL } },
    { JSDK_EPKEY_CURLIM,        { "axis0.motor.config.current_lim", NULL, NULL } },
    { JSDK_EPKEY_POS_MAX,       { "axis0.controller.config.mit_max_pos", NULL, NULL } },
    { JSDK_EPKEY_VEL_MAX,       { "axis0.controller.config.mit_max_vel", NULL, NULL } },
    { JSDK_EPKEY_TAU_MAX,       { "axis0.controller.config.mit_max_torque", NULL, NULL } },
    { JSDK_EPKEY_KP_MAX,        { "axis0.controller.config.mit_max_kp", NULL, NULL } },
    { JSDK_EPKEY_KD_MAX,        { "axis0.controller.config.mit_max_kd", NULL, NULL } },
    { JSDK_EPKEY_NODE_ID,       { "axis0.config.can.node_id", NULL, NULL } },
    { JSDK_EPKEY_HB_RATE,       { "axis0.config.can.heartbeat_rate_ms", NULL, NULL } },
    { JSDK_EPKEY_WATCHDOG,      { "can.config.break_timeout", NULL, NULL } },
    { JSDK_EPKEY_CURRENT_STATE, { "axis0.current_state", NULL, NULL } },
    { JSDK_EPKEY_REQUESTED_STATE, { "axis0.requested_state", NULL, NULL } },
};

/*
 * mini-cheetah 派生固件（N32H473 移植）：扁平 `config.*` + `action.*`。
 *
 * ⚠ 它**没有** `mit_max_torque` 端点：固件按 `I_MAX × KT × GR` 动态算
 *   （见 `can_cyberbeast.c` 的 `cb_mit_range()`），所以这里留空，由
 *   `calibrate_joint()` 用 `CURLIM × TCONST × GEAR` 派生。
 * ⚠ 状态与标定也不走端点枚举：状态看 MIT 响应 / 心跳 nibble，标定写
 *   `action.calibrate`。两者都用能力位表达。
 */
static const jsdk_profile_path_t k_mit_paths[] = {
    { JSDK_EPKEY_GEAR,          { "config.gr", NULL, NULL } },
    { JSDK_EPKEY_TCONST,        { "config.kt", NULL, NULL } },
    { JSDK_EPKEY_CURLIM,        { "config.i_max", NULL, NULL } },
    { JSDK_EPKEY_POS_MAX,       { "config.p_max", NULL, NULL } },
    { JSDK_EPKEY_VEL_MAX,       { "config.v_max", NULL, NULL } },
    { JSDK_EPKEY_KP_MAX,        { "config.kp_max", NULL, NULL } },
    { JSDK_EPKEY_KD_MAX,        { "config.kd_max", NULL, NULL } },
    { JSDK_EPKEY_NODE_ID,       { "config.can_id", NULL, NULL } },
    { JSDK_EPKEY_HB_RATE,       { "config.can_hb_ms", NULL, NULL } },
    { JSDK_EPKEY_WATCHDOG,      { "config.can_timeout", NULL, NULL } },
    { JSDK_EPKEY_ACTION_CALIBRATE, { "action.calibrate", NULL, NULL } },
    /*
     * 置零判据需要两个**原始计数**端点（见 `jsdk_joint_set_zero_here()`）：
     * `config.m_zero` 是存储的机械零点，`telemetry.count` 是当前编码器计数。
     * 该固件的 SET_ZERO 把 `M_ZERO = count` 存 Flash，但**运行期**位置由
     * `turns` 的连续性补偿保持连续 ⇒ 位置要到下次上电才归零，所以判据必须
     * 比对这两个端点，而不是看 `position`。
     */
    { JSDK_EPKEY_ZERO_OFFSET,   { "config.m_zero", NULL, NULL } },
    { JSDK_EPKEY_ENC_COUNT,     { "telemetry.count", NULL, NULL } },
};

#define JSDK_COUNT_OF(a) (unsigned)(sizeof(a) / sizeof((a)[0]))

/**
 * 注册表（AUTO 的尝试顺序 = 数组顺序）。
 *
 * 语义位说明：
 *  - MIT：`0x04 CURRENT` 在固件 `cb_torque_like()` 里也把 `controller.timeout = 0`
 *    ⇒ 它会喂超时（与 ODrive 的 F19 不同）。不给这个位，SDK 会在客户从 MIT 切到
 *    纯 CURRENT 后周期性补发 MIT 零力矩帧，把电流指令打回 0。
 *  - MIT：`SET_ZERO` 只在 `MENU_MODE` 生效；ODrive 则允许闭环里置零。
 */
static const jsdk_profile_def_t g_profiles[] = {
    { JSDK_DEVFAMILY_ODRIVE, "odrive", k_odrive_paths,
      JSDK_COUNT_OF(k_odrive_paths), 0u },
    { JSDK_DEVFAMILY_CYBERBEAST_MIT, "mit", k_mit_paths,
      JSDK_COUNT_OF(k_mit_paths),
      (uint8_t)(JSDK_PF_STATE_FROM_NIBBLE | JSDK_PF_ZERO_REQUIRES_IDLE
                | JSDK_PF_WD_FEEDS_CURRENT | JSDK_PF_ZERO_NEEDS_BOOT) },
};

#define JSDK_PROFILE_COUNT JSDK_COUNT_OF(g_profiles)

/* ==========================================================================
 * 解析
 * ======================================================================== */

static const jsdk_profile_def_t *profile_def(uint8_t family)
{
    unsigned i;
    for (i = 0u; i < JSDK_PROFILE_COUNT; ++i) {
        if (g_profiles[i].family == family) return &g_profiles[i];
    }
    return NULL;
}

static const jsdk_profile_path_t *profile_path(const jsdk_profile_def_t *d,
                                               jsdk_epkey_t key)
{
    unsigned i;
    if (!d) return NULL;
    for (i = 0u; i < d->n_paths; ++i) {
        if (d->paths[i].key == key) return &d->paths[i];
    }
    return NULL;
}

/**
 * 按候选顺序查一个语义键。只查描述符（无总线 I/O）。
 * @return JSDK_OK（命中）/ JSDK_ERR_NOT_FOUND
 */
static jsdk_status_t resolve_key(jsdk_context_t *ctx, const jsdk_profile_def_t *d,
                                 jsdk_epkey_t key, uint16_t *ep_id,
                                 jsdk_ep_type_t *type)
{
    const jsdk_profile_path_t *p = profile_path(d, key);
    unsigned i;

    if (!p) return JSDK_ERR_NOT_FOUND;

    for (i = 0u; i < JSDK_PROFILE_MAX_CANDIDATES && p->cand[i]; ++i) {
        if (jsdk_ep_store_lookup(&ctx->store, p->cand[i], ep_id, type, NULL)
                == JSDK_OK) {
            return JSDK_OK;
        }
    }
    return JSDK_ERR_NOT_FOUND;
}

/** 该键是否可缺省（缺了不影响 `configure()` 通过）。 */
static int key_is_optional(jsdk_epkey_t key)
{
    switch (key) {
    case JSDK_EPKEY_CURLIM:
    case JSDK_EPKEY_TAU_MAX:
    case JSDK_EPKEY_NODE_ID:
    case JSDK_EPKEY_HB_RATE:
    case JSDK_EPKEY_WATCHDOG:
    case JSDK_EPKEY_CURRENT_STATE:
    case JSDK_EPKEY_REQUESTED_STATE:
    case JSDK_EPKEY_ACTION_CALIBRATE:
        return 1;
    default:
        return 0;
    }
}

/** 调用方是否已用 `jsdk_joint_config_t` 显式给了该数值（0 = 从设备读）。 */
static int cfg_provides(const jsdk_joint_t *j, jsdk_epkey_t key)
{
    switch (key) {
    case JSDK_EPKEY_GEAR:    return j->cfg.gear_ratio != 0.0f;
    case JSDK_EPKEY_TCONST:  return j->cfg.torque_constant != 0.0f;
    case JSDK_EPKEY_POS_MAX: return j->cfg.mit_max_pos != 0.0f;
    case JSDK_EPKEY_VEL_MAX: return j->cfg.mit_max_vel != 0.0f;
    case JSDK_EPKEY_TAU_MAX: return j->cfg.mit_max_torque != 0.0f;
    case JSDK_EPKEY_KP_MAX:  return j->cfg.mit_max_kp != 0.0f;
    case JSDK_EPKEY_KD_MAX:  return j->cfg.mit_max_kd != 0.0f;
    default:                 return 0;
    }
}

/** 一次尝试的中间结果（成功后才写回关节，避免半新半旧）。 */
typedef struct {
    uint16_t ep[JSDK_EPKEY_COUNT];
    uint8_t  type[JSDK_EPKEY_COUNT];
} jsdk_ep_set_t;

/**
 * 试一个家族：先把所有候选路径查一遍，再判断必需项是否齐全。
 *
 * 必需项（除非调用方已在 cfg 里给了数值）：
 *   GEAR / TCONST / POS_MAX / VEL_MAX / KP_MAX / KD_MAX。
 * 力矩量程三选一：cfg.mit_max_torque / 显式 TAU_MAX 端点 /
 *   CURLIM 端点（可派生 = cur_lim × kt × gear）。
 */
static int try_profile(jsdk_context_t *ctx, const jsdk_joint_t *j,
                       const jsdk_profile_def_t *d, jsdk_ep_set_t *set)
{
    unsigned i;
    jsdk_epkey_t k;

    memset(set, 0, sizeof *set);

    for (i = 0u; i < d->n_paths; ++i) {
        jsdk_ep_type_t t = JSDK_EP_JSON;
        uint16_t       ep = 0u;

        k = d->paths[i].key;
        if (resolve_key(ctx, d, k, &ep, &t) == JSDK_OK) {
            set->ep[k]   = ep;
            set->type[k] = (uint8_t)t;
        }
    }

    /* --- 必需项 --- */
    for (k = JSDK_EPKEY_GEAR; k <= JSDK_EPKEY_KD_MAX; ++k) {
        if (key_is_optional(k)) continue;
        if (cfg_provides(j, k)) continue;
        if (set->ep[k] == 0u) return 0;
    }

    /* --- 力矩量程：显式 / 派生 / cfg --- */
    if (!cfg_provides(j, JSDK_EPKEY_TAU_MAX)
        && set->ep[JSDK_EPKEY_TAU_MAX] == 0u
        && set->ep[JSDK_EPKEY_CURLIM] == 0u) {
        return 0;
    }

    return 1;
}

jsdk_status_t jsdk_profile_resolve(jsdk_context_t *ctx, jsdk_joint_t *j)
{
    unsigned i;
    int tried_any = 0;
    uint8_t hint;

    if (!jsdk_ctx_check(ctx) || !j) return JSDK_ERR_INVALID_ARG;

    hint = j->family_hint;

    for (i = 0u; i < JSDK_PROFILE_COUNT; ++i) {
        const jsdk_profile_def_t *d = &g_profiles[i];
        jsdk_ep_set_t set;
        uint8_t flags;

        if (hint != JSDK_DEVFAMILY_AUTO && d->family != hint) continue;
        tried_any = 1;

        if (!try_profile(ctx, j, d, &set)) continue;

        /* --- 成功：一次性写回（先清旧的，避免强制换家族时残留） --- */
        j->ep_gear_ratio      = set.ep[JSDK_EPKEY_GEAR];
        j->ep_torque_constant = set.ep[JSDK_EPKEY_TCONST];
        j->ep_current_lim     = set.ep[JSDK_EPKEY_CURLIM];
        j->ep_mit_pos         = set.ep[JSDK_EPKEY_POS_MAX];
        j->ep_mit_vel         = set.ep[JSDK_EPKEY_VEL_MAX];
        j->ep_mit_tau         = set.ep[JSDK_EPKEY_TAU_MAX];
        j->ep_mit_kp          = set.ep[JSDK_EPKEY_KP_MAX];
        j->ep_mit_kd          = set.ep[JSDK_EPKEY_KD_MAX];
        j->ep_node_id         = set.ep[JSDK_EPKEY_NODE_ID];
        j->ep_break_timeout   = set.ep[JSDK_EPKEY_WATCHDOG];
        j->ep_current_state   = set.ep[JSDK_EPKEY_CURRENT_STATE];
        j->ep_requested_state = set.ep[JSDK_EPKEY_REQUESTED_STATE];
        j->ep_zero_offset     = set.ep[JSDK_EPKEY_ZERO_OFFSET];
        j->ep_enc_count       = set.ep[JSDK_EPKEY_ENC_COUNT];

        /* 看门狗端点可能是 u16（ODrive）或 int32（MIT）——按描述符宽度读写 */
        j->wd_type = set.ep[JSDK_EPKEY_WATCHDOG]
                         ? set.type[JSDK_EPKEY_WATCHDOG]
                         : (uint8_t)JSDK_EP_U16;

        flags = d->base_flags;
        if (set.ep[JSDK_EPKEY_TAU_MAX] != 0u) {
            flags |= JSDK_PF_TAU_MAX_EXPLICIT;
        } else if (set.ep[JSDK_EPKEY_CURLIM] != 0u) {
            flags |= JSDK_PF_TAU_MAX_DERIVED;
        }
        if (set.ep[JSDK_EPKEY_REQUESTED_STATE] != 0u) {
            flags |= JSDK_PF_HAS_HOME;   /* ODrive: HOMING(11) 走 requested_state */
        }
        if (set.ep[JSDK_EPKEY_ACTION_CALIBRATE] != 0u
            && set.type[JSDK_EPKEY_ACTION_CALIBRATE] == (uint8_t)JSDK_EP_FUNCTION) {
            flags |= JSDK_PF_ACTION_CALIBRATE;
        }

        j->profile_flags = flags;
        j->dev_family    = d->family;
        return JSDK_OK;
    }

    if (!tried_any) {
        jsdk_joint_seterr(j, "device family %u is not registered in this build; "
                             "use JSDK_DEVFAMILY_AUTO or a known family",
                          (unsigned)hint);
    } else if (hint != JSDK_DEVFAMILY_AUTO) {
        const jsdk_profile_def_t *d = profile_def(hint);
        const jsdk_profile_path_t *gear = profile_path(d, JSDK_EPKEY_GEAR);
        jsdk_joint_seterr(j, "forced device family '%s' does not match the "
                             "descriptor (missing required endpoint, e.g. '%s')",
                          jsdk_profile_name(hint),
                          (gear && gear->cand[0]) ? gear->cand[0] : "?");
    } else {
        jsdk_joint_seterr(j, "no device profile matched: descriptor has neither "
                             "ODrive naming (axis0.motor.config.gear_ratio) nor "
                             "MIT naming (config.gr); check the descriptor or "
                             "force a family with jsdk_joint_set_device_family()");
    }
    return JSDK_ERR_NOT_FOUND;
}

/* ==========================================================================
 * 查询
 * ======================================================================== */

int jsdk_profile_lookup(const jsdk_joint_t *j, jsdk_epkey_t key,
                        uint16_t *ep_id, jsdk_ep_type_t *type)
{
    const jsdk_profile_def_t *d;

    if (!jsdk_joint_check(j)) return JSDK_ERR_INVALID_ARG;
    d = profile_def(j->dev_family);
    if (!d) return JSDK_ERR_NOT_FOUND;
    return (int)resolve_key(j->ctx, d, key, ep_id, type);
}

const char *jsdk_profile_name(uint8_t family)
{
    const jsdk_profile_def_t *d = profile_def(family);
    return d ? d->name : "unknown";
}

const char *jsdk_profile_action_calibrate(const jsdk_joint_t *j)
{
    const jsdk_profile_def_t *d;
    const jsdk_profile_path_t *p;

    if (!jsdk_joint_check(j)) return NULL;
    if ((j->profile_flags & (uint8_t)JSDK_PF_ACTION_CALIBRATE) == 0u) return NULL;
    d = profile_def(j->dev_family);
    p = profile_path(d, JSDK_EPKEY_ACTION_CALIBRATE);
    return (p && p->cand[0]) ? p->cand[0] : NULL;
}

int jsdk_joint__feeds_watchdog(const jsdk_joint_t *j, uint8_t msgtype)
{
    if (jsdk_msgtype_feeds_watchdog(msgtype)) return 1;
    /* MIT 固件的 `cb_torque_like()` 对 0x04 也把 controller.timeout 清零
       （ODrive 不会 —— 那是 F19 安全缺口）。按 profile 区分，避免误补 MIT 帧。 */
    if (msgtype == (uint8_t)CB_MSG_CURRENT_CONTROL
        && (j->profile_flags & (uint8_t)JSDK_PF_WD_FEEDS_CURRENT) != 0u) {
        return 1;
    }
    return 0;
}

/* ==========================================================================
 * 公共 API
 * ======================================================================== */

jsdk_status_t jsdk_joint_set_device_family(jsdk_joint_t *j, int family)
{
    if (!jsdk_joint_check(j)) return JSDK_ERR_INVALID_ARG;
    if (j->calibrated) {
        jsdk_joint_seterr(j, "cannot change device family after configure()");
        return JSDK_ERR_BAD_STATE;
    }

    /* 未知值按 AUTO：旧二进制把未初始化的填充传进来时不至于直接失败。 */
    if (family != (int)JSDK_DEVFAMILY_AUTO
        && family != (int)JSDK_DEVFAMILY_ODRIVE
        && family != (int)JSDK_DEVFAMILY_CYBERBEAST_MIT) {
        family = (int)JSDK_DEVFAMILY_AUTO;
    }
    j->family_hint = (uint8_t)family;
    j->dev_family  = 0u;          /* 重新解析 */
    j->profile_flags = 0u;
    return JSDK_OK;
}

int jsdk_joint_get_device_family(const jsdk_joint_t *j)
{
    if (!jsdk_joint_check(j)) return (int)JSDK_DEVFAMILY_AUTO;
    return (int)j->dev_family;
}
