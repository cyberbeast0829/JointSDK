/**
 * @file    jsdk_core_internal.h
 * @brief   L3 关节层的内部结构（**不安装、不对外**）
 *
 * 分层：L1 HAL → L2 协议（cb_*）→ **L3 本文件** → L4 公共 C ABI（joint_sdk.h）。
 * 本层是纯逻辑：**零 malloc、零平台 #ifdef、零阻塞**（配置阶段 API 例外，见
 * `jsdk_joint_read_config_snapshot()` 等）。
 *
 * @par 为什么不用指针指向公共结构体
 *  `jsdk_context_t` / `jsdk_joint_t` 对外是不透明句柄，但**存储由调用者提供**
 *  （`jsdk_context_storage_t`）。因此本文件里的定义就是实际布局，其大小必须
 *  塞进 `JSDK_CONTEXT_MAX_SIZE`；`jsdk_context.c` 里有编译期断言守住这条。
 */

#ifndef JSDK_CORE_INTERNAL_H
#define JSDK_CORE_INTERNAL_H

#include "jsdk_internal.h"

#include "cb_frame.h"
#include "cb_ctrl.h"
#include "cb_desc_cache.h"
#include "cb_heartbeat.h"
#include "cb_jsondesc_fetch.h"
#include "cb_mit.h"
#include "cb_param.h"
#include "cb_query.h"

/* ==========================================================================
 * 编译期常量
 * ======================================================================== */

/** 上下文/关节魔数（ABI 守卫；`jsdk_context_init` 与每个公开入口校验）。 */
#define JSDK_CTX_MAGIC    0x4B43544Au  /**< "TCK" 变体 */
#define JSDK_JOINT_MAGIC  0x4B4E4A54u

/** `last_error` 缓冲长度（DESIGN §6.8：192 字节，可读给工程师）。 */
#define JSDK_ERRSTR_LEN   192u

/** 每个关节的参数槽位数（DESIGN §6.9）。槽位 0 保留给故障详情自动读取。 */
#define JSDK_SDO_SLOTS    8u

/** 关节容量硬上限（与公共头 `JSDK_MAX_JOINTS_STATIC` 一致）。 */
#define JSDK_MAX_JOINTS   JSDK_MAX_JOINTS_STATIC

/** 单周期接收帧数默认上限（防总线风暴阻塞控制循环）。 */
#define JSDK_RX_BURST_DEFAULT 32u

/** 控制帧「近超时」预警阈值下限（ms）。 */
#define JSDK_WD_RISK_MIN_MS  10u

/** 配置阶段等待单次响应/状态跳转的默认超时（ms）。 */
#define JSDK_CFG_TIMEOUT_MS  200u

/**
 * `jsdk_context_activate()` 的**整个过程**超时（ms）。
 *
 * ⚠ 不能复用 JSDK_CFG_TIMEOUT_MS(200)：那是"等单次响应"的尺度，而这里是
 *   "CLEAR_ERRORS → START_MOTOR → 进闭环 → 发安全首帧"整条使能链，
 *   真机上通常几百 ms，带制动/大惯量时可达数秒。用 200 ms 会在真机上
 *   必然失败（而仿真里因为时间推进得快，反而看不出来）。
 */
#define JSDK_ACTIVATE_TIMEOUT_MS 5000u

/**
 * 设备侧协议级超时被禁用时的取值（`can.config.break_timeout == 0`）。
 *
 * ⚠⚠ **最新固件语义（本项目 v0.25 修正）**：`auto_stop_if_timeout()` 首句就是
 *   `if (timeout_ms == 0) return;`，且 `Config_t::break_timeout` 的**默认值就是 0**
 *   ⇒ **0 = 超时检测被禁用**，不是“按 100 ms 处理”。
 *   旧固件（本项目早期真机联调时）确实把 0 当 100 ms，当时的适配是错的，
 *   它曾经把“没武装”显示成“100 ms 已武装”，非常容易误导（见 FIRMWARE_ISSUES F28）。
 *
 * 凡是从 `jsdk_watchdog_device_ms()` 取超时的地方，都必须把 `JSDK_WD_DISABLED_MS`
 * 当作“**无狗可喂 / 无门限可比较**”处理，而不是“一个很小的超时”。
 */
#define JSDK_WD_DISABLED_MS  0u

/** 未提供 period_ns 时，式微序列用这个周期估算（ms）。 */
#define JSDK_CFG_PERIOD_FALLBACK_MS  1u

/** 节点发现“被动阶段”的静置时长：2 个默认心跳周期（§6.7）。 */
#define JSDK_DISCOVER_PASSIVE_MS     200u

/**
 * 该 MsgType 的帧会不会刷新固件的**协议级超时计时器**（`last_cmd_time_`）。
 *
 * 与固件 `is_ctrl` 逐字对应：`msgtype <= 0x03 || (0x80 <= msgtype <= 0x83)`。
 *
 * ⚠ **不要**用 `cb_ctrl_is_control_msgtype()` 代替：那是"属于 ctrl 模块的实时
 *   控制帧"，**包含 0x04 CURRENT_CONTROL**，而固件的 `is_ctrl` 恰好**不含** 0x04。
 *   两者正好在唯一关键的那一个取值上不同 —— 用错就会让 SDK 以为电流指令能喂狗，
 *   于是永远不补 keepalive，把固件侧的 F19（安全阀不武装 / 误停）原样复制到
 *   SDK 内部。（本项目确实踩过：`test_joint` 的 `[6] watchdog` 用例暴露出来。）
 */
static inline int jsdk_msgtype_feeds_watchdog(uint8_t msgtype)
{
    return (msgtype <= (uint8_t)CB_MSG_TORQUE_CONTROL
            || (msgtype >= (uint8_t)CB_MSG_MIT_CONTROL_BCAST
                && msgtype <= (uint8_t)CB_MSG_TORQUE_CONTROL_BCAST)) ? 1 : 0;
}

/* ==========================================================================
 * 目标缓存
 * ------------------------------------------------------------------------
 * setter 是 `void`（RT 路径无分支），越界判定与编码统一放到 cycle_end()。
 * 每个量独立记 `have_*`，这样切换模式不会把别的模式的量一起清掉。
 * ======================================================================== */

typedef struct {
    double pos_rad;        /**< 输出端 rad（MIT / CSP） */
    double vel_rad_s;      /**< 输出端 rad/s（MIT / CSV） */
    double tau_Nm;         /**< 输出端 N·m（MIT / CST） */
    double kp, kd;         /**< MIT 线上值（原样透传，见 §6.2） */
    double cur_A;          /**< 电机端 A（CURRENT） */
    double vel_lim_rad_s;  /**< POS/VEL 模式的限速（输出端） */
    double cur_lim_A;      /**< POS/VEL 模式的限流（电机端） */

    uint8_t have_pos;      /**< set_target_position_rad / set_mit 调用过 */
    uint8_t have_vel;
    uint8_t have_tau;
    uint8_t have_mit;      /**< set_mit / set_mit_stiffness 调用过 */
    uint8_t have_cur;
    uint8_t have_raw_pos;  /**< set_target_position(raw) 调用过：原样透传 */
    uint8_t have_raw_vel;
    uint8_t have_raw_tau;

    int32_t raw_pos, raw_vel;  /**< 协议原始量（/1000 定点，见头文件 §11 说明） */
    int16_t raw_tau;
} jsdk_target_t;

/* ==========================================================================
 * 参数槽位（SDO 风格）
 * ======================================================================== */

typedef struct {
    uint16_t ep_id;
    uint16_t size;       /**< 缓冲字节数（描述符给出的类型长度） */
    uint8_t  in_use;
    uint8_t  state;      /**< jsdk_sdo_state_t */
    uint8_t  data[8];    /**< 端点值最大 8 字节（u64） */
} jsdk_sdo_slot_t;

/* ==========================================================================
 * 关节
 * ======================================================================== */

struct jsdk_joint {
    uint32_t magic;
    struct jsdk_context *ctx;
    uint8_t  index;        /**< 在 ctx->joints[] 中的下标 */
    uint8_t  reserved[3];

    jsdk_joint_config_t cfg;   /**< 用户给的配置副本（0 = 自动发现） */

    /* ---- 标定量程（configure() 从描述符读回，全部为设备实际值） ---- */
    cb_mit_range_t    range;
    float             gear_ratio;
    float             torque_constant;
    float             max_current_a;  /**< cb_mit_response_max_current() */
    jsdk_unit_scale_t scale;
    uint8_t           calibrated;     /**< 1 = 量程/齿比/力矩常数均有效 */
    uint8_t           shared_desc;    /**< 1 = 端点表复用自同总线其它节点 */

    /* ---- 端点 ID（由运行时描述符解析，无内置表） ---- */
    uint16_t ep_gear_ratio, ep_torque_constant;
    uint16_t ep_mit_pos, ep_mit_vel, ep_mit_tau, ep_mit_kp, ep_mit_kd;
    uint16_t ep_requested_state, ep_current_state, ep_node_id, ep_break_timeout;

    /* ---- 设备侧配置读回 ---- */
    uint32_t break_timeout_ms;    /**< can.config.break_timeout；**0 = 设备侧超时检测已禁用** */
    uint32_t node_id_readback;    /**< axis0.config.can.node_id */
    uint32_t heartbeat_rate_ms;   /**< axis0.config.can.heartbeat_rate_ms（0 = 设备不发心跳） */
    uint8_t  current_state_raw;   /**< axis0.current_state（固件 AxisState 0..16） */
    uint8_t  state_known;         /**< 1 = 至少读到过一次 current_state */

    /* ---- 目标 ---- */
    jsdk_target_t tgt;

    /* ---- 模式与状态机 ---- */
    jsdk_mode_t mode;
    jsdk_mode_t enable_mode;      /**< request_enable 携带的模式 */
    uint8_t  enabled;             /**< 归一化结论：闭环执行中（**且使能序列已走完**） */
    uint8_t  enable_pending;
    uint8_t  disable_pending;
    uint8_t  faultreset_pending;
    uint8_t  first_frame_done;    /**< 使能后的安全首帧已发（§6.4） */
    uint8_t  seq_step;            /**< 使能/失能序列步号（非阻塞步进） */
    uint8_t  ctrl_blocked;        /**< 1 = 标定/回零进行中，禁止发控制帧（§6.4） */
    /**
     * 1 = 本关节已在**发控制帧**（使能序列走完后置位，失能/复位后清除）。
     *
     * ⚠ 不能用「使能序列走完」以外的条件代替：没使能的关节如果也发 MIT，
     *   就等于替客户把设备的**协议级超时保护**武装了（`is_ctrl` 会刷新
     *   `last_cmd_time_`）——设备本来是 IDLE 安全的，现在“客户不再调用循环”
     *   反而会让它 `CAN_BUS_FAILED`。
     */
    uint8_t  tx_active;

    /**
     * 1 = 本周期已经给这个关节发过控制帧（单播或广播都算）。
     *
     * 由 `cycle_begin()` 清零；发送成功后置位。
     * 用途：`jsdk_group_set_mit()` 已经用一条广播帧驱动了 N 个关节，
     * `cycle_end()` 就不应再给它们各发一条单播 —— 否则“广播同步”
     * 反而比逐个单播还多一条帧，失去意义。
     */
    uint8_t  sent_cycle;
    uint8_t  hold_requested;      /**< 本周期调用过 hold_position*（用于诊断） */
    uint32_t seq_start_ms;        /**< 当前序列步的起始时刻（超时判定） */

    /* ---- 反馈（直接就是对外结构，避免二次转换） ---- */
    jsdk_joint_feedback_t fb;
    uint32_t last_fb_ms;      /**< 最近一次有效反馈时刻 */
    uint32_t hb_seen_ms;      /**< 最近一次心跳时刻 */
    uint8_t  hb_seen;

    /* ---- 记账 ---- */
    uint32_t last_ctrl_tx_ms; /**< 最近一次发出**控制类**帧（喂狗有效） */
    uint32_t keepalive_sent;  /**< 本关节被自动补喂狗的次数 */
    uint32_t tx_frames;       /**< 本关节发出的控制帧总数 */
    uint32_t tx_rejected;     /**< 因越界被拒绝（改发安全帧/钳位）的指令数 */
    uint16_t status_flags;    /**< JSDK_JF_* 粘滞位 */
    uint8_t  fault_prev;      /**< 上一次 is_fault（故障回调边沿检测） */
    jsdk_fault_info_t fault;

    /* ---- 状态轮询（v0.37；见 jsdk_state_poll.c）---- */
    uint32_t poll_next_ms;    /**< 下一次允许发送的**绝对时刻**（仅当 `poll_scheduled` 时有意义） */
    uint8_t  poll_scheduled;  /**< 1 = 已排期（`poll_next_ms` 有效）。**不要**用 `poll_next_ms`
                                   的 0 当“未排期”哨兵 —— 时刻 0 是合法值，两者混用会让
                                   “首次该不该发”变成一个说不清的边界（本项目踩过） */
    uint8_t  poll_pending;    /**< 1 = 本关节有在途请求未结 */
    uint8_t  poll_inflight_field; /**< 在途的是哪个字段（JSDK_STATE_*，单个位） */
    uint8_t  poll_reserved;

    jsdk_sdo_slot_t sdo[JSDK_SDO_SLOTS];
};

/* ==========================================================================
 * 上下文
 * ======================================================================== */

struct jsdk_context {
    uint32_t magic;
    jsdk_context_config_t cfg;   /**< 配置副本（hal 已被拷贝，可安全丢弃原结构） */

    /* ---- 循环状态 ---- */
    uint32_t now_ms;             /**< 本周期 HAL 时钟 */
    uint8_t  tx_seq;             /**< 广播/命令用的滚动 Seq（模 4） */
    uint8_t  in_cycle;
    uint8_t  destroyed;

    jsdk_joint_t joints[JSDK_MAX_JOINTS];
    unsigned     nj;

    /* ---- 描述符（端点表放在 jsdk_desc_config_t.arena，不占本结构） ---- */
    jsdk_ep_store_t   store;
    cb_desc_fetch_t   fetch;
    int               fetch_active;   /**< cb_desc_fetch 已被 init（需避免二次 init） */
    uint32_t          fetch_deadline_ms;
    uint32_t          fetch_bytes_seen; /**< 非阻塞路径的上一次进展字节数（静默预算用） */
    uint8_t           desc_present;   /**< 1 = 端点表可用（configure 不再下载） */
    /** 对端帧格式的学习结果：0 = 未知，1 = 已改为 Classic，2 = 已改为 FD，
        3 = 与配置一致（无需调整）。见 `jsdk_context_framing_learned()`。 */
    uint8_t           framing_learned;
    /** 1 = 会话预热已成功过（同一个会话里再调 `jsdk_context_warmup()` 是空操作）。 */
    uint8_t           warmed;
    /* 自动预热是否已经尝试过（成功或失败都置 1）：失败后**不**在每个请求前
       反复重试 500 ms —— 显式调用 jsdk_context_warmup() 仍然可以重试。 */
    uint8_t           warmup_tried;
    /* 1 = 正在跑会话预热。预热的收发**不参与帧格式学习**：设备对 0x46 的回包会
       按**请求**的格式回（经典 8 B / FD 16 B），学到“一致”只是镜像，会掩盖
       “对端其实是 FD”的真相（`jsdk_ctx__learn_framing()` 只学一次）。 */
    uint8_t           in_warmup;
    /** 1 = 正在预热（`jsdk_ctx_wait_response()` 里的懒预热靠它防递归）。 */
    jsdk_desc_info_t  desc;           /**< 对外元信息；crc/fw 同时是缓存键 */

    /**
     * 回写 `jsdk_desc_config_t.arena_used` 的槽位。
     *
     * ⚠ 公共头把 `arena_used` 声明为**输出**（“实际用量，可按实测缩容”），
     *   但 init 时配置是被**复制**的 —— 只写自己的副本的话调用方永远看不到。
     *   因此这里存一个槽位指针直接写回。
     *
     * @warning 因此**调用方的 `jsdk_context_config_t` 必须与 arena 同寿命**
     *          （只要 arena 还活着，配置结构体通常也在）。
     */
    size_t           *arena_used_slot;
    jsdk_desc_raw_sink_fn raw_sink;
    void                 *raw_sink_user;
    jsdk_desc_progress_fn progress;
    void                 *progress_user;

    /* ---- 记账 ---- */
    jsdk_bus_state_t bus;
    uint32_t         last_rx_ms;      /**< 最近一次收到与本主站相关帧的时刻 */

    /* ---- 状态轮询调度器（v0.37；配置在 cfg，运行态在这里）---- */
    uint32_t poll_inflight_ms;   /**< 在途请求的发出时刻；0 = 无在途 */
    uint16_t poll_timeout_ms;    /**< 在途超时（解析后的值，非 0） */
    uint8_t  poll_inflight_ji;   /**< 在途请求对应的关节下标（无在途时无意义） */
    uint8_t  poll_rotation;      /**< 轮转起点：上一次发到哪个关节（下一个从它之后找） */
    /**
     * 最近一次**自动重发**的时刻与类别（只为观测；`jsdk_context_get_bus_state()`
     * 把它们换算成 `last_retry_what` / `last_retry_age_ms` 报出去）。
     * 类别：0 从未 / 1 会话预热 / 2 幂等请求。
     */
    uint32_t         last_retry_ms;
    uint8_t          last_retry_what;
    /** 描述符下载中重发 `0x24` 请求的次数（每次下载开始时清零，见 `jsdk_desc.c`）。 */
    uint32_t         desc_retries;

    /**
     * 堆模式（`heap_optional.c`）下由 SDK 自己分配的 arena；零 malloc 模式下为 NULL。
     * 记在这里是为了让 `jsdk_context_free()` 知道该释放什么。
     */
    void                 *owned_arena;

    /* ---- 错误与回调 ---- */
    char                  last_error[JSDK_ERRSTR_LEN];
    jsdk_fault_callback_t fault_cb;
    void                 *fault_user;
};

/* ==========================================================================
 * 内部工具（跨文件共享）
 * ======================================================================== */

/** 上下文/关节魔数校验。 */
int jsdk_ctx_check(const jsdk_context_t *ctx);
int jsdk_joint_check(const jsdk_joint_t *j);

/** 写 `ctx->last_error`（printf 风格的最小子集：仅 %s / %u / %d / %%）。 */
void jsdk_ctx_seterr(jsdk_context_t *ctx, const char *fmt, ...);

/** `now_ms` 无符号差值（天然处理 32 位回绕）。 */
static inline uint32_t jsdk_elapsed(uint32_t now, uint32_t then)
{
    return (uint32_t)(now - then);
}

/**
 * 发送一帧并记账。@return HAL send 的返回值（0 = 成功）。
 * @note 失败时置 `JSDK_JF_TX_FAILED` 不在此处做（需要 joint 上下文），
 *       由调用方处理。
 */
int jsdk_ctx_send(jsdk_context_t *ctx, uint8_t pri, uint8_t msgtype,
                  uint8_t dest, const uint8_t *payload, uint8_t len);

/**
 * 记下“最近一次重发”的时刻与类别（纯观测；见 `jsdk_bus_state_t.last_retry_what`）。
 * 类别：1 = 会话预热；2 = 幂等请求。
 */
void jsdk_ctx_note_retry(jsdk_context_t *ctx, uint8_t what);

/** 不含自动预热的发送：控制帧 / 急停专用（延迟敏感，不能等）。 */
int jsdk_ctx_send_raw(jsdk_context_t *ctx, uint8_t pri, uint8_t msgtype,
                      uint8_t dest, const uint8_t *payload, uint8_t len);

/**
 * 幂等请求的**额外**尝试次数（总尝试 = 1 + 本值）。
 *
 * ⚠ 刻意取 1：目标是“单发命令不再随机失败”，不是“无限重试直到成功”。
 *   每多一次尝试，设备真不在时就要多等一个完整超时（现场 3 s 级），
 *   而“设备不在”和“这一帧丢了”在超时这一层本来就分不开。
 */
#define JSDK_REQ_RETRY_MAX 1u

/*
 * `jsdk_ctx_request()` 的行为开关。
 *
 * ⚠⚠ 别用“重发”与“记账”两个布尔参数去堆签名 —— 这两件事的**判据不同**：
 *   重发看“幂等吗”，记账看“这次没等到，值得让人知道吗”。
 *   典型反例（真机实测抓到的）：`scan` 会逐个问 1..16 号地址，
 *   **绝大多数地址本来就没人** ⇒ 15 次“等超时”是**正常结果**；
 *   把它们算进 `req_timeouts`，现场就会看到一条健康的扫描报 `超时=15`，
 *   与“链路坏了”完全分不清。
 */
/** 超时后重发同一帧（最多 `JSDK_REQ_RETRY_MAX` 次）。⚠ 只能用于**幂等**请求。 */
#define JSDK_REQ_RETRY 0x1u
/** 计入 `req_timeouts` / `last_retry_*`（“这次没等到值得让人知道”）。 */
#define JSDK_REQ_COUNT 0x2u

/**
 * 幂等的“请求 → 响应”：`allow_retry` 非 0 时超时后重发同一帧
 * （最多 `JSDK_REQ_RETRY_MAX` 次）。
 *
 * ⚠ **只能用于幂等请求**：读，以及“同一个值再写一遍”的写。
 *   定义与理由见 `jsdk_context.c` 的完整注释。
 */
/**
 * @param msgtype     请求的 MsgType。
 * @param rsp_msgtype **应答**的 MsgType：多数查询与请求同号（0x20/0x21/0x45/0x46），
 *                    但 `QUERY_STATUS(0x40)` 等的应答是 MIT 响应（**0x00**）——
 *                    写错了就是“永远等不到应答”（本函数无法替你猜）。
 * @param flags       `JSDK_REQ_RETRY` / `JSDK_REQ_COUNT` 的按位或；
 *                    **0 = 只发一次且不记账**。
 */
int jsdk_ctx_request(jsdk_context_t *ctx, uint8_t pri, uint8_t msgtype,
                     uint8_t dest, const uint8_t *payload, uint8_t len,
                     uint8_t rsp_msgtype, jsdk_can_frame_t *out,
                     uint32_t timeout_ms, unsigned flags);

/** 幂等请求 + 自动重发（面向用户的命令走这个）。 */
int jsdk_ctx_request_retry(jsdk_context_t *ctx, uint8_t pri, uint8_t msgtype,
                           uint8_t dest, const uint8_t *payload, uint8_t len,
                           jsdk_can_frame_t *out, uint32_t timeout_ms);


/**
 * 阻塞等待某个 `(msgtype, source)` 的响应，同时把所有收到的帧分派给
 * 反馈解复用器（否则会丢掉期间的心跳）。
 *
 * @param want_seen 可选输出：1 = 收到目标响应
 * @return JSDK_OK / JSDK_ERR_TIMEOUT / JSDK_ERR_TRANSPORT
 * @note **仅配置阶段可用**（会阻塞并调用 HAL 的 now_ms/recv）。
 */
int jsdk_ctx_wait_response(jsdk_context_t *ctx, uint8_t msgtype, uint8_t source,
                           jsdk_can_frame_t *out, uint32_t timeout_ms);

/**
 * 目标地址上是否**已经有人在应答**（定向探测，只发一帧 `QUERY_STATUS`）。
 *
 * 用途：`set-node-id` 的前置检查 —— 目标号被别的设备占用时，
 * 「验证新地址可应答」会被那台设备满足，于是静默造出两个同号设备。
 *
 * @return 1 = 有设备应答；0 = 无应答（含发送失败）
 * @note **仅配置阶段可用**（会阻塞）。
 */
/** 扫描用：单个地址问一句，**不重发**（“没人应答”就是正常结果）。 */
int jsdk_ctx_probe_node(jsdk_context_t *ctx, uint8_t node_id);

/** 改号前的安全检查：同一个探测但**允许重发**（假阴性会造出两个同号设备）。 */
int jsdk_ctx_probe_node_strict(jsdk_context_t *ctx, uint8_t node_id);

/**
 * 学一次对端的帧格式（Classic vs FD），只认**我们自己的关节**发来的帧。
 *
 * ⚠ 从 `jsdk_ctx_handle_frame()` 与 `jsdk_ctx_wait_response()` **两处**都要调：
 *   后者会把匹配的响应帧直接返回，**不再交给** handle_frame —— 只挂在
 *   handle_frame 上的话，“响应帧自己就能告诉我们对端格式”这条信息就丢了
 *   （而会话预热正是靠它工作的：静默总线上没有心跳可听）。
 *
 * @return 1 = 本次调用学到了/确认了（含“与配置一致”）；0 = 与本主站无关或已学过
 */
int jsdk_ctx__learn_framing(jsdk_context_t *ctx, const jsdk_can_frame_t *f);

/**
 * 接收并解复用**一帧**（不推进时钟、不记账 rx 计数以外的状态）。
 * @return 1 = 已处理；0 = 队列空；< 0 = 链路错误
 */
int jsdk_ctx_handle_frame(jsdk_context_t *ctx, const jsdk_can_frame_t *f);

/** 至少有一个关节已被使能（描述符下载与发现都要拒绝这种状态）。 */
int jsdk_ctx_any_enabled(const jsdk_context_t *ctx);

/** 按 (source, dest) 找关节；找不到返回 NULL。 */
jsdk_joint_t *jsdk_ctx_find_joint(jsdk_context_t *ctx, uint8_t node_id);

/** 更新 `JSDK_JF_*` 粘滞位（内部用 `|=`）。 */
void jsdk_joint_set_flags(jsdk_joint_t *j, uint16_t flags);

/** 把 arena 实际用量写回调用方的 `desc.arena_used`（未提供时无操作）。 */
void jsdk_ctx_publish_arena_used(jsdk_context_t *ctx);

/** 记录一条可读错误串（含关节号）。 */
void jsdk_joint_seterr(jsdk_joint_t *j, const char *fmt, ...);

/** 单个关节结构的字节数（用于验证 `JSDK_CONTEXT_MAX_SIZE` 的预算）。 */
size_t jsdk_context_joint_size(void);

/* ---- 单位换算（jsdk_units.c） ---- */

/** 输出端 rad → 电机 turns（心跳/QUERY 用）。 */
double jsdk_units_rad_to_turns(double rad, double gear_ratio);

/** 电机 turns → 输出端 rad。齿比非法（≤0）时返回 0。 */
double jsdk_units_turns_to_rad(double turns, double gear_ratio);

/** rad/s → RPM（POS/VEL 线上单位）。 */
double jsdk_units_rad_s_to_rpm(double rad_s);

/** RPM → rad/s。 */
double jsdk_units_rpm_to_rad_s(double rpm);

/** 输出端真实刚度 → 线上 kp（**恒等，不换算**；2026-09-29 真机实测定案，见 jsdk_units.c）。 */
double jsdk_units_stiffness_to_kp(double stiffness_nm_per_rad, double gear_ratio);

/** 线上 kp → 输出端真实刚度（**恒等，不换算**）。 */
double jsdk_units_kp_to_stiffness(double kp, double gear_ratio);

/**
 * 多圈位置展开（§6.1）。默认**不启用**：MIT 响应是 16-bit 定点，
 * 满量程 ±mit_max_pos，超出会被设备钳位；盲目累加会掩盖钳位。
 *
 * @param prev_rad  上次展开值
 * @param raw_rad   本次单圈读数
 * @param range_rad 量程（= mit_max_pos）
 * @param out_turns 输出：累计圈数（可 NULL）
 * @return 展开后的绝对位置（rad）
 *
 * @note 仅在调用方明确知道"越过 ±range 是真实运动而非钳位"时使用。
 */
double jsdk_units_pos_unwrap(double prev_rad, double raw_rad, double range_rad,
                             long *out_turns);

/* ---- 文本（jsdk_text.c） ---- */

/**
 * 把浮点写成定点文本（如 `-12.500`）。
 *
 * 为什么不直接用 `%f`：很多 MCU 工具链**默认不链接浮点 printf**，一旦用了
 * `%f` 要么增大 20 KB 代码，要么在运行期打印出 `%f` 字面量。错误串只在
 * 出错路径用，不值得为此付出代价。
 *
 * @return 写入的字符数（不含结尾 NUL）；缓冲不足时截断并仍返回 NUL 结尾。
 */
size_t jsdk_fmt_f(char *dst, size_t cap, double v, unsigned decimals);

/* ==========================================================================
 * 关节钩子（jsdk_joint.c）
 * ------------------------------------------------------------------------
 * 这些函数只由 jsdk_context.c（循环边界/解复用）与配置流程调用。
 * ======================================================================== */

/** 收到 MIT 响应（所有控制帧的应答都是这个格式）。 */
void jsdk_joint__on_mit_response(jsdk_joint_t *j, const uint8_t *data, uint8_t len);

/** 收到心跳 0x48。 */
void jsdk_joint__on_heartbeat(jsdk_joint_t *j, const uint8_t *data, uint8_t len);

/** QUERY_POS_VEL(0x41)：**电机端 turns**，需按 gear 换算成输出端 rad。 */
void jsdk_joint__on_pos_vel_turns(jsdk_joint_t *j, float pos_turns, float vel_turns_s);

void jsdk_joint__on_current_a(jsdk_joint_t *j, double iq_a);
void jsdk_joint__on_temps(jsdk_joint_t *j, double motor_c, double fet_c);
void jsdk_joint__on_bus_volts(jsdk_joint_t *j, double vbus_v, double ibus_a);

/** 收到 0xC1 FAULT_ALERT 广播。 */
void jsdk_joint__on_fault_alert(jsdk_joint_t *j);

/** cycle_end()：推进所有关节的状态机并发送本周期指令。 */
void jsdk_joint__cycle_end_all(jsdk_context_t *ctx);

/**
 * 立即编码并发送**一个**关节的本周期指令。
 * @return 1 = 已发；0 = 未发（未标定 / 本周期已发 / 发送失败）。
 * @note 用于广播同步的降级路径；正常路径由 `cycle_end()` 统一做。
 */
int jsdk_joint__send_now(jsdk_joint_t *j);

/** cycle_end()：看门狗/keepalive（jsdk_watchdog.c）。 */
void jsdk_watchdog__cycle_end(jsdk_context_t *ctx);

/* ==========================================================================
 * 状态轮询调度器（jsdk_state_poll.c）
 * ======================================================================== */

/**
 * cycle_end()：按配置的周期/轮转/每 tick 上限发出状态请求帧。
 *
 * 配置关闭（`cfg.state_poll_period_ms == 0`）时**立即返回**，不改变任何行为。
 */
void jsdk_state_poll__cycle_end(jsdk_context_t *ctx);

/**
 * cycle_begin()（**`pump_rx()` 之后**）：结掉已到期的在途状态轮询请求。
 *
 * 超时会在这里计数 `state_timeout` 并释放槽位（所以即使设备完全不答，
 * 调度器也不会永久卡在“有一个在途请求”上）。
 */
void jsdk_state_poll__cycle_begin(jsdk_context_t *ctx);

/**
 * 收帧路径钩子：某个关节收到了 0x41 / 0x44 的应答。
 *
 * 结掉在途请求并计数 `state_ok`（**只**处理在途字段匹配的那一帧，
 * 客户自己发的 `jsdk_joint_request_state()` 不会污染调度器的计数）。
 */
void jsdk_state_poll__on_reply(jsdk_context_t *ctx, jsdk_joint_t *j, uint8_t msgtype);

/** 解析后的轮询周期（ms）；0 = 关闭。供 `jsdk_joint_stale_ms()` 计入阈值。 */
uint32_t jsdk_state_poll_period_ms(const jsdk_context_t *ctx);

/**
 * 设备侧协议级超时（`can.config.break_timeout`，单位 ms）。
 *
 * @return `> 0` = 超时毫秒数（设备侧已武装后才生效）；**`JSDK_WD_DISABLED_MS`（0）
 *          = 设备侧超时检测已禁用**（最新固件语义，见 PROTOCOL_NOTES §4.6）。
 *         句柄无效时也返回 0（“未知”按“不巡喂”处理）。
 *
 * ⚠ 调用方必须把 0 当作“**没有门限**”而不是“一个很小的超时”：
 *   例如 `period_ms >= wd` 这种校验在 0 时会**恒真**，必须显式跳过。
 */
uint32_t jsdk_watchdog_device_ms(const jsdk_joint_t *j);

/** 刷新一个关节的归一化状态（在线/故障/使能）并触发故障回调边沿。 */
void jsdk_joint__refresh_state(jsdk_joint_t *j);

/** WP4：刷新 `age_ms` 并在反馈超时后置 `JSDK_JF_FEEDBACK_STALE`（jsdk_fault.c）。 */
void jsdk_joint__refresh_freshness(jsdk_joint_t *j);

/** WP4：反馈超时阈值（由心跳周期/控制周期观测推导）。 */
uint32_t jsdk_joint_stale_ms(const jsdk_joint_t *j);

/** 把描述符读到的标定量程应用到关节（由 configure() 调用）。 */
void jsdk_joint__apply_calibration(jsdk_joint_t *j);

/**
 * 阻塞读一个参数值（配置期）—— **只发一次请求，最多拿回一次能给的字节**。
 *
 * ⚠ 这是给“值已知 ≤ 4 B”的调用方（标定量程、`current_state`、`break_timeout`）
 *   用的**探索式**读：请求里写死 `ReqLen = 4`，返回多少字节全看设备。
 *   要读满一个值的全部字节（`u64/i64/f64` 这些 8 字节类型）**必须**用
 *   @ref jsdk_ctx_read_param_exact —— 否则设备只会回 4 字节，调用方再拿
 *   `len < 8` 去报错，就会得到一个指向错误方向的“描述符不符”消息。
 *
 * @param out     输出缓冲（≥ 8 字节）
 * @param out_len 输出实际字节数（可 NULL）
 * @param allow_retry 非 0 = 允许幂等重发（面向用户的命令）；0 = 只发一次（轮询）。
 */
int jsdk_ctx_read_param_ex(jsdk_context_t *ctx, uint8_t node_id, uint16_t ep_id,
                           uint8_t *out, uint8_t *out_len, uint32_t timeout_ms,
                           int allow_retry);

/** 读参数 + 幂等重发（面向用户的命令走这个）。 */
int jsdk_ctx_read_param(jsdk_context_t *ctx, uint8_t node_id, uint16_t ep_id,
                        uint8_t *out, uint8_t *out_len, uint32_t timeout_ms);

/**
 * 读参数，**只发一次**。
 *
 * ⚠ 用于**轮询**（标定/回零期间隔 `pace_ms` 重问）：那里不需要重发，
 *   多等一个超时（真机 3 s 级）只会把节奏拖坏，而且下一次问马上就要发。
 */
int jsdk_ctx_read_param_once(jsdk_context_t *ctx, uint8_t node_id, uint16_t ep_id,
                             uint8_t *out, uint8_t *out_len, uint32_t timeout_ms);

/**
 * 阻塞读一个参数值：**精确读满 `want` 字节**，必要时分块。
 *
 * 分块规则完全按固件的切分行为来（不猜）：
 *  - 一次请求能拿多少由**设备侧**归一化决定：FD 下 `≤ 8`，Classic 下 `≤ 4`；
 *  - FD 且 `want ≤ 8` → **一次请求**（请求帧里 `ReqLen = want`，用 4 B 旧式形式，
 *    兼容性最好）；
 *  - Classic 且 `want > 4` → 存 4 字节一块：第一块用 4 B 旧式形式（offset 隐含 0），
 *    后续块用 8 B 形式带 offset（`cb_param_pack_read_req` 的 `with_offset`）。
 *
 * 每一块都要求设备确实推进（`data_len > 0`）；循环因为“每块至少 1 字节”而必然
 * 终止（`want ≤ CB_PARAM_MAX_VALUE = 8`），不存在卡死风险。
 *
 * @param out      输出缓冲，至少 @p want 字节
 * @param want     需要的字节数（1..CB_PARAM_MAX_VALUE）
 * @param out_len  输出：实际读到的字节数（成功时等于 @p want，可 NULL）
 * @return JSDK_OK；值比 @p want 短 → `JSDK_ERR_PROTOCOL`（并写入一句话说明）；
 *         端点不存在 → `JSDK_ERR_NOT_FOUND`；设备回了 0 字节 → 同上
 */
int jsdk_ctx_read_param_exact(jsdk_context_t *ctx, uint8_t node_id, uint16_t ep_id,
                              uint8_t *out, uint8_t want, uint8_t *out_len,
                              uint32_t timeout_ms);

/**
 * 阻塞写一个参数值（配置阶段）；写完等 8 字节静默 ACK。
 *
 * @param val 值字节，**必须是线上小端序**（设备端参数通路是 memcpy 主机序；
 *            控制帧/查询响应才是大端，见 `cb_frame.h` 的 `cb_le_*` 说明）。
 *            要写数值请用 `jsdk_joint_param_set*()`，它内部会做 `cb_le_put_*()`。
 */
int jsdk_ctx_write_param(jsdk_context_t *ctx, uint8_t node_id, uint16_t ep_id,
                         const void *val, uint8_t len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* JSDK_CORE_INTERNAL_H */
