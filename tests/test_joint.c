/**
 * @file    test_joint.c
 * @brief   WP3 关节层端到端测试：虚拟总线上 enable → 5 种模式 → disable
 *
 * @par 为什么描述符要用**非阻塞**接口先下好
 *  虚拟 HAL 的时钟只在 `jsdk_hal_virtual_advance_ms()` 里前进，而设备侧
 *  `0x25` 数据帧是 `sim_tick()` 按每毫秒预算推送的。因此 `configure()` 内部
 *  那种"阻塞自旋等帧"的写法在**仿真时钟**下永远收不完（真实 HAL 的自由运行
 *  计数器没这个问题）。
 *  → 测试先跑 `jsdk_context_desc_poll()` 并把仿真时间一格一格推进，
 *    等描述符就位后再调 `configure()`（它会跳过下载，只做握手 + 标定）。
 *    这条路同时也是 MCU 裸机客户该走的路。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "jsdk_hal_builtin.h"
#include "hal_virtual_internal.h"
#include "jsdk_core_internal.h"

#ifndef JSDK_TEST_DATA_DIR
#  define JSDK_TEST_DATA_DIR "."
#endif

static unsigned g_checks;
static unsigned g_fail;

#define CHECK(cond)                                                            \
    do {                                                                       \
        g_checks++;                                                            \
        if (!(cond)) {                                                         \
            printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);           \
            g_fail++;                                                          \
        }                                                                      \
    } while (0)

#define CHECK_EQ(a, b)                                                         \
    do {                                                                       \
        long long _a = (long long)(a), _b = (long long)(b);                    \
        g_checks++;                                                            \
        if (_a != _b) {                                                        \
            printf("  FAIL %s:%d  %s = %lld, expected %s = %lld\n",            \
                   __FILE__, __LINE__, #a, _a, #b, _b);                        \
            g_fail++;                                                          \
        }                                                                      \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                  \
    do {                                                                       \
        double _a = (double)(a), _b = (double)(b);                             \
        g_checks++;                                                            \
        if (!(fabs(_a - _b) <= (double)(tol))) {                               \
            printf("  FAIL %s:%d  |%.6f - %.6f| > %g\n", __FILE__, __LINE__,   \
                   _a, _b, (double)(tol));                                     \
            g_fail++;                                                          \
        }                                                                      \
    } while (0)

/* ==========================================================================
 * 测试夹具
 * ======================================================================== */

static char  *g_json;
static size_t g_json_len;

static int load_fixture(void)
{
    FILE *fh = fopen(JSDK_TEST_DATA_DIR "/endpoints_v8.json", "rb");
    long  sz;

    if (!fh) return -1;
    if (fseek(fh, 0L, SEEK_END) != 0) { fclose(fh); return -1; }
    sz = ftell(fh);
    if (sz <= 0) { fclose(fh); return -1; }
    rewind(fh);
    g_json = (char *)malloc((size_t)sz);
    if (!g_json) { fclose(fh); return -1; }
    if (fread(g_json, 1u, (size_t)sz, fh) != (size_t)sz) {
        free(g_json); g_json = NULL; fclose(fh); return -1;
    }
    fclose(fh);
    g_json_len = (size_t)sz;
    return 0;
}

typedef struct {
    jsdk_hal_handle_t          *h;
    jsdk_can_hal_t              hal;
    sim_bus_t                  *sim;
    jsdk_context_storage_t      store;
    jsdk_context_t             *ctx;
    jsdk_joint_t               *j;
    uint32_t                    now_ms;
    unsigned                    cycles;
    /** 调用者配置：必须与 arena 同寿命（上下文会往 desc.arena_used 回写） */
    jsdk_context_config_t       cfg;
} fix_t;

static void fx_now(fix_t *fx) { fx->now_ms = fx->hal.now_ms(fx->hal.user); }

/**
 * 建一条虚拟总线 + 一个上下文 + 一个关节，并把描述符下好（非阻塞路径）。
 * @param node_spec 传给虚拟后端的节点规格
 */
static int fx_up(fix_t *fx, const char *node_spec)
{
    jsdk_context_config_t *cfg = &fx->cfg;   /* fixture 成员：与 arena 同寿命 */
    jsdk_joint_config_t    jc;
    jsdk_status_t          st;
    static uint8_t         arena[32768];
    unsigned               guard = 0u;

    memset(fx, 0, sizeof *fx);

    if (jsdk_hal_virtual_open(&fx->hal, &fx->h, node_spec) != JSDK_OK) return -1;
    fx->sim = jsdk_hal_virtual_sim(fx->h);
    if (!fx->sim) return -1;
    if (sim_set_desc(fx->sim, g_json, (uint32_t)g_json_len, 0x1234u) != 0) return -1;

    jsdk_context_config_default(cfg);
    cfg->hal = fx->hal;
    cfg->master_id = 1u;
    cfg->is_fd = 1u;
    cfg->period_ns = 1000000u;          /* 1 kHz */
    cfg->auto_keepalive = 1u;
    cfg->desc.retain = JSDK_DESC_RETAIN_ALL;
    cfg->desc.arena = arena;
    cfg->desc.arena_size = sizeof arena;
    cfg->desc.timeout_ms = 5000u;

    st = jsdk_context_init((jsdk_context_t *)&fx->store, cfg);
    if (st != JSDK_OK) return -1;
    fx->ctx = (jsdk_context_t *)&fx->store;

    memset(&jc, 0, sizeof jc);
    jc.node_id = 1u;
    jc.initial_mode = JSDK_MODE_MIT;
    if (jsdk_context_add_joint(fx->ctx, &jc, &fx->j) != JSDK_OK) return -1;

    /* ---- 非阻塞下载描述符：每次推进 1 ms 仿真时间 ---- */
    for (guard = 0u; guard < 4000u; ++guard) {
        st = jsdk_context_desc_poll(fx->ctx, 0u);
        if (st == JSDK_OK) break;
        if (st != JSDK_ERR_BUSY) {
            printf("      desc_poll -> %d (%s)\n", (int)st, jsdk_context_last_error(fx->ctx));
            return -1;
        }
        jsdk_hal_virtual_advance_ms(fx->h, 1u);
    }
    if (guard >= 4000u) return -1;

    if (jsdk_context_configure(fx->ctx) != JSDK_OK) {
        printf("      configure -> %s\n", jsdk_context_last_error(fx->ctx));
        return -1;
    }
    return 0;
}

static void fx_down(fix_t *fx)
{
    if (fx->ctx) jsdk_context_destroy(fx->ctx);
    if (fx->h) jsdk_hal_close(fx->h);
}

/** 跑一个控制周期（推进 1 ms 仿真时间，让设备物理模型与心跳都动起来）。 */
static void fx_cycle(fix_t *fx)
{
    jsdk_context_cycle_begin(fx->ctx, (uint64_t)fx->now_ms * 1000000ull);
    jsdk_context_cycle_end(fx->ctx);
    jsdk_hal_virtual_advance_ms(fx->h, 1u);
    fx_now(fx);
    fx->cycles++;
}

/** 取 SDK 发出的下一帧（供字节级断言）。 */
static int fx_take_tx(fix_t *fx, jsdk_can_frame_t *f)
{
    return jsdk_hal_virtual_capture(fx->h, f);
}

static void drain_tx(fix_t *fx)
{
    jsdk_can_frame_t f;
    while (fx_take_tx(fx, &f)) { /* 丢弃 */ }
}

/* ==========================================================================
 * 1. 单位换算（表驱动）
 * ======================================================================== */

static void test_units(void)
{
    struct row {
        double gear, in, expect;
        const char *what;
    };
    static const struct row rows[] = {
        { 16.0,  1.0,          1.0 * 16.0 / (2.0 * M_PI), "rad -> motor turns" },
        { 16.0,  6.283185307,  16.0,                     "one full output turn" },
        { 1.0,   1.0,          1.0 / (2.0 * M_PI),       "gear = 1" },
        { 100.0, 0.5,          0.5 * 100.0 / (2.0 * M_PI), "large gear" },
    };
    unsigned i;

    printf("[1] unit conversion\n");

    for (i = 0u; i < sizeof rows / sizeof rows[0]; ++i) {
        CHECK_NEAR(jsdk_units_rad_to_turns(rows[i].in, rows[i].gear), rows[i].expect, 1e-6);
        CHECK_NEAR(jsdk_units_turns_to_rad(rows[i].expect, rows[i].gear), rows[i].in, 1e-6);
    }

    /* rad/s <-> RPM */
    CHECK_NEAR(jsdk_units_rad_s_to_rpm(M_PI), 30.0, 1e-9);
    CHECK_NEAR(jsdk_units_rpm_to_rad_s(60.0), 2.0 * M_PI, 1e-9);
    CHECK_NEAR(jsdk_units_rpm_to_rad_s(jsdk_units_rad_s_to_rpm(1.234)), 1.234, 1e-12);

    /* kp 量纲修正（§6.2）：刚度 = kp × gear / 2π */
    {
        double gear = 16.5;
        double kp = 500.0;
        CHECK_NEAR(jsdk_units_kp_to_stiffness(kp, gear), 500.0 * 16.5 / (2.0 * M_PI), 1e-6);
        CHECK_NEAR(jsdk_units_stiffness_to_kp(jsdk_units_kp_to_stiffness(kp, gear), gear),
                   kp, 1e-9);
        /* 文档里的"约 2.63 倍" */
        CHECK_NEAR(jsdk_units_kp_to_stiffness(kp, gear) / kp, 2.626, 5e-3);
    }
    printf("      gear=16.5: kp=500 -> %.3f N.m/rad (%.3fx the naive reading)\n",
           jsdk_units_kp_to_stiffness(500.0, 16.5),
           jsdk_units_kp_to_stiffness(500.0, 16.5) / 500.0);

    /* 非法齿比：绝不除零，也不"猜 1" */
    CHECK_EQ(jsdk_units_rad_to_turns(1.0, 0.0), 0);
    CHECK_EQ(jsdk_units_turns_to_rad(1.0, -1.0), 0);
    CHECK_EQ(jsdk_units_stiffness_to_kp(1.0, 0.0), 0);

    /* jsdk_unit_scale_* */
    {
        jsdk_unit_scale_t s;

        jsdk_unit_scale_default(&s, 1u);
        CHECK_EQ(s.valid, 1);
        CHECK_NEAR(s.trq_to_Nm, 1.0, 0);

        jsdk_unit_scale_default(&s, 0u);
        CHECK_EQ(s.valid, 0);

        /* counts → rad：编码器在电机侧，gear = shaft_rev / motor_rev */
        jsdk_unit_scale_calc(&s, 8192u, 1u, 16u, 50u);
        CHECK_EQ(s.valid, 1);
        CHECK_NEAR(s.pos_counts_to_rad, 2.0 * M_PI * 1.0 / (8192.0 * 16.0), 1e-15);
        CHECK_NEAR(s.trq_to_Nm, 0.05, 1e-12);

        /* 任一参数为 0 → 一律无效（不猜） */
        jsdk_unit_scale_calc(&s, 0u, 1u, 16u, 50u);
        CHECK_EQ(s.valid, 0);
        jsdk_unit_scale_calc(&s, 8192u, 0u, 16u, 50u);
        CHECK_EQ(s.valid, 0);
        CHECK_NEAR(s.pos_counts_to_rad, 0.0, 0);
    }

    /* 多圈展开：默认不启用；显式用时跨圈必须正确 */
    {
        long turns = -99;

        CHECK_NEAR(jsdk_units_pos_unwrap(0.0, 1.0, 12.5, &turns), 1.0, 1e-12);
        CHECK_EQ(turns, 0);

        /* 从 +12 跳到 -12 → 认为是跨圈（越过 +12.5）→ 绝对位 12.5+12 = 24.5 */
        CHECK_NEAR(jsdk_units_pos_unwrap(12.0, -12.0, 12.5, &turns), 13.0, 1e-9);
        CHECK_EQ(turns, 1);

        /* 无量程信息 → 原样返回，不推断 */
        CHECK_NEAR(jsdk_units_pos_unwrap(3.0, 7.0, 0.0, &turns), 7.0, 0);
        CHECK_EQ(turns, 0);
    }
    printf("      table-driven rad/turns, rad/s/RPM, kp/stiffness, pos_unwrap OK\n");
}

/* ==========================================================================
 * 2. 框体与 ABI 守卫
 * ======================================================================== */

static void test_lifecycle(void)
{
    jsdk_context_storage_t store;
    jsdk_context_config_t  cfg;
    jsdk_hal_handle_t     *h = NULL;
    jsdk_can_hal_t         hal;
    jsdk_joint_config_t    jc;
    jsdk_context_t        *ctx = (jsdk_context_t *)&store;
    static uint8_t         arena[4096];

    printf("[2] lifecycle / ABI guards\n");

    CHECK_EQ(strcmp(jsdk_backend_name(), JSDK_BACKEND_NAME_CAN), 0);
    CHECK_EQ(jsdk_abi_version(), JSDK_ABI_VERSION_CAN);
    CHECK_EQ(strcmp(jsdk_status_string(JSDK_ERR_PROTOCOL), "protocol-error"), 0);
    CHECK_EQ(strcmp(jsdk_mode_string(JSDK_MODE_MIT), "mit(0x00)"), 0);
    CHECK_EQ(strcmp(jsdk_axis_state_string(JSDK_AXIS_FAULT), "fault"), 0);

    CHECK_EQ(jsdk_hal_virtual_open(&hal, &h, "0:id=1,fd"), JSDK_OK);

    jsdk_context_config_default(&cfg);
    CHECK_EQ(cfg.master_id, 1u);
    CHECK_EQ(cfg.is_fd, 1u);
    CHECK_EQ(cfg.auto_keepalive, 1u);
    CHECK_EQ(cfg.clamp_target_position, 0u);

    /* --- 参数校验 --- */
    {
        jsdk_context_config_t bad = cfg;

        bad.master_id = 0u;                     /* 设备将完全不回复 → 必须早拦 */
        bad.hal = hal;
        bad.desc.arena = arena; bad.desc.arena_size = sizeof arena;
        CHECK_EQ(jsdk_context_init(ctx, &bad), JSDK_ERR_INVALID_ARG);

        bad = cfg;
        bad.hal = hal;
        bad.desc.arena = NULL;                  /* 零 malloc：没有 arena 就不行 */
        CHECK_EQ(jsdk_context_init(ctx, &bad), JSDK_ERR_INVALID_ARG);

        bad = cfg;
        bad.hal.send = NULL;                    /* HAL 三件套必需 */
        bad.desc.arena = arena; bad.desc.arena_size = sizeof arena;
        CHECK_EQ(jsdk_context_init(ctx, &bad), JSDK_ERR_INVALID_ARG);
    }

    /* --- 魔数守卫：别人后端的存储必须被拒 --- */
    memset(&store, 0, sizeof store);
    store.bytes[0] = 0x5A;                      /* 非 0 且不是本后端魔数 */
    cfg.hal = hal;
    cfg.desc.arena = arena; cfg.desc.arena_size = sizeof arena;
    CHECK_EQ(jsdk_context_init(ctx, &cfg), JSDK_ERR_INVALID_ARG);

    /* --- 正常初始化 / 重复 initialize --- */
    memset(&store, 0, sizeof store);
    CHECK_EQ(jsdk_context_init(ctx, &cfg), JSDK_OK);
    CHECK_EQ(jsdk_context_init(ctx, &cfg), JSDK_ERR_BAD_STATE);  /* 已初始化 */

    /* 未加关节 → configure 必须拒绝，而不是"空跑成功" */
    CHECK_EQ(jsdk_context_configure(ctx), JSDK_ERR_BAD_STATE);

    memset(&jc, 0, sizeof jc);
    jc.node_id = 1u;
    jc.initial_mode = JSDK_MODE_MIT;
    CHECK_EQ(jsdk_context_add_joint(ctx, &jc, NULL), JSDK_OK);
    /* 重复 node_id 必须拒绝 */
    CHECK_EQ(jsdk_context_add_joint(ctx, &jc, NULL), JSDK_ERR_INVALID_ARG);
    jc.node_id = 0u;
    CHECK_EQ(jsdk_context_add_joint(ctx, &jc, NULL), JSDK_ERR_INVALID_ARG);
    jc.node_id = 1u; jc.axis = 1u;
    CHECK_EQ(jsdk_context_add_joint(ctx, &jc, NULL), JSDK_ERR_UNSUPPORTED);
    jc.node_id = 255u; jc.axis = 0u;
    CHECK_EQ(jsdk_context_add_joint(ctx, &jc, NULL), JSDK_ERR_INVALID_ARG);

    /* 记录一下上下文实测大小（DESIGN 里的存储预算靠它） */
    printf("      sizeof(context)=%u bytes, sizeof(joint)=%u bytes, "
           "JSDK_CONTEXT_MAX_SIZE=%u\n",
           (unsigned)jsdk_context_size(&cfg), (unsigned)jsdk_context_joint_size(),
           (unsigned)JSDK_CONTEXT_MAX_SIZE);
    CHECK(sizeof(jsdk_context_t) <= JSDK_CONTEXT_MAX_SIZE);

    jsdk_context_destroy(ctx);
    jsdk_hal_close(h);
}

/* ==========================================================================
 * 3. 配置：描述符 + 标定读回
 * ======================================================================== */

static void test_configure(void)
{
    fix_t fx;

    printf("[3] configure (descriptor + calibration read-back)\n");

    if (fx_up(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                   "kpmax=500,kdmax=5,hb=10,fd") != 0) {
        printf("      FATAL: fixture setup failed\n");
        g_fail++;
        g_checks++;
        return;
    }

    {
        jsdk_desc_info_t di;
        CHECK_EQ(jsdk_context_get_desc_info(fx.ctx, &di), JSDK_OK);
        CHECK_EQ(di.total_len, (uint32_t)g_json_len);
        CHECK_EQ(di.crc, 0x1234u);
        CHECK_EQ(di.complete, 1u);
        CHECK_EQ(di.endpoint_count, 594u);
        printf("      descriptor: %u endpoints, crc=0x%04X, %u frames\n",
               di.endpoint_count, (unsigned)di.crc, di.frames_rx);

        /* 公共头把 desc.arena_used 声明为**输出**（"可按实测缩容"）。
           init 会复制配置，所以必须显式写回调用方 —— 这里就验它真的回来了。 */
        printf("      arena_used = %u B written back into the caller's config\n",
               (unsigned)fx.cfg.desc.arena_used);
        CHECK(fx.cfg.desc.arena_used > 0u);
        CHECK(fx.cfg.desc.arena_used < fx.cfg.desc.arena_size);
        CHECK(fx.cfg.desc.arena_used <= 25493u);   /* 已知上界（594 端点） */

        /* 端点查名 → ID（证明用的是运行时描述符，不是内置表） */
        {
            uint16_t id = 0u;
            jsdk_ep_type_t t = JSDK_EP_JSON;
            uint8_t acc = 0u;
            CHECK_EQ(jsdk_endpoint_lookup(fx.ctx, "axis0.motor.config.gear_ratio",
                                          &id, &t, &acc), JSDK_OK);
            CHECK_EQ(id, 242u);
            CHECK_EQ(t, JSDK_EP_F32);
            CHECK_EQ(acc, (JSDK_EP_ACCESS_R | JSDK_EP_ACCESS_W));
            CHECK_EQ(jsdk_endpoint_lookup(fx.ctx, "no.such.path", &id, &t, &acc),
                     JSDK_ERR_NOT_FOUND);
        }
    }

    /* 标定值必须来自设备（gear=16 / tconst=0.0385 / 各量程） */
    {
        jsdk_joint_config_snapshot_t snap;
        CHECK_EQ(jsdk_joint_read_config_snapshot(fx.j, &snap), JSDK_OK);
        CHECK_EQ(snap.valid, 1);
        CHECK_NEAR(snap.gear_ratio, 16.0, 1e-6);
        CHECK_NEAR(snap.torque_constant, 0.0385, 1e-7);
        CHECK_NEAR(snap.mit_max_pos, 12.5, 1e-6);
        CHECK_NEAR(snap.mit_max_vel, 65.0, 1e-6);
        CHECK_NEAR(snap.mit_max_torque, 50.0, 1e-6);
        CHECK_NEAR(snap.mit_max_kp, 500.0, 1e-6);
        CHECK_NEAR(snap.mit_max_kd, 5.0, 1e-6);
        /* ⚠ 新语义：设备侧 break_timeout = 0 = **超时检测已禁用**（不再是“当 100 ms”）。
           本夹具没写 `timeout=` ⇒ 用固件默认的 0 ⇒ 快照必须是 0。
           （这里同时验证了“禁用时不得拿 0 去比周期”——fx_up 给的控制周期是 1 ms。） */
        CHECK_EQ(snap.break_timeout_ms, 0u);
        printf("      calibrated: gear=%.3f tconst=%.4f pos=%.1f vel=%.1f "
               "tau=%.1f kp=%.1f kd=%.1f wd=%ums(0=disabled)\n",
               snap.gear_ratio, snap.torque_constant, snap.mit_max_pos,
               snap.mit_max_vel, snap.mit_max_torque, snap.mit_max_kp,
               snap.mit_max_kd, (unsigned)snap.break_timeout_ms);
    }

    /* 反馈：设备在跑，反馈必须新鲜 */
    {
        jsdk_joint_feedback_t fb;
        unsigned i;
        for (i = 0u; i < 5u; ++i) fx_cycle(&fx);
        CHECK_EQ(jsdk_joint_get_feedback(fx.j, &fb), JSDK_OK);
        CHECK_EQ(fb.online, 1);
        CHECK_EQ(jsdk_axis_state_string(fb.axis_state)[0] != '\0', 1);
        printf("      feedback: pos=%.4f rad state=%s err=%u hb_err=%u\n",
               fb.pos, jsdk_axis_state_string(fb.axis_state),
               (unsigned)fb.err_code, (unsigned)fb.hb_error);
    }

    /* 未使能 → 不是 enabled，也不该误报故障 */
    CHECK_EQ(jsdk_joint_is_enabled(fx.j), 0);
    CHECK_EQ(jsdk_joint_is_fault(fx.j), 0);

    fx_down(&fx);
}

/* ==========================================================================
 * 4. enable → 5 种模式 → disable
 * ======================================================================== */

/** 找出一帧里 MsgType 匹配的记录。 */
static int saw_msgtype(fix_t *fx, uint8_t msgtype, jsdk_can_frame_t *out)
{
    jsdk_can_frame_t f;
    while (fx_take_tx(fx, &f)) {
        if (cb_id_msgtype(f.id) == msgtype) {
            if (out) *out = f;
            return 1;
        }
    }
    return 0;
}

static void test_enable_modes_disable(void)
{
    fix_t fx;
    unsigned i;

    printf("[4] enable -> 5 modes -> disable\n");

    if (fx_up(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                   "kpmax=500,kdmax=5,hb=10,fd") != 0) {
        printf("      FATAL: fixture setup failed\n");
        g_fail++; g_checks++;
        return;
    }

    /* ---- enable ---- */
    drain_tx(&fx);
    jsdk_joint_request_enable(fx.j, JSDK_MODE_MIT);
    for (i = 0u; i < 20u && !jsdk_joint_is_enabled(fx.j); ++i) fx_cycle(&fx);

    CHECK_EQ(jsdk_joint_is_enabled(fx.j), 1);
    CHECK_EQ(jsdk_joint_get_mode_state(fx.j) == JSDK_MODESTATE_MIT, 1);
    printf("      enabled: mode_state=%d axis_state=%s\n",
           (int)jsdk_joint_get_mode_state(fx.j),
           jsdk_axis_state_string(((jsdk_joint_feedback_t *)&fx.j->fb)->axis_state));

    /* CLEAR_ERRORS + START_MOTOR 必须都出现在总线上（§6.4 顺序） */
    drain_tx(&fx);

    /* ---- MIT：一条命令的字节必须能被设备逐字段解出 ---- */
    {
        jsdk_can_frame_t f;
        cb_mit_range_t   r;

        jsdk_joint_set_mit(fx.j, 1.0, -2.0, 30.0, 1.5, 3.0);
        drain_tx(&fx);
        fx_cycle(&fx);

        CHECK_EQ(saw_msgtype(&fx, CB_MSG_MIT_CONTROL, &f), 1);
        CHECK_EQ(f.len, 8u);
        CHECK_EQ(cb_id_dest(f.id), 1u);
        CHECK_EQ(cb_id_source(f.id), 1u);          /* master_id = 1 */

        r.pos_max = fx.j->range.pos_max; r.vel_max = fx.j->range.vel_max;
        r.kp_max  = fx.j->range.kp_max;  r.kd_max  = fx.j->range.kd_max;
        r.tau_max = fx.j->range.tau_max;
        {
            float pos = 0, vel = 0, kp = 0, kd = 0, tau = 0;
            cb_mit_unpack_command(f.data, &r, &pos, &vel, &kp, &kd, &tau);
            CHECK_NEAR(pos, 1.0, 12.5 / 32767.0 * 2);
            CHECK_NEAR(vel, -2.0, 65.0 / 4095.0 * 2);
            CHECK_NEAR(kp, 30.0, 500.0 / 4095.0 * 2);
            CHECK_NEAR(kd, 1.5, 5.0 / 4095.0 * 2);
            CHECK_NEAR(tau, 3.0, 50.0 / 4095.0 * 2);
        }
        printf("      MIT: 8 B, dest=1, src=1, all five fields round-trip\n");
    }

    /* ---- CSP：线上是【度】+【RPM】 ---- */
    {
        jsdk_can_frame_t f;
        float pos_deg = 0, vel_rpm = 0, cur_a = 0;

        jsdk_joint_set_mode(fx.j, JSDK_MODE_CSP);
        jsdk_joint_set_target_position_rad(fx.j, M_PI / 2.0);   /* 90 deg */
        jsdk_joint_set_limits(fx.j, M_PI, 10.0);                /* 180 deg/s, 10 A */
        drain_tx(&fx);
        fx_cycle(&fx);

        CHECK_EQ(saw_msgtype(&fx, CB_MSG_POS_CONTROL, &f), 1);
        CHECK_EQ(f.len, 12u);                                   /* FD 变体 */
        CHECK_EQ(cb_ctrl_pos_unpack(f.data, f.len, 0, &pos_deg, &vel_rpm, &cur_a), 0);
        CHECK_NEAR(pos_deg, 90.0, 1e-3);
        CHECK_NEAR(vel_rpm, 30.0, 1e-2);                        /* π rad/s = 30 RPM */
        CHECK_NEAR(cur_a, 10.0, 1e-5);
        printf("      CSP: pos=%.3f deg  vel_lim=%.2f rpm  cur_lim=%.2f A\n",
               pos_deg, vel_rpm, cur_a);
    }

    /* ---- CSV：线上是【RPM】 ---- */
    {
        jsdk_can_frame_t f;
        float vel_rpm = 0, cur_a = 0;

        jsdk_joint_set_mode(fx.j, JSDK_MODE_CSV);
        jsdk_joint_set_target_velocity_rad_s(fx.j, M_PI);       /* 30 RPM */
        jsdk_joint_set_limits(fx.j, 0.0, 5.0);
        drain_tx(&fx);
        fx_cycle(&fx);

        CHECK_EQ(saw_msgtype(&fx, CB_MSG_VEL_CONTROL, &f), 1);
        CHECK_EQ(cb_ctrl_vel_unpack(f.data, f.len, &vel_rpm, &cur_a), 0);
        CHECK_NEAR(vel_rpm, 30.0, 1e-3);
        CHECK_NEAR(cur_a, 5.0, 1e-5);
        printf("      CSV: vel=%.3f rpm  cur_lim=%.2f A\n", vel_rpm, cur_a);
    }

    /* ---- CST：**输出端** N·m → 线上【电机端】N·m（除以 gear） ---- */
    {
        jsdk_can_frame_t f;
        float tau_motor = 0;

        jsdk_joint_set_mode(fx.j, JSDK_MODE_CST);
        jsdk_joint_set_target_torque_Nm(fx.j, 16.0);            /* 输出端 16 N·m */
        drain_tx(&fx);
        fx_cycle(&fx);

        CHECK_EQ(saw_msgtype(&fx, CB_MSG_TORQUE_CONTROL, &f), 1);
        CHECK_EQ(cb_ctrl_torque_unpack(f.data, f.len, &tau_motor), 0);
        CHECK_NEAR(tau_motor, 1.0, 1e-5);                       /* 16 / gear16 = 1 */
        printf("      CST: 16 N.m(output) -> %.3f N.m(motor wire)\n", tau_motor);
    }

    /* ---- CURRENT：线上一律是电机端 A，且**不喂**协议看门狗 ---- */
    {
        jsdk_can_frame_t f;
        float cur = 0;

        jsdk_joint_set_mode(fx.j, JSDK_MODE_CURRENT);
        drain_tx(&fx);
        fx_cycle(&fx);

        CHECK_EQ(saw_msgtype(&fx, CB_MSG_CURRENT_CONTROL, &f), 1);
        CHECK_EQ(cb_ctrl_current_unpack(f.data, f.len, &cur), 0);
        CHECK_NEAR(cur, 0.0, 1e-6);       /* hold_position 之外的默认目标为 0 */
        printf("      CURRENT: %.3f A (is_ctrl=false -> keepalive must cover it)\n", cur);
    }

    /* ---- disable：顺序必须是 安全帧 → STOP_MOTOR → IDLE ---- */
    jsdk_joint_request_disable(fx.j);
    for (i = 0u; i < 20u && jsdk_joint_is_enabled(fx.j); ++i) fx_cycle(&fx);
    CHECK_EQ(jsdk_joint_is_enabled(fx.j), 0);
    printf("      disabled: mode_state=%d enabled=%d\n",
           (int)jsdk_joint_get_mode_state(fx.j), jsdk_joint_is_enabled(fx.j));

    fx_down(&fx);
}

/* ==========================================================================
 * 5. 越界策略（§6.10）
 * ======================================================================== */

static void test_target_rejection(void)
{
    fix_t fx;
    unsigned i;

    printf("[5] out-of-range target policy\n");

    if (fx_up(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                   "kpmax=500,kdmax=5,hb=10,fd") != 0) {
        printf("      FATAL: fixture setup failed\n");
        g_fail++; g_checks++;
        return;
    }

    jsdk_joint_request_enable(fx.j, JSDK_MODE_MIT);
    for (i = 0u; i < 20u && !jsdk_joint_is_enabled(fx.j); ++i) fx_cycle(&fx);
    CHECK_EQ(jsdk_joint_is_enabled(fx.j), 1);

    /* --- 默认策略 0：拒绝 + 改发安全帧（**必须仍然发帧**） --- */
    {
        jsdk_can_frame_t f;
        cb_mit_range_t   r;
        jsdk_joint_feedback_t fb;
        float pos = 0, vel = 0, kp = 0, kd = 0, tau = 0;

        jsdk_joint_set_mit(fx.j, 999.0, 0.0, 0.0, 0.0, 0.0);    /* 远超 ±12.5 */
        drain_tx(&fx);
        fx_cycle(&fx);

        CHECK_EQ(saw_msgtype(&fx, CB_MSG_MIT_CONTROL, &f), 1);   /* 仍然发帧 */
        r.pos_max = fx.j->range.pos_max; r.vel_max = fx.j->range.vel_max;
        r.kp_max  = fx.j->range.kp_max;  r.kd_max  = fx.j->range.kd_max;
        r.tau_max = fx.j->range.tau_max;
        cb_mit_unpack_command(f.data, &r, &pos, &vel, &kp, &kd, &tau);

        CHECK(fabs(pos) <= 12.5 + 1e-3);                        /* 不是客户给的 999 */
        /* 定点量化：±50 N·m / 12 bit → 1 LSB = 100/4095 ≈ 0.0244。
           0 本身不是可精确表示的码（pack 四舍五入后会落到 2048），
           所以容差取 1 LSB 而不是 1e-9。 */
        CHECK_NEAR(kp, 0.0, 500.0 / 4095.0);                    /* 安全帧：kp=0 */
        CHECK_NEAR(kd, 0.0, 5.0 / 4095.0);
        CHECK_NEAR(tau, 0.0, 100.0 / 4095.0);

        CHECK_EQ(jsdk_joint_get_feedback(fx.j, &fb), JSDK_OK);
        CHECK((fb.status_flags & JSDK_JF_TARGET_REJECTED) != 0);
        CHECK((fb.status_flags & JSDK_JF_SAFE_FRAME_SENT) != 0);
        CHECK(fb.tx_rejected >= 1u);
        CHECK(strstr(jsdk_context_last_error(fx.ctx), "rejected") != NULL);
        printf("      reject: pos=%.3f rad (asked 999), flags=0x%04X, "
               "tx_rejected=%u\n", pos, fb.status_flags, fb.tx_rejected);
        printf("      last_error: %s\n", jsdk_context_last_error(fx.ctx));

        /* 关键：拒绝之后**看门狗不会超时**（因为发的是安全帧而不是丢帧） */
        jsdk_joint_clear_status_flags(fx.j, JSDK_JF_TARGET_REJECTED);
        for (i = 0u; i < 30u; ++i) fx_cycle(&fx);
        CHECK_EQ(jsdk_joint_is_enabled(fx.j), 1);
        CHECK_EQ(jsdk_joint_is_fault(fx.j), 0);
    }

    /* --- 策略 1：静默钳位，但仍计数/置位 --- */
    {
        fix_t gx;
        jsdk_can_frame_t f;
        cb_mit_range_t   r;
        jsdk_joint_feedback_t fb;
        float pos = 0, vel = 0, kp = 0, kd = 0, tau = 0;

        if (fx_up(&gx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                       "kpmax=500,kdmax=5,hb=10,fd") != 0) {
            printf("      FATAL: second fixture failed\n");
            g_fail++; g_checks++;
            fx_down(&fx);
            return;
        }
        gx.ctx->cfg.clamp_target_position = 1u;   /* 测试直接改配置副本 */

        jsdk_joint_request_enable(gx.j, JSDK_MODE_MIT);
        for (i = 0u; i < 20u && !jsdk_joint_is_enabled(gx.j); ++i) fx_cycle(&gx);
        CHECK_EQ(jsdk_joint_is_enabled(gx.j), 1);

        jsdk_joint_set_mit(gx.j, 999.0, 0.0, 0.0, 0.0, 0.0);
        drain_tx(&gx);
        fx_cycle(&gx);

        CHECK_EQ(saw_msgtype(&gx, CB_MSG_MIT_CONTROL, &f), 1);
        r.pos_max = gx.j->range.pos_max; r.vel_max = gx.j->range.vel_max;
        r.kp_max  = gx.j->range.kp_max;  r.kd_max  = gx.j->range.kd_max;
        r.tau_max = gx.j->range.tau_max;
        cb_mit_unpack_command(f.data, &r, &pos, &vel, &kp, &kd, &tau);
        CHECK_NEAR(pos, 12.5, 12.5 / 32767.0 * 2);   /* 钳到量程上限 */

        CHECK_EQ(jsdk_joint_get_feedback(gx.j, &fb), JSDK_OK);
        CHECK((fb.status_flags & JSDK_JF_TARGET_REJECTED) != 0);
        CHECK(fb.tx_rejected >= 1u);
        printf("      clamp : pos=%.3f rad (asked 999), tx_rejected=%u\n",
               pos, fb.tx_rejected);

        fx_down(&gx);
    }

    fx_down(&fx);
}

/* ==========================================================================
 * 6. 看门狗 / keepalive（§6.3、F19）
 * ======================================================================== */

static void test_watchdog(void)
{
    fix_t fx;
    unsigned i;

    printf("[6] watchdog / keepalive\n");

    if (fx_up(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                   "kpmax=500,kdmax=5,hb=10,timeout=20,fd") != 0) {
        printf("      FATAL: fixture setup failed\n");
        g_fail++; g_checks++;
        return;
    }

    jsdk_joint_request_enable(fx.j, JSDK_MODE_MIT);
    for (i = 0u; i < 20u && !jsdk_joint_is_enabled(fx.j); ++i) fx_cycle(&fx);
    CHECK_EQ(jsdk_joint_is_enabled(fx.j), 1);

    /* --- 从 CURRENT 模式（不是 is_ctrl）切出去：keepalive 必须补 MIT --- */
    jsdk_joint_set_mode(fx.j, JSDK_MODE_CURRENT);
    jsdk_joint_set_current_A(fx.j, 0.0);
    drain_tx(&fx);

    {
        uint32_t before = fx.j->keepalive_sent;
        unsigned mit_seen = 0u;

        for (i = 0u; i < 40u; ++i) {
            jsdk_can_frame_t f;
            fx_cycle(&fx);
            while (fx_take_tx(&fx, &f)) {
                /* 只统计发送方是主站、且不是 CURRENT 的 is_ctrl 帧 */
                if (cb_id_msgtype(f.id) == CB_MSG_MIT_CONTROL) mit_seen++;
            }
        }
        CHECK(mit_seen > 0u);                       /* 必须补喂过 */
        CHECK(fx.j->keepalive_sent > before);
        printf("      CURRENT-only client: %u keepalive MIT frames injected "
               "(protocol watchdog would otherwise expire)\n",
               (unsigned)(fx.j->keepalive_sent - before));
    }

    /* --- 停止跑 SDK 循环 → 设备侧必须在 break_timeout 后自己打进故障 --- */
    {
        sim_bus_t *sim = fx.sim;
        sim_node_t *n = sim_find_node(sim, 1u);
        unsigned guard;

        CHECK(n != NULL);
        if (n) {
            /* 先确认现在是“已武装”（说明 keepalive 真的在喂 is_ctrl 帧） */
            for (guard = 0u; guard < 40u && n->armed == 0u; ++guard) {
                jsdk_hal_virtual_advance_ms(fx.h, 1u);
            }
            CHECK_EQ(n->armed, 1u);
            CHECK_EQ(n->last_cmd_ms != 0u, 1);

            /* 只推进仿真时间、不跑 SDK 循环 → 100 ms 后必然超时 */
            for (guard = 0u; guard < 100u; ++guard) {
                jsdk_hal_virtual_advance_ms(fx.h, 1u);
                if (n->armed == 0u) break;
            }
            CHECK_EQ(n->armed, 0u);
            CHECK((n->error_axis & SIM_ERR_CAN_BUS_FAILED) != 0u);
            printf("      stop sending -> device armed=%u axis_err=0x%X after "
                   "break_timeout=20 ms\n", (unsigned)n->armed,
                   (unsigned)n->error_axis);
        }
    }

    /* --- 先锁在一个回归点上：固件的 `is_ctrl` 与 ctrl 模块的判据在 0x04 上**必须**不同 ---
       它们是两个概念，用错就会把固件侧 F19（纯电流客户端的安全阀永不武装）
       原样复制到 SDK 内部（本项目真的胉过这个坑）。 */
    {
        CHECK_EQ(jsdk_msgtype_feeds_watchdog(CB_MSG_MIT_CONTROL), 1);
        CHECK_EQ(jsdk_msgtype_feeds_watchdog(CB_MSG_POS_CONTROL), 1);
        CHECK_EQ(jsdk_msgtype_feeds_watchdog(CB_MSG_VEL_CONTROL), 1);
        CHECK_EQ(jsdk_msgtype_feeds_watchdog(CB_MSG_TORQUE_CONTROL), 1);
        CHECK_EQ(jsdk_msgtype_feeds_watchdog(CB_MSG_MIT_CONTROL_BCAST), 1);
        CHECK_EQ(jsdk_msgtype_feeds_watchdog(CB_MSG_TORQUE_CONTROL_BCAST), 1);
        CHECK_EQ(jsdk_msgtype_feeds_watchdog(CB_MSG_CURRENT_CONTROL), 0); /* ⚠ 关键 */
        CHECK_EQ(jsdk_msgtype_feeds_watchdog(CB_MSG_QUERY_STATUS), 0);
        CHECK_EQ(jsdk_msgtype_feeds_watchdog(CB_MSG_PARAM_READ), 0);
        CHECK_EQ(jsdk_msgtype_feeds_watchdog(CB_MSG_HEARTBEAT), 0);

        /* ctrl 模块的判据把 0x04 算成控制帧（这是对的，它是控制帧）；
           两者不同正是本文档要锁住的事实。 */
        CHECK_EQ(cb_ctrl_is_control_msgtype(CB_MSG_CURRENT_CONTROL), 1);
        CHECK_EQ(jsdk_msgtype_feeds_watchdog(CB_MSG_CURRENT_CONTROL)
                 != cb_ctrl_is_control_msgtype(CB_MSG_CURRENT_CONTROL), 1);
        printf("      is_ctrl: 0x00-0x03/0x80-0x83 feed the watchdog; 0x04 does NOT "
               "(cb_ctrl_is_control_msgtype says otherwise by design)\n");
    }

    printf("[7] refusals / negative paths\n");

    /* --- 关节仍被标记为使能中 → configure() 必须拒绝 ---
       （上一段把设备跑到了超时故障，但 SDK 没跑循环就不知道；
        而按 SDK 的语义 enabled 仍为真，所以这里被拒是**正确**行为。） */
    CHECK_EQ(jsdk_context_configure(fx.ctx), JSDK_ERR_BAD_STATE);

    /* --- 使能中禁止下载描述符（硬约束 1） --- */
    {
        jsdk_joint_request_enable(fx.j, JSDK_MODE_MIT);
        for (i = 0u; i < 20u && !jsdk_joint_is_enabled(fx.j); ++i) fx_cycle(&fx);
        CHECK_EQ(jsdk_joint_is_enabled(fx.j), 1);
        CHECK_EQ(jsdk_context_desc_fetch(fx.ctx), JSDK_ERR_BAD_STATE);
        CHECK(strstr(jsdk_context_last_error(fx.ctx), "enabled") != NULL);

        /* --- 使能中禁止发现（会与响应争用） --- */
        {
            uint8_t ids[8]; unsigned found = 99u;
            CHECK_EQ(jsdk_context_discover(fx.ctx, ids, 8u, &found, 4u),
                     JSDK_ERR_BAD_STATE);
            CHECK_EQ(found, 0u);
        }
        printf("      desc_fetch / discover refused while enabled\n");

        jsdk_joint_request_disable(fx.j);
        for (i = 0u; i < 20u && jsdk_joint_is_enabled(fx.j); ++i) fx_cycle(&fx);
        CHECK_EQ(jsdk_joint_is_enabled(fx.j), 0);
    }

    /* --- 标定失效 → 物理量 API 必须拒绝发帧（宁可不动作，不可用错量程） --- */
    {
        fix_t gx;
        unsigned j;

        if (fx_up(&gx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                       "kpmax=500,kdmax=5,hb=10,fd") != 0) {
            printf("      FATAL: fixture setup failed\n");
            g_fail++; g_checks++;
        } else {
            jsdk_can_frame_t f;
            unsigned ctrl_before;

            jsdk_joint_request_enable(gx.j, JSDK_MODE_MIT);
            for (j = 0u; j < 20u && !jsdk_joint_is_enabled(gx.j); ++j) fx_cycle(&gx);
            CHECK_EQ(jsdk_joint_is_enabled(gx.j), 1);

            /* 人为把标定作废（模拟“标定参数读不出/不合理”） */
            gx.j->calibrated = 0u;
            drain_tx(&gx);
            ctrl_before = gx.j->tx_frames;
            for (j = 0u; j < 5u; ++j) fx_cycle(&gx);
            CHECK_EQ(gx.j->tx_frames, ctrl_before);        /* 一帧都没发 */
            CHECK_EQ(saw_msgtype(&gx, CB_MSG_MIT_CONTROL, &f), 0);
            printf("      calibrated=0 -> zero control frames emitted\n");
            fx_down(&gx);
        }
    }

    /* --- FROM_CACHE 模式但没给描述符 → configure 必须明确报错，而不是自己下载 --- */
    {
        fix_t gx;
        if (fx_up(&gx, "0:id=1,fd") != 0) {
            printf("      FATAL: fixture setup failed\n");
            g_fail++; g_checks++;
        } else {
            jsdk_desc_info_t di;

            /* --- 描述符已在手 → 重复 configure 不应重新下载 --- */
            {
                unsigned tx_before = gx.sim->tx_frames;
                CHECK_EQ(jsdk_context_configure(gx.ctx), JSDK_OK);
                /* 只该出现握手 + 标定读，不该有 0x24 请求（共 663 帧） */
                CHECK(gx.sim->tx_frames > tx_before);
                CHECK(gx.sim->tx_frames - tx_before < 100u);
            }

            /* --- 缓存字节格式不对 → 报错，但**不得**动已就位的端点表 --- */
            CHECK(jsdk_context_desc_import(gx.ctx, "\x00\x01\x02\x03", 4u) != JSDK_OK);
            CHECK_EQ(jsdk_context_get_desc_info(gx.ctx, &di), JSDK_OK);  /* 仍可用 */
            CHECK_EQ(di.endpoint_count, 594u);

            /* arena 太小 → NO_MEMORY（而不是静默截断） */
            {
                static uint8_t tiny[64];
                jsdk_desc_config_t saved = gx.ctx->cfg.desc;
                gx.ctx->cfg.desc.arena = tiny;
                gx.ctx->cfg.desc.arena_size = sizeof tiny;
                CHECK_EQ(jsdk_context_desc_import_raw(gx.ctx, g_json, g_json_len, NULL),
                         JSDK_ERR_INVALID_ARG);   /* hint 必需 */
                {
                    jsdk_desc_hint_t hint = { 0x1234u, 0u };
                    CHECK_EQ(jsdk_context_desc_import_raw(gx.ctx, g_json, g_json_len, &hint),
                             JSDK_ERR_NO_MEMORY);
                }
                gx.ctx->cfg.desc = saved;
            }
            printf("      bad cache / tiny arena -> protocol / no-memory (no partial state)\n");
            fx_down(&gx);
        }
    }

    fx_down(&fx);
}

/* ==========================================================================
 * 8. 路线 A 缓存（上下文级）
 * ======================================================================== */

static void test_cache_roundtrip(void)
{
    fix_t fx;
    static uint8_t cache[32768];
    size_t clen = 0u;

    printf("[8] descriptor cache via context API\n");

    if (fx_up(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                   "kpmax=500,kdmax=5,hb=10,fd") != 0) {
        printf("      FATAL: fixture setup failed\n");
        g_fail++; g_checks++;
        return;
    }

    CHECK_EQ(jsdk_context_desc_export(fx.ctx, cache, sizeof cache, &clen), JSDK_OK);
    CHECK(clen > 0u && clen < sizeof cache);
    printf("      export: %u B for %u endpoints\n",
           (unsigned)clen, (unsigned)jsdk_ep_store_count(&fx.ctx->store));

    /* 导出的字节里必须能看出 crc / fw —— 它们是路线 A 的两个失效键 */
    {
        cb_desc_cache_meta_t m;
        CHECK_EQ(cb_desc_cache_peek(cache, clen, &m), JSDK_OK);
        CHECK_EQ(m.endpoint_count, 594u);
        CHECK_EQ(cb_desc_cache_set_desc_crc(cache, clen, 0x1234u), JSDK_OK);
        CHECK_EQ(cb_desc_cache_peek(cache, clen, &m), JSDK_OK);
    }

    /* 导入到同一个上下文（先清掉现有描述符状态） */
    fx.ctx->desc_present = 0u;
    CHECK_EQ(jsdk_context_desc_import(fx.ctx, cache, clen), JSDK_OK);
    {
        jsdk_desc_info_t di;
        CHECK_EQ(jsdk_context_get_desc_info(fx.ctx, &di), JSDK_OK);
        CHECK_EQ(di.endpoint_count, 594u);
        CHECK_EQ(jsdk_context_configure(fx.ctx), JSDK_OK);   /* 描述符已在手 → 只握手+标定 */
        printf("      import + configure: %u endpoints, config ok\n", di.endpoint_count);
    }

    /* 容量不足 → NO_MEMORY（调用方扩容重试），且不留下部分结果 */
    fx.ctx->desc_present = 0u;
    CHECK_EQ(jsdk_context_desc_export(fx.ctx, cache, clen - 1u, &clen),
             JSDK_ERR_BAD_STATE);      /* desc_present 已清 → 调用被拒 */
    fx.ctx->desc_present = 1u;
    {
        size_t small = 0u;
        CHECK_EQ(jsdk_context_desc_export(fx.ctx, cache, 16u, &small),
                 JSDK_ERR_NO_MEMORY);
        CHECK_EQ(small, 0u);
    }

    fx_down(&fx);
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("=== joint layer (WP3) tests ===\n\n");

    if (load_fixture() != 0) {
        printf("FATAL: cannot read %s\n", JSDK_TEST_DATA_DIR "/endpoints_v8.json");
        return 1;
    }
    printf("fixture: %u bytes\n\n", (unsigned)g_json_len);

    test_units();              printf("\n");
    test_lifecycle();          printf("\n");
    test_configure();          printf("\n");
    test_enable_modes_disable(); printf("\n");
    test_target_rejection();   printf("\n");
    test_watchdog();           printf("\n");
    test_cache_roundtrip();    printf("\n");

    free(g_json);

    printf("=== %u checks, %u failures ===\n", g_checks, g_fail);
    return g_fail != 0u ? 1 : 0;
}
