/**
 * @file    test_group.c
 * @brief   WP6：分组广播同步 + 多上下文（多 CAN 口）
 *
 * @par “字节级对拍”是本文件的核心
 *  广播帧最容易出的错不是“发不出去”，而是**槽位排布错了** —— 设备收到帧、
 *  长度也够，但从错误的偏移读出别人的目标。因此这里既断言接收侧效果
 *  （每个虚拟节点解出的目标等于自己那一份），也用 `cb_mit_pack_command()`
 *  手工拼一份期望载荷做 `memcmp`。
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
 * 夹具（与 test_ops 同构：now_ms 自行推进仿真时钟）
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
    jsdk_can_hal_t     inner;
    jsdk_hal_handle_t *h;
    sim_bus_t         *sim;
    jsdk_can_hal_t     hal;

    jsdk_context_storage_t  store;
    jsdk_context_t         *ctx;
    jsdk_joint_config_t     jc[JSDK_MAX_JOINTS];
    jsdk_context_config_t   cfg;
} fix_t;

static int w_send(void *u, const jsdk_can_frame_t *f)
{
    fix_t *fx = (fix_t *)u;
    return fx->inner.send(fx->inner.user, f);
}
static int w_recv(void *u, jsdk_can_frame_t *f)
{
    fix_t *fx = (fix_t *)u;
    return fx->inner.recv(fx->inner.user, f);
}
static uint32_t w_now(void *u)
{
    fix_t *fx = (fix_t *)u;
    jsdk_hal_virtual_advance_ms(fx->h, 1u);
    return fx->inner.now_ms(fx->inner.user);
}

/** 开一条总线 + 上下文 + n 个关节（node_id = first_id .. first_id+n-1）。 */
static int fx_open_ex(fix_t *fx, const char *spec, unsigned n_joints,
                      uint8_t master_id, int fd, uint8_t first_id)
{
    static uint8_t arena_a[32768];
    static uint8_t arena_b[32768];
    static unsigned which;
    uint8_t *arena = (which++ % 2u == 0u) ? arena_a : arena_b;
    unsigned i;

    memset(fx, 0, sizeof *fx);

    if (jsdk_hal_virtual_open(&fx->inner, &fx->h, spec) != JSDK_OK) return -1;
    fx->sim = jsdk_hal_virtual_sim(fx->h);
    if (!fx->sim) return -1;
    if (sim_set_desc(fx->sim, g_json, (uint32_t)g_json_len, 0x1234u) != 0) return -1;

    fx->hal = fx->inner;
    fx->hal.user   = fx;
    fx->hal.send   = w_send;
    fx->hal.recv   = w_recv;
    fx->hal.now_ms = w_now;

    jsdk_context_config_default(&fx->cfg);
    fx->cfg.hal = fx->hal;
    fx->cfg.master_id = master_id;
    fx->cfg.is_fd = (uint8_t)(fd ? 1 : 0);
    fx->cfg.period_ns = 1000000u;
    fx->cfg.desc.retain = JSDK_DESC_RETAIN_ALL;
    fx->cfg.desc.arena = arena;
    fx->cfg.desc.arena_size = 32768u;
    fx->cfg.desc.timeout_ms = 5000u;

    if (jsdk_context_init((jsdk_context_t *)&fx->store, &fx->cfg) != JSDK_OK) return -1;
    fx->ctx = (jsdk_context_t *)&fx->store;

    for (i = 0u; i < n_joints; ++i) {
        memset(&fx->jc[i], 0, sizeof fx->jc[i]);
        fx->jc[i].node_id = (uint8_t)(first_id + i);
        fx->jc[i].initial_mode = JSDK_MODE_MIT;
        if (jsdk_context_add_joint(fx->ctx, &fx->jc[i], NULL) != JSDK_OK) return -1;
    }
    return 0;
}

/** 常规用法：node_id 从 1 开始。 */
static int fx_open(fix_t *fx, const char *spec, unsigned n_joints, uint8_t master_id,
                   int fd)
{
    return fx_open_ex(fx, spec, n_joints, master_id, fd, 1u);
}

static int fx_configure(fix_t *fx)
{
    if (jsdk_context_desc_fetch(fx->ctx) != JSDK_OK) return -1;
    return jsdk_context_configure(fx->ctx) == JSDK_OK ? 0 : -1;
}

/** 使能全部关节（非阻塞序列，靠循环推进）。 */
static int fx_enable_all(fix_t *fx)
{
    unsigned i, guard;

    for (i = 0u; i < fx->ctx->nj; ++i) {
        jsdk_joint_request_enable(&fx->ctx->joints[i],
                                  fx->ctx->joints[i].cfg.initial_mode);
    }
    for (guard = 0u; guard < 40u; ++guard) {
        unsigned n_active = 0u;
        jsdk_context_cycle_begin(fx->ctx, 0u);
        jsdk_context_cycle_end(fx->ctx);
        for (i = 0u; i < fx->ctx->nj; ++i) {
            if (fx->ctx->joints[i].tx_active) n_active++;
        }
        if (n_active == fx->ctx->nj) return 0;
    }
    printf("      enable_all -> %s\n", jsdk_context_last_error(fx->ctx));
    return -1;
}

static void fx_close(fix_t *fx)
{
    if (fx->ctx) jsdk_context_destroy(fx->ctx);
    if (fx->h) jsdk_hal_close(fx->h);
}

/** 取出下一帧（供断言）。 */
static int take_tx(fix_t *fx, jsdk_can_frame_t *f)
{
    return jsdk_hal_virtual_capture(fx->h, f);
}

/**
 * 一次排空发送队列，同时按 MsgType 分成两桶。
 *
 * ⚠ 不能写两次“计数并排空”的调用 —— 第一次就把队列清空了，第二次必然得 0，
 *   于是 `CHECK_EQ(unicast, 0)` 会**假通过**。本项目踩过。
 */
static void count2(fix_t *fx, uint8_t t_a, unsigned *n_a, jsdk_can_frame_t *keep_a,
                   uint8_t t_b, unsigned *n_b)
{
    jsdk_can_frame_t f;
    unsigned a = 0u, b = 0u;

    while (take_tx(fx, &f)) {
        uint8_t mt = (uint8_t)cb_id_msgtype(f.id);
        if (mt == t_a) {
            if (a == 0u && keep_a) *keep_a = f;
            a++;
        } else if (mt == t_b) {
            b++;
        }
    }
    if (n_a) *n_a = a;
    if (n_b) *n_b = b;
}

/** 清空发送队列（不计数）。 */
static void drain_all(fix_t *fx)
{
    jsdk_can_frame_t f;
    while (take_tx(fx, &f)) { }
}

/** 目标按 node_id 装进数组（下标 = node_id - 1）。 */
static void make_targets(jsdk_group_target_t *t, uint8_t first_id, unsigned n,
                         double base_pos)
{
    unsigned i;

    for (i = 0u; i < n; ++i) {
        t[i].node_id   = (uint8_t)(first_id + i);
        t[i].pos_rad   = base_pos + 0.25 * (double)i;
        t[i].vel_rad_s = 0.5 * (double)i;
        t[i].kp        = 20.0 + 5.0 * (double)i;
        t[i].kd        = 0.5 + 0.25 * (double)i;
        t[i].tau_Nm    = 1.0 + 0.5 * (double)i;
    }
}

/* ==========================================================================
 * 1. FD 广播：槽位布局字节级对拍
 * ======================================================================== */

static void test_fd_broadcast(void)
{
    fix_t fx;
    jsdk_group_target_t tg[4];
    jsdk_can_frame_t f;
    uint8_t expect[64];
    unsigned i;

    printf("[1] FD broadcast: slot layout (byte-level)\n");

    if (fx_open(&fx, "0:id=1,gear=16,pmax=12.5,vmax=65,tmax=50,kpmax=500,kdmax=5,"
                     "hb=10,timeout=30000,fd;"
                     "1:id=2,gear=16,pmax=12.5,vmax=65,tmax=50,kpmax=500,kdmax=5,"
                     "hb=10,timeout=30000,fd;"
                     "2:id=3,gear=16,pmax=12.5,vmax=65,tmax=50,kpmax=500,kdmax=5,"
                     "hb=10,timeout=30000,fd;"
                     "3:id=4,gear=16,pmax=12.5,vmax=65,tmax=50,kpmax=500,kdmax=5,"
                     "hb=10,timeout=30000,fd", 4u, 1u, 1) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    if (fx_configure(&fx) != 0) {
        printf("      configure -> %s\n", jsdk_context_last_error(fx.ctx));
        g_fail++; g_checks++; fx_close(&fx); return;
    }
    if (fx_enable_all(&fx) != 0) { g_fail++; g_checks++; fx_close(&fx); return; }

    make_targets(tg, 1u, 4u, 1.0);
    drain_all(&fx);

    /* 在**同一个周期**里下组指令：cycle_begin → group → cycle_end */
    jsdk_context_cycle_begin(fx.ctx, 0u);
    CHECK_EQ(jsdk_group_set_mit(fx.ctx, tg, 4u), JSDK_OK);
    jsdk_context_cycle_end(fx.ctx);
    printf("      %s\n", jsdk_context_last_error(fx.ctx));

    /* --- 只应有一条 MIT 相关帧，且是广播 --- */
    {
        unsigned n_bcast = 0u, n_uni = 0u;
        count2(&fx, CB_MSG_MIT_CONTROL_BCAST, &n_bcast, &f,
               CB_MSG_MIT_CONTROL, &n_uni);
        CHECK_EQ(n_bcast, 1u);
        CHECK_EQ(n_uni, 0u);      /* 广播已代表这些关节的本周期指令 */
    }

    /* --- 帧头 --- */
    CHECK_EQ(cb_id_msgtype(f.id), CB_MSG_MIT_CONTROL_BCAST);
    CHECK_EQ(cb_id_dest(f.id), (0x1u << 1) | (0x1u << 2) | (0x1u << 3) | (0x1u << 4));
    CHECK_EQ(cb_id_source(f.id), 1u);
    CHECK_EQ(cb_id_priority(f.id), CB_PRI_HIGH_CTRL);
    CHECK_EQ(f.flags & JSDK_FRAME_FD, (uint8_t)JSDK_FRAME_FD);
    /* 帧长 = (最大 node_id + 1) × 8 = 5 × 8 = 40；**槽位 0 属于不存在的设备 0** */
    CHECK_EQ(f.len, 40u);

    /* --- 槽位 0 必须是**显式安全指令**（设备 0 不存在，但不能留脏数据） ---
       ⚠ 这里刻意断言“不是全零”与“解出来是 0 目标/0 增益”两件事：
          全零字节解出来是 pos = -12.5 rad、vel = -65 rad/s、tau = -50 N·m ——
          一次飞车指令。 */
    {
        cb_mit_range_t r0;
        float pos = 9, vel = 9, kp = 9, kd = 9, tau = 9;
        static const uint8_t zeros[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

        CHECK(memcmp(f.data, zeros, 8u) != 0);      /* 绝不能是全零 */

        r0.pos_max = 12.5f; r0.vel_max = 65.0f; r0.kp_max = 500.0f;
        r0.kd_max = 5.0f;   r0.tau_max = 50.0f;
        cb_mit_unpack_command(f.data, &r0, &pos, &vel, &kp, &kd, &tau);
        CHECK_NEAR(pos, 0.0, 12.5 / 32767.0 * 2);
        CHECK_NEAR(vel, 0.0, 65.0 / 4095.0 * 2);
        CHECK_NEAR(kp, 0.0, 500.0 / 4095.0);
        CHECK_NEAR(kd, 0.0, 5.0 / 4095.0);
        CHECK_NEAR(tau, 0.0, 100.0 / 4095.0);
        printf("      unused slot 0 holds an explicit ZERO-gain command "
               "(all-zero bytes would decode to tau = -50 N.m)\n");
    }

    /* --- 逐槽解出：每个槽必须是**它自己**那一份目标 --- */
    for (i = 0u; i < 4u; ++i) {
        jsdk_joint_t  *j = &fx.ctx->joints[i];
        cb_mit_range_t r = j->range;
        float pos = 0, vel = 0, kp = 0, kd = 0, tau = 0;

        cb_mit_unpack_command(&f.data[(size_t)(i + 1u) * CB_MIT_SLOT_BYTES], &r,
                              &pos, &vel, &kp, &kd, &tau);
        CHECK_NEAR(pos, tg[i].pos_rad, 12.5 / 32767.0 * 2);
        CHECK_NEAR(vel, tg[i].vel_rad_s, 65.0 / 4095.0 * 2);
        CHECK_NEAR(kp,  tg[i].kp,     500.0 / 4095.0 * 2);
        CHECK_NEAR(kd,  tg[i].kd,     5.0 / 4095.0 * 2);
        CHECK_NEAR(tau, tg[i].tau_Nm, 100.0 / 4095.0 * 2);
    }

    /* --- 手工拼一份期望载荷做 memcmp（真正意义上的字节级对拍） ---
       ⚠ 槽位 0（不存在的设备 0）也必须是“零目标 + 零增益”的显式指令，
          而不是全零字节 —— 全零解出来是 pos = -12.5 rad、tau = -50 N·m。 */
    memset(expect, 0, sizeof expect);
    cb_mit_pack_command(expect, &fx.ctx->joints[0].range,
                        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, NULL);
    for (i = 0u; i < 4u; ++i) {
        cb_mit_pack_command(&expect[(size_t)(i + 1u) * CB_MIT_SLOT_BYTES],
                            &fx.ctx->joints[i].range,
                            (float)tg[i].pos_rad, (float)tg[i].vel_rad_s,
                            (float)tg[i].kp, (float)tg[i].kd, (float)tg[i].tau_Nm,
                            NULL);
    }
    CHECK(memcmp(expect, f.data, 40u) == 0);
    printf("      40 B frame, dest=0x%02X, slots 1..4 byte-identical to "
           "per-slot pack\n", (unsigned)cb_id_dest(f.id));

    /* --- 设备侧确实执行了各自的目标 --- */
    for (i = 0u; i < 4u; ++i) {
        const sim_node_t *n = &fx.sim->nodes[i];
        CHECK_EQ(n->bcast_seen, 1u);
        CHECK_NEAR(n->cmd_pos_out_rad, tg[i].pos_rad, 0.01);
        CHECK_NEAR(n->cmd_tau_out_nm, tg[i].tau_Nm, 0.1);
        printf("      node %u executed pos=%.3f tau=%.3f (its own slot)\n",
               (unsigned)n->node_id, (double)n->cmd_pos_out_rad,
               (double)n->cmd_tau_out_nm);
    }

    /* --- 组内关节的记账：广播帧同样喂协议级看门狗 --- */
    for (i = 0u; i < 4u; ++i) {
        CHECK(fx.ctx->joints[i].last_ctrl_tx_ms != 0u);
        CHECK(fx.ctx->joints[i].tx_frames > 0u);
    }

    /* --- 稀疏分组：{1,3} → max_slot = 3，帧长 32 B；槽位 0 与 2 都被跳过，
           但都必须是显式零增益，绝不能留全零 --- */
    {
        jsdk_group_target_t sp[2];
        jsdk_can_frame_t    bf;
        cb_mit_range_t      r0 = fx.ctx->joints[0].range;
        static const uint8_t zeros[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
        unsigned n_b = 0u;
        float pos = 9, vel = 9, kp = 9, kd = 9, tau = 9;

        sp[0] = tg[0]; sp[0].node_id = 1u;
        sp[1] = tg[2]; sp[1].node_id = 3u;

        drain_all(&fx);
        jsdk_context_cycle_begin(fx.ctx, 0u);
        CHECK_EQ(jsdk_group_set_mit(fx.ctx, sp, 2u), JSDK_OK);
        jsdk_context_cycle_end(fx.ctx);

        count2(&fx, CB_MSG_MIT_CONTROL_BCAST, &n_b, &bf, CB_MSG_MIT_CONTROL, NULL);
        CHECK_EQ(n_b, 1u);
        CHECK_EQ(bf.len, 32u);
        CHECK_EQ(cb_id_dest(bf.id), (1u << 1) | (1u << 3));

        CHECK(memcmp(bf.data, zeros, 8u) != 0);        /* 槽位 0 */
        CHECK(memcmp(&bf.data[16], zeros, 8u) != 0);   /* 槽位 2 */
        cb_mit_unpack_command(&bf.data[16], &r0, &pos, &vel, &kp, &kd, &tau);
        CHECK_NEAR(pos, 0.0, 12.5 / 32767.0 * 2);
        CHECK_NEAR(tau, 0.0, 100.0 / 4095.0 * 2);

        /* 两个成员各自看到自己的目标 */
        CHECK_NEAR(fx.sim->nodes[0].cmd_pos_out_rad, sp[0].pos_rad, 0.01);
        CHECK_NEAR(fx.sim->nodes[2].cmd_pos_out_rad, sp[1].pos_rad, 0.01);
        printf("      sparse group {1,3}: 32 B frame, skipped slots 0/2 hold "
               "zero-gain (not zero bytes)\n");
    }

    fx_close(&fx);
}

/* ==========================================================================
 * 2. Classic 广播：同目标一条帧，异目标降级单播
 * ======================================================================== */

static void test_classic_broadcast(void)
{
    fix_t fx;
    jsdk_group_target_t tg[4];
    jsdk_can_frame_t f;
    unsigned i;

    printf("[2] Classic broadcast: one shared command, else unicast fallback\n");

    if (fx_open(&fx, "0:id=1,gear=16,pmax=12.5,vmax=65,tmax=50,kpmax=500,kdmax=5,"
                     "hb=10,timeout=30000,classic;"
                     "1:id=2,gear=16,pmax=12.5,vmax=65,tmax=50,kpmax=500,kdmax=5,"
                     "hb=10,timeout=30000,classic;"
                     "2:id=3,gear=16,pmax=12.5,vmax=65,tmax=50,kpmax=500,kdmax=5,"
                     "hb=10,timeout=30000,classic;"
                     "3:id=4,gear=16,pmax=12.5,vmax=65,tmax=50,kpmax=500,kdmax=5,"
                     "hb=10,timeout=30000,classic", 4u, 1u, 0) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    if (fx_configure(&fx) != 0 || fx_enable_all(&fx) != 0) {
        printf("      setup -> %s\n", jsdk_context_last_error(fx.ctx));
        g_fail++; g_checks++; fx_close(&fx); return;
    }
    CHECK_EQ(fx.sim->nodes[0].is_fd, 0u);

    /* --- 全员同目标 → 一条 8 B 广播，槽位 0 --- */
    make_targets(tg, 1u, 4u, 2.0);
    for (i = 1u; i < 4u; ++i) {
        tg[i].pos_rad   = tg[0].pos_rad;
        tg[i].vel_rad_s = tg[0].vel_rad_s;
        tg[i].kp = tg[0].kp; tg[i].kd = tg[0].kd; tg[i].tau_Nm = tg[0].tau_Nm;
    }
    drain_all(&fx);
    jsdk_context_cycle_begin(fx.ctx, 0u);
    CHECK_EQ(jsdk_group_set_mit(fx.ctx, tg, 4u), JSDK_OK);
    jsdk_context_cycle_end(fx.ctx);

    {
        unsigned nb = 0u, nu = 0u;
        count2(&fx, CB_MSG_MIT_CONTROL_BCAST, &nb, &f, CB_MSG_MIT_CONTROL, &nu);
        CHECK_EQ(nb, 1u);
        CHECK_EQ(nu, 0u);
        CHECK_EQ(f.len, 8u);                 /* Classic 恒用槽位 0，长度只需 8 */
        CHECK_EQ(cb_id_msgtype(f.id), CB_MSG_MIT_CONTROL_BCAST);
        CHECK_EQ(f.flags & JSDK_FRAME_FD, 0u);
    }
    for (i = 0u; i < 4u; ++i) {
        CHECK_NEAR(fx.sim->nodes[i].cmd_pos_out_rad, tg[0].pos_rad, 0.01);
    }
    printf("      same target: 1 frame x 8 B, all 4 nodes ran the same command\n");

    /* --- 目标不一致 → 自动降级为逐关节单播，且**不发**广播 --- */
    make_targets(tg, 1u, 4u, 3.0);
    drain_all(&fx);
    jsdk_context_cycle_begin(fx.ctx, 0u);
    CHECK_EQ(jsdk_group_set_mit(fx.ctx, tg, 4u), JSDK_OK);
    jsdk_context_cycle_end(fx.ctx);
    printf("      %s\n", jsdk_context_last_error(fx.ctx));

    {
        unsigned nb = 0u, nu = 0u;
        count2(&fx, CB_MSG_MIT_CONTROL_BCAST, &nb, NULL, CB_MSG_MIT_CONTROL, &nu);
        CHECK_EQ(nb, 0u);
        CHECK_EQ(nu, 4u);       /* 降级后各发一条；cycle_end 不会再补 */
    }
    for (i = 0u; i < 4u; ++i) {
        CHECK_NEAR(fx.sim->nodes[i].cmd_pos_out_rad, tg[i].pos_rad, 0.01);
        printf("      node %u got its own target pos=%.3f (unicast)\n",
               (unsigned)(i + 1u), (double)fx.sim->nodes[i].cmd_pos_out_rad);
    }

    fx_close(&fx);
}

/* ==========================================================================
 * 3. 拒绝路径与告警
 * ======================================================================== */

static void test_group_rejections(void)
{
    fix_t fx;
    jsdk_group_target_t tg[9];
    unsigned i;

    printf("[3] group refusals / warnings\n");

    /* 9 个关节里含 node_id = 8/9 → 位图寻址不了 */
    if (fx_open(&fx, "0:id=1,timeout=30000,fd;1:id=2,timeout=30000,fd;"
                     "2:id=3,timeout=30000,fd;3:id=4,timeout=30000,fd", 4u, 1u, 1) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    if (fx_configure(&fx) != 0 || fx_enable_all(&fx) != 0) {
        g_fail++; g_checks++; fx_close(&fx); return;
    }
    make_targets(tg, 1u, 4u, 0.5);

    /* 未使能的关节不能进组 */
    {
        jsdk_group_target_t one[1];
        one[0] = tg[0];
        one[0].node_id = 4u;
        fx.ctx->joints[3].tx_active = 0u;              /* 人为让它“没使能” */
        CHECK_EQ(jsdk_group_set_mit(fx.ctx, one, 1u), JSDK_ERR_BAD_STATE);
        CHECK(strstr(jsdk_context_last_error(fx.ctx), "not enabled") != NULL);
        fx.ctx->joints[3].tx_active = 1u;
        printf("      joint that was never enabled is refused\n");
    }

    /* node_id ≥ 8 → UNSUPPORTED（位图只有 1..7） */
    {
        jsdk_group_target_t one[1];
        one[0] = tg[0];
        one[0].node_id = 8u;
        CHECK_EQ(jsdk_group_set_mit(fx.ctx, one, 1u), JSDK_ERR_UNSUPPORTED);
        CHECK(strstr(jsdk_context_last_error(fx.ctx), "bitmap") != NULL);
        printf("      node_id 8 refused: %s\n", jsdk_context_last_error(fx.ctx));
    }

    /* 组员不在本上下文 */
    {
        jsdk_group_target_t one[1];
        one[0] = tg[0];
        one[0].node_id = 7u;
        CHECK_EQ(jsdk_group_set_mit(fx.ctx, one, 1u), JSDK_ERR_NOT_FOUND);
    }

    /* 关节当前不是 MIT 模式 → 拒绝。否则一条 MIT 广播会把设备
       悄悄切到 MIT 输入模式，而客户以为它还在跑 CSP。 */
    {
        jsdk_group_target_t one[1];
        one[0] = tg[0];
        one[0].node_id = 1u;
        jsdk_joint_set_mode(&fx.ctx->joints[0], JSDK_MODE_CSP);
        CHECK_EQ(jsdk_group_set_mit(fx.ctx, one, 1u), JSDK_ERR_BAD_STATE);
        CHECK(strstr(jsdk_context_last_error(fx.ctx), "only drives MIT") != NULL);
        printf("      non-MIT member refused: %s\n", jsdk_context_last_error(fx.ctx));
        jsdk_joint_set_mode(&fx.ctx->joints[0], JSDK_MODE_MIT);
    }

    /* 重复 node_id / n=0 / n=8 */
    {
        jsdk_group_target_t dup[2];
        dup[0] = tg[0]; dup[0].node_id = 2u;
        dup[1] = tg[1]; dup[1].node_id = 2u;
        CHECK_EQ(jsdk_group_set_mit(fx.ctx, dup, 2u), JSDK_ERR_INVALID_ARG);
        CHECK_EQ(jsdk_group_set_mit(fx.ctx, tg, 0u), JSDK_ERR_INVALID_ARG);

        for (i = 0u; i < 8u; ++i) { tg[i] = tg[0]; tg[i].node_id = (uint8_t)(i + 1u); }
        CHECK_EQ(jsdk_group_set_mit(fx.ctx, tg, 8u), JSDK_ERR_UNSUPPORTED);
        printf("      duplicate / empty / oversized groups refused\n");
    }

    /* 组内目标越界 → 降级单播，走 §6.10 策略（发安全帧，不静默钳位） */
    {
        jsdk_group_target_t bad[2];
        unsigned rejected_before = fx.ctx->joints[0].tx_rejected;

        bad[0] = tg[0]; bad[0].node_id = 1u; bad[0].pos_rad = 999.0;
        bad[1] = tg[1]; bad[1].node_id = 2u;
        drain_all(&fx);
        jsdk_context_cycle_begin(fx.ctx, 0u);
        CHECK_EQ(jsdk_group_set_mit(fx.ctx, bad, 2u), JSDK_OK);
        jsdk_context_cycle_end(fx.ctx);
        {
            unsigned nb = 0u;
            count2(&fx, CB_MSG_MIT_CONTROL_BCAST, &nb, NULL, CB_MSG_MIT_CONTROL, NULL);
            CHECK_EQ(nb, 0u);
        }
        CHECK(fx.ctx->joints[0].tx_rejected > rejected_before);
        CHECK(fabs((double)fx.sim->nodes[0].cmd_pos_out_rad) <= 12.5 + 0.01);
        printf("      out-of-range member: degraded to unicast + safe frame "
               "(rejected=%u)\n", (unsigned)fx.ctx->joints[0].tx_rejected);

        /* 同一周期内重复命令同一个关节 → 报"本周期已发过"，
           而不是含糊的 TRANSPORT（客户不会为此去查 CAN 线），且不发出任何帧 */
        {
            uint32_t tx_before = fx.ctx->bus.tx_frames;
            CHECK_EQ(jsdk_group_set_mit(fx.ctx, bad, 2u), JSDK_ERR_BAD_STATE);
            CHECK(strstr(jsdk_context_last_error(fx.ctx), "already sent") != NULL);
            CHECK_EQ(fx.ctx->bus.tx_frames, tx_before);
            printf("      second command in the same cycle refused: %s\n",
                   jsdk_context_last_error(fx.ctx));
        }
    }

    /* --- >4 字节的参数写：FD 一次写完，Classic 必须分段 --- */
    {
        jsdk_value_t v;
        uint64_t want = 0x0123456789ABCDEFull;

        memset(&v, 0, sizeof v);
        v.type = JSDK_EP_U64;
        v.v.u64 = want;

        /* FD：4 + 8 = 12 ≤ 64，一帧即可 */
        CHECK_EQ(jsdk_joint_param_set(&fx.ctx->joints[0], "axis0.motor.error", &v),
                 JSDK_OK);
        CHECK_EQ(fx.sim->nodes[0].error_motor, want);

        /* Classic：同一长度必须分两块（每块 4 B，末块恰好补齐） */
        {
            fix_t gx;
            if (fx_open(&gx, "0:id=1,hb=10,timeout=30000,classic", 1u, 1u, 0) == 0) {
                if (fx_configure(&gx) == 0) {
                    jsdk_value_t w;
                    memset(&w, 0, sizeof w);
                    w.type = JSDK_EP_U64;
                    w.v.u64 = 0x0000000012345678ull;
                    CHECK_EQ(jsdk_joint_param_set(&gx.ctx->joints[0],
                                                  "axis0.motor.error", &w), JSDK_OK);
                    CHECK_EQ(gx.sim->nodes[0].error_motor, 0x0000000012345678ull);
                    printf("      >4 B write: FD = 1 frame, Classic = segmented; "
                           "both land correctly\n");
                } else {
                    printf("  FAIL classic fixture configure\n"); g_fail++; g_checks++;
                }
                fx_close(&gx);
            } else {
                printf("  FAIL classic fixture open\n"); g_fail++; g_checks++;
            }
        }
    }

    fx_close(&fx);

    /* --- configure() 的广播能力告警 --- */
    {
        fix_t gx;
        /* ⚠ 设备的 node_id 是 9，关节的 node_id 也必须是 9（否则握手发给 1 号，
           没有设备应答 → configure 超时）。 */
        if (fx_open_ex(&gx, "0:id=9,timeout=30000,fd", 1u, 1u, 1, 9u) == 0) {
            jsdk_status_t cst = jsdk_context_configure(gx.ctx);
            g_checks++;
            if (cst != JSDK_OK) {
                printf("  FAIL configure() for node_id=9 -> %d (%s)\n", (int)cst,
                       jsdk_context_last_error(gx.ctx));
                g_fail++;
            } else {
                CHECK(strstr(jsdk_context_last_error(gx.ctx), "WARNING") != NULL);
                CHECK(strstr(jsdk_context_last_error(gx.ctx), "bitmap") != NULL);
                printf("      configure() warns for node_id >= 8: %s\n",
                       jsdk_context_last_error(gx.ctx));
            }
            fx_close(&gx);
        } else {
            printf("  FAIL fixture for node_id=9\n");
            g_fail++; g_checks++;
        }
    }
}

/* ==========================================================================
 * 4. group_enable / group_disable
 * ======================================================================== */

static void test_group_enable_disable(void)
{
    fix_t fx;
    uint8_t ids[4] = { 1u, 2u, 3u, 4u };
    unsigned i, guard;

    printf("[4] group enable / disable\n");

    if (fx_open(&fx, "0:id=1,hb=10,timeout=30000,fd;1:id=2,hb=10,timeout=30000,fd;"
                     "2:id=3,hb=10,timeout=30000,fd;3:id=4,hb=10,timeout=30000,fd",
                4u, 1u, 1) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    if (fx_configure(&fx) != 0) { g_fail++; g_checks++; fx_close(&fx); return; }

    CHECK_EQ(jsdk_group_enable(fx.ctx, ids, 4u), JSDK_OK);
    printf("      %s\n", jsdk_context_last_error(fx.ctx));
    for (guard = 0u; guard < 40u; ++guard) {
        unsigned n = 0u;
        jsdk_context_cycle_begin(fx.ctx, 0u);
        jsdk_context_cycle_end(fx.ctx);
        for (i = 0u; i < 4u; ++i) if (fx.ctx->joints[i].tx_active) n++;
        if (n == 4u) break;
    }
    for (i = 0u; i < 4u; ++i) CHECK_EQ(fx.ctx->joints[i].tx_active, 1u);

    /* `tx_active` 在发完安全首帧那一刻就置位，而 `is_enabled()` 要等**设备应答**
       （里面带 ModeState）在下一周期被处理后才为真 —— 所以要多跑一两个周期。 */
    for (guard = 0u; guard < 4u; ++guard) {
        jsdk_context_cycle_begin(fx.ctx, 0u);
        jsdk_context_cycle_end(fx.ctx);
    }
    for (i = 0u; i < 4u; ++i) {
        CHECK_EQ(jsdk_joint_is_enabled(&fx.ctx->joints[i]), 1);
    }
    printf("      all 4 joints enabled (tx_active + device-reported state)\n");

    CHECK_EQ(jsdk_group_disable(fx.ctx, ids, 4u), JSDK_OK);
    for (guard = 0u; guard < 40u; ++guard) {
        unsigned n = 0u;
        jsdk_context_cycle_begin(fx.ctx, 0u);
        jsdk_context_cycle_end(fx.ctx);
        for (i = 0u; i < 4u; ++i) if (!fx.ctx->joints[i].tx_active) n++;
        if (n == 4u) break;
    }
    for (i = 0u; i < 4u; ++i) CHECK_EQ(fx.ctx->joints[i].tx_active, 0u);
    printf("      all 4 joints disabled\n");

    /* 未使能后再下组指令 → 拒绝 */
    {
        jsdk_group_target_t t[1];
        make_targets(t, 1u, 1u, 0.0);
        CHECK_EQ(jsdk_group_set_mit(fx.ctx, t, 1u), JSDK_ERR_BAD_STATE);
    }
    /* 未知节点 */
    {
        uint8_t bad[1] = { 9u };
        CHECK_EQ(jsdk_group_enable(fx.ctx, bad, 1u), JSDK_ERR_NOT_FOUND);
    }

    fx_close(&fx);
}

/* ==========================================================================
 * 5. 多上下文（多 CAN 口）
 * ======================================================================== */

static void test_multi_context(void)
{
    fix_t a, b;
    unsigned i, guard;

    printf("[5] multiple contexts (two CAN ports in one process)\n");

    /* ⚠ 用 **Classic** 两条总线：分段写（>4 B 值）是 Classic 专属机制，
       FD 下一次就能写完 8 B（4 + 8 = 12 ≤ 64），根本不经过装配器。
       要验证“装配器按总线隔离”，必须走 Classic。 */
    if (fx_open(&a, "0:id=1,gear=16,pmax=12.5,vmax=65,tmax=50,kpmax=500,kdmax=5,"
                    "hb=10,timeout=30000,classic", 1u, 1u, 0) != 0 ||
        fx_open(&b, "0:id=1,gear=8,pmax=12.5,vmax=65,tmax=50,kpmax=500,kdmax=5,"
                    "hb=10,timeout=30000,classic", 1u, 7u, 0) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    CHECK(a.h != b.h);
    CHECK(a.sim != b.sim);

    if (fx_configure(&a) != 0 || fx_configure(&b) != 0) {
        printf("      configure -> %s | %s\n", jsdk_context_last_error(a.ctx),
               jsdk_context_last_error(b.ctx));
        g_fail++; g_checks++; fx_close(&a); fx_close(&b); return;
    }
    /* 两个上下文用不同的 master_id；各自的 gear 也不同 */
    CHECK_NEAR(a.ctx->joints[0].gear_ratio, 16.0, 1e-6);
    CHECK_NEAR(b.ctx->joints[0].gear_ratio, 8.0, 1e-6);
    printf("      port A: master=1 gear=16 ; port B: master=7 gear=8\n");

    /* --- 交错跑循环：互不干扰 --- */
    if (fx_enable_all(&a) != 0 || fx_enable_all(&b) != 0) {
        g_fail++; g_checks++; fx_close(&a); fx_close(&b); return;
    }
    for (guard = 0u; guard < 10u; ++guard) {
        jsdk_context_cycle_begin(a.ctx, 0u); jsdk_context_cycle_end(a.ctx);
        jsdk_context_cycle_begin(b.ctx, 0u); jsdk_context_cycle_end(b.ctx);
    }

    /* A 的指令只出现在 A 的总线捕获里 */
    for (i = 0u; i < 2u; ++i) {
        fix_t *fx = (i == 0u) ? &a : &b;
        jsdk_can_frame_t f;
        unsigned n = 0u;
        while (take_tx(fx, &f)) {
            n++;
            CHECK_EQ(cb_id_source(f.id), fx->ctx->cfg.master_id);
        }
        CHECK(n > 0u);
        /* 反馈也只更新自己 */
        CHECK_EQ(fx->ctx->joints[0].fb.online, 1);
    }
    printf("      interleaved cycles: each port only sees its own master_id\n");

    /* --- 分段写：装配器按总线隔离（曾经是文件级 static，会串味） --- */
    {
        /* axis0.motor.error = u64（8 B）→ Classic 分段写需要 2 块 */
        static const char *const path = "axis0.motor.error";
        uint16_t ep_a = 0u, ep_b = 0u;
        uint8_t  v_a[8], v_b[8], req[CB_PARAM_READ_REQ_FULL];
        jsdk_can_frame_t dummy;

        CHECK_EQ(jsdk_endpoint_lookup(a.ctx, path, &ep_a, NULL, NULL), JSDK_OK);
        CHECK_EQ(jsdk_endpoint_lookup(b.ctx, path, &ep_b, NULL, NULL), JSDK_OK);
        CHECK_EQ(ep_a, 193u);
        CHECK_EQ(ep_b, 193u);

        cb_be_put_u64(v_a, 0x1111222233334444ull);
        cb_be_put_u64(v_b, 0xAAAABBBBCCCCDDDDull);

        /* 交错发块：A 的半块 → B 的半块 → A 的整块 → B 的整块。
           若装配器是共享的，两边都会拿到对方的字节。 */
        {
            uint8_t ck[8];
            size_t  n;

            n = cb_param_pack_write_chunk(ck, sizeof ck, ep_a, 8u, 0u, v_a, 4u, 1);
            CHECK(n > 0u);
            CHECK_EQ(jsdk_ctx_send(a.ctx, CB_PRI_CONFIG, CB_MSG_PARAM_WRITE, 1u,
                                   ck, (uint8_t)n), 0);
            n = cb_param_pack_write_chunk(ck, sizeof ck, ep_b, 8u, 0u, v_b, 4u, 1);
            CHECK(n > 0u);
            CHECK_EQ(jsdk_ctx_send(b.ctx, CB_PRI_CONFIG, CB_MSG_PARAM_WRITE, 1u,
                                   ck, (uint8_t)n), 0);

            /* B 先收尾（顺序故意交叉） */
            n = cb_param_pack_write_chunk(ck, sizeof ck, ep_b, 8u, 4u, v_b + 4, 4u, 0);
            CHECK(n > 0u);
            CHECK_EQ(jsdk_ctx_send(b.ctx, CB_PRI_CONFIG, CB_MSG_PARAM_WRITE, 1u,
                                   ck, (uint8_t)n), 0);
            n = cb_param_pack_write_chunk(ck, sizeof ck, ep_a, 8u, 4u, v_a + 4, 4u, 0);
            CHECK(n > 0u);
            CHECK_EQ(jsdk_ctx_send(a.ctx, CB_PRI_CONFIG, CB_MSG_PARAM_WRITE, 1u,
                                   ck, (uint8_t)n), 0);
        }
        (void)dummy;

        /* 各总线自己的装配器都该完成，并且值没有串 */
        CHECK_EQ(a.sim->nodes[0].error_motor, 0x1111222233334444ull);
        CHECK_EQ(b.sim->nodes[0].error_motor, 0xAAAABBBBCCCCDDDDull);

        /* 再各自读回一次（走 PARAM_READ，确认设备侧也隔离） */
        {
            uint8_t buf[8];
            uint8_t len = 0u;

            (void)req;
            CHECK_EQ(jsdk_ctx_read_param(a.ctx, 1u, ep_a, buf, &len, 0u), JSDK_OK);
            CHECK_EQ(len, 4u);          /* Classic 单读只回 4 B */
            CHECK_EQ(cb_be_get_u32(buf), 0x11112222u);
            CHECK_EQ(jsdk_ctx_read_param(b.ctx, 1u, ep_b, buf, &len, 0u), JSDK_OK);
            CHECK_EQ(cb_be_get_u32(buf), 0xAAAABBBBu);
        }
        printf("      interleaved segmented writes: assemblers are per-bus "
               "(no cross-talk)\n");
    }

    fx_close(&a);
    fx_close(&b);
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("=== WP6 tests (group broadcast sync + multiple contexts) ===\n\n");
    if (load_fixture() != 0) {
        printf("FATAL: cannot read fixture\n");
        return 1;
    }
    printf("fixture: %u bytes\n\n", (unsigned)g_json_len);

    test_fd_broadcast();        printf("\n");
    test_classic_broadcast();   printf("\n");
    test_group_rejections();    printf("\n");
    test_group_enable_disable(); printf("\n");
    test_multi_context();       printf("\n");

    free(g_json);
    printf("=== %u checks, %u failures ===\n", g_checks, g_fail);
    return g_fail != 0u ? 1 : 0;
}
