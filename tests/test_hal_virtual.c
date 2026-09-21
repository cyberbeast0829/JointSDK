/**
 * @file    test_hal_virtual.c
 * @brief   WP1 虚拟 HAL + 设备模型回归测试
 *
 * 这些测试是**端到端**的：从 HAL 的 send() 出发，经过设备模型，再从 recv() 收回，
 * 断言的是“模型收到的命令是否被正确解释”与“模型发出的帧是否能被 SDK 正确解码”。
 * 因此它同时约束了协议层（cb_*）与设备模型两侧，比单帧编解码测试强得多。
 *
 * 覆盖：
 *   1. 传输层：捕获、注入、TX 失败注入、丢弃计数、虚拟时钟、参数校验
 *   2. 寻址：单播匹配/不匹配、全局广播到多节点
 *   3. MIT 端到端：单位换算（输出端 ↔ 电机端）、响应解码、Seq 非回显、广播槽位
 *   4. 控制帧：POS/VEL/TORQUE/CURRENT 的换算与应答差异
 *   5. 查询帧：0x41 与 MIT 响应**单位不同**（关键陷阱）
 *   6. 心跳与超时：心跳周期、life 递增、break_timeout 触发 disarm
 *   7. 参数读写：单读 u32/u64（Classic 分段）、写入、批量读与 ERR、只读保护
 *   8. 描述符传输：真实 41029 B JSON 的分块重组逐字节一致 + 元数据帧判别
 *   9. ESTOP
 */

#include "jsdk_hal_builtin.h"
#include "hal_virtual_internal.h"

#include "cb_frame.h"
#include "cb_mit.h"
#include "cb_ctrl.h"
#include "cb_query.h"
#include "cb_heartbeat.h"
#include "cb_param.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

static int g_fail;
static int g_checks;

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

static int feq(float a, float b)
{
    return fabsf(a - b) <= 1e-4f * (fabsf(b) + 1.0f);
}

#define CHECK_FEQ(a, b)                                                        \
    do {                                                                       \
        float _a = (a), _b = (b);                                              \
        g_checks++;                                                            \
        if (!feq(_a, _b)) {                                                    \
            printf("  FAIL %s:%d  %s = %.9g, expected %s = %.9g\n",            \
                   __FILE__, __LINE__, #a, (double)_a, #b, (double)_b);        \
            g_fail++;                                                          \
        }                                                                      \
    } while (0)

/* --------------------------------------------------------------------------
 * 测试脚手架
 * ------------------------------------------------------------------------ */

#define MASTER_ID 1u

typedef struct {
    jsdk_can_hal_t    hal;
    jsdk_hal_handle_t *h;
    sim_bus_t         *sim;
} fix_t;

static void fix_open(fix_t *f, const char *spec)
{
    memset(f, 0, sizeof *f);
    if (jsdk_hal_virtual_open(&f->hal, &f->h, spec) != JSDK_OK) {
        printf("  FATAL: virtual open failed (%s)\n", spec ? spec : "default");
        g_fail++;
        f->sim = NULL;                 /* 防止后续解引用空指针 */
        return;
    }
    f->sim = jsdk_hal_virtual_sim(f->h);
    if (!f->sim) {
        printf("  FATAL: virtual open 未返回模型（%s）\n", spec ? spec : "default");
        g_fail++;
    }
}

/** 关闭（幂等；允许 h 为 NULL）。 */
static void fix_close(fix_t *f)
{
    if (!f) return;
    if (f->h) jsdk_hal_close(f->h);
    f->h = NULL;
    f->sim = NULL;
}

/** 换一套规格重新打开（先关旧句柄，避免泄漏）。 */
static void fix_reopen(fix_t *f, const char *spec)
{
    fix_close(f);
    fix_open(f, spec);
}

/** 通过 HAL 发一帧（目的 node_id、msgtype、载荷） */
static int send_frame(fix_t *f, uint32_t dest, uint8_t msgtype, uint8_t pri,
                      const uint8_t *payload, uint8_t len, int is_fd)
{
    jsdk_can_frame_t fr;
    memset(&fr, 0, sizeof fr);
    fr.id    = cb_make_id(pri, msgtype, (uint8_t)dest, (uint8_t)MASTER_ID, 0u);
    fr.len   = len;
    fr.flags = (uint8_t)(JSDK_FRAME_EXT | (is_fd ? (JSDK_FRAME_FD | JSDK_FRAME_BRS) : 0u));
    if (payload && len) memcpy(fr.data, payload, len);
    return f->hal.send(f->hal.user, &fr);
}

/** 从 HAL 收一帧；无帧返回 0 */
static int recv_frame(fix_t *f, jsdk_can_frame_t *out)
{
    return f->hal.recv(f->hal.user, out);
}

/** 收一帧并断言它是来自 node 的 MIT 响应；@return 1 = 收到 */
static int recv_mit_response(fix_t *f, uint32_t node, jsdk_can_frame_t *out)
{
    if (recv_frame(f, out) != 1) return 0;
    if (cb_id_msgtype(out->id) != CB_MSG_MIT_CONTROL) return 0;
    if (cb_id_source(out->id) != node) return 0;
    if (cb_id_dest(out->id) != MASTER_ID) return 0;
    return 1;
}

static void drain(fix_t *f)
{
    jsdk_can_frame_t x;
    while (recv_frame(f, &x) == 1) { /* 丢弃 */ }
}

/* ==========================================================================
 * 1. 传输层
 * ======================================================================== */

static void test_transport(void)
{
    fix_t f;
    jsdk_can_frame_t fr, got;
    uint8_t payload[4] = { 0xAA, 0xBB, 0xCC, 0xDD };

    printf("[1] transport\n");
    fix_open(&f, "0:fd");

    /* --- 捕获 SDK 发出的帧 --- */
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_QUERY_BUS, CB_PRI_QUERY, NULL, 0u, 1), 0);
    CHECK_EQ(jsdk_hal_virtual_capture(f.h, &got), 1);
    CHECK_EQ(cb_id_msgtype(got.id), CB_MSG_QUERY_BUS);
    CHECK_EQ(cb_id_source(got.id), MASTER_ID);
    CHECK_EQ(cb_id_dest(got.id), 1u);
    CHECK((got.flags & JSDK_FRAME_EXT) != 0u);
    CHECK((got.flags & JSDK_FRAME_FD) != 0u);
    CHECK_EQ(jsdk_hal_virtual_capture(f.h, &got), 0);   /* 队列已空 */

    /* --- 非扩展帧必须被拒 --- */
    memset(&fr, 0, sizeof fr);
    fr.id = 0x123u; fr.len = 1u; fr.flags = 0u;
    CHECK(f.hal.send(f.hal.user, &fr) != 0);
    CHECK_EQ(f.hal.send(f.hal.user, NULL), -1);

    /* --- 时钟（起点是 1 ms，见 hal_virtual.c 的说明） --- */
    {
        uint32_t t0 = f.hal.now_ms(f.hal.user);
        CHECK_EQ(t0, 1u);
        jsdk_hal_virtual_advance_ms(f.h, 100u);
        CHECK_EQ(f.hal.now_ms(f.hal.user) - t0, 100u);
        jsdk_hal_virtual_advance_ms(f.h, 50u);
        CHECK_EQ(f.hal.now_ms(f.hal.user) - t0, 150u);
        /* 单调 */
        CHECK(f.hal.now_ms(f.hal.user) >= t0);
    }

    /* --- 注入 --- */
    drain(&f);                     /* 前面的查询会留下模型应答，先清空 */
    memset(&fr, 0, sizeof fr);
    fr.id = cb_make_id(CB_PRI_QUERY, CB_MSG_QUERY_BUS, 1u, MASTER_ID, 0u);
    fr.len = 4u; fr.flags = JSDK_FRAME_EXT;
    memcpy(fr.data, payload, 4);
    CHECK_EQ(jsdk_hal_virtual_inject(f.h, &fr), JSDK_OK);
    CHECK_EQ(recv_frame(&f, &got), 1);
    CHECK_EQ(got.len, 4u);
    CHECK(memcmp(got.data, payload, 4) == 0);

    /* 注入非法帧必须被拒 */
    fr.flags = 0u;
    CHECK_EQ(jsdk_hal_virtual_inject(f.h, &fr), JSDK_ERR_INVALID_ARG);
    fr.flags = JSDK_FRAME_EXT; fr.len = 65u;
    CHECK_EQ(jsdk_hal_virtual_inject(f.h, &fr), JSDK_ERR_INVALID_ARG);
    CHECK_EQ(jsdk_hal_virtual_inject(NULL, &fr), JSDK_ERR_INVALID_ARG);

    /* --- TX 失败注入 --- */
    drain(&f);
    jsdk_hal_virtual_set_tx_fail(f.h, 1);
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_QUERY_BUS, CB_PRI_QUERY, NULL, 0u, 1), -1);
    CHECK_EQ(jsdk_hal_virtual_capture(f.h, &got), 0);   /* 失败帧不入捕获队列 */
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_QUERY_BUS, CB_PRI_QUERY, NULL, 0u, 1), 0);
    CHECK_EQ(jsdk_hal_virtual_capture(f.h, &got), 1);

    jsdk_hal_virtual_set_tx_fail(f.h, -1);              /* 永久失败 */
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_QUERY_BUS, CB_PRI_QUERY, NULL, 0u, 1), -1);
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_QUERY_BUS, CB_PRI_QUERY, NULL, 0u, 1), -1);
    jsdk_hal_virtual_set_tx_fail(f.h, 0);

    /* --- 错误路径不崩溃 --- */
    CHECK_EQ(jsdk_hal_virtual_capture(NULL, &got), 0);
    CHECK_EQ(jsdk_hal_virtual_dropped(NULL), 0u);
    jsdk_hal_virtual_set_tx_fail(NULL, 1);
    jsdk_hal_virtual_advance_ms(NULL, 10u);
    CHECK_EQ(jsdk_hal_close(NULL), JSDK_OK);

    /* --- open 的健壮性：失败时 *out 必须是 NULL，且要识别各种非法规格 --- */
    {
        jsdk_can_hal_t h2;
        jsdk_hal_handle_t *h2p = (jsdk_hal_handle_t *)0x1234;   /* 脏值 */

        CHECK_EQ(jsdk_hal_virtual_open(NULL, &h2p, NULL), JSDK_ERR_INVALID_ARG);
        CHECK_EQ(jsdk_hal_virtual_open(&h2, NULL, NULL), JSDK_ERR_INVALID_ARG);

        /* 语法错误（未知键） */
        h2p = (jsdk_hal_handle_t *)0x1234;
        CHECK_EQ(jsdk_hal_virtual_open(&h2, &h2p, "0:nosuchkey=1"), JSDK_ERR_INVALID_ARG);
        CHECK(h2p == NULL);                       /* 必须被置空，不能留脏值 */

        /* 下标越界（只有 1 个节点却引用下标 1） */
        h2p = (jsdk_hal_handle_t *)0x1234;
        CHECK_EQ(jsdk_hal_virtual_open(&h2, &h2p, "1:fd"), JSDK_ERR_INVALID_ARG);
        CHECK(h2p == NULL);

        /* 节点数超上限 → 明确报错而不是静默截断 */
        h2p = (jsdk_hal_handle_t *)0x1234;
        CHECK_EQ(jsdk_hal_virtual_open(&h2, &h2p, "0:fd;1:fd;2:fd;3:fd;4:fd;5:fd"),
                 JSDK_ERR_INVALID_ARG);
        CHECK(h2p == NULL);

        /* 合法规格能开、能关 */
        h2p = NULL;
        CHECK_EQ(jsdk_hal_virtual_open(&h2, &h2p, "0:id=1,gear=8,hb=10,classic"), JSDK_OK);
        CHECK(h2p != NULL);
        CHECK_EQ(h2.user, h2p);
        CHECK(h2.now_ms(h2.user) == 1u);
        CHECK_EQ(jsdk_hal_close(h2p), JSDK_OK);
    }

    fix_close(&f);
}

/* ==========================================================================
 * 2. 寻址
 * ======================================================================== */

static void test_addressing(void)
{
    fix_t f;
    jsdk_can_frame_t out;

    printf("[2] addressing\n");
    fix_open(&f, "0:id=1,fd;1:id=2,fd");
    CHECK_EQ(f.sim ? f.sim->n_nodes : 0u, 2u);

    /* --- 单播到 node 1 → 只有它应答 --- */
    drain(&f);
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_QUERY_BUS, CB_PRI_QUERY, NULL, 0u, 1), 0);
    CHECK_EQ(recv_frame(&f, &out), 1);
    CHECK_EQ(cb_id_source(out.id), 1u);
    CHECK_EQ(recv_frame(&f, &out), 0);

    /* --- 单播到 node 3（不存在）→ 无人应答 --- */
    drain(&f);
    CHECK_EQ(send_frame(&f, 3u, CB_MSG_QUERY_BUS, CB_PRI_QUERY, NULL, 0u, 1), 0);
    CHECK_EQ(recv_frame(&f, &out), 0);

    /* --- ⚠ 关键：dest = 0xFF 只有在 **MsgType ≥ 0x80**（广播类型）时才是全局广播。
           对 0x44 这种点对点类型，dest = 0xFF 谁也不匹配。--- */
    drain(&f);
    CHECK_EQ(send_frame(&f, 0xFFu, CB_MSG_QUERY_BUS, CB_PRI_QUERY, NULL, 0u, 1), 0);
    CHECK_EQ(recv_frame(&f, &out), 0);
    printf("      dest=0xFF with a point-to-point MsgType(0x44) reaches nobody\n");

    /* --- 用真正的广播类型（ESTOP 0xC0）→ 两个节点都收到 --- */
    drain(&f);
    {
        uint32_t before = f.sim->rx_for_me;
        CHECK_EQ(send_frame(&f, 0xFFu, CB_MSG_ESTOP, CB_PRI_CRITICAL, NULL, 0u, 1), 0);
        /* 全局广播（dest=0xFF + 广播类型）被两个节点都判定为“发给本机” */
        CHECK_EQ(f.sim->rx_for_me - before, 2u);
    }

    /* --- 位图广播：只有被选中的节点收到 --- */
    f.sim->nodes[0].estop = 0u;
    f.sim->nodes[0].armed = 1u;
    f.sim->nodes[1].armed = 1u;
    drain(&f);
    /* 位位置 = node_id：bit1 = 设备 1 */
    CHECK_EQ(send_frame(&f, 0x02u, CB_MSG_ESTOP, CB_PRI_CRITICAL, NULL, 0u, 1), 0);
    CHECK_EQ(f.sim->nodes[0].armed, 0u);       /* node 1 被选中 */
    CHECK_EQ(f.sim->nodes[1].armed, 1u);       /* node 2 未被选中 */

    /* --- Dest = 0 的位图 → 谁也不匹配 --- */
    f.sim->nodes[0].armed = 1u;
    f.sim->nodes[1].armed = 1u;
    CHECK_EQ(send_frame(&f, 0x00u, CB_MSG_ESTOP, CB_PRI_CRITICAL, NULL, 0u, 1), 0);
    CHECK_EQ(f.sim->nodes[0].armed, 1u);
    CHECK_EQ(f.sim->nodes[1].armed, 1u);

    /* --- node_id ≥ 8 不可位寻址，但仍是全局广播的接收者 --- */
    f.sim->nodes[0].node_id = 9u;
    f.sim->nodes[0].armed = 1u;
    drain(&f);
    CHECK_EQ(send_frame(&f, 0xFEu, CB_MSG_ESTOP, CB_PRI_CRITICAL, NULL, 0u, 1), 0);
    CHECK_EQ(f.sim->nodes[0].armed, 1u);       /* 位图路径：不匹配 */
    drain(&f);
    CHECK_EQ(send_frame(&f, 0xFFu, CB_MSG_ESTOP, CB_PRI_CRITICAL, NULL, 0u, 1), 0);
    CHECK_EQ(f.sim->nodes[0].armed, 0u);       /* 全局广播路径：匹配 */

    /* --- node_id = 0 的节点（禁用）完全不应答 --- */
    f.sim->nodes[0].node_id = 0u;
    f.sim->nodes[1].armed = 1u;
    drain(&f);
    CHECK_EQ(send_frame(&f, 0xFFu, CB_MSG_ESTOP, CB_PRI_CRITICAL, NULL, 0u, 1), 0);
    CHECK_EQ(f.sim->nodes[1].armed, 0u);

    fix_close(&f);
}


/* ==========================================================================
 * 3. MIT 端到端
 * ======================================================================== */

static void test_mit(void)
{
    fix_t f;
    jsdk_can_frame_t out;
    sim_node_t *n;
    cb_mit_range_t r;
    uint8_t payload[8];
    const float gear = 16.5f;
    const float pos_out = 1.25f;      /* 输出端 rad */
    const float vel_out = -3.5f;
    const float tau_out = 12.0f;
    /* 12/16 位定点的量化步长（期望值必须按**量化后**的值比） */
    float e_pos, e_vel, e_kp, e_kd, e_tau;

    printf("[3] MIT end-to-end\n");
    fix_open(&f, "0:id=1,gear=16.5,fd");
    n = f.sim ? sim_find_node(f.sim, 1u) : NULL;
    CHECK(n != NULL);
    if (!n) { fix_close(&f); return; }

    r.pos_max = n->mit_max_pos; r.vel_max = n->mit_max_vel;
    r.kp_max  = n->mit_max_kp;  r.kd_max  = n->mit_max_kd;
    r.tau_max = n->mit_max_torque;

    cb_mit_pack_command(payload, &r, pos_out, vel_out, 100.0f, 1.0f, tau_out, NULL);
    /* 期望值 = 把载荷解回来（量化后的值），而不是请求值 */
    cb_mit_unpack_command(payload, &r, &e_pos, &e_vel, &e_kp, &e_kd, &e_tau);

    /* 先使能，否则模型会把速度清零（未使能的轴不该有速度） */
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_START_MOTOR, CB_PRI_CTRL, NULL, 0u, 1), 0);
    CHECK_EQ(n->armed, 1u);

    drain(&f);
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_MIT_CONTROL, CB_PRI_HIGH_CTRL,
                        payload, 8u, 1), 0);

    /* --- 单位换算必须与固件一致（用量化后的值与 1e-5 级容差）--- */
    CHECK_FEQ(n->cmd_pos_out_rad, e_pos);
    CHECK_FEQ(n->cmd_vel_out_rad_s, e_vel);
    CHECK_FEQ(n->cmd_tau_out_nm, e_tau);
    CHECK_FEQ(n->pos_target_motor, e_pos * gear / (2.0f * (float)M_PI));
    CHECK_FEQ(n->vel_target_motor, e_vel * gear / (2.0f * (float)M_PI));
    CHECK_FEQ(n->cmd_torque_motor_nm, e_tau / gear);   /* ⚠ 力矩要 ÷gear */
    CHECK_EQ(n->control_mode, SIM_CM_POSITION);
    CHECK_EQ(n->input_mode, SIM_INPUT_MODE_MIT);
    CHECK((n->error_axis & SIM_ERR_CAN_BUS_FAILED) == 0u);

    /* --- 响应能被 SDK 解码，且 pos 回到**输出端 rad** --- */
    {
        cb_mit_response_t resp;
        float max_cur = cb_mit_response_max_current(n->mit_max_torque,
                                                    n->torque_constant);
        float lsb16 = 2.0f * n->mit_max_pos / 65535.0f;

        CHECK_EQ(recv_mit_response(&f, 1u, &out), 1);
        CHECK_EQ(out.len, 8u);
        cb_mit_unpack_response(out.data, &r, max_cur, &resp);
        CHECK_EQ(resp.mode, CB_MODE_MIT);          /* 已使能 + input_mode = MIT */
        CHECK_EQ(resp.motor_temp_c, (int16_t)n->motor_temp);
        /* 响应里的位置是**输出端** rad */
        CHECK(fabsf(resp.pos - n->pos_estimate * 2.0f * (float)M_PI / gear)
              <= lsb16 + 1e-6f);
        CHECK((out.flags & JSDK_FRAME_FD) != 0u);
    }

    /* --- ⚠ 响应 Seq **不回显**请求 Seq，而是设备本地滚动计数器 --- */
    {
        jsdk_can_frame_t fr;
        uint8_t seqs[5];
        unsigned i;

        for (i = 0u; i < 5u; ++i) {
            memset(&fr, 0, sizeof fr);
            fr.id = cb_make_id(CB_PRI_HIGH_CTRL, CB_MSG_MIT_CONTROL, 1u,
                               MASTER_ID, 3u);      /* 故意用请求 seq = 3 */
            fr.len = 8u;
            fr.flags = (uint8_t)(JSDK_FRAME_EXT | JSDK_FRAME_FD | JSDK_FRAME_BRS);
            memcpy(fr.data, payload, 8);
            drain(&f);
            CHECK_EQ(f.hal.send(f.hal.user, &fr), 0);
            CHECK_EQ(recv_mit_response(&f, 1u, &out), 1);
            seqs[i] = (uint8_t)cb_id_seq(out.id);
        }
        /* 性质 1：每次 +1（模 4）——是滚动计数器 */
        for (i = 1u; i < 5u; ++i) {
            CHECK(seqs[i] == (uint8_t)((seqs[i - 1u] + 1u) & 0x03u));
        }
        /* 性质 2：与请求的 seq(3) 无对应关系：5 次里只可能出现一次 3 */
        {
            unsigned hit3 = 0u;
            for (i = 0u; i < 5u; ++i) if (seqs[i] == 3u) hit3++;
            CHECK_EQ(hit3, 1u);          /* 只是计数器恰好路过 3，不是回显 */
        }
        printf("      response seq rolls %u,%u,%u,%u,%u (request seq was 3) "
               "→ cannot correlate by Seq\n",
               seqs[0], seqs[1], seqs[2], seqs[3], seqs[4]);
    }

    /* --- 广播 MIT（FD）：槽位 = node_id --- */
    {
        uint8_t bcast[64];
        uint8_t t0 = 0xEEu;

        memset(bcast, 0, sizeof bcast);
        /* 槽位 0 放“哨兵”命令，槽位 1 才是 node 1 的命令 */
        cb_mit_pack_command(&bcast[0], &r, 5.0f, 0.0f, 0.0f, 0.0f, 0.0f, &t0);
        cb_mit_pack_command(&bcast[8], &r, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, NULL);

        drain(&f);
        CHECK_EQ(send_frame(&f, 0xFFu, CB_MSG_MIT_CONTROL_BCAST, CB_PRI_HIGH_CTRL,
                            bcast, 64u, 1), 0);
        CHECK_FEQ(n->cmd_pos_out_rad, 0.5f);      /* 读的是槽位 1，不是槽位 0 */
        CHECK_EQ(n->bcast_seen, 1u);
        CHECK_EQ(recv_frame(&f, &out), 0);        /* 广播不应答 */

        /* 帧长不足 (slot+1)*8 → 整帧丢弃，状态不变 */
        drain(&f);
        cb_mit_pack_command(&bcast[0], &r, 9.0f, 0.0f, 0.0f, 0.0f, 0.0f, NULL);
        CHECK_EQ(send_frame(&f, 0xFFu, CB_MSG_MIT_CONTROL_BCAST, CB_PRI_HIGH_CTRL,
                            bcast, 8u, 1), 0);    /* 只有槽位 0，node 1 读不到 */
        CHECK_FEQ(n->cmd_pos_out_rad, 0.5f);      /* 未被改写 */
        CHECK(f.sim->bad_len_drops >= 1u);
    }

    /* --- Classic 广播：槽位恒为 0，所有被掩码命中的设备执行同一条 --- */
    {
        uint8_t c[8];
        sim_node_t *n2;

        fix_reopen(&f, "0:id=1,gear=16.5,classic;1:id=2,gear=8,classic");
        n  = f.sim ? sim_find_node(f.sim, 1u) : NULL;
        n2 = f.sim ? sim_find_node(f.sim, 2u) : NULL;
        CHECK(n != NULL && n2 != NULL);
        if (n && n2) {
            r.pos_max = n->mit_max_pos; r.vel_max = n->mit_max_vel;
            r.kp_max  = n->mit_max_kp;  r.kd_max  = n->mit_max_kd;
            r.tau_max = n->mit_max_torque;
            cb_mit_pack_command(c, &r, 0.75f, 0.0f, 0.0f, 0.0f, 0.0f, NULL);

            /* node 2 的槽位号是 2，Classic 下读不到它 → 只发 8 字节即可 */
            drain(&f);
            CHECK_EQ(send_frame(&f, 0xFFu, CB_MSG_MIT_CONTROL_BCAST,
                                CB_PRI_HIGH_CTRL, c, 8u, 0), 0);
            CHECK_FEQ(n->cmd_pos_out_rad, 0.75f);
            CHECK_FEQ(n2->cmd_pos_out_rad, 0.75f);   /* 同一个值 */
            /* 但各自 gear 不同 → 电机端目标不同 */
            CHECK(!feq(n->pos_target_motor, n2->pos_target_motor));
        }
    }

    fix_close(&f);
}


/* ==========================================================================
 * 4. 控制帧
 * ======================================================================== */

static void test_ctrl(void)
{
    fix_t f;
    jsdk_can_frame_t out;
    sim_node_t *n;
    uint8_t payload[12];
    const float gear = 16.5f;

    printf("[4] control frames\n");
    fix_open(&f, "0:id=1,gear=16.5,fd");
    n = sim_find_node(f.sim, 1u);
    CHECK(n != NULL);
    if (!n) { fix_close(&f); return; }

    /* --- POS (FD 12 B)：度 → 电机端 turns；会持久改写 vel_limit / torque_lim --- */
    {
        size_t len = cb_ctrl_pos_pack(payload, 0, 90.0f, 1500.0f, 10.0f, NULL);
        CHECK_EQ(len, 12u);
        drain(&f);
        CHECK_EQ(send_frame(&f, 1u, CB_MSG_POS_CONTROL, CB_PRI_HIGH_CTRL,
                            payload, 12u, 1), 0);
        CHECK_FEQ(n->pos_target_motor, 90.0f / 360.0f * gear);
        CHECK_FEQ(n->vel_limit, 1500.0f / 60.0f * gear);       /* 已被改写 */
        CHECK_FEQ(n->torque_lim, 10.0f * n->torque_constant);  /* 已被改写 */
        CHECK_EQ(n->control_mode, SIM_CM_POSITION);
        CHECK_EQ(n->input_mode, 3u);                           /* POS_FILTER */
        CHECK_EQ(recv_mit_response(&f, 1u, &out), 1);          /* 会应答 */
    }

    /* --- VEL (8 B) --- */
    {
        size_t len = cb_ctrl_vel_pack(payload, 300.0f, 5.0f, NULL);
        CHECK_EQ(len, 8u);
        drain(&f);
        CHECK_EQ(send_frame(&f, 1u, CB_MSG_VEL_CONTROL, CB_PRI_HIGH_CTRL,
                            payload, 8u, 1), 0);
        CHECK_FEQ(n->vel_target_motor, 300.0f / 60.0f * gear);
        CHECK_EQ(n->control_mode, SIM_CM_VELOCITY);
        CHECK_EQ(n->input_mode, 2u);                           /* VEL_RAMP */
        CHECK_EQ(recv_mit_response(&f, 1u, &out), 1);
    }

    /* --- TORQUE (4 B)：⚠ **不做** gear 换算（与 MIT 不同） --- */
    {
        size_t len = cb_ctrl_torque_pack(payload, 7.5f, NULL);
        CHECK_EQ(len, 4u);
        drain(&f);
        CHECK_EQ(send_frame(&f, 1u, CB_MSG_TORQUE_CONTROL, CB_PRI_HIGH_CTRL,
                            payload, 4u, 1), 0);
        CHECK_FEQ(n->cmd_torque_motor_nm, 7.5f);   /* 原值，不 ÷gear（f32 无量化） */
        CHECK_EQ(n->control_mode, SIM_CM_TORQUE);
        CHECK_EQ(recv_mit_response(&f, 1u, &out), 1);

        /* 对照：MIT 的 7.5 输出端 N·m 会变成 7.5/gear */
        {
            cb_mit_range_t r;
            r.pos_max = n->mit_max_pos; r.vel_max = n->mit_max_vel;
            r.kp_max = n->mit_max_kp;   r.kd_max = n->mit_max_kd;
            r.tau_max = n->mit_max_torque;
            float e_pos, e_vel, e_kp, e_kd, e_tau;
            cb_mit_pack_command(payload, &r, 0.0f, 0.0f, 0.0f, 0.0f, 7.5f, NULL);
            cb_mit_unpack_command(payload, &r, &e_pos, &e_vel, &e_kp, &e_kd, &e_tau);
            drain(&f);
            CHECK_EQ(send_frame(&f, 1u, CB_MSG_MIT_CONTROL, CB_PRI_HIGH_CTRL,
                                payload, 8u, 1), 0);
            CHECK_FEQ(n->cmd_tau_out_nm, e_tau);          /* 12 bit 量化后的值 */
            CHECK_FEQ(n->cmd_torque_motor_nm, e_tau / gear);
            CHECK(!feq(n->cmd_torque_motor_nm, 7.5f));    /* 与 TORQUE 帧确实不同 */
        }
    }

    /* --- CURRENT (4 B)：⚠ **不应答** --- */
    {
        size_t len = cb_ctrl_current_pack(payload, 3.0f, NULL);
        CHECK_EQ(len, 4u);
        drain(&f);
        CHECK_EQ(send_frame(&f, 1u, CB_MSG_CURRENT_CONTROL, CB_PRI_HIGH_CTRL,
                            payload, 4u, 1), 0);
        CHECK_FEQ(n->cmd_torque_motor_nm, 3.0f * n->torque_constant);
        CHECK_EQ(recv_frame(&f, &out), 0);        /* 无任何应答 */
    }

    /* --- 短帧被丢弃 --- */
    drain(&f);
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_VEL_CONTROL, CB_PRI_HIGH_CTRL,
                        payload, 4u, 1), 0);
    CHECK(f.sim->bad_len_drops >= 1u);

    fix_close(&f);
}

/* ==========================================================================
 * 5. 查询帧（含 0x41 与 MIT 响应单位不同这一关键陷阱）
 * ======================================================================== */

static void test_query(void)
{
    fix_t f;
    jsdk_can_frame_t out;
    sim_node_t *n;
    cb_mit_range_t r;
    uint8_t mp[8];

    printf("[5] query frames\n");
    fix_open(&f, "0:id=1,gear=16.5,fd");
    n = sim_find_node(f.sim, 1u);
    CHECK(n != NULL);
    if (!n) { fix_close(&f); return; }

    n->pos_estimate = 2.0f;       /* 电机端 turns */
    n->vel_estimate = -4.0f;
    n->iq_measured  = 5.5f;
    n->motor_temp   = 42.0f;
    n->fet_temp     = 51.0f;
    n->vbus_voltage = 48.5f;
    n->ibus         = 1.25f;

    /* --- 0x41：**电机端 turns**（不换算） --- */
    drain(&f);
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_QUERY_POS_VEL, CB_PRI_QUERY, NULL, 0u, 1), 0);
    CHECK_EQ(recv_frame(&f, &out), 1);
    CHECK_EQ(cb_id_msgtype(out.id), CB_MSG_QUERY_POS_VEL);   /* 同 MsgType */
    CHECK_EQ(cb_id_seq(out.id), 0u);
    {
        cb_query_pos_vel_t q;
        CHECK_EQ(cb_query_decode_pos_vel(out.data, out.len, &q), 0);
        CHECK_FEQ(q.pos_turns, 2.0f);              /* 就是电机端 turns */
        CHECK_FEQ(q.vel_turns_per_s, -4.0f);
    }

    /* --- 对照：MIT 响应把同一物理量换算成**输出端 rad** --- */
    r.pos_max = n->mit_max_pos; r.vel_max = n->mit_max_vel;
    r.kp_max = n->mit_max_kp;   r.kd_max = n->mit_max_kd;
    r.tau_max = n->mit_max_torque;
    cb_mit_pack_command(mp, &r, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, NULL);
    drain(&f);
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_MIT_CONTROL, CB_PRI_HIGH_CTRL, mp, 8u, 1), 0);
    CHECK_EQ(recv_mit_response(&f, 1u, &out), 1);
    {
        cb_mit_response_t resp;
        float max_cur = cb_mit_response_max_current(n->mit_max_torque,
                                                    n->torque_constant);
        float lsb16 = 2.0f * r.pos_max / 65535.0f;
        cb_mit_unpack_response(out.data, &r, max_cur, &resp);
        /* 同一位置，两种单位：电机端 2.0 turns = 输出端 2×2π/gear rad。
           16 位量化步长 0.00038 rad，所以按 1 LSB 容差比较。 */
        CHECK(fabsf(resp.pos - 2.0f * 2.0f * (float)M_PI / 16.5f)
              <= lsb16 + 1e-6f);
        CHECK(!feq(resp.pos, 2.0f));
        printf("      same position: 0x41 reports %.4f turns, MIT response reports "
               "%.4f rad\n", 2.0, (double)resp.pos);
    }

    /* --- 0x42 / 0x43 / 0x44 / 0x47 --- */
    drain(&f);
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_QUERY_CURRENT, CB_PRI_QUERY, NULL, 0u, 1), 0);
    CHECK_EQ(recv_frame(&f, &out), 1);
    { cb_query_current_t q; CHECK_EQ(cb_query_decode_current(out.data, out.len, &q), 0);
      CHECK_FEQ(q.iq_a, 5.5f); }

    drain(&f);
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_QUERY_TEMPERATURE, CB_PRI_QUERY, NULL, 0u, 1), 0);
    CHECK_EQ(recv_frame(&f, &out), 1);
    { cb_query_temp_t q; CHECK_EQ(cb_query_decode_temp(out.data, out.len, &q), 0);
      CHECK_FEQ(q.motor_c, 42.0f); CHECK_FEQ(q.fet_c, 51.0f); }

    drain(&f);
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_QUERY_BUS, CB_PRI_QUERY, NULL, 0u, 1), 0);
    CHECK_EQ(recv_frame(&f, &out), 1);
    { cb_query_bus_t q; CHECK_EQ(cb_query_decode_bus(out.data, out.len, &q), 0);
      CHECK_FEQ(q.vbus_v, 48.5f); CHECK_FEQ(q.ibus_a, 1.25f); }

    /* --- 0x45：错误查询（按子系统） --- */
    n->error_motor = 0x00000003u;
    drain(&f);
    {
        uint8_t req[1];
        req[0] = (uint8_t)CB_ET_MOTOR;
        CHECK_EQ(send_frame(&f, 1u, CB_MSG_QUERY_ERROR, CB_PRI_QUERY, req, 1u, 1), 0);
        CHECK_EQ(recv_frame(&f, &out), 1);
        {
            cb_query_error_t e;
            CHECK_EQ(cb_query_decode_error(out.data, out.len, &e), 0);
            CHECK_EQ(e.err_type, CB_ET_MOTOR);
            CHECK_EQ(e.err_value, 0x00000003u);
        }
    }
    n->error_motor = 0u;

    /* --- 0x46：Classic 8 B / FD 16 B --- */
    drain(&f);
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_QUERY_DEVICE_INFO, CB_PRI_QUERY, NULL, 0u, 1), 0);
    CHECK_EQ(recv_frame(&f, &out), 1);
    CHECK_EQ(out.len, 16u);
    { cb_query_device_info_t d; CHECK_EQ(cb_query_decode_device(out.data, out.len, &d), 0);
      CHECK_EQ(d.has_serial, 1);
      CHECK(d.serial == n->serial_number); }

    f.sim->nodes[0].is_fd = 0u;   /* ⚠ 改变后所有上报都走 Classic */
    drain(&f);
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_QUERY_DEVICE_INFO, CB_PRI_QUERY, NULL, 0u, 0), 0);
    CHECK_EQ(recv_frame(&f, &out), 1);
    CHECK_EQ(out.len, 8u);
    CHECK((out.flags & JSDK_FRAME_FD) == 0u);
    { cb_query_device_info_t d; CHECK_EQ(cb_query_decode_device(out.data, out.len, &d), 0);
      CHECK_EQ(d.has_serial, 0); }

    /* --- 0x40：MIT 响应帧 --- */
    f.sim->nodes[0].is_fd = 1u;
    drain(&f);
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_QUERY_STATUS, CB_PRI_QUERY, NULL, 0u, 1), 0);
    CHECK_EQ(recv_mit_response(&f, 1u, &out), 1);

    fix_close(&f);
}

/* ==========================================================================
 * 6. 心跳与超时
 * ======================================================================== */

static void test_heartbeat_timeout(void)
{
    fix_t f;
    jsdk_can_frame_t out;
    sim_node_t *n;

    printf("[6] heartbeat / timeout\n");
    /* hb=1：每毫秒一帧，便于逐 ms 断言 */
    fix_open(&f, "0:id=1,gear=16.5,hb=1,timeout=100,fd");
    n = f.sim ? sim_find_node(f.sim, 1u) : NULL;
    CHECK(n != NULL);
    if (!n) { fix_close(&f); return; }

    n->pos_estimate  = 1.5f;
    n->motor_temp    = 33.0f;
    n->fet_temp      = 44.0f;      /* MOS 温度 */
    n->vbus_voltage  = 48.5f;

    /* --- 心跳内容 --- */
    jsdk_hal_virtual_advance_ms(f.h, 1u);
    CHECK_EQ(recv_frame(&f, &out), 1);
    CHECK_EQ(cb_id_msgtype(out.id), CB_MSG_HEARTBEAT);
    CHECK_EQ(cb_id_source(out.id), 1u);
    CHECK_EQ(cb_id_dest(out.id), MASTER_ID);
    CHECK_EQ(cb_id_seq(out.id), 0u);
    CHECK_EQ(out.len, CB_HB_LEN_FD);
    {
        cb_heartbeat_t hb;
        CHECK_EQ(cb_heartbeat_decode(out.data, out.len, &hb), 0);
        CHECK_EQ(hb.is_fd, 1);
        CHECK_EQ(hb.life, 0u);
        CHECK_EQ(hb.state, SIM_AS_IDLE);          /* 尚未使能 */
        CHECK_EQ(hb.control_mode, SIM_CM_POSITION);
        CHECK_EQ(hb.motor_temp_c, 33);
        CHECK_EQ(hb.mos_temp_c, 44);               /* ⚠ 曾经因漏置 have_mos_temp 恒为 0 */
        CHECK_FEQ(hb.vbus_v, 48.5f);
        CHECK_FEQ(hb.pos_turns, n->pos_estimate);   /* 心跳是**电机端 turns** */
        CHECK_FEQ(hb.vel_turns_per_s, n->vel_estimate);
    }

    /* --- life 每帧 +1（3 bit 回绕） --- */
    {
        unsigned i;
        uint8_t prev = 0u;
        for (i = 1u; i <= 5u; ++i) {
            cb_heartbeat_t hb;
            jsdk_hal_virtual_advance_ms(f.h, 1u);
            CHECK_EQ(recv_frame(&f, &out), 1);
            CHECK_EQ(cb_heartbeat_decode(out.data, out.len, &hb), 0);
            CHECK_EQ(hb.life, (uint8_t)(i & 0x07u));
            CHECK_EQ(cb_heartbeat_life_is_next(prev, hb.life), 1);
            prev = hb.life;
        }
    }

    /* --- 使能后 state 变为 CLOSED_LOOP，且速度被模型保持 --- */
    drain(&f);
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_START_MOTOR, CB_PRI_CTRL, NULL, 0u, 1), 0);
    CHECK_EQ(n->armed, 1u);
    n->control_mode     = SIM_CM_POSITION;
    n->pos_target_motor = n->pos_estimate;      /* 位置误差置 0 */
    n->vel_target_motor = 2.25f;

    jsdk_hal_virtual_advance_ms(f.h, 1u);
    CHECK_EQ(recv_frame(&f, &out), 1);
    {
        cb_heartbeat_t hb;
        CHECK_EQ(cb_heartbeat_decode(out.data, out.len, &hb), 0);
        CHECK_EQ(hb.state, SIM_AS_CLOSED_LOOP);
        CHECK_FEQ(hb.vel_turns_per_s, 2.25f);
    }

    /* --- break_timeout：只要 is_ctrl 帧持续到达就不会超时 --- */
    {
        uint8_t mp[8];
        cb_mit_range_t r;
        r.pos_max = n->mit_max_pos; r.vel_max = n->mit_max_vel;
        r.kp_max = n->mit_max_kp;   r.kd_max = n->mit_max_kd;
        r.tau_max = n->mit_max_torque;
        cb_mit_pack_command(mp, &r, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, NULL);

        for (int i = 0; i < 4; ++i) {
            drain(&f);
            CHECK_EQ(send_frame(&f, 1u, CB_MSG_MIT_CONTROL, CB_PRI_HIGH_CTRL,
                                mp, 8u, 1), 0);
            jsdk_hal_virtual_advance_ms(f.h, 50u);
            CHECK_EQ(n->armed, 1u);
            CHECK((n->error_axis & SIM_ERR_CAN_BUS_FAILED) == 0u);
        }
    }

    /* --- 停止喂 → 101 ms 后置 CAN_BUS_FAILED 并 disarm --- */
    drain(&f);
    jsdk_hal_virtual_advance_ms(f.h, 101u);
    CHECK((n->error_axis & SIM_ERR_CAN_BUS_FAILED) != 0u);
    CHECK_EQ(n->armed, 0u);
    CHECK_EQ(n->last_cmd_ms, 0u);        /* 已防止重复触发 */

    /* 幂等：清了错再推进也不应重新置位（计时器已归零） */
    n->error_axis = 0u;
    jsdk_hal_virtual_advance_ms(f.h, 300u);
    CHECK_EQ(n->error_axis, 0u);

    /* ===== ⚠ 场景 A：**只用 CURRENT 的客户端，安全阀根本不会武装** =====
       固件 `is_ctrl` 不含 0x04，所以 last_cmd_time_ 永远为 0，
       而 auto_stop_if_timeout() 开头就是 `if (== 0) return;`
       → CAN 掉线也不会自动停机。这是安全缺口，不是“会被停机”。 */
    {
        uint8_t cur[4];
        fix_reopen(&f, "0:id=1,gear=16.5,fd");      /* 全新节点，从未收到 is_ctrl */
        n = f.sim ? sim_find_node(f.sim, 1u) : NULL;
        CHECK(n != NULL);
        if (n) {
            cb_ctrl_current_pack(cur, 2.0f, NULL);
            CHECK_EQ(send_frame(&f, 1u, CB_MSG_START_MOTOR, CB_PRI_CTRL,
                                NULL, 0u, 1), 0);
            n->last_cmd_ms = 0u;                    /* START_MOTOR 是 0x62，不是 is_ctrl */

            for (int i = 0; i < 10; ++i) {
                CHECK_EQ(send_frame(&f, 1u, CB_MSG_CURRENT_CONTROL, CB_PRI_CTRL,
                                    cur, 4u, 1), 0);
                jsdk_hal_virtual_advance_ms(f.h, 30u);
            }
            CHECK_EQ(n->last_cmd_ms, 0u);           /* 从未武装 */
            CHECK_EQ(n->armed, 1u);                 /* 300 ms 仍未停机 */
            CHECK((n->error_axis & SIM_ERR_CAN_BUS_FAILED) == 0u);
            printf("      [SAFETY HOLE] CURRENT-only client: timeout never arms "
                   "(last_cmd_ms == 0), axis keeps running\n");
        }
    }

    /* ===== ⚠ 场景 A2：break_timeout = 0 ⇒ **超时检测整个被禁用** =====
       新固件语义（也是 `Config_t::break_timeout` 的默认值）：0 **不是** 100 ms，
       而是“永不超时”。模型必须一致，否则“没武装”会被误读成“100 ms 已武装”。
       这里先真的武装它（发一条 MIT），再停发 5 s —— 不得停机、不得置错误位。 */
    {
        uint8_t mp[8];
        cb_mit_range_t r;

        fix_reopen(&f, "0:id=1,gear=16.5,timeout=0,fd");
        n = f.sim ? sim_find_node(f.sim, 1u) : NULL;
        CHECK(n != NULL);
        if (n) {
            CHECK_EQ(n->break_timeout, 0u);
            r.pos_max = n->mit_max_pos; r.vel_max = n->mit_max_vel;
            r.kp_max = n->mit_max_kp;   r.kd_max = n->mit_max_kd;
            r.tau_max = n->mit_max_torque;
            cb_mit_pack_command(mp, &r, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, NULL);
            CHECK_EQ(send_frame(&f, 1u, CB_MSG_START_MOTOR, CB_PRI_CTRL, NULL, 0u, 1), 0);
            CHECK_EQ(send_frame(&f, 1u, CB_MSG_MIT_CONTROL, CB_PRI_HIGH_CTRL,
                                mp, 8u, 1), 0);
            CHECK(n->last_cmd_ms != 0u);            /* 计时器已武装 */

            drain(&f);
            jsdk_hal_virtual_advance_ms(f.h, 5000u);/* 远超任何 100 ms 假设 */
            CHECK((n->error_axis & SIM_ERR_CAN_BUS_FAILED) == 0u);
            CHECK_EQ(n->armed, 1u);
            CHECK(n->last_cmd_ms != 0u);            /* 计时器未被清零 */
            printf("      timeout=0（= 禁用）：武装后停发 5 s 也不停（0 != 100 ms）\n");
        }
    }

    /* ===== ⚠ 场景 B：先发过一次 is_ctrl 再只发 CURRENT → 会被误停 ===== */
    {
        uint8_t mp[8], cur[4];
        cb_mit_range_t r;

        /* ⚠ 必须显式 `timeout=`：0（缺省）在新语义下是**禁用超时**，那样永远停不了，
              这个场景就测不出来了。 */
        fix_reopen(&f, "0:id=1,gear=16.5,timeout=100,fd");
        n = f.sim ? sim_find_node(f.sim, 1u) : NULL;
        CHECK(n != NULL);
        if (n) {
            r.pos_max = n->mit_max_pos; r.vel_max = n->mit_max_vel;
            r.kp_max = n->mit_max_kp;   r.kd_max = n->mit_max_kd;
            r.tau_max = n->mit_max_torque;
            cb_mit_pack_command(mp, &r, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, NULL);
            cb_ctrl_current_pack(cur, 2.0f, NULL);

            CHECK_EQ(send_frame(&f, 1u, CB_MSG_START_MOTOR, CB_PRI_CTRL,
                                NULL, 0u, 1), 0);
            /* 一条 MIT 把计时器武装起来 */
            CHECK_EQ(send_frame(&f, 1u, CB_MSG_MIT_CONTROL, CB_PRI_HIGH_CTRL,
                                mp, 8u, 1), 0);
            CHECK(n->last_cmd_ms != 0u);

            /* 之后只发 CURRENT（每条都喂了驱动器看门狗，但不刷新计时器） */
            for (int i = 0; i < 5; ++i) {
                drain(&f);
                CHECK_EQ(send_frame(&f, 1u, CB_MSG_CURRENT_CONTROL, CB_PRI_CTRL,
                                    cur, 4u, 1), 0);
                jsdk_hal_virtual_advance_ms(f.h, 30u);
                if (i == 0) CHECK_EQ(n->armed, 1u);
            }
            CHECK((n->error_axis & SIM_ERR_CAN_BUS_FAILED) != 0u);
            CHECK_EQ(n->armed, 0u);
            printf("      [MIS-STOP]  after one is_ctrl frame, CURRENT-only traffic "
                   "still trips break_timeout\n");
        }
    }

    fix_close(&f);
}


/* ==========================================================================
 * 7. 参数访问
 * ======================================================================== */

static void test_params(void)
{
    fix_t f;
    jsdk_can_frame_t out;
    sim_node_t *n;

    printf("[7] param access\n");
    fix_open(&f, "0:id=1,gear=16.5,fd");
    n = sim_find_node(f.sim, 1u);
    CHECK(n != NULL);
    if (!n) { fix_close(&f); return; }

    /* --- 单读 u32（node_id，端点 180） --- */
    drain(&f);
    {
        uint8_t req[8];
        cb_param_pack_read_req(req, sizeof req, 180u, 4u, 0u, 1);
        CHECK_EQ(send_frame(&f, 1u, CB_MSG_PARAM_READ, CB_PRI_CONFIG, req, 8u, 1), 0);
        CHECK_EQ(recv_frame(&f, &out), 1);
        CHECK_EQ(cb_id_msgtype(out.id), CB_MSG_PARAM_READ);
        {
            cb_param_read_rsp_t rs;
            CHECK_EQ(cb_param_unpack_read_rsp(out.data, out.len, &rs), 0);
            CHECK_EQ(rs.ep_id, 180u);
            CHECK_EQ(rs.data_len, 4u);
            CHECK_EQ(rs.value[0], 0x01u);      /* node_id = 1，u32 LE */
            CHECK_EQ(rs.value[1], 0x00u);
            CHECK_EQ(rs.value[2], 0x00u);
            CHECK_EQ(rs.value[3], 0x00u);
            CHECK_EQ(cb_param_rsp_has_more(&rs), 0);
        }
    }

    /* --- 单读 u64（motor.error，端点 193）→ Classic 分两段 --- */
    {
        uint8_t req[8];
        uint8_t joined[8];
        unsigned seg;
        const int classic = 1;

        n->error_motor = 0x1122334455667788ull;

        for (seg = 0u; seg < 2u; ++seg) {
            uint32_t off = seg * 4u;
            drain(&f);
            cb_param_pack_read_req(req, sizeof req, 193u, 8u, off, 1);
            CHECK_EQ(send_frame(&f, 1u, CB_MSG_PARAM_READ, CB_PRI_CONFIG,
                                req, 8u, (int)!classic), 0);
            CHECK_EQ(recv_frame(&f, &out), 1);
            {
                cb_param_read_rsp_t rs;
                CHECK_EQ(cb_param_unpack_read_rsp(out.data, out.len, &rs), 0);
                CHECK_EQ(rs.data_len, 4u);        /* Classic 被夹到 4 */
                CHECK_EQ(cb_param_rsp_has_more(&rs), seg == 0u ? 1 : 0);
                memcpy(&joined[off], rs.value, 4);
            }
        }
        /* 拼回来的字节是**线上字节流**：小端下 Low4 = 88 77 66 55，High4 = 44 33 22 11 */
        CHECK_EQ(joined[0], 0x88u); CHECK_EQ(joined[1], 0x77u);
        CHECK_EQ(joined[2], 0x66u); CHECK_EQ(joined[3], 0x55u);
        CHECK_EQ(joined[4], 0x44u); CHECK_EQ(joined[5], 0x33u);
        CHECK_EQ(joined[6], 0x22u); CHECK_EQ(joined[7], 0x11u);
    }

    /* --- 单读：FD 一次读完 8 字节 --- */
    {
        uint8_t req[8];
        drain(&f);
        cb_param_pack_read_req(req, sizeof req, 193u, 8u, 0u, 1);
        CHECK_EQ(send_frame(&f, 1u, CB_MSG_PARAM_READ, CB_PRI_CONFIG, req, 8u, 1), 0);
        CHECK_EQ(recv_frame(&f, &out), 1);
        {
            cb_param_read_rsp_t rs;
            CHECK_EQ(cb_param_unpack_read_rsp(out.data, out.len, &rs), 0);
            CHECK_EQ(rs.data_len, 8u);
            CHECK_EQ(cb_param_rsp_has_more(&rs), 0);
        }
    }

    /* --- 写 f32（gear_ratio，242）→ 断言模型字段真的变了 --- */
    {
        uint8_t req[12];
        uint8_t val[4];
        cb_le_put_f32(val, 8.0f);      /* 参数值 = 小端 */
        CHECK_EQ(cb_param_pack_write_req(req, sizeof req, 242u, val, 4u), 8u);

        drain(&f);
        CHECK_EQ(send_frame(&f, 1u, CB_MSG_PARAM_WRITE, CB_PRI_CONFIG, req, 8u, 1), 0);
        CHECK_FEQ(n->gear_ratio, 8.0f);

        /* 确认帧：8 B 静默（DataLen = 0，Flags 原样） */
        CHECK_EQ(recv_frame(&f, &out), 1);
        CHECK_EQ(cb_id_msgtype(out.id), CB_MSG_PARAM_WRITE);
        CHECK_EQ(out.len, CB_PARAM_ACK_LEN);
        {
            uint8_t fl = 0xEEu;
            uint16_t ep = 0u;
            CHECK_EQ(cb_param_unpack_write_ack(out.data, out.len, &fl, &ep), 0);
            CHECK_EQ(fl, 0x00u);
            CHECK_EQ(ep, 242u);
        }
        CHECK_FEQ(n->gear_ratio, 8.0f);
    }

    /* --- 写入只读端点必须无效 --- */
    {
        uint8_t req[12];
        uint8_t val[4];
        float before = n->pos_estimate;
        cb_le_put_f32(val, 123.0f);
        CHECK_EQ(cb_param_pack_write_req(req, sizeof req, 372u, val, 4u), 8u);
        drain(&f);
        CHECK_EQ(send_frame(&f, 1u, CB_MSG_PARAM_WRITE, CB_PRI_CONFIG, req, 8u, 1), 0);
        CHECK_FEQ(n->pos_estimate, before);         /* 未被改写 */
        CHECK_EQ(recv_frame(&f, &out), 1);          /* 仍会回确认（固件也这样） */
    }

    /* --- 未知端点：值长 0，响应 DataLen = 0 --- */
    {
        uint8_t req[8];
        drain(&f);
        cb_param_pack_read_req(req, sizeof req, 60000u, 4u, 0u, 1);
        CHECK_EQ(send_frame(&f, 1u, CB_MSG_PARAM_READ, CB_PRI_CONFIG, req, 8u, 1), 0);
        CHECK_EQ(recv_frame(&f, &out), 1);
        { cb_param_read_rsp_t rs;
          CHECK_EQ(cb_param_unpack_read_rsp(out.data, out.len, &rs), 0);
          CHECK_EQ(rs.data_len, 0u); }
    }

    /* --- 批量读（FD）：一次读 3 个 --- */
    {
        uint16_t eps[3];
        uint8_t req[64];
        size_t rlen;

        eps[0] = 180u;   /* node_id   u32 → 4 B */
        eps[1] = 73u;    /* timeout   u16 → 2 B */
        eps[2] = 390u;   /* cpr       i32 → 4 B */
        rlen = cb_param_pack_batch_req(req, sizeof req, eps, 3u);
        CHECK_EQ(rlen, 8u);

        drain(&f);
        CHECK_EQ(send_frame(&f, 1u, CB_MSG_PARAM_READ, CB_PRI_CONFIG, req,
                            (uint8_t)rlen, 1), 0);
        CHECK_EQ(recv_frame(&f, &out), 1);
        {
            cb_param_batch_rsp_t br;
            CHECK_EQ(cb_param_unpack_batch_rsp(out.data, out.len, 3u, &br), 0);
            CHECK_EQ(br.is_err, 0);
            CHECK_EQ(br.count, 3u);
            CHECK_EQ(br.bitmap_bytes, 1u);
            CHECK_EQ(br.bitmap[0], 0x07u);
            CHECK_EQ(br.values_len, 10u);           /* 4 + 2 + 4 */
            /* 值流里的字节也是**小端**：node_id = 1 */
            CHECK_EQ(br.values[0], 0x01u);
            CHECK_EQ(br.values[3], 0x00u);
            /* break_timeout = 0（缺省 = 禁用，见 sim_device.h）；u16 LE */
            CHECK_EQ(br.values[4], 0x00u);
            CHECK_EQ(br.values[5], 0x00u);
            /* cpr = 8192 → 0x00002000 (i32 LE) */
            CHECK_EQ(br.values[6], 0x00u);
            CHECK_EQ(br.values[7], 0x20u);
            CHECK_EQ(br.values[8], 0x00u);
            CHECK_EQ(br.values[9], 0x00u);
        }
    }

    /* --- 批量读 8 字节值 → 值流超预算 → ERR（2 B） --- */
    {
        uint16_t eps[8];
        uint8_t req[64];
        unsigned i;
        size_t rlen;

        for (i = 0u; i < 8u; ++i) eps[i] = 193u;    /* u64 × 8 = 64 B 值流 */
        rlen = cb_param_pack_batch_req(req, sizeof req, eps, 8u);

        drain(&f);
        CHECK_EQ(send_frame(&f, 1u, CB_MSG_PARAM_READ, CB_PRI_CONFIG, req,
                            (uint8_t)rlen, 1), 0);
        CHECK_EQ(recv_frame(&f, &out), 1);
        CHECK_EQ(out.len, 2u);
        CHECK_EQ(out.data[0], (uint8_t)(CB_PARAM_FLAG_BATCH | CB_PARAM_FLAG_ERR));
        CHECK_EQ(out.data[1], 0x00u);
        CHECK_EQ(n->param_err_count, 1u);
    }

    /* --- Classic 批量读 → ERR --- */
    {
        uint16_t eps[2];
        uint8_t req[64];
        size_t rlen;

        f.sim->nodes[0].is_fd = 0u;
        eps[0] = 180u; eps[1] = 390u;
        rlen = cb_param_pack_batch_req(req, sizeof req, eps, 2u);

        drain(&f);
        CHECK_EQ(send_frame(&f, 1u, CB_MSG_PARAM_READ, CB_PRI_CONFIG, req,
                            (uint8_t)rlen, 0), 0);
        CHECK_EQ(recv_frame(&f, &out), 1);
        CHECK_EQ(out.len, 2u);
        CHECK_EQ(out.data[1], 0x00u);
        f.sim->nodes[0].is_fd = 1u;
    }

    /* --- Classic 分段写 u64（motor.error，193）---
        先发一个 >4 字节的 DataLen 让模型转入分段装配路径 */
    {
        uint8_t chunk[8];
        uint64_t want = 0xDEADBEEFCAFEBABEull;

        n->error_motor = 0ull;
        drain(&f);

        /* 块 1：More = 1，TotalLen = 8，offset 0 —— 小端下 offset 0 是**低 4 字节**
           （0xCAFEBABE），所以字节是 BE BA FE CA */
        CHECK_EQ(cb_param_pack_write_chunk(chunk, sizeof chunk, 193u, 8u, 0u,
                                           (const uint8_t *)"\xBE\xBA\xFE\xCA",
                                           4u, 1), 8u);
        CHECK_EQ(send_frame(&f, 1u, CB_MSG_PARAM_WRITE, CB_PRI_CONFIG, chunk, 8u, 0), 0);
        CHECK_EQ(recv_frame(&f, &out), 0);           /* 还没完成，不应答 */
        CHECK(n->error_motor == 0ull);                /* 未写入 */

        /* 块 2：More = 0 —— offset 4 = 高 4 字节（0xDEADBEEF）→ EF BE AD DE */
        CHECK_EQ(cb_param_pack_write_chunk(chunk, sizeof chunk, 193u, 8u, 4u,
                                           (const uint8_t *)"\xEF\xBE\xAD\xDE",
                                           4u, 0), 8u);
        CHECK_EQ(send_frame(&f, 1u, CB_MSG_PARAM_WRITE, CB_PRI_CONFIG, chunk, 8u, 0), 0);
        CHECK(n->error_motor == want);
        CHECK_EQ(recv_frame(&f, &out), 1);           /* 末块后回确认 */
        CHECK_EQ(out.len, CB_PARAM_ACK_LEN);
    }

    /* --- 分段写被**另一个主站**打断 → 重新装配，且不得写入半截值 ---
       （不能用 gear_ratio 试：它是 f32 = 4 字节，而分段写只允许 TotalLen 5..8） */
    {
        uint8_t chunk[8];

        n->error_motor = 0ull;
        drain(&f);
        cb_param_pack_write_chunk(chunk, sizeof chunk, 193u, 8u, 0u,
                                  (const uint8_t *)"\x11\x22\x33\x44", 4u, 1);
        CHECK_EQ(send_frame(&f, 1u, CB_MSG_PARAM_WRITE, CB_PRI_CONFIG, chunk, 8u, 0), 0);
        CHECK_EQ(n->error_motor, 0ull);           /* 尚未写入 */

        /* 换主站（source = 2）→ 装配器重置 */
        {
            jsdk_can_frame_t fr;
            memset(&fr, 0, sizeof fr);
            fr.id = cb_make_id(CB_PRI_CONFIG, CB_MSG_PARAM_WRITE, 1u, 2u, 0u);
            fr.len = 8u;
            fr.flags = JSDK_FRAME_EXT;
            cb_param_pack_write_chunk(fr.data, 8u, 193u, 8u, 0u,
                                      (const uint8_t *)"\x01\x00\xFF\xEE", 4u, 1);
            CHECK_EQ(f.hal.send(f.hal.user, &fr), 0);
        }
        CHECK_EQ(n->error_motor, 0ull);           /* 原半截值必须被丢弃 */

        /* 主站 2 的末块 → 完成写入 */
        {
            jsdk_can_frame_t fr;
            memset(&fr, 0, sizeof fr);
            fr.id = cb_make_id(CB_PRI_CONFIG, CB_MSG_PARAM_WRITE, 1u, 2u, 0u);
            fr.len = 8u;
            fr.flags = JSDK_FRAME_EXT;
            cb_param_pack_write_chunk(fr.data, 8u, 193u, 8u, 4u,
                                      (const uint8_t *)"\xDD\xCC\xBB\xAA", 4u, 0);
            CHECK_EQ(f.hal.send(f.hal.user, &fr), 0);
        }
        CHECK(n->error_motor == 0xAABBCCDDEEFF0001ull);
        CHECK_EQ(recv_frame(&f, &out), 1);        /* 末块后回确认 */
        CHECK_EQ(cb_id_dest(out.id), 2u);         /* 回给主站 2 */
    }

    /* --- 畸形分段写（单块声明 TotalLen=5 却只带 4 字节）必须能写入，
           且第 5 字节为 0 —— 这是固件的健壮性缺口，模型如实复刻 --- */
    {
        uint8_t chunk[8];
        n->error_motor = 0xFFFFFFFFFFFFFFFFull;
        drain(&f);
        /* 打包侧必须拒绝这种调用：末块只给 4 字节却声明 TotalLen=8 */
        CHECK_EQ(cb_param_pack_write_chunk(chunk, sizeof chunk, 193u, 8u, 0u,
                                           (const uint8_t *)"\x11\x22\x33\x44",
                                           4u, 0), 0u);
        /* 非末块也不能只给半块 */
        CHECK_EQ(cb_param_pack_write_chunk(chunk, sizeof chunk, 193u, 8u, 0u,
                                           (const uint8_t *)"\x11\x22",
                                           2u, 1), 0u);
        /* 非末块声明 More 却已填满 → 拒绝 */
        CHECK_EQ(cb_param_pack_write_chunk(chunk, sizeof chunk, 193u, 8u, 4u,
                                           (const uint8_t *)"\x11\x22\x33\x44",
                                           4u, 1), 0u);

        /* 手写畸形帧（绕过打包侧校验）：TotalLen=8、More=0、只带 4 字节 */
        memset(chunk, 0, sizeof chunk);
        chunk[0] = 0x00u;                            /* More = 0 */
        cb_be_put_u16(chunk + 1, 193u);
        chunk[3] = 8u;                               /* TotalLen = 8（与 u64 宽度一致）*/
        chunk[4] = 0x11u; chunk[5] = 0x22u; chunk[6] = 0x33u; chunk[7] = 0x44u;
        CHECK_EQ(send_frame(&f, 1u, CB_MSG_PARAM_WRITE, CB_PRI_CONFIG, chunk, 8u, 0), 0);
        /* ⚠ 固件会直接写入一个后 4 字节被静默补 0 的"完整值"；
           小端下这 4 个字节是**低** 32 位，补 0 的是**高** 32 位 */
        CHECK(n->error_motor == 0x0000000044332211ull);
        drain(&f);
        printf("      malformed frame (TotalLen=8, only 4 B sent) wrote "
               "0x0000000044332211 → 高 4 字节被静默补 0（固件 F13）\n");
    }

    fix_close(&f);
}

/* ==========================================================================
 * 8. JSON 描述符传输（用真实的 41029 B 描述符）
 * ======================================================================== */

/** 收集一次完整的描述符传输。
 *
 * ⚠ 帧分类必须**有状态**：只有一次请求后的**第一帧**才是元数据帧；
 *   后续帧一律是数据帧。不能用「第 3 字节是不是左花括号」逐帧判断——
 *   只有 offset 0 的数据帧才以它开头，中间的数据帧是任意 JSON 文本。
 *   那个判断的作用仅仅是区分「元数据帧」与「offset = 0 的数据帧」。
 */
typedef struct {
    int      n_meta;
    int      n_data;
    int      bad;
    uint32_t meta_total;
    uint16_t meta_crc;
    uint32_t got_len;
    uint8_t  got[65536];
    int      first_seen;
} desc_collect_t;

#define DESC_JSON_FIRST_BYTE 0x7Bu   /* '{' */

static void desc_collect_frame(desc_collect_t *c, const jsdk_can_frame_t *f,
                               uint8_t expect_frame_len, uint8_t expect_payload)
{
    if (cb_id_msgtype(f->id) != CB_MSG_JSON_DESC_DATA) { c->bad++; return; }
    if (cb_id_source(f->id) != 1u) { c->bad++; return; }
    if (f->len != expect_frame_len) { c->bad++; return; }

    if (!c->first_seen) {
        /* 第一帧必须是元数据帧：前两字节为 0，且第 3 字节不是 JSON 起始符 */
        if (f->data[0] != 0u || f->data[1] != 0u) { c->bad++; return; }
        if (f->data[2] == DESC_JSON_FIRST_BYTE) { c->bad++; return; }
        c->meta_total = (uint32_t)f->data[2]
                      | ((uint32_t)f->data[3] << 8)
                      | ((uint32_t)f->data[4] << 16)
                      | ((uint32_t)f->data[5] << 24);
        c->meta_crc = (uint16_t)((uint16_t)f->data[6] | ((uint16_t)f->data[7] << 8));
        c->n_meta++;
        c->first_seen = 1;
        return;
    }

    /* 之后一律是数据帧：chunkOffset u16 LE + 载荷 */
    {
        uint32_t off = (uint32_t)f->data[0] | ((uint32_t)f->data[1] << 8);
        uint32_t n_copy = expect_payload;
        if (off != c->got_len) { c->bad++; return; }
        if (off + n_copy > c->meta_total) n_copy = c->meta_total - off;
        if (c->got_len + n_copy > sizeof c->got) { c->bad++; return; }
        memcpy(&c->got[c->got_len], &f->data[2], n_copy);
        c->got_len += n_copy;
        c->n_data++;
    }
}

static void test_desc(void)
{
    fix_t f;
    jsdk_can_frame_t out;
    uint32_t expect_len;
    unsigned i;

    printf("[8] JSON descriptor transfer\n");

    fix_open(&f, "0:id=1,hb=0,fd");
    if (!f.sim) { fix_close(&f); return; }

    if (sim_set_desc_file(f.sim, JSDK_TEST_DATA_DIR "/endpoints_v8.json", 0u) != 0) {
        printf("  FATAL: 无法读取描述符样本\n");
        g_fail++;
        fix_close(&f);
        return;
    }
    expect_len = f.sim->desc.len;
    CHECK_EQ(expect_len, 41029u);

    /* ---------- CAN FD ---------- */
    {
        static desc_collect_t c;

        memset(&c, 0, sizeof c);
        {
            uint8_t req[4] = { 0u, 0u, 0u, 0u };
            CHECK_EQ(send_frame(&f, 1u, CB_MSG_JSON_DESC_READ, CB_PRI_CONFIG,
                                req, 4u, 1), 0);
        }
        CHECK_EQ(sim_desc_active(f.sim), 1);

        for (i = 0u; i < 400u; ++i) {
            jsdk_hal_virtual_advance_ms(f.h, 1u);
            while (recv_frame(&f, &out) == 1) {
                desc_collect_frame(&c, &out, 64u, 62u);
            }
            if (!sim_desc_active(f.sim) && c.got_len >= expect_len) break;
        }

        CHECK_EQ(c.bad, 0);
        CHECK_EQ(c.n_meta, 1);
        CHECK_EQ(c.meta_total, expect_len);
        CHECK(c.meta_crc != 0u);
        CHECK_EQ(c.got_len, expect_len);
        CHECK_EQ(c.n_data, (int)((expect_len + 61u) / 62u));
        CHECK_EQ(c.n_meta + c.n_data, 663);
        CHECK_EQ(sim_desc_active(f.sim), 0);

        {
            FILE *fh = fopen(JSDK_TEST_DATA_DIR "/endpoints_v8.json", "rb");
            if (!fh) {
                printf("  FATAL: 无法打开描述符样本\n");
                g_fail++;
            } else {
                static uint8_t ref[65536];
                size_t n = fread(ref, 1u, sizeof ref, fh);
                fclose(fh);
                CHECK_EQ(n, (size_t)expect_len);
                CHECK(memcmp(c.got, ref, (size_t)expect_len) == 0);
            }
        }
        printf("      FD      : %d frames (%d meta + %d data), %u B reassembled, "
               "byte-identical\n",
               c.n_meta + c.n_data, c.n_meta, c.n_data, (unsigned)c.got_len);
    }

    /* ---------- Classic ---------- */
    {
        static desc_collect_t c;

        memset(&c, 0, sizeof c);
        f.sim->nodes[0].is_fd = 0u;
        drain(&f);
        {
            uint8_t req[4] = { 0u, 0u, 0u, 0u };
            CHECK_EQ(send_frame(&f, 1u, CB_MSG_JSON_DESC_READ, CB_PRI_CONFIG,
                                req, 4u, 0), 0);
        }
        for (i = 0u; i < 400u; ++i) {
            jsdk_hal_virtual_advance_ms(f.h, 1u);
            while (recv_frame(&f, &out) == 1) {
                desc_collect_frame(&c, &out, 8u, 6u);
            }
            if (!sim_desc_active(f.sim) && c.got_len >= expect_len) break;
        }
        CHECK_EQ(c.bad, 0);
        CHECK_EQ(c.n_meta, 1);
        CHECK_EQ(c.got_len, expect_len);
        CHECK_EQ(c.n_data, (int)((expect_len + 5u) / 6u));
        CHECK_EQ(c.n_meta + c.n_data, 6840);
        printf("      Classic : %d frames (%d meta + %d data), %u B reassembled\n",
               c.n_meta + c.n_data, c.n_meta, c.n_data, (unsigned)c.got_len);
    }

    /* ---------- 中途重新请求会重置传输 ---------- */
    {
        uint8_t req[4] = { 0u, 0u, 0u, 0u };
        f.sim->nodes[0].is_fd = 1u;
        drain(&f);
        CHECK_EQ(send_frame(&f, 1u, CB_MSG_JSON_DESC_READ, CB_PRI_CONFIG, req, 4u, 1), 0);
        jsdk_hal_virtual_advance_ms(f.h, 2u);
        drain(&f);
        /* 此时可能已经传完（14 ms 才够，2 ms 不够），确认仍在进行 */
        CHECK_EQ(sim_desc_active(f.sim), 1);
        CHECK_EQ(send_frame(&f, 1u, CB_MSG_JSON_DESC_READ, CB_PRI_CONFIG, req, 4u, 1), 0);
        CHECK_EQ(f.sim->desc.metadata_sent, 0);
        CHECK_EQ(f.sim->desc.offset, 0u);
        jsdk_hal_virtual_advance_ms(f.h, 1u);
        CHECK_EQ(recv_frame(&f, &out), 1);
        CHECK_EQ(out.data[0], 0u);
        CHECK(out.data[2] != DESC_JSON_FIRST_BYTE);      /* 又是元数据帧 */
    }

    /* ---------- 新总线上的设备**自带**描述符（由端点表生成） ----------
     *
     * 这条契约支撑 `jsdk-cli --if virtual`：现场没有硬件、也没有
     * 测试夹具时，仿真设备也必须能回答 0x24/0x25，否则 CLI 会以
     * "descriptor download ... (0/0 bytes)" 收场。
     *
     * 顺便验证"元数据与设备行为同源"：生成的 JSON 里必须出现 `k_eps`
     * 的**每一条** path / id（否则就是两份清单漂移，一般会以"读到的值类型
     * 不对"的形式在客户现场爆出来）。 */
    {
        fix_t  g;
        static char json[8192];
        size_t used = 0u;
        unsigned frames = 0u;

        fix_open(&g, "0:id=1,fd");
        if (g.sim) {
            uint8_t req[4] = { 0u, 0u, 0u, 0u };
            jsdk_can_frame_t fr;

            CHECK_EQ(send_frame(&g, 1u, CB_MSG_JSON_DESC_READ, CB_PRI_CONFIG,
                                req, 4u, 1), 0);
            jsdk_hal_virtual_advance_ms(g.h, 200u);

            while (recv_frame(&g, &fr)) {
                frames++;
                if (frames == 1u) continue;            /* 第一帧是元数据帧 */
                if (fr.len > 2u && used + (size_t)(fr.len - 2u) < sizeof json) {
                    memcpy(&json[used], &fr.data[2], (size_t)(fr.len - 2u));
                    used += (size_t)(fr.len - 2u);
                }
            }
            json[used] = '\0';

            CHECK(frames > 1u);
            CHECK(used > 0u);

            /* 逐条比对端点表：路径与 id 都必须出现在生成的 JSON 里 */
            {
                size_t n = 0u;
                const sim_ep_def_t *eps = sim_default_endpoints(&n);
                size_t k;
                unsigned missing = 0u;

                for (k = 0u; k < n; ++k) {
                    char needle[192];
                    snprintf(needle, sizeof needle, "\"name\":\"%s\"", eps[k].path);
                    if (!strstr(json, needle)) missing++;
                }
                CHECK_EQ(missing, 0u);
                printf("      built-in descriptor: %u B, %u endpoint defs all present\n",
                       (unsigned)used, (unsigned)n);
            }

            /* 一次请求只能有一个元数据帧（重复请求才会再发） */
            {
                jsdk_can_frame_t again;
                jsdk_hal_virtual_advance_ms(g.h, 100u);
                CHECK_EQ(recv_frame(&g, &again), 0);
            }
        }
        fix_close(&g);
    }

    /* ---------- 非法参数 ---------- */
    CHECK_EQ(sim_set_desc(f.sim, NULL, 10u, 0u), -1);
    CHECK_EQ(sim_set_desc(f.sim, "{}", 0u, 0u), -1);
    CHECK_EQ(sim_set_desc_file(f.sim, "no/such/file.json", 0u), -1);

    fix_close(&f);
}

static void test_estop(void)
{
    fix_t f;
    sim_node_t *n1, *n2;
    uint8_t mp[8];
    cb_mit_range_t r;

    printf("[9] ESTOP\n");
    fix_open(&f, "0:id=1,gear=16.5,fd;1:id=2,gear=8,fd");
    n1 = sim_find_node(f.sim, 1u);
    n2 = sim_find_node(f.sim, 2u);
    CHECK(n1 != NULL && n2 != NULL);
    if (!n1 || !n2) { fix_close(&f); return; }

    /* 先使能并让转子动起来 */
    drain(&f);
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_START_MOTOR, CB_PRI_CTRL, NULL, 0u, 1), 0);
    CHECK_EQ(send_frame(&f, 2u, CB_MSG_START_MOTOR, CB_PRI_CTRL, NULL, 0u, 1), 0);
    r.pos_max = n1->mit_max_pos; r.vel_max = n1->mit_max_vel;
    r.kp_max = n1->mit_max_kp;   r.kd_max = n1->mit_max_kd;
    r.tau_max = n1->mit_max_torque;
    cb_mit_pack_command(mp, &r, 0.5f, 2.0f, 0.0f, 0.0f, 5.0f, NULL);
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_MIT_CONTROL, CB_PRI_HIGH_CTRL, mp, 8u, 1), 0);
    jsdk_hal_virtual_advance_ms(f.h, 20u);
    drain(&f);
    CHECK_EQ(n1->armed, 1u);

    /* ESTOP：最高优先级，任何节点（含未使能的）都应停机 */
    CHECK_EQ(send_frame(&f, 0xFFu, CB_MSG_ESTOP, CB_PRI_CRITICAL, NULL, 0u, 1), 0);
    CHECK_EQ(n1->armed, 0u);
    CHECK_EQ(n2->armed, 0u);
    CHECK((n1->error_axis & SIM_ERR_ESTOP_REQUESTED) != 0u);
    CHECK((n2->error_axis & SIM_ERR_ESTOP_REQUESTED) != 0u);
    CHECK_EQ(n1->vel_estimate, 0.0f);
    CHECK_EQ(n1->iq_measured, 0.0f);

    /* 清错后重新使能 */
    drain(&f);
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_CLEAR_ERRORS, CB_PRI_CTRL, NULL, 0u, 1), 0);
    CHECK_EQ(send_frame(&f, 1u, CB_MSG_START_MOTOR, CB_PRI_CTRL, NULL, 0u, 1), 0);
    CHECK_EQ(n1->armed, 1u);

    fix_close(&f);
}

/* ==========================================================================
 * main
 * ======================================================================== */

int main(void)
{
    /* 无缓冲输出：本测试涉及大缓冲与指针运算，万一崩溃也要能看到进度 */
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("=== virtual HAL tests (WP1) ===\n\n");
    test_transport();           printf("\n");
    test_addressing();          printf("\n");
    test_mit();                 printf("\n");
    test_ctrl();                printf("\n");
    test_query();               printf("\n");
    test_heartbeat_timeout();   printf("\n");
    test_params();              printf("\n");
    test_desc();                printf("\n");
    test_estop();

    printf("\n=== %d checks, %d failures ===\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
