/**
 * @file    test_ops.c
 * @brief   WP4（健壮性）+ WP5（运维）测试
 *
 * @par 关于时钟
 *  与前几步不同，这里的被测 API（标定/回零/发现/批量读）**必须让时间流逝**：
 *  设备的瞬时状态靠 `sim_tick` 推进，而 `sim_tick` 只由
 *  `jsdk_hal_virtual_advance_ms()` 驱动。所以本文件把虚拟后端包一层：
 *  自己的 `now_ms` 每被调用一次就推进 1 ms 仿真时间——这正是**真实 HAL**
 *  的行为（自由运行的硬件计数器），于是 SDK 的阻塞等待自然能完成。
 *  `tests/test_joint.c` 则保留“手工推进时钟”的写法，用来验证**冻结时钟**下
 *  的非阻塞路径。
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

#define CHECK_STR(a, b)                                                        \
    do {                                                                       \
        const char *_a = (a), *_b = (b);                                       \
        g_checks++;                                                            \
        if ((_a == NULL) != (_b == NULL) || (_a && strcmp(_a, _b) != 0)) {     \
            printf("  FAIL %s:%d  \"%s\" != expected \"%s\"\n", __FILE__,      \
                   __LINE__, _a ? _a : "(null)", _b ? _b : "(null)");          \
            g_fail++;                                                          \
        }                                                                      \
    } while (0)

/* ==========================================================================
 * 夹具：自动推进时钟的虚拟总线
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
    /* 内层：虚拟后端原生回调（user = jsdk_hal_handle_t*） */
    jsdk_can_hal_t     inner;
    jsdk_hal_handle_t *h;
    sim_bus_t         *sim;

    /* 外层：给 SDK 用的 HAL（user = 本结构体） */
    jsdk_can_hal_t     hal;

    jsdk_context_storage_t   store;
    jsdk_context_t          *ctx;
    jsdk_joint_config_t      jc[JSDK_MAX_JOINTS];
    jsdk_joint_t            *j;
    jsdk_context_config_t    cfg;

    /* 注入故障：丢掉主站发出的前 N 帧（模拟“适配器刚打开、前几帧写丢”） */
    unsigned                 drop_tx_head;
    unsigned                 dropped;
    unsigned                 desc_reqs_on_bus;   /* 真正上总线的 0x24 请求数 */

    /* 注入故障 2：按 MsgType 丢（用于单独验证“握手帧丢了会怎样”） */
    uint8_t                  drop_msgtype;
    unsigned                 drop_msgtype_n;
    unsigned                 dropped_msgtype;

    /* 注入故障 3：RX 里先塞入 N 个“上一次传输的残留帧”（非元数据） */
    unsigned                 inject_stale;
    unsigned                 injected;
    unsigned                 desc_stream_seen;    /* 设备已开始发本次 0x25 流 */

    /* 注入故障 4：把某个端点的**读响应值**改成指定值
       （模拟真机 F28：`can.config.read_timeout` 读回恒为 0） */
    uint16_t                 patch_read_ep;
    uint16_t                 patch_read_value;

    /* 注入故障 5：时钟冻住（`now_ms` 不前进）—— 客户自写假时钟的常见写法。
       时间预算永远不到期，所以预热**必须有轮次上限**，否则就是死循环。 */
    int                      freeze_clock;
} fix_t;

static int w_send(void *u, const jsdk_can_frame_t *f)
{
    fix_t *fx = (fix_t *)u;

    if (fx->dropped < fx->drop_tx_head) {
        fx->dropped++;
        return 0;                 /* “发出去了”但从总线上消失（真机就是这个语义） */
    }
    if (fx->drop_msgtype != 0u && cb_id_msgtype(f->id) == fx->drop_msgtype
        && fx->dropped_msgtype < fx->drop_msgtype_n) {
        fx->dropped_msgtype++;
        return 0;
    }
    if (cb_id_msgtype(f->id) == CB_MSG_JSON_DESC_READ) fx->desc_reqs_on_bus++;
    return fx->inner.send(fx->inner.user, f);
}

static int w_recv(void *u, jsdk_can_frame_t *f)
{
    fix_t *fx = (fix_t *)u;
    int     n = fx->inner.recv(fx->inner.user, f);

    if (n == 1 && cb_id_msgtype(f->id) == 0x25u) fx->desc_stream_seen++;

    /* ⚠ 残留帧必须在**请求发出之后、设备开口之前**才开始注入：
       - 请求之前：`desc_drain_rx()` 会先排干净，复现不了真机现场；
       - 设备开口之后：残留帧会插进本次流里，那不是真机情形（真机上设备一开始
         应答，残留就已被 drain 排掉或落在 skip 阶段）。
       ⚠ 帧长必须是合法的 0x25 载荷（8 或 64 B）—— 长度不对会被当场判
         `frame length must be 8 (Classic) or 64 (FD)`，那样测的就不是判据了。 */
    if (n == 0
        && (fx->dropped_msgtype > 0u || fx->desc_reqs_on_bus > 0u)
        && fx->desc_stream_seen == 0u
        && fx->injected < fx->inject_stale) {
        static uint8_t stale[64];

        if (fx->injected == 0u) {
            /* 前两字节非 00 00 → 不是元数据帧；其余当 JSON 文本的尾巴 */
            memset(stale, 0, sizeof stale);
            stale[0] = 0x10u; stale[1] = 0x00u;
            stale[2] = '{'; stale[3] = '"'; stale[4] = 'x'; stale[5] = '"';
            stale[6] = ':'; stale[7] = '1'; stale[8] = '}';
        }
        memset(f, 0, sizeof *f);
        /* dest = 主站（夹具用 master_id = 1），src = 设备 node 1 */
        f->id    = cb_make_id(CB_PRI_CONFIG, 0x25u, 1u, 1u, 0u);
        f->len   = (uint8_t)sizeof stale;
        f->flags = JSDK_FRAME_EXT | JSDK_FRAME_FD;
        memcpy(f->data, stale, sizeof stale);
        fx->injected++;
        return 1;
    }
    /* 注入故障 4：把某个端点的**读响应值**改成指定值（模拟真机 F28 的“读回恒 0”） */
    if (n == 1 && fx->patch_read_ep != 0u
        && cb_id_msgtype(f->id) == CB_MSG_PARAM_READ && f->len >= 6u
        && cb_be_get_u16(f->data + 1) == fx->patch_read_ep) {
        /* 参数读响应：[flags][ep_id u16 BE][data_len][value…]；**值字节是小端** */
        cb_le_put_u16(f->data + 4, fx->patch_read_value);
        if (f->len >= 8u) cb_le_put_u16(f->data + 6, 0u);
    }
    return n;
}

/**
 * 每次被调用推进 1 ms 仿真时间再返回。
 * 真实 HAL 的 `now_ms` 是自由运行计数器，语义一致。
 */
static uint32_t w_now(void *u)
{
    fix_t *fx = (fix_t *)u;

    if (!fx->freeze_clock) jsdk_hal_virtual_advance_ms(fx->h, 1u);
    return fx->inner.now_ms(fx->inner.user);
}

/** 建总线 + 上下文 + n 个关节（不下载描述符）。`is_fd` = 0 时走 Classic。 */
static int fx_open_flags(fix_t *fx, const char *spec, unsigned n_joints, int is_fd)
{
    static uint8_t arena[32768];
    unsigned i;

    memset(fx, 0, sizeof *fx);

    if (jsdk_hal_virtual_open(&fx->inner, &fx->h, spec) != JSDK_OK) return -1;
    fx->sim = jsdk_hal_virtual_sim(fx->h);
    if (!fx->sim) return -1;
    if (sim_set_desc(fx->sim, g_json, (uint32_t)g_json_len, 0x1234u) != 0) return -1;

    fx->hal = fx->inner;          /* 复制回调，再换掉 user / now_ms */
    fx->hal.user   = fx;
    fx->hal.send   = w_send;
    fx->hal.recv   = w_recv;
    fx->hal.now_ms = w_now;

    jsdk_context_config_default(&fx->cfg);
    fx->cfg.hal = fx->hal;
    fx->cfg.master_id = 1u;
    fx->cfg.is_fd = (uint8_t)(is_fd ? 1u : 0u);
    fx->cfg.period_ns = 1000000u;
    fx->cfg.auto_keepalive = 1u;
    fx->cfg.desc.retain = JSDK_DESC_RETAIN_ALL;
    fx->cfg.desc.arena = arena;
    fx->cfg.desc.arena_size = sizeof arena;
    fx->cfg.desc.timeout_ms = 5000u;

    if (jsdk_context_init((jsdk_context_t *)&fx->store, &fx->cfg) != JSDK_OK) return -1;
    fx->ctx = (jsdk_context_t *)&fx->store;

    for (i = 0u; i < n_joints; ++i) {
        memset(&fx->jc[i], 0, sizeof fx->jc[i]);
        fx->jc[i].node_id = (uint8_t)(i + 1u);
        fx->jc[i].initial_mode = JSDK_MODE_MIT;
        if (jsdk_context_add_joint(fx->ctx, &fx->jc[i], &fx->j) != JSDK_OK) return -1;
    }
    fx->j = &fx->ctx->joints[0];
    return 0;
}

/** 便捷包装：默认 FD（本文件绝大多数用例都是 FD）。 */
static int fx_open(fix_t *fx, const char *spec, unsigned n_joints)
{
    return fx_open_flags(fx, spec, n_joints, 1);
}

/** 下好描述符并 configure（本文件的 HAL 会推进时钟，阻塞路径可用）。 */
static int fx_configure(fix_t *fx)
{
    jsdk_status_t st;

    st = jsdk_context_desc_fetch(fx->ctx);
    if (st != JSDK_OK) {
        printf("      desc_fetch -> %s\n", jsdk_context_last_error(fx->ctx));
        return -1;
    }
    st = jsdk_context_configure(fx->ctx);
    if (st != JSDK_OK) {
        printf("      configure -> %s\n", jsdk_context_last_error(fx->ctx));
        return -1;
    }
    return 0;
}

static void fx_close(fix_t *fx)
{
    if (fx->ctx) jsdk_context_destroy(fx->ctx);
    if (fx->h) jsdk_hal_close(fx->h);
}

static void fx_cycle(fix_t *fx)
{
    jsdk_context_cycle_begin(fx->ctx, 0u);
    jsdk_context_cycle_end(fx->ctx);
}

/** 统计主站发出的某类帧（从捕获队列里数，会清空队列）。 */
static unsigned count_tx(fix_t *fx, uint8_t msgtype, int batch_only)
{
    jsdk_can_frame_t f;
    unsigned n = 0u;

    while (jsdk_hal_virtual_capture(fx->h, &f)) {
        if (cb_id_msgtype(f.id) != msgtype) continue;
        if (batch_only && !(f.len > 0u && (f.data[0] & CB_PARAM_FLAG_BATCH))) continue;
        n++;
    }
    return n;
}

static void drain_tx(fix_t *fx) { (void)count_tx(fx, 0u, 0); }

/**
 * 取出捕获队列里所有 `PARAM_READ` **请求**帧，并记下决定分块行为的三个量：
 * 载荷长度（4 = 旧式无 offset / 8 = 带 offset）、`ReqLen` 字节、`offset` 字段。
 *
 * 为什么值得单独断言：第一块用哪种形式（旧式 4 B vs 带 offset 的 8 B）
 * **仿真设备两种都接受**，所以只看“读回来的值对不对”是发现不了写错的；
 * 而真机/旧固件上旧式形式兼容性更好，这个选择必须被测试钉住。
 */
static unsigned take_read_requests(fix_t *fx, uint8_t *lens_out,
                                  uint8_t *reqlen_out, uint32_t *offset_out,
                                  unsigned max_n)
{
    jsdk_can_frame_t f;
    unsigned n = 0u;

    while (jsdk_hal_virtual_capture(fx->h, &f)) {
        if (cb_id_msgtype(f.id) != CB_MSG_PARAM_READ) continue;
        if (n < max_n) {
            lens_out[n]   = f.len;
            reqlen_out[n] = (f.len > 3u) ? f.data[3] : 0u;
            offset_out[n] = (f.len >= CB_PARAM_READ_REQ_FULL)
                            ? cb_be_get_u32(f.data + 4) : 0u;
        }
        n++;
    }
    return n;
}

/* ==========================================================================
 * 1. 故障码文本（WP4）
 * ======================================================================== */

static void test_fault_text(void)
{
    unsigned i;
    /* 取自固件 `can_cyberbeast.hpp` 的 `ErrorCode` —— **不是** ODrive 的
       `ErrorCode` 枚举（两者顺序完全不同，凭记忆写必错）。 */
    static const char *const k_mit[9] = {
        "NONE", "MOTOR", "ENCODER", "CONTROLLER", "UNDER_VOLTAGE",
        "OVER_TEMP", "OVER_CURRENT", "STALL", "CAN_TIMEOUT"
    };

    printf("[1] fault code text\n");

    for (i = 0u; i < 9u; ++i) {
        CHECK_STR(jsdk_joint_error_string((uint8_t)i), k_mit[i]);
    }
    CHECK_STR(jsdk_joint_error_string(0xFu), "MULTIPLE");   /* 线宽 4 bit 的最高值 */
    CHECK(jsdk_joint_error_string(9u) != NULL);             /* 未定义值有兜底 */
    CHECK(strcmp(jsdk_joint_error_string(9u), "NONE") != 0);

    /* 心跳 5-bit 位名 */
    CHECK_STR(jsdk_hb_error_bit_name(0u), "axis");
    CHECK_STR(jsdk_hb_error_bit_name(4u), "board");
    CHECK(jsdk_hb_error_bit_name(5u) == NULL);
    CHECK(jsdk_hb_error_bit_name(99u) == NULL);

    /* 32-bit 位图：只认固件定义过的那 13 个位 */
    CHECK_STR(jsdk_axis_error_bit_name(0u),  "INVALID_STATE");
    CHECK_STR(jsdk_axis_error_bit_name(20u), "CAN_BUS_FAILED");
    CHECK_STR(jsdk_axis_error_bit_name(14u), "ESTOP_REQUESTED");
    CHECK(jsdk_axis_error_bit_name(1u) == NULL);        /* 固件未定义 → 不编名字 */
    CHECK(jsdk_axis_error_bit_name(21u) == NULL);

    {
        unsigned bit = 99u;
        const char *nm;

        CHECK(jsdk_axis_error_first(0u, &bit) == NULL);          /* 无错误 */
        CHECK_EQ(bit, 0u);

        nm = jsdk_axis_error_first(1u << 20, &bit);
        CHECK_STR(nm, "CAN_BUS_FAILED");
        CHECK_EQ(bit, 20u);

        /* 多个位同时置位 → 取**位号最小**的那个（结果稳定、可复现） */
        nm = jsdk_axis_error_first((1u << 14) | (1u << 20), &bit);
        CHECK_STR(nm, "ESTOP_REQUESTED");
        CHECK_EQ(bit, 14u);

        /* 只置了固件未定义的位 → 不猜，返回 NULL */
        CHECK(jsdk_axis_error_first(0x00000002u, &bit) == NULL);
    }

    CHECK_STR(jsdk_can_axis_state_name(0u),  "UNDEFINED");
    CHECK_STR(jsdk_can_axis_state_name(3u),  "FULL_CALIBRATION_SEQUENCE");
    CHECK_STR(jsdk_can_axis_state_name(11u), "HOMING");
    CHECK_STR(jsdk_can_axis_state_name(16u), "MOTOR_DEADTIME_CALIBRATION");
    CHECK_STR(jsdk_can_axis_state_name(17u), "unknown");

    /* 组合描述 */
    {
        fix_t fx;
        char   buf[256];
        int    n;

        if (fx_open(&fx, "0:id=1,hb=10,timeout=30000,fd", 1u) != 0) {
            printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
        }
        if (fx_configure(&fx) != 0) { g_fail++; g_checks++; fx_close(&fx); return; }

        /* 注入一个 32-bit 轴错误 + 心跳子系统位，再看描述串 */
        fx.j->fb.err_code   = 8u;                       /* CAN_TIMEOUT */
        fx.j->fb.hb_error   = (uint8_t)(CB_HB_ERR_AXIS | CB_HB_ERR_MOTOR);
        fx.j->fb.axis_error = 1u << 20;                 /* CAN_BUS_FAILED */
        fx.j->current_state_raw = 8u;                   /* CLOSED_LOOP_CONTROL */
        fx.j->state_known = 1u;

        n = jsdk_joint_describe_fault(fx.j, buf, sizeof buf);
        CHECK(n > 0);
        CHECK(strstr(buf, "CAN_BUS_FAILED") != NULL);
        CHECK(strstr(buf, "CAN_TIMEOUT") != NULL);
        CHECK(strstr(buf, "axis|motor") != NULL);
        CHECK(strstr(buf, "CLOSED_LOOP_CONTROL") != NULL);
        printf("      %s\n", buf);

        /* 无故障时也要给得出可读串 */
        fx.j->fb.err_code = 0u; fx.j->fb.hb_error = 0u; fx.j->fb.axis_error = 0u;
        n = jsdk_joint_describe_fault(fx.j, buf, sizeof buf);
        CHECK(n > 0);
        CHECK(strstr(buf, "hb=none") != NULL);
        printf("      %s\n", buf);

        CHECK_EQ(jsdk_joint_describe_fault(NULL, buf, sizeof buf), 0);
        fx_close(&fx);
    }
}

/* ==========================================================================
 * 2. 参数访问（WP5）
 * ======================================================================== */

static void test_params(void)
{
    fix_t fx;
    jsdk_value_t v;

    printf("[2] parameter access\n");

    if (fx_open(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                     "kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    if (fx_configure(&fx) != 0) { g_fail++; g_checks++; fx_close(&fx); return; }

    /* --- 类型化读 --- */
    CHECK_EQ(jsdk_joint_param_get(fx.j, "axis0.motor.config.gear_ratio", &v), JSDK_OK);
    CHECK_EQ(v.type, JSDK_EP_F32);
    CHECK_NEAR(v.v.f32, 16.0f, 1e-6);

    CHECK_EQ(jsdk_joint_param_get(fx.j, "axis0.config.can.node_id", &v), JSDK_OK);
    CHECK_EQ(v.type, JSDK_EP_U32);
    CHECK_EQ(v.v.u32, 1u);

    CHECK_EQ(jsdk_joint_param_get(fx.j, "axis0.config.enable_watchdog", &v), JSDK_OK);
    CHECK_EQ(v.type, JSDK_EP_BOOL);
    CHECK_EQ(v.v.boolean, 1);

    {
        uint32_t u = 0u;
        int      b = -1;
        float    f = 0.0f;

        CHECK_EQ(jsdk_joint_param_get_u32(fx.j, "axis0.config.can.node_id", &u), JSDK_OK);
        CHECK_EQ(u, 1u);
        CHECK_EQ(jsdk_joint_param_get_bool(fx.j, "axis0.config.enable_watchdog", &b),
                 JSDK_OK);
        CHECK_EQ(b, 1);
        CHECK_EQ(jsdk_joint_param_get_f32(fx.j, "axis0.controller.config.mit_max_pos",
                                          &f), JSDK_OK);
        CHECK_NEAR(f, 12.5f, 1e-6);
        /* 整型端点用 f32 读也应成功（宽化），但比值端点用 u32 读要报错 */
        CHECK_EQ(jsdk_joint_param_get_u32(fx.j, "axis0.motor.config.gear_ratio", &u),
                 JSDK_ERR_PROTOCOL);
    }

    /* --- 未命中：不猜、不近似 --- */
    CHECK_EQ(jsdk_joint_param_get(fx.j, "axis0.motor.config.nope", &v),
             JSDK_ERR_NOT_FOUND);
    CHECK(strstr(jsdk_context_last_error(fx.ctx), "not found") != NULL);

    /* --- 只读端点写入必须被拒 --- */
    CHECK_EQ(jsdk_joint_param_set_f32(fx.j, "vbus_voltage", 1.0f), JSDK_ERR_UNSUPPORTED);

    /* --- 类型不匹配必须报错（猜宽度会静默写坏邻字段） --- */
    CHECK_EQ(jsdk_joint_param_set_f32(fx.j, "axis0.config.can.node_id", 1.0f),
             JSDK_ERR_PROTOCOL);      /* 端点存在（非 NOT_FOUND），但 f32 ≠ u32 */
    {
        jsdk_value_t bad;
        memset(&bad, 0, sizeof bad);
        bad.type = JSDK_EP_F32;
        bad.v.f32 = 1.0f;
        CHECK_EQ(jsdk_joint_param_set(fx.j, "axis0.config.can.node_id", &bad),
                 JSDK_ERR_PROTOCOL);
        CHECK(strstr(jsdk_context_last_error(fx.ctx), "type mismatch") != NULL);
    }

    /* --- 真正的写入 + 读回 --- */
    {
        uint32_t before = 0u, after = 0u;
        float    flim = 0.0f;

        CHECK_EQ(jsdk_joint_param_get_u32(fx.j, "axis0.config.can.node_id", &before),
                 JSDK_OK);
        CHECK_EQ(before, 1u);

        /* 写 u32 端点：node_id 改成 1（幂等），确认写路径通 */
        CHECK_EQ(jsdk_joint_param_set_u32(fx.j, "axis0.config.can.node_id", 1u), JSDK_OK);
        CHECK_EQ(jsdk_joint_param_get_u32(fx.j, "axis0.config.can.node_id", &after),
                 JSDK_OK);
        CHECK_EQ(after, 1u);

        /* u32 便利函数写 u16 端点应自动按宽度降级（不截断成 0） */
        CHECK_EQ(jsdk_joint_param_set_u32(fx.j, "can.config.break_timeout", 1234u),
                 JSDK_OK);
        {
            uint32_t bt = 0u;
            CHECK_EQ(jsdk_joint_param_get_u32(fx.j, "can.config.break_timeout", &bt),
                     JSDK_OK);
            CHECK_EQ(bt, 1234u);
        }

        CHECK_EQ(jsdk_joint_param_set_f32(fx.j,
                                          "axis0.controller.config.vel_limit", 42.5f),
                 JSDK_OK);
        CHECK_EQ(jsdk_joint_param_get_f32(fx.j,
                                          "axis0.controller.config.vel_limit", &flim),
                 JSDK_OK);
        CHECK_NEAR(flim, 42.5f, 1e-6);
        printf("      typed get/set ok (f32/u32/bool, width auto-degrade, "
               "type mismatch rejected)\n");
    }

    /* --- 批量读（FD：必须在**一帧**里完成） --- */
    {
        jsdk_param_req_t reqs[6];
        static const char *const paths[6] = {
            "axis0.motor.config.gear_ratio",            /* f32 */
            "axis0.config.can.node_id",                 /* u32 */
            "can.config.break_timeout",                 /* u16 */
            "axis0.controller.config.control_mode",     /* u8  */
            "axis0.config.enable_watchdog",             /* bool */
            "axis0.motor.config.torque_constant"        /* f32 */
        };
        unsigned i;

        drain_tx(&fx);
        for (i = 0u; i < 6u; ++i) {
            reqs[i].path = paths[i];
            memset(&reqs[i].value, 0, sizeof reqs[i].value);
            reqs[i].status = JSDK_OK;
        }

        CHECK_EQ(jsdk_joint_param_get_batch(fx.j, reqs, 6u), JSDK_OK);
        for (i = 0u; i < 6u; ++i) {
            CHECK_EQ(reqs[i].status, JSDK_OK);
        }

        /* 逐条单读对照 */
        for (i = 0u; i < 6u; ++i) {
            jsdk_value_t solo;
            CHECK_EQ(jsdk_joint_param_get(fx.j, paths[i], &solo), JSDK_OK);
            CHECK_EQ(solo.type, reqs[i].value.type);
            switch (solo.type) {
            case JSDK_EP_F32:   CHECK_NEAR(solo.v.f32, reqs[i].value.v.f32, 1e-6); break;
            case JSDK_EP_U32:   CHECK_EQ(solo.v.u32, reqs[i].value.v.u32); break;
            case JSDK_EP_U16:   CHECK_EQ(solo.v.u16, reqs[i].value.v.u16); break;
            case JSDK_EP_U8:    CHECK_EQ(solo.v.u8,  reqs[i].value.v.u8);  break;
            case JSDK_EP_BOOL:  CHECK_EQ(solo.v.boolean, reqs[i].value.v.boolean); break;
            default: break;
            }
        }

        /* 批量请求帧数：应为 1 帧（62 B 预算装 6 条 18 B 绰绰有余） */
        {
            unsigned nb = count_tx(&fx, CB_MSG_PARAM_READ, 1);
            CHECK_EQ(nb, 1u);
            printf("      6 mixed-type endpoints in %u batch frame "
                   "(values match individual reads)\n", nb);
        }
    }

    /* --- 批量读里的坏路径：只影响那一条，其余照常 --- */
    {
        jsdk_param_req_t reqs[3];
        reqs[0].path = "axis0.motor.config.gear_ratio";
        reqs[1].path = "no.such.endpoint";
        reqs[2].path = "axis0.config.can.node_id";
        memset(reqs[0].value.v.u64 ? &reqs[0].value : &reqs[0].value, 0, 0);
        CHECK_EQ(jsdk_joint_param_get_batch(fx.j, reqs, 3u), JSDK_OK);
        CHECK_EQ(reqs[0].status, JSDK_OK);
        CHECK_EQ(reqs[1].status, JSDK_ERR_NOT_FOUND);
        CHECK_EQ(reqs[2].status, JSDK_OK);
        printf("      bad path in a batch only fails that entry\n");
    }

    /* --- Classic：自动退化为逐条，且**不发**批量请求 --- */
    {
        jsdk_param_req_t reqs[3];
        fix_t cx;

        if (fx_open(&cx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                         "kpmax=500,kdmax=5,hb=10,timeout=30000,classic", 1u) == 0) {
            cx.cfg.is_fd = 0u;
            /* is_fd 影响帧编码，必须重新 init */
            jsdk_context_destroy(cx.ctx);
            cx.cfg.hal = cx.hal;
            if (jsdk_context_init((jsdk_context_t *)&cx.store, &cx.cfg) == JSDK_OK) {
                cx.ctx = (jsdk_context_t *)&cx.store;
                if (jsdk_context_add_joint(cx.ctx, &cx.jc[0], &cx.j) == JSDK_OK) {
                    if (fx_configure(&cx) == 0) {
                        reqs[0].path = "axis0.motor.config.gear_ratio";
                        reqs[1].path = "axis0.config.enable_watchdog";
                        reqs[2].path = "axis0.motor.config.current_lim";
                        drain_tx(&cx);
                        CHECK_EQ(jsdk_joint_param_get_batch(cx.j, reqs, 3u), JSDK_OK);
                        CHECK_EQ(reqs[0].status, JSDK_OK);
                        CHECK_EQ(reqs[1].status, JSDK_OK);
                        CHECK_EQ(reqs[2].status, JSDK_OK);
                        CHECK_NEAR(reqs[0].value.v.f32, 16.0f, 1e-6);
                        CHECK_EQ(count_tx(&cx, CB_MSG_PARAM_READ, 1), 0u); /* 无批量帧 */
                        printf("      Classic: batch auto-degraded to single reads "
                               "(0 batch frames)\n");
                    }
                }
            }
            fx_close(&cx);
        }
    }

    fx_close(&fx);
}

/* ==========================================================================
 * 3. SDO 槽位（WP5）
 * ======================================================================== */

static void test_sdo(void)
{
    fix_t fx;
    jsdk_sdo_handle_t h;

    printf("[3] SDO-style slots\n");

    if (fx_open(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                     "kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    if (fx_configure(&fx) != 0) { g_fail++; g_checks++; fx_close(&fx); return; }

    h = jsdk_joint_sdo_create_by_name(fx.j, "axis0.motor.config.gear_ratio");
    CHECK(h > 0);
    CHECK_EQ(jsdk_joint_sdo_data_size(fx.j, h), 4u);
    CHECK_EQ(jsdk_joint_sdo_state(fx.j, h), JSDK_SDO_IDLE);
    CHECK(jsdk_joint_sdo_data(fx.j, h) != NULL);

    /* SDO 缓冲区里的字节就是**线上字节（小端）** */
    CHECK_EQ(jsdk_joint_sdo_read(fx.j, h), JSDK_OK);
    CHECK_EQ(jsdk_joint_sdo_state(fx.j, h), JSDK_SDO_SUCCESS);
    CHECK_EQ(cb_le_get_f32(jsdk_joint_sdo_data(fx.j, h)), 16.0f);

    /* 写回同一个值（幂等），再读回 */
    {
        uint8_t *d = jsdk_joint_sdo_data(fx.j, h);
        cb_le_put_f32(d, 20.0f);
        CHECK_EQ(jsdk_joint_sdo_write(fx.j, h), JSDK_OK);
        CHECK_EQ(jsdk_joint_sdo_state(fx.j, h), JSDK_SDO_SUCCESS);
        CHECK_EQ(jsdk_joint_sdo_read(fx.j, h), JSDK_OK);
        CHECK_NEAR(cb_le_get_f32(jsdk_joint_sdo_data(fx.j, h)), 20.0f, 1e-5);
    }

    /* 只读端点写：必须报 UNSUPPORTED */
    {
        jsdk_sdo_handle_t ro = jsdk_joint_sdo_create_by_name(fx.j, "axis0.encoder.pos_estimate");
        CHECK(ro > 0);
        CHECK_EQ(jsdk_joint_sdo_write(fx.j, ro), JSDK_ERR_UNSUPPORTED);
    }

    /* subindex 必须为 0（本协议端点 ID 是平铺的） */
    CHECK_EQ(jsdk_joint_sdo_create(fx.j, 242u, 1u, 4u), -1);
    CHECK_EQ(jsdk_joint_sdo_create(fx.j, 242u, 0u, 9u), -1);       /* 宽度上限 8 */

    /* 非法句柄 */
    CHECK_EQ(jsdk_joint_sdo_state(fx.j, 0), JSDK_SDO_ERROR);
    CHECK_EQ(jsdk_joint_sdo_state(fx.j, 99), JSDK_SDO_ERROR);
    CHECK(jsdk_joint_sdo_data(fx.j, 99) == NULL);

    printf("      create/read/write/state ok; read-only and bad handles rejected\n");
    fx_close(&fx);
}

/* ==========================================================================
 * 4. 节点发现（WP5）
 * ======================================================================== */

static void test_discover(void)
{
    fix_t fx;
    uint8_t ids[16];
    unsigned found = 0u;

    printf("[4] node discovery\n");

    /* 4 节点总线（SIM_MAX_NODES = 4），心跳 10 ms → 被动发现能全部看到 */
    if (fx_open(&fx, "0:id=1,hb=10,fd;1:id=2,hb=10,fd;"
                     "2:id=3,hb=10,fd;3:id=4,hb=10,fd", 1u) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    if (fx_configure(&fx) != 0) { g_fail++; g_checks++; fx_close(&fx); return; }

    /* 被动：先把心跳攒起来（本 HAL 的 now_ms 会自动推进时间） */
    {
        unsigned i;
        for (i = 0u; i < 40u; ++i) fx_cycle(&fx);
    }

    CHECK_EQ(jsdk_context_discover(fx.ctx, ids, 16u, &found, 0u), JSDK_OK);
    CHECK_EQ(found, 4u);
    {
        unsigned i; int seen[5] = { 0, 0, 0, 0, 0 };
        for (i = 0u; i < found; ++i) {
            CHECK(ids[i] >= 1u && ids[i] <= 4u);
            seen[ids[i]] = 1;
        }
        for (i = 1u; i <= 4u; ++i) CHECK(seen[i] == 1);
    }
    printf("      passive: found %u node(s) from heartbeats\n", found);

    /* 主动探测：关掉心跳后仍应找到（靠 QUERY_STATUS 轮询） */
    {
        unsigned n, i;
        for (n = 0u; n < fx.sim->n_nodes; ++n) fx.sim->nodes[n].heartbeat_rate_ms = 0u;
        /* 排空队列里残留的心跳 */
        for (i = 0u; i < 100u; ++i) fx_cycle(&fx);

        found = 0u;
        CHECK_EQ(jsdk_context_discover(fx.ctx, ids, 16u, &found, 4u), JSDK_OK);
        /* 4 个节点都必须被发现 */
        CHECK_EQ(found, 4u);
        printf("      active probe (heartbeats off): found %u node(s)\n", found);
    }

    /* 使能中禁止发现 */
    {
        jsdk_joint_request_enable(fx.j, JSDK_MODE_MIT);
        {
            unsigned i;
            for (i = 0u; i < 20u && !jsdk_joint_is_enabled(fx.j); ++i) fx_cycle(&fx);
        }
        CHECK_EQ(jsdk_joint_is_enabled(fx.j), 1);
        found = 99u;
        CHECK_EQ(jsdk_context_discover(fx.ctx, ids, 16u, &found, 4u),
                 JSDK_ERR_BAD_STATE);
        CHECK_EQ(found, 0u);
        printf("      refused while enabled\n");
    }

    fx_close(&fx);
}

/**
 * 5. 运维操作（WP5）
 * ======================================================================== */

static void test_ops(void)
{
    fix_t fx;
    unsigned i;

    printf("[5] operations: zero / calibrate / home / save / node-id / watchdog\n");

    if (fx_open(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                     "kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    if (fx_configure(&fx) != 0) { g_fail++; g_checks++; fx_close(&fx); return; }

    /* --- 零点：发 0x61 并**读回位置**确认真的归零了 --- */
    for (i = 0u; i < 5u; ++i) fx_cycle(&fx);
    CHECK_EQ(jsdk_joint_set_zero_here(fx.j), JSDK_OK);
    printf("      set_zero_here: %s\n", jsdk_context_last_error(fx.ctx));

    /* --- 标定：使能中必须被拒（§6.4：标定要求 IDLE） --- */
    {
        jsdk_joint_request_enable(fx.j, JSDK_MODE_MIT);
        for (i = 0u; i < 20u && !jsdk_joint_is_enabled(fx.j); ++i) fx_cycle(&fx);
        CHECK_EQ(jsdk_joint_is_enabled(fx.j), 1);

        CHECK_EQ(jsdk_joint_calibrate(fx.j), JSDK_ERR_BAD_STATE);
        CHECK(strstr(jsdk_context_last_error(fx.ctx), "disabled") != NULL);

        /* 失能后可以标定 */
        jsdk_joint_request_disable(fx.j);
        for (i = 0u; i < 20u && jsdk_joint_is_enabled(fx.j); ++i) fx_cycle(&fx);
        CHECK_EQ(jsdk_joint_is_enabled(fx.j), 0);
        printf("      calibrate refused while enabled (correct), enabled -> disabled\n");

        drain_tx(&fx);
        CHECK_EQ(jsdk_joint_calibrate(fx.j), JSDK_OK);
        printf("      calibrate: %s\n", jsdk_context_last_error(fx.ctx));
        /* 标定期间**不得**发出任何控制帧 */
        CHECK_EQ(count_tx(&fx, CB_MSG_MIT_CONTROL, 0), 0u);
        CHECK_EQ(count_tx(&fx, CB_MSG_POS_CONTROL, 0), 0u);
        CHECK_EQ(count_tx(&fx, CB_MSG_CURRENT_CONTROL, 0), 0u);
        /* 状态必须真的走完：current_state 回到 IDLE */
        CHECK_EQ(fx.j->current_state_raw, SIM_AS_IDLE);
        /*
         * ⚠ 这两条是**等待判据的牙齿**：模型现在按真机报**子状态**
         *   （4 电机标定 → 7 索引搜索），**从不等于请求值 3**；如果判据退回
         *   “current_state == 3”，`jsdk_joint_calibrate()` 就会超时 → 上面那条
         *   `JSDK_OK` 断言直接失败。
         * 同时“跑完 ≠ 生效”：两个 pre_calibrated 必须真的被置上。
         */
        CHECK_EQ(fx.sim->nodes[0].motor_pre_cal, 1u);
        CHECK_EQ(fx.sim->nodes[0].enc_pre_cal, 1u);
        {
            jsdk_value_t v;
            CHECK_EQ(jsdk_joint_param_get(fx.j, "axis0.motor.config.pre_calibrated",
                                          &v), JSDK_OK);
            CHECK_EQ(v.v.boolean, 1);
            CHECK_EQ(jsdk_joint_param_get(fx.j, "axis0.encoder.config.pre_calibrated",
                                          &v), JSDK_OK);
            CHECK_EQ(v.v.boolean, 1);
        }
        printf("      calibrate finished: current_state=%u (%s)\n",
               (unsigned)fx.j->current_state_raw,
               jsdk_can_axis_state_name(fx.j->current_state_raw));
    }

    /* --- 回零：写 requested_state = 11，等它离开瞬时态 --- */
    {
        drain_tx(&fx);
        CHECK_EQ(jsdk_joint_home(fx.j), JSDK_OK);
        CHECK_EQ(fx.j->current_state_raw, SIM_AS_CLOSED_LOOP);
        printf("      home finished: current_state=%u (%s)\n",
               (unsigned)fx.j->current_state_raw,
               jsdk_can_axis_state_name(fx.j->current_state_raw));
    }

    /*
     * --- 预算旋钮：`cfg.state_timeout_ms` 必须真的生效 ---
     * 真机上全标定要 29.5 s（旧的硬编码 20 s 会在序列**还在跑**时报超时），
     * 而客户也可能把预算调小。这里给 1 ms：必须报**超时**，且提示是
     * “did not finish”（已经开始了），不是“never left idle”。
     */
    {
        /* ⚠ 要改的是**上下文里已复制的那份**（`jsdk_context_init()` 会把 cfg
           拷贝进去），改 `fx.cfg` 对运行中的上下文不起作用。 */
        uint32_t keep = fx.ctx->cfg.state_timeout_ms;

        fx.ctx->cfg.state_timeout_ms = 100u;   /* < 第一段子状态的 2000 ms */
        drain_tx(&fx);
        CHECK_EQ(jsdk_joint_calibrate(fx.j), JSDK_ERR_TIMEOUT);
        /* 预算太短时**也不能**胡乱归因：这里已经看到过程开始了，
           所以必须是 “did not finish”，而不是 “never left idle”。 */
        CHECK(strstr(jsdk_context_last_error(fx.ctx), "did not finish") != NULL);
        printf("      state_timeout_ms=100 -> timeout: %s\n",
               jsdk_context_last_error(fx.ctx));
        fx.ctx->cfg.state_timeout_ms = keep;
    }

    /* --- 保存配置：无应答 → 用读回校验 --- */
    CHECK_EQ(jsdk_joint_save_config(fx.j), JSDK_OK);
    printf("      save_config verified: %s\n", jsdk_context_last_error(fx.ctx));

    /* --- 看门狗超时：写端点 73 并读回校验 --- */
    CHECK_EQ(jsdk_joint_set_watchdog_ms(fx.j, 250u), JSDK_OK);
    CHECK_EQ(fx.j->break_timeout_ms, 250u);
    CHECK_EQ(fx.sim->nodes[0].break_timeout, 250u);
    CHECK((fx.j->status_flags & JSDK_JF_WATCHDOG_UNVERIFIED) == 0u);
    printf("      set_watchdog_ms(250) verified by read-back\n");

    /* 0 ms = **关闭**设备侧超时检测（新固件语义；旧固件把 0 当 100 ms 且无法关闭）。
       这里必须“真的关掉且校验通过”，而不是发个“0 不等于关闭”的警告。 */
    CHECK_EQ(jsdk_joint_set_watchdog_ms(fx.j, 0u), JSDK_OK);
    CHECK_EQ(fx.j->break_timeout_ms, 0u);                  /* 读回一致 = 确认已关闭 */
    CHECK_EQ(fx.sim->nodes[0].break_timeout, 0u);
    CHECK((fx.j->status_flags & JSDK_JF_WATCHDOG_UNVERIFIED) == 0u);
    CHECK(strstr(jsdk_context_last_error(fx.ctx), "DISABLED on the device") != NULL);
    printf("      set_watchdog_ms(0) -> 关闭并读回确认: %s\n",
           jsdk_context_last_error(fx.ctx));
    CHECK_EQ(jsdk_joint_set_watchdog_ms(fx.j, 250u), JSDK_OK);   /* 复原 */

    /* 超出 u16 必须拒绝（静默截断会让客户以为设成了更大的值） */
    CHECK_EQ(jsdk_joint_set_watchdog_ms(fx.j, 70000u), JSDK_ERR_INVALID_ARG);

    /* --- 改节点号：新地址必须能应答，否则不改本地 id --- */
    CHECK_EQ(jsdk_joint_set_node_id(fx.j, 2u, 0), JSDK_OK);
    CHECK_EQ(fx.j->cfg.node_id, 2u);
    CHECK_EQ(fx.sim->nodes[0].node_id, 2u);
    CHECK_EQ(jsdk_ctx_find_joint(fx.ctx, 2u), fx.j);      /* 查找跟着走 */
    CHECK(jsdk_ctx_find_joint(fx.ctx, 1u) == NULL);
    printf("      set_node_id: %s\n", jsdk_context_last_error(fx.ctx));

    /* 冲突检测：本总线上其它关节已占用该号时要拒绝 */
    {
        fix_t gx;
        if (fx_open(&gx, "0:id=1,timeout=30000,fd;1:id=2,timeout=30000,fd", 2u) == 0) {
            if (fx_configure(&gx) == 0) {
                CHECK_EQ(jsdk_joint_set_node_id(&gx.ctx->joints[0], 2u, 0),
                         JSDK_ERR_INVALID_ARG);
                CHECK_EQ(jsdk_joint_set_node_id(&gx.ctx->joints[0], 0u, 0),
                         JSDK_ERR_INVALID_ARG);
                CHECK_EQ(jsdk_joint_set_node_id(&gx.ctx->joints[0], 255u, 0),
                         JSDK_ERR_INVALID_ARG);
                printf("      set_node_id rejects conflicts and out-of-range ids\n");
            }
            fx_close(&gx);
        }
    }

    /*
     * 冲突检测的另一半：目标号被**总线上的其它设备**占用，但那个设备不在本
     * 上下文里（CLI/Python 只加自己关心的关节时就长这样）。
     * 这里只有 1 个关节、总线上却有 2 台设备，所以「本上下文内冲突」检查看不到它。
     * 若不拦，总线上会出现两个同号设备，而“新地址能应答”的验证还会被原来那台
     * 设备满足 → 报成功。
     */
    {
        fix_t hx;
        if (fx_open(&hx, "0:id=1,timeout=30000,fd;1:id=2,timeout=30000,fd", 1u) == 0) {
            if (fx_configure(&hx) == 0) {
                CHECK_EQ(hx.ctx->nj, 1u);            /* 上下文里只有 node 1 */
                CHECK_EQ(hx.sim->nodes[1].node_id, 2u);  /* 但总线上确实有 node 2 */
                CHECK_EQ(jsdk_joint_set_node_id(hx.j, 2u, 0), JSDK_ERR_INVALID_ARG);
                CHECK(strstr(jsdk_context_last_error(hx.ctx), "already answers") != NULL);
                CHECK_EQ(hx.sim->nodes[0].node_id, 1u);  /* 设备没被改号 */

                /* 而一个**空闲**的号仍然能改过去 */
                CHECK_EQ(jsdk_joint_set_node_id(hx.j, 9u, 0), JSDK_OK);
                CHECK_EQ(hx.sim->nodes[0].node_id, 9u);

                /* 改成自己当前的号：探测会撞上自己 → 必须跳过探测 */
                CHECK_EQ(jsdk_joint_set_node_id(hx.j, 9u, 0), JSDK_OK);
                printf("      set_node_id refuses an id held by another device on the bus\n");
            }
            fx_close(&hx);
        }
    }

    /* --- 复位：本地状态必须被清干净（否则会拿旧状态判断新设备） --- */
    fx.j->first_frame_done = 1u;
    fx.j->current_state_raw = 8u;
    fx.j->state_known = 1u;
    CHECK_EQ(jsdk_joint_reset_device(fx.j), JSDK_OK);
    CHECK_EQ(fx.j->first_frame_done, 0u);
    CHECK_EQ(fx.j->state_known, 0u);
    CHECK_EQ(fx.j->enabled, 0u);
    CHECK((fx.j->status_flags & JSDK_JF_SCALE_INVALID) != 0);
    printf("      reset_device cleared local state and re-handshook\n");

    fx_close(&fx);
}

/* ==========================================================================
 * 6. WP4：反馈新鲜度与链路健康
 * ======================================================================== */

static unsigned g_fault_cb_hits;

/** 故障回调（RT 安全上下文：只计数，不打印、不加锁） */
static void faultcb_count(jsdk_joint_t *j, const jsdk_fault_info_t *info, void *user)
{
    (void)j; (void)info; (void)user;
    g_fault_cb_hits++;
}

/* ==========================================================================
 * 5b. jsdk_context_activate()（v0.14 新增覆盖）
 *
 * 这一段是**补漏**：整套测试此前**从未调用** activate()，于是它里面藏着一个
 * 会让"使能"永久卡死的缺陷（见下面两个用例的注释）。
 * ==========================================================================
 */

static void test_activate(void)
{
    printf("[5b] jsdk_context_activate()\n");

    /* --- 1. 纯 activate()：必须真的把关节驱起来（含安全首帧） --- */
    {
        fix_t fx;
        int rc;

        if (fx_open(&fx, "0:id=1,gear=16.5,tconst=0.0385,pmax=12.5,vmax=65,"
                         "tmax=50,kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
            printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
        }
        if (fx_configure(&fx) != 0) { g_fail++; g_checks++; fx_close(&fx); return; }

        rc = jsdk_context_activate(fx.ctx);
        if (rc != JSDK_OK) {
            printf("      FATAL: activate -> %s (%s)\n", jsdk_status_string((jsdk_status_t)rc),
                   jsdk_context_last_error(fx.ctx));
            g_fail++; g_checks++; fx_close(&fx); return;
        }

        CHECK(jsdk_joint_is_enabled(fx.j));
        /* ⚠ 只有**安全首帧真的发出去了**，tx_active / first_frame_done 才为真。
           旧实现的 activate() 自己发 CLEAR_ERRORS/START_MOTOR 然后干等，
           从不跑 L3 序列 —— 结果是 is_enabled() 变真、但安全首帧根本没发，
           设备还停在默认的 POSITION 输入模式（就是 WP3 修过的"使能瞬间大跳变"，
           只不过在 activate() 这条路上又活了一次）。 */
        CHECK_EQ(fx.ctx->joints[0].tx_active, 1u);
        CHECK_EQ(fx.ctx->joints[0].first_frame_done, 1u);
        CHECK_EQ(fx.ctx->joints[0].enable_pending, 0u);
        /* 设备侧真的被切到 MIT 输入模式（安全首帧把它从默认 POSITION 切过来）。
           注意固件的 InputMode 与 ModeState nibble **不是**同一个东西：
           这里看的是 `input_mode`（固件只在收到帧时才改它）。 */
        CHECK_EQ(fx.sim->nodes[0].input_mode, SIM_INPUT_MODE_MIT);

        printf("      activate() drives the joint: enabled, tx_active, "
               "first frame sent, device in MIT input mode\n");
        fx_close(&fx);
    }

    /* --- 2. 混用：先 request_enable() 再 activate() ---
       这是最自然的客户写法（"我想使能，然后确认就绪"），也是头文件承诺的
       "activate() 等价于逐个 request_enable + 等待就绪"。
       旧实现会**永久卡死**：request_enable() 置 enable_pending = 1，
       而 enabled 被 !enable_pending 门控，activate() 又不跑 cycle_end，
       于是 pending 永远清不掉 → 一路超时。 */
    {
        fix_t fx;
        int rc;

        if (fx_open(&fx, "0:id=1,gear=16.5,tconst=0.0385,pmax=12.5,vmax=65,"
                         "tmax=50,kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
            g_fail++; g_checks++; return;
        }
        if (fx_configure(&fx) != 0) { g_fail++; g_checks++; fx_close(&fx); return; }

        jsdk_joint_request_enable(fx.j, JSDK_MODE_MIT);
        CHECK_EQ(fx.ctx->joints[0].enable_pending, 1u);

        rc = jsdk_context_activate(fx.ctx);
        if (rc != JSDK_OK) {
            printf("      FATAL: enable()+activate() -> %s (%s)\n",
                   jsdk_status_string((jsdk_status_t)rc),
                   jsdk_context_last_error(fx.ctx));
            g_fail++; g_checks++; fx_close(&fx); return;
        }
        CHECK(jsdk_joint_is_enabled(fx.j));
        CHECK_EQ(fx.ctx->joints[0].enable_pending, 0u);
        printf("      request_enable() + activate() 不冲突（只有一份使能实现）\n");
        fx_close(&fx);
    }

    /* --- 3. 未标定 → 明确拒绝，且**不改任何状态** --- */
    {
        fix_t fx;

        if (fx_open(&fx, "0:id=1,timeout=30000,fd", 1u) != 0) {
            g_fail++; g_checks++; return;
        }
        /* 不 configure，直接 activate */
        CHECK_EQ(jsdk_context_activate(fx.ctx), JSDK_ERR_BAD_STATE);
        CHECK(strstr(jsdk_context_last_error(fx.ctx), "calibration") != NULL);
        CHECK(!jsdk_joint_is_enabled(fx.j));
        CHECK_EQ(fx.ctx->joints[0].enable_pending, 0u);   /* 没有被改坏 */
        printf("      no calibration -> refused before touching anything\n");
        fx_close(&fx);
    }

    /* --- 4. 幂等：已使能时再调一次不应报错、也不重复发序列 --- */
    {
        fix_t fx;
        uint32_t tx_before;

        if (fx_open(&fx, "0:id=1,gear=16.5,tconst=0.0385,pmax=12.5,vmax=65,"
                         "tmax=50,kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
            g_fail++; g_checks++; return;
        }
        if (fx_configure(&fx) != 0) { g_fail++; g_checks++; fx_close(&fx); return; }
        CHECK_EQ(jsdk_context_activate(fx.ctx), JSDK_OK);
        drain_tx(&fx);
        tx_before = fx.ctx->bus.tx_frames;

        CHECK_EQ(jsdk_context_activate(fx.ctx), JSDK_OK);
        CHECK(fx.ctx->bus.tx_frames >= tx_before);   /* 不要求零帧，但不该爆量 */
        CHECK(fx.ctx->bus.tx_frames - tx_before < 10u);
        printf("      second activate() is a no-op-ish retry (%u frame(s))\n",
               (unsigned)(fx.ctx->bus.tx_frames - tx_before));
        fx_close(&fx);
    }

    /* --- 5. deactivate() 必须掐掉排队中的使能请求 ---
       缺陷（WP8 self-review 发现，实测复现）：request_enable() 之后不跑周期，
       直接 deactivate()，本函数只清了 disable_pending/seq_step/tx_active，
       `enable_pending` 原样留着；而 advance_seq 的使能分支是**无条件**执行的，
       于是 deactivate() 返回后的第一个 cycle_end 就把电机重新使能了 ——
       "安全关闭"被一个陈旧请求悄悄撤销。而 deactivate() 的所有调用方
       （jsdk-cli stop、Python Context.close()）都把它当成"断电已完成"。 */
    {
        fix_t fx;
        unsigned n;

        if (fx_open(&fx, "0:id=1,gear=16.5,tconst=0.0385,pmax=12.5,vmax=65,"
                         "tmax=50,kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
            g_fail++; g_checks++; return;
        }
        if (fx_configure(&fx) != 0) { g_fail++; g_checks++; fx_close(&fx); return; }

        jsdk_joint_request_enable(fx.j, JSDK_MODE_MIT);   /* 只排队，不跑周期 */
        CHECK_EQ(fx.ctx->joints[0].enable_pending, 1u);

        jsdk_context_deactivate(fx.ctx);
        CHECK_EQ(fx.ctx->joints[0].enable_pending, 0u);   /* ← 缺陷点 */
        CHECK(!jsdk_joint_is_enabled(fx.j));

        /* 关键：再跑一段周期，关节**不能**自己回来 */
        for (n = 0u; n < 30u; ++n) fx_cycle(&fx);
        CHECK(!jsdk_joint_is_enabled(fx.j));
        CHECK_EQ(fx.ctx->joints[0].tx_active, 0u);
        printf("      deactivate() cancels a pending enable (still off after 30 cycles)\n");
        fx_close(&fx);
    }

    /* --- 6. disable() 后再 enable()：enable 必须咬得住 ---
       缺陷（同一轮 review 发现）：request_disable() 会清 enable_pending，
       但 request_enable() 不清 disable_pending，两个标志于是同时在排队。
       advance_seq 先跑使能（使能生效、is_enabled 变真），使能序列收尾后
       disable_pending 仍在 → 紧接着跑失能序列 → 关节在十几拍后**自己断开**，
       且没有任何错误码。现场表现是"我使能了，动了一下就掉使能"。 */
    {
        fix_t fx;
        unsigned n;
        int saw_enabled = 0, saw_disabled_after = 0;

        if (fx_open(&fx, "0:id=1,gear=16.5,tconst=0.0385,pmax=12.5,vmax=65,"
                         "tmax=50,kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
            g_fail++; g_checks++; return;
        }
        if (fx_configure(&fx) != 0) { g_fail++; g_checks++; fx_close(&fx); return; }

        CHECK_EQ(jsdk_context_activate(fx.ctx), JSDK_OK);
        CHECK(jsdk_joint_is_enabled(fx.j));

        jsdk_joint_request_disable(fx.j);                 /* 只排队，不跑周期 */
        jsdk_joint_request_enable(fx.j, JSDK_MODE_MIT);   /* 紧接着又要使能 */
        CHECK_EQ(fx.ctx->joints[0].disable_pending, 0u);  /* ← 缺陷点 */
        CHECK_EQ(fx.ctx->joints[0].enable_pending, 1u);

        for (n = 0u; n < 40u; ++n) {
            fx_cycle(&fx);
            if (jsdk_joint_is_enabled(fx.j)) saw_enabled = 1;
            else if (saw_enabled) { saw_disabled_after = 1; break; }
        }
        CHECK(saw_enabled);
        CHECK(!saw_disabled_after);   /* 不能被残留的 disable 请求反手关掉 */
        CHECK(jsdk_joint_is_enabled(fx.j));
        printf("      disable()+enable(): stays enabled (no stale disable fires)\n");
        fx_close(&fx);
    }
}

/* ==========================================================================
 * 5c. 8 字节参数的“精确读”（WP9）
 *
 * 缺陷背景：`jsdk_ctx_read_param()` 把请求里的 `ReqLen` 写死成 4，而
 * `jsdk_joint_param_get()` 按**描述符类型长度**（u64/i64/f64 = 8）校验，
 * 于是 8 字节端点**永远读不出来**，报的还是“设备只回了 4 字节，描述符说 8 字节”
 * —— 把矛头指向固件/描述符，真正的原因在主站自己的请求里。
 * ======================================================================== */

static void test_wide_param_read(void)
{
    /* 模拟器里 serial_number 是固定常量 0x1122334455667788，可直接断言值 */
    const unsigned long long k_serial = 0x1122334455667788ull;

    printf("[5c] 8-byte parameter read (exact/chunked)\n");

    /* --- 1. FD：一次请求就该拿满 8 字节 --- */
    {
        fix_t fx;
        jsdk_value_t v;
        uint32_t tx_before;
        uint8_t  lens[4], reqlen[4];
        uint32_t offs[4];
        unsigned n_req;

        if (fx_open(&fx, "0:id=1,gear=16.5,tconst=0.0385,pmax=12.5,vmax=65,"
                         "tmax=50,kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
            g_fail++; g_checks++; return;
        }
        if (fx_configure(&fx) != 0) { g_fail++; g_checks++; fx_close(&fx); return; }

        drain_tx(&fx);
        tx_before = fx.ctx->bus.tx_frames;
        memset(&v, 0, sizeof v);
        CHECK_EQ(jsdk_joint_param_get(fx.j, "serial_number", &v), JSDK_OK);
        CHECK_EQ(v.type, JSDK_EP_U64);
        CHECK_EQ(v.v.u64, k_serial);
        /* FD 下一次请求就够：多了就说明在无意义地重复读 */
        CHECK_EQ(fx.ctx->bus.tx_frames - tx_before, 1u);

        /* 请求帧形状：旧式 4 B 形式，但 ReqLen 必须是 8（否则设备只给 4 B）*/
        n_req = take_read_requests(&fx, lens, reqlen, offs, 4u);
        CHECK_EQ(n_req, 1u);
        CHECK_EQ(lens[0], 4u);
        CHECK_EQ(reqlen[0], 8u);
        CHECK_EQ(offs[0], 0u);

        /* 64-bit 错误字（排障最常用的字段之一） */
        memset(&v, 0, sizeof v);
        CHECK_EQ(jsdk_joint_param_get(fx.j, "axis0.motor.error", &v), JSDK_OK);
        CHECK_EQ(v.type, JSDK_EP_U64);
        CHECK_EQ(v.v.u64, 0u);

        printf("      FD: serial_number=0x%016llX, motor.error=0 (1 request each)\n",
               (unsigned long long)k_serial);
        fx_close(&fx);
    }

    /* --- 2. Classic：设备一次只能回 4 字节 → 必须分**两块**读满 --- */
    {
        fix_t fx;
        jsdk_value_t v;
        uint32_t tx_before;
        uint8_t  lens[4], reqlen[4];
        uint32_t offs[4];
        unsigned n_req;

        if (fx_open_flags(&fx, "0:id=1,gear=16.5,tconst=0.0385,pmax=12.5,vmax=65,"
                               "tmax=50,kpmax=500,kdmax=5,hb=10,timeout=30000,"
                               "classic", 1u, 0) != 0) {
            g_fail++; g_checks++; return;
        }
        if (fx_configure(&fx) != 0) { g_fail++; g_checks++; fx_close(&fx); return; }

        drain_tx(&fx);
        tx_before = fx.ctx->bus.tx_frames;
        memset(&v, 0, sizeof v);
        CHECK_EQ(jsdk_joint_param_get(fx.j, "serial_number", &v), JSDK_OK);
        CHECK_EQ(v.v.u64, k_serial);
        /* ⚠ 必须**恰好 2 次**：1 次 = 我们只读了 4 字节却当成功（错），
           3 次以上 = 分块逻辑在空转 */
        CHECK_EQ(fx.ctx->bus.tx_frames - tx_before, 2u);

        /* 两块请求的**帧形状**都要对：第一块旧式 4 B（offset 隐含 0），
           第二块必须带 offset=4，否则设备会从 0 重新开始 */
        n_req = take_read_requests(&fx, lens, reqlen, offs, 4u);
        CHECK_EQ(n_req, 2u);
        CHECK_EQ(lens[0], 4u);      CHECK_EQ(reqlen[0], 4u);  CHECK_EQ(offs[0], 0u);
        CHECK_EQ(lens[1], 8u);      CHECK_EQ(reqlen[1], 4u);  CHECK_EQ(offs[1], 4u);

        /* 4 字节的值在 Classic 下仍是一次搞定（不要“为了统一”多读一块） */
        tx_before = fx.ctx->bus.tx_frames;
        memset(&v, 0, sizeof v);
        CHECK_EQ(jsdk_joint_param_get(fx.j, "axis0.motor.config.gear_ratio", &v),
                 JSDK_OK);
        CHECK_NEAR(v.v.f32, 16.5f, 1e-6);
        CHECK_EQ(fx.ctx->bus.tx_frames - tx_before, 1u);

        printf("      Classic: same value in exactly 2 requests (4 B each), "
               "4 B value in 1\n");
        fx_close(&fx);
    }

    /* --- 3. 短值：设备给的比要的少 → 明确报错，不把截断值当成功 --- */
    {
        fix_t fx;
        uint8_t buf[8];
        uint8_t len = 0xEEu;

        if (fx_open(&fx, "0:id=1,gear=16.5,tconst=0.0385,pmax=12.5,vmax=65,"
                         "tmax=50,kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
            g_fail++; g_checks++; return;
        }
        if (fx_configure(&fx) != 0) { g_fail++; g_checks++; fx_close(&fx); return; }

        /* gear_ratio 是 4 字节，却硬要 8 字节：设备只能说“没有了” */
        CHECK_EQ(jsdk_ctx_read_param_exact(fx.ctx, 1u,
                                           (uint16_t)242u /* gear_ratio */,
                                           buf, 8u, &len, 0u),
                 JSDK_ERR_PROTOCOL);
        CHECK(strstr(jsdk_context_last_error(fx.ctx), "shorter") != NULL);
        printf("      short value -> PROTOCOL with an actionable message\n");
        fx_close(&fx);
    }
}

/**
 * “打开后的前几帧丢了”必须能自愈 —— 真机（slcan）约 1/10 的进程会命中。
 *
 * 现场症状：`configure()` 报 `timeout`、细节是 `descriptor download ... (0/0 bytes)`，
 * 再敲一次就好了。根因不在固件也不在参数解析：**适配器刚打开时主站的头几帧
 * 会被丢掉**，于是描述符请求（= 会话的第一帧）从未到达设备。
 *
 * 这里在 HAL 层注入“丢掉主站前 N 帧”，断言：
 *   - N=1、N=3 → `configure()` 必须**靠重发自愈**成功（且设备侧只收到一个请求）；
 *   - 全丢 → 必须是 1 + 3 = **4 次**尝试后 TIMEOUT，且错误串要能区分
 *     “一帧都没收到”（通道/适配器）与“收到了帧但请求丢”（重发不够 / 设备不应答）。
 *
 * ⚠ 只断言“没报错”是不够的：重发如果错误地**重启设备侧的流**，会出现
 *   “看着成功、其实是第二次传输的数据”这种问题，所以还要数清总线上真正
 *   出现的 0x24 请求个数。
 */
static void test_lost_first_request(void)
{
    unsigned n;

    printf("[7] lost first frame(s) after opening the port -> self-healing\n");

    /* --- 丢 1 / 丢 3 帧：递增间隔的重发（250/600/1200 ms）该救回来 --- */
    for (n = 1u; n <= 3u; n += 2u) {
        fix_t fx;

        if (fx_open(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                         "kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
            printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
        }
        fx.drop_tx_head = n;
        CHECK_EQ(fx_configure(&fx), 0);
        CHECK_EQ(fx.dropped, n);                       /* 确实丢了这么多 */
        CHECK_EQ(fx.desc_reqs_on_bus, 1u);              /* 只有 1 个请求真正到达设备 */
        CHECK_EQ(fx.j->calibrated, 1);
        CHECK_NEAR(fx.j->gear_ratio, 16.0f, 1e-4f);
        /*
         * ⚠ 这 n 帧是**会话预热**吸收掉的（不是描述符重发）：预热探测幂等，
         *   丢一帧就重发一次 ⇒ `tx_retries == n`，而描述符请求因此**一次**就到达设备。
         *   （去掉预热 → 这里会是 0，而描述符重发才去救 —— 用例就是要钉住这个差别。）
         */
        CHECK_EQ(fx.ctx->bus.tx_retries, n);
        CHECK_EQ(jsdk_context_warmup(fx.ctx, 0u), JSDK_OK);   /* 已预热 → 空操作 */
        CHECK_EQ(fx.ctx->bus.tx_retries, n);
        printf("      dropped %u frame(s): warm-up retried, configure() recovered "
               "(1 request reached the device)\n", n);
        fx_close(&fx);
    }

    /* --- 全丢：预热与描述符重发都拿不到响应 → 必须失败，且信息可判定 --- */
    {
        fix_t fx;

        if (fx_open(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                         "kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
            printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
        }
        fx.drop_tx_head = 1000u;
        CHECK_EQ(fx_configure(&fx), -1);
        /*
         * ⚠ “丢了几帧”现在是**时间相关**的（预热每轮 50 ms、预算 500 ms），
         *   所以断言行为而不是精确值：预热重发过，而描述符请求**一次都没**到达设备。
         */
        CHECK(fx.dropped >= 4u);
        CHECK(fx.ctx->bus.tx_retries >= 5u);           /* 预热确实在重发 */
        CHECK_EQ(fx.desc_reqs_on_bus, 0u);
        CHECK(strstr(jsdk_context_last_error(fx.ctx),
                     "stalled: no new bytes") != NULL);
        CHECK(strstr(jsdk_context_last_error(fx.ctx), "frames received") != NULL);
        printf("      everything dropped -> %s\n",
               jsdk_context_last_error(fx.ctx));
        fx_close(&fx);
    }

    /* --- ⚠ RX 里有“上一次传输的残留帧” + 我们的请求丢了 ---
          判据必须是“本次传输还没开始”，不能是“一帧都没收到”：
          后者会被残留帧骗过 → 从不重发 → 超时（真机 1/20 次就是这样）。 */
    {
        fix_t fx;

        if (fx_open(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                         "kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
            printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
        }
        fx.drop_msgtype   = (uint8_t)CB_MSG_JSON_DESC_READ;
        fx.drop_msgtype_n = 1u;          /* 第一个 0x24 丢 */
        fx.inject_stale   = 40u;         /* 同时有残留帧在流 */
        CHECK_EQ(fx_configure(&fx), 0);
        CHECK_EQ(fx.dropped_msgtype, 1u);
        CHECK(fx.injected >= 1u);        /* 残留帧真的出现过 */
        CHECK_EQ(fx.desc_reqs_on_bus, 1u);   /* 重发的那次才到达设备 */
        CHECK_EQ(fx.j->calibrated, 1);
        printf("      stale frames + lost request -> retried anyway "
               "(%u stale frame(s) seen)\n", fx.injected);
        fx_close(&fx);
    }

    /* --- 残留帧 + 请求全丢：超时信息必须反映“收到过帧”（区别于 0 帧） --- */
    {
        fix_t fx;

        if (fx_open(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                         "kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
            printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
        }
        fx.drop_msgtype   = (uint8_t)CB_MSG_JSON_DESC_READ;
        fx.drop_msgtype_n = 1000u;
        fx.inject_stale   = 40u;
        CHECK_EQ(fx_configure(&fx), -1);
        CHECK_EQ(fx.dropped_msgtype, 4u);          /* 1 + 3 次重发 */
        CHECK(strstr(jsdk_context_last_error(fx.ctx), "frames received") != NULL);
        CHECK(strstr(jsdk_context_last_error(fx.ctx), "0 frames received") == NULL);
        printf("      stale-but-no-metadata case reports: %s\n",
               jsdk_context_last_error(fx.ctx));
        fx_close(&fx);
    }

    /* --- 全丢 + 设备不发心跳：这才真的是“一帧都没收到” --- */
    {
        fix_t fx;

        if (fx_open(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                         "kpmax=500,kdmax=5,hb=0,timeout=30000,fd", 1u) != 0) {
            printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
        }
        fx.drop_tx_head = 1000u;
        CHECK_EQ(fx_configure(&fx), -1);
        CHECK(strstr(jsdk_context_last_error(fx.ctx), "0 frames received") != NULL);
        printf("      0-frame case is distinguishable: %s\n",
               jsdk_context_last_error(fx.ctx));
        fx_close(&fx);
    }
}

/**
 * 握手必须**被证实**：设备的 master_id 从收到的帧里学，丢了握手帧 = 整个会话哑掉。
 * 这里按 MsgType 丢掉 `QUERY_STATUS`（握手用的就是它），断言：
 *   - 丢 3 帧 → `configure()` 靠重发自愈；
 *   - 全丢 → TIMEOUT，且**恰好尝试 6 次**（钉住重试次数），错误串说明原因。
 */
static void test_handshake_retry(void)
{
    printf("[8] handshake is verified + retried (lost handshake frames)\n");

    /* --- 丢 3 次握手：必须自愈 --- */
    {
        fix_t fx;

        if (fx_open(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                         "kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
            printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
        }
        fx.drop_msgtype   = (uint8_t)CB_MSG_QUERY_STATUS;
        fx.drop_msgtype_n = 3u;
        CHECK_EQ(fx_configure(&fx), 0);
        CHECK_EQ(fx.dropped_msgtype, 3u);
        CHECK_EQ(fx.j->calibrated, 1);
        CHECK_EQ(fx.j->state_known, 1u);          /* 握手应答被当成首次反馈吃掉了 */
        printf("      3 handshake frames dropped -> configure() recovered\n");
        fx_close(&fx);
    }

    /* --- 握手全丢：必须超时，且尝试次数是固定的 6 次 --- */
    {
        fix_t fx;

        if (fx_open(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                         "kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
            printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
        }
        fx.drop_msgtype   = (uint8_t)CB_MSG_QUERY_STATUS;
        fx.drop_msgtype_n = 1000u;
        CHECK_EQ(fx_configure(&fx), -1);
        CHECK_EQ(fx.dropped_msgtype, 6u);
        CHECK(strstr(jsdk_context_last_error(fx.ctx),
                     "no response to handshake after 6 attempts") != NULL);
        printf("      all handshakes dropped -> %s\n",
               jsdk_context_last_error(fx.ctx));
        fx_close(&fx);
    }
}

/**
 * 看门狗的“读回无法校验”分支（真机 F28）。
 *
 * 真机实测（fw 1545）：`can.config.break_timeout` 的**读回恒为 0** —— 写 250 之后
 * 立刻读（同一进程、`sdo.data` 确认发出去的就是 `FA 00`）读回来的仍是 0；
 * 对照端点 `heartbeat_rate_ms` 的写→读是正常的，所以是该端点的固件问题。
 * 因为 `0` 在固件里的含义是**禁用超时检测**，所以“写 250 读回 0”意味着
 * **客户端无法证明自己武装了保护**。
 *
 * 正确行为：
 *   - 写入非 0 而读回 0 → 保留写入值（安全方向）+ 置 UNVERIFIED 位 + 明确说明（OK）；
 *   - **读回一个不同的非 0 值**属于真矛盾 → 仍必须 PROTOCOL（这条不能放松）；
 *   - 写 0 而读回 0 → **是真的关闭了**，校验通过、不得置位。
 */
static void test_watchdog_readback(void)
{
    fix_t fx;
    jsdk_joint_config_snapshot_t snap;

    printf("[9] watchdog read-back: F28（读回恒 0 不可校）vs 真不一致 vs 0=关闭\n");

    if (fx_open(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                     "kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    if (fx_configure(&fx) != 0) { g_fail++; g_checks++; fx_close(&fx); return; }
    CHECK(fx.j->ep_break_timeout != 0u);

    /* --- 读回恒 0：写入被接受，但不假装校验通过 --- */
    fx.patch_read_ep    = fx.j->ep_break_timeout;
    fx.patch_read_value = 0u;
    CHECK_EQ(jsdk_joint_set_watchdog_ms(fx.j, 250u), JSDK_OK);
    CHECK(strstr(jsdk_context_last_error(fx.ctx), "could not verify") != NULL);
    CHECK((fx.j->status_flags & JSDK_JF_WATCHDOG_UNVERIFIED) != 0u);
    CHECK_EQ(jsdk_joint_read_config_snapshot(fx.j, &snap), JSDK_OK);
    CHECK_EQ(snap.break_timeout_ms, 250u);      /* 保留写入值，不假装 0 */
    CHECK(strstr(jsdk_context_last_error(fx.ctx), "read back as 0") == NULL);
    printf("      read-back 0（F28）→ OK + “未校验”提示 + UNVERIFIED 位\n");

    /* --- 写 0 = 关闭，读回 0 = 真的关上了 → 校验通过（不得置 UNVERIFIED）--- */
    CHECK_EQ(jsdk_joint_set_watchdog_ms(fx.j, 0u), JSDK_OK);
    CHECK((fx.j->status_flags & JSDK_JF_WATCHDOG_UNVERIFIED) == 0u);
    CHECK_EQ(fx.j->break_timeout_ms, 0u);
    CHECK_EQ(jsdk_watchdog_device_ms(fx.j), JSDK_WD_DISABLED_MS);
    printf("      写 0（= 禁用）→ 校验通过，无 UNVERIFIED\n");

    /* --- 读回一个不同的非 0 值：真不一致，必须报 --- */
    fx.patch_read_value = 999u;
    CHECK_EQ(jsdk_joint_set_watchdog_ms(fx.j, 250u), JSDK_ERR_PROTOCOL);
    CHECK(strstr(jsdk_context_last_error(fx.ctx), "read back as 999") != NULL);
    printf("      read-back 999 → PROTOCOL（不一致仍不放过）\n");

    fx_close(&fx);
}

/**
 * `break_timeout == 0`（= 禁用）时的两条硬规则：
 *
 * 1. **不得把“没有门限”当成“门限为 0”**：`period_ms >= wd` 在 wd=0 时**恒真**，
 *    若不用 `JSDK_WD_DISABLED_MS` 显式跳过，任何跑循环的命令都会被 configure() 拒掉；
 * 2. **无狗可喂**：禁用时不补 keepalive 帧（否则只会白白增加总线流量，
 *    还会掩盖“你没在发控制帧”这个事实）。
 *
 * 旧实现在这两点上都错（它把 0 归一成 100 ms 掩盖了第 1 条，第 2 条则是多余的喂狗）。
 */
static void test_watchdog_disabled(void)
{
    fix_t fx;
    unsigned i;
    jsdk_joint_feedback_t fb;

    printf("[9b] break_timeout = 0（禁用）：不拒循环、不补喂\n");

    /* 显式 `timeout=0`：模拟新固件的默认状态（超时检测关着） */
    if (fx_open(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                     "kpmax=500,kdmax=5,hb=10,timeout=0,fd", 1u) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }

    /* 控制周期设成 100 ms，远大于“如果 0 被当成 100 ms”那个值 —— 仍然必须能配置 */
    fx.ctx->cfg.period_ns = 100000000u;
    CHECK_EQ(fx_configure(&fx), 0);
    CHECK_EQ(jsdk_watchdog_device_ms(fx.j), JSDK_WD_DISABLED_MS);
    CHECK_EQ(fx.j->break_timeout_ms, 0u);
    printf("      周期 100 ms + timeout=0 → configure() 通过（不再拿 0 当门限）\n");

    /* 使能并跑一阵：设备侧没有门限，所以**不应**产生 CAN_BUS_FAILED */
    jsdk_joint_request_enable(fx.j, JSDK_MODE_MIT);
    for (i = 0u; i < 30u && !jsdk_joint_is_enabled(fx.j); ++i) fx_cycle(&fx);
    CHECK_EQ(jsdk_joint_is_enabled(fx.j), 1);

    for (i = 0u; i < 40u; ++i) fx_cycle(&fx);
    CHECK_EQ(fx.sim->nodes[0].error_axis & SIM_ERR_CAN_BUS_FAILED, 0u);
    printf("      跑 40 个 100 ms 周期：设备未被停（error_axis=0x%X）\n",
           (unsigned)fx.sim->nodes[0].error_axis);

    /* 切到 CURRENT（不是 is_ctrl）：禁用状态下**不得**补 MIT keepalive */
    jsdk_joint_set_mode(fx.j, JSDK_MODE_CURRENT);
    jsdk_joint_set_current_A(fx.j, 0.0);
    for (i = 0u; i < 10u; ++i) fx_cycle(&fx);
    {
        uint32_t before = fx.j->keepalive_sent;
        for (i = 0u; i < 20u; ++i) fx_cycle(&fx);
        CHECK_EQ(fx.j->keepalive_sent, before);            /* 一帧都不补 */
        CHECK_EQ(jsdk_joint_get_feedback(fx.j, &fb), JSDK_OK);
        CHECK_EQ((fb.status_flags & JSDK_JF_WATCHDOG_RISK), 0u);
    }
    printf("      超时禁用 ⇒ 0 帧 keepalive、不置 WATCHDOG_RISK\n");

    fx_close(&fx);
}

/**
 * **短值写入**（bool / u8 / u16）必须真的生效。
 *
 * ⚠⚠ 这里曾经有个藏了很久的真缺陷：固件 `cmd_param_write()` 首句是
 *   `if (msg.len < 8) return;`（整帧丢弃、连 ACK 都不回），而发送侧按
 *   `4 + value_len` 组帧 ⇒ bool/u8 = 5 B、u16 = **6 B** 全部被丢掉；
 *   只有 u32/f32 刚好 8 B 能生效 —— 所以“写参数能用”这个印象是假的。
 *   真机表现：`write <bool> 1` 报成功、再读还是旧值；`can.config.break_timeout`
 *   （u16）写什么都没反应，还一度被误判成固件问题（FIRMWARE_ISSUES F28）。
 *
 * 现在发送侧一律补齐到 8 字节，**仿真设备也复刻了同一门限**，
 * 所以这条用例在缺了补齐时会红。
 */
static void test_write_short_values(void)
{
    fix_t fx;
    jsdk_value_t v;

    printf("[11] 短值写入：bool / u8 / u16 必须真的进设备（帧长 ≥ 8）\n");

    if (fx_open(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                     "kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    if (fx_configure(&fx) != 0) { g_fail++; g_checks++; fx_close(&fx); return; }

    /* --- u16（端点 73，也是 F28 那个端点）--- */
    v.type = JSDK_EP_U16; v.v.u16 = 250u;
    CHECK_EQ(jsdk_joint_param_set(fx.j, "can.config.break_timeout", &v), JSDK_OK);
    CHECK_EQ(fx.sim->nodes[0].break_timeout, 250u);        /* 设备侧真的变了 */
    CHECK_EQ(jsdk_joint_param_get_u32(fx.j, "can.config.break_timeout", &v.v.u32),
             JSDK_OK);
    CHECK_EQ(v.v.u32, 250u);
    printf("      u16 break_timeout -> %u（设备侧已生效）\n",
           (unsigned)fx.sim->nodes[0].break_timeout);

    /* --- bool（端点 154）--- */
    v.type = JSDK_EP_BOOL; v.v.boolean = 0;
    CHECK_EQ(jsdk_joint_param_set(fx.j, "axis0.config.enable_watchdog", &v), JSDK_OK);
    CHECK_EQ(fx.sim->nodes[0].enable_watchdog, 0u);
    v.v.boolean = 1;
    CHECK_EQ(jsdk_joint_param_set(fx.j, "axis0.config.enable_watchdog", &v), JSDK_OK);
    CHECK_EQ(fx.sim->nodes[0].enable_watchdog, 1u);
    printf("      bool enable_watchdog 0 -> 1（设备侧已生效）\n");

    /* --- u8（端点 287 control_mode）--- */
    v.type = JSDK_EP_U8; v.v.u8 = 3u;
    CHECK_EQ(jsdk_joint_param_set(fx.j, "axis0.controller.config.control_mode", &v),
             JSDK_OK);
    CHECK_EQ(fx.sim->nodes[0].control_mode, 3u);
    printf("      u8 control_mode -> %u（设备侧已生效）\n",
           (unsigned)fx.sim->nodes[0].control_mode);

    fx_close(&fx);
}

static void test_robustness(void)
{
    fix_t fx;
    unsigned i;

    printf("[10] robustness: feedback freshness / link health / fault edge\n");
    if (fx_open(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                     "kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    if (fx_configure(&fx) != 0) { g_fail++; g_checks++; fx_close(&fx); return; }

    /* 阈值由观测推导：心跳 10 ms → 3×10 = 30；控制周期 1 ms → 6×1 = 6；下限 50
       ⇒ 取 50 ms。客户不需要为它调参。 */
    CHECK_EQ(jsdk_joint_stale_ms(fx.j), 50u);
    printf("      stale threshold derived = %u ms\n", (unsigned)jsdk_joint_stale_ms(fx.j));

    /* 心跳让反馈保持新鲜 */
    for (i = 0u; i < 20u; ++i) fx_cycle(&fx);
    {
        jsdk_joint_feedback_t fb;
        CHECK_EQ(jsdk_joint_get_feedback(fx.j, &fb), JSDK_OK);
        CHECK_EQ(fb.online, 1);
        CHECK((fb.status_flags & JSDK_JF_FEEDBACK_STALE) == 0);
        CHECK(fb.age_ms < 50u);
    }

    /* 把设备的心跳**彻底关掉**，再收 5 个控制周期（这样 age 才会单调增长；
       只是"不跑循环"的话帧会堆在 RX 环里，下一周期一次性收上来，
       age 立刻回到 0，反而看不出问题）。 */
    {
        jsdk_joint_feedback_t fb;
        unsigned n;

        fx.sim->nodes[0].heartbeat_rate_ms = 0u;
        drain_tx(&fx);

        for (n = 0u; n < 120u; ++n) {
            fx_cycle(&fx);
            if ((fx.j->status_flags & JSDK_JF_FEEDBACK_STALE) != 0) break;
        }
        CHECK_EQ(jsdk_joint_get_feedback(fx.j, &fb), JSDK_OK);
        CHECK((fb.status_flags & JSDK_JF_FEEDBACK_STALE) != 0);
        CHECK(fb.age_ms > jsdk_joint_stale_ms(fx.j));
        printf("      heartbeats off: age_ms=%u > threshold %u, "
               "FEEDBACK_STALE set after %u cycles\n",
               (unsigned)fb.age_ms, (unsigned)jsdk_joint_stale_ms(fx.j), n + 1u);

        /* 粘滞：恢复后要显式清除，现场才能看到"曾经掉过" */
        fx.sim->nodes[0].heartbeat_rate_ms = 10u;
        for (n = 0u; n < 10u; ++n) fx_cycle(&fx);
        CHECK_EQ(jsdk_joint_get_feedback(fx.j, &fb), JSDK_OK);
        CHECK((fb.status_flags & JSDK_JF_FEEDBACK_STALE) != 0);   /* 仍粘滞 */
        jsdk_joint_clear_status_flags(fx.j, JSDK_JF_FEEDBACK_STALE);
        fx_cycle(&fx);
        CHECK_EQ(jsdk_joint_get_feedback(fx.j, &fb), JSDK_OK);
        CHECK((fb.status_flags & JSDK_JF_FEEDBACK_STALE) == 0);
        printf("      flag is sticky until cleared explicitly\n");
    }

    /* 发送失败 → tx_failed 与 JSDK_JF_TX_FAILED */
    {
        jsdk_bus_state_t bs;
        jsdk_joint_feedback_t fb;

        jsdk_joint_request_enable(fx.j, JSDK_MODE_MIT);
        for (i = 0u; i < 20u && !jsdk_joint_is_enabled(fx.j); ++i) fx_cycle(&fx);
        CHECK_EQ(jsdk_joint_is_enabled(fx.j), 1);

        jsdk_hal_virtual_set_tx_fail(fx.h, -1);       /* 后续全部发送失败 */
        for (i = 0u; i < 3u; ++i) fx_cycle(&fx);

        CHECK_EQ(jsdk_context_get_bus_state(fx.ctx, &bs), JSDK_OK);
        CHECK(bs.tx_failed > 0u);
        CHECK_EQ(jsdk_joint_get_feedback(fx.j, &fb), JSDK_OK);
        CHECK((fb.status_flags & JSDK_JF_TX_FAILED) != 0);
        printf("      tx failure recorded: tx_failed=%u, JSDK_JF_TX_FAILED set\n",
               (unsigned)bs.tx_failed);

        jsdk_hal_virtual_set_tx_fail(fx.h, 0);        /* 恢复 */
        for (i = 0u; i < 5u; ++i) fx_cycle(&fx);
        CHECK_EQ(jsdk_context_get_bus_state(fx.ctx, &bs), JSDK_OK);
        CHECK_EQ(bs.link_errors, 0u);                 /* 虚拟后端不上报 recv 错误 */
        CHECK_EQ(bs.nodes_online, 1u);
    }

    /* 故障边沿：回调只在 0→1 时触发一次，且错误串必须给出恢复路径 */
    {
        jsdk_joint_feedback_t fb;
        uint32_t saved_endpoint[2];

        g_fault_cb_hits = 0u;
        jsdk_context_set_fault_callback(fx.ctx, faultcb_count, NULL);

        /* 注入设备侧 CAN_BUS_FAILED（模拟协议级超时） */
        fx.sim->nodes[0].error_axis |= SIM_ERR_CAN_BUS_FAILED;
        saved_endpoint[0] = fx.sim->nodes[0].error_axis;
        (void)saved_endpoint;

        for (i = 0u; i < 3u; ++i) fx_cycle(&fx);

        CHECK_EQ(jsdk_joint_is_fault(fx.j), 1);
        CHECK_EQ(g_fault_cb_hits, 1u);                        /* 边沿 → 只一次 */
        CHECK(strstr(jsdk_context_last_error(fx.ctx), "recovery") != NULL);
        CHECK(strstr(jsdk_context_last_error(fx.ctx), "request_fault_reset") != NULL);
        CHECK_EQ(jsdk_joint_get_feedback(fx.j, &fb), JSDK_OK);
        CHECK((fb.status_flags & JSDK_JF_FEEDBACK_STALE) == 0);
        printf("      fault edge: cb=%u, err=\"%s\"\n", g_fault_cb_hits,
               jsdk_context_last_error(fx.ctx));

        /* 再跑几轮不应重复回调 */
        for (i = 0u; i < 3u; ++i) fx_cycle(&fx);
        CHECK_EQ(g_fault_cb_hits, 1u);

        /* 恢复路径：request_fault_reset → 故障清除 */
        jsdk_joint_request_fault_reset(fx.j);
        for (i = 0u; i < 30u && jsdk_joint_is_fault(fx.j); ++i) fx_cycle(&fx);
        CHECK_EQ(jsdk_joint_is_fault(fx.j), 0);
        printf("      recovery path: request_fault_reset cleared the fault\n");

        jsdk_context_set_fault_callback(fx.ctx, NULL, NULL);
    }

    fx_close(&fx);
}

/**
 * [12] 描述符的预算算的是“**静默多久**”，不是“总共多久”。
 *
 * 真机实测（CyberBeast USB2CAN / 1 Mbps Classic）：同一个 38433 字节描述符
 * 在 Classic 下是 1+6906 帧（FD 的 10.4 倍），连续有进展时整条流 ~3.5 s；
 * 而当时按“总预算”算，于是在 85% 处被截断，报的却是
 * `timed out after 3000 ms (32982/38433 bytes, 6057 frames received)` ——
 * 看上去像设备/线缆有问题，实际是预算量纲写错了。
 *
 * 本用例把流**放慢**（1 帧/ms，总时长远超预算）但让它一直有进展：必须成功。
 * ⚠ 必须走 `jsdk_context_desc_fetch()`：超时逻辑在那一层；直接喂
 *   `cb_desc_fetch_frame()` 测不到（试过，变异不红 —— 那条用例已删）。
 */
static void test_desc_stall_budget(void)
{
    fix_t fx;
    unsigned i;
    const uint32_t budget = 200u;   /* 故意很小：总时长会远超它 */

    printf("[12] descriptor budget is a STALL budget (slow stream must succeed)\n");

    if (fx_open(&fx, "0:id=1,hb=10,timeout=30000,fd", 1u) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    if (sim_set_desc(fx.sim, g_json, (uint32_t)g_json_len, 0x1234u) != 0) {
        g_fail++; g_checks++; fx_close(&fx); return;
    }
    sim_set_desc_rate(fx.sim, 1u);          /* 1 帧/ms：慢，但一直在动 */

    /* 预算只能通过**初始化时**的 cfg 生效（`jsdk_context_init` 会拷贝一份），
       所以这里重来一次：destroy → 改 cfg → init。 */
    jsdk_context_destroy(fx.ctx);
    memset(&fx.store, 0, sizeof fx.store);
    fx.cfg.desc.timeout_ms = budget;
    CHECK_EQ(jsdk_context_init((jsdk_context_t *)&fx.store, &fx.cfg), JSDK_OK);
    fx.ctx = (jsdk_context_t *)&fx.store;
    for (i = 0u; i < 1u; ++i) {
        CHECK_EQ(jsdk_context_add_joint(fx.ctx, &fx.jc[i], &fx.j), JSDK_OK);
    }
    fx.j = &fx.ctx->joints[0];

    if (jsdk_context_desc_fetch(fx.ctx) != JSDK_OK) {
        printf("      desc_fetch -> %s\n", jsdk_context_last_error(fx.ctx));
        g_fail++; g_checks++;
    } else {
        /* FD 下 ~663 帧 × 1 帧/ms ⇒ 总时长 ~663 ms ≫ 200 ms 预算 */
        printf("      %u frames at 1/ms (~%u ms total) with a %u ms budget -> OK, complete=%u\n",
               (unsigned)fx.ctx->desc.frames_rx, (unsigned)fx.ctx->desc.frames_rx,
               (unsigned)budget, (unsigned)fx.ctx->desc.complete);
        CHECK(fx.ctx->desc.complete == 1u);
    }
    sim_set_desc_rate(fx.sim, 0u);
    fx_close(&fx);
}

/**
 * [14] 帧格式（Classic / FD）：自动对齐的语义 + 仿真器的格式门限。
 *
 * 现场（Ubuntu + CyberBeast USB2CAN @ 1 Mbps）：设备是 **Classic**，SDK 默认发
 * **FD** ⇒ `desc-info` 报 `0/0 bytes, 198 frames received`（心跳收得到、请求没人应）。
 * 协议**没有**运行时协商，所以这条用例钉住三件事：
 *   ① 仿真器必须按真机挡住 FD→Classic（否则测试会在“格式猜错”时照样通过）；
 *   ② 默认（未显式指定）时 SDK 自动对齐过去并报告；
 *   ③ **显式**指定过就不许被偷偷改掉，只报 `framing_learned() == 4`。
 */
static void test_framing_semantics(void)
{
    fix_t    fx;
    unsigned i;
    uint32_t drops0;

    printf("[14] framing: FD->Classic is refused; auto-align only when not explicit\n");

    /* ---- ⓪ “先只听一耳朵”（= CLI 的自动探测）：一帧 FD 都不该发出去 ----
       这是真机现场那条修复的核心：默认发 FD 不只是“没人应”，它还会让适配器
       按 FD 配置，于是事后改学也没用 ⇒ 必须**发帧前**就听准。 */
    if (fx_open_flags(&fx, "0:id=1,hb=10,timeout=30000,classic", 1u, 0) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    CHECK_EQ(jsdk_context_framing_learned(fx.ctx), 0);
    for (i = 0u; i < 40u && jsdk_context_framing_learned(fx.ctx) == 0; ++i) {
        (void)jsdk_context_cycle_begin(fx.ctx, 0u);   /* 只收不发 */
    }
    CHECK_EQ(jsdk_context_framing_learned(fx.ctx), 3);   /* 与“起步 Classic”一致 */
    CHECK_EQ(fx.sim->fd_into_classic_drops, 0u);         /* 关键：没发过 FD 帧 */
    CHECK_EQ(jsdk_context_desc_fetch(fx.ctx), JSDK_OK);  /* 探测之后一次就成 */
    CHECK_EQ(fx.sim->fd_into_classic_drops, 0u);         /* 全程零 FD 帧 */
    fx_close(&fx);

    if (fx_open_flags(&fx, "0:id=1,hb=10,timeout=30000,classic", 1u, 1) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    CHECK_EQ(jsdk_context_framing_learned(fx.ctx), 0);   /* 还没收到过帧 */

    /* ---- ① 门限：同一帧，FD 版被丢，Classic 版被收 ---- */
    {
        jsdk_can_frame_t f;
        uint8_t          req[4] = { 0u, 0u, 0u, 0u };

        drops0 = fx.sim->fd_into_classic_drops;
        memset(&f, 0, sizeof f);
        f.id    = cb_make_id(CB_PRI_CONFIG, CB_MSG_JSON_DESC_READ, 1u, 1u, 0u);
        f.len   = 4u;
        f.flags = (uint8_t)(JSDK_FRAME_EXT | JSDK_FRAME_FD);
        memcpy(f.data, req, sizeof req);
        sim_rx(fx.sim, &f);
        CHECK_EQ(fx.sim->fd_into_classic_drops, drops0 + 1u);   /* 被挡住 */
        CHECK_EQ(fx.sim->desc.active, 0);                       /* 设备根本没开工 */

        f.flags = JSDK_FRAME_EXT;                               /* 反方向：真允许 */
        sim_rx(fx.sim, &f);
        CHECK_EQ(fx.sim->fd_into_classic_drops, drops0 + 1u);
        CHECK_EQ(fx.sim->desc.active, 1);         /* 这次设备真的开始发流了 */
        fx.sim->desc.active = 0;                  /* 别让它插进后面的下载 */
        fx.sim->desc.metadata_sent = 0;
        fx.sim->desc.offset = 0u;
    }

    /* ---- ② 默认（未显式指定）→ 自动对齐到对端 ---- */
    for (i = 0u; i < 40u && jsdk_context_framing_learned(fx.ctx) == 0; ++i) {
        (void)jsdk_context_cycle_begin(fx.ctx, 0u);
        (void)jsdk_context_cycle_end(fx.ctx);
    }
    CHECK_EQ(jsdk_context_framing_learned(fx.ctx), 1);   /* 1 = 已改学 Classic */
    CHECK_EQ(fx.ctx->cfg.is_fd, 0u);                     /* 格式真的改过去了 */
    CHECK_EQ(jsdk_context_desc_fetch(fx.ctx), JSDK_OK);  /* 改学之后就能下完 */
    fx_close(&fx);

    /* ---- ③ 显式指定过 → 冲突时不改调用者的值，只报告 ---- */
    if (fx_open_flags(&fx, "0:id=1,hb=10,timeout=30000,classic", 1u, 1) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    jsdk_context_destroy(fx.ctx);
    memset(&fx.store, 0, sizeof fx.store);
    fx.cfg.is_fd          = 1u;      /* 显式要 FD */
    fx.cfg.is_fd_explicit = 1u;
    fx.cfg.desc.timeout_ms = 500u;   /* 小预算：失败路径别拖时间 */
    CHECK_EQ(jsdk_context_init((jsdk_context_t *)&fx.store, &fx.cfg), JSDK_OK);
    fx.ctx = (jsdk_context_t *)&fx.store;
    CHECK_EQ(jsdk_context_add_joint(fx.ctx, &fx.jc[0], &fx.j), JSDK_OK);
    fx.j = &fx.ctx->joints[0];

    for (i = 0u; i < 40u && jsdk_context_framing_learned(fx.ctx) == 0; ++i) {
        (void)jsdk_context_cycle_begin(fx.ctx, 0u);
        (void)jsdk_context_cycle_end(fx.ctx);
    }
    CHECK_EQ(jsdk_context_framing_learned(fx.ctx), 4);   /* 4 = 显式配置与对端冲突 */
    CHECK_EQ(fx.ctx->cfg.is_fd, 1u);                     /* 一个字节都不许被改 */
    CHECK(jsdk_context_desc_fetch(fx.ctx) != JSDK_OK);   /* 设备收不到 FD 帧 ⇒ 必失败 */
    printf("      default -> learned=%d is_fd=%u (aligned); explicit -> learned=%d is_fd=%u (kept)\n",
           (int)1, (unsigned)0,
           jsdk_context_framing_learned(fx.ctx), (unsigned)fx.ctx->cfg.is_fd);
    fx_close(&fx);
}

/**
 * [15] 会话预热：**幂等重发**把“首帧丢失”挡在用户命令之前。
 *
 * 现场（slcan）：适配器在打开端口后重置输入缓冲，我们头一两条命令**被丢掉**，
 * 而 Lawicel slcan 对帧行**不回报结果**（实测 acks/nacks 恒 0）⇒ 主机没有任何
 * 可观测信号。唯一可验证的解法就是幂等请求 + 重发（见 joint_sdk.h 的长注释）。
 */
static void test_warmup(void)
{
    fix_t fx;
    uint32_t before;

    printf("[15] session warm-up: idempotent probe + retry absorbs the first-frame loss\n");

    /* ---- ① 链路健康：一次成功、零重发，第二次调用是空操作 ---- */
    if (fx_open(&fx, "0:id=1,hb=10,timeout=30000,fd", 1u) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    CHECK_EQ(jsdk_context_warmup(fx.ctx, 0u), JSDK_OK);
    CHECK_EQ(fx.ctx->bus.tx_retries, 0u);
    CHECK_EQ(fx.dropped, 0u);
    before = fx.ctx->bus.tx_frames;
    CHECK_EQ(jsdk_context_warmup(fx.ctx, 0u), JSDK_OK);
    CHECK_EQ(fx.ctx->bus.tx_frames, before);      /* 已预热 → 一帧都不再发 */
    fx_close(&fx);

    /* ---- ② 丢 1 帧：预热重发一次即恢复 ---- */
    if (fx_open(&fx, "0:id=1,hb=10,timeout=30000,fd", 1u) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    fx.drop_tx_head = 1u;
    CHECK_EQ(jsdk_context_warmup(fx.ctx, 0u), JSDK_OK);
    CHECK_EQ(fx.dropped, 1u);
    CHECK_EQ(fx.ctx->bus.tx_retries, 1u);
    fx_close(&fx);

    /* ---- ③ 懒预热：库用户不显式调用也能受益（第一次请求就是安全的）---- */
    if (fx_open(&fx, "0:id=1,hb=10,timeout=30000,fd", 1u) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    fx.drop_tx_head = 2u;                          /* 头两帧（= 预热的两轮）丢 */
    {
        jsdk_device_info_t info;

        CHECK_EQ(jsdk_joint_get_device_info(fx.j, &info), JSDK_OK);
        CHECK_EQ(info.hw_version != 0u, 1);
        CHECK_EQ(fx.ctx->bus.tx_retries, 2u);
        CHECK(fx.dropped >= 2u);
    }
    fx_close(&fx);

    /* ---- ④ 整条链没应答：TIMEOUT + 说明是“预热失败”，且预热重发过 ---- */
    if (fx_open(&fx, "0:id=1,hb=10,timeout=30000,fd", 1u) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    fx.drop_tx_head = 1000u;
    CHECK_EQ(jsdk_context_warmup(fx.ctx, 200u), JSDK_ERR_TIMEOUT);
    CHECK(fx.ctx->bus.tx_retries >= 2u);
    CHECK(strstr(jsdk_context_last_error(fx.ctx), "session warm-up failed") != NULL);
    CHECK(strstr(jsdk_context_last_error(fx.ctx), "node 1") != NULL);
    fx_close(&fx);

    /* ---- ⑤ 时钟冻住（客户自写 HAL 的假时钟）：必须靠**轮次上限**收尾，不能死循环 ---- */
    if (fx_open(&fx, "0:id=1,hb=10,timeout=30000,fd", 1u) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }
    fx.drop_tx_head = 1000u;
    fx.freeze_clock = 1;
    CHECK_EQ(jsdk_context_warmup(fx.ctx, 100u), JSDK_ERR_TIMEOUT);
    CHECK(fx.ctx->bus.tx_retries >= 1u);
    fx_close(&fx);
}

/**
 * [16] **幂等的“请求 → 响应”重发**（运行中途丢帧，不是会话开头）。
 *
 * 会话预热只保护“开头那几帧”；适配器中途抽一下时，`err`/`info`/`read` 这类
 * **单发即等**的命令以前没有任何兜底 —— 丢了就是一条超时（与首帧丢失同症状）。
 * 这里在 HAL 层按 MsgType 注入丢帧，断言：
 *   - 丢 1 帧 → 自动重发一次，命令成功，计数进 `tx_retries_req`；
 *   - 全丢 → **有界失败**（只多试一次，不是无限重试）；
 *   - 轮询路径（`jsdk_ctx_read_param_once`）**不**重发（节奏优先）；
 *   - 写（等 ACK）也重发；⚠ 但**不等 ACK 的写**（`timeout_ms = 0`，如
 *     `axis0.requested_state`）**绝不重发** —— 重发等于重复触发标定/回零。
 */
static void test_idempotent_request_retry(void)
{
    fix_t fx;
    uint32_t before;

    printf("[16] idempotent request retry: a dropped read/ACK is re-sent once\n");

    if (fx_open(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                     "kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0
        || fx_configure(&fx) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }

    /* ---- ① 丢 1 条 PARAM_READ → 读自愈，且归类到“幂等请求” ---- */
    {
        jsdk_value_t v;

        fx.drop_msgtype    = (uint8_t)CB_MSG_PARAM_READ;
        fx.drop_msgtype_n  = 1u;
        fx.dropped_msgtype = 0u;
        CHECK_EQ(jsdk_joint_param_get(fx.j, "axis0.motor.config.gear_ratio", &v), JSDK_OK);
        CHECK_EQ(fx.dropped_msgtype, 1u);
        CHECK_EQ(fx.ctx->bus.tx_retries_req, 1u);
        CHECK_EQ(fx.ctx->bus.tx_retries, 1u);

        /* 再读一次：没有丢帧 → 不应产生任何重发 */
        before = fx.ctx->bus.tx_retries;
        CHECK_EQ(jsdk_joint_param_get(fx.j, "axis0.motor.config.gear_ratio", &v), JSDK_OK);
        CHECK_EQ(fx.ctx->bus.tx_retries, before);
    }

    /* ---- ② 全丢 → 有界失败：只多试 1 次（总共 2 次），计数 +1 ---- */
    {
        jsdk_value_t v;

        fx.drop_msgtype_n = 1000u;
        before = fx.ctx->bus.tx_retries_req;
        CHECK_EQ(jsdk_joint_param_get(fx.j, "axis0.motor.config.gear_ratio", &v),
                 JSDK_ERR_TIMEOUT);
        CHECK_EQ(fx.ctx->bus.tx_retries_req, before + 1u);
        fx.drop_msgtype_n = 0u;
        fx.drop_msgtype   = 0u;
    }

    /* ---- ③ 轮询路径只发一次（不重发）：帧数增加恰好 1 ---- */
    {
        uint8_t buf[8];
        uint8_t len = 0u;
        uint32_t tx;

        fx.drop_msgtype    = (uint8_t)CB_MSG_PARAM_READ;
        fx.drop_msgtype_n  = 1u;
        fx.dropped_msgtype = 0u;      /* 重新上弦：计数器也要归零 */
        tx = fx.ctx->bus.tx_frames;
        CHECK_EQ(jsdk_ctx_read_param_once(fx.ctx, 1u, fx.j->ep_current_state,
                                          buf, &len, 100u), JSDK_ERR_TIMEOUT);
        CHECK_EQ(fx.ctx->bus.tx_frames, tx + 1u);        /* 只发了一次 */
        CHECK_EQ(fx.ctx->bus.tx_retries_req, before + 1u);  /* 没涨 */
        fx.drop_msgtype_n = 0u;
        fx.drop_msgtype   = 0u;
    }

    /* ---- ④ 写：等 ACK 的会重发，不等 ACK 的**绝**重发 ---- */
    {
        uint8_t one[1] = { 0u };

        fx.drop_msgtype    = (uint8_t)CB_MSG_PARAM_WRITE;
        fx.drop_msgtype_n  = 1u;
        fx.dropped_msgtype = 0u;
        before = fx.ctx->bus.tx_retries_req;
        CHECK_EQ(jsdk_ctx_write_param(fx.ctx, 1u, fx.j->ep_current_state, one, 1u,
                                      500u), JSDK_OK);
        CHECK_EQ(fx.ctx->bus.tx_retries_req, before + 1u);

        fx.drop_msgtype_n = 1000u;               /* 全丢 */
        before = fx.ctx->bus.tx_retries_req;
        /* timeout_ms = 0 = 不等 ACK（`requested_state` 那类）：发完就算，不重发 */
        CHECK_EQ(jsdk_ctx_write_param(fx.ctx, 1u, fx.j->ep_current_state, one, 1u,
                                      0u), JSDK_OK);
        CHECK_EQ(fx.ctx->bus.tx_retries_req, before);
        fx.drop_msgtype_n = 0u;
        fx.drop_msgtype   = 0u;
    }

    /* ---- ⑤ 扫描（大多数地址本来就没人）**不**重发：否则不存在的节点会把耗时翻倍
           （真机实测：`scan` 每次都报 `重发=15（预热 0 + 幂等请求 15）`，
             全是 2..16 号空地址的假重发）。 ---- */
    {
        uint8_t ids[16];
        unsigned found = 0u;
        uint32_t before_retries = fx.ctx->bus.tx_retries_req;

        CHECK_EQ(jsdk_context_discover(fx.ctx, ids, 16u, &found, 16u), JSDK_OK);
        CHECK(found >= 1u);
        CHECK_EQ(jsdk_ctx_probe_node(fx.ctx, 1u), 1);          /* 在线的能找到 */
        CHECK_EQ(jsdk_ctx_probe_node(fx.ctx, 9u), 0);          /* 不在线：正常结果 */
        CHECK_EQ(fx.ctx->bus.tx_retries_req, before_retries);  /* 一次都没重发 */

        /* 改号前的安全检查用 strict：丢一帧也要补一次（假阴性 = 造出两个同号设备） */
        fx.drop_msgtype    = (uint8_t)CB_MSG_QUERY_STATUS;
        fx.drop_msgtype_n  = 1u;
        fx.dropped_msgtype = 0u;
        CHECK_EQ(jsdk_ctx_probe_node_strict(fx.ctx, 1u), 1);
        CHECK_EQ(fx.ctx->bus.tx_retries_req, before_retries + 1u);
        fx.drop_msgtype_n = 0u;
        fx.drop_msgtype   = 0u;
    }

    printf("      dropped 1 read -> retried once (tx_retries_req=1); "
           "all dropped -> bounded failure; polling/no-ACK/scan never retry\n");
    fx_close(&fx);
}

/**
 * [17] 链路质量观测（v0.33）：**让现场不用靠猜**。
 *
 * 断言三件事：
 *   1. 三个计数器的**不变式**：`tx_retries == tx_retries_warm + tx_retries_req`；
 *   2. `req_timeouts` 数的是“等过但没等到”的次数（含后来被重发救回的 —— 那才是
 *      “链路偶发丢帧”的证据；只看“失败”是看不到丢帧的）；
 *   3. `last_retry_what` / `last_retry_age_ms` 说得出“最近一次重发是哪一类、多久之前”。
 *   另外把描述符下载的重发次数（`desc_info.retries`）也钉住 —— 它就是
 *   “0x24 请求丢了”的直接证据（v0.23 那次真机排查全靠它）。
 */
static void test_link_quality_observability(void)
{
    fix_t fx;
    jsdk_bus_state_t bs;

    printf("[17] link-quality observability: retry classes, timeouts, last-retry, "
           "desc retries\n");

    if (fx_open(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                     "kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) != 0) {
        printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
    }

    /* ---- ① 会话预热丢帧 → 归类为“预热”，并且不变式成立 ---- */
    fx.drop_tx_head = 2u;
    CHECK_EQ(fx_configure(&fx), 0);
    CHECK_EQ(jsdk_context_get_bus_state(fx.ctx, &bs), JSDK_OK);
    CHECK_EQ(bs.tx_retries, bs.tx_retries_warm + bs.tx_retries_req);
    CHECK(bs.tx_retries_warm >= 2u);
    CHECK_EQ(bs.tx_retries_req, 0u);              /* 预热丢帧不算“运行中途丢帧” */
    CHECK_EQ(bs.last_retry_what, 1u);             /* 1 = 会话预热 */
    CHECK(bs.last_retry_age_ms < 10000u);

    /* ---- ② 运行中途丢一条读 → 归类为“幂等请求”，且超时计数 +1 ---- */
    fx.drop_tx_head    = 0u;
    fx.dropped         = 0u;
    fx.drop_msgtype    = (uint8_t)CB_MSG_PARAM_READ;
    fx.drop_msgtype_n  = 1u;
    fx.dropped_msgtype = 0u;
    {
        jsdk_value_t v;
        uint32_t to_before;

        CHECK_EQ(jsdk_context_get_bus_state(fx.ctx, &bs), JSDK_OK);
        to_before = bs.req_timeouts;

        CHECK_EQ(jsdk_joint_param_get(fx.j, "axis0.motor.config.gear_ratio", &v), JSDK_OK);
        CHECK_EQ(jsdk_context_get_bus_state(fx.ctx, &bs), JSDK_OK);
        CHECK_EQ(bs.tx_retries, bs.tx_retries_warm + bs.tx_retries_req);
        CHECK_EQ(bs.tx_retries_req, 1u);
        CHECK_EQ(bs.req_timeouts, to_before + 1u);   /* 救回来了也要记账 */
        CHECK_EQ(bs.last_retry_what, 2u);            /* 2 = 幂等请求 */
    }

    /* ---- ③ 全丢 → 超时继续累加（这才是“链路真不稳”的判据） ---- */
    fx.drop_msgtype_n = 1000u;
    {
        jsdk_value_t v;
        uint32_t to_before;

        CHECK_EQ(jsdk_context_get_bus_state(fx.ctx, &bs), JSDK_OK);
        to_before = bs.req_timeouts;

        CHECK_EQ(jsdk_joint_param_get(fx.j, "axis0.motor.config.gear_ratio", &v),
                 JSDK_ERR_TIMEOUT);
        CHECK_EQ(jsdk_context_get_bus_state(fx.ctx, &bs), JSDK_OK);
        CHECK(bs.req_timeouts >= to_before + 1u);
        fx.drop_msgtype_n = 0u;
        fx.drop_msgtype   = 0u;
    }
    fx_close(&fx);

    /* ---- ④ 描述符下载的 0x24 重发次数（丢首帧时必须 >= 1） ---- */
    if (fx_open(&fx, "0:id=1,gear=16.0,tconst=0.0385,pmax=12.5,vmax=65,tmax=50,"
                     "kpmax=500,kdmax=5,hb=10,timeout=30000,fd", 1u) == 0) {
        jsdk_desc_info_t di;

        /* 注意：会话预热会先发一帧 0x46（不是 0x24），所以这里的丢帧正好落在
           描述符请求上 —— 那是 0x24 重发逻辑唯一的入口。 */
        fx.drop_msgtype    = (uint8_t)CB_MSG_JSON_DESC_READ;
        fx.drop_msgtype_n  = 1u;
        fx.dropped_msgtype = 0u;

        CHECK_EQ(fx_configure(&fx), 0);
        CHECK_EQ(jsdk_context_get_desc_info(fx.ctx, &di), JSDK_OK);
        CHECK(di.retries >= 1u);
        CHECK_EQ(di.complete, 1u);
        printf("      descriptor request re-sent %u time(s) after the injected drop\n",
               di.retries);
        fx_close(&fx);
    }

    /* ---- ⑤ “问了才知道”的探测与“马上会再问”的轮询**不算超时** ----
           （真机实测：一条正常的 `scan` 曾报 `超时=15` —— 全是 2..16 号
             空地址的“没人应答”。那正是本轮想消灭的“误导数字”） */
    {
        uint8_t  ids[16];
        unsigned found = 0u;
        uint8_t  buf[8];
        uint8_t  len = 0u;
        uint32_t to;
        uint32_t retries;
        fix_t    fx2;

        if (fx_open(&fx2, "0:id=1,hb=10,timeout=30000,fd", 1u) != 0
            || fx_configure(&fx2) != 0) {
            printf("      FATAL: fixture failed\n"); g_fail++; g_checks++; return;
        }
        CHECK_EQ(jsdk_context_get_bus_state(fx2.ctx, &bs), JSDK_OK);
        to      = bs.req_timeouts;
        retries = bs.tx_retries;

        CHECK_EQ(jsdk_context_discover(fx2.ctx, ids, 16u, &found, 16u), JSDK_OK);
        CHECK_EQ(jsdk_ctx_probe_node(fx2.ctx, 9u), 0);      /* 空地址：正常结果 */
        CHECK_EQ(jsdk_ctx_read_param_once(fx2.ctx, 1u, fx2.j->ep_current_state,
                                          buf, &len, 100u), JSDK_OK);
        CHECK_EQ(jsdk_context_get_bus_state(fx2.ctx, &bs), JSDK_OK);
        CHECK_EQ(bs.req_timeouts, to);        /* 探测/轮询不计 */
        CHECK_EQ(bs.tx_retries, retries);
        fx_close(&fx2);
    }

    printf("      retry classes split correctly; timeouts counted (but not for "
           "probes/polling); last retry reported; desc retries visible\n");
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("=== WP4/WP5 tests (robustness + operations) ===\n\n");
    if (load_fixture() != 0) {
        printf("FATAL: cannot read fixture\n");
        return 1;
    }
    printf("fixture: %u bytes\n\n", (unsigned)g_json_len);

    test_fault_text();   printf("\n");
    test_params();       printf("\n");
    test_sdo();          printf("\n");
    test_discover();     printf("\n");
    test_ops();          printf("\n");
    test_activate();     printf("\n");
    test_wide_param_read(); printf("\n");
    test_lost_first_request(); printf("\n");
    test_handshake_retry(); printf("\n");
    test_watchdog_readback(); printf("\n");
    test_watchdog_disabled(); printf("\n");
    test_write_short_values(); printf("\n");
    test_desc_stall_budget(); printf("\n");
    test_framing_semantics(); printf("\n");
    test_warmup();       printf("\n");
    test_idempotent_request_retry(); printf("\n");
    test_link_quality_observability(); printf("\n");
    test_robustness();   printf("\n");

    free(g_json);
    printf("=== %u checks, %u failures ===\n", g_checks, g_fail);
    return g_fail != 0u ? 1 : 0;
}
