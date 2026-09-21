/**
 * @file    test_desc_fetch.c
 * @brief   WP2b 传输层 + 描述符缓存 回归测试
 *
 * 全部是**端到端**的：真实 41029 字节描述符 → 虚拟 HAL 的 `0x25` 帧流 →
 * 传输状态机 → 增量解析器 → arena；再对导出/导入做往返与失效键校验。
 *
 * 覆盖：
 *   1. 完整下载（RETAIN_ALL，FD / Classic）——端点表、帧数、字节数
 *   2. raw sink：偏移连续、字节与原始文件逐字节相同
 *   3. RETAIN_FILTERED + stop_when_satisfied（提前终止、三条后置校验拦住缓存）
 *   4. RETAIN_FILTERED + 不提前终止（路线 B 可缓存）
 *   5. 失败路径：首帧非元数据 / total_len 非法 / 偏移跳号 / 帧长非法 /
 *      截断 / JSON 损坏 / arena 不足 —— 一律**不留部分结果**
 *   6. 缓存导出/导入往返（含**不同大小 arena**）+ 失效键 + CRC + 版本
 *   7. 路线 B：import_raw 与逐帧解析结果一致
 */

#include "jsdk_internal.h"
#include "cb_jsondesc_fetch.h"
#include "cb_desc_cache.h"

#include "jsdk_hal_builtin.h"
#include "hal_virtual_internal.h"

#include "cb_frame.h"
#include "cb_param.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef JSDK_TEST_DATA_DIR
#  define JSDK_TEST_DATA_DIR "tests/data"
#endif

#define FIXTURE_PATH JSDK_TEST_DATA_DIR "/endpoints_v8.json"

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

/* 字符串必须按**内容**比较：不同编译单元里的字面量地址不同，
   直接比指针会永远失败（本测试确实踩过这个坑）。两个都为空也算相等。 */
#define CHECK_STR(a, b)                                                        \
    do {                                                                       \
        const char *_a = (a), *_b = (b);                                       \
        g_checks++;                                                            \
        if ((_a == NULL) != (_b == NULL)                                       \
            || (_a && strcmp(_a, _b) != 0)) {                                  \
            printf("  FAIL %s:%d  \"%s\" != expected \"%s\"\n",                 \
                   __FILE__, __LINE__, _a ? _a : "(null)", _b ? _b : "(null)");\
            g_fail++;                                                          \
        }                                                                      \
    } while (0)

/* ==========================================================================
 * 夹具：原始 JSON
 * ======================================================================== */

static char  *g_json;
static size_t g_json_len;

static int load_fixture(void)
{
    FILE *fh = fopen(FIXTURE_PATH, "rb");
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

/* ==========================================================================
 * raw sink 记录器
 * ======================================================================== */

typedef struct {
    uint8_t *buf;
    size_t   cap;
    size_t   len;
    uint32_t expect_next;
    int      gap_seen;
    int      fail_after;      /* >=0：写够这么多字节后返回非 0 */
    unsigned calls;
} sink_rec_t;

static int sink_cb(jsdk_context_t *ctx, const void *data, size_t len,
                   uint32_t offset, void *user)
{
    sink_rec_t *r = (sink_rec_t *)user;
    (void)ctx;

    r->calls++;
    if (offset != r->expect_next) r->gap_seen = 1;
    r->expect_next = offset + (uint32_t)len;

    if (r->fail_after >= 0 && (int)r->len >= r->fail_after) return -1;
    if (r->len + len <= r->cap) {
        memcpy(r->buf + r->len, data, len);
        r->len += len;
    } else {
        return -1;
    }
    return 0;
}

/* ==========================================================================
 * 下载驱动：真实走 HAL 的 send/recv，模拟器按毫秒推进
 * ======================================================================== */

typedef struct {
    jsdk_can_hal_t     hal;
    jsdk_hal_handle_t *h;
    sim_bus_t         *sim;
} hal_fix_t;

static int hal_open(hal_fix_t *hf, const char *spec)
{
    memset(hf, 0, sizeof *hf);
    if (jsdk_hal_virtual_open(&hf->hal, &hf->h, spec) != JSDK_OK) return -1;
    hf->sim = jsdk_hal_virtual_sim(hf->h);
    return hf->sim ? 0 : -1;
}

static void hal_close(hal_fix_t *hf)
{
    if (hf->h) jsdk_hal_close(hf->h);
    hf->h = NULL;
    hf->sim = NULL;
}

typedef struct {
    unsigned frames;        /* 投喂给状态机的 0x25 帧数 */
    unsigned drained;       /* 从 HAL 收到的帧总数（含被忽略的） */
    int      ran_out;       /* 1 = 循环上限到了还没结束 */
} drive_t;

/**
 * 发出 0x24 并驱动到结束。
 * @param max_ms 最多推进的毫秒数（模拟器每毫秒 ≤50 帧）
 */
static void drive_fetch(hal_fix_t *hf, cb_desc_fetch_t *f, uint32_t max_ms,
                        drive_t *out)
{
    jsdk_can_frame_t fr;
    uint8_t req[CB_DESC_REQ_LEN];
    uint32_t t;

    memset(out, 0, sizeof *out);

    /* 发 0x24 请求 */
    CHECK_EQ(cb_desc_build_request(req, sizeof req, 0u), CB_DESC_REQ_LEN);
    memset(&fr, 0, sizeof fr);
    fr.id    = cb_make_id(CB_PRI_CONFIG, CB_MSG_JSON_DESC_READ, 1u, 1u, 0u);
    fr.len   = (uint8_t)CB_DESC_REQ_LEN;
    fr.flags = (uint8_t)(JSDK_FRAME_EXT
                         | (hf->sim->nodes[0].is_fd ? (JSDK_FRAME_FD | JSDK_FRAME_BRS) : 0u));
    memcpy(fr.data, req, CB_DESC_REQ_LEN);
    CHECK_EQ(hf->hal.send(hf->hal.user, &fr), 0);

    for (t = 0u; t < max_ms; ++t) {
        jsdk_hal_virtual_advance_ms(hf->h, 1u);
        while (hf->hal.recv(hf->hal.user, &fr) == 1) {
            out->drained++;
            if (cb_id_msgtype(fr.id) != CB_MSG_JSON_DESC_DATA) continue;
            out->frames++;
            (void)cb_desc_fetch_frame(f, fr.data, fr.len);
        }
        if (cb_desc_fetch_is_done(f)) return;
    }
    out->ran_out = 1;
}

/* 精确路径 filter（不含通配）→ 允许提前终止，也是 MCU 推荐写法 */
static const char *const k_filter_exact[] = {
    "axis0.motor.config.gear_ratio",
    "axis0.motor.config.torque_constant",
    "axis0.motor.config.current_lim",
    "axis0.controller.config.mit_max_pos",
    "axis0.controller.config.vel_limit",
    "axis0.controller.config.control_mode",
    "axis0.config.can.node_id",
    "axis0.config.can.heartbeat_rate_ms",
    "can.config.break_timeout",
    "axis0.error",
    "vbus_voltage",
    "serial_number",
};

/* 同一条目，但把 mit_max_pos 换成通配 mit_max_* → **不允许**提前终止 */
static const char *const k_filter_wild[] = {
    "axis0.motor.config.gear_ratio",
    "axis0.motor.config.torque_constant",
    "axis0.motor.config.current_lim",
    "axis0.controller.config.mit_max_*",
    "axis0.controller.config.vel_limit",
    "axis0.controller.config.control_mode",
    "axis0.config.can.node_id",
    "axis0.config.can.heartbeat_rate_ms",
    "can.config.break_timeout",
    "axis0.error",
    "vbus_voltage",
    "serial_number",
};

/* ==========================================================================
 * 1~4. 正常下载与提前终止
 * ======================================================================== */

static void test_full_fetch(void)
{
    hal_fix_t hf;
    cb_desc_fetch_t f;
    cb_desc_fetch_result_t res;
    jsdk_desc_config_t cfg;
    jsdk_ep_store_t store;
    static uint8_t arena[65536];
    static uint8_t raw[65536];
    sink_rec_t sink;
    drive_t drv;

    printf("[1] full fetch (RETAIN_ALL, FD)\n");

    memset(&cfg, 0, sizeof cfg);
    cfg.retain        = JSDK_DESC_RETAIN_ALL;
    cfg.max_endpoints = 0;                 /* → 2048 */
    cfg.max_path_len  = 0;                 /* → 128 */
    cfg.arena         = arena;
    cfg.arena_size    = sizeof arena;

    memset(&store, 0, sizeof store);
    store.arena.base = arena;
    store.arena.size = sizeof arena;
    store.arena.blob_top = sizeof arena;

    memset(&sink, 0, sizeof sink);
    sink.buf = raw; sink.cap = sizeof raw; sink.fail_after = -1;

    CHECK_EQ(hal_open(&hf, "0:id=1,fd"), 0);
    CHECK_EQ(sim_set_desc(hf.sim, g_json, (uint32_t)g_json_len, 0x1234u), 0);

    CHECK_EQ(cb_desc_fetch_init(&f, NULL, &cfg, &store), JSDK_OK);
    cb_desc_fetch_set_raw_sink(&f, sink_cb, &sink);

    drive_fetch(&hf, &f, 200u, &drv);

    CHECK_EQ(drv.ran_out, 0);
    CHECK_EQ(cb_desc_fetch_is_done(&f), 1);
    CHECK_EQ(cb_desc_fetch_is_ok(&f), 1);
    CHECK_EQ(cb_desc_fetch_error(&f), NULL);

    cb_desc_fetch_result(&f, &res);
    CHECK_EQ(res.total_len, g_json_len);
    CHECK_EQ(res.crc, 0x1234u);
    CHECK_EQ(res.complete, 1u);
    CHECK_EQ(res.stopped_early, 0u);
    CHECK_EQ(res.failed, 0u);
    CHECK_EQ(res.bytes_scanned, g_json_len);
    CHECK_EQ(res.frames_rx, 663u);                    /* 1 元数据 + 662 数据 */
    CHECK_EQ(res.endpoint_count, 594u);
    CHECK_EQ(res.parsed_total, 594u);
    CHECK_EQ(res.arena_used, 25493u);                 /* 与解析器单测一致 */
    CHECK_EQ(store.arena.blob_used, 20741u);
    CHECK_EQ(cb_desc_fetch_pct(&f), 100u);

    /* raw sink：偏移连续、字节与样本完全相同、三条后置校验通过 */
    CHECK_EQ(sink.gap_seen, 0);
    CHECK_EQ(sink.len, g_json_len);
    CHECK_EQ(sink.expect_next, (uint32_t)g_json_len);
    CHECK(memcmp(raw, g_json, g_json_len) == 0);
    CHECK_EQ(cb_desc_fetch_cache_safe(&f), 1);

    printf("      %u frames, %u endpoints, %u B raw tee'd (byte-identical)\n",
           res.frames_rx, res.endpoint_count, (unsigned)sink.len);

    /* --- Classic：帧数 6840，结果必须与 FD 完全相同 --- */
    {
        static uint8_t arena2[65536];
        jsdk_ep_store_t st2;
        cb_desc_fetch_t f2;
        cb_desc_fetch_result_t r2;

        hf.sim->nodes[0].is_fd = 0u;
        memset(&st2, 0, sizeof st2);
        st2.arena.base = arena2;
        st2.arena.size = sizeof arena2;
        st2.arena.blob_top = sizeof arena2;

        CHECK_EQ(cb_desc_fetch_init(&f2, NULL, &cfg, &st2), JSDK_OK);
        drive_fetch(&hf, &f2, 400u, &drv);
        CHECK_EQ(drv.ran_out, 0);
        CHECK_EQ(cb_desc_fetch_is_ok(&f2), 1);
        cb_desc_fetch_result(&f2, &r2);
        CHECK_EQ(r2.frames_rx, 6840u);                /* 1 + 6839 */
        CHECK_EQ(r2.endpoint_count, 594u);
        CHECK_EQ(r2.bytes_scanned, g_json_len);
        CHECK_EQ(r2.arena_used, 25493u);

        /* 两种帧长下 arena 内容必须逐字节相同 */
        CHECK(memcmp(arena, arena2, sizeof arena) == 0);
        printf("      classic: %u frames, identical arena (%u B)\n",
               r2.frames_rx, (unsigned)r2.arena_used);
    }

    hal_close(&hf);
}

static void test_filtered_stop(void)
{
    hal_fix_t hf;
    cb_desc_fetch_t f;
    cb_desc_fetch_result_t res;
    jsdk_desc_config_t cfg;
    jsdk_ep_store_t store;
    static uint8_t arena[4096];
    static uint8_t raw[65536];
    sink_rec_t sink;
    drive_t drv;
    uint16_t id = 0u;

    printf("[3] RETAIN_FILTERED\n");

    /* ===== 3a. 含通配 filter → **禁用**提前终止（关键安全规则）===== */
    memset(&cfg, 0, sizeof cfg);
    cfg.retain              = JSDK_DESC_RETAIN_FILTERED;
    cfg.stop_when_satisfied = 1u;
    cfg.max_path_len        = 128u;
    cfg.filter_paths        = k_filter_wild;
    cfg.filter_count        = 12u;
    cfg.arena               = arena;
    cfg.arena_size          = sizeof arena;

    memset(&store, 0, sizeof store);
    store.arena.base = arena;
    store.arena.size = sizeof arena;
    store.arena.blob_top = sizeof arena;

    memset(&sink, 0, sizeof sink);
    sink.buf = raw; sink.cap = sizeof raw; sink.fail_after = -1;

    CHECK_EQ(hal_open(&hf, "0:id=1,fd"), 0);
    CHECK_EQ(sim_set_desc(hf.sim, g_json, (uint32_t)g_json_len, 0x1234u), 0);

    CHECK_EQ(cb_desc_filters_all_exact(&cfg), 0);
    CHECK_EQ(cb_desc_fetch_init(&f, NULL, &cfg, &store), JSDK_OK);
    cb_desc_fetch_set_raw_sink(&f, sink_cb, &sink);
    drive_fetch(&hf, &f, 200u, &drv);

    CHECK_EQ(cb_desc_fetch_is_ok(&f), 1);
    cb_desc_fetch_result(&f, &res);
    CHECK_EQ(res.stop_allowed, 0u);      /* 含通配 → 不允许提前终止 */
    CHECK_EQ(res.stopped_early, 0u);
    CHECK_EQ(res.complete, 1u);          /* 整份扫完 */
    CHECK_EQ(res.bytes_scanned, res.total_len);
    CHECK_EQ(res.frames_rx, 663u);
    CHECK_EQ(jsdk_jsondesc_filter_hits(&f.parser), 12u);
    /* ⚠ 通配家族必须**完整**保留（这正是禁用提前终止的意义） */
    CHECK_EQ(jsdk_ep_store_lookup(&store, "axis0.controller.config.mit_max_pos",
                                  &id, NULL, NULL), JSDK_OK);
    CHECK_EQ(id, 335u);
    CHECK_EQ(jsdk_ep_store_lookup(&store, "axis0.controller.config.mit_max_vel",
                                  &id, NULL, NULL), JSDK_OK);
    CHECK_EQ(id, 336u);
    CHECK_EQ(jsdk_ep_store_lookup(&store, "axis0.controller.config.mit_max_torque",
                                  &id, NULL, NULL), JSDK_OK);
    CHECK_EQ(id, 337u);
    CHECK_EQ(jsdk_ep_store_lookup(&store, "axis0.controller.config.mit_max_kp",
                                  &id, NULL, NULL), JSDK_OK);
    CHECK_EQ(id, 338u);
    CHECK_EQ(jsdk_ep_store_lookup(&store, "axis0.controller.config.mit_max_kd",
                                  &id, NULL, NULL), JSDK_OK);
    CHECK_EQ(id, 339u);
    CHECK(res.endpoint_count >= 16u);
    CHECK(res.arena_used < 1024u);
    CHECK_EQ(cb_desc_fetch_cache_safe(&f), 1);   /* 完整 → raw 可缓存 */
    CHECK_EQ(sink.len, g_json_len);
    printf("      3a) wildcard filter -> stop_allowed=0, scanned all, "
           "%u endpoints (mit_max_* family complete)\n", res.endpoint_count);
    hal_close(&hf);

    /* ===== 3b. 全部精确路径 → 提前终止生效（MCU 推荐）===== */
    memset(&store, 0, sizeof store);
    store.arena.base = arena;
    store.arena.size = sizeof arena;
    store.arena.blob_top = sizeof arena;
    memset(&sink, 0, sizeof sink);
    sink.buf = raw; sink.cap = sizeof raw; sink.fail_after = -1;
    memset(arena, 0, sizeof arena);

    cfg.filter_paths = k_filter_exact;
    cfg.filter_count = 12u;

    CHECK_EQ(hal_open(&hf, "0:id=1,fd"), 0);
    CHECK_EQ(sim_set_desc(hf.sim, g_json, (uint32_t)g_json_len, 0x1234u), 0);
    CHECK_EQ(cb_desc_filters_all_exact(&cfg), 1);
    CHECK_EQ(cb_desc_fetch_init(&f, NULL, &cfg, &store), JSDK_OK);
    cb_desc_fetch_set_raw_sink(&f, sink_cb, &sink);
    drive_fetch(&hf, &f, 200u, &drv);

    CHECK_EQ(cb_desc_fetch_is_ok(&f), 1);
    cb_desc_fetch_result(&f, &res);
    CHECK_EQ(res.stop_allowed, 1u);
    CHECK_EQ(res.stopped_early, 1u);
    CHECK_EQ(res.complete, 0u);
    CHECK(res.bytes_scanned < res.total_len);
    CHECK_EQ(res.endpoint_count, 12u);
    CHECK_EQ(jsdk_jsondesc_filter_hits(&f.parser), 12u);
    CHECK_EQ(jsdk_ep_store_lookup(&store, "axis0.motor.config.gear_ratio",
                                  &id, NULL, NULL), JSDK_OK);
    CHECK_EQ(id, 242u);
    CHECK_EQ(jsdk_ep_store_lookup(&store, "vbus_voltage", &id, NULL, NULL), JSDK_OK);
    CHECK_EQ(id, 2u);
    CHECK_EQ(jsdk_ep_store_lookup(&store, "axis0.controller.pos_estimate",
                                  &id, NULL, NULL), JSDK_ERR_NOT_FOUND);

    /* ⚠ 三条后置校验必须拦住“把不完整的原始字节当缓存” */
    CHECK_EQ(cb_desc_fetch_cache_safe(&f), 0);
    CHECK(sink.len < g_json_len);
    printf("      3b) exact-only filter -> stop_allowed=1, stopped at %u/%u B "
           "(%u frames), 12 endpoints, arena %u B\n",
           res.bytes_scanned, res.total_len, res.frames_rx, (unsigned)res.arena_used);
    printf("          cache_safe=0 (early stop) -> raw bytes must NOT be cached\n");

    /* 早期终止后设备还会继续发；状态机要静默忽略而不是报错 */
    {
        jsdk_can_frame_t fr;
        unsigned extra = 0u;
        while (extra < 100u && hf.hal.recv(hf.hal.user, &fr) == 1) {
            if (cb_id_msgtype(fr.id) == CB_MSG_JSON_DESC_DATA) {
                CHECK_EQ(cb_desc_fetch_frame(&f, fr.data, fr.len), JSDK_OK);
            }
            extra++;
        }
        CHECK_STR(cb_desc_fetch_error(&f), CB_DESC_ERR_NONE);
        CHECK_EQ(cb_desc_fetch_is_ok(&f), 1);
    }

    /* ⚠ 关掉这一轮的 HAL：后面 3c) 会用 `hal_open` **重建**同一个 hf，
       不关的话上一轮的 bus / 描述符副本就漏了（ASan 当场报 leak）。 */
    hal_close(&hf);

    /* --- 3c) 边界：最后一个 filter 恰好在**末帧**才满足 ---------------------
       `.get_drv_fault.drv_fault`（id 521）是描述符里**最后一个**端点，其
       `"id"`/`"type"` 落在第 662 帧（字节 41007~41012）内。若“提前终止”判定
       排在“收满”判定之前，这次**完整**下载会被误标成 complete=0 /
       stopped_early=1，白丢一份可用的 raw 缓存。 */
    {
        static const char *const k_last[] = { "get_drv_fault.drv_fault" };
        static uint8_t arena_c[4096];
        cb_desc_fetch_t lastf;
        jsdk_ep_store_t st;
        size_t skipped = 0u;

        cfg.filter_paths = k_last;
        cfg.filter_count = 1u;
        cfg.arena = arena_c;
        cfg.arena_size = sizeof arena_c;
        memset(arena_c, 0, sizeof arena_c);
        memset(&st, 0, sizeof st);
        st.arena.base = arena_c; st.arena.size = sizeof arena_c;
        st.arena.blob_top = sizeof arena_c;

        CHECK_EQ(hal_open(&hf, "0:id=1,fd"), 0);
        CHECK_EQ(sim_set_desc(hf.sim, g_json, (uint32_t)g_json_len, 0x1234u), 0);
        CHECK_EQ(cb_desc_filters_all_exact(&cfg), 1);
        CHECK_EQ(cb_desc_fetch_init(&lastf, NULL, &cfg, &st), JSDK_OK);
        drive_fetch(&hf, &lastf, 200u, &drv);
        cb_desc_fetch_result(&lastf, &res);

        CHECK_EQ(res.endpoint_count, 1u);
        CHECK_EQ(jsdk_jsondesc_filter_hits(&lastf.parser), 1u);   /* 确实命中了 */
        CHECK_EQ(res.bytes_scanned, res.total_len);               /* 且是收满的 */
        CHECK_EQ(res.complete, 1u);                               /* 不得被误标 */
        CHECK_EQ(res.stopped_early, 0u);
        CHECK(!(res.complete && res.stopped_early));              /* 互斥 */
        CHECK_EQ(jsdk_ep_store_lookup(&st, "get_drv_fault.drv_fault",
                                      &id, NULL, NULL), JSDK_OK);
        CHECK_EQ(id, 521u);

        /* 设备余下的帧（若有）被静默丢弃 */
        {
            jsdk_can_frame_t fr;
            while (skipped < 200u && hf.hal.recv(hf.hal.user, &fr) == 1) {
                if (cb_id_msgtype(fr.id) == CB_MSG_JSON_DESC_DATA) {
                    CHECK_EQ(cb_desc_fetch_frame(&lastf, fr.data, fr.len), JSDK_OK);
                }
                skipped++;
            }
        }
        printf("      3c) last endpoint matched in the FINAL frame -> complete=1, "
               "stopped_early=0 (not mislabelled)\n");
        hal_close(&hf);
    }

    hal_close(&hf);
}


static void test_filtered_complete(void)
{
    hal_fix_t hf;
    cb_desc_fetch_t f;
    cb_desc_fetch_result_t res;
    jsdk_desc_config_t cfg;
    jsdk_ep_store_t store;
    static uint8_t arena[4096];
    static uint8_t raw[65536];
    sink_rec_t sink;
    drive_t drv;

    printf("[4] RETAIN_FILTERED without early stop (route B enabler)\n");

    memset(&cfg, 0, sizeof cfg);
    cfg.retain              = JSDK_DESC_RETAIN_FILTERED;
    cfg.stop_when_satisfied = 0u;               /* 路线 B 必须这样 */
    cfg.filter_paths        = k_filter_exact;
    cfg.filter_count        = 12u;
    cfg.arena                = arena;
    cfg.arena_size           = sizeof arena;

    memset(&store, 0, sizeof store);
    store.arena.base = arena;
    store.arena.size = sizeof arena;
    store.arena.blob_top = sizeof arena;

    memset(&sink, 0, sizeof sink);
    sink.buf = raw; sink.cap = sizeof raw; sink.fail_after = -1;

    CHECK_EQ(hal_open(&hf, "0:id=1,fd"), 0);
    CHECK_EQ(sim_set_desc(hf.sim, g_json, (uint32_t)g_json_len, 0x1234u), 0);
    CHECK_EQ(cb_desc_fetch_init(&f, NULL, &cfg, &store), JSDK_OK);
    cb_desc_fetch_set_raw_sink(&f, sink_cb, &sink);
    drive_fetch(&hf, &f, 200u, &drv);

    CHECK_EQ(cb_desc_fetch_is_ok(&f), 1);
    cb_desc_fetch_result(&f, &res);
    CHECK_EQ(res.complete, 1u);
    CHECK_EQ(res.stopped_early, 0u);
    CHECK_EQ(res.bytes_scanned, g_json_len);
    CHECK_EQ(res.endpoint_count, 12u);
    CHECK_EQ(res.frames_rx, 663u);              /* 全部收完 */
    /* 现在三条后置校验都成立 → 原始字节可信 */
    CHECK_EQ(cb_desc_fetch_cache_safe(&f), 1);
    CHECK_EQ(sink.len, g_json_len);
    CHECK(memcmp(raw, g_json, g_json_len) == 0);
    printf("      complete=1, %u frames, raw %u B cacheable\n",
           res.frames_rx, (unsigned)sink.len);

    hal_close(&hf);
}

/* ==========================================================================
 * 5. 失败路径
 * ======================================================================== */

/** 只喂一帧，看状态机的判定 */
static int feed_one(cb_desc_fetch_t *f, const uint8_t *payload, size_t len)
{
    return cb_desc_fetch_frame(f, payload, len);
}

static void test_failures(void)
{
    jsdk_desc_config_t cfg;
    jsdk_ep_store_t store;
    cb_desc_fetch_t f;
    static uint8_t arena[65536];
    uint8_t meta[64];
    uint8_t data[64];

    printf("[5] failure paths\n");

    memset(&cfg, 0, sizeof cfg);
    cfg.retain = JSDK_DESC_RETAIN_ALL;
    cfg.arena = arena; cfg.arena_size = sizeof arena;

#define RESET_FETCH()                                                          \
    do {                                                                       \
        memset(&store, 0, sizeof store);                                       \
        store.arena.base = arena;                                              \
        store.arena.size = sizeof arena;                                       \
        store.arena.blob_top = sizeof arena;                                   \
        CHECK_EQ(cb_desc_fetch_init(&f, NULL, &cfg, &store), JSDK_OK);          \
    } while (0)

    /* 构造一个合法元数据帧 */
    memset(meta, 0, sizeof meta);
    meta[2] = 0x45u; meta[3] = 0xA0u; meta[4] = 0x00u; meta[5] = 0x00u;  /* 41029 */
    meta[6] = 0x34u; meta[7] = 0x12u;                                    /* crc 0x1234 */

    /* --- 首帧是 offset=0 的数据帧（buf[2] == '{'）→ **跳过**，继续等元数据帧 ---
       ⚠ 旧行为是"直接失败"。真机（slcan）实测证明必须改成跳过：请求发出前后
         可能还残留上一条被中断传输的帧（其中 offset=0 的数据帧最像元数据帧），
         把它当元数据帧会报出看不懂的 total_len 错误，而且每次退出又留下新的
         残留 → 连接彻底锁死。上限见 CB_DESC_MAX_SKIPPED。 */
    RESET_FETCH();
    memset(data, 0, sizeof data);
    data[0] = 0u; data[1] = 0u; data[2] = (uint8_t)'{';
    CHECK_EQ(feed_one(&f, data, 64u), JSDK_OK);
    CHECK_EQ(cb_desc_fetch_is_done(&f), 0);
    CHECK_EQ(f.skipped_rx, 1u);
    CHECK_EQ(cb_desc_fetch_error(&f), CB_DESC_ERR_NONE);
    printf("      frame looking like data (buf[2]='{') -> skipped\n");

    /* --- 同一个坑，但描述符以 '[' 开头 —— **本固件的真实情况** ---
       （旧代码只认 '{'，于是这个交叉校验形同虚设：真机上 offset=0 的数据帧
         被当成元数据帧，报出 "total_len must be 1..65535"。） */
    memset(data, 0, sizeof data);
    data[2] = (uint8_t)'[';
    CHECK_EQ(feed_one(&f, data, 64u), JSDK_OK);
    CHECK_EQ(f.skipped_rx, 2u);
    CHECK_EQ(cb_desc_fetch_is_done(&f), 0);
    /* 关键：这一帧的拒绝理由必须是"它不是元数据帧"，而不是"total_len 越界"
       —— 后者说明交叉校验没认 `[`，只是碰巧被长度范围检查拦住（症状会变成
       真机上那句让人摸不着头脑的 "total_len must be 1..65535"）。 */
    CHECK_STR(f.err_detail, CB_DESC_ERR_META_EXPECTED);

    /* --- 元数据帧头两字节非 0 → 同样跳过 --- */
    RESET_FETCH();
    memset(data, 0, sizeof data);
    data[1] = 0x01u;
    CHECK_EQ(feed_one(&f, data, 64u), JSDK_OK);
    CHECK_EQ(f.skipped_rx, 1u);

    /* --- total_len = 0 / 65536 → 跳过（不是失败：可能只是残留帧） --- */
    RESET_FETCH();
    memset(meta, 0, sizeof meta);                       /* total_len = 0 */
    CHECK_EQ(feed_one(&f, meta, 64u), JSDK_OK);
    CHECK_EQ(f.skipped_rx, 1u);
    memset(meta, 0, sizeof meta);
    meta[2] = 0x00u; meta[3] = 0x00u; meta[4] = 0x01u; meta[5] = 0x00u;  /* 65536 */
    CHECK_EQ(feed_one(&f, meta, 64u), JSDK_OK);
    CHECK_EQ(f.skipped_rx, 2u);
    printf("      total_len 0 / 65536 -> skipped (could be a stale frame)\n");

    /* --- total_len = 65535 是上限（允许）--- */
    RESET_FETCH();
    memset(meta, 0, sizeof meta);
    meta[2] = 0xFFu; meta[3] = 0xFFu; meta[4] = 0x00u; meta[5] = 0x00u;  /* 65535 */
    CHECK_EQ(feed_one(&f, meta, 64u), JSDK_OK);
    CHECK_EQ(f.total_len, 65535u);
    printf("      total_len=65535 accepted (chunkOffset u16 limit)\n");

    /* --- **关键回归（真机现场）**：残留帧之后才来真正的元数据帧 → 必须成功 ---
       三帧模拟上一条传输的尾巴：offset=0 的 `[` 数据帧、offset=744 的数据帧。 */
    RESET_FETCH();
    memset(data, 0, sizeof data);
    data[0] = 0u; data[1] = 0u; data[2] = (uint8_t)'[';
    CHECK_EQ(feed_one(&f, data, 64u), JSDK_OK);
    memset(data, 0, sizeof data);
    data[0] = 0xE8u; data[1] = 0x02u;                    /* offset 744 */
    CHECK_EQ(feed_one(&f, data, 64u), JSDK_OK);
    memset(meta, 0, sizeof meta);                        /* 本次真正的元数据帧 */
    meta[2] = 0x45u; meta[3] = 0xA0u; meta[4] = 0x00u; meta[5] = 0x00u;  /* 41029 */
    meta[6] = 0x34u; meta[7] = 0x12u;                                    /* crc 0x1234 */
    CHECK_EQ(feed_one(&f, meta, 64u), JSDK_OK);
    CHECK_EQ(f.started, 1u);
    CHECK_EQ(f.total_len, 41029u);
    CHECK_EQ(f.crc, 0x1234u);
    CHECK_EQ(f.skipped_rx, 2u);
    CHECK_EQ(cb_desc_fetch_is_done(&f), 0);
    memset(data, 0, sizeof data);                        /* 之后 offset=0 必须被接受 */
    memcpy(&data[2], g_json, 62u);                       /* 真 JSON 片段（解析器才会接受） */
    CHECK_EQ(feed_one(&f, data, 64u), JSDK_OK);
    CHECK_EQ(f.next_offset, 62u);
    printf("      2 stale frames + real metadata -> accepted (skipped=%u)\n",
           f.skipped_rx);

    /* --- 残留太多（超上限）→ 必须失败，且原因可读、具体原因留在 err_detail --- */
    RESET_FETCH();
    {
        unsigned k;
        memset(data, 0, sizeof data);
        data[0] = 0u; data[1] = 0u; data[2] = (uint8_t)'[';
        for (k = 0u; k < CB_DESC_MAX_SKIPPED; ++k) {
            CHECK_EQ(feed_one(&f, data, 64u), JSDK_OK);
        }
        CHECK_EQ(f.skipped_rx, CB_DESC_MAX_SKIPPED);
        CHECK_EQ(feed_one(&f, data, 64u), JSDK_ERR_PROTOCOL);
        CHECK_EQ(cb_desc_fetch_is_done(&f), 1);
        CHECK_STR(cb_desc_fetch_error(&f), CB_DESC_ERR_META_SKIPPED);
        CHECK_STR(f.err_detail, CB_DESC_ERR_META_EXPECTED);
        printf("      more than %u stale frames -> %s\n", CB_DESC_MAX_SKIPPED,
               cb_desc_fetch_error(&f));
    }

    /* --- 帧长非法 --- */
    RESET_FETCH();
    CHECK_EQ(feed_one(&f, meta, 16u), JSDK_ERR_PROTOCOL);
    CHECK_STR(cb_desc_fetch_error(&f), CB_DESC_ERR_FRAME_LEN);

    /* --- chunkOffset 跳号 --- */
    RESET_FETCH();
    memset(meta, 0, sizeof meta);
    meta[2] = 0x10u; meta[3] = 0x00u; meta[4] = 0x00u; meta[5] = 0x00u;
    CHECK_EQ(feed_one(&f, meta, 64u), JSDK_OK);
    memset(data, 0, sizeof data);
    data[0] = 10u; data[1] = 0u;                 /* 应该是 0，却给了 10 */
    CHECK_EQ(feed_one(&f, data, 64u), JSDK_ERR_PROTOCOL);
    CHECK_STR(cb_desc_fetch_error(&f), CB_DESC_ERR_OFFSET);
    CHECK_EQ(jsdk_ep_store_count(&store), 0u);
    printf("      chunkOffset gap (0 -> 10) -> %s\n", cb_desc_fetch_error(&f));

    /* --- 截断：只喂一半就停 → 未 done、无部分结果入库（解析器内部状态仍在） --- */
    RESET_FETCH();
    {
        hal_fix_t hf;
        unsigned i;
        uint32_t total = (uint32_t)g_json_len;
        uint32_t sent = 0u;

        CHECK_EQ(hal_open(&hf, "0:id=1,fd"), 0);
        CHECK_EQ(sim_set_desc(hf.sim, g_json, total, 0x1234u), 0);

        memset(meta, 0, sizeof meta);
        meta[2] = (uint8_t)(total);
        meta[3] = (uint8_t)(total >> 8);
        meta[4] = (uint8_t)(total >> 16);
        meta[5] = (uint8_t)(total >> 24);
        CHECK_EQ(feed_one(&f, meta, 64u), JSDK_OK);

        /* 手动只喂前 100 帧数据帧 */
        for (i = 0u; i < 100u; ++i) {
            size_t n = g_json_len - sent;
            if (n > 62u) n = 62u;
            memset(data, 0, sizeof data);
            data[0] = (uint8_t)(sent);
            data[1] = (uint8_t)(sent >> 8);
            memcpy(&data[2], g_json + sent, n);
            CHECK_EQ(feed_one(&f, data, 64u), JSDK_OK);
            sent += (uint32_t)n;
        }
        CHECK_EQ(cb_desc_fetch_is_done(&f), 0);          /* 还没结束 */
        CHECK_EQ(cb_desc_fetch_is_ok(&f), 0);
        CHECK_EQ(cb_desc_fetch_pct(&f), (unsigned)(((uint64_t)sent * 100u) / total));

        /* 再喂到 661 帧（共 661 × 62 = 40982 B），故意差最后 47 B 不收 */
        for (i = 0u; i < 561u; ++i) {
            size_t n = g_json_len - sent;
            if (n > 62u) n = 62u;
            memset(data, 0, sizeof data);
            data[0] = (uint8_t)(sent);
            data[1] = (uint8_t)(sent >> 8);
            memcpy(&data[2], g_json + sent, n);
            CHECK_EQ(feed_one(&f, data, 64u), JSDK_OK);
            sent += (uint32_t)n;
        }
        CHECK_EQ(sent, 661u * 62u);                       /* 40982 */
        CHECK(sent < total);
        CHECK_EQ(cb_desc_fetch_is_done(&f), 0);           /* 差最后 47 B → 未结束 */
        CHECK_EQ(cb_desc_fetch_is_ok(&f), 0);
        CHECK_EQ(cb_desc_fetch_cache_safe(&f), 0);
        printf("      661/662 data frames fed: %u/%u B, not done, is_ok=0\n",
               sent, total);
        hal_close(&hf);
    }

    /* --- JSON 损坏 → PARSE --- */
    RESET_FETCH();
    {
        /* 一帧数据携带 62 字节；total_len 必须 ≥ 62，
           且帧内剩余字节要用**合法空白**补齐，否则补的 NUL 本身就是 JSON 错误，
           测到的会是“补出来的错”而不是“注入的错”。 */
        uint32_t total = 64u;
        uint8_t bad[64];
        static const char *prefix = "[{\"name\":\"a\",\"id\":1,\"type\":\"uint8\"},";
        size_t plen = strlen(prefix);

        memset(meta, 0, sizeof meta);
        meta[2] = (uint8_t)total; meta[3] = 0u; meta[4] = 0u; meta[5] = 0u;
        CHECK_EQ(feed_one(&f, meta, 64u), JSDK_OK);

        memset(bad, (uint8_t)' ', sizeof bad);     /* 用空格（合法空白）打底 */
        bad[0] = 0u; bad[1] = 0u;
        CHECK(plen <= 62u);
        memcpy(&bad[2], prefix, plen);
        CHECK_EQ(feed_one(&f, bad, 64u), JSDK_OK);  /* 第一帧本身合法 */

        memset(bad, 0, sizeof bad);
        bad[0] = 62u; bad[1] = 0u;                  /* 第二帧 offset = 62 */
        bad[2] = (uint8_t)'@';                      /* 非法字元 */
        CHECK_EQ(feed_one(&f, bad, 64u), JSDK_ERR_PARSE);
        CHECK_STR(cb_desc_fetch_error(&f), CB_DESC_ERR_PARSE);
        CHECK_EQ(jsdk_ep_store_count(&store), 0u);
        printf("      corrupt JSON -> %s (%s)\n", cb_desc_fetch_error(&f),
               cb_desc_fetch_detail(&f) ? cb_desc_fetch_detail(&f) : "-");
    }

    /* --- arena 不足 → NO_MEMORY（错误码必须可区分） --- */
    {
        static uint8_t tiny[256];
        jsdk_desc_config_t c2 = cfg;
        jsdk_ep_store_t s2;

        memset(&s2, 0, sizeof s2);
        s2.arena.base = tiny; s2.arena.size = sizeof tiny;
        s2.arena.blob_top = sizeof tiny;
        c2.arena = tiny; c2.arena_size = sizeof tiny;

        CHECK_EQ(cb_desc_fetch_init(&f, NULL, &c2, &s2), JSDK_OK);
        {
            uint32_t total = (uint32_t)g_json_len;
            memset(meta, 0, sizeof meta);
            meta[2] = (uint8_t)total; meta[3] = (uint8_t)(total >> 8);
            meta[4] = (uint8_t)(total >> 16); meta[5] = (uint8_t)(total >> 24);
            CHECK_EQ(feed_one(&f, meta, 64u), JSDK_OK);
        }
        /* 喂若干数据帧直到失败 */
        {
            unsigned i; uint32_t sent = 0u; int rc = JSDK_OK;
            for (i = 0u; i < 20u && rc == JSDK_OK; ++i) {
                memset(data, 0, sizeof data);
                data[0] = (uint8_t)sent; data[1] = (uint8_t)(sent >> 8);
                memcpy(&data[2], g_json + sent, 62u);
                rc = feed_one(&f, data, 64u);
                sent += 62u;
            }
            CHECK_EQ(rc, JSDK_ERR_NO_MEMORY);
            CHECK_STR(cb_desc_fetch_error(&f), CB_DESC_ERR_ARENA);
            CHECK_EQ(jsdk_ep_store_count(&s2), 0u);     /* 关键：不留部分结果 */
            printf("      tiny arena -> JSDK_ERR_NO_MEMORY (%s), store empty\n",
                   cb_desc_fetch_error(&f));
        }
    }

    /* --- NULL 防护 --- */
    CHECK_EQ(cb_desc_fetch_frame(NULL, meta, 64u), JSDK_ERR_INVALID_ARG);
    CHECK_EQ(cb_desc_build_request(NULL, 4u, 0u), 0u);
    CHECK_EQ(cb_desc_build_request(meta, 3u, 0u), 0u);
    CHECK_STR(cb_desc_fetch_error(NULL), CB_DESC_ERR_PARSE);
    {
        cb_desc_fetch_result_t r;
        cb_desc_fetch_result(&f, NULL);          /* 不得崩溃 */
        cb_desc_fetch_result(NULL, &r);
        CHECK_EQ(r.total_len, 0u);
    }
    CHECK_EQ(cb_desc_fetch_cache_safe(NULL), 0);
    CHECK_EQ(cb_desc_frame_len_valid(8u), 1);
    CHECK_EQ(cb_desc_frame_len_valid(64u), 1);
    CHECK_EQ(cb_desc_frame_len_valid(16u), 0);
    CHECK_EQ(cb_desc_timeout_ms(NULL), 5000u);
    {
        jsdk_desc_config_t c3;
        memset(&c3, 0, sizeof c3);
        CHECK_EQ(cb_desc_timeout_ms(&c3), 5000u);
        c3.timeout_ms = 1234u;
        CHECK_EQ(cb_desc_timeout_ms(&c3), 1234u);
    }

#undef RESET_FETCH
}

/* ==========================================================================
 * 6~7. 缓存往返与失效键
 * ======================================================================== */

static void test_cache(void)
{
    jsdk_desc_config_t cfg;
    jsdk_ep_store_t a, b;
    static uint8_t arena_a[65536];
    static uint8_t arena_b[65536];
    static uint8_t cache[65536];
    size_t clen = 0u;
    hal_fix_t hf;
    cb_desc_fetch_t f;
    drive_t drv;
    cb_desc_cache_meta_t meta;

    printf("[6] cache export / import\n");

    memset(&cfg, 0, sizeof cfg);
    cfg.retain        = JSDK_DESC_RETAIN_ALL;
    cfg.max_path_len  = 128u;
    cfg.arena         = arena_a;
    cfg.arena_size    = sizeof arena_a;

    memset(&a, 0, sizeof a);
    a.arena.base = arena_a; a.arena.size = sizeof arena_a;
    a.arena.blob_top = sizeof arena_a;

    CHECK_EQ(hal_open(&hf, "0:id=1,fd"), 0);
    CHECK_EQ(sim_set_desc(hf.sim, g_json, (uint32_t)g_json_len, 0xABCDu), 0);
    CHECK_EQ(cb_desc_fetch_init(&f, NULL, &cfg, &a), JSDK_OK);
    drive_fetch(&hf, &f, 200u, &drv);
    CHECK_EQ(cb_desc_fetch_is_ok(&f), 1);
    hal_close(&hf);

    CHECK_EQ(jsdk_ep_store_count(&a), 594u);

    /* --- 导出 --- */
    {
        size_t need = cb_desc_cache_size(&a);
        CHECK(need > CB_DESC_CACHE_HDR_LEN);
        CHECK(need < sizeof cache);
        CHECK_EQ(cb_desc_cache_export(&cfg, &a, 0, cache, sizeof cache, &clen), JSDK_OK);
        CHECK_EQ(clen, need);
        /* cap 不足 → NO_MEMORY，且不写坏缓冲 */
        CHECK_EQ(cb_desc_cache_export(&cfg, &a, 0, cache, need - 1u, NULL),
                 JSDK_ERR_NO_MEMORY);
        CHECK_EQ(cb_desc_cache_export(NULL, &a, 0, cache, sizeof cache, NULL),
                 JSDK_ERR_INVALID_ARG);
        printf("      exported %u endpoints -> %u B (arena was %u B)\n",
               594u, (unsigned)clen, 25493u);
    }

    /* --- 头部信息 --- */
    CHECK_EQ(cb_desc_cache_peek(cache, clen, &meta), JSDK_OK);
    CHECK_EQ(meta.endpoint_count, 594u);
    CHECK_EQ(meta.fw_version, 0u);
    CHECK_EQ(meta.desc_crc, 0u);
    CHECK_EQ(cb_desc_cache_export(&cfg, &a, 1, cache, sizeof cache, &clen), JSDK_OK);
    CHECK_EQ(cb_desc_cache_peek(cache, clen, &meta), JSDK_OK);
    CHECK((meta.flags & CB_DESC_CACHE_F_EARLY) != 0u);
    CHECK((meta.flags & CB_DESC_CACHE_F_FILTERED) == 0u);
    /* 补写两个键 */
    CHECK_EQ(cb_desc_cache_set_fw_version(cache, clen, 0x00050607u), JSDK_OK);
    CHECK_EQ(cb_desc_cache_set_desc_crc(cache, clen, 0xABCDu), JSDK_OK);
    CHECK_EQ(cb_desc_cache_peek(cache, clen, &meta), JSDK_OK);
    CHECK_EQ(meta.fw_version, 0x00050607u);
    CHECK_EQ(meta.desc_crc, 0xABCDu);
    /* 补写后再导出（不带 EARLY 位）作为后续导入用例 */
    CHECK_EQ(cb_desc_cache_export(&cfg, &a, 0, cache, sizeof cache, &clen), JSDK_OK);
    CHECK_EQ(cb_desc_cache_set_fw_version(cache, clen, 0x00050607u), JSDK_OK);

    /* --- 导入到**不同大小**的 arena（更小但够用） --- */
    {
        static uint8_t arena_small[26000];
        jsdk_ep_store_t s;
        memset(&s, 0, sizeof s);
        s.arena.base = arena_small; s.arena.size = sizeof arena_small;
        s.arena.blob_top = sizeof arena_small;
        CHECK_EQ(cb_desc_cache_import(&cfg, &s, cache, clen), JSDK_OK);
        CHECK_EQ(jsdk_ep_store_count(&s), 594u);
        /* 逐条比对 */
        {
            unsigned i; int same = 1;
            for (i = 0u; i < 594u; ++i) {
                const char *pa = jsdk_ep_store_path(&a, i);
                const char *pb = jsdk_ep_store_path(&s, i);
                uint16_t ia = 0u, ib = 0u;
                if (jsdk_ep_store_at(&a, i, NULL, &ia, NULL, NULL) != JSDK_OK ||
                    jsdk_ep_store_at(&s, i, NULL, &ib, NULL, NULL) != JSDK_OK ||
                    !pa || !pb || strcmp(pa, pb) != 0 || ia != ib) { same = 0; break; }
            }
            CHECK_EQ(same, 1);
        }
        printf("      imported into a 26000 B arena (was 25493 B needed): identical\n");
    }

    /* --- 导入到原 arena --- */
    memset(&b, 0, sizeof b);
    b.arena.base = arena_b; b.arena.size = sizeof arena_b;
    b.arena.blob_top = sizeof arena_b;
    CHECK_EQ(cb_desc_cache_import(&cfg, &b, cache, clen), JSDK_OK);
    CHECK_EQ(jsdk_ep_store_count(&b), 594u);
    {
        uint16_t id = 0u;
        CHECK_EQ(jsdk_ep_store_lookup(&b, "axis0.motor.config.gear_ratio",
                                      &id, NULL, NULL), JSDK_OK);
        CHECK_EQ(id, 242u);
        CHECK_EQ(jsdk_ep_store_lookup(&b, "can.config.break_timeout",
                                      &id, NULL, NULL), JSDK_OK);
        CHECK_EQ(id, 73u);
    }

    /* --- 失效键：改动任一项都必须拒绝 --- */
    {
        jsdk_desc_config_t c;

        /* retain 变了 */
        c = cfg; c.retain = JSDK_DESC_RETAIN_FILTERED;
        c.filter_paths = k_filter_wild; c.filter_count = 12u;
        CHECK_EQ(cb_desc_cache_import(&c, &b, cache, clen), JSDK_ERR_BAD_STATE);

        /* filter 变了（顺序也敏感） */
        c = cfg;
        {
            static const char *const f1[] = { "vbus_voltage" };
            c.filter_paths = f1; c.filter_count = 1u;
        }
        CHECK_EQ(cb_desc_cache_import(&c, &b, cache, clen), JSDK_ERR_BAD_STATE);

        /* max_path_len 变了 */
        c = cfg; c.max_path_len = 64u;
        CHECK_EQ(cb_desc_cache_import(&c, &b, cache, clen), JSDK_ERR_BAD_STATE);

        /* max_endpoints 变了 */
        c = cfg; c.max_endpoints = 100u;
        CHECK_EQ(cb_desc_cache_import(&c, &b, cache, clen), JSDK_ERR_BAD_STATE);

        printf("      invalidation keys: retain / filter / limits all rejected\n");
    }
    /* 失效后 store 必须被清空，不得留下上一份的残留 */
    CHECK_EQ(jsdk_ep_store_count(&b), 0u);

    /* --- 完整性 / 版本 / 长度 --- */
    {
        static uint8_t bad[65536];
        memcpy(bad, cache, clen);
        bad[CB_DESC_CACHE_HDR_LEN + 7u] ^= 0x01u;         /* 翻转体内 1 bit */
        CHECK_EQ(cb_desc_cache_import(&cfg, &b, bad, clen), JSDK_ERR_PROTOCOL);
        CHECK_EQ(jsdk_ep_store_count(&b), 0u);
        printf("      single-bit flip in body -> JSDK_ERR_PROTOCOL (body crc)\n");

        memcpy(bad, cache, clen);
        bad[0] ^= 0xFFu;                                  /* 魔数坏 */
        CHECK_EQ(cb_desc_cache_import(&cfg, &b, bad, clen), JSDK_ERR_PROTOCOL);

        memcpy(bad, cache, clen);
        bad[4] = 0xFFu;                                   /* 版本 = 255 */
        CHECK_EQ(cb_desc_cache_import(&cfg, &b, bad, clen), JSDK_ERR_PROTOCOL);

        CHECK_EQ(cb_desc_cache_import(&cfg, &b, cache, CB_DESC_CACHE_HDR_LEN - 1u),
                 JSDK_ERR_INVALID_ARG);
        CHECK_EQ(cb_desc_cache_import(&cfg, &b, cache, clen - 1u), JSDK_ERR_PROTOCOL);
        CHECK_EQ(cb_desc_cache_import(NULL, &b, cache, clen), JSDK_ERR_INVALID_ARG);
        CHECK_EQ(cb_desc_cache_import(&cfg, NULL, cache, clen), JSDK_ERR_INVALID_ARG);

        CHECK_EQ(cb_desc_cache_peek(bad, clen, &meta), JSDK_ERR_PROTOCOL);
        CHECK_EQ(cb_desc_cache_peek(cache, clen, NULL), JSDK_OK);
    }

    /* --- filter hash 的稳定性与敏感性 --- */
    {
        uint32_t h1 = cb_desc_filter_hash(&cfg);
        uint32_t h2 = cb_desc_filter_hash(&cfg);
        jsdk_desc_config_t c2 = cfg;

        CHECK_EQ(h1, h2);
        c2.max_path_len = 129u;
        CHECK(cb_desc_filter_hash(&c2) != h1);
        c2 = cfg;
        c2.retain = JSDK_DESC_RETAIN_FILTERED;
        c2.filter_paths = k_filter_wild; c2.filter_count = 12u;
        CHECK(cb_desc_filter_hash(&c2) != h1);
        CHECK_EQ(cb_desc_filter_hash(NULL), 0u);
        printf("      filter_hash = 0x%08X (stable, sensitive to retain/filter/limits)\n",
               h1);
    }

    /* --- 导出到 arena 装不下的情况：报 NO_MEMORY 而不是截断 --- */
    {
        static uint8_t tiny[512];
        jsdk_ep_store_t s;
        memset(&s, 0, sizeof s);
        s.arena.base = tiny; s.arena.size = sizeof tiny; s.arena.blob_top = sizeof tiny;
        CHECK_EQ(cb_desc_cache_import(&cfg, &s, cache, clen), JSDK_ERR_NO_MEMORY);
        CHECK_EQ(jsdk_ep_store_count(&s), 0u);
        printf("      import into 512 B arena -> JSDK_ERR_NO_MEMORY, store empty\n");
    }

    /* --- 路线 B：import_raw 与逐帧解析结果一致 --- */
    {
        jsdk_ep_store_t s;
        memset(&s, 0, sizeof s);
        s.arena.base = arena_b; s.arena.size = sizeof arena_b;
        s.arena.blob_top = sizeof arena_b;

        CHECK_EQ(cb_desc_import_raw(&cfg, &s, g_json, g_json_len), JSDK_OK);
        CHECK_EQ(jsdk_ep_store_count(&s), 594u);
        CHECK_EQ(s.arena.blob_used, a.arena.blob_used);
        CHECK(memcmp(arena_b + s.arena.blob_top, arena_a + a.arena.blob_top,
                     s.arena.blob_used) == 0);
        CHECK(memcmp(arena_b, arena_a, 594u * 8u) == 0);

        /* 截断的 JSON → PARSE；arena 不足 → NO_MEMORY */
        CHECK_EQ(cb_desc_import_raw(&cfg, &s, g_json, g_json_len - 1u), JSDK_ERR_PARSE);
        CHECK_EQ(jsdk_ep_store_count(&s), 0u);
        CHECK_EQ(cb_desc_import_raw(&cfg, &s, NULL, 10u), JSDK_ERR_INVALID_ARG);
        CHECK_EQ(cb_desc_import_raw(&cfg, &s, g_json, 0u), JSDK_ERR_INVALID_ARG);
        printf("      import_raw: identical arena to frame-by-frame parse\n");
    }

    /* --- 上限的“生效值”语义：0 与显式默认值必须等价 --- */
    {
        jsdk_desc_config_t c3 = cfg;                 /* cfg 里两项都是 0（默认） */
        uint32_t h0 = cb_desc_filter_hash(&cfg);
        jsdk_ep_store_t b3;

        c3.max_endpoints = 2048u;                    /* 显式写默认值 */
        c3.max_path_len  = 128u;
        CHECK_EQ(cb_desc_eff_max_endpoints(&cfg), 2048u);
        CHECK_EQ(cb_desc_eff_max_path_len(&cfg), 128u);
        CHECK_EQ(cb_desc_filter_hash(&c3), h0);      /* 同一配置 → 同一个键 */

        memset(&b3, 0, sizeof b3);
        b3.arena.base = arena_b; b3.arena.size = sizeof arena_b;
        b3.arena.blob_top = sizeof arena_b;
        CHECK_EQ(cb_desc_cache_import(&c3, &b3, cache, clen), JSDK_OK);
        CHECK_EQ(jsdk_ep_store_count(&b3), 594u);
        printf("      explicit defaults == 0 (hash and import both agree)\n");
    }

    /* --- 空表：只导出头，导入得到 0 条 --- */
    {
        static uint8_t hdr_only[CB_DESC_CACHE_HDR_LEN + 8u];
        jsdk_ep_store_t empty, back;
        size_t hn = 0u;

        memset(&empty, 0, sizeof empty);
        empty.arena.base = arena_b; empty.arena.size = sizeof arena_b;
        empty.arena.blob_top = sizeof arena_b;

        CHECK_EQ(cb_desc_cache_size(&empty), (size_t)CB_DESC_CACHE_HDR_LEN);
        /* out_len 可传 NULL：此时只能靠返回值判断容量 */
        CHECK_EQ(cb_desc_cache_export(&cfg, &empty, 0, hdr_only, 8u, NULL),
                 JSDK_ERR_NO_MEMORY);
        CHECK_EQ(cb_desc_cache_export(NULL, &empty, 0, hdr_only, sizeof hdr_only, &hn),
                 JSDK_ERR_INVALID_ARG);
        CHECK_EQ(cb_desc_cache_export(&cfg, NULL, 0, hdr_only, sizeof hdr_only, &hn),
                 JSDK_ERR_INVALID_ARG);
        CHECK_EQ(cb_desc_cache_export(&cfg, &empty, 0, NULL, sizeof hdr_only, &hn),
                 JSDK_ERR_INVALID_ARG);
        CHECK_EQ(cb_desc_cache_export(&cfg, &empty, 0, hdr_only, 8u, &hn),
                 JSDK_ERR_NO_MEMORY);
        CHECK_EQ(hn, 0u);
        CHECK_EQ(cb_desc_cache_export(&cfg, &empty, 0, hdr_only, sizeof hdr_only, &hn),
                 JSDK_OK);
        CHECK_EQ(hn, (size_t)CB_DESC_CACHE_HDR_LEN);

        memset(&back, 0, sizeof back);
        back.arena.base = arena_b; back.arena.size = sizeof arena_b;
        back.arena.blob_top = sizeof arena_b;
        CHECK_EQ(cb_desc_cache_import(&cfg, &back, hdr_only, hn), JSDK_OK);
        CHECK_EQ(jsdk_ep_store_count(&back), 0u);
        printf("      empty store round-trips (header only, %u B)\n", (unsigned)hn);
    }
}

/* ==========================================================================
 * main
 * ======================================================================== */

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("=== descriptor fetch + cache tests (WP2b transport) ===\n\n");

    if (load_fixture() != 0) {
        printf("FATAL: 无法读取 %s\n", FIXTURE_PATH);
        return 1;
    }
    printf("fixture: %u bytes\n\n", (unsigned)g_json_len);

    test_full_fetch();        printf("\n");
    test_filtered_stop();     printf("\n");
    test_filtered_complete(); printf("\n");
    test_failures();          printf("\n");
    test_cache();

    free(g_json);
    printf("\n=== %d checks, %d failures ===\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
