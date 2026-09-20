/**
 * @file    test_jsondesc.c
 * @brief   JSON 端点描述符增量解析器的回归测试
 *
 * 黄金向量：tests/data/endpoints_v8.json（由 tools/extract_endpoints_json.py 从
 * ODrive 固件 Firmware/autogen/endpoints.hpp 提取，即**设备实际回送的字节**）。
 *
 * 覆盖：
 *   1. 全量解析 41029 字节 → 594 个端点，逐项核对关键端点 ID
 *   2. 逐字节喂入 与 整块喂入 结果完全一致
 *   3. 截断 / 未知 type / 非法字面量 → 失败且不留部分结果
 *   4. RETAIN_FILTERED（精确 + 前缀）→ 只保留命中项，arena 预算可控
 *   5. arena 不足 → 失败且不留部分结果
 *   6. stop_when_satisfied 的命中统计
 *   7. 不透明类型（endpoint_ref / json / function）与缺省 access
 */

#include "jsdk_internal.h"

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

/* ==========================================================================
 * 夹具加载
 * ======================================================================== */

static char  *g_json;
static size_t g_json_len;

static int load_fixture(void)
{
    FILE *fp = fopen(FIXTURE_PATH, "rb");
    long  n;

    if (!fp) {
        printf("  ! cannot open fixture %s\n", FIXTURE_PATH);
        printf("    run: python tools/extract_endpoints_json.py "
               "--hpp ../ODrive/Firmware/autogen/endpoints.hpp\n");
        return -1;
    }
    fseek(fp, 0, SEEK_END);
    n = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if (n <= 0) { fclose(fp); return -1; }

    g_json = (char *)malloc((size_t)n + 1u);
    if (!g_json) { fclose(fp); return -1; }
    if (fread(g_json, 1, (size_t)n, fp) != (size_t)n) { fclose(fp); free(g_json); return -1; }
    g_json[n] = '\0';
    g_json_len = (size_t)n;
    fclose(fp);
    return 0;
}

/* ==========================================================================
 * 辅助
 * ======================================================================== */

typedef struct {
    uint8_t *mem;
    size_t   cap;
    jsdk_ep_store_t store;
    jsdk_desc_config_t cfg;
} test_env_t;

static void env_setup(test_env_t *e, size_t cap, uint8_t retain)
{
    memset(e, 0, sizeof *e);
    e->cap = cap;
    e->mem = (uint8_t *)malloc(cap);
    memset(e->mem, 0xA5, cap);              /* 预填垃圾，验证解析器确实写了值 */
    e->store.arena.base = e->mem;
    e->store.arena.size = cap;
    e->store.arena.blob_top = cap;
    e->cfg.retain = retain;
    e->cfg.arena = e->mem;
    e->cfg.arena_size = cap;
    e->cfg.max_endpoints = 2048;
    e->cfg.max_path_len = 128;
}

static void env_teardown(test_env_t *e)
{
    free(e->mem);
    e->mem = NULL;
}

static void expect_id(test_env_t *e, const char *path, long want)
{
    uint16_t id = 0xFFFF;
    jsdk_ep_type_t t = (jsdk_ep_type_t)0;
    uint8_t acc = 0;
    int rc = jsdk_ep_store_lookup(&e->store, path, &id, &t, &acc);
    g_checks++;
    if (rc != JSDK_OK) {
        printf("  FAIL lookup(\"%s\") -> rc=%d (expected id %ld)\n", path, rc, want);
        g_fail++;
        return;
    }
    if ((long)id != want) {
        printf("  FAIL lookup(\"%s\") id=%u, expected %ld\n", path, id, want);
        g_fail++;
        return;
    }
}

/* ==========================================================================
 * 1. 全量解析
 * ======================================================================== */

static void test_full_parse(void)
{
    test_env_t e;
    jsdk_ep_type_t t;
    uint8_t acc;
    uint16_t id;
    unsigned i, n_fn = 0, n_ref = 0, n_json = 0, n_obj = 0, n_bool = 0, n_f32 = 0;
    unsigned n_no_access = 0;

    printf("[1] full parse (RETAIN_ALL)\n");
    env_setup(&e, 65536u, JSDK_DESC_RETAIN_ALL);

    CHECK_EQ(jsdk_jsondesc_run(&e.cfg, &e.store, g_json, g_json_len), JSDK_OK);
    CHECK_EQ(e.store.arena.entry_count, 594u);
    CHECK_EQ(e.store.parsed_total, 594u);

    /* arena 用量应与 tools/extract_endpoints_json.py 的预算一致 */
    CHECK_EQ(e.store.arena.blob_used, 20741u);
    printf("      arena used = %u (entries %u x 8 + path pool %u)\n",
           (unsigned)(e.store.arena.entry_count * 8u + e.store.arena.blob_used),
           e.store.arena.entry_count, (unsigned)e.store.arena.blob_used);
    CHECK(e.store.arena.entry_count * 8u + e.store.arena.blob_used <= 65536u);

    /* 关键端点（v8 实测值，与 PROTOCOL_NOTES §9.1 一致） */
    expect_id(&e, "axis0.motor.config.gear_ratio", 242);
    expect_id(&e, "axis0.motor.config.torque_constant", 247);
    expect_id(&e, "axis0.motor.config.current_lim", 249);
    expect_id(&e, "axis0.motor.config.torque_lim", 251);
    expect_id(&e, "axis0.motor.config.pole_pairs", 241);
    expect_id(&e, "axis0.controller.config.mit_max_pos", 335);
    expect_id(&e, "axis0.controller.config.mit_max_vel", 336);
    expect_id(&e, "axis0.controller.config.mit_max_torque", 337);
    expect_id(&e, "axis0.controller.config.mit_max_kp", 338);
    expect_id(&e, "axis0.controller.config.mit_max_kd", 339);
    expect_id(&e, "axis0.controller.config.control_mode", 287);
    expect_id(&e, "axis0.controller.config.input_mode", 288);
    expect_id(&e, "axis0.controller.config.vel_limit", 301);
    expect_id(&e, "axis0.requested_state", 143);
    expect_id(&e, "axis0.current_state", 142);
    expect_id(&e, "axis0.config.can.node_id", 180);
    expect_id(&e, "axis0.config.can.heartbeat_rate_ms", 182);
    expect_id(&e, "axis0.error", 138);
    expect_id(&e, "axis0.motor.error", 193);
    expect_id(&e, "axis0.controller.error", 268);
    expect_id(&e, "axis0.encoder.config.cpr", 390);
    expect_id(&e, "can.config.break_timeout", 73);
    expect_id(&e, "serial_number", 5);
    expect_id(&e, "axis0.steps", 141);

    /* 根节点：空路径 + id 0 + type json */
    CHECK_EQ(jsdk_ep_store_lookup(&e.store, "", &id, &t, &acc), JSDK_OK);
    CHECK_EQ(id, 0u);
    CHECK_EQ((int)t, (int)JSDK_EP_JSON);

    /* 不透明类型与缺省 access 的统计 */
    for (i = 0; i < jsdk_ep_store_count(&e.store); ++i) {
        const char *path = NULL;
        uint8_t a = 0;
        jsdk_ep_type_t ty = (jsdk_ep_type_t)0;
        CHECK_EQ(jsdk_ep_store_at(&e.store, i, &path, &id, &ty, &a), JSDK_OK);
        CHECK(path != NULL);
        switch (ty) {
        case JSDK_EP_FUNCTION:     n_fn++;   break;
        case JSDK_EP_ENDPOINT_REF: n_ref++;  break;
        case JSDK_EP_JSON:         n_json++; break;
        case JSDK_EP_OBJECT:       n_obj++;  break;
        case JSDK_EP_F32:          n_f32++;  break;
        case JSDK_EP_BOOL:         n_bool++; break;
        default:                             break;
        }
        if ((ty == JSDK_EP_FUNCTION) && a == 0u) n_no_access++;
        if (ty != JSDK_EP_FUNCTION && ty != JSDK_EP_JSON)
            CHECK((a & (JSDK_EP_ACCESS_R | JSDK_EP_ACCESS_W)) != 0u);
    }
    CHECK_EQ(n_fn, 30u);
    CHECK_EQ(n_ref, 6u);
    CHECK_EQ(n_json, 1u);
    CHECK_EQ(n_obj, 0u);   /* 66 个 object 容器均无 id，不会成为端点 */
    CHECK_EQ(n_f32, 247u);
    CHECK_EQ(n_bool, 88u);
    CHECK_EQ(n_no_access, 30u);   /* 30 条 function 均无 access 字段 */

    /* 不透明类型的可读写性判定 */
    CHECK(jsdk_ep_type_is_scalar(JSDK_EP_F32));
    CHECK(jsdk_ep_type_is_scalar(JSDK_EP_BOOL));
    CHECK(!jsdk_ep_type_is_scalar(JSDK_EP_JSON));
    CHECK(!jsdk_ep_type_is_scalar(JSDK_EP_FUNCTION));
    CHECK(!jsdk_ep_type_is_scalar(JSDK_EP_ENDPOINT_REF));
    CHECK_EQ(jsdk_ep_type_size(JSDK_EP_F32), 4u);
    CHECK_EQ(jsdk_ep_type_size(JSDK_EP_U16), 2u);
    CHECK_EQ(jsdk_ep_type_size(JSDK_EP_U64), 8u);
    CHECK_EQ(jsdk_ep_type_size(JSDK_EP_FUNCTION), 0u);

    /* 未命中 */
    CHECK_EQ(jsdk_ep_store_lookup(&e.store, "axis0.no.such.path", &id, &t, &acc),
             JSDK_ERR_NOT_FOUND);

    printf("      594 endpoints: function=%u endpoint_ref=%u json=%u float=%u bool=%u\n",
           n_fn, n_ref, n_json, n_f32, n_bool);
    env_teardown(&e);
}

/* ==========================================================================
 * 2. 逐字节喂入 == 整块喂入
 * ======================================================================== */

static void test_incremental_equivalence(void)
{
    test_env_t a, b;
    jsdk_jsondesc_t p;
    size_t i;
    int rc;

    printf("[2] byte-by-byte feed == whole-buffer feed\n");
    env_setup(&a, 65536u, JSDK_DESC_RETAIN_ALL);
    env_setup(&b, 65536u, JSDK_DESC_RETAIN_ALL);

    CHECK_EQ(jsdk_jsondesc_run(&a.cfg, &a.store, g_json, g_json_len), JSDK_OK);

    rc = jsdk_jsondesc_init(&p, &b.cfg, &b.store);
    CHECK_EQ(rc, JSDK_OK);
    for (i = 0; i < g_json_len && rc == JSDK_OK; ++i)
        rc = jsdk_jsondesc_feed(&p, g_json + i, 1);
    CHECK_EQ(rc, JSDK_OK);
    CHECK_EQ(jsdk_jsondesc_finish(&p), JSDK_OK);

    CHECK_EQ(a.store.arena.entry_count, b.store.arena.entry_count);
    CHECK_EQ(a.store.arena.blob_used, b.store.arena.blob_used);
    CHECK_EQ(a.store.arena.blob_top, b.store.arena.blob_top);
    CHECK(memcmp(a.mem, b.mem, a.store.arena.entry_count * 8u) == 0);
    CHECK(memcmp(a.mem + a.store.arena.blob_top, b.mem + b.store.arena.blob_top,
                 a.store.arena.blob_used) == 0);

    /* 再按 62 字节（CAN FD 的 JSON 载荷）分块喂一次 */
    env_setup(&b, 65536u, JSDK_DESC_RETAIN_ALL);
    rc = jsdk_jsondesc_init(&p, &b.cfg, &b.store);
    CHECK_EQ(rc, JSDK_OK);
    for (i = 0; i < g_json_len && rc == JSDK_OK; i += 62u) {
        size_t n = g_json_len - i;
        if (n > 62u) n = 62u;
        rc = jsdk_jsondesc_feed(&p, g_json + i, n);
    }
    CHECK_EQ(rc, JSDK_OK);
    CHECK_EQ(jsdk_jsondesc_finish(&p), JSDK_OK);
    CHECK_EQ(a.store.arena.entry_count, b.store.arena.entry_count);
    CHECK(memcmp(a.mem + a.store.arena.blob_top, b.mem + b.store.arena.blob_top,
                 a.store.arena.blob_used) == 0);

    env_teardown(&a);
    env_teardown(&b);
}

/* ==========================================================================
 * 3. 失败路径：截断 / 未知 type / 非法字面量 → 不留部分结果
 * ======================================================================== */

static void expect_parse_failure(const char *label, const char *json, size_t len)
{
    test_env_t e;
    jsdk_jsondesc_t p;
    int rc;

    env_setup(&e, 65536u, JSDK_DESC_RETAIN_ALL);
    rc = jsdk_jsondesc_init(&p, &e.cfg, &e.store);
    g_checks++;
    if (rc != JSDK_OK) { printf("  FAIL %s: init rc=%d\n", label, rc); g_fail++; }

    rc = jsdk_jsondesc_feed(&p, json, len);
    if (rc == JSDK_OK) rc = jsdk_jsondesc_finish(&p);

    g_checks++;
    if (rc == JSDK_OK) {
        printf("  FAIL %s: expected failure but succeeded\n", label);
        g_fail++;
    } else {
        printf("      %-22s -> %s (%s)\n", label,
               rc == JSDK_ERR_PARSE ? "JSDK_ERR_PARSE" : "other error",
               jsdk_jsondesc_error(&p) ? jsdk_jsondesc_error(&p) : "?");
        /* 失败必须不留部分结果 */
        CHECK_EQ(jsdk_ep_store_count(&e.store), 0u);
    }
    env_teardown(&e);
}

static void test_failures(void)
{
    printf("[3] failure paths\n");
    expect_parse_failure("truncated (no ']')", "[{\"name\":\"a\",\"id\":1,\"type\":\"float\"", 35);
    expect_parse_failure("empty input", "", 0);
    expect_parse_failure("root not array", "{\"name\":\"a\"}", 13);
    expect_parse_failure("unknown type", "[{\"name\":\"a\",\"id\":1,\"type\":\"bogus\"}]", 38);
    expect_parse_failure("id > 65535", "[{\"name\":\"a\",\"id\":70000,\"type\":\"uint8\"}]", 42);
    expect_parse_failure("id is float", "[{\"name\":\"a\",\"id\":1.5,\"type\":\"uint8\"}]", 39);
    expect_parse_failure("bad access", "[{\"name\":\"a\",\"id\":1,\"type\":\"uint8\",\"access\":\"x\"}]", 56);
    expect_parse_failure("bad literal", "[{\"name\":\"a\",\"id\":1,\"type\":\"uint8\",\"x\":tru}]", 47);
    expect_parse_failure("trailing data", "[{\"name\":\"a\",\"id\":1,\"type\":\"uint8\"}] junk", 44);
    expect_parse_failure("object without name", "[{\"id\":1,\"type\":\"uint8\"}]", 25);
    expect_parse_failure("mismatched brackets", "[{\"name\":\"a\",\"id\":1,\"type\":\"uint8\"]}", 39);
    expect_parse_failure("unsupported escape", "[{\"name\":\"a\\u0041\",\"id\":1,\"type\":\"uint8\"}]", 42);
}

/* 真实描述符的截断（最有可能的现场故障：RX 丢帧） */
static void test_truncated_fixture(void)
{
    size_t cuts[] = { 41029u / 4u, 41029u / 2u, 24655u, 41028u };
    size_t i;

    printf("[3b] truncated real descriptor (simulating dropped frames)\n");
    for (i = 0; i < sizeof cuts / sizeof cuts[0]; ++i) {
        char label[48];
        sprintf(label, "truncated at %u bytes", (unsigned)cuts[i]);
        expect_parse_failure(label, g_json, cuts[i]);
    }
}

/* ==========================================================================
 * 4. RETAIN_FILTERED：精确 + 前缀
 * ======================================================================== */

static const char *const k_required[] = {
    "axis0.motor.config.gear_ratio",
    "axis0.motor.config.torque_constant",
    "axis0.controller.config.mit_max_pos",
    "axis0.controller.config.mit_max_vel",
    "axis0.controller.config.mit_max_torque",
    "axis0.controller.config.mit_max_kp",
    "axis0.controller.config.mit_max_kd",
    "axis0.requested_state",
    "axis0.current_state",
    "axis0.config.can.node_id",
    "axis0.config.can.heartbeat_rate_ms",
    "can.config.break_timeout"
};

static void test_filtered(void)
{
    test_env_t e;
    jsdk_jsondesc_t p;
    uint16_t id;
    jsdk_ep_type_t t;
    uint8_t acc;
    size_t rec;

    printf("[4] RETAIN_FILTERED (12 required paths)\n");
    env_setup(&e, 4096u, JSDK_DESC_RETAIN_FILTERED);
    e.cfg.filter_paths = k_required;
    e.cfg.filter_count = (unsigned)(sizeof k_required / sizeof k_required[0]);

    rec = jsdk_desc_arena_size(&e.cfg);
    printf("      jsdk_desc_arena_size 推荐 = %u B\n", (unsigned)rec);

    CHECK_EQ(jsdk_jsondesc_init(&p, &e.cfg, &e.store), JSDK_OK);
    CHECK_EQ(jsdk_jsondesc_feed(&p, g_json, g_json_len), JSDK_OK);
    CHECK_EQ(jsdk_jsondesc_finish(&p), JSDK_OK);

    CHECK_EQ(e.store.arena.entry_count, 12u);
    CHECK_EQ(e.store.parsed_total, 594u);          /* 仍在扫描全量 */
    CHECK_EQ(jsdk_jsondesc_filter_hits(&p), 12u);
    CHECK(jsdk_jsondesc_satisfied(&p));

    /* 保留项的 ID 必须与全量解析一致 */
    expect_id(&e, "axis0.motor.config.gear_ratio", 242);
    expect_id(&e, "axis0.controller.config.mit_max_torque", 337);
    expect_id(&e, "can.config.break_timeout", 73);

    /* 未保留的路径必须查不到（这正是 route A 缓存需要 filter_hash 的原因） */
    CHECK_EQ(jsdk_ep_store_lookup(&e.store, "axis0.controller.config.vel_limit",
                                  &id, &t, &acc), JSDK_ERR_NOT_FOUND);

    rec = e.store.arena.entry_count * 8u + e.store.arena.blob_used;
    printf("      actual usage = %u B, recommendation = %u B\n",
           (unsigned)rec, (unsigned)jsdk_desc_arena_size(&e.cfg));
    CHECK(rec <= jsdk_desc_arena_size(&e.cfg));
    CHECK(rec < 512u);

    env_teardown(&e);

    /* --- 前缀匹配 --- */
    {
        static const char *const prefix[] = { "axis0.controller.config.mit_max_*" };
        test_env_t f;
        jsdk_jsondesc_t q;
        unsigned i, n = 0;

        printf("[4b] RETAIN_FILTERED (prefix match)\n");
        env_setup(&f, 4096u, JSDK_DESC_RETAIN_FILTERED);
        f.cfg.filter_paths = prefix;
        f.cfg.filter_count = 1;

        CHECK_EQ(jsdk_jsondesc_init(&q, &f.cfg, &f.store), JSDK_OK);
        CHECK_EQ(jsdk_jsondesc_feed(&q, g_json, g_json_len), JSDK_OK);
        CHECK_EQ(jsdk_jsondesc_finish(&q), JSDK_OK);
        CHECK_EQ(f.store.arena.entry_count, 5u);   /* mit_max_pos/vel/torque/kp/kd */
        for (i = 0; i < jsdk_ep_store_count(&f.store); ++i) {
            const char *path = NULL;
            CHECK_EQ(jsdk_ep_store_at(&f.store, i, &path, &id, &t, &acc), JSDK_OK);
            CHECK(strncmp(path, prefix[0], strlen(prefix[0]) - 1u) == 0);
            n++;
        }
        CHECK_EQ(n, 5u);
        env_teardown(&f);
    }

    /* --- 通配 '*' --- */
    {
        static const char *const star[] = { "*" };
        test_env_t f;

        printf("[4c] RETAIN_FILTERED (wildcard '*')\n");
        env_setup(&f, 65536u, JSDK_DESC_RETAIN_FILTERED);
        f.cfg.filter_paths = star;
        f.cfg.filter_count = 1;
        CHECK_EQ(jsdk_jsondesc_run(&f.cfg, &f.store, g_json, g_json_len), JSDK_OK);
        CHECK_EQ(f.store.arena.entry_count, 594u);
        env_teardown(&f);
    }
}

/* ==========================================================================
 * 5. arena 不足 → 失败且不留部分结果
 * ======================================================================== */

static void test_arena_exhaustion(void)
{
    test_env_t e;
    jsdk_jsondesc_t p;
    int rc;

    printf("[5] arena exhaustion\n");
    env_setup(&e, 1024u, JSDK_DESC_RETAIN_ALL);      /* 远小于 25493 B */
    CHECK_EQ(jsdk_jsondesc_init(&p, &e.cfg, &e.store), JSDK_OK);
    rc = jsdk_jsondesc_feed(&p, g_json, g_json_len);
    if (rc == JSDK_OK) rc = jsdk_jsondesc_finish(&p);
    /* ⚠ 公共 API 承诺：arena 不足 → JSDK_ERR_NO_MEMORY（而不是 PARSE）。
       区分很实在：调用方对这两种错误的补救动作完全不同。 */
    CHECK_EQ(rc, JSDK_ERR_NO_MEMORY);
    printf("      1024 B -> %s (%s)\n",
           rc == JSDK_ERR_NO_MEMORY ? "JSDK_ERR_NO_MEMORY" : "WRONG CODE",
           jsdk_jsondesc_error(&p));
    CHECK(jsdk_jsondesc_error(&p) != NULL);
    CHECK_EQ(jsdk_ep_store_count(&e.store), 0u);   /* 关键：不留部分结果 */
    env_teardown(&e);

    /* 刚好不够 1 个条目也要失败，且错误码同为 NO_MEMORY
       （run = init+feed+finish，粘性错误码必须不被 finish 掩盖成 PARSE） */
    env_setup(&e, 25492u, JSDK_DESC_RETAIN_ALL);       /* 差 1 字节 */
    rc = jsdk_jsondesc_run(&e.cfg, &e.store, g_json, g_json_len);
    CHECK_EQ(rc, JSDK_ERR_NO_MEMORY);
    CHECK_EQ(jsdk_ep_store_count(&e.store), 0u);
    printf("      25492 B -> JSDK_ERR_NO_MEMORY (1 byte short, as expected)\n");
    env_teardown(&e);

    /* 25493 B 恰好够 */
    env_setup(&e, 25493u, JSDK_DESC_RETAIN_ALL);
    CHECK_EQ(jsdk_jsondesc_run(&e.cfg, &e.store, g_json, g_json_len), JSDK_OK);
    CHECK_EQ(jsdk_ep_store_count(&e.store), 594u);
    CHECK_EQ(e.store.arena.blob_top, (size_t)e.store.arena.entry_count * 8u);
    printf("      25493 B -> OK (entry array meets path pool exactly)\n");
    env_teardown(&e);

    /* 语法错误仍必须是 PARSE，不能与 NO_MEMORY 混为一谈 */
    env_setup(&e, 65536u, JSDK_DESC_RETAIN_ALL);
    CHECK_EQ(jsdk_jsondesc_run(&e.cfg, &e.store, "[{\"name\":\"a\"", 13u),
             JSDK_ERR_PARSE);
    printf("      truncated JSON -> JSDK_ERR_PARSE (distinct from NO_MEMORY)\n");
    env_teardown(&e);
}

/* ==========================================================================
 * 6. 配置校验与推荐值
 * ======================================================================== */

static void test_config_validation(void)
{
    test_env_t e;
    jsdk_jsondesc_t p;

    printf("[6] config validation\n");
    env_setup(&e, 4096u, JSDK_DESC_RETAIN_FILTERED);
    /* FILTERED 但无 filter → 非法 */
    CHECK_EQ(jsdk_jsondesc_init(&p, &e.cfg, &e.store), JSDK_ERR_INVALID_ARG);
    printf("      FILTERED with filter_count=0 -> JSDK_ERR_INVALID_ARG\n");

    /* max_path_len 超上限 → 非法 */
    e.cfg.filter_paths = k_required;
    e.cfg.filter_count = 12;
    e.cfg.max_path_len = JSDK_EP_PATH_MAX_HARD + 1u;
    CHECK_EQ(jsdk_jsondesc_init(&p, &e.cfg, &e.store), JSDK_ERR_INVALID_ARG);
    e.cfg.max_path_len = 128;

    /* filter 条数超上限 → 非法 */
    e.cfg.filter_count = JSDK_DESC_MAX_FILTERS + 1u;
    CHECK_EQ(jsdk_jsondesc_init(&p, &e.cfg, &e.store), JSDK_ERR_INVALID_ARG);
    printf("      over-limit max_path_len / filter_count -> JSDK_ERR_INVALID_ARG\n");
    env_teardown(&e);

    /* 默认值兜底：max_path_len = 0 → 128 */
    env_setup(&e, 32768u, JSDK_DESC_RETAIN_ALL);
    e.cfg.max_path_len = 0;
    e.cfg.max_endpoints = 0;
    CHECK_EQ(jsdk_jsondesc_run(&e.cfg, &e.store, g_json, g_json_len), JSDK_OK);
    CHECK_EQ(jsdk_ep_store_count(&e.store), 594u);
    printf("      max_path_len/max_endpoints = 0 -> defaults used (128 / 2048)\n");
    env_teardown(&e);
}

/* ==========================================================================
 * main
 * ======================================================================== */

int main(void)
{
    printf("=== jsondesc parser tests ===\n");
    if (load_fixture() != 0) return 2;
    printf("fixture: %s (%u bytes)\n\n", FIXTURE_PATH, (unsigned)g_json_len);

    test_full_parse();
    printf("\n");
    test_incremental_equivalence();
    printf("\n");
    test_failures();
    printf("\n");
    test_truncated_fixture();
    printf("\n");
    test_filtered();
    printf("\n");
    test_arena_exhaustion();
    printf("\n");
    test_config_validation();

    printf("\n=== %d 项检查，%d 项失败 ===\n", g_checks, g_fail);
    free(g_json);
    return g_fail ? 1 : 0;
}
