/**
 * @file    05_desc_cache.c
 * @brief   描述符：下载 → enumerate → 缓存（两条路线都演示）
 *
 * 学到的六件事：
 *   1. 端点表是**设备主动告诉你**的（`0x24`/`0x25` 流），SDK 不内置任何静态表 ——
 *      所以换固件版本不会读错参数；
 *   2. 下载是流式解析的：SDK **不保留**原始 JSON，要用原始字节必须装
 *      `desc_raw_sink`（路线 B）；
 *   3. 路线 A：`desc_export()` 存"已解析结果"（小，但受 retain/filter 绑定）；
 *      路线 B（推荐）：缓存**原始 JSON**（41 KB），只与 (fw_version, desc_crc) 绑定；
 *   4. `complete == 0`（filter 满足后提前终止）时**绝不能**缓存原始 JSON；
 *   5. `ep_id` 跨固件版本会漂移（实测 86%），**一律按路径访问**；
 *   6. arena 大小是调用者的责任：`jsdk_desc_arena_size()` 只是估算，
 *      事后拿 `desc_info.parsed_total` / 实际用量复核。
 *
 * 运行：
 *     ./build/examples/05_desc_cache
 */

#include "ex_common.h"

static int count_cb(void *user, const char *path, uint16_t ep_id,
                    jsdk_ep_type_t type, uint8_t access)
{
    unsigned *n = (unsigned *)user;
    (void)access;

    if (*n < 5u) {
        printf("    %-48s id=%-4u type=%d\n", path, (unsigned)ep_id, (int)type);
    } else if (*n == 5u) {
        printf("    ...\n");
    }
    (*n)++;
    return 0;
}

/* --- 路线 B 的 sink：真机上写 Flash，这里写到内存缓冲 --- */
typedef struct {
    unsigned char buf[65536];
    size_t        len;
    int           failed;
} raw_cache_t;

static int raw_sink(jsdk_context_t *ctx, const void *data, size_t len,
                    uint32_t offset, void *user)
{
    raw_cache_t *c = (raw_cache_t *)user;
    (void)ctx;

    if ((size_t)offset + len > sizeof c->buf) { c->failed = 1; return -1; }
    memcpy(c->buf + offset, data, len);
    if ((size_t)offset + len > c->len) c->len = (size_t)offset + len;
    return 0;
}

int main(void)
{
    ex_ctx_t          e;
    const unsigned    nodes[1] = { 1u };
    jsdk_desc_info_t  info;
    unsigned          n_eps = 0u;
    uint16_t          ep_id = 0u;
    jsdk_ep_type_t    type;
    uint8_t           access = 0u;
    static raw_cache_t cache;          /* 41 KB 原始 JSON，放静态区 */
    unsigned char     exp[8192];
    size_t            exp_len = 0u;

    printf("=== 05 描述符与缓存 ===\n");

    memset(&cache, 0, sizeof cache);

    /* --- 1. 先开上下文（**不**下描述符），装上 raw sink ------------------
       顺序很重要：sink 必须在下载**之前**装好，否则拿不到字节。 */
    {
        jsdk_status_t st;
        unsigned      i;

        memset(&e, 0, sizeof e);
        st = jsdk_hal_virtual_open(&e.hal, &e.hh,
                                  "0:id=1,gear=16.5,pmax=12.5,vmax=65,tmax=50,"
                                  "hb=10,timeout=30000,fd");
        if (st != JSDK_OK) ex_fatal("hal_virtual_open", st);
        jsdk_hal_virtual_set_autotick(e.hh, 1);

        jsdk_context_config_default(&e.cfg);
        e.cfg.hal             = e.hal;
        e.cfg.master_id       = 1u;
        e.cfg.is_fd           = 1u;
        e.cfg.period_ns       = 2000000u;
        e.cfg.desc.mode       = JSDK_DESC_DYNAMIC;
        e.cfg.desc.retain     = JSDK_DESC_RETAIN_ALL;
        e.cfg.desc.timeout_ms = 30000u;
        /* ⚠ 装了 sink 就必须 stop_when_satisfied = 0，否则流会被提前终止、
           缓存到的 JSON 是残缺的（事后 cache_safe 校验会挡住，但别去踩）。
           这里 retain=ALL 时本来就不会提前终止，显式写出来是为了让人看到这条。 */
        e.cfg.desc.stop_when_satisfied = 0;

        e.arena = calloc(1u, jsdk_desc_arena_size(&e.cfg.desc));
        if (!e.arena) ex_fatal("arena", JSDK_ERR_NO_MEMORY);
        e.cfg.desc.arena      = e.arena;
        e.cfg.desc.arena_size = jsdk_desc_arena_size(&e.cfg.desc);

        e.ctx = (jsdk_context_t *)&e.store;
        ex_check("context_init", jsdk_context_init(e.ctx, &e.cfg));

        for (i = 0u; i < 1u; ++i) {
            jsdk_joint_config_t jc;
            memset(&jc, 0, sizeof jc);
            jc.node_id      = (uint8_t)nodes[i];
            jc.initial_mode = JSDK_MODE_MIT;
            ex_check("add_joint", jsdk_context_add_joint(e.ctx, &jc, &e.joint[i]));
            e.nj++;
        }

        jsdk_context_set_desc_raw_sink(e.ctx, raw_sink, &cache);
        ex_check("desc_fetch", jsdk_context_desc_fetch(e.ctx));
    }

    /* --- 2. 下载结果自检 ------------------------------------------------ */
    ex_check("get_desc_info", jsdk_context_get_desc_info(e.ctx, &info));
    printf("  desc     : endpoints=%u parsed=%u frames=%u bytes=%u complete=%d\n",
           info.endpoint_count, info.parsed_total, info.frames_rx,
           info.bytes_scanned, (int)info.complete);
    printf("             fw=0x%X crc=0x%04X shared_hit=%d raw_sink_failed=%d\n",
           (unsigned)info.fw_version, (unsigned)info.crc,
           (int)info.shared_hit, (int)info.raw_sink_failed);
    printf("  raw tee  : %lu 字节（路线 B 的原始 JSON，真机上直接写 Flash）\n",
           (unsigned long)cache.len);
    if (info.complete == 0) {
        printf("  ⚠ complete=0：**不得**缓存原始 JSON（数据不完整）\n");
    }

    ex_check("configure", jsdk_context_configure(e.ctx));

    /* --- 3. enumerate：看看设备到底提供什么 ---------------------------- */
    printf("  端点（前几个）：\n");
    ex_check("endpoint_enumerate", jsdk_endpoint_enumerate(e.ctx, count_cb, &n_eps));
    printf("  共 %u 个已保留端点\n", n_eps);

    /* --- 4. 按路径查 ID（**不要**把 ID 写进代码：跨版本会漂移） ---------- */
    if (jsdk_endpoint_lookup(e.ctx, "axis0.controller.config.mit_max_kp",
                             &ep_id, &type, &access) == JSDK_OK) {
        printf("  lookup   : mit_max_kp id=%u type=%d access=0x%X\n",
               (unsigned)ep_id, (int)type, (unsigned)access);
    }

    /* --- 5. 路线 A：导出"已解析结果"（小，受 retain/filter 绑定） -------- */
    printf("  export A : max=%lu 字节\n", (unsigned long)jsdk_desc_export_max_size(e.ctx));
    if (jsdk_context_desc_export(e.ctx, exp, sizeof exp, &exp_len) == JSDK_OK) {
        printf("             实际 %lu 字节（RETAIN_ALL；RETAIN_FILTERED 约 0.5 KB）\n",
               (unsigned long)exp_len);
    }

    /* --- 6. 路线 B：用刚 tee 出来的原始 JSON 重建（证明 tee 的字节可用） - */
    if (cache.len > 0u && !cache.failed) {
        jsdk_desc_hint_t hint;
        jsdk_context_t  *ctx2 = (jsdk_context_t *)&e.store;   /* 复用同一块存储 */
        jsdk_status_t    st;

        hint.crc        = info.crc;
        hint.fw_version = info.fw_version;

        /* 真机上这里是"上电时从 Flash 读回 41 KB → import_raw"，于是
           完全不用走 CAN。这里用同一个上下文重放一次以证明字节有效：
           import_raw 会按当前 cfg.desc 重新解析。 */
        st = jsdk_context_desc_import_raw(ctx2, cache.buf, cache.len, &hint);
        printf("  import B : %s（%s）\n", jsdk_status_string(st),
               jsdk_context_last_error(ctx2));
    }

    ex_close(&e);
    printf("done\n");
    return 0;
}
