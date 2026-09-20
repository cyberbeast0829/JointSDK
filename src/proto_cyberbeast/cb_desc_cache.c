/**
 * @file    cb_desc_cache.c
 * @brief   描述符缓存导出/导入实现（见 cb_desc_cache.h）
 */

#include "cb_desc_cache.h"

#include <string.h>

/* ==========================================================================
 * 字节序工具（缓存固定小端，与 CPU 无关）
 * ======================================================================== */

static uint16_t rd16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
}

static void wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

/* ==========================================================================
 * CRC-16/CCITT-FALSE（poly 0x1021，init 0xFFFF）
 * 逐位实现，不占 ROM 表；描述符缓存只有几十 KB，速度不成问题。
 * ======================================================================== */

uint16_t cb_desc_cache_crc16(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint16_t crc = 0xFFFFu;
    size_t i;
    int b;

    if (!p) return 0u;

    /*
     * 内部全程用 `unsigned` 运算，每轮显式截回 16 位（与原来的 `(uint16_t)` 转换
     * 逐位等价）。
     *
     * ⚠ 为什么必须写显式转换：`uint16_t` 参与移位/异或时会整型提升 `int`，
     *   之后与无符号常量相遇就产生 `int → unsigned` 的隐式符号转换。
     *   GCC 的 `-Wsign-conversion` 在这条上**跟优化级别有关**：
     *   `-O2` 的值域分析能证明"掩码后必然非负"而不报，加上
     *   `-fsanitize=undefined`（为插桩降低优化）后就证不出来、开始报错。
     *   写死 `unsigned` 后，任何优化级别、任何 sanitizer 组合都零告警
     *   —— 这直接决定了 `-Werror` 的 sanitizer 构建能不能跑起来。
     */
    for (i = 0u; i < len; ++i) {
        unsigned c = (unsigned)crc;

        c ^= ((unsigned)p[i] << 8);
        for (b = 0; b < 8; ++b) {
            c = ((c & 0x8000u) != 0u) ? ((c << 1) ^ 0x1021u) : (c << 1);
            c &= 0xFFFFu;                       /* 逐位 16 位截断（与原实现一致） */
        }
        crc = (uint16_t)c;
    }
    return crc;
}

/* ==========================================================================
 * 失效键
 * ======================================================================== */

/* FNV-1a 32 位 */
#define FNV_OFFSET 2166136261u
#define FNV_PRIME  16777619u

static uint32_t fnv_bytes(uint32_t h, const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    size_t i;
    for (i = 0u; i < len; ++i) {
        h ^= (uint32_t)p[i];
        h *= FNV_PRIME;
    }
    return h;
}

static uint32_t fnv_u32(uint32_t h, uint32_t v)
{
    uint8_t b[4];
    wr32(b, v);
    return fnv_bytes(h, b, 4u);
}

/* 配置上限的“生效值”：0 表示使用内置默认值（见 joint_sdk.h）。
   哈希与缓存头都必须用生效值，否则“用默认值导出、用**显式**默认值导入”
   会被误判为配置不符（这是一个很容易踩的坑）。 */
uint32_t cb_desc_eff_max_endpoints(const jsdk_desc_config_t *cfg)
{
    return cfg->max_endpoints ? (uint32_t)cfg->max_endpoints : 2048u;
}

uint32_t cb_desc_eff_max_path_len(const jsdk_desc_config_t *cfg)
{
    return cfg->max_path_len ? (uint32_t)cfg->max_path_len : 128u;
}

uint32_t cb_desc_filter_hash(const jsdk_desc_config_t *cfg)
{
    uint32_t h = FNV_OFFSET;
    unsigned i;

    if (!cfg) return 0u;

    /* 线格式版本：SDK 升级导致内部表示变化时，缓存必须失效 */
    h = fnv_u32(h, CB_DESC_CACHE_VERSION);
    h = fnv_u32(h, (uint32_t)cfg->retain);
    h = fnv_u32(h, cb_desc_eff_max_endpoints(cfg));
    h = fnv_u32(h, cb_desc_eff_max_path_len(cfg));
    h = fnv_u32(h, (uint32_t)cfg->filter_count);

    /* filter 列表按**顺序**参与：顺序变了就视为不同的键（保守但简单） */
    if (cfg->filter_paths) {
        for (i = 0u; i < cfg->filter_count; ++i) {
            const char *s = cfg->filter_paths[i];
            size_t n = s ? strlen(s) : 0u;
            h = fnv_u32(h, (uint32_t)n);
            if (n) h = fnv_bytes(h, s, n);
        }
    }
    return h;
}

/* ==========================================================================
 * 导出
 * ======================================================================== */

static size_t path_len_in_arena(const jsdk_ep_store_t *store, unsigned idx)
{
    const char *p = jsdk_ep_store_path(store, idx);
    return p ? (strlen(p) + 1u) : 0u;
}

size_t cb_desc_cache_size(const jsdk_ep_store_t *store)
{
    size_t total = CB_DESC_CACHE_HDR_LEN;
    unsigned n, i;

    if (!store) return CB_DESC_CACHE_HDR_LEN;

    n = jsdk_ep_store_count(store);
    for (i = 0u; i < n; ++i) {
        total += CB_DESC_CACHE_REC_HDR + path_len_in_arena(store, i);
    }
    return total;
}

int cb_desc_cache_export(const jsdk_desc_config_t *cfg, const jsdk_ep_store_t *store,
                         int early, void *buf, size_t cap, size_t *out_len)
{
    uint8_t *b = (uint8_t *)buf;
    size_t need, body_len = 0u;
    unsigned n, i;
    uint8_t *body;
    size_t off;
    uint32_t path_bytes = 0u;

    if (out_len) *out_len = 0u;
    if (!cfg || !store || !buf) return JSDK_ERR_INVALID_ARG;

    need = cb_desc_cache_size(store);
    if (cap < need) return JSDK_ERR_NO_MEMORY;   /* 调用方扩容后重试 */

    n = jsdk_ep_store_count(store);

    /* --- 头 --- */
    wr32(b + 0u, CB_DESC_CACHE_MAGIC);
    wr16(b + 4u, (uint16_t)CB_DESC_CACHE_VERSION);
    wr16(b + 6u, (uint16_t)CB_DESC_CACHE_HDR_LEN);
    wr16(b + 8u, 0u);                                   /* desc_crc：由 set_fw/后续补写 */
    wr16(b + 10u, (uint16_t)((cfg->retain == JSDK_DESC_RETAIN_FILTERED
                              ? CB_DESC_CACHE_F_FILTERED : 0u)
                             | (early ? CB_DESC_CACHE_F_EARLY : 0u)));
    wr32(b + 12u, 0u);                                  /* fw_version：由 set 补写 */
    wr32(b + 16u, cb_desc_filter_hash(cfg));
    wr32(b + 20u, cb_desc_eff_max_endpoints(cfg));
    wr16(b + 24u, (uint16_t)cb_desc_eff_max_path_len(cfg));
    wr16(b + 26u, 0u);
    wr32(b + 28u, (uint32_t)n);
    wr32(b + 36u, 0u);                                  /* body_len：下面填 */
    wr32(b + 40u, 0u);                                  /* body_crc：下面填 */
    wr32(b + 44u, 0u);

    /* --- 体 --- */
    body = b + CB_DESC_CACHE_HDR_LEN;
    off = 0u;
    for (i = 0u; i < n; ++i) {
        const char *path = jsdk_ep_store_path(store, i);
        uint16_t ep_id = 0u;
        jsdk_ep_type_t type = JSDK_EP_OBJECT;
        uint8_t access = 0u;
        size_t plen;

        if (jsdk_ep_store_at(store, i, NULL, &ep_id, &type, &access) != JSDK_OK) {
            return JSDK_ERR_INVALID_ARG;
        }
        plen = path ? (strlen(path) + 1u) : 0u;
        if (plen == 0u || plen > 0xFFFFu) return JSDK_ERR_INVALID_ARG;

        wr16(body + off + 0u, ep_id);
        body[off + 2u] = (uint8_t)type;
        body[off + 3u] = access;
        wr16(body + off + 4u, (uint16_t)plen);
        if (plen) memcpy(body + off + CB_DESC_CACHE_REC_HDR, path, plen);
        off += CB_DESC_CACHE_REC_HDR + plen;
        path_bytes += (uint32_t)plen;
    }
    body_len = off;

    wr32(b + 32u, path_bytes);
    wr32(b + 36u, (uint32_t)body_len);
    wr32(b + 40u, (uint32_t)cb_desc_cache_crc16(body, body_len));

    if (out_len) *out_len = CB_DESC_CACHE_HDR_LEN + body_len;
    return JSDK_OK;
}

/* ==========================================================================
 * 头部查看 / 补写
 * ======================================================================== */

/** 头部合法性（不含失效键与体校验）。 */
static int hdr_check(const uint8_t *b, size_t len)
{
    if (!b || len < CB_DESC_CACHE_HDR_LEN)             return JSDK_ERR_INVALID_ARG;
    if (rd32(b + 0u) != CB_DESC_CACHE_MAGIC)           return JSDK_ERR_PROTOCOL;
    if (rd16(b + 4u) != (uint16_t)CB_DESC_CACHE_VERSION) return JSDK_ERR_PROTOCOL;
    if (rd16(b + 6u) != (uint16_t)CB_DESC_CACHE_HDR_LEN) return JSDK_ERR_PROTOCOL;
    return JSDK_OK;
}

int cb_desc_cache_peek(const void *buf, size_t len, cb_desc_cache_meta_t *out)
{
    const uint8_t *b = (const uint8_t *)buf;
    int rc = hdr_check(b, len);

    if (rc != JSDK_OK) return rc;
    if (out) {
        out->desc_crc       = rd16(b + 8u);
        out->flags          = rd16(b + 10u);
        out->fw_version     = rd32(b + 12u);
        out->endpoint_count = rd32(b + 28u);
    }
    return JSDK_OK;
}

int cb_desc_cache_set_fw_version(void *buf, size_t len, uint32_t fw_version)
{
    uint8_t *b = (uint8_t *)buf;
    int rc = hdr_check(b, len);

    if (rc != JSDK_OK) return rc;
    wr32(b + 12u, fw_version);
    return JSDK_OK;
}

/** 设置描述符 VersionCRC（导出后再补写，避免导出时还要查设备）。 */
int cb_desc_cache_set_desc_crc(void *buf, size_t len, uint16_t crc);
int cb_desc_cache_set_desc_crc(void *buf, size_t len, uint16_t crc)
{
    uint8_t *b = (uint8_t *)buf;
    int rc = hdr_check(b, len);

    if (rc != JSDK_OK) return rc;
    wr16(b + 8u, crc);
    return JSDK_OK;
}

/* ==========================================================================
 * 导入
 * ======================================================================== */

int cb_desc_cache_import(const jsdk_desc_config_t *cfg, jsdk_ep_store_t *store,
                         const void *buf, size_t len)
{
    const uint8_t *b = (const uint8_t *)buf;
    const uint8_t *body;
    uint32_t path_bytes, body_len, body_crc, n, i;
    size_t off;
    uint32_t got_path_bytes = 0u;
    int rc;

    if (!cfg || !store) return JSDK_ERR_INVALID_ARG;

    /* 先清空：任何失败都不留部分结果 */
    if (store->arena.base && store->arena.size) jsdk_ep_store_reset(store);

    rc = hdr_check(b, len);
    if (rc != JSDK_OK) return rc;

    /* --- 失效键：三者任一不匹配都拒绝（这是路线 A 的固有限制） --- */
    {
        uint16_t want = (uint16_t)((cfg->retain == JSDK_DESC_RETAIN_FILTERED
                                    ? CB_DESC_CACHE_F_FILTERED : 0u)
                                   | (rd16(b + 10u) & CB_DESC_CACHE_F_EARLY));
        if (rd16(b + 10u) != want)               return JSDK_ERR_BAD_STATE;
    }
    if (rd32(b + 16u) != cb_desc_filter_hash(cfg))  return JSDK_ERR_BAD_STATE;
    if (rd32(b + 20u) != cb_desc_eff_max_endpoints(cfg)) return JSDK_ERR_BAD_STATE;
    if (rd16(b + 24u) != (uint16_t)cb_desc_eff_max_path_len(cfg)) return JSDK_ERR_BAD_STATE;

    n          = rd32(b + 28u);
    path_bytes = rd32(b + 32u);
    body_len   = rd32(b + 36u);
    body_crc   = rd32(b + 40u);

    if (n > 0xFFFFu) return JSDK_ERR_PROTOCOL;                 /* ep_id 是 u16 */
    if ((size_t)CB_DESC_CACHE_HDR_LEN + body_len > len) return JSDK_ERR_PROTOCOL;

    body = b + CB_DESC_CACHE_HDR_LEN;
    if (cb_desc_cache_crc16(body, body_len) != (uint16_t)body_crc) {
        return JSDK_ERR_PROTOCOL;                              /* Flash 位翻转 */
    }

    /* --- 第一遍：只做边界校验，先不碰 arena --- */
    off = 0u;
    for (i = 0u; i < n; ++i) {
        uint16_t plen;
        if (off + CB_DESC_CACHE_REC_HDR > body_len) return JSDK_ERR_PROTOCOL;
        plen = rd16(body + off + 4u);
        if (plen == 0u || plen > JSDK_EP_PATH_MAX_HARD) return JSDK_ERR_PROTOCOL;
        if (off + CB_DESC_CACHE_REC_HDR + plen > body_len) return JSDK_ERR_PROTOCOL;
        got_path_bytes += (uint32_t)plen;
        off += CB_DESC_CACHE_REC_HDR + plen;
    }
    if (off != body_len)          return JSDK_ERR_PROTOCOL;
    if (got_path_bytes != path_bytes) return JSDK_ERR_PROTOCOL;

    /* --- 第二遍：写 arena --- */
    off = 0u;
    for (i = 0u; i < n; ++i) {
        uint16_t ep_id = rd16(body + off + 0u);
        uint8_t  type  = body[off + 2u];
        uint8_t  access= body[off + 3u];
        uint16_t plen  = rd16(body + off + 4u);
        const char *path = (const char *)(body + off + CB_DESC_CACHE_REC_HDR);

        if (jsdk_ep_arena_put(&store->arena, path, plen, ep_id, type, access)
                != JSDK_OK) {
            jsdk_ep_store_reset(store);
            return JSDK_ERR_NO_MEMORY;
        }
        store->parsed_total++;      /* 缓存里的条目都算“解析到过” */
        off += CB_DESC_CACHE_REC_HDR + plen;
    }

    /* 恢复上限：导入后继续解析新数据时应沿用当前 cfg（而非缓存里的旧值） */
    store->max_endpoints = cfg->max_endpoints ? cfg->max_endpoints : 2048u;
    return JSDK_OK;
}

/* ==========================================================================
 * 路线 B
 * ======================================================================== */

int cb_desc_import_raw(const jsdk_desc_config_t *cfg, jsdk_ep_store_t *store,
                       const void *json, size_t len)
{
    if (!cfg || !store || !json || len == 0u) return JSDK_ERR_INVALID_ARG;
    return jsdk_jsondesc_run(cfg, store, json, len);
}
