/**
 * @file    cb_jsondesc_fetch.c
 * @brief   JSON 端点描述符下载传输状态机（见 cb_jsondesc_fetch.h）
 *
 * 与固件 `cmd_json_desc_read()` / `send_json_desc_chunk()` 逐字段对齐。
 */

#include "cb_jsondesc_fetch.h"

#include <string.h>

/* ==========================================================================
 * 内部工具
 * ======================================================================== */

/** 小端 u32（描述符的 offset / totalLen 都用 LE，与其它帧的 BE 相反） */
static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

/** 记录失败并结束传输（首个原因优先）。 */
static int dfail(cb_desc_fetch_t *f, const char *msg, int code)
{
    if (!f->failed) {
        f->failed   = 1;
        f->err      = msg;
        f->fail_rc  = code;
        f->done     = 1;
        f->complete = 0;
    }
    return code;
}

/** 进度上报（节流：每 ≥4 KB 或按调用方要求立即上报） */
static void report_progress(cb_desc_fetch_t *f, int force)
{
    if (!f->progress) return;
    if (!force && (f->bytes_scanned - f->last_report) < 4096u) return;
    f->last_report = f->bytes_scanned;
    f->progress(f->ctx, f->bytes_scanned, f->total_len, f->progress_user);
}

/* ==========================================================================
 * 生命周期
 * ======================================================================== */

int cb_desc_fetch_init(cb_desc_fetch_t *f, jsdk_context_t *ctx,
                       const jsdk_desc_config_t *cfg, jsdk_ep_store_t *store)
{
    int rc;

    if (!f || !cfg || !store) return JSDK_ERR_INVALID_ARG;

    memset(f, 0, sizeof *f);
    f->ctx   = ctx;
    f->cfg   = cfg;
    f->store = store;

    rc = jsdk_jsondesc_init(&f->parser, cfg, store);
    if (rc != JSDK_OK) {
        (void)dfail(f, CB_DESC_ERR_PARSE, rc);
    }

    /* 提前终止只对“全部精确路径”的 filter 安全，见头文件说明。
       含通配时宁可多扫几百帧，也不交付依赖 JSON 字段顺序的残缺端点表。 */
    f->stop_allowed = (uint8_t)(cfg->stop_when_satisfied
                                && cb_desc_filters_all_exact(cfg));
    return rc;
}

void cb_desc_fetch_set_raw_sink(cb_desc_fetch_t *f, jsdk_desc_raw_sink_fn fn,
                                void *user)
{
    if (!f) return;
    f->sink      = fn;
    f->sink_user = user;
    if (!fn) f->raw_sink_failed = 1u;      /* 未安装 = raw 缓存不可用 */
}

void cb_desc_fetch_set_progress(cb_desc_fetch_t *f, jsdk_desc_progress_fn fn,
                                void *user)
{
    if (!f) return;
    f->progress      = fn;
    f->progress_user = user;
}

uint32_t cb_desc_timeout_ms(const jsdk_desc_config_t *cfg)
{
    if (cfg && cfg->timeout_ms) return cfg->timeout_ms;
    return 5000u;
}

int cb_desc_filters_all_exact(const jsdk_desc_config_t *cfg)
{
    unsigned i;

    if (!cfg || cfg->filter_count == 0u || !cfg->filter_paths) return 1;

    for (i = 0u; i < cfg->filter_count; ++i) {
        const char *s = cfg->filter_paths[i];
        size_t n;
        if (!s || s[0] == '\0') continue;
        n = strlen(s);
        /* `prefix*`（含全文通配 "*"）或 `segment.` 都算通配 → 不允许提前终止 */
        if (s[n - 1u] == '*' || s[n - 1u] == '.') return 0;
    }
    return 1;
}

/* ==========================================================================
 * 请求构造
 * ======================================================================== */

size_t cb_desc_build_request(uint8_t *dst, size_t cap, uint32_t offset)
{
    if (!dst || cap < CB_DESC_REQ_LEN) return 0u;
    put_le32(dst, offset);
    return CB_DESC_REQ_LEN;
}

int cb_desc_frame_len_valid(size_t len)
{
    return (len == CB_DESC_FRAME_LEN_CLASSIC || len == CB_DESC_FRAME_LEN_FD) ? 1 : 0;
}

/* ==========================================================================
 * 帧处理
 * ======================================================================== */

/** 元数据帧：解析 total_len / crc 并做全部合法性校验。 */
static int take_metadata(cb_desc_fetch_t *f, const uint8_t *payload, size_t len)
{
    uint32_t total;

    if (len < CB_DESC_META_MIN_LEN) {
        return dfail(f, CB_DESC_ERR_FRAME_LEN, JSDK_ERR_PROTOCOL);
    }

    /* 元数据帧的头两字节恒为 0 */
    if (payload[0] != 0u || payload[1] != 0u) {
        return dfail(f, CB_DESC_ERR_META_EXPECTED, JSDK_ERR_PROTOCOL);
    }

    /* 交叉校验：若第 3 字节是 JSON 起始符，说明这其实是 offset = 0 的数据帧，
       即设备跳过了元数据帧。此时无法得知 total_len，只能整体失败。 */
    if (payload[2] == CB_DESC_JSON_FIRST_BYTE) {
        return dfail(f, CB_DESC_ERR_META_EXPECTED, JSDK_ERR_PROTOCOL);
    }

    total = le32(payload + 2);
    if (total == 0u || total > CB_DESC_MAX_TOTAL_LEN) {
        /* ⚠ > 65535 必须硬拒绝：chunkOffset 只有 u16，固件会静默回绕 */
        return dfail(f, CB_DESC_ERR_META_TOTAL, JSDK_ERR_PROTOCOL);
    }

    f->total_len   = total;
    f->crc         = le16(payload + 6);
    f->next_offset = 0u;
    f->started     = 1u;
    f->last_report = 0u;
    report_progress(f, 1);
    return JSDK_OK;
}

/** 数据帧：tee → 解析 → 推进偏移 → 判定结束。 */
static int take_data(cb_desc_fetch_t *f, const uint8_t *payload, size_t len)
{
    uint32_t off;
    uint32_t avail;
    uint32_t n;
    int rc;

    off   = (uint32_t)le16(payload);
    avail = f->total_len - f->next_offset;

    if (off != f->next_offset) {
        return dfail(f, CB_DESC_ERR_OFFSET, JSDK_ERR_PROTOCOL);
    }

    n = (uint32_t)(len - CB_DESC_DATA_HDR_BYTES);
    if (n > avail) n = avail;              /* 末帧尾部是补零 */

    if (n > 0u) {
        /* ① 先 tee 再解析（公共 API 承诺：sink 拿到的是**解析之前**的字节） */
        if (f->sink && !f->raw_sink_failed) {
            if (f->sink(f->ctx, payload + CB_DESC_DATA_HDR_BYTES, n,
                        f->next_offset, f->sink_user) != 0) {
                f->raw_sink_failed = 1u;   /* 放弃 tee，但下载继续 */
            } else {
                f->sink_called = 1u;
            }
        }

        /* ② 增量解析 */
        rc = jsdk_jsondesc_feed(&f->parser, payload + CB_DESC_DATA_HDR_BYTES, n);
        if (rc != JSDK_OK) {
            if (rc == JSDK_ERR_NO_MEMORY) {
                f->arena_full = 1u;
                return dfail(f, CB_DESC_ERR_ARENA, JSDK_ERR_NO_MEMORY);
            }
            /* 详细原因由 jsdk_jsondesc_error() 给出（如“路径超过上限”）*/
            if (jsdk_jsondesc_error(&f->parser)) f->err_detail = jsdk_jsondesc_error(&f->parser);
            return dfail(f, CB_DESC_ERR_PARSE, JSDK_ERR_PARSE);
        }

        f->next_offset   += n;
        f->bytes_scanned += n;
    }

    /* ③ 收满 → 解析器必须能正常收尾，否则视为描述符损坏。
       ⚠ 必须先于“提前终止”判定：若最后一个 filter 恰好在**末帧**才满足，
          先判提前终止会把一次**完整**下载误标成 stopped_early（complete=0），
          进而让 `cache_safe()` 无谓地否掉一份可用的 raw 缓存。 */
    if (f->next_offset >= f->total_len) {
        rc = jsdk_jsondesc_finish(&f->parser);
        if (rc != JSDK_OK) {
            if (rc == JSDK_ERR_NO_MEMORY) {
                f->arena_full = 1u;
                return dfail(f, CB_DESC_ERR_ARENA, JSDK_ERR_NO_MEMORY);
            }
            return dfail(f, CB_DESC_ERR_PARSE, JSDK_ERR_PARSE);
        }
        f->complete = 1u;
        f->done     = 1u;
        if (!f->sink) f->raw_sink_failed = 1u;   /* 没装 sink → raw 缓存不可用 */
        report_progress(f, 1);
        return JSDK_OK;
    }

    /* ④ 提前终止：filter 全部命中就不再等剩下的帧（设备会继续发，由调用方丢弃）。
       ⚠ 仅在全部 filter 为精确路径时允许——前缀 filter 会被首个匹配项“满足”，
          提前停止会静默丢掉同前缀家族的其余路径。 */
    if (f->stop_allowed && jsdk_jsondesc_satisfied(&f->parser)) {
        f->stopped_early = 1u;
        f->done          = 1u;
        f->complete      = 0u;
        report_progress(f, 1);
    }

    return JSDK_OK;
}

int cb_desc_fetch_frame(cb_desc_fetch_t *f, const uint8_t *payload, size_t len)
{
    if (!f || !payload) return JSDK_ERR_INVALID_ARG;

    if (f->failed) return f->fail_rc ? f->fail_rc : JSDK_ERR_PARSE;
    /* 已提前终止后设备还会继续发帧；静默忽略，不算错 */
    if (f->done) return JSDK_OK;

    if (!cb_desc_frame_len_valid(len)) {
        return dfail(f, CB_DESC_ERR_FRAME_LEN, JSDK_ERR_PROTOCOL);
    }

    f->frames_rx++;

    if (!f->started) {
        return take_metadata(f, payload, len);
    }
    if (len < CB_DESC_DATA_HDR_BYTES) {
        return dfail(f, CB_DESC_ERR_FRAME_LEN, JSDK_ERR_PROTOCOL);
    }
    return take_data(f, payload, len);
}

/* ==========================================================================
 * 查询
 * ======================================================================== */

int cb_desc_fetch_is_done(const cb_desc_fetch_t *f)
{
    return (f && f->done) ? 1 : 0;
}

int cb_desc_fetch_is_ok(const cb_desc_fetch_t *f)
{
    if (!f || f->failed) return 0;
    if (f->complete) return 1;
    if (f->stopped_early) return jsdk_jsondesc_satisfied(&f->parser);
    return 0;
}

int cb_desc_fetch_cache_safe(const cb_desc_fetch_t *f)
{
    if (!f || f->failed)             return 0;
    if (!f->complete)                return 0;   /* 被提前终止 → 数据不完整 */
    if (f->raw_sink_failed)          return 0;   /* sink 报错或未安装 */
    if (!f->sink_called)             return 0;   /* 一次都没成功写出 */
    if (f->bytes_scanned != f->total_len) return 0;   /* 字节数对不上 */
    return 1;
}

const char *cb_desc_fetch_error(const cb_desc_fetch_t *f)
{
    if (!f) return CB_DESC_ERR_PARSE;
    return f->err;
}

const char *cb_desc_fetch_detail(const cb_desc_fetch_t *f)
{
    return f ? f->err_detail : (const char *)0;
}

void cb_desc_fetch_result(const cb_desc_fetch_t *f, cb_desc_fetch_result_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof *out);
    if (!f) return;

    out->total_len     = f->total_len;
    out->crc           = f->crc;
    out->bytes_scanned = f->bytes_scanned;
    out->frames_rx     = f->frames_rx;
    out->complete      = f->complete;
    out->stopped_early = f->stopped_early;
    out->stop_allowed  = f->stop_allowed;
    out->raw_sink_failed = f->raw_sink_failed;
    out->failed        = f->failed;

    if (f->store) {
        out->endpoint_count = jsdk_ep_store_count(f->store);
        out->parsed_total   = f->store->parsed_total;
        out->arena_used = (size_t)f->store->arena.entry_count * sizeof(jsdk_ep_entry_t)
                        + f->store->arena.blob_used;
    }
}

unsigned cb_desc_fetch_pct(const cb_desc_fetch_t *f)
{
    if (!f || f->total_len == 0u) return 0u;
    if (f->bytes_scanned >= f->total_len) return 100u;
    return (unsigned)(((uint64_t)f->bytes_scanned * 100u) / f->total_len);
}
