/**
 * @file    cb_param.c
 * @brief   CYBERBEAST 参数访问编解码实现（见 cb_param.h）
 *
 * 与固件 can_cyberbeast.cpp 的 cmd_param_read / cmd_param_read_batch /
 * cmd_param_write / cmd_param_write_segmented / send_param_write_ack
 * 逐字节对齐。
 */

#include "cb_param.h"
#include "cb_frame.h"

/* ==========================================================================
 * 一、单参数读
 * ======================================================================== */

/**
 * 固件在**设备侧**对 ReqLen 做的归一化（cmd_param_read 里逐条对应）。
 * 导出供主站预估实际会拿到多少字节。
 */
uint8_t cb_param_normalize_req_len(uint8_t req_len, int classic)
{
    if (req_len == 0u)                     req_len = 4u;   /* 旧客户端 */
    if (req_len > CB_PARAM_MAX_VALUE)      req_len = CB_PARAM_MAX_VALUE;
    if (classic && req_len > 4u)           req_len = 4u;   /* 8-4 = 4 */
    return req_len;
}

uint32_t cb_param_read_chunks(uint8_t value_len, uint8_t req_len, int classic)
{
    uint32_t eff = (uint32_t)cb_param_normalize_req_len(req_len, classic);

    if (value_len == 0u) return 0u;
    if (eff == 0u)       return 0u;
    return ((uint32_t)value_len + eff - 1u) / eff;
}

size_t cb_param_pack_read_req(uint8_t *dst, size_t cap, uint16_t ep_id,
                              uint8_t req_len, uint32_t offset,
                              int with_offset)
{
    size_t need = with_offset ? CB_PARAM_READ_REQ_FULL : CB_PARAM_READ_REQ_MIN;

    if (!dst || cap < need) return 0u;
    if (req_len == 0u)     return 0u;      /* 0 是“未指定”，发送侧不猜 */
    if (req_len > CB_PARAM_MAX_VALUE) return 0u;

    dst[0] = 0u;                            /* 单读：Flags 全 0 */
    cb_be_put_u16(dst + 1, ep_id);
    dst[3] = req_len;

    if (with_offset) {
        cb_be_put_u32(dst + 4, offset);
    }
    return need;
}

int cb_param_unpack_read_req(const uint8_t *src, size_t len, int classic,
                             cb_param_read_req_t *out)
{
    if (!src || len < CB_PARAM_READ_REQ_MIN) return -1;
    if (!out) return 0;

    out->flags      = src[0];
    out->ep_id      = cb_be_get_u16(src + 1);
    out->has_offset = (len >= CB_PARAM_READ_REQ_FULL) ? 1 : 0;
    out->offset     = out->has_offset ? cb_be_get_u32(src + 4) : 0u;
    out->req_len    = cb_param_normalize_req_len(src[3], classic);
    return 0;
}

size_t cb_param_pack_read_rsp(uint8_t *dst, size_t cap,
                              const cb_param_read_rsp_t *v)
{
    size_t need;

    if (!dst || !v) return 0u;
    if (v->data_len > CB_PARAM_MAX_VALUE) return 0u;

    need = 4u + (size_t)v->data_len;
    if (cap < need) return 0u;

    dst[0] = v->flags;
    cb_be_put_u16(dst + 1, v->ep_id);
    dst[3] = v->data_len;
    if (v->data_len > 0u) {
        uint8_t i;
        for (i = 0u; i < v->data_len; ++i) dst[4u + i] = v->value[i];
    }
    return need;
}

int cb_param_unpack_read_rsp(const uint8_t *src, size_t len,
                             cb_param_read_rsp_t *out)
{
    uint8_t i;

    if (!src || len < 4u) return -1;
    if (src[3] > CB_PARAM_MAX_VALUE) return -1;
    if (len < 4u + (size_t)src[3]) return -1;

    if (out) {
        out->flags    = src[0];
        out->ep_id    = cb_be_get_u16(src + 1);
        out->data_len = src[3];
        /* 未使用的尾部清零，避免调用方读到上一次的残留 */
        for (i = 0u; i < CB_PARAM_MAX_VALUE; ++i) out->value[i] = 0u;
        for (i = 0u; i < out->data_len; ++i) out->value[i] = src[4u + i];
    }
    return 0;
}

size_t cb_param_build_read_rsp(uint8_t *dst, size_t cap,
                               uint8_t req_flags, uint16_t ep_id,
                               const uint8_t *full_value, uint8_t full_len,
                               uint8_t req_len, uint32_t offset)
{
    uint8_t actual_len = 0u;
    uint8_t resp_flags;
    size_t  need;

    if (!dst) return 0u;
    if (full_len > CB_PARAM_MAX_VALUE) return 0u;

    /* ⚠ MSVC 会把 `(uint8_t)~0x80u` 报成 C4310（“类型强制转换截断常量值”，
       因为 `~0x80u` 是个 32 位常量）—— 先掩到 8 位再转，语义不变、两端都干净。 */
    resp_flags = (uint8_t)(req_flags & (uint8_t)(0xFFu & ~(unsigned)CB_PARAM_FLAG_MORE));

    if (offset < (uint32_t)full_len) {
        uint32_t available = (uint32_t)full_len - offset;
        actual_len = (req_len < available) ? req_len : (uint8_t)available;

        /* 还有剩余 → 置 More */
        if ((uint32_t)offset + (uint32_t)actual_len < (uint32_t)full_len) {
            resp_flags |= CB_PARAM_FLAG_MORE;
        }
    }
    /* offset ≥ full_len → actual_len 保持 0（合法：越界读完） */

    need = 4u + (size_t)actual_len;
    if (cap < need) return 0u;

    dst[0] = resp_flags;
    cb_be_put_u16(dst + 1, ep_id);
    dst[3] = actual_len;
    if (actual_len > 0u && full_value) {
        uint8_t i;
        for (i = 0u; i < actual_len; ++i) {
            dst[4u + i] = full_value[(size_t)offset + i];
        }
    }
    return need;
}

int cb_param_rsp_has_more(const cb_param_read_rsp_t *rsp)
{
    if (!rsp) return 0;
    return (rsp->flags & CB_PARAM_FLAG_MORE) ? 1 : 0;
}

/* ==========================================================================
 * 二、批量读
 * ======================================================================== */

uint8_t cb_param_bitmap_bytes(uint8_t n)
{
    return (uint8_t)(((unsigned)n + 7u) / 8u);
}

int cb_param_bitmap_test(const uint8_t *bitmap, uint8_t n_bytes, uint8_t index)
{
    uint8_t byte_idx = (uint8_t)(index / 8u);
    uint8_t bit_idx  = (uint8_t)(index % 8u);

    if (!bitmap || byte_idx >= n_bytes) return 0;
    return (bitmap[byte_idx] & (uint8_t)(1u << bit_idx)) ? 1 : 0;
}

size_t cb_param_pack_batch_req(uint8_t *dst, size_t cap,
                               const uint16_t *eps, uint8_t n)
{
    uint8_t i;
    size_t  need;

    if (!dst || !eps) return 0u;
    if (n == 0u || n > CB_PARAM_MAX_BATCH) return 0u;

    need = 2u + 2u * (size_t)n;
    if (cap < need) return 0u;

    dst[0] = CB_PARAM_FLAG_BATCH;
    dst[1] = n;
    for (i = 0u; i < n; ++i) {
        cb_be_put_u16(dst + 2u + 2u * i, eps[i]);
    }
    return need;
}

int cb_param_unpack_batch_req(const uint8_t *src, size_t len,
                              uint16_t *out_eps, uint8_t ep_cap,
                              cb_param_batch_req_t *out)
{
    size_t  avail;
    uint8_t n;

    if (!src || len < 2u) return -1;
    if (!(src[0] & CB_PARAM_FLAG_BATCH)) return -1;   /* 不是批量请求 */

    n = src[1];
    avail = (len - 2u) / 2u;                          /* 帧内完整条目数 */

    /* 固件行为：帧内条目不足时**按实际可用数**处理，而不是报错 */
    if ((size_t)n > avail) n = (uint8_t)avail;
    if (n == 0u || n > CB_PARAM_MAX_BATCH) return -1;

    if (out_eps) {
        uint8_t i;
        uint8_t lim = (n < ep_cap) ? n : ep_cap;
        for (i = 0u; i < lim; ++i) {
            out_eps[i] = cb_be_get_u16(src + 2u + 2u * i);
        }
    }

    if (out) {
        out->count    = n;
        out->ep_bytes = src + 2;
    }
    return 0;
}

int cb_param_unpack_batch_rsp(const uint8_t *src, size_t len, uint8_t n_req,
                              cb_param_batch_rsp_t *out)
{
    uint8_t expected_bm;

    if (!src || len < 2u) return -1;
    if (!(src[0] & CB_PARAM_FLAG_BATCH)) return -1;   /* 不是批量响应 */

    if (!out) return 0;

    out->flags        = src[0];
    out->bitmap       = NULL;
    out->values       = NULL;
    out->values_len   = 0u;
    out->bitmap_bytes = 0u;

    if (src[0] & CB_PARAM_FLAG_ERR) {
        /* ERR：长度必须恰为 2，Count = 0 */
        out->is_err = 1;
        out->count  = src[1];
        return 0;
    }

    out->is_err = 0;
    out->count  = src[1];

    expected_bm = cb_param_bitmap_bytes(n_req);
    if (out->count != n_req) return -1;               /* 设备应全量返回 */
    if (len < 2u + (size_t)expected_bm) return -1;

    out->bitmap_bytes = expected_bm;
    out->bitmap       = src + 2;
    out->values       = src + 2 + expected_bm;
    out->values_len   = len - 2u - (size_t)expected_bm;
    return 0;
}

size_t cb_param_pack_batch_err(uint8_t *dst, size_t cap)
{
    if (!dst || cap < 2u) return 0u;
    dst[0] = (uint8_t)(CB_PARAM_FLAG_BATCH | CB_PARAM_FLAG_ERR);
    dst[1] = 0u;
    return 2u;
}

size_t cb_param_plan_batches(const cb_param_batch_item_t *items, size_t n_items,
                             size_t budget,
                             size_t *out_counts, size_t *out_offsets,
                             size_t max_batches)
{
    size_t n_batches = 0u;
    size_t i = 0u;

    if (!items || !out_counts || n_items == 0u || max_batches == 0u) return 0u;
    if (budget > CB_PARAM_FD_FRAME_MAX) budget = CB_PARAM_FD_FRAME_MAX;

    while (i < n_items) {
        size_t batch_start = i;
        size_t sum = 0u;
        size_t n   = 0u;

        /* 贪心：尽量多装，但不超过 N 上限与字节预算。
           请求帧约束 `2 + 2n ≤ 64` 即 n ≤ 31，由 CB_PARAM_MAX_BATCH 保证。 */
        while (i < n_items && n < CB_PARAM_MAX_BATCH) {
            size_t len = (items[i].value_len > CB_PARAM_MAX_VALUE)
                       ? CB_PARAM_MAX_VALUE : (size_t)items[i].value_len;
            size_t try_n  = n + 1u;
            size_t try_bm = ((try_n + 7u) / 8u);

            if (2u + try_bm + sum + len > budget) break;   /* 装不下 */
            sum += len;
            n   = try_n;
            ++i;
        }

        if (n == 0u) return 0u;      /* 单条就装不下 → 整体失败 */

        if (n_batches >= max_batches) return 0u;
        out_counts[n_batches] = n;
        if (out_offsets) out_offsets[n_batches] = batch_start;
        ++n_batches;
    }

    return n_batches;
}

/* ==========================================================================
 * 三、参数写
 * ======================================================================== */

size_t cb_param_pack_write_req(uint8_t *dst, size_t cap, uint16_t ep_id,
                               const uint8_t *value, uint8_t value_len)
{
    size_t need;
    uint8_t i;

    if (!dst) return 0u;
    if (value_len == 0u || value_len > CB_PARAM_MAX_VALUE) return 0u;
    if (!value) return 0u;

    /* ⚠⚠ 帧长必须 ≥ CB_PARAM_WRITE_REQ_MIN（8）：固件 `cmd_param_write()` 首句
       就是 `if (msg.len < 8) return;` —— 短帧整帧丢掉、连 ACK 都不回。
       bool(5B)/u8(5B)/u16(6B) 都曾经因此**写了等于没写**（真机实测），
       而 u32/f32 刚好 8B 所以一直正常 —— 这个不对称让 bug 藏了很久。
       `dst[3] = value_len` 仍写**真实长度**，固件只把这么多字节交给端点处理器。 */
    need = 4u + (size_t)value_len;
    if (need < CB_PARAM_WRITE_REQ_MIN) need = CB_PARAM_WRITE_REQ_MIN;
    if (cap < need) return 0u;

    dst[0] = 0u;
    cb_be_put_u16(dst + 1, ep_id);
    dst[3] = value_len;
    for (i = 0u; i < value_len; ++i) dst[4u + i] = value[i];
    for (i = (uint8_t)(4u + value_len); i < (uint8_t)need; ++i) dst[i] = 0u;
    return need;
}

int cb_param_unpack_write_req(const uint8_t *src, size_t len,
                              uint8_t *out_flags, uint16_t *out_ep_id,
                              uint8_t *out_value, uint8_t *out_value_len)
{
    uint8_t data_len;
    uint8_t i;

    if (!src || len < 4u) return -1;

    data_len = src[3];
    if (data_len > CB_PARAM_MAX_VALUE) return -1;
    if (len < 4u + (size_t)data_len) return -1;

    if (out_flags)     *out_flags     = src[0];
    if (out_ep_id)     *out_ep_id     = cb_be_get_u16(src + 1);
    if (out_value_len) *out_value_len = data_len;
    if (out_value) {
        for (i = 0u; i < data_len; ++i) out_value[i] = src[4u + i];
    }
    return 0;
}

size_t cb_param_pack_write_ack(uint8_t *dst, size_t cap,
                               uint8_t flags, uint16_t ep_id)
{
    if (!dst || cap < CB_PARAM_ACK_LEN) return 0u;

    dst[0] = flags;
    cb_be_put_u16(dst + 1, ep_id);
    dst[3] = 0u;
    dst[4] = 0u;
    dst[5] = 0u;
    dst[6] = 0u;
    dst[7] = 0u;
    return CB_PARAM_ACK_LEN;
}

int cb_param_unpack_write_ack(const uint8_t *src, size_t len,
                              uint8_t *out_flags, uint16_t *out_ep_id)
{
    if (!src || len < CB_PARAM_ACK_LEN) return -1;
    /* DataLen 必须为 0：确认帧不携带数据 */
    if (src[3] != 0u) return -1;

    if (out_flags) *out_flags = src[0];
    if (out_ep_id) *out_ep_id = cb_be_get_u16(src + 1);
    return 0;
}

/* ==========================================================================
 * 四、分段写
 * ======================================================================== */

size_t cb_param_pack_write_chunk(uint8_t *dst, size_t cap, uint16_t ep_id,
                                 uint8_t total_len, uint32_t offset,
                                 const uint8_t *value, uint8_t value_len,
                                 int more)
{
    uint8_t i;

    if (!dst || cap < CB_PARAM_CLASSIC_FRAME) return 0u;
    if (total_len < CB_PARAM_SEG_MIN_LEN) return 0u;
    if (total_len > CB_PARAM_MAX_VALUE)   return 0u;
    if (value_len > CB_PARAM_CHUNK_BYTES) return 0u;
    if (offset % CB_PARAM_CHUNK_BYTES != 0u) return 0u;
    if (offset + (uint32_t)value_len > (uint32_t)total_len) return 0u;
    if (value_len > 0u && !value) return 0u;

    /* 加固：**绝不生成固件无法正确装配的块**。
       固件 `cmd_param_write_segmented()` 对末块不做完整性校验：
       若末块（More=0）携带的字节数不足，它会直接写入一个尾部被 0 填充的
       “完整值”（见 `docs/FIRMWARE_ISSUES.zh-CN.md` 的 F13）。因此发送侧必须保证：
         ① 非末块必须满 4 字节（否则中间的洞会被静默补 0）；
         ② 末块必须刚好把值补齐（offset + value_len == total_len）。 */
    if (more) {
        if (value_len != CB_PARAM_CHUNK_BYTES) return 0u;
        if (offset + (uint32_t)value_len >= (uint32_t)total_len) return 0u;
    } else {
        if (offset + (uint32_t)value_len != (uint32_t)total_len) return 0u;
    }

    dst[0] = more ? CB_PARAM_FLAG_MORE : 0u;
    cb_be_put_u16(dst + 1, ep_id);
    dst[3] = total_len;

    /* 末块不足 4 B：用 0 填充（固件按 TotalLen 截断，填充位被忽略） */
    for (i = 0u; i < CB_PARAM_CHUNK_BYTES; ++i) {
        dst[4u + i] = (i < value_len) ? value[i] : 0u;
    }
    return CB_PARAM_CLASSIC_FRAME;
}

void cb_param_write_asm_init(cb_param_write_asm_t *a)
{
    uint8_t i;

    if (!a) return;
    a->active    = 0;
    a->ep_id     = 0u;
    a->master_id = 0u;
    a->total_len = 0u;
    a->offset    = 0u;
    for (i = 0u; i < CB_PARAM_MAX_VALUE; ++i) a->buf[i] = 0u;
}

int cb_param_write_asm_feed(cb_param_write_asm_t *a, uint8_t master_id,
                            uint16_t ep_id, uint8_t total_len, uint8_t flags,
                            const uint8_t *chunk,
                            uint8_t *value_out, uint8_t *value_len_out)
{
    uint8_t remaining;
    uint8_t i;

    if (!a || !chunk) return -1;

    /* ① TotalLen 必须在 5..8（固件：否则中止装配） */
    if (total_len < CB_PARAM_SEG_MIN_LEN || total_len > CB_PARAM_MAX_VALUE) {
        cb_param_write_asm_init(a);
        return -1;
    }

    /* ④ 首块 / 换端点 / 换主站 → 重置装配（固件行为，非错误） */
    if (!a->active || a->ep_id != ep_id || a->master_id != master_id) {
        cb_param_write_asm_init(a);
        a->active    = 1;
        a->master_id = master_id;
        a->ep_id     = ep_id;
        a->total_len = total_len;
    }

    /* ② offset 已越界（块序错乱 / 重复末块）→ 中止 */
    if (a->offset >= a->total_len) {
        cb_param_write_asm_init(a);
        return -1;
    }

    remaining = (uint8_t)(a->total_len - a->offset);
    if (remaining > CB_PARAM_CHUNK_BYTES) remaining = CB_PARAM_CHUNK_BYTES;

    for (i = 0u; i < remaining; ++i) {
        a->buf[(size_t)a->offset + i] = chunk[i];
    }
    a->offset = (uint8_t)(a->offset + remaining);

    if (flags & CB_PARAM_FLAG_MORE) {
        /* ③ 已声明 More 但缓冲已填满 → 协议错误，中止 */
        if (a->offset >= a->total_len) {
            cb_param_write_asm_init(a);
            return -1;
        }
        return 1;                       /* 还需后续块 */
    }

    /* 末块：一次交付完整值 */
    if (value_out) {
        for (i = 0u; i < CB_PARAM_MAX_VALUE; ++i) value_out[i] = a->buf[i];
    }
    if (value_len_out) *value_len_out = a->total_len;

    cb_param_write_asm_init(a);
    return 0;
}
