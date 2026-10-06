/**
 * @file    jsdk_param.c
 * @brief   WP5 参数访问：类型化 get/set、批量读、SDO 风格槽位
 *
 * @par 为什么“批量读”必须能自动退化
 *  固件对 **Classic 帧**的批量请求**恒回 2 字节 ERR**（`cb_param_pack_batch_err`）——
 *  这是固件行为，不是我们没实现。所以 SDK 的策略是：
 *    - FD   → 按各端点类型长度装箱，尽量一帧装完（最多 31 条 / 62 字节预算）
 *    - Classic → 直接逐条单读，**不让客户看到 ERR**
 *  客户只调一次 `jsdk_joint_param_get_batch()`，由 SDK 决定走哪条路。
 *
 * @par 参数的字节序（⚙ 实测修正）
 *  参数值是**小端**。协议文档曾写"与全协议一致（大端）"，但真机不是：
 *  固件在 PARAM_READ/WRITE 里把端点值原样 `memcpy` 进/出载荷，线上就是
 *  ARM 主机序（LE）。完整证据（固件源码锚点 + 真机实测三例）见
 *  `src/proto_cyberbeast/cb_frame.h` 里的 `cb_le_*` 说明。
 *  控制帧与查询响应（0x40/0x46/0x49）**仍然是大端**。
 *  读到调用方缓冲后由本文件负责按端点类型宽度解成主机序的 `jsdk_value_t`。
 */

#include "jsdk_core_internal.h"

#include <string.h>

/* ==========================================================================
 * 名字 → 端点信息
 * ======================================================================== */

/** 解析路径；失败时写错误串并返回状态码。 */
static jsdk_status_t resolve(jsdk_joint_t *j, const char *path,
                             uint16_t *ep_id, jsdk_ep_type_t *type, uint8_t *access)
{
    jsdk_status_t st;

    if (!jsdk_joint_check(j) || !path) return JSDK_ERR_INVALID_ARG;
    if (!j->ctx->desc_present) {
        jsdk_joint_seterr(j, "descriptor not available: configure() first");
        return JSDK_ERR_BAD_STATE;
    }

    st = jsdk_ep_store_lookup(&j->ctx->store, path, ep_id, type, access);
    if (st != JSDK_OK) {
        /* 不猜、不近似：未命中就是未命中 */
        jsdk_joint_seterr(j, "endpoint not found: %s", path);
    }
    return st;
}

/* ==========================================================================
 * 参数值 ↔ 主机序：**小端**
 *
 * ⚠⚠ 参数值在线上是**小端**（设备端 `memcpy` 主机序），与控制/查询帧的大端
 *   相反。完整理由（固件源码锚点 + 真机实测三例）见 `cb_frame.h` 的小端
 *   存取器说明 —— 那里是这条约定的唯一权威注释。
 * ======================================================================== */

static void le_to_value(jsdk_ep_type_t t, const uint8_t *b, jsdk_value_t *out)
{
    memset(out, 0, sizeof *out);
    out->type = t;

    switch (t) {
    case JSDK_EP_U8:  out->v.u8  = b[0]; break;
    case JSDK_EP_I8:  out->v.i8  = (int8_t)b[0]; break;
    case JSDK_EP_BOOL: out->v.boolean = (b[0] != 0u) ? 1 : 0; break;
    case JSDK_EP_U16: out->v.u16 = cb_le_get_u16(b); break;
    case JSDK_EP_I16: out->v.i16 = cb_le_get_i16(b); break;
    case JSDK_EP_U32: out->v.u32 = cb_le_get_u32(b); break;
    case JSDK_EP_I32: out->v.i32 = cb_le_get_i32(b); break;
    case JSDK_EP_U64: out->v.u64 = cb_le_get_u64(b); break;
    case JSDK_EP_I64: out->v.i64 = (int64_t)cb_le_get_u64(b); break;
    case JSDK_EP_F32: out->v.f32 = cb_le_get_f32(b); break;
    case JSDK_EP_F64: {
        /* 协议里没有 f64 线格式；按两个 u32 拼（保留位，仅用于透传） */
        uint64_t lo = cb_le_get_u32(b);
        uint64_t hi = cb_le_get_u32(b + 4);
        uint64_t raw = (hi << 32) | lo;
        double d;
        memcpy(&d, &raw, sizeof d);      /* 位模式搬运，不做数值转换 */
        out->v.f64 = d;
        break;
    }
    default:
        break;                            /* 不透明类型：调用方不该走到这里 */
    }
}

static int value_to_le(jsdk_ep_type_t t, const jsdk_value_t *in,
                       uint8_t *b, uint8_t *out_len)
{
    unsigned w = jsdk_ep_type_size(t);

    if (w == 0u) return -1;
    memset(b, 0, 8u);

    switch (t) {
    case JSDK_EP_U8:   b[0] = in->v.u8; break;
    case JSDK_EP_I8:   b[0] = (uint8_t)in->v.i8; break;
    case JSDK_EP_BOOL: b[0] = (uint8_t)(in->v.boolean ? 1 : 0); break;
    case JSDK_EP_U16:  cb_le_put_u16(b, in->v.u16); break;
    case JSDK_EP_I16:  cb_le_put_i16(b, in->v.i16); break;
    case JSDK_EP_U32:  cb_le_put_u32(b, in->v.u32); break;
    case JSDK_EP_I32:  cb_le_put_i32(b, in->v.i32); break;
    case JSDK_EP_U64:  cb_le_put_u64(b, in->v.u64); break;
    case JSDK_EP_I64:  cb_le_put_u64(b, (uint64_t)in->v.i64); break;
    case JSDK_EP_F32:  cb_le_put_f32(b, in->v.f32); break;
    case JSDK_EP_F64: {
        uint64_t raw;
        memcpy(&raw, &in->v.f64, sizeof raw);
        cb_le_put_u32(b, (uint32_t)raw);
        cb_le_put_u32(b + 4, (uint32_t)(raw >> 32));
        break;
    }
    default:
        return -1;
    }

    *out_len = (uint8_t)w;
    return 0;
}

/* ==========================================================================
 * 类型化 get / set
 * ======================================================================== */

jsdk_status_t jsdk_joint_param_get(jsdk_joint_t *j, const char *path,
                                   jsdk_value_t *out)
{
    uint16_t ep = 0u;
    jsdk_ep_type_t t = JSDK_EP_JSON;
    uint8_t  access = 0u;
    uint8_t  buf[8];
    uint8_t  len = 0u;
    uint8_t  need;
    jsdk_status_t st;

    if (!out) return JSDK_ERR_INVALID_ARG;
    st = resolve(j, path, &ep, &t, &access);
    if (st != JSDK_OK) return st;
    if (!(access & JSDK_EP_ACCESS_R)) {
        jsdk_joint_seterr(j, "endpoint is write-only: %s", path);
        return JSDK_ERR_UNSUPPORTED;
    }
    need = (uint8_t)jsdk_ep_type_size(t);
    if (need == 0u) {
        jsdk_joint_seterr(j, "endpoint type is not directly readable: %s (%s)",
                          path, jsdk_ep_type_string(t));
        return JSDK_ERR_UNSUPPORTED;
    }

    st = (jsdk_status_t)jsdk_ctx_read_param_exact(j->ctx, j->cfg.node_id, ep,
                                                  buf, need, &len, 0u);
    if (st != JSDK_OK) {
        jsdk_joint_seterr(j, "read %s failed (%s)", path, jsdk_status_string(st));
        return st;
    }
    if (len != need) {
        /* read_param_exact 成功时保证 len == need；走到这里说明契约被破了 */
        jsdk_joint_seterr(j, "read %s: got %u bytes, expected %u",
                          path, (unsigned)len, (unsigned)need);
        return JSDK_ERR_PROTOCOL;
    }

    le_to_value(t, buf, out);
    return JSDK_OK;
}

jsdk_status_t jsdk_joint_param_set(jsdk_joint_t *j, const char *path,
                                   const jsdk_value_t *in)
{
    uint16_t ep = 0u;
    jsdk_ep_type_t t = JSDK_EP_JSON;
    uint8_t  access = 0u;
    uint8_t  buf[8];
    uint8_t  len = 0u;
    jsdk_status_t st;

    if (!in) return JSDK_ERR_INVALID_ARG;
    st = resolve(j, path, &ep, &t, &access);
    if (st != JSDK_OK) return st;
    if (!(access & JSDK_EP_ACCESS_W)) {
        jsdk_joint_seterr(j, "endpoint is read-only: %s", path);
        return JSDK_ERR_UNSUPPORTED;
    }
    if (in->type != t) {
        /* 类型不匹配必须报错：猜宽度会静默写坏邻近字段 */
        jsdk_joint_seterr(j, "type mismatch for %s: descriptor=%s given=%s",
                          path, jsdk_ep_type_string(t),
                          jsdk_ep_type_string(in->type));
        return JSDK_ERR_PROTOCOL;
    }
    if (value_to_le(t, in, buf, &len) != 0) {
        return JSDK_ERR_UNSUPPORTED;
    }

    st = (jsdk_status_t)jsdk_ctx_write_param(j->ctx, j->cfg.node_id, ep,
                                             buf, len, 0u);
    if (st != JSDK_OK) {
        jsdk_joint_seterr(j, "write %s failed (%s)", path, jsdk_status_string(st));
    }
    return st;
}

/* ==========================================================================
 * 便捷包装
 * ======================================================================== */

/** 共用的“读 + 按目标类型取值”。 */
static jsdk_status_t get_scalar(jsdk_joint_t *j, const char *path,
                                jsdk_value_t *v)
{
    return jsdk_joint_param_get(j, path, v);
}

jsdk_status_t jsdk_joint_param_get_f32(jsdk_joint_t *j, const char *path, float *out)
{
    jsdk_value_t v;
    jsdk_status_t st;

    if (!out) return JSDK_ERR_INVALID_ARG;
    st = get_scalar(j, path, &v);
    if (st != JSDK_OK) return st;
    switch (v.type) {
    case JSDK_EP_F32: *out = v.v.f32; return JSDK_OK;
    case JSDK_EP_U8:  *out = (float)v.v.u8;  return JSDK_OK;
    case JSDK_EP_I8:  *out = (float)v.v.i8;  return JSDK_OK;
    case JSDK_EP_U16: *out = (float)v.v.u16; return JSDK_OK;
    case JSDK_EP_I16: *out = (float)v.v.i16; return JSDK_OK;
    case JSDK_EP_U32: *out = (float)v.v.u32; return JSDK_OK;
    case JSDK_EP_I32: *out = (float)v.v.i32; return JSDK_OK;
    case JSDK_EP_BOOL: *out = (float)v.v.boolean; return JSDK_OK;
    default:
        jsdk_joint_seterr(j, "%s is %s, not a float", path, jsdk_ep_type_string(v.type));
        return JSDK_ERR_PROTOCOL;
    }
}

jsdk_status_t jsdk_joint_param_set_f32(jsdk_joint_t *j, const char *path, float v)
{
    jsdk_value_t in;

    memset(&in, 0, sizeof in);
    in.type = JSDK_EP_F32;
    in.v.f32 = v;
    return jsdk_joint_param_set(j, path, &in);
}

jsdk_status_t jsdk_joint_param_get_u32(jsdk_joint_t *j, const char *path, uint32_t *out)
{
    jsdk_value_t v;
    jsdk_status_t st;

    if (!out) return JSDK_ERR_INVALID_ARG;
    st = get_scalar(j, path, &v);
    if (st != JSDK_OK) return st;
    switch (v.type) {
    case JSDK_EP_U32: *out = v.v.u32; return JSDK_OK;
    case JSDK_EP_U16: *out = v.v.u16; return JSDK_OK;
    case JSDK_EP_U8:  *out = v.v.u8;  return JSDK_OK;
    case JSDK_EP_BOOL: *out = (uint32_t)v.v.boolean; return JSDK_OK;
    default:
        jsdk_joint_seterr(j, "%s is %s, not an unsigned integer",
                          path, jsdk_ep_type_string(v.type));
        return JSDK_ERR_PROTOCOL;
    }
}

jsdk_status_t jsdk_joint_param_set_u32(jsdk_joint_t *j, const char *path, uint32_t v)
{
    jsdk_value_t in;
    uint16_t ep = 0u;
    jsdk_ep_type_t t = JSDK_EP_JSON;
    uint8_t access = 0u;

    /* 目标端点的**真实宽度**由描述符决定：给 u32 值写 u16 端点要按宽度降级，
       但绝不能溢出（超出范围由固件截断） */
    if (resolve(j, path, &ep, &t, &access) != JSDK_OK) return JSDK_ERR_NOT_FOUND;

    memset(&in, 0, sizeof in);
    in.type = t;
    switch (t) {
    case JSDK_EP_U8:  in.v.u8  = (uint8_t)v;  break;
    case JSDK_EP_U16: in.v.u16 = (uint16_t)v; break;
    case JSDK_EP_U32: in.v.u32 = v;           break;
    case JSDK_EP_BOOL: in.v.boolean = (v != 0u) ? 1 : 0; break;
    default:
        jsdk_joint_seterr(j, "%s is %s; use the typed param_set() instead",
                          path, jsdk_ep_type_string(t));
        return JSDK_ERR_PROTOCOL;
    }
    return jsdk_joint_param_set(j, path, &in);
}

jsdk_status_t jsdk_joint_param_get_i32(jsdk_joint_t *j, const char *path, int32_t *out)
{
    jsdk_value_t v;
    jsdk_status_t st;

    if (!out) return JSDK_ERR_INVALID_ARG;
    st = get_scalar(j, path, &v);
    if (st != JSDK_OK) return st;
    switch (v.type) {
    case JSDK_EP_I32: *out = v.v.i32; return JSDK_OK;
    case JSDK_EP_I16: *out = v.v.i16; return JSDK_OK;
    case JSDK_EP_I8:  *out = v.v.i8;  return JSDK_OK;
    case JSDK_EP_U32: *out = (int32_t)v.v.u32; return JSDK_OK;
    default:
        jsdk_joint_seterr(j, "%s is %s, not a signed integer",
                          path, jsdk_ep_type_string(v.type));
        return JSDK_ERR_PROTOCOL;
    }
}

jsdk_status_t jsdk_joint_param_get_bool(jsdk_joint_t *j, const char *path, int *out)
{
    jsdk_value_t v;
    jsdk_status_t st;

    if (!out) return JSDK_ERR_INVALID_ARG;
    st = get_scalar(j, path, &v);
    if (st != JSDK_OK) return st;
    if (v.type == JSDK_EP_BOOL) { *out = v.v.boolean; return JSDK_OK; }
    if (v.type == JSDK_EP_U8)   { *out = (v.v.u8 != 0u) ? 1 : 0; return JSDK_OK; }
    jsdk_joint_seterr(j, "%s is %s, not a bool", path, jsdk_ep_type_string(v.type));
    return JSDK_ERR_PROTOCOL;
}

/* ==========================================================================
 * 批量读
 * ======================================================================== */

/** Classic：自动退化为逐条单读（固件对 Classic 批量请求恒回 ERR）。 */
static void batch_serial(jsdk_joint_t *j, jsdk_param_req_t *reqs, unsigned n)
{
    unsigned i;

    for (i = 0u; i < n; ++i) {
        if (!reqs[i].path) { reqs[i].status = JSDK_ERR_INVALID_ARG; continue; }
        reqs[i].status = jsdk_joint_param_get(j, reqs[i].path, &reqs[i].value);
    }
}

jsdk_status_t jsdk_joint_param_get_batch(jsdk_joint_t *j, jsdk_param_req_t *reqs,
                                        unsigned n)
{
    cb_param_batch_item_t plan[CB_PARAM_MAX_BATCH];
    unsigned  idx[CB_PARAM_MAX_BATCH];
    jsdk_ep_type_t types[CB_PARAM_MAX_BATCH];
    unsigned  n_plan = 0u;
    unsigned  i;

    if (!jsdk_joint_check(j) || (!reqs && n > 0u)) return JSDK_ERR_INVALID_ARG;
    if (n == 0u) return JSDK_OK;
    if (!j->ctx->desc_present) return JSDK_ERR_BAD_STATE;

    for (i = 0u; i < n; ++i) reqs[i].status = JSDK_ERR_NOT_FOUND;

    /* Classic：固件对批量请求**恒回 ERR**（不是我们没实现），
       所以别浪费一次往返 —— 直接逐条单读，让客户看不到 ERR。 */
    if (!j->ctx->cfg.is_fd) {
        batch_serial(j, reqs, n);
        return JSDK_OK;
    }

    /* --- 1. 解析：能打包的进 plan，不能的（未命中/只写/不透明）留在原地 --- */
    for (i = 0u; i < n && n_plan < CB_PARAM_MAX_BATCH; ++i) {
        uint16_t ep = 0u;
        jsdk_ep_type_t t = JSDK_EP_JSON;
        uint8_t access = 0u;
        uint8_t w;

        if (!reqs[i].path) { reqs[i].status = JSDK_ERR_INVALID_ARG; continue; }
        if (resolve(j, reqs[i].path, &ep, &t, &access) != JSDK_OK) continue;
        if (!(access & JSDK_EP_ACCESS_R)) {
            reqs[i].status = JSDK_ERR_UNSUPPORTED;
            continue;
        }
        w = (uint8_t)jsdk_ep_type_size(t);
        if (w == 0u) { reqs[i].status = JSDK_ERR_UNSUPPORTED; continue; }

        plan[n_plan].ep_id     = ep;
        plan[n_plan].value_len = w;
        idx[n_plan]   = i;
        types[n_plan] = t;
        n_plan++;
    }

    /* --- 2. 装箱（budget = 单帧 64 B）并逐批收发 --- */
    if (n_plan > 0u) {
        size_t counts[CB_PARAM_MAX_BATCH];
        size_t offsets[CB_PARAM_MAX_BATCH];
        size_t n_batches;
        size_t b;
        size_t base = 0u;

        n_batches = cb_param_plan_batches(plan, n_plan, CB_PARAM_FD_FRAME_MAX,
                                          counts, offsets, CB_PARAM_MAX_BATCH);
        if (n_batches == 0u) {
            /* 有条目连单帧都装不下（值 ≤ 8 B，正常不会发生）→ 老实逐条 */
            batch_serial(j, reqs, n);
            return JSDK_OK;
        }

        for (b = 0u; b < n_batches; ++b) {
            uint8_t  req[CB_PARAM_FD_FRAME_MAX];
            jsdk_can_frame_t frame;
            uint16_t eps[CB_PARAM_MAX_BATCH];   /* ep_id 是 u16，别收窄成 u8 */
            size_t   cnt = counts[b];
            size_t   k;
            size_t   rq_len;

            for (k = 0u; k < cnt; ++k) eps[k] = plan[base + k].ep_id;

            rq_len = cb_param_pack_batch_req(req, sizeof req, eps, (uint8_t)cnt);
            if (rq_len == 0u) { batch_serial(j, reqs, n); return JSDK_OK; }

            if (jsdk_ctx_request_retry(j->ctx, CB_PRI_CONFIG, CB_MSG_PARAM_READ,
                                       j->cfg.node_id, req, (uint8_t)rq_len,
                                       &frame, JSDK_CFG_TIMEOUT_MS) != JSDK_OK) {
                batch_serial(j, reqs, n);
                return JSDK_OK;
            }

            /* --- 3. 解析响应；形状不对就整批逐条兜底 ---
               ⚠ 绝不把“错位的字节”当成值用：批量响应是**值流 + 位图**，
                  一旦位图或长度对不上，后续所有值都会整体滑动错位。 */
            {
                cb_param_batch_rsp_t rsp;
                size_t off = 0u;
                int usable = 1;

                if (cb_param_unpack_batch_rsp(frame.data, frame.len,
                                              (uint8_t)cnt, &rsp) != 0
                    || rsp.is_err) {
                    usable = 0;
                }

                if (usable) {
                    for (k = 0u; k < cnt; ++k) {
                        unsigned w = plan[base + k].value_len;
                        unsigned slot = idx[base + k];

                        if (!cb_param_bitmap_test(rsp.bitmap, rsp.bitmap_bytes,
                                                  (uint8_t)k)) {
                            reqs[slot].status = JSDK_ERR_NOT_FOUND;  /* 设备说没取到 */
                            continue;
                        }
                        if (off + w > rsp.values_len) { usable = 0; break; }
                        le_to_value(types[base + k], rsp.values + off,
                                    &reqs[slot].value);
                        reqs[slot].status = JSDK_OK;
                        off += w;
                    }
                }

                if (!usable) {
                    for (k = 0u; k < cnt; ++k) {
                        unsigned slot = idx[base + k];
                        reqs[slot].status = jsdk_joint_param_get(j, reqs[slot].path,
                                                                 &reqs[slot].value);
                    }
                }
            }

            base += cnt;
        }
    }

    return JSDK_OK;
}

/* ==========================================================================
 * SDO 风格槽位
 * ======================================================================== */

jsdk_sdo_handle_t jsdk_joint_sdo_create(jsdk_joint_t *j, uint16_t ep_id,
                                        uint8_t subindex, size_t size)
{
    unsigned i;
    jsdk_ep_type_t t = JSDK_EP_JSON;
    uint8_t access = 0u;

    if (!jsdk_joint_check(j)) return -1;
    /* 本协议的 subindex 恒为 0（端点 ID 是平铺的，没有子索引概念） */
    if (subindex != 0u) {
        jsdk_joint_seterr(j, "subindex must be 0 for CYBERBEAST (flat endpoint ids)");
        return -1;
    }
    if (size == 0u) {
        /* 从描述符推断宽度 */
        unsigned n = jsdk_ep_store_count(&j->ctx->store);
        for (i = 0u; i < n; ++i) {
            uint16_t id = 0u;
            if (jsdk_ep_store_at(&j->ctx->store, i, NULL, &id, &t, &access) != JSDK_OK) {
                continue;
            }
            if (id == ep_id) break;
        }
        if (i >= n) {
            jsdk_joint_seterr(j, "endpoint %u not in descriptor", (unsigned)ep_id);
            return -1;
        }
        size = jsdk_ep_type_size(t);
        if (size == 0u || size > 8u) {
            jsdk_joint_seterr(j, "endpoint %u has no scalar width", (unsigned)ep_id);
            return -1;
        }
    }
    if (size > 8u) return -1;

    /* 槽位 0 保留给故障详情自动读取 */
    for (i = 1u; i < JSDK_SDO_SLOTS; ++i) {
        if (!j->sdo[i].in_use) {
            j->sdo[i].in_use = 1u;
            j->sdo[i].ep_id  = ep_id;
            j->sdo[i].size   = (uint16_t)size;
            j->sdo[i].state  = (uint8_t)JSDK_SDO_IDLE;
            memset(j->sdo[i].data, 0, sizeof j->sdo[i].data);
            return (jsdk_sdo_handle_t)i;
        }
    }
    jsdk_joint_seterr(j, "no free SDO slot (max %u per joint)", (unsigned)JSDK_SDO_SLOTS);
    return -1;
}

jsdk_sdo_handle_t jsdk_joint_sdo_create_by_name(jsdk_joint_t *j, const char *path)
{
    uint16_t ep = 0u;
    jsdk_ep_type_t t = JSDK_EP_JSON;
    uint8_t access = 0u;

    if (resolve(j, path, &ep, &t, &access) != JSDK_OK) return -1;
    return jsdk_joint_sdo_create(j, ep, 0u, jsdk_ep_type_size(t));
}

static jsdk_sdo_slot_t *sdo_slot(jsdk_joint_t *j, jsdk_sdo_handle_t h)
{
    if (!jsdk_joint_check(j)) return NULL;
    if (h <= 0 || (unsigned)h >= JSDK_SDO_SLOTS) return NULL;
    if (!j->sdo[h].in_use) return NULL;
    return &j->sdo[h];
}

jsdk_sdo_state_t jsdk_joint_sdo_state(jsdk_joint_t *j, jsdk_sdo_handle_t h)
{
    jsdk_sdo_slot_t *s = sdo_slot(j, h);
    return s ? (jsdk_sdo_state_t)s->state : JSDK_SDO_ERROR;
}

uint8_t *jsdk_joint_sdo_data(jsdk_joint_t *j, jsdk_sdo_handle_t h)
{
    jsdk_sdo_slot_t *s = sdo_slot(j, h);
    return s ? s->data : NULL;
}

size_t jsdk_joint_sdo_data_size(jsdk_joint_t *j, jsdk_sdo_handle_t h)
{
    jsdk_sdo_slot_t *s = sdo_slot(j, h);
    return s ? (size_t)s->size : 0u;
}

int jsdk_joint_sdo_read(jsdk_joint_t *j, jsdk_sdo_handle_t h)
{
    jsdk_sdo_slot_t *s = sdo_slot(j, h);
    uint8_t buf[8];
    uint8_t len = 0u;
    int rc;

    if (!s) return JSDK_ERR_INVALID_ARG;
    s->state = (uint8_t)JSDK_SDO_BUSY;

    /* SDO 的 size ≤ 8（见 jsdk_joint_sdo_create），所以精确读一定能读满；
       Classic 下这段会自己分两块（每块 4 B），不需要调用方关心。 */
    rc = jsdk_ctx_read_param_exact(j->ctx, j->cfg.node_id, s->ep_id,
                                   buf, (uint8_t)s->size, &len, 0u);
    if (rc != JSDK_OK || len < (uint8_t)s->size) {
        s->state = (uint8_t)JSDK_SDO_ERROR;
        return (rc != JSDK_OK) ? rc : JSDK_ERR_PROTOCOL;
    }
    memcpy(s->data, buf, s->size);
    s->state = (uint8_t)JSDK_SDO_SUCCESS;
    return JSDK_OK;
}

int jsdk_joint_sdo_write(jsdk_joint_t *j, jsdk_sdo_handle_t h)
{
    jsdk_sdo_slot_t *s = sdo_slot(j, h);
    jsdk_ep_type_t t = JSDK_EP_JSON;
    uint8_t access = 0u;
    unsigned n, i;
    int rc;

    if (!s) return JSDK_ERR_INVALID_ARG;

    /* 写之前必须确认端点可写（描述符里带 w 权限） */
    n = jsdk_ep_store_count(&j->ctx->store);
    for (i = 0u; i < n; ++i) {
        uint16_t id = 0u;
        if (jsdk_ep_store_at(&j->ctx->store, i, NULL, &id, &t, &access) != JSDK_OK) {
            continue;
        }
        if (id == s->ep_id) break;
    }
    if (i >= n || !(access & JSDK_EP_ACCESS_W)) {
        s->state = (uint8_t)JSDK_SDO_ERROR;
        jsdk_joint_seterr(j, "endpoint %u is not writable", (unsigned)s->ep_id);
        return JSDK_ERR_UNSUPPORTED;
    }

    s->state = (uint8_t)JSDK_SDO_BUSY;
    rc = jsdk_ctx_write_param(j->ctx, j->cfg.node_id, s->ep_id,
                              s->data, (uint8_t)s->size, 0u);
    s->state = (uint8_t)((rc == JSDK_OK) ? JSDK_SDO_SUCCESS : JSDK_SDO_ERROR);
    return rc;
}

/* ==========================================================================
 * function 端点调用（Fibre 方法）
 *
 * 线上序列（与 ODrive Fibre 一致）：
 *   1. 逐个写 inputs —— 每个入参在描述符里都有自己的端点 ID，
 *      路径 = "<function_path>.<input_name>"
 *   2. 写 function 端点本身（触发设备侧执行）
 *   3. 逐个读 outputs（同样各有自己的端点 ID）
 *
 * ⚠ 为什么不能用现成的 param_set/param_get：
 *   (a) function 端点**没有 access 字段** ⇒ param_set 会以"read-only"拒绝；
 *   (b) `jsdk_ep_type_size(FUNCTION) == 0` ⇒ value_to_le 直接失败。
 *   所以本函数自己按路径解析子端点并编解码。
 *
 * ⚠ 端点发现不靠"猜名字"：解析器把 inputs/outputs 里的嵌套对象**自动展平**成
 *   带路径的端点（实测 v8：`axis0.controller.move_incremental.displacement`
 *   → id 350，access=rw；其 output 为 `…remove_anticogging_bias.val`，access=r）。
 *   因此这里用精确查表；任一子端点缺失就在**发任何帧之前**返回 NOT_FOUND，
 *   绝不"写了一半才发现不对"。
 *
 * ⚠ 子端点顺序 = 描述符声明顺序 = `jsdk_ep_store_at()` 的遍历顺序
 *   （解析器按 JSON 文本顺序 emit）。inputs 在前、outputs 在后由**协议生成器**
 *   保证；这里不假设两者在文本里的相对次序，而是**按 access 位**区分：
 *   可写 → input，不可写 → output。这与固件语义一致（入参 rw，出参 r）。
 * ======================================================================== */

/** 收集 `<fn_path>.<name>` 形式的**直接**子端点（一层，名字里不再含 '.'）。 */
static unsigned fn_children(jsdk_joint_t *j, const char *fn_path,
                            uint16_t *ids, jsdk_ep_type_t *types, uint8_t *accs,
                            unsigned max)
{
    size_t flen = strlen(fn_path);
    unsigned n = jsdk_ep_store_count(&j->ctx->store);
    unsigned i, got = 0u;

    for (i = 0u; i < n && got < max; ++i) {
        const char *path = NULL;
        uint16_t id = 0u;
        jsdk_ep_type_t ty = JSDK_EP_JSON;
        uint8_t acc = 0u;

        if (jsdk_ep_store_at(&j->ctx->store, i, &path, &id, &ty, &acc) != JSDK_OK) {
            continue;
        }
        if (!path) continue;
        if (strncmp(path, fn_path, flen) != 0) continue;
        if (path[flen] != '.') continue;
        if (strchr(path + flen + 1u, '.') != NULL) continue;   /* 只要直接子节点 */

        ids[got]   = id;
        types[got] = ty;
        accs[got]  = acc;
        got++;
    }
    return got;
}

/** function 子端点上限：实测 v8 最多 3 个（inputs+outputs 合计）。留足余量。 */
#define JSDK_FN_MAX_CHILDREN 8u

jsdk_status_t jsdk_joint_ep_invoke(jsdk_joint_t *j, const char *path,
                                   const jsdk_value_t *in, unsigned n_in,
                                   jsdk_value_t *out,      unsigned n_out,
                                   unsigned *out_got)
{
    uint16_t fn_ep = 0u;
    jsdk_ep_type_t fn_type = JSDK_EP_JSON;
    uint8_t fn_access = 0u;
    jsdk_status_t st;
    uint16_t      cids[JSDK_FN_MAX_CHILDREN];
    jsdk_ep_type_t ctypes[JSDK_FN_MAX_CHILDREN];
    uint8_t       caccs[JSDK_FN_MAX_CHILDREN];
    unsigned n_child, n_in_ep = 0u, n_out_ep = 0u, k;

    if (out_got) *out_got = 0u;
    if (!j || !path) return JSDK_ERR_INVALID_ARG;
    if (!j->ctx->desc_present) {
        jsdk_joint_seterr(j, "descriptor not available: configure() first");
        return JSDK_ERR_BAD_STATE;
    }

    /* --- 0. 目标必须是 function --- */
    st = resolve(j, path, &fn_ep, &fn_type, &fn_access);
    if (st != JSDK_OK) return st;
    if (fn_type != JSDK_EP_FUNCTION) {
        jsdk_joint_seterr(j, "not a function endpoint: %s (type=%s)", path,
                          jsdk_ep_type_string(fn_type));
        return JSDK_ERR_UNSUPPORTED;
    }

    /*
     * --- 1. 预勘察：解析全部子端点，保证"要么全都合法、要么一帧都不发" ---
     *
     * 若边写边查，中途失败会把设备 input 改成"半新半旧"，下一次调用可能用错
     * 参数 —— 比直接报错危险得多。
     */
    n_child = fn_children(j, path, cids, ctypes, caccs, JSDK_FN_MAX_CHILDREN);
    for (k = 0u; k < n_child; ++k) {
        if (!jsdk_ep_type_is_scalar(ctypes[k]) || jsdk_ep_type_size(ctypes[k]) == 0u) {
            jsdk_joint_seterr(j, "function child %u is not a scalar (type=%s)", k,
                              jsdk_ep_type_string(ctypes[k]));
            return JSDK_ERR_UNSUPPORTED;
        }
        if (caccs[k] & JSDK_EP_ACCESS_W) n_in_ep++;
        else                            n_out_ep++;
    }

    if (n_in != n_in_ep) {
        jsdk_joint_seterr(j, "%s expects %u input(s), got %u", path, n_in_ep, n_in);
        return JSDK_ERR_INVALID_ARG;
    }

    /* --- 2. 逐个写 inputs（按声明顺序，第 written 个对应 in[written]） --- */
    {
        unsigned written = 0u;

        for (k = 0u; k < n_child && written < n_in_ep; ++k) {
            uint8_t buf[8];
            uint8_t len = 0u;

            if (!(caccs[k] & JSDK_EP_ACCESS_W)) continue;   /* 是 output */
            if (in[written].type != ctypes[k]) {
                jsdk_joint_seterr(j,
                    "input %u type mismatch for %s: descriptor=%s given=%s",
                    written, path, jsdk_ep_type_string(ctypes[k]),
                    jsdk_ep_type_string(in[written].type));
                return JSDK_ERR_PROTOCOL;
            }
            if (value_to_le(ctypes[k], &in[written], buf, &len) != 0) {
                jsdk_joint_seterr(j, "input %u cannot be encoded", written);
                return JSDK_ERR_UNSUPPORTED;
            }
            {
                int rc = jsdk_ctx_write_param(j->ctx, j->cfg.node_id, cids[k],
                                              buf, len, 0u);
                if (rc != JSDK_OK) {
                    jsdk_joint_seterr(j, "write input %u failed (%s)", written,
                                      jsdk_status_string((jsdk_status_t)rc));
                    return (jsdk_status_t)rc;
                }
            }
            written++;
        }
    }

    /* --- 3. 写 function 端点本身：这一步触发设备侧执行 --- */
    {
        int rc = jsdk_ctx_write_param(j->ctx, j->cfg.node_id, fn_ep, NULL, 0u, 0u);
        if (rc != JSDK_OK) {
            jsdk_joint_seterr(j, "invoke %s failed (%s)", path,
                              jsdk_status_string((jsdk_status_t)rc));
            return (jsdk_status_t)rc;
        }
    }

    /* --- 4. 逐个读 outputs --- */
    {
        unsigned got = 0u;

        for (k = 0u; k < n_child; ++k) {
            uint8_t buf[8];
            uint8_t need, len = 0u;

            if (caccs[k] & JSDK_EP_ACCESS_W) continue;      /* 是 input */

            if (out && got < n_out) {
                need = (uint8_t)jsdk_ep_type_size(ctypes[k]);
                {
                    int rc = jsdk_ctx_read_param_exact(j->ctx, j->cfg.node_id,
                                                       cids[k], buf, need, &len, 0u);
                    if (rc != JSDK_OK) {
                        jsdk_joint_seterr(j, "read output %u failed (%s)", got,
                                          jsdk_status_string((jsdk_status_t)rc));
                        return (jsdk_status_t)rc;
                    }
                }
                le_to_value(ctypes[k], buf, &out[got]);
            }
            got++;
        }
        if (out_got) *out_got = got;
    }

    return JSDK_OK;
}
