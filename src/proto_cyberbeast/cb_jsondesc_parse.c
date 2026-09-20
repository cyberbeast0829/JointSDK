/**
 * @file    cb_jsondesc_parse.c
 * @brief   CYBERBEAST JSON 端点描述符的**增量解析器** + 端点存储（arena）
 *
 * 为什么不用现成 JSON 库：描述符实测 41029 字节 / 594 个端点，
 * 受限 MCU 既拿不出 41 KB 缓冲，也不该为它引入动态分配与递归解析器。
 * 本实现：
 *   - 逐字节状态机（可 62 字节/帧地喂入），**无递归、无 malloc**；
 *   - 解析结果写进调用者提供的 arena（条目区向前、路径池向后增长）；
 *   - 路径在解析 name 字段时增量拼接，故 key 与 id/type 的先后顺序无关；
 *   - 任何异常（截断/非法字符/未知 type/超限）→ 整体失败，**不留部分结果**。
 *
 * 协议细节见 docs/PROTOCOL_NOTES.zh-CN.md §5.6 / §9.2。
 */

#include "jsdk_internal.h"
#include <string.h>

/* ==========================================================================
 * 小工具
 * ======================================================================== */

static int is_ws(uint8_t c)
{
    return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

static int is_digit(uint8_t c) { return c >= '0' && c <= '9'; }
static int is_alpha(uint8_t c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); }

static int jfail(jsdk_jsondesc_t *p, const char *msg)
{
    if (p->state != JSDK_JS_FAIL) {
        p->state = JSDK_JS_FAIL;
        p->err = msg;
        p->fail_code = JSDK_ERR_PARSE;
        /* 保证“失败不留部分结果”：调用者即使误用 store 也读不到任何东西 */
        if (p->store) {
            p->store->arena.entry_count = 0;
            p->store->arena.blob_top    = p->store->arena.size;
            p->store->arena.blob_used   = 0;
        }
    }
    return p->fail_code;
}

/**
 * 与 jfail 相同，但错误码用 JSDK_ERR_NO_MEMORY。
 *
 * 公共 API 明确承诺「arena 不足 → JSDK_ERR_NO_MEMORY」（见 joint_sdk.h 的
 * jsdk_desc_config_t.arena 与 jsdk_context_desc_import_raw 的说明）。
 * 这个区分很实在：调用方的补救动作完全不同——
 *   NO_MEMORY → 扩容 arena 或收紧 filter；PARSE → 固件发来的 JSON 有问题。
 */
static int jfail_mem(jsdk_jsondesc_t *p, const char *msg)
{
    (void)jfail(p, msg);
    p->fail_code = JSDK_ERR_NO_MEMORY;
    return JSDK_ERR_NO_MEMORY;
}

static int base_append(jsdk_jsondesc_t *p, char c)
{
    if ((size_t)p->cur_base_len + 2u > JSDK_EP_PATH_MAX_HARD)
        return jfail(p, "path exceeds JSDK_EP_PATH_MAX_HARD");
    p->cur_base[p->cur_base_len++] = c;
    p->cur_base[p->cur_base_len] = '\0';
    return JSDK_OK;
}

/* ==========================================================================
 * 类型映射
 * ======================================================================== */

uint8_t jsdk_ep_type_from_name(const char *name, size_t len)
{
    static const struct { const char *n; uint8_t v; } tbl[] = {
        {"uint8",        JSDK_EP_U8},
        {"int8",         JSDK_EP_I8},
        {"uint16",       JSDK_EP_U16},
        {"int16",        JSDK_EP_I16},
        {"uint32",       JSDK_EP_U32},
        {"int32",        JSDK_EP_I32},
        {"uint64",       JSDK_EP_U64},
        {"int64",        JSDK_EP_I64},
        {"float",        JSDK_EP_F32},
        {"double",       JSDK_EP_F64},
        {"bool",         JSDK_EP_BOOL},
        {"object",       JSDK_EP_OBJECT},
        {"endpoint_ref", JSDK_EP_ENDPOINT_REF},
        {"json",         JSDK_EP_JSON},
        {"function",     JSDK_EP_FUNCTION}
    };
    size_t i;
    for (i = 0; i < sizeof tbl / sizeof tbl[0]; ++i) {
        if (strlen(tbl[i].n) == len && memcmp(tbl[i].n, name, len) == 0)
            return tbl[i].v;
    }
    return 0;
}

const char *jsdk_ep_type_name(jsdk_ep_type_t t)
{
    switch (t) {
    case JSDK_EP_U8:  return "uint8";
    case JSDK_EP_I8:  return "int8";
    case JSDK_EP_U16: return "uint16";
    case JSDK_EP_I16: return "int16";
    case JSDK_EP_U32: return "uint32";
    case JSDK_EP_I32: return "int32";
    case JSDK_EP_U64: return "uint64";
    case JSDK_EP_I64: return "int64";
    case JSDK_EP_F32: return "float";
    case JSDK_EP_F64: return "double";
    case JSDK_EP_BOOL: return "bool";
    case JSDK_EP_OBJECT: return "object";
    case JSDK_EP_ENDPOINT_REF: return "endpoint_ref";
    case JSDK_EP_JSON: return "json";
    case JSDK_EP_FUNCTION: return "function";
    default: return "?";
    }
}

int jsdk_ep_type_is_scalar(jsdk_ep_type_t t)
{
    return (t >= JSDK_EP_U8 && t <= JSDK_EP_BOOL);
}

unsigned jsdk_ep_type_size(jsdk_ep_type_t t)
{
    switch (t) {
    case JSDK_EP_U8:
    case JSDK_EP_I8:
    case JSDK_EP_BOOL:  return 1u;
    case JSDK_EP_U16:
    case JSDK_EP_I16:   return 2u;
    case JSDK_EP_U32:
    case JSDK_EP_I32:
    case JSDK_EP_F32:   return 4u;
    case JSDK_EP_U64:
    case JSDK_EP_I64:
    case JSDK_EP_F64:   return 8u;
    default:            return 0u;
    }
}

/* ==========================================================================
 * arena / 端点存储
 * ======================================================================== */

void jsdk_ep_store_reset(jsdk_ep_store_t *s)
{
    if (!s) return;
    s->arena.entry_count = 0;
    s->arena.blob_top    = s->arena.size;
    s->arena.blob_used   = 0;
    s->parsed_total      = 0;
}

int jsdk_ep_arena_put(jsdk_arena_t *a, const char *path, size_t len,
                      uint16_t ep_id, uint8_t type, uint8_t access)
{
    jsdk_ep_entry_t e;
    size_t gap;

    if (!a || !a->base || !path || len == 0) return JSDK_ERR_INVALID_ARG;

    /* 条目区（前→后）与路径池（后→前）之间的空隙 */
    gap = a->blob_top - (size_t)a->entry_count * sizeof(jsdk_ep_entry_t);
    if (gap < len + sizeof(jsdk_ep_entry_t)) return JSDK_ERR_NO_MEMORY;

    a->blob_top -= len;
    memcpy(a->base + a->blob_top, path, len);

    e.path_off = (uint32_t)a->blob_top;
    e.ep_id    = ep_id;
    e.type     = type;
    e.access   = access;
    /* 用 memcpy 写条目：arena 由用户提供，不保证 4 字节对齐 */
    memcpy(a->base + (size_t)a->entry_count * sizeof e, &e, sizeof e);

    a->entry_count++;
    a->blob_used += len;
    return JSDK_OK;
}

static int entry_get(const jsdk_ep_store_t *s, unsigned index, jsdk_ep_entry_t *out)
{
    if (!s || index >= s->arena.entry_count) return JSDK_ERR_NOT_FOUND;
    memcpy(out, s->arena.base + (size_t)index * sizeof *out, sizeof *out);
    return JSDK_OK;
}

unsigned jsdk_ep_store_count(const jsdk_ep_store_t *s)
{
    return s ? s->arena.entry_count : 0u;
}

const char *jsdk_ep_store_path(const jsdk_ep_store_t *s, unsigned index)
{
    jsdk_ep_entry_t e;
    if (entry_get(s, index, &e) != JSDK_OK) return NULL;
    return (const char *)(s->arena.base + e.path_off);
}

int jsdk_ep_store_at(const jsdk_ep_store_t *s, unsigned index, const char **path,
                     uint16_t *ep_id, jsdk_ep_type_t *type, uint8_t *access)
{
    jsdk_ep_entry_t e;
    int rc = entry_get(s, index, &e);
    if (rc != JSDK_OK) return rc;
    if (path)   *path   = (const char *)(s->arena.base + e.path_off);
    if (ep_id)  *ep_id  = e.ep_id;
    if (type)   *type   = (jsdk_ep_type_t)e.type;
    if (access) *access = e.access;
    return JSDK_OK;
}

int jsdk_ep_store_lookup(const jsdk_ep_store_t *s, const char *path,
                         uint16_t *ep_id, jsdk_ep_type_t *type, uint8_t *access)
{
    unsigned i, n;
    if (!s || !path) return JSDK_ERR_INVALID_ARG;

    /* 线性查找：RETAIN_ALL 下最多 594 条，配置期使用足够。
       若将来成为热点，可换成按 path 哈希的小索引（不需要额外内存分配）。 */
    n = s->arena.entry_count;
    for (i = 0; i < n; ++i) {
        jsdk_ep_entry_t e;
        const char *cand;
        memcpy(&e, s->arena.base + (size_t)i * sizeof e, sizeof e);
        cand = (const char *)(s->arena.base + e.path_off);
        if (strcmp(cand, path) == 0) {
            if (ep_id)  *ep_id  = e.ep_id;
            if (type)   *type   = (jsdk_ep_type_t)e.type;
            if (access) *access = e.access;
            return JSDK_OK;
        }
    }
    return JSDK_ERR_NOT_FOUND;
}

/* ==========================================================================
 * 过滤器
 * ======================================================================== */

static unsigned popcount64(uint64_t v)
{
    unsigned n = 0;
    while (v) { v &= (v - 1u); n++; }
    return n;
}

unsigned jsdk_jsondesc_filter_hits(const jsdk_jsondesc_t *p)
{
    return p ? popcount64(p->filter.hit) : 0u;
}

int jsdk_jsondesc_satisfied(const jsdk_jsondesc_t *p)
{
    if (!p || p->filter.count == 0) return 0;
    return jsdk_jsondesc_filter_hits(p) >= p->filter.count;
}

/** 命中则返回 1，并记录到 hit 位图（供 stop_when_satisfied 使用）。 */
static int filter_match(jsdk_jsondesc_t *p, const char *path)
{
    unsigned i;
    size_t plen = strlen(path);
    int keep = 0;

    for (i = 0; i < p->filter.count; ++i) {
        const char *f = p->filter.paths[i];
        size_t flen;
        int hit = 0;

        if (!f) continue;
        flen = strlen(f);
        if (flen == 1u && f[0] == '*') {
            hit = 1;                                    /* 全保留 */
        } else if (flen > 0u && f[flen - 1u] == '*') {
            /* 前缀匹配："axis0.controller.config.mit_max_*" */
            size_t pl = flen - 1u;
            hit = (pl == 0u) ? 1 : ((plen > pl) && (memcmp(path, f, pl) == 0));
        } else if (flen > 0u && f[flen - 1u] == '.') {
            /* 段前缀：路径必须更长且以该段前缀开头 */
            hit = (plen > flen) && (memcmp(path, f, flen) == 0);
        } else if (flen == 0u) {
            hit = (plen == 0u);
        } else {
            hit = (plen == flen) && (memcmp(path, f, flen) == 0);
        }

        if (hit) {
            keep = 1;
            p->filter.hit |= (uint64_t)1u << i;
        }
    }
    return keep;
}

/* ==========================================================================
 * 解析器
 * ======================================================================== */

static jsdk_json_frame_t *top(jsdk_jsondesc_t *p) { return &p->f[p->depth]; }

static int push_frame(jsdk_jsondesc_t *p, int is_array)
{
    jsdk_json_frame_t *f;
    if (p->depth + 1u > JSDK_JSON_MAX_DEPTH)
        return jfail(p, "nesting deeper than JSDK_JSON_MAX_DEPTH");
    f = &p->f[++p->depth];
    memset(f, 0, sizeof *f);
    f->is_array    = (uint8_t)is_array;
    f->restore_len = p->cur_base_len;
    /* 新帧内的第一个字符串：对象内是键，数组内是值 */
    p->expect_key  = (uint8_t)(is_array ? 0u : 1u);
    return JSDK_OK;
}

/** 发出一个叶子端点。path 即 cur_base。 */
static int emit_leaf(jsdk_jsondesc_t *p, uint8_t type, uint16_t id, uint8_t access)
{
    size_t len = (size_t)p->cur_base_len;
    int keep;

    p->store->parsed_total++;
    if (p->store->parsed_total > p->store->max_endpoints)
        return jfail(p, "endpoint count exceeds max_endpoints");

    keep = p->retain_all ? 1 : filter_match(p, p->cur_base);
    if (!keep) return JSDK_OK;

    if (len + 1u > (size_t)p->max_path_len)
        return jfail(p, "path longer than max_path_len");

    /* 长度不足 → NO_MEMORY（由调用者扩容 arena 后重试，会整体重解析） */
    if (jsdk_ep_arena_put(&p->store->arena, p->cur_base, len + 1u, id, type, access)
            != JSDK_OK)
        return jfail_mem(p, "arena exhausted");

    return JSDK_OK;
}

static int pop_object(jsdk_jsondesc_t *p)
{
    jsdk_json_frame_t *f = top(p);

    if (f->have_id && f->have_type) {
        int rc;
        if (!f->have_name)
            return jfail(p, "object has id but no name");
        rc = emit_leaf(p, f->type, (uint16_t)f->id, f->access);
        if (rc != JSDK_OK) return rc;      /* ⚠ 不能一律转成 PARSE，会吞掉 NO_MEMORY */
    }

    p->cur_base_len = f->restore_len;
    p->cur_base[p->cur_base_len] = '\0';
    p->depth--;
    return JSDK_OK;
}

static int pop_array(jsdk_jsondesc_t *p)
{
    jsdk_json_frame_t *f = top(p);
    if (!f->is_array) return jfail(p, "closing ']' without matching '['");

    p->cur_base_len = f->restore_len;
    p->cur_base[p->cur_base_len] = '\0';
    p->depth--;

    if (p->depth == 0) p->state = JSDK_JS_DONE;
    return JSDK_OK;
}

/* ---------- 字段终结 ---------- */

static int finish_key(jsdk_jsondesc_t *p)
{
    jsdk_json_frame_t *f = top(p);

    p->key[p->key_len] = '\0';
    if (!p->key_overflow) {
        if      (p->key_len == 4u && memcmp(p->key, "name",   4) == 0) f->cur_key = JSDK_KEY_NAME;
        else if (p->key_len == 2u && memcmp(p->key, "id",     2) == 0) f->cur_key = JSDK_KEY_ID;
        else if (p->key_len == 4u && memcmp(p->key, "type",   4) == 0) f->cur_key = JSDK_KEY_TYPE;
        else if (p->key_len == 6u && memcmp(p->key, "access", 6) == 0) f->cur_key = JSDK_KEY_ACCESS;
        else                                                          f->cur_key = JSDK_KEY_OTHER;
    } else {
        f->cur_key = JSDK_KEY_OTHER;
    }
    p->state = JSDK_JS_AFTER_KEY;
    return JSDK_OK;
}

static int finish_number(jsdk_jsondesc_t *p)
{
    jsdk_json_frame_t *f = top(p);

    if (f->cur_key == JSDK_KEY_ID) {
        if (p->neg || p->num_digits == 0u) return jfail(p, "invalid id");
        if (p->num > 0xFFFFu)              return jfail(p, "id exceeds 65535");
        f->id      = p->num;
        f->have_id = 1u;
    }
    /* 其它键的数值一律忽略 */
    p->state = JSDK_JS_AFTER_VAL;
    return JSDK_OK;
}

static int finish_literal(jsdk_jsondesc_t *p)
{
    p->val[p->val_len] = '\0';
    if (!(strcmp(p->val, "true") == 0 || strcmp(p->val, "false") == 0 ||
          strcmp(p->val, "null") == 0))
        return jfail(p, "invalid literal");
    p->state = JSDK_JS_AFTER_VAL;
    return JSDK_OK;
}

static int finish_string_value(jsdk_jsondesc_t *p)
{
    jsdk_json_frame_t *f = top(p);

    p->val[p->val_len] = '\0';

    switch (f->cur_key) {
    case JSDK_KEY_TYPE: {
        uint8_t t = jsdk_ep_type_from_name(p->val, p->val_len);
        if (t == 0) return jfail(p, "unknown endpoint type in descriptor");
        f->type      = t;
        f->have_type = 1u;
        break;
    }
    case JSDK_KEY_ACCESS: {
        uint8_t acc = 0, i;
        for (i = 0; i < p->val_len; ++i) {
            if      (p->val[i] == 'r') acc |= JSDK_EP_ACCESS_R;
            else if (p->val[i] == 'w') acc |= JSDK_EP_ACCESS_W;
            else return jfail(p, "invalid access value");
        }
        f->access      = acc;
        f->have_access = 1u;
        break;
    }
    default:
        /* name 已增量写入 cur_base；其余键忽略 */
        break;
    }

    p->state = JSDK_JS_AFTER_VAL;
    return JSDK_OK;
}

static int finish_string_key(jsdk_jsondesc_t *p)
{
    (void)p;
    return finish_key(p);
}

/* ---------- 字符串字符处理 ---------- */

static int string_char(jsdk_jsondesc_t *p, uint8_t c)
{
    if (p->in_key) {
        if (p->key_len + 1u < JSDK_EP_KEY_MAX) {
            p->key[p->key_len++] = (char)c;
        } else {
            p->key_overflow = 1u;   /* 键名过长：按“其它键”处理，不失败 */
        }
        return JSDK_OK;
    }

    {
        jsdk_json_frame_t *f = top(p);

        if (f->cur_key == JSDK_KEY_NAME) {
            if (!f->name_appended) {
                f->have_name = 1u;
                if (p->cur_base_len > 0u) {
                    if (base_append(p, '.') != JSDK_OK) return JSDK_ERR_PARSE;
                }
                f->name_appended = 1u;
            }
            return base_append(p, (char)c);
        }

        if (f->cur_key == JSDK_KEY_TYPE || f->cur_key == JSDK_KEY_ACCESS) {
            if (p->val_len + 1u >= JSDK_EP_VAL_MAX)
                return jfail(p, "type/access value too long");
            p->val[p->val_len++] = (char)c;
        }
        /* 其余键的字符串值：跳过 */
        return JSDK_OK;
    }
}

/** 处理字符串内的转义；返回 1 表示该字符已被消费，0 表示需按普通字符处理。 */
static int escape_char(jsdk_jsondesc_t *p, uint8_t c, uint8_t *out)
{
    if (!p->esc) {
        if (c != '\\') return 0;
        p->esc = 1u;
        return 1;
    }
    p->esc = 0u;
    switch (c) {
    case '"':  *out = '"';  break;
    case '\\': *out = '\\'; break;
    case '/':  *out = '/';  break;
    case 'b':  *out = '\b'; break;
    case 'f':  *out = '\f'; break;
    case 'n':  *out = '\n'; break;
    case 'r':  *out = '\r'; break;
    case 't':  *out = '\t'; break;
    default:
        /* \uXXXX 等：本描述符不含，直接失败以免静默产生错误路径 */
        jfail(p, "unsupported escape sequence in descriptor");
        return 1;
    }
    return 1;
}

/* ---------- 主循环 ---------- */

int jsdk_jsondesc_feed(jsdk_jsondesc_t *p, const void *data, size_t len)
{
    const uint8_t *d = (const uint8_t *)data;
    size_t i = 0;

    if (!p || (!d && len)) return JSDK_ERR_INVALID_ARG;
    if (p->state == JSDK_JS_FAIL) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */

    while (i < len) {
        uint8_t c = d[i];

        p->bytes_fed++;

        if (p->state == JSDK_JS_FAIL) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */

        switch (p->state) {

        case JSDK_JS_BEGIN:
            if (is_ws(c)) { i++; break; }
            if (c != '[') return jfail(p, "root must be an array");
            if (push_frame(p, 1) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
            p->state = JSDK_JS_EXPECT;
            i++;
            break;

        case JSDK_JS_EXPECT: {
            jsdk_json_frame_t *f = top(p);
            if (is_ws(c)) { i++; break; }

            if (c == '}') {
                if (f->is_array) return jfail(p, "'}' inside array");
                if (pop_object(p) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
                p->state = JSDK_JS_AFTER_VAL;
                i++;
                break;
            }
            if (c == ']') {
                if (!f->is_array) return jfail(p, "']' inside object");
                if (pop_array(p) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
                if (p->state != JSDK_JS_DONE) p->state = JSDK_JS_AFTER_VAL;
                i++;
                break;
            }
            if (c == '{') {
                if (push_frame(p, 0) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
                i++;
                break;
            }
            if (c == '[') {
                if (push_frame(p, 1) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
                i++;
                break;
            }
            if (c == '"') {
                p->in_key        = p->expect_key;
                p->key_len       = 0u;
                p->key_overflow  = 0u;
                p->val_len       = 0u;
                p->esc           = 0u;
                /* name 字段：在这里就标记“见过”，否则空名字的节点（如根节点
                   {"name":"","id":0,"type":"json"}）会被误判为缺 name */
                if (!p->in_key && f->cur_key == JSDK_KEY_NAME) {
                    f->have_name     = 1u;
                    f->name_appended = 0u;
                }
                p->state = JSDK_JS_IN_STR;
                i++;
                break;
            }
            if (c == '-' || is_digit(c)) {
                p->neg        = (uint8_t)(c == '-');
                p->num        = is_digit(c) ? (uint32_t)(c - '0') : 0u;
                p->num_digits = is_digit(c) ? 1u : 0u;
                p->state      = JSDK_JS_IN_NUM;
                i++;
                break;
            }
            if (is_alpha(c)) {
                p->val_len = 0u;
                p->state   = JSDK_JS_IN_LIT;
                break;              /* 不消费：交给 IN_LIT */
            }
            return jfail(p, "unexpected character where key/value expected");
        }

        case JSDK_JS_IN_KEY:
            /* 不会进入：键与字符串值共用 IN_STR */
            return jfail(p, "internal state error");

        case JSDK_JS_IN_STR: {
            uint8_t out = 0;
            if (c == '"' && !p->esc) {
                i++;
                if (p->in_key) {
                    p->in_key = 0u;
                    if (finish_string_key(p) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
                } else {
                    if (finish_string_value(p) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
                    p->state = JSDK_JS_AFTER_VAL;
                }
                break;
            }
            if (escape_char(p, c, &out)) {
                i++;
                if (p->state == JSDK_JS_FAIL) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
                if (p->esc) break;      /* 刚吃掉反斜杠 */
                if (string_char(p, out) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
                break;
            }
            if (string_char(p, c) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
            i++;
            break;
        }

        case JSDK_JS_AFTER_KEY:
            if (is_ws(c)) { i++; break; }
            if (c != ':') return jfail(p, "expected ':' after key");
            p->expect_key = 0u;          /* ':' 之后是值 */
            p->state = JSDK_JS_EXPECT;
            i++;
            break;

        case JSDK_JS_IN_NUM:
            if (is_digit(c)) {
                if (p->num_digits < 9u) {
                    p->num = p->num * 10u + (uint32_t)(c - '0');
                    p->num_digits++;
                } else {
                    p->num = 0xFFFFFFFFu;   /* 溢出：后续校验会拒绝 */
                }
                i++;
                break;
            }
            if (c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-') {
                /* 浮点/指数形式：本描述符的 id 不会是浮点 → 交给校验拒绝 */
                if (top(p)->cur_key == JSDK_KEY_ID) return jfail(p, "id must be an integer");
                p->num_digits = 0u;         /* 标记为非整数，忽略其值 */
                p->num = 0u;
                i++;
                break;
            }
            if (finish_number(p) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
            break;                          /* 不消费终止符 */

        case JSDK_JS_IN_LIT:
            if (is_alpha(c) && p->val_len + 1u < JSDK_EP_VAL_MAX) {
                p->val[p->val_len++] = (char)c;
                i++;
                if (p->val_len == 5u) {     /* "false" 已完整 */
                    if (finish_literal(p) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
                }
                break;
            }
            if (finish_literal(p) != JSDK_OK) return p->fail_code;   /* 粘性：可能是 NO_MEMORY */
            break;                          /* 不消费终止符 */

        case JSDK_JS_AFTER_VAL:
            if (is_ws(c)) { i++; break; }
            if (c == ',') {
                p->expect_key = (uint8_t)(top(p)->is_array ? 0u : 1u);
                p->state = JSDK_JS_EXPECT;
                i++;
                break;
            }
            if (c == '}' || c == ']') { p->state = JSDK_JS_EXPECT; break; }  /* 不消费 */
            return jfail(p, "expected ',' or closing bracket");

        case JSDK_JS_DONE:
            if (is_ws(c)) { i++; break; }
            return jfail(p, "trailing data after root array");

        default:
            return jfail(p, "internal state error");
        }
    }

    return (p->state == JSDK_JS_FAIL) ? p->fail_code : JSDK_OK;
}

int jsdk_jsondesc_init(jsdk_jsondesc_t *p, const jsdk_desc_config_t *cfg,
                       jsdk_ep_store_t *store)
{
    if (!p || !cfg || !store)                        return JSDK_ERR_INVALID_ARG;
    if (!store->arena.base || store->arena.size == 0) return JSDK_ERR_INVALID_ARG;
    if (cfg->filter_count > JSDK_DESC_MAX_FILTERS)    return JSDK_ERR_INVALID_ARG;
    if (!cfg->filter_paths && cfg->filter_count)      return JSDK_ERR_INVALID_ARG;

    memset(p, 0, sizeof *p);
    p->cfg        = cfg;
    p->store      = store;
    p->retain_all = (uint8_t)(cfg->retain == JSDK_DESC_RETAIN_ALL);
    p->max_path_len = (uint16_t)(cfg->max_path_len ? cfg->max_path_len : 128u);
    if (p->max_path_len > JSDK_EP_PATH_MAX_HARD)      return JSDK_ERR_INVALID_ARG;

    if (!p->retain_all) {
        if (cfg->filter_count == 0u) return JSDK_ERR_INVALID_ARG;
        p->filter.paths = cfg->filter_paths;
        p->filter.count = cfg->filter_count;
    } else if (cfg->filter_paths && cfg->filter_count) {
        /* RETAIN_ALL 下仍记录 filter，用于 satisfied 统计；不影响保留策略 */
        p->filter.paths = cfg->filter_paths;
        p->filter.count = cfg->filter_count;
    }

    jsdk_ep_store_reset(store);
    store->max_endpoints = cfg->max_endpoints ? cfg->max_endpoints : 2048u;

    p->fail_code = JSDK_ERR_PARSE;   /* 默认码；jfail_mem 会覆盖为 NO_MEMORY */
    p->state = JSDK_JS_BEGIN;
    p->depth = 0;
    p->cur_base[0] = '\0';
    return JSDK_OK;
}

int jsdk_jsondesc_finish(jsdk_jsondesc_t *p)
{
    if (!p) return JSDK_ERR_INVALID_ARG;
    /* 粘性失败码：若前面已因 arena 不足失败，这里必须原样返回 NO_MEMORY，
       否则 run()（init+feed+finish）会把真实原因掩盖成 PARSE。 */
    if (p->state == JSDK_JS_FAIL) return p->fail_code;
    if (p->state != JSDK_JS_DONE) return jfail(p, "truncated descriptor (no closing ']')");
    if (p->depth != 0u)           return jfail(p, "unbalanced brackets");
    return JSDK_OK;
}

int jsdk_jsondesc_run(const jsdk_desc_config_t *cfg, jsdk_ep_store_t *store,
                      const void *json, size_t len)
{
    jsdk_jsondesc_t p;
    int rc = jsdk_jsondesc_init(&p, cfg, store);
    if (rc != JSDK_OK) return rc;
    rc = jsdk_jsondesc_feed(&p, json, len);
    if (rc != JSDK_OK) return rc;
    return jsdk_jsondesc_finish(&p);
}

const char *jsdk_jsondesc_error(const jsdk_jsondesc_t *p)
{
    return p ? p->err : NULL;
}

/* ==========================================================================
 * 公开 API（§18）：arena 尺寸估算
 * ======================================================================== */

size_t jsdk_desc_arena_size(const jsdk_desc_config_t *cfg)
{
    size_t need = 0;
    unsigned i;

    if (!cfg) return 0;

    if (cfg->retain == JSDK_DESC_RETAIN_FILTERED && cfg->filter_paths) {
        for (i = 0; i < cfg->filter_count; ++i) {
            const char *f = cfg->filter_paths[i];
            if (!f) continue;
            need += sizeof(jsdk_ep_entry_t) + strlen(f) + 1u;
        }
        if (need == 0) return 0;
        /* 估算：精确匹配时即为准确值；前缀匹配的实际路径更长，故加 25% + 128 余量 */
        return need + need / 4u + 128u;
    }

    /* RETAIN_ALL：端点数未知，返回保守推荐值 */
    return JSDK_DESC_ARENA_RECOMMEND_ALL;
}
