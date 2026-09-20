/**
 * @file    jsdk_internal.h
 * @brief   Joint SDK 内部接口（**不安装、不对外**）
 *
 * 本头文件只被 src/ 下的实现与 tests/ 使用。对外 ABI 见 joint_sdk.h。
 */

#ifndef JSDK_INTERNAL_H
#define JSDK_INTERNAL_H

#include <stdint.h>
#include <stddef.h>
#include "joint_sdk/joint_sdk.h"

/* ==========================================================================
 * 编译期上限
 * ======================================================================== */

/**
 * JSON 括号嵌套深度上限。
 * 实测（v8 描述符）最大嵌套 = **10**：最深端点为
 *   axis0.motor.motor_thermistor.config.temp_limit_upper
 * 每层 = 1 个 object + 1 个 members 数组，故帧深度 = 1(根数组) + 2×4 = 9。
 * 取 16 留出余量（固件新增一层嵌套仍可解析）。
 */
#define JSDK_JSON_MAX_DEPTH      16u

/** filter_paths 条数上限（受 filter_hit 位图宽度限制，见 jsdk_desc_config_t）。 */
#define JSDK_DESC_MAX_FILTERS    64u

/** 单条路径长度硬上限（实测最长 68，默认 max_path_len = 128）。 */
#define JSDK_EP_PATH_MAX_HARD    160u

/** JSON 键名缓冲上限（最长键为 "endpoint_ref" / "members"）。 */
#define JSDK_EP_KEY_MAX          16u

/** type / access 字段值缓冲上限（最长 "endpoint_ref" = 12）。 */
#define JSDK_EP_VAL_MAX          16u

/** RETAIN_ALL 下的保守推荐 arena 大小（无法预知端点数，见 jsdk_desc_arena_size）。 */
#define JSDK_DESC_ARENA_RECOMMEND_ALL   32768u

/* ==========================================================================
 * 端点存储（arena）
 * ======================================================================== */

/** arena 条目：8 字节。条目区从 arena 头部向后增长，路径池从尾部向前增长。 */
typedef struct {
    uint32_t path_off;   /**< 相对 arena 基址的偏移，指向 NUL 结尾的路径 */
    uint16_t ep_id;
    uint8_t  type;       /**< jsdk_ep_type_t */
    uint8_t  access;     /**< JSDK_EP_ACCESS_* 位掩码 */
} jsdk_ep_entry_t;

typedef struct {
    uint8_t *base;         /**< 调用者提供的 arena 基址 */
    size_t   size;         /**< arena 字节数 */
    uint32_t entry_count;  /**< 已写入的条目数 */
    size_t   blob_top;     /**< 路径池当前顶部（向下增长） */
    size_t   blob_used;    /**< 路径池已用字节 */
} jsdk_arena_t;

typedef struct {
    jsdk_arena_t arena;
    unsigned     parsed_total;   /**< 解析到的叶子总数（含未保留） */
    unsigned     max_endpoints;
} jsdk_ep_store_t;

/* ==========================================================================
 * 过滤器
 * ======================================================================== */

typedef struct {
    const char *const *paths;
    unsigned           count;
    uint64_t           hit;      /**< 第 i 条是否已命中（count ≤ 64） */
} jsdk_desc_filter_t;

/* ==========================================================================
 * 增量 JSON 解析器
 * ------------------------------------------------------------------------
 * 设计要点（见 docs/PROTOCOL_NOTES.zh-CN.md §9.2）：
 *   - 逐字节状态机，无递归、无 malloc，不需要缓存完整 41 KB；
 *   - 路径在解析 name 字段时**增量拼接到 cur_base**，因此不需要按层保存
 *     name 副本（key 与 id/type 的先后顺序无关）；
 *   - 叶子判定：对象闭合时同时见过 id 与 type。
 * ======================================================================== */

typedef struct {
    uint16_t restore_len;  /**< 弹出时把 cur_base 截断回的长度 */
    uint32_t id;
    uint8_t  have_id;
    uint8_t  type;         /**< jsdk_ep_type_t */
    uint8_t  have_type;
    uint8_t  access;
    uint8_t  have_access;
    uint8_t  have_name;    /**< 见过 name 字段（含空串） */
    uint8_t  name_appended;/**< 名字已追加到 cur_base（空名字时为 0） */
    uint8_t  is_array;
    uint8_t  cur_key;      /**< 当前对象的 key（见 JSDK_KEY_*） */
} jsdk_json_frame_t;

enum {
    JSDK_KEY_NONE = 0,
    JSDK_KEY_NAME,
    JSDK_KEY_ID,
    JSDK_KEY_TYPE,
    JSDK_KEY_ACCESS,
    JSDK_KEY_OTHER
};

enum {
    JSDK_JS_BEGIN = 0,
    JSDK_JS_EXPECT,      /**< 期待 key（对象内）或 value（数组内）或闭合符 */
    JSDK_JS_IN_KEY,
    JSDK_JS_AFTER_KEY,
    JSDK_JS_IN_STR,      /**< 字符串值 */
    JSDK_JS_IN_NUM,
    JSDK_JS_IN_LIT,
    JSDK_JS_AFTER_VAL,
    JSDK_JS_DONE,
    JSDK_JS_FAIL
};

typedef struct jsdk_jsondesc {
    /* 配置（不拥有） */
    const jsdk_desc_config_t *cfg;
    jsdk_ep_store_t          *store;

    jsdk_desc_filter_t filter;
    uint32_t           bytes_fed;
    uint16_t           max_path_len;

    /* 状态机 */
    uint8_t  state;
    uint8_t  depth;
    uint8_t  retain_all;
    uint8_t  esc;
    uint8_t  cap;          /**< 当前字符串是否要捕获 */
    uint8_t  in_key;       /**< 当前字符串是键（而非值） */
    uint8_t  expect_key;   /**< 下一字符串是键；由 '{' / ',' 判定，与帧类型无关 */
    uint8_t  key_overflow;
    uint8_t  neg;
    uint32_t num;
    uint8_t  num_digits;

    uint16_t cur_base_len;
    char     cur_base[JSDK_EP_PATH_MAX_HARD];
    uint16_t key_len;
    char     key[JSDK_EP_KEY_MAX];
    uint16_t val_len;
    char     val[JSDK_EP_VAL_MAX];

    jsdk_json_frame_t f[JSDK_JSON_MAX_DEPTH + 1u];

    const char *err;       /**< 失败原因（静态字符串） */
    int         fail_code; /**< 粘性失败码：JSDK_ERR_PARSE 或 JSDK_ERR_NO_MEMORY */
} jsdk_jsondesc_t;

/* ==========================================================================
 * 解析器 API
 * ======================================================================== */

int  jsdk_jsondesc_init(jsdk_jsondesc_t *p, const jsdk_desc_config_t *cfg,
                        jsdk_ep_store_t *store);
int  jsdk_jsondesc_feed(jsdk_jsondesc_t *p, const void *data, size_t len);
int  jsdk_jsondesc_finish(jsdk_jsondesc_t *p);

/** 一次跑完（等价 init + feed + finish），用于测试与 desc_import_raw()。 */
int  jsdk_jsondesc_run(const jsdk_desc_config_t *cfg, jsdk_ep_store_t *store,
                       const void *json, size_t len);

/** 已命中的 filter 条数。 */
unsigned jsdk_jsondesc_filter_hits(const jsdk_jsondesc_t *p);

/** filter 是否已全部命中（用于 stop_when_satisfied）。 */
int jsdk_jsondesc_satisfied(const jsdk_jsondesc_t *p);

/** 最后一次失败原因（可读文本）；未失败返回 NULL。 */
const char *jsdk_jsondesc_error(const jsdk_jsondesc_t *p);

/* ==========================================================================
 * 端点存储 API
 * ======================================================================== */

int          jsdk_ep_arena_put(jsdk_arena_t *a, const char *path, size_t len,
                               uint16_t ep_id, uint8_t type, uint8_t access);
unsigned     jsdk_ep_store_count(const jsdk_ep_store_t *s);
const char  *jsdk_ep_store_path(const jsdk_ep_store_t *s, unsigned index);
int          jsdk_ep_store_at(const jsdk_ep_store_t *s, unsigned index,
                              const char **path, uint16_t *ep_id,
                              jsdk_ep_type_t *type, uint8_t *access);
int          jsdk_ep_store_lookup(const jsdk_ep_store_t *s, const char *path,
                                  uint16_t *ep_id, jsdk_ep_type_t *type,
                                  uint8_t *access);
void         jsdk_ep_store_reset(jsdk_ep_store_t *s);

/** 名称 → 类型枚举；未知返回 0。 */
uint8_t jsdk_ep_type_from_name(const char *name, size_t len);

/** 类型枚举 → 名称（静态字符串）；未知返回 "?"。 */
const char *jsdk_ep_type_name(jsdk_ep_type_t t);

/** 该类型是否可直接读写（JSON / FUNCTION / ENDPOINT_REF 为不透明类型）。 */
int jsdk_ep_type_is_scalar(jsdk_ep_type_t t);

/** 该类型在协议上的字节长度；不透明类型返回 0。 */
unsigned jsdk_ep_type_size(jsdk_ep_type_t t);

#endif /* JSDK_INTERNAL_H */
