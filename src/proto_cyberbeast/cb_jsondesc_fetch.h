/**
 * @file    cb_jsondesc_fetch.h
 * @brief   JSON 端点描述符下载（0x24 请求 / 0x25 数据流）—— 传输状态机
 *
 * 本模块负责“把 41 KB 的描述符从设备搬到解析器里”，**不含任何上下文/关节逻辑**：
 * 它只认识 `(0x25 载荷) → (解析器 + raw sink + 进度回调)` 这条链路，
 * 因此可以在虚拟 HAL 上独立做端到端回归。
 *
 * @par 协议时序（已对固件 `send_json_desc_chunk()` 逐行核实）
 *  - 主站发 **一帧** `0x24`，载荷 = `[offset u32 LE]`（4 B）即触发**自主流式发送**；
 *    不需要逐块请求。
 *  - 设备回 **`0x25`**，每帧长度**恒定**（Classic 8 B / FD 64 B，末尾补 0）：
 *      · 元数据帧（整次传输的**第一帧**）：`[0..1]=00 00`、`[2..5]=totalLen u32 LE`、
 *        `[6..7]=crc u16 LE`
 *      · 数据帧：`[0..1]=chunkOffset u16 LE`、`[2..]=JSON 文本`
 *  - 设备每毫秒 ≤ `kMaxJsonFramesPerCycle = 50` 帧，发完后自动清理状态。
 *
 * @par ⚠ 帧分类必须“有状态”，不能逐帧瞎猜
 *  `chunkOffset` 与元数据帧的前两字节都是 `00 00` 时无法靠前两字节区分。
 *  真正可靠的做法是：**第一帧必为元数据帧**（设备就是这么实现的），
 *  之后全部是数据帧；`buf[2] == '{'` 只用来**交叉校验**第一帧——
 *  若“元数据帧”的第 3 字节竟然是 JSON 起始符，说明设备跳过了元数据，
 *  此时无法得知 total_len，必须**整体失败**（绝不接受部分结果）。
 *
 * @par 两个必须的硬约束
 *  1. **`total_len > 65535` 一律拒绝。** `chunkOffset` 只有 u16，超出会静默回绕。
 *  2. **`stop_when_satisfied` 与 raw sink 互斥。** 提前终止会让原始字节不完整，
 *     缓存下来下次解析必然失败。用 `cb_desc_fetch_cache_safe()` 做后置校验。
 *
 * @par ⚠ 提前终止只对**全部为精确路径**的 filter 生效（重要）
 *  前缀/通配 filter（`mit_max_*`）会被它的**第一个**匹配项“满足”——
 *  于是 `satisfied` 为真、扫描立即停止，同前缀的其余路径（`mit_max_torque`…）
 *  就**静默丢失**，而且丢失哪些取决于 JSON 的字段顺序。
 *  这是无法在流式扫描下可靠判定的（除非下完整份描述符），
 *  因此本模块的规则是：
 *
 *  > **只要 filter 列表里存在任何通配/前缀（`*` 结尾）或段前缀（`.` 结尾）条目，
 *  > 就禁用提前终止**，宁可多下几百帧，也不交付一个依赖字段顺序的残缺端点表。
 *
 *  想把 arena 压到最小，请**显式列出全部需要的精确路径**（如逐条写出 5 个
 *  `mit_max_*`），此时提前终止会正常生效（实测省 40% 帧数）。
 *  实际是否启用过提前终止由 `cb_desc_fetch_result_t.stop_allowed` 告知。
 */

#ifndef CB_JSONDESC_FETCH_H
#define CB_JSONDESC_FETCH_H

#include <stddef.h>
#include <stdint.h>

#include "jsdk_internal.h"   /* jsdk_jsondesc_t / jsdk_ep_store_t */

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * 常量
 * ------------------------------------------------------------------------ */

#define CB_DESC_REQ_LEN        4u      /**< 0x24 请求载荷长度：offset u32 LE */
#define CB_DESC_META_MIN_LEN   8u      /**< 元数据帧至少需要 8 字节 */
#define CB_DESC_META_HDR_BYTES 8u      /**< 元数据帧的有效字段长度 */
#define CB_DESC_DATA_HDR_BYTES 2u      /**< 数据帧头：chunkOffset u16 LE */

/**
 * `total_len` 上限。
 * ⚠ 不是随意取的：`chunkOffset` 是 **u16**，固件在 64 KB 处会静默回绕，
 *   因此 SDK 侧必须在 65535 处硬拒绝（见 PROTOCOL_NOTES §5.6）。
 */
#define CB_DESC_MAX_TOTAL_LEN  65535u

/** 合法帧长（固件恒发整帧：Classic 8 / FD 64） */
#define CB_DESC_FRAME_LEN_CLASSIC 8u
#define CB_DESC_FRAME_LEN_FD      64u

/** JSON 文本的第一个字节（用于第一帧的交叉校验） */
#define CB_DESC_JSON_FIRST_BYTE  0x7Bu   /* '{' */

/* --------------------------------------------------------------------------
 * 失败原因（静态字符串，便于日志与测试断言）
 * ------------------------------------------------------------------------ */

#define CB_DESC_ERR_NONE          ((const char *)0)
#define CB_DESC_ERR_FRAME_LEN     "0x25 frame length must be 8 (Classic) or 64 (FD)"
#define CB_DESC_ERR_META_EXPECTED "first 0x25 frame must be the metadata frame"
#define CB_DESC_ERR_META_TOTAL    "metadata total_len must be 1..65535"
#define CB_DESC_ERR_OFFSET        "chunkOffset out of sequence"
#define CB_DESC_ERR_PARSE         "JSON parser rejected the descriptor"
#define CB_DESC_ERR_ARENA         "endpoint arena exhausted"

/* 注：截断不会产生自己的错误码——它表现为 `cb_desc_fetch_is_done() == 0`
   （调用方靠自己给的超时终止）。raw sink 报错也不使下载失败，
   而是置 `raw_sink_failed` 并继续，因此同样没有独立的错误码。 */

/* --------------------------------------------------------------------------
 * 状态机
 * ------------------------------------------------------------------------ */

typedef struct {
    /* --- 注入的依赖（均不拥有） --- */
    jsdk_context_t        *ctx;        /**< 不透明透传给回调，可 NULL */
    const jsdk_desc_config_t *cfg;     /**< 提供 retain / filter / 上限 */
    jsdk_ep_store_t       *store;      /**< arena + 计数 */

    jsdk_desc_raw_sink_fn  sink;       /**< 可 NULL */
    void                  *sink_user;
    jsdk_desc_progress_fn  progress;   /**< 可 NULL */
    void                  *progress_user;

    /* --- 解析器 --- */
    jsdk_jsondesc_t parser;

    /* --- 传输状态 --- */
    uint32_t total_len;      /**< 来自元数据帧 */
    uint16_t crc;            /**< 来自元数据帧（VersionCRC） */
    uint32_t next_offset;    /**< 期望的下一个 chunkOffset */
    uint32_t bytes_scanned;  /**< 已喂给解析器的字节数 */
    uint32_t frames_rx;      /**< 收到的 0x25 帧数（含元数据帧） */
    uint32_t last_report;    /**< 上次回调进度时的字节数（节流用） */

    uint8_t  started;        /**< 已经收到过元数据帧 */
    uint8_t  done;           /**< 传输已结束（成功或失败） */
    uint8_t  failed;         /**< 1 = 失败 */
    uint8_t  complete;       /**< 1 = 收满 total_len 且解析器正常收尾 */
    uint8_t  stopped_early;  /**< 1 = filter 满足后主动终止（数据不完整） */
    uint8_t  stop_allowed;   /**< 1 = 本次配置允许提前终止（全部 filter 均为精确路径） */
    uint8_t  raw_sink_failed;/**< 1 = sink 报错，已停止回调 */
    uint8_t  sink_called;    /**< 至少成功调用过 sink 一次 */
    uint8_t  arena_full;     /**< 曾因 arena 不足被拒 */

    int      fail_rc;        /**< 首次失败返回的错误码（重复调用返回同一值） */
    const char *err;         /**< 失败原因（CB_DESC_ERR_*） */
    const char *err_detail;  /**< 解析器的细粒度原因（可 NULL） */
} cb_desc_fetch_t;

/** 传输结束后的汇总（供 desc_info / 缓存决策使用） */
typedef struct {
    uint32_t total_len;
    uint16_t crc;
    uint32_t bytes_scanned;
    unsigned frames_rx;
    unsigned endpoint_count;   /**< 已入 arena 的端点数 */
    unsigned parsed_total;     /**< 实际解析到的叶子总数（含未保留） */
    size_t   arena_used;       /**< arena 实际占用字节数 */
    uint8_t  complete;
    uint8_t  stopped_early;
    uint8_t  stop_allowed;     /**< 本次是否允许提前终止（见文件头说明） */
    uint8_t  raw_sink_failed;
    uint8_t  failed;
} cb_desc_fetch_result_t;

/* --------------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------------ */

/**
 * 初始化一次下载。
 * @param cfg   描述符配置（必需；提供 retain / filter_paths / max_* / arena）
 * @param store 端点存储（必需；其 arena 必须是调用者提供的有效内存）
 * @return JSDK_OK / JSDK_ERR_INVALID_ARG
 */
int cb_desc_fetch_init(cb_desc_fetch_t *f, jsdk_context_t *ctx,
                       const jsdk_desc_config_t *cfg, jsdk_ep_store_t *store);

/** 安装回调（可随时调用；sink = NULL 表示不 tee）。 */
void cb_desc_fetch_set_raw_sink(cb_desc_fetch_t *f, jsdk_desc_raw_sink_fn fn,
                                void *user);
void cb_desc_fetch_set_progress(cb_desc_fetch_t *f, jsdk_desc_progress_fn fn,
                                void *user);

/**
 * 构造 0x24 请求载荷。
 * @param offset 起始字节偏移（断点续传用；正常为 0）
 * @return 写入字节数（恒为 4）；cap 不足返回 0
 */
size_t cb_desc_build_request(uint8_t *dst, size_t cap, uint32_t offset);

/**
 * 喂入一帧 `0x25` 载荷。
 *
 * 必须在**同一线程**内按到达顺序调用；帧可能是元数据帧也可能是数据帧，
 * 由内部状态判定（见文件头说明）。
 *
 * @return JSDK_OK 已接受；JSDK_ERR_PARSE / JSDK_ERR_NO_MEMORY / JSDK_ERR_PROTOCOL
 */
int cb_desc_fetch_frame(cb_desc_fetch_t *f, const uint8_t *payload, size_t len);

/** 传输是否已结束（成功或失败）。 */
int cb_desc_fetch_is_done(const cb_desc_fetch_t *f);

/**
 * 传输是否**可用**：未失败，且（完整收到 或 按 filter 提前终止且 filter 全命中）。
 */
int cb_desc_fetch_is_ok(const cb_desc_fetch_t *f);

/**
 * 原始字节是否可信（三条后置校验同时成立）：
 *   ① `complete == 1`（没被提前终止）
 *   ② `raw_sink_failed == 0`
 *   ③ `bytes_scanned == total_len`
 * 只有本函数返回 1 时才允许把 raw sink 收到的字节写进 Flash 当缓存。
 */
int cb_desc_fetch_cache_safe(const cb_desc_fetch_t *f);

const char *cb_desc_fetch_error(const cb_desc_fetch_t *f);

/**
 * 失败时解析器给出的**更细**原因（如 "path longer than max_path_len"），
 * 无更细原因时返回 NULL。仅用于日志，**不要**用它做判定。
 */
const char *cb_desc_fetch_detail(const cb_desc_fetch_t *f);

void cb_desc_fetch_result(const cb_desc_fetch_t *f, cb_desc_fetch_result_t *out);

/** 已扫描比例（0..100）；尚未收到元数据帧时返回 0。 */
unsigned cb_desc_fetch_pct(const cb_desc_fetch_t *f);

/** 期望的帧长（按 cfg 的 is_fd 推断不了，由调用方传；此处仅做常量导出）。 */
int cb_desc_frame_len_valid(size_t len);

/** 生效的下载超时（ms）：`cfg->timeout_ms` 为 0 时取 5000。 */
uint32_t cb_desc_timeout_ms(const jsdk_desc_config_t *cfg);

/**
 * filter 列表是否**全部为精确路径**（无 `*` / `.` 结尾的通配）。
 * 只有为真时才允许提前终止，理由见文件头。filter_count = 0 时返回 1。
 */
int cb_desc_filters_all_exact(const jsdk_desc_config_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* CB_JSONDESC_FETCH_H */
