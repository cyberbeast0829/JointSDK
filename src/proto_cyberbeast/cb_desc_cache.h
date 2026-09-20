/**
 * @file    cb_desc_cache.h
 * @brief   描述符缓存：已解析结果的导出/导入（PORTING §3 路线 A）
 *
 * 目的：免掉每次上电重下 41 KB / 662 帧。两条路线：
 *
 *   **路线 A（本模块）**：缓存**已解析的端点表**。体积小
 *   （`RETAIN_FILTERED` + 12 条路径约 0.5 KB），但结果被
 *   **retain / filter_paths / SDK 内部格式**三者绑定。
 *   → 导出文件自带这三项的失效键，导入时逐项校验，不匹配就明确报错。
 *
 *   **路线 B**：缓存**原始 JSON**（`jsdk_desc_raw_sink_fn` + `cb_desc_import_raw()`），
 *   只与 `(fw_version, desc_crc)` 绑定，改 filter 无需重下。约 41 KB。
 *
 * @par 格式设计要点
 *  - **不依赖 arena 布局**：导出的是 `(ep_id, type, access, path)` 记录序列，
 *    导入时逐条重新 `jsdk_ep_arena_put()`。因此**导入侧的 arena 可以比导出侧小或大**
 *    —— 只要装得下即可，这对现场缩容很重要。
 *  - **自带完整性校验**：`body_crc` 覆盖全部记录，防 Flash 位翻转。
 *  - **失败不留部分结果**：任一步校验失败都 `jsdk_ep_store_reset()`。
 *  - **字节序固定小端**，与 CPU 无关（缓存要能跨平台读）。
 */

#ifndef CB_DESC_CACHE_H
#define CB_DESC_CACHE_H

#include <stddef.h>
#include <stdint.h>

#include "jsdk_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * 格式常量
 * ------------------------------------------------------------------------ */

/** 魔数：ASCII "JSDK" 的小端表示 */
#define CB_DESC_CACHE_MAGIC    0x4B44534Au
/** 格式版本。**改动线格式时必须 +1**，旧缓存会被判为不兼容而不是误读。 */
#define CB_DESC_CACHE_VERSION  1u
/** 固定头长度 */
#define CB_DESC_CACHE_HDR_LEN  48u
/** 单条记录的头：ep_id u16 + type u8 + access u8 + path_len u16 */
#define CB_DESC_CACHE_REC_HDR  6u

/** `max_endpoints` 的生效值（0 → 2048） */
uint32_t cb_desc_eff_max_endpoints(const jsdk_desc_config_t *cfg);
/** `max_path_len` 的生效值（0 → 128） */
uint32_t cb_desc_eff_max_path_len(const jsdk_desc_config_t *cfg);

/** 头里 `flags` 的位 */
#define CB_DESC_CACHE_F_FILTERED 0x0001u  /**< 1 = RETAIN_FILTERED */
#define CB_DESC_CACHE_F_EARLY    0x0002u  /**< 1 = 下载时被 stop_when_satisfied 提前终止 */

/* --------------------------------------------------------------------------
 * 失效键
 * ------------------------------------------------------------------------ */

/**
 * 计算 `cfg` 的失效键。覆盖：retain 模式、完整 filter 列表（顺序敏感）、
 * `max_endpoints`、`max_path_len`，以及本文件的线格式版本。
 *
 * 用途：① 写进导出头，导入时校验；② 应用也可用它做自己缓存头的键。
 */
uint32_t cb_desc_filter_hash(const jsdk_desc_config_t *cfg);

/* --------------------------------------------------------------------------
 * 导出
 * ------------------------------------------------------------------------ */

/** 导出所需字节数（`header + body`）。store 为空时返回头长度。 */
size_t cb_desc_cache_size(const jsdk_ep_store_t *store);

/**
 * 把已解析的端点表导出到 `buf`。
 *
 * @param cfg 必须与解析时**同一份**（用于写失效键）
 * @param early 下载是否被 `stop_when_satisfied` 提前终止（仅记录，不参与校验）
 * @param out_len 输出实际长度（可为 NULL）
 * @return JSDK_OK / JSDK_ERR_INVALID_ARG
 *         / JSDK_ERR_NO_MEMORY（cap 不足，需扩容后重试）
 */
int cb_desc_cache_export(const jsdk_desc_config_t *cfg, const jsdk_ep_store_t *store,
                         int early, void *buf, size_t cap, size_t *out_len);

/* --------------------------------------------------------------------------
 * 导入
 * ------------------------------------------------------------------------ */

/**
 * 从 `buf` 恢复端点表。
 *
 * 校验顺序：长度 → 魔数 → 版本 → 头长 → retain → filter_hash →
 *           max_endpoints → max_path_len → 逐条记录边界 → body_crc。
 * 全部通过后才开始写 arena；任一步失败都返回错误并**清空 store**。
 *
 * @return JSDK_OK
 *         JSDK_ERR_INVALID_ARG  指针/长度非法
 *         JSDK_ERR_PROTOCOL     魔数/版本/头长/记录越界/CRC 不符
 *         JSDK_ERR_BAD_STATE    失效键不匹配（retain / filter / 上限 变了）
 *         JSDK_ERR_NO_MEMORY    arena 装不下（**无部分结果**）
 */
int cb_desc_cache_import(const jsdk_desc_config_t *cfg, jsdk_ep_store_t *store,
                         const void *buf, size_t len);

/** 从导出缓冲里读出头部的只读信息（用于应用做 (fw_version, crc) 比对）。 */
typedef struct {
    uint16_t desc_crc;      /**< 描述符 VersionCRC */
    uint16_t flags;         /**< CB_DESC_CACHE_F_* */
    uint32_t fw_version;    /**< 导出时记录的固件版本（应用自己填的） */
    uint32_t endpoint_count;
} cb_desc_cache_meta_t;

/**
 * 只解析头部，不碰 store。用于“先看键对不对，再决定是否导入”。
 * @return JSDK_OK / JSDK_ERR_INVALID_ARG / JSDK_ERR_PROTOCOL
 */
int cb_desc_cache_peek(const void *buf, size_t len, cb_desc_cache_meta_t *out);

/** 覆盖导出头里的 fw_version（导出后再补写，避免导出时还要查设备）。 */
int cb_desc_cache_set_fw_version(void *buf, size_t len, uint32_t fw_version);

/**
 * 覆盖导出头里的描述符 VersionCRC（来自 `cb_desc_fetch_result_t.crc`）。
 * 应用把两个键写进自己的缓存头，启动时就能在导入前先比对。
 */
int cb_desc_cache_set_desc_crc(void *buf, size_t len, uint16_t crc);

/* --------------------------------------------------------------------------
 * 路线 B：从原始 JSON 重建
 * ------------------------------------------------------------------------ */

/**
 * 用**设备原始 JSON** 重建端点表（`retain` / `filter_paths` 按当前 cfg 生效）。
 *
 * 与 `cb_desc_cache_import()` 的区别：不依赖 SDK 内部格式，也不需要
 * `(fw_version, crc)` 之外的键；但要求 JSON 是**完整**的
 * （即下载时必须 `stop_when_satisfied = 0`，否则 `JSDK_ERR_PARSE`）。
 *
 * @return JSDK_OK / JSDK_ERR_INVALID_ARG / JSDK_ERR_PARSE / JSDK_ERR_NO_MEMORY
 */
int cb_desc_import_raw(const jsdk_desc_config_t *cfg, jsdk_ep_store_t *store,
                       const void *json, size_t len);

/** CRC-16/CCITT-FALSE（导出体校验用；也可供应用校验 Flash 内容）。 */
uint16_t cb_desc_cache_crc16(const void *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* CB_DESC_CACHE_H */
