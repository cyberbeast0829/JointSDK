/**
 * @file    cb_param.h
 * @brief   CYBERBEAST 参数访问编解码（PARAM_READ 0x20 / PARAM_WRITE 0x21）
 *
 * 这是“端点（Endpoint）”访问层：用 u16 端点 ID 读写设备的任意配置/状态项。
 * 端点 ID → 名称/类型 的映射来自 JSON 描述符（见 cb_jsondesc_parse.h）。
 *
 * 本模块提供四种形式的**纯编解码**，外加两个状态机/规划器：
 *   - `cb_param_write_asm_t`：Classic 分段写的**接收侧装配器**（虚拟设备用）
 *   - `cb_param_plan_batches()`：批量读的**发送侧装箱器**（主站用）
 *
 * ============================================================================
 * 一、单参数读（0x20，Flags bit6 = 0）
 * ============================================================================
 * ```
 * 请求（4 B 或 8 B）:  [Flags][EpID u16 BE][ReqLen][Offset u32 BE]
 *   - ReqLen: 期望字节数 1..8；**0 视为 4**（旧客户端兼容）
 *   - Offset: 起始偏移；**请求帧短于 8 B 时视为 0**（旧客户端兼容）
 *   - Classic: ReqLen 被强制 ≤4（响应帧 = 4 B 头 + 数据，必须 ≤8 B）
 *
 * 响应:  [Flags][EpID u16 BE][DataLen][Value(DataLen B)]
 *   - Flags bit7 (0x80) = More：1 表示还有数据未返回
 *   - 设备**总是先按 8 字节读满整个值**再按 offset 切片，因此
 *     `Offset + DataLen < FullLen` 时置 More。
 *   - Offset ≥ 值长度 → DataLen = 0（合法，表示越界读完）
 * ```
 * 用途：读取 uint64 序列号等大参数。Classic 下需两块（offset 0 和 4）。
 *
 * ============================================================================
 * 二、批量读（0x20，Flags bit6 = 1，**仅 CAN FD**）
 * ============================================================================
 * ```
 * 请求:  [0x40][Count N][N × EpID u16 BE]          N ∈ 1..31
 * 响应:  [0x40][Count][ValidBitmap ⌈N/8⌉ B][ValueStream]
 *        - 不回显 EpID / DataLen：主站按 JSON 描述符的类型长度依次切分值流
 *        - ValidBitmap bit i = 1 表示第 i 条有效
 *        - 无分页：设备**永远全量返回**
 * 错误响应: [0x40|0x20][0x00]  ← 仅 2 B
 *        - 触发条件：① Classic 收到批量请求（不支持）
 *                    ② 整批值流装不下 64 B（主站应按类型拆分重发）
 * ```
 * 值流预算：`64 − 2 − ⌈N/8⌉` 字节。
 *
 * ============================================================================
 * 三、参数写（0x21）
 * ============================================================================
 * ```
 * 请求:  [Flags][EpID u16 BE][DataLen][Value(≤8 B)]
 * 确认:  [Flags 原样][EpID u16 BE][0][4 × 0x00]    固定 8 B
 * ```
 * ⚠ 确认帧**不回传是否写入成功**，也不回传错误码 —— 是静默确认。
 *   要确认写入生效必须回读一次（`PARAM_READ`）。
 *
 * ============================================================================
 * 四、分段写（Classic，DataLen > 4）
 * ============================================================================
 * Classic 8 B 帧装不下「4 B 头 + 8 B 值」，故退化为 4 B/块：
 * ```
 * 块请求（8 B）: [Flags][EpID u16 BE][TotalLen][Chunk 4 B]
 *   - Flags bit7 (0x80) = More：1 还有后续块，0 末块
 *   - TotalLen: 参数完整字节数，**只允许 5..8**；每块一致
 *   - 末块不足 4 B 的部分用 0 填充，设备按 TotalLen 截断
 * ```
 * 设备端装配的中止条件（丢块防护，本模块的装配器逐条复刻）：
 *   ① TotalLen 不在 5..8      ② offset 已 ≥ TotalLen
 *   ③ 已声明 More 但缓冲已填满  ④ EpID 或 master_id 中途变化 → 重新开始装配
 *
 * ⚠ 装配器在**中止时返回 -1，且不写入任何数据**——避免把半截值写进设备。
 */

#ifndef CB_PARAM_H
#define CB_PARAM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 标志位 ---- */
#define CB_PARAM_FLAG_MORE  0x80u   /**< 通用：还有后续数据 */
#define CB_PARAM_FLAG_BATCH 0x40u   /**< PARAM_READ：批量模式 */
#define CB_PARAM_FLAG_ERR   0x20u   /**< PARAM_READ 批量：整批无法完成 */

/* ---- 容量上限 ---- */
#define CB_PARAM_MAX_VALUE      8u    /**< 单个参数值最大字节数 */
#define CB_PARAM_MAX_BATCH      31u   /**< 单批最大条目数（2 + 2×31 = 64） */
#define CB_PARAM_FD_FRAME_MAX   64u   /**< CAN FD 单帧上限 */
#define CB_PARAM_CLASSIC_FRAME  8u    /**< Classic 单帧上限 */
#define CB_PARAM_CHUNK_BYTES    4u    /**< 分段写每块的字节数 */
#define CB_PARAM_SEG_MIN_LEN    5u    /**< 分段写的 TotalLen 下限 */
#define CB_PARAM_READ_REQ_MIN   4u    /**< 单读请求最短（无 offset） */
#define CB_PARAM_READ_REQ_FULL  8u    /**< 单读请求带 offset */

/**
 * 单帧参数写请求的**最短帧长**（字节）。
 *
 * ⚠⚠ 固件 `CANCyberBeast::cmd_param_write()` 的**第一句**就是：
 *
 * ```cpp
 * if (msg.len < 8) return;            // 整帧静默丢弃，连 ACK 都不回
 * ```
 *
 * 也就是说“4 字节头 + 1/2 字节值”（bool / u8 / u16）**根本不会被执行**。
 * 真机上表现为：写 bool / u16 报成功、读回旧值；只有 u32 / f32（刚好 8 字节）能生效。
 * 而 `cmd_param_read()` 的门限是 4（所以读一直是好的）—— 两边不对称，极容易错。
 *
 * 因此发送侧**一律补齐到 8 字节**：`dst[3]`（真值字节数）不变，
 * 固件拿到的仍然是“长度正确”的值（它只把 `data_len` 个字节交给端点处理器）。
 */
#define CB_PARAM_WRITE_REQ_MIN  8u    /**< 单帧写请求最短帧长（固件硬要求 ≥ 8） */
#define CB_PARAM_ACK_LEN        8u    /**< 写确认固定 8 B */

/* ==========================================================================
 * 一、单参数读
 * ======================================================================== */

/** 单读请求（解析结果；字段已按固件规则归一化） */
typedef struct {
    uint8_t  flags;
    uint16_t ep_id;
    uint8_t  req_len;       /**< 已归一化：0→4，>8→8，Classic>4→4 */
    uint32_t offset;        /**< 无 offset 字段时为 0 */
    int      has_offset;    /**< 请求帧是否真的带了 offset 字段 */
} cb_param_read_req_t;

/** 单读响应 */
typedef struct {
    uint8_t  flags;         /**< bit7 = More */
    uint16_t ep_id;         /**< 请求 EpID 回显 */
    uint8_t  data_len;
    uint8_t  value[CB_PARAM_MAX_VALUE];
} cb_param_read_rsp_t;

/**
 * 打包单读请求。
 * @param with_offset 非 0 → 发出完整 8 B（含 offset）；0 → 发 4 B 旧式请求
 * @return 写入字节数（4 或 8）；参数非法返回 0
 *
 * ⚠ 这里**不**接受 `classic` 参数：Classic 的 `ReqLen ≤ 4` 限制是**设备侧**
 *   归一化的（见 cb_param_normalize_req_len），主站发 8 设备也只会回 4 字节。
 *   主站若要提前知道实际能拿到多少字节，应自己调 cb_param_normalize_req_len()
 *   或 cb_param_read_chunks()，而不是让编码函数默默改动请求内容。
 */
size_t cb_param_pack_read_req(uint8_t *dst, size_t cap, uint16_t ep_id,
                              uint8_t req_len, uint32_t offset,
                              int with_offset);

/**
 * 固件在**设备侧**对 ReqLen 做的归一化（导出供主站预估切片）。
 *   0 → 4（旧客户端兼容）；> 8 → 8；Classic 且 > 4 → 4
 */
uint8_t cb_param_normalize_req_len(uint8_t req_len, int classic);

/**
 * 读一个值需要几次请求/响应（主站用来预分配循环次数）。
 * @param value_len 该端点在 JSON 描述符里的类型长度
 * @return 需要的请求次数；value_len = 0 时返回 0
 */
uint32_t cb_param_read_chunks(uint8_t value_len, uint8_t req_len, int classic);

/** 解析单读请求（应用与固件相同的归一化） */
int cb_param_unpack_read_req(const uint8_t *src, size_t len, int classic,
                             cb_param_read_req_t *out);

/** 打包单读响应 */
size_t cb_param_pack_read_rsp(uint8_t *dst, size_t cap,
                              const cb_param_read_rsp_t *v);

/** 解析单读响应 */
int cb_param_unpack_read_rsp(const uint8_t *src, size_t len,
                             cb_param_read_rsp_t *out);

/**
 * **设备侧**：按请求对完整值做切片并生成响应帧（虚拟设备用，逐字节复刻固件）。
 *
 * @param full_value 设备的完整值（长度 = full_len，≤8）
 * @param full_len   完整值长度
 * @param req_flags  请求的 Flags（响应会回显其低 7 位）
 * @return 响应帧长度（4 + DataLen）；参数非法返回 0
 */
size_t cb_param_build_read_rsp(uint8_t *dst, size_t cap,
                               uint8_t req_flags, uint16_t ep_id,
                               const uint8_t *full_value, uint8_t full_len,
                               uint8_t req_len, uint32_t offset);

/** 响应是否还有后续块（More 位） */
int cb_param_rsp_has_more(const cb_param_read_rsp_t *rsp);

/* ==========================================================================
 * 二、批量读
 * ======================================================================== */

/** 批量读请求的解析结果 */
typedef struct {
    uint8_t        count;      /**< 实际解析出的条目数（≤31） */
    const uint8_t *ep_bytes;   /**< 指向 src[2]，每个条目 2 B BE */
} cb_param_batch_req_t;

/** 批量读响应的解析结果（值流需由调用方按 JSON 类型长度切分） */
typedef struct {
    uint8_t        flags;
    uint8_t        count;         /**< 正常 = N；ERR 时为 0 */
    uint8_t        bitmap_bytes;  /**< ⌈N/8⌉；ERR 时为 0 */
    const uint8_t *bitmap;        /**< 指向 src[2]；ERR 时为 NULL */
    const uint8_t *values;        /**< 值流起始；ERR 时为 NULL */
    size_t         values_len;    /**< 值流字节数 */
    int            is_err;        /**< 1 = 设备报 ERR，需拆分重发 */
} cb_param_batch_rsp_t;

/**
 * 打包批量读请求（仅 CAN FD）。
 * @param n 条目数，须在 1..31
 * @return 写入字节数（2 + 2n）；非法返回 0
 */
size_t cb_param_pack_batch_req(uint8_t *dst, size_t cap,
                               const uint16_t *eps, uint8_t n);

/**
 * 解析批量读请求。
 * @param ep_cap @p out_eps 的容量（条目数）
 * @return 0 成功（*out 已填）；-1 不是批量请求或长度非法
 */
int cb_param_unpack_batch_req(const uint8_t *src, size_t len,
                              uint16_t *out_eps, uint8_t ep_cap,
                              cb_param_batch_req_t *out);

/**
 * 解析批量读响应。
 * @param n_req 请求时的条目数 N（用于校验 bitmap 大小）
 * @return 0 成功；-1 格式非法
 */
int cb_param_unpack_batch_rsp(const uint8_t *src, size_t len, uint8_t n_req,
                              cb_param_batch_rsp_t *out);

/** 打包批量读的 ERR 响应（2 B） */
size_t cb_param_pack_batch_err(uint8_t *dst, size_t cap);

/** 测试 ValidBitmap 的第 i 位 */
int cb_param_bitmap_test(const uint8_t *bitmap, uint8_t n_bytes, uint8_t index);

/** 计算 N 个条目需要的 bitmap 字节数 */
uint8_t cb_param_bitmap_bytes(uint8_t n);

/** 批量装箱用的条目描述（value_len 来自 JSON 描述符的类型长度） */
typedef struct {
    uint16_t ep_id;
    uint8_t  value_len;     /**< 该端点值的序列化字节数（0 表示无效端点） */
} cb_param_batch_item_t;

/**
 * **主站侧**：把条目序列贪心装箱成若干批，每批满足
 * `2 + ⌈n/8⌉ + Σvalue_len ≤ budget`（budget 通常取 64）。
 *
 * @param out_counts      输出：每批的条目数
 * @param out_offsets     输出：每批在 items 中的起始下标（可为 NULL）
 * @param max_batches     out_counts 的容量
 * @return 批次数；若条目数为 0 或任一单条装不下则返回 0
 *
 * 注意：装箱只保证**响应**装得下；请求帧要求 `2 + 2n ≤ 64` 即 n ≤ 31，
 *       本函数同时满足该约束。
 */
size_t cb_param_plan_batches(const cb_param_batch_item_t *items, size_t n_items,
                             size_t budget,
                             size_t *out_counts, size_t *out_offsets,
                             size_t max_batches);

/* ==========================================================================
 * 三、参数写
 * ======================================================================== */

/** 打包写请求；value_len 须 ≤8 */
size_t cb_param_pack_write_req(uint8_t *dst, size_t cap, uint16_t ep_id,
                               const uint8_t *value, uint8_t value_len);

/** 解析写请求 */
int cb_param_unpack_write_req(const uint8_t *src, size_t len,
                              uint8_t *out_flags, uint16_t *out_ep_id,
                              uint8_t *out_value, uint8_t *out_value_len);

/** 打包写确认（8 B，Flags 原样返回） */
size_t cb_param_pack_write_ack(uint8_t *dst, size_t cap,
                               uint8_t flags, uint16_t ep_id);

/** 解析写确认（仅校验格式，确认帧不含成败信息） */
int cb_param_unpack_write_ack(const uint8_t *src, size_t len,
                              uint8_t *out_flags, uint16_t *out_ep_id);

/* ==========================================================================
 * 四、分段写
 * ======================================================================== */

/**
 * 打包一个分段写块（Classic，8 B）。
 * @param offset 该块在完整值中的起始偏移（须为 4 的倍数，最后一块可不足 4）
 * @param more   非 0 → 置 More 位
 * @return 8；参数非法返回 0
 */
size_t cb_param_pack_write_chunk(uint8_t *dst, size_t cap, uint16_t ep_id,
                                 uint8_t total_len, uint32_t offset,
                                 const uint8_t *value, uint8_t value_len,
                                 int more);

/** 分段写装配器（接收侧，复刻固件的丢块防护） */
typedef struct {
    int      active;
    uint16_t ep_id;
    uint8_t  master_id;
    uint8_t  total_len;
    uint8_t  offset;
    uint8_t  buf[CB_PARAM_MAX_VALUE];
} cb_param_write_asm_t;

/** 重置装配器 */
void cb_param_write_asm_init(cb_param_write_asm_t *a);

/**
 * 喂入一个分段写块。
 *
 * @param value_out     组装完成时输出完整值（至少 8 B）
 * @param value_len_out 组装完成时输出 total_len
 * @return
 *    1  已接收，**还需后续块**
 *    0  组装完成（*value_out / *value_len_out 有效，应执行写入并回 ACK）
 *   -1  中止（TotalLen 非法 / 块序错误 / 声明 More 却已填满），**不写入**
 */
int cb_param_write_asm_feed(cb_param_write_asm_t *a, uint8_t master_id,
                            uint16_t ep_id, uint8_t total_len, uint8_t flags,
                            const uint8_t *chunk,
                            uint8_t *value_out, uint8_t *value_len_out);

#ifdef __cplusplus
}
#endif

#endif /* CB_PARAM_H */
