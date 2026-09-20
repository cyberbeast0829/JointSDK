/**
 * @file    hal_slcan_codec.h
 * @brief   slcan（ASCII 行协议）编解码 —— **纯逻辑，无平台依赖**
 *
 * @par 为什么单独一个文件
 *  slcan 的编解码是整个后端里唯一"有判断逻辑"的部分（帧格式、转义、DL、
 *  RTR/EFF 标志、错误行）。把它和串口 I/O 分开，就能**在没有硬件的机器上**
 *  用单元测试把格式钉死；剩下的 open/read/write 只是系统调用搬运。
 *
 * @par 协议要点（Lawicel slcan + CANable 2.0 的 FD 扩展）
 *  - CAN 2.0 数据帧：`t` + 3 位十六进制 ID（标准帧）
 *                     `T` + 8 位十六进制 ID（扩展帧）
 *  - 可选 DLC 字符 `0..8`，随后是十六进制载荷。**DLC 省略 = 0 字节**
 *    （Lawicel 规范里 DLC 可省略；要发 8 字节必须写 `8` 并跟上 16 个十六进制字符）
 *  - 行尾必须是 `\r`（部分固件收 `\n` 也能跑，但**发送**要用 `\r`）
 *  - 远端 RTR 帧：`r`/`R` 前缀；本 SDK 不使用，但**必须能解析并拒绝**，
 *    否则会把 RTR 帧当成数据帧交给协议层
 *  - 应答/错误行：`\r`（仅回车，表示 OK）、`\a`（BELL，表示失败）
 *
 * @par CAN FD 帧（**CANable 2.0 固件扩展**，Lawicel 原版没有）
 *  四个前缀字母，**小写 = 标准帧，大写 = 扩展帧**：
 *
 *  | 前缀 | ID   | BRS |
 *  |------|------|-----|
 *  | `d`  | 标准 | 0   |
 *  | `D`  | 扩展 | 0   |
 *  | `b`  | 标准 | 1   |
 *  | `B`  | 扩展 | 1   |
 *
 *  ⚠ **`b/B` 是“带 BRS”，不是“不带”** —— 这一点反直觉，但以 CANable 2.0 固件
 *    与 python-can 的 `can/interfaces/slcan.py` 为准（本 SDK 早期版本据此把
 *    slcan 判定为“不支持 FD”，是错的）。
 *
 *  FD 帧的载荷长度由紧跟 ID 的**一位十六进制 FD DLC 码**决定：
 *  `0..8` → 0..8 字节，`9`→12、`A`→16、`B`→20、`C`→24、`D`→32、`E`→48、`F`→64。
 *  即 **9/10/11 字节在 FD 里无法表示**（下一个可用长度直接跳到 12）。
 *  载荷**必须写满** DLC 码对应的字节数（不写满 = 语法错误）。
 *
 *  @note **已知有损点（与 python-can 相同）**：slcan 的字母表里没有 ESI（错误状态
 *        指示位）与 FD 的远程帧，所以这两样信息在收发时**丢弃**。本协议不使用它们
 *        （MIT/参数/描述符帧均为无 ESI 的数据帧）。
 *  @note 长度 9/10/11 字节在 FD 里无对应码：编码**返回 0**、解码报 `MALFORMED`。
 *        SDK 自己的帧长（MIT 8 B、描述符 64 B）都在表内。
 */

#ifndef JSDK_HAL_SLCAN_CODEC_H
#define JSDK_HAL_SLCAN_CODEC_H

#include <stddef.h>
#include <stdint.h>

#include "joint_sdk.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 一条 slcan 文本行最坏情况长度（含 `\r` 与结尾 NUL）。
 *
 * 最坏是 FD 扩展帧：`D` + 8 位 ID + 1 位 DLC + 128 个十六进制字符 + `\r` = 139。
 * 取 144 留余量，且是 8 的倍数（便于在结构体里排布）。
 */
#define JSDK_SLCAN_LINE_MAX 144u

/** 解码结果。 */
typedef enum {
    JSDK_SLCAN_OK        = 0,   /**< 解出一帧（数据帧；FD 与 Classic 都可能） */
    JSDK_SLCAN_ACK       = 1,   /**< `\r`：固件确认 */
    JSDK_SLCAN_NACK      = 2,   /**< `\a`：固件/总线错误 */
    JSDK_SLCAN_RTR       = 3,   /**< 合法但是远端请求帧（本协议不用） */
    JSDK_SLCAN_MALFORMED = 4,   /**< 语法错误 */
    JSDK_SLCAN_NEED_MORE = 5    /**< 行未结束（未见到 `\r`），继续喂 */
} jsdk_slcan_rc_t;

/**
 * CAN FD 的 DLC 码 → 字节数。
 * @param code 0..15
 * @return 字节数（0..64）；@p code 越界返回 -1
 */
int jsdk_slcan_fd_len_from_dlc(unsigned code);

/**
 * 字节数 → CAN FD 的 DLC 码（上一函数的逆）。
 * @param len 0..64
 * @return DLC 码 0..15；**无法表示的长度（9/10/11 与 >64）返回 -1**
 */
int jsdk_slcan_fd_dlc_from_len(unsigned len);

/**
 * 把一帧编码成 slcan 行（含 `\r`，**不含** NUL）。
 *
 * 帧类型完全由 @p f 的标志位决定：
 *  - `JSDK_FRAME_EXT` → 8 位 ID 与大写前缀，否则 3 位 ID 与小写前缀；
 *  - `JSDK_FRAME_FD`  → 用 `d`/`D`（BRS=0）或 `b`/`B`（`JSDK_FRAME_BRS`，BRS=1）；
 *  - 否则用 `t`/`T` 走 Classic（此时 `len` 必须 ≤ 8）。
 *
 * @param f       待编码帧
 * @param out     输出缓冲，建议 `char out[JSDK_SLCAN_LINE_MAX]`
 * @param cap     输出缓冲容量
 * @return 写入的字节数（含 `\r`）；0 = 无法表示（FD 的 9/10/11 字节、Classic 的
 *         >8 字节）或缓冲不足 —— **绝不静默降级成另一条帧**
 */
size_t jsdk_slcan_encode(const jsdk_can_frame_t *f, char *out, size_t cap);

/**
 * 解码一行 slcan 文本。
 *
 * @param line    输入（**可以**包含尾部 `\r`；不需要 NUL 结尾，靠 @p len）
 * @param len     输入长度（解析到 `\r` 或 @p len 为止）
 * @param f       解出的帧（仅 @ref JSDK_SLCAN_OK 时有效）；FD 帧会带上
 *                `JSDK_FRAME_FD` / `JSDK_FRAME_BRS` 标志
 * @param consumed 已消费的字节数（含 `\r`）；可为 NULL
 * @return 见 @ref jsdk_slcan_rc_t
 *
 * @note `MALFORMED` 时也会设置 @p consumed，调用方据此丢弃整行后继续。
 *
 * @warning **载荷长度必须与 DLC 码严格对应**（FD 帧不补齐就不合法）。
 *  写短了会返回 `MALFORMED` 而不是“解出一条短帧”：把一帧截断的 JSON 交给
 *  描述符解析器，比当场报语法错误难查得多。
 */
jsdk_slcan_rc_t jsdk_slcan_decode(const char *line, size_t len,
                                  jsdk_can_frame_t *f, size_t *consumed);

#ifdef __cplusplus
}
#endif

#endif /* JSDK_HAL_SLCAN_CODEC_H */
