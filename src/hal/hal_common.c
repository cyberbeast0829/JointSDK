/**
 * @file    hal_common.c
 * @brief   内置 HAL 的公共部分：句柄关闭 + 未编译后端的明确报错
 *
 * @par 为什么单独一个文件（这是踩过的坑）
 *  早先 `jsdk_hal_close()` 和三个"未实现"占位都写在 `hal_virtual.c` 里。
 *  于是：
 *    ① 客户只开 `JSDK_BUILD_HAL_SOCKETCAN`（不开 virtual）时，**`jsdk_hal_close()`
 *       直接链接不到** —— 而且 `jsdk_hal_socketcan_open()` 也拿到占位版的
 *       `JSDK_ERR_UNSUPPORTED`，因为占位用 `#ifndef JSDK_BUILD_HAL_SOCKETCAN` 守卫，
 *       而那个宏以前从来没被 CMake 定义过；
 *    ② `jsdk_hal_close()` 对真实后端是错的：它对句柄直接 `free()`，
 *       不关 fd/串口 —— **fd 泄漏**，一晚上跑下来就耗尽。
 *
 *  现在：占位改成"只有真正的实现在编译时才不提供"（`JSDK_HAL_HAVE_*`），
 *  关闭走各后端自己的 `base.destroy`。
 */

#include "hal_handle.h"

#include <stdlib.h>

/* --------------------------------------------------------------------------
 * 后端类别
 * ------------------------------------------------------------------------ */

jsdk_hal_kind_t jsdk_hal_kind_of(const struct jsdk_hal_handle *h)
{
    return h ? h->kind : JSDK_HAL_KIND_NONE;
}

const char *jsdk_hal_kind_name(jsdk_hal_kind_t k)
{
    switch (k) {
    case JSDK_HAL_KIND_VIRTUAL:   return "virtual";
    case JSDK_HAL_KIND_SOCKETCAN: return "socketcan";
    case JSDK_HAL_KIND_PCAN:      return "pcan";
    case JSDK_HAL_KIND_SLCAN:     return "slcan";
    case JSDK_HAL_KIND_NONE:
    default:                      return "none";
    }
}

/* --------------------------------------------------------------------------
 * 关闭（幂等）
 * ------------------------------------------------------------------------ */

jsdk_status_t jsdk_hal_close(jsdk_hal_handle_t *h)
{
    if (!h) return JSDK_OK;
    if (h->destroy) {
        h->destroy(h);       /* 由各后端负责关 fd / 串口 / 释放内存 */
        return JSDK_OK;
    }
    /* 防御：不可能是"别人家的"句柄，但真出现了也别泄漏内存 */
    free(h);
    return JSDK_OK;
}

/* --------------------------------------------------------------------------
 * 未编译的后端：给明确错误，不静默失败
 *
 * `JSDK_HAL_HAVE_*` 由 CMake 在编译该后端时定义；对应源文件**不在**目标里时
 * 这里提供占位。
 *
 * ⚠ 占位**也要做参数校验**：如果某种参数在 Linux 上是 INVALID_ARG、在 Windows
 *   上却变成 UNSUPPORTED，客户就得写两套判断。参数错就是参数错。
 * ------------------------------------------------------------------------ */

#ifndef JSDK_HAL_HAVE_SOCKETCAN
jsdk_status_t jsdk_hal_socketcan_open(jsdk_can_hal_t *hal, jsdk_hal_handle_t **out,
                                      const char *ifname,
                                      uint32_t bitrate, uint32_t data_bitrate)
{
    (void)bitrate; (void)data_bitrate;
    if (!hal || !out || !ifname || !ifname[0]) return JSDK_ERR_INVALID_ARG;
    *out = NULL;
    return JSDK_ERR_UNSUPPORTED;   /* 该平台未启用 JSDK_BUILD_HAL_SOCKETCAN */
}
#endif

#ifndef JSDK_HAL_HAVE_PCAN
jsdk_status_t jsdk_hal_pcan_open(jsdk_can_hal_t *hal, jsdk_hal_handle_t **out,
                                 const char *channel,
                                 uint32_t bitrate, uint32_t data_bitrate)
{
    (void)bitrate; (void)data_bitrate;
    if (!hal || !out || !channel || !channel[0]) return JSDK_ERR_INVALID_ARG;
    *out = NULL;
    return JSDK_ERR_UNSUPPORTED;   /* 该平台未启用 JSDK_BUILD_HAL_PCAN */
}
#endif

#ifndef JSDK_HAL_HAVE_SLCAN
jsdk_status_t jsdk_hal_slcan_open(jsdk_can_hal_t *hal, jsdk_hal_handle_t **out,
                                  const char *port, uint32_t baud,
                                  uint32_t data_bitrate)
{
    (void)baud; (void)data_bitrate;
    if (!hal || !out || !port || !port[0]) return JSDK_ERR_INVALID_ARG;
    *out = NULL;
    return JSDK_ERR_UNSUPPORTED;   /* 该平台未启用 JSDK_BUILD_HAL_SLCAN */
}

/*
 * ⚠ 诊断函数也要有占位。
 *   它们现在是**公共 ABI**（`jsdk_hal_builtin.h` 里带 `JSDK_API`），客户代码
 *   可以无条件调用；少了占位就会在 `JSDK_BUILD_HAL_SLCAN=OFF` 的构建里变成
 *   "undefined reference"，而那种错误在 CI 上只在特定平台配置下才出现。
 *
 *   返回值语义：后端没编进来 → “这个后端这里不可用”，所以 `supports_fd` 返回 0、
 *   计数全 0（而不是"slcan 协议不支持 FD"—— 那是另一回事，slcan 是支持的）。
 */
int jsdk_hal_slcan_supports_fd(void)
{
    return 0;                      /* 本构建里没有 slcan 后端 */
}

void jsdk_hal_slcan_fd_config(jsdk_hal_handle_t *h, int *enabled,
                              uint32_t *bitrate)
{
    (void)h;
    if (enabled) *enabled = 0;
    if (bitrate) *bitrate = 0u;
}

void jsdk_hal_slcan_fd_frames(jsdk_hal_handle_t *h, uint32_t *tx_fd,
                              uint32_t *rx_fd)
{
    (void)h;
    if (tx_fd) *tx_fd = 0u;
    if (rx_fd) *rx_fd = 0u;
}

void jsdk_hal_slcan_stats(jsdk_hal_handle_t *h, uint32_t *tx, uint32_t *rx,
                          uint32_t *malformed, uint32_t *acks, uint32_t *nacks)
{
    (void)h;
    if (tx)        *tx        = 0u;
    if (rx)        *rx        = 0u;
    if (malformed) *malformed = 0u;
    if (acks)      *acks      = 0u;
    if (nacks)     *nacks     = 0u;
}
#endif

#ifndef JSDK_HAL_HAVE_VIRTUAL
jsdk_status_t jsdk_hal_virtual_open(jsdk_can_hal_t *hal, jsdk_hal_handle_t **out,
                                    const char *node_spec)
{
    (void)node_spec;
    if (!hal || !out) return JSDK_ERR_INVALID_ARG;
    *out = NULL;
    return JSDK_ERR_UNSUPPORTED;   /* 未启用 JSDK_BUILD_HAL_VIRTUAL */
}

jsdk_status_t jsdk_hal_virtual_inject(jsdk_hal_handle_t *h, const jsdk_can_frame_t *f)
{
    (void)h; (void)f;
    return JSDK_ERR_UNSUPPORTED;
}

int jsdk_hal_virtual_capture(jsdk_hal_handle_t *h, jsdk_can_frame_t *f)
{
    (void)h; (void)f;
    return 0;
}

void jsdk_hal_virtual_set_tx_fail(jsdk_hal_handle_t *h, int fail)
{
    (void)h; (void)fail;
}

uint32_t jsdk_hal_virtual_dropped(const jsdk_hal_handle_t *h)
{
    (void)h;
    return 0u;
}

void jsdk_hal_virtual_advance_ms(jsdk_hal_handle_t *h, uint32_t ms)
{
    (void)h; (void)ms;
}
#endif
