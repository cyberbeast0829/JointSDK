/**
 * @file    hal_handle.h
 * @brief   内置 HAL 后端的句柄布局（**内部头，不安装**）
 *
 * @par 为什么需要它
 *  `jsdk_hal_handle_t` 对客户是不透明的，但**每个后端要有自己的资源**
 *  （socket fd / 串口 fd / PCAN 句柄 / 虚拟模型）。早先的版本只有虚拟后端，
 *  `jsdk_hal_close()` 直接 `free()` 就够了；一旦有真实后端，就必须让
 *  "关闭"分派到各后端 —— 否则 **fd 泄漏**，而且是那种跑一晚上才发现的泄漏。
 *
 *  因此所有后端句柄都以一个公共前缀开头：
 *  @code
 *      typedef struct { struct jsdk_hal_handle base; ... } sc_handle_t;
 *  @endcode
 *  `base.destroy` 由各后端的 `open()` 填好，`jsdk_hal_close()` 只负责转调。
 *  由于 `base` 是首个成员，`(T *)(void *)h` 是合法的反向转换。
 */

#ifndef JSDK_HAL_HANDLE_H
#define JSDK_HAL_HANDLE_H

#include "jsdk_hal_builtin.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 后端类别（`jsdk_hal_close()` 与诊断输出都用它）。 */
typedef enum {
    JSDK_HAL_KIND_NONE      = 0,
    JSDK_HAL_KIND_VIRTUAL   = 1,
    JSDK_HAL_KIND_SOCKETCAN = 2,
    JSDK_HAL_KIND_PCAN      = 3,
    JSDK_HAL_KIND_SLCAN     = 4
} jsdk_hal_kind_t;

/** 所有后端句柄的公共前缀。`destroy` 必须释放 @p h 自身。 */
struct jsdk_hal_handle {
    jsdk_hal_kind_t kind;
    void          (*destroy)(struct jsdk_hal_handle *h);
};

/** 由公共前缀取回具体后端结构体（`base` 是首成员，转换合法）。 */
#define JSDK_HAL_CAST(type, h) ((type *)(void *)(h))

/** 取后端类别（允许 NULL）。 */
jsdk_hal_kind_t jsdk_hal_kind_of(const struct jsdk_hal_handle *h);

/** 后端类别名（"virtual" / "socketcan" / "pcan" / "slcan" / "none"）。 */
const char *jsdk_hal_kind_name(jsdk_hal_kind_t k);

#ifdef __cplusplus
}
#endif

#endif /* JSDK_HAL_HANDLE_H */
