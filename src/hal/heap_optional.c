/**
 * @file    heap_optional.c
 * @brief   可选的**堆模式**便利构造（默认不编译，见 CMake 选项 JSDK_ENABLE_HEAP）
 *
 * 核心库本身**零 malloc**：上下文与端点表都放在调用者提供的存储里。
 * 本文件只在桌面/CLI/测试这类"不在乎一次 malloc"的场景下提供更省事的入口。
 *
 * @par 与零 malloc 模式的关系
 *  - 零 malloc：`jsdk_context_init((jsdk_context_t*)&storage, &cfg)`
 *  - 堆模式  ：`ctx = jsdk_context_create(&cfg)` … `jsdk_context_free(ctx)`
 * 两者**不冲突**：`create()` 内部就是 malloc + `init()`，释放用 `free()`。
 *
 * @warning 可执行文件里**要么全用零 malloc，要么全用堆模式**，混用没有意义
 *          （而且会让"这个上下文该怎么释放"变得难以推理）。
 * @warning MCU 构建**不要**打开 `JSDK_ENABLE_HEAP`。
 */

#include "jsdk_core_internal.h"

#include <stdlib.h>
#include <string.h>

jsdk_context_t *jsdk_context_create(const jsdk_context_config_t *cfg)
{
    jsdk_context_t *ctx;
    jsdk_desc_config_t desc;
    void *arena = NULL;

    if (!cfg) return NULL;

    /* 端点表也要有人提供。堆模式下若调用者没给，就由我们代管（free 时释放）。 */
    desc = cfg->desc;
    if (!desc.arena || desc.arena_size == 0u) {
        size_t need = jsdk_desc_arena_size(&desc);
        if (need == 0u) return NULL;
        arena = malloc(need);
        if (!arena) return NULL;
        desc.arena      = arena;
        desc.arena_size = need;
    }

    ctx = (jsdk_context_t *)malloc(sizeof *ctx);
    if (!ctx) {
        free(arena);
        return NULL;
    }

    {
        jsdk_context_config_t c = *cfg;
        c.desc = desc;

        if (jsdk_context_init(ctx, &c) != JSDK_OK) {
            free(ctx);
            free(arena);
            return NULL;
        }
    }

    /* 记住哪些是我们要负责释放的（零 malloc 模式下两者都是 NULL） */
    ctx->owned_arena = arena;
    return ctx;
}

void jsdk_context_free(jsdk_context_t *ctx)
{
    void *arena;

    if (!jsdk_ctx_check(ctx)) return;

    arena = ctx->owned_arena;
    jsdk_context_destroy(ctx);
    free(ctx);
    free(arena);
}
