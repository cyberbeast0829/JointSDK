/**
 * @file    ex_common.h
 * @brief   示例共用的启动样板（**只给使用内置虚拟后端的示例用**）
 *
 * 为什么单独抽出来：6 个示例里"开虚拟后端 → 建上下文 → 给 arena → 加关节 →
 * 下描述符 → configure"这一段完全一样，重复 6 遍只会让每份示例的重点被淹没。
 * 真正要看懂的那几行（使能、控制、参数、广播、缓存）在每个示例里都是**直的**。
 *
 * `07_custom_hal.c` **故意不用本文件** —— 它演示的是"不用任何内置后端，
 * 自己实现 `jsdk_can_hal_t`"，那才是 MCU 客户的入口。
 *
 * ⚠ 这里全部是 `static inline`：示例各自编译成独立可执行文件，
 *   普通 `static` 函数在某个示例里没被用到会触发 `-Wunused-function`
 *   （而本项目的示例是按 `-Werror` 编译的）。
 */

#ifndef EX_COMMON_H
#define EX_COMMON_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "joint_sdk.h"
#include "jsdk_hal_builtin.h"

typedef struct {
    jsdk_can_hal_t        hal;      /**< 由虚拟后端填充；会被 init 复制 */
    jsdk_hal_handle_t    *hh;       /**< 虚拟后端句柄 */
    jsdk_context_config_t cfg;
    jsdk_context_storage_t store;   /**< 零 malloc：上下文放调用者的静态存储里 */
    jsdk_context_t       *ctx;
    void                 *arena;    /**< 描述符 arena（这里用堆，MCU 上换成静态数组） */
    jsdk_joint_t         *joint[JSDK_MAX_JOINTS_STATIC];
    unsigned              nj;
} ex_ctx_t;

/** 出错就"大声退场"：示例不吞错误（客户照抄时最怕示例里静默忽略失败）。 */
static inline void ex_fatal(const char *what, jsdk_status_t st)
{
    fprintf(stderr, "example: %s -> %s\n", what, jsdk_status_string(st));
    exit(1);
}

static inline void ex_check(const char *what, jsdk_status_t st)
{
    printf("  %-26s %s\n", what, jsdk_status_string(st));
    if (st != JSDK_OK) ex_fatal(what, st);
}

/**
 * 打开虚拟后端并完成配置阶段。
 *
 * @param spec      虚拟设备规格（见 jsdk_hal_virtual_open 的说明）
 * @param node_ids  要加入上下文的关节节点号（**必须与 spec 里的 id 对应**）
 * @param nj        关节数量
 * @param period_us 控制周期（微秒）；设得比设备的 break_timeout 小才能通过 configure
 */
static inline void ex_open(ex_ctx_t *e, const char *spec,
                           const unsigned *node_ids, unsigned nj, unsigned period_us)
{
    jsdk_status_t st;
    unsigned      i;

    memset(e, 0, sizeof *e);

    st = jsdk_hal_virtual_open(&e->hal, &e->hh, spec);
    if (st != JSDK_OK) ex_fatal("hal_virtual_open", st);

    /* ⚠ 必须打开：configure()/desc_fetch() 是**阻塞式**的，靠时钟前进轮询响应；
       虚拟时钟默认冻结（只在 advance_ms 里走），冻结时钟下它们会等到自旋上限。 */
    jsdk_hal_virtual_set_autotick(e->hh, 1);

    jsdk_context_config_default(&e->cfg);
    e->cfg.hal                 = e->hal;
    e->cfg.master_id           = 1u;
    e->cfg.is_fd               = 1u;
    e->cfg.period_ns           = (uint32_t)period_us * 1000u;
    e->cfg.desc.mode           = JSDK_DESC_DYNAMIC;   /* 没缓存就下载 */
    e->cfg.desc.retain         = JSDK_DESC_RETAIN_ALL;
    e->cfg.desc.timeout_ms     = 30000u;

    e->arena = calloc(1u, jsdk_desc_arena_size(&e->cfg.desc));
    if (!e->arena) ex_fatal("arena 分配", JSDK_ERR_NO_MEMORY);
    e->cfg.desc.arena      = e->arena;
    e->cfg.desc.arena_size = jsdk_desc_arena_size(&e->cfg.desc);

    e->ctx = (jsdk_context_t *)&e->store;
    ex_check("context_init", jsdk_context_init(e->ctx, &e->cfg));

    for (i = 0u; i < nj; ++i) {
        jsdk_joint_config_t jc;
        memset(&jc, 0, sizeof jc);
        jc.node_id      = (uint8_t)node_ids[i];
        jc.initial_mode = JSDK_MODE_MIT;
        st = jsdk_context_add_joint(e->ctx, &jc, &e->joint[i]);
        if (st != JSDK_OK) ex_fatal("add_joint", st);
        e->nj++;
    }

    ex_check("desc_fetch", jsdk_context_desc_fetch(e->ctx));
    ex_check("configure",  jsdk_context_configure(e->ctx));
}

/** 跑 n 个控制周期（每个周期之间推进 period_ns）。 */
static inline void ex_run_cycles(ex_ctx_t *e, unsigned n)
{
    uint64_t t;
    unsigned k;

    for (k = 0u; k < n; ++k) {
        t = (uint64_t)k * e->cfg.period_ns;
        (void)jsdk_context_cycle_begin(e->ctx, t);
        (void)jsdk_context_cycle_end(e->ctx);
    }
}

static inline void ex_close(ex_ctx_t *e)
{
    jsdk_context_deactivate(e->ctx);
    (void)jsdk_hal_close(e->hh);
    free(e->arena);
}

#endif /* EX_COMMON_H */
