/**
 * @file    consumer.c
 * @brief   安装后的消费者程序（A2+A6 的验收件）
 *
 * 这个文件**不属于** SDK 的构建树 —— 它是打包冒烟的"客户视角"：
 * 只通过**安装出去的**头文件与库工作，不碰 `src/`，也不依赖 CMake 变量。
 *
 * 用它验证两件事（见 `tools/packaging_smoke.sh`）：
 *   1. `pkg-config --cflags --libs joint-sdk-can` 给出的 flags 足够编译+链接；
 *   2. `find_package(jsdk_can)` + `target_link_libraries(app jsdk::can)` 同样够。
 *
 * 两处**故意这样写**，因为它们是打包最容易错的地方：
 *   - `#include "joint_sdk.h"`（引号形式）→ 证明 `-I<inc>/joint_sdk` 真的给到了；
 *   - 真的跑一遍虚拟后端（下描述符 + 使能 + 一帧控制 + 读参数）→ 证明链接的不只是
 *     `libm`/`libdl` 那些依赖，而是**整个库真能用**（静态库漏链依赖时，
 *     `-lm`/`-ldl` 缺失只会在链接期炸；漏了别的符号则在这一步才炸）。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "joint_sdk.h"
#include "jsdk_hal_builtin.h"

#define ARENA_GUARD 64u   /* arena 后面多给一点，便于发现越界写 */

static int g_fail;

static void step(const char *what, jsdk_status_t st)
{
    printf("  %-28s %s\n", what, jsdk_status_string(st));
    if (st != JSDK_OK) g_fail = 1;
}

int main(void)
{
    jsdk_can_hal_t        hal;
    jsdk_hal_handle_t    *hh = NULL;
    jsdk_context_config_t cfg;
    /* `jsdk_context_t` 是**不完整类型**（有意为之：尺寸由 jsdk_abi_types()
       与 JSDK_CONTEXT_MAX_SIZE 公开，但布局不暴露）——所以零 malloc 用法是
       让调用者提供存储，然后把它转成上下文指针。 */
    static jsdk_context_storage_t s_store;
    jsdk_context_t       *ctx = (jsdk_context_t *)&s_store;
    void                 *arena;
    jsdk_joint_config_t   jc;
    jsdk_joint_t         *joint = NULL;
    size_t                n_types = 0u;
    const jsdk_abi_type_t *types;
    jsdk_status_t         st;
    jsdk_value_t          v;
    jsdk_joint_feedback_t fb;

    printf("consumer: %s\n", jsdk_backend_name());

    /* --- ABI 自检：客户程序第一步就该做这件事 --- */
    types = jsdk_abi_types(&n_types);
    if (!types || n_types == 0u) { g_fail = 1; }
    else {
        size_t k;
        for (k = 0u; k < n_types; ++k) {
            if (strstr(types[k].name, "context") != NULL) {
                printf("  abi: %s size=%u align=%u\n", types[k].name,
                       (unsigned)types[k].size, (unsigned)types[k].align);
                break;
            }
        }
    }
    printf("  abi types=%lu, storage=%u bytes\n",
           (unsigned long)n_types, (unsigned)JSDK_CONTEXT_MAX_SIZE);

    /* --- 虚拟后端（自带设备模型；开 autotick 才能跑阻塞式配置 API） --- */
    st = jsdk_hal_virtual_open(&hal, &hh, "0:id=1,gear=16.5,pmax=12.5,vmax=65,"
                                         "tmax=50,hb=10,timeout=30000,fd");
    step("hal_virtual_open", st);
    if (st != JSDK_OK) return 1;
    jsdk_hal_virtual_set_autotick(hh, 1);

    jsdk_context_config_default(&cfg);
    cfg.hal           = hal;
    cfg.master_id     = 1u;
    cfg.is_fd         = 1u;
    cfg.period_ns     = 10000000u;             /* 100 Hz */
    cfg.desc.mode     = JSDK_DESC_DYNAMIC;
    cfg.desc.retain   = JSDK_DESC_RETAIN_ALL;
    cfg.desc.timeout_ms = 30000u;
    arena = calloc(1u, jsdk_desc_arena_size(&cfg.desc) + ARENA_GUARD);
    if (!arena) return 1;
    cfg.desc.arena      = arena;
    cfg.desc.arena_size = jsdk_desc_arena_size(&cfg.desc);

    st = jsdk_context_init(ctx, &cfg);
    step("context_init", st);
    if (st != JSDK_OK) return 1;

    memset(&jc, 0, sizeof jc);
    jc.node_id      = 1u;
    jc.initial_mode = JSDK_MODE_MIT;
    step("add_joint", jsdk_context_add_joint(ctx, &jc, &joint));
    if (!joint) return 1;

    /* --- 描述符下载 + 配置（阻塞式：靠 autotick 的时钟前进） --- */
    st = jsdk_context_desc_fetch(ctx);
    step("desc_fetch", st);
    if (st == JSDK_OK) step("configure", jsdk_context_configure(ctx));

    /* --- 读一个参数（走描述符 + 0x20） --- */
    if (jsdk_joint_param_get(joint, "axis0.motor.config.gear_ratio", &v) == JSDK_OK) {
        printf("  gear_ratio = %.3f (type %d)\n", (double)v.v.f32, (int)v.type);
    } else {
        printf("  param_get 失败：%s\n", jsdk_context_last_error(ctx));
        g_fail = 1;
    }

    /* --- 使能 + 一帧控制 + 取反馈 --- */
    st = jsdk_context_activate(ctx);
    step("activate", st);

    if (st == JSDK_OK) {
        uint64_t t;
        jsdk_joint_set_mit(joint, 0.05, 0.0, 2.0, 0.2, 0.0);
        for (t = 0u; t < 4u; ++t) {
            (void)jsdk_context_cycle_begin(ctx, t * cfg.period_ns);
            (void)jsdk_context_cycle_end(ctx);
        }
        if (jsdk_joint_get_feedback(joint, &fb) == JSDK_OK) {
            printf("  feedback: online=%d pos=%.4f rad enabled=%d\n",
                   fb.online, fb.pos, jsdk_joint_is_enabled(joint));
        }
        jsdk_context_deactivate(ctx);
    }

    jsdk_context_estop(ctx);                  /* 安全动作不需要 --yes 之类的确认 */
    jsdk_hal_close(hh);
    free(arena);

    printf("consumer: %s\n", g_fail ? "FAILED" : "OK");
    return g_fail;
}
