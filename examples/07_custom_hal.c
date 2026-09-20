/**
 * @file    07_custom_hal.c
 * @brief   **不用任何内置后端**：自己实现 `jsdk_can_hal_t`（MCU 客户的入口）
 *
 * 这是唯一一个**不包含 `jsdk_hal_builtin.h`** 的示例：SDK 在这里就是一个纯协议栈，
 * 传输层由你提供。下面的实现是一块**内存里的队列**，真机上换成 CAN 外设即可
 * （`HAL_CAN_AddTxMessage` / `FDCAN_AddMessageToTxFifoQ` / MCP2515 的 SPI 写 …）。
 *
 * 必须实现的只有三个回调：
 *   - `send(user, frame)`  ：把一帧交给总线。**不得阻塞**（非 0 = 失败/忙）
 *   - `recv(user, frame)`  ：取一帧。**不得阻塞**；1 = 取到，0 = 暂无，<0 = 链路错误
 *   - `now_ms(user)`       ：**单调**毫秒时钟（看门狗/超时/新鲜度全基于它）
 * 可选：`on_error`、`bus_status`（填 `JSDK_HAL_BUS_*` 位）。
 *
 * 三个最容易写错的点（代码里都有标注）：
 *   1. `recv()` 空队列要返回 **0**（不是 −1：−1 意味着"链路错误"，会让 SDK
 *      记一次 link_error 并把链路判为 down）；
 *   2. 两个回调**都不能阻塞** —— 它们在高频控制周期里被调用；
 *   3. 帧 ID 是**裸的 29 bit**（不含 EFF 标志位），扩展帧信息在 `flags` 里。
 *
 * MCU 上还需要知道的两件事：
 *   - **arena 用静态数组**（本例就是），配合 `RETAIN_FILTERED`：只留自己用到的
 *     十几条路径，实测 < 1.5 KB。`RETAIN_ALL`（594 端点）≈ 25 KB RAM，MCU 上别用。
 *   - **描述符不必走 CAN**：离线把设备 JSON 转成常量数组，上电用
 *     `jsdk_context_desc_import_raw()` 直接喂进去（示例 05 演示了这条路线）。
 *
 * 运行：
 *     ./build/examples/07_custom_hal
 */

#include <stdio.h>
#include <string.h>

#include "joint_sdk.h"        /* ⚠ 只有这一个头 ✓ */

/* ==========================================================================
 * 1. 你的总线（示例用内存队列冒充；真机上是 CAN 外设）
 * ======================================================================== */

#define BUS_DEPTH 16u

typedef struct {
    jsdk_can_frame_t q[BUS_DEPTH];
    unsigned         head;
    unsigned         tail;
    uint32_t         ticks_ms;    /* 自由运行计数器（示例里每调用一次 +1 ms） */
    uint32_t         tx_count;
} fake_bus_t;

static int bus_send(void *user, const jsdk_can_frame_t *f)
{
    fake_bus_t *b = (fake_bus_t *)user;
    unsigned    next = (b->head + 1u) % BUS_DEPTH;

    /* ⚠ 队列满 → 直接返回失败，**不要在这里等**：控制帧丢了比卡住整条回路好。 */
    if (next == b->tail) return -1;

    b->q[b->head] = *f;
    b->head = next;
    b->tx_count++;
    return 0;
}

static int bus_recv(void *user, jsdk_can_frame_t *f)
{
    fake_bus_t *b = (fake_bus_t *)user;

    /* ⚠ 空队列 = 0（"当前没帧"，正常状态）；只有真出错才返回负数。 */
    if (b->tail == b->head) return 0;

    *f = b->q[b->tail];
    b->tail = (b->tail + 1u) % BUS_DEPTH;
    return 1;
}

static uint32_t bus_now_ms(void *user)
{
    fake_bus_t *b = (fake_bus_t *)user;

    /* ⚠ 必须单调。真机直接返回自由运行计数器（HAL_GetTick() 之类）即可。 */
    return b->ticks_ms++;
}

static int bus_status(void *user, uint32_t *flags)
{
    (void)user;
    *flags = JSDK_HAL_BUS_OK;
    return 0;
}

/* ==========================================================================
 * 2. 零 malloc：上下文、配置、arena 全部静态（MCU 的标准做法）
 *
 * ⚠ `cfg` 必须与 arena **同寿命**：`jsdk_context_init()` 会保存
 *   `&cfg.desc.arena_used` 以便后续写回。若把 cfg 放在某个函数的栈上，那函数
 *   返回之后 SDK 就会往已失效的栈帧里写（ASAN 真抓到过，见 DESIGN v0.17）。
 * ======================================================================== */

static fake_bus_t             s_bus;
static jsdk_context_config_t  s_cfg;
static unsigned char          s_arena[2048];       /* RETAIN_FILTERED 够用 */
static jsdk_context_storage_t s_store;             /* JSDK_CONTEXT_MAX_SIZE 字节 */

/* 只想保留这几条路径（RETAIN_FILTERED）。数组必须与 s_cfg 同寿命 ——
   `jsdk_desc_config_t.filter_paths` 存的是**指向它的指针**，不是拷贝。 */
static const char *const      k_filters[] = {
    "axis0.config.can.node_id",
    "axis0.motor.config.gear_ratio",
    "axis0.controller.config.mit_max_pos"
};

int main(void)
{
    jsdk_can_hal_t      hal;
    jsdk_context_t     *ctx = (jsdk_context_t *)&s_store;
    jsdk_joint_config_t jc;
    jsdk_joint_t       *joint = NULL;
    jsdk_status_t       st;
    jsdk_bus_state_t    bs;

    printf("=== 07 自己实现 HAL（不用任何内置后端）===\n");

    memset(&s_bus, 0, sizeof s_bus);

    /* --- 组装 vtable ---------------------------------------------------- */
    memset(&hal, 0, sizeof hal);
    hal.user       = &s_bus;
    hal.send       = bus_send;
    hal.recv       = bus_recv;
    hal.now_ms     = bus_now_ms;
    hal.bus_status = bus_status;   /* 可选 */
    hal.on_error   = NULL;         /* 可选 */

    /* --- 上下文 --------------------------------------------------------- */
    jsdk_context_config_default(&s_cfg);
    s_cfg.hal              = hal;
    s_cfg.master_id        = 7u;                  /* 主站号，≠ 0 */
    s_cfg.is_fd            = 0u;                  /* 本例按 Classic 8 B 帧 */
    s_cfg.period_ns        = 1000000u;            /* 1 kHz */
    s_cfg.desc.mode        = JSDK_DESC_DYNAMIC;
    s_cfg.desc.retain      = JSDK_DESC_RETAIN_FILTERED;
    /* ⚠ `RETAIN_FILTERED` **必须**给至少一条路径（`filter_paths` 是"指向指针数组的
       指针"，数组本身要活到 configure() 之后），否则整个配置都不合法（`INVALID_ARG`）——
       这是有意的：没给 filter 就只剩空表。 */
    s_cfg.desc.filter_paths = k_filters;
    s_cfg.desc.filter_count = 3u;
    s_cfg.desc.timeout_ms  = 1000u;
    s_cfg.desc.arena       = s_arena;
    s_cfg.desc.arena_size  = sizeof s_arena;

    st = jsdk_context_init(ctx, &s_cfg);
    printf("  %-24s %s\n", "context_init", jsdk_status_string(st));
    if (st != JSDK_OK) return 1;

    memset(&jc, 0, sizeof jc);
    jc.node_id      = 1u;
    jc.initial_mode = JSDK_MODE_MIT;
    st = jsdk_context_add_joint(ctx, &jc, &joint);
    printf("  %-24s %s\n", "add_joint", jsdk_status_string(st));
    if (st != JSDK_OK) return 1;

    /* --- 描述符：真机走 CAN，MCU 更常见的是喂常量 JSON -------------------
       这里给一段最小 JSON，证明"不经 CAN 也能建起端点表"这条路是通的
       （真机上由离线脚本从 Firmware/autogen/endpoints.hpp 生成，可带全部 594 条）。 */
    {
        static const char k_json[] =
            "[{\"name\":\"axis0\",\"type\":\"object\",\"members\":[{"
              "\"name\":\"config\",\"type\":\"object\",\"members\":[{"
                "\"name\":\"can\",\"type\":\"object\",\"members\":[{"
                  "\"name\":\"node_id\",\"type\":\"uint32\",\"access\":\"rw\",\"id\":180"
                "}]}]}]}]";
        jsdk_desc_hint_t hint;

        hint.crc        = 0u;      /* 真机上：从缓存头读回上次下载得到的 crc */
        hint.fw_version = 0u;      /* 真机上：QUERY_DEVICE_INFO(0x46) 的 fw 字段 */
        st = jsdk_context_desc_import_raw(ctx, k_json, sizeof k_json - 1u, &hint);
        printf("  %-24s %s\n", "desc_import_raw", jsdk_status_string(st));

        if (st == JSDK_OK) {
            uint16_t ep_id = 0u;
            jsdk_ep_type_t type;
            uint8_t access = 0u;

            st = jsdk_endpoint_lookup(ctx, "axis0.config.can.node_id",
                                      &ep_id, &type, &access);
            printf("  %-24s %s", "endpoint_lookup",
                   jsdk_status_string(st));
            if (st == JSDK_OK) {
                printf("（id=%u type=%d access=0x%X）", (unsigned)ep_id,
                       (int)type, (unsigned)access);
            }
            printf("\n");
        }
    }

    /* --- 让总线真的动起来：发一条 ESTOP ---------------------------------
       ESTOP 是广播（MsgType 0xC0），**不需要**端点表、不需要使能、不期待响应 ——
       所以它在"只有 HAL、没有设备"的情况下也能走完整条发送路径，
       正好用来验证 HAL 接对了。

       真机上这里换成你的 1 kHz 任务/中断：
           cycle_begin() → 读反馈/设目标 → cycle_end()
       （控制帧只有在使能序列走完之后才会发出，所以本例不演示 MIT：
         没有真实设备应答时使能序列本来就不该走完。） */
    jsdk_context_estop(ctx);
    printf("  %-24s tx_count=%u\n", "estop → 1 帧", (unsigned)s_bus.tx_count);

    /* --- 看看发出去了什么（真机上是逻辑分析仪上的波形）----------------- */
    {
        jsdk_can_frame_t f;
        if (bus_recv(&s_bus, &f) == 1) {
            printf("  帧：id=0x%08lX len=%u flags=0x%X（29-bit 扩展帧 + 广播）\n",
                   (unsigned long)f.id, (unsigned)f.len, (unsigned)f.flags);
        }
    }

    /* --- 跑 10 个周期：验证 cycle API 在你自己的时钟下能正常转 --------- */
    {
        unsigned k;
        for (k = 0u; k < 10u; ++k) {
            (void)jsdk_context_cycle_begin(ctx, (uint64_t)k * s_cfg.period_ns);
            (void)jsdk_context_cycle_end(ctx);
        }
    }
    if (jsdk_context_get_bus_state(ctx, &bs) == JSDK_OK) {
        printf("  10 个周期后：tx=%u rx=%u tx_failed=%u dropped=%u link_errors=%u\n",
               (unsigned)bs.tx_frames, (unsigned)bs.rx_frames, (unsigned)bs.tx_failed,
               (unsigned)bs.rx_dropped, (unsigned)bs.link_errors);
    }

    printf("  本例的总线是内存队列、没有设备模型，所以只看\"帧有没有出去\"；\n"
           "  真正的协议对话需要真设备（或 tests/ 里的仿真设备）。\n");
    printf("  资源用量自查：arena 用了 %u / %u 字节，上下文 %u 字节（都是静态存储）。\n",
           (unsigned)s_cfg.desc.arena_used, (unsigned)s_cfg.desc.arena_size,
           (unsigned)sizeof s_store);

    printf("done\n");
    return 0;
}
