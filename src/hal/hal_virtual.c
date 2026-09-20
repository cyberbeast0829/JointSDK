/**
 * @file    hal_virtual.c
 * @brief   虚拟 CAN 总线后端（CI / 离线仿真）
 *
 * 三层结构：
 *
 * ```
 *   SDK ──send()──> [虚拟总线] ──> 设备模型(sim_device) ──> 出站队列
 *   SDK <──recv()── [RX 环]  <── 模型应答 / jsdk_hal_virtual_inject() ← 测试注入
 *   SDK <──now_ms()─ [虚拟时钟] <── jsdk_hal_virtual_advance_ms()
 * ```
 *
 * @par 为什么 `send` 会同步产生应答
 *  真实总线上，设备在收到帧后**下一个周期**才回。虚拟后端为了测试可确定性，
 *  在 `send()` 内就把模型对该帧的**即时应答**（MIT 响应、查询响应、参数响应、
 *  ACK）排入 RX 环。周期性行为（心跳、`break_timeout`、描述符分块）不走
 *  这条路径，必须由 `jsdk_hal_virtual_advance_ms()` 驱动——这样“同步应答”
 *  与“异步周期任务”的边界与固件一致，测试可以精确控制时间。
 *
 * @par 线程安全
 *  本后端**不加锁**，仅用于单线程测试。多线程场景请用真实后端或自行加锁。
 */

#include "jsdk_hal_builtin.h"

#include <stdlib.h>
#include <string.h>

#include "hal_handle.h"
#include "sim_device.h"
#include "cb_frame.h"

/* --------------------------------------------------------------------------
 * 内部结构
 * ------------------------------------------------------------------------ */

#define JSDK_VHAL_RX_CAP 2048u   /**< RX 环容量（注入 + 模型应答共用一个队列） */

/**
 * 虚拟后端句柄。`base` 必须是**首个成员**：`jsdk_hal_close()` 通过
 * `base.destroy` 分派，`JSDK_HAL_CAST()` 靠首成员地址相同做反向转换。
 */
typedef struct {
    struct jsdk_hal_handle base;

    sim_bus_t        sim;
    uint32_t         now_ms;
    uint32_t         tx_fail;       /**< 剩余失败次数；UINT32_MAX = 永久 */
    uint32_t         injected;      /**< 注入帧总数 */
    uint8_t          autotick;      /**< 1 = now_ms 自行推进 1 ms（见 set_autotick） */

    jsdk_can_frame_t rx[JSDK_VHAL_RX_CAP];
    uint32_t         rx_head;
    uint32_t         rx_tail;
    uint32_t         rx_dropped;    /**< 因 RX 环满而丢弃的帧数 */

    /* 捕获：SDK 发出的帧（供测试断言） */
    jsdk_can_frame_t cap[256];
    uint32_t         cap_head;
    uint32_t         cap_tail;

    /* 从模型/注入处转移到 RX 环的中转缓冲（send 时同步搬运，避免嵌套） */
    int              in_emit;
} vhal_t;

/** 由不透明句柄取回本后端结构（`base` 是首成员）。 */
static vhal_t *vh(jsdk_hal_handle_t *h)
{
    return JSDK_HAL_CAST(vhal_t, h);
}


/* --------------------------------------------------------------------------
 * RX / 捕获队列
 * ------------------------------------------------------------------------ */

static void vhal_rx_push(vhal_t *h, const jsdk_can_frame_t *f)
{
    uint32_t next = (h->rx_tail + 1u) % JSDK_VHAL_RX_CAP;
    if (next == h->rx_head) {
        h->rx_dropped++;
        return;
    }
    h->rx[h->rx_tail] = *f;
    h->rx_tail = next;
}

/** 把设备模型刚生成的帧搬进 RX 环。 */
static void vhal_drain_sim_tx(vhal_t *h)
{
    jsdk_can_frame_t f;
    while (sim_tx_pop(&h->sim, &f)) {
        vhal_rx_push(h, &f);
    }
}

static void vhal_capture_push(vhal_t *h, const jsdk_can_frame_t *f)
{
    uint32_t next = (h->cap_tail + 1u) % 256u;
    if (next == h->cap_head) {
        h->cap_head = (h->cap_head + 1u) % 256u;   /* 覆盖最旧 */
    }
    h->cap[h->cap_tail] = *f;
    h->cap_tail = next;
}

/* --------------------------------------------------------------------------
 * HAL 回调
 * ------------------------------------------------------------------------ */

static int vhal_send(void *user, const jsdk_can_frame_t *f)
{
    vhal_t *h = vh((jsdk_hal_handle_t *)user);

    if (!h || !f) return -1;
    if (!(f->flags & JSDK_FRAME_EXT)) return -1;   /* 本协议恒为扩展帧 */

    if (h->tx_fail != 0u) {
        if (h->tx_fail != 0xFFFFFFFFu) h->tx_fail--;
        return -1;                                  /* 模拟发送失败 */
    }

    vhal_capture_push(h, f);

    /* 交给设备模型；它可能直接产生应答 */
    sim_rx(&h->sim, f);
    if (h->in_emit == 0) {
        h->in_emit = 1;
        vhal_drain_sim_tx(h);
        h->in_emit = 0;
    }
    return 0;
}

static int vhal_recv(void *user, jsdk_can_frame_t *f)
{
    vhal_t *h = vh((jsdk_hal_handle_t *)user);

    if (!h || !f) return -1;
    if (h->rx_head == h->rx_tail) return 0;
    *f = h->rx[h->rx_head];
    h->rx_head = (h->rx_head + 1u) % JSDK_VHAL_RX_CAP;
    return 1;
}

static uint32_t vhal_now_ms(void *user)
{
    vhal_t *h = vh((jsdk_hal_handle_t *)user);

    if (!h) return 0u;

    /* 自动推进模式：每次被问到时间就前进 1 ms 并跑一次周期任务。
       语义与真实 HAL 的自由运行计数器一致，于是 SDK 里那些"等时间流逝"
       的阻塞 API（configure / discover / calibrate）在仿真下也能正常结束。
       ⚠ 默认关闭：测试需要冻结时钟来验证非阻塞路径。 */
    if (h->autotick) {
        h->now_ms += 1u;
        sim_tick(&h->sim, h->now_ms);
        vhal_drain_sim_tx(h);
    }
    return h->now_ms;
}

static void vhal_on_error(void *user, int kind, uint32_t detail)
{
    (void)user; (void)kind; (void)detail;   /* 虚拟后端不上报链路错误 */
}

static int vhal_bus_status(void *user, uint32_t *flags)
{
    (void)user;
    if (!flags) return -1;
    *flags = JSDK_HAL_BUS_OK;
    return 0;
}

/** 释放句柄：虚拟后端只需归还模型与自身（真实后端在这里关 fd）。 */
static void vhal_destroy(struct jsdk_hal_handle *base)
{
    vhal_t *h = JSDK_HAL_CAST(vhal_t, base);
    if (!h) return;
    sim_bus_free(&h->sim);
    free(h);
}

/* --------------------------------------------------------------------------
 * 公开 API
 * ------------------------------------------------------------------------ */

jsdk_status_t jsdk_hal_virtual_open(jsdk_can_hal_t *hal, jsdk_hal_handle_t **out,
                                    const char *node_spec)
{
    vhal_t *h;

    if (!hal || !out) return JSDK_ERR_INVALID_ARG;
    *out = NULL;                        /* 失败时调用方看到的一定是 NULL */

    h = (vhal_t *)calloc(1u, sizeof *h);
    if (!h) return JSDK_ERR_NO_MEMORY;

    /* 先按规格里的 ';' 数量确定节点数，再一次初始化（避免重复 init） */
    {
        size_t n = 1u;
        if (node_spec && node_spec[0]) {
            size_t i;
            for (i = 0u; node_spec[i]; ++i) {
                if (node_spec[i] == ';') n++;
            }
            if (n > SIM_MAX_NODES) {
                free(h);
                return JSDK_ERR_INVALID_ARG;   /* 超出上限：明确报错而不是静默截断 */
            }
        }
        sim_bus_init(&h->sim, n);
    }

    if (node_spec && node_spec[0] && sim_configure(&h->sim, node_spec) != 0) {
        sim_bus_free(&h->sim);
        free(h);
        return JSDK_ERR_INVALID_ARG;
    }

    h->base.kind    = JSDK_HAL_KIND_VIRTUAL;
    h->base.destroy = vhal_destroy;
    h->tx_fail      = 0u;

    /* ⚠ 虚拟时钟从 **1 ms** 起步而不是 0：固件用 `last_cmd_time_[i] == 0`
       表示“从未收到过 is_ctrl 帧”，而 `b->now_ms` 在 t = 0 时也是 0，
       会让第一条控制帧无法武装超时计时器。从 1 开始就无歧义了。 */
    h->now_ms = 1u;
    h->sim.now_ms = 1u;
    h->sim.last_tick_ms = 1u;

    hal->user       = h;
    hal->send       = vhal_send;
    hal->recv       = vhal_recv;
    hal->now_ms     = vhal_now_ms;
    hal->on_error   = vhal_on_error;
    hal->bus_status = vhal_bus_status;

    *out = &h->base;
    return JSDK_OK;
}

jsdk_status_t jsdk_hal_virtual_inject(jsdk_hal_handle_t *h,
                                      const jsdk_can_frame_t *f)
{
    vhal_t *v = vh(h);

    if (!v || !f) return JSDK_ERR_INVALID_ARG;
    if (!(f->flags & JSDK_FRAME_EXT)) return JSDK_ERR_INVALID_ARG;
    if (f->len > 64u) return JSDK_ERR_INVALID_ARG;

    vhal_rx_push(v, f);
    v->injected++;
    return JSDK_OK;
}

int jsdk_hal_virtual_capture(jsdk_hal_handle_t *h, jsdk_can_frame_t *f)
{
    vhal_t *v = vh(h);

    if (!v || !f) return 0;
    if (v->cap_head == v->cap_tail) return 0;
    *f = v->cap[v->cap_head];
    v->cap_head = (v->cap_head + 1u) % 256u;
    return 1;
}

void jsdk_hal_virtual_set_tx_fail(jsdk_hal_handle_t *h, int fail)
{
    vhal_t *v = vh(h);
    if (!v) return;
    v->tx_fail = (fail < 0) ? 0xFFFFFFFFu : (uint32_t)fail;
}

uint32_t jsdk_hal_virtual_dropped(const jsdk_hal_handle_t *h)
{
    const vhal_t *v = JSDK_HAL_CAST(const vhal_t, h);
    if (!v) return 0u;
    return v->rx_dropped + v->sim.txq_dropped;
}

void jsdk_hal_virtual_advance_ms(jsdk_hal_handle_t *h, uint32_t ms)
{
    vhal_t *v = vh(h);
    if (!v) return;
    v->now_ms += ms;
    sim_tick(&v->sim, v->now_ms);
    vhal_drain_sim_tx(v);
}

void jsdk_hal_virtual_set_autotick(jsdk_hal_handle_t *h, int enable)
{
    vhal_t *v = vh(h);
    if (!v) return;
    v->autotick = enable ? 1u : 0u;
}

/* --------------------------------------------------------------------------
 * 供测试直接访问内部模型（避免把 sim_bus_t 暴露到公共头文件）
 * ------------------------------------------------------------------------ */

sim_bus_t *jsdk_hal_virtual_sim(jsdk_hal_handle_t *h);
sim_bus_t *jsdk_hal_virtual_sim(jsdk_hal_handle_t *h)
{
    vhal_t *v = vh(h);
    return v ? &v->sim : NULL;
}
