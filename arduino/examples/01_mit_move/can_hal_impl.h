/*
 * can_hal_impl.h — 把 SDK 接到你的 CAN 控制器上（**只有三个回调要填**）
 *
 * 本文件把 HAL 与 sketch 分开，是为了让"要改的地方"一眼可见：
 * 下面 3 个 TODO 就是你在 Arduino 上要写的全部传输层代码。
 *
 * 参考资料（都只需实现同一件事）：
 *   - MCP2515（SPI）：`arduino-CAN` / `mcp2515` 库的 `CAN.begin()` / `CAN.sendMsgBuf()`
 *   - ESP32：内置 TWAI（`twai_transmit` / `twai_receive`），无需外置芯片
 *   - STM32：HAL_FDCAN_AddMessageToTxFifoQ / HAL_FDCAN_GetRxMessage
 *   - 想省事：用 slcan 适配器 + `jsdk_hal_slcan_open()`（PC 侧），但那样就不是 Arduino 了
 *
 * ⚠ 三条铁律（写错了症状都很奇怪）：
 *   1. `send` / `recv` **不得阻塞** —— 它们在控制回路里被调用；
 *   2. `recv` 空队列必须返回 **0**（−1 的意思是"链路错误"，会让 SDK 把链路判为 down）；
 *   3. `now_ms` 必须**单调**（用 millis()；注意它 49 天回绕，SDK 内部用无符号差值处理）。
 */

#ifndef CAN_HAL_IMPL_H
#define CAN_HAL_IMPL_H

#include <Arduino.h>

#include "jsdk_can_amalgam.h"

/* --------------------------------------------------------------------------
 * 你自己的状态：一个接收环形缓冲 + 一个自由运行毫秒计数
 * ------------------------------------------------------------------------ */

#define RX_RING 16u

struct CanHal {
    jsdk_can_frame_t rx[RX_RING];
    volatile unsigned head;
    volatile unsigned tail;
    volatile uint32_t rx_dropped;
};

static CanHal g_can;

/* --------------------------------------------------------------------------
 * 1) send：把一帧交给控制器
 * ------------------------------------------------------------------------ */

static int can_send(void *user, const jsdk_can_frame_t *f)
{
    CanHal *c = (CanHal *)user;
    (void)c;

    /* TODO #1：把 f 写进你的 CAN 控制器发送邮箱。
       协议**恒为 29-bit 扩展帧**（f->flags & JSDK_FRAME_EXT 总是置位）。
       CAN FD 时 f->len 可达 64，且 (f->flags & JSDK_FRAME_BRS) 表示数据段变速。

       ⚠ 下面是**伪代码**（别照抄语法，各库 API 不同）；也注意本注释里不能写
         嵌套的块注释记号 —— C 的块注释不嵌套，写了会把注释提前结束掉（踩过）。

       MCP2515 例：CAN.sendMsgBuf(f->id, ext=1, (uint8_t)f->len, f->data);
       ESP32 例：  twai_message_t m; m.identifier = f->id; m.extd = 1;
                   m.data_length_code = f->len; memcpy(m.data, f->data, f->len);
                   twai_transmit(&m, 0);            // 0 = 不等待

       返回 0 = 已入队/已发出；非 0 = 失败或忙（SDK 会计入 tx_failed 并继续）。
       队列满时**不要在这里等** —— 丢一帧控制帧比卡住整条回路好。 */
    (void)f;
    return 0;
}

/* --------------------------------------------------------------------------
 * 2) recv：从环形缓冲取一帧（中断里入队，这里出队）
 * ------------------------------------------------------------------------ */

static int can_recv(void *user, jsdk_can_frame_t *f)
{
    CanHal *c = (CanHal *)user;

    if (c->tail == c->head) return 0;      /* ⚠ 空 = 0，不是 −1 */

    *f = c->rx[c->tail];
    c->tail = (c->tail + 1u) % RX_RING;
    return 1;
}

/* 中断回调（或轮询函数）里调用：把收到的帧塞进环形缓冲。 */
static inline void can_on_rx_isr(uint32_t id, const uint8_t *data, uint8_t len, uint8_t is_fd)
{
    unsigned next = (g_can.head + 1u) % RX_RING;

    if (next == g_can.tail) {               /* 满了：丢最旧的那一帧并计数 */
        g_can.tail = (g_can.tail + 1u) % RX_RING;
        g_can.rx_dropped++;
    }
    g_can.rx[g_can.head].id    = id;         /* ⚠ 裸 29-bit，不要带 EFF 标志位 */
    g_can.rx[g_can.head].len   = len;
    g_can.rx[g_can.head].flags = (uint8_t)(JSDK_FRAME_EXT
                                          | (is_fd ? JSDK_FRAME_FD : 0u));
    if (len) memcpy(g_can.rx[g_can.head].data, data, len);
    g_can.head = next;
}

/* --------------------------------------------------------------------------
 * 3) now_ms：单调毫秒时钟
 * ------------------------------------------------------------------------ */

static uint32_t can_now_ms(void *user)
{
    (void)user;
    return (uint32_t)millis();               /* TODO #3：换成你的自由运行计数器 */
}

/* --------------------------------------------------------------------------
 * 组装
 * ------------------------------------------------------------------------ */

static inline void can_hal_init(jsdk_can_hal_t *hal)
{
    memset(&g_can, 0, sizeof g_can);

    memset(hal, 0, sizeof *hal);
    hal->user   = &g_can;
    hal->send   = can_send;
    hal->recv   = can_recv;
    hal->now_ms = can_now_ms;

    /* TODO：初始化你的控制器（例：CAN.begin(1000000) / twai_start()），
       并把 can_on_rx_isr 挂到接收中断上。 */

    Serial.println(F("CAN 控制器已初始化（TODO 还没填？看 can_hal_impl.h）"));
}

#endif /* CAN_HAL_IMPL_H */
