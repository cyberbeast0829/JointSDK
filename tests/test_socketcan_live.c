/**
 * @file    test_socketcan_live.c
 * @brief   Linux SocketCAN 后端的**真链路**冒烟测试（vcan 或真实 CAN 接口）
 *
 * 与 `test_hal.c` 的区别（两者互补，都不能省）：
 *   - `test_hal.c`：离线。验 slcan 编解码、句柄生命周期、错误码契约。
 *   - **本文件**：走**内核**。`open → bind → write → 对端 read` 真的经过
 *     CAN 子系统，所以能抓到"只在真链路上才暴露"的问题：`CAN_RAW_FD_FRAMES`
 *     有没有真的设上去、FD/Classic 帧的 MTU 判断、`ERR_QUEUE` 之外的错误路径、
 *     以及对端字节级是否与我们发的一致。
 *
 * 对端**故意**直接用 `socket(PF_CAN, ...)` 创建，不经本 SDK 的 HAL —— 否则就是
 * 自己跟自己握手，验证不了任何东西。
 *
 * ## 用法
 * ```
 *     JSDK_SC_IFACE=vcan0 ./test_socketcan_live        # 推荐：先 modprobe vcan
 *     JSDK_SC_IFACE=can0  ./test_socketcan_live        # 真硬件
 * ```
 * 未设置 `JSDK_SC_IFACE` 时打印"跳过"并以 0 退出（便于脚本无条件调用）。
 *
 * ## 仍**无法**在这里覆盖的（见 PORTING §7.5.3 人工清单）
 *   - 位定时/采样点、终端电阻、实际误码率；
 *   - `CAN_ERR_BUSOFF`/error-passive 等错误帧：vcan 上造不出来，只有真总线
 *     （拔线、短接）才会出现；
 *   - 设备侧协议行为（那是设备的事，不是 HAL 的）。
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "joint_sdk.h"
#include "jsdk_hal_builtin.h"

/* --------------------------------------------------------------------------
 * 断言
 * ------------------------------------------------------------------------ */

static unsigned g_checks;
static unsigned g_fail;

#define CHECK(cond)                                                          \
    do {                                                                     \
        g_checks++;                                                          \
        if (!(cond)) {                                                       \
            g_fail++;                                                        \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
        }                                                                    \
    } while (0)

#define CHECK_EQ(a, b)                                                       \
    do {                                                                     \
        g_checks++;                                                          \
        if ((long long)(a) != (long long)(b)) {                              \
            g_fail++;                                                        \
            printf("  FAIL %s:%d: %s == %s (%lld != %lld)\n", __FILE__,       \
                   __LINE__, #a, #b, (long long)(a), (long long)(b));        \
        }                                                                    \
    } while (0)

/* --------------------------------------------------------------------------
 * 对端 socket（裸内核 API）
 * ------------------------------------------------------------------------ */

static int peer_open(const char *ifname, int fd_mode)
{
    int s;
    struct ifreq ifr;
    struct sockaddr_can addr;
    int v;

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) return -1;

    memset(&ifr, 0, sizeof ifr);
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { close(s); return -1; }

    memset(&addr, 0, sizeof addr);
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof addr) < 0) { close(s); return -1; }

    if (fd_mode) {
        v = 1;
        if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &v, sizeof v) < 0) {
            close(s);
            return -1;
        }
    }

    /* 不要收到自己发的帧：否则断言要区分"回声"和"对端发的" */
    v = 0;
    (void)setsockopt(s, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &v, sizeof v);
    return s;
}

/** 对端等一帧（毫秒级超时）；返回 1 = 收到，0 = 超时，-1 = 出错。 */
static int peer_wait(int s, struct canfd_frame *out, int timeout_ms)
{
    struct pollfd p;
    int rc;

    memset(&p, 0, sizeof p);
    p.fd = s;
    p.events = POLLIN;
    rc = poll(&p, 1, timeout_ms);
    if (rc <= 0) return rc;                       /* 0 = 超时，<0 = 错 */

    rc = (int)read(s, out, sizeof *out);
    if (rc < (int)CAN_MTU) return -1;
    return 1;
}

/** 对端发一帧（用内核结构）。 */
static int peer_send(int s, const struct canfd_frame *f, int fd_frame)
{
    size_t n = fd_frame ? sizeof(struct canfd_frame) : sizeof(struct can_frame);
    return (int)write(s, f, n);
}

/* --------------------------------------------------------------------------
 * 用例
 * ------------------------------------------------------------------------ */

/** 1. 打开 + 链路状态。 */
static void test_open(const char *iface)
{
    jsdk_can_hal_t hal;
    jsdk_hal_handle_t *h = NULL;
    uint32_t flags = 0xFFFFFFFFu;

    printf("[1] open %s (FD, 1M/5M) + bus_status\n", iface);
    memset(&hal, 0, sizeof hal);

    CHECK_EQ(jsdk_hal_socketcan_open(&hal, &h, iface, 1000000u, 5000000u), JSDK_OK);
    CHECK(h != NULL);
    if (!h) return;

    /* 本文件**只用公共 API**（像客户那样用）：句柄类别的内部查询留给 test_hal.c */
    CHECK(hal.send != NULL);
    CHECK(hal.recv != NULL);

    /* `bus_status` 是可选的；实现了就必须返回 0 且给出 JSDK_HAL_BUS_OK */
    CHECK(hal.bus_status != NULL);
    if (hal.bus_status) {
        CHECK_EQ(hal.bus_status(hal.user, &flags), 0);
        CHECK((flags & JSDK_HAL_BUS_OK) != 0u);
    }
    CHECK(hal.now_ms != NULL);
    printf("      now_ms = %u, bus flags = 0x%X\n",
           (unsigned)(hal.now_ms ? hal.now_ms(hal.user) : 0u), (unsigned)flags);

    CHECK_EQ(jsdk_hal_close(h), JSDK_OK);
}

/** 2. 我们发 → 对端收（Classic 8 B），字节级比对。 */
static void test_tx_classic(const char *iface)
{
    jsdk_can_hal_t hal;
    jsdk_hal_handle_t *h = NULL;
    jsdk_can_frame_t f;
    struct canfd_frame rx;
    int peer;
    unsigned i;

    printf("[2] classic 8 B: SDK → 对端\n");
    memset(&hal, 0, sizeof hal);
    peer = peer_open(iface, 1);
    if (peer < 0) { printf("  FAIL peer socket: %s\n", strerror(errno)); g_fail++; return; }

    if (jsdk_hal_socketcan_open(&hal, &h, iface, 1000000u, 5000000u) != JSDK_OK) {
        g_fail++; close(peer); return;
    }

    memset(&f, 0, sizeof f);
    f.id    = 0x123u;                       /* 标准帧：本协议其实用扩展帧，两种都测 */
    f.flags = 0u;
    f.len   = 8u;
    for (i = 0u; i < 8u; ++i) f.data[i] = (uint8_t)(0xA0u + i);

    CHECK_EQ(hal.send(hal.user, &f), 0);

    if (peer_wait(peer, &rx, 1000) == 1) {
        CHECK_EQ(rx.can_id, 0x123u);
        CHECK_EQ(rx.len, 8u);
        CHECK(memcmp(rx.data, f.data, 8u) == 0);
        printf("      peer got id=0x%X len=%u data=%02X…\n",
               (unsigned)rx.can_id, (unsigned)rx.len, rx.data[0]);
    } else {
        printf("  FAIL peer 没收到帧（vcan 是否 up？）\n");
        g_fail++; g_checks++;
    }

    jsdk_hal_close(h);
    close(peer);
}

/** 3. 对端发 → 我们收（扩展帧，协议实况）。 */
static void test_rx_extended(const char *iface)
{
    jsdk_can_hal_t hal;
    jsdk_hal_handle_t *h = NULL;
    jsdk_can_frame_t f;
    struct canfd_frame tx;
    int peer;

    printf("[3] ext 29-bit: 对端 → SDK\n");
    memset(&hal, 0, sizeof hal);
    peer = peer_open(iface, 1);
    if (peer < 0) { g_fail++; return; }
    if (jsdk_hal_socketcan_open(&hal, &h, iface, 1000000u, 5000000u) != JSDK_OK) {
        g_fail++; close(peer); return;
    }

    memset(&tx, 0, sizeof tx);
    tx.can_id = (0x0A123456u & 0x1FFFFFFFu) | CAN_EFF_FLAG;
    tx.len    = 8u;
    tx.data[0] = 0x11;
    tx.data[7] = 0x77;
    CHECK(peer_send(peer, &tx, 0) > 0);

    memset(&f, 0, sizeof f);
    if (hal.recv(hal.user, &f) == 1) {
        CHECK_EQ(f.id, 0x0A123456u & 0x1FFFFFFFu);
        CHECK((f.flags & JSDK_FRAME_EXT) != 0u);
        CHECK((f.flags & JSDK_FRAME_FD) == 0u);
        CHECK_EQ(f.len, 8u);
        CHECK_EQ(f.data[0], 0x11u);
        CHECK_EQ(f.data[7], 0x77u);
        printf("      SDK got id=0x%08X len=%u flags=0x%X\n",
               (unsigned)f.id, (unsigned)f.len, (unsigned)f.flags);
    } else {
        printf("  FAIL SDK 没收到对端的帧\n");
        g_fail++; g_checks++;
    }

    jsdk_hal_close(h);
    close(peer);
}

/** 4. FD 双向：64 B 发出去、32 B 收进来（验证 CAN_RAW_FD_FRAMES 真的设上了）。 */
static void test_fd_both_ways(const char *iface)
{
    jsdk_can_hal_t hal;
    jsdk_hal_handle_t *h = NULL;
    jsdk_can_frame_t f;
    struct canfd_frame rx, tx;
    int peer;
    unsigned i;

    printf("[4] CAN FD 双向（64 B 出 / 32 B 入，带 BRS）\n");
    memset(&hal, 0, sizeof hal);
    peer = peer_open(iface, 1);
    if (peer < 0) { g_fail++; return; }
    if (jsdk_hal_socketcan_open(&hal, &h, iface, 1000000u, 5000000u) != JSDK_OK) {
        g_fail++; close(peer); return;
    }

    /* --- 出：64 字节（FD 上限）--- */
    memset(&f, 0, sizeof f);
    f.id    = 0x1E000002u;
    f.flags = (uint8_t)(JSDK_FRAME_EXT | JSDK_FRAME_FD | JSDK_FRAME_BRS);
    f.len   = 64u;
    for (i = 0u; i < 64u; ++i) f.data[i] = (uint8_t)i;

    CHECK_EQ(hal.send(hal.user, &f), 0);
    if (peer_wait(peer, &rx, 1000) == 1) {
        CHECK_EQ(rx.can_id & 0x1FFFFFFFu, 0x1E000002u);
        CHECK((rx.can_id & CAN_EFF_FLAG) != 0u);
        CHECK_EQ(rx.len, 64u);
        CHECK((rx.flags & CANFD_BRS) != 0u);
        CHECK_EQ(rx.data[0], 0u);
        CHECK_EQ(rx.data[63], 63u);
        printf("      peer got FD len=%u flags=0x%X（BRS 已置）\n",
               (unsigned)rx.len, (unsigned)rx.flags);
    } else {
        printf("  FAIL peer 没收到 FD 帧\n");
        g_fail++; g_checks++;
    }

    /* --- 入：32 字节 --- */
    memset(&tx, 0, sizeof tx);
    tx.can_id = 0x1E000003u | CAN_EFF_FLAG;
    tx.len    = 32u;
    tx.flags  = CANFD_BRS;
    for (i = 0u; i < 32u; ++i) tx.data[i] = (uint8_t)(0x80u + i);
    CHECK(peer_send(peer, &tx, 1) > 0);

    memset(&f, 0, sizeof f);
    if (hal.recv(hal.user, &f) == 1) {
        CHECK_EQ(f.id, 0x1E000003u);
        CHECK((f.flags & JSDK_FRAME_FD) != 0u);
        CHECK((f.flags & JSDK_FRAME_BRS) != 0u);
        CHECK((f.flags & JSDK_FRAME_EXT) != 0u);
        CHECK_EQ(f.len, 32u);
        CHECK_EQ(f.data[31], (uint8_t)(0x80u + 31u));
        printf("      SDK got FD len=%u flags=0x%X\n",
               (unsigned)f.len, (unsigned)f.flags);
    } else {
        printf("  FAIL SDK 没收到 FD 帧（CAN_RAW_FD_FRAMES 没设上？）\n");
        g_fail++; g_checks++;
    }

    jsdk_hal_close(h);
    close(peer);
}

/**
 * 5. Classic 模式下对端发 FD 帧：内核会丢弃/拒绝。
 *
 * 要的是"**不崩、不卡死，且链路仍可用**"：Classic socket 没开
 * `CAN_RAW_FD_FRAMES` 时，FD 帧根本不会进到应用层。
 */
static void test_fd_frame_on_classic_socket(const char *iface)
{
    jsdk_can_hal_t hal;
    jsdk_hal_handle_t *h = NULL;
    jsdk_can_frame_t f;
    struct canfd_frame tx;
    int peer;

    printf("[5] Classic socket 收到 FD 帧：应被内核挡住且链路仍可用\n");
    memset(&hal, 0, sizeof hal);
    peer = peer_open(iface, 1);                  /* 对端仍然是 FD 能力 */
    if (peer < 0) { g_fail++; return; }
    /* SDK 这一侧按 Classic 开（data_bitrate = 0） */
    if (jsdk_hal_socketcan_open(&hal, &h, iface, 1000000u, 0u) != JSDK_OK) {
        g_fail++; close(peer); return;
    }

    memset(&tx, 0, sizeof tx);
    tx.can_id = 0x1E000004u | CAN_EFF_FLAG;
    tx.len    = 16u;
    tx.data[0] = 0x55;
    CHECK(peer_send(peer, &tx, 1) > 0);

    memset(&f, 0, sizeof f);
    /* FD 帧应被内核挡掉：recv 返回"无帧"(0) 或错误(-1)，但**绝不能**是
       一条被截断/误认成 Classic 的帧。 */
    {
        int rc = hal.recv(hal.user, &f);
        CHECK(rc == 0 || rc == -1);
        if (rc == 1) {
            printf("  FAIL Classic socket 收到了一条帧（不应发生）\n");
            g_fail++;
        }
    }

    /* 链路仍可用：随后一条普通 Classic 帧必须能收到 */
    memset(&tx, 0, sizeof tx);
    tx.can_id = 0x1E000005u | CAN_EFF_FLAG;
    tx.len    = 4u;
    tx.data[0] = 0x66;
    CHECK(peer_send(peer, &tx, 0) > 0);
    if (hal.recv(hal.user, &f) == 1) {
        CHECK_EQ(f.id, 0x1E000005u);
        CHECK_EQ(f.len, 4u);
        CHECK_EQ(f.data[0], 0x66u);
        printf("      链路未被污染：随后的 Classic 帧正常收到\n");
    } else {
        printf("  FAIL FD 帧之后 Classic 帧也收不到了（链路状态被破坏）\n");
        g_fail++; g_checks++;
    }

    jsdk_hal_close(h);
    close(peer);
}

/**
 * 6. FD 请求 vs Classic 链路：必须**明确拒绝**。
 *
 * 这是 DESIGN 里承诺的行为：链路是 Classic（MTU=16）而调用方要 FD 时返回
 * `JSDK_ERR_INVALID_ARG`，而不是之后每条帧都 EINVAL 且看不出原因。
 * 通过 `JSDK_SC_IFACE_CLASSIC` 指定一个 MTU=16 的接口来验（没有就跳过）。
 */
static void test_fd_on_classic_link(const char *iface_classic)
{
    jsdk_can_hal_t hal;
    jsdk_hal_handle_t *h = NULL;

    if (!iface_classic || !iface_classic[0]) {
        printf("[6] FD 请求 vs Classic 链路：跳过（未设 JSDK_SC_IFACE_CLASSIC）\n");
        return;
    }
    printf("[6] FD 请求 vs Classic 链路（%s）：应返回 INVALID_ARG\n", iface_classic);
    memset(&hal, 0, sizeof hal);
    CHECK_EQ(jsdk_hal_socketcan_open(&hal, &h, iface_classic, 1000000u, 5000000u),
             JSDK_ERR_INVALID_ARG);
    CHECK(h == NULL);

    /* 同一接口按 Classic 开则必须成功 */
    CHECK_EQ(jsdk_hal_socketcan_open(&hal, &h, iface_classic, 1000000u, 0u), JSDK_OK);
    CHECK(h != NULL);
    if (h) jsdk_hal_close(h);
}

int main(void)
{
    const char *iface = getenv("JSDK_SC_IFACE");
    const char *iface_classic = getenv("JSDK_SC_IFACE_CLASSIC");

    setvbuf(stdout, NULL, _IONBF, 0);

    if (!iface || !iface[0]) {
        printf("=== SocketCAN 真链路冒烟：跳过（未设置 JSDK_SC_IFACE）===\n"
               "    用法：JSDK_SC_IFACE=vcan0 %s\n", "test_socketcan_live");
        return 0;
    }

    printf("=== SocketCAN 真链路冒烟（接口 %s）===\n\n", iface);

    test_open(iface);          printf("\n");
    test_tx_classic(iface);    printf("\n");
    test_rx_extended(iface);   printf("\n");
    test_fd_both_ways(iface);  printf("\n");
    test_fd_frame_on_classic_socket(iface);  printf("\n");
    test_fd_on_classic_link(iface_classic);  printf("\n");

    printf("=== %u checks, %u failures ===\n", g_checks, g_fail);
    return g_fail != 0u ? 1 : 0;
}
