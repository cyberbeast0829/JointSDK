/**
 * @file    hal_socketcan.c
 * @brief   Linux SocketCAN 后端（CAN + CAN-FD/BRS）
 *
 * @par 职责边界（很重要）
 *  本后端**只做帧的搬运与格式转换**，不碰位定时：
 *  波特率由内核链路负责，必须先配好，例如
 *  @code
 *      ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
 *  @endcode
 *  `bitrate` / `data_bitrate` 参数只用于**与链路核对**：如果链路是 Classic
 *  而调用方要求 FD，返回 `JSDK_ERR_INVALID_ARG` 而不是把 FD 帧丢进 Classic
 *  链路（那会得到一堆 EINVAL，客户看不出原因）。
 *
 * @par 为什么用 socket 而不是 `libsocketcan`
 *  零额外依赖：`socket(PF_CAN, SOCK_RAW, CAN_RAW)` + `bind()` + `read()/write()`
 *  就够。客户交叉编译时不需要再装一个库。
 *
 * @par 时间戳与阻塞
 *  接收用 `poll()` 加超时（`recv()` 契约允许"当前无帧"返回 0），**不设**
 *  `O_NONBLOCK` —— poll 已经够了，而 O_NONBLOCK 与 `write()` 的 EAGAIN 处理
 *  会让"发送失败"与"队列满"混在一起，客户无法区分。
 *
 * @par 线程安全
 *  单线程使用；`send`/`recv` 可被不同线程调用（内核 socket 本身是原子的），
 *  但 SDK 侧不保证。
 */

#include "jsdk_hal_builtin.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>          /* clock_gettime / CLOCK_MONOTONIC */

#include <poll.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/error.h>   /* ⚠ 错误帧的位定义（CAN_ERR_*）在**这个**头里，
                                  不在 <linux/can.h> —— 漏 include 时 GCC 的
                                  报错是"CAN_ERR_BUSOFF undeclared"，
                                  而这一整块代码在 Windows 上从不编译，
                                  所以直到第一次 Linux 构建才暴露。 */
#include <linux/can/raw.h>

#include "hal_handle.h"

/*
 * ⚠ `CANFD_FDF` 在 Linux 5.11 之前的内核头里**没有**（Ubuntu 20.04 自带 5.4 头）。
 *   而内核从 5.11 起在发送侧**强制**要求 FD 帧置这一位：不给的话，往
 *   `CAN_RAW_FD_FRAMES` 已开的 socket 写一条 `len > 8` 的 `canfd_frame`
 *   会被回绝（EINVAL），表现为"FD 帧一条也发不出去"。
 *   自己补上定义：老内核忽略这一位（无害），新内核则必须要有它。
 */
#ifndef CANFD_FDF
#define CANFD_FDF 0x04
#endif

/* --------------------------------------------------------------------------
 * 内部结构
 * ------------------------------------------------------------------------ */

/** `recv()` 的 poll 超时（毫秒）。0 = 立即返回，符合 "无帧返回 0" 的契约。 */
#define JSDK_SC_RECV_POLL_MS 0

/**
 * 是否在链路失败时自动尝试恢复。
 * 不自动恢复：CAN 控制器重启属于运维动作，静默重连会掩盖硬件问题。
 */
typedef struct {
    struct jsdk_hal_handle base;

    int      fd;
    char     ifname[IFNAMSIZ];
    int      is_fd;              /**< 调用方要求的模式（FD / Classic） */
    uint8_t  link_is_fd;         /**< 链路实际状态（MTU == CANFD_MTU） */

    /* 诊断计数 */
    uint32_t tx_frames;
    uint32_t rx_frames;
    uint32_t tx_errors;
    uint32_t rx_errors;
    uint32_t rx_overrun;

    /**
     * 链路错误**锁存**位（`JSDK_HAL_BUS_*`）。
     *
     * ⚠ 为什么是锁存而不是回调：`jsdk_can_hal_t` 在 `jsdk_context_init()`
     *   时被**复制**进 cfg，后端拿不到“调用方之后再改的” `hal->on_error`。
     *   所以后端把内核错误队列里看到的原因锁存下来，由 `bus_status` 上报
     *   （SDK 会读它），并在**一次成功发送**后清除 —— 能发出去就说明链路活了。
     */
    uint32_t err_latch;
} sc_handle_t;

static sc_handle_t *sc(jsdk_hal_handle_t *h)
{
    return JSDK_HAL_CAST(sc_handle_t, h);
}

/* --------------------------------------------------------------------------
 * 时间基准（单调毫秒）
 *
 * 用 CLOCK_MONOTONIC：SDK 的超时/健康判定都基于"经过时间"，
 * 系统时间被 NTP 调整时不能倒退。
 * ------------------------------------------------------------------------ */

static uint32_t sc_now_ms(void *user)
{
    (void)user;
    {
        struct timespec ts;
        if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0u;
        return (uint32_t)((uint64_t)ts.tv_sec * 1000ull
                          + (uint64_t)ts.tv_nsec / 1000000ull);
    }
}

/* --------------------------------------------------------------------------
 * 链路错误上报
 * ------------------------------------------------------------------------ */

static void sc_latch_error(sc_handle_t *h, int kind)
{
    switch (kind) {
    case JSDK_BUS_ERR_WARNING:  h->err_latch |= JSDK_HAL_BUS_ERROR_WARN; break;
    case JSDK_BUS_ERR_PASSIVE:  h->err_latch |= JSDK_HAL_BUS_ERROR_PASS; break;
    case JSDK_BUS_ERR_BUS_OFF:  h->err_latch |= JSDK_HAL_BUS_OFF;        break;
    case JSDK_BUS_ERR_RX_OVERRUN:h->err_latch |= JSDK_HAL_BUS_ERROR_WARN; break;
    default: break;
    }
}

/**
 * 处理一个**错误帧**：把它携带的原因锁存到 `err_latch`（`bus_status` 会报出去）。
 *
 * ⚠ 这里**不**去单独"排空错误队列"。早期版本那样做，并且把队列里
 *   `!(can_id & CAN_ERR_FLAG)` 的帧 continue 掉 —— 那等于**丢掉数据帧**：
 *   一次总线错误之后紧跟的 MIT 响应/参数响应会被静默吃掉，表现为"反馈偶尔
 *   断一下"，而且总线越不干净越严重。
 *   错误帧本来就会从正常 `read()` 出来（`CAN_RAW_ERR_FILTER` 已置位），
 *   就地识别、锁存、返回"本周期没有数据帧"即可，队列里其余帧一帧都不会少。
 */
static void sc_note_error_frame(sc_handle_t *h, const struct can_frame *ef)
{
    if (ef->can_id & CAN_ERR_BUSOFF) {
        sc_latch_error(h, JSDK_BUS_ERR_BUS_OFF);
    }
    if (ef->can_id & CAN_ERR_CRTL) {
        if (ef->data[1] & CAN_ERR_CRTL_RX_PASSIVE)  sc_latch_error(h, JSDK_BUS_ERR_PASSIVE);
        if (ef->data[1] & CAN_ERR_CRTL_TX_PASSIVE)  sc_latch_error(h, JSDK_BUS_ERR_PASSIVE);
        if (ef->data[1] & CAN_ERR_CRTL_RX_WARNING)  sc_latch_error(h, JSDK_BUS_ERR_WARNING);
        if (ef->data[1] & CAN_ERR_CRTL_TX_WARNING)  sc_latch_error(h, JSDK_BUS_ERR_WARNING);
        if (ef->data[1] & CAN_ERR_CRTL_RX_OVERFLOW) {
            h->rx_overrun++;
            sc_latch_error(h, JSDK_BUS_ERR_RX_OVERRUN);
        }
    }
    if (ef->can_id & CAN_ERR_ACK)  sc_latch_error(h, JSDK_BUS_ERR_TX_FAIL);
    if (ef->can_id & CAN_ERR_BUSERROR) sc_latch_error(h, JSDK_BUS_ERR_OTHER);
}

/* --------------------------------------------------------------------------
 * HAL 回调
 * ------------------------------------------------------------------------ */

static int sc_send(void *user, const jsdk_can_frame_t *f)
{
    sc_handle_t *h = sc((jsdk_hal_handle_t *)user);
    ssize_t n;

    if (!h || !f) return -1;
    if (h->fd < 0) return -1;

    if (f->flags & JSDK_FRAME_FD) {
        struct canfd_frame cf;

        if (!h->link_is_fd) return -1;               /* 链路是 Classic：发不了 FD */
        if (f->len > 64u) return -1;

        memset(&cf, 0, sizeof cf);
        cf.can_id = f->id;
        if (f->flags & JSDK_FRAME_EXT) cf.can_id |= CAN_EFF_FLAG;
        cf.len   = f->len;
        /* ⚠ `CANFD_FDF` 必须置位：内核据此认定"这是 FD 帧"，不置时
           len > 8 的写会直接 EINVAL（见文件头的说明）。 */
        cf.flags = (uint8_t)(CANFD_FDF
                             | ((f->flags & JSDK_FRAME_BRS) ? CANFD_BRS : 0u));
        if (f->len) memcpy(cf.data, f->data, f->len);

        n = write(h->fd, &cf, sizeof cf);
    } else {
        struct can_frame cf;

        if (f->len > 8u) return -1;                  /* Classic 上限 8 字节 */

        memset(&cf, 0, sizeof cf);
        cf.can_id = f->id;
        if (f->flags & JSDK_FRAME_EXT) cf.can_id |= CAN_EFF_FLAG;
        cf.can_dlc = f->len;
        if (f->len) memcpy(cf.data, f->data, f->len);

        n = write(h->fd, &cf, sizeof cf);
    }

    if (n < 0) {
        h->tx_errors++;
        /* ENOBUFS = TX 队列满，属"忙"而非"坏"；但两者对客户都是"这条帧没发出去"。
           `errno` 只用于诊断计数，不在此处区分（SDK 会按返回值重试）。 */
        return -1;
    }
    h->tx_frames++;
    h->err_latch = 0u;      /* 能发出去 → 链路活着，清掉旧的错误锁存 */
    return 0;
}

static int sc_recv(void *user, jsdk_can_frame_t *f)
{
    sc_handle_t *h = sc((jsdk_hal_handle_t *)user);
    struct pollfd pfd;
    uint8_t buf[sizeof(struct canfd_frame)];
    ssize_t n;

    if (!h || !f) return -1;
    if (h->fd < 0) return -1;

    pfd.fd = h->fd;
    pfd.events = POLLIN;
    pfd.revents = 0;

    n = poll(&pfd, 1u, JSDK_SC_RECV_POLL_MS);
    if (n < 0) {
        if (errno == EINTR) return 0;                /* 被信号打断：当作"无帧" */
        h->rx_errors++;
        return -1;
    }
    if (n == 0) return 0;                            /* 无帧 */

    /*
     * POLLERR 只表示"总线上出过错"，错误帧本身还在队列里等着被读出来
     * （下面对 can_id 判 CAN_ERR_FLAG 就会拿到它）。所以这里**不要**提前
     * 返回 -1：那会让 SDK 每周期都判链路失败，也让客户永远看不到原因。
     * 只有 POLLHUP/POLLNVAL（接口消失/句柄无效）才是真正的链路错误。
     */
    if (pfd.revents & (POLLHUP | POLLNVAL)) {
        h->rx_errors++;
        return -1;
    }

    n = read(h->fd, buf, sizeof buf);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
        h->rx_errors++;
        return -1;
    }

    if ((size_t)n == sizeof(struct canfd_frame)) {
        const struct canfd_frame *cf = (const struct canfd_frame *)(const void *)buf;

        if (cf->can_id & CAN_ERR_FLAG) {             /* 错误帧 → 不进数据通路 */
            h->rx_errors++;
            return 0;                                /* 原因已锁存，见下 */
        }
        memset(f, 0, sizeof *f);
        f->id    = (uint32_t)(cf->can_id & CAN_EFF_MASK);
        f->flags = (uint8_t)(JSDK_FRAME_FD
                             | ((cf->can_id & CAN_EFF_FLAG) ? JSDK_FRAME_EXT : 0u)
                             | ((cf->flags & CANFD_BRS) ? JSDK_FRAME_BRS : 0u));
        f->len   = (cf->len > 64u) ? 64u : cf->len;
        memcpy(f->data, cf->data, f->len);
        h->rx_frames++;
        return 1;
    }

    if ((size_t)n == sizeof(struct can_frame)) {
        const struct can_frame *cf = (const struct can_frame *)(const void *)buf;

        if (cf->can_id & CAN_ERR_FLAG) {
            h->rx_errors++;
            sc_note_error_frame(h, cf);              /* 锁存原因，供 bus_status 上报 */
            return 0;
        }
        memset(f, 0, sizeof *f);
        f->id    = (uint32_t)(cf->can_id & CAN_EFF_MASK);
        f->flags = (uint8_t)((cf->can_id & CAN_EFF_FLAG) ? JSDK_FRAME_EXT : 0u);
        f->len   = (cf->can_dlc > 8u) ? 8u : cf->can_dlc;
        memcpy(f->data, cf->data, f->len);
        h->rx_frames++;
        return 1;
    }

    h->rx_errors++;                                  /* 半帧/未知长度：丢掉 */
    return 0;
}

static int sc_bus_status(void *user, uint32_t *flags)
{
    sc_handle_t *h = sc((jsdk_hal_handle_t *)user);
    struct ifreq ifr;

    if (!h || !flags) return -1;
    *flags = 0u;

    memset(&ifr, 0, sizeof ifr);
    if (h->fd < 0) return -1;

    /* 用本接口名而不是 ifindex：客户看的是 "can0"，错误信息要对得上 */
    {
        size_t i;
        for (i = 0u; i < sizeof ifr.ifr_name; ++i) {
            ifr.ifr_name[i] = h->ifname[i];
            if (h->ifname[i] == '\0') break;
        }
    }

    if (ioctl(h->fd, SIOCGIFFLAGS, &ifr) != 0) return -1;
    if (ifr.ifr_flags & IFF_UP) *flags |= JSDK_HAL_BUS_OK;
    *flags |= h->err_latch;
    return 0;
}

static void sc_destroy(struct jsdk_hal_handle *base)
{
    sc_handle_t *h = JSDK_HAL_CAST(sc_handle_t, base);
    if (!h) return;
    if (h->fd >= 0) close(h->fd);      /* ⚠ 早先的 jsdk_hal_close() 不关 fd */
    free(h);
}

/* --------------------------------------------------------------------------
 * 打开
 * ------------------------------------------------------------------------ */

jsdk_status_t jsdk_hal_socketcan_open(jsdk_can_hal_t *hal, jsdk_hal_handle_t **out,
                                      const char *ifname,
                                      uint32_t bitrate, uint32_t data_bitrate)
{
    sc_handle_t *h;
    struct sockaddr_can addr;
    struct ifreq ifr;
    int fd;
    int mtu;
    int ifindex;
    int want_fd;

    if (!hal || !out || !ifname || !ifname[0]) return JSDK_ERR_INVALID_ARG;
    *out = NULL;

    if (strlen(ifname) >= IFNAMSIZ) return JSDK_ERR_INVALID_ARG;
    if (bitrate == 0u) { bitrate = 1000000u; }
    if (data_bitrate != 0u && data_bitrate < bitrate) {
        /* 数据段比仲裁段还慢只能说明参数写错了 */
        return JSDK_ERR_INVALID_ARG;
    }
    want_fd = (data_bitrate != 0u) ? 1 : 0;

    fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) return JSDK_ERR_TRANSPORT;

    /* --- 取 ifindex 并顺带核对 MTU（CANFD_MTU = 72 表示链路已开 FD） --- */
    memset(&ifr, 0, sizeof ifr);
    {
        size_t i;
        for (i = 0u; i < IFNAMSIZ - 1u && ifname[i]; ++i) ifr.ifr_name[i] = ifname[i];
    }
    if (ioctl(fd, SIOCGIFINDEX, &ifr) != 0) {
        close(fd);
        return JSDK_ERR_NOT_FOUND;        /* 接口不存在 */
    }

    /*
     * ⚠⚠ **必须立刻把 ifindex 存下来，后面不能再用 `ifr.ifr_ifindex`。**
     *
     *   `struct ifreq` 的载荷是一个 **union**（`ifr_ifindex` / `ifr_mtu` /
     *   `ifr_flags` / `ifr_name` 之外的绝大多数成员共用同一块内存），
     *   所以紧接着的 `SIOCGIFMTU` 会把 MTU 写进同一块内存 ——
     *   之后再读 `ifr.ifr_ifindex` 拿到的其实是**MTU**。
     *
     *   实测（vcan0：ifindex=9、MTU=72）：`bind(can_ifindex=72)` →
     *   `EADDRNOTAVAIL` → `jsdk_hal_socketcan_open()` **恒返回
     *   JSDK_ERR_TRANSPORT**。也就是说这个后端在 Linux 上**从来没有成功打开过
     *   一个接口**（该文件在 Windows 上根本不参与编译，所以编译器也从未看过它）。
     *   见 DESIGN v0.17 与 `tests/test_socketcan_live.c`（现在会真的打开接口）。
     */
    ifindex = ifr.ifr_ifindex;

    if (ioctl(fd, SIOCGIFMTU, &ifr) != 0) {
        close(fd);
        return JSDK_ERR_TRANSPORT;
    }
    mtu = ifr.ifr_mtu;

    /*
     * ⚠ 这里**故意做强校验**而不是"能用就用"：
     *   - 客户要求 FD 但链路是 Classic（MTU 16）→ 之后每次 write 都是 EINVAL，
     *     现场表现是"发不出去但看不出原因"。直接拒绝并说明怎么改链路。
     *   - 客户要求 Classic 但链路是 FD → 其实可以用（内核允许在 FD 链路上发
     *     Classic 帧），所以只警告不拒绝；为此在下方记录 link_is_fd。
     */
    if (want_fd && mtu != CANFD_MTU) {
        close(fd);
        return JSDK_ERR_INVALID_ARG;
    }

    /*
     * ⚠ 光确认"链路支持 FD"不够，**必须显式打开 socket 的 FD 帧能力**：
     *   没有 `CAN_RAW_FD_FRAMES` 时，写一条 72 字节的 `canfd_frame` 会被内核
     *   回绝（EINVAL），表现为"open() 成功、但 FD 帧一条也发不出去"
     *   —— 正是本后端之前的实况（见 DESIGN v0.17）。
     *   老内核（< 2.6.37）可能不认识这个选项 → 明确报 UNSUPPORTED。
     */
    if (want_fd) {
        int on = 1;
        if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof on) != 0) {
            close(fd);
            return JSDK_ERR_UNSUPPORTED;   /* 内核不支持 CAN FD socket */
        }
    }

    /* --- 绑定 --- */
    memset(&addr, 0, sizeof addr);
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifindex;          /* 用存下来的值，见上面的 union 说明 */
    if (bind(fd, (struct sockaddr *)&addr, sizeof addr) != 0) {
        close(fd);
        return JSDK_ERR_TRANSPORT;
    }

    /*
     * 只关"收自己的帧"，**不要**去关 LOOPBACK：
     *
     *   - `CAN_RAW_RECV_OWN_MSGS`（默认 0）：本 socket 要不要收到**自己发的**帧。
     *     这是我们想要的语义（主站不与自己通信），且本来就是默认值，
     *     显式设一次以免受系统/驱动差异影响。
     *   - `CAN_RAW_LOOPBACK`（默认 1）：本地发出的帧要不要回送给**本接口上的其它
     *     socket**。把它置 0 会让本地任何对端（自测的对端脚本、同一进程里的第二个
     *     上下文、co-simulation）**完全看不到**我们发的帧 ——
     *     而它在真实总线上毫无好处（真实设备收到的帧与这个开关无关）。
     *
     *   ✅ 两个开关配合后的正确行为：**我们自己看不到自己的帧，但本地别的
     *     socket 能看到**（而这正是真链路上对端的行为）。
     *
     *   ⚠ 多上下文共用同一接口时请给**不同的 `master_id`**：协议用
     *     `(SRC, DEST, MsgType)` 关联请求与响应，同一个 master_id 的两个主站
     *     会把彼此的请求帧误认为响应帧。
     */
    {
        int own = 0;
        (void)setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &own, sizeof own);
    }
    /* --- 错误帧要收进来（默认是开的，显式设一次以免被系统默认值搞懵） --- */
    {
        can_err_mask_t mask = CAN_ERR_MASK;
        (void)setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &mask, sizeof mask);
    }

    h = (sc_handle_t *)calloc(1u, sizeof *h);
    if (!h) {
        close(fd);
        return JSDK_ERR_NO_MEMORY;
    }

    h->base.kind    = JSDK_HAL_KIND_SOCKETCAN;
    h->base.destroy = sc_destroy;
    h->fd           = fd;
    h->is_fd        = want_fd;
    h->link_is_fd   = (mtu == CANFD_MTU) ? 1u : 0u;
    {
        size_t i;
        for (i = 0u; i + 1u < sizeof h->ifname && ifname[i]; ++i) h->ifname[i] = ifname[i];
        h->ifname[i] = '\0';
    }

    hal->user       = h;
    hal->send       = sc_send;
    hal->recv       = sc_recv;
    hal->now_ms     = sc_now_ms;
    hal->on_error   = NULL;   /* 见下方说明 */
    hal->bus_status = sc_bus_status;

    /*
     * ⚠ 这里**不**设 `hal->on_error`：`jsdk_can_hal_t` 在 `jsdk_context_init()`
     *   被复制进 cfg，后端拿不到"之后被调用方改过的"指针；而后端自己也没有
     *   反向引用。链路错误因此走两条明确的路：
     *     ① `recv()`/`send()` 返回 <0 → SDK 记账 `link_errors` 并置 `link_up = 0`；
     *     ② 内核错误队列里的原因被锁存到 `err_latch`，由 `bus_status` 上报。
     */
    h->err_latch = 0u;

    *out = &h->base;
    return JSDK_OK;
}

/* --------------------------------------------------------------------------
 * 诊断（CLI / 测试用；不进公共头，避免 MCU 客户看到平台细节）
 * ------------------------------------------------------------------------ */

/** 该句柄是否跑在 CAN-FD 链路上（1 = FD）。 */
int jsdk_hal_socketcan_is_fd(jsdk_hal_handle_t *h);
int jsdk_hal_socketcan_is_fd(jsdk_hal_handle_t *h)
{
    sc_handle_t *s = sc(h);
    return s ? (int)s->link_is_fd : 0;
}

/** 取诊断计数：`tx / rx / tx_err / rx_err / overrun`（任一可为 NULL）。 */
void jsdk_hal_socketcan_stats(jsdk_hal_handle_t *h, uint32_t *tx, uint32_t *rx,
                              uint32_t *tx_err, uint32_t *rx_err, uint32_t *overrun);
void jsdk_hal_socketcan_stats(jsdk_hal_handle_t *h, uint32_t *tx, uint32_t *rx,
                              uint32_t *tx_err, uint32_t *rx_err, uint32_t *overrun)
{
    sc_handle_t *s = sc(h);
    if (!s) return;
    if (tx)      *tx      = s->tx_frames;
    if (rx)      *rx      = s->rx_frames;
    if (tx_err)  *tx_err  = s->tx_errors;
    if (rx_err)  *rx_err  = s->rx_errors;
    if (overrun) *overrun = s->rx_overrun;
}
