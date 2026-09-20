/**
 * @file    hal_slcan.c
 * @brief   串口 slcan 后端（CANable / 兼容 USB-CAN 适配器）
 *
 * @par 结构
 *  本文件只做**串口 I/O**；帧的 ASCII 编解码在 `hal_slcan_codec.c`（纯逻辑，
 *  可离线单元测试）。
 *
 * @par 非阻塞语义
 *  `recv()` 契约要求"当前无帧返回 0"，所以：
 *   - POSIX：串口开 `O_NONBLOCK`，并先用 `select()` 判断可读；
 *   - Windows：`SetCommTimeouts` 把读超时全部置 0（立即返回当前已有字节）。
 *  串口是**流**，`recv()` 内部维持一个小缓冲，把读到的一半行留住直到 `\r`。
 *
 * @par CAN FD
 *  slcan 的 FD 表述来自 **CANable 2.0 固件**（Lawicel 原版没有）：帧前缀
 *  `d/D`（BRS=0）与 `b/B`（BRS=1），DLC 位是 FD 长度码。详见
 *  `hal_slcan_codec.h`。因此：
 *   - 帧类型由**帧自身的标志**决定（`JSDK_FRAME_FD` / `JSDK_FRAME_BRS`），
 *     本后端不做"后端级别"的 FD 开关；
 *   - 但**适配器侧**要能跑 FD 需要数据段速率：`data_bitrate != 0` 时本后端会
 *     主动发 `Y<n>`（这一步**会改适配器配置**，与 SocketCAN 的"内核管链路、
 *     SDK 不碰"不同，因此是显式 opt-in；传 0 则完全不碰适配器配置）。
 *
 * @par 打开序列
 *  `C\r`（回到关闭态）→ [`Y<n>\r`]（仅 FD）→ `O\r`（打开通道）。
 *  `O` 不可省：Lawicel 语义下通道上电是关闭的，只发 `C` 而不发 `O`
 *  等于让适配器一直闭着（现场表现为"一条帧都收不到"）。
 *
 * @par 已知局限（写进文档，不隐藏）
 *  - `Y<n>` 的索引是**固件私有表**，这里只收录 CANable 2.0 公认的两个
 *    （`Y2`=2 Mbps、`Y5`=5 Mbps）；要别的速率请用厂家工具先配好再传 0；
 *  - 吞吐：Classic 约 100~500 fps；FD 单帧载荷更大，同样带宽下帧率更低 ——
 *    只适合配置、监控与低速 MIT，不适合高频控制；
 *  - 适配器通常**不保证**发送时序，广播同步的"同一周期"精度受 USB 帧调度影响。
 *
 * @par 线程安全
 *  单线程使用（同虚拟后端）。
 */

#include "jsdk_hal_builtin.h"

#include <stdlib.h>
#include <string.h>

#include "hal_handle.h"
#include "hal_slcan_codec.h"

/* --------------------------------------------------------------------------
 * 平台头与串口原语
 * ------------------------------------------------------------------------ */

#ifdef _WIN32

#include <windows.h>

typedef struct {
    HANDLE h;
} sl_port_t;

static void sl_port_close(sl_port_t *p);    /* 前向声明：open 的失败路径要用 */

static int sl_port_open(sl_port_t *p, const char *name, uint32_t baud)
{
    char path[64];
    DCB  dcb;
    COMMTIMEOUTS to;

    /* COM10 及以上必须用 \\.\ 前缀；统一加，COM1..9 也接受这种写法 */
    if (strncmp(name, "\\\\.\\", 4) == 0) {
        if (strlen(name) >= sizeof path) return -1;
        strcpy(path, name);
    } else {
        size_t l = strlen(name);
        if (l + 5u >= sizeof path) return -1;
        memcpy(path, "\\\\.\\", 4);
        memcpy(path + 4, name, l + 1u);
    }

    p->h = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                       OPEN_EXISTING, 0, NULL);
    if (p->h == INVALID_HANDLE_VALUE) {
        p->h = NULL;
        return -1;
    }

    memset(&dcb, 0, sizeof dcb);
    dcb.DCBlength = (DWORD)sizeof dcb;
    if (!GetCommState(p->h, &dcb)) { sl_port_close(p); return -1; }
    dcb.BaudRate = (DWORD)baud;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary  = TRUE;
    dcb.fParity  = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl  = DTR_CONTROL_ENABLE;
    dcb.fRtsControl  = RTS_CONTROL_ENABLE;
    dcb.fOutX = FALSE;
    dcb.fInX  = FALSE;
    if (!SetCommState(p->h, &dcb)) { sl_port_close(p); return -1; }

    /* 读超时全 0 → ReadFile 立即返回"当前已有字节数"（0 也是合法结果） */
    memset(&to, 0, sizeof to);
    to.ReadIntervalTimeout         = MAXDWORD;
    to.ReadTotalTimeoutMultiplier  = 0;
    to.ReadTotalTimeoutConstant    = 0;
    to.WriteTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant   = 500;
    if (!SetCommTimeouts(p->h, &to)) { sl_port_close(p); return -1; }

    PurgeComm(p->h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return 0;
}

static void sl_port_close(sl_port_t *p)
{
    if (p->h) {
        CloseHandle(p->h);
        p->h = NULL;
    }
}

/** @return 读到的字节数（0 = 当前无数据）；<0 = 错误。 */
static int sl_port_read(sl_port_t *p, uint8_t *buf, size_t cap)
{
    DWORD got = 0;

    if (!p->h) return -1;
    if (!ReadFile(p->h, buf, (DWORD)cap, &got, NULL)) return -1;
    return (int)got;
}

static int sl_port_write_all(sl_port_t *p, const char *buf, size_t len)
{
    size_t off = 0u;

    if (!p->h) return -1;
    while (off < len) {
        DWORD wrote = 0;
        if (!WriteFile(p->h, buf + off, (DWORD)(len - off), &wrote, NULL)) return -1;
        if (wrote == 0u) return -1;
        off += (size_t)wrote;
    }
    return 0;
}

static uint32_t sl_now_ms(void)
{
    return (uint32_t)GetTickCount();
}

#else  /* ------------------------------- POSIX -------------------------------- */

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>          /* clock_gettime / CLOCK_MONOTONIC */
#include <unistd.h>

#include <sys/select.h>
#include <sys/time.h>

typedef struct {
    int fd;
} sl_port_t;

static void sl_port_close(sl_port_t *p);    /* 前向声明：open 的失败路径要用 */

static speed_t sl_baud_to_speed(uint32_t baud)
{
    switch (baud) {
    case 9600u:    return B9600;
    case 19200u:   return B19200;
    case 38400u:   return B38400;
    case 57600u:   return B57600;
    case 115200u:  return B115200;
    case 230400u:  return B230400;
#ifdef B460800
    case 460800u:  return B460800;
#endif
#ifdef B921600
    case 921600u:  return B921600;
#endif
#ifdef B1000000
    case 1000000u: return B1000000;
#endif
#ifdef B2000000
    case 2000000u: return B2000000;
#endif
#ifdef B3000000
    case 3000000u: return B3000000;
#endif
    default:       return (speed_t)0;      /* 0 表示"不认识的波特率" */
    }
}

static int sl_port_open(sl_port_t *p, const char *name, uint32_t baud)
{
    struct termios tio;
    speed_t sp = sl_baud_to_speed(baud);

    if (sp == (speed_t)0) return -1;      /* 不支持的波特率：明确失败 */

    /* O_NONBLOCK：recv() 契约要求非阻塞 */
    p->fd = open(name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (p->fd < 0) return -1;

    if (tcgetattr(p->fd, &tio) != 0) { sl_port_close(p); return -1; }

    /* raw 模式：8N1、无流控、无回显、无字符转换 */
    tio.c_iflag &= (tcflag_t)~(IGNBRK | BRKINT | PARMRK | ISTRIP
                               | INLCR | IGNCR | ICRNL | IXON);
    tio.c_oflag &= (tcflag_t)~OPOST;
    tio.c_lflag &= (tcflag_t)~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tio.c_cflag &= (tcflag_t)~(CSIZE | PARENB);
    tio.c_cflag |= (CS8 | CLOCAL | CREAD);
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    if (cfsetispeed(&tio, sp) != 0 || cfsetospeed(&tio, sp) != 0) {
        sl_port_close(p);
        return -1;
    }
    if (tcsetattr(p->fd, TCSANOW, &tio) != 0) { sl_port_close(p); return -1; }

    tcflush(p->fd, TCIOFLUSH);
    return 0;
}

static void sl_port_close(sl_port_t *p)
{
    if (p->fd >= 0) {
        close(p->fd);
        p->fd = -1;
    }
}

/** @return 读到的字节数（0 = 当前无数据）；<0 = 错误。 */
static int sl_port_read(sl_port_t *p, uint8_t *buf, size_t cap)
{
    fd_set rfds;
    struct timeval tv;
    ssize_t n;

    if (p->fd < 0) return -1;

    FD_ZERO(&rfds);
    FD_SET(p->fd, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    n = select(p->fd + 1, &rfds, NULL, NULL, &tv);
    if (n < 0) return (errno == EINTR) ? 0 : -1;
    if (n == 0) return 0;

    n = read(p->fd, buf, cap);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
        return -1;
    }
    return (int)n;
}

static int sl_port_write_all(sl_port_t *p, const char *buf, size_t len)
{
    size_t off = 0u;

    if (p->fd < 0) return -1;
    while (off < len) {
        ssize_t n = write(p->fd, buf + off, len - off);
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;  /* 忙等：见下方说明 */
            return -1;
        }
        if (n == 0) return -1;
        off += (size_t)n;
    }
    return 0;
}

static uint32_t sl_now_ms(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0u;
    return (uint32_t)((uint64_t)ts.tv_sec * 1000ull
                      + (uint64_t)ts.tv_nsec / 1000000ull);
}

#endif /* platform */

/**
 * ⚠ POSIX 的 `sl_port_write_all()` 在 EAGAIN 上是**忙等**：
 *   串口 115200 下发一条 8 字节帧约 0.7 ms，而这个函数只在一次 `send()` 内被
 *   调用，最坏情况占住 CPU 不到 1 ms。换成 poll 会更"正确"，但会把写超时逻辑
 *   复制一份，收益不抵复杂度。**Windows 路径不需要忙等**（WriteFile 带 500 ms
 *   写超时）。这条注释是为了让后来的人知道这是有意为之，而不是漏写。
 */

/* --------------------------------------------------------------------------
 * 句柄
 * ------------------------------------------------------------------------ */

/** RX 行缓冲上限。
 *
 * FD 单行最长 139 B（见 `JSDK_SLCAN_LINE_MAX`），取 1024 能稳稳装下"一次 read
 * 里来了一批 FD 帧"的情况；缓冲只在 **满且看不到 `\r`** 时才丢弃（那是坏数据），
 * 所以给大一点不会掩盖问题，只是让丢弃阈值更晚生效。
 */
#define JSDK_SLCAN_RXBUF 1024u

typedef struct {
    struct jsdk_hal_handle base;

    sl_port_t port;
    uint8_t   fd_enabled;          /**< 1 = 已向适配器发过 Y<n>（数据段速率已设） */
    uint32_t  fd_data_bitrate;     /**< 已设置的数据段速率（0 = 没设过） */

    uint8_t   rx[JSDK_SLCAN_RXBUF];
    size_t    rx_len;

    uint32_t  tx_frames;
    uint32_t  rx_frames;
    uint32_t  tx_fd_frames;        /**< 其中 FD 帧（诊断：0 = 对端按 Classic 回的） */
    uint32_t  rx_fd_frames;
    uint32_t  tx_errors;
    uint32_t  rx_errors;
    uint32_t  rx_malformed;
    uint32_t  rx_acks;
    uint32_t  rx_nacks;
} sl_handle_t;

static sl_handle_t *sl(jsdk_hal_handle_t *h)
{
    return JSDK_HAL_CAST(sl_handle_t, h);
}

static uint32_t sl_now_ms_hal(void *user)
{
    (void)user;
    return sl_now_ms();
}

/* --------------------------------------------------------------------------
 * HAL 回调
 * ------------------------------------------------------------------------ */

static int sl_send(void *user, const jsdk_can_frame_t *f)
{
    sl_handle_t *h = sl((jsdk_hal_handle_t *)user);
    char   line[JSDK_SLCAN_LINE_MAX];
    size_t n;

    if (!h || !f) return -1;

    n = jsdk_slcan_encode(f, line, sizeof line);
    if (n == 0u) {
        /* 表达不了的帧（FD 的 9/10/11 字节、Classic 的 >8 字节）：
           明确失败并计数，**不静默丢**也**不降级成另一条帧**。 */
        h->tx_errors++;
        return -1;
    }
    if (sl_port_write_all(&h->port, line, n) != 0) {
        h->tx_errors++;
        return -1;
    }
    h->tx_frames++;
    if ((f->flags & JSDK_FRAME_FD) != 0u) h->tx_fd_frames++;
    return 0;
}

/**
 * 从串口读一段并尝试解出一帧。
 * 缓冲里可能有：若干完整行 + 一段未结束的行 + ACK/NACK 字符。
 */
static int sl_try_frame(sl_handle_t *h, jsdk_can_frame_t *f)
{
    for (;;) {
        size_t consumed = 0u;
        jsdk_slcan_rc_t rc;

        if (h->rx_len == 0u) return 0;

        rc = jsdk_slcan_decode((const char *)h->rx, h->rx_len, f, &consumed);

        if (rc == JSDK_SLCAN_NEED_MORE) {
            /* 缓冲满且还没见到行尾 → 一定是坏数据，丢掉以免永久卡住 */
            if (h->rx_len >= sizeof h->rx) {
                h->rx_len = 0u;
                h->rx_malformed++;
            }
            return 0;
        }

        /* 非 NEED_MORE：消费掉这一行（至少 1 字节，否则会死循环） */
        if (consumed == 0u) consumed = 1u;
        if (consumed > h->rx_len) consumed = h->rx_len;
        memmove(h->rx, h->rx + consumed, h->rx_len - consumed);
        h->rx_len -= consumed;

        switch (rc) {
        case JSDK_SLCAN_OK:
            h->rx_frames++;
            if ((f->flags & JSDK_FRAME_FD) != 0u) h->rx_fd_frames++;
            return 1;
        case JSDK_SLCAN_ACK:
            h->rx_acks++;
            continue;                     /* 这是"上一帧发送成功"的回执 */
        case JSDK_SLCAN_NACK:
            h->rx_nacks++;
            continue;                     /* 总线错误：计数后继续找数据行 */
        case JSDK_SLCAN_RTR:
            h->rx_malformed++;            /* 本协议不用 RTR */
            continue;
        case JSDK_SLCAN_MALFORMED:
        default:
            h->rx_malformed++;            /* 状态行/坏行：丢掉整行，不切错行 */
            continue;
        }
    }
}

static int sl_recv(void *user, jsdk_can_frame_t *f)
{
    sl_handle_t *h = sl((jsdk_hal_handle_t *)user);
    int got;
    int r;

    if (!h || !f) return -1;

    /* 先看缓冲里是否已经有一整帧 */
    r = sl_try_frame(h, f);
    if (r != 0) return r;

    /* 再读一批；注意适配器可能在一次 read 里给出半行，必须保留 */
    got = sl_port_read(&h->port, h->rx + h->rx_len, sizeof h->rx - h->rx_len);
    if (got < 0) {
        h->rx_errors++;
        return -1;
    }
    if (got == 0) return 0;

    h->rx_len += (size_t)got;
    return sl_try_frame(h, f);
}

static int sl_bus_status(void *user, uint32_t *flags)
{
    sl_handle_t *h = sl((jsdk_hal_handle_t *)user);

    if (!h || !flags) return -1;
    *flags = JSDK_HAL_BUS_OK;
    /* 适配器报过 NACK：链路大概率是"线接反/无终端电阻/波特率不符"。
       有 ack 也有 nack 时以"有 nack"为准——客户更容易忽略 nack。 */
    if (h->rx_nacks > 0u) *flags |= JSDK_HAL_BUS_ERROR_WARN;
    return 0;
}

static void sl_destroy(struct jsdk_hal_handle *base)
{
    sl_handle_t *h = JSDK_HAL_CAST(sl_handle_t, base);
    if (!h) return;
    sl_port_close(&h->port);      /* ⚠ 不关串口就是 fd/句柄泄漏 */
    free(h);
}

/* --------------------------------------------------------------------------
 * 打开
 * ------------------------------------------------------------------------ */

/**
 * 数据段速率的命令表。
 *
 * ⚠ `Y<n>` 的 n 是**固件私有的表索引**，不是 Mbps。这里只收录 CANable 2.0
 *   固件（以及 python-can 的 `slcan` 后端）公认的两项。**故意不猜**其它索引：
 *   猜错就是把适配器的数据段速率设成别的值，而现场只会看到"帧发不出去"。
 */
static const struct {
    uint32_t bitrate;
    const char *cmd;        /* 含行尾 `\r`，一次写完 */
} k_slcan_fd_rates[] = {
    { 2000000u, "Y2\r" },    /* 2 Mbps */
    { 5000000u, "Y5\r" }     /* 5 Mbps（本协议默认的 FD 数据段速率） */
};

#define JSDK_SLCAN_FD_RATE_COUNT \
    (sizeof k_slcan_fd_rates / sizeof k_slcan_fd_rates[0])

jsdk_status_t jsdk_hal_slcan_open(jsdk_can_hal_t *hal, jsdk_hal_handle_t **out,
                                  const char *port, uint32_t baud,
                                  uint32_t data_bitrate)
{
    sl_handle_t *h;
    /* `C` 让适配器回到关闭态；`O` 再把通道打开（Lawicel 语义）。 */
    static const char cmd_reset[] = "C\r";
    static const char cmd_open[]  = "O\r";
    const char *fd_cmd = NULL;
    unsigned    i;

    if (!hal || !out || !port || !port[0]) return JSDK_ERR_INVALID_ARG;
    *out = NULL;

    if (baud == 0u) baud = 115200u;

    /*
     * ⚠ **参数校验要在打开串口之前**：
     *   否则"速率不支持"和"串口打不开"会互相掩盖，用户换一个串口还是同样的
     *   错，很容易误判成硬件问题。这条也让测试能在**没有硬件**的机器上断言
     *   这个错误码。
     */
    if (data_bitrate != 0u) {
        for (i = 0u; i < JSDK_SLCAN_FD_RATE_COUNT; ++i) {
            if (k_slcan_fd_rates[i].bitrate == data_bitrate) {
                fd_cmd = k_slcan_fd_rates[i].cmd;
                break;
            }
        }
        if (!fd_cmd) return JSDK_ERR_UNSUPPORTED;   /* 表外的速率：明确说不支持 */
    }

    h = (sl_handle_t *)calloc(1u, sizeof *h);
    if (!h) return JSDK_ERR_NO_MEMORY;

#ifdef _WIN32
    h->port.h = NULL;
#else
    h->port.fd = -1;
#endif

    if (sl_port_open(&h->port, port, baud) != 0) {
        free(h);
        /* 端口不存在 / 被占用 / 波特率不支持，都属于"参数问题"而不是链路问题 */
        return JSDK_ERR_INVALID_ARG;
    }

    /*
     * 复位 + （可选）FD 速率 + 打开。这三条都是**尽力而为**：
     * 有些固件不认识 `Y` 或 `C`，照样能收发，所以失败不上升到错误
     * （但适配器配置没生效时会更早、更明显地表现为"收不到帧"）。
     */
    (void)sl_port_write_all(&h->port, cmd_reset, sizeof cmd_reset - 1u);
    if (fd_cmd) {
        /* ⚠ 用 strlen 而不是写死 3：表里的命令今后可能变长，
           写死长度会静默截断 —— 那种 bug 在真机上只表现为"适配器没反应"。 */
        (void)sl_port_write_all(&h->port, fd_cmd, strlen(fd_cmd));
        h->fd_enabled       = 1u;
        h->fd_data_bitrate  = data_bitrate;
    }
    (void)sl_port_write_all(&h->port, cmd_open, sizeof cmd_open - 1u);

    h->base.kind    = JSDK_HAL_KIND_SLCAN;
    h->base.destroy = sl_destroy;

    hal->user       = h;
    hal->send       = sl_send;
    hal->recv       = sl_recv;
    hal->now_ms     = sl_now_ms_hal;
    hal->on_error   = NULL;
    hal->bus_status = sl_bus_status;

    *out = &h->base;
    return JSDK_OK;
}

/* --------------------------------------------------------------------------
 * 诊断（CLI / Python 绑定用）
 *
 * 声明在公共头 `jsdk_hal_builtin.h` 里带 `JSDK_API`：这几个函数是给
 * **动态库使用者**（ctypes 绑定、CLI）调的，所以要参与导出。
 * ------------------------------------------------------------------------ */

int jsdk_hal_slcan_supports_fd(void)
{
    return 1;      /* CANable 2.0 的 b/B/d/D 扩展，见 hal_slcan_codec.h */
}

void jsdk_hal_slcan_fd_config(jsdk_hal_handle_t *h, int *enabled,
                              uint32_t *bitrate)
{
    sl_handle_t *s = sl(h);
    if (!s) return;
    if (enabled) *enabled = (int)s->fd_enabled;
    if (bitrate) *bitrate = s->fd_data_bitrate;
}

void jsdk_hal_slcan_fd_frames(jsdk_hal_handle_t *h, uint32_t *tx_fd,
                              uint32_t *rx_fd)
{
    sl_handle_t *s = sl(h);
    if (!s) return;
    if (tx_fd) *tx_fd = s->tx_fd_frames;
    if (rx_fd) *rx_fd = s->rx_fd_frames;
}

void jsdk_hal_slcan_stats(jsdk_hal_handle_t *h, uint32_t *tx, uint32_t *rx,
                          uint32_t *malformed, uint32_t *acks, uint32_t *nacks)
{
    sl_handle_t *s = sl(h);
    if (!s) return;
    if (tx)        *tx        = s->tx_frames;
    if (rx)        *rx        = s->rx_frames;
    if (malformed) *malformed = s->rx_malformed;
    if (acks)      *acks      = s->rx_acks;
    if (nacks)     *nacks     = s->rx_nacks;
}
