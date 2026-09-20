/**
 * @file    hal_pcan.c
 * @brief   PEAK PCAN-Basic 后端（Windows / macOS / Linux + PEAK 驱动）
 *
 * @par 为什么**运行期加载**而不是链接 PCANBasic.lib
 *  客户拿到的是 wheel / 免安装目录 / 现成 exe。链接 `.lib`/`.so` 意味着
 *  "没有装 PCAN 驱动就起不来"，而现场最常见的场景恰恰是"用 CANable 顶一下"。
 *  改成 `LoadLibrary` / `dlopen` 后：**只有真正选 `--if pcan` 时才需要驱动**，
 *  找不到 `PCANBasic` 就报 `JSDK_ERR_NOT_FOUND` 并给出装哪个包的提示。
 *
 * @par 函数指针放在句柄里，不放文件级 static
 *  本项目已踩过"文件级可变 static 导致多上下文串味"（`sim_device.c` 的
 *  `g_asm`）。PCAN 的 API 表也一样：放在句柄里，两条总线各持有自己的一份，
 *  互不干扰，代价只是几次 `GetProcAddress`。
 *
 * @par 位定时
 *  PCAN-Basic 要求把位定时**算好传进去**（与 SocketCAN 相反：那边由内核算）。
 *  本文件内置常用组合表；表中没有的组合返回 `JSDK_ERR_UNSUPPORTED`，
 *  让客户用 PCAN-View 算好再传（宁可不支持，也不要"猜一个"时钟参数 ——
 *  猜错的后果是现场莫名其妙的 error frame）。注意 FD 的位定时字符串依赖
 *  **适配器晶振**（多数是 80 MHz，也有 40/20 MHz）；本表按 80 MHz 给。
 */

#include "jsdk_hal_builtin.h"

#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include "hal_handle.h"

/* --------------------------------------------------------------------------
 * 平台动态库加载
 * ------------------------------------------------------------------------ */

/**
 * 符号句柄类型。
 *
 * ⚠ 不能声明成 `void *`：ISO C 禁止对象指针与函数指针互转，`-Wpedantic`
 *   会直接报错（本项目 `-Werror`）。Windows 原生的 `FARPROC` 就是函数指针；
 *   POSIX 的 `dlsym` 返回 `void *`，POSIX 保证可以转换，用 `memcpy` 完成
 *   这一步就不触发 ISO C 的诊断。
 */
#ifdef _WIN32
typedef HMODULE  sl_lib_t;
typedef FARPROC  sl_sym_t;
#define PCAN_LIB_NAME "PCANBasic.dll"
static sl_lib_t pcan_lib_open(void) { return LoadLibraryA(PCAN_LIB_NAME); }
static sl_sym_t pcan_sym(sl_lib_t l, const char *n) { return GetProcAddress(l, n); }
static void    pcan_lib_close(sl_lib_t l) { if (l) FreeLibrary(l); }
static uint32_t pcan_now_ms(void) { return (uint32_t)GetTickCount(); }
static const char *pcan_lib_hint(void)
{
    return "install PEAK's PCAN-Basic driver (PCANBasic.dll); 32/64-bit must "
           "match this process";
}
#else
#include <dlfcn.h>
#include <time.h>
typedef void (*sl_sym_t)(void);
typedef void *sl_lib_t;
static sl_lib_t pcan_lib_open(void)
{
    /* 顺序：Linux 常见名 → macOS 的 PCBUSB 兼容层 */
    sl_lib_t l = dlopen("libpcanbasic.so", RTLD_LAZY | RTLD_LOCAL);
    if (!l) l = dlopen("libPCANBasic.so", RTLD_LAZY | RTLD_LOCAL);
    if (!l) l = dlopen("libPCBUSB.dylib", RTLD_LAZY | RTLD_LOCAL);
    return l;
}
static sl_sym_t pcan_sym(sl_lib_t l, const char *n)
{
    void *p = dlsym(l, n);
    sl_sym_t f;
    memcpy(&f, &p, sizeof f);     /* POSIX 保证可行；memcpy 避开 ISO C 限制 */
    return f;
}
static void    pcan_lib_close(sl_lib_t l) { if (l) dlclose(l); }
static uint32_t pcan_now_ms(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0u;
    return (uint32_t)((uint64_t)ts.tv_sec * 1000ull
                      + (uint64_t)ts.tv_nsec / 1000000ull);
}
static const char *pcan_lib_hint(void)
{
    return "install libpcanbasic (PEAK) or PCBUSB (macOS); make sure the "
           "loader can find it (LD_LIBRARY_PATH / DYLD_LIBRARY_PATH)";
}
#endif

/**
 * 调用约定：PCAN-Basic 在 **32 位 Windows** 上是 `__stdcall`，用 cdecl
 * 函数指针调用会损坏栈。x64 只有一种约定，宏展开为空即可。
 */
#if defined(_WIN32) && !defined(__x86_64__)
#  define PCAN_CALL __stdcall
#else
#  define PCAN_CALL
#endif

/* --------------------------------------------------------------------------
 * PCAN-Basic ABI（只声明本文件用到的部分）
 *
 * 这些结构与常量来自 PCANBasic.h；此处**只复制用得到的**，避免把 PEAK 的
 * 头文件变成客户的构建依赖。数值是 PCAN-Basic 的**稳定 ABI**。
 * ------------------------------------------------------------------------ */

typedef uint32_t         pcan_status_t;   /* TPCANStatus */
typedef uint16_t         pcan_handle_t;   /* TPCANHandle（实际是 BYTE 的别名，用 WORD 更稳） */
typedef uint32_t         pcan_baud_t;     /* TPCANBaudrate：BTR0|BTR1<<8 */

typedef struct {
    uint32_t id;
    uint8_t  msgtype;
    uint8_t  len;
    uint8_t  data[8];
} pcan_msg_t;

typedef struct {
    uint32_t id;
    uint8_t  msgtype;
    uint8_t  dlc;
    uint8_t  data[64];
} pcan_msg_fd_t;

/* 消息类型位 */
#define PCAN_MSG_STANDARD 0x00u
#define PCAN_MSG_RTR      0x01u
#define PCAN_MSG_EXTENDED 0x02u
#define PCAN_MSG_FD       0x04u
#define PCAN_MSG_BRS      0x08u
#define PCAN_MSG_ESI      0x10u
#define PCAN_MSG_ERRFRAME 0x40u
#define PCAN_MSG_STATUS   0x80u

/* 状态码 */
#define PCAN_ERROR_OK          0x00000u
#define PCAN_ERROR_QOVERRUN    0x00002u
#define PCAN_ERROR_QXMTFULL    0x00004u
#define PCAN_ERROR_QRCVEMPTY   0x00020u
#define PCAN_ERROR_BUSLIGHT    0x00040u
#define PCAN_ERROR_BUSHEAVY    0x00080u
#define PCAN_ERROR_BUSWARNING  PCAN_ERROR_BUSHEAVY
#define PCAN_ERROR_BUSPASSIVE  0x40000u
#define PCAN_ERROR_BUSOFF      0x00010u
#define PCAN_ERROR_INITIALIZE  0x4000000u

/* 常用通道常量 */
#define PCAN_USBBUS1 0x51u
#define PCAN_USBBUS2 0x52u
#define PCAN_USBBUS3 0x53u
#define PCAN_USBBUS4 0x54u

/* RX 队列参数不做配置：默认就够（PCAN 默认队列 8192 帧） */

/* --------------------------------------------------------------------------
 * 位定时表
 * ------------------------------------------------------------------------ */

typedef struct {
    uint32_t bitrate;
    pcan_baud_t btr0btr1;
} pcan_baud_entry_t;

/**
 * Classic 位定时（PCAN-Basic 常量）。
 * 本协议在 ≤1 Mbps 时走 Classic，5 Mbps 时走 FD；但客户可能用别的组合。
 */
static const pcan_baud_entry_t pcan_baud_table[] = {
    { 1000000u, 0x0014u },
    {  800000u, 0x0016u },
    {  500000u, 0x001Cu },
    {  250000u, 0x011Cu },
    {  125000u, 0x031Cu },
    {  100000u, 0x432Fu },
    {   50000u, 0x472Fu },
    {   20000u, 0x532Fu },
    {   10000u, 0x672Fu }
};

typedef struct {
    uint32_t nom;       /**< 仲裁段波特率 */
    uint32_t data;      /**< 数据段波特率 */
    const char *str;    /**< PCAN-Basic 的 FD 位定时串（按 80 MHz 晶振） */
} pcan_fd_entry_t;

static const pcan_fd_entry_t pcan_fd_table[] = {
    { 1000000u, 5000000u,
      "f_clock_mhz=80,nom_brp=1,nom_tseg1=63,nom_tseg2=16,nom_sjw=16,"
      "data_brp=1,data_tseg1=15,data_tseg2=4,data_sjw=4" },
    { 1000000u, 2000000u,
      "f_clock_mhz=80,nom_brp=1,nom_tseg1=63,nom_tseg2=16,nom_sjw=16,"
      "data_brp=1,data_tseg1=39,data_tseg2=8,data_sjw=8" },
    {  500000u, 5000000u,
      "f_clock_mhz=80,nom_brp=2,nom_tseg1=63,nom_tseg2=16,nom_sjw=16,"
      "data_brp=1,data_tseg1=15,data_tseg2=4,data_sjw=4" },
    {  500000u, 2000000u,
      "f_clock_mhz=80,nom_brp=2,nom_tseg1=63,nom_tseg2=16,nom_sjw=16,"
      "data_brp=1,data_tseg1=39,data_tseg2=8,data_sjw=8" }
};

/* --------------------------------------------------------------------------
 * API 表（每个句柄一份，见文件头注释）
 * ------------------------------------------------------------------------ */

typedef struct {
    pcan_status_t (PCAN_CALL *initialize)(pcan_handle_t, pcan_baud_t, uint8_t,
                                          uint32_t, uint16_t);
    pcan_status_t (PCAN_CALL *initialize_fd)(pcan_handle_t, const char *);
    pcan_status_t (PCAN_CALL *uninitialize)(pcan_handle_t);
    pcan_status_t (PCAN_CALL *write)(pcan_handle_t, const pcan_msg_t *);
    pcan_status_t (PCAN_CALL *write_fd)(pcan_handle_t, const pcan_msg_fd_t *);
    pcan_status_t (PCAN_CALL *read)(pcan_handle_t, pcan_msg_t *, void *);
    pcan_status_t (PCAN_CALL *read_fd)(pcan_handle_t, pcan_msg_fd_t *, void *);
    pcan_status_t (PCAN_CALL *get_status)(pcan_handle_t);
    pcan_status_t (PCAN_CALL *reset)(pcan_handle_t);
} pcan_api_t;

typedef struct {
    struct jsdk_hal_handle base;

    sl_lib_t  lib;
    pcan_api_t api;
    pcan_handle_t channel;
    int       is_fd;

    uint32_t  tx_frames;
    uint32_t  rx_frames;
    uint32_t  tx_errors;
    uint32_t  rx_errors;
    uint32_t  bus_errors;
    uint32_t  err_latch;      /**< JSDK_HAL_BUS_* 锁存（同 socketcan 的理由） */
} pcan_handle_int_t;

static pcan_handle_int_t *pc(jsdk_hal_handle_t *h)
{
    return JSDK_HAL_CAST(pcan_handle_int_t, h);
}

static uint32_t pcan_now_ms_hal(void *user)
{
    (void)user;
    return pcan_now_ms();
}

/**
 * 把 PCAN 状态码映射到 SDK 的链路锁存位。
 *
 * @param count 1 = 这是"一次收发失败"，计入 `bus_errors`；0 = 只是**查询**链路
 *              状态（`bus_status` 里轮询 `CAN_GetStatus`），不计入。
 *              ⚠ 早先不分这两种情况，于是链路一坏，`bus_errors` 就随每次轮询
 *              飞涨，诊断计数完全失去意义。
 */
static void pcan_latch(pcan_handle_int_t *h, pcan_status_t st, int count)
{
    if (st == PCAN_ERROR_OK) return;

    if (st & PCAN_ERROR_BUSOFF)      h->err_latch |= JSDK_HAL_BUS_OFF;
    if (st & PCAN_ERROR_BUSPASSIVE)  h->err_latch |= JSDK_HAL_BUS_ERROR_PASS;
    if (st & PCAN_ERROR_BUSHEAVY)    h->err_latch |= JSDK_HAL_BUS_ERROR_WARN;
    if (st & PCAN_ERROR_BUSLIGHT)    h->err_latch |= JSDK_HAL_BUS_ERROR_WARN;
    if (st & PCAN_ERROR_QOVERRUN) {
        h->rx_errors++;
        h->err_latch |= JSDK_HAL_BUS_ERROR_WARN;
    }
    if (st & PCAN_ERROR_INITIALIZE)  h->err_latch |= JSDK_HAL_BUS_OFF;
    if (count) h->bus_errors++;
}

/* --------------------------------------------------------------------------
 * HAL 回调
 * ------------------------------------------------------------------------ */

static int pcan_send(void *user, const jsdk_can_frame_t *f)
{
    pcan_handle_int_t *h = pc((jsdk_hal_handle_t *)user);
    pcan_status_t st;

    if (!h || !h->lib || !f) return -1;

    if (f->flags & JSDK_FRAME_FD) {
        pcan_msg_fd_t m;
        if (!h->api.write_fd) return -1;
        if (f->len > 64u) return -1;

        memset(&m, 0, sizeof m);
        m.id = f->id;
        m.msgtype = (uint8_t)((f->flags & JSDK_FRAME_EXT ? PCAN_MSG_EXTENDED
                                                         : PCAN_MSG_STANDARD)
                              | PCAN_MSG_FD
                              | (f->flags & JSDK_FRAME_BRS ? PCAN_MSG_BRS : 0u));
        m.dlc = f->len;
        if (f->len) memcpy(m.data, f->data, f->len);
        st = h->api.write_fd(h->channel, &m);
    } else {
        pcan_msg_t m;
        if (f->len > 8u) return -1;

        memset(&m, 0, sizeof m);
        m.id = f->id;
        m.msgtype = (uint8_t)(f->flags & JSDK_FRAME_EXT ? PCAN_MSG_EXTENDED
                                                        : PCAN_MSG_STANDARD);
        m.len = f->len;
        if (f->len) memcpy(m.data, f->data, f->len);
        st = h->api.write(h->channel, &m);
    }

    if (st != PCAN_ERROR_OK) {
        h->tx_errors++;
        pcan_latch(h, st, 1);
        return -1;
    }
    h->tx_frames++;
    h->err_latch = 0u;              /* 发出去了 → 链路活着 */
    return 0;
}

static int pcan_recv(void *user, jsdk_can_frame_t *f)
{
    pcan_handle_int_t *h = pc((jsdk_hal_handle_t *)user);
    pcan_status_t st;
    int guard = 0;

    if (!h || !h->lib || !f) return -1;

    /*
     * 循环一次：PCAN 的队列里可能混着"状态帧"（`PCAN_MESSAGE_STATUS`）和
     * 错误帧，它们不是数据。跳过它们而不是把它们交给协议层 —— 否则
     * `cb_frame` 会拿到一个 id 完全无意义的帧。
     *
     * guard 上限防止固件/驱动异常时死循环（每周期 SDK 已经会反复调 recv）。
     */
    for (guard = 0; guard < 16; ++guard) {
        if (h->api.read_fd) {
            pcan_msg_fd_t m;
            memset(&m, 0, sizeof m);
            st = h->api.read_fd(h->channel, &m, NULL);
            if (st == PCAN_ERROR_QRCVEMPTY) return 0;
            if (st != PCAN_ERROR_OK) { h->rx_errors++; pcan_latch(h, st, 1); return -1; }
            if (m.msgtype & (PCAN_MSG_STATUS | PCAN_MSG_ERRFRAME)) {
                h->rx_errors++;
                pcan_latch(h, PCAN_ERROR_BUSHEAVY, 1);   /* 有错误帧就说明链路不干净 */
                continue;
            }
            if (m.msgtype & PCAN_MSG_RTR) { h->rx_errors++; continue; }

            memset(f, 0, sizeof *f);
            f->id = m.id;
            f->flags = (uint8_t)((m.msgtype & PCAN_MSG_EXTENDED ? JSDK_FRAME_EXT : 0u)
                                 | ((m.msgtype & PCAN_MSG_FD) ? JSDK_FRAME_FD : 0u)
                                 | ((m.msgtype & PCAN_MSG_BRS) ? JSDK_FRAME_BRS : 0u));
            f->len = (m.dlc > 64u) ? 64u : m.dlc;
            if (f->len) memcpy(f->data, m.data, f->len);
            h->rx_frames++;
            return 1;
        } else {
            pcan_msg_t m;
            memset(&m, 0, sizeof m);
            st = h->api.read(h->channel, &m, NULL);
            if (st == PCAN_ERROR_QRCVEMPTY) return 0;
            if (st != PCAN_ERROR_OK) { h->rx_errors++; pcan_latch(h, st, 1); return -1; }
            if (m.msgtype & (PCAN_MSG_STATUS | PCAN_MSG_ERRFRAME)) { h->rx_errors++; continue; }
            if (m.msgtype & PCAN_MSG_RTR) { h->rx_errors++; continue; }

            memset(f, 0, sizeof *f);
            f->id = m.id;
            f->flags = (uint8_t)(m.msgtype & PCAN_MSG_EXTENDED ? JSDK_FRAME_EXT : 0u);
            f->len = (m.len > 8u) ? 8u : m.len;
            if (f->len) memcpy(f->data, m.data, f->len);
            h->rx_frames++;
            return 1;
        }
    }
    return 0;   /* 连读 16 次都是状态/错误帧：本周期先不报数据帧 */
}

static int pcan_bus_status(void *user, uint32_t *flags)
{
    pcan_handle_int_t *h = pc((jsdk_hal_handle_t *)user);
    pcan_status_t st;

    if (!h || !flags) return -1;
    *flags = 0u;

    if (h->api.get_status) {
        st = h->api.get_status(h->channel);
        if (st == PCAN_ERROR_OK) *flags |= JSDK_HAL_BUS_OK;
        else pcan_latch(h, st, 0);   /* 只是查询，不计数 */
    } else {
        *flags |= JSDK_HAL_BUS_OK;
    }
    *flags |= h->err_latch;
    return 0;
}

static void pcan_destroy(struct jsdk_hal_handle *base)
{
    pcan_handle_int_t *h = JSDK_HAL_CAST(pcan_handle_int_t, base);
    if (!h) return;
    if (h->lib) {
        if (h->api.uninitialize) (void)h->api.uninitialize(h->channel);
        pcan_lib_close(h->lib);       /* ⚠ 释放驱动句柄，否则反复 open 会漏 */
    }
    free(h);
}

/* --------------------------------------------------------------------------
 * 打开
 * ------------------------------------------------------------------------ */

/** 解析通道名："PCAN_USBBUS1" / "can0" / "0x51" 都认。 */
static int pcan_parse_channel(const char *name, pcan_handle_t *out)
{
    if (!name || !name[0]) return 0;

    if (strncmp(name, "PCAN_USBBUS", 11) == 0) {
        int n = (int)(name[11] - '0');
        if (n >= 1 && n <= 4) { *out = (pcan_handle_t)(PCAN_USBBUS1 + (unsigned)(n - 1)); return 1; }
        return 0;
    }
    if (strncmp(name, "can", 3) == 0) {
        int n = (int)(name[3] - '0');
        if (n >= 0 && n <= 3) { *out = (pcan_handle_t)(PCAN_USBBUS1 + (unsigned)n); return 1; }
        return 0;
    }
    if (strncmp(name, "0x", 2) == 0 || strncmp(name, "0X", 2) == 0) {
        char *end = NULL;
        unsigned long v = strtoul(name, &end, 16);
        if (end && *end == '\0' && v <= 0xFFul) { *out = (pcan_handle_t)v; return 1; }
        return 0;
    }
    return 0;
}

/**
 * 把动态库里的符号绑到一个类型化的函数指针字段上。
 *
 * ⚠ 用 `memcpy` 而不是强制转换：`-Wextra` 会开 `-Wcast-function-type`，
 *   在函数指针之间"跨签名"强制转换会被判为错误（本项目 `-Werror`）。
 *   两边都是函数指针、在同一平台上大小相同，`memcpy` 是明确的位拷贝，
 *   不触发任何诊断（而对象指针 ↔ 函数指针的转换在 ISO C 里本来也是非法的）。
 */
#define PCAN_BIND(field, sym)                                            \
    do {                                                                 \
        sl_sym_t pc_bind_ = pcan_sym(h->lib, (sym));                     \
        memcpy(&h->api.field, &pc_bind_, sizeof pc_bind_);               \
    } while (0)

static int pcan_resolve(pcan_handle_int_t *h)
{
    PCAN_BIND(initialize,    "CAN_Initialize");
    PCAN_BIND(initialize_fd, "CAN_InitializeFD");
    PCAN_BIND(uninitialize,  "CAN_Uninitialize");
    PCAN_BIND(write,         "CAN_Write");
    PCAN_BIND(write_fd,      "CAN_WriteFD");
    PCAN_BIND(read,          "CAN_Read");
    PCAN_BIND(read_fd,       "CAN_ReadFD");
    PCAN_BIND(get_status,    "CAN_GetStatus");
    PCAN_BIND(reset,         "CAN_Reset");

    /* Classic 三个是硬性要求；FD 三个可以缺（驱动太旧时退化为 Classic only） */
    return (h->api.initialize && h->api.uninitialize
            && h->api.write && h->api.read) ? 1 : 0;
}

#undef PCAN_BIND

jsdk_status_t jsdk_hal_pcan_open(jsdk_can_hal_t *hal, jsdk_hal_handle_t **out,
                                 const char *channel,
                                 uint32_t bitrate, uint32_t data_bitrate)
{
    pcan_handle_int_t *h;
    pcan_status_t st;
    int want_fd;
    size_t i;

    if (!hal || !out || !channel || !channel[0]) return JSDK_ERR_INVALID_ARG;
    *out = NULL;

    /* ⚠ 不替客户猜通道：写错通道名就报错，比“静默连到 1 号口”安全得多 */
    if (bitrate == 0u) bitrate = 1000000u;
    want_fd = (data_bitrate != 0u) ? 1 : 0;

    h = (pcan_handle_int_t *)calloc(1u, sizeof *h);
    if (!h) return JSDK_ERR_NO_MEMORY;

    if (!pcan_parse_channel(channel, &h->channel)) {
        free(h);
        return JSDK_ERR_INVALID_ARG;      /* 通道名不认识：别猜 */
    }

    h->lib = pcan_lib_open();
    if (!h->lib) {
        free(h);
        return JSDK_ERR_NOT_FOUND;        /* 驱动没装；提示见下 */
    }
    if (!pcan_resolve(h)) {
        pcan_lib_close(h->lib);
        free(h);
        return JSDK_ERR_UNSUPPORTED;      /* PCANBasic 版本太旧 */
    }

    /* --- 初始化 --- */
    if (want_fd) {
        const char *spec = NULL;

        if (!h->api.initialize_fd) {
            pcan_lib_close(h->lib);
            free(h);
            return JSDK_ERR_UNSUPPORTED;  /* 驱动不支持 FD */
        }
        for (i = 0u; i < sizeof pcan_fd_table / sizeof pcan_fd_table[0]; ++i) {
            if (pcan_fd_table[i].nom == bitrate && pcan_fd_table[i].data == data_bitrate) {
                spec = pcan_fd_table[i].str;
                break;
            }
        }
        if (!spec) {
            pcan_lib_close(h->lib);
            free(h);
            return JSDK_ERR_UNSUPPORTED;  /* 位定时组合不在表里；宁可不猜 */
        }
        st = h->api.initialize_fd(h->channel, spec);
    } else {
        pcan_baud_t btr = 0u;

        for (i = 0u; i < sizeof pcan_baud_table / sizeof pcan_baud_table[0]; ++i) {
            if (pcan_baud_table[i].bitrate == bitrate) {
                btr = pcan_baud_table[i].btr0btr1;
                break;
            }
        }
        if (btr == 0u) {
            pcan_lib_close(h->lib);
            free(h);
            return JSDK_ERR_UNSUPPORTED;
        }
        /* HwType / IOPort / Interrupt 对 USB 通道无意义：0 即可 */
        st = h->api.initialize(h->channel, btr, 0u, 0u, 0u);
    }

    if (st != PCAN_ERROR_OK) {
        if (h->api.uninitialize) (void)h->api.uninitialize(h->channel);
        pcan_lib_close(h->lib);
        free(h);
        /* 通道被 PCAN-View 等占用 / 设备没插：属于"没找到" */
        return (st & PCAN_ERROR_INITIALIZE) ? JSDK_ERR_BUSY : JSDK_ERR_NOT_FOUND;
    }

    h->base.kind    = JSDK_HAL_KIND_PCAN;
    h->base.destroy = pcan_destroy;
    h->is_fd        = want_fd;

    hal->user       = h;
    hal->send       = pcan_send;
    hal->recv       = pcan_recv;
    hal->now_ms     = pcan_now_ms_hal;
    hal->on_error   = NULL;
    hal->bus_status = pcan_bus_status;

    *out = &h->base;
    return JSDK_OK;
}

/* --------------------------------------------------------------------------
 * 诊断
 * ------------------------------------------------------------------------ */

/** 装驱动时的提示文本（CLI 在 JSDK_ERR_NOT_FOUND 时打印它）。 */
const char *jsdk_hal_pcan_driver_hint(void);
const char *jsdk_hal_pcan_driver_hint(void)
{
    return pcan_lib_hint();
}

/** 取诊断计数：`tx / rx / bus_err`（任一可为 NULL）。 */
void jsdk_hal_pcan_stats(jsdk_hal_handle_t *h, uint32_t *tx, uint32_t *rx,
                         uint32_t *bus_err);
void jsdk_hal_pcan_stats(jsdk_hal_handle_t *h, uint32_t *tx, uint32_t *rx,
                         uint32_t *bus_err)
{
    pcan_handle_int_t *s = pc(h);
    if (!s) return;
    if (tx)      *tx      = s->tx_frames;
    if (rx)      *rx      = s->rx_frames;
    if (bus_err) *bus_err = s->bus_errors;
}
