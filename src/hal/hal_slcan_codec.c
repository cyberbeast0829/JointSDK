/**
 * @file    hal_slcan_codec.c
 * @brief   slcan ASCII 行协议编解码（纯逻辑）
 */

#include "hal_slcan_codec.h"

#include <string.h>

/* --------------------------------------------------------------------------
 * 工具
 * ------------------------------------------------------------------------ */

static int hex_val(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static char hex_digit(unsigned v)
{
    return (v < 10u) ? (char)('0' + v) : (char)('A' + (v - 10u));
}

/** 解析 @p n 个十六进制字符。失败返回 0。 */
static int parse_hex(const char *s, size_t n, uint32_t *out)
{
    uint32_t v = 0u;
    size_t   i;

    if (n == 0u) return 0;
    for (i = 0u; i < n; ++i) {
        int d = hex_val(s[i]);
        if (d < 0) return 0;
        v = (v << 4) | (uint32_t)d;
    }
    *out = v;
    return 1;
}

/* --------------------------------------------------------------------------
 * CAN FD 的长度码（CANable 2.0 / python-can `CAN_FD_DLC` 同一张表）
 * ------------------------------------------------------------------------ */

/** FD DLC 码 → 字节数。`0..8` 直映，之后按 4/8/16/32 递增。 */
static const uint8_t k_fd_len[16] = {
    0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u,
    8u, 12u, 16u, 20u, 24u, 32u, 48u, 64u
};

int jsdk_slcan_fd_len_from_dlc(unsigned code)
{
    if (code > 15u) return -1;
    return (int)k_fd_len[code];
}

int jsdk_slcan_fd_dlc_from_len(unsigned len)
{
    unsigned i;

    if (len > 64u) return -1;
    for (i = 0u; i < 16u; ++i) {
        if ((unsigned)k_fd_len[i] == len) return (int)i;
    }
    return -1;                  /* 9 / 10 / 11：FD 里没有对应码 */
}

/* --------------------------------------------------------------------------
 * 编码
 * ------------------------------------------------------------------------ */

size_t jsdk_slcan_encode(const jsdk_can_frame_t *f, char *out, size_t cap)
{
    size_t   n = 0u;
    int      ext;
    int      fd;
    int      brs;
    unsigned dlc;
    unsigned len;
    uint8_t  i;
    char     idbuf[9];
    size_t   idlen;
    unsigned k;

    if (!f || !out) return 0u;

    fd  = (f->flags & JSDK_FRAME_FD) ? 1 : 0;
    ext = (f->flags & JSDK_FRAME_EXT) ? 1 : 0;
    brs = (f->flags & JSDK_FRAME_BRS) ? 1 : 0;
    len = (unsigned)f->len;

    if (fd) {
        int code = jsdk_slcan_fd_dlc_from_len(len);
        if (code < 0) return 0u;          /* 9/10/11 或 >64：FD 表达不了 */
        dlc = (unsigned)code;
    } else {
        if (len > 8u) return 0u;          /* Classic 上限 8 */
        dlc = len;
    }

    /*
     * 位宽固定（不是"去掉前导零"）：
     *   标准帧 3 位、扩展帧 8 位。多数固件接受短写，但固定宽度是所有
     *   固件实现都能解析的公共子集，而且长度可预测（便于定长缓冲）。
     */
    idlen = ext ? 8u : 3u;
    for (k = 0u; k < idlen; ++k) {
        unsigned shift = (unsigned)(4u * (idlen - 1u - k));
        idbuf[k] = hex_digit((unsigned)((f->id >> shift) & 0xFu));
    }

    /* 容量检查：1 + idlen + 1 + 2*len + 1(\r) */
    if (cap < 1u + idlen + 1u + (size_t)len * 2u + 1u) return 0u;

    /*
     * 前缀字母：
     *   Classic： t / T
     *   FD BRS=0： d / D     ← 注意：不带 BRS 用 d/D
     *   FD BRS=1： b / B     ← 带 BRS 用 b/B（反直觉，见头文件）
     */
    if (!fd)          out[n++] = ext ? 'T' : 't';
    else if (brs)     out[n++] = ext ? 'B' : 'b';
    else              out[n++] = ext ? 'D' : 'd';

    for (k = 0u; k < idlen; ++k) out[n++] = idbuf[k];

    /* DLC 恒显式输出（含 '0'）：省略会被解释为 0 字节，那没问题；但显式写
       出来最有助干现场抓串口波形时对账。FD 的 DLC 是**长度码**而不是字节数。 */
    out[n++] = hex_digit(dlc);

    for (i = 0u; i < len; ++i) {
        out[n++] = hex_digit((unsigned)((f->data[i] >> 4) & 0xFu));
        out[n++] = hex_digit((unsigned)(f->data[i] & 0xFu));
    }

    out[n++] = '\r';
    return n;
}

/* --------------------------------------------------------------------------
 * 解码
 * ------------------------------------------------------------------------ */

/**
 * 逐字符扫描一行，直到 `\r` 或结尾。
 * @return 行内容长度（不含 `\r`）；`consumed` 填"含 `\r` 的消费量"。
 *         `*complete` = 是否真的遇到了行尾。
 */
static size_t scan_line(const char *line, size_t len,
                        size_t *consumed, int *complete)
{
    size_t i;

    for (i = 0u; i < len; ++i) {
        if (line[i] == '\r') {
            *consumed = i + 1u;
            *complete = 1;
            return i;
        }
    }
    *consumed = len;
    *complete = 0;
    return len;
}

jsdk_slcan_rc_t jsdk_slcan_decode(const char *line, size_t len,
                                  jsdk_can_frame_t *f, size_t *consumed)
{
    size_t   n;
    size_t   take = 0u;
    int      complete = 0;
    char     kind;
    int      ext;
    int      fd;
    int      brs;
    size_t   idlen;
    uint32_t id = 0u;
    size_t   p;
    unsigned dlc_code;
    unsigned expect_payload;
    unsigned i;
    uint8_t  flags;

    if (!line) return JSDK_SLCAN_MALFORMED;

    n = scan_line(line, len, &take, &complete);
    if (consumed) *consumed = take;
    if (!complete) return JSDK_SLCAN_NEED_MORE;
    if (n == 0u) return JSDK_SLCAN_ACK;          /* 仅 \r = 固件确认 */

    kind = line[0];

    if (kind == '\a') return JSDK_SLCAN_NACK;

    /*
     * 前缀字母表：小写 = 标准帧、大写 = 扩展帧（两者一致）。
     *   t/T Classic；d/D FD 且 BRS=0；b/B FD 且 BRS=1。
     */
    switch (kind) {
    case 't': ext = 0; fd = 0; brs = 0; idlen = 3u; break;
    case 'T': ext = 1; fd = 0; brs = 0; idlen = 8u; break;
    case 'd': ext = 0; fd = 1; brs = 0; idlen = 3u; break;
    case 'D': ext = 1; fd = 1; brs = 0; idlen = 8u; break;
    case 'b': ext = 0; fd = 1; brs = 1; idlen = 3u; break;
    case 'B': ext = 1; fd = 1; brs = 1; idlen = 8u; break;
    case 'r': return JSDK_SLCAN_RTR;    /* 标准 RTR：合法但不支持 */
    case 'R': return JSDK_SLCAN_RTR;    /* 扩展 RTR */
    default:
        /*
         * 还有一批"状态行"（如 `F00`、`V1013`、`s8`、`S8`、版本号等）。
         * 它们不是帧，也不是错误：返回 MALFORMED 让调用方**丢掉整行**并继续，
         * 而不是跳掉一个字节 —— 后者会把后续行切错位。
         */
        return JSDK_SLCAN_MALFORMED;
    }

    if (n < 1u + idlen) return JSDK_SLCAN_MALFORMED;
    if (!parse_hex(line + 1, idlen, &id)) return JSDK_SLCAN_MALFORMED;

    p = 1u + idlen;

    /*
     * ⚠ DLC 省略 = **0 字节**，不是 8 字节。
     *   Lawicel 规范里 DLC 字符“可以省略”，省略即 0；真正发 8 字节的帧
     *   一定会写 `8` 并且后面跟着 16 个十六进制字符。
     *   （本文件早先写成“省略即 8”，被单元测试当场拓出来 —— 那样 `t123`
     *     会被解成“DLC=8 但没给数据”进而判为 MALFORMED，既解不出零长帧，
     *     也会把一行合法帧报成错误。）
     */
    if (p == n) {
        dlc_code = 0u;
    } else {
        int d = hex_val(line[p]);
        if (d < 0) return JSDK_SLCAN_MALFORMED;
        dlc_code = (unsigned)d;
        p++;
    }

    if (fd) {
        /* FD：DLC 是**长度码**（0..8 直映，9→12 … F→64），一位十六进制 */
        int bytes = jsdk_slcan_fd_len_from_dlc(dlc_code);
        if (bytes < 0) return JSDK_SLCAN_MALFORMED;
        expect_payload = (unsigned)bytes * 2u;
    } else {
        if (dlc_code > 8u) return JSDK_SLCAN_MALFORMED;
        expect_payload = dlc_code * 2u;
    }

    if (n < p + expect_payload) return JSDK_SLCAN_MALFORMED;
    if (n != p + expect_payload) return JSDK_SLCAN_MALFORMED;   /* 尾部有多余字符 */

    if (!f) return JSDK_SLCAN_MALFORMED;

    flags = (uint8_t)(ext ? JSDK_FRAME_EXT : 0u);
    if (fd)  flags = (uint8_t)(flags | JSDK_FRAME_FD);
    if (brs) flags = (uint8_t)(flags | JSDK_FRAME_BRS);

    memset(f, 0, sizeof *f);
    f->id    = id;
    f->flags = flags;
    f->len   = (uint8_t)(expect_payload / 2u);

    for (i = 0u; i < (unsigned)f->len; ++i) {
        int hi = hex_val(line[p + i * 2u]);
        int lo = hex_val(line[p + i * 2u + 1u]);
        if (hi < 0 || lo < 0) return JSDK_SLCAN_MALFORMED;
        f->data[i] = (uint8_t)((hi << 4) | lo);
    }

    return JSDK_SLCAN_OK;
}
