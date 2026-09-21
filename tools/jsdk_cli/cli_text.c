/**
 * @file    cli_text.c
 * @brief   终端文本输出的实现（见 cli_text.h 说明）
 */

#include "cli_text.h"

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#  include <windows.h>
#  include <io.h>
#endif

/** 需要跟踪"未完成 UTF-8 尾部"的流数（stdout/stderr + 测试的 tmpfile + 余量）。 */
#define CLI_TEXT_SLOT_MAX 6u

/** 转换输出缓冲：CP936 最坏 2 字节/码点，UTF-8 每块最多 CLI_TEXT_CHUNK 个码点。 */
#define CLI_TEXT_OUT_MAX (CLI_TEXT_CHUNK * 2u + 8u)

typedef struct {
    FILE    *f;
    int      is_console;                /**< 1 = 写之前要转码 */
    unsigned cp;                        /**< 控制台代码页 */
    char     tail[4];                   /**< 上一次没写完的 UTF-8 尾部（0~3 字节） */
    size_t   tail_n;
} cli_slot_t;

static cli_slot_t       g_slots[CLI_TEXT_SLOT_MAX];
static cli_text_conv_fn g_conv;         /**< NULL → cli_text_conv() */
static int              g_force_on;     /**< 测试：把所有流当控制台 */
static unsigned         g_force_cp;

/* 环境变量 JSDK_CLI_TEXT：0 = 未设（自动）；1 = 强制 UTF-8 直通；2 = 强制控制台 + 指定代码页 */
static int      g_env_mode;
static unsigned g_env_cp;
static int      g_env_checked;

/* --------------------------------------------------------------------------
 * 纯逻辑
 * ------------------------------------------------------------------------ */

size_t cli_utf8_complete_prefix(const char *s, size_t n)
{
    size_t i  = 0u;
    size_t ok = 0u;

    while (i < n) {
        unsigned char c = (unsigned char)s[i];
        size_t        need;
        size_t        k;
        int           bad = 0;

        if (c < 0x80u)                  need = 1u;
        else if ((c & 0xE0u) == 0xC0u)  need = 2u;
        else if ((c & 0xF0u) == 0xE0u)  need = 3u;
        else if ((c & 0xF8u) == 0xF0u)  need = 4u;
        else { i += 1u; ok = i; continue; }      /* 非法前导字节：按 1 字节前进 */

        if (i + need > n) break;                 /* 尾部不完整：留给下一次 */

        for (k = 1u; k < need; ++k) {
            if (((unsigned char)s[i + k] & 0xC0u) != 0x80u) { bad = 1; break; }
        }
        if (bad) { i += 1u; ok = i; continue; }  /* 续字节坏了：按 1 字节前进 */

        i += need;
        ok = i;
    }
    return ok;
}

/* --------------------------------------------------------------------------
 * 转码
 * ------------------------------------------------------------------------ */

size_t cli_text_conv(unsigned cp, const char *utf8, size_t n, char *out, size_t cap)
{
    if (n == 0u) return 0u;

#ifndef _WIN32
    /* Linux/macOS：终端本来就是 UTF-8，没有"控制台代码页"这回事。 */
    (void)cp;
    if (n > cap) return 0u;
    memcpy(out, utf8, n);
    return n;
#else
    {
        wchar_t wbuf[CLI_TEXT_CHUNK + 1u];
        int     wlen;
        int     m;

        if (n > (size_t)CLI_TEXT_CHUNK) return 0u;      /* 调用方必须先切块 */
        if (cp == 0u || cp == 65001u) {                 /* UTF-8 直通 */
            if (n > cap) return 0u;
            memcpy(out, utf8, n);
            return n;
        }
        wlen = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, utf8, (int)n,
                                   wbuf, (int)(CLI_TEXT_CHUNK + 1u));
        if (wlen <= 0) return 0u;                       /* 非法 UTF-8 → 降级原样写 */
        m = WideCharToMultiByte((UINT)cp, WC_NO_BEST_FIT_CHARS, wbuf, wlen,
                                out, (int)cap, "?", NULL);
        if (m <= 0) return 0u;                          /* 缓冲不足/不支持 → 降级 */
        return (size_t)m;
    }
#endif
}

/* --------------------------------------------------------------------------
 * 流的判别与状态
 * ------------------------------------------------------------------------ */

static int detect_console(FILE *f, unsigned *cp)
{
    *cp = 0u;
#ifdef _WIN32
    {
        int      fd;
        intptr_t h;
        DWORD    mode;

        if (!f) return 0;
        fd = _fileno(f);
        if (fd < 0) return 0;
        if (_isatty(fd) == 0) return 0;                 /* 管道/文件：原样写 UTF-8 */
        h = _get_osfhandle(fd);
        if (h == (intptr_t)-1) return 0;
        if (!GetConsoleMode((HANDLE)h, &mode)) return 0;
        *cp = (unsigned)GetConsoleOutputCP();
        return 1;
    }
#else
    (void)f;
    return 0;
#endif
}

static void env_check(void)
{
    const char *v;
    char       *end = NULL;
    unsigned long cp;

    if (g_env_checked) return;
    g_env_checked = 1;

    v = getenv("JSDK_CLI_TEXT");
    if (!v || !*v) return;                              /* 未设 → 自动判别 */

    if (strcmp(v, "utf8") == 0 || strcmp(v, "off") == 0) {
        g_env_mode = 1;
        return;
    }
    cp = strtoul(v, &end, 10);
    if (end != v && *end == '\0' && cp > 0u && cp <= 65535u) {
        g_env_mode = 2;
        g_env_cp   = (unsigned)cp;
        return;
    }
    /* 值不认识：不折腾，按自动走（并在 stderr 上提醒一句）。 */
    (void)fprintf(stderr, "jsdk-cli: 忽略不可识别的 JSDK_CLI_TEXT=%s"
                          "（用 utf8 或代码页数字，如 936）\n", v);
}

static cli_slot_t *slot_for(FILE *f)
{
    unsigned    i;
    cli_slot_t *slot = NULL;
    if (!f) f = stdout;
    for (i = 0u; i < CLI_TEXT_SLOT_MAX; ++i) {
        if (g_slots[i].f == f) return &g_slots[i];
        if (!g_slots[i].f && !slot) slot = &g_slots[i];
    }
    if (!slot) slot = &g_slots[0];                      /* 表满：复用 0 号（只丢尾巴状态） */

    memset(slot, 0, sizeof *slot);
    slot->f = f;

    env_check();
    if (g_force_on) {                                   /* 测试注入优先 */
        slot->is_console = 1;
        slot->cp         = g_force_cp;
    } else if (g_env_mode == 2) {                       /* 强制控制台 + 代码页 */
        slot->is_console = 1;
        slot->cp         = g_env_cp;
    } else if (g_env_mode == 1) {                       /* 强制 UTF-8 直通 */
        slot->is_console = 0;
    } else {
        slot->is_console = detect_console(f, &slot->cp);
    }
    return slot;
}

/* --------------------------------------------------------------------------
 * 写
 * ------------------------------------------------------------------------ */

/** 纯 ASCII 块不需要转码（任何目标代码页下字节都一样）。 */
static int all_ascii(const char *p, size_t n)
{
    size_t i;
    for (i = 0u; i < n; ++i) {
        if ((unsigned char)p[i] >= 0x80u) return 0;
    }
    return 1;
}

static void slot_write(cli_slot_t *s, const char *p, size_t n)
{
    char cbuf[CLI_TEXT_OUT_MAX];

    while (n > 0u) {
        size_t safe = (n > (size_t)CLI_TEXT_CHUNK) ? (size_t)CLI_TEXT_CHUNK : n;

        /* 块边界也可能落在码点中间 → 退到最后一个完整码点 */
        safe = cli_utf8_complete_prefix(p, safe);
        if (safe == 0u) break;                          /* 只有"不完整"，等下次 */

        if (s->is_console && !all_ascii(p, safe)) {
            size_t m = g_conv ? g_conv(s->cp, p, safe, cbuf, sizeof cbuf)
                              : cli_text_conv(s->cp, p, safe, cbuf, sizeof cbuf);
            if (m > 0u) {
                (void)fwrite(cbuf, 1u, m, s->f);
            } else {
                (void)fwrite(p, 1u, safe, s->f);        /* 转码不可行：至少别丢内容 */
            }
        } else {
            /*
             * 非控制台，或**纯 ASCII 块**：原样写。
             * ASCII 在所有目标代码页里都是同一组字节，白转一次没有意义 ——
             * 而 CLI 的大部分输出（数字、字段名、JSON）都是纯 ASCII。
             */
            (void)fwrite(p, 1u, safe, s->f);
        }
        p += safe;
        n -= safe;
    }
}

static void slot_put(cli_slot_t *s, const char *data, size_t n)
{
    char   carry[CLI_TEXT_CHUNK + 4u];
    size_t cn  = s->tail_n;
    size_t off = 0u;

    if (cn > 0u) {
        memcpy(carry, s->tail, cn);
        s->tail_n = 0u;
    }

    while (off < n) {
        size_t room = sizeof carry - cn;
        size_t take = (n - off) < room ? (n - off) : room;
        size_t safe;

        if (take == 0u) {                               /* carry 满：先写掉 */
            slot_write(s, carry, cn);
            cn = 0u;
            continue;
        }
        memcpy(carry + cn, data + off, take);
        cn  += take;
        off += take;

        safe = cli_utf8_complete_prefix(carry, cn);
        if (safe > 0u) {
            slot_write(s, carry, safe);
            memmove(carry, carry + safe, cn - safe);
            cn -= safe;
        }
    }

    if (cn > sizeof s->tail) {                          /* 理论上到不了这里（防御） */
        slot_write(s, carry, cn);
        cn = 0u;
    }
    if (cn > 0u) memcpy(s->tail, carry, cn);
    s->tail_n = cn;
}

/* --------------------------------------------------------------------------
 * 公开接口
 * ------------------------------------------------------------------------ */

/** 有界 strlen（`strnlen` 不是所有老工具链都有，自己算）。 */
static size_t bounded_len(const char *s, size_t cap)
{
    size_t n = 0u;
    while (n < cap && s[n] != '\0') ++n;
    return n;
}

static int put_fmt(FILE *f, const char *fmt, va_list ap)
{
    char    stack[1024];
    char   *buf = stack;
    char   *heap = NULL;
    size_t  len;
    int     n;
    va_list ap2;

    /*
     * ⚠ `vsnprintf` 会**消耗** va_list，所以要"重放"就必须先 `va_copy`。
     *   先 printf 再 copy 的写法能编过，但第二次拿到的是已经被用过的
     *   va_list —— 内容未定义（表现为长度对、内容乱）。
     */
    va_copy(ap2, ap);

    n = vsnprintf(stack, sizeof stack, fmt, ap);
    if (n < 0) {
        va_end(ap2);
        return n;                                       /* 编码错：什么都不写 */
    }

    if ((size_t)n >= sizeof stack) {                    /* 截断了 → 换堆上重来 */
        heap = (char *)malloc((size_t)n + 1u);
        if (heap) {
            (void)vsnprintf(heap, (size_t)n + 1u, fmt, ap2);
            buf = heap;
            len = (size_t)n;
        } else {
            len = bounded_len(stack, sizeof stack - 1u); /* 堆也没了：写能写的 */
        }
    } else {
        len = (size_t)n;
    }
    va_end(ap2);

    slot_put(slot_for(f), buf, len);
    free(heap);
    return n;
}

int cli_fprintf(FILE *f, const char *fmt, ...)
{
    va_list ap;
    int     n;

    va_start(ap, fmt);
    n = put_fmt(f, fmt, ap);
    va_end(ap);
    return n;
}

int cli_printf(const char *fmt, ...)
{
    va_list ap;
    int     n;

    va_start(ap, fmt);
    n = put_fmt(stdout, fmt, ap);
    va_end(ap);
    return n;
}

int cli_fputs(const char *s, FILE *f)
{
    if (!s) s = "(null)";
    slot_put(slot_for(f), s, strlen(s));
    return 0;
}

int cli_puts(const char *s)
{
    (void)cli_fputs(s, stdout);
    slot_put(slot_for(stdout), "\n", 1u);
    return 0;
}

int cli_fputc(int c, FILE *f)
{
    char b = (char)(unsigned char)c;
    slot_put(slot_for(f), &b, 1u);
    return (int)(unsigned char)c;
}

void cli_text_flush(FILE *f)
{
    if (!f) {
        (void)fflush(stdout);
        (void)fflush(stderr);
        return;
    }
    (void)fflush(f);
}

/* --------------------------------------------------------------------------
 * 测试注入点
 * ------------------------------------------------------------------------ */

void cli_text_set_conv(cli_text_conv_fn fn)
{
    g_conv = fn;
}

void cli_text_set_console_force(int on, unsigned cp)
{
    unsigned i;

    g_force_on = on ? 1 : 0;
    g_force_cp = cp;
    for (i = 0u; i < CLI_TEXT_SLOT_MAX; ++i) g_slots[i].f = NULL;   /* 重新判别 */
}
