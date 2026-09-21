/**
 * @file    test_cli_text.c
 * @brief   终端文本输出（`cli_text`）的自动化验收
 *
 * 背景：Windows 控制台默认 **CP936**，而工程里的中文是 **UTF-8** 字面量，
 * 不转换就是 `鍙戠幇 0 涓�鑺傜偣`（现场实测）。本套用例锁住三件事：
 *
 *  1. **纯逻辑**（`cli_utf8_complete_prefix`）：绝不把多字节汉字从中间切开 ——
 *     这是转换正确性的地基，而且它可以在没有控制台的机器上测。
 *  2. **转码**（`cli_text_conv`）：CP936/CP1252 的期望字节；UTF-8 直通；
 *     目标代码页**表示不了**的字符变成 `?`（而不是"最佳拟合"成另一个字）。
 *  3. **接线**（注入式转换器 + 强制"控制台"模式）：跨调用被切断的 UTF-8、
 *     逐字节写、以及"非控制台必须原样写 UTF-8"。
 *
 * 不做的部分：真控制台（ConPTY）下的端到端显示 —— 那需要交互式终端，
 * 只能人工看（见 docs/CLI.zh-CN.md 的"自查"小节）。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#  include <io.h>
#  include <fcntl.h>
#else
#  include <unistd.h>
#  include <fcntl.h>
#endif

#include "cli_text.h"

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

#define CHECK_MEM_EQ(a, n, ...)                                              \
    do {                                                                     \
        static const unsigned char _exp[] = { __VA_ARGS__ };                 \
        g_checks++;                                                          \
        if ((n) != sizeof _exp || memcmp((a), _exp, sizeof _exp) != 0) {      \
            size_t _i;                                                       \
            g_fail++;                                                        \
            printf("  FAIL %s:%d: 字节不符\n    实际:", __FILE__, __LINE__); \
            for (_i = 0; _i < (n); ++_i)                                     \
                printf(" %02x", (unsigned char)(a)[_i]);                     \
            printf("\n    期望:");                                           \
            for (_i = 0; _i < sizeof _exp; ++_i) printf(" %02x", _exp[_i]);  \
            printf("\n");                                                    \
        }                                                                    \
    } while (0)

/* --------------------------------------------------------------------------
 * 假转换器：只记录"被调用了什么"，每块只输出一个 '#'，便于数块数
 * ------------------------------------------------------------------------ */

static struct {
    unsigned calls;
    unsigned cp;
    size_t   in_total;
    size_t   last_n;
    size_t   min_n;
    char     seen[1024];
    size_t   seen_n;
} g_fake;

static void fake_reset(void)
{
    memset(&g_fake, 0, sizeof g_fake);
    g_fake.min_n = (size_t)-1;
}

static size_t fake_conv(unsigned cp, const char *utf8, size_t n, char *out, size_t cap)
{
    size_t i;

    if (n + 1u > cap) return 0u;
    g_fake.calls++;
    g_fake.cp = cp;
    g_fake.in_total += n;
    g_fake.last_n = n;
    if (n < g_fake.min_n) g_fake.min_n = n;
    for (i = 0u; i < n && g_fake.seen_n < sizeof g_fake.seen; ++i) {
        g_fake.seen[g_fake.seen_n++] = utf8[i];
    }
    out[0] = '#';
    return 1u;
}

/* --------------------------------------------------------------------------
 * 读回 tmpfile 内容
 * ------------------------------------------------------------------------ */

static size_t slurp(FILE *f, unsigned char *buf, size_t cap)
{
    size_t n;

    (void)fflush(f);
    (void)rewind(f);
    n = fread(buf, 1u, cap, f);
    return n;
}

/* --------------------------------------------------------------------------
 * 1. 纯逻辑：UTF-8 完整码点前缀
 * ------------------------------------------------------------------------ */

static void test_utf8_prefix(void)
{
    const char *cn  = "\xE4\xB8\xAD";              /* "中" */
    const char *cn2 = "\xE5\x8F\x91\xE7\x8E\xB0";  /* "发现" */

    /* ASCII 与完整汉字：整串都是完整码点 */
    CHECK(cli_utf8_complete_prefix("abc", 3u) == 3u);
    CHECK(cli_utf8_complete_prefix(cn, 3u) == 3u);
    CHECK(cli_utf8_complete_prefix(cn2, 6u) == 6u);
    CHECK(cli_utf8_complete_prefix("", 0u) == 0u);

    /* 被切开：只能取回 0 —— 绝不允许"切一半先发出去" */
    CHECK(cli_utf8_complete_prefix(cn, 1u) == 0u);
    CHECK(cli_utf8_complete_prefix(cn, 2u) == 0u);

    /* 前面完整、尾部残缺：只返回完整部分 */
    CHECK(cli_utf8_complete_prefix("ab\xE4\xB8", 4u) == 2u);
    CHECK(cli_utf8_complete_prefix("\xE4\xB8\xAD\xE4", 4u) == 3u);   /* 汉字 + 半个下一个 */
    CHECK(cli_utf8_complete_prefix("ab\xE4\xB8\xADz", 6u) == 6u);

    /* 4 字节序列（emoji 量级）也要支持 */
    CHECK(cli_utf8_complete_prefix("\xF0\x9F\x98\x80", 4u) == 4u);
    CHECK(cli_utf8_complete_prefix("\xF0\x9F\x98", 3u) == 0u);

    /* 非法输入不得死循环，也不得吞掉后面的内容 */
    CHECK(cli_utf8_complete_prefix("\x80", 1u) == 1u);            /* 孤立续字节 */
    CHECK(cli_utf8_complete_prefix("\x80\x80", 2u) == 2u);
    CHECK(cli_utf8_complete_prefix("\xC3z", 2u) == 2u);           /* 2 字节前导 + 非续字节 */
    CHECK(cli_utf8_complete_prefix("\xFF\xE4\xB8\xAD", 4u) == 4u);/* 坏字节后仍走完 */

    /* 3 字节前导但长度不够 → 整串都不能算完整（宁可等） */
    CHECK(cli_utf8_complete_prefix("\xE4z", 2u) == 0u);
}

/* --------------------------------------------------------------------------
 * 2. 转码
 * ------------------------------------------------------------------------ */

static void test_conv(void)
{
    char out[64];
    size_t n;

#if defined(_WIN32)
    /* CP936：本机的控制台代码页。"发现" → B7 A2 CF D6（python -c "…".encode('gbk') 校对过） */
    n = cli_text_conv(936u, "\xE5\x8F\x91\xE7\x8E\xB0", 6u, out, sizeof out);
    CHECK_MEM_EQ(out, n, 0xB7, 0xA2, 0xCF, 0xD6);

    /* CP1252（西欧）：é → E9 */
    n = cli_text_conv(1252u, "\xC3\xA9", 2u, out, sizeof out);
    CHECK_MEM_EQ(out, n, 0xE9);

    /* 目标代码页表示不了的字符 → '?'（不是"最佳拟合"成别的字，那样更难查） */
    n = cli_text_conv(936u, "\xE2\x9A\xA0", 3u, out, sizeof out);   /* U+26A0 ⚠ */
    CHECK_MEM_EQ(out, n, '?');

    /* 混合：可表示的原样、不可表示的变 '?'，长度按目标代码页算 */
    n = cli_text_conv(936u, "A\xE2\x9A\xA0Z", 5u, out, sizeof out);
    CHECK_MEM_EQ(out, n, 'A', '?', 'Z');

    /* 非法 UTF-8 → 返回 0，调用方降级为原样写（绝不丢内容） */
    CHECK(cli_text_conv(936u, "\xE4\xB8", 2u, out, sizeof out) == 0u);
    CHECK(cli_text_conv(936u, "\xFF\xFE", 2u, out, sizeof out) == 0u);

    /* 缓冲不足 → 0 */
    CHECK(cli_text_conv(936u, "\xE5\x8F\x91\xE7\x8E\xB0", 6u, out, 1u) == 0u);
#endif

    /* UTF-8 直通（cp = 0 或 65001）：两种平台行为一致 */
    n = cli_text_conv(0u, "\xE4\xB8\xAD", 3u, out, sizeof out);
    CHECK_MEM_EQ(out, n, 0xE4, 0xB8, 0xAD);
    n = cli_text_conv(65001u, "\xE4\xB8\xAD", 3u, out, sizeof out);
    CHECK_MEM_EQ(out, n, 0xE4, 0xB8, 0xAD);

    /* 空输入 */
    CHECK(cli_text_conv(936u, "", 0u, out, sizeof out) == 0u);

    /* 超过单块上限：要求调用方先切块（返回 0 而不是静默截断） */
    {
        char big[CLI_TEXT_CHUNK + 2u];
        memset(big, 'a', sizeof big);
        CHECK(cli_text_conv(936u, big, sizeof big, out, sizeof out) == 0u);
    }
}

/* --------------------------------------------------------------------------
 * 3. 接线（注入式转换器）
 * ------------------------------------------------------------------------ */

static void test_wiring(void)
{
    unsigned char buf[256];
    size_t        n;
    FILE         *f;

    /* --- 控制台模式：走转换器，块数 = 1 --- */
    fake_reset();
    cli_text_set_conv(fake_conv);
    cli_text_set_console_force(1, 999u);
    f = tmpfile();
    CHECK(f != NULL);
    if (!f) return;

    (void)cli_fputs("\xE4\xB8\xAD", f);                 /* "中" */
    n = slurp(f, buf, sizeof buf);
    CHECK_MEM_EQ(buf, n, '#');
    CHECK(g_fake.calls == 1u);
    CHECK(g_fake.cp == 999u);                            /* 代码页原样透传 */
    CHECK(g_fake.seen_n == 3u && memcmp(g_fake.seen, "\xE4\xB8\xAD", 3u) == 0);
    (void)fclose(f);

    /* --- 关键回归：跨调用被切开的汉字，必须等齐了再转 --- */
    fake_reset();
    f = tmpfile();
    if (f) {
        (void)cli_fputs("\xE4\xB8", f);                  /* 半个"中" */
        n = slurp(f, buf, sizeof buf);
        CHECK(n == 0u);                                  /* 一个字节都不许先发 */
        CHECK(g_fake.calls == 0u);

        (void)cli_fputs("\xAD", f);                      /* 补齐 */
        n = slurp(f, buf, sizeof buf);
        CHECK_MEM_EQ(buf, n, '#');
        CHECK(g_fake.calls == 1u);
        CHECK(g_fake.in_total == 3u);
        CHECK(g_fake.seen_n == 3u && memcmp(g_fake.seen, "\xE4\xB8\xAD", 3u) == 0);
        (void)fclose(f);
    }

    /* --- 逐字节写（fputc）同样安全 --- */
    fake_reset();
    f = tmpfile();
    if (f) {
        (void)cli_fputc(0xE4, f);
        (void)cli_fputc(0xB8, f);
        CHECK(slurp(f, buf, sizeof buf) == 0u);
        (void)cli_fputc(0xAD, f);
        n = slurp(f, buf, sizeof buf);
        CHECK_MEM_EQ(buf, n, '#');
        CHECK(g_fake.calls == 1u);
        (void)fclose(f);
    }

    /* --- 一次写多块：整行一次性交给转换器（不足 CLI_TEXT_CHUNK 不拆） --- */
    fake_reset();
    f = tmpfile();
    if (f) {
        (void)cli_fprintf(f, "%s %u %s", "\xE5\x8F\x91\xE7\x8E\xB0",
                          (unsigned)0, "\xE4\xB8\xAD\xE6\x96\x87");
        n = slurp(f, buf, sizeof buf);
        CHECK_MEM_EQ(buf, n, '#');
        CHECK(g_fake.calls == 1u);
        CHECK(g_fake.in_total == 15u);                    /* 6 + 1 + 1 + 1 + 6 字节 */
        (void)fclose(f);
    }

    /* --- 超过单块上限：必须拆块，而且**一个字节都不能丢/重复** --- */
    fake_reset();
    f = tmpfile();
    if (f) {
        char big[601];                                    /* 200 个"中" = 600 B + NUL */
        size_t i;
        for (i = 0u; i + 3u <= 600u; i += 3u) memcpy(big + i, "\xE4\xB8\xAD", 3u);
        big[600] = '\0';
        (void)cli_fputs(big, f);
        /*
         * 具体拆成几块属于实现细节（当前是 510 + 6 + 84），不锁死；
         * 要锁的是**不变式**：总字节一致、顺序一致、**每块都落在码点边界上**
         * （每块长度都是 3 的倍数 —— 一旦把汉字切开，就会出现 511/512 这种长度）。
         */
        CHECK(g_fake.calls >= 2u);
        CHECK(g_fake.in_total == 600u);
        CHECK(g_fake.min_n % 3u == 0u);
        CHECK(g_fake.last_n % 3u == 0u);
        CHECK(g_fake.seen_n == 600u);
        CHECK(memcmp(g_fake.seen, big, 600u) == 0);        /* 顺序、内容完全一致 */
        (void)fclose(f);
    }

    /* --- 非控制台：必须原样写 UTF-8，且**不调**转换器 --- */
    fake_reset();
    cli_text_set_console_force(0, 0u);
    f = tmpfile();
    if (f) {
        (void)cli_fputs("\xE4\xB8\xAD", f);
        n = slurp(f, buf, sizeof buf);
        CHECK_MEM_EQ(buf, n, 0xE4, 0xB8, 0xAD);
        CHECK(g_fake.calls == 0u);
        (void)fclose(f);
    }

    /* --- 行结束符不由本模块改动（'\n' 原样通过，交给 CRT/平台） --- */
    f = tmpfile();
    if (f) {
        (void)cli_fputs("a\nb", f);
        n = slurp(f, buf, sizeof buf);
        CHECK_MEM_EQ(buf, n, 'a', '\n', 'b');
        (void)fclose(f);
    }

    /* --- 转换器"干不了"时降级为原样写：内容绝不丢 --- */
    fake_reset();
    cli_text_set_console_force(1, 999u);
    cli_text_set_conv(NULL);                              /* 真转换器 + cp=999 会失败 */
    f = tmpfile();
    if (f) {
        (void)cli_fputs("\xE4\xB8\xAD", f);
        n = slurp(f, buf, sizeof buf);
        CHECK_MEM_EQ(buf, n, 0xE4, 0xB8, 0xAD);
        (void)fclose(f);
    }

    cli_text_set_console_force(0, 0u);
    cli_text_set_conv(NULL);
}

/* --------------------------------------------------------------------------
 * 4. cli_printf / cli_puts 写的是真 stdout（把 fd 1 临时换掉再验）
 * ------------------------------------------------------------------------ */

#if defined(_WIN32)
#  define T_DUP(fd)     _dup(fd)
#  define T_DUP2(a, b)  _dup2((a), (b))
#  define T_CLOSE(fd)   _close(fd)
#  define T_OPENW(p)    _open((p), _O_CREAT | _O_TRUNC | _O_WRONLY | _O_BINARY, 0600)
#else
#  define T_DUP(fd)     dup(fd)
#  define T_DUP2(a, b)  dup2((a), (b))
#  define T_CLOSE(fd)   close(fd)
#  define T_OPENW(p)    open((p), O_CREAT | O_TRUNC | O_WRONLY, 0600)
#endif

#define T_TMPNAME "cli_text_stdout_test.tmp"

static void test_stdout_wrappers(void)
{
    unsigned char buf[256];
    size_t        n;
    int           saved;
    int           fd;

    fake_reset();
    cli_text_set_conv(fake_conv);
    cli_text_set_console_force(1, 936u);

    (void)fflush(stdout);
    saved = T_DUP(1);
    fd    = T_OPENW(T_TMPNAME);
    CHECK(saved >= 0);
    CHECK(fd >= 0);
    if (saved < 0 || fd < 0) {
        cli_text_set_console_force(0, 0u);
        cli_text_set_conv(NULL);
        return;
    }
    (void)T_DUP2(fd, 1);
    (void)T_CLOSE(fd);

    (void)cli_printf("%s", "\xE4\xB8\xAD");               /* "中" */
    (void)cli_puts("\xE4\xB8\xAD");
    (void)fflush(stdout);

    (void)T_DUP2(saved, 1);
    (void)T_CLOSE(saved);

    {
        FILE *f = fopen(T_TMPNAME, "rb");
        CHECK(f != NULL);
        n = f ? fread(buf, 1u, sizeof buf, f) : 0u;
        if (f) (void)fclose(f);
        (void)remove(T_TMPNAME);
    }
    /* 两次写调用 → 两个非 ASCII 块；`cli_puts` 补的 '\n' 是 ASCII，
       走快路径原样写（所以字节里有第三个 0x0a，但不进转换器）。 */
    CHECK_MEM_EQ(buf, n, '#', '#', '\n');
    CHECK(g_fake.calls == 2u);
    CHECK(g_fake.cp == 936u);

    cli_text_set_console_force(0, 0u);
    cli_text_set_conv(NULL);
}

/* ------------------------------------------------------------------------ */

int main(void)
{
    printf("=== cli_text：终端文本输出 ===\n");

    test_utf8_prefix();
    test_conv();
    test_wiring();
    test_stdout_wrappers();

    printf("=== %u checks, %u failures ===\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
