/**
 * @file    test_hal.c
 * @brief   WP1（其余后端）：内置 HAL 的**可离线测试**部分
 *
 * 这里测三件事，都不需要真实 CAN 硬件：
 *   1. slcan 的 ASCII 编解码（纯逻辑，最容易出错的部分）；
 *   2. 句柄生命周期：`jsdk_hal_close()` 必须分派到各后端自己的 destroy
 *      （早先的版本对真实后端直接 `free()`，会漏 fd）；
 *   3. 未编译/不存在的后端返回**明确错误**而不是静默成功。
 *
 * 真机收发（socketcan / pcan / slcan 实机）无法在 CI 断言，见
 * `docs/PORTING.zh-CN.md` 的手工冒烟清单。
 */

#include <stdio.h>
#include <string.h>

#include "joint_sdk.h"
#include "jsdk_hal_builtin.h"

#include "hal_handle.h"          /* 内部：句柄类别 */
#include "hal_slcan_codec.h"

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

/* ==========================================================================
 * 1. slcan 编码
 * ======================================================================== */

/** 把一帧编码出来后与期望文本比较。 */
static void expect_line(const jsdk_can_frame_t *f, const char *want,
                        const char *what)
{
    char   buf[JSDK_SLCAN_LINE_MAX];
    size_t n = jsdk_slcan_encode(f, buf, sizeof buf);

    g_checks++;
    if (n != strlen(want) || memcmp(buf, want, n) != 0) {
        g_fail++;
        printf("  FAIL %s: got \"%.*s\" (%u B), want \"%s\"\n",
               what, (int)n, buf, (unsigned)n, want);
    } else {
        printf("      %-28s -> \"%s\"\n", what, want);
    }
}

static void test_encode(void)
{
    jsdk_can_frame_t f;
    unsigned i;

    printf("[1] slcan encode\n");

    /* 扩展帧（本协议恒用） */
    memset(&f, 0, sizeof f);
    f.id = 0x0A123456u & 0x1FFFFFFFu;
    f.flags = JSDK_FRAME_EXT;
    f.len = 8u;
    f.data[0] = 0x01; f.data[1] = 0x02; f.data[2] = 0x03; f.data[3] = 0x04;
    f.data[4] = 0xAA; f.data[5] = 0xBB; f.data[6] = 0xCC; f.data[7] = 0xDD;
    expect_line(&f, "T0A123456801020304AABBCCDD\r", "ext 8B");

    /* 小 ID：必须**补零到 8 位**（不是去掉前导零） */
    memset(&f, 0, sizeof f);
    f.id = 0x1E000001u;
    f.flags = JSDK_FRAME_EXT;
    f.len = 0u;
    expect_line(&f, "T1E0000010\r", "ext 0B");

    /* 标准帧 3 位 */
    memset(&f, 0, sizeof f);
    f.id = 0x123u;
    f.len = 2u;
    f.data[0] = 0xDE; f.data[1] = 0xAD;
    expect_line(&f, "t1232DEAD\r", "std 2B");

    /* 大小写：十六进制字母统一大写 */
    memset(&f, 0, sizeof f);
    f.id = 0x1FFFFFFFu;
    f.flags = JSDK_FRAME_EXT;
    f.len = 1u;
    f.data[0] = 0xFF;
    expect_line(&f, "T1FFFFFFF1FF\r", "ext max id");

    /* ------------------------------------------------------------------
     * CAN FD（CANable 2.0 的 b/B/d/D 前缀）
     *
     * ⚠ 字母映射很容易反：**b/B 是“带 BRS”**，d/D 是“不带 BRS”。
     *   本用例把四种组合与 DLC 长度码全钉死。
     * ---------------------------------------------------------------- */

    /* 标准帧 FD、BRS=0 → 'd'；4 字节以内 DLC 码 == 字节数 */
    memset(&f, 0, sizeof f);
    f.id = 0x123u;
    f.flags = JSDK_FRAME_FD;
    f.len = 2u;
    f.data[0] = 0xDE; f.data[1] = 0xAD;
    expect_line(&f, "d1232DEAD\r", "fd std brs=0 2B");

    /* 扩展帧 FD、BRS=0 → 'D' */
    memset(&f, 0, sizeof f);
    f.id = 0x1E000001u;
    f.flags = (uint8_t)(JSDK_FRAME_EXT | JSDK_FRAME_FD);
    f.len = 0u;
    expect_line(&f, "D1E0000010\r", "fd ext brs=0 0B");

    /* 标准帧 FD、BRS=1 → 'b' */
    memset(&f, 0, sizeof f);
    f.id = 0x123u;
    f.flags = (uint8_t)(JSDK_FRAME_FD | JSDK_FRAME_BRS);
    f.len = 1u;
    f.data[0] = 0x5Au;
    expect_line(&f, "b12315A\r", "fd std brs=1 1B");

    /* 扩展帧 FD、BRS=1 → 'B'；**12 字节的 DLC 码是 '9'（不是 'C'）** */
    memset(&f, 0, sizeof f);
    f.id = 0x0A123456u & 0x1FFFFFFFu;
    f.flags = (uint8_t)(JSDK_FRAME_EXT | JSDK_FRAME_FD | JSDK_FRAME_BRS);
    f.len = 12u;
    for (i = 0u; i < 12u; ++i) f.data[i] = (uint8_t)i;
    expect_line(&f, "B0A1234569000102030405060708090A0B\r", "fd ext brs=1 12B");

    /* 64 字节（FD 上限）→ DLC 码 'F'，整行 139 字符 —— 验证 LINE_MAX 够用 */
    memset(&f, 0, sizeof f);
    f.id = 0x1FFFFFFFu;
    f.flags = (uint8_t)(JSDK_FRAME_EXT | JSDK_FRAME_FD | JSDK_FRAME_BRS);
    f.len = 64u;
    for (i = 0u; i < 64u; ++i) f.data[i] = 0xA5u;
    {
        char big[JSDK_SLCAN_LINE_MAX];
        size_t n64 = jsdk_slcan_encode(&f, big, sizeof big);
        CHECK_EQ(n64, 1u + 8u + 1u + 128u + 1u);        /* 139 */
        CHECK_EQ(big[0], 'B');
        CHECK_EQ(big[9], 'F');
        CHECK_EQ(n64 <= JSDK_SLCAN_LINE_MAX, 1);
    }

    /* ❌ 9/10/11 字节在 FD 里**无法表示**（长度码没有对应项）→ 拒绝，而不是
       静默按 12 字节发（那会让设备收到一条长度不对的帧） */
    memset(&f, 0, sizeof f);
    f.id = 1u; f.flags = (uint8_t)(JSDK_FRAME_EXT | JSDK_FRAME_FD);
    f.len = 9u;
    CHECK_EQ(jsdk_slcan_encode(&f, (char[256]){0}, 256u), 0u);
    f.len = 11u;
    CHECK_EQ(jsdk_slcan_encode(&f, (char[256]){0}, 256u), 0u);

    /* ❌ Classic 帧 >8 字节 → 拒绝（不静默当成 FD 发出去）*/
    memset(&f, 0, sizeof f);
    f.id = 1u; f.flags = JSDK_FRAME_EXT; f.len = 12u;
    CHECK_EQ(jsdk_slcan_encode(&f, (char[256]){0}, 256u), 0u);

    /* DLC 长度码映射表（与 python-can 的 CAN_FD_DLC 一致）*/
    CHECK_EQ(jsdk_slcan_fd_len_from_dlc(0u), 0);
    CHECK_EQ(jsdk_slcan_fd_len_from_dlc(8u), 8);
    CHECK_EQ(jsdk_slcan_fd_len_from_dlc(9u), 12);
    CHECK_EQ(jsdk_slcan_fd_len_from_dlc(10u), 16);
    CHECK_EQ(jsdk_slcan_fd_len_from_dlc(11u), 20);
    CHECK_EQ(jsdk_slcan_fd_len_from_dlc(12u), 24);
    CHECK_EQ(jsdk_slcan_fd_len_from_dlc(13u), 32);
    CHECK_EQ(jsdk_slcan_fd_len_from_dlc(14u), 48);
    CHECK_EQ(jsdk_slcan_fd_len_from_dlc(15u), 64);
    CHECK_EQ(jsdk_slcan_fd_len_from_dlc(16u), -1);
    CHECK_EQ(jsdk_slcan_fd_dlc_from_len(12u), 9);
    CHECK_EQ(jsdk_slcan_fd_dlc_from_len(64u), 15);
    CHECK_EQ(jsdk_slcan_fd_dlc_from_len(9u), -1);
    CHECK_EQ(jsdk_slcan_fd_dlc_from_len(65u), -1);

    /* 缓冲不足：整帧拒绝，**不**输出半截 */
    memset(&f, 0, sizeof f);
    f.id = 0x1u; f.flags = JSDK_FRAME_EXT; f.len = 8u;
    CHECK_EQ(jsdk_slcan_encode(&f, (char[16]){0}, 16u), 0u);
    printf("      FD (d/D/b/B) encoded, unrepresentable lengths rejected, "
           "short buffer rejected\n");
}

/* ==========================================================================
 * 2. slcan 解码
 * ======================================================================== */

static void test_decode(void)
{
    printf("[2] slcan decode\n");
    {
        const char *line = "T0A123456801020304AABBCCDD\r";
        jsdk_can_frame_t f;
        size_t consumed = 0u;
        jsdk_slcan_rc_t rc = jsdk_slcan_decode(line, strlen(line), &f, &consumed);

        CHECK_EQ(rc, JSDK_SLCAN_OK);
        CHECK_EQ(consumed, strlen(line));
        CHECK_EQ(f.id, 0x0A123456u & 0x1FFFFFFFu);
        CHECK(f.flags & JSDK_FRAME_EXT);
        CHECK(!(f.flags & JSDK_FRAME_FD));
        CHECK_EQ(f.len, 8u);
        CHECK_EQ(f.data[0], 0x01u);
        CHECK_EQ(f.data[3], 0x04u);
        CHECK_EQ(f.data[7], 0xDDu);
    }

    /* ⚠ DLC 省略 = **0 字节**（Lawicel 规范）。本文件早期版本当成 8 字节，
       被用例当场拓出来：那样 `t123` 会被判成 MALFORMED。 */
    {
        const char *line = "t123\r";
        jsdk_can_frame_t f;
        jsdk_slcan_rc_t rc = jsdk_slcan_decode(line, strlen(line), &f, NULL);

        CHECK_EQ(rc, JSDK_SLCAN_OK);
        CHECK_EQ(f.len, 0u);
    }

    /* DLC 写 8 且跟上 16 个十六进制字符 → 8 字节零载荷 */
    {
        const char *line = "t12380000000000000000\r";
        jsdk_can_frame_t f;

        CHECK_EQ(jsdk_slcan_decode(line, strlen(line), &f, NULL), JSDK_SLCAN_OK);
        CHECK_EQ(f.len, 8u);
        CHECK_EQ(f.data[7], 0u);
    }

    /* DLC 写 8 但不给数据 → 长度不符，报错而不是补零 */
    {
        jsdk_can_frame_t f;
        CHECK_EQ(jsdk_slcan_decode("t1238\r", 6u, &f, NULL), JSDK_SLCAN_MALFORMED);
    }

    /* ACK / NACK */
    {
        jsdk_can_frame_t f;
        CHECK_EQ(jsdk_slcan_decode("\r", 1u, &f, NULL), JSDK_SLCAN_ACK);
        CHECK_EQ(jsdk_slcan_decode("\a\r", 2u, &f, NULL), JSDK_SLCAN_NACK);
    }

    /* 半行 → NEED_MORE，且**不能**消费掉（否则下批数据会错位） */
    {
        jsdk_can_frame_t f;
        size_t consumed = 0u;
        const char *half = "T0A12345680102";

        CHECK_EQ(jsdk_slcan_decode(half, strlen(half), &f, &consumed),
                 JSDK_SLCAN_NEED_MORE);
        CHECK_EQ(consumed, strlen(half));
    }

    /* 一行里带"半行 + 完整行"：必须只消费前一行 */
    {
        const char *two = "t1231AA\rT000000010\r";
        jsdk_can_frame_t f;
        size_t consumed = 0u;

        CHECK_EQ(jsdk_slcan_decode(two, strlen(two), &f, &consumed), JSDK_SLCAN_OK);
        CHECK_EQ(consumed, 8u);            /* "t1231AA\r" 共 8 字节 */
        CHECK_EQ(f.len, 1u);
        CHECK_EQ(f.data[0], 0xAAu);
    }

    /* 坏行（状态行 / 非法字符 / 长度不符）→ MALFORMED，且仍要消费整行 */
    {
        jsdk_can_frame_t f;
        size_t consumed = 0u;
        const char *bad = "V1013\r";

        CHECK_EQ(jsdk_slcan_decode(bad, strlen(bad), &f, &consumed),
                 JSDK_SLCAN_MALFORMED);
        CHECK_EQ(consumed, 6u);            /* 整行丢掉，不切错位 */

        consumed = 0u;
        CHECK_EQ(jsdk_slcan_decode("t123Z\r", 6u, &f, &consumed),
                 JSDK_SLCAN_MALFORMED);

        consumed = 0u;
        /* DLC 声明 2 字节但只给了 1 字节 */
        CHECK_EQ(jsdk_slcan_decode("t1232AA\r", 8u, &f, &consumed),
                 JSDK_SLCAN_MALFORMED);

        consumed = 0u;
        /* DLC 声明 2 字节、载荷给了 3 字节 → 长度不符 */
        CHECK_EQ(jsdk_slcan_decode("t1232AABBC\r", 11u, &f, &consumed),
                 JSDK_SLCAN_MALFORMED);

        consumed = 0u;
        /* DLC > 8 */
        CHECK_EQ(jsdk_slcan_decode("t1239AABB\r", 10u, &f, &consumed),
                 JSDK_SLCAN_MALFORMED);
    }

    /* RTR：合法但本协议不用；必须能识别出来（否则会被当数据帧） */
    {
        jsdk_can_frame_t f;
        CHECK_EQ(jsdk_slcan_decode("r1234\r", 6u, &f, NULL), JSDK_SLCAN_RTR);
        CHECK_EQ(jsdk_slcan_decode("R000000014\r", 11u, &f, NULL), JSDK_SLCAN_RTR);
    }

    /* ------------------------------------------------------------------
     * CAN FD 解码（四个前缀 + DLC 长度码）
     * ---------------------------------------------------------------- */
    {
        jsdk_can_frame_t f;

        /* 'd' = 标准 FD、BRS=0 */
        CHECK_EQ(jsdk_slcan_decode("d1232DEAD\r", 10u, &f, NULL), JSDK_SLCAN_OK);
        CHECK_EQ(f.id, 0x123u);
        CHECK_EQ(f.len, 2u);
        CHECK_EQ(f.flags, JSDK_FRAME_FD);                 /* 无 EXT、无 BRS */
        CHECK_EQ(f.data[1], 0xADu);

        /* 'D' = 扩展 FD、BRS=0 */
        CHECK_EQ(jsdk_slcan_decode("D0A123456801020304AABBCCDD\r", 27u, &f, NULL),
                 JSDK_SLCAN_OK);
        CHECK_EQ(f.id, 0x0A123456u & 0x1FFFFFFFu);
        CHECK_EQ(f.len, 8u);
        CHECK_EQ(f.flags, (uint8_t)(JSDK_FRAME_EXT | JSDK_FRAME_FD));

        /* 'b' = 标准 FD、**BRS=1**（最容易搞反的一条）*/
        CHECK_EQ(jsdk_slcan_decode("b12315A\r", 8u, &f, NULL), JSDK_SLCAN_OK);
        CHECK_EQ(f.flags, (uint8_t)(JSDK_FRAME_FD | JSDK_FRAME_BRS));
        CHECK_EQ(f.data[0], 0x5Au);

        /* 'B' = 扩展 FD、BRS=1；DLC 码 F = 64 字节 */
        {
            char big[JSDK_SLCAN_LINE_MAX + 4];
            size_t n = 0u;
            unsigned k;

            big[n++] = 'B';
            memcpy(big + n, "1FFFFFFF", 8u); n += 8u;
            big[n++] = 'F';
            for (k = 0u; k < 64u; ++k) { big[n++] = 'A'; big[n++] = '5'; }
            big[n++] = '\r';

            CHECK_EQ(jsdk_slcan_decode(big, n, &f, NULL), JSDK_SLCAN_OK);
            CHECK_EQ(f.len, 64u);
            CHECK_EQ(f.flags,
                     (uint8_t)(JSDK_FRAME_EXT | JSDK_FRAME_FD | JSDK_FRAME_BRS));
            CHECK_EQ(f.data[63], 0xA5u);
        }

        /* DLC 码 9 = 12 字节（不是 9 字节）*/
        CHECK_EQ(jsdk_slcan_decode(
                     "d1239000102030405060708090A0B\r", 30u, &f, NULL),
                 JSDK_SLCAN_OK);
        CHECK_EQ(f.len, 12u);

        /* ❌ FD 帧载荷写短了（DLC 说 12 字节只给 10 字节）→ MALFORMED，
           不能"解出一条 10 字节的帧"：截断的描述符帧比语法错误难查得多 */
        CHECK_EQ(jsdk_slcan_decode(
                     "d123900010203040506070809\r", 26u, &f, NULL),
                 JSDK_SLCAN_MALFORMED);

        /* ❌ 载荷写多了也不行 */
        CHECK_EQ(jsdk_slcan_decode(
                     "d1239000102030405060708090A0B0C\r", 32u, &f, NULL),
                 JSDK_SLCAN_MALFORMED);

        /* ❌ Classic 的 DLC 不允许 >8（那个位置在 FD 才是长度码）*/
        CHECK_EQ(jsdk_slcan_decode("t1239AABB\r", 10u, &f, NULL),
                 JSDK_SLCAN_MALFORMED);

        /* ❌ FD 的 DLC 码也必须是一位十六进制 */
        CHECK_EQ(jsdk_slcan_decode("d123Z\r", 6u, &f, NULL),
                 JSDK_SLCAN_MALFORMED);

        printf("      FD decode: d/D/b/B + FD DLC code map + strict payload length\n");
    }

    printf("      round-trip + malformed/RTR/ACK handling verified\n");
}

/* 编解码往返：一组随机-ish 的帧（Classic + FD），encode → decode 必须一致 */
static void test_roundtrip(void)
{
    unsigned i;

    printf("[3] slcan round-trip\n");
    for (i = 0u; i < 16u; ++i) {
        jsdk_can_frame_t a, b;
        char   line[JSDK_SLCAN_LINE_MAX];
        size_t n;
        unsigned k;

        memset(&a, 0, sizeof a);
        a.id    = 0x10000000u + i * 0x12345u + 7u;
        a.flags = JSDK_FRAME_EXT;
        a.len   = (uint8_t)(i % 9u);
        for (k = 0u; k < a.len; ++k) a.data[k] = (uint8_t)(i * 31u + k * 7u);

        n = jsdk_slcan_encode(&a, line, sizeof line);
        CHECK(n > 0u);

        memset(&b, 0, sizeof b);
        CHECK_EQ(jsdk_slcan_decode(line, n, &b, NULL), JSDK_SLCAN_OK);
        CHECK_EQ(b.id, a.id);
        CHECK_EQ(b.len, a.len);
        CHECK_EQ(b.flags, a.flags);
        CHECK(memcmp(a.data, b.data, a.len) == 0);
    }

    /* FD：16 种长度码全覆盖（含 9→12 … F→64 的跳变），标准/扩展 × BRS 两态 */
    {
        static const uint8_t lens[16] = {
            0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u,
            8u, 12u, 16u, 20u, 24u, 32u, 48u, 64u
        };
        unsigned variant;

        for (variant = 0u; variant < 4u; ++variant) {
            unsigned ext = variant & 1u;
            unsigned brs = (variant >> 1) & 1u;
            unsigned li;

            for (li = 0u; li < 16u; ++li) {
                jsdk_can_frame_t a, b;
                char   line[JSDK_SLCAN_LINE_MAX];
                size_t n;
                unsigned k;

                memset(&a, 0, sizeof a);
                a.id    = ext ? (0x1ABCDEFu) : 0x7ABu;
                a.flags = (uint8_t)(JSDK_FRAME_FD
                                    | (ext ? JSDK_FRAME_EXT : 0u)
                                    | (brs ? JSDK_FRAME_BRS : 0u));
                a.len   = lens[li];
                for (k = 0u; k < a.len; ++k) {
                    a.data[k] = (uint8_t)(0x80u + k);
                }

                n = jsdk_slcan_encode(&a, line, sizeof line);
                CHECK(n > 0u);
                CHECK(n <= JSDK_SLCAN_LINE_MAX);

                memset(&b, 0, sizeof b);
                CHECK_EQ(jsdk_slcan_decode(line, n, &b, NULL), JSDK_SLCAN_OK);
                CHECK_EQ(b.id, a.id);
                CHECK_EQ(b.len, a.len);
                CHECK_EQ(b.flags, a.flags);          /* FD/BRS/EXT 一个不多一个不少 */
                CHECK(memcmp(a.data, b.data, a.len) == 0);
            }
        }
        printf("      16 classic + 64 FD frames (4 flag variants x 16 DLC codes) "
               "byte-identical\n");
    }
}

/* ==========================================================================
 * 4. 句柄生命周期与后端错误码
 * ======================================================================== */

static void test_handles(void)
{
    printf("[4] handle lifecycle / backend error codes\n");

    /* --- 虚拟后端：类别正确，关闭幂等 --- */
    {
        jsdk_can_hal_t hal;
        jsdk_hal_handle_t *h = NULL;

        memset(&hal, 0, sizeof hal);
        CHECK_EQ(jsdk_hal_virtual_open(&hal, &h, "0:id=1,fd"), JSDK_OK);
        CHECK(h != NULL);
        CHECK_EQ(jsdk_hal_kind_of(h), JSDK_HAL_KIND_VIRTUAL);
        CHECK(strcmp(jsdk_hal_kind_name(jsdk_hal_kind_of(h)), "virtual") == 0);
        CHECK_EQ(jsdk_hal_close(h), JSDK_OK);
        /* ⚠ 幂等：允许重复 close 是 API 承诺 */
        CHECK_EQ(jsdk_hal_close(NULL), JSDK_OK);
    }

    /* --- 打开失败时 *out 必须是 NULL（否则调用方 close 一个垃圾指针） --- */
    {
        jsdk_can_hal_t hal;
        jsdk_hal_handle_t *h = (jsdk_hal_handle_t *)(uintptr_t)0x1u;   /* 脏值 */
        jsdk_status_t st;

        memset(&hal, 0, sizeof hal);
        st = jsdk_hal_virtual_open(&hal, &h, "0:id=1;1:id=2;2:id=3;3:id=4;4:id=5");
        CHECK_EQ(st, JSDK_ERR_INVALID_ARG);      /* 超 4 节点 */
        CHECK(h == NULL);
    }

    /* --- 各后端：未编译 → UNSUPPORTED；已编译 → 参数校验生效 --- */
    {
        jsdk_can_hal_t hal;
        jsdk_hal_handle_t *h = NULL;
        jsdk_status_t st;

        memset(&hal, 0, sizeof hal);

        /* 参数非法必须在**任何**后端上先被挡住 */
        CHECK_EQ(jsdk_hal_socketcan_open(&hal, &h, NULL, 1000000u, 5000000u),
                 JSDK_ERR_INVALID_ARG);
        CHECK_EQ(jsdk_hal_slcan_open(&hal, &h, NULL, 115200u, 0u),
                 JSDK_ERR_INVALID_ARG);
        CHECK_EQ(jsdk_hal_pcan_open(&hal, &h, NULL, 1000000u, 0u), JSDK_ERR_INVALID_ARG);
        CHECK(h == NULL);

        /* 数据段波特率低于仲裁段 → 参数错，不是"链路问题" */
        st = jsdk_hal_socketcan_open(&hal, &h, "can0", 1000000u, 500000u);
        CHECK(st == JSDK_ERR_INVALID_ARG || st == JSDK_ERR_UNSUPPORTED);
        CHECK(h == NULL);

        /* 真实通道：本机不一定有硬件/驱动，但**绝不能**返回 JSDK_OK 却给个空句柄 */
        st = jsdk_hal_socketcan_open(&hal, &h, "jsdk_no_such_can0", 1000000u, 0u);
        CHECK(st != JSDK_OK);
        CHECK(h == NULL);

        st = jsdk_hal_slcan_open(&hal, &h, "JSDK_NO_SUCH_PORT", 115200u, 0u);
        CHECK(st != JSDK_OK);
        CHECK(h == NULL);

        /*
         * 能力自报必须与"后端是否真的编进来"一致：
         *   编进来了 → 参数校验生效（端口不存在 → INVALID_ARG）
         *   没编进来 → 占位函数直接 UNSUPPORTED
         * 用行为反推而不是靠编译宏：`JSDK_HAL_HAVE_SLCAN` 是库目标的 PRIVATE
         * 定义，测试看不到它，而"两种配置下都得对"正是该验的东西。
         */
        if (st == JSDK_ERR_UNSUPPORTED) {
            CHECK_EQ(jsdk_hal_slcan_supports_fd(), 0);
        } else {
            CHECK_EQ(jsdk_hal_slcan_supports_fd(), 1);
        }

        /* slcan 的 FD 数据段速率只收录 CANable 2.0 公认的两项；表外速率必须在
           **打开串口之前**就被拒（否则"速率不支持"与"串口打不开"会互相掩盖）。
           这条能在没有硬件的机器上断言，正是因为校验顺序是对的。 */
        CHECK_EQ(jsdk_hal_slcan_open(&hal, &h, "JSDK_NO_SUCH_PORT", 115200u,
                                     3000000u),
                 JSDK_ERR_UNSUPPORTED);
        CHECK_EQ(jsdk_hal_slcan_open(&hal, &h, "COM_ALSO_MISSING", 115200u,
                                     1000000u),
                 JSDK_ERR_UNSUPPORTED);
        CHECK(h == NULL);

        st = jsdk_hal_pcan_open(&hal, &h, "PCAN_USBBUS1", 1000000u, 5000000u);
        CHECK(st != JSDK_OK);           /* 本机没有 PCAN 驱动/硬件 */
        CHECK(h == NULL);
    }
    /* --- 后端能力自报：slcan **支持** FD（CANable 2.0 的 b/B/d/D）。
           具体断言在上面“能力自报与后端可用性一致”那块里 —— 那里能同时
           覆盖“后端没编进来”的配置（那时占位函数返回 0）。 --- */
    /* --- 不认识的通道名：PCAN 必须拒绝而不是"猜一个" ---
       ⚠ 期望值**与平台有关**：PCAN 后端在 Windows/macOS 上才编译，
         在 Linux 上是 `hal_common.c` 的占位实现 —— 占位只能校验"非空"，
         无法知道哪些通道名合法，于是返回的是 `UNSUPPORTED` 而不是
         `INVALID_ARG`。用**空名字**（两边都必须先拒绝）来断言语义，
         而不是把"是否编译了该后端"写死在期望值里。
         （Linux 首次构建时这条就是这么红的。） */
    {
        jsdk_can_hal_t hal;
        jsdk_hal_handle_t *h = NULL;

        memset(&hal, 0, sizeof hal);

        /* 空/NULL 名字：**任何**配置下都是调用方参数错 */
        CHECK_EQ(jsdk_hal_pcan_open(&hal, &h, "", 1000000u, 0u),
                 JSDK_ERR_INVALID_ARG);
        CHECK_EQ(jsdk_hal_pcan_open(&hal, &h, NULL, 1000000u, 0u),
                 JSDK_ERR_INVALID_ARG);

        /* 名字非空但不认识：要么参数错（真后端），要么后端没编进来 */
        {
            jsdk_status_t st = jsdk_hal_pcan_open(&hal, &h, "PCAN_NOPE",
                                                 1000000u, 0u);
            CHECK(st == JSDK_ERR_INVALID_ARG || st == JSDK_ERR_UNSUPPORTED);
        }
        CHECK(h == NULL);
    }

    printf("      kind/close/error-code contract verified\n");
}

/* ==========================================================================
 * main
 * ======================================================================== */

int main(void)
{
    printf("=== WP1 tests (built-in HAL backends) ===\n\n");

    test_encode();
    printf("\n");
    test_decode();
    printf("\n");
    test_roundtrip();
    printf("\n");
    test_handles();

    printf("\n=== %u checks, %u failures ===\n", g_checks, g_fail);
    return (g_fail == 0u) ? 0 : 1;
}
