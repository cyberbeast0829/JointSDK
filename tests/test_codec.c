/**
 * @file    test_codec.c
 * @brief   WP2 帧层 / MIT 编解码回归测试
 *
 * 黄金向量：tests/data/golden_vectors.h
 *   - CAN ID 部分由 cyberbeast_tool.py（独立实现）计算
 *   - MIT 部分按**固件公式**（截断、不钳位）在 Python 侧模拟 float32 计算
 *     ⚠ 不能直接用 cyberbeast_tool.py：它用 round()+钳位，与固件差 ±1 LSB
 *
 * 覆盖：
 *   1. 平台自检（IEEE-754 / 字节序）
 *   2. CAN ID 编解码 + 协议文档 §2.4 四条示例
 *   3. 寻址判定（单播 / 全局广播 / 位图广播 / node_id 0 与 ≥8）
 *   4. MIT 命令打包位级对拍 + 钳位标志 + 往返一致 + 幂等
 *   5. **越界危害回归**：固件语义会回绕；SDK 钳位后必须落在量程内且不变号
 *   6. MIT 响应解包（含温度 int16 边界、错误码、模式）
 *   7. 响应电流满量程（含 80 A 钳位与 40 A 回退）
 *   8. BE/LE 字节序显式校验、广播位图/帧长辅助
 */

#include "cb_frame.h"
#include "cb_mit.h"

#include <stdio.h>
#include <math.h>

/* ⚠⚠ 故意传 NaN/±Inf 的用例**不能**写成常量 `(float)(0.0 / 0.0)`、`(float)(1.0 / 0.0)`：
   MSVC 在**编译期**就把它们判为 C2124（“被零除或对零求模”）而直接失败；
   gcc/clang 只是给个警告，所以以前没暴露。一律走 C99 的 NAN / INFINITY；
   万一某个老工具链没定义，再用运行期算的兼容写法。 */
#ifndef NAN
static float t_nan_(void) { volatile float z = 0.0f; return z / z; }
#  define NAN (t_nan_())
#endif
#ifndef INFINITY
static float t_inf_(void) { volatile float z = 0.0f; return 1.0f / z; }
#  define INFINITY (t_inf_())
#endif

#include "data/golden_vectors.h"

static int g_fail;
static int g_checks;

#define CHECK(cond)                                                            \
    do {                                                                       \
        g_checks++;                                                            \
        if (!(cond)) {                                                         \
            printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);           \
            g_fail++;                                                          \
        }                                                                      \
    } while (0)

#define CHECK_EQ(a, b)                                                         \
    do {                                                                       \
        long long _a = (long long)(a), _b = (long long)(b);                    \
        g_checks++;                                                            \
        if (_a != _b) {                                                        \
            printf("  FAIL %s:%d  %s = %lld, expected %s = %lld\n",            \
                   __FILE__, __LINE__, #a, _a, #b, _b);                        \
            g_fail++;                                                          \
        }                                                                      \
    } while (0)

static int feq(float a, float b)
{
    float tol = 1e-5f * (fabsf(b) + 1.0f);
    return fabsf(a - b) <= tol;
}

#define CHECK_FEQ(a, b)                                                        \
    do {                                                                       \
        float _a = (a), _b = (b);                                              \
        g_checks++;                                                            \
        if (!feq(_a, _b)) {                                                    \
            printf("  FAIL %s:%d  %s = %.9g, expected %s = %.9g\n",            \
                   __FILE__, __LINE__, #a, (double)_a, #b, (double)_b);        \
            g_fail++;                                                          \
        }                                                                      \
    } while (0)

/* ==========================================================================
 * 1. 平台自检与字节序
 * ======================================================================== */

static void test_platform(void)
{
    uint8_t b[8];

    printf("[1] platform selfcheck / byte order\n");
    CHECK_EQ(cb_frame_selfcheck(), 0);

    /* Big-Endian */
    cb_be_put_u16(b, 0x1234u);
    CHECK_EQ(b[0], 0x12u); CHECK_EQ(b[1], 0x34u);
    CHECK_EQ(cb_be_get_u16(b), 0x1234u);

    cb_be_put_u32(b, 0x01020304u);
    CHECK_EQ(b[0], 0x01u); CHECK_EQ(b[3], 0x04u);
    CHECK_EQ(cb_be_get_u32(b), 0x01020304u);

    cb_be_put_u64(b, 0x0102030405060708ull);
    CHECK_EQ(b[0], 0x01u); CHECK_EQ(b[7], 0x08u);
    CHECK_EQ(cb_be_get_u64(b), 0x0102030405060708ull);

    /* f32：-12.5 → C1 48 00 00 */
    cb_be_put_f32(b, -12.5f);
    CHECK_EQ(b[0], 0xC1u); CHECK_EQ(b[1], 0x48u);
    CHECK_EQ(b[2], 0x00u); CHECK_EQ(b[3], 0x00u);
    CHECK_FEQ(cb_be_get_f32(b), -12.5f);

    /* 有符号 */
    cb_be_put_i16(b, -2);
    CHECK_EQ(b[0], 0xFFu); CHECK_EQ(b[1], 0xFEu);
    CHECK_EQ(cb_be_get_i16(b), -2);

    /* ⚙ 参数值在线上是**小端**（设备端 memcpy 主机序，见 cb_frame.h）——
       这里把两套字节序的**布局**都钉死：谁被改反了都会立刻红。 */
    printf("  param-value (LE) codec\n");

    cb_le_put_u16(b, 0xA045u);
    CHECK_EQ(b[0], 0x45u); CHECK_EQ(b[1], 0xA0u);
    CHECK_EQ(cb_le_get_u16(b), 0xA045u);

    cb_le_put_i16(b, -2);
    CHECK_EQ(b[0], 0xFEu); CHECK_EQ(b[1], 0xFFu);
    CHECK_EQ(cb_le_get_i16(b), -2);

    cb_le_put_u32(b, 0x01020304u);
    CHECK_EQ(b[0], 0x04u); CHECK_EQ(b[1], 0x03u);
    CHECK_EQ(b[2], 0x02u); CHECK_EQ(b[3], 0x01u);
    CHECK_EQ(cb_le_get_u32(b), 0x01020304u);
    CHECK_EQ(cb_le_get_i32(b), 0x01020304);

    cb_le_put_u64(b, 0x0102030405060708ull);
    CHECK_EQ(b[0], 0x08u); CHECK_EQ(b[7], 0x01u);
    CHECK_EQ(cb_le_get_u64(b), 0x0102030405060708ull);

    /* f32：-12.5 = 0xC1480000 → 小端字节 C1 48 的**反序** 00 00 48 C1 */
    cb_le_put_f32(b, -12.5f);
    CHECK_EQ(b[0], 0x00u); CHECK_EQ(b[1], 0x00u);
    CHECK_EQ(b[2], 0x48u); CHECK_EQ(b[3], 0xC1u);
    CHECK_FEQ(cb_le_get_f32(b), -12.5f);

    /* 真机实测过的三个值，按小端写出时必须得到设备真正在用的字节 */
    cb_le_put_u32(b, 1u);                          /* node_id */
    CHECK_EQ(cb_le_get_u32(b), 1u);
    CHECK_EQ(b[0], 0x01u); CHECK_EQ(b[1], 0x00u);
    CHECK_EQ(b[2], 0x00u); CHECK_EQ(b[3], 0x00u);

    cb_le_put_u32(b, 100u);                        /* heartbeat_rate_ms */
    CHECK_EQ(b[0], 0x64u); CHECK_EQ(b[1], 0x00u);

    cb_le_put_f32(b, 7.75f);                       /* gear_ratio（真机值） */
    CHECK_EQ(b[0], 0x00u); CHECK_EQ(b[1], 0x00u);
    CHECK_EQ(b[2], 0xF8u); CHECK_EQ(b[3], 0x40u);  /* 0x40F80000 反序 */
    CHECK_FEQ(cb_le_get_f32(b), 7.75f);

    /* 两套字节序必须**互为反序**（同一个值，BE 与 LE 的字节数组正好倒过来） */
    {
        uint8_t be[4], le[4];
        cb_be_put_f32(be, 16.5f);
        cb_le_put_f32(le, 16.5f);
        CHECK_EQ(be[0], le[3]); CHECK_EQ(be[1], le[2]);
        CHECK_EQ(be[2], le[1]); CHECK_EQ(be[3], le[0]);
    }
}

/* ==========================================================================
 * 2. CAN ID
 * ======================================================================== */

static void test_can_id(void)
{
    unsigned i;

    printf("[2] CAN ID (%u vectors)\n", (unsigned)GV_ID_COUNT);

    for (i = 0; i < GV_ID_COUNT; ++i) {
        const gv_id_t *v = &gv_ids[i];
        uint32_t id = cb_make_id(v->pri, v->msgtype, v->dest, v->src, v->seq);

        if (id != v->expected) {
            printf("  FAIL make_id(%u,0x%02X,%u,%u,%u) = 0x%08X, expected 0x%08X\n",
                   v->pri, v->msgtype, v->dest, v->src, v->seq, id, v->expected);
            g_fail++;
        }
        g_checks++;

        CHECK_EQ(cb_id_priority(id), v->pri);
        CHECK_EQ(cb_id_msgtype(id),  v->msgtype);
        CHECK_EQ(cb_id_dest(id),     v->dest);
        CHECK_EQ(cb_id_source(id),   v->src);
        CHECK_EQ(cb_id_seq(id),      v->seq);
    }

    /* 协议文档 §2.4 的四条示例（输入字段与文档逐字一致） */
    {
        static const struct {
            unsigned pri, mt, dest, src, seq; uint32_t expect;
        } k_doc[] = {
            { 2, 0x00, 0x05, 1, 1, 0x08001405u },
            { 2, 0x80, 0xFF, 1, 2, 0x0A03FC06u },
            { 6, 0x48, 0x01, 3, 0, 0x1920040Cu },
            /* ⚠ 上游文档写 0x003FFC04，是笔误（反解得 msgtype=0x0F） */
            { 0, 0xC0, 0xFF, 1, 0, 0x0303FC04u }
        };
        unsigned k;
        for (k = 0; k < sizeof k_doc / sizeof k_doc[0]; ++k) {
            uint32_t id = cb_make_id((uint8_t)k_doc[k].pri, (uint8_t)k_doc[k].mt,
                                     (uint8_t)k_doc[k].dest, (uint8_t)k_doc[k].src,
                                     (uint8_t)k_doc[k].seq);
            if (id != k_doc[k].expect) {
                printf("  FAIL doc example %u: 0x%08X, expected 0x%08X\n",
                       k + 1u, id, k_doc[k].expect);
                g_fail++;
            }
            g_checks++;
        }
        printf("      doc §2.4 examples: 4 checked (incl. corrected ESTOP value)\n");
    }

    /* 广播判定 */
    CHECK(cb_id_is_broadcast(cb_make_id(2, 0x80, 0xFF, 1, 0)));
    CHECK(!cb_id_is_broadcast(cb_make_id(2, 0x00, 5, 1, 0)));
    CHECK(cb_id_is_broadcast(cb_make_id(0, 0x80, 0x01, 1, 0)));   /* 0x80 是分界线 */
    CHECK(!cb_id_is_broadcast(cb_make_id(2, 0x7F, 5, 1, 0)));

    /* seq 滚动 */
    CHECK_EQ(cb_seq_next(0), 1);
    CHECK_EQ(cb_seq_next(1), 2);
    CHECK_EQ(cb_seq_next(2), 3);
    CHECK_EQ(cb_seq_next(3), 0);
}

/* ==========================================================================
 * 3. 寻址
 * ======================================================================== */

static void test_addressing(void)
{
    unsigned i;

    printf("[3] addressing (%u vectors)\n", (unsigned)GV_ADDR_COUNT);
    for (i = 0; i < GV_ADDR_COUNT; ++i) {
        int got = cb_id_is_for_me(gv_addr[i].my_node_id, gv_addr[i].id);
        if (got != (int)gv_addr[i].expected) {
            printf("  FAIL is_for_me(node=%u, id=0x%08X) = %d, expected %u\n",
                   gv_addr[i].my_node_id, gv_addr[i].id, got, gv_addr[i].expected);
            g_fail++;
        }
        g_checks++;
    }

    /* 位图广播辅助 */
    {
        static const uint8_t n12[] = { 1, 2 };
        static const uint8_t n1_8[] = { 1, 8 };
        static const uint8_t n0[]   = { 0 };
        static const uint8_t n7[]   = { 7 };
        uint8_t buf[2048];
        int i2;

        CHECK_EQ(cb_make_broadcast_mask(n12, 2), 0x06);
        CHECK_EQ(cb_make_broadcast_mask(n7, 1), 0x80);
        CHECK_EQ(cb_make_broadcast_mask(n1_8, 2), -1);   /* node_id 8 不可位寻址 */
        CHECK_EQ(cb_make_broadcast_mask(n0, 1), -1);     /* node_id 0 非法 */
        CHECK_EQ(cb_make_broadcast_mask(NULL, 0), -1);

        /* 大数组：{1,8} 里出现 8 → 失败，且验证循环不会越界 */
        for (i2 = 0; i2 < 2048; ++i2) buf[i2] = 1u;
        CHECK_EQ(cb_make_broadcast_mask(buf, 2048), 0x02);
        CHECK_EQ(cb_make_broadcast_mask(buf, 0), -1);
    }

    /* 广播帧长度：(max_slot+1)*8，≥8 槽位不可用 */
    CHECK_EQ(cb_mit_bcast_frame_len(0), 8u);
    CHECK_EQ(cb_mit_bcast_frame_len(7), 64u);
    CHECK_EQ(cb_mit_bcast_frame_len(1), 16u);
    CHECK_EQ(cb_mit_bcast_frame_len(8), 0u);
}

/* ==========================================================================
 * 4. MIT 命令打包
 * ======================================================================== */

static void test_mit_pack(void)
{
    unsigned i, j;

    printf("[4] MIT command pack (%u vectors)\n", (unsigned)GV_MIT_CMD_COUNT);

    for (i = 0; i < GV_MIT_CMD_COUNT; ++i) {
        const gv_mit_cmd_t *v = &gv_mit_cmds[i];
        cb_mit_range_t r;
        uint8_t got[8];
        uint8_t clamped = 0xEEu;
        float pos, vel, kp, kd, tau;

        r.pos_max = v->pos_max; r.vel_max = v->vel_max;
        r.kp_max  = v->kp_max;  r.kd_max  = v->kd_max; r.tau_max = v->tau_max;

        cb_mit_pack_command(got, &r, v->pos, v->vel, v->kp, v->kd, v->tau, &clamped);

        for (j = 0; j < 8u; ++j) {
            if (got[j] != v->bytes[j]) {
                printf("  FAIL vector %u byte[%u] = 0x%02X, expected 0x%02X"
                       "  (pos=%.6f vel=%.6f kp=%.4f kd=%.4f tau=%.4f)\n",
                       i, j, got[j], v->bytes[j],
                       (double)v->pos, (double)v->vel,
                       (double)v->kp, (double)v->kd, (double)v->tau);
                g_fail++;
                break;
            }
            g_checks++;
        }

        if (clamped != v->clamped) {
            printf("  FAIL vector %u clamped = 0x%02X, expected 0x%02X\n",
                   i, clamped, v->clamped);
            g_fail++;
        }
        g_checks++;

        /* 解包往返：与生成器给出的期望值一致 */
        cb_mit_unpack_command(v->bytes, &r, &pos, &vel, &kp, &kd, &tau);
        CHECK_FEQ(pos, gv_mit_cmd_vals[i].pos);
        CHECK_FEQ(vel, gv_mit_cmd_vals[i].vel);
        CHECK_FEQ(kp,  gv_mit_cmd_vals[i].kp);
        CHECK_FEQ(kd,  gv_mit_cmd_vals[i].kd);
        CHECK_FEQ(tau, gv_mit_cmd_vals[i].tau);

        /* 幂等：解出的值重新打包必须得到相同字节 */
        {
            uint8_t again[8];
            cb_mit_pack_command(again, &r, pos, vel, kp, kd, tau, NULL);
            for (j = 0; j < 8u; ++j) {
                if (again[j] != v->bytes[j]) {
                    printf("  FAIL vector %u not idempotent at byte %u"
                           " (0x%02X vs 0x%02X)\n", i, j, again[j], v->bytes[j]);
                    g_fail++;
                    break;
                }
                g_checks++;
            }
        }

        /* 钳位安全性质：越界输入必须落在量程内，且**不得变号**（回绕会变号） */
        if (clamped & CB_MIT_CLAMP_POS) {
            float lsb = 2.0f * r.pos_max / 65535.0f;
            CHECK(pos >= -r.pos_max - lsb && pos <= r.pos_max + lsb);
            if (v->pos > 0.0f) CHECK(pos > 0.0f);
            if (v->pos < 0.0f) CHECK(pos < 0.0f);
        }
        if (clamped & CB_MIT_CLAMP_VEL) {
            float lsb = 2.0f * r.vel_max / 4095.0f;
            CHECK(vel >= -r.vel_max - lsb && vel <= r.vel_max + lsb);
        }
        if (clamped & CB_MIT_CLAMP_TAU) {
            float lsb = 2.0f * r.tau_max / 4095.0f;
            CHECK(tau >= -r.tau_max - lsb && tau <= r.tau_max + lsb);
        }
        if (clamped & CB_MIT_CLAMP_KP) CHECK(kp >= 0.0f && kp <= r.kp_max + 1e-6f);
        if (clamped & CB_MIT_CLAMP_KD) CHECK(kd >= 0.0f && kd <= r.kd_max + 1e-6f);
    }

    /* 满量程必须落在定点边界上 */
    {
        cb_mit_range_t r;
        float a, b, c, d, e;
        uint8_t bytes[8];
        r.pos_max = 12.5f; r.vel_max = 65.0f; r.kp_max = 500.0f;
        r.kd_max = 5.0f;   r.tau_max = 50.0f;

        cb_mit_pack_command(bytes, &r, 12.5f, 65.0f, 500.0f, 5.0f, 50.0f, NULL);
        cb_mit_unpack_command(bytes, &r, &a, &b, &c, &d, &e);
        CHECK_FEQ(a, 12.5f); CHECK_FEQ(b, 65.0f);
        CHECK_FEQ(c, 500.0f); CHECK_FEQ(d, 5.0f); CHECK_FEQ(e, 50.0f);

        cb_mit_pack_command(bytes, &r, -12.5f, -65.0f, 0.0f, 0.0f, -50.0f, NULL);
        cb_mit_unpack_command(bytes, &r, &a, &b, &c, &d, &e);
        CHECK_FEQ(a, -12.5f); CHECK_FEQ(b, -65.0f); CHECK_FEQ(e, -50.0f);
    }

    /* 量程有效性 */
    {
        cb_mit_range_t ok, bad;
        ok.pos_max = 12.5f; ok.vel_max = 65.0f; ok.kp_max = 500.0f;
        ok.kd_max = 5.0f;   ok.tau_max = 50.0f;
        bad = ok; bad.kd_max = 0.0f;
        CHECK(cb_mit_range_valid(&ok));
        CHECK(!cb_mit_range_valid(&bad));
        CHECK(!cb_mit_range_valid(NULL));
    }

    /* 空指针防护不得崩溃 */
    {
        cb_mit_range_t r;
        uint8_t bytes[8];
        uint8_t cl = 0;
        r.pos_max = 1.0f; r.vel_max = 1.0f; r.kp_max = 1.0f;
        r.kd_max = 1.0f;  r.tau_max = 1.0f;
        cb_mit_pack_command(NULL, &r, 0, 0, 0, 0, 0, &cl);
        CHECK_EQ(cl, 0x80u);
        cb_mit_pack_command(bytes, NULL, 0, 0, 0, 0, 0, &cl);
        CHECK_EQ(cl, 0x80u);
    }
}

/* ==========================================================================
 * 5. 越界危害回归
 * ======================================================================== */

static void test_mit_hazard(void)
{
    unsigned i;

    printf("[5] out-of-range hazard (%u vectors)\n", (unsigned)GV_MIT_HAZARD_COUNT);

    for (i = 0; i < GV_MIT_HAZARD_COUNT; ++i) {
        const gv_mit_hazard_t *h = &gv_mit_hazard[i];
        int32_t raw_p = cb_mit_f2u_raw(h->pos, -h->pos_max, h->pos_max, 16);
        int32_t raw_t = cb_mit_f2u_raw(h->tau, -h->tau_max, h->tau_max, 12);

        /* 固件语义（截断、不钳位）必须被精确复刻 */
        CHECK_EQ(raw_p, h->raw_p);
        CHECK_EQ(raw_t, h->raw_t);
        CHECK_EQ((raw_p < 0 || raw_p > 0xFFFF) ? 1 : 0, (int)h->p_overflow);
        CHECK_EQ((raw_t < 0 || raw_t > 0xFFF) ? 1 : 0, (int)h->t_overflow);

        /* 回绕后解出的值会跑到量程另一端（甚至变号）—— 这就是必须钳位的原因 */
        if (h->p_overflow) {
            uint16_t wrapped = (uint16_t)((uint32_t)raw_p & 0xFFFFu);
            float dec = cb_mit_u2f((int32_t)wrapped, -h->pos_max, h->pos_max, 16);
            CHECK_EQ(wrapped, h->wrapped_p);
            if (h->pos > 0.0f) CHECK(dec < 0.0f);      /* 正越界 → 回绕成负 */
            if (h->pos < 0.0f) CHECK(dec > 0.0f);
            printf("      pos=%.4f (%.1f%% over) -> raw=%d wrapped=0x%04X decodes %.3f rad\n",
                   (double)h->pos, (double)((h->pos / h->pos_max - 1.0f) * 100.0f),
                   raw_p, wrapped, (double)dec);
        }

        /* 力矩同理：50.5 Nm（仅超 1.2%）会回绕成 -49.5 Nm —— 反向满力矩！
         * 验证要点：解出的力矩与请求值**严重不符**（这才是危寄的本质）。
         * 注意不能断言“一定变号”：回绕到哪一端取决于掩码后的余数，
         * 例如 tau=150（超 200%）会回绕成 +49.98 Nm，符号碰巧一致。 */
        if (h->t_overflow) {
            uint16_t wrapped = (uint16_t)((uint32_t)raw_t & 0xFFFu);
            float dec = cb_mit_u2f((int32_t)wrapped, -h->tau_max, h->tau_max, 12);
            CHECK_EQ(wrapped, h->wrapped_t);
            CHECK(fabsf(dec - h->tau) > 1.0f);      /* 误差 >= 1 Nm 即不可接受 */
            printf("      tau=%.4f (%.1f%% over) -> raw=%d wrapped=0x%03X decodes %.3f Nm"
                   "  [error %.1f Nm]\n",
                   (double)h->tau,
                   (double)((h->tau / h->tau_max - 1.0f) * 100.0f),
                   raw_t, wrapped, (double)dec, (double)(dec - h->tau));
        }

        /* SDK 路径：钳位后必须在量程内且不变号 */
        {
            cb_mit_range_t r;
            uint8_t bytes[8];
            uint8_t cl = 0;
            float pos, vel, kp, kd, tau;
            r.pos_max = h->pos_max; r.vel_max = 65.0f; r.kp_max = 500.0f;
            r.kd_max = 5.0f;        r.tau_max = h->tau_max;

            cb_mit_pack_command(bytes, &r, h->pos, 0.0f, 0.0f, 0.0f, h->tau, &cl);

            /* 钳位标志只在真正越界时置位（向量里 pos 可能刚好在量程上） */
            if (h->pos < -r.pos_max || h->pos > r.pos_max) {
                CHECK((cl & CB_MIT_CLAMP_POS) != 0u);
            } else {
                CHECK((cl & CB_MIT_CLAMP_POS) == 0u);
            }
            if (h->tau < -r.tau_max || h->tau > r.tau_max) {
                CHECK((cl & CB_MIT_CLAMP_TAU) != 0u);
            } else {
                CHECK((cl & CB_MIT_CLAMP_TAU) == 0u);
            }

            cb_mit_unpack_command(bytes, &r, &pos, &vel, &kp, &kd, &tau);
            CHECK(pos >= -r.pos_max - 1e-3f && pos <= r.pos_max + 1e-3f);
            CHECK(tau >= -r.tau_max - 1e-3f && tau <= r.tau_max + 1e-3f);
            /* 关键安全性质：不得回绕变号（不管原值大小） */
            if (h->pos > 0.0f) CHECK(pos > 0.0f);
            if (h->pos < 0.0f) CHECK(pos < 0.0f);
            if (h->tau > 0.0f) CHECK(tau > 0.0f);
            if (h->tau < 0.0f) CHECK(tau < 0.0f);
        }
    }

    /* 负越界同样处理 */
    {
        cb_mit_range_t r;
        uint8_t bytes[8];
        uint8_t cl = 0;
        float pos;
        r.pos_max = 12.5f; r.vel_max = 65.0f; r.kp_max = 500.0f;
        r.kd_max = 5.0f;   r.tau_max = 50.0f;
        cb_mit_pack_command(bytes, &r, -30.0f, 0, 0, 0, 0, &cl);
        CHECK((cl & CB_MIT_CLAMP_POS) != 0u);
        pos = 0.0f;
        cb_mit_unpack_command(bytes, &r, &pos, NULL, NULL, NULL, NULL);
        CHECK(pos <= -12.49f);
    }

    /* NaN → 置 0 并报 INVALID；±Inf → 边界钳位 */
    {
        cb_mit_range_t r;
        uint8_t bytes[8];
        uint8_t cl = 0;
        float pos = 1.0f;
        /* 位置 16 位对称量程的零点落在两码之间：1 LSB = 25.0/65535 = 0.3815 mrad，
           因此 0 rad 只能表示成 -0.5 LSB 或 +0.5 LSB（协议用 (1<<bits)-1 作比例，
           码数 65535 为奇数，无法让中点恰好落在整数码上）。这是协议固有误差。 */
        const float pos_lsb = 2.0f * 12.5f / 65535.0f;
        r.pos_max = 12.5f; r.vel_max = 65.0f; r.kp_max = 500.0f;
        r.kd_max = 5.0f;   r.tau_max = 50.0f;

        cb_mit_pack_command(bytes, &r, NAN, 0, 0, 0, 0, &cl);
        CHECK((cl & CB_MIT_INVALID) != 0u);
        /* 分层语义：NaN 既置 INVALID（原因），也置该字段的 CLAMP（值被改过），
           这样 `if (flags != 0)` 的调用方不会漏掉 NaN 这种情况 */
        CHECK((cl & CB_MIT_CLAMP_POS) != 0u);
        CHECK_EQ(cl, (unsigned)(CB_MIT_INVALID | CB_MIT_CLAMP_POS));
        cb_mit_unpack_command(bytes, &r, &pos, NULL, NULL, NULL, NULL);
        CHECK(fabsf(pos) <= 0.5f * pos_lsb + 1e-9f);

        /* 零值量化偏置必须小于半 LSB —— 证明四舍五入生效 */
        cl = 0;
        cb_mit_pack_command(bytes, &r, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, &cl);
        CHECK_EQ(cl, 0x00u);
        cb_mit_unpack_command(bytes, &r, &pos, NULL, NULL, NULL, NULL);
        CHECK(fabsf(pos) <= 0.5f * pos_lsb + 1e-9f);

        cl = 0;
        cb_mit_pack_command(bytes, &r, INFINITY, 0, 0, 0, 0, &cl);
        CHECK((cl & CB_MIT_CLAMP_POS) != 0u);
        CHECK((cl & CB_MIT_INVALID) == 0u);      /* Inf 只算钳位，不是 NaN */
        cb_mit_unpack_command(bytes, &r, &pos, NULL, NULL, NULL, NULL);
        CHECK_FEQ(pos, 12.5f);

        /* 负 Inf 同理 */
        cl = 0;
        cb_mit_pack_command(bytes, &r, -INFINITY, 0, 0, 0, 0, &cl);
        CHECK((cl & CB_MIT_CLAMP_POS) != 0u);
        cb_mit_unpack_command(bytes, &r, &pos, NULL, NULL, NULL, NULL);
        CHECK_FEQ(pos, -12.5f);
    }

    /* 量化误差上界：任意值单次编码误差 <= 0.5 LSB（四舍五入的直接后果） */
    {
        cb_mit_range_t r;
        uint8_t bytes[8];
        const float lsb = 2.0f * 12.5f / 65535.0f;
        int k;
        r.pos_max = 12.5f; r.vel_max = 65.0f; r.kp_max = 500.0f;
        r.kd_max = 5.0f;   r.tau_max = 50.0f;

        for (k = 0; k <= 2000; ++k) {
            float want = -12.5f + 25.0f * (float)k / 2000.0f;
            float got;
            uint8_t again[8];
            uint8_t j;

            cb_mit_pack_command(bytes, &r, want, 0, 0, 0, 0, NULL);
            cb_mit_unpack_command(bytes, &r, &got, NULL, NULL, NULL, NULL);
            CHECK(fabsf(got - want) <= 0.5f * lsb + 1e-6f);

            /* 幂等：解出的值重新打包必须得到完全相同的字节 */
            cb_mit_pack_command(again, &r, got, 0, 0, 0, 0, NULL);
            for (j = 0; j < 8u; ++j) {
                if (again[j] != bytes[j]) {
                    printf("  FAIL not idempotent at k=%d byte %u (0x%02X vs 0x%02X)\n",
                           k, j, again[j], bytes[j]);
                    g_fail++;
                    break;
                }
                g_checks++;
            }
        }
    }
}

/* ==========================================================================
 * 6. MIT 响应
 * ======================================================================== */

static void test_mit_response(void)
{
    unsigned i;

    printf("[6] MIT response (%u vectors)\n", (unsigned)GV_MIT_RESP_COUNT);

    for (i = 0; i < GV_MIT_RESP_COUNT; ++i) {
        const gv_mit_resp_t *v = &gv_mit_resps[i];
        cb_mit_range_t r;
        cb_mit_response_t out;

        r.pos_max = 12.5f; r.vel_max = 65.0f; r.kp_max = 500.0f;
        r.kd_max = 5.0f;   r.tau_max = 50.0f;

        cb_mit_unpack_response(v->bytes, &r, v->max_current, &out);
        CHECK_FEQ(out.pos,     v->pos);
        CHECK_FEQ(out.vel,     v->vel);
        CHECK_FEQ(out.current, v->current);
        CHECK_EQ(out.err_code, v->err);
        CHECK_EQ(out.mode,     v->mode);
        CHECK_EQ(out.motor_temp_c, v->motor_temp);
        CHECK_EQ(out.mos_temp_c,   v->mos_temp);

        /* 温度必须用 int16：字节 0xFF → 205 °C（int8 会变成 -51） */
        CHECK(out.motor_temp_c >= -50 && out.motor_temp_c <= 205);
        CHECK(out.mos_temp_c   >= -50 && out.mos_temp_c   <= 205);
    }

    /* 定点温度边界显式校验 */
    {
        cb_mit_range_t r;
        cb_mit_response_t out;
        uint8_t b[8] = { 0, 0, 0, 0, 0, 0, 0x00u, 0xFFu };
        r.pos_max = 12.5f; r.vel_max = 65.0f; r.kp_max = 500.0f;
        r.kd_max = 5.0f;   r.tau_max = 50.0f;
        cb_mit_unpack_response(b, &r, 40.0f, &out);
        CHECK_EQ(out.motor_temp_c, -50);
        CHECK_EQ(out.mos_temp_c, 205);
    }

    /* 量程未知时的安全解码：pos/vel 为 0，电流按 ±40 A 回退 */
    {
        uint8_t b[8] = { 0xFF, 0xFF, 0xFF, 0xF0u, 0xFF, 0xF0u, 0x32u, 0x32u };
        cb_mit_response_t out;
        cb_mit_unpack_response_raw(b, &out);
        CHECK_FEQ(out.pos, 0.0f);
        CHECK_FEQ(out.vel, 0.0f);
        CHECK_EQ(out.err_code, 0x0u);
        CHECK_EQ(out.mode, 0x0u);
        CHECK_EQ(out.motor_temp_c, 0);
    }

    /* 错误码 / 模式名称 */
    CHECK(cb_mit_error_name(CB_ERR_NONE) != NULL);
    CHECK(cb_mit_mode_name(CB_MODE_MIT) != NULL);
    CHECK_EQ((int)CB_ERR_MULTIPLE, 0xF);
    CHECK_EQ((int)CB_MODE_TORQUE, 7);
}

/* ==========================================================================
 * 7. 响应电流满量程
 * ======================================================================== */

static void test_max_current(void)
{
    unsigned i;

    printf("[7] response max current (%u vectors)\n", (unsigned)GV_MAXCUR_COUNT);
    for (i = 0; i < GV_MAXCUR_COUNT; ++i) {
        float got = cb_mit_response_max_current(gv_maxcur[i].tau_max,
                                                gv_maxcur[i].torque_constant);
        CHECK_FEQ(got, gv_maxcur[i].expected);
    }

    /* 边界语义显式确认 */
    CHECK_FEQ(cb_mit_response_max_current(50.0f, 0.0f), 40.0f);      /* 无力矩常数 → 回退 */
    CHECK_FEQ(cb_mit_response_max_current(50.0f, 0.001f), 40.0f);    /* 不 > 0.001 → 回退 */
    CHECK_FEQ(cb_mit_response_max_current(50.0f, 0.002f), 80.0f);    /* 25000 → 钳到 80 */
    CHECK_FEQ(cb_mit_response_max_current(0.08f, 0.002f), 40.0f);    /* 40 → 不钳 */
}

/* ==========================================================================
 * 8. 名称映射枚举完整性
 *
 * 自审项：防止“枚举有值但 switch 漏掉 case”这类静默盲区
 * （本会话已在 JSON 描述符解析器里踩过一次：object 类型被完全遗漏）。
 * ======================================================================== */

static void test_name_coverage(void)
{
    static const uint8_t mt[] = {
        0x00, 0x01, 0x02, 0x03, 0x04,
        0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
        0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
        0x60, 0x61, 0x62, 0x63, 0x64, 0x65,
        0x80, 0x81, 0x82, 0x83,
        0xC0, 0xC1
    };
    unsigned i, k;

    printf("[8] name coverage\n");
    for (i = 0; i < sizeof mt / sizeof mt[0]; ++i) {
        const char *n = cb_msgtype_name(mt[i]);
        if (!n || n[0] == '\0' || (n[0] == 'u' && n[1] == 'n')) {
            printf("  FAIL msgtype 0x%02X has no name\n", mt[i]);
            g_fail++;
        }
        g_checks++;
    }
    CHECK_EQ(sizeof mt / sizeof mt[0], 33u);   /* 固件 MsgType 全部 33 个取值 */

    for (k = 0; k < 8u; ++k) {
        const char *n = cb_priority_name((uint8_t)k);
        if (!n || n[0] == 'u') {
            printf("  FAIL priority %u has no name\n", k);
            g_fail++;
        }
        g_checks++;
    }

    for (k = 0; k <= 0x0Fu; ++k) {
        const char *n = cb_mit_error_name((uint8_t)k);
        if (!n || n[0] == '\0') {
            printf("  FAIL error code 0x%X has no name\n", k);
            g_fail++;
        }
        g_checks++;
    }

    for (k = 0; k <= 7u; ++k) {
        const char *n = cb_mit_mode_name((uint8_t)k);
        if (!n || n[0] == '\0') {
            printf("  FAIL mode %u has no name\n", k);
            g_fail++;
        }
        g_checks++;
    }
}

/* ==========================================================================
 * 9. 与固件编码器的有意偏离（量化误差）
 *
 * 自审项：SDK 用四舍五入，固件 float_to_uint 用截断，两者最多差 1 LSB。
 * 这是一个**有意的设计偏离**，不是 bug；此处固定住它，避免以后被误改回去，
 * 也便于将来若要改成“与固件字节完全一致”时能一键发现影响面。
 * ======================================================================== */

static void test_rounding_deviation(void)
{
    const float want = 1.2345f;
    int32_t raw_fw;
    int32_t raw_sdk;

    printf("[9] rounding vs firmware truncation (intentional)\n");

    raw_fw  = cb_mit_f2u_raw(want, -12.5f, 12.5f, 16);
    raw_sdk = cb_mit_f2u(want, -12.5f, 12.5f, 16);

    CHECK(raw_sdk >= raw_fw);
    CHECK(raw_sdk - raw_fw <= 1);        /* 四舍五入最多比截断大 1 */
    CHECK(raw_sdk >= 0 && raw_sdk <= 65535);

    /* 恰好落在中点附近的值最容易暴露差异 */
    {
        const float mid = 0.0f;
        raw_fw  = cb_mit_f2u_raw(mid, -12.5f, 12.5f, 16);
        raw_sdk = cb_mit_f2u(mid, -12.5f, 12.5f, 16);
        CHECK_EQ(raw_fw, 32767);
        CHECK_EQ(raw_sdk, 32768);       /* 零点落在两码之间 → 取上半格 */
    }

    /* 而钳位这一项**必须**与固件不同：固件会回绕，SDK 必须饱和
     * 固件：(20 - (-12.5)) * 65535 / 25 = 85195.5 → 截断 85195（已超 16 位）
     *       该值下发会被固件掩码成 0x4CCB = 19659 → 解码 ≈ -5.00 rad（错得离谱）*/
    CHECK_EQ(cb_mit_f2u_raw(20.0f, -12.5f, 12.5f, 16), 85195);
    CHECK((uint16_t)(85195 & 0xFFFF) == 0x4CCBu);
    CHECK_EQ(cb_mit_f2u(20.0f, -12.5f, 12.5f, 16), 65535);      /* SDK：饱和 */
}

/* ==========================================================================
 * main
 * ======================================================================== */

int main(void)
{
    printf("=== codec tests (WP2) ===\n\n");
    test_platform();             printf("\n");
    test_can_id();               printf("\n");
    test_addressing();           printf("\n");
    test_mit_pack();             printf("\n");
    test_mit_hazard();           printf("\n");
    test_mit_response();         printf("\n");
    test_max_current();          printf("\n");
    test_name_coverage();        printf("\n");
    test_rounding_deviation();

    printf("\n=== %d checks, %d failures ===\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
