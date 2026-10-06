/**
 * @file    test_proto.c
 * @brief   WP2 第 2 步回归测试：控制帧 / 查询帧 / 心跳 / 参数读写
 *
 * 黄金向量：tests/data/golden_vectors_proto.h
 *   - 心跳、设备信息、错误查询的**期望值取自 cyberbeast_tool.py 的独立解码器**
 *   - AxisState / ControlMode 名称取自**固件 autogen 头文件**（权威来源）
 *   - 参数帧的期望值由生成器按固件 cmd_param_* 逐行复刻
 *
 * 覆盖：
 *   1. POS 控制（Classic 8B / FD 12B）逐字节 + 量化期望值 + 钳位
 *   2. VEL / TORQUE / CURRENT 逐字节 + 单位约定 + NaN 防护
 *   3. 控制帧元信息：最小长度、是否应答、CURRENT 无广播
 *   4. 查询帧 0x41..0x47 编解码
 *   5. 查询请求构造 + 响应长度表
 *   6. 心跳 Classic 8B / FD 18B（期望值来自参考工具）+ 4 bit 状态溢出回归
 *   7. AxisState / ControlMode 名称与固件 autogen 逐条对齐（含"工具已过时"断言）
 *   8. 单参数读：设备侧切片 + More 位 + 越界 + Classic ReqLen 归一化
 *   9. 批量读：请求/响应/ERR + 装箱器
 *  10. 参数写：请求 / 静默确认 / 分段写装配器（含全部中止条件）
 */

#include "cb_ctrl.h"
#include "cb_mit.h"
#include "cb_query.h"
#include "cb_heartbeat.h"
#include "cb_param.h"
#include "cb_frame.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "data/golden_vectors_proto.h"

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
    return fabsf(a - b) <= 1e-5f * (fabsf(b) + 1.0f);
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

#define CHECK_BYTES(got, exp, n)                                               \
    do {                                                                       \
        unsigned _i;                                                           \
        for (_i = 0; _i < (unsigned)(n); ++_i) {                               \
            g_checks++;                                                        \
            if ((uint8_t)(got)[_i] != (uint8_t)(exp)[_i]) {                    \
                printf("  FAIL %s:%d  byte[%u] = 0x%02X, expected 0x%02X\n",   \
                       __FILE__, __LINE__, _i, (unsigned)(uint8_t)(got)[_i],    \
                       (unsigned)(uint8_t)(exp)[_i]);                          \
                g_fail++;                                                      \
                break;                                                         \
            }                                                                  \
        }                                                                      \
    } while (0)

/* ==========================================================================
 * 1. POS_CONTROL
 * ======================================================================== */

static void test_ctrl_pos(void)
{
    unsigned i;

    printf("[1] POS_CONTROL (%u classic + %u fd)\n",
           (unsigned)GV_CTRL_POS_C_COUNT, (unsigned)GV_CTRL_POS_FD_COUNT);

    /* --- Classic 8 B --- */
    for (i = 0; i < GV_CTRL_POS_C_COUNT; ++i) {
        const gv_ctrl_pos_c_t *v = &gv_ctrl_pos_c[i];
        uint8_t got[8];
        uint8_t flags = 0xEEu;
        float pos, vel, cur;
        size_t n = cb_ctrl_pos_pack(got, 1, v->pos_deg, v->vel_limit_rpm,
                                    v->cur_limit_a, &flags);

        CHECK_EQ(n, 8u);
        CHECK_BYTES(got, v->bytes, 8);

        CHECK_EQ(cb_ctrl_pos_unpack(v->bytes, 8, 1, &pos, &vel, &cur), 0);
        CHECK_FEQ(pos, v->pos_deg);
        CHECK_FEQ(vel, v->exp_vel_rpm);
        CHECK_FEQ(cur, v->exp_cur_a);

        /* 越界必须置钳位标志；不越界不得置 */
        if (v->vel_limit_rpm < -32768.0f || v->vel_limit_rpm > 32767.0f) {
            CHECK((flags & CB_CTRL_POS_F_VEL) != 0u);
        }
        if (v->cur_limit_a < -3276.8f || v->cur_limit_a > 3276.7f) {
            CHECK((flags & CB_CTRL_POS_F_CUR) != 0u);
        }
        /* 位置是 float32，永远不该被报告为"被修正" */
        CHECK((flags & CB_CTRL_POS_F_POS) == 0u);
    }

    /* --- FD 12 B --- */
    for (i = 0; i < GV_CTRL_POS_FD_COUNT; ++i) {
        const gv_ctrl_pos_fd_t *v = &gv_ctrl_pos_fd[i];
        uint8_t got[12];
        uint8_t flags = 0xEEu;
        float pos, vel, cur;
        size_t n = cb_ctrl_pos_pack(got, 0, v->pos_deg, v->vel_limit_rpm,
                                    v->cur_limit_a, &flags);

        CHECK_EQ(n, 12u);
        CHECK_BYTES(got, v->bytes, 12);
        CHECK_EQ(flags, 0x00u);        /* FD 全 float32，无需钳位 */

        CHECK_EQ(cb_ctrl_pos_unpack(v->bytes, 12, 0, &pos, &vel, &cur), 0);
        CHECK_FEQ(pos, v->pos_deg);
        CHECK_FEQ(vel, v->vel_limit_rpm);   /* FD 无量化，应精确相等 */
        CHECK_FEQ(cur, v->cur_limit_a);
    }

    /* 长度不足必须拒绝（与固件 `if (msg.len < N) return;` 对应） */
    CHECK_EQ(cb_ctrl_pos_unpack(gv_ctrl_pos_fd[0].bytes, 8, 0, NULL, NULL, NULL), -1);
    CHECK_EQ(cb_ctrl_pos_unpack(gv_ctrl_pos_c[0].bytes, 7, 1, NULL, NULL, NULL), -1);
    CHECK_EQ(cb_ctrl_pos_pack(NULL, 1, 0, 0, 0, NULL), 0u);
}

/* ==========================================================================
 * 2. VEL / TORQUE / CURRENT
 * ======================================================================== */

static void test_ctrl_others(void)
{
    unsigned i;

    printf("[2] VEL/TORQUE/CURRENT\n");

    for (i = 0; i < GV_CTRL_VEL_COUNT; ++i) {
        const gv_ctrl_vel_t *v = &gv_ctrl_vel[i];
        uint8_t got[8];
        uint8_t flags = 0xEEu;
        float vel, cur;
        size_t n = cb_ctrl_vel_pack(got, v->vel_rpm, v->cur_a, &flags);
        CHECK_EQ(n, 8u);
        CHECK_BYTES(got, v->bytes, 8);
        CHECK_EQ(cb_ctrl_vel_unpack(v->bytes, 8, &vel, &cur), 0);
        CHECK_FEQ(vel, v->vel_rpm);
        CHECK_FEQ(cur, v->cur_a);
        CHECK_EQ(flags, 0x00u);
    }

    for (i = 0; i < GV_CTRL_TAU_COUNT; ++i) {
        const gv_ctrl_tau_t *v = &gv_ctrl_tau[i];
        uint8_t got[4];
        uint8_t flags = 0xEEu;
        float tau;
        CHECK_EQ(cb_ctrl_torque_pack(got, v->tau_nm, &flags), 4u);
        CHECK_BYTES(got, v->bytes, 4);
        CHECK_EQ(cb_ctrl_torque_unpack(v->bytes, 4, &tau), 0);
        CHECK_FEQ(tau, v->tau_nm);
        CHECK_EQ(flags, 0x00u);    /* float32 载荷无线上量程，不钳位 */
    }

    for (i = 0; i < GV_CTRL_CUR_COUNT; ++i) {
        const gv_ctrl_cur_t *v = &gv_ctrl_cur[i];
        uint8_t got[4];
        uint8_t flags = 0xEEu;
        float cur;
        CHECK_EQ(cb_ctrl_current_pack(got, v->cur_a, &flags), 4u);
        CHECK_BYTES(got, v->bytes, 4);
        CHECK_EQ(cb_ctrl_current_unpack(v->bytes, 4, &cur), 0);
        CHECK_FEQ(cur, v->cur_a);
        CHECK_EQ(flags, 0x00u);
    }

    /* NaN 防护：必须置 INVALID 且该字段被修正，载荷为 0 */
    {
        uint8_t b[12];
        uint8_t f = 0u;
        float   pos = 0.0f;
        cb_ctrl_pos_pack(b, 0, NAN, 0, 0, &f);
        CHECK((f & CB_CTRL_F_INVALID) != 0u);
        CHECK((f & CB_CTRL_POS_F_POS) != 0u);
        cb_ctrl_pos_unpack(b, 12, 0, &pos, NULL, NULL);
        CHECK_FEQ(pos, 0.0f);

        f = 0u;
        cb_ctrl_vel_pack(b, NAN, 0, &f);
        CHECK((f & CB_CTRL_F_INVALID) != 0u);
        CHECK((f & CB_CTRL_VEL_F_VEL) != 0u);

        f = 0u;
        cb_ctrl_torque_pack(b, NAN, &f);
        CHECK((f & CB_CTRL_F_INVALID) != 0u);

        f = 0u;
        cb_ctrl_current_pack(b, NAN, &f);
        CHECK((f & CB_CTRL_F_INVALID) != 0u);
    }

    /* ±Inf 只算钳位（Classic 定点路径） */
    {
        uint8_t b[8];
        uint8_t f = 0u;
        float vel = 0.0f;
        cb_ctrl_pos_pack(b, 1, 0.0f, INFINITY, -INFINITY, &f);
        CHECK((f & CB_CTRL_POS_F_VEL) != 0u);
        CHECK((f & CB_CTRL_POS_F_CUR) != 0u);
        CHECK((f & CB_CTRL_F_INVALID) == 0u);
        cb_ctrl_pos_unpack(b, 8, 1, NULL, &vel, NULL);
        CHECK_FEQ(vel, 32767.0f);
    }
}

/* ==========================================================================
 * 3. 控制帧元信息
 * ======================================================================== */

static void test_ctrl_meta(void)
{
    printf("[3] control meta\n");

    /* 最小长度（与固件逐条对应） */
    CHECK_EQ(cb_ctrl_min_len(CB_MSG_POS_CONTROL, 1), 8u);
    CHECK_EQ(cb_ctrl_min_len(CB_MSG_POS_CONTROL, 0), 12u);
    CHECK_EQ(cb_ctrl_min_len(CB_MSG_POS_CONTROL_BCAST, 1), 8u);
    CHECK_EQ(cb_ctrl_min_len(CB_MSG_VEL_CONTROL, 1), 8u);
    CHECK_EQ(cb_ctrl_min_len(CB_MSG_VEL_CONTROL, 0), 8u);   /* 两种变体同布局 */
    CHECK_EQ(cb_ctrl_min_len(CB_MSG_VEL_CONTROL_BCAST, 0), 8u);
    CHECK_EQ(cb_ctrl_min_len(CB_MSG_TORQUE_CONTROL, 0), 4u);
    CHECK_EQ(cb_ctrl_min_len(CB_MSG_TORQUE_CONTROL_BCAST, 0), 4u);
    CHECK_EQ(cb_ctrl_min_len(CB_MSG_CURRENT_CONTROL, 0), 4u);
    CHECK_EQ(cb_ctrl_min_len(CB_MSG_MIT_CONTROL, 0), 8u);
    CHECK_EQ(cb_ctrl_min_len(CB_MSG_QUERY_STATUS, 0), 0u);  /* 不是控制帧 */

    /* 是否属于控制帧 —— ⚠ CURRENT 没有广播变体 0x84 */
    CHECK(cb_ctrl_is_control_msgtype(CB_MSG_MIT_CONTROL));
    CHECK(cb_ctrl_is_control_msgtype(CB_MSG_POS_CONTROL_BCAST));
    CHECK(cb_ctrl_is_control_msgtype(CB_MSG_TORQUE_CONTROL_BCAST));
    CHECK(cb_ctrl_is_control_msgtype(CB_MSG_CURRENT_CONTROL));
    CHECK(!cb_ctrl_is_control_msgtype(0x84u));   /* 不存在 */
    CHECK(!cb_ctrl_is_control_msgtype(CB_MSG_HEARTBEAT));
    CHECK(!cb_ctrl_is_control_msgtype(CB_MSG_PARAM_READ));

    /* 应答：POS/VEL/TORQUE 会回 MIT 响应；CURRENT 不回（已由工具 send_cmd 佐证） */
    CHECK_EQ(cb_ctrl_expects_response(CB_MSG_POS_CONTROL), 1);
    CHECK_EQ(cb_ctrl_expects_response(CB_MSG_VEL_CONTROL), 1);
    CHECK_EQ(cb_ctrl_expects_response(CB_MSG_TORQUE_CONTROL), 1);
    CHECK_EQ(cb_ctrl_expects_response(CB_MSG_CURRENT_CONTROL), 0);
    CHECK_EQ(cb_ctrl_expects_response(CB_MSG_MIT_CONTROL), 0);   /* 广播不可达此处 */

    /* ===== 自审：接口之间的交叉一致性（防止枚举盲区）=====
       1) is_control_msgtype(mt) 为真 ⇒ 两种变体的 min_len 都必须 > 0
       2) expects_response(mt) 为真 ⇒ 必须也是控制帧
       3) 两种变体的 min_len 要么都是 0，要么都 > 0（不能只在某一种下有效）
       4) min_len 必须 ≤ 该 MsgType 在 CAN FD 下可能的长度上限（64） */
    {
        unsigned mt;
        int n_ctrl = 0;
        for (mt = 0u; mt <= 0xFFu; ++mt) {
            int is_ctrl = cb_ctrl_is_control_msgtype((uint8_t)mt);
            size_t len_c = cb_ctrl_min_len((uint8_t)mt, 1);
            size_t len_f = cb_ctrl_min_len((uint8_t)mt, 0);
            int wants_rsp = cb_ctrl_expects_response((uint8_t)mt);

            if (is_ctrl) {
                n_ctrl++;
                CHECK(len_c > 0u);
                CHECK(len_f > 0u);
                CHECK(len_f <= CB_PARAM_FD_FRAME_MAX);
                CHECK(len_c <= CB_PARAM_CLASSIC_FRAME);
            } else {
                CHECK_EQ(len_c, 0u);
                CHECK_EQ(len_f, 0u);
                CHECK_EQ(wants_rsp, 0);
            }
            if (wants_rsp) CHECK(is_ctrl);
        }
        /* 固件里正好有 9 个实时控制 MsgType：0x00..0x04 + 0x80..0x83 */
        CHECK_EQ(n_ctrl, 9);
    }
}

/* ==========================================================================
 * 4~5. 查询帧
 * ======================================================================== */

static void test_query(void)
{
    unsigned i;

    printf("[4] query responses (%u f32x2 + %u err + %u dev)\n",
           (unsigned)GV_Q_F32X2_COUNT, (unsigned)GV_Q_ERR_COUNT,
           (unsigned)GV_Q_DEV_COUNT);

    for (i = 0; i < GV_Q_F32X2_COUNT; ++i) {
        const gv_q_f32x2_t *v = &gv_q_f32x2[i];
        float a = 0.0f, b = 0.0f;

        switch (v->kind) {
        case 0: { cb_query_pos_vel_t o;
                  CHECK_EQ(cb_query_decode_pos_vel(v->bytes, 8, &o), 0);
                  a = o.pos_turns; b = o.vel_turns_per_s; break; }
        case 1: { cb_query_current_t o;
                  CHECK_EQ(cb_query_decode_current(v->bytes, 8, &o), 0);
                  a = o.iq_a; b = o.id_a; break; }
        case 2: { cb_query_temp_t o;
                  CHECK_EQ(cb_query_decode_temp(v->bytes, 8, &o), 0);
                  a = o.motor_c; b = o.fet_c; break; }
        case 3: { cb_query_bus_t o;
                  CHECK_EQ(cb_query_decode_bus(v->bytes, 8, &o), 0);
                  a = o.vbus_v; b = o.ibus_a; break; }
        default: { cb_query_power_t o;
                   CHECK_EQ(cb_query_decode_power(v->bytes, 8, &o), 0);
                   a = o.elec_w; b = o.mech_w; break; }
        }
        CHECK_FEQ(a, v->a);
        CHECK_FEQ(b, v->b);
    }

    /* 编码往返必须与向量逐字节一致 */
    for (i = 0; i < GV_Q_F32X2_COUNT; ++i) {
        const gv_q_f32x2_t *v = &gv_q_f32x2[i];
        uint8_t got[8];
        memset(got, 0xEE, sizeof got);
        switch (v->kind) {
        case 0: { cb_query_pos_vel_t o; o.pos_turns = v->a; o.vel_turns_per_s = v->b;
                  CHECK_EQ(cb_query_encode_pos_vel(got, &o), 8u); break; }
        case 1: { cb_query_current_t o; o.iq_a = v->a; o.id_a = v->b;
                  CHECK_EQ(cb_query_encode_current(got, &o), 8u); break; }
        case 2: { cb_query_temp_t o; o.motor_c = v->a; o.fet_c = v->b;
                  CHECK_EQ(cb_query_encode_temp(got, &o), 8u); break; }
        case 3: { cb_query_bus_t o; o.vbus_v = v->a; o.ibus_a = v->b;
                  CHECK_EQ(cb_query_encode_bus(got, &o), 8u); break; }
        default: { cb_query_power_t o; o.elec_w = v->a; o.mech_w = v->b;
                   CHECK_EQ(cb_query_encode_power(got, &o), 8u); break; }
        }
        CHECK_BYTES(got, v->bytes, 8);
    }

    /* 0x45 QUERY_ERROR */
    for (i = 0; i < GV_Q_ERR_COUNT; ++i) {
        const gv_q_err_t *v = &gv_q_err[i];
        cb_query_error_t o;
        uint8_t got[8];
        CHECK_EQ(cb_query_decode_error(v->bytes, 8, &o), 0);
        CHECK_EQ(o.err_type, v->err_type);
        CHECK_EQ(o.err_value, v->err_value);
        CHECK_EQ(cb_query_encode_error(got, &o), 8u);
        CHECK_BYTES(got, v->bytes, 8);
    }

    /* 0x46 QUERY_DEVICE_INFO：Classic 8B / FD 16B */
    for (i = 0; i < GV_Q_DEV_COUNT; ++i) {
        const gv_q_dev_t *v = &gv_q_dev[i];
        cb_query_device_info_t o;
        uint8_t got[16];

        CHECK_EQ(cb_query_decode_device(v->classic, 8, &o), 0);
        CHECK_EQ(o.hw_ver, v->hw_ver);
        CHECK_EQ(o.fw_ver, v->fw_ver);
        CHECK_EQ(o.has_serial, 0);          /* Classic 无序列号 */
        CHECK_EQ(o.serial, 0u);

        CHECK_EQ(cb_query_decode_device(v->fd, 16, &o), 0);
        CHECK_EQ(o.hw_ver, v->hw_ver);
        CHECK_EQ(o.fw_ver, v->fw_ver);
        CHECK_EQ(o.has_serial, 1);
        CHECK(o.serial == v->serial);

        CHECK_EQ(cb_query_encode_device(got, &o, 1), 8u);
        CHECK_BYTES(got, v->classic, 8);
        CHECK_EQ(cb_query_encode_device(got, &o, 0), 16u);
        CHECK_BYTES(got, v->fd, 16);
    }

    /* 版本号打包只占**低 24 位**：(MAJOR << 16) | (MINOR << 8) | 第三段 */
    CHECK_EQ(CB_QUERY_VER_PACK(1, 2, 3), 0x010203u);
    CHECK_EQ(CB_QUERY_VER_MAJOR(0x010203u), 1u);
    CHECK_EQ(CB_QUERY_VER_MINOR(0x010203u), 2u);
    CHECK_EQ(CB_QUERY_VER_THIRD(0x010203u), 3u);
    CHECK_EQ(CB_QUERY_VER_PACK(8, 9, 2), 0x080902u);
    /* hw 的第三段是 VARIANT，fw 的第三段是 REVISION —— 打包方式相同、语义不同 */
    CHECK_EQ(CB_QUERY_VER_THIRD(CB_QUERY_VER_PACK(1, 2, 255)), 255u);
    CHECK_EQ(CB_QUERY_VER_PACK(255, 255, 255), 0xFFFFFFu);

    /* 长度不足必须拒绝 */
    CHECK_EQ(cb_query_decode_pos_vel(gv_q_f32x2[0].bytes, 7, NULL), -1);
    CHECK_EQ(cb_query_decode_error(gv_q_err[0].bytes, 7, NULL), -1);
    CHECK_EQ(cb_query_decode_device(gv_q_dev[0].classic, 7, NULL), -1);

    printf("[5] query requests / length table\n");

    /* 请求长度 */
    CHECK_EQ(cb_query_request_len(CB_MSG_QUERY_STATUS), 0);
    CHECK_EQ(cb_query_request_len(CB_MSG_QUERY_DEVICE_INFO), 0);
    CHECK_EQ(cb_query_request_len(CB_MSG_QUERY_ERROR), 1);
    CHECK_EQ(cb_query_request_len(CB_MSG_HEARTBEAT), -1);   /* 不是查询帧 */

    /* 只有 0x45 写 1 字节，其余不写 */
    {
        uint8_t b[4];
        memset(b, 0xEE, sizeof b);
        CHECK_EQ(cb_query_build_request(b, 4, CB_MSG_QUERY_ERROR, 3), 1u);
        CHECK_EQ(b[0], 3u);
        memset(b, 0xEE, sizeof b);
        CHECK_EQ(cb_query_build_request(b, 4, CB_MSG_QUERY_POS_VEL, 0), 0u);
        CHECK_EQ(b[0], 0xEEu);                       /* 未写入 */
        CHECK_EQ(cb_query_build_request(b, 4, CB_MSG_HEARTBEAT, 0), 0u);
    }

    /* 响应长度表（⚠ 0x46 的 Classic/FD 不同） */
    CHECK_EQ(cb_query_response_len(CB_MSG_QUERY_STATUS, 0), 8u);   /* MIT 响应帧 */
    CHECK_EQ(cb_query_response_len(CB_MSG_QUERY_POS_VEL, 0), 8u);
    CHECK_EQ(cb_query_response_len(CB_MSG_QUERY_CURRENT, 0), 8u);
    CHECK_EQ(cb_query_response_len(CB_MSG_QUERY_TEMPERATURE, 0), 8u);
    CHECK_EQ(cb_query_response_len(CB_MSG_QUERY_BUS, 0), 8u);
    CHECK_EQ(cb_query_response_len(CB_MSG_QUERY_ERROR, 0), 8u);
    CHECK_EQ(cb_query_response_len(CB_MSG_QUERY_POWER, 0), 8u);
    CHECK_EQ(cb_query_response_len(CB_MSG_QUERY_DEVICE_INFO, 1), 8u);
    CHECK_EQ(cb_query_response_len(CB_MSG_QUERY_DEVICE_INFO, 0), 16u);
    CHECK_EQ(cb_query_response_len(CB_MSG_HEARTBEAT, 0), 0u);

    /* ErrorType 名称无遗漏 */
    CHECK(cb_error_type_name(CB_ET_MOTOR)[0] != 'u');
    CHECK(cb_error_type_name(CB_ET_ENCODER)[0] != 'u');
    CHECK(cb_error_type_name(CB_ET_SENSORLESS)[0] != 'u');
    CHECK(cb_error_type_name(CB_ET_CONTROLLER)[0] != 'u');
    CHECK(cb_error_type_name(CB_ET_SYSTEM)[0] != 'u');
    CHECK(cb_error_type_name(CB_ET_AXIS)[0] != 'u');
    CHECK(strcmp(cb_error_type_name(6u), "unknown") == 0);
}

/* ==========================================================================
 * 6. 心跳
 * ======================================================================== */

static void test_heartbeat(void)
{
    unsigned i;

    printf("[6] heartbeat (%u classic + %u fd)\n",
           (unsigned)GV_HB_C_COUNT, (unsigned)GV_HB_F_COUNT);

    for (i = 0; i < GV_HB_C_COUNT; ++i) {
        const gv_hb_c_t *v = &gv_hb_c[i];
        cb_heartbeat_t o;
        uint8_t got[8];

        CHECK_EQ(cb_heartbeat_decode(v->bytes, 8, &o), 0);
        CHECK_EQ(o.is_fd, 0);
        CHECK_EQ(o.life, v->life);
        CHECK_EQ(o.err_flags, v->err_flags);
        CHECK_EQ(o.state, v->state);
        CHECK_EQ(o.control_mode, v->control_mode);
        CHECK_EQ(o.motor_temp_c, v->motor_temp_c);
        CHECK_FEQ(o.pos_turns, v->pos_turns);
        CHECK_FEQ(o.vel_turns_per_s, v->vel_turns_s);
        CHECK_FEQ(o.iq_a, v->iq_a);
        CHECK_EQ(o.have_mos_temp, 0);       /* Classic 无 MOS 温度 */
        CHECK_EQ(o.have_vbus, 0);           /* Classic 无母线 */

        /* 逐字段辅助与整体解码必须一致 */
        CHECK_EQ(cb_heartbeat_life(v->bytes[0]), v->life);
        CHECK_EQ(cb_heartbeat_flags(v->bytes[0]), v->err_flags);
        CHECK_EQ(cb_heartbeat_state(v->bytes[1]), v->state);
        CHECK_EQ(cb_heartbeat_control_mode(v->bytes[1]), v->control_mode);

        /* 编码往返 */
        CHECK_EQ(cb_heartbeat_encode_classic(got, &o), 8u);
        CHECK_BYTES(got, v->bytes, 8);
    }

    for (i = 0; i < GV_HB_F_COUNT; ++i) {
        const gv_hb_f_t *v = &gv_hb_f[i];
        cb_heartbeat_t o;
        uint8_t got[18];

        CHECK_EQ(cb_heartbeat_decode(v->bytes, 18, &o), 0);
        CHECK_EQ(o.is_fd, 1);
        CHECK_EQ(o.life, v->life);
        CHECK_EQ(o.err_flags, v->err_flags);
        CHECK_EQ(o.state, v->state);
        CHECK_EQ(o.control_mode, v->control_mode);
        CHECK_EQ(o.motor_temp_c, v->motor_temp_c);
        CHECK_EQ(o.mos_temp_c, v->mos_temp_c);
        CHECK_FEQ(o.vbus_v, v->vbus_v);
        CHECK_FEQ(o.ibus_a, v->ibus_a);
        CHECK_FEQ(o.pos_turns, v->pos_turns);
        CHECK_FEQ(o.vel_turns_per_s, v->vel_turns_s);
        CHECK_FEQ(o.iq_a, v->iq_a);
        CHECK_EQ(o.have_mos_temp, 1);
        CHECK_EQ(o.have_vbus, 1);

        CHECK_EQ(cb_heartbeat_encode_fd(got, &o), 18u);
        /* ⚠ 位置/速度存在 float32 精度上限：FD 的 LSB 是 0.0001 turns，
           而 float32 在 |turns| > ~840 时 ulp 已超过该 LSB，因此极端幅值下
           重新编码**不可能**逐字节重现（见 cb_heartbeat.h 说明 4）。
           其余字节仍要求精确相等。 */
        {
            uint8_t exp_fuzzy[18];
            unsigned k;
            memcpy(exp_fuzzy, v->bytes, 18u);
            memcpy(exp_fuzzy + 8,  got + 8,  4u);   /* pos 允许 1 LSB 内偏差 */
            memcpy(exp_fuzzy + 12, got + 12, 4u);   /* vel 同上 */
            for (k = 0u; k < 18u; ++k) {
                if (exp_fuzzy[k] != got[k]) {
                    printf("  FAIL hb_f[%u] byte[%u] = 0x%02X, expected 0x%02X\n",
                           i, k, got[k], exp_fuzzy[k]);
                    g_fail++;
                    break;
                }
                g_checks++;
            }
            /* pos/vel 的偏差必须 ≤ 1 个 int32 LSB（0.0001 turns） */
            CHECK(fabsf(o.pos_turns - v->pos_turns) <= 0.00011f +
                  fabsf(v->pos_turns) * 1.3e-7f);
            CHECK(fabsf(o.vel_turns_per_s - v->vel_turns_s) <= 0.00011f +
                  fabsf(v->vel_turns_s) * 1.3e-7f);
        }
    }

    /* 长度分派 */
    CHECK_EQ(cb_heartbeat_decode(gv_hb_c[0].bytes, 8, NULL), 0);
    CHECK_EQ(cb_heartbeat_decode(gv_hb_f[0].bytes, 18, NULL), 0);
    CHECK_EQ(cb_heartbeat_decode(gv_hb_f[0].bytes, 64, NULL), 0);   /* 更长也接受 */
    CHECK_EQ(cb_heartbeat_decode(gv_hb_c[0].bytes, 7, NULL), -1);
    CHECK_EQ(cb_heartbeat_decode(gv_hb_c[0].bytes, 9, NULL), -1);   /* 9 不是合法长度 */
    CHECK(cb_heartbeat_len_valid(8));
    CHECK(cb_heartbeat_len_valid(18));
    CHECK(!cb_heartbeat_len_valid(9));

    /* life 连续性（丢帧检测） */
    CHECK_EQ(cb_heartbeat_life_is_next(0, 1), 1);
    CHECK_EQ(cb_heartbeat_life_is_next(7, 0), 1);
    CHECK_EQ(cb_heartbeat_life_is_next(3, 5), 0);
    CHECK_EQ(cb_heartbeat_life_is_next(7, 7), 0);

    /* 温度边界：u8=255 → 205 °C（⚠ 必须 16 位，int8 会变成 −51） */
    {
        uint8_t b[8] = { 0, 0, 0x00u, 0, 0, 0, 0, 0 };
        cb_heartbeat_t o;
        b[2] = 0x00u;
        CHECK_EQ(cb_heartbeat_decode(b, 8, &o), 0);
        CHECK_EQ(o.motor_temp_c, -50);
        b[2] = 0xFFu;
        CHECK_EQ(cb_heartbeat_decode(b, 8, &o), 0);
        CHECK_EQ(o.motor_temp_c, 205);
        CHECK(o.motor_temp_c > 127);        /* 证明 int8 装不下 */
    }

    /* ===== 回归：AxisState 16 无法用 4 bit 表示 ===== */
    {
        int sm = cb_heartbeat_make_state_mode(16u, 3u);
        uint8_t b[18];
        cb_heartbeat_t o;
        unsigned k;

        CHECK_EQ(sm, -1);                   /* SDK 明确拒绝，而不是静默丢数据 */
        CHECK_EQ(cb_heartbeat_make_state_mode(15u, 3u), 0xF3);   /* (15<<4)|3 */

        /* 固件的写法会静默截断成 0 —— 用向量证明 */
        for (k = 0; k < 18u; ++k) b[k] = 0u;
        b[1] = (uint8_t)(((16u & 0x0Fu) << 4) | 3u);   /* 复刻固件表达式 */
        CHECK_EQ(b[1], 0x03u);
        CHECK_EQ(cb_heartbeat_decode(b, 18, &o), 0);
        CHECK_EQ(o.state, 0u);              /* 16 → 0：状态信息静默丢失 */
        CHECK_EQ(o.control_mode, 3u);

        /* 最大 4 bit 状态仍可表达 */
        CHECK_EQ(cb_heartbeat_make_state_mode(15u, 0u), 0xF0);
    }

    /* ===== 回归：F32 —— FD 心跳 pos/vel 量程必须按**原始计数域**钳位 =====
     *
     * 固件曾把 FD 的 pos/vel 钳在 ±214748.0f（**物理量域**的 214748 turns，
     * 因为源码里写成 `214748.0f` 当"原始计数上界"）而不是 ±2^31（原始计数域）。
     * 后果：电机端位置一旦超过 214748/10000 = 21.4748 turns，心跳 pos 就静默
     * 饱和到 214748（0x000346DC）—— 21.47 turns 只是 int32 满量程的 **0.01%**，
     * 对 7.75 齿比的关节也就不到 3 圈输出端，属于必现区间。
     *
     * 固件 2f72ea09 用 `scale_clamped()` 根治。SDK 侧是**忠实解码**，本不该
     * 受影响；但此前的黄金向量只到 214748.0 turns（0x7FFFF1C0），恰好**避开**
     * 了 (214748.0, 214748.3648] 这段"旧 bug 判别区"，等于没有覆盖 ——
     * 这正是 F32 能静默溜进固件的原因之一。下面把该区间的边界钉死。
     */
    {
        cb_heartbeat_t o;
        uint8_t b[18];
        unsigned k;

        /* 1) 21.4748 turns —— 旧 bug 的触发阈值，必须能正常解出，不得饱和 */
        for (k = 0u; k < 18u; ++k) b[k] = 0u;
        {
            /* raw = 214748 = 0x000346DC（旧 bug 的饱和值） */
            b[8] = 0x00u; b[9] = 0x03u; b[10] = 0x46u; b[11] = 0xDCu;
            CHECK_EQ(cb_heartbeat_decode(b, 18, &o), 0);
            CHECK_FEQ(o.pos_turns, 21.4748f);
        }

        /* 2) 214748.0 turns（旧黄金向量的值）—— 旧 bug 恰好还能正确表达 */
        for (k = 0u; k < 18u; ++k) b[k] = 0u;
        b[8] = 0x7Fu; b[9] = 0xFFu; b[10] = 0xF1u; b[11] = 0xC0u;
        CHECK_EQ(cb_heartbeat_decode(b, 18, &o), 0);
        CHECK_FEQ(o.pos_turns, 214748.0f);

        /* 3) INT32_MAX（214748.3647 turns）—— **旧 bug 判别区**的核心：
              旧实现把上界当 214748.0，本值会被钳到 214748.0；正确满量程是
              214748.3648 turns，故这里必须解出 > 214748.0，而不是等于它。 */
        for (k = 0u; k < 18u; ++k) b[k] = 0u;
        b[8] = 0x7Fu; b[9] = 0xFFu; b[10] = 0xFFu; b[11] = 0xFFu;
        CHECK_EQ(cb_heartbeat_decode(b, 18, &o), 0);
        CHECK(o.pos_turns > 214748.0f);
        CHECK(o.pos_turns < 214748.5f);

        /* 4) 正满量程 2^31 会被固件钳到 INT32_MAX（见 can_heartbeat_codec.hpp
              的 kInt32RawLimit 说明）—— 这里按 INT32_MIN 解释，只要求可解不崩 */
        for (k = 0u; k < 18u; ++k) b[k] = 0u;
        b[8] = 0x80u; b[9] = 0x00u; b[10] = 0x00u; b[11] = 0x00u;
        CHECK_EQ(cb_heartbeat_decode(b, 18, &o), 0);

        /* 5) vel 走同一量程，同样不得在 21.4748 turns/s 处饱和 */
        for (k = 0u; k < 18u; ++k) b[k] = 0u;
        b[12] = 0x7Fu; b[13] = 0xFFu; b[14] = 0xFFu; b[15] = 0xFFu;
        CHECK_EQ(cb_heartbeat_decode(b, 18, &o), 0);
        CHECK(o.vel_turns_per_s > 214748.0f);

        /* 6) 负向对照：证明上面的判别式**真能**区分旧/新行为。
              若实现退化成"钳在 ±214748.0"，第 3 条会变成 == 214748.0 而红。 */
        for (k = 0u; k < 18u; ++k) b[k] = 0u;
        b[8] = 0x7Fu; b[9] = 0xFFu; b[10] = 0xFFu; b[11] = 0xFFu;
        CHECK_EQ(cb_heartbeat_decode(b, 18, &o), 0);
        CHECK(fabsf(o.pos_turns - 214748.0f) > 0.1f);
    }
}

/* ==========================================================================
 * 7. 名称与固件 autogen 对齐
 * ======================================================================== */

static void test_names(void)
{
    unsigned i;
    int tool_differs = 0;

    printf("[7] names vs firmware autogen (%u states, %u modes)\n",
           (unsigned)GV_STATE_NAME_COUNT, (unsigned)GV_CMODE_NAME_COUNT);

    for (i = 0; i < GV_STATE_NAME_COUNT; ++i) {
        uint8_t v = gv_state_names[i].value;
        const char *got = cb_heartbeat_state_name(v);
        if (strcmp(got, gv_state_names[i].name) != 0) {
            printf("  FAIL state %u: got \"%s\", expected \"%s\"\n",
                   v, got, gv_state_names[i].name);
            g_fail++;
        }
        g_checks++;
    }

    /* 值 5 在本固件里被跳过 —— 实现不得为它编造名字 */
    CHECK_EQ(GV_STATE_5_IS_SKIPPED, 1);
    for (i = 0; i < GV_STATE_NAME_COUNT; ++i) {
        CHECK(gv_state_names[i].value != 5u);
    }
    CHECK(strcmp(cb_heartbeat_state_name(5u), "reserved") == 0);
    /* 表内没有的最大 4 bit 状态是 15，应能表达 */
    CHECK(strcmp(cb_heartbeat_state_name(15u), "ENCODER_LINEARIZATION") == 0);

    for (i = 0; i < GV_CMODE_NAME_COUNT; ++i) {
        uint8_t v = gv_cmode_names[i].value;
        const char *got = cb_heartbeat_control_mode_name(v);
        if (strcmp(got, gv_cmode_names[i].name) != 0) {
            printf("  FAIL cmode %u: got \"%s\", expected \"%s\"\n",
                   v, got, gv_cmode_names[i].name);
            g_fail++;
        }
        g_checks++;
    }

    /* 参考工具的状态名表与本固件不一致 —— 断言这个差异客观存在，
       防止以后有人"照着工具改"而把正确的实现改坏。 */
    for (i = 0; i < GV_STATE_NAME_TOOL_COUNT; ++i) {
        uint8_t v = gv_state_names_tool_stale[i].value;
        const char *mine = cb_heartbeat_state_name(v);
        if (strcmp(mine, gv_state_names_tool_stale[i].name) != 0) tool_differs = 1;
    }
    CHECK_EQ(tool_differs, 1);
    printf("      tool name table is stale (5=sensorless-control, no 15) — "
           "firmware autogen is authoritative\n");
}

/* ==========================================================================
 * 8. 单参数读
 * ======================================================================== */

static void test_param_read(void)
{
    unsigned i;

    printf("[8] param read (%u)\n", (unsigned)GV_P_READ_COUNT);

    for (i = 0; i < GV_P_READ_COUNT; ++i) {
        const gv_p_read_t *v = &gv_p_read[i];
        cb_param_read_req_t rq;
        cb_param_read_rsp_t rs;
        uint8_t rqbuf[8];

        /* 请求：主站发出的 8 B 帧 */
        memset(rqbuf, 0, sizeof rqbuf);
        cb_be_put_u16(rqbuf + 1, v->ep_id);
        rqbuf[3] = v->req_len_raw;
        cb_be_put_u32(rqbuf + 4, v->offset);

        CHECK_EQ(cb_param_unpack_read_req(rqbuf, 8, (int)v->classic, &rq), 0);
        CHECK_EQ(rq.ep_id, v->ep_id);
        CHECK_EQ(rq.offset, v->offset);
        CHECK_EQ(rq.has_offset, 1);
        CHECK_EQ(rq.req_len, v->req_len_eff);   /* 归一化必须与固件一致 */

        /* 大端序字节序自检 */
        CHECK_EQ(rqbuf[1], (uint8_t)(v->ep_id >> 8));
        CHECK_EQ(rqbuf[2], (uint8_t)(v->ep_id & 0xFFu));

        /* 响应：把向量里的帧交给解析器 */
        CHECK_EQ(cb_param_unpack_read_rsp(v->rsp, v->rsp_len, &rs), 0);
        CHECK_EQ(rs.ep_id, v->ep_id);
        CHECK_EQ(rs.data_len, v->data_len);
        CHECK_BYTES(rs.value, v->value, v->data_len);
        /* 未使用尾部必须清零 */
        {
            unsigned k;
            for (k = v->data_len; k < CB_PARAM_MAX_VALUE; ++k) {
                CHECK_EQ(rs.value[k], 0x00u);
            }
        }

        /* More 位：DataLen 小于剩余时置位 */
        {
            unsigned remaining = v->full_len > v->offset ? v->full_len - v->offset : 0u;
            int more = (remaining > v->data_len) ? 1 : 0;
            CHECK_EQ(cb_param_rsp_has_more(&rs), more);
        }
    }

    /* 设备侧切片应能被主站解析回来 —— 用 8 字节值走完整的两段读 */
    {
        const uint8_t full[8] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
        uint8_t rsp[12];
        cb_param_read_rsp_t rs;
        size_t n;
        unsigned off = 0u;
        uint8_t joined[8];

        /* Classic 下每段最多 4 字节，读两次 */
        for (off = 0u; off < 8u; off += 4u) {
            n = cb_param_build_read_rsp(rsp, sizeof rsp, 0x00, 500, full, 8, 4, off);
            CHECK_EQ(n, 8u);
            CHECK_EQ(cb_param_unpack_read_rsp(rsp, n, &rs), 0);
            CHECK_EQ(rs.data_len, 4u);
            memcpy(&joined[off], rs.value, 4);
            if (off == 0u) CHECK_EQ(cb_param_rsp_has_more(&rs), 1);   /* 前半 */
            if (off == 4u) CHECK_EQ(cb_param_rsp_has_more(&rs), 0);   /* 后半 */
        }
        CHECK_BYTES(joined, full, 8);
    }

    /* offset 越界 → DataLen = 0 且无 More（合法，不是错误） */
    {
        uint8_t rsp[8];
        cb_param_read_rsp_t rs;
        size_t n = cb_param_build_read_rsp(rsp, sizeof rsp, 0x00, 1,
                                           (const uint8_t *)"\xAA", 1, 4, 1);
        CHECK_EQ(n, 4u);
        CHECK_EQ(cb_param_unpack_read_rsp(rsp, n, &rs), 0);
        CHECK_EQ(rs.data_len, 0u);
        CHECK_EQ(cb_param_rsp_has_more(&rs), 0);
    }

    /* 旧式 4 B 请求：无 offset，应被当作 offset = 0 */
    {
        uint8_t rq[4];
        cb_param_read_req_t rqo;
        rq[0] = 0u;
        cb_be_put_u16(rq + 1, 42);
        rq[3] = 4u;
        CHECK_EQ(cb_param_unpack_read_req(rq, 4, 1, &rqo), 0);
        CHECK_EQ(rqo.has_offset, 0);
        CHECK_EQ(rqo.offset, 0u);
        CHECK_EQ(rqo.ep_id, 42);
        /* ReqLen = 0 → 固件当作 4 */
        rq[3] = 0u;
        CHECK_EQ(cb_param_unpack_read_req(rq, 4, 1, &rqo), 0);
        CHECK_EQ(rqo.req_len, 4u);
    }

    /* 打包请求：带/不带 offset 的长度差异 */
    {
        uint8_t b[8];
        CHECK_EQ(cb_param_pack_read_req(b, 8, 100, 8, 4, 1), 8u);
        CHECK_EQ(cb_param_pack_read_req(b, 8, 100, 8, 4, 0), 4u);
        CHECK_EQ(cb_param_pack_read_req(b, 8, 100, 0, 4, 1), 0u);  /* ReqLen=0 拒绝 */
        CHECK_EQ(cb_param_pack_read_req(b, 8, 100, 9, 4, 1), 0u);  /* >8 拒绝 */
        CHECK_EQ(cb_param_pack_read_req(NULL, 8, 100, 4, 0, 1), 0u);
    }

    /* ReqLen 归一化（导出给主站预估用）与分块数 */
    {
        CHECK_EQ(cb_param_normalize_req_len(0, 0), 4u);    /* 0 → 4 */
        CHECK_EQ(cb_param_normalize_req_len(8, 0), 8u);
        CHECK_EQ(cb_param_normalize_req_len(9, 0), 8u);    /* >8 → 8 */
        CHECK_EQ(cb_param_normalize_req_len(8, 1), 4u);    /* Classic 夹到 4 */
        CHECK_EQ(cb_param_normalize_req_len(3, 1), 3u);

        /* 8 字节参数：FD 1 次，Classic 2 次 */
        CHECK_EQ(cb_param_read_chunks(8, 8, 0), 1u);
        CHECK_EQ(cb_param_read_chunks(8, 8, 1), 2u);
        /* 4 字节参数：两者都 1 次 */
        CHECK_EQ(cb_param_read_chunks(4, 8, 1), 1u);
        /* 5 字节参数：Classic 需 2 次 */
        CHECK_EQ(cb_param_read_chunks(5, 8, 1), 2u);
        /* 边界 */
        CHECK_EQ(cb_param_read_chunks(0, 8, 0), 0u);
        CHECK_EQ(cb_param_read_chunks(1, 1, 0), 1u);
        CHECK_EQ(cb_param_read_chunks(9, 4, 0), 3u);
    }
}

/* ==========================================================================
 * 9. 批量读
 * ======================================================================== */

static void test_param_batch(void)
{
    unsigned i;

    printf("[9] param batch read\n");

    /* 请求打包/解析往返 */
    {
        static const uint16_t eps[] = { 100, 282, 300 };
        uint8_t buf[64];
        uint16_t got[3];
        cb_param_batch_req_t rq;
        size_t n = cb_param_pack_batch_req(buf, sizeof buf, eps, 3);

        CHECK_EQ(n, 8u);
        CHECK_EQ(buf[0], CB_PARAM_FLAG_BATCH);
        CHECK_EQ(buf[1], 3u);
        CHECK_EQ(cb_param_unpack_batch_req(buf, n, got, 3, &rq), 0);
        CHECK_EQ(rq.count, 3u);
        CHECK_EQ(got[0], 100u);
        CHECK_EQ(got[1], 282u);
        CHECK_EQ(got[2], 300u);

        /* 帧内条目不足 → 按实际可用数处理（复刻固件行为） */
        CHECK_EQ(cb_param_unpack_batch_req(buf, 6, got, 3, &rq), 0);
        CHECK_EQ(rq.count, 2u);

        /* 非批量标志 → 拒绝 */
        buf[0] = 0x00u;
        CHECK_EQ(cb_param_unpack_batch_req(buf, 8, got, 3, &rq), -1);

        /* 上限：N = 0 / > 31 拒绝 */
        CHECK_EQ(cb_param_pack_batch_req(buf, sizeof buf, eps, 0), 0u);
        CHECK_EQ(cb_param_pack_batch_req(buf, sizeof buf, eps, 32), 0u);
    }

    /* 批量响应解析 */
    for (i = 0; i < GV_P_BATCH_COUNT; ++i) {
        const gv_p_batch_t *v = &gv_p_batch[i];
        cb_param_batch_rsp_t o;

        CHECK_EQ(cb_param_unpack_batch_rsp(v->bytes, v->bytes_len, v->n_req, &o), 0);
        CHECK_EQ(o.is_err, v->is_err);
        if (v->is_err) {
            CHECK_EQ(o.count, 0u);
            CHECK(o.bitmap == NULL);
            CHECK(o.values == NULL);
            CHECK_EQ(v->bytes_len, 2u);
            /* ERR 帧必须恰好 2 字节 */
            CHECK_EQ(v->bytes[1], 0u);
        } else {
            CHECK_EQ(o.count, v->count);
            CHECK_EQ(o.bitmap_bytes, 1u);
            CHECK_EQ(o.bitmap[0], v->bitmap);
            /* 位图：N=3 → 0b111 */
            CHECK(cb_param_bitmap_test(o.bitmap, o.bitmap_bytes, 0));
            CHECK(cb_param_bitmap_test(o.bitmap, o.bitmap_bytes, 1));
            CHECK(cb_param_bitmap_test(o.bitmap, o.bitmap_bytes, 2));
            CHECK(!cb_param_bitmap_test(o.bitmap, o.bitmap_bytes, 3));
            /* 值流：4 + 2 + 8 = 14 字节 */
            CHECK_EQ(o.values_len, 14u);
            CHECK_EQ(o.values[0], 1u);
            CHECK_EQ(o.values[3], 4u);
            CHECK_EQ(o.values[4], 5u);
            CHECK_EQ(o.values[5], 6u);
            CHECK_EQ(o.values[6], 0u);
            CHECK_EQ(o.values[13], 7u);
        }
    }

    /* Count 与请求不符 → 拒绝 */
    {
        cb_param_batch_rsp_t o;
        CHECK_EQ(cb_param_unpack_batch_rsp(gv_p_batch[0].bytes, gv_p_batch[0].bytes_len,
                                           5u, &o), -1);
    }

    /* 位图字节数 */
    CHECK_EQ(cb_param_bitmap_bytes(1), 1u);
    CHECK_EQ(cb_param_bitmap_bytes(8), 1u);
    CHECK_EQ(cb_param_bitmap_bytes(9), 2u);
    CHECK_EQ(cb_param_bitmap_bytes(31), 4u);

    /* ERR 响应编码 */
    {
        uint8_t b[8];
        CHECK_EQ(cb_param_pack_batch_err(b, sizeof b), 2u);
        CHECK_EQ(b[0], 0x60u);
        CHECK_EQ(b[1], 0x00u);
    }

    /* 装箱器：8 个 8 字节值装不进单帧（2+1+64 > 64） */
    {
        cb_param_batch_item_t items[8];
        size_t counts[8];
        size_t offs[8];
        size_t n;
        unsigned k;
        size_t total = 0u;

        for (k = 0; k < 8u; ++k) { items[k].ep_id = (uint16_t)(100u + k); items[k].value_len = 8u; }
        n = cb_param_plan_batches(items, 8u, 64u, counts, offs, 8u);
        CHECK(n >= 2u);
        for (k = 0; k < n; ++k) {
            /* 每批必须满足 2 + ⌈n/8⌉ + Σlen ≤ 64 */
            size_t bm = (counts[k] + 7u) / 8u;
            CHECK(2u + bm + 8u * counts[k] <= 64u);
            total += counts[k];
        }
        CHECK_EQ(total, 8u);
        CHECK_EQ(offs[0], 0u);
        CHECK_EQ(offs[1], counts[0]);

        /* 单条就装不下 → 返回 0 */
        items[0].value_len = 0u;
        CHECK_EQ(cb_param_plan_batches(items, 0u, 64u, counts, offs, 8u), 0u);
    }

    /* 装箱器：小值可一次装完 3 条 */
    {
        cb_param_batch_item_t items[3] = {
            { 1, 4 }, { 2, 4 }, { 3, 4 }
        };
        size_t counts[4], offs[4];
        size_t n = cb_param_plan_batches(items, 3u, 64u, counts, offs, 4u);
        CHECK_EQ(n, 1u);
        CHECK_EQ(counts[0], 3u);
    }
}

/* ==========================================================================
 * 10. 参数写
 * ======================================================================== */

static void test_param_write(void)
{
    printf("[10] param write / segmented\n");

    /* 写请求往返 */
    {
        const uint8_t val[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
        uint8_t buf[12];
        uint8_t got[8];
        uint8_t fl = 0u, len = 0u;
        uint16_t ep = 0u, ep2 = 0u;
        size_t n = cb_param_pack_write_req(buf, sizeof buf, 777, val, 4);

        CHECK_EQ(n, 8u);
        CHECK_EQ(buf[0], 0x00u);
        CHECK_EQ(buf[1], 0x03u);           /* 777 = 0x0309 */
        CHECK_EQ(buf[2], 0x09u);
        CHECK_EQ(buf[3], 4u);
        CHECK_BYTES(&buf[4], val, 4);

        CHECK_EQ(cb_param_unpack_write_req(buf, n, &fl, &ep, got, &len), 0);
        CHECK_EQ(fl, 0x00u);
        CHECK_EQ(ep, 777u);
        CHECK_EQ(len, 4u);
        CHECK_BYTES(got, val, 4);

        /* 8 字节值（uint64） */
        n = cb_param_pack_write_req(buf, sizeof buf, 1, (const uint8_t *)"ABCDEFGH", 8u);
        CHECK_EQ(n, 12u);
        CHECK_EQ(cb_param_unpack_write_req(buf, n, &fl, &ep2, got, &len), 0);
        CHECK_EQ(len, 8u);
        CHECK(memcmp(got, "ABCDEFGH", 8) == 0);

        /*
         * 长度 0 是**合法**的：调用 function 端点（Fibre 方法）就是这个形状 ——
         * 无值，写即执行（入参在各自的 input 子端点上）。帧长仍补到 8 字节，
         * 否则固件 `cmd_param_write()` 的 `if (msg.len < 8) return;` 会整帧丢掉。
         */
        n = cb_param_pack_write_req(buf, sizeof buf, 1, val, 0u);
        CHECK_EQ(n, 8u);                       /* 补到最小帧长 */
        CHECK_EQ(cb_param_unpack_write_req(buf, n, &fl, &ep2, got, &len), 0);
        CHECK_EQ(len, 0u);                     /* 真实长度是 0 */
        CHECK_EQ(fl, 0u);

        /* 值可以为 NULL（function 调用不携带载荷） */
        CHECK_EQ(cb_param_pack_write_req(buf, sizeof buf, 1, NULL, 0u), 8u);

        /* 非法：> 8；或长度非 0 却没给值 */
        CHECK_EQ(cb_param_pack_write_req(buf, sizeof buf, 1, val, 9u), 0u);
        CHECK_EQ(cb_param_pack_write_req(buf, sizeof buf, 1, NULL, 1u), 0u);
    }

    /* 写确认 */
    {
        unsigned i;
        for (i = 0; i < GV_P_ACK_COUNT; ++i) {
            const gv_p_ack_t *v = &gv_p_ack[i];
            uint8_t got[8];
            uint8_t fl = 0u;
            uint16_t ep = 0u;

            CHECK_EQ(cb_param_pack_write_ack(got, sizeof got, v->flags, v->ep_id), 8u);
            CHECK_BYTES(got, v->bytes, 8);

            CHECK_EQ(cb_param_unpack_write_ack(v->bytes, 8, &fl, &ep), 0);
            CHECK_EQ(fl, v->flags);
            CHECK_EQ(ep, v->ep_id);
        }

        /* DataLen 非 0 的帧不是合法确认 */
        CHECK_EQ(cb_param_unpack_write_ack(gv_p_ack[0].bytes, 7, NULL, NULL), -1);
        {
            uint8_t bad[8];
            memcpy(bad, gv_p_ack[0].bytes, 8);
            bad[3] = 4u;
            CHECK_EQ(cb_param_unpack_write_ack(bad, 8, NULL, NULL), -1);
        }
    }

    /* 分段写块：逐字节与向量一致 */
    {
        unsigned i;
        for (i = 0; i < GV_P_SEG_COUNT; ++i) {
            const gv_p_seg_t *v = &gv_p_seg[i];
            uint8_t got[8];
            size_t n = cb_param_pack_write_chunk(got, sizeof got, v->ep_id,
                                                 v->total_len, v->offset,
                                                 v->value, v->value_len,
                                                 (int)v->expect_more);
            CHECK_EQ(n, 8u);
            CHECK_BYTES(got, v->bytes, 8);
        }
    }

    /* 分段写装配器：走通 8 字节参数 */
    {
        cb_param_write_asm_t a;
        uint8_t out[8], len = 0u;
        int rc;

        cb_param_write_asm_init(&a);
        rc = cb_param_write_asm_feed(&a, 1u, 400u, 8u, CB_PARAM_FLAG_MORE,
                                     (const uint8_t *)"\x11\x22\x33\x44", out, &len);
        CHECK_EQ(rc, 1);                   /* 还要后续块 */
        CHECK_EQ(a.offset, 4u);

        rc = cb_param_write_asm_feed(&a, 1u, 400u, 8u, 0x00u,
                                     (const uint8_t *)"\x55\x66\x77\x88", out, &len);
        CHECK_EQ(rc, 0);                   /* 完成 */
        CHECK_EQ(len, 8u);
        CHECK_BYTES(out, "\x11\x22\x33\x44\x55\x66\x77\x88", 8);
        CHECK_EQ(a.active, 0);             /* 完成后必须复位 */
    }

    /* 6 字节参数：末块 2 字节有效 + 2 字节填充 */
    {
        cb_param_write_asm_t a;
        uint8_t out[8], len = 0u;
        cb_param_write_asm_init(&a);
        CHECK_EQ(cb_param_write_asm_feed(&a, 1u, 401u, 6u, 0x80u,
                                         (const uint8_t *)"\xAA\xBB\xCC\xDD",
                                         out, &len), 1);
        CHECK_EQ(cb_param_write_asm_feed(&a, 1u, 401u, 6u, 0x00u,
                                         (const uint8_t *)"\xEE\xFF\x00\x00",
                                         out, &len), 0);
        CHECK_EQ(len, 6u);
        CHECK_BYTES(out, "\xAA\xBB\xCC\xDD\xEE\xFF", 6);
    }

    /* 中止条件 ①：TotalLen 不在 5..8 */
    {
        cb_param_write_asm_t a;
        uint8_t out[8], len = 0u;
        cb_param_write_asm_init(&a);
        CHECK_EQ(cb_param_write_asm_feed(&a, 1u, 1u, 4u, 0x00u,
                                         (const uint8_t *)"\x01\x02\x03\x04",
                                         out, &len), -1);
        CHECK_EQ(a.active, 0);
        cb_param_write_asm_init(&a);
        CHECK_EQ(cb_param_write_asm_feed(&a, 1u, 1u, 9u, 0x00u,
                                         (const uint8_t *)"\x01\x02\x03\x04",
                                         out, &len), -1);
        CHECK_EQ(a.active, 0);
    }

    /* 中止条件 ③：缓冲已填满却仍声明 More */
    {
        cb_param_write_asm_t a;
        uint8_t out[8], len = 0u;
        cb_param_write_asm_init(&a);
        CHECK_EQ(cb_param_write_asm_feed(&a, 1u, 402u, 5u, 0x80u,
                                         (const uint8_t *)"\x01\x02\x03\x04",
                                         out, &len), 1);
        /* 第二块只剩 1 字节，但声明 More → 已填满却说还有 → 中止 */
        CHECK_EQ(cb_param_write_asm_feed(&a, 1u, 402u, 5u, 0x80u,
                                         (const uint8_t *)"\x05\x00\x00\x00",
                                         out, &len), -1);
        CHECK_EQ(a.active, 0);
    }

    /* 中止条件 ②：重复末块（offset 已越界） */
    {
        cb_param_write_asm_t a;
        uint8_t out[8], len = 0u;
        cb_param_write_asm_init(&a);
        CHECK_EQ(cb_param_write_asm_feed(&a, 1u, 403u, 5u, 0x80u,
                                         (const uint8_t *)"\x01\x02\x03\x04",
                                         out, &len), 1);
        CHECK_EQ(cb_param_write_asm_feed(&a, 1u, 403u, 5u, 0x00u,
                                         (const uint8_t *)"\x05\x00\x00\x00",
                                         out, &len), 0);
        /* 再喂一块 → 设备已完成并复位装配器，于是把它当作**新装配的首块**。
           ⚠ 这里暴露了固件的一个健壮性缺口：单块声明 TotalLen=5 却只带 4 字节，
           固件会直接写入一个第 5 字节为 0 的“完整值”。SDK 的装配器如实复刻
           该行为（L2 必须忠于协议），但主站侧永远按 4 字节分块发送。
           要检测这种畸形主站，应在虚拟设备层另加校验。 */
        CHECK_EQ(cb_param_write_asm_feed(&a, 1u, 403u, 5u, 0x00u,
                                         (const uint8_t *)"\x99\x99\x99\x99",
                                         out, &len), 0);
        CHECK_EQ(len, 5u);
        CHECK_EQ(out[4], 0x00u);            /* 第 5 字节是复位时的 0，不是 0x99 */
        CHECK_EQ(a.active, 0);
    }

    /* 条件 ④：中途换端点 → 重新开始装配，旧数据必须被丢弃 */
    {
        cb_param_write_asm_t a;
        uint8_t out[8], len = 0u;
        cb_param_write_asm_init(&a);
        CHECK_EQ(cb_param_write_asm_feed(&a, 1u, 500u, 8u, 0x80u,
                                         (const uint8_t *)"\x11\x22\x33\x44",
                                         out, &len), 1);
        /* 换端点 → 重置，新首块 */
        CHECK_EQ(cb_param_write_asm_feed(&a, 1u, 501u, 8u, 0x80u,
                                         (const uint8_t *)"\xAA\xBB\xCC\xDD",
                                         out, &len), 1);
        CHECK_EQ(a.ep_id, 501u);
        CHECK_EQ(a.offset, 4u);
        CHECK_EQ(cb_param_write_asm_feed(&a, 1u, 501u, 8u, 0x00u,
                                         (const uint8_t *)"\xEE\xFF\x00\x01",
                                         out, &len), 0);
        /* 必须只含第二个端点的数据，绝不能混入第一次的半截值 */
        CHECK_BYTES(out, "\xAA\xBB\xCC\xDD\xEE\xFF\x00\x01", 8);
    }

    /* 条件 ④b：中途换主站 → 同样重置 */
    {
        cb_param_write_asm_t a;
        uint8_t out[8], len = 0u;
        cb_param_write_asm_init(&a);
        CHECK_EQ(cb_param_write_asm_feed(&a, 1u, 600u, 8u, 0x80u,
                                         (const uint8_t *)"\x11\x22\x33\x44",
                                         out, &len), 1);
        CHECK_EQ(cb_param_write_asm_feed(&a, 2u, 600u, 8u, 0x80u,
                                         (const uint8_t *)"\x55\x66\x77\x88",
                                         out, &len), 1);
        CHECK_EQ(a.master_id, 2u);
        CHECK_EQ(a.offset, 4u);
    }

    /* 分段块打包的参数校验
       ⚠ 加固后还有两条**语义**约束（防止生成固件无法正确装配的块）：
          · 非末块（More=1）必须满 4 字节，且不能已经填满
          · 末块（More=0）必须刚好把值补齐（offset + len == TotalLen）
       理由：固件对末块不做完整性校验，会写入尾部补 0 的“完整值”（F13）。 */
    {
        uint8_t b[8];
        /* 合法：末块刚好补齐 */
        CHECK_EQ(cb_param_pack_write_chunk(b, 8, 1, 8, 4,
                                           (const uint8_t *)"ABCD", 4, 0), 8u);
        CHECK_EQ(cb_param_pack_write_chunk(b, 8, 1, 5, 4,
                                           (const uint8_t *)"A", 1, 0), 8u);
        /* 合法：非末块满 4 字节且未填满 */
        CHECK_EQ(cb_param_pack_write_chunk(b, 8, 1, 8, 0,
                                           (const uint8_t *)"ABCD", 4, 1), 8u);
        CHECK_EQ(cb_param_pack_write_chunk(b, 8, 1, 5, 0,
                                           (const uint8_t *)"ABCD", 4, 1), 8u);

        /* 末块不完整 → 拒绝（否则会触发 F13 的静默补 0） */
        CHECK_EQ(cb_param_pack_write_chunk(b, 8, 1, 8, 4,
                                           (const uint8_t *)"AB", 2, 0), 0u);
        /* 非末块只给半块 → 拒绝（中间的洞会被静默补 0） */
        CHECK_EQ(cb_param_pack_write_chunk(b, 8, 1, 8, 0,
                                           (const uint8_t *)"AB", 2, 1), 0u);
        /* 非末块声明 More 却已填满 → 拒绝 */
        CHECK_EQ(cb_param_pack_write_chunk(b, 8, 1, 8, 4,
                                           (const uint8_t *)"ABCD", 4, 1), 0u);

        /* 其他参数校验 */
        CHECK_EQ(cb_param_pack_write_chunk(b, 8, 1, 4, 0,
                                           (const uint8_t *)"AB", 2, 0), 0u);   /* TotalLen < 5 */
        CHECK_EQ(cb_param_pack_write_chunk(b, 8, 1, 9, 0,
                                           (const uint8_t *)"AB", 2, 0), 0u);   /* TotalLen > 8 */
        CHECK_EQ(cb_param_pack_write_chunk(b, 8, 1, 8, 2,
                                           (const uint8_t *)"AB", 2, 0), 0u);   /* offset 非 4 对齐 */
        CHECK_EQ(cb_param_pack_write_chunk(b, 8, 1, 8, 8,
                                           (const uint8_t *)"AB", 2, 0), 0u);   /* offset 越界 */
        CHECK_EQ(cb_param_pack_write_chunk(b, 4, 1, 8, 0,
                                           (const uint8_t *)"ABCD", 4, 1), 0u); /* cap 不足 */
    }
}

/* ==========================================================================
 * main
 * ======================================================================== */

int main(void)
{
    printf("=== proto tests (WP2 step 2) ===\n\n");
    test_ctrl_pos();      printf("\n");
    test_ctrl_others();   printf("\n");
    test_ctrl_meta();     printf("\n");
    test_query();         printf("\n");
    test_heartbeat();     printf("\n");
    test_names();         printf("\n");
    test_param_read();    printf("\n");
    test_param_batch();   printf("\n");
    test_param_write();

    printf("\n=== %d checks, %d failures ===\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
