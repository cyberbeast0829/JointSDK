/**
 * @file    jsdk_fault.c
 * @brief   WP4：故障码映射、链路健康、反馈新鲜度（诊断侧）
 *
 * 三套错误编码的层级关系（详见 joint_sdk.h §13.1）：
 *
 * @verbatim
 *   固件内部：  32-bit 子系统位图（motor/encoder/controller/… 各自一份）
 *                   │  固件 detect_error_code() 归约
 *                   ▼
 *   线上摘要：  4-bit ErrorCode（MIT 响应的 err 半字节）
 *
 *   另有一条独立的快速通道：心跳 5-bit 子系统位图（只表示"哪个子系统有错"）
 * @endverbatim
 *
 * 因此 `err_code != 0` 与 `hb_error != 0` 是**两个不同粒度的信号**，
 * 而 `axis_error` 的第 20 位（CAN_BUS_FAILED）专指协议级超时。
 */

#include "jsdk_core_internal.h"

#include <string.h>

/* ==========================================================================
 * 32-bit 子系统错误位（固件 Firmware/autogen/interfaces.hpp 的 Axis::Error）
 *
 * ⚠ 位号不连续：固件只定义了这些取值。未定义的位返回 NULL，
 *   而不是编一个名字出来（"不猜"原则）。
 * ======================================================================== */

typedef struct {
    unsigned    bit;
    const char *name;
} bit_name_t;

static const bit_name_t k_axis_err[] = {
    {  0u, "INVALID_STATE" },
    {  6u, "MOTOR_FAILED" },
    {  7u, "SENSORLESS_ESTIMATOR_FAILED" },
    {  8u, "ENCODER_FAILED" },
    {  9u, "CONTROLLER_FAILED" },
    { 11u, "WATCHDOG_TIMER_EXPIRED" },
    { 12u, "MIN_ENDSTOP_PRESSED" },
    { 13u, "MAX_ENDSTOP_PRESSED" },
    { 14u, "ESTOP_REQUESTED" },
    { 17u, "HOMING_WITHOUT_ENDSTOP" },
    { 18u, "OVER_TEMP" },
    { 19u, "UNKNOWN_POSITION" },
    { 20u, "CAN_BUS_FAILED" },
};

static const char *const k_hb_err[5] = {
    "axis",        /* CB_HB_ERR_AXIS       0x01 */
    "motor",       /* CB_HB_ERR_MOTOR      0x02 */
    "encoder",     /* CB_HB_ERR_ENCODER    0x04 */
    "controller",  /* CB_HB_ERR_CONTROLLER 0x08 */
    "board"        /* CB_HB_ERR_BOARD      0x10 */
};

/** 固件 `AxisState` 0..16（**注意** 5 是空缺，16 超出心跳的 4 bit）。 */
static const char *const k_can_axis_state[17] = {
    "UNDEFINED",                    /*  0 */
    "IDLE",                         /*  1 */
    "STARTUP_SEQUENCE",             /*  2 */
    "FULL_CALIBRATION_SEQUENCE",    /*  3 */
    "MOTOR_CALIBRATION",            /*  4 */
    "reserved",                     /*  5 —— 固件未定义该值 */
    "ENCODER_INDEX_SEARCH",         /*  6 */
    "ENCODER_OFFSET_CALIBRATION",   /*  7 */
    "CLOSED_LOOP_CONTROL",          /*  8 */
    "LOCKIN_SPIN",                  /*  9 */
    "ENCODER_DIR_FIND",             /* 10 */
    "HOMING",                       /* 11 */
    "ENCODER_HALL_POLARITY_CALIBRATION", /* 12 */
    "ENCODER_HALL_PHASE_CALIBRATION",    /* 13 */
    "INERTIA_CALIBRATION",          /* 14 */
    "ENCODER_LINEARIZATION",        /* 15 */
    "MOTOR_DEADTIME_CALIBRATION"    /* 16 */
};

/* ==========================================================================
 * 名称查询
 * ======================================================================== */

const char *jsdk_joint_error_string(uint8_t mit_err_code)
{
    /* 复用协议层表，保证与 MIT 响应解码永远一致 */
    return cb_mit_error_name(mit_err_code);
}

const char *jsdk_hb_error_bit_name(unsigned bit)
{
    if (bit >= (unsigned)(sizeof k_hb_err / sizeof k_hb_err[0])) return NULL;
    return k_hb_err[bit];
}

const char *jsdk_axis_error_bit_name(unsigned bit)
{
    unsigned i;

    for (i = 0u; i < (unsigned)(sizeof k_axis_err / sizeof k_axis_err[0]); ++i) {
        if (k_axis_err[i].bit == bit) return k_axis_err[i].name;
    }
    return NULL;   /* 固件未定义的位：不编名字 */
}

const char *jsdk_axis_error_first(uint32_t value, unsigned *bit_out)
{
    unsigned i;

    /* 按**固件位号从小到大**找第一个置位，保证同一 value 永远给同一答案 */
    for (i = 0u; i < (unsigned)(sizeof k_axis_err / sizeof k_axis_err[0]); ++i) {
        if (value & (1u << k_axis_err[i].bit)) {
            if (bit_out) *bit_out = k_axis_err[i].bit;
            return k_axis_err[i].name;
        }
    }
    if (bit_out) *bit_out = 0u;
    return NULL;
}

const char *jsdk_can_axis_state_name(uint8_t can_axis_state)
{
    if (can_axis_state >= (uint8_t)(sizeof k_can_axis_state / sizeof k_can_axis_state[0])) {
        return "unknown";
    }
    return k_can_axis_state[can_axis_state];
}

/* ==========================================================================
 * 组合描述
 * ======================================================================== */

/** 在 @p n 之后追加，返回新的长度（含长度上限保护）。 */
static size_t app(char *buf, size_t cap, size_t n, const char *s)
{
    size_t l = strlen(s);
    size_t i;

    if (n >= cap) return n;
    for (i = 0u; i < l && n + 1u < cap; ++i) buf[n++] = s[i];
    buf[n] = '\0';
    return n;
}

static size_t app_u32_hex(char *buf, size_t cap, size_t n, uint32_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    int shift;

    if (n + 1u >= cap) return n;
    buf[n++] = '0'; buf[n++] = 'x'; buf[n] = '\0';
    for (shift = 28; shift >= 0; shift -= 4) {
        if (n + 1u < cap) buf[n++] = hex[(v >> (unsigned)shift) & 0xFu];
    }
    buf[n] = '\0';
    return n;
}

int jsdk_joint_describe_fault(const jsdk_joint_t *j, char *buf, size_t cap)
{
    size_t n = 0u;
    unsigned b;

    if (!j || !buf || cap == 0u) return 0;
    buf[0] = '\0';

    if (!jsdk_joint_check(j)) {
        return (int)app(buf, cap, n, "invalid joint");
    }

    /* 32-bit 明细（来自 0x45；未查询时为 0） */
    if (j->fault.valid) {
        uint32_t v = j->fault.motor_error | j->fault.encoder_error
                   | j->fault.sensorless_error | j->fault.controller_error
                   | j->fault.system_error | j->fault.axis_error;
        n = app(buf, cap, n, "detail_err=");
        n = app_u32_hex(buf, cap, n, v);
        if (v != 0u) {
            const char *nm = jsdk_axis_error_first(j->fault.axis_error, &b);
            if (nm) { n = app(buf, cap, n, "("); n = app(buf, cap, n, nm);
                      n = app(buf, cap, n, ")"); }
        }
        n = app(buf, cap, n, " ");
    }

    n = app(buf, cap, n, "axis_error=");
    n = app_u32_hex(buf, cap, n, j->fb.axis_error);
    {
        const char *nm = jsdk_axis_error_first(j->fb.axis_error, &b);
        if (nm) { n = app(buf, cap, n, "("); n = app(buf, cap, n, nm);
                  n = app(buf, cap, n, ")"); }
    }

    n = app(buf, cap, n, " mit_err=");
    n = app(buf, cap, n, jsdk_joint_error_string(j->fb.err_code));

    n = app(buf, cap, n, " hb=");
    if (j->fb.hb_error == 0u) {
        n = app(buf, cap, n, "none");
    } else {
        unsigned i;
        int first = 1;
        for (i = 0u; i < 5u; ++i) {
            if (j->fb.hb_error & (uint8_t)(1u << i)) {
                if (!first) n = app(buf, cap, n, "|");
                n = app(buf, cap, n, jsdk_hb_error_bit_name(i));
                first = 0;
            }
        }
    }

    n = app(buf, cap, n, " can_state=");
    n = app(buf, cap, n, jsdk_can_axis_state_name(j->current_state_raw));

    return (int)n;
}

/* ==========================================================================
 * WP4：反馈新鲜度
 * ======================================================================== */

/**
 * 反馈超时阈值（ms）。
 *
 * 三个来源取最保守的一个（**越大越保守**，因为误报 stale 会让客户以为设备挂了）：
 *   1. 心跳周期 × 3（心跳是唯一无请求的周期反馈）
 *   2. 控制周期 × 3
 *   3. 硬下限 50 ms
 *
 * @note 该阈值只能由**观测**推导，不引入新配置项 —— 客户不需要为它调参。
 */
uint32_t jsdk_joint_stale_ms(const jsdk_joint_t *j)
{
    uint32_t ms = 50u;

    if (!jsdk_joint_check(j)) return ms;

    if (j->heartbeat_rate_ms != 0u) {
        uint32_t v = j->heartbeat_rate_ms * 3u;
        if (v > ms) ms = v;
    }
    if (j->ctx->cfg.period_ns != 0u) {
        uint32_t v = (uint32_t)(j->ctx->cfg.period_ns / 1000000u) * 6u;
        if (v > ms) ms = v;
    }
    return ms;
}

/**
 * 每个周期调用：刷新 `age_ms`，并在超时后置 `JSDK_JF_FEEDBACK_STALE`。
 *
 * ⚠ 置位是**粘滞**的（与其余 `JSDK_JF_*` 一致）：恢复后要客户显式
 *   `jsdk_joint_clear_status_flags()`，否则现场会看不到"曾经掉过反馈"。
 */
void jsdk_joint__refresh_freshness(jsdk_joint_t *j)
{
    uint32_t age;

    if (!jsdk_joint_check(j)) return;
    if (!j->fb.online) return;          /* 从没收到过 → 不算 stale，是 offline */

    age = jsdk_elapsed(j->ctx->now_ms, j->last_fb_ms);
    j->fb.age_ms = age;

    if (age > jsdk_joint_stale_ms(j)) {
        jsdk_joint_set_flags(j, (uint16_t)JSDK_JF_FEEDBACK_STALE);
    }
}
