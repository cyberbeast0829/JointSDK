/**
 * @file    jsdk_text.c
 * @brief   文本与 ABI 标识（纯 rodata，零依赖，可在任何上下文调用）
 *
 * 全部函数都接受**任意**输入：未知枚举返回 "unknown(...)" 而不是 NULL，
 * 这样日志/CLI 不必到处判空。
 */

#include "jsdk_core_internal.h"

/* ==========================================================================
 * ABI 守卫
 * ======================================================================== */

const char *jsdk_backend_name(void)
{
    return JSDK_BACKEND_NAME_CAN;
}

uint32_t jsdk_abi_version(void)
{
    return JSDK_ABI_VERSION_CAN;
}

/* --------------------------------------------------------------------------
 * 结构体尺寸/对齐表（FFI 自检）
 *
 * 为什么需要：Python(ctypes)/C#/Rust 等 FFI 绑定必须**复刻**这些结构体的
 * 字段顺序与对齐。复刻错了不会报错，而是**直接踩内存** —— 症状是"值偶尔不对"
 * 或随机崩溃，现场几乎查不出来。
 *
 * 有了这张表，绑定方可以在导入时逐项比对，不一致就立刻抛出
 * "ABI 不匹配"（并指出是哪个类型），而不是等到跑起来才出事。
 *
 * 表**只增不改**：改字段顺序必须同时改这里，而这会让所有绑定在导入时立刻失败 ——
 * 这正是我们想要的信号（ABI 变了）。
 * ------------------------------------------------------------------------ */

/** 用"结构体里紧跟在 char 之后的成员偏移"求对齐（C99 可移植做法）。
 *
 * ⚠ MSVC 对**宏里**的匿名结构体会报 C4116（“括号中的未命名类型定义”）。
 *   这是 C11 允许的写法（`offsetof` 的第一个参数可以是类型定义），而且它
 *   只在本文件里用 —— 所以就地抑制，不为了它改动公共写法。
 *   （gcc/clang 对这个写法一言不发，所以以前没暴露过。） */
#if defined(_MSC_VER)
#  pragma warning(push)
#  pragma warning(disable: 4116)
#endif
#define JSDK_ALIGNOF(T) ((uint32_t)offsetof(struct { char c; T t; }, t))

static const jsdk_abi_type_t k_abi_types[] = {
    { "jsdk_can_frame_t",     (uint32_t)sizeof(jsdk_can_frame_t),     JSDK_ALIGNOF(jsdk_can_frame_t) },
    { "jsdk_can_hal_t",       (uint32_t)sizeof(jsdk_can_hal_t),       JSDK_ALIGNOF(jsdk_can_hal_t) },
    { "jsdk_context_config_t",(uint32_t)sizeof(jsdk_context_config_t),JSDK_ALIGNOF(jsdk_context_config_t) },
    { "jsdk_desc_config_t",   (uint32_t)sizeof(jsdk_desc_config_t),   JSDK_ALIGNOF(jsdk_desc_config_t) },
    { "jsdk_joint_config_t",  (uint32_t)sizeof(jsdk_joint_config_t),  JSDK_ALIGNOF(jsdk_joint_config_t) },
    { "jsdk_joint_config_snapshot_t",
      (uint32_t)sizeof(jsdk_joint_config_snapshot_t), JSDK_ALIGNOF(jsdk_joint_config_snapshot_t) },
    { "jsdk_joint_feedback_t",(uint32_t)sizeof(jsdk_joint_feedback_t),JSDK_ALIGNOF(jsdk_joint_feedback_t) },
    { "jsdk_bus_state_t",     (uint32_t)sizeof(jsdk_bus_state_t),     JSDK_ALIGNOF(jsdk_bus_state_t) },
    { "jsdk_device_info_t",   (uint32_t)sizeof(jsdk_device_info_t),   JSDK_ALIGNOF(jsdk_device_info_t) },
    { "jsdk_fault_info_t",    (uint32_t)sizeof(jsdk_fault_info_t),    JSDK_ALIGNOF(jsdk_fault_info_t) },
    { "jsdk_value_t",         (uint32_t)sizeof(jsdk_value_t),         JSDK_ALIGNOF(jsdk_value_t) },
    { "jsdk_unit_scale_t",    (uint32_t)sizeof(jsdk_unit_scale_t),    JSDK_ALIGNOF(jsdk_unit_scale_t) },
    { "jsdk_param_req_t",     (uint32_t)sizeof(jsdk_param_req_t),     JSDK_ALIGNOF(jsdk_param_req_t) },
    { "jsdk_group_target_t",  (uint32_t)sizeof(jsdk_group_target_t),  JSDK_ALIGNOF(jsdk_group_target_t) },
    { "jsdk_desc_info_t",     (uint32_t)sizeof(jsdk_desc_info_t),     JSDK_ALIGNOF(jsdk_desc_info_t) }
};

#if defined(_MSC_VER)
#  pragma warning(pop)
#endif

const jsdk_abi_type_t *jsdk_abi_types(size_t *count_out)
{
    if (count_out) *count_out = sizeof k_abi_types / sizeof k_abi_types[0];
    return k_abi_types;
}

/* ==========================================================================
 * 状态码
 * ======================================================================== */

const char *jsdk_status_string(jsdk_status_t status)
{
    switch (status) {
    case JSDK_OK:              return "ok";
    case JSDK_ERR_INVALID_ARG: return "invalid-argument";
    case JSDK_ERR_NO_MEMORY:   return "no-memory";
    case JSDK_ERR_NOT_FOUND:   return "not-found";
    case JSDK_ERR_BAD_STATE:   return "bad-state";
    case JSDK_ERR_TRANSPORT:   return "transport-error";
    case JSDK_ERR_UNSUPPORTED: return "unsupported";
    case JSDK_ERR_TIMEOUT:     return "timeout";
    case JSDK_ERR_PROTOCOL:    return "protocol-error";
    case JSDK_ERR_BUSY:        return "busy";
    case JSDK_ERR_PARSE:       return "parse-error";
    default:                   return "unknown-status";
    }
}

/* ==========================================================================
 * 关节状态
 * ------------------------------------------------------------------------
 * CAN 侧只有 IDLE / CLOSED_LOOP 等少数状态；枚举名沿用 EtherCAT 家族。
 * 名称里附上 CAN 的真实含义，避免现场按 EtherCAT 的经验误读。
 * ======================================================================== */

const char *jsdk_axis_state_string(jsdk_axis_state_t state)
{
    switch (state) {
    case JSDK_AXIS_UNKNOWN:              return "unknown";
    case JSDK_AXIS_SWITCH_ON_DISABLED:   return "switch-on-disabled(idle)";
    case JSDK_AXIS_READY_TO_SWITCH_ON:   return "ready-to-switch-on(calibrating)";
    case JSDK_AXIS_SWITCHED_ON:          return "switched-on(closed-loop,no-cmd)";
    case JSDK_AXIS_OPERATION_ENABLED:    return "operation-enabled";
    case JSDK_AXIS_FAULT:                return "fault";
    default:                             return "unknown-axis-state";
    }
}

/* ==========================================================================
 * 控制模式
 * ------------------------------------------------------------------------
 * 名字同时给出 SDK 模式名与线上 MsgType，便于对着总线日志排查。
 * ======================================================================== */

const char *jsdk_mode_string(jsdk_mode_t m)
{
    switch (m) {
    case JSDK_MODE_MIT:     return "mit(0x00)";
    case JSDK_MODE_CSP:     return "csp/pos(0x01)";
    case JSDK_MODE_CSV:     return "csv/vel(0x02)";
    case JSDK_MODE_CST:     return "cst/torque(0x03)";
    case JSDK_MODE_CURRENT: return "current(0x04)";
    default:                return "unknown-mode";
    }
}

/* ==========================================================================
 * 端点类型
 * ======================================================================== */

const char *jsdk_ep_type_string(jsdk_ep_type_t type)
{
    /* 复用协议层的名称表，保证与描述符解析器永远一致 */
    return jsdk_ep_type_name(type);
}

/* ==========================================================================
 * 浮点 → 定点文本
 * ------------------------------------------------------------------------
 * 纯整数实现，不依赖 libc 的浮点 printf（见头文件说明）。
 * ======================================================================== */

size_t jsdk_fmt_f(char *dst, size_t cap, double v, unsigned decimals)
{
    size_t   n = 0u;
    uint64_t scale = 1u;
    uint64_t mag;
    unsigned i;
    int      neg = 0;

    if (!dst || cap == 0u) return 0u;

    if (decimals > 9u) decimals = 9u;
    for (i = 0u; i < decimals; ++i) scale *= 10u;

    if (!(v == v)) {                       /* NaN：不能走 (int) 转换 */
        return jsdk_fmt_f(dst, cap, 0.0, decimals);   /* 统一写 0.000 */
    }
    if (v < 0.0) { neg = 1; v = -v; }

    /* ±1e15 以上不再保证精度；这里直接饱和，绝不产生 UB */
    if (v > 1.0e15) v = 1.0e15;

    mag = (uint64_t)(v * (double)scale + 0.5);

    /* 整数部分（手工十进制，避免依赖 %llu 的 libc 支持） */
    {
        char     tmp[24];
        unsigned t = 0u;
        uint64_t ip = mag / scale;
        uint64_t fp = mag % scale;

        if (neg && cap > n + 1u) dst[n++] = '-';

        do {
            tmp[t++] = (char)('0' + (int)(ip % 10u));
            ip /= 10u;
        } while (ip != 0u && t < sizeof tmp);
        while (t > 0u && n + 1u < cap) dst[n++] = tmp[--t];

        if (decimals > 0u && n + 1u < cap) {
            dst[n++] = '.';
            for (i = 0u; i < decimals; ++i) {
                scale /= 10u;
                if (n + 1u < cap) dst[n++] = (char)('0' + (int)((fp / scale) % 10u));
            }
        }
    }

    dst[n] = '\0';
    return n;
}

