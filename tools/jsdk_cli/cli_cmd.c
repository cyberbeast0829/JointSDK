/**
 * @file    cli_cmd.c
 * @brief   `jsdk-cli` 的子命令实现与分发
 *
 * @par 分级与安全闸
 *  只读子命令永不需要 `--yes`；**任何会写设备或让电机动的**子命令都需要。
 *  这比设计文档 §7.1 里"只有 mit 需要"更严一档，理由写在下面 `cmd_write()`
 *  的注释里：改 `gear_ratio` 一样会让电机跳。
 *
 * @par `--json`
 *  每个子命令都要在 `--json` 下输出**同样的字段名**（Python 绑定与 CI 会按
 *  这些名字断言），人工模式只是把同一份数据排版成表格。因此实现上先在脑子里
 *  定好字段，再分别写两条输出路径，而不是"先打印再想办法加 JSON"。
 */

#include "cli_app.h"
#include "jsdk_cli.h"

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

/* ==========================================================================
 * 小工具
 * ======================================================================== */

static void out_kv(cli_app_t *a, const char *k, const char *fmt, ...)
{
    va_list ap;

    if (a->o.json) return;
    fprintf(a->out, "  %-22s ", k);
    va_start(ap, fmt);
    vfprintf(a->out, fmt, ap);
    va_end(ap);
    fputc('\n', a->out);
}

/** 退出码：安全闸拒绝用 3，用法错误 2，运行失败 1。 */
#define CLI_EXIT_OK        0
#define CLI_EXIT_FAIL      1
#define CLI_EXIT_USAGE     2
#define CLI_EXIT_REFUSED   3

/** 解析一个无符号整数（拒绝负号与尾随垃圾）。 */
static int parse_u32_arg(const char *s, uint32_t *out)
{
    char *end = NULL;
    unsigned long v;

    if (!s || !*s || s[0] == '-') return -1;
    v = strtoul(s, &end, 0);
    if (!end || *end != '\0') return -1;
    *out = (uint32_t)v;
    return 0;
}


/** 要求 `--yes`；未给时打印拒绝原因并返回非 0。 */
static int require_yes(cli_app_t *a, const char *what)
{
    if (a->o.yes) return 0;

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_str(&j, "refused", what);
        cli_json_str(&j, "reason", "--yes is required for commands that write to "
                                  "or move the device");
        cli_json_obj_begin(&j, "hint");
        cli_json_str(&j, "rerun", "add --yes");
        cli_json_obj_end(&j);
        cli_json_finish(&j);
    }
    fprintf(a->err,
            "jsdk-cli: 拒绝执行 %s —— 会写设备/驱动电机，必须显式加 --yes\n"
            "          （先用只读子命令确认节点、量程与限值）\n", what);
    return CLI_EXIT_REFUSED;
}

/** 数值 → 人类可读字符串（jsdk_value_t）。 */
static void value_to_text(const jsdk_value_t *v, char *buf, size_t cap)
{
    switch (v->type) {
    case JSDK_EP_F32:  snprintf(buf, cap, "%.6g", (double)v->v.f32); break;
    case JSDK_EP_F64:  snprintf(buf, cap, "%.6g", v->v.f64); break;
    case JSDK_EP_BOOL: snprintf(buf, cap, "%s", v->v.boolean ? "true" : "false"); break;
    case JSDK_EP_U8:   snprintf(buf, cap, "%u", (unsigned)v->v.u8); break;
    case JSDK_EP_I8:   snprintf(buf, cap, "%d", (int)v->v.i8); break;
    case JSDK_EP_U16:  snprintf(buf, cap, "%u", (unsigned)v->v.u16); break;
    case JSDK_EP_I16:  snprintf(buf, cap, "%d", (int)v->v.i16); break;
    case JSDK_EP_U32:  snprintf(buf, cap, "%lu", (unsigned long)v->v.u32); break;
    case JSDK_EP_I32:  snprintf(buf, cap, "%ld", (long)v->v.i32); break;
    case JSDK_EP_U64:  snprintf(buf, cap, "%llu", (unsigned long long)v->v.u64); break;
    case JSDK_EP_I64:  snprintf(buf, cap, "%lld", (long long)v->v.i64); break;
    default:           snprintf(buf, cap, "<%s>", jsdk_ep_type_string(v->type)); break;
    }
}

/** 写入 JSON 的数值（按类型分别输出，避免 u64 精度丢失）。 */
static void value_to_json(cli_json_t *j, const char *key, const jsdk_value_t *v)
{
    switch (v->type) {
    case JSDK_EP_F32:  cli_json_num(j, key, (double)v->v.f32); break;
    case JSDK_EP_F64:  cli_json_num(j, key, v->v.f64); break;
    case JSDK_EP_BOOL: cli_json_bool(j, key, v->v.boolean); break;
    case JSDK_EP_U64:  cli_json_i64(j, key, (long long)v->v.u64); break;
    case JSDK_EP_I64:  cli_json_i64(j, key, (long long)v->v.i64); break;
    case JSDK_EP_I32:  cli_json_i64(j, key, (long long)v->v.i32); break;
    case JSDK_EP_I16:  cli_json_i64(j, key, (long long)v->v.i16); break;
    case JSDK_EP_I8:   cli_json_i64(j, key, (long long)v->v.i8); break;
    case JSDK_EP_U8:   cli_json_i64(j, key, (long long)v->v.u8); break;
    case JSDK_EP_U16:  cli_json_i64(j, key, (long long)v->v.u16); break;
    case JSDK_EP_U32:  cli_json_i64(j, key, (long long)v->v.u32); break;
    default:           cli_json_i64(j, key, 0); break;
    }
}

/** 按端点类型把文本解析成 jsdk_value_t（`write` 用）。 */
static int text_to_value(const char *text, jsdk_ep_type_t type, jsdk_value_t *v)
{
    char *end = NULL;

    memset(v, 0, sizeof *v);
    v->type = type;
    if (!text) return -1;

    if (type == JSDK_EP_BOOL) {
        if (strcmp(text, "true") == 0 || strcmp(text, "1") == 0)  { v->v.boolean = 1; return 0; }
        if (strcmp(text, "false") == 0 || strcmp(text, "0") == 0) { v->v.boolean = 0; return 0; }
        return -1;
    }
    if (type == JSDK_EP_F32 || type == JSDK_EP_F64) {
        double d = strtod(text, &end);
        if (!end || *end != '\0') return -1;
        if (type == JSDK_EP_F32) v->v.f32 = (float)d; else v->v.f64 = d;
        return 0;
    }

    /* 整型：按位宽解析，越界即拒绝（**不静默截断**） */
    if (text[0] == '-') {
        long long s = strtoll(text, &end, 0);
        if (!end || *end != '\0') return -1;
        switch (type) {
        case JSDK_EP_I8:  if (s < -128LL   || s > 127LL)   return -1; v->v.i8  = (int8_t)s;  return 0;
        case JSDK_EP_I16: if (s < -32768LL || s > 32767LL) return -1; v->v.i16 = (int16_t)s; return 0;
        case JSDK_EP_I32: if (s < -2147483648LL || s > 2147483647LL) return -1;
                          v->v.i32 = (int32_t)s; return 0;
        case JSDK_EP_I64: v->v.i64 = (int64_t)s; return 0;
        case JSDK_EP_U8: case JSDK_EP_U16: case JSDK_EP_U32: case JSDK_EP_U64:
            return -1;                                  /* 无符号却给了负数 */
        default: return -1;
        }
    } else {
        unsigned long long u = strtoull(text, &end, 0);
        if (!end || *end != '\0') return -1;
        switch (type) {
        case JSDK_EP_U8:  if (u > 255ull)   return -1; v->v.u8  = (uint8_t)u;  return 0;
        case JSDK_EP_U16: if (u > 65535ull) return -1; v->v.u16 = (uint16_t)u; return 0;
        case JSDK_EP_U32: if (u > 4294967295ull) return -1; v->v.u32 = (uint32_t)u; return 0;
        case JSDK_EP_U64: v->v.u64 = (uint64_t)u; return 0;
        case JSDK_EP_I8:  if (u > 127ull)   return -1; v->v.i8  = (int8_t)u;  return 0;
        case JSDK_EP_I16: if (u > 32767ull) return -1; v->v.i16 = (int16_t)u; return 0;
        case JSDK_EP_I32: if (u > 2147483647ull) return -1; v->v.i32 = (int32_t)u; return 0;
        case JSDK_EP_I64: if (u > 9223372036854775807ull) return -1;
                          v->v.i64 = (int64_t)u; return 0;
        default: return -1;
        }
    }
}

/* ==========================================================================
 * 只读：scan
 * ======================================================================== */

static int cmd_scan(cli_app_t *a)
{
    uint8_t ids[256];
    unsigned found = 0u;
    jsdk_status_t st;
    unsigned i;

    st = jsdk_context_discover(a->ctx, ids, sizeof ids, &found, (uint8_t)a->o.max_probe);
    if (st != JSDK_OK) {
        cli_error(a, "scan", st);
        return CLI_EXIT_FAIL;
    }

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_arr_begin(&j, "nodes");
        for (i = 0u; i < found; ++i) cli_json_raw_num(&j, (double)ids[i]);
        cli_json_arr_end(&j);
        cli_json_i64(&j, "count", (long long)found);
        cli_json_finish(&j);
    } else {
        fprintf(a->out, "发现 %u 个节点（被动 200 ms + 主动探测 1..%u）:\n",
                found, a->o.max_probe);
        for (i = 0u; i < found; ++i) {
            fprintf(a->out, "  node %u\n", (unsigned)ids[i]);
        }
        if (found == 0u) {
            fprintf(a->out, "  （无）—— 逐条检查：\n"
                            "    1. 总线是否 up；波特率 / FD / BRS 是否与设备 "
                            "can.config.baud_rate 一致（协议无运行时协商）\n"
                            "    2. 是否有 120Ω 终端电阻、CAN_H/L 是否接反\n"
                            "    3. 设备**必须先收到过一帧**才学到主站地址（地址为 0 时"
                            "设备完全不回复）\n");
            if (a->o.max_probe == 0u) {
                fprintf(a->out, "    4. 本次是 `--probe 0`（仅被动听心跳）：首次连接"
                                "请用 `--probe 16` 主动探测\n");
            }
        }
    }
    return CLI_EXIT_OK;
}

/* ==========================================================================
 * 只读：info
 * ======================================================================== */

static int cmd_info(cli_app_t *a)
{
    jsdk_device_info_t info;
    jsdk_joint_t *j = cli_joint(a);
    jsdk_status_t st;

    if (!j) return CLI_EXIT_FAIL;
    st = jsdk_joint_get_device_info(j, &info);
    if (st != JSDK_OK) { cli_error(a, "info", st); return CLI_EXIT_FAIL; }

    if (a->o.json) {
        cli_json_t js;
        cli_json_init(&js, a->out, 0);
        cli_json_i64(&js, "hw_version", (long long)info.hw_version);
        cli_json_i64(&js, "fw_version", (long long)info.fw_version);
        cli_json_i64(&js, "serial", (long long)info.serial);
        cli_json_bool(&js, "classic", info.classic);
        cli_json_i64(&js, "node", (long long)a->o.node);
        cli_json_finish(&js);
    } else {
        fprintf(a->out, "节点 %u:\n", (unsigned)a->o.node);
        out_kv(a, "hw_version", "%lu", (unsigned long)info.hw_version);
        out_kv(a, "fw_version", "%lu", (unsigned long)info.fw_version);
        out_kv(a, "serial", "%llu", (unsigned long long)info.serial);
        out_kv(a, "device_mode", "%s", info.classic ? "Classic" : "FD");
    }
    return CLI_EXIT_OK;
}

/* ==========================================================================
 * 只读：health
 * ======================================================================== */

static void print_health_json(cli_app_t *a, const jsdk_joint_feedback_t *fb,
                              const jsdk_bus_state_t *bs)
{
    cli_json_t j;

    cli_json_init(&j, a->out, 0);
    cli_json_obj_begin(&j, "joint");
    cli_json_i64(&j, "node", (long long)a->o.node);
    cli_json_num(&j, "pos_rad", fb->pos);
    cli_json_num(&j, "vel_rad_s", fb->vel);
    cli_json_num(&j, "current_A", fb->current_A);
    cli_json_num(&j, "torque_Nm", fb->torque_Nm);
    cli_json_num(&j, "t_motor_C", fb->t_motor_C);
    cli_json_num(&j, "t_fet_C", fb->t_fet_C);
    cli_json_num(&j, "vbus_V", fb->vbus_V);
    cli_json_num(&j, "ibus_A", fb->ibus_A);
    cli_json_str(&j, "axis_state", jsdk_axis_state_string(fb->axis_state));
    cli_json_str(&j, "mode", cli_mode_name(fb->mode));
    cli_json_i64(&j, "mode_state_nibble", (long long)fb->mode_state);
    cli_json_i64(&j, "err_code", (long long)fb->err_code);
    cli_json_str(&j, "err_name", jsdk_joint_error_string(fb->err_code));
    cli_json_i64(&j, "hb_error", (long long)fb->hb_error);
    cli_json_i64(&j, "axis_error", (long long)fb->axis_error);
    cli_json_i64(&j, "age_ms", (long long)fb->age_ms);
    cli_json_i64(&j, "tx_frames", (long long)fb->tx_frames);
    cli_json_i64(&j, "tx_rejected", (long long)fb->tx_rejected);
    cli_json_i64(&j, "status_flags", (long long)fb->status_flags);
    cli_json_bool(&j, "online", fb->online);
    cli_json_bool(&j, "enabled", jsdk_joint_is_enabled(a->joint));
    cli_json_bool(&j, "fault", jsdk_joint_is_fault(a->joint));
    cli_json_obj_end(&j);

    cli_json_obj_begin(&j, "bus");
    cli_json_i64(&j, "tx_frames", (long long)bs->tx_frames);
    cli_json_i64(&j, "rx_frames", (long long)bs->rx_frames);
    cli_json_i64(&j, "tx_failed", (long long)bs->tx_failed);
    cli_json_i64(&j, "rx_dropped", (long long)bs->rx_dropped);
    cli_json_i64(&j, "keepalive_sent", (long long)bs->keepalive_sent);
    cli_json_i64(&j, "last_rx_age_ms", (long long)bs->last_rx_age_ms);
    cli_json_i64(&j, "link_errors", (long long)bs->link_errors);
    cli_json_i64(&j, "hal_bus_flags", (long long)bs->hal_bus_flags);
    cli_json_i64(&j, "nodes_online", (long long)bs->nodes_online);
    cli_json_bool(&j, "link_up", bs->link_up);
    cli_json_obj_end(&j);
    cli_json_finish(&j);
}

static int health_common(cli_app_t *a, int with_json_wrapper)
{
    jsdk_joint_feedback_t fb;
    jsdk_bus_state_t      bs;
    jsdk_joint_t         *j = cli_joint(a);
    jsdk_status_t st;

    (void)with_json_wrapper;
    if (!j) return CLI_EXIT_FAIL;

    st = jsdk_joint_get_feedback(j, &fb);
    if (st != JSDK_OK) { cli_error(a, "health", st); return CLI_EXIT_FAIL; }
    st = jsdk_context_get_bus_state(a->ctx, &bs);
    if (st != JSDK_OK) { cli_error(a, "health(bus)", st); return CLI_EXIT_FAIL; }

    if (a->o.json) {
        print_health_json(a, &fb, &bs);
        return CLI_EXIT_OK;
    }

    fprintf(a->out, "节点 %u 健康快照:\n", (unsigned)a->o.node);
    out_kv(a, "online", "%s", fb.online ? "yes" : "no");
    out_kv(a, "enabled", "%s", jsdk_joint_is_enabled(j) ? "yes" : "no");
    out_kv(a, "fault", "%s", jsdk_joint_is_fault(j) ? "yes" : "no");
    out_kv(a, "axis_state", "%s", jsdk_axis_state_string(fb.axis_state));
    out_kv(a, "mode", "%s", cli_mode_name(fb.mode));
    out_kv(a, "mode_state", "%u", (unsigned)fb.mode_state);
    out_kv(a, "err_code", "%u (%s)", (unsigned)fb.err_code,
           jsdk_joint_error_string(fb.err_code));
    out_kv(a, "hb_error", "0x%02X", (unsigned)fb.hb_error);
    out_kv(a, "axis_error", "0x%08lX", (unsigned long)fb.axis_error);
    out_kv(a, "pos_rad", "%.4f", fb.pos);
    out_kv(a, "vel_rad_s", "%.4f", fb.vel);
    out_kv(a, "current_A", "%.3f", fb.current_A);
    out_kv(a, "torque_Nm", "%.3f", fb.torque_Nm);
    out_kv(a, "t_motor_C", "%.1f", fb.t_motor_C);
    out_kv(a, "t_fet_C", "%.1f", fb.t_fet_C);
    out_kv(a, "vbus_V", "%.2f", fb.vbus_V);
    out_kv(a, "ibus_A", "%.3f", fb.ibus_A);
    out_kv(a, "age_ms", "%lu", (unsigned long)fb.age_ms);
    out_kv(a, "tx_frames", "%lu", (unsigned long)fb.tx_frames);
    out_kv(a, "tx_rejected", "%lu", (unsigned long)fb.tx_rejected);
    out_kv(a, "status_flags", "0x%04X", (unsigned)fb.status_flags);

    if (jsdk_joint_is_fault(j)) {
        char line[192];
        if (jsdk_joint_describe_fault(j, line, sizeof line) > 0) {
            out_kv(a, "fault_detail", "%s", line);
        }
    }

    fprintf(a->out, "总线:\n");
    out_kv(a, "link_up", "%s", bs.link_up ? "yes" : "no");
    out_kv(a, "nodes_online", "%u", (unsigned)bs.nodes_online);
    out_kv(a, "tx/rx", "%lu / %lu", (unsigned long)bs.tx_frames,
           (unsigned long)bs.rx_frames);
    out_kv(a, "tx_failed", "%lu", (unsigned long)bs.tx_failed);
    out_kv(a, "rx_dropped", "%lu", (unsigned long)bs.rx_dropped);
    out_kv(a, "keepalive_sent", "%lu", (unsigned long)bs.keepalive_sent);
    out_kv(a, "last_rx_age_ms", "%lu", (unsigned long)bs.last_rx_age_ms);
    out_kv(a, "link_errors", "%lu", (unsigned long)bs.link_errors);
    out_kv(a, "hal_bus_flags", "0x%08lX", (unsigned long)bs.hal_bus_flags);
    return CLI_EXIT_OK;
}

static int cmd_health(cli_app_t *a) { return health_common(a, 0); }

/* ==========================================================================
 * 只读：mon
 * ======================================================================== */

static int cmd_mon(cli_app_t *a)
{
    uint32_t elapsed = 0u;
    unsigned period_ms = (unsigned)(1000 / a->o.rate_hz);
    FILE *csv = NULL;
    int first = 1;

    if (a->o.csv) {
        csv = fopen(a->o.csv, "w");
        if (!csv) {
            fprintf(a->err, "jsdk-cli: 打不开 --csv 文件 %s\n", a->o.csv);
            return CLI_EXIT_FAIL;
        }
        /* 列：时间 → 物理量 → 状态 → 诊断计数。取的是 cyberbeast_tool.py
           采集脚本里共有的量，方便两边对着看（列顺序不保证逐列相同）。 */
        fprintf(csv, "t_ms,node,pos_rad,vel_rad_s,current_A,torque_Nm,"
                     "t_motor_C,t_fet_C,vbus_V,ibus_A,axis_state,mode,err_code,"
                     "hb_error,age_ms,tx_frames,tx_rejected\n");
    }

    /* JSON 模式下 mon 输出 NDJSON（每行一个对象），便于流式管道消费。 */

    while (cli_should_continue(a, elapsed)) {
        jsdk_joint_feedback_t fb;
        jsdk_status_t st;

        jsdk_context_poll(a->ctx, 0u);

        st = jsdk_joint_get_feedback(a->joint, &fb);
        if (st == JSDK_OK) {
            if (a->o.json) {
                cli_json_t j;
                cli_json_init(&j, a->out, 0);
                cli_json_i64(&j, "t_ms", (long long)elapsed);
                cli_json_i64(&j, "node", (long long)a->o.node);
                cli_json_num(&j, "pos_rad", fb.pos);
                cli_json_num(&j, "vel_rad_s", fb.vel);
                cli_json_num(&j, "torque_Nm", fb.torque_Nm);
                cli_json_num(&j, "t_motor_C", fb.t_motor_C);
                cli_json_num(&j, "vbus_V", fb.vbus_V);
                cli_json_str(&j, "axis_state", jsdk_axis_state_string(fb.axis_state));
                cli_json_str(&j, "mode", cli_mode_name(fb.mode));
                cli_json_i64(&j, "err_code", (long long)fb.err_code);
                cli_json_i64(&j, "age_ms", (long long)fb.age_ms);
                cli_json_bool(&j, "valid", fb.valid);
                cli_json_finish(&j);
            } else {
                if (first) {
                    fprintf(a->out, "%9s %7s %9s %9s %7s %7s %7s  %-12s %s\n",
                            "t_ms", "pos_rad", "vel_rad/s", "tau_Nm", "tMot", "vbus",
                            "age_ms", "state", "err");
                }
                fprintf(a->out, "%9lu %7.3f %9.3f %9.3f %7.1f %7.1f %7lu  %-12s %u\n",
                        (unsigned long)elapsed, fb.pos, fb.vel, fb.torque_Nm,
                        fb.t_motor_C, fb.vbus_V, (unsigned long)fb.age_ms,
                        jsdk_axis_state_string(fb.axis_state), (unsigned)fb.err_code);
            }
            if (csv) {
                fprintf(csv, "%lu,%u,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.3f,%.3f,%s,%s,%u,%u,%lu,%lu,%lu\n",
                        (unsigned long)elapsed, (unsigned)a->o.node,
                        fb.pos, fb.vel, fb.current_A, fb.torque_Nm,
                        fb.t_motor_C, fb.t_fet_C, fb.vbus_V, fb.ibus_A,
                        jsdk_axis_state_string(fb.axis_state), cli_mode_name(fb.mode),
                        (unsigned)fb.err_code, (unsigned)fb.hb_error,
                        (unsigned long)fb.age_ms, (unsigned long)fb.tx_frames,
                        (unsigned long)fb.tx_rejected);
                fflush(csv);
            }
        }
        first = 0;

        if (period_ms > 0u && cli_sleep_ms(period_ms)) break;
        elapsed += period_ms;
    }

    if (csv) fclose(csv);
    if (!a->o.json && !a->o.quiet) fprintf(a->out, "（mon 结束）\n");
    return CLI_EXIT_OK;
}

/* ==========================================================================
 * 只读：read / batch-read
 * ======================================================================== */

static int cmd_read(cli_app_t *a)
{
    jsdk_value_t v;
    jsdk_status_t st;
    char text[64];
    const char *path = (a->o.nargs > 0u) ? a->o.args[0] : NULL;

    if (!path) { fprintf(a->err, "jsdk-cli: read 需要 <path>\n"); return CLI_EXIT_USAGE; }

    st = jsdk_joint_param_get(a->joint, path, &v);
    if (st != JSDK_OK) { cli_error(a, path, st); return CLI_EXIT_FAIL; }

    value_to_text(&v, text, sizeof text);
    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_str(&j, "path", path);
        cli_json_str(&j, "type", jsdk_ep_type_string(v.type));
        value_to_json(&j, "value", &v);
        cli_json_str(&j, "value_text", text);
        cli_json_finish(&j);
    } else {
        fprintf(a->out, "%s = %s (%s)\n", path, text, jsdk_ep_type_string(v.type));
    }
    return CLI_EXIT_OK;
}

static int cmd_batch_read(cli_app_t *a)
{
    jsdk_param_req_t reqs[8];
    unsigned n, i;
    jsdk_status_t st;
    char text[64];

    if (a->o.nargs == 0u) { fprintf(a->err, "jsdk-cli: batch-read 需要 <path>...\n"); return CLI_EXIT_USAGE; }
    if (a->o.nargs > 8u)   { fprintf(a->err, "jsdk-cli: 一次最多 8 条（超了请分批）\n"); return CLI_EXIT_USAGE; }

    n = a->o.nargs;
    memset(reqs, 0, sizeof reqs);
    for (i = 0u; i < n; ++i) reqs[i].path = a->o.args[i];

    st = jsdk_joint_param_get_batch(a->joint, reqs, n);

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_arr_begin(&j, "values");
        for (i = 0u; i < n; ++i) {
            cli_json_obj_begin(&j, NULL);
            cli_json_str(&j, "path", reqs[i].path);
            if (reqs[i].status == JSDK_OK) {
                value_to_json(&j, "value", &reqs[i].value);
                cli_json_str(&j, "type", jsdk_ep_type_string(reqs[i].value.type));
            } else {
                cli_json_str(&j, "error", jsdk_status_string(reqs[i].status));
            }
            cli_json_obj_end(&j);
        }
        cli_json_arr_end(&j);
        cli_json_str(&j, "status", jsdk_status_string(st));
        cli_json_bool(&j, "single_frame", st == JSDK_OK ? 1 : 0);
        cli_json_finish(&j);
    } else {
        for (i = 0u; i < n; ++i) {
            if (reqs[i].status == JSDK_OK) {
                value_to_text(&reqs[i].value, text, sizeof text);
                fprintf(a->out, "  %-52s = %s\n", reqs[i].path, text);
            } else {
                fprintf(a->out, "  %-52s ! %s\n", reqs[i].path,
                        jsdk_status_string(reqs[i].status));
            }
        }
    }
    return (st == JSDK_OK) ? CLI_EXIT_OK : CLI_EXIT_FAIL;
}

/* ==========================================================================
 * 只读：dump-config
 * ======================================================================== */

static int cmd_dump_config(cli_app_t *a)
{
    jsdk_joint_config_snapshot_t s;
    jsdk_status_t st = jsdk_joint_read_config_snapshot(a->joint, &s);

    if (st != JSDK_OK) { cli_error(a, "dump-config", st); return CLI_EXIT_FAIL; }

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_bool(&j, "valid", s.valid);
        cli_json_num(&j, "gear_ratio", s.gear_ratio);
        cli_json_num(&j, "mit_max_pos", s.mit_max_pos);
        cli_json_num(&j, "mit_max_vel", s.mit_max_vel);
        cli_json_num(&j, "mit_max_torque", s.mit_max_torque);
        cli_json_num(&j, "mit_max_kp", s.mit_max_kp);
        cli_json_num(&j, "mit_max_kd", s.mit_max_kd);
        cli_json_num(&j, "torque_constant", s.torque_constant);
        cli_json_i64(&j, "node_id", (long long)s.node_id);
        cli_json_i64(&j, "heartbeat_rate_ms", (long long)s.heartbeat_rate_ms);
        cli_json_i64(&j, "break_timeout_ms", (long long)s.break_timeout_ms);
        cli_json_finish(&j);
    } else {
        fprintf(a->out, "节点 %u 配置快照 (valid=%d):\n", (unsigned)a->o.node, s.valid);
        out_kv(a, "gear_ratio", "%.6g", (double)s.gear_ratio);
        out_kv(a, "mit_max_pos", "%.6g rad", (double)s.mit_max_pos);
        out_kv(a, "mit_max_vel", "%.6g rad/s", (double)s.mit_max_vel);
        out_kv(a, "mit_max_torque", "%.6g N·m", (double)s.mit_max_torque);
        out_kv(a, "mit_max_kp", "%.6g", (double)s.mit_max_kp);
        out_kv(a, "mit_max_kd", "%.6g", (double)s.mit_max_kd);
        out_kv(a, "torque_constant", "%.6g N·m/A", (double)s.torque_constant);
        out_kv(a, "node_id", "%lu", (unsigned long)s.node_id);
        out_kv(a, "heartbeat_rate_ms", "%lu", (unsigned long)s.heartbeat_rate_ms);
        out_kv(a, "break_timeout_ms", "%lu%s", (unsigned long)s.break_timeout_ms,
               (s.break_timeout_ms == 0u) ? "  ← 固件按 100 ms 处理，0 不等于关闭" : "");
    }
    return CLI_EXIT_OK;
}

/* ==========================================================================
 * 只读：err
 * ======================================================================== */

static int cmd_err(cli_app_t *a)
{
    jsdk_fault_info_t f;
    jsdk_status_t st = jsdk_joint_query_error_detail(a->joint, &f);

    if (st != JSDK_OK) { cli_error(a, "err", st); return CLI_EXIT_FAIL; }

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_i64(&j, "mit_err", (long long)f.mit_err);
        cli_json_str(&j, "mit_err_name", jsdk_joint_error_string(f.mit_err));
        cli_json_i64(&j, "hb_flags", (long long)f.hb_flags);
        cli_json_i64(&j, "motor_error", (long long)f.motor_error);
        cli_json_i64(&j, "encoder_error", (long long)f.encoder_error);
        cli_json_i64(&j, "sensorless_error", (long long)f.sensorless_error);
        cli_json_i64(&j, "controller_error", (long long)f.controller_error);
        cli_json_i64(&j, "system_error", (long long)f.system_error);
        cli_json_i64(&j, "axis_error", (long long)f.axis_error);
        cli_json_finish(&j);
    } else {
        static const char *names[6] = { "motor", "encoder", "sensorless",
                                        "controller", "system", "axis" };
        const uint32_t vals[6] = { f.motor_error, f.encoder_error,
                                   f.sensorless_error, f.controller_error,
                                   f.system_error, f.axis_error };
        unsigned i;

        fprintf(a->out, "节点 %u 错误明细:\n", (unsigned)a->o.node);
        out_kv(a, "mit_err", "%u (%s)", (unsigned)f.mit_err,
               jsdk_joint_error_string(f.mit_err));
        out_kv(a, "hb_flags", "0x%02X", (unsigned)f.hb_flags);
        for (i = 0u; i < 6u; ++i) {
            unsigned bit = 0u;
            const char *first = jsdk_axis_error_first(vals[i], &bit);
            fprintf(a->out, "  %-22s 0x%08lX  %s\n", names[i],
                    (unsigned long)vals[i],
                    first ? first : "无错误");
            if (first) fprintf(a->out, "  %-22s           bit %u\n", "", bit);
        }
    }
    return CLI_EXIT_OK;
}

/* ==========================================================================
 * 只读：hb-dump
 * ======================================================================== */

/**
 * 心跳原始字节 + 解码对照。
 *
 * 关键在"原始字节"：这些字节来自 HAL 包装层的 RX 留存（`cli_app_t.rx_ring`），
 * 不是从解码结果**重新编码**出来的 —— 后者只反映我们以为对方发了什么，
 * 也正因如此才需要把真帧打出来和客户的抓包工具对照。
 *
 * 载荷字节的含义（见 docs/PROTOCOL_NOTES.zh-CN.md §6.2）：
 *   b0 = life<<4 | err_flags（5 bit），b1 = state<<4 | control_mode
 */
static int cmd_hb_dump(cli_app_t *a)
{
    unsigned i;
    unsigned shown = 0u;
    cli_json_t j;
    uint32_t waited = 0u;

    /*
     * ⚠ 必须先**自己等**一个心跳。
     *   每次进程只跑一条命令，所以"当前已留存的帧"在 hb-dump 单跑时永远是空的
     *   —— 早期版本就直接报"没有缓存到心跳"，客户会以为命令坏了。
     *   这里轮询到拿到该节点的心跳为止（最多 2 s，或 --timeout 更小时以它为准）。
     */
    {
        uint32_t budget = a->o.timeout_ms;
        if (budget == 0u || budget > 2000u) budget = 2000u;

        for (waited = 0u; waited < budget; waited += 5u) {
            jsdk_context_poll(a->ctx, 0u);
            for (i = 0u; i < a->rx_ring_n; ++i) {
                unsigned mt  = (unsigned)((a->rx_ring[i].id >> 18) & 0xFFu);
                unsigned src = (unsigned)((a->rx_ring[i].id >> 2) & 0xFFu);
                if (mt == 0x48u && src == (unsigned)a->o.node) { waited = budget; break; }
            }
            if (waited >= budget) break;
            if (cli_sleep_ms(5u)) break;
        }
    }

    memset(&j, 0, sizeof j);
    if (a->o.json) {
        cli_json_init(&j, a->out, 0);
        cli_json_arr_begin(&j, "heartbeats");
    }

    for (i = 0u; i < a->rx_ring_n; ++i) {
        const jsdk_can_frame_t *f = &a->rx_ring[i];
        unsigned mt = (unsigned)((f->id >> 18) & 0xFFu);
        unsigned src = (unsigned)((f->id >> 2) & 0xFFu);
        unsigned k;
        char hex[3u * 64u + 1u];
        size_t off = 0u;

        static const char digits[] = "0123456789ABCDEF";

        if (mt != 0x48u) continue;              /* 只要心跳（HEARTBEAT） */
        if (src != (unsigned)a->o.node) continue;

        hex[0] = '\0';
        for (k = 0u; k < f->len && k < 64u; ++k) {
            hex[off++] = digits[(f->data[k] >> 4) & 0xFu];
            hex[off++] = digits[f->data[k] & 0xFu];
            hex[off++] = ' ';
        }
        hex[off] = '\0';

        if (a->o.json) {
            cli_json_obj_begin(&j, NULL);
            cli_json_i64(&j, "src", (long long)src);
            cli_json_i64(&j, "len", (long long)f->len);
            cli_json_str(&j, "bytes", hex);
            cli_json_i64(&j, "life", (long long)(f->len > 0u ? (f->data[0] >> 4) & 0xFu : 0u));
            cli_json_i64(&j, "err_flags", (long long)(f->len > 0u ? f->data[0] & 0xFu : 0u));
            cli_json_i64(&j, "state", (long long)(f->len > 1u ? (f->data[1] >> 4) & 0xFu : 0u));
            cli_json_i64(&j, "control_mode", (long long)(f->len > 1u ? f->data[1] & 0x0Fu : 0u));
            cli_json_obj_end(&j);
        } else {
            fprintf(a->out, "src=%u len=%u  bytes: %s\n", src, (unsigned)f->len, hex);
            if (f->len >= 2u) {
                unsigned b0 = f->data[0];
                unsigned b1 = f->data[1];
                fprintf(a->out, "    b0 life=%u%u%u%u err=0x%X   b1 state=%u%u%u%u ctrl=%u\n",
                        (b0 >> 4) & 1u, (b0 >> 5) & 1u, (b0 >> 6) & 1u, (b0 >> 7) & 1u,
                        b0 & 0xFu,
                        (b1 >> 4) & 1u, (b1 >> 5) & 1u, (b1 >> 6) & 1u, (b1 >> 7) & 1u,
                        b1 & 0xFu);
            }
        }
        shown++;
    }

    if (a->o.json) {
        cli_json_arr_end(&j);
        cli_json_i64(&j, "count", (long long)shown);
        cli_json_i64(&j, "waited_ms", (long long)waited);
        cli_json_finish(&j);
    } else if (shown == 0u) {
        fprintf(a->out,
                "等了 %lu ms 没等到节点 %u 的心跳（本次共收到 %u 帧）。\n"
                "逐条检查：\n"
                "  1. 节点地址是否正确（--node）\n"
                "  2. 波特率 / FD / BRS 是否与设备一致；是否有 120Ω 终端\n"
                "  3. 设备是否曾经收到过至少一帧（地址为 0 时设备完全不回复）\n"
                "  4. 设备的心跳周期是否被设得极长（`read "
                "axis0.config.can.heartbeat_rate_ms`）\n",
                (unsigned long)waited, (unsigned)a->o.node, (unsigned)a->rx_frames);
    }
    return (shown > 0u) ? CLI_EXIT_OK : CLI_EXIT_FAIL;
}

/* ==========================================================================
 * 只读：desc-info / ep-list / ep-lookup / desc-export / desc-import
 * ======================================================================== */

static int cmd_desc_info(cli_app_t *a)
{
    jsdk_desc_info_t di;
    jsdk_status_t st = jsdk_context_get_desc_info(a->ctx, &di);

    if (st != JSDK_OK) { cli_error(a, "desc-info", st); return CLI_EXIT_FAIL; }

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_i64(&j, "total_len", (long long)di.total_len);
        cli_json_i64(&j, "crc", (long long)di.crc);
        cli_json_i64(&j, "fw_version", (long long)di.fw_version);
        cli_json_i64(&j, "hw_version", (long long)di.hw_version);
        cli_json_i64(&j, "endpoint_count", (long long)di.endpoint_count);
        cli_json_i64(&j, "parsed_total", (long long)di.parsed_total);
        cli_json_i64(&j, "frames_rx", (long long)di.frames_rx);
        cli_json_i64(&j, "bytes_scanned", (long long)di.bytes_scanned);
        cli_json_bool(&j, "complete", di.complete);
        cli_json_i64(&j, "mode_used", (long long)di.mode_used);
        cli_json_bool(&j, "shared_hit", di.shared_hit);
        cli_json_bool(&j, "raw_sink_failed", di.raw_sink_failed);
        cli_json_finish(&j);
    } else {
        fprintf(a->out, "描述符:\n");
        out_kv(a, "total_len", "%lu 字节", (unsigned long)di.total_len);
        out_kv(a, "crc", "0x%04X", (unsigned)di.crc);
        out_kv(a, "fw_version", "%lu", (unsigned long)di.fw_version);
        out_kv(a, "hw_version", "%lu", (unsigned long)di.hw_version);
        out_kv(a, "endpoint_count", "%u（已保留）", di.endpoint_count);
        out_kv(a, "parsed_total", "%u（解析到）", di.parsed_total);
        out_kv(a, "frames_rx", "%u", di.frames_rx);
        out_kv(a, "bytes_scanned", "%lu", (unsigned long)di.bytes_scanned);
        out_kv(a, "complete", "%s", di.complete ? "yes" : "NO (提前终止)");
        out_kv(a, "shared_hit", "%s", di.shared_hit ? "yes" : "no");
    }
    return CLI_EXIT_OK;
}

/* ep-list 的遍历上下文 */
typedef struct {
    cli_app_t *a;
    cli_json_t *j;
    const char *filter;
    unsigned    n;
    unsigned    limit;
} ep_list_ctx_t;

static int ep_list_visit(void *user, const char *path, uint16_t ep_id,
                         jsdk_ep_type_t type, uint8_t access)
{
    ep_list_ctx_t *c = (ep_list_ctx_t *)user;
    char acc[4];
    size_t flen;

    if (c->filter && c->filter[0]) {
        flen = strlen(c->filter);
        /*
         * 以 `*` 结尾 → **前缀**匹配（`axis0.controller.config.mit_max_*`）；
         * 否则 → **子串**匹配。
         *
         * 为什么默认子串：现场最常见的用法是 `--filter mit_max_`（"我要看
         * mit_max 这一族"），而完整路径前缀是 `axis0.controller.config.`。
         * 只做前缀匹配会让这条命令**静默返回 0 条**，客户会以为端点丢了。
         */
        if (flen > 0u && c->filter[flen - 1u] == '*') {
            flen--;
            if (strncmp(path, c->filter, flen) != 0) return 0;
        } else if (strstr(path, c->filter) == NULL) {
            return 0;
        }
    }

    acc[0] = (access & JSDK_EP_ACCESS_R) ? 'r' : '-';
    acc[1] = (access & JSDK_EP_ACCESS_W) ? 'w' : '-';
    acc[2] = '\0';

    if (c->j) {
        cli_json_obj_begin(c->j, NULL);
        cli_json_str(c->j, "path", path);
        cli_json_i64(c->j, "id", (long long)ep_id);
        cli_json_str(c->j, "type", jsdk_ep_type_string(type));
        cli_json_str(c->j, "access", acc);
        cli_json_obj_end(c->j);
    } else {
        fprintf(c->a->out, "  %5u  %-3s  %-14s %s\n", (unsigned)ep_id, acc,
                jsdk_ep_type_string(type), path);
    }
    c->n++;
    return (c->limit && c->n >= c->limit) ? 1 : 0;
}

static int cmd_ep_list(cli_app_t *a)
{
    ep_list_ctx_t c;
    jsdk_status_t st;

    memset(&c, 0, sizeof c);
    c.a = a;
    c.filter = a->o.filter;
    c.limit = 0u;

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_arr_begin(&j, "endpoints");
        c.j = &j;

        st = jsdk_endpoint_enumerate(a->ctx, ep_list_visit, &c);

        cli_json_arr_end(&j);
        cli_json_i64(&j, "count", (long long)c.n);
        cli_json_str(&j, "status", jsdk_status_string(st));
        cli_json_finish(&j);
    } else {
        fprintf(a->out, "%5s  %-3s  %-14s %s\n", "id", "acc", "type", "path");
        st = jsdk_endpoint_enumerate(a->ctx, ep_list_visit, &c);
        fprintf(a->out, "共 %u 个端点%s%s\n", c.n,
                a->o.filter ? "（filter=" : "", a->o.filter ? a->o.filter : "");
    }

    return (st == JSDK_OK) ? CLI_EXIT_OK : CLI_EXIT_FAIL;
}

static int cmd_ep_lookup(cli_app_t *a)
{
    uint16_t ep_id = 0u;
    jsdk_ep_type_t type = JSDK_EP_JSON;
    uint8_t access = 0u;
    jsdk_status_t st;
    const char *path = (a->o.nargs > 0u) ? a->o.args[0] : NULL;

    if (!path) { fprintf(a->err, "jsdk-cli: ep-lookup 需要 <path>\n"); return CLI_EXIT_USAGE; }

    st = jsdk_endpoint_lookup(a->ctx, path, &ep_id, &type, &access);
    if (st != JSDK_OK) { cli_error(a, path, st); return CLI_EXIT_FAIL; }

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_str(&j, "path", path);
        cli_json_i64(&j, "id", (long long)ep_id);
        cli_json_str(&j, "type", jsdk_ep_type_string(type));
        cli_json_bool(&j, "readable", (access & JSDK_EP_ACCESS_R) ? 1 : 0);
        cli_json_bool(&j, "writable", (access & JSDK_EP_ACCESS_W) ? 1 : 0);
        cli_json_finish(&j);
    } else {
        fprintf(a->out, "%s\n  id=%u type=%s access=%c%c\n", path, (unsigned)ep_id,
                jsdk_ep_type_string(type),
                (access & JSDK_EP_ACCESS_R) ? 'r' : '-',
                (access & JSDK_EP_ACCESS_W) ? 'w' : '-');
    }
    return CLI_EXIT_OK;
}

static int cmd_desc_export(cli_app_t *a)
{
    size_t cap, len = 0u;
    void  *buf;
    jsdk_status_t st;
    FILE  *f;
    const char *path = (a->o.nargs > 0u) ? a->o.args[0] : NULL;

    if (!path) { fprintf(a->err, "jsdk-cli: desc-export 需要 <file>\n"); return CLI_EXIT_USAGE; }

    cap = jsdk_desc_export_max_size(a->ctx);
    buf = calloc(1u, cap ? cap : 1u);
    if (!buf) { fprintf(a->err, "jsdk-cli: 内存不足（%u 字节）\n", (unsigned)cap); return CLI_EXIT_FAIL; }

    st = jsdk_context_desc_export(a->ctx, buf, cap, &len);
    if (st != JSDK_OK) { free(buf); cli_error(a, "desc-export", st); return CLI_EXIT_FAIL; }

    f = fopen(path, "wb");
    if (!f) {
        free(buf);
        fprintf(a->err, "jsdk-cli: 写不了 %s\n", path);
        return CLI_EXIT_FAIL;
    }
    if (fwrite(buf, 1u, len, f) != len) {
        fclose(f); free(buf);
        fprintf(a->err, "jsdk-cli: 写 %s 不完整\n", path);
        return CLI_EXIT_FAIL;
    }
    fclose(f);
    free(buf);

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_str(&j, "file", path);
        cli_json_i64(&j, "bytes", (long long)len);
        cli_json_finish(&j);
    } else {
        fprintf(a->out, "已导出 %lu 字节 → %s\n", (unsigned long)len, path);
        fprintf(a->out, "提示：缓存与 retain/filter_paths/SDK 格式版本绑定，"
                        "改其中任一项都要重新下载。\n");
    }
    return CLI_EXIT_OK;
}

static int cmd_desc_import(cli_app_t *a)
{
    const char *path = (a->o.nargs > 0u) ? a->o.args[0] : NULL;
    unsigned count = 0u;
    FILE *f;
    long  sz;
    void *buf;
    size_t got;
    jsdk_status_t st;

    if (!path) { fprintf(a->err, "jsdk-cli: desc-import 需要 <file>\n"); return CLI_EXIT_USAGE; }

    f = fopen(path, "rb");
    if (!f) { fprintf(a->err, "jsdk-cli: 读不了 %s\n", path); return CLI_EXIT_FAIL; }
    if (fseek(f, 0L, SEEK_END) != 0) { fclose(f); return CLI_EXIT_FAIL; }
    sz = ftell(f);
    if (sz <= 0) { fclose(f); fprintf(a->err, "jsdk-cli: %s 是空文件\n", path); return CLI_EXIT_FAIL; }
    rewind(f);
    buf = calloc(1u, (size_t)sz);
    if (!buf) { fclose(f); fprintf(a->err, "jsdk-cli: 内存不足\n"); return CLI_EXIT_FAIL; }
    got = fread(buf, 1u, (size_t)sz, f);
    fclose(f);
    if (got != (size_t)sz) {
        free(buf);
        fprintf(a->err, "jsdk-cli: 读 %s 不完整\n", path);
        return CLI_EXIT_FAIL;
    }

    st = jsdk_context_desc_import(a->ctx, buf, got);
    free(buf);
    if (st != JSDK_OK) { cli_error(a, "desc-import", st); return CLI_EXIT_FAIL; }

    /*
     * 打印端点表规模：这证明"导入之后端点表直接可用"，**没有**经过 configure()
     * （本命令的 needs_desc = 0，所以确实没有下载那 41 KB）。
     * 客户最后确认缓存可用的依据就是这个数字 + 随后 `ep-lookup <path>`。
     */
    {
        jsdk_desc_info_t di;
        int ok = (jsdk_context_get_desc_info(a->ctx, &di) == JSDK_OK);

        if (ok) {
            count = di.endpoint_count;   /* 0 表示导入后端点表仍为空 → 报失败 */
            if (a->o.json) {
                cli_json_t j;
                cli_json_init(&j, a->out, 0);
                cli_json_str(&j, "file", path);
                cli_json_i64(&j, "bytes", (long long)got);
                cli_json_bool(&j, "imported", 1);
                cli_json_i64(&j, "endpoint_count", (long long)count);
                cli_json_bool(&j, "downloaded", 0);
                cli_json_finish(&j);
            } else {
                fprintf(a->out, "已从 %s 导入 %lu 字节，端点表 %u 条（未下载）\n",
                        path, (unsigned long)got, count);
                fprintf(a->out, "提示：正式用法是在启动时先 desc-import，"
                                "成功就不再下载；失败再走 configure()。\n");
            }
        }
    }
    return (count > 0u) ? CLI_EXIT_OK : CLI_EXIT_FAIL;
}

/* ==========================================================================
 * 写：write / save / set-node-id / watchdog / set-zero / reset
 * ======================================================================== */

/**
 * 参数写。
 *
 * ⚠ 本命令需要 `--yes`，虽然设计文档把 `write` 放在"配置"级里。
 *   理由：`write axis0.motor.config.gear_ratio 8` 会让同一个 MIT 命令的
 *   实际输出差一倍 —— 那足以让电机跳起来。凡是写设备的一律要 `--yes`，
 *   规则统一比"哪些参数危险"更可靠。
 */
static int cmd_write(cli_app_t *a)
{
    uint16_t ep_id = 0u;
    jsdk_ep_type_t type = JSDK_EP_JSON;
    uint8_t access = 0u;
    jsdk_value_t val;
    jsdk_status_t st;
    int rc = require_yes(a, "write");

    if (rc != 0) return rc;
    if (a->o.nargs < 2u) {
        fprintf(a->err, "jsdk-cli: write 需要 <path> <value>\n");
        return CLI_EXIT_USAGE;
    }

    /* 先查类型：客户给的文本要按端点真实位宽解析，超范围直接拒绝 */
    st = jsdk_endpoint_lookup(a->ctx, a->o.args[0], &ep_id, &type, &access);
    if (st != JSDK_OK) { cli_error(a, a->o.args[0], st); return CLI_EXIT_FAIL; }
    if (!(access & JSDK_EP_ACCESS_W)) {
        fprintf(a->err, "jsdk-cli: %s 不可写（access=%c%c）\n", a->o.args[0],
                (access & JSDK_EP_ACCESS_R) ? 'r' : '-', 'w');
        return CLI_EXIT_FAIL;
    }
    if (text_to_value(a->o.args[1], type, &val) != 0) {
        fprintf(a->err, "jsdk-cli: 值 \"%s\" 不是合法的 %s（超范围也算非法，"
                        "不静默截断）\n", a->o.args[1], jsdk_ep_type_string(type));
        return CLI_EXIT_FAIL;
    }

    st = jsdk_joint_param_set(a->joint, a->o.args[0], &val);
    if (st != JSDK_OK) { cli_error(a, "write", st); return CLI_EXIT_FAIL; }

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_str(&j, "path", a->o.args[0]);
        cli_json_i64(&j, "id", (long long)ep_id);
        cli_json_str(&j, "type", jsdk_ep_type_string(type));
        value_to_json(&j, "value", &val);
        cli_json_bool(&j, "written", 1);
        cli_json_str(&j, "persisted", "no (use save to persist)");
        cli_json_finish(&j);
    } else {
        fprintf(a->out, "已写入 %s = %s（%s，ep %u）\n", a->o.args[0], a->o.args[1],
                jsdk_ep_type_string(type), (unsigned)ep_id);
        fprintf(a->out, "注意：未落 Flash；需要持久化请再跑 `save`。\n");
    }
    return CLI_EXIT_OK;
}

static int cmd_save(cli_app_t *a)
{
    jsdk_status_t st;
    int rc = require_yes(a, "save");
    if (rc != 0) return rc;

    st = jsdk_joint_save_config(a->joint);
    if (st != JSDK_OK) { cli_error(a, "save", st); return CLI_EXIT_FAIL; }

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_bool(&j, "saved", 1);
        cli_json_finish(&j);
    } else {
        fprintf(a->out, "配置已保存到 Flash（并已读回校验）\n");
    }
    return CLI_EXIT_OK;
}

static int cmd_set_node_id(cli_app_t *a)
{
    uint32_t id;
    jsdk_status_t st;
    int rc = require_yes(a, "set-node-id");

    if (rc != 0) return rc;
    if (a->o.nargs < 1u) { fprintf(a->err, "jsdk-cli: set-node-id 需要 <N>\n"); return CLI_EXIT_USAGE; }
    if (parse_u32_arg(a->o.args[0], &id) != 0 || id == 0u || id > 254u) {
        fprintf(a->err, "jsdk-cli: 节点 ID 非法（1..254）：%s\n", a->o.args[0]);
        return CLI_EXIT_USAGE;
    }

    st = jsdk_joint_set_node_id(a->joint, (uint8_t)id, 1);
    if (st != JSDK_OK) { cli_error(a, "set-node-id", st); return CLI_EXIT_FAIL; }

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_i64(&j, "old_node", (long long)a->o.node);
        cli_json_i64(&j, "new_node", (long long)id);
        cli_json_bool(&j, "persisted", 1);
        cli_json_finish(&j);
    } else {
        fprintf(a->out, "节点 ID %u → %u（已落 Flash）\n", (unsigned)a->o.node, (unsigned)id);
        fprintf(a->out, "后续命令请用 --node %u\n", (unsigned)id);
    }
    return CLI_EXIT_OK;
}

static int cmd_watchdog(cli_app_t *a)
{
    uint32_t ms;
    jsdk_status_t st;
    int rc = require_yes(a, "watchdog");

    if (rc != 0) return rc;
    if (a->o.nargs < 1u) { fprintf(a->err, "jsdk-cli: watchdog 需要 <MS>\n"); return CLI_EXIT_USAGE; }
    if (parse_u32_arg(a->o.args[0], &ms) != 0) {
        fprintf(a->err, "jsdk-cli: 毫秒数非法：%s\n", a->o.args[0]);
        return CLI_EXIT_USAGE;
    }

    st = jsdk_joint_set_watchdog_ms(a->joint, ms);
    if (st != JSDK_OK) { cli_error(a, "watchdog", st); return CLI_EXIT_FAIL; }

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_i64(&j, "break_timeout_ms", (long long)ms);
        cli_json_bool(&j, "persisted", 0);
        cli_json_finish(&j);
    } else {
        fprintf(a->out, "break_timeout = %lu ms\n", (unsigned long)ms);
        if (ms == 0u) {
            fprintf(a->out, "⚠ 固件把 0 解释为 **100 ms**（0 不等于关闭）。"
                            "要真正放宽请写一个大值（如 65535）。\n");
        }
    }
    return CLI_EXIT_OK;
}

static int cmd_set_zero(cli_app_t *a)
{
    jsdk_status_t st;
    int rc = require_yes(a, "set-zero");
    if (rc != 0) return rc;

    st = jsdk_joint_set_zero_here(a->joint);
    if (st != JSDK_OK) { cli_error(a, "set-zero", st); return CLI_EXIT_FAIL; }

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_bool(&j, "zeroed", 1);
        cli_json_finish(&j);
    } else {
        fprintf(a->out, "当前位置已设为零点（未落 Flash，需 save 持久化）\n");
    }
    return CLI_EXIT_OK;
}

static int cmd_reset(cli_app_t *a)
{
    jsdk_status_t st;
    int rc = require_yes(a, "reset");
    if (rc != 0) return rc;

    st = jsdk_joint_reset_device(a->joint);
    if (st != JSDK_OK) { cli_error(a, "reset", st); return CLI_EXIT_FAIL; }

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_bool(&j, "reset", 1);
        cli_json_finish(&j);
    } else {
        fprintf(a->out, "设备已软复位（之后需要重新握手 / configure）\n");
    }
    return CLI_EXIT_OK;
}

/* ==========================================================================
 * 动作：calibrate / home / estop / mit
 * ======================================================================== */

static int cmd_calibrate(cli_app_t *a)
{
    jsdk_status_t st;
    int rc = require_yes(a, "calibrate");
    if (rc != 0) return rc;

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_str(&j, "action", "calibrate");
        cli_json_str(&j, "warning", "motor moves during full calibration");
        cli_json_finish(&j);
    } else {
        fprintf(a->out, "开始全标定（电机会动，耗时可能数秒）…\n");
    }

    st = jsdk_joint_calibrate(a->joint);
    if (st != JSDK_OK) { cli_error(a, "calibrate", st); return CLI_EXIT_FAIL; }

    if (!a->o.json) fprintf(a->out, "标定完成（状态已离开瞬时态）\n");
    return CLI_EXIT_OK;
}

static int cmd_home(cli_app_t *a)
{
    jsdk_status_t st;
    int rc = require_yes(a, "home");
    if (rc != 0) return rc;

    if (!a->o.json) fprintf(a->out, "开始回零…\n");

    st = jsdk_joint_home(a->joint);
    if (st != JSDK_OK) { cli_error(a, "home", st); return CLI_EXIT_FAIL; }

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_bool(&j, "homed", 1);
        cli_json_finish(&j);
    } else {
        fprintf(a->out, "回零完成\n");
    }
    return CLI_EXIT_OK;
}

static int cmd_estop(cli_app_t *a)
{
    jsdk_context_estop(a->ctx);

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_bool(&j, "estop_sent", 1);
        cli_json_finish(&j);
    } else {
        fprintf(a->out, "已广播 ESTOP(0xC0)——最高仲裁优先级，总线上的设备应进入"
                        "安全状态\n");
    }
    /* ESTOP 是安全动作，不需要 --yes：拒绝执行反而更危险 */
    return CLI_EXIT_OK;
}

/**
 * `mit`：唯一会驱动电机的命令。
 *
 * 三道闸：
 *   1. `--yes`（缺了直接拒绝）；
 *   2. `--hold`（**必需**，1..60 s）—— 到期自动 hold_position + disable，
 *      即使客户忘了 Ctrl-C 也不会一直跑；
 *   3. 执行前打印将发送的量程与限值，让客户在日志里能看到实际生效的边界。
 *
 * 收到 Ctrl-C 时先 hold_position 再 disable 再退出（顺序不能反：
 * 先停发控制帧会直接触发设备的 break_timeout 保护）。
 */
static int cmd_mit(cli_app_t *a)
{
    jsdk_joint_config_snapshot_t snap;
    uint32_t elapsed;
    unsigned period_ms;
    if (!a->o.yes) return require_yes(a, "mit");

    /* --- 第二道闸：--hold 必需且有上限 --- */
    if (a->o.hold_s < 0) {
        fprintf(a->err,
                "jsdk-cli: mit 必须同时给 --hold <秒>（1..60）。\n"
                "          它是唯一会驱动电机的命令，--hold 到期会自动 hold + disable。\n");
        return CLI_EXIT_REFUSED;
    }
    if (a->o.hold_s == 0 || a->o.hold_s > 60) {
        fprintf(a->err, "jsdk-cli: --hold 必须在 1..60 秒之间（给的是 %d）\n", a->o.hold_s);
        return CLI_EXIT_USAGE;
    }

    if (jsdk_joint_read_config_snapshot(a->joint, &snap) != JSDK_OK || !snap.valid) {
        fprintf(a->err, "jsdk-cli: 量程无效，拒绝驱动电机"
                        "（绝不能拿一个猜的量程去发 MIT 帧）\n");
        return CLI_EXIT_FAIL;
    }

    /* --- 第三道闸：把边界写出来 --- */
    fprintf(a->err,
            "jsdk-cli: MIT 将发送 pos=%.4f vel=%.4f kp=%.4f kd=%.4f tau=%.4f，"
            "持续 %d s\n"
            "          量程: pos=±%.4f vel=±%.4f tau=±%.4f kp=%.4f kd=%.4f "
            "gear=%.4f\n",
            a->o.have[0] ? a->o.pos : 0.0,
            a->o.have[1] ? a->o.vel : 0.0,
            a->o.have_stiff ? 0.0 : (a->o.have[2] ? a->o.kp : 0.0),
            a->o.have[3] ? a->o.kd : 0.0,
            a->o.have[4] ? a->o.tau : 0.0,
            a->o.hold_s,
            (double)snap.mit_max_pos, (double)snap.mit_max_vel,
            (double)snap.mit_max_torque, (double)snap.mit_max_kp,
            (double)snap.mit_max_kd, (double)snap.gear_ratio);
    if (a->o.have_stiff) {
        fprintf(a->err, "          --stiffness %.4f N·m/rad 将换算为线上 kp=%.4f\n",
                a->o.kp_stiff, a->o.kp_stiff * 2.0 * 3.14159265358979323846
                              / (double)snap.gear_ratio);
    }

    /* --- 使能 --- */
    jsdk_joint_request_enable(a->joint, JSDK_MODE_MIT);
    {
        unsigned spin = 0u;
        while (!jsdk_joint_is_enabled(a->joint) && spin < 4000u) {
            jsdk_context_poll(a->ctx, 0u);
            spin++;
        }
    }
    if (!jsdk_joint_is_enabled(a->joint)) {
        fprintf(a->err, "jsdk-cli: 使能失败 —— %s\n", jsdk_context_last_error(a->ctx));
        return CLI_EXIT_FAIL;
    }

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_str(&j, "action", "mit");
        cli_json_num(&j, "pos_rad", a->o.have[0] ? a->o.pos : 0.0);
        cli_json_num(&j, "vel_rad_s", a->o.have[1] ? a->o.vel : 0.0);
        cli_json_num(&j, "kp", a->o.have[2] ? a->o.kp : 0.0);
        cli_json_num(&j, "kd", a->o.have[3] ? a->o.kd : 0.0);
        cli_json_num(&j, "tau_Nm", a->o.have[4] ? a->o.tau : 0.0);
        cli_json_i64(&j, "hold_s", (long long)a->o.hold_s);
        cli_json_finish(&j);
    }

    period_ms = (unsigned)(1000 / a->o.rate_hz);
    if (period_ms == 0u) period_ms = 1u;
    elapsed = 0u;

    while (elapsed < (uint32_t)a->o.hold_s * 1000u) {
        if (jsdk_cli_stop_requested()) {
            fprintf(a->err, "jsdk-cli: 收到停止请求 —— 先 hold 再 disable\n");
            break;
        }
        jsdk_context_cycle_begin(a->ctx, 0u);

        if (a->o.have_stiff) {
            jsdk_joint_set_mit_stiffness(a->joint,
                                         a->o.have[0] ? a->o.pos : 0.0,
                                         a->o.have[1] ? a->o.vel : 0.0,
                                         a->o.kp_stiff,
                                         a->o.have[3] ? a->o.kd : 0.0,
                                         a->o.have[4] ? a->o.tau : 0.0);
        } else {
            jsdk_joint_set_mit(a->joint,
                               a->o.have[0] ? a->o.pos : 0.0,
                               a->o.have[1] ? a->o.vel : 0.0,
                               a->o.have[2] ? a->o.kp : 0.0,
                               a->o.have[3] ? a->o.kd : 0.0,
                               a->o.have[4] ? a->o.tau : 0.0);
        }
        jsdk_context_cycle_end(a->ctx);

        if (cli_sleep_ms(period_ms)) {
            fprintf(a->err, "jsdk-cli: 收到停止请求 —— 先 hold 再 disable\n");
            break;
        }
        elapsed += period_ms;
    }

    /* --- 安全收尾：顺序不可颠倒 --- */
    jsdk_joint_hold_position(a->joint);
    jsdk_context_poll(a->ctx, 0u);
    jsdk_joint_request_disable(a->joint);
    {
        unsigned spin = 0u;
        while (jsdk_joint_is_enabled(a->joint) && spin < 4000u) {
            jsdk_context_poll(a->ctx, 0u);
            spin++;
        }
    }

    if (!a->o.json) {
        fprintf(a->out, "MIT 结束：已 hold_position 并 disable（%lu ms 实际运行）\n",
                (unsigned long)elapsed);
    }
    return CLI_EXIT_OK;
}

/* ==========================================================================
 * 分发
 * ======================================================================== */

typedef struct {
    const char *name;
    int (*fn)(cli_app_t *a);
    int  needs_desc;     /**< 1 = 需要端点表（即要先下载/解析描述符） */
} cmd_t;

static const cmd_t CMDS[] = {
    /* 不需要端点表：整条命令不碰描述符（省 41 KB 流量） */
    { "scan",           cmd_scan,           0 },
    { "estop",          cmd_estop,          0 },
    /* 其余都要读参数 / 看端点 / 使能，因此需要端点表 */
    { "info",           cmd_info,           1 },
    { "health",         cmd_health,         1 },
    { "mon",            cmd_mon,            1 },
    { "read",           cmd_read,           1 },
    { "batch-read",     cmd_batch_read,     1 },
    { "dump-config",    cmd_dump_config,    1 },
    { "err",            cmd_err,            1 },
    { "hb-dump",        cmd_hb_dump,        1 },
    { "desc-info",      cmd_desc_info,      1 },
    { "ep-list",        cmd_ep_list,        1 },
    { "ep-lookup",      cmd_ep_lookup,      1 },
    { "desc-export",    cmd_desc_export,    1 },
    { "desc-import",    cmd_desc_import,    0 },   /* 缓存的意义就是"不下载" */
    { "write",          cmd_write,          1 },
    { "save",           cmd_save,           1 },
    { "set-node-id",    cmd_set_node_id,    1 },
    { "watchdog",       cmd_watchdog,       1 },
    { "set-zero",       cmd_set_zero,       1 },
    { "reset",          cmd_reset,          1 },
    { "calibrate",      cmd_calibrate,      1 },
    { "home",           cmd_home,           1 },
    { "mit",            cmd_mit,            1 }
};


static const cmd_t *find_cmd(const char *name)
{
    unsigned i;
    if (!name) return NULL;
    for (i = 0u; i < sizeof CMDS / sizeof CMDS[0]; ++i) {
        if (strcmp(CMDS[i].name, name) == 0) return &CMDS[i];
    }
    return NULL;
}

int jsdk_cli_run(int argc, char **argv, FILE *out, FILE *err)
{
    cli_app_t a;
    const cmd_t *c;
    int rc;

    memset(&a, 0, sizeof a);
    a.out = out;
    a.err = err;

    rc = cli_opts_parse(&a.o, argc, argv, err);
    if (rc == 1) return CLI_EXIT_OK;       /* --help：用法已打印 */
    if (rc != 0) return CLI_EXIT_USAGE;

    c = find_cmd(a.o.sub);
    if (!c) {
        fprintf(err, "jsdk-cli: 未知子命令 %s（--help 看用法）\n", a.o.sub);
        return CLI_EXIT_USAGE;
    }

    if (!a.o.quiet && !a.o.json && a.o.verbose) {
        fprintf(err, "jsdk-cli: %s\n", a.o.sub);
    }

    /* cli_open 自己区分"用法/参数问题（2）"与"运行时失败（1）"——
       早先这里无条件返回 1，于是 `--if nosuchbus` 这种明显的用法错误
       退出码成了 1，脚本里没法区分"配错了"和"跑失败了"。 */
    rc = cli_open(&a);
    if (rc != 0) { cli_close(&a); return rc; }

    if (c->needs_desc) {
        if (cli_load_desc(&a) != 0) { cli_close(&a); return CLI_EXIT_FAIL; }
    }

    rc = c->fn(&a);

    cli_close(&a);
    return rc;
}
