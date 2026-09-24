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
#include "cli_text.h"

/* ==========================================================================
 * 小工具
 * ======================================================================== */

static void out_kv(cli_app_t *a, const char *k, const char *fmt, ...)
{
    va_list ap;

    if (a->o.json) return;
    cli_fprintf(a->out, "  %-22s ", k);
    va_start(ap, fmt);
    vfprintf(a->out, fmt, ap);
    va_end(ap);
    cli_fputc('\n', a->out);
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
    cli_fprintf(a->err,
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

/** 两个值是否“同一个值”（浮点给 1e-6 相对容差，免得因显示精度误报）。 */
static int value_same(const jsdk_value_t *a, const jsdk_value_t *b)
{
    double x, y, m;

    if (a->type != b->type) return 0;
    switch (a->type) {
    case JSDK_EP_BOOL: return (a->v.boolean != 0) == (b->v.boolean != 0);
    case JSDK_EP_U8:   return a->v.u8  == b->v.u8;
    case JSDK_EP_I8:   return a->v.i8  == b->v.i8;
    case JSDK_EP_U16:  return a->v.u16 == b->v.u16;
    case JSDK_EP_I16:  return a->v.i16 == b->v.i16;
    case JSDK_EP_U32:  return a->v.u32 == b->v.u32;
    case JSDK_EP_I32:  return a->v.i32 == b->v.i32;
    case JSDK_EP_U64:  return a->v.u64 == b->v.u64;
    case JSDK_EP_I64:  return a->v.i64 == b->v.i64;
    case JSDK_EP_F32:  x = (double)a->v.f32; y = (double)b->v.f32; break;
    case JSDK_EP_F64:  x = a->v.f64;         y = b->v.f64;         break;
    default:           return 0;
    }
    if (x != x || y != y) return 0;              /* NaN 视为不同（不想拿它当“成功”） */
    m = (x < 0.0 ? -x : x);
    if ((y < 0.0 ? -y : y) > m) m = (y < 0.0 ? -y : y);
    if (m < 1.0) m = 1.0;
    return ((x - y) < 0.0 ? (y - x) : (x - y)) <= 1e-6 * m;
}

/** 值的可读文本（供人看的输出用）。 */

/** 按端点类型把文本解析成 jsdk_value_t（`write` 用）。 */
/**
 * 把命令行文本按端点类型解析成值。
 *
 * @return 0 = 成功；
 *         **-1 = 文本本身就不是合法的数/布尔字**（用法错 → 退出码 2）；
 *         **-2 =  ！解析出来了，但超出该类型的值域**（运行期错 → 退出码 1）。
 *
 * ⚠ 两种错必须分开：Python 版把“不是数字”归在**参数层**（退出码 2），
 *   把“超范围”留给 SDK（退出码 1）。真机实测曾经出现 C=1 / Py=2 的不一致，
 *   而文档写的是“两版同一份退出码契约”。
 */
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
        case JSDK_EP_I8:  if (s < -128LL   || s > 127LL)   return -2; v->v.i8  = (int8_t)s;  return 0;
        case JSDK_EP_I16: if (s < -32768LL || s > 32767LL) return -2; v->v.i16 = (int16_t)s; return 0;
        case JSDK_EP_I32: if (s < -2147483648LL || s > 2147483647LL) return -2;
                          v->v.i32 = (int32_t)s; return 0;
        case JSDK_EP_I64: v->v.i64 = (int64_t)s; return 0;
        case JSDK_EP_U8: case JSDK_EP_U16: case JSDK_EP_U32: case JSDK_EP_U64:
            return -2;                                  /* 无符号却给了负数 */
        default: return -2;
        }
    } else {
        unsigned long long u = strtoull(text, &end, 0);
        if (!end || *end != '\0') return -1;
        switch (type) {
        case JSDK_EP_U8:  if (u > 255ull)   return -2; v->v.u8  = (uint8_t)u;  return 0;
        case JSDK_EP_U16: if (u > 65535ull) return -2; v->v.u16 = (uint16_t)u; return 0;
        case JSDK_EP_U32: if (u > 4294967295ull) return -2; v->v.u32 = (uint32_t)u; return 0;
        case JSDK_EP_U64: v->v.u64 = (uint64_t)u; return 0;
        case JSDK_EP_I8:  if (u > 127ull)   return -2; v->v.i8  = (int8_t)u;  return 0;
        case JSDK_EP_I16: if (u > 32767ull) return -2; v->v.i16 = (int16_t)u; return 0;
        case JSDK_EP_I32: if (u > 2147483647ull) return -2; v->v.i32 = (int32_t)u; return 0;
        case JSDK_EP_I64: if (u > 9223372036854775807ull) return -2;
                          v->v.i64 = (int64_t)u; return 0;
        default: return -2;
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
        cli_fprintf(a->out, "发现 %u 个节点（被动 200 ms + 主动探测 1..%u）:\n",
                found, a->o.max_probe);
        for (i = 0u; i < found; ++i) {
            cli_fprintf(a->out, "  node %u\n", (unsigned)ids[i]);
        }
        if (found == 0u) {
            cli_fprintf(a->out, "  （无）—— 逐条检查：\n"
                            "    1. 总线是否 up；波特率 / FD / BRS 是否与设备 "
                            "can.config.baud_rate 一致（协议无运行时协商）\n"
                            "    2. 是否有 120Ω 终端电阻、CAN_H/L 是否接反\n"
                            "    3. 设备**必须先收到过一帧**才学到主站地址（地址为 0 时"
                            "设备完全不回复）\n");
            if (a->o.max_probe == 0u) {
                cli_fprintf(a->out, "    4. 本次是 `--probe 0`（仅被动听心跳）：首次连接"
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
        cli_fprintf(a->out, "节点 %u:\n", (unsigned)a->o.node);
        out_kv(a, "hw_version", "%lu", (unsigned long)info.hw_version);
        out_kv(a, "fw_version", "%lu", (unsigned long)info.fw_version);
        out_kv(a, "serial", "%llu", (unsigned long long)info.serial);
        out_kv(a, "device_mode", "%s", info.classic ? "Classic" : "FD");
    }
    /*
     * ⚠ Classic 下 `serial:0` 是**协议如此**，不是"没读出来"：真机实测（1 Mbps
     *   Classic 的设备）`0x46` 的**响应只有 8 字节**，装得下 `hw u32 + fw u32`，
     *   **没有 serial 字段**（见 PROTOCOL_NOTES §3 的 `0x46` 行）。用户很容易把
     *   这个 0 当成故障，所以这里直接说清 + 给出正确取法。
     */
    if (info.serial == 0u && info.classic) {
        cli_fprintf(a->err, "jsdk-cli: 提示：Classic 下 `0x46` 的响应只有 hw+fw"
                        "（8 字节，协议如此），所以 `serial` 恒为 0；"
                        "要序列号请用 `read serial_number`（描述符端点）。\n");
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

    cli_fprintf(a->out, "节点 %u 健康快照:\n", (unsigned)a->o.node);
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

    cli_fprintf(a->out, "总线:\n");
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

/* --------------------------------------------------------------------------
 * CSV 列定义（`mon`）
 * ------------------------------------------------------------------------
 * 列：时间 → 物理量 → 状态 → 诊断计数。取的是 cyberbeast_tool.py 采集脚本里
 * 共有的量，方便两边对着看。
 *
 * ⚠ 这份表头**两版 CLI 共用一份契约**：`jsdk-cli mon --csv` 写到 stdout 的
 *   第一行、`--csv-file` 文件的第一行、以及 `python -m jsdk_can mon --csv`
 *   输出的第一行，必须是**逐字节相同**的（有跨版本对拍用例钉住）。
 */
#define CLI_MON_CSV_HEADER \
    "t_ms,node,pos_rad,vel_rad_s,current_A,torque_Nm," \
    "t_motor_C,t_fet_C,vbus_V,ibus_A,axis_state,mode,err_code," \
    "hb_error,age_ms,tx_frames,tx_rejected\n"

/** 写一行 CSV（stdout 与文件用同一个实现，避免两边列数漂移）。 */
static void mon_csv_row(FILE *f, const jsdk_joint_feedback_t *fb,
                        uint32_t elapsed_ms, unsigned node)
{
    cli_fprintf(f, "%lu,%u,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.3f,%.3f,%s,%s,%u,%u,%lu,%lu,%lu\n",
            (unsigned long)elapsed_ms, node,
            fb->pos, fb->vel, fb->current_A, fb->torque_Nm,
            fb->t_motor_C, fb->t_fet_C, fb->vbus_V, fb->ibus_A,
            jsdk_axis_state_string(fb->axis_state), cli_mode_name(fb->mode),
            (unsigned)fb->err_code, (unsigned)fb->hb_error,
            (unsigned long)fb->age_ms, (unsigned long)fb->tx_frames,
            (unsigned long)fb->tx_rejected);
}

static int cmd_mon(cli_app_t *a)
{
    uint32_t elapsed = 0u;
    unsigned period_ms = (unsigned)(1000 / a->o.rate_hz);
    FILE *csv = NULL;
    int first = 1;

    /* `--csv` = 格式开关（写到 stdout）；`--csv-file` = 额外落盘（可选）。
       ⚠ 旧版 `--csv` 是要文件名的，改名的理由见 cli_app.c（会静默写错文件名）。 */
    if (a->o.csv_file) {
        csv = fopen(a->o.csv_file, "w");
        if (!csv) {
            cli_fprintf(a->err, "jsdk-cli: 打不开 --csv-file 文件 %s\n", a->o.csv_file);
            return CLI_EXIT_FAIL;
        }
        cli_fprintf(csv, CLI_MON_CSV_HEADER);
    }
    if (a->o.csv) cli_fprintf(a->out, CLI_MON_CSV_HEADER);

    /* JSON 模式下 mon 输出 NDJSON（每行一个对象），便于流式管道消费。 */

    while (cli_should_continue(a, elapsed)) {
        jsdk_joint_feedback_t fb;
        jsdk_status_t st;

        jsdk_context_poll(a->ctx, 0u);

        st = jsdk_joint_get_feedback(a->joint, &fb);
        if (st == JSDK_OK) {
            if (a->o.csv) {
                mon_csv_row(a->out, &fb, elapsed, (unsigned)a->o.node);
            } else if (a->o.json) {
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
                    cli_fprintf(a->out, "%9s %7s %9s %9s %7s %7s %7s  %-12s %s\n",
                            "t_ms", "pos_rad", "vel_rad/s", "tau_Nm", "tMot", "vbus",
                            "age_ms", "state", "err");
                }
                cli_fprintf(a->out, "%9lu %7.3f %9.3f %9.3f %7.1f %7.1f %7lu  %-12s %u\n",
                        (unsigned long)elapsed, fb.pos, fb.vel, fb.torque_Nm,
                        fb.t_motor_C, fb.vbus_V, (unsigned long)fb.age_ms,
                        jsdk_axis_state_string(fb.axis_state), (unsigned)fb.err_code);
            }
            if (csv) {
                mon_csv_row(csv, &fb, elapsed, (unsigned)a->o.node);
                fflush(csv);
            }
        }
        first = 0;

        if (period_ms > 0u && cli_sleep_ms(period_ms)) break;
        elapsed += period_ms;
    }

    if (csv) fclose(csv);
    if (!a->o.json && !a->o.quiet) cli_fprintf(a->out, "（mon 结束）\n");
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

    if (!path) { cli_fprintf(a->err, "jsdk-cli: read 需要 <path>\n"); return CLI_EXIT_USAGE; }

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
        cli_fprintf(a->out, "%s = %s (%s)\n", path, text, jsdk_ep_type_string(v.type));
    }
    return CLI_EXIT_OK;
}

static int cmd_batch_read(cli_app_t *a)
{
    jsdk_param_req_t reqs[8];
    unsigned n, i;
    jsdk_status_t st;
    char text[64];

    if (a->o.nargs == 0u) { cli_fprintf(a->err, "jsdk-cli: batch-read 需要 <path>...\n"); return CLI_EXIT_USAGE; }
    if (a->o.nargs > 8u)   { cli_fprintf(a->err, "jsdk-cli: 一次最多 8 条（超了请分批）\n"); return CLI_EXIT_USAGE; }

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
                cli_fprintf(a->out, "  %-52s = %s\n", reqs[i].path, text);
            } else {
                cli_fprintf(a->out, "  %-52s ! %s\n", reqs[i].path,
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
        cli_fprintf(a->out, "节点 %u 配置快照 (valid=%d):\n", (unsigned)a->o.node, s.valid);
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
               (s.break_timeout_ms == 0u)
                   ? "  ← 0 = 设备侧协议超时已禁用（不是 100 ms）" : "");
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

        cli_fprintf(a->out, "节点 %u 错误明细:\n", (unsigned)a->o.node);
        out_kv(a, "mit_err", "%u (%s)", (unsigned)f.mit_err,
               jsdk_joint_error_string(f.mit_err));
        out_kv(a, "hb_flags", "0x%02X", (unsigned)f.hb_flags);
        for (i = 0u; i < 6u; ++i) {
            unsigned bit = 0u;
            const char *first = jsdk_axis_error_first(vals[i], &bit);
            cli_fprintf(a->out, "  %-22s 0x%08lX  %s\n", names[i],
                    (unsigned long)vals[i],
                    first ? first : "无错误");
            if (first) cli_fprintf(a->out, "  %-22s           bit %u\n", "", bit);
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
            cli_fprintf(a->out, "src=%u len=%u  bytes: %s\n", src, (unsigned)f->len, hex);
            if (f->len >= 2u) {
                unsigned b0 = f->data[0];
                unsigned b1 = f->data[1];
                cli_fprintf(a->out, "    b0 life=%u%u%u%u err=0x%X   b1 state=%u%u%u%u ctrl=%u\n",
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
        cli_fprintf(a->out,
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
        cli_fprintf(a->out, "描述符:\n");
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
        cli_fprintf(c->a->out, "  %5u  %-3s  %-14s %s\n", (unsigned)ep_id, acc,
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
        cli_fprintf(a->out, "%5s  %-3s  %-14s %s\n", "id", "acc", "type", "path");
        st = jsdk_endpoint_enumerate(a->ctx, ep_list_visit, &c);
        cli_fprintf(a->out, "共 %u 个端点%s%s\n", c.n,
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

    if (!path) { cli_fprintf(a->err, "jsdk-cli: ep-lookup 需要 <path>\n"); return CLI_EXIT_USAGE; }

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
        cli_fprintf(a->out, "%s\n  id=%u type=%s access=%c%c\n", path, (unsigned)ep_id,
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

    if (!path) { cli_fprintf(a->err, "jsdk-cli: desc-export 需要 <file>\n"); return CLI_EXIT_USAGE; }

    cap = jsdk_desc_export_max_size(a->ctx);
    buf = calloc(1u, cap ? cap : 1u);
    if (!buf) { cli_fprintf(a->err, "jsdk-cli: 内存不足（%u 字节）\n", (unsigned)cap); return CLI_EXIT_FAIL; }

    st = jsdk_context_desc_export(a->ctx, buf, cap, &len);
    if (st != JSDK_OK) { free(buf); cli_error(a, "desc-export", st); return CLI_EXIT_FAIL; }

    f = fopen(path, "wb");
    if (!f) {
        free(buf);
        cli_fprintf(a->err, "jsdk-cli: 写不了 %s\n", path);
        return CLI_EXIT_FAIL;
    }
    if (fwrite(buf, 1u, len, f) != len) {
        fclose(f); free(buf);
        cli_fprintf(a->err, "jsdk-cli: 写 %s 不完整\n", path);
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
        cli_fprintf(a->out, "已导出 %lu 字节 → %s\n", (unsigned long)len, path);
        cli_fprintf(a->out, "提示：缓存与 retain/filter_paths/SDK 格式版本绑定，"
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

    if (!path) { cli_fprintf(a->err, "jsdk-cli: desc-import 需要 <file>\n"); return CLI_EXIT_USAGE; }

    f = fopen(path, "rb");
    if (!f) { cli_fprintf(a->err, "jsdk-cli: 读不了 %s\n", path); return CLI_EXIT_FAIL; }
    if (fseek(f, 0L, SEEK_END) != 0) { fclose(f); return CLI_EXIT_FAIL; }
    sz = ftell(f);
    if (sz <= 0) { fclose(f); cli_fprintf(a->err, "jsdk-cli: %s 是空文件\n", path); return CLI_EXIT_FAIL; }
    rewind(f);
    buf = calloc(1u, (size_t)sz);
    if (!buf) { fclose(f); cli_fprintf(a->err, "jsdk-cli: 内存不足\n"); return CLI_EXIT_FAIL; }
    got = fread(buf, 1u, (size_t)sz, f);
    fclose(f);
    if (got != (size_t)sz) {
        free(buf);
        cli_fprintf(a->err, "jsdk-cli: 读 %s 不完整\n", path);
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
                cli_fprintf(a->out, "已从 %s 导入 %lu 字节，端点表 %u 条（未下载）\n",
                        path, (unsigned long)got, count);
                cli_fprintf(a->out, "提示：正式用法是在启动时先 desc-import，"
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
/**
 * 写进去就被设备**消费掉**的端点：读回值必然和写进去的不一样。
 *
 * 固件 `axis.cpp:514` 在控制环拿到请求后立刻 `requested_state_ = AXIS_STATE_UNDEFINED;`
 * （0）—— 所以 `write axis0.requested_state 3` 之后读回几乎总是 0。
 * 拿读回值判“是否接受”在这里就是误报；验证要走 `current_state`
 * （`calibrate` / `home` / `state` 就是这么做的）。
 */
static int ep_write_consumed(const char *path)
{
    return path != NULL && strcmp(path, "axis0.requested_state") == 0;
}

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
        cli_fprintf(a->err, "jsdk-cli: write 需要 <path> <value>\n");
        return CLI_EXIT_USAGE;
    }

    /* 先查类型：客户给的文本要按端点真实位宽解析，超范围直接拒绝 */
    st = jsdk_endpoint_lookup(a->ctx, a->o.args[0], &ep_id, &type, &access);
    if (st != JSDK_OK) { cli_error(a, a->o.args[0], st); return CLI_EXIT_FAIL; }
    if (!(access & JSDK_EP_ACCESS_W)) {
        /* ⚠ 第二个字符以前是**写死的 'w'**，于是一个只读端点会显示成 `access=rw`
           （“明明说是 rw 却不让我写”）。 */
        cli_fprintf(a->err, "jsdk-cli: %s 不可写（access=%c%c）\n", a->o.args[0],
                (access & JSDK_EP_ACCESS_R) ? 'r' : '-',
                (access & JSDK_EP_ACCESS_W) ? 'w' : '-');
        return CLI_EXIT_FAIL;
    }
    switch (text_to_value(a->o.args[1], type, &val)) {
    case 0:
        break;
    case -1:      /* 文本就不是个数/布尔字 → 用法错（与 Python 版一致） */
        cli_fprintf(a->err, "jsdk-cli: 值 \"%s\" 不是合法的 %s 文本\n",
                    a->o.args[1], jsdk_ep_type_string(type));
        return CLI_EXIT_USAGE;
    default:      /* 解析出来了但超出该端点类型的值域 → 运行期错 */
        cli_fprintf(a->err, "jsdk-cli: 值 \"%s\" 超出 %s 的值域（不静默截断）\n",
                    a->o.args[1], jsdk_ep_type_string(type));
        return CLI_EXIT_FAIL;
    }

    st = jsdk_joint_param_set(a->joint, a->o.args[0], &val);
    if (st != JSDK_OK) { cli_error(a, "write", st); return CLI_EXIT_FAIL; }

    if (ep_write_consumed(a->o.args[0])) {
        /* 不复读（复读必是 0，会误判成“未被接受”）；也不要谎称已确认。 */
        if (a->o.json) {
            cli_json_t j;
            cli_json_init(&j, a->out, 0);
            cli_json_str(&j, "path", a->o.args[0]);
            cli_json_i64(&j, "id", (long long)ep_id);
            cli_json_str(&j, "type", jsdk_ep_type_string(type));
            value_to_json(&j, "requested", &val);
            cli_json_null(&j, "value");
            cli_json_null(&j, "verified");
            cli_json_str(&j, "note", "写进去就被状态机消费；用 `state` 看 current_state");
            cli_json_str(&j, "persisted", "not applicable");
            cli_json_finish(&j);
        } else {
            cli_fprintf(a->out, "已写入 %s = %s（%s，ep %u）\n",
                        a->o.args[0], a->o.args[1], jsdk_ep_type_string(type),
                        (unsigned)ep_id);
            cli_fprintf(a->out, "  该端点写进去就被状态机消费（固件立刻复位为 0），"
                                "无法用读回值判生效\n");
            cli_fprintf(a->out, "  要看结果请用 `state`（读 current_state）或 "
                                "`calibrate` / `home`。\n");
        }
        return CLI_EXIT_OK;
    }

    /* ⚠⚠ **写后必须读回**：设备可能**静默丢弃**整帧（真发生过 —— 参数写帧不足 8 字节时，
       固件 `cmd_param_write()` 首句直接 return，连 ACK 都不回），
       而以前这里直接把“请求值”当成“已写入”打印 —— 说得比知道的多。 */
    {
        jsdk_value_t  back;
        char          txt[64];
        int           have = (jsdk_joint_param_get(a->joint, a->o.args[0], &back)
                              == JSDK_OK);
        int           same = have ? value_same(&val, &back) : 0;

        if (same) value_to_text(&back, txt, sizeof txt);
        else      snprintf(txt, sizeof txt, "?");

        if (a->o.json) {
            cli_json_t j;
            cli_json_init(&j, a->out, 0);
            cli_json_str(&j, "path", a->o.args[0]);
            cli_json_i64(&j, "id", (long long)ep_id);
            cli_json_str(&j, "type", jsdk_ep_type_string(type));
            value_to_json(&j, "requested", &val);
            if (have) value_to_json(&j, "value", &back);
            else      cli_json_null(&j, "value");
            cli_json_bool(&j, "verified", same);
            /* ⚠ 退出码必须由**判定**决定，不能只在人读分支里返回：
               否则脚本拿 `--json` 时会把“写丢了”当成功（rc=0）。 */
            if (have && !same) cli_json_str(&j, "error", "not_accepted");
            cli_json_str(&j, "persisted", "no (use save to persist)");
            cli_json_finish(&j);
        } else if (!have) {
            /* 读不回来 ≠ 没写成功：改 `node_id` 这类参数会**改变寻址**，
               读完自然超时。这里只警告，不改退出码。 */
            cli_fprintf(a->out, "已写入 %s = %s（%s，ep %u）\n",
                        a->o.args[0], a->o.args[1], jsdk_ep_type_string(type),
                        (unsigned)ep_id);
            cli_fprintf(a->out, "  写入已发出，但**读不回来**，无法确认生效"
                                "（若改的是 node_id 类参数，这是正常的）\n");
        } else if (!same) {
            value_to_text(&back, txt, sizeof txt);
            cli_fprintf(a->err,
                "jsdk-cli: 写入未被设备接受：写 %s 后读回 %s（设备可能丢弃了该帧）\n",
                a->o.args[1], txt);
        } else {
            cli_fprintf(a->out, "已写入并读回确认：%s = %s（%s，ep %u）\n",
                        a->o.args[0], txt, jsdk_ep_type_string(type),
                        (unsigned)ep_id);
            cli_fprintf(a->out, "注意：未落 Flash；需要持久化请再跑 `save`。\n");
        }

        /* 读回值与写入值不一致 = 设备没接受这一帧 → 非零退出（JSON/人读一致） */
        if (have && !same) return CLI_EXIT_FAIL;
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
        cli_fprintf(a->out, "配置已保存到 Flash（并已读回校验）\n");
    }
    return CLI_EXIT_OK;
}

static int cmd_set_node_id(cli_app_t *a)
{
    uint32_t id;
    jsdk_status_t st;
    int rc = require_yes(a, "set-node-id");

    if (rc != 0) return rc;
    if (a->o.nargs < 1u) { cli_fprintf(a->err, "jsdk-cli: set-node-id 需要 <N>\n"); return CLI_EXIT_USAGE; }
    if (parse_u32_arg(a->o.args[0], &id) != 0 || id == 0u || id > 254u) {
        cli_fprintf(a->err, "jsdk-cli: 节点 ID 非法（1..254）：%s\n", a->o.args[0]);
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
        cli_fprintf(a->out, "节点 ID %u → %u（已落 Flash）\n", (unsigned)a->o.node, (unsigned)id);
        cli_fprintf(a->out, "后续命令请用 --node %u\n", (unsigned)id);
    }
    return CLI_EXIT_OK;
}

static int cmd_watchdog(cli_app_t *a)
{
    uint32_t ms;
    uint32_t raw = 0u;
    int raw_ok;
    jsdk_status_t st;
    jsdk_joint_config_snapshot_t s;
    int verified;
    int rc = require_yes(a, "watchdog");

    if (rc != 0) return rc;
    if (a->o.nargs < 1u) { cli_fprintf(a->err, "jsdk-cli: watchdog 需要 <MS>\n"); return CLI_EXIT_USAGE; }
    if (parse_u32_arg(a->o.args[0], &ms) != 0) {
        cli_fprintf(a->err, "jsdk-cli: 毫秒数非法：%s\n", a->o.args[0]);
        return CLI_EXIT_USAGE;
    }

    st = jsdk_joint_set_watchdog_ms(a->joint, ms);
    if (st != JSDK_OK) { cli_error(a, "watchdog", st); return CLI_EXIT_FAIL; }

    /* 校验结论只认标志位：真机上该端点读回恒为 0（F28），
       所以“写成功”不能当作“已武装”。
       ⚠ 这里**再独立读一次设备**，而不是回显 SDK 内部记住的写入值 ——
         否则 JSON 会在“写 250、设备读回 0”时说 “device_reports_ms: 250”，
         那就又变成“说得比知道的多了”。 */
    {
        jsdk_joint_feedback_t fb;
        jsdk_value_t v;
        memset(&fb, 0, sizeof fb);
        (void)jsdk_joint_get_feedback(a->joint, &fb);
        verified = ((fb.status_flags & JSDK_JF_WATCHDOG_UNVERIFIED) == 0u);

        raw_ok = 0;
        if (jsdk_joint_param_get(a->joint, "can.config.break_timeout", &v) == JSDK_OK) {
            switch (v.type) {
            case JSDK_EP_U8:  raw = v.v.u8;  raw_ok = 1; break;
            case JSDK_EP_U16: raw = v.v.u16; raw_ok = 1; break;
            case JSDK_EP_U32: raw = v.v.u32; raw_ok = 1; break;
            default: break;
            }
        }
    }
    if (jsdk_joint_read_config_snapshot(a->joint, &s) != JSDK_OK) {
        s.break_timeout_ms = ms;
        s.valid = 0;
    }

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_i64(&j, "ms", (long long)ms);
        if (raw_ok) cli_json_i64(&j, "device_reports_ms", (long long)raw);
        else       cli_json_null(&j, "device_reports_ms");
        cli_json_bool(&j, "verified", verified);
        cli_json_bool(&j, "disabled", ms == 0u);
        cli_json_finish(&j);
    } else {
        cli_fprintf(a->out, "写入 break_timeout = %lu ms\n", (unsigned long)ms);
        if (raw_ok)
            cli_fprintf(a->out, "  设备独立读回：%lu ms\n", (unsigned long)raw);
        if (ms == 0u) {
            cli_fprintf(a->out, "  0 = **关闭**设备侧协议级超时检测"
                                "（最新固件语义，不等于 100 ms）\n");
        } else if (!verified) {
            /* 不用 U+26A0：CP936 表示不了它，中文控制台上会变成 '?'
               （“输出文本必须在 CP936 里可表示”是 cli_text_lint 的静态检查）。 */
            cli_fprintf(a->out, "  写入已接受，但**读回校验未成功**："
                                "无法确认保护已武装\n");
            cli_fprintf(a->out, "  原因：本固件该端点读回恒为 0（= 禁用）→ "
                                "FIRMWARE_ISSUES F28\n");
        } else {
            cli_fprintf(a->out, "  已读回确认\n");
        }
        cli_fprintf(a->out, "保护只在设备收到过控制类帧后武装；"
                            "纯 CURRENT(0x04) 客户端武装不了（F19）\n");
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
        cli_fprintf(a->out, "当前位置已设为零点（未落 Flash，需 save 持久化）\n");
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
        cli_fprintf(a->out, "设备已软复位（之后需要重新握手 / configure）\n");
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
        cli_fprintf(a->out, "开始全标定（电机会动，可能数十秒）…\n");
    }

    st = jsdk_joint_calibrate(a->joint);
    if (st != JSDK_OK) { cli_error(a, "calibrate", st); return CLI_EXIT_FAIL; }

    /*
     * ⚠ **状态跑完了 ≠ 标定生效**。固件在电机标定/编码器偏置各会置一个
     *   `pre_calibrated`，而 `check_pre_calibrated()` 又会在编码器没就绪时把它
     *   清回 false —— 所以结束之后必须**读回两个标志**才知道到底落上了没有
     *   （真机实测踩过：“流程跑完了”但 `encoder.pre_calibrated` 仍是 false）。
     */
    {
        jsdk_value_t mv, ev;
        int have_m = (jsdk_joint_param_get(a->joint,
                        "axis0.motor.config.pre_calibrated", &mv) == JSDK_OK);
        int have_e = (jsdk_joint_param_get(a->joint,
                        "axis0.encoder.config.pre_calibrated", &ev) == JSDK_OK);
        int m_ok = have_m && mv.v.boolean != 0;
        int e_ok = have_e && ev.v.boolean != 0;

        if (a->o.json) {
            cli_json_t j;
            cli_json_init(&j, a->out, 0);
            cli_json_bool(&j, "calibrated", 1);
            cli_json_bool(&j, "motor_pre_calibrated", have_m ? m_ok : 0);
            cli_json_bool(&j, "encoder_pre_calibrated", have_e ? e_ok : 0);
            cli_json_str(&j, "persisted", "no (use save to persist)");
            cli_json_finish(&j);
        } else {
            cli_fprintf(a->out, "标定完成：motor.pre_calibrated=%s encoder.pre_calibrated=%s\n",
                        have_m ? (m_ok ? "true" : "false") : "?",
                        have_e ? (e_ok ? "true" : "false") : "?");
            if (!e_ok) {
                cli_fprintf(a->out, "  [注意] 编码器偏置没落上（pre_calibrated=false）："
                                    "闭环使能会被拒；先看 `err`，必要时重跑或查编码器\n");
            }
            cli_fprintf(a->out, "  注意：未落 Flash；需要持久化请再跑 `save`。\n");
        }
    }
    return CLI_EXIT_OK;
}

static int cmd_home(cli_app_t *a)
{
    jsdk_status_t st;
    int rc = require_yes(a, "home");
    if (rc != 0) return rc;

    if (!a->o.json) cli_fprintf(a->out, "开始回零…\n");

    st = jsdk_joint_home(a->joint);
    if (st != JSDK_OK) { cli_error(a, "home", st); return CLI_EXIT_FAIL; }

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_bool(&j, "homed", 1);
        cli_json_finish(&j);
    } else {
        cli_fprintf(a->out, "回零完成\n");
    }
    return CLI_EXIT_OK;
}

/**
 * fault-reset：清故障（`STOP_MOTOR` → `CLEAR_ERRORS(0x65)` → 等错误位归零）。
 *
 * ⚠ 为什么必须单独有一条命令：`estop` / 固件的 `FAULT_ALERT(0xC1)` 会置
 *   `ERROR_ESTOP_REQUESTED`，而**写 `axis0.error = 0` 清不掉它**（实测写后同进程
 *   读回 0，~120 ms 后再读又变回 2048）。不给出这条恢复路径，客户遇到锁死的关节
 *   就只能断电重启 —— 而且 `calibrate` / `home` / `enable` 全都会被它挡住。
 *   这是**唯一**能在软件里恢复的路径（失败时还有 `reset-device` 软复位）。
 */
static int cmd_fault_reset(cli_app_t *a)
{
    unsigned spin = 0u;
    int      rc = require_yes(a, "fault-reset");

    if (rc != 0) return rc;

    jsdk_joint_request_fault_reset(a->joint);

    /* 非阻塞请求：靠跑周期推进（和其它命令同一套 advance_seq 语义） */
    while (spin < 3000u && jsdk_joint_is_fault(a->joint)) {
        jsdk_context_poll(a->ctx, 0u);
        spin++;
    }

    if (jsdk_joint_is_fault(a->joint)) {
        if (a->o.json) {
            cli_json_t j;
            cli_json_init(&j, a->out, 0);
            cli_json_bool(&j, "cleared", 0);
            cli_json_str(&j, "error", "fault still present");
            cli_json_str(&j, "hint",
                         "总线上可能还有节点在广播 ESTOP(0xC0)/FAULT_ALERT(0xC1)；"
                         "也可试 `reset-device`（软复位）或断电重启");
            cli_json_finish(&j);
        } else {
            cli_fprintf(a->err, "jsdk-cli: 故障未清除 —— %s\n",
                        jsdk_context_last_error(a->ctx));
            cli_fprintf(a->err, "  提示：总线上可能还有节点在广播 ESTOP(0xC0)/"
                                "FAULT_ALERT(0xC1)；\n");
            cli_fprintf(a->err, "        也可试 `reset-device`（软复位）或断电重启。\n");
        }
        return CLI_EXIT_FAIL;
    }

    if (a->o.json) {
        cli_json_t j;
        cli_json_init(&j, a->out, 0);
        cli_json_bool(&j, "cleared", 1);
        cli_json_finish(&j);
    } else {
        cli_fprintf(a->out, "故障已清除（STOP_MOTOR → CLEAR_ERRORS → 错误位归零）\n");
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
        cli_fprintf(a->out, "已广播 ESTOP(0xC0)——最高仲裁优先级，总线上的设备应进入"
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
        cli_fprintf(a->err,
                "jsdk-cli: mit 必须同时给 --hold <秒>（1..60）。\n"
                "          它是唯一会驱动电机的命令，--hold 到期会自动 hold + disable。\n");
        return CLI_EXIT_REFUSED;
    }
    if (a->o.hold_s == 0 || a->o.hold_s > 60) {
        cli_fprintf(a->err, "jsdk-cli: --hold 必须在 1..60 秒之间（给的是 %d）\n", a->o.hold_s);
        return CLI_EXIT_USAGE;
    }

    if (jsdk_joint_read_config_snapshot(a->joint, &snap) != JSDK_OK || !snap.valid) {
        cli_fprintf(a->err, "jsdk-cli: 量程无效，拒绝驱动电机"
                        "（绝不能拿一个猜的量程去发 MIT 帧）\n");
        return CLI_EXIT_FAIL;
    }

    /* --- 第三道闸：把边界写出来 --- */
    cli_fprintf(a->err,
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
        cli_fprintf(a->err, "          --stiffness %.4f N·m/rad 将换算为线上 kp=%.4f\n",
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
        cli_fprintf(a->err, "jsdk-cli: 使能失败 —— %s\n", jsdk_context_last_error(a->ctx));
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
            cli_fprintf(a->err, "jsdk-cli: 收到停止请求 —— 先 hold 再 disable\n");
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
            cli_fprintf(a->err, "jsdk-cli: 收到停止请求 —— 先 hold 再 disable\n");
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
        cli_fprintf(a->out, "MIT 结束：已 hold_position 并 disable（%lu ms 实际运行）\n",
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
    int  desc_mode;      /**< CLI_DESC_NONE / CLI_DESC_ONLY / CLI_DESC_FULL（见 cli_app.h） */
    /**
     * 1 = 本命令会跑**控制/keepalive 循环**，因此必须向 SDK 声明控制周期。
     *
     * ⚠ 只给真跑循环的命令声明周期：SDK 有一条安全闸 ——“周期 >= 设备
     *   break_timeout 则拒绝 configure()”（因为那样的循环喂不了协议看门狗；
     *   `break_timeout = 0` = **禁用**时该闸不适用）。
     *   CLI 默认周期是 10 Hz = 100 ms，历史上真机设备又被默认成 100 ms
     *   → 只读诊断命令全被这条闸拦住（实测，已修）。
     *   只读命令本来就不跑循环，声明周期对它毫无意义 —— 所以 `period_ns` 传 0。
     */
    int  needs_loop;
} cmd_t;

static const cmd_t *find_cmd(const char *name);   /* 定义在本节末尾（检索函数要用） */

static const cmd_t CMDS[] = {
    /* 不需要端点表：整条命令不碰描述符（省 ~40 KB 流量） */
    { "scan",           cmd_scan,           CLI_DESC_NONE, 0 },
    { "estop",          cmd_estop,          CLI_DESC_NONE, 0 },
    /* 只读且**不需要端点表**：这三条直接问设备，不依赖 JSON 描述符 */
    { "info",           cmd_info,           CLI_DESC_NONE, 0 },  /* QUERY_DEVICE_INFO(0x46) */
    { "err",            cmd_err,            CLI_DESC_NONE, 0 },  /* QUERY_ERROR(0x44) */
    { "hb-dump",        cmd_hb_dump,        CLI_DESC_NONE, 0 },  /* 心跳原始字节 */
    /* **只要端点表**（不标定）：标定失败时这些命令必须仍然可用，否则看不清现场 */
    { "desc-info",      cmd_desc_info,      CLI_DESC_ONLY, 0 },
    { "ep-list",        cmd_ep_list,        CLI_DESC_ONLY, 0 },
    { "ep-lookup",      cmd_ep_lookup,      CLI_DESC_ONLY, 0 },
    { "desc-export",    cmd_desc_export,    CLI_DESC_ONLY, 0 },
    { "desc-import",    cmd_desc_import,    CLI_DESC_NONE, 0 },  /* 缓存的意义就是"不下载" */
    /*
     * ⚠ **读/写原始端点值不需要标定** —— 标定只是把原始值换算成物理量
     *   （gear_ratio / torque_constant / mit_max_* …）用的。
     *   `jsdk_joint_param_get/set()` 全程没有标定门（已核对：真正的门在
     *   `configure()`、group、物理量 API、`home()`、keepalive）。
     *   把它们标成 FULL 会造成一个**死结**：设备上某个标定值坏了 → 标定失败
     *   → 连 `read` 都看不了值、连 `write` 都改不了那个坏值 → 无法自救。
     *   实测（真机）：`health` 报 "calibration values out of range" 之后，
     *   `read` / `dump-config` 全部不可用，而它们本来与此无关。
     */
    { "read",           cmd_read,           CLI_DESC_ONLY, 0 },
    { "batch-read",     cmd_batch_read,     CLI_DESC_ONLY, 0 },
    { "write",          cmd_write,          CLI_DESC_ONLY, 0 },
    { "save",           cmd_save,           CLI_DESC_ONLY, 0 },
    { "set-node-id",    cmd_set_node_id,    CLI_DESC_ONLY, 0 },
    { "reset",          cmd_reset,          CLI_DESC_ONLY, 0 },
    /* 清故障：需要描述符（要发 STOP_MOTOR/CLEAR_ERRORS 序列并读回错误位）+ 跑周期 */
    { "fault-reset",    cmd_fault_reset,    CLI_DESC_FULL, 1 },
    /* **描述符 + 标定**：要物理量、要动电机的命令才需要 */
    { "health",         cmd_health,         CLI_DESC_FULL, 0 },
    /* watchdog **必须是 FULL 档**：它写的是 `can.config.break_timeout`，而那个端点 ID
       是在 configure() 的标定阶段解析出来的；放在 ONLY 档时 `ep_break_timeout` 恒为 0，
       命令必然以 “endpoint unavailable” 失败（真机与仿真上都复现过 → 见逐命令用例）。 */
    { "watchdog",       cmd_watchdog,       CLI_DESC_FULL, 0 },
    /* dump-config 打印的是**标定后的快照**（gear_ratio / mit_max_* / torque_constant /
       heartbeat_rate_ms…）：放在 ONLY 档时它只能打出一堆 0 与 `valid=0`，
       看上去像“设备没配好” —— 实际上只是那一次调用没跑标定。 */
    { "dump-config",    cmd_dump_config,    CLI_DESC_FULL, 0 },
    { "mon",            cmd_mon,            CLI_DESC_FULL, 1 },   /* 跑采样循环 */
    { "set-zero",       cmd_set_zero,       CLI_DESC_FULL, 0 },
    { "calibrate",      cmd_calibrate,      CLI_DESC_FULL, 1 },   /* 阻塞式运转：周期有意义 */
    { "home",           cmd_home,           CLI_DESC_FULL, 1 },
    { "mit",            cmd_mit,            CLI_DESC_FULL, 1 }    /* 唯一会驱动电机的命令 */
};

/** 子命令的描述符需求档（未知子命令 → NONE）。 */
int cli_cmd_desc_mode(const char *sub)
{
    const cmd_t *c = find_cmd(sub);
    return c ? c->desc_mode : CLI_DESC_NONE;
}

/** 子命令是否跑控制/keepalive 循环（未知子命令 → 0）。 */
int cli_cmd_needs_loop(const char *sub)
{
    const cmd_t *c = find_cmd(sub);
    return c ? c->needs_loop : 0;
}


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
        cli_fprintf(err, "jsdk-cli: 未知子命令 %s（--help 看用法）\n", a.o.sub);
        return CLI_EXIT_USAGE;
    }

    if (!a.o.quiet && !a.o.json && a.o.verbose) {
        cli_fprintf(err, "jsdk-cli: %s\n", a.o.sub);
    }

    /* cli_open 自己区分"用法/参数问题（2）"与"运行时失败（1）"——
       早先这里无条件返回 1，于是 `--if nosuchbus` 这种明显的用法错误
       退出码成了 1，脚本里没法区分"配错了"和"跑失败了"。 */
    rc = cli_open(&a);
    if (rc != 0) { cli_close(&a); return rc; }

    /*
     * 端点表：
     *   FULL → configure()（描述符 + 标定）；
     *   ONLY → 只下描述符（标定失败也能看端点表）。
     *
     * ⚠⚠ 描述符加载失败时**也得**走到下面的“帧格式报告”（所以这里只记 rc，
     *   不提前 return）。帧格式猜错正是“下载 0 字节但心跳正常”的头号原因，
     *   而这个分支恰恰是最需要把那句话打出来的时候 —— 早先直接 return，
     *   于是用户只能看到一条超时，而那正是一整轮排查被拖长的原因。
     */
    {
        int dm = c->desc_mode;
        if (dm != CLI_DESC_NONE && cli_load_desc(&a, dm == CLI_DESC_FULL) != 0) {
            rc = CLI_EXIT_FAIL;
        } else {
            rc = c->fn(&a);
        }
    }

    /*
     * ⚠ 对端帧格式与我们的配置不一致时要**说出来**（协议没有运行时协商）：
     *   猜错的后果是“收得到心跳、但请求没人应”（真机实测：1 Mbps Classic 的设备
     *   + 默认发 FD ⇒ `desc-info` 报 `0/0 bytes, 198 frames received`，
     *   非常像线缆/波特率问题）。SDK 已经对齐过去，这里负责报告 + 告知下次怎么写。
     *
     * ⚠⚠ 但**显式指定过格式**时（`--classic` / `--data-bitrate`）SDK 不会改它
     *   （改掉就成了“你说了不算”），此时报的是 got == 4：“你写的与对端冲突” ——
     *   这种情形命令照样可能失败，所以必须说清楚该删/该加哪个选项。
     */
    {
        int got = a.ctx ? jsdk_context_framing_learned(a.ctx) : 0;

        /* `-v`：把“怎么定的帧格式”和 slcan 的收发/回执计数一起说出来 ——
           这两样是排“到底发出去了没有”最快的东西（真机上一整轮排查都卡在
           看不出自己发的帧有没有上总线）。 */
        if (a.o.verbose) {
            /* ⚠ 报**实际生效**的格式，而不是 `a.fd`：SDK 的自动对齐可能已经把
               上下文改成对端的格式（此时 `a.fd` 还是起步时的值，说了会自相矛盾）。 */
            int eff_fd = (got == 2) ? 1 : ((got == 1) ? 0 : a.fd);
            jsdk_bus_state_t bs;
            uint32_t retries = 0u;
            uint32_t retries_req = 0u;

            if (a.ctx && jsdk_context_get_bus_state(a.ctx, &bs) == JSDK_OK) {
                retries     = bs.tx_retries;
                retries_req = bs.tx_retries_req;
            }

            cli_fprintf(err, "jsdk-cli: 帧格式：%s%s；framing_learned=%d；"
                            "首发=%s；重发=%u（预热 %u + 幂等请求 %u）；tx=%u rx=%u\n",
                    eff_fd ? "CAN FD" : "Classic",
                    a.fd_auto ? "（探测决定）" : "（显式/已定）", got,
                    a.first_tx_fd < 0 ? "none"
                                      : (a.first_tx_fd ? "CAN FD" : "Classic"),
                    (unsigned)retries,
                    (unsigned)(retries - retries_req), (unsigned)retries_req,
                    (unsigned)a.tx_frames, (unsigned)a.rx_frames);
            if (a.hal && a.o.ifname && strcmp(a.o.ifname, "slcan") == 0) {
                uint32_t tx = 0u, rx = 0u, bad = 0u, acks = 0u, nacks = 0u;

                jsdk_hal_slcan_stats(a.hal, &tx, &rx, &bad, &acks, &nacks);
                cli_fprintf(err, "jsdk-cli: slcan 统计 tx=%u rx=%u acks=%u "
                                "nacks=%u malformed=%u\n",
                        (unsigned)tx, (unsigned)rx, (unsigned)acks,
                        (unsigned)nacks, (unsigned)bad);
            }
        }

        if (got == 1 || got == 2) {
            /*
             * ⚠ 只在**预热也没得到应答**时才报“猜错了、已改学”：自动模式下 CLI
             *   会先问一句对端（见 cli_app.c 的 cli_warmup_and_align），帧格式是
             *   “探测决定”的，再报一句“本次的 is_fd 猜错了”就是噪声（而且是假话）。
             *   探测的结果在 `-v` 里如实打出来。
             */
            if (a.framing_probe == 0) {
                cli_fprintf(err,
                    "jsdk-cli: [注意] 对端在发 %s 帧，已自动按 %s 发送"
                    "（本次的 is_fd 猜错了）。下次请显式传 %s。\n",
                    got == 1 ? "Classic" : "CAN FD",
                    got == 1 ? "Classic" : "CAN FD",
                    got == 1 ? "--classic" : "--data-bitrate 5000000");
            }
        } else if (got == 4) {
            /*
             * 显式指定的格式与对端**冲突**。两个方向都得说（真机 + 仿真都验过）：
             *   ① 对端 Classic、我们发 FD → 设备**收不到**我们的帧，命令必失败；
             *   ② 对端 FD、我们发 Classic → FD 控制器（及适配器）收得下 8 B 经典帧，
             *      命令**能过**，但 8 字节参数退化成两次请求，且白丢 FD 的带宽。
             * 所以这里不说“必定失败”，只说清事实 + 该删/该改成什么。
             */
            if (a.fd) {
                cli_fprintf(err,
                    "jsdk-cli: [警告] 对端在发 Classic 帧，而你**显式指定**了 CAN FD"
                    "（--data-bitrate）：设备收不到我们的帧，本次请求会全部超时。"
                    "请改用 --classic。\n");
            } else {
                cli_fprintf(err,
                    "jsdk-cli: [警告] 对端在发 CAN FD 帧，而你**显式指定**了 --classic："
                    "命令能跑，但 8 字节参数会退化成两次请求（白丢 FD 带宽）。"
                    "若非本意，去掉 --classic 让 SDK 自动对齐。\n");
            }
        }
    }

    cli_close(&a);
    return rc;
}
