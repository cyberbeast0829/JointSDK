/**
 * @file    cli_app.c
 * @brief   选项解析 + 运行环境（HAL 包装、上下文生命周期、输出辅助）
 */

#include "cli_app.h"

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#endif

#include "jsdk_cli.h"        /* jsdk_cli_stop_requested() */
#include "cli_text.h"

/* ==========================================================================
 * 选项解析
 * ======================================================================== */

void cli_usage(FILE *f, const char *argv0)
{
    cli_fprintf(f,
        "用法: %s [全局选项] <子命令> [子命令选项]\n"
        "\n"
        "全局选项\n"
        "  --if <socketcan|pcan|slcan|virtual>  传输后端（默认按平台）\n"
        "  --channel <can0|PCAN_USBBUS1|COM5|规格>  通道名\n"
        "  --bitrate N          仲裁段波特率（默认 1000000）\n"
        "  --data-bitrate N     数据段波特率（默认 5000000；写 0 或加 --classic 用 Classic）\n"
        "  --baud N             **串口**波特率（仅 slcan，默认 115200）\n"
        "  --classic            强制 Classic CAN\n"
        "  --master-id N        主站源地址（默认 1）\n"
        "  --node N             目标节点 ID（默认 1）\n"
        "  --probe N            scan 的主动探测上限（默认 16，0 = 仅被动）\n"
        "  --timeout MS         单次操作超时（默认 3000）\n"
        "  --timeout-ms MS      等状态序列跑完的预算（calibrate/home）\n"
        "                       默认标定 120000 / 回零 5000（全标定要转十几圈电气角）\n"
        "  --json               机器可读输出\n"
        "  --rate-hz N          mon 的采样率（默认 10）\n"
        "  --duration S         运行时长，0 = 直到 Ctrl-C（默认 0）\n"
        "  -v / -q              日志级别\n"
        "\n"
        "安全（写/动的前置条件）\n"
        "  --yes                确认执行会写设备或驱动电机的子命令\n"
        "  --hold S             mit 的持续时间（秒，1..60，必需）\n"
        "\n"
        "只读子命令\n"
        "  scan                          节点发现（被动 200 ms + 主动探测）\n"
        "  info                          QUERY_DEVICE_INFO：hw/fw/serial\n"
        "  health                        模式/错误码/心跳标志/温度/母线/新鲜度/链路统计\n"
        "  mon [--csv] [--csv-file F]   周期监控\n"
        "  read <path>                   按名读参数\n"
        "  batch-read <path>...          批量读（FD 单帧；Classic 自动退化）\n"
        "  dump-config                   读回关键配置快照\n"
        "  err                           QUERY_ERROR 六类 32-bit 错误明细\n"
        "  hb-dump                       最近心跳的**原始字节** + 解码对照\n"
        "  desc-info                     描述符元信息\n"
        "  ep-list [--filter P]          枚举端点（P 为子串；以 '*' 结尾则按前缀）\n"
        "  ep-lookup <path>              路径 → 端点 ID/类型/权限\n"
        "  desc-export <file>            导出描述符缓存\n"
        "  desc-import <file>            导入描述符缓存\n"
        "\n"
        "配置子命令（需 --yes）\n"
        "  write <path> <value>          参数写\n"
        "  save                          CONFIG_SAVE(0x22)\n"
        "  set-node-id N                 改节点地址（可持久化）\n"
        "  watchdog MS                   设定 break_timeout\n"
        "  set-zero                      SET_ZERO(0x61)\n"
        "  reset                         RESET_DEVICE(0x64)\n"
        "\n"
        "动作子命令（需 --yes）\n"
        "  calibrate                     写 requested_state = 3 并等待\n"
        "  home                          写 requested_state = 11 并等待\n"
        "  estop                         广播 ESTOP(0xC0)（**全局广播**，打到总线上所有节点）\n"
        "  fault-reset                   清故障：STOP_MOTOR → CLEAR_ERRORS → 等错误位归零\n"
        "                                （estop/FAULT_ALERT 锁死关节后唯一的软件恢复路径）\n"
        "  mit [--pos R --vel R --kp K --kd D --tau T] --hold S\n"
        "                                唯一会驱动电机的命令\n",
        argv0 ? argv0 : "jsdk-cli");
}

/* --------------------------------------------------------------------------
 * 选项匹配
 *
 * ⚠ 这里曾经有一个很隐蔽的 bug：取值辅助函数写成"把下一个 argv 当作值"，
 *   然后在**每一个**选项名上试探性调用。结果是 `--channel X` 会让 `--if` 的
 *   试探把 X 吃掉并当成后端名 —— 命令只报"未知后端"，真正的原因看不出来。
 *   现在的契约很明确：**先确认是本选项，才消费下一个 argv**。
 *
 *   返回：1 = 命中（*out 已赋值）；0 = 不是本选项；-1 = 是本选项但缺值。
 * ------------------------------------------------------------------------ */
static int opt_value(int *i, int argc, char **argv, const char *tok,
                     const char *name, const char **out, FILE *err)
{
    size_t n = strlen(name);

    if (strcmp(tok, name) == 0) {
        if (*i + 1 >= argc) {
            cli_fprintf(err, "jsdk-cli: 选项 %s 需要一个值\n", name);
            return -1;
        }
        (*i)++;
        *out = argv[*i];
        return 1;
    }
    if (strncmp(tok, name, n) == 0 && tok[n] == '=') {
        *out = tok + n + 1u;
        return 1;
    }
    return 0;
}

static int parse_u32(const char *s, uint32_t *out)
{
    char *end = NULL;
    unsigned long v;

    if (!s || !*s) return -1;
    v = strtoul(s, &end, 0);
    if (!end || *end != '\0') return -1;
    *out = (uint32_t)v;
    return 0;
}

static int parse_i32(const char *s, int *out)
{
    char *end = NULL;
    long v;

    if (!s || !*s) return -1;
    v = strtol(s, &end, 0);
    if (!end || *end != '\0') return -1;
    *out = (int)v;
    return 0;
}

static int parse_f64(const char *s, double *out)
{
    char *end = NULL;
    double v;

    if (!s || !*s) return -1;
    v = strtod(s, &end);
    if (!end || *end != '\0') return -1;
    *out = v;
    return 0;
}

int cli_opts_parse(cli_opts_t *o, int argc, char **argv, FILE *err)
{
    int i;
    int no_more_opts = 0;

    memset(o, 0, sizeof *o);
    o->bitrate      = 1000000u;
    o->data_bitrate = 5000000u;
    o->master_id    = 1u;
    o->node         = 1u;
    o->rate_hz      = 10;
    o->max_probe    = 16u;
    o->timeout_ms   = 3000u;
    o->baud         = 115200u;
    o->hold_s       = -1;          /* -1 = 未给出（与 0 区分：0 非法） */

    for (i = 1; i < argc; ++i) {
        const char *t = argv[i];
        const char *v = NULL;
        int         is_opt = (!no_more_opts && t[0] == '-' && t[1] != '\0');

        /* ⚠ **负数是位置参数，不是选项**：`write axis0.controller.config.vel_limit -5.0`
           以前会被当成"未知选项 -5.0"直接拒掉 —— 也就是**没法从 CLI 写任何负数**
           （速度/力矩限值、力矩指令、有符号增益…）。Python 版（argparse）本来就认，
           两版又对不上。判定规则与 argparse 的负数字面量一致：`-` 后面紧跟数字或 `.`。 */
        if (is_opt && (t[1] >= '0' && t[1] <= '9')) is_opt = 0;
        if (is_opt && t[1] == '.' && t[2] >= '0' && t[2] <= '9') is_opt = 0;

        if (is_opt && strcmp(t, "--") == 0) { no_more_opts = 1; continue; }

        if (!is_opt) {
            if (!o->sub) {
                o->sub = t;
            } else if (o->nargs < (unsigned)(sizeof o->args / sizeof o->args[0])) {
                o->args[o->nargs++] = t;
            } else {
                cli_fprintf(err, "jsdk-cli: 位置参数过多（最多 %u 个）\n",
                        (unsigned)(sizeof o->args / sizeof o->args[0]));
                return 2;
            }
            continue;
        }

        /* --- 布尔 --- */
        if (strcmp(t, "--json") == 0) { o->json = 1; continue; }
        if (strcmp(t, "--classic") == 0) { o->classic = 1; o->fd_explicit = 1; o->data_bitrate = 0u; continue; }
        if (strcmp(t, "--yes") == 0) { o->yes = 1; continue; }
        /* ⚠ `--csv` 是**格式开关**（与 Python 版 `python -m jsdk_can` 一致）。
           它以前是“要一个文件名”，于是 `mon --csv --duration 1` 会把 `--duration`
           当成文件名、**静默写出一个叫 `--duration` 的 CSV**（真发生过，清理时才发现）。
           写文件的能力保留在 `--csv-file <file>`（取值时也会拒绍以 `-` 开头的东西）。 */
        if (strcmp(t, "--csv") == 0) { o->csv = 1; continue; }
        if (strcmp(t, "--verbose") == 0 || strcmp(t, "-v") == 0) { o->verbose = 1; continue; }
        if (strcmp(t, "--quiet") == 0 || strcmp(t, "-q") == 0) { o->quiet = 1; continue; }
        if (strcmp(t, "--help") == 0 || strcmp(t, "-h") == 0) { cli_usage(err, argv[0]); return 1; }

        /* --- 取值（表驱动；先确认是本选项才消费下一个 argv） --- */
        {
            static const struct { const char *name; int key; } VAL[] = {
                { "--if",           0 },  { "--channel",      1 },
                { "--bitrate",      2 },  { "--data-bitrate", 3 },
                { "--master-id",    4 },  { "--node",         5 },
                { "--probe",        6 },  { "--timeout",      7 },
                { "--rate-hz",      8 },  { "--duration",     9 },
                { "--hold",        10 },  { "--csv-file",    20 },
                { "--filter",      12 },  { "--pos",         13 },
                { "--vel",         14 },  { "--kp",          15 },
                { "--kd",          16 },  { "--tau",         17 },
                { "--stiffness",   18 },  { "--baud",        19 },
                { "--timeout-ms",  21 }
            };
            unsigned k;
            int      hit = 0;

            for (k = 0u; k < sizeof VAL / sizeof VAL[0]; ++k) {
                uint32_t u;
                int      m = opt_value(&i, argc, argv, t, VAL[k].name, &v, err);

                if (m < 0) return 2;
                if (m == 0) continue;

                /* 一处集中报"值非法"，避免每个分支重复四行 */
#define CLI_BAD_VALUE()                                                      \
                do {                                                         \
                    cli_fprintf(err, "jsdk-cli: 选项 %s 的值非法: %s\n",          \
                            VAL[k].name, v);                                 \
                    return 2;                                                \
                } while (0)

                switch (VAL[k].key) {
                case 0:  o->ifname = v; break;
                case 1:  o->channel = v; break;
                case 2:  if (parse_u32(v, &o->bitrate) != 0) CLI_BAD_VALUE(); break;
                case 3:  if (parse_u32(v, &o->data_bitrate) != 0) CLI_BAD_VALUE();
                         o->fd_explicit = 1;   /* 写了数据段波特率 = 明确要 FD */
                         break;
                case 4:
                    /* 主站 0 → 设备完全不回复：这不是"高级用法"，是配错了 */
                    if (parse_u32(v, &u) != 0 || u == 0u || u > 254u) CLI_BAD_VALUE();
                    o->master_id = (uint8_t)u;
                    break;
                case 5:
                    if (parse_u32(v, &u) != 0 || u == 0u || u > 254u) CLI_BAD_VALUE();
                    o->node = (uint8_t)u;
                    break;
                case 6:
                    if (parse_u32(v, &u) != 0 || u > 254u) CLI_BAD_VALUE();
                    o->max_probe = u;
                    break;
                case 7:  if (parse_u32(v, &o->timeout_ms) != 0) CLI_BAD_VALUE(); break;
                case 21:
                    /* 等状态序列跑完的预算（标定/回零）。全标定真机实测 >20 s，
                       默认给到 120 s；客户如果缩短它，超时提示会说“还没跑完”。 */
                    if (parse_u32(v, &o->state_timeout_ms) != 0) CLI_BAD_VALUE();
                    break;
                case 8:
                    if (parse_i32(v, &o->rate_hz) != 0
                        || o->rate_hz <= 0 || o->rate_hz > 1000) CLI_BAD_VALUE();
                    break;
                case 9:
                    if (parse_i32(v, &o->duration_s) != 0 || o->duration_s < 0)
                        CLI_BAD_VALUE();
                    break;
                case 10:
                    if (parse_i32(v, &o->hold_s) != 0 || o->hold_s < 0) CLI_BAD_VALUE();
                    break;
                case 20:
                    /* 防的就是那一个坑：`--csv-file --duration 1` 会把下一个选项当文件名，
                       然后静默写出一个叫 `--duration` 的文件（真发生过）。 */
                    if (v[0] == '-') {
                        cli_fprintf(err, "jsdk-cli: %s 需要一个文件路径，"
                                         "但得到的是另一个选项（%s）\n", VAL[k].name, v);
                        return 2;
                    }
                    o->csv_file = v;
                    break;
                case 12: o->filter = v; break;
                case 13:
                    if (parse_f64(v, &o->pos) != 0) CLI_BAD_VALUE();
                    o->have[0] = 1;
                    break;
                case 14:
                    if (parse_f64(v, &o->vel) != 0) CLI_BAD_VALUE();
                    o->have[1] = 1;
                    break;
                case 15:
                    if (parse_f64(v, &o->kp) != 0) CLI_BAD_VALUE();
                    o->have[2] = 1;
                    break;
                case 16:
                    if (parse_f64(v, &o->kd) != 0) CLI_BAD_VALUE();
                    o->have[3] = 1;
                    break;
                case 17:
                    if (parse_f64(v, &o->tau) != 0) CLI_BAD_VALUE();
                    o->have[4] = 1;
                    break;
                case 18:
                    if (parse_f64(v, &o->kp_stiff) != 0) CLI_BAD_VALUE();
                    o->have_stiff = 1;
                    break;
                case 19:
                    if (parse_u32(v, &o->baud) != 0 || o->baud == 0u) CLI_BAD_VALUE();
                    break;
                default: break;
                }
#undef CLI_BAD_VALUE

                hit = 1;
                break;
            }
            if (hit) continue;
        }

        cli_fprintf(err, "jsdk-cli: 未知选项 %s（--help 看用法）\n", t);
        return 2;
    }

    if (!o->sub) {
        cli_fprintf(err, "jsdk-cli: 缺少子命令（--help 看用法）\n");
        return 2;
    }
    return 0;
}

/* ==========================================================================
 * HAL 包装
 * ======================================================================== */

#ifdef _WIN32
static void cli_sleep_impl(unsigned ms) { Sleep((DWORD)ms); }
static uint32_t cli_wall_ms(void) { return (uint32_t)GetTickCount(); }
#else
static void cli_sleep_impl(unsigned ms)
{
    struct timespec ts;
    ts.tv_sec  = (time_t)(ms / 1000u);
    ts.tv_nsec = (long)(ms % 1000u) * 1000000L;
    nanosleep(&ts, NULL);
}
static uint32_t cli_wall_ms(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0u;
    return (uint32_t)((uint64_t)ts.tv_sec * 1000ull
                      + (uint64_t)ts.tv_nsec / 1000000ull);
}
#endif

int cli_sleep_ms(unsigned ms)
{
    /* 分片睡眠：Ctrl-C 时最多 20 ms 就能响应，而不是等满一整段 */
    unsigned left = ms;

    while (left > 0u) {
        unsigned chunk = (left > 20u) ? 20u : left;
        if (jsdk_cli_stop_requested()) return 1;
        cli_sleep_impl(chunk);
        left -= chunk;
    }
    return jsdk_cli_stop_requested();
}

static int wrap_send(void *user, const jsdk_can_frame_t *f)
{
    cli_app_t *a = (cli_app_t *)user;
    int rc = a->user_hal.send(a->user_hal.user, f);

    if (rc == 0) {
        if (a->tx_frames == 0u) {          /* 记录**首发**用的帧格式（排障和用例都要） */
            a->first_tx_fd = (f->flags & JSDK_FRAME_FD) ? 1 : 0;
        }
        a->tx_frames++;
    }
    return rc;
}

static int wrap_recv(void *user, jsdk_can_frame_t *f)
{
    cli_app_t *a = (cli_app_t *)user;
    int rc = a->user_hal.recv(a->user_hal.user, f);

    if (rc > 0) {
        a->rx_frames++;
        if (a->rx_ring_n < CLI_RX_RING) {
            a->rx_ring[a->rx_ring_n++] = *f;
        } else {
            /* 环形覆盖最旧（保住"最近的帧"，这正是排障需要的） */
            memmove(&a->rx_ring[0], &a->rx_ring[1],
                    sizeof(a->rx_ring[0]) * (CLI_RX_RING - 1u));
            a->rx_ring[CLI_RX_RING - 1u] = *f;
        }
    }
    return rc;
}

static uint32_t wrap_now_ms(void *user)
{
    cli_app_t *a = (cli_app_t *)user;

    /*
     * 虚拟后端：直接转发即可 —— 后端本身已打开"自动推进时钟"
     * （`jsdk_hal_virtual_set_autotick()`，在 cli_open 里打开）。
     *
     * ⚠ 早期版本在**这里**手动 `advance_ms(1)`。功能一样，但那是第二份实现：
     *   Python 绑定与离线脚本没法给 C 填进去的 `now_ms` 打补丁，所以这件事
     *   最终应该只存在于后端里。现在只有一处。
     */
    return a->user_hal.now_ms(a->user_hal.user);
}

static void wrap_on_error(void *user, int kind, uint32_t detail)
{
    cli_app_t *a = (cli_app_t *)user;
    if (a->user_hal.on_error) a->user_hal.on_error(a->user_hal.user, kind, detail);
}

static int wrap_bus_status(void *user, uint32_t *flags)
{
    cli_app_t *a = (cli_app_t *)user;
    if (a->user_hal.bus_status) return a->user_hal.bus_status(a->user_hal.user, flags);
    *flags = 0u;
    return 0;
}

/* ==========================================================================
 * 打开 / 配置 / 关闭
 * ======================================================================== */

static int cli_init_ctx(cli_app_t *a);   /* 定义在 cli_open 之后 */

/** 平台默认后端：Windows/macOS 用 pcan，其余用 socketcan。 */
static const char *default_ifname(void)
{
#if defined(_WIN32) || defined(__APPLE__)
    return "pcan";
#else
    return "socketcan";
#endif
}

static const char *default_channel(const char *ifname)
{
    if (!ifname) return "";
    if (strcmp(ifname, "pcan") == 0)      return "PCAN_USBBUS1";
    if (strcmp(ifname, "slcan") == 0) {
#ifdef _WIN32
        return "COM3";
#else
        return "/dev/ttyACM0";
#endif
    }
    if (strcmp(ifname, "virtual") == 0)   return "";
    return "can0";
}

/**
 * `--if virtual` 且未给 `--channel` 时的默认节点规格。
 *
 * ⚠ **必须带 `timeout=`**（设备的 `break_timeout`）。缺了它设备就用固件默认的
 *   100 ms，而 CLI 的默认控制周期是 `1e9 / --rate-hz(10)` = **100 ms** ——
 *   `configure()` 会直接以 `bad-state` 拒绝（"控制周期 ≥ break_timeout，回路
 *   喂不了看门狗"）。现场表现是“不加 --channel 就什么都干不了、加了就好了”，
 *   而 `test_cli.c` 一直传 `--channel`，所以这条路径之前从未被测到。
 *   取值与 Python 绑定的 `DEFAULT_VIRTUAL_SPEC` 保持一致。
 */
#define JSDK_CLI_DEFAULT_VIRTUAL_SPEC                                        \
    "0:id=1,gear=16.5,pmax=12.5,vmax=65,tmax=50,kpmax=500,kdmax=5,"          \
    "hb=10,timeout=30000,fd"

#define CLI_FRAMING_PROBE_MS 500u

/**
 * 打开传输层 + 建上下文 + 加关节（**不发任何帧**）。
 *
 * 拆出来是为了“探测后重开”那条路：`fd_auto` 时先按 Classic 起步、听一耳朵，
 * 对端是 FD 才重开一次（见 `cli_probe_framing()` / `cli_app.h` 的 `fd_auto`）。
 */
static int cli_bringup(cli_app_t *a, const char *ifname, const char *chan)
{
    jsdk_status_t st;

    memset(&a->user_hal, 0, sizeof a->user_hal);
    a->virt_ms = 0u;

    if (strcmp(ifname, "virtual") == 0) {
        a->is_virtual = 1;
        st = jsdk_hal_virtual_open(&a->user_hal, &a->hal,
                                   (chan && chan[0]) ? chan
                                                     : JSDK_CLI_DEFAULT_VIRTUAL_SPEC);
        if (st == JSDK_OK) {
            /* 让仿真时钟像真实 HAL 一样自由运行：否则 configure()/discover()
               这些阻塞式配置 API 会在冻结时钟下直到自旋上限才失败。 */
            jsdk_hal_virtual_set_autotick(a->hal, 1);
        }
    } else if (strcmp(ifname, "socketcan") == 0) {
        st = jsdk_hal_socketcan_open(&a->user_hal, &a->hal, chan,
                                     a->o.bitrate, a->o.data_bitrate);
    } else if (strcmp(ifname, "pcan") == 0) {
        st = jsdk_hal_pcan_open(&a->user_hal, &a->hal, chan,
                               a->o.bitrate, a->o.data_bitrate);
        if (st == JSDK_ERR_NOT_FOUND) {
            cli_fprintf(a->err, "jsdk-cli: 打不开 PCAN：找不到 PCANBasic 库。%s\n",
                    "请安装 PEAK 的 PCAN-Basic 驱动（32/64 位要与本进程一致）");
        }
    } else if (strcmp(ifname, "slcan") == 0) {
        /* ⚠ 第 4 个参数是**串口**波特率（--baud），不是 CAN 仲裁段波特率。
           早先错传了 --bitrate（默认 1 000 000），串口会以 1 Mbaud 打开，
           而适配器通常在 115200 → 收到乱码，现场只表现为"什么都收不到"。
           第 5 个参数是 FD **数据段**速率（--data-bitrate）：非 0 时后端会主动
           发 Y<n> 适配器命令。

           ⚠⚠ `fd_auto`（客户没写 --classic/--data-bitrate）时**传 0**：
             先**不要**按 FD 配置适配器。真机实测（CyberBeast USB2CAN @1 Mbps
             Classic）：默认发 Y5 之后，即便 SDK 随后按对端改学成 Classic，
             那些帧也**已经发不出去了**（现场：`0/0 bytes` + 心跳正常 + 提示说
             已按 Classic 发送 —— 自相矛盾）。改成“先只听、听准了再决定（必要时
             重开）”后，同一条命令不传任何格式选项也能跑通。 */
        {
            uint32_t fd_rate = a->fd_auto ? 0u : a->o.data_bitrate;

            st = jsdk_hal_slcan_open(&a->user_hal, &a->hal, chan, a->o.baud,
                                     fd_rate);
            if (st == JSDK_ERR_UNSUPPORTED) {
                cli_fprintf(a->err, "jsdk-cli: slcan 的数据段速率 %u 不在已知表里"
                                "（仅支持 2000000 与 5000000；"
                                "其它速率请先用厂家工具配好，再传 --data-bitrate 0）\n",
                        (unsigned)fd_rate);
            }
        }
        if (a->o.bitrate != 1000000u) {
            cli_fprintf(a->err, "jsdk-cli: 提示：--bitrate 对 slcan 无意义"
                            "（CAN 仲裁段速率由适配器自己配），串口速率用 --baud %u\n",
                    (unsigned)a->o.baud);
        }
        /* 注：--classic 在解析阶段就把 data_bitrate 归 0（见 cli_app.c 的选项循环），
           所以"Classic 却要发 Y 命令"这种情况不可能出现，不需要额外提示。 */
    } else {
        cli_fprintf(a->err, "jsdk-cli: 未知后端 --if %s（socketcan|pcan|slcan|virtual）\n",
                ifname);
        return 2;
    }

    if (st != JSDK_OK) {
        cli_fprintf(a->err, "jsdk-cli: 打开 %s(%s) 失败：%s\n",
                ifname, chan ? chan : "", jsdk_status_string(st));
        /* ⚠ slcan 的 `invalid-argument` **什么也没说**：真因可能是权限、设备不存在、
           被占用……而返回码只能是一个。把后端记下的原因（含 errno 与建议）打出来 ——
           现场为了分辨这几种情况花的时间，比写这段代码多得多。 */
        if (strcmp(ifname, "slcan") == 0) {
            const char *why = jsdk_hal_slcan_last_open_error();

            if (why && why[0]) cli_fprintf(a->err, "  原因：%s\n", why);
        }
        return 1;
    }

    /* 包装回调 */
    a->sdk_hal              = a->user_hal;
    a->sdk_hal.user         = a;
    a->sdk_hal.send         = wrap_send;
    a->sdk_hal.recv         = wrap_recv;
    a->sdk_hal.now_ms       = wrap_now_ms;
    a->sdk_hal.on_error     = wrap_on_error;
    a->sdk_hal.bus_status   = wrap_bus_status;

    /* ⚠ `a->fd` / `a->fd_auto` 由 `cli_open()` 定（这里是“按现有决定建立连接”，
       不再自己从 `--data-bitrate` 推导 —— 否则“探测后重开”那一步会被覆盖掉）。 */

    if (a->o.verbose) {
        cli_fprintf(a->err, "jsdk-cli: %s(%s) bitrate=%u data=%u %s%s master=%u node=%u\n",
                ifname, chan ? chan : "", a->o.bitrate,
                (unsigned)(a->fd_auto ? 0u : a->o.data_bitrate),
                a->fd ? "FD" : "Classic",
                a->fd_auto ? "(起步：待探测)" : "(显式指定)",
                (unsigned)a->o.master_id,
                (unsigned)a->o.node);
    }

    /* --- 上下文（不下载描述符：scan/estop 用不到那 41 KB） --- */
    return cli_init_ctx(a);
}

/**
 * 重开传输层（并重建上下文），改用 `want_fd` 指定的帧格式。
 *
 * 为什么必须“重开”而不能只改 `cfg`：slcan 的 FD **数据段速率**只能在打开序列里
 * 发（`C` → `Y<n>` → `O`），而“先按 Classic 打开、探测到对端是 FD 之后”已经没有
 * 那个时机了。其它后端也有同样的“打开参数已定”问题，所以统一重开。
 */
static int cli_reopen_as(cli_app_t *a, const char *ifname, const char *chan,
                         int want_fd)
{
    if (a->o.verbose) {
        cli_fprintf(a->err, "jsdk-cli: 探测结果 %d → 重开为 %s\n",
                a->framing_probe, want_fd ? "CAN FD" : "Classic");
    }
    cli_close(a);

    a->fd_auto = 0;                          /* 已经定了：第二遍不再探测 */
    a->fd      = want_fd;
    if (want_fd && a->o.data_bitrate == 0u) a->o.data_bitrate = 5000000u;

    return cli_bringup(a, ifname, chan);
}

/**
 * **发帧前**只收不发地听一耳朵对端的帧格式（≤ `CLI_FRAMING_PROBE_MS`）。
 *
 * 为什么要有这一步（真机现场，2026-09）：协议没有运行时协商，而“第一条帧就用错
 * 格式”的代价**不只是没人应** —— 默认发 FD 还会让适配器按 FD 去配（`Y5`），
 * 于是一台 1 Mbps Classic 的设备即使随后被“自动改学”正确，请求也**发不出去**：
 * 现场表现为 `0/0 bytes` + 心跳正常 + 末尾却提示“已自动按 Classic 发送”。
 * 先只听（Classic 是更兼容的方向：FD 控制器也接受经典帧）就能彻底躲开这一整类。
 *
 * @return 0 = 探测完成（不管有没有听到）；非 0 = 重开失败（错误已打印）
 */
static int cli_probe_framing(cli_app_t *a, const char *ifname, const char *chan)
{
    uint32_t t0  = a->sdk_hal.now_ms(a->sdk_hal.user);
    int      got = 0;

    for (;;) {
        /* `cycle_begin()` = 只收不发（发送在 `cycle_end()` 里）—— 正是这里要的。 */
        (void)jsdk_context_cycle_begin(a->ctx, 0u);
        got = jsdk_context_framing_learned(a->ctx);
        if (got != 0) break;
        if ((uint32_t)(a->sdk_hal.now_ms(a->sdk_hal.user) - t0) >= CLI_FRAMING_PROBE_MS) {
            break;
        }
        (void)cli_sleep_ms(1u);
    }
    a->framing_probe = got;

    if (got == 0) {
        /* 一路静默：听不出对端是哪种格式。按客户给的值（默认 FD）走老路，
           免得“设备只在被问时才开口”的现场从 FD 退化成 Classic。 */
        if (a->o.data_bitrate != 0u && a->fd == 0) {
            return cli_reopen_as(a, ifname, chan, 1);
        }
        return 0;
    }

    if (got == 2 && a->fd == 0) {
        /* 对端是 FD、我们起步是 Classic：
           slcan 必须重开（要补发 `Y<n>`）；其它后端的 FD 能力是打开时的
           socket/句柄属性，不用重开 —— 而 SDK 的学习已经把 ctx 的格式对齐成 FD。 */
        if (strcmp(ifname, "slcan") == 0) {
            return cli_reopen_as(a, ifname, chan, 1);
        }
        if (a->o.verbose) {
            cli_fprintf(a->err, "jsdk-cli: 探测到对端是 CAN FD，已切到 FD 发送\n");
        }
    }
    return 0;
}

int cli_open(cli_app_t *a)
{
    const char *ifname = a->o.ifname ? a->o.ifname : default_ifname();
    const char *chan   = a->o.channel ? a->o.channel : default_channel(ifname);
    int         rc;

    /*
     * 帧格式：**没被显式指定**时先按 Classic 起步（见 cli_app.h 的 `fd_auto`）。
     * FD 控制器兼容经典帧，反方向不兼容 —— 所以“先 Classic”是两者都安全的方向；
     * 若对端是 FD，探测后会重开成 FD（拿到 FD 带宽与 8 字节整读）。
     */
    a->fd_auto = (a->o.fd_explicit == 0) ? 1 : 0;
    a->fd      = a->fd_auto ? 0 : ((a->o.data_bitrate != 0u) ? 1 : 0);

    rc = cli_bringup(a, ifname, chan);
    if (rc != 0) return rc;

    /*
     * ⚠ `estop` 不做探测：它是安全命令，必须**立刻**发出去。按 Classic 发是安全的
     *   选择（FD 控制器也接受经典帧，而 8 字节以内的载荷两种格式完全一样）；
     *   顺带把“Classic 总线上 estop 静默发不出去”这个老问题也避开了。
     */
    if (a->fd_auto && (!a->o.sub || strcmp(a->o.sub, "estop") != 0)) {
        rc = cli_probe_framing(a, ifname, chan);
    }
    return rc;
}

/**
 * 建上下文并添加关节。
 *
 * ⚠ 这里**不**调用 `jsdk_context_configure()`：`scan` / `estop` 只需要一个能
 *   收发帧的上下文，不需要端点表。描述符下载最多 6840 帧，纯诊断命令不该付。
 *   （`scan` 在真机上也没问题：主动探测的第一帧就足以让设备学到主站地址。）
 */
static int cli_init_ctx(cli_app_t *a)
{
    jsdk_context_config_t *cfg = &a->cfg;   /* ⚠ 必须存在 a 上：见 cli_app.h 里的说明 */
    jsdk_joint_config_t   jc;
    jsdk_status_t         st;

    a->ctx = (jsdk_context_t *)calloc(1u, jsdk_context_size(NULL));
    if (!a->ctx) {
        cli_fprintf(a->err, "jsdk-cli: 内存不足（上下文 %u 字节）\n",
                (unsigned)jsdk_context_size(NULL));
        return 1;
    }

    jsdk_context_config_default(cfg);
    cfg->hal       = a->sdk_hal;
    cfg->master_id = a->o.master_id;
    cfg->is_fd     = (uint8_t)a->fd;
    /* 显式指定的格式优先：只写 --if/--channel 时保持“可自动对齐”（见 joint_sdk.h） */
    cfg->is_fd_explicit = (uint8_t)(a->o.fd_explicit ? 1u : 0u);
    /* `period_ns` 是 **uint32_t**（ns 计，上限 ~4.29 s）；
       这里不要多此一举地转成 uint64_t —— `-Wconversion` 会正确地报
       "long unsigned → uint32_t 可能丢值"（rate_hz 已校验 1..1000）。

       ⚠ **只给真跑循环的命令声明周期**（理由见 `cli_cmd_needs_loop()`）：
         SDK 有一条安全闸“周期 >= 设备 break_timeout 就拒绝 configure()”
         （因为那样的循环喂不了协议看门狗；`break_timeout = 0` = 禁用时该闸**不适用**）。
         而 CLI 默认周期是 10 Hz = 100 ms，历史上真机设备又被默认成 100 ms，
         于是只读诊断命令会**全部**被这条闸拦住（真机实测，已修）。
         只读命令本来就不跑循环，周期对它没有意义。 */
    if (cli_cmd_needs_loop(a->o.sub)) {
        cfg->period_ns = (uint32_t)(1000000000u / (uint32_t)a->o.rate_hz);
    } else {
        cfg->period_ns = 0u;   /* 0 = 不声明控制周期（SDK 跳过那条安全闸） */
    }

    /* 等状态序列跑完的预算（calibrate/home）：0 = SDK 内置默认
       （标定 120 s、回零 5 s）。`--timeout-ms` 可覆盖 —— 真机上全标定要转
       十几圈电气角，实测 >20 s，旧的硬编码 20 s 会在**序列还在跑**时报超时。 */
    cfg->state_timeout_ms = a->o.state_timeout_ms;

    /* arena 先给上（大小按 RETAIN_ALL 的保守推荐值）；下载描述符时再用。
       ⚠ arena 本身要由我们释放（否则每次 run 漏 32 KB）；cfg 由 a 持有。 */
    cfg->desc.mode       = JSDK_DESC_DYNAMIC;
    cfg->desc.retain     = JSDK_DESC_RETAIN_ALL;
    cfg->desc.timeout_ms = a->o.timeout_ms;
    a->arena = calloc(1u, jsdk_desc_arena_size(&cfg->desc));
    if (!a->arena) {
        cli_fprintf(a->err, "jsdk-cli: 内存不足（描述符 arena）\n");
        return 1;
    }
    cfg->desc.arena      = a->arena;
    cfg->desc.arena_size = jsdk_desc_arena_size(&cfg->desc);

    st = jsdk_context_init(a->ctx, cfg);
    if (st != JSDK_OK) {
        cli_fprintf(a->err, "jsdk-cli: 初始化上下文失败：%s\n", jsdk_status_string(st));
        return 1;
    }

    memset(&jc, 0, sizeof jc);
    jc.node_id      = a->o.node;
    jc.initial_mode = JSDK_MODE_MIT;
    st = jsdk_context_add_joint(a->ctx, &jc, &a->joint);
    if (st != JSDK_OK) {
        cli_fprintf(a->err, "jsdk-cli: 添加关节 %u 失败：%s\n",
                (unsigned)a->o.node, jsdk_status_string(st));
        return 1;
    }
    return 0;
}

int cli_load_desc(cli_app_t *a, int full)
{
    jsdk_status_t st;
    uint32_t      t0;

    if (!a->ctx) return 1;

    t0 = cli_wall_ms();
    /*
     * 两档：
     *   full = 1 → configure()：描述符 + 标定（要读值的命令）；
     *   full = 0 → 只下描述符（`desc-info`/`ep-list`/`ep-lookup`/`desc-export`）。
     * 为什么必须分：标定会因"标定量超范围"失败，而那些命令根本不需要标定 ——
     * 合在一起会让"最需要看端点表的时候看不到"（真机实测）。
     */
    st = full ? jsdk_context_configure(a->ctx)
              : jsdk_context_desc_fetch(a->ctx);
    if (st != JSDK_OK) {
        cli_fprintf(a->err, "jsdk-cli: %s 失败：%s\n%s\n",
                full ? "configure()" : "desc_fetch()",
                jsdk_status_string(st), jsdk_context_last_error(a->ctx));
        /*
         * 真机上最常见的一条：设备的 break_timeout 比我们的控制周期还短。
         * SDK 的提示只说"周期 >= break_timeout"，这里补上**怎么改**。
         */
        if (st == JSDK_ERR_BAD_STATE && cli_cmd_needs_loop(a->o.sub)) {
            cli_fprintf(a->err,
                    "提示：本命令会跑控制循环，周期 = 1e9/--rate-hz = %u ms，"
                    "而设备侧 break_timeout 是它自己的配置。\n"
                    "      提高 --rate-hz（例如 --rate-hz 100 → 10 ms），"
                    "或把设备的 can.config.break_timeout 调大。\n",
                    (unsigned)(1000u / (unsigned)a->o.rate_hz));
        }
        /* 标定失败时提醒：端点是有的，用 desc-only 命令看现场 */
        if (full && st == JSDK_ERR_PROTOCOL) {
            cli_fprintf(a->err,
                    "提示：描述符已经下完，失败的是**标定**。用 `desc-info` / "
                    "`ep-lookup <path>` / `ep-list` 看设备到底提供了哪些端点。\n");
        }
        return 1;
    }

    if (a->o.verbose) {
        jsdk_desc_info_t di;
        if (jsdk_context_get_desc_info(a->ctx, &di) == JSDK_OK) {
            cli_fprintf(a->err, "jsdk-cli: 描述符 %u 字节 / %u 端点 / %u 帧 / %u ms\n",
                    (unsigned)di.total_len, di.endpoint_count,
                    di.frames_rx, (unsigned)(cli_wall_ms() - t0));
        }
    }
    return 0;
}

void cli_close(cli_app_t *a)
{
    if (a->ctx) {
        /* deactivate() 会 hold_position + STOP_MOTOR + 等 IDLE；未使能时它只是
           destroy。只在有人使能过的时候调用，避免无谓的总线流量。 */
        if (a->joint && jsdk_joint_is_enabled(a->joint)) {
            jsdk_context_deactivate(a->ctx);
        }
        jsdk_context_destroy(a->ctx);
        free(a->ctx);
        a->ctx = NULL;
    }
    if (a->arena) {
        free(a->arena);
        a->arena = NULL;
    }
    if (a->hal) {
        jsdk_hal_close(a->hal);
        a->hal = NULL;
    }
}

jsdk_joint_t *cli_joint(cli_app_t *a)
{
    if (!a->joint) {
        cli_fprintf(a->err, "jsdk-cli: 没有节点 %u\n", (unsigned)a->o.node);
    }
    return a->joint;
}

int cli_should_continue(const cli_app_t *a, uint32_t elapsed_ms)
{
    if (jsdk_cli_stop_requested()) return 0;
    if (a->o.duration_s <= 0) return 1;
    return (elapsed_ms < (uint32_t)a->o.duration_s * 1000u) ? 1 : 0;
}

/* ==========================================================================
 * 输出辅助
 * ======================================================================== */

void cli_error(cli_app_t *a, const char *what, jsdk_status_t st)
{
    const char *detail = a->ctx ? jsdk_context_last_error(a->ctx) : NULL;

    cli_fprintf(a->err, "jsdk-cli: %s 失败：%s\n", what ? what : "操作",
            jsdk_status_string(st));
    if (detail && detail[0]) cli_fprintf(a->err, "         %s\n", detail);
}

void cli_error_json(cli_app_t *a, cli_json_t *j, const char *what, jsdk_status_t st)
{
    const char *detail = a->ctx ? jsdk_context_last_error(a->ctx) : NULL;

    cli_json_obj_begin(j, "error");
    cli_json_str(j, "op", what ? what : "operation");
    cli_json_str(j, "status", jsdk_status_string(st));
    cli_json_i64(j, "status_code", (long long)st);
    cli_json_str(j, "message", (detail && detail[0]) ? detail : "");
    cli_json_obj_end(j);

    cli_fprintf(a->err, "jsdk-cli: %s 失败：%s%s%s\n", what ? what : "操作",
            jsdk_status_string(st),
            (detail && detail[0]) ? " — " : "",
            (detail && detail[0]) ? detail : "");
}

const char *cli_mode_name(jsdk_mode_t m)
{
    const char *s = jsdk_mode_string(m);
    return s ? s : "?";
}
