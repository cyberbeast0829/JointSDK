/**
 * @file    test_cli.c
 * @brief   WP7：`jsdk-cli` 的自动化验收（设计文档 §8 的「CLI」行）
 *
 * 做法：**同进程**调用 `jsdk_cli_run(argc, argv, out, err)`，把输出重定向到
 * `tmpfile()`，然后断言：
 *   - 全部只读子命令都能在 virtual 后端跑通（退出码 0）；
 *   - `--json` 输出里含关键字段名（Python 绑定与 CI 会依赖这些名字）；
 *   - 写/动类子命令加 `--yes` 能跑，不加就**被拒绝**（退出码 3）；
 *   - `mit` 缺 `--hold` 被拒绝；`--hold 1` 到期后自动 hold + disable。
 *
 * 不做的部分：真机收发（见 docs/PORTING.zh-CN.md 的手工冒烟清单）。
 *
 * ⚠ **不要手数 argc**。本文件第一版把命令行写成字符串宏、argc 手写常量，
 *   而那个宏里有相邻字符串字面量会**拼接成一个**，于是实际元素比手写的少 1 →
 *   `argv[]` 末尾是 NULL → `t[0]` 直接段错误（且因为 stdout 是重定向到文件的
 *   全缓冲，连一行错误都看不到）。现在统一用 `RUN_CLI()`：argc 由
 *   `sizeof` 算出来，永远对得上。
 *
 * ⚠ 每次 `RUN_CLI()` 都新建一条虚拟总线 + 一个仿真设备（CLI 自己 open/close）。
 *   用例之间**没有共享状态**，每个用例必须自给自足 —— 顺带验证"CLI 单次调用
 *   不依赖任何全局状态"。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "jsdk_cli.h"

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

/* 统一的"虚拟后端"：单节点、FD、量程与仿真设备默认值一致。
   hb=10 让心跳/发现很快到位；timeout=30000 给配置阶段留足时间。 */
#define CH "0:id=1,gear=16.5,pmax=12.5,vmax=65,tmax=50,kpmax=500,kdmax=5," \
           "hb=10,timeout=30000,fd"
#define VIF "--if", "virtual", "--channel", CH

/* --------------------------------------------------------------------------
 * 运行辅助
 * ------------------------------------------------------------------------ */

typedef struct {
    char out[32768];
    char err[8192];
    int  rc;
} run_t;

/**
 * 跑一次 CLI。
 *
 * ⚠ 用 `tmpfile()` 而不是 `open_memstream()`：后者是 POSIX-2008，MinGW 没有。
 *   `tmpfile()` 两端都有，`rewind()` + `fread()` 就能取回全部输出。
 */
static void run_args(run_t *r, const char *const *argv, int argc)
{
    FILE  *fo = tmpfile();
    FILE  *fe = tmpfile();
    char **av;
    int    i;
    size_t n;

    memset(r, 0, sizeof *r);
    if (!fo || !fe) {
        g_fail++; g_checks++;
        printf("  FAIL: tmpfile() 失败\n");
        return;
    }

    av = (char **)calloc((size_t)argc + 1u, sizeof *av);
    if (!av) { fclose(fo); fclose(fe); g_fail++; g_checks++; return; }
    for (i = 0; i < argc; ++i) {
        if (!argv[i]) {                   /* 传进来的列表比 argc 短：直接暴露 */
            printf("  FAIL: argv[%d] 为 NULL（argc=%d）\n", i, argc);
            g_fail++; g_checks++;
            free(av); fclose(fo); fclose(fe);
            return;
        }
        av[i] = (char *)argv[i];
    }

    r->rc = jsdk_cli_run(argc, av, fo, fe);
    free(av);

    rewind(fo);
    n = fread(r->out, 1u, sizeof r->out - 1u, fo);
    r->out[n] = '\0';
    fclose(fo);

    rewind(fe);
    n = fread(r->err, 1u, sizeof r->err - 1u, fe);
    r->err[n] = '\0';
    fclose(fe);
}

/** 用可变参数列表跑一次 CLI，**argc 自动推导**（不再手数）。 */
#define RUN_CLI(r, ...)                                                      \
    run_args((r),                                                            \
             (const char *const[]){ "jsdk-cli", __VA_ARGS__ },                \
             (int)(sizeof((const char *const[]){ "jsdk-cli", __VA_ARGS__ })   \
                   / sizeof(const char *)))

/** 断言 out/err 里含某子串；失败时打印整个输出便于定位。 */
static void expect_has(const run_t *r, const char *where, const char *needle)
{
    const char *hay = (strcmp(where, "err") == 0) ? r->err : r->out;

    g_checks++;
    if (!strstr(hay, needle)) {
        g_fail++;
        printf("  FAIL: %s 里没有 \"%s\"\n      --- %s ---\n%s\n",
               where, needle, where, hay);
    }
}

/* ==========================================================================
 * 1. 用法与参数校验
 * ======================================================================== */

static void test_usage(void)
{
    run_t r;

    printf("[1] usage / argument validation\n");

    RUN_CLI(&r, "--help");
    CHECK(r.rc == 0);                       /* --help 是成功，不是错误 */
    expect_has(&r, "err", "用法:");

    RUN_CLI(&r);
    CHECK(r.rc == 2);
    expect_has(&r, "err", "缺少子命令");

    RUN_CLI(&r, "--if", "virtual", "nosuchcmd");
    CHECK(r.rc == 2);
    expect_has(&r, "err", "未知子命令");

    RUN_CLI(&r, "--bitrate", "abc", "scan");
    CHECK(r.rc == 2);
    expect_has(&r, "err", "--bitrate 的值非法");

    RUN_CLI(&r, "--master-id", "0", "scan");
    CHECK(r.rc == 2);                       /* 主站 0 → 设备完全不回复 */
    expect_has(&r, "err", "--master-id 的值非法");

    RUN_CLI(&r, "--if", "nosuchbus", "scan");
    CHECK(r.rc == 2);
    expect_has(&r, "err", "未知后端");

    /* 选项出现在子命令之后也要认（现场两种顺序都会敲） */
    RUN_CLI(&r, "scan", VIF);
    CHECK(r.rc == 0);

    /* `--opt=value` 形式 */
    RUN_CLI(&r, "--if=virtual", "--channel", CH, "--node=1", "health");
    CHECK(r.rc == 0);

    /* 未知选项要报出来，而不是被当成位置参数 */
    RUN_CLI(&r, VIF, "--nosuchopt", "health");
    CHECK(r.rc == 2);
    expect_has(&r, "err", "未知选项");

    printf("      help / missing sub / bad values / bad backend / option order\n");
}

/* ==========================================================================
 * 2. 只读子命令
 * ======================================================================== */

static void ro_ok(const char *label, run_t *r, const char *needle)
{
    if (r->rc != 0) {
        printf("  FAIL %s: 退出码 %d（期望 0）\n      --- err ---\n%s\n",
               label, r->rc, r->err);
        g_fail++; g_checks++;
        return;
    }
    g_checks++;
    if (needle && !strstr(r->out, needle)) {
        printf("  FAIL %s: 输出里没有 \"%s\"\n      --- out ---\n%s\n",
               label, needle, r->out);
        g_fail++;
    }
    printf("      %-26s ok\n", label);
}

static void test_readonly(void)
{
    run_t r;

    printf("[2] read-only subcommands (virtual backend)\n");

    RUN_CLI(&r, VIF, "scan");
    ro_ok("scan", &r, "发现 1 个节点");

    RUN_CLI(&r, VIF, "info");
    ro_ok("info", &r, "hw_version");

    RUN_CLI(&r, VIF, "health");
    ro_ok("health", &r, "总线");

    RUN_CLI(&r, VIF, "read", "axis0.controller.config.mit_max_torque");
    ro_ok("read", &r, "50");

    /*
     * 8 字节端点（u64）：单读要能读满（FD 一次请求；Classic 自动分两块）。
     * 修复前这里是 `got 4 bytes, descriptor says 8` —— 排障时最想读的
     * `axis0.motor.error` / `serial_number` 恰好都是 u64。
     */
    RUN_CLI(&r, VIF, "read", "serial_number");
    ro_ok("read", &r, "1234605616436508552");

    RUN_CLI(&r, VIF, "read", "axis0.motor.error");
    ro_ok("read", &r, "0");

    /*
     * ⚠ **默认**虚拟规格路径（不给 `--channel`）必须也能用。
     * 这条路径原先一直是坏的：默认规格漏了 `timeout=`，设备于是用固件默认的
     * 100 ms break_timeout，而 CLI 默认控制周期也是 100 ms → `configure()`
     * 直接 `bad-state`，表现为“不加 --channel 就什么都干不了”。
     * 本文件的其它用例都传了 VIF（带 --channel），所以从未覆盖到这里。
     */
    RUN_CLI(&r, "--if", "virtual", "read", "axis0.motor.config.gear_ratio");
    ro_ok("default-if-read", &r, "16.5");

    RUN_CLI(&r, "--if", "virtual", "dump-config");
    ro_ok("default-if-dump", &r, "gear_ratio");

    RUN_CLI(&r, "--if", "virtual", "health");
    ro_ok("default-if-health", &r, "总线");

    RUN_CLI(&r, VIF, "batch-read", "axis0.motor.config.gear_ratio",
            "axis0.controller.config.mit_max_kp");
    ro_ok("batch-read", &r, "gear_ratio");

    RUN_CLI(&r, VIF, "dump-config");
    ro_ok("dump-config", &r, "gear_ratio");

    RUN_CLI(&r, VIF, "err");
    ro_ok("err", &r, "axis");

    RUN_CLI(&r, VIF, "desc-info");
    ro_ok("desc-info", &r, "endpoint_count");

    RUN_CLI(&r, VIF, "ep-list");
    ro_ok("ep-list", &r, "axis0");

    RUN_CLI(&r, VIF, "ep-lookup", "axis0.controller.config.mit_max_kd");
    ro_ok("ep-lookup", &r, "type=");

    /* mon 必须有界运行，否则测试会挂住 */
    RUN_CLI(&r, VIF, "--rate-hz", "200", "--duration", "1", "--quiet", "mon");
    ro_ok("mon --duration 1", &r, NULL);

    /*
     * hb-dump：这条命令必须**自己等**一个心跳。
     * 早期版本只看"HAL 包装层当前已留存的帧"，而每次进程只跑一条命令 ——
     * 于是单跑 hb-dump 永远报"没有缓存到心跳"，客户会以为命令坏了。
     */
    RUN_CLI(&r, VIF, "hb-dump");
    ro_ok("hb-dump (waits for a heartbeat)", &r, "src=1");

    RUN_CLI(&r, VIF, "--json", "hb-dump");
    CHECK(r.rc == 0);
    expect_has(&r, "out", "\"heartbeats\"");
    expect_has(&r, "out", "\"bytes\"");
    expect_has(&r, "out", "\"count\"");
}

/* ==========================================================================
 * 3. --json 的字段名（Python 绑定与 CI 依赖它们）
 * ======================================================================== */

static void test_json(void)
{
    run_t r;

    printf("[3] --json field names\n");

    RUN_CLI(&r, VIF, "--json", "health");
    CHECK(r.rc == 0);
    expect_has(&r, "out", "\"joint\"");
    expect_has(&r, "out", "\"bus\"");
    expect_has(&r, "out", "\"pos_rad\"");
    expect_has(&r, "out", "\"axis_state\"");
    expect_has(&r, "out", "\"age_ms\"");
    expect_has(&r, "out", "\"link_up\"");

    RUN_CLI(&r, VIF, "--json", "dump-config");
    CHECK(r.rc == 0);
    expect_has(&r, "out", "\"gear_ratio\"");
    expect_has(&r, "out", "\"mit_max_torque\"");
    expect_has(&r, "out", "\"torque_constant\"");
    expect_has(&r, "out", "\"break_timeout_ms\"");

    RUN_CLI(&r, VIF, "--json", "read", "axis0.motor.config.gear_ratio");
    CHECK(r.rc == 0);
    expect_has(&r, "out", "\"path\"");
    expect_has(&r, "out", "\"value\"");
    expect_has(&r, "out", "\"type\"");

    RUN_CLI(&r, VIF, "--json", "--filter", "mit_max_", "ep-list");
    CHECK(r.rc == 0);
    expect_has(&r, "out", "\"endpoints\"");
    expect_has(&r, "out", "mit_max_torque");
    expect_has(&r, "out", "\"count\"");

    RUN_CLI(&r, VIF, "--json", "scan");
    CHECK(r.rc == 0);
    expect_has(&r, "out", "\"nodes\"");
    expect_has(&r, "out", "\"count\"");

    printf("      health / dump-config / read / ep-list / scan keys present\n");
}

/* ==========================================================================
 * 4. 安全闸：--yes / --hold
 * ======================================================================== */

static void test_safety_gates(void)
{
    run_t r;

    printf("[4] safety gates\n");

    /* --- 写类命令无 --yes → 拒绝，退出码 3 --- */
    RUN_CLI(&r, VIF, "write", "axis0.motor.config.gear_ratio", "8");
    CHECK(r.rc == 3);
    expect_has(&r, "err", "--yes");
    expect_has(&r, "err", "拒绝执行");

    RUN_CLI(&r, VIF, "save");
    CHECK(r.rc == 3);

    RUN_CLI(&r, VIF, "set-zero");
    CHECK(r.rc == 3);

    RUN_CLI(&r, VIF, "calibrate");
    CHECK(r.rc == 3);

    RUN_CLI(&r, VIF, "home");
    CHECK(r.rc == 3);

    RUN_CLI(&r, VIF, "watchdog", "200");
    CHECK(r.rc == 3);

    /* --- mit：先要 --yes，再要 --hold --- */
    RUN_CLI(&r, VIF, "mit", "--pos", "0", "--hold", "1");
    CHECK(r.rc == 3);
    expect_has(&r, "err", "--yes");

    RUN_CLI(&r, VIF, "--yes", "mit", "--pos", "0");
    CHECK(r.rc == 3);
    expect_has(&r, "err", "--hold");

    RUN_CLI(&r, VIF, "--yes", "mit", "--hold", "0");
    CHECK(r.rc == 2);                 /* 0 秒是非法值，不是"拒绝" */
    expect_has(&r, "err", "1..60");

    RUN_CLI(&r, VIF, "--yes", "mit", "--hold", "61");
    CHECK(r.rc == 2);                 /* 上限 60 s */

    /* --- JSON 模式下拒绝也要是结构化输出 --- */
    RUN_CLI(&r, VIF, "--json", "save");
    CHECK(r.rc == 3);
    expect_has(&r, "out", "\"refused\"");
    expect_has(&r, "out", "\"reason\"");

    printf("      write/save/set-zero/calibrate/home/watchdog/mit gated by --yes\n");
    printf("      mit additionally requires a valid --hold (1..60 s)\n");
}

/* ==========================================================================
 * 5. 写路径
 * ======================================================================== */

static void test_write_path(void)
{
    run_t r;

    printf("[5] write path\n");

    RUN_CLI(&r, VIF, "--json", "--yes", "write",
            "axis0.motor.config.gear_ratio", "8");
    if (r.rc != 0) {
        printf("  FAIL write: rc=%d\n      --- err ---\n%s\n", r.rc, r.err);
        g_fail++; g_checks++;
    } else {
        g_checks++;
        expect_has(&r, "out", "\"written\"");
        expect_has(&r, "out", "\"value\"");
    }

    /* 类型不符必须在客户端就被挡住（不静默截断） */
    RUN_CLI(&r, VIF, "--yes", "write",
            "axis0.motor.config.gear_ratio", "notanumber");
    CHECK(r.rc != 0);
    expect_has(&r, "err", "不是合法的");

    /* 不存在的路径 → 明确失败，不猜 */
    RUN_CLI(&r, VIF, "--yes", "write", "axis0.no.such.path", "1");
    CHECK(r.rc != 0);

    printf("      write + client-side type validation + not-found\n");
}

/* ==========================================================================
 * 6. mit 的完整生命周期（唯一会驱动电机的命令）
 * ======================================================================== */

static void test_mit(void)
{
    run_t r;

    printf("[6] mit --hold 1 (enable → drive → auto hold+disable)\n");

    RUN_CLI(&r, VIF, "--yes", "--rate-hz", "200", "mit",
            "--pos", "0.1", "--vel", "0", "--kp", "2", "--kd", "0.2",
            "--tau", "0", "--hold", "1");
    if (r.rc != 0) {
        printf("  FAIL mit: rc=%d\n      --- err ---\n%s\n", r.rc, r.err);
        g_fail++; g_checks++;
    } else {
        g_checks++;
        /* 安全要求：执行前必须把将发送的量和量程打出来 */
        expect_has(&r, "err", "量程");
        expect_has(&r, "out", "disable");
    }

    /* --stiffness 走"输出端真实刚度"换算路径 */
    RUN_CLI(&r, VIF, "--yes", "--rate-hz", "200", "mit",
            "--pos", "0", "--stiffness", "10", "--hold", "1");
    CHECK(r.rc == 0);
    expect_has(&r, "err", "stiffness");

    printf("      hold expiry triggers hold_position + disable\n");
}

/* ==========================================================================
 * 7. 描述符缓存导出/导入
 * ======================================================================== */

static void test_desc_cache(void)
{
    run_t r;
    const char *file = "test_cli_desc_cache.bin";

    printf("[7] desc-export / desc-import\n");

    RUN_CLI(&r, VIF, "desc-export", file);
    CHECK(r.rc == 0);
    expect_has(&r, "out", "已导出");

    /*
     * 导入必须**不依赖下载**就能得到可用的端点表 —— 这正是缓存存在的理由。
     * （早期把 desc-import 标成 needs_desc=1，它会先 configure() 下 41 KB
     *    再导入，把自己存在的意义抹掉了。）
     */
    RUN_CLI(&r, VIF, "--json", "desc-import", file);
    CHECK(r.rc == 0);
    expect_has(&r, "out", "\"imported\"");
    expect_has(&r, "out", "\"endpoint_count\"");
    expect_has(&r, "out", "\"downloaded\":false");

    remove(file);

    printf("      export → import round-trip ok\n");
}

/* ==========================================================================
 * 8. 系统管理类命令：estop / reset / set-node-id
 *
 * 为什么单列一组：这三个命令原先**只有 "无 --yes → 拒绝" 这条路径被测过**
 * （见 [4] safety gates），**执行路径一次都没跑过** —— 而它们都会改设备状态
 * （ESTOP 停机、reset 复位、set-node-id 换地址），恰好是最该有回归的一类。
 *
 * set-node-id 尤其值得测：SDK 在发完之后还会**验证新地址能应答**，否则不改本地
 * node_id（否则本地就把自己弄失联了）。仿真设备会真的换号并在新号上应答，
 * 所以这条验证能跑通 —— 如果哪天固件/仿真不再应答新号，这里会变红。
 * ======================================================================== */

static void test_system_cmds(void)
{
    run_t r;

    printf("[8] estop / reset / set-node-id\n");

    /* --- estop：**故意不要求 `--yes`**（拒绝执行反而更危险），必须能直接跑通 --- */
    RUN_CLI(&r, VIF, "--json", "estop");
    CHECK(r.rc == 0);
    expect_has(&r, "out", "\"estop_sent\":true");

    RUN_CLI(&r, VIF, "estop");
    CHECK(r.rc == 0);
    expect_has(&r, "out", "ESTOP");

    /* --- reset：写设备 → 无 `--yes` 必须拒绝（退出码 3） --- */
    RUN_CLI(&r, VIF, "reset");
    CHECK(r.rc == 3);
    expect_has(&r, "err", "拒绝执行");

    RUN_CLI(&r, VIF, "--json", "--yes", "reset");
    CHECK(r.rc == 0);
    expect_has(&r, "out", "\"reset\":true");

    /* --- set-node-id：三道校验（--yes / 参数个数 / 取值域） --- */
    RUN_CLI(&r, VIF, "set-node-id", "5");
    CHECK(r.rc == 3);                       /* 没 --yes */
    expect_has(&r, "err", "--yes");

    RUN_CLI(&r, VIF, "--yes", "set-node-id");
    CHECK(r.rc == 2);                       /* 缺参数是用法错误，不是"拒绝" */
    expect_has(&r, "err", "\u9700\u8981 <N>");

    RUN_CLI(&r, VIF, "--yes", "set-node-id", "0");
    CHECK(r.rc == 2);                       /* 0 是保留地址 */
    RUN_CLI(&r, VIF, "--yes", "set-node-id", "255");
    CHECK(r.rc == 2);                       /* 上限 254 */
    expect_has(&r, "err", "1..254");
    RUN_CLI(&r, VIF, "--yes", "set-node-id", "abc");
    CHECK(r.rc == 2);                       /* 非数字 */

    /* --- 执行路径：仿真设备真的换号并在新号上应答（SDK 会验证这一点） --- */
    RUN_CLI(&r, VIF, "--json", "--yes", "set-node-id", "5");
    if (r.rc != 0) {
        printf("  FAIL set-node-id: rc=%d\n      --- out ---\n%s\n      --- err ---\n%s\n",
               r.rc, r.out, r.err);
        g_fail++; g_checks++;
    } else {
        g_checks++;
        expect_has(&r, "out", "\"old_node\":1");
        expect_has(&r, "out", "\"new_node\":5");
        expect_has(&r, "out", "\"persisted\":true");
    }

    /* 非 JSON 下必须提示"后续命令请用 --node <新号>" ——
       改了号还照旧用 `--node 1` 是现场最常见的错误。 */
    RUN_CLI(&r, VIF, "--yes", "set-node-id", "7");
    CHECK(r.rc == 0);
    expect_has(&r, "out", "--node 7");

    /* 同一总线上**已有设备**占用了目标号 → 必须提前拦住（不让总线打架）。
       注意这是**总线级**冲突：CLI 只往上下文里加自己关心的那个关节，
       所以 SDK 的"本上下文内冲突"检查看不到另一台设备 ——
       靠改号前的那次定向探测才拦得住（v0.19 修的，否则会静默造出两个同号设备）。
       ⚠ 多节点规格里 `timeout=` 是**逐节点**的（它是设备自己的 break_timeout）：
       这里两个节点都写 30000 —— 不写的话设备就是 `0` = **禁用超时**（新固件默认），
       那也能跑，但就测不到“两个节点都真的配好”这件事了。 */
    RUN_CLI(&r, "--if", "virtual", "--channel",
            "0:id=1,timeout=30000,fd;1:id=2,timeout=30000,fd",
            "--node", "1", "--yes", "set-node-id", "2");
    CHECK(r.rc != 0);
    expect_has(&r, "err", "already answers");

    printf("      estop executes without --yes; reset/set-node-id gated;\n"
           "      set-node-id validation (1..254) + new-address verification\n");
}

/* ==========================================================================
 * 9. mon 的 CSV 契约（两版 CLI 共用）
 * ======================================================================== */

/**
 * ⚠ 这一组是从一次**真实事故**补出来的：`--csv` 以前是"要一个文件名"，于是
 *   `mon --csv --duration 1` 把 `--duration` 当成了文件名，**静默写出一个叫
 *   `--duration` 的 CSV**（在仓库根目录躺了几天才被发现）。
 *   现在：`--csv` 是格式开关（→ stdout），`--csv-file F` 落盘，且取值以 `-`
 *   开头时**当场拒绝**（返回用法错）。
 */
static void test_mon_csv(void)
{
    run_t r;
    const char *hdr = "t_ms,node,pos_rad,vel_rad_s,current_A,torque_Nm,";

    printf("[9] mon 的 CSV 契约（--csv 是开关，--csv-file 落盘）\n");

    RUN_CLI(&r, VIF, "--node", "1", "--csv", "--duration", "1",
            "--rate-hz", "20", "mon");
    CHECK(r.rc == 0);
    expect_has(&r, "out", hdr);
    expect_has(&r, "out", "t_fet_C,vbus_V,ibus_A");   /* 17 列的完整契约 */

    {
        const char *path = "cli_mon_csv_test.csv";
        FILE *f;
        char  line[256];

        remove(path);
        RUN_CLI(&r, VIF, "--node", "1", "--csv-file", path, "--duration", "1",
                "--rate-hz", "20", "mon");
        CHECK(r.rc == 0);
        f = fopen(path, "r");
        CHECK(f != NULL);
        if (f) {
            CHECK(fgets(line, sizeof line, f) != NULL);
            CHECK(strncmp(line, hdr, strlen(hdr)) == 0);
            CHECK(fgets(line, sizeof line, f) != NULL);   /* 至少一行数据 */
            fclose(f);
        }
        remove(path);
    }

    /* 拿到另一个选项 → 用法错（而不是静默写出怪文件名） */
    RUN_CLI(&r, VIF, "--csv-file", "--duration", "1", "mon");
    CHECK(r.rc == 2);
    expect_has(&r, "err", "另一个选项");
}

/* ==========================================================================
 * main
 * ======================================================================== */

int main(void)
{
    printf("=== WP7 tests (jsdk-cli) ===\n\n");

    test_usage();
    printf("\n");
    test_readonly();
    printf("\n");
    test_json();
    printf("\n");
    test_safety_gates();
    printf("\n");
    test_write_path();
    printf("\n");
    test_mit();
    printf("\n");
    test_desc_cache();
    printf("\n");
    test_system_cmds();
    printf("\n");
    test_mon_csv();

    printf("\n=== %u checks, %u failures ===\n", g_checks, g_fail);
    return (g_fail == 0u) ? 0 : 1;
}
