/**
 * @file    cli_app.h
 * @brief   CLI 的运行环境：选项、HAL 包装、上下文、辅助输出
 *
 * @par HAL 包装（`cli_hal`）
 *  CLI 自己持有传输层，于是可以**在中间加一层**做两件公共 API 做不到的事：
 *   1. 记录最近若干帧的原始字节 → `hb-dump` 能给客户"抓包级"的对照；
 *   2. 对虚拟后端推进模拟时钟 → 所有配置阶段（阻塞）API 在仿真下也能跑完。
 *  SDK 只会看到包装后的回调，对上层完全透明。
 */

#ifndef JSDK_CLI_APP_H
#define JSDK_CLI_APP_H

#include <stdint.h>
#include <stdio.h>

#include "joint_sdk.h"
#include "jsdk_hal_builtin.h"

#include "cli_json.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 最近接收帧的留存深度（`hb-dump` / 诊断用）。 */
#define CLI_RX_RING 16u

/** 命令行选项。 */
typedef struct {
    /* --- 全局 --- */
    const char *ifname;      /**< --if socketcan|pcan|slcan|virtual（NULL → 平台默认） */
    const char *channel;     /**< --channel */
    uint32_t    bitrate;     /**< --bitrate，默认 1000000（CAN 仲裁段） */
    uint32_t    data_bitrate;/**< --data-bitrate，默认 5000000；0 = Classic */
    uint32_t    baud;        /**< --baud，**串口**波特率（仅 slcan），默认 115200。
                                  ⚠ 与 `--bitrate` 是两个完全不同的量：slcan 适配器
                                  的串口波特率通常是 115200/1000000，而 CAN 段波特率
                                  由适配器自己按 CAN 帧速率跑。混用会收到乱码。 */
    int         classic;     /**< --classic */
    uint8_t     master_id;   /**< --master-id，默认 1 */
    uint8_t     node;        /**< --node，默认 1 */
    int         json;        /**< --json */
    int         rate_hz;     /**< --rate-hz，默认 10 */
    int         duration_s;  /**< --duration，0 = 直到 Ctrl-C */
    int         verbose;     /**< -v */
    int         quiet;       /**< -q */

    /* --- 安全 --- */
    int         yes;         /**< --yes：写/动的前置条件 */
    int         hold_s;      /**< --hold（秒），`mit` 必需，上限 60 */

    /* --- 子命令专用 --- */
    unsigned    max_probe;   /**< --probe，主动探测上限，默认 16 */
    uint32_t    timeout_ms;  /**< --timeout，单次操作超时，默认 3000 */
    int         csv;         /**< --csv：**输出格式开关**（CSV 代替 NDJSON/表格） */
    const char *csv_file;    /**< --csv-file <file>：额外把同一份 CSV 写到文件 */
    const char *filter;      /**< --filter <prefix>（ep-list） */
    double      pos, vel, kp, kd, tau;
    int         have[5];     /**< 各 MIT 分量是否给出过（含显式 0） */
    double      kp_stiff;    /**< --stiffness（输出端真实刚度，替代 --kp） */
    int         have_stiff;

    /* --- 位置参数 --- */
    const char *sub;         /**< 子命令名 */
    const char *args[8];     /**< 位置参数 */
    unsigned    nargs;
} cli_opts_t;

/** CLI 运行实例。 */
typedef struct {
    cli_opts_t      o;
    FILE           *out;
    FILE           *err;

    jsdk_hal_handle_t *hal;      /**< 真实后端句柄 */
    jsdk_can_hal_t     user_hal; /**< 后端返回的回调 */
    jsdk_can_hal_t     sdk_hal;  /**< 包装后交给 SDK 的回调 */

    jsdk_context_t *ctx;         /**< 堆模式创建的上下文 */
    void           *arena;       /**< 描述符 arena（与 cfg.desc 同寿命，需自己释放） */
    jsdk_joint_t   *joint;

    /**
     * 上下文的配置**必须活到 `jsdk_context_destroy()`**。
     *
     * ⚠ 不能改成 `cli_init_ctx()` 里的局部变量：`jsdk_context_init()` 会**复制**
     *   这份配置，同时把 `&cfg.desc.arena_used` 存下来当回写槽位（该字段是输出，
     *   `arena_used_slot`）—— 局部变量一返回，这个指针就是悬垂的，之后
     *   `configure()` → `desc_fetch()` → `publish_arena_used()` 会**往已经
     *   不属于本函数的栈帧里写 8 字节**。
     *   Linux + AddressSanitizer 实测当场报 `stack-buffer-underflow`
     *   （旧实现就是局部变量，见 DESIGN v0.17）。
     */
    jsdk_context_config_t cfg;

    /* 包装层的计数与最近帧 */
    uint32_t tx_frames;
    uint32_t rx_frames;
    jsdk_can_frame_t rx_ring[CLI_RX_RING];
    unsigned rx_ring_n;

    uint32_t virt_ms;            /**< 虚拟时钟（仅 virtual 后端） */
    int      is_virtual;
    int      fd;                 /**< 生效的 FD 标志 */
} cli_app_t;

/* --------------------------------------------------------------------------
 * 选项解析
 * ------------------------------------------------------------------------ */

/**
 * 解析命令行。**选项可以出现在子命令前后**（设计文档写的是
 * `[全局选项] <子命令> [子命令选项]`，但客户现场按两种顺序敲都很常见，
 * 与其报错不如都接受）。
 * @return 0 = 成功；非 0 = 用法错误（已打印到 @p err）
 */
int cli_opts_parse(cli_opts_t *o, int argc, char **argv, FILE *err);

/** 打印用法。 */
void cli_usage(FILE *f, const char *argv0);

/* --------------------------------------------------------------------------
 * 运行环境
 * ------------------------------------------------------------------------ */

/**
 * 打开传输层 + 创建上下文 + 添加关节。
 *
 * ⚠ **只做到"上下文可用"**，不下载描述符 —— 因为 `scan` / `estop` 这类命令
 *   不需要端点表，而描述符是 41 KB / 最多 6840 帧的流量，纯诊断命令不该付。
 */
int cli_open(cli_app_t *a);

/**
 * 下载/解析描述符（可选带标定）。
 *
 * @param full 1 = 连标定量一起读（= `jsdk_context_configure()`）；
 *             0 = **只要描述符**（= `jsdk_context_desc_fetch()`）。
 *
 * ⚠ 分成两档的理由：`desc-info` / `ep-list` / `ep-lookup` / `desc-export`
 *   只关心「设备有哪些端点」，**不需要**标定量。而 `configure()` 的最后一步是
 *   标定，它会因为“标定量超范围”而整个失败 —— 于是这些只读诊断命令
 *   在真机上根本用不了（实测：明明描述符已经下完了，却报 calibration 错）。
 *   这在现场是个死结：最需要看端点表的时候（标定失败），恰恰看不到。
 */
int cli_load_desc(cli_app_t *a, int full);

/** 关闭：先安全停车（若使能过），再释放上下文与 HAL。 */
void cli_close(cli_app_t *a);

/** 按 `--node` 找关节（找不到打印错误）。 */
jsdk_joint_t *cli_joint(cli_app_t *a);

/**
 * 子命令对描述符的需求档。
 *
 * - `CLI_DESC_NONE`：不碰描述符（省 ~40 KB 流量）；
 * - `CLI_DESC_ONLY`：只要端点表（`desc-info`/`ep-list`/`ep-lookup`/`desc-export`）；
 * - `CLI_DESC_FULL`：还要标定量（值读写、物理量、使能…）。
 */
#define CLI_DESC_NONE 0
#define CLI_DESC_ONLY 1
#define CLI_DESC_FULL 2

/** 子命令的描述符需求档（未知子命令 → `CLI_DESC_NONE`）。实现在 `cli_cmd.c`。 */
int cli_cmd_desc_mode(const char *sub);

/**
 * 子命令是否会跑**控制/keepalive 循环**。
 *
 * ⚠ 只有跑循环的命令才应向 SDK 声明 `period_ns`：SDK 有一条安全闸
 *   “周期 >= 设备 break_timeout 就拒绝 configure()”（`break_timeout = 0`
 *    = 禁用时该闸不适用），而 CLI 默认周期是 100 ms —— 只读诊断命令
 *   会因此被拦（真机实测，已修）。
 */
int cli_cmd_needs_loop(const char *sub);

/** 睡眠（毫秒），用于 `--rate-hz` 节奏；被 Ctrl-C 请求时提前返回 1。 */
int cli_sleep_ms(unsigned ms);

/** 是否应继续运行（受 `--duration` 与 Ctrl-C 控制）。 */
int cli_should_continue(const cli_app_t *a, uint32_t elapsed_ms);

/* --------------------------------------------------------------------------
 * 输出辅助
 * ------------------------------------------------------------------------ */

/** 统一错误打印："错误: <last_error> (status)"，并给出可选提示。 */
void cli_error(cli_app_t *a, const char *what, jsdk_status_t st);

/** JSON 模式下的一个错误对象 + 人类可读行。 */
void cli_error_json(cli_app_t *a, cli_json_t *j, const char *what, jsdk_status_t st);

/** 把模式/轴状态等公共字符串安全取出（NULL 安全）。 */
const char *cli_mode_name(jsdk_mode_t m);

#ifdef __cplusplus
}
#endif

#endif /* JSDK_CLI_APP_H */
