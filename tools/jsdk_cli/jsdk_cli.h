/**
 * @file    jsdk_cli.h
 * @brief   `jsdk-cli` 的入口（WP7）
 *
 * @par 为什么把 `main()` 拆成 `jsdk_cli_run()`
 *  自动化测试要断言"`--json` 输出里有这些字段"、"`mit` 没有 `--yes` 被拒绝"。
 *  如果逻辑全在 `main()` 里，就只能起子进程 + 抓 stdout，在 Windows 上还要处理
 *  路径与引号。把入口做成一个接受 `FILE *out / *err` 的函数后，测试可以**同进程**
 *  调用并断言输出，命令行解析、子命令、退出码全部覆盖。
 *  `cli_main.c` 里的真 `main()` 只负责装 SIGINT 处理并转调本函数。
 */

#ifndef JSDK_CLI_H
#define JSDK_CLI_H

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 执行一次 CLI 调用。
 *
 * @param argc 参数个数（含 argv[0]）
 * @param argv 参数
 * @param out  正常输出（`--json` 的 JSON 也写到这里）
 * @param err  诊断/错误输出
 * @return 进程退出码：0 = 成功；1 = 运行时错误；2 = 用法错误；
 *         3 = 被 `--yes` 之类的安全闸拒绝
 */
int jsdk_cli_run(int argc, char **argv, FILE *out, FILE *err);

/**
 * 请求停止（SIGINT 处理函数调用，或测试直接调用）。
 * 运行中的 `mon` / `mit` 会在本周期结束后退出。
 * @warning 信号处理函数里唯一允许的动作就是调用本函数。
 */
void jsdk_cli_request_stop(void);

/** @return 1 = 已请求停止 */
int jsdk_cli_stop_requested(void);

#ifdef __cplusplus
}
#endif

#endif /* JSDK_CLI_H */
