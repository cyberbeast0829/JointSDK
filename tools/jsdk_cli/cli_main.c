/**
 * @file    cli_main.c
 * @brief   `jsdk-cli` 的可执行入口
 *
 * 逻辑一律在 `jsdk_cli_run()`（见 jsdk_cli.h），停止标志在 `cli_stop.c`
 * （它属于核心库，因为测试也要能触发停止）。本文件只做两件事：
 *   1. 把 SIGINT/SIGTERM 变成 `jsdk_cli_request_stop()`；
 *   2. 让 stdout/stderr 尽快落盘（现场重定向到文件时更容易拿到最后一屏）。
 */

#include "jsdk_cli.h"

#include <signal.h>
#include <stdio.h>

static void on_signal(int sig)
{
    /*
     * ⚠ 信号处理函数里只能做这种"置一个 flag"的事。真正的安全停车
     *   （hold_position → disable）在主循环里做 —— 那是 SDK 调用，可能要走
     *   总线，在信号上下文里执行是不可接受的。
     */
    (void)sig;
    jsdk_cli_request_stop();
}

int main(int argc, char **argv)
{
    int rc;

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    rc = jsdk_cli_run(argc, argv, stdout, stderr);

    fflush(stdout);
    fflush(stderr);
    return rc;
}
