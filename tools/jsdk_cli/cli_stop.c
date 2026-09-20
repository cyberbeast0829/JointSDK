/**
 * @file    cli_stop.c
 * @brief   "请求停止"标志 —— 属于**核心库**，不属于可执行文件
 *
 * @par 为什么放在这里
 *  早先把这两个函数写在 `cli_main.c`（可执行文件）里，结果静态库
 *  `jsdk_cli_core` 里的 `cli_app.c` / `cli_cmd.c` 就引用了外部符号，
 *  测试链接 `jsdk_cli_core` 时直接 undefined reference。
 *  更重要的是：**测试需要能自己触发停止**（断言 mon/mit 会干净地退出），
 *  而测试不会去链接可执行文件。所以标志必须和逻辑在同一层。
 *
 * @par 信号安全
 *  `jsdk_cli_request_stop()` 只写一个 `volatile sig_atomic_t`，因此可以在
 *  SIGINT 处理函数里调用；真正的安全停车（hold_position → disable）在主循环里做。
 */

#include "jsdk_cli.h"

#include <signal.h>

static volatile sig_atomic_t g_stop = 0;

void jsdk_cli_request_stop(void)
{
    g_stop = 1;
}

int jsdk_cli_stop_requested(void)
{
    return g_stop ? 1 : 0;
}
