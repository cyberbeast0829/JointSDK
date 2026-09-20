/**
 * @file    01_hello_virtual.c
 * @brief   最小可运行示例：不用硬件、不用描述符，先把"库能跑"这件事确认掉
 *
 * 学到的三件事：
 *   1. `jsdk_backend_name()` / `jsdk_abi_types()` —— 客户程序启动时的自检；
 *   2. `jsdk_hal_virtual_open()` 不需要任何硬件，设备模型是库自带的；
 *   3. 关句柄用 `jsdk_hal_close()`（**不要 free**：真实后端要关 fd/串口）。
 *
 * 构建与运行（仓库根）：
 *     cmake -S . -B build -DJSDK_BUILD_EXAMPLES=ON && cmake --build build -j
 *     ./build/examples/01_hello_virtual
 */

#include <stdio.h>
#include <string.h>

#include "joint_sdk.h"
#include "jsdk_hal_builtin.h"

int main(void)
{
    jsdk_can_hal_t     hal;
    jsdk_hal_handle_t *hh = NULL;
    size_t             n_types = 0u;
    const jsdk_abi_type_t *types;
    jsdk_status_t      st;
    uint32_t           flags = 0u;

    /* --- 1. ABI 自检 ------------------------------------------------ */
    printf("backend      : %s\n", jsdk_backend_name());
    printf("context RAM  : %u bytes/上下文（JSDK_CONTEXT_MAX_SIZE，调用者提供）\n",
           (unsigned)JSDK_CONTEXT_MAX_SIZE);
    printf("max joints   : %u\n", (unsigned)JSDK_MAX_JOINTS_STATIC);

    types = jsdk_abi_types(&n_types);
    if (!types || n_types == 0u) {
        fprintf(stderr, "abi_types() 返回空 —— 头文件与库版本不匹配？\n");
        return 1;
    }
    printf("abi types    : %lu 个\n", (unsigned long)n_types);
    {
        size_t i;
        /* 结构体布局是 ABI 契约的一部分：客户可以把它打到日志里，出问题时用来对账 */
        for (i = 0u; i < 3u && i < n_types; ++i) {
            printf("  %-24s size=%-4u align=%u\n", types[i].name,
                   (unsigned)types[i].size, (unsigned)types[i].align);
        }
    }

    /* --- 2. 打开虚拟后端（自带一个简化固件模型） --------------------- */
    st = jsdk_hal_virtual_open(&hal, &hh, "0:id=1,gear=16.5,fd");
    if (st != JSDK_OK) {
        fprintf(stderr, "hal_virtual_open -> %s\n", jsdk_status_string(st));
        return 1;
    }
    printf("hal          : 虚拟后端已打开（未连任何硬件）\n");

    /* --- 3. 读一次链路状态 ------------------------------------------ */
    if (hal.bus_status && hal.bus_status(hal.user, &flags) == 0) {
        printf("bus          : link_up=%d\n", (flags & JSDK_HAL_BUS_OK) ? 1 : 0);
    }

    if (jsdk_hal_close(hh) != JSDK_OK) {
        fprintf(stderr, "hal_close 失败\n");
        return 1;
    }
    printf("done\n");
    return 0;
}
