/**
 * @file    hal_virtual_internal.h
 * @brief   `jsdk_hal_virtual` 的内部接口（**仅供本工程测试使用**，不安装）
 *
 * 设备模型 `sim_device.h` 并不适合塞进公共头文件（它是桌面测试专用的实现细节），
 * 但测试需要直接检视/操纵模型内部状态（例如断言单位换算结果、检查超时后的
 * 位、配置心跳周期）。因此用这个内部头把桥接函数暴露出来。
 */

#ifndef JSDK_HAL_VIRTUAL_INTERNAL_H
#define JSDK_HAL_VIRTUAL_INTERNAL_H

#include "jsdk_hal_builtin.h"
#include "sim_device.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 取出虚拟后端内部的设备模型。仅在本工程测试中使用。
 * @return 模型指针；@p h 为空时返回 NULL
 */
sim_bus_t *jsdk_hal_virtual_sim(jsdk_hal_handle_t *h);

#ifdef __cplusplus
}
#endif

#endif /* JSDK_HAL_VIRTUAL_INTERNAL_H */
