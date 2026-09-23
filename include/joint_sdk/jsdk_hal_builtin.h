/**
 * @file    jsdk_hal_builtin.h
 * @brief   内置传输后端工厂（CYBERBEAST CAN 后端，可选编译）
 *
 * @details
 *  本头文件与 `joint_sdk.h` **相互独立**，仅在选择以下 CMake 选项时安装：
 *      JSDK_BUILD_HAL_SOCKETCAN    Linux  SocketCAN（含 CAN-FD + BRS）
 *      JSDK_BUILD_HAL_PCAN         Windows / macOS  PEAK PCAN-Basic
 *      JSDK_BUILD_HAL_SLCAN        serial slcan（CANable 等 USB-CAN 适配器）
 *      JSDK_BUILD_HAL_VIRTUAL      虚拟总线 + 驱动器行为模拟器（CI/仿真必需）
 *
 *  MCU 客户**看不到也不需要**本头文件：他们实现自己的 `jsdk_can_hal_t`。
 *
 * @par 与零 malloc 核心的关系
 *  内置后端位于零 malloc 核心**之外**，允许 malloc 操作系统句柄
 *  （socket fd / PCAN 句柄 / 串口 fd）。它们只在桌面平台编译。
 *
 * @par 典型用法
 *  @code
 *      jsdk_hal_handle_t *h = NULL;
 *      jsdk_context_config_t cfg;
 *      jsdk_context_config_default(&cfg);
 *      jsdk_hal_socketcan_open(&cfg.hal, &h, "can0", 1000000, 5000000);
 *      cfg.master_id = 1; cfg.is_fd = 1;
 *      jsdk_context_init((jsdk_context_t *)&store, &cfg);
 *      ...
 *      jsdk_hal_close(h);
 *  @endcode
 */

/* ==========================================================================
 * 符号导出（JSDK_API）
 * --------------------------------------------------------------------------
 * 静态链接时此宏为空，**不改变任何调用方代码**。
 * 构建**共享库**（`JSDK_BUILD_SHARED=ON`；Python 绑定与桌面工具需要）时：
 *   - Windows：必须是 `__declspec(dllexport/dllimport)`，否则默认不导出；
 *   - GCC/Clang：用 `visibility("default")` 显式放行，其余符号保持 hidden
 *     —— 这正是"符号同名、互斥链接"那条家族约束所需要的：内部符号
 *     （`cb_*` 等）不应该与 EtherCAT 版后端在同一进程里撞名。
 *
 * 由构建系统定义 `JSDK_SHARED`（构建库自身）或 `JSDK_SHARED_IMPORT`
 * （使用库的一方）；两者都不定义时就是静态链接。
 * ========================================================================== */
#if defined(JSDK_SHARED)
#  if defined(_WIN32)
#    define JSDK_API __declspec(dllexport)
#  elif defined(__GNUC__) || defined(__clang__)
#    define JSDK_API __attribute__((visibility("default")))
#  else
#    define JSDK_API
#  endif
#elif defined(JSDK_SHARED_IMPORT)
#  if defined(_WIN32)
#    define JSDK_API __declspec(dllimport)
#  else
#    define JSDK_API
#  endif
#else
#  define JSDK_API
#endif

#ifndef JOINT_SDK_JSDK_HAL_BUILTIN_H
#define JOINT_SDK_JSDK_HAL_BUILTIN_H

#include <stdint.h>
#include "joint_sdk.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 内置后端的实例句柄（不透明，由 open 分配、由 jsdk_hal_close 释放）。 */
typedef struct jsdk_hal_handle jsdk_hal_handle_t;

/* --------------------------------------------------------------------------
 * Linux SocketCAN
 * ------------------------------------------------------------------------ */

/**
 * 打开 SocketCAN 接口。
 * @param ifname       接口名，如 "can0" / "vcan0"
 * @param bitrate      仲裁段波特率（仅用于校验，实际由 ip link 配置）
 * @param data_bitrate CAN-FD 数据段波特率（0 = Classic）
 *
 * @note 位定时由内核负责：CAN 链路必须先以匹配的参数 up 起来，例如
 *       `ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on`
 *       （或使用 `scripts/canfd_restart.sh`）。SDK 不修改链路参数。
 */
JSDK_API jsdk_status_t jsdk_hal_socketcan_open(jsdk_can_hal_t *hal, jsdk_hal_handle_t **out,
                                      const char *ifname,
                                      uint32_t bitrate, uint32_t data_bitrate);

/* --------------------------------------------------------------------------
 * PEAK PCAN-Basic（Windows / macOS）
 * ------------------------------------------------------------------------ */

/**
 * 打开 PEAK 通道。
 * @param channel 如 "PCAN_USBBUS1"。也接受简写 "can0" / "can1"
 *                （映射到 PCAN_USBBUS1 / PCAN_USBBUS2）。
 *
 * @note Windows 需要安装 PCAN-Basic 驱动；无需 PCANBasic.lib 链接（运行期加载）。
 */
JSDK_API jsdk_status_t jsdk_hal_pcan_open(jsdk_can_hal_t *hal, jsdk_hal_handle_t **out,
                                 const char *channel,
                                 uint32_t bitrate, uint32_t data_bitrate);

/* --------------------------------------------------------------------------
 * 串口 slcan（CANable / 兼容 USB-CAN 适配器）
 * ------------------------------------------------------------------------ */

/**
 * 打开 slcan 串口。
 * @param port 如 "COM5"（Windows）/ "/dev/ttyACM0"（Linux）
 * @param baud 串口波特率，如 115200 / 1000000（取决于适配器固件）
 * @param data_bitrate CAN FD 数据段速率（bps）：
 *         - **0 = 不碰适配器配置**（保持它当前的设置；与 SocketCAN 的
 *           “内核管链路、SDK 不插手”同一姿态）；
 *         - 非 0 = ▸**主动**向适配器发 `Y<n>` 设置数据段速率（即改适配器配置）。
 *           目前只收录 CANable 2.0 公认的两项：**2 000 000（`Y2`）与
 *           5 000 000（`Y5`）**；其它速率返回 `JSDK_ERR_UNSUPPORTED`
 *           —— `Y` 的索引是固件私有表，**不猜**（猜错就是把数据段速率设成
 *           别的值，现场只会看到“帧发不出去”）。要别的速率请用厂家工具先配好
 *           再传 0。
 *
 * @note 打开时会发 `C\r`（回到关闭态）→ [`Y<n>\r`] → `O\r`（打开通道）。
 *       `O` 不可省：Lawicel 语义下通道上电是关闭的。
 * @note slcan 为 ASCII 行协议，Classic 实际约 100~500 fps，FD 单帧载荷更大、
 *       帧率更低；只建议用于配置、监控与低速 MIT 控制，高频控制请用
 *       SocketCAN / PCAN。
 * @note **支持 CAN FD**（CANable 2.0 的 `b/B/d/D` 帧前缀）。帧类型由
 *       `jsdk_can_frame_t.flags` 的 `JSDK_FRAME_FD` / `JSDK_FRAME_BRS` 决定，
 *       本后端不再有“后端级 FD 开关”。
 */
JSDK_API jsdk_status_t jsdk_hal_slcan_open(jsdk_can_hal_t *hal, jsdk_hal_handle_t **out,
                                  const char *port, uint32_t baud,
                                  uint32_t data_bitrate);

/* --- slcan 诊断（CLI / Python 绑定用；都只读，不动硬件状态） --- */

/**
 * 本后端的 slcan 实现是否支持 CAN FD。
 *
 * **恒为 1**：`d/D`（BRS=0）与 `b/B`（BRS=1）四个帧前缀属于 CANable 2.0
 * 固件对 Lawicel slcan 的扩展。保留成函数是为了让"能力自报"成为统一接口
 * （客户不必记住哪个后端支持什么）。
 */
JSDK_API int jsdk_hal_slcan_supports_fd(void);

/**
 * 上一次 `jsdk_hal_slcan_open()` 失败的**原因**（人可读，含 errno/错误码与建议）。
 *
 * @return 以 NUL 结尾的字符串；从未失败（或刚成功打开）时为空串 `""`
 *
 * 为什么需要它：打开失败时**没有句柄**可挂诊断信息（句柄已释放），返回码只有
 * `JSDK_ERR_INVALID_ARG` —— 现场（Ubuntu 上忘了 `sudo` → `EACCES`）只能看到
 * “invalid-argument”，分不清**权限不足 / 设备不存在 / 被占用**。
 * 这条把 `strerror()` 与“下一步该做什么”一起带出来，例如：
 *
 * ```
 * open(/dev/ttyACM0, O_RDWR) 失败：Permission denied (errno=13)，
 * 权限不足：把当前用户加入 dialout 组（sudo usermod -aG dialout $USER，
 * 重新登录生效），或本次用 sudo 运行
 * ```
 *
 * @note 非线程安全（文件级缓冲，诊断用）；每次 `open()` 开头重置。
 */
JSDK_API const char *jsdk_hal_slcan_last_open_error(void);

/**
 * 取当前 FD 速率配置。
 * @param enabled 输出：是否已向适配器发过 `Y<n>`（0 = 没碰过适配器配置）
 * @param bitrate 输出：已设置的数据段速率（bps）；`enabled=0` 时为 0
 */
JSDK_API void jsdk_hal_slcan_fd_config(jsdk_hal_handle_t *h, int *enabled,
                                       uint32_t *bitrate);

/**
 * 取 FD 帧计数。
 * @param tx_fd 输出：主站发出的 FD 帧数
 * @param rx_fd 输出：收到的 FD 帧数
 *
 * @note `rx_fd == 0` 而 `tx_fd > 0` 通常意味着**对端在按 Classic 回**，
 *       或者适配器没进 FD 模式 —— 这比"反馈解析不出来"早一步指出现场问题。
 */
JSDK_API void jsdk_hal_slcan_fd_frames(jsdk_hal_handle_t *h, uint32_t *tx_fd,
                                       uint32_t *rx_fd);

/**
 * 取收发/错误计数（任一可为 NULL）。
 * @param tx 输出：成功发出的帧数
 * @param rx 输出：成功解出的帧数
 * @param malformed 输出：丢掉的行数（状态行、坏行都算）
 * @param acks 输出：`\r` 回执数
 * @param nacks 输出：`\a` 错误回执数
 */
JSDK_API void jsdk_hal_slcan_stats(jsdk_hal_handle_t *h, uint32_t *tx,
                                   uint32_t *rx, uint32_t *malformed,
                                   uint32_t *acks, uint32_t *nacks);

/* --------------------------------------------------------------------------
 * 虚拟总线 + 驱动器行为模拟器（CI / 离线仿真）
 * ------------------------------------------------------------------------ */

/**
 * 打开虚拟后端。虚拟后端自带一个简化固件模型：模式 nibble、看门狗计时与
 * disarm、量程钳位、SET_ZERO、CONFIG_SAVE、故障注入、Classic/FD 两套编码。
 *
 * @param node_spec 节点规格，`;` 分隔多个节点。**每个节点的起始数字是
 *                  `nodes[]` 下标（0 开始）**，不是 node_id；node_id 用
 *                  `id=` 设置（缺省下标 i 对应 node_id i+1）。
 *
 *                  支持的键：`id` `gear` `tconst` `pmax` `vmax` `kpmax` `kdmax`
 *                  `tmax` `hb` `timeout` `vb` `temp`；
 *                  无值键：`fd` / `classic` / `arm` / `disarm` / `enabled` / `disabled`。
 *
 *                  例：`"0:gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=10,fd"`
 *                      `"0:id=1,gear=16.5,fd;1:id=2,gear=8,classic"`
 *
 *                  NULL 或空串 = 单节点（下标 0、node_id = 1、固件缺省参数）。
 * @return JSDK_OK；规格语法错误返回 JSDK_ERR_INVALID_ARG，内存不足返回 JSDK_ERR_NO_MEMORY
 */
JSDK_API jsdk_status_t jsdk_hal_virtual_open(jsdk_can_hal_t *hal, jsdk_hal_handle_t **out,
                                    const char *node_spec);

/**
 * 由**测试代码冒充设备**向主站注入一帧（走 SDK 的接收路径）。
 * 用于构造异常场景：心跳丢失、错误码、非法载荷、乱序响应等。
 */
JSDK_API jsdk_status_t jsdk_hal_virtual_inject(jsdk_hal_handle_t *h, const jsdk_can_frame_t *f);

/**
 * 取出主站（SDK）发出的下一帧，供断言使用（字节级对拍）。
 * @return 1 = 取到；0 = 队列为空。
 */
JSDK_API int jsdk_hal_virtual_capture(jsdk_hal_handle_t *h, jsdk_can_frame_t *f);

/** 令后续 N 次 send 失败（模拟总线错误）。fail = -1 表示全部失败。 */
JSDK_API void jsdk_hal_virtual_set_tx_fail(jsdk_hal_handle_t *h, int fail);

/** 累计因内部队列满而丢弃的注入帧数。 */
JSDK_API uint32_t jsdk_hal_virtual_dropped(const jsdk_hal_handle_t *h);

/** 推进模拟驱动器的时间轴（毫秒），用于测试看门狗超时。 */
JSDK_API void jsdk_hal_virtual_advance_ms(jsdk_hal_handle_t *h, uint32_t ms);

/**
 * 让虚拟后端的 `now_ms` **自己推进**时钟（每次调用 +1 ms，并顺带跑周期任务）。
 *
 * @par 为什么需要这个开关
 *  SDK 里有一批**阻塞式**配置阶段 API（`configure()` / `discover()` /
 *  `calibrate()` / `home()`），它们靠"等时间流逝"来轮询响应。而虚拟后端的
 *  时钟默认**只在** `jsdk_hal_virtual_advance_ms()` 里前进 —— 冻结时钟下这些
 *  API 会一直等到内部自旋上限，然后报 `stalled (frozen clock?)`。
 *
 *  打开自动推进后，仿真时钟的语义就与真实 HAL 的自由运行计数器一致，
 *  `jsdk-cli --if virtual`、Python 绑定、离线脚本都能**像连真机一样**直接调
 *  那些阻塞 API，不必自己写一层包装 HAL。
 *
 * @param enable 1 = 开启，0 = 关闭
 *
 * @warning **默认关闭**，且必须保持默认：`tests/test_joint.c` 用冻结时钟验证
 *          非阻塞路径（"时钟不前进时 SDK 也不能卡住"）。测试里要主动打开它
 *          才能跑配置阶段。
 */
JSDK_API void jsdk_hal_virtual_set_autotick(jsdk_hal_handle_t *h, int enable);

/* --------------------------------------------------------------------------
 * 通用
 * ------------------------------------------------------------------------ */

/** 关闭并释放句柄（幂等，允许 NULL）。 */
JSDK_API jsdk_status_t jsdk_hal_close(jsdk_hal_handle_t *h);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* JOINT_SDK_JSDK_HAL_BUILTIN_H */
