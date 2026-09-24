/**
 * @file    joint_sdk.h
 * @brief   CyberBeast Joint SDK — CYBERBEAST (CAN / CAN-FD) 后端公共 C ABI
 *
 * @details
 *  本头文件是 SDK 的**唯一**对外接口。它把 CyberBeast 固件的
 *  CYBERBEAST 协议（CAN ID 位域、定点打包、Classic/FD 分支、
 *  参数分段传输、JSON 描述符分块）全部隔离在内部，客户端只使用
 *  物理量（输出端 rad / rad/s / N·m）与关节级语义。
 *
 *  ⚠ 字节序提醒（若要直接用 `jsdk_ctx_write_param()` / SDO 缓冲区等裸字节接口）：
 *    控制帧与查询响应是**大端**；**参数值是小端**（固件 memcpy 主机序）。
 *    细节与真机实测证据见 `src/proto_cyberbeast/cb_frame.h` 的 `cb_le_*` 说明。
 *
 *  设计文档：docs/DESIGN.zh-CN.md
 *  协议细节：docs/PROTOCOL_NOTES.zh-CN.md
 *  移植指南：docs/PORTING.zh-CN.md
 *
 * @par 家族关系（重要）
 *  本库是 `jsdk_*` 家族的**第三个后端**：
 *      1 = IgH  EtherCAT   (EtherCAT_Master/joint-sdk)
 *      2 = SOEM EtherCAT   (SOEM/joint-sdk)
 *      3 = CYBERBEAST CAN   (本目录)
 *  三者**符号同名、互斥链接**。C 语言没有名字修饰，同名但 ABI 不同会导致
 *  **静默的**内存错配，因此：
 *    - 本库的文件名与其他两版故意不同（`libjsdk_can` / `jsdk_can.dll`）；
 *    - 头文件与库通过 JSDK_BACKEND_TAG 强制一致，不一致时编译报错；
 *    - 所有不透明/可见结构体首字段为魔数，jsdk_context_init() 运行期校验。
 *  **禁止**在同一进程内同时链接本库与 EtherCAT 版。
 *
 * @par 线程模型
 *  核心库**不创建任何线程**，不阻塞。调用者拥有循环：
 *    - 桌面/实时系统：jsdk_context_cycle_begin() → 读反馈/写目标 → jsdk_context_cycle_end()
 *    - 裸机 MCU    ：jsdk_context_poll()（等价于 begin + end）
 *  一个 `jsdk_context_t` 必须只被一个线程访问。
 *
 * @par 内存模型
 *  默认**零 malloc**：上下文与关节全部放在调用者提供的 jsdk_context_storage_t
 *  中（见 §5.5 设计文档）。堆模式 jsdk_context_create() 为独立可选文件，
 *  默认不编译。
 *
 * @par 许可证
 *  专有许可（Proprietary）。未经授权不得再分发。详见 LICENSE。
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

#ifndef JOINT_SDK_JOINT_SDK_H
#define JOINT_SDK_JOINT_SDK_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 0. 后端标识与 ABI 守卫
 * ======================================================================== */

#define JSDK_BACKEND_TAG_CAN     3u            /**< 本后端的标签 */
#define JSDK_BACKEND_NAME_CAN    "cyberbeast-can"
#define JSDK_ABI_VERSION_CAN     0x00010000u   /**< 1.0.0 */

/**
 * 编译期 ABI 守卫。
 * 若调用者的构建系统同时定义了另一后端的标签（或直接链接了 EtherCAT 版），
 * 在此处失败，避免运行期静默错配。
 */
#if defined(JSDK_BACKEND_TAG) && (JSDK_BACKEND_TAG != JSDK_BACKEND_TAG_CAN)
#  error "joint_sdk.h(CAN): JSDK_BACKEND_TAG mismatch - this header belongs to the CYBERBEAST CAN backend (3). Do not mix with the IgH(1)/SOEM(2) EtherCAT backends."
#endif
#ifndef JSDK_BACKEND_TAG
#  define JSDK_BACKEND_TAG JSDK_BACKEND_TAG_CAN
#endif

/** @return 后端名，恒为 "cyberbeast-can"（用于运行期自检/日志）。 */
JSDK_API const char *jsdk_backend_name(void);

/** @return ABI 版本（JSDK_ABI_VERSION_CAN）。 */
JSDK_API uint32_t jsdk_abi_version(void);

/**
 * 结构体尺寸/对齐表，供 **FFI 绑定自检**（Python ctypes / C# / Rust）。
 *
 * @par 为什么绑定方一定需要它
 *  FFI 绑定必须**复刻**这些结构体的字段顺序与对齐。复刻错了不会报错，而是
 *  直接踩内存 —— 症状是"值偶尔不对"或随机崩溃，现场几乎查不出来。
 *  绑定方应在**导入时**逐项比对，不一致立刻抛"ABI 不匹配（哪个类型）"。
 *
 * @param count_out 输出：条目数（可为 NULL）
 * @return 静态表；永不返回 NULL
 *
 * @note 表**只增不改**：改动字段顺序必须同时改这里，而这会让所有绑定在导入时
 *       立刻失败 —— 这正是我们想要的信号（ABI 变了）。
 */
typedef struct {
    const char *name;    /**< 类型名（与 C 里的拼写一致） */
    uint32_t    size;    /**< sizeof(T) */
    uint32_t    align;   /**< 对齐（C99 下用 offsetof 技巧求得） */
} jsdk_abi_type_t;

/**
 * 返回公共结构体的 `sizeof`/对齐表（**ABI 自检**用；表是静态的，不要 free）。
 *
 * 用途：语言绑定/工具在**加载时**比对，把“库与头文件版本不一致”当场报出来，
 * 而不是等到某个字段读出垃圾值。`count_out` 可传 `NULL`。
 */
JSDK_API const jsdk_abi_type_t *jsdk_abi_types(size_t *count_out);

/* ==========================================================================
 * 1. 状态码（与 EtherCAT 版数值对齐，便于客户迁移）
 * ======================================================================== */

typedef enum {
    JSDK_OK              =  0,
    JSDK_ERR_INVALID_ARG = -1,
    JSDK_ERR_NO_MEMORY   = -2,
    JSDK_ERR_NOT_FOUND   = -3,
    JSDK_ERR_BAD_STATE   = -4,
    JSDK_ERR_TRANSPORT   = -5,   /**< 链路/HAL 层错误（原 JSDK_ERR_ECRT 语义泛化） */
    JSDK_ERR_UNSUPPORTED = -6,
    JSDK_ERR_TIMEOUT     = -7,   /**< CAN 扩展：等待响应/状态变化超时 */
    JSDK_ERR_PROTOCOL    = -8,   /**< CAN 扩展：对端返回非法/错误码（见 QUERY_ERROR） */
    JSDK_ERR_BUSY        = -9,   /**< CAN 扩展：TX 队列满 / 请求进行中 */
    JSDK_ERR_PARSE       = -10   /**< 描述符 JSON 解析失败 / 超出上限 */
} jsdk_status_t;

/** 源码兼容别名（EtherCAT 版使用此名）。 */
#define JSDK_ERR_ECRT JSDK_ERR_TRANSPORT

/* ==========================================================================
 * 2. 传输层 HAL —— MCU 客户唯一需要实现的部分
 * ======================================================================== */

#define JSDK_FRAME_FD  0x01u   /**< 帧为 CAN FD */
#define JSDK_FRAME_BRS 0x02u   /**< 数据段变速（仅 CAN FD 有效） */
#define JSDK_FRAME_EXT 0x04u   /**< 扩展帧；本协议**恒为 1** */

/** 一帧 CAN / CAN-FD 数据。id 为非 RTR 帧 ID（29-bit，无需左移）。 */
typedef struct {
    uint32_t id;       /**< 29-bit 扩展 ID（本协议始终使用） */
    uint8_t  len;      /**< 0..8 (Classic) / 0..64 (FD) */
    uint8_t  flags;    /**< JSDK_FRAME_* 位掩码 */
    uint8_t  data[64]; /**< 载荷 */
} jsdk_can_frame_t;

/** on_error 的 kind 取值。 */
typedef enum {
    JSDK_BUS_ERR_NONE      = 0,
    JSDK_BUS_ERR_TX_FAIL   = 1,   /**< 发送失败（无 ACK / 队列满） */
    JSDK_BUS_ERR_RX_OVERRUN= 2,   /**< 接收溢出，可能丢帧 */
    JSDK_BUS_ERR_WARNING   = 3,   /**< 错误警告（error-warning） */
    JSDK_BUS_ERR_PASSIVE   = 4,   /**< error-passive */
    JSDK_BUS_ERR_BUS_OFF   = 5,   /**< bus-off，需硬件/驱动恢复 */
    JSDK_BUS_ERR_OTHER     = 255
} jsdk_bus_error_t;

/** HAL 自报的链路状态位（bus_status 回调，可选）。 */
#define JSDK_HAL_BUS_OK         0x00000001u
#define JSDK_HAL_BUS_ERROR_WARN 0x00000002u
#define JSDK_HAL_BUS_ERROR_PASS 0x00000004u
#define JSDK_HAL_BUS_OFF        0x00000008u
#define JSDK_HAL_LISTEN_ONLY    0x00000010u

/**
 * 传输层抽象。SDK 内部**不做任何平台 #ifdef**，所有平台差异由此结构体注入。
 *
 * @par 契约
 *  - `send` / `recv` 必须**非阻塞**：SDK 保证不在 cycle 内阻塞等待。
 *  - `send` 返回非 0 时 SDK 会记账（tx_failed）并在下一周期重试，不会中止循环。
 *  - `recv` 应逐帧返回；一周期内 SDK 会反复调用直到返回 0（有内部上限）。
 *  - `now_ms` 必须单调递增；溢出按 uint32_t 回绕处理（SDK 内部使用无符号差值）。
 *  - 本结构体在 jsdk_context_init() 时被**复制**，之后调用者可丢弃。
 */
typedef struct {
    void *user;   /**< 回调透传指针 */

    /** 发送一帧。@return 0 = 已发出/已入队；非 0 = 失败或忙。 */
    int (*send)(void *user, const jsdk_can_frame_t *f);

    /** 接收一帧。@return 1 = 取到；0 = 当前无帧；<0 = 链路错误。 */
    int (*recv)(void *user, jsdk_can_frame_t *f);

    /** 单调毫秒时钟（看门狗/超时/新鲜度均基于此）。 */
    uint32_t (*now_ms)(void *user);

    /** 可选（可为 NULL）：链路错误通知。 */
    void (*on_error)(void *user, int kind, uint32_t detail);

    /** 可选（可为 NULL）：填充 JSDK_HAL_BUS_* 位；@return 0 = 成功。 */
    int (*bus_status)(void *user, uint32_t *flags);
} jsdk_can_hal_t;

/* ==========================================================================
 * 3. 控制模式与关节状态
 * ======================================================================== */

/**
 * 控制模式。
 *  - MIT(4) 与固件 ModeState nibble 对齐（固件 MODE_MIT = 4）。
 *  - CSP/CSV/CST 沿用 EtherCAT 家族数值，使客户代码可移植；CAN 侧分别映射到
 *    固件 POS_CONTROL(0x01) / VEL_CONTROL(0x02) / TORQUE_CONTROL(0x03)。
 *    注意线上单位不同（度 / RPM / A），SDK 负责换算。
 */
typedef enum {
    JSDK_MODE_MIT     =  4,
    JSDK_MODE_CSP     =  8,   /**< → POS_CONTROL(0x01) */
    JSDK_MODE_CSV     =  9,   /**< → VEL_CONTROL(0x02) */
    JSDK_MODE_CST     = 10,   /**< → TORQUE_CONTROL(0x03) */
    JSDK_MODE_CURRENT = 11    /**< CAN 扩展 → CURRENT_CONTROL(0x04)，不喂看门狗，见 §2.5 */
} jsdk_mode_t;

/**
 * 关节状态。枚举名沿用 EtherCAT 家族以利源码迁移；CAN 语义映射见
 * docs/DESIGN.zh-CN.md §5.6。
 */
typedef enum {
    JSDK_AXIS_UNKNOWN = 0,
    JSDK_AXIS_SWITCH_ON_DISABLED,   /**< CAN: IDLE */
    JSDK_AXIS_READY_TO_SWITCH_ON,   /**< CAN: CALIBRATING / 使能过渡中 */
    JSDK_AXIS_SWITCHED_ON,          /**< CAN: 已闭环但尚无控制帧（瞬态） */
    JSDK_AXIS_OPERATION_ENABLED,    /**< CAN: 闭环执行中，配合 mode 区分 */
    JSDK_AXIS_FAULT
} jsdk_axis_state_t;

/** 固件 ModeState nibble 原始值（jsdk_joint_get_mode_state()）。 */
typedef enum {
    JSDK_MODESTATE_RESET       = 0,
    JSDK_MODESTATE_CALIBRATING = 1,
    JSDK_MODESTATE_IDLE        = 2,
    JSDK_MODESTATE_CLOSED_LOOP = 3,
    JSDK_MODESTATE_MIT         = 4,
    JSDK_MODESTATE_POSITION    = 5,
    JSDK_MODESTATE_VELOCITY    = 6,
    JSDK_MODESTATE_TORQUE      = 7
} jsdk_modestate_t;

/* ==========================================================================
 * 4. 关节状态标志位（粘滞，需显式清除）
 * ======================================================================== */

#define JSDK_JF_TARGET_REJECTED  0x0001u  /**< 曾因越界拒绝目标位置 */
#define JSDK_JF_SAFE_FRAME_SENT  0x0002u  /**< 曾在本周期改发安全帧 */
#define JSDK_JF_WATCHDOG_RISK    0x0004u  /**< 控制周期接近 break_timeout */
#define JSDK_JF_FEEDBACK_STALE   0x0008u  /**< 反馈超时（age_ms 超阈值） */
#define JSDK_JF_TX_FAILED        0x0010u  /**< HAL send 连续失败 */
#define JSDK_JF_SCALE_INVALID    0x0020u  /**< 未取得标定参数，物理量 API 不可用 */
#define JSDK_JF_WATCHDOG_UNVERIFIED 0x0040u
                                         /**< 写 `can.config.break_timeout` 后**读回不符**
                                              （真机 F28：该端点读回恒为 0）。
                                              SDK 已按**写入值**保守处理（宁可多喂几帧），
                                              但调用方**不得**把它当作“已武装”的证据。
                                              由每次 `jsdk_joint_set_watchdog_ms()`
                                              重新判定（成功即清除）。 */

/* ==========================================================================
 * 5. 不透明句柄与上下文存储
 * ======================================================================== */

typedef struct jsdk_context jsdk_context_t;
typedef struct jsdk_joint   jsdk_joint_t;

#ifndef JSDK_MAX_JOINTS_STATIC
#  define JSDK_MAX_JOINTS_STATIC 8u
#endif

/**
 * 与 JSDK_MAX_JOINTS_STATIC 匹配的上下文最大字节数（编译期固定）。
 *
 * ⚠ 这个值是**实测得出**的，不是拍脑袋：x86-64 / MinGW 下
 *   `sizeof(jsdk_joint_t) = 560`（含反馈 112 + 目标 88 + 8 个参数槽位 112 + 端点 ID 22 …）
 *   `sizeof(cb_desc_fetch_t) = 728`（增量 JSON 解析器 + 传输状态机）
 *   ⇒ `sizeof(jsdk_context_t) = 5744`（8 个内嵌关节）。
 *   `src/core/jsdk_context.c` 里有编译期断言守住"实际大小 ≤ 本常量"，
 *   所以结构体一旦变大而这里没跟着改，**编译就会失败**（而不是静默写溢出
 *   调用者的静态存储 —— 那是最危险的一类回归）。
 *   测试 `tests/test_joint.c` 的 `[2]` 会把这个实测值打印出来，便于随时复核。
 *
 * RAM 敏感时：把 `JSDK_MAX_JOINTS_STATIC` 调小（例如 4 → 约 3.6 KB），
 * 两个宏必须一起改。
 */
#ifndef JSDK_CONTEXT_MAX_SIZE
#  define JSDK_CONTEXT_MAX_SIZE 6144u
#endif

/**
 * 零 malloc 模式下的上下文存储。用法：
 * @code
 *   static jsdk_context_storage_t s_ctx_store;
 *   jsdk_context_init((jsdk_context_t *)&s_ctx_store, &cfg);
 * @endcode
 */
typedef union {
    uint64_t _align;
    uint8_t  bytes[JSDK_CONTEXT_MAX_SIZE];
} jsdk_context_storage_t;

/* ==========================================================================
 * 5.5 描述符策略与端点类型（前向定义；相关函数见 §18）
 * ------------------------------------------------------------------------
 * CYBERBEAST 的端点 ID 由固件 JSON 描述符的**声明顺序**决定，实测跨固件版本
 * 漂移率 86%（v8 594 个端点 vs 0.5.13 480 个；471 个共有路径中 405 个 ID 不同）。
 * 因此 SDK **不内置任何静态端点表**：连接时从设备下载 JSON 描述符并动态解析，
 * 从根本上消除 ID 漂移问题。
 *
 * 代价（集成时必须纳入设计，详见 docs/DESIGN.zh-CN.md §6.6）：
 *   - 描述符实测 41029 字节。CAN FD 需 1 + 662 帧（约 0.1~0.2 s）；
 *     Classic 1 Mbps 需 1 + 6839 帧（约 1~2 s）；
 *   - 全量保留端点表约需 24.9 KB RAM（594 端点）。RAM 受限时改用
 *     JSDK_DESC_RETAIN_FILTERED + filter_paths，典型 < 1 KB。
 * ======================================================================== */

/**
 * 描述符获取方式。
 *
 * 通用规则：**只要描述符已存在**（经 jsdk_context_desc_import() 或
 * jsdk_context_desc_import_raw() 提供），configure() 就**不会再下载**，与 mode 无关。
 * 因此典型启动流程是：先试缓存导入，失败则不导入，让 configure() 去下载。
 */
typedef enum {
    JSDK_DESC_DYNAMIC    = 0,  /**< 默认：描述符尚不存在时向设备下载并解析 */
    JSDK_DESC_FROM_CACHE = 1   /**< 强制不下载：configure() 时若描述符仍不存在，
                                    返回 JSDK_ERR_BAD_STATE（用于保证启动时间或离线场景） */
} jsdk_desc_mode_t;

/** 端点表保留策略。 */
typedef enum {
    JSDK_DESC_RETAIN_ALL      = 0,  /**< 默认：保留全部端点（约 24 KB；桌面 / CLI） */
    JSDK_DESC_RETAIN_FILTERED = 1   /**< 只保留 filter_paths 命中的端点（MCU，典型 < 1 KB） */
} jsdk_desc_retain_t;

/**
 * 描述符获取与解析配置。
 *
 * filter_paths 匹配规则（仅 JSDK_DESC_RETAIN_FILTERED 使用）：
 *   - 精确匹配："axis0.motor.config.gear_ratio"
 *   - 前缀匹配（推荐）：以 '*' 结尾，如 "axis0.controller.config.mit_max_*"
 *   - 段前缀：以 '.' 结尾，如 "axis0.motor.config."
 *   - 全保留："*"
 * 未命中的端点不进 arena（但仍被解析，用于 stop_when_satisfied 判定）。
 */
typedef struct {
    uint8_t  mode;                 /**< jsdk_desc_mode_t；默认 JSDK_DESC_DYNAMIC */
    uint8_t  retain;               /**< jsdk_desc_retain_t；默认 JSDK_DESC_RETAIN_ALL */
    uint8_t  share_by_crc;         /**< 默认 1：同一总线上 (fw, crc) 相同的节点只下载一次 */
    uint8_t  stop_when_satisfied;  /**< 默认 1：filter 全部命中即提前终止下载。
                                        ⚠ 仅当**所有** filter 都是精确路径时才会生效；
                                        §10 通配/前缀 filter 下不提前终止，否则会静默丢字段。
                                        实际是否启用见 cb_desc_fetch_result_t.stop_allowed */
    uint16_t max_endpoints;        /**< 解析上限（防异常固件），0 = 2048 */
    uint16_t max_path_len;         /**< 单条路径长度上限（含 '\0'），0 = 128 */
    uint32_t timeout_ms;           /**< **静默（卡死）预算**：多久没有新字节就算失败，0 = 5000。
                                        ⚠ 不是“总时长”：描述符是**流式**的，同一个
                                        38433 B 的描述符 FD 是 1+662 帧、**Classic 是
                                        1+6906 帧（10.4 倍）**；按总时长算会在 Classic
                                        下把一条一直在推进的流误判成超时（真机实测：
                                        85% 处被 3000 ms 预算截断）。另有一条
                                        `JSDK_DESC_TOTAL_MAX_MS`(120 s) 总时长兜底。 */
    const char *const *filter_paths; /**< RETAIN_FILTERED 时必须提供 */
    unsigned filter_count;
    void    *arena;                /**< 解析区，**必需**：本 SDK 不做堆分配。
                                        NULL 时 configure() 返回 JSDK_ERR_INVALID_ARG。
                                        ⚠ 与本配置结构体**同寿命**：上下文会往里写 arena_used */
    size_t   arena_size;           /**< 解析区字节数；不足时 configure() 返回 JSDK_ERR_NO_MEMORY */
    size_t   arena_used;           /**< 输出：实际用量（**写回本结构体**，见上面的寿命要求）。
                                        解析完成后才有效；RETAIN_ALL 下实测 25493 B */
} jsdk_desc_config_t;

/**
 * 端点数据类型（对应 JSON 描述符的 type 字段）。
 * ⚠ 取值必须覆盖设备实际输出的全部 type。实测（v8，41029 字节）原始 JSON 中出现的
 *   type 及其次数：bool 88 / endpoint_ref 6 / float 247 / function 30 / int32 20 /
 *   int64 1 / json 1 / **object 66** / uint8 57 / uint16 18 / uint32 123 / uint64 3。
 *   其中 object 只出现在**无 id 的容器节点**上（因此不会成为端点，但解析器必须认识它）。
 * 解析器遇到未知 type 会**整体失败**（JSDK_ERR_PARSE），因为 type 决定了
 * PARAM_READ/WRITE 的字节长度与批量打包宽度，猜错会静默读错。
 */
typedef enum {
    JSDK_EP_U8 = 1, JSDK_EP_I8,
    JSDK_EP_U16,    JSDK_EP_I16,
    JSDK_EP_U32,    JSDK_EP_I32,
    JSDK_EP_U64,    JSDK_EP_I64,
    JSDK_EP_F32,    JSDK_EP_F64,
    JSDK_EP_BOOL,
    /* --- 以下为不可直接读写的“不透明”类型（仅用于枚举） --- */
    JSDK_EP_OBJECT,       /**< 容器对象（v8 共 66 个，均无 id） */
    JSDK_EP_ENDPOINT_REF, /**< 指向另一端点的引用（如 config.gpioN_pwm_mapping.endpoint） */
    JSDK_EP_JSON,         /**< 根节点占位（name="", id=0） */
    JSDK_EP_FUNCTION      /**< 方法（30 条，无 access 字段） */
} jsdk_ep_type_t;

/** 端点访问权限位。 */
#define JSDK_EP_ACCESS_R 0x01u
#define JSDK_EP_ACCESS_W 0x02u

/** 类型化值（jsdk_joint_param_get/set 使用）。 */
typedef struct {
    jsdk_ep_type_t type;
    union {
        uint8_t  u8;  int8_t  i8;
        uint16_t u16; int16_t i16;
        uint32_t u32; int32_t i32;
        uint64_t u64; int64_t i64;
        float    f32; double  f64;
        int      boolean;
    } v;
} jsdk_value_t;

/* ==========================================================================
 * 6. 上下文配置
 * ======================================================================== */

typedef struct {
    uint32_t magic;                 /**< 内部使用：ABI 魔数。必须置 0，由 SDK 填充。 */

    jsdk_can_hal_t hal;             /**< 必需。会在 init 时被复制。 */

    uint8_t  master_id;             /**< 主站源地址 1..254。**禁止 0**（设备将完全不回复）。 */
    uint8_t  is_fd;                 /**< 1 = CAN FD（默认，1M/5M BRS）；0 = Classic。*/
    uint8_t  is_fd_explicit;        /**< 1 = 调用者**明确**指定了 `is_fd`（CLI 的 `--classic` /
                                         `--data-bitrate`、Python 的 `is_fd=` 会置 1）。
                                         此时自动对齐**不会覆盖**它：与对端冲突只报告
                                         （`jsdk_context_framing_learned()` 返回 4）。
                                         0（默认）= `is_fd` 只是猜测，允许 SDK 在收到本
                                         关节第一帧时自动对齐到对端格式。

                                         ⚠⚠ **推荐组合**（两个 CLI 的自动模式）：
                                         `is_fd = 0; is_fd_explicit = 0;` —— 先按
                                         Classic 起步（FD 控制器也收经典帧，反之不成立），
                                         然后只收不发地泵 `jsdk_context_cycle_begin()`
                                         直到 `jsdk_context_framing_learned() != 0`，
                                         再决定是否改成 FD。理由：协议没有运行时协商，
                                         而“第一条帧就用错格式”不只是没人应 —— slcan 的
                                         FD 数据段速率是打开时配的，先按 FD 打开就回不
                                         去了（真机实测：之后改学也发不出去）。 */
    uint32_t period_ns;             /**< 期望控制周期（ns），用于 keepalive 与超时判定。0 = 自动。 */

    /** 等“状态序列跑完”的预算（ms）：`jsdk_joint_calibrate()` / `jsdk_joint_home()`
        这类“写 requested_state → 等它跑完”的阻塞命令。

        **0 = 用各自的内置默认**（标定 `JSDK_STATE_TIMEOUT_CALIBRATE_MS` = 120000，
        回零 `JSDK_STATE_TIMEOUT_HOME_MS` = 5000）；非 0 = 两个都用这个值。

        ⚠ 为什么标定默认是 **120 s**（原来是硬编码 20 s）：全标定要转**十几圈电气角**
          （`calib_scan_distance` 是电气弧度，本机 87.96 rad ÷ 2π = 14 圈；
          真机实测整条序列 >20 s）。旧值会在**序列还在跑**时就报“timeout”，
          把“没写进去”和“还没跑完”两件完全不同的事混成同一个提示。 */
    uint32_t state_timeout_ms;

    uint8_t  auto_keepalive;        /**< 1（默认）= cycle_end 内按需自动补喂狗帧。
                                         只对**设备侧开着**的协议超时（`break_timeout > 0`）
                                         生效；`0` = 设备侧已禁用超时 ⇒ 无狗可喂，
                                         此时不补帧（也不置风险位）。
                                         背景见 docs/DESIGN.zh-CN.md §6.3。 */
    uint8_t  clamp_target_position; /**< 目标越界策略（§6.10）：
                                         0（默认）= 拒绝并计数，本周期改发安全帧；
                                         1 = 静默钳位到量程后发送。 */
    uint8_t  enable_watchdog_hint;  /**< 1 = configure() 在**需要时**把设备 break_timeout
                                         写为 2×period：当前是 0（禁用）或现有超时
                                         比控制周期还短。设备本来设得很宽松时不动它。
                                         写完会**读回确认**；读不回来（真机 F28）只记
                                         一条 NOTE 并置 JSDK_JF_WATCHDOG_UNVERIFIED，
                                         不算致命错误。
                                         默认 0（不擅自修改客户设备配置）。 */
    uint8_t  max_joints;            /**< 关节容量；0 = JSDK_MAX_JOINTS_STATIC。 */
    uint8_t  rx_burst_limit;        /**< 单周期最多处理的接收帧数；0 = 默认 32。
                                         防止总线风暴阻塞控制循环。
                                         注：描述符下载走独立排空循环，不受此限。 */

    /** 描述符获取与解析策略（必需，见 §5.5）。arena 必须由调用者提供。 */
    jsdk_desc_config_t desc;
} jsdk_context_config_t;

/** 填充默认值（hal 置零，master_id = 1，is_fd = 1，auto_keepalive = 1）。 */
JSDK_API void jsdk_context_config_default(jsdk_context_config_t *cfg);

/** `calibrate()` 的默认预算（ms）：全标定要转十几圈电气角，实测 >20 s。 */
#define JSDK_STATE_TIMEOUT_CALIBRATE_MS 120000u

/** `home()` 的默认预算（ms）：回零只是一段限速运动。 */
#define JSDK_STATE_TIMEOUT_HOME_MS        5000u

/* --------------------------------------------------------------------------
 * 6.2 关节配置
 * ------------------------------------------------------------------------ */

typedef struct {
    uint32_t magic;                 /**< 内部使用，置 0；由 SDK 填充。 */

    uint8_t  node_id;               /**< 1..254，对应设备 axis0.config.can.node_id。 */
    uint8_t  axis;                  /**< 预留：多轴板轴号；当前固定 0。 */

    /* 以下为 0 时表示“从设备读取”。读取失败不会静默取默认值，
       而是置 unit_scale.valid = 0 并拒绍物理量 API。
       端点 ID 由运行时解析的 JSON 描述符提供（见 §18），无内置表。 */
    float gear_ratio;               /**< axis0.motor.config.gear_ratio */
    float mit_max_pos;              /**< 单位：rad（输出端） */
    float mit_max_vel;              /**< 单位：rad/s（输出端） */
    float mit_max_torque;           /**< 单位：N·m（输出端） */
    float mit_max_kp;               /**< axis0.controller.config.mit_max_kp */
    float mit_max_kd;               /**< axis0.controller.config.mit_max_kd */
    float torque_constant;          /**< N·m/A；解码 MIT 响应电流所必需 */

    jsdk_mode_t initial_mode;       /**< jsdk_context_activate() 使用的模式 */
} jsdk_joint_config_t;

/**
 * 关键配置参数快照（供 CLI `dump-config` 与 Python `Context.dump_config()` 复用）。
 * 字段均为设备实际生效值（configure() 已读回并校验）。
 *
 * ⚠ 本快照需要**已标定**的关节：描述符下载完但没跑标定时，除 `node_id` 外的字段
 *   都填 0 而 `valid == 0`（`dump-config` 因此属于需要完整配置的命令）。
 */
typedef struct {
    float    gear_ratio;
    float    mit_max_pos, mit_max_vel, mit_max_torque, mit_max_kp, mit_max_kd;
    float    torque_constant;
    uint32_t node_id;
    uint32_t heartbeat_rate_ms;  /**< 设备侧心跳周期；**0 = 固件已关闭心跳**（不是"没读到"） */
    uint32_t break_timeout_ms;   /**< 设备侧协议级 CAN 超时；**0 = 已禁用超时检测**
                                      （最新固件语义；不再有 0→100 ms 的归一化）。
                                      ⚠ 真机实测该端点读回恒为 0（F28）——
                                      想确认“真的武装了”请看 JSDK_JF_WATCHDOG_UNVERIFIED。 */
    int      valid;
} jsdk_joint_config_snapshot_t;

/** 读取关键参数快照（配置阶段 API，可阻塞）。 */
JSDK_API jsdk_status_t jsdk_joint_read_config_snapshot(jsdk_joint_t *j,
                                              jsdk_joint_config_snapshot_t *out);

/* ==========================================================================
 * 7. 生命周期
 * ======================================================================== */

/**
 * 初始化上下文（零 malloc）。`ctx` 指向调用者提供的 jsdk_context_storage_t。
 * 失败时 ctx 处于未初始化状态，可直接丢弃。
 */
JSDK_API jsdk_status_t jsdk_context_init(jsdk_context_t *ctx, const jsdk_context_config_t *cfg);

/**
 * 添加关节。必须在 configure() 之前调用。
 * `jc->axis` 当前保留（固件 AXIS_COUNT = 1），置 0。
 */
JSDK_API jsdk_status_t jsdk_context_add_joint(jsdk_context_t *ctx,
                                     const jsdk_joint_config_t *jc,
                                     jsdk_joint_t **out_joint);

/**
 * 配置阶段（非实时上下文，允许阻塞式等待响应）：
 *   1. 握手：发一帧让设备学到本机 master_id（否则心跳发往 0x01）；
 *   2. 发现标定参数：读 gear_ratio / mit_max_* / torque_constant，并做数值合理性校验；
 *   3. 端点解析：从设备下载 JSON 描述符（JSON_DESC_READ 0x24 → 0x25 流）并增量解析，
 *      得到全部端点 ID（见 §18）。**SDK 不内置静态端点表**，因此换固件不会读错参数。
 *      ⚠ 代价：实测描述符 41029 字节，FD 下 1+662 帧、Classic 1M 下 1+6839 帧；
 *      全量保留约 24.9 KB RAM。用 desc.retain / filter_paths 可降到 <1 KB。
 *      ⚠ 禁止在关节使能时执行（设备以 50 帧/ms 灌入，会挤掉控制帧触发看门狗）；
 *   4. 若 enable_watchdog_hint 且 period_ns 已知：在需要时设定 break_timeout
 *      （当前为 0 或比周期还短），并**读回确认**（读不回来只记 NOTE + 置
 *      JSDK_JF_WATCHDOG_UNVERIFIED，不阻断配置）。
 *      ⚠ 设备侧 `break_timeout == 0` = **超时检测已禁用** ⇒ 不做“周期必须小于它”
 *        的校验（那个校验只在设备真的开着超时时有意义）。
 * 任一标定参数缺失/异常 → JSDK_ERR_PROTOCOL 且 unit_scale.valid = 0。
 */
JSDK_API jsdk_status_t jsdk_context_configure(jsdk_context_t *ctx);

/**
 * 按各关节 initial_mode 使能（等价于逐个调用 request_enable + 等待就绪）。
 *
 * **阻塞**：内部反复 `cycle_begin()` / `cycle_end()` 直到全部关节使能，
 * 因此**不能在 RT 控制回路里调用**（会使回路周期失控），也不能在它运行期间
 * 并发调用 cycle_*。
 *
 * 全有或全无的前置校验：任一关节没有有效标定 → 返回 `JSDK_ERR_BAD_STATE`，
 * 且**不改变任何状态**。
 *
 * ⚠ 超时（`JSDK_ACTIVATE_TIMEOUT_MS`）返回 `JSDK_ERR_TIMEOUT` 时，**设备状态是
 *   "部分已知"**：已经进入闭环的关节确实带电且由 SDK 驱动，卡住的那个没有。
 *   调用方**不能**当作"什么都没发生"：要么调 `jsdk_context_deactivate()` 收拾
 *   （它会逐个关节走安全关机并清掉所有排队请求），要么继续跑周期。直接退出进程
 *   的话，已使能的关节只能靠设备侧 break_timeout 兜底。
 *   超时消息里会报出**哪个**关节卡住、卡在哪一步、mode_state 是多少。
 */
JSDK_API jsdk_status_t jsdk_context_activate(jsdk_context_t *ctx);

/**
 * 安全关闭：hold_position() → 等待 2 周期 → STOP_MOTOR → 等待 IDLE。
 * 顺序不可颠倒，否则"停发控制帧"会直接触发看门狗故障。
 *
 * 本函数会**丢弃所有排队中的请求**（`enable_pending` / `disable_pending` /
 * `faultreset_pending`）后才逐个关节断电 —— 否则一个陈旧的使能请求会在返回后的
 * 下一个周期把电机重新使能，让"安全关闭"名不副实。
 * 返回后 `jsdk_joint_is_enabled()` 为 0、`tx_active` 为 0。
 */
JSDK_API void jsdk_context_deactivate(jsdk_context_t *ctx);

/** 释放内部状态（不做 deactivate；零 malloc 模式下不释放存储）。 */
JSDK_API void jsdk_context_destroy(jsdk_context_t *ctx);

/**
 * 堆模式便利构造（**独立可选文件** heap_optional.c，默认不编译）。
 * 需要在 CMake 打开 JSDK_ENABLE_HEAP。返回的上下文用 jsdk_context_free() 释放。
 */
JSDK_API jsdk_context_t *jsdk_context_create(const jsdk_context_config_t *cfg);
JSDK_API void            jsdk_context_free(jsdk_context_t *ctx);

/* ==========================================================================
 * 8. 循环边界（RT 安全：无阻塞、无 malloc、无日志）
 * ======================================================================== */

/**
 * 周期开始：接收并解码全部待处理帧，更新反馈与内部状态机。
 * @param app_time_ns 调用者的单调时间（ns）。用于新鲜度与超时判定。
 */
JSDK_API jsdk_status_t jsdk_context_cycle_begin(jsdk_context_t *ctx, uint64_t app_time_ns);

/**
 * 周期结束：编码并发送本周期指令、按需补喂看门狗、推进超时/故障状态。
 * 返回 JSDK_ERR_BAD_STATE 表示本周期检测到链路或配置性问题（细节见 last_error）。
 */
JSDK_API jsdk_status_t jsdk_context_cycle_end(jsdk_context_t *ctx);

/** 裸机 MCU 便利入口：等价于 cycle_begin() + cycle_end()。 */
JSDK_API jsdk_status_t jsdk_context_poll(jsdk_context_t *ctx, uint64_t app_time_ns);

/**
 * 对端 CAN 帧格式（Classic / FD）的**学习结果**。
 *
 * 协议**没有**运行时协商：设备用哪种格式由它自己的 `can.config.baud_rate` 决定，
 * 而 `jsdk_context_config_t.is_fd` 只是主站的**猜测**。猜错时发出去的帧设备
 * **根本不收**，现场只表现为“收得到心跳、但我的请求没人应”。
 *
 * 因此 SDK 在**第一次收到本关节发来的帧**时，会把发送格式对齐成对端的格式
 * （只学一次，且只认自己的 node_id），并通过本函数如实报告。
 *
 * ⚠ 若调用者**明确**指定了格式（`cfg.is_fd_explicit = 1`），自动对齐不会覆盖它 ——
 *   那属于“你写错了”，本函数返回 4 让你报出来（否则你会継续看着
 *   “心跳收得到、请求没人应”而无从下手）。
 *
 * @return 0 = 还没收到过本关节的帧（未学习）；
 *         1 = 已改为 **Classic**（配置是 FD，对端在发 Classic 帧）；
 *         2 = 已改为 **FD**（配置是 Classic，对端在发 FD 帧）；
 *         3 = 与配置一致，无需调整；
 *         4 = **你明确指定的格式与对端不一致**（帧格式未改动 —— 请改配置）
 *
 * @note 客户若显式知道对端格式（例如产线固定 1 Mbps Classic），
 *       应在 `cfg.is_fd` 里写对（并置 `is_fd_explicit`），把本函数返回的 1/2/4
 *       当成**配置提醒**打印出来。
 */
JSDK_API int jsdk_context_framing_learned(const jsdk_context_t *ctx);

/**
 * 会话预热的默认预算（ms）；`jsdk_context_warmup(ctx, 0)` 用这个值。 */
#define JSDK_WARMUP_TIMEOUT_MS 500u

/**
 * 会话预热**每轮**等响应的窗口（ms）。
 *
 * ⚠ 刻意取小：设备正常时一个往返只要几毫秒，而“首帧丢失”那种情况**等再久也没用**
 *（那条命令根本没进适配器）—— 该做的是**重发**，不是把窗口调大。窗口大只有一个
 *  效果：失败时更慢地告诉你。总预算见 `JSDK_WARMUP_TIMEOUT_MS`。
 * ⚠ 自写 HAL（尤其是 `now_ms()` 可能不前进的测试夹具）时注意：本函数另有
 ***轮次上限**（`预算/本值 + 2`），时钟冻住也不会卡死。
 */
#define JSDK_WARMUP_ATTEMPT_MS 50u

/**
 * **会话预热**：确认“主机 → 适配器 → 设备 → 回来”这一圈已经通了。
 *
 * @par 为什么需要它（真机现场，2026-09）
 *  slcan 这类 USB-CDC 适配器在主机打开端口（DTR/线控变化）后**会重置自己的输入缓冲**，
 *  于是我们发过去的**头一两条 ASCII 命令在它准备好之前就被丢掉**；而 Lawicel slcan
 *  **对帧行不回报结果**（本项目实测：`acks/nacks` 恒为 0）——主机**没有任何可观测信号**
 *  知道“这条命令没进去”。所以“等一会儿再发”（sleep）只能是概率性赌运气，
 *  唯一可验证的做法是：**用一个幂等请求去确认链路，没确认到就重发**。
 *
 * @par 做法
 *  发一个幂等的 `QUERY_DEVICE_INFO(0x46)`，然后等响应；
 *  每轮只等 `JSDK_WARMUP_ATTEMPT_MS`（50 ms，设备正常时一个往返只要几毫秒），
 *  没等到就重发，直到总预算用完。**丢几次都无所谓**——丢的帧发生在预热阶段，
 *  而预热本身就是“发→等→重发”的循环。因此**用户可见的第一条命令不再会莫名失败**。
 *
 *  成功时会顺便把 `tx_retries` 记上（“第 2 次尝试成功”在 `-v` 里看得到）。
 *
 *  ⚠ **预热的收发不参与帧格式学习**（`jsdk_context_framing_learned()`）：
 *    设备对 0x46 的回包可能**按请求的格式回**（经典 8 B / FD 16 B），
 *    所以在自动模式下会学到“与配置一致”——那只是**镜像**，而
 *    帧格式学习只认第一次结果，一旦锁死“一致”就**永远发现不了对端其实是
 *    FD**（自动对齐失效、显式配置冲突也报不出来）。
 *    对端真实格式仍然从**设备主动发出的帧**（心跳/告警）判断；
 *    两个 CLI 的做法是“先只听一耳朵（`cycle_begin()` 只收不发）→ 再预热”。
 *
 * @par 契约
 *  - **幂等且无副作用**：只读一个设备信息，可以在任何“会话开始”的时刻调用；
 *  - 同一个会话里成功后**再调是空操作**（已预热就直接返回 OK）；
 *  - **失败不致命**：返回 `JSDK_ERR_TIMEOUT` 只代表“设备没有应答”，
 *    调用方可以继续（只收不发的命令 —— `hb-dump` 这类 —— 仍然可用）；
 *    但**别把它当成功**：真机上设备掉电/线松了就是这一步最先暴露。
 *  - 需要至少一个关节（用它来确定 `node_id`），否则返回 `JSDK_ERR_BAD_STATE`。
 *
 * @note 推荐顺序（也是两个 CLI 的做法）：
 *       先按 **Classic** 起步（`is_fd = 0; is_fd_explicit = 0;`，FD 控制器也收经典帧）
 *       → `jsdk_context_warmup()` → 看 `jsdk_context_framing_learned()` 决定是否切 FD。
 */
JSDK_API jsdk_status_t jsdk_context_warmup(jsdk_context_t *ctx, uint32_t timeout_ms);

/* ==========================================================================
 * 9. 总线级操作
 * ======================================================================== */

/**
 * 节点发现。两级：
 *   1. 被动：静置 2 个心跳周期（默认 200 ms）收集 MsgType 0x48 的 Source 字段；
 *   2. 主动：对 1..max_probe 逐一发 QUERY_STATUS(0x40)，收响应。
 * @param ids       输出数组
 * @param cap       数组容量
 * @param found     输出实际数量
 * @param max_probe 主动探测的最大节点 ID（建议 16，0 = 仅被动）
 * 注意：不得在 activate() 之后、控制循环运行期间调用（会争用响应）。
 */
JSDK_API jsdk_status_t jsdk_context_discover(jsdk_context_t *ctx, uint8_t *ids, unsigned cap,
                                    unsigned *found, uint8_t max_probe);

typedef struct {
    uint32_t tx_frames;      /**< SDK 累计发送帧数 */
    uint32_t rx_frames;      /**< SDK 累计接收帧数 */
    uint32_t tx_failed;      /**< 发送失败次数 */
    uint32_t rx_dropped;     /**< 解析失败/未知消息丢弃数 */
    uint32_t keepalive_sent; /**< 自动补喂狗帧数 */
    uint32_t tx_retries;     /**< 因**无响应**而重发的次数（会话预热 + 幂等请求重发）。
                                  非 0 就说明这条链丢过帧——正常（slcan 首帧丢失是已知行为），
                                  但值得看一眼：它只花掉一个往返，语义不变。 */
    uint32_t tx_retries_req; /**< 上面那个总数里属于**幂等请求重发**的部分（运行中途丢帧）。
                                  预热那种“会话开头丢帧”是已知且无害的；**运行中途**
                                  还在持续丢帧则说明链路/适配器有问题，所以单独计数、
                                  便于区分（`err`/`info`/`read` 这类单发命令现在会
                                  自动重发一次 —— 见 `jsdk_ctx_request_retry()`）。 */
    uint32_t last_rx_age_ms; /**< 距最后一次成功接收的毫秒数 */
    uint32_t hal_bus_flags;  /**< HAL 自报链路状态（JSDK_HAL_BUS_*；无回调则 0） */
    uint32_t link_errors;    /**< HAL `recv()` 报错次数（总线抖动/掉线） */
    uint8_t  nodes_online;   /**< 在线节点数（按收到的 Source 统计） */
    uint8_t  link_up;        /**< 1 = 链路当前可用（曾成功收发且无未恢复的链路错误） */

    /* ---- 链路质量观测（v0.33 新增；都是“只读计数器”，不影响任何协议行为） ---- */
    uint32_t tx_retries_warm;  /**< 其中属于**会话预热**的部分（`jsdk_context_warmup()`）。
                                    会话开头丢帧是已知行为、重发即可，看到非 0 不必惊慌 */
    uint32_t req_timeouts;     /**< 请求**等待超时**的次数（含后来被重发救回的）。
                                    判“链路到底稳不稳”看这个：
                                    `req_timeouts` 持续增长而 `tx_retries_req` 不增长 =
                                    重发也救不回（设备/线缆问题）；两个一起长 = 偶发丢帧 */
    uint8_t  last_retry_what;  /**< 最近一次自动重发的类别：0 从未 / 1 会话预热 / 2 幂等请求 */
    uint8_t  _reserved[3];     /**< 对齐占位（保持字段偏移稳定，不要写） */
    uint32_t last_retry_age_ms;/**< 距最近一次自动重发的毫秒数（`last_retry_what == 0` 时无意义） */
} jsdk_bus_state_t;

/**
 * 读总线/链路统计快照（不产生总线交互；**MCU 侧建议周期调用做健康告警**）。
 *
 * @note 字段里“归零/回填”的差异：`nodes_online`、`last_rx_age_ms`、`last_retry_age_ms`
 *       是按**当前时刻**现算的，其余是累计计数器。
 * @note 判“链路稳不稳”看 `req_timeouts`（等超时的次数，含被重发救回的）与
 *       `tx_retries{,_warm,_req}`（按类别拆分）；`last_retry_what` 说最近一次重发是哪类。
 */
JSDK_API jsdk_status_t jsdk_context_get_bus_state(jsdk_context_t *ctx, jsdk_bus_state_t *state);

/** 最近一次错误的可读文本（含关节/给定量/量程，可直接读给工程师）。 */
JSDK_API const char *jsdk_context_last_error(jsdk_context_t *ctx);

/** 广播 ESTOP（MsgType 0xC0，最高仲裁优先级）。载荷被固件忽略，无响应。 */
JSDK_API void jsdk_context_estop(jsdk_context_t *ctx);

/* ==========================================================================
 * 10. 关节反馈
 * ------------------------------------------------------------------------
 * 关节配置结构体 jsdk_joint_config_t 定义在 §6.2。
 * ======================================================================== */

typedef struct {
    double   pos;            /**< 输出端 rad */
    double   vel;            /**< 输出端 rad/s */
    double   current_A;      /**< 电机端 A（由 MIT 响应解码，量程需 torque_constant） */
    double   torque_Nm;      /**< 估算值：current_A × torque_constant × gear_ratio */
    double   t_motor_C;      /**< 电机温度（心跳，1 °C） */
    double   t_fet_C;        /**< MOSFET 温度（心跳，1 °C） */
    double   vbus_V;         /**< 母线电压（心跳） */
    double   ibus_A;         /**< 母线电流（心跳） */

    jsdk_axis_state_t axis_state;   /**< 归一化状态 */
    jsdk_mode_t       mode;         /**< 当前模式 */
    jsdk_modestate_t  mode_state;   /**< 固件原始 nibble */

    uint8_t  err_code;       /**< MIT 响应 4-bit ErrorCode（0 = NONE） */
    uint8_t  hb_error;       /**< 心跳 5-bit 子系统错误位图 */
    uint32_t axis_error;     /**< 可选：QUERY_ERROR(0x45) type=5 的 32-bit */

    uint32_t age_ms;         /**< 距上次有效反馈 */
    uint32_t tx_rejected;    /**< 累计被拒绝的越界指令数 */
    uint32_t tx_frames;      /**< 累计发出的控制帧数 */
    uint16_t status_flags;   /**< JSDK_JF_* 粘滞标志 */

    int      online;         /**< 曾经收到过有效帧 */
    int      valid;          /**< 本次数据来自新的有效帧 */
} jsdk_joint_feedback_t;

/**
 * 读**缓存**的关节反馈（不产生总线交互）。
 *
 * @note `age_ms` 是距最近一次有效反馈的毫秒数：**先看它再看数据** ——
 *       陈旧数据上的 `pos/vel` 不能当“当前状态”用（`JSDK_JF_STALE` 会置位）。
 */
JSDK_API jsdk_status_t jsdk_joint_get_feedback(const jsdk_joint_t *j, jsdk_joint_feedback_t *fb);

/** 是否已使能（使能序列走完才算 `1`；只发控制帧但没走完序列时为 `0`）。不产生总线交互。 */
JSDK_API int jsdk_joint_is_enabled(const jsdk_joint_t *j);
/** 是否有故障（MIT 4-bit 错误码 / 心跳位 / 0x45 明细的**并集**）。不产生总线交互。 */
JSDK_API int jsdk_joint_is_fault  (const jsdk_joint_t *j);

/** 固件原始 ModeState nibble（不经归一化），用于排障。 */
JSDK_API jsdk_modestate_t jsdk_joint_get_mode_state(const jsdk_joint_t *j);

/**
 * 固件 `axis0.current_state`（AxisState 0..16）—— **与上面的 ModeState nibble
 * 不是同一个枚举**。回报值 3 = 标定中、11 = 回零中。
 * @return 状态值；尚未读到过返回 -1。用 `jsdk_can_axis_state_name()` 取名字。
 */
JSDK_API int jsdk_joint_get_can_state(const jsdk_joint_t *j);

/** 清除粘滞状态标志位。 */
JSDK_API void jsdk_joint_clear_status_flags(jsdk_joint_t *j, uint16_t mask);

/* ==========================================================================
 * 11. 控制（单位：输出端 rad / rad/s / N·m）
 * ------------------------------------------------------------------------
 * 本组函数均为 void 且 RT 安全（无分支、无日志、不返回错误）。
 * 越界与非法值在 cycle_end() 编码阶段按 §6.10 策略处理。
 * ======================================================================== */

/* --- 使能与模式 ---
 *
 * ⚠⚠ `request_enable()` / `request_disable()` **都是非阻塞请求**，真正的状态变化
 *     发生在后续若干次 `cycle_end()` 里（见下）。**它们返回后设备并没有立刻动作**，
 *     这是本 API 最容易踩的地方：
 *
 *       - `request_enable()` 之后 `is_enabled()` 仍为 0，直到整条序列走完 ——
 *         期间**绝不允许**把电机当成"已使能"去下发运动指令；
 *       - `request_disable()` 之后 `is_enabled()` 仍可能为 1（实测仿真后端需 5 个
 *         控制周期才落地），也就是"我调了 disable()，电机还带着劲"。
 *         要"确认已断电再返回"，用 `jsdk_context_deactivate()`（阻塞，幂等）。
 *
 * 需要"请求并等到就绪"的语义，用 `jsdk_context_activate()` / `jsdk_context_deactivate()`。
 *
 * ## 使能序列（request_enable）
 * 1. `CLEAR_ERRORS` → 2. `START_MOTOR` → 3. 轮询直到 mode_state 进入闭环
 * → 4. 发**安全首帧**并置 `tx_active`。
 * 第 4 步是硬性安全要求：首帧的 pos 取**当前反馈位置**（若客户没设过目标）、
 * vel/tau 取 0、kp/kd 取 0 —— 因此"使能瞬间"电机既不跳向残留目标也不输出力矩，
 * 而是**原地保持**。见 §6.4。
 *
 * ## 失能序列（request_disable）
 * 1. 发一帧安全目标（先发再停，避免"停发即看门狗故障"）
 * → 2. **等 2 个控制周期** → 3. `STOP_MOTOR`
 * → 4. 轮询直到 mode_state 为 IDLE/RESET，然后清 `tx_active`。
 * 第 1、2 步不可省：直接 `STOP_MOTOR` 或干脆停发帧，设备会在 break_timeout 后
 * 报看门狗故障，现场表现为"正常关机却留下一个故障码"。
 */
JSDK_API void jsdk_joint_request_enable     (jsdk_joint_t *j, jsdk_mode_t mode);
JSDK_API void jsdk_joint_request_disable    (jsdk_joint_t *j);
JSDK_API void jsdk_joint_request_fault_reset(jsdk_joint_t *j);
JSDK_API void jsdk_joint_set_mode           (jsdk_joint_t *j, jsdk_mode_t mode);

/* --- 物理量入口（推荐） --- */
JSDK_API void jsdk_joint_set_target_position_rad  (jsdk_joint_t *j, double rad);
JSDK_API void jsdk_joint_set_target_velocity_rad_s(jsdk_joint_t *j, double rad_s);
JSDK_API void jsdk_joint_set_target_torque_Nm     (jsdk_joint_t *j, double Nm);

/* --- MIT 力位混合 --- */

/**
 * 发送 MIT 指令。**kp/kd 为线上值，原样透传**（与 cyberbeast_tool.py 一致）。
 *
 * @warning 固件把 kp/kd 作用在**电机端 turns 误差**上，而 pos/vel 由输出端换算而来，
 *          因此实际输出端刚度 ≠ kp：
 *              实际刚度 [N·m/rad] = kp × gear_ratio / (2π)
 *          以 gear_ratio = 16.5 计约为 kp 的 2.63 倍。
 *          需要按"真实刚度"给值时请用 jsdk_joint_set_mit_stiffness()。
 */
JSDK_API void jsdk_joint_set_mit(jsdk_joint_t *j,
                        double pos_rad, double vel_rad_s,
                        double kp, double kd, double tau_Nm);

/** 以**输出端真实刚度/阻尼**为输入；SDK 内部换算 kp = stiffness × 2π / gear_ratio。 */
JSDK_API void jsdk_joint_set_mit_stiffness(jsdk_joint_t *j,
                                  double pos_rad, double vel_rad_s,
                                  double stiffness_Nm_per_rad,
                                  double damping_Nm_per_rad_s,
                                  double tau_Nm);

/* --- 协议原始量入口（逃生通道） ---
 *
 * 语义 = **各模式的协议原始量**，SDK 不做任何换算。逐模式定义：
 *   MIT      : 不适用（请用 set_mit）
 *   CSP/POS  : raw / 1000 = 目标位置，单位【度】（输出端）
 *   CSV/VEL  : raw / 1000 = 目标速度，单位【RPM】（输出端）
 *   CST      : raw / 1000 = 目标力矩，单位【N·m】（输出端）
 *   CURRENT  : raw / 1000 = 目标电流，单位【A】（电机端）
 * SDK 将 raw 按 /1000 定点化后按模式编码；Classic 模式下 POS 的
 * vel_lim/cur_lim 为 i16 定点，超出部分按 §6.10 处理。
 */
JSDK_API void jsdk_joint_set_target_position(jsdk_joint_t *j, int32_t raw);
JSDK_API void jsdk_joint_set_target_velocity(jsdk_joint_t *j, int32_t raw);
JSDK_API void jsdk_joint_set_target_torque  (jsdk_joint_t *j, int16_t raw);

/* --- 限制量（CSP/POS、CSV/VEL、CURRENT 模式） --- */
JSDK_API void jsdk_joint_set_limits   (jsdk_joint_t *j, double vel_lim_rad_s, double cur_lim_A);
JSDK_API void jsdk_joint_set_current_A(jsdk_joint_t *j, double amp);  /**< 注意：不喂看门狗 */

/* --- 安全动作 --- */

/**
 * 按当前模式发送**最小能量**指令（默认"自由"，不主动抱持）：
 *   MIT            : pos = 实际位置, vel = 0, kp = kd = 0, tau = 0  → 电机泄力
 *   CSP/POS        : target = 实际位置，保留上次 vel_lim / cur_lim
 *   CSV/VEL        : target = 0
 *   CST/CURRENT    : 0
 * 无效（不发送）：若实际位置尚未获得（online == 0），本周期退回 keepalive 语义。
 */
JSDK_API void jsdk_joint_hold_position(jsdk_joint_t *j);

/** 主动抱持（PD 锁位）。kp/kd 为线上值，量纲同 jsdk_joint_set_mit()。 */
JSDK_API void jsdk_joint_hold_position_pd(jsdk_joint_t *j, double kp, double kd);

/* ==========================================================================
 * 12. 系统管理 / 运维（配置阶段 API，可阻塞等待响应）
 * ======================================================================== */

/** SET_ZERO(0x61)：把当前位置设为零点。不落 Flash。 */
JSDK_API jsdk_status_t jsdk_joint_set_zero_here(jsdk_joint_t *j);

/** 标定：向 axis0.requested_state 写 3（FULL_CALIBRATION_SEQUENCE），等待状态跳转。
 *  期间禁止发送控制帧。耗时可能数秒。 */
JSDK_API jsdk_status_t jsdk_joint_calibrate(jsdk_joint_t *j);

/** 回零：向 axis0.requested_state 写 11（AXIS_STATE_HOMING）。 */
JSDK_API jsdk_status_t jsdk_joint_home(jsdk_joint_t *j);

/** CONFIG_SAVE(0x22)：把当前配置写入 Flash（无响应，SDK 等待并读回校验）。 */
JSDK_API jsdk_status_t jsdk_joint_save_config(jsdk_joint_t *j);

/** RESET_DEVICE(0x64)：软复位。之后需重新握手。 */
JSDK_API jsdk_status_t jsdk_joint_reset_device(jsdk_joint_t *j);

/**
 * SET_NODE_ID(0x60)。注意：协议不落 Flash；persist=1 时 SDK 追加 CONFIG_SAVE，
 * 并在成功后更新本关节的 node_id。
 */
JSDK_API jsdk_status_t jsdk_joint_set_node_id(jsdk_joint_t *j, uint8_t new_id, int persist);

/**
 * 设置设备看门狗超时（写端点 73 = `can.config.break_timeout`，单位 ms）。
 *
 * **`ms == 0` = 关闭设备侧的协议级 CAN 超时检测**（最新固件 `auto_stop_if_timeout()`
 *   首句 `if (timeout_ms == 0) return;`，且配置项默认值就是 0）。
 *   早期固件把 0 当成 100 ms 且无法关闭，所以旧文档写的“0 ≠ 关闭”**已经过时**。
 *   `ms > 0` 时才真正开启；且这个保护**只在该设备收到过控制类帧（`is_ctrl`）后才武装**
 *   —— 纯 `CURRENT_CONTROL(0x04)` 的客户端永远武装不了（FIRMWARE_ISSUES F19）。
 *
 * @warning 写入不落 Flash；需要持久化请再调 `jsdk_joint_save_config()`。
 * @warning 真机（fw 1545）实测：**该端点的读回恒为 0**，写 250 立刻读也是 0。
 *          SDK 因此按“写入值”保守处理（继续喂狗是安全方向），但会置
 *          `JSDK_JF_WATCHDOG_UNVERIFIED` 并在 `last_error` 里说明“未能校验”。
 *          详见 `FIRMWARE_ISSUES.zh-CN.md` F28。
 */
JSDK_API jsdk_status_t jsdk_joint_set_watchdog_ms(jsdk_joint_t *j, uint32_t ms);

typedef struct {
    uint32_t hw_version;   /**< QUERY_DEVICE_INFO(0x46) */
    uint32_t fw_version;
    uint64_t serial;       /**< 仅 CAN FD 返回；Classic 为 0 */
    uint8_t  classic;      /**< 1 = 设备工作在 Classic 模式 */
} jsdk_device_info_t;

/**
 * 读设备信息（`QUERY_DEVICE_INFO` 0x46：hw/fw 版本 + 序列号；配置阶段 API，阻塞）。
 *
 * @note ⚠ **Classic 下响应只有 8 字节**（hw + fw），因此 `serial` 恒为 0 —— 那是协议
 *       如此，不是设备没序列号（FD 下是 16 字节、含序列号）。
 */
JSDK_API jsdk_status_t jsdk_joint_get_device_info(jsdk_joint_t *j, jsdk_device_info_t *info);

/* ==========================================================================
 * 13. 故障诊断
 * ======================================================================== */

typedef struct {
    int      valid;
    uint8_t  mit_err;      /**< MIT 响应 4-bit ErrorCode */
    uint8_t  hb_flags;     /**< 心跳 5-bit 子系统位图 */
    /* 以下来自 QUERY_ERROR(0x45)，0 表示未查询或该子系统无错误 */
    uint32_t motor_error;      /**< type 0 */
    uint32_t encoder_error;    /**< type 1 */
    uint32_t sensorless_error; /**< type 2 */
    uint32_t controller_error; /**< type 3 */
    uint32_t system_error;     /**< type 4 */
    uint32_t axis_error;       /**< type 5 */
} jsdk_fault_info_t;

/** 读取缓存的故障信息（不发起总线交互）。 */
JSDK_API jsdk_status_t jsdk_joint_get_fault_info(const jsdk_joint_t *j, jsdk_fault_info_t *info);

/** 主动发起 QUERY_ERROR(0x45) 六次查询并填充 info（配置阶段 API，可阻塞）。 */
JSDK_API jsdk_status_t jsdk_joint_query_error_detail(jsdk_joint_t *j, jsdk_fault_info_t *info);

/* --------------------------------------------------------------------------
 * 13.1 故障码文本（诊断用；**不在 RT 路径**，不保证数据竞争安全）
 *
 * 三套错误编码来源不同，必须分开看（表均取自固件 `Firmware/autogen/interfaces.hpp`）：
 *
 *   a) **MIT 响应的 4-bit ErrorCode**（`fb.err_code`）—— 节流过的摘要，等价于
 *      固件 `ErrorCode::` 枚举；`uint8_t` 输入，超范围返回 "?"。
 *   b) **心跳的 5-bit 子系统位图**（`fb.hb_error`）—— bit0 axis / bit1 motor /
 *      bit2 encoder / bit3 controller / bit4 board。
 *   c) **32-bit 子系统错误位图**（`jsdk_fault_info_t.*_error`，来自 0x45）——
 *      位名与固件枚举逐值对应；v8 只用到 bit 0、6..13、17..20。
 *
 * 这三个的**语义层级**完全不同（4-bit 是摘要，32-bit 是明细），不要互相比对。
 * ------------------------------------------------------------------------ */

/** @return MIT 4-bit ErrorCode 的名称（如 "CAN_TIMEOUT"）；未知返回 "?"。 */
JSDK_API const char *jsdk_joint_error_string(uint8_t mit_err_code);

/** 心跳 5-bit 子系统位名（bit 0..4）；越界返回 NULL。 */
JSDK_API const char *jsdk_hb_error_bit_name(unsigned bit);

/**
 * 找到 32-bit 错误位图里**最低**的置位，并返回其名称。
 * @param bit_out 可选输出：位号（0..21）
 * @return 名称；`value == 0` 返回 NULL（“无错误”）
 */
JSDK_API const char *jsdk_axis_error_first(uint32_t value, unsigned *bit_out);

/** 32-bit 错误位名（bit 0..21）；越界或未使用位返回 NULL。 */
JSDK_API const char *jsdk_axis_error_bit_name(unsigned bit);

/**
 * 固件 `AxisState`（0..16）的名称 —— 这是 `axis0.current_state` 的取值空间，
 * 与 `jsdk_modestate_t`（ModeState nibble，0..7）**不是同一个枚举**。
 * 注意 state = 16 无法装进心跳的 4 bit，只能从描述符端点读到（固件问题 F11，
 * 见 `docs/FIRMWARE_ISSUES.zh-CN.md`）。
 */
JSDK_API const char *jsdk_can_axis_state_name(uint8_t can_axis_state);

/**
 * 把关节的故障状态写成人可读的一行（CLI / 日志用）。
 *
 * 形如：`axis_error=0x00100000(CAN_BUS_FAILED) mit_err=CAN_TIMEOUT hb=axis|motor`
 * @param cap 建议 ≥ 128；始终 NUL 结尾
 * @return 写入的字符数（不含结尾 NUL）；参数非法返回 0
 */
JSDK_API int jsdk_joint_describe_fault(const jsdk_joint_t *j, char *buf, size_t cap);


/** 故障回调函数类型（传给 @ref jsdk_context_set_fault_callback）。 */
typedef void (*jsdk_fault_callback_t)(jsdk_joint_t *j, const jsdk_fault_info_t *info,
                                      void *user);

/**
 * 注册故障回调（**边沿触发**：只在“无故障 → 有故障”或故障位变化时调一次）。
 *
 * @param cb 传 `NULL` 即注销。
 * @warning RT 安全上下文（可能在 `cycle_begin/end` 里被调用）：禁止 sleep / malloc /
 *          加锁 / 打印，也不要回调 SDK 的阻塞 API。
 */
JSDK_API void jsdk_context_set_fault_callback(jsdk_context_t *ctx,
                                     jsdk_fault_callback_t cb, void *user);

/* ==========================================================================
 * 14. 参数访问（SDO 风格，映射到 PARAM_READ/WRITE 0x20 / 0x21）
 * ------------------------------------------------------------------------
 * 兼容说明：index 语义 = endpoint ID，subindex 保留固定 0。
 * Classic 模式下自动使用分段写；FD 模式自动使用批量读。
 * ======================================================================== */

typedef int jsdk_sdo_handle_t;

typedef enum {
    JSDK_SDO_IDLE = 0,
    JSDK_SDO_BUSY,
    JSDK_SDO_SUCCESS,
    JSDK_SDO_ERROR
} jsdk_sdo_state_t;

/** 创建参数槽位（每个关节最多 8 个，槽位 0 保留给故障详情自动读取）。
 *  @param subindex 保留参数，CYBERBEAST 恒为 0（端点 ID 是平铺的，无子索引）。
 *  @param size     缓冲区字节数；**0 = 从描述符自动推断类型长度**（推荐）。
 *  @return 句柄（0..7），失败返回 -1。 */
JSDK_API jsdk_sdo_handle_t jsdk_joint_sdo_create(jsdk_joint_t *j, uint16_t ep_id,
                                        uint8_t subindex, size_t size);

/** 按路径名创建参数槽位（动态描述符的自然用法）。未命中返回 -1。 */
JSDK_API jsdk_sdo_handle_t jsdk_joint_sdo_create_by_name(jsdk_joint_t *j, const char *path);
JSDK_API jsdk_sdo_state_t  jsdk_joint_sdo_state    (jsdk_joint_t *j, jsdk_sdo_handle_t h);

/** 参数槽位的**裸字节缓冲区**（长度 = data_size()，由描述符类型推断或创建时指定）。
 *
 *  ⚠ **字节序**：这里就是**线上字节**，不经过类型化解码 —— 参数值是
 *  **小端**（设备端把端点内存原样 memcpy 进/出载荷），而控制帧/查询响应才是大端。
 *  也就是说：`*(uint16_t *)(void *)jsdk_joint_sdo_data(...)` 在本项目支持的平台
 *  （x86 / ARM，均小端）上直接可用，但**把这段字节发到别处前要知道它是小端**。
 *  不想碰裸字节就用 `jsdk_joint_param_get/set*()`（内部按类型做小端编解码）。
 *  证据与真机实测见 `src/proto_cyberbeast/cb_frame.h` 的 `cb_le_*` 说明。 */
JSDK_API uint8_t          *jsdk_joint_sdo_data     (jsdk_joint_t *j, jsdk_sdo_handle_t h);
JSDK_API size_t            jsdk_joint_sdo_data_size(jsdk_joint_t *j, jsdk_sdo_handle_t h);

/** 发起读/写（非阻塞，返回 0 表示已发起）。用 sdo_state() 轮询结果。 */
JSDK_API int jsdk_joint_sdo_read (jsdk_joint_t *j, jsdk_sdo_handle_t h);
JSDK_API int jsdk_joint_sdo_write(jsdk_joint_t *j, jsdk_sdo_handle_t h);

/** 通用类型化读/写。path 形如 "axis0.controller.config.mit_max_torque"，
 *  由运行时解析的描述符解析为端点 ID（见 §18）。
 *  未命中返回 JSDK_ERR_NOT_FOUND；类型不匹配返回 JSDK_ERR_PROTOCOL。
 *
 *  读的字节数由**描述符里的类型长度**决定，8 字节类型（`u64/i64/f64`，
 *  例如 `serial_number`、`axis0.motor.error`）也能读满：
 *  FD 链路一次请求完成；**Classic 链路会自动分两块**（设备侧把单次 `ReqLen`
 *  钳到 4），因此 Classic 上读 8 字节值的耗时约为 4 字节值的两倍。
 *  非标量类型（`object`/`json`/`endpoint_ref`）没有标量尺寸 → 返回
 *  `JSDK_ERR_UNSUPPORTED`（不做"读一半"）。 */
JSDK_API jsdk_status_t jsdk_joint_param_get(jsdk_joint_t *j, const char *path, jsdk_value_t *out);
JSDK_API jsdk_status_t jsdk_joint_param_set(jsdk_joint_t *j, const char *path, const jsdk_value_t *in);

/* 类型化便利包装（实际端点中大量为整型：node_id/heartbeat_rate_ms 为 u32、
 * break_timeout 为 u16、requested_state 为 u8、enable_watchdog 为 bool） */
JSDK_API jsdk_status_t jsdk_joint_param_get_f32 (jsdk_joint_t *j, const char *path, float    *out);
JSDK_API jsdk_status_t jsdk_joint_param_set_f32 (jsdk_joint_t *j, const char *path, float     v);
JSDK_API jsdk_status_t jsdk_joint_param_get_u32 (jsdk_joint_t *j, const char *path, uint32_t *out);
JSDK_API jsdk_status_t jsdk_joint_param_set_u32 (jsdk_joint_t *j, const char *path, uint32_t  v);
JSDK_API jsdk_status_t jsdk_joint_param_get_i32 (jsdk_joint_t *j, const char *path, int32_t  *out);
JSDK_API jsdk_status_t jsdk_joint_param_get_bool(jsdk_joint_t *j, const char *path, int      *out);

/**
 * 批量读（混合类型）。FD 下按各端点类型长度自动分组打包到单帧（必要时拆多帧）；
 * Classic 下自动退化为逐条（固件对 Classic 批量请求恒回 ERR）。
 */
typedef struct {
    const char   *path;
    jsdk_value_t  value;   /**< 输出 */
    jsdk_status_t status;  /**< 输出：本条的结果 */
} jsdk_param_req_t;

/**
 * 批量读参数（**一次请求拿多个端点**，配置阶段 API，阻塞）。
 *
 * @param reqs 请求/结果数组（就地回填 `value` / `len` / `status`）；`n` ≤ 描述符
 *             允许的批量上限（见 `jsdk_param_req_t`）。
 * @note FD 下走设备原生批量读；**Classic 下无法批量**（一帧只能回 4 字节）⇒
 *       **自动退化为逐条读**，语义不变、只是慢（原因写在 `jsdk_context_last_error()`）。
 */
JSDK_API jsdk_status_t jsdk_joint_param_get_batch(jsdk_joint_t *j, jsdk_param_req_t *reqs, unsigned n);

/* ==========================================================================
 * 15. 分组与广播同步（仅 node_id 1..7 可被位掩码寻址）
 * ======================================================================== */

typedef struct {
    uint8_t node_id;
    double  pos_rad, vel_rad_s, kp, kd, tau_Nm;
} jsdk_group_target_t;

/**
 * 一条 CAN FD 帧同时驱动多个关节（槽位 = node_id，每槽 8 字节）。
 *
 * **调用时机**：必须在 `jsdk_context_cycle_begin()` 与 `jsdk_context_cycle_end()`
 * 之间调用一次。本调用自己就把帧发出去了（不走 cycle_end 的补发逻辑），并给组内
 * 每个关节打上"本周期已发"标记 —— 于是在 `cycle_end()` 不会再补发单播，
 * `n` 个关节就真的只占 **1 条帧**。在周期之外调用不会报错，但下一个
 * `cycle_begin()` 会清掉该标记，`cycle_end()` 因而又给每个关节补一条单播。
 *
 * 约束与行为：
 *   - 组内 node_id 必须 ∈ 1..7；≥8 的成员使本调用返回 JSDK_ERR_UNSUPPORTED
 *     （配置阶段会提前告警，见 configure()）；
 *   - 成员必须**已使能**（`tx_active`）、**已标定**，且当前模式为
 *     `JSDK_MODE_MIT` —— 否则返回 `JSDK_ERR_BAD_STATE`。最后一条很重要：本函数发的是
 *     MIT 帧，若关节正按 CSP/CSV/力矩模式驱动，一条 MIT 广播会把设备**悄悄切到
 *     MIT 输入模式**；
 *   - Classic 模式下仅支持"全员同一目标"（槽位 0，帧长 8 B），目标不一致时
 *     **自动降级为逐关节单播**；
 *   - 组内有成员的位形/速度/力矩/kp/kd 超出该关节量程时**不静默钳位**，同样降级为
 *     单播，让 §6.10 的策略（发安全帧）生效；
 *   - FD 路径上未被使用的槽位会被显式写入"零增益指令"。**切不可改为全零字节**：
 *     MIT 载荷的 0 是各字段的最小码（pos → −pos_max、tau → −tau_max），全零等于
 *     满力矩反向指令；
 *   - 降级是**正常结果**，不是失败：此时返回值仍是 `JSDK_OK`，原因文本在
 *     `jsdk_context_last_error()` 里（建议周期性打印一次，避免客户以为还是同步广播）；
 *   - **广播不回复**：本调用不更新任何反馈，反馈依赖心跳与后续单播查询。
 *     只驱动 1 个关节时请直接用 `jsdk_joint_set_mit()`：单播会带来 MIT 响应帧，
 *     而广播帧不会有响应。
 */
JSDK_API jsdk_status_t jsdk_group_set_mit(jsdk_context_t *ctx, const jsdk_group_target_t *targets,
                                 unsigned n);

/**
 * 成组使能 / 失能（`node_ids` 里每个节点**逐个单播**）。
 *
 * @note ⚠ 协议**没有**广播版 `START_MOTOR`/`STOP_MOTOR`，所以这不是“一条帧驱动 N 台”
 *       —— 与 @ref jsdk_group_set_mit() 的广播语义不同，别把它当同步使能。
 *       “无法广播，已改为逐个单播”会写进 `jsdk_context_last_error()`。
 * @note 返回 `JSDK_OK` 只表示**请求已排队**（`request_enable/disable` 语义），
 *       真正的状态变化由调用者的 `cycle_begin/cycle_end` 推进，看
 *       @ref jsdk_joint_is_enabled()。
 */
JSDK_API jsdk_status_t jsdk_group_enable (jsdk_context_t *ctx, const uint8_t *node_ids, unsigned n);
JSDK_API jsdk_status_t jsdk_group_disable(jsdk_context_t *ctx, const uint8_t *node_ids, unsigned n);

/* ==========================================================================
 * 16. 单位与标定
 * ------------------------------------------------------------------------
 * ⚠ **本后端的 scale 与 EtherCAT 版同名不同义**（迁移者最容易踩的一条）：
 *
 *   | | CAN / CyberBeast（本库） | EtherCAT 版 |
 *   |---|---|---|
 *   | 线上量 | 模式映射之后**已经是物理量**（MIT = 输出端 rad / rad·s⁻¹ / N·m） | 原始**编码器计数** |
 *   | `unit_scale_default()` | **恒等映射**（三个比例都是 `1.0`） | counts → rad 的真实比例 |
 *   | 什么时候需要 `_calc()` | 只有你**按计数驱动**时（例如用 `set_*_raw()` 下发自己约定的计数） | 总是需要 |
 *
 *   换句话说：**CAN 上基本不需要 scale** —— 只有在“把线上量当计数用”的时候才需要。
 *   详见 `docs/UNITS.zh-CN.md`。
 * ======================================================================== */

typedef struct {
    double pos_counts_to_rad;    /**< 输出端：1 命令单位 = N rad（CAN 默认 1.0 = 恒等） */
    double vel_counts_to_rad_s;  /**< 输出端：1 命令单位/s = N rad/s（CAN 默认 1.0） */
    double trq_to_Nm;            /**< 1 力矩单位 = N N·m（CAN 默认 1.0） */
    int    valid;                /**< 0 = 调用方并不知道这台电机的标定（**不是**“数据非法”）：
                                      本后端只看 `rated_trq > 0` 就置 1（见 `_default()`），
                                      `_calc()` 则在任一入参为 0 时置 0。
                                      置 0 会把关节的 `JSDK_JF_SCALE_INVALID` 粘滞位置起。 */
} jsdk_unit_scale_t;

/**
 * 取**本后端默认**的 scale（CAN 上是恒等映射）。
 *
 * @param rated_trq 额定力矩（N·m）；**只用来判断“调用方是否真的知道这台电机”**：
 *                  `> 0` → `valid = 1`，否则 `valid = 0`。三个比例恒为 `1.0`。
 * @note 为什么这里不是“从设备读标定”：本后端的线上量已经是物理量（`configure()`
 *       读回的标定值用于量程校验，不是用来换算的）。所以恒等映射不是“没实现”。
 * @note `scale == NULL` 时直接返回（不报错）。
 */
JSDK_API void jsdk_unit_scale_default(jsdk_unit_scale_t *scale, uint32_t rated_trq);

/**
 * 按**编码器计数**换算的 scale（面向“按计数驱动”的场景）。
 *
 * @param encoder_resolution 编码器每**电机**转的计数（CPR，正交后）
 * @param motor_rev          齿轮箱电机侧转数
 * @param shaft_rev          齿轮箱输出侧转数（`gear = shaft_rev / motor_rev`）
 * @param rated_torque       额定力矩（N·m）
 *
 * 公式（编码器装在**电机**侧）：
 * @verbatim
 *   pos_counts_to_rad = 2π × motor_rev / (encoder_resolution × shaft_rev)
 *   vel_counts_to_rad_s = pos_counts_to_rad
 *   trq_to_Nm         = rated_torque / 1000      （线力矩单位 = 0.1% 额定）
 * @endverbatim
 *
 * @note 任一参数为 0、或算出的比例不合理（≤0 或 > 1e12）→ `valid = 0`
 *       且三个比例**全部置 0**（**绝不猜**：宁可让调用方看见“无效”，也不要给一个错的换算）。
 * @note 常规用法（MIT/CSP/CSV/CST 的物理量 API）**不需要**调用本函数。
 */
JSDK_API void jsdk_unit_scale_calc(jsdk_unit_scale_t *scale,
                          uint32_t encoder_resolution,
                          uint32_t motor_rev,
                          uint32_t shaft_rev,
                          uint32_t rated_torque);

/**
 * 设置关节用的 scale，并同步 `JSDK_JF_SCALE_INVALID` 粘滞位。
 *
 * @param scale  `valid == 0` 时置位 `JSDK_JF_SCALE_INVALID`（提示“这个关节的标定不可信”），
 *               非 0 时清除该位。**不改动任何线上量**：本后端的物理量入口本来就直通。
 * @note 只是“告诉 SDK 你按什么比例在理解 raw 量”，不会触发总线交互。
 */
JSDK_API void jsdk_joint_set_scale(jsdk_joint_t *j, const jsdk_unit_scale_t *scale);

/**
 * 读回当前关节用的 scale（`configure()` 已按设备标定填好）。
 *
 * @param scale 输出；`j` 非法时写回 `jsdk_unit_scale_default(scale, 0)`（`valid = 0`），
 *              **不会**留下未初始化的结构体。
 */
JSDK_API void jsdk_joint_get_scale(const jsdk_joint_t *j, jsdk_unit_scale_t *scale);

/* ==========================================================================
 * 17. 文本与自检
 * ======================================================================== */

/**
 * 枚举 → 人类可读文本（诊断/日志用，**不在 RT 路径**）。
 *
 * @note 四个函数都**永不返回 NULL**：不认识的值返回 `"unknown-..."`，
 *       所以可以直接 `printf("%s", ...)`。返回的是**静态**字符串，不要 free。
 */
JSDK_API const char *jsdk_status_string(jsdk_status_t status);
JSDK_API const char *jsdk_axis_state_string(jsdk_axis_state_t state);
JSDK_API const char *jsdk_mode_string(jsdk_mode_t mode);
JSDK_API const char *jsdk_ep_type_string(jsdk_ep_type_t type);

/* ==========================================================================
 * 18. JSON 描述符与端点解析（动态；类型定义见 §5.5）
 * ------------------------------------------------------------------------
 * 与两个 EtherCAT 后端不同，本后端**没有编译期端点表**：端点 ID 完全来自
 * 运行时从设备读回的 JSON 描述符。因此不仅免除了端点 ID 漂移（实测 86%），
 * 也让客户能访问任意底层参数，无需依赖 SDK 发布新版本。
 * ======================================================================== */

/** 上下文本身所需字节数（**不含** desc.arena）。用于精确声明存储。 */
JSDK_API size_t jsdk_context_size(const jsdk_context_config_t *cfg);

/**
 * 估算描述符解析区所需字节数（保守上界）。
 * 典型值：RETAIN_ALL ≈ 25 KB；RETAIN_FILTERED（约 12 条路径）≈ 0.5 KB。
 * 建议先按本函数分配，运行后读 desc.arena_used 按实测缩容。
 *
 * @note RETAIN_FILTERED 下本函数给出的值基于 filter 字符串长度**估算**
 *       （前缀匹配的实际路径可能更长），已含余量；真实校验在解析时进行，
 *       不足会返回 JSDK_ERR_NO_MEMORY。
 * @note RETAIN_ALL 下无法预知端点数，本函数返回一个保守推荐值（32 KB）。
 */
JSDK_API size_t jsdk_desc_arena_size(const jsdk_desc_config_t *cfg);

/**
 * 下载并解析 JSON 描述符（阻塞式，仅可用于配置阶段，最长 timeout_ms）。
 *
 * 流程：一次 JSON_DESC_READ(0x24) 请求 → 设备自主流式回送 0x25 帧（每 ms 最多 50 帧）
 *      → SDK 逐帧增量解析（无递归、无 malloc、不需 41 KB 缓冲）→ 按 retain 策略入库。
 * 共享：同一总线上 (fw_version, crc) 相同的节点只下载一次（share_by_crc）。
 *
 * @warning 禁止在任一关节使能时调用：描述符帧会占满 TX 并挤掉控制帧，
 *          导致驱动器看门狗（break_timeout）触发 disarm。此时返回 JSDK_ERR_BAD_STATE。
 * @warning 本函数的 RX 排空不受 context_config.rx_burst_limit 限制。
 */
JSDK_API jsdk_status_t jsdk_context_desc_fetch(jsdk_context_t *ctx);

/** 非阻塞推进（MCU 主循环用）。返回 JSDK_ERR_BUSY 表示仍在进行中。 */
JSDK_API jsdk_status_t jsdk_context_desc_poll(jsdk_context_t *ctx, uint64_t app_time_ns);

/** 下载进度回调（配置阶段调用；允许慢速操作）。 */
/** 描述符下载进度回调函数类型（传给 @ref jsdk_context_set_desc_progress）。 */
typedef void (*jsdk_desc_progress_fn)(jsdk_context_t *ctx, uint32_t bytes_done,
                                      uint32_t bytes_total, void *user);

/**
 * 注册下载进度回调（**可选**，仅配置阶段用）。
 *
 * @param fn 传 `NULL` 即注销。回调在下载过程中被周期调用，可以画进度条。
 * @warning 回调内**不得**调用其它 SDK API（重入）。
 */
JSDK_API void jsdk_context_set_desc_progress(jsdk_context_t *ctx,
                                    jsdk_desc_progress_fn fn, void *user);

/** 描述符元信息。 */
typedef struct {
    uint32_t total_len;        /**< 设备侧 JSON 字节数 */
    uint16_t crc;              /**< VersionCRC —— 缓存与节点间共享的键 */
    uint32_t fw_version;       /**< QUERY_DEVICE_INFO(0x46) 的 fw 字段 */
    uint32_t hw_version;
    unsigned endpoint_count;   /**< 已保留（入 arena）的端点数 */
    unsigned parsed_total;     /**< 实际解析到的端点数（含未保留） */
    unsigned frames_rx;        /**< 消耗的 0x25 帧数 */
    unsigned bytes_scanned;
    uint8_t  complete;         /**< 1 = 扫描完整份；0 = filter 满足后提前终止。
                                    ⚠ complete == 0 时**不得**缓存原始 JSON（数据不完整） */
    uint8_t  mode_used;        /**< 实际生效的 jsdk_desc_mode_t */
    uint8_t  shared_hit;       /**< 1 = 复用了同总线其它节点已解析的结果 */
    uint8_t  raw_sink_failed;  /**< 1 = 原始字节流出出错或未安装（raw 缓存不可用） */
    unsigned retries;          /**< 本次下载**重发** `0x24` 请求的次数（“请求丢了”的直接证据）。
                                    ⚠ 大概率不是 0：真机 slcan 实测第一条请求会丢，
                                    重发救回属于正常；**持续**增长才是问题 */
} jsdk_desc_info_t;

/**
 * 读描述符元信息（不产生总线交互）。
 *
 * @note **描述符还没下载时返回 `JSDK_ERR_BAD_STATE`** —— 而不是给一份全 0 的
 *       “看起来像空的描述符”（那会让“没下载”和“设备描述符是空的”分不清）。
 * @note `retries` 是本次下载重发 `0x24` 请求的次数：非 0 说明**请求丢过**
 *       （真机 slcan 上很常见），持续增长才说明链路有问题。
 */
JSDK_API jsdk_status_t jsdk_context_get_desc_info(jsdk_context_t *ctx, jsdk_desc_info_t *info);

/** 名称 → 端点 ID / 类型 / 权限。未命中返回 JSDK_ERR_NOT_FOUND（**不猜、不近似**）。 */
JSDK_API jsdk_status_t jsdk_endpoint_lookup(jsdk_context_t *ctx, const char *path,
                                   uint16_t *ep_id, jsdk_ep_type_t *type,
                                   uint8_t *access);

/** 遍历已保留的端点（供 CLI `ep-list` / Python `Context.endpoints()` 使用）。
 *  回调返回非 0 即停止遍历。 */
/** 端点遍历回调：返回非 0 即停止遍历。 */
typedef int (*jsdk_endpoint_visit_fn)(void *user, const char *path, uint16_t ep_id,
                                      jsdk_ep_type_t type, uint8_t access);

/**
 * 遍历**已保留**的端点（顺序 = 描述符里的声明顺序，不保证字典序）。
 *
 * @param fn 回调；返回非 0 立即停止遍历（用于“找到就收工”）。
 * @note 一次遍历**不产生**总线交互（表已在内存里）。
 */
JSDK_API jsdk_status_t jsdk_endpoint_enumerate(jsdk_context_t *ctx,
                                      jsdk_endpoint_visit_fn fn, void *user);

/**
 * 原始描述符字节的“流出”（tee）回调。
 *
 * 下载过程中，SDK 在**解析之前**把每段 JSON 字节交给本回调；典型用途是直接
 * 写入外部 Flash 作为缓存（见 docs/PORTING.zh-CN.md §3）。SDK 自己不保留原始
 * JSON（全程流式解析，不需 41 KB 缓冲），因此这是应用获取原始字节的**唯一途径**。
 *
 * @param data   本次可用字节（**仅在回调期间有效**，必须立即复制或写入 Flash）
 * @param len    字节数
 * @param offset 本段在描述符中的偏移（便于写入 Flash 的固定偏移）
 * @return 0 = 成功；非 0 = 放弃（下载继续，但 desc_info.raw_sink_failed = 1，
 *              且此后不再调用本回调）
 *
 * @warning 在配置阶段（非 RT）调用，**允许阻塞**（例如等待 Flash 写入完成）。
 * @warning 必须配合 stop_when_satisfied = 0，否则流会被提前终止、缓存不完整。
 */
typedef int (*jsdk_desc_raw_sink_fn)(jsdk_context_t *ctx, const void *data,
                                     size_t len, uint32_t offset, void *user);

/** 安装（fn 非 NULL）或取消（fn = NULL）原始字节流出回调。 */
JSDK_API void jsdk_context_set_desc_raw_sink(jsdk_context_t *ctx,
                                    jsdk_desc_raw_sink_fn fn, void *user);

/* ---- 描述符缓存（避免每次上电重复下载 41 KB / 662 帧） ----
 *
 * 两条路线，按 Flash 预算与是否需要现场调参选择：
 *
 *   A) 缓存**已解析结果**：desc_export() 存起来 → 启动时 desc_import()。
 *      体积小（RETAIN_FILTERED 约 0.5 KB），但结果被 retain / filter_paths /
 *      SDK 内部格式三者绑定，应用封装头里必须把这三项也纳入失效键
 *      （见 docs/PORTING.zh-CN.md §3.6）。
 *
 *   B) 缓存**原始 JSON**（推荐）：安装 desc_raw_sink 把下载流同时写到 Flash
 *      （约 41 KB），启动时 desc_import_raw() 重新解析。只与
 *      (fw_version, desc_crc) 两个键绑定 —— 改 filter 或升级 SDK 均无需重新下载。
 *      ⚠ 必须 stop_when_satisfied = 0，且事后校验 desc_info.complete == 1。
 */
JSDK_API size_t        jsdk_desc_export_max_size(const jsdk_context_t *ctx);
/**
 * 把已解析的描述符导出成**紧凑格式**（路线 A：MCU 侧 Flash 缓存，免掉重复下载与解析）。
 *
 * @param buf 输出缓冲，`cap` 字节；不够返回 `JSDK_ERR_BUFFER_TOO_SMALL`（`out_len` 仍写需求值）。
 * @note 导出是**版本/参数相关**的：缓存只能配**相同的 retain / filter / max_endpoints /
 *       max_path_len** 组合用（这些写在缓存头里，@ref jsdk_context_desc_import() 会校验）。
 */
JSDK_API jsdk_status_t jsdk_context_desc_export(jsdk_context_t *ctx, void *buf, size_t cap,
                                       size_t *out_len);

/**
 * 导入 @ref jsdk_context_desc_export() 产出的**紧凑格式**（路线 A 的 Flash 缓存）。
 *
 * @note 只与**相同的 retain / filter / max_endpoints / max_path_len** 组合兼容
 *       （这些都在缓存头里，不一致 → `JSDK_ERR_BAD_STATE`，**不会**给你半份表）。
 * @note 导入成功后 `configure()` 不再下载描述符。
 * @note 要导入**设备原始 JSON**（改 filter 无需重下）请用
 *       @ref jsdk_context_desc_import_raw()。
 */
JSDK_API jsdk_status_t jsdk_context_desc_import(jsdk_context_t *ctx, const void *buf, size_t len);

/** 原始 JSON 的元信息提示（路线 B 必需）。由应用从自己的缓存头提供（PORTING §3.2）。 */
typedef struct {
    uint16_t crc;         /**< 描述符 VersionCRC（下载时从 desc_info.crc 取得并存入 Flash） */
    uint32_t fw_version;  /**< 设备固件版本（QUERY_DEVICE_INFO 0x46） */
} jsdk_desc_hint_t;

/**
 * 用**原始 JSON** 构建端点表（不经 CAN）。用于从 Flash 缓存恢复，见 PORTING §3。
 *
 * 与 jsdk_context_desc_import() 的区别：
 *   - import()     ：导入 SDK 自己的紧凑格式（export() 的产物），已解析、已按
 *                    retain 裁剪；只与相同的 retain/filter 组合兼容。
 *   - import_raw() ：导入设备原始 JSON，**按当前 cfg.desc 重新解析**
 *                    （retain / filter_paths 生效），因此改 filter 无需重新下载。
 *
 * @param hint 必需。传 NULL 返回 JSDK_ERR_INVALID_ARG。
 * @note 成功后描述符即视为已存在，configure() 不会再下载。
 * @note 截断/非法 JSON → JSDK_ERR_PARSE；arena 不足 → JSDK_ERR_NO_MEMORY。
 *       两种情况都**不留下部分结果**。
 */
JSDK_API jsdk_status_t jsdk_context_desc_import_raw(jsdk_context_t *ctx,
                                           const void *json, size_t len,
                                           const jsdk_desc_hint_t *hint);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* JOINT_SDK_JOINT_SDK_H */
