# 移植指南（PORTING）

> **读者**：自己实现 CAN 后端（MCU / 自研适配器 / RTOS）的工程师，以及需要做**启动时间与 RAM 预算**的系统工程师。
> **前置**：先读 `docs/DESIGN.zh-CN.md` §6.6（全动态端点解析的决策与硬约束）和 `docs/PROTOCOL_NOTES.zh-CN.md` §5.6（描述符传输细节）。
> **本篇重点**：① 实现 `jsdk_can_hal_t`；② **arena 选型**；③ **把描述符缓存到外部 Flash 的完整流程**。

---

## 0. 一句话结论

SDK 不含任何平台代码、不做任何堆分配。移植 = 实现 **4 个回调** + 提供 **1 块 arena** + （强烈建议）**把描述符缓存到 Flash**。
不做第 3 步的代价是：每次上电都要花 **0.1~2 s** 从驱动器下载 41029 字节的描述符。

缓存有两条路线（详见 §3.1b 与 §7）：**A 缓存解析结果**（小，≈0.5 KB）或 **B 缓存原始 JSON**（≈41 KB，更稳）。
路线 B 需要 SDK 的两个接口：下载时用 `jsdk_context_set_desc_raw_sink()` 把字节流出到 Flash，
启动时用 `jsdk_context_desc_import_raw()` 解析回来。

---

## 1. 最小移植：实现 `jsdk_can_hal_t`

### 1.1 回调契约

| 回调 | 契约 | 违反后果 |
|---|---|---|
| `send(user, frame)` | **非阻塞**。0 = 已发出/已入队；非 0 = 失败或忙。 | 若阻塞等待 ACK，会拖长控制周期 → 看门狗风险 |
| `recv(user, frame)` | **非阻塞**。1 = 取到一帧；0 = 当前无帧；<0 = 链路错误。 | 返回 0 但实际有帧 = 反馈延迟；返回 <0 会被记入 `rx_dropped` |
| `now_ms(user)` | **单调**毫秒时钟。SDK 内部用无符号差值，`uint32_t` 回绕安全。 | 用非单调时基（如 RTC 校准）会导致看门狗误判 |
| `bus_status(user, &flags)` | 可选（NULL 允许）。填 `JSDK_HAL_BUS_*` 位。 | 无法在 `jsdk_bus_state_t` 看到 bus-off |
| `on_error(user, kind, detail)` | 可选（NULL 允许）。**从中断回调也可以调用**（SDK 内部只做计数/置位）。 | 丢失 bus-off 早期告警 |

**`send` / `recv` 都在「非中断上下文」被调用**（控制循环里）。因此 HAL 内部只需保护与 ISR 共享的环形队列。

### 1.2 帧结构约定

```c
typedef struct {
    uint32_t id;       /* 29-bit 扩展 ID，**不要左移**，不要带 EFF 标志位 */
    uint8_t  len;      /* 数据长度：Classic 0..8，FD 0..64 */
    uint8_t  flags;    /* JSDK_FRAME_FD | JSDK_FRAME_BRS（| JSDK_FRAME_EXT） */
    uint8_t  data[64];
} jsdk_can_frame_t;
```

| 注意点 | 说明 |
|---|---|
| 扩展帧 | CYBERBEAST **只用 29-bit 扩展帧**。HAL 必须设置硬件 EFF 位，并在接收时过滤非扩展帧 |
| FD 检测 | `flags & JSDK_FRAME_FD`。必须与设备 `can.config.baud_rate` 匹配，**协议无运行时协商**；不匹配 = 静默无响应 |
| BRS | `flags & JSDK_FRAME_BRS` → 硬件需使能比特率切换（并正确配置 TDC/SSP） |
| 长度 | Classic 帧 `len` 必须 ≥ 实际使用长度；设备对 MIT 帧会校验 `len >= (slot+1)*8`，**长度不足会整帧丢弃** |
| 过滤器 | 建议只放行**扩展帧 + 目标 node_id**，避免无关帧占满队列。`0x40..0x49` 查询回复的 ID 中 `Dest` 是本机 `master_id`，也是过滤依据 |

### 1.3 环形队列深度怎么定

SDK 单周期最多处理 `jsdk_context_config_t.rx_burst_limit` 帧（默认 32），但**描述符下载走独立排空循环，不受该限制**：设备以 **≤50 帧/ms** 灌入，按 1 ms 控制周期算就是每秒 5 万帧的理论突发。

| 场景 | 建议 RX 队列深度（帧） | 说明 |
|---|---|---|
| 仅常规控制（FD） | 16 ~ 32 | 每周期回 1 帧 MIT 响应 + 偶发心跳 |
| 仅常规控制（Classic） | 8 ~ 16 | 同上 |
| **需要下载描述符** | **≥ 128（FD）/ ≥ 256（Classic）** | 必须能吸收突发，否则丢帧导致 JSON 断裂 → 整体失败重来 |
| 多关节（>4 轴）+ 广播 | 64 ~ 128 | 每个单播指令都有响应 |

**溢出策略建议**：丢弃**最旧的**帧并累加一个溢出计数（供 `jsdk_bus_state_t.rx_dropped` 参考），**不要**丢弃最新帧——描述符流的后续字节比前面的更重要。

> ⚠ 若按 32 深度实现却在 Classic 下下载描述符，几乎必然丢帧。两个可行做法：
> ① 把队列开到 ≥256；② 用 `jsdk_context_desc_poll()` 在**主循环里高频调用**（而非跟着 1 ms 控制周期），把排空速率提上去。

### 1.4 参考骨架（以 STM32 FDCAN 为例）

```c
/* ---------- HAL 私有状态 ---------- */
#define JSDK_RXQ_LEN 256u          /* 见 §1.3 */

typedef struct {
    jsdk_can_frame_t buf[JSDK_RXQ_LEN];
    volatile uint16_t head, tail;  /* head: ISR 写入；tail: 主循环读出 */
    volatile uint32_t ovf;
} rxq_t;

static rxq_t              s_rxq;
static FDCAN_HandleTypeDef *s_hfdcan;

/* ---------- ISR：把硬件 FIFO 搬进环形队列 ---------- */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *h, uint32_t fifo)
{
    FDCAN_RxHeaderTypeDef hdr;
    jsdk_can_frame_t f;
    while (HAL_FDCAN_GetRxMessage(h, FDCAN_RX_FIFO0, &hdr, f.data) == HAL_OK) {
        if (hdr.IdType != FDCAN_EXTENDED_ID) continue;   /* 只收扩展帧 */
        f.id    = hdr.Identifier & 0x1FFFFFFFu;
        f.len   = (uint8_t)hdr.DataLength;
        f.flags = (hdr.FDFormat == FDCAN_FD_CAN) ? JSDK_FRAME_FD : 0u;
        if (hdr.BitRateSwitch == FDCAN_BRS_ON) f.flags |= JSDK_FRAME_BRS;
        f.flags |= JSDK_FRAME_EXT;

        uint16_t nh = (uint16_t)((s_rxq.head + 1u) % JSDK_RXQ_LEN);
        if (nh == s_rxq.tail) { s_rxq.ovf++; continue; }  /* 满：丢弃最新 + 计数 */
        s_rxq.buf[s_rxq.head] = f;
        s_rxq.head = nh;
    }
}

/* ---------- HAL 回调 ---------- */
static int hal_send(void *user, const jsdk_can_frame_t *f)
{
    (void)user;
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier          = f->id;
    tx.IdType              = FDCAN_EXTENDED_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.FDFormat            = (f->flags & JSDK_FRAME_FD)  ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    tx.BitRateSwitch       = (f->flags & JSDK_FRAME_BRS) ? FDCAN_BRS_ON  : FDCAN_BRS_OFF;
    tx.DataLength          = dlc_from_len(f->len);        /* 见下方说明 */
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    return (HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan, &tx, (uint8_t *)f->data) == HAL_OK) ? 0 : -1;
}

static int hal_recv(void *user, jsdk_can_frame_t *f)
{
    (void)user;
    if (s_rxq.tail == s_rxq.head) return 0;
    *f = s_rxq.buf[s_rxq.tail];
    s_rxq.tail = (uint16_t)((s_rxq.tail + 1u) % JSDK_RXQ_LEN);
    return 1;
}

static uint32_t hal_now_ms(void *user) { (void)user; return HAL_GetTick(); }

static int hal_bus_status(void *user, uint32_t *flags)
{
    (void)user;
    uint32_t v = 0, psr = s_hfdcan->Instance->PSR;
    if (psr & 0x00000001u) v |= JSDK_HAL_BUS_ERROR_WARN;   /* EW */
    if (psr & 0x00000002u) v |= JSDK_HAL_BUS_ERROR_PASS;   /* EP */
    if (psr & 0x00000004u) v |= JSDK_HAL_BUS_OFF;          /* BO */
    *flags = v;
    return 0;
}

static const jsdk_can_hal_t HAL_OPS = {
    .user = NULL, .send = hal_send, .recv = hal_recv,
    .now_ms = hal_now_ms, .bus_status = hal_bus_status, .on_error = NULL,
};
```

**`DataLength` 的坑**：FDCAN 的 `DataLength` 是编码值（8→`FDCAN_DLC_BYTES_8`=8，12→9，16→10，20→11，24→12，32→13，48→14，64→15），**不是字节数**。

```c
static uint32_t dlc_from_len(uint8_t len)
{
    if (len <= 8)  return len;                     /* 0..8 直接相等 */
    if (len <= 12) return 9;
    if (len <= 16) return 10;
    if (len <= 20) return 11;
    if (len <= 24) return 12;
    if (len <= 32) return 13;
    if (len <= 48) return 14;
    return 15;                                     /* 64 */
}
```

### 1.5 常见移植错误

| # | 错误 | 现象 |
|---|---|---|
| 1 | `id` 里带了 EFF 标志位（如 `0x80000000`）或左移过 | 全部帧被设备忽略；抓包看 ID 是 `0x8xxxxxxx` |
| 2 | 忘记设硬件 EFF | 发成标准帧，设备 `handle_can_message` 直接 `return`（它要求 `isExt`） |
| 3 | FD `DataLength` 用了字节数而非 DLC 编码 | 12 字节的 POS 指令实际只发 8 字节 → 设备丢帧 |
| 4 | `recv` 在 ISR 里被调用 | 与 SDK 的主循环重入，队列指针撕裂 |
| 5 | RX 队列过浅（<64） | 描述符下载必失败；正常控制偶发反馈丢失 |
| 6 | `now_ms` 返回非单调值 | 看门狗与 `age_ms` 错乱 |
| 7 | `send` 内部阻塞等 TX 完成 | 描述符下载期间 TX 溢出、控制周期抖动 |
| 8 | 过滤器放行所有 ID | 无关流量占满队列，`rx_dropped` 飙升 |
| 9 | `master_id` 设为 0 | 设备完全不回复（固件 `if (master_id != 0)` 保护） |
| 10 | Classic/FD 配置与设备不符 | 完全静默：无响应、无心跳、无错误 |

---

## 2. arena 选型（重点一）

### 2.1 实测预算

以下数字由解析 `Firmware/autogen/endpoints.hpp`（v8，41029 字节，594 个端点）实测得出：

| 项 | 数值 |
|---|---|
| 端点总数 | **594** |
| 路径字符串总长（含 `'\0'`） | 20741 B |
| 路径平均长度 | 33.9 B |
| **`RETAIN_ALL` arena 需要** | **25493 B ≈ 24.9 KB** |
| 必需集 12 条 → arena | **469 B** |
| 必需集 + 8 条诊断（20 条）→ arena | **725 B** |

arena 布局（SDK 内部）：`N × 8 B 条目（路径偏移 u32 + ep_id u16 + type u8 + access u8）` + `路径字符串池`。

### 2.2 MCU 推荐配置：`RETAIN_FILTERED`

```c
/* 12 条必需集（缺任一条 → configure() 返回 JSDK_ERR_NOT_FOUND） */
static const char *const kFilter[] = {
    "axis0.motor.config.gear_ratio",
    "axis0.motor.config.torque_constant",
    "axis0.controller.config.mit_max_pos",
    "axis0.controller.config.mit_max_vel",
    "axis0.controller.config.mit_max_torque",
    "axis0.controller.config.mit_max_kp",
    "axis0.controller.config.mit_max_kd",
    "axis0.requested_state",
    "axis0.current_state",
    "axis0.config.can.node_id",
    "axis0.config.can.heartbeat_rate_ms",
    "can.config.break_timeout",

    /* 以下 8 条为诊断便利，可按需删除（省略则 arena 降到 469 B） */
    "axis0.error",
    "axis0.motor.error",
    "axis0.encoder.error",
    "axis0.controller.error",
    "axis0.motor.config.current_lim",
    "axis0.motor.config.torque_lim",
    "axis0.controller.config.control_mode",
    "can.config.baud_rate",
};

static uint8_t s_desc_arena[1024];   /* 20 条路径实测需 725 B，留 40% 余量 */
```

> **注意**：`can.config.break_timeout` 与 `can.config.baud_rate` 是**根节点**端点（不带 `axis0.` 前缀），
> 别写成 `axis0.can.config.*`。

### 2.3 尺寸计算与缩容流程

**不要靠猜**，按这个顺序做：

```c
/* 第 1 步：上电前用工具估算（离线）—— 或直接按 §2.1 的表取上界 */
/* 第 2 步：初始化时按估算分配，SDK 会在 configure() 里精确校验 */
cfg.desc.arena      = s_desc_arena;
cfg.desc.arena_size = sizeof s_desc_arena;

/* 第 3 步：configure() 成功后读实测用量 */
if (jsdk_context_configure(ctx) == JSDK_OK) {
    printf("arena_used = %u / %u\n",
           (unsigned)cfg.desc.arena_used, (unsigned)cfg.desc.arena_size);
}
/* 第 4 步：按实测值 × 1.3 收缩数组，量产固件里定死 */
```

若 `arena_size` 不足，`configure()` 返回 `JSDK_ERR_NO_MEMORY`，且 `jsdk_context_last_error()` 会给出**所需字节数**：

```
configure: descriptor arena too small: need 25493 B, have 1024 B (retain=ALL)
```

也可以在运行前调用以预估：

```c
size_t need = jsdk_desc_arena_size(&cfg.desc);
```

### 2.4 arena 放哪儿

| 位置 | 建议 | 注意 |
|---|---|---|
| 内部 SRAM | ✅ 首选（1~2 KB 而已） | 无 |
| CCM / DTCM | ✅ 可以 | 若 DMA 需要访问则不可（描述符解析由 CPU 做，无 DMA，通常安全） |
| 外部 SRAM（QSPI/FSMC） | ⚠️ 尽量避免 | 解析路径会高频读写 arena，慢速外部 SRAM 会拖长 `configure()` |
| 栈上 | ❌ 不要 | 1 KB 以上会爆小任务栈；且 `configure()` 期间需一直有效 |
| 堆 | ❌ 不要 | 与 SDK「零 malloc」定位冲突；`JSDK_ENABLE_HEAP` 仅用于桌面 |

**建议**：`static` 数组 + 编译期定死，并用 `_Static_assert`/`#if` 断言非零：

```c
#if (sizeof s_desc_arena) < 512
#  error "descriptor arena too small for the 12-endpoint required set (needs 469 B)"
#endif
```

### 2.5 `RETAIN_ALL` 还是 `RETAIN_FILTERED`

| 维度 | `RETAIN_ALL` | `RETAIN_FILTERED` |
|---|---|---|
| arena | ≈24.9 KB | ≈0.5~1 KB |
| 适用 | 桌面 / CLI / 网关 / 有 SDRAM 的 Linux 板 | **MCU 默认选择** |
| 能力 | 任意路径读写参数、`enumerate` 全表、CLI `ep-list` | 只有 filter 里的路径可读写；`enumerate` 只返回这些 |
| 缓存体积 | ≈25 KB | ≈0.5 KB |
| 后续想读新路径 | 直接用 | 必须改 filter 后重新下载（缓存也要重做） |

> ⚠ **关键取舍**：`RETAIN_FILTERED` 下，任何**不在 filter 里**的 `jsdk_joint_param_get()` 都会返回
> `JSDK_ERR_NOT_FOUND`。因此 filter 里务必包含**未来可能调试需要**的参数（诊断那 8 条就是为此）。
> 若产品需要"现场任意调参"，用 `RETAIN_ALL`（24.9 KB）或准备一条"临时改 filter 重启"的产测流程。

### 2.6 `stop_when_satisfied` 的收益

实测：**最后一个必需端点出现在描述符第 24655 字节处**（占全文 60%）。因此 `stop_when_satisfied = 1`（默认）可**省下约 40% 的下载**：

| | 全文下载 | 命中后提前终止 |
|---|---|---|
| 字节 | 41029 | ≈24800 |
| CAN FD 帧数 | 1 + 662 | 1 + 400 |
| Classic 帧数 | 1 + 6839 | 1 + 4134 |
| FD 耗时（估） | 0.1~0.2 s | 0.06~0.12 s |
| Classic 1M 耗时（估） | 1~2 s | 0.6~1.2 s |

代价：`jsdk_desc_info_t.complete = 0`，且**未保留的端点全部不可用**。首次上电做一次、结果缓存后，这张账就不重要了——所以**缓存比提前终止更值钱**。

---

## 3. 缓存到外部 Flash 的完整流程（重点二）

### 3.1 为什么必须做

不做缓存 = **每次上电**都要：

| 总线 | 下载耗时 | 风险 |
|---|---|---|
| CAN FD | 0.1~0.2 s | 上电时序被拉长；若此时关节已使能会触发看门狗 |
| Classic 1M | 1~2 s | 明显影响整机就绪时间 |
| Classic 500k / slcan | 3 s ~ 数秒 | 基本不可接受 |

做了缓存 = 上电从 Flash 读取（几毫秒）→ 跳过下载。

### 3.1b 哪条路线（先定这个，再决定 §3.2 的头部格式）

| | A：缓存解析结果 | B：缓存原始 JSON（推荐） |
|---|---|---|
| Flash 占用 | RETAIN_FILTERED ≈0.5 KB；RETAIN_ALL ≈25 KB | **恒定 ≈41 KB**（与 retain/filter 无关） |
| 失效键 | `fw_version` + `desc_crc` + **`filter_hash`** + SDK 格式版本 | **`fw_version` + `desc_crc`** |
| 改 filter / 升级 SDK | **必须重新下载**（否则 `JSDK_ERR_NOT_FOUND`） | 无需下载，重新解析即可 |
| 上电成本 | 0（已解析） | 数~数十 ms CPU（解析 41 KB） |
| 需要 `stop_when_satisfied = 0` | 否（可提前终止） | **是**（否则流被截断，缓存的 JSON 不完整） |
| 适用 | Flash 极紧张、filter 长期不变 | **Flash 宽裕、需要现场调参、多型号共用固件** |

> 多数 MCU 的 Flash 远宽裕于 RAM，**推荐 B**：它把“缓存有效性”从 4 个键减到 2 个，
> 消除了 §3.6 那个最隐蔽的坑。

### 3.2 缓存封装格式

#### 3.2.A 路线 A：缓存解析结果

`jsdk_context_desc_export()` 产出的 blob 是 **SDK 私有格式**（含 magic/format version），对应用不可见。
**应用必须再包一层自己的头**，用于 Flash 存储与失效判定：

```c
#define DESC_CACHE_MAGIC     0x4A534443u   /* "JSDC" */
#define DESC_CACHE_VERSION   1u            /* 应用侧格式版本；改动本结构体时 +1 */

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;         /* DESC_CACHE_MAGIC */
    uint16_t version;       /* DESC_CACHE_VERSION */
    uint16_t header_size;   /* sizeof(desc_cache_hdr_t)，为将来扩展留活口 */
    uint32_t payload_len;   /* 后续 payload 字节数 */
    uint32_t payload_crc32; /* payload 的 CRC32 */

    /* ---- 失效判定键（必须三项全比） ---- */
    uint32_t fw_version;    /* QUERY_DEVICE_INFO(0x46) 的 fw 字段 */
    uint16_t desc_crc;      /* 描述符的 VersionCRC */
    uint16_t filter_hash;   /* filter_paths 列表的哈希（见 §3.6 的坑） */

    uint32_t reserved;
} desc_cache_hdr_t;
#pragma pack(pop)
/* 之后紧跟 payload_len 字节的 jsdk_context_desc_export() 输出 */
```

#### 3.2.B 路线 B：缓存原始 JSON（推荐）

结构更简单——**只有两个失效键**，因为原始 JSON 与 retain / filter / SDK 版本全都无关：

```c
#define RAW_CACHE_MAGIC    0x4A534452u   /* "JSDR" */
#define RAW_CACHE_VERSION  1u

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;         /* RAW_CACHE_MAGIC */
    uint16_t version;       /* RAW_CACHE_VERSION */
    uint16_t header_size;   /* sizeof(raw_cache_hdr_t) */
    uint32_t payload_len;   /* 应等于 jsdk_desc_info_t.total_len */
    uint32_t payload_crc32;

    /* ---- 失效键（只两项）---- */
    uint32_t fw_version;    /* QUERY_DEVICE_INFO(0x46) */
    uint16_t desc_crc;      /* desc_info.crc */

    uint16_t reserved;
} raw_cache_hdr_t;
#pragma pack(pop)
/* 之后紧跟 payload_len 字节的**设备原始 JSON** */
```

写入方式：下载过程中由 sink **边收边落 Flash**（不需要 41 KB RAM）：

```c
#define OFF_HDR     0u                       /* 槽头部偏移 */
#define OFF_PAYLOAD sizeof(raw_cache_hdr_t)  /* payload 起点 */

static uint32_t s_raw_crc;    /* 增量 CRC32 */
static uint32_t s_raw_len;

static int raw_sink(jsdk_context_t *ctx, const void *data, size_t len,
                    uint32_t offset, void *user)
{
    (void)ctx; (void)user;
    if (flash_write(CUR_SLOT + OFF_PAYLOAD + offset, data, len) != 0) return -1;
    s_raw_crc = crc32_update(s_raw_crc, data, len);
    s_raw_len += (uint32_t)len;
    return 0;
}

/* ---- 下载前 ---- */
cfg.desc.stop_when_satisfied = 0;        /* ⚠ 必须关掉提前终止 */
jsdk_context_set_desc_raw_sink(ctx, raw_sink, NULL);
s_raw_crc = 0xFFFFFFFFu; s_raw_len = 0;
flash_erase_slot(CUR_SLOT);

/* ---- 下载 ---- */
if (jsdk_context_desc_fetch(ctx) == JSDK_OK) {
    jsdk_desc_info_t di;
    jsdk_context_get_desc_info(ctx, &di);

    /* 三重后置校验：少一项都可能存入不完整数据 */
    if (di.complete && !di.raw_sink_failed && s_raw_len == di.total_len) {
        raw_cache_hdr_t h = {
            .magic = RAW_CACHE_MAGIC, .version = RAW_CACHE_VERSION,
            .header_size = sizeof h,
            .payload_len = s_raw_len,
            .payload_crc32 = s_raw_crc ^ 0xFFFFFFFFu,
            .fw_version = di.fw_version, .desc_crc = di.crc,
        };
        flash_write(CUR_SLOT + OFF_HDR, &h, sizeof h);  /* 头部最后写：CRC 自证 */
        flash_cache_commit();                            /* 擦除旧槽 */
    } else {
        flash_invalidate_slot(CUR_SLOT);
    }
}
jsdk_context_set_desc_raw_sink(ctx, NULL, NULL);
```

> ⚠ **`stop_when_satisfied` 必须为 0**。若为 1，解析会在必需端点全部命中后（实测约第 24655 字节）
> 提前终止 → 流被截断 → 缓存里只有前半段 JSON → 下次 `desc_import_raw()` 必然 `JSDK_ERR_PARSE`。
> 上面三个后置校验（`complete` / `raw_sink_failed` / `s_raw_len == total_len`）就是为防这个。

> ℹ sink 回调在**配置阶段**被调用，**允许阻塞**（可以等 Flash 编程完成）。
> 但**不要在 sink 里调用其他 SDK API**（避免重入），只做 Flash 写入与 CRC 累计。

启动时恢复（失败就当作“无缓存”，`configure()` 会自动回落到下载）：

```c
const raw_cache_hdr_t *h = (const raw_cache_hdr_t *)slot_ptr;
const void *payload = (const uint8_t *)slot_ptr + sizeof *h;

jsdk_desc_hint_t hint = { .crc = h->desc_crc, .fw_version = h->fw_version };
if (crc32(payload, h->payload_len) != h->payload_crc32 ||
    jsdk_context_desc_import_raw(ctx, payload, h->payload_len, &hint) != JSDK_OK) {
    flash_invalidate_slot(CUR_SLOT);      /* 不导入 → configure() 会去下载 */
}
```

### 3.3 Flash 布局

```
┌─────────────────────────────────────────────────────────────┐
│ Bootloader / App                                            │
├─────────────────────────────────────────────────────────────┤
│ Slot A:  desc_cache_hdr_t + payload     ← 扇区对齐           │
│   ...（按 max payload 预留，RETAIN_ALL 约 26 KB；FILTERED 1 KB）│
├─────────────────────────────────────────────────────────────┤
│ Slot B:  desc_cache_hdr_t + payload     ← 同上，双槽轮换     │
└─────────────────────────────────────────────────────────────┘
```

**要求**：

| 要求 | 说明 |
|---|---|
| 扇区对齐 | 每个 Slot 起始地址必须是 Flash **擦除扇区大小**的整数倍（STM32F4 16/64/128 KB，G4/H7 8 KB，STM32F1 1/2 KB）。**这是最容易出错的一条** |
| 双槽 | A/B 轮换写入。只在写新槽成功并校验后才擦旧槽 → 任何时刻至少有一份可用 |
| 大小 | 每槽 = `sizeof(desc_cache_hdr_t) + max_payload`。按 `jsdk_desc_export_max_size()` 上取整到扇区 |
| 磨损 | 描述符只在固件升级后变化 → 写入次数极少，寿命无虞 |

### 3.4 掉电安全的写入序列

**顺序不可颠倒**（关键：先写数据、再校验、最后才擦旧数据）：

```
1. 目标槽 = (当前有效槽 == A) ? B : A
2. 擦除目标槽所在扇区
3. 写入 desc_cache_hdr_t（先写头，payload_crc32 先填 0 或最后回填均可，见下）
4. 写入 payload
5. **回读并逐字节校验**（或算 CRC32 对比）
6. 校验通过 → 在头部回填/更新一个 valid 标记（或依赖 magic + CRC 已能自证）
7. 校验失败 → 擦除目标槽，返回失败（保留旧槽）
8. 擦除旧槽
```

**更稳妥的做法（推荐）**：不要单独维护 valid 标记，而是**靠 `magic + payload_crc32` 自证**——
只有 `magic` 正确**且** CRC 匹配才算有效。这样第 6 步可省，且掉电在任意位置都只会得到"无效槽"，不会得到"看似有效的坏数据"。

```c
static bool cache_slot_valid(const desc_cache_hdr_t *h, const void *payload)
{
    if (h->magic != DESC_CACHE_MAGIC)               return false;
    if (h->version != DESC_CACHE_VERSION)           return false;
    if (h->header_size != sizeof(desc_cache_hdr_t)) return false;
    if (h->payload_len == 0 || h->payload_len > DESC_CACHE_MAX_PAYLOAD) return false;
    return crc32(payload, h->payload_len) == h->payload_crc32;
}
```

### 3.5 启动流程（状态机）

```
BOOT
 │
 ├─ 1. 扫描 A/B 两槽，选出 magic + CRC 均有效的槽
 │       └─ 两份都有效 → 取「fw_version/desc_crc 匹配度更高」的那份；否则取较新的一份
 │
 ├─ 2. 若存在有效槽：
 │       路线 B: jsdk_context_desc_import_raw(ctx, payload, len, &hint)
 │       路线 A: jsdk_context_desc_import(ctx, payload, len)
 │         ├─ 成功 → 描述符已就绪，configure() 不再下载（省 0.1~2 s）
 │         └─ 失败（JSDK_ERR_PARSE / 格式不匹配）→ 标记该槽作废，落到第 3 步
 │
 ├─ 3. 无可用缓存 / import 失败：
 │       cfg.desc.mode = JSDK_DESC_DYNAMIC
 │       jsdk_context_desc_fetch(ctx)      ← 必须有额外 RX 队列深度（§1.3）
 │         ├─ 成功 → 写入 Flash 缓存（§3.4）→ 进入 NORMAL
 │         └─ 失败 → 重试（建议 ≤3 次，间隔 100 ms）
 │                  └─ 仍失败 → 进入 DEGRADED（见 §3.7）
 │
 └─ 4. NORMAL: 校验缓存键与设备实际值
        QUERY_DEVICE_INFO(0x46) → fw_version
        desc_info.crc
        与缓存头的 fw_version / desc_crc 比对
          └─ 不一致 → 缓存的量程可能已变 → 重新走第 3 步（并可保留旧缓存做兜底）
```

### 3.6 失效条件（路线 A 需三+1 个键；路线 B 只需两个）

| 键 | 变化原因 | 不检测的后果 |
|---|---|---|
| `fw_version` | 驱动器固件升级/降级 | 端点 ID 全变（实测漂移率 86%）→ 读到的值张冠李戴 |
| `desc_crc` | 固件的端点集/顺序变化 | 同上 |
| **`filter_hash`** | **应用侧改了 `filter_paths` 列表** | ⚠️ **最隐蔽的坑**：缓存里只有旧 filter 的端点，新代码去读新路径会 `JSDK_ERR_NOT_FOUND`，但缓存"看起来完全有效" |
| `max_endpoints` / `max_path_len` | 应用侧改了这两个解析上限 | 上限**会改变哪些端点被接受**（路径超限或条目超限的端点会被丢弃）→ 与 `filter_hash` 同类的坑 |
| **SDK 线格式版本** | SDK 升级后内部表示变化 | 旧缓存按新布局解释 → 内存错乱。已含在 `cb_desc_filter_hash()` 里（`CB_DESC_CACHE_VERSION`），格式一改就必须 +1 |

> ⚠ **上限必须按“生效值”入键**：`max_endpoints` / `max_path_len` 为 0 时表示内置默认值
> （2048 / 128）。若把原始 0 写进键，则“用默认值导出、用**显式**默认值导入”会被误判为
> 配置不符（`JSDK_ERR_BAD_STATE`），反之亦然。`cb_desc_eff_max_endpoints()` /
> `cb_desc_eff_max_path_len()` 负责归一化，哈希、导出头、导入比对三处**必须**统一使用。

```c
static uint16_t filter_hash(const char *const *paths, unsigned n)
{
    uint32_t h = 2166136261u;                  /* FNV-1a */
    for (unsigned i = 0; i < n; ++i)
        for (const char *p = paths[i]; *p; ++p) { h ^= (uint8_t)*p; h *= 16777619u; }
    return (uint16_t)(h ^ (h >> 16));
}
```

> 还要注意：**SDK 的 export 格式版本**也可能变（`jsdk_abi_version()`）。`desc_import()` 会在格式不匹配时
> 返回错误，应用应当把该槽作废并重新下载，而不是反复重试。
>
> ✅ **路线 B（缓存原始 JSON）不受上述两个坑影响**：原始 JSON 与 `retain`/`filter_paths`/SDK 格式
> 全都无关，失效键只剩 `fw_version` + `desc_crc`。这是路线 B 除了“改 filter 不用重下”之外的第二个优点。

### 3.7 降级路径（下载失败时怎么办）

| 方案 | 说明 | 建议 |
|---|---|---|
| 重试 | `desc_fetch()` 重试 ≤3 次，间隔 100 ms | ✅ 必做（总线噪声/上电抖动很常见） |
| 用旧缓存兜底 | 即使键不匹配也先用旧缓存，同时后台重试下载 | ⚠️ 有风险（量程可能已变），**必须**同时置 `unit_scale.valid = 0` 并拒绝物理量 API |
| 只做诊断（不控制） | 允许 `scan`/`health`/心跳监控，但禁止 `activate()` | ✅ 推荐的安全降级：至少能让上位机看到设备在不在 |
| 硬失败 | 直接报错停机 | ⚠️ 仅在安全关键场景可接受 |

**推荐组合**：`重试 3 次 → 仍失败则进入"仅诊断模式"`，并在 HMI 上给出明确提示（"驱动器描述符读取失败，请检查 CAN 链路/FD 配置"）。

---

## 4. 端到端示例（MCU）

```c
/* ============================ 应用侧资源 ============================ */
static jsdk_context_storage_t s_ctx_store;           /* 上下文（约 2 KB） */
static uint8_t                s_desc_arena[1024];    /* 见 §2.2 */

static const char *const kFilter[] = { /* §2.2 的 12~20 条 */ };
#define FILTER_N (sizeof kFilter / sizeof kFilter[0])

static uint8_t s_flash_scratch[DESC_CACHE_MAX_PAYLOAD];  /* 缓存读写暂存 */

/* ============================ 初始化 ============================ */
jsdk_status_t app_joint_init(void)
{
    jsdk_context_config_t cfg;
    jsdk_context_config_default(&cfg);

    cfg.hal             = HAL_OPS;        /* §1.4 */
    cfg.master_id       = 1;              /* 禁止 0 */
    cfg.is_fd           = 1;              /* 必须与设备 can.config.baud_rate 匹配 */
    cfg.period_ns       = 1000000u;       /* 1 ms */
    cfg.auto_keepalive  = 1;
    cfg.rx_burst_limit  = 32u;            /* 正常控制用；描述符下载不受此限 */

    cfg.desc.retain             = JSDK_DESC_RETAIN_FILTERED;
    cfg.desc.filter_paths       = kFilter;
    cfg.desc.filter_count       = (unsigned)FILTER_N;
    cfg.desc.arena              = s_desc_arena;
    cfg.desc.arena_size         = sizeof s_desc_arena;
    cfg.desc.share_by_crc       = 1;
    cfg.desc.stop_when_satisfied= 1;
    cfg.desc.timeout_ms         = 8000;   /* Classic 下留足余量 */

    if (jsdk_context_init((jsdk_context_t *)&s_ctx_store, &cfg) != JSDK_OK)
        return JSDK_ERR_BAD_STATE;

    jsdk_joint_config_t jc = {0};
    jc.node_id      = 1;
    jc.axis         = 0;
    jc.initial_mode = JSDK_MODE_MIT;
    /* 其余字段置 0 = 从设备描述符读取 */
    if (jsdk_context_add_joint((jsdk_context_t *)&s_ctx_store, &jc, &g_joint) != JSDK_OK)
        return JSDK_ERR_BAD_STATE;

    /* ---- 描述符：先试缓存 ---- */
    size_t blen = 0;
    if (flash_cache_load(s_flash_scratch, sizeof s_flash_scratch, &blen) == 0) {
        cfg.desc.mode = JSDK_DESC_FROM_CACHE;
        if (jsdk_context_desc_import((jsdk_context_t *)&s_ctx_store,
                                     s_flash_scratch, blen) == JSDK_OK) {
            goto have_desc;
        }
        flash_cache_invalidate();          /* 格式不匹配 → 作废，走下载 */
    }

    /* ---- 下载（仅首次上电/固件升级后）---- */
    cfg.desc.mode = JSDK_DESC_DYNAMIC;
    for (int retry = 0; retry < 3; ++retry) {
        if (jsdk_context_desc_fetch((jsdk_context_t *)&s_ctx_store) == JSDK_OK) break;
        HAL_Delay(100);
        if (retry == 2) { app_enter_diag_only_mode(); return JSDK_ERR_TIMEOUT; }
    }
    flash_cache_store_from_ctx((jsdk_context_t *)&s_ctx_store);   /* §3.4 */

have_desc:
    /* ---- configure 完成剩余工作并校验量程 ---- */
    if (jsdk_context_configure((jsdk_context_t *)&s_ctx_store) != JSDK_OK)
        return JSDK_ERR_PROTOCOL;          /* 量程不合理，拒绝进入物理量 API */

    jsdk_joint_config_snapshot_t snap;
    jsdk_joint_read_config_snapshot(g_joint, &snap);
    log("gear=%.3f pmax=%.2f tmax=%.2f tc=%.5f",
        snap.gear_ratio, snap.mit_max_pos, snap.mit_max_torque, snap.torque_constant);
    return JSDK_OK;
}

/* ============================ 1 ms 控制循环 ============================ */
void app_joint_tick_1ms(void)
{
    jsdk_context_poll((jsdk_context_t *)&s_ctx_store, HAL_GetTick() * 1000000ull);

    jsdk_joint_feedback_t fb;
    jsdk_joint_get_feedback(g_joint, &fb);
    if (!fb.valid || fb.age_ms > 20) return;      /* 反馈过期 → 本周期不发指令 */

    /* 你的控制律 …… */
    jsdk_joint_set_mit(g_joint, /*pos*/0.0, /*vel*/0.0, /*kp*/20.0, /*kd*/0.5, /*tau*/0.0);
}
```

---

## 5. 启动时序与时间预算

| 阶段 | 有缓存 | 无缓存（FD） | 无缓存（Classic 1M） |
|---|---|---|---|
| HAL 初始化 | — | — | — |
| 读 Flash 缓存 | 1~5 ms | — | — |
| `desc_import()` | <1 ms | — | — |
| `desc_fetch()` | — | 60~200 ms | 0.6~2 s |
| 写 Flash 缓存 | — | 5~30 ms | — |
| `configure()` 其他（握手/参数读回/`QUERY_DEVICE_INFO`） | 5~20 ms | 5~20 ms | 20~80 ms |
| **合计** | **≈10~30 ms** | **≈0.1~0.25 s** | **≈0.7~2.2 s** |

> 上表假设 `RETAIN_FILTERED` + `stop_when_satisfied = 1`（≈400 FD 帧 / 4134 Classic 帧）。

**与看门狗的交互**：`desc_fetch()` 期间**不允许有使能的关节**（否则 JSON 帧挤掉控制帧 → `break_timeout` 触发 `disarm`）。
因此启动流程必须是：**先取描述符 → 再 `activate()`**，绝不可颠倒。

---

## 6. 常见故障排查

| 现象 | 排查方向 |
|---|---|
| `configure()` 返回 `JSDK_ERR_TIMEOUT` | ① `is_fd` 与设备 `baud_rate` 是否匹配（不匹配=静默无响应）；② RX 队列是否 ≥128/256；③ `master_id` 是否非 0；④ 终端电阻 |
| `JSDK_ERR_NO_MEMORY` | `arena_size` 不足。看 `jsdk_context_last_error()` 里的所需字节数；确认 `retain` 是否被设成了 `ALL` |
| `JSDK_ERR_PARSE` | RX 丢帧导致 JSON 断裂。加大队列 / 提高 `desc_poll()` 调用频率 / 降低总线负载后重试 |
| `JSDK_ERR_UNSUPPORTED` | `total_len > 65535`（协议 `chunkOffset` 是 u16 LE）。需固件侧扩展协议 |
| `JSDK_ERR_NOT_FOUND` | 必需端点不在 filter 里（`RETAIN_FILTERED`）；或读的路径根本没保留。检查 `filter_paths` |
| `JSDK_ERR_BAD_STATE` | 在关节使能状态下调了 `desc_fetch()` |
| 缓存"有效"但读新路径失败 | `filter_hash` 没纳入缓存键（§3.6 的坑） |
| 缓存有效但量程不对 | `fw_version` / `desc_crc` 没纳入缓存键；或用了旧缓存兜底却未置 `unit_scale.valid = 0` |
| 上电偶发失败、重试就好 | 正常现象（上电抖动）。必须实现重试；建议同时把每次失败写入日志区便于统计 |
| 下载期间关节抖动/掉使能 | 违反了"禁止在使能状态下下载"；或 `desc_fetch()` 期间仍在跑控制循环 |

---

## 7. 两条缓存路线怎么选

> 对应 API（**均已提供**）：
> 路线 A = `jsdk_context_desc_export()` / `jsdk_context_desc_import()`；
> 路线 B = `jsdk_context_set_desc_raw_sink()` + `jsdk_context_desc_import_raw()`。

**路线 A：缓存解析结果**

`jsdk_context_desc_export()` 导出的是 SDK 私有紧凑格式，内容等于「按当前 `retain` / `filter_paths` 裁剪后的端点表」。
因此它被四项绑定：`retain`、`filter_paths`、SDK 内部格式版本，再加 `fw_version` + `desc_crc`。

**路线 B：缓存原始 JSON**

把设备原始 JSON（41029 B）落到 Flash，启动时用 `jsdk_context_desc_import_raw()` 按当前配置重新解析。
因为原始 JSON 与 `retain` / `filter` / SDK 版本全部无关，失效键只剩 `fw_version` + `desc_crc`。

**对比与选择**

| | A：缓存解析结果 | B：缓存原始 JSON（推荐） |
|---|---|---|
| Flash 占用（FILTERED/12 条） | ≈0.5 KB | ≈41 KB |
| Flash 占用（RETAIN_ALL） | ≈25 KB | ≈41 KB |
| 缓存键 | `fw_version` + `desc_crc` + **`filter_hash`** + SDK 格式版本 | `fw_version` + `desc_crc` |
| 改 filter / 升级 SDK | **必须重新下载** | 无需下载，重新解析即可 |
| 上电解析耗时 | 0（已解析） | 数~数十 ms（CPU 解析 41 KB） |
| 需要 `stop_when_satisfied = 0` | 否 | **是** |
| 能否与另一路线同时产出 | 可以（sink 与 export 可并存） | 可以 |
| 适用 | Flash 极紧张、filter 长期不变 | **Flash 宽裕、需要现场调参、多型号共用固件** |

**建议**：

- MCU Flash ≥ 256 KB → 直接选 **B**。失效键从 4 个减到 2 个，消灭了 §3.6 那个“缓存看似有效却读不到新路径”的隐蔽坑。
- Flash 真的紧张（如 64 KB 芯片）→ 选 **A**，但必须在缓存头里放 `filter_hash`，
  并且**把 filter 列表当作 ABI**：一改就必须走一次重新下载。
- 两者可以**同时做**：下载时挂 sink 写原始 JSON，同时 `desc_export()` 再存一份紧凑格式，互为兜底。
- **离线预热**：JSON 由固件版本唯一决定，因此可以先用产测工装/桌面 CLI 从一台同版本设备下载，
  把原始 JSON 随固件一起烧进 Flash。这样产线上电根本不需要下载（也不占用产线 CAN 带宽）。

---

## 7.5 桌面平台：直接用内置 HAL 后端（`jsdk_hal_builtin.h`）

MCU 客户**看不到**这一节：他们按 §1 自己实现 `jsdk_can_hal_t`，不需要
`jsdk_hal_builtin.h`（它不随 SDK 源码一起被 MCU 工程包含）。这里写给
**PC 上位机 / 现场工具 / CI** 的使用者。

### 7.5.1 后端与编译开关

| 后端 | CMake 选项 | 默认 | 依赖 | 备注 |
|---|---|---|---|---|
| `socketcan` | `JSDK_BUILD_HAL_SOCKETCAN` | Linux ON / 其它 OFF | 无（内核接口） | CAN FD + BRS 全支持；**位定时由内核负责** |
| `pcan` | `JSDK_BUILD_HAL_PCAN` | Windows / macOS ON | **运行期**加载 `PCANBasic` | 不需要链接 `.lib`；没装驱动只影响 `--if pcan` |
| `slcan` | `JSDK_BUILD_HAL_SLCAN` | ON | 串口 | **支持 CAN FD**（CANable 2.0 的 `b/B/d/D` 扩展）；实测约 100~500 fps |
| `virtual` | `JSDK_BUILD_HAL_VIRTUAL` | ON | 无 | 自带驱动器模型 + **自带描述符**，CI/离线演示用 |

只要启用**任意**一个后端，`hal_common.c` 就会被编进去（它提供
`jsdk_hal_close()` 与"未编译后端"的明确报错）。三者全关时 `jsdk_hal_close()`
不存在——这是有意的：桌面后端是可选件，核心库不依赖它们。

### 7.5.2 三个后端各自必须知道的坑

**SocketCAN**

- 链路**必须先用 `ip` 配好**，SDK 不改位定时：
  `ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on`
- 如果要求 FD 而链路是 Classic（MTU 16），`jsdk_hal_socketcan_open()` 直接返回
  `JSDK_ERR_INVALID_ARG`，**不会**把 FD 帧丢进 Classic 链路（那会得到一堆
  `EINVAL`，客户看不出原因）。反向（链路 FD、要求 Classic）允许。
- 总线错误（bus-off / error-passive / RX 溢出）通过**锁存位**经
  `hal.bus_status` 上报，SDK 的 `link_errors` 由 `recv() < 0` 驱动。两层信息都看。

**PCAN**

- 位定时要**算好传进去**（与 SocketCAN 相反）。本实现内置常用组合表
  （1M/5M、1M/2M、500k/5M、500k/2M，按 80 MHz 晶振）；表外组合返回
  `JSDK_ERR_UNSUPPORTED` —— **宁可拒绝也不猜时钟参数**，猜错的后果是现场莫名其妙的
  error frame。
- `PCANBasic.dll` / `libpcanbasic.so` 是**运行期**加载的：找不到返回
  `JSDK_ERR_NOT_FOUND`，提示装哪个包。
- 32 位 Windows 上 PCAN-Basic 是 `__stdcall`，函数指针已按平台加调用约定宏。

**slcan**

- 串口波特率（`--baud`）与 CAN 段波特率是**两个量**，别混：
  适配器通常在 115200，而 CAN 段按 1 Mbps 跑。
- **支持 CAN FD**（CANable 2.0 固件对 Lawicel 的扩展）：帧前缀 `d/D`（BRS=0）、
  `b/B`（BRS=1），DLC 位是 **FD 长度码**（`0..8` 直映，`9`→12 … `F`→64）。
  ⚠ **`b/B` 是“带 BRS”**，字母与 BRS 的对应关系是反直觉的。
  ⚠ 长度 9/10/11 在 FD 里**无法表示**：SDK 拒绝而不是静默按 12 字节发。
  ⚠ FD 载荷必须与 DLC 码严格对应（不补齐 = 语法错误，不是“解出一条短帧”）。
- **数据段速率是适配器私有表**：传 `--data-bitrate 2000000|5000000` 时 SDK 会主动发
  `Y2`/`Y5`；其它速率返回 `UNSUPPORTED`（**不猜** `Y<n>` 索引，猜错就是把数据段速率
  设成别的值）。要别的速率请用厂家工具先配好，再传 `--data-bitrate 0`。
  传 **0 = 完全不碰适配器配置**（与 SocketCAN 的“内核管链路、SDK 不插手”同一姿态）。
- 打开序列是 `C\r`（复位到关闭态）→ [`Y<n>\r`] → **`O\r`（打开通道）**。
  `O` 不可省：Lawicel 语义下通道上电是关闭的，只发 `C` 会让适配器一直闭着
  （现场表现为“一条帧都收不到”）。
- DLC 省略时按 **0 字节**解释（Lawicel 规范）。要发 8 字节必须写 `8` 并跟上
  16 个十六进制字符。
- 适配器的状态行（`V1013`、`F00`、`s8`…）不是帧也不是错误：解析器返回
  `MALFORMED` 让调用方**丢掉整行**，而不是跳一个字节（后者会把后续行切错位）。
- 收到 ``(BELL) 会计数并置 `JSDK_HAL_BUS_ERROR_WARN`：NACK 通常意味着
  线接反 / 无终端电阻 / 波特率不符。
- `jsdk_hal_slcan_fd_frames()`：`rx_fd == 0` 而 `tx_fd > 0` 通常说明**对端在按
  Classic 回**或适配器没进 FD 模式 —— 比“反馈解析不出来”早一步指出现场问题。
- **打开失败要说清楚原因**：失败时没有句柄可挂诊断信息，所以原因记在
  `jsdk_hal_slcan_last_open_error()`（静态缓冲，每次 `open()` 重置）。`jsdk-cli`
  失败时会多打一行 `原因：…`，例如 Linux 上忘了 `sudo`：

  ```console
  $ ./build/jsdk-cli --if slcan --channel /dev/ttyACM0 scan
  jsdk-cli: 打开 slcan(/dev/ttyACM0) 失败：invalid-argument
    原因：open(/dev/ttyACM0, O_RDWR) 失败：Permission denied (errno=13)，
          权限不足：把当前用户加入 dialout 组（sudo usermod -aG dialout $USER，
          重新登录生效），或本次用 sudo 运行
  ```

  常见 errno 都已配好“下一步该做什么”：`EACCES/EPERM`（权限）、`ENOENT`（节点不在，
  插拔后可能变成 `ttyACM1`）、`EBUSY`（被 `slcand`/`candump` 占着）、`ENOTTY`
  （不是串口）、`ENXIO/EIO`（已断开）。

#### Linux 上从 `slcand` 换到 SDK 直连 slcan

`slcand` 与 SDK 是**两种互斥的用法**：前者把适配器接管成 SocketCAN 的 `can0`
（内核拥有它），后者由 SDK 直接读那个串口。切换顺序不能少：

```bash
sudo ip link set down can0        # 1) 先停接口
sudo pkill slcand                 # 2) 再放掉串口（不放掉会 EBUSY）
ls -l /dev/ttyACM0                # 3) 看一眼权限：通常是 root:dialout 660
./build/jsdk-cli --if slcan --channel /dev/ttyACM0 scan
```

要点：

- **权限**：`/dev/ttyACM0` 一般属于 `dialout` 组。把自己加进去
  （`sudo usermod -aG dialout $USER`，**重新登录**生效）或本次用 `sudo` 跑。
  用 `slcand` 时是 `sudo` 开的，很容易忘记 SDK 这条也需要。
- **CAN 段波特率由适配器自己持有**，SDK 打开时只发 `C`（关闭态）→ [`Y<n>`] → `O`
  （打开通道），**不设置** CAN 段速率。之前用 `slcand -s8` 配过 1 Mbps 就会留在
  适配器里；要改成别的速率请用厂家工具（或再跑一次 `slcand -s<n>`）配好。
- `--baud` 是**串口**速率（适配器通常 115200，CDC 下甚至被忽略），**不是** CAN 速率；
  `--bitrate` 对 slcan 无意义（CLI 会提示）。
- 要跑 **CAN FD** 再加 `--data-bitrate 5000000`（或 `0` = 不碰适配器配置）。
- ⚠⚠ **Classic 还是 FD 由设备自己决定，协议没有运行时协商**（`can.config.baud_rate`）。
  现场真实例子：一套 **CyberBeast USB2CAN（`/dev/ttyACM0`）+ 1 Mbps Classic** 的设备，
  主机默认按 **FD** 发帧 ⇒ 设备“根本不收”，`desc-info` 报
  `0/0 bytes, 198 frames received`（心跳收得到、请求没人应，看着特像线缆问题）。
  现在 SDK 会在收到本关节第一帧时**自动对齐**过去，并打一行提示
  （`jsdk_context_framing_learned()` 返回 0 未学 / 1 改学 Classic / 2 改学 FD /
  3 一致 / **4 与显式配置冲突**）。注意那是“猜错补救”，不是“你说了不算”：
  `cfg.is_fd_explicit = 1`（CLI 的 `--classic` / `--data-bitrate`、Python 传 `is_fd=`）时
  自动对齐**不会**动你写的值，冲突只报 4 —— 否则调用者看到的 cfg 与实际发出的帧不一致，
  而且 8 字节参数的分块读（FD 一次 ≤8 B / Classic 一次 ≤4 B）就是靠 `is_fd` 的。
  两个方向的后果**不对称**：对端 Classic + 我们发 FD ⇒ 设备**收不到**（必失败）；
  对端 FD + 我们发 Classic ⇒ FD 控制器收得下经典帧（命令能跑，但退化成两次请求）。
- 描述符下载在 Classic 下是 **1+6906 帧**（FD 是 1+662，10.4 倍）。`--timeout` 是
  “**多久没有新字节**”的静默预算（不是总时长），默认 5 s/CLI 3 s 都够；实现见
  `jsdk_desc.c` 的 `desc_fetch_from()`（外加 120 s 总时长兜底）。
- 用 `socketcan` 后端（即保留 `slcand`、`can0` 那套）也可以：`--if socketcan
  --channel can0` —— 那是“内核管链路”的路线，与本文这条二选一。

### 7.5.3 真链路冒烟：自动能跑的部分 + 必须人工的部分

#### A. 已自动化（不需要任何硬件；Linux/WSL 三条命令）

```bash
./tools/wsl_build.sh all        # Linux/gcc9 + -Werror：0 告警；ctest 10/10
./tools/wsl_build.sh asan       # ASAN + UBSan：ctest 10/10
./tools/wsl_build.sh pcan       # 强制编 PCAN 后端（Linux 下默认 OFF，不编就“从未被编译器看过”）
sudo ./tools/live_can_smoke.sh  # 真链路：modprobe + vcan0(MTU72)/vcan1(MTU16) + 冒烟 + 清理
./tools/live_can_mutation_test.sh  # 证明上面那个冒烟**真的有牙齿**（把 4 个修复逐个改回缺陷）
```

⚠ **自动化到不了的地方（别把"冒烟绿了"当成全覆盖）**：`vcan` 只把 skb 回环，**不做收发器
层的校验** —— 所以“FD 帧没置 `CANFD_FDF`”这种错误在 vcan 上照样能发能收。
`live_can_mutation_test.sh` 实测：4 个修复里 **3 个能被自动化检出**（不设
`CAN_RAW_FD_FRAMES`、`bind` 读 union、`LOOPBACK=0`），**`CANFD_FDF` 那一处检不出来**
→ 只能落到下面的人工清单第 12 项。

`live_can_smoke.sh` 自己会做完 `modprobe can can-raw vcan`、建接口、构建 `build-live`、
跑 `tests/test_socketcan_live.c`、最后清理。它验的是**内核真链路**（不是桩、不是 mock）：

| # | 场景 | 断言 |
|---|---|---|
| 1 | `open` vcan0（FD，1M/5M）+ `bus_status` | 打开成功、`JSDK_HAL_BUS_OK` 置位、`now_ms` 合理 |
| 2 | Classic 8 B：SDK → 对端 | 独立对端 socket **确实收到**（这条曾因 `CAN_RAW_LOOPBACK=0` 而失败） |
| 3 | 29-bit 扩展帧：对端 → SDK | `id` 与 `JSDK_FRAME_EXT` 正确 |
| 4 | **FD 64 B 出 / 32 B 入，带 BRS** | 双向都过、BRS 位正确（这条曾因缺 `CANFD_FDF` + `CAN_RAW_FD_FRAMES` 而失败） |
| 5 | FD 帧落到 Classic socket 上 | 被内核挡掉，且**链路不被污染**（随后的 Classic 帧仍正常） |
| 6 | FD 请求打到 Classic 链路（vcan1） | `JSDK_ERR_INVALID_ARG`，**不会**把 FD 帧丢进 Classic 链路 |

⚠ 两个必须知道的坑（`live_can_smoke.sh` 已经替你处理好，改脚本时别踩）：

- **模块 + 建接口 + 测试必须在同一次 WSL 调用里**。WSL2 会在两次 `wsl.exe` 调用之间重启 VM，
  `modprobe` 的状态会丢，表现是"上一条命令里建好的 `vcan0` 不见了"。
- **`modprobe vcan` 不够**：`socket(PF_CAN, SOCK_RAW, CAN_RAW)` 还需要 `can.ko` 与 `can-raw.ko`，
  否则报 `EAFNOSUPPORT`（"Address family not supported by protocol"）—— 看着像内核不支持 CAN，
  其实只是少 load 两个模块。

#### B. 必须人工（真硬件；桩与 `vcan` 都替代不了）

`vcan` 是纯软件回环，所以下面这些**只能在真适配器 / 真驱动器上**验：

| # | 场景 | 预期 | 为什么自动化不了 |
|---|---|---|---|
| 1 | `jsdk-cli --if socketcan --channel can0 scan` | 列出设备节点 ID | 需要真设备应答 |
| 2 | 同一命令换 `--if pcan --channel PCAN_USBBUS1`（Windows） | 同上；未装驱动时报 `JSDK_ERR_NOT_FOUND` 且提示装驱动 | 需要 PCAN 硬件 |
| 3 | `--if slcan --channel /dev/ttyACM0 --baud 115200 scan` | 同上；`--bitrate` 被忽略且打印提示 | 需要 CANable 之类的适配器 |
| 3b | 同一适配器跑 **FD**：`--if slcan --data-bitrate 5000000 scan` | 与 3 同样列出节点；但 `--data-bitrate` 为**表外值**（如 3000000）时必须报 `UNSUPPORTED` 并提示支持的速率 | `Y<n>` 是**适配器固件的私有表**，只有真适配器知道 |
| 3c | 适配器拔掉（端口不存在）时跑 `--data-bitrate 0` | 报 `INVALID_ARG`（**不是** `UNSUPPORTED`）—— 证明校验顺序是"先参数、后串口" | 需要真实的串口故障 |
| 4 | `health` 在三种后端上 | `link_up=yes`、`age_ms` 稳定、`vbus_V` 与万用表一致（±2%） | 需要真电压基准 |
| 5 | 拔掉 CAN 线 3 s 再插回 | 期间 `link_errors` 增长、`link_up=0`；插回后自动恢复，无需重启 | `vcan` 不会掉线 |
| 6 | 设备断电再上电 | `age_ms` 增长；恢复后 `health` 重新在线 | 需要真设备 |
| 7 | SocketCAN 链路改成 Classic（`bitrate 1000000` 无 `fd on`）后跑 `--classic` | FD 请求被拒（`INVALID_ARG`）并说明怎么改链路 | 需要真的改 CAN 控制器位定时 |
| 8 | PCAN 用表外组合（如 250k/1M） | 明确 `UNSUPPORTED`（不是静默用错时钟） | 需要真 PCAN 硬件 |
| 9 | **位定时与终端电阻** | 两端合计 60 Ω；示波器上看不到明显反射；`candump -e can0` 无 error frame | 纯硬件问题 |
| 10 | **error-frame / bus-off** | 拔线或做短路 → `bus_status` 报 `JSDK_HAL_BUS_ERROR_WARN`；接回后清除 | **`vcan` 不产生 error frame**（内核侧没有收发器） |
| 11 | **真驱动器**：使能 → MIT 定点 → 失能 | 关节按预期运动；断链 100 ms 内看门狗抱死 | 需要电机 |
| 12 | **真链路上跑 FD**（`--data-bitrate 5000000`） | FD 帧能发出去、对端能收到；缺 `CANFD_FDF` 时真控制器/新内核会拒（EINVAL），**而 vcan 不会拒** | `vcan` 不做收发器层校验 → 这一处修复**无法自动化证明**（见 M3） |

⚠ 第 10 项特别容易误判：`vcan` 上永远拿不到 `CAN_ERR_BUSOFF` / `CAN_ERR_CRTL_RX_PASSIVE`，
所以"错误帧路径在软件里测过了"是**不成立**的结论 —— 那部分只能靠第 9~11 项人工验。
**推荐一次性跑完上面这些（可重复、可量化）**：

```bash
./tools/hw_verify.sh --channel COM3 --node 1 --runs 5        # 10 层 × 5 轮，逐层给结论
./tools/hw_verify.sh --if virtual --runs 1                   # 没硬件时先自检脚本本身
```

它把"真机验证"拆成 **10 层**（发现 / 设备信息 / 描述符 / 心跳 / 单读 / 批量读 / 真值断言 /
健康 / **写路径** / **写探针**），**每层单独起进程**，因此能定位到具体哪一层坏；
`read` 与 `batch-read` 的同一参数会**互相对拍**（两条解析路径不一致会当场报出来）。

**写路径**（上表第 11 项里"能自动化的那部分"）现在是脚本化的：`--write-probe` 会
`读原值 → 写原值+Δ → 校验 → 恢复原值 → 校验`，**任何一步失败都先尝试恢复**；
只碰 `--write-path` 指定的那一个参数（默认 `axis0.config.can.heartbeat_rate_ms`，u32、
非安全关键、不落 Flash）。真机实测 `100 → 150 → 恢复 100` ✓ —— 这同时验证了
**写方向的字节序**（若按大端写，设备会把 150 存成 `0x96000000`）。
⚠ 它**不会**去动 `set-node-id` / `reset` / `save` 这类**不可幂等重放**的命令 ——
那些仍然只能人工、且要有明确目的。

为什么不能"跑一次看到成功就宣布通过"：slcan 适配器打开端口后的头几帧会丢
（实测约 1/10 次进程），而且**字节序错时 `read` 依然成功返回、只是数值荒谬**
—— 详见 `CLI.zh-CN.md` §8。
### 7.5.4 ⚠ 客户程序打印中文时的显示问题（Windows，与硬件无关但一定会遇到）

SDK 自己**从不打印**任何东西 —— 它只是把中文（UTF-8）放进
`jsdk_context_last_error()`、`jsdk_status_string()`、`jsdk_mode_name()` 等返回值里，
由你决定怎么输出。于是这个坑落在调用方：Windows 控制台默认代码页是 **CP936**，
把 UTF-8 字节直接 `printf` 出去，屏幕上就是 `鍙戠幇 0 涓�鑺傜偣` 这种乱码
（`jsdk-cli` 早期就是这样，见 `CLI.zh-CN.md` §1）。

**规则一句话**：输出的字节要符合**接收端**的约定 —— 目标是控制台就按
`GetConsoleOutputCP()` 转码；目标是管道/文件就原样 UTF-8。

最小可用骨架（约 20 行，可直接抄）：

```c
/* 需要 <stdio.h> <string.h>；以下只看关键部分 */
#ifdef _WIN32
#include <windows.h>
#include <io.h>

static int is_console(FILE *f)
{
    DWORD mode;
    int fd = _fileno(f);
    intptr_t h;
    if (fd < 0 || _isatty(fd) == 0) return 0;          /* 管道/文件 → 不转 */
    h = _get_osfhandle(fd);
    return (h != (intptr_t)-1) && GetConsoleMode((HANDLE)h, &mode) != 0;
}

/* 把一段 UTF-8 文本写到 f。是控制台就转成控制台代码页，否则原样。 */
static void say(FILE *f, const char *utf8)
{
    wchar_t w[1024];
    char    out[2048];
    int     wlen, m;

    if (!is_console(f) || (strlen(utf8) > 1000u)) { fputs(utf8, f); return; }
    wlen = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, utf8, -1, w, 1024);
    if (wlen <= 0) { fputs(utf8, f); return; }
    m = WideCharToMultiByte(GetConsoleOutputCP(), WC_NO_BEST_FIT_CHARS,
                            w, wlen - 1, out, (int)sizeof out, "?", NULL);
    if (m <= 0) { fputs(utf8, f); return; }
    fwrite(out, 1u, (size_t)m, f);
}
#else
#define is_console(f) 0                                  /* Linux/macOS 终端就是 UTF-8 */
#define say(f, s)     fputs((s), (f))
#endif

say(stdout, jsdk_context_last_error(ctx));               /* 替代 printf("%s", ...) */
```

三个要点：

1. **按完整 UTF-8 码点发**。上面这个"一次调用一个串"的写法天然安全；若要自己
   切缓冲（例如逐字节写、或写超过 1 KB 的文本），**绝不能把 3 字节汉字从中间切开**
   （切开的那一段单独转码会失败/变成 `?`）。参考实现见
   `tools/jsdk_cli/cli_text.h` 的 `cli_utf8_complete_prefix()` 与
   `cli_text.c` 的分片状态机（含跨调用残留处理、纯 ASCII 快路径）。
2. **别用 `chcp 65001` 了事**：那改的是**共享的控制台状态**，程序被 kill 就回不去，
   同控制台的其它进程跟着遭殃，而且要求控制台字体有 CJK 字形。
3. **不要在输出里用 `⚠`（U+26A0）、`✓`（U+2713）** 这类字符：CP936 表示不了，
   转码后只能变成 `?`（本工程把这条写成了静态检查 `tools/check_cli_text.py`）。

> 什么时候可以不管：目标是 Linux/macOS 终端，或者你的程序只在 UTF-8 终端里跑
> （MinTTY、Windows Terminal 里显式 `chcp 65001`、CI 日志）。只有"Windows 原生控制台 +
> 中文"这一种组合需要上面这段。

---

## 8. 移植检查表

| # | 项 | ✔ |
|---|---|---|
| 1 | `send`/`recv`/`now_ms` 三个必要回调已实现，且 `send`/`recv` 非阻塞 | |
| 2 | 29-bit **扩展帧**已正确设置（`id` 不含 EFF 标志位，硬件 EFF 已开） | |
| 3 | FD 的 `DataLength` 使用了 **DLC 编码**而非字节数 | |
| 4 | `is_fd` 与设备 `can.config.baud_rate` 一致（Classic ≤1 Mbps）。⚠ 不确定就**不要**置 `cfg.is_fd_explicit`：留 0 时 SDK 会按对端帧自动对齐；置 1 且写错就是必失败（设备不收 FD 帧） | |
| 4b | 仿真 / HIL 后端：配成 Classic 的节点必须**真的丢掉 FD 帧**（别比真机宽容），否则“帧格式猜错”这类测试会碰巧通过 | |
| 5 | RX 环形队列 ≥128（FD）/ ≥256（Classic），溢出丢最旧帧并计数 | |
| 6 | `now_ms` 单调，`uint32_t` 回绕安全 | |
| 7 | `master_id` ≠ 0 | |
| 8 | arena 已按 §2.3 流程实测并缩容，有编译期下限断言 | |
| 9 | `filter_paths` 覆盖 12 条必需集（+ 诊断项） | |
| 10 | Flash 缓存实现：双槽、扇区对齐、magic+CRC 自证、写后回读校验 | |
| 11 | 缓存键：路线 B = `fw_version` + `desc_crc`；路线 A 还需 + **`filter_hash`** + SDK 格式版本 | |
| 12 | 启动顺序：**取描述符 → 再 `activate()`** | |
| 13 | `desc_fetch()` 有 ≤3 次重试 + 仅诊断模式的降级路径 | |
| 14 | 缓存失效/损坏时能自动回退到下载，且不阻塞启动 | |
| 15 | `configure()` 失败（量程不合理）时**拒绝进入物理量 API** | || 16 | 路线 B：`stop_when_satisfied = 0`，且事后校验 `complete` / `raw_sink_failed` / `s_raw_len == total_len` | |
| 17 | `desc_raw_sink` 回调里只做 Flash 写入 + CRC 累计，不调用其他 SDK API（避免重入） | |
| 18 | Flash 缓存读取失败/失效时**不阻塞启动**，能自动回落到下载 | |