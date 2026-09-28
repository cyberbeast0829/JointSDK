/**
 * @file    sim_device.h
 * @brief   虚拟驱动器行为模型（内置，仅桌面平台）
 *
 * 这是 `jsdk_hal_virtual` 内部的“固件替身”：接收主站发来的帧，按固件语义
 * 更新状态并生成应答。目的是让 SDK 在没有硬件时也能做**端到端**回归
 * （而不只是单帧编解码测试），并让客户在没有驱动器时就能开发上位机。
 *
 * @par 设计边界（重要）
 *  本模型**不是**固件的精确仿真，也不用于验证控制性能。它只保证：
 *    ① 帧级行为正确（寻址、长度校验、应答类型与 seq、错误语义、超时）
 *    ② 单位换算与固件一致（输出端/电机端、gear_ratio、torque_constant）
 *    ③ 描述符传输时序（元数据帧 + 数据帧、每毫秒帧数上限）
 *  物理模型是**玩具级**一阶模型，不可用于调参或性能评估。
 *
 * @par 端点表用真实 ID
 *  ID 与类型全部取自固件 v8 的 JSON 描述符（`tests/data/endpoints_v8.json`），
 *  因此模拟器可以直接服务真实描述符，SDK 的动态端点解析路径能被完整测到。
 *  其中 `axis0.motor.error` 是 **uint64（8 字节）**——用于测试 Classic 分段读写。
 *
 * @par 与零 malloc 核心的关系
 *  本模块位于零 malloc 核心**之外**，允许 malloc（JSON 描述符缓冲）。
 *  只在内置 HAL 被选择编译时参与构建。
 *
 * @par 行为基准
 *  `ODrive @ CyberBeast`，`can_cyberbeast.cpp`。已复刻：
 *  `is_message_for_me` 判定顺序、广播槽位 = node_id、`is_ctrl` 只含 0x00~0x03
 *  与 0x80~0x83、`break_timeout` 缺省 100 ms、`kMaxJsonFramesPerCycle = 50`、
 *  超时置 `CAN_BUS_FAILED` 后 `disarm()`、描述符元数据帧判别（看 `buf[2]`）。
 */

#ifndef JSDK_SIM_DEVICE_H
#define JSDK_SIM_DEVICE_H

#include <stddef.h>
#include <stdint.h>

#include "joint_sdk.h"   /* jsdk_can_frame_t / JSDK_FRAME_* */
#include "cb_param.h"    /* cb_param_write_asm_t（分段写装配器随总线走） */

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * 容量与缺省常量
 * ------------------------------------------------------------------------ */

#define SIM_MAX_NODES     4u     /**< 同一条虚拟总线上的节点数上限 */
#define SIM_MAX_ENDPOINTS 48u    /**< 端点表容量 */
#define SIM_TX_QUEUE      1024u  /**< 出站（D→M）帧队列容量 */

#define SIM_GEAR_RATIO_DEFAULT       16.0f
#define SIM_TORQUE_CONST_DEFAULT     0.0385f
#define SIM_MIT_POS_DEFAULT          12.5f
#define SIM_MIT_VEL_DEFAULT          65.0f
#define SIM_MIT_KP_DEFAULT           500.0f
#define SIM_MIT_KD_DEFAULT           5.0f
#define SIM_MIT_TAU_DEFAULT          50.0f
/**
 * 仿真节点的 `can.config.break_timeout` 默认值。
 *
 * ⚠ **0 = 协议级超时检测被禁用** —— 与最新固件的默认值（`Config_t::break_timeout = 0`）
 *   以及 `auto_stop_if_timeout()` 的首句 `if (timeout_ms == 0) return;` 一致。
 *   旧模型把 0 当 100 ms，会让“未武装”看起来像“已武装 100 ms”（曾被真机扇了一耳光，
 *   见 FIRMWARE_ISSUES F28）。要模拟“已武装”请在节点规格里显式写 `timeout=<ms>`。
 */
#define SIM_BREAK_TIMEOUT_DEFAULT_MS 0u
#define SIM_JSON_FRAMES_PER_CYCLE    50u     /**< 固件 kMaxJsonFramesPerCycle */
#define SIM_NODE_ID_DEFAULT          1u
#define SIM_DEFAULT_VBUS             48.0f
#define SIM_DEFAULT_MOTOR_TEMP       25.0f
#define SIM_DEFAULT_FET_TEMP         30.0f

/** `Axis::ERROR_CAN_BUS_FAILED`（固件值，见 autogen/interfaces.hpp） */
#define SIM_ERR_CAN_BUS_FAILED       0x00100000u
/** `Axis::ERROR_ESTOP_REQUESTED` */
#define SIM_ERR_ESTOP_REQUESTED      0x00004000u
/** `Motor::ERROR_STALL`（固件值）——用于让 `detect_error_code()` 产出 `CB_ERR_STALL` */
#define SIM_MERR_STALL               0x2000000000ull
/** `Motor::ERROR_OVERLOAD`（固件值）——用于产出 `CB_ERR_OVERLOAD` */
#define SIM_MERR_OVERLOAD            0x4000000000ull
/** `InputMode::INPUT_MODE_MIT`（MIT 帧会把 input_mode 设为它） */
#define SIM_INPUT_MODE_MIT           9u
/** `Controller::ControlMode` */
#define SIM_CM_VOLTAGE  0u
#define SIM_CM_TORQUE   1u
#define SIM_CM_VELOCITY 2u
#define SIM_CM_POSITION 3u
/** `Axis::AxisState` */
#define SIM_AS_IDLE              1u
#define SIM_AS_MOTOR_CALIB       4u   /* 全标定的第一个子状态 */
#define SIM_AS_ENC_INDEX_SEARCH  7u   /* 真机实测的第二个子状态 */
#define SIM_AS_CLOSED_LOOP       8u
#define SIM_AS_FULL_CALIB        3u   /* AXIS_STATE_FULL_CALIBRATION_SEQUENCE */
#define SIM_AS_HOMING           11u   /* AXIS_STATE_HOMING */
#define SIM_AS_UNDEFINED         0u

/** 标定/回零瞬时状态持续多久（ms）—— 真实固件是几百 ms 到数秒。 */
#define SIM_TRANSIENT_MS        50u

/**
 * 全标定的**子状态**各持续多久（ms）。
 *
 * ⚠⚠ 模型必须报**子状态**：真机上 `FULL_CALIBRATION_SEQUENCE(3)` 期间
 * `current_state` 是 4（电机标定）→ 7（索引搜索）→ 1，**从不等于 3**。
 * 以前模型直接报 3 且只持续 `SIM_TRANSIENT_MS`(50 ms)，于是：
 *   ① 上位机拿“== 3”当判据的 bug 在仿真上**永远绿**；
 *   ② 上位机改成限速轮询（真机必须这么做）之后，50 ms 的子状态
 *      **被整个错过** → 误报 “never left idle”。
 * 现在按真机的量级建模（实测整条序列 29.5 s，两段共 ~30 s）。
 */
#define SIM_CAL_STAGE_MS       2000u

/** 端点值类型（线宽，与 JSON 描述符的 type 字符串一致） */
typedef enum {
    SIM_T_U8 = 0, SIM_T_I8, SIM_T_U16, SIM_T_I16,
    SIM_T_U32, SIM_T_I32, SIM_T_U64, SIM_T_I64,
    SIM_T_F32, SIM_T_F64, SIM_T_BOOL
} sim_val_type_t;

#define SIM_ACC_READ  0x01u
#define SIM_ACC_WRITE 0x02u

/** 端点定义：值放在 sim_node_t 内，用 offset 定位（避免逐字段写代码）。 */
typedef struct {
    uint16_t    id;
    const char *path;
    uint8_t     type;      /**< sim_val_type_t */
    uint8_t     access;    /**< SIM_ACC_* */
    size_t      offset;    /**< offsetof(sim_node_t, 字段) */
} sim_ep_def_t;

/**
 * 一个节点（轴）的状态。
 *
 * ⚠ 前一段字段被端点表按 offset 引用，**类型必须与描述符完全一致**
 *   （例如 `axis0.motor.error` 是 uint64，就不能写成 uint32）。
 */
typedef struct {
    /* ---- 端点可见字段 ---- */
    uint8_t  error_board;          /*   1  error                             u8  rw */
    float    vbus_voltage;         /*   2  vbus_voltage                      f32 r  */
    float    ibus;                 /*   3  ibus                              f32 r  */
    uint64_t serial_number;        /*   5  serial_number                     u64 r  */
    uint8_t  hw_version_major;     /*   6  hw_version_major                  u8  r  */
    uint8_t  fw_version_major;     /*   9  fw_version_major                  u8  r  */
    uint16_t break_timeout;        /*  73  can.config.break_timeout          u16 rw */
    uint32_t error_axis;           /* 138  axis0.error                       u32 rw */
    uint8_t  requested_state;      /* 143  axis0.requested_state            u8  rw */
    uint8_t  current_state;        /* 142  axis0.current_state              u8  r  实际状态 */
    float    watchdog_timeout;     /* 153  axis0.config.watchdog_timeout     f32 rw */
    uint8_t  enable_watchdog;      /* 154  axis0.config.enable_watchdog      bool rw */
    uint32_t node_id;              /* 180  axis0.config.can.node_id          u32 rw */
    uint8_t  is_extended;          /* 181  axis0.config.can.is_extended      bool rw */
    uint32_t heartbeat_rate_ms;    /* 182  axis0.config.can.heartbeat_rate_ms u32 rw */
    uint64_t error_motor;          /* 193  axis0.motor.error                 u64 rw ⚠8B */
    float    fet_temp;             /* 207  ...fet_thermistor.temperature     f32 r  */
    float    motor_temp;           /* 211  ...motor_thermistor.temperature   f32 r  */
    float    iq_measured;          /* 232  ...current_control.Iq_measured    f32 r  */
    float    gear_ratio;           /* 242  axis0.motor.config.gear_ratio     f32 rw */
    float    torque_constant;      /* 247  axis0.motor.config.torque_constant f32 rw */
    float    current_lim;          /* 249  axis0.motor.config.current_lim    f32 rw */
    float    torque_lim;           /* 251  axis0.motor.config.torque_lim     f32 rw */
                                     /*      ↳ POS/VEL 帧会持久改写它（固件副作用） */
    uint8_t  error_controller;     /* 268  axis0.controller.error            u8  rw */
    float    input_pos;            /* 270  axis0.controller.input_pos        f32 rw */
    float    input_vel;            /* 271  axis0.controller.input_vel        f32 rw */
    float    input_torque;         /* 272  axis0.controller.input_torque     f32 rw */
    uint8_t  control_mode;         /* 287  ...config.control_mode            u8  rw */
    uint8_t  input_mode;           /* 288  ...config.input_mode              u8  rw */
    float    vel_limit;            /* 301  ...config.vel_limit               f32 rw */
    float    mit_max_pos;          /* 335  ...config.mit_max_pos             f32 rw */
    float    mit_max_vel;          /* 336  ...config.mit_max_vel             f32 rw */
    float    mit_max_torque;       /* 337  ...config.mit_max_torque          f32 rw */
    float    mit_max_kp;           /* 338  ...config.mit_max_kp              f32 rw */
    float    mit_max_kd;           /* 339  ...config.mit_max_kd              f32 rw */
    uint16_t error_encoder;        /* 365  axis0.encoder.error               u16 rw */
    float    pos_estimate;         /* 372  axis0.encoder.pos_estimate        f32 r  电机端 turns */
    float    vel_estimate;         /* 378  axis0.encoder.vel_estimate        f32 r  电机端 turns/s */
    int32_t  cpr;                  /* 390  axis0.encoder.config.cpr          i32 rw */
    uint8_t  motor_pre_cal;        /* 240  axis0.motor.config.pre_calibrated   bool rw */
    uint8_t  enc_pre_cal;          /* 394  axis0.encoder.config.pre_calibrated bool rw */

    /* ---- 非端点字段 ---- */
    uint32_t is_fd;                /* 该节点用 CAN FD 通信 */
    uint8_t  armed;
    uint8_t  estop;
    uint8_t  life;
    uint8_t  bcast_seen;
    uint8_t  drop_writes;          /* 故障注入：像固件 `cmd_param_write()` 的
                                      `if (msg.len < 8) return;` 那样**静默丢弃**
                                      PARAM_WRITE（不回 ACK、不改值）。
                                      用于验证“写后读回”真的能抓到丢帧。 */

    uint32_t last_cmd_ms;          /* is_ctrl 帧上次到达（0 = 从未收到） */
    uint32_t last_heartbeat_ms;
    uint32_t cmd_count;
    uint32_t state_change_ms;      /* current_state 最近一次**由瞬时态转定态**的时刻 */
    uint32_t transient_until_ms;   /* > now_ms 时 current_state 停在瞬时态 */
    uint8_t  settle_to;            /* 瞬时态结束后回到哪个状态 */
    uint32_t requested_hits;       /* 通过端点写入 requested_state 的次数 */
    uint8_t  cal_stage;            /* 全标定子状态进度：0=未开始 1=电机标定 2=索引搜索 */
    uint32_t param_err_count;      /* 因装不下而回 ERR 的批量请求次数 */
    uint8_t  tx_seq;               /* ⚠ 设备本地滚动计数器（固件 tx_seq_[axis]++）
                                      响应帧用它，**不回显请求的 Seq** */

    /* 最近一条控制帧的解码结果（供测试断言单位换算；非端点） */
    uint8_t  last_control_msgtype;
    float    cmd_pos_out_rad;
    float    cmd_vel_out_rad_s;
    float    cmd_kp;
    float    cmd_kd;
    float    cmd_tau_out_nm;
    float    cmd_torque_motor_nm;
    float    pos_target_motor;
    float    vel_target_motor;
} sim_node_t;

/** 虚拟总线（多节点） */
typedef struct {
    sim_node_t nodes[SIM_MAX_NODES];
    size_t     n_nodes;
    uint32_t   now_ms;
    uint32_t   last_tick_ms;
    /** 描述符流速率（帧/ms）；0 = 默认 `SIM_JSON_FRAMES_PER_CYCLE`。
        用来复现“**流得很慢但一直在动**”（真机 Classic 就是 FD 的 10.4 倍帧数）——
        那种情况下按“总时长”算的预算会误判成超时。 */
    uint32_t   desc_rate;

    /* 出站帧队列（模型 → HAL → SDK 接收路径） */
    jsdk_can_frame_t txq[SIM_TX_QUEUE];
    uint32_t         txq_head;
    uint32_t         txq_tail;
    uint32_t         txq_dropped;

    /* 统计 */
    uint32_t rx_frames;
    uint32_t rx_for_me;
    uint32_t tx_frames;
    uint32_t bad_len_drops;    /** 被“帧格式门限”丢掉的帧数：配成 **Classic** 的节点收到 FD 帧（真实控制器
        解析不了 FD 帧，现场表现就是“心跳收得到、请求没人应”）。
        有它才能让测试断言“请求**确实**被丢了”，而不是刚好被宽容地放过。 */
    uint32_t fd_into_classic_drops;
    uint32_t unhandled;
    /** 首帧丢失注入：丢掉主站最初的 `drop_tx_head` 帧（真机 = 适配器刚打开时
        头一两帧上不了总线）。`dropped_tx_head` 是实际丢掉的计数。 */
    uint32_t drop_tx_head;
    uint32_t dropped_tx_head;
    /** 运行中途丢帧注入：丢掉主站发出的**某个 MsgType** 的前 `drop_msgtype_n` 帧。
        与 `drop_tx_head` 的区别是“发生在会话中间” —— 用来验证幂等请求重发。 */
    uint32_t drop_msgtype;
    uint32_t drop_msgtype_n;
    uint32_t dropped_msgtype;

    /* JSON 描述符（0x24 / 0x25） */
    struct {
        uint8_t *json;
        uint32_t len;
        uint16_t crc;
        int      owned;
        int      active;
        int      metadata_sent;
        uint32_t offset;
        uint32_t master_id;
        uint32_t my_id;
        int      is_classic;
    } desc;

    /* 故障注入 */
    uint32_t force_txq_full;

    /**
     * 分段写（Classic `PARAM_WRITE` 多块）的装配器，**每个总线一份**。
     *
     * ⚠ 早期版本是文件级 `static g_asm[SIM_MAX_NODES]`，那样**同一进程里两条
     *   虚拟总线会共用同一组槽位**（节点下标相同就碰撞）→ 多 CAN 口测试
     *   会出现“A 总线写入的数据拼进了 B 总线”的假象。装配器属于总线状态，
     *   必须跟着总线走。
     */
    cb_param_write_asm_t asm_state[SIM_MAX_NODES];
} sim_bus_t;

/* --------------------------------------------------------------------------
 * 生命周期
 * ------------------------------------------------------------------------ */

/** 初始化总线；`n` = 0 时创建缺省单节点（node_id = 1）。 */
void sim_bus_init(sim_bus_t *b, size_t n);

/** 释放动态内存（幂等）。 */
void sim_bus_free(sim_bus_t *b);

/**
 * 按 `node_spec` 配置节点。
 *
 * 语法：`<index>[:k=v[,k=v...]][;...]`（index 从 0 开始，对应 nodes[]）
 * 键：`id` `gear` `tconst` `pmax` `vmax` `kpmax` `kdmax` `tmax` `hb`
 *     `timeout` `vb` `temp`；无值键：`fd` / `classic` / `arm` / `disarm`
 * 例：`"0:gear=16.5,pmax=12.5,vmax=65,tmax=50,fd;1:gear=8,id=2,classic"`
 *
 * @return 0 成功；-1 语法错误
 */
int sim_configure(sim_bus_t *b, const char *node_spec);

/* --------------------------------------------------------------------------
 * 收发与时间
 * ------------------------------------------------------------------------ */

/** 把一帧交给模型处理（等价于“总线上出现了一帧”）。 */
void sim_rx(sim_bus_t *b, const jsdk_can_frame_t *f);

/** 取出模型生成的下一帧。@return 1 = 取到；0 = 空。 */
int sim_tx_pop(sim_bus_t *b, jsdk_can_frame_t *f);

/**
 * 推进时间到 `now_ms` 并执行周期任务：
 * 逐毫秒推进物理模型 → 检查 `break_timeout` → 到期发心跳 →
 * 继续未完成的描述符传输（每毫秒 ≤ SIM_JSON_FRAMES_PER_CYCLE 帧）。
 */
void sim_tick(sim_bus_t *b, uint32_t now_ms);

/* --------------------------------------------------------------------------
 * JSON 描述符
 * ------------------------------------------------------------------------ */

/** 设置描述符内容（内部拷贝）。crc = 0 时用字节和占位。 */
int sim_set_desc(sim_bus_t *b, const char *json, uint32_t len, uint16_t crc);

/** 从文件读入描述符（stdio，仅桌面平台）。crc 语义同上。 */
int sim_set_desc_file(sim_bus_t *b, const char *path, uint16_t crc);

/** 描述符传输是否仍在进行。 */
int sim_desc_active(const sim_bus_t *b);

/* --------------------------------------------------------------------------
 * 端点表
 * ------------------------------------------------------------------------ */

const sim_ep_def_t *sim_default_endpoints(size_t *count_out);
const sim_ep_def_t *sim_find_ep(const sim_node_t *n, uint16_t ep_id);

/** 读端点值（线上字节序）。@return 实际字节数；未知端点返回 0。 */
uint8_t sim_ep_read(const sim_node_t *n, const sim_ep_def_t *def,
                    uint8_t *out, uint8_t cap);

/** 写端点值（线上字节序）。@return 0 成功；-1 只读或长度不符。 */
int sim_ep_write(sim_node_t *n, const sim_ep_def_t *def,
                 const uint8_t *in, uint8_t len);

/** 端点值的线宽（字节）；未知返回 0。 */
uint8_t sim_ep_width(uint8_t type);

/* --------------------------------------------------------------------------
 * 便捷查询
 * ------------------------------------------------------------------------ */

/** 按当前 node_id 查找节点（注意 node_id 可能被主站改写）。 */
sim_node_t *sim_find_node(sim_bus_t *b, uint32_t node_id);

/** 清空统计与故障注入（不清节点状态）。 */
void sim_clear_stats(sim_bus_t *b);

/** 设置描述符流速率（帧/ms）；0 = 恢复默认。用于“慢但持续”的流。 */
void sim_set_desc_rate(sim_bus_t *b, uint32_t frames_per_ms);

#ifdef __cplusplus
}
#endif

#endif /* JSDK_SIM_DEVICE_H */
