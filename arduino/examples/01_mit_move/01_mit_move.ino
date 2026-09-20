/*
 * 01_mit_move — Arduino 上跑一个关节：使能 → MIT 周期控制 → 失能
 *
 * 结构就是 MCU 客户该有的样子：
 *   - **零 malloc**：上下文、arena、配置全是 static；
 *   - **不建线程**：控制循环在 loop() 里，靠 millis() 门控周期；
 *   - **描述符不过 CAN**：常量 JSON + desc_import_raw()（见文件末尾的 kJson）；
 *   - 传输层只有三个回调（can_hal_impl.h）。
 *
 * ⚠ 上板前必读：
 *   1. `kJson` 里的端点路径/ID 必须与你设备固件**一致**（用
 *      `tools/extract_endpoints_json.py` 从 Firmware/autogen/endpoints.hpp 生成，
 *      再按需裁剪到 RETAIN_FILTERED 的十几条）；
 *   2. `break_timeout`：本 sketch 周期 10 ms，远小于固件默认 100 ms ✓；
 *   3. 第一次上板请**先不接电机**（或把 MAX_TAU 设成 0），确认能收到心跳/响应，
 *      再逐步加上力矩。
 */

#include "can_hal_impl.h"

/* --------------------------------------------------------------------------
 * 配置（按你的硬件改）
 * ------------------------------------------------------------------------ */

#define JOINT_NODE_ID   1u          /* 设备的 node_id */
#define MASTER_ID       7u          /* 主站地址（≠ 0） */
#define PERIOD_US       10000u      /* 100 Hz（= 10 ms，必须 < break_timeout） */
#define MAX_TAU_NM      2.0         /* 本示例的力矩上限：先给很小，确认能跑再加 */
#define AMPLITUDE_RAD   0.3         /* 正弦摆幅 */

/* --------------------------------------------------------------------------
 * 零 malloc：全部静态
 * ------------------------------------------------------------------------ */

static jsdk_can_hal_t         s_hal;
static jsdk_context_config_t  s_cfg;      /* ⚠ 必须与 arena 同寿命（见 SDK 文档） */
static unsigned char          s_arena[4096];
static jsdk_context_storage_t s_store;
static jsdk_joint_t          *s_joint = nullptr;
static uint32_t               s_t0_ms;
static uint32_t               s_last_cycle_ms;
static unsigned               s_cycles;

/* 只保留自己用到的路径（RETAIN_FILTERED）。数组必须与 s_cfg 同寿命。 */
static const char *const kFilters[] = {
    "axis0.config.can.node_id",
    "axis0.motor.config.gear_ratio",
    "axis0.motor.config.torque_constant",
    "axis0.controller.config.mit_max_pos",
    "axis0.controller.config.mit_max_vel",
    "axis0.controller.config.mit_max_torque",
    "axis0.controller.config.mit_max_kp",
    "axis0.controller.config.mit_max_kd",
    "axis0.config.can.heartbeat_rate_ms",
    "axis0.error"
};

const char kJson[] PROGMEM =
    "[{\"name\":\"axis0\",\"type\":\"object\",\"members\":["
      "{\"name\":\"config\",\"type\":\"object\",\"members\":["
        "{\"name\":\"can\",\"type\":\"object\",\"members\":["
          "{\"name\":\"node_id\",\"type\":\"uint32\",\"access\":\"rw\",\"id\":180},"
          "{\"name\":\"heartbeat_rate_ms\",\"type\":\"uint32\",\"access\":\"rw\",\"id\":182}"
        "]}"
      "]},"
      "{\"name\":\"motor\",\"type\":\"object\",\"members\":["
        "{\"name\":\"config\",\"type\":\"object\",\"members\":["
          "{\"name\":\"gear_ratio\",\"type\":\"float\",\"access\":\"rw\",\"id\":242},"
          "{\"name\":\"torque_constant\",\"type\":\"float\",\"access\":\"rw\",\"id\":247}"
        "]}"
      "]},"
      "{\"name\":\"controller\",\"type\":\"object\",\"members\":["
        "{\"name\":\"config\",\"type\":\"object\",\"members\":["
          "{\"name\":\"mit_max_pos\",\"type\":\"float\",\"access\":\"rw\",\"id\":335},"
          "{\"name\":\"mit_max_vel\",\"type\":\"float\",\"access\":\"rw\",\"id\":336},"
          "{\"name\":\"mit_max_torque\",\"type\":\"float\",\"access\":\"rw\",\"id\":337},"
          "{\"name\":\"mit_max_kp\",\"type\":\"float\",\"access\":\"rw\",\"id\":338},"
          "{\"name\":\"mit_max_kd\",\"type\":\"float\",\"access\":\"rw\",\"id\":339}"
        "]}"
      "]},"
      "{\"name\":\"error\",\"type\":\"uint32\",\"access\":\"r\",\"id\":138}"
    "]}]";

static void fatal(const __FlashStringHelper *what, jsdk_status_t st)
{
    Serial.print(F("jsdk: "));
    Serial.print(what);
    Serial.print(F(" -> "));
    Serial.println(jsdk_status_string(st));
    for (;;) { delay(1000); }              /* 停在明确的状态上，别带病跑电机 */
}

void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println(F("=== CyberBeast Joint SDK / Arduino ==="));
    Serial.print(F("backend="));
    Serial.println(jsdk_backend_name());

    can_hal_init(&s_hal);

    jsdk_context_config_default(&s_cfg);
    s_cfg.hal              = s_hal;
    s_cfg.master_id        = MASTER_ID;
    s_cfg.is_fd            = 0u;           /* MCP2515 一般是 Classic；FDCAN 改 1 */
    s_cfg.period_ns        = (uint32_t)PERIOD_US * 1000u;
    s_cfg.desc.mode        = JSDK_DESC_DYNAMIC;
    s_cfg.desc.retain      = JSDK_DESC_RETAIN_FILTERED;
    s_cfg.desc.filter_paths = kFilters;
    s_cfg.desc.filter_count = sizeof kFilters / sizeof kFilters[0];
    s_cfg.desc.timeout_ms  = 2000u;
    s_cfg.desc.arena       = s_arena;
    s_cfg.desc.arena_size  = sizeof s_arena;

    jsdk_status_t st = jsdk_context_init((jsdk_context_t *)&s_store, &s_cfg);
    if (st != JSDK_OK) fatal(F("context_init"), st);

    jsdk_joint_config_t jc;
    memset(&jc, 0, sizeof jc);
    jc.node_id      = JOINT_NODE_ID;
    jc.initial_mode = JSDK_MODE_MIT;
    st = jsdk_context_add_joint((jsdk_context_t *)&s_store, &jc, &s_joint);
    if (st != JSDK_OK) fatal(F("add_joint"), st);

    /* --- 描述符：用常量 JSON（MCU 上不要走 CAN 下 41 KB） --- */
    jsdk_desc_hint_t hint;
    hint.crc        = 0u;                  /* 真机上存进 Flash，用于缓存失效判定 */
    hint.fw_version = 0u;
    st = jsdk_context_desc_import_raw((jsdk_context_t *)&s_store,
                                      kJson, sizeof(kJson) - 1u, &hint);
    if (st != JSDK_OK) fatal(F("desc_import_raw"), st);

    st = jsdk_context_configure((jsdk_context_t *)&s_store);
    if (st != JSDK_OK) {
        Serial.print(F("configure: "));
        Serial.println(jsdk_context_last_error((jsdk_context_t *)&s_store));
        fatal(F("configure"), st);
    }
    Serial.print(F("arena 用了 "));
    Serial.print((unsigned)s_cfg.desc.arena_used);
    Serial.print(F(" / "));
    Serial.println(sizeof s_arena);

    /* --- 使能（MCU 上不要用阻塞版 activate()，会把 loop() 卡住；
           用非阻塞请求 + 在 loop() 里跑周期） --- */
    jsdk_joint_request_enable(s_joint, JSDK_MODE_MIT);
    s_t0_ms = millis();
    s_last_cycle_ms = s_t0_ms;

    Serial.println(F("使能请求已发出；等 is_enabled() 变 1 之后开始摆动"));
}

void loop()
{
    uint32_t now = millis();

    /* 1 kHz 以内都可以这么门控；更高频率请用定时器中断（别在中断里跑整条回路） */
    if ((uint32_t)(now - s_last_cycle_ms) < (PERIOD_US / 1000u)) return;
    s_last_cycle_ms = now;

    jsdk_context_t *ctx = (jsdk_context_t *)&s_store;

    /* 1) 周期开始：收帧（心跳/响应都在这一步被吃掉） */
    if (jsdk_context_cycle_begin(ctx, (uint64_t)(now - s_t0_ms) * 1000000ull) != JSDK_OK) {
        return;
    }

    /* 2) 控制律：使能完成后开始正弦摆动，否则发"零增益"安全指令 */
    if (jsdk_joint_is_enabled(s_joint)) {
        double phase = (double)(now - s_t0_ms) * 0.0005;      /* ~0.5 rad/s */
        double pos   = AMPLITUDE_RAD * sin(phase);

        /* kp/kd 是**线上值**：输出端等效刚度 = kp × gear/(2π)。
           想让"真实刚度"就是这个数，用 jsdk_joint_set_mit_stiffness()。 */
        jsdk_joint_set_mit(s_joint, pos, 0.0, 2.0, 0.2, 0.0);
    } else {
        jsdk_joint_set_mit(s_joint, 0.0, 0.0, 0.0, 0.0, 0.0);  /* 零增益 */
    }

    /* 3) 周期结束：补 keepalive、判超时、发帧 */
    (void)jsdk_context_cycle_end(ctx);
    s_cycles++;

    /* 4) 每 2 秒报一次状态（串口是慢速外设，别每周期都打） */
    if ((s_cycles % 200u) == 0u) {
        jsdk_joint_feedback_t fb;
        if (jsdk_joint_get_feedback(s_joint, &fb) == JSDK_OK) {
            Serial.print(F("enabled=")); Serial.print(jsdk_joint_is_enabled(s_joint));
            Serial.print(F(" online=")); Serial.print(fb.online);
            Serial.print(F(" age="));    Serial.print(fb.age_ms);
            Serial.print(F(" pos="));    Serial.print(fb.pos, 3);
            Serial.print(F(" err=0x"));  Serial.print((unsigned)fb.err_code, HEX);
            Serial.print(F(" tx="));     Serial.println((unsigned)fb.tx_frames);
        }
        if (g_can.rx_dropped) {               /* 收不过来 = 循环太慢，先解决这个 */
            Serial.print(F("⚠ RX 溢出 ")); Serial.println(g_can.rx_dropped);
        }
    }

    /* 5) 安全兜底：串口打 'x' 立即停机 */
    if (Serial.available() && Serial.read() == 'x') {
        jsdk_context_estop(ctx);
        Serial.println(F("ESTOP 已广播"));
    }
}

/* --------------------------------------------------------------------------
 * 端点描述符（常量）
 *
 * ⚠ 下面只是**最小示例**：字段随便改几个就够跑通链路，但真实使用必须与你的
 *   固件一致 —— 用 `tools/extract_endpoints_json.py --hpp <固件的 endpoints.hpp>`
 *   生成完整表，再按 kFilters 裁剪（或用你自己的脚本 filter 出这十几条）。
 *   假 JSON 会导致 configure() 读不到量程而拒绝驱动电机（这是**有意的**保护）。
 * ------------------------------------------------------------------------ */
