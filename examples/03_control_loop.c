/**
 * @file    03_control_loop.c
 * @brief   1 kHz 控制循环 + 节拍统计（真机上你要写的就是这个骨架）
 *
 * 学到的四件事：
 *   1. 循环**由调用者拥有**：SDK 不建线程、不阻塞（除了配置阶段的几个显式阻塞 API）；
 *   2. `cycle_begin(app_time_ns)` / `cycle_end()` 的边界语义 ——
 *      begin 里做接收与到期发送，end 里补 keepalive 与超时判定；
 *      **目标必须在两者之间设置**（在这之外设的不会在本周期生效）；
 *   3. 用 `age_ms` 判"反馈新鲜"，而不是用"有没有数据"；
 *   4. 节拍统计（最大落后）必须自己算 —— 这是实时系统最先坏掉的地方。
 *
 * 真机上把末尾的忙等换成你的定时器/睡眠即可（注意别把周期睡过头，
 * 落后于设备的 break_timeout 就会触发它自己的看门狗）。
 *
 * 运行：
 *     ./build/examples/03_control_loop
 */

#include "ex_common.h"

#define LOOP_HZ      1000u
#define LOOP_SECONDS 2u
#define N_CYCLES     (LOOP_HZ * LOOP_SECONDS)

int main(void)
{
    ex_ctx_t              e;
    const unsigned        nodes[1] = { 1u };
    jsdk_joint_feedback_t fb;
    unsigned              k;
    double                worst_late_ms = 0.0;
    unsigned              sent = 0u;
    jsdk_bus_state_t      bus;

    printf("=== 03 控制循环 %u Hz × %u s ===\n", (unsigned)LOOP_HZ, (unsigned)LOOP_SECONDS);

    /* 周期 1 ms → 1000 Hz */
    ex_open(&e, "0:id=1,gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=10,timeout=30000,fd",
            nodes, 1u, 1000000u / LOOP_HZ);
    ex_check("activate", jsdk_context_activate(e.ctx));

    for (k = 0u; k < N_CYCLES; ++k) {
        uint64_t t_ns  = (uint64_t)k * e.cfg.period_ns;
        uint32_t t0_ms = e.hal.now_ms(e.hal.user);
        jsdk_status_t st;

        /* 1) 周期开始：收帧、把到期的东西发出去 */
        st = jsdk_context_cycle_begin(e.ctx, t_ns);
        if (st != JSDK_OK) { printf("  cycle_begin -> %s\n", jsdk_status_string(st)); break; }

        /* 2) 读写：这里放你的控制律（示例用正弦小幅摆动） */
        {
            double pos = 0.2 * (double)(k % 100u) / 100.0;   /* 0 → 0.2 rad 锯齿 */
            jsdk_joint_set_mit(e.joint[0], pos, 0.0, 1.5, 0.15, 0.0);
        }

        /* 3) 周期结束：补 keepalive、判定反馈超时 */
        st = jsdk_context_cycle_end(e.ctx);
        if (st != JSDK_OK) { printf("  cycle_end -> %s\n", jsdk_status_string(st)); break; }
        sent++;

        /* 4) 节拍统计：本周期实际花了多久 */
        {
            double late_ms = (double)(e.hal.now_ms(e.hal.user) - t0_ms);
            if (late_ms > worst_late_ms) worst_late_ms = late_ms;
        }
    }

    if (jsdk_joint_get_feedback(e.joint[0], &fb) == JSDK_OK) {
        printf("  cycles   : %u 个，每周期 %u ns（%.0f Hz）\n",
               sent, (unsigned)e.cfg.period_ns, 1e9 / (double)e.cfg.period_ns);
        printf("  worst late: %.3f ms（真机上这个值必须远小于 break_timeout）\n",
               worst_late_ms);
        printf("  feedback : online=%d age=%u ms pos=%.4f rad\n",
               fb.online, (unsigned)fb.age_ms, fb.pos);
    }

    if (jsdk_context_get_bus_state(e.ctx, &bus) == JSDK_OK) {
        printf("  bus      : tx=%u rx=%u keepalive=%u link_errors=%u dropped=%u\n",
               (unsigned)bus.tx_frames, (unsigned)bus.rx_frames,
               (unsigned)bus.keepalive_sent, (unsigned)bus.link_errors,
               (unsigned)bus.rx_dropped);
        /* 只看 keepalive：SDK 会在你忘记发控制帧时自动补喂狗，避免"自己把自己停掉" */
    }

    ex_close(&e);
    printf("done\n");
    return 0;
}
