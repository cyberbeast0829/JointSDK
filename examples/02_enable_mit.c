/**
 * @file    02_enable_mit.c
 * @brief   使能 → MIT 定点 → 失能（一次完整的安全动作序列）
 *
 * 学到的四件事：
 *   1. **顺序不可颠倒**：`configure()`（取量程）→ `activate()`（使能）→ 控制 → 失能；
 *   2. `is_enabled()` 只有在**使能序列走完（含安全首帧）**之后才为真 ——
 *      设备一进闭环就报 POSITION，那时下发运动指令就是"使能瞬间大跳变"；
 *   3. 失能**必须**走 `deactivate()`（它会先发安全帧再停），
 *      直接停发控制帧会在 `break_timeout` 后留下一个看门狗故障码；
 *   4. 反馈里的 `age_ms` 是"数据有多旧"，比"有没有反馈"更有意义。
 *
 * 运行：
 *     ./build/examples/02_enable_mit
 */

#include "ex_common.h"

int main(void)
{
    ex_ctx_t              e;
    const unsigned        nodes[1] = { 1u };
    jsdk_joint_feedback_t fb;
    unsigned              k;

    printf("=== 02 enable → MIT → disable ===\n");

    /* 周期 2 ms（500 Hz）；设备的 break_timeout 在 spec 里设成 30 s，
       所以 configure() 不会报"回路喂不了看门狗"（真机上要按 100 ms 默认值算）。 */
    ex_open(&e, "0:id=1,gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=10,timeout=30000,fd",
            nodes, 1u, 2000u);
    printf("  device  : gear=16.5 pos_max=12.5 rad tau_max=50 N·m\n");

    /* --- 使能（阻塞式：内部逐个发 CLEAR_ERRORS / START_MOTOR / 安全首帧） --- */
    ex_check("activate (enable)", jsdk_context_activate(e.ctx));
    printf("  enabled : %d（只有走完安全首帧才为真）\n",
           jsdk_joint_is_enabled(e.joint[0]));

    /* --- 跑 50 个周期，把目标从 0 慢慢推到 +0.5 rad --- */
    for (k = 0u; k < 50u; ++k) {
        double pos = 0.5 * (double)k / 49.0;

        /* MIT 的 kp/kd 是**线上值**：固件把它作用在电机端 turns 误差上，
           实际输出端刚度 = kp × gear_ratio / (2π) ≈ kp × 2.63（gear=16.5）。
           想让"真实刚度"就是这个数，请用 jsdk_joint_set_mit_stiffness()。 */
        jsdk_joint_set_mit(e.joint[0], pos, 0.0, 2.0, 0.2, 0.0);
        if (jsdk_context_cycle_begin(e.ctx, (uint64_t)k * e.cfg.period_ns) != JSDK_OK) break;
        if (jsdk_context_cycle_end(e.ctx) != JSDK_OK) break;
    }

    if (jsdk_joint_get_feedback(e.joint[0], &fb) == JSDK_OK) {
        printf("  feedback: online=%d age=%u ms pos=%.4f rad vel=%.4f rad/s\n",
               fb.online, (unsigned)fb.age_ms, fb.pos, fb.vel);
        printf("            err=0x%X hb_err=0x%X tx_frames=%u rejected=%u\n",
               (unsigned)fb.err_code, (unsigned)fb.hb_error,
               (unsigned)fb.tx_frames, (unsigned)fb.tx_rejected);
    }

    /* --- 失能：先发安全帧 + 等 2 周期，再 STOP_MOTOR（SDK 内部做完） --- */
    printf("  deactivate...\n");
    jsdk_context_deactivate(e.ctx);
    printf("  enabled : %d\n", jsdk_joint_is_enabled(e.joint[0]));

    ex_close(&e);
    printf("done\n");
    return 0;
}
