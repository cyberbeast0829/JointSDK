/**
 * @file    06_group_broadcast.c
 * @brief   4 个关节一条帧同步驱动（广播同步）—— 以及它什么时候**自动降级**
 *
 * 学到的六件事：
 *   1. 广播只在 `cycle_begin()`/`cycle_end()` **之间**调用才生效：
 *      否则 `cycle_end()` 会给每个关节补一条单播，N 关节变 N+1 帧；
 *   2. 组内 `node_id` 必须 ∈ 1..7（位掩码寻址的上限）；
 *   3. 成员必须**已使能 + 已标定 + 当前是 MIT 模式** —— 一条 MIT 广播会把
 *      正在跑 CSP 的设备**悄悄切到 MIT 输入模式**，所以 SDK 直接拒绝；
 *   4. FD 上未被使用的槽位会被显式写成"零增益指令"。
 *      **绝不能**是全零字节：MIT 载荷的 0 是各字段的**最小值**
 *      （tau → −tau_max），全零 = 满力矩反向；
 *   5. 降级是**正常结果**（返回仍是 OK），原因在 `last_error()` 里 ——
 *      建议周期性地打印一次，否则客户会以为还在同步广播；
 *   6. 只驱动一个关节时**别用广播**：单播有响应帧，广播没有。
 *
 * 运行：
 *     ./build/examples/06_group_broadcast
 */

#include "ex_common.h"

int main(void)
{
    ex_ctx_t                e;
    const unsigned          nodes[4] = { 1u, 2u, 3u, 4u };
    jsdk_group_target_t     tgts[4];
    jsdk_joint_feedback_t   fb;
    unsigned                k;
    unsigned                i;
    jsdk_status_t           st;

    printf("=== 06 广播同步（4 关节，一条 FD 帧）===\n");

    ex_open(&e, "0:id=1,gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=10,timeout=30000,fd;"
                "1:id=2,gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=10,timeout=30000,fd;"
                "2:id=3,gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=10,timeout=30000,fd;"
                "3:id=4,gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=10,timeout=30000,fd",
            nodes, 4u, 2000u);

    /* --- 先让 4 个关节都使能（组使能 = 逐个单播扇形展开，协议没有组使能帧） ---
       ⚠ `group_enable()` 内部是**非阻塞**的 `request_enable()`：它只是把请求排队，
         必须继续跑控制周期才会走完使能序列（实测约 5~10 个周期）。
         所以这里必须先跑几个周期，再发起广播 —— 否则会得到
         `bad-state: joint 0 is not enabled`。 */
    {
        uint8_t ids[4] = { 1u, 2u, 3u, 4u };
        unsigned spin;

        ex_check("group_enable", jsdk_group_enable(e.ctx, ids, 4u));

        for (spin = 0u; spin < 40u; ++spin) {
            unsigned j;
            int all = 1;
            for (j = 0u; j < 4u; ++j) {
                if (!jsdk_joint_is_enabled(e.joint[j])) all = 0;
            }
            if (all) break;
            ex_run_cycles(&e, 1u);
        }
    }
    printf("  enabled  : %d %d %d %d（非阻塞请求 + 跑周期后才为真）\n",
           jsdk_joint_is_enabled(e.joint[0]), jsdk_joint_is_enabled(e.joint[1]),
           jsdk_joint_is_enabled(e.joint[2]), jsdk_joint_is_enabled(e.joint[3]));

    /* --- 目标：4 个关节各自走到不同角度（FD 才能做到"每槽不同"） --- */
    for (i = 0u; i < 4u; ++i) {
        memset(&tgts[i], 0, sizeof tgts[i]);
        tgts[i].node_id = (uint8_t)nodes[i];
        tgts[i].kp      = 2.0;
        tgts[i].kd      = 0.2;
    }

    {
        uint32_t tx_before = 0u;
        jsdk_bus_state_t bus;

        (void)jsdk_context_get_bus_state(e.ctx, &bus);
        tx_before = bus.tx_frames;

        for (k = 0u; k < 20u; ++k) {
            double a = 0.1 * (double)k;

            for (i = 0u; i < 4u; ++i) {
                tgts[i].pos_rad = a + 0.02 * (double)i;   /* 每个关节差一点点 */
            }

            (void)jsdk_context_cycle_begin(e.ctx, (uint64_t)k * e.cfg.period_ns);

            /* ⚠ 就在 begin 与 end 之间：这一条就是"同步"的全部秘密 */
            st = jsdk_group_set_mit(e.ctx, tgts, 4u);
            if (k == 0u) {
                printf("  broadcast: %s（%s）\n", jsdk_status_string(st),
                       jsdk_context_last_error(e.ctx));
            }

            (void)jsdk_context_cycle_end(e.ctx);
        }

        (void)jsdk_context_get_bus_state(e.ctx, &bus);
        printf("  20 个周期发了 %u 帧（若每个关节单播会是 20×4=80 帧）\n",
               (unsigned)(bus.tx_frames - tx_before));
        printf("  注：把 `jsdk_group_set_mit()` 挪到 cycle_begin/end 之外，\n"
               "      同一条帧就会变成 4 条单播（体验一下再改回去）。\n");
    }

    /* --- 反馈：广播**不回复**，所以反馈来自心跳与单播查询 --- */
    if (jsdk_joint_get_feedback(e.joint[0], &fb) == JSDK_OK) {
        printf("  joint 1  : online=%d age=%u ms pos=%.4f rad\n",
               fb.online, (unsigned)fb.age_ms, fb.pos);
    }

    /* --- 什么时候会降级（返回仍是 OK，但不再是同步广播）---
       - 成员 node_id ≥ 8（位掩码放不下）
       - 有成员没使能 / 没标定 / 模式不是 MIT
       - 目标超量程（不静默钳位，降级让"安全帧"策略生效）
       - Classic 链路上目标不一致（只能"全员同一目标"） */
    {
        jsdk_group_target_t bad;
        memset(&bad, 0, sizeof bad);
        bad.node_id = 8u;               /* ≥ 8：位掩码寻址不到 */
        bad.kp = 2.0; bad.kd = 0.2;
        st = jsdk_group_set_mit(e.ctx, &bad, 1u);
        printf("  node_id=8: %s（%s）\n", jsdk_status_string(st),
               jsdk_context_last_error(e.ctx));
    }

    ex_close(&e);
    printf("done\n");
    return 0;
}
