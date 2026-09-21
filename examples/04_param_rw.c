/**
 * @file    04_param_rw.c
 * @brief   按名字读写参数（端点 SDO）：类型化读、8 字节读、批量读、写回读
 *
 * 学到的五件事：
 *   1. 参数**按路径字符串**访问，不按数字 ID —— ID 跨固件版本会漂移（实测 86%），
 *      路径不会；
 *   2. `jsdk_joint_param_get_*()` 系列直接给你 C 类型；写用 `_set_*`；
 *   3. **8 字节端点**（u64/i64/f64）在 Classic 上会被 SDK 自动分段读
 *      （设备侧 `ReqLen` 每块最多 4），你不需要关心链路类型；
 *   4. 批量读（`_batch`）在 FD 上只占 **1 帧**，Classic 上自动退化；
 *   5. 写参数**必须** `save` 才落 Flash，否则重启即丢。
 *
 * 运行：
 *     ./build/examples/04_param_rw
 */

#include "ex_common.h"

int main(void)
{
    ex_ctx_t        e;
    const unsigned  nodes[1] = { 1u };
    jsdk_joint_t   *j;
    jsdk_value_t    v;
    float           gear = 0.0f;
    uint32_t        node_id = 0u;
    uint32_t        hb_ms = 0u;
    jsdk_status_t   st;

    printf("=== 04 参数读写 ===\n");

    ex_open(&e, "0:id=1,gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=10,timeout=30000,fd",
            nodes, 1u, 2000u);
    j = e.joint[0];

    /* --- 类型化读：直接给 C 类型，省掉一次手工解字节序 --- */
    ex_check("get_f32 gear_ratio", jsdk_joint_param_get_f32(j, "axis0.motor.config.gear_ratio", &gear));
    printf("  gear_ratio = %.3f\n", (double)gear);

    ex_check("get_u32 node_id",    jsdk_joint_param_get_u32(j, "axis0.config.can.node_id", &node_id));
    printf("  node_id    = %u\n", (unsigned)node_id);

    ex_check("get_u32 hb_rate",    jsdk_joint_param_get_u32(j, "axis0.config.can.heartbeat_rate_ms", &hb_ms));
    printf("  hb_rate    = %u ms\n", (unsigned)hb_ms);

    /* --- 标签读：`jsdk_value_t` 自带类型，适合"路径由配置给"的场景 --- */
    if (jsdk_joint_param_get(j, "axis0.controller.config.mit_max_torque", &v) == JSDK_OK) {
        printf("  tau_max    = %.3f (type enum %d)\n", (double)v.v.f32, (int)v.type);
    }

    /* --- 8 字节端点：Classic 链路上 SDK 会自动分块读（无需客户处理） --- */
    st = jsdk_joint_param_get(j, "serial_number", &v);
    if (st == JSDK_OK) {
        printf("  serial     = %llu (u64，8 字节；Classic 下自动分块)\n",
               (unsigned long long)v.v.u64);
    } else {
        printf("  serial     : %s（%s）\n", jsdk_status_string(st),
               jsdk_context_last_error(e.ctx));
    }

    /* --- 批量读：FD 上一条帧搞定多个端点 --- */
    {
        jsdk_param_req_t reqs[4];
        unsigned i;
        const char *paths[4] = {
            "axis0.motor.config.gear_ratio",
            "axis0.controller.config.mit_max_pos",
            "axis0.controller.config.mit_max_vel",
            "axis0.controller.config.mit_max_torque"
        };

        memset(reqs, 0, sizeof reqs);
        for (i = 0u; i < 4u; ++i) reqs[i].path = paths[i];

        st = jsdk_joint_param_get_batch(j, reqs, 4u);
        printf("  batch_read : %s\n", jsdk_status_string(st));
        for (i = 0u; i < 4u; ++i) {
            printf("    %-42s %s\n", reqs[i].path,
                   (reqs[i].status == JSDK_OK) ? "ok" : jsdk_status_string(reqs[i].status));
        }
    }

    /* --- 写：改一个"不影响运动"的参数，写完读回校验，再 save --- */
    st = jsdk_joint_param_set_u32(j, "axis0.config.can.heartbeat_rate_ms", 50u);
    printf("  set hb=50  : %s\n", jsdk_status_string(st));
    if (st == JSDK_OK) {
        uint32_t back = 0u;
        if (jsdk_joint_param_get_u32(j, "axis0.config.can.heartbeat_rate_ms", &back) == JSDK_OK) {
            printf("  读回校验   : %s（要的就是这个：写完必须读回，别信'已发送'）\n",
                   (back == 50u) ? "一致 [OK]" : "不一致 [FAIL]");
        }
    }

    /* --- 落 Flash：不 save 的话重启就回到旧值 --- */
    ex_check("save_config", jsdk_joint_save_config(j));

    ex_close(&e);
    printf("done\n");
    return 0;
}
