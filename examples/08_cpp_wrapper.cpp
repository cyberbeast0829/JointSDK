/**
 * @file    08_cpp_wrapper.cpp
 * @brief   C++ 客户：用 `jsdk::Group` / `jsdk::Joint`（RAII）跑一遍完整流程
 *
 * 这是唯一一个 C++ 示例，演示 `include/joint_sdk/joint_group.hpp`：
 *   - RAII：`Group` 析构时自动 `deactivate()`（发安全帧后停电机）；
 *   - 异常：配置阶段失败抛 `jsdk::Error`（带状态码 + `last_error()` 文本）；
 *   - 周期内不抛：`run()` 返回状态码，由调用者决定（与 C API 语义一致）；
 *   - 类型安全读参数：`j.param<float>("...")`。
 *
 * ⚠ 包装层**不拥有传输层**：`jsdk_hal_handle_t` 由你自己关（本例在最后一行）。
 *   这是有意的 —— 客户常常把句柄做成全局单例，包装层不该替它决定生死。
 *
 * 运行：
 *     ./build/examples/08_cpp_wrapper
 */

#include <cstdio>
#include <stdexcept>

#include "joint_group.hpp"        /* 只依赖 joint_sdk.h */
#include "jsdk_hal_builtin.h"     /* 内置虚拟后端（真机换成 socketcan/slcan/自实现） */

int main()
{
    jsdk_can_hal_t     hal;
    jsdk_hal_handle_t *hh = nullptr;

    printf("=== 08 C++ 包装（RAII + 异常）===\n");

    /* --- 传输层：这里用虚拟后端，真机上换掉这两行即可 ------------------- */
    if (jsdk_hal_virtual_open(&hal, &hh,
                              "0:id=1,gear=16.5,pmax=12.5,vmax=65,tmax=50,"
                              "hb=10,timeout=30000,fd") != JSDK_OK) {
        fprintf(stderr, "hal_virtual_open 失败\n");
        return 1;
    }
    jsdk_hal_virtual_set_autotick(hh, 1);     /* 仿真：让阻塞式配置 API 能跑完 */

    try {
        jsdk::Group ctx(hal);                 /* 默认配置（100 Hz / FD / 自动 arena） */

        jsdk::Joint &j = ctx.add_joint(1);
        printf("  arena    : %lu 字节（RETAIN_ALL 的估算值，由包装层自管）\n",
               (unsigned long)ctx.arena_size());

        ctx.desc_fetch();
        ctx.configure();                      /* 读量程 + 校验控制周期 */
        ctx.activate();                       /* 阻塞到使能序列（含安全首帧）走完 */

        printf("  enabled  : %d\n", j.enabled() ? 1 : 0);
        printf("  gear     : %.3f\n", (double)j.param<float>("axis0.motor.config.gear_ratio"));
        printf("  tau_max  : %.3f N·m\n",
               (double)j.param<float>("axis0.controller.config.mit_max_torque"));
        printf("  node_id  : %u\n",
               (unsigned)j.param<uint32_t>("axis0.config.can.node_id"));

        /* --- 控制循环：把"每周期做什么"写成一个 lambda ------------------ */
        jsdk_status_t st = ctx.run(200, [&](unsigned k) {
            double pos = 0.3 * (double)(k % 100u) / 100.0;      /* 0 → 0.3 rad 锯齿 */

            /* kp/kd 是线上值；想要"真实输出端刚度"就用 set_mit_stiffness() */
            j.set_mit(pos, 0.0, 2.0, 0.2, 0.0);
        });
        if (st != JSDK_OK) {
            fprintf(stderr, "  控制循环中断：%s\n", jsdk_status_string(st));
        }

        jsdk_joint_feedback_t fb = j.feedback();
        printf("  feedback : online=%d age=%u ms pos=%.4f rad vel=%.4f rad/s\n",
               fb.online, (unsigned)fb.age_ms, fb.pos, fb.vel);

        jsdk_bus_state_t bs = ctx.bus_state();
        printf("  bus      : tx=%u rx=%u keepalive=%u link_errors=%u\n",
               (unsigned)bs.tx_frames, (unsigned)bs.rx_frames,
               (unsigned)bs.keepalive_sent, (unsigned)bs.link_errors);

        jsdk_desc_info_t di = ctx.desc_info();
        printf("  desc     : endpoints=%u frames=%u complete=%d\n",
               di.endpoint_count, di.frames_rx, (int)di.complete);

        std::vector<jsdk::Endpoint> eps = ctx.endpoints();
        printf("  端点列表 : %lu 个（前 3 个）\n", (unsigned long)eps.size());
        for (size_t i = 0u; i < 3u && i < eps.size(); ++i) {
            printf("    %-40s id=%u type=%d access=0x%X\n",
                   eps[i].path.c_str(), (unsigned)eps[i].id,
                   (int)eps[i].type, (unsigned)eps[i].access);
        }

        /* 退出作用域 → ~Group() 里自动 deactivate()（发安全帧 + 等 2 周期 + STOP） */
        printf("  即将析构 Group（自动 deactivate）...\n");
    } catch (const jsdk::Error &e) {
        fprintf(stderr, "  jsdk::Error: %s（code=%d）\n", e.what(), (int)e.code());
        jsdk_hal_close(hh);
        return 1;
    } catch (const std::exception &e) {
        fprintf(stderr, "  std::exception: %s\n", e.what());
        jsdk_hal_close(hh);
        return 1;
    }

    jsdk_hal_close(hh);     /* ⚠ 传输层由客户自己关（包装层不拥有句柄） */
    printf("done\n");
    return 0;
}
