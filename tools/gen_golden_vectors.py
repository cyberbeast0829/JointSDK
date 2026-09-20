#!/usr/bin/env python3
"""生成 CYBERBEAST 帧/MIT 编解码的黄金向量（C 头文件）。

为什么要自己写参考实现，而不是直接调用 `cyberbeast_tool.py`
-----------------------------------------------------------
两个实现存在**语义差异**，直接调用会得到错的向量：

| | 固件 `can_simple.cpp` | `cyberbeast_tool.py` |
|---|---|---|
| 浮点→定点 | `(int)(...)` **截断**，**不钳位** | `round(...)` + 钳位 |

固件是权威。本工具按固件公式重写，并用 `f32()` 在每一步模拟 IEEE-754 单精度
舍入（C 的 float 运算），从而得到与固件**位级一致**的向量。

CAN ID 部分无此问题，直接复用 `cyberbeast_tool.py` 的 `make_cyberbeast_id`
作为独立实现交叉验证。

用法
----
    python tools/gen_golden_vectors.py \
        --tool ../ODrive/tools/can/cyberbeast_tool.py \
        --out  tests/data/golden_vectors.h
"""

import argparse
import importlib.util
import os
import struct
import sys
from datetime import datetime, timezone

# ---------------------------------------------------------------------------
# float32 模拟
# ---------------------------------------------------------------------------

def f32(x):
    """把 Python double 舍入为 IEEE-754 单精度（模拟 C 的 float）。"""
    return struct.unpack("<f", struct.pack("<f", x))[0]


# ---------------------------------------------------------------------------
# 固件语义的定点原语（权威：can_simple.cpp）
# ---------------------------------------------------------------------------

def fw_f2u(x, x_min, x_max, bits):
    """固件 float_to_uint：(int)((x-offset)*((float)((1<<bits)-1))/span)。无钳位。"""
    span = f32(x_max - x_min)
    offset = x_min
    if span == 0.0:
        return 0
    t = f32(x - offset)
    t = f32(t * f32(float((1 << bits) - 1)))
    t = f32(t / span)
    return int(t)          # C 的 (int) 是向零截断


def fw_u2f(x_int, x_min, x_max, bits):
    """固件 uint_to_float。"""
    span = f32(x_max - x_min)
    t = f32(float(x_int) * span)
    t = f32(t / f32(float((1 << bits) - 1)))
    return f32(t + x_min)


# ---------------------------------------------------------------------------
# SDK 语义（先钳位，再**四舍五入**）
#
# 与固件的差异是有意的：
#   * 固件 float_to_uint 用于**响应编码**，语义是截断且不钳位；
#   * SDK 只负责**命令编码**，选择四舍五入：量化误差 0.5 LSB（而非 1.0），
#     且保证 pack(unpack(b)) == b 幂等，避免回读重发的控制环漂移。
# 固件的截断语义由 fw_f2u() 单独模拟，只用于危寄对照。
# ---------------------------------------------------------------------------

import math as _math


def sdk_f2u(x, x_min, x_max, bits):
    if x != x:                       # NaN
        x = x_min
    if x < x_min:
        x = x_min
    elif x > x_max:
        x = x_max
    maxv = (1 << bits) - 1
    t = f32(x - x_min)
    t = f32(t * f32(float(maxv)))
    t = f32(t / f32(x_max - x_min))
    # 对称四舍五入（远离零）：与 C 侧 round_nonneg 一致
    v = int(_math.floor(t + 0.5)) if t >= 0.0 else -int(_math.floor(0.5 - t))
    return max(0, min(v, maxv))


def clamp_flags(pos, vel, kp, kd, tau, r):
    """按文档规则推导期望的钳位标志（不是复刻 SDK 代码，而是复述规则）。"""
    f = 0
    if pos != pos or pos < -r["pos_max"] or pos > r["pos_max"]:
        f |= 0x01
    if vel != vel or vel < -r["vel_max"] or vel > r["vel_max"]:
        f |= 0x02
    if kp != kp or kp < 0.0 or kp > r["kp_max"]:
        f |= 0x04
    if kd != kd or kd < 0.0 or kd > r["kd_max"]:
        f |= 0x08
    if tau != tau or tau < -r["tau_max"] or tau > r["tau_max"]:
        f |= 0x10
    if pos != pos or vel != vel or kp != kp or kd != kd or tau != tau:
        f |= 0x80        # CB_MIT_INVALID：出现 NaN
    return f


# ---------------------------------------------------------------------------
# MIT 打包（布局同固件 pack_mit_command）
# ---------------------------------------------------------------------------

def pack_mit(r, pos, vel, kp, kd, tau, use_raw=False):
    f = fw_f2u if use_raw else sdk_f2u
    p_int = f(pos, -r["pos_max"], r["pos_max"], 16)
    v_int = f(vel, -r["vel_max"], r["vel_max"], 12)
    kp_int = f(kp, 0.0, r["kp_max"], 12)
    kd_int = f(kd, 0.0, r["kd_max"], 12)
    t_int = f(tau, -r["tau_max"], r["tau_max"], 12)

    b = [0] * 8
    b[0] = (p_int >> 8) & 0xFF
    b[1] = p_int & 0xFF
    b[2] = (v_int >> 4) & 0xFF
    b[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F)
    b[4] = kp_int & 0xFF
    b[5] = (kd_int >> 4) & 0xFF
    b[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F)
    b[7] = t_int & 0xFF
    return b, dict(p=p_int, v=v_int, kp=kp_int, kd=kd_int, t=t_int)


def unpack_mit_cmd(r, b):
    p_int = (b[0] << 8) | b[1]
    v_int = (b[2] << 4) | (b[3] >> 4)
    kp_int = ((b[3] & 0x0F) << 8) | b[4]
    kd_int = (b[5] << 4) | (b[6] >> 4)
    t_int = ((b[6] & 0x0F) << 8) | b[7]
    return dict(
        pos=fw_u2f(p_int, -r["pos_max"], r["pos_max"], 16),
        vel=fw_u2f(v_int, -r["vel_max"], r["vel_max"], 12),
        kp=fw_u2f(kp_int, 0.0, r["kp_max"], 12),
        kd=fw_u2f(kd_int, 0.0, r["kd_max"], 12),
        tau=fw_u2f(t_int, -r["tau_max"], r["tau_max"], 12),
    )


def response_max_current(tau_max, torque_constant):
    """同固件 pack_mit_response。"""
    mc = 40.0
    if torque_constant > 0.001:
        mc = tau_max / torque_constant
        if mc > 80.0:
            mc = 80.0
    return f32(mc)


def unpack_mit_response(b, r, torque_constant=0.0385):
    max_current = response_max_current(r["tau_max"], torque_constant)
    p_int = (b[0] << 8) | b[1]
    v_int = (b[2] << 4) | (b[3] >> 4)
    c_int = (b[4] << 4) | (b[5] >> 4)
    return dict(
        pos=fw_u2f(p_int, -r["pos_max"], r["pos_max"], 16),
        vel=fw_u2f(v_int, -r["vel_max"], r["vel_max"], 12),
        current=fw_u2f(c_int, -max_current, max_current, 12),
        max_current=max_current,
        err=b[3] & 0x0F,
        mode=b[5] & 0x0F,
        motor_temp=b[6] - 50,
        mos_temp=b[7] - 50,
    )


# ---------------------------------------------------------------------------
# 向量集
# ---------------------------------------------------------------------------

# 固件出厂默认（paras.h），仅作为向量的一组量程；SDK 实际从设备读取
R_DEFAULT = dict(pos_max=12.5, vel_max=65.0, kp_max=500.0, kd_max=5.0, tau_max=50.0)
# 第二组：刻意用非整数量程，检验公式而非巧合
R_ODD = dict(pos_max=6.28318, vel_max=33.3, kp_max=200.0, kd_max=2.5, tau_max=17.7)


def build_mit_cmd_vectors():
    out = []
    for r in (R_DEFAULT, R_ODD):
        pm, vm, km, dm, tm = r["pos_max"], r["vel_max"], r["kp_max"], r["kd_max"], r["tau_max"]
        cases = [
            (0.0, 0.0, 0.0, 0.0, 0.0),                       # 全零
            (pm, vm, km, dm, tm),                            # 正满量程
            (-pm, -vm, 0.0, 0.0, -tm),                       # 负满量程（kp/kd 只能 0）
            (pm / 2, vm / 2, km / 2, dm / 2, tm / 2),        # 半量程
            (-pm / 2, -vm / 2, 0.0, 0.0, -tm / 2),
            (pm / 4, -vm / 4, km / 4, dm / 4, -tm / 4),
            (1.0, -2.0, 20.0, 0.5, 3.0),                     # 常见控制值
            (0.1 + 0.2, 0.0, 1.0 / 3.0, 1.0 / 7.0, -0.1),    # 舍入敏感
            (pm / 3, vm / 7, km / 3, dm / 7, tm / 3),
            # 越界（SDK 会钳位并置标志）
            (pm * 1.5, vm * 2.0, km * 2.0, dm * 5.0, tm * 1.5),
            (-pm * 1.5, -vm * 2.0, -1.0, -1.0, -tm * 1.5),
            (1e30, 0.0, 20.0, 0.5, 0.0),                     # 超大值
        ]
        for pos, vel, kp, kd, tau in cases:
            b, _ = pack_mit(r, pos, vel, kp, kd, tau)
            out.append((r, pos, vel, kp, kd, tau, b, clamp_flags(pos, vel, kp, kd, tau, r)))
    return out


def build_mit_hazard_vectors():
    """越界输入：固件（无钳位）会回绕；SDK 必须钳位。"""
    out = []
    r = R_DEFAULT
    pm, tm = r["pos_max"], r["tau_max"]
    for pos, tau in [(pm * 1.01, 0.0), (pm * 1.5, 0.0), (pm * 100.0, 0.0),
                     (0.0, tm * 1.01), (0.0, tm * 3.0)]:
        raw_p = fw_f2u(pos, -pm, pm, 16)
        raw_t = fw_f2u(tau, -tm, tm, 12)
        out.append(dict(
            pos=pos, tau=tau,
            raw_p=raw_p, raw_t=raw_t,
            wrapped_p=raw_p & 0xFFFF,
            wrapped_t=raw_t & 0xFFF,
            p_overflow=1 if not (0 <= raw_p <= 0xFFFF) else 0,
            t_overflow=1 if not (0 <= raw_t <= 0xFFF) else 0,
        ))
    return out


def build_mit_resp_vectors():
    out = []
    r = R_DEFAULT
    cases = [
        ([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x32], 0x0, 0x0),   # 全零，温度 0
        ([0xFF, 0xFF, 0xFF, 0xF0, 0xFF, 0xF0, 0x00, 0xFF], 0x0, 0x0),   # 边界
        ([0x80, 0x00, 0x7F, 0xF7, 0x7F, 0xF4, 0x82, 0xFF], 0x7, 0x4),   # 中点附近 + 温度极值
        ([0xC0, 0x00, 0x00, 0x01, 0x00, 0x08, 0x00, 0xC8], 0x1, 0x8),   # 错误码/模式
        ([0x40, 0x00, 0x00, 0x0F, 0x00, 0x0F, 0xFF, 0x00], 0xF, 0xF),   # ERR_MULTIPLE
    ]
    for b, _, _ in cases:
        out.append((b, unpack_mit_response(b, r)))
    return out


def build_id_vectors(make_id):
    out = []
    for pri, mt, dest, src, seq in [
        # 以下四条与 docs/cyberbeast-protocol.md §2.4 的示例**逐字段一致**，
        # 用于和文档对拍（⚠ 文档示例 4 的十六进制结果是笔误，见下）
        (2, 0x00, 0x05, 1, 1),       # 示例1 MIT 单播  → 0x08001405
        (2, 0x80, 0xFF, 1, 2),       # 示例2 MIT 广播  → 0x0A03FC06
        (6, 0x48, 0x01, 3, 0),       # 示例3 心跳      → 0x1920040C
        (0, 0xC0, 0xFF, 1, 0),       # 示例4 ESTOP     → 0x0303FC04
                                     #   （文档写 0x003FFC04，反解得 msgtype=0x0F，
                                     #    与示例自身的 (0xC0<<18) 矛盾，是文档笔误）
        (0, 0x00, 0, 0, 0),          # 全零边界
        (7, 0xFF, 0xFF, 0xFF, 3),    # 各字段全满
        (5, 0x41, 200, 100, 1),
        (4, 0x25, 7, 250, 2),
    ]:
        out.append((pri, mt, dest, src, seq, make_id(pri, mt, dest, src, seq)))
    return out


def build_addressing_vectors():
    """(my_node_id, can_id, expected_is_for_me)

    严格复刻固件 Firmware/communication/can/can_cyberbeast.cpp 的
    CANCyberBeast::is_message_for_me()：

        if (my_id == 0) return false;
        if (is_broadcast(can_id)) {
            if (dest == ADDR_BROADCAST) return true;   // 0xFF 优先，含 node_id >= 8
            if (my_id >= MAX_BROADCAST_DEVICES) return false;
            return (dest & (1 << my_id)) != 0;         // 位位置 = node_id
        }
        return (dest == my_id);

    注意：dest == 0xFF 是**全局广播**，不是“位图 0xFF”。因此没法用位图
    只找 1..7 而排除 node_id >= 8；那需要 dest = 0xFE（bit0 自然为 0）。
    """
    def mid(pri, mt, dest, src, seq=0):
        return (((pri & 7) << 26) | ((mt & 0xFF) << 18) |
                ((dest & 0xFF) << 10) | ((src & 0xFF) << 2) | (seq & 3))

    UNI = 0x00          # 点对点类型
    BC  = 0x80          # 广播类型
    EMR = 0xC0          # 紧急广播类型
    ALL = 0xFE          # 位图：device 1..7 全选（bit0 恒 0）

    cases = []
    cases += [(5, mid(2, UNI, 5, 1), 1), (5, mid(2, UNI, 6, 1), 0)]

    # node_id = 0 → 永不匹配（单播与广播都不行）
    cases += [(0, mid(2, UNI, 0, 1), 0), (0, mid(0, EMR, 0xFF, 1), 0),
              (0, mid(2, BC, 0x02, 1), 0)]

    # Dest == 0xFF：全局广播 → 任何合法 node_id 都匹配，**包括 >= 8**
    for n in (1, 7, 8, 9, 254):
        cases.append((n, mid(0, EMR, 0xFF, 1), 1))
        cases.append((n, mid(2, BC, 0xFF, 1), 1))

    # 位图广播（dest != 0xFF）：只有 1..7 可被逐位寻址
    for n in (1, 3, 7):
        cases.append((n, mid(2, BC, 1 << n, 1), 1))
        cases.append((n, mid(2, BC, ALL ^ (1 << n), 1), 0))
        cases.append((n, mid(2, BC, 0x00, 1), 0))      # Dest=0 → 谁都不匹配

    # node_id >= 8 遇位图 → 一律不匹配
    for n in (8, 9, 254):
        cases.append((n, mid(2, BC, ALL, 1), 0))
        cases.append((n, mid(2, BC, 0x02, 1), 0))

    # 位图 0xFE = device 1..7 全选
    for n in (1, 4, 7):
        cases.append((n, mid(2, BC, ALL, 1), 1))

    # 广播阈值边界：0x7F 走单播分支，0x80 走广播分支
    cases += [(5, mid(2, 0x7F, 5, 1), 1), (5, mid(2, 0x7F, 6, 1), 0),
              (5, mid(2, 0x80, 5, 1), 0)]
    return cases


# ---------------------------------------------------------------------------
# 输出 C 头文件
# ---------------------------------------------------------------------------

def fnum(x):
    """按 float 精度输出字面量，并保证是合法的 C 浮点写法。"""
    if x != x:
        return "(0.0f/0.0f)"          # NaN
    if x == float("inf"):
        return "(1.0f/0.0f)"
    if x == float("-inf"):
        return "(-1.0f/0.0f)"
    s = repr(f32(x))
    if "e" not in s and "E" not in s and "." not in s:
        s += ".0"
    return s + "f"


def u8(b):
    return ", ".join("0x%02X" % v for v in b)


def emit(path, make_id, tool_path):
    L = []
    A = L.append
    A("/*")
    A(" * 自动生成 —— 请勿手工编辑。")
    A(" * 生成器：tools/gen_golden_vectors.py")
    A(" * 生成时间：%s" % datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%SZ"))
    A(" *")
    A(" * 语义基准：固件 can_simple.cpp 的 float_to_uint（截断、不钳位）")
    A(" * 交叉验证：CAN ID 用 %s 独立实现对照" % os.path.basename(tool_path))
    A(" */")
    A("")
    A("#ifndef JSDK_GOLDEN_VECTORS_H")
    A("#define JSDK_GOLDEN_VECTORS_H")
    A("")
    A("#include <stdint.h>")
    A("")

    # --- CAN ID ---
    ids = build_id_vectors(make_id)
    A("/* ---- CAN ID ---- */")
    A("typedef struct { uint8_t pri, msgtype, dest, src, seq; uint32_t expected; } gv_id_t;")
    A("static const gv_id_t gv_ids[] = {")
    for pri, mt, dest, src, seq, exp in ids:
        A("    { %d, 0x%02X, %d, %d, %d, 0x%08Xu }," % (pri, mt, dest, src, seq, exp))
    A("};")
    A("#define GV_ID_COUNT %d" % len(ids))
    A("")

    # --- 寻址 ---
    addr = build_addressing_vectors()
    A("/* ---- 寻址判定（my_node_id, can_id, expected） ---- */")
    A("typedef struct { uint8_t my_node_id; uint32_t id; uint8_t expected; } gv_addr_t;")
    A("static const gv_addr_t gv_addr[] = {")
    for nid, cid, exp in addr:
        A("    { %d, 0x%08Xu, %d }," % (nid, cid, exp))
    A("};")
    A("#define GV_ADDR_COUNT %d" % len(addr))
    A("")

    # --- MIT 命令 ---
    cmds = build_mit_cmd_vectors()
    A("/* ---- MIT 命令打包（含量程） ---- */")
    A("typedef struct {")
    A("    float pos_max, vel_max, kp_max, kd_max, tau_max;")
    A("    float pos, vel, kp, kd, tau;")
    A("    uint8_t bytes[8];")
    A("    uint8_t clamped;")
    A("} gv_mit_cmd_t;")
    A("static const gv_mit_cmd_t gv_mit_cmds[] = {")
    for r, pos, vel, kp, kd, tau, b, cl in cmds:
        A("    { %s, %s, %s, %s, %s," % (fnum(r["pos_max"]), fnum(r["vel_max"]),
                                        fnum(r["kp_max"]), fnum(r["kd_max"]), fnum(r["tau_max"])))
        A("      %s, %s, %s, %s, %s," % (fnum(pos), fnum(vel), fnum(kp), fnum(kd), fnum(tau)))
        A("      { %s }, 0x%02X }," % (u8(b), cl))
    A("};")
    A("#define GV_MIT_CMD_COUNT %d" % len(cmds))
    A("")

    # --- 解包期望值 ---
    A("/* ---- MIT 命令解包期望值（由上面的 bytes 反解，用于往返校验） ---- */")
    A("typedef struct { float pos, vel, kp, kd, tau; } gv_mit_val_t;")
    A("static const gv_mit_val_t gv_mit_cmd_vals[] = {")
    for r, pos, vel, kp, kd, tau, b, cl in cmds:
        u = unpack_mit_cmd(r, b)
        A("    { %s, %s, %s, %s, %s },   /* clamped=0x%02X */" % (
            fnum(u["pos"]), fnum(u["vel"]), fnum(u["kp"]), fnum(u["kd"]), fnum(u["tau"]), cl))
    A("};")
    A("#define GV_MIT_CMD_VAL_COUNT %d" % len(cmds))
    A("")

    # --- 越界危害 ---
    hz = build_mit_hazard_vectors()
    A("/* ---- 越界危害：固件（无钳位）会回绕，SDK 必须钳位 ---- */")
    A("typedef struct {")
    A("    float pos_max, tau_max;")
    A("    float pos, tau;")
    A("    int32_t raw_p, raw_t;      /* 固件语义得到的定点（可能超位宽） */")
    A("    uint16_t wrapped_p;        /* 掩码回绕后的 16 位位置值 */")
    A("    uint16_t wrapped_t;        /* 掩码回绕后的 12 位力矩值 */")
    A("    uint8_t p_overflow, t_overflow;")
    A("} gv_mit_hazard_t;")
    A("static const gv_mit_hazard_t gv_mit_hazard[] = {")
    for h in hz:
        A("    { %s, %s, %s, %s, %d, %d, 0x%04X, 0x%03X, %d, %d }," % (
            fnum(R_DEFAULT["pos_max"]), fnum(R_DEFAULT["tau_max"]),
            fnum(h["pos"]), fnum(h["tau"]), h["raw_p"], h["raw_t"],
            h["wrapped_p"], h["wrapped_t"], h["p_overflow"], h["t_overflow"]))
    A("};")
    A("#define GV_MIT_HAZARD_COUNT %d" % len(hz))
    A("")

    # --- MIT 响应 ---
    rsps = build_mit_resp_vectors()
    A("/* ---- MIT 响应解包 ---- */")
    A("typedef struct {")
    A("    uint8_t bytes[8];")
    A("    float pos, vel, current, max_current;")
    A("    uint8_t err, mode;")
    A("    int16_t motor_temp, mos_temp;")
    A("} gv_mit_resp_t;")
    A("static const gv_mit_resp_t gv_mit_resps[] = {")
    for b, u in rsps:
        A("    { { %s }," % u8(b))
        A("      %s, %s, %s, %s," % (fnum(u["pos"]), fnum(u["vel"]),
                                      fnum(u["current"]), fnum(u["max_current"])))
        A("      %d, %d, %d, %d }," % (u["err"], u["mode"], u["motor_temp"], u["mos_temp"]))
    A("};")
    A("#define GV_MIT_RESP_COUNT %d" % len(rsps))
    A("")

    # --- max_current 表 ---
    A("/* ---- 响应电流满量程（tau_max, torque_constant, expected） ---- */")
    A("typedef struct { float tau_max, torque_constant, expected; } gv_maxcur_t;")
    A("static const gv_maxcur_t gv_maxcur[] = {")
    maxcur = [(50.0, 0.0385), (50.0, 0.5), (18.0, 0.05), (2.0, 0.05),
              (50.0, 0.0), (50.0, 0.001), (12.5, 0.02)]
    for tau, tc in maxcur:
        A("    { %s, %s, %s }," % (fnum(tau), fnum(tc),
                                    fnum(response_max_current(tau, tc))))
    A("};")
    A("#define GV_MAXCUR_COUNT %d" % len(maxcur))
    A("")
    A("#endif /* JSDK_GOLDEN_VECTORS_H */")

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(L) + "\n")

    print("golden vectors written: %s" % path)
    print("  ids        : %d" % len(ids))
    print("  addr       : %d" % len(addr))
    print("  mit cmds   : %d" % len(cmds))
    print("  mit hazard : %d" % len(hz))
    print("  mit resp   : %d" % len(rsps))
    print("  max current: %d" % 7)


def load_tool(path):
    spec = importlib.util.spec_from_file_location("cbt", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tool", default="../ODrive/tools/can/cyberbeast_tool.py",
                    help="参考实现路径（仅用于 CAN ID 交叉验证）")
    ap.add_argument("--out", default="tests/data/golden_vectors.h")
    args = ap.parse_args()

    if not os.path.isfile(args.tool):
        print("error: 找不到参考实现 %s" % args.tool, file=sys.stderr)
        return 2

    mod = load_tool(args.tool)
    emit(args.out, mod.make_cyberbeast_id, args.tool)
    return 0


if __name__ == "__main__":
    sys.exit(main())
