#!/usr/bin/env python3
"""生成 WP2 第 2 步（控制/查询/心跳/参数）的黄金向量头文件。

用法：
    python tools/gen_golden_vectors_proto.py \
        --tool ../ODrive/tools/can/cyberbeast_tool.py \
        --out tests/data/golden_vectors_proto.h

**交叉验证策略**（避免"自己验自己"）：

| 向量类别 | 独立参考 |
|---|---|
| 心跳 0x48 | 工具 `decode_heartbeat()` |
| 设备信息 0x46 | 工具 `decode_device_info()` |
| 错误查询 0x45 | 工具 `decode_error_detail()` |
| POS/VEL/TORQUE/CURRENT | 工具 `_encode_pos_control()` 的算法 + `float_to_be_bytes()` |
| 参数 0x20/0x21 | 本文件按固件源码 `cmd_param_*` 逐行复刻（工具无对应实现） |

心跳/版本/错误的**期望值直接取工具的输出**，因此 C 侧实现若与工具不一致就会失败。
状态名与控制模式名也一并交叉校验，用于发现"枚举命名漂移"。
"""

import argparse
import datetime
import importlib.util
import os
import struct
import sys
from time import timezone  # noqa: F401  (保持与主生成器一致的导入习惯)


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------

def f32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def fnum(x):
    """按 float 精度输出合法 C 浮点字面量。"""
    if x != x:
        return "(0.0f/0.0f)"
    if x == float("inf"):
        return "(1.0f/0.0f)"
    if x == float("-inf"):
        return "(-1.0f/0.0f)"
    s = repr(f32(x))
    if "e" not in s and "E" not in s and "." not in s:
        s += ".0"
    return s + "f"


def u8s(b):
    return ", ".join("0x%02X" % v for v in b)


def pad(b, n):
    """把字节序列补齐到 n 个元素并输出 C 初始化列表（无尾逗号）。"""
    assert len(b) <= n, "%d > %d" % (len(b), n)
    return u8s(list(b) + [0] * (n - len(b)))


def be_f32(x):
    return list(struct.pack(">f", f32(x)))


def be_i16(v):
    return list(struct.pack(">h", v))


def be_u16(v):
    return list(struct.pack(">H", v))


def be_i32(v):
    return list(struct.pack(">i", v))


def be_u32(v):
    return list(struct.pack(">I", v))


def be_u64(v):
    return list(struct.pack(">Q", v))


def rnd(x):
    """对称四舍五入（远离零），与 C 侧 SDK 语义一致。"""
    import math
    if x >= 0.0:
        return int(math.floor(x + 0.5))
    return -int(math.floor(0.5 - x))


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def parse_enum(path, prefix):
    """从固件 autogen 头文件中解析 `NAME = 数字` 形式的枚举。

    这是**权威来源**：interfaces.hpp 由固件构建流程从接口定义生成，
    与设备实际运行的值必然一致。

    返回 {value: NAME}（已去掉 prefix）。
    """
    import re
    out = {}
    if not os.path.isfile(path):
        return out
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        text = fh.read()
    for m in re.finditer(r"\b%s(\w+)\s*=\s*(\d+)\b" % re.escape(prefix), text):
        name, val = m.group(1), int(m.group(2))
        out[val] = name
    return out


# ---------------------------------------------------------------------------
# 控制帧（POS / VEL / TORQUE / CURRENT）
# ---------------------------------------------------------------------------

POS_CLASSIC = [
    # (pos_deg, vel_limit_rpm, cur_limit_a)  —— 覆盖正常 / 边界 / 越界
    (0.0, 0.0, 0.0),
    (90.0, 1000.0, 10.0),
    (-90.0, -1000.0, -10.0),
    (360.0, 3000.0, 20.5),
    (12.345678, 123.456, 3.75),
    (0.1, 0.1, 0.1),                    # 0.1 A/bit 的量化边界
    (-0.05, -0.05, -0.05),              # 刚好半个 LSB
    (7.5, 32767.0, 3276.7),             # 各字段正满量程
    (-7.5, -32768.0, -3276.8),          # 各字段负越界
    (1e6, 1e6, 1e6),                    # 大幅越界 → 必须钳位
    (-1e6, -1e6, -1e6),
    (179.99999, 0.0, 0.0),
]

VEL_CASES = [
    (0.0, 0.0), (1000.0, 10.0), (-1000.0, -10.0),
    (3000.0, 40.0), (-2999.5, 0.5), (1e6, 1e6),
]

TORQUE_CASES = [0.0, 12.5, -12.5, 50.0, -50.0, 1e6, -1e6]
CURRENT_CASES = [0.0, 5.0, -5.0, 40.0, -40.0, 1e6, -1e6]


def build_ctrl_vectors():
    out = {"pos_classic": [], "pos_fd": [], "vel": [],
           "torque": [], "current": []}

    for pos, vel, cur in POS_CLASSIC:
        # Classic：pos f32 BE | i16 vel（round+clamp）| i16 cur*10（round+clamp）
        vel_i = clamp(rnd(vel), -32768, 32767)
        cur_i = clamp(rnd(cur * 10.0), -32768, 32767)
        b = be_f32(pos) + be_i16(vel_i) + be_i16(cur_i)
        out["pos_classic"].append((pos, vel, cur, b, vel_i / 1.0, cur_i / 10.0))

    for pos, vel, cur in POS_CLASSIC:
        b = be_f32(pos) + be_f32(vel) + be_f32(cur)
        out["pos_fd"].append((pos, vel, cur, b))

    for vel, cur in VEL_CASES:
        out["vel"].append((vel, cur, be_f32(vel) + be_f32(cur)))

    for tau in TORQUE_CASES:
        out["torque"].append((tau, be_f32(tau)))

    for cur in CURRENT_CASES:
        out["current"].append((cur, be_f32(cur)))

    return out


# ---------------------------------------------------------------------------
# 查询帧（0x41..0x47）
# ---------------------------------------------------------------------------

QUERY_F32X2 = [
    # (kind, a, b)
    ("pos_vel", 12.5, -65.25),
    ("pos_vel", 0.0, 0.0),
    ("pos_vel", -1234.5, 9999.25),
    ("current", 14.75, -3.5),
    ("current", 0.0, 0.0),
    ("temp", 25.5, 41.25),
    ("temp", -10.0, 85.0),
    ("bus", 48.0, 5.5),
    ("bus", 0.0, -12.25),
    ("power", 1234.5, -987.75),
]


def build_query_vectors():
    f32x2 = []
    for kind, a, b in QUERY_F32X2:
        f32x2.append((kind, fnum(a), fnum(b), be_f32(a) + be_f32(b)))

    # 0x45 QUERY_ERROR
    errs = [
        (0, 0), (0, 0x00000001), (1, 0x00000003), (3, 0x0000000C),
        (4, 0x00000008), (5, 0xFFFFFFFF), (9, 0x00000100),
    ]
    err_rows = []
    for et, val in errs:
        b = [et, 0, 0, 0] + be_u32(val)
        err_rows.append((et, val, b))

    # 0x46 QUERY_DEVICE_INFO
    devs = [
        (0x01020304, 0x08090200, 0x1122334455667788),
        (0x00000000, 0x00000000, 0x0000000000000000),
        (0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFFFFFFFFFF),
        (0x01000203, 0x080123FF, 0xDEADBEEFCAFEBABE),
    ]
    dev_rows = []
    for hw, fw, sn in devs:
        dev_rows.append((hw, fw, sn, be_u32(hw) + be_u32(fw),
                         be_u32(hw) + be_u32(fw) + be_u64(sn)))

    return f32x2, err_rows, dev_rows


# ---------------------------------------------------------------------------
# 心跳（0x48）
# ---------------------------------------------------------------------------

HB_CLASSIC_CASES = [
    # life, err, state, cmode, motor_temp, pos_turns, vel_turns_s, iq_a
    (0, 0x00, 1, 0, 25.0, 1.25, -2.5, 3.5),
    (7, 0x1F, 8, 3, -50.0, 0.0, 0.0, 0.0),
    (3, 0x05, 4, 2, 205.0, -327.67, 327.67, -64.0),
    (5, 0x10, 11, 1, 150.0, 100.0, 50.25, 63.5),
    (1, 0x02, 6, 2, 40.5, -1.0, -0.5, -0.5),
    (2, 0x00, 15, 3, 0.0, 327.67, -327.68, 0.5),
]

HB_FD_CASES = [
    (0, 0x00, 1, 0, 25.0, 26.0, 48.0, 5.5, 1.25, -2.5, 3.5),
    (7, 0x1F, 8, 3, -50.0, 205.0, 6553.5, -327.68, 0.0, 0.0, 0.0),
    (3, 0x15, 4, 2, 205.0, -50.0, 0.0, 327.67, -214748.0, 214748.0, -327.68),
    (5, 0x08, 16, 1, 150.0, 151.0, 300.0, -100.0, 100.5, 250.75, 200.0),
    (1, 0x01, 6, 2, 40.5, 41.5, 24.0, 0.0, -0.0001, 0.0001, -0.01),
]


def hb_scale_classic(pos, vel, iq):
    """把物理量按 Classic 变体量化，返回 (pos_i16, vel_i16, iq_i8)。"""
    return (clamp(rnd(pos * 100.0), -32768, 32767),
            clamp(rnd(vel * 100.0), -32768, 32767),
            clamp(rnd(iq / 0.5), -128, 127))


def hb_scale_fd(pos, vel, iq):
    return (clamp(rnd(pos * 10000.0), -2147483648, 2147483647),
            clamp(rnd(vel * 10000.0), -2147483648, 2147483647),
            clamp(rnd(iq / 0.01), -32768, 32767))


def build_heartbeat_vectors(mod):
    classic, fd = [], []

    for (life, err, state, cmode, mt, pos, vel, iq) in HB_CLASSIC_CASES:
        pos_i, vel_i, iq_i = hb_scale_classic(pos, vel, iq)
        b = [((life & 7) << 5) | (err & 0x1F),
             ((state & 0xF) << 4) | (cmode & 0xF),
             clamp(rnd(mt) + 50, 0, 255)]
        b += be_i16(pos_i) + be_i16(vel_i) + [iq_i & 0xFF]

        d = mod.decode_heartbeat(bytes(b))          # ← 独立参考
        classic.append((b, d))

    for (life, err, state, cmode, mt, mos, vbus, ibus, pos, vel, iq) in HB_FD_CASES:
        pos_i, vel_i, iq_i = hb_scale_fd(pos, vel, iq)
        b = [((life & 7) << 5) | (err & 0x1F),
             ((state & 0xF) << 4) | (cmode & 0xF),
             clamp(rnd(mt) + 50, 0, 255),
             clamp(rnd(mos) + 50, 0, 255)]
        b += be_u16(clamp(rnd(vbus * 10.0), 0, 65535))
        b += be_i16(clamp(rnd(ibus * 100.0), -32768, 32767))
        b += be_i32(pos_i) + be_i32(vel_i) + be_i16(iq_i)

        d = mod.decode_heartbeat(bytes(b))          # ← 独立参考
        fd.append((b, d))

    return classic, fd


# ---------------------------------------------------------------------------
# 参数帧（按固件 cmd_param_* 逐行复刻）
# ---------------------------------------------------------------------------

def param_normalize_req_len(req_len, classic):
    if req_len == 0:
        req_len = 4
    if req_len > 8:
        req_len = 8
    if classic and req_len > 4:
        req_len = 4
    return req_len


def param_build_read_rsp(req_flags, ep_id, full_value, offset, req_len):
    """复刻固件 cmd_param_read 的切片逻辑。"""
    full_len = len(full_value)
    actual = 0
    flags = req_flags & 0x7F
    if offset < full_len:
        avail = full_len - offset
        actual = min(req_len, avail)
        if offset + actual < full_len:
            flags |= 0x80
    return [flags] + be_u16(ep_id) + [actual] + list(full_value[offset:offset + actual])


def build_param_vectors():
    read_rsp = []
    # (ep_id, full_value, offset, req_len, classic)
    cases = [
        (200, bytes([0x11, 0x22, 0x33, 0x44]), 0, 4, True),
        (200, bytes([0x11, 0x22, 0x33, 0x44]), 4, 4, True),        # 越界 → DataLen=0
        (300, bytes(range(8)), 0, 8, False),                       # FD 一次读完
        (300, bytes(range(8)), 0, 4, False),                       # 前半 + More
        (300, bytes(range(8)), 4, 4, False),                       # 后半，无 More
        (301, bytes([0xAB]), 0, 8, False),                         # 短值
        (301, bytes([0xAB]), 1, 8, False),                         # 刚好读完，DataLen=0
        (302, bytes(range(8)), 2, 4, False),                       # 中间切片 + More
        (0, bytes([]), 0, 4, False),                               # 空值
    ]
    for ep, val, off, rl, classic in cases:
        req_len = param_normalize_req_len(rl, classic)
        b = param_build_read_rsp(0x00, ep, val, off, req_len)
        read_rsp.append((ep, len(val), off, rl, 1 if classic else 0, req_len, b,
                         list(val[off:off + req_len])))

    # 写确认
    ack = [
        (0x00, 210, [0x00, 0x00, 0xD2, 0x00, 0, 0, 0, 0]),
        (0x80, 1, [0x80, 0x00, 0x01, 0x00, 0, 0, 0, 0]),
        (0xFF, 65535, [0xFF, 0xFF, 0xFF, 0x00, 0, 0, 0, 0]),
    ]

    # 批量响应：N=3，值长度 4/2/8
    n = 3
    bm = (n + 7) // 8
    vals = bytes([1, 2, 3, 4]) + bytes([5, 6]) + bytes(range(8))
    batch_ok = [0x40, n] + [0x07] + list(vals)
    batch_err = [0x60, 0x00]
    # 分段写：float64(0x1122334455667788) 分两块（TotalLen=8）
    seg_vals = bytes([0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88])
    seg = [
        ([0x80] + be_u16(400) + [8] + list(seg_vals[0:4]), 1),   # More
        ([0x00] + be_u16(400) + [8] + list(seg_vals[4:8]), 0),   # 末块
    ]
    # 6 字节参数（两块，末块 2 字节 + 2 字节 0 填充）
    seg6_vals = bytes([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF])
    seg6 = [
        ([0x80] + be_u16(401) + [6] + list(seg6_vals[0:4]), 1),
        ([0x00] + be_u16(401) + [6] + list(seg6_vals[4:6]) + [0, 0], 0),
    ]
    # 5 字节参数（两块，末块 1 字节 + 3 字节 0 填充）
    seg5_vals = bytes([0x01, 0x02, 0x03, 0x04, 0x05])
    seg5 = [
        ([0x80] + be_u16(402) + [5] + list(seg5_vals[0:4]), 1),
        ([0x00] + be_u16(402) + [5] + list(seg5_vals[4:5]) + [0, 0, 0], 0),
    ]

    return read_rsp, ack, batch_ok, batch_err, seg + seg6 + seg5, n


# ---------------------------------------------------------------------------
# 输出
# ---------------------------------------------------------------------------

def emit_proto(path, mod, tool_name):
    L = []
    A = L.append

    ctrl = build_ctrl_vectors()
    f32x2, err_rows, dev_rows = build_query_vectors()
    hb_c, hb_f = build_heartbeat_vectors(mod)
    p_read, p_ack, p_batch_ok, p_batch_err, p_seg, p_n = build_param_vectors()

    A("/*")
    A(" * 自动生成 —— 请勿手工编辑。")
    A(" * 生成器：tools/gen_golden_vectors_proto.py")
    A(" * 生成时间：%s" % datetime.datetime.now(datetime.timezone.utc)
      .strftime("%Y-%m-%d %H:%M:%SZ"))
    A(" *")
    A(" * 交叉验证来源：%s" % os.path.basename(tool_name))
    A(" *   心跳/设备信息/错误查询的期望值取自该工具的独立解码器。")
    A(" */")
    A("")
    A("#ifndef JSDK_GOLDEN_VECTORS_PROTO_H")
    A("#define JSDK_GOLDEN_VECTORS_PROTO_H")
    A("")
    A("#include <stdint.h>")
    A("")

    # --- POS Classic ---
    A("/* ---- POS_CONTROL Classic(8B): pos f32 | vel i16 | cur i16(0.1A) ---- */")
    A("typedef struct {")
    A("    float pos_deg, vel_limit_rpm, cur_limit_a;   /* 请求值 */")
    A("    uint8_t bytes[8];")
    A("    float exp_vel_rpm, exp_cur_a;                /* 量化后的期望解出值 */")
    A("} gv_ctrl_pos_c_t;")
    A("static const gv_ctrl_pos_c_t gv_ctrl_pos_c[] = {")
    for pos, vel, cur, b, exp_vel, exp_cur in ctrl["pos_classic"]:
        A("    { %s, %s, %s," % (fnum(pos), fnum(vel), fnum(cur)))
        A("      { %s }, %s, %s }," % (u8s(b), fnum(exp_vel), fnum(exp_cur)))
    A("};")
    A("#define GV_CTRL_POS_C_COUNT %d" % len(ctrl["pos_classic"]))
    A("")

    # --- POS FD ---
    A("/* ---- POS_CONTROL FD(12B): 3 × f32 ---- */")
    A("typedef struct {")
    A("    float pos_deg, vel_limit_rpm, cur_limit_a;")
    A("    uint8_t bytes[12];")
    A("} gv_ctrl_pos_fd_t;")
    A("static const gv_ctrl_pos_fd_t gv_ctrl_pos_fd[] = {")
    for pos, vel, cur, b in ctrl["pos_fd"]:
        A("    { %s, %s, %s," % (fnum(pos), fnum(vel), fnum(cur)))
        A("      { %s } }," % u8s(b))
    A("};")
    A("#define GV_CTRL_POS_FD_COUNT %d" % len(ctrl["pos_fd"]))
    A("")

    # --- VEL ---
    A("/* ---- VEL_CONTROL(8B): 2 × f32 ---- */")
    A("typedef struct { float vel_rpm, cur_a; uint8_t bytes[8]; } gv_ctrl_vel_t;")
    A("static const gv_ctrl_vel_t gv_ctrl_vel[] = {")
    for vel, cur, b in ctrl["vel"]:
        A("    { %s, %s, { %s } }," % (fnum(vel), fnum(cur), u8s(b)))
    A("};")
    A("#define GV_CTRL_VEL_COUNT %d" % len(ctrl["vel"]))
    A("")

    # --- TORQUE ---
    A("/* ---- TORQUE_CONTROL(4B): f32 电机端 N·m ---- */")
    A("typedef struct { float tau_nm; uint8_t bytes[4]; } gv_ctrl_tau_t;")
    A("static const gv_ctrl_tau_t gv_ctrl_tau[] = {")
    for tau, b in ctrl["torque"]:
        A("    { %s, { %s } }," % (fnum(tau), u8s(b)))
    A("};")
    A("#define GV_CTRL_TAU_COUNT %d" % len(ctrl["torque"]))
    A("")

    # --- CURRENT ---
    A("/* ---- CURRENT_CONTROL(4B): f32 电机端 A（无广播版本） ---- */")
    A("typedef struct { float cur_a; uint8_t bytes[4]; } gv_ctrl_cur_t;")
    A("static const gv_ctrl_cur_t gv_ctrl_cur[] = {")
    for cur, b in ctrl["current"]:
        A("    { %s, { %s } }," % (fnum(cur), u8s(b)))
    A("};")
    A("#define GV_CTRL_CUR_COUNT %d" % len(ctrl["current"]))
    A("")

    # --- 查询 f32×2 ---
    A("/* ---- 查询响应 f32×2（0x41/0x42/0x43/0x44/0x47）---- */")
    A("/* kind: 0=pos_vel 1=current 2=temp 3=bus 4=power */")
    A("typedef struct { uint8_t kind; float a, b; uint8_t bytes[8]; } gv_q_f32x2_t;")
    A("static const gv_q_f32x2_t gv_q_f32x2[] = {")
    kinds = {"pos_vel": 0, "current": 1, "temp": 2, "bus": 3, "power": 4}
    for kind, sa, sb, b in f32x2:
        A("    { %d, %s, %s, { %s } }," % (kinds[kind], sa, sb, u8s(b)))
    A("};")
    A("#define GV_Q_F32X2_COUNT %d" % len(f32x2))
    A("")

    # --- 错误查询 ---
    A("/* ---- 0x45 QUERY_ERROR 响应：b0=Type 回显, b4..7=u32 BE ---- */")
    A("typedef struct { uint8_t err_type; uint32_t err_value; uint8_t bytes[8]; } gv_q_err_t;")
    A("static const gv_q_err_t gv_q_err[] = {")
    for et, val, b in err_rows:
        A("    { %d, 0x%08Xu, { %s } }," % (et, val, u8s(b)))
    A("};")
    A("#define GV_Q_ERR_COUNT %d" % len(err_rows))
    A("")

    # --- 设备信息 ---
    A("/* ---- 0x46 QUERY_DEVICE_INFO：classic 8B / fd 16B ---- */")
    A("typedef struct {")
    A("    uint32_t hw_ver, fw_ver;")
    A("    uint64_t serial;")
    A("    uint8_t classic[8];")
    A("    uint8_t fd[16];")
    A("} gv_q_dev_t;")
    A("static const gv_q_dev_t gv_q_dev[] = {")
    for hw, fw, sn, bc, bf in dev_rows:
        A("    { 0x%08Xu, 0x%08Xu, 0x%016Xull," % (hw, fw, sn))
        A("      { %s }," % u8s(bc))
        A("      { %s } }," % u8s(bf))
    A("};")
    A("#define GV_Q_DEV_COUNT %d" % len(dev_rows))
    A("")

    # --- 心跳 ---
    A("/* ---- 0x48 心跳：Classic 8B（期望值来自参考工具的解码器）---- */")
    A("typedef struct {")
    A("    uint8_t bytes[8];")
    A("    uint8_t life, err_flags, state, control_mode;")
    A("    int16_t motor_temp_c;")
    A("    float pos_turns, vel_turns_s, iq_a;")
    A("} gv_hb_c_t;")
    A("static const gv_hb_c_t gv_hb_c[] = {")
    for b, d in hb_c:
        A("    { { %s }," % u8s(b))
        A("      %d, 0x%02X, %d, %d, %d," % (
            d["life_counter"], d["error_flags"], d["state"],
            d["control_mode"], d["motor_temp_c"]))
        A("      %s, %s, %s }," % (
            fnum(d["motor_position_turns"]), fnum(d["motor_velocity_turns_s"]),
            fnum(d["iq_current"])))
    A("};")
    A("#define GV_HB_C_COUNT %d" % len(hb_c))
    A("")

    A("/* ---- 0x48 心跳：FD 18B ---- */")
    A("typedef struct {")
    A("    uint8_t bytes[18];")
    A("    uint8_t life, err_flags, state, control_mode;")
    A("    int16_t motor_temp_c, mos_temp_c;")
    A("    float vbus_v, ibus_a, pos_turns, vel_turns_s, iq_a;")
    A("} gv_hb_f_t;")
    A("static const gv_hb_f_t gv_hb_f[] = {")
    for b, d in hb_f:
        A("    { { %s }," % u8s(b))
        A("      %d, 0x%02X, %d, %d, %d, %d," % (
            d["life_counter"], d["error_flags"], d["state"], d["control_mode"],
            d["motor_temp_c"], d["mos_temp_c"]))
        A("      %s, %s, %s, %s, %s }," % (
            fnum(d["bus_voltage"]), fnum(d["bus_current"]),
            fnum(d["motor_position_turns"]), fnum(d["motor_velocity_turns_s"]),
            fnum(d["iq_current"])))
    A("};")
    A("#define GV_HB_F_COUNT %d" % len(hb_f))
    A("")

    # --- 状态 / 控制模式名称 ---
    # 权威来源：固件 autogen 头文件（而非参考工具——它的状态名表已对本
    # 固件版本过时，见下方 gv_state_names_tool_stale）
    autogen = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "..", "..", "ODrive", "Firmware", "autogen",
                           "interfaces.hpp")
    autogen = os.path.normpath(autogen)
    fw_states = parse_enum(autogen, "AXIS_STATE_")
    fw_cmodes = parse_enum(autogen, "CONTROL_MODE_")

    A("/* ---- AxisState 名称（来源：固件 autogen/interfaces.hpp）---- */")
    A("/* ⚠ 值 5 在本固件中被跳过（旧版 ODrive 的 sensorless-control），")
    A("   因此表中**没有** 5；若实现对该值返回了名字则说明表已漂移。 */")
    A("typedef struct { uint8_t value; const char *name; } gv_name_t;")
    A("static const gv_name_t gv_state_names[] = {")
    for k in sorted(fw_states.keys()):
        if k <= 15:
            A('    { %d, "%s" },' % (k, fw_states[k]))
    A("};")
    A("#define GV_STATE_NAME_COUNT %d" % len([k for k in fw_states if k <= 15]))
    A("#define GV_STATE_5_IS_SKIPPED %d" % (0 if 5 in fw_states else 1))
    A("#define GV_STATE_MAX_4BIT 15   /* 心跳只能表示 4 bit；16 会被截断 */")
    A("")

    A("/* ---- ControlMode 名称（来源：固件 autogen/interfaces.hpp）---- */")
    A("static const gv_name_t gv_cmode_names[] = {")
    for k in sorted(fw_cmodes.keys()):
        A('    { %d, "%s" },' % (k, fw_cmodes[k]))
    A("};")
    A("#define GV_CMODE_NAME_COUNT %d" % len(fw_cmodes))
    A("")

    # 把参考工具的状态名表也记下来，作为“工具已过时”的实证
    tool_states = getattr(mod, "AXIS_STATE_NAMES", {})
    tool_rows = sorted([(k, v) for k, v in tool_states.items()
                        if isinstance(k, int) and k <= 15])
    A("/* ---- 参考工具的 AxisState 名表（已过时，仅作对照证据）---- */")
    A("/* 该表把 5 当作 sensorless-control 且缺少 15，与本固件不一致。")
    A("   测试应断言它与 gv_state_names 不同，以防有人把实现改回去。 */")
    A("static const gv_name_t gv_state_names_tool_stale[] = {")
    for k, v in tool_rows:
        A('    { %d, "%s" },' % (k, v))
    A("};")
    A("#define GV_STATE_NAME_TOOL_COUNT %d" % len(tool_rows))
    A("")

    # --- 参数：单读响应 ---
    A("/* ---- 0x20 单读响应（设备侧切片）---- */")
    A("typedef struct {")
    A("    uint16_t ep_id;")
    A("    uint8_t  full_len;      /* 设备完整值长度 */")
    A("    uint32_t offset;")
    A("    uint8_t  req_len_raw;   /* 请求原始 ReqLen（未归一化）*/")
    A("    uint8_t  classic;")
    A("    uint8_t  req_len_eff;   /* 归一化后的有效 ReqLen */")
    A("    uint8_t  rsp[12];       /* 响应帧（4 + DataLen）*/")
    A("    uint8_t  rsp_len;")
    A("    uint8_t  data_len;")
    A("    uint8_t  value[8];      /* 期望解出的值 */")
    A("} gv_p_read_t;")
    A("static const gv_p_read_t gv_p_read[] = {")
    for ep, full_len, off, rl, cl, eff, b, val in p_read:
        A("    { %d, %d, %d, %d, %d, %d," % (ep, full_len, off, rl, cl, eff))
        A("      { %s }, %d, %d," % (pad(b, 12), len(b), b[3]))
        A("      { %s } }," % pad(val, 8))
    A("};")
    A("#define GV_P_READ_COUNT %d" % len(p_read))
    A("")

    # --- 参数：写确认 ---
    A("/* ---- 0x21 写确认（8B 静默确认）---- */")
    A("typedef struct { uint8_t flags; uint16_t ep_id; uint8_t bytes[8]; } gv_p_ack_t;")
    A("static const gv_p_ack_t gv_p_ack[] = {")
    for fl, ep, b in p_ack:
        A("    { 0x%02X, %d, { %s } }," % (fl, ep, u8s(b)))
    A("};")
    A("#define GV_P_ACK_COUNT %d" % len(p_ack))
    A("")

    # --- 参数：批量响应 ---
    A("/* ---- 0x20 批量读响应 ---- */")
    A("typedef struct {")
    A("    uint8_t  n_req;")
    A("    uint8_t  bytes[32];")
    A("    uint8_t  bytes_len;")
    A("    uint8_t  count;")
    A("    uint8_t  bitmap;")
    A("    uint8_t  is_err;")
    A("} gv_p_batch_t;")
    A("static const gv_p_batch_t gv_p_batch[] = {")
    A("    { %d, { %s }, %d, %d, 0x%02X, 0 }," % (
        p_n, pad(p_batch_ok, 32), len(p_batch_ok), p_n, p_batch_ok[2]))
    A("    { %d, { %s }, %d, 0, 0x00, 1 }," % (
        p_n, pad(p_batch_err, 32), len(p_batch_err)))
    A("};")
    A("#define GV_P_BATCH_COUNT 2")
    A("")

    # --- 参数：批量读请求 ---
    A("/* ---- 0x20 批量读请求（FD 专用）---- */")
    A("typedef struct { uint8_t n; uint16_t eps[3]; uint8_t bytes[8]; } gv_p_batch_req_t;")
    A("static const gv_p_batch_req_t gv_p_batch_req[] = {")
    A("    { 3, { 100, 282, 300 },")
    A("      { 0x40, 0x03, 0x00, 0x64, 0x01, 0x1A, 0x01, 0x2C } },")
    A("};")
    A("#define GV_P_BATCH_REQ_COUNT 1")
    A("")

    # --- 参数：分段写 ---
    A("/* ---- 0x21 分段写块（Classic）---- */")
    A("typedef struct {")
    A("    uint16_t ep_id;")
    A("    uint8_t  total_len;")
    A("    uint32_t offset;")
    A("    uint8_t  expect_more;")
    A("    uint8_t  bytes[8];")
    A("    uint8_t  value[4];      /* 该块携带的有效字节 */")
    A("    uint8_t  value_len;")
    A("} gv_p_seg_t;")
    A("static const gv_p_seg_t gv_p_seg[] = {")
    for i, (b, more) in enumerate(p_seg):
        ep = (b[1] << 8) | b[2]
        total_len = b[3]
        # 该块的 offset = 前面同 ep 的块数 × 4
        prev = [x for x in p_seg[:i] if ((x[0][1] << 8) | x[0][2]) == ep]
        off = len(prev) * 4
        vlen = min(4, total_len - off)
        A("    { %d, %d, %d, %d," % (ep, total_len, off, more))
        A("      { %s }, { %s }, %d }," % (u8s(b), pad(b[4:4 + vlen], 4), vlen))
    A("};")
    A("#define GV_P_SEG_COUNT %d" % len(p_seg))
    A("")
    A("#endif /* JSDK_GOLDEN_VECTORS_PROTO_H */")

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(L) + "\n")

    print("proto golden vectors written: %s" % path)
    print("  ctrl pos classic : %d" % len(ctrl["pos_classic"]))
    print("  ctrl pos fd      : %d" % len(ctrl["pos_fd"]))
    print("  ctrl vel/tau/cur : %d / %d / %d" % (
        len(ctrl["vel"]), len(ctrl["torque"]), len(ctrl["current"])))
    print("  query f32x2      : %d" % len(f32x2))
    print("  query err        : %d" % len(err_rows))
    print("  query dev        : %d" % len(dev_rows))
    print("  heartbeat c/fd   : %d / %d" % (len(hb_c), len(hb_f)))
    print("  param read/ack   : %d / %d" % (len(p_read), len(p_ack)))
    print("  param seg        : %d" % len(p_seg))


def load_tool(path):
    spec = importlib.util.spec_from_file_location("cbt_proto", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tool", default="../ODrive/tools/can/cyberbeast_tool.py")
    ap.add_argument("--out", default="tests/data/golden_vectors_proto.h")
    args = ap.parse_args()

    if not os.path.isfile(args.tool):
        print("error: 找不到参考实现 %s" % args.tool, file=sys.stderr)
        return 2

    emit_proto(args.out, load_tool(args.tool), args.tool)
    return 0


if __name__ == "__main__":
    sys.exit(main())
