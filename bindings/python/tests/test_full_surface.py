"""A13：Python 侧补齐**全部**公共 C API，并让 ``python -m jsdk_can`` 与 ``jsdk-cli`` 对齐。

这一模块的定位是"覆盖面守卫"，不是重复既有用例：

* :func:`test_all_public_functions_bound` —— **零缺口**守卫（防"A13 又退回去"）；
* SDO / 单位标度 / 描述符缓存与回调 —— 新绑定的功能，含**跨路径对拍**；
* CLI —— 新子命令的 JSON 形状与**安全闸**，并（可选）与 C 版 ``jsdk-cli`` 对拍字段名。
"""

from __future__ import annotations

import glob
import json
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

import jsdk_can
from jsdk_can import Context, Mode, VirtualHal
from jsdk_can import _abi

from conftest import REPO_PY, lib_dir, run_module, virtual_spec

REPO_ROOT = REPO_PY.parents[1]


def _find_c_cli() -> Path | None:
    """找到**本平台原生**的 `jsdk-cli`（没有就返回 None，相关用例自动 skip）。

    ⚠ 这里踩过一次坑，值得写下来：最初硬编码的是 `build/jsdk-cli.exe`。在 WSL 里跑
    pytest 时，Linux 内核能通过 binfmt **直接执行 Windows .exe**，于是测试真的跑起来了 ——
    但 `tmp_path` 给的是 `/tmp/...`，Windows 进程根本写不了，于是 `desc-export` 报
    “写不了 …”，看着像 SDK 的 bug（实际是**测试选错了二进制**）。
    所以：非 Windows 平台**只认原生二进制**（不带 `.exe`）。
    """
    candidates = [
        REPO_ROOT / "build" / "jsdk-cli",
        REPO_ROOT / "build-linux" / "jsdk-cli",
        REPO_ROOT / "build-asan" / "jsdk-cli",
        REPO_ROOT / "build" / "jsdk-cli.exe",
        REPO_ROOT / "build-win" / "jsdk-cli.exe",
    ]
    natives = [c for c in candidates if c.suffix != ".exe"]
    exes = [c for c in candidates if c.suffix == ".exe"]
    for c in (exes + natives) if os.name == "nt" else natives:
        if c.is_file():
            return c
    return None


C_CLI = _find_c_cli()


# ==========================================================================
# 覆盖守卫：公共 API 里不允许再有"没绑定"的函数
# ==========================================================================


def _public_functions() -> list[str]:
    names: list[str] = []
    for hdr in ("joint_sdk.h", "jsdk_hal_builtin.h"):
        p = REPO_ROOT / "include" / "joint_sdk" / hdr
        if not p.is_file():
            continue
        txt = p.read_text(encoding="utf-8")
        names += re.findall(r"JSDK_API\s+(?:[A-Za-z_][\w \t\*]*?)\s*(jsdk_\w+)\s*\(",
                            txt)
    return sorted(set(names))


def test_all_public_functions_bound():
    """**每一个** ``JSDK_API`` 函数都必须在绑定的**签名表**里（A13 的验收条件）。

    ⚠ 判据是 ``_abi._FUNCS``（真正装着 argtypes/restype 的表），**不是**"源码里
      出现过这个名字"：后者会被"只在 wrapper 里被提到、但没登记签名"骗过
      （自测过：从 `_FUNCS` 里删掉一条，只扫源码的版本依然是绿的）。
    纯源码安装（没有 include/）时自动跳过。
    """
    api = _public_functions()
    if not api:                                     # pragma: no cover - 仅 wheel
        pytest.skip("没有 include/ 目录（安装态），无法比对")

    missing = [n for n in api if n not in _abi._FUNCS]
    assert not missing, ("以下公共 API 没有登记 ctypes 签名：\n  - "
                         + "\n  - ".join(missing))
    # 顺带钉住规模：114 是 0.1.0 的公共函数数（只增不减）
    assert len(api) >= 114
    assert len(_abi._FUNCS) >= 114


def test_bound_functions_are_all_public_api():
    """反向也要查：`_FUNCS` 里不该有头文件里不存在的名字（拼错/过期的声明）。

    拼错的名字在调用前不会报错（`_initialise` 里找不到就只是跳过），所以"多"也要
    被钉住 —— 否则一个 typo 会静静躺到有人调用它为止。
    """
    api = set(_public_functions())
    if not api:                                     # pragma: no cover - 仅 wheel
        pytest.skip("没有 include/ 目录（安装态），无法比对")
    extra = sorted(set(_abi._FUNCS) - api)
    assert not extra, "绑定了头文件里不存在的函数（拼写？）：\n  - " + "\n  - ".join(extra)


def test_desc_hint_layout():
    """``jsdk_desc_hint_t`` 的布局要显式钉住（C 的 ABI 探针**不报**这个类型）。

    u16 + 2 字节填充 + u32 = 8 字节。猜错不会报错，而是把 crc/fw 读成垃圾 ——
    而它正是"原始 JSON 缓存能不能复用"的失效键。
    """
    assert _abi.ctypes.sizeof(_abi.DescHint) == 8
    assert _abi.DescHint.crc.offset == 0
    assert _abi.DescHint.fw_version.offset == 4


def test_callbacks_are_held_and_removable(ctx_joint):
    """ctypes 回调对象必须被**持有**，且可取消。

    真事故形态：CFUNCTYPE 对象被 GC → C 侧还留着函数指针 → 下一次回调跳转到
    已释放内存。所以 Context 里留了 ``_callbacks`` 字典。
    """
    ctx, _j = ctx_joint
    assert ctx._callbacks == {}

    ctx.desc_progress(lambda done, total: None)
    ctx.desc_raw_sink(lambda data, off: None)
    ctx.on_fault(lambda joint, info: None)
    assert set(ctx._callbacks) == {"progress", "raw_sink", "fault"}

    ctx.desc_progress(None)
    ctx.desc_raw_sink(None)
    ctx.on_fault(None)
    assert ctx._callbacks == {}


# ==========================================================================
# SDO 槽位（裸字节 + 状态机）
# ==========================================================================


def test_sdo_read_matches_param_get(ctx_joint):
    """SDO 的裸字节解码与 ``param_get`` **必须一致**（两条路径互相对拍）。

    这是本仓一贯做法：字节序/类型宽度写错时，单看一边可能"看着像数值"。
    """
    ctx, j = ctx_joint
    for path in ("axis0.motor.config.gear_ratio",     # f32
                 "axis0.config.can.node_id",          # u32
                 "serial_number"):                    # u64（根级；无 axis0 前缀）
        s = j.sdo(path)
        assert s.size == {"axis0.motor.config.gear_ratio": 4,
                          "axis0.config.can.node_id": 4,
                          "serial_number": 8}[path]
        assert s.read_value() == pytest.approx(j.param_get(path))
        assert len(s.read()) == s.size


def test_sdo_write_roundtrip(ctx_joint):
    """写进去的值要能被**两条路径**读回来（并复原，避免影响后续用例）。"""
    ctx, j = ctx_joint
    s = j.sdo("axis0.config.can.heartbeat_rate_ms")
    original = s.read_value()

    s.write_value(original + 50)
    assert s.read_value() == original + 50
    assert j.param_get("axis0.config.can.heartbeat_rate_ms") == original + 50

    s.write_value(original)                          # 复原
    assert s.read_value() == original


def test_sdo_state_machine_and_length_check(ctx_joint):
    ctx, j = ctx_joint
    s = j.sdo("axis0.config.can.node_id")
    s.read()
    assert s.state == 2, "阻塞读返回后状态必须是 success"   # JSDK_SDO_SUCCESS
    assert s.state_name == "success"

    with pytest.raises(ValueError):
        s.write(b"\x00\x00")                         # 长度不符：本地就该拦下


def test_sdo_slots_are_reused_per_endpoint(ctx_joint):
    """同一端点**复用**同一个槽位对象（C 侧没有 free，用尽就是死路）。"""
    ctx, j = ctx_joint
    a = j.sdo("axis0.config.can.node_id")
    b = j.sdo("axis0.config.can.node_id")
    assert a is b
    assert j.sdo_slots_used == 1

    # 端点 ID 的写法也走同一条路
    c = j.sdo(a.ep_id)
    assert c is a


# ==========================================================================
# 单位标度
# ==========================================================================


def test_unit_scale_default_is_identity():
    """CAN 后端的默认标度是**恒等映射**（线上量已是物理量）。

    与 EtherCAT 版**同名不同义**（那边是 counts→rad），所以这条要钉住。
    """
    s = jsdk_can.unit_scale_default(50)
    assert (s.pos_counts_to_rad, s.vel_counts_to_rad_s, s.trq_to_Nm) == (1.0, 1.0, 1.0)
    assert s.valid == 1


def test_unit_scale_calc_formula():
    """``calc`` 的公式要显式对拍：位置/速度 = 2π/(分辨率×减速比)，力矩 = 额定/1000。"""
    import math

    s = jsdk_can.unit_scale_calc(8192, 1, 1, 50)
    assert s.pos_counts_to_rad == pytest.approx(2 * math.pi / 8192)
    assert s.vel_counts_to_rad_s == pytest.approx(2 * math.pi / 8192)
    assert s.trq_to_Nm == pytest.approx(0.05)
    assert s.valid == 1


def test_joint_set_get_scale(ctx_joint):
    """``set_scale`` / ``get_scale`` 往返（并复原）。"""
    ctx, j = ctx_joint
    before = j.get_scale()
    assert before.valid == 1

    custom = jsdk_can.UnitScale(0.5, 0.25, 2.0, 1)
    j.set_scale(custom)
    got = j.get_scale()
    assert (got.pos_counts_to_rad, got.vel_counts_to_rad_s, got.trq_to_Nm) == \
           (0.5, 0.25, 2.0)

    j.set_scale(before)                              # 复原
    assert j.get_scale().as_dict() == before.as_dict()


# ==========================================================================
# 描述符：缓存（两条路线）+ 回调
# ==========================================================================


def test_desc_import_then_configure_still_calibrates(lib_dir):
    """⚠ 回归：导入缓存**不能**让 ``configure()`` 变成空操作。

    真事故：``desc_import()`` 把"已配置"标记置上了，于是随后的 ``configure()``
    静默返回 —— 没握手、没标定，之后所有物理量/标定量都拿到 0。
    """
    with Context(VirtualHal(virtual_spec(1))) as ctx:
        ctx.add_joint(1)
        ctx.configure()
        blob = ctx.desc_export()
        assert len(blob) > 0

    with Context(VirtualHal(virtual_spec(1))) as ctx2:
        ctx2.add_joint(1)
        ctx2.desc_import(blob)
        assert len(ctx2.endpoints()) > 0            # 描述符在手
        ctx2.configure()                            # ← 必须真的跑
        snap = ctx2.joint(1).config_snapshot()
        assert snap.valid == 1, "导入缓存后 configure() 必须仍然完成标定"
        assert snap.gear_ratio == pytest.approx(16.5)


def test_desc_raw_sink_and_import_raw_roundtrip():
    """路线 B：下载时 tee 出**原始 JSON**，下次用 ``desc_import_raw`` 直接建表。"""
    chunks: list[bytes] = []

    with Context(VirtualHal(virtual_spec(1))) as ctx:
        ctx.add_joint(1)
        seen: list[tuple[int, int]] = []
        ctx.desc_progress(lambda done, total: seen.append((done, total)))
        ctx.desc_raw_sink(lambda data, off: chunks.append(bytes(data)))
        ctx.configure()

        raw = b"".join(chunks)
        info = ctx.desc_info()
        assert len(raw) == info.total_len, "tee 出的字节数必须等于描述符总长"
        assert info.complete == 1
        assert seen and seen[-1][1] == info.total_len
        assert [t for _d, t in seen if t] and len({t for _d, t in seen if t}) == 1

    with Context(VirtualHal(virtual_spec(1))) as ctx2:
        ctx2.add_joint(1)
        ctx2.desc_import_raw(raw, crc=info.crc, fw_version=info.fw_version)
        # 描述符已在手：configure() 不该再下载
        ctx2.configure()
        assert ctx2.desc_info().total_len == info.total_len
        assert ctx2.joint(1).param_get("axis0.config.can.node_id") == 1


def test_desc_import_raw_rejects_garbage(ctx):
    ctx.add_joint(1)
    with pytest.raises(jsdk_can.JsdkError):
        ctx.desc_import_raw(b"[{\"not\": ", crc=0, fw_version=1)


def _hb_frame(node: int, master: int, err_flags: int, state: int = 1) -> _abi.CanFrame:
    """构造一帧心跳（Classic 8 B 布局）：b0 life3|err5，b1 state4|cmode4。"""
    f = _abi.CanFrame()
    f.id = (0 << 26) | (0x48 << 18) | (master << 10) | (node << 2)
    f.len = 8
    f.flags = _abi.JSDK_FRAME_EXT
    f.data[0] = err_flags & 0x1F          # err 5 bit（低 5 位）
    f.data[1] = ((state & 0x0F) << 4) | 2  # state + control_mode
    return f


def test_fault_callback_fires_on_edge_only():
    """故障回调只在**边沿**触发一次（与 C 侧一致）。

    ⚠ 回调里的 ``FaultInfo`` 是 C 侧缓存的那份（``j->fault``）：只有在调用过
      ``query_error_detail()`` 之后 ``valid`` 才为 1。所以这里**不去断言 valid**
      （那是把用例建在错的契约上），只钉：“传进来的关节对、只报一次、能取消”。
    """
    with Context(VirtualHal(virtual_spec(1))) as ctx:
        j = ctx.add_joint(1)
        ctx.configure()

        hits: list = []
        ctx.on_fault(lambda joint, info: hits.append((joint, info)))

        # 注入一条带错误的**心跳**：把设备标成 axis|motor 有错
        ctx.hal.inject(_hb_frame(1, 1, err_flags=0x03))
        for _ in range(5):
            ctx.poll()

        assert hits, "注入带错误的心跳后回调必须被触发"
        joint_seen, info_seen = hits[0]
        assert joint_seen is j, "回调必须带上对应的 Joint 对象"
        assert isinstance(info_seen, jsdk_can.FaultInfo)

        n = len(hits)
        for _ in range(5):
            ctx.poll()
        assert len(hits) == n, "边沿语义：同一次故障不该重复触发"

        ctx.on_fault(None)                            # 取消注册
        assert "fault" not in ctx._callbacks


# ==========================================================================
# CLI：新子命令的 JSON 形状与安全闸
# ==========================================================================


def test_cli_write_reports_readback(lib_dir):
    """``write`` 的 JSON 与 C 版同形状，且 ``value`` 是**读回值**（不是输入值）。"""
    r = run_module("--if", "virtual", "--json", "--node", "1", "--yes",
                   "write", "axis0.config.can.heartbeat_rate_ms", "100",
                   lib_dir=lib_dir)
    assert r.returncode == 0, r.stderr
    p = json.loads(r.stdout)
    assert p["path"] == "axis0.config.can.heartbeat_rate_ms"
    assert p["id"] == 182
    assert p["type"] == "uint32"
    assert p["value"] == 100
    assert p["written"] is True
    assert "save" in p["persisted"]


def test_cli_info_and_err_shapes(lib_dir):
    info = json.loads(run_module("--if", "virtual", "--json", "info",
                                 lib_dir=lib_dir).stdout)
    assert set(info) == {"hw_version", "fw_version", "serial", "classic", "node"}

    err = json.loads(run_module("--if", "virtual", "--json", "err",
                                lib_dir=lib_dir).stdout)
    assert err["mit_err_name"] == "NONE"
    assert set(err) >= {"motor_error", "encoder_error", "axis_error"}


def test_cli_batch_read_matches_single_read(lib_dir):
    """批量读与单读必须给出同一个值（两条解析路径对拍）。"""
    paths = ["axis0.config.can.node_id", "axis0.motor.config.gear_ratio"]
    batch = json.loads(run_module("--if", "virtual", "--json", "batch-read",
                                  *paths, lib_dir=lib_dir).stdout)
    got = {v["path"]: v["value"] for v in batch["values"]}
    for p in paths:
        solo = json.loads(run_module("--if", "virtual", "--json", "read", p,
                                     lib_dir=lib_dir).stdout)
        assert got[p] == solo["value"], p


def test_cli_desc_export_import_file(lib_dir, tmp_path):
    f = tmp_path / "desc.bin"
    r = run_module("--if", "virtual", "--json", "desc-export", str(f),
                   lib_dir=lib_dir)
    assert r.returncode == 0, r.stderr
    assert json.loads(r.stdout)["bytes"] == f.stat().st_size > 0

    r2 = run_module("--if", "virtual", "--json", "desc-import", str(f),
                    lib_dir=lib_dir)
    assert r2.returncode == 0, r2.stderr
    p = json.loads(r2.stdout)
    assert p["imported"] is True and p["downloaded"] is False


_MON_CSV_HEADER = (
    "t_ms,node,pos_rad,vel_rad_s,current_A,torque_Nm,"
    "t_motor_C,t_fet_C,vbus_V,ibus_A,axis_state,mode,err_code,"
    "hb_error,age_ms,tx_frames,tx_rejected"
)


def test_mon_csv_is_identical_to_c_cli(lib_dir, tmp_path):
    """`mon --csv` 两版必须**逐字节相同**（表头 + 数据行的格式）。

    ⚠ 这条用例不是“锦上添花”：修之前两版的 CSV 列数都不一样（12 vs 17），
       而文档却写“方便两边对着看”。另外 C 版 `--csv` 曾经要一个文件名，
       `mon --csv --duration 1` 会静默写出一个叫 `--duration` 的 CSV 文件。
    """
    py = run_module("--if", "virtual", "--node", "1", "--csv",
                    "--duration", "1", "--rate-hz", "20", "mon",
                    lib_dir=lib_dir)
    assert py.returncode == 0, py.stderr
    py_lines = py.stdout.splitlines()
    assert py_lines[0] == _MON_CSV_HEADER
    assert py_lines[0].count(",") == 16                 # 17 列

    if C_CLI is None:
        pytest.skip("没构建 C 版 jsdk-cli（或本平台只有别的平台的二进制）")
    cc = subprocess.run([str(C_CLI), "--if", "virtual", "--node", "1", "--csv",
                         "--duration", "1", "--rate-hz", "20", "mon"],
                        capture_output=True, text=True, encoding="utf-8",
                        errors="replace", timeout=120)
    assert cc.returncode == 0, cc.stderr
    c_lines = cc.stdout.splitlines()
    assert c_lines[0] == py_lines[0], "两版 CSV 表头必须逐字节一致"
    # 数据行的**列数**必须一致（数值本身随仿真时钟会不同，不比）
    assert c_lines[1].count(",") == py_lines[1].count(",") == 16


def test_mon_csv_file_both_clis(lib_dir, tmp_path):
    """`--csv-file` 落盘：两版都要能写，且表头一致；子命令前后两种写法都要认。"""
    out = tmp_path / "py_mon.csv"
    r = run_module("--if", "virtual", "--node", "1", "--csv-file", str(out),
                   "--duration", "1", "--rate-hz", "20", "mon", lib_dir=lib_dir)
    assert r.returncode == 0, r.stderr
    lines = out.read_text(encoding="utf-8").splitlines()
    assert lines[0] == _MON_CSV_HEADER and len(lines) >= 2

    # ⚠ 选项写在子命令之后也必须生效（argparse 子解析器会把 default 写回去，
    #    真踩过：文件根本不生成而且不报错）
    out2 = tmp_path / "py_mon2.csv"
    r2 = run_module("--if", "virtual", "--node", "1", "mon",
                    "--csv-file", str(out2), "--duration", "1", "--rate-hz", "20",
                    lib_dir=lib_dir)
    assert r2.returncode == 0, r2.stderr
    assert out2.read_text(encoding="utf-8").splitlines()[0] == _MON_CSV_HEADER


@pytest.mark.skipif(C_CLI is None, reason="没构建 C 版 jsdk-cli")
def test_mon_csv_file_value_must_not_be_an_option(lib_dir, tmp_path):
    """C 版把另一个选项当成文件名 → 必须当场报用法错（不再静默写怪文件）。"""
    cp = subprocess.run([str(C_CLI), "--if", "virtual", "--csv-file",
                         "--duration", "1", "mon"],
                        capture_output=True, text=True, encoding="utf-8",
                        errors="replace", timeout=60)
    assert cp.returncode == 2
    assert "另一个选项" in cp.stderr


def test_write_value_exit_codes_match_c_cli(lib_dir):
    """`write` 的两种值错要**两版一致**：非数字 = 用法错(2)，超范围 = 运行期错误(1)。

    ⚠ 这条是实测出来的偏差：修之前 C 版两种都返回 1，Python 版返回 2/1，
       而文档写着“两版同一份退出码契约”。
    """
    r = run_module("--if", "virtual", "--node", "1", "--yes", "write",
                   "can.config.break_timeout", "abc", lib_dir=lib_dir)
    assert r.returncode == 2, r.stderr

    r2 = run_module("--if", "virtual", "--node", "1", "--yes", "write",
                    "can.config.break_timeout", "99999", lib_dir=lib_dir)
    assert r2.returncode == 1, r2.stderr

    if C_CLI is None:
        return
    for args, want in ((["write", "can.config.break_timeout", "abc"], 2),
                       (["write", "can.config.break_timeout", "99999"], 1)):
        cp = subprocess.run([str(C_CLI), "--if", "virtual", "--node", "1",
                             "--yes", *args],
                            capture_output=True, text=True, encoding="utf-8",
                            errors="replace", timeout=60)
        assert cp.returncode == want, (args, cp.returncode, cp.stderr)


def test_cli_mit_requires_hold(lib_dir):
    """``mit`` 的两道闸：--yes 与 --hold（自限时）。"""
    r = run_module("--if", "virtual", "--node", "1", "--yes", "mit",
                   lib_dir=lib_dir)
    assert r.returncode == 3
    assert "--hold" in r.stderr

    r2 = run_module("--if", "virtual", "--node", "1", "--yes", "--hold", "99",
                    "mit", lib_dir=lib_dir)
    assert r2.returncode == 3, "超过 60 s 也必须被拒"


def test_cli_mon_csv(lib_dir):
    r = run_module("--if", "virtual", "--node", "1", "--csv",
                   "--duration", "1", "--rate-hz", "20", "mon", lib_dir=lib_dir)
    assert r.returncode == 0, r.stderr
    lines = [ln for ln in r.stdout.splitlines() if ln.strip()]
    assert lines[0] == _MON_CSV_HEADER        # 17 列，与 C 版同一份契约
    assert len(lines) >= 2
    assert len(lines[1].split(",")) == 17


@pytest.mark.skipif(C_CLI is None,
                    reason="没构建 C 版 jsdk-cli（或本平台只有别的平台的二进制）")
def test_cli_json_field_parity_with_c_cli(lib_dir):
    """同一命令在 Python 与 C 两个入口下，**JSON 字段名必须一致**（脚本可互换）。

    只比字段名（值会随仿真推进而变），这正是"互换使用"的前提。
    """
    for args in (("info",), ("err",), ("read", "axis0.motor.config.gear_ratio"),):
        py = json.loads(run_module("--if", "virtual", "--json", *args,
                                   lib_dir=lib_dir).stdout)
        cp = subprocess.run([str(C_CLI), "--if", "virtual", "--json", *args],
                            capture_output=True, text=True, encoding="utf-8",
                            errors="replace", timeout=120)
        assert cp.returncode == 0, cp.stderr
        cc = json.loads(cp.stdout)
        assert set(py) == set(cc), f"{args}: 字段不一致 {set(py)} != {set(cc)}"


#: 逐命令冒烟清单（`{tmp}` 会被替换成临时目录）。
#: ⚠ 这份清单存在的理由：真机上"命令能不能跑"不是靠读代码能确认的 ——
#:   `watchdog` 曾因为被归到"只需描述符"档而**必然**以
#:   `can.config.break_timeout endpoint unavailable` 失败（两版 CLI 都错），
#:   而所有单命令用例都恰好没覆盖它。
_COMMAND_SMOKE: list[list[str]] = [
    ["scan"], ["info"], ["err"], ["hb-dump"], ["health"],
    ["mon", "--csv", "--duration", "1", "--rate-hz", "20"],
    ["read", "axis0.motor.config.gear_ratio"],
    ["batch-read", "axis0.config.can.node_id", "axis0.motor.config.gear_ratio"],
    ["dump-config"], ["desc-info"], ["ep-list"],
    ["ep-lookup", "axis0.motor.config.gear_ratio"],
    ["desc-export", "{tmp}/desc.bin"],
    ["desc-import", "{tmp}/desc.bin"],
    ["write", "--yes", "axis0.config.can.heartbeat_rate_ms", "100"],
    ["watchdog", "--yes", "100"],
    ["save", "--yes"], ["set-node-id", "--yes", "2"], ["reset", "--yes"],
    ["set-zero", "--yes"], ["calibrate", "--yes"], ["home", "--yes"],
    ["estop"],
    ["mit", "--yes", "--hold", "1", "--pos", "0", "--kp", "1", "--kd", "0.1"],
]


@pytest.mark.parametrize("args", _COMMAND_SMOKE,
                         ids=[" ".join(a) for a in _COMMAND_SMOKE])
def test_every_subcommand_runs_on_virtual(args, lib_dir, tmp_path):
    """每个子命令都要能在**默认虚拟后端**上跑通（退出码 0）。"""
    args = [a.replace("{tmp}", str(tmp_path)) for a in args]
    if args[0] == "desc-import":                     # 先造出要导入的文件
        r0 = run_module("--if", "virtual", "desc-export", str(tmp_path / "desc.bin"),
                        lib_dir=lib_dir)
        assert r0.returncode == 0, r0.stderr

    r = run_module("--if", "virtual", "--node", "1", *args, lib_dir=lib_dir)
    assert r.returncode == 0, f"{' '.join(args)} 失败：{r.stderr}"


@pytest.mark.skipif(C_CLI is None, reason="没构建 C 版 jsdk-cli（或只有别的平台的二进制）")
@pytest.mark.parametrize("args", _COMMAND_SMOKE,
                         ids=[" ".join(a) for a in _COMMAND_SMOKE])
def test_every_subcommand_runs_on_virtual_c_cli(args, tmp_path):
    """C 版 `jsdk-cli` 也要能跑通同一份清单（两个入口保持同一份契约）。

    这条是"两版行为对齐"的兜底：Python 侧发现的问题（例如 `watchdog` 档位）
    很可能同样存在于 C 侧 —— 真发生过。
    """
    args = [a.replace("{tmp}", str(tmp_path)) for a in args]
    if args[0] == "mon":
        # ⚠ C 版 `mon` 的 `--duration` 走的是 **SDK 时钟**，而虚拟后端的时间只在
        #   `now_ms()` 被调用时前进 ⇒ "1 秒"要跑上千个周期、墙钟上是几分钟
        #   （实测 >60 s 未结束）。这是虚拟后端的固有特性，与本次改动无关；
        #   Python 版 `mon` 用墙钟计时，所以那条在清单里保留。
        pytest.skip("C 版 mon 在虚拟后端不受墙钟约束（已记录）")
    if args[0] == "desc-import":
        r0 = subprocess.run([str(C_CLI), "--if", "virtual", "desc-export",
                             str(tmp_path / "desc.bin")],
                            capture_output=True, text=True, encoding="utf-8",
                            errors="replace", timeout=120)
        assert r0.returncode == 0, r0.stderr

    cp = subprocess.run([str(C_CLI), "--if", "virtual", "--node", "1", *args],
                        capture_output=True, text=True, encoding="utf-8",
                        errors="replace", timeout=120)
    assert cp.returncode == 0, f"C 版 {' '.join(args)} 失败：{cp.stderr}"
