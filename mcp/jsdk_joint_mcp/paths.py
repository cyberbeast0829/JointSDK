"""定位 `jsdk_can` 绑定 —— 让 MCP server 在任何工作目录下都能加载。

⚠ 为什么需要这一层
------------------
`mcp/` 与 `bindings/python/` 是**两个独立的包**。用户跑
`python -m jsdk_joint_mcp` 时，`jsdk_can` 未必在 `sys.path` 上 ——
而报错是 `No module named 'jsdk_can'`，看起来像"SDK 装坏了"，
实际只是路径没配对。

解析顺序（先显式、后推断，且**每一步都会说明它做了什么**）：
1. `JSDK_PYTHON_SRC` 环境变量（显式指定 `bindings/python/src`）
2. `JSDK_LIB_PATH` 环境变量（显式指定库目录，如 `bsh/`）
3. 已安装的 `jsdk_can` 包（`pip install -e bindings/python` 之后）
4. **从本文件位置向上找** `bindings/python/src`（开发树里的常见布局）
5. 若 2 给了，把它插进 `sys.path` 并尝试直接从那里加载

⚠ 找不到时**不要静默失败** —— 要给一条能照着做的错误信息。
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

#: 记住我们是否已经成功解析过（避免重复扫描）
_resolved = False
_diagnosis: list[str] = []


def diagnosis() -> list[str]:
    """返回解析过程的每一步 —— 用于排"为什么加载不到"。"""
    return list(_diagnosis)


def _candidate_roots() -> list[Path]:
    """候选的 `bindings/python/src` 目录。"""
    out: list[Path] = []

    env = os.environ.get("JSDK_PYTHON_SRC")
    if env:
        out.append(Path(env))

    # 从本文件向上找：mcp/jsdk_joint_mcp/paths.py → 仓库根
    here = Path(__file__).resolve()
    for up in here.parents:
        out.append(up / "bindings" / "python" / "src")
        out.append(up / "bindings" / "python")

    return out


def ensure_jsdk_can() -> bool:
    """确保 ``import jsdk_can`` 能成功。返回是否可用。

    幂等：成功后不再重复扫描。
    """
    global _resolved
    if _resolved:
        return True

    _diagnosis.clear()

    # ---- 1) 已经在 sys.path 上？（pip install -e / PYTHONPATH 设对了） ----
    try:
        import jsdk_can  # noqa: F401
        _diagnosis.append("OK: jsdk_can 已在 sys.path 上（无需额外处理）")
        _resolved = True
        return True
    except ImportError as exc:
        _diagnosis.append(f"jsdk_can 不在 sys.path 上: {exc}")

    # ---- 2) 从候选根里找 ----
    for cand in _candidate_roots():
        pkg = cand / "jsdk_can" / "__init__.py"
        if pkg.is_file():
            if str(cand) not in sys.path:
                sys.path.insert(0, str(cand))
            try:
                import jsdk_can  # noqa: F401
                _diagnosis.append(f"OK: 从 {cand} 加载到 jsdk_can")
                _resolved = True
                return True
            except ImportError as exc:
                _diagnosis.append(f"找到 {cand} 但导入失败: {exc}")
                # 继续找下一个候选

    # ---- 3) JSDK_LIB_PATH 给了库目录？把它的父级也试一下 ----
    lib = os.environ.get("JSDK_LIB_PATH")
    if lib:
        _diagnosis.append(f"JSDK_LIB_PATH={lib}（库目录已设，但没找到 Python 包）")
    else:
        _diagnosis.append("JSDK_LIB_PATH 未设置（若包内有 lib/ 则不需要）")

    _diagnosis.append(
        "建议：cd JointSDK && pip install -e bindings/python"
        "  或设 JSDK_PYTHON_SRC=<JointSDK>/bindings/python/src"
    )
    return False


def import_error_message() -> str:
    """给 agent/用户看的、**可照着做**的加载失败说明。"""
    lines = ["无法加载 jsdk_can（Python 绑定）。", "", "排查过程:"]
    lines += [f"  - {d}" for d in _diagnosis]
    lines += [
        "",
        "修法（任选一种）:",
        "  1) 安装绑定:      cd JointSDK && pip install -e bindings/python",
        "  2) 显式指定源码:  set JSDK_PYTHON_SRC=D:\\projects\\cheetah\\JointSDK\\bindings\\python\\src",
        "  3) 显式指定库:    set JSDK_LIB_PATH=D:\\projects\\cheetah\\JointSDK\\bsh",
        "",
        "⚠ 包内自带 lib/libjsdk_can.dll，通常**不需要**设 JSDK_LIB_PATH。",
    ]
    return "\n".join(lines)
