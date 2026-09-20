"""
为什么这个包除了 `pyproject.toml` 还需要一个 `setup.py`：

包里**捆绑了平台相关的共享库**（`jsdk_can/lib/libjsdk_can.{dll,so,dylib}`），
而 setuptools 默认把这种包打成 **`py3-none-any`** —— `any` 的含义是"纯 Python、
任何平台都能装"。于是 pip 在 Linux/macOS 上也会认为它可安装，**装完才在
`ctypes.CDLL(libjsdk_can.dll)` 那一步炸掉**（而且报错发生在 `import jsdk_can` 之后，
看着像绑定有 bug，不像"装错了平台"）。

这里把 wheel 标成 **平台相关、但 Python 版本无关**（如 `py3-none-win_amd64`）：

- 不能是 `any`：库是编译好的本机二进制；
- 也不要 `cp312-cp312` 那种钉死版本的 tag：ctypes 绑定不依赖 CPython ABI，
  同一个 wheel 本来就能给 3.9~3.13 用，钉死只会让客户被迫重编。

⚠ 平台标签取的是**构建机**的平台，所以"给哪个平台发包"必须在那个平台上构建
（本项目的做法：`cmake -S . -B bsh -DJSDK_BUILD_SHARED=ON -DJSDK_BUILD_PYTHON=ON`
把库放到 `src/jsdk_can/lib/`，然后在这台机器上 `python -m build --wheel`）。
"""

import glob
import os
import sys

from setuptools import setup
from setuptools.dist import Distribution

_HERE = os.path.dirname(os.path.abspath(__file__))
_LIB_DIR = os.path.join(_HERE, "src", "jsdk_can", "lib")
_LIB_GLOBS = ("*.dll", "*.so", "*.so.*", "*.dylib")


def _bundled_libs():
    found = []
    for pat in _LIB_GLOBS:
        found.extend(glob.glob(os.path.join(_LIB_DIR, pat)))
    return sorted(found)


class BinaryDistribution(Distribution):
    """声明"本包不是纯 Python"（`has_ext_modules` 是 wheel 判纯/非纯的依据）。"""

    def has_ext_modules(self):  # pragma: no cover - 只被构建流程调用
        return True


try:
    from wheel.bdist_wheel import bdist_wheel as _bdist_wheel
except ImportError:  # pragma: no cover - sdist 场景（只需要 metadata）
    _bdist_wheel = None


if _bdist_wheel is not None:

    class bdist_wheel(_bdist_wheel):  # noqa: N801 - 类名由 setuptools 认
        """把 tag 钉成 `py3-none-<plat>`。"""

        def finalize_options(self):
            super().finalize_options()
            self.root_is_pure = False

        def get_tag(self):
            _py, _abi, plat = super().get_tag()
            return "py3", "none", plat

    _cmdclass = {"bdist_wheel": bdist_wheel}
else:
    _cmdclass = {}


def _warn_if_no_lib():
    """库里没共享库时不报错（源码安装 + `JSDK_LIB_PATH` 是合法用法），但要说清楚。"""
    if not _bundled_libs():
        sys.stderr.write(
            "warning: src/jsdk_can/lib/ 里没有共享库 —— 这样打出来的 wheel\n"
            "         装到目标机上只能靠 JSDK_LIB_PATH 指向外部库。\n"
            "         要捆绑库请先构建：\n"
            "             cmake -S . -B bsh -DJSDK_BUILD_SHARED=ON -DJSDK_BUILD_PYTHON=ON\n"
            "             cmake --build bsh\n"
        )


_warn_if_no_lib()

setup(distclass=BinaryDistribution, cmdclass=_cmdclass)
