# 发布到 PyPI（`cyberbeast-joint-sdk` wheel）流程说明

> 面向：负责把 Python 绑定发出去的人。
> 相关：`PYTHON.zh-CN.md` §6.1（库定位与平台支持）、`BACKLOG.zh-CN.md`（待办编号）、`DESIGN.zh-CN.md` §7（构建与交付形态）。
> 本文只讲**怎么发**与**哪些坑必须绕开**，不讲绑定 API。

**状态**：本文的**结论**都来自实测（下面的证据块是跑出来的，不是推测）；
但**只有 Windows 那条路径真正端到端跑过**，Linux（manylinux）与 macOS 两条路径
**尚未实跑**，凡未实测处均已显式标注 —— 别把"应该行"当成"已经行"。

---

## 1. 核心结论

| # | 结论 |
|---|---|
| 1 | **客户不需要编译任何东西**。发的是**平台相关的 wheel**，编译好的 `libjsdk_can.{dll,so,dylib}` 就在 wheel 里 |
| 2 | wheel tag 是 **`py3-none-<plat>`**：`py3` ⇒ 一个 wheel 覆盖**所有 Python ≥3.8**（ctypes 不依赖 CPython ABI）；`none` ⇒ 绝不会被装到错误平台 |
| 3 | 所以只需 **每平台 1 个 wheel**（不是"每 Python 版本 1 个"），总数量 ≈ Windows ×1 + Linux ×2(arch) + macOS ×2(arch) |
| 4 | Linux 的 wheel **必须在 manylinux 基线里构建**（不是在开发机上）；macOS 的 wheel **必须在 Mac 上构建** |
| 5 | **只发 wheel，不发 sdist**（没有 C 源码的 sdist 会在客户机上编出"装着别的平台二进制"的 wheel，见 §5） |
| 6 | 上传前必须真的**装上跑一遍**（`import` + 一次仿真往返），见 §7 |

已实测产出的 wheel 名字长这样（§4.2）：

```
cyberbeast_joint_sdk-0.1.0-py3-none-win_amd64.whl
```

---

## 2. 为什么 tag 不是 `any`、也不是 `cp312-cp312`

`bindings/python/setup.py` 里做了两件事，这里解释它们各自防的是什么：

| 写法 | 会出什么事 |
|---|---|
| `py3-none-any`（setuptools 默认） | `any` = "纯 Python，任何平台都能装" ⇒ pip 在 Linux/macOS 上也会认为可安装，**装完才在 `ctypes.CDLL` 那步炸**，而且报错发生在 `import jsdk_can` 之后，看着像绑定有 bug，不像"装错了平台" |
| `cp312-cp312-win_amd64` | 把 CPython 版本钉死 ⇒ 3.9/3.10/3.11 的客户全都要各自一个 wheel，纯属自找麻烦（ctypes 绑定不依赖 ABI） |
| **`py3-none-win_amd64`**（本项目的选择） | 每平台一个 wheel，覆盖 3.8+；平台不对时 pip 直接说"没有匹配的分发"，错误发生在**安装期**而不是运行期 |

> ⚠ 平台标签取的是**构建机**的平台（`bdist_wheel.get_tag()` 无法凭空指定）——
> 这正是"给哪个平台发包就必须在那个平台上构建"的根本原因。

---

## 3. 版本库该提交什么、不该提交什么

| 路径 | 提交？ | 说明 |
|---|---|---|
| `src/`、`include/`、`CMakeLists.txt`、`tools/`、`tests/`、`examples/`、`cmake/` | ✅ | 源码与交付面 |
| `docs/`、`README.md` | ✅ | |
| `bindings/python/{pyproject.toml,setup.py,src/jsdk_can/*.py,tests/,examples/,README.md}` | ✅ | 纯 Python 部分 |
| `arduino/src/jsdk_can_amalgam.{h,c}` | ✅ | **是产物，但必须提交**：Arduino 库规范要求源码放在 `src/`，两份必须逐字节一致（`tools/arduino_smoke.sh` 会查） |
| `bsh/` | ❌ | CMake 构建目录（`CMakeCache.txt`、`.a/.dll/.exe`、`Makefile`…）。**注意 `.gitignore` 里的 `build*/` 并不匹配 `bsh`** —— 补上 `bsh/` 之前，它就是以未跟踪状态露在 `git status` 里的 |
| `dist/`（仓库根） | ❌ | `tools/amalgamate.py` 的产物。唯一被消费的副本在 `arduino/src/` |
| `bindings/python/src/jsdk_can/lib/` | ❌ | 构建产物（由 `-DJSDK_BUILD_PYTHON=ON` 填充）。**但打包时必须先填上**，见 §4.1 |
| `bindings/python/{build,dist}/`、`.pytest_cache/`、`*.egg-info/`、`*.whl` | ❌ | 打包/测试产物 |

**现状（2026-09-20 实测）**：`.gitignore` 已经补齐，索引里 **142** 个文件，`bsh/`、`dist/`、
`jsdk_can/lib/`、`cyberbeast_joint_sdk.egg-info/` 都不在其中：

```
*.pyc
*.obj
*.d
build*/                       ← 注意它匹配的是"名字以 build 开头的目录"，所以 build-linux/、build-py/
                                也都覆盖了（不必逐条加）
bsh/                          ← 这一行必须显式写：build*/ 并**不**匹配 bsh
dist/                         ← amalgamation 产物（不带前导斜杠 → 任意层级的 dist/ 都被匹配）
bindings/python/dist/         ← wheel 产物；因为上一行已经覆盖，这行严格说是冗余的（留着无害）
*.egg-info/
.pytest_cache
*.whl
```

> ⚠ **`cyberbeast_joint_sdk.egg-info/` 曾经被 stage 过**（`bindings/python/src/` 下 6 个文件：
> `PKG-INFO`、`SOURCES.txt`…）。它是 `pip`/`build` 生成的元数据，每次构建都会变
> （实测：跑一次 `python -m build` 就会把 `PKG-INFO`/`SOURCES.txt` 改掉），
> 放在版本库里只会制造无意义的 diff。
> **当前状态**：`git ls-files | grep -c egg-info` = `0`，已被撤出并被 `*.egg-info/` 兜住 ——
> 以后别再把 `git add -A` 之后的这类文件又加回去。

---

## 4. 构建

### 4.1 通用前提：先让包内 `lib/` 有**本平台**的库

顺序不能反：**先 CMake 构建（把库复制进 `src/jsdk_can/lib/`），再打 wheel**。
CMake 这一步会（a）把库拷成**不带版本号**的规范名，（b）**删掉其它平台的库** ——
所以只要是在目标平台上构建的，`lib/` 里就只会有该平台的库。

```bash
cmake -S . -B bsh -DJSDK_BUILD_SHARED=ON -DJSDK_BUILD_PYTHON=ON
cmake --build bsh
```

> 构建类型：`CMakeLists.txt:21` 已经在没指定时兜底成 **`RelWithDebInfo`**（`-O2 -g`），
> 所以**不写 `-DCMAKE_BUILD_TYPE` 也是带优化的**，不会误发一份没优化的库。
> 想发更干净的二进制（去掉调试信息、体积更小）再显式加 `-DCMAKE_BUILD_TYPE=Release`。

构建日志里应出现：

```
Copying shared library into bindings/python/src/jsdk_can/lib/libjsdk_can.dll   # 平台名随之变化
```

> 若跳过这一步直接打包，`setup.py` 目前**只打印 warning 不报错**（→ §9 待办），
> 打出来的 wheel 装到客户机上只能靠 `JSDK_LIB_PATH` 指向外部库。

### 4.2 Windows（**已实测**）

```bash
cmake -S . -B bsh -G "MinGW Makefiles" -DJSDK_BUILD_SHARED=ON -DJSDK_BUILD_PYTHON=ON
cmake --build bsh -j
cd bindings/python && python -m build --wheel        # 产物在 bindings/python/dist/
```
实测结果：`cyberbeast_joint_sdk-0.1.0-py3-none-win_amd64.whl`，包内含 `jsdk_can/lib/libjsdk_can.dll`。

**这个 wheel 在客户机上是自足的**（实测导入表，见下），不需要客户额外安装 MinGW 运行库 ——
"MinGW 编出来的 dll 缺 `libgcc_s_seh-1.dll`/`libwinpthread-1.dll` 报 `WinError 126`"这个常见坑，**本项目不存在**：

```
$ objdump -p bsh/libjsdk_can.dll | grep "DLL Name"
        DLL Name: KERNEL32.dll
        DLL Name: msvcrt.dll          ← 每台 Windows 都有
$ objdump -p bsh/libjsdk_can.dll | grep -c "jsdk_"      # 导出符号数
114
```

> ⚠ 换工具链（例如改用 MSVC，或给 CMake 加 `-static-libgcc` 之外的改动）后，
> **要重新跑一次上面这两条命令**，别沿用旧结论。

### 4.3 Linux（**未实测**，方案已定）

先看实测到的链接面（Ubuntu 20.04 上构建的 `build-py/libjsdk_can.so.0`）：

```
NEEDED   libm.so.6
NEEDED   libc.so.6
SONAME   libjsdk_can.so.0
该 .so 用到的最高 glibc 符号版本:  GLIBC_2.17
构建机 glibc:                     2.31  (Ubuntu 20.04)
```

也就是说**依赖面非常干净**（只有 libm/libc），符号面只要求 glibc ≥ 2.17。
但**不要在开发机上直接发**：wheel 的平台标签取决于构建环境，本机打出来的会带
`linux_x86_64`（pip 确实认这个标签 —— 它是平台标签列表的最后一项），
但那个标签**不含任何兼容性承诺**，只保证"x86_64"。

正规做法是在官方基线镜像里构建：

```bash
docker run --rm -v "$PWD:/io" -w /io quay.io/pypa/manylinux_2_28_x86_64 bash -c '
  cmake -S . -B bsh -DJSDK_BUILD_SHARED=ON -DJSDK_BUILD_PYTHON=ON &&
  cmake --build bsh -j &&
  cd bindings/python &&
  /opt/python/cp312-cp312/bin/python -m build --wheel --outdir /io/wheelhouse &&
  /opt/python/cp312-cp312/bin/python -m auditwheel repair /io/wheelhouse/*.whl -w /io/wheelhouse
'
```

> ⚠ 在 **Git Bash** 里跑这段得加 `MSYS_NO_PATHCONV=1 MSYS2_ARG_CONV_EXCL='*'`，
> 否则 `-v "$PWD:/io"` 会被翻成 `D:/…` 而挂载失败（本项目在 `tools/*.sh` 里已经踩过一次）。
> 干脆在 **WSL** 里跑最省事。

- `manylinux_2_28`（glibc 2.28）⇒ 覆盖 RHEL 8+/Ubuntu 20.04+/Debian 10+；
- 想连 CentOS 7 / Ubuntu 18.04 一起覆盖，用 `manylinux2014_x86_64`（= glibc 2.17）镜像 ——
  从符号面看**是够的**（最高只要 2.17），但要实跑确认；
- `auditwheel repair` 的职责是把 tag 改成合规的 `manylinux_*`；本项目的
  `libjsdk_can.so` 没有第三方依赖，正常情况下不需要它搬内部库（不需要 RPATH：
  加载器是用**绝对路径** `ctypes.CDLL` 打开的）；
- arm64 客户需要另跑一次 `manylinux_2_28_aarch64`（原生 runner 或 qemu）；
- 镜像里若没有 cmake：`dnf install -y cmake ninja`。

### 4.4 macOS（**未实测**，本仓无 Mac 机器）

必须在 Mac 上构建，x86_64 与 arm64 各一次；要 universal2 需
`-DCMAKE_OSX_ARCHITECTURES="x86_64;arm64"`（CMake ≥ 3.16 支持，但本项目未验证过）。

### 4.5 汇总

所有 wheel 收进同一个目录再上传；文件名天然互不相同（`...win_amd64`、`...manylinux_2_28_x86_64`、
`...macosx_11_0_arm64`…），`twine` 一次传完即可。

---

## 5. ⚠ 为什么**不发** sdist（实测证据）

`sdist` 会把项目当成"源码分发"，让 pip 在**客户机上编译**。本项目这条路是**坏的**：

```
$ python -m build --sdist --outdir /tmp/x
$ tar tzf /tmp/x/*.tar.gz | grep -cE '\.c$'          # sdist 里的 C 源码个数
0
$ tar tzf /tmp/x/*.tar.gz | grep 'jsdk_can/lib/'
.../src/jsdk_can/lib/
.../src/jsdk_can/lib/libjsdk_can.dll                 # ← 构建机（Windows）的二进制
```

原因：sdist 的根是 `bindings/python/`，而 C 源码在仓库根的 `src/`、`include/`、`CMakeLists.txt`，
**进不了 sdist**；反而包内 `lib/` 被当 `package-data` 打了进去。

后果很具体：Linux 客户如果没有匹配的 wheel，pip 会回退到 sdist，在**客户机上**"编译"出
一个 `py3-none-linux_x86_64` 的 wheel，**里面装的却是 Windows 的 DLL** ——
装得下、导入才炸。所以：

- **现在只发 wheel**；上传时用 `twine upload dist/*.whl`（不要 `dist/*`）；
- 若将来确实要发 sdist，前置条件是：把 C 源拉进 sdist（`MANIFEST.in` 指定仓库根文件）、
  让构建时能自己编出共享库、并把 §9 的"空库必须硬失败"先做掉。

---

## 6. 上传

### 6.1 先拿 TestPyPI 演练

```bash
python -m twine upload -r testpypi dist/*.whl
pip install -i https://test.pypi.org/simple/ cyberbeast-joint-sdk     # 在一台干净机器上
```

### 6.2 正式上传

```bash
python -m twine upload dist/*.whl
```

> ⚠ PyPI **不允许同一个版本号覆盖重传**（哪怕只改了一个字节）—— 每次发布都必须 bump 版本，
> 而且要把 `pyproject.toml` 与 `CMakeLists.txt` 的 `project(... VERSION ...)` **一起改**（→ §7/§9）。

**推荐用 Trusted Publishing（OIDC）+ GitHub Actions**：不需要 API token、不需要在本地存密码，
且每次由 CI 在干净环境里构建 —— 顺便消掉"我这台机器的环境是不是也进了二进制"的疑虑。
仓库已有远端 `github.com/cyberbeast0829/JointSDK`，配 workflow 即可（→ §9 待办）。

### 6.3 手动上传时的凭据纪律

- token 放 `~/.pypirc` 或 `TWINE_PASSWORD` 环境变量；
- **不要**写进提交、issue/工单、聊天记录或脚本。

### 6.4 cibuildwheel 的两个要点

1. 本项目 C 库要**在打 wheel 之前**编出来 ⇒ 用 `CIBW_BEFORE_ALL`（或 workflow 里先跑一步 CMake）
   把库放进 `src/jsdk_can/lib/`；
2. 因为 tag 恒为 `py3-none-<plat>`，**不要**让 cibuildwheel 为每个 Python 版本都编一遍 ——
   否则 6 个 Python 版本会产出 6 个**同名** wheel（互相覆盖或直接报重复）。
   用 `CIBW_BUILD: "cp312-*"` 只留一个即可。

---

## 7. 发布前 / 发布后校验清单

发布前（在构建机上）：

- [ ] `pyproject.toml` 的 `version` == `CMakeLists.txt` 的 `project(... VERSION ...)`
      （目前都是 `0.1.0`；**靠手工同步**，见 §9）
- [ ] `bindings/python/src/jsdk_can/lib/` 里是**本平台**的库，且只有本平台的
- [ ] wheel 里确实含库，且是**本平台**的库：
      `python -c "import sys,zipfile;[print(n) for n in zipfile.ZipFile(sys.argv[1]).namelist() if '/lib/' in n]" dist/*.whl`
- [ ] wheel 文件名里的平台段是预期值（`win_amd64` / `manylinux_*` / `macosx_*`）
- [ ] `py.typed` 在 wheel 里（否则客户侧类型检查器不认注解）

发布后（在干净环境，**不设** `JSDK_LIB_PATH`）：

- [ ] `pip install cyberbeast-joint-sdk`（或 `pip install ./xxx.whl`）成功
- [ ] `python -c "import jsdk_can; print(jsdk_can.library_search_path())"` 指向**包内** `lib/`
- [ ] 一次仿真往返（`VirtualHal` 建上下文 + 读一个参数）真的能跑
- [ ] 平台矩阵逐个确认（同一个版本号在各平台都装得上）

> 上面"装上跑一遍"目前**还没有自动化脚本**（→ §9）。在补齐之前，每次发版都得手工做，
> 别因为 Windows 上自测通过就认为 Linux 的 wheel 也没问题。

---

## 8. 许可证与可见性（**发之前必须先定**）

- `pyproject.toml` 里 `license = { text = "Proprietary" }`，与 ADR 7「专有许可」一致；
- **但 PyPI 是公开的**：一旦上传，全世界都能下载这份**编译好的二进制**，
  而且 PyPI **没有**"只给某个客户"的机制；
- 若客户只是少数几家，更合适的载体：

| 方案 | 客户侧命令 |
|---|---|
| 私有索引（devpi / Artifactory / Nexus / Cloudsmith / AWS CodeArtifact） | `pip install --index-url https://<内部>/simple/ cyberbeast-joint-sdk` |
| GitHub Release 附件（免建索引，但需要鉴权时得客户自己带 token） | `pip install https://github.com/cyberbeast0829/JointSDK/releases/download/v0.1.0/<wheel>` |
| 直接把 `.whl` 发给客户 | `pip install ./cyberbeast_joint_sdk-0.1.0-py3-none-win_amd64.whl` |

> 注：**GitHub Packages 不提供 PyPI 索引**（它只支持 npm/Docker/Maven/NuGet/RubyGems），
> 所以"用 GitHub Packages 发 Python 包"这条路是走不通的，别写成方案。

- 发布前还要改掉占位符：`[project.urls] Documentation = "https://example.invalid/joint-sdk"`
  会显示在 PyPI 页面上。

---

## 9. 未做 / 待办

| # | 事项 |
|---|---|
| 1 | GitHub Actions workflow（`cibuildwheel` + Trusted Publishing，win/manylinux/macos 三平台）——**尚未编写** |
| 2 | Linux manylinux 路径**未实跑**（§4.3 是方案，不是实测报告） |
| 3 | macOS wheel **从未构建过**（本仓无 Mac；`PYTHON.zh-CN.md` §6.1 的平台矩阵里 macOS 标的是"未实测"） |
| 4 | `setup.py` 在"包内无库"时只 warning ⇒ 应改为**打 wheel 时硬失败**（否则可能默默发出版本装不上库的 wheel） |
| 5 | `pyproject.toml` 与 `CMakeLists.txt` 的版本号**手工同步** ⇒ 应加自动一致性检查 |
| 6 | wheel 冒烟脚本（干净 venv → 装 → import → 仿真往返）尚未编写，见 §7 |
| 7 | 本文档**尚未列入 `README.md` 的文档索引表**；顺带发现 `README.md` 里"未完成：**WP8 Python 绑定**"是**过期**的（WP8 已在 v0.14 完成） |

---

## 10. 变更记录

| 版本 | 日期 | 说明 |
|---|---|---|
| v0.1 | 2026-09-20 | 初稿。Windows 打包路径端到端实测（wheel 名称、wheel 内含库、PE 导入表、114 导出符号）；sdist 的实测内容作为"不发 sdist"的依据；Linux manylinux 与 macOS 两条路径给出方案并标注**未实测**；许可证/可见性与凭据纪律单列 |
