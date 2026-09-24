# CI（只读验证型）：两个 job，命令清单只有一份

> 目标：**把 `tools/check_all.sh` / `tools/_check_all_body.sh` 跑起来**，而不是在 CI 里
> 再抄一份命令。抄出来的第二套清单迟早与本地跑偏，于是出现"CI 绿了、本地红了"。

## 1. 为什么 CI 只做"只读验证"

| 能进 CI | 不进 CI（保持人工） |
|---|---|
| 虚拟后端上的全部 C 测试 / 示例 / 冒烟 | **真机**：`tools/hw_verify.sh --if slcan`、`tools/py_hw_smoke.py`（需要适配器 + 关节 + 终端电阻 + 人工确认急停） |
| Linux（socketcan 编译）/ ASan + UBSan / 共享库 / Python | 位定时、error-frame/bus-off、真驱动器运动（`PORTING.zh-CN.md` §7.5.3 B 段） |
| 合并件 / Arduino / 打包冒烟、三个静态守卫 | 现场环境差异（适配器固件、线缆长度） |

CI 里**没有任何写设备的动作**；`hw_verify.sh` 的 `L8/L8b` 写路径探针不会被执行（它根本不在 CI 步骤里）。

## 2. 三个 workflow（`.github/workflows/verify.yml`）

| job | runner | 跑什么 |
|---|---|---|
| `full-matrix` | **Windows 自托管**（`[self-hosted, Windows, X64]`） | `./tools/check_all.sh` —— 完整 **14 步**（Windows 三套构建 + Python + 3 冒烟 + 3 守卫 + WSL 里的 Linux/ASan/共享库/Python 四步） |
| `linux-native` | `ubuntu-latest` | 没有自托管机时的兜底：`bash tools/_check_all_body.sh "$PWD" "$(nproc)" 1`（同一个脚本，Linux 侧四步）+ `./tools/check_all.sh --only lint-cli-text,lint-api-docs,abi-gap`（同一份命令表） |
| （可选）云效 Codeup | 自托管"机器组" | 见 §4 |

两个 job 都会把 `build/check_all/` 作为 artifact 上传（失败时尤其要看）。

## 3. 自托管 Windows runner 的前置条件

runner 就是开发机（或任意一台同配置的机器），需要：

- **Git-Bash**（`shell: bash` 用；`check_all.sh` 是 bash 脚本）
- **MSYS2 / MinGW-w64 gcc**（`mingw32-make` 与 `gcc` 在 PATH 上）
- **VS 2022 Build Tools**（`msvc` 那步用 `Visual Studio 17 2022` 生成器）
- **WSL + Ubuntu**（默认 `Ubuntu-20.04`；`cmake`、`build-essential`、`python3-pytest`）
  - 换发行版：`JSDK_WSL_DISTRO=Ubuntu-22.04 ./tools/check_all.sh`
- **Python 3.12**（Windows 侧）+ `pip install -e "bindings/python[test]"`
- 首次跑会在仓库里建 `build/ bsh/ build-msvc/ build-linux/ build-asan/ bsh-linux/`
  （都在 `.gitignore` 里），耗时约 **10 分钟**（默认 `--no-touch` 关闭，即每次全量重建）

注册 runner：GitHub 仓库 → Settings → Actions → Runners → New self-hosted runner（Windows），
按提示下载并 `./config.cmd`。**只勾选本仓**，不要给组织级权限。

## 4. 云效 Codeup 流水线（`origin`，主仓）

Codeup 的流水线是 YAML 定义 + 机器组，schema 与 GitHub Actions 不同 ⇒ 本仓**不维护两份**：
在流水线里只放"一条 Shell 执行"，命令与本文件 §2 完全一致。

1. 建**机器组**（自托管）：一台满足 §3 前置条件的 Windows 机器；
2. 新建流水线 → 源码源 = 本仓库，触发 = 提交到 `main` / 合并请求；
3. 加一个"Shell 执行"步骤：
   ```bash
   bash ./tools/check_all.sh          # 全矩阵；失败即流水线失败（脚本已在失败时非 0 退出）
   ```
   只想跑 Linux 时（若机器组是 Linux）：
   ```bash
   bash tools/_check_all_body.sh "$PWD" "$(nproc)" 1
   ```
4. 产出物加 `build/check_all/`（失败时能直接看到每步日志）。

> 为什么不做成"两边各维护一份 YAML"：`check_all.sh` 的派生结果（哪些步骤、什么顺序、
> 什么失败判据）只应该有一个来源 —— 这也正是它取代"每轮手工敲十几条命令"的原因。

## 5. 本地等价命令

CI 绿不等于本机绿（反之亦然）：两边跑的是同一个脚本，所以本机先跑同样的入口即可。

```bash
./tools/check_all.sh                 # 全 14 步（Windows + WSL）
./tools/check_all.sh --no-wsl        # 只 Windows 侧
./tools/check_all.sh --only win-build,py-win
./tools/check_all.sh --list
```

真机部分（**人工**，见 `CLI.zh-CN.md` §8 与 `PORTING.zh-CN.md` §7.5.3 B 段）：

```bash
tools/hw_verify.sh --if slcan --channel /dev/ttyACM0 --runs 3 --write-probe
python3 tools/py_hw_smoke.py
```
