# 真机测试 MCP Server — 完整步骤

> 目标：让 Codex / VS Code (Copilot) / 豆包 通过 MCP 帮你调关节模组。
> 本文假设：Windows + CANable（slcan）+ 一台关节。

---

## ✅ 已实测通过（2026-10-08）

| 检查 | 结果 |
|---|---|
| 适配器 | CANable → `COM4`（"USB 串行设备"） |
| 关节 | node 1、`hw 262711`、**`fw 1547`**、`serial 108005767394384`、**FD** |
| `selftest`（无硬件） | **79/79** |
| `hw_smoke`（真机，只读） | **24/24** |
| **写操作**（真机） | **3/3** —— uint32 / bool / uint16 全部 `verified:true` |
| **标定**（真机，动电机） | ✅ **30.9 s** 跑完并真正回到 IDLE |
| 描述符下载 | ✅ 38433 B 级，`descriptor_loaded=true` |
| 交叉校验 | ✅ 单读与批量读的 `gear_ratio` 均为 `7.75` |

### 一键脚本

```bash
cd /d/projects/cheetah/JointSDK/mcp
python -m jsdk_joint_mcp.selftest                                 # 无硬件：79/79
python -m jsdk_joint_mcp.hw_smoke --channel COM4 --node 1         # 真机只读：24/24
python -m jsdk_joint_mcp.hw_write_probe --channel COM4 --node 1   # 真机写（含恢复）
python -m jsdk_joint_mcp.hw_state_poll --channel COM4 --node 1    # 只读：看状态轨迹
python -m jsdk_joint_mcp.hw_calibrate_probe --channel COM4 --node 1 --yes
                                              # ⚠ 真机标定（动电机 ~30 s；不加 --yes 会拒绝）
```

⚠ 跑真机脚本前先确认加载的是**当前构建**的库：

```bash
python -c "from jsdk_can import _abi; print(_abi.loaded_path())"
```

> 真踩过：包内同时存在 `jsdk_can.dll`（MSVC）与 `libjsdk_can.dll`（MinGW），
> 加载器偏好前者 ⇒ **重建后行为一点没变**，差点得出"修复无效"的错误结论。

### 本次真机读到的值（**仅作"某一次测量"的记录，不是规格**）

⚠ 按 `AGENTS.md` §2.2，下面每个数都**必须现场重读**：

| 项 | 本次值 | 备注 |
|---|---|---|
| `gear_ratio` | `7.75` | 与旧记录一致 |
| `torque_constant` | **`0.0804683`** | ⚠ **第 4 个不同值**（0.0385 → 0.0904 → 0.0824 → 0.0865 → 0.08199 → 0.08047） |
| `mit_max_pos / vel / torque` | `12.5 / 65.0 / 50.0` | `mit_max_torque` 曾被误当 18 |
| `mit_max_kp / kd` | `500.0 / 5.0` | |
| `break_timeout_ms` | `0` | = **超时检测禁用**（不是 100 ms） |
| `heartbeat_rate_ms` | `100` | 路径是 `axis0.config.can.heartbeat_rate_ms` |

---

## 0. 前提检查（**先做这一步，否则后面会报看不懂的错**）

### 0.1 插上 CANable，确认端口

```bash
cd /d/projects/cheetah/JointSDK
python -c "import serial.tools.list_ports as lp; [print(p.device, p.description) for p in lp.comports()]"
```

没插适配器时只有 `COM1`/`COM2`（主板自带）。
插上 CANable 后应该多出 **`COM3` 或更高**。记下这个端口号，后面要用。

### 0.2 确认无硬件时 MCP 能跑

```bash
cd /d/projects/cheetah/JointSDK/mcp
python -m jsdk_joint_mcp.selftest          # 应输出 68/68 通过
```

### 0.3 确认绑定可加载

如果 `selftest` 报 `No module named 'jsdk_can'`，先做一次（任选一种）：

```bash
# 方式 A：安装绑定（推荐，一次就好）
cd /d/projects/cheetah/JointSDK && pip install -e bindings/python

# 方式 B：设环境变量指向源码树
set JSDK_PYTHON_SRC=D:\projects\cheetah\JointSDK\bindings\python\src
```

> ⚠ 包内自带 `lib/libjsdk_can.dll`，所以**通常不需要**设 `JSDK_LIB_PATH`。

---

## 1. 先看关节有没有反应（**不开 AI，先手动确认链路**）

⚠ **这一步不能跳。** 如果链路本来就不通，走 AI 只会把"链路问题"误诊成"SDK 问题"。

```bash
cd /d/projects/cheetah/JointSDK
./build/jsdk-cli.exe --if slcan --channel COM3 info
```

**读法（`AGENTS.md` §3.1）：**

| 结果 | 含义 | 下一步 |
|---|---|---|
| 打印出 hw/fw/serial | ✅ 链路通 | 继续第 2 步 |
| `0/0 bytes` + 有帧数 | 通道通、**我们的请求丢了** | 正常（首帧丢失），重跑一次 |
| `0/0 bytes` + **0 帧** | 链路/格式问题 | 查端口、权限、Classic/FD |
| 超时且心跳收得到 | **Classic/FD 猜错** | 加 `--classic` 再试 |

确认能读之后，再往下。

---

## 2. 配置 MCP Server

**三家的配置文件位置不同，但内容一样。** 先把这段准备好：

```json
{
  "mcpServers": {
    "jsdk-joint": {
      "command": "python",
      "args": ["-m", "jsdk_joint_mcp"],
      "cwd": "D:\\projects\\cheetah\\JointSDK\\mcp",
      "env": {
        "PYTHONIOENCODING": "utf-8",
        "JSDK_PYTHON_SRC": "D:\\projects\\cheetah\\JointSDK\\bindings\\python\\src"
      }
    }
  }
}
```

> ⚠ `cwd` **必须**是 `...\JointSDK\mcp`（否则 `-m jsdk_joint_mcp` 找不到模块）。
> ⚠ `PYTHONIOENCODING=utf-8` 是必需的：中文 Windows 默认 cp936，
> 输出里的非 cp936 字符会崩在**打印点**（看起来像测试失败）。
> ⚠ 如果你已经 `pip install -e bindings/python`，`JSDK_PYTHON_SRC` 可以删掉。

### 2.1 VS Code (GitHub Copilot)

**方式一：工作区配置（推荐，跟项目一起走）**

在 `D:\projects\cheetah\JointSDK\.vscode\mcp.json` 写入：

```json
{
  "servers": {
    "jsdk-joint": {
      "type": "stdio",
      "command": "python",
      "args": ["-m", "jsdk_joint_mcp"],
      "cwd": "${workspaceFolder}\\mcp",
      "env": {
        "PYTHONIOENCODING": "utf-8",
        "JSDK_PYTHON_SRC": "${workspaceFolder}\\bindings\\python\\src"
      }
    }
  }
}
```

> ⚠ VS Code 用 **`"servers"`**（不是 `"mcpServers"`），并且要 `"type": "stdio"`。

**方式二：用户级配置** —— 命令面板 `MCP: Open User Configuration`。

**验证：**
1. 命令面板 → `MCP: List Servers` → 应看到 `jsdk-joint`，状态 running
2. 打开 Copilot Chat → 模式切到 **Agent**
3. 点工具图标 → 应看到 16 个 `jsdk_*` 工具

**第一次对话建议这么问：**

```
用 jsdk_connect 连接 iface=slcan channel=COM3 node=1，
然后 jsdk_get_state 读原始错误位，按 AGENTS.md §3.4 的格式汇报：
分开"我看到的"和"我推断的"。
```

### 2.2 Codex (OpenAI Codex CLI)

Codex 的 MCP 配置在 `~/.codex/config.toml`（或项目级 `.codex/config.toml`）：

```toml
[mcp_servers.jsdk-joint]
command = "python"
args = ["-m", "jsdk_joint_mcp"]
cwd = "D:\\projects\\cheetah\\JointSDK\\mcp"

[mcp_servers.jsdk-joint.env]
PYTHONIOENCODING = "utf-8"
JSDK_PYTHON_SRC = "D:\\projects\\cheetah\\JointSDK\\bindings\\python\\src"
```

> ⚠ TOML 里反斜杠要转义（`\\`），或者用正斜杠 `D:/projects/...`。

**验证：**

```bash
codex mcp list          # 应列出 jsdk-joint
```

然后在会话里直接说：

```
连接关节（COM3 / node 1），读状态并告诉我有没有故障位
```

### 2.3 豆包

豆包（及其桌面客户端）对 MCP 的支持方式**随版本变化**，且**不一定支持本地 stdio** ——
有的版本只支持**远程 HTTP/SSE** MCP。**先确认你的版本支持哪种**，两条路：

**路线 A：如果支持本地 stdio 配置** —— 配置内容与上面一致，
按豆包的文档找到 MCP 配置文件位置（通常在设置 → 扩展/MCP 里）。

**路线 B：如果只支持远程 MCP** —— 需要给 server 加一个 HTTP 传输层。当前骨架**只实现了 stdio**，
要用这条路得先扩展 `server.py`（加一个基于 `http.server` 的 SSE 端点）。
告诉我你要走 B，我来加。

> ⚠ 无论哪条路，**先跑通 §0/§1**，否则在豆包里排错会更难（它的日志不如前两家直观）。

---

## 3. 第一次真机对话（建议流程）

**不要一上来就让它动电机。** 按 `AGENTS.md` §3.1 的顺序走：

### 第 1 轮：只读，不出手

```
先用 jsdk_connect 连上（slcan / COM3 / node 1），再调用 jsdk_get_state。
按 AGENTS.md §3.4 格式汇报：分开"我看到的"（原始位图/帧计数）和"我推断的"。
```

**你应该看到**结构化输出：`error_raw`、各子系统 `*_error_hex`、`current_state_name`、
`session.tx_frames` / `rx_frames` / `tx_retries`。

⚠ **如果 `tx_retries > 0`，问它这意味着什么** —— 正确回答是"链路上确实丢过帧"，
而不是"设备坏了"。

### 第 2 轮：交叉校验（验证它真的在核数据）

```
用 jsdk_read_param 单读 axis0.motor.config.gear_ratio，
再用同一工具同时读 [gear_ratio, torque_constant]，
对比两次的值是否一致。不一致说明什么？
```

⚠ **这一步是"有牙齿"的测试**：SDK 会预热+重发，但如果链路在丢帧，两次读数会不一致。
**一致才是正常的**；不一致的话，让 agent 解释——它应该说"链路在丢帧"，而不是"参数是错的"。

### 第 3 轮：读量程（写探针的前置）

```
用 jsdk_probe_ranges 读量程，然后告诉我：
如果我要写一个 MIT 力矩指令，安全范围是多少？
```

**关键看它有没有说这三件事：**
1. 量程是**从设备读的**（不是它记的）
2. MIT 的 pos/vel/tau 是**对称量程** ±max，kp/kd 是**无符号** [0, max]
3. **越界不钳位而是回绕**（会反向满力矩）

如果它漏了第 3 条，说明 `AGENTS.md` 没被读到——检查 agent 有没有加载它。

### 第 4 轮：安全的写（验证读回）

选一个**无副作用**的参数，比如回写原值：

```
用 jsdk_probe_ranges 拿到当前的 heartbeat_rate_ms，
然后用 jsdk_write_param 把它写成同一个值。
检查返回的 requested / value / verified 三个字段，说明它们各是什么意思。
```

⚠ **它必须说清楚 `requested` 不是结果**。如果它说"写入成功，因为 requested=100"，
那就是踩了 `AGENTS.md` §1.1 禁止的坑。

### 第 5 轮：故障诊断（如果设备有故障）

```
用 jsdk_get_fault_detail 读六类故障分项。
如果 error_raw 非零，告诉我具体是哪一类，以及下一步查什么。
```

---

## 4. 安全警告（**请务必读**）

### 4.1 绝对不要在没确认的情况下让它动电机

`jsdk_calibrate` / `jsdk_home` 会**真的转动电机**。server 已经加了安全闸：

- 不给 `confirm=true` → 直接拒绝（返回 `SAFETY_REFUSED`）
- 但**给不给 `confirm` 是你的决定** —— agent 可能会问你

⚠ **第一次跑标定前，确认：**
- 关节**固定牢**、周围无人无障碍
- 你**知道它会转**（全标定要转十几圈电气角，真机实测 29.5 s）
- 电源/急停**随手可及**

### 4.2 `CURRENT_CONTROL` 的安全缺口

如果你的测试涉及纯电流控制（`jsdk` 的 `set_current`）：

> ⚠ **纯电流控制的客户端，CAN 掉线也不会停机**（固件 `is_ctrl` 不含 `CURRENT_CONTROL`）。
> 这是一个真实的安全缺口。

### 4.3 不要相信"ok: true"就代表一切正常

本 server 的设计是**如实报告**，不是**替你做判断**：

| 字段 | 含义 |
|---|---|
| `ok: false` + `error` | **设备告诉我们一件事**（不是工具坏了） |
| `verified: false` | 写进去的值没读回来 —— **先数帧长，不是先怀疑设备** |
| `verified: null` | 该端点写进去就被状态机消费（如 `requested_state`），**不算失败** |
| `pre_calibrated_both: false` | 标定跑完但没生效，**这是正常行为**（要人工写 + save） |
| `summary`（遥测） | **可核查的统计**，不是结论 —— 自己判断 |

---

## 5. 排错

| 症状 | 原因 | 修法 |
|---|---|---|
| `No module named 'jsdk_can'` | 路径没配对 | §0.3（`pip install -e` 或设 `JSDK_PYTHON_SRC`） |
| MCP 里看不到工具 | 配置格式错 | VS Code 用 `"servers"`，别家用 `"mcpServers"` |
| 工具报 `INTERNAL` + `ModuleNotFoundError` | 同上 | 看错误里的 `hint`，它会列出排查过程 |
| 连接报 `0/0 bytes, 0 frames` | 端口/权限/格式 | 先跑 §1 的 CLI 确认 |
| 连接报 `0/0 bytes, N frames` | **首帧丢失**（正常） | 重试一次；或看 `session.tx_retries` |
| 中文输出乱码/崩溃 | cp936 | 确认 `env.PYTHONIOENCODING=utf-8` |
| 日志里 `framing_learned=4` | 显式配置与对端冲突 | 去掉 `classic` 参数重新连 |

### 看 server 自己的日志

MCP 的 stdout 是协议流，**不能**混调试输出。要看 server 行为：

```bash
cd /d/projects/cheetah/JointSDK/mcp
echo '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"jsdk_get_state","arguments":{"node":1,"iface":"slcan","channel":"COM3"}}}' | python -m jsdk_joint_mcp
```

这能直接看到 server 返回的原始 JSON —— 排"是 server 的问题还是客户端的问题"最有效。

---

## 6. 什么算"测试通过"

| 检查 | 通过标准 |
|---|---|
| ① 无硬件自检 | `68/68 通过` |
| ② CLI 直连 | `info` 能打印 hw/fw/serial |
| ③ MCP 工具可见 | 客户端里能看到 16 个 `jsdk_*` 工具 |
| ④ 只读调用 | `jsdk_get_state` 返回**结构化**字段（不是一句话） |
| ⑤ 交叉校验 | 单读 vs 批量读**数值一致** |
| ⑥ 写后读回 | 返回 `requested`/`value`/`verified` 三字段，且 agent 说清三者区别 |
| ⑦ 安全闸 | 未确认的 `jsdk_calibrate` 被**拒绝** |
| ⑧ 诊断质量 | 问到 `err_name=CAN_TIMEOUT` 时，它会去读**原始位**而不是查波特率 |

**⑦ 和 ⑧ 是最有区分度的两条** —— 它们检验的是 `AGENTS.md` 是否真的被 agent 用上了，
而不只是"MCP 能连上"。

---

## 7. 有问题怎么报

告诉我在哪一步、把**原始输出**贴出来（不要转述）。特别是：

- `selftest` 的输出
- server 返回的完整 JSON（用 §5 里那条命令）
- agent 的原始回答（不是你的总结）

⚠ 如果是"agent 回答得不对"（不是连接失败），告诉我它**具体说了什么** ——
那可能是 `AGENTS.md` 或工具 description 需要改进的地方，而不是 bug。
