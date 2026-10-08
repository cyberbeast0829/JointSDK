# 把 AI 调测能力装到你的工具里

> 面向**使用 JointSDK 的工程师**：不管你的 AI 工具是 VS Code Copilot、Codex、Claude、
> Cursor 还是别的，本文说明要把哪几个东西放到哪里。
>
> 只想自己改 SDK 代码的人看 **`CONTRIBUTING.md`**；想知道"AI 该怎么调关节"看 **`AGENTS.md`**。

---

## 0. 三个交付物，三种分发方式

| 交付物 | 作用 | 它是什么形态 | 怎么分发 |
|---|---|---|---|
| **`AGENTS.md`**（仓库根） | **纪律** —— 关节的领域规则（写后必读回、窄类型会静默丢帧、单位陷阱…） | 一个 Markdown 文件 | 放进仓库根，**多数工具自动读** |
| **`.agents/skills/joint-bringup/`** | **脑** —— 把规则编排成可执行流程 | Agent Skills 标准目录 | 复制/软链到你的工具的 skills 目录 |
| **`mcp/jsdk_joint_mcp/`** | **手** —— 16 个工具，能实际读写关节 | Python 包（任何 MCP 客户端可用） | **随仓库克隆**（不必安装 Python 包）+ 在客户端里配 MCP server |

三者**缺一不可**：
只有 MCP → agent 有能力但不知何时用、容易误用（尤其写操作与动电机）；
只有 Skill → 有流程但碰不到设备；
只有 AGENTS.md → 是文档，不是能力。

⚠ 三者**都不替你下结论**：工具如实报告，判断由 AI（和你）做。

---

## 1. 前提：让 `jsdk_can` 能被加载

MCP server 通过 **JointSDK 的 Python 绑定**访问设备，所以先要有一个能用的共享库。

```bash
# 在 JointSDK 仓库里构建（MinGW 例）
cmake -S . -B bsh -G "MinGW Makefiles" -DJSDK_WERROR=ON -DJSDK_ENABLE_HEAP=ON -DJSDK_BUILD_SHARED=ON
cmake --build bsh

# 装 Python 绑定
pip install -e bindings/python
```

⚠ 包内自带 `lib/libjsdk_can.dll`（或 `.so`），所以**通常不用**设 `JSDK_LIB_PATH`。

**自检**（无硬件也能跑）：

```bash
cd mcp
python -m jsdk_joint_mcp.selftest        # 应输出 79/79 通过
```

**排查"我到底在跑哪个库"**：

```bash
python -c "from jsdk_can import _abi; print(_abi.loaded_path())"
```

> ⚠ 真踩过：Windows 上 `jsdk_can.dll`（MSVC）与 `libjsdk_can.dll`（MinGW）可能同时存在，
> 加载器原先偏好前者 ⇒ **改了 C 代码、重建了、行为一点没变**。已修（按 mtime 择优），
> 但这条命令能让你随时确认。

---

## 2. 装 Skill（`joint-bringup`）

### 2.1 为什么放在 `.agents/skills/`

Agent Skills 是**开放标准**（[agentskills.io](https://agentskills.io/)），但**各家读的目录不重合**：

| 工具 | 仓库内读取的路径 | 个人（用户级）路径 |
|---|---|---|
| **Codex** | `.agents/skills/`（从 CWD 一路扫到仓库根） | `~/.agents/skills/`、`/etc/codex/skills` |
| **VS Code / Copilot** | `.github/skills/`、`.claude/skills/`、**`.agents/skills/`** | `~/.copilot/skills/`、`~/.claude/skills/`、`~/.agents/skills/` |
| **Claude Code** | `.claude/skills/` | `~/.claude/skills/` |

⇒ **`.agents/skills/` 是唯一的中立选择**：Codex 原生读，VS Code 也读。
（本仓库原先放在 `.github/skills/`，那只对 Copilot 有效，已移出。）

⚠⚠ **不要在多处各放一份副本**：VS Code 明确说明——同名 skill 出现在多个受支持位置时
**两份都会出现**（不是覆盖），会变成重复条目。**保持单一副本**。

### 2.2 仓库内（推荐：团队共享）

什么都不用做 —— `.agents/skills/joint-bringup/` 已在仓库里。
你在 JointSDK 仓库里打开 AI 工具，**Codex 与 VS Code Copilot 会直接发现它**。

### 2.3 Claude Code 用户

Claude Code **只**读 `.claude/skills/`。在你的项目里加一个软链或副本：

```bash
# 软链（推荐：改一处即可，不会漂移）
mkdir -p .claude/skills
ln -s ../../.agents/skills/joint-bringup .claude/skills/joint-bringup   # Linux / macOS
# Windows（需开发者模式或管理员）：
mklink /D .claude\skills\joint-bringup ...\.agents\skills\joint-bringup

# 或者直接拷贝（简单，但要记得重新同步）
cp -r .agents/skills/joint-bringup .claude/skills/
```

### 2.4 装在**用户级**（不克隆仓库也想用）

技能文件夹是自包含的，拷到用户目录即可在所有项目里生效：

```bash
SKILL=<JointSDK>/.agents/skills/joint-bringup

mkdir -p ~/.agents/skills  && cp -r "$SKILL" ~/.agents/skills/     # Codex
mkdir -p ~/.claude/skills  && cp -r "$SKILL" ~/.claude/skills/     # Claude Code
mkdir -p ~/.copilot/skills && cp -r "$SKILL" ~/.copilot/skills/    # VS Code / Copilot
```

⚠ **只拷 skill 是不够的**：skill 里的流程会让你调 `jsdk_*` 工具（**手**），
而那些工具来自本仓库的 MCP server。所以要同时满足：**skill 已装** + **MCP server 已配**（第 3 节）。
只装 skill 的表现是——agent 知道该做什么，但伸手都是空的。

⚠ 本仓库目前**不发 PyPI**：MCP server 随仓库克隆使用。
所以“用户级装 skill”适合**你已经有 JointSDK 仓库**、只是想在别的项目里也用上它的场景。

> ⚠ 用户级与仓库级**同时存在**时，会出现两份 —— 按上面的表格选**一种**即可。
> （Claude Code 对指向同一目标的软链会去重；仓库级路径在 `~/.claude/skills/` 面前
> 优先级更低，所以用户级会覆盖。）

### 2.5 验证

| 工具 | 怎么看装好了 |
|---|---|
| VS Code | 命令面板 `MCP: List Servers` 旁的 skill 列表，或聊天框输入 `/` 找 `joint-bringup` |
| Codex | `/skills` |
| Claude Code | `/skills` |

**能触发**的提问示例：*"帮我看看 COM4 上 node 1 的关节状态"*、*"标定过不去"*、
*"参数写不进去"*。

⚠ **发现 ≠ 一定会调用**：各家都是按 `description` 匹配，不保证每个相关提问都会触发。
需要时直接点名：`/joint-bringup`。

---

## 3. 接 MCP server

### 3.1 通用部分

命令与参数在所有客户端里都一样，只是**配置文件的写法不同**：

```
命令: python
参数: -m jsdk_joint_mcp
工作目录: <JointSDK>/mcp
环境变量: PYTHONIOENCODING=utf-8      # ⚠ 中文 Windows 必需
           JSDK_PYTHON_SRC=<JointSDK>/bindings/python/src   # 未 pip install 时使用
```

⚠ `PYTHONIOENCODING=utf-8` **不是可选项**：中文 Windows 默认 cp936，
工具返回里出现 `⇒` 这类字符会让 server **直接崩掉**（客户端只看到"连接断开"）。

### 3.2 VS Code / Copilot

项目级 `.vscode/mcp.json`（本仓库已附）：

```json
{
  "servers": {
    "jsdk-joint": {
      "type": "stdio",
      "command": "python",
      "args": ["-m", "jsdk_joint_mcp"],
      "cwd": "${workspaceFolder}\\mcp",
      "env": { "PYTHONIOENCODING": "utf-8",
               "JSDK_PYTHON_SRC": "${workspaceFolder}\\bindings\\python\\src" }
    }
  }
}
```

⚠ VS Code 用 **`servers`**（不是 `mcpServers`），且要 `"type": "stdio"`。

### 3.3 Codex

`~/.codex/config.toml`（或项目级 `.codex/config.toml`）：

```toml
[mcp_servers.jsdk-joint]
command = "python"
args = ["-m", "jsdk_joint_mcp"]
cwd = "D:\\projects\\cheetah\\JointSDK\\mcp"

[mcp_servers.jsdk-joint.env]
PYTHONIOENCODING = "utf-8"
JSDK_PYTHON_SRC = "D:\\projects\\cheetah\\JointSDK\\bindings\\python\\src"
```

验证：`codex mcp list`

### 3.4 Claude Code

```bash
claude mcp add jsdk-joint -- python -m jsdk_joint_mcp
```

若需指定工作目录与环境变量，改用它支持的配置文件写法（各家版本差异较大，
以 `claude mcp` 的帮助为准）。

### 3.5 其它 MCP 客户端

绝大多数客户端用同一套 `mcpServers` JSON：

```json
{
  "mcpServers": {
    "jsdk-joint": {
      "command": "python",
      "args": ["-m", "jsdk_joint_mcp"],
      "cwd": "/path/to/JointSDK/mcp",
      "env": { "PYTHONIOENCODING": "utf-8" }
    }
  }
}
```

⚠ **先确认你的客户端支持本地 stdio**：部分产品只支持远程 HTTP/SSE。
本 server 目前**只实现 stdio** —— 需要远程传输的话要另外加（见 `mcp/README.md`）。

### 3.6 验证

```bash
cd <JointSDK>/mcp
echo '{"jsonrpc":"2.0","id":1,"method":"tools/list"}' | python -m jsdk_joint_mcp
```

应返回 16 个 `jsdk_*` 工具。**这一条能区分"server 的问题"还是"客户端配置的问题"**。

---

## 4.`AGENTS.md` 怎么被读到

放在**仓库根**，多数工具会自动加载：

| 工具 | 约定 |
|---|---|
| Codex | `AGENTS.md`（原生） |
| VS Code / Copilot | `AGENTS.md`（"Open standard"，也支持 `.github/copilot-instructions.md`） |
| Cursor / 其它 | 多数已支持 `AGENTS.md` |

⚠ **Claude Code 以 `CLAUDE.md` 为主**。若它没有加载 `AGENTS.md`，
在仓库根加一个 `CLAUDE.md` 指向它即可（一行）：

```markdown
@AGENTS.md
```

⚠ **不要两个都写内容** —— 会漂移。让 `CLAUDE.md` 只做引用。

---

## 5. 装完先跑什么

```bash
cd mcp
python -m jsdk_joint_mcp.selftest                                    # 无硬件：79/79
python -m jsdk_joint_mcp.hw_smoke --channel COM4 --node 1            # 真机只读：24/24
```

真机细节（含标定、写操作验证）见 **`mcp/TESTING.md`**。

⚠ **标定会驱动电机**（转十几圈电气角，约 30 s），跑之前确认关节固定牢、周围无障碍。

---

## 6. 一句话

**`AGENTS.md` 放仓库根（自动生效）；Skill 放 `.agents/skills/`（Codex/VS Code 直接可用，
Claude Code 加一步）；MCP server 用同一套命令配到各客户端（注意 `PYTHONIOENCODING=utf-8`）。**
