# jsdk-joint-mcp — 让 AI agent 帮你把关节模组调好

> 目标不是"帮用户用 JointSDK"，而是"**帮用户把关节调好**"。
> 所以这里暴露的是**关节的物理概念**（错误位图、参数读回、遥测、标定），
> 而不是 SDK 的 API 形状。

---

## 快速开始

```bash
# 1. 确保 Python 绑定已构建（库要能加载）
cd bindings/python && JSDK_LIB_PATH=../../bsh python -m pytest tests/ -q

# 2. 无硬件自检（先跑这个）
cd mcp && python -m jsdk_joint_mcp.selftest

# 3. 作为 MCP server 跑（stdio）
cd mcp && python -m jsdk_joint_mcp
```

### 接到 Claude Desktop / VS Code

```json
{
  "mcpServers": {
    "jsdk-joint": {
      "command": "python",
      "args": ["-m", "jsdk_joint_mcp"],
      "cwd": "/path/to/JointSDK/mcp",
      "env": { "JSDK_LIB_PATH": "/path/to/JointSDK/bsh" }
    }
  }
}
```

---

## 设计原则（都来自真机教训）

### 1. 写后必读回，且**分字段**

每个写操作返回三个字段，**绝不把请求值当结果**：

```json
{
  "requested": 250,     // 我请求写入的
  "value": 0,           // 设备读回的值
  "verified": false,    // 是否一致
  "hint": "先数写帧长度：值宽 <4 的端点不补齐到 >=8 字节会被固件静默丢弃"
}
```

历史缺陷：CLI 曾经对用户说 `written:true`，其实什么都不知道。
这里从类型系统上杜绝。

### 2. 端点 ID 绝不硬编码

跨固件版本约 **86% 的端点 ID 会漂移**。所有 ID 只能从
设备运行时下载的 JSON 描述符里查（`jsdk_get_descriptor`）。

### 3. 量程 / 单位绝不写死

`gear_ratio`、`torque_constant`、`mit_max_torque` 这些**实测变过**
（同一台关节 `torque_constant` 从 0.0824 → 0.0865）。
写探针前必须先读。本 server 不提供任何"典型值"。

### 4. 错误给**原始位**，不给摘要

`err_name = "CAN_TIMEOUT"` 会把**看门狗超时**和**急停**混成同一个名字。
`jsdk_get_state` 返回原始位图 + hex；`jsdk_get_fault_detail` 给六类分项。

### 5. 会话是长寿命的

⚠ 不是每次都开一次总线。原因：

| 真机事实 | 后果 |
|---|---|
| slcan 打开端口后头 1~2 帧上不了总线 | 每次重开 = 每次踩首帧丢失 |
| 描述符 38 KB（FD 1+662 帧 / Classic 1+6906 帧） | 每次重下要 3.5 s |
| 标定要 29.5 s | 必须能被轮询观察 |

### 6. 失败是**结构化数据**，不是"tool failed"

工具内部的失败（写丢了、读不回来）在**正常返回**里带 `ok:false` + `error` + `hint`，
让 agent 能继续推理，而不是看到一个空洞的 "tool execution failed"。

### 7. 安全闸：拒绝执行 优于 猜一下试试

会改变设备状态的操作（`set_zero`、`fault_reset`）必须显式 `confirm=true`。

---

## 工具清单

| 工具 | 风险 | 说明 |
|---|---|---|
| `jsdk_connect` | 无 | 建会话 + 下描述符（安全起步：先按 Classic） |
| `jsdk_disconnect` | 无 | 关会话 |
| `jsdk_list_sessions` | 无 | 列会话及链路统计 |
| `jsdk_device_info` | 无 | hw/fw/serial |
| `jsdk_get_state` | 无 | **原始错误位图** + 轴状态 |
| `jsdk_get_fault_detail` | 无 | 六类故障分项位图 |
| `jsdk_get_health` | 无 | 心跳/新鲜度/重发计数 |
| `jsdk_read_param` | 无 | 按路径读（单条/批量） |
| `jsdk_get_descriptor` | 无 | 端点表（ID/类型/权限） |
| `jsdk_get_telemetry` | 无 | 抓一段时间反馈样本（**只读**） |
| `jsdk_probe_ranges` | 无 | 读量程/换常量（**写探针前必跑**） |
| `jsdk_write_param` | ⚠ 写 | 写 + 强制读回 |
| `jsdk_fault_reset` | ⚠ 动作 | 需 `confirm=true` |
| `jsdk_set_zero` | ⚠ 动作 | 需 `confirm=true`；会改写零点 |
| `jsdk_calibrate` | ⚠⚠ 动电机 | 需 `confirm=true`；全标定 |
| `jsdk_home` | ⚠⚠ 动电机 | 需 `confirm=true`；回零 |

---

## 标定/回零的特殊说明

`jsdk_calibrate` / `jsdk_home` **不重新实现轮询** —— 那是 SDK 内部
`request_state_and_wait()` 的职责，且它已满足全部四条约束：

| 约束 | SDK 实现 |
|---|---|
| 限速轮询 | 首段 20 ms（不漏瞬时态）→ 确认开始后转 200 ms |
| 判据 = 离开静息态 | 不是"等于请求值"（全标定只报子状态 4/7） |
| 空闲只收不发 | 直接 `recv()`，不调 `poll()`（那会补发控制帧） |
| 超时来自时钟 | 不是迭代次数 |

MCP 层只做一件事：**如实转译语义**，并补上 SDK 不做的那一步 ——
结束时读回两个 `pre_calibrated`。

返回里三样收尾证据缺一不可（缺一个就会"谎称成功"）：

```jsonc
{
  "state":  { ... },              // 是否回到静息态
  "fault":  { "subsystems": {} }, // ⚠ 跑完≠成功：失败路径正是"进子态→出错→回静息"
  "pre_calibrated": { ... },       // ⚠ 跑完≠生效：不会自动置位，要人工写+save
  "pre_calibrated_both": false,
  "note": "...（说明这是正常行为，附 save 三步）"
}
```

----

## 与 `AGENTS.md` / Skill 的关系

三者分工：

| 部件 | 是什么 | 给 agent 的 |
|---|---|---|
| `AGENTS.md` | 关节模组的**领域知识**（铁律、物理事实、单位陷阱） | **纪律** |
| 本 MCP server | **能实际读、写设备**的 16 个工具 | **手** |
| `.agents/skills/joint-debug/` | 把上面两者编排成**可执行流程** | **脑** |

⚠ **要给别的 AI 工具的用户用，看 [`docs/AI_AGENT_SETUP.zh-CN.md`](../docs/AI_AGENT_SETUP.zh-CN.md)** ——
skill 目录各家不重合（Codex 读 `.agents/skills/`、Claude Code 只读 `.claude/skills/`），
那份文档给了完整的位置表与安装步骤。

配合方式：agent 从 `AGENTS.md` 学到"值宽 <4 的端点会静默丢帧"，
从本 server 的 `jsdk_write_param` 拿到 `verified:false` + 对应的 `hint`，
从 Skill 的 `references/write-param.md` 知道**下一步按什么顺序查** ——
而不是去怀疑波特率。

⚠ 三者**都不替 agent 下结论**：工具如实报告，判断由 agent 做。

---

## 开发

```bash
# 语法检查
python -m compileall jsdk_joint_mcp

# 无硬件自检（68 项；含标定收尾判定、遥测统计、装箱边界）
python -m jsdk_joint_mcp.selftest

# 手工发一条 JSON-RPC
echo '{"jsonrpc":"2.0","id":1,"method":"tools/list"}' | python -m jsdk_joint_mcp
```

⚠ **stdout 只能有协议消息**。调试输出一律走 stderr。
⚠ 在 Windows 控制台跑（cp936）时，含非 cp936 字符的输出需要在入口
`sys.stdout.reconfigure(encoding="utf-8", errors="replace")`，否则会崩在**打印点**，
看起来像测试失败。
