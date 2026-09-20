# `cyberbeast-joint-sdk`（Python 绑定）

CyberBeast Joint SDK 的 ctypes 绑定：加载 `libjsdk_can`，把 C ABI 包成 Python API。
**没有编译扩展**，装完就能用；**非实时**（GIL + 调度抖动），定位是配置、采集、可视化、
原型验证。硬实时控制请用 C/C++。

## 30 秒上手

```bash
cd bindings/python
pip install -e ".[test]"

# 无硬件：用内置虚拟后端（推荐先这样确认链路通）
python examples/01_scan_and_health.py
python -m jsdk_can scan --json
```

真机：

```bash
python examples/01_scan_and_health.py --if socketcan --channel can0
python examples/04_mit_hold_and_move.py --if socketcan --channel can0 --arm   # 会让电机出力
```

## 库从哪来

绑定需要一个**共享库**（`jsdk_can.dll` / `libjsdk_can.so`）。按以下顺序找：

1. 环境变量 `JSDK_LIB_PATH`（可以是文件，也可以是目录）
2. 包内 `jsdk_can/lib/`
3. 仓库里的 `build/`、`build-shared/`、`build/Release/`

构建共享库：

```bash
cmake -S . -B bsh -G "MinGW Makefiles" -DJSDK_BUILD_SHARED=ON -DJSDK_BUILD_PYTHON=ON
cmake --build bsh
```

`JSDK_BUILD_PYTHON=ON` 会把库复制进 `src/jsdk_can/lib/`，这样 `pip install .`
出来的包自带库，用户不用再设环境变量。

**加载时会做 ABI 自检**：`sizeof`/对齐与后端名逐项比对，不一致立刻抛
`AbiMismatchError`，而不是等你踩内存。

## 最小控制回路

```python
from jsdk_can import Context, VirtualHal

with Context(VirtualHal(), period_ns=2_000_000) as ctx:
    j = ctx.add_joint(1)
    ctx.configure()                  # 下载描述符 → 解析端点 → 读回标定量并校验
    print(ctx.dump_config().gear_ratio)     # 16.5

    ctx.activate()                   # 阻塞：使能序列 + 安全首帧
    for _ in range(1000):
        ctx.cycle_begin()
        j.set_mit(pos=0.10, vel=0.0, kp=0.5, kd=0.05, tau=0.0)
        ctx.cycle_end()
        ctx.pace()                   # 桌面节拍；cycle_end() 自己不等待
    ctx.deactivate()                 # 阻塞：hold → 等 2 周期 → STOP → 等 IDLE
```

## 四条必须知道的事
1. **`enable()` / `disable()` 是请求，不是动作。** 返回时设备还没变。等"确实就绪"
   用 `ctx.activate()` / `ctx.deactivate()`（阻塞）。实测 `disable()` 要 5 个控制周期
   才落地 —— 在那之前 `is_enabled()` 仍是 `True`。
2. **第一帧 MIT 必须是全零/保持位。** SDK 的使能序列已经替你做（发当前位置 + kp=kd=0），
   自己绕过使能序列直接发运动指令就会跳。
3. **`cycle_end()` 不会等周期。** 要按 `period_ns` 走就调 `ctx.pace()`，并用
   `ctx.pace_stats()` 确认没有长时间落后。
4. **不要凭记忆写端点 ID。** 实测跨固件版本 86% 的端点 ID 会漂移，SDK 全部现读现用
   （`ctx.lookup()` / `ctx.endpoints()`）。

## 后端速查

```python
VirtualHal("0:id=1,fd")             # 无硬件（默认）
SocketCanHal("can0")                # Linux；先用 ip link 配好链路
PcanHal("PCAN_USBBUS1")             # Windows/macOS，运行期加载 PCANBasic
SlcanHal("COM5", 115_200, 5_000_000)  # 串口速率、FD 数据段速率
```

- **slcan 支持 CAN FD**（CANable 2.0 的 `b/B/d/D` 帧前缀）。`data_bitrate`：`0` = 不碰适配器
  配置，`2000000`/`5000000` = 主动发 `Y2`/`Y5`，其它值 → `JsdkUnsupportedError`（`Y<n>` 是固件
  私有表，不猜）。`hal.fd_frames()` 能告诉你对端是不是按 Classic 回的。
- slcan 吞吐 100~500 fps，只适合配置/监控/低速控制。

## 目录

| 路径 | 说明 |
|---|---|
| `examples/` | 5 个可运行示例，全部支持 `--if virtual` |
| `tests/` | pytest 套件（124 项），用虚拟后端，不需要硬件 |
| `src/jsdk_can/_abi.py` | ctypes 结构体 + 函数签名 + ABI 自检 |

详细设计见 `docs/PYTHON.zh-CN.md`，完整方案见 `docs/DESIGN.zh-CN.md` §7.2。

## 测试

```bash
JSDK_LIB_PATH=/path/to/bsh python -m pytest tests/ -q
```

接了 CMake 的构建里也会自动注册成 ctest 目标 `python_bindings`
（`JSDK_BUILD_SHARED=ON` 且装了 pytest 时）。
