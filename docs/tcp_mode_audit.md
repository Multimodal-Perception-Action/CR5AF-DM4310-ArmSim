# TCP 控制模式与只读反馈审计（2026-11-04）

## 结论

`Control Mode Is Not Tcp` 是控制器对 Dashboard 命令的应用层拒绝，不是 TCP 连接失败。
官方 V4 SDK 把包含 `Not Tcp` 的回复明确显示为该文本（`official/TCP-IP-Python-V4/demo/DobotDemo.py`, lines 114–116）；官方 V4 README 也要求机器人处于 TCP/IP control mode（lines 307–311）。因此，29999 的 TCP 三次握手可以成功，而当前控制权仍属于 DobotStudio Pro 的非 TCP/IP 二次开发模式。

用户已确认的“IO 远程控制”和“Modbus 远程控制”页面用于以外部触发运行工程，**不是**这里的 TCP/IP 二次开发模式切换入口。官方 V3 Python README 的唯一明确操作说明是：在 DobotStudio Pro 连接机器人后，将 Robot Mode 设为 **TCP/IP secondary development**（`official/TCP-IP-Python-V3/README-EN.md`, lines 229–231；中文 README lines 259–261）。官方 V4/V3 TCP 协议本身未提供一个可安全调用的 `SetControlMode` / `EnterTcpMode` Dashboard 命令。

**未经确认不得切换。** 在 Studio 的机器人模式页人工选择 “TCP/IP secondary development” 后，Studio 的本机控制/工程控制会让出控制权；官方 V3 还注明每次进入 TCP mode 均须调用 `EnableRobot`，且运动中调用会停止当前运动（V3 protocol PDF, EnableRobot note）。这也是本审计没有切换模式、更没有发送 `EnableRobot` 的原因。

## 实测：纯接收 30004

运行：

```powershell
& 'C:\\ProgramData\\Anaconda3\\python.exe' .\\src\\connection_test.py
& 'C:\\ProgramData\\Anaconda3\\python.exe' .\\src\\feedback_monitor.py --packets 3
```

日志：[feedback_30004_20261104_211843.jsonl](../logs/feedback_30004_20261104_211843.jsonl)。程序向 30004 发送 **0 bytes**，只调用 `connect`/`recv`，并正常关闭 socket。

| 项目 | 实际结果 |
| --- | --- |
| TCP/29999、TCP/30004 | 均连接成功 |
| 30004 帧长 | 三帧均为 1440 bytes |
| 帧长字段（offset 0, little-endian uint16） | 1440 |
| 校验 TestValue（offset 48, uint64） | `0x0123456789ABCDEF`，三帧均匹配 |
| 间隔 | 时间戳 8 ms（9537 → 9545 → 9553） |
| `RobotMode` 字段（offset 24） | 5，官方定义为 enabled and idle |

这验证了在非 TCP 控制模式下，30004 实时反馈依然可读；它是控制器主动发送的状态流，不依赖于向 29999 发 Dashboard 命令。V4 协议 PDF 第 100–101 页规定 30004 每 8 ms、每包 1440 bytes、小端；其字段表规定上述 offset。`RobotMode=5` 是一次观测，不是本工具导致的状态改变。

最新 Dashboard 只读查询返回 `0,{5},RobotMode();`。V4.5.1 协议的 RobotMode 表将值 **5** 定义为 `ROBOT_MODE_ENABLE`：机器人已使能且空闲；它不是“TCP 控制模式”的标志。2026-11-04 21:34 的被动 30004 单帧再次观测到 `RobotMode=5`、`enable_status=1`、`running_status=0`、`error_status=0`。原始证据：[feedback_30004_20261104_213447.jsonl](../logs/feedback_30004_20261104_213447.jsonl)。

## V3/V4 与固件版本

官方定位：V4 协议仓库标为 **CRA/E6**，V4.5.1 文档对应 six-axis controller V4.5.1；V3 协议仓库标为 **CR/Nova**，并要求 six-axis controller V3.5.1.19 以上。CR5AF 属于 CR 系列，因此不能仅凭型号或 1440-byte feedback 判定其安装固件的精确版本。

已验证的兼容性证据是：控制器返回的精确字符串被官方 V4 demo 处理，且所收 30004 帧符合 V4.5.1 固定布局。它足以支持“按 V4 解析反馈”的结论，但**不能证明固件精确为 V4.5.1，亦不能排除相近兼容版本**。由于 `RobotMode()` 已被非 TCP 模式拒绝，本次没有发送未文档化版本查询。请在 DobotStudio Pro 的 controller/system information（或控制器标签）读取完整 Controller/Robot firmware version 后再固定 SDK 分支。

## 可用与不可用的只读路径

V4 文档列出 `RobotMode`、`GetAngle`、`GetPose`、`GetErrorID` 为查询命令；但本控制器已证明 `RobotMode()` 在当前模式被拒绝。因此不能假定其余 29999 查询在此模式会成功。`src/robot_status.py` 仅为显式执行时的白名单工具，本次未运行它。已实际验证且安全的读取路径只有 30004。

官方来源：

- [V4 protocol repository](https://github.com/Dobot-Arm/TCP-IP-Protocol-6AXis-V4)（V4.5.1 PDF）
- [V4 Python SDK](https://github.com/Dobot-Arm/TCP-IP-Python-V4)
- [V3 protocol repository](https://github.com/Dobot-Arm/TCP-IP-Protocol-6AXis-V3)（2025-05-08 PDF）
- [V3 Python SDK](https://github.com/Dobot-Arm/TCP-IP-Python-V3)
