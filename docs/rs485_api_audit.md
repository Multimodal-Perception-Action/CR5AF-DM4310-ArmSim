# CC262 / 末端 RS485 API 审计（未连接任何测试器、未发送命令）

## 范围与证据

检索了官方 V4.5.1、V3（2025-05-08）协议 PDF、两个官方 Python SDK，以及工作区内 Dobot 《如何使用末端的485》资料。没有向控制器、末端 RS485、Modbus 或 Lua 发送任何数据。

| API / 能力 | V4 官方 TCP/IP 证据 | 本审计结论 |
| --- | --- | --- |
| `GWrite()` / `GRead()` | V4/V3 PDF、V4 SDK 全文均无 TCP 命令；《如何使用末端的485》第 1–2 页将其列作**脚本指令** | **Lua 原始字节读写有资料证据；不是 V4 TCP Dashboard API** |
| `SetToolBaudRate()` | V4/V3 PDF、V4 SDK 全文均无 TCP 命令；《如何使用末端的485》第 1 页将其列作**脚本指令** | **Lua 设置末端波特率有资料证据；不是 V4 TCP Dashboard API** |
| `SetTool485(baud, parity, stopbit[, identify])` | V4 PDF 第 54 页；`official/TCP-IP-Python-V4/dobot_sdk/api/io.py:288-302` | 受支持的**配置**命令；会改末端端口配置，未调用 |
| `SetToolMode(...)` | V4 PDF 第 55–56 页；`io.py:317-332` | 受支持的**复用端子模式设置**；会改配置，未调用 |
| `ModbusRTUCreate(...)` | V4 PDF 第 59 页；`communication.py:39-59` | 受支持的 RS485 **Modbus RTU 主站创建**；会建立通信配置，未调用 |
| `SetHoldRegs(...)` | V4 PDF 第 62 页 | 写 Modbus 保持寄存器；禁止且未调用 |

## 原始 RS485 透传与 Lua

官方 V4 文档将 `SetTool485` 描述为设置末端 RS485 的波特率/校验/停止位，将 `ModbusRTUCreate` 描述为“基于 RS485 创建 Modbus master”，随后提供标准 Modbus 寄存器读写命令。它没有记录任意原始字节的 TCP 命令，也没有记录 `GWrite/GRead`。所以目前结论是：

1. 不能用本地 Python 经 V4 Dashboard 直接把任意 RS485 字节写到末端；Python 只能经 29999 调用**已文档化** Dashboard 命令。
2. 官方资料证明 TCP 可调用 `SetTool485`/`ModbusRTUCreate`，但它们不是原始串口透传且是写/配置操作；当前非 TCP 模式下预计会被同样拒绝，尚未测试。
3. `RunScript` 的确是官方 Dashboard API（V4 SDK `robot_control.py:75-81`），但会运行工程，违反本次只读范围。它可作为“已审核 Lua 工程”的技术启动入口，却不是当前安全许可下可调用的入口。
4. CC262 的官方 CRA/CRAF V3.1 硬件手册已确认控制柜 RS485（485A/485B）主要用于 Modbus RTU，末端 M8 的 Pin 1=485A/AI_1、Pin 2=485B/AI_2，且与两路 AI 复用。两者为不同物理接口；是否可同时使用仍要受固件/资源占用限制，尚未通电验证。

## Lua 原始字节接口的本地资料证据

《如何使用末端的485》第 1 页写明末端 485 可用“脚本”和“TCP 的数据透传”控制；其中脚本 API 为 `SetToolBaudRate(baudRate)`（资料称理论 2400–2.5M）、`GRead(0x01)`（返回数据 table 与 size）和 `GWrite(0x01,{...})`。第 2–5 页的 Modbus RTU 示例也用 `GWrite`/`GRead` 组装/读取原始字节。该资料未给出当前 V4 TCP 透传的命令名、端口、帧边界或 V4.6 固件兼容性，因此 TCP 透传本身仍为**未验证**，不能据此自行发包。

## 对 CAN 网关的影响

若目标是用末端 RS485 对接 CAN 电机，只有在网关明确接受你的自定义串口协议并能映射到 DM4310 所需 CAN 帧时才可能工作。现有官方 Dobot API 仅记录 Modbus RTU，故不能把 ZQWL-RCAN-1C311 的“格式转换模式”视为已验证方案。先拿到网关完整数据手册，并在隔离的 RS485 测试器上验证；不得直连 RS485 A/B 与 CAN_H/L，详见 `can_direct_connection.md`。

CC262 原生 CAN 是否能开放给 DM4310：本次官方 TCP V3/V4 文档没有提供用户 CAN 帧收发 API，结论是**尚未验证，不能开放使用**。在取得 CC262 专用 CAN 接口官方文件及 Dobot 书面确认前，不接入该电机。
