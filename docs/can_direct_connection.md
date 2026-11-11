# RS485 与 CAN 不能直连

禁止将 RS485 A/B 接到 CAN_H/CAN_L。两者虽然都是差分信号，但物理层、电气规范、仲裁/帧格式和终端要求不同；这不是针脚映射问题。直连可能导致通信失败，且不应假定收发器具有容错保护。

DM4310 的 CAN 总线应使用正确的 CAN 控制器/收发器、适合的供电与共地策略、两端各一个 120 ohm 终端电阻（仅在总线两物理端），并先按达妙官方手册确认位速率、ID、帧格式及电机安全条件。

如确需由 RS485 一侧控制 CAN 设备，使用专用、隔离优先的 RS485-to-CAN 网关；网关必须明确支持所需的串口封包到 CAN 帧转换。ZQWL-RCAN-1C311 是否合适，尚待其数据手册验证：串口格式、CAN 2.0A/B、位速率、帧 ID/数据映射、缓存延迟、隔离和供电均须匹配。它不能让 CC262 原生 CAN 自动成为用户 CAN API。

## 阶段 D：CC262 原生 CAN 可行性结论

结论：**未发现公开、受支持的 CC262 用户自定义 CAN 报文收发 API；因此当前判定为不支持将 CC262 CAN_H/CAN_L 直接用于达妙 DM4310。** 不接线、不发帧、不修改控制器底层固件。

核查范围是 Dobot 官方 V4.5.1/V3 TCP/IP 协议 PDF、官方 V4/V3 Python SDK、公开产品资料和工作区内官方《Dobot CRA系列硬件使用手册 V3.1》。V4/V3 TCP 协议仅公开 Dashboard、运动、实时反馈、I/O、末端 RS485/Modbus 等接口；其全文、SDK 全文均没有 `CAN`、`CAN_H`、`CAN_L`、CAN frame 或用户 CAN send/receive 命令。更关键的是，CRA/CRAF V3.1 手册的控制柜 I/O 表直接将 `CAN_H`/`CAN_L` 写为：**“CAN总线接口（内部预留，不对外开放）”**。这适用于手册列出的 CR5AF。

这不是“CAN 硬件不存在”的结论，而是官方手册已明确“内部预留、不对外开放”。不能将其接入 DM4310，也不能尝试底层固件修改。只有 Dobot 针对 **CC262 + 当前固件** 书面开放用户 CAN API 和端子电气规范（权限、仲裁、位速率、终端、故障隔离）后，才能重新评审。

来源：

- 官方 [V4 TCP/IP protocol](https://github.com/Dobot-Arm/TCP-IP-Protocol-6AXis-V4) 与 [V4 Python SDK](https://github.com/Dobot-Arm/TCP-IP-Python-V4)
- 官方 [V3 TCP/IP protocol](https://github.com/Dobot-Arm/TCP-IP-Protocol-6AXis-V3) 与 [V3 Python SDK](https://github.com/Dobot-Arm/TCP-IP-Python-V3)
- Dobot 官方 [CC262/CC263 maintenance-tool release note](https://www.dobot-robots.com/insights/news/917.html)
