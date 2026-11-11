# DM4310 V4 与 RS485-to-CAN 网关审计（只读资料核查）

## 最新实机结果（2026-11-10 15:33，有源测试已获用户授权）

只读链路确认后，用户明确授权电机使能和转动。T-CAN485 已刷入受限 build 0.8；
固件只接受电机 ID 1 的固定测试，不提供任意 CAN 转发、清错、置零或参数写入。

第一次有源测试采用官方 MIT 编码的 `kp=0`、`kd=0`、速度 0、前馈力矩 0：

- 使能帧 `CAN ID 1 / FF FF FF FF FF FF FF FC` 得到状态 1；
- 250 ms 内位置变化为 `0.000000 rad`；
- 失能帧 `... FD` 得到状态 0，随后 `0x7FF/0xCC` 再次确认失能；
- 日志：`logs/dm4310_zero_torque_enable_test_20261110_153038.json`。

第二次有源测试从实时位置缓升到 `+0.02 rad`，再返回起点；参数为
`Kp=10`、`Kd=0.2`、速度 0、前馈力矩 0。实测最大位移 `0.011444 rad`
（约 0.66°），结束偏差 `0.008011 rad`，最终状态 0（失能），无 CAN 总线错误。
日志：`logs/dm4310_tiny_move_test_20261110_153338.json`。

因此以下闭环已经由实机证实：

`Python -> TCP 60000 -> CC262末端RS485 -> T-CAN485 -> Classic CAN 1 Mbps -> DM-J4310 MIT控制 -> 状态反馈 -> 自动失能`

build 0.8 固件 SHA-256：
`A5133CD7462D0AF5387DA3E3555EC01913B09771011B1A1138D97B360BF4F874`。

后续夹爪 `open/close` 封装只缺机械标定：正方向对应开还是合，以及全开/全闭两个
电机角度。现有 SO-101 URDF 的 0–20 mm 行程不属于这套 DM-J4310 机构，不能直接
当作真机端点。

## 当前结论（2026-11-10 15:13，链路已打通）

电机接入独立 24 V 电源后，先出现一次末端 RS485 无数据。重新显式执行
`SetTool485(115200,"N",1)`，CC262 返回 0，随后无 CAN 输出的 RS485 探针再次
成功到达 T-CAN485。证据分别为：

- `logs/configure_tool485_20261110_151227.json`
- `logs/rs485_gateway_probe_20261110_151228.json`

再次读取 `sw_ver` 时，1 Mbps 首帧即得到 `TX_SUCCESS + RX_DATA`，无发送错误和
总线错误；电机回复 CAN ID `0x000`、数据 `01 00 33 0E 37 30 31 38`。
证据为 `logs/dm4310_read_probe_20261110_151245.json`。

随后按固定白名单读取七个寄存器，七项全部返回：

| 寄存器 | 原始值 | 已确认含义 |
| --- | --- | --- |
| `MST_ID 0x07` | `00 00 00 00` | 反馈 CAN ID = 0 |
| `ESC_ID 0x08` | `01 00 00 00` | 接收/电机 CAN ID = 1 |
| `hw_ver 0x0D` | `34 30 30 56` | 原始 ASCII `400V`；按小端字符串反向显示可解释为 `V004`，需原厂工具确认格式 |
| `sw_ver 0x0E` | `37 30 31 38` | 原始 ASCII `7018` |
| `can_br 0x23` | `04 00 00 00` | 代码 4 = 1 Mbps Classic CAN |
| `sub_ver 0x24` | `39 00 00 00` | uint32 = 57 |
| `Boot_ver 0x25` | `03 02 00 06` | 字节版本 = 3.2.0.6 |

完整证据为 `logs/dm4310_register_audit_20261110_151350.json`。如果按达妙常见
`sw_ver + sub_ver` 方式组合，可能显示为 `V7018_57`；由于公开手册未规定最终
字符串格式，报告保留原始寄存器值，并将组合显示标为推断。

至此已经实际验证：

`PC -> TCP 60000 -> CC262 -> 末端 RS485 -> T-CAN485 -> 1 Mbps Classic CAN -> DM-J4310-2EC V1.2 -> 反馈原路返回 PC`

仍未发送任何机械臂或电机的使能、失能、清错、置零、参数写入或运动帧。

扩展只读审计随后确认：

- `TIMEOUT = 0`（仅记录原始值，手册未在该表中明确单位/零值语义）；
- `CTRL_MODE = 1`，即 MIT 模式；
- `PMAX = 12.5 rad`；
- `VMAX = 30 rad/s`；
- `TMAX = 10 N·m`。

证据为 `logs/dm4310_register_audit_20261110_151521.json`。

最后使用达妙官方 SDK `refresh_motor_status()` 所用的 `0x7FF/0xCC` 只读状态请求，
收到反馈 CAN ID 0、数据 `01 78 04 7F F8 00 1A 17`。按电机自身实测
`PMAX/VMAX/TMAX` 解码为：

- 电机 ID 1；错误码 0，失能且无故障；
- 位置约 `-0.779545 rad`；
- 速度约 `-0.007326 rad/s`，处于 12-bit 量化零点附近；
- 转矩约 `0.002442 N·m`，处于 12-bit 量化零点附近；
- MOS 温度 26 °C；转子温度 23 °C。

证据为 `logs/dm4310_status_probe_20261110_151727.json`。状态请求没有使能电机，
CAN 侧为 `TX_SUCCESS + RX_DATA`，错误计数均为 0。

## 2026-11-10 实机只读验证

测试对象为铭牌 `DM-J4310-2EC V1.2 / 24 V`、LILYGO T-CAN485
（CH9102，COM6）和 CR5AF/CC262 (`192.168.201.1`)。本轮没有发送机械臂
使能、运动、清错、模式切换或工程运行命令，也没有发送电机使能、失能、
清错、置零或运动帧。

### 已验证链路

1. 控制器 TCP `60000` 可连接。
2. 为匹配网关，Dashboard 执行了 `SetTool485(115200,"N",1)`，控制器返回
   `0,{},SetTool485(115200,"N",1);`。这是通信参数设置，不是只读命令，已在
   本次用户明确要求的硬件联调范围内执行；它没有使能或运动机械臂。
3. PC 经 TCP 60000 发送 `D5 4D 01 00 54 45 53 54` 后，T-CAN485 的 COM6
   原样记录 `RS485_RX data=D5 4D 01 00 54 45 53 54`。证据为
   `logs/rs485_gateway_probe_20261110_143210.json`。

由此已实证：`PC -> Ethernet -> CC262 -> CR5AF 末端 RS485 -> T-CAN485 -> ESP32/USB`
整段链路可用。

### DM-J4310-2EC V1.2 单帧只读查询

达妙官方 V1.2 中文手册 V1.4（2026-09-14）第 11 页规定默认 `1 Mbps`、
标准帧；第 15 页规定读取参数帧为 CAN ID `0x7FF`、DLC 8、数据
`CANID_L CANID_H 33 RID 00 00 00 00`；第 18 页规定 `RID 0x0E` 是只读的
`sw_ver`。官方 `motor-sdk` 的 `Python例程/u2can/DM_CAN.py` 也使用相同帧。

网关 build 0.3 只允许上述 `0x7FF/0x33` 参数读取，且 CAN 帧设置为
single-shot；无 ACK 时不会自动重发。对候选电机 ID 1 发送的软件版本查询为：

`CAN ID 0x7FF, STD, DLC 8, DATA 01 00 33 0E 00 00 00 00`

实测结果：

- RS485 请求到达网关；
- `twai_transmit()` 入队成功；
- 随后产生 `TX_FAILED` 和 `BUS_ERROR`；
- `tx_error_counter=8`、`tx_failed_count=1`、`bus_error_count=1`；
- 没有 `TX_SUCCESS`，也没有电机反馈。

证据为 `logs/dm4310_read_probe_20261110_143818.json`。这表示 1 Mbps CAN 帧
没有被总线上的另一节点 ACK。CAN ACK 发生在报文内容筛选之前，因此此时还不能把
问题归因于“电机 ID 不是 1”；应优先检查电机 24 V 供电、CAN_H/CAN_L、CAN
参考地、终端和实际存储波特率。

### 物理层证据和下一步

LILYGO 官方原理图 `official/T-CAN485/project/T-CAN485.pdf` 第 2 页显示：

- CAN 收发器为 SN65HVD231；
- CAN_H 经网络标为 `CAN_A`，CAN_L 经网络标为 `CAN_B`；
- 板上已有固定 `120R` 终端；
- 三针 CAN 端子中间为 CAN 侧参考地 `AGND`。

在断电条件下，先确认 T-CAN485 端子顺序为 `CAN_A / GND / CAN_B`，并确认：

1. `CAN_A -> 电机 CAN_H`，`CAN_B -> 电机 CAN_L`；
2. 电机 VCC/GND 由独立、限流的 24 V 电源供电；
3. T-CAN485 的 CAN 中间 GND 与电机电源 GND/低电位参考点相连；
4. 总线断电后测 CAN_H 与 CAN_L 电阻：T-CAN485 自身已有 120 ohm，不能再无条件
   并入多个终端；短总线两端各 120 ohm 时应约为 60 ohm。

完成这些确认后，先原样重试一次 `sw_ver` 只读帧。只有出现 `TX_SUCCESS` 后，
才有必要读取 `sw_ver`、`sub_ver`、`can_br` 或有限扫描接收 ID。

### 2026-11-10 14:50 Classic CAN 波特率扫描

在 1 Mbps 再次无 ACK 后，网关升级为 build 0.4。它按官方 V1.2 手册列出的
所有 Classic CAN 速率依次测试 `1000/500/250/200/125 kbit/s`，每档只发送一次
完全相同的 `sw_ver` 读取帧，均为 single-shot；任一速率出现 ACK 时会立即停止。

五档实测均为 `TX_FAILED + BUS_ERROR + NO_ACK`，日志为
`logs/dm4310_read_probe_20261110_145022.json`。因此已基本排除“电机仅被改成另一档
Classic CAN 波特率”。在修复物理 ACK 前，不扫描电机 ID，因为 CAN ACK 不依赖
应用层 ID 是否匹配。

剩余优先级：

1. 电机独立 24 V 实际未上电或电源限流/接触异常；
2. `CAN_A/CAN_B` 与 `CAN_H/CAN_L` 接反、接错端子或断线；
3. T-CAN485 的 CAN 中间 GND 未接到电机 24 V GND 参考点；
4. 终端或线束异常；
5. 电机被配置为高于 1 Mbps 的 CAN FD。ESP32 TWAI 与 SN65HVD231 方案不能测试
   2/2.5/3.2/4/5 Mbps CAN FD，需达妙调试串口或 CAN FD 适配器读取/恢复配置。

## 结论先行

工作区中的 ZQWL-RCAN-1C311 V1.7 原厂手册已确认它具备“格式转换”模式：每个 CAN 帧由**固定 13 bytes**串口记录表示，即 `FrameInfo(1) + CAN_ID(4, 高字节在前) + Data(8)`；`FrameInfo` 的 bit3..0 是 DLC、bit7 是扩展帧标志、bit6 是 RTR。故其技术能力足以表示 DM4310 所需的标准 CAN、CAN ID、DLC 8 和八个数据字节。它是可行候选，但不是“已验证可接电机”的方案。

`ZQWL-RCAN-1A442` 在目录及公开检索中均无手册，型号与能力未确认，不能比较或采购。

## 新增 RCANFD 手册核查

新增的《ZQWL-RCANFD系列产品使用手册 V1.0.5》**没有**列出 `ZQWL-RCAN-1A442`。它列出的 RS485 候选包括 RCANFD-1C331（非隔离）和 RCANFD-1C332（通讯隔离），均可在“普通 CAN”模式兼容 CAN 2.0A/B。其格式转换与 1C311 的关键机制相同：普通 CAN 为固定 13-byte 记录，CANFD 为 69-byte 记录；普通 CAN 的 13-byte 记录已经覆盖 DM4310 所需的 8-byte Classic CAN 帧。DM4310 不需要 CANFD，因此 RCANFD 系列不是协议上的必要升级；只有在隔离、串口最高 921600 bps、供货或安装形式要求驱动选型时才值得比较。

该手册还规定 RS485 1200–460800 bps、CAN 10–1000 kbps、标准/扩展帧与 CAN 2.0B（兼容 2.0A）。DM4310 的旧版资料采用固定 1 Mbps、标准 CAN、MIT 控制 DLC=8，因此 C311 的参数范围匹配。注意“透明转换”仅把串行数据按每 8 bytes 分割到**固定配置**的 CAN ID；要承载不同 ID/DLC 或确保回复帧信息，应选择手册第 8.3 节的**格式转换**，而不是透明转换或 Modbus 转换。

## DM4310 协议证据与版本限制

现已从达妙官方仓库取得与实物铭牌匹配的 `DM-J4310-2EC V1.2` 中文手册
V1.4（2026-09-14）。手册确认默认标准 CAN 1 Mbps、读参数 `0x7FF/0x33`、
`MST_ID`、`ESC_ID`、MIT DLC 8 和 `sw_ver/sub_ver/Boot_ver` 等寄存器。
“V4”仍不是官方产品手册中的型号/硬件版本名；实际固件版本只能在 CAN 物理层
恢复 ACK 后读取。

## Dobot 侧 API 分类

| 类别 | 公开 API / 证据 | 原始 RS485 字节？ | 本次动作 |
| --- | --- | --- | --- |
| V4 TCP Dashboard | `SetTool485(baud, parity, stopbit)`；V4 SDK `dobot_sdk/api/io.py:288-302` | 否，只设置通信格式 | 已调用，返回 0 |
| V4 TCP Modbus RTU | `ModbusRTUCreate(slave_id, baud, parity, data_bit, stop_bit)` 与寄存器 API；`communication.py:39-59` | 否，标准 Modbus RTU 主站模型 | 未调用 |
| TCP 末端透明口 | 旧版官方协议定义端口 60000；当前 CC262 实机接受连接 | 是，原始 bytes | 已实测写入并由末端 T-CAN485 收到 |
| V4 Dashboard 原始串口命令 | V4 SDK 仍无 `GWrite` / `GRead` / `SetToolBaudRate` 命令 | 无 | 未调用 |
| Lua | 本地 Dobot《如何使用末端的485》记录 `SetToolBaudRate/GRead/GWrite` | 是，脚本 API | 未运行 |

因此，当前 CC262 并非“只能通过 Lua 才能发原始 RS485”：端口 60000 已经实机验证
可以从本地 Python 透传原始 bytes。`GWrite/GRead` 仍属于 Lua/脚本 API，而不是
V4 Dashboard 命令；本项目没有运行 Lua 工程。

## 若官方 Lua 指南日后证实原始 RS485

在 DobotStudio Pro 中创建/导入 Lua Script project、选择机器人、在无运动/无输出的隔离项目中进行代码审查和保存，是编辑流程；实际“Run/Execute project”会改变控制器项目运行状态，不属于当前许可范围。官方 TCP SDK 暴露 `RunScript("project_name")`，但它也会启动工程，因此**技术上存在入口，不等于当前安全可调用**。未经逐行审核 Lua 且用户显式批准前，不应由 TCP 触发它。

## 最小硬件方案（当前安全结论）

1. **当前实装方案：**CR5AF 485A/485B -> T-CAN485 RS485 A/B；T-CAN485
   CAN_A/CAN_B -> 电机 CAN_H/CAN_L；CAN 中间 GND -> 电机电源 GND 参考点；
   电机使用独立、限流的 24 V 电源。
2. **当前软件：**T-CAN485 固件负责把自定义 RS485 请求转换成受白名单限制的
   CAN 读取帧；不需要运行 Lua，也不需要购买 ZQWL 转换器才能继续当前测试。
3. **当前阻塞：**CAN 侧无 ACK。修复 24 V 供电/线序/参考地/终端后先读版本，
   再决定是否扩展为控制网关。不得把 RS485 A/B 直接接 CAN_H/CAN_L。

来源：

- [Dobot V4 protocol](https://github.com/Dobot-Arm/TCP-IP-Protocol-6AXis-V4) / [V4 Python SDK](https://github.com/Dobot-Arm/TCP-IP-Python-V4)
- Dobot [download center / Lua Syntax Guide (CR)](https://id.dobot-robots.com/service/download-center)
- 达妙 [DM-J4310-2EC official repository](https://github.com/dmBots/DM-J4310-2EC)
- 达妙 [official motor SDK](https://github.com/dmBots/motor-sdk)
- Seeed [Damiao-series integration reference](https://wiki.seeedstudio.com/cn/damiao_series/)
- ZQWL-RCAN-1C311 [manual index](https://www.manuallib.com/file/zhiqwl-zqwl-rcan-1c311-user-manual/) (third-party index; obtain the manufacturer PDF before purchase)
