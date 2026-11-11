# U-Arm（FE-URT-2）到 Dobot CR5AF 二次开发记录

## 关节与夹爪分流

- U-Arm 舵机 ID 1–6：只映射 Dobot CR5AF 的 J1–J6；
- U-Arm 舵机 ID 7：独立作为夹爪开合输入，不得放入 CR5 六轴 `ServoJ` 数组；
- ID7 数据路径固定为：FE-URT-2/USB → Windows PC → Ethernet → CC262/CR5AF
  → 末端 RS485 → RS485-CAN 网关 → Classic CAN → DM4310夹爪电机；
- 早期静态端点为 `2895/2962 ticks`，但后续人手舒适范围往复测试证明这只是操作行程中的
  一小段，因此不再作为控制端点；
- 操作者重新确认物理最小角度就是完全闭合端；只读实测为 `2966 ticks`，40次采样峰峰值0；
- 机械结构角度从该位置增大时逐渐张开，但ID7原始编码从2966向约2500减小。因此程序使用
  `physical_opening_ticks = 2966 - current_ticks` 表示人手物理张开角；
- 舒适最大张开端实测为 `2591 ticks`，40次采样峰峰值0；完全闭合端为 `2966 ticks`，
  有效行程375 ticks（32.958984375°编码角），两个端点均已确认；
- 控制采用绝对位置映射而非增量累加，并在物理两端饱和。未发送任何硬件输出。标定状态见
  `config/uarm_gripper_calibration.json`。

## 实际连接与识别结果

- Dobot：Windows 的 ASIX 有线网卡 `192.168.201.100` 到控制器 `192.168.201.1`。
- U-Arm：FE-URT-2/CH343 为 `COM5`，STS 总线为 `1,000,000 baud`。
- `COM5` 上 ID 1–7 均能响应，返回模型号 `777`，与 STS3215 对应。
- ID 1–7 的扭矩开关、模式和 Homing Offset 也以只读方式记录在预览日志的
  `servo_health` 字段中，未改写这些寄存器。
- `COM6` 的 CH9102 不是这条 U-Arm 总线；此前对它的无响应不代表 U-Arm 故障。
- Dobot TCP/29999 与 TCP/30004 均连通，30004 实测收到符合 V4.5.1 布局的 1440 字节反馈。
- Dashboard 只读实测：`GetAngle()`、`GetPose()` 和 `GetErrorID()` 均返回错误码 0；
  `GetErrorID()` 返回空列表。关节角与 30004 的 `QActual` 相符。

## 官方 U-Arm 源码审计

审计版本：`MINT-SJTU/LeRobot-Anything-U-Arm` commit
`bd31c691e0818367e93a743bb49be4abaaff8b59`。

官方 `feetech_servo_reader.py` 不能直接作为安全的读取程序运行：

1. 启动时对 ID 1–7 解锁 EEPROM；
2. 清零 Homing Offset；
3. 读取当前位置并计算相对 2047 的偏移；
4. 把新偏移写回 EEPROM 后重新上锁；
5. 运行时按 `(current - zero) / 4096 * 360` 发布角度。

官方 `servo2Dobot.py` 将舵机 ID 1–6 依次映射到 Dobot J1–J6，只反转第 2 轴，
然后加到固定初始关节角 `[180, -20, -96, 62, 93, 205]`。其构造函数会先让机器人
运动到该固定姿态；配套 `api.py` 构造函数还会自动发送 `PowerOn()` 与
`EnableRobot()`。因此这些原脚本没有在当前设备上直接启动。

## 本地安全实现

`src/uarm_dobot_preview.py` 只进行以下操作：

- 从 COM5 查询 ID 1–7 的型号、当前位置、电压、温度和状态；
- 从 Dobot TCP/30004 被动接收实时反馈；
- 以启动瞬间的 U-Arm 与 Dobot 姿态为共同零点，离线计算 J1–J6 目标预览；
- 第 2 轴使用官方示例的反向规则；ID 7 只输出夹爪相对角度；
- 写 JSONL 日志。

程序不包含 Dobot 29999 发送逻辑，不写舵机寄存器，也没有运动开关，因此误用参数也不能
触发机器人运动。

## 当前阻塞项

七个 STS3215 均读取到 `3.6–3.7 V`，状态寄存器均为 `1`。STS 内存表定义状态位
BIT0 为电压异常，当前因此不能把总线判为可执行运动的健康状态。需要先使 U-Arm 舵机总线
获得符合所装 STS3215 版本规格的外部电源，再重新只读检查电压和状态。

U-Arm Global Version 的官方 Hardware Guide 把 Dobot 所属 Config 2 的 BOM 写为 **7 个
ST-3215-C046（7.4 V、1:147）**，电源项标注为 **5VOUT**。本地 C046 A/1 规格书第 3 页
给出的工作电压范围为 **5–8.4 V**，第 5 页说明低于 5 V 会进入保护。因此这次的
3.6–3.7 V 不是可接受的低端正常值。总线返回的型号号 777 只能确认 STS3215 家族，不能
单独证明后缀 C046；最终型号仍以舵机标签为准。

官方 FE-URT-2 接线说明要求舵机使用独立外部电源并与电脑、调试板共地；USB 主要承担通信
和调试板供电。当前读数说明外部舵机电源缺失、未打开、规格不符或在线路上发生了明显压降，
但仅靠软件无法区分这几种物理原因。

2026-11-10 14:34 再次检查时，Windows 仍把 CH343 列为 `COM5` 且设备管理器问题码为 0，
但打开串口返回 Windows 错误 31（“连接到系统上的设备没有发挥作用”）。这发生在此前多次
成功只读之后，说明 URT-2/USB 链路随后出现了设备级掉线或驱动通信异常。本次没有自动禁用、
重启设备或结束任何进程；失败记录已写入 `logs/uarm_readonly_probe.jsonl`。COM6 仍不是这条
U-Arm 总线，不能用它代替 COM5。

随后尝试通过 Windows `pnputil` 仅重启该精确 CH343 设备实例，系统返回 `Access is denied`，
设备没有被重启；再次只读打开 COM5 仍返回错误 31。当前会话没有管理员权限，软件侧不能再
完成该设备复位。

当前安装的是 WCH `ch343ser.inf` 版本 `2.1.2025.7`（2025-07-19，Microsoft WHQL 签名）。
WCH 官方下载页当前提供 CH343SER 2.2（2026-06-26）。由于同一驱动在本轮早些时候曾连续
成功读取 7 个舵机，应先通过物理重新插拔确认是否只是设备/USB Hub 的临时失效；若错误 31
持续存在，再由管理员安装 WCH 官方 2.2 驱动。不要安装面向 CH340/CH341 的 CH341SER 包。

程序现已按 VID:PID `1A86:55D3` 和序列号 `5C84288247` 自动识别 U-Arm 适配器。即使重新
枚举后的端口号变化，也不会把 VID:PID `1A86:55D4` 的 COM6 当作 U-Arm。

即使电压恢复，首次联动仍需先完成：确认 Dobot TCP 控制模式、核对各关节方向和软限位、
为 ID 7 建立夹爪开合标定，以及加入速度/步长限制和失联停止策略。

## 联动控制器的硬锁

`src/uarm_dobot_teleop.py` 已离线实现真实 `ServoJ` 输出路径，但默认只做预览。实际输出同时
要求配置文件中的 `motion_unlocked`、`mapping_validated`、`power_validated` 全部为 true，
并要求显式的 `--live-motion` 与确认字符串。当前三项均为 false。

控制器不会自动执行 `PowerOn`、`EnableRobot`、`ClearError` 或 `RequestControl`。它使用
CR5AF 手册的关节范围 J1/J2/J4/J5/J6 ±360°、J3 ±160°，额外保留 5° 软件裕量；首次测试
限制相对位移为每轴 10°、J1–J3 为 5°/s、J4–J6 为 8°/s，并把单次实时运行限制为 10 秒。
反馈失联、状态变化、目标越界或 U-Arm 读取失败时终止输出；只有已经进入真实输出后才调用
`Stop()` 做收尾。

## 证据来源

- U-Arm 官方仓库：<https://github.com/MINT-SJTU/LeRobot-Anything-U-Arm>
- FEETECH STS 内存表：<https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/>
- FE-URT-2：<https://ftservo.github.io/ftservo-wiki/tools/adapters/>
