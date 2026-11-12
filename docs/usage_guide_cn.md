# U-Arm + CR5AF + DM4310 整体使用说明

适用版本：GitHub 预发布版 `v0.1.0`，T-CAN485 网关固件 build 4.6。

本说明对应当前原型：U-Arm 主臂通过 FE-URT-2 连接电脑，Dobot CR5AF/CC262 通过网线连接电脑，DM-J4310-2EC V1.2 夹爪电机通过 CR5AF 末端 RS485 和 T-CAN485 接入。

除特别说明外，下面的一键入口和 `prebuilt` 路径以**解压后的 Windows 发布 ZIP**或当前完整交付工作目录为准。GitHub 源码仓库不提交 Python/PlatformIO 构建产物，也不会在仓库根目录直接保存这些生成的一键入口；源码开发者需要先运行 `packaging/build_release.ps1` 生成交付目录。

## 1. 当前 T-CAN485 烧录的是哪个程序

你称为 U-CAN485 的板，在当前工程和发布包中记录为 **T-CAN485（CH9102 USB 串口 + ESP32）**。

当前实际运行的是：

- 固件名称：`guarded gateway build 4.6 reversal-feedback-recovery`
- 源码工程：`firmware/tcan485_dm4310_read_probe/`
- 主程序：`firmware/tcan485_dm4310_read_probe/src/main.cpp`
- 会话状态机：`firmware/tcan485_dm4310_read_probe/src/gripper_session_logic.h`
- 发布包中的固件：`firmware/tcan485_dm4310_read_probe/prebuilt/firmware.bin`
- 完整开发工作目录中的编译文件：`firmware/tcan485_dm4310_read_probe/.pio/build/esp32dev/firmware.bin`
- SHA-256：`BEEF14C39410AA1D4C057CC33DA6659185CBF9101F7EA2515F26FAA5BE2083BF`
- 网络身份返回值：`4.6`。该数值是固件版本标记，不是电机角度。

`firmware/tcan485_dm4310_gateway/` 是早期 build 0.1 审计程序，不是当前夹爪运行固件。目录名 `tcan485_dm4310_read_probe` 是开发早期留下的名称，里面现在已经包含开合、接触保持、反向、心跳、STOP、CAN 反馈恢复和故障停止逻辑。

2026-11-12 00:31:50 的实机记录确认：build 4.6 烧写成功，重新启动后读回 4.6 身份，DM 电机处于失能、无故障状态。

## 2. 整体通信路径

```text
U-Arm 舵机
  │ TTL 总线
FE-URT-2 / CH343
  │ USB
Windows 电脑
  ├─ Ethernet ─> CR5AF / CC262（29999、30004、60000）
  │                         │
  │                         └─ 末端 RS485 ─> T-CAN485 ─> Classic CAN ─> DM-J4310-2EC
  │
  └─ 本地仿真和控制面板
```

RS485 A/B 不能直接连接 CAN_H/CAN_L。T-CAN485 负责把经过定义的 RS485 请求转换成 DM 电机 CAN 帧。当前 CAN 为 Classic CAN、1 Mbps，电机 ID 为 1。

## 3. 首次安装

1. 从 GitHub Releases 下载 `uarm-cr5af-dm4310-teleop-windows-x64-v0.1.0.zip` 和对应 `.sha256` 文件。
2. 完整解压 ZIP；不要直接在压缩包内运行程序。
3. 可在 PowerShell 中校验文件：

   ```powershell
   Get-FileHash .\uarm-cr5af-dm4310-teleop-windows-x64-v0.1.0.zip -Algorithm SHA256
   ```

   正确值应为：

   ```text
   51F02C7A99F052D058F79B4DD342939136094FD3C63779C5B95D7F7F0C3D361A
   ```

4. 双击 `INSTALL.bat`。安装器会使用包内 Python 3.13.9 和固定版本 wheels，在当前目录建立独立运行环境，不修改系统 PATH。
5. 安装完成后编辑 `config/device_connection.json`：

   ```json
   {
     "robot_host": "192.168.201.1",
     "leader_port": "auto"
   }
   ```

   `robot_host` 是机器人控制器地址。只连接一个 VID:PID 为 `1A86:55D3` 的 CH343 时，`leader_port` 可以使用 `auto`；同时连接多个相同适配器时，应改成设备管理器中确认的端口，例如 `COM12`。

## 4. 每次开机后的检查顺序

1. 检查 CR5AF、U-Arm、夹爪、线缆和工作区域，确认没有机械卡住或线缆缠绕。
2. 确认实体急停可以立即触及。
3. 打开控制柜和所需外部电源，确认 DM4310 与网关供电正常。
4. 确认电脑有线网卡和 `192.168.201.1` 位于同一网段。
5. 在 DobotStudio Pro 中确认机器人处于 TCP 二次开发模式。
6. 双击 `READONLY_DIAGNOSTICS.bat`。它只做设备枚举和网络端口检查，不使能、不运动、不清错、不刷固件。
7. 诊断通过后再启动仿真或遥操。

## 5. 启动仿真

双击：

```text
START_SIMULATION.bat
```

该入口显示 U-Arm 与 CR5AF/夹爪模型，并读取实时反馈。遥操正在使用 FE-URT-2 串口时，不要再单独启动另一个会争用同一串口的仿真进程；优先使用遥操面板内的“打开 3D 实测监视”。

模型显示正确不等于真实机械行程、环境避障和自碰撞已经验证。当前仿真用于姿态、方向、映射和反馈检查。

## 6. 启动真实遥操

1. 让 U-Arm 和 CR5AF 位于已经确认的安全接管姿态。
2. 清空机械臂和夹爪运动区域。
3. 双击：

   ```text
   START_TELEOP.bat
   ```

4. 控制面板打开本身不会使能或移动机器人。
5. 检查面板显示的控制器地址、U-Arm 端口、六轴来源、方向、倍率、速度限制和夹爪状态。
6. 确认现场安全后，在面板中启动持续遥操。
7. U-Arm J1–J6 控制 CR5AF J1–J6；ID7 扳机控制夹爪开合。

当前映射和限位针对这套原型。更换 U-Arm 舵机、机械结构、CR5 安装位置、夹爪、电机零位、网关固件或控制柜后，必须重新标定，不能直接沿用当前参数。

## 7. 夹爪使用逻辑

- 扳机采用全开/全闭两态逻辑，中间死区保持上一次意图，不用于任意比例开度。
- 闭合碰到物体后允许进入低力接触保持，不要求继续运动到机械完全闭合位置。
- 从夹紧切换到松开，或松开一部分后再次夹紧，网关会从最新实测位置重新规划。
- 正常接触和端点保持不会自动失能。
- STOP、真实心跳丢失、连续 CAN 反馈失败、电机故障、硬位置/速度限制、超温或动作超时会停止并失能。
- 软件接触判断不等于力传感器，不能自动区分所有工件接触和机构卡住。

## 8. 正常停止、紧急停止和报警恢复

### 正常停止

点击面板中的“停止并失能”，等待面板显示控制程序结束和失能结果。正常停止后不要直接假设机械臂已经回到安全零位；根据现场姿态决定是否执行经过确认的回零流程。

### 紧急情况

如果出现失控趋势、碰撞风险、程序无响应、线缆被拉扯或人员进入运动区域，立即按实体急停。软件按钮不替代实体急停。

### 报警恢复

1. 先解除真实碰撞、限位、夹持卡住、供电或接线原因。
2. 保持人员离开机械臂运动路径，确认回安全位的整条路径畅通。
3. 再使用控制面板的恢复功能或经过批准的单次恢复程序。
4. 不要在原因未解除时连续重复清警或回零。

## 9. T-CAN485 固件升级

正常安装和打开遥操面板不会刷写固件。当前板已经是 build 4.6 时，不需要重复烧写。

发布包使用 `src/flash_gripper_gateway.py` 作为受保护升级器。它只接受唯一的 CH9102 网关，不会把 U-Arm CH343 当成刷写目标；烧写前还会检查：

- CR5 模式为 4、未使能、未运行、无报警；
- DM4310 无故障、低速、温度低于限制；
- 固件 SHA-256 与已审核 build 4.6 完全一致；
- 遥操进程没有占用同一控制锁。

只读预检命令：

```powershell
.\.venv\Scripts\python.exe .\src\flash_gripper_gateway.py
```

只有确实需要升级，并且现场满足烧写条件时，才使用：

```powershell
.\.venv\Scripts\python.exe .\src\flash_gripper_gateway.py `
  --flash `
  --confirm I_UNDERSTAND_TCAN485_FIRMWARE_WILL_BE_REPLACED
```

当前升级器只把已审核应用程序写到 ESP32 的 `0x10000`，不会重新编译，也不会自动重写 bootloader 和分区表。它适用于已经配置好的现有网关；全新空白板不能直接使用该升级流程。

烧写结束后程序会重新读取 4.6 身份和 DM 失能/无故障状态，并把结果写到 `logs/gripper_gateway_flash_*.json`。

## 10. 发布包中的常用文件位置

| 用途 | 文件 |
|---|---|
| 一键安装 | `INSTALL.bat` |
| 只读诊断 | `READONLY_DIAGNOSTICS.bat` |
| 启动遥操 | `START_TELEOP.bat` |
| 启动仿真 | `START_SIMULATION.bat` |
| 离线测试 | `RUN_OFFLINE_TESTS.bat` |
| 连接设置 | `config/device_connection.json` |
| 遥操面板 | `src/uarm_cr5_live_panel.py` |
| 连续控制 | `src/uarm_cr5_continuous_teleop.py` |
| 夹爪主机协议 | `src/gripper.py` |
| 固件升级器 | `src/flash_gripper_gateway.py` |
| 当前网关源码 | `firmware/tcan485_dm4310_read_probe/` |
| 当前预编译固件 | `firmware/tcan485_dm4310_read_probe/prebuilt/firmware.bin` |

## 11. 当前版本边界

- `v0.1.0` 是当前原型的首个公开预发布版。
- 软件不是安全控制器，不提供安全认证、环境避障或完整自碰撞保护。
- 六轴范围、速度、比例、夹爪端点和保持参数仍属于设备专用标定。
- GitHub Release 中的 ZIP 与校验文件是普通用户入口；GitHub Packages 当前不使用。
- 后续修改控制逻辑、标定或固件后，应使用新的版本号发布，不覆盖已经发布的 `v0.1.0`。
