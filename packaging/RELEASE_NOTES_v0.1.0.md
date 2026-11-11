# U-Arm + CR5AF Teleoperation v0.1.0

首个 Windows x64 公开预发布版，面向当前 U-Arm / FE-URT-2、Dobot CR5AF / CC262 和 DM-J4310-2EC V1.2 夹爪原型。

## 包含内容

- 一键安装与启动入口；依赖和 Python 运行时可离线安装。
- U-Arm 六轴到 CR5AF 的连续遥操控制和 3D 实测监视。
- DM4310 夹爪开合、接触保持、方向反转及有限次数 CAN 反馈恢复逻辑。
- 当前原型标定、仿真模型、T-CAN485 固件源码和已验证固件镜像。
- 离线完整性检查、离线回归测试源码、SHA-256 清单和第三方许可说明。

## 使用方法

1. 下载并完整解压 ZIP，不要直接在压缩包中运行。
2. 双击 `INSTALL.bat`。
3. 编辑 `config/device_connection.json`，确认控制器 IP 和 U-Arm COM 口；只连接一个 CH343 时可以使用 `auto`。
4. 先运行 `READONLY_DIAGNOSTICS.bat`。
5. 现场清空、急停可触及后，运行 `START_TELEOP.bat`。

## 适用边界

这是预发布版，不是安全控制器。包内关节映射、夹爪端点和速度限制针对当前原型；更换机械结构、舵机、夹爪、电机零位、控制柜或固件后必须重新标定。软件不提供环境避障或安全认证。

实体失控、碰撞风险或程序无响应时，应立即使用实体急停。RS485 A/B 不得直接连接 CAN_H/CAN_L。
