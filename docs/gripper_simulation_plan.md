# CR5AF + DM-J4310 夹爪控制与仿真收敛方案

## 已有资产

- Dobot 官方 CR5 URDF、7 个 DAE 网格、惯量和 MoveIt/Gazebo 配置：
  `official/TCP-IP-ROS-6AXis`，commit
  `0924e785f1827b7125ba4ba2bc3325811ed21c18`。
- 达妙官方 DM-J4310-2EC STEP：
  `official/DM-J4310-2EC/models/3d/DM-J4310-2EC-V1.1_3d_20260228_1.stp`。
- 本地夹爪/安装件 STL、STEP/BREP 和 SO-101 夹爪 URDF。SO-101 URDF 只能用于
  几何参考，不能替代当前 DM-J4310 夹爪的传动和端点。

CR5AF 没有在当前官方 ROS 仓库中单列；仿真第一版可用 CR5 官方模型做机器人本体
基线，但在确认 CR5 与 CR5AF 的法兰、末端力传感器外形和惯量差异前，不能声称是
精确 CR5AF 动力学模型。

## 最终命令接口

目标是一个 Windows PowerShell 入口：

```powershell
python .\src\gripper.py status
python .\src\gripper.py jog +0.02
python .\src\gripper.py open
python .\src\gripper.py close
python .\src\gripper.py stop
python .\src\gripper.py open --dry-run
```

`--dry-run` 与真机命令共享相同的软限位和目标生成逻辑，但只输出状态迁移、目标角度
和 MIT 帧，不连接 192.168.201.1。真机执行必须先读取状态，异常时不使能；执行结束
或通信异常时失能。

## 还需一次机械标定

1. 用小步命令确认电机正方向对应夹爪张开还是闭合。
2. 记录全开和全闭时的只读电机角度。
3. 把两点写入配置，留出机械安全余量；`open/close` 只在这两个软限位内运行。
4. 将电机角映射为 URDF 夹爪关节值，随后把夹爪固定到 CR5 法兰坐标系。

完成这两点标定后，通信层不再改变，只需生成高层命令和仿真映射。
