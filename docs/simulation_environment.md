# U-Arm → Dobot CR5AF 数字孪生环境

## 当前实现

第一版仿真采用 Windows 原生 MuJoCo，不依赖 ROS、Gazebo 或 WSL。模型来自 Dobot 官方
`TCP-IP-ROS-6AXis` 仓库提交
`0924e785f1827b7125ba4ba2bc3325811ed21c18` 中的 `cr5_robot.urdf` 和 CR5 DAE 网格。
运行时会把 DAE 转换为 OBJ，并在 `simulation/generated/cr5/` 生成 MuJoCo 可载入的 URDF。

`src/cr5_mujoco_twin.py` 有三种只读数据源：

- `synthetic`：完全离线的合成关节轨迹；
- `dobot`：只接收 `192.168.201.1:30004` 的真实关节反馈；
- `uarm`：只读 COM7 上 ID 1–7，并把 U-Arm 相对关节变化映射到 CR5 模型。

程序没有 29999 Dashboard 运动客户端，没有使能、清错、切换模式或 ServoJ 实现。

## 安装和启动

```powershell
python -m pip install -r .\requirements-simulation.txt

# 离线自检
python .\src\cr5_mujoco_twin.py --source synthetic --duration 5 --gui

# 显示真实 CR5 当前姿态（30004只读）
python .\src\cr5_mujoco_twin.py --source dobot --duration 60 --gui

# U-Arm 驱动虚拟 CR5，真实 CR5不运动
python .\src\cr5_mujoco_twin.py --source uarm --duration 60 --gui
```

去掉 `--gui` 即为无窗口自动测试；每次运行都在 `logs/` 保存 JSONL。

## 已验证结果

- 官方 URDF 能被 MuJoCo 4.0.0 载入：6个关节、7个刚体；
- 合成轨迹、30004真实反馈、COM7 U-Arm映射三种输入均已运行成功；
- Dobot 数据源以 20 Hz 连续运行100帧并完成可视化；
- 同一组真实关节角下，URDF Link6 与控制器 TCP 的姿态角差小于约0.4°；
- Link6 与当前控制器 TCP 位置相差约11.8 mm，需通过 CR5AF法兰/工具坐标偏置校准；
- 全部测试向 Dobot发送0条命令，向U-Arm写入0次。

## 最终验收状态

真实联动前依次达到：

1. U-Arm 单关节人工变化能在虚拟 CR5上得到正确方向和比例；
2. 六个关节的方向、零位、软限位和跨零点连续性全部通过；
3. 根据当前工具配置补偿约11.8 mm的 Link6/TCP偏置；
4. 仿真中验证限位、速度限制、反馈超时和断联停止；
5. 实测 U-Arm VCC–GND 电压并关闭供电疑点；
6. 最后才在隔离工作区、低速、小范围下解锁真实 CR5联动。

官方 CR5模型可以验证当前 CR5AF的六轴运动学和控制链路，但它没有单独建模 CR5AF力传感器、
实际末端夹具和现场环境。这些部分需要按实物尺寸继续加入，才能成为碰撞级数字孪生。
