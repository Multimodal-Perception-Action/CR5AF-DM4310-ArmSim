# U-Arm → CR5AF → DM4310 完整联调顺序

## 固定系统架构

- U-Arm ID 1–6 → FE-URT-2/USB/COM7 → Windows PC → CR5AF J1–J6；
- U-Arm ID 7 → FE-URT-2/USB/COM7 → Windows PC → Ethernet/TCP60000
  → CC262/CR5AF末端RS485 → T-CAN485 → Classic CAN 1 Mbps → DM-J4310-2EC夹爪；
- CR5AF实时状态只读入口为 TCP30004；运动入口与夹爪入口必须分开限权。

ID7不得进入六轴 `ServoJ` 数组，ID1–6不得生成夹爪命令。

## 已完成

- FE-URT-2 COM7、STS3215 ID1–7、1 Mbps通信；
- CR5AF TCP30004、1440-byte V4反馈解析；
- 官方CR5 URDF/MuJoCo数字孪生及真实反馈显示；
- CR5AF末端RS485至T-CAN485至DM4310链路；
- DM4310固定开、闭、停动作及动作结束后失能验证；
- CR5 J1–J6逐关节真实冒烟测试：每轴+1°并返回、速度/加速度5%，12次动作全部通过，
  最大目标误差约0.00132°，最终Stop、Disable且无报警；
- U-Arm ID1–6到CR5 J1–J6逐轴端到端跟随全部通过：每轴硬限±1°、速度≤3°/s，
  其他五轴固定；全部会话均回基线、Stop、Disable且无报警；
- U-Arm ID7重复往复500次的原始活动范围：2005–3017 ticks；
- 操作者重新确认物理最小角度即完全闭合端；实测为2966 ticks（40次稳定采样、峰峰值0）；
- 机械结构角度增大表示张开，但ID7原始编码随之减小。程序以2966为物理0°，先换算
  相对张开角，再进行夹爪映射；
- 舒适最大张开端实测为2591 ticks（40次稳定采样、峰峰值0）；完全闭合端2966 ticks；
  有效行程375 ticks（32.958984375°编码角），两个物理端点均已确认；
- 映射、固定全开/全闭和安全停止均已实机验证；硬件输出只允许受限的开、闭、停动作。
- 2026-11-10 二进制扳机跟随完成首次真实链路测试：闭合动作完整通过；张开动作已启动并在
  扳机反转时安全停止，最终DM4310失能且无故障。完整全开跟随仍需在扳机保持张开端时复测。
- 随后保持ID7在2422 ticks完成全开复测：DM4310到达-6.26974 rad，目标误差约0.011 rad，
  最终失能且无故障。该30秒窗口内ID7没有返回闭合区，因此未触发闭合动作。
- 最终保持ID7在2971 ticks完成闭合复测：DM4310由全开到达-1.12631 rad，闭合目标误差
  约0.00493 rad，最终失能且无故障。二进制扳机全开/全闭端到端联动均已通过。
  固定开闭硬件输出门禁已根据上述通过证据解锁。
- 已完成单进程六轴加夹爪控制器 `src/uarm_cr5_full_teleop.py`。2026-11-10 的20秒实机
  组合验收收到400个20 Hz样本与400个成功的 `ServoJ` 回复；J1–J6均实际到达各自
  ±2°硬限，夹爪完成闭合并完成两次反向安全停止。随后37个周期回到基线；CR5最终
  Mode 4、未使能、无报警，DM4310最终 `disabled/no fault`。证据为
  `logs/uarm_cr5_full_teleop_20261110_170412.json`。
- 完成200次静态标定：ID1–7峰峰值全部为0 tick；ID1–6配置2 tick输入死区。完成50轮
  电源/状态采样：无通信或数据包错误、无电压报警、温度26–28°C、扭矩全部关闭。

## 已完成的门禁

1. DM4310空闲状态轮询、故障与失联检测已恢复并用于联调前后检查；
2. ID1–6方向已逐轴实机验证；
3. MuJoCo合成输入和U-Arm输入均已完成只读运行；
4. ID7单通道全开/全闭、反向停止已通过；
5. CR5六轴各自低速±1°跟随均已通过；
6. 六轴加夹爪单进程控制链路已完成20秒组合验收。

## 最终运行程序应具备

- 启动前确认7个舵机型号、状态、扭矩关闭和反馈新鲜度；
- ID1–6与ID7严格分流；
- 角度解包、会话零位、方向、2 tick死区、速度和位置限幅；
- CR5反馈超过250 ms未更新立即停止输出；
- U-Arm或RS485/CAN失联立即停止并使DM4310失能；
- 默认仅仿真/预览，真实输出必须显式解锁；
- 全程记录输入、映射目标、机器人反馈、夹爪反馈和停止原因；
- 程序退出时释放COM和所有TCP Socket。

## 当前剩余限制

- 电压寄存器回报8.8–8.9 V；操作者已明确接受，软件范围设为5.0–9.0 V，状态位、
  电压报警、温度、通信和扭矩检查仍保留；
- 夹爪当前网关固件完成一次全行程约10.5秒。更快的受限固件已经离线构建，但必须先让
  网关自身的CH9102 USB串口重新枚举；COM7是U-Arm，禁止误刷。
- 8.7–8.9 V设备寄存器读数保留为合并提示 `UARM_VOLTAGE_READBACK`，不会被隐藏；
  若状态位、电压报警位或5.0–9.0 V硬范围任一不满足，程序仍立即拒绝运行。

## 运行方式

默认只读预演（不会使能CR5，不会动作夹爪）：

```powershell
python .\src\uarm_cr5_full_teleop.py --duration 5
```

受限实机运行（每轴相对启动姿态硬限±2°、速度≤3°/s，最长30秒）：

```powershell
python .\src\uarm_cr5_full_teleop.py --duration 20 --live-output --enable-robot --confirm I_UNDERSTAND_FULL_UARM_CR5_WILL_MOVE
```

每次退出都会尝试夹爪STOP、CR5回启动基线、`Stop()`、`DisableRobot()`，并记录最终状态。

## ID7标定证据

- 最大张开：`logs/uarm_pose_gripper_max_open_20261110_154754.json`；
- 完全闭合：`logs/uarm_pose_gripper_fully_closed_20261110_155330.json`；
- 人手舒适范围往复记录：`logs/uarm_pose_gripper_operator_comfort_range_20261110_155934.json`；
- 张开点动阈值：`logs/uarm_pose_gripper_open_jog_trigger_20261110_160254.json`；
- 修正后的物理最小/完全闭合端：`logs/uarm_pose_gripper_physical_min_fully_closed_corrected_20261110_160630.json`；
- 确认后的舒适最大张开端：`logs/uarm_pose_gripper_comfort_max_open_confirmed_20261110_160945.json`；
- 只读映射工具：`src/uarm_gripper_preview.py`；
- 标定配置：`config/uarm_gripper_calibration.json`。
