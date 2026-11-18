# 相机和夹爪标定

| 项目 | 现状 | 还需要的数据 |
| --- | --- | --- |
| ZED Mini 内参、双目相对位姿 | 厂家提供每台设备、每档分辨率的标定数据 | 在实际相机和运行分辨率上用 SDK 读取，记录使用原始图还是校正图 |
| ZED Mini 到机械臂的外参 | 现场外参待测，横梁安装位置待定 | 最终支架装好后，采集图像和机器人位姿配对 |
| 夹爪电机全开、全闭 | 已有本机端点记录 | 每台复核方向和机械端点 |
| 夹爪开口宽度、TCP | 待测 | 采集电机角度与实测开口，另外测量工具中心和轴向 |

## ZED Mini 固定在横梁上

1. 定好支架和视野后，把标定板刚性固定在末端。记录标定板的格点尺寸、实机相机分辨率、成像通道及内参。开启相机后通过 ZED SDK 读取与采图一致的一套参数。厂家双目标定描述左右相机的相对关系。相机装到机器人系统后的外参需要单独实测。 [Stereolabs 相机标定文档](https://docs.stereolabs.com/docs/development/zed-sdk/modules/camera/camera-calibration)
2. 用静止位姿成对记录图像和机械臂位姿。让标定板覆盖预计工作区域，并改变距离及倾角。至少包含绕两条非平行轴的转动，保留几组姿态用于独立验证。
3. 记 `T_parent_child` 为将 child 坐标转到 parent 的刚体变换。对于固定相机、末端带标定板的情形，用机器人上报的 `T_base_flange` 的逆 `T_flange_base` 作 `calibrateHandEye` 的第一个位姿序列。用图像求得的 `T_camera_target` 作第二个序列。按 OpenCV 的 eye-to-hand 约定，输出为 `T_base_camera`。若最后把相机装在腕上，使用原始的 `T_base_flange` 并求 `T_flange_camera`。输入帧必须与实际图像的光学帧一致。 [OpenCV 手眼标定 API](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html)
4. 用保留姿态验证。固定相机时，由 `inv(T_base_flange) × T_base_camera × T_camera_target` 得到板相对末端的位姿。检查多次预测的一致性，记录重投影误差和工作区内实际位置误差。再把结果写入 `camera_extrinsics.example.json` 的本地副本。偏移单位为米，旋转矩阵为右手系。接入 URDF 时要确认父链和 optical frame 的轴定义。

## 夹爪开口与 TCP

1. 固定量具和测量基准，在同一对指面上量开口。分别从开向闭、从闭向开，在端点之间选多个角度采点。记录静止时实际反馈的电机输出轴角度（rad）和开口宽度（mm），将采集方向写作 `opening` 或 `closing`。同角度可重复采样，重复数据使用中位开口。
2. 使用独立的本地文件填写 `gripper_measurements.csv`。每个方向至少需要三个不同的角度。角度采用同一连续坐标和零点基准。不要混入跨回绕的角度原值。建议每个方向采 7–10 点，并留几组中间点验证插值误差。
3. 使用 Python 标准库离线生成两条分段线性曲线。

   ```powershell
   py .\fit_gripper_opening.py .\gripper_measurements.local.csv --out .\gripper_opening.local.json
   ```

   工具拒绝空表、角度与开口关系反转的曲线，以及两条曲线方向不一致的结果。只在各方向的测量区间内插值。需要检测抓住布料时的力、接触位置或开口变化，还要在实际任务工况下另外验证。
4. TCP 单独测量。在约定夹口宽度定义夹持中心及轴向，记录法兰到该工具坐标系的六自由度位姿。使用标定治具或控制器 TCP 流程实测并复核。夹爪曲线和两个电机端点均不能给出这个位姿。
