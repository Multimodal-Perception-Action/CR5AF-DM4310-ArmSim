# CR5AF 机器人本体 URDF

- ROS 入口为 [dobot_rviz/urdf/cr5af_robot.urdf](dobot_rviz/urdf/cr5af_robot.urdf)。ROS 中让 `package://dobot_rviz` 指向对应的 Dobot 包。
- [cr5af_portable.urdf](cr5af_portable.urdf) 把网格地址改为相对于该文件的地址，便于直接从本目录加载。保留同目录的 `dobot_rviz/meshes/cr5af/`。
- URDF 的长度单位是米，角度是弧度。现有 `tool0` 是官方 URDF 的工具参考坐标，需要根据实际夹爪装配求出机械法兰到抓取 TCP 的变换。
- 组合模型需按现装版本建立夹爪基体到两指的关节及碰撞体。若 ZED Mini 固定在横梁上，其相机外参应从 `base_link`（或经测量转换的共同工作站坐标系）发布。

原始 URDF 和网格来自 Dobot ROS2 V4 提交 `7807ef8fb31e8f86a76cd474a4a9cddffef6eca9`，保留 [MIT 许可](LICENSE_DOBOT_MIT)。两份 URDF 都移除了 SolidWorks 导出注释里的联系信息，其余建模参数保持原样。便携版另外替换了 14 处网格路径。[Dobot 上游模型](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4/tree/7807ef8fb31e8f86a76cd474a4a9cddffef6eca9)
