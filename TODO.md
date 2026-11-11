# CR5AF + DM-J4310-2EC project TODO

This checklist tracks work for the custom gripper project. The existing ArmSimLab CR5AF/B601-DM assets and configs should remain unchanged until the new model is reviewed.

## 1. Establish the as-built model

- [ ] Identify the exact custom gripper CAD assembly and the DM-J4310-2EC mounting orientation.
- [ ] Record CAD source, owner, and redistribution license for every mesh.
- [ ] Measure the flange-to-gripper transform from a repeatable datum.
- [ ] Separate motor, transmission, and jaw links; document joint axes and mechanical stops.
- [ ] Create a MuJoCo model for the actual mechanism. Add a URDF/Xacro representation if a ROS consumer needs it.
- [ ] Compare visual and collision meshes against the assembled hardware.

## 2. Calibrate the gripper

- [ ] Define motor-position and jaw-opening units and sign convention.
- [ ] Measure fully open, fully closed, and intermediate jaw positions with repeat samples.
- [ ] Choose conservative software limits inside the measured mechanical endpoints.
- [ ] Record raw observations, fitted mapping, uncertainty, date, and hardware revision.
- [ ] Keep per-device measured values out of public examples unless publication is approved.

## 3. Add an isolated simulation cell

- [ ] Add a dedicated CR5AF/custom-gripper cell config without overwriting cells/cr5af.json.
- [ ] Extend the scene loader to select the custom gripper model instead of the existing B601-DM model.
- [ ] Add an offline open/close trajectory example and a report showing joint limits and contacts.
- [ ] Check frame conventions, initial pose, workspace clearance, and collision geometry.
- [ ] Add repeatable offline validation after model integration.

## 4. Document the hardware interface

- [ ] Keep the data path explicit: PC application -> CC262 TCP tool-485 tunnel -> T-CAN485 -> DM motor CAN.
- [ ] Document which process originates each command and which device forwards or converts it.
- [ ] Keep robot motion, motor enable, fault-clear, and configuration writes out of simulation examples.
- [ ] Design a separate hardware adapter only after its command allow-list, feedback checks, timeout, and stop behavior are reviewed.

## 5. Release hygiene

- [ ] Add only reviewed CAD and source files with their license notices.
- [ ] Add sanitized evidence summaries; do not commit raw logs, credentials, local IP addresses, or private serial-port details.
- [ ] Record software and firmware versions for any future repeatable hardware validation.