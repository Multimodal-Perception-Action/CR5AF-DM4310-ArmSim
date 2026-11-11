# CR5AF-DM4310 ArmSim

A work-in-progress project for documenting and simulating a Dobot CR5AF with a custom gripper driven by a DM-J4310-2EC motor.

## Scope

- Build a mechanically faithful gripper model and integrate it with the CR5AF model.
- Record repeatable flange, gripper, and motor-to-jaw calibration procedures.
- Keep offline simulation assets separate from any live hardware interface.
- Document the PC, CC262 tool-RS485, T-CAN485, and motor-CAN communication path.

This initial repository contains documentation and an empty calibration template. It does not contain a live robot or motor command sender.

## Bench-verified communication path

The tested byte path was initiated by software on the Windows PC:

PC application -> Ethernet/TCP port 60000 -> CC262 tool-RS485 channel -> T-CAN485 -> Classic CAN -> DM-J4310-2EC

A read-only motor status/parameter query was received through the gateway. This confirms the tested transport path; it does not mean the robot controller autonomously creates the motor command. In this arrangement, the PC application originates the bytes and the controller forwards them to tool RS485.

The DM-J4310-2EC V1.2 bench setup was read back as Classic CAN at 1 Mbit/s. Refer to the exact motor revision and its official manual before using these values with another unit.

## Current status

See TODO.md and docs/verification-status.md. The custom gripper assembly model, measured transforms, public simulation adapter, and full offline regression are not yet delivered here. The CR5AF is modeled separately from the gripper; the ReBot B601-DM model used by other projects is not a substitute for this custom mechanism.

## Safety and data policy

- This repository is for simulation and documentation. Do not treat it as a safety-rated controller.
- Do not connect RS485 A/B directly to CAN_H/CAN_L.
- No actual controller IPs, serial numbers, raw machine logs, credentials, or per-device calibration values are included in this starter.
- Keep hardware-specific calibration in local files until its owner explicitly approves publication.

## References

- Dobot TCP/IP Protocol V4: https://github.com/Dobot-Arm/TCP-IP-Protocol-6AXis-V4
- Dobot TCP/IP Python SDK V4: https://github.com/Dobot-Arm/TCP-IP-Python-V4
- Dobot ROS 2 V4: https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4
- Damiao DM-J4310-2EC: https://github.com/dmBots/DM-J4310-2EC
- ArmSimLab: https://github.com/Multimodal-Perception-Action/ArmSimLab