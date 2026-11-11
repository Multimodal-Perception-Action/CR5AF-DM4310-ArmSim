# CR5AF + DM4310 Gripper Simulation and Integration

This repository collects the CR5AF / CC262 communication audits, DM-J4310-2EC V1.2 gateway work, U-Arm teleoperation integration, simulation assets, calibration/configuration data, and custom gripper CAD used in this project.

## Project folders

- `src/`: Python status, simulation, integration, and hardware-facing programs.
- `firmware/`: T-CAN485 ESP32 gateway source; build output is excluded.
- `simulation/`: generated MuJoCo URDFs, scenes, and mesh assets.
- `assets/`: custom gripper geometry, wiring diagrams, U-Arm trigger models, and a separately labeled B601-DM adapter reference.
- `config/`: example and project calibration/configuration JSON files.
- `docs/`: protocol/API audits, integration notes, calibration and simulation guidance.
- `evidence/read_only/raw_logs/`: selected status/query logs only.

## Bench communication path

The verified transport path was initiated by the Windows PC:

`PC application -> CR5AF TCP 60000 -> CC262 tool RS485 -> T-CAN485 -> Classic CAN -> DM-J4310-2EC V1.2`

A read-only status query traversed the gateway. The PC originates the bytes; this result does not mean the robot controller autonomously generates motor commands. See `docs/dm4310_gateway_audit.md` and `docs/rs485_api_audit.md`.

## Hardware safety

This repository contains archived scripts that can enable or move the robot or motor, clear errors, write tool configuration, operate U-Arm servos, or flash gateway firmware. Importing these files did not execute them. Review `docs/script_safety_index.md` and each program before running anything. The project is not a safety-rated controller and does not provide collision protection.

Never connect RS485 A/B directly to CAN_H/CAN_L. The included B601-DM adapter files are reference geometry for a different gripper model and are not validated as a DM4310 mount.

The `config/` folder includes machine-specific calibration values. Review them before use; do not assume a stored calibration is valid after hardware, tool, or mounting changes.

## Verification status

The selected evidence logs contain read-only feedback and query records. Movement, enable, error-clear, configuration-write, and control-stream logs are not included in this evidence set. No hardware command was sent during this repository import. No tests were run as part of the import.

See `docs/verification-status.md`, `docs/tcp_mode_audit.md`, `docs/dm4310_v4_protocol_audit.md`, and `docs/can_direct_connection.md` for evidence-backed status and official references. See `docs/workspace_import_inventory.md` for the import scope.
