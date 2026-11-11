# Verification status

## Confirmed on the bench

- CR5AF/CC262 tool-RS485 transport accepted PC-originated bytes through TCP port 60000.
- The T-CAN485 board received the bytes on its RS485 interface.
- A read-only DM-J4310-2EC V1.2 query returned motor status/parameter data through the CAN gateway.
- The observed motor configuration used Classic CAN at 1 Mbit/s.
- The PC, not an autonomous robot-side program, originated the tested request bytes.

These are bench observations, not a certification of a general-purpose controller. Raw logs and device-specific addresses are intentionally not published here.

## Not yet delivered or not established by this repository

- A reviewed CAD model of the complete as-built custom gripper.
- A calibrated CR5AF flange-to-gripper transform for the current assembly.
- A validated motor-angle-to-jaw-opening model with tolerances and per-device limits.
- A MuJoCo scene that loads the custom gripper. Existing B601-DM assets represent a different gripper.
- A public hardware adapter suitable for routine use.
- A user-defined CAN transmit API on the CC262.

## Evidence handling

Keep original logs in the private project workspace. Publish only a reviewed summary that removes local network addresses, serial numbers, user paths, and other machine-specific identifiers.