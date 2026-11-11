# DM4310 V4 / DM-J4310-2EC protocol audit (read-only)

Date: 2026-11-04

## Identity and version status

The connected unit's label has now been reported as `DM-J4310-2EC V1.2 / 24 V`. The official DAMIAO public repository names the product `DM-J4310-2EC` and publishes V1.1 and V1.2 manuals; it does not identify the motor as “DM4310 V4”. Therefore “V4” is treated as an informal seller/firmware label. Read-only register responses now identify `sw_ver` raw ASCII as `7018`, `sub_ver` as 57, and boot version bytes as `03 02 00 06` (3.2.0.6). See `logs/dm4310_register_audit_20261110_151350.json`.

Official model repository: https://github.com/dmBots/DM-J4310-2EC/tree/master/manual

## Protocol extracted from the official V1.2 manual

The official `DM-J4310-2EC V1.2 减速电机使用说明书 V1.4 2026-09-14.pdf`, page 11, states that the default bus is standard CAN at 1 Mbps. Page 17 defines baud-rate register 0x23 values 0=125 kbit/s, 1=200 kbit/s, 2=250 kbit/s, 3=500 kbit/s, 4=1 Mbit/s, 5=2 Mbit/s, 6=2.5 Mbit/s, 7=3.2 Mbit/s, 8=4 Mbit/s, 9=5 Mbit/s. At power-up, <=1 Mbit/s selects CAN 2.0B; >1 Mbit/s selects CAN FD. The actual stored rate can differ from the default.

The command ID is register `ESC_ID` (0x08, range 0..0x7FF). The feedback ID is `MST_ID` (0x07, range 0..0x7FF), default 0. Both are standard 11-bit IDs in the documented frames. The manual says command reception is request/response: a frame matching the configured motor CAN ID causes a feedback frame.

MIT mode uses an 8-byte standard frame whose CAN ID equals `ESC_ID`:

`p_des:16 | v_des:12 | Kp:12 | Kd:12 | t_ff:12`.

The documented packing is bytes 0..7: position high/low; velocity high 8 bits; velocity low nibble + Kp high nibble; Kp low byte; Kd high 8 bits; Kd low nibble + torque high nibble; torque low byte. Kp is [0,500], Kd is [0,5]. Position, velocity and torque mapping limits are the configurable `PMAX` (0x15), `VMAX` (0x16) and `TMAX` (0x17), so the V1.2 manual does not justify hard-coding the older V1.1 limits for every unit.

Feedback is an 8-byte frame on `MST_ID`: `D0=ID | (ERR<<4)`, `D1..D2=POS[15:0]`, `D3..D4=VEL[11:0]`, `D4..D5=T[11:0]`, `D6=T_MOS`, `D7=T_Rotor`. Error/status values are 0 disabled, 1 enabled, 8 over-voltage, 9 under-voltage, A over-current, B MOS over-temperature, C motor winding over-temperature, D communication loss, E overload. Position is signed 16-bit mapped, velocity and torque signed 12-bit mapped.

The documented control frames are: enable payload `FF FF FF FF FF FF FF FC`; disable `FF FF FF FF FF FF FF FD`; clear fault `FF FF FF FF FF FF FF FB`. These are control frames and are intentionally not transmitted by this project.

Power-up state is disabled. The manual says enable is required after self-test, and communication-loss is reported as status D. It also exposes timeout register 0x09. Exact timeout-to-output behavior, emergency-stop behavior, and the effect of the host adapter's loss-of-link policy remain unverified on the specific motor.

Primary protocol source: `official/DM-J4310-2EC/manual/DM-J4310-2EC V1.2 减速电机使用说明书 V1.4 2026-09-14.pdf`, downloaded from the official repository at https://github.com/dmBots/DM-J4310-2EC/tree/master/manual .

Official generic SDK cross-check: `DM_CAN.py` defines DM4310 limits [12.5, 30, 10], packs the same MIT 8-byte layout, maps feedback from `MasterID`, and implements enable/disable command frames. The SDK source is generic and is not a firmware-version certificate for a particular “V4” unit: https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py

## Dobot Lua/TCP boundary

The supplied Dobot note `步进电机连接/如何使用末端的485(1).pdf`, pp.1-2, documents script calls `SetToolBaudRate`, `GRead` and `GWrite` and shows raw Modbus-RTU byte arrays. The V4 TCP SDK source exposes `SetTool485`, `SetToolMode`, and `ModbusRTUCreate`, but no `GRead`, `GWrite`, or `SetToolBaudRate` function. V4 TCP also exposes `RunScript`, which starts a project and is therefore not a read-only trigger.

Conclusion: raw end RS485 bytes are evidenced for the script API, not for the V4 Dashboard TCP API. Compatibility with CR5AF/CC262 firmware and DobotStudio Pro 4.6 requires the matching CR Lua Syntax Guide and a controlled bench test; neither was executed here.

## Safety boundary

No enable, disable, clear-fault, mode-switch, parameter-write, zeroing or motion frame was sent. After the motor received independent 24 V power and tool RS485 was explicitly restored to 115200/N/1, the read-only frame on CAN ID `0x7FF` succeeded at 1 Mbps. Seven allowlisted registers were read successfully; ESC_ID=1, MST_ID=0 and can_br=4 (1 Mbps Classic CAN). See `logs/dm4310_register_audit_20261110_151350.json`.
