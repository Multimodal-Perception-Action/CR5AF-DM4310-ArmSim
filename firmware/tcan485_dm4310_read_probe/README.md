# T-CAN485 DM4310 guarded probe

## Guarded gateway build 1.5 guarded-feedback-tolerance

The current source keeps the 1.x network protocol and all allow-lists,
feedback checks, endpoint limits, emergency stop handling, timeout, and final
disable verification. The fixed gripper ramp step is increased from 0.04 rad
to 0.10 rad, with 10 ms pacing and feedback validation on every step. The hold
phase may finish after three confirmed target samples and is capped at ten.
The measured-speed guard is 3.2 rad/s. `esp32dev_legacy` builds the verified
1.1 profile (0.04 rad, 15 ms, up to 20 hold samples) for rollback.

Build 1.4 extends only the initial OPEN/CLOSE status-feedback wait from 120 ms
to 500 ms. Hardware logs showed a valid disabled response arriving after the
old preflight window. Motion-step feedback deadlines, endpoint limits, speed
and temperature checks, emergency STOP, and final disable verification are
unchanged.

Build 1.5 keeps the same command profile and endpoints. It changes only the
motion-feedback validation ceiling from 3.0 to 3.2 rad/s because the motor's
12-bit feedback encoded the nominal 3 rad/s point as 3.01099 rad/s and caused
a false protective abort. This tolerance does not raise commanded speed.

The identity event remains protocol family `flags=0x10`; its diagnostic
`position_rad` field is `1.5` in this build so the host can confirm the flashed
profile without issuing a motor command.

The board USB upload port must be present before flashing. Building the image
does not change the attached hardware.

This firmware accepts one fixed RS485 request packet and converts it into the
official DM parameter-read CAN frame:

`D5 4D 52 51 ID_LO ID_HI RID XOR`

The read path transmits CAN identifier `0x7FF`; byte 2 is hard-coded to read
opcode `0x33`. Generic CAN forwarding is absent.

The read path scans the five Classic CAN rates documented by the V1.2 manual:
1000, 500, 250, 200, and 125 kbit/s. Every attempt is the same single-shot
read-only frame; a missing ACK cannot cause automatic retries, and the scan
stops immediately on `TX_SUCCESS`. USB output includes the tested rate, TWAI
alert names, and transmit/receive error counters.

It also accepts one separately framed, fixed status-refresh request and emits
only the official `0x7FF`/`0xCC` read-only status query.

Build 0.8 additionally contains deliberately fixed active tests. It accepts
only motor ID 1 with a guard byte, verifies fresh status, preloads an MIT frame
with `kp=0`, `kd=0`, `velocity=0`, and `torque=0`, enables for at most 250 ms,
then disables and verifies the disabled state. Board-side limits abort on
fault, missing feedback, temperature, speed, or more than 0.05 rad position
change. The command cannot supply a target position. Clear-fault, zeroing,
parameter writes, arbitrary IDs, and generic CAN frames remain unavailable.

The second guarded command performs one fixed `+0.02 rad` ramp and return with
`kp=10`, `kd=0.2`, and zero feed-forward torque, then disables. It rejects any
motor ID other than 1 and aborts on feedback, speed, temperature, or excursion
limits. Request: `D5 4D 4D 51 01 00 B6 XOR`.

The guarded test request is:

`D5 4D 45 51 01 00 A5 XOR`

The safety-only disable request is:

`D5 4D 44 51 01 00 5A XOR`

Before an active test, the host sends the identity-only request
`D5 4D 49 51 01 00 60 XOR`. It does not access CAN. The active request is
blocked unless the host observes the exact build 0.8 identity on COM6.

CAN responses are logged on USB and returned over RS485 as:

`D5 4D 52 50 ID_LO ID_HI FLAGS DLC DATA[8] XOR`

Host-side read-only probe:

```powershell
C:\ProgramData\Anaconda3\python.exe .\src\dm4310_read_probe.py `
  --motor-id 0x01 --rid 0x0E
```

`RID 0x0E` is `sw_ver` (read-only) in the official DM-J4310-2EC V1.2 manual.
The script waits for a possible CH9102-triggered ESP32 reset, closes TCP and
serial handles in `finally`, and writes its result under `logs\`.
