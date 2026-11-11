# Gripper host and gateway behavior audit

Date: 2026-11-12. Raw device logs remain private. The firmware source does include project-specific gripper range and speed-limit constants; they are not portable defaults.

## Failure modes reviewed

The host controller and gateway use a persistent session for open, close, and contact hold. A direction reversal updates the pending target while preserving the current session. STOP is latched separately and takes priority over direction changes. Heartbeats continue during startup, movement, and hold.

The gateway distinguishes normal endpoint/contact hold from communication loss, motor faults, hard position/speed limits, over-temperature, and action timeout. A normal contact is not treated as a fault or as a requirement to close fully. Loss of feedback still has a bounded recovery policy and ends in a verified stop if status cannot be recovered.

## Safety behavior

- No timed release occurs unless a caller explicitly requests a hold duration.
- Normal open/close completion may remain in a verified hold state.
- An explicit stop, true heartbeat loss, motor fault, thermal limit, hard limit, or exhausted feedback recovery ends the action.
- Stop completion is reported only after the gateway confirms disable.
- This software does not provide force sensing or distinguish every obstruction from a workpiece.

## Project-specific limits changed in this revision

- The closed-position hard ceiling increased from `5.311811232` to `5.647718013` rad to accommodate the observed passive settling position of this replacement gripper. The commanded close target was not increased.
- The feedback-speed abort threshold increased from `9.6` to `24.0` rad/s because the faster ramp could exceed the former threshold. The protocol range remains `DM_VMAX = 30` rad/s; the 24 rad/s cutoff is a software guard, not a safety-rated limit.

These are device-specific changes. Re-measure the mechanism and review the motion/thermal behavior before reusing them on another unit. Do not copy these values as generic DM4310 settings.

## Validation status

Offline tests cover event decoding, contact hold, reversal, cancellation, heartbeat handling, bounded CAN feedback recovery, and STOP confirmation. The operator reported a follow-up loaded "grip, release slightly, grip again" retest completed without an unexpected stop. Raw logs are not included. The numerical limits above are specific to this prototype. This report does not establish that physical CAN or power faults can never occur.
