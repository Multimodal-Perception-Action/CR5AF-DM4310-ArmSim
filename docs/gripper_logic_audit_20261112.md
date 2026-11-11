# Gripper host and gateway behavior audit

Date: 2026-11-12. This is a sanitized engineering summary; raw device logs and per-device calibration values remain in the private workspace.

## Failure modes reviewed

The host controller and gateway use a persistent session for open, close, and contact hold. A direction reversal updates the pending target while preserving the current session. STOP is latched separately and takes priority over direction changes. Heartbeats continue during startup, movement, and hold.

The gateway distinguishes normal endpoint/contact hold from communication loss, motor faults, hard position/speed limits, over-temperature, and action timeout. A normal contact is not treated as a fault or as a requirement to close fully. Loss of feedback still has a bounded recovery policy and ends in a verified stop if status cannot be recovered.

## Safety behavior

- No timed release occurs unless a caller explicitly requests a hold duration.
- Normal open/close completion may remain in a verified hold state.
- An explicit stop, true heartbeat loss, motor fault, thermal limit, hard limit, or exhausted feedback recovery ends the action.
- Stop completion is reported only after the gateway confirms disable.
- This software does not provide force sensing or distinguish every obstruction from a workpiece.

## Validation status

Offline tests cover event decoding, contact hold, reversal, cancellation, heartbeat handling, bounded CAN feedback recovery, and STOP confirmation. The operator reported a follow-up loaded "grip, release slightly, grip again" retest completed without an unexpected stop. Raw logs and machine-specific details are intentionally not included. This report does not establish that physical CAN or power faults can never occur.
