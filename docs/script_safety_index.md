# Script safety index

The source archive is not uniformly read-only. This index is a first-pass classification by API names and file purpose; it is not a substitute for reviewing the source.

## Read-only or offline-oriented entry points

- `src/connection_test.py`, `src/robot_status.py`, `src/feedback_monitor.py`
- `src/dm4310_read_probe.py`, `src/dm4310_status_probe.py`, `src/dm4310_status_monitor.py`, `src/dm_gateway_identity_probe.py`
- `src/rs485_gateway_probe.py`, `src/uarm_readonly_probe.py`, `src/uarm_dobot_preview.py`
- `src/uarm_gripper_preview.py`, `src/uarm_gripper_jog_preview.py`
- `src/cr5_mujoco_twin.py`, `src/uarm_cr5_dual_twin.py`, `src/uarm_cr5_mapping_audit.py`

Some read-only programs still open live network or serial connections. Check command-line options and data sources before use.

## Programs that can change hardware or configuration

Review these before any use. They contain or implement robot motion/enable, error clearing, tool configuration writes, servo output, motor enable/motion, or firmware flashing:

- `src/clear_error_once.py`
- `src/configure_tool485.py`
- `src/cr5_joint_smoke_test.py`, `src/cr5_return_home.py`
- `src/gripper.py`: `status` is a read/query path; `open`, `close`, and `stop` send motor actions over TCP port 60000. `open`/`close` keep an active hold by default until Ctrl+C; `--hold-seconds` explicitly requests timed stop/disable.
- `src/dm4310_zero_torque_enable_test.py`
- `src/flash_gripper_gateway.py`
- `src/uarm_cr5_axis_follow_test.py`, `src/uarm_cr5_continuous_teleop.py`, `src/uarm_cr5_full_teleop.py`, `src/uarm_cr5_joint_range_test.py`, `src/uarm_cr5_live_panel.py`
- `src/uarm_dobot_teleop.py`: preview is the default; explicit live acknowledgement enables ServoJ output.
- `src/uarm_gripper_binary_follow.py`
- `firmware/tcan485_dm4310_read_probe/`

This import copied source only. It did not run, build, or flash any program.

The `tests/` directory contains offline tests. The firmware test compiles production logic with mocked I/O; it does not connect to a robot, gateway, motor, or serial port.
