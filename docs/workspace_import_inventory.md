# Workspace import inventory

This import was assembled from the local robot workspace. It keeps project source and assets in the repository while omitting unrelated projects, generated environments, vendor manuals/installers, and credentials.

## Imported

- All 44 Python source files from the project `src/` folder. These include read-only tools and hardware-action programs; see `script_safety_index.md`.
- The 15 project audit/integration documents from the workspace `docs/` folder, plus the U-Arm teleoperation runbook and joint mapping files.
- Eight JSON files from the workspace `config/` folder, including machine-specific calibration data.
- The two T-CAN485 PlatformIO gateway source projects (`platformio.ini`, `README.md`, and `src/main.cpp` only). PlatformIO build folders and compiled binaries were omitted.
- The 28 generated simulation files: scene/model XML, URDF, mesh exports, and supporting evidence JSON.
- User-created wiring diagrams, U-Arm trigger CAD, R66/R67 gripper CAD, and CR5AF connector-design candidate files. The B601-DM two-piece adapter is under `assets/reference_adapters/` and remains labeled as a different gripper design.
- 105 selected read-only feedback/query logs under `evidence/read_only/raw_logs/`: 44 CR5 30004 feedback captures, 17 DM4310 read probes, 7 DM4310 status probes, 7 DM4310 status monitors, 4 RS485 gateway probes, 19 gateway identity records, 2 DM4310 register audits, and 5 other read-only status/run records.

## Deliberately not imported

- The `.github_token` file, browser profile/token stores, credentials, and private user data. The token file was excluded by name; its contents were not read.
- Python virtual environments, `__pycache__`, PlatformIO build output, local application state, cache databases, and temporary artifacts.
- Unrelated hexapod, CR10 workstation, camera-stand, and other independent project trees.
- Full upstream repository clones, the CR10A-focused teleoperation simulation archive, duplicate archives, and downloaded source archives. The audit documents retain upstream links where relevant.
- Dobot, Damiao, and gateway vendor manuals, software installers, and other vendor-distributed files without an identified redistribution license. Use the official links in the audit documents.
- The broad local logs tree, including motion, enable, error-clear, configuration-write, control-stream, and screenshot records. Only the selected read-only status/query records were copied.
- The separate local `tests/` directory and the U-Arm/CR5 continuous-control launch scripts.

## Import checks

- No known API-token patterns were found in the imported text files.
- No user-specific absolute Windows paths were found in imported text files.
- No repository tests were run and no robot, motor, serial port, or controller was contacted during this import.
