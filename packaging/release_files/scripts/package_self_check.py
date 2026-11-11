"""Offline import and asset check for the distributable package.

This program never opens serial or network hardware.
"""

from __future__ import annotations

import importlib
import argparse
from contextlib import ExitStack
from pathlib import Path
import shutil
import struct
import sys
import tempfile
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]


def offline_model_and_panel_check() -> None:
    """Exercise real compilation/UI constructors; forbid any hardware access."""
    sys.path.insert(0, str(ROOT / "src"))
    import mujoco
    import serial
    import socket
    import tkinter as tk
    import cr5_mujoco_twin as cr5
    import uarm_cr5_dual_twin as twin
    import uarm_cr5_live_panel as panel

    def forbidden(*_args, **_kwargs):
        raise AssertionError("Hardware access or child launch attempted during offline self-check")

    with ExitStack() as guards:
        guards.enter_context(patch.object(socket, "create_connection", side_effect=forbidden))
        guards.enter_context(patch.object(socket.socket, "connect", side_effect=forbidden))
        guards.enter_context(patch.object(socket.socket, "connect_ex", side_effect=forbidden))
        guards.enter_context(patch.object(serial.Serial, "open", side_effect=forbidden))
        guards.enter_context(patch.object(panel.subprocess, "Popen", side_effect=forbidden))
        with tempfile.TemporaryDirectory(prefix="teleop_model_check_") as folder:
            # ensure_assets may regenerate files; keep all writes in this copy.
            temporary = Path(folder) / "generated"
            shutil.copytree(ROOT / "simulation/generated", temporary)
            with ExitStack() as paths:
                paths.enter_context(patch.object(cr5, "GENERATED", temporary / "cr5"))
                paths.enter_context(patch.object(cr5, "GENERATED_MESHES", temporary / "cr5/meshes"))
                paths.enter_context(patch.object(cr5, "GENERATED_URDF", temporary / "cr5/cr5_robot_mujoco.urdf"))
                paths.enter_context(patch.object(twin, "UARM_MODEL", temporary / "uarm_repaired/uarm_config2_repaired.urdf"))
                paths.enter_context(patch.object(twin, "CR5_GRIPPER", temporary / "dual_twin/cr5_dm4310_gripper.xml"))
                model, data = twin.build_scene()
                mujoco.mj_forward(model, data)
                expected = [f"u_joint{i}" for i in range(1, 7)] + ["u_trigger_joint"]
                expected += [f"a_joint{i}" for i in range(1, 7)]
                for name in expected:
                    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) < 0:
                        raise RuntimeError(f"Model is missing required joint: {name}")
                if model.nq != 15:
                    raise RuntimeError(f"Expected 15 model positions including gripper; got {model.nq}")
                print(f"PASS: real MuJoCo scene compiled (nq={model.nq}, meshes={model.nmesh}).")
                # Rendering validates OpenGL availability as well as geometry.
                with mujoco.Renderer(model, height=128, width=128) as renderer:
                    renderer.update_scene(data)
                    pixels = renderer.render()
                    if pixels.shape != (128, 128, 3):
                        raise RuntimeError("Unexpected offline render dimensions")
                print("PASS: MuJoCo offscreen rendering.")
        window = tk.Tk()
        window.withdraw()
        try:
            controller = panel.LivePanel(window)
            window.update_idletasks()
            if controller.process is not None or controller.recovery_process is not None:
                raise RuntimeError("Opening the panel unexpectedly launched a controller")
        finally:
            window.destroy()
        print("PASS: actual Tk teleoperation panel constructed without hardware access.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true", help="imports/assets only, used before launch")
    args = parser.parse_args()
    if sys.version_info[:2] != (3, 13) or struct.calcsize("P") != 8:
        raise RuntimeError(f"Python 3.13 x64 required; current interpreter: {sys.version}")

    for name in ("tkinter", "serial", "scservo_sdk", "numpy", "mujoco", "trimesh", "collada", "scipy", "PIL", "esptool"):
        importlib.import_module(name)

    required = (
        "src/uarm_cr5_live_panel.py",
        "src/uarm_cr5_continuous_teleop.py",
        "src/gripper.py",
        "config/uarm_cr5_continuous_ui.json",
        "config/uarm_cr5_full_calibration.json",
        "config/uarm_gripper_calibration.json",
        "config/dm4310_gripper_calibration.json",
        "simulation/generated/uarm_repaired/uarm_config2_repaired.urdf",
        "simulation/generated/dual_twin/cr5_dm4310_gripper.xml",
        "official/TCP-IP-ROS-6AXis-models/dobot_description/urdf/cr5_robot.urdf",
        "firmware/tcan485_dm4310_read_probe/prebuilt/firmware.bin",
    )
    missing = [item for item in required if not (ROOT / item).is_file()]
    if missing:
        raise FileNotFoundError("Required package files are missing: " + ", ".join(missing))

    print("PASS: imports, configuration, assets and prebuilt firmware are present.")
    if not args.quick:
        offline_model_and_panel_check()
    print("PASS: self-check performed without serial/network hardware access.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
