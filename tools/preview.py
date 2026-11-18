"""Offline viewer for the local CR5AF V2R1 assembly."""
import argparse
from pathlib import Path

import mujoco


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gui", action="store_true", help="open the MuJoCo viewer")
    parser.add_argument("--jaw-mm", type=float, default=0.0,
                        help="per-jaw displacement in mm relative to CAD nominal pose")
    args = parser.parse_args()
    if not -7.5 <= args.jaw_mm <= 12.5:
        parser.error("--jaw-mm must be between -7.5 and 12.5")
    model_path = Path(__file__).resolve().with_name("cr5af_v2r1_x3.mjcf.xml")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    slide = args.jaw_mm * 0.001
    for name in ("jaw_positive_slide", "jaw_negative_slide"):
        data.qpos[int(model.joint(name).qposadr[0])] = slide
    data.ctrl[int(model.actuator("jaw_per_side_travel_m").id)] = slide
    mujoco.mj_forward(model, data)
    print(f"CR5AF V2R1 + X3: {model.njnt} joints, {model.nmesh} meshes, "
          f"per-jaw travel {args.jaw_mm:g} mm")
    if args.gui:
        from mujoco import viewer
        with viewer.launch_passive(model, data) as window:
            import time
            window.opt.geomgroup[3] = 0
            while window.is_running():
                with window.lock():
                    mujoco.mj_step(model, data)
                window.sync()
                time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
