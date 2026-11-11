"""Read-only Dobot CR5 digital twin using the official URDF and MuJoCo.

Sources:
  synthetic - offline deterministic motion, no hardware connection
  dobot     - TCP/30004 feedback only
  uarm      - FE-URT-2 positions mapped onto the current Dobot pose

No mode-change, enable, clear-error, or motion command is implemented here.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import socket
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh

from feedback_monitor import FRAME_BYTES, decode_v4, recv_exact
from feetech_port import resolve_uarm_port
from uarm_dobot_preview import (
    DEGREES_PER_TICK,
    JOINT_SIGNS,
    SERVO_IDS,
    read_positions,
    shortest_tick_step,
)


ROOT = Path(__file__).parents[1]
DESCRIPTION = ROOT / "official" / "TCP-IP-ROS-6AXis-models" / "dobot_description"
SOURCE_URDF = DESCRIPTION / "urdf" / "cr5_robot.urdf"
SOURCE_MESHES = DESCRIPTION / "meshes" / "cr5"
GENERATED = ROOT / "simulation" / "generated" / "cr5"
GENERATED_MESHES = GENERATED / "meshes"
GENERATED_URDF = GENERATED / "cr5_robot_mujoco.urdf"
JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def ensure_assets() -> Path:
    if not SOURCE_URDF.is_file():
        raise FileNotFoundError(f"official CR5 URDF not found: {SOURCE_URDF}")
    GENERATED_MESHES.mkdir(parents=True, exist_ok=True)
    for source in sorted(SOURCE_MESHES.glob("*.dae")):
        target = GENERATED_MESHES / f"{source.stem}.obj"
        if target.is_file() and target.stat().st_mtime >= source.stat().st_mtime:
            continue
        scene = trimesh.load(source, force="scene")
        geometry = scene.to_geometry()
        geometry.export(target)

    tree = ET.parse(SOURCE_URDF)
    root = tree.getroot()
    for mesh in root.findall(".//mesh"):
        source_name = Path(mesh.attrib["filename"].replace("\\", "/")).stem
        mesh.set("filename", f"meshes/{source_name}.obj")
    ET.indent(tree, space="  ")
    tree.write(GENERATED_URDF, encoding="utf-8", xml_declaration=True)
    return GENERATED_URDF


def load_model() -> tuple[mujoco.MjModel, mujoco.MjData, list[int]]:
    urdf = ensure_assets()
    model = mujoco.MjModel.from_xml_path(str(urdf))
    data = mujoco.MjData(model)
    qpos_addresses: list[int] = []
    for name in JOINT_NAMES:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise RuntimeError(f"joint missing after URDF import: {name}")
        qpos_addresses.append(int(model.jnt_qposadr[joint_id]))
    return model, data, qpos_addresses


def apply_degrees(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    qpos_addresses: list[int],
    degrees: list[float],
) -> None:
    if len(degrees) != 6 or not all(math.isfinite(value) for value in degrees):
        raise ValueError(f"invalid six-joint pose: {degrees}")
    for address, value in zip(qpos_addresses, degrees):
        data.qpos[address] = math.radians(value)
    mujoco.mj_forward(model, data)


def link6_pose(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, list[float]]:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Link6")
    if body_id < 0:
        raise RuntimeError("Link6 missing after URDF import")
    matrix = np.asarray(data.xmat[body_id]).reshape(3, 3)
    quaternion = np.empty(4, dtype=float)
    mujoco.mju_mat2Quat(quaternion, matrix.reshape(-1))
    return {
        "position_m": [float(value) for value in data.xpos[body_id]],
        "quaternion_wxyz": [float(value) for value in quaternion],
    }


def receive_feedback(sock: socket.socket) -> dict[str, object]:
    # CC262 can occasionally emit a zero/invalid 1440-byte slot.  Keep strict
    # decoding, but discard a bounded number of bad slots instead of letting a
    # visualization session die on the first one.
    last_error: ValueError | None = None
    for _ in range(40):
        try:
            feedback = decode_v4(recv_exact(sock, FRAME_BYTES))
            # MessageSize/TestValue prove frame alignment, but a CC262 stream can
            # still expose an internally torn slot while the controller updates
            # the shared feedback buffer.  Reject impossible status bytes and
            # non-physical joint values so a visualization never displays that
            # transient slot as a real controller state.
            joints = [float(value) for value in feedback["q_actual_deg"]]
            robot_mode = int(feedback["robot_mode"])
            status_bytes = (
                int(feedback["enable_status"]),
                int(feedback["running_status"]),
                int(feedback["error_status"]),
            )
            if not 1 <= robot_mode <= 11:
                raise ValueError(f"implausible RobotMode in feedback slot: {robot_mode}")
            if any(value not in (0, 1) for value in status_bytes):
                raise ValueError(f"implausible status bytes in feedback slot: {status_bytes}")
            if not all(math.isfinite(value) and abs(value) <= 720.0 for value in joints):
                raise ValueError(f"implausible QActual in feedback slot: {joints}")
            return feedback
        except ValueError as exc:
            last_error = exc
    raise RuntimeError(f"could not obtain a valid V4 feedback frame: {last_error}")


class PoseSource:
    def read(self, elapsed: float) -> tuple[list[float], dict[str, object]]:
        raise NotImplementedError

    def close(self) -> None:
        pass


class SyntheticSource(PoseSource):
    def read(self, elapsed: float) -> tuple[list[float], dict[str, object]]:
        pose = [
            20.0 * math.sin(elapsed * 0.7),
            -30.0 + 15.0 * math.sin(elapsed * 0.5),
            -60.0 + 20.0 * math.sin(elapsed * 0.4),
            15.0 * math.sin(elapsed * 0.9),
            20.0 * math.sin(elapsed * 0.8),
            25.0 * math.sin(elapsed * 0.6),
        ]
        return pose, {"hardware_connections": 0}


class DobotSource(PoseSource):
    def __init__(self, host: str, timeout: float) -> None:
        self.socket = socket.create_connection((host, 30004), timeout=timeout)
        self.socket.settimeout(timeout)

    def read(self, elapsed: float) -> tuple[list[float], dict[str, object]]:
        feedback = receive_feedback(self.socket)
        pose = [float(value) for value in feedback["q_actual_deg"]]
        return pose, {
            "robot_mode": feedback["robot_mode"],
            "enable_status": feedback["enable_status"],
            "running_status": feedback["running_status"],
            "error_status": feedback["error_status"],
            "controller_tool_vector": feedback["tool_vector_actual"],
        }

    def close(self) -> None:
        self.socket.close()


class UArmSource(PoseSource):
    def __init__(self, port_request: str, host: str, timeout: float) -> None:
        from scservo_sdk import PortHandler, sms_sts

        self.port_name = resolve_uarm_port(port_request)
        self.port = PortHandler(self.port_name)
        self.protocol = sms_sts(self.port)
        self.opened = False
        try:
            self.opened = bool(self.port.openPort())
            if not self.opened or not self.port.setBaudRate(1_000_000):
                raise RuntimeError(f"could not open {self.port_name} at 1 Mbps")
            self.robot_socket = socket.create_connection((host, 30004), timeout=timeout)
            self.robot_socket.settimeout(timeout)
            feedback = receive_feedback(self.robot_socket)
            self.robot_baseline = [float(value) for value in feedback["q_actual_deg"]]
            self.previous, _ = read_positions(self.protocol)
            self.accumulated = [0] * len(SERVO_IDS)
        except Exception:
            self.close()
            raise

    def read(self, elapsed: float) -> tuple[list[float], dict[str, object]]:
        positions, transport = read_positions(self.protocol)
        for index, value in enumerate(positions):
            self.accumulated[index] += shortest_tick_step(self.previous[index], value)
        self.previous = positions
        offsets = [ticks * DEGREES_PER_TICK for ticks in self.accumulated]
        relative = [JOINT_SIGNS[index] * offsets[index] for index in range(6)]
        pose = [self.robot_baseline[index] + relative[index] for index in range(6)]
        return pose, {
            "uarm_port": self.port_name,
            "servo_positions_ticks": positions,
            "servo_transport": transport,
            "relative_joint_deg": relative,
            "gripper_offset_deg": offsets[6],
        }

    def close(self) -> None:
        robot_socket = getattr(self, "robot_socket", None)
        if robot_socket is not None:
            robot_socket.close()
        if getattr(self, "opened", False) and self.port.ser is not None:
            self.port.closePort()


def make_source(args: argparse.Namespace) -> PoseSource:
    if args.source == "synthetic":
        return SyntheticSource()
    if args.source == "dobot":
        return DobotSource(args.host, args.timeout)
    return UArmSource(args.port, args.host, args.timeout)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", choices=("synthetic", "dobot", "uarm"),
                        default="synthetic")
    parser.add_argument("--host", default="192.168.201.1")
    parser.add_argument("--port", default="auto")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()
    if args.duration <= 0 or args.rate <= 0:
        parser.error("duration and rate must be positive")

    model, data, qpos_addresses = load_model()
    source = make_source(args)
    log_dir = ROOT / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = log_dir / f"cr5_mujoco_twin_{args.source}_{stamp}.jsonl"
    viewer = None
    if args.gui:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, data)

    period = 1.0 / args.rate
    started = time.monotonic()
    deadline = started + args.duration
    next_tick = started
    samples = 0
    try:
        with log_path.open("x", encoding="utf-8") as log:
            session = {
                "timestamp_utc": utc_now(),
                "kind": "session",
                "source": args.source,
                "gui": args.gui,
                "urdf": str(SOURCE_URDF),
                "generated_urdf": str(GENERATED_URDF),
                "joint_names": list(JOINT_NAMES),
                "model_nq": model.nq,
                "model_nbody": model.nbody,
                "commands_to_dobot": 0,
                "writes_to_uarm": 0,
            }
            log.write(json.dumps(session, ensure_ascii=False) + "\n")
            print(json.dumps(session, ensure_ascii=False))
            while time.monotonic() < deadline and (viewer is None or viewer.is_running()):
                now = time.monotonic()
                pose, metadata = source.read(now - started)
                apply_degrees(model, data, qpos_addresses, pose)
                if viewer is not None:
                    viewer.sync()
                record = {
                    "timestamp_utc": utc_now(),
                    "kind": "sample",
                    "sample": samples,
                    "joint_deg": pose,
                    "link6": link6_pose(model, data),
                    "metadata": metadata,
                }
                log.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
                if samples == 0 or samples % max(1, int(args.rate)) == 0:
                    print(json.dumps(record, ensure_ascii=False, allow_nan=False))
                samples += 1
                next_tick += period
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
    finally:
        if viewer is not None:
            viewer.close()
        source.close()

    summary = {
        "log": str(log_path),
        "samples": samples,
        "source": args.source,
        "commands_to_dobot": 0,
        "writes_to_uarm": 0,
    }
    print(json.dumps(summary, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
