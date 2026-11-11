"""Guarded uploader for the T-CAN485 DM4310 build 1.5 firmware.

Default is a read-only preflight. Flashing requires both --flash and the exact
confirmation token. The U-Arm CH343 (VID:PID 1A86:55D3) is never accepted.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import socket
import subprocess
import sys
import time

from serial.tools import list_ports

from gripper import (
    CAN_RESPONSE_HEADER, GATEWAY_HEADER, PACKETS, decode_feedback,
    decode_gateway_event, load_config, receive_packet,
)
from uarm_dobot_teleop import FeedbackStream

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "firmware/tcan485_dm4310_read_probe"
BINARY = PROJECT / ".pio/build/esp32dev/firmware.bin"
PIO = ROOT / ".venv-platformio/Scripts/pio.exe"
EXPECTED_SHA256 = "8E53ABDF39FD750A421A98CC505FB682B9989CE514A8809815839A081C358F7A"
CONFIRM = "I_UNDERSTAND_TCAN485_FIRMWARE_WILL_BE_REPLACED"
CH9102_VID_PID = (0x1A86, 0x55D4)
UARM_CH343_VID_PID = (0x1A86, 0x55D3)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def find_gateway_port() -> dict[str, object]:
    ports = list(list_ports.comports())
    gateway = [p for p in ports if (p.vid, p.pid) == CH9102_VID_PID]
    if any((p.vid, p.pid) == UARM_CH343_VID_PID and p in gateway for p in ports):
        raise RuntimeError("U-Arm CH343 was incorrectly classified as gateway")
    if len(gateway) != 1:
        observed = [
            {"device": p.device, "description": p.description,
             "vid": p.vid, "pid": p.pid, "serial": p.serial_number}
            for p in ports
        ]
        raise RuntimeError(
            "expected exactly one T-CAN485 CH9102 (VID:PID 1A86:55D4); "
            f"observed={observed}"
        )
    port = gateway[0]
    return {"device": port.device, "description": port.description,
            "vid": port.vid, "pid": port.pid, "serial": port.serial_number}


def request_packet(host: str, request: bytes, header: bytes, timeout: float) -> bytes:
    sock = None
    try:
        sock = socket.create_connection((host, 60000), timeout=3.0)
        sock.settimeout(.05)
        sock.sendall(request)
        return receive_packet(sock, bytearray(), header, timeout)
    finally:
        if sock is not None:
            sock.close()


def identity(host: str) -> dict[str, object]:
    return decode_gateway_event(
        request_packet(host, PACKETS["identity"], GATEWAY_HEADER, 3.0)
    )


def dm_status(host: str) -> dict[str, object]:
    config = load_config()
    return decode_feedback(
        request_packet(host, PACKETS["status"], CAN_RESPONSE_HEADER, 5.0),
        float(config["pmax_rad"]), float(config["vmax_rad_s"]),
        float(config["tmax_nm"]),
    )


def require_safe_preflight(host: str) -> dict[str, object]:
    feedback = FeedbackStream(host, timeout=3.0)
    try:
        state = feedback.wait_first(3.0)
    finally:
        feedback.close()
    if not (
        int(state["robot_mode"]) == 4
        and int(state["enable_status"]) == 0
        and int(state["running_status"]) == 0
        and int(state["error_status"]) == 0
    ):
        raise RuntimeError("CR5 must be disabled, idle and fault-free before gateway flash")
    motor = dm_status(host)
    if not (
        int(motor["error_code"]) == 0
        and abs(float(motor["velocity_rad_s"])) <= .20
        and int(motor["mos_temperature_c"]) < 60
        and int(motor["rotor_temperature_c"]) < 60
    ):
        raise RuntimeError(f"DM4310 preflight rejected: {motor}")
    return {"cr5": {
        "robot_mode": state["robot_mode"], "enable_status": state["enable_status"],
        "running_status": state["running_status"], "error_status": state["error_status"],
    }, "dm4310": motor, "identity": identity(host)}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.201.1")
    parser.add_argument("--flash", action="store_true")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args(argv)
    if args.flash and args.confirm != CONFIRM:
        parser.error(f"--flash requires --confirm {CONFIRM}")

    report: dict[str, object] = {
        "timestamp_utc": utc_now(), "flash_requested": args.flash,
        "expected_sha256": EXPECTED_SHA256,
    }
    code = 1
    try:
        actual_hash = hashlib.sha256(BINARY.read_bytes()).hexdigest().upper()
        report["binary"] = str(BINARY)
        report["actual_sha256"] = actual_hash
        if actual_hash != EXPECTED_SHA256:
            raise RuntimeError("firmware hash does not match the reviewed build")
        report["gateway_port"] = find_gateway_port()
        report["preflight"] = require_safe_preflight(args.host)
        if not args.flash:
            report["result"] = "read-only preflight passed; firmware not changed"
            code = 0
        else:
            if not PIO.exists():
                raise RuntimeError(f"PlatformIO executable missing: {PIO}")
            command = [str(PIO), "run", "-d", str(PROJECT), "-e", "esp32dev",
                       "-t", "upload", "--upload-port",
                       str(report["gateway_port"]["device"])]
            completed = subprocess.run(command, cwd=ROOT, capture_output=True,
                                       text=True, encoding="utf-8", errors="replace")
            report["upload_exit_code"] = completed.returncode
            report["upload_stdout"] = completed.stdout[-12000:]
            report["upload_stderr"] = completed.stderr[-4000:]
            if completed.returncode != 0:
                raise RuntimeError("PlatformIO upload failed")
            deadline = time.monotonic()+15.0
            observed = None
            last_error = None
            while time.monotonic() < deadline:
                try:
                    observed = identity(args.host)
                    if abs(float(observed["position_rad"])-1.5) < .01:
                        break
                except Exception as exc:
                    last_error = f"{type(exc).__name__}: {exc}"
                time.sleep(.5)
            report["post_flash_identity"] = observed
            report["post_flash_identity_last_error"] = last_error
            if observed is None or abs(float(observed["position_rad"])-1.5) >= .01:
                raise RuntimeError("build 1.5 network identity was not observed after upload")
            report["post_flash_dm4310"] = dm_status(args.host)
            report["result"] = "build 1.5 flashed; read-only identity/status passed"
            code = 0
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        report["completed_at_utc"] = utc_now()
        log = ROOT / "logs" / f"gripper_gateway_flash_{datetime.now():%Y%m%d_%H%M%S}.json"
        log.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        report["log_file"] = str(log)
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
