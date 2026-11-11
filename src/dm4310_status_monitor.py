"""Collect repeated read-only DM-J4310 status samples through the CR5AF gateway."""
from __future__ import annotations

import argparse
import json
import socket
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

from dm4310_status_probe import decode_feedback, extract_packet, request_packet


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only DM4310 status monitor")
    parser.add_argument("--host", default="192.168.201.1")
    parser.add_argument("--com", default="COM6")
    parser.add_argument("--motor-id", type=lambda value: int(value, 0), default=1)
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--pmax", type=float, default=12.5)
    parser.add_argument("--vmax", type=float, default=30.0)
    parser.add_argument("--tmax", type=float, default=10.0)
    args = parser.parse_args()
    if not 1 <= args.samples <= 100:
        raise SystemExit("--samples must be between 1 and 100")
    if args.interval < 0.05:
        raise SystemExit("--interval must be at least 0.05 seconds")

    request = request_packet(args.motor_id)
    report: dict[str, object] = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "operation": "repeated DM status refresh (official opcode 0xCC, read-only)",
        "requested_samples": args.samples,
        "interval_seconds": args.interval,
        "request_hex": request.hex(" ").upper(),
        "scales": {"PMAX": args.pmax, "VMAX": args.vmax, "TMAX": args.tmax},
        "samples": [],
    }
    serial_port = None
    tcp_socket = None
    tcp_buffer = bytearray()
    usb_log = bytearray()
    try:
        serial_port = serial.Serial(args.com, 115200, timeout=0.02)
        serial_port.reset_input_buffer()
        startup_deadline = time.monotonic() + 4.0
        while time.monotonic() < startup_deadline:
            usb_log.extend(serial_port.read(4096))

        tcp_socket = socket.create_connection((args.host, 60000), timeout=3.0)
        tcp_socket.settimeout(0.03)
        monitor_start = time.monotonic()
        for index in range(args.samples):
            target_time = monitor_start + index * args.interval
            remaining = target_time - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            sent_at = time.monotonic()
            tcp_socket.sendall(request)
            response = None
            deadline = sent_at + args.timeout
            while time.monotonic() < deadline:
                usb_log.extend(serial_port.read(4096))
                try:
                    data = tcp_socket.recv(4096)
                    if data:
                        tcp_buffer.extend(data)
                except socket.timeout:
                    pass
                response = extract_packet(tcp_buffer)
                if response is not None:
                    break
            sample: dict[str, object] = {
                "index": index,
                "elapsed_seconds": sent_at - monitor_start,
                "response_observed": response is not None,
            }
            if response is not None:
                sample["latency_ms"] = (time.monotonic() - sent_at) * 1000.0
                sample["packet_hex"] = response.hex(" ").upper()
                sample["feedback"] = decode_feedback(
                    response, args.pmax, args.vmax, args.tmax
                )
            report["samples"].append(sample)
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if tcp_socket is not None:
            tcp_socket.close()
        if serial_port is not None:
            serial_port.close()

    report["usb_log"] = usb_log.decode("utf-8", errors="replace")
    successful = [
        sample for sample in report["samples"] if sample.get("response_observed")
    ]
    report["successful_samples"] = len(successful)
    report["all_samples_successful"] = len(successful) == args.samples
    if successful:
        feedback = [sample["feedback"] for sample in successful]
        latencies = [sample["latency_ms"] for sample in successful]
        report["summary"] = {
            "latency_ms_min": min(latencies),
            "latency_ms_max": max(latencies),
            "latency_ms_mean": sum(latencies) / len(latencies),
            "error_codes": sorted({item["error_code"] for item in feedback}),
            "states": sorted({item["state"] for item in feedback}),
            "position_rad_min": min(item["position_rad"] for item in feedback),
            "position_rad_max": max(item["position_rad"] for item in feedback),
            "velocity_rad_s_min": min(item["velocity_rad_s"] for item in feedback),
            "velocity_rad_s_max": max(item["velocity_rad_s"] for item in feedback),
            "torque_nm_min": min(item["torque_nm"] for item in feedback),
            "torque_nm_max": max(item["torque_nm"] for item in feedback),
            "mos_temperature_c": sorted({item["mos_temperature_c"] for item in feedback}),
            "rotor_temperature_c": sorted(
                {item["rotor_temperature_c"] for item in feedback}
            ),
        }

    log_dir = Path(__file__).resolve().parents[1] / "logs"
    log_dir.mkdir(exist_ok=True)
    log_file = log_dir / f"dm4310_status_monitor_{datetime.now():%Y%m%d_%H%M%S}.json"
    log_file.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    report["log_file"] = str(log_file)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["all_samples_successful"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
