"""Offline checks for explicit deployment targets; no device I/O."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from device_connection import load_connection_settings
import feetech_port


class ConnectionSettingsTests(unittest.TestCase):
    def test_missing_file_preserves_existing_defaults(self):
        with tempfile.TemporaryDirectory() as directory:
            self.assertEqual(load_connection_settings(Path(directory)),
                             {"robot_host": "192.168.201.1", "leader_port": "auto"})

    def test_explicit_replacement_port_does_not_auto_scan(self):
        with patch.object(feetech_port, "load_connection_settings", return_value={"leader_port": "COM12"}), \
             patch.object(feetech_port.list_ports, "comports", side_effect=AssertionError("no device scan")):
            self.assertEqual(feetech_port.resolve_uarm_port("auto"), "COM12")
            self.assertEqual(feetech_port.resolve_uarm_port("COM9"), "COM9")

    def test_utf8_bom_config_and_nondefault_host(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "config").mkdir()
            (root / "config/device_connection.json").write_text(
                json.dumps({"robot_host": "192.168.10.2", "leader_port": "com12"}), encoding="utf-8-sig")
            self.assertEqual(load_connection_settings(root),
                             {"robot_host": "192.168.10.2", "leader_port": "COM12"})

    def test_invalid_destinations_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "config").mkdir()
            for payload in ({"robot_host": "bad hostname"}, {"leader_port": "COM0"},
                            {"leader_port": "COM1;command"}, {"leader_port": None}, {"port": "COM1"}):
                with self.subTest(payload=payload):
                    (root / "config/device_connection.json").write_text(json.dumps(payload), encoding="utf-8")
                    with self.assertRaises(ValueError):
                        load_connection_settings(root)
