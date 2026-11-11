"""Offline uploader regression checks. No serial, network, or flash access."""
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import flash_gripper_gateway as uploader


class ReviewedFirmwareUploadTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="reviewed_firmware_")
        self.project = Path(self.temporary.name)
        self.payload = b"test-only reviewed application image"
        self.digest = hashlib.sha256(self.payload).hexdigest().upper()

    def tearDown(self):
        self.temporary.cleanup()

    def write_image(self, relative, payload=None):
        path = self.project / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(self.payload if payload is None else payload)
        return path

    def test_prebuilt_takes_priority_over_developer_build(self):
        self.write_image(".pio/build/esp32dev/firmware.bin", b"unreviewed build")
        expected = self.write_image("prebuilt/firmware.bin")
        self.assertEqual(uploader.resolve_binary(self.project), expected)

    def test_developer_build_is_fallback_when_prebuilt_absent(self):
        expected = self.write_image(".pio/build/esp32dev/firmware.bin")
        self.assertEqual(uploader.resolve_binary(self.project), expected)

    def test_missing_reviewed_image_is_error(self):
        with self.assertRaises(FileNotFoundError):
            uploader.resolve_binary(self.project)

    def test_exact_verified_image_is_written_without_platformio_rebuild(self):
        image = self.write_image("prebuilt/firmware.bin")
        with patch.object(uploader, "EXPECTED_SHA256", self.digest), \
             patch.object(uploader.importlib.util, "find_spec", return_value=object()), \
             patch.object(uploader.subprocess, "run", return_value=subprocess.CompletedProcess([], 0)) as run:
            uploader.upload_verified_binary(image, "COM_TEST_ONLY")
        args = run.call_args.args[0]
        self.assertEqual(args[:3], [sys.executable, "-m", "esptool"])
        self.assertEqual(args[-3:], ["write_flash", "0x10000", str(image)])
        self.assertEqual(args[args.index("--port") + 1], "COM_TEST_ONLY")
        self.assertNotIn("upload", args)
        self.assertEqual(image.read_bytes(), self.payload)

    def test_changed_image_is_rejected_before_uploader_starts(self):
        image = self.write_image("prebuilt/firmware.bin")
        with patch.object(uploader, "EXPECTED_SHA256", self.digest), \
             patch.object(uploader.importlib.util, "find_spec", return_value=object()), \
             patch.object(uploader.subprocess, "run") as run:
            self.assertEqual(uploader.verify_binary(image), self.digest)
            image.write_bytes(b"changed after preflight")
            with self.assertRaisesRegex(RuntimeError, "hash"):
                uploader.upload_verified_binary(image, "COM_TEST_ONLY")
            run.assert_not_called()

    def test_missing_tool_has_actionable_error_and_no_upload(self):
        image = self.write_image("prebuilt/firmware.bin")
        with patch.object(uploader.importlib.util, "find_spec", return_value=None), \
             patch.object(uploader.subprocess, "run") as run:
            with self.assertRaisesRegex(RuntimeError, "INSTALL.bat"):
                uploader.upload_verified_binary(image, "COM_TEST_ONLY")
            run.assert_not_called()


if __name__ == "__main__":
    unittest.main()
