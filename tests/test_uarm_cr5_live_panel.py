"""GUI lifecycle tests with a fake child; never open robot/serial connections."""
import json
from datetime import datetime, timedelta, timezone
from pathlib import Path
import sys
import tempfile
import tkinter as tk
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import uarm_cr5_live_panel as panel


class PanelLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.workspace = Path(self.temp.name)
        (self.workspace / "logs").mkdir()
        (self.workspace / "config").mkdir()
        (self.workspace / "config/uarm_dobot_teleop.json").write_text(
            json.dumps({"joint_signs": [1, -1, 1, 1, 1, 1]}), encoding="utf-8")
        self.root_patch = patch.object(panel, "ROOT", self.workspace)
        self.root_patch.start()
        self.child = Mock()
        self.child.poll.return_value = None
        self.spawn_patch = patch.object(panel.subprocess, "Popen", return_value=self.child)
        self.spawn = self.spawn_patch.start()
        self.root = tk.Tk()
        self.root.withdraw()
        self.ui = panel.LivePanel(self.root)

    def tearDown(self):
        if self.ui.console:
            self.ui.console.close()
        self.root.destroy()
        self.spawn_patch.stop()
        self.root_patch.stop()
        self.temp.cleanup()

    def test_opening_panel_never_launches_controller(self):
        self.spawn.assert_not_called()

    def test_explicit_start_has_no_duration_or_relative_cap_and_no_duplicate_child(self):
        self.ui.reversed[0].set(True)
        self.ui.enabled[5].set(False)
        self.ui.start()
        self.ui.start()
        self.spawn.assert_called_once()
        args = self.spawn.call_args.args[0]
        self.assertIn("--joint-signs=-1,-1,1,1,1,1", args)
        self.assertIn("--joint-sources=1,2,3,4,5,6", args)
        self.assertIn("--joint-scales=1,1,1,1,1,1", args)
        self.assertIn("--joint-offsets-deg=0,0,0,0,0,0", args)
        self.assertEqual(args[args.index("--max-speed-deg-s") + 1], "12")
        self.assertEqual(args[args.index("--max-follow-lag-deg") + 1], "20")
        self.assertEqual(args[args.index("--joints") + 1], "1,2,3,4,5")
        self.assertNotIn("--duration", args)
        self.assertNotIn("--max-relative-deg", args)
        self.assertTrue(self.ui.heartbeat.exists())

    def test_source_scale_and_speed_are_persisted_and_passed_to_child(self):
        self.ui.sources[0].set("2")
        self.ui.sources[1].set("1")
        self.ui.scales[0].set("0.5")
        self.ui.scales[1].set("1.5")
        self.ui.offsets[1].set("180")
        self.ui.max_speed.set("15")
        self.ui.max_follow_lag.set("30")
        self.ui.start()
        args = self.spawn.call_args.args[0]
        self.assertIn("--joint-sources=2,1,3,4,5,6", args)
        self.assertIn("--joint-scales=0.5,1.5,1,1,1,1", args)
        self.assertIn("--joint-offsets-deg=0,180,0,0,0,0", args)
        self.assertEqual(args[args.index("--max-speed-deg-s") + 1], "15")
        self.assertEqual(args[args.index("--max-follow-lag-deg") + 1], "30")
        saved = json.loads((self.workspace / "config/uarm_cr5_continuous_ui.json").read_text(encoding="utf-8"))
        self.assertEqual(saved["joint_sources"], [2,1,3,4,5,6])
        self.assertEqual(saved["joint_scales"][:2], [.5,1.5])
        self.assertEqual(saved["joint_offsets_deg"][:2], [0,180])
        self.assertEqual(saved["max_speed_deg_s"], 15)
        self.assertEqual(saved["max_follow_lag_deg"], 30)

    def test_stop_file_failure_still_withdraws_heartbeat(self):
        self.ui.start()
        with patch.object(Path, "write_text", side_effect=OSError("disk error")):
            self.ui.stop()
        self.assertIsNotNone(self.ui.stop_requested_at)
        with patch.object(self.ui, "write_heartbeat") as heartbeat:
            self.ui.tick()
        heartbeat.assert_not_called()

    def test_stale_live_telemetry_requests_stop(self):
        self.ui.start()
        stale = {"kind": "sample", "timestamp_utc": (datetime.now(timezone.utc) - timedelta(seconds=2)).isoformat(),
                 "cr5": {"q_actual_deg": [0] * 6}, "arm_target_deg": [1] * 6}
        with patch.object(panel, "tail_rows", return_value=[stale]):
            self.ui.tick()
        self.assertTrue(self.ui.stop_path.exists())
        self.assertIsNotNone(self.ui.stop_requested_at)

    def test_closing_requests_cleanup_without_killing_child(self):
        self.ui.start()
        self.ui.close()
        self.assertTrue(self.ui.closing)
        self.assertTrue(self.ui.stop_path.exists())
        self.child.kill.assert_not_called()
        self.child.terminate.assert_not_called()
        self.assertTrue(self.root.winfo_exists())

    def test_unverified_stop_blocks_new_start(self):
        self.ui.start()
        self.ui.finish(1, {"kind": "terminal", "cleanup": {"enable_requested": True,
                            "cr5_final_disable_verified": False}, "error": "feedback lost"})
        self.assertTrue(self.ui.uncertain_stop)
        self.assertEqual(str(self.ui.start_button["state"]), "disabled")

    def test_verified_stop_consumes_applied_offset_without_stacking(self):
        self.ui.offsets[1].set("180")
        self.ui.start()
        self.ui.stream.write_text(json.dumps({
            "kind": "sample",
            "joint_offset_progress_deg": [0, 126, 0, 0, 0, 0],
        }) + "\n", encoding="utf-8")
        self.ui.finish(0, {
            "kind": "terminal",
            "cleanup": {
                "enable_requested": True,
                "cr5_final_disable_verified": True,
                "cr5_final": {"robot_mode": 4, "enable_status": 0, "error_status": 0},
            },
        })
        saved = json.loads((self.workspace / "config/uarm_cr5_continuous_ui.json").read_text(encoding="utf-8"))
        self.assertEqual(saved["joint_offsets_deg"], [0, 54, 0, 0, 0, 0])
        self.assertEqual(self.ui.offsets[1].get(), "54")

    def test_preflight_failure_without_enable_allows_retry(self):
        self.ui.start()
        self.ui.finish(1, {"kind": "terminal", "cleanup": {"enable_requested": False},
                            "error": "COM busy"})
        self.assertFalse(self.ui.uncertain_stop)
        self.assertEqual(str(self.ui.start_button["state"]), "normal")


if __name__ == "__main__":
    unittest.main()
