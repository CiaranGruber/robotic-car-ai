"""Hardware-free tests for the standalone pan tool; no ROS or vehicle needed."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import csv
import sys

# Works from tests/policy in the repository or work/ beside outputs/.
ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "scripts" / "camera_pan_test.py"
if not SOURCE.exists():
    SOURCE = Path(__file__).with_name("camera_pan_test.py")
if not SOURCE.exists():
    SOURCE = Path(__file__).resolve().parents[1] / "outputs" / "camera_pan_test.py"
spec = importlib.util.spec_from_file_location("camera_pan_test", SOURCE)
pan = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pan)


class TestPanTool(unittest.TestCase):
    def test_unknown_start_requires_centre(self):
        with self.assertRaises(ValueError):
            pan.target_value("0.05", None, 0.15, 0.05)
        self.assertEqual(pan.target_value("0", None, 0.15, 0.05), 0)

    def test_nonfinite_and_outside_range_rejected(self):
        for value in ("nan", "inf", "-inf", "0.16", "-0.16", "bad"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                pan.target_value(value, 0, 0.15, 0.05)

    def test_large_jump_rejected_and_small_steps_allowed(self):
        with self.assertRaises(ValueError):
            pan.target_value("-0.10", 0.10, 0.15, 0.05)
        self.assertAlmostEqual(pan.target_value("0.15", 0.10, 0.15, 0.05), 0.15)

    def test_dry_run_logs_only_valid_commands_and_notes(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "log.csv"
            with patch("builtins.input", side_effect=["0.1", "0", "nan", "0.05", "note points right", "q"]):
                self.assertEqual(pan.main(["--log", str(path)]), 0)
            with path.open(encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual([r["requested_target"] for r in rows if r["status"] == "dry_run"], ["0.0", "0.05"])
            self.assertEqual(rows[-1]["note"], "points right")

    def test_log_never_overwritten(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "log.csv"
            path.write_text("original", encoding="utf-8")
            self.assertEqual(pan.main(["--log", str(path)]), 1)
            self.assertEqual(path.read_text(encoding="utf-8"), "original")

    def test_quit_never_sends_startup_or_shutdown_target(self):
        class Backend:
            topic = "/car/pan_set_point_normalized"
            node = None
            ros = type("Ros", (), {"spin_once": staticmethod(lambda *a, **k: None)})()
            sent = []
            closed = False
            def __init__(self, namespace):
                pass
            def send(self, value):
                self.sent.append(value)
            def close(self):
                Backend.closed = True
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(pan, "RosPan", Backend), patch.object(pan.time, "monotonic", side_effect=[0, 3]), patch("builtins.input", return_value="q"):
                self.assertEqual(pan.main(["--live", "--log", str(Path(folder) / "log.csv")]), 0)
        self.assertEqual(Backend.sent, [])
        self.assertTrue(Backend.closed)

    def test_competing_publisher_blocks_command(self):
        backend = object.__new__(pan.RosPan)
        backend.ros = type("Ros", (), {"spin_once": staticmethod(lambda *a, **k: None)})()
        backend.node = type("Node", (), {"count_publishers": lambda self, topic: 2})()
        backend.topic = "/car/pan_set_point_normalized"
        with self.assertRaisesRegex(RuntimeError, "another pan publisher"):
            backend.ready()

    def test_no_receiver_blocks_command(self):
        backend = object.__new__(pan.RosPan)
        backend.ros = type("Ros", (), {"spin_once": staticmethod(lambda *a, **k: None)})()
        backend.node = type("Node", (), {"count_publishers": lambda self, topic: 1})()
        backend.publisher = type("Publisher", (), {"get_subscription_count": lambda self: 0})()
        backend.topic = "/car/pan_set_point_normalized"
        with self.assertRaisesRegex(RuntimeError, "no pan subscriber"):
            backend.ready()


if __name__ == "__main__":
    unittest.main()
