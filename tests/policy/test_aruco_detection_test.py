"""Offline tests of the observer; never import ROS or connect to hardware."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

SOURCE = Path(__file__).resolve().parents[2] / "scripts" / "aruco_detection_test.py"
spec = importlib.util.spec_from_file_location("aruco_detection_test", SOURCE)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


def record(ids=(), pose=True, dictionary="DICT_4X4_50", stamp=10_000_000_000):
    return {"stamp_ns": stamp, "dictionary_name": dictionary, "frame_id": "optical",
            "detections": [{"id": id_, "pose_usable": pose} for id_ in ids]}


class MonitorTests(unittest.TestCase):
    def test_waiting_then_empty_then_visible(self):
        m = tool.Monitor()
        self.assertEqual(m.status(0, 10_000_000_000), "WAITING_FOR_MESSAGES")
        m.update(record(), 0, 10_000_000_000)
        self.assertEqual(m.status(0, 10_000_000_000), "TARGET_NOT_VISIBLE")
        m.update(record([2], stamp=10_100_000_000), .1, 10_100_000_000)
        self.assertEqual(m.status(.1, 10_100_000_000), "TARGET_SEEN_POSE_OK")
        self.assertEqual((m.accepted, m.seen), (2, 1))

    def test_visibility_disappears_on_empty_batch(self):
        m = tool.Monitor()
        m.update(record([2]), 0, 10_000_000_000)
        m.update(record(stamp=10_100_000_000), .1, 10_100_000_000)
        self.assertEqual(m.status(.1, 10_100_000_000), "TARGET_NOT_VISIBLE")

    def test_stale_cannot_keep_showing_target(self):
        m = tool.Monitor()
        m.update(record([2]), 0, 10_000_000_000)
        self.assertEqual(m.status(1.1, 11_100_000_000), "STREAM_STALE")

    def test_id_without_pose_and_multiple_ids(self):
        m = tool.Monitor()
        m.update(record([2], pose=False), 0, 10_000_000_000)
        self.assertEqual(m.status(0, 10_000_000_000), "TARGET_SEEN_NO_POSE")
        m.update(record([2, 2], stamp=10_100_000_000), .1, 10_100_000_000)
        self.assertEqual(m.status(.1, 10_100_000_000), "MULTIPLE_TARGETS")

    def test_wrong_dictionary_not_counted(self):
        m = tool.Monitor()
        m.update(record([2], dictionary="DICT_5X5_50"), 0, 10_000_000_000)
        self.assertEqual(m.status(0, 10_000_000_000), "DICTIONARY_MISMATCH")
        self.assertEqual(m.seen, 0)

    def test_bad_and_repeated_timestamps(self):
        for stamp in (0, 1_000_000_000, 11_000_000_000):
            m = tool.Monitor()
            self.assertEqual(m.update(record([2], stamp=stamp), 0, 10_000_000_000), "BAD_TIMESTAMP")
            self.assertEqual(m.seen, 0)
        m = tool.Monitor()
        m.update(record([2]), 0, 10_000_000_000)
        m.update(record([2]), .1, 10_100_000_000)
        self.assertEqual(m.status(.1, 10_100_000_000), "REPEATED_OR_OLDER_TIMESTAMP")
        self.assertEqual(m.seen, 1)

    def test_raw_message_conversion_and_invalid_pose(self):
        detection = NS(id=2, marker_size_m=.25, pose_valid=False,
                       corners=[NS(u=1., v=2.)] * 4, reprojection_error_px=float("nan"),
                       pose=NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.)))
        msg = NS(header=NS(stamp=NS(sec=10, nanosec=0), frame_id="optical"),
                 dictionary_name="DICT_4X4_50", detections=[detection])
        result = tool.batch_record(msg)
        self.assertIsNone(result["detections"][0]["position_xyz_m"])
        self.assertIsNone(result["detections"][0]["reprojection_error_px"])
        detection.pose_valid = True
        detection.pose.position.z = 1.
        detection.reprojection_error_px = .2
        self.assertTrue(tool.batch_record(msg)["detections"][0]["pose_usable"])
        detection.pose.orientation.w = 0.
        self.assertFalse(tool.batch_record(msg)["detections"][0]["pose_usable"])


if __name__ == "__main__":
    unittest.main()
