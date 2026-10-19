#!/usr/bin/env python3
"""Read-only ArUco topic test. Does not open the camera or publish any commands.

Run on the car in its ROS 2 + dream_interfaces environment. No student build is
needed. Positions are in the incoming header.frame_id, normally camera optical
coordinates, NOT automatically base_link. A reported valid pose is not verified
physical accuracy. JSONL logs contain one record per received detection batch.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import logging
import math
from pathlib import Path
import time

_LOGGER = logging.getLogger(__name__)


def finite_or_none(value):
    return float(value) if math.isfinite(value) else None


def batch_record(msg):
    """Extract the same public FiducialDetections fields used by policy_node."""
    markers = []
    for detection in msg.detections:
        position = detection.pose.position
        quaternion = detection.pose.orientation
        xyz = [finite_or_none(v) for v in (position.x, position.y, position.z)]
        quat = [finite_or_none(v) for v in (quaternion.x, quaternion.y, quaternion.z, quaternion.w)]
        error = finite_or_none(detection.reprojection_error_px)
        usable = (bool(detection.pose_valid) and all(v is not None for v in xyz + quat)
                  and error is not None and error >= 0
                  and math.isclose(math.hypot(*quat), 1.0, abs_tol=1e-3))
        markers.append({
            "id": int(detection.id), "marker_size_m": finite_or_none(detection.marker_size_m),
            "corners_px": [[finite_or_none(c.u), finite_or_none(c.v)] for c in detection.corners],
            "pose_reported_valid": bool(detection.pose_valid), "pose_usable": usable,
            "position_xyz_m": xyz if usable else None,
            "orientation_xyzw": quat if usable else None, "reprojection_error_px": error,
        })
    return {"stamp_ns": msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
            "frame_id": msg.header.frame_id, "dictionary_name": msg.dictionary_name,
            "detections": markers}


class Monitor:
    """Track stream freshness independently of target visibility."""

    def __init__(self, target=2, dictionary="DICT_4X4_50", timeout=1.0):
        self.target, self.dictionary, self.timeout = target, dictionary, timeout
        self.latest = None
        self.received_at = None
        self.previous_stamp = None
        self.rejection = None
        self.total = self.accepted = self.seen = 0

    def update(self, record, monotonic_now, ros_now_ns):
        self.total += 1
        self.latest, self.received_at = record, monotonic_now
        stamp = record["stamp_ns"]
        age = (ros_now_ns - stamp) / 1e9
        if stamp <= 0 or age < -0.05 or age >= self.timeout:
            self.rejection = "BAD_TIMESTAMP"
        elif self.previous_stamp is not None and stamp <= self.previous_stamp:
            self.rejection = "REPEATED_OR_OLDER_TIMESTAMP"
        else:
            self.rejection = None
        # Allows recovery after a ROS-clock reset, without accepting the reset batch.
        self.previous_stamp = stamp
        if self.rejection is None and record["dictionary_name"] == self.dictionary:
            self.accepted += 1
            self.seen += bool(self.targets())
        return self.rejection

    def targets(self):
        if self.latest is None:
            return []
        return [m for m in self.latest["detections"] if m["id"] == self.target]

    def status(self, monotonic_now, ros_now_ns):
        if self.latest is None:
            return "WAITING_FOR_MESSAGES"
        age = max(monotonic_now - self.received_at,
                  (ros_now_ns - self.latest["stamp_ns"]) / 1e9)
        if age >= self.timeout:
            return "STREAM_STALE"
        if self.rejection:
            return self.rejection
        if self.latest["dictionary_name"] != self.dictionary:
            return "DICTIONARY_MISMATCH"
        targets = self.targets()
        if not targets:
            return "TARGET_NOT_VISIBLE"
        if len(targets) > 1:
            return "MULTIPLE_TARGETS"
        return "TARGET_SEEN_POSE_OK" if targets[0]["pose_usable"] else "TARGET_SEEN_NO_POSE"


def nonnegative(text):
    value = float(text)
    if not math.isfinite(value) or value < 0:
        raise argparse.ArgumentTypeError("must be finite and nonnegative")
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/car/fiducial_detections")
    parser.add_argument("--id", type=int, default=2, dest="target")
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument("--timeout", type=nonnegative, default=1.0)
    parser.add_argument("--duration", type=nonnegative, default=60.0, help="seconds; 0 runs until Ctrl+C")
    parser.add_argument("--log", type=Path,
                        default=Path("aruco-test-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ".jsonl"))
    args = parser.parse_args(argv)
    if args.timeout <= 0 or args.target < 0:
        parser.error("timeout must be positive and ID nonnegative")
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    try:
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from dream_interfaces.msg import FiducialDetections
    except ImportError as exc:
        _LOGGER.error("Use the car's ROS environment with dream_interfaces: %s", exc)
        return 1
    node = None
    monitor = Monitor(args.target, args.dictionary, args.timeout)
    try:
        with args.log.open("x", encoding="utf-8") as stream:
            rclpy.init()
            node = rclpy.create_node("aruco_detection_test")

            def receive(msg):
                record = batch_record(msg)
                record["received_utc"] = datetime.now(timezone.utc).isoformat()
                record["timestamp_rejection"] = monitor.update(
                    record, time.monotonic(), node.get_clock().now().nanoseconds)
                stream.write(json.dumps(record, allow_nan=False) + "\n")
                stream.flush()

            # Best effort also receives a reliable publisher and minimizes queued old samples.
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            node.create_subscription(FiducialDetections, args.topic, receive, qos)
            _LOGGER.info("READ ONLY: topic=%s expected=%s ID=%s log=%s", args.topic, args.dictionary, args.target, args.log)
            _LOGGER.info("No camera access, drive, steering, pan, enable or state commands. Duration: %ss", args.duration)
            start = last_report = time.monotonic()
            while rclpy.ok() and (args.duration == 0 or time.monotonic() - start < args.duration):
                rclpy.spin_once(node, timeout_sec=0.1)
                now = time.monotonic()
                if now - last_report >= 1.0:
                    status = monitor.status(now, node.get_clock().now().nanoseconds)
                    latest = monitor.latest
                    _LOGGER.info("%s | publishers=%s batches=%s | ids=%s | frame=%s dictionary=%s",
                                 status, node.count_publishers(args.topic), monitor.total,
                                 [] if latest is None else [m["id"] for m in latest["detections"]],
                                 "-" if latest is None else latest["frame_id"],
                                 "-" if latest is None else latest["dictionary_name"])
                    if status == "TARGET_SEEN_POSE_OK":
                        marker = monitor.targets()[0]
                        _LOGGER.info("  raw-frame xyz(m)=%s size(m)=%s reprojection(px)=%s [not ground-truth validation]",
                                     marker["position_xyz_m"], marker["marker_size_m"], marker["reprojection_error_px"])
                    last_report = now
    except KeyboardInterrupt:
        pass
    except OSError as exc:
        _LOGGER.error("Log error: %s", exc)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    _LOGGER.info("Summary: %s received batches; %s fresh expected-dictionary batches; target present in %s.",
                 monitor.total, monitor.accepted, monitor.seen)
    _LOGGER.info("These counts are NOT raw-image detection accuracy and do not prove driving readiness.")
    return 0 if monitor.accepted else 2


if __name__ == "__main__":
    raise SystemExit(main())
