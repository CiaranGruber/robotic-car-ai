#!/usr/bin/env python3
"""Manual, standalone ROS 2 pan test; no drive, enable, TF or policy commands.

Without --live this is a ROS-free dry run. In live mode stop ai4r_policy first
and keep the vehicle Disabled. A pan servo can move even while drive is disabled.
Targets are normalized positions, not angles. No command is sent at startup or
exit. Enter 0 explicitly to centre; q/Ctrl+C leaves the last target unchanged.
This tests the actuator only: off-centre cone coordinates need a correct TF.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import logging
import math
from pathlib import Path
import time

_LOGGER = logging.getLogger(__name__)


def positive_limit(text: str) -> float:
    """Parse a normalized positive bound."""
    value = float(text)
    if not math.isfinite(value) or not 0 < value <= 1:
        raise argparse.ArgumentTypeError("must be finite and in (0, 1]")
    return value


def target_value(text: str, last: float | None, limit: float, step: float) -> float:
    """Validate an explicit absolute target; initial position is unknown."""
    value = float(text)
    if not math.isfinite(value) or abs(value) > limit:
        raise ValueError(f"target must be finite and within [-{limit}, {limit}]")
    if last is None and value != 0:
        raise ValueError("first enter 0 to request centre; observe the servo before continuing")
    if last is not None and abs(value - last) > step + 1e-12:
        raise ValueError(f"change at most {step} per command; last requested target was {last}")
    return value


class RosPan:
    """Publish a short burst only on an explicit operator command."""

    def __init__(self, namespace: str):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_msgs.msg import Float32

        self.ros = rclpy
        self.message_type = Float32
        rclpy.init()
        self.node = rclpy.create_node("manual_camera_pan_test", namespace=namespace)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.node.create_publisher(Float32, "pan_set_point_normalized", qos)
        self.topic = self.publisher.topic_name

    def ready(self) -> None:
        """Refuse known competing publishers or an absent receiver.

        ROS discovery is asynchronous; this check is not a hardware interlock.
        """
        self.ros.spin_once(self.node, timeout_sec=0.05)
        if self.node.count_publishers(self.topic) > 1:
            raise RuntimeError("another pan publisher exists; stop ai4r_policy/other pan tools first")
        if self.publisher.get_subscription_count() == 0:
            raise RuntimeError("no pan subscriber discovered; check namespace and vehicle interface")

    def send(self, value: float) -> None:
        """Send the same requested target three times, checking each time."""
        for _ in range(3):
            self.ready()
            self.publisher.publish(self.message_type(data=float(value)))
            self.ros.spin_once(self.node, timeout_sec=0.1)

    def close(self) -> None:
        """Exit without commanding another physical movement."""
        self.node.destroy_node()
        if self.ros.ok():
            self.ros.shutdown()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--live", action="store_true", help="publish to ROS; otherwise dry run")
    parser.add_argument("--namespace", default="/car", help="robot ROS namespace (default /car)")
    parser.add_argument("--limit", type=positive_limit, default=0.15,
                        help="absolute target bound, default 0.15; not a verified mechanical limit")
    parser.add_argument("--step", type=positive_limit, default=0.05,
                        help="maximum change per command, default 0.05")
    parser.add_argument("--log", type=Path,
                        default=Path("pan-test-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ".csv"),
                        help="new CSV file for command records (never overwrite)")
    args = parser.parse_args(argv)
    if args.step > args.limit:
        parser.error("--step must not exceed --limit")

    logging.basicConfig(level=logging.INFO, format="%(message)s")
    backend = None
    last = None
    try:
        # Fail before connecting or moving if the log cannot be created.
        with args.log.open("x", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["utc", "mode", "requested_target", "status", "note"])
            stream.flush()

            def record(value, status, note=""):
                writer.writerow([datetime.now(timezone.utc).isoformat(),
                                 "live" if args.live else "dry-run", value, status, note])
                stream.flush()

            if args.live:
                backend = RosPan(args.namespace)
                # Discovery only; never publish a startup target.
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    backend.ros.spin_once(backend.node, timeout_sec=0.1)
                _LOGGER.info("LIVE topic: %s", backend.topic)
                _LOGGER.info("Keep vehicle Disabled, policy stopped, and servo travel clear.")
                _LOGGER.info("Checks cannot verify physical vehicle state or servo position.")
            else:
                _LOGGER.info("DRY RUN: no ROS connection and no hardware commands.")
            _LOGGER.info("Absolute targets only: start with 0, then small steps (e.g. 0.05).")
            _LOGGER.info("Range +/- %s, max step %s; sign/angle must be measured.", args.limit, args.step)
            _LOGGER.info("Enter 'note YOUR OBSERVATION' to record direction/alignment; q exits.")
            _LOGGER.info("No automatic sweep or automatic return to centre. CSV: %s", args.log)
            while True:
                try:
                    text = input("pan target / note / q > ").strip()
                except EOFError:
                    break
                if text.lower() in ("q", "quit", "exit"):
                    break
                if text.startswith("note "):
                    record("" if last is None else last, "observation", text[5:])
                    continue
                try:
                    value = target_value(text, last, args.limit, args.step)
                except ValueError as exc:
                    _LOGGER.warning("Rejected: %s", exc)
                    continue
                # Write intent before any command. 'sent' is not a position acknowledgement.
                record(value, "requested")
                try:
                    if backend is not None:
                        backend.send(value)
                except RuntimeError as exc:
                    record(value, "interrupted_or_rejected", str(exc))
                    _LOGGER.error("Stopping: %s. Some burst messages may already have been sent.", exc)
                    return 1
                last = value
                record(value, "sent_unverified" if backend is not None else "dry_run")
                _LOGGER.info("Requested %+.3f; inspect the camera before entering another target.", value)
    except KeyboardInterrupt:
        _LOGGER.info("Interrupted; no return-to-centre command sent.")
    except (OSError, ImportError) as exc:
        _LOGGER.error("%s", exc)
        if isinstance(exc, ImportError):
            _LOGGER.error("Live mode requires the car's ROS 2 Python environment (rclpy, std_msgs).")
        return 1
    finally:
        if backend is not None:
            backend.close()
    _LOGGER.info("Finished. Last target is held; this does not confirm physical alignment.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
