#!/usr/bin/env python3
"""Draws the detected lane in Foxglove, so it can be checked over the camera image. It never drives the car.

On every cone batch it runs the policy's cone filter and lane detection (policy/cone_filter, policy/lane_detection),
and publishes:
- lane_overlay/markers (visualization_msgs/MarkerArray, in the cones' frame, base_link): the left edge, right edge and
  centre line on the ground, the kept cones in their colour and the filtered-out cones in red, and a text summary.
  Foxglove's Image panel draws them over the camera image when its calibration (camera_info) topic is set; the 3D
  panel shows them too.
- lane_overlay/lateral_error_m, lane_overlay/heading_error_deg and lane_overlay/lane_width_m (std_msgs/Float32), for
  the Plot panel, only while a lane is found.

It only subscribes to cone_detections. It publishes no actions and does not touch the policy state, so it can run
beside policy_node in any state.

Run it on the car from the folder holding policy/ and scripts/, with ROS sourced:
    python3 scripts/lane_overlay_node.py --ros-args -r __ns:=/car

Its settings are the cone_filter.* and lane_detection.* parameters of config/ai4r_policy.yaml, with the same defaults.
Change one with -p, for example for a curved lane, whose cones are not on straight rows:
    python3 scripts/lane_overlay_node.py --ros-args -r __ns:=/car -p cone_filter.check_straight_rows:=false
"""
from dataclasses import asdict
import math
from pathlib import Path
import sys

import rclpy
from builtin_interfaces.msg import Duration
from dream_interfaces.msg import ConeDetection as ConeDetectionMsg, ConeDetections
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, Float32
from visualization_msgs.msg import Marker, MarkerArray

# The policy package is beside this script's folder, and is not installed with the ROS package
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones  # noqa: E402
from policy.input_output import (CarObservations, ConeBatch, ConeColour, ConeDetection,  # noqa: E402
                                 Position, SensorAge)
from policy.lane_detection.lane_detection import LaneDetectionConfig, LaneDetector  # noqa: E402
from policy.lane_detection.lanes import LaneArc  # noqa: E402

COLOURS = {
    "left": ColorRGBA(r=0.18, g=0.5, b=0.93, a=1.0),
    "right": ColorRGBA(r=0.95, g=0.79, b=0.3, a=1.0),
    "centre": ColorRGBA(r=0.2, g=0.85, b=0.3, a=1.0),
    "removed": ColorRGBA(r=0.9, g=0.2, b=0.2, a=1.0),
    "text": ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),
}
"""Marker colours: blue left edge, yellow right edge, green centre line, red filtered-out cones."""

CONE_COLOURS = {ConeColour.BLUE: COLOURS["left"], ConeColour.YELLOW: COLOURS["right"]}
"""Marker colour of a kept cone."""

LINE_WIDTH_M = 0.03
"""Width of the drawn lines in metres."""

LINE_STEP_M = 0.1
"""Distance in metres along a drawn line between its points."""

MARKER_LIFETIME_S = 0.5
"""Seconds a drawing stays without a newer cone batch, so a stale lane disappears."""


class LaneOverlayNode(Node):
    """Publishes the lane detected from each cone batch for Foxglove."""

    def __init__(self):
        super().__init__("lane_overlay")
        self.cone_filter_config = ConeFilterConfig(**self._read_parameters("cone_filter", ConeFilterConfig()))
        self.lane_detector = LaneDetector(
            LaneDetectionConfig(**self._read_parameters("lane_detection", LaneDetectionConfig())))
        self.marker_publisher = self.create_publisher(MarkerArray, "lane_overlay/markers", 10)
        self.value_publishers = {name: self.create_publisher(Float32, f"lane_overlay/{name}", 10)
                                 for name in ("lateral_error_m", "heading_error_deg", "lane_width_m")}
        self.create_subscription(ConeDetections, "cone_detections", self.cone_detection_callback,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().info(f"Drawing the lane from {self.resolve_topic_name('cone_detections')} on "
                               f"{self.resolve_topic_name('lane_overlay/markers')}")

    def _read_parameters(self, prefix: str, defaults) -> dict:
        """Declares a parameter for each setting of a config dataclass, as policy_node does.

        :return: The settings' values, keyed by name.
        """
        values = {}
        for name, default in asdict(defaults).items():
            self.declare_parameter(f"{prefix}.{name}", default)
            values[name] = self.get_parameter(f"{prefix}.{name}").value
        return values

    def cone_detection_callback(self, msg: ConeDetections):
        colours = {ConeDetectionMsg.COLOR_YELLOW: ConeColour.YELLOW, ConeDetectionMsg.COLOR_BLUE: ConeColour.BLUE}
        cones = [ConeDetection(Position(cone.position.x, cone.position.y, cone.position.z), colours[cone.color],
                               cone.classification_confidence)
                 for cone in msg.detections if cone.color in colours]
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        batch = ConeBatch(cones, SensorAge(0.0, stamp_ns), msg.acquisition_to_publish_latency_s)
        kept = filter_cones(batch, self.cone_filter_config)
        observations = CarObservations(cones=kept, fiducials=None, lidar_scan=None, lidar_cartesian=None, car=None,
                                       policy=None)
        lane = self.lane_detector.detect(observations)

        markers = [Marker(header=msg.header, action=Marker.DELETEALL)]
        if lane.left_edge is not None:
            markers.append(self._line(msg, 0, lane.left_edge.arc, lane.left_edge.nearest_m,
                                      lane.left_edge.farthest_m, COLOURS["left"]))
        if lane.right_edge is not None:
            markers.append(self._line(msg, 1, lane.right_edge.arc, lane.right_edge.nearest_m,
                                      lane.right_edge.farthest_m, COLOURS["right"]))
        if lane.has_lane:
            markers.append(self._line(msg, 2, lane.centre_line, 0.0, lane.visible_until_m, COLOURS["centre"]))
        markers.append(self._cones(msg, 3, [(cone, CONE_COLOURS[cone.colour]) for cone in kept]))
        removed = [cone for cone in cones if cone not in kept]
        markers.append(self._cones(msg, 4, [(cone, COLOURS["removed"]) for cone in removed]))

        if lane.has_lane:
            radius = 1.0 / lane.path_curvature_per_m if abs(lane.path_curvature_per_m) > 0.01 else math.inf
            summary = (f"{lane.status.name}  lateral {lane.lateral_error_m:+.2f} m  "
                       f"heading {math.degrees(lane.heading_error_rad):+.1f} deg  width {lane.lane_width_m:.2f} m  "
                       f"radius {radius:+.1f} m (+ left)")
            for name, value in (("lateral_error_m", lane.lateral_error_m),
                                ("heading_error_deg", math.degrees(lane.heading_error_rad)),
                                ("lane_width_m", lane.lane_width_m)):
                self.value_publishers[name].publish(Float32(data=float(value)))
        else:
            summary = f"NO_LANE  ({len(kept)} of {len(cones)} cones kept)"
        markers.append(self._text(msg, 5, summary))
        self.marker_publisher.publish(MarkerArray(markers=markers))
        self.get_logger().info(summary, throttle_duration_sec=1.0)

    def _marker(self, msg: ConeDetections, marker_id: int, marker_type: int) -> Marker:
        """
        :return: A marker in the cone batch's frame and time, which disappears after MARKER_LIFETIME_S.
        """
        marker = Marker(header=msg.header, ns="lane", id=marker_id, type=marker_type, action=Marker.ADD)
        marker.pose.orientation.w = 1.0
        marker.lifetime = Duration(sec=0, nanosec=int(MARKER_LIFETIME_S * 1e9))
        return marker

    def _line(self, msg: ConeDetections, marker_id: int, arc: LaneArc, start_m: float, end_m: float,
              colour: ColorRGBA) -> Marker:
        """
        :return: The arc drawn on the ground from start_m to end_m along it.
        """
        marker = self._marker(msg, marker_id, Marker.LINE_STRIP)
        marker.scale.x = LINE_WIDTH_M
        marker.color = colour
        steps = max(1, math.ceil((end_m - start_m) / LINE_STEP_M))
        points = [arc.point_at(start_m + (end_m - start_m) * i / steps) for i in range(steps + 1)]
        marker.points = [Point(x=x, y=y, z=0.0) for x, y in points]
        return marker

    def _cones(self, msg: ConeDetections, marker_id: int, cones: list[tuple[ConeDetection, ColorRGBA]]) -> Marker:
        """
        :return: A small sphere at each cone's detected point, in the given colour.
        """
        marker = self._marker(msg, marker_id, Marker.SPHERE_LIST)
        marker.scale.x = marker.scale.y = marker.scale.z = 0.06
        marker.color = COLOURS["text"]
        marker.points = [Point(x=cone.pos.x, y=cone.pos.y, z=cone.pos.z) for cone, _ in cones]
        marker.colors = [colour for _, colour in cones]
        return marker

    def _text(self, msg: ConeDetections, marker_id: int, text: str) -> Marker:
        """
        :return: The text floating 1 m ahead of the car.
        """
        marker = self._marker(msg, marker_id, Marker.TEXT_VIEW_FACING)
        marker.pose.position.x, marker.pose.position.z = 1.0, 0.3
        marker.scale.z = 0.06
        marker.color = COLOURS["text"]
        marker.text = text
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = LaneOverlayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
