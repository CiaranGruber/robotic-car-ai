"""What the car's sensors would report in the simulated world, in base_link, as the policy receives them.

Every model reuses a measured or documented value already in the repository:
- camera cones: real_car_measurements.LAB_CAR, measured from the lab recordings: an 86 degree view from 0.4 to 2.5 m
  ahead, 0.03 m position noise, about 0.2 s old, 10 Hz. Boxes hide the cones behind them, seen from the camera
  mount's default position (0.10 m behind base_link, config/camera_mount.yaml).
- ArUco markers: decoded marker poses from the same camera frames, as the aruco_detector would publish them once it
  has seen a marker in min_consecutive_frames frames (config/aruco_detector.yaml). A marker counts as seen inside the
  camera's view and range, facing the camera within MARKER_MAX_VIEW_ANGLE_DEG. Pixels are not simulated.
- lidar: obstacle_simulation.lidar_points, from the lidar mount's default (0.20 m ahead, 0.12 m up): 0.5 degree
  beams to 8 m with 0.01 m noise, cones cut at the scan height and boxes.
- wheel speed: the car's unsigned speed. Gyro yaw rate: speed times driven curvature. Heading: since the start.
The camera's real yaw (camera_yaw_deg) turns every camera detection the other way, as on the car; the policy
corrects for its own estimate, mpc.camera_yaw_offset_deg.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from policy.input_output import (ConeDetection, FiducialDetection, FiducialPose, ImagePoint, LidarCartesianPoint,
                                 Position, Quaternion)
from tests.policy.obstacle_simulation import CAMERA_FORWARD_M, CONE_RADIUS_M, cone_gaps, lidar_points, visible_cones
from tests.policy.real_car_measurements import LAB_CAR
from tests.policy.simulator.track import Track

CONE_HEIGHT_M = 0.1
"""Height of a detected cone point above the floor in metres, inside cone_filter's height limits."""
CONE_CONFIDENCE = 0.9
"""Confidence of every simulated detection, above cone_filter.min_confidence."""
MARKER_MAX_VIEW_ANGLE_DEG = 60.0
"""A marker seen more than this far off its face is not decoded. Not measured; ArUco decoding degrades at grazing
angles."""
MARKER_MIN_RANGE_M = 0.2
"""Closest a marker can be to the camera and still fit in its picture, metres. Not measured."""
MARKER_POSITION_NOISE_M = 0.01
"""Standard deviation of a decoded marker position in metres. Not measured."""
IMAGE_CORNERS = tuple(ImagePoint(u, v) for u, v in ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)))
"""Placeholder pixel corners: the policy only uses the decoded pose."""


def body_point(x: float, y: float, pose: tuple[float, float, float]) -> tuple[float, float]:
    """
    :return: The world point (x, y) in base_link at the pose: forwards and left in metres.
    """
    dx, dy = x - pose[0], y - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return c * dx + s * dy, -s * dx + c * dy


def yaw_quaternion(yaw: float) -> Quaternion:
    """
    :return: A rotation of yaw radians about the vertical axis.
    """
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _marker_quaternion(facing_in_body: float) -> Quaternion:
    """
    :return: The orientation of an upright marker whose face points along facing_in_body (radians, base_link), with
        the marker's z axis out of its face and y up, as an ArUco pose is given.
    """
    nx, ny = math.cos(facing_in_body), math.sin(facing_in_body)
    # Columns: marker x (along the face, to the right of someone looking at it), y (up), z (out of the face)
    rotation = np.array([[-ny, 0.0, nx], [nx, 0.0, ny], [0.0, 1.0, 0.0]])
    w = math.sqrt(max(0.0, 1.0 + np.trace(rotation))) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])) / 2.0,
                      rotation[2, 1] - rotation[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - rotation[0, 0] + rotation[1, 1] - rotation[2, 2])) / 2.0,
                      rotation[0, 2] - rotation[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - rotation[0, 0] - rotation[1, 1] + rotation[2, 2])) / 2.0,
                      rotation[1, 0] - rotation[0, 1])
    return Quaternion(x, y, z, w)


@dataclass
class SensorSettings:
    """How the simulated sensors behave. The defaults are the measured lab car."""
    camera_yaw_deg: float = 0.0
    """How far left of the car's forwards direction the camera really points, degrees."""
    camera_fov_deg: float = LAB_CAR.detector_fov_deg
    camera_min_range_m: float = LAB_CAR.detector_min_range_m
    camera_range_m: float = LAB_CAR.detector_range_m
    cone_noise_m: float = LAB_CAR.detector_noise_m
    cone_miss_probability: float = 0.0
    """Chance that one cone is missing from one camera frame."""
    noise: bool = True
    """False makes every sensor exact, for repeatable checks."""
    min_consecutive_frames: int = 3
    """Frames the aruco_detector must see a marker in before publishing it (config/aruco_detector.yaml)."""
    allowed_marker_ids: tuple[int, ...] = ()
    """IDs the aruco_detector publishes; empty publishes every ID (config/aruco_detector.yaml)."""


class Sensors:
    """Makes the sensor frames for one track. Keeps the aruco_detector's consecutive-frame counts between frames."""

    def __init__(self, track: Track, settings: SensorSettings, rng: np.random.Generator):
        self.track, self.settings, self.rng = track, settings, rng
        self.cones = [(cone.colour, 0.0, cone.x, cone.y) for cone in track.cones]
        self.cone_xy = np.array([(cone.x, cone.y) for cone in track.cones]).reshape(-1, 2)
        self.boxes = [obstacle.polygon() for obstacle in track.obstacles]
        self.marker_frames = [0] * len(track.markers)

    def _noise(self, scale: float, size: int = 2) -> np.ndarray:
        return self.rng.normal(0.0, scale, size) if self.settings.noise and scale > 0.0 else np.zeros(size)

    def _camera_turn(self, forward: float, left: float) -> tuple[float, float]:
        """A camera pointing left reports everything turned right."""
        yaw = -math.radians(self.settings.camera_yaw_deg)
        return math.cos(yaw) * forward - math.sin(yaw) * left, math.sin(yaw) * forward + math.cos(yaw) * left

    def _in_view(self, forward: float, left: float, min_range_m: float) -> bool:
        return (min_range_m <= forward and math.hypot(forward, left) <= self.settings.camera_range_m
                and abs(math.atan2(left, forward)) <= math.radians(self.settings.camera_fov_deg) / 2.0)

    def camera_cones(self, pose: tuple[float, float, float]) -> list[ConeDetection]:
        """
        :return: The cone detections of one camera frame taken at the pose, nearest first.
        """
        found = []
        for colour, _, x, y in visible_cones(self.cones, pose, self.boxes):
            if self.settings.noise and self.rng.random() < self.settings.cone_miss_probability:
                continue
            forward, left = np.add(body_point(x, y, pose), self._noise(self.settings.cone_noise_m))
            forward, left = self._camera_turn(forward, left)
            if forward >= self.settings.camera_min_range_m and self._in_view(forward, left, 0.0):
                found.append(ConeDetection(Position(float(forward), float(left), CONE_HEIGHT_M), colour,
                                           CONE_CONFIDENCE))
        return sorted(found, key=lambda cone: cone.pos.x)

    def markers_seen(self, pose: tuple[float, float, float]) -> list[bool]:
        """
        :return: For each marker, whether the camera can decode it from the pose (ignoring the frame count).
        """
        seen = []
        for marker in self.track.markers:
            forward, left = self._camera_turn(*body_point(marker.x, marker.y, pose))
            from_camera = forward - CAMERA_FORWARD_M
            # Direction from the marker to the camera, against the way its face points
            camera_x = pose[0] + CAMERA_FORWARD_M * math.cos(pose[2])
            camera_y = pose[1] + CAMERA_FORWARD_M * math.sin(pose[2])
            to_camera = math.atan2(camera_y - marker.y, camera_x - marker.x)
            off_face = abs(math.atan2(math.sin(to_camera - math.radians(marker.facing_deg)),
                                      math.cos(to_camera - math.radians(marker.facing_deg))))
            seen.append(self._in_view(from_camera, left, MARKER_MIN_RANGE_M)
                        and off_face <= math.radians(MARKER_MAX_VIEW_ANGLE_DEG))
        return seen

    def camera_markers(self, pose: tuple[float, float, float]) -> list[FiducialDetection]:
        """
        :return: The marker detections the aruco_detector publishes for one camera frame taken at the pose.
        """
        detections = []
        for index, (marker, seen) in enumerate(zip(self.track.markers, self.markers_seen(pose))):
            self.marker_frames[index] = self.marker_frames[index] + 1 if seen else 0
            allowed = self.settings.allowed_marker_ids
            if self.marker_frames[index] < self.settings.min_consecutive_frames or (allowed and marker.id not in allowed):
                continue
            forward, left = np.add(body_point(marker.x, marker.y, pose), self._noise(MARKER_POSITION_NOISE_M))
            forward, left = self._camera_turn(forward, left)
            facing = math.radians(marker.facing_deg) - pose[2] - math.radians(self.settings.camera_yaw_deg)
            detections.append(FiducialDetection(
                marker.id, marker.size_m, IMAGE_CORNERS,
                FiducialPose(Position(float(forward), float(left), marker.height_m), _marker_quaternion(facing)), 0.5))
        return detections

    def lidar(self, pose: tuple[float, float, float]) -> list[LidarCartesianPoint]:
        """
        :return: The returns of one lidar scan from the pose, in base_link.
        """
        return lidar_points(pose, self.cone_xy, self.boxes, self.rng if self.settings.noise else _NoNoise())

    def yaw_rate(self, speed_m_per_s: float, curvature_per_m: float) -> float:
        """
        :return: The gyro's yaw rate in rad/s, positive turning left.
        """
        return speed_m_per_s * curvature_per_m + float(self._noise(0.005, 1)[0])


class _NoNoise:
    """Stands in for a random generator that adds no lidar range noise."""

    @staticmethod
    def normal(mean, scale, size):
        return np.zeros(size)


def cone_contacts(cone_xy: np.ndarray, pose: tuple[float, float, float], front_m: float, rear_m: float,
                  width_m: float) -> set[int]:
    """
    :param cone_xy: World (x, y) of every cone, as an Nx2 array.
    :param pose: The car's world x, y and heading.
    :return: The indices of the cones the car body touches (closer than a cone's base radius).
    """
    if not len(cone_xy):
        return set()
    return set(np.flatnonzero(cone_gaps(cone_xy, *pose, front_m, rear_m, width_m) < CONE_RADIUS_M).tolist())
