"""Hardware-free checks of the lidar obstacle observation, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_obstacle_detection.py
"""
import dataclasses
import json
import math

import pytest

from policy.input_output import (CarObservations, LidarCartesian, LidarCartesianPoint, ObservedCarState, PolicyState,
                                 Position, SensorAge)
from policy.obstacle_detection.obstacle_detection import ObstacleDetectionConfig, detect_obstacles
from policy.obstacle_detection.obstacles import ObstacleDetection
from tests.policy.test_lane_detection import lane_cones
from tests.policy.test_lane_following import movement_policy, observe

CONFIG = ObstacleDetectionConfig()
AHEAD_M = CONFIG.car_front_m + 1.0
"""Forwards position of an object 1 m ahead of the car's front."""


def lidar(points):
    """Fresh body-frame lidar returns at the (x, y) points, at the default lidar height."""
    return LidarCartesian([LidarCartesianPoint(Position(x, y, 0.12), index) for index, (x, y) in enumerate(points)],
                          SensorAge(0.05, 7), "base_link")


def observe_lidar(points):
    """Observations holding only lidar returns, or no lidar data when points is None."""
    return CarObservations(None, None, None, None if points is None else lidar(points),
                           ObservedCarState(None, None, None, None), PolicyState(0.1, 1.0, False))


def box(x, y, width=0.45):
    """Returns 1 cm apart from the near face of a box across the path at x, centred at y."""
    count = round(width / 0.01) + 1
    return [(x, y - width / 2.0 + i * 0.01) for i in range(count)]


def cone_rows():
    """Returns from the cones of a straight 1 m lane centred on the car, three per cone."""
    return [(x, side * 0.5 + dy) for x in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0) for side in (1, -1) for dy in (-0.03, 0, 0.03)]


def arc_points(along_m, curvature_per_m):
    """Returns across a path of the curvature from base_link's origin, along_m along it, up to 0.1 m either side."""
    radius = 1.0 / curvature_per_m
    side = math.copysign(1.0, curvature_per_m)
    turned = along_m / abs(radius)
    distances = (abs(radius) - side * left for left in (-0.1, -0.05, 0.0, 0.05, 0.1))
    return [(r * math.sin(turned), radius - side * r * math.cos(turned)) for r in distances]


def test_no_cartesian_lidar_gives_no_observation():
    assert detect_obstacles(observe_lidar(None), CONFIG) is None


def test_scan_without_returns_is_clear_and_reads_max_range():
    obstacles = detect_obstacles(observe_lidar([]), CONFIG)
    assert obstacles.path_clear and obstacles.in_path_point is None and obstacles.in_path_point_count == 0
    assert obstacles.forward_ranges_m == [CONFIG.max_range_m] * CONFIG.beam_count


def test_forward_fan_matches_the_notebook_beam_layout():
    def at(bearing_deg, distance):
        return distance * math.cos(math.radians(bearing_deg)), distance * math.sin(math.radians(bearing_deg))

    points = [at(-90, 1.0), at(0, 2.0), at(90, 1.5), at(31, 3.0), at(-45, 0.03), at(45, 5.0), at(180, 1.0)]
    ranges = detect_obstacles(observe_lidar(points), CONFIG).forward_ranges_m
    expected = {0: 1.0,  # Index 0 points right
                30: 2.0,  # The middle index points straight ahead
                60: 1.5,  # The last index points left
                40: 3.0,  # A return between beams goes to the nearest beam (30 degrees)
                15: CONFIG.min_range_m}  # Closer than min_range_m
    # Beyond max_range_m (45 degrees) and behind the car are no return
    assert ranges == pytest.approx([expected.get(beam, CONFIG.max_range_m) for beam in range(CONFIG.beam_count)])


def test_cones_beside_the_lane_set_the_nearest_range_but_are_not_in_the_path():
    obstacles = detect_obstacles(observe_lidar(cone_rows()), CONFIG)
    assert obstacles.path_clear and obstacles.in_path_point_count == 0
    # A rule on the nearest range overall, as in the notebook's first exercise, would brake for the cones
    assert min(obstacles.forward_ranges_m) < 1.0


def test_box_in_the_lane_gives_its_distance_from_the_car_front():
    obstacles = detect_obstacles(observe_lidar(cone_rows() + box(AHEAD_M, 0.0)), CONFIG)
    assert not obstacles.path_clear
    assert obstacles.in_path_distance_m == pytest.approx(1.0)
    assert obstacles.in_path_point.x == pytest.approx(AHEAD_M)
    assert abs(obstacles.in_path_point.y) <= CONFIG.corridor_half_width_m
    assert obstacles.sensor_age.stamp_ns == 7


def test_stray_returns_closer_than_the_box_do_not_count():
    strays = [(CONFIG.car_front_m + 0.2, 0.0), (CONFIG.car_front_m + 0.4, 0.1)]
    assert detect_obstacles(observe_lidar(strays), CONFIG).path_clear
    obstacles = detect_obstacles(observe_lidar(strays + box(AHEAD_M, 0.0)), CONFIG)
    assert obstacles.in_path_distance_m == pytest.approx(1.0)


@pytest.mark.parametrize("points", [
    box(CONFIG.car_front_m - 0.1, 0.0),  # Beside the car, not ahead of its front
    box(CONFIG.car_front_m + CONFIG.corridor_length_m + 0.1, 0.0),  # Beyond the corridor
    box(AHEAD_M, 0.45, width=0.2),  # Beside the path
], ids=["beside-car", "beyond-corridor", "beside-path"])
def test_objects_outside_the_corridor_are_not_in_the_path(points):
    assert detect_obstacles(observe_lidar(points), CONFIG).path_clear


@pytest.mark.parametrize("curvature_per_m", [0.5, -0.5])
def test_corridor_follows_the_steering_curvature(curvature_per_m):
    on_curve = arc_points(AHEAD_M, curvature_per_m)
    turning = detect_obstacles(observe_lidar(on_curve), CONFIG, curvature_per_m)
    assert turning.in_path_distance_m == pytest.approx(1.0)
    assert turning.path_curvature_per_m == curvature_per_m
    # Steering away from a box straight ahead does not drive into it, and a straight corridor misses the curve's box
    assert detect_obstacles(observe_lidar(box(AHEAD_M, 0.0, width=0.2)), CONFIG, curvature_per_m).path_clear
    assert detect_obstacles(observe_lidar(on_curve), CONFIG).path_clear


@pytest.mark.parametrize("points, path_clear", [([], True), (box(AHEAD_M, 0.0), False)], ids=["clear", "box"])
def test_movement_policy_observes_obstacles_without_changing_the_drive(points, path_clear):
    without_lidar = movement_policy()
    expected = without_lidar.step(observe(lane_cones()))
    policy = movement_policy()
    actions = policy.step(dataclasses.replace(observe(lane_cones()), lidar_cartesian=lidar(points)))
    assert (actions.drive_action, actions.steering_action) == (expected.drive_action, expected.steering_action)
    assert without_lidar.obstacles is None
    assert policy.obstacles.path_clear == path_clear


@pytest.mark.parametrize("points", [[], box(AHEAD_M, 0.0)], ids=["clear", "box"])
def test_observation_survives_json_for_logs(points):
    obstacles = detect_obstacles(observe_lidar(points), CONFIG)
    assert ObstacleDetection.deserialise(json.loads(json.dumps(obstacles.serialise()))) == obstacles


@pytest.mark.parametrize("change", [
    {"beam_count": 1},
    {"beam_count": 61.0},
    {"field_of_view_deg": 0.0},
    {"min_range_m": 4.0},
    {"car_front_m": -0.1},
    {"corridor_half_width_m": 0.0},
    {"min_points": 0},
])
def test_invalid_settings_are_rejected(change):
    with pytest.raises(ValueError):
        ObstacleDetectionConfig(**change)
