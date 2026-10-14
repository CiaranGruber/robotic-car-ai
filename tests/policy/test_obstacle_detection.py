"""Hardware-free checks of the lidar obstacle observation, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_obstacle_detection.py
"""
import dataclasses
import json
import math

import pytest

from policy.action_policy.mpc import MpcConfig
from policy.cone_filter.cone_filter import ConeFilterConfig
from policy.control.control import ControlConfig
from policy.input_output import (CarObservations, LidarCartesian, LidarCartesianPoint, ObservedCarState, PolicyState,
                                 Position, SensorAge)
from policy.lane_detection.lane_detection import LaneDetectionConfig, detect_lanes
from policy.obstacle_detection.obstacle_detection import (ObstacleDetectionConfig, detect_lane_obstacles,
                                                          detect_obstacles, nearest_in_band)
from policy.obstacle_detection.obstacles import LaneObstacle, ObstacleDetection
from policy.policy_runner import MovementPolicy
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


def lane_view(points, **lane):
    """Observations of a lab-width (0.8 m) lane from lane_cones with lidar returns at the points, and its lane."""
    observations = dataclasses.replace(observe(lane_cones(width_m=0.8, **lane)), lidar_cartesian=lidar(points))
    return observations, detect_lanes(observations, LaneDetectionConfig())


def test_object_in_the_lane_is_found_in_lane_coordinates():
    observations, lane = lane_view(box(AHEAD_M, -0.1, width=0.2))
    (obstacle,) = detect_lane_obstacles(observations, lane, CONFIG)
    assert obstacle.distance_m == pytest.approx(1.0, abs=0.02)
    assert (obstacle.right_m, obstacle.left_m) == (pytest.approx(-0.2, abs=0.02), pytest.approx(0.0, abs=0.02))
    assert obstacle.point_count == 21


def test_lane_cones_and_objects_beside_the_lane_are_not_obstacles():
    cones = [(x, side * 0.4 + dy) for x in (0.5, 1.0, 1.5, 2.0) for side in (1, -1) for dy in (-0.03, 0, 0.03)]
    observations, lane = lane_view(cones + box(AHEAD_M, -0.75, width=0.3))
    assert detect_lane_obstacles(observations, lane, CONFIG) == []


def test_objects_are_separated_and_reported_nearest_first():
    observations, lane = lane_view(box(CONFIG.car_front_m + 1.2, 0.15, width=0.1)
                                   + box(CONFIG.car_front_m + 0.6, -0.15, width=0.1))
    near, far = detect_lane_obstacles(observations, lane, CONFIG)
    assert (near.distance_m, far.distance_m) == (pytest.approx(0.6, abs=0.02), pytest.approx(1.2, abs=0.02))
    assert near.left_m < 0.0 < far.right_m


def test_object_split_by_the_scan_start_is_one_object():
    """With the lidar turned around, the scan starts and ends straight ahead."""
    points = box(AHEAD_M, 0.0, width=0.2)
    indices = list(range(400, 410)) + list(range(len(points) - 10))
    split = LidarCartesian([LidarCartesianPoint(Position(x, y, 0.12), index) for index, (x, y) in zip(indices, points)],
                           SensorAge(0.05, 7), "base_link")
    observations, lane = lane_view([])
    observations = dataclasses.replace(observations, lidar_cartesian=split)
    (obstacle,) = detect_lane_obstacles(observations, lane, CONFIG)
    assert obstacle.point_count == len(points)


def test_object_on_a_curved_lane_is_found_along_it():
    observations, lane = lane_view([], radius_m=2.0)
    centre_line = lane.centre_line
    points = [centre_line.offset(left).point_at(CONFIG.car_front_m + 1.0) for left in (-0.05, -0.025, 0.0, 0.025, 0.05)]
    observations = dataclasses.replace(observations, lidar_cartesian=lidar(points))
    (obstacle,) = detect_lane_obstacles(observations, lane, CONFIG)
    assert obstacle.distance_m == pytest.approx(1.0, abs=0.05)
    assert (obstacle.right_m, obstacle.left_m) == (pytest.approx(-0.05, abs=0.01), pytest.approx(0.05, abs=0.01))


def test_stray_returns_and_missing_data_give_no_lane_obstacles():
    observations, lane = lane_view([(AHEAD_M, 0.0), (AHEAD_M + 0.5, 0.1)])
    assert detect_lane_obstacles(observations, lane, CONFIG) == []
    assert detect_lane_obstacles(dataclasses.replace(observations, lidar_cartesian=None), lane, CONFIG) is None
    assert detect_lane_obstacles(observations, None, CONFIG) is None


def test_nearest_in_band_only_counts_objects_reaching_into_it():
    right = LaneObstacle(0.5, 0.6, -0.3, -0.2, 5)
    centre = LaneObstacle(1.0, 1.1, -0.05, 0.05, 5)
    assert nearest_in_band([right, centre], -0.2, 0.2) == 0.5
    assert nearest_in_band([right, centre], -0.15, 0.2) == 1.0
    assert nearest_in_band([right, centre], 0.1, 0.3) == math.inf


@pytest.mark.parametrize("points", [[], box(AHEAD_M, 0.0)], ids=["clear", "box"])
def test_obstacle_beyond_the_stopping_distance_does_not_change_the_drive(points):
    """1 m ahead is beyond the default stopping distance: (0.6 s + 0.05 s) * 0.5 m/s + 0.2 m = 0.53 m."""
    without_lidar = movement_policy()
    expected = without_lidar.step(observe(lane_cones()))
    policy = movement_policy()
    actions = policy.step(dataclasses.replace(observe(lane_cones()), lidar_cartesian=lidar(points)))
    assert (actions.drive_action, actions.steering_action) == (expected.drive_action, expected.steering_action)
    assert without_lidar.obstacles is None and without_lidar.obstacle_distance_m is None
    assert policy.obstacles.path_clear == (not points)
    assert policy.obstacle_distance_m == (math.inf if not points else pytest.approx(1.0, abs=0.02))


def drives(policy, points, first):
    """Whether the movement policy drives the car in a straight 1 m lane with lidar returns at the points."""
    observations = dataclasses.replace(observe(lane_cones(), first=first), lidar_cartesian=lidar(points))
    return policy.step(observations).drive_action > 0.0


def test_obstacle_in_the_lane_stops_the_car_until_a_new_run():
    policy = movement_policy()
    near_box = box(CONFIG.car_front_m + 0.4, 0.0, width=0.2)
    assert [drives(policy, near_box, first=step == 0) for step in range(3)] == [True, False, False]
    assert policy.action_policy.stopped_for_obstacle
    assert not drives(policy, [], first=False)
    assert drives(policy, [], first=True)


@pytest.mark.parametrize("points", [
    box(CONFIG.car_front_m + 0.4, -0.75, width=0.3),  # Beside the 1 m lane
    cone_rows(),  # The lane's own cones
], ids=["beside-lane", "lane-cones"])
def test_objects_outside_the_cars_way_do_not_stop_it(points):
    policy = movement_policy()
    assert all(drives(policy, points, first=step == 0) for step in range(3))


def test_without_a_lane_the_straight_corridor_is_checked():
    policy = movement_policy()
    policy.step(dataclasses.replace(observe([]), lidar_cartesian=lidar(box(AHEAD_M, 0.0))))
    assert policy.lane_obstacles is None
    assert policy.obstacle_distance_m == pytest.approx(1.0, abs=0.02)


def test_stopping_distance_must_fit_within_the_searched_distance():
    # 3.0 s * 0.5 m/s + 0.2 m = 1.7 m > 1.5 m
    with pytest.raises(ValueError):
        MovementPolicy(MpcConfig(obstacle_stop_distance_per_speed_s=3.0), ConeFilterConfig(), ControlConfig())
    MovementPolicy(MpcConfig(obstacle_stop_distance_per_speed_s=3.0, obstacle_stop_enabled=False),
                   ConeFilterConfig(), ControlConfig())


def test_a_short_obstacle_stopping_distance_needs_braking():
    """Without braking the car coasts as far for an obstacle as for any other stop."""
    short = MpcConfig(stop_distance_per_speed_s=2.1, obstacle_stop_distance_per_speed_s=0.6)
    with pytest.raises(ValueError):
        MovementPolicy(short, ConeFilterConfig(), ControlConfig(brake_action=0.0))
    MovementPolicy(short, ConeFilterConfig(), ControlConfig(brake_action=0.3))
    MovementPolicy(dataclasses.replace(short, obstacle_stop_enabled=False), ConeFilterConfig(),
                   ControlConfig(brake_action=0.0))


def test_obstacle_in_the_lane_brakes_the_car_straight_from_driving():
    policy = MovementPolicy(MpcConfig(), ConeFilterConfig(), ControlConfig(brake_action=0.3))
    near_box = box(CONFIG.car_front_m + 0.4, 0.0, width=0.2)
    drive = [policy.step(dataclasses.replace(observe(lane_cones(), first=step == 0), lidar_cartesian=lidar(near_box)))
             .drive_action for step in range(2)]
    assert drive[0] > 0.0
    assert drive[1] == -0.3


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
    {"lane_edge_margin_m": -0.1},
    {"lane_edge_margin_m": math.nan},
    {"cluster_gap_m": 0.0},
])
def test_invalid_settings_are_rejected(change):
    with pytest.raises(ValueError):
        ObstacleDetectionConfig(**change)
