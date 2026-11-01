"""Marker stands left out of obstacle detection, and avoidance waiting for confirmed obstacles."""
from dataclasses import replace

import pytest

from policy.action_policy.avoidance import AvoidanceConfig
from policy.markers.commands import MarkerCommand
from policy.memory import CarMemory, CarPose, IntersectionTurn, TurnPhase
from policy.obstacle_detection.obstacle_detection import ObstacleDetectionConfig
from scripts.mcap_export import DEFAULT_SETTINGS_PATH, build_policy, read_settings
from tests.policy.test_markers import marker, straight_lane
from tests.policy.test_obstacle_detection import lidar

STAND = (1.02, 0.5)
"""A stand's return just behind marker(…, x=1.0, y=0.5)."""
BOX = (1.0, 0.0)
"""A real object in the lane."""


def policy_and_memory():
    memory = CarMemory()
    memory.start_map(None)
    memory.move_car(CarPose(0.0, 0.0, 0.0), None)
    return build_policy(read_settings(DEFAULT_SETTINGS_PATH)), memory


def kept_points(observations):
    return [(round(p.pos.x, 3), round(p.pos.y, 3)) for p in observations.lidar_cartesian.points]


def test_stand_returns_beside_a_seen_marker_are_left_out_but_other_objects_kept():
    policy, memory = policy_and_memory()
    observations = replace(straight_lane(markers=[marker(1, 1.0, 0.5)]), lidar_cartesian=lidar([STAND, BOX]))
    assert kept_points(policy._without_marker_stands(observations, memory)) == [BOX]


def test_a_stand_stays_left_out_once_its_marker_is_out_of_view():
    policy, memory = policy_and_memory()
    policy._without_marker_stands(replace(straight_lane(markers=[marker(1, 1.0, 0.5)]), lidar_cartesian=lidar([])),
                                  memory)
    memory.move_car(CarPose(0.5, 0.0, 0.0), None)
    moved = replace(straight_lane(markers=None), lidar_cartesian=lidar([(STAND[0] - 0.5, STAND[1]), BOX]))
    assert kept_points(policy._without_marker_stands(moved, memory)) == [BOX]


def test_zero_clearance_keeps_every_return():
    settings = read_settings(DEFAULT_SETTINGS_PATH)
    settings["obstacle_detection.marker_clearance_m"] = 0.0
    policy, memory = build_policy(settings), CarMemory()
    observations = replace(straight_lane(markers=[marker(1, 1.0, 0.5)]), lidar_cartesian=lidar([STAND, BOX]))
    assert kept_points(policy._without_marker_stands(observations, memory)) == [STAND, BOX]


def test_avoidance_waits_for_three_sightings_in_five_steps():
    policy, memory = policy_and_memory()
    seen = ["obstacle"]
    given = []
    for sighting in (seen, [], seen, seen):
        policy.lane_obstacles = sighting
        given.append(policy._confirmed_lane_obstacles(memory))
    assert given == [[], [], [], seen]


def test_no_new_avoidance_line_while_a_turn_is_armed_but_a_planned_line_is_kept():
    policy, memory = policy_and_memory()
    memory.plan_turn(IntersectionTurn(MarkerCommand.TURN_LEFT, TurnPhase.APPROACH, 2.0, 0.0, 0.0, 1.0, "CENTRE"))
    policy.lane_obstacles = ["obstacle"]
    assert [policy._confirmed_lane_obstacles(memory) for _ in range(5)] == [[]] * 5
    policy.action_policy.avoidance_path = object()
    assert policy._confirmed_lane_obstacles(memory) == ["obstacle"]


@pytest.mark.parametrize("changes", [{"confirm_steps": 0}, {"confirm_steps": 6}, {"confirm_window_steps": 0},
                                     {"confirm_steps": 1.5}, {"hold_at_intersections": 1}])
def test_invalid_avoidance_confirmation_cannot_start(changes):
    with pytest.raises(ValueError):
        AvoidanceConfig(**changes)


def test_negative_marker_clearance_cannot_start():
    with pytest.raises(ValueError, match="marker_clearance_m"):
        ObstacleDetectionConfig(marker_clearance_m=-0.1)
