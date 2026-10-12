"""Hardware-free checks of the RC-assisted open-space policy, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_open_space.py
"""
import dataclasses
import itertools
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from policy.action_policy.mpc import MpcConfig  # noqa: E402
from policy.action_policy.open_space import OpenSpaceConfig, OpenSpacePolicy, OpenSpaceStatus  # noqa: E402
from policy.cone_filter.cone_filter import ConeFilterConfig, remove_implausible_cones  # noqa: E402
from policy.control.actions import DriveCommand  # noqa: E402
from policy.control.control import ControlConfig  # noqa: E402
from policy.input_output import (  # noqa: E402
    CarObservations, ConeBatch, ConeColour, ConeDetection, ObservedCarState, PolicyState, Position, RcInput,
    SensorAge, WheelSpeed)
from policy.policy_runner import MovementPolicy  # noqa: E402

FORWARDS = (0.6, 0.0)
"""RC drive held forwards with centred steering."""
BATCH_STAMPS = itertools.count(1)
"""Gives every cone batch its own ROS stamp, so each is a new batch for the cone memory."""


def cone(x, y, colour=ConeColour.YELLOW, confidence=0.9):
    return ConeDetection(Position(x, y, 0.1), colour, confidence)


def observe(cones=(), rc=FORWARDS, first_step=False, wheel_speed=0.5, dt=0.05, stamp_ns=None):
    """Observations with a new cone batch (None for no cone data) and the RC (drive, steer), or None for no RC."""
    stamp_ns = next(BATCH_STAMPS) if stamp_ns is None else stamp_ns
    return CarObservations(
        cones=None if cones is None else ConeBatch(list(cones), SensorAge(0.05, stamp_ns),
                                                   acquisition_to_publish_latency_s=0.03),
        fiducials=None,
        lidar_scan=None,
        lidar_cartesian=None,
        car=ObservedCarState(None, None, None, WheelSpeed(wheel_speed, SensorAge(0.0, None))),
        policy=PolicyState(dt=0.0 if first_step else dt, policy_elapsed_s=0.0, is_first_policy_step=first_step),
        rc=None if rc is None else RcInput(rc[0], rc[1], SensorAge(0.02, None)),
    )


def run(observations, policy=None, turns_left=True):
    policy = policy or OpenSpacePolicy(OpenSpaceConfig(), turns_left)
    [command] = policy.run_policy(observations)
    assert isinstance(command, DriveCommand)
    return command, policy


@pytest.mark.parametrize("rc", [None, (0.0, 0.0), (0.2, 0.0), (-0.5, 0.0)], ids=["no-rc", "neutral", "light", "back"])
def test_stops_unless_the_rc_drive_is_held_forwards(rc):
    command, policy = run(observe(rc=rc))
    assert command == DriveCommand(0.0, 0.0)
    assert policy.status == OpenSpaceStatus.WAITING_FOR_RC


def test_drives_straight_ahead_with_no_cones_in_view():
    command, policy = run(observe(cones=[]))
    assert command.speed_m_per_s == OpenSpaceConfig().speed_m_per_s
    assert command.curvature_per_m == pytest.approx(0.0, abs=1e-9)
    assert policy.status == OpenSpaceStatus.DRIVING


def test_stops_without_fresh_cone_data_because_the_space_ahead_is_unknown():
    command, policy = run(observe(cones=None))
    assert command == DriveCommand(0.0, 0.0)
    assert policy.status == OpenSpaceStatus.NO_CONE_DATA
    assert policy.heading_rad is None


def test_drives_straight_through_a_gap_that_is_wide_enough():
    command, _ = run(observe(cones=[cone(1.0, 0.5), cone(1.0, -0.5)]))
    assert command.speed_m_per_s > 0.0
    assert command.curvature_per_m == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("cone_y, turns_left", [(0.1, False), (-0.1, True)])
def test_turns_away_from_a_cone_ahead_to_the_more_open_side(cone_y, turns_left):
    command, policy = run(observe(cones=[cone(1.0, cone_y)]))
    assert command.speed_m_per_s > 0.0
    assert (command.curvature_per_m > 0.0) == turns_left
    assert policy.clearance_m([Position(1.0, cone_y, 0.1)], policy.heading_rad) >= OpenSpaceConfig().clearance_m


@pytest.mark.parametrize("positive_steering_turns_left", [True, False])
@pytest.mark.parametrize("rc_steer", [0.8, -0.8])
def test_rc_steering_chooses_the_side_to_pass_a_cone_straight_ahead(rc_steer, positive_steering_turns_left):
    command, _ = run(observe(cones=[cone(1.0, 0.0)], rc=(0.6, rc_steer)), turns_left=positive_steering_turns_left)
    rc_asks_for_left = (rc_steer > 0.0) == positive_steering_turns_left
    assert (command.curvature_per_m > 0.0) == rc_asks_for_left


def test_rc_steering_turns_towards_its_side_when_nothing_blocks_it():
    config = OpenSpaceConfig()
    command, policy = run(observe(cones=[], rc=(0.6, 1.0)))
    assert policy.heading_rad == pytest.approx(config.max_preferred_heading_rad, abs=config.max_heading_rad / 28)
    assert command.curvature_per_m > 0.0


def test_stops_when_no_heading_is_open():
    wall = [cone(1.0, y / 10.0) for y in range(-15, 16, 3)]
    command, policy = run(observe(cones=wall))
    assert command == DriveCommand(0.0, 0.0)
    assert policy.status == OpenSpaceStatus.BLOCKED
    assert policy.heading_rad is None


def test_keeps_its_side_when_the_cone_ahead_shifts_slightly():
    policy = OpenSpacePolicy(OpenSpaceConfig(), True)
    first, _ = run(observe(cones=[cone(1.0, 0.0)], first_step=True), policy)
    side = math.copysign(1.0, first.curvature_per_m)
    # The cone moves slightly towards the chosen side, which alone would make the other side a little better
    second, _ = run(observe(cones=[cone(1.0, 0.02 * side)]), policy)
    assert math.copysign(1.0, second.curvature_per_m) == side


def test_heading_is_still_chosen_while_waiting_so_it_can_be_checked_by_hand():
    command, policy = run(observe(cones=[cone(1.0, 0.1)], rc=(0.0, 0.0)))
    assert command == DriveCommand(0.0, 0.0)
    assert policy.heading_rad < 0.0


def test_cones_behind_or_beyond_the_corridor_are_ignored():
    policy = OpenSpacePolicy(OpenSpaceConfig(), True)
    assert policy.clearance_m([Position(-0.5, 0.0, 0.1), Position(2.0, 0.0, 0.1)], 0.0) == math.inf
    assert policy.clearance_m([Position(1.0, -0.2, 0.1)], 0.0) == pytest.approx(0.2)


def test_clearance_is_measured_from_the_arc_the_car_would_drive():
    policy = OpenSpacePolicy(OpenSpaceConfig(), True)
    heading = 0.5
    radius = 1.0 / policy.curvature_per_m(heading)
    # A cone on the arc a quarter turn round, and one 0.3 m outside the arc an eighth of a turn round
    on_arc = Position(radius * math.sin(0.5), radius * (1.0 - math.cos(0.5)), 0.1)
    outside = Position((radius + 0.3) * math.sin(0.3), radius - (radius + 0.3) * math.cos(0.3), 0.1)
    assert policy.clearance_m([on_arc], heading) == pytest.approx(0.0, abs=1e-9)
    assert policy.clearance_m([outside], heading) == pytest.approx(0.3)
    assert policy.clearance_m([on_arc], -heading) > 0.2


def test_a_cone_that_leaves_the_camera_view_is_remembered_and_moved_with_the_car():
    policy = OpenSpacePolicy(OpenSpaceConfig(), True)
    run(observe(cones=[cone(0.6, 0.3)], rc=(0.0, 0.0), first_step=True), policy)
    # The camera loses the cone; the car coasts 0.1 m straight in 0.2 s at 0.5 m/s wheel speed
    for _ in range(4):
        run(observe(cones=[], rc=(0.0, 0.0), wheel_speed=0.5, dt=0.05), policy)
    [(position, age)] = policy.remembered_cones
    assert (position.x, position.y) == pytest.approx((0.5, 0.3))
    assert age == pytest.approx(0.2)


def test_a_reused_batch_is_remembered_once_and_old_cones_are_forgotten():
    config = OpenSpaceConfig(cone_memory_s=0.3)
    policy = OpenSpacePolicy(config, True)
    run(observe(cones=[cone(1.0, 0.5)], first_step=True, stamp_ns=7), policy)
    run(observe(cones=[cone(1.0, 0.5)], wheel_speed=0.0, stamp_ns=7), policy)
    assert len(policy.remembered_cones) == 1
    # A new batch replaces the remembered detection of the same cone
    run(observe(cones=[cone(1.05, 0.5)], wheel_speed=0.0), policy)
    assert [position.x for position, _ in policy.remembered_cones] == [1.05]
    for _ in range(6):
        run(observe(cones=[], wheel_speed=0.0), policy)
    assert policy.remembered_cones == []


@pytest.mark.parametrize("memory_s", [0.0, 0.03])
def test_without_memory_only_the_latest_batch_counts_even_while_timer_mode_reuses_it(memory_s):
    policy = OpenSpacePolicy(OpenSpaceConfig(cone_memory_s=memory_s), True)
    run(observe(cones=[cone(1.0, 0.0)], first_step=True, stamp_ns=3), policy)
    reused, _ = run(observe(cones=[cone(1.0, 0.0)], stamp_ns=3), policy)
    assert reused.curvature_per_m != pytest.approx(0.0, abs=1e-6)
    command, _ = run(observe(cones=[]), policy)
    assert command.curvature_per_m == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("change", [
    {"heading_count": 2}, {"heading_count": 29.0}, {"speed_m_per_s": 0.0}, {"rc_drive_threshold": 0.0},
    {"rc_steer_deadband": 1.0}, {"max_heading_rad": math.pi / 2.0}, {"max_preferred_heading_rad": 0.8},
    {"clearance_cap_m": 0.3}, {"heading_change_weight_m_per_rad": -0.1}, {"look_ahead_m": math.inf},
    {"cone_memory_s": -0.1}, {"cone_memory_s": math.inf}, {"memory_merge_distance_m": 0.0},
])
def test_invalid_settings_are_rejected(change):
    with pytest.raises(ValueError):
        dataclasses.replace(OpenSpaceConfig(), **change)


def test_obstacle_cones_are_kept_off_any_lane_row_but_implausible_ones_are_removed():
    kept_cone = cone(1.0, 0.3)
    batch = ConeBatch([kept_cone, cone(1.02, 0.3, confidence=0.6), cone(-0.3, 0.0), cone(1.0, 0.0, confidence=0.2)],
                      SensorAge(0.05, 1), acquisition_to_publish_latency_s=0.03)
    assert list(remove_implausible_cones(batch, ConeFilterConfig())) == [kept_cone]
    assert remove_implausible_cones(None, ConeFilterConfig()) is None


def test_movement_policy_open_space_drives_round_a_cone_within_the_effort_limit():
    """The whole chain: plausible cones, open-space policy, then control."""
    control_config = ControlConfig()
    movement = MovementPolicy(MpcConfig(), ConeFilterConfig(), control_config, OpenSpaceConfig(), "open_space")
    actions = movement.step(observe(cones=[cone(1.0, 0.1, ConeColour.BLUE)], first_step=True, wheel_speed=0.3))
    assert 0.0 < actions.drive_action <= control_config.max_drive_effort
    assert actions.steering_action < 0.0  # right, away from the cone, with positive steering turning left
    assert actions.debug1 < 0.0  # the chosen heading in degrees
    assert actions.debug2 == float(OpenSpaceStatus.DRIVING)


def test_movement_policy_open_space_stops_when_the_rc_drive_is_released():
    movement = MovementPolicy(MpcConfig(), ConeFilterConfig(), ControlConfig(), OpenSpaceConfig(), "open_space")
    actions = movement.step(observe(cones=[], rc=(0.0, 0.0), first_step=True))
    assert actions.drive_action == 0.0 and actions.steering_action == 0.0
    assert actions.debug2 == float(OpenSpaceStatus.WAITING_FOR_RC)


def test_unknown_action_policy_is_rejected():
    with pytest.raises(ValueError):
        MovementPolicy(MpcConfig(), ConeFilterConfig(), ControlConfig(), OpenSpaceConfig(), "follow_the_gap")
