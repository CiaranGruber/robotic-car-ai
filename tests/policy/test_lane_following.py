"""Hardware-free checks of the whole movement policy, from cones to drive and steering actions, without ROS.

The cone filter, lane detection, MPC and car 20's control run together as in MovementPolicy.step on the car.

Run from the repository root with: python3 -m pytest tests/policy/test_lane_following.py
"""
import dataclasses

import pytest

from policy.action_policy.mpc import MpcConfig
from policy.cone_filter.cone_filter import ConeFilterConfig
from policy.control.control import ControlConfig
from policy.input_output import CarObservations, ConeBatch, ObservedCarState, PolicyState, SensorAge, WheelSpeed
from policy.lane_detection.lane_detection import LaneDetectionConfig
from policy.policy_runner import MovementPolicy
from scenarios.scenario_wrapper import Scenario, ScenarioType
from tests.policy.test_lane_detection import lane_cones

CONTROL = ControlConfig()


def movement_policy():
    """The movement policy with the lane-following test run's settings."""
    return MovementPolicy(MpcConfig(target_speed_m_per_s=0.5, max_curvature_per_m=0.5), ConeFilterConfig(),
                          LaneDetectionConfig(), CONTROL)


def observe(cones, wheel_speed=0.5, first=True):
    """Observations holding a cone batch, the wheel speed and the policy timing."""
    return CarObservations(
        cones=None if cones is None else ConeBatch(cones, SensorAge(0.05, 1), 0.03),
        fiducials=None, lidar_scan=None, lidar_cartesian=None,
        car=ObservedCarState(None, None, None, WheelSpeed(wheel_speed, SensorAge(0.01, None))),
        policy=PolicyState(0.0 if first else 0.1, 0.0, first))


def test_centred_car_drives_straight_at_the_target_speed():
    actions = movement_policy().step(observe(lane_cones()))
    assert actions.drive_action == pytest.approx(CONTROL.speed_ff_offset + 0.5 / CONTROL.speed_ff_gain)
    assert actions.steering_action == pytest.approx(CONTROL.steering_centre_action, abs=1e-6)


@pytest.mark.parametrize("offset_m, heading_deg", [(0.15, 0.0), (0.0, 10.0)])
def test_car_left_of_the_lane_steers_right_and_right_of_it_steers_left(offset_m, heading_deg):
    # On car 20, a steering action above the centre turns right
    right = movement_policy().step(observe(lane_cones(offset_m=offset_m, heading_deg=heading_deg)))
    left = movement_policy().step(observe(lane_cones(offset_m=-offset_m, heading_deg=-heading_deg)))
    assert right.steering_action > CONTROL.steering_centre_action > left.steering_action
    assert right.drive_action > 0.0 and left.drive_action > 0.0


@pytest.mark.parametrize("cones", [None, []], ids=["no-data", "no-cones"])
def test_no_lane_stops_the_car(cones):
    actions = movement_policy().step(observe(cones))
    assert actions.drive_action == 0.0
    assert actions.steering_action == pytest.approx(CONTROL.steering_centre_action)


def test_missing_wheel_speed_stops_the_car():
    observations = dataclasses.replace(observe(lane_cones()), car=ObservedCarState(None, None, None, None))
    assert movement_policy().step(observations).drive_action == 0.0


def test_recorded_run_drives_while_the_lane_is_seen_and_stops_after_it():
    policy = movement_policy()
    drives = []
    for i, (observations, _) in enumerate(Scenario.from_name(ScenarioType.REAL, "straight_even").steps()):
        # The recording's wheel speed starts at 0.0, which the speed controller accepts as fresh
        observations = dataclasses.replace(observations, policy=PolicyState(0.1 if i else 0.0, 0.1 * i, i == 0))
        actions = policy.step(observations)
        assert 0.0 <= actions.drive_action <= CONTROL.max_drive_action
        assert abs(actions.steering_action) <= CONTROL.max_steering_action
        drives.append(actions.drive_action)
    assert all(drive > 0.0 for drive in drives[:30])
    assert all(drive == 0.0 for drive in drives[-5:])
