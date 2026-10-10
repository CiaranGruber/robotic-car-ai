"""Hardware-free checks of the temporary control conversion and the whole movement policy, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_control.py
"""
import dataclasses
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from policy.action_policy.mpc import MpcConfig  # noqa: E402
from policy.cone_filter.cone_filter import ConeFilterConfig  # noqa: E402
from policy.control.actions import DriveCommand, MoveCameraTo  # noqa: E402
from policy.control.control import CarControl, ControlConfig  # noqa: E402
from policy.input_output import (  # noqa: E402
    CarActions, CarObservations, ConeBatch, ObservedCarState, PolicyState, SensorAge, WheelSpeed, default_actions)
from policy.policy_runner import MovementPolicy  # noqa: E402
from test_mpc import lane_cones  # noqa: E402


def observe(wheel_speed=None, first_step=False, dt=0.1, cones=None):
    speed = None if wheel_speed is None else WheelSpeed(wheel_speed, SensorAge(0.0, None))
    return CarObservations(
        cones=None if cones is None else ConeBatch(cones, SensorAge(0.0, None), acquisition_to_publish_latency_s=0.0),
        fiducials=None,
        lidar_scan=None,
        lidar_cartesian=None,
        car=ObservedCarState(None, None, None, speed),
        policy=PolicyState(dt=0.0 if first_step else dt, policy_elapsed_s=0.0, is_first_policy_step=first_step),
    )


@pytest.mark.parametrize("turns_left", [True, False])
def test_steering_follows_the_bicycle_model_and_configured_direction(turns_left):
    config = ControlConfig(positive_steering_turns_left=turns_left)
    actions = CarControl(config).determine_car_actions(observe(0.5), [DriveCommand(0.5, 1.0)])
    expected = math.atan(config.wheelbase_m * 1.0) / config.max_steering_angle_rad
    assert actions.steering_action == pytest.approx(expected if turns_left else -expected)


def test_steering_saturates_at_full_lock():
    control = CarControl(ControlConfig())
    assert control.determine_car_actions(observe(0.5), [DriveCommand(0.5, -50.0)]).steering_action == -1.0


def test_drive_effort_rises_while_too_slow_but_never_exceeds_the_limit():
    # Explicit saturation target, independent of the constructor's tuning defaults.
    config = ControlConfig(max_drive_effort=0.2)
    control = CarControl(config)
    efforts = [control.determine_car_actions(observe(0.3, first_step=step == 0), [DriveCommand(0.5, 0.0)]).drive_action
               for step in range(150)]
    assert all(0.0 <= effort <= config.max_drive_effort for effort in efforts)
    assert efforts[-1] > efforts[0]
    assert efforts[-1] == pytest.approx(config.max_drive_effort)


def test_drive_effort_is_the_feedforward_at_the_requested_speed():
    config = ControlConfig()
    actions = CarControl(config).determine_car_actions(observe(0.5, first_step=True), [DriveCommand(0.5, 0.0)])
    assert actions.drive_action == pytest.approx(config.drive_effort_per_m_per_s * 0.5)


@pytest.mark.parametrize("wheel_speed, instructions", [
    (0.5, [DriveCommand(0.0, 0.0)]),  # stop requested
    (None, [DriveCommand(0.5, 0.0)]),  # no speed feedback
], ids=["stop", "no-wheel-speed"])
def test_no_drive_effort_when_stopping_or_without_speed_feedback(wheel_speed, instructions):
    assert CarControl(ControlConfig()).determine_car_actions(observe(wheel_speed), instructions).drive_action == 0.0


def test_no_drive_command_gives_default_actions():
    assert CarControl(ControlConfig()).determine_car_actions(observe(0.5), [MoveCameraTo(0)]) == default_actions()


def test_first_policy_step_clears_the_speed_integral():
    control = CarControl(ControlConfig())
    for _ in range(50):
        control.determine_car_actions(observe(0.0), [DriveCommand(1.0, 0.0)])
    fresh = CarControl(ControlConfig()).determine_car_actions(observe(0.0, first_step=True), [DriveCommand(1.0, 0.0)])
    reused = control.determine_car_actions(observe(0.0, first_step=True), [DriveCommand(1.0, 0.0)])
    assert reused == fresh


@pytest.mark.parametrize("change", [
    {"wheelbase_m": 0.0}, {"max_steering_angle_rad": math.pi / 2.0}, {"speed_kp": -0.1},
    {"max_drive_effort": 1.5}, {"max_drive_effort": 0.0}, {"positive_steering_turns_left": 1},
])
def test_invalid_settings_are_rejected(change):
    with pytest.raises(ValueError):
        dataclasses.replace(ControlConfig(), **change)


def test_movement_policy_turns_back_to_the_lane_centre_within_the_effort_limit():
    """The whole chain: cones, MPC, then control. The car is 0.3 m left of the lane centre."""
    control_config = ControlConfig()
    actions = MovementPolicy(MpcConfig(), ConeFilterConfig(), control_config).step(
        observe(0.3, first_step=True, cones=lane_cones(0.3, 0.0)))
    assert isinstance(actions, CarActions)
    assert actions.steering_action < 0.0  # right, with positive steering turning left
    assert 0.0 < actions.drive_action <= control_config.max_drive_effort
    assert actions.camera_pan_action is None


def test_movement_policy_stops_without_a_lane():
    actions = MovementPolicy(MpcConfig(), ConeFilterConfig(), ControlConfig()).step(observe(0.5, first_step=True, cones=[]))
    assert actions == default_actions()
