"""Hardware-free checks of the low-level control module, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_control.py
"""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from policy.control.actions import DriveCommand, MoveCameraTo  # noqa: E402
from policy.control.control import (CarController, ControlConfig, SpeedController,  # noqa: E402
                                    steering_for_curvature)
from policy.input_output import CarObservations, ObservedCarState, PolicyState, SensorAge, WheelSpeed  # noqa: E402

CONFIG = ControlConfig()


def observations(wheel_speed, dt=0.02, first=False):
    speed = None if wheel_speed is None else WheelSpeed(wheel_speed, SensorAge(0.01, None))
    return CarObservations(None, None, None, None, ObservedCarState(None, None, None, speed),
                           PolicyState(dt, 0.0, first))


def test_stop_and_no_command_give_zero_drive_and_centred_steering():
    controller = CarController(CONFIG)
    for instructions in ([], [MoveCameraTo(0)], [DriveCommand(0.0, 0.0)]):
        actions = controller.step(observations(0.0), instructions)
        assert actions.drive_action == 0.0
        assert abs(actions.steering_action) <= abs(CONFIG.steering_centre_action)


def test_feedforward_matches_speed_map_at_target_speed():
    controller = SpeedController(CONFIG)
    drive = controller.step(0.6, 0.6, 0.02)
    assert drive == pytest.approx(CONFIG.speed_ff_offset + 0.6 / CONFIG.speed_ff_gain)


def test_feedback_pushes_towards_target_and_is_limited():
    controller = SpeedController(CONFIG)
    ff = CONFIG.speed_ff_offset + 0.6 / CONFIG.speed_ff_gain
    assert controller.step(0.6, 0.4, 0.02) > ff
    assert controller.step(0.6, 0.8, 0.02) < ff
    assert controller.step(5.0, 0.0, 0.02) == CONFIG.max_drive_action


def test_integral_removes_steady_error_and_stays_bounded():
    controller = SpeedController(CONFIG)
    for _ in range(10000):
        controller.step(0.45, 0.35, 0.02)  # Stays below max_drive_action, so anti-windup does not stop it
    assert controller.integral == pytest.approx(CONFIG.speed_i_max)
    for _ in range(10000):
        controller.step(0.6, 0.5, 0.02)  # Reaches max_drive_action, so anti-windup holds the integral
    assert controller.step(0.6, 0.5, 0.02) == CONFIG.max_drive_action
    controller.step(0.0, 0.5, 0.02)
    assert controller.integral == 0.0


def test_integral_only_near_target():
    controller = SpeedController(CONFIG)
    controller.step(1.0, 0.0, 0.02)  # Error 1.0 m/s is outside speed_i_band
    assert controller.integral == 0.0


def test_missing_wheel_speed_stops_when_feedback_is_needed():
    controller = CarController(CONFIG)
    assert controller.step(observations(None), [DriveCommand(0.6, 0.0)]).drive_action == 0.0


def test_steering_signs_and_limits_follow_car_20_map():
    # Positive curvature (left) needs negative steering on car 20
    assert steering_for_curvature(0.3, CONFIG) < 0.0 < steering_for_curvature(-0.3, CONFIG)
    assert steering_for_curvature(0.0, CONFIG) == CONFIG.steering_centre_action
    assert steering_for_curvature(10.0, CONFIG) == -CONFIG.max_steering_action
    assert steering_for_curvature(-10.0, CONFIG) == CONFIG.max_steering_action


def test_first_policy_step_resets_integral():
    controller = CarController(CONFIG)
    for _ in range(100):
        controller.step(observations(0.5), [DriveCommand(0.6, 0.0)])
    assert controller.speed_controller.integral > 0.0
    controller.step(observations(0.6, dt=0.0, first=True), [DriveCommand(0.6, 0.0)])
    assert controller.speed_controller.integral == 0.0


@pytest.mark.parametrize("change", [
    {"speed_ff_gain": 0.0},
    {"speed_kp": -0.1},
    {"max_drive_action": 1.5},
    {"steering_left_action_per_curvature": 1.0},
    {"steering_right_action_per_curvature": 0.0},
])
def test_invalid_settings_are_rejected(change):
    with pytest.raises(ValueError):
        ControlConfig(**change)
