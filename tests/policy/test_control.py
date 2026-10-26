"""Hardware-free checks of the low-level control module and the whole movement policy, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_control.py
"""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from policy.action_policy.mpc import MpcConfig  # noqa: E402
from policy.cone_filter.cone_filter import ConeFilterConfig  # noqa: E402
from policy.control.actions import DriveCommand, MoveCameraTo  # noqa: E402
from policy.control.control import (CarController, ControlConfig, SpeedController,  # noqa: E402
                                    steering_for_curvature)
from policy.input_output import (  # noqa: E402
    CarActions, CarObservations, ConeBatch, ObservedCarState, PolicyState, SensorAge, WheelSpeed)
from policy.memory import CarMemory  # noqa: E402
from policy.policy_runner import MovementPolicy  # noqa: E402
from tests.policy.test_mpc import lane_cones  # noqa: E402

CONFIG = ControlConfig()


def observations(wheel_speed, dt=0.02, first=False, cones=None):
    speed = None if wheel_speed is None else WheelSpeed(wheel_speed, SensorAge(0.01, None))
    batch = None if cones is None else ConeBatch(cones, SensorAge(0.0, None), acquisition_to_publish_latency_s=0.0)
    return CarObservations(batch, None, None, None, ObservedCarState(None, None, None, speed),
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
    # A lower speed_ff_offset keeps the drive below max_drive_action, so anti-windup does not stop the integral
    controller = SpeedController(ControlConfig(speed_ff_offset=0.3))
    for _ in range(10000):
        controller.step(0.45, 0.35, 0.02)  # Stays below max_drive_action, so anti-windup does not stop it
    assert controller.integral == pytest.approx(CONFIG.speed_i_max)
    for _ in range(10000):
        controller.step(1.5, 1.4, 0.02)  # Reaches max_drive_action, so anti-windup holds the integral
    assert controller.step(1.5, 1.4, 0.02) == CONFIG.max_drive_action
    controller.step(0.0, 0.5, 0.02)
    assert controller.integral == 0.0


def test_integral_only_near_target():
    controller = SpeedController(CONFIG)
    controller.step(1.0, 0.3, 0.02)  # Error 0.7 m/s while moving is outside speed_i_band
    assert controller.integral == 0.0


def test_integral_starts_a_car_whose_feedforward_is_too_low():
    # 26 Oct: with speed_ff_offset 0.15 the wheels never turned for a 0.5 target. The error from standstill is outside
    # speed_i_band, but the integral must still raise the drive until the car moves.
    controller = SpeedController(CONFIG)
    first = controller.step(0.5, 0.0, 0.1)
    for _ in range(20):
        controller.step(0.5, 0.0, 0.1)
    assert controller.integral > 0.0
    assert controller.step(0.5, 0.0, 0.1) > first
    controller.reset()
    controller.step(0.5, 0.2, 0.1)  # Moving and still outside the band while accelerating: no windup
    assert controller.integral == 0.0


def test_integral_pulls_a_large_overspeed_down():
    # 26 Oct: with speed_ff_offset too high the car held 0.87 m/s for a 0.5 target. That overspeed is outside
    # speed_i_band, but the integral must still accumulate, or only the P term corrects it.
    controller = SpeedController(CONFIG)
    first = controller.step(0.5, 0.87, 0.1)
    assert controller.integral < 0.0
    for _ in range(100):
        controller.step(0.5, 0.87, 0.1)
    assert controller.step(0.5, 0.87, 0.1) < first


def test_missing_wheel_speed_stops_when_feedback_is_needed():
    controller = CarController(CONFIG)
    assert controller.step(observations(None), [DriveCommand(0.6, 0.0)]).drive_action == 0.0


BRAKING = ControlConfig(brake_action=0.3, brake_max_s=0.5)
BRAKE = DriveCommand(0.0, 0.0, brake=True)


def test_braking_stop_brakes_straight_from_driving_until_the_wheels_stop():
    controller = CarController(BRAKING)
    assert controller.step(observations(0.5, dt=0.0, first=True), [DriveCommand(0.5, 0.0)]).drive_action > 0.0
    assert controller.step(observations(0.5), [BRAKE]).drive_action == -0.3
    assert controller.step(observations(0.2), [BRAKE]).drive_action == -0.3
    assert controller.step(observations(0.0), [BRAKE]).drive_action == 0.0
    # Reverse effort after neutral could drive the car backwards, so braking does not start again
    assert controller.step(observations(0.2), [BRAKE]).drive_action == 0.0


def test_braking_ends_after_brake_max_s_while_the_wheel_speed_lags():
    controller = CarController(BRAKING)
    controller.step(observations(0.5, dt=0.0, first=True), [DriveCommand(0.5, 0.0)])
    drive = [controller.step(observations(0.3, dt=0.1), [BRAKE]).drive_action for _ in range(10)]
    assert drive == [-0.3] * 5 + [0.0] * 5


@pytest.mark.parametrize("config, before, wheel_speed", [
    (BRAKING, DriveCommand(0.0, 0.0), 0.5),  # Already at zero drive
    (BRAKING, DriveCommand(0.5, 0.0), None),  # Wheel speed missing: zero drive before, and nothing to check
    (CONFIG, DriveCommand(0.5, 0.0), 0.5),  # brake_action 0.0 only coasts
])
def test_braking_stop_coasts_unless_braking_can_start_from_driving(config, before, wheel_speed):
    controller = CarController(config)
    controller.step(observations(wheel_speed, dt=0.0, first=True), [before])
    assert controller.step(observations(wheel_speed), [BRAKE]).drive_action == 0.0


def test_driving_again_allows_the_next_stop_to_brake():
    controller = CarController(BRAKING)
    controller.step(observations(0.5, dt=0.0, first=True), [DriveCommand(0.5, 0.0)])
    controller.step(observations(0.0), [BRAKE])
    controller.step(observations(0.0), [DriveCommand(0.5, 0.0)])
    assert controller.step(observations(0.5), [BRAKE]).drive_action == -0.3


def test_stop_without_brake_coasts_even_with_brake_action():
    controller = CarController(BRAKING)
    controller.step(observations(0.5, dt=0.0, first=True), [DriveCommand(0.5, 0.0)])
    assert controller.step(observations(0.5), [DriveCommand(0.0, 0.0)]).drive_action == 0.0


@pytest.mark.parametrize("dt, wheel_speed", [(float("nan"), 0.5), (-0.1, 0.5), (0.02, float("nan")), (0.02, -0.1)])
def test_invalid_braking_inputs_are_rejected_and_end_braking(dt, wheel_speed):
    controller = CarController(BRAKING)
    controller.step(observations(0.5, dt=0.0, first=True), [DriveCommand(0.5, 0.0)])
    with pytest.raises(ValueError):
        controller.step(observations(wheel_speed, dt=dt), [BRAKE])
    assert controller.step(observations(0.5), [BRAKE]).drive_action == 0.0


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
    {"brake_action": -0.1},
    {"brake_action": 1.0},
    {"brake_max_s": -0.1},
    {"brake_max_s": float("inf")},
])
def test_invalid_settings_are_rejected(change):
    with pytest.raises(ValueError):
        ControlConfig(**change)


def test_steering_direction_follows_the_map_sign():
    assert not CONFIG.positive_steering_turns_left  # car 20: positive steering turns right
    assert ControlConfig(steering_left_action_per_curvature=1.0,
                         steering_right_action_per_curvature=1.0).positive_steering_turns_left


def test_movement_policy_turns_back_to_the_lane_centre_within_the_drive_limit():
    """The whole chain: cones, MPC, then control. The car is 0.3 m left of the lane centre."""
    actions = MovementPolicy(MpcConfig(), ConeFilterConfig(), CONFIG).step(
        observations(0.3, dt=0.0, first=True, cones=lane_cones(0.3, 0.0)), CarMemory())
    assert isinstance(actions, CarActions)
    assert actions.steering_action > CONFIG.steering_centre_action  # right, with positive steering turning right
    assert 0.0 < actions.drive_action <= CONFIG.max_drive_action
    assert actions.camera_pan_action == 0.0  # held at zero pan, which the camera's fixed transform assumes


def test_movement_policy_publishes_the_lane_state_on_debug1_and_debug2():
    """debug1 is the lateral error in metres and debug2 the heading error in radians, for Foxglove."""
    actions = MovementPolicy(MpcConfig(), ConeFilterConfig(), CONFIG).step(
        observations(0.3, dt=0.0, first=True, cones=lane_cones(0.3, 0.1)), CarMemory())
    assert actions.debug1 == pytest.approx(0.3) and actions.debug2 == pytest.approx(0.1)


def test_movement_policy_stops_without_a_lane():
    actions = MovementPolicy(MpcConfig(), ConeFilterConfig(), CONFIG).step(observations(0.5, dt=0.0, first=True, cones=[]), CarMemory())
    assert actions.drive_action == 0.0
    assert actions.steering_action == CONFIG.steering_centre_action  # the MPC asks to stop, steering straight
    assert actions.debug1 is None and actions.debug2 is None


@pytest.mark.parametrize("curvature", [float("nan"), float("inf"), -float("inf")])
def test_invalid_curvature_is_rejected_before_clipping(curvature):
    with pytest.raises(ValueError):
        CarController(CONFIG).step(observations(0.5), [DriveCommand(0.5, curvature)])


@pytest.mark.parametrize("target, measured, dt", [
    (float("inf"), 0.5, 0.1), (float("nan"), 0.5, 0.1),
    (0.5, float("nan"), 0.1), (0.5, float("inf"), 0.1), (0.5, -0.1, 0.1),
    (0.5, 0.5, float("nan")), (0.5, 0.5, float("inf")), (0.5, 0.5, -0.1),
    (0.1, float("nan"), 0.1),
])
def test_invalid_speed_inputs_clear_integral_and_are_rejected(target, measured, dt):
    controller = SpeedController(CONFIG)
    controller.step(0.6, 0.5, 0.1)
    assert controller.integral > 0.0
    with pytest.raises(ValueError):
        controller.step(target, measured, dt)
    assert controller.integral == 0.0
