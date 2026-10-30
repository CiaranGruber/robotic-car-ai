"""Hardware-free checks of the steering test (policy/action_policy/steering_test.py), without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_steering_test.py
"""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from policy.action_policy.mpc import MpcConfig  # noqa: E402
from policy.action_policy.steering_test import SteeringTest, SteeringTestConfig  # noqa: E402
from policy.cone_filter.cone_filter import ConeFilterConfig  # noqa: E402
from policy.control.control import ControlConfig  # noqa: E402
from policy.input_output import (  # noqa: E402
    AngularVelocity, CarObservations, ObservedCarState, PolicyState, SensorAge, WheelSpeed)
from policy.memory import CarMemory  # noqa: E402
from policy.policy_runner import MovementPolicy  # noqa: E402


def observations(elapsed_s, wheel_speed=0.0, yaw_rate=None, dt=0.1):
    speed = None if wheel_speed is None else WheelSpeed(wheel_speed, SensorAge(0.01, None))
    rates = None if yaw_rate is None else AngularVelocity(0.0, 0.0, yaw_rate, SensorAge(0.01, None))
    return CarObservations(None, None, None, None, ObservedCarState(None, rates, None, speed),
                           PolicyState(0.0 if elapsed_s == 0.0 else dt, elapsed_s, elapsed_s == 0.0))


def test_holds_each_action_in_turn_then_stops_centred():
    test = SteeringTest(SteeringTestConfig("-0.5,-1.0", hold_s=2.0), ControlConfig())
    held = [(test.step(observations(t / 10)).steering_action, test.step(observations(t / 10)).debug1)
            for t in (0, 10, 19, 20, 39, 40, 60)]
    assert held == [(-0.5, 0.0), (-0.5, 0.0), (-0.5, 0.0), (-1.0, 1.0), (-1.0, 1.0), (0.0, -1.0), (0.0, -1.0)]


def test_stand_test_never_drives():
    test = SteeringTest(SteeringTestConfig("1.0", speed_m_per_s=0.0), ControlConfig())
    assert all(test.step(observations(t / 10)).drive_action == 0.0 for t in range(30))


def test_drives_and_measures_curvature_from_yaw_rate():
    test = SteeringTest(SteeringTestConfig("-1.0", speed_m_per_s=0.5), ControlConfig())
    test.step(observations(0.0))
    actions = test.step(observations(1.0, wheel_speed=0.5, yaw_rate=0.235))
    assert actions.drive_action > 0.0
    assert actions.debug2 == pytest.approx(0.47)
    # No curvature from a near-standstill or without the gyro
    assert test.step(observations(1.1, wheel_speed=0.05, yaw_rate=0.2)).debug2 == 0.0
    assert test.step(observations(1.2, wheel_speed=0.5, yaw_rate=None)).debug2 == 0.0


@pytest.mark.parametrize("limit, observed", [("max_distance_m", {"wheel_speed": 0.6}),
                                             ("max_speed_m_per_s", {"wheel_speed": 1.5})])
def test_stops_for_good_after_a_safety_limit(limit, observed):
    config = SteeringTestConfig("-1.0", hold_s=100.0, speed_m_per_s=0.5, max_distance_m=1.0)
    test = SteeringTest(config, ControlConfig())
    test.step(observations(0.0))
    for step in range(1, 30):
        actions = test.step(observations(step / 10, **observed))
    assert test.stop_reason is not None and limit.split("_m")[0].replace("max_", "") in test.stop_reason
    assert (actions.drive_action, actions.steering_action, actions.debug1) == (0.0, 0.0, -1.0)
    # A new run starts the list again
    assert test.step(observations(0.0)).debug1 == 0.0


@pytest.mark.parametrize("settings", [{"steering_actions": "-1.2"}, {"steering_actions": "left"},
                                      {"steering_actions": ""}, {"hold_s": 0.0}, {"speed_m_per_s": -0.1},
                                      {"speed_m_per_s": 2.0}])
def test_rejects_invalid_settings(settings):
    with pytest.raises(ValueError):
        SteeringTestConfig(**settings)


def test_movement_policy_runs_the_steering_test():
    policy = MovementPolicy(MpcConfig(), ConeFilterConfig(), ControlConfig(), action_policy="steering_test",
                            steering_test_config=SteeringTestConfig("0.8"))
    actions = policy.step(observations(0.0), CarMemory())
    assert actions.steering_action == 0.8 and actions.debug1 == 0.0
