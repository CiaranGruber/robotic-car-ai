"""Checks that the car models invert Car Control's maps and that dream-gym's car follows them.

Run from the repository root with: python3 -m pytest tests/policy/test_car_models.py
"""
import dataclasses
import math

import pytest

pytest.importorskip("dreamgym")

import stage1_scenarios as scenarios  # noqa: E402
from car_models import CAR_20, CAR_22A, CARS  # noqa: E402
from policy.control.control import ControlConfig, steering_for_curvature  # noqa: E402
from policy.input_output import CarActions  # noqa: E402


@pytest.mark.parametrize("curvature", [-0.9, -0.3, 0.0, 0.3, 0.45])
def test_car_20_steering_inverts_car_controller_map(curvature):
    assert CAR_20.curvature_per_m(steering_for_curvature(curvature, ControlConfig())) == pytest.approx(curvature)


def test_steering_saturates_at_the_measured_limits():
    assert CAR_20.curvature_per_m(-1.0) == CAR_20.max_left_curvature_per_m
    assert CAR_20.curvature_per_m(1.0) == -CAR_20.max_right_curvature_per_m
    assert CAR_20.max_curvature_per_m == 0.48


def test_no_drive_below_breakaway_and_the_steady_speed_above():
    assert CAR_22A.steady_speed_m_per_s(CAR_22A.breakaway_drive - 0.01) == 0.0
    assert CAR_22A.steady_speed_m_per_s(0.42) == pytest.approx(CAR_22A.speed_gain_m_per_s * 0.10)


@pytest.mark.parametrize("drive, start_speed, time_constant", [
    (0.42, 0.5, CAR_22A.drive_time_constant_s),  # speeding up
    (0.0, 1.0, CAR_22A.coast_time_constant_s),  # coasting
], ids=["drive", "coast"])
def test_simulated_speed_follows_the_car_model(drive, start_speed, time_constant):
    case = dataclasses.replace(scenarios.Stage1Case("model", "tuning", "speed check"), speed_m_per_s=start_speed)
    env = scenarios.make_env(case)
    env.reset(seed=0)
    actions = CarActions(drive, CAR_22A.steering_centre_action, None, None, None)
    speed, steady = start_speed, CAR_22A.steady_speed_m_per_s(drive)
    for step in range(1, 31):
        _, _, _, _, info = env.step(scenarios.car_model_action(actions, speed, CAR_22A, case.disturbances.car_mass_kg))
        speed = scenarios.scalar(info["ground_truth_body_longitudinal_velocity_mps"])
        expected = steady + (start_speed - steady) * math.exp(-step * scenarios.ENV_STEP_S / time_constant)
        assert speed == pytest.approx(expected, abs=0.02)
    env.close()


def test_every_model_runs_a_case_through_car_controller():
    case = dataclasses.replace(scenarios.Stage1Case("model", "tuning", "chain check"), speed_m_per_s=0.5)
    for car in CARS.values():
        result = scenarios.run_case(case, scenarios.shipped_mpc_policy(case, car), car=car)
        assert result.x_m[-1] > case.lane.length_m  # it drove past the lane end
