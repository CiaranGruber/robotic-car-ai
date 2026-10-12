"""Hardware-free checks of the MPC movement policy, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_mpc.py
"""
import dataclasses
import math

import pytest
import numpy as np

from policy.action_policy.mpc import MpcConfig, MpcController, MpcSolution, PathTrackingState
from policy.action_policy.policy import MpcPolicy
from policy.action_policy.reference import reference_from_cones
from policy.control.actions import DriveCommand
from policy.input_output import (
    CarObservations, ConeBatch, ConeColour, ConeDetection, ObservedCarState, PolicyState, Position, SensorAge)

LANE_WIDTH_M = 1.0


def lane_cones(offset_m, heading_rad, colours=(ConeColour.YELLOW, ConeColour.BLUE)):
    """Cones of a straight lane, seen from a car offset_m left of the lane centre and heading_rad left of the
    lane direction. Yellow is on the left and blue on the right, 0.5 m apart, visible up to 4 m ahead."""
    cones = []
    for colour, side in ((ConeColour.YELLOW, 1.0), (ConeColour.BLUE, -1.0)):
        if colour not in colours:
            continue
        for i in range(-4, 20):
            along, across = i * 0.5, side * LANE_WIDTH_M / 2.0 - offset_m
            x = math.cos(heading_rad) * along + math.sin(heading_rad) * across
            y = -math.sin(heading_rad) * along + math.cos(heading_rad) * across
            if 0.0 < x <= 4.0:
                cones.append(ConeDetection(Position(x, y, 0.1), colour, 0.9))
    return cones


def observe(cones, first_step=False, cone_age_s=0.0):
    return CarObservations(
        cones=(None if cones is None else ConeBatch(cones, SensorAge(cone_age_s, None), acquisition_to_publish_latency_s=0.0)),
        fiducials=None,
        lidar_scan=None,
        lidar_cartesian=None,
        car=ObservedCarState(None, None, None, None),
        policy=PolicyState(dt=0.0 if first_step else 0.1, policy_elapsed_s=0.0, is_first_policy_step=first_step),
    )


def command(policy, cones, first_step=True, cone_age_s=0.0):
    instructions = policy.run_policy(observe(cones, first_step, cone_age_s), None)
    assert len(instructions) == 1 and isinstance(instructions[0], DriveCommand)
    return instructions[0]


def test_centred_car_drives_straight_at_the_target_speed():
    result = command(MpcPolicy(MpcConfig()), lane_cones(0.0, 0.0))
    assert result.speed_m_per_s == MpcConfig().target_speed_m_per_s
    assert result.curvature_per_m == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize("offset_m, heading_rad", [(0.3, 0.0), (-0.3, 0.0), (0.0, 0.2), (0.0, -0.2)])
def test_turns_back_towards_the_lane_centre(offset_m, heading_rad):
    result = command(MpcPolicy(MpcConfig()), lane_cones(offset_m, heading_rad))
    # Left of the centre or pointing left needs a right turn (negative curvature), and the reverse
    assert math.copysign(1.0, result.curvature_per_m) == -math.copysign(1.0, offset_m + heading_rad)


def test_curvature_stays_within_the_limit():
    config = MpcConfig(max_curvature_per_m=0.5)
    result = command(MpcPolicy(config), lane_cones(0.4, 0.5))
    assert result.curvature_per_m == pytest.approx(-config.max_curvature_per_m)


@pytest.mark.parametrize("colours", [(ConeColour.YELLOW,), (ConeColour.BLUE,)])
def test_one_visible_boundary_still_gives_the_lane_centre(colours):
    state = reference_from_cones(lane_cones(0.2, 0.1, colours), LANE_WIDTH_M)
    both = reference_from_cones(lane_cones(0.2, 0.1), LANE_WIDTH_M)
    assert state.lateral_error_m == pytest.approx(both.lateral_error_m)
    assert state.heading_error_rad == pytest.approx(both.heading_error_rad)


@pytest.mark.parametrize("cones", [
    None,  # no fresh detection batch
    [],  # a valid empty frame
    lane_cones(0.0, 0.0)[:1],  # one cone cannot be fitted
    lane_cones(0.8, 0.0),  # outside the lane: both boundaries are on the right
], ids=["missing", "empty", "one-cone", "outside-lane"])
def test_stops_without_a_usable_lane(cones):
    assert command(MpcPolicy(MpcConfig()), cones) == DriveCommand(0.0, 0.0)


def test_first_policy_step_forgets_the_previous_run():
    policy = MpcPolicy(MpcConfig())
    for _ in range(5):
        command(policy, lane_cones(0.4, 0.0), first_step=False)
    assert policy.previous_curvature_per_m < 0.0
    # A new run starting centred must not continue the previous run's turn
    assert command(policy, lane_cones(0.0, 0.0), first_step=True).curvature_per_m == pytest.approx(0.0, abs=1e-6)


def drive(policy, offset_m, steps, delay_steps=1, missed=lambda _: False, step_s=0.1, substeps=10):
    """Drives a kinematic car at the commanded speed and curvature, starting offset_m left of the lane centre.

    Policy steps are step_s apart. Each detection shows the car delay_steps steps ago, and steps where missed(step) is
    true have no cones. Returns the car's offset after each step and its final heading.
    """
    offset, heading = offset_m, 0.0
    poses = [(offset, heading)] * (delay_steps + 1)
    offsets = []
    for step in range(steps):
        cones = [] if missed(step) else lane_cones(*poses[-1 - delay_steps])
        result = command(policy, cones, first_step=step == 0, cone_age_s=delay_steps * step_s)
        for _ in range(substeps):
            distance = result.speed_m_per_s * step_s / substeps
            heading += distance * result.curvature_per_m
            offset += distance * math.sin(heading)
        poses.append((offset, heading))
        offsets.append(offset)
    return offsets, heading


def test_closed_loop_settles_on_the_lane_centre_despite_detection_delay():
    policy = MpcPolicy(MpcConfig())
    offsets, heading = drive(policy, 0.3, 80)
    assert abs(offsets[-1]) < 0.02 and abs(heading) < 0.02
    assert min(offsets) > -0.1  # crosses the centre by less than 10 cm
    assert policy.fallback_count == 0


def test_closed_loop_settles_with_the_measured_delay_and_missed_batches():
    """Cones at 10 Hz and 0.2 s old, as measured on the car on 2026-10-06, with every third batch unusable."""
    policy = MpcPolicy(MpcConfig(target_speed_m_per_s=0.5, horizon_steps=30))
    offsets, heading = drive(policy, 0.3, 160, delay_steps=2, missed=lambda step: step % 3 == 2)
    assert abs(offsets[-1]) < 0.02 and abs(heading) < 0.02
    assert min(offsets) > -0.1
    assert not policy.stopped_for_lane_loss
    assert policy.lane_lost_step_count == 53 and policy.fallback_count == 0


def test_follows_the_plan_at_reduced_speed_while_the_lane_is_briefly_lost():
    config = MpcConfig()
    policy = MpcPolicy(config)
    command(policy, lane_cones(0.3, 0.0))
    plan = policy.plan_curvatures_per_m
    assert plan is not None
    lost = command(policy, [], first_step=False)
    assert lost.speed_m_per_s == pytest.approx(config.lane_loss_speed_fraction * config.target_speed_m_per_s)
    # 0.1 s at 1 m/s is one 0.1 m step of the plan
    assert lost.curvature_per_m == pytest.approx(plan[1])
    assert policy.lane_lost_step_count == 1
    # The lane comes back: drive at the target speed again
    assert command(policy, lane_cones(0.25, 0.0), first_step=False).speed_m_per_s == config.target_speed_m_per_s


@pytest.mark.parametrize("timeout_s, driven_steps", [(0.25, 2), (0.0, 0)])
def test_stops_for_the_rest_of_the_run_after_the_lane_loss_timeout(timeout_s, driven_steps):
    config = MpcConfig(lane_loss_timeout_s=timeout_s)
    policy = MpcPolicy(config)
    command(policy, lane_cones(0.0, 0.0))
    # Steps are 0.1 s apart
    speeds = [command(policy, [], first_step=False).speed_m_per_s for _ in range(4)]
    assert all(speed > 0.0 for speed in speeds[:driven_steps])
    assert speeds[driven_steps:] == [0.0] * (4 - driven_steps)
    # Cones reappearing do not restart the run; a new run does
    assert command(policy, lane_cones(0.0, 0.0), first_step=False) == DriveCommand(0.0, 0.0)
    assert command(policy, lane_cones(0.0, 0.0), first_step=True).speed_m_per_s == config.target_speed_m_per_s


def test_waits_stopped_until_the_first_usable_lane():
    policy = MpcPolicy(MpcConfig())
    # Longer than the lane loss timeout, which only applies after the first usable lane
    for step in range(10):
        assert command(policy, [], first_step=step == 0) == DriveCommand(0.0, 0.0)
    assert command(policy, lane_cones(0.0, 0.0), first_step=False).speed_m_per_s == MpcConfig().target_speed_m_per_s


def test_run_duration_stops_with_fresh_lane_and_resets_on_new_run():
    policy = MpcPolicy(MpcConfig(run_duration_s=0.2))
    cones = lane_cones(0.2, 0.0)
    assert command(policy, cones).speed_m_per_s > 0
    observations = observe(cones)
    for elapsed in (0.2, 0.3):
        timed = dataclasses.replace(observations, policy=PolicyState(0.1, elapsed, False))
        assert policy.run_policy(timed, None) == [DriveCommand(0.0, 0.0)]
    assert command(policy, cones, first_step=True).speed_m_per_s > 0


@pytest.mark.parametrize("change", [
    {"horizon_steps": 0}, {"horizon_steps": 2.0}, {"step_s": 0.0}, {"curvature_weight": 0.0},
    {"lateral_error_weight": -1.0}, {"target_speed_m_per_s": math.nan}, {"max_curvature_per_m": math.inf},
    {"lane_loss_timeout_s": -0.1}, {"lane_loss_timeout_s": math.inf}, {"lane_loss_speed_fraction": 1.5},
    {"run_duration_s": -0.1}, {"run_duration_s": math.inf},
])
def test_invalid_settings_are_rejected(change):
    with pytest.raises(ValueError):
        dataclasses.replace(MpcConfig(), **change)


def test_measured_speed_matches_an_equivalent_nominal_speed_model():
    state = PathTrackingState(0.2, 0.1, 0.05)
    controller = MpcController(MpcConfig())
    for speed in (0.2, 0.7, 1.5):
        actual = controller.solve(state, -0.1, 0.2, speed_m_per_s=speed)
        equivalent = MpcController(MpcConfig(target_speed_m_per_s=speed)).solve(state, -0.1, 0.2)
        assert actual.converged and equivalent.converged
        np.testing.assert_allclose(actual.curvatures_per_m, equivalent.curvatures_per_m)


def test_stationary_car_does_not_predict_lateral_motion_during_delay():
    controller = MpcController(MpcConfig())
    state = PathTrackingState(0.3, 0.2, 0.0)
    stationary = controller.solve(state, 0.0, 0.5, speed_m_per_s=0.0)
    moving = controller.solve(state, 0.0, 0.5, speed_m_per_s=1.0)
    assert stationary.converged
    np.testing.assert_allclose(stationary.curvatures_per_m, 0.0, atol=1e-12)
    assert moving.curvatures_per_m[0] < 0


@pytest.mark.parametrize("speed,delay", [(math.nan, 0), (math.inf, 0), (-1, 0), (1, -0.1), (1, math.nan)])
def test_invalid_motion_inputs_do_not_reach_solver(speed, delay):
    with pytest.raises(ValueError):
        MpcController(MpcConfig()).solve(PathTrackingState(0, 0, 0), 0, delay, speed)


def test_solver_failure_stops_and_requires_explicit_new_run(monkeypatch):
    policy = MpcPolicy(MpcConfig())
    cones = lane_cones(0.2, 0.0)
    assert command(policy, cones).speed_m_per_s > 0
    original_solve = policy.controller.solve
    monkeypatch.setattr(policy.controller, "solve", lambda *args, **kwargs:
                        MpcSolution(np.ones(12), False, 0.001))
    assert command(policy, cones, first_step=False) == DriveCommand(0, 0)
    assert policy.stopped_for_solver_failure and policy.fallback_count == 1
    monkeypatch.setattr(policy.controller, "solve", original_solve)
    assert command(policy, cones, first_step=False) == DriveCommand(0, 0)
    assert command(policy, cones, first_step=True).speed_m_per_s > 0
    assert policy.fallback_count == 0


@pytest.mark.parametrize("bad_plan", [np.full(12, math.nan), np.full(12, 4.0), np.zeros(2)])
def test_invalid_solver_result_is_rejected(monkeypatch, bad_plan):
    from types import SimpleNamespace
    monkeypatch.setattr("policy.action_policy.mpc.lsq_linear",
                        lambda *args, **kwargs: SimpleNamespace(status=1, x=bad_plan))
    result = MpcController(MpcConfig()).solve(PathTrackingState(0, 0, 0), 0, 0)
    assert not result.converged
    np.testing.assert_array_equal(result.curvatures_per_m, np.zeros(12))


def test_exhausted_plan_stops_even_before_lane_loss_timeout():
    policy = MpcPolicy(MpcConfig(horizon_steps=1, lane_loss_timeout_s=5.0))
    command(policy, lane_cones(0.2, 0.0))
    assert command(policy, [], first_step=False) == DriveCommand(0, 0)
    assert policy.stopped_for_lane_loss


def test_measured_speed_controls_plan_progress():
    policy = MpcPolicy(MpcConfig())
    observations = observe(lane_cones(0.2, 0.0), first_step=True)
    observations = dataclasses.replace(observations, car=dataclasses.replace(observations.car, wheel_speed=0.5))
    policy.run_policy(observations, None)
    assert policy.plan_step_distance_m == pytest.approx(0.05)
    plan = policy.plan_curvatures_per_m.copy()
    lost = dataclasses.replace(observations, cones=None, policy=PolicyState(0.1, 0.1, False))
    result = policy.run_policy(lost, None)[0]
    assert policy.distance_since_plan_m == pytest.approx(0.05)
    assert result.curvature_per_m == pytest.approx(plan[1])


def test_linalg_failure_returns_a_failed_plan(monkeypatch):
    def fail(*args, **kwargs):
        raise np.linalg.LinAlgError("synthetic failure")
    monkeypatch.setattr("policy.action_policy.mpc.lsq_linear", fail)
    result = MpcController(MpcConfig()).solve(PathTrackingState(0, 0, 0), 0, 0)
    assert not result.converged


def test_solution_minimises_independently_rolled_out_tracking_cost():
    config = MpcConfig(horizon_steps=5)
    state = PathTrackingState(0.08, -0.02, 0.03)
    speed, previous = 0.6, 0.02
    solution = MpcController(config).solve(state, previous, 0.0, speed)
    assert solution.converged

    def cost(plan):
        lateral, heading, last, total = state.lateral_error_m, state.heading_error_rad, previous, 0.0
        distance = speed * config.step_s
        for curvature in plan:
            turn = curvature - state.path_curvature_per_m
            lateral += distance * heading + 0.5 * distance ** 2 * turn
            heading += distance * turn
            total += (config.lateral_error_weight * lateral ** 2 + config.heading_error_weight * heading ** 2
                      + config.curvature_weight * turn ** 2 + config.curvature_change_weight * (curvature-last) ** 2)
            last = curvature
        return total

    # This mild case has an interior optimum: perturbing any action either way must increase cost.
    optimal = cost(solution.curvatures_per_m)
    for index in range(config.horizon_steps):
        for delta in (-0.001, 0.001):
            perturbed = solution.curvatures_per_m.copy()
            perturbed[index] += delta
            assert cost(perturbed) > optimal
