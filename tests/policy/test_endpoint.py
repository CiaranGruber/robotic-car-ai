"""Endpoint evidence must be temporal, bounded, and independently stoppable."""
import dataclasses
import math

import pytest

from evaluate_mpc import Case, rollout
from policy.action_policy.endpoint import EndpointConfig, EndpointMpcPolicy, EndpointTracker, stopping_speed
from policy.action_policy.mpc import MpcConfig, PathTrackingState
from policy.input_output import ConeBatch, ConeColour, ConeDetection, Position, SensorAge
from test_mpc import observe


def pair_rows(distance, stamp=1, age=0.0):
    return ConeBatch([ConeDetection(Position(x, side * 0.5, 0.1), colour, 0.9)
                      for x in (distance-0.5, distance)
                      for side, colour in ((1, ConeColour.YELLOW), (-1, ConeColour.BLUE))],
                     SensorAge(age, stamp), age)


def test_confirmation_needs_distinct_frames_inside_range_and_motion_consistency():
    tracker = EndpointTracker(EndpointConfig())
    state = PathTrackingState(0, 0, 0)
    tracker.update(pair_rows(3.5), state, 0, 0)
    assert not tracker.confirmed
    tracker.update(pair_rows(2.0, 2), state, 0, 0)
    for _ in range(5):
        tracker.update(pair_rows(2.0, 2), state, 0, 0)
    assert tracker.confirmations == 1
    tracker.update(pair_rows(1.9, 3), state, 1, 0.1)
    tracker.update(pair_rows(1.8, 4), state, 1, 0.1)
    assert tracker.confirmed
    tracker.update(pair_rows(2.4, 5), state, 1, 0.1)
    assert tracker.contradicted and not tracker.confirmed


def test_empty_one_sided_and_stale_batches_do_not_confirm():
    tracker = EndpointTracker(EndpointConfig())
    state = PathTrackingState(0, 0, 0)
    for stamp in range(10):
        batch = pair_rows(2, stamp, age=0.5)
        tracker.update(batch, state, 0, 0.1)
    assert not tracker.confirmed
    for stamp in range(10, 20):
        batch = pair_rows(2, stamp)
        del batch[1::2]
        tracker.update(batch, state, 0, 0.1)
    assert not tracker.confirmed


def test_speed_envelope_decreases_with_distance_and_uncertainty():
    config = EndpointConfig()
    far = stopping_speed(1, 0.04, config)
    near = stopping_speed(0.1, 0.04, config)
    uncertain = stopping_speed(0.1, 0.15, config)
    assert 0 < uncertain < near <= far
    assert stopping_speed(-0.15, 0.04, config) == 0
    usable = 0.1 + config.stop_offset_m - 0.04
    assert near * config.reaction_s + near**2/(2*config.deceleration_m_per_s2) <= usable + 1e-12


def test_missing_stream_latches_stop_until_explicit_restart():
    policy = EndpointMpcPolicy(MpcConfig())
    obs = observe(pair_rows(2), first_step=True)
    obs = dataclasses.replace(obs, cones=pair_rows(2), car=dataclasses.replace(obs.car, wheel_speed=0.5))
    assert policy.run_policy(obs, None)[0].speed_m_per_s > 0
    lost = dataclasses.replace(obs, cones=None, policy=dataclasses.replace(obs.policy, is_first_policy_step=False))
    assert policy.run_policy(lost, None)[0].speed_m_per_s == 0
    restored = dataclasses.replace(lost, cones=pair_rows(2, 2))
    assert policy.run_policy(restored, None)[0].speed_m_per_s == 0
    assert policy.run_policy(obs, None)[0].speed_m_per_s > 0


def test_lost_endpoint_reference_times_out():
    policy = EndpointMpcPolicy(MpcConfig(), EndpointConfig(max_blind_s=0.1))
    obs = observe(pair_rows(2))
    obs = dataclasses.replace(obs, car=dataclasses.replace(obs.car, wheel_speed=0.5))
    for step in range(3):
        policy.run_policy(dataclasses.replace(obs, cones=pair_rows(2-step*0.05, step)), None)
    assert policy.approaching_endpoint
    empty = dataclasses.replace(obs, cones=ConeBatch([], SensorAge(0.0, 4), 0.0))
    policy.run_policy(empty, None)
    assert policy.run_policy(empty, None)[0].speed_m_per_s == 0
    assert policy.endpoint_abort_reason is not None


@pytest.mark.parametrize("changes", [{"confirmation_frames": 1}, {"deceleration_m_per_s2": 0},
                                     {"max_blind_s": math.inf}, {"max_uncertainty_m": 0.01}])
def test_invalid_endpoint_settings_rejected(changes):
    with pytest.raises(ValueError):
        EndpointConfig(**changes)


def test_endpoint_approach_completes_course_with_delay_and_coasting():
    metrics, trace = rollout(MpcConfig(), Case("endpoint", noise=0.01, delay_steps=2, speed_response_s=0.3),
                             301, measured_speed=True, policy_factory=EndpointMpcPolicy)
    assert metrics["completed"] and metrics["endpoint_stop"]
    assert metrics["min_cone_clearance_m"] > 0
    assert metrics["fallbacks"] == 0
    assert any(0 < speed < 0.5 for speed in trace[:, 4])


def test_missing_final_pair_remains_a_documented_false_endpoint():
    metrics, _ = rollout(MpcConfig(), Case("hidden", hide_from_x_m=6.0),
                         301, measured_speed=True, policy_factory=EndpointMpcPolicy)
    # A stable earlier pair is observationally indistinguishable from a real end.
    # This guards against silently reporting it as a successful finish.
    assert metrics["endpoint_stop"]
    assert metrics["stop_x_m"] < 6.0 and not metrics["completed"]


def test_excessive_odometry_uncertainty_aborts_approach():
    policy = EndpointMpcPolicy(MpcConfig(), EndpointConfig(max_uncertainty_m=0.05))
    obs = observe(pair_rows(2))
    obs = dataclasses.replace(obs, car=dataclasses.replace(obs.car, wheel_speed=1.0))
    for step in range(3):
        policy.run_policy(dataclasses.replace(obs, cones=pair_rows(2-step*0.1, step)), None)
    empty = dataclasses.replace(obs, cones=ConeBatch([], SensorAge(0.0, 4), 0.0))
    policy.run_policy(empty, None)
    assert policy.run_policy(empty, None)[0].speed_m_per_s == 0
    assert policy.endpoint_abort_reason == "approach confidence or time limit"


def test_endpoint_stop_stays_latched_until_a_new_run():
    policy = EndpointMpcPolicy(MpcConfig())
    obs = observe(pair_rows(2))
    obs = dataclasses.replace(obs, car=dataclasses.replace(obs.car, wheel_speed=0.5))
    for step in range(3):
        policy.run_policy(dataclasses.replace(obs, cones=pair_rows(2-step*0.05, step)), None)
    assert policy.approaching_endpoint
    policy.endpoint.remaining_m = -0.15
    empty = dataclasses.replace(obs, cones=ConeBatch([], SensorAge(0.0, 4), 0.0))
    assert policy.run_policy(empty, None)[0].speed_m_per_s == 0
    assert policy.stopped_at_endpoint
    assert policy.run_policy(obs, None)[0].speed_m_per_s == 0
    restarted = dataclasses.replace(obs, policy=dataclasses.replace(obs.policy, is_first_policy_step=True, dt=0.0))
    assert policy.run_policy(restarted, None)[0].speed_m_per_s > 0
