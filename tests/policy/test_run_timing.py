"""Hardware-free checks of the policy step timing and the run summary, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_run_timing.py
"""
import json

import pytest

from policy.action_policy.mpc import MpcConfig
from policy.cone_filter.cone_filter import ConeFilterConfig
from policy.control.control import ControlConfig
from policy.data_logging.data_logging import log_timing, start_data_log, validate_parameters
from policy.data_logging.run_timing import RunTiming, StepTiming
from policy.memory import MpcStop
from policy.policy_runner import MovementPolicy


def test_steps_are_numbered_from_zero_and_a_new_run_starts_again():
    timing = RunTiming()
    first = timing.record(0.0, 0.004, None, 0.06)
    second = timing.record(0.04, 0.005, 0.04, 0.07)
    assert (first.policy_step, second.policy_step) == (0, 1)
    timing.start()
    assert timing.steps == []
    assert timing.record(0.0, 0.003, None, 0.05).policy_step == 0


def test_summary_gives_the_end_and_the_spread_of_each_duration():
    timing = RunTiming()
    # 20 steps 40 ms apart, each calculating in 2 ms except a 30 ms one at 0.4 s
    for step in range(20):
        timing.record(step * 0.04, 0.030 if step == 10 else 0.002, None if step == 0 else 0.04, 0.06)
    summary = timing.summary(0.9, "Operator request", "an obstacle")
    assert summary.startswith("Run summary: 0.9 s, 20 policy steps, ended by: Operator request")
    assert "the policy had stopped the car for an obstacle" in summary
    assert "calculation median 2.0 ms, p95 2.0 ms, max 30.0 ms (slowest at 0.4 s)" in summary
    assert "step interval median 40.0 ms, p95 40.0 ms, max 40.0 ms" in summary
    assert "trigger sample age median 60.0 ms" in summary


def test_summary_of_a_run_without_steps_or_trigger_ages():
    timing = RunTiming()
    assert timing.summary(0.2, "Required sensor wheel_speed is missing or stale") == (
        "Run summary: 0.2 s, 0 policy steps, ended by: Required sensor wheel_speed is missing or stale")
    # Timer mode has no triggering sensor, and the first step no interval
    timing.record(0.0, 0.001, None, None)
    summary = timing.summary(0.1, "Operator request")
    assert "step interval" not in summary and "trigger sample age" not in summary


def test_step_timing_survives_serialisation():
    step = StepTiming(3, 1.25, 0.0042, 0.039, None)
    assert StepTiming.deserialise(json.loads(json.dumps(step.serialise()))) == step


def test_each_step_is_appended_to_the_runs_timing_log(tmp_path):
    params = start_data_log(validate_parameters(True, str(tmp_path), "info", False))
    assert params.timing_log_path.name.endswith("_timing.json")
    timing = RunTiming()
    log_timing(timing.record(0.0, 0.003, None, 0.05), params)
    log_timing(timing.record(0.04, 0.004, 0.04, 0.06), params)
    logged = [StepTiming.deserialise(entry) for entry in json.loads(params.timing_log_path.read_text())]
    assert logged == timing.steps


def test_timing_is_not_logged_without_data_logging(tmp_path):
    params = start_data_log(validate_parameters(False, str(tmp_path), "info", False))
    log_timing(StepTiming(0, 0.0, 0.003, None, None), params)
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize("stop, reason", [
    (None, None),
    (MpcStop.OBSTACLE, "an obstacle"),
    (MpcStop.LANE_LOSS, "the lane ending or being lost"),
    (MpcStop.SOLVER_FAILURE, "a solver failure"),
], ids=["driving", "obstacle", "lane-loss", "solver-failure"])
def test_latched_stop_reason_names_the_mpcs_stop(stop, reason):
    policy = MovementPolicy(MpcConfig(), ConeFilterConfig(), ControlConfig())
    if stop is not None:
        policy.action_policy.memory.stop_mpc(stop)
    assert policy.latched_stop_reason(1.0) == reason


def test_latched_stop_reason_includes_the_run_duration():
    policy = MovementPolicy(MpcConfig(run_duration_s=5.0), ConeFilterConfig(), ControlConfig())
    assert policy.latched_stop_reason(4.9) is None
    assert policy.latched_stop_reason(5.0) == "reaching mpc.run_duration_s"
