"""Hardware-free checks of the car memory and the MPC state kept in it, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_memory.py
"""
import dataclasses
import json

import numpy as np
import pytest

from policy.action_policy.mpc import MpcConfig, MpcSolution
from policy.action_policy.policy import MpcPolicy
from policy.data_logging.data_logging import log_memory, start_data_log, validate_parameters
from policy.input_output import ObservedCarState, PolicyState, SensorAge, WheelSpeed
from policy.memory import CarMemory, MpcPlan, MpcStop
from tests.policy.test_mpc import lane_cones, observe


def test_memory_with_a_stop_and_a_plan_survives_serialising():
    memory = CarMemory()
    memory.plan_mpc(np.array([0.1, -0.2]), 0.05)
    memory.drive_along_mpc_plan(0.03)
    memory.stop_mpc(MpcStop.OBSTACLE)
    memory.step()

    data = json.loads(json.dumps(memory.data.serialise()))

    assert data["mpc_stop"] == "OBSTACLE"
    assert CarMemory.deserialise(data).data == memory.data
    assert memory.data.mpc_plan == MpcPlan((0.1, -0.2), 0.05, 0.03)


def test_memory_logged_before_the_mpc_kept_its_state_still_loads():
    old = CarMemory.deserialise({"policy_step": 7}).data
    assert (old.policy_step, old.mpc_stop, old.mpc_plan) == (7, None, None)


def test_the_mpc_keeps_its_plan_and_stop_in_the_memory_it_is_given(monkeypatch):
    policy, memory = MpcPolicy(MpcConfig()), CarMemory()
    moving = observe(lane_cones(0.2, 0.0), first_step=True)
    moving = dataclasses.replace(moving, car=ObservedCarState(None, None, None, WheelSpeed(0.5, SensorAge(0.0, None))))
    policy.run_policy(moving, None, memory=memory)
    plan = memory.data.mpc_plan
    assert plan is not None and plan.step_distance_m == pytest.approx(0.05) and plan.distance_driven_m == 0.0
    assert plan.curvatures_per_m == policy.plan_curvatures_per_m

    # Followed while the lane is lost: 0.1 s at 0.5 m/s along the plan
    policy.run_policy(dataclasses.replace(moving, cones=None, policy=PolicyState(0.1, 0.1, False)), None,
                      memory=memory)
    assert memory.data.mpc_plan.distance_driven_m == pytest.approx(0.05)

    monkeypatch.setattr(policy.controller, "solve", lambda *args, **kwargs: MpcSolution(np.ones(12), False, 0.001))
    policy.run_policy(dataclasses.replace(moving, policy=PolicyState(0.1, 0.2, False)), None, memory=memory)
    assert (memory.data.mpc_stop, memory.data.mpc_plan) == (MpcStop.SOLVER_FAILURE, None)

    # A new run forgets both
    monkeypatch.undo()
    policy.run_policy(moving, None, memory=memory)
    assert memory.data.mpc_stop is None and memory.data.mpc_plan is not None


def test_the_memory_log_shows_why_the_mpc_stopped(tmp_path):
    policy, memory = MpcPolicy(MpcConfig()), CarMemory()
    params = start_data_log(validate_parameters(True, str(tmp_path), "info", False))
    for step, distance_m in enumerate((0.45, 0.45, 0.45)):
        log_memory(memory, params)
        policy.run_policy(observe(lane_cones(0.0, 0.0), first_step=step == 0), None, distance_m, memory)
        memory.step()

    logged = json.loads(params.memory_log_path.read_text())
    # Each record is the memory as the step began: the obstacle stop is confirmed in the second step
    assert [record["mpc_stop"] for record in logged] == [None, None, "OBSTACLE"]
