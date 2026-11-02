"""Hardware-free checks of the car memory and the MPC state kept in it, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_memory.py
"""
import dataclasses
import json

import numpy as np
import pytest

from policy.action_policy.avoidance import AvoidanceConfig, LaneAvoidance
from policy.action_policy.mpc import MpcConfig, MpcSolution
from policy.action_policy.policy import MpcPolicy
from policy.data_logging.data_logging import log_memory, start_data_log, validate_parameters
from policy.input_output import CarObservations, ConeColour, ObservedCarState, PolicyState, SensorAge, WheelSpeed
from policy.lane_detection.lanes import LaneArc, LaneDetection, LaneStatus
from policy.intersection_state import IntersectionMemory
from policy.markers.commands import MarkerCommand
from policy.memory import CarMemory, CarPose, MpcPlan, MpcStop, RememberedCone
from policy.obstacle_detection.obstacle_detection import ObstacleDetectionConfig
from policy.obstacle_detection.obstacles import LaneObstacle
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
    assert (old.pose, old.imu_heading_rad, old.cones, old.lane_obstacles) == (None, None, (), ())


def test_memory_with_map_obstacles_and_intersection_survives_serialising():
    memory = CarMemory()
    memory.start_map(0.3)
    memory.move_car(CarPose(1.0, -0.2, 0.1), 0.4)
    memory.remember_cones([RememberedCone(1.5, 0.5, ConeColour.BLUE, 4, 2.0, 0.9)])
    memory.remember_lane_obstacles([LaneObstacle(-0.2, 0.0, -0.3, -0.1, 12)])
    memory.command_intersection(MarkerCommand.TURN_RIGHT, 2.0)
    memory.update_intersection(IntersectionMemory(command=MarkerCommand.TURN_RIGHT))

    data = json.loads(json.dumps(memory.data.serialise()))

    assert data["cones"][0]["colour"] == "BLUE"
    assert CarMemory.deserialise(data).data == memory.data
    # A new map keeps the obstacles, which the avoidance forgets on its own at a new run
    memory.start_map(None)
    assert memory.data.pose == CarPose(0.0, 0.0, 0.0) and memory.data.cones == ()
    assert len(memory.data.lane_obstacles) == 1


def test_the_avoidance_keeps_the_obstacles_beside_the_car_in_the_memory_it_is_given():
    avoidance, memory = LaneAvoidance(AvoidanceConfig(), ObstacleDetectionConfig().car_front_m), CarMemory()
    lane = LaneDetection(LaneStatus.BOTH_EDGES, None, None, LaneArc(0.0, 0.0, 0.0, 0.0), 1.0, 3.0, 0.0, 0.0, 0.0,
                         SensorAge(0.0, None))
    near = LaneObstacle(0.2, 0.4, -0.3, -0.1, 10)
    moving = CarObservations(None, None, None, None,
                             ObservedCarState(None, None, None, WheelSpeed(0.5, SensorAge(0.0, None))),
                             PolicyState(0.0, 0.0, True))
    avoidance.step(moving, lane, [near], 0.0, memory)
    assert memory.data.lane_obstacles == (near,) and avoidance.remembered == [near]
    # Moved back by the distance driven once the lidar no longer sees it
    avoidance.step(dataclasses.replace(moving, policy=PolicyState(0.1, 0.1, False)), lane, [], 0.0, memory)
    assert memory.data.lane_obstacles[0].distance_m == pytest.approx(0.15)
    # A new run forgets it
    avoidance.step(moving, lane, [], 0.0, memory)
    assert memory.data.lane_obstacles == ()


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
