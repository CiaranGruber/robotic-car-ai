"""Hardware-free checks of driving around obstacles in the lane, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_avoidance.py
"""
import math

import pytest

from policy.action_policy.avoidance import AvoidanceConfig, LaneAvoidance
from policy.action_policy.mpc import MpcConfig
from policy.cone_filter.cone_filter import ConeFilterConfig
from policy.control.control import ControlConfig
from policy.input_output import (CarObservations, ConeBatch, ConeColour, ConeDetection, LidarCartesian,
                                 LidarCartesianPoint, ObservedCarState, PolicyState, Position, SensorAge, WheelSpeed)
from policy.lane_detection.lanes import LaneArc, LaneDetection, LaneStatus
from policy.obstacle_detection.obstacle_detection import ObstacleDetectionConfig
from policy.obstacle_detection.obstacles import LaneObstacle
from policy.policy_runner import MovementPolicy

CAR_FRONT_M = ObstacleDetectionConfig().car_front_m


def straight_lane(width_m):
    """A straight lane centred on the car, as lane detection gives it."""
    return LaneDetection(LaneStatus.BOTH_EDGES, None, None, LaneArc(0.0, 0.0, 0.0, 0.0), width_m, 3.0, 0.0, 0.0, 0.0,
                         SensorAge(0.0, None))


def moving(first=False, wheel_speed=0.5):
    """Observations of a car moving at wheel_speed, 0.1 s after the previous step."""
    speed = None if wheel_speed is None else WheelSpeed(wheel_speed, SensorAge(0.0, None))
    return CarObservations(None, None, None, None, ObservedCarState(None, None, None, speed),
                           PolicyState(0.0 if first else 0.1, 0.0, first))


def obstacle(right_m, left_m, distance_m=1.5, far_distance_m=None):
    return LaneObstacle(distance_m, distance_m if far_distance_m is None else far_distance_m, right_m, left_m, 10)


def line(avoidance, obstacles, width_m=1.0, first=True, **observed):
    return avoidance.step(moving(first, **observed), straight_lane(width_m), obstacles, 0.0)


def test_clear_lane_keeps_the_usual_line():
    avoidance = LaneAvoidance(AvoidanceConfig(), CAR_FRONT_M)
    assert line(avoidance, []) is None
    # An object that leaves 0.1 m to the car's side at the centre needs no move
    assert line(avoidance, [obstacle(-0.5, -0.225)], first=False) is None


@pytest.mark.parametrize("right_m, left_m, expected_m", [
    (-0.30, -0.10, 0.125),  # On the right: pass on its left, with the car's right side 0.1 m from it
    (0.10, 0.30, -0.125),  # On the left: pass on its right
])
def test_moves_to_the_nearer_side_with_room(right_m, left_m, expected_m):
    # 1 m lane: the line may move up to 0.5 - 0.05 - 0.1 - 0.125 = 0.225 m from the centre
    assert line(LaneAvoidance(AvoidanceConfig(), CAR_FRONT_M), [obstacle(right_m, left_m)]) == pytest.approx(expected_m)


@pytest.mark.parametrize("width_m, right_m, left_m", [
    (0.8, -0.28, -0.08),  # Needs 0.145 m; the 0.8 m lane allows 0.125 m
    (1.0, -0.15, 0.15),  # Across the middle of the lane
])
def test_no_room_keeps_the_usual_line(width_m, right_m, left_m):
    assert line(LaneAvoidance(AvoidanceConfig(), CAR_FRONT_M), [obstacle(right_m, left_m)], width_m) is None


def test_keeps_the_line_until_the_rear_has_passed_the_obstacle():
    avoidance = LaneAvoidance(AvoidanceConfig(), CAR_FRONT_M)
    assert line(avoidance, [obstacle(-0.3, -0.1, 1.0, 1.2)]) == pytest.approx(0.125)
    # Out of view beside the car. The rear passes 0.3 m beyond the far side after 1.2 + 0.3 + 0.3 + 0.3 = 2.1 m,
    # 42 steps of 0.05 m
    lines = [line(avoidance, [], first=False) for _ in range(44)]
    assert all(value == pytest.approx(0.125) for value in lines[:41])
    assert lines[-1] is None


@pytest.mark.parametrize("steps_driven, expected_m", [
    (5, -0.175),  # 0.25 m on, still before the first box: it moves to the only open line, right of the second box
    (10, 0.125),  # 0.5 m on, beside the first box: it keeps the line rather than cross the lane into that box
])
def test_beside_an_obstacle_the_line_is_not_moved_across_the_lane(steps_driven, expected_m):
    avoidance = LaneAvoidance(AvoidanceConfig(), CAR_FRONT_M)
    assert line(avoidance, [obstacle(-0.3, -0.1, 0.5)]) == pytest.approx(0.125)
    for _ in range(steps_driven):
        line(avoidance, [], first=False)
    # A second box farther on blocks the line, and the lane centre
    assert line(avoidance, [obstacle(0.05, 0.3, 1.5)], first=False) == pytest.approx(expected_m)


def test_without_wheel_speed_the_line_is_kept():
    avoidance = LaneAvoidance(AvoidanceConfig(), CAR_FRONT_M)
    line(avoidance, [obstacle(-0.3, -0.1)])
    assert all(line(avoidance, None, first=False, wheel_speed=None) == pytest.approx(0.125) for _ in range(100))


def test_new_run_and_switching_off_forget_the_line():
    avoidance = LaneAvoidance(AvoidanceConfig(), CAR_FRONT_M)
    line(avoidance, [obstacle(-0.3, -0.1)])
    assert line(avoidance, [], first=True) is None
    assert line(LaneAvoidance(AvoidanceConfig(enabled=False), CAR_FRONT_M), [obstacle(-0.3, -0.1)]) is None


@pytest.mark.parametrize("change", [
    {"enabled": 1}, {"car_width_m": 0.0}, {"car_rear_m": -0.1}, {"cone_radius_m": math.nan},
    {"cone_clearance_m": -0.1}, {"obstacle_clearance_m": math.inf}, {"pass_distance_m": -0.1},
])
def test_invalid_settings_are_rejected(change):
    with pytest.raises(ValueError):
        AvoidanceConfig(**change)


def test_passing_line_must_clear_the_obstacle_check():
    # 0.125 + 0.05 = 0.175 m < corridor_half_width_m 0.20 m: the passing line would still count as blocked
    with pytest.raises(ValueError):
        MovementPolicy(MpcConfig(), ConeFilterConfig(), ControlConfig(),
                       avoidance_config=AvoidanceConfig(obstacle_clearance_m=0.05))


# ---------------------------------------------------------------------------------------------------------------
# Closed loop: the movement policy drives a simple car model along a straight lane past a box in it
# ---------------------------------------------------------------------------------------------------------------
BOX_X_M = 3.5
"""Distance along the lane from the car's starting origin to the box's near side."""
BOX_LENGTH_M = 0.2


def drive_past_box(lane_width_m, right_m, left_m, steps=140, dt=0.1):
    """Drives the car from the lane centre at 0.5 m/s towards a box across the lane from right_m to left_m.

    The car follows the MPC's curvature exactly; its speed coasts down with a 2.1 s time constant, like
    mpc.stop_distance_per_speed_s on car 9. The camera sees the cones from 0.4 to 3.5 m ahead within 40 degrees,
    and the lidar sees the box's outline.

    :return: The movement policy, and for each step the car's (x, y, heading) and the followed line.
    """
    config = MpcConfig(stop_distance_per_speed_s=2.1, obstacle_stop_distance_per_speed_s=2.1)
    policy = MovementPolicy(config, ConeFilterConfig(), ControlConfig())
    cones = [(x * 0.5, side * lane_width_m / 2.0, colour) for x in range(-2, 24)
             for side, colour in ((1.0, ConeColour.BLUE), (-1.0, ConeColour.YELLOW))]
    box = ([(BOX_X_M + i * 0.01, y) for i in range(round(BOX_LENGTH_M / 0.01) + 1) for y in (right_m, left_m)]
           + [(x, right_m + i * 0.01) for i in range(round((left_m - right_m) / 0.01) + 1)
              for x in (BOX_X_M, BOX_X_M + BOX_LENGTH_M)])
    x = y = heading = 0.0
    speed = config.target_speed_m_per_s
    trace = []
    for step in range(steps):
        def in_car(px, py):
            dx, dy = px - x, py - y
            return (math.cos(heading) * dx + math.sin(heading) * dy, -math.sin(heading) * dx + math.cos(heading) * dy)
        seen = [(in_car(px, py), colour) for px, py, colour in cones]
        seen = [ConeDetection(Position(cx, cy, 0.1), colour, 0.9) for (cx, cy), colour in seen
                if 0.4 < cx < 3.5 and abs(math.atan2(cy, cx)) < math.radians(40)]
        returns = sorted((in_car(px, py) for px, py in box), key=lambda point: math.atan2(point[1], point[0]))
        lidar = LidarCartesian([LidarCartesianPoint(Position(cx, cy, 0.12), index)
                                for index, (cx, cy) in enumerate(returns) if cx > 0.2],
                               SensorAge(0.0, None), "base_link")
        observations = CarObservations(ConeBatch(seen, SensorAge(0.0, None), 0.0), None, None, lidar,
                                       ObservedCarState(None, None, None, WheelSpeed(speed, SensorAge(0.0, None))),
                                       PolicyState(0.0 if step == 0 else dt, step * dt, step == 0))
        policy.step(observations)
        mpc = policy.action_policy
        trace.append((x, y, heading, mpc.avoidance_offset_m))
        commanded = mpc.previous_speed_m_per_s
        speed = commanded if commanded >= speed else speed + (commanded - speed) * dt / 2.1
        heading += speed * mpc.previous_curvature_per_m * dt
        x += speed * math.cos(heading) * dt
        y += speed * math.sin(heading) * dt
    return policy, trace


def test_car_drives_around_a_box_with_room_beside_it_and_returns_to_the_centre():
    # 1 m lane, box on the right: the line moves 0.125 m left
    policy, trace = drive_past_box(1.0, -0.30, -0.10)
    assert not policy.action_policy.stopped_for_obstacle
    car = AvoidanceConfig()
    beside = [y for x, y, _, _ in trace if x + CAR_FRONT_M > BOX_X_M and x - car.car_rear_m < BOX_X_M + BOX_LENGTH_M]
    assert beside, "the car did not reach the box"
    # The car's right side stays clear of the box, and its left side clear of the cones
    assert min(beside) - car.car_width_m / 2.0 > -0.10 + 0.05
    assert max(y for _, y, _, _ in trace) + car.car_width_m / 2.0 < 0.5 - car.cone_radius_m
    assert any(offset is not None for _, _, _, offset in trace)
    # Back on the centre line once past
    assert trace[-1][3] is None and abs(trace[-1][1]) < 0.05


def test_car_stops_before_a_box_without_room_beside_it():
    # 0.8 m lane: the line could only move 0.125 m, and the box needs 0.145 m
    policy, trace = drive_past_box(0.8, -0.28, -0.08)
    assert policy.action_policy.stopped_for_obstacle
    assert all(offset is None for _, _, _, offset in trace)
    final_x = trace[-1][0]
    assert final_x + CAR_FRONT_M < BOX_X_M
