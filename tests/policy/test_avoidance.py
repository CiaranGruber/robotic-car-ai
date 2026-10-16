"""Hardware-free checks of driving around obstacles in the lane, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_avoidance.py
"""
import math

import pytest

from policy.action_policy.avoidance import AvoidanceConfig, LaneAvoidance
from policy.action_policy.lane_path import LanePath, PathSegment
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
CAR = AvoidanceConfig()
CAR_LENGTH_M = CAR_FRONT_M + CAR.car_rear_m


def lane(width_m, curvature_per_m=0.0):
    """A lane centred on the car, as lane detection gives it."""
    return LaneDetection(LaneStatus.BOTH_EDGES, None, None, LaneArc(0.0, 0.0, 0.0, curvature_per_m), width_m, 3.0,
                         0.0, 0.0, curvature_per_m, SensorAge(0.0, None))


def moving(first=False, wheel_speed=0.5):
    """Observations of a car moving at wheel_speed, 0.1 s after the previous step."""
    speed = None if wheel_speed is None else WheelSpeed(wheel_speed, SensorAge(0.0, None))
    return CarObservations(None, None, None, None, ObservedCarState(None, None, None, speed),
                           PolicyState(0.0 if first else 0.1, 0.0, first))


def obstacle(right_m, left_m, distance_m=2.5, far_distance_m=None):
    return LaneObstacle(distance_m, distance_m if far_distance_m is None else far_distance_m, right_m, left_m, 10)


def plan(avoidance, obstacles, width_m=1.0, first=True, curvature_per_m=0.0, **observed):
    return avoidance.step(moving(first, **observed), lane(width_m, curvature_per_m), obstacles, 0.0)


def beside(path, item):
    """The line's offsets while the car's body is beside the obstacle, from base_link's origin."""
    return path.offset_range(item.distance_m, item.far_distance_m + CAR_LENGTH_M)


def ramp_m(change_m, curvature_per_m=CAR.max_ramp_curvature_per_m):
    return math.pi * math.sqrt(abs(change_m) / (2.0 * curvature_per_m))


def test_path_offsets_follow_its_segments():
    path = LanePath(0.0, (PathSegment(1.0, 2.0, 0.0, 0.2), PathSegment(2.0, 3.0, 0.2, 0.2)))
    assert (path.offset_at(0.5), path.offset_at(1.5), path.offset_at(2.5), path.offset_at(4.0)) == pytest.approx(
        (0.0, 0.1, 0.2, 0.0))
    assert path.offset_range(0.0, 2.5) == pytest.approx((0.0, 0.2))
    assert path.offset_range(1.2, 1.4) == pytest.approx((path.offset_at(1.2), path.offset_at(1.4)))


def test_clear_lane_keeps_the_usual_line():
    avoidance = LaneAvoidance(CAR, CAR_FRONT_M)
    assert plan(avoidance, []).segments == ()
    # Within the cones, which bound the lane anyway
    assert plan(avoidance, [obstacle(-0.5, -0.46)], first=False).segments == ()


@pytest.mark.parametrize("right_m, left_m, expected_m", [
    (-0.30, -0.10, 0.175),  # On the right: the middle of the gap from it to the left cones' inner side (0.45 m)
    (0.10, 0.30, -0.175),  # On the left: pass on its right
    (0.05, 0.50, -0.20),  # Reaching in from the left cones: the middle of the gap from it to the right cones
])
def test_passes_through_the_middle_of_the_free_gap(right_m, left_m, expected_m):
    item = obstacle(right_m, left_m)
    path = plan(LaneAvoidance(CAR, CAR_FRONT_M), [item])
    assert beside(path, item) == pytest.approx((expected_m, expected_m))
    # The car's sides stay clear of the obstacle and the cones by the clearances
    half_width_m = CAR.car_width_m / 2.0
    assert expected_m - half_width_m >= left_m + CAR.obstacle_clearance_m or (
        expected_m + half_width_m <= right_m - CAR.obstacle_clearance_m)
    assert abs(expected_m) + half_width_m <= 0.5 - CAR.cone_radius_m - CAR.cone_clearance_m
    # Still on the usual line beside the car: the ramp starts later
    assert path.offset_at(0.0) == 0.0


def test_ramps_are_smooth_and_within_the_curvature_limit():
    item = obstacle(-0.30, -0.10)
    path = plan(LaneAvoidance(CAR, CAR_FRONT_M), [item])
    start_m = item.distance_m - CAR.pass_distance_m
    end_m = item.far_distance_m + CAR_LENGTH_M + CAR.pass_distance_m
    length_m = ramp_m(0.175)
    assert path.offset_at(start_m - length_m) == pytest.approx(0.0)
    assert path.offset_at(end_m + length_m) == pytest.approx(0.0)
    step_m = 0.01
    offsets = [path.offset_at(i * step_m) for i in range(round(6.0 / step_m))]
    assert max(abs(b - a) for a, b in zip(offsets, offsets[1:])) < 0.002
    curvatures = [(a - 2.0 * b + c) / step_m ** 2 for a, b, c in zip(offsets, offsets[1:], offsets[2:])]
    assert max(abs(value) for value in curvatures) <= CAR.max_ramp_curvature_per_m * 1.01


@pytest.mark.parametrize("width_m, right_m, left_m", [
    (0.8, -0.28, -0.08),  # A 0.43 m gap; the car needs 0.25 + 0.10 + 0.10 = 0.45 m
    (1.0, -0.15, 0.15),  # Across the middle of the lane
])
def test_no_room_keeps_the_usual_line(width_m, right_m, left_m):
    assert plan(LaneAvoidance(CAR, CAR_FRONT_M), [obstacle(right_m, left_m)], width_m).segments == ()


def test_passes_on_the_side_with_more_room():
    item = obstacle(-0.05, 0.15)
    # 1.4 m lane: 0.60 m on its right, 0.50 m on its left
    assert beside(plan(LaneAvoidance(CAR, CAR_FRONT_M), [item], 1.4), item) == pytest.approx((-0.35, -0.35))


def test_side_changes_only_for_clearly_more_room():
    avoidance = LaneAvoidance(CAR, CAR_FRONT_M)
    first = obstacle(-0.15, 0.05)
    assert beside(plan(avoidance, [first], 1.4), first)[0] == pytest.approx(0.35)
    # Now 0.02 m more room on its right: within side_hysteresis_m, so the car keeps passing on its left
    nearly = obstacle(-0.09, 0.11)
    assert beside(plan(avoidance, [nearly], 1.4, first=False, wheel_speed=0.0), nearly)[0] == pytest.approx(0.38)
    clearly = obstacle(-0.05, 0.15)
    assert beside(plan(avoidance, [clearly], 1.4, first=False, wheel_speed=0.0), clearly)[0] == pytest.approx(-0.35)


def test_keeps_the_line_beside_the_obstacle_until_the_rear_has_passed_it():
    avoidance = LaneAvoidance(CAR, CAR_FRONT_M)
    item = obstacle(-0.30, -0.10, 0.2, 0.4)
    assert plan(avoidance, [item]).offset_at(0.0) == pytest.approx(0.175)
    # Out of the lidar's view beside the car: 0.05 m per step. base_link leaves the line beside it once it is
    # 0.4 + 0.6 + 0.3 = 1.3 m on, and is back on the usual line a ramp later
    lines = [plan(avoidance, [], first=False).offset_at(0.0) for _ in range(70)]
    held = round(1.3 / 0.05)
    assert lines[:held] == pytest.approx([0.175] * held)
    assert all(b <= a for a, b in zip(lines[held:], lines[held + 1:]))
    back = math.ceil((1.3 + ramp_m(0.175)) / 0.05)
    assert lines[back:] == pytest.approx([0.0] * (len(lines) - back))


def test_beside_an_obstacle_the_line_is_held_rather_than_moved_into_it():
    avoidance = LaneAvoidance(CAR, CAR_FRONT_M)
    plan(avoidance, [obstacle(-0.15, 0.05, 0.2, 0.4)], 1.4)
    for _ in range(10):
        path = plan(avoidance, [], 1.4, first=False)
    assert path.offset_at(0.0) == pytest.approx(0.35)
    # The lane now looks narrower, leaving no room on either side: keep the line, rather than drive back into it
    assert plan(avoidance, [], 1.0, first=False).offset_at(0.0) == pytest.approx(0.35)


def test_without_wheel_speed_the_line_is_kept():
    avoidance = LaneAvoidance(CAR, CAR_FRONT_M)
    plan(avoidance, [obstacle(-0.3, -0.1, 0.2, 0.4)])
    assert all(plan(avoidance, None, first=False, wheel_speed=None).offset_at(0.0) == pytest.approx(0.175)
               for _ in range(100))


def test_new_run_and_switching_off_forget_the_line():
    avoidance = LaneAvoidance(CAR, CAR_FRONT_M)
    plan(avoidance, [obstacle(-0.3, -0.1, 0.2, 0.4)])
    assert plan(avoidance, [], first=True).segments == ()
    assert plan(LaneAvoidance(AvoidanceConfig(enabled=False), CAR_FRONT_M), [obstacle(-0.3, -0.1)]) is None


def test_obstacles_close_together_share_one_line():
    near, far = obstacle(-0.30, -0.10, 1.0, 1.2), obstacle(-0.35, -0.05, 1.5, 1.7)
    path = plan(LaneAvoidance(CAR, CAR_FRONT_M), [near, far])
    # The narrower gap, beside the second, sets the line for both: the middle of -0.05 to 0.45
    assert path.offset_range(near.distance_m, far.far_distance_m + CAR_LENGTH_M) == pytest.approx((0.20, 0.20))


def test_obstacles_on_alternate_sides_give_an_s_shaped_line_when_the_car_can_turn_that_sharply():
    first, second = obstacle(-0.30, -0.10, 0.3), obstacle(0.10, 0.30, 2.5)
    path = plan(LaneAvoidance(AvoidanceConfig(max_ramp_curvature_per_m=2.0), CAR_FRONT_M), [first, second])
    assert beside(path, first) == pytest.approx((0.175, 0.175))
    assert beside(path, second) == pytest.approx((-0.175, -0.175))
    # Too sharp for the default ramps, and no one line passes both: only the first is passed, and the MPC stops for
    # the second if it is in the car's way
    path = plan(LaneAvoidance(CAR, CAR_FRONT_M), [first, second])
    assert beside(path, first) == pytest.approx((0.175, 0.175))
    assert beside(path, second)[0] >= 0.0


@pytest.mark.parametrize("curvature_per_m, plans", [
    (0.5, False),  # Turning left as sharply as the car can (0.48 1/m): no room for a ramp
    (-0.5, True),  # Turning right, where the car can reach 0.92 1/m
])
def test_ramps_keep_within_the_cars_limits_on_curves(curvature_per_m, plans):
    avoidance = LaneAvoidance(CAR, CAR_FRONT_M, max_left_curvature_per_m=0.48, max_right_curvature_per_m=0.92)
    path = plan(avoidance, [obstacle(-0.30, -0.10)], curvature_per_m=curvature_per_m)
    assert bool(path.segments) == plans


@pytest.mark.parametrize("change", [
    {"enabled": 1}, {"car_width_m": 0.0}, {"car_rear_m": -0.1}, {"cone_radius_m": math.nan},
    {"cone_clearance_m": -0.1}, {"obstacle_clearance_m": math.inf}, {"pass_distance_m": -0.1},
    {"max_ramp_curvature_per_m": 0.0}, {"side_hysteresis_m": -0.1}, {"lookahead_m": math.nan},
])
def test_invalid_settings_are_rejected(change):
    with pytest.raises(ValueError):
        AvoidanceConfig(**change)


def test_passing_line_must_clear_the_obstacle_check():
    # 0.125 + 0.05 = 0.175 m < corridor_half_width_m 0.20 m: the passing line would still count as blocked
    with pytest.raises(ValueError):
        MovementPolicy(MpcConfig(), ConeFilterConfig(), ControlConfig(),
                       avoidance_config=AvoidanceConfig(obstacle_clearance_m=0.05))


def test_ramps_must_be_within_the_cars_curvature_limits():
    with pytest.raises(ValueError):
        MovementPolicy(MpcConfig(max_curvature_per_m=0.48), ConeFilterConfig(), ControlConfig(),
                       avoidance_config=AvoidanceConfig(max_ramp_curvature_per_m=0.5))
    MovementPolicy(MpcConfig(max_curvature_per_m=0.48), ConeFilterConfig(), ControlConfig(),
                   avoidance_config=AvoidanceConfig(enabled=False, max_ramp_curvature_per_m=0.5))


# ---------------------------------------------------------------------------------------------------------------
# Closed loop: the movement policy drives a simple car model along a straight lane past boxes in it
# ---------------------------------------------------------------------------------------------------------------
BOX_LENGTH_M = 0.2


def drive_past(lane_width_m, boxes, steps=140, dt=0.1):
    """Drives the car from the lane centre at 0.5 m/s past boxes in the lane.

    The car follows the MPC's curvature exactly; its speed coasts down with a 2.1 s time constant, like
    mpc.stop_distance_per_speed_s on car 9. The camera sees the cones from 0.4 to 3.5 m ahead within 40 degrees,
    and the lidar sees the boxes' outlines.

    :param boxes: For each box, the distance along the lane from the car's starting origin to its near side, and its
        right and left sides across the lane.
    :return: The movement policy, and for each step the car's (x, y, heading) and the followed line.
    """
    config = MpcConfig(stop_distance_per_speed_s=2.1, obstacle_stop_distance_per_speed_s=2.1)
    policy = MovementPolicy(config, ConeFilterConfig(), ControlConfig())
    cones = [(x * 0.5, side * lane_width_m / 2.0, colour) for x in range(-2, 40)
             for side, colour in ((1.0, ConeColour.BLUE), (-1.0, ConeColour.YELLOW))]
    outline = []
    for box_x, right_m, left_m in boxes:
        outline += ([(box_x + i * 0.01, y) for i in range(round(BOX_LENGTH_M / 0.01) + 1) for y in (right_m, left_m)]
                    + [(x, right_m + i * 0.01) for i in range(round((left_m - right_m) / 0.01) + 1)
                       for x in (box_x, box_x + BOX_LENGTH_M)])
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
        returns = sorted((in_car(px, py) for px, py in outline), key=lambda point: math.atan2(point[1], point[0]))
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


def sides_beside(trace, box_x):
    """The car's right and left sides at each step its body is beside the box at box_x."""
    half_width_m = CAR.car_width_m / 2.0
    return [(y - half_width_m, y + half_width_m) for x, y, _, _ in trace
            if x + CAR_FRONT_M > box_x and x - CAR.car_rear_m < box_x + BOX_LENGTH_M]


@pytest.mark.parametrize("lane_width_m, right_m, left_m", [
    (1.0, -0.30, -0.10),  # The line moves to 0.175 m, 0.15 m from the box and from the cones
    (0.8, -0.38, -0.12),  # The lab's lane: a 0.47 m gap leaves 0.11 m either side
], ids=["1m-lane", "0.8m-lane"])
def test_car_drives_around_a_box_with_room_beside_it_and_returns_to_the_line(lane_width_m, right_m, left_m):
    policy, trace = drive_past(lane_width_m, [(3.5, right_m, left_m)])
    assert not policy.action_policy.stopped_for_obstacle
    sides = sides_beside(trace, 3.5)
    assert sides, "the car did not reach the box"
    # The car's right side stays clear of the box, and its left side clear of the cones' inner side, by nearly the
    # clearances (the MPC follows the line within 1 cm)
    assert min(right for right, _ in sides) > left_m + CAR.obstacle_clearance_m - 0.01
    assert max(left for _, left in sides) < lane_width_m / 2.0 - CAR.cone_radius_m - CAR.cone_clearance_m + 0.01
    assert any(offset is not None for _, _, _, offset in trace)
    # Back on the centre line once past
    assert abs(trace[-1][1]) < 0.05 and (trace[-1][3] is None or abs(trace[-1][3]) < 1e-9)


def test_car_stops_before_a_box_without_room_beside_it():
    # 0.8 m lane: a 0.43 m gap beside the box, and the car needs 0.45 m
    policy, trace = drive_past(0.8, [(3.5, -0.28, -0.08)])
    assert policy.action_policy.stopped_for_obstacle
    assert all(offset is None for _, _, _, offset in trace)
    assert trace[-1][0] + CAR_FRONT_M < 3.5


def test_car_weaves_between_boxes_on_alternate_sides():
    boxes = [(3.5, -0.30, -0.10), (8.5, 0.10, 0.30)]
    policy, trace = drive_past(1.0, boxes, steps=240)
    assert not policy.action_policy.stopped_for_obstacle
    assert min(right for right, _ in sides_beside(trace, 3.5)) > -0.10 + CAR.obstacle_clearance_m - 0.01
    assert max(left for _, left in sides_beside(trace, 8.5)) < 0.10 - CAR.obstacle_clearance_m + 0.01
    assert trace[-1][0] > 8.5 + BOX_LENGTH_M


def test_car_stops_for_a_second_box_too_close_to_move_across_for():
    # Seen 2.5 m ahead, with 1.25 m to stop: too little to ramp 0.29 m across, no sharper than 0.3 1/m
    policy, trace = drive_past(1.0, [(3.5, -0.30, -0.10), (8.0, 0.10, 0.30)], steps=240)
    assert policy.action_policy.stopped_for_obstacle
    assert trace[-1][0] > 3.5 + BOX_LENGTH_M
    assert trace[-1][0] + CAR_FRONT_M < 8.0
