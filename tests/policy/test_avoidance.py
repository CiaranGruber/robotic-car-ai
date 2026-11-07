"""Hardware-free checks of driving around obstacles in the lane, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_avoidance.py
"""
import math

import numpy as np
import pytest

from policy.action_policy.avoidance import AvoidanceConfig, LaneAvoidance
from policy.action_policy.lane_path import LanePath, LineState, PathSegment, move_length_m
from policy.action_policy.mpc import MpcConfig
from policy.cone_filter.cone_filter import ConeFilterConfig
from policy.control.control import ControlConfig
from policy.input_output import (CarObservations, ConeBatch, ConeColour, ConeDetection, LidarCartesian,
                                 LidarCartesianPoint, ObservedCarState, PolicyState, Position, SensorAge, WheelSpeed)
from policy.lane_detection.lanes import LaneArc, LaneDetection, LaneStatus
from policy.memory import CarMemory
from policy.obstacle_detection.obstacle_detection import ObstacleDetectionConfig
from policy.obstacle_detection.obstacles import LaneObstacle
from policy.policy_runner import MovementPolicy

CAR_FRONT_M = ObstacleDetectionConfig().car_front_m
CAR = AvoidanceConfig()
CAR_LENGTH_M = CAR_FRONT_M + CAR.car_rear_m
HALF_WIDTH_M = CAR.car_width_m / 2.0


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


def approach(avoidance, item, width_m=1.0, steps=200):
    """Drives at 0.05 m per step towards an obstacle first seen 2.5 m ahead; the lidar loses it 0.25 m ahead.

    :return: For each step, how far the car's front has driven and the planned line.
    """
    lines = []
    for step in range(steps):
        driven_m = 0.05 * step
        ahead = obstacle(item.right_m, item.left_m, item.distance_m - driven_m, item.far_distance_m - driven_m)
        lines.append((driven_m, plan(avoidance, [ahead] if ahead.distance_m > 0.25 else [], width_m,
                                     first=step == 0)))
    return lines


# ---------------------------------------------------------------------------------------------------------------
# The line: quintic polynomials in the lane's Frenet frame
# ---------------------------------------------------------------------------------------------------------------
def values(state):
    return state.offset_m, state.slope, state.curvature_per_m


def test_a_move_matches_the_offset_slope_and_curvature_at_both_ends():
    start, end = LineState(0.1, -0.2, 0.3), LineState(-0.05, 0.1, -0.2)
    move = PathSegment.move(1.0, 2.5, start, end)
    assert values(move.state_at(1.0)) == pytest.approx(values(start))
    assert values(move.state_at(2.5)) == pytest.approx(values(end))
    assert PathSegment.hold(0.0, 1.0, 0.2).state_at(0.4) == LineState(0.2)


def test_the_shortest_move_is_as_sharp_as_allowed():
    move = PathSegment.move(0.0, move_length_m(0.2, 0.3), LineState(0.0), LineState(0.2))
    # Found from 41 points along it, which can miss the very peak slightly
    assert move.peak_curvature_per_m() == pytest.approx(0.3, rel=0.01)
    assert move_length_m(0.0, 0.3) == 0.0 and move_length_m(0.1, 0.0) == math.inf


def test_path_offsets_follow_its_segments():
    path = LanePath(0.0, (PathSegment.move(1.0, 2.0, LineState(0.0), LineState(0.2)), PathSegment.hold(2.0, 3.0, 0.2)))
    assert (path.offset_at(0.5), path.offset_at(1.5), path.offset_at(2.5), path.offset_at(4.0)) == pytest.approx(
        (0.0, 0.1, 0.2, 0.0))
    assert path.offset_range(0.0, 2.5) == pytest.approx((0.0, 0.2))
    moved = path.moved_back(1.2, 0.0)
    assert all(moved.offset_at(s_m) == pytest.approx(path.offset_at(s_m + 1.2)) for s_m in np.arange(0.0, 3.0, 0.1))
    assert values(moved.state_at(0.0)) == pytest.approx(values(path.state_at(1.2)))


def test_the_turned_body_is_checked_beside_the_obstacle():
    avoidance = LaneAvoidance(CAR, CAR_FRONT_M)
    item = obstacle(-0.30, -0.10, 1.0, 1.2)
    on_line = LanePath(0.175, ())
    assert not avoidance.in_way(on_line, item, CAR.obstacle_clearance_m, CAR.obstacle_clearance_m)
    assert avoidance.in_way(on_line, item, 0.2, 0.2)
    # Moving right across the lane while beside it: the rear swings left, but the front turns into it
    across = LanePath(0.175, (PathSegment.move(1.0, 2.0, LineState(0.175), LineState(-0.175)),
                              PathSegment.hold(2.0, 5.0, -0.175)))
    assert avoidance.in_way(across, item, 0.0, 0.0)


# ---------------------------------------------------------------------------------------------------------------
# One step's plan
# ---------------------------------------------------------------------------------------------------------------
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
    assert expected_m - HALF_WIDTH_M >= left_m + CAR.obstacle_clearance_m or (
        expected_m + HALF_WIDTH_M <= right_m - CAR.obstacle_clearance_m)
    assert abs(expected_m) + HALF_WIDTH_M <= 0.5 - CAR.cone_radius_m - CAR.cone_clearance_m
    # Still on the usual line beside the car: the move starts later
    assert path.offset_at(0.0) == 0.0


def test_moves_are_smooth_and_within_the_curvature_limit():
    item = obstacle(-0.30, -0.10)
    path = plan(LaneAvoidance(CAR, CAR_FRONT_M), [item])
    s_m = np.arange(0.0, 8.0, 0.005)
    offsets, slopes, curvatures = path.sample(s_m)
    # Offset, slope and curvature all change continuously, from and back to the usual line
    assert offsets[0] == offsets[-1] == 0.0
    assert np.max(np.abs(np.diff(offsets))) < 0.001
    assert np.max(np.abs(np.diff(slopes))) < 0.002
    assert np.max(np.abs(np.diff(curvatures))) < 0.02
    assert np.max(np.abs(curvatures)) <= CAR.max_ramp_curvature_per_m * 1.001


@pytest.mark.parametrize("width_m, right_m, left_m", [
    (0.8, -0.28, -0.08),  # A 0.43 m gap; the car needs 0.25 + 0.10 + 0.10 = 0.45 m
    (1.0, -0.15, 0.15),  # Across the middle of the lane
])
def test_no_room_keeps_the_usual_line(width_m, right_m, left_m):
    assert plan(LaneAvoidance(CAR, CAR_FRONT_M), [obstacle(right_m, left_m)], width_m).segments == ()


def test_passes_on_the_side_that_needs_the_smaller_move():
    item = obstacle(-0.05, 0.15)
    # 1.4 m lane: 0.60 m on its right, 0.50 m on its left
    lowest_m, highest_m = beside(plan(LaneAvoidance(CAR, CAR_FRONT_M), [item], 1.4), item)
    assert -0.65 + HALF_WIDTH_M + CAR.cone_clearance_m <= lowest_m <= highest_m <= (
        item.right_m - HALF_WIDTH_M - CAR.obstacle_clearance_m)


def test_keeps_its_side_while_that_side_stays_clear():
    avoidance = LaneAvoidance(CAR, CAR_FRONT_M)
    first = obstacle(-0.15, 0.05)
    assert beside(plan(avoidance, [first], 1.4), first)[0] > 0.0
    # Now more room on its right, but the left still leaves the clearances: the car keeps passing on its left
    for item in (obstacle(-0.09, 0.11), obstacle(-0.05, 0.15)):
        lowest_m, _ = beside(plan(avoidance, [item], 1.4, first=False, wheel_speed=0.0), item)
        assert lowest_m - HALF_WIDTH_M >= item.left_m + CAR.obstacle_clearance_m - 1e-6
    # The left no longer leaves them: the car changes to the right
    blocked = obstacle(-0.05, 0.25)
    assert beside(plan(avoidance, [blocked], 1.4, first=False, wheel_speed=0.0), blocked)[1] < 0.0


def test_a_face_seen_edge_on_still_counts():
    # The lidar sees the side of a box along the lane as returns with no width across it
    item = obstacle(-0.10, -0.10)
    assert beside(plan(LaneAvoidance(CAR, CAR_FRONT_M), [item]), item)[0] >= (
        -0.10 + HALF_WIDTH_M + CAR.obstacle_clearance_m - 1e-6)


def test_pieces_of_one_object_are_passed_as_one():
    pieces = [obstacle(-0.30, -0.20), obstacle(-0.19, -0.10, 2.52, 2.7)]
    path = plan(LaneAvoidance(CAR, CAR_FRONT_M), pieces)
    assert beside(path, obstacle(-0.30, -0.10, 2.5, 2.7)) == pytest.approx((0.175, 0.175))


def test_keeps_clear_of_the_obstacle_until_the_rear_has_passed_it():
    item = obstacle(-0.30, -0.10, 2.5, 2.7)
    avoidance = LaneAvoidance(CAR, CAR_FRONT_M)
    lines = approach(avoidance, item)
    beside_car = [(driven_m, path) for driven_m, path in lines
                  if item.distance_m <= driven_m <= item.far_distance_m + CAR_LENGTH_M]
    assert beside_car
    # Out of the lidar's view beside the car, the remembered obstacle is still passed with the clearance. The car may
    # turn back as its rear passes it, because the rear then swings away from it
    for driven_m, path in beside_car:
        now = obstacle(item.right_m, item.left_m, item.distance_m - driven_m, item.far_distance_m - driven_m)
        assert not avoidance.in_way(path, now, CAR.obstacle_clearance_m, CAR.obstacle_clearance_m)
    assert beside_car[0][1].offset_at(0.0) == pytest.approx(0.175)
    offsets = [path.offset_at(0.0) for _, path in lines]
    # It moves over and back without jumping, and is back on the usual line once past
    assert max(abs(b - a) for a, b in zip(offsets, offsets[1:])) < 0.02
    assert offsets[-1] == pytest.approx(0.0)


@pytest.mark.parametrize("right_m, left_m", [(-0.30, -0.10), (0.10, 0.30)])
def test_moves_back_before_the_rear_has_passed_a_thin_obstacle_when_the_body_keeps_clear(right_m, left_m):
    item = obstacle(right_m, left_m, 2.5, 2.5)
    # The car's origin where its front is pass_distance_m past the obstacle
    front_past_m = item.far_distance_m + CAR.pass_distance_m
    waiting = plan(LaneAvoidance(CAR, CAR_FRONT_M), [item])
    assert waiting.segments[-1].start_m == pytest.approx(front_past_m + CAR_LENGTH_M)
    early = LaneAvoidance(AvoidanceConfig(return_after_m=0.0), CAR_FRONT_M)
    assert plan(early, [item]).segments[-1].start_m == pytest.approx(front_past_m)
    # Turning back while the rear is still beside it, the body keeps the clearance
    lines = approach(early, item)
    for driven_m, path in lines:
        if item.distance_m <= driven_m <= item.far_distance_m + CAR_LENGTH_M:
            now = obstacle(item.right_m, item.left_m, item.distance_m - driven_m, item.far_distance_m - driven_m)
            assert not early.in_way(path, now, CAR.obstacle_clearance_m, CAR.obstacle_clearance_m)
    assert lines[-1][1].offset_at(0.0) == pytest.approx(0.0)


def test_without_room_to_spare_the_move_back_waits_until_the_body_keeps_clear():
    item = obstacle(-0.30, -0.10, 2.5, 2.5)
    # clearance_weight 0 passes exactly obstacle_clearance_m from it, so turning back at once would bring the body
    # nearer
    avoidance = LaneAvoidance(AvoidanceConfig(return_after_m=0.0, clearance_weight=0.0), CAR_FRONT_M)
    path = plan(avoidance, [item])
    front_past_m = item.far_distance_m + CAR.pass_distance_m
    assert front_past_m < path.segments[-1].start_m < front_past_m + CAR_LENGTH_M
    assert not avoidance.in_way(path, item, CAR.obstacle_clearance_m, CAR.obstacle_clearance_m)


def test_the_move_back_may_be_sharper_than_the_move_out():
    item = obstacle(-0.30, -0.10, 2.5, 2.5)
    config = AvoidanceConfig(max_return_curvature_per_m=0.45)
    path = plan(LaneAvoidance(config, CAR_FRONT_M), [item])
    out, back = path.segments[0], path.segments[-1]
    # peak_curvature_per_m samples the segment, so it may fall just short of the peak
    assert out.peak_curvature_per_m() == pytest.approx(CAR.max_ramp_curvature_per_m, rel=0.01)
    assert back.peak_curvature_per_m() == pytest.approx(config.max_return_curvature_per_m, rel=0.01)
    assert back.end_m - back.start_m == pytest.approx(move_length_m(0.175, config.max_return_curvature_per_m))
    # Once past, with nothing left to drive around, the line from where the car is goes back as sharply
    lines = approach(LaneAvoidance(config, CAR_FRONT_M), item)
    assert max(segment.peak_curvature_per_m() for _, path in lines for segment in path.segments) <= (
        config.max_return_curvature_per_m + 1e-6)
    assert lines[-1][1].offset_at(0.0) == pytest.approx(0.0)


def test_the_line_does_not_jump_when_more_obstacles_come_into_view():
    avoidance = LaneAvoidance(CAR, CAR_FRONT_M)
    path = plan(avoidance, [obstacle(-0.30, -0.10, 2.5, 2.7)])
    for step in range(1, 30):
        driven_m = 0.05 * step
        seen = [obstacle(-0.30, -0.10, 2.5 - driven_m, 2.7 - driven_m)]
        if step >= 20:
            seen.append(obstacle(0.10, 0.30, 7.0 - driven_m, 7.2 - driven_m))
        before = path.moved_back(0.05, 0.0).state_at(0.0)
        path = plan(avoidance, seen, first=False)
        assert path.state_at(0.0) == pytest.approx(before, abs=1e-9)


def test_beside_an_obstacle_the_line_is_held_rather_than_moved_into_it():
    item = obstacle(-0.15, 0.05, 2.5, 2.7)
    avoidance = LaneAvoidance(CAR, CAR_FRONT_M)
    lines = approach(avoidance, item, 1.4, steps=60)
    held = lines[-1][1].moved_back(0.05, 0.0)
    assert held.offset_at(0.0) > 0.0
    # The lane now looks narrower, leaving no room on either side: keep the line, rather than drive back into it
    narrower = plan(avoidance, [], 1.0, first=False)
    assert all(narrower.offset_at(s_m) == pytest.approx(held.offset_at(s_m)) for s_m in np.arange(0.0, 1.0, 0.1))


@pytest.mark.parametrize("wheel_speed", [None, 0.5])
def test_without_fresh_lidar_the_line_is_kept(wheel_speed):
    avoidance = LaneAvoidance(CAR, CAR_FRONT_M)
    path = plan(avoidance, [obstacle(-0.3, -0.1)])
    for _ in range(10):
        kept = plan(avoidance, None, first=False, wheel_speed=wheel_speed)
    # Moved back by the distance driven, if known
    driven_m = 0.0 if wheel_speed is None else 10 * 0.05
    assert all(kept.offset_at(s_m) == pytest.approx(path.offset_at(s_m + driven_m))
               for s_m in np.arange(0.0, 6.0, 0.1))


def test_new_run_and_switching_off_forget_the_line():
    avoidance = LaneAvoidance(CAR, CAR_FRONT_M)
    plan(avoidance, [obstacle(-0.3, -0.1)])
    assert plan(avoidance, [], first=True).segments == ()
    assert plan(LaneAvoidance(AvoidanceConfig(enabled=False), CAR_FRONT_M), [obstacle(-0.3, -0.1)]) is None


def test_obstacles_close_together_share_one_line():
    near, far = obstacle(-0.30, -0.10, 2.5, 2.7), obstacle(-0.35, -0.05, 3.0, 3.2)
    lowest_m, highest_m = plan(LaneAvoidance(CAR, CAR_FRONT_M), [near, far]).offset_range(
        near.distance_m, far.far_distance_m + CAR_LENGTH_M)
    # One line beside both, clear of the one reaching further in
    assert lowest_m == pytest.approx(highest_m)
    assert lowest_m - HALF_WIDTH_M >= far.left_m + CAR.obstacle_clearance_m - 1e-6


def test_obstacles_on_alternate_sides_are_planned_together():
    first, second = obstacle(-0.30, -0.10, 2.5, 2.7), obstacle(0.10, 0.30, 6.5, 6.7)
    path = plan(LaneAvoidance(CAR, CAR_FRONT_M), [first, second])
    assert beside(path, first) == pytest.approx((0.175, 0.175))
    assert beside(path, second) == pytest.approx((-0.175, -0.175))
    # Only 0.6 m apart, they reach too far in to weave between: the first is passed, and the MPC stops for the
    # second if it is in the car's way
    second = obstacle(0.10, 0.30, 3.3, 3.5)
    path = plan(LaneAvoidance(CAR, CAR_FRONT_M), [first, second])
    assert beside(path, first)[0] >= -0.10 + HALF_WIDTH_M + CAR.obstacle_clearance_m - 1e-6
    assert beside(path, second)[0] >= 0.0


def test_weaves_through_a_0_6_m_gap_with_the_car_turned_between_the_obstacles():
    # 0.8 m lane: each obstacle ends 0.20 m short of the centre line, too far in for one straight line past both
    avoidance = LaneAvoidance(CAR, CAR_FRONT_M)
    first, second = obstacle(-0.30, -0.20, 2.5, 2.7), obstacle(0.20, 0.30, 3.3, 3.5)
    path = plan(avoidance, [first, second], 0.8)
    clearance_m = CAR.obstacle_clearance_m
    assert not avoidance.in_way(path, first, clearance_m, clearance_m)
    assert not avoidance.in_way(path, second, clearance_m, clearance_m)
    assert beside(path, first)[1] > 0.0 > beside(path, second)[0]


@pytest.mark.parametrize("curvature_per_m, plans", [
    (0.5, False),  # Turning left as sharply as the car can (0.48 1/m): no room for a move
    (-0.5, True),  # Turning right, where the car can reach 0.92 1/m
])
def test_moves_keep_within_the_cars_limits_on_curves(curvature_per_m, plans):
    avoidance = LaneAvoidance(CAR, CAR_FRONT_M, max_left_curvature_per_m=0.48, max_right_curvature_per_m=0.92)
    path = plan(avoidance, [obstacle(-0.30, -0.10)], curvature_per_m=curvature_per_m)
    assert bool(path.segments) == plans


@pytest.mark.parametrize("change", [
    {"enabled": 1}, {"car_width_m": 0.0}, {"car_rear_m": -0.1}, {"cone_radius_m": math.nan},
    {"cone_clearance_m": -0.1}, {"obstacle_clearance_m": math.inf}, {"pass_distance_m": -0.1},
    {"return_after_m": -0.1}, {"return_after_m": math.nan},
    {"max_ramp_curvature_per_m": 0.0}, {"max_return_curvature_per_m": 0.0}, {"max_return_curvature_per_m": math.inf},
    {"deviation_weight": -1.0}, {"smoothness_weight": math.nan},
    {"clearance_weight": math.inf}, {"change_weight": -0.1},
])
def test_invalid_settings_are_rejected(change):
    with pytest.raises(ValueError):
        AvoidanceConfig(**change)


def test_passing_line_must_clear_the_obstacle_check():
    # 0.125 + 0.05 = 0.175 m < corridor_half_width_m 0.20 m: the passing line would still count as blocked
    with pytest.raises(ValueError):
        MovementPolicy(MpcConfig(), ConeFilterConfig(), ControlConfig(),
                       avoidance_config=AvoidanceConfig(obstacle_clearance_m=0.05))


@pytest.mark.parametrize("name", ["max_ramp_curvature_per_m", "max_return_curvature_per_m"])
def test_moves_must_be_within_the_cars_curvature_limits(name):
    with pytest.raises(ValueError):
        MovementPolicy(MpcConfig(max_curvature_per_m=0.48), ConeFilterConfig(), ControlConfig(),
                       avoidance_config=AvoidanceConfig(**{name: 0.5}))
    MovementPolicy(MpcConfig(max_curvature_per_m=0.48), ConeFilterConfig(), ControlConfig(),
                   avoidance_config=AvoidanceConfig(enabled=False, **{name: 0.5}))


# ---------------------------------------------------------------------------------------------------------------
# Closed loop: the movement policy drives a simple car model along a straight lane past boxes in it
# ---------------------------------------------------------------------------------------------------------------
BOX_LENGTH_M = 0.2


def drive_past(lane_width_m, boxes, steps=140, dt=0.1, lane_side="CENTRE"):
    """Drives the car from the lane centre at 0.5 m/s past boxes in the lane.

    The car follows the MPC's curvature exactly; its speed coasts down with a 2.1 s time constant, like
    mpc.stop_distance_per_speed_s on car 9. The camera sees the cones from 0.4 to 3.5 m ahead within 40 degrees,
    and the lidar sees the boxes' outlines.

    :param boxes: For each box, the distance along the lane from the car's starting origin to its near side, and its
        right and left sides across the lane.
    :param lane_side: mpc.lane_side.
    :return: The movement policy, and for each step the car's (x, y, heading) and the line's offset beside it.
    """
    config = MpcConfig(stop_distance_per_speed_s=2.1, obstacle_stop_distance_per_speed_s=2.1, lane_side=lane_side)
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
    car_memory = CarMemory()
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
        policy.step(observations, car_memory)
        mpc = policy.action_policy
        trace.append((x, y, heading, None if mpc.avoidance_path is None else mpc.avoidance_path.offset_at(0.0)))
        commanded = mpc.previous_speed_m_per_s
        speed = commanded if commanded >= speed else speed + (commanded - speed) * dt / 2.1
        heading += speed * mpc.previous_curvature_per_m * dt
        x += speed * math.cos(heading) * dt
        y += speed * math.sin(heading) * dt
    return policy, trace


def body_outline(x, y, heading, step_m=0.01):
    """Points every step_m around the car's body, a rectangle turned to its heading."""
    rear_m, front_m = CAR.car_rear_m, CAR_FRONT_M
    along = np.arange(-rear_m, front_m + step_m / 2, step_m)
    across = np.arange(-HALF_WIDTH_M, HALF_WIDTH_M + step_m / 2, step_m)
    u = np.concatenate([along, along, np.full(across.shape, front_m), np.full(across.shape, -rear_m)])
    w = np.concatenate([np.full(along.shape, HALF_WIDTH_M), np.full(along.shape, -HALF_WIDTH_M), across, across])
    return x + u * math.cos(heading) - w * math.sin(heading), y + u * math.sin(heading) + w * math.cos(heading)


def gap_to_box(trace, box):
    """The smallest distance in metres between the car's turned body and a box over the drive; 0.0 if they touch."""
    box_x, right_m, left_m = box
    gap_m = math.inf
    for x, y, heading, _ in trace:
        px, py = body_outline(x, y, heading)
        dx = np.maximum(np.maximum(box_x - px, px - (box_x + BOX_LENGTH_M)), 0.0)
        dy = np.maximum(np.maximum(right_m - py, py - left_m), 0.0)
        gap_m = min(gap_m, float(np.min(np.hypot(dx, dy))))
    return gap_m


def widest_reach(trace):
    """The farthest the car's body reaches across the lane from its centre line over the drive, in metres."""
    return max(float(np.max(np.abs(body_outline(x, y, heading)[1]))) for x, y, heading, _ in trace)


@pytest.mark.parametrize("lane_width_m, right_m, left_m", [
    (1.0, -0.30, -0.10),
    (0.8, -0.38, -0.12),  # The lab's lane: a 0.47 m gap leaves 0.11 m either side
], ids=["1m-lane", "0.8m-lane"])
def test_car_drives_around_a_box_with_room_beside_it_and_returns_to_the_line(lane_width_m, right_m, left_m):
    box = (3.5, right_m, left_m)
    policy, trace = drive_past(lane_width_m, [box])
    assert not policy.action_policy.stopped_for_obstacle
    # The body stays clear of the box and of the cones' inner side by nearly the clearances (the MPC follows the
    # line within 1 cm)
    assert gap_to_box(trace, box) > CAR.obstacle_clearance_m - 0.01
    assert widest_reach(trace) < lane_width_m / 2.0 - CAR.cone_radius_m - CAR.cone_clearance_m + 0.01
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
    assert min(gap_to_box(trace, box) for box in boxes) > CAR.obstacle_clearance_m - 0.01
    assert trace[-1][0] > 8.5 + BOX_LENGTH_M


def test_car_moves_across_for_a_second_box_soon_after_the_first():
    # The second box comes into view while the car is moving back from the first: one move takes it across
    boxes = [(3.5, -0.30, -0.10), (6.5, 0.10, 0.30)]
    policy, trace = drive_past(1.0, boxes, steps=240)
    assert not policy.action_policy.stopped_for_obstacle
    assert min(gap_to_box(trace, box) for box in boxes) > CAR.obstacle_clearance_m - 0.01


@pytest.mark.parametrize("lane_width_m, inner_m", [(0.8, 0.20), (1.0, 0.20)])
def test_car_weaves_through_a_0_6_m_gap_between_boxes_on_alternate_sides(lane_width_m, inner_m):
    # Each box ends inner_m short of the centre line, so no straight line passes both; the car turns between them,
    # its rear still beside the first while its front passes the second
    edge_m = lane_width_m / 2.0 - 0.02
    boxes = [(3.5, -edge_m, -inner_m), (3.5 + BOX_LENGTH_M + 0.6, inner_m, edge_m)]
    policy, trace = drive_past(lane_width_m, boxes, steps=200)
    assert not policy.action_policy.stopped_for_obstacle
    assert min(gap_to_box(trace, box) for box in boxes) > CAR.obstacle_clearance_m - 0.01
    assert trace[-1][0] > boxes[1][0] + BOX_LENGTH_M


@pytest.mark.parametrize("inner_m", [0.15, 0.10])
def test_car_stops_before_a_0_6_m_gap_too_tight_to_weave_through(inner_m):
    boxes = [(3.5, -0.38, -inner_m), (4.3, inner_m, 0.38)]
    policy, trace = drive_past(0.8, boxes, steps=200)
    assert policy.action_policy.stopped_for_obstacle
    assert min(gap_to_box(trace, box) for box in boxes) > 0.05
    assert trace[-1][0] + CAR_FRONT_M < boxes[1][0]


@pytest.mark.parametrize("gap_m", [1.6, 2.0])
def test_car_passes_boxes_on_both_edges(gap_m):
    # As on the car on 15 Oct: a box on each edge of a 1.0 m lane, the second gap_m after the first. Once the second
    # came into view the old planner stopped the car beside the first
    boxes = [(3.5, 0.25, 0.45), (3.5 + BOX_LENGTH_M + gap_m, -0.45, -0.25)]
    policy, trace = drive_past(1.0, boxes, steps=240)
    assert not policy.action_policy.stopped_for_obstacle
    assert min(gap_to_box(trace, box) for box in boxes) > CAR.obstacle_clearance_m - 0.01
    assert trace[-1][0] > boxes[1][0] + BOX_LENGTH_M


@pytest.mark.parametrize("right_m, left_m", [
    (0.10, 0.50),  # On the left line (0.3 m): pass on its right
    (-0.50, -0.10),  # In the other half of the lane: stay to its left
], ids=["box-on-the-line", "box-on-the-other-side"])
def test_car_keeping_left_drives_around_a_box_and_returns_to_its_side(right_m, left_m):
    # 1.4 m lane: mpc.lane_side "LEFT" follows the line 0.3 m left of the centre, edge_clearance_m from the left edge
    box = (6.0, right_m, left_m)
    policy, trace = drive_past(1.4, [box], steps=220, lane_side="LEFT")
    assert not policy.action_policy.stopped_for_obstacle
    assert gap_to_box(trace, box) > CAR.obstacle_clearance_m - 0.01
    assert widest_reach(trace) < 1.4 / 2.0 - CAR.cone_radius_m - CAR.cone_clearance_m + 0.01
    # Back on the left line once past
    assert trace[-1][0] > 6.0 + BOX_LENGTH_M and abs(trace[-1][1] - 0.3) < 0.05


def straight_lane(lane_width_m, first=False):
    """Observations of a car on the centre line of a straight lane at 0.5 m/s, 0.1 s after the previous step, without
    lidar."""
    cones = [ConeDetection(Position(x * 0.5, side * lane_width_m / 2.0, 0.1), colour, 0.9) for x in range(1, 7)
             for side, colour in ((1.0, ConeColour.BLUE), (-1.0, ConeColour.YELLOW))]
    return CarObservations(ConeBatch(cones, SensorAge(0.0, None), 0.0), None, None, None,
                           ObservedCarState(None, None, None, WheelSpeed(0.5, SensorAge(0.0, None))),
                           PolicyState(0.0 if first else 0.1, 0.0, first))


def test_new_run_plans_from_the_configured_lane_side():
    # The avoidance plans from the MPC's line before the MPC's own step, so on the first step of a run the line must
    # already be back at mpc.lane_side, not where a manoeuvre left the previous run
    policy = MovementPolicy(MpcConfig(lane_side="LEFT"), ConeFilterConfig(), ControlConfig())
    car_memory = CarMemory()
    policy.step(straight_lane(1.6, first=True), car_memory)
    assert policy.lane_path.usual_offset_m == pytest.approx(0.4, abs=0.02)
    policy.action_policy.lane_side = "RIGHT"
    for _ in range(100):
        policy.step(straight_lane(1.6), car_memory)
    assert policy.lane_path.usual_offset_m == pytest.approx(-0.4, abs=0.02)
    policy.step(straight_lane(1.6, first=True), car_memory)
    assert policy.lane_path.usual_offset_m == pytest.approx(0.4, abs=0.02)
