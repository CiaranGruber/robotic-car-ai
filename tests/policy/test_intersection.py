"""Checks the intersection manoeuvre (policy/action_policy/intersection.py): its settings, where it starts the arc,
which cones it keeps for lane detection, its memory, and whole runs of the marker simulation (marker_simulation.py)."""
import dataclasses
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import marker_simulation as ms  # noqa: E402
from policy.action_policy.intersection import (IntersectionConfig, IntersectionTurner, arc_start_m,  # noqa: E402
                                               CONE_RADIUS_M)
from policy.action_policy.mpc import MpcConfig  # noqa: E402
from policy.action_policy.policy import MpcPolicy  # noqa: E402
from policy.control.control import ControlConfig, steering_for_curvature  # noqa: E402
from policy.input_output import ConeColour, ConeDetection, Position  # noqa: E402
from policy.markers.commands import MarkerCommand  # noqa: E402
from policy.memory import CarMemory, CarPose, IntersectionTurn, Memory, MpcStop, TurnPhase  # noqa: E402
from policy.visualisation import _intersection_lines  # noqa: E402

HALF_WIDTH_M, FRONT_M, MARGIN_M = 0.125, 0.30, 0.03


def test_settings_are_checked():
    for name, value in (("left_turn_radius_m", 0.0), ("turn_speed_m_per_s", -0.1), ("corner_clearance_m", -0.01),
                        ("handover_heading_deg", 91.0), ("marker_to_entry_m", math.inf), ("enabled", 1),
                        ("aside_clearance_m", 0.0)):
        with pytest.raises(ValueError, match=name):
            IntersectionConfig(**{name: value})


def test_the_arc_starts_where_it_ends_on_the_exit_centre_line_when_it_can():
    # 1 m roads, right turn of 1.2 m from the lane centre: the start that ends on the exit's centre line (0.5 - 1.2)
    # keeps both clearances
    start, feasible = arc_start_m(1.0, 1.0, 1.2, 0.0, HALF_WIDTH_M, FRONT_M, MARGIN_M)
    assert feasible and start == pytest.approx(0.5 - 1.2)
    # Its front corner stays clear of the far row
    assert start + math.hypot(FRONT_M, 1.2 + HALF_WIDTH_M) <= 1.0 - CONE_RADIUS_M - MARGIN_M
    # From 0.1 m towards the turn side, that start would pass the inside corner too close: it starts at the earliest
    # start that keeps the corner clear
    start, feasible = arc_start_m(1.0, 1.0, 1.2, -0.1, HALF_WIDTH_M, FRONT_M, MARGIN_M)
    room = 1.2 - HALF_WIDTH_M - CONE_RADIUS_M - MARGIN_M
    assert feasible and start == pytest.approx(-math.sqrt(room ** 2 - 0.8 ** 2))


def test_moving_aside_makes_room_for_a_turn_in_a_narrow_lane():
    # 0.8 m roads, right turn of 1.2 m: from the centre no start clears both the corner and the far row
    _, feasible = arc_start_m(0.8, 0.8, 1.2, 0.0, HALF_WIDTH_M, FRONT_M, MARGIN_M)
    assert not feasible
    # Keeping left, 0.25 m from the left edge (0.15 m off the centre), it can
    start, feasible = arc_start_m(0.8, 0.8, 1.2, 0.15, HALF_WIDTH_M, FRONT_M, MARGIN_M)
    assert feasible and start < 0.0


def test_approaching_lane_detection_only_gets_the_approach_roads_cones():
    memory = CarMemory()
    memory.start_map(None)
    turner = IntersectionTurner(IntersectionConfig(), MpcPolicy(MpcConfig()), 0.25, 0.30)
    memory.plan_turn(IntersectionTurn(MarkerCommand.TURN_RIGHT, TurnPhase.APPROACH, 2.0, 0.0, 0.0, 0.8))
    cones = [ConeDetection(Position(x, y, 0.0), ConeColour.BLUE, 0.9)
             for x, y in ((1.0, 0.4), (2.1, 0.4), (2.6, 0.4), (1.5, 0.9), (1.9, -0.6))]
    kept = turner.lane_cones(cones, memory)
    # Past the entry (2.6 m) and outside the lane's edges (0.9 m left) are left out
    assert [(c.pos.x, c.pos.y) for c in kept] == [(1.0, 0.4), (2.1, 0.4), (1.9, -0.6)]
    # Once turning, every cone is used again
    memory.plan_turn(dataclasses.replace(memory.data.intersection_turn, phase=TurnPhase.ARC))
    assert turner.lane_cones(cones, memory) == cones


def test_a_turn_marker_places_the_entry_and_moves_aside():
    memory = CarMemory()
    memory.start_map(None)
    mpc = MpcPolicy(MpcConfig())
    turner = IntersectionTurner(IntersectionConfig(marker_to_entry_m=1.0), mpc, 0.25, 0.30)
    turner.start(MarkerCommand.TURN_RIGHT, Position(1.5, -0.6, 0.15), None, 0.8, memory)
    turn = memory.data.intersection_turn
    assert (turn.entry_x_m, turn.entry_y_m, turn.phase) == (pytest.approx(2.5), pytest.approx(0.0), TurnPhase.APPROACH)
    assert mpc.lane_side == "LEFT" and mpc.edge_clearance_m == 0.25
    # A side marker does not start a turn, and a pose is needed
    other = CarMemory()
    turner.start(MarkerCommand.TURN_LEFT, Position(1.5, 0.6, 0.15), None, 0.8, other)
    assert other.data.intersection_turn is None


def test_memory_with_a_turn_saves_and_loads():
    memory = CarMemory()
    memory.plan_turn(IntersectionTurn(MarkerCommand.TURN_LEFT, TurnPhase.EXIT, 1.0, 2.0, 0.1, 0.9, "RIGHT", 3.0, 4.0,
                                      5.0, 6.0, 1.6))
    assert Memory.deserialise(memory.data.serialise()) == memory.data
    old = {key: value for key, value in memory.data.serialise().items() if key != "intersection_turn"}
    assert Memory.deserialise(old).intersection_turn is None
    memory.finish_turn()
    assert memory.data.intersection_turn is None and memory.data.intersection_command is None
    memory.plan_turn(IntersectionTurn(MarkerCommand.STRAIGHT, TurnPhase.APPROACH, 0.0, 0.0, 0.0, 0.8))
    memory.start_marker_run()
    assert memory.data.intersection_turn is None


def test_a_lost_pose_abandons_the_turn():
    memory = CarMemory()
    mpc = MpcPolicy(MpcConfig())
    turner = IntersectionTurner(IntersectionConfig(), mpc, 0.25, 0.30)
    memory.plan_turn(IntersectionTurn(MarkerCommand.TURN_LEFT, TurnPhase.ARC, 0.0, 0.0, 0.0, 0.8, "CENTRE", 0.0, 1.1))
    assert turner.step(None, memory) is None
    assert memory.data.intersection_turn is None and mpc.lane_side == "CENTRE"


def test_the_arc_command_stays_within_the_cars_reach():
    memory = CarMemory()
    memory.start_map(None)
    memory.move_car(CarPose(0.0, 0.5, 0.0), None)
    config = IntersectionConfig()
    turner = IntersectionTurner(config, MpcPolicy(MpcConfig()), 0.25, 0.30)
    # Far outside a right turn's arc, pure pursuit would ask for far more than the car can steer
    memory.plan_turn(IntersectionTurn(MarkerCommand.TURN_RIGHT, TurnPhase.ARC, 0.0, 0.0, 0.0, 0.8, "CENTRE", 0.0, -1.2))
    speed, curvature = turner.step(None, memory)
    assert speed == config.turn_speed_m_per_s
    assert curvature == pytest.approx(-turner.max_right_curvature_per_m)
    assert turner.path_curvature_per_m == curvature


@pytest.mark.parametrize("command, side, expected_radius, along", [
    (MarkerCommand.TURN_LEFT, 1.0, 1.0 / .48, -1.3),
    (MarkerCommand.TURN_RIGHT, -1.0, 1.2, -.5),
])
def test_planned_arc_uses_each_sides_capability_before_choosing_when_to_turn(command, side, expected_radius, along):
    memory = CarMemory()
    memory.start_map(None)
    memory.move_car(CarPose(along, -side * .2, 0.0), None)
    turner = IntersectionTurner(IntersectionConfig(), MpcPolicy(MpcConfig()), .25, .30)
    memory.plan_turn(IntersectionTurn(command, TurnPhase.APPROACH, 0., 0., 0., 1.2))
    speed, curvature = turner.step(None, memory)
    turn = memory.data.intersection_turn
    assert turn.phase is TurnPhase.ARC and turner.feasible
    radius = math.hypot(turn.arc_centre_x_m - along, turn.arc_centre_y_m + side * .2)
    assert radius == pytest.approx(expected_radius)
    assert speed == .3 and curvature == pytest.approx(side / expected_radius)
    assert turner.path_curvature_per_m == curvature
    # The old 1.1 m left arc would still be waiting at this pose; the right arc keeps its 1.2 m radius.
    if command is MarkerCommand.TURN_LEFT:
        old_start, _ = arc_start_m(1.2, 1.2, 1.1, .2, HALF_WIDTH_M, FRONT_M, MARGIN_M)
        assert along < old_start


def test_turn_radius_also_respects_the_actual_controllers_action_limit():
    memory = CarMemory()
    memory.start_map(None)
    control = ControlConfig(max_steering_action=.6)
    turner = IntersectionTurner(IntersectionConfig(), MpcPolicy(MpcConfig(max_curvature_per_m=2.0)),
                                .25, .30, control)
    memory.plan_turn(IntersectionTurn(MarkerCommand.TURN_LEFT, TurnPhase.APPROACH, 0., 0., 0., 3.0))
    speed, curvature = turner.step(None, memory)
    turn = memory.data.intersection_turn
    expected = (.6 + control.steering_centre_action) / abs(control.steering_left_action_per_curvature)
    assert turn.phase is TurnPhase.ARC and speed > 0.0
    assert curvature == pytest.approx(expected)
    assert math.hypot(turn.arc_centre_x_m, turn.arc_centre_y_m) == pytest.approx(1.0 / expected)
    assert steering_for_curvature(curvature, control) == pytest.approx(-.6)


def test_a_turn_that_cannot_fit_stops_instead_of_attempting_the_arc():
    memory = CarMemory()
    memory.start_map(None)
    turner = IntersectionTurner(IntersectionConfig(), MpcPolicy(MpcConfig()), .25, .30)
    # A 2.08 m left radius cannot fit a .8 m crossing from its centre.
    memory.plan_turn(IntersectionTurn(MarkerCommand.TURN_LEFT, TurnPhase.APPROACH, 0., 0., 0., .8))
    assert turner.step(None, memory) == (0.0, 0.0)
    assert memory.data.intersection_turn.phase is TurnPhase.APPROACH
    assert not turner.feasible


def test_a_direction_with_no_available_steering_range_cannot_start_a_turn():
    memory = CarMemory()
    memory.start_map(None)
    turner = IntersectionTurner(IntersectionConfig(), MpcPolicy(MpcConfig()), .25, .30,
                                ControlConfig(steering_centre_action=-1.0))
    memory.plan_turn(IntersectionTurn(MarkerCommand.TURN_LEFT, TurnPhase.APPROACH, 0., 0., 0., 1.2))
    assert turner.step(None, memory) == (0.0, 0.0)
    assert memory.data.intersection_turn.phase is TurnPhase.APPROACH


@pytest.mark.parametrize("command, side", [(MarkerCommand.TURN_LEFT, 1.), (MarkerCommand.TURN_RIGHT, -1.)])
@pytest.mark.parametrize("heading", [-.15, .15])
def test_admitted_arc_is_tangent_to_actual_car_heading(command, side, heading):
    memory = CarMemory()
    memory.start_map(None)
    pose = CarPose(1., 0., heading)
    memory.move_car(pose, None)
    turner = IntersectionTurner(IntersectionConfig(), MpcPolicy(MpcConfig()), .25, .30)
    memory.plan_turn(IntersectionTurn(command, TurnPhase.APPROACH, 0., 0., 0., 4.))
    speed, curvature = turner.step(None, memory)
    turn = memory.data.intersection_turn
    assert turn.phase is TurnPhase.ARC and speed > 0.
    radial_x, radial_y = pose.x_m - turn.arc_centre_x_m, pose.y_m - turn.arc_centre_y_m
    radius = math.hypot(radial_x, radial_y)
    assert radial_x * math.cos(heading) + radial_y * math.sin(heading) == pytest.approx(0., abs=1e-12)
    assert curvature == pytest.approx(side / radius)
    drawn = _intersection_lines(SimpleNamespace(intersection_turn=turn, pose=pose, turner=turner))[1]
    assert drawn.points[0] == pytest.approx((0., 0.), abs=1e-12)


def test_a_feasible_arc_cannot_be_started_after_its_last_clear_position():
    memory = CarMemory()
    memory.start_map(None)
    memory.move_car(CarPose(0., -.2, 0.), None)
    turner = IntersectionTurner(IntersectionConfig(), MpcPolicy(MpcConfig()), .25, .30)
    memory.plan_turn(IntersectionTurn(MarkerCommand.TURN_LEFT, TurnPhase.APPROACH, 0., 0., 0., 1.2))
    assert turner.step(None, memory) == (0., 0.)
    assert not turner.feasible and memory.data.intersection_turn.phase is TurnPhase.APPROACH


def test_stopped_run_does_not_announce_an_arc_as_the_pose_keeps_updating():
    memory = CarMemory()
    memory.start_map(None)
    turner = IntersectionTurner(IntersectionConfig(), MpcPolicy(MpcConfig()), .25, .30)
    turn = IntersectionTurn(MarkerCommand.TURN_LEFT, TurnPhase.APPROACH, 0., 0., 0., 1.2)
    memory.plan_turn(turn)
    memory.stop_mpc(MpcStop.LANE_LOSS)
    memory.move_car(CarPose(-1.3, -.2, 0.), None)  # A clear start, but this run already stopped.
    assert turner.step(None, memory) == (0., 0.)
    assert memory.data.intersection_turn == turn


def test_geometric_preview_is_bounded_and_cannot_change_the_active_turn_or_controller():
    memory = CarMemory()
    memory.start_map(None)
    memory.move_car(CarPose(-2.483, .055, -.205), None)
    memory.plan_turn(IntersectionTurn(MarkerCommand.TURN_LEFT, TurnPhase.APPROACH, 0., 0., 0., 1.037))
    mpc = MpcPolicy(MpcConfig())
    turner = IntersectionTurner(IntersectionConfig(), mpc, .25, .30)
    turner.step(None, memory)
    before = memory.data.serialise(), mpc.lane_side, mpc.edge_clearance_m, mpc.side_offset_m, turner.arc_start_m
    points = turner.preview_path(memory, 1.8)
    assert points[0] == (0., 0.) and 2 <= len(points) <= 129
    assert sum(math.dist(a, b) for a, b in zip(points, points[1:])) == pytest.approx(1.8, abs=.001)
    assert (memory.data.serialise(), mpc.lane_side, mpc.edge_clearance_m,
            mpc.side_offset_m, turner.arc_start_m) == before
    with pytest.raises(ValueError, match="6.4"):
        turner.preview_path(memory, 6.5)


@pytest.mark.parametrize("command, side, radius", [
    (MarkerCommand.TURN_LEFT, 1., 1. / .48), (MarkerCommand.TURN_RIGHT, -1., 1.2),
])
def test_exit_search_distance_starts_at_arc_end_even_for_a_wide_left_turn(command, side, radius):
    memory = CarMemory()
    memory.start_map(None)
    centre_x, centre_y = .5 - radius, side * (radius - .25)
    heading = side * math.radians(89.)
    memory.move_car(CarPose(centre_x + side * radius * math.sin(heading),
                           centre_y - side * radius * math.cos(heading), heading), None)
    turner = IntersectionTurner(IntersectionConfig(), MpcPolicy(MpcConfig()), .25, .30)
    memory.plan_turn(IntersectionTurn(command, TurnPhase.ARC, 0., 0., 0., 1., "CENTRE", centre_x, centre_y))
    assert turner.step(None, memory)[0] > 0.
    turn = memory.data.intersection_turn
    assert turn.phase is TurnPhase.EXIT
    assert turn.exit_y_m == pytest.approx(centre_y)
    memory.move_car(CarPose(turn.exit_x_m, turn.exit_y_m + side * 1.6, side * math.pi / 2.), None)
    assert turner.step(None, memory) == (0., 0.)
    assert memory.data.intersection_turn is None


@pytest.mark.parametrize("command, side", [(MarkerCommand.TURN_LEFT, 1.), (MarkerCommand.TURN_RIGHT, -1.)])
def test_preparation_and_tangent_turn_reach_exit_with_missing_cones_in_nominal_feedback(command, side):
    """Small noiseless kinematic check, seeded from the latest run's confirmation geometry; not car qualification."""
    from tests.policy.intersection_simulation import CrossIntersection

    memory = CarMemory()
    memory.start_map(None)
    pose = CarPose(-2.483, side * .055, -side * .205)
    memory.move_car(pose, None)
    turner = IntersectionTurner(IntersectionConfig(), MpcPolicy(MpcConfig()), .25, .30)
    memory.plan_turn(IntersectionTurn(command, TurnPhase.APPROACH, 0., 0., 0., 1.037))
    road = CrossIntersection(width_m=1.037)
    for _ in range(400):
        speed, curvature = turner.step(None, memory)
        assert speed > 0. and memory.data.intersection_turn is not None
        assert -turner.max_right_curvature_per_m <= curvature <= turner.max_left_curvature_per_m
        assert road.body_outside_m((pose.x_m - road.width_m / 2., pose.y_m, pose.heading_rad)) < 1e-9
        if memory.data.intersection_turn.phase is TurnPhase.EXIT:
            break
        distance = speed * .05
        heading = pose.heading_rad + distance * curvature
        if abs(curvature) > 1e-6:
            pose = CarPose(pose.x_m + (math.sin(heading) - math.sin(pose.heading_rad)) / curvature,
                           pose.y_m + (math.cos(pose.heading_rad) - math.cos(heading)) / curvature, heading)
        else:
            pose = CarPose(pose.x_m + distance * math.cos(heading), pose.y_m + distance * math.sin(heading), heading)
        memory.move_car(pose, None)
    else:
        pytest.fail("Nominal missing-cone turn did not reach its exit")


SETTINGS = ms.read_settings(ms.DEFAULT_SETTINGS_PATH)


@pytest.mark.parametrize("name", ["cross_left_1m", "cross_right_1m", "cross_straight_1m", "t_left_1m", "t_right_1m",
                                  "cross_straight_0.8m", "lanes_0.8m", "lanes_1m"])
def test_the_simulated_car_follows_the_markers(name):
    """With config/ai4r_policy.yaml and car 21's provisional limits, the car leaves each intersection by the road its
    marker asks for, and keeps to each side a lane marker asks for, without touching a cone or leaving the road.

    Turns in 0.8 m roads are at the edge of what those limits allow and pass in most runs, not all
    (docs/marker-commands.md), so they are not checked here."""
    if name.startswith("lanes_") and not any(case.name == name for case in ms.CASES):
        pytest.skip("Legacy keep-side commands are disabled by the current direction-only mapping")
    case = next(case for case in ms.CASES if case.name == name)
    result = ms.run_case(case, SETTINGS, seed=0)
    assert result.outcome == "passed", result.metrics


def test_without_the_manoeuvre_or_branch_association_the_car_stops_where_the_approach_ends():
    case = next(case for case in ms.CASES if case.name == "cross_left_1m")
    result = ms.run_case(case, {**SETTINGS, "intersection.enabled": False,
                                "lane_detection.associate_branches": False}, seed=0)
    assert result.outcome == "stopped"
    assert result.x[-1] < case.world.roads[1].x_m


def test_without_the_manoeuvre_branch_association_carries_the_car_straight_across():
    """lane_detection.associate_branches separates the crossing arms, so the approach lane continues across the
    crossing: with the manoeuvre off the car ignores the turn marker and drives straight over, touching no cone."""
    case = next(case for case in ms.CASES if case.name == "cross_left_1m")
    result = ms.run_case(case, {**SETTINGS, "intersection.enabled": False}, seed=0)
    assert result.outcome == "wrong exit"
    assert result.x[-1] > case.world.roads[1].x_m
    assert result.metrics["cones_touched"] == 0
    assert result.metrics["max_outside_m"] < 0.0
