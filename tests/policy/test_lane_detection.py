"""Hardware-free checks of lane detection, on drawn lanes and on the recorded real scenarios, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_lane_detection.py
"""
import dataclasses
import json
import math
import random
import statistics

import pytest

from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones
from policy.input_output import CarObservations, ConeBatch, ConeColour, ConeDetection, Position, SensorAge
from policy.lane_detection.lane_detection import LaneDetectionConfig, detect_lanes
from policy.lane_detection.lanes import LaneArc, LaneDetection, LaneStatus
from scenarios.scenario_wrapper import Scenario, ScenarioType

YELLOW, BLUE = ConeColour.YELLOW, ConeColour.BLUE
CONFIG = LaneDetectionConfig()
CURVED = dataclasses.replace(CONFIG, max_curvature_per_m=2.0)


def lane_cones(radius_m=math.inf, turn=1.0, offset_m=0.0, heading_deg=0.0, width_m=1.0, spacing_m=0.5, length_m=3.0,
               noise_m=0.0, seed=0, colours=(BLUE, YELLOW)):
    """Cones of a lane in base_link, blue on the left and yellow on the right by default.

    The lane starts beside the car and is straight, or curves around a circle of radius_m to the left (turn = 1) or
    right (turn = -1).

    :param offset_m: How far the car's origin is left of the lane centre.
    :param heading_deg: How far the car points left of the lane direction.
    :param noise_m: Standard deviation of random detector noise added to each position.
    """
    rng = random.Random(seed)
    heading = math.radians(heading_deg)
    cones = []
    for i in range(round(length_m / spacing_m) + 1):
        s = i * spacing_m
        for colour, side in zip(colours, (1.0, -1.0)):
            if math.isinf(radius_m):
                lane_x, lane_y = s, side * width_m / 2.0
            else:
                # Lane frame: the centre line starts at the origin heading along +x, turning around (0, turn * R)
                radius = radius_m - turn * side * width_m / 2.0
                angle = s / radius_m
                lane_x, lane_y = radius * math.sin(angle), turn * (radius_m - radius * math.cos(angle))
            dx, dy = lane_x, lane_y - offset_m
            x = math.cos(heading) * dx + math.sin(heading) * dy + rng.gauss(0.0, noise_m)
            y = -math.sin(heading) * dx + math.cos(heading) * dy + rng.gauss(0.0, noise_m)
            if x > 0.0:
                cones.append(ConeDetection(Position(x, y, 0.05), colour, 0.9))
    return cones


def observe(cones):
    """Observations holding only a cone batch, as the lane detection receives them."""
    return CarObservations(
        cones=None if cones is None else ConeBatch(cones, SensorAge(0.08, 123), 0.03),
        fiducials=None, lidar_scan=None, lidar_cartesian=None, car=None, policy=None)


# ------------------------------------------------------------------------------------------------
# Drawn straight lanes
# ------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("offset_m, heading_deg", [(0.0, 0.0), (0.15, 0.0), (0.0, 10.0), (-0.15, -10.0), (0.2, -5.0)])
def test_straight_lane_gives_the_car_position_in_the_lane(offset_m, heading_deg):
    lane = detect_lanes(observe(lane_cones(offset_m=offset_m, heading_deg=heading_deg)), CONFIG)
    assert lane.status == LaneStatus.BOTH_EDGES and lane.has_lane
    assert lane.lateral_error_m == pytest.approx(offset_m, abs=1e-6)
    assert lane.heading_error_rad == pytest.approx(math.radians(heading_deg), abs=1e-6)
    assert lane.lane_width_m == pytest.approx(1.0)
    assert lane.path_curvature_per_m == pytest.approx(0.0, abs=1e-6)
    assert lane.left_edge.colour == BLUE and lane.right_edge.colour == YELLOW
    assert lane.left_edge.rms_residual_m == pytest.approx(0.0, abs=1e-6)
    assert lane.sensor_age == pytest.approx(0.08) and lane.sensor_age.stamp_ns == 123


def test_visible_range_is_measured_along_the_lane():
    cones = lane_cones(width_m=0.8)
    lane = detect_lanes(observe(cones), CONFIG)
    assert lane.lane_width_m == pytest.approx(0.8)
    assert lane.visible_until_m == pytest.approx(max(cone.pos.x for cone in cones))
    assert lane.left_edge.nearest_m == pytest.approx(0.5) and lane.left_edge.farthest_m == pytest.approx(3.0)


@pytest.mark.parametrize("keep, status", [(BLUE, LaneStatus.LEFT_EDGE_ONLY), (YELLOW, LaneStatus.RIGHT_EDGE_ONLY)])
def test_one_edge_places_the_centre_half_a_lane_width_away(keep, status):
    cones = [cone for cone in lane_cones(offset_m=0.1, heading_deg=10.0) if cone.colour == keep]
    lane = detect_lanes(observe(cones), CONFIG)
    assert lane.status == status
    assert (lane.left_edge is None) == (keep == YELLOW) and (lane.right_edge is None) == (keep == BLUE)
    assert lane.lateral_error_m == pytest.approx(0.1, abs=1e-6)
    assert lane.heading_error_rad == pytest.approx(math.radians(10.0), abs=1e-6)
    assert lane.lane_width_m == CONFIG.lane_width_m


def test_left_colour_setting_swaps_the_edges():
    cones = lane_cones(offset_m=0.1, heading_deg=5.0, colours=(YELLOW, BLUE))
    lane = detect_lanes(observe(cones), dataclasses.replace(CONFIG, left_colour="YELLOW"))
    assert lane.status == LaneStatus.BOTH_EDGES
    assert lane.left_edge.colour == YELLOW and lane.lateral_error_m == pytest.approx(0.1, abs=1e-6)
    # With the wrong setting, the edges cross, so the lane is doubtful
    assert detect_lanes(observe(cones), CONFIG).status == LaneStatus.NO_LANE


@pytest.mark.parametrize("width_m", [0.3, 2.0])
def test_implausible_width_gives_no_lane(width_m):
    lane = detect_lanes(observe(lane_cones(width_m=width_m)), CONFIG)
    assert lane.status == LaneStatus.NO_LANE and not lane.has_lane
    assert lane.centre_line is None and lane.lateral_error_m is None and lane.lane_width_m is None


def test_a_short_row_borrows_the_direction_of_a_long_one():
    cones = lane_cones(heading_deg=10.0)
    blue = [cone for cone in cones if cone.colour == BLUE]
    one_yellow = [cone for cone in cones if cone.colour == YELLOW][2:3]
    assert detect_lanes(observe(blue + one_yellow), CONFIG).status == LaneStatus.LEFT_EDGE_ONLY
    lane = detect_lanes(observe(blue + one_yellow), dataclasses.replace(CONFIG, min_cones_per_edge=1))
    assert lane.status == LaneStatus.BOTH_EDGES
    assert lane.lane_width_m == pytest.approx(1.0, abs=1e-6) and lane.lateral_error_m == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize("cones", [
    [],
    [ConeDetection(Position(1.0, 0.5, 0.05), BLUE, 0.9)],
    # Two cones per edge, but too close together to give a direction
    [ConeDetection(Position(1.0, y, 0.05), BLUE, 0.9) for y in (0.5, 0.4)]
    + [ConeDetection(Position(1.1, y, 0.05), YELLOW, 0.9) for y in (-0.5, -0.4)],
])
def test_too_few_cones_give_no_lane(cones):
    assert detect_lanes(observe(cones), CONFIG).status == LaneStatus.NO_LANE


def test_no_cone_data_gives_none():
    assert detect_lanes(observe(None), CONFIG) is None


@pytest.mark.parametrize("config", [CONFIG, CURVED], ids=["straight", "curved"])
def test_a_misplaced_cone_is_left_out(config):
    cones = lane_cones(offset_m=0.1, heading_deg=5.0)
    # Blue, but in the middle of the lane, about 0.25 m from the left edge
    misplaced = ConeDetection(Position(1.6, 0.0, 0.05), BLUE, 0.9)
    lane = detect_lanes(observe(cones + [misplaced]), config)
    assert lane.status == LaneStatus.BOTH_EDGES
    assert lane.left_edge.cone_count == sum(cone.colour == BLUE for cone in cones)
    assert lane.lateral_error_m == pytest.approx(0.1, abs=0.01)


# ------------------------------------------------------------------------------------------------
# Drawn curved lanes
# ------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("turn", [1.0, -1.0], ids=["left", "right"])
@pytest.mark.parametrize("radius_m", [1.0, 1.5, 3.0, 8.0])
def test_curved_lane_gives_its_curvature_and_the_car_position(radius_m, turn):
    lane = detect_lanes(observe(lane_cones(radius_m, turn, offset_m=0.1, heading_deg=5.0, length_m=2.5)), CURVED)
    assert lane.status == LaneStatus.BOTH_EDGES
    assert lane.path_curvature_per_m == pytest.approx(turn / radius_m, rel=0.05)
    assert lane.lateral_error_m == pytest.approx(0.1, abs=0.01)
    assert lane.heading_error_rad == pytest.approx(math.radians(5.0), abs=math.radians(1.0))
    assert lane.lane_width_m == pytest.approx(1.0, abs=0.01)
    assert lane.visible_until_m == pytest.approx(2.5, abs=0.05)
    # The inner edge curves more than the outer one, and the centre line stays half a lane width from both
    inner, outer = (lane.left_edge, lane.right_edge) if turn > 0 else (lane.right_edge, lane.left_edge)
    assert abs(inner.arc.curvature_per_m) > abs(lane.path_curvature_per_m) > abs(outer.arc.curvature_per_m)
    for s in (0.0, 1.0, 2.0):
        point = lane.centre_line.point_at(s)
        assert lane.left_edge.arc.signed_distance(*point) == pytest.approx(-0.5, abs=0.01)
        assert lane.right_edge.arc.signed_distance(*point) == pytest.approx(0.5, abs=0.01)


def test_edges_are_straight_by_default():
    lane = detect_lanes(observe(lane_cones(8.0)), CONFIG)
    assert lane.status == LaneStatus.BOTH_EDGES and lane.path_curvature_per_m == 0.0


def test_detector_noise_does_not_bend_a_straight_lane():
    curvatures = [abs(detect_lanes(observe(lane_cones(length_m=2.0, noise_m=0.03, seed=seed)), CURVED)
                      .path_curvature_per_m) for seed in range(20)]
    # 0.1 1/m is a 10 m radius, 5 times the visible length
    assert statistics.median(curvatures) < 0.1


def test_detector_noise_keeps_a_curve():
    curvatures = [detect_lanes(observe(lane_cones(2.0, length_m=2.0, noise_m=0.03, seed=seed)), CURVED)
                  .path_curvature_per_m for seed in range(20)]
    assert statistics.median(curvatures) == pytest.approx(0.5, rel=0.15)


# ------------------------------------------------------------------------------------------------
# Types
# ------------------------------------------------------------------------------------------------
def test_lane_arc_geometry():
    straight = LaneArc(0.0, 0.2, math.radians(30.0), 0.0)
    assert straight.point_at(2.0) == pytest.approx((2.0 * math.cos(math.radians(30.0)), 0.2 + 1.0))
    assert straight.signed_distance(0.0, 0.0) == pytest.approx(-0.2 * math.cos(math.radians(30.0)))
    # A quarter circle of radius 2 m turning left, then one turning right
    left = LaneArc(0.0, 0.0, 0.0, 0.5)
    assert left.point_at(math.pi) == pytest.approx((2.0, 2.0))
    assert left.heading_at(math.pi) == pytest.approx(math.pi / 2.0)
    assert left.project(2.0, 2.0) == pytest.approx(math.pi)
    assert left.signed_distance(0.0, 0.5) == pytest.approx(0.5)
    assert left.signed_distance(2.5, 2.0) == pytest.approx(-0.5)
    assert LaneArc(0.0, 0.0, 0.0, -0.5).point_at(math.pi) == pytest.approx((2.0, -2.0))
    inner = left.offset(0.5)
    assert (inner.x, inner.y, inner.curvature_per_m) == pytest.approx((0.0, 0.5, 1.0 / 1.5))


def test_lane_detection_serialises_round_trip():
    lane = detect_lanes(observe(lane_cones(3.0, offset_m=0.1)), CURVED)
    data = json.loads(json.dumps(lane.serialise()))
    assert data["status"] == "BOTH_EDGES" and data["left_edge"]["colour"] == "BLUE"
    assert LaneDetection.deserialise(data) == lane


@pytest.mark.parametrize("change", [
    {"left_colour": "RED"}, {"left_colour": "blue"}, {"lane_width_m": 0.0}, {"lane_width_m": math.nan},
    {"min_cones_per_edge": 0}, {"min_cones_per_edge": 2.0}, {"min_cones_per_edge": True},
    {"min_lane_width_m": 1.6}, {"max_lane_width_m": math.inf}, {"max_curvature_per_m": -0.1},
    {"typical_curvature_per_m": 0.0}, {"outlier_distance_m": -0.1},
])
def test_invalid_settings_are_rejected(change):
    with pytest.raises(ValueError):
        dataclasses.replace(CONFIG, **change)


# ------------------------------------------------------------------------------------------------
# Recorded real scenarios (straight lanes, blue on the left), after the cone filter as in MovementPolicy.step
# ------------------------------------------------------------------------------------------------
def detect_scenario(name):
    """:return: The filtered cones and the lane detected at each step of a recorded scenario."""
    steps = []
    for observations, _ in Scenario.from_name(ScenarioType.REAL, name).steps():
        cones = filter_cones(observations.cones, ConeFilterConfig())
        steps.append((cones, detect_lanes(dataclasses.replace(observations, cones=cones), CONFIG)))
    return steps


@pytest.mark.parametrize("name", ["straight_even", "messy_cone", "outlier_cone", "long_track"])
def test_real_scenarios_find_a_steady_straight_lane(name):
    steps = detect_scenario(name)
    # Until the lane end, both rows have several cones reaching a metre ahead, so both edges must be found. Near the
    # end, the few cones left are inside a metre, where the detections are rough, and no lane is a correct answer.
    for cones, lane in steps:
        rows = [[cone for cone in cones if cone.colour == colour] for colour in (BLUE, YELLOW)]
        if all(len(row) >= 3 and max(cone.pos.x for cone in row) >= 1.0 for row in rows):
            assert lane.status == LaneStatus.BOTH_EDGES
    both = [lane for _, lane in steps if lane.status == LaneStatus.BOTH_EDGES]
    assert len(both) >= 0.3 * len(steps)
    widths = [lane.lane_width_m for lane in both]
    assert statistics.pstdev(widths) < 0.05
    assert 0.6 < statistics.median(widths) < 1.0
    # The car stays in the lane
    assert all(abs(lane.lateral_error_m) < lane.lane_width_m / 2.0 for lane in both)


@pytest.mark.parametrize("name", ["straight_even", "outlier_cone", "long_track"])
def test_real_straight_lanes_look_gently_curved(name):
    # Recorded behaviour, not a requirement: with curvature allowed, these straight lanes read as curving gently left
    # (a 6-12 m radius) in almost every step, so the detected cone positions bend straight rows slightly. This is why
    # the default fits straight edges.
    curvatures = []
    for observations, _ in Scenario.from_name(ScenarioType.REAL, name).steps():
        cones = filter_cones(observations.cones, ConeFilterConfig())
        lane = detect_lanes(dataclasses.replace(observations, cones=cones), CURVED)
        if lane.status == LaneStatus.BOTH_EDGES:
            curvatures.append(lane.path_curvature_per_m)
    assert sum(curvature > 0.0 for curvature in curvatures) >= 0.9 * len(curvatures)
    assert 0.05 < statistics.median(curvatures) < 0.2
