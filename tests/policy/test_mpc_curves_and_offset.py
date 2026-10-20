"""Tests for the MPC's lane offset (towards the hook turn), its separate left and right curvature limits, and the
curved-lane simulation (curved_lanes.py) that checks the MPC on curves."""
import dataclasses
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import curved_lanes as cl  # noqa: E402
from policy.action_policy.lane_path import LanePath, LineState, PathSegment  # noqa: E402
from policy.action_policy.mpc import MpcConfig, MpcController, PathTrackingState, offset_state  # noqa: E402
from policy.action_policy.policy import MpcPolicy  # noqa: E402
from policy.cone_filter.cone_filter import ConeFilterConfig  # noqa: E402
from real_car_measurements import LAB_CAR  # noqa: E402
from stage1_scenarios import ConeFilteredPolicy  # noqa: E402

PAN_HELD_CAR = dataclasses.replace(LAB_CAR, camera_yaw_deg=0.0)
"""The lab car with the camera pan held at zero."""
NO_TRIM_CAR = dataclasses.replace(PAN_HELD_CAR, straight_curvature_per_m=0.0)
"""The same car with its steering trim compensated by Car Control."""


def centred_state(curvature: float = 0.0) -> PathTrackingState:
    return PathTrackingState(0.0, 0.0, curvature)


# ---------------------------------------------------------------------------------------------------------------
# Lane offset
# ---------------------------------------------------------------------------------------------------------------
def test_offset_moves_the_followed_line_sideways_on_a_straight():
    state = offset_state(PathTrackingState(0.05, 0.1, 0.0), 0.15)
    assert state == PathTrackingState(pytest.approx(-0.10), 0.1, 0.0)


@pytest.mark.parametrize("curvature, offset, expected", [(0.5, 0.2, 0.5 / 0.9), (0.5, -0.2, 0.5 / 1.1),
                                                         (-0.5, -0.2, -0.5 / 0.9)])
def test_offset_line_on_a_curve_is_concentric(curvature, offset, expected):
    """Inside the curve the followed line turns more sharply, outside it less."""
    assert offset_state(centred_state(curvature), offset).path_curvature_per_m == pytest.approx(expected)


def test_offset_past_the_turning_centre_is_refused():
    with pytest.raises(ValueError):
        offset_state(centred_state(2.0), 0.6)


@pytest.mark.parametrize("requested, width, expected", [(0.05, 0.8, 0.05), (0.4, 0.8, 0.1), (-0.4, 0.8, -0.1),
                                                        (0.2, 0.5, 0.0)])
def test_followed_offset_keeps_clear_of_the_lane_edges(requested, width, expected):
    policy = MpcPolicy(MpcConfig(lane_offset_m=requested))
    assert policy.followed_offset_m(width) == pytest.approx(expected)


@pytest.mark.parametrize("curvature, expected", [(0.0, 0.1), (0.15, 0.05), (-0.15, 0.05), (0.3, 0.0), (1.0, 0.0)])
def test_followed_offset_fades_out_on_curves(curvature, expected):
    policy = MpcPolicy(MpcConfig(lane_offset_m=0.1))
    assert policy.followed_offset_m(0.8, curvature) == pytest.approx(expected)


def test_offset_steers_a_centred_car_towards_that_side():
    controller = MpcController(MpcConfig())
    left = controller.solve(centred_state(), 0.0, 0.0, lateral_offset_m=0.1).curvatures_per_m[0]
    right = controller.solve(centred_state(), 0.0, 0.0, lateral_offset_m=-0.1).curvatures_per_m[0]
    assert left > 0.0 > right
    assert controller.solve(centred_state(), 0.0, 0.0).curvatures_per_m[0] == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("curvature", [0.0, 0.3])
def test_a_planned_line_at_one_offset_is_followed_like_the_offset(curvature):
    controller = MpcController(MpcConfig())
    state = PathTrackingState(0.02, 0.05, curvature)
    by_offset = controller.solve(state, 0.0, 0.05, lateral_offset_m=0.1).curvatures_per_m
    by_line = controller.solve(state, 0.0, 0.05, line=LanePath(0.1, ())).curvatures_per_m
    assert by_line == pytest.approx(by_offset)


def test_a_planned_line_moving_across_is_steered_before_it_moves():
    controller = MpcController(MpcConfig())
    # The line moves 0.2 m left over 1.5 m, starting 0.3 m ahead: the car starts turning now, but less than it would
    # for a line 0.2 m left already
    line = LanePath(0.0, (PathSegment.move(0.3, 1.8, LineState(0.0), LineState(0.2)), PathSegment.hold(1.8, 5.0, 0.2)))
    ahead = controller.solve(centred_state(), 0.0, 0.0, line=line).curvatures_per_m[0]
    already = controller.solve(centred_state(), 0.0, 0.0, lateral_offset_m=0.2).curvatures_per_m[0]
    assert 0.0 < ahead < already


@pytest.mark.parametrize("offset", [0.1, -0.1])
def test_car_settles_at_the_requested_offset(offset):
    lane = cl.CurvedLane(pieces=(("straight", 5.0),), width_m=0.8)
    policy = ConeFilteredPolicy(MpcPolicy(MpcConfig(lane_offset_m=offset)), ConeFilterConfig())
    result = cl.run_curve(cl.CurveCase(lane, car=NO_TRIM_CAR), policy)
    assert result.lane_kept
    assert np.mean(result.lateral_m[result.t > 6.0]) == pytest.approx(offset, abs=0.03)


@pytest.mark.parametrize("kwargs", [{"lane_offset_m": math.nan}, {"edge_clearance_m": 0.0},
                                    {"offset_fade_curvature_per_m": 0.0},
                                    {"max_right_curvature_per_m": -0.1}, {"max_right_curvature_per_m": math.inf}])
def test_invalid_offset_and_limit_settings_are_refused(kwargs):
    with pytest.raises(ValueError):
        MpcConfig(**kwargs)


# ---------------------------------------------------------------------------------------------------------------
# Separate left and right curvature limits
# ---------------------------------------------------------------------------------------------------------------
def test_right_limit_of_zero_uses_the_left_one():
    assert MpcController(MpcConfig(max_curvature_per_m=0.48, max_right_curvature_per_m=0.0)).curvature_bounds == (
        -0.48, 0.48)


def test_default_limits_are_car_20s():
    assert MpcController(MpcConfig()).curvature_bounds == (-0.92, 0.48)


def test_plan_stays_within_each_sides_own_limit():
    controller = MpcController(MpcConfig(max_curvature_per_m=0.48, max_right_curvature_per_m=0.92))
    hard_right = controller.solve(PathTrackingState(0.4, 0.5, 0.0), 0.0, 0.0).curvatures_per_m
    hard_left = controller.solve(PathTrackingState(-0.4, -0.5, 0.0), 0.0, 0.0).curvatures_per_m
    assert min(hard_right) == pytest.approx(-0.92)
    assert max(hard_left) == pytest.approx(0.48)


# ---------------------------------------------------------------------------------------------------------------
# The curved-lane simulation
# ---------------------------------------------------------------------------------------------------------------
def test_arc_cones_lie_on_both_edges_around_one_turning_centre():
    radius, width = 2.0, 0.8
    lane = cl.CurvedLane(pieces=(("arc", 1.0 / radius, 90.0),), width_m=width)
    centre = (0.0, radius)  # the arc starts at the start line, after the run-in, heading +x and turning left
    for colour, _, x, y in lane.cones():
        distance = math.hypot(x - centre[0], y - centre[1])
        expected = radius - width / 2 if colour == lane.left_colour else radius + width / 2
        assert distance == pytest.approx(expected, abs=0.01)


def test_centre_line_ends_turned_by_the_arcs():
    lane = cl.CurvedLane(pieces=(("straight", 1.0), ("arc", 0.5, 90.0), ("arc", -0.4, 45.0)))
    assert math.degrees(lane.centre_line()["heading"][-1]) == pytest.approx(45.0, abs=0.1)


def test_detections_see_only_cones_in_view_and_turn_with_the_camera():
    lane = cl.CurvedLane(pieces=(("straight", 4.0),))
    rng = np.random.default_rng(0)
    car = dataclasses.replace(PAN_HELD_CAR, detector_noise_m=0.0)
    seen = cl.detections(lane.cones(), (0.0, 0.0, 0.0), car, rng)
    assert seen and all(car.detector_min_range_m <= d.pos.x <= car.detector_range_m for d in seen)
    yawed = cl.detections(lane.cones(), (0.0, 0.0, 0.0), dataclasses.replace(car, camera_yaw_deg=10.0), rng)
    # A camera pointing left sees the lane turned right: the left row's cones move right
    assert min(d.pos.y for d in yawed) < min(d.pos.y for d in seen)


def test_dev_pipeline_keeps_a_straight_lane():
    policy = ConeFilteredPolicy(MpcPolicy(MpcConfig()), ConeFilterConfig())
    result = cl.run_curve(cl.CurveCase(cl.CurvedLane(pieces=(("straight", 4.0),)), car=PAN_HELD_CAR), policy)
    assert result.lane_kept and result.metrics["max_lateral_m"] < 0.05


def test_measured_speed_pipeline_keeps_the_three_metre_curve():
    """The measured-speed MPC completes this curve without leaving the lane or touching cones."""
    lane = cl.CurvedLane(pieces=(("straight", 1.5), ("arc", 1.0 / 3.0, 90.0), ("straight", 1.5)))
    policy = ConeFilteredPolicy(MpcPolicy(MpcConfig()), ConeFilterConfig())
    result = cl.run_curve(cl.CurveCase(lane, car=PAN_HELD_CAR), policy)
    assert result.lane_kept
    assert result.metrics["cones_touched"] == 0
    assert result.metrics["max_lateral_m"] < 0.05


@pytest.mark.parametrize("speed", [0.0, 0.2, 0.8])
def test_offset_and_measured_speed_use_the_same_prediction_model(speed):
    controller = MpcController(MpcConfig())
    state = PathTrackingState(0.05, 0.1, 0.2)
    actual = controller.solve(state, 0.1, 0.2, speed_m_per_s=speed, lateral_offset_m=0.1)
    expected = controller.solve(offset_state(state, 0.1), 0.1, 0.2, speed_m_per_s=speed)
    assert actual.converged and expected.converged
    np.testing.assert_allclose(actual.curvatures_per_m, expected.curvatures_per_m)
