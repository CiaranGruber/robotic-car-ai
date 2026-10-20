"""Tests for the MPC's lane offset and lane side (towards the hook turn), its separate left and right curvature
limits, and the curved-lane simulation (curved_lanes.py) that checks the MPC on curves."""
import dataclasses
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import curved_lanes as cl  # noqa: E402
from policy.action_policy.mpc import (  # noqa: E402
    MpcConfig, MpcController, PathTrackingState, inside_offset_limit_m, offset_state,
)
from policy.action_policy.policy import MpcPolicy  # noqa: E402
from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones  # noqa: E402
from policy.lane_detection.lane_detection import LaneDetectionConfig, LaneDetector  # noqa: E402
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


@pytest.mark.parametrize("requested, width, expected", [(0.05, 0.8, 0.0), (0.05, 1.2, 0.05), (0.4, 1.2, 0.2),
                                                        (-0.4, 1.6, -0.4), (0.2, 0.7, 0.0)])
def test_followed_offset_keeps_clear_of_the_lane_edges(requested, width, expected):
    policy = MpcPolicy(MpcConfig(lane_offset_m=requested))
    assert policy.followed_offset_m(width) == pytest.approx(expected)


@pytest.mark.parametrize("side, width, expected", [("LEFT", 1.2, 0.2), ("RIGHT", 1.2, -0.2), ("LEFT", 1.6, 0.4),
                                                   ("RIGHT", 1.6, -0.4), ("LEFT", 0.8, 0.0), ("CENTRE", 1.6, 0.0)])
def test_lane_side_follows_the_line_edge_clearance_from_that_edge(side, width, expected):
    assert MpcPolicy(MpcConfig(lane_side=side)).followed_offset_m(width) == pytest.approx(expected)


def test_inside_offset_limit_stops_halfway_to_the_turning_centre():
    assert inside_offset_limit_m(0.0) == math.inf
    assert inside_offset_limit_m(2.0) == pytest.approx(0.25)
    assert inside_offset_limit_m(-1.0) == pytest.approx(0.5)


@pytest.mark.parametrize("side, curvature, expected", [("LEFT", 2.0, 0.25), ("RIGHT", 2.0, -0.4),
                                                       ("RIGHT", -2.0, -0.25), ("LEFT", -2.0, 0.4),
                                                       ("LEFT", 0.5, 0.4)])
def test_lane_side_stays_off_the_turning_centre_on_tight_curves(side, curvature, expected):
    """Only the inside of a tight curve is limited; the outside keeps the full offset."""
    policy = MpcPolicy(MpcConfig(lane_side=side))
    assert policy.followed_offset_m(1.6, curvature) == pytest.approx(expected)


def test_side_change_moves_the_line_over_gradually():
    policy = MpcPolicy(MpcConfig(lane_side="LEFT", offset_change_per_m=0.2))
    policy._move_side_offset(1.6, 0.0, 0.0)
    assert policy.usual_offset_m(1.6) == pytest.approx(0.4)
    policy.lane_side = "RIGHT"
    policy._move_side_offset(1.6, 0.0, 0.5)
    assert policy.usual_offset_m(1.6) == pytest.approx(0.3)
    for _ in range(10):
        policy._move_side_offset(1.6, 0.0, 0.5)
    assert policy.usual_offset_m(1.6) == pytest.approx(-0.4)


def test_narrowing_lane_moves_the_line_in_at_once():
    policy = MpcPolicy(MpcConfig(lane_side="LEFT"))
    policy._move_side_offset(1.6, 0.0, 0.0)
    policy._move_side_offset(1.0, 0.0, 0.05)
    assert policy.usual_offset_m(1.0) == pytest.approx(0.1)
    policy._move_side_offset(1.6, 0.0, 0.5)
    assert policy.usual_offset_m(1.6) == pytest.approx(0.2)


def test_new_run_starts_at_the_configured_lane_side():
    """A side a manoeuvre chose in the previous run is forgotten: after an explicit resume the car starts at
    mpc.lane_side and mpc.lane_offset_m again."""
    policy = MpcPolicy(MpcConfig(lane_side="LEFT", lane_offset_m=0.1))
    policy._move_side_offset(1.6, 0.0, 0.0)
    policy.lane_side, policy.lane_offset_m = "RIGHT", -0.2
    policy._move_side_offset(1.6, 0.0, 10.0)
    assert policy.usual_offset_m(1.6) == pytest.approx(-0.4)
    policy.reset()
    assert (policy.lane_side, policy.lane_offset_m) == ("LEFT", 0.1)
    policy._move_side_offset(1.6, 0.0, 0.0)
    assert policy.usual_offset_m(1.6) == pytest.approx(0.4)


def test_offset_steers_a_centred_car_towards_that_side():
    controller = MpcController(MpcConfig())
    left = controller.solve(centred_state(), 0.0, 0.0, lateral_offset_m=0.1).curvatures_per_m[0]
    right = controller.solve(centred_state(), 0.0, 0.0, lateral_offset_m=-0.1).curvatures_per_m[0]
    assert left > 0.0 > right
    assert controller.solve(centred_state(), 0.0, 0.0).curvatures_per_m[0] == pytest.approx(0.0, abs=1e-9)


def lane_detection():
    """Lane detection with curves on, as config/ai4r_policy.yaml sets it, after the cone filter."""
    filter_config = ConeFilterConfig(max_forward_m=2.5, check_straight_rows=False)
    detector = LaneDetector(LaneDetectionConfig(max_curvature_per_m=2.0))
    return lambda observations: detector.detect(
        dataclasses.replace(observations, cones=filter_cones(observations.cones, filter_config)))


@pytest.mark.parametrize("offset", [0.1, -0.1])
def test_car_settles_at_the_requested_offset(offset):
    lane = cl.CurvedLane(pieces=(("straight", 5.0),), width_m=0.8)
    config = MpcConfig(lane_offset_m=offset, edge_clearance_m=0.30)
    policy = ConeFilteredPolicy(MpcPolicy(config), ConeFilterConfig())
    result = cl.run_curve(cl.CurveCase(lane, car=NO_TRIM_CAR), policy)
    assert result.lane_kept
    assert np.mean(result.lateral_m[result.t > 6.0]) == pytest.approx(offset, abs=0.03)


@pytest.mark.parametrize("side, expected", [("LEFT", 0.2), ("RIGHT", -0.2)])
def test_car_keeps_to_one_side_round_a_curve(side, expected):
    lane = cl.CurvedLane(pieces=(("straight", 1.5), ("arc", 1.0 / 3.0, 90.0), ("straight", 2.0)), width_m=1.2)
    result = cl.run_curve(cl.CurveCase(lane, car=NO_TRIM_CAR), MpcPolicy(MpcConfig(lane_side=side)), lane_detection())
    assert result.lane_kept
    assert np.median(result.lateral_m[result.t > 3.0]) == pytest.approx(expected, abs=0.04)


class _SwitchSide:
    """Changes MpcPolicy.lane_side after a time, as a manoeuvre would."""

    def __init__(self, policy, side, at_s):
        self.policy, self.side, self.at_s = policy, side, at_s

    def run_policy(self, observations, lanes):
        if observations.policy.policy_elapsed_s >= self.at_s:
            self.policy.lane_side = self.side
        return self.policy.run_policy(observations, lanes)


def test_car_moves_across_to_the_other_side_without_touching_a_cone():
    lane = cl.CurvedLane(pieces=(("straight", 8.0),), width_m=1.6)
    policy = _SwitchSide(MpcPolicy(MpcConfig(lane_side="LEFT")), "RIGHT", at_s=7.0)
    result = cl.run_curve(cl.CurveCase(lane, car=NO_TRIM_CAR), policy, lane_detection())
    assert result.lane_kept
    assert np.median(result.lateral_m[(result.t > 4.0) & (result.t < 7.0)]) == pytest.approx(0.4, abs=0.04)
    assert np.median(result.lateral_m[result.t > 17.0]) == pytest.approx(-0.4, abs=0.04)


@pytest.mark.parametrize("kwargs", [{"lane_offset_m": math.nan}, {"edge_clearance_m": 0.0},
                                    {"offset_change_per_m": 0.0}, {"lane_side": "left"}, {"lane_side": "MIDDLE"},
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
