"""Regressions for GPU-output checks, road-arm association and preservation of real bends."""
from dataclasses import replace
import math
import random

import numpy as np
import pytest

from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones
from policy.input_output import ConeBatch, ConeColour, ConeDetection, Position, SensorAge
from policy.lane_detection.lane_detection import LaneDetectionConfig, _fit_lane_edges, detect_lanes
from policy.lane_detection.lanes import LaneStatus
from tests.policy.intersection_identification import CameraModel, synthetic_frame
from tests.policy.intersection_simulation import CrossIntersection, camera_frame, settings
from tests.policy.intersection_turning import track_for
from tests.policy.test_lane_detection import CURVED, lane_cones, observe


@pytest.mark.parametrize("field", ("x", "y", "z", "confidence"))
@pytest.mark.parametrize("invalid", (math.nan, math.inf, -math.inf))
def test_gpu_output_with_nonfinite_position_or_confidence_is_removed(field, invalid):
    valid = ConeDetection(Position(1., .5, .1), ConeColour.BLUE, .9)
    bad = replace(valid, confidence=invalid) if field == "confidence" else replace(
        valid, pos=replace(valid.pos, **{field: invalid}))
    batch = ConeBatch([bad, valid], SensorAge(.2, 123), .08)
    result = filter_cones(batch, ConeFilterConfig(check_straight_rows=False))
    assert list(result) == [valid]
    assert result.sensor_age is batch.sensor_age
    assert result.acquisition_to_publish_latency_s == .08


def test_equal_confidence_duplicate_choice_does_not_depend_on_packet_order():
    a = ConeDetection(Position(1., .5, .1), ConeColour.BLUE, .9)
    b = replace(a, pos=Position(1.02, .52, .1))
    config = ConeFilterConfig(check_straight_rows=False)
    for cones in ([a, b], [b, a]):
        kept = filter_cones(ConeBatch(cones, SensorAge(.1, 1), .03), config)
        assert len(kept) == 1 and kept[0] is a


def test_finite_large_forward_distance_does_not_overflow_duplicate_sort():
    cone = ConeDetection(Position(1e200, .5, .1), ConeColour.BLUE, .9)
    kept = filter_cones(ConeBatch([cone], SensorAge(.1, 1), .03), ConeFilterConfig(check_straight_rows=False))
    assert list(kept) == [cone]


@pytest.mark.parametrize("heading_deg", (-20., 0., 20.))
@pytest.mark.parametrize("offset_m", (-.15, .15))
@pytest.mark.parametrize("left_colour", ("AUTO", "BLUE"))
def test_incoming_arm_is_found_with_rotated_and_offset_car(heading_deg, offset_m, left_colour):
    raw, _ = synthetic_frame(CrossIntersection(width_m=1.4, cone_spacing_m=.4),
                             (-3.5, offset_m, math.radians(heading_deg)), CameraModel(),
                             "clean", np.random.default_rng(23))
    cones = filter_cones(ConeBatch(raw, SensorAge(.2, 42), .08), ConeFilterConfig(check_straight_rows=False))
    original_ids = [id(p) for p in cones]
    lane = detect_lanes(observe(cones), replace(CURVED, left_colour=left_colour, lane_width_m=1.4))
    assert lane.has_lane
    assert lane.heading_error_rad == pytest.approx(math.radians(heading_deg), abs=math.radians(3))
    assert lane.lateral_error_m == pytest.approx(offset_m, abs=.06)
    assert lane.lane_width_m == pytest.approx(1.4, abs=.06)
    assert [id(p) for p in cones] == original_ids  # No side-road cones removed from observations.


def test_dense_cross_fit_membership_is_invariant_under_input_order():
    raw, _ = synthetic_frame(CrossIntersection(width_m=1.4, cone_spacing_m=.05), (-.5, 0., 0.),
                             CameraModel(), "clean", np.random.default_rng(23))
    config = replace(CURVED, left_colour="BLUE")
    memberships = []
    for seed in range(12):
        shuffled = list(raw)
        random.Random(seed).shuffle(shuffled)
        rows = {colour: [p for p in shuffled if p.colour is colour] for colour in ConeColour}
        fit = _fit_lane_edges(rows, config)
        memberships.append(None if fit is None else frozenset(id(p) for row in fit[2].values() for p in row))
    assert len(set(memberships)) == 1


@pytest.mark.parametrize("turn", (-1., 1.))
def test_full_uturn_with_an_outlier_keeps_its_curvature(turn):
    cones = lane_cones(radius_m=1., turn=turn, length_m=math.pi, spacing_m=.4)
    cones.append(ConeDetection(Position(1.6, -.1*turn, .05), ConeColour.BLUE, .9))
    lane = detect_lanes(observe(cones), CURVED)
    assert lane.status is LaneStatus.BOTH_EDGES
    assert lane.path_curvature_per_m == pytest.approx(turn, rel=.03)
    assert lane.heading_error_rad == pytest.approx(0., abs=.03)


@pytest.mark.parametrize("offset", (-.6, .6))
@pytest.mark.parametrize("turn", (-1., 1.))
def test_coherent_curve_remains_visible_for_recovery_from_outside_the_lane(offset, turn):
    cones = lane_cones(radius_m=2., turn=turn, offset_m=offset, length_m=2*math.pi, spacing_m=.4)
    lane = detect_lanes(observe(cones), CURVED)
    assert lane.status is LaneStatus.BOTH_EDGES
    assert lane.lateral_error_m == pytest.approx(offset, abs=.03)
    assert lane.path_curvature_per_m == pytest.approx(turn/2, rel=.03)


def test_a_transverse_row_without_incoming_support_is_uncertain():
    cones = [ConeDetection(Position(2., y, .1), ConeColour.BLUE, .9) for y in (-.8, -.4, .4, .8)]
    cones.append(ConeDetection(Position(2.7, -.7, .1), ConeColour.YELLOW, .9))
    lane = detect_lanes(observe(cones), CURVED)
    assert lane.status is LaneStatus.NO_LANE


@pytest.mark.parametrize("x", (-4.08, -3.32))
def test_a_dense_transverse_fit_on_one_side_does_not_override_the_wide_incoming_road(x):
    raw, _ = synthetic_frame(CrossIntersection(width_m=2.4, cone_spacing_m=.4), (x, 0., 0.),
                             CameraModel(), "clean", np.random.default_rng(23))
    cones = filter_cones(ConeBatch(raw, SensorAge(.2, 1), .08), ConeFilterConfig(check_straight_rows=False))
    lane = detect_lanes(observe(cones), replace(CURVED, left_colour="BLUE"))
    assert lane.status is LaneStatus.BOTH_EDGES
    assert abs(lane.heading_error_rad) < math.radians(3)
    assert lane.lane_width_m == pytest.approx(2.4, abs=.03)


def test_branch_selection_can_be_disabled_and_rejects_invalid_setting():
    with pytest.raises(ValueError):
        LaneDetectionConfig(associate_branches=1)
    raw, _ = synthetic_frame(CrossIntersection(width_m=1.4, cone_spacing_m=.4), (-3.5, 0., 0.),
                             CameraModel(), "clean", np.random.default_rng(23))
    baseline = detect_lanes(observe(raw), replace(CURVED, left_colour="BLUE", associate_branches=False))
    current = detect_lanes(observe(raw), replace(CURVED, left_colour="BLUE"))
    assert current.has_lane and abs(current.heading_error_rad) < math.radians(3)
    assert not baseline.has_lane or abs(baseline.heading_error_rad) > math.radians(30)


@pytest.mark.parametrize("closed", ("left", "right"))
@pytest.mark.parametrize("x", (-3.23, -3.07, -2.9, -2.8))
def test_t_corner_cannot_become_an_accepted_sideways_curve_on_the_approach(closed, x):
    track = replace(track_for(.2, corner_anchored=True), closed_exit=closed)
    cones = camera_frame(track, (x, 0., 0.), np.random.default_rng(0), "nominal")
    params, _ = settings()
    lane = detect_lanes(observe(cones), LaneDetectionConfig(**params["lane_detection"]))
    if lane.has_lane:
        assert abs(lane.heading_error_rad) < math.radians(3)
        assert abs(lane.path_curvature_per_m) < .03
        assert lane.lane_width_m == pytest.approx(2.4, abs=.03)


@pytest.mark.parametrize("turn", (-1., 1.))
def test_noisy_normal_curve_with_a_transverse_outlier_keeps_accepted_curvature(turn):
    cones = lane_cones(radius_m=2., turn=turn, length_m=2.5, spacing_m=.25)
    rng = np.random.default_rng(26)
    cones = [replace(p, pos=replace(p.pos, x=p.pos.x+rng.normal(0., .01), y=p.pos.y+rng.normal(0., .01))) for p in cones]
    cones.append(ConeDetection(Position(.45, -turn*1.6, .1), ConeColour.BLUE, .9))
    baseline = detect_lanes(observe(cones), replace(CURVED, associate_branches=False))
    current = detect_lanes(observe(cones), CURVED)
    assert baseline.has_lane and current.has_lane
    assert current.path_curvature_per_m == pytest.approx(baseline.path_curvature_per_m, abs=.01)
    assert current.heading_error_rad == pytest.approx(baseline.heading_error_rad, abs=.02)
