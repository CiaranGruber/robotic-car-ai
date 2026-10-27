"""Checks that the diagnostic scores physical geometry and never mistakes sign memory for a turn."""
import math
from dataclasses import asdict

import numpy as np
import pytest

from tests.policy.intersection_simulation import (CrossIntersection, lidar_frame, marker_frame, rollout, settings)
from tests.policy.stage1_scenarios import body_corners


@pytest.mark.parametrize("spacing", [.5, .6])
def test_sparse_cross_keeps_corners_and_uniform_symmetric_rows(spacing):
    track = CrossIntersection(width_m=2.4, arm_length_m=8., cone_spacing_m=spacing, corner_anchored=True)
    restored = CrossIntersection(**asdict(track))
    positions = {(round(x, 8), round(y, 8)) for x, y, _ in restored.cones()}
    assert len(positions) == len(restored.cones())
    assert {(x, y) for x in (-1.2, 1.2) for y in (-1.2, 1.2)} <= positions
    assert {(-x, -y) for x, y in positions} == positions
    # Every arm starts at the same physical corner, irrespective of sampling density.
    for fixed_axis in (0, 1):
        for side in (-1.2, 1.2):
            for sign in (-1, 1):
                row = sorted(sign*p[1-fixed_axis] for p in positions
                             if p[fixed_axis] == side and sign*p[1-fixed_axis] >= 1.2)
                assert row[0] == 1.2
                assert np.allclose(np.diff(row), spacing)
    assert track.body_outside_m((0., 0., 0.)) == 0.


def test_legacy_cross_sampling_remains_available_for_saved_comparisons():
    track = CrossIntersection(width_m=2.4, arm_length_m=8., cone_spacing_m=.6)
    incoming_row = sorted(x for x, y, _ in track.cones() if y == 1.2)
    assert max(x for x in incoming_row if x < 0.) == pytest.approx(-1.4)
    assert min(x for x in incoming_row if x > 0.) == pytest.approx(1.6)


def test_cross_body_scoring_catches_an_edge_cut_with_every_corner_on_road():
    track = CrossIntersection()
    pose = (.45, .45, -math.pi/4)
    assert all(track.point_outside_m(*p) <= 0. for p in body_corners(*pose))
    assert track.body_outside_m(pose) > 0.
    assert track.body_outside_m((-2., 0., 0.)) == 0.


def test_exit_gate_requires_the_actual_arm_and_heading():
    track = CrossIntersection()
    assert track.exit_at((0., 0., 0.)) is None
    assert track.exit_at((2.2, 0., 0.)) == "straight"
    assert track.exit_at((2.2, 0., math.pi/2)) is None
    assert track.exit_at((0., 2.2, math.pi/2)) == "left"
    assert track.exit_at((0., -2.2, -math.pi/2)) == "right"


def test_lidar_scan_indices_preserve_angular_adjacency_for_obstacle_grouping():
    scan = lidar_frame(CrossIntersection(), (-.6, .1, .2))
    bearings = [math.atan2(p.pos.y, p.pos.x) for p in sorted(scan.points, key=lambda p: p.scan_index)]
    assert bearings == sorted(bearings)


def test_synthetic_detector_gate_drops_single_image_and_restarts_confirmation():
    track, pose = CrossIntersection(), (-3.5, 0., 0.)
    _, detector = settings()
    consecutive = 0
    for step in range(detector["min_consecutive_frames"]):
        markers, consecutive = marker_frame(track, pose, 3, "nominal", step, detector, consecutive)
        assert bool(markers) == (step == detector["min_consecutive_frames"]-1)
    markers, consecutive = marker_frame(track, pose, 3, "missing_marker", 4, detector, consecutive)
    assert not markers and consecutive == 0
    markers, consecutive = marker_frame(track, pose, 3, "nominal", 5, detector, consecutive)
    assert not markers and consecutive == 1


def test_command_retained_after_the_marker_disappears_is_not_navigation_success():
    result = rollout("left", "marker_disappears")
    metrics = result["metrics"]
    assert metrics["remembered_command"] == "TURN_LEFT"
    assert metrics["marker_disappeared_at_s"] > metrics["recognised_at_s"]
    if metrics["actual_exit"] != "left":
        assert not metrics["navigation_pass"]
