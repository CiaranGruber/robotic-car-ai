"""Verify angular test geometry independently of lane fitting/control performance."""
import math

import pytest

from policy.input_output import ConeColour
from tests.policy.angular_lanes import AngularUTurn


def test_cones_lie_on_three_straight_segments_per_boundary_without_arc_samples():
    lane = AngularUTurn(placement_jitter_m=0)
    for colour, _, x, y in lane.cones():
        offset = lane.width_m / 2 * (1 if colour == ConeColour.BLUE else -1)
        assert (y == pytest.approx(offset) or x == pytest.approx(lane.leg_length_m - offset)
                or y == pytest.approx(lane.cross_length_m - offset))
    line = lane.centre_line()
    assert set(line["heading"]) == {0, math.pi / 2, math.pi}
    assert line["s"][-1] == pytest.approx(2 * lane.leg_length_m + lane.cross_length_m)


@pytest.mark.parametrize("x,y,inside", [(1, 0, True), (3, 1, True), (1, 3, True),
                                       (1, 1, False), (3.5, 1, False), (3.3, 0.3, True),
                                       (-0.2, 0, True), (-0.2, 3, True)])
def test_corridor_membership_uses_polygonal_boundaries(x, y, inside):
    assert (AngularUTurn().outside_distance(x, y) < 0) == inside


def test_irregular_cone_placement_is_reproducible_and_not_mistaken_for_sensor_noise():
    lane = AngularUTurn()
    assert lane.cones() == AngularUTurn().cones()
    assert lane.cones() != AngularUTurn(seed=18).cones()
    assert lane.cones() != AngularUTurn(placement_jitter_m=0).cones()


def test_vehicle_side_crossing_inner_notch_is_detected_with_all_corners_inside():
    lane = AngularUTurn()
    corners = ((2.4, 0.35), (2.65, 0.1), (2.9, 0.35), (2.65, 0.6))
    assert all(lane.outside_distance(x, y) < 0 for x, y in corners)
    assert lane.footprint_outside_distance(corners) > 0
