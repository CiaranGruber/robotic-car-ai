"""Hardware-free checks of the cone outlier filter, without ROS.

Run from the repository root with: python3 -m pytest tests/policy/test_cone_filter.py
"""
import dataclasses
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from policy.action_policy.reference import reference_from_cones  # noqa: E402
from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones  # noqa: E402
from policy.input_output import ConeBatch, ConeColour, ConeDetection, Position, SensorAge  # noqa: E402

YELLOW, BLUE = ConeColour.YELLOW, ConeColour.BLUE
SLOPE = math.tan(math.radians(10.0))  # The car points 10 degrees right of the lane direction


def cone(x, y, colour, z=0.1, confidence=0.9):
    return ConeDetection(Position(x, y, z), colour, confidence)


def lane(xs=(0.5, 1.0, 1.5, 2.0, 2.5, 3.0)):
    """A straight lane, yellow on the left and blue on the right, as rows y = +-0.5 + SLOPE * x."""
    return [cone(x, side * 0.5 + SLOPE * x, colour) for x in xs for side, colour in ((1, YELLOW), (-1, BLUE))]


def batch(cones):
    return ConeBatch(cones, SensorAge(0.08, 123), acquisition_to_publish_latency_s=0.03)


OUTLIERS = [
    cone(1.25, 0.9 + SLOPE * 1.25, YELLOW),  # Off the yellow row
    cone(1.75, -0.5 + SLOPE * 1.75, YELLOW),  # Wrong colour in the blue row
    cone(1.0, 0.52 + SLOPE, YELLOW, confidence=0.6),  # Less confident duplicate of a yellow cone
    cone(5.0, 0.0, BLUE),  # Beyond the range
    cone(-0.3, 0.5, YELLOW),  # Behind the car
    cone(2.0, 2.5, BLUE),  # Too far to the side
    cone(2.0, 0.0, BLUE, z=1.2),  # Too high to be a cone
    cone(2.0, 0.0, BLUE, confidence=0.3),  # Not confident enough
]


def test_removes_every_kind_of_outlier_and_keeps_the_lane():
    cones = batch(lane() + OUTLIERS)
    result = filter_cones(cones, ConeFilterConfig())
    assert isinstance(result, ConeBatch)
    assert list(result) == lane()
    assert result.sensor_age == cones.sensor_age and result.sensor_age.stamp_ns == 123
    assert result.acquisition_to_publish_latency_s == cones.acquisition_to_publish_latency_s


def test_outliers_no_longer_shift_the_lane_reference():
    clean = reference_from_cones(lane(), 1.0)
    raw = reference_from_cones(lane() + OUTLIERS, 1.0)
    filtered = reference_from_cones(filter_cones(batch(lane() + OUTLIERS), ConeFilterConfig()), 1.0)
    assert abs(raw.lateral_error_m - clean.lateral_error_m) > 0.05
    assert filtered.lateral_error_m == pytest.approx(clean.lateral_error_m)
    assert filtered.heading_error_rad == pytest.approx(clean.heading_error_rad)


def test_a_long_row_outvotes_an_outlier_in_a_short_row():
    # Alone, any two of the three yellow cones form a row; the blue row decides which two are right
    yellow = [cone(0.5, 0.5, YELLOW), cone(1.0, 0.5, YELLOW), cone(1.5, 0.9, YELLOW)]
    blue = [cone(x, -0.5, BLUE) for x in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0)]
    assert list(filter_cones(batch(yellow + blue), ConeFilterConfig())) == yellow[:2] + blue


@pytest.mark.parametrize("cones", [
    [],  # A valid empty frame
    [cone(1.0, 0.5, YELLOW), cone(1.0, -0.5, BLUE)],  # One cone per colour gives no row to check against
], ids=["empty", "one-per-colour"])
def test_keeps_the_cones_it_cannot_check(cones):
    result = filter_cones(batch(cones), ConeFilterConfig())
    assert isinstance(result, ConeBatch) and list(result) == cones


def test_curved_lanes_keep_their_cones_without_the_straight_row_check():
    # A lane curving left around a 2 m radius, seen 3 m along
    curve = [cone(r * math.sin(i * 0.5 / 2.0), 2.0 - r * math.cos(i * 0.5 / 2.0), colour)
             for i in range(1, 7) for r, colour in ((1.5, YELLOW), (2.5, BLUE))]
    assert len(filter_cones(batch(curve), ConeFilterConfig())) < len(curve)
    no_rows = dataclasses.replace(ConeFilterConfig(), check_straight_rows=False)
    assert list(filter_cones(batch(curve + OUTLIERS[3:]), no_rows)) == curve


def test_missing_cones_stay_missing():
    assert filter_cones(None, ConeFilterConfig()) is None


@pytest.mark.parametrize("change", [
    {"max_forward_m": 0.0}, {"max_lateral_m": math.inf}, {"merge_distance_m": -0.1}, {"row_residual_m": math.nan},
    {"min_height_m": 0.5}, {"min_confidence": 1.5}, {"max_fit_cones_per_colour": 1},
    {"max_fit_cones_per_colour": 13}, {"max_fit_cones_per_colour": 8.0}, {"check_straight_rows": 1},
])
def test_invalid_settings_are_rejected(change):
    with pytest.raises(ValueError):
        dataclasses.replace(ConeFilterConfig(), **change)
