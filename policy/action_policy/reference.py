"""
reference.py

TEMPORARY: this file builds the MPC's reference path directly from the cone detections, until the lane detection
module's output is agreed (Task 1.4 in docs/movement-policy-tasks.md). Then replace reference_from_cones with a
conversion from LaneDetection.

It assumes a straight lane (Stage 1).
"""
import math

import numpy as np

from policy.action_policy.mpc import PathTrackingState
from policy.input_output import ConeColour, ConeDetection

MIN_ROW_LENGTH_M = 0.25
"""Smallest forwards distance in metres between the nearest and farthest cone of a row for it to be fitted."""


def reference_from_cones(cones: list[ConeDetection] | None, lane_width_m: float) -> PathTrackingState | None:
    """Finds where the car is relative to the centre of a straight lane from the cone rows.

    Each cone colour is one lane boundary, fitted with a straight line in base_link. With both boundaries, the
    centre line is halfway between them. With one, the centre line is half a lane width from it, on the car's side.
    Which colour is on which side is not assumed.

    :param cones: The cone detections for this policy step, or None when none are available.
    :param lane_width_m: Distance between the lane boundaries in metres, used when only one is visible.
    :return: Where the car is relative to the lane centre, or None when the lane is missing or doubtful: no
        boundary has enough cones, or both boundaries are on the same side of the car.
    """
    if cones is None:
        return None
    # Fit each boundary as y = intercept + slope * x
    boundaries = []
    for colour in ConeColour:
        xs = [cone.pos.x for cone in cones if cone.colour == colour]
        ys = [cone.pos.y for cone in cones if cone.colour == colour]
        if len(xs) >= 2 and max(xs) - min(xs) >= MIN_ROW_LENGTH_M:
            slope, intercept = np.polyfit(xs, ys, 1)
            boundaries.append((float(intercept), float(slope)))
    if len(boundaries) == 2:
        (intercept_a, slope_a), (intercept_b, slope_b) = boundaries
        if intercept_a * intercept_b >= 0.0:
            return None
        intercept, slope = (intercept_a + intercept_b) / 2.0, (slope_a + slope_b) / 2.0
    elif len(boundaries) == 1:
        boundary_intercept, slope = boundaries[0]
        if boundary_intercept == 0.0:
            return None
        # Moving a line half a lane width sideways changes its intercept by this much
        half_width_intercept = lane_width_m / 2.0 * math.hypot(1.0, slope)
        intercept = boundary_intercept - math.copysign(half_width_intercept, boundary_intercept)
    else:
        return None
    return PathTrackingState(
        lateral_error_m=-intercept / math.hypot(1.0, slope),
        heading_error_rad=-math.atan(slope),
        path_curvature_per_m=0.0,
    )
