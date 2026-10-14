"""
lane_detection.py

This file finds the lane edges, the lane centre line and where the car is in the lane from the filtered cones.

Each cone colour is one lane edge. By default the side of each colour is found from where its edge lies relative to
the car (config.left_colour "AUTO"), so either colour may be on the left. The edges are fitted together as circular
arcs around one centre point (concentric arcs), which is what a lane of constant width and
curvature looks like. With curvature 0 they are parallel straight lines. Fitting both edges together lets a long row
steady a short one, and an edge with too few cones to give a direction alone still gives its offset.

The fit's unknowns are the lane's direction and curvature beside the car, and each edge's sideways offset. It
minimises the cones' distances from their edge at right angles, with a robust loss so a few misplaced cones count
less, then drops cones farther than config.outlier_distance_m from their edge and fits again. A small penalty on
curvature keeps detector noise on a straight lane from looking like a curve. By default the curvature is fixed at 0
(config.max_curvature_per_m), for straight Stage 1 lanes.

detect_lanes keeps no state between policy steps. LaneDetector wraps it and remembers only the lane width measured while
both edges are seen, to place the centre line when one edge is seen, so lanes of any width work without setting their
width. The work is bounded by the batch size and a fixed number of solver iterations, so it is safe to run inside the
real-time policy step.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from policy.input_output import CarObservations, ConeColour, ConeDetection
from policy.lane_detection.lanes import LaneArc, LaneDetection, LaneEdge, LaneStatus

MIN_DIRECTION_SPAN_M = 0.25
"""Smallest distance in metres between two cones of one edge for it to give the lane's direction. At least one found
edge must reach it."""

CONE_POSITION_NOISE_M = 0.05
"""Typical error in metres of a detected cone position, which sets how much the curvature penalty weighs against the
cones' distances from their edges."""

MAX_SOLVER_EVALUATIONS = 50
"""Most evaluations of the fit's distances per solve, which bounds the calculation time."""

MAX_REFITS = 2
"""Most times the fit is repeated after dropping outlier cones."""

WIDTH_SMOOTHING = 0.2
"""Fraction of the difference to each newly measured lane width that LaneDetector's remembered width moves by."""


@dataclass(frozen=True)
class LaneDetectionConfig:
    """Settings for lane detection, loaded from the lane_detection parameters in config/ai4r_policy.yaml.

    These defaults are also the parameters' defaults in policy_node. Invalid settings raise a ValueError, which stops
    policy_node from starting.
    """
    left_colour: str = "AUTO"
    """Colour of the cones on the left edge of the lane: "AUTO", "BLUE" or "YELLOW".

    "AUTO" takes the edge further left as the left edge, and a single edge as left when it is on the car's left. The
    lab floor has had blue on the left on some days and yellow on others, and the Stage 1 simulation cases put yellow
    on the left. "BLUE" or "YELLOW" fixes the sides instead; edges then found the wrong way round give NO_LANE.
    """
    lane_width_m: float = 1.0
    """Lane width in metres used to place the centre line when only one edge is found, until both edges have been
    seen. Must be positive. LaneDetector then uses the width it measured instead. The default is the shared width."""
    min_cones_per_edge: int = 2
    """Fewest cones of one colour for that edge to count as found. Must be an integer of at least 1.

    With 1, a single cone gives an edge's offset when the other edge gives the direction, but a single misdetected
    cone then moves the lane centre.
    """
    min_lane_width_m: float = 0.3
    """Narrowest measured lane width in metres, with both edges found, that is still a plausible lane. Must be
    positive. A narrower pair of edges gives NO_LANE, because the lane is doubtful. The car is about 0.2 m wide."""
    max_lane_width_m: float = 3.0
    """Widest measured lane width in metres that is still a plausible lane. Must be above min_lane_width_m."""
    max_curvature_per_m: float = 0.0
    """Largest lane curvature in 1/m the fit may find, in either direction. Must be at least 0.

    0.0 fits the edges as straight lines, for Stage 1. On the recorded straight lanes, fitting curvature found a false
    gentle left curve (a 6-12 m radius) and doubled the heading error at the car, because the curve is extended back
    to the car from cones 0.5 m and more ahead. For curved roads set 2.0 (a 0.5 m radius), and set the cone filter's
    check_straight_rows to False so it keeps the cones on the curve.
    """
    typical_curvature_per_m: float = 0.5
    """Lane curvature in 1/m that the fit treats as ordinary, 0.5 being a 2 m radius. Must be positive.

    Smaller values pull the fit more strongly towards a straight lane, which steadies straight lanes but flattens
    real curves seen over few cones.
    """
    outlier_distance_m: float = 0.15
    """Cones farther than this in metres from their fitted edge are left out, and the edges fitted again. Must be
    positive. Keep it above the detector's position noise but well below half the lane width."""

    def __post_init__(self):
        if self.left_colour not in ("AUTO", "BLUE", "YELLOW"):
            raise ValueError(f"lane_detection.left_colour must be \"AUTO\", \"BLUE\" or \"YELLOW\", "
                             f"not {self.left_colour!r}")
        for name in ("lane_width_m", "min_lane_width_m", "max_lane_width_m", "typical_curvature_per_m",
                     "outlier_distance_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"lane_detection.{name} must be positive and finite, not {value}")
        if self.min_lane_width_m >= self.max_lane_width_m:
            raise ValueError("lane_detection.min_lane_width_m must be below max_lane_width_m")
        if not (math.isfinite(self.max_curvature_per_m) and self.max_curvature_per_m >= 0.0):
            raise ValueError(f"lane_detection.max_curvature_per_m must be finite and at least 0, "
                             f"not {self.max_curvature_per_m}")
        count = self.min_cones_per_edge
        if isinstance(count, bool) or not isinstance(count, int) or count < 1:
            raise ValueError(f"lane_detection.min_cones_per_edge must be an integer of at least 1, not {count}")


class LaneDetector:
    """Detects the lane at each policy step, remembering the lane width measured while both edges are seen.

    With one edge seen, the centre line is placed half the remembered width from it, so narrow and wide lanes both
    work. Create one when the node starts and call detect once per policy step.
    """

    def __init__(self, config: LaneDetectionConfig):
        """
        :param config: The lane detection settings.
        """
        self.config = config
        self.reset()

    def reset(self):
        """Forgets the measured width. This is done on the first step after entering the publishing-policy state."""
        self.lane_width_m = self.config.lane_width_m
        """Remembered lane width in metres, moved towards each width measured with both edges."""

    def detect(self, observations: CarObservations) -> LaneDetection | None:
        """Detects the lane from the cones, as detect_lanes does, then remembers its width if both edges were seen.

        :param observations: The observations taken by the car at the current policy step, with the cones already
            filtered.
        :return: The detected lane, or None when there is no cone data.
        """
        if observations.policy is not None and observations.policy.is_first_policy_step:
            self.reset()
        lane = detect_lanes(observations, self.config, self.lane_width_m)
        if lane is not None and lane.status == LaneStatus.BOTH_EDGES:
            self.lane_width_m += WIDTH_SMOOTHING * (lane.lane_width_m - self.lane_width_m)
        return lane


def detect_lanes(observations: CarObservations, config: LaneDetectionConfig,
                 lane_width_m: float | None = None) -> LaneDetection | None:
    """Detects the lane edges, centre line and the car's position in the lane from the cones.

    :param observations: The observations taken by the car at the current policy step, with the cones already
        filtered.
    :param config: The lane detection settings.
    :param lane_width_m: Lane width in metres used when only one edge is found, or None for config.lane_width_m.
    :return: The detected lane, or None when there is no cone data (observations.cones is None). A lane with status
        NO_LANE means the cones were fresh but showed no plausible lane.
    """
    cones = observations.cones
    if cones is None:
        return None
    no_lane = LaneDetection(LaneStatus.NO_LANE, None, None, None, None, None, None, None, None, cones.sensor_age)
    rows = {colour: [cone for cone in cones if cone.colour == colour] for colour in ConeColour}
    fit = _fit_concentric_edges(rows, config)
    if fit is None:
        return no_lane
    reference, offsets, rows = fit
    left_colour, right_colour = _sides(offsets, config.left_colour)
    one_edge_width = config.lane_width_m if lane_width_m is None else lane_width_m

    if left_colour is not None and right_colour is not None:
        status = LaneStatus.BOTH_EDGES
        lane_width = offsets[left_colour] - offsets[right_colour]
        if not config.min_lane_width_m <= lane_width <= config.max_lane_width_m:
            return no_lane
        centre_offset = (offsets[left_colour] + offsets[right_colour]) / 2.0
    elif left_colour is not None:
        status, lane_width = LaneStatus.LEFT_EDGE_ONLY, one_edge_width
        centre_offset = offsets[left_colour] - lane_width / 2.0
    else:
        status, lane_width = LaneStatus.RIGHT_EDGE_ONLY, one_edge_width
        centre_offset = offsets[right_colour] + lane_width / 2.0
    # An edge or the centre line past the lane's turning centre is not a lane
    if any(reference.curvature_per_m * offset >= 0.9 for offset in (*offsets.values(), centre_offset)):
        return no_lane

    edges = {colour: _make_edge(colour, rows[colour], reference.offset(offset)) for colour, offset in offsets.items()}
    centre_line = reference.offset(centre_offset)
    return LaneDetection(
        status=status,
        left_edge=edges.get(left_colour),
        right_edge=edges.get(right_colour),
        centre_line=centre_line,
        lane_width_m=lane_width,
        visible_until_m=max(centre_line.project(cone.pos.x, cone.pos.y) for row in rows.values() for cone in row),
        # The reference arc starts at the car's origin, so the car is this far from the centre line, at right angles
        lateral_error_m=-centre_offset,
        heading_error_rad=-reference.heading_rad,
        path_curvature_per_m=centre_line.curvature_per_m,
        sensor_age=cones.sensor_age,
    )


def _sides(offsets: dict[ConeColour, float], left_colour: str) -> tuple[ConeColour | None, ConeColour | None]:
    """Decides which found edge is the lane's left edge and which its right.

    :param offsets: Each found edge's sideways offset from the car in metres, positive to the left, keyed by colour.
    :param left_colour: config.left_colour.
    :return: The colour of the left edge and of the right edge, each None when that edge was not found.
    """
    if left_colour != "AUTO":
        left = ConeColour[left_colour]
        right = ConeColour.YELLOW if left == ConeColour.BLUE else ConeColour.BLUE
        return (left if left in offsets else None), (right if right in offsets else None)
    if len(offsets) == 2:
        left, right = sorted(offsets, key=offsets.get, reverse=True)
        return left, right
    ((colour, offset),) = offsets.items()
    return (colour, None) if offset > 0.0 else (None, colour)


def _fit_concentric_edges(
        rows: dict[ConeColour, list[ConeDetection]],
        config: LaneDetectionConfig) -> tuple[LaneArc, dict[ConeColour, float], dict[ConeColour, list[ConeDetection]]] | None:
    """Fits each row as an arc at its own sideways offset from one reference arc, which starts at the car's origin.

    :param rows: The cones of each colour.
    :param config: The lane detection settings.
    :return: The reference arc, each found edge's offset from it in metres (positive to the left), and the cones of
        each found edge left after dropping outliers; or None when no edge has enough cones to give a direction.
    """
    for _ in range(MAX_REFITS + 1):
        rows = {colour: row for colour, row in rows.items() if len(row) >= config.min_cones_per_edge}
        if not any(_span(row) >= MIN_DIRECTION_SPAN_M for row in rows.values()):
            return None
        colours = list(rows)
        xs = np.array([cone.pos.x for colour in colours for cone in rows[colour]])
        ys = np.array([cone.pos.y for colour in colours for cone in rows[colour]])
        edge_index = np.array([i for i, colour in enumerate(colours) for _ in rows[colour]])
        solution = _solve(xs, ys, edge_index, len(colours), config)
        heading, curvature, offsets = solution[0], solution[1], solution[2:]
        reference = LaneArc(0.0, 0.0, float(heading), float(curvature))
        distances = np.array([reference.signed_distance(x, y) for x, y in zip(xs, ys)]) - offsets[edge_index]
        inlier = np.abs(distances) <= config.outlier_distance_m
        if inlier.all():
            return reference, {colour: float(offsets[i]) for i, colour in enumerate(colours)}, rows
        rows = {colour: [cone for cone, keep in zip(rows[colour], inlier[edge_index == i]) if keep]
                for i, colour in enumerate(colours)}
    return None


def _solve(xs: np.ndarray, ys: np.ndarray, edge_index: np.ndarray, edge_count: int,
           config: LaneDetectionConfig) -> np.ndarray:
    """Finds the reference arc's heading and curvature and each edge's offset that best fit the cones.

    :param xs: Cone distances forwards in metres.
    :param ys: Cone distances left in metres.
    :param edge_index: Which edge each cone belongs to, from 0 to edge_count - 1.
    :param edge_count: Number of edges.
    :param config: The lane detection settings.
    :return: [heading in radians, curvature in 1/m, offset of edge 0 in metres, offset of edge 1...].
    """
    squared_ranges = xs * xs + ys * ys
    prior_weight = CONE_POSITION_NOISE_M / config.typical_curvature_per_m
    one_hot = np.zeros((len(xs), edge_count))
    one_hot[np.arange(len(xs)), edge_index] = 1.0

    def parts(params):
        heading, curvature = params[0], params[1]
        along = xs * math.cos(heading) + ys * math.sin(heading)
        left = -xs * math.sin(heading) + ys * math.cos(heading)
        numerator = 2.0 * left - curvature * squared_ranges
        root = np.sqrt(np.maximum(1.0 - curvature * numerator, 1e-12))
        return along, left, numerator, root, 1.0 + root

    def residuals(params):
        _, _, numerator, _, denominator = parts(params)
        distances = numerator / denominator - params[2:][edge_index]
        return np.append(distances, prior_weight * params[1])

    def jacobian(params):
        curvature = params[1]
        along, left, numerator, root, denominator = parts(params)
        d_heading = (-2.0 * along * denominator - numerator * curvature * along / root) / denominator ** 2
        d_curvature = (-squared_ranges * denominator
                       - numerator * (curvature * squared_ranges - left) / root) / denominator ** 2
        prior = np.zeros(2 + edge_count)
        prior[1] = prior_weight
        return np.vstack([np.column_stack([d_heading, d_curvature, -one_hot]), prior])

    # Start from the best straight lines, which a linear least-squares fit finds directly
    straight = np.linalg.lstsq(np.column_stack([one_hot, xs]), ys, rcond=None)[0]
    heading = math.atan(straight[-1])
    initial = np.concatenate([[heading, 0.0], straight[:edge_count] * math.cos(heading)])
    limit = config.max_curvature_per_m
    if limit == 0.0:
        return initial
    # On a sharp turn the straight lines are a poor start, so also try the best concentric circles, which are
    # x^2 + y^2 = 2 a x + 2 b y + d_edge around the centre (a, b) and a linear least-squares fit too
    circles = np.linalg.lstsq(np.column_stack([2.0 * xs, 2.0 * ys, one_hot]), squared_ranges, rcond=None)[0]
    (a, b), squared_radii = circles[:2], circles[2:] + circles[0] ** 2 + circles[1] ** 2
    centre_distance = math.hypot(a, b)
    if centre_distance > 0.0 and (squared_radii > 0.0).all():
        # The lane ahead runs at right angles to the line from the car to the centre; a centre on the left turns left
        turn = 1.0 if b >= 0.0 else -1.0
        circle_start = np.concatenate([[math.atan2(-turn * a, turn * b), turn / centre_distance],
                                       turn * (centre_distance - np.sqrt(squared_radii))])
        if abs(circle_start[1]) < limit and abs(circle_start[0]) < math.pi / 2.0:
            initial = min(initial, circle_start, key=lambda start: float(np.sum(residuals(start) ** 2)))
    lower = np.concatenate([[-math.pi / 2.0, -limit], np.full(edge_count, -np.inf)])
    upper = np.concatenate([[math.pi / 2.0, limit], np.full(edge_count, np.inf)])
    initial[1] = np.clip(initial[1], -limit, limit)
    result = least_squares(residuals, initial, jac=jacobian, bounds=(lower, upper), loss="soft_l1",
                           f_scale=config.outlier_distance_m / 3.0, max_nfev=MAX_SOLVER_EVALUATIONS)
    return result.x


def _span(row: list[ConeDetection]) -> float:
    """
    :return: Largest distance in metres between two cones of the row.
    """
    return max((math.hypot(a.pos.x - b.pos.x, a.pos.y - b.pos.y) for a in row for b in row), default=0.0)


def _make_edge(colour: ConeColour, row: list[ConeDetection], arc: LaneArc) -> LaneEdge:
    """
    :return: The edge fitted to the row, with its fit statistics.
    """
    along = [arc.project(cone.pos.x, cone.pos.y) for cone in row]
    distances = [arc.signed_distance(cone.pos.x, cone.pos.y) for cone in row]
    return LaneEdge(
        arc=arc,
        colour=colour,
        cone_count=len(row),
        nearest_m=min(along),
        farthest_m=max(along),
        rms_residual_m=math.sqrt(sum(d * d for d in distances) / len(distances)),
    )
