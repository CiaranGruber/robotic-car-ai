"""
lane_detection.py

This file finds the lane edges, the lane centre line and where the car is in the lane from the filtered cones.

Each cone colour is one lane edge. By default the side of each colour is found from where its edge lies relative to
the car (config.left_colour "AUTO"), so either colour may be on the left. The edges are fitted together as circular
arcs around one centre point (concentric arcs), which is what a lane of constant width and
curvature looks like. With curvature 0 they are parallel straight lines. Fitting both edges together lets a long row
steady a short one, and an edge with too few cones to give a direction alone still gives its offset.

The fit's unknowns are the lane's direction and curvature beside the car, and each edge's sideways offset. First the
cones that agree on one lane are found by RANSAC: small samples of cones each propose a lane, and the lane that the most
cones lie within config.outlier_distance_m of wins. Misdetected cones, or cones of another part of the track in view,
then cannot pull the fit away from the cones that agree, which a fit of all the cones followed by dropping the farthest
does when they pull hard enough. The agreeing cones are then fitted, minimising their distances from their edge at
right angles with a robust loss, and every cone is checked against that fit again. A small penalty on curvature keeps
detector noise on a straight lane from looking like a curve. By default the curvature is fixed at 0
(config.max_curvature_per_m), for straight Stage 1 lanes.

detect_lanes keeps no state between policy steps. LaneDetector wraps it and remembers only the lane width measured while
both edges are seen, to place the centre line when one edge is seen, so lanes of any width work without setting their
width. The work is bounded by the batch size, a fixed number of RANSAC samples and a fixed number of solver iterations,
so it is safe to run inside the real-time policy step.
"""
from __future__ import annotations

import itertools
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
"""Most times the fit is repeated after the cones within config.outlier_distance_m of it change."""

RANSAC_SAMPLES = 200
"""Most samples of cones RANSAC draws for each shape of proposed lane (straight or curved), which bounds the
calculation time. With fewer possible samples, every one is tried; otherwise a fixed random selection
is, so a cone batch always gives the same lane."""

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
    """Cones within this distance in metres of their edge agree with a lane. RANSAC keeps the proposed lane the most
    cones agree with, and cones farther from the fitted edges are left out. Must be positive. Keep it above the
    detector's position noise but well below half the lane width."""

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

    The cones that RANSAC finds agreeing on one lane (_consensus) are fitted. Every cone is then checked against the
    fit, so a cone the first choice left out can come back, and the fit is repeated until the agreeing cones no longer
    change.

    :param rows: The cones of each colour.
    :param config: The lane detection settings.
    :return: The reference arc, each found edge's offset from it in metres (positive to the left), and the cones of
        each found edge within config.outlier_distance_m of it; or None when no edge has enough agreeing cones to give a
        direction, or the agreeing cones still change after MAX_REFITS refits.
    """
    rows = {colour: row for colour, row in rows.items() if len(row) >= config.min_cones_per_edge}
    if not any(_span(row) >= MIN_DIRECTION_SPAN_M for row in rows.values()):
        return None
    colours = list(rows)
    cones = [cone for colour in colours for cone in rows[colour]]
    xs = np.array([cone.pos.x for cone in cones])
    ys = np.array([cone.pos.y for cone in cones])
    edge_index = np.array([i for i, colour in enumerate(colours) for _ in rows[colour]])
    agree = _consensus(xs, ys, edge_index, len(colours), config)
    for _ in range(MAX_REFITS + 1):
        # An edge with too few agreeing cones is not found
        found = [i for i in range(len(colours))
                 if np.count_nonzero(agree & (edge_index == i)) >= config.min_cones_per_edge]
        agree = agree & np.isin(edge_index, found)
        kept = {colours[i]: [cone for cone, keep, edge in zip(cones, agree, edge_index) if keep and edge == i]
                for i in found}
        if not any(_span(row) >= MIN_DIRECTION_SPAN_M for row in kept.values()):
            return None
        solution = _solve(xs[agree], ys[agree], np.searchsorted(found, edge_index[agree]), len(found), config)
        offsets = np.full(len(colours), np.nan)
        offsets[found] = solution[2:]
        distances = _distances(solution[:1], solution[1:2], offsets[None, :], xs, ys, edge_index)[0]
        with np.errstate(invalid="ignore"):
            now_agree = np.abs(distances) <= config.outlier_distance_m
        if np.array_equal(now_agree, agree):
            reference = LaneArc(0.0, 0.0, float(solution[0]), float(solution[1]))
            return reference, {colours[i]: float(offsets[i]) for i in found}, kept
        agree = now_agree
    return None


def _consensus(xs: np.ndarray, ys: np.ndarray, edge_index: np.ndarray, edge_count: int,
               config: LaneDetectionConfig) -> np.ndarray:
    """Finds the cones that agree on one lane, by RANSAC.

    Small samples of cones each propose a lane passing exactly through them (_proposals): parallel lines, and also
    concentric circles when curves are allowed. Every sample has cones of every edge, so two tidy rows a doubtful width
    apart still give no lane, rather than one of them being dropped. A proposal must be a plausible lane: within the
    curvature limit, with no edge past the turning centre, and with both edges a plausible width apart. The proposal
    with the most cones within config.outlier_distance_m of their edge wins, and of those, the one they lie closest to.

    :param xs: Cone distances forwards in metres.
    :param ys: Cone distances left in metres.
    :param edge_index: Which edge each cone belongs to, from 0 to edge_count - 1.
    :param edge_count: Number of edges.
    :param config: The lane detection settings.
    :return: Whether each cone agrees with the winning proposal; all True when no proposal is plausible, so the fit
        starts from every cone.
    """
    # A fixed seed: the same cone batch always gives the same lane
    rng = np.random.default_rng(0)
    shapes = (False, True) if config.max_curvature_per_m > 0.0 else (False,)
    proposals = np.vstack([_proposals(xs, ys, edge_index, edge_count, curved, rng) for curved in shapes])
    heading, curvature, offsets = proposals[:, 0], proposals[:, 1], proposals[:, 2:]
    # Samples with all their cones on one circle around the car's origin give an infinite curvature: not plausible
    with np.errstate(invalid="ignore"):
        plausible = ((np.abs(heading) < math.pi / 2.0) & (np.abs(curvature) <= config.max_curvature_per_m)
                     & np.all(curvature[:, None] * offsets < 0.9, axis=1))
        if edge_count == 2:
            width = np.abs(offsets[:, 0] - offsets[:, 1])
            plausible &= (width >= config.min_lane_width_m) & (width <= config.max_lane_width_m)
        distances = _distances(heading, curvature, offsets, xs, ys, edge_index)
        agree = np.abs(distances) <= config.outlier_distance_m
    if not plausible.any():
        return np.ones(len(xs), dtype=bool)
    counts = np.where(plausible, np.count_nonzero(agree, axis=1), -1)
    spread = np.where(agree, distances, 0.0) ** 2
    best = np.lexsort((spread.sum(axis=1), -counts))[0]
    return agree[best]


def _proposals(xs: np.ndarray, ys: np.ndarray, edge_index: np.ndarray, edge_count: int, curved: bool,
               rng: np.random.Generator) -> np.ndarray:
    """Proposes lanes through samples of cones, each passing exactly through the cones of its sample.

    A straight proposal solves y = slope * x + intercept of each cone's edge, from two cones of one edge and one of each
    other. A curved one solves x^2 + y^2 = 2 a x + 2 b y + d of each cone's edge, concentric circles around (a, b), from
    one cone more. Both are linear, so every sample is solved at once.

    :param xs: Cone distances forwards in metres.
    :param ys: Cone distances left in metres.
    :param edge_index: Which edge each cone belongs to, from 0 to edge_count - 1.
    :param edge_count: Number of edges.
    :param curved: Whether to propose concentric circles instead of parallel lines.
    :param rng: Chooses the samples when there are more than RANSAC_SAMPLES.
    :return: One row per proposal: [heading in radians, curvature in 1/m, offset of each edge in metres].
    """
    size = edge_count + (2 if curved else 1)
    samples = _samples([np.flatnonzero(edge_index == edge) for edge in range(edge_count)], size, rng)
    sample_xs, sample_ys = xs[samples], ys[samples]
    one_hot = (edge_index[samples][..., None] == np.arange(edge_count)).astype(float)
    if curved:
        matrix = np.concatenate([2.0 * sample_xs[..., None], 2.0 * sample_ys[..., None], one_hot], axis=2)
        target = sample_xs ** 2 + sample_ys ** 2
    else:
        matrix = np.concatenate([sample_xs[..., None], one_hot], axis=2)
        target = sample_ys
    # Cones on one line, or one circle, of an edge do not fix a lane
    solvable = np.abs(np.linalg.det(matrix)) > 1e-9
    solution = np.linalg.solve(matrix[solvable], target[solvable][..., None])[..., 0]
    if curved:
        a, b = solution[:, 0], solution[:, 1]
        squared_radii = solution[:, 2:] + (a * a + b * b)[:, None]
        real = np.all(squared_radii > 0.0, axis=1)
        a, b, squared_radii = a[real], b[real], squared_radii[real]
        centre_distance = np.hypot(a, b)
        # As in _solve: the lane ahead runs at right angles to the line from the car to the centre
        turn = np.where(b >= 0.0, 1.0, -1.0)
        with np.errstate(divide="ignore"):
            curvature = turn / centre_distance
        return np.column_stack([np.arctan2(-turn * a, turn * b), curvature,
                                turn[:, None] * (centre_distance[:, None] - np.sqrt(squared_radii))])
    heading = np.arctan(solution[:, 0])
    return np.column_stack([heading, np.zeros(len(heading)), solution[:, 1:] * np.cos(heading)[:, None]])


def _samples(members: list[np.ndarray], size: int, rng: np.random.Generator) -> np.ndarray:
    """Chooses samples of size cones with at least one cone of each edge: every such sample when there are at most
    RANSAC_SAMPLES, otherwise RANSAC_SAMPLES of them at random.

    :param members: The cone indices of each edge.
    :param size: Number of cones in a sample.
    :param rng: Chooses the samples.
    :return: One row of cone indices per sample.
    """
    # How many cones each edge gives to a sample: at least one each, size in all
    splits = [split for split in itertools.product(*(range(1, len(member) + 1) for member in members))
              if sum(split) == size]
    counts = [math.prod(math.comb(len(member), count) for member, count in zip(members, split)) for split in splits]
    total = sum(counts)
    if total == 0:
        return np.empty((0, size), dtype=int)
    if total <= RANSAC_SAMPLES:
        return np.array([sum(parts, ()) for split in splits
                         for parts in itertools.product(*(itertools.combinations(member.tolist(), count)
                                                          for member, count in zip(members, split)))])
    # Too many to try them all: draw RANSAC_SAMPLES, each split as often as it has samples. A repeated sample only
    # costs time.
    chosen = rng.choice(len(splits), RANSAC_SAMPLES, p=np.array(counts) / total)
    drawn = []
    for index, split in enumerate(splits):
        draws = int(np.count_nonzero(chosen == index))
        if draws:
            # The first cones of a random order of each edge's cones, for every draw at once
            drawn.append(np.concatenate([member[np.argsort(rng.random((draws, len(member))), axis=1)[:, :count]]
                                         for member, count in zip(members, split)], axis=1))
    return np.concatenate(drawn)


def _distances(heading: np.ndarray, curvature: np.ndarray, offsets: np.ndarray, xs: np.ndarray, ys: np.ndarray,
               edge_index: np.ndarray) -> np.ndarray:
    """Signed distances of the cones from their edges, at right angles and positive to the left, for several lanes, as
    _solve measures them.

    :param heading: Each lane's direction beside the car in radians.
    :param curvature: Each lane's reference curvature in 1/m.
    :param offsets: Each lane's edge offsets in metres, one row per lane; a NaN offset gives NaN distances.
    :param xs: Cone distances forwards in metres.
    :param ys: Cone distances left in metres.
    :param edge_index: Which edge each cone belongs to.
    :return: Distances in metres, one row per lane and one column per cone.
    """
    heading, curvature = heading[:, None], curvature[:, None]
    left = -xs * np.sin(heading) + ys * np.cos(heading)
    numerator = 2.0 * left - curvature * (xs * xs + ys * ys)
    root = np.sqrt(np.maximum(1.0 - curvature * numerator, 1e-12))
    return numerator / (1.0 + root) - offsets[:, edge_index]


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
