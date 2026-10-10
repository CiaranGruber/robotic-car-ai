"""
cone_filter.py

This file removes the cone detections that cannot belong to the lane, before the later stages use the cones.

The cone detector already detects the cones: oakd_cone_detector on the car, or the simulated detector in dream-gym.
It only drops cones below its confidence threshold or outside its depth range, so duplicate detections, cones of
other lanes, misclassified colours and other objects still reach the policy. This filter removes, in order:
1. Cones outside the space the lane can occupy, or below the confidence floor.
2. Duplicate detections of one cone: same-colour detections closer than merge_distance_m. The most confident is kept.
3. Cones off their colour's row. Stage 1 is a straight lane, so each colour is one straight row and both rows are
   parallel. The rows are fitted together, so a long row can outvote a wrong cone in a short one.

remove_implausible_cones applies only steps 1 and 2, for policies that treat the cones as obstacles rather than lane
rows.

It keeps no state between policy steps, and its loops are bounded by the batch size and max_fit_cones_per_colour, so
it is safe to run inside the real-time policy step.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from policy.input_output import ConeBatch, ConeColour, ConeDetection

MIN_PAIR_SPACING_M = 0.05
"""Smallest forwards distance in metres between two same-colour cones for them to propose a row direction."""


@dataclass(frozen=True)
class ConeFilterConfig:
    """Settings for the cone filter, loaded from the cone_filter parameters in config/ai4r_policy.yaml.

    These defaults are also the parameters' defaults in policy_node. They follow the shared defaults in
    docs/movement-policy-tasks.md where one exists; replace them when Task A1 measures the camera. Invalid settings
    raise a ValueError, which stops policy_node from starting.
    """
    max_forward_m: float = 4.0
    """Largest kept forwards distance (x) in metres. Cones at or behind x = 0 are always removed. Must be positive.

    The default is the shared camera range.
    """
    max_lateral_m: float = 2.0
    """Largest kept sideways distance (|y|) in metres. Must be positive.

    The default covers half the 1.0 m lane, a 0.2 m start offset and a 15 degree heading error seen 4 m ahead.
    """
    min_height_m: float = -0.15
    """Lowest kept detected-point height (z) in metres above the nominal ground. Must be below max_height_m.

    The fixed camera pose ignores vehicle pitch, so the height band is kept loose.
    """
    max_height_m: float = 0.45
    """Highest kept detected-point height (z) in metres above the nominal ground."""
    min_confidence: float = 0.5
    """Lowest kept classification confidence, from 0.0 to 1.0.

    The detector already applies its own threshold (default 0.5), so the default removes nothing more.
    """
    merge_distance_m: float = 0.1
    """Same-colour detections closer than this in metres count as one cone. Must be positive.

    Keep it well below the 0.5 m cone spacing.
    """
    row_residual_m: float = 0.15
    """Largest kept distance in metres from a cone to its fitted colour row. Must be positive.

    Keep it above the detector's position noise but well below half the 1.0 m lane width.
    """
    max_fit_cones_per_colour: int = 8
    """Number of nearest cones of each colour used to fit the rows. Must be an integer from 2 to 12.

    The fit's calculation time grows with the fourth power of this, so it is bounded. Farther cones are still checked
    against the fitted rows.
    """

    def __post_init__(self):
        for name in ("max_forward_m", "max_lateral_m", "merge_distance_m", "row_residual_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"cone_filter.{name} must be positive and finite, not {value}")
        if not (math.isfinite(self.min_height_m) and math.isfinite(self.max_height_m)
                and self.min_height_m < self.max_height_m):
            raise ValueError("cone_filter.min_height_m and max_height_m must be finite, with min_height_m below "
                             "max_height_m")
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError(f"cone_filter.min_confidence must be within [0, 1], not {self.min_confidence}")
        count = self.max_fit_cones_per_colour
        if isinstance(count, bool) or not isinstance(count, int) or not 2 <= count <= 12:
            raise ValueError(f"cone_filter.max_fit_cones_per_colour must be an integer from 2 to 12, not {count}")


def filter_cones(cones: ConeBatch | None, config: ConeFilterConfig) -> ConeBatch | None:
    """Removes the cone detections that cannot belong to the lane.

    When no colour has two cones far enough apart, there is no row to check against, so every cone that passes the
    first two steps is kept. A colour with only two cones cannot tell which of them is wrong unless the other colour's
    row decides the direction; more cones make the fit stronger.

    :param cones: The cone detections for this policy step, or None when none are available.
    :param config: The cone filter settings.
    :return: The kept cones in their received order, with the batch's sensor age and latency, or None when cones is
        None. An empty batch means no cone was plausible; it is still fresh data.
    """
    cones = remove_implausible_cones(cones, config)
    if cones is None:
        return None
    kept = list(cones)
    fit = _fit_parallel_rows(kept, config)
    if fit is not None:
        slope, offsets = fit
        kept = [cone for cone in kept
                if _distance_from_row(cone, slope, offsets[cone.colour]) <= config.row_residual_m]
    return ConeBatch(kept, cones.sensor_age, cones.acquisition_to_publish_latency_s)


def remove_implausible_cones(cones: ConeBatch | None, config: ConeFilterConfig) -> ConeBatch | None:
    """Removes only the first two kinds of outlier: cones outside the kept space or below the confidence floor,
    and duplicate detections. Use it when the cones are obstacles rather than lane rows.

    :param cones: The cone detections for this policy step, or None when none are available.
    :param config: The cone filter settings. The row settings are not used.
    :return: The kept cones in their received order, with the batch's sensor age and latency, or None when cones is
        None.
    """
    if cones is None:
        return None
    kept = _merge_duplicates([cone for cone in cones if _is_plausible(cone, config)], config.merge_distance_m)
    return ConeBatch(kept, cones.sensor_age, cones.acquisition_to_publish_latency_s)


def _is_plausible(cone: ConeDetection, config: ConeFilterConfig) -> bool:
    """
    :return: True when the cone is within the space the lane can occupy and is confident enough.
    """
    return (0.0 < cone.pos.x <= config.max_forward_m
            and abs(cone.pos.y) <= config.max_lateral_m
            and config.min_height_m <= cone.pos.z <= config.max_height_m
            and cone.confidence >= config.min_confidence)


def _merge_duplicates(cones: list[ConeDetection], merge_distance_m: float) -> list[ConeDetection]:
    """Keeps only the most confident of same-colour detections closer than merge_distance_m.

    :return: The kept cones, in their received order.
    """
    kept = []
    # Most confident first; sorted is stable, so equally confident cones keep their received order
    for i in sorted(range(len(cones)), key=lambda i: cones[i].confidence, reverse=True):
        if all(cones[i].colour != cones[j].colour
               or math.hypot(cones[i].pos.x - cones[j].pos.x, cones[i].pos.y - cones[j].pos.y) >= merge_distance_m
               for j in kept):
            kept.append(i)
    return [cones[i] for i in sorted(kept)]


def _fit_parallel_rows(cones: list[ConeDetection],
                       config: ConeFilterConfig) -> tuple[float, dict[ConeColour, float]] | None:
    """Fits each colour's row as y = offset + slope * x in base_link, with one slope shared by every row.

    This is an exhaustive, so deterministic, RANSAC over the nearest cones of each colour: every pair of same-colour
    cones proposes a slope, each row takes the offset that most of its cones agree with at that slope, and the slope
    that most cones agree with (then with the smallest total distance) wins.

    :param cones: The cones to fit.
    :param config: The cone filter settings.
    :return: The shared slope and each colour's offset in metres, or None when no colour has two cones far enough
        apart.
    """
    rows: dict[ConeColour, list[ConeDetection]] = {}
    for cone in sorted(cones, key=lambda cone: cone.pos.x):
        row = rows.setdefault(cone.colour, [])
        if len(row) < config.max_fit_cones_per_colour:
            row.append(cone)
    slopes = [(far.pos.y - near.pos.y) / (far.pos.x - near.pos.x)
              for row in rows.values() for i, near in enumerate(row) for far in row[i + 1:]
              if far.pos.x - near.pos.x >= MIN_PAIR_SPACING_M]
    best = None
    for slope in slopes:
        count, distance, offsets = 0, 0.0, {}
        for colour, row in rows.items():
            row_count, row_distance, offsets[colour] = _best_row_offset(row, slope, config.row_residual_m)
            count += row_count
            distance += row_distance
        if best is None or (count, -distance) > best[0]:
            best = ((count, -distance), slope, offsets)
    return None if best is None else (best[1], best[2])


def _best_row_offset(row: list[ConeDetection], slope: float, row_residual_m: float) -> tuple[int, float, float]:
    """Finds the offset of one colour's row, at the given slope, that most of its cones agree with.

    :return: The number of cones within row_residual_m of the row, their total distance from it in metres, and the
        row's offset in metres, which is the mean of those cones' offsets.
    """
    # Dividing a difference in offset by this gives the distance between the parallel lines
    scale = math.hypot(1.0, slope)
    offsets = [cone.pos.y - slope * cone.pos.x for cone in row]
    best = None
    for candidate in offsets:
        agreeing = [offset for offset in offsets if abs(offset - candidate) / scale <= row_residual_m]
        distance = sum(abs(offset - candidate) for offset in agreeing) / scale
        if best is None or (len(agreeing), -distance) > best[0]:
            best = ((len(agreeing), -distance), sum(agreeing) / len(agreeing))
    (count, negative_distance), offset = best
    return count, -negative_distance, offset


def _distance_from_row(cone: ConeDetection, slope: float, offset: float) -> float:
    """
    :return: The distance in metres from the cone to the row y = offset + slope * x.
    """
    return abs(cone.pos.y - offset - slope * cone.pos.x) / math.hypot(1.0, slope)
