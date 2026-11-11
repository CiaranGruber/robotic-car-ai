"""
lane_path.py

This file holds the type LaneAvoidance plans: the line across the lane the car follows past the obstacles inside it.

Positions are in the lane coordinates (a Frenet frame) of the step's LaneDetection: distance s along its centre line
from beside the car's origin (base_link), and offset d left of the centre line, in metres. The line is a chain of
quintic polynomials d(s). Where two meet they share their offset, slope and curvature, so the car's heading and
steering change smoothly along the line.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

QUINTIC_PEAK_CURVATURE = 10.0 / math.sqrt(3.0)
"""A quintic that moves the line by d metres over L metres, parallel to the lane at both ends, is at most
QUINTIC_PEAK_CURVATURE * d / L^2 curved (in 1/m)."""
RANGE_STEP_M = 0.02
"""Spacing in metres of the points LanePath.offset_range checks. The line moves less than 0.1 mm between them."""


@dataclass(frozen=True)
class LineState:
    """The line at one distance along the lane."""
    offset_m: float
    """Offset in metres left of the lane centre."""
    slope: float = 0.0
    """Change of the offset per metre along the lane: the tangent of the line's heading relative to the lane, positive
    turning left."""
    curvature_per_m: float = 0.0
    """Change of the slope per metre along the lane in 1/m: about the curvature the line adds to the lane's own."""

    @property
    def parallel(self) -> bool:
        """
        :return: Whether the line here runs along the lane without turning across it.
        """
        return self.slope == 0.0 and self.curvature_per_m == 0.0


@dataclass(frozen=True)
class PathSegment:
    """A part of the line: a quintic polynomial in the distance t = s - start_m,
    offset = c0 + c1 t + c2 t^2 + c3 t^3 + c4 t^4 + c5 t^5.

    A hold keeps one offset (only c0); a move joins two LineStates, matching the offset, slope and curvature at both
    ends.
    """
    start_m: float
    """Distance in metres along the lane where the segment starts."""
    end_m: float
    """Distance in metres along the lane where the segment ends; above start_m."""
    coefficients: tuple[float, ...]
    """The polynomial's coefficients c0 to c5, in metres per metre^i."""

    @classmethod
    def hold(cls, start_m: float, end_m: float, offset_m: float) -> PathSegment:
        """
        :return: The segment that keeps offset_m from start_m to end_m.
        """
        return cls(start_m, end_m, (offset_m, 0.0, 0.0, 0.0, 0.0, 0.0))

    @classmethod
    def move(cls, start_m: float, end_m: float, start: LineState, end: LineState) -> PathSegment:
        """The only quintic that leaves start at start_m and arrives at end at end_m.

        :return: The segment.
        """
        length = end_m - start_m
        c0, c1, c2 = start.offset_m, start.slope, start.curvature_per_m / 2.0
        # What is left to reach at the end after the first three terms
        offset = end.offset_m - (c0 + c1 * length + c2 * length ** 2)
        slope = end.slope - (c1 + 2.0 * c2 * length)
        curvature = end.curvature_per_m - 2.0 * c2
        c3 = (10.0 * offset - 4.0 * slope * length + curvature * length ** 2 / 2.0) / length ** 3
        c4 = (-15.0 * offset + 7.0 * slope * length - curvature * length ** 2) / length ** 4
        c5 = (6.0 * offset - 3.0 * slope * length + curvature * length ** 2 / 2.0) / length ** 5
        return cls(start_m, end_m, (c0, c1, c2, c3, c4, c5))

    def sample(self, s_m: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        :param s_m: Distances in metres along the lane, within the segment.
        :return: The offsets, slopes and curvatures there (see LineState).
        """
        t = np.asarray(s_m, dtype=float) - self.start_m
        c0, c1, c2, c3, c4, c5 = self.coefficients
        # Horner's rule for the polynomial and its first two derivatives
        return (c0 + t * (c1 + t * (c2 + t * (c3 + t * (c4 + t * c5)))),
                c1 + t * (2.0 * c2 + t * (3.0 * c3 + t * (4.0 * c4 + t * 5.0 * c5))),
                2.0 * c2 + t * (6.0 * c3 + t * (12.0 * c4 + t * 20.0 * c5)))

    def state_at(self, s_m: float) -> LineState:
        """
        :param s_m: Distance in metres along the lane, within the segment.
        :return: The line there.
        """
        offset, slope, curvature = self.sample(np.array([s_m]))
        return LineState(float(offset[0]), float(slope[0]), float(curvature[0]))

    def peak_curvature_per_m(self) -> float:
        """
        :return: The largest curvature along the segment in 1/m, positive.
        """
        return float(np.max(np.abs(self.sample(np.linspace(self.start_m, self.end_m, 41))[2])))


@dataclass(frozen=True)
class LanePath:
    """The planned line across the lane for one policy step."""
    usual_offset_m: float
    """The line followed away from obstacles, in metres left of the lane centre (MpcPolicy.usual_offset_m)."""
    segments: tuple[PathSegment, ...]
    """The parts of the line that leave the usual line, in order along the lane and not overlapping; the line is on
    the usual line between them. Empty when no obstacle in the lane changes the line."""

    def sample(self, s_m: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        :param s_m: Distances in metres along the lane from beside the car's origin.
        :return: The line's offsets, slopes and curvatures there (see LineState).
        """
        s_m = np.asarray(s_m, dtype=float)
        offsets = np.full(s_m.shape, float(self.usual_offset_m))
        slopes, curvatures = np.zeros(s_m.shape), np.zeros(s_m.shape)
        for segment in self.segments:
            inside = (s_m >= segment.start_m) & (s_m <= segment.end_m)
            if np.any(inside):
                offsets[inside], slopes[inside], curvatures[inside] = segment.sample(s_m[inside])
        return offsets, slopes, curvatures

    def state_at(self, s_m: float) -> LineState:
        """
        :param s_m: Distance in metres along the lane from beside the car's origin.
        :return: The line there.
        """
        for segment in self.segments:
            if segment.start_m <= s_m <= segment.end_m:
                return segment.state_at(s_m)
        return LineState(self.usual_offset_m)

    def offset_at(self, s_m: float) -> float:
        """
        :param s_m: Distance in metres along the lane from beside the car's origin.
        :return: The line's offset in metres left of the lane centre there.
        """
        return self.state_at(s_m).offset_m

    def offset_range(self, start_m: float, end_m: float) -> tuple[float, float]:
        """
        :param start_m: Distance in metres along the lane where the stretch starts.
        :param end_m: Distance in metres along the lane where the stretch ends; at least start_m.
        :return: The smallest and largest offset of the line along the stretch, in metres left of the lane centre.
        """
        s_m = np.append(np.arange(start_m, end_m, RANGE_STEP_M), end_m)
        offsets = self.sample(s_m)[0]
        return float(np.min(offsets)), float(np.max(offsets))

    def moved_back(self, driven_m: float, usual_offset_m: float) -> LanePath:
        """
        :param driven_m: Distance in metres the car has driven along the lane since this line was planned.
        :param usual_offset_m: The usual line now, in metres left of the lane centre.
        :return: The same line, from beside where the car's origin is now, leaving out the part behind it.
        """
        return LanePath(usual_offset_m, tuple(
            PathSegment(segment.start_m - driven_m, segment.end_m - driven_m, segment.coefficients)
            for segment in self.segments if segment.end_m - driven_m > 0.0))


def move_length_m(change_m: float, curvature_per_m: float) -> float:
    """
    :param change_m: How far a move takes the line across the lane, in metres, from and to a line parallel to it.
    :param curvature_per_m: Sharpest curvature the move may have, in 1/m.
    :return: The shortest such move's length in metres, math.inf when no curvature is left for it.
    """
    if change_m == 0.0:
        return 0.0
    if curvature_per_m <= 0.0:
        return math.inf
    return math.sqrt(QUINTIC_PEAK_CURVATURE * abs(change_m) / curvature_per_m)
