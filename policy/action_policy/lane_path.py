"""
lane_path.py

This file holds the type LaneAvoidance plans: the line across the lane the car follows past the obstacles inside it.

Positions are in the lane coordinates of the step's LaneDetection: distance along its centre line from beside the car's
origin (base_link), and distance left of the centre line, in metres.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PathSegment:
    """A part of the line where it moves smoothly from one offset to another, or holds one offset.

    The offset follows half a cosine wave from start_offset_m to end_offset_m, so the line starts and ends parallel to
    the lane, and its sharpest curvature is pi^2 / 2 * change / length^2.
    """
    start_m: float
    """Distance in metres along the lane where the segment starts."""
    end_m: float
    """Distance in metres along the lane where the segment ends; above start_m."""
    start_offset_m: float
    """Offset in metres left of the lane centre at start_m."""
    end_offset_m: float
    """Offset in metres left of the lane centre at end_m."""

    def offset_at(self, s_m: float) -> float:
        """
        :param s_m: Distance in metres along the lane, within the segment.
        :return: The offset in metres left of the lane centre there.
        """
        progress = (s_m - self.start_m) / (self.end_m - self.start_m)
        change_m = self.end_offset_m - self.start_offset_m
        return self.start_offset_m + change_m * (1.0 - math.cos(math.pi * progress)) / 2.0


@dataclass(frozen=True)
class LanePath:
    """The planned line across the lane for one policy step."""
    usual_offset_m: float
    """The line followed away from obstacles, in metres left of the lane centre (MpcPolicy.usual_offset_m)."""
    segments: tuple[PathSegment, ...]
    """The parts of the line that leave the usual line, in order along the lane and not overlapping. Empty when no
    obstacle in the lane changes the line."""

    def offset_at(self, s_m: float) -> float:
        """
        :param s_m: Distance in metres along the lane from beside the car's origin.
        :return: The line's offset in metres left of the lane centre there.
        """
        for segment in self.segments:
            if segment.start_m <= s_m <= segment.end_m:
                return segment.offset_at(s_m)
        return self.usual_offset_m

    def offset_range(self, start_m: float, end_m: float) -> tuple[float, float]:
        """
        :param start_m: Distance in metres along the lane where the stretch starts.
        :param end_m: Distance in metres along the lane where the stretch ends; at least start_m.
        :return: The smallest and largest offset of the line along the stretch, in metres left of the lane centre.
        """
        # Each segment's offset changes in one direction only, so the extremes are at the stretch's ends or at the
        # segments' ends within it
        offsets = [self.offset_at(start_m), self.offset_at(end_m)]
        for segment in self.segments:
            offsets += [offset for s_m, offset in ((segment.start_m, segment.start_offset_m),
                                                    (segment.end_m, segment.end_offset_m)) if start_m < s_m < end_m]
        return min(offsets), max(offsets)
