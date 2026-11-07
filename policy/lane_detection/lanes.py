"""
lanes.py

This file holds the types the lane detection module produces for the action policy.

Everything is in the car's body frame (base_link) when the cones were detected: +x forwards, +y left, metres and
radians. Signs follow PathTrackingState in policy/action_policy/mpc.py, so the policy can use the values directly:
left of the lane centre, pointing left and turning left are positive.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Self

from policy.input_output import ConeColour, SensorAge
from policy.serialisation import Serialisable


class LaneStatus(Serialisable, Enum):
    """Which lane edges were found at one policy step."""
    BOTH_EDGES = 0
    """Both edges were found, so the centre line and width are measured."""
    LEFT_EDGE_ONLY = 1
    """Only the left edge was found. The centre line is half the configured lane width to its right."""
    RIGHT_EDGE_ONLY = 2
    """Only the right edge was found. The centre line is half the configured lane width to its left."""
    NO_LANE = 3
    """No edge was found, or the edges found are not a plausible lane. There is no centre line."""

    def serialise(self) -> str:
        """
        :return: The enum member name.
        """
        return self.name

    @classmethod
    def deserialise(cls, data: Any) -> Self:
        """
        :param data: Enum member name, or the underlying integer value.
        :return: The matching lane status.
        """
        if isinstance(data, str):
            return cls[data]
        return cls(data)


@dataclass(frozen=True)
class LaneArc(Serialisable):
    """A circular arc on the ground in base_link, or a straight line when its curvature is 0.

    Positions along it are measured by s, the distance along the arc from its start point in metres, positive in its
    heading direction. A lane's edges and centre line are arcs around one centre point, so they stay the same distance
    apart however sharply the lane turns, and all three start on the line through the car's origin at right angles to
    the lane.
    """
    x: float
    """Start point's distance forwards in metres."""
    y: float
    """Start point's distance left in metres."""
    heading_rad: float
    """Direction at the start point relative to the car's +x axis in radians; positive to the left."""
    curvature_per_m: float
    """1 / turning radius in 1/m, positive turning left. It is the same all along the arc."""

    def point_at(self, s: float) -> tuple[float, float]:
        """
        :param s: Distance along the arc from its start point in metres.
        :return: The (x, y) position in metres at that distance.
        """
        turn = self.curvature_per_m * s
        # The chord to that point, which works for any curvature including 0
        chord = s * _sinc(turn / 2.0)
        direction = self.heading_rad + turn / 2.0
        return self.x + chord * math.cos(direction), self.y + chord * math.sin(direction)

    def heading_at(self, s: float) -> float:
        """
        :param s: Distance along the arc from its start point in metres.
        :return: The arc's direction in radians relative to the car's +x axis at that distance; positive to the left.
        """
        return self.heading_rad + self.curvature_per_m * s

    def signed_distance(self, x: float, y: float) -> float:
        """
        :param x: Point's distance forwards in metres.
        :param y: Point's distance left in metres.
        :return: Shortest distance in metres from the point to the arc's full circle (or line); positive when the
            point is on the arc's left.
        """
        along, left = self._local(x, y)
        k = self.curvature_per_m
        # Rearranged so it stays accurate as the curvature approaches 0
        numerator = 2.0 * left - k * (along * along + left * left)
        return numerator / (1.0 + math.sqrt(max(0.0, 1.0 - k * numerator)))

    def project(self, x: float, y: float) -> float:
        """
        :param x: Point's distance forwards in metres.
        :param y: Point's distance left in metres.
        :return: Distance s in metres along the arc to the point on it nearest the given point.
        """
        along, left = self._local(x, y)
        k = self.curvature_per_m
        if abs(k) < 1e-9:
            return along
        return math.atan2(k * along, 1.0 - k * left) / k

    def offset(self, distance_m: float) -> LaneArc:
        """Moves the arc sideways, keeping its centre point, so the result stays the same distance from it everywhere.

        :param distance_m: Distance to move in metres; positive moves it to the left. It must not reach the centre
            point, so curvature_per_m * distance_m must be below 1.
        :return: The moved arc, starting beside this arc's start point.
        """
        normal = self.heading_rad + math.pi / 2.0
        return LaneArc(self.x + distance_m * math.cos(normal), self.y + distance_m * math.sin(normal),
                       self.heading_rad, self.curvature_per_m / (1.0 - self.curvature_per_m * distance_m))

    def _local(self, x: float, y: float) -> tuple[float, float]:
        """
        :return: The point's distance in metres along the start direction and to its left, from the start point.
        """
        dx, dy = x - self.x, y - self.y
        cos, sin = math.cos(self.heading_rad), math.sin(self.heading_rad)
        return dx * cos + dy * sin, -dx * sin + dy * cos


def _sinc(angle: float) -> float:
    """
    :return: sin(angle) / angle, which is 1 at angle 0.
    """
    return 1.0 - angle * angle / 6.0 if abs(angle) < 1e-4 else math.sin(angle) / angle


@dataclass(frozen=True)
class LaneEdge(Serialisable):
    """One lane boundary, fitted to one colour's cones."""
    arc: LaneArc
    """The fitted edge, starting beside the car."""
    colour: ConeColour
    """Colour of the cones this edge was fitted to."""
    cone_count: int
    """Number of cones the edge was fitted to, at least config.min_cones_per_edge. Cones farther than
    config.outlier_distance_m from the fitted edge are left out."""
    nearest_m: float
    """Distance along the edge in metres to its nearest cone used."""
    farthest_m: float
    """Distance along the edge in metres to its farthest cone used. The edge is unknown beyond this."""
    rms_residual_m: float
    """Root mean square distance in metres from the cones used to the fitted edge, at right angles to it.

    It grows with detector noise, misplaced cones, and a lane whose curvature changes within view.
    """


@dataclass(frozen=True)
class LaneDetection(Serialisable):
    """The lane detected from the cones of one policy step.

    Check status first: the centre line and the tracking values are None when status is NO_LANE.
    """
    status: LaneStatus
    """Which edges were found."""
    left_edge: LaneEdge | None
    """The left boundary, or None when it was not found."""
    right_edge: LaneEdge | None
    """The right boundary, or None when it was not found."""
    centre_line: LaneArc | None
    """Middle of the lane, starting at its point nearest the car's origin, or None when status is NO_LANE.

    With one edge, it is half of lane_width_m from that edge, so it is only as good as the configured width.
    """
    lane_width_m: float | None
    """Distance between the edges in metres, the same all along the lane, or None when status is NO_LANE.

    Measured with both edges; otherwise the configured lane width.
    """
    visible_until_m: float | None
    """Distance along the centre line in metres to beside the farthest cone used, or None when status is NO_LANE.

    The lane is unknown beyond this, so a planner should not predict farther. It falls as the car nears the lane end.
    """
    lateral_error_m: float | None
    """Signed distance in metres from the centre line to the car's origin; positive when the car is left of centre.

    None when status is NO_LANE. The car is outside the lane when its magnitude is above half the lane width.
    """
    heading_error_rad: float | None
    """Car heading minus centre line heading beside the car in radians; positive when the car points left of the
    lane. None when status is NO_LANE."""
    path_curvature_per_m: float | None
    """Curvature of the centre line in 1/m, positive turning left, or None when status is NO_LANE.

    The lane is fitted with constant curvature over the visible cones, so this is their average curvature.
    """
    sensor_age: SensorAge
    """Age of the cone batch the lane was detected from. The car has moved since, so values describe that moment."""

    @property
    def has_lane(self) -> bool:
        """
        :return: True when a centre line and the tracking values are available.
        """
        return self.status != LaneStatus.NO_LANE
