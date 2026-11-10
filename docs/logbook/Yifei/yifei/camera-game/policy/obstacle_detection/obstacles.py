"""
obstacles.py

This file holds the types the obstacle detection module produces from the lidar.

ObstacleDetection is in the car's body frame (base_link) when the lidar scanned: +x forwards, +y left, metres and
radians. LaneObstacle is in lane coordinates instead: distance along the lane's centre line and distance left of it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from policy.input_output import Position, SensorAge
from policy.serialisation import Serialisable


@dataclass(frozen=True)
class ObstacleDetection(Serialisable):
    """The lidar obstacle observation of one policy step.

    It gives the obstacle observation of the dream-gym system-project notebook (section 8) for the real car, and
    the part of it in the car's path. The real lidar's scan plane also hits the lane's cones, so the nearest return
    overall is often a cone beside the lane; in_path_distance_m only counts the returns the car would drive into.
    """
    forward_ranges_m: list[float]
    """The notebook's obstacle_lidar_ranges_m: one range in metres per beam of a forwards fan.

    The beams are spread evenly across the configured field of view: index 0 points right, the middle index straight
    ahead and the last index left. Each value is the horizontal distance from base_link's origin to the nearest lidar
    return within half a beam spacing of the beam's direction. That is the notebook's sensor position; the real lidar
    is mounted ahead of it (0.20 m by default). A beam with no return reads the configured max_range_m, and returns
    closer than min_range_m read min_range_m, as in the notebook. So max_range_m cannot tell an object at the range
    limit from no return.
    """
    in_path_distance_m: float
    """Distance in metres along the path from the car's front to the nearest obstacle in the corridor, or math.inf
    when the corridor is clear.

    The corridor is config.corridor_half_width_m either side of the path and config.corridor_length_m long. The
    distance is that of the config.min_points-th nearest return in it, so one or two stray returns do not count.
    """
    in_path_point: Position | None
    """The lidar return at in_path_distance_m in base_link, or None when the corridor is clear."""
    in_path_point_count: int
    """Number of lidar returns in the corridor."""
    path_curvature_per_m: float
    """Curvature of the corridor's centre line in 1/m, positive turning left: the car's path if it keeps steering.

    The path starts at base_link's origin pointing forwards; 0.0 is a straight corridor ahead of the car.
    """
    sensor_age: SensorAge
    """Age of the lidar scan. The car has moved since, so the values describe that moment."""

    @property
    def path_clear(self) -> bool:
        """
        :return: True when no obstacle is in the corridor.
        """
        return math.isinf(self.in_path_distance_m)


@dataclass(frozen=True)
class LaneObstacle(Serialisable):
    """One object inside the lane ahead of the car, made of neighbouring lidar returns that are not on the lane's edges.

    Positions are in the lane coordinates of the step's LaneDetection: distance along its centre line, and distance
    left of it. The edges and the centre line are concentric arcs, so the edges are half the lane width either side of
    the centre line all along the lane, and an object's place across the lane does not depend on the curve.
    """
    distance_m: float
    """Distance in metres along the lane from beside the car's front to the object's nearest return."""
    far_distance_m: float
    """Distance in metres along the lane from beside the car's front to the object's farthest return.

    The lidar only sees the faces turned towards it, so this is a lower bound on how far the object reaches.
    """
    right_m: float
    """Position across the lane of the object's rightmost return, in metres left of the centre line (negative when it
    is right of the centre line)."""
    left_m: float
    """Position across the lane of the object's leftmost return, in metres left of the centre line."""
    point_count: int
    """Number of lidar returns in the object."""
