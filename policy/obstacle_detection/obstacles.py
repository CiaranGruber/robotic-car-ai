"""
obstacles.py

This file holds the type the obstacle detection module produces from the lidar.

Everything is in the car's body frame (base_link) when the lidar scanned: +x forwards, +y left, metres and radians.
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
