"""
obstacle_detection.py

This file turns the lidar into the obstacle observation: what the lidar sees ahead, and what is in the car's path.

It follows the dream-gym system-project notebook, which adds obstacle_lidar_ranges_m to the cone observations (section
8), and its hint for the stopping exercise (section 11): the nearest return overall also responds to objects beside the
road, so a return's direction and the car's width decide whether it is in the way. On the real car this matters more:
the lidar's scan plane is low enough to hit the lane's cones, which the notebook's obstacle lidar does not see.

It uses the body-frame Cartesian points (lidar_cartesian), not the raw scan. The raw scan is in the lidar's own frame,
which is mounted turned 180 degrees by default, so its ray order does not match the notebook's beams.

Each scan gives:
1. A forwards fan of ranges laid out like the notebook's beams, so policies written in the notebook can be ported.
2. The returns in a corridor along the car's path: an arc of the curvature the car is steering along, as wide as the
   car plus a margin. The distance along the path to the nearest of them is the obstacle in the way. A fixed
   forwards corridor would take the cones ahead for an obstacle whenever the car turns.

It keeps no state between policy steps, and its loop is bounded by the number of lidar points, so it is safe to run
inside the real-time policy step.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from policy.input_output import CarObservations
from policy.obstacle_detection.obstacles import ObstacleDetection

STRAIGHT_CURVATURE_PER_M = 1e-3
"""Path curvatures smaller than this in 1/m give a straight corridor. Over 2 m, that moves the path by 2 mm."""


@dataclass(frozen=True)
class ObstacleDetectionConfig:
    """Settings for obstacle detection, loaded from the obstacle_detection parameters in config/ai4r_policy.yaml.

    These defaults are also the parameters' defaults in policy_node. The fan settings follow the notebook's lidar.
    Invalid settings raise a ValueError, which stops policy_node from starting.
    """
    beam_count: int = 61
    """Number of beams in the forwards fan. Must be an integer of at least 2. The notebook's 61 are 3 degrees apart."""
    field_of_view_deg: float = 180.0
    """Angle in degrees from the fan's first beam (right) to its last (left), centred straight ahead. Must be within
    (0, 360]."""
    min_range_m: float = 0.05
    """Fan ranges closer than this in metres read this value. Must be at least 0 and below max_range_m."""
    max_range_m: float = 4.0
    """Fan range in metres of a beam with no return. Farther returns count as no return."""
    car_front_m: float = 0.30
    """Distance in metres from base_link's origin forwards to the car's front. Must be at least 0.

    Returns beside or behind the front are not ahead of the car. The default is the notebook car's front (0.297 m);
    measure the real car.
    """
    corridor_half_width_m: float = 0.20
    """Half the corridor's width in metres: half the car's width plus a margin. Must be positive.

    The default suits the notebook car's 0.25 m width. Keep it well below half the lane width (0.5 m), or the cones
    beside the lane count as obstacles.
    """
    corridor_length_m: float = 1.5
    """Corridor length in metres along the path ahead of the car's front. Must be positive.

    Longer sees obstacles sooner, but the path is less certain farther ahead, and a heading error brings the cones
    into a long corridor.
    """
    min_points: int = 3
    """Fewest returns in the corridor that make an obstacle. Must be an integer of at least 1.

    A scan of 720 rays spaces them 0.5 degrees apart, so an object 0.1 m wide and 1.5 m away gives about 7 returns.
    """

    def __post_init__(self):
        for name, lowest in (("beam_count", 2), ("min_points", 1)):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < lowest:
                raise ValueError(f"obstacle_detection.{name} must be an integer of at least {lowest}, not {value}")
        if not (math.isfinite(self.field_of_view_deg) and 0.0 < self.field_of_view_deg <= 360.0):
            raise ValueError(f"obstacle_detection.field_of_view_deg must be within (0, 360], "
                             f"not {self.field_of_view_deg}")
        for name in ("min_range_m", "car_front_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f"obstacle_detection.{name} must be finite and at least 0, not {value}")
        for name in ("max_range_m", "corridor_half_width_m", "corridor_length_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"obstacle_detection.{name} must be positive and finite, not {value}")
        if self.min_range_m >= self.max_range_m:
            raise ValueError("obstacle_detection.min_range_m must be below max_range_m")


def detect_obstacles(observations: CarObservations, config: ObstacleDetectionConfig,
                     path_curvature_per_m: float = 0.0) -> ObstacleDetection | None:
    """Finds what the lidar sees ahead of the car, and the nearest obstacle in its path.

    :param observations: The observations taken by the car at the current policy step.
    :param config: The obstacle detection settings.
    :param path_curvature_per_m: Curvature in 1/m of the path the car will drive, positive turning left, normally the
        curvature it is steering along. 0.0 gives a straight corridor ahead.
    :return: The obstacle observation, or None when there are no fresh Cartesian lidar points
        (observations.lidar_cartesian is None). A fresh scan with no returns gives a clear observation.
    """
    lidar = observations.lidar_cartesian
    if lidar is None:
        return None
    half_view_rad = math.radians(config.field_of_view_deg) / 2.0
    beam_spacing_rad = 2.0 * half_view_rad / (config.beam_count - 1)
    ranges = [config.max_range_m] * config.beam_count
    in_path = []
    for point in lidar.points:
        x, y = point.pos.x, point.pos.y
        # Forwards fan: the beam nearest the return's direction from base_link's origin
        beam = round((math.atan2(y, x) + half_view_rad) / beam_spacing_rad)
        distance = math.hypot(x, y)
        if 0 <= beam < config.beam_count and distance < ranges[beam]:
            ranges[beam] = max(config.min_range_m, distance)
        # Corridor: the return's position along the path and to its side
        along, left = _path_coordinates(x, y, path_curvature_per_m)
        if (config.car_front_m < along <= config.car_front_m + config.corridor_length_m
                and abs(left) <= config.corridor_half_width_m):
            in_path.append((along, point.pos))
    in_path.sort(key=lambda item: item[0])
    if len(in_path) >= config.min_points:
        along, nearest = in_path[config.min_points - 1]
        distance_m = along - config.car_front_m
    else:
        distance_m, nearest = math.inf, None
    return ObstacleDetection(ranges, distance_m, nearest, len(in_path), path_curvature_per_m, lidar.sensor_age)


def _path_coordinates(x: float, y: float, curvature_per_m: float) -> tuple[float, float]:
    """
    :param x: The point's distance forwards of base_link's origin in metres.
    :param y: The point's distance left of base_link's origin in metres.
    :param curvature_per_m: Curvature of the path in 1/m, positive turning left. The path starts at base_link's
        origin, pointing forwards.
    :return: The distance in metres along the path to beside the point, and the point's distance in metres left of
        the path. The distance along is 0 or less behind the start, and past half a turn it comes back negative.
    """
    if abs(curvature_per_m) < STRAIGHT_CURVATURE_PER_M:
        return x, y
    # The path is a circle around (0, radius); the radius is negative when the path turns right
    radius = 1.0 / curvature_per_m
    side = math.copysign(1.0, curvature_per_m)
    turned_rad = math.atan2(x, side * (radius - y))
    return abs(radius) * turned_rad, side * (abs(radius) - math.hypot(x, y - radius))
