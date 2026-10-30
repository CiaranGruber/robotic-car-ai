"""
obstacle_detection.py

This file turns the lidar into the obstacle observation: what the lidar sees ahead, and what is in the car's path.

It follows the dream-gym system-project notebook, which adds obstacle_lidar_ranges_m to the cone observations (section
8), and its hint for the stopping exercise (section 11): the nearest return overall also responds to objects beside the
road, so a return's direction and the car's width decide whether it is in the way. On the real car this matters more:
the lidar's scan plane is low enough to hit the lane's cones, which the notebook's obstacle lidar does not see.

It uses the body-frame Cartesian points (lidar_cartesian), not the raw scan. The raw scan is in the lidar's own frame,
which is mounted turned 180 degrees by default, so its ray order does not match the notebook's beams.

Each scan gives, from detect_obstacles:
1. A forwards fan of ranges laid out like the notebook's beams, so policies written in the notebook can be ported.
2. The returns in a corridor along a path from the car: an arc of a given curvature, as wide as the car plus a margin.
   The distance along the path to the nearest of them is the obstacle in the way.

When a lane is detected, detect_lane_obstacles gives the objects inside it instead, in lane coordinates. Returns on or
outside the lane's edges are the cones or the lane's surroundings, so they are left out, and neighbouring returns are
grouped into objects with their extent across the lane. A policy can then check the part of the lane the car will
drive through (nearest_in_band), and find room beside an object. A corridor in the car's own frame would take the
cones ahead for an obstacle whenever the car points away from the lane or the lane turns.

It keeps no state between policy steps, and its loops are bounded by the number of lidar points, so it is safe to run
inside the real-time policy step.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from policy.input_output import CarObservations
from policy.lane_detection.lanes import LaneDetection
from policy.obstacle_detection.obstacles import LaneObstacle, ObstacleDetection

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
    """Half the width in metres of the way the car drives through: half the car's width plus a margin. Must be
    positive. Used for the corridor and for the band across the lane a policy checks with nearest_in_band.

    The default suits the notebook car's 0.25 m width. In the corridor, keep it well below half the lane width (0.4 m
    for the lab's 0.8 m lanes), or the cones beside the lane count as obstacles.
    """
    corridor_length_m: float = 1.5
    """Length in metres of the straight corridor ahead of the car's front. Must be positive.

    Longer sees obstacles sooner, but the path is less certain farther ahead, and a heading error brings the cones
    into a long corridor. A policy that stops for obstacles needs its stopping distance to fit within it.
    """
    lane_length_m: float = 2.5
    """Length in metres along the lane, ahead of the car's front, searched for obstacles in the lane. Must be positive.

    The cones on the edges keep the lane's own cones out, so it can be longer than the corridor: about as far as the
    camera sees the lane. Driving around an obstacle needs it seen early, before it is within the stopping distance.
    """
    min_points: int = 3
    """Fewest returns that make an obstacle, in the corridor or in one object in the lane. Must be an integer of at
    least 1.

    A scan of 720 rays spaces them 0.5 degrees apart, so an object 0.1 m wide and 1.5 m away gives about 7 returns.
    """
    lane_edge_margin_m: float = 0.10
    """Returns this close in metres inside either lane edge, or outside the lane, are not obstacles in the lane. Must
    be finite and at least 0.

    It covers a cone's radius at the lidar's height and the error of the fitted edges. Larger misses objects close to
    the cones; smaller lets the cones count as obstacles.
    """
    cluster_gap_m: float = 0.15
    """Neighbouring returns, in scan order, closer than this in metres belong to one object in the lane. Must be
    positive.

    Returns on one face are a few centimetres apart, while separate objects at different distances leave a jump.
    """

    def __post_init__(self):
        for name, lowest in (("beam_count", 2), ("min_points", 1)):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < lowest:
                raise ValueError(f"obstacle_detection.{name} must be an integer of at least {lowest}, not {value}")
        if not (math.isfinite(self.field_of_view_deg) and 0.0 < self.field_of_view_deg <= 360.0):
            raise ValueError(f"obstacle_detection.field_of_view_deg must be within (0, 360], "
                             f"not {self.field_of_view_deg}")
        for name in ("min_range_m", "car_front_m", "lane_edge_margin_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f"obstacle_detection.{name} must be finite and at least 0, not {value}")
        for name in ("max_range_m", "corridor_half_width_m", "corridor_length_m", "lane_length_m", "cluster_gap_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"obstacle_detection.{name} must be positive and finite, not {value}")
        if self.min_range_m >= self.max_range_m:
            raise ValueError("obstacle_detection.min_range_m must be below max_range_m")


def detect_obstacles(observations: CarObservations, config: ObstacleDetectionConfig,
                     path_curvature_per_m: float = 0.0, *,
                     path_points_m: list[tuple[float, float]] | None = None,
                     path_error_m: float = 0.0) -> ObstacleDetection | None:
    """Finds what the lidar sees ahead of the car, and the nearest obstacle in its path.

    :param observations: The observations taken by the car at the current policy step.
    :param config: The obstacle detection settings.
    :param path_curvature_per_m: Curvature in 1/m of the path the car will drive, positive turning left, normally the
        curvature it is steering along. 0.0 gives a straight corridor ahead.
    :param path_points_m: Optional geometric steering preview in base_link, starting at (0, 0). When present, the
        corridor follows these segments rather than extending the initial steering curvature indefinitely.
    :param path_error_m: Nonnegative geometric approximation margin added to a preview's half width, in metres.
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
    segments = None if path_points_m is None else _path_segments(path_points_m)
    if not math.isfinite(path_error_m) or path_error_m < 0.:
        raise ValueError("Obstacle preview error must be finite and nonnegative")
    half_width = config.corridor_half_width_m + (path_error_m if segments is not None else 0.)
    for point in lidar.points:
        x, y = point.pos.x, point.pos.y
        # Forwards fan: the beam nearest the return's direction from base_link's origin
        beam = round((math.atan2(y, x) + half_view_rad) / beam_spacing_rad)
        distance = math.hypot(x, y)
        if 0 <= beam < config.beam_count and distance < ranges[beam]:
            ranges[beam] = max(config.min_range_m, distance)
        if (segments is not None
                and distance > segments[-1][5] + segments[-1][4] + half_width):
            continue  # Euclidean range exceeds the entire preview plus its margin.
        # Corridor: the return's position along the path and to its side
        along, left = (_path_coordinates(x, y, path_curvature_per_m) if segments is None
                       else _preview_coordinates(x, y, segments))
        if (config.car_front_m < along <= config.car_front_m + config.corridor_length_m
                and abs(left) <= half_width):
            in_path.append((along, point.pos))
    in_path.sort(key=lambda item: item[0])
    if len(in_path) >= config.min_points:
        along, nearest = in_path[config.min_points - 1]
        distance_m = along - config.car_front_m
    else:
        distance_m, nearest = math.inf, None
    return ObstacleDetection(ranges, distance_m, nearest, len(in_path), path_curvature_per_m, lidar.sensor_age)


def _path_segments(points: list[tuple[float, float]]) -> list[tuple[float, ...]]:
    """Precompute nonzero preview segments and their cumulative distances once per scan."""
    if len(points) < 2 or points[0] != (0., 0.) or any(not math.isfinite(v) for point in points for v in point):
        raise ValueError("Obstacle preview must contain finite points starting at (0, 0)")
    segments, along = [], 0.
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        if length > 1e-9:
            segments.append((ax, ay, dx, dy, length, along))
            along += length
    if not segments:
        raise ValueError("Obstacle preview must have positive length")
    return segments


def _preview_coordinates(x: float, y: float, segments: list[tuple[float, ...]]) -> tuple[float, float]:
    """Distance along and signed distance to the nearest segment, including the finite path's end caps."""
    nearest_sq, result = math.inf, (0., math.inf)
    for ax, ay, dx, dy, length, start in segments:
        t = min(1., max(0., ((x - ax) * dx + (y - ay) * dy) / (length * length)))
        px, py = x - ax - t * dx, y - ay - t * dy
        distance_sq = px * px + py * py
        if distance_sq < nearest_sq:
            nearest_sq = distance_sq
            result = start + t * length, math.copysign(math.sqrt(distance_sq), dx * py - dy * px)
    return result


def detect_lane_obstacles(observations: CarObservations, lane: LaneDetection | None,
                          config: ObstacleDetectionConfig) -> list[LaneObstacle] | None:
    """Finds the objects inside the lane ahead of the car, in lane coordinates.

    Only returns ahead of the car's front, at most lane_length_m along the lane, and more than lane_edge_margin_m
    inside both edges count. Neighbouring returns in scan order, closer than cluster_gap_m, form one object, and an
    object needs at least min_points returns, so stray returns do not count.

    :param observations: The observations taken by the car at the current policy step.
    :param lane: The lane detected at this step, or None.
    :param config: The obstacle detection settings.
    :return: The objects, nearest first, or None when there are no fresh Cartesian lidar points or no detected lane
        (lane is None or has no centre line). An empty list means the lane ahead is clear.
    """
    lidar = observations.lidar_cartesian
    if lidar is None or lane is None or not lane.has_lane:
        return None
    centre_line = lane.centre_line
    inside_m = lane.lane_width_m / 2.0 - config.lane_edge_margin_m
    returns = []
    for point in lidar.points:
        x, y = point.pos.x, point.pos.y
        along = centre_line.project(x, y) - config.car_front_m
        left = centre_line.signed_distance(x, y)
        if 0.0 < along <= config.lane_length_m and abs(left) < inside_m:
            returns.append((point.scan_index, x, y, along, left))
    returns.sort()
    groups = []
    for item in returns:
        if groups and math.dist(groups[-1][-1][1:3], item[1:3]) < config.cluster_gap_m:
            groups[-1].append(item)
        else:
            groups.append([item])
    # The scan's first and last rays point the same way, straight ahead with the default mounting (turned around), so
    # an object there is split between the first and last groups
    if len(groups) > 1 and math.dist(groups[-1][-1][1:3], groups[0][0][1:3]) < config.cluster_gap_m:
        groups[0] = groups.pop() + groups[0]
    obstacles = [LaneObstacle(min(item[3] for item in group), max(item[3] for item in group),
                              min(item[4] for item in group), max(item[4] for item in group), len(group))
                 for group in groups if len(group) >= config.min_points]
    return sorted(obstacles, key=lambda obstacle: obstacle.distance_m)


def nearest_in_band(obstacles: list[LaneObstacle], right_m: float, left_m: float) -> float:
    """
    :param obstacles: Objects in the lane, from detect_lane_obstacles.
    :param right_m: Right side of a band along the lane, in metres left of the centre line.
    :param left_m: Left side of the band, in metres left of the centre line; at least right_m.
    :return: Distance in metres along the lane from beside the car's front to the nearest object that reaches into
        the band, or math.inf when none does.
    """
    return min((obstacle.distance_m for obstacle in obstacles
                if obstacle.right_m <= left_m and obstacle.left_m >= right_m), default=math.inf)


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
