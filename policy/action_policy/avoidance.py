"""
avoidance.py

This file plans the line across the lane that the MPC follows past the obstacles inside it.

The lane is bounded by its two cone rows, and the MPC normally follows a line along it (its centre, moved by
MpcPolicy.lane_offset_m). An obstacle in the lane becomes part of the boundary on one side: the car passes through the
free gap between it and the other boundary, along the middle of that gap. While the car's body is beside the obstacles
(from before the car would start stopping for them, or pass_distance_m before its front reaches them if that is
farther, until pass_distance_m after its rear has passed them), the line is the middle of the gap, kept at least
obstacle_clearance_m from the obstacles and cone_clearance_m from the cones on the car's sides. Elsewhere it is the
usual line, and smooth ramps join the two, no sharper than max_ramp_curvature_per_m.

The line is planned again at every step. The lidar only sees the obstacles ahead of the car's front, so the obstacles
reaching the car's front are remembered in lane coordinates and moved back by the distance driven (from the wheel
speed) until the car has passed them and ramped back.

An obstacle with no gap wide enough beside it, or with a ramp too sharp to reach it, does not change the line. MpcPolicy
still stops for an obstacle in the car's way, so stopping stays the fallback when the car cannot drive around it.

It keeps state between policy steps, and its loops are bounded by the number of obstacles, so it is safe to run inside
the real-time policy step.
"""
from __future__ import annotations

import dataclasses
import logging
import math
from dataclasses import dataclass

from policy.action_policy.lane_path import LanePath, PathSegment
from policy.input_output import CarObservations
from policy.lane_detection.lanes import LaneDetection
from policy.obstacle_detection.obstacles import LaneObstacle

_LOGGER = logging.getLogger(__name__)

REMEMBER_WITHIN_M = 0.3
"""Obstacles seen this close in metres to the car's front are remembered, so they are still known once the car is
beside them. It must exceed the distance driven in one policy step."""
MAX_REMEMBERED = 16
"""Most obstacles remembered at once; the farthest behind are forgotten first."""


@dataclass(frozen=True)
class AvoidanceConfig:
    """Settings for driving around obstacles in the lane, loaded from the avoidance parameters in
    config/ai4r_policy.yaml.

    These defaults are also the parameters' defaults in policy_node. Invalid settings raise a ValueError, which stops
    policy_node from starting.
    """
    enabled: bool = True
    """Whether the car moves over in the lane to pass an obstacle. Must be true or false. When false, or when there is
    no room, the MPC only stops for obstacles in the car's way."""
    car_width_m: float = 0.25
    """Width of the car in metres. Must be positive. The default is the notebook car's; measure the real car."""
    car_rear_m: float = 0.30
    """Distance in metres from base_link's origin backwards to the car's rear. Must be at least 0. Measure the real
    car; with obstacle_detection.car_front_m it gives the car's length."""
    cone_radius_m: float = 0.05
    """Radius in metres of a cone at its base. Must be at least 0. The lane edges pass through the cones' centres."""
    cone_clearance_m: float = 0.10
    """Smallest gap in metres between the car's side and a cone while passing an obstacle. Must be at least 0. The
    line is in the middle of the free gap, so the gap is usually larger."""
    obstacle_clearance_m: float = 0.10
    """Smallest gap in metres between the car's side and an obstacle it passes. Must be at least 0.

    Half the car's width plus this must be at least obstacle_detection.corridor_half_width_m, or the line that passes
    the obstacle would still count as blocked, and the car would stop instead (MovementPolicy checks it).
    """
    pass_distance_m: float = 0.30
    """Distance in metres before the car's front reaches an obstacle, and after its rear has passed it, over which the
    line stays beside the obstacle. Must be at least 0. Before the obstacle, the line is beside it from the stopping
    distance instead when that is longer, so the car has moved over before it would stop for the obstacle."""
    max_ramp_curvature_per_m: float = 0.30
    """Sharpest curvature in 1/m a ramp between the usual line and a line beside an obstacle may add to the lane's
    own. Must be positive and within the MPC's curvature limits (MovementPolicy checks it); on a curve it is also kept
    within them with the lane's curvature added. A ramp moving the line by d metres is pi * sqrt(d / (2 * this))
    metres long."""
    side_hysteresis_m: float = 0.05
    """How much more room in metres the other side of an obstacle must leave before the car changes the side it passes
    it on. Must be at least 0. Once the car's front has reached an obstacle, the side no longer changes."""
    lookahead_m: float = 0.30
    """Distance in metres along the lane ahead of base_link's origin at which the planned line is given to the MPC to
    follow. Must be at least 0. The MPC follows one line over its horizon, so looking ahead lets it start each ramp in
    time."""

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError(f"avoidance.enabled must be true or false, not {self.enabled}")
        for name in ("car_width_m", "max_ramp_curvature_per_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"avoidance.{name} must be positive and finite, not {value}")
        for name in ("car_rear_m", "cone_radius_m", "cone_clearance_m", "obstacle_clearance_m", "pass_distance_m",
                     "side_hysteresis_m", "lookahead_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f"avoidance.{name} must be finite and at least 0, not {value}")


@dataclass
class _Group:
    """Obstacles whose stretches of line overlap, so the car passes them all on one line."""
    obstacles: list[LaneObstacle]
    """The obstacles, with distances from the car's front."""
    start_m: float
    """Distance in metres along the lane from beside the car's origin where the line must be beside them."""
    end_m: float
    """Distance in metres along the lane where the line may leave them."""
    offset_m: float | None
    """The line beside them, in metres left of the lane centre, or None when no gap beside them is wide enough."""
    locked: bool
    """Whether the car's front has reached one of them, so the side it passes them on no longer changes."""
    lowest_m: float | None = None
    """Lowest line in metres left of the lane centre that still keeps the clearances beside them, or None."""
    highest_m: float | None = None
    """Highest such line, or None."""

    def allows(self, offset_m: float) -> bool:
        """
        :return: Whether a line offset_m left of the lane centre keeps the clearances beside them.
        """
        return self.lowest_m is not None and self.lowest_m <= offset_m <= self.highest_m


class LaneAvoidance:
    """Plans the line across the lane for the MPC to follow past obstacles, remembering the obstacles beside the car.

    Create one when the node starts and call step once per policy step.
    """

    def __init__(self, config: AvoidanceConfig, car_front_m: float, max_left_curvature_per_m: float = math.inf,
                 max_right_curvature_per_m: float = math.inf, stopping_distance_m: float = 0.0):
        """
        :param config: The avoidance settings.
        :param car_front_m: Distance in metres from base_link's origin forwards to the car's front
            (obstacle_detection.car_front_m), to which the obstacle distances are measured.
        :param max_left_curvature_per_m: Sharpest curvature in 1/m the car can drive to the left (the MPC's limit).
        :param max_right_curvature_per_m: Sharpest curvature in 1/m the car can drive to the right, as a positive
            number.
        :param stopping_distance_m: Distance in metres from the car's front within which MpcPolicy stops for an
            obstacle in its way at the target speed, or 0.0 when it does not stop for obstacles.
        """
        self.config = config
        self.car_length_m = car_front_m + config.car_rear_m
        """Length of the car in metres."""
        self.max_left_curvature_per_m = max_left_curvature_per_m
        self.max_right_curvature_per_m = max_right_curvature_per_m
        self.approach_m = max(config.pass_distance_m, stopping_distance_m)
        """Distance in metres before the car's front reaches an obstacle from which the line is beside it."""
        self.reset()

    def reset(self):
        """Forgets the previous run. This is done on the first step after entering the publishing-policy state."""
        self.remembered: list[LaneObstacle] = []
        """Obstacles that came within REMEMBER_WITHIN_M of the car's front, with their distances from the car's front
        reduced by the distance driven since (negative once beside or behind it)."""
        self.path: LanePath | None = None
        """The line planned at the latest step with a lane, or None before the first."""
        self.driven_since_path_m = 0.0
        """Distance in metres driven since path was planned."""

    def step(self, observations: CarObservations, lane: LaneDetection | None, obstacles: list[LaneObstacle] | None,
             usual_offset_m: float | None) -> LanePath | None:
        """Plans the line for this policy step.

        :param observations: The observations at this step, for the wheel speed and the time since the last step.
        :param lane: The lane detected at this step, or None.
        :param obstacles: The objects inside the lane ahead at this step (detect_lane_obstacles), or None without a lane
            or fresh lidar. Without fresh lidar, only the remembered obstacles are passed.
        :param usual_offset_m: The line the MPC follows away from obstacles (MpcPolicy.usual_offset_m), in metres left
            of the lane centre, or None without a lane.
        :return: The planned line, or None when avoidance is off or there is no lane to plan in.
        """
        if observations.policy.is_first_policy_step:
            self.reset()
        if not self.config.enabled:
            return None
        if not observations.policy.is_first_policy_step and observations.car.wheel_speed is not None:
            driven_m = observations.car.wheel_speed * observations.policy.dt
            self.remembered = [_moved_back(obstacle, driven_m) for obstacle in self.remembered]
            self.driven_since_path_m += driven_m
        if lane is None or not lane.has_lane or usual_offset_m is None:
            return None
        config = self.config
        fresh = obstacles or []
        # Forget an obstacle once the car's rear has passed it and the line could have ramped back from it
        surface_m = lane.lane_width_m / 2.0 - config.cone_radius_m
        forget_m = (self.car_length_m + config.pass_distance_m
                    + _ramp_length_m(surface_m, config.max_ramp_curvature_per_m))
        self.remembered = [obstacle for obstacle in self.remembered if obstacle.far_distance_m > -forget_m]
        for obstacle in fresh:
            if obstacle.distance_m <= REMEMBER_WITHIN_M:
                self.remembered = _remember(self.remembered, obstacle)
        # Fresh lidar covers the lane ahead of the car's front, and the remembered obstacles the rest, as well as an
        # obstacle close ahead that the lidar misses at a step; one seen both ways is passed on one line
        known = fresh + self.remembered
        # A ramp curves both ways about the lane's own curvature, so on a curve it has less room towards the turn
        lane_curvature = lane.path_curvature_per_m
        limit = self.max_left_curvature_per_m if lane_curvature >= 0.0 else self.max_right_curvature_per_m
        curvature_room = min(config.max_ramp_curvature_per_m, limit - abs(lane_curvature))
        path = self._plan(known, surface_m, usual_offset_m, curvature_room)
        if bool(path.segments) != bool(self.path is not None and self.path.segments):
            if path.segments:
                _LOGGER.info("Obstacle in the lane: planned a line around it, %.2f m left of the lane centre beside it",
                             path.segments[0].end_offset_m)
            else:
                _LOGGER.info("No obstacle to drive around in the lane: back to the usual line")
        self.path = path
        self.driven_since_path_m = 0.0
        return path

    def _plan(self, obstacles: list[LaneObstacle], surface_m: float, usual_m: float,
              curvature_room: float) -> LanePath:
        """Plans the line past the obstacles.

        :param obstacles: The known obstacles, with distances from the car's front.
        :param surface_m: Distance in metres from the lane centre to the inner side of the cones.
        :param usual_m: The usual line in metres left of the lane centre.
        :param curvature_room: Curvature in 1/m left for the ramps on this lane.
        :return: The planned line.
        """
        if curvature_room <= 0.0:
            # The lane already turns as sharply as the car can: no room to ramp across it
            return LanePath(usual_m, ())
        # The stretch where base_link must be beside each obstacle: from approach_m before the car's front reaches it
        # until pass_distance_m after the rear has passed it
        after_m = self.car_length_m + self.config.pass_distance_m
        spans = sorted(((obstacle.distance_m - self.approach_m, obstacle.far_distance_m + after_m, obstacle)
                        for obstacle in obstacles), key=lambda span: span[0])
        groups = []
        for start_m, end_m, obstacle in spans:
            if groups and start_m <= groups[-1][1]:
                groups[-1][1] = max(groups[-1][1], end_m)
                groups[-1][2].append(obstacle)
            else:
                groups.append([start_m, end_m, [obstacle]])
        planned: list[_Group] = []
        for start_m, end_m, members in groups:
            group = self._group(members, start_m, end_m, surface_m)
            if group.offset_m is None:
                # No room beside it: it does not change the line, and MpcPolicy stops if it is in the car's way
                continue
            # Close behind the previous group: stay on its line when that line also clears this group, so the car
            # does not ramp across the lane and back between them
            if planned and group.allows(planned[-1].offset_m) and not _can_join(planned[-1], group, usual_m,
                                                                              curvature_room):
                group = dataclasses.replace(group, offset_m=planned[-1].offset_m)
            # Too close to the previous group to ramp between them: pass both on one line, or drop the later one
            while planned and not _can_join(planned[-1], group, usual_m, curvature_room):
                if planned[-1].locked:
                    group = None
                    break
                merged = self._group(planned[-1].obstacles + group.obstacles, planned[-1].start_m,
                                     max(planned[-1].end_m, group.end_m), surface_m)
                if merged.offset_m is None:
                    group = None
                    break
                planned.pop()
                group = merged
            if group is not None:
                planned.append(group)
        return LanePath(usual_m, _segments(planned, usual_m, curvature_room))

    def _group(self, obstacles: list[LaneObstacle], start_m: float, end_m: float, surface_m: float) -> _Group:
        """Chooses the gap the car passes the obstacles through, and the line in it.

        :param obstacles: The obstacles passed on one line.
        :param start_m: Where the line must be beside them, in metres along the lane from beside the car's origin.
        :param end_m: Where the line may leave them.
        :param surface_m: Distance in metres from the lane centre to the inner side of the cones.
        :return: The group, with its line, or with None when no gap is wide enough.
        """
        config = self.config
        half_width_m = config.car_width_m / 2.0
        # The free gaps across the lane between the cones and the obstacles: (right side, left side, whether each
        # side is an obstacle)
        gaps = []
        edge_m, edge_is_obstacle = -surface_m, False
        for right_m, left_m in sorted((max(obstacle.right_m, -surface_m), min(obstacle.left_m, surface_m))
                                      for obstacle in obstacles):
            if right_m >= left_m:
                # Within the cones, which bound the lane anyway
                continue
            if right_m > edge_m:
                gaps.append((edge_m, right_m, edge_is_obstacle, True))
            if left_m > edge_m:
                edge_m, edge_is_obstacle = left_m, True
        if edge_m < surface_m:
            gaps.append((edge_m, surface_m, edge_is_obstacle, False))
        # The lines in each gap that keep the car's sides clear of both sides
        options = [(right_m, left_m,
                    right_m + half_width_m + (config.obstacle_clearance_m if right_is_obstacle
                                              else config.cone_clearance_m),
                    left_m - half_width_m - (config.obstacle_clearance_m if left_is_obstacle
                                             else config.cone_clearance_m))
                   for right_m, left_m, right_is_obstacle, left_is_obstacle in gaps]
        previous_m = (None if self.path is None
                      else self.path.offset_at((start_m + end_m) / 2.0 + self.driven_since_path_m))
        current = next((option for option in options
                        if previous_m is not None and option[0] < previous_m < option[1]), None)
        locked = current is not None and any(obstacle.distance_m <= 0.0 for obstacle in obstacles)
        if locked:
            # Beside them: keep to the gap the car is in, rather than move across the lane into them
            chosen = current
        else:
            open_options = [option for option in options if option[2] <= option[3]]
            chosen = max(open_options, key=lambda option: option[3] - option[2], default=None)
            if (chosen is not None and current is not None and current[2] <= current[3]
                    and (chosen[3] - chosen[2]) - (current[3] - current[2]) <= config.side_hysteresis_m):
                chosen = current
        if chosen is None:
            offset_m = None
        elif locked and previous_m is not None and chosen[2] <= previous_m <= chosen[3]:
            # Beside them on a line that still clears them: keep it, rather than move to the middle of the gap, which
            # changes as the obstacles ahead are passed or come into view
            offset_m = previous_m
        elif chosen[2] <= chosen[3]:
            # The middle of the free gap, moved only as far as the clearances need
            offset_m = min(max((chosen[0] + chosen[1]) / 2.0, chosen[2]), chosen[3])
        else:
            # Beside them with the gap now too narrow: hold the line rather than move it into them
            offset_m = previous_m
        if chosen is None or chosen[2] > chosen[3]:
            return _Group(obstacles, start_m, end_m, offset_m, locked)
        return _Group(obstacles, start_m, end_m, offset_m, locked, chosen[2], chosen[3])


def _moved_back(obstacle: LaneObstacle, driven_m: float) -> LaneObstacle:
    """
    :param obstacle: A remembered obstacle, with distances from the car's front.
    :param driven_m: Distance in metres driven since.
    :return: The obstacle with its distances from where the car's front is now.
    """
    return dataclasses.replace(obstacle, distance_m=obstacle.distance_m - driven_m,
                               far_distance_m=obstacle.far_distance_m - driven_m)


def _remember(remembered: list[LaneObstacle], obstacle: LaneObstacle) -> list[LaneObstacle]:
    """
    :param remembered: The remembered obstacles.
    :param obstacle: An obstacle seen at this step.
    :return: The remembered obstacles with it added, joined with any it overlaps (along and across the lane) into one
        that covers both, at most MAX_REMEMBERED of them.
    """
    kept = []
    for other in remembered:
        if (other.distance_m <= obstacle.far_distance_m and obstacle.distance_m <= other.far_distance_m
                and other.right_m <= obstacle.left_m and obstacle.right_m <= other.left_m):
            obstacle = LaneObstacle(min(obstacle.distance_m, other.distance_m),
                                    max(obstacle.far_distance_m, other.far_distance_m),
                                    min(obstacle.right_m, other.right_m), max(obstacle.left_m, other.left_m),
                                    max(obstacle.point_count, other.point_count))
        else:
            kept.append(other)
    kept.append(obstacle)
    return sorted(kept, key=lambda other: other.far_distance_m)[-MAX_REMEMBERED:]


def _ramp_length_m(change_m: float, curvature_per_m: float) -> float:
    """
    :param change_m: How far the ramp moves the line across the lane, in metres.
    :param curvature_per_m: Sharpest curvature the ramp may need, in 1/m.
    :return: The shortest ramp length in metres, math.inf when no curvature is left for it.
    """
    if change_m == 0.0:
        return 0.0
    if curvature_per_m <= 0.0:
        return math.inf
    return math.pi * math.sqrt(abs(change_m) / (2.0 * curvature_per_m))


def _can_join(previous: _Group, group: _Group, usual_m: float, curvature_room: float) -> bool:
    """
    :return: Whether the line can ramp from the previous group's line to the next group's within the stretch between
        them, either through the usual line or directly.
    """
    gap_m = group.start_m - previous.end_m
    via_usual_m = (_ramp_length_m(previous.offset_m - usual_m, curvature_room)
                   + _ramp_length_m(group.offset_m - usual_m, curvature_room))
    return min(via_usual_m, _ramp_length_m(group.offset_m - previous.offset_m, curvature_room)) <= gap_m


def _segments(groups: list[_Group], usual_m: float, curvature_room: float) -> tuple[PathSegment, ...]:
    """
    :param groups: The groups the line passes, in order along the lane, each joinable to the next (_can_join).
    :param usual_m: The usual line in metres left of the lane centre.
    :param curvature_room: Curvature in 1/m left for the ramps on this lane.
    :return: The line's segments: a ramp onto each group's line, the line beside it, and a ramp back.
    """
    segments = []

    def add(start_m, end_m, start_offset_m, end_offset_m):
        if end_m > start_m and not start_offset_m == end_offset_m == usual_m:
            segments.append(PathSegment(start_m, end_m, start_offset_m, end_offset_m))

    for index, group in enumerate(groups):
        if index == 0:
            add(group.start_m - _ramp_length_m(group.offset_m - usual_m, curvature_room), group.start_m, usual_m,
                group.offset_m)
        else:
            previous = groups[index - 1]
            out_m = _ramp_length_m(previous.offset_m - usual_m, curvature_room)
            in_m = _ramp_length_m(group.offset_m - usual_m, curvature_room)
            if out_m + in_m <= group.start_m - previous.end_m:
                add(previous.end_m, previous.end_m + out_m, previous.offset_m, usual_m)
                add(group.start_m - in_m, group.start_m, usual_m, group.offset_m)
            else:
                add(previous.end_m, group.start_m, previous.offset_m, group.offset_m)
        add(group.start_m, group.end_m, group.offset_m, group.offset_m)
    if groups:
        last = groups[-1]
        add(last.end_m, last.end_m + _ramp_length_m(last.offset_m - usual_m, curvature_room), last.offset_m, usual_m)
    return tuple(segments)
