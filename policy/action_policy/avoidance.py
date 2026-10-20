"""
avoidance.py

This file plans the line across the lane that the MPC follows past the obstacles inside it. It is a local trajectory
planner in the lane's Frenet frame: s is the distance along the lane's centre line from beside the car's origin
(base_link), and d the offset left of the centre line (MpcPolicy.lane_offset_m and the usual line are offsets too).

Rather than plan around one obstacle and then the next, it plans one line past all the obstacles it knows at once:
1. Each obstacle becomes a stretch of lane the car's body must keep clear of: from before the car would start
   stopping for it (or pass_distance_m before it, if farther) until pass_distance_m after it, the car's sides must stay
   obstacle_clearance_m from it and cone_clearance_m from the cones.
2. Candidate lines are generated. Obstacles whose stretches overlap are passed on one held offset, and the offsets
   tried beside them are the middle of each free gap that is wide enough, the point of that gap nearest the usual
   line, the previous step's line and the offsets held beside the obstacles before. Quintic polynomials d(s) join the
   held offsets: centred on the stretch between two obstacles (an S between obstacles on opposite sides, which may
   start while the car is still beside the first and end once it is beside the next), spread over that whole
   stretch, or back to the usual line and out again. Each matches the offset, slope and curvature where it starts
   and ends, and the first starts from where the previous step's line is now, so the line never jumps and the
   steering changes smoothly.
3. Each candidate is checked along its whole length, every SAMPLE_STEP_M. The car's body is a rectangle turned along
   the line, and only the part of it beside an obstacle must keep clear of it, so the rear may still be beside one
   obstacle while the front turns past the next. Every move must be no sharper than max_ramp_curvature_per_m (less on
   a curve, so the car's limits hold with the lane's own curvature).
4. The line that passes the most obstacles, nearest first, wins; then the one that is beside them before the stopping
   distance; then the cheapest. The cost adds up along the line its offset from the usual line, its curvature and its
   change from the previous step's line, and takes off the room it leaves beside the obstacles (the *_weight
   settings).
The lines are searched run by run, keeping the MAX_HELD cheapest partial lines after each run, so the work grows with
the number of runs of obstacles rather than with the number of ways to combine them.

The line is planned again at every step. The lidar only sees the obstacles ahead of the car's front, so the obstacles
reaching the car's front are remembered in lane coordinates and moved back by the distance driven (from the wheel
speed) until the car has passed them. Obstacles are assumed not to move.

An obstacle that no candidate can pass does not change the line beyond the obstacles before it. MpcPolicy still stops
for an obstacle in the car's way (in_way), so stopping stays the fallback when the car cannot drive around it.

It keeps state between policy steps, and its loops are bounded by MAX_PLANNED_RUNS, MAX_HELD, MAX_REMEMBERED and
MAX_HORIZON_M, so it is safe to run inside the real-time policy step.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, replace

import numpy as np

from policy.action_policy.lane_path import LanePath, LineState, PathSegment, move_length_m
from policy.input_output import CarObservations
from policy.lane_detection.lanes import LaneDetection
from policy.obstacle_detection.obstacles import LaneObstacle

_LOGGER = logging.getLogger(__name__)

REMEMBER_WITHIN_M = 0.3
"""Obstacles seen this close in metres to the car's front are remembered, so they are still known once the car is
beside them. It must exceed the distance driven in one policy step."""
MAX_REMEMBERED = 16
"""Most obstacles remembered at once; the farthest behind are forgotten first."""
JOIN_ACROSS_M = 0.05
"""Obstacles closer together than this in metres across the lane, and JOIN_ALONG_M along it, are planned past as one:
far too narrow a gap for the car, and usually the faces of one object. Joining them also keeps the checks few."""
JOIN_ALONG_M = 0.3
"""See JOIN_ACROSS_M. The car is longer than this, so it could not move between such obstacles anyway."""
SAMPLE_STEP_M = 0.05
"""Spacing in metres along the lane of the points at which each candidate line is checked and costed."""
MAX_HORIZON_M = 12.0
"""Longest stretch of lane in metres a line is planned and checked over, from beside the car's origin."""
MAX_PLANNED_RUNS = 3
"""Most runs of obstacles, nearest first, planned past at one step; farther ones are planned once nearer."""
MAX_HELD = 6
"""Most partial lines kept after each run to plan the next, the cheapest first."""
TARGET_SPREAD = 5
"""Number of evenly spread offsets tried across each free gap, from one side's clearance to the other's."""
MOVE_STRETCHES = (1.0, 1.25, 1.5, 2.0, 3.0)
"""Lengths tried, as multiples of the shortest move from a line parallel to the lane, for a move that starts while
the line is turning."""
TOLERANCE_M = 1e-6
"""Rounding allowance in metres when checking clearances and comparing offsets."""
_NEXT_CORNER = [1, 2, 3, 0]
"""For each corner of the car's body, the next one around it."""


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
    """Smallest gap in metres between the car's side and a cone while passing an obstacle. Must be at least 0."""
    obstacle_clearance_m: float = 0.10
    """Smallest gap in metres between the car's body and an obstacle it passes. Must be at least 0.

    Half the car's width plus this must be at least obstacle_detection.corridor_half_width_m, or the line that passes
    the obstacle would still count as blocked, and the car would stop instead (MovementPolicy checks it).
    """
    pass_distance_m: float = 0.05
    """Distance in metres along the lane before and after an obstacle over which the car's body also keeps the
    clearances from it, for the obstacle's unseen depth and the error of the distance driven. Must be at least 0.

    The line is planned to keep them from the stopping distance before the obstacle instead when that is longer, so
    the car has moved over before it would stop for the obstacle; only when that cannot be done does it move over
    later. Two obstacles on opposite sides need more than twice this between them for the car to weave between them.
    """
    max_ramp_curvature_per_m: float = 0.30
    """Sharpest curvature in 1/m a move across the lane may add to the lane's own. Must be positive and within the
    MPC's curvature limits (MovementPolicy checks it); on a curve it is also kept within them with the lane's curvature
    added. A move of d metres between lines parallel to the lane is sqrt(10 / sqrt(3) * d / this) metres long."""
    deviation_weight: float = 1.0
    """Cost per m^2 of offset from the usual line, per metre along the lane. Must be finite and at least 0. Higher
    keeps the car nearer the usual line, and passes an obstacle on the side that needs the smaller move."""
    smoothness_weight: float = 1.0
    """Cost per (1/m)^2 of curvature the line adds to the lane's, per metre along the lane. Must be finite and at
    least 0. Higher prefers gentle moves, such as straight across between two obstacles on opposite sides rather than
    back to the usual line and out again."""
    clearance_weight: float = 5.0
    """Reward per metre of room left beyond the clearances beside an obstacle, per metre along the lane. Must be finite
    and at least 0. With 0 the car passes as near the usual line as the clearances allow; higher moves it towards the
    middle of the free gap."""
    change_weight: float = 1.0
    """Cost per m^2 of difference from the line planned at the previous step, per metre along the lane. Must be finite
    and at least 0. It keeps the line steady from step to step, so the car keeps the side it passes an obstacle on
    while that side stays clear."""

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError(f"avoidance.enabled must be true or false, not {self.enabled}")
        for name in ("car_width_m", "max_ramp_curvature_per_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"avoidance.{name} must be positive and finite, not {value}")
        for name in ("car_rear_m", "cone_radius_m", "cone_clearance_m", "obstacle_clearance_m", "pass_distance_m",
                     "deviation_weight", "smoothness_weight", "clearance_weight", "change_weight"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f"avoidance.{name} must be finite and at least 0, not {value}")


@dataclass(frozen=True)
class _Run:
    """Obstacles whose stretches overlap, so the car passes them all on one held offset."""
    obstacles: tuple[LaneObstacle, ...]
    """The obstacles, with distances from the car's front."""
    start_m: float
    """Distance in metres along the lane from beside the car's origin at which the car's front reaches their
    stretch."""
    end_m: float
    """Distance in metres along the lane at which the car's rear leaves their stretch."""


@dataclass(frozen=True)
class _Partial:
    """A line planned past the runs so far, holding an offset beside the latest."""
    segments: tuple[PathSegment, ...]
    """Its segments, up to where it holds the offset."""
    held_m: float
    """The offset held beside the latest run, in metres left of the lane centre."""
    free_m: float
    """Distance in metres along the lane from which it holds that offset."""


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
        self.car_front_m = car_front_m
        self.car_length_m = car_front_m + config.car_rear_m
        """Length of the car in metres."""
        self.max_left_curvature_per_m = max_left_curvature_per_m
        self.max_right_curvature_per_m = max_right_curvature_per_m
        self.approach_m = max(config.pass_distance_m, stopping_distance_m)
        """Distance in metres before an obstacle from which the car should already keep clear of it."""
        half_width_m = config.car_width_m / 2.0
        self._corner_along = np.array([car_front_m, car_front_m, -config.car_rear_m, -config.car_rear_m])
        """The body's corners (front left, front right, rear right, rear left), metres ahead of base_link."""
        self._corner_across = np.array([half_width_m, -half_width_m, -half_width_m, half_width_m])
        """The same corners, metres left of base_link."""
        self._reach_m = math.hypot(max(car_front_m, config.car_rear_m), half_width_m)
        """Farthest a corner can be along the lane from base_link, in metres."""
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
            or fresh lidar. Without fresh lidar, the previous step's line is kept, moved back by the distance driven.
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
        fresh = obstacles or []
        # Forget an obstacle once the car's rear is pass_distance_m past it
        self.remembered = [obstacle for obstacle in self.remembered if not self._behind(obstacle)]
        for obstacle in fresh:
            if obstacle.distance_m <= REMEMBER_WITHIN_M:
                self.remembered = _remember(self.remembered, obstacle)
        # Fresh lidar covers the lane ahead of the car's front, and the remembered obstacles the rest, as well as an
        # obstacle close ahead that the lidar misses at a step. The pieces of one object (its faces, or one seen both
        # ways) are joined into one, which also keeps the number of checks down.
        known = []
        for obstacle in fresh + self.remembered:
            if not self._behind(obstacle):
                known = _joined(known, obstacle, JOIN_ALONG_M, JOIN_ACROSS_M)
        # A move curves both ways about the lane's own curvature, so on a curve it has less room towards the turn
        lane_curvature = lane.path_curvature_per_m
        limit = self.max_left_curvature_per_m if lane_curvature >= 0.0 else self.max_right_curvature_per_m
        curvature_room = min(self.config.max_ramp_curvature_per_m, limit - abs(lane_curvature))
        surface_m = lane.lane_width_m / 2.0 - self.config.cone_radius_m
        previous = None if self.path is None else self.path.moved_back(self.driven_since_path_m, usual_offset_m)
        if obstacles is None and previous is not None:
            # No fresh lidar: the obstacles ahead are unknown at this step, so keep the line planned past them
            path = previous
        else:
            path = self._plan(sorted(known, key=lambda obstacle: obstacle.distance_m), surface_m, usual_offset_m,
                              curvature_room, previous)
        if bool(path.segments) != bool(self.path is not None and self.path.segments):
            if path.segments:
                _LOGGER.info("Obstacle in the lane: planned a line around it, %.2f m left of the lane centre at most",
                             max(path.offset_range(0.0, path.segments[-1].end_m), key=abs))
            else:
                _LOGGER.info("No obstacle to drive around in the lane: back to the usual line")
        self.path = path
        self.driven_since_path_m = 0.0
        return path

    def in_way(self, path: LanePath, obstacle: LaneObstacle, right_margin_m: float, left_margin_m: float) -> bool:
        """Checks whether the car, following a line, would come too close to an obstacle across the lane.

        :param path: The line the car follows.
        :param obstacle: An obstacle in the lane, with distances from the car's front.
        :param right_margin_m: Room in metres the car's right side needs from an obstacle it passes on its left.
        :param left_margin_m: Room in metres the car's left side needs from an obstacle it passes on its right.
        :return: Whether some part of the car's body, while beside the obstacle, would be closer to it than that.
        """
        near_m = self.car_front_m + obstacle.distance_m
        far_m = self.car_front_m + obstacle.far_distance_m
        s = np.arange(max(0.0, near_m - self._reach_m), far_m + self._reach_m + SAMPLE_STEP_M, SAMPLE_STEP_M)
        offsets, slopes, _ = path.sample(s)
        corner_s, corner_d = self._corners(s, offsets, slopes)
        lowest, highest = _across_within(corner_s, corner_d, near_m, far_m)
        beside = np.isfinite(lowest)
        clear = ((lowest[beside] - obstacle.left_m >= right_margin_m - TOLERANCE_M)
                 | (obstacle.right_m - highest[beside] >= left_margin_m - TOLERANCE_M))
        return not bool(np.all(clear))

    def _behind(self, obstacle: LaneObstacle) -> bool:
        """
        :return: Whether the car's rear is more than pass_distance_m past the obstacle, so it no longer matters.
        """
        return obstacle.far_distance_m + self.car_length_m + self.config.pass_distance_m < 0.0

    def _plan(self, obstacles: list[LaneObstacle], surface_m: float, usual_m: float, curvature_room: float,
              previous: LanePath | None) -> LanePath:
        """Generates the candidate lines past the obstacles, and chooses the best (see the file's description).

        :param obstacles: The known obstacles, nearest first, with distances from the car's front.
        :param surface_m: Distance in metres from the lane centre to the inner side of the cones.
        :param usual_m: The usual line in metres left of the lane centre.
        :param curvature_room: Curvature in 1/m left for moves across this lane.
        :param previous: The previous step's line from beside where the car's origin is now, or None.
        :return: The chosen line.
        """
        if curvature_room <= 0.0:
            # The lane already turns as sharply as the car can: no room to move across it, so keep the line
            return previous if previous is not None else LanePath(usual_m, ())
        start = LineState(usual_m) if previous is None else previous.state_at(0.0)
        # Every line is checked and costed over the same stretch: past every obstacle, and far enough to move back to
        # the usual line from anywhere across the lane
        horizon_m = min(MAX_HORIZON_M, max([self.car_front_m + obstacle.far_distance_m + self.config.pass_distance_m
                                            + self.config.car_rear_m for obstacle in obstacles], default=0.0)
                        + move_length_m(2.0 * surface_m, curvature_room))
        s = np.arange(0.0, horizon_m + SAMPLE_STEP_M, SAMPLE_STEP_M)
        previous_offsets = previous.sample(s)[0] if previous is not None else np.full(s.shape, usual_m)
        # The previous step's line comes first, so it is kept when nothing is better
        candidates = [] if previous is None else [previous]
        candidates.append(LanePath(usual_m, tuple(self._back_to_usual_from(start, usual_m, curvature_room))))
        # Plan to keep clear of each obstacle from the stopping distance and, if that cannot pass as many, from
        # pass_distance_m before it
        for lead_m in sorted({self.approach_m, self.config.pass_distance_m}, reverse=True):
            candidates += self._search(self._runs(obstacles, lead_m), start, usual_m, curvature_room, s,
                                       previous_offsets, surface_m)
        return max(candidates, key=lambda path: self._rank(path, obstacles, s, previous_offsets, usual_m, surface_m))

    def _runs(self, obstacles: list[LaneObstacle], lead_m: float) -> list[_Run]:
        """
        :param obstacles: The known obstacles.
        :param lead_m: Distance in metres before an obstacle from which the car must keep clear of it.
        :return: The runs of obstacles whose stretches overlap, nearest first, at most MAX_PLANNED_RUNS of them.
        """
        runs: list[_Run] = []
        # Each obstacle's stretch, from lead_m before it to pass_distance_m after it
        for near_m, far_m, obstacle in sorted(((self.car_front_m + obstacle.distance_m - lead_m,
                                                self.car_front_m + obstacle.far_distance_m
                                                + self.config.pass_distance_m, obstacle)
                                               for obstacle in obstacles), key=lambda span: span[0]):
            start_m, end_m = near_m - self.car_front_m, far_m + self.config.car_rear_m
            if runs and near_m <= runs[-1].end_m - self.config.car_rear_m:
                runs[-1] = _Run(runs[-1].obstacles + (obstacle,), runs[-1].start_m, max(runs[-1].end_m, end_m))
            else:
                runs.append(_Run((obstacle,), start_m, end_m))
        return runs[:MAX_PLANNED_RUNS]

    def _search(self, runs: list[_Run], start: LineState, usual_m: float, curvature_room: float, s: np.ndarray,
                previous_offsets: np.ndarray, surface_m: float) -> list[LanePath]:
        """Finds lines past the first run, past the first two, and so on.

        After each run only the MAX_HELD cheapest partial lines are carried on to the next, preferring those that
        keep clear of the runs passed so far.

        :param runs: The runs of obstacles, nearest first.
        :param start: Where the previous step's line is now, at s = 0.
        :param usual_m: The usual line in metres left of the lane centre.
        :param curvature_room: Curvature in 1/m left for moves across this lane.
        :param s: The distances along the lane at which lines are checked.
        :param previous_offsets: The previous step's line at s.
        :param surface_m: Distance in metres from the lane centre to the inner side of the cones.
        :return: For each number of runs passed, the kept lines past them, each moving back to the usual line after.
        """
        lines = []
        kept: list[_Partial] = []
        for index, run in enumerate(runs):
            # (whether it seems to keep clear of this run, less its cost, partial line), by target offset and where it
            # starts holding it
            grown: dict[tuple[float, float], tuple[bool, float, _Partial]] = {}
            for target_m in self._targets(run, usual_m, previous_offsets, s, {partial.held_m for partial in kept},
                                          surface_m):
                if index == 0:
                    ways = [((), segments, free_m)
                            for segments, free_m in self._first_moves(start, target_m, run, usual_m, curvature_room)]
                else:
                    ways = [(partial.segments, segments, free_m) for partial in kept
                            for segments, free_m in self._ways(runs[index - 1], partial, run, target_m, usual_m,
                                                               curvature_room)]
                for before, segments, free_m in ways:
                    partial = _Partial(before + tuple(segments), target_m, free_m)
                    checked = self._check_partial(partial, runs, index, usual_m, s, previous_offsets, surface_m)
                    if checked is None:
                        continue
                    key = (round(target_m, 4), round(free_m, 3))
                    if key not in grown or checked > grown[key][:2]:
                        grown[key] = (*checked, partial)
            if not grown:
                # No line passes the runs before this one: the car stops if one is in its way
                break
            kept = [partial for _, _, partial in sorted(grown.values(), key=lambda item: item[:2],
                                                        reverse=True)[:MAX_HELD]]
            for partial in kept:
                lines.append(LanePath(usual_m, partial.segments
                                      + tuple(_hold(partial.free_m, run.end_m, partial.held_m, usual_m))
                                      + tuple(self._back_to_usual(max(partial.free_m, run.end_m), partial.held_m,
                                                                  usual_m, curvature_room))))
        return lines

    def _check_partial(self, partial: _Partial, runs: list[_Run], index: int, usual_m: float, s: np.ndarray,
                       previous_offsets: np.ndarray, surface_m: float) -> tuple[bool, float] | None:
        """
        :param partial: A partial line, holding its offset beside runs[index].
        :param runs: The runs of obstacles, nearest first.
        :param index: The latest run's index.
        :param usual_m: The usual line in metres left of the lane centre.
        :param s: The distances along the lane at which lines are checked.
        :param previous_offsets: The previous step's line at s.
        :param surface_m: Distance in metres from the lane centre to the inner side of the cones.
        :return: None when it does not keep clear of the runs before the latest (their lines are complete now);
            otherwise whether it keeps clear of the latest run while it holds the offset, and less its cost up to the
            latest run's end.
        """
        run = runs[index]
        inside = s < run.end_m
        line = LanePath(usual_m, partial.segments + tuple(_hold(partial.free_m, run.end_m, partial.held_m, usual_m)))
        offsets, slopes, curvatures = line.sample(s[inside])
        corner_s, corner_d = self._corners(s[inside], offsets, slopes)
        passed = [obstacle for before in runs[:index] for obstacle in before.obstacles]
        clear, room, _ = self._clearances(corner_s, corner_d, passed + list(run.obstacles), self.config.pass_distance_m,
                                          surface_m)
        if not np.all(clear[:len(passed)]):
            return None
        return (bool(np.all(clear[len(passed):])),
                -self._cost(offsets, curvatures, previous_offsets[inside], usual_m, float(np.sum(room[:len(passed)]))))

    def _targets(self, run: _Run, usual_m: float, previous_offsets: np.ndarray, s: np.ndarray, held: set[float],
                 surface_m: float) -> list[float]:
        """
        :param run: The run of obstacles.
        :param usual_m: The usual line in metres left of the lane centre.
        :param previous_offsets: The previous step's line at s.
        :param s: The distances along the lane at which lines are checked.
        :param held: The offsets held beside the run before, if any.
        :param surface_m: Distance in metres from the lane centre to the inner side of the cones.
        :return: The offsets to try holding beside the run, in metres left of the lane centre: in each free gap
            beside its obstacles that is wide enough, the middle, the point nearest the usual line, TARGET_SPREAD points
            spread across the offsets that keep the clearances (so a tight S between two obstacles can keep a little
            room beside both), the previous step's line beside the run and the offsets held beside the run before.
        """
        middle = int(np.searchsorted(s, (max(run.start_m, 0.0) + run.end_m) / 2.0))
        previous_m = float(previous_offsets[min(middle, len(s) - 1)])
        targets = set()
        for right_m, left_m, lowest_m, highest_m in self._gaps(run.obstacles, surface_m):
            if lowest_m > highest_m + TOLERANCE_M:
                continue
            targets.add(min(max((right_m + left_m) / 2.0, lowest_m), highest_m))
            targets.add(min(max(usual_m, lowest_m), highest_m))
            targets.update(np.linspace(lowest_m, max(lowest_m, highest_m), TARGET_SPREAD).tolist())
            targets.update(offset_m for offset_m in (previous_m, *held)
                           if lowest_m - TOLERANCE_M <= offset_m <= highest_m + TOLERANCE_M)
        return sorted(targets)

    def _gaps(self, obstacles: tuple[LaneObstacle, ...], surface_m: float) -> list[tuple[float, float, float, float]]:
        """
        :param obstacles: Obstacles passed on one held offset.
        :param surface_m: Distance in metres from the lane centre to the inner side of the cones.
        :return: The free gaps across the lane between the cones and the obstacles. For each, its right and left
            sides, and the lowest and highest offset of base_link that keep the car's sides clear of both by the
            clearances, all in metres left of the lane centre.
        """
        config = self.config
        half_width_m = config.car_width_m / 2.0
        # (right side, left side, whether each side is an obstacle)
        gaps = []
        edge_m, edge_is_obstacle = -surface_m, False
        for right_m, left_m in sorted((max(obstacle.right_m, -surface_m), min(obstacle.left_m, surface_m))
                                      for obstacle in obstacles):
            if right_m > left_m:
                # Within the cones, which bound the lane anyway. A face seen edge on has no width but still counts.
                continue
            if right_m > edge_m:
                gaps.append((edge_m, right_m, edge_is_obstacle, True))
            if left_m > edge_m:
                edge_m, edge_is_obstacle = left_m, True
        if edge_m < surface_m:
            gaps.append((edge_m, surface_m, edge_is_obstacle, False))
        return [(right_m, left_m,
                 right_m + half_width_m + (config.obstacle_clearance_m if right_is_obstacle else config.cone_clearance_m),
                 left_m - half_width_m - (config.obstacle_clearance_m if left_is_obstacle else config.cone_clearance_m))
                for right_m, left_m, right_is_obstacle, left_is_obstacle in gaps]

    def _first_moves(self, start: LineState, target_m: float, run: _Run, usual_m: float,
                     curvature_room: float) -> list[tuple[list[PathSegment], float]]:
        """
        :param start: Where the previous step's line is now, at s = 0.
        :param target_m: The offset to hold beside the first run.
        :param run: The first run of obstacles.
        :param usual_m: The usual line in metres left of the lane centre.
        :param curvature_room: Curvature in 1/m left for moves across this lane.
        :return: The ways from s = 0 to target_m, each as its segments and where it reaches target_m: there by the
            run's start if it can be (else as soon as it can).
        """
        if start.parallel:
            if abs(target_m - start.offset_m) <= TOLERANCE_M:
                return [([], 0.0)]
            # Hold the line, then move over with the shortest move, ending as the car's front reaches the run
            length_m = move_length_m(target_m - start.offset_m, curvature_room)
            begin_m = max(0.0, run.start_m - length_m)
            return [(_hold(0.0, begin_m, start.offset_m, usual_m)
                     + [PathSegment.move(begin_m, begin_m + length_m, start, LineState(target_m))], begin_m + length_m)]
        # Already turning across the lane: carry on smoothly from here
        move = _move_from(start, target_m, run.start_m, curvature_room)
        return [] if move is None else [([move], move.end_m)]

    def _ways(self, before: _Run, partial: _Partial, run: _Run, target_m: float, usual_m: float,
              curvature_room: float) -> list[tuple[list[PathSegment], float]]:
        """
        :param before: The run before.
        :param partial: The partial line past it, holding its offset.
        :param run: The next run.
        :param target_m: The offset to hold beside the next run.
        :param usual_m: The usual line in metres left of the lane centre.
        :param curvature_room: Curvature in 1/m left for moves across this lane.
        :return: The ways on to target_m, each as its segments after the partial line's and where it reaches target_m:
            keeping the offset when it does not change; the shortest move centred on the stretch between the runs,
            even when the car is longer than that stretch; a move spread over that whole stretch; or back to the usual
            line and out again within it.
        """
        held_m, free_m = partial.held_m, partial.free_m
        # The stretch where the car's body is beside neither run (negative when the car is longer than the gap)
        after_m, by_m = before.end_m, run.start_m
        ways = []
        moves = []
        if abs(target_m - held_m) <= TOLERANCE_M:
            ways.append(([], free_m))
        else:
            length_m = move_length_m(target_m - held_m, curvature_room)
            moves.append(((after_m + by_m - length_m) / 2.0, length_m))
            if by_m - after_m > length_m + SAMPLE_STEP_M:
                moves.append((after_m, by_m - after_m))
        for begin_m, length_m in moves:
            if begin_m >= free_m - TOLERANCE_M:
                ways.append((_hold(free_m, begin_m, held_m, usual_m)
                             + [PathSegment.move(begin_m, begin_m + length_m, LineState(held_m), LineState(target_m))],
                             begin_m + length_m))
        out_m = move_length_m(usual_m - held_m, curvature_room)
        in_m = move_length_m(target_m - usual_m, curvature_room)
        if out_m > 0.0 and in_m > 0.0 and out_m + in_m <= by_m - after_m and after_m >= free_m - TOLERANCE_M:
            ways.append((_hold(free_m, after_m, held_m, usual_m)
                         + [PathSegment.move(after_m, after_m + out_m, LineState(held_m), LineState(usual_m)),
                            PathSegment.move(by_m - in_m, by_m, LineState(usual_m), LineState(target_m))], by_m))
        return ways

    def _back_to_usual(self, begin_m: float, held_m: float, usual_m: float, curvature_room: float) -> list[PathSegment]:
        """
        :return: The shortest move from the offset held_m at begin_m back to the usual line.
        """
        length_m = move_length_m(usual_m - held_m, curvature_room)
        if length_m == 0.0:
            return []
        return [PathSegment.move(begin_m, begin_m + length_m, LineState(held_m), LineState(usual_m))]

    def _back_to_usual_from(self, start: LineState, usual_m: float, curvature_room: float) -> list[PathSegment]:
        """
        :return: The line that passes no obstacle: from where the previous step's line is now straight back to the
            usual line, as soon as it can.
        """
        if start.parallel:
            return self._back_to_usual(0.0, start.offset_m, usual_m, curvature_room)
        move = _move_from(start, usual_m, 0.0, curvature_room)
        return [] if move is None else [move]

    def _rank(self, path: LanePath, obstacles: list[LaneObstacle], s: np.ndarray, previous_offsets: np.ndarray,
              usual_m: float, surface_m: float) -> tuple[int, float, int, float]:
        """
        :param path: A candidate line.
        :param obstacles: The known obstacles, nearest first.
        :param s: The distances along the lane at which lines are checked.
        :param previous_offsets: The previous step's line at s.
        :param usual_m: The usual line in metres left of the lane centre.
        :param surface_m: Distance in metres from the lane centre to the inner side of the cones.
        :return: The line's rank, highest best: how many obstacles, nearest first, it keeps the clearances from; how
            near it comes to keeping them from the next one (the room it lacks there, as a negative number, 0.0 when it
            passes them all), so a line that only just misses them is kept rather than one that drives into the
            obstacle; how many of those it passes from the stopping distance; and less its cost.
        """
        offsets, slopes, curvatures = path.sample(s)
        corner_s, corner_d = self._corners(s, offsets, slopes)
        clear, room, worst = self._clearances(corner_s, corner_d, obstacles, self.config.pass_distance_m, surface_m)
        # Each count runs from the nearest obstacle up to the first one the line does not pass
        passed = _leading(clear)
        short_m = float(worst[passed]) if passed < len(obstacles) else 0.0
        in_time = _leading(self._clearances(corner_s, corner_d, obstacles[:passed], self.approach_m, surface_m)[0])
        return (passed, short_m, in_time,
                -self._cost(offsets, curvatures, previous_offsets, usual_m, float(np.sum(room[:passed]))))

    def _corners(self, s: np.ndarray, offsets: np.ndarray, slopes: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        :param s: Distances along the lane of base_link following a line.
        :param offsets: The line's offsets there.
        :param slopes: The line's slopes there; the car points along the line.
        :return: The body's corners at each of them (in the order of _corner_along): their distances along the lane
            and their offsets, each an array of shape (len(s), 4).
        """
        heading = np.arctan(slopes)[:, None]
        cos, sin = np.cos(heading), np.sin(heading)
        return (s[:, None] + self._corner_along * cos - self._corner_across * sin,
                offsets[:, None] + self._corner_along * sin + self._corner_across * cos)

    def _clearances(self, corner_s: np.ndarray, corner_d: np.ndarray, obstacles: list[LaneObstacle], lead_m: float,
                    surface_m: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Checks a line beside each obstacle: from lead_m before it until pass_distance_m after it, the part of the
        car's body there must keep obstacle_clearance_m from it, and the whole body cone_clearance_m from the cones.

        :param corner_s: The body's corners' distances along the lane at the points checked (_corners).
        :param corner_d: The body's corners' offsets there.
        :param obstacles: The obstacles, with distances from the car's front.
        :param lead_m: Distance in metres before each obstacle from which it is checked.
        :param surface_m: Distance in metres from the lane centre to the inner side of the cones.
        :return: For each obstacle: whether the body keeps both clearances; the room left beyond them, summed over the
            points checked, in metres; and the least room at any of them (negative where the body is too close, 0.0
            when it is never beside the obstacle).
        """
        config = self.config
        # Each obstacle's stretch and sides, shaped to broadcast against the points and the corners
        near_m = np.array([self.car_front_m + obstacle.distance_m - lead_m for obstacle in obstacles])
        far_m = np.array([self.car_front_m + obstacle.far_distance_m + config.pass_distance_m
                          for obstacle in obstacles])
        right_m = np.array([obstacle.right_m for obstacle in obstacles])[:, None]
        left_m = np.array([obstacle.left_m for obstacle in obstacles])[:, None]
        lowest, highest = _across_within(corner_s, corner_d, near_m[:, None, None], far_m[:, None, None])
        beside = np.isfinite(lowest)
        obstacle_room = np.maximum(lowest - left_m, right_m - highest) - config.obstacle_clearance_m
        cone_room = (np.minimum(surface_m - corner_d.max(axis=1), corner_d.min(axis=1) + surface_m)
                     - config.cone_clearance_m)
        room = np.where(beside, np.minimum(obstacle_room, cone_room), np.inf)
        worst_m = room.min(axis=1, initial=np.inf)
        worst_m = np.where(np.isfinite(worst_m), worst_m, 0.0)
        return worst_m >= -TOLERANCE_M, np.where(beside, np.maximum(room, 0.0), 0.0).sum(axis=1), worst_m

    def _cost(self, offsets: np.ndarray, curvatures: np.ndarray, previous_offsets: np.ndarray, usual_m: float,
              room_m: float) -> float:
        """
        :param offsets: A line's offsets at points SAMPLE_STEP_M apart.
        :param curvatures: Its curvatures there.
        :param previous_offsets: The previous step's line there.
        :param usual_m: The usual line in metres left of the lane centre.
        :param room_m: The room it leaves beyond the clearances beside the obstacles it passes, summed over the points.
        :return: The line's cost (see the *_weight settings).
        """
        config = self.config
        return SAMPLE_STEP_M * float(config.deviation_weight * np.sum((offsets - usual_m) ** 2)
                                     + config.smoothness_weight * np.sum(curvatures ** 2)
                                     + config.change_weight * np.sum((offsets - previous_offsets) ** 2)
                                     - config.clearance_weight * room_m)


def _across_within(corner_s: np.ndarray, corner_d: np.ndarray, near_m: float | np.ndarray,
                   far_m: float | np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The part of the car's body (a rectangle, so convex) that lies in a stretch of lane, at each point checked.

    :param corner_s: The body's corners' distances along the lane, shape (points, 4), in order around the body.
    :param corner_d: The corners' offsets.
    :param near_m: Where the stretch starts, in metres along the lane; an array of shape (stretches, 1, 1) checks
        several stretches at once.
    :param far_m: Where it ends, shaped like near_m.
    :return: The rightmost and leftmost offset of that part at each point (for each stretch); inf and -inf where none
        of the body is in the stretch. They are found among the corners in the stretch and where the body's edges
        cross its ends.
    """
    inside = (corner_s >= near_m) & (corner_s <= far_m)
    lowest = np.where(inside, corner_d, np.inf).min(axis=-1, initial=np.inf)
    highest = np.where(inside, corner_d, -np.inf).max(axis=-1, initial=-np.inf)
    next_s, next_d = corner_s[:, _NEXT_CORNER], corner_d[:, _NEXT_CORNER]
    with np.errstate(divide="ignore", invalid="ignore"):
        for end_m in (near_m, far_m):
            along = (end_m - corner_s) / (next_s - corner_s)
            crosses = (along >= 0.0) & (along <= 1.0)
            d = corner_d + along * (next_d - corner_d)
            lowest = np.minimum(lowest, np.where(crosses, d, np.inf).min(axis=-1, initial=np.inf))
            highest = np.maximum(highest, np.where(crosses, d, -np.inf).max(axis=-1, initial=-np.inf))
    return lowest, highest


def _leading(clear: np.ndarray) -> int:
    """
    :return: How many of the first entries are true, up to the first false one.
    """
    return int(np.argmin(np.append(clear, False)))


def _hold(start_m: float, end_m: float, offset_m: float, usual_m: float) -> list[PathSegment]:
    """
    :return: The segment holding offset_m from start_m to end_m, or none when that is the usual line or no stretch.
    """
    if end_m <= start_m or abs(offset_m - usual_m) <= TOLERANCE_M:
        return []
    return [PathSegment.hold(start_m, end_m, offset_m)]


def _move_from(start: LineState, target_m: float, by_m: float, curvature_room: float) -> PathSegment | None:
    """
    :param start: Where the line is at s = 0, turning across the lane.
    :param target_m: The offset to move to, parallel to the lane.
    :param by_m: Where the move should end, in metres along the lane, if it is gentle enough; 0.0 for none.
    :param curvature_room: Sharpest curvature in 1/m the move may have.
    :return: The move ending at by_m, or else the shortest of MOVE_STRETCHES that is gentle enough; None when none is.
    """
    shortest_m = max(move_length_m(target_m - start.offset_m, curvature_room), SAMPLE_STEP_M)
    for length_m in ([by_m] if by_m > 0.0 else []) + [shortest_m * stretch for stretch in MOVE_STRETCHES]:
        move = PathSegment.move(0.0, length_m, start, LineState(target_m))
        if move.peak_curvature_per_m() <= curvature_room + TOLERANCE_M:
            return move
    return None


def _moved_back(obstacle: LaneObstacle, driven_m: float) -> LaneObstacle:
    """
    :param obstacle: A remembered obstacle, with distances from the car's front.
    :param driven_m: Distance in metres driven since.
    :return: The obstacle with its distances from where the car's front is now.
    """
    return replace(obstacle, distance_m=obstacle.distance_m - driven_m,
                   far_distance_m=obstacle.far_distance_m - driven_m)


def _remember(remembered: list[LaneObstacle], obstacle: LaneObstacle) -> list[LaneObstacle]:
    """
    :param remembered: The remembered obstacles.
    :param obstacle: An obstacle seen at this step.
    :return: The remembered obstacles with it added, joined with any it overlaps (along and across the lane) into one
        that covers both, at most MAX_REMEMBERED of them.
    """
    return sorted(_joined(remembered, obstacle, 0.0, 0.0), key=lambda other: other.far_distance_m)[-MAX_REMEMBERED:]


def _joined(obstacles: list[LaneObstacle], obstacle: LaneObstacle, along_m: float,
            across_m: float) -> list[LaneObstacle]:
    """
    :param obstacles: Obstacles, none of them that close to another.
    :param obstacle: Another obstacle.
    :param along_m: Obstacles closer than this in metres along the lane, and across_m across it, are joined.
    :param across_m: See along_m.
    :return: The obstacles with it added, joined with any it is that close to into one that covers them all.
    """
    kept = []
    for other in obstacles:
        if (other.distance_m <= obstacle.far_distance_m + along_m
                and obstacle.distance_m <= other.far_distance_m + along_m
                and other.right_m <= obstacle.left_m + across_m and obstacle.right_m <= other.left_m + across_m):
            obstacle = LaneObstacle(min(obstacle.distance_m, other.distance_m),
                                    max(obstacle.far_distance_m, other.far_distance_m),
                                    min(obstacle.right_m, other.right_m), max(obstacle.left_m, other.left_m),
                                    max(obstacle.point_count, other.point_count))
        else:
            kept.append(other)
    kept.append(obstacle)
    return kept
