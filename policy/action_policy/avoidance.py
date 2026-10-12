"""
avoidance.py

This file chooses where in the lane the MPC should drive to pass the obstacles inside it.

The MPC follows a line along the lane, normally its centre (MpcPolicy.lane_offset_m moves it). When an obstacle in the
lane is too close to that line, LaneAvoidance moves the line to the nearest place in the lane that keeps the car's sides
at least obstacle_clearance_m from every obstacle ahead and cone_clearance_m from the cones on the lane's edges. It
keeps that line until the car's rear has passed the obstacle by pass_distance_m, measured with the wheel speed, because
the lidar no longer sees the obstacle ahead once the car is beside it. Then the MPC returns to its usual line.

With no such place in the lane, or no lane or lidar, nothing changes: MpcPolicy still stops for an obstacle in the car's
way, so stopping stays the fallback when the car cannot drive around it, or cannot move over before it is within the
stopping distance.

It keeps state between policy steps, and its loops are bounded by the number of obstacles, so it is safe to run inside
the real-time policy step.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass

from policy.input_output import CarObservations
from policy.lane_detection.lanes import LaneDetection
from policy.obstacle_detection.obstacles import LaneObstacle

_LOGGER = logging.getLogger(__name__)


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
    """Smallest gap in metres between the car's side and an obstacle it passes. Must be at least 0.

    Half the car's width plus this must be at least obstacle_detection.corridor_half_width_m, or the line that passes
    the obstacle would still count as blocked, and the car would stop instead (MovementPolicy checks it).
    """
    pass_distance_m: float = 0.30
    """Distance in metres the car's rear drives past an obstacle's far side before the car returns to its usual line.
    Must be at least 0."""

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError(f"avoidance.enabled must be true or false, not {self.enabled}")
        if not (math.isfinite(self.car_width_m) and self.car_width_m > 0.0):
            raise ValueError(f"avoidance.car_width_m must be positive and finite, not {self.car_width_m}")
        for name in ("car_rear_m", "cone_radius_m", "cone_clearance_m", "obstacle_clearance_m", "pass_distance_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f"avoidance.{name} must be finite and at least 0, not {value}")


class LaneAvoidance:
    """Chooses the line across the lane for the MPC to follow past obstacles, remembering it until they are passed.

    Create one when the node starts and call step once per policy step.
    """

    def __init__(self, config: AvoidanceConfig, car_front_m: float):
        """
        :param config: The avoidance settings.
        :param car_front_m: Distance in metres from base_link's origin forwards to the car's front
            (obstacle_detection.car_front_m), to which the obstacle distances are measured.
        """
        self.config = config
        self.car_length_m = car_front_m + config.car_rear_m
        """Length of the car in metres."""
        self.reset()

    def reset(self):
        """Forgets the previous run. This is done on the first step after entering the publishing-policy state."""
        self.offset_m = None
        """The line the MPC follows to pass obstacles, in metres left of the lane centre, or None when the car keeps
        to its usual line."""
        self.hold_m = 0.0
        """Distance in metres still to drive before the car has passed the obstacles it moved over for."""
        self.reach_m = math.inf
        """Distance in metres still to drive before the car's front is beside the nearest obstacle it moved over for,
        or math.inf when there is none. From then on the line is kept until the obstacles are passed."""

    def step(self, observations: CarObservations, lane: LaneDetection | None, obstacles: list[LaneObstacle] | None,
             usual_offset_m: float) -> float | None:
        """Chooses the line for this policy step.

        :param observations: The observations at this step, for the wheel speed and the time since the last step.
        :param lane: The lane detected at this step, or None.
        :param obstacles: The objects inside the lane ahead at this step (detect_lane_obstacles), or None without a lane
            or fresh lidar.
        :param usual_offset_m: The line the MPC follows otherwise (MpcPolicy.lane_offset_m), in metres left of the
            lane centre.
        :return: The line to follow, in metres left of the lane centre, or None to keep the usual line.
        """
        if observations.policy.is_first_policy_step:
            self.reset()
        if not self.config.enabled:
            return None
        if not observations.policy.is_first_policy_step and observations.car.wheel_speed is not None:
            driven_m = observations.car.wheel_speed * observations.policy.dt
            self.hold_m -= driven_m
            self.reach_m -= driven_m
        config = self.config
        half_width_m = config.car_width_m / 2.0
        keep_out_m = half_width_m + config.obstacle_clearance_m
        # Lines across the lane that bring the car's side closer than obstacle_clearance_m to each obstacle ahead
        blocked = ([] if lane is None or not lane.has_lane or obstacles is None
                   else [(obstacle.right_m - keep_out_m, obstacle.left_m + keep_out_m) for obstacle in obstacles])
        in_the_way = [obstacle for obstacle, (low, high) in zip(obstacles or [], blocked)
                      if low < usual_offset_m < high]
        if in_the_way:
            # The car must still pass these, from its front to beyond its rear
            self.hold_m = max(self.hold_m, max(obstacle.far_distance_m for obstacle in in_the_way)
                              + self.car_length_m + config.pass_distance_m)
            self.reach_m = min(self.reach_m, min(obstacle.distance_m for obstacle in in_the_way))
        elif self.hold_m <= 0.0:
            self.reach_m = math.inf
            self._set(None)
            return None
        if self.reach_m <= 0.0:
            # Beside an obstacle it moved over for, which the lidar no longer sees ahead: keep the line until the rear
            # has passed it, rather than move across the lane into it. MpcPolicy stops if another obstacle is in the
            # way of the line.
            return self.offset_m
        line_m = _nearest_open(usual_offset_m if self.offset_m is None else self.offset_m,
                               lane.lane_width_m / 2.0 - config.cone_radius_m - config.cone_clearance_m - half_width_m,
                               blocked) if blocked else None
        # With no room, keep the line; MpcPolicy stops if an obstacle is in the car's way
        self._set(self.offset_m if line_m is None else line_m)
        return self.offset_m

    def _set(self, offset_m: float | None):
        """Changes the line, logging when the car starts or stops passing obstacles."""
        if (offset_m is None) != (self.offset_m is None):
            if offset_m is None:
                _LOGGER.info("Passed the obstacles in the lane: back to the usual line")
            else:
                _LOGGER.info("Obstacle in the lane: following the line %.2f m left of the lane centre", offset_m)
        self.offset_m = offset_m


def _nearest_open(target_m: float, reach_m: float, blocked: list[tuple[float, float]]) -> float | None:
    """
    :param target_m: The line wanted, in metres left of the lane centre.
    :param reach_m: Farthest the line may be from the lane centre on either side, in metres.
    :param blocked: Open intervals (low, high) of lines that are too close to an obstacle.
    :return: The line nearest target_m within reach_m of the centre and outside every blocked interval, or None when
        there is none.
    """
    if reach_m < 0.0:
        return None
    candidates = [min(max(target_m, -reach_m), reach_m)]
    candidates += [edge for interval in blocked for edge in interval if -reach_m <= edge <= reach_m]
    open_lines = [line for line in candidates if not any(low < line < high for low, high in blocked)]
    return min(open_lines, key=lambda line: abs(line - target_m), default=None)
