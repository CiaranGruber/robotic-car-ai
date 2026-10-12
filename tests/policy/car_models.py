"""Measured responses of the real cars, for running the Stage 1 cases through the real control chain (Task A2).

dream-gym's car turns a normalised drive into a motor force and a normalised steering into a front wheel angle, which
is not how a real car responds to the drive and steering actions that CarController publishes. A CarModel holds one
car's measured response to those actions, and stage1_scenarios.py drives dream-gym so that it follows it:
- speed: first order towards the steady speed speed_gain_m_per_s * (drive - breakaway_drive), with one time constant
  while the motor pushes and another while the car coasts. The drive action is never negative, so the car never
  brakes; below COAST_STOP_SPEED_M_PER_S a coasting car stops.
- steering: the steering map measured for Car Control (docs/control/car-calibration.md), inverted, and limited to the
  tightest curvature the car reaches in each direction.

Run the cases with a car from the repository root with, for example:
    python3 tests/policy/stage1_scenarios.py --car car22a
"""
from __future__ import annotations

import math
from dataclasses import dataclass

COAST_STOP_SPEED_M_PER_S = 0.05
"""A coasting car below this speed in m/s stops within one simulation step (rolling resistance, not measured)."""


@dataclass(frozen=True)
class CarModel:
    """One car's measured response to the vehicle interface's drive and steering actions."""
    name: str
    """Short name used on the command line, such as car22a."""
    source: str
    """Where the values come from and which are not measured."""
    speed_gain_m_per_s: float
    """Steady speed per unit of drive above breakaway_drive, in m/s."""
    breakaway_drive: float
    """Drive action below which the car does not move."""
    drive_time_constant_s: float
    """Time constant of the speed while the drive pushes the car towards a higher steady speed, in seconds."""
    coast_time_constant_s: float
    """Time constant of the speed while the car slows down towards a lower steady speed or coasts, in seconds."""
    steering_centre_action: float
    """Steering action that drives straight."""
    steering_left_action_per_curvature: float
    """Steering action per 1/m of left (positive) curvature."""
    steering_right_action_per_curvature: float
    """Steering action per 1/m of right (negative) curvature."""
    max_left_curvature_per_m: float
    """Tightest left curvature the car reaches in 1/m, where its steering saturates."""
    max_right_curvature_per_m: float
    """Tightest right curvature the car reaches in 1/m, as a positive number."""

    def __post_init__(self):
        for name in ("speed_gain_m_per_s", "drive_time_constant_s", "coast_time_constant_s",
                     "max_left_curvature_per_m", "max_right_curvature_per_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"{self.name}: {name} must be positive and finite, not {value}")
        if not 0.0 <= self.breakaway_drive < 1.0:
            raise ValueError(f"{self.name}: breakaway_drive must be within [0, 1), not {self.breakaway_drive}")
        left, right = self.steering_left_action_per_curvature, self.steering_right_action_per_curvature
        if not (math.isfinite(left) and math.isfinite(right) and left * right > 0.0):
            raise ValueError(f"{self.name}: the steering slopes must be finite, non-zero and have the same sign")

    @property
    def max_curvature_per_m(self) -> float:
        """Tightest curvature the car reaches in both directions, in 1/m: the MPC's single curvature limit."""
        return min(self.max_left_curvature_per_m, self.max_right_curvature_per_m)

    def steady_speed_m_per_s(self, drive_action: float) -> float:
        """Speed the car settles at with a constant drive action, in m/s."""
        return self.speed_gain_m_per_s * max(0.0, drive_action - self.breakaway_drive)

    def acceleration_m_per_s2(self, drive_action: float, speed_m_per_s: float, step_s: float) -> float:
        """Forwards acceleration of the car in m/s^2 with this drive action at this speed.

        :param drive_action: The drive action applied.
        :param speed_m_per_s: The car's forwards speed in m/s.
        :param step_s: The simulation step in seconds, over which a coasting car below COAST_STOP_SPEED_M_PER_S
            stops.
        """
        steady = self.steady_speed_m_per_s(drive_action)
        if steady > speed_m_per_s:
            return (steady - speed_m_per_s) / self.drive_time_constant_s
        if steady == 0.0 and speed_m_per_s < COAST_STOP_SPEED_M_PER_S:
            return -speed_m_per_s / step_s
        return (steady - speed_m_per_s) / self.coast_time_constant_s

    def curvature_per_m(self, steering_action: float) -> float:
        """Curvature the car drives with this steering action in 1/m, positive turning left."""
        offset = steering_action - self.steering_centre_action
        curvature = offset / self.steering_left_action_per_curvature
        if curvature < 0.0:
            curvature = offset / self.steering_right_action_per_curvature
        return min(self.max_left_curvature_per_m, max(-self.max_right_curvature_per_m, curvature))


# Car 20's steering, from Chris's calibration on 2026-10-05 (docs/control/car-calibration.md and ControlConfig): the
# steering map's fits, a left turn saturating beyond -0.75 and a right turn reaching about 0.92 1/m at 1.0.
_CAR_20_STEERING = dict(steering_centre_action=0.02, steering_left_action_per_curvature=-1.587,
                        steering_right_action_per_curvature=-1.043, max_left_curvature_per_m=0.48,
                        max_right_curvature_per_m=0.92)

CAR_20 = CarModel(
    name="car20",
    source="Chris's calibration on 2026-10-05: speed map v = 9.18 * (drive - 0.475) and the steering map. The time "
           "constants are NOT measured on car 20; they are car 22's single time constant fit (car22b).",
    speed_gain_m_per_s=9.18, breakaway_drive=0.475, drive_time_constant_s=2.3, coast_time_constant_s=2.3,
    **_CAR_20_STEERING)

# Car 22 (10.43.254.22) speed, fitted to the wheel speed of one run on 2026-10-12 11:00 with car 20's ControlConfig:
# drive 0.40-0.61 for 2.4 s, then coasting for 1.3 s. The speed never settled, so the fit cannot separate the speed
# gain from the drive time constant: these two variants fit the run equally well (wheel speed RMS error 0.044 and
# 0.049 m/s) and bracket the car. Both agree that car 22 drives over 1.2 m/s with a drive of 0.42 and coasts with a
# time constant of about 2 s. Replace them with the open-loop speed calibration (docs/control/car-calibration.md).
CAR_22A = CarModel(
    name="car22a",
    source="Car 22 run on 2026-10-12, best fit: breakaway 0.32, slow drive response (its time constant reached the "
           "fit's 5 s bound). Steering is car 20's map, NOT measured on car 22.",
    speed_gain_m_per_s=24.7, breakaway_drive=0.32, drive_time_constant_s=5.0, coast_time_constant_s=2.05,
    **_CAR_20_STEERING)
CAR_22B = CarModel(
    name="car22b",
    source="Car 22 run on 2026-10-12, fit with one time constant: breakaway 0.17. Steering is car 20's map, NOT "
           "measured on car 22.",
    speed_gain_m_per_s=7.65, breakaway_drive=0.17, drive_time_constant_s=2.3, coast_time_constant_s=2.3,
    **_CAR_20_STEERING)

CARS = {car.name: car for car in (CAR_20, CAR_22A, CAR_22B)}
"""The car models by name, for --car."""
