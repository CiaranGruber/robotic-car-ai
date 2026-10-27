"""The simulated car: the measured response to drive and steering, the vehicle interface's limits, and its body.

The response is a CarModel from car_models.py (car 20's measured speed and steering maps by default), driven as a
kinematic car: curvature in, no tyre slip, which suits the lab speeds of about 0.5 m/s. On top of it, as the DREAM
vehicle interface does with config/traxxas_vehicle_interface.yaml:
- the applied drive and steering follow the commands at most drive_slew_rate_per_s and steering_slew_rate_per_s;
- a command older than policy_command_timeout_s is replaced by zeros.
A negative drive brakes at BRAKE_DECELERATION_M_PER_S2, as in intersection_simulation.py. That braking is NOT
measured.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import yaml

from tests.policy.car_models import CarModel
from tests.policy.stage1_scenarios import CAR_FRONT_M, CAR_REAR_M, CAR_WIDTH_M, WHEELBASE_M
from tests.policy.simulator.track import ROOT

BRAKE_DECELERATION_M_PER_S2 = 0.8
"""Deceleration in m/s^2 while the drive action is negative. Illustrative, NOT measured stopping evidence."""
FRONT_AXLE_M = 0.6 * WHEELBASE_M
"""Front axle, metres ahead of base_link (the centre of gravity), for drawing the wheels."""
REAR_AXLE_M = -0.4 * WHEELBASE_M
"""Rear axle, metres ahead of base_link."""
WHEEL_TRACK_M = CAR_WIDTH_M - 0.05
"""Distance between the left and right wheel centres in metres."""


def vehicle_settings() -> dict:
    """
    :return: The traxxas_vehicle_interface parameters from config/traxxas_vehicle_interface.yaml.
    """
    document = yaml.safe_load((ROOT / "config/traxxas_vehicle_interface.yaml").read_text())
    return document["/**/traxxas_vehicle_interface"]["ros__parameters"]


@dataclass
class CarState:
    """Where the car is and what it is doing, in world coordinates."""
    x: float
    y: float
    heading: float
    """Radians anticlockwise from +x."""
    speed_m_per_s: float = 0.0
    curvature_per_m: float = 0.0
    """Curvature the car is driving, 1/m, positive turning left."""
    drive_applied: float = 0.0
    """The drive action the vehicle interface applies after its slew limit."""
    steering_applied: float = 0.0
    """The steering action the vehicle interface applies after its slew limit."""
    distance_m: float = 0.0
    """Distance driven since the start, metres."""

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.heading


class SimulatedCar:
    """Moves the car by the commanded actions, as the vehicle interface and the measured car would."""

    body_front_m, body_rear_m, body_width_m = CAR_FRONT_M, CAR_REAR_M, CAR_WIDTH_M

    def __init__(self, model: CarModel, start: tuple[float, float, float], settings: dict | None = None):
        """
        :param model: The car's measured response.
        :param start: Starting world x, y and heading in radians.
        :param settings: The vehicle interface parameters; default config/traxxas_vehicle_interface.yaml.
        """
        settings = vehicle_settings() if settings is None else settings
        self.model = model
        self.drive_slew_per_s = float(settings["drive_slew_rate_per_s"])
        self.steering_slew_per_s = float(settings["steering_slew_rate_per_s"])
        self.command_timeout_s = float(settings["policy_command_timeout_s"])
        self.wheel_diameter_m = float(settings["wheel_diameter_m"])
        self.state = CarState(*start)
        self.command = (0.0, 0.0)
        self.command_age_s = math.inf

    def set_command(self, drive_action: float, steering_action: float):
        """Receives one drive and steering command, as drive_and_steer_set_point_normalized."""
        self.command = (max(-1.0, min(1.0, drive_action)), max(-1.0, min(1.0, steering_action)))
        self.command_age_s = 0.0

    def step(self, dt: float):
        """Moves the car on by dt seconds."""
        state = self.state
        self.command_age_s += dt
        drive, steering = self.command if self.command_age_s <= self.command_timeout_s else (0.0, 0.0)
        state.drive_applied += max(-self.drive_slew_per_s * dt, min(self.drive_slew_per_s * dt,
                                                                    drive - state.drive_applied))
        state.steering_applied += max(-self.steering_slew_per_s * dt, min(self.steering_slew_per_s * dt,
                                                                          steering - state.steering_applied))
        if state.drive_applied < 0.0:
            state.speed_m_per_s = max(0.0, state.speed_m_per_s - BRAKE_DECELERATION_M_PER_S2 * dt)
        else:
            acceleration = self.model.acceleration_m_per_s2(state.drive_applied, state.speed_m_per_s, dt)
            state.speed_m_per_s = max(0.0, state.speed_m_per_s + acceleration * dt)
        state.curvature_per_m = self.model.curvature_per_m(state.steering_applied)
        distance = state.speed_m_per_s * dt
        middle = state.heading + distance * state.curvature_per_m / 2.0
        state.x += distance * math.cos(middle)
        state.y += distance * math.sin(middle)
        state.heading = math.atan2(math.sin(state.heading + distance * state.curvature_per_m),
                                   math.cos(state.heading + distance * state.curvature_per_m))
        state.distance_m += distance

    @property
    def front_wheel_angle_rad(self) -> float:
        """The front wheels' steering angle for the driven curvature, positive left, for drawing."""
        return math.atan(WHEELBASE_M * self.state.curvature_per_m)
