"""
Control.py

This file is responsible for receiving the car commands from the policy and converting them to the drive and steering
actions

TEMPORARY: CarControl is a conservative stand-in until Car Control's conversion is ready. Its steering direction,
steering angle and drive effort values are unmeasured starting guesses; check them on a stand before driving.
"""
import math
from dataclasses import dataclass

from policy.control.actions import DriveCommand, Instruction
from policy.input_output import CarActions, CarObservations, WheelSpeed, default_actions


@dataclass(frozen=True)
class ControlConfig:
    """Settings for converting drive commands into actions, loaded from the control parameters in
    config/ai4r_policy.yaml.

    These defaults are also the parameters' defaults in policy_node. Invalid settings raise a ValueError, which stops
    policy_node from starting.
    """
    wheelbase_m: float = 0.335
    """Distance between the front and rear axles in metres (approximate). Must be positive."""
    max_steering_angle_rad: float = 0.785
    """Front wheel angle in radians at full steering (steering_action of +-1), assumed linear in between.

    The default is the shared 45 degree default, likely larger than the real car's. Must be in (0, pi/2).
    """
    positive_steering_turns_left: bool = True
    """Whether a positive steering_action turns the car left. UNVERIFIED: check it on a stand first."""
    drive_effort_per_m_per_s: float = 0.1
    """Feedforward drive effort per m/s of requested speed. Must be at least 0."""
    speed_kp: float = 0.1
    """Drive effort per m/s of speed error (requested minus measured). Must be at least 0."""
    speed_ki: float = 0.1
    """Drive effort per metre of accumulated speed error. Must be at least 0."""
    max_drive_effort: float = 1
    """Largest drive effort ever requested, in (0, 1]. Drive is forwards only."""

    def __post_init__(self):
        if not isinstance(self.positive_steering_turns_left, bool):
            raise ValueError("control.positive_steering_turns_left must be true or false")
        if not (math.isfinite(self.wheelbase_m) and self.wheelbase_m > 0.0):
            raise ValueError(f"control.wheelbase_m must be positive and finite, not {self.wheelbase_m}")
        if not 0.0 < self.max_steering_angle_rad < math.pi / 2.0:
            raise ValueError(f"control.max_steering_angle_rad must be in (0, pi/2), not {self.max_steering_angle_rad}")
        for name in ("drive_effort_per_m_per_s", "speed_kp", "speed_ki"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f"control.{name} must be finite and at least 0, not {value}")
        if not 0.0 < self.max_drive_effort <= 1.0:
            raise ValueError(f"control.max_drive_effort must be in (0, 1], not {self.max_drive_effort}")


class CarControl:
    """Converts the policy's drive commands into drive and steering actions.

    Steering follows the bicycle model: wheel angle = atan(wheelbase * curvature). Drive is a feedforward plus
    proportional-integral (PI) speed controller on the measured wheel speed. It keeps its integral between policy
    steps, so create one when the node starts and reuse it for every step.
    """

    def __init__(self, config: ControlConfig):
        """
        :param config: The control settings.
        """
        self.config = config
        self.reset()

    def reset(self):
        """Clears the speed controller's integral."""
        self.speed_error_integral = 0.0
        """Accumulated speed error in metres."""

    def determine_car_actions(self, observations: CarObservations, instructions: list[Instruction]) -> CarActions:
        """Converts the instructions chosen by the policy into the car actions for this step

        :param observations: The observations taken by the car at the current policy step
        :param instructions: The instructions chosen by the policy. The last drive command is followed; other
            instructions are ignored for now.
        :return: The drive, steering and camera pan actions for the car to take
        """
        if observations.policy.is_first_policy_step:
            self.reset()
        commands = [instruction for instruction in instructions if isinstance(instruction, DriveCommand)]
        if not commands:
            self.reset()
            return default_actions()
        command = commands[-1]
        drive = self._drive_action(command.speed_m_per_s, observations.car.wheel_speed, observations.policy.dt)
        return CarActions(drive, self._steering_action(command.curvature_per_m), None, None, None)

    def _steering_action(self, curvature_per_m: float) -> float:
        """
        :param curvature_per_m: Requested curvature in 1/m, positive turning left.
        :return: The steering action in [-1, 1].
        """
        steering = math.atan(self.config.wheelbase_m * curvature_per_m) / self.config.max_steering_angle_rad
        if not self.config.positive_steering_turns_left:
            steering = -steering
        return min(max(steering, -1.0), 1.0)

    def _drive_action(self, speed_m_per_s: float, wheel_speed: WheelSpeed | None, dt: float) -> float:
        """
        :param speed_m_per_s: Requested speed in m/s.
        :param wheel_speed: Measured wheel speed in m/s, or None when it is missing.
        :param dt: Seconds since the previous policy step; 0.0 on the first step.
        :return: The drive effort in [0, max_drive_effort].
        """
        config = self.config
        if speed_m_per_s <= 0.0 or wheel_speed is None:
            # Stop, or no speed feedback: request no effort rather than drive without feedback
            self.reset()
            return 0.0
        error = speed_m_per_s - wheel_speed
        self.speed_error_integral += error * dt
        if config.speed_ki > 0.0:
            # Anti-windup: the integral alone can never ask for more than the effort limit
            limit = config.max_drive_effort / config.speed_ki
            self.speed_error_integral = min(max(self.speed_error_integral, -limit), limit)
        effort = (config.drive_effort_per_m_per_s * speed_m_per_s + config.speed_kp * error
                  + config.speed_ki * self.speed_error_integral)
        return min(max(effort, 0.0), config.max_drive_effort)
