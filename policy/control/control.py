"""
Control.py

This file is responsible for receiving the car commands from the policy and converting them to the drive and steering
actions.

A DriveCommand asks for a speed in m/s and a path curvature in 1/m. The car's actions are normalised efforts instead,
so this module converts them:
- Speed: feedforward from the measured speed map, plus a PI correction on the wheel speed (SpeedController).
- Curvature: a measured piecewise-linear steering map, with separate left and right slopes (steering_for_curvature).

The default settings are car 20's measured calibration from 5 Oct (see policy/control/README.md). Other cars need
their own values; docs/car-calibration.md describes how to measure them.
"""
import math
from dataclasses import dataclass

from policy.control.actions import DriveCommand, Instruction
from policy.input_output import CarActions, CarObservations, default_actions


@dataclass(frozen=True)
class ControlConfig:
    """Settings for the control module, loaded from the control parameters in config/ai4r_policy.yaml.

    These defaults are also the parameters' defaults in policy_node. Invalid settings raise a ValueError, which stops
    policy_node from starting.
    """
    speed_ff_offset: float = 0.48
    """Drive action at which the car just starts to move, from the speed map fit. Must be within [0, 1).

    The open-loop speed map is speed = speed_ff_gain * (drive - speed_ff_offset). It drifts upwards as the motor
    battery drains (car 8: about 0.62 when full, 0.70 when drained); the integral term absorbs that drift.
    """
    speed_ff_gain: float = 9.18
    """Slope of the speed map in m/s per unit of drive action. Must be positive."""
    speed_kp: float = 0.15
    """Proportional gain in drive action per m/s of speed error. Must be at least 0."""
    speed_ki: float = 0.04
    """Integral gain in drive action per metre of accumulated speed error. Must be at least 0; 0.0 gives FF + P."""
    speed_i_max: float = 0.2
    """Largest magnitude of the integral term in drive action. Must be at least 0."""
    speed_i_band: float = 0.2
    """The integral only accumulates while |speed error| is below this, in m/s. Must be positive.

    This stops the large error while accelerating from winding up an overshoot.
    """
    speed_feedback_min_m_per_s: float = 0.4
    """Feedback (P and I) is only used for target speeds at or above this, in m/s. Must be at least 0.

    Wheel speed comes from sparse encoder periods, so it lags by seconds at low speeds. Below this the drive is
    feedforward only, and car 20 may not start moving (its breakaway drive is about 0.52 to 0.53).
    """
    max_drive_action: float = 0.75
    """Upper limit of the drive action. Must be within (0, 1].

    On car 20, open-loop drive 0.75 passes 1.5 m/s in about 3 s.
    """
    steering_centre_action: float = 0.02
    """Steering action that drives straight. Must be within [-1, 1].

    It is measured with the vehicle interface's steering trim set to -0.18 on car 20; the trim resets to 0.0 when
    the car's main board restarts, so set it again before driving.
    """
    steering_left_action_per_curvature: float = -1.587
    """Steering action per 1/m of left (positive) curvature. Must be finite and non-zero.

    On car 20 positive steering turns RIGHT, so this is negative: the fit is curvature = -0.630 * steering + 0.008.
    """
    steering_right_action_per_curvature: float = -1.043
    """Steering action per 1/m of right (negative) curvature. Must have the same sign as the left value.

    Car 20's fit is curvature = -0.959 * steering + 0.038.
    """
    max_steering_action: float = 1.0
    """Largest steering action magnitude. Must be within (0, 1].

    Car 20's left turn saturates beyond about -0.75 (radius about 2.1 m); right reaches radius about 1.1 m at 1.0.
    """

    def __post_init__(self):
        for name in ("speed_ff_gain", "speed_i_band"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"control.{name} must be positive and finite, not {value}")
        for name in ("speed_kp", "speed_ki", "speed_i_max", "speed_feedback_min_m_per_s"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f"control.{name} must be finite and at least 0, not {value}")
        if not 0.0 <= self.speed_ff_offset < 1.0:
            raise ValueError(f"control.speed_ff_offset must be within [0, 1), not {self.speed_ff_offset}")
        for name in ("max_drive_action", "max_steering_action"):
            value = getattr(self, name)
            if not 0.0 < value <= 1.0:
                raise ValueError(f"control.{name} must be within (0, 1], not {value}")
        if not -1.0 <= self.steering_centre_action <= 1.0:
            raise ValueError(f"control.steering_centre_action must be within [-1, 1], not "
                             f"{self.steering_centre_action}")
        left, right = self.steering_left_action_per_curvature, self.steering_right_action_per_curvature
        if not (math.isfinite(left) and math.isfinite(right) and left * right > 0.0):
            raise ValueError("control.steering_left_action_per_curvature and steering_right_action_per_curvature "
                             "must be finite, non-zero and have the same sign")


def steering_for_curvature(curvature_per_m: float, config: ControlConfig) -> float:
    """Converts a path curvature into a steering action with the measured steering map.

    :param curvature_per_m: Requested curvature in 1/m; positive turns left.
    :param config: The control settings.
    :return: The steering action, limited to +-max_steering_action.
    """
    slope = (config.steering_left_action_per_curvature if curvature_per_m >= 0.0
             else config.steering_right_action_per_curvature)
    steering = config.steering_centre_action + slope * curvature_per_m
    return max(-config.max_steering_action, min(config.max_steering_action, steering))


class SpeedController:
    """Feedforward + PI wheel-speed controller.

    drive = speed_ff_offset + v_ref / speed_ff_gain + speed_kp * error + integral, with error = v_ref - wheel speed.
    It keeps the integral between policy steps, so reset it when a run starts.
    """

    def __init__(self, config: ControlConfig):
        """
        :param config: The control settings.
        """
        self.config = config
        self.reset()

    def reset(self):
        """Forgets the integral. This is done on the first step of a run and whenever the car is asked to stop."""
        self.integral = 0.0
        """Integral term in drive action."""

    def step(self, target_speed_m_per_s: float, wheel_speed_m_per_s: float | None, dt: float) -> float:
        """Calculates the drive action for one policy step.

        :param target_speed_m_per_s: Requested forwards speed in m/s. 0.0 or less gives zero drive.
        :param wheel_speed_m_per_s: Measured unsigned wheel speed in m/s, or None when it is missing or expired.
        :param dt: Seconds since the previous policy step; 0.0 on the first step.
        :return: The drive action within [0, max_drive_action]. It is zero when feedback is needed but the wheel
            speed is missing, because then neither the speed nor an overspeed can be checked.
        """
        config = self.config
        if not target_speed_m_per_s > 0.0:
            self.reset()
            return 0.0
        drive = config.speed_ff_offset + target_speed_m_per_s / config.speed_ff_gain
        if target_speed_m_per_s >= config.speed_feedback_min_m_per_s:
            if wheel_speed_m_per_s is None:
                return 0.0
            error = target_speed_m_per_s - wheel_speed_m_per_s
            unclipped = drive + config.speed_kp * error + self.integral
            # Anti-windup: do not integrate further into a limit
            pushing_high = unclipped >= config.max_drive_action and error > 0.0
            pushing_low = unclipped <= 0.0 and error < 0.0
            if dt > 0.0 and abs(error) < config.speed_i_band and not (pushing_high or pushing_low):
                self.integral = max(-config.speed_i_max,
                                    min(config.speed_i_max, self.integral + config.speed_ki * error * dt))
            drive += config.speed_kp * error + self.integral
        return max(0.0, min(config.max_drive_action, drive))


class CarController:
    """Converts the policy's instructions into the car's actions, keeping the speed controller's state.

    Create one when the node starts and call step once per policy step.
    """

    def __init__(self, config: ControlConfig):
        """
        :param config: The control settings.
        """
        self.config = config
        self.speed_controller = SpeedController(config)

    def step(self, observations: CarObservations, instructions: list[Instruction]) -> CarActions:
        """Converts the instructions chosen by the policy into the car actions for this step

        The last DriveCommand in the instructions is driven. Without one, the car stops with its steering centred.

        :param observations: The observations taken by the car at the current policy step
        :param instructions: The instructions chosen by the policy
        :return: The drive, steering and camera pan actions for the car to take
        """
        if observations.policy.is_first_policy_step:
            self.speed_controller.reset()
        actions = default_actions()
        commands = [instruction for instruction in instructions if isinstance(instruction, DriveCommand)]
        if not commands:
            self.speed_controller.reset()
            return actions
        command = commands[-1]
        actions.drive_action = self.speed_controller.step(
            command.speed_m_per_s, observations.car.wheel_speed, observations.policy.dt)
        actions.steering_action = steering_for_curvature(command.curvature_per_m, self.config)
        return actions
