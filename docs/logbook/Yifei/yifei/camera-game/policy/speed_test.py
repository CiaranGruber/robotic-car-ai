"""Bounded straight-line speed experiment using the production PI controller."""
import math

from policy.input_output import default_actions

TARGET_SPEED = 0.4
DURATION_S = 5.0


def validate_setup(mode, required, control):
    """Reject setups that bypass the experiment's timer or wheel-speed feedback."""
    if mode != "timer" or "wheel_speed" not in required:
        raise ValueError("Speed test requires timer mode and required_sensors containing wheel_speed")
    if control.speed_feedback_min_m_per_s > TARGET_SPEED:
        raise ValueError("Speed test requires PI feedback at 0.4 m/s")


def finished(elapsed):
    """Invalid timing also requests a stop."""
    return not math.isfinite(elapsed) or elapsed < 0 or elapsed >= DURATION_S


def step(observations, controller):
    """Request straight steering and 0.4 m/s for at most five policy seconds."""
    actions = default_actions()
    speed = observations.car.wheel_speed
    if observations.policy.is_first_policy_step:
        controller.speed_controller.reset()
    if finished(observations.policy.policy_elapsed_s) or speed is None:
        controller.speed_controller.reset()
        return actions
    actions.drive_action = controller.speed_controller.step(TARGET_SPEED, speed, observations.policy.dt)
    actions.steering_action = controller.config.steering_centre_action
    actions.debug1 = float(speed)
    actions.debug2 = controller.speed_controller.integral
    return actions
