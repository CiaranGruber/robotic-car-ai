"""Stationary yaw compensation experiment; never requests driving."""
import math
from policy.input_output import default_actions


def validate_setup(mode, required, speed_test, gain, centre, limit, rate):
    if mode != "timer" or "imu_orientation" not in required or speed_test:
        raise ValueError("Pan test needs timer, required imu_orientation and speed_test_enabled=false")
    if not all(math.isfinite(v) for v in (gain, centre, limit, rate)):
        raise ValueError("Pan calibration must be finite")
    if gain == 0 or not 0 < limit <= 0.3 or abs(centre) + limit > 1 or not 0 < rate <= 0.5:
        raise ValueError("Set measured signed pan gain; limit (0,0.3], centre +/- limit within [-1,1], rate (0,0.5]")


def step(obs, memory, gain, centre, limit, rate):
    """Gain is normalized command per camera radian left, measured on this car.

    Requires a manually centred camera before each explicit policy start.
    IMU yaw is already relative to state entry in the body frame. Range violations
    latch the node into zero/hold through its existing exception handler.
    """
    orientation = obs.car.orientation
    if orientation is None or not math.isfinite(orientation.heading_angle):
        raise ValueError("Pan test has no valid relative IMU heading")
    yaw = orientation.heading_angle
    dt = obs.policy.dt
    if not math.isfinite(dt) or dt < 0 or dt > 0.2:
        raise ValueError("Pan test timing gap; restart explicitly")
    target = centre - gain * yaw
    if abs(target - centre) > limit:
        raise ValueError("Pan compensation range exceeded; hold and restart explicitly")
    previous = centre if obs.policy.is_first_policy_step else memory.data.pan_test_target
    if previous is None:
        raise ValueError("Pan test needs an explicit new run")
    command = previous + max(-rate * dt, min(rate * dt, target - previous))
    memory.set_pan_test_target(command)
    actions = default_actions()
    actions.camera_pan_action = command
    actions.debug1 = yaw
    actions.debug2 = command
    return actions
