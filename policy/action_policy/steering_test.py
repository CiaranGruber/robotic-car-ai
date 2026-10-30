"""
steering_test.py

This file holds the steering test: it holds a list of raw steering actions in turn, each for hold_s, to measure how far
the wheels turn and what curvature each action drives. It bypasses the steering map (control.steering_*), so the
actions are the ones published to the vehicle interface, which scales them into its steering_min_normalized ..
steering_max_normalized interval and adds the steering trim.

Two uses:
- On a stand (speed_m_per_s 0.0): the drive stays at zero and only the steering moves, to see whether the wheels
  still turn further between -0.75 and -1.0, or stop at a mechanical end first.
- On the ground (speed_m_per_s about 0.5): the car drives circles at the target speed (Car Control's speed
  controller), and debug2 gives the curvature from the IMU yaw rate over the wheel speed.

debug1 is the index of the action being held (0, 1, ...), and -1 before the first and once the list is done or the
test stopped; debug2 is the measured curvature in 1/m (positive left), and 0.0 without a fresh yaw rate or below
MIN_CURVATURE_SPEED_M_PER_S. tools/sysid_analyze.py reads these: it averages debug2 over the last 3 s of each hold.

The car stops (zero drive, centred steering) once the list is done, after max_distance_m of driving, or above
max_speed_m_per_s, and stays stopped until the next run.
"""
import logging
import math
from dataclasses import dataclass

from policy.control.control import ControlConfig, SpeedController
from policy.input_output import CarActions, CarObservations, default_actions

_LOGGER = logging.getLogger(__name__)

MIN_CURVATURE_SPEED_M_PER_S = 0.15
"""Below this wheel speed in m/s the yaw rate over the speed is too noisy to give a curvature."""


@dataclass(frozen=True)
class SteeringTestConfig:
    """Settings for the steering test, loaded from the steering_test parameters in config/ai4r_policy.yaml.

    These defaults are also the parameters' defaults in policy_node. Invalid settings raise a ValueError, which stops
    policy_node from starting.
    """
    steering_actions: str = "-0.5,-0.75,-1.0"
    """Raw steering actions to hold in turn, comma separated, each within [-1, 1]. On car 20 negative turns LEFT."""
    hold_s: float = 6.0
    """Seconds each action is held. Must be positive. On the ground the last 3 s give the curvature, so keep it at
    least 5 s."""
    speed_m_per_s: float = 0.0
    """Speed to drive at in m/s, at least 0. 0.0 keeps the drive at zero (stand test)."""
    max_distance_m: float = 15.0
    """The test stops after driving this far in metres, measured with the wheel speed. Must be positive."""
    max_speed_m_per_s: float = 1.0
    """The test stops if the wheel speed exceeds this in m/s. Must be positive."""

    def __post_init__(self):
        actions = self.actions
        if not actions or not all(math.isfinite(a) and -1.0 <= a <= 1.0 for a in actions):
            raise ValueError(f"steering_test.steering_actions must be comma-separated numbers within [-1, 1], not "
                             f"{self.steering_actions!r}")
        for name in ("hold_s", "max_distance_m", "max_speed_m_per_s"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"steering_test.{name} must be positive and finite, not {value}")
        if not (math.isfinite(self.speed_m_per_s) and 0.0 <= self.speed_m_per_s < self.max_speed_m_per_s):
            raise ValueError(f"steering_test.speed_m_per_s must be at least 0 and below max_speed_m_per_s, not "
                             f"{self.speed_m_per_s}")

    @property
    def actions(self) -> list[float]:
        """The steering actions to hold, in order."""
        try:
            return [float(text) for text in self.steering_actions.split(",") if text.strip()]
        except ValueError:
            return []


class SteeringTest:
    """Holds each steering action of the list in turn, keeping the run's progress between policy steps.

    Create one when the node starts and call step once per policy step.
    """

    def __init__(self, config: SteeringTestConfig, control_config: ControlConfig):
        """
        :param config: The steering test settings.
        :param control_config: The control settings, for the speed controller.
        """
        self.config = config
        self.actions = config.actions
        self.speed_controller = SpeedController(control_config)
        self.reset()

    def reset(self):
        """Starts the list again. This is done on the first step of a run."""
        self.speed_controller.reset()
        self.distance_m = 0.0
        """Distance in metres driven in this run."""
        self.stop_reason: str | None = None
        """Why the test stopped early, or None."""

    def step(self, observations: CarObservations) -> CarActions:
        """
        :param observations: The observations at this step.
        :return: The actions: the held steering action and the drive for the target speed, or zero drive and centred
            steering once done or stopped.
        """
        policy = observations.policy
        if policy.is_first_policy_step:
            self.reset()
        speed = observations.car.wheel_speed
        if speed is not None:
            self.distance_m += float(speed) * policy.dt
            if self.stop_reason is None and speed > self.config.max_speed_m_per_s:
                self._stop(f"wheel speed {float(speed):.2f} m/s above max_speed_m_per_s")
        if self.stop_reason is None and self.distance_m > self.config.max_distance_m:
            self._stop(f"drove {self.distance_m:.1f} m, more than max_distance_m")
        index = int(policy.policy_elapsed_s // self.config.hold_s)
        actions = default_actions()
        if self.stop_reason is not None or index >= len(self.actions):
            self.speed_controller.reset()
            actions.debug1 = -1.0
            actions.debug2 = 0.0
            return actions
        actions.steering_action = self.actions[index]
        actions.drive_action = self.speed_controller.step(self.config.speed_m_per_s, speed, policy.dt)
        actions.debug1 = float(index)
        actions.debug2 = self._curvature(observations)
        return actions

    def _stop(self, reason: str):
        self.stop_reason = reason
        _LOGGER.warning("Steering test stopped: %s", reason)

    @staticmethod
    def _curvature(observations: CarObservations) -> float:
        """
        :return: The measured curvature in 1/m (positive left): the IMU yaw rate over the wheel speed, or 0.0 without
            a yaw rate or below MIN_CURVATURE_SPEED_M_PER_S.
        """
        rates, speed = observations.car.ang_velocity, observations.car.wheel_speed
        if rates is None or speed is None or speed < MIN_CURVATURE_SPEED_M_PER_S:
            return 0.0
        return float(rates.yaw) / float(speed)
