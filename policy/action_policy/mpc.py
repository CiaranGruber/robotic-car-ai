"""
mpc.py

This file holds the model predictive control (MPC) used to follow a reference path, such as the lane centre.

At each policy step, the controller predicts how the car moves relative to the path over a short horizon and
chooses the curvature for each predicted step that keeps the car close to the path while steering smoothly.
Only the first curvature is driven; the problem is solved again at the next step with new observations.

The prediction model is in the path's frame, linearised for small heading errors, at a constant speed v:
    d(lateral_error)/dt = v * heading_error
    d(heading_error)/dt = v * (curvature - path_curvature)
"""
import math
import time
from dataclasses import dataclass

import numpy as np
from scipy.optimize import lsq_linear


@dataclass(frozen=True)
class MpcConfig:
    """Settings for the MPC, loaded from the mpc parameters in config/ai4r_policy.yaml.

    These defaults are also the parameters' defaults in policy_node. They follow the shared defaults in
    docs/movement-policy-tasks.md where one exists, and are starting guesses otherwise. Invalid settings raise a
    ValueError, which stops policy_node from starting.
    """
    target_speed_m_per_s: float = 1.0
    """Speed to drive at in m/s, which the prediction also assumes. Must be positive."""
    horizon_steps: int = 12
    """Number of predicted steps. Must be an integer of at least 1."""
    step_s: float = 0.1
    """Duration of one predicted step in seconds. Must be positive.

    The predicted distance (target_speed_m_per_s * step_s * horizon_steps) should stay within the camera's range,
    because the path is unknown beyond it.
    """
    max_curvature_per_m: float = 3.0
    """Largest curvature the car can drive in 1/m, in either direction. Must be positive.

    The default is tan(45 degrees) / 0.335 m, the shared steering default with an approximate wheelbase. It is
    likely larger than the real car's; replace it with the control module's measured value.
    """
    lateral_error_weight: float = 10.0
    """Cost per m^2 of distance from the path at each predicted step. Must be at least 0."""
    heading_error_weight: float = 1.0
    """Cost per rad^2 of heading error relative to the path at each predicted step. Must be at least 0."""
    curvature_weight: float = 0.1
    """Cost per (1/m)^2 of curvature beyond the path's own curvature at each predicted step.

    Must be positive, which keeps the best curvatures unique.
    """
    curvature_change_weight: float = 1.0
    """Cost per (1/m)^2 of curvature change between consecutive steps, for smooth steering. Must be at least 0."""
    lane_width_m: float = 1.0
    """Distance between the lane boundaries in metres, used when only one boundary is visible. Must be positive."""

    def __post_init__(self):
        if isinstance(self.horizon_steps, bool) or not isinstance(self.horizon_steps, int) or self.horizon_steps < 1:
            raise ValueError("mpc.horizon_steps must be an integer of at least 1")
        for name in ("target_speed_m_per_s", "step_s", "max_curvature_per_m", "curvature_weight", "lane_width_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"mpc.{name} must be positive and finite, not {value}")
        for name in ("lateral_error_weight", "heading_error_weight", "curvature_change_weight"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f"mpc.{name} must be finite and at least 0, not {value}")


@dataclass(frozen=True)
class PathTrackingState:
    """Where the car is relative to the reference path when the observation was taken."""
    lateral_error_m: float
    """Signed distance from the path to the car in metres; positive when the car is left of the path."""
    heading_error_rad: float
    """Car heading minus path heading in radians; positive when the car points left of the path."""
    path_curvature_per_m: float
    """Curvature of the path near the car in 1/m, positive turning left. It is assumed constant over the horizon."""


@dataclass(frozen=True)
class MpcSolution:
    """The result of solving the MPC problem for one policy step."""
    curvatures_per_m: np.ndarray
    """Planned curvature in 1/m for each predicted step. Only the first is driven."""
    converged: bool
    """False when the solver stopped before finding the best curvatures; they must not be driven then."""
    solve_time_s: float
    """Calculation time in seconds, for the time budget metrics."""


class MpcController:
    """Solves the MPC problem for one policy step.

    The problem is a least-squares cost ||M u - c||^2 over the planned curvatures u, with each curvature limited to
    +-max_curvature_per_m. M depends only on the settings, so it is built once; c depends on the car's state.
    """

    def __init__(self, config: MpcConfig):
        """
        :param config: The MPC settings.
        """
        self.config = config
        steps = config.horizon_steps
        step_distance = config.target_speed_m_per_s * config.step_s
        # One predicted step of the model, holding the curvature constant during the step
        step_matrix = np.array([[1.0, step_distance], [0.0, 1.0]])
        curvature_effect = np.array([step_distance ** 2 / 2.0, step_distance])
        # The predicted states [lateral_error, heading_error] after steps 1..n, stacked, are:
        #   free_response @ initial_state + forced_response @ (curvatures - path_curvature)
        self._free_response = np.zeros((2 * steps, 2))
        self._forced_response = np.zeros((2 * steps, steps))
        for k in range(steps):
            self._free_response[2 * k:2 * k + 2] = np.linalg.matrix_power(step_matrix, k + 1)
            for j in range(k + 1):
                self._forced_response[2 * k:2 * k + 2, j] = (
                    np.linalg.matrix_power(step_matrix, k - j) @ curvature_effect)
        self._state_weights = np.sqrt(np.tile([config.lateral_error_weight, config.heading_error_weight], steps))
        # Row k gives curvature k minus curvature k - 1; row 0 is compared with the previous curvature in c
        differences = np.eye(steps) - np.eye(steps, k=-1)
        self._cost_matrix = np.vstack([
            self._state_weights[:, None] * self._forced_response,
            math.sqrt(config.curvature_weight) * np.eye(steps),
            math.sqrt(config.curvature_change_weight) * differences,
        ])

    def solve(self, state: PathTrackingState, previous_curvature_per_m: float, delay_s: float) -> MpcSolution:
        """Chooses the curvatures that best follow the path from the car's state.

        :param state: Where the car was relative to the path when the observation was taken.
        :param previous_curvature_per_m: Curvature requested at the previous step in 1/m, which the car is assumed
            to have driven since the observation, and which the first curvature change is measured from.
        :param delay_s: Age of the observation in seconds. The state is predicted forwards by this much first.
        :return: The planned curvatures and whether they can be driven.
        """
        start = time.perf_counter()
        config = self.config
        steps = config.horizon_steps
        path_curvature = state.path_curvature_per_m
        # Predict the current state from the delayed observation
        delay_distance = config.target_speed_m_per_s * delay_s
        turn = previous_curvature_per_m - path_curvature
        initial_state = np.array([
            state.lateral_error_m + delay_distance * state.heading_error_rad + delay_distance ** 2 / 2.0 * turn,
            state.heading_error_rad + delay_distance * turn,
        ])
        path_curvatures = np.full(steps, path_curvature)
        previous = np.zeros(steps)
        previous[0] = previous_curvature_per_m
        targets = np.concatenate([
            -self._state_weights * (self._free_response @ initial_state - self._forced_response @ path_curvatures),
            math.sqrt(config.curvature_weight) * path_curvatures,
            math.sqrt(config.curvature_change_weight) * previous,
        ])
        # Bounded-variable least squares solves this small problem exactly; max_iter bounds its calculation time
        result = lsq_linear(self._cost_matrix, targets,
                            bounds=(-config.max_curvature_per_m, config.max_curvature_per_m),
                            method="bvls", max_iter=4 * steps)
        converged = result.status > 0 and bool(np.all(np.isfinite(result.x)))
        return MpcSolution(result.x, converged, time.perf_counter() - start)
