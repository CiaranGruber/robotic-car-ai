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
    target_speed_m_per_s: float = 0.5
    """Speed to drive at in m/s, which the prediction also assumes. Must be positive.

    The default is the speed Car Control's speed controller was checked at on car 20; below 0.4 m/s it may not start
    the car (docs/mpc-lab-calibration.md).
    """
    horizon_steps: int = 12
    """Number of predicted steps. Must be an integer of at least 1."""
    step_s: float = 0.1
    """Duration of one predicted step in seconds. Must be positive.

    The predicted distance (target_speed_m_per_s * step_s * horizon_steps) should stay within the camera's range,
    because the path is unknown beyond it.
    """
    max_curvature_per_m: float = 0.48
    """Largest curvature the car can drive to the left in 1/m, and to the right too unless max_right_curvature_per_m
    is set. Must be positive.

    The default is car 20's measured left limit, a 2.1 m turning circle radius (docs/mpc-lab-calibration.md).
    Re-measure it for another car.
    """
    max_right_curvature_per_m: float = 0.92
    """Largest curvature the car can drive to the right in 1/m, as a positive number. 0.0 uses max_curvature_per_m.
    Must be finite and at least 0.

    The default is car 20's measured right limit: it turns right about twice as sharply as left. With the left limit
    both ways, the MPC could not plan the 1.25-1.5 m radius right curves car 20 can drive, and the hook turn is a
    right turn (docs/mpc-lab-calibration.md).
    """
    lateral_error_weight: float = 10.0
    """Cost per m^2 of distance from the path at each predicted step. Must be at least 0."""
    heading_error_weight: float = 1.0
    """Cost per rad^2 of heading error relative to the path at each predicted step. Must be at least 0.

    With car 20's limits, 1.0 kept every simulated curve of 1.5-3 m radius when Car Control compensates the steering
    trim; 3.0 lost S-bends and straight lanes, and 5.0, the car 20 test run's value, lost more curves
    (docs/mpc-lab-calibration.md).
    """
    curvature_weight: float = 0.1
    """Cost per (1/m)^2 of curvature beyond the path's own curvature at each predicted step.

    Must be positive, which keeps the best curvatures unique.
    """
    curvature_change_weight: float = 2.0
    """Cost per (1/m)^2 of curvature change between consecutive steps, for smooth steering. Must be at least 0.

    2.0 steers about a quarter to a third less than 1.0 with the same lane keeping on simulated curves, and kept more
    simulated straight lanes; 3.0 drifted further from the centre on straights and on the wet-floor case
    (docs/mpc-lab-calibration.md)."""
    lane_width_m: float = 0.8
    """Distance between the lane boundaries in metres, used when only one boundary is visible. Must be positive.

    The default is the lab tracks' measured width, 0.73-0.85 m.
    """
    lane_offset_m: float = 0.0
    """Where in the lane to drive, in metres from the centre line, positive to the left. Must be finite.

    The hook turn needs the car to keep to one side of the lane before it turns. The MPC follows the line this far
    from the centre, kept at least edge_clearance_m from both edges, so a lane narrower than the car plus that
    clearance is still driven along its centre. On curves the offset shrinks (offset_fade_curvature_per_m).
    MpcPolicy.lane_offset_m can change it between policy steps.
    """
    edge_clearance_m: float = 0.30
    """Smallest distance in metres from the followed line to either lane edge when lane_offset_m moves it. Must be
    positive. The default is half the car's 0.25 m width, a 0.05 m cone radius and 0.125 m for tracking error: with
    0.25 the car touched cones on simulated curves (docs/mpc-lab-calibration.md). It leaves 0.1 m of offset in a
    0.8 m lane."""
    offset_fade_curvature_per_m: float = 0.3
    """Path curvature in 1/m at which the lane offset has shrunk to nothing; it shrinks in proportion below that.
    Must be positive. The default fades it out on curves tighter than about a 3.3 m radius, where the car's own
    tracking error uses up the room beside it."""
    camera_yaw_offset_deg: float = 0.0
    """How far left of the car's forwards direction the camera points, in degrees, beyond what base_link assumes.
    Must be finite.

    The cone positions are rotated back by this much before the lane is fitted. Without it, a camera that points
    left makes a straight lane look turned to the right, and the MPC steers right to follow it. Ciaran's recordings
    show 6-10 degrees (docs/mpc-lab-calibration.md), but the camera pan was never commanded in them; the policy
    now holds the pan at zero, which may remove it. So the default is 0.0 until it is measured again with the pan
    held: put a cone 2 m straight ahead and read its detected sideways position.
    """

    def __post_init__(self):
        if isinstance(self.horizon_steps, bool) or not isinstance(self.horizon_steps, int) or self.horizon_steps < 1:
            raise ValueError("mpc.horizon_steps must be an integer of at least 1")
        for name in ("target_speed_m_per_s", "step_s", "max_curvature_per_m", "curvature_weight", "lane_width_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"mpc.{name} must be positive and finite, not {value}")
        for name in ("camera_yaw_offset_deg", "lane_offset_m"):
            if not math.isfinite(getattr(self, name)):
                raise ValueError(f"mpc.{name} must be finite, not {getattr(self, name)}")
        for name in ("edge_clearance_m", "offset_fade_curvature_per_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"mpc.{name} must be positive and finite, not {value}")
        for name in ("lateral_error_weight", "heading_error_weight", "curvature_change_weight",
                     "max_right_curvature_per_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f"mpc.{name} must be finite and at least 0, not {value}")
        if not 0.0 <= self.lane_loss_speed_fraction <= 1.0:
            raise ValueError(f"mpc.lane_loss_speed_fraction must be in [0, 1], not {self.lane_loss_speed_fraction}")


@dataclass(frozen=True)
class PathTrackingState:
    """Where the car is relative to the reference path when the observation was taken."""
    lateral_error_m: float
    """Signed distance from the path to the car in metres; positive when the car is left of the path."""
    heading_error_rad: float
    """Car heading minus path heading in radians; positive when the car points left of the path."""
    path_curvature_per_m: float
    """Curvature of the path near the car in 1/m, positive turning left. It is assumed constant over the horizon."""


def offset_state(state: PathTrackingState, offset_m: float) -> PathTrackingState:
    """Where the car is relative to the line offset_m to the left of the path (right when negative).

    On a curve the offset line is concentric with the path: inside the curve it turns more sharply. The heading
    error is unchanged, because concentric lines are parallel at the points beside each other.

    :param state: Where the car is relative to the path.
    :param offset_m: Distance of the line from the path in metres, positive to the left. On a curve it must stay on
        the path's side of the turning centre.
    :return: Where the car is relative to the offset line.
    """
    remaining = 1.0 - state.path_curvature_per_m * offset_m
    if remaining <= 0.0:
        raise ValueError(f"an offset of {offset_m} m is past the turning centre of a {state.path_curvature_per_m} "
                         f"1/m path")
    return PathTrackingState(state.lateral_error_m - offset_m, state.heading_error_rad,
                             state.path_curvature_per_m / remaining)


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
    the car's right and left limits. M depends only on the settings, so it is built once; c depends on the car's
    state.
    """

    def __init__(self, config: MpcConfig):
        """
        :param config: The MPC settings.
        """
        self.config = config
        steps = config.horizon_steps
        self._step_numbers = np.arange(1, steps + 1)
        separation = np.arange(steps)[:, None] - np.arange(steps)[None, :]
        self._heading_effect = (separation >= 0).astype(float)
        self._lateral_effect = np.where(separation >= 0, separation + 0.5, 0.0)
        self._state_weights = np.sqrt(np.tile([config.lateral_error_weight, config.heading_error_weight], steps))
        # Row k gives curvature k minus curvature k - 1; row 0 is compared with the previous curvature in c
        differences = np.eye(steps) - np.eye(steps, k=-1)
        self._curvature_cost = np.vstack([
            math.sqrt(config.curvature_weight) * np.eye(steps),
            math.sqrt(config.curvature_change_weight) * differences,
        ])

    @property
    def curvature_bounds(self) -> tuple[float, float]:
        """The most right (negative) and left curvature the MPC may plan, in 1/m."""
        config = self.config
        right = config.max_right_curvature_per_m or config.max_curvature_per_m
        return -right, config.max_curvature_per_m

    def solve(self, state: PathTrackingState, previous_curvature_per_m: float, delay_s: float,
              lateral_offset_m: float = 0.0) -> MpcSolution:
        """Chooses the curvatures that best follow the path from the car's state.

        :param state: Where the car was relative to the path when the observation was taken.
        :param previous_curvature_per_m: Curvature requested at the previous step in 1/m, which the car is assumed
            to have driven since the observation, and which the first curvature change is measured from.
        :param delay_s: Age of the observation in seconds. The state is predicted forwards by this much first.
        :param lateral_offset_m: Follow the line this far left of the path (right when negative) instead of the path.
            On a curve it is the concentric line, so its curvature differs from the path's.
        :return: The planned curvatures and whether they can be driven.
        """
        start = time.perf_counter()
        config = self.config
        steps = config.horizon_steps
        if lateral_offset_m:
            state = offset_state(state, lateral_offset_m)
        path_curvature = state.path_curvature_per_m
        # Predict the current state from the delayed observation
        delay_distance = speed * delay_s
        turn = previous_curvature_per_m - path_curvature
        initial_state = np.array([
            state.lateral_error_m + delay_distance * state.heading_error_rad + delay_distance ** 2 / 2.0 * turn,
            state.heading_error_rad + delay_distance * turn,
        ])
        path_curvatures = np.full(steps, path_curvature)
        previous = np.zeros(steps)
        previous[0] = previous_curvature_per_m
        targets = np.concatenate([
            -self._state_weights * (free_response @ initial_state - forced_response @ path_curvatures),
            math.sqrt(config.curvature_weight) * path_curvatures,
            math.sqrt(config.curvature_change_weight) * previous,
        ])
        # Bounded-variable least squares solves this small problem exactly; max_iter bounds its calculation time
        result = lsq_linear(self._cost_matrix, targets, bounds=self.curvature_bounds, method="bvls",
                            max_iter=4 * steps)
        converged = result.status > 0 and bool(np.all(np.isfinite(result.x)))
        return MpcSolution(result.x, converged, time.perf_counter() - start)
