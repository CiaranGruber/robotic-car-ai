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
    turn_slowdown_enabled: bool = False
    """Reduce requested speed as detected or planned absolute curvature increases."""
    turn_speed_m_per_s: float = 0.4
    """Speed cap on tight turns, m/s, positive. Never increases the normal target speed."""
    turn_start_curvature_per_m: float = 0.15
    """Begin reducing speed at this absolute curvature, 1/m, finite and nonnegative."""
    turn_full_curvature_per_m: float = 0.45
    """Reach the turn speed cap here, 1/m, strictly above turn_start_curvature_per_m."""
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

    lane_loss_timeout_s: float = 0.5
    """Seconds to keep driving after the last usable lane before stopping. Must be finite and at least 0.

    While the lane is lost, the car follows the curvatures planned at the last usable lane, at
    lane_loss_speed_fraction of the target speed. After this time it stops for the rest of the run. 0.0 stops at the
    first step without a usable lane. A starting value for Task A7; keep it short, because the plan only covers the
    horizon.
    """
    lane_loss_speed_fraction: float = 0.5
    """Fraction of target_speed_m_per_s to drive at while the lane is lost. Must be in [0, 1]."""
    lane_end_distance_m: float = 1.5
    """If the lane is lost while the farthest cone last seen is at most this far ahead in metres, the cone rows have
    treated as an inferred endpoint. Occlusion can produce the same observation; this is not a confirmed finish.
    Must be finite and at least 0.

    The camera loses the last cone pair before the car reaches it: a row needs two cones to be fitted, and the 80
    degree view cannot see a cone beside the lane closer than about 0.6 m ahead (more on wider lanes). Keep this
    below the distance the camera reliably sees cones at (Task A1), so a loss mid-lane is still unexpected.
    """
    lane_end_stop_past_m: float = 0.25
    """At a lane end, distance in metres past the farthest cone seen to stop at. Must be finite and at least 0.

    The Stage 1 stop zone is 0.0 to 0.5 m past the last cone pair (R5).
    """
    stop_distance_per_speed_s: float = 0.6
    """Distance in metres the car rolls on after a stop request, per m/s of speed it had. Must be finite and at
    least 0.

    A lane-end stop is requested this far before the stop point. 0.6 s matches the stand-in Car Control in the
    simulation. Measure it on the car: drive at a known speed, request a stop and divide the roll-out distance by the
    speed. CarController coasts for these stops (zero drive, no braking), so it is larger there. Obstacle stops brake
    and use obstacle_stop_distance_per_speed_s instead.
    """
    run_duration_s: float = 0.0
    """Seconds after entering the publishing-policy state before the car stops for the rest of the run.

    Must be finite and at least 0. 0.0 drives until the lane ends.
    """
    obstacle_stop_enabled: bool = True
    """Whether an obstacle in the car's way stops the car for the rest of the run (MpcPolicy.run_policy's
    obstacle_distance_m). Must be true or false. Without an obstacle distance (no lidar), nothing is checked."""
    obstacle_stop_margin_m: float = 0.2
    """Distance in metres the car should stay short of an obstacle in its way. Must be finite and at least 0.

    The stop is requested once the obstacle is within the stopping distance of the car's front:
    (obstacle_stop_distance_per_speed_s + lidar age) * speed + this margin, with the larger of target_speed_m_per_s
    and the car's speed. The car then stops about this far before the obstacle. MovementPolicy checks that the
    stopping distance at the target speed fits within obstacle_detection.corridor_length_m.
    """
    obstacle_stop_distance_per_speed_s: float = 0.6
    """Like stop_distance_per_speed_s, but for an obstacle stop, which asks to brake (DriveCommand.brake). Must be
    finite and at least 0.

    With control.brake_action the car brakes, so this is shorter than stop_distance_per_speed_s and the car drives
    nearer before stopping. Measure it with braking on. Without braking the car coasts, so MovementPolicy rejects a
    value below stop_distance_per_speed_s.
    """
    obstacle_confirm_steps: int = 2
    """Consecutive policy steps that must see an obstacle within the stopping distance before the car stops. Must be
    an integer of at least 1.

    The stop lasts for the rest of the run, so one step with stray returns should not cause it. Each extra step lets
    the car drive on for one more policy step, about 0.05 m at 0.5 m/s with cones at 10 Hz.
    """

    def __post_init__(self):
        for name in ("horizon_steps", "obstacle_confirm_steps"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"mpc.{name} must be an integer of at least 1")
        if not isinstance(self.turn_slowdown_enabled, bool):
            raise ValueError("mpc.turn_slowdown_enabled must be true or false")
        if not (math.isfinite(self.turn_speed_m_per_s) and self.turn_speed_m_per_s > 0
                and math.isfinite(self.turn_start_curvature_per_m) and self.turn_start_curvature_per_m >= 0
                and math.isfinite(self.turn_full_curvature_per_m)
                and self.turn_full_curvature_per_m > self.turn_start_curvature_per_m):
            raise ValueError("Invalid MPC turn speed or curvature thresholds")
        if not isinstance(self.obstacle_stop_enabled, bool):
            raise ValueError(f"mpc.obstacle_stop_enabled must be true or false, not {self.obstacle_stop_enabled}")
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
                     "max_right_curvature_per_m", "lane_loss_timeout_s", "run_duration_s", "lane_end_distance_m",
                     "lane_end_stop_past_m", "stop_distance_per_speed_s", "obstacle_stop_margin_m",
                     "obstacle_stop_distance_per_speed_s"):
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
    the car's right and left limits. Motion matrices use the current measured speed; curvature penalties
    are constant. Speed is held constant across the prediction horizon.
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
              speed_m_per_s: float | None = None, *, lateral_offset_m: float = 0.0) -> MpcSolution:
        """Chooses the curvatures that best follow the path from the car's state.

        :param state: Where the car was relative to the path when the observation was taken.
        :param previous_curvature_per_m: Curvature requested at the previous step in 1/m, which the car is assumed
            to have driven since the observation, and which the first curvature change is measured from.
        :param delay_s: Age of the observation in seconds. The state is predicted forwards by this much first.
        :param speed_m_per_s: Current finite, nonnegative wheel speed. None retains the nominal-speed model
            for callers without feedback. Zero is a real measurement, not missing data. Delay propagation assumes
            this speed and the previous curvature throughout the delay; it is not motion-history reconstruction.
        :param lateral_offset_m: Metres left of the path to follow, using its concentric line on curves.
        :return: The planned curvatures and whether they can be driven.
        """
        start = time.perf_counter()
        config = self.config
        steps = config.horizon_steps
        speed = config.target_speed_m_per_s if speed_m_per_s is None else float(speed_m_per_s)
        values = (state.lateral_error_m, state.heading_error_rad, state.path_curvature_per_m,
                  previous_curvature_per_m, delay_s, speed)
        if not all(math.isfinite(value) for value in values) or delay_s < 0.0 or speed < 0.0:
            raise ValueError("MPC requires finite inputs, nonnegative speed and nonnegative observation age")
        distance = speed * config.step_s
        # Exact zero-order-hold discretisation of the small-heading-error model:
        # e_next = e + d*heading + d^2/2*(curvature - path_curvature).
        free_response = np.zeros((2 * steps, 2))
        free_response[0::2, 0] = 1.0
        free_response[0::2, 1] = self._step_numbers * distance
        free_response[1::2, 1] = 1.0
        forced_response = np.empty((2 * steps, steps))
        forced_response[0::2] = self._lateral_effect * distance ** 2
        forced_response[1::2] = self._heading_effect * distance
        cost_matrix = np.vstack([self._state_weights[:, None] * forced_response, self._curvature_cost])
        if not math.isfinite(lateral_offset_m):
            raise ValueError("MPC lateral offset must be finite")
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
        # max_iter limits iterations, not wall-clock time; the node still enforces its calculation budget.
        try:
            result = lsq_linear(cost_matrix, targets,
                                bounds=self.curvature_bounds,
                                method="bvls", max_iter=4 * steps)
            plan = np.asarray(result.x, dtype=float)
            converged = (result.status > 0 and plan.shape == (steps,)
                         and bool(np.all(np.isfinite(plan)))
                         and bool(np.all(plan >= self.curvature_bounds[0]))
                         and bool(np.all(plan <= self.curvature_bounds[1])))
        except (ValueError, np.linalg.LinAlgError):
            converged = False
        # Failed plans must not leak arbitrary solver iterates to a caller.
        if not converged:
            plan = np.zeros(steps)
        return MpcSolution(plan, converged, time.perf_counter() - start)
