"""
Policy.py

This file is responsible for taking in the observations and detected lanes and choosing the instructions for the
control module
"""

import math

from policy.action_policy.mpc import MpcConfig, MpcController
from policy.action_policy.reference import reference_from_cones
from policy.control.actions import DriveCommand, Instruction
from policy.input_output import CarObservations
from policy.lane_detection.lanes import LaneDetection


class MpcPolicy:
    """Chooses where to drive and how fast with model predictive control (MPC).

    It keeps state between policy steps, so create one when the node starts and reuse it for every step.
    """

    def __init__(self, config: MpcConfig):
        """
        :param config: The MPC settings.
        """
        self.config = config
        self.controller = MpcController(config)
        self.reset()

    def reset(self):
        """Forgets the previous run. This is done on the first step after entering the publishing-policy state."""
        self.previous_curvature_per_m = 0.0
        """Curvature requested at the previous step in 1/m."""
        self.previous_speed_m_per_s = 0.0
        """Speed requested at the previous step in m/s."""
        self.plan_curvatures_per_m = None
        """Curvatures in 1/m planned by the latest converged solve, followed while the lane is lost. None before
        the first one in this run."""
        self.distance_since_plan_m = 0.0
        """Estimated distance since the plan, using current wheel speed over each elapsed step when available.
        Otherwise uses the previous requested speed; this is approximate odometry, not measured pose."""
        self.plan_step_distance_m = 0.0
        """Spatial spacing of the cached plan, using the speed of its solve. Zero means no reusable moving plan."""
        self.farthest_cone_ahead_m = None
        """Distance in metres from the car to the farthest cone seen at the latest usable lane, less the distance
        driven since (estimated with _conservative_speed). None before the first usable lane in this run."""
        self.lane_state = None
        """Where the car was relative to the lane centre in this step's observation, or None when this step had no
        usable lane. It is found even while stopped, so the lane estimate can be checked with the car held still."""
        self.lane_lost_s = 0.0
        """Seconds since the last usable lane; 0.0 while the lane is usable."""
        self.stopped_for_lane_loss = False
        """True once the lane was lost for lane_loss_timeout_s. The car then stays stopped for the rest of the run."""
        self.solve_count = 0
        """Number of times the MPC problem was solved in this run."""
        self.fallback_count = 0
        """Number of failed solves in this run. The fallback is a latched zero command, not continued driving."""
        self.stopped_for_solver_failure = False
        """True after a failed solve; only the first step of a new explicitly started run resets it."""
        self.lane_lost_step_count = 0
        """Number of steps in this run driven along the plan because the lane was lost."""
        self.last_solve_time_s = 0.0
        """Calculation time of the latest solve in seconds."""

    def run_policy(self, observations: CarObservations, lanes: LaneDetection | None) -> list[Instruction]:
        """Chooses the instructions for the car to follow from the observations and detected lanes

        :param observations: The observations taken by the car at the current policy step
        :param lanes: The lanes detected by the lane detection module. Not used yet: the reference path comes from
            the cones until the lane detection output is agreed (Task 1.4).
        :return: The instructions for the control module to convert into car actions
        """
        if observations.policy.is_first_policy_step:
            self.reset()
        cones = observations.cones
        state = reference_from_cones(cones, self.config.lane_width_m)
        self.lane_state = state
        run_duration_s = self.config.run_duration_s
        if (self.stopped_for_lane_loss or self.stopped_for_solver_failure
                or 0.0 < run_duration_s <= observations.policy.policy_elapsed_s):
            return [self._command(0.0, 0.0)]
        measured_speed = observations.car.wheel_speed
        if measured_speed is not None and (not math.isfinite(measured_speed) or measured_speed < 0.0):
            raise ValueError("MPC wheel speed must be finite and nonnegative")
        if not math.isfinite(observations.policy.dt) or observations.policy.dt < 0.0:
            raise ValueError("MPC policy dt must be finite and nonnegative")
        if not observations.policy.is_first_policy_step:
            travelled_speed = self.previous_speed_m_per_s if measured_speed is None else measured_speed
            self.distance_since_plan_m += travelled_speed * observations.policy.dt
            if self.farthest_cone_ahead_m is not None:
                self.farthest_cone_ahead_m -= self._conservative_speed(observations) * observations.policy.dt
        if cones is None or state is None:
            # No lane or a doubtful lane
            return [self._command_without_lane(observations)]
        self.lane_lost_s = 0.0
        # The cones were seen sensor_age ago, so the car has driven on since
        self.farthest_cone_ahead_m = (max(cone.pos.x for cone in cones)
                                      - self._conservative_speed(observations) * cones.sensor_age)
        prediction_speed = self.config.target_speed_m_per_s if measured_speed is None else float(measured_speed)
        solution = self.controller.solve(state, self.previous_curvature_per_m, cones.sensor_age,
                                         speed_m_per_s=prediction_speed)
        self.solve_count += 1
        self.last_solve_time_s = solution.solve_time_s
        if solution.converged:
            curvature = float(solution.curvatures_per_m[0])
            self.plan_curvatures_per_m = solution.curvatures_per_m
            self.distance_since_plan_m = 0.0
            self.plan_step_distance_m = prediction_speed * self.config.step_s
        else:
            # Never continue at target speed on an unvalidated optimisation result.
            self.fallback_count += 1
            self.stopped_for_solver_failure = True
            self.plan_curvatures_per_m = None
            return [self._command(0.0, 0.0)]
        return [self._command(self.config.target_speed_m_per_s, curvature)]

    def _command_without_lane(self, observations: CarObservations) -> DriveCommand:
        """Chooses the command when the lane is missing or doubtful (the Task A7 rule).

        Before the first usable lane of the run, the car waits stopped. After it, the car follows the latest plan:
        - If the farthest cone last seen is within lane_end_distance_m, the cone rows have ended. The car keeps its
          speed and requests the stop stop_distance_per_speed_s * speed before lane_end_stop_past_m past that cone,
          so it rolls to a stop there.
        - Otherwise the lane was lost unexpectedly. The car drives at lane_loss_speed_fraction of the target speed for
          up to lane_loss_timeout_s, in case the lane comes back.
        Either way it stops early if the plan's remaining distance runs out, and then stops for the rest of the run.
        A plan solved at zero speed cannot be followed while moving.

        :param observations: The observations at this step.
        :return: The command for this step.
        """
        config = self.config
        if self.solve_count == 0:
            # No usable lane yet in this run
            return self._command(0.0, 0.0)
        self.lane_lost_s += observations.policy.dt
        lane_ended = (self.farthest_cone_ahead_m is not None
                      and self.farthest_cone_ahead_m <= config.lane_end_distance_m)
        if lane_ended:
            to_go_m = self.farthest_cone_ahead_m + config.lane_end_stop_past_m
            stop = to_go_m <= config.stop_distance_per_speed_s * self._conservative_speed(observations)
            speed = config.target_speed_m_per_s
        else:
            stop = self.lane_lost_s >= config.lane_loss_timeout_s
            speed = config.lane_loss_speed_fraction * config.target_speed_m_per_s
        if (stop or self.plan_curvatures_per_m is None or self.plan_step_distance_m <= 0.0
                or self.distance_since_plan_m >= len(self.plan_curvatures_per_m) * self.plan_step_distance_m):
            self.stopped_for_lane_loss = True
            return self._command(0.0, 0.0)
        self.lane_lost_step_count += 1
        # No extrapolation beyond the horizon by repeating the last curvature.
        step = int(self.distance_since_plan_m / self.plan_step_distance_m)
        return self._command(speed, float(self.plan_curvatures_per_m[step]))

    def _conservative_speed(self, observations: CarObservations) -> float:
        """
        :param observations: The observations at this step.
        :return: The car's speed in m/s for the lane-end distance: the larger of the measured wheel speed and the
            speed requested at the previous step. A missing or stuck wheel speed then overestimates the distance
            driven and the roll-out, so the car stops early rather than late.
        """
        wheel_speed = observations.car.wheel_speed
        return max(self.previous_speed_m_per_s, 0.0 if wheel_speed is None else float(wheel_speed))

    def _command(self, speed_m_per_s: float, curvature_per_m: float) -> DriveCommand:
        """Remembers the command for the next step.

        :param speed_m_per_s: Requested speed in m/s.
        :param curvature_per_m: Requested curvature in 1/m.
        :return: The drive command.
        """
        self.previous_speed_m_per_s = speed_m_per_s
        self.previous_curvature_per_m = curvature_per_m
        return DriveCommand(speed_m_per_s, curvature_per_m)
