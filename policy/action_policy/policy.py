"""
Policy.py

This file is responsible for taking in the observations and detected lanes and choosing the instructions for the
control module
"""

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
        self.solve_count = 0
        """Number of times the MPC problem was solved in this run."""
        self.fallback_count = 0
        """Number of steps in this run where the solver did not converge and the previous curvature was reused."""
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
        if cones is None or state is None:
            # No lane or a doubtful lane: stop. Task A7 decides how long to continue and how to slow down instead.
            self.previous_curvature_per_m = 0.0
            return [DriveCommand(0.0, 0.0)]
        solution = self.controller.solve(state, self.previous_curvature_per_m, cones.sensor_age)
        self.solve_count += 1
        self.last_solve_time_s = solution.solve_time_s
        if solution.converged:
            curvature = float(solution.curvatures_per_m[0])
        else:
            # Fallback: keep the previous curvature, and count it for the Task 1.1 metrics
            self.fallback_count += 1
            curvature = self.previous_curvature_per_m
        self.previous_curvature_per_m = curvature
        return [DriveCommand(self.config.target_speed_m_per_s, curvature)]
