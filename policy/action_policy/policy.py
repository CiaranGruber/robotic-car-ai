"""
Policy.py

This file is responsible for taking in the observations and detected lanes and choosing the instructions for the
control module
"""

import math

from policy.action_policy.mpc import MpcConfig, MpcController
from policy.action_policy.reference import reference_from_cones, rotate_cones
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
        self.lane_offset_m = config.lane_offset_m
        """Where in the lane to drive, in metres from the centre line, positive to the left. Starts at
        mpc.lane_offset_m; a manoeuvre such as the hook turn may change it between steps. It is kept
        mpc.edge_clearance_m from both lane edges and shrinks on curves (followed_offset_m)."""
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

    def followed_offset_m(self, lane_width_m: float, path_curvature_per_m: float = 0.0) -> float:
        """The offset from the lane centre actually followed: lane_offset_m, limited so the followed line stays
        edge_clearance_m from both edges of a lane this wide, and shrunk in proportion to the path's curvature until
        it is nothing at offset_fade_curvature_per_m.

        :param lane_width_m: Width of the lane in metres.
        :param path_curvature_per_m: Curvature of the lane's centre line in 1/m.
        :return: The offset in metres, positive to the left; 0.0 when the lane is too narrow to move over at all.
        """
        config = self.config
        room = max(0.0, lane_width_m / 2.0 - config.edge_clearance_m)
        fade = max(0.0, 1.0 - abs(path_curvature_per_m) / config.offset_fade_curvature_per_m)
        return min(max(self.lane_offset_m, -room), room) * fade

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
        corrected = None if cones is None else rotate_cones(cones, math.radians(self.config.camera_yaw_offset_deg))
        state = reference_from_cones(corrected, self.config.lane_width_m)
        if cones is None or state is None:
            # No lane or a doubtful lane: stop. Task A7 decides how long to continue and how to slow down instead.
            self.previous_curvature_per_m = 0.0
            return [DriveCommand(0.0, 0.0)]
        offset = self.followed_offset_m(self.config.lane_width_m, state.path_curvature_per_m)
        solution = self.controller.solve(state, self.previous_curvature_per_m, cones.sensor_age, offset)
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
