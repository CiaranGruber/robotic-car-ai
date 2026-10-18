from __future__ import annotations

import dataclasses
import math

from policy.action_policy.avoidance import AvoidanceConfig, LaneAvoidance
from policy.action_policy.lane_path import LanePath
from policy.action_policy.mpc import MpcConfig
from policy.action_policy.open_space import OpenSpaceConfig, OpenSpacePolicy
from policy.action_policy.policy import MpcPolicy
from policy.action_policy.reference import rotate_cones
from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones, remove_implausible_cones
from policy.control.control import CarController, ControlConfig
from policy.input_output import CarActions, CarObservations, ConeBatch
from policy.lane_detection.lane_detection import LaneDetectionConfig, LaneDetector
from policy.lane_detection.lanes import LaneDetection
from policy.memory import CarMemory
from policy.obstacle_detection.obstacle_detection import (ObstacleDetectionConfig, detect_lane_obstacles,
                                                          detect_obstacles, nearest_in_band)
from policy.obstacle_detection.obstacles import LaneObstacle, ObstacleDetection

ACTION_POLICIES = ("mpc", "open_space")
"""Valid names for the action_policy parameter in config/ai4r_policy.yaml."""


class MovementPolicy:
    """Runs the movement policy for the car, keeping the stages' state between policy steps.

    policy_node creates one when it starts and calls step once per policy step.
    """

    def __init__(self, mpc_config: MpcConfig, cone_filter_config: ConeFilterConfig, control_config: ControlConfig,
                 open_space_config: OpenSpaceConfig = OpenSpaceConfig(), action_policy: str = "mpc",
                 lane_detection_config: LaneDetectionConfig = LaneDetectionConfig(),
                 obstacle_detection_config: ObstacleDetectionConfig = ObstacleDetectionConfig(),
                 avoidance_config: AvoidanceConfig = AvoidanceConfig()):
        """
        :param mpc_config: The MPC settings, from the mpc parameters in config/ai4r_policy.yaml.
        :param cone_filter_config: The cone filter settings, from the cone_filter parameters in
            config/ai4r_policy.yaml.
        :param control_config: The control settings, from the control parameters in config/ai4r_policy.yaml.
        :param open_space_config: The open-space settings, from the open_space parameters in config/ai4r_policy.yaml.
        :param action_policy: Which policy chooses the instructions, from the action_policy parameter:
            "mpc" follows the cone lane, and "open_space" drives into the open space between the cones while the
            operator holds the RC drive forwards. A ValueError is raised for any other name.
        :param lane_detection_config: The lane detection settings, from the lane_detection parameters in
            config/ai4r_policy.yaml.
        :param obstacle_detection_config: The obstacle detection settings, from the obstacle_detection parameters in
            config/ai4r_policy.yaml. A ValueError is raised when the MPC stops for obstacles and its stopping distance
            at the target speed is longer than the distance searched, corridor_length_m or lane_length_m, or when
            control.brake_action is 0.0 (coasting) but obstacle_stop_distance_per_speed_s is below
            stop_distance_per_speed_s.
        :param avoidance_config: The settings for driving around obstacles in the lane, from the avoidance
            parameters in config/ai4r_policy.yaml. A ValueError is raised when the line that passes an obstacle would
            still count as blocked: half the car's width plus obstacle_clearance_m below corridor_half_width_m, or when
            max_ramp_curvature_per_m is sharper than the MPC's curvature limits.
        """
        if action_policy not in ACTION_POLICIES:
            raise ValueError(f"action_policy must be one of {ACTION_POLICIES}, not {action_policy!r}")
        stopping_m = (mpc_config.obstacle_stop_distance_per_speed_s * mpc_config.target_speed_m_per_s
                      + mpc_config.obstacle_stop_margin_m)
        searched_m = min(obstacle_detection_config.corridor_length_m, obstacle_detection_config.lane_length_m)
        if mpc_config.obstacle_stop_enabled and stopping_m > searched_m:
            raise ValueError(f"The obstacle stopping distance at the target speed, {stopping_m:.2f} m, is longer than "
                             f"the {searched_m} m searched for obstacles (obstacle_detection.corridor_length_m and "
                             f"lane_length_m), so obstacles would be seen too late to stop")
        if (mpc_config.obstacle_stop_enabled and control_config.brake_action == 0.0
                and mpc_config.obstacle_stop_distance_per_speed_s < mpc_config.stop_distance_per_speed_s):
            raise ValueError(f"Without control.brake_action the car coasts for an obstacle, so "
                             f"mpc.obstacle_stop_distance_per_speed_s "
                             f"({mpc_config.obstacle_stop_distance_per_speed_s}) must be at least "
                             f"mpc.stop_distance_per_speed_s ({mpc_config.stop_distance_per_speed_s})")
        keep_out_m = avoidance_config.car_width_m / 2.0 + avoidance_config.obstacle_clearance_m
        if avoidance_config.enabled and keep_out_m < obstacle_detection_config.corridor_half_width_m:
            raise ValueError(f"Half the car's width plus avoidance.obstacle_clearance_m, {keep_out_m:.2f} m, is below "
                             f"obstacle_detection.corridor_half_width_m "
                             f"({obstacle_detection_config.corridor_half_width_m} m), so the car would stop instead "
                             f"of passing an obstacle")
        max_left = mpc_config.max_curvature_per_m
        max_right = mpc_config.max_right_curvature_per_m or mpc_config.max_curvature_per_m
        if avoidance_config.enabled and avoidance_config.max_ramp_curvature_per_m > min(max_left, max_right):
            raise ValueError(f"avoidance.max_ramp_curvature_per_m ({avoidance_config.max_ramp_curvature_per_m}) is "
                             f"sharper than the car can turn (mpc.max_curvature_per_m {max_left}, right {max_right})")
        self.cone_filter_config = cone_filter_config
        self.action_policy_name = action_policy
        self.lane_detector = LaneDetector(lane_detection_config)
        self.obstacle_detection_config = obstacle_detection_config
        self.action_policy = MpcPolicy(mpc_config)
        self.avoidance = LaneAvoidance(avoidance_config, obstacle_detection_config.car_front_m, max_left, max_right,
                                       stopping_m if mpc_config.obstacle_stop_enabled else 0.0)
        self.open_space_policy = OpenSpacePolicy(open_space_config, control_config.positive_steering_turns_left)
        self.car_controller = CarController(control_config)
        self.obstacles: ObstacleDetection | None = None
        """The lidar obstacle observation of the latest step, with a straight corridor ahead of the car, or None
        without fresh Cartesian lidar points."""
        self.lane_obstacles: list[LaneObstacle] | None = None
        """The objects inside the lane ahead in the latest step, or None without fresh lidar points or a lane."""
        self.lane_path: LanePath | None = None
        """The line across the lane planned past the obstacles in the latest step, or None when avoidance is off or
        there was no lane."""
        self.obstacle_distance_m: float | None = None
        """Distance in metres from the car's front to the nearest obstacle in its way in the latest step, math.inf when
        its way is clear, or None without fresh lidar points."""

    def step(self, observations: CarObservations, memory: CarMemory) -> CarActions:
        """Runs the movement policy for the car using the observations provided to determine the actions to take

        :param observations: The observations received by the car
        :param memory: The memory stored by the car
        :return: The actions for the car to take given the observations
        """
        if self.action_policy_name == "open_space":
            return self._open_space_step(observations)
        # Remove the cone outliers, so every later stage uses the same plausible cones
        observations = dataclasses.replace(
            observations, cones=filter_cones(observations.cones, self.cone_filter_config))
        # Rotate the cones back by mpc.camera_yaw_offset_deg, for a camera that points to one side
        cones = observations.cones
        yaw_offset_rad = math.radians(self.action_policy.config.camera_yaw_offset_deg)
        if cones is not None and yaw_offset_rad != 0.0:
            observations = dataclasses.replace(observations, cones=ConeBatch(
                rotate_cones(cones, yaw_offset_rad), cones.sensor_age, cones.acquisition_to_publish_latency_s))
        # Detect the lane edges, centre line and where the car is in the lane
        lanes = self.lane_detector.detect(observations)
        # Find the obstacles: inside the lane when there is one, otherwise in a straight corridor ahead of the car
        self.obstacles = detect_obstacles(observations, self.obstacle_detection_config)
        self.lane_obstacles = detect_lane_obstacles(observations, lanes, self.obstacle_detection_config)
        # Plan the line past the obstacles: through the middle of the free gap beside each, where there is room. The
        # MPC follows it as seen lookahead_m ahead, and the obstacle check follows it at each obstacle.
        usual_m = (self.action_policy.usual_offset_m(lanes.lane_width_m, lanes.path_curvature_per_m)
                   if lanes is not None and lanes.has_lane else None)
        self.lane_path = self.avoidance.step(observations, lanes, self.lane_obstacles, usual_m)
        self.action_policy.avoidance_offset_m = (
            self.lane_path.offset_at(self.avoidance.config.lookahead_m)
            if self.lane_path is not None and self.lane_path.segments else None)
        self.obstacle_distance_m = self._obstacle_distance_m(lanes)
        # Determine the policy to take based upon the car location; an obstacle in the car's way stops it
        instructions = self.action_policy.run_policy(observations, lanes, self.obstacle_distance_m)
        # The car actions to determine
        car_actions = self.car_controller.step(observations, instructions)
        # Hold the camera at zero pan: the camera's fixed transform to base_link assumes it, and any pan rotates every
        # detected cone position by the pan angle
        car_actions.camera_pan_action = 0.0
        # Where the car is relative to the lane centre, to plot in Foxglove: debug1 is the lateral error in metres
        # (positive left of the centre) and debug2 the heading error in radians (positive pointing left of the lane).
        # Nothing is published on them when this step has no usable lane.
        lane_state = self.action_policy.lane_state
        if lane_state is not None:
            car_actions = dataclasses.replace(
                car_actions, debug1=lane_state.lateral_error_m, debug2=lane_state.heading_error_rad)
        # Return final car actions
        return car_actions

    def _obstacle_distance_m(self, lanes: LaneDetection | None) -> float | None:
        """Finds how far ahead the nearest obstacle in the car's way is, from this step's obstacles.

        With a planned line (lane_path), the car's way past each obstacle is the band across the lane the line covers
        while the car's body is beside it, widened by corridor_half_width_m on both sides and by how far the car is
        off the line now (beyond where the line is between here and lookahead_m ahead), as if it stays that far off.
        So an obstacle the car is not moving clear of in time still stops it. With a lane but no planned line, the
        band reaches from where the car is to the line the MPC follows. Without a lane, it is the straight corridor
        ahead.

        :param lanes: The lane detected at this step, or None.
        :return: Distance in metres from the car's front to the nearest obstacle in the car's way, math.inf when its
            way is clear, or None without fresh lidar points.
        """
        if self.lane_obstacles is not None:
            half_width_m = self.obstacle_detection_config.corridor_half_width_m
            car_m = lanes.lateral_error_m
            path = self.lane_path
            if path is None:
                line_m = self.action_policy.followed_offset_m(lanes.lane_width_m, lanes.path_curvature_per_m)
                return nearest_in_band(self.lane_obstacles, min(car_m, line_m) - half_width_m,
                                       max(car_m, line_m) + half_width_m)
            lowest_m, highest_m = path.offset_range(0.0, self.avoidance.config.lookahead_m)
            # Widen the band towards the car's side only: a car right of the line sweeps more of the lane to the right
            # while it moves back onto it, but no more to the left
            right_margin_m = half_width_m + max(0.0, lowest_m - car_m)
            left_margin_m = half_width_m + max(0.0, car_m - highest_m)
            # base_link is beside an obstacle's near side when the car's front reaches it, and a car length past its
            # far side when the rear leaves it
            car_length_m = self.avoidance.car_length_m
            in_way = math.inf
            for obstacle in self.lane_obstacles:
                right_m, left_m = path.offset_range(obstacle.distance_m, obstacle.far_distance_m + car_length_m)
                in_way = min(in_way, nearest_in_band([obstacle], right_m - right_margin_m, left_m + left_margin_m))
            return in_way
        if self.obstacles is not None:
            return self.obstacles.in_path_distance_m
        return None

    def _open_space_step(self, observations: CarObservations) -> CarActions:
        """Runs the open-space policy, where every cone is an obstacle rather than part of a lane row.

        :param observations: The observations received by the car
        :return: The actions for the car to take given the observations
        """
        observations = dataclasses.replace(
            observations, cones=remove_implausible_cones(observations.cones, self.cone_filter_config))
        instructions = self.open_space_policy.run_policy(observations)
        car_actions = self.car_controller.step(observations, instructions)
        # To plot in Foxglove: debug1 is the chosen heading in degrees (positive left), even while stopped, and
        # nothing when there is none; debug2 is the OpenSpaceStatus number. The camera is held at zero pan, as in step.
        heading = self.open_space_policy.heading_rad
        return dataclasses.replace(car_actions, camera_pan_action=0.0,
                                   debug1=None if heading is None else math.degrees(heading),
                                   debug2=float(self.open_space_policy.status))
