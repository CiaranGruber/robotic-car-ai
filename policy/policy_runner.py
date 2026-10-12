from __future__ import annotations

import dataclasses
import math

from policy.action_policy.avoidance import AvoidanceConfig, LaneAvoidance
from policy.action_policy.mpc import MpcConfig
from policy.action_policy.open_space import OpenSpaceConfig, OpenSpacePolicy
from policy.action_policy.policy import MpcPolicy
from policy.action_policy.reference import rotate_cones
from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones, remove_implausible_cones
from policy.control.control import CarController, ControlConfig
from policy.input_output import CarActions, CarObservations, ConeBatch
from policy.lane_detection.lane_detection import LaneDetectionConfig, LaneDetector
from policy.lane_detection.lanes import LaneDetection
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
            at the target speed is longer than the distance searched, corridor_length_m or lane_length_m.
        :param avoidance_config: The settings for driving around obstacles in the lane, from the avoidance
            parameters in config/ai4r_policy.yaml. A ValueError is raised when the line that passes an obstacle would
            still count as blocked: half the car's width plus obstacle_clearance_m below corridor_half_width_m.
        """
        if action_policy not in ACTION_POLICIES:
            raise ValueError(f"action_policy must be one of {ACTION_POLICIES}, not {action_policy!r}")
        stopping_m = (mpc_config.stop_distance_per_speed_s * mpc_config.target_speed_m_per_s
                      + mpc_config.obstacle_stop_margin_m)
        searched_m = min(obstacle_detection_config.corridor_length_m, obstacle_detection_config.lane_length_m)
        if mpc_config.obstacle_stop_enabled and stopping_m > searched_m:
            raise ValueError(f"The obstacle stopping distance at the target speed, {stopping_m:.2f} m, is longer than "
                             f"the {searched_m} m searched for obstacles (obstacle_detection.corridor_length_m and "
                             f"lane_length_m), so obstacles would be seen too late to stop")
        keep_out_m = avoidance_config.car_width_m / 2.0 + avoidance_config.obstacle_clearance_m
        if avoidance_config.enabled and keep_out_m < obstacle_detection_config.corridor_half_width_m:
            raise ValueError(f"Half the car's width plus avoidance.obstacle_clearance_m, {keep_out_m:.2f} m, is below "
                             f"obstacle_detection.corridor_half_width_m "
                             f"({obstacle_detection_config.corridor_half_width_m} m), so the car would stop instead "
                             f"of passing an obstacle")
        self.cone_filter_config = cone_filter_config
        self.action_policy_name = action_policy
        self.lane_detector = LaneDetector(lane_detection_config)
        self.obstacle_detection_config = obstacle_detection_config
        self.action_policy = MpcPolicy(mpc_config)
        self.avoidance = LaneAvoidance(avoidance_config, obstacle_detection_config.car_front_m)
        self.open_space_policy = OpenSpacePolicy(open_space_config, control_config.positive_steering_turns_left)
        self.car_controller = CarController(control_config)
        self.obstacles: ObstacleDetection | None = None
        """The lidar obstacle observation of the latest step, with a straight corridor ahead of the car, or None
        without fresh Cartesian lidar points."""
        self.lane_obstacles: list[LaneObstacle] | None = None
        """The objects inside the lane ahead in the latest step, or None without fresh lidar points or a lane."""
        self.obstacle_distance_m: float | None = None
        """Distance in metres from the car's front to the nearest obstacle in its way in the latest step, math.inf when
        its way is clear, or None without fresh lidar points."""

    def step(self, observations: CarObservations) -> CarActions:
        """Runs the movement policy for the car using the observations provided to determine the actions to take

        :param observations: The observations received by the car
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
        # Move over in the lane to pass the obstacles when there is room; the obstacle check then follows that line
        self.action_policy.avoidance_offset_m = self.avoidance.step(
            observations, lanes, self.lane_obstacles, self.action_policy.lane_offset_m)
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

        With a lane, the car's way is the band across the lane from where the car is to the line the MPC follows
        (the lane centre moved by lane_offset_m, or the avoidance line), widened by corridor_half_width_m on both
        sides. Covering both includes the car's move over to the line, so an obstacle the car cannot move clear of
        before it is within the stopping distance still stops it. Without a lane, it is the straight corridor ahead.

        :param lanes: The lane detected at this step, or None.
        :return: Distance in metres from the car's front to the nearest obstacle in the car's way, math.inf when its
            way is clear, or None without fresh lidar points.
        """
        if self.lane_obstacles is not None:
            half_width_m = self.obstacle_detection_config.corridor_half_width_m
            car_m = lanes.lateral_error_m
            line_m = self.action_policy.followed_offset_m(lanes.lane_width_m, lanes.path_curvature_per_m)
            return nearest_in_band(self.lane_obstacles, min(car_m, line_m) - half_width_m,
                                   max(car_m, line_m) + half_width_m)
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
