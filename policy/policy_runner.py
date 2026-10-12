from __future__ import annotations

import dataclasses

from policy.action_policy.mpc import MpcConfig
from policy.action_policy.policy import MpcPolicy
from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones
from policy.control.control import CarController, ControlConfig
from policy.input_output import CarActions, CarObservations
from policy.lane_detection.lane_detection import LaneDetectionConfig, LaneDetector
from policy.obstacle_detection.obstacle_detection import ObstacleDetectionConfig, detect_obstacles
from policy.obstacle_detection.obstacles import ObstacleDetection


class MovementPolicy:
    """Runs the movement policy for the car, keeping the stages' state between policy steps.

    policy_node creates one when it starts and calls step once per policy step.
    """

    def __init__(self, mpc_config: MpcConfig, cone_filter_config: ConeFilterConfig,
                 lane_detection_config: LaneDetectionConfig,
                 control_config: ControlConfig = ControlConfig(),
                 obstacle_detection_config: ObstacleDetectionConfig = ObstacleDetectionConfig()):
        """
        :param mpc_config: The MPC settings, from the mpc parameters in config/ai4r_policy.yaml.
        :param cone_filter_config: The cone filter settings, from the cone_filter parameters in
            config/ai4r_policy.yaml.
        :param lane_detection_config: The lane detection settings, from the lane_detection parameters in
            config/ai4r_policy.yaml.
        :param control_config: The control settings, from the control parameters in
            config/ai4r_policy.yaml.
        :param obstacle_detection_config: The obstacle detection settings, from the obstacle_detection parameters in
            config/ai4r_policy.yaml.
        """
        self.cone_filter_config = cone_filter_config
        self.lane_detector = LaneDetector(lane_detection_config)
        self.obstacle_detection_config = obstacle_detection_config
        self.action_policy = MpcPolicy(mpc_config)
        self.car_controller = CarController(control_config)
        self.obstacles: ObstacleDetection | None = None
        """The lidar obstacle observation of the latest step, or None without fresh Cartesian lidar points."""

    def step(self, observations: CarObservations) -> CarActions:
        """Runs the movement policy for the car using the observations provided to determine the actions to take

        :param observations: The observations received by the car
        :return: The actions for the car to take given the observations
        """
        # Remove the cone outliers, so every later stage uses the same plausible cones
        observations = dataclasses.replace(
            observations, cones=filter_cones(observations.cones, self.cone_filter_config))
        # Detect the lane edges, centre line and where the car is in the lane
        lanes = self.lane_detector.detect(observations)
        # Find the lidar returns in the path the car is steering along: the previous step's curvature, or straight
        # ahead when a run starts
        steering_curvature_per_m = (0.0 if observations.policy.is_first_policy_step
                                    else self.action_policy.previous_curvature_per_m)
        self.obstacles = detect_obstacles(observations, self.obstacle_detection_config, steering_curvature_per_m)
        # Determine the policy to take based upon the car location
        instructions = self.action_policy.run_policy(observations, lanes)
        # The car actions to determine
        car_actions = self.car_controller.step(observations, instructions)
        # Hold the camera at zero pan: the camera's fixed transform to base_link assumes it, and any pan rotates every
        # detected cone position by the pan angle
        car_actions.camera_pan_action = 0.0
        # Nothing acts on the obstacles yet; report them for Foxglove. debug1 is the distance from the car's front to
        # the obstacle in its path (corridor_length_m when clear, as debug values must be finite). debug2 is the
        # nearest return in the forwards fan, like the notebook's minimum lidar range, which also sees the cones.
        if self.obstacles is not None:
            car_actions.debug1 = min(self.obstacles.in_path_distance_m,
                                     self.obstacle_detection_config.corridor_length_m)
            car_actions.debug2 = min(self.obstacles.forward_ranges_m)
        # Return final car actions
        return car_actions
