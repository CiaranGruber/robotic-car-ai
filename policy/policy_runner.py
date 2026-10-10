from __future__ import annotations

import dataclasses

from policy.action_policy.mpc import MpcConfig
from policy.action_policy.policy import MpcPolicy
from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones
from policy.control.control import CarController, ControlConfig
from policy.input_output import CarActions, CarObservations
from policy.lane_detection.lane_detection import LaneDetectionConfig, LaneDetector


class MovementPolicy:
    """Runs the movement policy for the car, keeping the stages' state between policy steps.

    policy_node creates one when it starts and calls step once per policy step.
    """

    def __init__(self, mpc_config: MpcConfig, cone_filter_config: ConeFilterConfig,
                 lane_detection_config: LaneDetectionConfig,
                 control_config: ControlConfig = ControlConfig()):
        """
        :param mpc_config: The MPC settings, from the mpc parameters in config/ai4r_policy.yaml.
        :param cone_filter_config: The cone filter settings, from the cone_filter parameters in
            config/ai4r_policy.yaml.
        :param lane_detection_config: The lane detection settings, from the lane_detection parameters in
            config/ai4r_policy.yaml.
        :param control_config: The control settings, from the control parameters in
            config/ai4r_policy.yaml.
        """
        self.cone_filter_config = cone_filter_config
        self.lane_detector = LaneDetector(lane_detection_config)
        self.action_policy = MpcPolicy(mpc_config)
        self.car_controller = CarController(control_config)

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
        # Determine the policy to take based upon the car location
        instructions = self.action_policy.run_policy(observations, lanes)
        # The car actions to determine
        car_actions = self.car_controller.step(observations, instructions)
        # Hold the camera at zero pan: the camera's fixed transform to base_link assumes it, and any pan rotates every
        # detected cone position by the pan angle
        car_actions.camera_pan_action = 0.0
        # Return final car actions
        return car_actions
