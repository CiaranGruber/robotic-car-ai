from __future__ import annotations

import dataclasses

from policy.action_policy.mpc import MpcConfig
from policy.action_policy.policy import MpcPolicy
from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones
from policy.control.control import CarControl, ControlConfig
from policy.input_output import CarActions, CarObservations
from policy.lane_detection.lane_detection import detect_lanes


class MovementPolicy:
    """Runs the movement policy for the car, keeping the stages' state between policy steps.

    policy_node creates one when it starts and calls step once per policy step.
    """

    def __init__(self, mpc_config: MpcConfig, cone_filter_config: ConeFilterConfig, control_config: ControlConfig):
        """
        :param mpc_config: The MPC settings, from the mpc parameters in config/ai4r_policy.yaml.
        :param cone_filter_config: The cone filter settings, from the cone_filter parameters in
            config/ai4r_policy.yaml.
        :param control_config: The control settings, from the control parameters in config/ai4r_policy.yaml.
        """
        self.cone_filter_config = cone_filter_config
        self.action_policy = MpcPolicy(mpc_config)
        self.control = CarControl(control_config)

    def step(self, observations: CarObservations) -> CarActions:
        """Runs the movement policy for the car using the observations provided to determine the actions to take

        :param observations: The observations received by the car
        :return: The actions for the car to take given the observations
        """
        # Remove the cone outliers, so every later stage uses the same plausible cones
        observations = dataclasses.replace(
            observations, cones=filter_cones(observations.cones, self.cone_filter_config))
        # Detect the lanes and construct a road instance
        road_map = detect_lanes(observations)
        # Determine the policy to take based upon the car location
        instructions = self.action_policy.run_policy(observations, road_map)
        # The car actions to determine
        car_actions = self.control.determine_car_actions(observations, instructions)
        # Return final car actions
        return car_actions
