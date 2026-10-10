from __future__ import annotations

import dataclasses
import math

from policy.action_policy.mpc import MpcConfig
from policy.action_policy.open_space import OpenSpaceConfig, OpenSpacePolicy
from policy.action_policy.policy import MpcPolicy
from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones, remove_implausible_cones
from policy.control.control import CarControl, ControlConfig
from policy.input_output import CarActions, CarObservations
from policy.lane_detection.lane_detection import detect_lanes

ACTION_POLICIES = ("mpc", "open_space")
"""Valid names for the action_policy parameter in config/ai4r_policy.yaml."""


class MovementPolicy:
    """Runs the movement policy for the car, keeping the stages' state between policy steps.

    policy_node creates one when it starts and calls step once per policy step.
    """

    def __init__(self, mpc_config: MpcConfig, cone_filter_config: ConeFilterConfig, control_config: ControlConfig,
                 open_space_config: OpenSpaceConfig = OpenSpaceConfig(), action_policy: str = "mpc"):
        """
        :param mpc_config: The MPC settings, from the mpc parameters in config/ai4r_policy.yaml.
        :param cone_filter_config: The cone filter settings, from the cone_filter parameters in
            config/ai4r_policy.yaml.
        :param control_config: The control settings, from the control parameters in config/ai4r_policy.yaml.
        :param open_space_config: The open-space settings, from the open_space parameters in config/ai4r_policy.yaml.
        :param action_policy: Which policy chooses the instructions, from the action_policy parameter:
            "mpc" follows the cone lane, and "open_space" drives into the open space between the cones while the
            operator holds the RC drive forwards. A ValueError is raised for any other name.
        """
        if action_policy not in ACTION_POLICIES:
            raise ValueError(f"action_policy must be one of {ACTION_POLICIES}, not {action_policy!r}")
        self.cone_filter_config = cone_filter_config
        self.action_policy_name = action_policy
        self.action_policy = MpcPolicy(mpc_config)
        self.open_space_policy = OpenSpacePolicy(open_space_config, control_config.positive_steering_turns_left)
        self.control = CarControl(control_config)

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
        # Detect the lanes and construct a road instance
        road_map = detect_lanes(observations)
        # Determine the policy to take based upon the car location
        instructions = self.action_policy.run_policy(observations, road_map)
        # The car actions to determine
        car_actions = self.control.determine_car_actions(observations, instructions)
        # Return final car actions
        return car_actions

    def _open_space_step(self, observations: CarObservations) -> CarActions:
        """Runs the open-space policy, where every cone is an obstacle rather than part of a lane row.

        :param observations: The observations received by the car
        :return: The actions for the car to take given the observations
        """
        observations = dataclasses.replace(
            observations, cones=remove_implausible_cones(observations.cones, self.cone_filter_config))
        instructions = self.open_space_policy.run_policy(observations)
        car_actions = self.control.determine_car_actions(observations, instructions)
        # To plot in Foxglove: debug1 is the chosen heading in degrees (positive left), even while stopped, and
        # nothing when there is none; debug2 is the OpenSpaceStatus number.
        heading = self.open_space_policy.heading_rad
        return dataclasses.replace(car_actions, debug1=None if heading is None else math.degrees(heading),
                                   debug2=float(self.open_space_policy.status))
