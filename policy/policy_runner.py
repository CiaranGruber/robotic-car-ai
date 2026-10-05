from __future__ import annotations

from policy.action_policy.mpc import MpcConfig
from policy.action_policy.policy import MpcPolicy
from policy.control.control import CarControl, ControlConfig
from policy.input_output import CarActions, CarObservations
from policy.lane_detection.lane_detection import detect_lanes


class MovementPolicy:
    """Runs the movement policy for the car, keeping the stages' state between policy steps.

    policy_node creates one when it starts and calls step once per policy step.
    """

    def __init__(self, mpc_config: MpcConfig, control_config: ControlConfig):
        """
        :param mpc_config: The MPC settings, from the mpc parameters in config/ai4r_policy.yaml.
        :param control_config: The control settings, from the control parameters in config/ai4r_policy.yaml.
        """
        self.action_policy = MpcPolicy(mpc_config)
        self.control = CarControl(control_config)

    def step(self, observations: CarObservations) -> CarActions:
        """Runs the movement policy for the car using the observations provided to determine the actions to take

        :param observations: The observations received by the car
        :return: The actions for the car to take given the observations
        """
        # Detect the lanes and construct a road instance
        road_map = detect_lanes(observations)
        # Determine the policy to take based upon the car location
        instructions = self.action_policy.run_policy(observations, road_map)
        # The car actions to determine
        car_actions = self.control.determine_car_actions(observations, instructions)
        # Return final car actions
        return car_actions
