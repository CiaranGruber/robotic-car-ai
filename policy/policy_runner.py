from __future__ import annotations

from policy.action_policy.policy import run_policy
from policy.control.control import determine_car_actions
from policy.input_output import CarActions, CarObservations, default_actions
from policy.lane_detection.lane_detection import detect_lanes


def run_movement_policy(observations: CarObservations) -> CarActions:
    """Runs the movement policy for the car using the observations provided to determine the actions to take

    :param observations: The observations received by the car
    :return: The actions for the car to take given the observations
    """
    # Detect the lanes and construct a road instance
    road_map = detect_lanes(observations)
    # Determine the policy to take based upon the car location
    instructions = run_policy(observations, road_map)
    # The car actions to determine
    car_actions = determine_car_actions(observations, instructions)
    # Return final car actions
    return car_actions
