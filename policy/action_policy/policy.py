"""
Policy.py

This file is responsible for taking in the observations and detected lanes and choosing the instructions for the
control module
"""

from policy.control.actions import Instruction
from policy.input_output import CarObservations
from policy.lane_detection.lanes import LaneDetection


def run_policy(observations: CarObservations, lanes: LaneDetection) -> list[Instruction]:
    """Chooses the instructions for the car to follow from the observations and detected lanes

    :param observations: The observations taken by the car at the current policy step
    :param lanes: The lanes detected by the lane detection module
    :return: The instructions for the control module to convert into car actions
    """
    pass
