"""
Control.py

This file is responsible for receiving the car commands from the policy and converting them to the drive and steering
actions
"""
from policy.control.actions import Instruction
from policy.input_output import CarActions, CarObservations


def determine_car_actions(observations: CarObservations, instructions: list[Instruction]) -> CarActions:
    """Converts the instructions chosen by the policy into the car actions for this step

    :param observations: The observations taken by the car at the current policy step
    :param instructions: The instructions chosen by the policy
    :return: The drive, steering and camera pan actions for the car to take
    """
    pass
