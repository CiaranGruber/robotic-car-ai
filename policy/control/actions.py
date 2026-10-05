"""
Actions.py

This file contains all the potential actions that the control module can take
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class MoveCameraTo:
    """Defines an instruction to move the camera to the desired location"""
    direction: int
    """The direction to move the camera to."""


@dataclass(frozen=True)
class DriveCommand:
    """Defines an instruction to drive at a speed along a path of constant curvature.

    This is the movement policy's (MPC or RL) output for the control module, which converts it into
    the drive and steering actions. Converting curvature into a steering angle needs the car's
    wheelbase and steering calibration, which belong to the control module.
    """
    speed_m_per_s: float
    """Requested forwards speed in m/s. 0.0 requests a stop; it is never negative."""
    curvature_per_m: float
    """Requested path curvature in 1/m, which is 1 / turning radius.

    Positive turns left, negative turns right and 0.0 drives straight.
    """


Instruction = MoveCameraTo | DriveCommand
"""An instruction by the policy that can be received by the control module to determine car actions"""
