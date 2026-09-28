"""
Actions.py

This file contains all the potential actions that the control module can take
"""

from dataclasses import dataclass
from typing import TypeVar


@dataclass(frozen=True)
class MoveCameraTo:
    """Defines an instruction to move the camera to the desired location"""
    direction: int
    """The direction to move the camera to."""


Instruction = TypeVar('Instruction', bound='MoveCameraTo')
"""An instruction by the policy that can be received by the control module to determine car actions"""