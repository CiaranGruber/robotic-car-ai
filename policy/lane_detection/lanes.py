"""
lanes.py

This file holds the related types for the lane detection module
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    # Only imported for type checking, because dreamgym is not installed on the car
    from dreamgym.envs import Road


@dataclass
class LaneDetection:
    """The lanes detected from the observations at one policy step"""
    road_map: Road
    """The detected road, as a DreamGym road"""
