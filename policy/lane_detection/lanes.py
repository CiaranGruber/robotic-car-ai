"""
lanes.py

This file holds the related types for the lane detection module
"""
from dataclasses import dataclass

from dreamgym.envs import Road


@dataclass
class LaneDetection:
    """The lanes detected from the observations at one policy step"""
    road_map: Road
    """The detected road, as a DreamGym road"""
