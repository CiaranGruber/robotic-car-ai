"""
markers.py

Reads the ArUco markers the car sees and turns the ones mapped in the markers parameters of config/ai4r_policy.yaml
into commands: keep to the left, right or centre of the lane, or turn left, turn right or go straight at the next
intersection.
"""
import logging
import math
from dataclasses import dataclass, fields

from policy.input_output import CarObservations
from policy.markers.commands import MarkerCommand
from policy.memory import CarMemory

_LOGGER = logging.getLogger(__name__)

NO_MARKER = -1
"""A marker ID setting that maps no marker to its command."""


@dataclass(frozen=True)
class MarkerConfig:
    """Settings for the marker commands, loaded from the markers parameters in config/ai4r_policy.yaml.

    Each *_id setting is the ArUco ID, in the aruco_detector's dictionary, that gives that command, or NO_MARKER (-1)
    for none. No two commands may share an ID.
    """
    keep_left_id: int = NO_MARKER
    """Marker that moves the car to the left side of the lane (lane side "LEFT") until another side marker."""
    keep_right_id: int = NO_MARKER
    """Marker that moves the car to the right side of the lane (lane side "RIGHT") until another side marker."""
    keep_centre_id: int = NO_MARKER
    """Marker that moves the car back to mpc.lane_offset_m (lane side "CENTRE") until another side marker."""
    turn_left_id: int = NO_MARKER
    """Marker that asks for a left turn at the next intersection."""
    turn_right_id: int = NO_MARKER
    """Marker that asks for a right turn at the next intersection."""
    straight_id: int = NO_MARKER
    """Marker that asks to go straight on at the next intersection."""
    max_distance_m: float = 1.5
    """A marker counts only when its pose is at most this far from base_link, in metres, and ahead of it. Markers
    without a pose do not count. Positive and finite."""
    confirm_steps: int = 2
    """Consecutive policy steps that must see the same command marker in range before its command is given, on top of
    the aruco_detector's own min_consecutive_frames. At least 1."""

    def __post_init__(self):
        ids = [getattr(self, field.name) for field in fields(self) if field.name.endswith("_id")]
        if any(not isinstance(marker_id, int) or marker_id < NO_MARKER for marker_id in ids):
            raise ValueError(f"markers.*_id must be integers, {NO_MARKER} for no marker or an ArUco ID >= 0: {ids}")
        used = [marker_id for marker_id in ids if marker_id != NO_MARKER]
        if len(used) != len(set(used)):
            raise ValueError(f"Two marker commands share an ID: {ids}")
        if not (math.isfinite(self.max_distance_m) and self.max_distance_m > 0.0):
            raise ValueError(f"markers.max_distance_m must be positive and finite, not {self.max_distance_m}")
        if self.confirm_steps < 1:
            raise ValueError(f"markers.confirm_steps must be at least 1, not {self.confirm_steps}")

    @property
    def commands(self) -> dict[int, MarkerCommand]:
        """
        :return: The command of each mapped marker ID.
        """
        mapping = {self.keep_left_id: MarkerCommand.KEEP_LEFT, self.keep_right_id: MarkerCommand.KEEP_RIGHT,
                   self.keep_centre_id: MarkerCommand.KEEP_CENTRE,
                   self.turn_left_id: MarkerCommand.TURN_LEFT, self.turn_right_id: MarkerCommand.TURN_RIGHT,
                   self.straight_id: MarkerCommand.STRAIGHT}
        return {marker_id: command for marker_id, command in mapping.items() if marker_id != NO_MARKER}


class MarkerReader:
    """Finds the command marker in range at each policy step and gives its command once it is confirmed."""

    def __init__(self, config: MarkerConfig):
        """
        :param config: The marker settings.
        """
        self.config = config
        self.commands = config.commands

    def nearest_in_range(self, observations: CarObservations) -> int | None:
        """
        :param observations: This step's observations.
        :return: The ID of the nearest mapped marker with a pose ahead of base_link within max_distance_m, or None
            without one or without a fresh fiducial batch.
        """
        fiducials = observations.fiducials
        if fiducials is None:
            return None
        nearest_id, nearest_m = None, math.inf
        for detection in fiducials.detections:
            if detection.id not in self.commands or detection.pose is None:
                continue
            position = detection.pose.position
            distance_m = math.hypot(position.x, position.y)
            if position.x > 0.0 and distance_m <= self.config.max_distance_m and distance_m < nearest_m:
                nearest_id, nearest_m = detection.id, distance_m
        return nearest_id

    def step(self, observations: CarObservations, memory: CarMemory) -> MarkerCommand | None:
        """Records this step's sighting in the memory, and records an intersection command once it is confirmed.

        Only the nearest marker in range is followed. A command is given once per sighting: at the step where the same
        marker has been the nearest in range for confirm_steps steps in a row. Losing it for a step and seeing it again
        gives it again.

        :param observations: This step's observations.
        :param memory: The car memory.
        :return: The command given at this step, or None. A lane side command is for the caller to apply to the MPC,
            which keeps it until another lane side command or a new run.
        """
        memory.see_marker(self.nearest_in_range(observations))
        sighting = memory.data.marker_sighting
        if sighting is None or sighting.steps != self.config.confirm_steps:
            return None
        command = self.commands[sighting.marker_id]
        intersection = memory.data.intersection
        if intersection is not None and intersection.phase in ("turn", "exit", "stopped"):
            return None  # A new sighting cannot change or rearm a committed manoeuvre.
        if command.lane_side is None:
            memory.command_intersection(command, observations.policy.policy_elapsed_s-float(observations.fiducials.sensor_age))
            _LOGGER.info("Marker %d: %s at the next intersection", sighting.marker_id, command.name)
        else:
            _LOGGER.info("Marker %d: %s, lane side %s", sighting.marker_id, command.name, command.lane_side)
        return command
