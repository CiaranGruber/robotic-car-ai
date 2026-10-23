"""The commands ArUco markers give the car, and the sighting of a marker being confirmed."""
from dataclasses import dataclass
from enum import Enum
from typing import Any, Self

from policy.serialisation import Serialisable


class MarkerCommand(Serialisable, Enum):
    """What a marker tells the car to do. Each has its marker ID in the markers parameters of
    config/ai4r_policy.yaml."""
    KEEP_LEFT = 0
    """Drive along the left side of the lane (mpc.lane_side "LEFT") from now on."""
    KEEP_RIGHT = 1
    """Drive along the right side of the lane (mpc.lane_side "RIGHT") from now on."""
    TURN_LEFT = 2
    """Turn left at the next intersection."""
    TURN_RIGHT = 3
    """Turn right at the next intersection."""
    STRAIGHT = 4
    """Go straight on at the next intersection."""
    KEEP_CENTRE = 5
    """Drive along the lane at mpc.lane_offset_m (lane side "CENTRE") from now on."""

    @property
    def lane_side(self) -> str | None:
        """
        :return: The lane side this command sets, or None for an intersection command.
        """
        return {MarkerCommand.KEEP_LEFT: "LEFT", MarkerCommand.KEEP_RIGHT: "RIGHT",
                MarkerCommand.KEEP_CENTRE: "CENTRE"}.get(self)

    def serialise(self) -> str:
        """
        :return: The enum member name.
        """
        return self.name

    @classmethod
    def deserialise(cls, data: Any) -> Self:
        """
        :param data: Enum member name, or the underlying integer value.
        :return: The matching command.
        """
        if isinstance(data, str):
            return cls[data]
        return cls(data)


@dataclass(frozen=True)
class MarkerSighting(Serialisable):
    """A command marker seen in range in consecutive policy steps, up to the latest one."""

    marker_id: int
    """The marker's ID."""
    steps: int
    """How many consecutive policy steps up to the latest one saw it in range."""
