from dataclasses import dataclass, replace
from typing import Any, Self

from policy.serialisation import Serialisable


@dataclass(frozen=True)
class MotionPose(Serialisable):
    """An integrated local pose and measured forward motion at policy time, metres/radians/seconds."""
    time_s: float
    x: float
    y: float
    yaw: float
    distance_m: float
    speed: float
    yaw_rate: float


@dataclass(frozen=True)
class ConeTrack(Serialisable):
    """A cone in the run's local frame, with its original last-observed time and travel distance."""
    x: float
    y: float
    z: float
    colour: str
    confidence: float
    seen_s: float
    seen_distance_m: float


@dataclass(frozen=True)
class LaneReference(Serialisable):
    """The last accepted centre arc in its acquisition frame, and that frame's local pose."""
    pose: MotionPose
    x: float
    y: float
    heading: float
    curvature: float
    visible_until_m: float


@dataclass(frozen=True)
class BothEdgeLaneReference(Serialisable):
    """A fresh accepted two-edge lane and its acquisition pose, retained across bounded one-edge gaps."""
    pose: MotionPose
    x: float
    y: float
    heading: float
    curvature: float
    visible_until_m: float
    width_m: float
    left_colour: str
    right_colour: str


@dataclass(frozen=True)
class LaneTrackingMemory(Serialisable):
    """Bounded motion history, cone tracks and accepted lane reference for one policy run."""
    poses: tuple[MotionPose, ...] = ()
    tracks: tuple[ConeTrack, ...] = ()
    last_stamp_ns: int | None = None
    last_acquisition_s: float | None = None
    reference: LaneReference | None = None
    rejected: bool = False
    remembered_count: int = 0
    both_edge_reference: BothEdgeLaneReference | None = None
    holding_single_edge: bool = False

    @classmethod
    def deserialise(cls, data: Any) -> Self:
        """Accepts existing temporal recordings without the one-edge fallback fields."""
        return super().deserialise({**cls().serialise(), **data})


@dataclass(frozen=True)
class CorridorExperimentMemory(Serialisable):
    """Latched status and previous curvature of the explicitly selected offline corridor experiment."""
    started: bool = False
    stopped: bool = False
    curvature: float = 0.0
    filtered_tracks: tuple[ConeTrack, ...] = ()


@dataclass(frozen=True)
class Memory(Serialisable):
    """Data kept from one policy step to a later one.

    Read the current set with CarMemory.data. Change it only through CarMemory.
    """

    policy_step: int
    """How many policy steps have finished. It starts at 0."""
    lane_tracking: LaneTrackingMemory = LaneTrackingMemory()
    """Optional cone memory and lane continuity evidence; empty when disabled or motion is unavailable."""
    corridor_experiment: CorridorExperimentMemory = CorridorExperimentMemory()
    """Unused by the shipped ROS policy; stores the offline experimental follower's temporal state."""

    @classmethod
    def deserialise(cls, data: Any) -> Self:
        """Also accepts recordings made before lane tracking was added."""
        return super().deserialise({"lane_tracking": LaneTrackingMemory().serialise(),
                                    "corridor_experiment": CorridorExperimentMemory().serialise(), **data})


class CarMemory:
    """Holds the policy's memory for one run and is the only place that changes it.

    The node creates one at startup, passes it into each policy step, and calls step once that
    step has finished.
    """

    def __init__(self):
        """Starts a run with no policy steps finished."""
        self.__internal = Memory(
            policy_step = 0,
        )

    @property
    def data(self) -> Memory:
        """
        :return: The memory held now.
        """
        return self.__internal

    def step(self):
        """Records that one policy step has finished by adding one to the policy step count."""
        self.__internal = replace(self.__internal, policy_step=self.__internal.policy_step + 1)

    def update_lane_tracking(self, state: LaneTrackingMemory):
        """Replaces the serialisable temporal lane state after one policy calculation."""
        self.__internal = replace(self.__internal, lane_tracking=state)

    def update_corridor_experiment(self, state: CorridorExperimentMemory):
        """Updates only the explicitly selected offline follower's state."""
        self.__internal = replace(self.__internal, corridor_experiment=state)

    @classmethod
    def deserialise(cls, data: Any) -> Self:
        """
        :param data: JSON-friendly data produced by serialising Memory.
        :return: A car memory holding that set.
        """
        memory = cls()
        memory.__internal = Memory.deserialise(data)
        return memory
