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
    """Latched status, motion-compensated route and command history of the corridor follower."""
    started: bool = False
    stopped: bool = False
    curvature: float = 0.0
    filtered_tracks: tuple[ConeTrack, ...] = ()
    route: tuple[tuple[float, float], ...] = ()
    route_seen_s: float | None = None
    route_seen_distance_m: float | None = None
    left_colour: str | None = None
    stop_reason: str = ""
    previous_speed_m_per_s: float = 0.0
    runtime_started: bool = False
    runtime_route: tuple[tuple[float, float], ...] = ()
    runtime_route_seen_s: float | None = None
    runtime_route_seen_distance_m: float | None = None

    @classmethod
    def deserialise(cls, data: Any) -> Self:
        """Older offline recordings have no retained route."""
        return super().deserialise({**cls().serialise(), **data})


@dataclass(frozen=True)
class ThreeRowTurnMemory(Serialisable):
    """Observed row geometry and a bounded odometry-frame manoeuvre for the offline three-row learner."""
    poses: tuple[MotionPose, ...] = ()
    tracks: tuple[ConeTrack, ...] = ()
    last_stamp_ns: int | None = None
    last_acquisition_s: float | None = None
    row_heading: float | None = None
    row_offsets: tuple[float, ...] = ()
    row_ends: tuple[float, ...] = ()
    row_end_seen_s: tuple[float, ...] = ()
    observed_start: float | None = None
    observed_start_xy: tuple[float, float] | None = None
    observed_start_seen_s: float | None = None
    row_end_xy: tuple[tuple[float, float], ...] = ()
    startup_heading: float | None = None
    startup_since_s: float | None = None
    startup_seen_s: float | None = None
    startup_support: tuple[tuple[float, float], ...] = ()
    startup_samples: int = 0
    route: tuple[tuple[float, float], ...] = ()
    planned_row_ends: tuple[float, ...] = ()
    turn_start_u: float | None = None
    cap_u: float | None = None
    cap_v: float | None = None
    cap_radius_m: float | None = None
    cap_seen_s: float | None = None
    route_index: int = 0
    route_seen_s: float | None = None
    route_seen_distance_m: float | None = None
    previous_curvature: float = 0.0
    last_nonempty_s: float | None = None
    has_driven: bool = False
    speed_commands: tuple[tuple[float, float], ...] = ()
    stopped: bool = False
    stop_reason: str = ""

    @classmethod
    def deserialise(cls, data: Any) -> Self:
        return super().deserialise({**cls().serialise(), **data})


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
    """State for the explicitly selected corridor follower; unused by MPC/open-space modes."""
    three_row_turn: ThreeRowTurnMemory = ThreeRowTurnMemory()
    """Offline learned three-row experiment; never selected by the ROS runner."""

    @classmethod
    def deserialise(cls, data: Any) -> Self:
        """Also accepts recordings made before lane tracking was added."""
        return super().deserialise({"lane_tracking": LaneTrackingMemory().serialise(),
                                    "corridor_experiment": CorridorExperimentMemory().serialise(),
                                    "three_row_turn": ThreeRowTurnMemory().serialise(), **data})


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
        """Updates the corridor follower's serialisable state."""
        self.__internal = replace(self.__internal, corridor_experiment=state)

    def update_three_row_turn(self, state: ThreeRowTurnMemory):
        """Replace the bounded offline learner's observation and route state."""
        self.__internal = replace(self.__internal, three_row_turn=state)

    @classmethod
    def deserialise(cls, data: Any) -> Self:
        """
        :param data: JSON-friendly data produced by serialising Memory.
        :return: A car memory holding that set.
        """
        memory = cls()
        memory.__internal = Memory.deserialise(data)
        return memory
