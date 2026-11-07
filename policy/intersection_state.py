"""Serialisable acquisition-time motion and geometry for the optional intersection navigator."""
from dataclasses import dataclass

from policy.markers.commands import MarkerCommand
from policy.serialisation import Serialisable


@dataclass(frozen=True)
class IntersectionPose(Serialisable):
    time_s: float
    x: float
    y: float
    heading: float
    distance_m: float
    speed: float
    yaw_rate: float


@dataclass(frozen=True)
class IntersectionCone(Serialisable):
    x: float
    y: float
    colour: str
    seen_s: float


@dataclass(frozen=True)
class IntersectionMemory(Serialisable):
    """Bounded observations and a committed, finite intersection route."""
    phase: str = "approach"
    poses: tuple[IntersectionPose, ...] = ()
    cones: tuple[IntersectionCone, ...] = ()
    frame_heading: float | None = None
    frame_left_colour: str | None = None
    centre_v: float = 0.
    width_m: float | None = None
    frame_seen_s: float | None = None
    frame_distance_m: float = 0.
    command: MarkerCommand | None = None
    command_seen_s: float | None = None
    marker_id: int | None = None
    marker_xy: tuple[float, float] | None = None
    consumed_marker_xy: tuple[float, float] | None = None
    centre_u: float | None = None
    approach_end_u: float | None = None
    available_exits: tuple[MarkerCommand, ...] = ()
    closed_exits: tuple[MarkerCommand, ...] = ()
    route: tuple[tuple[float, float, float, float], ...] = ()
    route_seen_s: float | None = None
    route_distance_m: float = 0.
    progress_m: float = 0.
    previous_curvature: float = 0.
    last_nonempty_s: float | None = None
    exit_confirmed_since_s: float | None = None
    exit_last_evidence_s: float | None = None
    completed_distance_m: float | None = None
    stop_reason: str = ""

    @classmethod
    def deserialise(cls, data):
        return super().deserialise({"available_exits": [], "closed_exits": [], "frame_left_colour": None,
                                   "approach_end_u": None, **data})
