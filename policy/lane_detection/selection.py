"""The direction-selected centre path through an admitted junction, separate from observed cone edges.

The normal lane remains a measured circular arc. A junction instead joins an incoming straight,
a turn and an outgoing straight. This reference uses the navigator's sensor-supported route and
estimated current pose; it never creates detected boundary cones or substitutes simulator truth.
"""
from dataclasses import dataclass
import math

import numpy as np

from policy.intersection_state import IntersectionMemory, IntersectionPose
from policy.markers.commands import MarkerCommand


@dataclass(frozen=True)
class SelectedIntersectionLane:
    """The active junction reference in the current body frame, used by control and drawing."""
    command: MarkerCommand
    phase: str
    points: tuple[tuple[float, float], ...]
    width_m: float
    evidence_age_s: float
    reference_curvatures_per_m: tuple[float, ...]
    mpc_curvatures_per_m: tuple[float, ...] = ()
    prediction_step_m: float = 0.


def select_intersection_lane(state: IntersectionMemory, now: IntersectionPose, progress_m: float,
                             arc: np.ndarray, prediction_speed: float, step_s: float,
                             horizon_steps: int) -> SelectedIntersectionLane:
    """Form the selected centre path from a route already admitted from cones, lidar and a marker.

    The same route/progress supplies the displayed centre and MPC's curvature reference. A marker
    alone cannot call this path active: normal straight/curve detection remains the source until
    the navigator commits. Evidence lifetime and clearance are checked by that caller every step.
    """
    if state.phase not in ("turn", "exit") or not state.route or state.command is None:
        raise ValueError("A selected intersection lane requires an admitted active route")
    points = np.asarray(state.route)
    start = np.array((np.interp(progress_m, arc, points[:, 0]), np.interp(progress_m, arc, points[:, 1])))
    ahead = np.vstack((start, points[arc > progress_m+1e-9, :2]))
    c, s = math.cos(now.heading), math.sin(now.heading)
    delta = ahead-np.array((now.x, now.y))
    body = np.column_stack((c*delta[:, 0]+s*delta[:, 1], -s*delta[:, 0]+c*delta[:, 1]))
    sample_s = progress_m+prediction_speed*step_s*np.arange(horizon_steps)
    curvatures = np.interp(sample_s, arc, points[:, 3])
    return SelectedIntersectionLane(state.command, state.phase,
              tuple((float(x), float(y)) for x, y in body), state.width_m,
              now.time_s-state.route_seen_s, tuple(float(k) for k in curvatures))
