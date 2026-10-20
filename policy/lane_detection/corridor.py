"""Offline experimental centreline from connected opposite-colour cone pairs.

The existing cone filter/edge observer remains unchanged. Adjacent mixed-colour Delaunay triangles connect
cross-lane midpoints into a polyline, preserving corners instead of forcing all cones onto one circle.
This is not wired into MovementPolicy/ROS: it needs independent sensor, obstacle and timing qualification.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial import Delaunay, QhullError


@dataclass(frozen=True)
class CorridorConfig:
    """Bounds for an offline cone-corridor experiment, in metres and radians."""
    min_width_m: float = 0.3
    max_width_m: float = 1.4
    max_cone_gap_m: float = 0.9
    max_cones: int = 96
    lookahead_m: float = 0.9

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in (
                self.min_width_m, self.max_width_m, self.max_cone_gap_m, self.lookahead_m)):
            raise ValueError("Corridor lengths must be positive and finite")
        if self.min_width_m >= self.max_width_m or not isinstance(self.max_cones, int) or not 4 <= self.max_cones <= 256:
            raise ValueError("Invalid corridor width/count bounds")


def cone_corridor(cones, config=CorridorConfig()):
    """Return a forward-connected midpoint chain in the input cone frame, or an empty tuple.

    Work is bounded by max_cones. Same-colour gaps and cross-lane gate widths are bounded independently, so
    triangulation cannot silently bridge an arbitrarily missing boundary. Degenerate geometry gives no route.
    """
    cones = sorted(cones or [], key=lambda c: math.hypot(c.pos.x, c.pos.y))[:config.max_cones]
    if len(cones) < 4 or len({c.colour for c in cones}) != 2:
        return ()
    points = np.array([(c.pos.x, c.pos.y) for c in cones])
    if not np.isfinite(points).all() or len(np.unique(points, axis=0)) != len(points):
        return ()
    try:
        triangles = Delaunay(points).simplices
    except (QhullError, ValueError):
        return ()
    positions, graph = {}, {}
    for triangle in triangles:
        gates = []
        good = True
        for i, j in ((triangle[0], triangle[1]), (triangle[1], triangle[2]), (triangle[2], triangle[0])):
            distance = float(np.linalg.norm(points[i] - points[j]))
            if cones[i].colour == cones[j].colour:
                good &= distance <= config.max_cone_gap_m
            elif config.min_width_m <= distance <= config.max_width_m:
                key = tuple(sorted((int(i), int(j))))
                gates.append(key)
                positions[key] = (points[i] + points[j]) / 2
        if good and len(gates) == 2:
            a, b = gates
            graph.setdefault(a, set()).add(b)
            graph.setdefault(b, set()).add(a)
    if not graph:
        return ()
    # Each gate belongs to at most two triangles: the graph is a collection of chains/cycles.
    # Anchor near the car, then follow the forward tangent; never pick a disconnected remote branch.
    starts = [key for key in graph if positions[key][0] > -0.2]
    if not starts:
        return ()
    start = min(starts, key=lambda key: float(positions[key] @ positions[key]))
    candidates = []
    for neighbor in sorted(graph[start]):
        tangent = positions[neighbor] - positions[start]
        if tangent[0] <= 0:
            continue
        chain, seen = [start, neighbor], {start, neighbor}
        while len(chain) < 2 * config.max_cones:
            following = sorted(graph[chain[-1]] - seen)
            if not following:
                break
            next_key = following[0]
            chain.append(next_key)
            seen.add(next_key)
        candidates.append(chain)
    if not candidates:
        return ()
    chain = max(candidates, key=len)
    route = np.array([positions[k] for k in chain])
    if len(route) > 2:
        route[1:-1] = (route[:-2] + 2 * route[1:-1] + route[2:]) / 4
    return tuple((float(x), float(y)) for x, y in route)


def pursuit_curvature(path, lookahead_m):
    """Curvature towards a point lookahead_m along a connected polyline; None without a forward target."""
    if len(path) < 2 or not math.isfinite(lookahead_m) or lookahead_m <= 0:
        return None
    remaining = lookahead_m - math.hypot(*path[0])
    target = path[0]
    for a, b in zip(path, path[1:]):
        if remaining <= 0:
            break
        length = math.dist(a, b)
        if length <= 1e-9:
            continue
        fraction = min(1.0, remaining / length)
        target = (a[0] + fraction * (b[0] - a[0]), a[1] + fraction * (b[1] - a[1]))
        remaining -= length
    x, y = target
    if x <= 0.05 or x * x + y * y < 1e-6:
        return None
    return 2 * y / (x * x + y * y)


def left_uturn_span_check(max_left_curvature_per_m, available_span_m):
    """Necessary point-vehicle feasibility bound for the intended forward left-winding 180-degree turn.

    Between the last heading 0 and first heading pi, sin(heading)>=0 and d(heading)/ds<=k.
    Thus dy/ds >= sin(heading)*d(heading)/ds/k and total vertical travel is at least 2/k.
    A pass is only 'not ruled out'; it does not prove clearance or feasibility of a full vehicle.
    """
    if not all(math.isfinite(v) and v > 0 for v in (max_left_curvature_per_m, available_span_m)):
        raise ValueError("Curvature and available span must be positive and finite")
    required = 2 / max_left_curvature_per_m
    return {"necessary_span_m": required, "available_span_m": available_span_m,
            "ruled_out": required > available_span_m,
            "scope": "forward left-winding turn; necessary point-car bound, not a clearance certificate"}
