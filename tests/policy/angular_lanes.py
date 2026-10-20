"""Test-only U-turn with straight cone segments and sharp corners, rather than circular cone placement."""
from dataclasses import dataclass
import math

import numpy as np

from policy.input_output import ConeColour
from tests.policy.curved_lanes import RUN_IN_M, SAMPLE_M


@dataclass(frozen=True)
class AngularUTurn:
    """A rectangular U corridor with independently placed cones on its two polygonal boundaries."""
    leg_length_m: float = 3.0
    cross_length_m: float = 3.0
    width_m: float = 0.8
    cone_spacing_m: float = 0.5
    placement_jitter_m: float = 0.025
    seed: int = 17

    def __post_init__(self):
        for value in (self.leg_length_m, self.cross_length_m, self.width_m, self.cone_spacing_m):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Angular U dimensions must be positive and finite")
        if min(self.leg_length_m, self.cross_length_m) <= self.width_m:
            raise ValueError("Angular U legs must exceed corridor width")
        if not math.isfinite(self.placement_jitter_m) or not 0 <= self.placement_jitter_m < self.width_m / 4:
            raise ValueError("Placement jitter must be finite and below a quarter lane width")

    @property
    def vertices(self):
        return ((0.0, 0.0), (self.leg_length_m, 0.0),
                (self.leg_length_m, self.cross_length_m), (0.0, self.cross_length_m))

    def boundary_vertices(self, side):
        offset = side * self.width_m / 2
        return ((0.0, offset), (self.leg_length_m - offset, offset),
                (self.leg_length_m - offset, self.cross_length_m - offset),
                (0.0, self.cross_length_m - offset))

    def centre_line(self):
        # This polyline is a scoring reference, not a demand to drive infinite curvature at its corners.
        points = ((-RUN_IN_M, 0.0), *self.vertices)
        rows = []
        distance = -RUN_IN_M
        for a, b in zip(points, points[1:]):
            length = math.dist(a, b)
            heading = math.atan2(b[1] - a[1], b[0] - a[0])
            for along in np.arange(0.0, length, SAMPLE_M):
                rows.append((distance + along, a[0] + along * math.cos(heading),
                             a[1] + along * math.sin(heading), heading, 0.0))
            distance += length
        rows.append((distance, *points[-1], math.pi, 0.0))
        return dict(zip(("s", "x", "y", "heading", "curvature"), np.array(rows).T))

    def _corridor_polygon(self):
        # Entry/exit are open: extend the scoring corridor over the run-in so a rear/nose crossing x=0
        # is not incorrectly counted as a lateral boundary crossing. Actual cone positions stay unchanged.
        outer, inner = list(self.boundary_vertices(-1)), list(self.boundary_vertices(1))
        for boundary in (outer, inner):
            boundary[0] = (-RUN_IN_M, boundary[0][1])
            boundary[-1] = (-RUN_IN_M, boundary[-1][1])
        return (*outer, *reversed(inner))

    def outside_distance(self, x, y):
        """Signed distance to the actual polygonal corridor, positive outside, in metres."""
        polygon = self._corridor_polygon()
        inside = False
        nearest = math.inf
        for a, b in zip(polygon, (*polygon[1:], polygon[0])):
            dx, dy = b[0] - a[0], b[1] - a[1]
            along = min(1.0, max(0.0, ((x - a[0]) * dx + (y - a[1]) * dy) / (dx * dx + dy * dy)))
            nearest = min(nearest, math.hypot(x - a[0] - along * dx, y - a[1] - along * dy))
            if (a[1] > y) != (b[1] > y) and x < a[0] + (y - a[1]) * dx / dy:
                inside = not inside
        return -nearest if inside else nearest

    def footprint_outside_distance(self, corners):
        """Detect body-edge excursions, including inner-notch crossings with all corners inside.

        Split each body edge at every corridor intersection and test each resulting interval's midpoint.
        Positive values establish an excursion; their magnitude is a sampled lower bound on maximum intrusion.
        """
        polygon = self._corridor_polygon()
        worst = -math.inf
        for a, b in zip(corners, (*corners[1:], corners[0])):
            rx, ry = b[0] - a[0], b[1] - a[1]
            cuts = [0.0, 1.0]
            for p, q in zip(polygon, (*polygon[1:], polygon[0])):
                dx, dy = q[0] - p[0], q[1] - p[1]
                denominator = rx * dy - ry * dx
                if abs(denominator) < 1e-12:
                    continue
                px, py = p[0] - a[0], p[1] - a[1]
                t = (px * dy - py * dx) / denominator
                u = (px * ry - py * rx) / denominator
                if 0 <= t <= 1 and 0 <= u <= 1:
                    cuts.append(t)
            cuts = sorted(set(cuts))
            samples = [*cuts, *((lo + hi) / 2 for lo, hi in zip(cuts, cuts[1:]))]
            worst = max(worst, *(self.outside_distance(a[0] + t * rx, a[1] + t * ry) for t in samples))
        return worst

    def cones(self):
        rng = np.random.default_rng(self.seed)
        cones = []
        for colour, side in ((ConeColour.BLUE, 1), (ConeColour.YELLOW, -1)):
            vertices = self.boundary_vertices(side)
            distance = 0.0
            for a, b in zip(vertices, vertices[1:]):
                length = math.dist(a, b)
                direction = ((b[0] - a[0]) / length, (b[1] - a[1]) / length)
                # Include each corner once, and vary spacing independently on each boundary.
                along = 0.0
                while along < length:
                    dx, dy = rng.uniform(-self.placement_jitter_m, self.placement_jitter_m, 2)
                    cones.append((colour, distance + along, a[0] + along * direction[0] + dx,
                                  a[1] + along * direction[1] + dy))
                    along += self.cone_spacing_m * rng.uniform(0.8, 1.2)
                distance += length
            dx, dy = rng.uniform(-self.placement_jitter_m, self.placement_jitter_m, 2)
            cones.append((colour, distance, vertices[-1][0] + dx, vertices[-1][1] + dy))
        return cones
