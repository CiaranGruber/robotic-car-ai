"""Confirmed course: two blue straight rows continue beyond the divider, then join through a blue cap.

The continuous outer scoring boundary is closed. No car body may escape between blue cones or use
an apron. Inherit the same camera, acquisition latency, physics, physical contact and motion sensors.
"""
from dataclasses import dataclass
import math

import numpy as np

from policy.input_output import ConeColour
from tests.policy.three_row_simulation import ThreeRowTrack, ThreeRowCase, simulate


@dataclass(frozen=True)
class ConnectedUTurnTrack:
    """Straight rows plus a connected curved outer boundary and a continuous middle divider."""
    divider_length_m: float = 3.
    row_gap_m: float = 1.
    outer_extension_m: float = 1.
    cone_spacing_m: float = .5
    placement_jitter_m: float = .025
    placement_seed: int = 17
    removed_fraction: float = 0.
    remove_divider_tip: bool = False
    remove_cap_cone: bool = False
    cap_depth_scale: float = 1.

    def __post_init__(self):
        self._straight_track()  # Share placement validation; no apron is part of this course.
        if not math.isfinite(self.cap_depth_scale) or not .5 <= self.cap_depth_scale <= 2.:
            raise ValueError("Cap depth scale must be finite and between 0.5 and 2")

    def _straight_track(self):
        return ThreeRowTrack(**{name: value for name, value in vars(self).items()
                                if name not in ("remove_cap_cone", "cap_depth_scale")})

    @property
    def outer_end_m(self):
        return self.divider_length_m+self.outer_extension_m

    @property
    def furthest_blue_x_m(self):
        return self.outer_end_m+self.row_gap_m*self.cap_depth_scale

    def row_cones(self):
        cones = self._straight_track().row_cones()
        rng = np.random.default_rng(self.placement_seed+789)
        segments = max(6, math.ceil(math.pi*self.row_gap_m/self.cone_spacing_m))
        for i, angle in enumerate(np.linspace(-math.pi/2, math.pi/2, segments+1)[1:-1]):
            if self.remove_cap_cone and i == segments//2-1:
                continue
            dx, dy = rng.uniform(-self.placement_jitter_m, self.placement_jitter_m, 2)
            cones.append((3, ConeColour.BLUE, self.outer_end_m+self.cap_depth_scale*self.row_gap_m*math.cos(angle)+dx,
                          self.row_gap_m/2+self.row_gap_m*math.sin(angle)+dy))
        return cones

    def cones(self):
        return [(colour, float(row), x, y) for row, colour, x, y in self.row_cones()]

    def boundary(self):
        low, high = -self.row_gap_m/2, 1.5*self.row_gap_m
        points = [(-1.2, low), (self.outer_end_m, low)]
        for angle in np.linspace(-math.pi/2, math.pi/2, 100)[1:]:
            points.append((self.outer_end_m+self.cap_depth_scale*self.row_gap_m*math.cos(angle),
                           self.row_gap_m/2+self.row_gap_m*math.sin(angle)))
        return np.array([*points, (-1.2, high)])

    def rectangles(self):
        # Inherited rectangle consumers must never accidentally recreate the old open apron.
        w, end = self.row_gap_m, self.divider_length_m
        return ((-1.2, end, -w/2, w/2-1e-6), (-1.2, end, w/2+1e-6, 1.5*w),
                (end, self.outer_end_m, -w/2, 1.5*w))

    def outside_distance(self, x, y):
        w = self.row_gap_m
        if x >= self.outer_end_m:
            outside = (math.hypot((x-self.outer_end_m)/self.cap_depth_scale, y-w/2)-w)*min(1., self.cap_depth_scale)
        else:
            outside = max(-1.2-x, -w/2-y, y-1.5*w)
        if -1.2 <= x <= self.divider_length_m and abs(y-w/2) < 1e-6:
            outside = max(outside, 1e-6-abs(y-w/2))
        return outside

    def footprint_outside_distance(self, corners):
        # The outer enclosure is convex, so all corners inside implies all edges inside. Add
        # explicit cuts through the thin divider; checking only body corners would miss crossing it.
        worst = max(self.outside_distance(x, y) for x, y in corners)
        for a, b in zip(corners, np.roll(corners, -1, axis=0)):
            delta = b-a
            cuts = [0., 1.]
            for dim, boundaries in ((0, (-1.2, self.divider_length_m)),
                                    (1, (self.row_gap_m/2-1e-6, self.row_gap_m/2, self.row_gap_m/2+1e-6))):
                if abs(delta[dim]) > 1e-12:
                    cuts.extend(t for boundary in boundaries if 0 < (t := (boundary-a[dim])/delta[dim]) < 1)
            worst = max(worst, *(self.outside_distance(*(a+t*delta)) for t in cuts))
        return worst
