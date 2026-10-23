"""Offline closed-blue-boundary U-turn experiment; no ROS selector or vehicle calibration changes.

Keep the existing straight-row observer and motion-compensated memory. Fit the connected blue cap
from sensor tracks, then form a left semicircle INSIDE it. The default controller computes analytic
steering from current odometry and lookahead on every tick; no checkpoint or online learning is used.
An optional network remains solely for the earlier offline comparison. The formed turn is retained
through occlusion; this is feedback control, not continuous rebuilding of the committed path.
The default 1/m steering assumption is geometrically infeasible; larger limits are explicitly
hypothetical offline sensitivity settings, never evidence of physical steering capability.
"""
from dataclasses import dataclass, replace
import math

import numpy as np

from policy.action_policy.three_row_uturn import ThreeRowConfig, ThreeRowNetwork, ThreeRowPolicy
from policy.lane_detection.temporal import _pose_at


def minimum_strip_width(curvature, front=.3, rear=.3, width=.25):
    """Necessary width for any forward heading reversal, including countersteering, without padding.

    Between the last heading 0 and first heading pi, sin(theta)>=0 and ds>=dtheta/k.
    Add the rectangle's lateral support at intermediate headings; optimizing those headings gives
    the two square roots. This lower bound does not establish fit inside a finite cap or past a divider.
    """
    if not math.isfinite(curvature) or curvature <= 0:
        raise ValueError("Curvature must be positive and finite")
    radius = 1/curvature
    return math.hypot(radius+width/2, rear)+math.hypot(radius+width/2, front)


@dataclass(frozen=True)
class ConnectedTurnConfig(ThreeRowConfig):
    """Offline closed-turn geometry and assumed controller limits."""
    turning_radius_m: float = 1.
    approach_speed_m_per_s: float = .3
    turn_speed_m_per_s: float = .15
    route_max_age_s: float = 36.
    route_max_distance_m: float = 12.
    camera_empty_hold_s: float = 3.
    turn_inset_m: float = .35

    @property
    def minimum_planning_radius_m(self):
        return 1/self.max_left_curvature_per_m


class ConnectedTurnNetwork(ThreeRowNetwork):
    """Closed-cap weights have a different feature/geometry contract from open-apron weights."""
    feature_version = 2

    @classmethod
    def contract(cls, config):
        return np.r_[super().contract(config), config.turning_radius_m, config.turn_inset_m,
                     config.approach_speed_m_per_s, config.turn_speed_m_per_s,
                     config.route_max_age_s, config.route_max_distance_m, config.camera_empty_hold_s]


def inside_turn_path(start_u, start_v, centre_u, centre_v, radius, heading, end_u, return_v):
    """Smooth approach, forward left semicircle and return, expressed in the observed row frame."""
    if centre_u-start_u < .3:
        return ()
    points = []
    count = max(2, math.ceil((centre_u-start_u)/.03))
    for t in np.linspace(0., 1., count):
        blend = t*t*(3-2*t)
        points.append((start_u+(centre_u-start_u)*t, start_v+(centre_v-radius-start_v)*blend))
    for angle in np.linspace(0., math.pi, math.ceil(math.pi*radius/.03)+1)[1:]:
        points.append((centre_u+radius*math.sin(angle), centre_v-radius*math.cos(angle)))
    for u in np.arange(centre_u-.03, end_u-.01, -.03):
        t = min(1., (centre_u-u)/1.2)
        blend = t*t*(3-2*t)
        points.append((float(u), centre_v+radius+(return_v-centre_v-radius)*blend))
    c, s = math.cos(heading), math.sin(heading)
    return tuple((c*u-s*v, s*u+c*v) for u, v in points)


class ConnectedTurnPolicy(ThreeRowPolicy):
    """Online geometric feedback by default; an optional network is an offline comparison only."""

    def __init__(self, config=ConnectedTurnConfig(), network=None, collector=None):
        super().__init__(config, network, collector)

    def _rows(self, now, pose, cones):
        state = self.state
        tracks = state.tracks
        locked_offsets = state.row_offsets if state.route else None
        filter_heading, filter_offsets = state.row_heading, state.row_offsets
        start_backed = (state.observed_start is None or state.observed_start_xy is not None
                        and all(math.isfinite(v) for v in state.observed_start_xy))
        ends_backed = (not state.row_ends or len(state.row_end_xy) == 3
                       and all(not math.isfinite(end) or all(math.isfinite(v) for v in point)
                               for end, point in zip(state.row_ends, state.row_end_xy)))
        backed_geometry = start_backed and ends_backed
        if state.row_heading is not None and not state.route and backed_geometry:
            middle = np.array([(t.x, t.y) for t in tracks if t.colour == "YELLOW"])
            if len(middle) >= 4 and np.ptp(middle[:, 0]) >= 1.:
                fit = np.polyfit(middle[:, 0], middle[:, 1], 1)
                residual = middle[:, 1]-np.polyval(fit, middle[:, 0])
                heading = math.atan(float(fit[0]))
                if np.max(np.abs(residual)) < .12 and abs(heading) < .35:
                    # Refine a noisy short startup fit as more divider is actually observed.
                    # Limit each change; freeze the observed frame when the turn route is formed.
                    refined = state.row_heading+float(np.clip(heading-state.row_heading, -.005, .005))
                    c_new, s_new = math.cos(refined), math.sin(refined)
                    ends = tuple(c_new*x+s_new*y if math.isfinite(x) and math.isfinite(y) else old
                                 for (x, y), old in zip(state.row_end_xy, state.row_ends))
                    start = state.observed_start
                    if state.observed_start_xy is not None:
                        start = c_new*state.observed_start_xy[0]+s_new*state.observed_start_xy[1]
                    state = replace(state, row_heading=refined, cap_u=None, cap_v=None,
                                    cap_radius_m=None, cap_seen_s=None, row_ends=ends, observed_start=start)
                    self._store(state)
        if (filter_heading is not None and len(filter_offsets) == 3
                and all(math.isfinite(v) for v in filter_offsets[:2])):
            theta = filter_heading
            c, s = math.cos(theta), math.sin(theta)
            low, middle = filter_offsets[:2]
            high = filter_offsets[2] if math.isfinite(filter_offsets[2]) else 2*middle-low
            # Curved blue observations must not pull the existing straight-row fitter inward.
            straight = tuple(t for t in tracks if t.colour == "YELLOW"
                             or min(abs(-s*t.x+c*t.y-low), abs(-s*t.x+c*t.y-high)) < .13)
            self._store(replace(state, tracks=straight))
        super()._rows(now, pose, cones)
        self._store(replace(self.state, tracks=tracks,
                            row_offsets=locked_offsets if locked_offsets is not None else self.state.row_offsets))
        state = self.state
        if len(state.row_offsets) != 3 or not all(math.isfinite(v) for v in state.row_offsets):
            return
        low, middle, high = state.row_offsets
        if not .7 <= middle-low <= 1.35 or not .7 <= high-middle <= 1.35:
            return
        c, s = math.cos(state.row_heading), math.sin(state.row_heading)
        centre, radius = (low+high)/2, (high-low)/2
        candidates = []
        for track in tracks:
            u, v = c*track.x+s*track.y, -s*track.x+c*track.y
            if track.colour != "BLUE" or now.time_s-track.seen_s > .8 or abs(v-centre) > .8*radius:
                continue
            candidates.append((u-math.sqrt(radius*radius-(v-centre)**2), u, v, track.seen_s))
        if len(candidates) < 3:
            return
        cap_u = float(np.median([point[0] for point in candidates]))
        inliers = [p for p in candidates if abs(math.hypot(p[1]-cap_u, p[2]-centre)-radius) < .09
                   and p[1] > cap_u]
        if len(inliers) < 3 or np.ptp([p[2] for p in inliers]) < .4:
            return
        if any(math.hypot(a[1]-b[1], a[2]-b[2]) < .18
               for i, a in enumerate(inliers) for b in inliers[i+1:]):
            return
        # Keep original acquisition ages. A predicted frame cannot make a cap observation newer.
        seen = min(p[3] for p in inliers)
        self._store(replace(state, cap_u=cap_u, cap_v=centre, cap_radius_m=radius, cap_seen_s=seen))

    def _plan(self, now):
        state, cfg = self.state, self.config
        if (state.route and state.last_nonempty_s is not None
                and now.time_s-state.last_nonempty_s > cfg.camera_empty_hold_s):
            self._stop("empty cone stream beyond closed-turn hold")
            return
        if state.route or len(state.row_offsets) != 3 or not all(math.isfinite(v) for v in state.row_offsets):
            return
        low, middle, high = state.row_offsets
        if minimum_strip_width(max(cfg.max_left_curvature_per_m, cfg.max_right_curvature_per_m)) > high-low:
            self._stop("closed boundary too narrow for steering limit")
            return
        if state.cap_u is None or now.time_s-state.cap_seen_s > .8:
            return
        # This radial body-envelope check is a planning admission bound, in addition to lidar and
        # reaction/braking checks. Physical cone radius .05m and guard padding .075m stay explicit.
        if math.hypot(cfg.turning_radius_m+.125, .3)+.125 > state.cap_radius_m:
            self._stop("observed blue cap cannot contain planned turn footprint")
            return
        c, s = math.cos(state.row_heading), math.sin(state.row_heading)
        u = c*now.x+s*now.y
        turn_u = state.cap_u-cfg.turn_inset_m
        if len(state.row_ends) == 3 and turn_u < state.row_ends[1]+.4:
            self._stop("insufficient observed clearance past divider tip")
            return
        # Use the clear straight extension instead of pushing the car up against the curved cap.
        # Translating the semicircle left preserves the strip envelope and increases cap clearance.
        route = inside_turn_path(u, (low+middle)/2, turn_u, state.cap_v,
                                 cfg.turning_radius_m, state.row_heading, state.observed_start+.12,
                                 (middle+high)/2)
        if not route:
            self._stop("blue cap observed too late to form approach")
            return
        cap_pose = _pose_at(state.poses, state.cap_seen_s)
        if cap_pose is None:
            self._stop("cap acquisition pose unavailable")
            return
        self._store(replace(state, route=route, route_index=0, route_seen_s=state.cap_seen_s,
                            route_seen_distance_m=cap_pose.distance_m, turn_start_u=turn_u))

    def _guard_speed(self, curvature, observations):
        # Match the NEXT admitted speed, conservatively including current measured speed. This
        # includes lag during slowdown; it never budgets a .15m/s stop while the car still moves .3m/s.
        requested = self.config.turn_speed_m_per_s if abs(curvature) > .25 else self.config.approach_speed_m_per_s
        time_s = observations.policy.policy_elapsed_s
        pending = [speed for time, speed in self.state.speed_commands if 0 <= time_s-time <= .35]
        return max(float(observations.car.wheel_speed), requested, *pending)

    def run_policy(self, observations, lanes=None):
        commands = super().run_policy(observations, lanes)
        time_s = observations.policy.policy_elapsed_s
        if math.isfinite(time_s):
            history = [(t, speed) for t, speed in self.state.speed_commands if 0 <= time_s-t <= .35]
            bucket = math.floor(time_s/.05)*.05
            speed = commands[0].speed_m_per_s
            if history and abs(history[-1][0]-bucket) < 1e-8:
                history[-1] = (bucket, max(history[-1][1], speed))
            else:
                history.append((bucket, speed))
            # Fixed .05s buckets retain maxima, including .2s actuator delay plus timer/bucket
            # margins. At most nine entries; a high command cannot be lost at faster call rates.
            self._store(replace(self.state, speed_commands=tuple(history)))
        return commands

    def _fresh_return(self, now, pose, cones):
        previous = self.state
        super()._fresh_return(now, pose, cones)
        if previous.route and self.state.route and previous.row_heading is not None:
            c, s = math.cos(previous.row_heading), math.sin(previous.row_heading)
            old_end = c*previous.route[-1][0]+s*previous.route[-1][1]
            new_end = c*self.state.route[-1][0]+s*self.state.route[-1][1]
            if new_end > old_end+.05:
                # A short fresh prefix must not discard the already observed terminal route.
                # Retain its ORIGINAL age/distance anchor; never renew the unsupported tail.
                self._store(previous)
