"""Offline-only three-row U-turn learner with observation-derived planning and bounded memory.

Only the steering regressor is learned. Sensor admission, acquisition-time odometry, row/end observation,
route expiry and cone/lidar rejection remain deterministic. No ROS selector, hardware calibration or upload.
The explicit finite-row/open-apron experiment permits bounded fresh-empty camera frames on an already observed
manoeuvre; this does NOT change the deployed corridor policy's empty-cone stop contract.
"""
from dataclasses import dataclass, replace
import math
from pathlib import Path

import numpy as np

from policy.action_policy.corridor import swept_lidar
from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones
from policy.control.actions import DriveCommand
from policy.lane_detection.lane_detection import LaneDetectionConfig
from policy.lane_detection.temporal import _body, _motion, _pose_at, _world
from policy.memory import CarMemory, ConeTrack, ThreeRowTurnMemory

FEATURE_VERSION = 1
FEATURE_COUNT = 17


@dataclass(frozen=True)
class ThreeRowConfig:
    """Offline experiment geometry, freshness, motion and bounded-route settings."""
    max_left_curvature_per_m: float = 1.0
    max_right_curvature_per_m: float = .92
    approach_speed_m_per_s: float = .4
    turn_speed_m_per_s: float = .32
    turning_radius_m: float = 1.18
    lookahead_m: float = .6
    endpoint_margin_m: float = .75
    track_max_age_s: float = 8.0
    route_max_age_s: float = 36.0
    route_max_distance_m: float = 12.0
    camera_max_age_s: float = .4
    lidar_max_age_s: float = .2
    empty_without_route_s: float = .8
    memory_enabled: bool = True

    def __post_init__(self):
        values = [value for name, value in vars(self).items() if name != "memory_enabled"]
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise ValueError("Three-row limits must be positive and finite")
        if self.turning_radius_m < self.minimum_planning_radius_m:
            raise ValueError("Planning radius exceeds the configured steering capability")
        if self.route_max_age_s > 40 or self.route_max_distance_m > 14:
            raise ValueError("Offline route hold must remain bounded")
        if self.lookahead_m != .6:
            raise ValueError("Feature version 1 requires the 0.6 m steering preview")
        if self.turn_speed_m_per_s > self.approach_speed_m_per_s:
            raise ValueError("Turning speed cannot exceed approach speed")

    @property
    def minimum_planning_radius_m(self):
        return max(1/self.max_left_curvature_per_m, 1/self.max_right_curvature_per_m)


class ThreeRowNetwork:
    """Validated NumPy steering-regression weights; no training or state is hidden in inference."""

    feature_version = FEATURE_VERSION

    @classmethod
    def contract(cls, config):
        return np.array([cls.feature_version, config.max_left_curvature_per_m,
                         config.max_right_curvature_per_m, config.lookahead_m])

    def __init__(self, path, config=ThreeRowConfig()):
        with np.load(Path(path), allow_pickle=False) as data:
            arrays = [data[name].copy() for name in ("mean", "scale", "w1", "b1", "w2", "b2", "w3", "b3")]
            expected = self.contract(config)
            if not np.array_equal(data["contract"], expected):
                raise ValueError("Three-row model contract does not match the controller")
        shapes = ((FEATURE_COUNT,), (FEATURE_COUNT,), (FEATURE_COUNT, 32), (32,),
                  (32, 32), (32,), (32, 1), (1,))
        if any(a.shape != shape or not np.isfinite(a).all() for a, shape in zip(arrays, shapes)):
            raise ValueError("Invalid three-row model weights")
        self.mean, self.scale, self.w1, self.b1, self.w2, self.b2, self.w3, self.b3 = arrays
        if np.any(self.scale <= 0):
            raise ValueError("Invalid three-row feature scale")
        self.config = config

    def predict(self, values):
        values = np.asarray(values, dtype=float)
        if values.shape != (FEATURE_COUNT,) or not np.isfinite(values).all():
            return None
        z = (values-self.mean)/self.scale
        if np.max(np.abs(z)) > 10:
            return None
        hidden = np.tanh(z@self.w1+self.b1)
        hidden = np.tanh(hidden@self.w2+self.b2)
        value = float((hidden@self.w3+self.b3)[0])
        return float(np.clip(value, -self.config.max_right_curvature_per_m,
                             self.config.max_left_curvature_per_m)) if math.isfinite(value) else None


def turn_path(start_u, lower_v, upper_v, radius, heading, end_u):
    """A forward R-L-R connector between observed parallel rows, not a simulator-provided route."""
    separation = upper_v-lower_v
    if not 0 < separation < 2*radius:
        return ()
    a = math.acos(.5+separation/(4*radius))
    x, y, h = start_u, lower_v, 0.
    points = [(x, y)]
    for curvature, angle in ((-1/radius, a), (1/radius, math.pi+2*a), (-1/radius, a)):
        length = angle/abs(curvature)
        count = max(1, math.ceil(length/.035))
        for _ in range(count):
            ds = length/count
            nh = h+curvature*ds
            x += (math.sin(nh)-math.sin(h))/curvature
            y += (math.cos(h)-math.cos(nh))/curvature
            h = nh
            points.append((x, y))
    for u in np.linspace(start_u, end_u, max(2, math.ceil((start_u-end_u)/.035)))[1:]:
        points.append((float(u), upper_v))
    c, s = math.cos(heading), math.sin(heading)
    return tuple((c*u-s*v, s*u+c*v) for u, v in points)


class ThreeRowPolicy:
    """Sensory planner plus either its analytic steering teacher or an offline learned student."""

    def __init__(self, config=ThreeRowConfig(), network=None, collector=None):
        self.config, self.network, self.collector = config, network, collector
        self.memory = CarMemory()
        self.motion_config = LaneDetectionConfig()
        self.last_features = None
        self.guard_interventions = 0
        self.path = ()
        self.max_empty_s = 0.

    @property
    def state(self):
        return self.memory.data.three_row_turn

    def _store(self, state):
        self.memory.update_three_row_turn(state)

    def _stop(self, reason, latch=True):
        state = self.state
        self._store(replace(state, stopped=state.stopped or latch, stop_reason=reason, previous_curvature=0.))
        return [DriveCommand(0., 0., brake=True)]

    def _observe(self, observations):
        state, cfg = self.state, self.config
        poses = _motion(observations, state, self.motion_config)
        if poses is None:
            return None, "invalid or stale motion"
        now = poses[-1]
        cones = observations.cones
        if cones is None and state.last_acquisition_s is None and now.time_s <= 1.:
            self._store(replace(state, poses=poses))
            return (), ""
        if cones is None or not math.isfinite(float(cones.sensor_age)) or not 0 <= cones.sensor_age <= cfg.camera_max_age_s:
            return None, "missing or stale cone stream"
        acquisition_s = now.time_s-float(cones.sensor_age)
        stamp = cones.sensor_age.stamp_ns
        if (state.last_acquisition_s is not None and acquisition_s <= state.last_acquisition_s+1e-8
                or stamp is not None and state.last_stamp_ns is not None and stamp <= state.last_stamp_ns):
            return None, "repeated or out-of-order cone frame"
        pose = _pose_at(poses, acquisition_s)
        if pose is None:
            self._store(replace(state, poses=poses))
            return (), ""
        cones = filter_cones(cones, ConeFilterConfig(check_straight_rows=False))
        tracks = [t for t in state.tracks if now.time_s-t.seen_s <= cfg.track_max_age_s] if cfg.memory_enabled else []
        fresh = []
        for cone in cones:
            x, y = _world(cone.pos.x, cone.pos.y, pose)
            match = min(((math.hypot(t.x-x, t.y-y), i) for i, t in enumerate(tracks)
                         if t.colour == cone.colour.name), default=(math.inf, -1))
            if match[0] < .18:
                tracks.pop(match[1])
            fresh.append(ConeTrack(x, y, cone.pos.z, cone.colour.name, cone.confidence,
                                   acquisition_s, pose.distance_m))
        tracks = tuple((fresh+tracks)[:128])
        nonempty = acquisition_s if cones else state.last_nonempty_s
        if not cfg.memory_enabled:
            state = replace(state, row_heading=None, row_offsets=(), row_ends=(), row_end_seen_s=(),
                            observed_start=None, observed_start_xy=None, observed_start_seen_s=None,
                            row_end_xy=(), route=(), route_index=0, route_seen_s=None,
                            route_seen_distance_m=None, startup_heading=None, startup_since_s=None,
                            startup_seen_s=None, startup_support=(), startup_samples=0,
                            planned_row_ends=(), turn_start_u=None, cap_u=None, cap_v=None,
                            cap_radius_m=None, cap_seen_s=None)
        self._store(replace(state, poses=poses, tracks=tracks, last_stamp_ns=stamp,
                            last_acquisition_s=acquisition_s, last_nonempty_s=nonempty))
        return (now, pose, cones, fresh), ""

    def _rows(self, now, pose, cones):
        state = self.state
        middle = [t for t in state.tracks if t.colour == "YELLOW"]
        if len(middle) < 2 or np.ptp([t.x for t in middle]) < .6:
            return
        xy = np.array([(t.x, t.y) for t in middle])
        slope = np.polyfit(xy[:, 0], xy[:, 1], 1)[0]
        if len(middle) == 2:
            if state.row_heading is None:
                slope = self._sparse_startup(now, pose, cones)
                if slope is None:
                    return
                state = self.state
            else:
                slope = math.tan(state.row_heading)
        theta = math.atan(float(slope))
        if abs(theta) > .35:
            return
        if state.row_heading is not None:
            theta = state.row_heading
        c, s = math.cos(theta), math.sin(theta)
        project = lambda t: (c*t.x+s*t.y, -s*t.x+c*t.y)
        mid = float(np.median([project(t)[1] for t in middle]))
        groups = [[t for t in state.tracks if t.colour == "BLUE" and project(t)[1] < mid-.3],
                  middle,
                  [t for t in state.tracks if t.colour == "BLUE" and project(t)[1] > mid+.3]]
        offsets = tuple(float(np.median([project(t)[1] for t in group])) if group else math.nan for group in groups)
        first = min(state.tracks, key=lambda t: project(t)[0])
        start, start_xy, start_seen = state.observed_start, state.observed_start_xy, state.observed_start_seen_s
        if start is None or project(first)[0] < start:
            start, start_xy, start_seen = project(first)[0], (first.x, first.y), first.seen_s
        ends = list(state.row_ends or (-math.inf,)*3)
        seen = list(state.row_end_seen_s or (-math.inf,)*3)
        endpoints = list(state.row_end_xy or ((math.nan, math.nan),)*3)
        for i, group in enumerate(groups):
            if len(group) < 2 or np.ptp([project(t)[0] for t in group]) < .35:
                continue
            terminal = max(group, key=lambda t: project(t)[0])
            bx, by = _body(terminal.x, terminal.y, pose)
            # Endpoint evidence is admitted only away from range/FOV truncation. The planner also
            # reserves 0.75 m for uncertain terminal geometry; no hidden row ID is used.
            if .4 <= bx <= 2.05 and abs(math.atan2(by, bx)) <= math.radians(40):
                u = project(terminal)[0]
                if u >= ends[i]-.08:
                    if u >= ends[i]:
                        endpoints[i] = (terminal.x, terminal.y)
                    ends[i], seen[i] = max(u, ends[i]), terminal.seen_s
        self._store(replace(state, row_heading=theta, row_offsets=offsets, row_ends=tuple(ends),
                            row_end_seen_s=tuple(seen), row_end_xy=tuple(endpoints), observed_start=start,
                            observed_start_xy=start_xy, observed_start_seen_s=start_seen))

    def _sparse_startup(self, now, pose, cones):
        """Two divider points need a persistent independent parallel boundary before first motion."""
        state = self.state
        def reset():
            self._store(replace(state, startup_since_s=None, startup_seen_s=None,
                                startup_support=(), startup_samples=0, startup_heading=None))
            return None
        if state.has_driven or now.speed > .03 or abs(now.yaw_rate) > .05:
            return reset()
        yellow = np.array([(t.x, t.y) for t in state.tracks if t.colour == "YELLOW"
                           and 0 <= pose.time_s-t.seen_s <= .4])
        if len(yellow) != 2 or np.ptp(yellow[:, 0]) < .6:
            return reset()
        if not any(c.colour.name == "YELLOW" for c in cones):
            return None
        blue = np.array([_world(c.pos.x, c.pos.y, pose) for c in cones if c.colour.name == "BLUE"
                         and _world(c.pos.x, c.pos.y, pose)[1] < np.median(yellow[:, 1])-.3])
        if len(blue) < 3 or np.ptp(blue[:, 0]) < .8:
            return reset()
        for group in (yellow, blue):
            distances = np.linalg.norm(group[:, None]-group[None, :], axis=2)+np.eye(len(group))*100
            if np.min(distances) < .18:
                return reset()
        yellow_fit, blue_fit = (np.polyfit(group[:, 0], group[:, 1], 1) for group in (yellow, blue))
        if (abs(math.atan(float(blue_fit[0]))-math.atan(float(yellow_fit[0]))) > .12
                or np.max(np.abs(blue[:, 1]-np.polyval(blue_fit, blue[:, 0]))) > .08):
            return reset()
        centred = np.concatenate((yellow-yellow.mean(0), blue-blue.mean(0)))
        slope = float(centred[:, 0]@centred[:, 1]/(centred[:, 0]@centred[:, 0]))
        theta = math.atan(slope)
        gap = (yellow_fit[1]-blue_fit[1])*math.cos(theta)
        if not .7 <= gap <= 1.35:
            return reset()
        # Persistence uses fresh acquisitions of the same distinct local positions. Missing frames,
        # motion or inconsistent heading reset the one-second stationary observation interval.
        support = tuple(map(tuple, np.concatenate((yellow, blue))))
        consistent = (state.startup_seen_s is not None and 0 < pose.time_s-state.startup_seen_s <= .21
                      and abs(theta-state.startup_heading) <= .06 and state.startup_support
                      and all(min(math.hypot(x-a, y-b) for a, b in state.startup_support) < .15
                              for x, y in support))
        since = state.startup_since_s if consistent else pose.time_s
        samples = state.startup_samples+1 if consistent else 1
        theta = (state.startup_heading*(samples-1)+theta)/samples if consistent else theta
        self._store(replace(state, startup_since_s=since, startup_seen_s=pose.time_s,
                            startup_heading=theta, startup_support=support, startup_samples=samples))
        return math.tan(theta) if pose.time_s-since >= 1. and samples >= 8 else None

    def _plan(self, now):
        state, cfg = self.state, self.config
        if len(state.row_offsets) != 3 or not all(math.isfinite(v) for v in state.row_offsets):
            return
        lower, mid, upper = state.row_offsets
        if not .7 <= mid-lower <= 1.35 or not .7 <= upper-mid <= 1.35:
            return
        theta = state.row_heading
        c, s = math.cos(theta), math.sin(theta)
        u_now = c*now.x+s*now.y
        if state.route:
            # Newly observed endpoints can disprove a truncated initial view. Replace the entire
            # approach/turn only before steering into it, with new evidence from all three rows.
            heading_error = math.atan2(math.sin(now.yaw-theta), math.cos(now.yaw-theta))
            if (state.turn_start_u is None or u_now > state.turn_start_u-.15
                    or abs(heading_error) > .15 or abs(state.previous_curvature) > .25
                    or len(state.planned_row_ends) != 3
                    or not any(new > old+.15 for new, old in zip(state.row_ends, state.planned_row_ends))
                    or not all(seen > state.route_seen_s for seen in state.row_end_seen_s)):
                return
        if (not all(math.isfinite(v) for v in state.row_ends)
                or max(now.time_s-t for t in state.row_end_seen_s) > 5.
                or u_now < state.row_ends[1]-.35):
            return
        start_u = max(state.row_ends[0], state.row_ends[2])+cfg.endpoint_margin_m
        lower_v, upper_v = (lower+mid)/2, (upper+mid)/2
        prefix = tuple((c*u-s*lower_v, s*u+c*lower_v)
                       for u in np.arange(u_now, start_u, .035))
        route = (*prefix, *turn_path(start_u, lower_v, upper_v, cfg.turning_radius_m,
                                     theta, state.observed_start+.12))
        self._store(replace(state, route=route, route_seen_s=now.time_s,
                            route_seen_distance_m=now.distance_m, route_index=0,
                            planned_row_ends=state.row_ends, turn_start_u=start_u))

    def _fresh_return(self, now, pose, cones):
        state = self.state
        if not state.route or len(state.row_offsets) != 3:
            return
        theta = state.row_heading
        error = abs(math.atan2(math.sin(now.yaw-theta-math.pi), math.cos(now.yaw-theta-math.pi)))
        if error > .3:
            return
        c, s = math.cos(theta), math.sin(theta)
        u_now, v_now = c*now.x+s*now.y, -s*now.x+c*now.y
        target = sum(state.row_offsets[1:])/2
        if abs(v_now-target) > .2:
            return
        supports = {"YELLOW": [], "BLUE": []}
        for cone in cones:
            x, y = _world(cone.pos.x, cone.pos.y, pose)
            u, v = c*x+s*y, -s*x+c*y
            expected_v = state.row_offsets[1 if cone.colour.name == "YELLOW" else 2]
            if abs(v-expected_v) < .13:
                supports[cone.colour.name].append(u)
        if any(len(row) < 2 or max(row)-min(row) < .35 for row in supports.values()):
            return
        end = max(min(row) for row in supports.values())
        if u_now-end < .45:
            return
        # This is new, two-boundary, spatially spread return-lane evidence. A surviving single row
        # or a predicted frame must never renew the old turn anchor.
        route = tuple((c*u-s*target, s*u+c*target)
                      for u in np.linspace(u_now, end, max(2, math.ceil((u_now-end)/.035))))
        self._store(replace(state, route=route, route_index=0, route_seen_s=pose.time_s,
                            route_seen_distance_m=pose.distance_m))

    def _preview(self, now, cones):
        state, cfg = self.state, self.config
        if state.route:
            route = np.array(state.route)
            low = state.route_index
            high = min(len(route), low+45)
            index = low+int(np.argmin(np.sum((route[low:high]-[now.x, now.y])**2, axis=1)))
            self._store(replace(state, route_index=index))
            route = route[index:]
            age = now.time_s-state.route_seen_s
            travel = now.distance_m-state.route_seen_distance_m
        else:
            if len(state.row_offsets) < 2 or not all(math.isfinite(v) for v in state.row_offsets[:2]):
                return None
            c, s = math.cos(state.row_heading), math.sin(state.row_heading)
            v = sum(state.row_offsets[:2])/2
            u = c*now.x+s*now.y
            extent = max((c*t.x+s*t.y for t in state.tracks), default=u)
            if extent-u < .65:
                return None
            route = np.array([(c*p-s*v, s*p+c*v) for p in np.arange(u, extent+.01, .035)])
            age, travel = 0., 0.
        if len(route) < 2:
            return None
        distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
        if distances[-1] < .12:
            return None
        points = []
        for d in (.3, .6, .9, 1.2):
            x, y = (float(np.interp(d, distances, route[:, k])) for k in (0, 1))
            points.extend(_body(x, y, now))
        divider = sum(cone.colour.name == "YELLOW" for cone in cones)
        outer_left = sum(cone.colour.name == "BLUE" and cone.pos.y > 0 for cone in cones)
        outer_right = sum(cone.colour.name == "BLUE" and cone.pos.y <= 0 for cone in cones)
        empty = 0. if state.last_nonempty_s is None else now.time_s-state.last_nonempty_s
        self.max_empty_s = max(self.max_empty_s, empty)
        values = np.array([*points, state.previous_curvature, now.speed, now.yaw_rate,
                           age/cfg.route_max_age_s, travel/cfg.route_max_distance_m,
                           min(divider, 8)/8, min(outer_left, 8)/8, min(outer_right, 8)/8, empty/20])
        self.path = tuple(_body(float(x), float(y), now) for x, y in route)
        return values

    def _guard_speed(self, curvature, observations):
        return max(float(observations.car.wheel_speed), self.config.approach_speed_m_per_s)

    def _safe(self, curvature, observations):
        lidar = observations.lidar_cartesian
        if (lidar is None or not math.isfinite(float(lidar.sensor_age))
                or not 0 <= lidar.sensor_age <= self.config.lidar_max_age_s):
            return False
        speed = self._guard_speed(curvature, observations)
        length = speed*(.6+.25+float(lidar.sensor_age))+.2
        now = self.state.poses[-1]
        scan_pose = _pose_at(self.state.poses, now.time_s-float(lidar.sensor_age))
        if scan_pose is None:
            return False
        points = [_body(*_world(p.pos.x, p.pos.y, scan_pose), now) for p in lidar.points]
        if not all(math.isfinite(v) for point in points for v in point):
            return False
        initial = now.yaw_rate/now.speed if now.speed > .05 else self.state.previous_curvature
        settle = speed*.15
        if swept_lidar(points, curvature, .3, .3, .25, .075, length, 1,
                       initial_curvature=initial, settle_distance_m=settle)[0] <= length:
            return False
        reaction = speed*(.25+float(lidar.sensor_age))
        delays = np.linspace(0., reaction, max(2, math.ceil(reaction/.025)+1))
        half_step = float(delays[1]-delays[0])/2
        # Bounded reaction-then-centred-steering stopping, including measured initial curvature,
        # spatial first-order settling and the between-delay interpolation envelope.
        for delay in delays:
            distance = float(delay+speed*.6+half_step)
            if swept_lidar(points, curvature, .3, .3, .25, .075, distance, 1,
                           initial_curvature=initial, settle_distance_m=settle,
                           brake_after_m=float(delay), delay_uncertainty_m=half_step)[0] <= distance:
                return False
        return True

    @staticmethod
    def teacher(values):
        x, y = values[2:4]
        return 2*y/max(.04, x*x+y*y)

    def run_policy(self, observations, lanes=None):
        cfg = self.config
        if observations.policy.is_first_policy_step:
            self._store(ThreeRowTurnMemory())
        if self.state.stopped:
            return self._stop(self.state.stop_reason)
        observed, error = self._observe(observations)
        if observed is None:
            return self._stop(error)
        if not observed:
            return [DriveCommand(0., 0.)]
        now, pose, cones, fresh = observed
        if self.state.route:
            if (now.time_s-self.state.route_seen_s > cfg.route_max_age_s
                    or now.distance_m-self.state.route_seen_distance_m > cfg.route_max_distance_m):
                return self._stop("observed route expired")
            if not cfg.memory_enabled and not cones:
                return self._stop("cone memory disabled")
        elif not cones and (not cfg.memory_enabled or self.state.last_nonempty_s is None
                            or now.time_s-self.state.last_nonempty_s > cfg.empty_without_route_s):
            return self._stop("no observed route during cone loss", latch=self.state.has_driven)
        self._rows(now, pose, cones)
        self._plan(now)
        if self.state.stopped:
            return self._stop(self.state.stop_reason)
        self._fresh_return(now, pose, cones)
        values = self._preview(now, cones)
        if values is None:
            return self._stop("waiting for supported path", latch=self.state.has_driven)
        self.last_features = values.copy()
        teacher = float(np.clip(self.teacher(values), -cfg.max_right_curvature_per_m, cfg.max_left_curvature_per_m))
        if self.collector is not None:
            self.collector(values.copy(), teacher)
        requested = teacher if self.network is None else self.network.predict(values)
        if requested is None:
            return self._stop("invalid or out-of-distribution model inference")
        change = 4.*observations.policy.dt
        k = float(np.clip(requested, self.state.previous_curvature-change, self.state.previous_curvature+change))
        if not self._safe(k, observations):
            self.guard_interventions += 1
            return self._stop("lidar or braking footprint blocked")
        self._store(replace(self.state, previous_curvature=k, stop_reason="", has_driven=True))
        speed = cfg.turn_speed_m_per_s if abs(k) > .25 else cfg.approach_speed_m_per_s
        return [DriveCommand(speed, k)]
