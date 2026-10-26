"""Opt-in sensed, equal-width orthogonal junction navigator using the existing MPC and control map.

The incoming width/frame comes from cones; road walls and closing rows come from current lidar.
Those observations admit a finite route under an explicit orthogonal/equal-width junction prior. The
marker selects its exit, never its location. Motion history transports camera detections at their
acquisition time. A committed choice survives short occlusion but cannot renew its own evidence.
"""
from dataclasses import dataclass, replace
import math

import numpy as np

from policy.action_policy.mpc import MpcController, PathTrackingState
from policy.control.actions import DriveCommand
from policy.intersection_state import IntersectionCone, IntersectionMemory, IntersectionPose
from policy.lane_detection.selection import SelectedIntersectionLane, select_intersection_lane
from policy.markers.commands import MarkerCommand
from policy.memory import MpcStop


@dataclass(frozen=True)
class IntersectionConfig:
    """Admission, motion and lifetime limits for the experimental junction navigator."""
    enabled: bool = False
    turning_radius_m: float = 2.5
    turn_speed_m_per_s: float = .4
    sensor_timeout_s: float = .3
    camera_empty_hold_s: float = 4.
    command_max_age_s: float = 12.
    route_max_age_s: float = 24.
    route_max_distance_m: float = 10.

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError("intersection.enabled must be boolean")
        for name, value in vars(self).items():
            if name != "enabled" and (not math.isfinite(value) or value <= 0.):
                raise ValueError(f"intersection.{name} must be positive and finite")
        if not .5 <= self.turning_radius_m <= 8.:
            raise ValueError("intersection.turning_radius_m must be between .5 and 8 metres")


def pose_at(poses, stamp):
    if not poses or stamp < poses[0].time_s-1e-6 or stamp > poses[-1].time_s+1e-6:
        return None
    for a, b in zip(poses, poses[1:]):
        if a.time_s <= stamp <= b.time_s:
            ratio = (stamp-a.time_s)/(b.time_s-a.time_s)
            return IntersectionPose(stamp, *(getattr(a, key)+(getattr(b, key)-getattr(a, key))*ratio
                      for key in ("x", "y", "heading", "distance_m", "speed", "yaw_rate")))
    return poses[-1] if stamp >= poses[-1].time_s-1e-6 else poses[0]


def world_xy(pose, x, y):
    c, s = math.cos(pose.heading), math.sin(pose.heading)
    return pose.x+c*x-s*y, pose.y+s*x+c*y


def frame_xy(state, x, y):
    c, s = math.cos(state.frame_heading), math.sin(state.frame_heading)
    return c*x+s*y, -s*x+c*y-state.centre_v


def body_corners(x, y, heading, front=.3, rear=.3, width=.25):
    local = np.array(((front, width/2), (front, -width/2), (-rear, -width/2), (-rear, width/2)))
    c, s = math.cos(heading), math.sin(heading)
    return local @ np.array(((c, s), (-s, c)))+np.array((x, y))


def cross_fits(centre_u, width, radius, command, front, rear, car_width, clearance=.10, exits=None):
    exits = (MarkerCommand.TURN_LEFT, MarkerCommand.TURN_RIGHT, MarkerCommand.STRAIGHT) if exits is None else exits
    if command not in exits:
        return False
    if command is MarkerCommand.STRAIGHT:
        return width/2 > car_width/2+clearance
    sign = 1. if command is MarkerCommand.TURN_LEFT else -1.
    for a in np.linspace(0., math.pi/2, 100):
        corners = body_corners(centre_u-radius+radius*math.sin(a), sign*radius*(1-math.cos(a)),
                               sign*a, front, rear, car_width+2*clearance)
        # For a convex rectangle, all boundary intervals must stay in the non-convex cross.
        for p, q in zip(corners, np.roll(corners, -1, axis=0)):
            for t in np.linspace(0., 1., 15):
                x, y = p+t*(q-p)
                incoming = abs(y) <= width/2 and (MarkerCommand.STRAIGHT in exits or x <= centre_u+width/2)
                outgoing = abs(x-centre_u) <= width/2 and (
                    (MarkerCommand.TURN_LEFT in exits and y >= -width/2)
                    or (MarkerCommand.TURN_RIGHT in exits and y <= width/2))
                if not (incoming or outgoing):
                    return False
    return True


def make_route(state, now, radius, command):
    u, _ = frame_xy(state, now.x, now.y)
    centre = state.centre_u
    samples = []
    if command is MarkerCommand.STRAIGHT:
        samples = [(x, 0., 0., 0.) for x in np.arange(u-.2, centre+state.width_m+1., .04)]
    else:
        sign = 1. if command is MarkerCommand.TURN_LEFT else -1.
        samples = [(x, 0., 0., 0.) for x in np.arange(u-.2, centre-radius, .04)]
        samples += [(centre-radius+radius*math.sin(a), sign*radius*(1-math.cos(a)), sign*a, sign/radius)
                    for a in np.linspace(0., math.pi/2, math.ceil(math.pi*radius/2/.04)+1)]
        samples += [(centre, sign*v, sign*math.pi/2, 0.) for v in np.arange(radius+.04, radius+2., .04)]
    # Floating-point arange can include its nominal endpoint. At the straight/arc join
    # retain the later sample's curvature and one position, so projection has no zero segment.
    joined = []
    for sample in samples:
        if joined and math.hypot(sample[0]-joined[-1][0], sample[1]-joined[-1][1]) < 1e-9:
            joined[-1] = sample
        else:
            joined.append(sample)
    c, s = math.cos(state.frame_heading), math.sin(state.frame_heading)
    return tuple((c*x-s*(y+state.centre_v), s*x+c*(y+state.centre_v), heading+state.frame_heading, k)
                 for x, y, heading, k in joined)


def route_projection(route, pose):
    points = np.asarray(route)
    delta = np.diff(points[:, :2], axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    ratios = np.clip(np.sum((np.array((pose.x, pose.y))-points[:-1, :2])*delta, axis=1)/lengths**2, 0., 1.)
    projected = points[:-1, :2]+ratios[:, None]*delta
    index = int(np.argmin(np.linalg.norm(projected-np.array((pose.x, pose.y)), axis=1)))
    heading = math.atan2(delta[index, 1], delta[index, 0])
    error = np.array((pose.x, pose.y))-projected[index]
    lateral = -math.sin(heading)*error[0]+math.cos(heading)*error[1]
    yaw_error = math.atan2(math.sin(pose.heading-heading), math.cos(pose.heading-heading))
    arc = np.r_[0., np.cumsum(lengths)]
    progress = arc[index]+ratios[index]*lengths[index]
    return PathTrackingState(float(lateral), yaw_error, points[index, 3]), float(progress), arc


class IntersectionNavigator:
    """Finite junction-route admission, MPC feedback, swept clearance and observed exit handoff."""

    def __init__(self, config, mpc_config, obstacle_config, avoidance_config):
        self.config, self.mpc_config = config, mpc_config
        self.obstacle_config, self.avoidance_config = obstacle_config, avoidance_config
        self.controller = MpcController(replace(mpc_config, target_speed_m_per_s=config.turn_speed_m_per_s))
        self.selected_lane: SelectedIntersectionLane | None = None
        if config.enabled and config.turn_speed_m_per_s > mpc_config.target_speed_m_per_s:
            raise ValueError("Intersection speed must not exceed the normal MPC target")
        right = mpc_config.max_right_curvature_per_m or mpc_config.max_curvature_per_m
        if config.enabled and 1/config.turning_radius_m > min(mpc_config.max_curvature_per_m, right):
            raise ValueError("Intersection radius exceeds the configured physical curvature limits")

    def _stop(self, memory, state, reason):
        self.selected_lane = None
        memory.update_intersection(replace(state, phase="stopped", stop_reason=reason))
        if memory.data.mpc_stop is None:
            memory.stop_mpc(MpcStop.INTERSECTION)
        return [DriveCommand(0., 0., True)]

    def _motion(self, observations, state):
        car, now, dt = observations.car, observations.policy.policy_elapsed_s, observations.policy.dt
        speed, gyro = car.wheel_speed, car.ang_velocity
        limit = self.config.sensor_timeout_s
        if (speed is None or gyro is None or not all(math.isfinite(v) for v in (speed, gyro.yaw, now, dt))
                or speed < 0. or not 0. <= dt <= limit
                or not all(math.isfinite(float(age)) and 0. <= float(age) < limit for age in (speed.sensor_age, gyro.sensor_age))):
            return None
        if state.poses:
            prev = state.poses[-1]
            if abs(now-prev.time_s-dt) > .02 or now <= prev.time_s:
                return None
            rate, v = (prev.yaw_rate+gyro.yaw)/2, (prev.speed+speed)/2
            heading = prev.heading+rate*dt
            middle = (heading+prev.heading)/2
            pose = IntersectionPose(now, prev.x+v*dt*math.cos(middle), prev.y+v*dt*math.sin(middle),
                                    heading, prev.distance_m+v*dt, float(speed), gyro.yaw)
        else:
            pose = IntersectionPose(now, 0., 0., 0., 0., float(speed), gyro.yaw)
        return replace(state, poses=(tuple(p for p in state.poses if p.time_s >= now-3.)+(pose,))[-80:])

    def _observe(self, observations, lanes, state):
        batch, now = observations.cones, state.poses[-1]
        if batch is None:
            return state
        if not math.isfinite(float(batch.sensor_age)) or not 0. <= float(batch.sensor_age) < self.config.sensor_timeout_s:
            return state
        acquisition = now.time_s-float(batch.sensor_age)
        pose = pose_at(state.poses, acquisition)
        if pose is None:
            return state  # Startup has not acquired enough motion history yet.
        tracks = list(state.cones)
        for cone in batch:
            x, y = world_xy(pose, cone.pos.x, cone.pos.y)
            matching = [i for i, t in enumerate(tracks) if t.colour == cone.colour.name and math.hypot(t.x-x, t.y-y) < .12]
            if matching:
                i = min(matching, key=lambda i: math.hypot(tracks[i].x-x, tracks[i].y-y))
                if acquisition > tracks[i].seen_s:
                    tracks[i] = IntersectionCone(x, y, cone.colour.name, acquisition)
            else:
                tracks.append(IntersectionCone(x, y, cone.colour.name, acquisition))
        state = replace(state, cones=tuple(sorted((t for t in tracks if now.time_s-t.seen_s <= 8.),
                                                 key=lambda t: t.seen_s)[-240:]),
                        last_nonempty_s=acquisition if batch else state.last_nonempty_s)
        if not state.route:
            # The normal curved-lane fit is poorly conditioned over a wide road's short visible
            # span. Fit the incoming STRAIGHT rows jointly over transported, distinct observations.
            # Side-road points are excluded by distance from the already established rows.
            tracks = [t for t in state.cones if now.time_s-t.seen_s <= 3.]
            if state.frame_heading is not None:
                # An opposite arm may place the same colour across the incoming road.
                # Once its sides are measured, preserve colour membership as well as
                # distance to a row; otherwise near-corner tracks can rotate the frame.
                def incoming_row(t):
                    v = frame_xy(state, t.x, t.y)[1]
                    if state.frame_left_colour is None:  # Legacy recording without measured colour sides.
                        return abs(abs(v)-state.width_m/2) < .12
                    expected = state.width_m/2 if t.colour == state.frame_left_colour else -state.width_m/2
                    return abs(v-expected) < .12
                tracks = [t for t in tracks if incoming_row(t)]
            if all(sum(t.colour == colour for t in tracks) >= 3 for colour in ("BLUE", "YELLOW")):
                base = pose.heading if state.frame_heading is None else state.frame_heading
                c, s = math.cos(base), math.sin(base)
                points = np.array([[c*t.x+s*t.y, -s*t.x+c*t.y, float(t.colour == "BLUE"), t.seen_s] for t in tracks])
                for _ in range(2):
                    matrix = np.c_[points[:, 0], points[:, 2], 1-points[:, 2]]
                    fit = np.linalg.lstsq(matrix, points[:, 1], rcond=None)[0]
                    points = points[np.abs(matrix@fit-points[:, 1]) < .08]
                rows = [points[points[:, 2] == value] for value in (0., 1.)]
                if all(len(row) >= 3 and np.ptp(row[:, 0]) >= .8 for row in rows):
                    slope, blue, yellow = fit
                    correction = math.atan(slope)
                    heading = base+correction
                    v = (blue+yellow)/2*math.cos(correction)
                    width = abs(blue-yellow)*math.cos(correction)
                    left_colour = "BLUE" if blue > yellow else "YELLOW"
                    seen = min(max(row[:, 3]) for row in rows)
                    support_pose = pose_at(state.poses, seen)
                    if .6 < width <= 3.2 and state.frame_left_colour in (None, left_colour) and (state.frame_heading is None or
                            (abs(heading-state.frame_heading) < .02 and abs(v-state.centre_v) < .08)) and support_pose is not None:
                        # A candidate centre and its finite endpoint use this frame's
                        # coordinates. Once anchored, fresh matching rows may renew evidence
                        # but cannot rotate/resize the frame underneath those anchors.
                        geometry = (dict(frame_heading=heading, centre_v=v, width_m=width)
                                    if state.centre_u is None else {})
                        state = replace(state, **geometry, frame_left_colour=left_colour,
                                        frame_seen_s=seen, frame_distance_m=support_pose.distance_m)
        return state

    def _lidar_world(self, observations, state):
        stamp = state.poses[-1].time_s-float(observations.lidar_cartesian.sensor_age)
        pose = pose_at(state.poses, stamp)
        if pose is None:
            return None, None
        return pose, np.array([world_xy(pose, p.pos.x, p.pos.y) for p in observations.lidar_cartesian.points]).reshape(-1, 2)

    def _junction_evidence(self, observations, state):
        if state.frame_heading is None:
            return None
        now = state.poses[-1]
        _, world = self._lidar_world(observations, state)
        if world is None:
            return None
        points = np.array([frame_xy(state, *p) for p in world])
        if len(points) == 0:
            return None
        here, _ = frame_xy(state, now.x, now.y)
        h = state.width_m/2
        candidates = points[(points[:, 0] > here+.4) & (np.abs(points[:, 1]) > h+.12)]
        def supported(row, along, span=.6, bins=4):
            # A single cone can yield several returns. Require spatial extent as well
            # as distinct bins, including the weaker far-wall corroboration below.
            return len(row) > 0 and len(np.unique(np.round(row[:, along]/.12))) >= bins and np.ptp(row[:, along]) >= span

        for u in sorted(set(np.round(candidates[:, 0]/.10)*.10)):
            row = candidates[np.abs(candidates[:, 0]-u) < .07]
            near = [supported(row[row[:, 1]*sign > h+.12], 1) for sign in (1., -1.)]
            if not any(near):
                continue
            centre = float(np.median(row[:, 0])+h)
            far = points[np.abs(points[:, 0]-(centre+h)) < .07]
            available, closed = [], []
            for command, sign, near_support in zip((MarkerCommand.TURN_LEFT, MarkerCommand.TURN_RIGHT), (1., -1.), near):
                cap = points[(np.abs(points[:, 1]-sign*h) < .07) & (np.abs(points[:, 0]-centre) < h-.15)]
                if supported(cap, 0):
                    closed.append(command)
                # The far boundary only enters a 4 m scan shortly before a 2.5 m turn.
                # Two separated cones corroborate it; a full .6 m span arrives too late.
                elif near_support:
                    continuation = far[far[:, 1]*sign >= h-.07]
                    if np.any(continuation[:, 1]*sign > h+.12) and supported(continuation, 1, .15, 2):
                        available.append(command)
            cap = far[np.abs(far[:, 1]) < h-.15]
            if supported(cap, 1):
                closed.append(MarkerCommand.STRAIGHT)
            elif all(supported(points[(np.abs(points[:, 1]-sign*h) < .07)
                                      & (points[:, 0] > centre+h+.12)], 0, .15, 2) for sign in (1., -1.)):
                available.append(MarkerCommand.STRAIGHT)
            # A perpendicular near wall establishes a candidate. Unknown exits stay
            # unknown; the selected exit must acquire positive support before its deadline.
            return centre, tuple(available), tuple(closed)
        return None

    def _outgoing_supported(self, state, pose):
        if state.command is MarkerCommand.STRAIGHT:
            heading = state.frame_heading
            along = lambda t: frame_xy(state, t.x, t.y)[0]
            across = lambda t: frame_xy(state, t.x, t.y)[1]
            here = frame_xy(state, pose.x, pose.y)[0]
            target_across = 0.
        else:
            sign = 1 if state.command is MarkerCommand.TURN_LEFT else -1
            heading = state.frame_heading+sign*math.pi/2
            along = lambda t: sign*frame_xy(state, t.x, t.y)[1]
            across = lambda t: frame_xy(state, t.x, t.y)[0]
            here = sign*frame_xy(state, pose.x, pose.y)[1]
            target_across = state.centre_u
        error = abs(math.atan2(math.sin(pose.heading-heading), math.cos(pose.heading-heading)))
        if error > .15:
            return None
        evidence = []
        for side in (-1., 1.):
            expected = target_across+side*state.width_m/2
            tracks = [t for t in state.cones if pose.time_s-t.seen_s <= .5
                      and abs(across(t)-expected) < .12 and along(t) > here+.3]
            points = [along(t) for t in tracks]
            if len(points) < 2 or max(points)-min(points) < .35 or max(points) < here+.8:
                return None
            evidence.append(max(t.seen_s for t in tracks))
        return min(evidence)

    def _approach(self, observations, state, memory, mpc):
        # This is the measured incoming corridor, not a committed exit. Its endpoint
        # is anchored once, before the junction mouth; moving predictions cannot extend it.
        now = state.poses[-1]
        here, _ = frame_xy(state, now.x, now.y)
        _, world = self._lidar_world(observations, state)
        points = np.array([frame_xy(state, *p) for p in world]).reshape(-1, 2)
        for sign in (-1., 1.):
            row = points[(np.abs(points[:, 1]-sign*state.width_m/2) < .12)
                         & (points[:, 0] > here+.3) & (points[:, 0] < state.centre_u-state.width_m/2-.1)]
            if len(row) == 0 or np.ptp(row[:, 0]) < .4-1e-9 or len(np.unique(np.round(row[:, 0]/.12))) < 2:
                return self._stop(memory, state, "incoming junction corridor lacks current paired lidar support")
        remaining = state.approach_end_u-here
        reaction = self.mpc_config.obstacle_stop_distance_per_speed_s+self.config.sensor_timeout_s+.3+observations.policy.dt
        margin = self.mpc_config.obstacle_stop_margin_m
        if remaining <= now.speed*reaction+margin:
            return self._stop(memory, state, "incoming corridor stopping envelope reached before exit support")
        speed = min(self.config.turn_speed_m_per_s, max(0., (remaining-margin)/reaction))
        if speed < .05:
            return self._stop(memory, state, "incoming corridor ends before exit support")
        c, s = math.cos(state.frame_heading), math.sin(state.frame_heading)
        route = tuple((c*u-s*state.centre_v, s*u+c*state.centre_v, state.frame_heading, 0.)
                      for u in np.arange(here-.2, state.approach_end_u+.001, .04))
        tracking, progress, arc = route_projection(route, now)
        if abs(tracking.lateral_error_m) > .35 or abs(tracking.heading_error_rad) > .45:
            return self._stop(memory, state, "incoming junction corridor tracking error exceeded")
        previous = mpc.previous_curvature_per_m
        predicted_speed = max(now.speed, .05)
        references = np.zeros(self.mpc_config.horizon_steps)
        solution = self.controller.solve(tracking, previous, 0., predicted_speed, reference_curvatures=references)
        if not solution.converged:
            return self._stop(memory, state, "incoming junction corridor MPC solver failed")
        k = float(solution.curvatures_per_m[0])
        temporary = replace(state, route=route, progress_m=progress, previous_curvature=previous)
        if not self._clear(observations, temporary, k):
            return self._stop(memory, state, "incoming junction corridor swept lidar footprint blocked")
        memory.update_intersection(replace(state, previous_curvature=k))
        mpc.lane_state = tracking
        memory.plan_mpc(solution.curvatures_per_m, predicted_speed*self.mpc_config.step_s)
        c, s = math.cos(now.heading), math.sin(now.heading)
        body = tuple((c*(x-now.x)+s*(y-now.y), -s*(x-now.x)+c*(y-now.y))
                     for (x, y, _, _), distance in zip(route, arc) if distance >= progress)
        self.selected_lane = SelectedIntersectionLane(state.command, "approach", body, state.width_m,
            now.time_s-state.frame_seen_s, tuple(references), tuple(float(v) for v in solution.curvatures_per_m),
            predicted_speed*self.mpc_config.step_s)
        return [mpc._command(speed, k)]

    def _clear(self, observations, state, curvature):
        # Full rectangular body at a rolling current-curvature delay, planned route, and straight
        # braking tail. Every raw finite return counts, including cones. No colour exemptions.
        now = state.poses[-1]
        _, world = self._lidar_world(observations, state)
        if world is None:
            return False
        c, s = math.cos(now.heading), math.sin(now.heading)
        delta = world-np.array((now.x, now.y))
        points = delta @ np.array(((c, -s), (s, c)))
        if len(points) == 0:
            return True
        if not np.all(np.isfinite(points)):
            return False
        speed = max(now.speed, self.config.turn_speed_m_per_s, self.mpc_config.target_speed_m_per_s)
        travel = speed*(self.mpc_config.obstacle_stop_distance_per_speed_s+.3)+self.mpc_config.obstacle_stop_margin_m
        front, rear = self.obstacle_config.car_front_m, self.avoidance_config.car_rear_m
        half_width = self.avoidance_config.car_width_m/2
        pad = .075
        def hits(x, y, heading):
            c, s = math.cos(heading), math.sin(heading)
            dx, dy = points[:, 0]-x, points[:, 1]-y
            forward, left = c*dx+s*dy, -s*dx+c*dy
            return bool(np.any((forward >= -rear-pad) & (forward <= front+pad) & (np.abs(left) <= half_width+pad)))
        # Actual command sweep and previous steering during a conservative .3 s actuation delay.
        for k in (curvature, state.previous_curvature, 0.):
            for d in np.arange(0., travel+.025, .025):
                a = d*k
                x = d if abs(k) < 1e-9 else math.sin(a)/k
                y = 0. if abs(k) < 1e-9 else (1-math.cos(a))/k
                if hits(x, y, a):
                    return False
            reaction = min(travel, speed*.3)
            angle = reaction*k
            rx = reaction if abs(k) < 1e-9 else math.sin(angle)/k
            ry = 0. if abs(k) < 1e-9 else (1-math.cos(angle))/k
            for tail in np.arange(0., travel-reaction+.025, .025):
                if hits(rx+tail*math.cos(angle), ry+tail*math.sin(angle), angle):
                    return False
        # Check the retained route ahead as well; turning changes the swept corridor.
        path = np.array(state.route)
        _, _, arc = route_projection(state.route, now)
        for x, y, heading, _ in path[(arc >= state.progress_m) & (arc <= state.progress_m+travel)]:
            c, s = math.cos(now.heading), math.sin(now.heading)
            dx, dy = x-now.x, y-now.y
            if hits(c*dx+s*dy, -s*dx+c*dy, heading-now.heading):
                return False
        return True

    def step(self, observations, lanes, memory, mpc):
        self.selected_lane = None
        if not self.config.enabled:
            return None
        state = memory.data.intersection or IntersectionMemory()
        if state.phase == "stopped" or memory.data.mpc_stop is not None:
            return self._stop(memory, state, state.stop_reason or "existing stop latch")
        if 0. < self.mpc_config.run_duration_s <= observations.policy.policy_elapsed_s:
            return self._stop(memory, state, "configured run duration elapsed")
        state = self._motion(observations, state)
        if state is None:
            return self._stop(memory, memory.data.intersection or IntersectionMemory(), "invalid or stale wheel/gyro motion")
        lidar = observations.lidar_cartesian
        if (lidar is None or not math.isfinite(float(lidar.sensor_age))
                or not 0. <= float(lidar.sensor_age) < self.config.sensor_timeout_s
                or any(not math.isfinite(v) for p in lidar.points for v in (p.pos.x, p.pos.y))):
            return self._stop(memory, state, "missing or stale intersection lidar")
        state = self._observe(observations, lanes, state)
        now = state.poses[-1]
        queued = memory.data.intersection_command
        if state.phase == "complete":
            # Re-arm only for a spatially new marker, after leaving this crossing.
            if queued is None:
                memory.update_intersection(state)
                return None
            if now.distance_m-state.completed_distance_m < state.width_m:
                memory.consume_intersection_command()
                memory.update_intersection(state)
                return None
            state = replace(IntersectionMemory(), poses=state.poses, cones=state.cones,
                            consumed_marker_xy=state.consumed_marker_xy)
        if not state.route:
            if queued is not None and memory.data.intersection_command_seen_s != state.command_seen_s:
                marker_xy = None
                batch = observations.fiducials
                pose = None if batch is None else pose_at(state.poses, now.time_s-float(batch.sensor_age))
                if pose is not None:
                    markers = [t for t in batch.detections if t.pose is not None and memory.data.marker_sighting is not None
                               and t.id == memory.data.marker_sighting.marker_id]
                    if markers:
                        p = markers[0].pose.position
                        marker_xy = world_xy(pose, p.x, p.y)
                if marker_xy is not None and state.consumed_marker_xy is not None and math.dist(marker_xy, state.consumed_marker_xy) < 2.:
                    memory.consume_intersection_command()
                    queued = None
                else:
                    state = replace(state, command=queued, command_seen_s=memory.data.intersection_command_seen_s,
                                    marker_xy=marker_xy,
                                    marker_id=None if memory.data.marker_sighting is None else memory.data.marker_sighting.marker_id)
            if (state.command is not None and state.command_seen_s is not None
                    and now.time_s-state.command_seen_s > self.config.command_max_age_s):
                return self._stop(memory, state, "intersection direction expired")
            evidence = self._junction_evidence(observations, state)
            if evidence is None:
                if state.centre_u is not None and state.command is not None:
                    here, _ = frame_xy(state, now.x, now.y)
                    deadline = (state.centre_u-state.width_m/2-.5 if state.command is MarkerCommand.STRAIGHT
                                else state.centre_u-self.config.turning_radius_m+.15)
                    if here >= deadline:
                        return self._stop(memory, state, "junction exit evidence lost before admission deadline")
                    if state.approach_end_u is not None:
                        return self._stop(memory, state, "junction exit evidence lost during the finite approach")
                memory.update_intersection(state)
                return None
            centre, available, closed = evidence
            if state.centre_u is not None and abs(centre-state.centre_u) > .2:
                return self._stop(memory, state, "junction geometry changed before admission")
            if state.centre_u is not None:
                centre = state.centre_u  # Preserve the first candidate and its finite approach endpoint.
            if (state.frame_seen_s is None or now.time_s-state.frame_seen_s > 3.
                    or now.distance_m-state.frame_distance_m > 1.5):
                return self._stop(memory, state, "incoming cross frame evidence expired")
            here, _ = frame_xy(state, now.x, now.y)
            if centre-self.config.turning_radius_m-here > 1.:
                memory.update_intersection(state)
                return None
            if state.command is None:
                return self._stop(memory, state, "crossing observed without a confirmed direction")
            if state.command_seen_s is None or now.time_s-state.command_seen_s > self.config.command_max_age_s:
                return self._stop(memory, state, "intersection direction expired")
            end = (centre-state.width_m/2-self.obstacle_config.car_front_m-self.mpc_config.obstacle_stop_margin_m
                   if state.approach_end_u is None else state.approach_end_u)
            state = replace(state, centre_u=centre, approach_end_u=end, available_exits=available, closed_exits=closed)
            if state.command in closed:
                return self._stop(memory, state, "requested exit is closed by a supported cone row")
            if state.command not in available:
                deadline = (centre-state.width_m/2-.5 if state.command is MarkerCommand.STRAIGHT
                            else centre-self.config.turning_radius_m+.15)
                if here >= deadline:
                    return self._stop(memory, state, "requested exit lacks boundary support before admission deadline")
                return self._approach(observations, state, memory, mpc)
            if state.command is not MarkerCommand.STRAIGHT and here > centre-self.config.turning_radius_m+.15:
                return self._stop(memory, state, "insufficient approach distance for the turn")
            if not cross_fits(centre, state.width_m, self.config.turning_radius_m, state.command,
                              self.obstacle_config.car_front_m, self.avoidance_config.car_rear_m,
                              self.avoidance_config.car_width_m, exits=available):
                return self._stop(memory, state, "crossing too narrow for the configured turning radius")
            lidar_pose, _ = self._lidar_world(observations, state)
            if lidar_pose is None:
                return self._stop(memory, state, "lidar acquisition pose unavailable")
            state = replace(state, centre_u=centre, phase="turn", route_seen_s=lidar_pose.time_s,
                            route_distance_m=lidar_pose.distance_m, consumed_marker_xy=state.marker_xy)
            state = replace(state, route=make_route(state, now, self.config.turning_radius_m, state.command))
            memory.consume_intersection_command()
        if (now.time_s-state.route_seen_s > self.config.route_max_age_s
                or now.distance_m-state.route_distance_m > self.config.route_max_distance_m):
            return self._stop(memory, state, "intersection route evidence expired")
        if state.last_nonempty_s is None or now.time_s-state.last_nonempty_s > self.config.camera_empty_hold_s:
            return self._stop(memory, state, "all cones missing beyond intersection hold")
        tracking, progress, arc = route_projection(state.route, now)
        mpc.lane_state = tracking
        state = replace(state, progress_m=max(state.progress_m, progress))
        if abs(tracking.lateral_error_m) > .35 or abs(tracking.heading_error_rad) > .45:
            return self._stop(memory, state, "intersection tracking error exceeded")
        end_heading = state.route[-1][2]
        aligned = abs(math.atan2(math.sin(now.heading-end_heading), math.cos(now.heading-end_heading))) < .15
        reached = (frame_xy(state, now.x, now.y)[0] > state.centre_u+state.width_m/2+.3
                   if state.command is MarkerCommand.STRAIGHT else
                   abs(frame_xy(state, now.x, now.y)[1]) > self.config.turning_radius_m+.3)
        evidence = self._outgoing_supported(state, now) if reached and aligned else None
        if evidence is not None:
            advancing = state.exit_last_evidence_s is None or evidence > state.exit_last_evidence_s+1e-6
            gap = state.exit_last_evidence_s is None or evidence-state.exit_last_evidence_s > .25
            first = evidence if state.exit_confirmed_since_s is None or gap else state.exit_confirmed_since_s
            state = replace(state, phase="exit", exit_confirmed_since_s=first,
                            exit_last_evidence_s=evidence if advancing else state.exit_last_evidence_s)
            if advancing and evidence-first >= .3-1e-6:
                state = replace(state, phase="complete", completed_distance_m=now.distance_m)
                memory.update_intersection(state)
                mpc.avoidance_path = None
                return None
        else:
            state = replace(state, exit_confirmed_since_s=None, exit_last_evidence_s=None)
        if arc[-1]-progress < .3:
            return self._stop(memory, state, "outgoing lane not confirmed before route exhaustion")
        prediction_speed = max(now.speed, .05)
        selected = select_intersection_lane(state, now, progress, arc, prediction_speed,
                                             self.mpc_config.step_s, self.mpc_config.horizon_steps)
        solution = self.controller.solve(tracking, state.previous_curvature, 0., prediction_speed,
                                         reference_curvatures=np.asarray(selected.reference_curvatures_per_m))
        if not solution.converged:
            return self._stop(memory, state, "intersection MPC solver failed")
        k = float(solution.curvatures_per_m[0])
        if not self._clear(observations, state, k):
            return self._stop(memory, state, "intersection swept lidar footprint blocked")
        memory.update_intersection(replace(state, previous_curvature=k))
        memory.plan_mpc(solution.curvatures_per_m, prediction_speed*self.mpc_config.step_s)
        self.selected_lane = replace(selected, mpc_curvatures_per_m=tuple(float(k) for k in solution.curvatures_per_m),
                                     prediction_step_m=prediction_speed*self.mpc_config.step_s)
        return [mpc._command(self.config.turn_speed_m_per_s, k)]
