"""Optional forward-only cone memory and whole-fit lane continuity checks.

Integrate fresh measured speed and gyro yaw into a bounded local pose history. Interpolate the pose at each
camera acquisition; never extrapolate before history starts. Tracks are returned in that acquisition frame,
with the original batch age, because the controller performs its own latency compensation. Planar odometry
assumes forward motion without slip: unsigned wheel speed cannot detect reverse or rollback. These experimental
features default off and do not change sensor freshness, node state controls or the existing edge fitter.
"""
from dataclasses import replace
import math

from policy.input_output import CarObservations, ConeBatch, ConeColour, ConeDetection, Position
from policy.lane_detection.lanes import LaneArc, LaneDetection, LaneEdge, LaneStatus
from policy.memory import BothEdgeLaneReference, CarMemory, ConeTrack, LaneReference, LaneTrackingMemory, MotionPose


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _world(x, y, pose):
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return pose.x + c * x - s * y, pose.y + s * x + c * y


def _body(x, y, pose):
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    dx, dy = x - pose.x, y - pose.y
    return c * dx + s * dy, -s * dx + c * dy


def _advance(previous, time_s, speed, yaw_rate):
    dt = time_s - previous.time_s
    v, w = (previous.speed + speed) / 2, (previous.yaw_rate + yaw_rate) / 2
    turn = w * dt
    chord = v * dt * (1.0 if abs(turn) < 1e-9 else math.sin(turn / 2) / (turn / 2))
    return MotionPose(time_s, previous.x + chord * math.cos(previous.yaw + turn / 2),
                      previous.y + chord * math.sin(previous.yaw + turn / 2), previous.yaw + turn,
                      previous.distance_m + v * dt, speed, yaw_rate)


def _pose_at(poses, time_s):
    if not poses or time_s < poses[0].time_s - 1e-8 or time_s > poses[-1].time_s + 1e-8:
        return None
    time_s = min(max(time_s, poses[0].time_s), poses[-1].time_s)
    for a, b in zip(poses, poses[1:]):
        if a.time_s <= time_s <= b.time_s:
            fraction = (time_s - a.time_s) / (b.time_s - a.time_s)
            # Interpolate the constant-twist segment, not its chord, so turning cones stay correctly positioned.
            v = (a.speed + b.speed) / 2
            w = (a.yaw_rate + b.yaw_rate) / 2
            partial = _advance(replace(a, speed=v, yaw_rate=w), time_s, v, w)
            return replace(partial, distance_m=a.distance_m + fraction * (b.distance_m - a.distance_m))
    return poses[-1]


def _motion(observations, state, config):
    car, policy = observations.car, observations.policy
    if car is None or policy is None or car.wheel_speed is None or car.ang_velocity is None:
        return None
    speed, gyro = car.wheel_speed, car.ang_velocity
    time_s, dt, yaw_rate = policy.policy_elapsed_s, policy.dt, gyro.yaw
    values = (time_s, dt, float(speed), yaw_rate, float(speed.sensor_age), float(gyro.sensor_age))
    if (not all(math.isfinite(v) for v in values) or time_s < 0 or not 0 <= dt <= config.motion_max_gap_s
            or not 0 <= speed <= config.motion_max_speed_m_per_s
            or abs(yaw_rate) > config.motion_max_yaw_rate_rad_per_s
            or not 0 <= speed.sensor_age <= config.motion_max_age_s
            or not 0 <= gyro.sensor_age <= config.motion_max_age_s
            or (observations.rc is not None and observations.rc.drive < 0)):
        return None
    if not state.poses:
        return (MotionPose(time_s, 0.0, 0.0, 0.0, 0.0, float(speed), yaw_rate),)
    previous = state.poses[-1]
    gap = time_s - previous.time_s
    if not 0 < gap <= config.motion_max_gap_s or abs(gap - dt) > 1e-5:
        return None
    current = _advance(previous, time_s, float(speed), yaw_rate)
    # Both a time bound and a count bound; extremely fast policy timers cannot grow the history indefinitely.
    return tuple(p for p in (*state.poses, current) if p.time_s >= time_s - 2.5)[-512:]


def _remember(cones, pose, state, config, now):
    tracks = [track for track in state.tracks
              if now.time_s - track.seen_s <= config.memory_max_age_s
              and now.distance_m - track.seen_distance_m <= config.memory_max_distance_m]
    new = state.last_acquisition_s is None or pose.time_s > state.last_acquisition_s + 1e-8
    fresh_world = [(*_world(c.pos.x, c.pos.y, pose), c) for c in cones]
    if new:
        # One fresh detection can replace at most one same-colour track. Fresh coordinates always win.
        for x, y, cone in fresh_world:
            matches = [(math.hypot(t.x - x, t.y - y), i) for i, t in enumerate(tracks)
                       if t.colour == cone.colour.name]
            if matches:
                distance, index = min(matches)
                if distance <= config.memory_match_distance_m:
                    tracks.pop(index)
        fresh_tracks = [ConeTrack(x, y, c.pos.z, c.colour.name, c.confidence, pose.time_s, pose.distance_m)
                        for x, y, c in fresh_world
                        if now.time_s - pose.time_s <= config.memory_max_age_s]
        tracks = sorted([*fresh_tracks, *tracks], key=lambda t: t.seen_s, reverse=True)[:config.memory_max_tracks]
    remembered = []
    for track in tracks:
        if track.seen_s >= pose.time_s - 1e-8:
            continue
        x, y = _body(track.x, track.y, pose)
        # Keep cones just behind/beside the car through a turn, but never extend the remembered map indefinitely.
        if x < -0.5 or math.hypot(x, y) > 4.0:
            continue
        if any(c.colour.name == track.colour and math.hypot(track.x - fx, track.y - fy)
               <= config.memory_match_distance_m for fx, fy, c in fresh_world):
            continue
        remembered.append(ConeDetection(Position(x, y, track.z), ConeColour[track.colour], track.confidence))
    augmented = ConeBatch([*cones, *remembered], cones.sensor_age, cones.acquisition_to_publish_latency_s)
    return augmented, tuple(tracks), len(remembered)


def _no_lane(lane):
    return LaneDetection(LaneStatus.NO_LANE, None, None, None, None, None, None, None, None, lane.sensor_age)


def _consistent(lane, reference, pose, config):
    dt = pose.time_s - reference.pose.time_s
    if dt < -1e-8 or dt > config.memory_max_age_s:
        return False
    x, y = _body(*_world(reference.x, reference.y, reference.pose), pose)
    previous = LaneArc(x, y, reference.heading + reference.pose.yaw - pose.yaw, reference.curvature)
    current = lane.centre_line
    # Restrict the comparison to the overlap of both observations' supported extents.
    end_x, end_y = previous.point_at(reference.visible_until_m)
    overlap_start = max(0.0, current.project(previous.x, previous.y))
    overlap_end = min(1.0, lane.visible_until_m, current.project(end_x, end_y))
    if overlap_end <= overlap_start:
        return False
    for distance in (overlap_start, overlap_end):
        x, y = current.point_at(distance)
        projected = previous.project(x, y)
        if projected < -0.01 or projected > reference.visible_until_m + 0.01:
            return False
        if abs(previous.signed_distance(x, y)) > (config.continuity_lateral_slack_m
                                                  + config.continuity_max_lateral_rate_m_per_s * dt):
            return False
        if abs(_wrap(current.heading_at(distance) - previous.heading_at(projected))) > (
                config.continuity_heading_slack_rad + config.continuity_max_heading_rate_rad_per_s * dt):
            return False
    return abs(current.curvature_per_m - previous.curvature_per_m) <= (
        config.continuity_curvature_slack_per_m + config.continuity_max_curvature_rate_per_m_s * dt)


def _single_edge_prediction(fresh_lane, cones, reference, pose, now, config, colour=None):
    """Transport a two-edge lane to this acquisition frame, requiring support from the remaining fresh edge."""
    if reference is None or pose is None or now is None:
        return _no_lane(fresh_lane)
    if (not 0 <= now.time_s - reference.pose.time_s <= config.single_edge_hold_max_age_s
            or not 0 <= now.distance_m - reference.pose.distance_m <= config.single_edge_hold_max_distance_m):
        return _no_lane(fresh_lane)
    x, y = _body(*_world(reference.x, reference.y, reference.pose), pose)
    transported = LaneArc(x, y, reference.heading + reference.pose.yaw - pose.yaw, reference.curvature)
    progress = transported.project(0.0, 0.0)
    remaining = reference.visible_until_m - progress
    if remaining <= 0.05 or abs(reference.curvature * reference.width_m / 2) >= 0.9:
        return _no_lane(fresh_lane)
    x, y = transported.point_at(progress)
    centre = LaneArc(x, y, _wrap(transported.heading_at(progress)), reference.curvature)
    # Ignore the one-edge fit's AUTO side label: its colour keeps the role seen with BOTH edges.
    edge = fresh_lane.left_edge or fresh_lane.right_edge
    colour = edge.colour if colour is None and edge is not None else colour
    if colour is None:
        return _no_lane(fresh_lane)
    left = colour.name == reference.left_colour
    if not left and colour.name != reference.right_colour:
        return _no_lane(fresh_lane)
    boundary = centre.offset((1 if left else -1) * reference.width_m / 2)
    supporting = [c for c in cones if c.colour == colour
                  and abs(boundary.signed_distance(c.pos.x, c.pos.y)) <= config.outlier_distance_m
                  and -0.2 <= centre.project(c.pos.x, c.pos.y) <= remaining + config.outlier_distance_m]
    if (len(supporting) < config.min_cones_per_edge
            or max((math.dist((a.pos.x, a.pos.y), (b.pos.x, b.pos.y))
                    for a in supporting for b in supporting), default=0.0) < 0.25):
        return _no_lane(fresh_lane)
    along = [boundary.project(c.pos.x, c.pos.y) for c in supporting]
    predicted_edge = LaneEdge(boundary, colour, len(supporting), min(along), max(along),
                              math.sqrt(sum(boundary.signed_distance(c.pos.x, c.pos.y) ** 2
                                            for c in supporting) / len(supporting)))
    extent = min(remaining, max(centre.project(c.pos.x, c.pos.y) for c in supporting))
    if extent <= 0.05:
        return _no_lane(fresh_lane)
    return LaneDetection(LaneStatus.LEFT_EDGE_ONLY if left else LaneStatus.RIGHT_EDGE_ONLY,
                         predicted_edge if left else None, None if left else predicted_edge, centre,
                         reference.width_m, extent, centre.signed_distance(0, 0), -centre.heading_rad,
                         centre.curvature_per_m, cones.sensor_age)


def detect_temporal_lane(observations: CarObservations, memory: CarMemory, detector):
    """Run the unchanged edge fitter with optional cone memory, then accept or reject its complete lane geometry.

    Fresh-only observations remain with the caller for endpoint and MPC horizon evidence. No cone batch or invalid
    motion clears memory; a continuity-enabled policy then reports no lane and uses its existing bounded lane-loss
    behaviour. A rejected reference never renews its age; after expiry it stays rejected until a run/motion reset.
    """
    config = detector.config
    if not config.memory_enabled and not config.continuity_enabled and not config.single_edge_hold_enabled:
        return detector.detect(observations), observations.cones
    state = memory.data.lane_tracking
    if observations.policy is not None and observations.policy.is_first_policy_step:
        state = LaneTrackingMemory()
    poses = _motion(observations, state, config)
    cones = observations.cones
    ready = False
    pose = None
    if poses is not None and cones is not None:
        age = float(cones.sensor_age)
        stamp = cones.sensor_age.stamp_ns
        acquisition_s = poses[-1].time_s - age
        if stamp is not None and stamp == state.last_stamp_ns and state.last_acquisition_s is not None:
            acquisition_s = state.last_acquisition_s
        if math.isfinite(age) and 0 <= age <= config.memory_max_age_s:
            pose = _pose_at(poses, acquisition_s)
        ready = pose is not None
        if ((stamp is not None and state.last_stamp_ns is not None and stamp < state.last_stamp_ns)
                or (pose is not None and state.last_acquisition_s is not None
                    and pose.time_s < state.last_acquisition_s - 1e-8)):
            poses, pose, ready = None, None, False
    if poses is None or cones is None:
        state = LaneTrackingMemory()
    else:
        tracks = tuple(t for t in state.tracks if poses[-1].time_s - t.seen_s <= config.memory_max_age_s
                       and poses[-1].distance_m - t.seen_distance_m <= config.memory_max_distance_m)
        state = replace(state, poses=poses, tracks=tracks, remembered_count=0, rejected=False, holding_single_edge=False)
    prior_tracks = state.tracks
    augmented = cones
    if ready:
        if config.memory_enabled:
            augmented, tracks, count = _remember(cones, pose, state, config, poses[-1])
            state = replace(state, tracks=tracks, remembered_count=count)
        state = replace(state, last_stamp_ns=cones.sensor_age.stamp_ns, last_acquisition_s=pose.time_s)
    width_before = (config.lane_width_m if observations.policy is not None
                    and observations.policy.is_first_policy_step else detector.lane_width_m)
    fresh_lane = None
    if config.single_edge_hold_enabled:
        # Visibility is established from fresh-only fitted evidence. Remembered cones cannot invent BOTH edges.
        fresh_lane = detector.detect(observations)
        lane = fresh_lane
        if fresh_lane is not None and fresh_lane.status in (LaneStatus.LEFT_EDGE_ONLY, LaneStatus.RIGHT_EDGE_ONLY):
            lane = _single_edge_prediction(fresh_lane, cones, state.both_edge_reference, pose,
                                           poses[-1] if ready else None, config)
            state = replace(state, holding_single_edge=lane.has_lane)
        elif (fresh_lane is not None and fresh_lane.status == LaneStatus.NO_LANE
              and state.both_edge_reference is not None):
            # An edge fit can fail with sparse/partially occluded cones even when those cones still
            # support the saved lane. Validate against its boundaries instead of requiring another arc fit.
            anchor = state.both_edge_reference
            candidates = [_single_edge_prediction(fresh_lane, cones, anchor, pose,
                          poses[-1] if ready else None, config, ConeColour[name])
                          for name in (anchor.left_colour, anchor.right_colour)]
            supported = [candidate for candidate in candidates if candidate.has_lane]
            if len(supported) == 1:
                lane = supported[0]
            elif len(supported) == 2:
                lane = replace(supported[0], status=LaneStatus.BOTH_EDGES,
                               right_edge=supported[1].right_edge,
                               visible_until_m=min(candidate.visible_until_m for candidate in supported))
            state = replace(state, holding_single_edge=lane.has_lane)
        elif fresh_lane is not None and fresh_lane.status == LaneStatus.BOTH_EDGES and state.both_edge_reference:
            anchor = state.both_edge_reference
            if (fresh_lane.left_edge.colour.name != anchor.left_colour
                    or fresh_lane.right_edge.colour.name != anchor.right_colour):
                lane = _no_lane(fresh_lane)
                state = replace(state, rejected=True)
    else:
        lane = detector.detect(replace(observations, cones=augmented))
    # No remembered endpoint extension or blind whole-lane driving: at least one current cone must support the fit.
    if lane is not None and lane.has_lane and config.memory_enabled:
        edges = {e.colour: e.arc for e in (lane.left_edge, lane.right_edge) if e is not None}
        supporting = [c for c in cones if c.colour in edges
                      and abs(edges[c.colour].signed_distance(c.pos.x, c.pos.y)) <= config.outlier_distance_m]
        if not supporting:
            lane = _no_lane(lane)
        else:
            lane = replace(lane, visible_until_m=min(lane.visible_until_m, max(0.0, max(
                lane.centre_line.project(c.pos.x, c.pos.y) for c in supporting))))
    if config.continuity_enabled and lane is not None and lane.has_lane:
        reference = state.reference
        if not ready or (reference is not None and not _consistent(lane, reference, pose, config)):
            lane = _no_lane(lane)
            state = replace(state, rejected=True, tracks=prior_tracks, remembered_count=0)
        else:
            arc = lane.centre_line
            # Reused measurements do not renew the accepted reference's timestamp.
            if reference is None or pose.time_s > reference.pose.time_s + 1e-8:
                state = replace(state, reference=LaneReference(pose, arc.x, arc.y, arc.heading_rad,
                                                               arc.curvature_per_m, lane.visible_until_m))
    if (config.single_edge_hold_enabled and ready and lane is not None and lane.has_lane
            and fresh_lane.status == LaneStatus.BOTH_EDGES
            and (state.both_edge_reference is None or pose.time_s > state.both_edge_reference.pose.time_s + 1e-8)):
        arc = lane.centre_line
        state = replace(state, both_edge_reference=BothEdgeLaneReference(
            pose, arc.x, arc.y, arc.heading_rad, arc.curvature_per_m, lane.visible_until_m,
            lane.lane_width_m, lane.left_edge.colour.name, lane.right_edge.colour.name))
    if lane is None or not lane.has_lane:
        state = replace(state, holding_single_edge=False)
    if lane is None or not lane.has_lane or state.holding_single_edge:
        detector.lane_width_m = width_before
    memory.update_lane_tracking(state)
    return lane, augmented
