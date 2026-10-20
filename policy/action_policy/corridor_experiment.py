"""Connected-corridor geometry engine, also used by the guarded runtime corridor policy.

Uses actual observations, never simulator route vertices. Keeps the existing boundary observer running for
comparison, while a separate connected midpoint centreline can change direction through corners. Motion memory
is acquisition-aligned and then transformed to the current frame before following. The simple cone rollout
check does not certify continuous corridor clearance or replace the runtime lidar/obstacle safety pipeline.
"""
from dataclasses import replace
import math

import numpy as np

from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones
from policy.control.actions import DriveCommand
from policy.input_output import ConeColour, ConeDetection, Position
from policy.lane_detection.corridor import CorridorConfig, cone_corridor, pursuit_curvature
from policy.lane_detection.lane_detection import LaneDetectionConfig, LaneDetector
from policy.lane_detection.temporal import _body, _pose_at, _world, detect_temporal_lane
from policy.memory import CarMemory, CorridorExperimentMemory


def remaining_route(path):
    """Trim a transported polyline to its nearest point, preserving the connected forward remainder."""
    choices = []
    for index, (a, b) in enumerate(zip(path, path[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length2 = dx*dx+dy*dy
        if length2 < 1e-10:
            continue
        fraction = min(1., max(0., -(a[0]*dx+a[1]*dy)/length2))
        point = (a[0]+fraction*dx, a[1]+fraction*dy)
        choices.append((math.hypot(*point), index, point))
    if not choices:
        return ()
    _, index, point = min(choices)
    return (point, *path[index+1:])


def route_sides(path, cones, config):
    """Return fresh colour roles only with spatially spread support on a consistent signed side."""
    if len(path)<2:
        return {}
    points=np.array(path);starts=points[:-1];segments=np.diff(points,axis=0)
    lengths2=np.maximum(np.sum(segments*segments,axis=1),1e-10)
    supports={}
    for cone in cones:
        point=np.array((cone.pos.x,cone.pos.y))
        fractions=np.clip(np.sum((point-starts)*segments,axis=1)/lengths2,0,1)
        differences=point-starts-fractions[:,None]*segments
        index=int(np.argmin(np.linalg.norm(differences,axis=1)))
        signed=float((segments[index,0]*differences[index,1]-segments[index,1]*differences[index,0])/math.sqrt(lengths2[index]))
        distance=np.linalg.norm(differences[index])
        if (config.min_width_m/2 <= abs(signed)
                and distance <= config.max_width_m/2+.1):
            supports.setdefault(cone.colour,[]).append((tuple(point),signed))
    sides={}
    for colour,values in supports.items():
        for sign in (-1,1):
            points=[point for point,signed in values if sign*signed>0]
            if len(points)>=2 and max(math.dist(a,b) for a in points for b in points)>=.25:
                if colour in sides:
                    sides.pop(colour)
                    break
                sides[colour]=sign
    return sides


def route_support(path, cones, config, require_both=False, left_colour=None):
    sides=route_sides(path,cones,config)
    if left_colour is not None:
        sides={colour:sign for colour,sign in sides.items()
               if (sign>0)==(colour.name==left_colour)}
    return (len(sides)==2 and set(sides.values())=={-1,1}) if require_both else bool(sides)


def simplify_boundary(points, tolerance_m=.08):
    """Preserve real corners while removing detector-scale bends before inward offsetting."""
    points=np.asarray(points,dtype=float)
    if len(points)<=2:
        return points
    delta=points[-1]-points[0];length2=float(delta@delta)
    if length2<1e-10:
        return points[:1]
    fractions=np.clip((points-points[0])@delta/length2,0,1)
    distances=np.linalg.norm(points-points[0]-fractions[:,None]*delta,axis=1)
    index=int(np.argmax(distances))
    if distances[index]<=tolerance_m:
        return points[[0,-1]]
    return np.vstack((simplify_boundary(points[:index+1],tolerance_m)[:-1],
                      simplify_boundary(points[index:],tolerance_m)))


def remove_offset_loops(points):
    """Clip self-intersections of an inward offset instead of following small reversed loops."""
    points=np.asarray(points,dtype=float)
    for _ in range(len(points)):
        crossing=None
        for i in range(len(points)-1):
            r=points[i+1]-points[i]
            for j in range(i+2,len(points)-1):
                s=points[j+1]-points[j];delta=points[j]-points[i]
                cross=r[0]*s[1]-r[1]*s[0]
                if abs(cross)<1e-10:
                    continue
                t=(delta[0]*s[1]-delta[1]*s[0])/cross
                u=(delta[0]*r[1]-delta[1]*r[0])/cross
                if 0<t<1 and 0<u<1:
                    crossing=(i,j,points[i]+t*r)
                    break
            if crossing is not None:
                break
        if crossing is None:
            return points
        i,j,point=crossing
        points=np.vstack((points[:i+1],point,points[j+1:]))
    return points


def boundary_centre_path(cones, colour, left_colour, width_m, config, *, terminal_straight=False):
    """Offset a connected fresh boundary inward; never use remembered points as fresh evidence."""
    points = [(cone.pos.x,cone.pos.y) for cone in cones if cone.colour==colour and cone.pos.x>-.2]
    points = sorted(points,key=lambda point:math.hypot(*point))[:config.max_cones]
    # A cone immediately before a corner can be only centimetres from the corner cone. Treat that
    # local cluster as one boundary sample so greedy ordering cannot detour backwards through it.
    clusters=[]
    for point in points:
        group=next((group for group in clusters
                    if max(math.dist(point,other) for other in group)<min(.2,width_m*.2)),None)
        if group is None:
            clusters.append([point])
        else:
            group.append(point)
    points=[tuple(np.mean(group,axis=0)) for group in clusters]
    if len(points)<(2 if terminal_straight else 3):
        return ()
    start = min(range(len(points)),key=lambda i:math.hypot(*points[i]))
    chain=[points.pop(start)];tangent=np.array((1.,0.))
    while points and len(chain)<config.max_cones:
        options=[]
        for i,point in enumerate(points):
            vector=np.array(point)-chain[-1];distance=np.linalg.norm(vector)
            forward = vector[0] > 0 if len(chain)==1 else vector@tangent>-.7*distance
            if .1<distance<=config.max_cone_gap_m and forward:
                options.append((distance,i,vector/distance))
        if not options:
            break
        _,index,tangent=min(options,key=lambda choice:choice[0]);chain.append(points.pop(index))
    span = sum(math.dist(a,b) for a,b in zip(chain,chain[1:]))
    if len(chain)<(2 if terminal_straight else 3) or span < (.25 if terminal_straight else config.lookahead_m):
        return ()
    # Smoothing a right-angle boundary before inward offset can create reversed loops at the corner.
    # Fit its straight stretches first, retain the corner, then clip any residual offset self-intersection.
    points=simplify_boundary(chain)
    if len(points)<2:
        return ()
    if span < config.lookahead_m and len(points)>2:
        return ()  # A short terminal fit may establish a straight, never an under-observed corner.
    segments=np.diff(points,axis=0);segments/=np.linalg.norm(segments,axis=1)[:,None]
    normals=np.c_[-segments[:,1],segments[:,0]]
    offsets=np.vstack((normals[0],(normals[:-1]+normals[1:])/
                       np.maximum(1+np.sum(segments[:-1]*segments[1:],axis=1),.5)[:,None],normals[-1]))
    direction=-1 if colour.name==left_colour else 1
    centre=remove_offset_loops(points+direction*width_m/2*offsets)
    if terminal_straight and len(points)==2:
        # Two fresh, separated cones can support a short terminal straight with an already learned colour
        # role. Bridge backwards only to the vehicle's projection, within one bounded cone gap; never extend
        # beyond the measured forward endpoint or use this to refresh a remembered two-colour route.
        segment = centre[1]-centre[0]
        along = float(-centre[0]@segment/np.linalg.norm(segment))
        projection = centre[0]+along*segment/np.linalg.norm(segment)
        if along < 0:
            if -along > config.max_cone_gap_m or np.linalg.norm(projection) > width_m/2:
                return ()
            centre = np.vstack((projection, centre))
    return remaining_route(tuple((float(x),float(y)) for x,y in centre))


class OfflineCorridorPolicy:
    """Experimental forward follower with finite cone/rollout bounds and latched loss/collision stops."""

    def __init__(self, max_left_curvature, max_right_curvature, memory: CarMemory,
                 config=CorridorConfig(), speed_m_per_s=0.5, *, front_m=.297, rear_m=.198, width_m=.25,
                 command_guard=None, path_transform=None, terminal_straight=False,
                 preferred_clearance_m=.2, clearance_cost_weight=0.):
        if not all(math.isfinite(v) and v > 0 for v in (max_left_curvature, max_right_curvature, speed_m_per_s)) or speed_m_per_s > 2.0:
            raise ValueError("Experimental motion limits must be finite and positive")
        self.left, self.right, self.memory, self.config = max_left_curvature, max_right_curvature, memory, config
        self.speed = speed_m_per_s
        if not all(math.isfinite(v) and v > 0 for v in (front_m, rear_m, width_m)):
            raise ValueError("Corridor footprint dimensions must be finite and positive")
        self.front_m, self.rear_m, self.half_width_m = front_m, rear_m, width_m/2
        self.command_guard = command_guard
        self.path_transform = path_transform
        self.terminal_straight = terminal_straight
        if (not math.isfinite(preferred_clearance_m) or preferred_clearance_m <= 0
                or not math.isfinite(clearance_cost_weight) or not 0 <= clearance_cost_weight <= 100):
            raise ValueError("Invalid preferred cone clearance/cost weight")
        self.preferred_clearance_m, self.clearance_cost_weight = preferred_clearance_m, clearance_cost_weight
        self.detector = LaneDetector(LaneDetectionConfig(memory_enabled=True, max_curvature_per_m=2.0,
                                                       memory_max_age_s=2.0, memory_max_distance_m=1.0))
        self.path = ()
        self.last_lane = None

    def _stop(self, latch):
        state = self.memory.data.corridor_experiment
        self.memory.update_corridor_experiment(replace(state, stopped=state.stopped or latch, curvature=0.0))
        return [DriveCommand(0.0, 0.0)]

    def run_policy(self, observations, lanes=None):
        if observations.policy.is_first_policy_step:
            self.memory.update_corridor_experiment(CorridorExperimentMemory())
        state = self.memory.data.corridor_experiment
        self.path = ()
        if state.stopped:
            return self._stop(True)
        filtered = replace(observations, cones=filter_cones(
            observations.cones, ConeFilterConfig(check_straight_rows=False)))
        self.last_lane, augmented = detect_temporal_lane(filtered, self.memory, self.detector)
        temporal = self.memory.data.lane_tracking
        if not temporal.poses or temporal.last_acquisition_s is None or not filtered.cones:
            self.memory.update_corridor_experiment(replace(
                state, filtered_tracks=(), route=(), route_seen_s=None, route_seen_distance_m=None,
                runtime_route=(), runtime_route_seen_s=None, runtime_route_seen_distance_m=None))
            return self._stop(state.started)
        acquisition = _pose_at(temporal.poses, temporal.last_acquisition_s)
        if acquisition is None:
            self.memory.update_corridor_experiment(replace(
                state, filtered_tracks=(), route=(), route_seen_s=None, route_seen_distance_m=None,
                runtime_route=(), runtime_route_seen_s=None, runtime_route_seen_distance_m=None))
            return self._stop(state.started)
        now = temporal.poses[-1]
        smoothed = []
        unused = list(state.filtered_tracks)
        for track in temporal.tracks:
            choices = [(math.hypot(track.x - old.x, track.y - old.y), i) for i, old in enumerate(unused)
                       if track.colour == old.colour]
            if choices:
                distance, index = min(choices)
                if distance < 0.15:
                    old = unused.pop(index)
                    fraction = 0.3 if track.seen_s > old.seen_s else 0.0
                    track = replace(track, x=old.x + fraction * (track.x - old.x),
                                    y=old.y + fraction * (track.y - old.y))
            smoothed.append(track)
        state = replace(state, filtered_tracks=tuple(smoothed))
        self.memory.update_corridor_experiment(state)
        current = []
        for track in smoothed:
            x, y = _body(track.x, track.y, now)
            if x >= -0.5 and math.hypot(x, y) <= 4.0:
                current.append(ConeDetection(Position(x, y, track.z), ConeColour[track.colour], track.confidence))
        candidate = cone_corridor(current, self.config)
        self.path = candidate
        path_is_fresh = False
        if self.config.route_memory_enabled:
            fresh_now = [replace(cone,pos=Position(*_body(*_world(cone.pos.x,cone.pos.y,acquisition),now),cone.pos.z))
                         for cone in filtered.cones]
            both_supported = route_support(candidate,fresh_now,self.config,require_both=True,
                                           left_colour=state.left_colour)
            path_is_fresh = both_supported
            if both_supported and state.left_colour is None:
                sides=route_sides(candidate,fresh_now,self.config)
                state=replace(state,left_colour=next(colour.name for colour,sign in sides.items() if sign>0))
            if (both_supported and len(candidate) >= 3
                    and sum(math.dist(a,b) for a,b in zip(candidate,candidate[1:])) > self.config.lookahead_m+.4):
                state = replace(state, route=tuple(_world(x,y,now) for x,y in candidate),
                                route_seen_s=acquisition.time_s, route_seen_distance_m=acquisition.distance_m)
            self.path = candidate
            if state.route_seen_s is not None:
                if (0 <= now.time_s-state.route_seen_s <= self.config.route_max_age_s
                        and 0 <= now.distance_m-state.route_seen_distance_m <= self.config.route_max_distance_m):
                    saved = remaining_route(tuple(_body(x,y,now) for x,y in state.route))
                    if not both_supported:
                        self.path = saved if route_support(saved,fresh_now,self.config,left_colour=state.left_colour) else ()
                elif not both_supported:
                    self.path = ()
            elif not both_supported:
                self.path = ()
            self.memory.update_corridor_experiment(state)
        if self.config.route_memory_enabled and state.left_colour is not None:
            counts = {colour:sum(cone.colour==colour for cone in fresh_now)
                      for colour in (ConeColour.BLUE,ConeColour.YELLOW)}
            fresh_geometry = fresh_now
            if self.path_transform is not None:
                # Runtime corner construction needs stable positions, while support must remain fresh.
                # Match ONLY this frame's detections to the smoothed motion-compensated tracks; never add
                # a remembered cone that was absent from the current image.
                fresh_geometry = []
                for cone in fresh_now:
                    matches = [(math.hypot(c.pos.x-cone.pos.x, c.pos.y-cone.pos.y), c) for c in current
                               if c.colour == cone.colour]
                    distance, matched = min(matches, key=lambda pair: pair[0]) if matches else (math.inf, cone)
                    fresh_geometry.append(matched if distance < .15 else cone)
            if not both_supported:
                # Prefer current connected boundary geometry whenever it covers the lookahead. A short or
                # occluded dominant row must not hide a usable smaller row. Remembered paths bridge gaps only.
                measured_paths = []
                for colour in sorted(counts,key=counts.get,reverse=True):
                    measured = boundary_centre_path(fresh_geometry,colour,state.left_colour,
                                                    self.config.nominal_width_m,self.config,
                                                    terminal_straight=self.terminal_straight)
                    if measured and pursuit_curvature(measured,self.config.lookahead_m) is not None:
                        measured_paths.append(measured)
                        if not self.terminal_straight:
                            break
                if measured_paths:
                    # A short terminal straight must not hide a longer observed route on the other side.
                    self.path = max(measured_paths, key=lambda p: sum(math.dist(a,b) for a,b in zip(p,p[1:])))
                    path_is_fresh = True
        if self.path_transform is not None:
            proposed = self.path_transform(self.path)
            state = self.memory.data.corridor_experiment
            retained = ()
            if (self.config.route_memory_enabled and state.runtime_route_seen_s is not None
                    and 0 <= now.time_s-state.runtime_route_seen_s <= self.config.route_max_age_s
                    and 0 <= now.distance_m-state.runtime_route_seen_distance_m <= self.config.route_max_distance_m):
                retained = remaining_route(tuple(_body(x,y,now) for x,y in state.runtime_route))
                if not route_support(retained, fresh_now, self.config, left_colour=state.left_colour):
                    retained = ()
            span = lambda p: sum(math.dist(a,b) for a,b in zip(p,p[1:]))
            old_k = pursuit_curvature(retained, self.config.lookahead_m)
            new_k = pursuit_curvature(proposed, self.config.lookahead_m)
            revealed_turn = (path_is_fresh and span(proposed) >= self.config.lookahead_m+.4
                             and old_k is not None and new_k is not None and old_k*new_k >= -.01
                             and abs(new_k) > abs(old_k)+.25*min(self.left, self.right))
            if not revealed_turn and (span(retained) >= self.config.lookahead_m+.4 or span(retained) > span(proposed)):
                # Keep the original anchor while this observed rounded plan still covers pursuit/support.
                # A supporting frame does not renew its age/distance limits. Fresh full geometry revealing
                # a stronger turn may replace it; a saved straight must not hide an observed upcoming corner.
                self.path = retained
            else:
                self.path = proposed
                if path_is_fresh and span(proposed) >= self.config.lookahead_m+.4:
                    state = replace(
                        state, runtime_route=tuple(_world(x,y,now) for x,y in proposed),
                        runtime_route_seen_s=acquisition.time_s,
                        runtime_route_seen_distance_m=acquisition.distance_m)
                    self.memory.update_corridor_experiment(state)
        desired = pursuit_curvature(self.path,self.config.lookahead_m)
        if desired is None:
            return self._stop(state.started)
        dt = observations.policy.dt
        if not math.isfinite(dt) or dt < 0:
            return self._stop(True)
        change = 4.0 * dt
        low, high = max(-self.right, state.curvature - change), min(self.left, state.curvature + change)
        candidates = np.unique(np.r_[np.linspace(low, high, 25), np.clip(desired, low, high)])
        # Constant-curvature lookahead; sample every <=2.5 cm at the experimental target speed.
        horizon_m = max(float(observations.car.wheel_speed), self.speed) * 0.8
        sample_count = max(2, math.ceil(horizon_m / 0.025) + 1)
        distances = np.linspace(0.0, horizon_m, sample_count)
        angles = candidates[:, None] * distances
        chord = distances * np.sinc(angles / (2 * math.pi))
        xs, ys = chord * np.cos(angles / 2), chord * np.sin(angles / 2)
        points = np.array([(c.pos.x, c.pos.y) for c in current[:self.config.max_cones]])
        dx, dy = points[None, None, :, 0] - xs[:, :, None], points[None, None, :, 1] - ys[:, :, None]
        bx = np.cos(angles)[:, :, None] * dx + np.sin(angles)[:, :, None] * dy
        by = -np.sin(angles)[:, :, None] * dx + np.cos(angles)[:, :, None] * dy
        # Runtime supplies the configured footprint; offline callers retain their original test dimensions.
        clearance = np.hypot(np.maximum.reduce([-self.rear_m - bx, np.zeros_like(bx), bx - self.front_m]),
                             np.maximum(np.abs(by) - self.half_width_m, 0.0))
        minimum_clearance = np.min(clearance, axis=(1, 2))
        safe = minimum_clearance >= 0.07  # 5 cm cone radius + 2 cm experiment margin.
        if self.command_guard is not None:
            safe &= self.command_guard(candidates, observations)
        if not safe.any():
            return self._stop(True)
        costs = (candidates - desired) ** 2 + 0.1 * (candidates - state.curvature) ** 2
        # Prefer room before a hard guard is reached, instead of pursuing the centre target into a tight corner.
        # This cost cannot admit any candidate rejected by cone or runtime lidar checks.
        costs += self.clearance_cost_weight*(np.maximum(0., self.preferred_clearance_m-minimum_clearance)
                                             /self.preferred_clearance_m)**2
        costs[~safe] = np.inf
        curvature = float(candidates[np.argmin(costs)])
        self.memory.update_corridor_experiment(replace(state, started=True, curvature=curvature))
        return [DriveCommand(self.speed, curvature)]
