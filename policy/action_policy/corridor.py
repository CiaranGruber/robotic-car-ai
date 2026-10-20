"""Fixed-camera runtime corridor follower with measured-motion, route-support and lidar stop guards.

The geometry engine does not replace the node's sensor supervision or vehicle watchdog. Lidar is transported
using planar forward/no-slip odometry, with one pose per scan rather than per-ray deskew. All lidar returns
are checked against the commanded curved footprint; cone colour never exempts a lidar return.
"""
from dataclasses import dataclass, fields, replace
import logging
import math

import numpy as np

from policy.action_policy.corridor_experiment import OfflineCorridorPolicy, remaining_route, simplify_boundary
from policy.control.actions import DriveCommand
from policy.control.control import steering_for_curvature
from policy.input_output import Position
from policy.lane_detection.corridor import CorridorConfig
from policy.lane_detection.temporal import _body, _pose_at, _world
from policy.memory import CorridorExperimentMemory
from policy.obstacle_detection.obstacles import ObstacleDetection

_LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class CorridorControlConfig(CorridorConfig):
    """Startup settings for the selected runtime corridor policy."""
    target_speed_m_per_s: float = .4
    max_left_curvature_per_m: float = .48
    max_right_curvature_per_m: float = .92
    overspeed_m_per_s: float = .7
    full_range_steering: bool = False
    steering_settle_s: float = .15
    command_timeout_s: float = .2
    reaction_allowance_s: float = .25
    preferred_cone_clearance_m: float = .2
    clearance_cost_weight: float = 3.

    def __post_init__(self):
        super().__post_init__()
        if not isinstance(self.full_range_steering, bool):
            raise ValueError("corridor.full_range_steering must be boolean")
        if (not math.isfinite(self.preferred_cone_clearance_m)
                or not 0 < self.preferred_cone_clearance_m <= self.nominal_width_m/2
                or not math.isfinite(self.clearance_cost_weight) or not 0 <= self.clearance_cost_weight <= 100):
            raise ValueError("Invalid corridor preferred clearance/cost weight")
        if (not all(math.isfinite(v) and v > 0 for v in (
                self.target_speed_m_per_s, self.max_left_curvature_per_m,
                self.max_right_curvature_per_m, self.overspeed_m_per_s, self.steering_settle_s,
                self.command_timeout_s, self.reaction_allowance_s))
                or not self.command_timeout_s < self.reaction_allowance_s <= .5
                or not self.target_speed_m_per_s < self.overspeed_m_per_s <= 2
                or self.steering_settle_s > 1
                or max(self.max_left_curvature_per_m, self.max_right_curvature_per_m) > 2):
            raise ValueError("Invalid corridor speed/curvature limits")


def swept_lidar(points, curvature, front, rear, width, padding, length, min_points,
                *, initial_curvature=None, settle_distance_m=None, brake_after_m=None, delay_uncertainty_m=0.):
    """First travel distance at which enough returns intersect a conservatively sampled rectangular footprint."""
    if brake_after_m is not None and (initial_curvature is None or settle_distance_m is None):
        raise ValueError("Braking sweeps require an initial curvature and steering response distance")
    distances = np.linspace(0., length, max(2, math.ceil(length/.025)+1))
    quadrature_error = 0.
    if initial_curvature is None or settle_distance_m is None:
        angles = curvature*distances
        chord = distances*np.sinc(angles/(2*math.pi))
        xs, ys = chord*np.cos(angles/2), chord*np.sin(angles/2)
    else:
        scale = max(.001, settle_distance_m)
        def heading(s):
            driven = curvature*s+(initial_curvature-curvature)*scale*(1-np.exp(-s/scale))
            if brake_after_m is None:
                return driven
            delay = brake_after_m
            at_delay = curvature*delay+(initial_curvature-curvature)*scale*(1-math.exp(-delay/scale))
            k_at_delay = curvature+(initial_curvature-curvature)*math.exp(-delay/scale)
            stopped = at_delay+k_at_delay*scale*(1-np.exp(-np.maximum(0., s-delay)/scale))
            return np.where(s <= delay, driven, stopped)
        angles = heading(distances)
        middle = heading((distances[1:]+distances[:-1])/2)
        xs = np.r_[0., np.cumsum(np.diff(distances)*np.cos(middle))]
        ys = np.r_[0., np.cumsum(np.diff(distances)*np.sin(middle))]
        max_k = max(abs(curvature), abs(initial_curvature))
        # Composite midpoint integration error for unit tangent: |f''| <= K²+2K/scale, also across
        # the continuous-curvature target switch. Account for it separately from between-sample motion.
        quadrature_error = length*(distances[1]-distances[0])**2/24*(max_k**2+2*max_k/scale)
    rollout = tuple(zip(xs.tolist(), ys.tolist()))
    if not points:
        return math.inf, None, 0, rollout
    xy = np.array(points)
    if not np.isfinite(xy).all():
        raise ValueError("Nonfinite Cartesian lidar")
    # At most half a spatial sample separates a true pose from its nearest sample. Bound longitudinal and
    # lateral corner movement separately: forward translation must not widen a straight lane's side band.
    max_k = max(abs(curvature), abs(initial_curvature or 0.))
    half_step = (distances[1]-distances[0])/2
    turn = max_k*half_step
    extent, half_width = max(front, rear), width/2
    longitudinal = half_step+half_width*turn+extent*turn**2/2
    lateral = max_k*half_step**2/2+extent*turn+half_width*turn**2/2
    dx, dy = xy[None, :, 0]-xs[:, None], xy[None, :, 1]-ys[:, None]
    bx = np.cos(angles)[:, None]*dx+np.sin(angles)[:, None]*dy
    by = -np.sin(angles)[:, None]*dx+np.cos(angles)[:, None]*dy
    clearance = np.hypot(np.maximum.reduce([-rear-longitudinal-bx, np.zeros_like(bx), bx-front-longitudinal]),
                         np.maximum(np.abs(by)-half_width-lateral, 0.))
    delay_padding = np.zeros_like(distances)
    if brake_after_m is not None and delay_uncertainty_m > 0:
        # For this first-order spatial response, d(heading)/d(stop_delay)=k*(1-exp(-t/scale)),
        # t=max(0,s-stop_delay). Integrate its bound for centre displacement and add corner rotation.
        # It is zero at the common initial pose; a constant delay inflation falsely blocks nearby side cones.
        t = np.maximum(0., distances-max(0., brake_after_m-delay_uncertainty_m))
        response = 1-np.exp(-t/scale)
        # Include rotation of the expanded rectangle when composing the spatial and stop-delay bounds.
        corner = math.hypot(extent, half_width)+math.hypot(longitudinal, lateral)
        delay_padding = abs(curvature)*delay_uncertainty_m*(t-scale*response+corner*response)
    hit = clearance <= padding+quadrature_error+delay_padding[:, None]
    first = np.min(np.where(hit, distances[:, None], np.inf), axis=0)
    count = int(np.isfinite(first).sum())
    if count < min_points:
        return math.inf, None, count, rollout
    index = int(np.argsort(first)[min_points-1])
    return float(first[index]), Position(*points[index], 0.), count, rollout


def rounded_path(path, left_curvature, right_curvature):
    """Round observed polyline corners at the planning radius, retaining the measured forward endpoint.

    Only the first incoming straight may extend backwards, to retain its tangent as the vehicle enters a
    corner. There is no forward extension. Rollout/cone/lidar guards still decide whether to admit driving.
    """
    if len(path) < 3:
        return path
    points = simplify_boundary(path, .12)
    result = [points[0]]
    previous_cut = 0.
    for index in range(1, len(points)-1):
        a, b, c = points[index-1:index+2]
        incoming, outgoing = b-a, c-b
        before, after = np.linalg.norm(incoming), np.linalg.norm(outgoing)
        if min(before, after) < 1e-6:
            continue
        u, v = incoming/before, outgoing/after
        cross = float(u[0]*v[1]-u[1]*v[0])
        angle = math.acos(float(np.clip(u@v, -1., 1.)))
        radius = 1/(left_curvature if cross > 0 else right_curvature)
        cut = radius*math.tan(angle/2)
        if (abs(cross) < .2 or angle > 2.5
                or (index > 1 and cut+previous_cut > before) or cut > after*.9):
            result.append(b)
            previous_cut = 0.
            continue
        start = b-cut*u
        if index == 1 and cut > before:
            result = []  # Do not connect a later point backwards to the extended incoming tangent.
        centre = start+math.copysign(radius, cross)*np.array([-u[1], u[0]])
        theta = math.atan2(start[1]-centre[1], start[0]-centre[0])
        angles = np.linspace(theta, theta+math.copysign(angle, cross),
                             min(64, max(3, math.ceil(radius*angle/.05))))
        result.extend(centre+radius*np.c_[np.cos(angles), np.sin(angles)])
        previous_cut = cut
    result.append(points[-1])
    return remaining_route(tuple((float(x), float(y)) for x, y in result))


class CorridorPolicy:
    """Selects corridor geometry commands, applying latched runtime guards before existing actuation control."""

    def __init__(self, config, mpc_config, obstacle_config, avoidance_config, control_config):
        self.config, self.stop_config, self.obstacle_config = config, mpc_config, obstacle_config
        self.control_config = control_config
        self.footprint = (obstacle_config.car_front_m, avoidance_config.car_rear_m, avoidance_config.car_width_m)
        if not mpc_config.obstacle_stop_enabled:
            raise ValueError("corridor requires obstacle stopping")
        if min(self.footprint) <= 0 or obstacle_config.corridor_half_width_m < self.footprint[2]/2:
            raise ValueError("Corridor lidar band must cover the configured footprint")
        if config.target_speed_m_per_s < control_config.speed_feedback_min_m_per_s:
            raise ValueError("corridor target speed requires wheel-speed feedback")
        if (config.target_speed_m_per_s*mpc_config.obstacle_stop_distance_per_speed_s
                + mpc_config.obstacle_stop_margin_m > obstacle_config.corridor_length_m):
            raise ValueError("Corridor stopping distance exceeds lidar search")
        self.follower = None
        self.path, self.rollout = (), ()
        self.obstacles = None
        self.last_command = DriveCommand(0., 0.)
        self.stop_reason = "waiting for observations"
        self.steering_limited = False
        self._candidate_scans = {}
        self._braking_scan = None
        self._scan_inputs = None

    def _scan_for_command(self, curvature, observations, *, brake_after_m=None, length=None, delay_uncertainty_m=0.):
        if self._scan_inputs is None:
            poses = self.follower.memory.data.lane_tracking.poses
            lidar = observations.lidar_cartesian
            if not poses or lidar is None:
                return None
            now = poses[-1]
            scan_pose = _pose_at(poses, now.time_s-float(lidar.sensor_age))
            if scan_pose is None:
                return None
            points = [_body(*_world(p.pos.x, p.pos.y, scan_pose), now) for p in lidar.points]
            speed = float(observations.car.wheel_speed)
            measured = observations.car.ang_velocity.yaw/speed if speed > .05 else 0.
            self._scan_inputs = points, speed, measured
        points, speed, measured = self._scan_inputs
        control = self.control_config
        slope = (control.steering_left_action_per_curvature if curvature >= 0
                 else control.steering_right_action_per_curvature)
        mapped = (steering_for_curvature(curvature, control)-control.steering_centre_action)/slope
        front, rear, width = self.footprint
        padding = self.obstacle_config.corridor_half_width_m-width/2
        curvatures = (float(curvature),) if math.isclose(mapped, curvature, abs_tol=1e-12) else (float(curvature), mapped)
        results = [swept_lidar(points, k, front, rear, width, padding,
                              self.obstacle_config.corridor_length_m if length is None else length,
                              self.obstacle_config.min_points,
                              initial_curvature=measured,
                              settle_distance_m=max(speed, self.config.target_speed_m_per_s)*self.config.steering_settle_s,
                              brake_after_m=brake_after_m, delay_uncertainty_m=delay_uncertainty_m)
                   for k in curvatures]
        nearest = min(results, key=lambda r: r[0])
        return (*nearest[:3], results[0][3])

    def _command_guard(self, candidates, observations):
        speed = max(float(observations.car.wheel_speed), self.config.target_speed_m_per_s,
                    self.follower.memory.data.corridor_experiment.previous_speed_m_per_s)
        stopping = speed*(self.stop_config.obstacle_stop_distance_per_speed_s
                          + float(observations.lidar_cartesian.sensor_age)+self.config.reaction_allowance_s
                          )+self.stop_config.obstacle_stop_margin_m
        reaction = speed*(float(observations.lidar_cartesian.sensor_age)+self.config.reaction_allowance_s)
        braking = speed*self.stop_config.obstacle_stop_distance_per_speed_s
        delays = np.linspace(0., reaction, max(2, math.ceil(reaction/.025)+1))
        # The guard may stop at any time before the next deadline. Follow the admitted turn until each
        # sampled stop time, then centre steering while braking. The sweep bounds the paths between delays.
        delay_step = float(delays[1]-delays[0])
        safe = []
        for k in candidates:
            result = self._scan_for_command(float(k), observations)
            self._candidate_scans[float(k)] = result
            braking_clear = True
            for delay in delays:
                length = float(delay+braking+delay_step/2)
                escape = self._scan_for_command(float(k), observations, brake_after_m=float(delay),
                                               length=length, delay_uncertainty_m=delay_step/2)
                if escape is not None and escape[0] <= length:
                    self._braking_scan = escape
                    braking_clear = False
                    break
            # Missing initial pose history is handled by the runtime's stationary warm-up guard.
            safe.append(braking_clear and (result is None or result[0] > stopping))
        return np.array(safe)

    def _stop(self, memory, reason, latch=True):
        state = memory.data.corridor_experiment
        if latch and not state.stopped:
            _LOGGER.warning("Corridor stop: %s", reason)
        memory.update_corridor_experiment(replace(
            state, stopped=state.stopped or latch, stop_reason=reason, previous_speed_m_per_s=0., curvature=0.,
            started=state.started if latch else state.runtime_started))
        self.stop_reason = reason
        self.last_command = DriveCommand(0., 0., brake=True)
        return [self.last_command]

    def run_policy(self, observations, memory):
        self.path, self.rollout, self.obstacles = (), (), None
        self.steering_limited = False
        self._candidate_scans = {}
        self._braking_scan = None
        self._scan_inputs = None
        if observations.policy.is_first_policy_step:
            memory.update_corridor_experiment(CorridorExperimentMemory())
        if self.follower is None or self.follower.memory is not memory:
            geometry = CorridorConfig(**{f.name: getattr(self.config, f.name) for f in fields(CorridorConfig)})
            front, rear, width = self.footprint
            self.follower = OfflineCorridorPolicy(
                self.config.max_left_curvature_per_m, self.config.max_right_curvature_per_m,
                memory, geometry, self.config.target_speed_m_per_s, front_m=front, rear_m=rear, width_m=width,
                command_guard=self._command_guard,
                path_transform=lambda path: rounded_path(path, self.config.max_left_curvature_per_m,
                                                         self.config.max_right_curvature_per_m),
                terminal_straight=True, preferred_clearance_m=self.config.preferred_cone_clearance_m,
                clearance_cost_weight=self.config.clearance_cost_weight)
        state = memory.data.corridor_experiment
        if state.stopped:
            return self._stop(memory, state.stop_reason or "corridor geometry lost")
        required = (observations.cones, observations.car.wheel_speed,
                    observations.car.ang_velocity, observations.lidar_cartesian)
        if any(v is None for v in required):
            return self._stop(memory, "missing cones, speed, gyro or Cartesian lidar")
        if any(not math.isfinite(float(v.sensor_age)) or not 0 <= float(v.sensor_age) < .5 for v in required):
            return self._stop(memory, "stale or invalid sensor age")
        speed = float(observations.car.wheel_speed)
        if not math.isfinite(speed) or speed < 0 or speed > self.config.overspeed_m_per_s:
            return self._stop(memory, "wheel-speed limit exceeded")
        if self.stop_config.run_duration_s and observations.policy.policy_elapsed_s >= self.stop_config.run_duration_s:
            return self._stop(memory, "run duration reached")
        command = self.follower.run_policy(observations)[0]
        self.path = self.follower.path
        if command.speed_m_per_s <= 0:
            reason = "corridor geometry or motion unavailable"
            scans = [(k, r) for k, r in self._candidate_scans.items() if r is not None]
            stopping = max(speed, self.config.target_speed_m_per_s, state.previous_speed_m_per_s)*(
                self.stop_config.obstacle_stop_distance_per_speed_s
                + float(observations.lidar_cartesian.sensor_age)+self.config.reaction_allowance_s
                )+self.stop_config.obstacle_stop_margin_m
            if scans and all(r[0] <= stopping for _, r in scans):
                k, (distance, point, count, self.rollout) = min(scans, key=lambda pair: pair[1][0])
                self.obstacles = ObstacleDetection([], distance, point, count, k, observations.lidar_cartesian.sensor_age)
                reason = "lidar obstacle blocks available steering commands"
            if self._braking_scan is not None:
                distance, point, count, self.rollout = self._braking_scan
                self.obstacles = ObstacleDetection([], distance, point, count, 0., observations.lidar_cartesian.sensor_age)
                reason = "lidar obstacle in centered-steering braking path"
            return self._stop(memory, reason,
                              latch=memory.data.corridor_experiment.stopped)
        poses = memory.data.lane_tracking.poses
        if not poses:
            return self._stop(memory, "motion history unavailable")
        now = poses[-1]
        lidar = observations.lidar_cartesian
        scan_pose = _pose_at(poses, now.time_s-float(lidar.sensor_age))
        if scan_pose is None:
            return self._stop(memory, "waiting for lidar motion history", latch=state.runtime_started)
        conservative_speed = max(speed, command.speed_m_per_s, state.previous_speed_m_per_s)
        stopping = conservative_speed*(self.stop_config.obstacle_stop_distance_per_speed_s
                                      + float(lidar.sensor_age)+self.config.reaction_allowance_s
                                      )+self.stop_config.obstacle_stop_margin_m
        # Every runtime guard requests the same braking stop as an obstacle. The constructor's existing
        # coasting check requires this setting to cover coasting too when brake_action is zero.
        support = conservative_speed*(self.stop_config.obstacle_stop_distance_per_speed_s
                                     + float(observations.cones.sensor_age)+self.config.reaction_allowance_s
                                     )+self.stop_config.obstacle_stop_margin_m
        available = sum(math.dist(a, b) for a, b in zip(self.path, self.path[1:]))
        if available < support:
            return self._stop(memory, "connected path shorter than stopping support")
        if stopping > self.obstacle_config.corridor_length_m:
            return self._stop(memory, "stopping distance exceeds lidar coverage")
        control = self.control_config
        slope = (control.steering_left_action_per_curvature if command.curvature_per_m >= 0
                 else control.steering_right_action_per_curvature)
        applied = steering_for_curvature(command.curvature_per_m, control)
        mapped = (applied-control.steering_centre_action)/slope
        self.steering_limited = not math.isclose(mapped, command.curvature_per_m, abs_tol=1e-8)
        distance, point, count, self.rollout = self._candidate_scans[command.curvature_per_m]
        self.obstacles = ObstacleDetection([], distance, point, count, command.curvature_per_m, lidar.sensor_age)
        # Immediate confirmation on a scan containing enough returns; no extra unbudgeted policy-frame delay.
        if distance <= stopping:
            return self._stop(memory, "lidar obstacle in commanded turn")
        memory.update_corridor_experiment(replace(memory.data.corridor_experiment,
                                                 stop_reason="", previous_speed_m_per_s=command.speed_m_per_s,
                                                 runtime_started=True))
        self.stop_reason = ""
        self.last_command = command
        return [command]
