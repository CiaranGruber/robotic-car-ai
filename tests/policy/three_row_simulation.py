"""Exactly three finite straight cone rows and an explicitly declared open turnaround apron.

The simulated camera alone loses cones; lidar still sees every physical cone within 4m. Removed-cone profiles
also remove those physical cones from lidar and contact scoring. Row IDs, true pose, dropout triggers and the
goal gate stay entirely inside the simulator and are never supplied to ThreeRowPolicy or its network.
"""
from dataclasses import dataclass, field, replace
import math

import numpy as np

from policy.input_output import (AngularVelocity, CarObservations, ConeBatch, ObservedCarState,
                                 PolicyState, SensorAge, WheelSpeed)
from tests.policy.corridor_runtime_simulation import cone_lidar
from tests.policy.curved_lanes import detections, SIM_STEP_S, STEERING_TIME_CONSTANT_S
from tests.policy.real_car_measurements import LAB_CAR
from tests.policy.stage1_scenarios import (body_corners, CAR_FRONT_M, CAR_REAR_M, CAR_WIDTH_M,
                                          CONE_RADIUS_M, POLICY_PERIOD_S)
from policy.input_output import ConeColour


@dataclass(frozen=True)
class ThreeRowTrack:
    """Physical cone placement and a declared scoring floor, in metres."""
    divider_length_m: float = 3.
    row_gap_m: float = 1.
    outer_extension_m: float = 1.
    cone_spacing_m: float = .5
    placement_jitter_m: float = .025
    apron_depth_m: float = 4.
    apron_side_m: float = .9
    placement_seed: int = 17
    removed_fraction: float = 0.
    remove_divider_tip: bool = False

    def __post_init__(self):
        for name in ("divider_length_m", "row_gap_m", "outer_extension_m", "cone_spacing_m",
                     "apron_depth_m", "apron_side_m"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError("Three-row dimensions must be positive and finite")
        if not 0 <= self.placement_jitter_m < .1 or not 0 <= self.removed_fraction < .5:
            raise ValueError("Invalid three-row placement disturbance")

    @property
    def outer_end_m(self):
        return self.divider_length_m+self.outer_extension_m

    def row_cones(self):
        rng = np.random.default_rng(self.placement_seed)
        found = []
        for row, colour, y, length in (
                (0, ConeColour.BLUE, -self.row_gap_m/2, self.outer_end_m),
                (1, ConeColour.YELLOW, self.row_gap_m/2, self.divider_length_m),
                (2, ConeColour.BLUE, 1.5*self.row_gap_m, self.outer_end_m)):
            positions = [0.]
            while positions[-1]+self.cone_spacing_m*1.2 < length:
                positions.append(positions[-1]+self.cone_spacing_m*rng.uniform(.8, 1.2))
            positions.append(length)
            for index, x in enumerate(positions):
                if self.remove_divider_tip and row == 1 and index == len(positions)-1:
                    continue
                # Random physical gaps retain terminal markers; missing-terminal uncertainty is tested separately.
                if 0 < index < len(positions)-1 and rng.random() < self.removed_fraction:
                    continue
                dx, dy = rng.uniform(-self.placement_jitter_m, self.placement_jitter_m, 2)
                found.append((row, colour, x+dx, y+dy))
        return found

    def cones(self):
        return [(colour, float(row), x, y) for row, colour, x, y in self.row_cones()]

    def rectangles(self):
        w, end, outer = self.row_gap_m, self.divider_length_m, self.outer_end_m
        # The tiny slit represents a continuous divider: crossing between its cones before the tip is a
        # lane excursion even if no physical cone is hit. There is no invented fourth/cap cone row.
        return ((-1.2, end, -w/2, w/2-1e-6), (-1.2, end, w/2+1e-6, 1.5*w),
                (end, outer, -w/2, 1.5*w),
                (outer, outer+self.apron_depth_m, -w/2-self.apron_side_m, 1.5*w+self.apron_side_m))

    def outside_distance(self, x, y):
        values = []
        for lo, hi, bottom, top in self.rectangles():
            dx, dy = max(lo-x, 0., x-hi), max(bottom-y, 0., y-top)
            values.append(math.hypot(dx, dy) if dx or dy else -min(x-lo, hi-x, y-bottom, top-y))
        return min(values)

    def footprint_outside_distance(self, corners):
        xs = sorted({v for rectangle in self.rectangles() for v in rectangle[:2]})
        ys = sorted({v for rectangle in self.rectangles() for v in rectangle[2:]})
        worst = -math.inf
        for a, b in zip(corners, np.roll(corners, -1, axis=0)):
            delta = b-a
            cuts = [0., 1.]
            for dim, boundaries in enumerate((xs, ys)):
                if abs(delta[dim]) > 1e-12:
                    cuts.extend(t for boundary in boundaries if 0 < (t := (boundary-a[dim])/delta[dim]) < 1)
            cuts = sorted(set(cuts))
            samples = [*cuts, *((lo+hi)/2 for lo, hi in zip(cuts, cuts[1:]))]
            worst = max(worst, *(self.outside_distance(*(a+t*delta)) for t in samples))
        return worst


@dataclass(frozen=True)
class ThreeRowCase:
    """Independent physical layout, sensor corruption and initial-condition seeds."""
    track: ThreeRowTrack = field(default_factory=ThreeRowTrack)
    seed: int = 101
    profile: str = "nominal"
    start_left_m: float = 0.
    start_heading_deg: float = 0.
    duration_s: float = 80.
    trim_per_m: float = -.03
    max_left_curvature_per_m: float = 1.
    max_right_curvature_per_m: float = .92


def simulate(case, policy):
    """Closed-loop no-slip simulation with the shared detector, latency, lag and full-body contact rules."""
    track = case.track
    rng = np.random.default_rng(case.seed)
    car = replace(LAB_CAR, camera_yaw_deg=0., rolling_start=False,
                  max_left_curvature_per_m=case.max_left_curvature_per_m,
                  max_right_curvature_per_m=case.max_right_curvature_per_m,
                  straight_curvature_per_m=case.trim_per_m)
    physical = track.row_cones()
    cones = [(colour, float(row), x, y) for row, colour, x, y in physical]
    world = np.array([(x, y) for _, _, x, y in cones])
    x, y, h, speed, driven = -.6, case.start_left_m, math.radians(case.start_heading_deg), 0., 0.
    latency = round(car.detection_latency_s/POLICY_PERIOD_S)
    dead = round(car.speed_dead_time_s/SIM_STEP_S)
    queue, speeds = [], [0.]*(dead+1)
    command_speed, command_k, brake = 0., 0., False
    brake_time, brake_speed = None, 0.
    touched, worst, worst_pose = set(), 0., None
    sensor_started, mask_start, return_seen = False, None, False
    empty_since, max_empty = None, 0.
    log, policy_log = [], []
    goal = False
    for step in range(round(case.duration_s/SIM_STEP_S)):
        now = step*SIM_STEP_S
        if step % round(POLICY_PERIOD_S/SIM_STEP_S) == 0:
            if mask_start is None and x >= track.divider_length_m-.7:
                mask_start = now
            elapsed = -1. if mask_start is None else now-mask_start
            camera_cones = cones
            if case.profile == "terminal_loss":
                camera_cones = [c for c in cones if c[1] == 1 or c[2] < track.divider_length_m-.7]
            if case.profile == "divider_hidden" and 0 <= elapsed < 2.:
                camera_cones = [c for c in cones if c[1] != 1]
            elif case.profile == "outer_hidden" and 0 <= elapsed < 1.5:
                camera_cones = [c for c in cones if c[1] != 0]
            detected = detections(camera_cones, (x, y, h), car, rng)
            probability = .25 if case.profile == "random_drops" else .03
            detected = [cone for cone in detected if rng.random() >= probability]
            if case.profile == "short_all_hidden" and 0 <= elapsed < .4:
                detected = []
            if case.profile == "long_all_hidden" and elapsed >= 2.:
                detected = []
            queue.append(detected)
            batch = queue.pop(0) if len(queue) > latency else None
            sensor_started |= batch is not None
            if batch is not None:
                if batch:
                    empty_since = None
                else:
                    empty_since = now if empty_since is None else empty_since
                    max_empty = max(max_empty, now-empty_since+POLICY_PERIOD_S)
            cones_obs = None if batch is None else ConeBatch(
                batch, SensorAge(car.detection_latency_s, round((100+now-car.detection_latency_s)*1e9)), 0.)
            gyro_age = 1. if case.profile == "stale_gyro" and now >= 3. else 0.
            wheel_age = 1. if case.profile == "stale_wheel" and now >= 3. else 0.
            obs = CarObservations(cones=cones_obs, fiducials=None, lidar_scan=None,
                lidar_cartesian=cone_lidar(cones, (x, y, h), now),
                car=ObservedCarState(None, AngularVelocity(0., 0., speed*driven, SensorAge(gyro_age, None)),
                                     None, WheelSpeed(speed, SensorAge(wheel_age, None))),
                policy=PolicyState(0. if step == 0 else POLICY_PERIOD_S, now, step == 0))
            command = policy.run_policy(obs)[0]
            command_speed, command_k, brake = command.speed_m_per_s, command.curvature_per_m, command.brake
            state = policy.state
            policy_log.append((now, command_speed, command_k, len(batch or ()), bool(state.route),
                               state.stop_reason, len(state.tracks)))
        speeds.append(command_speed)
        delayed_speed = speeds.pop(0)
        if brake:
            if brake_time is None:
                brake_time, brake_speed = 0., speed
            brake_time += SIM_STEP_S
            speed = brake_speed*max(0., 1-max(0., brake_time-car.speed_dead_time_s)/.8)
            speeds = [0.]*len(speeds)
        else:
            brake_time = None
            speed += (delayed_speed-speed)*SIM_STEP_S/(car.speed_time_constant_s or .3)
        target = float(np.clip(command_k, -car.max_right_curvature_per_m,
                               car.max_left_curvature_per_m))+car.straight_curvature_per_m
        driven += (target-driven)*SIM_STEP_S/STEERING_TIME_CONSTANT_S
        h += speed*driven*SIM_STEP_S/2
        x, y = x+speed*math.cos(h)*SIM_STEP_S, y+speed*math.sin(h)*SIM_STEP_S
        h += speed*driven*SIM_STEP_S/2
        log.append((now, x, y, h, speed))
        c, s = math.cos(h), math.sin(h)
        dx, dy = world[:, 0]-x, world[:, 1]-y
        bx, by = c*dx+s*dy, -s*dx+c*dy
        clearance = np.hypot(np.maximum.reduce([-CAR_REAR_M-bx, np.zeros_like(bx), bx-CAR_FRONT_M]),
                             np.maximum(np.abs(by)-CAR_WIDTH_M/2, 0.))
        touched.update(np.flatnonzero(clearance < CONE_RADIUS_M).tolist())
        outside = track.footprint_outside_distance(body_corners(x, y, h))
        if outside > worst:
            worst, worst_pose = outside, (x, y, h)
        return_seen |= x > track.outer_end_m and h > math.pi/2
        heading_error = abs(math.atan2(math.sin(h-math.pi), math.cos(h-math.pi)))
        goal = bool(return_seen and x <= .25 and abs(y-track.row_gap_m) <= .3
                    and heading_error <= math.radians(20))
        if goal or policy.state.stopped and speed < .015:
            break
        if now > 5 and not sensor_started:
            break
    metrics = {"completed": goal, "cones_touched": len(touched), "max_outside_m": max(0., worst),
               "safe_completion": goal and not touched and worst <= 1e-8,
               "duration_s": float(now), "distance_m": float(np.sum(np.array(log)[:, 4])*SIM_STEP_S),
               "stop_reason": policy.state.stop_reason, "latched_stop": policy.state.stopped,
               "guard_interventions": policy.guard_interventions, "max_empty_camera_s": float(max_empty),
               "max_preview_empty_age_s": policy.max_empty_s}
    metrics["worst_outside_pose"] = worst_pose
    metrics["goal_speed_m_per_s"] = float(speed) if goal else None
    return {"metrics": metrics, "trajectory": np.array(log), "policy_log": policy_log}
