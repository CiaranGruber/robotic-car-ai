"""Offline-only connected-corridor follower. Not integrated into vehicle control or ROS selection.

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


class OfflineCorridorPolicy:
    """Experimental forward follower with finite cone/rollout bounds and latched loss/collision stops."""

    def __init__(self, max_left_curvature, max_right_curvature, memory: CarMemory,
                 config=CorridorConfig(), speed_m_per_s=0.5):
        if not all(math.isfinite(v) and v > 0 for v in (max_left_curvature, max_right_curvature, speed_m_per_s)) or speed_m_per_s > 2.0:
            raise ValueError("Experimental motion limits must be finite and positive")
        self.left, self.right, self.memory, self.config = max_left_curvature, max_right_curvature, memory, config
        self.speed = speed_m_per_s
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
            self.memory.update_corridor_experiment(replace(state, filtered_tracks=()))
            return self._stop(state.started)
        acquisition = _pose_at(temporal.poses, temporal.last_acquisition_s)
        if acquisition is None:
            self.memory.update_corridor_experiment(replace(state, filtered_tracks=()))
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
        self.path = cone_corridor(current, self.config)
        desired = pursuit_curvature(self.path, self.config.lookahead_m)
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
        # Same footprint as the existing test vehicle; explicit experiment constants, not admitted calibration.
        clearance = np.hypot(np.maximum.reduce([-0.198 - bx, np.zeros_like(bx), bx - 0.297]),
                             np.maximum(np.abs(by) - 0.125, 0.0))
        safe = np.min(clearance, axis=(1, 2)) >= 0.07  # 5 cm cone radius + 2 cm experiment margin.
        if not safe.any():
            return self._stop(True)
        costs = (candidates - desired) ** 2 + 0.1 * (candidates - state.curvature) ** 2
        costs[~safe] = np.inf
        curvature = float(candidates[np.argmin(costs)])
        self.memory.update_corridor_experiment(replace(state, started=True, curvature=curvature))
        return [DriveCommand(self.speed, curvature)]
