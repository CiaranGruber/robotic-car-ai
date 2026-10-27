"""One simulated run: the track, the car's sensors on their own clocks, the policy node's rules and the car.

Time moves on in PLANT_STEP_S steps. Each sensor takes a frame at its own rate and delivers it after its latency, as
the DREAM components publish them. The node part follows scripts/policy_node.py with the node settings of
config/ai4r_policy.yaml:
- one policy step per new message of the policy_update_mode sensor, or a timer at policy_update_rate_hz;
- an observation older than its sensor_timeout_s reaches the policy as None;
- the node starts in state 2 (zero actions). After start_delay_s a simulated operator requests state 3, which the
  node refuses while a required sensor is missing or stale, or no cone batch had cones within cone_empty_timeout_s.
  The operator retries every OPERATOR_RETRY_S until the node starts;
- in state 3 the same checks run at supervision_rate_hz and drop the node to state 2, where it stays, as on the car.
The vehicle (vehicle.py) only moves by the commands the node publishes.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import logging
import math
import traceback
from typing import Any, Callable

import numpy as np
import yaml

from policy.input_output import (AngularVelocity, CarActions, CarObservations, CarOrientation, ConeBatch, FiducialBatch,
                                 LidarCartesian, ObservedCarState, PolicyState, SensorAge, WheelSpeed)
from policy.memory import CarMemory
from policy.policy_runner import MovementPolicy
from scripts.mcap_export import DEFAULT_SETTINGS_PATH, build_policy, read_settings
from tests.policy.car_models import CAR_20, CarModel
from tests.policy.obstacle_simulation import body_polygon, polygon_gap
from tests.policy.real_car_measurements import LAB_CAR
from tests.policy.simulator.sensors import SensorSettings, Sensors, cone_contacts
from tests.policy.simulator.track import ROOT, Track
from tests.policy.simulator.vehicle import SimulatedCar

_LOGGER = logging.getLogger(__name__)

PLANT_STEP_S = 0.01
"""Simulation time step in seconds (100 Hz)."""
OPERATOR_RETRY_S = 0.5
"""How often the simulated operator asks again for state 3 while the node refuses to start, in seconds."""
STOPPED_FOR_S = 2.0
"""A run ends once the car has stood still for this long after it drove, in seconds."""
OFF_ROAD_END_M = 0.5
"""A run ends once the car's centre is this far outside every road, in metres."""

ZERO_STATE, POLICY_STATE = 2, 3
"""The policy node's states: publishing zero actions, and publishing policy actions."""


@dataclass(frozen=True)
class SensorTiming:
    """When one sensor takes frames and how late they arrive."""
    period_s: float
    latency_s: float


CAMERA_TIMING = SensorTiming(0.1, LAB_CAR.detection_latency_s)
"""The cone and marker camera: 10 Hz, batches about 0.2 s old (real_car_measurements.LAB_CAR)."""
LIDAR_TIMING = SensorTiming(0.1, 0.15)
"""The lidar: 10 Hz scans delivered 0.15 s after they start, so they are 0.15-0.25 s old when the policy uses them.
The recordings showed 0.15-0.3 s (obstacle_simulation.LIDAR_AGE_S). Older scans would sit on the intersection
navigator's 0.3 s staleness limit (intersection.sensor_timeout_s); test that with a slower lidar by hand."""
WHEEL_TIMING = SensorTiming(0.02, 0.02)
"""Wheel speed telemetry, about 50 Hz."""
IMU_TIMING = SensorTiming(0.01, 0.01)
"""The IMU, 100 Hz."""


class _Stream:
    """One sensor's frames: taken every period, each delivered latency later."""

    def __init__(self, timing: SensorTiming):
        self.timing = timing
        self.next_at = 0.0
        self.pending: deque = deque()
        self.latest: tuple[float, Any] | None = None
        """The latest delivered frame: (acquisition time, value)."""
        self.new = False
        """Whether a frame was delivered in the latest simulation step."""

    def tick(self, now: float, capture: Callable[[], Any]):
        self.new = False
        if now + 1e-9 >= self.next_at:
            self.pending.append((now + self.timing.latency_s, now, capture()))
            self.next_at += self.timing.period_s
        while self.pending and self.pending[0][0] <= now + 1e-9:
            _, acquired, value = self.pending.popleft()
            self.latest, self.new = (acquired, value), True

    def age(self, now: float) -> float:
        return math.inf if self.latest is None else now - self.latest[0]


def aruco_settings() -> dict:
    """
    :return: The aruco_detector parameters from config/aruco_detector.yaml.
    """
    return yaml.safe_load((ROOT / "config/aruco_detector.yaml").read_text())["/**/aruco_detector"]["ros__parameters"]


def run_settings(track: Track, overrides: dict | None = None) -> dict:
    """
    :param overrides: Settings by full parameter name that replace the file's and the track's.
    :return: config/ai4r_policy.yaml's settings by full parameter name, with the track's and then the overrides on top.
    """
    settings = read_settings(DEFAULT_SETTINGS_PATH)
    settings.update(track.policy_settings)
    settings.update(overrides or {})
    return settings


def _inside(polygon: np.ndarray, x: float, y: float) -> bool:
    """Even-odd rule: whether the point is inside the polygon."""
    xs, ys = polygon[:, 0], polygon[:, 1]
    crossing = (ys > y) != (np.roll(ys, -1) > y)
    with np.errstate(divide="ignore", invalid="ignore"):
        at_x = xs + (y - ys) * (np.roll(xs, -1) - xs) / (np.roll(ys, -1) - ys)
    return bool(np.count_nonzero(crossing & (x < at_x)) % 2)


def _distance_outside(polygon: np.ndarray, x: float, y: float) -> float:
    """
    :return: How far the point is outside the polygon in metres, 0.0 inside it.
    """
    if _inside(polygon, x, y):
        return 0.0
    a, b = polygon, np.roll(polygon, -1, axis=0)
    ab = b - a
    t = np.clip(((np.array([x, y]) - a) * ab).sum(axis=1) / np.maximum((ab * ab).sum(axis=1), 1e-12), 0.0, 1.0)
    return float(np.hypot(*(a + t[:, None] * ab - (x, y)).T).min())


@dataclass
class Event:
    """Something that happened in a run, for the log and the drawing."""
    time_s: float
    text: str


@dataclass
class RunSummary:
    """How a run went."""
    track: str
    ended_because: str
    time_s: float
    distance_m: float
    cones_touched: int
    obstacles_touched: int
    max_off_road_m: float
    node_state: int
    node_reason: str
    events: list[Event] = field(default_factory=list)


class Simulation:
    """One run of the policy on a track, one PLANT_STEP_S at a time."""

    def __init__(self, track: Track, settings: dict | None = None, car_model: CarModel = CAR_20,
                 sensor_settings: SensorSettings | None = None, seed: int = 0, start_delay_s: float = 1.0):
        """
        :param track: The test area.
        :param settings: Policy and node settings by full parameter name; default run_settings(track).
        :param car_model: The car's measured drive and steering response.
        :param sensor_settings: How the sensors behave; default the measured lab car and config/aruco_detector.yaml.
        :param seed: Seed of the sensor noise, so a run can be repeated exactly.
        :param start_delay_s: When the simulated operator first requests state 3, in seconds.
        """
        self.track = track
        self.settings = run_settings(track) if settings is None else settings
        aruco = aruco_settings()
        if sensor_settings is None:
            sensor_settings = SensorSettings(min_consecutive_frames=int(aruco["min_consecutive_frames"]),
                                             allowed_marker_ids=tuple(aruco.get("allowed_marker_ids", []) or ()))
        self.dictionary_name = str(aruco["dictionary_name"])
        self.sensors = Sensors(track, sensor_settings, np.random.default_rng(seed))
        self.car = SimulatedCar(car_model, track.start)
        self.policy: MovementPolicy = build_policy(self.settings)
        self.memory = CarMemory()
        self.time_s = 0.0
        self.camera, self.lidar = _Stream(CAMERA_TIMING), _Stream(LIDAR_TIMING)
        self.wheel, self.imu = _Stream(WHEEL_TIMING), _Stream(IMU_TIMING)
        self.mode = str(self.settings.get("policy_update_mode", "cone_detection"))
        self.timer_period_s = 1.0 / float(self.settings.get("policy_update_rate_hz", 50.0))
        self.supervision_period_s = 1.0 / float(self.settings.get("supervision_rate_hz", 20.0))
        self.required = list(self.settings.get("required_sensors", []))
        self.cone_empty_timeout_s = float(self.settings.get("cone_empty_timeout_s", 1.0))
        self.start_delay_s = start_delay_s
        self.node_state, self.node_reason = ZERO_STATE, "Startup: waiting for the operator"
        self.ever_started = False
        self.next_request_at, self.next_supervision_at, self.next_timer_at = start_delay_s, 0.0, 0.0
        self.policy_started_at, self.previous_step_at, self.heading_reference = None, None, None
        self.last_nonempty_cones_at: float | None = None
        self.observations: CarObservations | None = None
        """The observations of the latest policy step."""
        self.actions: CarActions | None = None
        """The actions of the latest policy step."""
        self.policy_steps = 0
        self.last_marker_command = None
        self.last_intersection_phase = None
        self.manual_command: tuple[float, float] | None = None
        """Drive and steering actions that replace the node's commands while set, as a person driving by hand."""
        self.cone_xy = np.array([(c.x, c.y) for c in track.cones]).reshape(-1, 2)
        self.boxes = [o.polygon() for o in track.obstacles]
        self.cones_touched: set[int] = set()
        self.obstacles_touched: set[int] = set()
        self.max_off_road_m = 0.0
        self.events: list[Event] = []
        self.stopped_since: float | None = None
        self.ended_because: str | None = None

    # ---- Sensors ----------------------------------------------------------------------------------------------
    def _capture_camera(self):
        pose = self.car.state.pose
        return self.sensors.camera_cones(pose), self.sensors.camera_markers(pose)

    def _capture_lidar(self):
        return self.sensors.lidar(self.car.state.pose)

    def _capture_wheel(self):
        return self.car.state.speed_m_per_s

    def _capture_imu(self):
        state = self.car.state
        return state.heading, self.sensors.yaw_rate(state.speed_m_per_s, state.curvature_per_m)

    def _stream_for(self, sensor: str) -> _Stream | None:
        return {"cone_detections": self.camera, "fiducial_detections": self.camera, "lidar_cartesian": self.lidar,
                "lidar_scan": self.lidar, "wheel_speed": self.wheel, "imu_orientation": self.imu,
                "imu_angular_velocity": self.imu}.get(sensor)

    def _fresh(self, sensor: str) -> bool:
        stream = self._stream_for(sensor)
        timeout = float(self.settings.get(f"sensor_timeout_s.{sensor}", 0.5))
        return stream is not None and stream.age(self.time_s) <= timeout

    def _age(self, stream: _Stream) -> SensorAge:
        acquired = stream.latest[0]
        return SensorAge(self.time_s - acquired, round(acquired * 1e9))

    # ---- The policy node --------------------------------------------------------------------------------------
    def health_problem(self) -> str | None:
        """
        :return: Why the node may not run the policy now, as scripts/policy_node.py says it, or None.
        """
        for sensor in self.required:
            if not self._fresh(sensor):
                return f"Required sensor {sensor} is missing or stale"
        if "cone_detections" in self.required and (
                self.last_nonempty_cones_at is None
                or self.time_s - self.last_nonempty_cones_at >= self.cone_empty_timeout_s):
            return "No recent nonempty cone detections"
        return None

    def _set_state(self, state: int, reason: str):
        if state != self.node_state or reason != self.node_reason:
            self.events.append(Event(self.time_s, f"node state {state}: {reason}"))
            _LOGGER.info("%.2f s: node state %d: %s", self.time_s, state, reason)
        self.node_state, self.node_reason = state, reason

    def _operator(self):
        if self.ever_started or self.time_s + 1e-9 < self.next_request_at:
            return
        self.next_request_at = self.time_s + OPERATOR_RETRY_S
        problem = self.health_problem()
        if problem:
            self._set_state(ZERO_STATE, f"Policy start refused: {problem}")
            return
        self.policy_started_at, self.previous_step_at = self.time_s, None
        self.heading_reference = self.imu.latest[1][0] if self._fresh("imu_orientation") else None
        self.ever_started = True
        self._set_state(POLICY_STATE, "Operator request")

    def _observations(self, first: bool) -> CarObservations:
        now = self.time_s
        cones = fiducials = lidar = wheel = gyro = orientation = None
        if self._fresh("cone_detections"):
            cones = ConeBatch(list(self.camera.latest[1][0]), self._age(self.camera), CAMERA_TIMING.latency_s)
        if self._fresh("fiducial_detections"):
            fiducials = FiducialBatch(list(self.camera.latest[1][1]), self._age(self.camera), self.dictionary_name,
                                      "oakd_rgb_camera_optical_frame")
        if self._fresh("lidar_cartesian"):
            lidar = LidarCartesian(list(self.lidar.latest[1]), self._age(self.lidar), "base_link")
        if self._fresh("wheel_speed"):
            wheel = WheelSpeed(self.wheel.latest[1], self._age(self.wheel))
        if self._fresh("imu_angular_velocity"):
            gyro = AngularVelocity(0.0, 0.0, self.imu.latest[1][1], self._age(self.imu))
        if self._fresh("imu_orientation") and self.heading_reference is not None:
            heading = self.imu.latest[1][0] - self.heading_reference
            orientation = CarOrientation(0.0, 0.0, math.atan2(math.sin(heading), math.cos(heading)),
                                         self._age(self.imu))
        dt = 0.0 if first else now - self.previous_step_at
        return CarObservations(cones, fiducials, None, lidar, ObservedCarState(orientation, gyro, None, wheel),
                               PolicyState(dt, now - self.policy_started_at, first))

    def _triggered(self) -> bool:
        if self.mode == "timer":
            if self.time_s + 1e-9 >= self.next_timer_at:
                self.next_timer_at += self.timer_period_s
                return True
            return False
        return {"cone_detection": self.camera, "fiducial_detection": self.camera, "lidar": self.lidar}[self.mode].new

    def _policy_step(self):
        problem = self.health_problem()
        if problem:
            self._set_state(ZERO_STATE, problem)
            return
        first = self.previous_step_at is None
        observations = self._observations(first)
        try:
            actions = self.policy.step(observations, self.memory)
            self.memory.step()
            outputs = [actions.drive_action, actions.steering_action]
            if not all(math.isfinite(v) for v in outputs):
                raise ValueError("Policy outputs must be finite numbers")
        except Exception:
            _LOGGER.error("Student policy failed:\n%s", traceback.format_exc())
            self._set_state(ZERO_STATE, "Policy calculation or output was invalid")
            return
        self.previous_step_at = self.time_s
        self.observations, self.actions = observations, actions
        self.policy_steps += 1
        self.car.set_command(actions.drive_action, actions.steering_action)
        command = self.policy.marker_command
        if command is not None and command != self.last_marker_command:
            self.events.append(Event(self.time_s, f"marker command {command.name}"))
        self.last_marker_command = command
        intersection = self.memory.data.intersection
        phase = None if intersection is None else (intersection.phase, intersection.stop_reason)
        if phase != self.last_intersection_phase and phase is not None:
            reason = f" ({intersection.stop_reason})" if intersection.stop_reason else ""
            self.events.append(Event(self.time_s, f"intersection {intersection.phase}{reason}"))
        self.last_intersection_phase = phase

    # ---- One step ---------------------------------------------------------------------------------------------
    def step(self) -> dict[str, bool]:
        """Moves the run on by PLANT_STEP_S.

        :return: Which things happened in this step: policy (a policy step ran), camera and lidar (a frame arrived).
        """
        self.camera.tick(self.time_s, self._capture_camera)
        self.lidar.tick(self.time_s, self._capture_lidar)
        self.wheel.tick(self.time_s, self._capture_wheel)
        self.imu.tick(self.time_s, self._capture_imu)
        if self.camera.new and self.camera.latest[1][0]:
            self.last_nonempty_cones_at = self.camera.latest[0]
        self._operator()
        stepped = False
        if self.node_state == POLICY_STATE and self._triggered():
            self._policy_step()
            stepped = self.node_state == POLICY_STATE
        if self.time_s + 1e-9 >= self.next_supervision_at:
            self.next_supervision_at += self.supervision_period_s
            if self.node_state == POLICY_STATE:
                problem = self.health_problem()
                if problem:
                    self._set_state(ZERO_STATE, problem)
            if self.node_state == ZERO_STATE:
                self.car.set_command(0.0, 0.0)
        if self.manual_command is not None:
            # A person driving by hand, as with the RC transmitter: the policy still runs and draws
            self.car.set_command(*self.manual_command)
        self.car.step(PLANT_STEP_S)
        self.time_s += PLANT_STEP_S
        self._check_contacts_and_end()
        return {"policy": stepped, "camera": self.camera.new, "lidar": self.lidar.new}

    def _check_contacts_and_end(self):
        state, car = self.car.state, self.car
        touched = cone_contacts(self.cone_xy, state.pose, car.body_front_m, car.body_rear_m, car.body_width_m)
        for index in sorted(touched - self.cones_touched):
            cone = self.track.cones[index]
            self.events.append(Event(self.time_s, f"touched a {cone.colour.name.lower()} cone at "
                                                  f"({cone.x:.2f}, {cone.y:.2f})"))
        self.cones_touched |= touched
        body = body_polygon(*state.pose, car.body_front_m, car.body_rear_m, car.body_width_m)
        for index, box in enumerate(self.boxes):
            if index not in self.obstacles_touched and polygon_gap(body, box) <= 0.0:
                self.obstacles_touched.add(index)
                self.events.append(Event(self.time_s, f"hit obstacle {index}"))
        if self.track.roads:
            off_road = min(_distance_outside(road, state.x, state.y) for road in self.track.roads)
            self.max_off_road_m = max(self.max_off_road_m, off_road)
            if off_road > OFF_ROAD_END_M:
                self.ended_because = "left the road"
        if self.time_s >= self.track.duration_s - 1e-9:
            self.ended_because = self.ended_because or "time limit"
        moving = state.speed_m_per_s > 0.02
        if moving or not self.ever_started:
            self.stopped_since = None
        elif self.stopped_since is None:
            self.stopped_since = self.time_s
        if (self.ever_started and self.stopped_since is not None and state.distance_m > 0.05
                and self.time_s - self.stopped_since >= STOPPED_FOR_S):
            stop = self.memory.data.mpc_stop
            self.ended_because = self.ended_because or (
                f"policy stopped the car ({stop.name})" if stop is not None
                else f"node state {self.node_state}: {self.node_reason}" if self.node_state != POLICY_STATE
                else "car stopped")

    @property
    def finished(self) -> bool:
        return self.ended_because is not None

    def summary(self) -> RunSummary:
        """
        :return: How the run has gone so far.
        """
        return RunSummary(self.track.name, self.ended_because or "still running", self.time_s,
                          self.car.state.distance_m, len(self.cones_touched), len(self.obstacles_touched),
                          self.max_off_road_m, self.node_state, self.node_reason, list(self.events))
