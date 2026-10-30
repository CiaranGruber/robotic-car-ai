"""ArUco marker commands: a closed-loop simulation of the whole MovementPolicy reading markers, changing lane side and
turning at intersections.

Every policy step the policy gets the simulated camera's cones and markers, the lidar's points, the wheel speed and the
IMU heading, and the car moves by what it commands. It runs MovementPolicy as policy_node does, with the settings in
config/ai4r_policy.yaml: the cone filter, the local map, lane detection, the markers, the intersection manoeuvre, the
avoidance planner, the MPC and the obstacle stop.

The camera, its delay and noise, and the car's speed and steering response are those of curved_lanes.py and
obstacle_simulation.py, with car 21's provisional steering limits: 1.0 1/m left (about the 1 m radius Yuhao reported)
and car 20's measured 0.92 1/m right, and the camera pan held at zero. Added here:
- roads of straight cone rows, blue on the left, joined at four-way and T intersections whose cross roads are as wide
  as the approach road;
- ArUco markers standing beside the road, seen like the cones (in view, 0.3-2.5 m, MARKER_NOISE_M of noise, the
  camera's delay) and missed MARKER_MISS_PROBABILITY of the time;
- the IMU's relative heading with HEADING_NOISE_RAD of noise, as the car's bno08x gives it;
- the lidar of obstacle_simulation.py, hitting the cones, so the obstacle stop runs through the turns.
It is a kinematic model without tyre slip; the intersection layouts and the car 21 limits are assumptions, not
measurements of the lab, so a pass here does not establish that the car can make the turn.

Run every case, print the table and save the figure from the repository root with:
    python3 tests/policy/marker_simulation.py
Some cases, other settings, or the runs saved as scenario files for scripts/mcap_export.py:
    python3 tests/policy/marker_simulation.py --cases cross_right_1m --seeds 5
    python3 tests/policy/marker_simulation.py --set intersection.right_turn_radius_m=1.3
    python3 tests/policy/marker_simulation.py --export logs/marker_simulation
    python3 -m scripts.mcap_export -p logs/marker_simulation/cross_right_1m_s0_scenario.json
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import curved_lanes as cl  # noqa: E402
from obstacle_simulation import (  # noqa: E402
    LIDAR_AGE_S, CONE_RADIUS_M, _CommandRecorder, _setting, body_polygon, cone_gaps, lidar_points,
)
from policy.input_output import (  # noqa: E402
    CarActions, CarObservations, CarOrientation, ConeBatch, ConeColour, FiducialBatch, FiducialDetection,
    FiducialPose, ImagePoint, LidarCartesian, ObservedCarState, PolicyState, Position, Quaternion, SensorAge,
    WheelSpeed,
)
from policy.markers.commands import MarkerCommand  # noqa: E402
from policy.memory import CarMemory  # noqa: E402
from real_car_measurements import LAB_CAR  # noqa: E402
from scripts.mcap_export import DEFAULT_SETTINGS_PATH, build_policy, read_settings  # noqa: E402
from policy.markers.markers import MarkerConfig  # noqa: E402
from stage1_scenarios import POLICY_PERIOD_S  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = REPO_ROOT / "logs" / "marker_simulation"
"""Folder for the figure; logs/ is not committed or uploaded to the car."""

CAR21 = dataclasses.replace(LAB_CAR, name="car 21 (provisional)", camera_yaw_deg=0.0, straight_curvature_per_m=0.0,
                            max_left_curvature_per_m=1.0, max_right_curvature_per_m=0.92)
"""The lab car with car 21's provisional steering limits (1.0 1/m left from Yuhao's reported 1 m radius, the measured
0.92 1/m right), the camera pan held at zero and the steering trim compensated."""
CONE_SPACING_M = 0.25
"""Distance between neighbouring cones along a row, in metres (lab tracks: about 0.2-0.3 m)."""
MARKER_GAP_M = 0.2
"""Distance in metres from a cone row to the marker standing beside it."""
TURN_MARKER_BEFORE_M = 1.0
"""Distance in metres before the intersection's entry of its turn marker, as intersection.marker_to_entry_m says."""
MARKER_SIZE_M = 0.081
"""Printed marker size in metres, as config/aruco_detector.yaml sets it for the markers used on 2026-10-21."""
MARKER_MIN_RANGE_M = 0.3
"""Nearest a marker is detected, in metres ahead of the camera."""
MARKER_NOISE_M = 0.03
"""Standard deviation of each marker position coordinate, in metres. UNMEASURED: like the cones'."""
MARKER_MISS_PROBABILITY = 0.1
"""Chance that a marker in view is not detected in a batch. UNMEASURED."""
HEADING_NOISE_RAD = 0.005
"""Standard deviation of the IMU's relative heading, in radians. UNMEASURED."""
GOAL_ALONG_M = 1.2
"""A run has left an intersection by a road once the car is this far along it, in metres, inside its edges. The MPC
stops by itself where that road's cones end."""
SIDE_SCORED_M = 0.8
"""A lane change run's side is scored over this distance driven before the next side marker counted, in metres."""
SIDE_TOLERANCE_M = 0.04
"""Largest distance in metres from a side's line, as a median over SIDE_SCORED_M, that passes."""
LANES_END_M = 1.0
"""A lane change run has finished once the car is within this distance of the road's end, in metres."""
MAX_DURATION_S = 45.0
"""A run ends after this long even if the car has not stopped."""
STOPPED_SPEED_M_PER_S = 0.02
"""Speeds below this count as stopped."""
CORNERS = (ImagePoint(0.0, 0.0), ImagePoint(1.0, 0.0), ImagePoint(1.0, 1.0), ImagePoint(0.0, 1.0))
"""Image corners of a detected marker. The policy does not use them."""


@dataclass(frozen=True)
class Road:
    """A straight road with a cone row on each side, blue on the left of its direction and yellow on the right."""
    name: str
    x_m: float
    """Middle of the road where it starts: x, in metres."""
    y_m: float
    """Where it starts: y, in metres."""
    heading_rad: float
    """Direction of the road (of driving along it)."""
    length_m: float
    width_m: float

    def frame(self, x: float, y: float) -> tuple[float, float]:
        """
        :return: Distance along the road from its start and signed distance left of its centre line, in metres.
        """
        dx, dy = x - self.x_m, y - self.y_m
        cos_h, sin_h = math.cos(self.heading_rad), math.sin(self.heading_rad)
        return cos_h * dx + sin_h * dy, -sin_h * dx + cos_h * dy

    def point(self, along_m: float, left_m: float) -> tuple[float, float]:
        """
        :return: The world (x, y) of a point given along the road and left of its centre line.
        """
        cos_h, sin_h = math.cos(self.heading_rad), math.sin(self.heading_rad)
        return self.x_m + along_m * cos_h - left_m * sin_h, self.y_m + along_m * sin_h + left_m * cos_h

    def cones(self, first_m: float = 0.0) -> list[tuple[ConeColour, float, float, float]]:
        """
        :param first_m: Distance along the road of the first cone pair.
        :return: (colour, distance along, x, y) of each cone on both edges, every CONE_SPACING_M.
        """
        cones = []
        for along in np.arange(first_m, self.length_m + 1e-6, CONE_SPACING_M):
            for colour, side in ((ConeColour.BLUE, 1.0), (ConeColour.YELLOW, -1.0)):
                cones.append((colour, float(along), *self.point(float(along), side * self.width_m / 2.0)))
        return cones


@dataclass(frozen=True)
class Marker:
    """An ArUco marker standing beside a road."""
    marker_id: int
    x_m: float
    y_m: float


@dataclass(frozen=True)
class World:
    """The roads, their cones and the markers of a case. The car starts on the first road's centre line, before it."""
    roads: tuple[Road, ...]
    cones: tuple[tuple[ConeColour, float, float, float], ...]
    markers: tuple[Marker, ...]
    open_areas: tuple[np.ndarray, ...] = ()
    """Polygons (Nx2 world x, y) the car may drive in besides the roads: the intersection's middle."""


def lane_change_world(width_m: float, markers: tuple[tuple[str, float, str], ...], length_m: float = 7.5) -> World:
    """A straight road with side markers beside it.

    :param width_m: The road's width in metres.
    :param markers: (command name, distance along the road, "LEFT" or "RIGHT" side it stands on) of each marker.
    :param length_m: The road's length in metres.
    :return: The world.
    """
    road = Road("road", 0.0, 0.0, 0.0, length_m, width_m)
    ids = _marker_ids()
    placed = tuple(Marker(ids[command], *road.point(along, (1.0 if side == "LEFT" else -1.0)
                                                    * (width_m / 2.0 + MARKER_GAP_M)))
                   for command, along, side in markers)
    return World((road,), tuple(road.cones()), placed)


def intersection_world(width_m: float, command: str, exits: tuple[str, ...] = ("LEFT", "STRAIGHT", "RIGHT"),
                       approach_m: float = 3.0, exit_m: float = 2.5, marker_side: str = "RIGHT",
                       extra_markers: tuple[tuple[str, float, str], ...] = ()) -> World:
    """An approach road ending at an intersection with roads of the same width leaving to the left, straight on
    and to the right, and the command's marker beside the approach road before the intersection. Sides of the
    intersection without an exit are closed by a cone row.

    :param width_m: Width of every road in metres.
    :param command: The marker's command: "TURN_LEFT", "TURN_RIGHT" or "STRAIGHT".
    :param exits: Which exits there are.
    :param approach_m: Length of the approach road in metres; the intersection's entry is at x = 0.
    :param exit_m: Length of each exit road in metres.
    :param marker_side: Which side of the approach road the marker stands on, TURN_MARKER_BEFORE_M before the entry.
    :param extra_markers: (command name, distance along the approach road, side) of more markers, such as a side
        marker before the turn.
    :return: The world.
    """
    half = width_m / 2.0
    approach = Road("approach", -approach_m, 0.0, 0.0, approach_m, width_m)
    exit_roads = {"STRAIGHT": Road("straight", width_m, 0.0, 0.0, exit_m, width_m),
                  "LEFT": Road("left", half, half, math.pi / 2.0, exit_m, width_m),
                  "RIGHT": Road("right", half, -half, -math.pi / 2.0, exit_m, width_m)}
    roads = (approach,) + tuple(exit_roads[name] for name in exits)
    cones = approach.cones()
    for name in exits:
        # The exits' rows start one spacing on from the intersection's corners, which the approach rows end at
        cones += exit_roads[name].cones(first_m=0.0 if name == "STRAIGHT" else CONE_SPACING_M)
    # Close the sides without an exit
    closing = {"STRAIGHT": [(width_m, y) for y in np.arange(-half, half + 1e-6, CONE_SPACING_M)],
               "LEFT": [(x, half) for x in np.arange(CONE_SPACING_M, width_m + 1e-6, CONE_SPACING_M)],
               "RIGHT": [(x, -half) for x in np.arange(CONE_SPACING_M, width_m + 1e-6, CONE_SPACING_M)]}
    for name, points in closing.items():
        if name not in exits:
            cones += [(ConeColour.BLUE if y >= 0.0 else ConeColour.YELLOW, 0.0, float(x), float(y))
                      for x, y in points]
    ids = _marker_ids()
    sign = 1.0 if marker_side == "LEFT" else -1.0
    markers = [Marker(ids[command], -TURN_MARKER_BEFORE_M, sign * (half + MARKER_GAP_M))]
    for name, along, side in extra_markers:
        markers.append(Marker(ids[name], *approach.point(along, (1.0 if side == "LEFT" else -1.0)
                                                         * (half + MARKER_GAP_M))))
    middle = np.array([(0.0, -half), (width_m, -half), (width_m, half), (0.0, half)])
    return World(roads, tuple(cones), tuple(markers), (middle,))


def _marker_ids() -> dict[str, int]:
    """
    :return: The marker ID of each command, from the markers settings in config/ai4r_policy.yaml.
    """
    settings = read_settings(DEFAULT_SETTINGS_PATH)
    config = MarkerConfig(**{key.removeprefix("markers."): value for key, value in settings.items()
                             if key.startswith("markers.")})
    return {command.name: marker_id for marker_id, command in config.commands.items()}


@dataclass(frozen=True)
class MarkerCase:
    """A world and what the car should do in it."""
    name: str
    description: str
    world: World
    goal: str
    """For an intersection, the road the car should leave by ("left", "straight" or "right"); for a lane change,
    "lanes", scored by side_checks."""
    side_checks: tuple[str, ...] = ()
    """For a lane change: the lane sides its markers ask for, in order."""
    start_left_m: float = 0.0


def _cases() -> tuple[MarkerCase, ...]:
    # Markers 3 m apart: each counts once the one before is passed and it is within markers.max_distance_m (2 m),
    # and the line then moves over at mpc.offset_change_per_m
    lanes = (("KEEP_LEFT", 1.0, "LEFT"), ("KEEP_RIGHT", 4.0, "RIGHT"), ("KEEP_CENTRE", 7.0, "LEFT"))
    checks = ("LEFT", "RIGHT", "CENTRE")
    cases = [
        MarkerCase("lanes_0.8m", "0.8 m road: keep left, right, then centre markers",
                   lane_change_world(0.8, lanes, 9.0), "lanes", checks),
        MarkerCase("lanes_1m", "1 m road: keep left, right, then centre markers", lane_change_world(1.0, lanes, 9.0),
                   "lanes", checks),
    ] if all(command in _marker_ids() for command, _, _ in lanes) else []
    for width in (1.0, 0.8):
        label = f"{width:g}m"
        for command, goal in (("TURN_LEFT", "left"), ("TURN_RIGHT", "right"), ("STRAIGHT", "straight")):
            cases.append(MarkerCase(f"cross_{goal}_{label}", f"Four-way, {width:g} m roads: {command} marker",
                                    intersection_world(width, command), goal))
        cases.append(MarkerCase(f"t_left_{label}", f"T, {width:g} m roads, no straight exit: TURN_LEFT marker",
                                intersection_world(width, "TURN_LEFT", exits=("LEFT", "RIGHT")), "left"))
        cases.append(MarkerCase(f"t_right_{label}", f"T, {width:g} m roads, no straight exit: TURN_RIGHT marker",
                                intersection_world(width, "TURN_RIGHT", exits=("LEFT", "RIGHT")), "right"))
    if "KEEP_LEFT" in _marker_ids():
        cases.append(MarkerCase("hook_right_0.8m", "Four-way, 0.8 m roads: KEEP_LEFT 1.5 m before, then TURN_RIGHT",
                            intersection_world(0.8, "TURN_RIGHT", approach_m=3.5,
                                               extra_markers=(("KEEP_LEFT", 0.6, "LEFT"),)), "right"))
    return tuple(cases)


CASES = _cases()
"""The cases: lane changes in a 0.8 m and a 1 m road, every turn at four-way and T intersections of 1 m roads (the
width Yuhao reported for car 21's track) and 0.8 m roads (the lab tracks' measured 0.73-0.85 m), and a right turn
after keeping left (the hook turn's order) in 0.8 m roads."""


@dataclass
class MarkerResult:
    """One run of a case."""
    case: MarkerCase
    seed: int
    x: np.ndarray
    y: np.ndarray
    heading: np.ndarray
    outcome: str
    """For an intersection: "passed" (left by the goal road), "wrong exit", "touched cones", "left the road" or
    "stopped". For a lane change: "passed", "wrong side", "touched cones", "left the road" or "stopped"."""
    metrics: dict[str, float] = field(default_factory=dict)
    trace: list[dict] = field(default_factory=list)
    """Per policy step: time, pose, speed, curvature, lane side, manoeuvre phase and the markers given, for plots."""
    steps: list[tuple[CarObservations, CarActions]] = field(default_factory=list)
    """Each policy step's observations and actions, for saving the run as a scenario file."""

    @property
    def ok(self) -> bool:
        return self.outcome == "passed"


def marker_detections(markers: tuple[Marker, ...], pose: tuple[float, float, float], car, rng) -> FiducialBatch:
    """What the ArUco detector reports from a pose: the markers in the camera's view, in base_link, with noise.

    :param markers: The markers.
    :param pose: The car's world (x, y, heading).
    :param car: The camera's view and range.
    :param rng: Random generator for the noise and misses.
    :return: The batch, possibly empty.
    """
    x, y, heading = pose
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    half_fov = math.radians(car.detector_fov_deg) / 2.0
    found = []
    for marker in markers:
        dx, dy = marker.x_m - x, marker.y_m - y
        bx, by = cos_h * dx + sin_h * dy, -sin_h * dx + cos_h * dy
        if not (MARKER_MIN_RANGE_M <= bx <= car.detector_range_m) or abs(math.atan2(by, bx)) > half_fov:
            continue
        if rng.random() < MARKER_MISS_PROBABILITY:
            continue
        position = Position(bx + rng.normal(0.0, MARKER_NOISE_M), by + rng.normal(0.0, MARKER_NOISE_M), 0.15)
        found.append(FiducialDetection(marker.marker_id, MARKER_SIZE_M, CORNERS,
                                       FiducialPose(position, Quaternion(0.0, 0.0, 0.0, 1.0)), 0.5))
    return FiducialBatch(found, SensorAge(car.detection_latency_s, None), "DICT_4X4_50",
                         "oak_rgb_camera_optical_frame")


def _outside_m(world: World, corners: np.ndarray) -> float:
    """
    :return: How far in metres the body corner farthest outside every road and open area is beyond their edges; at
        most 0.0 while every corner is inside one of them.
    """
    worst = -math.inf
    for cx, cy in corners:
        best = math.inf
        for index, road in enumerate(world.roads):
            along, left = road.frame(cx, cy)
            # Behind the first road's start is where the car starts; past a road's end, the run is over
            behind = -math.inf if index == 0 else -along - road.width_m / 2.0
            best = min(best, max(behind, along - road.length_m - 1.0, abs(left) - road.width_m / 2.0))
        for polygon in world.open_areas:
            low, high = polygon.min(axis=0), polygon.max(axis=0)
            best = min(best, max(low[0] - cx, cx - high[0], low[1] - cy, cy - high[1]))
        worst = max(worst, best)
    return worst


def run_case(case: MarkerCase, settings: dict, seed: int = 0, car=CAR21) -> MarkerResult:
    """Drives MovementPolicy through a case.

    :param case: The world and the goal.
    :param settings: Policy settings by full parameter name (scripts.mcap_export.read_settings).
    :param seed: Seed of the camera, marker, IMU and lidar noise.
    :param car: The simulated car (a stage1_scenarios.CarProfile).
    :return: The trajectory, outcome and metrics.
    """
    policy = build_policy(settings)
    recorder = _CommandRecorder(policy.car_controller)
    policy.car_controller = recorder
    memory = CarMemory()
    world = case.world
    front_m = policy.obstacle_detection_config.car_front_m
    rear_m, width_m = policy.avoidance.config.car_rear_m, policy.avoidance.config.car_width_m
    rng = np.random.default_rng(seed)
    cone_xy = np.array([(cx, cy) for _, _, cx, cy in world.cones])
    start = world.roads[0]
    x, y = start.point(-cl.RUN_IN_M, case.start_left_m)
    heading = start.heading_rad
    speed, driven_curvature = 0.0, car.straight_curvature_per_m
    camera_delay = round(car.detection_latency_s / POLICY_PERIOD_S)
    lidar_delay = round(LIDAR_AGE_S / POLICY_PERIOD_S)
    dead_steps = round(car.speed_dead_time_s / cl.SIM_STEP_S)
    steps_per_policy = round(POLICY_PERIOD_S / cl.SIM_STEP_S)
    camera_queue, marker_queue, lidar_queue = [], [], []
    requested = [recorder.command] * (dead_steps + 1)
    command = recorder.command
    log = {"x": [], "y": [], "heading": []}
    steps, trace, touched = [], [], set()
    max_outside = -math.inf
    step = 0
    while step * cl.SIM_STEP_S < MAX_DURATION_S:
        for key, value in (("x", x), ("y", y), ("heading", heading)):
            log[key].append(value)
        if step % steps_per_policy == 0:
            pose = (x, y, heading)
            camera_queue.append(cl.detections(list(world.cones), pose, car, rng))
            marker_queue.append(marker_detections(world.markers, pose, car, rng))
            lidar_queue.append(lidar_points(pose, cone_xy, [], rng))
            batch = camera_queue.pop(0) if len(camera_queue) > camera_delay else None
            fiducials = marker_queue.pop(0) if len(marker_queue) > camera_delay else None
            scan = lidar_queue.pop(0) if len(lidar_queue) > lidar_delay else None
            first = step == 0
            orientation = CarOrientation(0.0, 0.0, math.atan2(math.sin(heading - start.heading_rad),
                                                              math.cos(heading - start.heading_rad))
                                         + rng.normal(0.0, HEADING_NOISE_RAD), SensorAge(0.02, None))
            observations = CarObservations(
                cones=None if batch is None else ConeBatch(batch, SensorAge(car.detection_latency_s, None),
                                                           acquisition_to_publish_latency_s=0.0),
                fiducials=fiducials, lidar_scan=None,
                lidar_cartesian=None if scan is None else LidarCartesian(scan, SensorAge(LIDAR_AGE_S, None),
                                                                         "base_link"),
                car=ObservedCarState(orientation, None, None, WheelSpeed(speed, SensorAge(0.0, None))),
                policy=PolicyState(dt=0.0 if first else POLICY_PERIOD_S, policy_elapsed_s=step * cl.SIM_STEP_S,
                                   is_first_policy_step=first))
            actions = policy.step(observations, memory)
            memory.step()
            command = recorder.command
            steps.append((observations, actions))
            turn = memory.data.intersection_turn
            trace.append({"t": round(step * cl.SIM_STEP_S, 3), "x": round(x, 4), "y": round(y, 4),
                          "heading": round(heading, 4), "speed": round(speed, 3),
                          "curvature": round(command.curvature_per_m, 4), "side": policy.action_policy.lane_side,
                          "phase": None if turn is None else turn.phase.name,
                          "manoeuvre": policy.manoeuvre is not None,
                          "marker": None if policy.marker_command is None else policy.marker_command.name,
                          "arc_start_m": policy.turner.arc_start_m})
        # The car: speed after a dead time and a lag, curvature clipped to the steering limits plus the trim and lagged
        # by the servo, as in curved_lanes.run_curve
        requested.append(command)
        applied = requested.pop(0)
        speed = max(0.0, speed + (applied.speed_m_per_s - speed) * cl.SIM_STEP_S / (car.speed_time_constant_s or 0.3))
        target = (min(max(command.curvature_per_m, -car.max_right_curvature_per_m), car.max_left_curvature_per_m)
                  + car.straight_curvature_per_m)
        driven_curvature += (target - driven_curvature) * cl.SIM_STEP_S / cl.STEERING_TIME_CONSTANT_S
        heading += speed * driven_curvature * cl.SIM_STEP_S / 2.0
        x += speed * math.cos(heading) * cl.SIM_STEP_S
        y += speed * math.sin(heading) * cl.SIM_STEP_S
        heading += speed * driven_curvature * cl.SIM_STEP_S / 2.0
        step += 1
        touched.update(np.flatnonzero(cone_gaps(cone_xy, x, y, heading, front_m, rear_m, width_m)
                                      < CONE_RADIUS_M).tolist())
        body = body_polygon(x, y, heading, front_m, rear_m, width_m)
        if start.frame(x, y)[0] >= 0.0:
            max_outside = max(max_outside, _outside_m(world, body))
        if (_finished(case, x, y) is not None and case.goal != "lanes") or (
                step > 50 and speed < STOPPED_SPEED_M_PER_S and command.speed_m_per_s == 0.0):
            break
    for key, value in (("x", x), ("y", y), ("heading", heading)):
        log[key].append(value)
    result = MarkerResult(case, seed, np.array(log["x"]), np.array(log["y"]), np.array(log["heading"]), "",
                          trace=trace, steps=steps)
    result.metrics = {"cones_touched": float(len(touched)), "max_outside_m": float(max_outside),
                      "time_s": step * cl.SIM_STEP_S}
    result.outcome = _score(result, policy.action_policy, settings)
    return result


def _finished(case: MarkerCase, x: float, y: float) -> str | None:
    """
    :return: The name of the road the run has finished on: for a lane change, the road once within LANES_END_M of its
        end; for an intersection, the exit road the car is GOAL_ALONG_M along. None while it has not finished.
    """
    if case.goal == "lanes":
        road = case.world.roads[0]
        along, left = road.frame(x, y)
        return road.name if abs(left) <= road.width_m / 2.0 and along >= road.length_m - LANES_END_M else None
    for road in case.world.roads[1:]:
        along, left = road.frame(x, y)
        if abs(left) <= road.width_m / 2.0 and along >= GOAL_ALONG_M:
            return road.name
    return None


def _score(result: MarkerResult, mpc, settings: dict) -> str:
    """Sets the result's goal metrics and returns its outcome."""
    case, metrics = result.case, result.metrics
    if metrics["cones_touched"] > 0:
        return "touched cones"
    if metrics["max_outside_m"] > 0.0:
        return "left the road"
    if case.goal == "lanes":
        # Each side is scored over the SIDE_SCORED_M driven before the next side marker counted (or the run ended),
        # against where mpc.edge_clearance_m puts its line
        road = case.world.roads[0]
        given = []
        for t in result.trace:
            # A marker lost for a step and seen again gives its command again: only a change of side counts
            if t["marker"] in ("KEEP_LEFT", "KEEP_RIGHT", "KEEP_CENTRE") and (not given or given[-1][1] != t["side"]):
                given.append((road.frame(t["x"], t["y"])[0], t["side"]))
        frames = np.array([road.frame(t["x"], t["y"]) for t in result.trace])
        ends = [along for along, _ in given[1:]] + [frames[-1, 0]]
        room = max(0.0, road.width_m / 2.0 - float(settings["mpc.edge_clearance_m"]))
        if len(given) < len(case.side_checks):
            return "wrong side"
        worst = 0.0
        for index, ((_, side), end) in enumerate(zip(given, ends)):
            inside = (frames[:, 0] > end - SIDE_SCORED_M) & (frames[:, 0] <= end)
            median = float(np.median(frames[inside, 1])) if inside.any() else math.nan
            target = {"LEFT": room, "RIGHT": -room, "CENTRE": 0.0}[side]
            metrics[f"side{index}_m"], metrics[f"side{index}_target_m"] = median, target
            worst = max(worst, abs(median - target))
        metrics["side_error_m"] = worst
        if _finished(case, result.x[-1], result.y[-1]) is None:
            return "stopped"
        return "passed" if worst <= SIDE_TOLERANCE_M else "wrong side"
    ended = _finished(case, result.x[-1], result.y[-1])
    if ended is None:
        return "stopped"
    exit_road = next(road for road in case.world.roads if road.name == ended)
    lefts = [exit_road.frame(px, py)[1] for px, py in zip(result.x, result.y)
             if 1.0 <= exit_road.frame(px, py)[0] <= exit_road.length_m]
    metrics["exit_lateral_max_m"] = float(np.max(np.abs(lefts))) if lefts else math.nan
    return "passed" if ended == case.goal else "wrong exit"


def summarise(runs: list[MarkerResult]) -> str:
    """
    :return: One table cell for a case's runs: passes out of runs, and the other outcomes.
    """
    passed = sum(run.ok for run in runs)
    others = sorted({run.outcome for run in runs if not run.ok})
    return f"{passed}/{len(runs)}" + (f" ({', '.join(others)})" if others else "")


def plot(results: dict[str, list[MarkerResult]], path: Path, title: str):
    """Saves a figure with each case's world and trajectories."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    names = list(results)
    columns = 4
    rows = math.ceil(len(names) / columns)
    figure, axes = plt.subplots(rows, columns, figsize=(4.2 * columns, 4.2 * rows), squeeze=False)
    for axis, name in zip(axes.flat, names):
        runs = results[name]
        world = runs[0].case.world
        colours = {ConeColour.BLUE: "#1f5fd1", ConeColour.YELLOW: "#e0b000"}
        for colour, _, cx, cy in world.cones:
            axis.plot(cx, cy, "o", ms=2.5, color=colours.get(colour, "grey"))
        for marker in world.markers:
            axis.plot(marker.x_m, marker.y_m, "s", ms=6, color="black")
            axis.annotate(str(marker.marker_id), (marker.x_m, marker.y_m), fontsize=7, xytext=(3, 3),
                          textcoords="offset points")
        for run in runs:
            axis.plot(run.x, run.y, "-", lw=1.2, color="#2a9d4b" if run.ok else "#d62728", alpha=0.8)
        axis.set_title(f"{name}: {summarise(runs)}", fontsize=9)
        axis.set_aspect("equal")
        axis.grid(alpha=0.3)
    for axis in list(axes.flat)[len(names):]:
        axis.axis("off")
    figure.suptitle(title)
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=110)
    plt.close(figure)


def scenario_document(result: MarkerResult) -> dict:
    """
    :return: The run as a JSON-ready scenario, in the format of scenarios/data (scenario_wrapper.Scenario).
    """
    case = result.case
    description = (f"Marker simulation (tests/policy/marker_simulation.py), case {case.name}: {case.description}. "
                   f"Car 21 provisional limits, seed {result.seed}. Outcome: {result.outcome}.")
    return {"name": f"Marker simulation {case.name} (seed {result.seed})", "description": description,
            "test_data": [{"observations": observations.serialise(), "actions": actions.serialise()}
                          for observations, actions in result.steps]}


def viewer_document(results: dict[str, list[MarkerResult]]) -> dict:
    """
    :return: Every case's world and runs as JSON-ready data for a web viewer: cones, markers, roads and each run's
        policy-step trace.
    """
    cases = []
    for name, runs in results.items():
        world = runs[0].case.world
        cases.append({
            "name": name, "description": runs[0].case.description, "goal": runs[0].case.goal,
            "cones": [[colour.name[0], round(cx, 3), round(cy, 3)] for colour, _, cx, cy in world.cones],
            "markers": [[m.marker_id, round(m.x_m, 3), round(m.y_m, 3)] for m in world.markers],
            "roads": [{"name": r.name, "x": r.x_m, "y": r.y_m, "heading": r.heading_rad, "length": r.length_m,
                       "width": r.width_m} for r in world.roads],
            "runs": [{"seed": run.seed, "outcome": run.outcome, "metrics": run.metrics, "trace": run.trace}
                     for run in runs]})
    return {"cases": cases}


def main(argv: list[str] | None = None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--cases", nargs="*", choices=[c.name for c in CASES], help="cases to run (default: all)")
    parser.add_argument("--seeds", type=int, default=3, help="noise seeds per case (default 3)")
    parser.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", type=_setting,
                        help="override a policy setting, e.g. intersection.right_turn_radius_m=1.3")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT / "marker_simulation.png", help="figure to save")
    parser.add_argument("--export", type=Path, metavar="FOLDER",
                        help="also save each run as <case>_s<seed>_scenario.json for scripts/mcap_export.py")
    parser.add_argument("--viewer", type=Path, metavar="FILE", help="also save every run for the web viewer")
    args = parser.parse_args(argv)
    settings = {**read_settings(DEFAULT_SETTINGS_PATH), **dict(args.set)}
    results = {case.name: [run_case(case, settings, seed) for seed in range(args.seeds)]
               for case in CASES if not args.cases or case.name in args.cases}
    print(f"MovementPolicy with config/ai4r_policy.yaml, {CAR21.name}; {args.seeds} noise seeds; "
          f"overrides {dict(args.set) or 'none'}\n")
    width = max(len(name) for name in results)
    for name, runs in results.items():
        extra = ""
        if runs[0].case.goal == "lanes":
            extra = "  side error " + " ".join(f"{run.metrics.get('side_error_m', math.nan):.3f}" for run in runs)
        print(f"{name:<{width}}  {summarise(runs)}{extra}")
    plot(results, args.output, f"Marker simulation: MovementPolicy with config/ai4r_policy.yaml, {CAR21.name}")
    print(f"\nFigure: {args.output}")
    if args.export:
        args.export.mkdir(parents=True, exist_ok=True)
        for runs in results.values():
            for run in runs:
                path = args.export / f"{run.case.name}_s{run.seed}_scenario.json"
                path.write_text(json.dumps(scenario_document(run)) + "\n")
        print(f"Scenarios: {args.export}")
    if args.viewer:
        args.viewer.parent.mkdir(parents=True, exist_ok=True)
        args.viewer.write_text(json.dumps(viewer_document(results)))
        print(f"Viewer data: {args.viewer}")


if __name__ == "__main__":
    main()
