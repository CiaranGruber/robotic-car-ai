"""Obstacles in cone lanes: a closed-loop simulation of the whole MovementPolicy driving the lab car around boxes.

The recorded scenarios replay the car's own drive, so they cannot show whether the policy would have got past an
obstacle, or where it would have stopped. Here the policy drives: every policy step it gets the simulated camera's
cones and the simulated lidar's points, and the car moves by what it commands. It runs MovementPolicy as policy_node
does, with the settings in config/ai4r_policy.yaml: the cone filter, lane detection, obstacle detection, the
avoidance planner and the MPC, and the obstacle stop.

The lanes, the camera and the car's speed and steering response are those of curved_lanes.py (the measured lab car,
real_car_measurements.LAB_CAR, with the camera pan held at zero). Added here:
- boxes standing in the lane, placed in lane coordinates;
- a 2D lidar, as lidar_cartesian in base_link: beams every LIDAR_STEP_DEG from LIDAR_FORWARD_M ahead of base_link,
  hitting the boxes and the cones, LIDAR_AGE_S old;
- boxes hide the cones behind them from the camera;
- active braking for an obstacle stop (DriveCommand.brake). Nobody has measured it, so its travel is a setting
  (--brake-s), like mpc.obstacle_stop_distance_per_speed_s in the policy.
The car's true size is the size the policy is configured with (avoidance.car_width_m, car_rear_m and
obstacle_detection.car_front_m), which have not been measured on the car either.

Each case is run with each variant of the settings, by default the current config and
mpc.obstacle_stop_distance_per_speed_s 0.6, and checked against what it should do: get past, stop short of the
obstacle, or either; hitting a box or a cone fails it. This is a kinematic model without tyre slip or real lidar
artefacts, so it cannot establish the real car's stopping distance or clearances.

Run all cases, print the table and save the figure from the repository root with:
    python3 tests/policy/obstacle_simulation.py
Some cases, other settings, or the run saved as scenario files for scripts/mcap_export.py:
    python3 tests/policy/obstacle_simulation.py --cases straight_right straight_centre
    python3 tests/policy/obstacle_simulation.py --stop-per-speed 0.0 0.3 0.6 --brake-s 0.8 --seeds 5
    python3 tests/policy/obstacle_simulation.py --set avoidance.obstacle_clearance_m=0.05
    python3 tests/policy/obstacle_simulation.py --variant narrow obstacle_detection.corridor_half_width_m=0.175 \
        mpc.obstacle_confirm_steps=3
    python3 tests/policy/obstacle_simulation.py --cases straight_slalom --export logs/obstacle_simulation
    python3 -m scripts.mcap_export -p logs/obstacle_simulation/straight_slalom_current_s0_scenario.json
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
from policy.control.actions import DriveCommand  # noqa: E402
from policy.input_output import (  # noqa: E402
    CarActions, CarObservations, ConeBatch, LidarCartesian, LidarCartesianPoint, ObservedCarState, PolicyState,
    Position, SensorAge, WheelSpeed,
)
from policy.memory import CarMemory  # noqa: E402
from real_car_measurements import LAB_CAR  # noqa: E402
from scripts.mcap_export import DEFAULT_SETTINGS_PATH, build_policy, read_settings  # noqa: E402
from stage1_scenarios import POLICY_PERIOD_S  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = REPO_ROOT / "logs" / "obstacle_simulation"
"""Folder for the figure; logs/ is not committed or uploaded to the car."""

LIDAR_FORWARD_M = 0.20
"""Lidar scan origin, metres ahead of base_link (the forward_from_cg_m example in config/lidar_mount.yaml)."""
LIDAR_HEIGHT_M = 0.12
"""Height of the scan plane in metres, as in the recordings."""
LIDAR_STEP_DEG = 0.5
"""Angle between lidar beams in degrees, as in the recordings (about 400 returns per scan)."""
LIDAR_RANGE_M = 8.0
"""Farthest lidar return in metres."""
LIDAR_NOISE_M = 0.01
"""Standard deviation of each lidar range in metres."""
LIDAR_AGE_S = 0.2
"""Age of every lidar scan when the policy gets it, in seconds (0.15-0.3 s in the recordings). A multiple of
POLICY_PERIOD_S."""
CONE_SCAN_RADIUS_M = 0.035
"""Radius in metres of a cone where the scan plane cuts it, LIDAR_HEIGHT_M up its side."""
CAMERA_FORWARD_M = -0.10
"""Camera position, metres ahead of base_link (the forward_from_cg_m example in config/camera_mount.yaml), from which
the boxes hide the cones behind them."""
CONE_RADIUS_M = 0.05
"""Radius of a cone's base in metres, for the cone-touched check."""
DEFAULT_BRAKE_S = 0.6
"""Distance in metres the car travels per m/s of speed once it is asked to brake, including the speed dead time.
UNMEASURED: the same starting guess as mpc.obstacle_stop_distance_per_speed_s's comment."""
MAX_DURATION_S = 40.0
"""A run ends after this long even if the car has not stopped."""
STOPPED_SPEED_M_PER_S = 0.02
"""Speeds below this count as stopped."""


@dataclass(frozen=True)
class Box:
    """A box standing in the lane, placed in the lane's coordinates."""
    s_m: float
    """Distance in metres along the centre line from the first cone pair to the box's near face."""
    right_m: float
    """Its right side, in metres left of the centre line (negative right of it). The cone rows are at +-width/2."""
    left_m: float
    """Its left side, in metres left of the centre line."""
    length_m: float = 0.2
    """Its length along the lane in metres."""

    def polygon(self, line: dict[str, np.ndarray]) -> np.ndarray:
        """
        :param line: The lane's centre line (CurvedLane.centre_line).
        :return: Its outline's world (x, y) corners, as a 4x2 array going around it.
        """
        corners = []
        for s, d in ((self.s_m, self.right_m), (self.s_m + self.length_m, self.right_m),
                     (self.s_m + self.length_m, self.left_m), (self.s_m, self.left_m)):
            x, y = np.interp(s, line["s"], line["x"]), np.interp(s, line["s"], line["y"])
            heading = np.interp(s, line["s"], line["heading"])
            corners.append((x - d * math.sin(heading), y + d * math.cos(heading)))
        return np.array(corners)


@dataclass(frozen=True)
class ObstacleCase:
    """A lane with boxes in it, and what the car should do."""
    name: str
    """Short name, used on the command line and in file names."""
    description: str
    lane: cl.CurvedLane
    boxes: tuple[Box, ...] = ()
    expect: str = "pass"
    """"pass" to reach the lane end, "stop" to stop short of a box, or "either"."""
    start_left_m: float = 0.0
    seed: int = 0


LAB_WIDTH_M = 0.8
"""Lane width of the lab tracks, between the cone rows."""
STRAIGHT = cl.CurvedLane(pieces=(("straight", 5.0),), width_m=LAB_WIDTH_M)
CASES = (
    ObstacleCase("straight_clear", "Straight 0.8 m lane, no obstacle", STRAIGHT),
    ObstacleCase("straight_right", "Box on the right, 0.5 m gap on the left",
                 STRAIGHT, (Box(2.0, -0.35, -0.15),)),
    ObstacleCase("straight_left", "Box on the left, 0.5 m gap on the right",
                 STRAIGHT, (Box(2.0, 0.15, 0.35),)),
    ObstacleCase("straight_tight", "Box on the right, exactly the 0.45 m gap the planner needs",
                 STRAIGHT, (Box(2.0, -0.35, -0.10),), expect="either"),
    ObstacleCase("straight_centre", "Box in the middle, 0.25 m either side: must stop",
                 STRAIGHT, (Box(2.0, -0.10, 0.10),), expect="stop"),
    ObstacleCase("straight_edge", "Box on the right cone row, partly outside the lane",
                 STRAIGHT, (Box(2.05, -0.48, -0.25, 0.15),)),
    ObstacleCase("straight_slalom", "Box on the right, then one on the left 2.5 m on",
                 cl.CurvedLane(pieces=(("straight", 6.5),), width_m=LAB_WIDTH_M),
                 (Box(1.8, -0.35, -0.15), Box(4.3, 0.15, 0.35))),
    ObstacleCase("two_boxes_same_side", "Two boxes on the right, 1 m gap between them",
                 cl.CurvedLane(pieces=(("straight", 6.0),), width_m=LAB_WIDTH_M),
                 (Box(2.0, -0.35, -0.15), Box(3.2, -0.35, -0.15))),
    ObstacleCase("two_boxes_opposite", "Box on the right, then one on the left after a 1 m gap: too close to weave",
                 cl.CurvedLane(pieces=(("straight", 6.0),), width_m=LAB_WIDTH_M),
                 (Box(2.0, -0.35, -0.15), Box(3.2, 0.15, 0.35)), expect="either"),
    ObstacleCase("wide_centre", "1.3 m lane, box in the middle: 0.5 m either side",
                 cl.CurvedLane(pieces=(("straight", 5.0),), width_m=1.3), (Box(2.0, -0.10, 0.10),)),
    ObstacleCase("uturn_clear", "Left U-turn of 2.5 m radius, no obstacle",
                 cl.CurvedLane(pieces=(("straight", 1.0), ("arc", 1 / 2.5, 180.0), ("straight", 1.5)),
                               width_m=LAB_WIDTH_M)),
)
"""The cases. Obstacles only stand on straights; the U-turn checks that the obstacle detection lets the car follow a
curve. The boxes are 0.2 m long, about the lab obstacles' 0.21-0.22 m faces in the recordings. In a 0.8 m lane the
planner needs a gap of car_width_m + obstacle_clearance_m + cone_clearance_m = 0.45 m between a box and the inner side
of the cones (0.35 m from the centre)."""
EXPECTED_OUTCOMES = {"pass": {"passed"}, "stop": {"stopped"}, "either": {"passed", "stopped"}}


@dataclass
class ObstacleResult:
    """One run of a case with one variant of the settings."""
    case: ObstacleCase
    variant: str
    x: np.ndarray
    y: np.ndarray
    heading: np.ndarray
    outcome: str
    """"passed", "stopped" (for an obstacle), "hit box", "touched cones", "lane lost", "solver failed" or
    "stopped short"."""
    metrics: dict[str, float] = field(default_factory=dict)
    seed: int = 0
    """Seed of the camera and lidar noise."""
    final_body: np.ndarray | None = None
    """World corners of the car body where the run ended."""
    stop_request: tuple[float, float, float] | None = None
    """The car's world (x, y, heading) when the policy first stopped for an obstacle, or None."""
    steps: list[tuple[CarObservations, CarActions]] = field(default_factory=list)
    """Each policy step's observations and actions, for saving the run as a scenario file."""

    @property
    def ok(self) -> bool:
        """Whether it did what the case expects."""
        return self.outcome in EXPECTED_OUTCOMES[self.case.expect]


class _CommandRecorder:
    """Stands in for MovementPolicy.car_controller to keep the DriveCommand the policy chose at each step."""

    def __init__(self, controller):
        self._controller = controller
        self.command = DriveCommand(0.0, 0.0)

    def step(self, observations, instructions):
        self.command = next(i for i in instructions if isinstance(i, DriveCommand))
        return self._controller.step(observations, instructions)

    def __getattr__(self, name):
        return getattr(self._controller, name)


def body_polygon(x: float, y: float, heading: float, front_m: float, rear_m: float, width_m: float) -> np.ndarray:
    """
    :return: The world (x, y) of the car body's corners (front left, front right, rear right, rear left).
    """
    local = np.array([[front_m, width_m / 2], [front_m, -width_m / 2], [-rear_m, -width_m / 2],
                      [-rear_m, width_m / 2]])
    rotation = np.array([[math.cos(heading), -math.sin(heading)], [math.sin(heading), math.cos(heading)]])
    return local @ rotation.T + np.array([x, y])


def cone_gaps(cones: np.ndarray, x: float, y: float, heading: float, front_m: float, rear_m: float,
              width_m: float) -> np.ndarray:
    """
    :return: Distance in metres from each cone centre (an Nx2 array of world x, y) to the car body, 0.0 inside it.
    """
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    dx, dy = cones[:, 0] - x, cones[:, 1] - y
    along, across = cos_h * dx + sin_h * dy, -sin_h * dx + cos_h * dy
    beyond_along = np.maximum(np.maximum(along - front_m, -rear_m - along), 0.0)
    beyond_across = np.maximum(np.abs(across) - width_m / 2.0, 0.0)
    return np.hypot(beyond_along, beyond_across)


def _segment_distance(points: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """
    :return: Distance from each of the points to the segment from a to b.
    """
    ab = b - a
    t = np.clip(((points - a) @ ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    return np.hypot(*(points - (a + t[:, None] * ab)).T)


def polygon_gap(first: np.ndarray, second: np.ndarray) -> float:
    """
    :return: Smallest distance in metres between two convex polygons, or 0.0 when they touch or overlap.
    """
    for polygon in (first, second):
        edges = np.roll(polygon, -1, axis=0) - polygon
        for normal in np.stack([-edges[:, 1], edges[:, 0]], axis=1):
            a, b = first @ normal, second @ normal
            if a.max() < b.min() or b.max() < a.min():
                return float(min(_segment_distance(points, p, q).min()
                                 for points, other in ((first, second), (second, first))
                                 for p, q in zip(other, np.roll(other, -1, axis=0))))
    return 0.0


def lidar_points(pose: tuple[float, float, float], cones: np.ndarray, boxes: list[np.ndarray],
                 rng: np.random.Generator) -> list[LidarCartesianPoint]:
    """The returns of one lidar scan from a pose, in base_link at that pose.

    :param pose: The car's world (x, y, heading).
    :param cones: World (x, y) of every cone, as an Nx2 array.
    :param boxes: The boxes' world outlines.
    :param rng: Random generator for the range noise.
    :return: One point per beam that hits a cone or a box within LIDAR_RANGE_M.
    """
    x, y, heading = pose
    origin = np.array([x + LIDAR_FORWARD_M * math.cos(heading), y + LIDAR_FORWARD_M * math.sin(heading)])
    bearings = np.radians(np.arange(-180.0, 180.0, LIDAR_STEP_DEG))
    directions = np.stack([np.cos(heading + bearings), np.sin(heading + bearings)], axis=1)
    ranges = np.full(len(bearings), np.inf)
    # Cones: the nearer intersection of each beam with each circle
    offsets = cones - origin
    near = np.hypot(*offsets.T) < LIDAR_RANGE_M + CONE_SCAN_RADIUS_M
    if near.any():
        offsets = offsets[near]
        along = directions @ offsets.T
        disc = along ** 2 - (np.sum(offsets ** 2, axis=1) - CONE_SCAN_RADIUS_M ** 2)
        hit = along - np.sqrt(np.where(disc >= 0.0, disc, np.nan))
        hit = np.where((disc >= 0.0) & (hit > 0.0), hit, np.inf)
        ranges = np.minimum(ranges, hit.min(axis=1))
    # Boxes: each beam against each edge
    for polygon in boxes:
        for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
            edge, start = b - a, a - origin
            denominator = directions[:, 0] * edge[1] - directions[:, 1] * edge[0]
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (start[0] * edge[1] - start[1] * edge[0]) / denominator
                u = (start[0] * directions[:, 1] - start[1] * directions[:, 0]) / denominator
            ranges = np.minimum(ranges, np.where((t > 0.0) & (u >= 0.0) & (u <= 1.0), t, np.inf))
    hits = np.flatnonzero(ranges < LIDAR_RANGE_M)
    measured = ranges[hits] + rng.normal(0.0, LIDAR_NOISE_M, len(hits))
    return [LidarCartesianPoint(Position(LIDAR_FORWARD_M + float(r) * math.cos(bearings[i]),
                                         float(r) * math.sin(bearings[i]), LIDAR_HEIGHT_M), int(i))
            for i, r in zip(hits, measured)]


def _crosses(p: np.ndarray, q: np.ndarray, polygon: np.ndarray) -> bool:
    """
    :return: Whether the segment from p to q crosses the polygon's outline.
    """
    def side(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        if side(p, q, a) * side(p, q, b) < 0.0 and side(a, b, p) * side(a, b, q) < 0.0:
            return True
    return False


def visible_cones(cones: list, pose: tuple[float, float, float], boxes: list[np.ndarray]) -> list:
    """
    :param cones: (colour, s, x, y) of every cone.
    :param pose: The car's world (x, y, heading).
    :param boxes: The boxes' world outlines.
    :return: The cones no box hides from the camera.
    """
    if not boxes:
        return cones
    x, y, heading = pose
    camera = np.array([x + CAMERA_FORWARD_M * math.cos(heading), y + CAMERA_FORWARD_M * math.sin(heading)])
    return [cone for cone in cones
            if not any(_crosses(camera, np.array(cone[2:]), polygon) for polygon in boxes)]


def run_case(case: ObstacleCase, settings: dict, variant: str = "current", brake_s: float = DEFAULT_BRAKE_S,
             car=None, seed: int | None = None) -> ObstacleResult:
    """Drives MovementPolicy through a case.

    :param case: The lane, boxes and start.
    :param settings: Policy settings by full parameter name (scripts.mcap_export.read_settings).
    :param variant: Name of these settings, for the results.
    :param brake_s: Distance travelled per m/s of speed once braking is asked for, including the dead time.
    :param car: The simulated car (a stage1_scenarios.CarProfile). Default: the lab car with the camera yaw that
        mpc.camera_yaw_offset_deg corrects.
    :param seed: Seed of the camera and lidar noise, or None for the case's.
    :return: The trajectory, outcome and metrics.
    """
    policy = build_policy(settings)
    recorder = _CommandRecorder(policy.car_controller)
    policy.car_controller = recorder
    memory = CarMemory()
    if car is None:
        car = dataclasses.replace(LAB_CAR, camera_yaw_deg=float(settings.get("mpc.camera_yaw_offset_deg", 0.0)))
    front_m = policy.obstacle_detection_config.car_front_m
    rear_m, width_m = policy.avoidance.config.car_rear_m, policy.avoidance.config.car_width_m
    rng = np.random.default_rng(case.seed if seed is None else seed)
    line = case.lane.centre_line()
    cones = case.lane.cones()
    cone_xy = np.array([(cx, cy) for _, _, cx, cy in cones])
    boxes = [box.polygon(line) for box in case.boxes]
    centre = cl._CentreLine(line)
    end_s = line["s"][-1]
    x, y, heading = -cl.RUN_IN_M, case.start_left_m, 0.0
    speed, driven_curvature = 0.0, car.straight_curvature_per_m
    camera_delay = round(car.detection_latency_s / POLICY_PERIOD_S)
    lidar_delay = round(LIDAR_AGE_S / POLICY_PERIOD_S)
    dead_steps = round(car.speed_dead_time_s / cl.SIM_STEP_S)
    brake_time_constant = max(brake_s - car.speed_dead_time_s, cl.SIM_STEP_S)
    steps_per_policy = round(POLICY_PERIOD_S / cl.SIM_STEP_S)
    camera_queue, lidar_queue = [], []
    requested = [DriveCommand(0.0, 0.0)] * (dead_steps + 1)
    command = DriveCommand(0.0, 0.0)
    log = {"x": [], "y": [], "heading": []}
    steps, touched = [], set()
    min_gap, max_outside, max_s = math.inf, -math.inf, -cl.RUN_IN_M
    stop_request = None
    step = 0
    while step * cl.SIM_STEP_S < MAX_DURATION_S:
        s, _, _ = centre.locate(x, y)
        max_s = max(max_s, s)
        for key, value in (("x", x), ("y", y), ("heading", heading)):
            log[key].append(value)
        if step % steps_per_policy == 0:
            pose = (x, y, heading)
            camera_queue.append(cl.detections(visible_cones(cones, pose, boxes), pose, car, rng))
            lidar_queue.append(lidar_points(pose, cone_xy, boxes, rng))
            batch = camera_queue.pop(0) if len(camera_queue) > camera_delay else None
            scan = lidar_queue.pop(0) if len(lidar_queue) > lidar_delay else None
            first = step == 0
            observations = CarObservations(
                cones=None if batch is None else ConeBatch(batch, SensorAge(car.detection_latency_s, None),
                                                           acquisition_to_publish_latency_s=0.0),
                fiducials=None, lidar_scan=None,
                lidar_cartesian=None if scan is None else LidarCartesian(scan, SensorAge(LIDAR_AGE_S, None),
                                                                         "base_link"),
                car=ObservedCarState(None, None, None, WheelSpeed(speed, SensorAge(0.0, None))),
                policy=PolicyState(dt=0.0 if first else POLICY_PERIOD_S, policy_elapsed_s=step * cl.SIM_STEP_S,
                                   is_first_policy_step=first))
            actions = policy.step(observations, memory)
            memory.step()
            command = recorder.command
            steps.append((observations, actions))
            if stop_request is None and policy.action_policy.stopped_for_obstacle:
                stop_request = pose
        # The car: speed after a dead time and a lag (shorter while braking), curvature clipped to the steering
        # limits plus the trim and lagged by the servo, as in curved_lanes.run_curve
        requested.append(command)
        applied = requested.pop(0)
        braking = applied.brake and applied.speed_m_per_s == 0.0
        time_constant = brake_time_constant if braking else (car.speed_time_constant_s or 0.3)
        speed = max(0.0, speed + (applied.speed_m_per_s - speed) * cl.SIM_STEP_S / time_constant)
        target = (min(max(command.curvature_per_m, -car.max_right_curvature_per_m), car.max_left_curvature_per_m)
                  + car.straight_curvature_per_m)
        driven_curvature += (target - driven_curvature) * cl.SIM_STEP_S / cl.STEERING_TIME_CONSTANT_S
        heading += speed * driven_curvature * cl.SIM_STEP_S / 2.0
        x += speed * math.cos(heading) * cl.SIM_STEP_S
        y += speed * math.sin(heading) * cl.SIM_STEP_S
        heading += speed * driven_curvature * cl.SIM_STEP_S / 2.0
        step += 1
        # Scores against the true lane and boxes
        body = body_polygon(x, y, heading, front_m, rear_m, width_m)
        for polygon in boxes:
            min_gap = min(min_gap, polygon_gap(body, polygon))
        touched.update(np.flatnonzero(cone_gaps(cone_xy, x, y, heading, front_m, rear_m, width_m)
                                      < CONE_RADIUS_M).tolist())
        if 0.0 <= s <= end_s:
            for corner_x, corner_y in body:
                _, corner_lateral, _ = centre.locate(corner_x, corner_y)
                max_outside = max(max_outside, abs(corner_lateral) - case.lane.width_m / 2.0)
            centre.locate(x, y)
        if s >= end_s or (step > 50 and speed < STOPPED_SPEED_M_PER_S and command.speed_m_per_s == 0.0):
            break
    body = body_polygon(x, y, heading, front_m, rear_m, width_m)
    final_gap = min((polygon_gap(body, polygon) for polygon in boxes), default=math.inf)
    mpc = policy.action_policy
    if min_gap <= 0.0:
        outcome = "hit box"
    elif touched:
        outcome = "touched cones"
    elif mpc.stopped_for_obstacle:
        outcome = "stopped"
    elif max_s >= end_s - cl.END_TOLERANCE_M:
        outcome = "passed"
    elif mpc.stopped_for_lane_loss:
        outcome = "lane lost"
    elif mpc.stopped_for_solver_failure:
        outcome = "solver failed"
    else:
        outcome = "stopped short"
    metrics = {"min_box_gap_m": min_gap, "final_box_gap_m": final_gap, "cones_touched": float(len(touched)),
               "max_outside_m": float(max_outside), "distance_m": float(max(0.0, max_s)), "time_s": step * cl.SIM_STEP_S}
    return ObstacleResult(case, variant, np.array(log["x"]), np.array(log["y"]), np.array(log["heading"]), outcome,
                          metrics, case.seed if seed is None else seed, body, stop_request, steps)


def variants(stop_per_speed: list[float] | None, overrides: dict,
             named: list[tuple[str, dict]] | None = None) -> dict[str, dict]:
    """
    :param stop_per_speed: Values of mpc.obstacle_stop_distance_per_speed_s to compare, or None for the config's
        value and 0.6.
    :param overrides: Settings applied to every variant, by full parameter name.
    :param named: Variants to compare with the config instead, each a name and its settings by full parameter name.
    :return: The settings of each variant, by name.
    """
    base = read_settings(DEFAULT_SETTINGS_PATH)
    base.update(overrides)
    if named:
        return {"current": base, **{name: {**base, **settings} for name, settings in named}}
    key = "mpc.obstacle_stop_distance_per_speed_s"
    current = base.get(key)
    values = stop_per_speed if stop_per_speed is not None else [current] + ([0.6] if current != 0.6 else [])
    found = {}
    for value in values:
        name = "current" if value == current else f"stop{value:g}"
        found[name] = {**base, key: value}
    return found


def describe(result: ObstacleResult) -> str:
    """
    :return: The outcome with its distance: the gap left to the box when stopped, else the closest pass.
    """
    m = result.metrics
    if result.outcome == "stopped":
        return f"stopped {m['final_box_gap_m']:.2f} m short"
    if result.outcome == "passed" and math.isfinite(m["min_box_gap_m"]):
        return f"passed, {m['min_box_gap_m']:.2f} m clear"
    if result.outcome == "touched cones":
        return f"touched {m['cones_touched']:.0f} cones"
    return result.outcome


def summarise(runs: list[ObstacleResult]) -> str:
    """
    :param runs: One case and variant with several noise seeds.
    :return: How many did what the case expects, the outcomes, the closest pass and the shortest stop.
    """
    counts = {}
    for run in runs:
        counts[run.outcome] = counts.get(run.outcome, 0) + 1
    text = f"{sum(run.ok for run in runs)}/{len(runs)} OK: " + ", ".join(f"{k} x{v}" for k, v in counts.items())
    passes = [run.metrics["min_box_gap_m"] for run in runs if run.outcome == "passed"]
    stops = [run.metrics["final_box_gap_m"] for run in runs if run.outcome == "stopped"]
    if passes and math.isfinite(min(passes)):
        text += f"; closest pass {min(passes):.2f} m"
    if stops:
        text += f"; shortest stop {min(stops):.2f} m"
    return text


def table(results: dict[str, list[list[ObstacleResult]]], names: list[str]) -> str:
    """
    :param results: For each case, each variant's runs.
    :return: A Markdown table: one row per case, one column per variant.
    """
    lines = ["| Case | Should | " + " | ".join(names) + " |", "| --- | --- | " + " | ".join("---" for _ in names) + " |"]
    for case_name, variant_runs in results.items():
        lines.append(f"| {case_name} | {variant_runs[0][0].case.expect} | "
                     + " | ".join(summarise(runs) for runs in variant_runs) + " |")
    return "\n".join(lines)


def plot(results: dict[str, list[list[ObstacleResult]]], names: list[str], path: Path, title: str):
    """Draws every case from above in world coordinates, with each variant's paths, like the lane memory plots: the
    first seed's run solid and the others thin."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch, Polygon

    colours = ["tab:green", "tab:orange", "tab:purple", "tab:red", "tab:brown"]
    columns, panel_width, title_height = 3, 5.8, 1.0
    # Each panel shows its lane at one scale, so a row is as tall as its tallest lane needs
    bounds = []
    for variant_runs in results.values():
        cones = np.array([(cx, cy) for _, _, cx, cy in variant_runs[0][0].case.lane.cones()])
        low, high = cones.min(axis=0) - 0.6, cones.max(axis=0) + 0.6
        bounds.append((low, high))
    rows = math.ceil(len(results) / columns)
    ratios = [max((high[1] - low[1]) / (high[0] - low[0]) for low, high in bounds[row * columns:(row + 1) * columns])
              for row in range(rows)]
    heights = [ratio * panel_width + title_height for ratio in ratios]
    fig = plt.figure(figsize=(panel_width * columns, sum(heights) + 0.8), layout="constrained")
    grid = fig.add_gridspec(rows, columns, height_ratios=heights)
    axes = [fig.add_subplot(grid[index // columns, index % columns]) for index in range(len(results))]
    for ax, (low, high), (case_name, variant_runs) in zip(axes, bounds, results.items()):
        case = variant_runs[0][0].case
        line = case.lane.centre_line()
        ax.plot(line["x"], line["y"], ls=":", color="0.7", lw=1)
        for colour, _, cx, cy in case.lane.cones():
            ax.plot(cx, cy, "o", ms=2.5, color="tab:blue" if colour.name == "BLUE" else "goldenrod")
        for box in case.boxes:
            ax.add_patch(Polygon(box.polygon(line), closed=True, fc="tab:red", ec="darkred", alpha=0.6))
        for colour, runs in zip(colours, variant_runs):
            for index, run in enumerate(runs):
                first = index == 0
                ax.plot(run.x, run.y, color=colour, lw=1.6 if first else 0.7, alpha=1.0 if first else 0.5)
                ax.add_patch(Polygon(run.final_body, closed=True, fill=False, ec=colour, lw=1.0 if first else 0.5,
                                     alpha=1.0 if first else 0.5))
                if run.stop_request is not None:
                    ax.plot(*run.stop_request[:2], "x", color=colour, ms=7 if first else 4, mew=2 if first else 1)
        verdicts = "\n".join(f"{name}: {summarise(runs)}" for name, runs in zip(names, variant_runs))
        ax.set_title(f"{case_name}: {case.description}\n{verdicts}", fontsize=8, loc="left")
        ax.set_xlim(low[0], high[0])
        ax.set_ylim(low[1], high[1])
        ax.set_aspect("equal")
        ax.set_xlabel("world x (m)", fontsize=8)
        ax.set_ylabel("world y (m)", fontsize=8)
        ax.tick_params(labelsize=7)
    handles = ([Line2D([], [], color=c, lw=1.6, label=f"{n}") for c, n in zip(colours, names)]
               + [Line2D([], [], color="k", marker="x", ls="", label="obstacle stop requested"),
                  Patch(fc="tab:red", alpha=0.6, label="box"),
                  Line2D([], [], color="0.7", ls=":", label="lane centre line")])
    fig.legend(handles=handles, loc="outside lower center", ncol=len(handles), fontsize=9, frameon=False)
    fig.suptitle(title, fontsize=12)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def scenario_document(result: ObstacleResult, brake_s: float) -> dict:
    """
    :return: The run as a JSON-ready scenario, in the format of scenarios/data (scenario_wrapper.Scenario).
    """
    case = result.case
    boxes = "; ".join(f"box at {b.s_m:g} m, {b.right_m:+g} to {b.left_m:+g} m" for b in case.boxes) or "no boxes"
    description = (f"Obstacle simulation (tests/policy/obstacle_simulation.py), case {case.name}: {case.description}. "
                   f"{case.lane.name}, {case.lane.width_m:g} m lane, {boxes}. Settings {result.variant}, braking "
                   f"{brake_s:g} s per m/s. Outcome: {describe(result)}.")
    return {"name": f"Obstacle simulation {case.name} ({result.variant}, seed {result.seed})",
            "description": description,
            "test_data": [{"observations": observations.serialise(), "actions": actions.serialise()}
                          for observations, actions in result.steps]}


def _setting(text: str) -> tuple[str, object]:
    """
    :return: The --set argument NAME=VALUE as a name and a bool, number or string.
    """
    name, _, value = text.partition("=")
    if value.lower() in ("true", "false"):
        return name, value.lower() == "true"
    try:
        return name, int(value)
    except ValueError:
        try:
            return name, float(value)
        except ValueError:
            return name, value.strip("\"'")


def main(argv: list[str] | None = None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--cases", nargs="*", choices=[c.name for c in CASES], help="cases to run (default: all)")
    parser.add_argument("--stop-per-speed", type=float, nargs="+", metavar="S",
                        help="values of mpc.obstacle_stop_distance_per_speed_s to compare (default: config and 0.6)")
    parser.add_argument("--seeds", type=int, default=3, help="noise seeds per case and variant (default 3)")
    parser.add_argument("--brake-s", type=float, default=DEFAULT_BRAKE_S,
                        help=f"simulated braking travel per m/s, UNMEASURED (default {DEFAULT_BRAKE_S})")
    parser.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", type=_setting,
                        help="override a policy setting in every variant, e.g. avoidance.obstacle_clearance_m=0.05")
    parser.add_argument("--variant", action="append", nargs="+", metavar=("VARIANT", "NAME=VALUE"),
                        help="compare the config with this named set of settings instead of --stop-per-speed; "
                             "repeat for more")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT / "obstacle_simulation.png",
                        help="figure to save")
    parser.add_argument("--export", type=Path, metavar="FOLDER",
                        help="also save each run as <case>_<variant>_scenario.json for scripts/mcap_export.py")
    args = parser.parse_args(argv)
    named = [(variant[0], dict(_setting(text) for text in variant[1:])) for variant in args.variant or ()]
    found = variants(args.stop_per_speed, dict(args.set), named)
    names = list(found)
    results = {}
    for case in CASES:
        if args.cases and case.name not in args.cases:
            continue
        results[case.name] = [[run_case(case, settings, name, args.brake_s, seed=seed) for seed in range(args.seeds)]
                              for name, settings in found.items()]
    print(f"Braking travel {args.brake_s:g} s per m/s (simulated, unmeasured); {args.seeds} noise seeds; "
          f"overrides {dict(args.set) or 'none'}\n")
    print(table(results, names))
    plot(results, names, args.output,
         f"Obstacle simulation: MovementPolicy with config/ai4r_policy.yaml, braking {args.brake_s:g} s per m/s, "
         f"{args.seeds} noise seeds")
    print(f"\nFigure: {args.output}")
    if args.export:
        args.export.mkdir(parents=True, exist_ok=True)
        for variant_runs in results.values():
            for run in (run for runs in variant_runs for run in runs):
                path = args.export / f"{run.case.name}_{run.variant}_s{run.seed}_scenario.json"
                path.write_text(json.dumps(scenario_document(run, args.brake_s)) + "\n")
                print(path)


if __name__ == "__main__":
    main()
