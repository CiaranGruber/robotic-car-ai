"""Curved cone lanes for the MPC: a small kinematic simulation of the lab car on straights joined by arcs.

The Stage 1 harness (stage1_scenarios.py) only builds straight lanes, but the team reported the car drifting
towards the cones on curved roads, and the hook turn needs curves too. This module draws a lane from straight and
circular-arc pieces, places the cones along both edges, and drives a policy along it with the measured lab car
(real_car_measurements.LAB_CAR): the camera's view, range, noise, 0.2 s delay and yaw, car 20's steering limits
and trim, and a lagged speed and steering response.

The car is a kinematic model (curvature in, no tyre slip), which is right for the lab speeds of about 0.5 m/s. It
is not dream-gym, so the dynamic floor cases stay in stage1_scenarios.py.

The policy is anything with run_policy(observations, lanes), as MovementPolicy calls MpcPolicy on the car. An
optional lanes_from(observations) stands in for lane detection, so the same run can use the dev cone fit
(lanes=None) or a lane detector.

Print the curve results with:
    python3 tests/policy/curved_lanes.py
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from policy.control.actions import DriveCommand  # noqa: E402
from policy.input_output import (  # noqa: E402
    CarActions, ConeBatch, ConeColour, ConeDetection, ObservedCarState, PolicyState, Position, SensorAge, WheelSpeed,
    CarObservations,
)
from real_car_measurements import LAB_CAR  # noqa: E402
from stage1_scenarios import (  # noqa: E402
    CONE_RADIUS_M, POLICY_PERIOD_S, CarProfile, body_corners, command_to_action, distance_to_body,
)

SIM_STEP_S = 0.01
"""Integration step in seconds."""
SAMPLE_M = 0.005
"""Spacing of the centre line samples in metres."""
RUN_IN_M = 0.6
"""Straight run-up before the first cone pair, in metres, so the car starts with cones in view."""
END_TOLERANCE_M = 0.75
"""A run reaches the lane end if the car gets within this distance of the last cone pair. Lane detection reports
no lane once only the last few cones, nearer than the camera's 0.4 m, are left, so the car stops just short."""
STEERING_TIME_CONSTANT_S = 0.1
"""Time constant of the steering servo, in seconds: the driven curvature follows the requested one with this lag."""


@dataclass(frozen=True)
class CurvedLane:
    """A cone lane made of pieces. Each piece is ("straight", length_m) or ("arc", curvature_per_m, sweep_deg),
    with positive curvature turning left. The cones sit on both edges at the same distance along the centre line."""
    pieces: tuple = (("straight", 1.5), ("arc", 0.5, 90.0), ("straight", 1.5))
    width_m: float = 0.8
    cone_spacing_m: float = 0.25
    left_colour: ConeColour = ConeColour.BLUE

    def __post_init__(self):
        if not (self.width_m > 0.0 and self.cone_spacing_m > 0.0):
            raise ValueError("width_m and cone_spacing_m must be positive")
        for piece in self.pieces:
            if piece[0] == "arc" and abs(piece[1]) * self.width_m / 2.0 >= 1.0:
                raise ValueError(f"the arc {piece} is tighter than half the lane width")

    @property
    def name(self) -> str:
        """Short description, such as 'L r2.0 90deg' for one left arc of 2 m radius."""
        arcs = [p for p in self.pieces if p[0] == "arc"]
        return " + ".join(f"{'L' if p[1] > 0 else 'R'} r{1 / abs(p[1]):.1f} {p[2]:g}deg" for p in arcs) or "straight"

    def centre_line(self) -> dict[str, np.ndarray]:
        """Samples the centre line every SAMPLE_M from the start line, after a straight run-in behind it.

        :return: Arrays s (distance from the first cone pair), x, y, heading and curvature.
        """
        pieces = (("straight", RUN_IN_M),) + tuple(self.pieces)
        s_values, curvatures = [], []
        s = -RUN_IN_M
        for piece in pieces:
            if piece[0] == "straight":
                length, curvature = piece[1], 0.0
            elif piece[0] == "arc":
                curvature = piece[1]
                length = math.radians(piece[2]) / abs(curvature)
            else:
                raise ValueError(f"unknown piece {piece}")
            count = max(1, round(length / SAMPLE_M))
            s_values.extend(s + (np.arange(count) + 0.5) * length / count)
            curvatures.extend([curvature] * count)
            s += length
        s_values = np.concatenate([[-RUN_IN_M], np.array(s_values), [s]])
        curvatures = np.concatenate([[0.0], np.array(curvatures), [curvatures[-1]]])
        ds = np.diff(s_values)
        heading = np.concatenate([[0.0], np.cumsum(curvatures[1:] * ds)])
        x = np.concatenate([[0.0], np.cumsum(np.cos(heading[1:] - curvatures[1:] * ds / 2) * ds)]) - RUN_IN_M
        y = np.concatenate([[0.0], np.cumsum(np.sin(heading[1:] - curvatures[1:] * ds / 2) * ds)])
        return {"s": s_values, "x": x, "y": y, "heading": heading, "curvature": curvatures}

    def cones(self) -> list[tuple[ConeColour, float, float, float]]:
        """
        :return: (colour, s, world x, world y) of every cone, the left edge in left_colour.
        """
        line = self.centre_line()
        length = line["s"][-1]
        right_colour = ConeColour.YELLOW if self.left_colour == ConeColour.BLUE else ConeColour.BLUE
        cones = []
        for s in np.arange(0.0, length + 1e-9, self.cone_spacing_m):
            x, y = np.interp(s, line["s"], line["x"]), np.interp(s, line["s"], line["y"])
            heading = np.interp(s, line["s"], line["heading"])
            for side, colour in ((1.0, self.left_colour), (-1.0, right_colour)):
                offset = side * self.width_m / 2.0
                cones.append((colour, float(s), float(x - offset * math.sin(heading)),
                              float(y + offset * math.cos(heading))))
        return cones


@dataclass(frozen=True)
class CurveCase:
    """One run: a lane, where the car starts relative to the centre line, its speed and the car."""
    lane: CurvedLane
    speed_m_per_s: float = 0.5
    start_left_m: float = 0.0
    start_heading_deg: float = 0.0
    car: CarProfile = field(default_factory=lambda: dataclasses.replace(LAB_CAR, camera_yaw_deg=0.0))
    """The lab car with the camera pan held at zero."""
    seed: int = 0
    max_duration_s: float = 40.0


@dataclass
class CurveResult:
    """The trajectory and its scores."""
    case: CurveCase
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    heading: np.ndarray
    lateral_m: np.ndarray
    """Signed distance from the lane centre line, + left, at each sample."""
    commands: list[DriveCommand]
    metrics: dict[str, float] = field(default_factory=dict)
    steps: list[tuple[CarObservations, CarActions]] = field(default_factory=list)
    """Each policy step's observations and the [drive, steering] action of stage1_scenarios.command_to_action, for
    saving the run as a scenario file."""

    @property
    def lane_kept(self) -> bool:
        """No cone touched, the body inside the lane throughout, and the car reached the lane end."""
        m = self.metrics
        return m["cones_touched"] == 0 and m["max_outside_m"] <= 0.0 and m["reached_end"] == 1.0


class _CentreLine:
    """Finds the nearest centre line sample to a point, searching forwards from the last one."""

    def __init__(self, line: dict[str, np.ndarray]):
        self.line = line
        self.index = 0

    def locate(self, x: float, y: float) -> tuple[float, float, float]:
        """
        :return: Distance s along the centre line, signed lateral offset (+ left) and the line's heading there.
        """
        line = self.line
        low, high = max(0, self.index - 40), min(len(line["s"]), self.index + 400)
        d2 = (line["x"][low:high] - x) ** 2 + (line["y"][low:high] - y) ** 2
        self.index = low + int(np.argmin(d2))
        i = self.index
        heading = line["heading"][i]
        dx, dy = x - line["x"][i], y - line["y"][i]
        return line["s"][i], -math.sin(heading) * dx + math.cos(heading) * dy, heading


def detections(cones, pose, car: CarProfile, rng) -> list[ConeDetection]:
    """What the camera reports from a pose: cones in view, in base_link, with noise and the camera's yaw.

    :param cones: (colour, s, x, y) of every cone.
    :param pose: The car's world (x, y, heading).
    :param car: The camera's view, range, noise and yaw.
    :param rng: Random generator for the noise.
    :return: The detections, nearest first.
    """
    car_x, car_y, car_heading = pose
    cos_h, sin_h = math.cos(car_heading), math.sin(car_heading)
    yaw = math.radians(car.camera_yaw_deg)
    cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
    half_fov = math.radians(car.detector_fov_deg) / 2.0
    found = []
    for colour, _, x, y in cones:
        dx, dy = x - car_x, y - car_y
        bx, by = cos_h * dx + sin_h * dy, -sin_h * dx + cos_h * dy
        bx, by = bx + rng.normal(0.0, car.detector_noise_m), by + rng.normal(0.0, car.detector_noise_m)
        # A camera pointing left sees everything turned right
        bx, by = cos_y * bx - sin_y * by, sin_y * bx + cos_y * by
        if not (car.detector_min_range_m <= bx <= car.detector_range_m) or abs(math.atan2(by, bx)) > half_fov:
            continue
        found.append(ConeDetection(Position(bx, by, 0.0), colour, 1.0))
    return sorted(found, key=lambda d: d.pos.x)


def run_curve(case: CurveCase, policy, lanes_from=None) -> CurveResult:
    """Drives a policy along a curved lane.

    :param case: The lane, start and car.
    :param policy: Anything with run_policy(observations, lanes) returning instructions with one DriveCommand.
    :param lanes_from: Optional lane detection, observations -> LaneDetection or None, passed to the policy as
        lanes. None passes lanes=None, so the MPC fits the lane from the cones itself.
    :return: The trajectory and metrics.
    """
    lane, car = case.lane, case.car
    rng = np.random.default_rng(case.seed)
    line = lane.centre_line()
    cones = lane.cones()
    centre = _CentreLine(line)
    end_s = line["s"][-1]
    x = -RUN_IN_M + 0.0
    y = case.start_left_m
    heading = math.radians(case.start_heading_deg)
    speed = case.speed_m_per_s if car.rolling_start else 0.0
    driven_curvature = car.straight_curvature_per_m
    delay_steps = round(car.detection_latency_s / POLICY_PERIOD_S)
    dead_steps = round(car.speed_dead_time_s / SIM_STEP_S)
    pending, requested_speeds = [], [speed] * (dead_steps + 1)
    steps_per_policy = round(POLICY_PERIOD_S / SIM_STEP_S)
    command = DriveCommand(0.0, 0.0)
    log = {"t": [], "x": [], "y": [], "heading": [], "lateral": []}
    commands, steps = [], []
    touched = set()
    max_outside = -math.inf
    max_s = -RUN_IN_M
    step = 0
    time_constant = car.speed_time_constant_s or 0.3
    while step * SIM_STEP_S < case.max_duration_s:
        s, lateral, _ = centre.locate(x, y)
        max_s = max(max_s, s)
        log["t"].append(step * SIM_STEP_S)
        log["x"].append(x)
        log["y"].append(y)
        log["heading"].append(heading)
        log["lateral"].append(lateral)
        if step % steps_per_policy == 0:
            pending.append(detections(cones, (x, y, heading), car, rng))
            batch = pending.pop(0) if len(pending) > delay_steps else None
            first = step == 0
            observations = CarObservations(
                cones=None if batch is None else ConeBatch(batch, SensorAge(car.detection_latency_s, None),
                                                           acquisition_to_publish_latency_s=0.0),
                fiducials=None, lidar_scan=None, lidar_cartesian=None,
                car=ObservedCarState(None, None, None, WheelSpeed(speed, SensorAge(0.0, None))),
                policy=PolicyState(dt=0.0 if first else POLICY_PERIOD_S, policy_elapsed_s=step * SIM_STEP_S,
                                   is_first_policy_step=first),
            )
            lanes = lanes_from(observations) if lanes_from is not None else None
            command = next(i for i in policy.run_policy(observations, lanes) if isinstance(i, DriveCommand))
            commands.append(command)
            action = command_to_action(command, speed)
            steps.append((observations, CarActions(float(action[0]), float(action[1]), None, None, None)))
        # The car: speed after a dead time and a lag, curvature clipped to the steering limits plus the trim
        requested_speeds.append(command.speed_m_per_s)
        speed += (requested_speeds.pop(0) - speed) * SIM_STEP_S / time_constant
        speed = max(speed, 0.0)
        target = (min(max(command.curvature_per_m, -car.max_right_curvature_per_m), car.max_left_curvature_per_m)
                  + car.straight_curvature_per_m)
        driven_curvature += (target - driven_curvature) * SIM_STEP_S / STEERING_TIME_CONSTANT_S
        heading += speed * driven_curvature * SIM_STEP_S / 2.0
        x += speed * math.cos(heading) * SIM_STEP_S
        y += speed * math.sin(heading) * SIM_STEP_S
        heading += speed * driven_curvature * SIM_STEP_S / 2.0
        step += 1
        # Scores against the true lane
        for index, (_, _, cx, cy) in enumerate(cones):
            if abs(cx - x) < 0.5 and abs(cy - y) < 0.5 and distance_to_body(cx, cy, x, y, heading) < CONE_RADIUS_M:
                touched.add(index)
        if 0.0 <= s <= end_s:
            for corner_x, corner_y in body_corners(x, y, heading):
                _, corner_lateral, _ = centre.locate(corner_x, corner_y)
                max_outside = max(max_outside, abs(corner_lateral) - lane.width_m / 2.0)
            centre.locate(x, y)
        if s >= end_s or (step > 50 and speed < 0.02 and command.speed_m_per_s == 0.0):
            break
    result = CurveResult(case, *(np.array(log[k]) for k in ("t", "x", "y", "heading", "lateral")),
                         commands=commands, steps=steps)
    curvatures = np.array([c.curvature_per_m for c in commands]) if commands else np.zeros(1)
    rates = np.diff(curvatures) / POLICY_PERIOD_S if len(curvatures) > 1 else np.zeros(1)
    lateral = np.array(log["lateral"])
    result.metrics = {
        "max_lateral_m": float(np.max(np.abs(lateral))),
        "max_outside_m": float(max_outside),
        "cones_touched": float(len(touched)),
        "reached_end": float(max_s >= end_s - END_TOLERANCE_M),
        "distance_m": float(max(0.0, max_s)),
        "curvature_rate_rms": float(np.sqrt(np.mean(rates ** 2))),
    }
    return result


CURVE_LANES = tuple(
    [CurvedLane(pieces=(("straight", 4.0),))]
    + [CurvedLane(pieces=(("straight", 1.5), ("arc", sign / radius, 90.0), ("straight", 1.5)))
       for radius in (1.5, 2.0, 3.0) for sign in (1.0, -1.0) if not (sign > 0 and radius < 2.0)]
    + [CurvedLane(pieces=(("straight", 1.0), ("arc", 1 / 2.5, 60.0), ("arc", -1 / 2.5, 60.0), ("straight", 1.0)))]
)
"""The lanes the MPC is checked on: a straight, 90 degree curves of 1.5-3 m radius both ways, and an S bend. The
1.5 m left curve is left out because car 20 cannot steer it (0.48 1/m left, a 2.1 m radius)."""
CAMERA_YAWS_DEG = (-3.0, 0.0, 3.0)
"""Camera yaw left over after holding the pan at zero, which nobody has measured yet."""


def pipelines() -> dict:
    """The pipelines to compare, as name -> (make_policy, make_lanes_from). Lane detection is included only where
    policy.lane_detection has LaneDetector (Road Detection's lane edges, not yet on dev)."""
    from policy.action_policy.mpc import MpcConfig
    from policy.action_policy.policy import MpcPolicy
    from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones
    from stage1_scenarios import ConeFilteredPolicy

    found = {"dev: straight-row cone filter, MPC's straight cone fit":
             (lambda: ConeFilteredPolicy(MpcPolicy(MpcConfig()), ConeFilterConfig()), lambda: None)}
    try:
        from policy.lane_detection.lane_detection import LaneDetectionConfig, LaneDetector
    except ImportError:
        return found

    def lane_detection(curves: bool):
        filter_config = ConeFilterConfig(check_straight_rows=not curves)
        detector = LaneDetector(LaneDetectionConfig(left_colour="AUTO", max_curvature_per_m=2.0 if curves else 0.0))
        return lambda observations: detector.detect(
            dataclasses.replace(observations, cones=filter_cones(observations.cones, filter_config)))

    found["lane detection, straight edges (car 20 test run)"] = (lambda: MpcPolicy(MpcConfig()),
                                                                 lambda: lane_detection(False))
    found["lane detection, curves on"] = (lambda: MpcPolicy(MpcConfig()), lambda: lane_detection(True))
    return found


def main():
    car = dataclasses.replace(LAB_CAR, camera_yaw_deg=0.0)
    for name, (make_policy, make_lanes_from) in pipelines().items():
        print(f"\n{name} (MPC defaults; camera yaw {CAMERA_YAWS_DEG} deg, 2 seeds each)")
        print("| Lane | Kept | Largest lat. err. [m] | Cones touched | Curv. rate RMS [1/(m s)] |\n"
              "| --- | --- | --- | --- | --- |")
        total = kept = 0
        for lane in CURVE_LANES:
            results = [run_curve(CurveCase(lane, car=dataclasses.replace(car, camera_yaw_deg=yaw), seed=seed),
                                 make_policy(), make_lanes_from())
                       for yaw in CAMERA_YAWS_DEG for seed in range(2)]
            lane_kept = sum(r.lane_kept for r in results)
            total, kept = total + len(results), kept + lane_kept
            print(f"| {lane.name} | {lane_kept}/{len(results)} | "
                  f"{max(r.metrics['max_lateral_m'] for r in results):.2f} | "
                  f"{max(r.metrics['cones_touched'] for r in results):.0f} | "
                  f"{np.mean([r.metrics['curvature_rate_rms'] for r in results]):.2f} |")
        print(f"Kept {kept}/{total}")


if __name__ == "__main__":
    main()
