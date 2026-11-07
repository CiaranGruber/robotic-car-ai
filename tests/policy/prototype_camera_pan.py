"""PROTOTYPE (throwaway): does panning the camera along the lane keep tight curves and loops in view?

Question: with the OAK-D on its pan servo, can the policy aim the camera at the lane ahead so that the MPC keeps the
tight curves and loops the fixed camera loses? And how much do the servo facts nobody has measured yet matter: the
angle per normalised unit, the servo's speed, and knowing the angle at the moment each image was taken?

Not production code and not a test. Run it, read the table, then fold the decision into policy/ or delete it:
    python3 tests/policy/prototype_camera_pan.py

The model, on top of curved_lanes.py's lanes, kinematic car and lab-car camera (86 degrees, 0.4-2.5 m, 0.03 m noise,
0.2 s delay):
- The camera turns about a vertical pan axis at the camera, 0.10 m behind base_link (config/oakd_cone_detector.yaml).
- DREAM's camera TF is fixed at zero pan, so the detector reports every cone as if the camera pointed forwards. The
  policy turns them back about the pan axis by the angle it believes the camera had when the image was taken.
- The servo follows its target with a lag and a speed limit, and turns pan_scale times the angle the policy expects.
- The policy aims the camera at the detected lane centre AIM_M ahead, and holds it while the lane is lost.
Modes: fixed (pan held at zero, as now); ideal (the true angle at the image's time, as a working encoder would give);
modelled (no encoder: a nominal servo model of the commands, while the real servo turns 10% further and slower);
naive (the latest commanded angle, not the angle at the image's time, with the same 10% error).
"""
from __future__ import annotations

import dataclasses
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import curved_lanes as cl  # noqa: E402
from policy.action_policy.mpc import MpcConfig  # noqa: E402
from policy.action_policy.policy import MpcPolicy  # noqa: E402
from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones  # noqa: E402
from policy.control.actions import DriveCommand  # noqa: E402
from policy.input_output import (  # noqa: E402
    CarObservations, ConeBatch, ConeColour, ConeDetection, ObservedCarState, PolicyState, Position, SensorAge,
    WheelSpeed,
)
from policy.lane_detection.lane_detection import LaneDetectionConfig, LaneDetector  # noqa: E402
from real_car_measurements import LAB_CAR  # noqa: E402
from stage1_scenarios import (  # noqa: E402
    CONE_RADIUS_M, POLICY_PERIOD_S, body_corners, distance_to_body,
)

AXIS_X_M = -0.10
"""Pan axis (and camera) position along base_link's x axis."""
AIM_M = 1.2
"""Distance along the detected lane centre that the camera aims at."""
MAX_PAN_DEG = 60.0
"""Assumed mechanical pan limit either side. Not measured."""
CAMERA = LAB_CAR
"""The camera's view; --fov and --range narrow it to check how much the conclusion depends on the measured view."""


@dataclass(frozen=True)
class Servo:
    """A pan servo: first-order lag, then a speed limit."""
    lag_s: float = 0.05
    speed_deg_per_s: float = 300.0
    scale: float = 1.0
    """Angle actually turned per angle the policy expects."""

    def step(self, angle_deg: float, target_deg: float, dt: float) -> float:
        rate = (self.scale * target_deg - angle_deg) / self.lag_s
        rate = max(-self.speed_deg_per_s, min(self.speed_deg_per_s, rate))
        return angle_deg + rate * dt


NOMINAL = Servo()
"""What the policy assumes about the servo."""
REAL = Servo(lag_s=0.08, speed_deg_per_s=200.0, scale=1.1)
"""The servo in the modelled and naive modes: 10% further, slower and laggier than assumed."""


def detect(cones, pose, pan_deg: float, rng) -> list[ConeDetection]:
    """Cones the panned camera sees, reported in base_link as if the camera were at zero pan (DREAM's fixed TF)."""
    car = CAMERA
    car_x, car_y, car_heading = pose
    cos_h, sin_h = math.cos(car_heading), math.sin(car_heading)
    cos_p, sin_p = math.cos(math.radians(pan_deg)), math.sin(math.radians(pan_deg))
    half_fov = math.radians(car.detector_fov_deg) / 2.0
    found = []
    for colour, _, x, y in cones:
        dx, dy = x - car_x, y - car_y
        bx = cos_h * dx + sin_h * dy + rng.normal(0.0, car.detector_noise_m)
        by = -sin_h * dx + cos_h * dy + rng.normal(0.0, car.detector_noise_m)
        # Into the camera's frame, about the pan axis
        rx = bx - AXIS_X_M
        cx, cy = cos_p * rx + sin_p * by, -sin_p * rx + cos_p * by
        if not (car.detector_min_range_m <= cx <= car.detector_range_m) or abs(math.atan2(cy, cx)) > half_fov:
            continue
        found.append(ConeDetection(Position(cx + AXIS_X_M, cy, 0.0), colour, 1.0))
    return sorted(found, key=lambda d: d.pos.x)


def in_view(cones, pose, pan_deg: float) -> list[tuple[float, float]]:
    """World positions of the cones inside the camera's view, without noise, for drawing."""
    car = CAMERA
    car_x, car_y, car_heading = pose
    cos_h, sin_h = math.cos(car_heading), math.sin(car_heading)
    cos_p, sin_p = math.cos(math.radians(pan_deg)), math.sin(math.radians(pan_deg))
    half_fov = math.radians(car.detector_fov_deg) / 2.0
    seen = []
    for _, _, x, y in cones:
        dx, dy = x - car_x, y - car_y
        rx, by = cos_h * dx + sin_h * dy - AXIS_X_M, -sin_h * dx + cos_h * dy
        cx, cy = cos_p * rx + sin_p * by, -sin_p * rx + cos_p * by
        if car.detector_min_range_m <= cx <= car.detector_range_m and abs(math.atan2(cy, cx)) <= half_fov:
            seen.append((x, y))
    return seen


def unpan(cones: list[ConeDetection], pan_deg: float) -> list[ConeDetection]:
    """Turns reported cones back about the pan axis by the angle the camera is believed to have had."""
    if pan_deg == 0.0:
        return list(cones)
    cos_p, sin_p = math.cos(math.radians(pan_deg)), math.sin(math.radians(pan_deg))
    out = []
    for c in cones:
        rx = c.pos.x - AXIS_X_M
        out.append(ConeDetection(Position(AXIS_X_M + cos_p * rx - sin_p * c.pos.y, sin_p * rx + cos_p * c.pos.y,
                                          c.pos.z), c.colour, c.confidence))
    return out


def aim_deg(lanes) -> float | None:
    """Pan angle that points the camera at the detected lane centre AIM_M ahead, or None without a lane."""
    if lanes is None or not lanes.has_lane or lanes.centre_line is None:
        return None
    arc = lanes.centre_line
    px, py = arc.point_at(arc.project(0.0, 0.0) + AIM_M)
    return max(-MAX_PAN_DEG, min(MAX_PAN_DEG, math.degrees(math.atan2(py, px - AXIS_X_M))))


def run(lane: cl.CurvedLane, mode: str, seed: int, frames: list | None = None) -> dict:
    """Drives the MPC with lane detection (curves on) along a lane, panning the camera as the mode says. With frames,
    appends each policy step's world-frame picture for write_world_mcap."""
    rng = np.random.default_rng(seed)
    car = dataclasses.replace(CAMERA, camera_yaw_deg=0.0)
    line = lane.centre_line()
    cones = lane.cones()
    centre = cl._CentreLine(line)
    end_s = line["s"][-1]
    x, y, heading = -cl.RUN_IN_M, float(rng.uniform(-0.1, 0.1)), math.radians(float(rng.uniform(-5.0, 5.0)))
    speed, driven_curvature = 0.0, car.straight_curvature_per_m
    servo = NOMINAL if mode == "ideal" else REAL
    pan_true = pan_cmd = believed = 0.0
    history, poses = [], []
    delay_steps = round(car.detection_latency_s / POLICY_PERIOD_S)
    dead_steps = round(car.speed_dead_time_s / cl.SIM_STEP_S)
    pending, requested_speeds = [], [speed] * (dead_steps + 1)
    steps_per_policy = round(POLICY_PERIOD_S / cl.SIM_STEP_S)
    mpc = MpcPolicy(MpcConfig())
    detector = LaneDetector(LaneDetectionConfig(left_colour="AUTO", max_curvature_per_m=2.0))
    filter_config = ConeFilterConfig(check_straight_rows=False)
    command = DriveCommand(0.0, 0.0)
    touched, max_outside, max_s = set(), -math.inf, -cl.RUN_IN_M
    lane_steps = policy_steps = 0
    pans = []
    step = 0
    while step * cl.SIM_STEP_S < 60.0:
        s, lateral, _ = centre.locate(x, y)
        max_s = max(max_s, s)
        if step % steps_per_policy == 0:
            pending.append((detect(cones, (x, y, heading), pan_true, rng), pan_true))
            poses.append((x, y, heading))
            delivered = pending.pop(0) if len(pending) > delay_steps else None
            history.append(believed)
            first = step == 0
            batch = None
            correction = 0.0
            if delivered is not None:
                reported, pan_at_image = delivered
                if mode == "ideal":
                    correction = pan_at_image
                elif mode == "modelled":
                    correction = history[-1 - delay_steps]
                elif mode == "naive":
                    correction = pan_cmd
                else:
                    correction = 0.0
                batch = ConeBatch(unpan(reported, correction), SensorAge(car.detection_latency_s, None), 0.0)
            observations = CarObservations(
                cones=batch, fiducials=None, lidar_scan=None, lidar_cartesian=None,
                car=ObservedCarState(None, None, None, WheelSpeed(speed, SensorAge(0.0, None))),
                policy=PolicyState(dt=0.0 if first else POLICY_PERIOD_S, policy_elapsed_s=step * cl.SIM_STEP_S,
                                   is_first_policy_step=first))
            lanes = detector.detect(dataclasses.replace(observations, cones=filter_cones(batch, filter_config)))
            command = next(i for i in mpc.run_policy(observations, lanes) if isinstance(i, DriveCommand))
            policy_steps += 1
            lane_steps += bool(lanes is not None and lanes.has_lane)
            if mode != "fixed":
                aimed = aim_deg(lanes)
                if aimed is not None:
                    # The lane was seen with the camera at the image's angle; aim relative to the car now
                    pan_cmd = aimed
            if frames is not None:
                # The policy's lane centre, placed in the world from where the car was when the image was taken
                lane_world = []
                if len(poses) > delay_steps and lanes is not None and lanes.has_lane and lanes.centre_line:
                    arc, (ax, ay, ah) = lanes.centre_line, poses[-1 - delay_steps]
                    start = arc.project(0.0, 0.0)
                    for along in np.linspace(start, start + (lanes.visible_until_m or 2.0), 25):
                        bx, by = arc.point_at(float(along))
                        lane_world.append((ax + bx * math.cos(ah) - by * math.sin(ah),
                                           ay + bx * math.sin(ah) + by * math.cos(ah)))
                frames.append({"t": step * cl.SIM_STEP_S, "pose": (x, y, heading), "pan_true": pan_true,
                               "pan_cmd": pan_cmd, "correction": correction, "lane_world": lane_world,
                               "seen": in_view(cones, (x, y, heading), pan_true), "speed": speed,
                               "curvature": command.curvature_per_m, "lateral_true": lateral,
                               "lateral_est": lanes.lateral_error_m if lanes is not None and lanes.has_lane else None,
                               "lane_est_ok": bool(lanes is not None and lanes.has_lane)})
            # The policy's belief: a nominal servo model of its own commands (exact in the ideal mode)
            for _ in range(steps_per_policy):
                believed = NOMINAL.step(believed, pan_cmd, cl.SIM_STEP_S)
            pans.append(abs(pan_true))
        pan_true = servo.step(pan_true, pan_cmd, cl.SIM_STEP_S)
        requested_speeds.append(command.speed_m_per_s)
        speed = max(0.0, speed + (requested_speeds.pop(0) - speed) * cl.SIM_STEP_S / car.speed_time_constant_s)
        target = (min(max(command.curvature_per_m, -car.max_right_curvature_per_m), car.max_left_curvature_per_m)
                  + car.straight_curvature_per_m)
        driven_curvature += (target - driven_curvature) * cl.SIM_STEP_S / cl.STEERING_TIME_CONSTANT_S
        heading += speed * driven_curvature * cl.SIM_STEP_S / 2.0
        x += speed * math.cos(heading) * cl.SIM_STEP_S
        y += speed * math.sin(heading) * cl.SIM_STEP_S
        heading += speed * driven_curvature * cl.SIM_STEP_S / 2.0
        step += 1
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
    reached = max_s >= end_s - cl.END_TOLERANCE_M
    hit = bool(touched) or max_outside > 0.0
    stop = ("lane_loss" if mpc.stopped_for_lane_loss else "obstacle" if mpc.stopped_for_obstacle
            else "solver" if mpc.stopped_for_solver_failure else "")
    return {"kept": reached and not hit, "hit": hit, "done": max(0.0, max_s) / end_s,
            "lane": lane_steps / max(policy_steps, 1), "pan": float(np.max(pans)) if pans else 0.0, "stop": stop}


def oval(radius: float, straight: float, sign: float) -> cl.CurvedLane:
    return cl.CurvedLane(pieces=(("straight", straight / 2), ("arc", sign / radius, 180.0), ("straight", straight),
                                 ("arc", sign / radius, 180.0), ("straight", straight / 2 - 0.3)))


TRACKS = {
    "right 90 r1.25": cl.CurvedLane(pieces=(("straight", 1.5), ("arc", -1 / 1.25, 90.0), ("straight", 1.5))),
    "right 90 r1.5": cl.CurvedLane(pieces=(("straight", 1.5), ("arc", -1 / 1.5, 90.0), ("straight", 1.5))),
    "right hairpin r1.25": cl.CurvedLane(pieces=(("straight", 1.5), ("arc", -1 / 1.25, 180.0), ("straight", 1.5))),
    "left 90 r2.1": cl.CurvedLane(pieces=(("straight", 1.5), ("arc", 1 / 2.1, 90.0), ("straight", 1.5))),
    "left 90 r2.5": cl.CurvedLane(pieces=(("straight", 1.5), ("arc", 1 / 2.5, 90.0), ("straight", 1.5))),
    "S bend r2.5": cl.CurvedLane(pieces=(("straight", 1.0), ("arc", 1 / 2.5, 60.0), ("arc", -1 / 2.5, 60.0),
                                        ("straight", 1.0))),
    "oval cw r1.5 (loop)": oval(1.5, 3.0, -1.0),
    # A circle starts on its arc: a straight before it would put cones across the circle's last part
    "circle cw r1.5 (loop)": cl.CurvedLane(pieces=(("arc", -1 / 1.5, 340.0),)),
    "circle ccw r1.5 (loop)": cl.CurvedLane(pieces=(("arc", 1 / 1.5, 340.0),)),
    "oval ccw r2.5 (loop)": oval(2.5, 3.0, 1.0),
}
MODES = ("fixed", "ideal", "modelled", "naive")
SEEDS = (0, 1, 2)


EXPORTS = (("right hairpin r1.25", "fixed"), ("right hairpin r1.25", "ideal"), ("circle ccw r1.5 (loop)", "fixed"),
           ("circle ccw r1.5 (loop)", "ideal"), ("oval cw r1.5 (loop)", "fixed"), ("oval cw r1.5 (loop)", "naive"))
"""The runs to look at in Foxglove: a hairpin with and without pan, the left circle car 20 cannot steer, and the
unstable naive pan."""


def export(folder: Path, camera) -> None:
    """Writes each EXPORTS run (seed 0) as a top-down Foxglove MCAP file, and a layout to import once."""
    import json
    import re
    global CAMERA
    CAMERA = camera
    folder.mkdir(parents=True, exist_ok=True)
    for name, mode in EXPORTS:
        frames = []
        result = run(TRACKS[name], mode, 0, frames)
        stem = "pan_" + re.sub(r"[^a-z0-9]+", "_", f"{name} {mode}".lower()).strip("_")
        write_world_mcap(folder / f"{stem}.mcap", TRACKS[name], frames, f"{name}, {mode}")
        print(f"{folder / stem}.mcap: {'kept' if result['kept'] else 'NOT kept'}, lane {result['lane']:.0%}, "
              f"largest pan {result['pan']:.0f} deg")
    (folder / "pan_sim_layout.json").write_text(json.dumps(layout(), indent=1) + "\n")
    print(f"Layout: {folder / 'pan_sim_layout.json'}")


def write_world_mcap(path: Path, lane: cl.CurvedLane, frames: list[dict], title: str) -> None:
    """Writes a run as a top-down world view: /sim/track (cones and centre line), /sim/car (body, camera view in
    green, pan target in orange, trail in blue, the policy's lane centre in magenta, seen cones ringed) and
    /sim/state (numbers for the Plot panel)."""
    import foxglove
    from foxglove.channels import SceneUpdateChannel
    from foxglove.messages import (Color, LinePrimitive, LinePrimitiveLineType, Point3, Pose, Quaternion,
                                   SceneEntity, SceneUpdate, SpherePrimitive, TextPrimitive, Timestamp, Vector3)

    def pose(px=0.0, py=0.0):
        return Pose(position=Vector3(x=float(px), y=float(py), z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

    def line(points, rgba, closed=False, width=0.02):
        return LinePrimitive(type=LinePrimitiveLineType.LineLoop if closed else LinePrimitiveLineType.LineStrip,
                             pose=pose(), thickness=width, color=Color(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3]),
                             points=[Point3(x=float(px), y=float(py), z=0.0) for px, py in points])

    def sphere(px, py, size, rgba):
        return SpherePrimitive(pose=pose(px, py), size=Vector3(x=size, y=size, z=size),
                               color=Color(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3]))

    start_ns = 1_700_000_000 * 10 ** 9
    context = foxglove.Context()
    centre = lane.centre_line()
    half_fov = math.radians(CAMERA.detector_fov_deg) / 2.0
    with foxglove.open_mcap(path, allow_overwrite=True, context=context):
        track = SceneUpdateChannel("/sim/track", context=context)
        car = SceneUpdateChannel("/sim/car", context=context)
        state = foxglove.Channel("/sim/state", message_encoding="json", context=context)
        trail = []
        for index, frame in enumerate(frames):
            time_ns = start_ns + round(frame["t"] * 1e9)
            stamp = Timestamp(sec=time_ns // 10 ** 9, nsec=time_ns % 10 ** 9)
            if index == 0:
                cones = [sphere(cx, cy, 0.1, (0.15, 0.35, 1.0, 1.0) if colour is ConeColour.BLUE
                                else (1.0, 0.85, 0.0, 1.0)) for colour, _, cx, cy in lane.cones()]
                track.log(SceneUpdate(entities=[SceneEntity(
                    timestamp=stamp, frame_id="world", id="track", spheres=cones,
                    lines=[line(list(zip(centre["x"][::20], centre["y"][::20])), (0.6, 0.6, 0.6, 0.6), width=0.01)])]),
                    log_time=time_ns)
            x, y, heading = frame["pose"]
            trail.append((x, y))
            cam_x, cam_y = x + AXIS_X_M * math.cos(heading), y + AXIS_X_M * math.sin(heading)
            view = heading + math.radians(frame["pan_true"])
            reach = CAMERA.detector_range_m
            wedge = [(cam_x, cam_y)] + [(cam_x + reach * math.cos(view + a), cam_y + reach * math.sin(view + a))
                                         for a in np.linspace(-half_fov, half_fov, 15)]
            aim = heading + math.radians(frame["pan_cmd"])
            lines = [line(body_corners(x, y, heading), (1.0, 1.0, 1.0, 1.0), closed=True, width=0.025),
                     line(wedge, (0.2, 0.9, 0.3, 0.9), closed=True, width=0.015),
                     line([(cam_x, cam_y), (cam_x + 1.0 * math.cos(aim), cam_y + 1.0 * math.sin(aim))],
                          (1.0, 0.5, 0.0, 1.0), width=0.02),
                     line(trail[-600:], (0.3, 0.8, 1.0, 0.8), width=0.015)]
            if frame["lane_world"]:
                lines.append(line(frame["lane_world"], (1.0, 0.2, 0.8, 1.0), width=0.03))
            text = TextPrimitive(pose=pose(x, y + 0.6), billboard=True, font_size=14.0, scale_invariant=True,
                                 color=Color(r=1.0, g=1.0, b=1.0, a=1.0),
                                 text=f"{title}  t={frame['t']:.1f} s  pan {frame['pan_true']:+.0f} deg")
            car.log(SceneUpdate(entities=[SceneEntity(
                timestamp=stamp, frame_id="world", id="car", lines=lines, texts=[text],
                spheres=[sphere(px, py, 0.17, (0.2, 1.0, 0.3, 0.45)) for px, py in frame["seen"]])]),
                log_time=time_ns)
            state.log({"pan_true_deg": frame["pan_true"], "pan_target_deg": frame["pan_cmd"],
                       "correction_deg": frame["correction"], "lane_seen": 1 if frame["lane_est_ok"] else 0,
                       "lateral_true_m": frame["lateral_true"], "lateral_estimated_m": frame["lateral_est"],
                       "speed_m_per_s": frame["speed"], "curvature_command_per_m": frame["curvature"]},
                      log_time=time_ns)


def layout() -> dict:
    """A Foxglove layout for the /sim topics, made from docs/foxglove/mcap_replay_layout.json: a top-down 3D panel
    fixed on the world frame beside plots of the camera pan and the lane."""
    import copy
    import json
    base = json.loads((Path(__file__).resolve().parents[2] / "docs/foxglove/mcap_replay_layout.json").read_text())
    view = copy.deepcopy(base["configById"]["3D!replay"])
    view.update(followMode="follow-none", followTf="world",
                topics={"/sim/track": {"visible": True}, "/sim/car": {"visible": True}})
    view["cameraState"].update(distance=12, target=[1.5, 0.0, 0.0], targetOffset=[0.0, 0.0, 0.0])
    for layer in view.get("layers", {}).values():
        if "frameId" in layer:
            layer["frameId"] = "world"
    plot = {key: value for key, value in base["configById"]["Plot!lane"].items() if key not in ("title", "paths")}

    def paths(*pairs):
        return [{"value": f"/sim/state.{field}", "enabled": True, "timestampMethod": "receiveTime", "label": label}
                for field, label in pairs]
    return {"configById": {
        "3D!sim": view,
        "Plot!pan": {**plot, "title": "Camera pan (deg)", "paths": paths(
            ("pan_true_deg", "true angle"), ("pan_target_deg", "target"), ("correction_deg", "cones turned back by"))},
        "Plot!lane": {**plot, "title": "Lane", "paths": paths(
            ("lateral_true_m", "true distance left of centre (m)"), ("lateral_estimated_m", "policy's estimate (m)"),
            ("lane_seen", "lane seen (1/0)"), ("curvature_command_per_m", "curvature command (1/m)"))},
        "RawMessages!state": {**base["configById"]["RawMessages!state"], "topicPath": "/sim/state"}},
        "globalVariables": {}, "userNodes": {}, "playbackConfig": base.get("playbackConfig", {"speed": 1.0}),
        "layout": {"direction": "row", "first": "3D!sim", "second": {
            "direction": "column", "first": "Plot!pan", "second": {
                "direction": "column", "first": "Plot!lane", "second": "RawMessages!state", "splitPercentage": 65},
            "splitPercentage": 40}, "splitPercentage": 62}}


def _job(args):
    global CAMERA
    name, mode, seed, camera = args
    CAMERA = camera
    return name, mode, seed, run(TRACKS[name], mode, seed)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--fov", type=float, default=LAB_CAR.detector_fov_deg, help="camera view, degrees")
    parser.add_argument("--range", type=float, default=LAB_CAR.detector_range_m, help="detection range, metres")
    parser.add_argument("--export", type=Path, metavar="DIR", help="save EXPORTS as scenarios and Foxglove MCAP files")
    args = parser.parse_args()
    camera = dataclasses.replace(LAB_CAR, detector_fov_deg=args.fov, detector_range_m=args.range)
    if args.export:
        export(args.export, camera)
        return
    jobs = [(name, mode, seed, camera) for name in TRACKS for mode in MODES for seed in SEEDS]
    with ProcessPoolExecutor() as pool:
        results = list(pool.map(_job, jobs))
    print(f"Camera view {args.fov:g} degrees, range {args.range:g} m.")
    print("PROTOTYPE camera pan. Per track and mode over seeds", SEEDS, ": lanes kept / runs, mean distance done,")
    print("mean share of steps with a lane, largest pan angle, and why runs stopped early.\n")
    print(f"{'track':24s}" + "".join(f"{mode:>27s}" for mode in MODES))
    totals = {mode: [0, 0] for mode in MODES}
    for name in TRACKS:
        cells = []
        for mode in MODES:
            rows = [r for n, m, _, r in results if n == name and m == mode]
            kept = sum(r["kept"] for r in rows)
            totals[mode][0] += kept
            totals[mode][1] += len(rows)
            stops = ",".join(sorted({r["stop"] for r in rows if r["stop"] and not r["kept"]})) or "-"
            hits = sum(r["hit"] for r in rows)
            cells.append(f"{kept}/{len(rows)} {np.mean([r['done'] for r in rows]):4.0%} "
                         f"L{np.mean([r['lane'] for r in rows]):4.0%} p{max(r['pan'] for r in rows):3.0f} "
                         f"{'H' + str(hits) if hits else ''}{stops[:9]:>9s}")
        print(f"{name:24s}" + "".join(f"{c:>27s}" for c in cells))
    print("\nTotal kept: " + ", ".join(f"{mode} {k}/{n}" for mode, (k, n) in totals.items()))


if __name__ == "__main__":
    main()
