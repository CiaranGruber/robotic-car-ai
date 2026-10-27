"""Local four-way intersection baseline and opt-in navigation evaluation.

Run: OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m \
    tests.policy.intersection_simulation --output build/aruco-intersection

Add --navigate --output build/aruco-navigation-reviewed for the opt-in 2.4 m-wide cross.

Synthetic decoded marker poses exercise MarkerReader, not the camera's pixel detector. The same
physical cones, sensor disturbances and full control chain are used for each requested direction.
Ground truth is confined to sensor generation, the plant and scoring. No branch-selected lane or
reference path is supplied to the policy. This is a diagnostic, not vehicle qualification.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time

import numpy as np
import yaml

from policy.action_policy.avoidance import AvoidanceConfig
from policy.action_policy.mpc import MpcConfig
from policy.action_policy.intersection import IntersectionConfig
from policy.action_policy.intersection_navigator import IntersectionNavigatorConfig
from policy.cone_filter.cone_filter import ConeFilterConfig
from policy.control.control import ControlConfig
from policy.input_output import (AngularVelocity, CarObservations, ConeBatch, ConeColour, ConeDetection, FiducialBatch,
                                 FiducialDetection, FiducialPose, ImagePoint, LidarCartesian,
                                 LidarCartesianPoint, ObservedCarState, PolicyState, Position, Quaternion,
                                 SensorAge, WheelSpeed)
from policy.lane_detection.lane_detection import LaneDetectionConfig
from policy.markers.markers import MarkerConfig
from policy.memory import CarMemory
from policy.obstacle_detection.obstacle_detection import ObstacleDetectionConfig
from policy.policy_runner import MovementPolicy
from tests.policy.car_models import CAR_20
from tests.policy.stage1_scenarios import body_corners, distance_to_body

ROOT = Path(__file__).resolve().parents[2]
PERIOD_S, SUBSTEP_S, LATENCY_STEPS = .1, .01, 2
CORNERS = tuple(ImagePoint(u, v) for u, v in ((0., 0.), (1., 0.), (1., 1.), (0., 1.)))
DIRECTIONS = {"left": "TURN_LEFT", "right": "TURN_RIGHT", "straight": "STRAIGHT"}


@dataclass(frozen=True)
class CrossIntersection:
    """Orthogonal road arms with physical cone boundaries and no oracle driving line."""
    width_m: float = 1.
    arm_length_m: float = 5.
    cone_spacing_m: float = .5
    marker_x_m: float = -2.4
    marker_y_m: float = -.7
    exit_gate_m: float = 2.
    corner_anchored: bool = False
    closed_exit: str | None = None

    def __post_init__(self):
        if self.closed_exit not in (None, "left", "right", "straight"):
            raise ValueError("closed_exit must be left, right, straight or None")

    def cones(self):
        half = self.width_m/2
        cones = []
        if self.corner_anchored:
            distances = np.arange(half, self.arm_length_m+.001, self.cone_spacing_m)
            positions = np.r_[-distances[::-1], distances]
        else:
            positions = np.arange(-self.arm_length_m, self.arm_length_m+.001, self.cone_spacing_m)
        for along in positions:
            if abs(along) < half-1e-9:
                continue  # Leave the crossing open; never select cones using the marker command.
            for across, colour in ((half, ConeColour.BLUE), (-half, ConeColour.YELLOW)):
                cones.append((float(along), across, colour))
                if abs(along) > half+1e-9:  # No overlapping corner cones of contradictory colours.
                    cones.append((-across, float(along), colour))
        if self.closed_exit is not None:
            cones = [(x, y, c) for x, y, c in cones if not (
                (self.closed_exit == "straight" and x > half+1e-9)
                or (self.closed_exit == "left" and y > half+1e-9)
                or (self.closed_exit == "right" and y < -half-1e-9))]
            for across in np.arange(-half+self.cone_spacing_m, half-1e-9, self.cone_spacing_m):
                if self.closed_exit == "straight":
                    cones.append((half, float(across), ConeColour.YELLOW))
                else:
                    sign = 1 if self.closed_exit == "left" else -1
                    cones.append((float(across), sign*half, ConeColour.BLUE if sign == 1 else ConeColour.YELLOW))
        return cones

    def road_rectangles(self):
        h, end = self.width_m/2, self.arm_length_m
        right = h if self.closed_exit == "straight" else end
        lower = -h if self.closed_exit == "right" else -end
        upper = h if self.closed_exit == "left" else end
        return ((-end, -h, right+end, 2*h), (-h, lower, 2*h, upper-lower))

    def point_outside_m(self, x, y):
        return min(max(left-x, x-left-width, bottom-y, y-bottom-height)
                   for left, bottom, width, height in self.road_rectangles())

    def body_outside_m(self, pose):
        # A cross is non-convex. Split every body edge where road membership can change, then
        # check each interval as well as corners. Corners alone miss cuts across an inner corner.
        corners = body_corners(*pose)
        worst = max(self.point_outside_m(*p) for p in corners)
        bounds = (-self.arm_length_m, -self.width_m/2, self.width_m/2, self.arm_length_m)
        for a, b in zip(corners, np.roll(corners, -1, axis=0)):
            delta, cuts = b-a, [0., 1.]
            for dim in (0, 1):
                if abs(delta[dim]) > 1e-12:
                    cuts.extend(t for bound in bounds if 0 < (t := (bound-a[dim])/delta[dim]) < 1)
            cuts = sorted(cuts)
            points = cuts+[(a+b)/2 for a, b in zip(cuts, cuts[1:])]
            worst = max(worst, *(self.point_outside_m(*(a+t*delta)) for t in points))
        return max(0., float(worst))

    def exit_at(self, pose):
        x, y, heading = pose
        h = self.width_m/2-.125  # Keep the body width inside the selected arm.
        angle = lambda target: abs(math.atan2(math.sin(heading-target), math.cos(heading-target)))
        if x >= self.exit_gate_m and abs(y) <= h and angle(0.) < math.radians(20):
            return None if self.closed_exit == "straight" else "straight"
        if y >= self.exit_gate_m and abs(x) <= h and angle(math.pi/2) < math.radians(20):
            return None if self.closed_exit == "left" else "left"
        if y <= -self.exit_gate_m and abs(x) <= h and angle(-math.pi/2) < math.radians(20):
            return None if self.closed_exit == "right" else "right"
        return None


def settings():
    policy = yaml.safe_load((ROOT/"config/ai4r_policy.yaml").read_text())["/**/ai4r_policy"]["ros__parameters"]
    detector = yaml.safe_load((ROOT/"config/aruco_detector.yaml").read_text())["/**/aruco_detector"]["ros__parameters"]
    return policy, detector


def make_policy(params, navigation=False):
    navigator = dict(params.get("intersection_navigator", {}), enabled=navigation)
    # The plant is CAR_20, so the speed feed-forward uses its breakaway rather than the configured car's
    control = dict(params["control"], speed_ff_offset=CAR_20.breakaway_drive)
    return MovementPolicy(MpcConfig(**params["mpc"]), ConeFilterConfig(**params["cone_filter"]),
                          ControlConfig(**control), action_policy=params["action_policy"],
                          lane_detection_config=LaneDetectionConfig(**params["lane_detection"]),
                          obstacle_detection_config=ObstacleDetectionConfig(**params["obstacle_detection"]),
                          avoidance_config=AvoidanceConfig(**params["avoidance"]),
                          marker_config=MarkerConfig(**params["markers"]),
                          intersection_config=IntersectionConfig(enabled=False),
                          navigator_config=IntersectionNavigatorConfig(**navigator))


def body_point(x, y, pose):
    dx, dy = x-pose[0], y-pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return c*dx+s*dy, -s*dx+c*dy


def visible(x, y):
    return x >= .4 and math.hypot(x, y) <= 2.5 and abs(math.atan2(y, x)) <= math.radians(43)


def camera_frame(track, pose, rng, profile):
    detections = []
    for x, y, colour in track.cones():
        missed = rng.random() < (.2 if profile == "noisy_cones" else 0.)
        noise = rng.normal(0., .02 if profile == "noisy_cones" else 0., 2)
        forward, left = body_point(x, y, pose)
        if visible(forward, left) and not missed:
            detections.append(ConeDetection(Position(forward+noise[0], left+noise[1], .1), colour, .9))
    if profile == "brief_cone_loss" and -1.3 < pose[0] < -1.1:
        detections = []
    return detections


def marker_frame(track, pose, marker_id, profile, step, config, consecutive):
    forward, left = body_point(track.marker_x_m, track.marker_y_m, pose)
    seen = visible(forward, left) and profile != "missing_marker"
    if profile == "marker_disappears" and pose[0] > -3.25:
        seen = False
    if profile == "single_marker_frame" and step != 12:
        seen = False
    consecutive = consecutive+1 if seen else 0
    allowed = config.get("allowed_marker_ids", [])
    accepted = seen and consecutive >= config["min_consecutive_frames"] and (not allowed or marker_id in allowed)
    if not accepted:
        return [], consecutive
    pose = None if profile == "unposed_marker" else FiducialPose(Position(forward, left, .2), Quaternion(0., 0., 0., 1.))
    return [FiducialDetection(marker_id, config["default_marker_size_m"], CORNERS, pose, .5)], consecutive


def lidar_frame(track, pose, objects=()):
    positions = []
    for x, y, _ in track.cones():
        forward, left = body_point(x, y, pose)
        if math.hypot(forward, left) <= 4.:
            # Ideal three-return cone surfaces; no false clearance due to camera-only occlusion.
            for offset in (-.025, 0., .025):
                positions.append(Position(forward, left+offset, .1))
    for x, y, radius in objects:
        for angle in np.linspace(0., 2*math.pi, 24, endpoint=False):
            forward, left = body_point(x+radius*math.cos(angle), y+radius*math.sin(angle), pose)
            if math.hypot(forward, left) <= 4.:
                positions.append(Position(forward, left, .1))
    # Scan indices encode angular adjacency, not the order in which world cones were created.
    positions.sort(key=lambda p: math.atan2(p.y, p.x))
    points = [LidarCartesianPoint(p, i) for i, p in enumerate(positions)]
    return LidarCartesian(points, SensorAge(0., None), "base_link")


TURN_LOSS_DURATIONS_S = {"turn_blue_missing": 1.2, "turn_camera_gap": .8, "turn_corner_missing": 1.2}


def turn_occlusion(cones, track, pose, profile, now, started):
    """Apply a finite camera disturbance triggered by the actual plant's turn, not world-x windows."""
    if profile not in TURN_LOSS_DURATIONS_S:
        return cones, started, False
    if started is None and abs(pose[2]) >= math.radians(20):
        started = now
    active = started is not None and now < started+TURN_LOSS_DURATIONS_S[profile]-1e-8
    if not active:
        return cones, started, False
    if profile == "turn_camera_gap":
        return [], started, True
    if profile == "turn_blue_missing":
        return [c for c in cones if c.colour is not ConeColour.BLUE], started, True
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return [p for p in cones if max(abs(pose[0]+c*p.pos.x-s*p.pos.y),
                                    abs(pose[1]+s*p.pos.x+c*p.pos.y)) >= track.width_m/2+.6], started, True


def rollout(direction, profile="nominal", seed=23000, *, track=CrossIntersection(), duration_s=40., navigation=False,
            start_left_m=0., start_heading_deg=0., record_frames=False):
    params, detector = settings()
    policy, memory = make_policy(params, navigation), CarMemory()
    if params["action_policy"] != "mpc":
        raise ValueError("This diagnostic expects this branch's selected mpc action policy")
    marker_id = params["markers"][{"left": "turn_left_id", "right": "turn_right_id", "straight": "straight_id"}[direction]]
    if profile == "unmapped_marker":
        marker_id = next(i for i in range(49, -1, -1) if i not in MarkerConfig(**params["markers"]).commands)
    rng = np.random.default_rng(seed)
    pose, speed = np.array([-min(5.5, track.arm_length_m-.5) if navigation else -4., start_left_m, math.radians(start_heading_deg)]), 0.
    previous_curvature = 0.
    frames, trajectory, events, snapshots = [], [], [], []
    disturbance_started, disturbance_steps = None, 0
    contact_ids, outside, consecutive = set(), 0., 0
    objects = [(-1.5, .4, .12)] if profile == "blocked_turn" else []
    object_contacts = set()
    longest_empty_s, empty_since = 0., None
    recognised_at, disappeared_at, entered_at = None, None, None
    phases = []
    tick_ms = []
    candidate_exit, exit_steps, reached_exit, reason = None, 0, None, "timeout"
    for step in range(round(duration_s/PERIOD_S)):
        now = step*PERIOD_S
        cones = camera_frame(track, pose, rng, profile)
        cones, disturbance_started, disturbance_active = turn_occlusion(
            cones, track, pose, profile, now, disturbance_started)
        disturbance_steps += int(disturbance_active)
        if profile == "one_boundary_hidden" and 6. <= now < 11.:
            cones = [c for c in cones if c.colour is not ConeColour.BLUE]
        if (profile == "short_camera_loss" and 6. <= now < 6.6
                or profile == "long_camera_loss" and now >= 6.
                or profile == "terminal_lane_missing" and now >= 14.):
            cones = []
        if cones:
            empty_since = None
        else:
            empty_since = now if empty_since is None else empty_since
            longest_empty_s = max(longest_empty_s, now-empty_since+PERIOD_S)
        markers, consecutive = marker_frame(track, pose, marker_id, profile, step, detector, consecutive)
        frames.append((cones, markers, tuple(float(v) for v in pose), disturbance_active))
        if step >= LATENCY_STEPS:
            cones, markers, acquisition_pose, delivered_disturbance = frames[step-LATENCY_STEPS]
            age = SensorAge(LATENCY_STEPS*PERIOD_S, round((step-LATENCY_STEPS)*PERIOD_S*1e9))
            cone_batch = ConeBatch(cones, age, LATENCY_STEPS*PERIOD_S)
            fiducials = FiducialBatch(markers, age, detector["dictionary_name"], "oak_rgb_camera_optical_frame")
        else:
            cone_batch, fiducials = None, None
            acquisition_pose, delivered_disturbance = None, False
        gyro = AngularVelocity(0., 0., speed*previous_curvature, SensorAge(.4 if profile == "stale_gyro" and now >= 7. else 0., None))
        lidar = lidar_frame(track, pose, objects)
        if profile == "stale_lidar" and now >= 7.:
            lidar = LidarCartesian(lidar.points, SensorAge(.4, None), lidar.frame_id)
        observations = CarObservations(cone_batch, fiducials, None, lidar,
                           ObservedCarState(None, gyro, None, WheelSpeed(speed, SensorAge(.4 if profile == "stale_wheel" and now >= 7. else 0., None))),
                           PolicyState(PERIOD_S if step else 0., now, step == 0))
        started = time.perf_counter()
        actions = policy.step(observations, memory)
        tick_ms.append((time.perf_counter()-started)*1000)
        memory.step()
        intersection = memory.data.intersection
        command = (intersection.command if intersection is not None and intersection.command is not None
                   else memory.data.intersection_command)
        if intersection is not None and (not phases or phases[-1]["phase"] != intersection.phase):
            phases.append({"time_s": now, "phase": intersection.phase, "stop_reason": intersection.stop_reason})
        if policy.marker_command is not None:
            events.append({"time_s": now, "x_m": float(pose[0]), "command": policy.marker_command.name})
        if command is not None and recognised_at is None:
            recognised_at = now
        if recognised_at is not None and fiducials is not None and not fiducials.detections and disappeared_at is None:
            disappeared_at = now
        if pose[0] >= -track.width_m/2 and entered_at is None:
            entered_at = now
        if record_frames:
            lane, selected = policy.lanes, policy.selected_lane
            observed_points = ([] if lane is None or not lane.has_lane else
                [lane.centre_line.point_at(s) for s in np.linspace(0., min(4., max(0., lane.visible_until_m)), 50)])
            snapshots.append({"time_s": now, "pose": tuple(float(v) for v in pose),
                "acquisition_pose": acquisition_pose, "disturbance_active": delivered_disturbance,
                "raw_cones": [] if cone_batch is None else [(p.pos.x, p.pos.y, p.colour.name) for p in cone_batch],
                "checked_cones": [] if policy.cones is None else [(p.pos.x, p.pos.y, p.colour.name) for p in policy.cones],
                "lane_status": None if lane is None else lane.status.name, "observed_centre": observed_points,
                "selected_centre": [] if selected is None else selected.points,
                "selected_command": None if selected is None else selected.command.name,
                "phase": None if intersection is None else intersection.phase,
                "effective_left_colour": policy.lane_detector.config.left_colour,
                "drive": actions.drive_action, "steer": actions.steering_action,
                "speed_m_per_s": float(speed)})
        trajectory.append([now, *pose, speed, actions.drive_action, actions.steering_action,
                           -1 if command is None else command.value])
        curvature = CAR_20.curvature_per_m(actions.steering_action)
        previous_curvature = curvature
        for _ in range(round(PERIOD_S/SUBSTEP_S)):
            if actions.drive_action < 0.:
                # Illustrative active brake response, NOT measured stopping evidence.
                speed = max(0., speed-.8*SUBSTEP_S)
            else:
                speed = max(0., speed+CAR_20.acceleration_m_per_s2(actions.drive_action, speed, SUBSTEP_S)*SUBSTEP_S)
            distance = speed*SUBSTEP_S
            middle = pose[2]+distance*curvature/2
            pose += (distance*math.cos(middle), distance*math.sin(middle), distance*curvature)
            outside = max(outside, track.body_outside_m(pose))
            contact_ids.update(i for i, (x, y, _) in enumerate(track.cones()) if distance_to_body(x, y, *pose) < .05)
            object_contacts.update(i for i, (x, y, radius) in enumerate(objects) if distance_to_body(x, y, *pose) < radius)
        exit_now = track.exit_at(pose)
        exit_steps = exit_steps+1 if exit_now is not None and exit_now == candidate_exit else int(exit_now is not None)
        candidate_exit = exit_now
        if exit_steps >= 3:  # Sustain an aligned exit for .3 s; crossing the centre is not completion.
            reached_exit, reason = exit_now, "exit_reached"
            break
        if contact_ids or object_contacts or outside > 1e-6:
            reason = "cone_contact" if contact_ids else "obstacle_contact" if object_contacts else "body_outside_road"
            break
        if memory.data.mpc_stop is not None and speed < .02:
            reason = "policy_stop"
            break
    # Include the actual final plant pose rather than leaving the last substep unrecorded.
    trajectory.append([(step+1)*PERIOD_S, *pose, speed, actions.drive_action, actions.steering_action,
                       -1 if command is None else command.value])
    remembered = command
    correct_command = any(e["command"] == DIRECTIONS[direction] for e in events)
    completed = memory.data.intersection is not None and memory.data.intersection.phase == "complete"
    result = {"direction": direction, "marker_id": marker_id, "profile": profile, "seed": seed,
            "track": asdict(track), "start_left_m": start_left_m, "start_heading_deg": start_heading_deg,
            "navigation_enabled": navigation,
            "metrics": {"command_recognised": correct_command, "recognised_at_s": recognised_at,
                        "marker_disappeared_at_s": disappeared_at, "intersection_entered_at_s": entered_at,
                        "command_before_intersection": correct_command and recognised_at is not None
                            and (entered_at is None or recognised_at < entered_at),
                        "remembered_command": None if remembered is None else remembered.name,
                        "actual_exit": reached_exit, "requested_exit_reached": reached_exit == direction,
                        "wrong_exit": reached_exit is not None and reached_exit != direction,
                        "cone_contacts": len(contact_ids), "max_body_outside_m": outside,
                        "obstacle_contacts": len(object_contacts), "max_camera_empty_s": longest_empty_s,
                        "reason": reason, "policy_stop": None if memory.data.mpc_stop is None else memory.data.mpc_stop.name,
                        "navigation_pass": correct_command and reached_exit == direction
                            and not contact_ids and not object_contacts and outside <= 1e-6 and (completed or not navigation),
                        "handoff_completed": completed,
                        "command_consumed": navigation and correct_command and memory.data.intersection_command is None,
                        "final_speed_m_per_s": speed},
            "timing": {"tick_p95_ms": float(np.percentile(tick_ms, 95)), "tick_max_ms": float(max(tick_ms))},
            "events": events, "phases": phases, "trajectory": np.array(trajectory)}
    result["metrics"]["intersection_stop_reason"] = "" if intersection is None else intersection.stop_reason
    result["metrics"]["available_exits"] = [] if intersection is None else [c.name for c in intersection.available_exits]
    result["metrics"]["closed_exits"] = [] if intersection is None else [c.name for c in intersection.closed_exits]
    if profile in TURN_LOSS_DURATIONS_S:
        result["disturbance"] = {"trigger_heading_deg": 20., "started_at_s": disturbance_started,
                                  "duration_s": TURN_LOSS_DURATIONS_S[profile],
                                  "acquisition_steps_active": disturbance_steps}
    if record_frames:
        result["frames"] = snapshots
    return result


def source_hashes():
    paths = [*sorted((ROOT/"policy").rglob("*.py")), ROOT/"config/ai4r_policy.yaml",
             ROOT/"config/aruco_detector.yaml", Path(__file__), ROOT/"tests/policy/car_models.py",
             ROOT/"tests/policy/stage1_scenarios.py"]
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


def plot(results, track, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    figure, axes = plt.subplots(2, 3, figsize=(13, 9), sharex=True, sharey=True)
    for (profile, direction), axis in zip(((p, d) for p in ("nominal", "noisy_cones") for d in DIRECTIONS), axes.flat):
        half, end = track.width_m/2, track.arm_length_m
        for rectangle in track.road_rectangles():
            axis.add_patch(Rectangle(rectangle[:2], *rectangle[2:], facecolor="#edf0f4", zorder=0))
        for x, y, colour in track.cones():
            axis.scatter(x, y, s=12, color="royalblue" if colour is ConeColour.BLUE else "goldenrod")
        axis.scatter(track.marker_x_m, track.marker_y_m, color="purple", marker="s", s=60, label="ArUco sign")
        result = next(r for r in results if r["direction"] == direction and r["profile"] == profile)
        t, m = result["trajectory"], result["metrics"]
        axis.plot(t[:, 1], t[:, 2], color="darkorange", label="actual path")
        axis.scatter(t[-1, 1], t[-1, 2], color="darkorange", marker="x")
        footprint = body_corners(*t[-1, 1:4])
        axis.plot(*np.vstack((footprint, footprint[0])).T, color="darkorange", linewidth=1)
        target = {"left": (0., 3.), "right": (0., -3.), "straight": (2.5, .15)}[direction]
        axis.annotate("requested exit", target, ha="center", fontsize=9, color="darkgreen")
        axis.set_title(f"{profile} · ID {result['marker_id']}: {direction}\nremembered {m['remembered_command']}\nexit {m['actual_exit'] or 'none'} · {m['reason']}", fontsize=10)
        axis.set_aspect("equal")
        axis.set_xlim(-max(4.5, track.arm_length_m*.75), max(4., track.exit_gate_m+1.))
        axis.set_ylim(-max(3.5, track.exit_gate_m+.8), max(3.5, track.exit_gate_m+.8))
        axis.set_xlabel("metres")
        axis.legend(loc="lower left", fontsize=8)
    for axis in axes[:, 0]:
        axis.set_ylabel("metres")
    figure.suptitle("ArUco four-way intersection · decoded-marker / kinematic simulation")
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"build/aruco-intersection")
    parser.add_argument("--navigate", action="store_true", help="Enable the local intersection prototype with a 2.4 m cross")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    hashes = source_hashes()
    track = (CrossIntersection(width_m=2.4, arm_length_m=8., cone_spacing_m=.2,
                              marker_x_m=-3.8, marker_y_m=-.7, exit_gate_m=3.6)
             if args.navigate else CrossIntersection())
    results, comparisons = [], []
    profiles = (("nominal", "noisy_cones", "marker_disappears", "brief_cone_loss", "one_boundary_hidden", "short_camera_loss")
                if args.navigate else ("nominal", "noisy_cones", "marker_disappears", "brief_cone_loss"))
    for i, profile in enumerate(profiles):
        matched = [rollout(direction, profile, 23000+i, track=track, navigation=args.navigate) for direction in DIRECTIONS]
        results.extend(matched)
        controls = [r["trajectory"][:, :7] for r in matched]  # Exclude the remembered command column.
        identical = all(a.shape == controls[0].shape and np.allclose(a, controls[0], atol=1e-12, rtol=0.)
                        for a in controls[1:])
        comparisons.append({"profile": profile, "same_trajectory_and_actions_for_all_ids": identical})
        for result in matched:
            print(result["profile"], result["direction"], result["metrics"], flush=True)
    negatives = (("missing_marker", "unmapped_marker", "unposed_marker", "single_marker_frame", "long_camera_loss",
                  "stale_gyro", "stale_wheel", "stale_lidar", "terminal_lane_missing", "blocked_turn")
                 if args.navigate else ("missing_marker", "unmapped_marker", "unposed_marker", "single_marker_frame"))
    for profile in negatives:
        result = rollout("left" if profile == "blocked_turn" else "straight", profile, track=track, navigation=args.navigate)
        results.append(result)
        print(profile, result["metrics"], flush=True)
    variations, spacing_probes = [], []
    if args.navigate:
        for i, width in enumerate((2.2, 2.6)):
            varied = CrossIntersection(width_m=width, arm_length_m=8., cone_spacing_m=.2,
                                      marker_x_m=-3.8, marker_y_m=-.7, exit_gate_m=3.6)
            for direction in DIRECTIONS:
                result = rollout(direction, "noisy_cones", 24000+i, track=varied, navigation=True,
                                 start_left_m=.04*(-1)**i, start_heading_deg=2.*(-1)**i)
                variations.append({"width_m": width, "run": {k: v for k, v in result.items() if k != "trajectory"}})
                np.savetxt(args.output/f"width-{width}-{direction}.csv", result["trajectory"], delimiter=",",
                           header="time_s,x_m,y_m,heading_rad,speed_m_per_s,drive_action,steering_action,remembered_command", comments="")
                print("width", width, direction, result["metrics"], flush=True)
        for spacing in (.4, .5):
            varied = CrossIntersection(width_m=2.4, arm_length_m=8., cone_spacing_m=spacing,
                                      marker_x_m=-3.8, marker_y_m=-.7, exit_gate_m=3.6)
            for direction in DIRECTIONS:
                result = rollout(direction, track=varied, navigation=True)
                spacing_probes.append({k: v for k, v in result.items() if k != "trajectory"})
                np.savetxt(args.output/f"spacing-{spacing}-{direction}.csv", result["trajectory"], delimiter=",",
                           header="time_s,x_m,y_m,heading_rad,speed_m_per_s,drive_action,steering_action,remembered_command", comments="")
                print("spacing", spacing, direction, result["metrics"], flush=True)
    params, detector = settings()
    report = {"branch": subprocess.check_output(["git", "branch", "--show-current"], text=True).strip(),
              "revision": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
              "source_sha256": hashes, "policy_parameters": params, "detector_parameters": detector,
              "track": asdict(track), "car_model": asdict(CAR_20), "navigation_enabled": args.navigate,
              "effective_intersection_parameters": dict(params.get("intersection_navigator", {}), enabled=args.navigate),
              "scope": "MovementPolicy -> CarController -> repository CAR_20 response, 10Hz policy / "
              "100Hz no-slip plant. 86deg/0.4..2.5m camera, 0.2s latency. Decoded marker poses with a simplified "
              "consecutive-image publication gate, not ArUco pixel detection. Ideal current three-point cone lidar "
              "within4m. Synthetic negative-drive deceleration0.8m/s^2, no reverse. No ROS watchdog/node state "
              "machine, scan occlusion, actuator deadtime, tyre dynamics or physical qualification. CAR_20 time "
              "constants borrowed from car22. Geometry/orthogonal equal-width cross are explicit synthetic assumptions. "
              "Navigator enabled only by local --navigate; node sensor/empty-timeout admission is not exercised here. "
              "Physical marker board/stand is not a collision obstacle. Goal gate means navigation while moving, "
              "not stopping. Only marker ID differs within each comparison; no oracle route, learned model or training.",
              "comparisons": comparisons,
              "width_probes": variations,
              "spacing_probes": spacing_probes,
              "runs": [{k: v for k, v in r.items() if k != "trajectory"} for r in results]}
    for result in results:
        np.savetxt(args.output/f"{result['profile']}-{result['direction']}.csv", result["trajectory"], delimiter=",",
                   header="time_s,x_m,y_m,heading_rad,speed_m_per_s,drive_action,steering_action,remembered_command", comments="")
    plot(results, track, args.output/"intersection.png")
    if hashes != source_hashes():
        raise ValueError("Executed sources changed during evaluation; report not written")
    (args.output/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    print("Saved", args.output, flush=True)


if __name__ == "__main__":
    main()
