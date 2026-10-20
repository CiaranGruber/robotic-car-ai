"""Exercise the selected ROS policy's pure-Python path, guards and normalized control in the U-turn harness.

The ideal-curvature mode tests runtime geometry/guards; it does NOT simulate normalized actuator calibration.
The mapped-steering mode additionally inverts the configured effective steering map, including explicit
endpoint scaling if selected. It tests normalized conversion/clipping, not an independently measured plant.
Drive effort is checked but the generic harness models requested speed, not the physical ESC/encoder loop.
Braking is an explicit assumption: .2s dead time, then linear deceleration; stopping distance/speed is the
configured .6s, so stopping takes 1s. Negative normalized drive activates it. This is not a measured brake profile.
No code here is uploaded. The camera stays fixed, and lidar includes every cone surface within 4m.
"""
import argparse
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path

import yaml

from policy.action_policy.corridor import CorridorControlConfig
from policy.action_policy.mpc import MpcConfig
from policy.action_policy.avoidance import AvoidanceConfig
from policy.cone_filter.cone_filter import ConeFilterConfig
from policy.control.actions import DriveCommand
from policy.control.control import ControlConfig
from policy.input_output import LidarCartesian, LidarCartesianPoint, Position, SensorAge
from policy.lane_detection.lane_detection import LaneDetectionConfig
from policy.memory import CarMemory
from policy.obstacle_detection.obstacle_detection import ObstacleDetectionConfig
from policy.policy_runner import MovementPolicy
from tests.policy.angular_lanes import AngularUTurn
from tests.policy.car21_uturn_simulation import case
from tests.policy.curved_lanes import run_curve


def cone_lidar(cones, pose, time_s):
    x, y, h = pose
    c, s = math.cos(h), math.sin(h)
    points = []
    for _, _, cx, cy in cones:
        bx, by = c*(cx-x)+s*(cy-y), -s*(cx-x)+c*(cy-y)
        distance = math.hypot(bx, by)
        if not .05 < distance <= 4.:
            continue
        ux, uy = bx/distance, by/distance
        for offset in (-.02, 0., .02):
            radius = math.sqrt(.05**2-offset**2)
            points.append(LidarCartesianPoint(Position(bx-radius*ux-offset*uy,
                                                        by-radius*uy+offset*ux, .12), len(points)))
    return LidarCartesian(points, SensorAge(0., round((100+time_s)*1e9)), "base_link")


class RuntimeAdapter:
    def __init__(self, settings, mapped=False):
        self.control = ControlConfig(**settings["control"])
        self.mapped = mapped
        self.memory = CarMemory()
        self.started = False
        self.movement = MovementPolicy(
            MpcConfig(**settings["mpc"]), ConeFilterConfig(**settings["cone_filter"]), self.control,
            action_policy="corridor", corridor_config=CorridorControlConfig(**settings["corridor"]),
            lane_detection_config=LaneDetectionConfig(**settings["lane_detection"]),
            obstacle_detection_config=ObstacleDetectionConfig(**settings["obstacle_detection"]),
            avoidance_config=AvoidanceConfig(**settings["avoidance"]))
        self.control = self.movement.car_controller.config

    def run_policy(self, observations, lanes=None):
        # The ROS node refuses entry into policy-actions until every required stream is present.
        # The generic harness calls from t=0, before the camera delay queue produces its first frame.
        if observations.cones is None and not self.started:
            return [DriveCommand(0., 0.)]
        if not self.started:
            observations = replace(observations, policy=replace(observations.policy,
                                   is_first_policy_step=True, dt=0.))
            self.started = True
        actions = self.movement.step(observations, self.memory)
        assert abs(actions.steering_action) <= 1 and abs(actions.drive_action) <= 1 and actions.camera_pan_action == 0
        self.memory.step()
        command = self.movement.corridor_policy.last_command
        command = replace(command, brake=actions.drive_action < 0.)
        if not self.mapped:
            return [command]
        offset = actions.steering_action-self.control.steering_centre_action
        left = offset/self.control.steering_left_action_per_curvature
        k = left if left >= 0 else offset/self.control.steering_right_action_per_curvature
        return [DriveCommand(command.speed_m_per_s, k, command.brake)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("build/corridor-runtime-validation"))
    parser.add_argument("--seed-start", type=int, default=201)
    parser.add_argument("--count", type=int, default=5)
    parser.add_argument("--placement-seed", type=int, default=17)
    parser.add_argument("--mapped-only", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    settings = yaml.safe_load(Path("config/ai4r_policy.yaml").read_text())["/**/ai4r_policy"]["ros__parameters"]
    sources = ("config/ai4r_policy.yaml", "policy/policy_runner.py", "policy/action_policy/corridor.py",
               "policy/action_policy/corridor_experiment.py", "policy/lane_detection/corridor.py",
               "policy/memory.py", "tests/policy/curved_lanes.py", __file__)
    hashes = {p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in sources}
    report = {"source_sha256": hashes, "cases": {}, "limitations": __doc__}
    examples = {}
    modes = [("runtime_configured_steering_map", True)]
    if not args.mapped_only: modes.insert(0, ("runtime_ideal_curvature", False))
    for mode, mapped in modes:
        for seed in range(args.seed_start, args.seed_start+args.count):
            for occluded in (False, True):
                policy = RuntimeAdapter(settings, mapped)
                scenario = replace(case(seed=seed, occlusion=occluded),
                                   lane=AngularUTurn(width_m=1., seed=args.placement_seed))
                r = run_curve(scenario, policy, lidar_from=cone_lidar,
                              braking_distance_per_speed_s=settings["mpc"]["obstacle_stop_distance_per_speed_s"])
                name = f"{mode}/seed{seed}"+("_occluded" if occluded else "")
                report["cases"][name] = {**r.metrics, "lane_kept": r.lane_kept,
                                         "stop_reason": policy.movement.corridor_policy.stop_reason}
                if seed == args.seed_start and not occluded: examples[mode] = r
                print(name, r.lane_kept, r.metrics, report["cases"][name]["stop_reason"], flush=True)
                (args.output/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    assert hashes == {p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in sources}
    report["summary"] = {mode: {"passed": sum(v["lane_kept"] for n, v in report["cases"].items() if n.startswith(mode+"/")),
                                "total": sum(n.startswith(mode+"/") for n in report["cases"])} for mode in examples}
    (args.output/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, len(modes), figsize=(5.5*len(modes), 5), squeeze=False)
    lane = AngularUTurn(width_m=1., seed=args.placement_seed)
    for axis, (mode, r) in zip(axes[0], examples.items()):
        axis.fill(*zip(*lane._corridor_polygon()), color="lightgrey", alpha=.3)
        for colour, _, x, y in lane.cones(): axis.plot(x, y, ".", color="royalblue" if colour.name == "BLUE" else "goldenrod")
        axis.plot(r.x, r.y, color="seagreen")
        axis.set_title(mode.replace("_", " "))
        axis.set_aspect("equal")
    fig.suptitle("Selected runtime corridor + cone-surface lidar; fixed camera, provisional geometry")
    fig.tight_layout()
    fig.savefig(args.output/"trajectories.png", dpi=150)
    print(json.dumps(report["summary"], indent=2), flush=True)


if __name__ == "__main__": main()
