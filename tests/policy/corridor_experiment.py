"""Offline squared-U investigation; never changes the deployed policy or its steering calibration.

python -m tests.policy.corridor_experiment --output build/corridor-experiment
The sharper-steering profile is hypothetical. Original car-20 limits are kept as the infeasible control case.
"""
from dataclasses import replace
import argparse
import hashlib
import json
import itertools
from pathlib import Path

from policy.action_policy.corridor_experiment import OfflineCorridorPolicy
from policy.action_policy.mpc import MpcConfig
from policy.action_policy.policy import MpcPolicy
from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones
from policy.lane_detection.corridor import CorridorConfig, left_uturn_span_check
from policy.lane_detection.lane_detection import LaneDetectionConfig, LaneDetector
from policy.lane_detection.temporal import detect_temporal_lane
from policy.memory import CarMemory
from scripts.mcap_export import _finite
from tests.policy.angular_lanes import AngularUTurn
from tests.policy.curved_lanes import CurveCase, run_curve
from tests.policy.real_car_measurements import LAB_CAR


def run(output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(1, 2, figsize=(12, 5))
    result = {}
    for axis, (name, left, right) in zip(axes, (("car20_measured_limits", 0.48, 0.92),
                                              ("hypothetical_tighter_steering", 1.2, 1.2))):
        result[name] = {"left_limit_per_m": left, "right_limit_per_m": right,
                        "necessary_feasibility": left_uturn_span_check(left, 3.8), "cases": {}}
        for seed, occlusion, speed in itertools.product(range(5), (False, True), (0.4, 0.5)):
                label = f"speed{speed}_seed{seed}" + ("_inner_occlusion" if occlusion else "")
                case = CurveCase(AngularUTurn(), max_duration_s=45, seed=seed, speed_m_per_s=speed,
                                 inner_occlusion_s=(8.0, 8.4) if occlusion else None,
                                 car=replace(LAB_CAR, camera_yaw_deg=0.0,
                                             max_left_curvature_per_m=left, max_right_curvature_per_m=right))
                detector = LaneDetector(LaneDetectionConfig(memory_enabled=True, continuity_enabled=True,
                                                           max_curvature_per_m=2.0))
                memory = CarMemory()

                def lanes(observations):
                    filtered = replace(observations, cones=filter_cones(
                        observations.cones, ConeFilterConfig(check_straight_rows=False)))
                    return detect_temporal_lane(filtered, memory, detector)[0]

                matched_detector = LaneDetector(LaneDetectionConfig(memory_enabled=True, max_curvature_per_m=2.0,
                                                memory_max_age_s=2.0, memory_max_distance_m=1.0))
                matched_memory = CarMemory()

                def matched_lanes(observations):
                    filtered = replace(observations, cones=filter_cones(
                        observations.cones, ConeFilterConfig(check_straight_rows=False)))
                    return detect_temporal_lane(filtered, matched_memory, matched_detector)[0]

                controllers = {
                    "arc_with_memory_constraints": (MpcPolicy(MpcConfig(target_speed_m_per_s=speed, max_curvature_per_m=left,
                        max_right_curvature_per_m=right, run_duration_s=0.0)), lanes),
                    "arc_with_matched_memory": (MpcPolicy(MpcConfig(target_speed_m_per_s=speed,
                        max_curvature_per_m=left, max_right_curvature_per_m=right, run_duration_s=0.0)), matched_lanes),
                    "connected_midpoint_corridor": (OfflineCorridorPolicy(left, right, CarMemory(),
                        CorridorConfig(lookahead_m=1.2), speed), None),
                }
                result[name]["cases"][label] = {}
                for mode, (controller, observer) in controllers.items():
                    trajectory = run_curve(case, controller, observer)
                    result[name]["cases"][label][mode] = {**trajectory.metrics, "lane_kept": trajectory.lane_kept}
                    if seed == 0 and not occlusion and speed == 0.4:
                        axis.plot(trajectory.x, trajectory.y, label=mode.replace("_", " "))
        lane = AngularUTurn()
        for colour, _, x, y in lane.cones():
            axis.plot(x, y, ".", color="royalblue" if colour.name == "BLUE" else "goldenrod", markersize=4)
        axis.set_title(name.replace("_", " ") + f"\nleft/right limits {left}/{right} 1/m")
        axis.set_aspect("equal")
        axis.set_xlabel("world x (m)")
        axis.set_ylabel("world y (m)")
        axis.legend(fontsize=7)
    figure.suptitle("Same squared-U cones; measured limits vs explicitly hypothetical steering")
    figure.tight_layout()
    figure.savefig(output / "comparison.png", dpi=150)
    plt.close(figure)
    sources = ("policy/lane_detection/corridor.py", "policy/action_policy/corridor_experiment.py",
               "policy/lane_detection/temporal.py", "policy/memory.py", "tests/policy/angular_lanes.py",
               "tests/policy/curved_lanes.py", __file__)
    result["source_sha256"] = {str(p): hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in sources}
    result["experiment_settings"] = {
        "layout": "unchanged AngularUTurn: leg=3m, cross=3m, width=.8m, placement seed=17",
        "noise_seeds": list(range(5)), "speeds_m_per_s": [0.4, 0.5], "occlusion_s": [8.0, 8.4],
        "corridor_lookahead_m": 1.2, "corridor_memory_max_age_s": 2.0, "corridor_memory_max_distance_m": 1.0,
        "smoothing_fraction": 0.3, "curvature_change_limit_per_m_s": 4.0,
        "limitations": "Offline follower, cone-only sampled rollout, no runtime lidar/vehicle safety integration.",
    }
    (output / "summary.json").write_text(json.dumps(_finite(result), indent=2, allow_nan=False) + "\n")
    print(output / "summary.json")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("build/corridor-experiment"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    run(args.output)
