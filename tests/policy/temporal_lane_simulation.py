"""Compare opt-in temporal geometry locally, without ROS/hardware.

python -m tests.policy.temporal_lane_simulation --output build/temporal-lanes
Optional --recording PATH replays a physical-run observations log with its saved settings.
Closed-loop cases use the existing kinematic curved-lane harness (measured car-20 response, no slip).
Recorded replay is an open-loop algorithm comparison, not a simulated outcome of changed actions.
"""
import argparse
from dataclasses import replace
import json
import hashlib
import importlib.metadata
from pathlib import Path
import sys
import time

from policy.action_policy.mpc import MpcConfig
from policy.action_policy.policy import MpcPolicy
from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones
from policy.lane_detection.lane_detection import LaneDetectionConfig, LaneDetector
from policy.lane_detection.temporal import detect_temporal_lane
from policy.memory import CarMemory
from tests.policy.curved_lanes import CurveCase, CurvedLane, run_curve
from tests.policy.angular_lanes import AngularUTurn
from scripts import mcap_export


MODES = {"baseline": (False, False), "memory": (True, False), "memory_and_constraints": (True, True),
         "single_edge_hold": (False, False), "turn_slowdown": (False, False)}


def closed_loop(output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cases = {
        "straight": CurveCase(CurvedLane(pieces=(("straight", 4.0),))),
        "left_90": CurveCase(CurvedLane(pieces=(("straight", 1.5), ("arc", 1 / 2.5, 90), ("straight", 1.5)))),
        "uturn": CurveCase(CurvedLane(pieces=(("straight", 1.0), ("arc", 1 / 2.5, 180), ("straight", 1.0)))),
        "uturn_inner_occlusion": CurveCase(CurvedLane(
            pieces=(("straight", 1.0), ("arc", 1 / 2.5, 180), ("straight", 1.0))), inner_occlusion_s=(8.0, 8.4)),
        "angular_uturn": CurveCase(AngularUTurn()),
        "angular_uturn_inner_occlusion": CurveCase(AngularUTurn(), inner_occlusion_s=(4.0, 4.4)),
    }
    results = {}
    figure, axes = plt.subplots(3, 2, figsize=(11, 14))
    for (name, case), axis in zip(cases.items(), axes.flat):
        for colour, _, x, y in case.lane.cones():
            axis.plot(x, y, ".", color="royalblue" if colour.name == "BLUE" else "goldenrod", markersize=3)
        results[name] = {}
        line = case.lane.centre_line()
        axis.plot(line["x"], line["y"], ":", color="grey", alpha=0.5, label="scoring reference")
        for mode, (remember, constrain) in MODES.items():
            detector = LaneDetector(LaneDetectionConfig(max_curvature_per_m=2.0,
                                                       memory_enabled=remember, continuity_enabled=constrain,
                                                       single_edge_hold_enabled=mode in ("single_edge_hold", "turn_slowdown")))
            memory = CarMemory()
            stats = {"remembered_steps": 0, "rejected_steps": 0, "no_lane_steps": 0, "held_steps": 0}

            def lanes_from(observations):
                filtered = replace(observations, cones=filter_cones(
                    observations.cones, ConeFilterConfig(check_straight_rows=False)))
                lanes, _ = detect_temporal_lane(filtered, memory, detector)
                state = memory.data.lane_tracking
                stats["remembered_steps"] += state.remembered_count > 0
                stats["held_steps"] += state.holding_single_edge
                stats["rejected_steps"] += state.rejected
                stats["no_lane_steps"] += lanes is None or not lanes.has_lane
                memory.step()
                return lanes

            controller = MpcPolicy(MpcConfig(target_speed_m_per_s=case.speed_m_per_s, run_duration_s=0.0,
                                             turn_slowdown_enabled=mode == "turn_slowdown"))
            if mode in ("single_edge_hold", "turn_slowdown"):
                original_run = controller.run_policy

                def run_with_support(observations, lanes, obstacle_distance_m=None):
                    return original_run(observations, lanes, obstacle_distance_m,
                                        lane_support_m=lanes.visible_until_m
                                        if memory.data.lane_tracking.holding_single_edge else None)

                controller.run_policy = run_with_support
            result = run_curve(case, controller, lanes_from)
            results[name][mode] = {**result.metrics, **stats}
            axis.plot(result.x, result.y, label=mode.replace("_", " "))
        axis.set_title(name.replace("_", " "))
        axis.set_aspect("equal")
        axis.set_xlabel("world x (m)")
        axis.set_ylabel("world y (m)")
        axis.legend(fontsize=7)
    figure.suptitle("Local kinematic simulation: experimental temporal lane geometry")
    figure.tight_layout()
    figure.savefig(output / "trajectories.png", dpi=150)
    plt.close(figure)
    return results


def recorded(path, output):
    steps = mcap_export.read_data_log(path)
    settings = mcap_export.read_settings(path.with_name(path.name.replace("_observations.json", "_config.json")))
    results = {}
    for mode, (remember, constrain) in MODES.items():
        policy = mcap_export.build_policy({**settings, "lane_detection.memory_enabled": remember,
                                         "lane_detection.continuity_enabled": constrain,
                                         "lane_detection.single_edge_hold_enabled": mode in ("single_edge_hold", "turn_slowdown"),
                                         "mpc.turn_slowdown_enabled": mode == "turn_slowdown"})
        memory = CarMemory()
        stats = {"steps": len(steps), "remembered_steps": 0, "rejected_steps": 0, "no_lane_steps": 0,
                 "action_mismatches": 0, "max_step_ms": 0.0, "max_remembered_cones": 0,
                 "gyro_available_steps": sum(s.observations.car.ang_velocity is not None for s in steps)}
        for step in steps:
            start = time.perf_counter()
            actions = policy.step(step.observations, memory)
            stats["max_step_ms"] = max(stats["max_step_ms"], 1000 * (time.perf_counter() - start))
            state = memory.data.lane_tracking
            stats["remembered_steps"] += state.remembered_count > 0
            stats["max_remembered_cones"] = max(stats["max_remembered_cones"], state.remembered_count)
            stats["rejected_steps"] += state.rejected
            stats["no_lane_steps"] += policy.lanes is None or not policy.lanes.has_lane
            stats["action_mismatches"] += step.actions is not None and not mcap_export._same_actions(actions, step.actions)
            memory.step()
        # Produce directly inspectable MCAPs for the same settings, through the standard exporter.
        mcap_export.export(steps, mcap_export.build_policy({**settings, "lane_detection.memory_enabled": remember,
                                                         "lane_detection.continuity_enabled": constrain,
                                         "lane_detection.single_edge_hold_enabled": mode in ("single_edge_hold", "turn_slowdown"),
                                         "mpc.turn_slowdown_enabled": mode == "turn_slowdown"}),
                           output / f"{path.stem}_{mode}.mcap")
        results[mode] = stats
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("build/temporal-lanes"))
    parser.add_argument("--recording", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = {"closed_loop": closed_loop(args.output)}
    if args.recording:
        results["recorded_open_loop"] = recorded(args.recording, args.output)
    sources = ("policy/lane_detection/temporal.py", "policy/lane_detection/lane_detection.py", "policy/memory.py",
               "policy/policy_runner.py", "tests/policy/curved_lanes.py", "tests/policy/angular_lanes.py", __file__)
    results["evidence"] = {
        "python": sys.version,
        "dependencies": {name: importlib.metadata.version(name) for name in ("numpy", "scipy", "foxglove-sdk")},
        "source_sha256": {str(path): hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in sources},
        "seed": 0, "speed_m_per_s": 0.5, "uturn_radius_m": 2.5,
        "angular_uturn": {"layout": "two three-segment polygonal boundaries; squared U",
                          "leg_length_m": 3.0, "cross_length_m": 3.0, "width_m": 0.8,
                          "spacing_m": 0.5, "spacing_variation": "0.8..1.2 times nominal",
                          "placement_jitter_m": 0.025, "placement_seed": 17},
        "limitations": "Kinematic no-slip model; measured-speed/gyro simulation; recorded replay is open-loop.",
    }
    target = args.output / "summary.json"
    target.write_text(json.dumps(mcap_export._finite(results), indent=2, allow_nan=False) + "\n")
    print(target)


if __name__ == "__main__":
    main()
