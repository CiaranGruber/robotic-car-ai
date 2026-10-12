"""Repeatable, offline Stage 1 MPC comparison (Python 3.12+).

Run: .venv/bin/python tests/policy/evaluate_mpc.py --output build/mpc-evaluation

This is a kinematic screening model, NOT DREAMGym or a calibrated vehicle.
The controller receives delayed cone detections only. Ground truth is used by
the sensor model and scorer. Speed/curvature commands are applied ideally;
CarControl, ROS watchdogs, tyre dynamics and actuator delay are not simulated.
Candidate selection uses tuning cases only, then evaluates the frozen choice
on separate validation cases/seeds. Never retune from validation results.
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import numpy as np
import scipy

from policy.action_policy.mpc import MpcConfig
from policy.action_policy.policy import MpcPolicy
from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones
from policy.input_output import (
    CarObservations, ConeBatch, ConeColour, ConeDetection, ObservedCarState,
    PolicyState, Position, SensorAge, WheelSpeed,
)

DT = 0.1
LANE_LENGTH = 6.0
LANE_HALF_WIDTH = 0.5
# Illustrative circular footprint, not measured Traxxas geometry.
CAR_RADIUS = 0.18
CONE_RADIUS = 0.05
WORLD_CONES = [(x, side * LANE_HALF_WIDTH, colour)
               for x in np.arange(0.0, LANE_LENGTH + 0.01, 0.5)
               for side, colour in ((1, ConeColour.YELLOW), (-1, ConeColour.BLUE))]


@dataclasses.dataclass(frozen=True)
class Case:
    name: str
    offset: float = 0.0
    heading: float = 0.0
    noise: float = 0.0
    delay_steps: int = 0
    miss_probability: float = 0.0
    empty_every: int = 0
    speed: float = 1.0
    speed_response_s: float = 0.0
    """Optional first-order speed response time; zero retains the ideal model."""
    speed_gain: float = 1.0
    """Achieved steady speed / command, for sensitivity experiments, not calibration."""
    hide_from_x_m: float | None = None
    """Hide cones at/after a world position, modelling persistent occlusion."""
    stream_loss_step: int | None = None
    """Stop delivering detections from this update onwards."""
    occlusion_start_step: int | None = None
    occlusion_end_step: int | None = None


TUNING = [Case("T1-centred"), Case("T2-offset", offset=0.15),
          Case("T3-heading", heading=math.radians(10)),
          Case("T4-disturbed", offset=0.15, noise=0.01, delay_steps=2,
               miss_probability=0.1, empty_every=3)]
# Public scenarios, held-out random seeds: these are validation, not blinded cases.
VALIDATION = [Case("V1-right-delay", offset=-0.2, heading=math.radians(-5),
                   noise=0.015, delay_steps=2, miss_probability=0.15, speed=0.7),
              Case("V2-heading-dropout", offset=0.1, heading=math.radians(15),
                   noise=0.01, delay_steps=1, miss_probability=0.2, empty_every=4)]
CANDIDATES = {
    "baseline": MpcConfig(),
    "longer": MpcConfig(horizon_steps=20),
    "smooth": MpcConfig(horizon_steps=20, curvature_change_weight=4.0),
}


def detect(pose, case, rng, step):
    """80 degree view, 4 m radial range, noise applied after visibility."""
    if case.empty_every and (step + 1) % case.empty_every == 0:
        return []
    x, y, heading = pose
    detections = []
    for cx, cy, colour in WORLD_CONES:
        # Draw for every world cone so candidates get matching disturbances
        # even when their different poses change which cones are visible.
        missed = rng.random() < case.miss_probability
        noise = rng.normal(0.0, case.noise, 2)
        if case.hide_from_x_m is not None and cx >= case.hide_from_x_m:
            continue
        if (case.occlusion_start_step is not None and case.occlusion_end_step is not None
                and case.occlusion_start_step <= step < case.occlusion_end_step):
            continue
        dx, dy = cx - x, cy - y
        forward = math.cos(heading) * dx + math.sin(heading) * dy
        left = -math.sin(heading) * dx + math.cos(heading) * dy
        if (forward <= 0 or math.hypot(forward, left) > 4.0
                or abs(math.atan2(left, forward)) > math.radians(40)):
            continue
        if missed:
            continue
        detections.append(ConeDetection(Position(forward + noise[0], left + noise[1], 0.1), colour, 0.9))
    return detections


def rollout(config, case, seed, *, measured_speed=False, policy_factory=MpcPolicy):
    rng = np.random.default_rng(seed)
    policy = policy_factory(dataclasses.replace(config, target_speed_m_per_s=case.speed))
    pose = np.array([0.0, case.offset, case.heading])
    frames, trace, times = [], [], []
    clearance = math.inf
    max_lateral = abs(case.offset)
    actual_speed = 0.0
    for step in range(200):
        frames.append(detect(pose, case, rng, step))
        # No fictitious pre-start observations: wait for the first delayed frame.
        frame = frames[step - case.delay_steps] if step >= case.delay_steps else []
        batch = ConeBatch(frame, SensorAge(case.delay_steps * DT, int((step-case.delay_steps) * DT * 1e9)),
                          case.delay_steps * DT)
        if case.stream_loss_step is not None and step >= case.stream_loss_step:
            batch = None
        started = time.perf_counter()
        filtered = filter_cones(batch, ConeFilterConfig())
        observations = CarObservations(
            cones=filtered, fiducials=None, lidar_scan=None, lidar_cartesian=None,
            car=ObservedCarState(None, None, None,
                                WheelSpeed(actual_speed, SensorAge(0.0, None)) if measured_speed else None),
            policy=PolicyState(0.0 if step == 0 else DT, step * DT, step == 0))
        command = policy.run_policy(observations, None)[0]
        times.append(time.perf_counter() - started)
        trace.append([step * DT, *pose, command.speed_m_per_s, command.curvature_per_m, actual_speed])
        # 100 Hz integration; sample collision/containment throughout each update.
        for _ in range(10):
            target = command.speed_m_per_s * case.speed_gain
            if case.speed_response_s > 0.0:
                actual_speed += (target - actual_speed) * (1.0 - math.exp(-DT / 10 / case.speed_response_s))
            else:
                actual_speed = target
            distance = actual_speed * DT / 10
            middle_heading = pose[2] + distance * command.curvature_per_m / 2
            pose[0] += distance * math.cos(middle_heading)
            pose[1] += distance * math.sin(middle_heading)
            pose[2] += distance * command.curvature_per_m
            max_lateral = max(max_lateral, abs(pose[1]))
            clearance = min(clearance, min(math.hypot(pose[0] - cx, pose[1] - cy)
                                           - CAR_RADIUS - CONE_RADIUS for cx, cy, _ in WORLD_CONES))
        if (policy.stopped_for_lane_loss or policy.stopped_for_solver_failure
                or getattr(policy, "stopped_at_endpoint", False)) and actual_speed < 0.01:
            break
    trace = np.asarray(trace)
    # The gate requires completion as well as stopping, preventing an early stop
    # from earning a false pass. Thresholds are provisional, not qualification.
    complete = LANE_LENGTH <= pose[0] <= LANE_LENGTH + 0.5
    stopped = (policy.stopped_for_lane_loss or policy.stopped_for_solver_failure
               or getattr(policy, "stopped_at_endpoint", False)) and actual_speed < 0.01
    metrics = {
        "case": case.name, "seed": seed,
        "rms_lateral_m": float(np.sqrt(np.mean(trace[:, 2] ** 2))),
        "max_lateral_m": max_lateral,
        "rms_curvature_rate": float(np.sqrt(np.mean((np.diff(trace[:, 5]) / DT) ** 2))),
        "p99_update_ms": float(np.percentile(times, 99) * 1000),
        "fallbacks": policy.fallback_count, "stop_x_m": float(pose[0]),
        "stop_speed_m_per_s": actual_speed,
        "solver_stop": policy.stopped_for_solver_failure,
        "endpoint_stop": getattr(policy, "stopped_at_endpoint", False),
        "endpoint_abort": getattr(policy, "endpoint_abort_reason", None),
        "min_cone_clearance_m": clearance, "stopped": stopped,
        "completed": bool(complete),
        "pass": bool(complete and stopped and clearance > 0
                     and max_lateral + CAR_RADIUS < LANE_HALF_WIDTH
                     and policy.fallback_count == 0 and np.percentile(times, 99) < 0.05),
    }
    return metrics, trace


def pareto_front(summaries):
    objectives = ("mean_rms_lateral_m", "mean_rms_curvature_rate")
    return [name for name, point in summaries.items() if not any(
        all(other[key] <= point[key] for key in objectives)
        and any(other[key] < point[key] for key in objectives)
        for rival, other in summaries.items() if rival != name)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "build/mpc-evaluation")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rows, traces, summaries = [], {}, {}
    for name, config in CANDIDATES.items():
        candidate_rows = []
        for case in TUNING:
            for seed in range(5):
                metrics, trace = rollout(config, case, seed)
                rows.append(dict(candidate=name, split="tuning", **metrics))
                candidate_rows.append(metrics)
                if seed == 0:
                    traces[(name, case.name)] = trace
        summaries[name] = {
            "passes": sum(row["pass"] for row in candidate_rows),
            "runs": len(candidate_rows),
            "mean_rms_lateral_m": float(np.mean([row["rms_lateral_m"] for row in candidate_rows])),
            "mean_rms_curvature_rate": float(np.mean([row["rms_curvature_rate"] for row in candidate_rows])),
            "worst_p99_update_ms": max(row["p99_update_ms"] for row in candidate_rows),
            "fallbacks": sum(row["fallbacks"] for row in candidate_rows),
        }
    front = pareto_front(summaries)
    # Predefined selection: all gates first, then lowest lateral RMS on the front.
    eligible = [name for name in front if summaries[name]["passes"] == summaries[name]["runs"]]
    selected = min(eligible, key=lambda n: summaries[n]["mean_rms_lateral_m"]) if eligible else None
    # If every design fails, validate the lowest-error Pareto candidate diagnostically.
    diagnostic = selected or min(front, key=lambda n: summaries[n]["mean_rms_lateral_m"])
    for case in VALIDATION:
        for seed in range(100, 105):
            metrics, trace = rollout(CANDIDATES[diagnostic], case, seed)
            rows.append(dict(candidate=diagnostic, split="validation", **metrics))
            if seed == 100:
                traces[(diagnostic, case.name)] = trace
    with (args.output / "metrics.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    report = {
        "model": "Ideal speed/curvature kinematic model; not DREAMGym or hardware acceptance",
        "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "python": platform.python_version(), "platform": platform.platform(),
        "numpy": np.__version__, "scipy": scipy.__version__,
        "source_sha256": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in [Path(__file__).resolve(), *sorted((ROOT / "policy").rglob("*.py"))]},
        "candidates": {name: dataclasses.asdict(config) for name, config in CANDIDATES.items()},
        "tuning_cases": [dataclasses.asdict(case) for case in TUNING],
        "validation_cases": [dataclasses.asdict(case) for case in VALIDATION],
        "summaries": summaries, "pareto_front": front, "selected": selected,
        "validation_candidate": diagnostic,
        "validation_passes": sum(row["pass"] for row in rows if row["split"] == "validation"),
        "validation_runs": 10,
    }
    (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    os.environ.setdefault("MPLCONFIGDIR", str(args.output / ".matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 3, figsize=(13, 7), constrained_layout=True)
    for ax, case in zip(axes.flat, TUNING + VALIDATION):
        for (name, case_name), trace in traces.items():
            if case_name == case.name:
                ax.plot(trace[:, 1], trace[:, 2], label=name)
                ax.plot(trace[-1, 1], trace[-1, 2], "x")
        ax.axhline(0, color="gray", linewidth=0.5)
        ax.axhline(0.32, color="red", linestyle=":")
        ax.axhline(-0.32, color="red", linestyle=":")
        ax.axvline(6, color="gray", linestyle="--")
        ax.set(title=case.name, xlabel="Along lane (m)", ylabel="Lateral position (m)", ylim=(-0.5, 0.5))
        ax.legend(fontsize=8)
    fig.suptitle("MPC kinematic screening — example seeds; x marks final stop")
    fig.savefig(args.output / "trajectories.png", dpi=160)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(7, 5), constrained_layout=True)
    for name, summary in summaries.items():
        x, y = summary["mean_rms_lateral_m"], summary["mean_rms_curvature_rate"]
        ax.scatter(x, y, marker="o" if name in front else "x")
        ax.annotate(name, (x, y), xytext=(5, 5), textcoords="offset points")
    ax.set(xlabel="Mean lateral RMS (m), lower is better",
           ylabel="Mean curvature-rate RMS (1/m/s), lower is better",
           title="Tuning trade-off — circles are Pareto candidates")
    fig.savefig(args.output / "pareto.png", dpi=160)
    plt.close(fig)
    print(json.dumps({key: report[key] for key in ("summaries", "pareto_front", "selected",
                      "validation_candidate", "validation_passes", "validation_runs")}, indent=2))


if __name__ == "__main__":
    main()
