"""Closed blue-boundary feasibility, sensitivity and explicitly hypothetical offline steering training.

OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m \
  tests.policy.train_connected_uturn --train-hypothesis --output build/connected-uturn

Default vehicle assumption remains +1/-0.92 per m and is reported as infeasible. --train-hypothesis
also trains a DISTINCT model at an explicitly unverified +2.2/-0.92 per m, planned radius .5m.
No hardware capability, vehicle setting, ROS selection or upload is changed by this experiment.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, fields, replace
import hashlib
import json
from pathlib import Path

import numpy as np

from policy.action_policy.connected_uturn import (ConnectedTurnConfig, ConnectedTurnNetwork,
                                                 ConnectedTurnPolicy, minimum_strip_width)
from tests.policy.connected_uturn_simulation import ConnectedUTurnTrack, ThreeRowCase, simulate
from tests.policy.train_three_row_uturn import (FEATURE_COUNT, SOURCES, training_cases, validation_cases,
                                               holdout_cases, records, summary, train_network)

CONNECTED_SOURCES = (*SOURCES, "policy/action_policy/connected_uturn.py",
                     "tests/policy/connected_uturn_simulation.py", "tests/policy/train_connected_uturn.py")


def source_hashes():
    return {p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in CONNECTED_SOURCES}


def closed_cases(split, config):
    original = {"training": training_cases, "validation": validation_cases, "holdout": holdout_cases}[split]()
    result = []
    allowed = {field.name for field in fields(ConnectedUTurnTrack)}
    for i, case in enumerate(original):
        values = {key: value for key, value in asdict(case.track).items() if key in allowed}
        if split == "holdout":
            values["placement_seed"] += 6000
        track = ConnectedUTurnTrack(**values, remove_cap_cone=(i == 5 and split == "holdout"))
        result.append(replace(case, track=track, seed=case.seed+(6000 if split == "holdout" else 0),
                              max_left_curvature_per_m=config.max_left_curvature_per_m,
                              max_right_curvature_per_m=config.max_right_curvature_per_m))
    return result


def rollout(job):
    case, config, path = job
    x, y = [], []
    def collect(features, target):
        x.append(features)
        y.append([target])
    network = ConnectedTurnNetwork(path, config) if path else None
    result = simulate(case, ConnectedTurnPolicy(config, network, collect))
    return {"case": asdict(case), "metrics": result["metrics"], "trajectory": result["trajectory"],
            "x": np.asarray(x).reshape(-1, FEATURE_COUNT), "y": np.asarray(y).reshape(-1, 1)}


def run_cases(cases, config, path=None, workers=2):
    jobs = [(case, config, str(path) if path else None) for case in cases]
    if workers == 1:
        result = list(map(rollout, jobs))
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            result = list(pool.map(rollout, jobs))
    for i, row in enumerate(result):
        metric = row["metrics"]
        print(i, row["case"]["profile"], "pass", metric["safe_completion"], "contacts",
              metric["cones_touched"], "outside", metric["max_outside_m"], "stop", metric["stop_reason"], flush=True)
    return result


def draw(cases, teacher, student, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(2, 4, figsize=(16, 7))
    for i, (case, base, learned, axis) in enumerate(zip(cases, teacher, student, axes.flat)):
        border = case.track.boundary()
        axis.fill(border[:, 0], border[:, 1], color="#eef2f6", zorder=0)
        outline = border.copy()
        outline[[0, -1], 0] = 0.  # Left entrance is open; no invented blue closing row.
        axis.plot(*outline.T, color="royalblue", alpha=.35, linewidth=1)
        for _, colour, x, y in case.track.row_cones():
            axis.scatter(x, y, color="royalblue" if colour.name == "BLUE" else "goldenrod", s=10)
        for row, label in ((base, "analytic teacher"), (learned, "learned steering")):
            t = row["trajectory"]
            axis.plot(t[:, 1], t[:, 2], linewidth=1.2, label=label)
        axis.set_title(f"{i+1}: {case.profile}\n{'PASS' if learned['metrics']['safe_completion'] else 'FAIL'}", fontsize=10)
        axis.set_xlabel("metres")
        axis.set_aspect("equal")
        axis.legend(fontsize=7)
    figure.suptitle("Connected blue boundary · hypothetical +2.2/m steering · physical capability unverified")
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("build/connected-uturn"))
    parser.add_argument("--workers", type=int, default=2, choices=(1, 2, 3, 4))
    parser.add_argument("--train-hypothesis", action="store_true")
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    report = {"source_sha256": source_hashes(), "offline_only": True,
        "course": asdict(ConnectedUTurnTrack()),
        "course_interpretation": "Three straight rows with 1m adjacent gaps. Both blue straight rows continue "
            "1m beyond the last yellow cone, then join through a semicircular cap of radius1m. "
            "Furthest blue x is2m beyond yellow tip. Continuous closed floor; no open apron.",
        "limitations": "Only steering is learned; cap fitting, cone memory, path planning and stop guards are "
            "deterministic. Sensed cap has no row IDs or simulator pose. Fixed86deg camera,0.2s latency, "
            "0.03m detection noise, ideal lidar cone surfaces within4m, no-slip ideal measured odometry. "
            "Requested-speed plant with lag; braking0.2s dead time+0.8s linear deceleration. "
            "Completion is navigation through return goal while moving, not final stopping. "
            "No steering, TF, braking or vehicle qualification; no deployment.",
        "memory_contract": "Generated route retains original acquisition anchor, at most36s/12m. "
            "Fresh-empty filtered camera stream for3s causes latched stop even if route survives. "
            "Surviving filtered detections prevent the empty timeout but cannot refresh the retained anchor. "
            "Fresh two-boundary return paths cannot shorten or refresh an unsupported retained terminal tail."}
    def save():
        (out/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    baseline = ConnectedTurnConfig()
    base_case = ThreeRowCase(ConnectedUTurnTrack())
    base = run_cases([base_case], baseline, workers=args.workers)
    report["baseline"] = {"config": asdict(baseline), "required_strip_width_m": minimum_strip_width(1.),
                          "available_width_m": 2., "necessary_feasibility": False, "episodes": records(base)}
    sensitivity = []
    for limit in (1., 1.2, 1.4, 1.6, 1.8, 2., 2.2):
        radius = max(.5, 1/limit)
        cfg = ConnectedTurnConfig(max_left_curvature_per_m=limit, turning_radius_m=radius)
        case = replace(base_case, max_left_curvature_per_m=limit)
        result = run_cases([case], cfg, workers=1)
        sensitivity.append({"assumed_left_limit_per_m": limit, "planned_radius_m": radius,
                            "necessary_strip_width_m": minimum_strip_width(limit), "metrics": result[0]["metrics"]})
    report["hypothetical_sensitivity"] = sensitivity
    save()
    if not args.train_hypothesis:
        return
    config = ConnectedTurnConfig(max_left_curvature_per_m=2.2, turning_radius_m=.5)
    report["hypothetical_training_config"] = asdict(config)
    demos = run_cases(closed_cases("training", config), config, workers=args.workers)
    report["training_episodes"] = records(demos)
    x, y = np.concatenate([r["x"] for r in demos]), np.concatenate([r["y"] for r in demos])
    weights = out/"hypothetical-steering.npz"
    report["initial_fit"] = train_network(x, y, weights, config, network_type=ConnectedTurnNetwork)
    print("initial fit", report["initial_fit"], flush=True)
    dagger = run_cases(closed_cases("training", config), config, weights, args.workers)
    report["dagger_episodes"] = records(dagger)
    x, y = np.concatenate([x, *[r["x"] for r in dagger]]), np.concatenate([y, *[r["y"] for r in dagger]])
    np.savez_compressed(out/"demonstrations.npz", x=x, y=y)
    report["final_fit"] = train_network(x, y, weights, config, network_type=ConnectedTurnNetwork)
    print("final fit", report["final_fit"], flush=True)
    report["validation"] = records(run_cases(closed_cases("validation", config), config, weights, args.workers))
    report["model_sha256"] = hashlib.sha256(weights.read_bytes()).hexdigest()
    save()
    if source_hashes() != report["source_sha256"]:
        raise ValueError("Source changed during training; holdouts were not run")
    cases = closed_cases("holdout", config)
    teacher = run_cases(cases, config, workers=args.workers)
    student = run_cases(cases, config, weights, args.workers)
    report["holdout"] = {"teacher": records(teacher), "student": records(student),
                          "teacher_summary": summary(teacher), "student_summary": summary(student)}
    save()
    negatives = [replace(base_case, seed=7100+i, profile=profile, max_left_curvature_per_m=2.2)
                 for i, profile in enumerate(("long_all_hidden", "stale_gyro", "stale_wheel", "terminal_loss"))]
    report["negative_tests"] = records(run_cases(negatives, config, weights, args.workers))
    report["without_memory"] = records(run_cases(cases[:2], replace(config, memory_enabled=False), weights, args.workers))
    for i, row in enumerate(student):
        np.savez_compressed(out/f"holdout-{i+1}.npz", trajectory=row["trajectory"])
    draw(cases, teacher, student, out/"evaluation.png")
    if (source_hashes() != report["source_sha256"]
            or hashlib.sha256(weights.read_bytes()).hexdigest() != report["model_sha256"]):
        raise ValueError("Frozen source/checkpoint changed during evaluation")
    print("closed-boundary holdout", report["holdout"]["student_summary"], flush=True)
    save()


if __name__ == "__main__":
    main()
