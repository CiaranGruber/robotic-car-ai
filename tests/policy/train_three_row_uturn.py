"""Train and evaluate an OFFLINE steering student on three finite parallel cone rows.

OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m \
    tests.policy.train_three_row_uturn --output build/three-row-uturn

Training is from scratch, with no downloaded checkpoint. Episode/layout splits and every failed
rollout are retained in report.json. Only measured/remembered sensor features enter the network;
simulator ground truth is used exclusively for sensor generation and scoring. The deterministic
planner, cone memory and stop guards are shared by teacher and student. No ROS selection or upload.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

from policy.action_policy.three_row_uturn import (FEATURE_COUNT, FEATURE_VERSION, ThreeRowConfig,
                                                 ThreeRowNetwork, ThreeRowPolicy)
from tests.policy.three_row_simulation import ThreeRowCase, ThreeRowTrack, simulate

SOURCES = ("policy/action_policy/three_row_uturn.py", "policy/memory.py", "policy/input_output.py",
           "policy/serialisation.py", "policy/cone_filter/cone_filter.py", "policy/control/actions.py",
           "policy/lane_detection/lane_detection.py", "policy/action_policy/corridor.py",
           "policy/lane_detection/temporal.py", "tests/policy/three_row_simulation.py",
           "tests/policy/train_three_row_uturn.py", "tests/policy/curved_lanes.py",
           "tests/policy/corridor_runtime_simulation.py", "tests/policy/real_car_measurements.py",
           "tests/policy/stage1_scenarios.py")


def source_hashes():
    return {p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in SOURCES}


def check_frozen_inputs(report, config, path):
    """Separate evaluation cannot relabel an old checkpoint with new code or configuration."""
    if (hashlib.sha256(Path(path).read_bytes()).hexdigest() != report["model_sha256"]
            or report["config"] != asdict(config) or report["source_sha256"] != source_hashes()):
        raise ValueError("Frozen checkpoint, training source or configuration changed since validation")


def training_cases():
    profiles = ("nominal", "random_drops", "divider_hidden", "outer_hidden", "short_all_hidden", "nominal")
    cases = []
    for i in range(12):
        track = ThreeRowTrack(divider_length_m=(2.7, 3.2, 3.6)[i % 3],
                              row_gap_m=(.95, 1.05)[i % 2], outer_extension_m=(.85, 1.15)[i % 2],
                              placement_seed=30+i, removed_fraction=.12 if i in (5, 11) else 0.,
                              remove_divider_tip=i == 11)
        cases.append(ThreeRowCase(track, seed=200+i, profile=profiles[i % 6],
                                  start_left_m=(-.03, .03)[i % 2], start_heading_deg=(-1., 1.)[i % 2]))
    return cases


def validation_cases():
    return [ThreeRowCase(ThreeRowTrack(divider_length_m=length, row_gap_m=gap,
                        outer_extension_m=extension, placement_seed=500+i), seed=600+i,
                        profile=profile, start_left_m=.02*(-1)**i)
            for i, (length, gap, extension, profile) in enumerate((
                (2.9, .98, .95, "nominal"), (3.5, 1.02, 1.05, "random_drops"),
                (2.9, 1.02, 1.05, "outer_hidden"), (3.5, .98, .95, "short_all_hidden")))]


def holdout_cases():
    # These layouts are not used for normalisation, optimisation, DAgger or checkpoint selection.
    profiles = ("nominal", "random_drops", "divider_hidden", "outer_hidden", "short_all_hidden",
                "nominal", "random_drops", "divider_hidden")
    return [ThreeRowCase(ThreeRowTrack(divider_length_m=(2.8, 3.1, 3.4, 3.7)[i % 4],
                        row_gap_m=(.97, 1.03)[i % 2], outer_extension_m=(.9, 1.1)[i % 2],
                        placement_seed=4000+i, removed_fraction=.12 if i == 5 else 0.,
                        remove_divider_tip=i == 6), seed=4200+i, profile=profiles[i],
                        start_left_m=.025*(-1)**i, start_heading_deg=.75*(-1)**i)
            for i in range(len(profiles))]


def negative_cases():
    track = ThreeRowTrack(placement_seed=1500)
    return [ThreeRowCase(track, seed=1600+i, profile=profile)
            for i, profile in enumerate(("long_all_hidden", "stale_gyro", "stale_wheel", "terminal_loss"))]


def rollout(job):
    case, config, weights = job
    x, y = [], []
    def collect(features, target):
        x.append(features)
        y.append([target])
    model = None if weights is None else ThreeRowNetwork(weights, config)
    policy = ThreeRowPolicy(config, model, collect)
    result = simulate(case, policy)
    return {"case": asdict(case), "metrics": result["metrics"], "trajectory": result["trajectory"],
            "x": np.asarray(x).reshape(-1, FEATURE_COUNT), "y": np.asarray(y).reshape(-1, 1)}


def run_cases(cases, config, weights, workers):
    jobs = [(case, config, None if weights is None else str(weights)) for case in cases]
    if workers == 1:
        results = list(map(rollout, jobs))
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            results = list(pool.map(rollout, jobs))
    for i, result in enumerate(results):
        metrics = result["metrics"]
        print("rollout", i, result["case"]["profile"], "pass", metrics["safe_completion"],
              "contacts", metrics["cones_touched"], "outside", metrics["max_outside_m"],
              "stop", metrics["stop_reason"], flush=True)
    return results


def train_network(x, y, path, config, seed=4, network_type=ThreeRowNetwork):
    mean = x.mean(0)
    scale = np.maximum(x.std(0), .05)
    z = (x-mean)/scale
    shapes = ((FEATURE_COUNT, 32), (32,), (32, 32), (32,), (32, 1), (1,))
    ends = np.cumsum([0]+[int(np.prod(shape)) for shape in shapes])
    def unpack(vector):
        return [vector[ends[i]:ends[i+1]].reshape(shape) for i, shape in enumerate(shapes)]
    rng = np.random.default_rng(seed)
    initial = np.concatenate([(rng.normal(0, .12, shape) if len(shape) == 2 else np.zeros(shape)).ravel()
                              for shape in shapes])
    def objective(vector):
        w1, b1, w2, b2, w3, b3 = unpack(vector)
        h1 = np.tanh(z@w1+b1)
        h2 = np.tanh(h1@w2+b2)
        error = h2@w3+b3-y
        loss = np.mean(error**2)+1e-7*np.sum(vector**2)
        d = 2*error/error.size
        gw3, gb3 = h2.T@d, d.sum(0)
        d2 = (d@w3.T)*(1-h2*h2)
        gw2, gb2 = h1.T@d2, d2.sum(0)
        d1 = (d2@w2.T)*(1-h1*h1)
        gw1, gb1 = z.T@d1, d1.sum(0)
        gradient = np.concatenate([a.ravel() for a in (gw1, gb1, gw2, gb2, gw3, gb3)])+2e-7*vector
        return loss, gradient
    result = minimize(objective, initial, jac=True, method="L-BFGS-B",
                      options={"maxiter": 650, "ftol": 1e-11, "maxcor": 12})
    arrays = dict(zip(("w1", "b1", "w2", "b2", "w3", "b3"), unpack(result.x)))
    np.savez(path, mean=mean, scale=scale,
             contract=network_type.contract(config), **arrays)
    model = network_type(path, config)
    error = np.array([model.predict(row) for row in x])-y[:, 0]
    return {"samples": len(x), "iterations": int(result.nit), "loss": float(result.fun),
            "rmse_per_m": float(np.sqrt(np.mean(error**2))),
            "optimiser_success": bool(result.success), "message": str(result.message)}


def records(results):
    return [{"case": result["case"], "metrics": result["metrics"]} for result in results]


def summary(results):
    return {"episodes": len(results),
            "navigation_passes": sum(r["metrics"]["safe_completion"] for r in results),
            "cone_contacts": sum(r["metrics"]["cones_touched"] for r in results),
            "max_footprint_excursion_m": max(r["metrics"]["max_outside_m"] for r in results)}


def plot_results(cases, teacher, student, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(2, 4, figsize=(16, 7), squeeze=False)
    for i, (case, baseline, learned, axis) in enumerate(zip(cases, teacher, student, axes.flat)):
        for lo, hi, bottom, top in case.track.rectangles():
            axis.fill([lo, hi, hi, lo], [bottom, bottom, top, top], color="#eef2f6", zorder=0)
        for _, colour, x, y in case.track.row_cones():
            axis.scatter(x, y, c="royalblue" if colour.name == "BLUE" else "goldenrod", s=10)
        for result, label in ((baseline, "teacher"), (learned, "learned steering")):
            t = result["trajectory"]
            axis.plot(t[:, 1], t[:, 2], label=label, linewidth=1.3)
        metrics = learned["metrics"]
        axis.set_title(f"{i+1}: {case.profile}\n{'PASS' if metrics['safe_completion'] else 'FAIL'}; "
                       f"empty camera up to {metrics['max_empty_camera_s']:.1f}s", fontsize=9)
        axis.set_aspect("equal")
        axis.set_xlabel("metres")
        axis.legend(fontsize=7)
    figure.suptitle("Three straight cone rows: frozen model on unseen layouts (declared open apron)")
    figure.tight_layout()
    figure.savefig(output, dpi=150)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("build/three-row-uturn"))
    parser.add_argument("--workers", type=int, default=2, choices=(1, 2, 3, 4))
    parser.add_argument("--stage", choices=("train", "evaluate", "all"), default="all")
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    config = ThreeRowConfig()
    path, report_path = out/"steering.npz", out/"report.json"
    report = json.loads(report_path.read_text()) if args.stage == "evaluate" else {
        "method": "Behaviour cloning + one DAgger round; 17->32->32->1 tanh MLP trained from scratch",
        "offline_only": True, "config": asdict(config), "source_sha256": source_hashes(),
        "assumptions": "Three parallel straight rows; yellow divider, blue outers; nominal 1m gaps and "
        "1m outer extension. Forward only, curvature +1/-0.92 per m, 1.18m planning radius, "
        "4m deep and 0.9m extra lateral clear apron, fixed 86deg camera, 0.2s latency, ideal 360deg "
        "cone lidar, no slip and measured-motion-equivalent odometry. Full normalized steering "
        "endpoints and apron are unverified on the vehicle. Goal gate measures navigation, not final stopping.",
        "memory_contract": "Offline bounded fresh-empty stream hold, at most 36s/12m from observed turn. "
        "A new approach plan requires newly revealed extent and newer observations of all three endpoints; "
        "fresh separated support from both return boundaries can replace the return route. Camera loss can last "
        "about 20s in this wider manoeuvre; deployed ROS corridor contracts are unchanged."}
    def save():
        report_path.write_text(json.dumps(report, indent=2)+"\n")
    if args.stage in ("train", "all"):
        demos = run_cases(training_cases(), config, None, args.workers)
        x = np.concatenate([r["x"] for r in demos])
        y = np.concatenate([r["y"] for r in demos])
        report["training_episodes"] = records(demos)
        report["initial_fit"] = train_network(x, y, path, config)
        save()
        print("initial fit", report["initial_fit"], flush=True)
        dagger = run_cases(training_cases(), config, path, args.workers)
        report["dagger_episodes"] = records(dagger)
        x = np.concatenate([x, *[r["x"] for r in dagger]])
        y = np.concatenate([y, *[r["y"] for r in dagger]])
        np.savez_compressed(out/"demonstrations.npz", x=x, y=y)
        report["final_fit"] = train_network(x, y, path, config)
        report["validation"] = records(run_cases(validation_cases(), config, path, args.workers))
        report["model_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
        save()
        print("final fit", report["final_fit"], flush=True)
    if args.stage in ("evaluate", "all"):
        check_frozen_inputs(report, config, path)
        cases = holdout_cases()
        teacher = run_cases(cases, config, None, args.workers)
        student = run_cases(cases, config, path, args.workers)
        report["holdout"] = {"teacher": records(teacher), "student": records(student),
                              "teacher_summary": summary(teacher), "student_summary": summary(student)}
        save()
        negative = run_cases(negative_cases(), config, path, args.workers)
        report["negative_tests"] = records(negative)
        ablation = run_cases(cases[:2], replace(config, memory_enabled=False), path, args.workers)
        report["without_memory"] = records(ablation)
        for i, result in enumerate(student):
            np.savez_compressed(out/f"holdout-{i+1}.npz", trajectory=result["trajectory"])
        plot_results(cases, teacher, student, out/"evaluation.png")
        print("frozen holdout", report["holdout"]["student_summary"], flush=True)
    if report["source_sha256"] != source_hashes():
        raise ValueError("Source changed during this experiment; evidence has not been relabelled")
    save()


if __name__ == "__main__":
    main()
