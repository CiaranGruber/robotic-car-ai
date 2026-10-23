"""Preferred local connected-U experiment: online geometry and analytic feedback, NO trained weights.

OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m \
  tests.policy.evaluate_connected_realtime --hypothetical-tight-steering --output build/connected-realtime

Without the explicit hypothetical flag, use the earlier +1/-0.92 /m assumption and report its
infeasibility. The flag tests +2.2/-0.92 /m only in simulation; it does not establish car capability.
The committed turn remains bounded memory; steering is recomputed every tick, not learned online.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from policy.action_policy.connected_uturn import ConnectedTurnConfig, ConnectedTurnPolicy
from tests.policy.connected_uturn_simulation import ConnectedUTurnTrack, ThreeRowCase, simulate

SOURCES = ("policy/action_policy/connected_uturn.py", "policy/action_policy/three_row_uturn.py",
           "policy/action_policy/corridor.py", "policy/memory.py", "policy/input_output.py",
           "policy/serialisation.py", "policy/cone_filter/cone_filter.py", "policy/control/actions.py",
           "policy/lane_detection/lane_detection.py", "policy/lane_detection/temporal.py",
           "tests/policy/connected_uturn_simulation.py", "tests/policy/three_row_simulation.py",
           "tests/policy/curved_lanes.py", "tests/policy/corridor_runtime_simulation.py",
           "tests/policy/real_car_measurements.py", "tests/policy/stage1_scenarios.py",
           "tests/policy/evaluate_connected_realtime.py")


def source_hashes():
    return {p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in SOURCES}


def varied_cases(config):
    profiles = ("nominal", "random_drops", "divider_hidden", "outer_hidden", "short_all_hidden", "nominal")
    result = []
    for i in range(12):
        track = ConnectedUTurnTrack(divider_length_m=(2.6, 3.0, 3.8)[i % 3],
                  row_gap_m=(.9, 1., 1.1)[(i//2) % 3], outer_extension_m=(.9, 1.2)[i % 2],
                  cone_spacing_m=(.4, .55)[i % 2], placement_jitter_m=(.025, .04)[i % 2],
                  placement_seed=14000+i, removed_fraction=.1 if i == 5 else 0.,
                  remove_divider_tip=i == 9, remove_cap_cone=i == 11)
        result.append(ThreeRowCase(track, seed=14200+i, profile=profiles[i % 6],
                      start_left_m=.035*(-1)**i, start_heading_deg=(-1.)**i,
                      max_left_curvature_per_m=config.max_left_curvature_per_m,
                      max_right_curvature_per_m=config.max_right_curvature_per_m))
    return result


def shape_probes(config):
    return [ThreeRowCase(ConnectedUTurnTrack(cap_depth_scale=scale, placement_seed=15000+i),
                  seed=15200+i, max_left_curvature_per_m=config.max_left_curvature_per_m,
                  max_right_curvature_per_m=config.max_right_curvature_per_m)
            for i, scale in enumerate((.8, .9, 1.1, 1.2))]


def rollout(job):
    case, config = job
    # Deliberately no checkpoint argument, network construction, np.load or optimisation.
    controller = ConnectedTurnPolicy(config)
    assert controller.network is None
    durations = []
    calculate = controller.run_policy
    def timed(observations, lanes=None):
        started = time.perf_counter()
        commands = calculate(observations, lanes)
        durations.append((time.perf_counter()-started)*1000)
        return commands
    controller.run_policy = timed
    result = simulate(case, controller)
    result["case"] = asdict(case)
    result["metrics"]["tick_p95_ms"] = float(np.percentile(durations, 95))
    result["metrics"]["tick_max_ms"] = float(max(durations))
    return result


def run_cases(cases, config, workers):
    jobs = [(case, config) for case in cases]
    if workers == 1:
        results = list(map(rollout, jobs))
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            results = list(pool.map(rollout, jobs))
    for i, result in enumerate(results):
        m = result["metrics"]
        print(i, result["case"]["profile"], "completed", m["safe_completion"],
              "contacts", m["cones_touched"], "outside", m["max_outside_m"], "stop", m["stop_reason"], flush=True)
    return results


def records(results):
    return [{"case": result["case"], "metrics": result["metrics"]} for result in results]


def summarize(results):
    return {"episodes": len(results), "navigation_passes": sum(r["metrics"]["safe_completion"] for r in results),
            "guarded_stops": sum(r["metrics"]["latched_stop"] and not r["metrics"]["completed"] for r in results),
            "cone_contacts": sum(r["metrics"]["cones_touched"] for r in results),
            "max_outside_m": max(r["metrics"]["max_outside_m"] for r in results)}


def draw(cases, results, output, hypothetical_tight_steering):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(2, 3, figsize=(13, 7))
    for i, (case, result, axis) in enumerate(zip(cases, results, axes.flat)):
        boundary = case.track.boundary()
        axis.fill(*boundary.T, color="#eef2f6", zorder=0)
        outline = boundary.copy()
        outline[[0, -1], 0] = 0.
        axis.plot(*outline.T, color="royalblue", alpha=.35, linewidth=1)
        for _, colour, x, y in case.track.row_cones():
            axis.scatter(x, y, color="royalblue" if colour.name == "BLUE" else "goldenrod", s=12)
        t = result["trajectory"]
        axis.plot(t[:, 1], t[:, 2], color="darkorange", label="online geometric steering")
        label = "PASS" if result["metrics"]["safe_completion"] else "STOP" if result["metrics"]["latched_stop"] else "INCOMPLETE"
        axis.set_title(f"gap {case.track.row_gap_m:.2f}m · cap depth ×{case.track.cap_depth_scale:.1f}\n{label}: {case.profile}", fontsize=10)
        axis.set_aspect("equal")
        axis.set_xlabel("metres")
        axis.legend(fontsize=8)
    steering_label = "hypothetical tighter steering" if hypothetical_tight_steering else "earlier steering assumption"
    figure.suptitle(f"Live geometry + feedback, no trained weights · {steering_label}")
    figure.tight_layout()
    figure.savefig(output, dpi=150)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("build/connected-realtime"))
    parser.add_argument("--workers", type=int, default=2, choices=(1, 2, 3, 4))
    parser.add_argument("--hypothetical-tight-steering", action="store_true")
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    config = (ConnectedTurnConfig(max_left_curvature_per_m=2.2, turning_radius_m=.5)
              if args.hypothetical_tight_steering else ConnectedTurnConfig())
    report = {"controller": "online sensed geometry + analytic lookahead feedback", "learned_model_used": False,
              "online_learning": False, "config": asdict(config), "source_sha256": source_hashes(),
              "scope": "Local fixed-step10Hz simulation, no car upload/configuration changes. Geometry from sensed "
              "cones, acquisition-time wheel/gyro odometry and bounded memory. Initial turn radius/inset and "
              "circular-cap family remain assumptions. Turn/row frame retained after commitment; steering recomputed "
              "each tick. Supported fresh return geometry may replace the route. This is not continuous replanning "
              "or online model training. Elliptical caps are out-of-model probes. Stopping and physical steering "
              "are unverified. Measured tick times are local diagnostics, not hard realtime guarantees."}
    baseline = run_cases([ThreeRowCase(ConnectedUTurnTrack())], ConnectedTurnConfig(), args.workers)
    report["earlier_steering_baseline"] = records(baseline)
    cases = varied_cases(config)
    results = run_cases(cases, config, args.workers)
    probes = shape_probes(config)
    probe_results = run_cases(probes, config, args.workers)
    report["varied_layouts"], report["varied_summary"] = records(results), summarize(results)
    report["out_of_model_caps"], report["cap_summary"] = records(probe_results), summarize(probe_results)
    negatives = [replace(cases[0], profile=profile, seed=16200+i)
                 for i, profile in enumerate(("long_all_hidden", "stale_gyro", "stale_wheel", "terminal_loss"))]
    report["negative_tests"] = records(run_cases(negatives, config, args.workers))
    report["cone_memory_disabled"] = records(run_cases(cases[:2], replace(config, memory_enabled=False), args.workers))
    for i, result in enumerate(results):
        np.savetxt(out/f"trajectory-{i+1}.csv", result["trajectory"], delimiter=",", header="t,x,y,yaw,speed", comments="")
    draw([*cases[:4], *probes[::3]], [*results[:4], *probe_results[::3]], out/"evaluation.png",
         args.hypothetical_tight_steering)
    if source_hashes() != report["source_sha256"]:
        raise ValueError("Source changed during evaluation; evidence has not been relabelled")
    (out/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    print("varied layouts", report["varied_summary"], "shape probes", report["cap_summary"], flush=True)


if __name__ == "__main__":
    main()
