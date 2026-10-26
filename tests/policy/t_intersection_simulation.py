"""Offline T-junction navigation, forbidden directions and sparse-cone probes.

Run: OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m \
    tests.policy.t_intersection_simulation --output build/t-intersection-20261026

Only the marker ID changes between requested directions on each fixed physical layout.
Geometry is supplied to sensor generation/scoring, never as policy topology or a target.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np

from tests.policy.intersection_simulation import rollout, settings, source_hashes
from tests.policy.intersection_turning import draw, track_for

ROOT = Path(__file__).resolve().parents[2]
LAYOUTS = ("straight", "left", "right")
DIRECTIONS = ("left", "right", "straight")


def expected_stop(result):
    m = result["metrics"]
    return (m["command_recognised"] and m["policy_stop"] == "INTERSECTION" and m["actual_exit"] is None
            and "closed by a supported cone row" in m["intersection_stop_reason"]
            and m["cone_contacts"] == 0 and m["obstacle_contacts"] == 0 and m["max_body_outside_m"] == 0.)


def figures(results, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.lines import Line2D

    def case(closed, direction, profile="nominal"):
        return next(r for r in results if r["track"]["closed_exit"] == closed and r["direction"] == direction
                    and r["profile"] == profile and r["track"]["cone_spacing_m"] == .2)

    figure, axes = plt.subplots(3, 3, figsize=(16, 16))
    figure.subplots_adjust(left=.05, right=.99, bottom=.07, top=.91, hspace=.38, wspace=.2)
    for row, closed in enumerate(LAYOUTS):
        for col, direction in enumerate(DIRECTIONS):
            result = case(closed, direction)
            axis = axes[row, col]
            draw(axis, result, result["frames"][-1], complete=True)
            axis.set_title(f"T: {closed} road closed\n"+axis.get_title(), fontsize=9)
    figure.suptitle("Three T-junction layouts: follow an observed exit; stop for a closed road\n"
                   "0.2 m cone spacing | 2.4 m road width | controller-driven simulated paths", fontsize=14)
    legend = [Line2D([], [], color=c, linestyle=style, label=label) for c, style, label in (
        ("#334155", "-", "Actual simulated trajectory"), ("#94a3b8", ":", "Observed local cone fit"),
        ("#16a34a", "-", "Selected centre path used by MPC"))]
    figure.legend(handles=legend, loc="lower center", ncol=3, bbox_to_anchor=(.5, .01))
    figure.savefig(output/"t_intersection.png", dpi=130)
    plt.close(figure)

    animated = [case(closed, d, "nominal" if d == "straight" else "turn_camera_gap")
                for closed in LAYOUTS for d in DIRECTIONS if d != closed]
    figure, axes = plt.subplots(3, 2, figsize=(12, 16))
    figure.subplots_adjust(left=.06, right=.99, bottom=.04, top=.88, hspace=.4, wspace=.2)
    duration = max(r["frames"][-1]["time_s"] for r in animated)

    def update(time):
        for axis, result in zip(axes.flat, animated):
            frame = result["frames"][min(round(time/.1), len(result["frames"])-1)]
            draw(axis, result, frame)
            axis.set_title(f"T: {result['track']['closed_exit']} road closed\n"+axis.get_title(), fontsize=9)
        figure.suptitle("Available T exits | 0.8 s camera-gap scenario on turns (trigger: 20° heading)\n"
                       "Fixed forward camera | 0.2 m cones | 5× playback", fontsize=12)
    animation = FuncAnimation(figure, update, frames=np.arange(0., duration+.01, .5), interval=100)
    animation.save(output/"t_intersection.gif", writer=PillowWriter(fps=10), dpi=85)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"build/t-intersection-20261026")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    params, detector = settings()
    if params["mpc"]["camera_yaw_offset_deg"] != 0.:
        raise ValueError("The fixture requires the configured zero fixed camera yaw")
    hashes = source_hashes()
    for name in ("t_intersection_simulation.py", "intersection_turning.py", "intersection_identification.py"):
        path = ROOT/"tests/policy"/name
        hashes[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    results = []
    for spacing in (.2, .5, .6):
        for closed in LAYOUTS:
            track = replace(track_for(spacing, corner_anchored=True), closed_exit=closed)
            for direction in DIRECTIONS:
                profiles = ["nominal"]
                if spacing == .2:
                    profiles.append("noisy_cones")
                    if direction != closed and direction != "straight":
                        profiles += ["turn_blue_missing", "turn_camera_gap", "turn_corner_missing"]
                for profile in profiles:
                    result = rollout(direction, profile, 261026, track=track, navigation=True, record_frames=True)
                    available = direction != closed
                    result["evaluation"] = {"requested_exit_exists": available,
                        "expectation_met": result["metrics"]["navigation_pass"] if available else expected_stop(result)}
                    results.append(result)
                    print(spacing, "closed", closed, direction, profile, result["metrics"]["reason"],
                          "expected result", result["evaluation"]["expectation_met"], flush=True)
    dense = [r for r in results if r["track"]["cone_spacing_m"] == .2]
    allowed = [r for r in dense if r["evaluation"]["requested_exit_exists"]]
    forbidden = [r for r in dense if not r["evaluation"]["requested_exit_exists"]]
    sparse = [r for r in results if r["track"]["cone_spacing_m"] != .2]
    report = {"scope": "Closed-loop local T-junctions using decoded cones/markers, current lidar, motion estimates, MPC and CAR_20.",
        "source_sha256": hashes, "policy_parameters": params, "detector_parameters": detector,
        "effective_intersection_parameters": dict(params["intersection"], enabled=True),
        "assumptions": {"width_m": 2.4, "radius_m": params["intersection"]["turning_radius_m"],
            "cone_spacings_m": [.2, .5, .6], "corner_anchored": True, "closed_exit_layouts": list(LAYOUTS),
            "camera_half_fov_deg": 43., "camera_range_m": 2.5, "near_forward_m": .4, "sensor_age_s": .2,
            "lidar_range_m": 4., "policy_period_s": .1, "plant_period_s": .01,
            "approach_endpoint": "First candidate centre minus half width, car front and configured stopping margin; never advanced.",
            "approach_control": "Current bilateral incoming lidar and fresh cone frame; command pending until selected-exit admission."},
        "limitations": ["Synthetic decoded sensors, not GPU/ArUco pixels, ROS watchdogs or physical qualification.",
            "Equal-width perpendicular-junction prior; no arbitrary angles, staggered roads or unequal widths.",
            "Ideal three-return cone lidar without occlusion; no tyre slip, measured braking or actuator deadtime.",
            "Sparse cases can fail on approach before marker activation, topology admission or turn-loss injection.",
            "Equal seeds reproduce cases but do not pair noise across different cone counts.",
            "Optional navigator enabled only locally; no vehicle connection or deployment."],
        "case_count": len(results), "policy_frames": sum(len(r["frames"]) for r in results),
        "dense_available_cases": len(allowed), "dense_navigation_passes": sum(r["metrics"]["navigation_pass"] for r in allowed),
        "dense_forbidden_cases": len(forbidden), "dense_closed_exit_stops": sum(expected_stop(r) for r in forbidden),
        "sparse_cases": len(sparse), "sparse_navigation_passes": sum(r["metrics"]["navigation_pass"] for r in sparse),
        "disturbance_cases": sum("disturbance" in r for r in results),
        "disturbance_triggered_cases": sum(r.get("disturbance", {}).get("started_at_s") is not None for r in results),
        "runs": [{k: v for k, v in r.items() if k not in ("trajectory", "frames")} for r in results]}
    serialised = [{k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in r.items()} for r in results]
    with gzip.open(args.output/"runs.json.gz", "wt") as stream:
        json.dump(serialised, stream)
    for result in results:
        name = f"spacing-{result['track']['cone_spacing_m']}-closed-{result['track']['closed_exit']}-{result['profile']}-{result['direction']}.csv"
        np.savetxt(args.output/name, result["trajectory"], delimiter=",",
                   header="time_s,x_m,y_m,heading_rad,speed_m_per_s,drive,steer,command", comments="")
    figures(results, args.output)
    for name, digest in hashes.items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != digest:
            raise ValueError(f"Executed source changed during simulation: {name}")
    (args.output/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({k: v for k, v in report.items() if k.endswith("cases") or k.endswith("passes")
                      or k in ("case_count", "policy_frames", "dense_closed_exit_stops")}, indent=2))


if __name__ == "__main__":
    main()
