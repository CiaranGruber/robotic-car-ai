"""Closed-loop left/right/straight ArUco scenarios with cone loss during the actual turn.

Run: OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m \
    tests.policy.intersection_turning --output build/intersection-turning-20261026

The existing sensed navigator, MPC, control map and plant generate the trajectory. Only the
marker ID selects direction; simulator poses are used for sensors, disturbances and scoring.
The GPU detector remains unchanged; these inputs are decoded synthetic detections, not pixels.
"""
from __future__ import annotations

import argparse
from collections import Counter
import gzip
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from policy.input_output import ConeColour
from tests.policy.intersection_identification import world_point
from tests.policy.intersection_simulation import (CrossIntersection, rollout, settings, source_hashes,
                                                  TURN_LOSS_DURATIONS_S)
from tests.policy.stage1_scenarios import body_corners

ROOT = Path(__file__).resolve().parents[2]
PROFILES = ("nominal", "noisy_cones", "turn_blue_missing", "turn_camera_gap", "turn_corner_missing")


def track_for(spacing=.2, *, corner_anchored=False):
    return CrossIntersection(width_m=2.4, arm_length_m=8., cone_spacing_m=spacing,
                             marker_x_m=-3.8, marker_y_m=-.7, exit_gate_m=3.6, corner_anchored=corner_anchored)


def draw(axis, result, frame, *, complete=False):
    from matplotlib.patches import Polygon, Rectangle, Wedge
    axis.clear()
    track = CrossIntersection(**result["track"])
    h = track.width_m/2
    for rectangle in track.road_rectangles():
        axis.add_patch(Rectangle(rectangle[:2], *rectangle[2:], facecolor="#f1f5f9"))
    for colour, display in ((ConeColour.BLUE, "#2563eb"), (ConeColour.YELLOW, "#d69c00")):
        cones = [(x, y) for x, y, c in track.cones() if c is colour]
        axis.scatter(*np.asarray(cones).T, s=13, c=display, marker="^", alpha=.3)
    trajectory = result["trajectory"]
    visible = trajectory if complete else trajectory[trajectory[:, 0] <= frame["time_s"]+1e-8]
    axis.plot(visible[:, 1], visible[:, 2], color="#334155", linewidth=1.8, label="Actual simulated trajectory")
    pose = frame["pose"]
    acquisition = frame["acquisition_pose"]
    if acquisition is not None:
        heading = math.degrees(acquisition[2])
        axis.add_patch(Wedge(acquisition[:2], 2.5, heading-43, heading+43, facecolor="#bfdbfe", alpha=.2))
        for name, display in (("BLUE", "#2563eb"), ("YELLOW", "#d69c00")):
            measured = [world_point(x, y, acquisition) for x, y, c in frame["checked_cones"] if c == name]
            if measured:
                axis.scatter(*np.asarray(measured).T, c=display, marker="^", s=33)
        if frame["observed_centre"]:
            points = np.array([world_point(x, y, acquisition) for x, y in frame["observed_centre"]])
            axis.plot(*points.T, color="#94a3b8", linewidth=1.1, linestyle=":", label="Observed local lane fit")
    if frame["selected_centre"]:
        points = np.array([world_point(x, y, pose) for x, y in frame["selected_centre"]])
        axis.plot(*points.T, color="#16a34a", linewidth=2.2, label="Selected centre path → MPC")
    axis.add_patch(Polygon(body_corners(*pose), facecolor="#334155", zorder=7))
    axis.annotate("", (pose[0]+.5*math.cos(pose[2]), pose[1]+.5*math.sin(pose[2])), pose[:2],
                  arrowprops={"arrowstyle": "->", "color": "#334155"})
    axis.scatter(track.marker_x_m, track.marker_y_m, marker="s", s=35, c="#9333ea")
    metrics = result["metrics"]
    outcome = metrics['policy_stop'] or metrics['reason']
    status = (f"exit {metrics['actual_exit'] or 'none'} | {outcome}" if complete else
              f"t={frame['time_s']:.1f}s | {frame['phase']} | steer {frame['steer']:+.2f}")
    axis.set_title(f"ID {result['marker_id']} → {result['direction']} | {result['profile']} | gaps {track.cone_spacing_m:g} m\n{status}\n"
                   f"cones {len(frame['raw_cones'])} → {len(frame['checked_cones'])} | observed {frame['lane_status']}"
                   +f" | selected {frame['selected_command'] or 'none'}"
                   +( " | occlusion active" if frame["disturbance_active"] else ""), fontsize=9)
    axis.set(xlim=(-6., 4.5), ylim=(-4.5, 4.5), aspect="equal", xlabel="World x (m)", ylabel="World y (m)")
    axis.grid(alpha=.15)


def figures(results, output, plot_spacing=.2):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.lines import Line2D
    from matplotlib.patches import Rectangle
    spacings = sorted({r["track"]["cone_spacing_m"] for r in results})
    figure, axes = plt.subplots(1, len(spacings), figsize=(7*len(spacings), 7), squeeze=False)
    for axis, spacing in zip(axes.flat, spacings):
        track = CrossIntersection(**next(r["track"] for r in results if r["track"]["cone_spacing_m"] == spacing))
        h = track.width_m/2
        for rectangle in track.road_rectangles():
            axis.add_patch(Rectangle(rectangle[:2], *rectangle[2:], facecolor="#f1f5f9"))
        for colour, display in ((ConeColour.BLUE, "#2563eb"), (ConeColour.YELLOW, "#d69c00")):
            cones = [(x, y) for x, y, c in track.cones() if c is colour]
            axis.scatter(*np.asarray(cones).T, s=35, c=display, marker="^")
        row = sorted(x for x, y, _ in track.cones() if y == h and x <= -h+1e-9)
        start, end = row[-2:]
        axis.annotate("", (end, h+.45), (start, h+.45), arrowprops={"arrowstyle": "<->", "color": "#334155"})
        axis.plot([start, start], [h+.05, h+.5], color="#64748b", linewidth=.8)
        axis.plot([end, end], [h+.05, h+.5], color="#64748b", linewidth=.8)
        axis.text((start+end)/2, h+.65, f"{spacing:g} m", ha="center")
        axis.set(title=f"{spacing:g} m between neighbouring cones | road width {track.width_m:g} m",
                 xlim=(-6., 4.5), ylim=(-4.5, 4.5), aspect="equal", xlabel="World x (m)", ylabel="World y (m)")
        axis.grid(alpha=.15)
    figure.suptitle("Cone spacing along the rows; intersection corners stay fixed", fontsize=14)
    figure.tight_layout()
    figure.savefig(output/"cone_layout.png", dpi=150)
    plt.close(figure)
    figure, axes = plt.subplots(2, len(PROFILES), figsize=(20, 10))
    figure.subplots_adjust(left=.04, right=.99, bottom=.10, top=.87, hspace=.45, wspace=.3)
    for row, direction in enumerate(("left", "right")):
        for col, profile in enumerate(PROFILES):
            result = next(r for r in results if r["direction"] == direction and r["profile"] == profile
                          and r["track"]["cone_spacing_m"] == plot_spacing)
            draw(axes[row, col], result, result["frames"][-1], complete=True)
    figure.suptitle(f"ArUco-selected turns — 2.4 m cross, {plot_spacing:g} m between neighbouring cones", fontsize=14)
    legend = [Line2D([], [], color=c, linestyle=style, label=label) for c, style, label in (
        ("#334155", "-", "Actual simulated trajectory"), ("#94a3b8", ":", "Observed local lane fit"),
        ("#16a34a", "-", "Selected centre path used by MPC"))]
    figure.legend(handles=legend, loc="lower center", ncol=3, bbox_to_anchor=(.5, .015))
    figure.savefig(output/"turning.png", dpi=150)
    plt.close(figure)
    figure, axes = plt.subplots(1, 3, figsize=(15, 6.5))
    figure.subplots_adjust(left=.05, right=.99, bottom=.17, top=.76, wspace=.22)
    comparisons = [next(r for r in results if r["direction"] == d and r["profile"] == "nominal"
                        and r["track"]["cone_spacing_m"] == plot_spacing) for d in ("left", "right", "straight")]
    for axis, result in zip(axes, comparisons):
        selected = next((f for f in result["frames"] if f["selected_centre"] and f["phase"] in ("turn", "exit")),
                        result["frames"][-1])
        draw(axis, result, selected)
    figure.suptitle("ArUco left/right/straight command scenarios on the same crossing", fontsize=13)
    figure.legend(handles=legend, loc="lower center", ncol=3, fontsize=9, bbox_to_anchor=(.5, .015))
    figure.savefig(output/"signal_selection.png", dpi=150)
    plt.close(figure)
    animated = [next(r for r in results if r["direction"] == d and r["profile"] == p
                     and r["track"]["cone_spacing_m"] == plot_spacing)
                for p in ("nominal", "turn_camera_gap") for d in ("left", "right")]
    figure, axes = plt.subplots(2, 2, figsize=(12, 11))
    figure.subplots_adjust(left=.06, right=.99, bottom=.06, top=.88, hspace=.35, wspace=.18)
    duration = max(r["frames"][-1]["time_s"] for r in animated)
    times = np.arange(0., duration+.01, .5)
    def update(time):
        for axis, result in zip(axes.flat, animated):
            frame = result["frames"][min(round(time/.1), len(result["frames"])-1)]
            draw(axis, result, frame)
        figure.suptitle(f"Gaps {plot_spacing:g} m | clean / camera-gap scenarios (loss triggers at 20° heading) | 5× playback", fontsize=12)
    animation = FuncAnimation(figure, update, frames=times, interval=100)
    animation.save(output/"turning.gif", writer=PillowWriter(fps=10), dpi=90)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"build/intersection-turning-20261026")
    parser.add_argument("--cone-spacings", type=float, nargs="+", default=[.2, .4], metavar="METRES",
                        help="Gaps along each row; the road width remains 2.4 m.")
    parser.add_argument("--plot-spacing", type=float, help="Which supplied gap to show in the figures (default: first).")
    parser.add_argument("--corner-anchored", action="store_true", help="Keep exact corner cones and space outwards symmetrically.")
    args = parser.parse_args()
    if any(not math.isfinite(s) or s <= 0. or s > 2. for s in args.cone_spacings):
        parser.error("Cone spacings must be finite and in (0, 2] metres")
    if len(set(args.cone_spacings)) != len(args.cone_spacings):
        parser.error("Cone spacings must be distinct")
    plot_spacing = args.cone_spacings[0] if args.plot_spacing is None else args.plot_spacing
    if plot_spacing not in args.cone_spacings:
        parser.error("Plot spacing must be one of the supplied cone spacings")
    args.output.mkdir(parents=True, exist_ok=True)
    params, detector = settings()
    if params["mpc"]["camera_yaw_offset_deg"] != 0.:
        raise ValueError("This camera fixture assumes the configured zero fixed camera yaw")
    hashes = {**source_hashes(), str(Path(__file__).relative_to(ROOT)): hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "tests/policy/intersection_identification.py": hashlib.sha256((ROOT/"tests/policy/intersection_identification.py").read_bytes()).hexdigest()}
    results = []
    for spacing in args.cone_spacings:
        for profile in PROFILES:
            for direction in ("left", "right"):
                result = rollout(direction, profile, 26026, track=track_for(spacing, corner_anchored=args.corner_anchored),
                                 navigation=True, record_frames=True)
                results.append(result)
                print(spacing, profile, direction, result["metrics"]["reason"],
                      "pass", result["metrics"]["navigation_pass"], flush=True)
    for profile in ("nominal", "noisy_cones"):
        results.append(rollout("straight", profile, 26026,
                              track=track_for(plot_spacing, corner_anchored=args.corner_anchored), navigation=True, record_frames=True))
    serialised = [{k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in r.items()} for r in results]
    with gzip.open(args.output/"runs.json.gz", "wt") as stream:
        json.dump(serialised, stream)
    for result in results:
        np.savetxt(args.output/f"spacing-{result['track']['cone_spacing_m']}-{result['profile']}-{result['direction']}.csv",
                   result["trajectory"], delimiter=",", header="time_s,x_m,y_m,heading_rad,speed_m_per_s,drive,steer,command", comments="")
    summaries = [{k: v for k, v in r.items() if k not in ("trajectory", "frames")} for r in serialised]
    report = {"scope": "Closed-loop MovementPolicy/MPC/CarController and CAR_20 kinematic plant; decoded sensors, no GPU pixels or ROS.",
              "source_sha256": hashes, "policy_parameters": params, "detector_parameters": detector,
              "effective_intersection_parameters": dict(params["intersection"], enabled=True),
              "assumptions": {"cross_width_m": 2.4, "cone_spacings_m": args.cone_spacings, "plot_spacing_m": plot_spacing,
                              "corner_anchored": args.corner_anchored, "turn_radius_m": params["intersection"]["turning_radius_m"],
                              "camera_half_fov_deg": 43., "camera_range_m": 2.5, "near_forward_m": .4,
                              "sensor_age_s": .2, "policy_period_s": .1, "plant_period_s": .01,
                              "turn_occlusion_trigger_abs_heading_deg": 20., "occlusion_durations_s": TURN_LOSS_DURATIONS_S},
              "limitations": ["Synthetic ideal three-return cone lidar without occlusion; no actuator deadtime, tyre slip or measured braking.",
                              "The optional navigator is enabled only for these local simulations; deployment configuration is not changed.",
                              "Route formation uses an equal-width perpendicular-cross assumption after sensor admission.",
                              "Observed cone-fit arcs and selected current-body route are separate; selected routes are not detected boundaries.",
                              "Physical cone colours remain fixed; the existing completed-exit handoff reacquires sides with AUTO.",
                              "Equal seeds reproduce each case; noise samples are not paired across different cone counts.",
                              "Goal crossing is sustained moving exit alignment, not a stopped goal or physical qualification."],
              "case_count": len(results), "policy_frames": sum(len(r["frames"]) for r in results),
              "navigation_passes": sum(r["metrics"]["navigation_pass"] for r in results),
              "reasons": dict(Counter(r["metrics"]["reason"] for r in results)),
              "stop_reasons": dict(Counter(r["metrics"]["policy_stop"] for r in results if r["metrics"]["policy_stop"])),
              "disturbance_cases": sum("disturbance" in r for r in results),
              "disturbance_triggered_cases": sum(r.get("disturbance", {}).get("started_at_s") is not None for r in results),
              "runs": summaries}
    figures(results, args.output, plot_spacing)
    for path, expected in hashes.items():
        if hashlib.sha256((ROOT/path).read_bytes()).hexdigest() != expected:
            raise ValueError(f"Executed source changed during simulation: {path}")
    (args.output/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({k: report[k] for k in ("case_count", "policy_frames", "navigation_passes", "reasons")}, indent=2))


if __name__ == "__main__":
    main()
