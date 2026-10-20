"""Compare fixed and turn-aware cameras without selecting a physical pan policy.

OPENBLAS_NUM_THREADS=1 .venv/bin/python -m tests.policy.camera_pan_simulation
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path

from tests.policy.angular_lanes import AngularUTurn
from tests.policy.car21_uturn_simulation import case, controller
from tests.policy.curved_lanes import run_curve
from tests.policy.panning_camera import PanningCamera


SOURCES = ("tests/policy/camera_pan_simulation.py", "tests/policy/panning_camera.py",
           "tests/policy/car21_uturn_simulation.py", "tests/policy/curved_lanes.py",
           "tests/policy/angular_lanes.py", "policy/action_policy/corridor_experiment.py",
           "policy/lane_detection/corridor.py", "policy/memory.py")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("build/camera-pan-comparison"))
    parser.add_argument("--seed-start", type=int, default=501)
    parser.add_argument("--placements", type=int, nargs="+", default=[17, 89])
    modes = ("fixed", "pan25", "path_pan25", "path_pan25_uncorrected")
    parser.add_argument("--modes", choices=modes, nargs="+", default=list(modes))
    args = parser.parse_args()
    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    hashes = {p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in SOURCES}
    report = {"source_sha256": hashes, "assumptions": {
        "profile": "Owner provisional width1m/left radius1m; inherited right .92/m; 3m legs/cross",
        "speed_m_per_s": .4, "camera_latency_s": .2, "camera_noise_m": .03,
        "pan_cap_deg": 25, "preview_m": .6, "slew_deg_per_s": 60, "lag_s": .15,
        "pan_status": "Unmeasured simulation assumptions; planar pivot at body origin; no image blur or physical occlusion",
        "stopped_servo": "Retains last target, continuing bounded motion toward it; no recenter command",
        "feedback": "Requested curvature, or 2m preview of the policy's cone-derived path; no true route/pose enters target",
        "path_pan_lookahead_m": 2.,
        "coordinates": "Each image carries acquisition pan through delay, corrected into acquisition base_link",
        "turn_visibility_definition": "Both colours visible; frames with absolute requested curvature > .3/m, not lane-fit validity",
        "occlusion_s": [6., 6.4], "placements": args.placements,
        "noise_seeds": list(range(args.seed_start, args.seed_start+5)),
    }, "cases": {}, "summary": {}}
    representatives = {}
    for placement in args.placements:
        for seed in range(args.seed_start, args.seed_start+5):
            for occluded in (False, True):
                label = f"placement{placement}_seed{seed}" + ("_occluded" if occluded else "")
                scenario = replace(case(seed=seed, occlusion=occluded), lane=AngularUTurn(width_m=1., seed=placement))
                for mode in args.modes:
                    camera = PanningCamera(max_pan_deg=0. if mode == "fixed" else 25.,
                                           compensate=not mode.endswith("uncorrected"),
                                           path_lookahead_m=2. if mode.startswith("path_") else None)
                    result = run_curve(scenario, controller(), camera=camera)
                    report["cases"][f"{label}/{mode}"] = {
                        **result.metrics, "lane_kept": result.lane_kept, **camera.metrics()}
                    if placement == args.placements[0] and seed == args.seed_start+4 and not occluded:
                        representatives[mode] = result, camera
                    print(label, mode, result.lane_kept, camera.metrics(), flush=True)
                    (out/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    for mode in args.modes:
        cases = [r for name, r in report["cases"].items() if name.endswith("/"+mode)]
        frames = sum(r["turn_camera_frames"] for r in cases)
        report["summary"][mode] = {
            "passed": sum(r["lane_kept"] for r in cases), "total": len(cases),
            "cone_contacts": sum(r["cones_touched"] for r in cases),
            "worst_footprint_outside_m": max(r["max_outside_m"] for r in cases),
            "turn_camera_frames": frames,
            "turn_both_colours_fraction": sum(r["turn_both_colours_fraction"]*r["turn_camera_frames"] for r in cases)/max(1, frames),
            "turn_mean_cones": sum(r["turn_mean_cones"]*r["turn_camera_frames"] for r in cases)/max(1, frames),
        }
    assert hashes == {p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in SOURCES}, "Source changed during evaluation"
    report["limitations"] = "Offline simulated pan only. Fixed runtime pan and physical services unchanged. Requires measured acquisition-time optical TF and calibrated servo limits before physical driving. Default scoring unchanged. Uncorrected mode is a deliberate frame-error control."
    (out/"report.json").write_text(json.dumps(report, indent=2)+"\n")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plot_modes = [m for m in args.modes if m != "pan25"] or args.modes
    fig, axes = plt.subplots(1, len(plot_modes), figsize=(4.7*len(plot_modes), 4.5), squeeze=False)
    lane = AngularUTurn(width_m=1., seed=args.placements[0])
    colours = {"fixed": "slateblue", "pan25": "darkorange", "path_pan25": "seagreen", "path_pan25_uncorrected": "indianred"}
    titles = {"fixed": "Fixed camera", "pan25": "Pan following steering",
              "path_pan25": "Pan previewing observed path",
              "path_pan25_uncorrected": "Path pan with fixed TF (error control)"}
    for axis, mode in zip(axes[0], plot_modes):
        polygon = lane._corridor_polygon()
        axis.fill(*zip(*polygon), color="lightgrey", alpha=.3)
        for colour, _, x, y in lane.cones():
            axis.plot(x, y, ".", color="royalblue" if colour.name == "BLUE" else "goldenrod")
        result, _ = representatives[mode]
        axis.plot(result.x, result.y, color=colours[mode])
        axis.set_title(titles[mode])
        axis.set_aspect("equal")
        axis.set_xlabel("world x (m)")
        axis.set_ylabel("world y (m)")
    fig.suptitle(f"Same three-line U, noise seed{args.seed_start+4}; provisional 1m width/radius, .4m/s")
    fig.tight_layout()
    fig.savefig(out/"trajectories.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    positive_modes = [m for m in args.modes if not m.endswith("uncorrected")]
    for mode in positive_modes:
        _, camera = representatives[mode]
        axes[0].plot([s["time_s"] for s in camera.samples], [s["pan_deg"] for s in camera.samples],
                     color=colours[mode], label=titles[mode])
    axes[0].set_xlabel("camera acquisition time (s)")
    axes[0].set_ylabel("simulated pan left (degrees)")
    axes[0].legend()
    labels = ["Fixed" if m == "fixed" else "Path preview" if m == "path_pan25" else "Steering pan" for m in positive_modes]
    values = [100*report["summary"][m]["turn_both_colours_fraction"] for m in positive_modes]
    bars = axes[1].bar(labels, values, color=[colours[m] for m in positive_modes])
    axes[1].bar_label(bars, labels=[f"{v:.1f}%" for v in values], padding=3)
    axes[1].set_ylim(0, 110)
    axes[1].set_ylabel("turn camera frames with both colours visible (%)")
    axes[1].set_title(f"{len(args.placements)*10} cases per mode, frame-weighted")
    fig.tight_layout()
    fig.savefig(out/"visibility.png", dpi=150)
    plt.close(fig)
    print(json.dumps(report["summary"], indent=2), flush=True)


if __name__ == "__main__":
    main()
