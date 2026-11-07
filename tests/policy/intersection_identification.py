"""Offline identification-only diagnostic using the current cone checks, association and lane fitter.

Run with OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/ai4r-mpl .venv/bin/python -m \
    tests.policy.intersection_identification --output build/intersection-association-20261026 \
    --baseline build/intersection-identification-20261026

The camera generates decoded cone observations, not pixels. Scripted poses ensure the junction
is examined even when a driving policy would stop. World cone identities are scoring/drawing
metadata only. No movement controller, chosen exit, simulator pose or branch identity enters
the lane detector. This does not establish GPU image recognition or navigation success.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, dataclass
import gzip
import hashlib
import json
import math
from pathlib import Path
import subprocess

import numpy as np

from policy.action_policy.reference import rotate_cones
from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones
from policy.input_output import CarObservations, ConeBatch, ConeColour, ConeDetection, PolicyState, Position, SensorAge
from policy.lane_detection.lane_detection import LaneDetectionConfig, LaneDetector, _fit_lane_edges
from tests.policy.intersection_simulation import CrossIntersection, body_point, settings

ROOT = Path(__file__).resolve().parents[2]
PERIOD_S, SPEED_M_PER_S, DELAY_STEPS = .1, .4, 2
START_X, END_X = -5., 2.
PROFILES = ("clean", "noise_and_misses", "blue_boundary_missing", "corner_cones_missing", "all_cones_missing")
STAMP_BASE_NS = 1_800_000_000_000_000_000


@dataclass(frozen=True)
class CameraModel:
    """Assumed camera visibility, not a measured hardware calibration."""
    range_m: float = 3.5
    near_forward_m: float = .4
    half_fov_deg: float = 43.
    measurement_age_s: float = DELAY_STEPS * PERIOD_S
    acquisition_to_publish_s: float = .08


def world_point(x, y, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return pose[0] + c*x-s*y, pose[1] + s*x+c*y


def physical_cones(track, include_cross_road=True):
    half = track.width_m/2
    return [p for p in track.cones() if include_cross_road or abs(abs(p[1])-half) < 1e-8]


def synthetic_frame(track, pose, camera, profile, rng, yaw_deg=0., include_cross_road=True):
    """Return policy observations and a separate scorer-only identity map."""
    cones, truth = [], {}
    half = track.width_m/2
    camera_pose = (*pose[:2], pose[2]+math.radians(yaw_deg))
    for index, (x, y, colour) in enumerate(physical_cones(track, include_cross_road)):
        forward, left = body_point(x, y, camera_pose)
        if (forward < camera.near_forward_m or math.hypot(forward, left) > camera.range_m
                or abs(math.atan2(left, forward)) > math.radians(camera.half_fov_deg)):
            continue
        if profile == "noise_and_misses" and rng.random() < .2:
            continue
        if profile == "blue_boundary_missing" and -2. <= pose[0] <= .6 and colour is ConeColour.BLUE:
            continue
        if profile == "corner_cones_missing" and max(abs(x), abs(y)) < half+.7:
            continue
        if profile == "all_cones_missing" and -1.5 <= pose[0] <= -.8:
            continue
        noise = rng.normal(0., .02, 2) if profile == "noise_and_misses" else np.zeros(2)
        cone = ConeDetection(Position(forward+noise[0], left+noise[1], .1), colour, .9)
        cones.append(cone)
        # This map never becomes a field on a ConeDetection or CarObservations.
        truth[id(cone)] = {"id": index, "axis": "incoming/straight" if abs(abs(y)-half) < 1e-8 else "cross-road"}
    return cones, truth


def identify(raw, truth, sensor_age, publish_latency_s, policy_state, detector, filter_config, yaw_deg=0.):
    batch = ConeBatch(raw, sensor_age, publish_latency_s)
    batch = filter_cones(batch, filter_config)
    if yaw_deg:
        rotated = rotate_cones(batch, math.radians(yaw_deg))
        truth = {id(new): truth[id(old)] for old, new in zip(batch, rotated) if id(old) in truth}
        batch = ConeBatch(rotated, batch.sensor_age, batch.acquisition_to_publish_latency_s)
    observations = CarObservations(batch, None, None, None, None, policy_state)
    lane = detector.detect(observations)
    # Diagnostic-only second call; membership cannot be recovered from final residual alone.
    fit = _fit_lane_edges({c: [p for p in batch if p.colour is c] for c in ConeColour}, detector.config)
    inliers = set() if fit is None else {id(p) for row in fit[2].values() for p in row}
    members = Counter(truth[id(p)]["axis"] for p in batch if id(p) in inliers and id(p) in truth)
    candidate = None if fit is None else {
        "heading_deg": math.degrees(fit[0].heading_rad), "curvature_per_m": fit[0].curvature_per_m,
        "offsets_m": {c.name: value for c, value in fit[1].items()}}
    return lane, batch, inliers, members, candidate, truth


def geometric_score(lane, acquisition_pose, width_m):
    half = width_m/2
    phase = ("approach" if acquisition_pose[0]+.3 < -half else
             "departure" if acquisition_pose[0]-.3 > half else "junction: multiple arms")
    if not lane.has_lane:
        return {"phase": phase, "heading_error_deg": None, "centre_offset_m": None,
                "aligned_straight_arm": False if phase != "junction: multiple arms" else None}
    world_heading = acquisition_pose[2] + lane.centre_line.heading_rad
    error = math.degrees(math.atan2(math.sin(world_heading), math.cos(world_heading)))
    centre_y = world_point(lane.centre_line.x, lane.centre_line.y, acquisition_pose)[1]
    return {"phase": phase, "heading_error_deg": error, "centre_offset_m": centre_y,
            "aligned_straight_arm": None if phase == "junction: multiple arms" else abs(error) <= 15 and abs(centre_y) <= .2}


def sequence(width_m, spacing_m, camera, profile, seed, include_cross_road=True):
    params, _ = settings()
    config = LaneDetectionConfig(**params["lane_detection"])
    detector, filter_config = LaneDetector(config), ConeFilterConfig(**params["cone_filter"])
    track = CrossIntersection(width_m=width_m, arm_length_m=6., cone_spacing_m=spacing_m)
    yaw_deg = params["mpc"]["camera_yaw_offset_deg"]
    rng = np.random.default_rng(seed)
    history, frames, previous = [], [], None
    count = round((END_X-START_X)/(SPEED_M_PER_S*PERIOD_S))+1
    for tick in range(count):
        pose = (START_X + SPEED_M_PER_S*PERIOD_S*tick, 0., 0.)
        history.append((pose, *synthetic_frame(track, pose, camera, profile, rng, yaw_deg, include_cross_road)))
        if tick < DELAY_STEPS:
            continue
        acquisition_pose, raw, truth = history[tick-DELAY_STEPS]
        step = tick-DELAY_STEPS
        stamp = STAMP_BASE_NS+round(step*PERIOD_S*1e9)
        lane, batch, inliers, members, candidate, truth = identify(
            raw, truth, SensorAge(camera.measurement_age_s, stamp), camera.acquisition_to_publish_s,
            PolicyState(PERIOD_S if step else 0., tick*PERIOD_S, step == 0), detector, filter_config, yaw_deg)
        score = geometric_score(lane, acquisition_pose, width_m)
        jump = None
        if lane.has_lane and previous is not None:
            difference = lane.centre_line.heading_rad-previous
            jump = abs(math.degrees(math.atan2(math.sin(difference), math.cos(difference))))
        previous = lane.centre_line.heading_rad if lane.has_lane else None
        frame = {"time_s": tick*PERIOD_S, "pose": pose, "acquisition_pose": acquisition_pose,
                 "raw_count": len(raw), "filtered_count": len(batch), "fit_inliers": len(inliers),
                 "fit_rejected": len(batch)-len(inliers) if candidate is not None else None,
                 "status": lane.status.name, "width_m": lane.lane_width_m,
                 "curvature_per_m": lane.path_curvature_per_m, "candidate": candidate,
                 "inlier_branches": dict(members), "mixed_branches": len(members) > 1,
                 "heading_jump_deg": jump, **score,
                 "raw_detections": [{"x": p.pos.x, "y": p.pos.y, "colour": p.colour.name,
                                     "stage": "measured"} for p in rotate_cones(raw, math.radians(yaw_deg))],
                 "detections": [{"x": p.pos.x, "y": p.pos.y, "colour": p.colour.name,
                                 "stage": "inlier" if id(p) in inliers else "unused",
                                 "truth_axis": truth.get(id(p), {}).get("axis")} for p in batch],
                 "arcs": {}, "centre": []}
        if lane.has_lane:
            for side, edge in (("left", lane.left_edge), ("right", lane.right_edge)):
                if edge is not None:
                    frame["arcs"][side] = [edge.arc.point_at(s) for s in np.linspace(edge.nearest_m, edge.farthest_m, 70)]
            if lane.visible_until_m > 0:
                frame["centre"] = [lane.centre_line.point_at(s) for s in np.linspace(0, min(4., lane.visible_until_m), 70)]
        frames.append(frame)
    approach = [f for f in frames if f["phase"] == "approach"]
    valid = [f for f in approach if f["status"] != "NO_LANE"]
    return {"width_m": width_m, "spacing_m": spacing_m, "camera": asdict(camera), "profile": profile, "seed": seed,
            "include_cross_road": include_cross_road, "camera_yaw_offset_deg": yaw_deg,
            "metrics": {"frames": len(frames), "lane_status": dict(Counter(f["status"] for f in frames)),
                        "approach_frames": len(approach), "approach_usable_fits": len(valid),
                        "approach_aligned_fits": sum(f["aligned_straight_arm"] is True for f in approach),
                        "approach_valid_but_misaligned": sum(f["aligned_straight_arm"] is False for f in valid),
                        "approach_no_lane": sum(f["status"] == "NO_LANE" for f in approach),
                        "approach_mixed_branch_fits": sum(f["mixed_branches"] for f in valid),
                        "consecutive_heading_jumps_over_30deg": sum((f["heading_jump_deg"] or 0) > 30 for f in frames)},
            "frames": frames}


def recorded_samples():
    archive = ROOT/"build/car-no-motion-20261023/ai4r-no-motion-v2-20261023.json.gz"
    if not archive.exists():
        return {"available": False, "reason": "Historical capture absent; synthetic run is independent."}
    params, _ = settings()
    output = []
    with gzip.open(archive, "rt") as stream:
        runs = json.load(stream)
    for stem, indices in (("2026-10-23_17-17-13", (0,)), ("2026-10-23_17-21-05", (0,)),
                           ("2026-10-23_17-21-18", (18, 19, 20))):
        run = next(r for r in runs if r["stem"] == stem)
        detector = LaneDetector(LaneDetectionConfig(**params["lane_detection"]))
        for index in range(max(indices)+1):
            obs = CarObservations.deserialise(run["observations"][index])
            lane, batch, inliers, _, candidate, _ = identify(
                obs.cones, {}, obs.cones.sensor_age, obs.cones.acquisition_to_publish_latency_s, obs.policy,
                detector, ConeFilterConfig(**params["cone_filter"]), params["mpc"]["camera_yaw_offset_deg"])
            if index in indices:
                output.append({"stem": stem, "index": index, "raw": len(obs.cones), "filtered": len(batch),
                               "candidate_inliers": len(inliers), "status": lane.status.name,
                               "candidate": candidate, "width_m": lane.lane_width_m})
    return {"available": True, "archive_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
            "scope": "Historical cone frames evaluated with today's local filter/fitter; no labelled road ground truth.",
            "samples": output}


def draw_frame(axis, frame, result, title):
    from matplotlib.patches import Polygon, Rectangle, Wedge
    axis.clear()
    half = result["width_m"]/2
    axis.add_patch(Rectangle((-6., -half), 12., 2*half, color="#f1f5f9", zorder=0))
    axis.add_patch(Rectangle((-half, -6.), 2*half, 12., color="#f1f5f9", zorder=0))
    track = CrossIntersection(width_m=result["width_m"], arm_length_m=6., cone_spacing_m=result["spacing_m"])
    world = physical_cones(track, result["include_cross_road"])
    axis.scatter([p[0] for p in world], [p[1] for p in world], s=14, c="#cbd5e1", marker="^")
    pose = frame["acquisition_pose"]
    camera = result["camera"]
    camera_heading = math.degrees(pose[2])+result["camera_yaw_offset_deg"]
    axis.add_patch(Wedge(pose[:2], camera["range_m"], camera_heading-camera["half_fov_deg"],
                         camera_heading+camera["half_fov_deg"],
                         facecolor="#dbeafe", alpha=.2, zorder=1))
    for colour, display in (("BLUE", "#2563eb"), ("YELLOW", "#d69c00")):
        items = [p for p in frame["detections"] if p["colour"] == colour]
        coords = np.asarray([world_point(p["x"], p["y"], pose) for p in items]).reshape(-1, 2)
        axis.scatter(coords[:, 0], coords[:, 1], s=38, c=display, marker="^", zorder=4)
    for stage, colour in (("inlier", "#16a34a"), ("unused", "#dc2626")):
        coords = np.asarray([world_point(p["x"], p["y"], pose) for p in frame["detections"] if p["stage"] == stage]).reshape(-1, 2)
        axis.scatter(coords[:, 0], coords[:, 1], s=95, facecolors="none", edgecolors=colour, linewidths=1.3, zorder=5)
    for side, points in frame["arcs"].items():
        coords = np.array([world_point(*p, pose) for p in points])
        axis.plot(coords[:, 0], coords[:, 1], color="#2563eb" if side == "left" else "#b58900", linewidth=1.5)
    if frame["centre"]:
        coords = np.array([world_point(*p, pose) for p in frame["centre"]])
        axis.plot(coords[:, 0], coords[:, 1], color="#ea580c", linewidth=2.5)
    x, y, _ = frame["pose"]
    axis.add_patch(Rectangle((x-.3, y-.125), .6, .25, facecolor="#334155", zorder=7))
    axis.annotate("", (x+.48, y), (x, y), arrowprops={"arrowstyle": "->", "color": "#334155"})
    text = (frame.get("stage_label", f"{frame['status']} | {frame['filtered_count']} filtered / {frame['fit_inliers']} candidate inliers")+"\n"
            f"acquisition x={pose[0]:+.2f} m | {frame['phase']}")
    if frame["heading_error_deg"] is not None:
        text += f"\nworld heading {frame['heading_error_deg']:+.1f}° | centre {frame['centre_offset_m']:+.2f} m"
    elif frame["candidate"] is not None:
        text += f"\nrejected candidate heading {frame['candidate']['heading_deg']:+.1f}°"
    axis.set_title(title+"\n"+text, fontsize=9)
    axis.set(xlim=(-5.6, 3.8), ylim=(-3.2, 3.2), xlabel="World forward / east (m)", ylabel="World left / north (m)", aspect="equal")
    axis.grid(alpha=.15)


def plots(nominal, disturbed, output, baseline=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.lines import Line2D
    figure, axes = plt.subplots(2, 3, figsize=(15, 8), constrained_layout=True)
    for axis, (result, target, title) in zip(axes.flat, (
            (nominal, -4.8, "1. Straight approach"), (nominal, -3.5, "2. Cross-road cones enter view"),
            (nominal, -1.5, "3. Near the junction"), (nominal, -.5, "4. Junction entry"),
            (nominal, .1, "5. Junction centre"), (disturbed, -1.5, "6. One boundary missing"))):
        frame = min(result["frames"], key=lambda f: abs(f["acquisition_pose"][0]-target))
        draw_frame(axis, frame, result, title)
    figure.suptitle("Current local lane formation at a four-way cross — scripted sensor poses", fontsize=14)
    legend = [Line2D([], [], marker="o", linestyle="none", markerfacecolor="none", markeredgecolor=c, label=l)
              for c, l in (("#16a34a", "Candidate-fit inlier"), ("#dc2626", "Not used by candidate fit"))]
    legend += [Line2D([], [], color="#ea580c", label="Returned lane centre")]
    figure.legend(handles=legend, loc="outside lower center", ncol=3, fontsize=9)
    figure.savefig(output/"identification.png", dpi=150)
    plt.close(figure)
    figure, axes = plt.subplots(1, 3, figsize=(15, 5), constrained_layout=True)
    frame = min(nominal["frames"], key=lambda f: abs(f["acquisition_pose"][0]+3.5))
    for axis, stage in zip(axes, ("raw", "grouped", "lane")):
        current = dict(frame)
        if stage != "lane":
            current.update(arcs={}, centre=[], heading_error_deg=None, candidate=None)
        if stage == "raw":
            current.update(detections=frame["raw_detections"], stage_label=f"{frame['raw_count']} synthetic cone measurements")
        elif stage == "grouped":
            current["stage_label"] = f"{frame['filtered_count']} checked cones / {frame['fit_inliers']} fit members"
        draw_frame(axis, current, nominal, {"raw": "1. GPU-style decoded detections",
                                           "grouped": "2. Position checks and boundary grouping",
                                           "lane": "3. Lane formation"}[stage])
    figure.suptitle("Updated processing after the GPU detector — synthetic positions, unchanged camera model", fontsize=12)
    figure.legend(handles=legend, loc="outside lower center", ncol=3, fontsize=9)
    figure.savefig(output/"stages.png", dpi=150)
    plt.close(figure)
    figure, axes = plt.subplots(1, 2 if baseline else 1, figsize=(16 if baseline else 9.4, 6.4),
                                constrained_layout=True, squeeze=False)
    indices = list(range(0, len(nominal["frames"]), 3))
    def update(index):
        frame = nominal["frames"][index]
        if baseline:
            draw_frame(axes[0, 0], baseline["frames"][index], baseline, "Before: cones grouped only by colour")
        draw_frame(axes[0, -1], frame, nominal, "After: checked cones and incoming-arm association")
        figure.suptitle("Grey: physical cones | colour: filtered detections | rings: candidate-fit membership", fontsize=10)
    animation = FuncAnimation(figure, update, frames=indices, interval=100)
    animation.save(output/("comparison.gif" if baseline else "identification.gif"), writer=PillowWriter(fps=10), dpi=90)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"build/intersection-association-20261026")
    parser.add_argument("--baseline", type=Path, help="Prior report/frame directory for the same scenario matrix.")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for width in (1., 1.4, 2.4):
        for spacing in (.4, .6):
            for view_range in (2.5, 3.5):
                for profile in PROFILES:
                    for seed in ((26000, 26001) if profile == "noise_and_misses" else (26000,)):
                        results.append(sequence(width, spacing, CameraModel(range_m=view_range), profile, seed))
    nominal = next(r for r in results if r["width_m"] == 1.4 and r["spacing_m"] == .4
                   and r["camera"]["range_m"] == 3.5 and r["profile"] == "clean")
    disturbed = next(r for r in results if r["width_m"] == 1.4 and r["spacing_m"] == .4
                     and r["camera"]["range_m"] == 3.5 and r["profile"] == "blue_boundary_missing")
    control = sequence(1.4, .4, CameraModel(), "clean", 26000, include_cross_road=False)
    params, _ = settings()
    paths = sorted((ROOT/"policy").rglob("*.py"))+[
        ROOT/"config/ai4r_policy.yaml", ROOT/"config/aruco_detector.yaml",
        ROOT/"tests/policy/intersection_simulation.py", Path(__file__)]
    report = {"scope": "2026-10-26 local working-tree ArUco branch; decoded-cone identification only",
              "branch": subprocess.check_output(["git", "branch", "--show-current"], cwd=ROOT, text=True).strip(),
              "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
              "config": {**{k: params[k] for k in ("cone_filter", "lane_detection")},
                         "camera_yaw_offset_deg": params["mpc"]["camera_yaw_offset_deg"]},
              "assumptions": {"camera": asdict(CameraModel()), "period_s": PERIOD_S, "scripted_speed_m_per_s": SPEED_M_PER_S,
                              "widths_m": [1., 1.4, 2.4], "cone_spacings_m": [.4, .6], "camera_ranges_m": [2.5, 3.5],
                              "scripted_poses": {"start_x_m": START_X, "end_x_m": END_X,
                                                 "lateral_offset_m": 0., "heading_deg": 0.},
                              "approach_alignment_thresholds": {"heading_deg": 15, "centre_offset_m": .2}},
              "limitations": ["No camera pixels, neural GPU detector, measured occlusion or depth failures are simulated.",
                              "The lane fitter returns one lane, not a junction class or multiple exits.",
                              "The scripted pose sweep is not a driving simulation or evidence of navigation success.",
                              "Synthetic branch identities are scoring-only; no expected path is passed to the detector.",
                              "Inside the junction, no unique straight lane is asserted as ground truth.",
                              "Only a centred straight pose sweep is tested: no rotated approach or turning trajectory.",
                              "Alignment measures local heading/centre only, not width or full boundary correctness.",
                              "This is not a check of any current physical-car deployment or the captured LocalMap wrapper."],
              "scenario_count": len(results), "frame_count": sum(r["metrics"]["frames"] for r in results),
              "nominal": {k:v for k,v in nominal.items() if k != "frames"},
              "control": {"scope": "Same straight rows and junction gap, with cross-road cones removed.",
                          **{k:v for k,v in control.items() if k != "frames"}},
              "results": [{k:v for k,v in r.items() if k != "frames"} for r in results],
              "recorded_samples": recorded_samples()}
    baseline_nominal = None
    if args.baseline:
        before = json.loads((args.baseline/"report.json").read_text())
        with gzip.open(args.baseline/"frames.json.gz", "rt") as stream:
            archived = json.load(stream)["scenarios"]
        key = lambda r: (r["width_m"], r["spacing_m"], r["camera"]["range_m"], r["profile"], r["seed"])
        if [key(r) for r in archived] != [key(r) for r in results]:
            raise ValueError("Baseline scenarios differ; cannot claim a paired comparison")
        for old, new in zip(archived, results):
            if old["camera"] != new["camera"] or len(old["frames"]) != len(new["frames"]):
                raise ValueError("Baseline camera or frame count differs")
            if any(a["acquisition_pose"] != list(b["acquisition_pose"]) or a["raw_count"] != b["raw_count"]
                   or a["time_s"] != b["time_s"] for a, b in zip(old["frames"], new["frames"])):
                raise ValueError("Baseline acquisition poses, timing or raw counts differ")
        baseline_nominal = next(r for r in archived if key(r) == key(nominal))
        if [f["acquisition_pose"] for f in baseline_nominal["frames"]] != [list(f["acquisition_pose"]) for f in nominal["frames"]]:
            raise ValueError("Baseline poses differ; cannot compare animation frames")
        keys = ("approach_frames", "approach_aligned_fits", "approach_valid_but_misaligned", "approach_no_lane")
        report["comparison"] = {
            "baseline_report_sha256": hashlib.sha256((args.baseline/"report.json").read_bytes()).hexdigest(),
            "baseline_frames_sha256": hashlib.sha256((args.baseline/"frames.json.gz").read_bytes()).hexdigest(),
            "baseline_source_sha256": before["source_sha256"], "nominal_before": before["nominal"]["metrics"],
            "nominal_after": nominal["metrics"],
            "by_profile": {profile: {label: {k: sum(r["metrics"][k] for r in items if r["profile"] == profile) for k in keys}
                                     for label, items in (("before", archived), ("after", results))} for profile in PROFILES}}
    (args.output/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    with gzip.open(args.output/"frames.json.gz", "wt") as stream:
        json.dump({"scenarios": results, "control": control}, stream)
    plots(nominal, disturbed, args.output, baseline_nominal)
    print(json.dumps({"scenarios": len(results), "frames": report["frame_count"], "nominal": nominal["metrics"],
                      "control": control["metrics"],
                      "output": str(args.output.resolve())}, indent=2))


if __name__ == "__main__":
    main()
