"""Makes the figures and result files in Thanh Long's logbook section (docs/logbook/long/).

    python3 docs/logbook/long/make_figures.py                          # lane estimation figures (needs Git LFS data)
    python3 docs/logbook/long/make_figures.py --sim-repo <checkout>    # also re-run the simulator (about 10 min)

The lane estimation figures run policy/lane_detection/ from this checkout (the logbook branch, whose policy code is
`dev` at 37b7b39) on drawn lanes and on the recorded drives in scenarios/data/real_scenarios/, which are Git LFS files:
run `git lfs pull` first. The simulator figures run tests/policy/simulator/ from --sim-repo, a checkout of
origin/feature/car-simulator (6c4cd17), in a separate process, so its policy code is the one the simulator was
published with (d1d8096); the runs are saved in data/simulator_runs_full.json, and without --sim-repo the simulator
figures are redrawn from that file. Every number quoted in long-nguyen.md is written to data/*.json by this script.

The viewer screenshots in figures/08_simulator_viewer.png come from the simulator's browser viewer: take them with
viewer_screenshots.js, then pass their folder as --viewer-shots. Without it the existing figure is kept.
"""
import argparse
import dataclasses
import json
import math
import statistics
import subprocess
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402

HERE = Path(__file__).resolve().parent
OUT = HERE / "figures"
DATA = HERE / "data"
REPO = HERE.parents[2]
BAD, GOOD, NEUTRAL, ACCENT, WARN = "#c8553d", "#2a7f62", "#8a8a8a", "#1f5fd1", "#b26b00"
BLUE_CONE, YELLOW_CONE = "#1f5fd1", "#e0a800"
plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False, "savefig.dpi": 150})

STRAIGHT_RECORDINGS = ("straight_even", "messy_cone", "outlier_cone", "long_track")
"""Recorded 6 Oct (car not named in the files), drive 0.55, steering 0, camera pan never commanded."""
AFTER_FIX_RECORDINGS = ("long_block_in_way", "short_block_in_way", "fail_obstacle_track_thins", "fail_complex_course")
"""Recorded 13 Oct with the policy holding the pan at 0 (camera_pan_action 0.0 in every step). The lanes start
straight. fail_two_obstacles_curve is left out: its steps are identical to short_block_in_way."""


def save(figure, name):
    OUT.mkdir(parents=True, exist_ok=True)
    figure.savefig(OUT / name, bbox_inches="tight")
    plt.close(figure)
    print("wrote", OUT / name)


def write_json(name, value):
    DATA.mkdir(parents=True, exist_ok=True)
    (DATA / name).write_text(json.dumps(value, indent=1, sort_keys=True) + "\n")
    print("wrote", DATA / name)


def box(axis, x, y, w, h, text, mine=False, grey=False, size=8.3, dashed=False):
    axis.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.03",
                                  fc="#eef1f5" if grey else ("#eef4ff" if mine else "white"),
                                  ec=ACCENT if mine else NEUTRAL, lw=2.2 if mine else 1.1,
                                  ls="--" if dashed else "-"))
    axis.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=size)


def arrow(axis, start, end, text="", colour="black", lw=1.2, offset=(0, 0.08), size=7.5, ha="center"):
    axis.annotate("", end, start, arrowprops=dict(arrowstyle="->", lw=lw, color=colour))
    if text:
        axis.text((start[0] + end[0]) / 2 + offset[0], (start[1] + end[1]) / 2 + offset[1], text, ha=ha,
                  fontsize=size, color="#444" if colour == "black" else colour)


# ----------------------------------------------------------------------------------------------------------------
# Standard 1 and 2 diagrams
# ----------------------------------------------------------------------------------------------------------------
def system_diagram():
    """Where lane estimation, the lane overlay, the pan hold and the simulator sit in the whole system."""
    figure, axis = plt.subplots(figsize=(13.4, 6.2))
    axis.set_xlim(0, 13.4)
    axis.set_ylim(0, 6.2)
    axis.axis("off")
    axis.text(0.05, 6.05, "Policy step once per cone batch (10 Hz); whole step <= 20 ms at p99 (R6a).   Grey = DREAM / hardware,  "
                          "white = teammates,  blue outline = what I built", fontsize=8.5, color="#444")
    # DREAM and hardware
    box(axis, 0.05, 4.45, 1.75, 0.95, "OAK-D camera +\ncone detector\n(DREAM)", grey=True)
    box(axis, 0.05, 3.15, 1.75, 0.95, "ArUco detector\n(DREAM)", grey=True)
    box(axis, 0.05, 1.85, 1.75, 0.95, "Lidar, wheel speed,\nIMU (DREAM)", grey=True)
    box(axis, 11.75, 0.25, 1.6, 1.0, "Vehicle interface:\ndrive, steering,\npan servo (DREAM)", grey=True)
    # Policy
    box(axis, 2.2, 4.45, 1.5, 0.95, "Cone filter\n(Zhenling)")
    box(axis, 4.05, 4.45, 1.6, 0.95, "Camera-yaw\ncorrection\n(Toby; set to 0)")
    box(axis, 4.05, 3.15, 1.6, 0.95, "Local map:\nrecalled cones\n(Zhenling)")
    box(axis, 6.05, 3.4, 2.2, 2.0, "LANE ESTIMATION\nLaneDetector\n\nconcentric-arc fit,\ncolour sides (AUTO),\n"
                                   "width memory", mine=True, size=8.6)
    box(axis, 9.55, 4.65, 1.85, 0.9, "MPC lane following\n(Peiyu, Yuhao, Toby)")
    box(axis, 9.55, 3.45, 1.85, 0.9, "Obstacles in the lane,\navoidance (Zhenling)\n+ lidar")
    box(axis, 9.55, 2.25, 1.85, 0.9, "Intersection turner /\nnavigator (Toby, Yuhao)\n+ markers")
    box(axis, 11.75, 4.65, 1.6, 0.9, "CarController\n(Chris)")
    box(axis, 6.05, 2.05, 2.2, 0.95, "Lane overlay node\n-> Foxglove 3D panel\n(never drives)", mine=True)
    box(axis, 11.75, 2.55, 1.6, 1.4, "Camera pan\nheld at 0\nin every step\n(policy_runner)", mine=True)
    arrow(axis, (1.8, 4.92), (2.2, 4.92), "cones", offset=(0, 0.1))
    axis.text(2.95, 4.36, "cones: (x, y) in m, colour;\n10 Hz, about 0.2 s old", fontsize=6.8, color="#666",
              ha="center", va="top")
    arrow(axis, (3.7, 4.92), (4.05, 4.92))
    arrow(axis, (4.85, 4.45), (4.85, 4.1))
    arrow(axis, (5.65, 4.92), (6.05, 4.92), "batch", offset=(0, 0.1))
    arrow(axis, (5.65, 3.62), (6.05, 3.75), "recalled", offset=(0.02, -0.25))
    arrow(axis, (8.25, 5.1), (9.55, 5.1), colour=ACCENT)
    axis.text(8.9, 5.2, "e_y (m), e_psi (rad),\nkappa (1/m)", fontsize=6.8, color=ACCENT, ha="center")
    arrow(axis, (8.25, 4.3), (9.55, 3.9), colour=ACCENT)
    axis.text(8.9, 4.3, "centre arc,\nwidth (m)", fontsize=6.8, color=ACCENT, ha="center")
    arrow(axis, (8.25, 3.6), (9.55, 2.7), colour=ACCENT)
    axis.text(8.4, 2.75, "lane at\nthe junction", fontsize=6.8, color=ACCENT, ha="left")
    arrow(axis, (7.15, 3.4), (7.15, 3.0), "", colour=ACCENT)
    arrow(axis, (11.4, 5.1), (11.75, 5.1))
    arrow(axis, (12.55, 4.65), (12.55, 3.95))
    arrow(axis, (12.55, 2.55), (12.55, 1.25))
    axis.text(12.45, 1.9, "drive, steering,\npan = 0.0\n(normalised)", fontsize=7.5, color="#444", ha="right",
              va="center")
    axis.text(0.95, 1.6, "markers -> turner; lidar,\nwheel speed, IMU -> obstacles,\nlocal map, controller",
              fontsize=7.3, color="#666", ha="center", va="top")
    # Simulator band
    axis.add_patch(FancyBboxPatch((0.05, 0.05), 11.45, 1.0, boxstyle="round,pad=0.03", fc="#f7f9ff", ec=ACCENT,
                                  lw=2.2, ls="--"))
    axis.text(0.2, 0.86, "CAR SIMULATOR + LIVE 3D VIEWER  (feature/car-simulator, 27 Oct, not merged into dev)",
              fontsize=8.5, color=ACCENT, weight="bold", va="center")
    axis.text(0.2, 0.55, "Off-car: track file -> simulated cones, ArUco, lidar, wheel speed, gyro -> the same "
                         "MovementPolicy and settings -> measured car-20 response (replaces every grey box)",
              fontsize=7.6, va="center")
    axis.text(0.2, 0.25, "On-car: subscribes to the car's Foxglove bridge (publishes nothing) and redraws its "
                         "detections and the policy's own drawings in 3D.", fontsize=7.8, va="center")
    save(figure, "01_system.png")


def lane_estimation_diagram():
    """The steps inside LaneDetector.detect, with who wrote each, and the geometry of the model."""
    figure = plt.figure(figsize=(13.2, 6.6))
    top = figure.add_axes([0.0, 0.52, 1.0, 0.48])
    top.set_xlim(0, 13.2)
    top.set_ylim(0, 3.1)
    top.axis("off")
    stages = [
        ("1 Split by colour", "me, 9 Oct", True, "one row per colour;\nan edge needs >= 2 cones;\none row must span\n"
                                                  ">= 0.25 m for a direction"),
        ("2 Junction check", "Yuhao, 26 Oct", False, "if the arc misses > 15%\nof the cones, keep only\nthe incoming "
                                                      "straight\nrows"),
        ("3 Start", "me, 9 Oct", True, "linear least squares:\nparallel lines and\nconcentric circles;\nkeep the "
                                        "better"),
        ("4 Concentric-arc fit", "me, 9 Oct", True, "unknowns psi, kappa, d_L, d_R\nsoft-L1 loss (0.05 m)\n"
                                                    "prior pulls kappa to 0\n|kappa| <= 2 1/m"),
        ("5 Outlier check", "me, 9 Oct", True, "cone > 0.15 m from\nits edge -> drop it\nand refit\n(at most twice)"),
        ("6 Sides + centre", "me, 9-10 Oct", True, "AUTO: larger offset\n= left edge\nw = d_L - d_R\none edge: "
                                                   "+-w_mem/2"),
        ("7 Plausibility", "me, 9 Oct", True, "0.3 <= w <= 3.0 m\nno edge past the\nturning centre\nelse NO_LANE"),
    ]
    width, gap, y0 = 1.62, 0.2, 1.05
    for i, (title, owner, mine, text) in enumerate(stages):
        x = 0.15 + i * (width + gap)
        top.add_patch(FancyBboxPatch((x, y0), width, 1.75, boxstyle="round,pad=0.03", fc="#eef4ff" if mine else "white",
                                     ec=ACCENT if mine else NEUTRAL, lw=2.0 if mine else 1.1, ls="-" if mine else "--"))
        top.text(x + 0.08, y0 + 1.55, title, fontsize=8.6, weight="bold", va="center")
        top.text(x + 0.08, y0 + 1.3, owner, fontsize=7.6, color=ACCENT if mine else "#666", va="center")
        top.text(x + 0.08, y0 + 0.62, text, fontsize=7.4, va="center")
        if i < len(stages) - 1:
            top.annotate("", (x + width + gap - 0.02, y0 + 0.9), (x + width + 0.02, y0 + 0.9),
                         arrowprops=dict(arrowstyle="->", lw=1.1))
    x4, x5 = 0.15 + 3 * (width + gap) + width / 2, 0.15 + 4 * (width + gap) + width / 2
    top.annotate("", (x4, y0 + 1.78), (x5, y0 + 1.78), arrowprops=dict(arrowstyle="->", lw=1.1,
                                                                        connectionstyle="arc3,rad=0.35"))
    top.text((x4 + x5) / 2 + 0.6, y0 + 2.0, "refit", fontsize=7.5, ha="center", color="#444")
    top.add_patch(FancyBboxPatch((0.15, 0.08), 5.3, 0.72, boxstyle="round,pad=0.03", fc="white", ec=NEUTRAL, ls="--"))
    top.text(0.25, 0.44, "Second pass (Zhenling, 23 Oct): only if this batch alone gives a lane, run 1-7 again with "
                         "the\nlocal map's recalled cones added, and keep that only if it is still a lane.", fontsize=7.5,
             va="center")
    top.add_patch(FancyBboxPatch((5.7, 0.08), 3.6, 0.72, boxstyle="round,pad=0.03", fc="#eef4ff", ec=ACCENT))
    top.text(5.8, 0.44, "Width memory (me, 10 Oct): w_mem += 0.2 (w - w_mem)\nwhen both edges are seen; reset each run",
             fontsize=7.5, va="center")
    top.add_patch(FancyBboxPatch((9.55, 0.08), 3.55, 0.72, boxstyle="round,pad=0.03", fc="#eef4ff", ec=ACCENT))
    top.text(9.65, 0.44, "Out: LaneDetection = status, edges, centre arc,\nwidth, visible_until, e_y = -d_c, e_psi = "
                         "-psi, kappa_c", fontsize=7.5, va="center")

    # Geometry: a lane curving left, the reference arc through the car, the two edge offsets
    geo = figure.add_axes([0.03, 0.03, 0.33, 0.45])
    psi, kappa, d_left, d_right = math.radians(12), 0.35, 0.7, -0.1
    s = np.linspace(0, 2.6, 80)

    def arc(offset):
        k = kappa / (1 - kappa * offset)
        heading = psi + k * s
        x0, y0_ = -offset * math.sin(psi), offset * math.cos(psi)
        return x0 + (np.sin(heading) - math.sin(psi)) / k, y0_ - (np.cos(heading) - math.cos(psi)) / k

    rng = np.random.default_rng(3)
    for offset, colour, label in ((d_left, BLUE_CONE, "left edge = reference + d_L"),
                                  (d_right, YELLOW_CONE, "right edge = reference + d_R")):
        x, y = arc(offset)
        geo.plot(-y, x, color=colour, lw=2, label=label)
        cone_index = np.arange(8, 80, 9)
        geo.scatter(-(y[cone_index] + rng.normal(0, 0.02, len(cone_index))),
                    x[cone_index] + rng.normal(0, 0.02, len(cone_index)), s=28, color=colour, edgecolor="black",
                    lw=0.6, zorder=3)
    x, y = arc(0.0)
    geo.plot(-y, x, color=ACCENT, ls="--", lw=1.3, label="reference arc through the car (psi, kappa)")
    x, y = arc((d_left + d_right) / 2)
    geo.plot(-y, x, color="black", ls="-.", lw=1.1, label="centre line = reference + d_c")
    xo, yo = arc(d_right)
    geo.scatter([-(yo[44] + 0.2)], [xo[44]], s=28, color=YELLOW_CONE, edgecolor="black", lw=0.6, zorder=3)
    geo.scatter([-(yo[44] + 0.2)], [xo[44]], s=150, facecolor="none", edgecolor=BAD, lw=1.5, zorder=3,
                label="cone > 0.15 m from its edge: left out, refit")
    geo.add_patch(plt.Rectangle((-0.1, -0.35), 0.2, 0.45, fc="#ddd", ec="black"))
    geo.plot([0, 0], [0, 0.75], color="#666", ls=":", lw=1)
    theta = np.linspace(math.pi / 2, math.pi / 2 + psi, 20)
    geo.plot(0.6 * np.cos(theta), 0.6 * np.sin(theta), color=ACCENT, lw=1.2)
    geo.text(0.03, 0.8, "car +x", fontsize=7.5, color="#666")
    geo.annotate("psi: lane heading\n(e_psi = -psi)", (-0.06, 0.6), (0.3, 0.25), fontsize=7.5, color=ACCENT,
                 arrowprops=dict(arrowstyle="-", color=ACCENT, lw=0.8))
    dc = (d_left + d_right) / 2
    geo.annotate("", (-dc * math.cos(psi), -dc * math.sin(psi)), (0, 0),
                 arrowprops=dict(arrowstyle="<->", color=BAD, lw=1.3, shrinkA=0, shrinkB=0))
    geo.text(-dc / 2 - 0.05, -0.12, "e_y = -d_c", fontsize=8, color=BAD, ha="right", va="top")
    geo.set_aspect("equal")
    geo.set_xlim(-1.9, 0.9)
    geo.set_ylim(-0.45, 2.7)
    geo.set_xlabel("-y in base_link (m), left is left")
    geo.set_ylabel("x in base_link (m), forwards")
    geo.legend(fontsize=7.2, loc="upper left", frameon=True, bbox_to_anchor=(1.02, 1.0))
    geo.set_title("Model: concentric arcs, one per colour (drawn example)", fontsize=9.5)

    text = figure.add_axes([0.62, 0.03, 0.38, 0.45])
    text.axis("off")
    lines = [
        (0.97, 8.6, r"Unknowns  $\theta = (\psi, \kappa, d_L, d_R)$"),
        (0.86, 8.6, r"Cone $j$:  $s_j = x_j\cos\psi + y_j\sin\psi$,  $\ell_j = -x_j\sin\psi + y_j\cos\psi$"),
        (0.75, 8.6, r"Distance from the reference arc (exact, and stable as $\kappa \to 0$):"),
        (0.66, 11.5, r"$D_j = \dfrac{2\ell_j - \kappa(s_j^2 + \ell_j^2)}{1 + \sqrt{1 - \kappa\,(2\ell_j - "
                     r"\kappa(s_j^2 + \ell_j^2))}}$"),
        (0.42, 8.6, r"Residuals  $r_j = D_j - d_{edge(j)}$,  prior  $r_\kappa = (\sigma/\kappa_{typ})\,\kappa$,  "
                    r"$\sigma = 0.05$ m, $\kappa_{typ} = 0.5$ 1/m"),
        (0.30, 8.6, r"Fit  $\min_\theta \sum_j \rho(r_j) + \rho(r_\kappa)$,  $\rho$ = soft-L1 (0.05 m), "
                    r"$|\kappa| \leq 2$, $\leq 50$ evaluations"),
        (0.18, 8.6, r"Straight mode ($\kappa = 0$, 9 Oct default): one linear least-squares solve"),
        (0.06, 8.6, r"Out  $w = d_L - d_R$,  $d_c = (d_L + d_R)/2$,  $e_y = -d_c$,  $e_\psi = -\psi$,  "
                    r"$\kappa_c = \kappa / (1 - \kappa d_c)$"),
    ]
    for y, size, line in lines:
        text.text(0.0, y, line, fontsize=size, va="top")
    save(figure, "02_lane_estimation.png")


# ----------------------------------------------------------------------------------------------------------------
# Lane estimation on the recorded drives (Standard 4) and on drawn lanes (Standard 3)
# ----------------------------------------------------------------------------------------------------------------
def lane_modules():
    """Imports this checkout's policy code (dev at 37b7b39)."""
    sys.path.insert(0, str(REPO))
    from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones
    from policy.input_output import (CarObservations, ConeBatch, ConeColour, ConeDetection, Position,
                                     SensorAge)
    from policy.lane_detection.lane_detection import LaneDetectionConfig, LaneDetector, detect_lanes
    from policy.lane_detection.lanes import LaneStatus
    from scenarios.scenario_wrapper import Scenario, ScenarioType
    return dict(ConeFilterConfig=ConeFilterConfig, filter_cones=filter_cones, CarObservations=CarObservations,
                ConeBatch=ConeBatch, ConeColour=ConeColour, ConeDetection=ConeDetection, Position=Position,
                SensorAge=SensorAge, LaneDetectionConfig=LaneDetectionConfig, LaneDetector=LaneDetector,
                detect_lanes=detect_lanes, LaneStatus=LaneStatus, Scenario=Scenario, ScenarioType=ScenarioType)


def straight_config(m):
    """The 9 Oct default: straight edges, no junction check (which was added on 26 Oct)."""
    return m["LaneDetectionConfig"](max_curvature_per_m=0.0, associate_branches=False)


def curved_config(m):
    return m["LaneDetectionConfig"](max_curvature_per_m=2.0, associate_branches=False)


def replay(m, name, config, cone_filter=None):
    """Runs the cone filter and lane estimation on every step of a recorded drive.

    :return: One dict per step: distance driven so far (wheel speed x dt), wheel speed, the actions, the lane.
    """
    scenario = m["Scenario"].from_name(m["ScenarioType"].REAL, name)
    cone_filter = cone_filter or m["ConeFilterConfig"]()
    steps, distance = [], 0.0
    for observations, actions in scenario.steps():
        speed = float(observations.car.wheel_speed) if observations.car and observations.car.wheel_speed is not None \
            else None
        distance += (speed or 0.0) * observations.policy.dt
        cones = m["filter_cones"](observations.cones, cone_filter)
        observed = dataclasses.replace(observations, cones=cones)
        started = time.perf_counter()
        lane = m["detect_lanes"](observed, config)
        elapsed = time.perf_counter() - started
        steps.append(dict(index=len(steps), distance=distance, speed=speed, actions=actions, lane=lane, cones=cones,
                          raw=observations.cones, seconds=elapsed, stamp=observations.cones.sensor_age.stamp_ns))
    return steps


def both(m, steps):
    return [step for step in steps if step["lane"] is not None and step["lane"].status == m["LaneStatus"].BOTH_EDGES]


def pan_bias(m):
    """Recorded drives: the heading error the lane estimate reports, and the drift it would cause if it were real."""
    results = {"before": {}, "after": {}}
    figure, (left, middle, right) = plt.subplots(1, 3, figsize=(14, 4.0), gridspec_kw={"width_ratios": [1.3, 1, 1]})
    for name in STRAIGHT_RECORDINGS:
        steps = both(m, replay(m, name, straight_config(m)))
        moving = [s for s in steps if s["speed"] and s["speed"] > 0.02]
        heading = [math.degrees(s["lane"].heading_error_rad) for s in steps]
        start, end = steps[0], steps[-1]
        driven = end["distance"] - start["distance"]
        lateral_start = statistics.median(s["lane"].lateral_error_m for s in steps[:5])
        lateral_end = statistics.median(s["lane"].lateral_error_m for s in steps[-5:])
        median_heading = statistics.median(heading)
        rest = [math.degrees(s["lane"].heading_error_rad) for s in steps if s["speed"] is not None and s["speed"] < 0.02]
        steering = sorted({round(float(s["actions"].steering_action or 0.0), 3) for s in steps})
        pans = sorted({str(s["actions"].camera_pan_action) for s in steps})
        results["before"][name] = dict(
            steps_both_edges=len(steps), moving_steps=len(moving), driven_m=round(driven, 2),
            median_heading_deg=round(median_heading, 1), heading_iqr_deg=[round(q, 1) for q in
                                                                          np.percentile(heading, [25, 75])],
            median_heading_at_rest_deg=round(statistics.median(rest), 1) if rest else None,
            lateral_start_m=round(lateral_start, 3), lateral_end_m=round(lateral_end, 3),
            measured_drift_m=round(lateral_end - lateral_start, 3),
            predicted_drift_m=round(driven * math.sin(math.radians(median_heading)), 3),
            steering_actions=steering, camera_pan_actions=pans)
        # Break the line where steps without both edges were left out (the drive's end, or a second run)
        xs, ys, previous = [], [], None
        for step, value in zip(steps, heading):
            if previous is not None and step["index"] - previous > 3:
                xs.append(np.nan)
                ys.append(np.nan)
            xs.append(step["distance"] - start["distance"])
            ys.append(value)
            previous = step["index"]
        left.plot(xs, ys, marker=".", lw=1, label=name)
        results["before"][name]["width_median_m"] = round(statistics.median(s["lane"].lane_width_m for s in steps), 3)
    left.axhline(0, color="black", lw=0.8)
    left.set_xlabel("distance driven (m, from wheel speed)")
    left.set_ylabel("heading error from the lane fit (deg)")
    left.set_title("(a) 6 Oct drives, steering 0, pan never commanded:\nthe lane always looks turned 6-11 deg",
                   fontsize=9.5)
    left.legend(fontsize=7.5)
    names = list(STRAIGHT_RECORDINGS)
    x = np.arange(len(names))
    predicted = [results["before"][n]["predicted_drift_m"] for n in names]
    measured = [results["before"][n]["measured_drift_m"] for n in names]
    middle.bar(x - 0.2, predicted, 0.4, color=BAD, label="if the heading were real:\ndistance x sin(heading)")
    middle.bar(x + 0.2, measured, 0.4, color=GOOD, label="measured change in\nlateral error")
    for i, (p, q) in enumerate(zip(predicted, measured)):
        middle.text(i - 0.2, p + 0.01, f"{p:+.2f}", ha="center", fontsize=7.5)
        middle.text(i + 0.2, (q - 0.035) if q < 0 else q + 0.01, f"{q:+.2f}", ha="center", fontsize=7.5)
    middle.axhline(0, color="black", lw=0.8)
    middle.set_xticks(x, [n.replace("_", "\n") for n in names], fontsize=8)
    middle.set_ylabel("sideways drift over the drive (m, + = left)")
    middle.set_title("(b) The car did not drift: the cone\npositions were rotated, not the car", fontsize=9.5)
    middle.legend(fontsize=7.5, loc="upper left")
    middle.set_ylim(-0.12, 0.62)
    # At rest, before and after the pan hold
    for name in AFTER_FIX_RECORDINGS:
        steps = both(m, replay(m, name, straight_config(m)))
        rest = [math.degrees(s["lane"].heading_error_rad) for s in steps if s["speed"] is not None and s["speed"] < 0.02]
        pans = sorted({str(s["actions"].camera_pan_action) for s in replay(m, name, straight_config(m))})
        results["after"][name] = dict(median_heading_at_rest_deg=round(statistics.median(rest), 1), rest_steps=len(rest),
                                      camera_pan_actions=pans)
    before = [results["before"][n]["median_heading_at_rest_deg"] for n in STRAIGHT_RECORDINGS]
    after = [results["after"][n]["median_heading_at_rest_deg"] for n in AFTER_FIX_RECORDINGS]
    right.scatter(np.zeros(len(before)) + np.linspace(-0.08, 0.08, len(before)), before, color=BAD, s=40,
                  label="6 Oct: pan never commanded")
    right.scatter(np.ones(len(after)) + np.linspace(-0.08, 0.08, len(after)), after, color=GOOD, s=40,
                  label="13 Oct: policy holds pan at 0")
    right.axhline(0, color="black", lw=0.8)
    right.set_xticks([0, 1], ["before\n(4 drives)", "after f49333d\n(4 drives)"])
    right.set_xlim(-0.6, 1.6)
    right.set_ylabel("heading error at rest, before moving (deg)")
    right.set_title("(c) Before and after holding the pan\n(car placed by hand: not a controlled test)",
                    fontsize=9.5)
    right.legend(fontsize=7.5, loc="lower left")
    save(figure, "11_pan_bias.png")
    return results


def straight_or_curved(m):
    """Recorded straight lanes: with curvature allowed the fit bends them, and the heading error doubles."""
    results = {}
    for name in STRAIGHT_RECORDINGS:
        curved = both(m, replay(m, name, curved_config(m)))
        straight = both(m, replay(m, name, straight_config(m)))
        curvatures = [s["lane"].path_curvature_per_m for s in curved]
        results[name] = dict(
            curved_steps=len(curved), median_curvature_per_m=round(statistics.median(curvatures), 3),
            share_curving_left=round(sum(k > 0 for k in curvatures) / len(curvatures), 2),
            median_heading_straight_deg=round(statistics.median(math.degrees(s["lane"].heading_error_rad)
                                                                for s in straight), 1),
            median_heading_curved_deg=round(statistics.median(math.degrees(s["lane"].heading_error_rad)
                                                              for s in curved), 1),
            median_visible_until_m=round(statistics.median(s["lane"].visible_until_m for s in curved), 2),
            all_curvatures=[round(k, 4) for k in curvatures])
    figure, (left, right) = plt.subplots(1, 2, figsize=(12.5, 3.8))
    names = list(STRAIGHT_RECORDINGS)
    left.boxplot([results[n]["all_curvatures"] for n in names], showfliers=False)
    left.set_xticks(range(1, len(names) + 1), [n.replace("_", "\n") for n in names], fontsize=8)
    for i, n in enumerate(names):
        left.text(i + 1.28, results[n]["median_curvature_per_m"], f"{results[n]['share_curving_left']:.0%} left",
                  fontsize=7.5, va="center")
    left.axhline(0, color="black", lw=0.8)
    left.axhspan(1 / 12, 1 / 6, color=BAD, alpha=0.08)
    left.text(0.55, 1 / 6 + 0.01, "6-12 m radius", fontsize=7.5, color=BAD)
    left.set_ylabel("fitted lane curvature (1/m, + = left)")
    left.set_title("(a) Straight lanes, curvature allowed (max 2.0 1/m):\nmost steps read a gentle left bend",
                   fontsize=9.5)
    x = np.arange(len(names))
    s_values = [results[n]["median_heading_straight_deg"] for n in names]
    c_values = [results[n]["median_heading_curved_deg"] for n in names]
    right.bar(x - 0.2, s_values, 0.4, color=GOOD, label="straight edges (9 Oct default)")
    right.bar(x + 0.2, c_values, 0.4, color=BAD, label="curvature allowed")
    for i, (a, b) in enumerate(zip(s_values, c_values)):
        right.text(i - 0.2, a + 0.3, f"{a:.1f}", ha="center", fontsize=7.5)
        right.text(i + 0.2, b + 0.3, f"{b:.1f}", ha="center", fontsize=7.5)
    right.set_xticks(x, [n.replace("_", "\n") for n in names], fontsize=8)
    right.set_ylabel("median heading error (deg)")
    right.set_title("(b) The bend is extended back to the car,\nso the heading error grows", fontsize=9.5)
    right.legend(fontsize=7.5)
    save(figure, "12_straight_or_curved.png")
    for n in names:
        del results[n]["all_curvatures"]
    return results


def outlier_frame(m):
    """One recorded step where the fit leaves out a misplaced cone (outlier_cone, 6 Oct)."""
    steps = replay(m, "outlier_cone", straight_config(m))
    dropped_steps = []
    for index, step in enumerate(steps):
        lane = step["lane"]
        if lane.status == m["LaneStatus"].BOTH_EDGES:
            used = lane.left_edge.cone_count + lane.right_edge.cone_count
            if used < len(step["cones"]):
                dropped_steps.append(index)
    both_steps = both(m, steps)
    index = 5
    step = steps[index]
    lane = step["lane"]
    figure, axis = plt.subplots(figsize=(6.6, 5.2))
    filtered = {(c.pos.x, c.pos.y) for c in step["cones"]}
    for cone in step["raw"]:
        colour = BLUE_CONE if cone.colour == m["ConeColour"].BLUE else YELLOW_CONE
        if (cone.pos.x, cone.pos.y) not in filtered:
            axis.scatter(-cone.pos.y, cone.pos.x, s=30, facecolor="none", edgecolor=colour, marker="s")
    edges = {lane.left_edge.colour: lane.left_edge, lane.right_edge.colour: lane.right_edge}
    for cone in step["cones"]:
        colour = BLUE_CONE if cone.colour == m["ConeColour"].BLUE else YELLOW_CONE
        edge = edges[cone.colour]
        off = abs(edge.arc.signed_distance(cone.pos.x, cone.pos.y))
        axis.scatter(-cone.pos.y, cone.pos.x, s=46, color=colour, edgecolor="black", lw=0.6, zorder=3)
        if off > 0.15:
            axis.scatter(-cone.pos.y, cone.pos.x, s=260, facecolor="none", edgecolor=BAD, lw=1.8, zorder=3)
            axis.annotate(f"{off:.2f} m from its edge:\nleft out, edges refitted", (-cone.pos.y, cone.pos.x),
                          (-cone.pos.y + 0.35, cone.pos.x + 0.35), fontsize=8, color=BAD,
                          arrowprops=dict(arrowstyle="->", color=BAD))
    s = np.linspace(0, 2.6, 50)
    for arc, colour, style in ((lane.left_edge.arc, BLUE_CONE, "-"), (lane.right_edge.arc, YELLOW_CONE, "-"),
                               (lane.centre_line, "black", "-.")):
        points = np.array([arc.point_at(v) for v in s])
        axis.plot(-points[:, 1], points[:, 0], color=colour, ls=style, lw=1.6)
    axis.add_patch(plt.Rectangle((-0.1, -0.3), 0.2, 0.45, fc="#ddd", ec="black"))
    axis.set_aspect("equal")
    axis.set_xlabel("-y in base_link (m)")
    axis.set_ylabel("x in base_link (m)")
    axis.set_title(f"outlier_cone, step {index}: {len(step['raw'])} detections, {len(step['cones'])} after the "
                   f"cone filter,\n{lane.left_edge.cone_count + lane.right_edge.cone_count} used. Hollow squares: "
                   f"removed by the filter", fontsize=9)
    axis.text(0.02, 0.02, f"width {lane.lane_width_m:.2f} m, e_y {lane.lateral_error_m:+.2f} m, "
                          f"e_psi {math.degrees(lane.heading_error_rad):+.1f} deg", transform=axis.transAxes,
              fontsize=8)
    save(figure, "13_outlier_frame.png")
    widths = [s["lane"].lane_width_m for s in both_steps]
    return dict(steps=len(steps), both_edge_steps=len(both_steps), steps_with_a_cone_left_out=len(dropped_steps),
                width_median_m=round(statistics.median(widths), 3), width_std_m=round(statistics.pstdev(widths), 3),
                frame_step=index)


def timing(m):
    """Calculation time of detect_lanes on the recorded drives, straight and curved (this laptop, not the car)."""
    results = {}
    for label, config in (("straight", straight_config(m)), ("curved", curved_config(m))):
        times = []
        for name in STRAIGHT_RECORDINGS:
            for _ in range(3):
                times += [s["seconds"] * 1000 for s in replay(m, name, config)]
        results[label] = dict(median_ms=round(statistics.median(times), 3),
                              p99_ms=round(float(np.percentile(times, 99)), 3), max_ms=round(max(times), 3),
                              steps=len(times))
    return results


# ----------------------------------------------------------------------------------------------------------------
# Drawn lanes (Standard 3): cones placed on known lanes, seen through the measured camera model
# ----------------------------------------------------------------------------------------------------------------
CAMERA = dict(fov_deg=86.0, near_m=0.4, far_m=2.5, noise_m=0.03)
"""The camera model the team measured from real drives (tests/policy/real_car_measurements.py LAB_CAR, Toby)."""


def lane_points(m, radius_m, width_m=0.8, spacing_m=0.2, length_m=3.5, left_blue=True):
    """World cones of a lane whose centre starts at (0, 0) heading +x, curving left (radius > 0) or right (< 0)."""
    blue, yellow = m["ConeColour"].BLUE, m["ConeColour"].YELLOW
    left, right = (blue, yellow) if left_blue else (yellow, blue)
    points = []
    for i in range(-3, int(round(length_m / spacing_m)) + 1):
        s = i * spacing_m
        for colour, side in ((left, 1.0), (right, -1.0)):
            if math.isinf(radius_m):
                points.append((s, side * width_m / 2, colour))
            else:
                turn, radius = math.copysign(1.0, radius_m), abs(radius_m)
                r = radius - turn * side * width_m / 2
                points.append((r * math.sin(s / radius), turn * (radius - r * math.cos(s / radius)), colour))
    return points


def camera_view(m, points, car_x, car_y, car_heading, rng=None, noise_m=0.0, far_m=CAMERA["far_m"]):
    """The cones the camera sees from a car pose, in base_link, with Gaussian position noise."""
    cones = []
    c, s = math.cos(car_heading), math.sin(car_heading)
    for x, y, colour in points:
        bx, by = c * (x - car_x) + s * (y - car_y), -s * (x - car_x) + c * (y - car_y)
        if not CAMERA["near_m"] <= math.hypot(bx, by) <= far_m or abs(math.degrees(math.atan2(by, bx))) > 43.0:
            continue
        if noise_m:
            bx, by = bx + rng.gauss(0, noise_m), by + rng.gauss(0, noise_m)
        cones.append(m["ConeDetection"](m["Position"](bx, by, 0.05), colour, 0.9))
    return cones


def observe(m, cones):
    return m["CarObservations"](cones=m["ConeBatch"](cones, m["SensorAge"](0.2, 1), 0.2), fiducials=None,
                                lidar_scan=None, lidar_cartesian=None, car=None, policy=None)


def fit_accuracy(m):
    """How well the fit recovers a known lane, straight and curved, with and without the measured noise."""
    import random
    radii = [1.0, 1.5, 2.0, 3.0, 5.0, 8.0, math.inf, -8.0, -5.0, -3.0, -2.0, -1.5, -1.0]
    offset_m, heading_deg, seeds = 0.10, 5.0, 30
    results = {}
    for label, config, noise in (("curved, no noise", curved_config(m), 0.0),
                                 ("curved, 3 cm noise", curved_config(m), CAMERA["noise_m"]),
                                 ("straight edges, 3 cm noise", straight_config(m), CAMERA["noise_m"])):
        rows = []
        for radius in radii:
            points = lane_points(m, radius)
            errors, lost = [], 0
            for seed in range(seeds if noise else 1):
                cones = camera_view(m, points, 0.0, offset_m, math.radians(heading_deg), random.Random(seed), noise)
                lane = m["detect_lanes"](observe(m, cones), config)
                if lane is None or not lane.has_lane:
                    lost += 1
                    continue
                truth = 0.0 if math.isinf(radius) else 1.0 / radius
                errors.append((100 * abs(lane.lateral_error_m - offset_m),
                               abs(math.degrees(lane.heading_error_rad) - heading_deg),
                               lane.path_curvature_per_m - truth))
            e = np.array(errors)
            rows.append(dict(curvature_per_m=0.0 if math.isinf(radius) else round(1 / radius, 3),
                             cones_seen=len(camera_view(m, points, 0.0, offset_m, math.radians(heading_deg))),
                             no_lane=lost, lateral_cm=[round(v, 2) for v in np.percentile(e[:, 0], [25, 50, 75])],
                             heading_deg=[round(v, 2) for v in np.percentile(e[:, 1], [25, 50, 75])],
                             curvature_error=[round(v, 3) for v in np.percentile(e[:, 2], [25, 50, 75])]))
        results[label] = rows
    colours = {"curved, no noise": NEUTRAL, "curved, 3 cm noise": GOOD, "straight edges, 3 cm noise": BAD}
    figure, axes = plt.subplots(1, 3, figsize=(14, 3.9))
    for key, title, unit in (("lateral_cm", "(a) lateral error e_y", "|error| (cm)"),
                             ("heading_deg", "(b) heading error e_psi", "|error| (deg)"),
                             ("curvature_error", "(c) lane curvature", "fitted - true (1/m)")):
        axis = axes[["lateral_cm", "heading_deg", "curvature_error"].index(key)]
        for label, rows in results.items():
            x = [r["curvature_per_m"] for r in rows]
            q = np.array([r[key] for r in rows])
            axis.plot(x, q[:, 1], marker="o", ms=3.5, color=colours[label], label=label)
            if "noise" in label and "no noise" not in label:
                axis.fill_between(x, q[:, 0], q[:, 2], color=colours[label], alpha=0.15)
        axis.set_xlabel("true lane curvature (1/m; + left, 1.0 = 1 m radius)")
        axis.set_ylabel(unit)
        axis.set_title(title + " (median, IQR of 30 seeds)", fontsize=9.5)
        if key == "heading_deg":
            axis.set_ylim(0, 30)
        if key == "lateral_cm":
            axis.set_ylim(0, 30)
    axes[0].legend(fontsize=7.5)
    figure.suptitle("Drawn 0.8 m lanes seen through the measured camera (86 deg, 0.4-2.5 m); car 0.10 m left of "
                    "centre, pointing 5 deg left", fontsize=9.5, y=1.02)
    save(figure, "04_fit_accuracy.png")
    return dict(setup=dict(lane_width_m=0.8, cone_spacing_m=0.2, car_offset_m=offset_m, car_heading_deg=heading_deg,
                           seeds=seeds, **CAMERA), results=results)


def sides_and_width(m):
    """A drive down a 0.6 m lane whose right row is missing for a stretch, and lanes with either colour on the left."""
    import random
    rng = random.Random(1)
    width, steps = 0.6, 90
    points = lane_points(m, math.inf, width_m=width, length_m=14.0)
    yellow = m["ConeColour"].YELLOW
    detector = m["LaneDetector"](straight_config(m))
    truth, fixed, remembered, status = [], [], [], []
    for k in range(steps):
        x = 0.1 * k
        offset = 0.08 * math.sin(2 * math.pi * k / 45)
        cones = camera_view(m, points, x, offset, 0.0, rng, CAMERA["noise_m"])
        if 30 <= k < 60:
            cones = [c for c in cones if c.colour != yellow]
        lane_fixed = m["detect_lanes"](observe(m, cones), straight_config(m))
        lane_memory = detector.detect(observe(m, cones))
        truth.append(offset)
        fixed.append(lane_fixed.lateral_error_m if lane_fixed.has_lane else np.nan)
        remembered.append(lane_memory.lateral_error_m if lane_memory.has_lane else np.nan)
        status.append(lane_memory.status.name)
    one_edge = slice(30, 60)
    err_fixed = np.nanmean(np.abs(np.array(fixed[one_edge]) - np.array(truth[one_edge])))
    err_memory = np.nanmean(np.abs(np.array(remembered[one_edge]) - np.array(truth[one_edge])))
    # Either colour on the left, with left_colour fixed to BLUE (8 Oct) or AUTO (10 Oct)
    found = {}
    for left_blue in (True, False):
        for name in ("BLUE", "AUTO"):
            config = dataclasses.replace(straight_config(m), left_colour=name)
            ok = 0
            for seed in range(50):
                r = random.Random(seed)
                pts = lane_points(m, math.inf, width_m=0.8, left_blue=left_blue)
                lane = m["detect_lanes"](observe(m, camera_view(m, pts, 0.0, r.uniform(-0.15, 0.15),
                                                                math.radians(r.uniform(-10, 10)), r,
                                                                CAMERA["noise_m"])), config)
                ok += lane.has_lane
            found[f"{'blue' if left_blue else 'yellow'} on the left, left_colour {name}"] = ok / 50
    figure, (left, right) = plt.subplots(1, 2, figsize=(13, 3.9), gridspec_kw={"width_ratios": [1.7, 1]})
    k = np.arange(steps) * 0.1
    left.axvspan(3.0, 6.0, color="#eee")
    left.text(4.5, 0.2, "right (yellow) row missing:\none edge only", ha="center", fontsize=8)
    left.plot(k, truth, color="black", lw=2, label="true offset from the lane centre")
    left.plot(k, fixed, color=BAD, lw=1.2, label="width fixed at the configured 1.0 m (9 Oct)")
    left.plot(k, remembered, color=GOOD, lw=1.2, label="width remembered from both edges (10 Oct)")
    left.set_xlabel("distance driven (m)")
    left.set_ylabel("lateral error e_y (m, + = left)")
    left.set_ylim(-0.3, 0.32)
    left.set_title(f"(a) 0.6 m lane: with one edge the fixed width puts the centre "
                   f"{100 * err_fixed:.0f} cm off; remembered {100 * err_memory:.1f} cm", fontsize=9.5)
    left.legend(fontsize=7.5, loc="lower left")
    labels = list(found)
    right.barh(range(len(labels)), [100 * found[x] for x in labels],
               color=[GOOD if found[x] > 0.9 else BAD for x in labels])
    right.set_yticks(range(len(labels)), [x.replace(", ", "\n") for x in labels], fontsize=8)
    for i, x in enumerate(labels):
        right.text(100 * found[x] + 1, i, f"{100 * found[x]:.0f}%", va="center", fontsize=8)
    right.set_xlim(0, 115)
    right.set_xlabel("steps with a lane found (%, 50 random poses)")
    right.set_title("(b) Which colour is the left edge: fixed or AUTO", fontsize=9.5)
    save(figure, "05_sides_and_width.png")
    return dict(lane_width_m=width, one_edge_mean_abs_error_fixed_m=round(float(err_fixed), 3),
                one_edge_mean_abs_error_remembered_m=round(float(err_memory), 3),
                remembered_width_m=round(detector.lane_width_m, 3), lane_found_share=found)


def wrong_colour_cone(m):
    """One misclassified cone: a yellow cone where the left (blue) row should be, with short or long rows in view."""
    blue, yellow = m["ConeColour"].BLUE, m["ConeColour"].YELLOW
    counts = list(range(2, 13))
    outcomes = {}
    for mode, config in (("straight", straight_config(m)), ("curved", curved_config(m))):
        for where in ("near", "far"):
            row = []
            for n in counts:
                xs = [0.4 + 0.2 * i for i in range(n)]
                cones = [m["ConeDetection"](m["Position"](x, side, 0.05), colour, 0.9)
                         for x in xs for side, colour in ((0.4, blue), (-0.4, yellow))]
                stray_x = 0.5 if where == "near" else xs[-1] + 0.3
                cones.append(m["ConeDetection"](m["Position"](stray_x, 0.4, 0.05), yellow, 0.9))
                lane = m["detect_lanes"](observe(m, cones), config)
                if not lane.has_lane:
                    row.append(("no lane: stop", None))
                elif lane.status != m["LaneStatus"].BOTH_EDGES:
                    row.append(("one edge kept", dict(e_y=round(lane.lateral_error_m, 3), status=lane.status.name)))
                elif abs(lane.lateral_error_m) < 0.03 and abs(math.degrees(lane.heading_error_rad)) < 2:
                    row.append(("cone left out", round(lane.lateral_error_m, 3)))
                else:
                    row.append(("false lane", dict(e_y=round(lane.lateral_error_m, 3),
                                                   e_psi_deg=round(math.degrees(lane.heading_error_rad), 1),
                                                   width=round(lane.lane_width_m, 2))))
            outcomes[f"{mode}, stray {where}"] = row
    figure, axis = plt.subplots(figsize=(11, 2.9))
    colour = {"cone left out": GOOD, "one edge kept": "#7fa9d6", "no lane: stop": WARN, "false lane": BAD}
    for i, (label, row) in enumerate(outcomes.items()):
        for j, (outcome, detail) in enumerate(row):
            axis.add_patch(plt.Rectangle((j, i), 0.95, 0.9, color=colour[outcome]))
            if outcome == "false lane":
                axis.text(j + 0.47, i + 0.45, f"{detail['e_psi_deg']:+.0f} deg", ha="center", va="center",
                          fontsize=7, color="white")
    axis.set_xlim(0, len(counts))
    axis.set_ylim(0, len(outcomes))
    axis.set_xticks(np.arange(len(counts)) + 0.47, [f"{n}\n{0.2 * (n - 1):.1f} m" for n in counts])
    axis.set_yticks(np.arange(len(outcomes)) + 0.45, list(outcomes))
    axis.set_xlabel("cones per row in view (and how far the rows reach beyond the first cone)")
    for name, c in colour.items():
        axis.bar([0], [0], color=c, label=name)
    axis.legend(fontsize=8, ncol=4, loc="upper center", bbox_to_anchor=(0.5, -0.32))
    axis.set_title("One yellow cone in the blue row (near: 0.5 m ahead; far: just past the last cone). "
                   "Text in red cells: the false heading", fontsize=9.5)
    save(figure, "06_wrong_colour_cone.png")
    return dict(counts=counts, outcomes=outcomes)


def joint_vs_per_row(m):
    """The alternative: each colour fitted alone as a straight line (reference_from_cones, the MPC's fit before mine)."""
    import random
    sys.path.insert(0, str(REPO))
    from policy.action_policy.reference import reference_from_cones
    blue, yellow = m["ConeColour"].BLUE, m["ConeColour"].YELLOW
    shorts, seeds = [2, 3, 4, 6, 8, 12], 200
    drawn = {"joint (mine)": [], "per row (before)": []}
    for n in shorts:
        mine, before = [], []
        for seed in range(seeds):
            rng = random.Random(seed)
            cones = [m["ConeDetection"](m["Position"](0.4 + 0.2 * i + rng.gauss(0, 0.03), 0.4 + rng.gauss(0, 0.03), 0.05),
                                        blue, 0.9) for i in range(12)]
            cones += [m["ConeDetection"](m["Position"](0.4 + 0.2 * i + rng.gauss(0, 0.03), -0.4 + rng.gauss(0, 0.03),
                                                       0.05), yellow, 0.9) for i in range(n)]
            lane = m["detect_lanes"](observe(m, cones), straight_config(m))
            old = reference_from_cones(cones, 0.8)
            mine.append(math.degrees(lane.heading_error_rad))
            before.append(math.degrees(old.heading_error_rad))
        rms = lambda v: math.sqrt(sum(x * x for x in v) / len(v))
        drawn["joint (mine)"].append(round(rms(mine), 2))
        drawn["per row (before)"].append(round(rms(before), 2))
    real = {}
    for name in STRAIGHT_RECORDINGS:
        pairs = []
        for step in replay(m, name, straight_config(m)):
            lane, old = step["lane"], reference_from_cones(step["cones"], 1.0)
            if lane is not None and lane.status == m["LaneStatus"].BOTH_EDGES and old is not None:
                pairs.append((step["index"], lane.heading_error_rad, old.heading_error_rad))
        jumps = [(abs(b[1] - a[1]), abs(b[2] - a[2])) for a, b in zip(pairs, pairs[1:]) if b[0] - a[0] == 1]
        real[name] = dict(steps=len(pairs), median_heading_jump_deg_joint=round(math.degrees(
            statistics.median(j[0] for j in jumps)), 2), median_heading_jump_deg_per_row=round(math.degrees(
                statistics.median(j[1] for j in jumps)), 2))
    figure, (left, right) = plt.subplots(1, 2, figsize=(12.5, 3.8))
    left.plot(shorts, drawn["per row (before)"], marker="s", color=BAD, label="each row its own line, slopes averaged "
                                                                                "(reference_from_cones)")
    left.plot(shorts, drawn["joint (mine)"], marker="o", color=GOOD, label="both rows fitted together (mine)")
    left.set_xlabel("cones in the short (right) row; the left row has 12")
    left.set_ylabel("heading error RMS (deg, 200 seeds)")
    left.set_ylim(0, 3.5)
    left.annotate("2 cones span < 0.25 m: the per-row\nfit drops that row and uses one", (2, 1.32), (5.2, 2.2),
                  fontsize=7.5, color="#444", arrowprops=dict(arrowstyle="->", color="#888"))
    left.set_title("(a) Drawn straight lane, 3 cm noise: a short row\nno longer tips the heading", fontsize=9.5)
    left.legend(fontsize=7.5, loc="upper right")
    names = list(STRAIGHT_RECORDINGS)
    x = np.arange(len(names))
    right.bar(x - 0.2, [real[n]["median_heading_jump_deg_per_row"] for n in names], 0.4, color=BAD,
              label="per row (before)")
    right.bar(x + 0.2, [real[n]["median_heading_jump_deg_joint"] for n in names], 0.4, color=GOOD,
              label="joint (mine)")
    right.set_xticks(x, [n.replace("_", "\n") for n in names], fontsize=8)
    right.set_ylabel("median step-to-step change in e_psi (deg)")
    right.set_title("(b) The 6 Oct drives, even rows: about the same jitter", fontsize=9.5)
    right.legend(fontsize=7.5)
    save(figure, "03_joint_vs_per_row.png")
    return dict(short_row_cones=shorts, heading_rms_deg=drawn, real_drives=real)


def characterise(m):
    """What lane estimation measured on the four 6 Oct drives (straight edges), and the lever-arm check of Section 2."""
    table = {}
    for name in STRAIGHT_RECORDINGS:
        steps = replay(m, name, straight_config(m))
        good = both(m, steps)
        widths = [s["lane"].lane_width_m for s in good]
        lateral = [(s["index"], s["lane"].lateral_error_m) for s in good]
        jumps = [abs(b[1] - a[1]) for a, b in zip(lateral, lateral[1:]) if b[0] - a[0] == 1]
        mean_along = [statistics.mean(s["lane"].centre_line.project(c.pos.x, c.pos.y) for c in s["cones"])
                      for s in good]
        curved = both(m, replay(m, name, curved_config(m)))
        kappa = statistics.median(s["lane"].path_curvature_per_m for s in curved)
        heading_growth = (statistics.median(math.degrees(s["lane"].heading_error_rad) for s in curved)
                          - statistics.median(math.degrees(s["lane"].heading_error_rad) for s in good))
        table[name] = dict(
            steps=len(steps), both_edges_share=round(len(good) / len(steps), 2),
            width_mean_m=round(statistics.mean(widths), 3), width_sd_m=round(statistics.pstdev(widths), 3),
            worse_edge_rms_cm=round(100 * statistics.median(max(s["lane"].left_edge.rms_residual_m,
                                                                 s["lane"].right_edge.rms_residual_m)
                                                             for s in good), 1),
            visible_until_m=round(statistics.median(s["lane"].visible_until_m for s in good), 2),
            lateral_jump_median_cm=round(100 * statistics.median(jumps), 1),
            lateral_jump_p95_cm=round(100 * float(np.percentile(jumps, 95)), 1),
            mean_cone_distance_m=round(statistics.median(mean_along), 2),
            curved_median_curvature_per_m=round(kappa, 3),
            predicted_heading_growth_deg=round(math.degrees(kappa * statistics.median(mean_along)), 1),
            observed_heading_growth_deg=round(heading_growth, 1))
    return table


# ----------------------------------------------------------------------------------------------------------------
# The simulator (Standard 3), run in a separate process inside the feature/car-simulator checkout
# ----------------------------------------------------------------------------------------------------------------
SIM_CASES = [("lab_straight", None), ("curvy_road", None), ("obstacle_lane", None), ("aruco_lane", None),
             ("cross", 3), ("cross", 4), ("cross", 5), ("t_junction", 3), ("t_junction", 5)]
"""The bundled tracks; at the cross and T, marker IDs 3, 4 and 5 ask for left, right and straight (26 Oct mapping)."""


def _sim_worker(out_path):
    """Runs inside the simulator checkout: every bundled track, seeds and cone misses, and a camera-yaw sweep."""
    import logging
    logging.disable(logging.CRITICAL)
    sys.path.insert(0, str(Path.cwd()))
    from tests.policy.car_models import CARS
    from tests.policy.simulator.sensors import SensorSettings
    from tests.policy.simulator.simulation import Simulation, aruco_settings, run_settings
    from tests.policy.simulator.track import load_track

    aruco = aruco_settings()

    def run(track_name, marker_id, seed=0, miss=0.0, yaw=0.0, keep_path=False):
        track = load_track(track_name, marker_id)
        sensors = SensorSettings(camera_yaw_deg=yaw, cone_miss_probability=miss,
                                 min_consecutive_frames=int(aruco["min_consecutive_frames"]),
                                 allowed_marker_ids=tuple(aruco.get("allowed_marker_ids", []) or ()))
        sim = Simulation(track, run_settings(track), CARS["car20"], sensors, seed)
        path, lanes, frames, k = [], [], [], 0
        while not sim.finished:
            stepped = sim.step()
            k += 1
            if keep_path and k % 10 == 0:
                state = sim.car.state
                path.append((round(state.x, 3), round(state.y, 3)))
            if stepped["policy"]:
                lane = sim.policy.lanes
                cones = sim.policy.cones
                if lane is not None and lane.status.name == "BOTH_EDGES":
                    lanes.append((len(cones),
                                  max(lane.left_edge.rms_residual_m, lane.right_edge.rms_residual_m),
                                  lane.lane_width_m, lane.visible_until_m,
                                  max(math.hypot(c.pos.x, c.pos.y) for c in cones)))
        summary = sim.summary()
        state = sim.car.state
        heading_change = math.degrees(math.atan2(math.sin(state.heading - track.start[2]),
                                                 math.cos(state.heading - track.start[2])))
        events = [e.text for e in summary.events]
        result = dict(track=track_name, marker_id=marker_id, seed=seed, miss=miss, yaw_deg=yaw,
                      ended=summary.ended_because, time_s=round(summary.time_s, 1),
                      distance_m=round(summary.distance_m, 2), cones_touched=summary.cones_touched,
                      obstacles_touched=summary.obstacles_touched, max_off_road_m=round(summary.max_off_road_m, 3),
                      final=(round(state.x, 2), round(state.y, 2)), heading_change_deg=round(heading_change, 1),
                      events=events)
        result["passed"] = passed(result)
        if keep_path:
            result["path"] = path
            result["cones"] = [(round(c.x, 3), round(c.y, 3), c.colour.name) for c in track.cones]
            result["markers"] = [(m.x, m.y, m.id) for m in track.markers]
            result["obstacles"] = [o.polygon().round(3).tolist() for o in track.obstacles]
            result["lane_steps"] = lanes
        return result

    def passed(r):
        clean = r["cones_touched"] == 0 and r["obstacles_touched"] == 0
        name, marker = r["track"], r["marker_id"]
        if name == "lab_straight":
            return clean and 3.2 <= r["final"][0] <= 3.8
        if name == "curvy_road":
            return clean and r["distance_m"] >= 10.5
        if name == "obstacle_lane":
            return clean and r["distance_m"] >= 5.0
        if name == "aruco_lane":
            return clean and any("KEEP_LEFT" in e for e in r["events"]) and any("KEEP_CENTRE" in e for e in r["events"])
        if name == "t_junction" and marker == 5:
            return clean and any("requested exit is closed" in e for e in r["events"])
        target = {3: 90.0, 4: -90.0, 5: 0.0}[marker]
        return clean and abs(r["heading_change_deg"] - target) < 30.0 and r["distance_m"] > 8.0

    output = {"tracks": [run(t, mid, keep_path=True) for t, mid in SIM_CASES]}
    output["robustness"] = [run(t, mid, seed, miss) for t, mid in SIM_CASES for miss in (0.0, 0.2)
                            for seed in range(5)]
    output["camera_yaw"] = [run(t, mid, seed, 0.0, yaw) for t, mid in (("lab_straight", None), ("curvy_road", None),
                                                                        ("cross", 3), ("cross", 4))
                            for yaw in (-8.0, -4.0, -2.0, 0.0, 2.0, 4.0, 8.0) for seed in range(3)]
    Path(out_path).write_text(json.dumps(output))


def run_simulator(sim_repo):
    out = DATA / "simulator_runs_full.json"
    DATA.mkdir(parents=True, exist_ok=True)
    subprocess.run([sys.executable, "-W", "ignore", str(Path(__file__).resolve()), "--sim-worker", str(out)],
                   cwd=sim_repo, check=True)
    return json.loads(out.read_text())


TRACK_TITLES = {("lab_straight", None): "lab_straight", ("curvy_road", None): "curvy_road",
                ("obstacle_lane", None): "obstacle_lane", ("aruco_lane", None): "aruco_lane (IDs 0, 1)",
                ("cross", 3): "cross, ID 3: turn left", ("cross", 4): "cross, ID 4: turn right",
                ("cross", 5): "cross, ID 5: straight on", ("t_junction", 3): "T, ID 3: turn left",
                ("t_junction", 5): "T, ID 5: straight (closed)"}


def simulator_figures(runs, m):
    """The bundled tracks, the robustness runs, the camera-yaw sweep, and the simulated camera against the car's."""
    figure, axes = plt.subplots(3, 3, figsize=(13, 11.5))
    for axis, r in zip(axes.flat, runs["tracks"]):
        cones = np.array([(x, y) for x, y, _ in r["cones"]])
        colours = [BLUE_CONE if c == "BLUE" else YELLOW_CONE for _, _, c in r["cones"]]
        axis.scatter(cones[:, 0], cones[:, 1], s=4, c=colours)
        for polygon in r["obstacles"]:
            p = np.array(polygon + [polygon[0]])
            axis.fill(p[:, 0], p[:, 1], color="#8b5a2b")
        for x, y, marker_id in r["markers"]:
            axis.scatter([x], [y], marker="s", s=40, color="black")
            axis.text(x, y - 0.35, f"ID {marker_id}", fontsize=7, ha="center", va="top")
        path = np.array(r["path"])
        axis.plot(path[:, 0], path[:, 1], color=BAD if not r["passed"] else GOOD, lw=2)
        axis.scatter([path[0, 0]], [path[0, 1]], color="black", s=12, zorder=3)
        axis.set_aspect("equal")
        pad = 0.6
        xs, ys = np.concatenate([cones[:, 0], path[:, 0]]), np.concatenate([cones[:, 1], path[:, 1]])
        if r["track"] in ("cross", "t_junction"):
            axis.set_xlim(-6, 6)
            axis.set_ylim(-6, 6)
        else:
            axis.set_xlim(xs.min() - pad, xs.max() + pad)
            axis.set_ylim(ys.min() - pad, ys.max() + pad)
        verdict = "pass" if r["passed"] else "FAIL"
        axis.set_title(f"{TRACK_TITLES[(r['track'], r['marker_id'])]}: {verdict}\n{r['distance_m']:.1f} m, "
                       f"{r['cones_touched']} cones touched; {r['ended'][:42]}", fontsize=8.5)
        axis.tick_params(labelsize=7)
    for axis in axes[:, 0]:
        axis.set_ylabel("y (m)")
    for axis in axes[-1, :]:
        axis.set_xlabel("x (m)")
    figure.suptitle("The simulator's bundled tracks, seed 0, car-20 model, measured sensor noise (green path = "
                    "pass)", fontsize=10.5, y=1.0)
    figure.tight_layout()
    save(figure, "07_simulator_tracks.png")

    # Camera yaw: the policy assumes the camera points straight ahead (pan held at 0, no yaw correction)
    yaws = sorted({r["yaw_deg"] for r in runs["camera_yaw"]})
    figure, (left, right) = plt.subplots(1, 2, figsize=(12.5, 3.8))
    cases = [("lab_straight", None), ("curvy_road", None), ("cross", 3), ("cross", 4)]
    styles = {("lab_straight", None): ("black", "o"), ("curvy_road", None): (GOOD, "s"),
              ("cross", 3): (ACCENT, "^"), ("cross", 4): (BAD, "v")}
    yaw_table = {}
    for case in cases:
        rate = []
        for yaw in yaws:
            group = [r for r in runs["camera_yaw"] if (r["track"], r["marker_id"]) == case and r["yaw_deg"] == yaw]
            rate.append(100 * sum(r["passed"] for r in group) / len(group))
        yaw_table[TRACK_TITLES[case]] = dict(zip([str(y) for y in yaws], rate))
        colour, marker = styles[case]
        left.plot(yaws, rate, marker=marker, color=colour, label=TRACK_TITLES[case])
    left.set_xlabel("simulated camera yaw (deg, + = camera points left); the policy is not told")
    left.set_ylabel("runs passed (%, 3 seeds)")
    left.set_ylim(-5, 105)
    left.axvspan(6, 11, color=BAD, alpha=0.08)
    left.text(8.5, 50, "bias seen in the\n6 Oct drives", ha="center", fontsize=7.5, color=BAD)
    left.legend(fontsize=7.5)
    left.set_title("(a) What an uncorrected camera yaw does to the whole system", fontsize=9.5)
    finals = []
    for yaw in yaws:
        group = [r for r in runs["camera_yaw"] if r["track"] == "lab_straight" and r["yaw_deg"] == yaw]
        finals.append([r["final"][1] for r in group])
    right.boxplot(finals, positions=yaws, widths=0.8)
    right.axhline(0, color="black", lw=0.8)
    right.axhline(0.4, color=BAD, ls="--", lw=0.8)
    right.axhline(-0.4, color=BAD, ls="--", lw=0.8)
    right.text(yaws[0], 0.42, "lane edge", fontsize=7.5, color=BAD)
    right.set_xlabel("simulated camera yaw (deg)")
    right.set_ylabel("car's final y on lab_straight (m, + = left)")
    right.set_title("(b) A camera turned left puts the car right of centre, and the other way round", fontsize=9.5)
    save(figure, "09_camera_yaw.png")

    # Simulated camera against the recorded drives, both after the shipped cone filter
    import yaml
    shipped = yaml.safe_load((REPO / "config" / "ai4r_policy.yaml").read_text())
    shipped = shipped[next(iter(shipped))]["ros__parameters"]["cone_filter"]
    cone_filter = m["ConeFilterConfig"](**shipped)
    real = []
    for name in STRAIGHT_RECORDINGS:
        for step in both(m, replay(m, name, straight_config(m), cone_filter)):
            lane = step["lane"]
            real.append((len(step["cones"]), max(lane.left_edge.rms_residual_m, lane.right_edge.rms_residual_m),
                         lane.lane_width_m, lane.visible_until_m, max(math.hypot(c.pos.x, c.pos.y) for c in step["cones"])))
    sim = [tuple(x) for r in runs["tracks"] if r["track"] == "lab_straight" for x in r["lane_steps"]]
    real, sim = np.array(real), np.array(sim)
    figure, axes = plt.subplots(1, 3, figsize=(13, 3.6))
    comparison = {}
    for axis, (index, label, unit, scale) in zip(axes, ((0, "cones per step after the filter", "cones", 1),
                                                         (4, "farthest cone used", "m", 1),
                                                         (1, "worse edge's RMS residual", "cm", 100))):
        bins = np.linspace(min(real[:, index].min(), sim[:, index].min()) * scale,
                           max(real[:, index].max(), sim[:, index].max()) * scale, 16)
        axis.hist(real[:, index] * scale, bins=bins, alpha=0.6, color=BAD, density=True,
                  label=f"6 Oct drives ({len(real)} steps)")
        axis.hist(sim[:, index] * scale, bins=bins, alpha=0.6, color=GOOD, density=True,
                  label=f"simulated lab_straight ({len(sim)} steps)")
        axis.set_xlabel(f"{label} ({unit})")
        axis.set_ylabel("share of steps (density)")
        comparison[label] = dict(real_median=round(float(np.median(real[:, index]) * scale), 3),
                                 sim_median=round(float(np.median(sim[:, index]) * scale), 3))
        axis.set_title(f"{label}: median real {comparison[label]['real_median']:.2f}, "
                       f"sim {comparison[label]['sim_median']:.2f}", fontsize=9)
    axes[0].legend(fontsize=7.5)
    figure.suptitle("Does the simulated camera give the lane fit the same input as the real one? (both through the "
                    "shipped cone filter, straight lanes)", fontsize=9.5, y=1.03)
    save(figure, "10_sim_vs_real.png")

    robustness = {}
    for case in SIM_CASES:
        for miss in (0.0, 0.2):
            group = [r for r in runs["robustness"] if (r["track"], r["marker_id"]) == case and r["miss"] == miss]
            robustness.setdefault(TRACK_TITLES[case], {})[f"miss {miss:.0%}"] = (
                f"{sum(r['passed'] for r in group)}/{len(group)}")
            failures = [dict(seed=r["seed"], ended=r["ended"], cones=r["cones_touched"],
                             heading_change=r["heading_change_deg"]) for r in group if not r["passed"]]
            if failures:
                robustness[TRACK_TITLES[case]][f"failures, miss {miss:.0%}"] = failures
    tracks = {TRACK_TITLES[(r["track"], r["marker_id"])]: {k: r[k] for k in ("passed", "ended", "distance_m",
                                                                             "cones_touched", "obstacles_touched",
                                                                             "heading_change_deg", "events")}
              for r in runs["tracks"]}
    return dict(tracks_seed0=tracks, robustness_5_seeds=robustness, camera_yaw_pass_percent=yaw_table,
                sim_vs_real=comparison)


def slide_figures(m, recorded, drawn):
    """Small versions of the main results for the slide (figures/slide_*.png)."""
    from PIL import Image
    # 1 Lane estimation on a real frame
    steps = replay(m, "outlier_cone", straight_config(m))
    step, lane = steps[5], steps[5]["lane"]
    figure, axis = plt.subplots(figsize=(3.4, 2.3))
    for cone in step["cones"]:
        colour = BLUE_CONE if cone.colour == m["ConeColour"].BLUE else YELLOW_CONE
        axis.scatter(cone.pos.x, cone.pos.y, s=18, color=colour, edgecolor="black", lw=0.4, zorder=3)
        edge = lane.left_edge if cone.colour == lane.left_edge.colour else lane.right_edge
        if abs(edge.arc.signed_distance(cone.pos.x, cone.pos.y)) > 0.15:
            axis.scatter(cone.pos.x, cone.pos.y, s=120, facecolor="none", edgecolor=BAD, lw=1.4, zorder=3)
    s = np.linspace(0, 2.4, 30)
    for arc, colour, style in ((lane.left_edge.arc, BLUE_CONE, "-"), (lane.right_edge.arc, YELLOW_CONE, "-"),
                               (lane.centre_line, "black", "-.")):
        points = np.array([arc.point_at(v) for v in s])
        axis.plot(points[:, 0], points[:, 1], color=colour, ls=style, lw=1.3)
    axis.add_patch(plt.Rectangle((-0.3, -0.1), 0.45, 0.2, fc="#ddd", ec="black"))
    axis.set_xlabel("x ahead (m)", fontsize=7)
    axis.set_ylabel("y left (m)", fontsize=7)
    axis.tick_params(labelsize=6.5)
    axis.set_aspect("equal")
    axis.set_title("Real drive (6 Oct): edges fitted, misplaced\ncone (red) left out", fontsize=7.5)
    save(figure, "slide_lane.png")
    # 2 The pan bias
    before = recorded["pan_bias"]["before"]
    names = list(STRAIGHT_RECORDINGS)
    figure, axis = plt.subplots(figsize=(3.4, 2.3))
    x = np.arange(len(names))
    axis.bar(x - 0.2, [before[n]["predicted_drift_m"] for n in names], 0.4, color=BAD,
             label="drift if the 6-11 deg\nheading were real")
    axis.bar(x + 0.2, [before[n]["measured_drift_m"] for n in names], 0.4, color=GOOD, label="measured drift")
    axis.axhline(0, color="black", lw=0.6)
    axis.set_xticks(x, ["straight", "messy", "outlier", "long"], fontsize=6.5)
    axis.set_ylabel("sideways drift (m)", fontsize=7)
    axis.tick_params(labelsize=6.5)
    axis.legend(fontsize=6, loc="upper left")
    axis.set_ylim(-0.12, 0.6)
    axis.set_title("4 real drives, steering 0: the car went\nstraight, so the camera was turned", fontsize=7.5)
    save(figure, "slide_pan.png")
    # 3 Width memory and colour sides
    width = drawn["sides_and_width"]
    figure, axis = plt.subplots(figsize=(3.4, 2.3))
    labels = ["fixed width\n(9 Oct)", "remembered\nwidth (10 Oct)"]
    values = [100 * width["one_edge_mean_abs_error_fixed_m"], 100 * width["one_edge_mean_abs_error_remembered_m"]]
    bars = axis.bar(labels, values, color=[BAD, GOOD])
    for bar, value in zip(bars, values):
        axis.text(bar.get_x() + bar.get_width() / 2, value + 0.5, f"{value:.1f} cm", ha="center", fontsize=7)
    axis.set_ylabel("centre error, one edge seen (cm)", fontsize=7)
    axis.tick_params(labelsize=6.5)
    axis.set_ylim(0, 25)
    axis.set_title("0.6 m lane, right row missing (sim);\nAUTO: either colour on the left, 100%", fontsize=7.5)
    save(figure, "slide_floor.png")
    # 4 The simulator viewer
    image = Image.open(OUT / "08_simulator_viewer.png")
    # Panel (a) without the side panels: the car, the cones and the planned turn
    image.crop((300, 70, 1080, 610)).save(OUT / "slide_sim.png")
    print("wrote", OUT / "slide_sim.png")


VIEWER_PANELS = [
    ("cross_chase_approach.png", "(a) cross, ID 3: sign read at 2.8 s; at 6.8 s the planned left turn (green)"),
    ("cross_camera_approach.png", "(b) the same moment from the car camera (86 deg view, 0.30 m up, 20 deg down)"),
    ("obstacle_chase.png", "(c) obstacle_lane: lidar points (red) on the box; the car passes it on the left"),
    ("aruco_chase.png", "(d) aruco_lane: KEEP_CENTRE read from sign ID 1; left panel: my lane estimate, live"),
]


def viewer_figure(shot_dir):
    """Puts the four viewer screenshots (taken with viewer_screenshots.js) into one figure."""
    from PIL import Image, ImageDraw, ImageFont
    width, height, pad, caption = 1200, 675, 8, 34
    canvas = Image.new("RGB", (2 * width + 3 * pad, 2 * (height + caption) + 3 * pad), "white")
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 19)
    except OSError:
        font = ImageFont.load_default()
    draw = ImageDraw.Draw(canvas)
    for i, (name, text) in enumerate(VIEWER_PANELS):
        image = Image.open(Path(shot_dir) / name).convert("RGB").resize((width, height), Image.LANCZOS)
        x, y = pad + (i % 2) * (width + pad), pad + (i // 2) * (height + caption + pad)
        draw.text((x + 4, y + 6), text, fill=(30, 30, 30), font=font)
        canvas.paste(image, (x, y + caption))
    OUT.mkdir(parents=True, exist_ok=True)
    canvas.save(OUT / "08_simulator_viewer.png", optimize=True)
    print("wrote", OUT / "08_simulator_viewer.png")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sim-repo", type=Path, help="A checkout of origin/feature/car-simulator (6c4cd17).")
    parser.add_argument("--viewer-shots", type=Path, help="Folder of screenshots from viewer_screenshots.js.")
    parser.add_argument("--sim-worker", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.sim_worker:
        _sim_worker(args.sim_worker)
        return
    m = lane_modules()
    if args.viewer_shots:
        viewer_figure(args.viewer_shots)
    system_diagram()
    lane_estimation_diagram()
    recorded = dict(characterisation=characterise(m), pan_bias=pan_bias(m), straight_or_curved=straight_or_curved(m),
                    outlier_frame=outlier_frame(m), timing_ms=timing(m))
    write_json("recorded_drives.json", recorded)
    drawn = dict(joint_vs_per_row=joint_vs_per_row(m), fit_accuracy=fit_accuracy(m), sides_and_width=sides_and_width(m),
                 wrong_colour_cone=wrong_colour_cone(m))
    write_json("drawn_lanes.json", drawn)
    slide_figures(m, recorded, drawn)
    if args.sim_repo:
        runs = run_simulator(args.sim_repo.resolve())
    elif (DATA / "simulator_runs_full.json").exists():
        # Redraw the simulator figures from the saved runs (about 10 minutes to repeat with --sim-repo)
        runs = json.loads((DATA / "simulator_runs_full.json").read_text())
    else:
        return
    write_json("simulator_runs.json", simulator_figures(runs, m))


if __name__ == "__main__":
    main()
