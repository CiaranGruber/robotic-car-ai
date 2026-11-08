"""Makes the figures in Peiyu's logbook section (docs/logbook/peiyu/figures/).

    python3 docs/logbook/peiyu/make_figures.py              # diagram and summary figures only (numbers below)
    python3 docs/logbook/peiyu/make_figures.py --simulate   # also the simulated and replayed figures

--simulate needs dreamgym (wheels/dreamgym-0.2.2-py3-none-any.whl) and foxglove-sdk. It extracts each commit it
simulates with `git archive` into a temporary folder, and runs each one in its own process, so two versions of the
policy package never mix:

- 01a47a0 and 9a7f9d8: just before and at the Task A7 lane-end rule (figure 03).
- 47f8838: milestone M1, the MPC and the baseline on the unseen cases, and T12 (figures 03 and 05).
- 45eb015: PR #18's control settings on the three car models (figure 06).

Figure 08 replays a car 20 data log from scenarios/data/original_data/ (not in git) through this checkout's
scripts/mcap_export.py, and is skipped when the log is missing. The numbers in the summary figures are copied from the
documents named beside each one, so they can be checked there.
"""
import argparse
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch, Rectangle  # noqa: E402

HERE = Path(__file__).resolve().parent
OUT = HERE / "figures"
REPO = HERE.parents[2]
BAD, GOOD, NEUTRAL, ACCENT, AMBER = "#c8553d", "#2a7f62", "#8a8a8a", "#1f5fd1", "#b26b00"
plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False, "savefig.dpi": 150})

COMMITS = {"a7_before": "01a47a0", "a7_after": "9a7f9d8", "m1": "47f8838", "control": "45eb015"}
"""The checkout each simulation job runs in."""
CAR20_LOG = REPO / "scenarios" / "data" / "original_data" / "2026-10-20_14-24-21"
"""Car 20, 20 Oct: a drive recorded with data logging on (MPC, CENTRE, 0.5 m/s), blue cones on the left."""


def save(figure, name):
    OUT.mkdir(parents=True, exist_ok=True)
    figure.savefig(OUT / name, bbox_inches="tight")
    plt.close(figure)
    print("wrote", OUT / name)


def label_bars(axis, bars, fmt="{:.0f}", size=8):
    for bar in bars:
        value = bar.get_height()
        axis.annotate(fmt.format(value), (bar.get_x() + bar.get_width() / 2, value), ha="center", va="bottom",
                      fontsize=size, xytext=(0, 2), textcoords="offset points")


# ---------------------------------------------------------------------------------------------------------------------
# Diagram and summary figures (no simulation)
# ---------------------------------------------------------------------------------------------------------------------
def system_diagram():
    """Where my parts sit in the MovementPolicy and around it, with the interfaces between them."""
    figure, axis = plt.subplots(figsize=(12.6, 4.6))
    axis.set_xlim(0, 12.6)
    axis.set_ylim(0, 4.6)
    axis.axis("off")
    boxes = {
        "cam": (0.1, 3.3, "OAK-D camera\n(oakd_cone_detector)\ncones at 10 Hz, ~0.2 s old", NEUTRAL),
        "sensors": (0.1, 1.9, "Wheel speed, IMU,\nlidar, RC sticks", NEUTRAL),
        "node": (0.1, 0.5, "policy_node (DREAM)\none step per detection\nbatch; data logs", NEUTRAL),
        "lanes": (2.9, 3.3, "Cone filter, lane detection,\nlocal map\n(teammates)", NEUTRAL),
        "obst": (2.9, 1.9, "Obstacle detection\n+ avoidance (teammates)\n-> obstacle stop, line", NEUTRAL),
        "timing": (2.9, 0.5, "Step timing + run summary\nmedian / p95 / max\n(PR #31)", ACCENT),
        "mpc": (5.7, 3.3, "MPC lane follower\npath-frame model, delay\ncompensation, bounded LSQ", ACCENT),
        "a7": (5.7, 1.9, "Lane end / lost lane (A7)\nstop at the lane end; else\n0.5 s hold, then speed 0", ACCENT),
        "fox": (5.7, 0.5, "Foxglove: live markers,\nMCAP replay of data logs,\nrun_report (PR #25, #32)", ACCENT),
        "open": (8.5, 3.3, "Open-space RC policy\n(alternative action policy;\nRC drive held = go)", ACCENT),
        "ctrl": (8.5, 1.9, "CarController (Chris)\n(speed, curvature) ->\nthrottle, steering", NEUTRAL),
        "rl": (8.5, 0.5, "RL steering (PPO)\nsame MpcPolicy, other solver\nsimulation only (A5)", ACCENT),
    }
    for key, (x, y, text, colour) in boxes.items():
        axis.add_patch(FancyBboxPatch((x, y), 2.4, 1.0, boxstyle="round,pad=0.04", fc="white", ec=colour,
                                      lw=2.2 if colour == ACCENT else 1.2, ls="--" if key == "rl" else "-"))
        axis.text(x + 1.2, y + 0.5, text, ha="center", va="center", fontsize=8.5)

    def arrow(start, end, text="", offset=(0.0, 0.08), ha="center", both=False):
        axis.annotate("", end, start, arrowprops=dict(arrowstyle="<->" if both else "->", lw=1.2))
        if text:
            axis.text((start[0] + end[0]) / 2 + offset[0], (start[1] + end[1]) / 2 + offset[1], text, ha=ha,
                      fontsize=7.5, color="#444")

    arrow((2.55, 3.8), (2.85, 3.8), "cones")
    arrow((5.35, 3.8), (5.65, 3.8), "lane")
    arrow((2.55, 2.4), (2.85, 2.4), "lidar")
    arrow((5.35, 2.4), (5.65, 2.4))
    arrow((6.9, 3.28), (6.9, 2.92), "no usable lane", offset=(-0.08, -0.06), ha="right", both=True)
    arrow((8.15, 3.55), (8.45, 2.6))
    axis.text(7.62, 3.05, "(speed, curvature)", fontsize=7.5, color="#444", ha="center")
    arrow((8.15, 2.4), (8.45, 2.4))
    arrow((9.7, 3.28), (9.7, 2.92), "or, if action_policy is open_space", offset=(0.08, -0.06), ha="left")
    arrow((2.55, 1.0), (2.85, 1.0), "each step")
    arrow((5.35, 1.0), (5.65, 1.0), "logs")
    axis.text(12.5, 4.45, "blue outline = parts I built", ha="right", fontsize=8, color=ACCENT)
    axis.text(0.1, 4.45, "Policy step every 0.1 s (budget 20 ms); policy never enables the vehicle", fontsize=8,
              color="#444")
    save(figure, "01_system.png")


def rl_reward():
    """docs/stage1-design.md A5: the reward shapes and the three training iterations (24 tuning runs each)."""
    figure, (left, right) = plt.subplots(1, 2, figsize=(12.5, 3.9), gridspec_kw={"width_ratios": [1, 1.2]})
    lateral = [i / 200 for i in range(0, 81)]
    unbounded = [1 - 0.5 * (y / 0.1) ** 2 for y in lateral]
    bounded = [1 - 0.5 * (1 - math.exp(-(y / 0.1) ** 2)) for y in lateral]
    left.plot(lateral, unbounded, color=BAD, lw=2, label="iteration 1: 1 - a (y / 0.1)^2 (unbounded)")
    left.plot(lateral, bounded, color=GOOD, lw=2, label="iteration 2: 1 - a P(y, 0.1), P = 1 - exp(-(y/s)^2)")
    left.axhline(0, color="#444", lw=0.8)
    left.axvline(0.375, color=NEUTRAL, ls=":", lw=1)
    left.text(0.365, -1.4, "body reaches the\ncones (1 m lane)", fontsize=8, ha="right", color="#444")
    left.set_ylim(-8, 1.8)
    left.set_xlabel("distance from the lane centre y (m), heading and curvature rate 0")
    left.set_ylabel("reward per 0.1 s step")
    left.set_title("Lateral term (a = 0.5, 'balanced'); a collision ends the episode")
    left.legend(fontsize=8, frameon=False, loc="lower left")
    names = ["it. 1\nbalanced\n300k", "it. 2\ntracking\n300k", "it. 2\nbalanced\n300k", "it. 2\nsmooth\n300k",
             "it. 3\nbalanced\n1M"]
    passed = [0, 11, 15, 3, 18]
    bars = right.bar(names, passed, color=[BAD, NEUTRAL, NEUTRAL, NEUTRAL, AMBER], width=0.6)
    label_bars(right, bars)
    right.axhline(24, color=GOOD, ls="--", lw=1.2)
    right.text(4.4, 24.6, "MPC (A4) and baseline (A3): 24/24", fontsize=8, ha="right", color=GOOD)
    right.annotate("reward exploit: drove out of the lane\nto end the episode (9 steps on average)", (0, 0.3),
                   xytext=(-0.35, 19.2), fontsize=8, color=BAD, arrowprops=dict(arrowstyle="->", color=BAD))
    right.set_ylim(0, 28)
    right.set_ylabel("tuning runs passed (of 24, simulation)")
    right.set_title("PPO steering policy: three training iterations")
    right.tick_params(axis="x", labelsize=8)
    save(figure, "02_rl_reward.png")


DESIGNS = [
    # name, tracking RMS (mm), curvature-rate RMS mean (1/(m s)), tuning runs passed of 24, kind
    ("A3 baseline: pure pursuit\nL = 1.0 m, gain 1.0", 9.0, 0.37, 24, "pp"),
    ("MPC N12 q3/0.3 r0.3", 9.3, 0.37, 24, "front"),
    ("MPC N12 q3/0.3 r1", 9.8, 0.23, 24, "front"),
    ("MPC N12 q3/1 r1", 12.0, 0.21, 24, "front"),
    ("MPC N20 q10/3 r10", 12.2, 0.17, 24, "front"),
    ("A4 choice: MPC N20 q3/1 r3", 12.4, 0.16, 24, "chosen"),
    ("MPC starting values\nN12 q10/1 r1", 8.0, 0.41, 21, "start"),
    ("RL balanced 1M", 21.4, 0.68, 18, "rl"),
    ("RL balanced", 27.4, 0.86, 15, "rl"),
    ("RL tracking", 27.0, 0.94, 11, "rl"),
    ("RL smooth", 78.9, 2.84, 3, "rl"),
]
"""docs/stage1-design.md, the A4 Pareto table and the A5 table: 12 tuning cases with their own noise and with 0.03 m."""


def design_front():
    """Tracking against smoothness for every design that ran on the 24 tuning runs, and how many runs each passed."""
    figure, (left, right) = plt.subplots(1, 2, figsize=(13, 4.2), gridspec_kw={"width_ratios": [1.25, 1]})
    styles = {"pp": ("D", NEUTRAL, 70), "front": ("o", ACCENT, 40), "chosen": ("*", GOOD, 220),
              "start": ("X", BAD, 80), "rl": ("s", AMBER, 50)}
    front = sorted((d for d in DESIGNS if d[4] in ("front", "chosen", "pp")), key=lambda d: d[2])
    left.plot([d[2] for d in front], [d[1] for d in front], "-", color=ACCENT, lw=1, alpha=0.5, zorder=1)
    for name, tracking, rate, passed, kind in DESIGNS:
        marker, colour, size = styles[kind]
        if rate > 1.2:
            left.annotate(f"{name.split(chr(10))[0]}: {rate:.2f}, {tracking:.0f} mm, {passed}/24 ->", (1.18, 29),
                          fontsize=7.5, ha="right", color=AMBER)
            continue
        left.scatter(rate, tracking, marker=marker, color=colour, s=size, zorder=3)
    for name, tracking, rate, passed, kind in DESIGNS:
        if kind in ("pp", "chosen", "start", "rl"):
            offsets = {"pp": (8, 6), "chosen": (-10, 12), "start": (8, -16), "RL balanced": (-118, 4),
                       "RL tracking": (6, -14)}
            left.annotate(f"{name} ({passed}/24)", (rate, tracking), xytext=offsets.get(kind, offsets.get(name, (8, -4))),
                          textcoords="offset points", fontsize=7.5)
    left.text(0.02, 1.5, "blue line: Pareto front of the designs that pass all 24 runs", fontsize=7.5, color=ACCENT)
    left.set_xlim(0, 1.2)
    left.set_ylim(0, 31)
    left.set_xlabel("steering roughness: curvature-rate RMS, mean over runs (1/(m s), R4 limit 1.0)")
    left.set_ylabel("tracking: lateral RMS (mm)")
    left.set_title("A3 / A4 / A5: lower left is better (324 MPC options, 73 pass all runs)")
    names = ["A3 pure\npursuit", "MPC start\n(fails T11 R1b,\nT5 R4)", "MPC\nA4 choice", "RL balanced 1M\n(fails R4, R1b\non 6 runs)"]
    passed = [24, 21, 24, 18]
    bars = right.bar(names, passed, color=[NEUTRAL, BAD, GOOD, AMBER], width=0.55)
    label_bars(right, bars)
    right.set_ylim(0, 28)
    right.set_ylabel("tuning runs passed (of 24)")
    right.set_title("Same 12 cases, own noise and 0.03 m noise")
    save(figure, "04_design_front.png")


A6_HORIZONS = [12, 20, 30, 40, 60]
A6_MS = {"median": [0.31, 0.46, 0.74, 1.17, 2.94], "99th percentile": [0.55, 0.53, 1.57, 2.47, 5.96],
         "maximum": [1.61, 0.86, 2.32, 3.65, 8.89]}
"""docs/stage1-design.md A6: MpcController.solve on car 20's computer, 300 solves per horizon, 6 Oct."""


def car_timing():
    """The MPC's solve time on the car computer (A6), and the 22 Oct car log's step timing (PR #31)."""
    figure, (left, right) = plt.subplots(1, 2, figsize=(13, 4.0))
    width = 0.27
    colours = {"median": GOOD, "99th percentile": ACCENT, "maximum": NEUTRAL}
    for i, (name, values) in enumerate(A6_MS.items()):
        bars = left.bar([j + (i - 1) * width for j in range(len(A6_HORIZONS))], values, width, color=colours[name],
                        label=name)
        label_bars(left, bars, "{:.1f}", size=7)
    left.axhline(20, color=BAD, ls="--", lw=1.2)
    left.text(4.4, 21, "budget: 20 ms per step (1.5)", ha="right", fontsize=8, color=BAD)
    left.set_xticks(range(len(A6_HORIZONS)), [f"{n}\n{n * 0.05:.1f} m" for n in A6_HORIZONS], fontsize=8)
    left.set_xlabel("horizon N (steps of 0.1 s), and the distance predicted at 0.5 m/s")
    left.set_ylim(0, 23)
    left.set_ylabel("MPC solve time (ms)")
    left.set_title("Car 20's computer, 6 Oct: 300 solves per horizon")
    left.legend(fontsize=8, frameon=False, loc="upper left")
    groups = ["calculation\n(replayed on a laptop)", "time between\npolicy steps", "age of the\ntriggering sample"]
    stats = {"median": [2.1, 57.4, 78.1], "p95": [5.6, 94.1, 102.2], "max": [11.7, 367.8, 107.8]}
    colours = {"median": GOOD, "p95": ACCENT, "max": NEUTRAL}
    for i, (name, values) in enumerate(stats.items()):
        bars = right.bar([j + (i - 1) * width for j in range(3)], values, width, color=colours[name], label=name)
        label_bars(right, bars, "{:.0f}" if max(values) > 50 else "{:.1f}", size=7)
    right.axhline(20, color=BAD, ls="--", lw=1.2)
    right.text(-0.42, 23, "20 ms budget", ha="left", fontsize=8, color=BAD)
    right.set_yscale("log")
    right.set_ylim(1, 900)
    right.set_xticks(range(3), groups, fontsize=8)
    right.set_ylabel("ms (log scale)")
    right.set_title("Car log 22 Oct 16:58, 30.2 s, 477 steps: the run summary line")
    right.legend(fontsize=8, frameon=False, loc="upper left")
    right.set_xlim(-0.5, 2.5)
    save(figure, "07_car_timing.png")


def slide_figures():
    """Small figures for the one presentation slide that need no simulation."""
    figure, axis = plt.subplots(figsize=(4.4, 2.3))
    bars = axis.bar([str(n) for n in A6_HORIZONS], A6_MS["99th percentile"], color=ACCENT, width=0.55)
    label_bars(axis, bars, "{:.1f}")
    axis.axhline(20, color=BAD, ls="--", lw=1.2)
    axis.text(4.3, 17.2, "20 ms budget", fontsize=8, ha="right", color=BAD)
    axis.set_ylim(0, 22)
    axis.set_xlabel("MPC horizon (steps of 0.1 s)", fontsize=8)
    axis.set_ylabel("p99 solve time (ms)", fontsize=8)
    axis.tick_params(labelsize=8)
    axis.set_title("MPC on car 20's computer, 6 Oct", fontsize=9)
    save(figure, "slide_mpc.png")

    figure, axis = plt.subplots(figsize=(4.4, 2.3))
    names = ["pure pursuit\n(A3 baseline)", "MPC\n(A4)", "RL, PPO\n(A5)"]
    worst = [0.80, 0.51, 1.59]
    bars = axis.bar(names, worst, color=[NEUTRAL, GOOD, BAD], width=0.55)
    label_bars(axis, bars, "{:.2f}")
    axis.axhline(1.0, color="#444", ls="--", lw=1)
    axis.text(-0.38, 1.04, "R4 limit 1.0", fontsize=7.5, ha="left")
    axis.set_ylim(0, 1.9)
    axis.set_ylabel("worst steering roughness\n(unseen U1-U3, 1/(m s))", fontsize=8)
    axis.tick_params(labelsize=8)
    axis.set_title("M1: every unseen case must pass", fontsize=9)
    save(figure, "slide_design.png")


# ---------------------------------------------------------------------------------------------------------------------
# Simulation jobs: each runs in its own process from the checkout of COMMITS[job] and returns plain numbers
# ---------------------------------------------------------------------------------------------------------------------
def _import_harness(repo: Path):
    sys.path[:0] = [str(repo), str(repo / "tests" / "policy"), str(repo / "tests" / "policy" / "unseen")]
    import stage1_scenarios as s1
    return s1


def job_lane_end(repo: Path) -> dict:
    """Stop position past the last cone pair on T1-T10 with the shipped MPC of this checkout."""
    s1 = _import_harness(repo)
    from stage1_tuning_cases import TUNING_CASES
    stops = {}
    for case in TUNING_CASES:
        if case.name in [f"T{i}" for i in range(1, 11)]:
            result = s1.run_case(case, s1.shipped_mpc_policy(case))
            stops[case.name] = result.metrics["stop_past_last_cone_m"]
    return {"stops": stops}


def _trajectory(result, every=2) -> dict:
    return {"x": [round(float(v), 4) for v in result.x_m[::every]], "y": [round(float(v), 4) for v in result.y_m[::every]],
            "t": [round(float(v), 3) for v in result.time_s[::every]],
            "speed": [round(float(v), 4) for v in result.speed_m_per_s[::every]],
            "commands": [(round(t, 3), c.speed_m_per_s, c.curvature_per_m) for t, c in result.commands],
            "metrics": {k: (None if isinstance(v, float) and math.isnan(v) else v) for k, v in result.metrics.items()},
            "passed": result.passed}


def job_m1(repo: Path) -> dict:
    """The A3 baseline and the A4 MPC on the unseen cases, and the chosen MPC on T12 (camera covered from 3 m)."""
    s1 = _import_harness(repo)
    import stage1_design as design
    from stage1_unseen_cases import UNSEEN_CASES
    from stage1_tuning_cases import TUNING_CASES
    designs = {"pure pursuit (A3)": design.Design(design.BASELINE, "pure_pursuit", (("lookahead_m", 1.0), ("gain", 1.0))),
               "MPC (A4)": design.Design(design.CHOSEN_MPC, "mpc")}
    unseen = {}
    for name, chosen in designs.items():
        unseen[name] = {case.name: _trajectory(s1.run_case(case, chosen.policy(case))) for case in UNSEEN_CASES}
    u2 = next(case for case in UNSEEN_CASES if case.name == "U2")
    cones = [(colour.name, index, x, y) for colour, index, x, y in s1.all_cones(u2.lane)]
    t12 = next(case for case in TUNING_CASES if case.name == "T12")
    return {"unseen": unseen, "u2_cones": cones,
            "u2_missed": [(c.name, i) for c, i in u2.disturbances.missed_cones],
            "u2_wrong": [(c.name, i) for c, i in u2.disturbances.wrong_colour_cones],
            "t12": _trajectory(s1.run_case(t12, s1.shipped_mpc_policy(t12)), every=1),
            "t12_blackout_m": t12.disturbances.camera_blackout_from_m,
            "t12_lane_length_m": t12.lane.length_m}


CONTROL_SETTINGS = {
    "car 20's calibration (ControlConfig defaults)": {},
    "dev after #16: car 22's values": dict(speed_ff_offset=0.25, speed_ff_gain=12.0, speed_kp=0.2, speed_ki=0.15,
                                           speed_i_band=0.5, max_drive_action=0.45),
    "PR #18: one setting for both cars": dict(speed_ff_offset=0.30, speed_ff_gain=12.0, speed_kp=0.3, speed_ki=0.3,
                                              speed_i_band=0.6, max_drive_action=0.55),
}
"""ControlConfig changes compared in figure 06 (config/ai4r_policy.yaml at 45eb015~1 and 45eb015)."""


def job_control(repo: Path) -> dict:
    """T1 at 0.5 m/s through CarController and each car model, with each control setting."""
    import dataclasses
    s1 = _import_harness(repo)
    from car_models import CARS
    from policy.control.control import ControlConfig
    from stage1_tuning_cases import TUNING_CASES
    case = dataclasses.replace(next(c for c in TUNING_CASES if c.name == "T1"), speed_m_per_s=0.5)
    runs = {}
    for car_name, car in CARS.items():
        for label, changes in CONTROL_SETTINGS.items():
            policy = s1.shipped_mpc_policy(case, car, stop_distance_per_speed_s=2.1)
            result = s1.run_case(case, policy, car=car, control_config=ControlConfig(**changes))
            runs[f"{car_name}|{label}"] = {"t": [round(float(v), 3) for v in result.time_s[::2]],
                                           "speed": [round(float(v), 4) for v in result.speed_m_per_s[::2]]}
    return {"runs": runs, "cars": list(CARS)}


def job_car20_replay(repo: Path) -> dict:
    """Replays the car 20 data log through scripts/mcap_export.py and draws, with policy/visualisation.py, the first
    step after 3 s where the car is moving and the replay has a lane and a new MPC plan: the layers Foxglove shows."""
    import logging
    sys.path.insert(0, str(repo))
    logging.disable(logging.WARNING)
    from scripts import mcap_export
    from policy import visualisation
    from policy.memory import CarMemory
    steps = mcap_export.read_data_log(Path(f"{CAR20_LOG}_observations.json"))
    policy = mcap_export.build_policy(mcap_export.read_settings(Path(f"{CAR20_LOG}_config.json")))
    memory, drawer = CarMemory(), visualisation.StepDrawer()
    drawing, mismatches = None, 0
    start = steps[0].observations.policy.policy_elapsed_s
    for step in steps:
        replayed = policy.step(step.observations, memory)
        memory.step()
        layers = drawer.draw(policy, step.observations, replayed)
        if step.actions is not None and not mcap_export._same_actions(replayed, step.actions):
            mismatches += 1
        elapsed = step.observations.policy.policy_elapsed_s - start
        speed = step.observations.car.wheel_speed
        if (drawing is None and elapsed > 3.0 and speed is not None and speed > 0.3 and len(layers["lane"].lines) >= 2
                and any(len(line.points) > 1 for line in layers["plan/mpc"].lines)):
            drawing = {name: {"lines": [(line.points, line.colour) for line in layer.lines],
                              "spheres": [(s.x, s.y, s.diameter_m, s.colour) for s in layer.spheres],
                              "boxes": [(b.position, b.size, b.colour) for b in layer.boxes]}
                       for name, layer in layers.items()}
            drawing["t"], drawing["speed"] = elapsed, float(speed)
            drawing["summary"] = visualisation.summary(policy, step.observations, replayed)
    return {"drawing": drawing, "steps": len(steps), "mismatches": mismatches}

JOBS = {"a7_before": job_lane_end, "a7_after": job_lane_end, "m1": job_m1, "control": job_control,
        "car20": job_car20_replay}


def run_job(name: str, repo: Path, workdir: Path) -> dict:
    output = workdir / f"{name}.json"
    subprocess.run([sys.executable, __file__, "--job", name, "--repo", str(repo), "--out", str(output)], check=True,
                   env={**__import__("os").environ, "OPENBLAS_NUM_THREADS": "1", "MPLBACKEND": "Agg"})
    return json.loads(output.read_text())


def checkout(commit: str, workdir: Path) -> Path:
    target = workdir / commit
    if not target.exists():
        target.mkdir(parents=True)
        archive = subprocess.run(["git", "-C", str(REPO), "archive", commit], check=True, capture_output=True).stdout
        subprocess.run(["tar", "-x", "-C", str(target)], input=archive, check=True)
    return target


# ---------------------------------------------------------------------------------------------------------------------
# Simulated and replayed figures
# ---------------------------------------------------------------------------------------------------------------------
def lane_end_figures(before: dict, after: dict, m1: dict):
    cases = list(before["stops"])
    figure, (left, right) = plt.subplots(1, 2, figsize=(13, 4.0), gridspec_kw={"width_ratios": [1.2, 1]})
    left.axhspan(0.0, 0.5, color=GOOD, alpha=0.12)
    left.text(9.45, 0.42, "R5 stop zone: 0 to 0.5 m past the last pair", fontsize=8, ha="right", color=GOOD)
    width = 0.38
    b1 = left.bar([i - width / 2 for i in range(len(cases))], [before["stops"][c] for c in cases], width, color=BAD,
                  label=f"before A7 ({COMMITS['a7_before']}): stops when the last cones leave the view")
    b2 = left.bar([i + width / 2 for i in range(len(cases))], [after["stops"][c] for c in cases], width, color=GOOD,
                  label=f"A7 rule ({COMMITS['a7_after']}): follow the last plan, stop at the roll-out point")
    label_bars(left, b2, "{:.2f}", size=7)
    for bar in b1:
        left.annotate(f"{bar.get_height():.2f}", (bar.get_x() + bar.get_width() / 2, bar.get_height()), ha="center",
                      va="top", fontsize=7, xytext=(0, -2), textcoords="offset points")
    left.axhline(0, color="#444", lw=0.8)
    left.set_xticks(range(len(cases)), cases)
    left.set_ylim(-1.05, 0.65)
    left.set_ylabel("stop past the last cone pair (m)")
    left.set_title("Lane end: where the car stops (simulation, tuning cases T1-T10)")
    left.legend(fontsize=7.5, frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=1)

    t12 = m1["t12"]
    blackout = m1["t12_blackout_m"]
    lost_t = next(t for t, x in zip(t12["t"], t12["x"]) if x >= blackout)
    request_t = next(t for t, speed, _ in t12["commands"] if t > lost_t and speed == 0.0)
    stop_t = next(t for t, s in zip(t12["t"], t12["speed"]) if t > request_t and s < 0.02)
    right.plot(t12["t"], t12["speed"], color=ACCENT, lw=2, label="car speed")
    right.step([c[0] for c in t12["commands"]], [c[1] for c in t12["commands"]], where="post", color=NEUTRAL, lw=1.2,
               label="requested speed")
    right.axvspan(lost_t, request_t, color=AMBER, alpha=0.15)
    right.axvline(lost_t, color=AMBER, ls=":", lw=1)
    right.axvline(request_t, color=BAD, ls="--", lw=1)
    right.text(lost_t - 0.08, 0.15, f"camera covered\nat {blackout:g} m", fontsize=8, ha="right", color=AMBER)
    right.text(request_t + 0.08, 0.82, f"stop request\n{t12['metrics']['lane_lost_to_stop_request_s']:.2f} s later",
               fontsize=8, color=BAD)
    stop_x = next(x for t, x in zip(t12["t"], t12["x"]) if t >= stop_t)
    right.text(stop_t + 0.05, 0.05, f"stopped at x = {stop_x:.1f} m\n(lane ends at {m1['t12_lane_length_m']:g} m)",
               fontsize=8)
    right.set_xlim(0, stop_t + 1.6)
    right.set_ylim(0, 1.15)
    right.set_xlabel("time (s)")
    right.set_ylabel("speed (m/s)")
    right.set_title(f"Lost lane, T12 ({COMMITS['m1']}): half speed for 0.5 s, then stop")
    right.legend(fontsize=8, frameon=False, loc="upper right")
    save(figure, "03_lane_end.png")

    figure, axis = plt.subplots(figsize=(4.4, 2.3))
    axis.axhspan(0.0, 0.5, color=GOOD, alpha=0.12)
    axis.bar([i - width / 2 for i in range(len(cases))], [before["stops"][c] for c in cases], width, color=BAD,
             label="before")
    axis.bar([i + width / 2 for i in range(len(cases))], [after["stops"][c] for c in cases], width, color=GOOD,
             label="A7 rule")
    axis.axhline(0, color="#444", lw=0.8)
    axis.set_xticks(range(len(cases)), cases, fontsize=7)
    axis.tick_params(labelsize=7)
    axis.set_ylim(-1.0, 0.7)
    axis.set_ylabel("stop past last\ncone pair (m)", fontsize=8)
    axis.legend(fontsize=7, frameon=False, loc="lower right", ncol=2)
    axis.set_title("Lane end, T1-T10 (simulation): zone 0-0.5 m", fontsize=9)
    save(figure, "slide_lane_end.png")


def m1_figure(m1: dict):
    figure, axes = plt.subplots(1, 3, figsize=(15, 4.0), gridspec_kw={"width_ratios": [1.4, 1, 0.9]})
    colours = {"pure pursuit (A3)": NEUTRAL, "MPC (A4)": GOOD}
    axis = axes[0]
    missed = {tuple(c) for c in m1["u2_missed"]}
    wrong = {tuple(c) for c in m1["u2_wrong"]}
    for colour, index, x, y in m1["u2_cones"]:
        face = "#e0b000" if colour == "YELLOW" else ACCENT
        if (colour, index) in missed:
            axis.plot(x, y, "o", ms=4, mfc="none", mec=face)
        else:
            axis.plot(x, y, "o", ms=3.2, color=face)
        if (colour, index) in wrong:
            axis.annotate("seen as blue", (x, y), xytext=(0, -14), textcoords="offset points", fontsize=7, ha="center")
    axis.annotate("missed\n(hollow)", (2.25, -0.5), xytext=(0, -26), textcoords="offset points", fontsize=7,
                  ha="center")
    for name, runs in m1["unseen"].items():
        run = runs["U2"]
        axis.plot(run["x"], run["y"], color=colours[name], lw=2, label=f"{name}: {'pass' if run['passed'] else 'FAIL'}")
    axis.set_ylim(-0.75, 0.75)
    axis.set_xlabel("along the lane (m)")
    axis.set_ylabel("left of the centre (m)")
    axis.set_title("U2: starts 0.1 m left, 15 deg off; 2 missed cones, 1 wrong colour", fontsize=9)
    axis.legend(fontsize=8, frameon=False, loc="center right")
    axis = axes[1]
    for name, runs in m1["unseen"].items():
        commands = runs["U2"]["commands"]
        axis.step([c[0] for c in commands], [c[2] for c in commands], where="post", color=colours[name], lw=1.4,
                  label=f"{name}: R4 = {runs['U2']['metrics']['rms_curvature_rate_per_m_s']:.2f}")
    axis.set_xlabel("time (s)")
    axis.set_ylabel("requested curvature (1/m, + = left)")
    axis.set_title("U2: the MPC steers more smoothly", fontsize=9)
    axis.legend(fontsize=8, frameon=False, loc="upper right")
    axis = axes[2]
    designs = list(m1["unseen"]) + ["RL balanced 1M (A5)"]
    values = {name: [runs[c]["metrics"]["rms_curvature_rate_per_m_s"] for c in ("U1", "U2", "U3")]
              for name, runs in m1["unseen"].items()}
    values["RL balanced 1M (A5)"] = [1.51, 1.59, 0.50]  # docs/stage1-design.md M1 (needs stable-baselines3 to rerun)
    width = 0.27
    for i, name in enumerate(designs):
        colour = {"pure pursuit (A3)": NEUTRAL, "MPC (A4)": GOOD}.get(name, BAD)
        bars = axis.bar([j + (i - 1) * width for j in range(3)], values[name], width, color=colour, label=name)
        label_bars(axis, bars, "{:.2f}", size=6.5)
    axis.axhline(1.0, color="#444", ls="--", lw=1)
    axis.text(2.45, 1.03, "R4 limit 1.0", fontsize=7.5, ha="right")
    axis.set_xticks(range(3), ["U1", "U2", "U3"])
    axis.set_ylim(0, 1.95)
    axis.set_ylabel("curvature-rate RMS (1/(m s))")
    axis.set_title("M1 on the unseen cases: RL fails R4 twice", fontsize=9)
    axis.legend(fontsize=7, frameon=False, loc="upper left")
    figure.text(0.5, -0.03, f"Simulation at {COMMITS['m1']} with the cases' own noise and the 0.2 s detection age. "
                "The RL bars are copied from docs/stage1-design.md M1.", ha="center", fontsize=8.5)
    save(figure, "05_m1_unseen.png")


def control_figure(control: dict):
    labels = list(CONTROL_SETTINGS)
    colours = dict(zip(labels, (NEUTRAL, BAD, GOOD)))
    figure, axes = plt.subplots(1, 3, figsize=(15, 3.8), sharey=True)
    for axis, car in zip(axes, control["cars"]):
        for label in labels:
            run = control["runs"][f"{car}|{label}"]
            axis.plot(run["t"], run["speed"], color=colours[label], lw=1.8, label=label)
        axis.axhline(0.5, color="#444", ls=":", lw=1, label="target 0.5 m/s")
        axis.set_title(f"{car} model", fontsize=10)
        axis.set_xlabel("time (s)")
        axis.set_ylim(0, 1.75)
    axes[0].set_ylabel("speed (m/s)")
    axes[0].legend(fontsize=7.5, frameon=False, loc="upper right")
    axes[1].text(5.6, 1.62, "car 22 on the floor, 12 Oct: about 1.4 m/s\n(up to 1.7) with car 20's values;"
                 " 0.46-0.49 m/s\nwith car 22's", fontsize=7.5, va="top", color="#444")
    figure.text(0.5, -0.04, f"Simulation at {COMMITS['control']}: T1 at 0.5 m/s, MPC -> CarController -> a measured "
                "car model (first-order speed response, inverted steering map). car22a and car22b are two equally "
                "good fits of one car 22 run.", ha="center", fontsize=8.5)
    save(figure, "06_car_models.png")


def _draw_layers(axis, drawing, names):
    for name in names:
        layer = drawing[name]
        for (x, y, z), (length, width, _), colour in layer["boxes"]:
            axis.add_patch(Rectangle((x - length / 2, y - width / 2), length, width, fc=colour[:3], alpha=0.5,
                                     ec="#444", lw=0.8))
        for points, colour in layer["lines"]:
            if len(points) > 1:
                axis.plot([p[0] for p in points], [p[1] for p in points], color=colour[:3], alpha=colour[3], lw=2)
            else:
                axis.plot(points[0][0], points[0][1], "o", color=colour[:3], ms=5)
        for x, y, diameter, colour in layer["spheres"]:
            axis.plot(x, y, "o", color=colour[:3], alpha=colour[3], ms=3 + diameter * 40, mec="#333", mew=0.4)


def car20_figure(replay: dict):
    drawing = replay["drawing"]
    figure, axis = plt.subplots(figsize=(8.2, 4.6))
    _draw_layers(axis, drawing, ["car", "cones/detected", "cones/used", "memory/cones", "lane", "plan/avoidance",
                                 "plan/mpc"])
    axis.set_aspect("equal")
    axis.set_xlim(-0.6, 2.8)
    axis.set_ylim(-1.0, 1.0)
    axis.grid(alpha=0.25)
    axis.set_xlabel("x forwards in base_link (m)")
    axis.set_ylabel("y left (m)")
    handles = [plt.Line2D([], [], marker="o", ls="", color=(0.18, 0.5, 0.93), label="cones used: blue (left)"),
               plt.Line2D([], [], marker="o", ls="", color=(0.95, 0.79, 0.3), label="cones used: yellow (right)"),
               plt.Line2D([], [], marker="o", ls="", ms=3, color=(0.9, 0.2, 0.2), label="every detection"),
               plt.Line2D([], [], color=(0.2, 0.85, 0.3), lw=2, label="lane edges and centre"),
               plt.Line2D([], [], color=(0.9, 0.3, 0.9), lw=2, label="usual line (mpc.lane_side)"),
               plt.Line2D([], [], color=(0.2, 0.85, 0.95), lw=2, label="MPC predicted path")]
    axis.legend(handles=handles, fontsize=7.5, frameon=False, loc="upper left", bbox_to_anchor=(1.01, 1.0))
    axis.text(1.04, 0.38, drawing["summary"], transform=axis.transAxes, fontsize=7, va="top", family="monospace",
              color="#444")
    axis.set_title(f"Car 20 data log {CAR20_LOG.name}: step at {drawing['t']:.1f} s, wheel speed "
                   f"{drawing['speed']:.2f} m/s", fontsize=9)
    figure.text(0.5, -0.03, f"Cones and wheel speed are recorded on the car. The lane and the plan are this checkout's "
                f"replay (mcap_export), which chose other actions than the car on {replay['mismatches']} of "
                f"{replay['steps']} steps.", ha="center", fontsize=8)
    save(figure, "08_car20_replay.png")

    figure, axis = plt.subplots(figsize=(4.4, 2.3))
    _draw_layers(axis, drawing, ["car", "cones/detected", "cones/used", "lane", "plan/mpc"])
    axis.set_aspect("equal")
    axis.set_xlim(-0.5, 2.6)
    axis.set_ylim(-0.95, 0.95)
    axis.tick_params(labelsize=7)
    axis.set_title("Car 20 data log, replayed in the Foxglove layers", fontsize=9)
    save(figure, "slide_tools.png")


def simulated(workdir: Path):
    data = {}
    for name in ("a7_before", "a7_after", "m1", "control"):
        print("simulating", name, "at", COMMITS[name])
        data[name] = run_job(name, checkout(COMMITS[name], workdir), workdir)
    lane_end_figures(data["a7_before"], data["a7_after"], data["m1"])
    m1_figure(data["m1"])
    control_figure(data["control"])
    if Path(f"{CAR20_LOG}_observations.json").exists():
        print("replaying", CAR20_LOG.name)
        car20_figure(run_job("car20", REPO, workdir))
    else:
        print("skip car 20 figure: no", CAR20_LOG)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--simulate", action="store_true", help="also run the simulations and the car 20 replay")
    parser.add_argument("--workdir", type=Path, help="folder for the extracted commits (default: a temporary one)")
    parser.add_argument("--job", choices=sorted(JOBS), help=argparse.SUPPRESS)
    parser.add_argument("--repo", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--out", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.job:
        args.out.write_text(json.dumps(JOBS[args.job](args.repo.resolve())))
        return
    system_diagram()
    slide_figures()
    rl_reward()
    design_front()
    car_timing()
    if args.simulate:
        if args.workdir:
            args.workdir.mkdir(parents=True, exist_ok=True)
            simulated(args.workdir)
        else:
            with tempfile.TemporaryDirectory() as workdir:
                simulated(Path(workdir))


if __name__ == "__main__":
    main()
