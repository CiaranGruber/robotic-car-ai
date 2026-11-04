"""Makes the figures in Toby's logbook section (docs/logbook/toby/figures/).

    python3 docs/logbook/toby/make_figures.py                        # summary figures only (numbers below)
    python3 docs/logbook/toby/make_figures.py --lane-repo <3ab7ac9 checkout> --sim-repo <740c575 checkout>

The trajectories and the turn geometry run tests/policy/marker_simulation.py and policy/action_policy/intersection.py
from --sim-repo. The logbook's figures used origin/fix/left-turn-steering-and-obstacles (740c575). The numbers in the
summary figures are copied from the documents named beside each one, so they can be checked there.
"""
import argparse
import dataclasses
import math
import re
import subprocess
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Arc, Circle, FancyBboxPatch, Rectangle  # noqa: E402

HERE = Path(__file__).resolve().parent
OUT = HERE / "figures"
REPO = HERE.parents[2]
BAD, GOOD, NEUTRAL, ACCENT = "#c8553d", "#2a7f62", "#8a8a8a", "#1f5fd1"
plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False, "savefig.dpi": 150})


def save(figure, name):
    OUT.mkdir(parents=True, exist_ok=True)
    figure.savefig(OUT / name, bbox_inches="tight")
    plt.close(figure)
    print("wrote", OUT / name)


def label_bars(axis, bars, fmt="{:.0f}"):
    for bar in bars:
        value = bar.get_height()
        axis.annotate(fmt.format(value), (bar.get_x() + bar.get_width() / 2, value), ha="center", va="bottom",
                      fontsize=8, xytext=(0, 2), textcoords="offset points")


def system_diagram():
    """Where my parts sit in the MovementPolicy, with the interfaces between them."""
    figure, axis = plt.subplots(figsize=(12.6, 4.6))
    axis.set_xlim(0, 12.6)
    axis.set_ylim(0, 4.6)
    axis.axis("off")
    boxes = {
        "cam": (0.1, 3.3, "OAK-D camera\n(oakd_cone_detector)", NEUTRAL),
        "aruco": (0.1, 1.9, "aruco_detector\nFiducialBatch:\nid, pose in base_link", NEUTRAL),
        "lidar": (0.1, 0.5, "Lidar, wheel speed,\nIMU heading", NEUTRAL),
        "lanes": (2.9, 3.3, "Cone filter, local map,\nlane detection\n(teammates)", NEUTRAL),
        "reader": (2.9, 1.9, "MarkerReader\nnearest marker <= 2 m,\n2 steps in a row", ACCENT),
        "obst": (2.9, 0.5, "Obstacle detection\n+ avoidance (teammates;\nmy marker/cone filters)", NEUTRAL),
        "memory": (5.7, 1.9, "CarMemory\nlane side, intersection\ncommand, turn phase", ACCENT),
        "mpc": (8.5, 3.3, "MPC lane follower\nlane_side -> offset line\ncar 20/21 limits", ACCENT),
        "turn": (8.5, 1.9, "IntersectionTurner\nAPPROACH -> ARC -> EXIT", ACCENT),
        "ctrl": (8.5, 0.5, "CarController\n(speed, curvature) ->\nthrottle, steering", NEUTRAL),
    }
    for x, y, text, colour in boxes.values():
        axis.add_patch(FancyBboxPatch((x, y), 2.4, 1.0, boxstyle="round,pad=0.04", fc="white", ec=colour,
                                      lw=2.2 if colour == ACCENT else 1.2))
        axis.text(x + 1.2, y + 0.5, text, ha="center", va="center", fontsize=8.5)

    def arrow(a, b, text="", dy=0.0):
        ax_, ay, _, _ = boxes[a]
        bx, by, _, _ = boxes[b]
        start = (ax_ + 2.45, ay + 0.5 + dy) if bx > ax_ else (ax_ + 1.2, ay - 0.02)
        end = (bx - 0.05, by + 0.5 + dy) if bx > ax_ else (bx + 1.2, by + 1.02)
        axis.annotate("", end, start, arrowprops=dict(arrowstyle="->", lw=1.2))
        if text:
            axis.text((start[0] + end[0]) / 2, (start[1] + end[1]) / 2 + 0.08, text, ha="center", fontsize=7.5,
                      color="#444")

    arrow("cam", "lanes", "cones")
    arrow("cam", "aruco", "")
    arrow("aruco", "reader", "fiducials")
    arrow("lidar", "obst", "points")
    arrow("reader", "memory", "command")
    arrow("lanes", "mpc", "LaneDetection (centre line, width)")
    arrow("memory", "turn", "")
    axis.text(8.2, 2.1, "TURN_*,\nSTRAIGHT", fontsize=7.5, color="#444", ha="center")
    arrow("obst", "ctrl", "obstacle stop (always on);\navoidance line -> MPC")
    axis.annotate("", (9.7, 3.28), (9.7, 2.92), arrowprops=dict(arrowstyle="<->", lw=1.2))
    axis.text(10.95, 3.1, "moves lane side aside\nbefore a turn", fontsize=7.5, color="#444", ha="left")
    axis.annotate("", (9.7, 1.52), (9.7, 1.86), arrowprops=dict(arrowstyle="->", lw=1.2))
    axis.text(9.78, 1.58, "(speed, curvature) from the turner\nduring a turn, else from the MPC", fontsize=7.5,
              color="#444")
    axis.annotate("", (8.45, 3.6), (8.15, 2.55), arrowprops=dict(arrowstyle="->", lw=1.2))
    axis.text(7.0, 3.05, "KEEP_LEFT/RIGHT/\nCENTRE -> lane_side", fontsize=7.5, color="#444")
    axis.text(12.5, 4.45, "blue outline = parts I built or tuned", ha="right", fontsize=8, color=ACCENT)
    axis.text(0.1, 4.45, "Policy step every 0.1 s (budget 20 ms); policy never enables the vehicle", fontsize=8,
              color="#444")
    save(figure, "01_system.png")


def mpc_tuning():
    """docs/mpc-lab-calibration.md section 4: lanes kept of 84 curved-lane runs (lane detection, curves on)."""
    names = ["dev defaults\n(3.0 both ways,\nheading 1, change 1)", "car 20 run\n(0.5 both ways,\nheading 5)",
             "chosen\n(0.48 L / 0.92 R,\nheading 1, change 2)", "chosen,\ntrim -0.05\nuncompensated"]
    kept = [84, 73, 84, 72]
    rms = [0.27, 0.71, 0.20, 0.24]
    figure, (left, right) = plt.subplots(1, 2, figsize=(13, 3.9))
    bars = left.bar(names, kept, color=[NEUTRAL, BAD, GOOD, NEUTRAL])
    label_bars(left, bars)
    left.set_ylabel("lanes kept (of 84 simulated runs)")
    left.set_ylim(0, 95)
    left.set_title("Curved lanes: 1.5-3 m radius, S bend, camera yaw -3..+3 deg")
    left.tick_params(axis="x", labelsize=7.5)
    bars = right.bar(names, rms, color=[NEUTRAL, BAD, GOOD, NEUTRAL])
    label_bars(right, bars, "{:.2f}")
    right.set_ylabel("curvature-rate RMS (1/m per s)")
    right.set_title("Steering activity (lower = smoother)")
    right.tick_params(axis="x", labelsize=7.5)
    save(figure, "02_mpc_tuning.png")


def lane_side_room():
    """Offset available beside the centre line: w/2 - edge_clearance (docs/marker-commands.md, Lane changes)."""
    widths = [0.8, 1.0]
    clearances = [0.40, 0.28, 0.25]
    figure, axis = plt.subplots(figsize=(6.5, 3.6))
    step = 0.25
    for i, clearance in enumerate(clearances):
        values = [max(0.0, w / 2 - clearance) * 100 for w in widths]
        bars = axis.bar([j + (i - 1) * step for j in range(2)], values, step,
                        color=[BAD, GOOD, NEUTRAL][i], label=f"edge_clearance_m = {clearance:.2f}"
                        + (" (old)" if i == 0 else " (chosen)" if i == 1 else ""))
        label_bars(axis, bars)
    axis.set_xticks(range(2), [f"{w:g} m lane" for w in widths])
    axis.set_ylabel("offset from centre each side (cm)")
    axis.axhline(5, color="#444", ls=":", lw=1)
    axis.text(1.45, 5.6, "~ visible", fontsize=8, ha="right")
    axis.legend(fontsize=8, frameon=False)
    axis.set_title("Why the lane change was not visible: room = w/2 - clearance")
    save(figure, "03_lane_side_room.png")


def root_cause_bars():
    """logs/sim-2026-11-02/RESULTS.md nominal matrix (10 seeds) and the pushed branch numbers."""
    cases = ["left turn\n1 m road", "left turn\n0.8 m road", "right turn", "straight"]
    before = [10, 0, 88, 100]
    after = [95, 100, 100, 100]
    figure, axis = plt.subplots(figsize=(6.5, 3.6))
    width = 0.38
    b1 = axis.bar([i - width / 2 for i in range(4)], before, width, color=BAD,
                  label="30 Oct code: left limit 0.48 1/m (car 20, +-0.5 steering)")
    b2 = axis.bar([i + width / 2 for i in range(4)], after, width, color=GOOD,
                  label="fix: +-0.8 steering, left 0.91 1/m, lookahead 0.8 m, obstacle filters")
    label_bars(axis, b1)
    label_bars(axis, b2)
    axis.set_xticks(range(4), cases)
    axis.set_ylabel("simulated pass rate (%)")
    axis.set_ylim(0, 125)
    axis.legend(fontsize=7.5, frameon=False, loc="upper left")
    axis.set_title("Simulation reproduced 'right works, left fails' (20-40 runs per bar)")
    save(figure, "06_root_cause_bars.png")


def sensitivity():
    """logs/sim-2026-11-02/RESULTS.md robustness tables (v5, 5 seeds x 10 cases per condition)."""
    figure, (left, right) = plt.subplots(1, 2, figsize=(12, 3.9), gridspec_kw={"width_ratios": [1, 1.25]})
    reach = [0.55, 0.7, 0.85, 1.0]
    as_is = [0, 0, 90, 100]
    matched = [50, 50, 100, 100]
    left.plot(reach, as_is, "o-", color=BAD, label="config left limit left at 0.91 1/m")
    left.plot(reach, matched, "s-", color=GOOD, label="config set to the measured reach")
    left.axvline(0.91, color="#444", ls=":", lw=1)
    left.text(0.905, 8, "configured\n0.91", fontsize=8, ha="right")
    left.set_xlabel("real left steering reach of the simulated car (1/m)")
    left.set_ylabel("left-turn pass rate (%), 20 runs")
    left.set_ylim(-5, 110)
    left.set_title("Steering reach must be measured, not assumed")
    left.legend(fontsize=8, frameon=False, loc="center right")
    conditions = ["nominal", "camera yaw\n+4 deg", "camera yaw\n-4 deg", "marker\n1.5 m early", "latency x2",
                  "cone dropout\n25%", "cone jitter\n3 cm"]
    uncal = [99, 60, 56, 20, 92, 96, 68]
    cal = [99, 100, 100, 100, None, None, None]
    xs = range(len(conditions))
    b1 = right.bar([x - 0.2 for x in xs], uncal, 0.4, color=BAD, label="unknown to the policy")
    label_bars(right, b1)
    cal_x = [x + 0.2 for x, v in zip(xs, cal) if v is not None]
    b2 = right.bar(cal_x, [v for v in cal if v is not None], 0.4, color=GOOD,
                   label="measured and set in config (yaw offset, marker_to_entry_m)")
    label_bars(right, b2)
    right.set_xticks(list(xs), conditions, fontsize=7.5, rotation=25, ha="right")
    right.set_ylabel("pass rate, all 10 cases (%), 50 runs")
    right.set_ylim(0, 125)
    right.legend(fontsize=8, frameon=False, loc="upper right")
    right.set_title("What the car setup must match (v5 code)")
    save(figure, "07_sensitivity.png")


def car13_offsets():
    """Real car 13 log, 27 Oct (logs/car13/policy.log): every avoidance plan and marker command, in log order."""
    log = REPO / "logs" / "car13" / "policy.log"
    if not log.exists():
        print("skip car13 figure: no", log)
        return
    offsets, markers = [], []
    for line in log.read_text().splitlines():
        found = re.search(r"planned a line around it, (-?[\d.]+) m left", line)
        if found:
            offsets.append(float(found.group(1)))
        elif "KEEP_RIGHT" in line:
            markers.append(len(offsets))
    figure, axis = plt.subplots(figsize=(7.5, 3.3))
    axis.stem(range(1, len(offsets) + 1), offsets, linefmt=BAD, markerfmt="o", basefmt="#444")
    for i, m in enumerate(markers):
        axis.axvline(m + 0.5, color=ACCENT, lw=1, alpha=0.6, label="Marker 14: KEEP_RIGHT logged" if i == 0 else None)
    axis.set_xlabel("avoidance plan number, in log order (one short run, no obstacle on the track)")
    axis.set_ylabel("planned offset (m, + = left)")
    axis.set_title(f"Car 13, 27 Oct: {len(offsets)} avoidance lines planned around obstacles that were not there")
    axis.legend(fontsize=8, frameon=False, loc="lower left")
    save(figure, "09_car13_phantom_avoidance.png")


def draw_world(axis, world):
    colours = {"BLUE": "#1f5fd1", "YELLOW": "#e0b000"}
    for colour, _, cx, cy in world.cones:
        axis.plot(cx, cy, "o", ms=2.6, color=colours.get(getattr(colour, "name", ""), "grey"))
    for marker in world.markers:
        axis.plot(marker.x_m, marker.y_m, "s", ms=6, color="black")
        axis.annotate(str(marker.marker_id), (marker.x_m, marker.y_m), fontsize=6.5, xytext=(3, 4),
                      textcoords="offset points")
    axis.set_aspect("equal")
    axis.grid(alpha=0.25)
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")


def load_simulation(repo: Path):
    """Imports marker_simulation and the policy from a checkout (once per process)."""
    sys.path[:0] = [str(repo), str(repo / "tests" / "policy")]
    import marker_simulation as ms
    settings = ms.read_settings(repo / "config" / "ai4r_policy.yaml")
    return ms, settings, {case.name: case for case in ms.CASES}


def lane_changes(lane_repo: Path):
    """Lane changes as tuned on 27 Oct (3ab7ac9, keep_left 0 / keep_right 9 / keep_centre 1). Today's dev maps no
    keep-side IDs (Yuhao's grouped IDs), so its marker simulation has no lane-change cases."""
    ms, settings, cases = load_simulation(lane_repo)

    # Lane change, 1 m road
    figure, axes = plt.subplots(2, 1, figsize=(10, 6.2), gridspec_kw={"hspace": 0.45})
    for axis, name in zip(axes, ("lanes_1m", "lanes_0.8m")):
        case = cases[name]
        draw_world(axis, case.world)
        for seed in range(3):
            run = ms.run_case(case, settings, seed)
            axis.plot(run.x, run.y, "-", color=GOOD if run.ok else BAD, lw=1.3, alpha=0.85,
                      label=f"seed {seed}: {run.outcome}")
        width = case.world.roads[0].width_m
        room = width / 2 - settings["mpc.edge_clearance_m"]
        for value, text in ((room, "LEFT line"), (-room, "RIGHT line")):
            axis.axhline(value, color="#444", ls=":", lw=0.9)
            axis.text(case.world.roads[0].length_m + 0.15, value, text, fontsize=7, va="center")
        axis.set_aspect("auto")
        axis.set_xlim(-0.8, 10.2)
        axis.set_ylim(-0.85, 0.85)
        axis.set_title(f"{name}: markers (squares, by ID) ask KEEP_LEFT at x=1, KEEP_RIGHT at 4, KEEP_CENTRE at 7 m;"
                       f" offset lines at +-{room:.2f} m", fontsize=9)
        axis.legend(fontsize=7, frameon=False, loc="upper right", ncol=3, bbox_to_anchor=(1.0, 1.0))
    save(figure, "04_lane_change_sim.png")


def simulated(sim_repo: Path):
    ms, settings, cases = load_simulation(sim_repo)
    from policy.action_policy.intersection import arc_start_m, CONE_RADIUS_M

    # Turns: fixed config, and the 30 Oct left-turn failure
    old = {**settings, "mpc.max_curvature_per_m": 0.48, "intersection.lookahead_m": 0.5}
    old_car = dataclasses.replace(ms.CAR21, max_left_curvature_per_m=0.50)
    panels = (("cross_left_1m", old, old_car, "30 Oct: left limit 0.48 1/m"),
              ("cross_left_1m", settings, ms.CAR21, "fix: left 0.91 1/m"),
              ("cross_left_0.8m", settings, ms.CAR21, "fix: left turn, narrow roads"),
              ("cross_right_0.8m", settings, ms.CAR21, "fix: right turn (moves aside left first)"))
    figure, axes = plt.subplots(1, 4, figsize=(15, 4.3))
    for axis, (name, config, car, title) in zip(axes, panels):
        case = cases[name]
        draw_world(axis, case.world)
        outcomes = []
        for seed in range(5):
            run = ms.run_case(case, config, seed, car=car)
            outcomes.append(run.ok)
            axis.plot(run.x, run.y, "-", color=GOOD if run.ok else BAD, lw=1.2, alpha=0.8)
        axis.set_title(f"{title}\n{name}: {sum(outcomes)}/5 passed", fontsize=9)
        if not any(outcomes):
            axis.text(-3.4, -2.6, "stops at the marker: the 2.08 m left arc\nhas no clear start in a 1 m crossing",
                      fontsize=8, color=BAD)
    figure.text(0.5, -0.02, "green = left by the commanded road without touching a cone; red = failed "
                "(stopped, touched cones or left the road); 5 noise seeds per panel", ha="center", fontsize=9)
    save(figure, "05_turn_sim.png")

    # Turn geometry, right turn in 0.8 m roads after moving aside
    width, radius, half, front, margin = 0.8, 1.2, 0.125, 0.30, 0.03
    away = width / 2 - 0.25  # 0.25 m from the left edge
    start, feasible = arc_start_m(width, width, radius, away, half, front, margin)
    _, centre_feasible = arc_start_m(width, width, radius, 0.0, half, front, margin)
    figure, axis = plt.subplots(figsize=(6.8, 6.4))
    axis.add_patch(Rectangle((-2.2, -width / 2), 2.2, width, fc="#f2f2f2", ec="none"))
    axis.add_patch(Rectangle((0, -2.0), width, 2.0 + width / 2 + 1.0, fc="#f2f2f2", ec="none"))
    for y in (-width / 2, width / 2):
        axis.plot([-2.2, 0], [y, y], "o", ms=3, color="#e0b000" if y < 0 else "#1f5fd1",
                  markevery=None, ls="--", lw=0.5)
    axis.plot([0, 0], [-2.0, -width / 2], "--o", ms=3, lw=0.5, color="#e0b000")
    axis.plot([width, width], [-2.0, width / 2 + 1.0], "--o", ms=3, lw=0.5, color="#1f5fd1")
    centre = (start, away - radius)
    axis.add_patch(Arc(centre, 2 * radius, 2 * radius, theta1=0, theta2=90, ec=ACCENT, lw=2.4))
    axis.plot([start + radius] * 2, [away - radius, -2.0], color=ACCENT, lw=2.4)
    axis.add_patch(Arc(centre, 2 * (radius - half), 2 * (radius - half), theta1=-10, theta2=100, ec=GOOD, ls="--"))
    reach = math.hypot(front, radius + half)
    axis.add_patch(Arc(centre, 2 * reach, 2 * reach, theta1=-25, theta2=100, ec=BAD, ls="--"))
    axis.add_patch(Circle((0, -width / 2), CONE_RADIUS_M + margin, fc="none", ec="black", lw=1))
    axis.plot([-2.2, start], [away, away], color=ACCENT, lw=2)
    axis.plot(*centre, "k+")
    axis.annotate("arc centre", centre, xytext=(-60, -12), textcoords="offset points", fontsize=8)
    axis.text(-2.15, 0.62, f"approach path, moved aside {away:.2f} m left\n(0.25 m from the left edge)", fontsize=8,
              color=ACCENT)
    axis.annotate(f"arc start: {-start:.2f} m before the entry", (start, away), xytext=(-150, -45),
                  textcoords="offset points", fontsize=8, arrowprops=dict(arrowstyle="->"))
    axis.text(-1.0, -0.65, f"inner body side\nR - w/2 = {radius - half:.3f} m", fontsize=8, color=GOOD)
    axis.text(0.62, 0.55, f"outer front corner\nsqrt(f^2 + (R + w/2)^2)\n= {reach:.3f} m", fontsize=8, color=BAD)
    axis.text(-2.15, -1.95, f"R = {radius} m, car width w = {2 * half} m, front f = {front} m (base_link to bumper)",
              fontsize=8)
    axis.annotate("inside corner cone\n+ 3 cm margin", (0, -width / 2), xytext=(12, -28), textcoords="offset points",
                  fontsize=8, arrowprops=dict(arrowstyle="->"))
    axis.annotate("exit far row", (width, -1.6), xytext=(8, 0), textcoords="offset points", fontsize=8)
    axis.annotate("exit lane\ncentre", (width / 2, -1.85), xytext=(-55, 0), textcoords="offset points", fontsize=8)
    axis.set_xlim(-2.2, 1.6)
    axis.set_ylim(-2.0, 1.4)
    axis.set_aspect("equal")
    axis.set_xlabel("along the approach, from the intersection entry (m)")
    axis.set_ylabel("left of the approach centre line (m)")
    axis.set_title(f"Right turn, 0.8 m roads, R = {radius} m: from the centre line feasible = "
                   f"{centre_feasible}; moved aside feasible = {feasible}", fontsize=9)
    save(figure, "08_turn_geometry.png")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sim-repo", type=Path, help="checkout to run the turn simulation from (740c575)")
    parser.add_argument("--lane-repo", type=Path, help="checkout to run the lane-change simulation from (3ab7ac9)")
    parser.add_argument("--only-lanes", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--only-turns", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.only_lanes:
        return lane_changes(args.only_lanes)
    if args.only_turns:
        return simulated(args.only_turns)
    system_diagram()
    mpc_tuning()
    lane_side_room()
    root_cause_bars()
    sensitivity()
    car13_offsets()
    # Each checkout runs in its own process, so the two versions of the policy package never mix
    if args.lane_repo:
        subprocess.run([sys.executable, __file__, "--only-lanes", str(args.lane_repo.resolve())], check=True)
    if args.sim_repo:
        subprocess.run([sys.executable, __file__, "--only-turns", str(args.sim_repo.resolve())], check=True)


if __name__ == "__main__":
    main()
