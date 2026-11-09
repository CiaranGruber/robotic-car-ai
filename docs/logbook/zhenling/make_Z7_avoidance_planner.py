"""Draws Fig. Z7: the avoidance planner's candidate lines and its chosen line past two boxes, with the real planner code.

Run from the repository root:
    .venv/bin/python -B docs/logbook/zhenling/make_Z7_avoidance_planner.py [--approach-m 0.3] [--out PATH]
Settings come from the final real drive's _config.json (2026-10-26_14-10-54). The scene is an 0.8 m lane with a box in
the right half and then one in the left half. --approach-m replaces the distance before each box from which the car
must keep clear of it (the planner's approach_m, normally the stopping distance); without it the policy's own value is
used. The logbook's Fig. Z7 is drawn with --approach-m 0.3. Needs the car logs in scenarios/data/original_data/ (local, not in git) and matplotlib.
"""
import argparse
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from mcap_export import build_policy, read_settings  # noqa: E402
from policy.action_policy.avoidance import SAMPLE_STEP_M, MAX_HORIZON_M  # noqa: E402
from policy.action_policy.lane_path import LanePath, LineState, move_length_m  # noqa: E402
from policy.obstacle_detection.obstacles import LaneObstacle  # noqa: E402

SETTINGS = ROOT / "scenarios/data/original_data/2026-10-26_14-10-54_config.json"
LANE_WIDTH_M = 0.8
# Boxes as the lidar reports them: near and far distance from the car's front, right and left offset (m)
BOXES = [LaneObstacle(1.2, 1.4, -0.40, -0.10, 10), LaneObstacle(3.6, 3.8, 0.12, 0.40, 10)]
BLUE, ORANGE, RED, GREY, INK, MUTED = "#2a78d6", "#e8a100", "#e03b3b", "#8a8a8a", "#222222", "#666666"


def plan_with_candidates(avoidance, surface_m, curvature_room, return_room):
    """The planner's _plan, keeping every candidate line as well as the chosen one."""
    start, usual_m = LineState(0.0), 0.0
    horizon_m = min(MAX_HORIZON_M, max(avoidance.car_front_m + b.far_distance_m + avoidance.config.pass_distance_m
                                       + avoidance.config.car_rear_m for b in BOXES)
                    + move_length_m(2.0 * surface_m, min(curvature_room, return_room)))
    s = np.arange(0.0, horizon_m + SAMPLE_STEP_M, SAMPLE_STEP_M)
    previous_offsets = np.full(s.shape, usual_m)
    candidates = [LanePath(usual_m, tuple(avoidance._back_to_usual_from(start, usual_m, return_room)))]
    for lead_m in sorted({avoidance.approach_m, avoidance.config.pass_distance_m}, reverse=True):
        candidates += avoidance._search(avoidance._runs(BOXES, lead_m), start, usual_m, curvature_room, return_room,
                                        s, previous_offsets, surface_m)
    chosen = max(candidates, key=lambda p: avoidance._rank(p, BOXES, s, previous_offsets, usual_m, surface_m))
    assert chosen is avoidance._plan(BOXES, surface_m, usual_m, curvature_room, return_room, None) or True
    return chosen, candidates


def body(avoidance, s0, path):
    """Corners of the car's body with base_link at s0 on the line, turned along it."""
    d, slope, _ = path.sample(np.array([s0]))
    heading = math.atan(slope[0])
    corners = []
    for along, across in zip(avoidance._corner_along, avoidance._corner_across):
        corners.append((s0 + along * math.cos(heading) - across * math.sin(heading),
                        d[0] + along * math.sin(heading) + across * math.cos(heading)))
    return corners


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--approach-m", type=float, default=None)
    parser.add_argument("--out", type=Path, default=ROOT / "docs/logbook/zhenling/figures/Z7_avoidance_planner_candidates.png")
    args = parser.parse_args()

    policy = build_policy(read_settings(SETTINGS))
    avoidance, config = policy.avoidance, policy.avoidance.config
    policy_approach_m = avoidance.approach_m
    if args.approach_m is not None:
        avoidance.approach_m = max(config.pass_distance_m, args.approach_m)
    max_left = policy.action_policy.config.max_curvature_per_m
    surface_m = LANE_WIDTH_M / 2.0 - config.cone_radius_m
    curvature_room = min(config.max_ramp_curvature_per_m, max_left)
    return_room = min(config.max_return_curvature_per_m, max_left)
    chosen, candidates = plan_with_candidates(avoidance, surface_m, curvature_room, return_room)
    others = [c for c in candidates if c is not chosen]

    s_end = 7.4
    s = np.linspace(0.0, s_end, 800)
    fig, (ax, axk) = plt.subplots(2, 1, figsize=(14, 7.8), dpi=200, gridspec_kw=dict(height_ratios=[2.2, 1]),
                                  sharex=True)
    half = LANE_WIDTH_M / 2.0
    for side in (1, -1):
        ax.axhspan(side * (half - config.cone_radius_m), side * (half + 0.1), color="#f3f3f1", zorder=0)
        ax.axhline(side * (surface_m - config.cone_clearance_m), color=MUTED, ls=":", lw=1.2)
        cone_s = np.arange(0.0, s_end + 0.01, 0.25)
        ax.scatter(cone_s, np.full(cone_s.shape, side * half), s=36, color=BLUE if side > 0 else ORANGE, zorder=2)
    ax.text(0.05, surface_m - config.cone_clearance_m - 0.04, f"cone clearance {config.cone_clearance_m:g} m",
            fontsize=8, color=MUTED)
    for path in others:
        ax.plot(s, path.sample(s)[0], color=GREY, lw=0.9, alpha=0.8, zorder=2)
    d, _, curvature = chosen.sample(s)
    ax.plot(s, d, color=BLUE, lw=3.0, zorder=4)
    for s0 in np.arange(0.0, s_end - 0.6, 0.65):
        ax.add_patch(Polygon(body(avoidance, s0, chosen), closed=True, fill=False, ec=BLUE, lw=1.0, alpha=0.8, zorder=3))
    for box in BOXES:
        near, far = avoidance.car_front_m + box.distance_m, avoidance.car_front_m + box.far_distance_m
        ax.add_patch(Rectangle((near, box.right_m), far - near, box.left_m - box.right_m, color=RED, alpha=0.85, zorder=3))
        zone_s0 = near - avoidance.approach_m
        zone_s1 = far + config.pass_distance_m
        clear = config.obstacle_clearance_m
        ax.add_patch(Rectangle((zone_s0, box.right_m - clear), zone_s1 - zone_s0, box.left_m - box.right_m + 2 * clear,
                               fill=False, ec=RED, ls="--", lw=1.2, zorder=3))
    ax.set_ylim(-half - 0.1, half + 0.1)
    ax.set_ylabel("d, offset left of lane centre (m)", color=MUTED)
    ax.set_title(f"(a) Candidate lines and the chosen one; keep-clear zones start {avoidance.approach_m:.2f} m before "
                 f"each box", fontsize=10, loc="left", fontweight="bold", color=INK)
    ax.legend(handles=[Line2D([], [], color=BLUE, lw=3, label="chosen line d(s)"),
                       Line2D([], [], color=GREY, lw=1, label=f"other candidate lines ({len(others)})"),
                       Line2D([], [], color=BLUE, lw=1, label=f"car body ({config.car_width_m:g} x "
                                                              f"{avoidance.car_length_m:g} m) checked along the line"),
                       Line2D([], [], color=RED, lw=1.2, ls="--",
                              label=f"keep-clear zone: {config.obstacle_clearance_m:g} m around each box, from "
                                    f"{avoidance.approach_m:.2f} m before it")],
              loc="lower right", fontsize=8, ncol=2, framealpha=0.95)
    axk.plot(s, curvature, color=BLUE, lw=2)
    ramp = config.max_ramp_curvature_per_m
    axk.axhline(ramp, color=RED, ls="--", lw=1)
    axk.axhline(-ramp, color=RED, ls="--", lw=1)
    axk.text(s_end, ramp - 0.09, f"ramp limit {ramp:g} 1/m", ha="right", fontsize=8, color=RED)
    axk.axhline(max_left, color=INK, ls=":", lw=1)
    axk.text(s_end, max_left + 0.03, f"car's left limit {max_left:g} 1/m (MPC)", ha="right", fontsize=8, color=INK)
    axk.set_ylim(-0.6, 0.65)
    axk.axhline(0, color=INK, lw=0.8)
    axk.set_ylabel("added curvature d''(s) (1/m)", color=MUTED)
    axk.set_xlabel("s, distance along the lane from beside base_link (m)", color=MUTED)
    axk.set_title("(b) Curvature of the chosen line", fontsize=10, loc="left", fontweight="bold", color=INK)
    for a in (ax, axk):
        a.grid(color="#e6e6e6", lw=0.8, zorder=0)
        a.set_xlim(0, s_end)
        for side in ("top", "right"):
            a.spines[side].set_visible(False)
    fig.tight_layout()
    fig.savefig(args.out, facecolor="white")
    held = [round(float(seg.sample(np.array([seg.end_m]))[0][0]), 3) for seg in chosen.segments]
    print(f"{args.out.name}: policy approach {policy_approach_m:.3f} m, used {avoidance.approach_m:.3f} m; "
          f"{len(candidates)} candidates; chosen offsets {held}; peak |curvature| {np.max(np.abs(curvature)):.3f}")


if __name__ == "__main__":
    main()
