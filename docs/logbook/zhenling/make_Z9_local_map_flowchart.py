"""Draws Fig. Z9: the local map's steps at each policy step, as a flowchart, and how lane detection uses its cones.

Run from the repository root:
    .venv/bin/python -B docs/logbook/zhenling/make_Z9_local_map_flowchart.py [OUT]
The numbers come from policy/local_map/local_map.py and the local_map parameters in config/ai4r_policy.yaml, so the
figure follows the code. Needs matplotlib and PyYAML.
"""
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from policy.local_map import local_map as lm  # noqa: E402

OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "docs/logbook/zhenling/figures/Z9_local_map_flowchart.png"
with open(ROOT / "config/ai4r_policy.yaml") as file:
    SETTINGS = next(iter(yaml.safe_load(file).values()))["ros__parameters"]["local_map"]
MATCH_M = SETTINGS["cone_match_distance_m"]
SIGHTINGS = SETTINGS["cone_min_sightings"]
BEHIND_M = SETTINGS["cone_keep_behind_m"]
KEEP_S = SETTINGS["cone_keep_s"]
assert SETTINGS["use_remembered_cones"], "the figure shows the remembered cones going to lane detection"

W, H = 9.6, 13.0
FONT = "Arial"
INK, MUTED = "#222222", "#555555"
MINE, MINE_LINE = "#FBE5D6", "#C55A11"  # Zhenling's modules, as in Fig. Z1
TEAM, TEAM_LINE = "#C3DFF3", "#2E75B6"  # a teammate's module
IO, IO_LINE = "#CDE8C4", "#70AD47"  # data in and out
END, END_LINE = "#F2F2F2", "#8C8C8C"
MAIN_X, LEFT_X, RIGHT_X = 4.7, 1.15, 8.35


def text_block(ax, x, y, title, lines=(), size=10.5):
    """Bold title over regular lines, centred on (x, y)."""
    title_h, line_h = 0.23, 0.185
    top = y - (title_h + line_h * len(lines)) / 2.0
    ax.text(x, top + title_h / 2.0, title, ha="center", va="center", fontsize=size + 1.5, fontweight="bold",
            color=INK, family=FONT)
    for i, line in enumerate(lines):
        ax.text(x, top + title_h + line_h * (i + 0.5), line, ha="center", va="center", fontsize=size, color=INK,
                family=FONT)


def box(ax, x, y, w, h, title, lines=(), fill=MINE, edge=MINE_LINE):
    """A rounded box centred on (x, y); returns its edges (left, right, top, bottom)."""
    ax.add_patch(FancyBboxPatch((x - w / 2.0, y - h / 2.0), w, h, boxstyle="round,pad=0,rounding_size=0.12",
                                facecolor=fill, edgecolor=edge, linewidth=1.8, zorder=2))
    text_block(ax, x, y, title, lines)
    return x - w / 2.0, x + w / 2.0, y - h / 2.0, y + h / 2.0


def diamond(ax, x, y, w, h, lines, edge=MINE_LINE):
    """A decision centred on (x, y); returns its points (left, right, top, bottom)."""
    ax.add_patch(Polygon([(x - w / 2.0, y), (x, y - h / 2.0), (x + w / 2.0, y), (x, y + h / 2.0)], closed=True,
                         facecolor="white", edgecolor=edge, linewidth=1.8, zorder=2))
    for i, line in enumerate(lines):
        ax.text(x, y + (i - (len(lines) - 1) / 2.0) * 0.19, line, ha="center", va="center", fontsize=10.5,
                color=INK, family=FONT, zorder=3)
    return x - w / 2.0, x + w / 2.0, y - h / 2.0, y + h / 2.0


def arrow(ax, points, label=None, label_at=0, offset=(0.08, -0.1), ha="left"):
    """A polyline through points, with an arrow head at the end and an optional label beside one of its corners."""
    for (x1, y1), (x2, y2) in zip(points[:-2], points[1:-1]):
        ax.plot([x1, x2], [y1, y2], color=INK, linewidth=1.5, solid_capstyle="butt", zorder=1)
    ax.add_patch(FancyArrowPatch(points[-2], points[-1], arrowstyle="-|>", mutation_scale=14, color=INK,
                                 linewidth=1.5, shrinkA=0, shrinkB=0, zorder=1))
    if label:
        x, y = points[label_at]
        ax.text(x + offset[0], y + offset[1], label, ha=ha, va="center", fontsize=10, color=MUTED, style="italic",
                family=FONT)


def region(ax, top, bottom, fill, edge, title):
    ax.add_patch(FancyBboxPatch((0.1, top), W - 0.2, bottom - top, boxstyle="round,pad=0,rounding_size=0.15",
                                facecolor=fill, edgecolor=edge, linewidth=1.0, linestyle=(0, (4, 3)), zorder=0))
    ax.text(0.3, top + 0.22, title, ha="left", va="center", fontsize=11, fontweight="bold", color=edge, family=FONT)


def main():
    fig = plt.figure(figsize=(W, H))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(H, 0)
    ax.axis("off")

    # Input
    _, _, _, in_bottom = box(ax, MAIN_X, 0.5, 6.6, 0.62, "Each policy step, before lane detection",
                             ["cone batch (in base_link) · wheel speed · IMU · last step's command"], IO, IO_LINE)

    # The local map
    region(ax, 1.0, 9.8, "#FEF6F0", MINE_LINE, "Local map  (policy/local_map/local_map.py)")
    _, _, p1_top, p1_bottom = box(ax, MAIN_X, 1.95, 4.6, 0.95, "1  Dead reckoning", [
        "move the pose by wheel speed × dt, along an arc",
        "turn: IMU, else yaw rate, else commanded curvature",
        "first step of a run: start a new, empty map"])
    arrow(ax, [(MAIN_X, in_bottom), (MAIN_X, p1_top)])

    d1 = diamond(ax, MAIN_X, 3.25, 2.6, 0.9, ["Cone batch", "this step?"])
    arrow(ax, [(MAIN_X, p1_bottom), (MAIN_X, d1[2])])
    s1_left, _, _, _ = box(ax, RIGHT_X, 3.25, 2.2, 0.62, "Pose only", ["recall nothing"], END, END_LINE)
    arrow(ax, [(d1[1], 3.25), (s1_left, 3.25)], "no", offset=(0.12, -0.14))

    _, _, p2_top, p2_bottom = box(ax, MAIN_X, 4.45, 4.6, 0.75, "2  Match", [
        "each cone to the nearest remembered cone",
        f"of its colour within {MATCH_M:g} m, nearest pairs first"])
    arrow(ax, [(MAIN_X, d1[3]), (MAIN_X, p2_top)], "yes", offset=(0.1, 0.12))

    d2 = diamond(ax, MAIN_X, 5.7, 3.1, 1.0, ["Under 1/3 of the confirmed", "cones in view matched?"])
    arrow(ax, [(MAIN_X, p2_bottom), (MAIN_X, d2[2])])
    _, s2_right, _, s2_bottom = box(ax, LEFT_X, 5.7, 2.0, 0.8, "Map lost", ["forget every", "remembered cone"])
    arrow(ax, [(d2[0], 5.7), (s2_right, 5.7)], "yes", offset=(-0.12, -0.14), ha="right")

    d3 = diamond(ax, MAIN_X, 6.95, 2.6, 0.9, [f"≥ {lm.MIN_ALIGN_CONES} confirmed", "matches?"])
    arrow(ax, [(MAIN_X, d2[3]), (MAIN_X, d3[2])], "no", offset=(0.1, 0.12))
    s3_left, _, _, s3_bottom = box(ax, RIGHT_X, 6.95, 2.2, 1.1, "3  Correct pose", [
        "least-squares rigid fit,",
        f"apply {lm.ALIGN_GAIN:.0%}, ≤ {lm.MAX_SHIFT_CORRECTION_M:g} m",
        f"and ≤ {math.degrees(lm.MAX_TURN_CORRECTION_RAD):g}°, match again"])
    arrow(ax, [(d3[1], 6.95), (s3_left, 6.95)], "yes", offset=(0.12, -0.14))

    p4_y = 8.2
    p4_left, p4_right, p4_top, p4_bottom = box(ax, MAIN_X, p4_y, 4.6, 0.95, "4  Update and forget", [
        f"matched: running mean of up to {lm.MAX_AVERAGED_SIGHTINGS} sightings · new: add",
        f"forget: > {BEHIND_M:g} m behind the car, unseen {KEEP_S:g} s,",
        f"or still unconfirmed after {lm.UNCONFIRMED_KEEP_S:g} s"])
    arrow(ax, [(MAIN_X, d3[3]), (MAIN_X, p4_top)], "no", offset=(0.1, 0.12))
    arrow(ax, [(LEFT_X, s2_bottom), (LEFT_X, p4_y), (p4_left, p4_y)])
    arrow(ax, [(RIGHT_X, s3_bottom), (RIGHT_X, p4_y), (p4_right, p4_y)])

    _, _, p5_top, p5_bottom = box(ax, MAIN_X, 9.25, 4.6, 0.75, "5  Recall", [
        "confirmed cones with no batch cone of their colour",
        f"within {MATCH_M:g} m, back in the batch's frame"])
    arrow(ax, [(MAIN_X, p4_bottom), (MAIN_X, p5_top)])

    # Lane detection, a teammate's module, using the recalled cones
    region(ax, 10.05, 12.55, "#EEF6FC", TEAM_LINE, "Lane detection  (policy/lane_detection/lane_detection.py)")
    row_y = 11.05
    l1_left, l1_right, l1_top, _ = box(ax, 1.5, row_y, 2.5, 0.62, "Fit the lane", ["batch cones only"], TEAM, TEAM_LINE)
    arrow(ax, [(MAIN_X, p5_bottom), (MAIN_X, 9.93), (1.5, 9.93), (1.5, l1_top)])
    ld = diamond(ax, 4.25, row_y, 2.0, 0.85, ["Lane", "found?"], TEAM_LINE)
    arrow(ax, [(l1_right, row_y), (ld[0], row_y)])
    l2_left, _, l2_top, l2_bottom = box(ax, 7.55, row_y, 3.4, 0.8, "Fit again", [
        "batch + recalled cones; if that fails,", "keep the batch-only lane"], TEAM, TEAM_LINE)
    arrow(ax, [(ld[1], row_y), (l2_left, row_y)], "yes", offset=(0.1, -0.14))
    # The recalled cones are data for the second fit only
    ax.plot([MAIN_X + 2.3, 7.55, 7.55], [9.25, 9.25, l2_top - 0.12], color=MINE_LINE, linewidth=1.5,
            linestyle=(0, (4, 3)), zorder=1)
    ax.add_patch(FancyArrowPatch((7.55, l2_top - 0.12), (7.55, l2_top), arrowstyle="-|>", mutation_scale=14,
                                 color=MINE_LINE, linewidth=1.5, shrinkA=0, shrinkB=0, zorder=1))
    ax.text(7.67, 10.3, "recalled cones", ha="left", va="center", fontsize=10, color=MINE_LINE, style="italic",
            family=FONT)
    _, _, n1_top, _ = box(ax, 4.25, 12.05, 2.6, 0.62, "No lane", ["memory alone never makes one"], END, END_LINE)
    arrow(ax, [(4.25, ld[3]), (4.25, n1_top)], "no", offset=(0.1, 0.13))
    _, _, out_top, _ = box(ax, 7.55, 12.05, 3.4, 0.62, "Lane", ["to the rest of the policy"], IO, IO_LINE)
    arrow(ax, [(7.55, l2_bottom), (7.55, out_top)])

    ax.text(0.15, 12.82, f"Confirmed: seen in at least {SIGHTINGS} batches. Orange: my module; blue: a teammate's; "
            "green: data in and out.", ha="left", va="center", fontsize=9.5, color=MUTED, family=FONT)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=200, facecolor="white")
    print(OUT)


if __name__ == "__main__":
    main()
