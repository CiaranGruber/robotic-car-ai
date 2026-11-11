"""Draws Fig. Z2: the cone filter on one frame of a real drive, with the settings that drive used.

Run from the repository root:
    python docs/logbook/zhenling/make_Z2_cone_filter_real_drive.py [RUN] [STEP]
The default is the real drive 2026-10-22_15-42-42, step 14. The cone filter settings are read from the run's
own _config.json (check_straight_rows was false on the car), and the cones are the batch as received, before the
filter. Needs the car logs in scenarios/data/original_data/ (local, not in git) and matplotlib.
"""
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Arc, Rectangle  # noqa: E402
from matplotlib.ticker import MultipleLocator  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from policy.cone_filter.cone_filter import ConeFilterConfig, filter_cones  # noqa: E402
from policy.input_output import ConeBatch  # noqa: E402

RUN = sys.argv[1] if len(sys.argv) > 1 else "2026-10-22_15-42-42"
STEP = int(sys.argv[2]) if len(sys.argv) > 2 else 14
OUT = Path(sys.argv[3]) if len(sys.argv) > 3 else Path("docs/logbook/zhenling/figures/Z2_cone_filter_real_drive.png")
DATA = Path("scenarios/data/original_data")
CAR_FRONT_M, CAR_REAR_M, CAR_HALF_WIDTH_M = 0.30, 0.30, 0.125  # notebook car, not measured
COLOUR = {"BLUE": "#2a78d6", "YELLOW": "#e8a100"}
INK, MUTED, GRID = "#222222", "#666666", "#e6e6e6"


def reason(cone, config):
    """Why the filter removed this cone: the first failed check, in the order the logbook lists them."""
    p = cone.pos
    if config.max_distance_m and math.hypot(p.x, p.y) > config.max_distance_m:
        return "far"
    if not config.min_height_m <= p.z <= config.max_height_m:
        return "height"
    if p.x <= 0.0 or abs(p.y) > config.max_lateral_m or cone.confidence < config.min_confidence:
        return "space"
    return "duplicate"


def main():
    run_config = json.loads((DATA / f"{RUN}_config.json").read_text())
    config = ConeFilterConfig(**{k.split(".", 1)[1]: v for k, v in run_config.items() if k.startswith("cone_filter.")})
    observation = json.loads((DATA / f"{RUN}_observations.json").read_text())[STEP]
    batch = ConeBatch.deserialise(observation["cones"])
    kept_ids = {id(cone) for cone in filter_cones(batch, config)}
    rows = [(cone, None if id(cone) in kept_ids else reason(cone, config)) for cone in batch]
    counts = {}
    for _, why in rows:
        counts[why] = counts.get(why, 0) + 1

    fig, ax = plt.subplots(figsize=(6.4, 5.2), dpi=200)
    style = {None: dict(marker="o", s=70, filled=True), "far": dict(marker="s", s=70, filled=False),
             "height": dict(marker="^", s=90, filled=False), "duplicate": dict(marker="P", s=110, filled=False),
             "space": dict(marker="X", s=90, filled=False)}
    for cone, why in rows:
        st = style[why]
        colour = COLOUR[cone.colour.name]
        ax.scatter(cone.pos.y, cone.pos.x, marker=st["marker"], s=st["s"], zorder=3, linewidths=2.0,
                   facecolors=colour if st["filled"] else "white", edgecolors=colour)
    radius = config.max_distance_m
    ax.add_patch(Arc((0, 0), 2 * radius, 2 * radius, theta1=0, theta2=180, color=INK, lw=1.0, ls="--", zorder=1))
    ax.add_patch(Rectangle((-CAR_HALF_WIDTH_M, -CAR_REAR_M), 2 * CAR_HALF_WIDTH_M, CAR_FRONT_M + CAR_REAR_M,
                           facecolor="#f3f3f1", edgecolor="#444444", lw=1.2, zorder=2))

    xs = [c.pos.x for c, _ in rows] + [radius]
    ys = [abs(c.pos.y) for c, _ in rows] + [1.0]
    lim_y = math.ceil(max(ys) * 2 + 0.5) / 2
    ax.set_xlim(lim_y, -lim_y)  # y is left of the car, so left is drawn on the left
    # Equal scales, so distances read true; the limits stay tight around the car and the cones to keep it short
    ax.set_ylim(-CAR_REAR_M - 0.1, max(xs) + 0.25)
    ax.set_aspect("equal")
    # Labels every metre, grid lines every half metre
    for axis in (ax.xaxis, ax.yaxis):
        axis.set_major_locator(MultipleLocator(1.0))
        axis.set_minor_locator(MultipleLocator(0.5))
    ax.set_xlabel("y, left of car (m)", color=MUTED)
    ax.set_ylabel("x, ahead of car (m)", color=MUTED)
    ax.grid(which="both", color=GRID, lw=0.8, zorder=0)
    ax.tick_params(which="minor", length=0)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color("#999999")
    ax.tick_params(colors=MUTED)

    kept = counts.get(None, 0)
    ax.set_title(f"Real drive {RUN}, step {STEP}: {len(rows)} -> {kept} cones kept\n"
                 f"with the car's settings (check_straight_rows false)", fontsize=10, color=INK, loc="left",
                 fontweight="bold")
    labels = {None: "kept", "far": f"beyond {radius:g} m", "height": "too high (z = " + ", ".join(f"{c.pos.z:.2f}" for c, w in rows if w == "height")
                        + f" > {config.max_height_m:g} m)",
              "duplicate": f"duplicate (< {config.merge_distance_m:g} m)", "space": "outside space / low confidence"}
    handles = []
    for key in (None, "far", "height", "duplicate", "space"):
        if counts.get(key):
            st = style[key]
            handles.append(Line2D([], [], ls="", marker=st["marker"], markersize=8, markeredgewidth=1.8,
                                  markerfacecolor="#555555" if st["filled"] else "white", markeredgecolor="#555555",
                                  label=f"{labels[key]}: {counts[key]}"))
    handles.append(Line2D([], [], ls="--", color=INK, lw=1.0, label="range limit"))
    # Right of the axes, so it never hides a cone
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=8, frameon=False,
              title="filled = kept, hollow = removed", title_fontsize=8, alignment="left")
    fig.tight_layout()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, facecolor="white", bbox_inches="tight", pad_inches=0.15)
    print(OUT, {("kept" if k is None else k): v for k, v in counts.items()})


if __name__ == "__main__":
    main()
