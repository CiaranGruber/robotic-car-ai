"""Makes Peiyu's presentation slide (docs/logbook/peiyu/slide/peiyu-wang-slide.pptx).

    python3 docs/logbook/peiyu/make_slide.py     # needs python-pptx and matplotlib; run make_figures.py first

Assertion-evidence design (Michael Alley, Penn State): a one-sentence headline, then pictures as the evidence and
very few words, so the audience reads no more than about 15 a minute. One picture per piece of work:
1. what the MPC does each step: one plan solved with the Stage 1 model and weights (mpc.py at 47f8838), from a car
   0.25 m left of centre and pointing 8 degrees left, moved forward by the 0.2 s detection age first;
2. MPC against RL: worst steering roughness on the unseen cases against the smoothness limit (R4), values from
   docs/stage1-design.md M1, drawn without numbers;
3. where the car stops at the lane end, before and with the A7 rule (typical simulated stops, figures/03_lane_end.png);
4. Foxglove showing the camera-pan simulation (cropped from figures/foxglove_pan_sim.png: Foxglove with
   logs/foxglove/pan_sim/pan_right_hairpin_r1_25_ideal.mcap and its pan_sim_layout.json, at 8.4 s).
The one-minute script is NOTES, in the speaker notes. The details are in the logbook, Part 4.

It also writes slide/peiyu-wang-slide.png, a preview drawn with matplotlib from the same pictures and positions, so
the slide can be checked without PowerPoint. The .pptx is the one to present.
"""
from pathlib import Path

import math

import matplotlib
import numpy as np
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import FancyBboxPatch, Rectangle  # noqa: E402
from pptx import Presentation  # noqa: E402
from pptx.dml.color import RGBColor  # noqa: E402
from pptx.enum.text import PP_ALIGN  # noqa: E402
from pptx.util import Inches, Pt  # noqa: E402

HERE = Path(__file__).resolve().parent
FIGURES = HERE / "figures"
TILES = HERE / "slide" / "tiles"
OUT = HERE / "slide" / "peiyu-wang-slide.pptx"
PREVIEW = HERE / "slide" / "peiyu-wang-slide.png"
DARK, GREY = RGBColor(0x22, 0x22, 0x22), RGBColor(0x66, 0x66, 0x66)
GREEN, AMBER = RGBColor(0x2A, 0x7F, 0x62), RGBColor(0xB2, 0x6B, 0x00)
BAD, GOOD, NEUTRAL = "#c8553d", "#2a7f62", "#8a8a8a"
HEADLINE = "Peiyu: designed the car's MPC lane follower, tested it against RL, and built a virtual debug tool"
NAME = "Peiyu Wang, MPC sub-team"
NOTES = """\
I'm Peiyu, on the MPC sub-team. My part turns the lane into a speed and a curvature for Chris's controller, once \
per batch of cones, inside the time budget, on camera data that's already a fifth of a second old.

I built the team's first MPC. It predicts the car's distance from the lane centre and its heading, and picks \
curvatures that keep it close without jerky steering. It also steps the car forward past the camera delay.

To choose between MPC and RL, I ran a baseline, the MPC and an RL policy on the same simulated lanes. RL steered too \
roughly on the unseen ones, so we used the MPC.

The car used to stop short of the lane end. Now it follows its last plan and stops in the zone, and if it loses the \
lane it stops safely. Car 22 did that stop on the floor.

Last, Foxglove, which I set up so we can step through any run. Here it's my camera-pan simulation. The green fan is \
the camera's view, turned into the bend, so the policy still finds the lane, in pink."""
"""The one-minute script."""
TILE_W, TILE_H, TOP, LEFT, GAP = 6.15, 2.78, 1.3, 0.4, 0.2
"""Each tile's size, the grid's top left corner and the gap between tiles, in inches."""
CAPTION_H = 0.42
TAG_W = 1.9
"""Width of each tile's tag, right-aligned beside its title, in inches."""


def crop(name: str, rows: tuple[float, float], columns: tuple[float, float], out: str) -> Path:
    """Crops a figure by fractions of its height and width."""
    image = plt.imread(FIGURES / name)
    height, width = image.shape[:2]
    path = TILES / out
    plt.imsave(path, image[int(rows[0] * height):int(rows[1] * height), int(columns[0] * width):int(columns[1] * width)])
    return path


def mpc_plan() -> Path:
    """One MPC plan, solved as in policy/action_policy/mpc.py at 47f8838 (the Stage 1 design: N = 20 steps of
    0.1 s at 1 m/s, weights 3/1/0.1/3, straight lane, previous curvature 0). The bounds (|curvature| <= 3 1/m) are
    not reached, so plain least squares gives the same plan as bvls."""
    steps, speed, step_s, age = 20, 1.0, 0.1, 0.2
    weights = {"lateral": 3.0, "heading": 1.0, "curvature": 0.1, "change": 3.0}
    seen_lateral, heading = 0.25, math.radians(8.0)
    d = speed * step_s
    step = np.array([[1.0, d], [0.0, 1.0]])
    effect = np.array([d * d / 2.0, d])
    free = np.vstack([np.linalg.matrix_power(step, k + 1) for k in range(steps)])
    forced = np.zeros((2 * steps, steps))
    for k in range(steps):
        for j in range(k + 1):
            forced[2 * k:2 * k + 2, j] = np.linalg.matrix_power(step, k - j) @ effect
    state_weights = np.sqrt(np.tile([weights["lateral"], weights["heading"]], steps))
    now = np.array([seen_lateral + speed * age * heading, heading])
    cost = np.vstack([state_weights[:, None] * forced, math.sqrt(weights["curvature"]) * np.eye(steps),
                      math.sqrt(weights["change"]) * (np.eye(steps) - np.eye(steps, k=-1))])
    target = np.concatenate([-state_weights * (free @ now), np.zeros(2 * steps)])
    curvatures = np.linalg.lstsq(cost, target, rcond=None)[0]
    assert np.max(np.abs(curvatures)) < 3.0
    lateral = (free @ now + forced @ curvatures)[0::2]
    figure, axis = plt.subplots(figsize=(5.2, 3.0))
    for x in np.arange(-0.5, 2.75, 0.5):
        axis.plot(x, 0.5, "o", color="#e3b800", ms=8)
        axis.plot(x, -0.5, "o", color="#2f6fdf", ms=8)
    axis.plot([-0.6, 2.6], [0, 0], ls="--", color="#999999", lw=1.2)
    axis.text(2.6, -0.1, "lane centre", ha="right", fontsize=10.5, color="#777777")

    def car(x, y, colour, alpha):
        axis.add_patch(Rectangle((x - 0.2, y - 0.11), 0.4, 0.22, angle=math.degrees(heading), rotation_point="center",
                                 color=colour, alpha=alpha, lw=0))

    car(0.0, seen_lateral, "#888888", 0.35)
    car(speed * age, now[0], "#1f5fd1", 0.95)
    xs = speed * age + d * np.arange(1, steps + 1)
    axis.plot(xs, lateral, "o-", color=GOOD, ms=4.5, lw=1.5)
    arrow = dict(arrowstyle="->", lw=1.3)
    axis.annotate("camera saw the car here, 0.2 s ago", xy=(-0.05, 0.33), xytext=(-0.6, 0.78), fontsize=10.5,
                  color="#666666", arrowprops=arrow | {"color": "#888888"})
    axis.annotate("where it is now", xy=(0.25, 0.2), xytext=(0.1, -0.33), fontsize=11, weight="bold",
                  color="#1f5fd1", arrowprops=arrow | {"color": "#1f5fd1"})
    axis.text(1.0, 0.36, "MPC plan, 2 s ahead", fontsize=11.5, weight="bold", color=GOOD)
    axis.set_xlim(-0.65, 2.65)
    axis.set_ylim(-0.6, 0.9)
    axis.set_aspect("equal")
    axis.axis("off")
    figure.tight_layout()
    path = TILES / "mpc_plan.png"
    figure.savefig(path, dpi=200)
    plt.close(figure)
    return path


def mpc_or_rl() -> Path:
    """Worst steering roughness on the unseen cases U1-U3 (docs/stage1-design.md, M1) against the R4 limit."""
    figure, axis = plt.subplots(figsize=(5.2, 3.0))
    axis.bar(["Pure pursuit", "MPC", "RL"], [0.80, 0.51, 1.59], color=[NEUTRAL, GOOD, BAD], width=0.55)
    axis.axhline(1.0, color="#333333", ls="--", lw=1.5)
    axis.text(-0.4, 1.06, "smoothness limit", fontsize=12, color="#333333")
    axis.text(2, 1.64, "too rough", ha="center", fontsize=13, weight="bold", color=BAD)
    axis.text(1, 0.57, "smoothest", ha="center", fontsize=13, weight="bold", color=GOOD)
    axis.set_ylabel("steering roughness", fontsize=12)
    axis.set_yticks([])
    axis.tick_params(axis="x", labelsize=13)
    axis.spines[["top", "right"]].set_visible(False)
    axis.set_ylim(0, 1.8)
    figure.tight_layout()
    path = TILES / "mpc_or_rl.png"
    figure.savefig(path, dpi=200)
    plt.close(figure)
    return path


def lane_end() -> Path:
    """Top view of the lane end: the stop zone 0-0.5 m past the last cone pair, a typical stop before the A7 rule
    (0.5 m short) and with it (0.2 m past), from figures/03_lane_end.png."""
    figure, axis = plt.subplots(figsize=(5.2, 3.0))
    for x in [-2.0, -1.5, -1.0, -0.5, 0.0]:
        axis.plot(x, 0.5, "o", color="#2f6fdf", ms=9)
        axis.plot(x, -0.5, "o", color="#e3b800", ms=9)
    axis.add_patch(Rectangle((0.0, -0.5), 0.5, 1.0, color=GOOD, alpha=0.18, lw=0))
    axis.text(0.25, 0.62, "stop zone", ha="center", fontsize=12, color=GOOD)
    for centre, colour, label in [(-0.5, BAD, "before: short"), (0.2, GOOD, "now: in zone")]:
        axis.add_patch(Rectangle((centre - 0.3, -0.13 if label == "now" else -0.13), 0.6, 0.26, color=colour,
                                 alpha=0.85, lw=0))
        axis.text(centre - 0.15 if label.startswith("before") else centre + 0.15, -0.3, label, ha="center",
                  fontsize=12, color=colour, weight="bold")
    axis.annotate("", xy=(1.1, 0.0), xytext=(0.7, 0.0), arrowprops=dict(arrowstyle="->", color="#555555", lw=1.5))
    axis.text(0.9, 0.08, "driving", ha="center", fontsize=11, color="#555555")
    axis.set_xlim(-2.3, 1.3)
    axis.set_ylim(-0.8, 0.85)
    axis.set_aspect("equal")
    axis.axis("off")
    figure.tight_layout()
    path = TILES / "lane_end.png"
    figure.savefig(path, dpi=200)
    plt.close(figure)
    return path


def foxglove_pan() -> Path:
    """The Foxglove screenshot of the hairpin, with a key to its colours (tests/policy/prototype_camera_pan.py,
    write_world_mcap)."""
    shot = crop("foxglove_pan_sim.png", (110 / 1450, 830 / 1450), (420 / 2400, 1180 / 2400), "foxglove_crop.png")
    figure = plt.figure(figsize=(6.1, 2.3))
    picture = figure.add_axes((0.0, 0.0, 0.38, 1.0))
    picture.imshow(plt.imread(shot))
    picture.axis("off")
    key = figure.add_axes((0.39, 0.0, 0.61, 1.0))
    key.axis("off")
    handles = [
        Line2D([], [], color=(0.2, 0.9, 0.3), lw=2, label="Camera view, panned into the bend"),
        Line2D([], [], ls="", marker="o", ms=11, markerfacecolor=(0.2, 1.0, 0.3, 0.45),
               markeredgecolor=(0.1, 0.6, 0.2), label="Cones the camera sees"),
        Line2D([], [], color=(1.0, 0.2, 0.8), lw=3, label="Lane the policy found"),
        Line2D([], [], color=(1.0, 0.5, 0.0), lw=2.5, label="Where the camera aims"),
        Line2D([], [], color=(0.3, 0.8, 1.0), lw=2.5, label="The car's path"),
        Line2D([], [], color=(0.6, 0.6, 0.6), lw=2, label="True lane centre"),
    ]
    key.legend(handles=handles, loc="center left", frameon=False, fontsize=11, handlelength=2.0, labelspacing=0.8)
    path = TILES / "foxglove_pan.png"
    figure.savefig(path, dpi=250)
    plt.close(figure)
    return path


def tiles() -> list[tuple[Path, str, str, RGBColor]]:
    """:return: (picture, title, where it was shown, tag colour) for each tile, in reading order."""
    TILES.mkdir(parents=True, exist_ok=True)
    return [
        (mpc_plan(), "Plans from where the car is now", "Ran on car 20", GREEN),
        (mpc_or_rl(), "Smoother than RL", "Simulation", AMBER),
        (lane_end(), "Stops in the zone, not short", "Simulation + car 22", GREEN),
        (foxglove_pan(), "The camera keeps the bend in view", "Foxglove", AMBER),
    ]


def placement(index: int, path: Path) -> tuple[float, float, float, float, float, float]:
    """:return: The tile's left and top, and the picture's left, top, width and height, in inches."""
    left = LEFT + (index % 2) * (TILE_W + GAP)
    top = TOP + (index // 2) * (TILE_H + GAP)
    image = plt.imread(path)
    height = TILE_H - CAPTION_H - 0.05
    width = height * image.shape[1] / image.shape[0]
    if width > TILE_W:
        width, height = TILE_W, TILE_W * image.shape[0] / image.shape[1]
    return left, top, left + (TILE_W - width) / 2, top + CAPTION_H, width, height


def text_box(slide, x, y, w, h, runs, align=PP_ALIGN.LEFT):
    frame = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h)).text_frame
    frame.word_wrap = True
    frame.paragraphs[0].alignment = align
    for text, size, bold, colour in runs:
        run = frame.paragraphs[0].add_run()
        run.text = text
        run.font.size, run.font.bold, run.font.color.rgb = Pt(size), bold, colour


def pptx(pictures):
    deck = Presentation()
    deck.slide_width, deck.slide_height = Inches(13.333), Inches(7.5)
    slide = deck.slides.add_slide(deck.slide_layouts[6])
    text_box(slide, 0.4, 0.2, 12.5, 1.1, [(HEADLINE, 26, True, DARK)])
    for i, (path, title, tag, colour) in enumerate(pictures):
        left, top, x, y, width, height = placement(i, path)
        text_box(slide, left, top, TILE_W - TAG_W, CAPTION_H, [(f"{i + 1}  {title}", 16, True, DARK)])
        text_box(slide, left + TILE_W - TAG_W, top + 0.05, TAG_W, CAPTION_H, [(tag, 11, False, colour)], PP_ALIGN.RIGHT)
        slide.shapes.add_picture(str(path), Inches(x), Inches(y), width=Inches(width), height=Inches(height))
    text_box(slide, 0.4, 7.12, 6, 0.3, [(NAME, 11, False, GREY)])
    slide.notes_slide.notes_text_frame.text = NOTES
    deck.save(OUT)
    print("wrote", OUT)


def preview(pictures):
    """The same slide drawn with matplotlib, in inches from the top left like the .pptx."""
    figure = plt.figure(figsize=(13.333, 7.5), dpi=110)
    axis = figure.add_axes((0, 0, 1, 1))
    axis.set_xlim(0, 13.333)
    axis.set_ylim(7.5, 0)
    axis.axis("off")
    axis.text(0.5, 0.3, "Peiyu: designed the car's MPC lane follower, tested it against RL,\nand built a virtual debug tool", fontsize=21, weight="bold", color=f"#{DARK}", va="top", linespacing=1.25)
    for i, (path, title, tag, colour) in enumerate(pictures):
        left, top, x, y, width, height = placement(i, path)
        axis.text(left + 0.08, top + 0.08, f"{i + 1}  {title}", fontsize=14, weight="bold", color=f"#{DARK}",
                  va="top")
        axis.text(left + TILE_W - 0.05, top + 0.12, tag, fontsize=9.5, color=f"#{colour}", va="top", ha="right")
        axis.imshow(plt.imread(path), extent=(x, x + width, y + height, y))
        axis.add_patch(FancyBboxPatch((x, y), width, height, boxstyle="square,pad=0", fc="none", ec="#e2e5ea",
                                      lw=0.8))
    axis.text(0.5, 7.15, NAME, fontsize=10.5, color=f"#{GREY}", va="top")
    figure.savefig(PREVIEW, dpi=110)
    plt.close(figure)
    print("wrote", PREVIEW)


def main():
    pictures = tiles()
    pptx(pictures)
    preview(pictures)


if __name__ == "__main__":
    main()
