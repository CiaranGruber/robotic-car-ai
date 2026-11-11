"""Makes Peiyu's presentation slide (docs/logbook/peiyu/slide/peiyu-wang-slide.pptx).

    python3 docs/logbook/peiyu/make_slide.py     # needs python-pptx and matplotlib; run make_figures.py first

The layout is the team's (docs/logbook/chris/make_slide.py, long/make_slide.py): the name and a blue subtitle, four
cards, each with a picture, three bullets and a status badge (green = shown on a real car, amber = simulation so far
or not merged). The cards follow the script, NOTES, one step of the work each:
1. the MPC base and who built on it (PR #1 and #8; Toby's and Yuhao's additions as the submission logbook's 3.3 and
   3.4 name them);
2. choosing MPC over RL: the steering each asks for on the unseen case U2 (47f8838, figures/m1_u2_steering.json);
3. the virtual debug tool: Foxglove replaying car 20's run car20_2026-10-19_12-49-21.mcap at 10.7 s
   (figures/foxglove_car20_replay.png; the replay chose the car's own actions on every step);
4. keeping it all working: speed on T1 with each car model and control setting (45eb015,
   figures/car_models_speed.json, the same runs as the logbook's figures/06_car_models.png).
It also writes the pictures only the submission logbook uses (car_timing.png, lane_end.png, foxglove_pan.png), and
slide/peiyu-wang-slide.png, a preview drawn with matplotlib from the same positions, so the slide can be checked
without PowerPoint. The .pptx is the one to present; the script is also in slide/script.md.
"""
import json
import textwrap
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.legend_handler import HandlerTuple  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402
from pptx import Presentation  # noqa: E402
from pptx.dml.color import RGBColor  # noqa: E402
from pptx.enum.shapes import MSO_SHAPE  # noqa: E402
from pptx.util import Inches, Pt  # noqa: E402

HERE = Path(__file__).resolve().parent
FIGURES = HERE / "figures"
TILES = HERE / "slide" / "tiles"
OUT = HERE / "slide" / "peiyu-wang-slide.pptx"
PREVIEW = HERE / "slide" / "peiyu-wang-slide.png"
DARK, BLUE, GREY = RGBColor(0x22, 0x22, 0x22), RGBColor(0x1F, 0x5F, 0xD1), RGBColor(0x66, 0x66, 0x66)
GREEN, AMBER = RGBColor(0x2A, 0x7F, 0x62), RGBColor(0xB2, 0x6B, 0x00)
BAD, GOOD, NEUTRAL = "#c8553d", "#2a7f62", "#8a8a8a"
NAME = "Peiyu Wang"
SUBTITLE = "MPC sub-team: the MPC base, MPC over RL, virtual debug, keeping it all working"
FOOTER = ("Green = shown on a real car; amber = simulation so far or not merged. Details, evidence and the hows and "
          "whys: logbook, Peiyu's section.")
CARD_FIGURE = (4.2, 2.6)
"""Size of each card's picture in inches when drawn; the card shows it 2.86 inches wide."""
NOTES = """\
I'm Peiyu, from the MPC sub-team.

When we started, there was no movement policy. I wrote the first MPC framework and got it running on car 20: it \
predicts where the car will be and picks steering that brings it back to the lane centre smoothly. That became \
the base. Toby then tuned it for car 20 and added lane side and turns; Yuhao added measured speed and stopping.

Then we had to decide: MPC, or a learned RL policy. I tested both in the same simulator, on lanes neither had \
seen. RL turned the wheel hard right, then hard left; the MPC made one smooth correction. RL broke our \
smoothness limit and the MPC passed every lane, so we chose the MPC.

Next was debugging. Before, when a run went wrong, all we had was the car's path: we watched where it drove and \
guessed why. So I built a virtual debug tool: every run is logged and replayed in Foxglove, step by step, with \
the cones the car saw, the lane it found and the path it planned. Here car 20 has drifted right, and you can see \
its plan steering back. After that, the team found problems by replaying runs instead of rerunning the car.

Last, keeping it all working. Every car was different: car 20's setting drove car 22 three times too fast, so I \
made car models and one setting for both. The cars kept failing to start, and each time meant restarting, \
reading logs and redeploying. And with eight people pushing at once, branches collided: I resolved the merge \
conflicts and reviewed pull requests, like Toby's lane side, where I fixed five problems before merging."""
"""The script, about 280 words: two minutes, one paragraph per card."""


def crop(name: str, rows: tuple[float, float], columns: tuple[float, float], out: str) -> Path:
    """Crops a figure by fractions of its height and width."""
    image = plt.imread(FIGURES / name)
    height, width = image.shape[:2]
    path = TILES / out
    plt.imsave(path, image[int(rows[0] * height):int(rows[1] * height), int(columns[0] * width):int(columns[1] * width)])
    return path


def save(figure, name: str, dpi: int = 250) -> Path:
    path = TILES / name
    figure.savefig(path, dpi=dpi)
    plt.close(figure)
    return path


def mpc_base() -> Path:
    """Who built what on the MPC: the framework and first car run (PR #1, #8), then the teammates' additions as the
    submission logbook's 3.3 and 3.4 name them."""
    figure, axis = plt.subplots(figsize=CARD_FIGURE)
    axis.set_xlim(0, 10)
    axis.set_ylim(0, 6)
    axis.axis("off")

    def box(x, y, w, h, text, mine):
        axis.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.1", fc="#eaf1fb" if mine else "#f3f3f3",
                                      ec="#1f5fd1" if mine else "#9a9a9a", lw=2.2 if mine else 1.2))
        axis.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=11,
                  weight="bold" if mine else "normal", color="#1f3f8f" if mine else "#333333")

    axis.text(0.1, 5.55, "Before: no movement policy", fontsize=10.5, color="#777777", va="center")
    axis.annotate("", xy=(1.6, 4.35), xytext=(1.6, 5.2), arrowprops=dict(arrowstyle="->", color="#555555", lw=1.5))
    box(0.2, 2.65, 2.9, 1.6, "MPC\nframework\n+ first car\nrun (me)", True)
    for y, text in ((3.75, "Toby: car 20 tuning,\nlane side, turns"), (0.45, "Yuhao: measured\nspeed, stopping")):
        box(5.2, y, 4.6, 1.55, text, False)
        axis.annotate("", xy=(5.1, y + 0.78), xytext=(3.2, 3.45), arrowprops=dict(arrowstyle="->", color="#555555",
                                                                              lw=1.5))
    axis.text(4.1, 3.45, "built on", ha="center", va="center", fontsize=10, color="#555555",
              bbox=dict(fc="white", ec="none", pad=1))
    figure.tight_layout(pad=0.3)
    return save(figure, "mpc_base.png")


def rl_steering() -> Path:
    """The steering each design asks for on the unseen case U2 (figures/m1_u2_steering.json)."""
    data = json.loads((FIGURES / "m1_u2_steering.json").read_text())
    figure, axis = plt.subplots(figsize=CARD_FIGURE)
    for name, colour in (("RL", BAD), ("MPC", GOOD)):
        times, curvatures = zip(*[(t, k) for t, k in data["commands"][name] if t <= 3.5])
        axis.step(times, curvatures, where="post", color=colour, lw=2.6)
    axis.axhline(0, color="#bbbbbb", lw=1)
    axis.text(0.62, -1.42, "RL: hard right,", fontsize=11, weight="bold", color=BAD, va="center")
    axis.text(1.4, 0.58, "then hard left", fontsize=11, weight="bold", color=BAD, va="center")
    axis.text(1.0, -0.7, "MPC: one smooth\ncorrection", fontsize=11, weight="bold", color=GOOD, va="center")
    axis.set_yticks([0.45, 0, -0.45], ["left", "straight", "right"], fontsize=10)
    axis.set_xticks([0, 1, 2, 3])
    axis.set_xlabel("time (s), same unseen lane", fontsize=10)
    axis.tick_params(axis="x", labelsize=9.5)
    axis.spines[["top", "right"]].set_visible(False)
    axis.set_xlim(0, 3.5)
    axis.set_ylim(-1.6, 0.8)
    figure.tight_layout(pad=0.3)
    return save(figure, "rl_steering.png")


def car20_replay() -> Path:
    """Foxglove replaying car 20's run car20_2026-10-19_12-49-21.mcap at 10.7 s (the replay chose the car's own actions
    on every step), drawn with policy/visualisation.py's layers, with a key to its colours."""
    shot = crop("foxglove_car20_replay.png", (220 / 1520, 1320 / 1520), (820 / 2400, 1640 / 2400),
                "foxglove_car20_crop.png")
    figure = plt.figure(figsize=CARD_FIGURE)
    picture = figure.add_axes((0.0, 0.0, 0.42, 1.0))
    picture.imshow(plt.imread(shot))
    picture.axis("off")
    key = figure.add_axes((0.44, 0.0, 0.56, 1.0))
    key.axis("off")
    cone = dict(ls="", marker="o", ms=9, markeredgecolor="#555555", markeredgewidth=0.5)
    handles = [
        (Line2D([], [], markerfacecolor=(0.18, 0.5, 0.93), **cone),
         Line2D([], [], markerfacecolor=(0.95, 0.79, 0.3), **cone)),
        Line2D([], [], color=(0.2, 0.85, 0.3), lw=3),
        Line2D([], [], color=(0.2, 0.85, 0.95), lw=3),
        Line2D([], [], ls="", marker="s", ms=12, markerfacecolor=(0.92, 0.92, 0.93), markeredgecolor="#777777"),
    ]
    labels = ["Cones it saw", "Lane it found", "Path it planned", "Car 20, drifted\nright"]
    key.legend(handles, labels, loc="center left", frameon=False, fontsize=10.5, handlelength=1.8, labelspacing=1.0,
               handler_map={tuple: HandlerTuple(ndivide=None, pad=0.5)})
    return save(figure, "car20_replay.png")


def car_models() -> Path:
    """Speed on tuning case T1 at 0.5 m/s with each car model and control setting (45eb015,
    figures/car_models_speed.json; the logbook's figures/06_car_models.png draws the same runs)."""
    runs = json.loads((FIGURES / "car_models_speed.json").read_text())["runs"]
    settings = [("car 20's calibration (ControlConfig defaults)", NEUTRAL), ("dev after #16: car 22's values", BAD),
                ("PR #18: one setting for both cars", GOOD)]
    figure, axes = plt.subplots(1, 2, figsize=CARD_FIGURE, sharey=True)
    for axis, car, title in zip(axes, ("car20", "car22a"), ("Car 20", "Car 22")):
        for setting, colour in settings:
            run = runs[f"{car}|{setting}"]
            axis.plot(run["t"], run["speed"], color=colour, lw=2.2)
        axis.axhline(0.5, color="#444444", ls=":", lw=1)
        axis.set_title(title, fontsize=11, weight="bold")
        axis.set_xlim(0, 13)
        axis.set_xticks([0, 5, 10])
        axis.set_xlabel("time (s)", fontsize=9.5)
        axis.tick_params(labelsize=9)
        axis.spines[["top", "right"]].set_visible(False)
    axes[0].set_ylim(0, 1.75)
    axes[0].set_yticks([0, 0.5, 1.5])
    axes[0].set_ylabel("speed (m/s)", fontsize=9.5)
    axes[0].text(6.2, 0.04, "car 22's\nsetting:\nstalls", fontsize=9.5, weight="bold", color=BAD)
    axes[1].text(6.0, 1.12, "car 20's\nsetting:\n3x too fast", fontsize=9.5, weight="bold", color="#666666")
    axes[1].text(7.6, 0.62, "one setting\nfor both", fontsize=9.5, weight="bold", color=GOOD)
    figure.tight_layout(pad=0.3)
    return save(figure, "car_models.png")


def car_timing() -> Path:
    """The left half of figures/07_car_timing.png: the MPC's solve time per horizon on car 20's computer."""
    return crop("07_car_timing.png", (42 / 607, 1.0), (0.0, 790 / 1656), "car_timing.png")


def lane_end() -> Path:
    """The last 2.7 m of tuning case T1 in the simulator's own top view (x 3.9 to 6.6 m along the lane, 159 pixels a
    metre), before the A7 rule and with it, side by side."""
    figure = plt.figure(figsize=(5.6, 2.7))
    for column, (name, label, colour) in enumerate([("sim_T1_before_a7.png", "Before: stops short", BAD),
                                                     ("sim_T1_with_a7.png", "Now: stops in the zone", GOOD)]):
        axis = figure.add_axes((0.01 + column * 0.5, 0.02, 0.47, 0.8))
        axis.imshow(plt.imread(FIGURES / name)[60:300, 861:1290])
        axis.axis("off")
        axis.text(388, -6, "stop zone", ha="center", va="bottom", fontsize=10.5, color=GOOD, clip_on=False)
        figure.text(0.01 + column * 0.5, 0.92, label, fontsize=12.5, weight="bold", color=colour, va="center")
    return save(figure, "lane_end.png")


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
    return save(figure, "foxglove_pan.png")


def cards() -> list:
    """:return: (title, bullets, picture, (status badge, colour)) for each card, in the order of the script."""
    TILES.mkdir(parents=True, exist_ok=True)
    return [
        ("1  The MPC base",
         ["Before: no movement policy. I wrote the first MPC framework and ran it on car 20",
          "It predicts where the car will be and steers it back to the lane centre smoothly",
          "The team built on it: Toby tuned car 20, added lane side and turns; Yuhao added measured speed, stopping"],
         mpc_base(), ("Ran on car 20 (PR #1, #8)", GREEN)),
        ("2  Choosing MPC over RL",
         ["We had to choose: MPC or a learned RL policy, tested in the same simulator",
          "On lanes neither had seen, RL steered hard right, then hard left; the MPC made one smooth correction",
          "RL broke the smoothness limit, the MPC passed every lane: we chose the MPC"],
         rl_steering(), ("Simulation, unseen lanes; MPC on dev (PR #16)", AMBER)),
        ("3  Virtual debug tool",
         ["Before: only the car's path. We watched where it drove and guessed why",
          "Now every run replays in Foxglove step by step: the cones it saw, the lane it found, the path it planned",
          "After: the team finds problems by replaying runs, not by rerunning the car"],
         car20_replay(), ("Car 20 runs replayed; merged PR #25", GREEN)),
        ("4  Keeping it all working",
         ["Every car differs: car 20's setting drove car 22 3x too fast, so car models and one setting for both",
          "Cars kept failing to start: restart, read the logs, redeploy, every time",
          "8 people pushing at once: resolved the merge conflicts; reviewed PRs (#27: 5 fixes before merging)"],
         car_models(), ("Cars 20 + 22; one setting in sim (PR #18, open)", AMBER)),
    ]


WIDTH, GAP, TOP, LEFT, CARD_H = 3.02, 0.12, 1.0, 0.4, 6.0
"""The cards' width, the gap between them, their top and left edge and their height, in inches (Chris's layout)."""


def text_box(slide, x, y, w, h, paragraphs):
    frame = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h)).text_frame
    frame.word_wrap = True
    for i, runs in enumerate(paragraphs):
        paragraph = frame.paragraphs[0] if i == 0 else frame.add_paragraph()
        paragraph.space_after = Pt(3)
        for text, size, bold, colour in runs:
            run = paragraph.add_run()
            run.text = text
            run.font.size, run.font.bold, run.font.color.rgb = Pt(size), bold, colour


def pptx(card_list):
    deck = Presentation()
    deck.slide_width, deck.slide_height = Inches(13.333), Inches(7.5)
    slide = deck.slides.add_slide(deck.slide_layouts[6])
    text_box(slide, 0.4, 0.2, 12.5, 0.7, [[(f"{NAME}  ", 26, True, DARK), (SUBTITLE, 18, False, BLUE)]])
    for i, (title, bullets, figure, (status, colour)) in enumerate(card_list):
        x = LEFT + i * (WIDTH + GAP)
        card = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(x), Inches(TOP), Inches(WIDTH),
                                      Inches(CARD_H))
        card.adjustments[0] = 0.04
        card.fill.solid()
        card.fill.fore_color.rgb = RGBColor(0xF6, 0xF7, 0xF9)
        card.line.color.rgb = RGBColor(0xD0, 0xD4, 0xDA)
        text_box(slide, x + 0.1, TOP + 0.08, WIDTH - 0.2, 0.45, [[(title, 15, True, BLUE)]])
        slide.shapes.add_picture(str(figure), Inches(x + 0.08), Inches(TOP + 0.6), width=Inches(WIDTH - 0.16))
        text_box(slide, x + 0.1, TOP + 2.45, WIDTH - 0.2, 2.9, [[("• " + b, 12, False, DARK)] for b in bullets])
        badge = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(x + 0.1), Inches(TOP + 5.35),
                                       Inches(WIDTH - 0.2), Inches(0.5))
        badge.fill.solid()
        badge.fill.fore_color.rgb = colour
        badge.line.fill.background()
        frame = badge.text_frame
        frame.word_wrap = True
        run = frame.paragraphs[0].add_run()
        run.text = status
        run.font.size, run.font.bold, run.font.color.rgb = Pt(10.5), True, RGBColor(0xFF, 0xFF, 0xFF)
    text_box(slide, 0.4, 7.05, 12.5, 0.4, [[(FOOTER, 10, False, GREY)]])
    slide.notes_slide.notes_text_frame.text = NOTES
    OUT.parent.mkdir(parents=True, exist_ok=True)
    deck.save(OUT)
    print("wrote", OUT)


def preview(card_list):
    """The same slide drawn with matplotlib, in inches from the top left like the .pptx, for checking only."""
    figure = plt.figure(figsize=(13.333, 7.5), dpi=110)
    axis = figure.add_axes((0, 0, 1, 1))
    axis.set_xlim(0, 13.333)
    axis.set_ylim(7.5, 0)
    axis.axis("off")
    axis.text(0.5, 0.55, f"{NAME}", fontsize=24, weight="bold", color=f"#{DARK}", va="center")
    axis.text(2.75, 0.57, SUBTITLE, fontsize=16, color=f"#{BLUE}", va="center")
    for i, (title, bullets, path, (status, colour)) in enumerate(card_list):
        x = LEFT + i * (WIDTH + GAP)
        axis.add_patch(FancyBboxPatch((x, TOP), WIDTH, CARD_H, boxstyle="round,pad=0,rounding_size=0.12",
                                      fc="#f6f7f9", ec="#d0d4da", lw=1, zorder=0))
        axis.text(x + 0.17, TOP + 0.3, title, fontsize=13.5, weight="bold", color=f"#{BLUE}", va="center")
        image = plt.imread(path)
        height = (WIDTH - 0.16) * image.shape[0] / image.shape[1]
        axis.imshow(image, extent=(x + 0.08, x + WIDTH - 0.08, TOP + 0.6 + height, TOP + 0.6), zorder=2)
        y = TOP + 2.5
        for bullet in bullets:
            text = textwrap.fill("• " + bullet, 36)
            axis.text(x + 0.15, y, text, fontsize=10.8, color=f"#{DARK}", va="top", linespacing=1.25)
            y += 0.24 * (text.count("\n") + 1) + 0.08
        axis.add_patch(FancyBboxPatch((x + 0.1, TOP + 5.35), WIDTH - 0.2, 0.5, boxstyle="round,pad=0,rounding_size=0.08",
                                      fc=f"#{colour}", ec="none"))
        axis.text(x + WIDTH / 2, TOP + 5.6, textwrap.fill(status, 34), fontsize=9.5, weight="bold", color="white",
                  ha="center", va="center")
    axis.text(0.4, 7.2, FOOTER, fontsize=9, color=f"#{GREY}", va="center")
    figure.savefig(PREVIEW, dpi=110)
    plt.close(figure)
    print("wrote", PREVIEW)


def main():
    card_list = cards()
    car_timing()
    lane_end()
    foxglove_pan()
    pptx(card_list)
    preview(card_list)


if __name__ == "__main__":
    main()
