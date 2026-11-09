"""Makes Peiyu's presentation slide (docs/logbook/peiyu/slide/peiyu-wang-slide.pptx) from the figures in figures/.

    python3 docs/logbook/peiyu/make_slide.py     # needs python-pptx and matplotlib; run make_figures.py first

It also writes slide/peiyu-wang-slide.png, a preview drawn with matplotlib from the same cards and positions, so the
slide can be checked without PowerPoint. The .pptx is the one to present.
"""
import textwrap
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402
from pptx import Presentation  # noqa: E402
from pptx.dml.color import RGBColor  # noqa: E402
from pptx.enum.shapes import MSO_SHAPE  # noqa: E402
from pptx.util import Inches, Pt  # noqa: E402

HERE = Path(__file__).resolve().parent
FIGURES = HERE / "figures"
OUT = HERE / "slide" / "peiyu-wang-slide.pptx"
PREVIEW = HERE / "slide" / "peiyu-wang-slide.png"
DARK, BLUE, GREY = RGBColor(0x22, 0x22, 0x22), RGBColor(0x1F, 0x5F, 0xD1), RGBColor(0x66, 0x66, 0x66)
GREEN, AMBER = RGBColor(0x2A, 0x7F, 0x62), RGBColor(0xB2, 0x6B, 0x00)
TITLE = ("Peiyu Wang  ", "MPC sub-team: first MPC, the Stage 1 MPC-or-RL choice, tools to check runs")

CARDS = [
    ("1  MPC lane follower",
     ["Built the first MPC lane follower: path-frame model, 0.2 s delay compensation, bounded least squares",
      "Wrote a temporary speed/curvature -> drive and steering conversion for the first car run",
      "Measured the time budget on car 20: cones at 10 Hz, 0.2 s old, so 20 ms per step"],
     "slide_mpc.png", ("Car 20, 6 Oct: 0.5 ms per MPC solve", GREEN)),
    ("2  Choosing MPC or RL",
     ["Same harness for a pure pursuit baseline, a 324-option MPC grid and a PPO steering policy",
      "Found and fixed an RL reward exploit: it drove out of the lane to end the episode",
      "Unseen cases (M1): the MPC passed all three; RL failed smoothness (R4) on two"],
     "slide_design.png", ("Simulation: MPC chosen for Stage 1", AMBER)),
    ("3  Lane end and lost lane",
     ["Lane end: follow the last plan, stop at the roll-out point: 0.19-0.30 m past the last cones (was up to 0.91 m short)",
      "Lost lane: half speed for 0.5 s, then a latched stop",
      "Measured car models: one control setting that drives car 20 and car 22"],
     "slide_lane_end.png", ("Car 22, 12 Oct: stopped 0.8 s after lane loss", GREEN)),
    ("4  Checking car runs",
     ["Foxglove: live policy markers, and replay of car data logs into MCAP",
      "Run summary: step timing on the car (22 Oct log: 57 ms between steps)",
      "run_report: lane error, cone clearance and obstacle passes per drive",
      "Found the OpenCV 5 crash that stopped car 22's cone detector"],
     "slide_tools.png", ("Car 22: detector fixed, cones at 10 Hz", GREEN)),
]
NOTES = (
    "About 90 seconds, whats only.\n"
    "1. I built the first MPC lane follower: it predicts the car relative to the lane centre, allows for the 0.2 s "
    "camera delay and solves a small bounded least-squares problem. For the first car run I wrote a temporary "
    "conversion to drive and steering, and measured on car 20 that cones arrive at 10 Hz about 0.2 s old, which gave "
    "the 20 ms budget. The MPC solves in half a millisecond on the car computer.\n"
    "2. Then I compared three designs on the same simulated cases: a pure pursuit baseline, a grid of 324 MPC options "
    "and a reinforcement learning steering policy. RL first learned to drive out of the lane to end the episode; I "
    "fixed the reward. On the unseen cases the MPC passed everything and RL steered too roughly, so we chose the MPC.\n"
    "3. The car used to stop well before the lane end, because the last cones leave the camera view early. Now it "
    "follows its last plan and stops in the zone; if the lane is lost mid-way it slows for half a second, then stops. "
    "Car 22 stopped 0.8 s after losing the lane. I also made car models so one control setting works on both cars.\n"
    "4. Finally, tools to check runs: the policy is drawn live in Foxglove and car logs replay into it, every run logs "
    "its timing, and run_report measures each drive. On car 22 I traced a cone detector crash to OpenCV 5.")


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


WIDTH, GAP, TOP = 3.02, 0.12, 1.0
"""Card width, gap and top in inches; the preview uses the same."""


def pptx():
    deck = Presentation()
    deck.slide_width, deck.slide_height = Inches(13.333), Inches(7.5)
    slide = deck.slides.add_slide(deck.slide_layouts[6])
    text_box(slide, 0.4, 0.2, 12.5, 0.7, [[(TITLE[0], 26, True, DARK), (TITLE[1], 18, False, BLUE)]])
    for i, (title, bullets, figure, (status, colour)) in enumerate(CARDS):
        x = 0.4 + i * (WIDTH + GAP)
        card = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(x), Inches(TOP), Inches(WIDTH), Inches(6.0))
        card.adjustments[0] = 0.04
        card.fill.solid()
        card.fill.fore_color.rgb = RGBColor(0xF6, 0xF7, 0xF9)
        card.line.color.rgb = RGBColor(0xD0, 0xD4, 0xDA)
        text_box(slide, x + 0.1, TOP + 0.08, WIDTH - 0.2, 0.45, [[(title, 15, True, BLUE)]])
        slide.shapes.add_picture(str(FIGURES / figure), Inches(x + 0.08), Inches(TOP + 0.6), width=Inches(WIDTH - 0.16))
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
    text_box(slide, 0.4, 7.05, 12.5, 0.4, [[("Green = shown on a real car; amber = simulation so far. Details, "
                                             "evidence and the hows and whys: logbook, Peiyu's section.", 10, False,
                                             GREY)]])
    slide.notes_slide.notes_text_frame.text = NOTES
    OUT.parent.mkdir(parents=True, exist_ok=True)
    deck.save(OUT)
    print("wrote", OUT)


def _hex(colour: RGBColor) -> str:
    return f"#{colour}"


def preview():
    """The same slide drawn with matplotlib, in inches from the top left like the .pptx."""
    figure = plt.figure(figsize=(13.333, 7.5), dpi=96)
    axis = figure.add_axes((0, 0, 1, 1))
    axis.set_xlim(0, 13.333)
    axis.set_ylim(7.5, 0)
    axis.axis("off")
    axis.text(0.5, 0.62, TITLE[0], fontsize=24, weight="bold", color=_hex(DARK), va="baseline")
    axis.text(2.75, 0.62, TITLE[1], fontsize=15.5, color=_hex(BLUE), va="baseline")
    for i, (title, bullets, name, (status, colour)) in enumerate(CARDS):
        x = 0.4 + i * (WIDTH + GAP)
        axis.add_patch(FancyBboxPatch((x, TOP), WIDTH, 6.0, boxstyle="round,pad=0,rounding_size=0.12", fc="#f6f7f9",
                                      ec="#d0d4da", lw=1))
        axis.text(x + 0.18, TOP + 0.38, title, fontsize=13.5, weight="bold", color=_hex(BLUE))
        image = plt.imread(FIGURES / name)
        height = (WIDTH - 0.16) * image.shape[0] / image.shape[1]
        axis.imshow(image, extent=(x + 0.08, x + WIDTH - 0.08, TOP + 0.6 + height, TOP + 0.6), zorder=2)
        y = TOP + 2.55
        for bullet in bullets:
            lines = textwrap.wrap("• " + bullet, 40)
            axis.text(x + 0.18, y, "\n".join(lines), fontsize=10.5, color=_hex(DARK), va="top", linespacing=1.3)
            y += 0.2 * len(lines) + 0.12
        axis.add_patch(FancyBboxPatch((x + 0.1, TOP + 5.35), WIDTH - 0.2, 0.5,
                                      boxstyle="round,pad=0,rounding_size=0.08", fc=_hex(colour), ec="none"))
        axis.text(x + WIDTH / 2, TOP + 5.6, "\n".join(textwrap.wrap(status, 40)), fontsize=9, weight="bold",
                  color="white", ha="center", va="center")
    axis.text(0.5, 7.25, "Green = shown on a real car; amber = simulation so far. Details, evidence and the hows and "
              "whys: logbook, Peiyu's section.", fontsize=9.5, color=_hex(GREY))
    figure.savefig(PREVIEW, dpi=96)
    plt.close(figure)
    print("wrote", PREVIEW)


def main():
    pptx()
    preview()


if __name__ == "__main__":
    main()
