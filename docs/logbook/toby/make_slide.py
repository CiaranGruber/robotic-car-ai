"""Makes Toby's presentation slide (docs/logbook/toby/slide/toby-lee-slide.pptx) from the figures in figures/.

    python3 docs/logbook/toby/make_slide.py     # needs python-pptx; run make_figures.py first
"""
from pathlib import Path

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.util import Inches, Pt

HERE = Path(__file__).resolve().parent
FIGURES = HERE / "figures"
OUT = HERE / "slide" / "toby-lee-slide.pptx"
DARK, BLUE, GREY = RGBColor(0x22, 0x22, 0x22), RGBColor(0x1F, 0x5F, 0xD1), RGBColor(0x66, 0x66, 0x66)
GREEN, AMBER = RGBColor(0x2A, 0x7F, 0x62), RGBColor(0xB2, 0x6B, 0x00)

CARDS = [
    ("1  MPC lane following", "slide_mpc.png", "12 cases",
     "test harness we used to choose the MPC over RL and tune it", ("Car runs: MPC stays between the cones", GREEN)),
    ("2  Lane shifting", "slide_lane_change.png", "L · C · R",
     "keep left or right of centre, moving across smoothly", ("Car 20, 20 Oct: kept to the left", GREEN)),
    ("3  Markers -> actions", "slide_aruco.png", "ID -> action",
     "keep-side markers first, then turns; nearest within 2 m, seen twice", ("Car 13, 27 Oct: \"Marker 14: KEEP_RIGHT\"", GREEN)),
    ("4  Refining intersection turns", "slide_left_turn.png", "39 / 40",
     "left turns in simulation, up from 2/40 once traced to steering", ("Simulation; car test pending", AMBER)),
]
"""(title, figure, key number, one line under it, (status badge, colour)). The rest is said, not written: see
SCRIPT."""
SCRIPT = """About 2-3 minutes. [pause] = breathe or move to the next card.
[1] Hi, I'm Toby, and I'm in the MPC sub-team. [pause] The first thing I did was work with Vincent and Kevin to set up our first lane-following MPC. Vincent designed the controller itself, and my part in Stage 1 was the testing, so I built the test cases and a test harness to run them. [pause] I made twelve lane test cases. The car starts off-centre or at an angle, the lanes are different widths, and some cases add problems like missing cones, noise, extra delay or a wet floor. [pause] The harness can run any design, whether that's the MPC, RL or pure pursuit, and it scores each run against our requirements. We used it to choose the MPC over RL, and then to tune it.
[2] The second main task I worked on was lane shifting. [pause] I wrote the part of the policy that lets the car keep to the left or the right side of the lane, instead of always driving down the middle, and it moves across gradually so the shift is smooth. [pause] On car 20, we changed just that one setting to "left", and the car stayed on the left. [pause] In the lab's narrower lanes the shift was hard to see, so I retuned how close it goes to the edge, and after that it passed all 40 simulation runs.
[3] Next, I built the part that turns ArUco markers into actions for the policy. [pause] The first thing I tested was combining it with lane shifting, so a marker could tell the car to keep left or right. On car 13, the policy acted on a keep-right marker. [pause] I then expanded the marker actions to link up with the intersection turning that Kevin and I worked on, so a marker can also mean turn left, turn right or go straight. [pause] To avoid false triggers, the car only reacts to the nearest marker within two metres, and only after it has seen it twice in a row.
[4] My last task was refining the intersection turns with Kevin. [pause] When we tested the first version on the robot, it wasn't very consistent. Left turns kept failing, while right turns worked. [pause] So I built a simulation suite that runs different turning scenarios with different parameters, to find out when the car can make a turn and when it can't. [pause] It showed the same pattern as the real car, and it traced the problem to the steering. Car 20 can't turn left any tighter than about a two-metre radius, and that's too wide for a one-metre crossing. [pause] With more steering range, left turns went from 2 out of 40 to 39 out of 40 in simulation, but a full turn on the real car is still to be shown."""


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


def main():
    deck = Presentation()
    deck.slide_width, deck.slide_height = Inches(13.333), Inches(7.5)
    slide = deck.slides.add_slide(deck.slide_layouts[6])
    text_box(slide, 0.4, 0.2, 12.5, 0.7, [[("Toby Lee  ", 26, True, DARK),
                                           ("MPC sub-team: lane-following tests, lane shifting, marker actions and turns",
                                            18, False, BLUE)]])
    width, height, gap = 6.2, 2.9, 0.13
    for i, (title, figure, number, line, (status, colour)) in enumerate(CARDS):
        x, y = 0.4 + (i % 2) * (width + gap), 1.0 + (i // 2) * (height + gap)
        card = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(x), Inches(y), Inches(width), Inches(height))
        card.adjustments[0] = 0.04
        card.fill.solid()
        card.fill.fore_color.rgb = RGBColor(0xF6, 0xF7, 0xF9)
        card.line.color.rgb = RGBColor(0xD0, 0xD4, 0xDA)
        text_box(slide, x + 0.12, y + 0.06, width - 0.24, 0.45, [[(title, 16, True, BLUE)]])
        slide.shapes.add_picture(str(FIGURES / figure), Inches(x + 0.1), Inches(y + 0.5), height=Inches(2.28))
        right = x + 4.1
        text_box(slide, right, y + 0.55, width - 4.2, 0.7, [[(number, 26, True, DARK)]])
        text_box(slide, right, y + 1.25, width - 4.2, 0.9, [[(line, 13, False, GREY)]])
        badge = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(right), Inches(y + 2.18),
                                       Inches(width - 4.22), Inches(0.55))
        badge.fill.solid()
        badge.fill.fore_color.rgb = colour
        badge.line.fill.background()
        frame = badge.text_frame
        frame.word_wrap = True
        run = frame.paragraphs[0].add_run()
        run.text = status
        run.font.size, run.font.bold, run.font.color.rgb = Pt(10.5), True, RGBColor(0xFF, 0xFF, 0xFF)
    text_box(slide, 0.4, 7.05, 12.5, 0.4, [[("Green = shown on a real car; amber = simulation so far. Details, "
                                             "evidence and the hows and whys: logbook, Toby's section.", 10, False,
                                             GREY)]])
    slide.notes_slide.notes_text_frame.text = SCRIPT.replace("\n[", "\n\n[")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    deck.save(OUT)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
