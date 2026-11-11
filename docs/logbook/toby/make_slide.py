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
     "keep left, centre or right, moving across smoothly", ("Car 20, 20 Oct: kept to the left", GREEN)),
    ("3  Markers -> actions", "slide_aruco.png", "ID -> action",
     "nearest marker within 2 m, seen twice", ("Car 13, 27 Oct: \"Marker 14: KEEP_RIGHT\"", GREEN)),
    ("4  Why left turns failed", "slide_left_turn.png", "R = 2.08 m",
     "too wide for a 1 m crossing; fix: left turns 2/40 -> 39/40", ("Simulation; car test pending", AMBER)),
]
"""(title, figure, key number, one line under it, (status badge, colour)). The rest is said, not written: see
SCRIPT."""
SCRIPT = """About 100 seconds. Point at each card as you reach it.
[1] I'm Toby, in the MPC sub-team. Peiyu designed our MPC; my part was making it testable. I wrote the Stage 1
test cases and the harness that runs them: twelve cases with the car starting off-centre or at an angle, lanes 0.8 to
1.2 metres wide, missed cones, detector noise, extra delay and a wet floor. The harness drives any design, MPC, RL or
pure pursuit, closed loop in dream-gym and scores it against requirements R1 to R7. That is what we used to choose the
MPC over RL and then tune it. Later I added the real car's delay, cone noise and steering limits, measured from
recorded drives, so settings tuned in simulation carried over to the car.
[2] Next, lane shifting. Instead of the lane centre, the MPC can follow a line beside it, so the car keeps left, centre
or right, and it moves across gradually, at most 0.3 metres sideways per metre driven. On car 20 on 20 October,
changing only that setting to left kept the car on the left. In the lab's narrow lanes the shift was hard to see, so I
worked out how much room there really is and retuned it: 40 out of 40 in simulation.
[3] Then I connected the ArUco markers to the policy. Only the nearest marker within two metres counts, and only once
it has been seen in two steps in a row. Its ID means keep a side, turn left, turn right or go straight. A turn is its
own manoeuvre: move to the outside of the lane, drive a planned arc, then hand back to lane following. On car 13 the
policy acted on a keep-right marker.
[4] Last, four car sessions of testing. Left turns kept failing on car 20 while right turns worked. I reproduced the
same pattern in simulation and traced it to the steering: car 20 can only turn left on a 2.08 metre radius, and that
doesn't fit a one-metre crossing. With more steering range, left turns go from 2 out of 40 to 39 out of 40 in
simulation. A full turn on a car is still to be shown."""


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
