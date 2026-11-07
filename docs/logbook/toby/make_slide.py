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
    ("1  MPC lane following",
     ["Helped build the first MPC lane follower with the MPC sub-team, and the Card 2 test cases we tuned it on",
      "Measured latency, noise and camera yaw from real drives to make the simulation match the car",
      "Tuned it to car 20's real steering: 0.48 / 0.92 1/m, smoother steering"],
     "slide_mpc.png", ("Team car runs: stays between the cones", GREEN)),
    ("2  Lane shifting left / right",
     ["MPC follows a line beside the centre: keep LEFT, CENTRE or RIGHT",
      "Shifts smoothly mid-run (0.3 m sideways per m driven)",
      "Retuned so the shift is visible in 0.8–1 m lab lanes (40/40 in sim)"],
     "slide_lane_change.png", ("Car 20, 20 Oct: kept to the left lane", GREEN)),
    ("3  ArUco markers -> actions",
     ["Marker ID -> keep side, turn left / right, straight",
      "Only the nearest marker counts; the side is kept until the next side marker",
      "Intersection turn: move aside, planned arc, hand back to lane following"],
     "slide_aruco.png", ("Car 13, 27 Oct: \"Marker 14: KEEP_RIGHT\"", GREEN)),
    ("4  Real-robot tuning",
     ["4 car sessions, 20 Oct–2 Nov (cars 20, 21, 13, 14): lane shifts, 90° turns, obstacles",
      "Made lane shifts visible (clearance 0.40 -> 0.28 m) and fixed the ArUco camera feed (--aruco)",
      "Phantom obstacles from marker stands and cones: 29% -> 4% of runs, real-box stops kept",
      "Left 90° turns failed on the car: traced to the steering limit; 2/40 -> 39/40 in sim"],
     "slide_left_turn.png", ("Turns + obstacles: sim only, car test pending", AMBER)),
]


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
                                           ("MPC sub-team: from lane following to marker-driven lane changes and turns",
                                            18, False, BLUE)]])
    width, gap, top = 3.02, 0.12, 1.0
    for i, (title, bullets, figure, (status, colour)) in enumerate(CARDS):
        x = 0.4 + i * (width + gap)
        card = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(x), Inches(top), Inches(width), Inches(6.0))
        card.adjustments[0] = 0.04
        card.fill.solid()
        card.fill.fore_color.rgb = RGBColor(0xF6, 0xF7, 0xF9)
        card.line.color.rgb = RGBColor(0xD0, 0xD4, 0xDA)
        text_box(slide, x + 0.1, top + 0.08, width - 0.2, 0.45, [[(title, 15, True, BLUE)]])
        slide.shapes.add_picture(str(FIGURES / figure), Inches(x + 0.08), Inches(top + 0.6), width=Inches(width - 0.16))
        text_box(slide, x + 0.1, top + 2.45, width - 0.2, 2.9, [[("• " + b, 12, False, DARK)] for b in bullets])
        badge = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(x + 0.1), Inches(top + 5.35),
                                       Inches(width - 0.2), Inches(0.5))
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
    slide.notes_slide.notes_text_frame.text = (
        "About 90 seconds, whats only.\n"
        "1. With the MPC sub-team I helped build the first MPC lane follower and the Card 2 test cases we tuned it on. "
        "I measured the real car's latency, noise and camera yaw from recordings and tuned the MPC to car 20's real "
        "steering limits.\n"
        "2. Then I added lane shifting: the MPC can keep to the left, centre or right of the lane and move across "
        "mid-run. On car 20 on 20 Oct, setting it to left kept the car on the left.\n"
        "3. Then I mapped ArUco markers to policy actions: keep a side, or turn left, right or go straight at the next "
        "junction, which uses a planned arc and hands back to lane following. Car 13 acted on a keep-right marker.\n"
        "4. Finally, real-robot tuning and testing over four car sessions: I fixed the camera feed for the marker "
        "detector, filtered the phantom obstacles the marker stands caused, and traced the failing left turns to car "
        "20's steering. The fix works in simulation; the full turn still needs a car test.")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    deck.save(OUT)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
