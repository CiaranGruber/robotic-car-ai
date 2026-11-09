"""Makes Thanh Long's presentation slide (docs/logbook/long/slide/long-nguyen-slide.pptx) from figures/.

    python3 docs/logbook/long/make_slide.py     # needs python-pptx; run make_figures.py first

The layout is the same as Toby's slide (docs/logbook/toby/make_slide.py), so the team's slides match.
"""
from pathlib import Path

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.util import Inches, Pt

HERE = Path(__file__).resolve().parent
FIGURES = HERE / "figures"
OUT = HERE / "slide" / "long-nguyen-slide.pptx"
DARK, BLUE, GREY = RGBColor(0x22, 0x22, 0x22), RGBColor(0x1F, 0x5F, 0xD1), RGBColor(0x66, 0x66, 0x66)
GREEN, AMBER = RGBColor(0x2A, 0x7F, 0x62), RGBColor(0xB2, 0x6B, 0x00)

CARDS = [
    ("1  Lane estimation",
     ["Built the block that turns cones into the lane: both edges fitted together as concentric arcs, misplaced "
      "cones left out",
      "Defined its output (LaneDetection) that the MPC, obstacle avoidance and intersection turns all use",
      "With a short row, 0.7° heading error against 3.0° for fitting each row alone (sim)"],
     "slide_lane.png", ("Ran on car 20, 9 Oct (no log kept)", GREEN)),
    ("2  Camera pan bias",
     ["Every recorded drive read the lane 6-11° turned, yet the car didn't drift",
      "So the cones were rotated, not the car: the camera pan was never commanded",
      "The policy now holds the pan at 0 in every step (Toby measured the same 7°)"],
     "slide_pan.png", ("Found in real drives; fix on dev since 14 Oct", GREEN)),
    ("3  Working on the lab floor",
     ["AUTO: works out which colour is on which side, because the lab used both ways round",
      "Remembers the measured width, so one visible edge still gives the centre",
      "Limit: with short rows, one wrong-colour cone can stop the car or bend the lane"],
     "slide_floor.png", ("On dev, so in every car run since 14 Oct", GREEN)),
    ("4  Whole-system simulator",
     ["Runs our real MovementPolicy and settings on 6 tracks against the measured car-20 model, in 3D",
      "76 of 90 seeded runs passed, none touched a cone; junctions are the weakest part",
      "Its camera is optimistic (20 cones a step against 14 on the car): pass rates are an upper bound"],
     "slide_sim.png", ("Simulation only; branch not merged", AMBER)),
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
    text_box(slide, 0.4, 0.2, 12.5, 0.7, [[("Thanh Long Nguyen  ", 26, True, DARK),
                                           ("Lane estimation, and making the whole system visible and testable",
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
        picture = slide.shapes.add_picture(str(FIGURES / figure), Inches(x + 0.08), Inches(top + 0.6),
                                           width=Inches(width - 0.16))
        if picture.height > Inches(2.05):  # keep every figure above the bullets
            ratio = Inches(2.05) / picture.height
            picture.height, picture.width = Inches(2.05), int(picture.width * ratio)
            picture.left = Inches(x) + (Inches(width) - picture.width) // 2
        text_box(slide, x + 0.1, top + 2.75, width - 0.2, 2.6, [[("• " + b, 12, False, DARK)] for b in bullets])
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
    text_box(slide, 0.4, 7.05, 12.5, 0.4, [[("Green = shown on a real car or in real recordings; amber = simulation "
                                             "so far. Details and evidence: logbook, Thanh Long's section.", 10, False,
                                             GREY)]])
    slide.notes_slide.notes_text_frame.text = (
        "About 1 minute 45 seconds (about 250 words). Point at each card as you reach it.\n\n"
        "Hi, I'm Thanh Long. I owned lane estimation: the block that turns the cone detections into the lane, where "
        "the edges are, where the centre line is, and where the car sits in it. The MPC, the obstacle avoidance and "
        "the intersection turns all steer from its output.\n\n"
        "[Card 1] I fit both edges together, as two arcs around one centre. That way a short row borrows the long "
        "row's direction: in simulation, with a three-cone row, the heading error fell from three degrees to under "
        "one compared with fitting each row alone. Misplaced cones are dropped and the fit is repeated.\n\n"
        "[Card 2] On the real drives, every recording said the lane was turned six to eleven degrees, but the car "
        "drove straight. If that heading were real, the car would have drifted up to 38 centimetres; it moved less "
        "than seven. So the camera was turned, not the car: the pan servo was never commanded. The policy now holds "
        "it at zero, and in my simulator that much yaw would have failed every junction.\n\n"
        "[Card 3] To work on the lab floor, it finds which colour is on which side and learns the lane width, so "
        "one edge is enough. A limit I measured: in a short row, one wrong-colour cone can stop the car.\n\n"
        "[Card 4] Last, I built a whole-system simulator that runs our real policy on six tracks in 3D. It showed "
        "the junctions are our weakest part. Its camera is optimistic, so I treat its pass rates as an upper bound.")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    deck.save(OUT)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
