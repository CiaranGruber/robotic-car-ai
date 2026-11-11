"""Makes Chris's presentation slide (docs/logbook/chris/slide/chris-wang-slide.pptx) from the figures in figures/.

    python3 docs/logbook/chris/make_slide.py     # needs python-pptx; run make_figures.py first
"""
from pathlib import Path

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.util import Inches, Pt

HERE = Path(__file__).resolve().parent
FIGURES = HERE / "figures"
OUT = HERE / "slide" / "chris-wang-slide.pptx"
DARK, BLUE, GREY = RGBColor(0x22, 0x22, 0x22), RGBColor(0x1F, 0x5F, 0xD1), RGBColor(0x66, 0x66, 0x66)
GREEN, AMBER = RGBColor(0x2A, 0x7F, 0x62), RGBColor(0xB2, 0x6B, 0x00)

CARDS = [
    ("1  System ID on the real car",
     ["Designed and ran the experiments: drive -> speed, steer -> curvature, dead-band, battery check",
      "Car 20: v = 9.18 (u - 0.475) m/s, tau 2.7 s, 0.2 s delay; car 8 was 3.97 (u - 0.697)",
      "Steering asymmetric: right R 1.1 m, left saturates at R 2.1 m"],
     "slide_sysid.png", ("Cars 8 + 20, real data (29 Sep, 5 Oct)", GREEN)),
    ("2  Low-level car controller",
     ["MPC commands m/s and 1/m; I turn them into drive and steer",
      "Speed: feedforward from the map + PI on wheel speed",
      "Steering: inverse map fitted per side; limits +0.48 / -0.92 1/m given to the MPC"],
     "slide_steer.png", ("Merged PR #17; car 20: 0.5 m/s, 15% overshoot", GREEN)),
    ("3  Fixing a speed fault",
     ["26 Oct: car held 0.87 m/s for a 0.5 target, then could not start",
      "Cause: integral frozen outside its band; works only if the map is within ~0.03",
      "New integral rule tested in sim, then 3 car runs: 0.50-0.54 m/s"],
     "slide_fix.png", ("Car 20, 26 Oct: 0.87 -> 0.50-0.54 m/s", GREEN)),
    ("4  Avoidance validation",
     ["Reproduced the video's two-board stop in sim, fixed 2 planner bugs (PR #23)",
      "On car 20 found a second cause: lane estimate jumps 0.35 m -> false stop",
      "Sim study: the planner's curve cap, not steering, limits obstacle spacing"],
     "slide_avoid.png", ("PR #23 merged; lane-jump fix + spacing: sim / branch", AMBER)),
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
    text_box(slide, 0.4, 0.2, 12.5, 0.7, [[("Chris (Chuhan Wang)  ", 26, True, DARK),
                                           ("Car control sub-team: from measuring the car to controlling it",
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
    text_box(slide, 0.4, 7.05, 12.5, 0.4, [[("Green = shown on a real car; amber = partly simulation or not merged. "
                                             "Details, evidence and the hows and whys: logbook, Chris's section.", 10, False,
                                             GREY)]])
    slide.notes_slide.notes_text_frame.text = (
        "About 90 seconds, whats only.\n"
        "1. I designed and ran the system ID experiments on the real car: how drive maps to speed, how fast the car "
        "responds, and how steering maps to curvature. Every car was different, so I wrote a short recalibration.\n"
        "2. From those maps I built the low-level controller: feedforward plus PI for speed, and a per-side inverse "
        "steering map, so the MPC can command metres per second and curvature. It is merged into the team code.\n"
        "3. On 26 Oct the car ran far too fast, then could not start. I traced it to the integral rule, checked the fix "
        "in simulation and on car 20, where it settled at 0.5 m/s.\n"
        "4. I validated obstacle avoidance: fixed two planner bugs found in simulation, found a lane-jump false stop on "
        "the car, and showed in simulation what limits how close two obstacles can be.")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    deck.save(OUT)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
