import sys
from pathlib import Path
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor

base = Path(sys.argv[1])
fig = base / "figures"
out = base / "slide" / "toby-lee-slide.pptx"
prs = Presentation()
prs.slide_width, prs.slide_height = Inches(13.333), Inches(7.5)
slide = prs.slides.add_slide(prs.slide_layouts[6])
DARK, BLUE, GREY, RED = RGBColor(0x22, 0x22, 0x22), RGBColor(0x1F, 0x5F, 0xD1), RGBColor(0x66, 0x66, 0x66), RGBColor(0xC8, 0x55, 0x3D)

def text(x, y, w, h, runs, size=14):
    box = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = box.text_frame
    tf.word_wrap = True
    first = True
    for para in runs:
        p = tf.paragraphs[0] if first else tf.add_paragraph()
        first = False
        p.space_after = Pt(4)
        for chunk, bold, colour, sz in para:
            r = p.add_run()
            r.text = chunk
            r.font.bold = bold
            r.font.size = Pt(sz or size)
            r.font.color.rgb = colour or DARK
    return box

text(0.45, 0.25, 12.4, 0.6, [[("Toby Lee  ", True, DARK, 26), ("MPC sub-team: lane following, ArUco marker commands, intersection turns", False, BLUE, 20)]])
items = [
    ("Card 2 test cases + simulation harness", "T1–T11 tuning cases, unseen U1–U3, dream-gym closed loop with missed, wrong-colour, noisy and late cones; printable floor layouts (PR #6, #10)"),
    ("MPC tuned to car 20", "Measured the simulator from real recordings (0.2 s latency, 4 cm noise, 7° camera yaw); separate 0.48 / 0.92 1/m steering limits, smoother steering: 84/84 curved lanes in sim (PR #15)"),
    ("Lane side from markers", "KEEP_LEFT / RIGHT / CENTRE set the MPC's offset line; retuned so the change is visible in 0.8 m lanes (40/40 in sim). Car 13 logged \"Marker 14: KEEP_RIGHT\" (PR #27, #30, #35)"),
    ("Intersection turns from markers", "TURN_LEFT / TURN_RIGHT / STRAIGHT: move aside, planned arc, hand back to lane following, obstacle stop always on; full-policy marker simulation (PR #35)"),
    ("Found why left turns failed on the car", "Reproduced \"right works, left fails\" in sim: car 20's left limit needs a 2.08 m arc. Fix + phantom-obstacle filters: left turns 0/40 → 39/40 in sim (branch pushed, PR pending)"),
]
paras = []
for head, body in items:
    paras.append([(head, True, BLUE, 15)])
    paras.append([(body, False, DARK, 12)])
text(0.45, 1.0, 6.3, 5.6, paras)
slide.shapes.add_picture(str(fig / "06_root_cause_bars.png"), Inches(7.0), Inches(1.0), width=Inches(5.9))
slide.shapes.add_picture(str(fig / "05_turn_sim.png"), Inches(6.85), Inches(4.45), width=Inches(6.2))
text(0.45, 6.85, 12.4, 0.5, [[("On the car: marker detection and commands worked (car 13 log, 27 Oct). A full intersection turn was not shown before car access ended on 2 Nov; turn results are simulation.", False, GREY, 11)]])
slide.notes_slide.notes_text_frame.text = (
    "About 75 seconds. Whats only; the hows and whys are in the logbook.\n"
    "1. I was in the MPC sub-team. First I wrote the Card 2 test cases and the closed-loop harness we tuned against.\n"
    "2. I tuned the MPC to car 20: I measured the simulator's latency, noise and camera yaw from real recordings, "
    "gave it the car's real left and right steering limits and made the steering smoother.\n"
    "3. Then I connected ArUco markers to the policy: keep left, right or centre changes the MPC's line, and the car "
    "13 log shows it acting on a real marker.\n"
    "4. I built the intersection turn: the marker places the junction, the car moves aside, drives a planned arc and "
    "hands back to lane following, with the obstacle stop always on.\n"
    "5. When the car turned right but not left, I reproduced that exactly in simulation (left chart, red) and traced "
    "it to car 20's left steering limit. The fix brings left turns to 39 of 40 in simulation; it still needs a car test.")
out.parent.mkdir(parents=True, exist_ok=True)
prs.save(out)
print(out)
