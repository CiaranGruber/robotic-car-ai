"""Builds the team presentation: a title slide, three team-overview slides, then each person's slide.

    python3 docs/logbook/submission/make_team_slides.py

Writes docs/logbook/submission/team-slides.pptx. Each person's slide is their own .pptx (in <name>/slide/),
rendered with LibreOffice (`soffice`) and placed as one full-slide picture, with their speaker notes copied over. So
edit a person's own slide and run this again; the overview slides are drawn here. Only the first slide of a deck is
used (Yuhao's backups stay in his own file). Needs python-pptx, LibreOffice and pdftoppm (poppler).
"""
import subprocess
import tempfile
from pathlib import Path

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN
from pptx.util import Emu, Inches, Pt

HERE = Path(__file__).resolve().parent
LOGBOOK = HERE.parent
OUT = HERE / "team-slides.pptx"
PEOPLE = [
    ("Ciaran Gruber", None),
    ("Thanh Long Nguyen", "long/slide/long-nguyen-slide.pptx"),
    ("Zhenling Zhang", "zhenling/slides/zhenling_slides_2page.pptx"),
    ("Peiyu Wang", "peiyu/slide/peiyu-wang-slide.pptx"),
    ("Toby Lee", "toby/slide/toby-lee-slide.pptx"),
    ("Yuhao Liu", "yuhao/slide/yuhao-liu-slides.pptx"),
    ("Chris (Chuhan) Wang", "chris/slide/chris-wang-slide.pptx"),
    ("Yifei Xia", "Yifei/slide/yifei-xia-slide.pptx"),
]
"""(name, slide deck) in logbook order. None: no slide uploaded yet, so a placeholder is drawn."""
INK, MUTED, BLUE = RGBColor(0x1F, 0x23, 0x28), RGBColor(0x55, 0x61, 0x6E), RGBColor(0x1F, 0x5F, 0xD6)
GREEN, AMBER = RGBColor(0x2A, 0x7F, 0x62), RGBColor(0xB7, 0x79, 0x1F)
W, H = Emu(12192000), Emu(6858000)


def text(slide, x, y, w, h, lines, size=16, colour=INK, bold=False):
    """Adds a text box. lines is a list of strings or (string, colour, bold) tuples, one paragraph each."""
    frame = slide.shapes.add_textbox(x, y, w, h).text_frame
    frame.word_wrap = True
    for i, line in enumerate(lines):
        words, c, b = (line, colour, bold) if isinstance(line, str) else line
        paragraph = frame.paragraphs[0] if i == 0 else frame.add_paragraph()
        run = paragraph.add_run()
        run.text = words
        run.font.size, run.font.bold, run.font.color.rgb = Pt(size), b, c
        paragraph.space_after = Pt(4)
    return frame


def table(slide, x, y, w, rows, widths, size=12):
    """Adds a table; the first row is the header."""
    shape = slide.shapes.add_table(len(rows), len(rows[0]), x, y, w, Inches(0.3 * len(rows)))
    for j, width in enumerate(widths):
        shape.table.columns[j].width = Inches(width)
    for i, row in enumerate(rows):
        for j, value in enumerate(row):
            cell = shape.table.cell(i, j)
            cell.text = value
            for paragraph in cell.text_frame.paragraphs:
                for run in paragraph.runs:
                    run.font.size, run.font.bold = Pt(size), i == 0
                    if value.startswith(("Sim", "Not ")):
                        run.font.color.rgb = AMBER
    return shape


def title(slide, words, subtitle=None):
    text(slide, Inches(0.5), Inches(0.3), Inches(12.3), Inches(0.7), [words], size=28, bold=True)
    if subtitle:
        text(slide, Inches(0.5), Inches(0.95), Inches(12.3), Inches(0.5), [subtitle], size=15, colour=BLUE)


def notes(slide, words):
    slide.notes_slide.notes_text_frame.text = words


def overview(deck):
    blank = deck.slide_layouts[6]

    slide = deck.slides.add_slide(blank)
    text(slide, Inches(0.8), Inches(2.3), Inches(11.5), Inches(1.2), ["AI4R System Project"], size=44, bold=True)
    text(slide, Inches(0.8), Inches(3.4), Inches(11.5), Inches(1.6), [
        ("A lane-following, obstacle-avoiding, marker-reading policy for a 1/10 car on DREAM", BLUE, False),
        ", ".join(name for name, _ in PEOPLE)], size=18, colour=MUTED)

    slide = deck.slides.add_slide(blank)
    title(slide, "Goals and what we achieved", "Stage 1: follow a straight cone lane, hold speed, stop at the end and "
          "when the lane is lost. Stage 2: curves, obstacles, markers for lane side and turns")
    table(slide, Inches(0.5), Inches(1.6), Inches(8.2), [
        ["Behaviour", "On a real car"],
        ["Speed control at 0.5 m/s (feedforward + PI)", "Car 20: 15% overshoot, 0.013 m/s error"],
        ["MPC lane following within 20 ms", "Car 20: 0.46 ms median solve"],
        ["Lost-lane stop", "Car 22: stopped 0.8 s after loss"],
        ["Keep to the left / right of the lane", "Car 20: kept left (by eye)"],
        ["Obstacle stops", "5 stops, 0.21–0.33 m short every time"],
        ["Pass obstacles left and right, stop when blocked", "26 Oct: passed 2 boxes, stopped 0.23 m short"],
        ["Read ArUco markers and act on them", "Car 13: 'Marker 14: KEEP_RIGHT'"],
        ["Turn at an intersection from a marker", "Not shown on a car; simulation only"],
        ["MPC chosen over RL (milestone M1)", "Sim: MPC passed every unseen case"],
    ], [4.4, 3.8], size=12)
    text(slide, Inches(9.0), Inches(1.6), Inches(3.9), Inches(4.5), [
        ("Best video", INK, True),
        "26 Oct: the car passes a box on the right, a box on the left, then stops 0.23 m short of a box blocking "
        "the lane.",
        ("[play video here]", AMBER, True),
        ("Process", INK, True),
        "Measure the car → simulate with the measured values → test on the car → explain each car result in "
        "simulation."], size=14)
    notes(slide, "About 1 minute. Goals of each stage, then the table: green is shown on a real car. Then play the "
               "26 Oct obstacle video. The intersection turn is the one behaviour we only showed in simulation.")

    slide = deck.slides.add_slide(blank)
    title(slide, "The system and who built it")
    slide.shapes.add_picture(str(LOGBOOK / "ciaran/figures/Overall block diagram.png"), Inches(0.4), Inches(1.1),
                             width=Inches(6.6))
    table(slide, Inches(7.2), Inches(1.1), Inches(5.8), [
        ["Sub-team", "Person", "Main work"],
        ["Interfaces", "Ciaran", "Data model, logging, CarMemory, repo, CI"],
        ["Perception", "Thanh Long", "Lane estimation, camera pan, simulator"],
        ["", "Zhenling", "Cone filter, obstacles, avoidance, local map"],
        ["Movement policy", "Peiyu", "MPC framework, Stage 1 design, lane end"],
        ["(MPC)", "Toby", "Car 20 tuning, lane side, markers, turns"],
        ["", "Yuhao", "Measured-speed MPC, junctions, stops"],
        ["Car control", "Chris", "System ID, speed and steering control"],
        ["", "Yifei", "Control tests, ArUco tests, camera pan"],
    ], [1.5, 1.2, 3.1], size=11)
    notes(slide, "About 1 minute. The policy runs once per cone batch, about 10 Hz. Data flows from the sensors "
               "through perception and the MPC to car control. Four sub-teams; each person's slide follows.")

    slide = deck.slides.add_slide(blank)
    title(slide, "How the work was split over time", "Weeks from 5 Oct; car access ended 2 Nov")
    table(slide, Inches(0.5), Inches(1.6), Inches(12.3), [
        ["Sub-team", "Wk 0–1 (to 11 Oct)", "Wk 2 (12–18 Oct)", "Wk 3 (19–25 Oct)", "Wk 4–5 (26 Oct – 2 Nov)"],
        ["Interfaces", "Repo, CI, upload, logging", "Lidar scenarios, CarMemory", "—", "Curvy-road replay"],
        ["Perception", "Cone filter, lane estimation, pan hold", "Obstacle detection, stop, brake", "Planner, local map, sim retune", "Final obstacle drive, car simulator"],
        ["Movement policy", "MPC framework, Stage 1 tests, M1: MPC vs RL", "Car 20 MPC calibration, car models", "Lane side, Foxglove, markers", "Intersection turns, left-turn diagnosis"],
        ["Car control", "System ID cars 8 and 20, PI", "CarController merged, two-box fix", "Avoidance runs on car 20", "Speed fix, steering stand test"],
    ], [1.8, 2.6, 2.6, 2.6, 2.7], size=12)
    text(slide, Inches(0.5), Inches(5.1), Inches(12.3), Inches(1.2), [
        "Integration: 29 PRs merged into dev, then dev merged to main on 7 Nov (#38). Car sessions on cars 8, 13, 14, 20, 21 and 22."],
        size=14, colour=MUTED)
    notes(slide, "About 1 minute. Read across each row: perception and control first, then the MPC on top, then "
               "Stage 2 features as the cars allowed. Hand over to Ciaran.")


def render(deck_path: Path, folder: Path) -> Path:
    """:return: A PNG of the first slide of deck_path, rendered by LibreOffice at 200 dpi."""
    subprocess.run(["soffice", "--headless", "--convert-to", "pdf", "--outdir", str(folder), str(deck_path)],
                   check=True, capture_output=True)
    pdf = folder / (deck_path.stem + ".pdf")
    stem = folder / deck_path.stem
    subprocess.run(["pdftoppm", "-r", "200", "-png", "-singlefile", "-f", "1", "-l", "1", str(pdf), str(stem)],
                   check=True)
    return stem.with_suffix(".png")


def main():
    deck = Presentation()
    deck.slide_width, deck.slide_height = W, H
    overview(deck)
    with tempfile.TemporaryDirectory() as folder:
        for name, source in PEOPLE:
            slide = deck.slides.add_slide(deck.slide_layouts[6])
            if source is None:
                title(slide, name, "Slide to add")
                notes(slide, f"{name}: slide not uploaded yet.")
                continue
            path = LOGBOOK / source
            slide.shapes.add_picture(str(render(path, Path(folder))), 0, 0, width=W, height=H)
            original = Presentation(path).slides[0]
            if original.has_notes_slide:
                notes(slide, original.notes_slide.notes_text_frame.text)
    deck.save(OUT)
    print("wrote", OUT, f"({len(deck.slides)} slides)")


if __name__ == "__main__":
    main()
