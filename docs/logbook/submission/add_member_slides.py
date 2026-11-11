"""Adds each member's slide to the team presentation, in the speaking order from the presentation plan.

    python3 docs/logbook/submission/add_member_slides.py Presentation.pptx Presentation-with-members.pptx

The team deck (Ciaran's template: title, goals, project breakdown, Ciaran's slide, Peiyu's slide, closing) is kept as
it is. Each other member's slide is copied in as editable shapes (text, pictures, tables), with its background and
speaker notes, and the template's side bar hidden on it. Only the first slide of a member's deck is used (Yuhao's
backups stay in his own file). A placeholder slide for the best video is added after the project breakdown. Running
it again on a deck that already has the member slides would add them twice, so always start from the team's file.
Needs python-pptx.
"""
import argparse
import copy
import io
from pathlib import Path

from lxml import etree
from pptx import Presentation
from pptx.opc.constants import RELATIONSHIP_TYPE as RT
from pptx.util import Inches, Pt

LOGBOOK = Path(__file__).resolve().parent.parent
ORDER = ["Ciaran", "Chris", "Yifei", "Long", "Yuhao", "Toby", "Zhenling", "Peiyu"]
"""Speaking order from the presentation plan."""
SLIDES = {
    "Chris": "chris/slide/chris-wang-slide.pptx",
    "Yifei": "Yifei/slide/yifei-xia-slide.pptx",
    "Long": "long/slide/long-nguyen-slide.pptx",
    "Yuhao": "yuhao/slide/yuhao-liu-slides.pptx",
    "Toby": "toby/slide/toby-lee-slide.pptx",
    "Zhenling": "zhenling/slides/zhenling_slides_2page.pptx",
}
"""Members whose slide is copied in. Ciaran's slide is already in the team deck (found by his name), and so is
Peiyu's, pasted as one full-slide picture."""
NS = {"p": "http://schemas.openxmlformats.org/presentationml/2006/main",
      "a": "http://schemas.openxmlformats.org/drawingml/2006/main",
      "r": "http://schemas.openxmlformats.org/officeDocument/2006/relationships"}
R_ATTRIBUTES = [f"{{{NS['r']}}}embed", f"{{{NS['r']}}}link", f"{{{NS['r']}}}id"]


def slide_text(slide) -> str:
    """:return: All text on a slide, plus the descriptions of its pictures."""
    words = [shape.text_frame.text for shape in slide.shapes if shape.has_text_frame]
    words += [element.get("descr", "") for element in slide.element.iter(f"{{{NS['p']}}}cNvPr")]
    return " ".join(words)


def full_picture_slide(deck):
    """:return: The first slide that is one picture covering the slide (Peiyu's, as pasted into the deck), else None."""
    return next((slide for slide in deck.slides if len(slide.shapes) == 1 and slide.shapes[0].element.tag.endswith("}pic")
                 and slide.shapes[0].width > 0.9 * deck.slide_width), None)


def find_slide(deck, words: str):
    """:return: The first slide whose text or picture descriptions contain words, else None."""
    return next((slide for slide in deck.slides if words in slide_text(slide)), None)


def scale_shape(element, sx: float, sy: float):
    """Scales a top-level shape's position, size and font sizes (for a deck of another slide size)."""
    xfrm = next((child for child in element.iter() if child.tag.endswith("}xfrm")), None)
    if xfrm is not None:
        for tag, keys, scale in (("off", ("x", "y"), (sx, sy)), ("ext", ("cx", "cy"), (sx, sy))):
            node = xfrm.find(f"a:{tag}", NS)
            if node is not None:
                for key, factor in zip(keys, scale):
                    node.set(key, str(round(int(node.get(key)) * factor)))
    for run in element.iter():
        if run.tag.endswith(("}rPr", "}defRPr", "}endParaRPr")) and run.get("sz"):
            run.set("sz", str(round(int(run.get("sz")) * min(sx, sy))))


def theme_font(deck) -> str:
    """:return: The body (minor) font of a deck's theme."""
    theme = etree.fromstring(deck.slide_master.part.part_related_by(RT.THEME).blob)
    return theme.find(".//a:minorFont/a:latin", NS).get("typeface")


def pin_font(element, typeface: str):
    """Gives every run without its own font the member's theme font, so the text doesn't take this template's font."""
    a = f"{{{NS['a']}}}"
    for run in element.iter(f"{a}r", f"{a}fld"):
        if run.find("a:rPr", NS) is None:
            run.insert(0, etree.Element(f"{a}rPr"))
    for properties in element.iter(f"{a}rPr", f"{a}endParaRPr", f"{a}defRPr"):
        if properties.find("a:latin", NS) is None:
            latin = etree.Element(f"{a}latin", typeface=typeface)
            after = [properties.find(f"a:{tag}", NS) for tag in ("ea", "cs", "sym", "hlinkClick", "hlinkMouseOver",
                                                                 "rtl", "extLst")]
            after = [node for node in after if node is not None]
            if after:
                after[0].addprevious(latin)
            else:
                properties.append(latin)


def unplaceholder(element, source_shape):
    """Turns a copied placeholder into a plain shape at the same place, so it keeps its position in this template."""
    nv = element.find(".//p:nvPr", NS)
    ph = None if nv is None else nv.find("p:ph", NS)
    if ph is None:
        return
    nv.remove(ph)
    sppr = element.find("p:spPr", NS)
    if sppr is not None and sppr.find("a:xfrm", NS) is None:
        xfrm = etree.SubElement(sppr, f"{{{NS['a']}}}xfrm")
        sppr.insert(0, xfrm)
        etree.SubElement(xfrm, f"{{{NS['a']}}}off", x=str(source_shape.left), y=str(source_shape.top))
        etree.SubElement(xfrm, f"{{{NS['a']}}}ext", cx=str(source_shape.width), cy=str(source_shape.height))


def copy_slide(deck, source_path: Path):
    """:return: A new slide at the end of deck, a copy of the first slide of source_path."""
    source_deck = Presentation(source_path)
    source = source_deck.slides[0]
    sx = deck.slide_width / source_deck.slide_width
    sy = deck.slide_height / source_deck.slide_height
    slide = deck.slides.add_slide(next(layout for layout in deck.slide_layouts if layout.name == "Title and content 02"))
    for placeholder in list(slide.placeholders):
        placeholder.element.getparent().remove(placeholder.element)
    # Hide the template's side bar and use the member's own background (white unless they set one)
    slide.element.set("showMasterSp", "0")
    c_sld = slide.element.find("p:cSld", NS)
    background = source.element.find("p:cSld/p:bg", NS)
    if background is None:
        background = etree.fromstring(
            f'<p:bg xmlns:p="{NS["p"]}" xmlns:a="{NS["a"]}"><p:bgPr><a:solidFill><a:srgbClr val="FFFFFF"/>'
            f'</a:solidFill><a:effectLst/></p:bgPr></p:bg>')
    c_sld.insert(0, copy.deepcopy(background))
    # Pictures: copy each image the slide uses and point the copied shapes at the new relationship ids
    new_ids = {}
    for r_id, rel in source.part.rels.items():
        if rel.reltype == RT.IMAGE:
            _, new_ids[r_id] = slide.part.get_or_add_image_part(io.BytesIO(rel.target_part.blob))
    tree = slide.shapes._spTree
    font = theme_font(source_deck)
    shapes = {shape.element: shape for shape in source.shapes}
    for element in source.shapes._spTree.iterchildren():
        if element.tag.endswith(("}nvGrpSpPr", "}grpSpPr")) or element.tag.endswith("}extLst"):
            continue
        new = copy.deepcopy(element)
        for node in new.iter():
            for attribute in R_ATTRIBUTES:
                if node.get(attribute) in new_ids:
                    node.set(attribute, new_ids[node.get(attribute)])
        if element in shapes:
            unplaceholder(new, shapes[element])
        if abs(sx - 1) > 0.01 or abs(sy - 1) > 0.01:
            scale_shape(new, sx, sy)
        pin_font(new, font)
        tree.append(new)
    if source.has_notes_slide:
        slide.notes_slide.notes_text_frame.text = source.notes_slide.notes_text_frame.text
    return slide


def video_slide(deck):
    """:return: A slide for the best video, to be filled in by the team."""
    slide = deck.slides.add_slide(next(layout for layout in deck.slide_layouts if layout.name == "Title and content 01"))
    slide.shapes.title.text = "Best video"
    body = next(shape for shape in slide.placeholders if shape.placeholder_format.idx != 0)
    body.text_frame.text = "[Insert the video here: Insert > Video > This Device]"
    for line in ["Proposed: 26 Oct, car 20 (run 2026-10-26_14-10-54): passes a box on the right, a box on the left, "
                 "then stops 0.23 m short of a box blocking the lane.",
                 "Team to confirm. Real car only."]:
        paragraph = body.text_frame.add_paragraph()
        paragraph.text = line
        paragraph.runs[0].font.size = Pt(16)
    slide.notes_slide.notes_text_frame.text = (
        "About 1 minute. Play the video; while it runs say what the car is doing: lane following, box on the right, "
        "box on the left, then no gap is wide enough so it brakes and stays stopped.")
    return slide


def main():
    arguments = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    arguments.add_argument("deck", type=Path, help="the team's Presentation.pptx")
    arguments.add_argument("out", type=Path, help="where to write the deck with every member's slide")
    options = arguments.parse_args()
    deck = Presentation(options.deck)
    ids = deck.slides._sldIdLst
    original = list(ids)
    existing = {"Ciaran": find_slide(deck, "Ciaran"), "Peiyu": full_picture_slide(deck)}
    breakdown = find_slide(deck, "Project Breakdown")
    added = {name: copy_slide(deck, LOGBOOK / path) for name, path in SLIDES.items()}
    video = video_slide(deck)

    def entry(slide):
        return next(item for item in ids if item.rId == deck.part.relate_to(slide.part, RT.SLIDE))

    # Order: everything up to the project breakdown, the video, members in speaking order, then the rest
    members = {name: (existing.get(name) or added.get(name)) for name in ORDER}
    member_entries = [entry(slide) for slide in members.values() if slide is not None]
    video_entry = entry(video)
    head_end = original.index(entry(breakdown)) + 1
    head = original[:head_end]
    tail = [item for item in original[head_end:] if item not in member_entries]
    for item in list(ids):
        ids.remove(item)
    for item in head + [video_entry] + member_entries + tail:
        ids.append(item)
    deck.save(options.out)
    missing = [name for name, slide in members.items() if slide is None]
    print("wrote", options.out, f"({len(ids)} slides)", "missing: " + ", ".join(missing) if missing else "")


if __name__ == "__main__":
    main()
