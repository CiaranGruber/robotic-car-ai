"""Compiles the team overview and every person's section into one logbook, docs/logbook/logbook.md.

    python3 docs/logbook/build_logbook.py

Edit overview.md and the <name>.md files, not logbook.md: this script overwrites it. It keeps each file's text and
figures, puts the parts in pipeline order (data and interfaces, perception, movement policy, car control), demotes
every heading one level under its part, gives each part and each of its top-level sections a fixed anchor for the
contents list, and turns links between the files into links inside the one document. Figure paths stay relative to
docs/logbook, so the compiled file must stay in this folder.

    python3 docs/logbook/build_logbook.py --pdf

also prints it to docs/logbook/logbook.pdf, one self-contained file with every figure, for the submission. That needs
`pip install markdown-it-py mdit-py-plugins`, Google Chrome, and internet access while it runs (MathJax and Mermaid
are loaded from cdn.jsdelivr.net to draw the equations and flowcharts).
"""
import argparse
import html
import os
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "logbook.md"
PDF = HERE / "logbook.pdf"


def find_chrome() -> str:
    """
    :return: Path to a Chrome/Chromium executable for headless PDF printing.
    :raises FileNotFoundError: If no known install is found.
    """
    named = shutil.which("google-chrome") or shutil.which("chromium") or shutil.which("chrome")
    if named:
        return named
    candidates = [
        Path("/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"),
        Path(os.environ.get("PROGRAMFILES", r"C:\Program Files")) / "Google/Chrome/Application/chrome.exe",
        Path(os.environ.get("PROGRAMFILES(X86)", r"C:\Program Files (x86)")) / "Google/Chrome/Application/chrome.exe",
        Path(os.environ.get("LOCALAPPDATA", "")) / "Google/Chrome/Application/chrome.exe",
        Path("/usr/bin/google-chrome"),
        Path("/usr/bin/chromium"),
        Path("/usr/bin/chromium-browser"),
    ]
    for path in candidates:
        if path and path.is_file():
            return str(path)
    raise FileNotFoundError(
        "Google Chrome not found; install it or pass --chrome with the executable path"
    )
PARTS = [
    ("overview", "overview.md"),
    ("ciaran", "ciaran-gruber.md"),
    ("long", "long-nguyen.md"),
    ("zhenling", "zhenling-zhang.md"),
    ("peiyu", "peiyu-wang.md"),
    ("toby", "toby-lee.md"),
    ("yuhao", "yuhao-liu.md"),
    ("chris", "chris-wang.md"),
    ("yifei", "yifei-xia.md"),
]
"""(anchor, file) for each part, in the order they appear. Part 0 is the overview."""
ANCHOR_OF_FILE = {name: key for key, name in PARTS} | {"README.md": "overview"}
TITLE = "AI4R System Project: team logbook"
BACK_LINK = re.compile(r"^\[Back to the team logbook\]\(README\.md\)\s*$")
HEADING = re.compile(r"^(#{1,6})\s+(.*?)\s*$")
FENCE = re.compile(r"^\s*(```|~~~)")
FILE_LINK = re.compile(r"\]\(([A-Za-z]+(?:-[A-Za-z]+)*\.md)(#[^)\s]*)?\)")
LOCAL_LINK = re.compile(r"\]\(#([^)\s]+)\)")


def slug(heading: str) -> str:
    """
    :param heading: A heading's text.
    :return: The anchor GitHub gives it in a file of its own.
    """
    return re.sub(r"[^\w\- ]", "", heading.lower()).replace(" ", "-")


def link_to_anchor(match: re.Match) -> str:
    """A link to another logbook file becomes a link to that part's anchor; other links are left alone."""
    key = ANCHOR_OF_FILE.get(match.group(1))
    return f"](#{key})" if key else match.group(0)


def compile_part(number: int, key: str, path: Path) -> tuple[list[str], list[tuple[str, str]]]:
    """
    :param number: The part's number; 0 is the overview.
    :param key: The part's anchor.
    :param path: The part's source file.
    :return: The part's lines, and (anchor, heading) for its title and each of its top-level sections.
    """
    lines, contents, local = [], [], {}
    in_fence = False
    section = 0
    for line in path.read_text(encoding="utf-8").splitlines():
        if FENCE.match(line):
            in_fence = not in_fence
            lines.append(line)
            continue
        if in_fence:
            lines.append(line)
            continue
        if BACK_LINK.match(line):
            continue
        heading = HEADING.match(line)
        if heading:
            level, text = len(heading.group(1)), heading.group(2)
            if level == 1:
                title = f"Part {number}: {text}"
                lines += [f'<a id="{key}"></a>', "", f"## {title}"]
                contents.append((key, title))
                continue
            if level == 2:
                section += 1
                anchor = f"{key}-{section}"
                lines += [f'<a id="{anchor}"></a>', ""]
                contents.append((anchor, text))
                local[slug(text)] = anchor
            lines.append(f"{'#' * min(level + 1, 6)} {text}")
            continue
        lines.append(FILE_LINK.sub(link_to_anchor, line))
    # A link to a heading of the same file goes to that section's fixed anchor, since every part repeats the same
    # section names and GitHub would send it to the first part's
    lines = [LOCAL_LINK.sub(lambda m: f"](#{local.get(m.group(1), m.group(1))})", line) for line in lines]
    return lines, contents


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>{title}</title>
<style>
@page {{ size: A4; margin: 14mm 13mm; }}
body {{ font: 10pt/1.45 -apple-system, "Helvetica Neue", Arial, sans-serif; color: #1f2328; }}
h1 {{ font-size: 20pt; }} h2 {{ font-size: 16pt; break-before: page; border-bottom: 1px solid #d0d7de; }}
h2:first-of-type {{ break-before: auto; }}
h3 {{ font-size: 13pt; border-bottom: 1px solid #eaeef2; }} h4 {{ font-size: 11pt; }} h5, h6 {{ font-size: 10pt; }}
h2, h3, h4, h5, h6 {{ break-after: avoid; }}
table {{ border-collapse: collapse; margin: 6pt 0; font-size: 8.5pt; width: 100%; }}
th, td {{ border: 1px solid #d0d7de; padding: 3pt 5pt; vertical-align: top; overflow-wrap: break-word; }}
td code {{ overflow-wrap: anywhere; }}
th {{ background: #f6f8fa; }} tr, img, pre, .math {{ break-inside: avoid; }}
img {{ max-width: 100%; max-height: 235mm; display: block; margin: 4pt auto; }}
code {{ font: 8.5pt Menlo, Consolas, monospace; background: #f6f8fa; padding: 0 2pt; border-radius: 3pt; }}
pre {{ background: #f6f8fa; padding: 6pt; overflow-wrap: anywhere; white-space: pre-wrap; }}
pre.mermaid {{ background: none; text-align: center; }}
pre.mermaid svg {{ max-height: 115mm !important; max-width: 100% !important; width: auto !important; height: auto; }}
em {{ color: #444c56; }} a {{ color: #0969da; text-decoration: none; }}
hr {{ border: 0; border-top: 1px solid #d0d7de; }}
</style>
<script>window.MathJax = {{ tex: {{ inlineMath: [["\\(", "\\)"]], displayMath: [["\\[", "\\]"]] }},
  svg: {{ fontCache: "global" }} }};</script>
<script src="https://cdn.jsdelivr.net/npm/mathjax@3.2.2/es5/tex-svg.js"></script>
<script src="https://cdn.jsdelivr.net/npm/mermaid@10.9.1/dist/mermaid.min.js"></script>
<script>mermaid.initialize({{ startOnLoad: true, theme: "neutral" }});</script>
</head><body>
{body}
</body></html>
"""
"""The print page: GitHub-like tables and code, a new page for each part, MathJax for $...$ and Mermaid for flowcharts."""


def to_html(markdown: str) -> str:
    """
    :param markdown: The compiled logbook.
    :return: Its HTML body, with figures linked by absolute file URLs and maths left for MathJax.
    """
    from markdown_it import MarkdownIt
    from mdit_py_plugins.dollarmath import dollarmath_plugin

    def maths(content: str, options: dict) -> str:
        opening, closing = ("\\[", "\\]") if options["display_mode"] else ("\\(", "\\)")
        return html.escape(f"{opening}{content}{closing}")

    parser = MarkdownIt("commonmark", {"html": True}).enable(["table", "strikethrough"])
    dollarmath_plugin(parser, double_inline=True, renderer=maths)
    fence = parser.renderer.rules["fence"]

    def fences(tokens, idx, options, env):
        """```math is a display equation and ```mermaid a flowchart; other blocks stay code."""
        token = tokens[idx]
        language = token.info.strip().split(" ")[0]
        if language == "math":
            return f'<div class="math">{maths(token.content.strip(), {"display_mode": True})}</div>'
        if language == "mermaid":
            return f'<pre class="mermaid">{html.escape(token.content)}</pre>'
        return fence(tokens, idx, options, env)

    parser.renderer.rules["fence"] = fences
    body = parser.render(markdown)
    return re.sub(r'<img src="(?!https?:|file:)([^"]+)"', lambda m: f'<img src="{(HERE / m.group(1)).as_uri()}"'
                  if "%" not in m.group(1) else f'<img src="{HERE.as_uri()}/{m.group(1)}"', body)


def print_pdf(markdown: str, chrome: str):
    """Writes PDF from the compiled logbook with headless Chrome.

    :param markdown: The compiled logbook.
    :param chrome: Path to the Chrome executable.
    """
    with tempfile.TemporaryDirectory() as folder:
        page = Path(folder) / "logbook.html"
        page.write_text(PAGE.format(title=TITLE, body=to_html(markdown)), encoding="utf-8")
        subprocess.run([chrome, "--headless=new", "--disable-gpu", "--no-pdf-header-footer",
                        "--allow-file-access-from-files", "--virtual-time-budget=60000",
                        f"--print-to-pdf={PDF}", page.as_uri()], check=True, capture_output=True, timeout=300)
    print("wrote", PDF, f"({PDF.stat().st_size / 1e6:.1f} MB)")


def main():
    arguments = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    arguments.add_argument("--pdf", action="store_true", help="also print logbook.pdf (see the docstring)")
    arguments.add_argument("--chrome", default=None, help="path to Google Chrome (auto-detected if omitted)")
    options = arguments.parse_args()
    body, toc = [], []
    for number, (key, name) in enumerate(PARTS):
        lines, contents = compile_part(number, key, HERE / name)
        body += lines + ["", "---", ""]
        (part_anchor, part_title), sections = contents[0], contents[1:]
        toc.append(f"- [{part_title}](#{part_anchor})")
        toc += [f"  - [{text}](#{anchor})" for anchor, text in sections]
    header = [
        "<!-- Generated by docs/logbook/build_logbook.py from overview.md and the <name>.md files. Edit those. -->",
        "",
        f"# {TITLE}",
        "",
        "Ciaran Gruber, Thanh Long Nguyen, Zhenling Zhang, Peiyu Wang, Toby Lee, Yuhao Liu, Chris (Chuhan) Wang, "
        "Yifei Xia",
        "",
        "## Contents",
        "",
        *toc,
        "",
        "---",
        "",
    ]
    markdown = "\n".join(header + body).rstrip() + "\n"
    OUT.write_text(markdown, encoding="utf-8")
    print("wrote", OUT, f"({len(header) + len(body)} lines)")
    if options.pdf:
        print_pdf(markdown, options.chrome or find_chrome())


if __name__ == "__main__":
    main()
