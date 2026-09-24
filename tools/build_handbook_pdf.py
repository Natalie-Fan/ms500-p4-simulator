"""Build docs/HANDBOOK.pdf from docs/HANDBOOK.md, with diagrams.

The Markdown file stays the single source of text; this script converts it and
inserts figures drawn from the real code (scene geometry from the firmware core,
a GUI screenshot rendered offscreen).

    .venv/Scripts/python -m pip install reportlab matplotlib   # doc-build only
    .venv/Scripts/python tools/build_handbook_pdf.py [--dataset D:/git/MS500_Flooding_Detection/dataset]
"""
from __future__ import annotations

import argparse
import datetime as dt
import os
import re
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
# The offscreen platform ships no fonts; without this every label renders as boxes.
os.environ.setdefault("QT_QPA_FONTDIR", str(Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle  # noqa: E402

from reportlab.lib import colors  # noqa: E402
from reportlab.lib.enums import TA_CENTER  # noqa: E402
from reportlab.lib.pagesizes import A4  # noqa: E402
from reportlab.lib.styles import ParagraphStyle  # noqa: E402
from reportlab.lib.units import mm  # noqa: E402
from reportlab.pdfbase import pdfmetrics  # noqa: E402
from reportlab.pdfbase.ttfonts import TTFont  # noqa: E402
from reportlab.platypus import (Image, KeepTogether, PageBreak, Paragraph, SimpleDocTemplate,  # noqa: E402
                                Spacer, Table, TableStyle, XPreformatted)
from reportlab.platypus.flowables import CondPageBreak  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
MD = REPO / "docs" / "HANDBOOK.md"
OUT = REPO / "docs" / "HANDBOOK.pdf"
FIG = REPO / "build" / "handbook"

ACCENT = "#1f4e79"
INK = "#222222"
CURB, LANE, STRIP, ALIGN = "#e03030", "#1ea0d8", "#e0b800", "#2e9e4f"


# --------------------------------------------------------------------- figures

def _box(ax, x, y, w, h, text, fc="#eef3f8", ec=ACCENT, size=9, bold=False):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.06",
                                fc=fc, ec=ec, lw=1.2))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=size,
            color=INK, weight="bold" if bold else "normal", wrap=True)


def _arrow(ax, a, b, color=ACCENT, style="-|>", ls="-"):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle=style, mutation_scale=12, color=color, lw=1.3,
                                 linestyle=ls))


def _canvas(w, h):
    fig, ax = plt.subplots(figsize=(w, h), dpi=200)
    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    ax.axis("off")
    return fig, ax


def _save(fig, name):
    FIG.mkdir(parents=True, exist_ok=True)
    path = FIG / f"{name}.png"
    fig.savefig(path, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return path


def fig_architecture():
    fig, ax = _canvas(9, 4.3)
    _box(ax, 0.2, 3.1, 3.3, 0.9, "Firmware repo (unmodified)\nprocess_flood_detection/*.c", fc="#fdf2e3", ec="#b36b00")
    _box(ax, 0.2, 1.7, 3.3, 0.9, "native/sim_shim.c\nflat C API · device loop", fc="#fdf2e3", ec="#b36b00")
    _box(ax, 4.3, 3.1, 2.1, 0.9, "flood_core.dll", bold=True)
    _box(ax, 4.3, 1.7, 2.1, 0.9, "flood_sim.exe\n(pure C)", bold=True)
    _box(ax, 7.0, 3.1, 1.8, 0.9, "floodsim\n(Python)", bold=True)
    _box(ax, 7.0, 1.55, 0.85, 0.75, "CLI", size=9)
    _box(ax, 7.95, 1.55, 0.85, 0.75, "GUI", size=9)
    _box(ax, 0.2, 0.2, 8.6, 0.9, "Video · images · folders   →   decode · YOLO boxes (cached) · runners · labels · reports",
         fc="#f4f4f4", ec="#888888", size=8.5)
    _arrow(ax, (1.85, 3.1), (1.85, 2.6))
    ax.text(2.0, 2.82, "compiled in place", fontsize=7.5, color="#b36b00")
    _arrow(ax, (3.5, 2.15), (4.3, 3.45))
    _arrow(ax, (3.5, 2.15), (4.3, 2.15))
    _arrow(ax, (6.4, 3.55), (7.0, 3.55))
    ax.text(6.42, 3.7, "ctypes", fontsize=7.5, color=ACCENT)
    _arrow(ax, (7.9, 3.1), (7.45, 2.3))
    _arrow(ax, (7.9, 3.1), (8.35, 2.3))
    _arrow(ax, (5.35, 1.7), (5.35, 1.1), ls="--")
    ax.text(5.45, 1.3, "PGM frames", fontsize=7.5, color=ACCENT)
    _arrow(ax, (7.9, 1.55), (7.9, 1.1), ls="--")
    return _save(fig, "architecture")


def fig_pipeline():
    fig, ax = _canvas(9.6, 3.6)
    _box(ax, 0.1, 1.5, 1.4, 0.9, "Frame\n1920×1080 RGB")
    _box(ax, 1.9, 1.5, 1.4, 0.9, "Gray\nr·77+g·151+b·28\n>> 8", size=7)
    _box(ax, 3.7, 1.5, 1.9, 0.9, "Stage 1 · veto\nbox hides ≥15 % of curb?", size=8)
    _box(ax, 6.0, 1.5, 1.9, 0.9, "Stage 2 · curb check\nalign → correlation", size=8)
    _box(ax, 8.3, 2.55, 1.2, 0.6, "UPLOAD", fc="#fbe3e3", ec="#c0392b", bold=True, size=8)
    _box(ax, 8.3, 1.65, 1.2, 0.6, "CLEAR", fc="#e3f5e8", ec="#2e8b57", bold=True, size=8)
    _box(ax, 8.3, 0.75, 1.2, 0.6, "HEARTBEAT", fc="#e3edfb", ec="#1f77b4", bold=True, size=7.5)
    _box(ax, 3.9, 0.1, 1.5, 0.6, "retry next frame", fc="#f4f4f4", ec="#888888", size=7.5)
    _box(ax, 1.4, 0.1, 1.5, 0.6, "BLOCKED\n(abstain)", fc="#eeeeee", ec="#666666", bold=True, size=7.5)
    for a, b in (((1.5, 1.95), (1.9, 1.95)), ((3.3, 1.95), (3.7, 1.95)), ((5.6, 1.95), (6.0, 1.95))):
        _arrow(ax, a, b)
    ax.text(5.8, 2.5, "usable", fontsize=7, color=ACCENT, ha="center")
    _arrow(ax, (7.9, 2.2), (8.3, 2.85))
    ax.text(7.2, 2.95, "corr < 0.45", fontsize=7, color="#c0392b")
    _arrow(ax, (7.9, 1.95), (8.3, 1.95))
    _arrow(ax, (7.9, 1.7), (8.3, 1.05))
    ax.text(7.35, 1.05, "60th clear\nreading", fontsize=6.5, color="#1f77b4")
    _arrow(ax, (4.65, 1.5), (4.65, 0.7))
    ax.text(4.72, 1.0, "obstructed", fontsize=7, color="#666666")
    _arrow(ax, (3.9, 0.4), (2.9, 0.4))
    ax.text(2.95, 0.75, "still obstructed", fontsize=7, color="#666666")
    _arrow(ax, (5.4, 0.4), (6.95, 1.5), ls="--")
    ax.text(6.0, 0.55, "clear on retry", fontsize=7, color=ACCENT)
    return _save(fig, "pipeline")


def fig_geometry(dataset: Path | None):
    import cv2
    import numpy as np
    from floodsim.core import load_core
    from floodsim.geometry import Geometry

    core, _ = load_core()
    core.set_geometry(Geometry.from_list(core.default_geometry()))
    g = core.default_geometry()
    xs, lo, hi = core.strip_outline()
    ax0, ay0, ax1, ay1 = core.align_patch()
    d0, d1 = core.domain()

    fig, ax = plt.subplots(figsize=(9, 5.2), dpi=200)
    img = None
    if dataset and (dataset / "dry_reference_full.jpg").exists():
        bgr = cv2.imdecode(np.fromfile(str(dataset / "dry_reference_full.jpg"), np.uint8), cv2.IMREAD_COLOR)
        img = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    if img is not None:
        ax.imshow(img, extent=(0, 1920, 1080, 0), alpha=0.85)
    else:
        ax.add_patch(Rectangle((0, 0), 1920, 1080, fc="#dddddd"))
    ax.set_xlim(0, 1920)
    ax.set_ylim(1080, 0)
    ax.add_patch(Rectangle((ax0, ay0), ax1 - ax0, ay1 - ay0, fill=False, ec=ALIGN, lw=1.8, ls="--"))
    ax.text(ax0 + 10, ay1 - 12, "align patch (camera-shift estimate)", color="white", fontsize=8,
            bbox=dict(fc=ALIGN, ec="none", pad=2))
    poly = list(zip(xs, lo)) + list(zip(xs[::-1], hi[::-1]))
    ax.add_patch(Polygon(poly, closed=True, fc=STRIP, alpha=0.35, ec=STRIP, lw=1.5))
    for x in (d0, d1):
        ax.axvline(x, color=STRIP, lw=1, ls=":")
    ax.plot(g[0:4:2], g[1:4:2], color=CURB, lw=2.4)
    ax.plot(g[4:8:2], g[5:8:2], color=LANE, lw=2.4)
    ax.annotate("curb line", (1300, 230 + 125 * 740 / 1360), (1350, 150), color="white", fontsize=9,
                bbox=dict(fc=CURB, ec="none", pad=2), arrowprops=dict(arrowstyle="->", color=CURB))
    ax.annotate("lane line", (1500, 378 + 68 * 940 / 1360), (1560, 560), color="white", fontsize=9,
                bbox=dict(fc=LANE, ec="none", pad=2), arrowprops=dict(arrowstyle="->", color=LANE))
    ax.annotate("sampled kerb strip\n(compared with the dry reference)", (xs[len(xs) // 2], lo[len(lo) // 2]),
                (760, 470), color=INK, fontsize=8.5, bbox=dict(fc="white", ec=STRIP, pad=3),
                arrowprops=dict(arrowstyle="->", color="#a08000"))
    ax.text(d0 + 8, 1060, "domain", color=INK, fontsize=7.5, bbox=dict(fc="white", ec="none", pad=1))
    ax.set_xticks([0, 480, 960, 1440, 1920])
    ax.set_yticks([0, 270, 540, 810, 1080])
    ax.tick_params(labelsize=7)
    ax.set_title("Scene geometry on the dry reference (firmware default lines)", fontsize=10, color=INK)
    return _save(fig, "geometry")


def fig_scale():
    fig, ax = plt.subplots(figsize=(9, 1.9), dpi=200)
    bands = [(-1.0, -0.10, "severe", "#8e1b1b"), (-0.10, 0.05, "moderate", "#d9534f"),
             (0.05, 0.45, "minor", "#f0ad4e"), (0.45, 1.0, "none (curb visible)", "#5cb85c")]
    for a, b, name, c in bands:
        ax.add_patch(Rectangle((a, 0), b - a, 1, fc=c, ec="white"))
        ax.text((a + b) / 2, 0.5, name, ha="center", va="center", color="white", fontsize=8.5, weight="bold")
    ax.axvline(0.45, color="black", lw=2)
    ax.text(0.45, 1.12, "threshold 0.45: below = submerged", ha="center", fontsize=8)
    for v, lab in ((0.408, "Dry-day, no alignment 0.41"), (0.533, "Dry-day aligned 0.53"),
                   (0.832, "Dry-night aligned 0.83")):
        ax.plot([v], [-0.18], marker="^", color=INK, ms=6)
    ax.text(0.53, -0.5, "dataset examples ▲ (Dry-day 0.53, Dry-night 0.83 after alignment)", ha="center", fontsize=7.5)
    ax.set_xlim(-1, 1)
    ax.set_ylim(-0.7, 1.35)
    ax.set_yticks([])
    ax.set_xticks([-1, -0.5, 0, 0.5, 1])
    ax.tick_params(labelsize=7.5)
    ax.set_xlabel("curb correlation with the dry reference", fontsize=8)
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    return _save(fig, "scale")


def fig_timeline():
    fig, ax = plt.subplots(figsize=(9, 2.3), dpi=200)
    events = [(0, "clear", "#2e8b57"), (60, "clear", "#2e8b57"), (120, "blocked → retry", "#666666"),
              (180, "upload", "#c0392b"), (240, "clear", "#2e8b57"), (300, "skipped\n(no ref)", "#ff7f0e")]
    ax.axhline(0, color="#999999", lw=1)
    for t, lab, c in events:
        ax.plot([t], [0], marker="v", color=c, ms=11)
        ax.text(t, 0.35, lab, ha="center", fontsize=8, color=c)
    ax.plot([121], [0], marker="v", color="#2e8b57", ms=7)
    ax.annotate("retry frame t+1 s → clear", (121, -0.05), (150, -0.6), fontsize=7.5,
                arrowprops=dict(arrowstyle="->", color="#2e8b57"), color="#2e8b57")
    ax.text(180, -0.6, "image sent", fontsize=7.5, color="#c0392b", ha="center")
    ax.text(300, -0.6, "(only if no dry reference is set)", fontsize=7, color="#ff7f0e", ha="center")
    ax.set_xlim(-15, 330)
    ax.set_ylim(-0.9, 0.8)
    ax.set_yticks([])
    ax.set_xticks(range(0, 331, 60))
    ax.set_xlabel("video time (s): one device sample per 60 s; heartbeat on the 60th clear reading", fontsize=8)
    ax.tick_params(labelsize=7.5)
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    return _save(fig, "timeline")


def fig_regression():
    fig, ax = _canvas(9, 1.9)
    steps = ["floodsim run …\n--out out/before", "edit firmware C\n(NETWORK_P4_…)", "floodsim run …\n--out out/after\n(auto-rebuild)",
             "floodsim compare\nout/before out/after", "exit 0 = identical\nexit 1 = changed"]
    x = 0.1
    for i, s in enumerate(steps):
        _box(ax, x, 0.45, 1.6, 1.0, s, size=7.8, fc="#fdf2e3" if i == 1 else "#eef3f8",
             ec="#b36b00" if i == 1 else ACCENT)
        if i < len(steps) - 1:
            _arrow(ax, (x + 1.6, 0.95), (x + 1.8, 0.95))
        x += 1.8
    return _save(fig, "regression")


def fig_gui(dataset: Path | None):
    """Real GUI rendered offscreen on the dataset frames with the dry reference."""
    if not dataset or not (dataset / "frames").exists():
        return None
    from PySide6.QtWidgets import QApplication
    from floodsim.core import load_core
    from floodsim.gui.main_window import MainWindow
    from floodsim.gui.session import Session
    from floodsim.gui.worker import Worker
    from floodsim.modes import RunParams

    app = QApplication.instance() or QApplication([])
    core, info = load_core()
    worker = Worker()
    session = Session(core, info)
    win = MainWindow(session, worker, info)
    win.resize(1600, 950)
    win.show()

    def wait(cond, ms=20000):
        import time
        end = time.monotonic() + ms / 1000
        while not cond() and time.monotonic() < end:
            app.processEvents()
            time.sleep(0.01)

    win.open_path(dataset / "frames", fps=1.0)
    wait(lambda: win._last_frame is not None)
    worker.submit(lambda: session.set_reference_image(dataset / "dry_reference_full.jpg"), win._after_reference)
    wait(lambda: win.verdict.reference_label.text().startswith("image"))
    worker.submit(lambda: session.run_batch(RunParams(mode="dense")), win._on_run_done)
    wait(lambda: win.samples.rowCount() > 0)
    win.show_frame(0)
    wait(lambda: win._last_frame is not None and win._last_frame.score is not None)
    for _ in range(50):
        app.processEvents()
    FIG.mkdir(parents=True, exist_ok=True)
    path = FIG / "gui.png"
    win.grab().save(str(path))
    win.close()
    worker.stop()
    return path


# ---------------------------------------------------------------------- fonts

def register_fonts():
    win = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts"
    pdfmetrics.registerFont(TTFont("Body", str(win / "arial.ttf")))
    pdfmetrics.registerFont(TTFont("Body-Bold", str(win / "arialbd.ttf")))
    pdfmetrics.registerFont(TTFont("Body-Italic", str(win / "ariali.ttf")))
    pdfmetrics.registerFont(TTFont("Mono", str(win / "consola.ttf")))
    pdfmetrics.registerFont(TTFont("Sym", str(win / "seguisym.ttf")))
    pdfmetrics.registerFontFamily("Body", normal="Body", bold="Body-Bold", italic="Body-Italic",
                                  boldItalic="Body-Bold")
    return {name: set(pdfmetrics.getFont(name).face.charToGlyph) for name in ("Body", "Mono", "Sym")}


CMAPS: dict[str, set[int]] = {}


def glyphs(text: str, font: str) -> str:
    """Route characters the font lacks to Segoe UI Symbol (already XML-escaped text)."""
    out = []
    for ch in text:
        if ord(ch) < 128 or ord(ch) in CMAPS[font]:
            out.append(ch)
        elif ord(ch) in CMAPS["Sym"]:
            out.append(f'<font name="Sym">{ch}</font>')
        else:
            out.append("?")
    return "".join(out)


def esc(s: str) -> str:
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def inline(s: str) -> str:
    s = re.sub(r"\[([^\]]+)\]\([^)]+\)", r"\1", s)          # links -> text
    parts = re.split(r"(`[^`]+`)", s)
    out = []
    for p in parts:
        if p.startswith("`") and p.endswith("`") and len(p) > 1:
            out.append(f'<font name="Mono" color="#8a2b06">{glyphs(esc(p[1:-1]), "Mono")}</font>')
        else:
            t = glyphs(esc(p), "Body")
            t = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", t)
            t = re.sub(r"(?<![\w*])\*(?!\s)(.+?)(?<!\s)\*(?![\w*])", r"<i>\1</i>", t)
            out.append(t)
    return "".join(out)


# --------------------------------------------------------------------- styles

def styles():
    base = dict(fontName="Body", textColor=colors.HexColor(INK))
    return {
        "body": ParagraphStyle("body", fontSize=9.6, leading=13.4, spaceAfter=5, **base),
        "h1": ParagraphStyle("h1", fontName="Body-Bold", fontSize=17, leading=21, spaceBefore=6,
                             spaceAfter=8, textColor=colors.HexColor(ACCENT)),
        "h2": ParagraphStyle("h2", fontName="Body-Bold", fontSize=12.5, leading=16, spaceBefore=10,
                             spaceAfter=5, textColor=colors.HexColor(ACCENT)),
        "h3": ParagraphStyle("h3", fontName="Body-Bold", fontSize=10.5, leading=14, spaceBefore=8,
                             spaceAfter=4, textColor=colors.HexColor("#333333")),
        "bullet": ParagraphStyle("bullet", fontSize=9.6, leading=13.2, spaceAfter=2.5, **base),
        "cell": ParagraphStyle("cell", fontSize=8.3, leading=10.6, **base),
        "cellh": ParagraphStyle("cellh", fontName="Body-Bold", fontSize=8.3, leading=10.6,
                                textColor=colors.white),
        "code": ParagraphStyle("code", fontName="Mono", fontSize=7.7, leading=10,
                               textColor=colors.HexColor("#1b1b1b")),
        "caption": ParagraphStyle("caption", fontName="Body-Italic", fontSize=8.3, leading=11,
                                  alignment=TA_CENTER, textColor=colors.HexColor("#555555"), spaceAfter=8),
    }


FRAME_W = A4[0] - 36 * mm


def figure(path, caption, st, max_h=110 * mm):
    if path is None:
        return []
    from PIL import Image as PILImage
    w, h = PILImage.open(path).size
    scale = min(FRAME_W / w, max_h / h)
    return [Spacer(1, 3), KeepTogether([Image(str(path), w * scale, h * scale),
                                        Paragraph(glyphs(esc(caption), "Body-Italic" if False else "Body"), st["caption"])])]


# ------------------------------------------------------------------ markdown

def parse_table(lines, st):
    rows = [[c.strip() for c in ln.strip().strip("|").split("|")] for ln in lines
            if not re.match(r"^\s*\|?\s*:?-{3,}", ln)]
    ncol = len(rows[0])
    lens = [max(len(re.sub(r"[*`]", "", r[i])) if i < len(r) else 0 for r in rows) for i in range(ncol)]
    lens = [min(max(n, 6), 60) for n in lens]
    widths = [FRAME_W * n / sum(lens) for n in lens]
    data = [[Paragraph(inline(c), st["cellh" if i == 0 else "cell"]) for c in r] + [""] * (ncol - len(r))
            for i, r in enumerate(rows)]
    t = Table(data, colWidths=widths, repeatRows=1)
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor(ACCENT)),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f2f6fa")]),
        ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#c8d3de")),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 4), ("RIGHTPADDING", (0, 0), (-1, -1), 4),
        ("TOPPADDING", (0, 0), (-1, -1), 2.5), ("BOTTOMPADDING", (0, 0), (-1, -1), 2.5),
    ]))
    return [t, Spacer(1, 6)]


def code_block(text, st):
    body = glyphs(esc(text.rstrip("\n")), "Mono")
    t = Table([[XPreformatted(body, st["code"])]], colWidths=[FRAME_W])
    t.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), colors.HexColor("#f5f5f2")),
                           ("BOX", (0, 0), (-1, -1), 0.5, colors.HexColor("#d8d8d0")),
                           ("LEFTPADDING", (0, 0), (-1, -1), 6), ("TOPPADDING", (0, 0), (-1, -1), 4),
                           ("BOTTOMPADDING", (0, 0), (-1, -1), 4)]))
    return [t, Spacer(1, 6)]


def convert(md: str, st, figs: dict, hooks: dict):
    """hooks: heading text -> ("after" | "replace_code", [flowables])."""
    story = []
    lines = md.split("\n")
    i = 0
    replace_next_code = None
    first_h1 = True

    def para_lines(start):
        j = start
        buf = []
        while j < len(lines) and lines[j].strip() and not re.match(r"^(#|\s*[-*] |\s*\d+\. |\||```|---)", lines[j]):
            buf.append(lines[j].strip())
            j += 1
        return " ".join(buf), j

    while i < len(lines):
        ln = lines[i]
        s = ln.strip()
        if not s or s == "---":
            i += 1
            continue
        m = re.match(r"^(#{1,3}) (.*)", ln)
        if m:
            level, text = len(m.group(1)), m.group(2).strip()
            if level == 1 and first_h1:
                first_h1 = False            # title goes on the cover
                i += 1
                continue
            if level == 2 and text.startswith(tuple(f"{n}." for n in range(2, 12))):
                story.append(CondPageBreak(95 * mm))
            story.append(Paragraph(inline(text), st[f"h{min(level + (0 if level > 1 else 0), 3)}"]
                                   if level > 1 else st["h1"]))
            hook = hooks.get(text)
            if hook:
                kind, flow = hook
                if kind == "after":
                    story.extend(flow)
                elif kind == "replace_code":
                    replace_next_code = flow
            i += 1
            continue
        if s.startswith("```"):
            j = i + 1
            buf = []
            while j < len(lines) and not lines[j].strip().startswith("```"):
                buf.append(lines[j])
                j += 1
            if replace_next_code is not None:
                story.extend(replace_next_code)
                replace_next_code = None
            else:
                story.extend(code_block("\n".join(buf), st))
            i = j + 1
            continue
        if s.startswith("|"):
            j = i
            while j < len(lines) and lines[j].strip().startswith("|"):
                j += 1
            story.extend(parse_table(lines[i:j], st))
            i = j
            continue
        lm = re.match(r"^(\s*)([-*]|\d+\.) (.*)", ln)
        if lm:
            indent = len(lm.group(1)) // 2
            marker = "•" if lm.group(2) in "-*" else lm.group(2)
            buf = [lm.group(3).strip()]
            j = i + 1
            while j < len(lines) and lines[j].strip() and not re.match(r"^(\s*)([-*]|\d+\.) ", lines[j]) \
                    and lines[j].startswith(" ") and not lines[j].strip().startswith("```"):
                buf.append(lines[j].strip())
                j += 1
            style = ParagraphStyle("b", parent=st["bullet"], leftIndent=14 + 14 * indent, bulletIndent=3 + 14 * indent)
            story.append(Paragraph(inline(" ".join(buf)), style, bulletText=marker))
            i = j
            continue
        text, j = para_lines(i)
        story.append(Paragraph(inline(text), st["body"]))
        i = j if j > i else i + 1
    return story


# ---------------------------------------------------------------------- build

def cover(st, hero):
    title = ParagraphStyle("t", fontName="Body-Bold", fontSize=26, leading=31, textColor=colors.HexColor(ACCENT))
    sub = ParagraphStyle("s", fontName="Body", fontSize=12.5, leading=17, textColor=colors.HexColor("#444444"))
    out = [Spacer(1, 30 * mm), Paragraph("MS500-P4 Flood Simulator", title), Spacer(1, 4),
           Paragraph("Handbook", ParagraphStyle("h", parent=title, fontSize=18, textColor=colors.HexColor("#555555"))),
           Spacer(1, 10),
           Paragraph("Run the camera firmware's flood-detection C code, unmodified, on recorded video and images: "
                     "evaluate, regression-test and tune it on your PC.", sub),
           Spacer(1, 12 * mm)]
    if hero:
        from PIL import Image as PILImage
        w, h = PILImage.open(hero).size
        s = FRAME_W / w
        out.append(Image(str(hero), w * s, h * s))
    out += [Spacer(1, 10 * mm),
            Paragraph(f"Repository: D:\\git\\ms500-p4-simulator · generated {dt.date.today():%Y-%m-%d} "
                      f"from docs/HANDBOOK.md", st["caption"]),
            PageBreak()]
    return out


def on_page(canvas, doc):
    if doc.page == 1:
        return
    canvas.saveState()
    canvas.setFont("Body", 7.5)
    canvas.setFillColor(colors.HexColor("#777777"))
    canvas.drawString(18 * mm, 10 * mm, "MS500-P4 Flood Simulator · Handbook")
    canvas.drawRightString(A4[0] - 18 * mm, 10 * mm, f"{doc.page}")
    canvas.setStrokeColor(colors.HexColor("#d0d7de"))
    canvas.line(18 * mm, 13 * mm, A4[0] - 18 * mm, 13 * mm)
    canvas.restoreState()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", type=Path, default=Path("D:/git/MS500_Flooding_Detection/dataset"))
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()
    dataset = args.dataset if args.dataset.exists() else None

    CMAPS.update(register_fonts())
    st = styles()
    f = {
        "architecture": fig_architecture(), "pipeline": fig_pipeline(), "geometry": fig_geometry(dataset),
        "scale": fig_scale(), "timeline": fig_timeline(), "regression": fig_regression(), "gui": fig_gui(dataset),
    }
    hooks = {
        "Contents": ("after", []),
        "1. Setup": ("after", figure(f["architecture"], "Figure 1 - How the pieces fit: the firmware C code is "
                                     "compiled in place into a DLL (used by Python) and a standalone exe.", st)),
        "2. How the detector works (the 2-minute version)": ("after",
            figure(f["pipeline"], "Figure 2 - One device sample: veto first, then the curb check; "
                                  "an obstructed frame is retried once, then reported as BLOCKED.", st)
            + figure(f["geometry"], "Figure 3 - What the firmware looks at: curb and lane lines, the sampled "
                                    "kerb strip and the alignment patch.", st, max_h=105 * mm)),
        "What one device sample can report": ("after", figure(f["scale"], "Figure 4 - Correlation, "
                                               "severity bands and the 0.45 threshold.", st)),
        "Three ways to run it": ("after", figure(f["timeline"], "Figure 5 - Device mode on a video "
                                  "timeline: sample every 60 s, retry after 1 s when obstructed.", st)),
        "Layout": ("replace_code", figure(f["gui"], "Figure 6 - The GUI (layout A) on the dataset frames "
                                            "with the dry reference and a dense run.", st, max_h=120 * mm)
                   if f["gui"] else None),
        "Check a firmware change before flashing": ("after", figure(f["regression"], "Figure 7 - "
                                                     "Regression check before flashing.", st)),
    }
    if hooks["Layout"][1] is None:
        del hooks["Layout"]

    story = cover(st, f["geometry"]) + convert(MD.read_text(encoding="utf-8"), st, f, hooks)
    doc = SimpleDocTemplate(str(args.out), pagesize=A4, leftMargin=18 * mm, rightMargin=18 * mm,
                            topMargin=16 * mm, bottomMargin=18 * mm,
                            title="MS500-P4 Flood Simulator - Handbook", author="floodsim")
    doc.build(story, onFirstPage=on_page, onLaterPages=on_page)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
