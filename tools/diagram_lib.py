"""
Diagram source -> Draw.io (.drawio) and LaTeX-ready PDF, from one description.

Why generate rather than draw by hand: a diagram that disagrees with the code
is worse than no diagram, and hand-drawn ones always drift. Here each figure is
a short Python description in tools/make_diagrams.py, and both outputs come
from it:

    docs/diagrams/*.drawio   editable in Draw.io / diagrams.net, the artefact
                             an engineer opens to change the architecture
    docs/figures/*.pdf       vector, included directly by the LaTeX document

The .drawio files are real mxGraph XML, not an export: open, edit and re-render
by hand if you prefer, or change the Python and regenerate both.

Run:  python3 tools/make_diagrams.py
"""
from __future__ import annotations

import html
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

# --------------------------------------------------------------------------- #
# palette -- shared by both renderers so the PDF and the .drawio match
# --------------------------------------------------------------------------- #
STYLE = {
    #                fill        stroke      text
    "layer":      ("#eef2f7", "#8fa6bf", "#26333f"),
    "package":    ("#dbeafe", "#5b8def", "#14305e"),
    "node":       ("#dcfce7", "#3ecf8e", "#0f3f2c"),
    "topic":      ("#fef3c7", "#e0a800", "#4a3600"),
    "service":    ("#fde8e8", "#e88", "#5a1d1d"),
    "hardware":   ("#ffe8d6", "#f0894e", "#5c2c10"),
    "external":   ("#ede9fe", "#8b7cf6", "#2e2159"),
    "store":      ("#e5e7eb", "#9ca3af", "#26333f"),
    "state":      ("#dcfce7", "#3ecf8e", "#0f3f2c"),
    "state_bad":  ("#fde8e8", "#e05252", "#5a1d1d"),
    "note":       ("#ffffff", "#c8d0da", "#5a6675"),
    "boundary":   ("#ffffff", "#e05252", "#8a2020"),
}
FONT = "Helvetica"


@dataclass
class Box:
    id: str
    x: float
    y: float
    w: float
    h: float
    label: str
    sub: str = ""            # smaller second line, drawn under `label`
    kind: str = "node"
    dashed: bool = False
    rounded: bool = True
    z: int = 1               # drawn in ascending order; containers use z=0


@dataclass
class Edge:
    src: str
    dst: str
    label: str = ""
    style: str = "solid"     # solid | dashed | thick
    bidir: bool = False
    #: routing hint: 'auto', 'h' (horizontal first), 'v' (vertical first)
    route: str = "auto"
    #: fraction along the edge at which to place the label
    lpos: float = 0.5


@dataclass
class Diagram:
    name: str
    title: str
    boxes: list[Box] = field(default_factory=list)
    edges: list[Edge] = field(default_factory=list)
    w: float = 1200
    h: float = 800
    caption: str = ""

    def box(self, *a, **k) -> Box:
        b = Box(*a, **k)
        self.boxes.append(b)
        return b

    def edge(self, *a, **k) -> Edge:
        e = Edge(*a, **k)
        self.edges.append(e)
        return e

    def find(self, bid: str) -> Box:
        for b in self.boxes:
            if b.id == bid:
                return b
        raise KeyError(f"{self.name}: no box '{bid}'")


# --------------------------------------------------------------------------- #
# Draw.io (mxGraph XML)
# --------------------------------------------------------------------------- #
def to_drawio(d: Diagram) -> str:
    mxfile = ET.Element("mxfile", host="robodog", modified="", agent="tools/make_diagrams.py")
    diagram = ET.SubElement(mxfile, "diagram", id=d.name, name=d.title)
    model = ET.SubElement(diagram, "mxGraphModel", dx="1400", dy="900", grid="1",
                          gridSize="10", guides="1", tooltips="1", connect="1",
                          arrows="1", fold="1", page="1", pageScale="1",
                          pageWidth=str(int(d.w)), pageHeight=str(int(d.h)),
                          math="0", shadow="0")
    root = ET.SubElement(model, "root")
    ET.SubElement(root, "mxCell", id="0")
    ET.SubElement(root, "mxCell", id="1", parent="0")

    for b in sorted(d.boxes, key=lambda x: x.z):
        fill, stroke, text = STYLE.get(b.kind, STYLE["node"])
        style = (f"rounded={1 if b.rounded else 0};whiteSpace=wrap;html=1;"
                 f"fillColor={fill};strokeColor={stroke};fontColor={text};"
                 f"fontFamily={FONT};fontSize=12;arcSize=8;verticalAlign=middle;"
                 f"align=center;spacing=4;")
        if b.dashed:
            style += "dashed=1;dashPattern=6 4;"
        if b.kind in ("layer", "boundary"):
            style += "verticalAlign=top;fontStyle=1;fontSize=11;"
        label = html.escape(b.label)
        if b.sub:
            label += f"<br/><font style='font-size:10px;opacity:0.75'>{html.escape(b.sub)}</font>"
        cell = ET.SubElement(root, "mxCell", id=b.id, value=label, style=style,
                             vertex="1", parent="1")
        ET.SubElement(cell, "mxGeometry", x=str(b.x), y=str(b.y),
                      width=str(b.w), height=str(b.h), **{"as": "geometry"})

    for i, e in enumerate(d.edges):
        style = ("edgeStyle=orthogonalEdgeStyle;rounded=1;html=1;"
                 f"fontFamily={FONT};fontSize=10;fontColor=#4a5568;"
                 "strokeColor=#7a8899;endArrow=block;endFill=1;")
        if e.style == "dashed":
            style += "dashed=1;dashPattern=6 4;"
        if e.style == "thick":
            style += "strokeWidth=2.5;strokeColor=#3b82f6;"
        if e.bidir:
            style += "startArrow=block;startFill=1;"
        cell = ET.SubElement(root, "mxCell", id=f"e{i}", value=html.escape(e.label),
                             style=style, edge="1", parent="1",
                             source=e.src, target=e.dst)
        ET.SubElement(cell, "mxGeometry", relative="1", **{"as": "geometry"})

    raw = ET.tostring(mxfile, encoding="unicode")
    return '<?xml version="1.0" encoding="UTF-8"?>\n' + raw


# --------------------------------------------------------------------------- #
# PDF (matplotlib) -- vector output for \includegraphics
# --------------------------------------------------------------------------- #
def to_pdf(d: Diagram, path: str, fmt: str = "pdf", dpi: int = 150) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.patches as mpatches
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

    scale = 100.0                      # drawio units per inch
    fig, ax = plt.subplots(figsize=(d.w / scale, d.h / scale))
    ax.set_xlim(0, d.w)
    ax.set_ylim(d.h, 0)                # drawio's y grows downward
    ax.axis("off")
    ax.set_aspect("equal")

    for b in sorted(d.boxes, key=lambda x: x.z):
        fill, stroke, text = STYLE.get(b.kind, STYLE["node"])
        ax.add_patch(FancyBboxPatch(
            (b.x, b.y), b.w, b.h,
            boxstyle=f"round,pad=0,rounding_size={6 if b.rounded else 0}",
            linewidth=1.3, edgecolor=stroke, facecolor=fill,
            linestyle=(0, (5, 3)) if b.dashed else "solid", zorder=b.z))
        if b.kind in ("layer", "boundary"):
            ax.text(b.x + 10, b.y + 15, b.label, ha="left", va="center",
                    fontsize=8.5, color=text, fontweight="bold", zorder=b.z + 1)
            if b.sub:
                ax.text(b.x + b.w - 10, b.y + 15, b.sub, ha="right", va="center",
                        fontsize=7.5, color=text, alpha=0.8, zorder=b.z + 1)
        else:
            cy = b.y + b.h / 2 - (5 if b.sub else 0)
            ax.text(b.x + b.w / 2, cy, b.label, ha="center", va="center",
                    fontsize=8.2, color=text, zorder=b.z + 1, wrap=True)
            if b.sub:
                ax.text(b.x + b.w / 2, cy + 11, b.sub, ha="center", va="center",
                        fontsize=6.8, color=text, alpha=0.75, zorder=b.z + 1)

    for e in d.edges:
        a, bb = d.find(e.src), d.find(e.dst)
        p0, p1 = _anchor(a, bb, e.route)
        colour = "#3b82f6" if e.style == "thick" else "#7a8899"
        lw = 2.0 if e.style == "thick" else 1.1
        ax.add_patch(FancyArrowPatch(
            p0, p1, arrowstyle="-|>" if not e.bidir else "<|-|>",
            mutation_scale=9, linewidth=lw, color=colour,
            linestyle=(0, (5, 3)) if e.style == "dashed" else "solid",
            connectionstyle="arc3,rad=0", shrinkA=0, shrinkB=0, zorder=50))
        if e.label:
            lx = p0[0] + (p1[0] - p0[0]) * e.lpos
            ly = p0[1] + (p1[1] - p0[1]) * e.lpos
            ax.text(lx, ly, e.label, ha="center", va="center", fontsize=6.4,
                    color="#4a5568", zorder=51,
                    bbox=dict(boxstyle="round,pad=0.18", fc="white",
                              ec="none", alpha=0.92))

    if d.caption:
        ax.text(d.w / 2, d.h - 6, d.caption, ha="center", va="bottom",
                fontsize=7, color="#6b7280", style="italic")

    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, format=fmt, bbox_inches="tight", pad_inches=0.04,
                dpi=dpi, facecolor="white")
    plt.close(fig)


def _anchor(a: Box, b: Box, route: str) -> tuple[tuple, tuple]:
    """Pick the pair of box-edge midpoints that face each other."""
    ac = (a.x + a.w / 2, a.y + a.h / 2)
    bc = (b.x + b.w / 2, b.y + b.h / 2)
    dx, dy = bc[0] - ac[0], bc[1] - ac[1]
    vertical = abs(dy) >= abs(dx) if route == "auto" else route == "v"
    if vertical:
        if dy >= 0:
            return (ac[0], a.y + a.h), (bc[0], b.y)
        return (ac[0], a.y), (bc[0], b.y + b.h)
    if dx >= 0:
        return (a.x + a.w, ac[1]), (b.x, bc[1])
    return (a.x, ac[1]), (b.x + b.w, bc[1])


# --------------------------------------------------------------------------- #
def render(diagrams: list[Diagram], root: str) -> None:
    """Write each diagram three ways.

    .drawio  the editable source, opened in Draw.io / diagrams.net
    .pdf     vector, included by the LaTeX document
    .png     raster, because GitHub will not render a PDF inline in a README
    """
    dio = os.path.join(root, "docs", "diagrams")
    pdf = os.path.join(root, "docs", "figures")
    png = os.path.join(root, "docs", "images")
    for p in (dio, pdf, png):
        os.makedirs(p, exist_ok=True)
    print(f"{'diagram':28s} {'boxes':>6s} {'edges':>6s}   drawio  pdf  png")
    print("-" * 66)
    for d in diagrams:
        with open(os.path.join(dio, f"{d.name}.drawio"), "w") as f:
            f.write(to_drawio(d))
        to_pdf(d, os.path.join(pdf, f"{d.name}.pdf"))
        to_pdf(d, os.path.join(png, f"{d.name}.png"), fmt="png", dpi=140)
        print(f"{d.name:28s} {len(d.boxes):6d} {len(d.edges):6d}      ok    ok   ok")
