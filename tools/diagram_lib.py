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

    def containers(self) -> set[str]:
        """Ids of boxes that enclose other boxes.

        A container's title belongs at the top, like a banner: centred, it is
        drawn straight over its own children. Detected rather than declared, so
        any diagram that groups boxes gets this for free.
        """
        solid = [b for b in self.boxes if b.kind not in ("layer", "boundary")]
        out = set()
        for a in solid:
            for c in solid:
                if c is not a and c.x >= a.x - 2 and c.y >= a.y - 2 \
                        and c.x + c.w <= a.x + a.w + 2 and c.y + c.h <= a.y + a.h + 2:
                    out.add(a.id)
                    break
        return out

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
    is_container = d.containers()

    for b in sorted(d.boxes, key=lambda x: x.z):
        fill, stroke, text = STYLE.get(b.kind, STYLE["node"])
        style = (f"rounded={1 if b.rounded else 0};whiteSpace=wrap;html=1;"
                 f"fillColor={fill};strokeColor={stroke};fontColor={text};"
                 f"fontFamily={FONT};fontSize=12;arcSize=8;verticalAlign=middle;"
                 f"align=center;spacing=4;")
        if b.dashed:
            style += "dashed=1;dashPattern=6 4;"
        if b.kind in ("layer", "boundary") or b.id in is_container:
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
# PDF / PNG (matplotlib) -- vector output for \includegraphics
# --------------------------------------------------------------------------- #
# Diagram units are 1/100 inch, so 1 unit = 0.72 pt.
PT_PER_UNIT = 0.72
#: Helvetica-ish average advance width as a fraction of the font size. Used to
#: fit text to a box without needing a renderer to measure it.
CHAR_W = 0.55
FONT_MAX, FONT_MIN = 8.2, 5.4
SUB_RATIO = 0.82          # sub-label size relative to the fitted label size


def _wrap(text: str, width_units: float, size: float) -> list[str]:
    """Split on explicit newlines, then soft-wrap each piece to the box width."""
    budget = max(int(width_units * PT_PER_UNIT / (size * CHAR_W)), 6)
    out: list[str] = []
    for para in str(text).split("\n"):
        words, line = para.split(), ""
        if not words:
            out.append("")
            continue
        for w in words:
            trial = f"{line} {w}".strip()
            if len(trial) <= budget or not line:
                line = trial
            else:
                out.append(line)
                line = w
        out.append(line)
    return out


def _fit(text: str, width_units: float, start: float = FONT_MAX) -> tuple[float, list[str]]:
    """Largest font size at which `text` fits the box on at most a few lines."""
    size = start
    while size > FONT_MIN:
        lines = _wrap(text, width_units - 12, size)
        longest = max((len(l) for l in lines), default=0)
        if longest * size * CHAR_W <= (width_units - 12) * PT_PER_UNIT:
            return size, lines
        size -= 0.3
    return FONT_MIN, _wrap(text, width_units - 12, FONT_MIN)


def to_pdf(d: Diagram, path: str, fmt: str = "pdf", dpi: int = 150) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, PathPatch
    from matplotlib.path import Path as MplPath

    scale = 100.0
    fig, ax = plt.subplots(figsize=(d.w / scale, d.h / scale))
    ax.set_xlim(0, d.w)
    ax.set_ylim(d.h, 0)                # drawio's y grows downward
    ax.axis("off")
    ax.set_aspect("equal")

    is_container = d.containers()

    # ---- boxes -----------------------------------------------------------
    for b in sorted(d.boxes, key=lambda x: x.z):
        fill, stroke, text = STYLE.get(b.kind, STYLE["node"])
        ax.add_patch(FancyBboxPatch(
            (b.x, b.y), b.w, b.h,
            boxstyle=f"round,pad=0,rounding_size={6 if b.rounded else 0}",
            linewidth=1.3, edgecolor=stroke, facecolor=fill,
            linestyle=(0, (5, 3)) if b.dashed else "solid", zorder=b.z))

        if b.kind in ("layer", "boundary") or b.id in is_container:
            # banner: title left, note right, on one line at the top
            ax.text(b.x + 10, b.y + 15, b.label, ha="left", va="center",
                    fontsize=8.5, color=text, fontweight="bold", zorder=b.z + 1)
            if b.sub:
                ax.text(b.x + b.w - 10, b.y + 15, b.sub, ha="right", va="center",
                        fontsize=7.4, color=text, alpha=0.8, zorder=b.z + 1)
            continue

        # Stack the label and the sub as one block, vertically centred. Doing
        # this by hand rather than with two fixed offsets is what stops a
        # two-line sub from being drawn over its own label.
        lsize, llines = _fit(b.label, b.w)
        ssize, slines = (0.0, [])
        if b.sub:
            ssize, slines = _fit(b.sub, b.w, start=lsize * SUB_RATIO)
        lead_l = lsize * 1.25 / PT_PER_UNIT          # line height in units
        lead_s = ssize * 1.30 / PT_PER_UNIT if slines else 0.0
        gap = 2.0 if slines else 0.0
        total = len(llines) * lead_l + gap + len(slines) * lead_s
        y = b.y + b.h / 2 - total / 2 + lead_l / 2
        cx = b.x + b.w / 2
        for line in llines:
            ax.text(cx, y, line, ha="center", va="center", fontsize=lsize,
                    color=text, zorder=b.z + 1)
            y += lead_l
        y += gap - lead_l / 2 + lead_s / 2
        for line in slines:
            ax.text(cx, y, line, ha="center", va="center", fontsize=ssize,
                    color=text, alpha=0.78, zorder=b.z + 1)
            y += lead_s

    # ---- edges -----------------------------------------------------------
    obstacles = [b for b in d.boxes if b.kind not in ("layer", "boundary")]
    for e in d.edges:
        a, bb = d.find(e.src), d.find(e.dst)
        pts = _route(a, bb, e.route, obstacles, (d.w, d.h))
        colour = "#3b82f6" if e.style == "thick" else "#7a8899"
        lw = 1.9 if e.style == "thick" else 1.05
        dash = (0, (5, 3)) if e.style == "dashed" else "solid"

        if len(pts) > 2:
            ax.add_patch(PathPatch(
                MplPath(pts[:-1], [MplPath.MOVETO] + [MplPath.LINETO] * (len(pts) - 2)),
                fill=False, edgecolor=colour, linewidth=lw, linestyle=dash,
                joinstyle="round", capstyle="round", zorder=50))
        ax.add_patch(FancyArrowPatch(
            pts[-2], pts[-1], arrowstyle="-|>" if not e.bidir else "<|-|>",
            mutation_scale=8.5, linewidth=lw, color=colour, linestyle=dash,
            shrinkA=0, shrinkB=0, zorder=50))

        if e.label:
            lx, ly = _label_point(pts, e.lpos, d.boxes, e.label)
            ax.text(lx, ly, e.label, ha="center", va="center", fontsize=6.2,
                    color="#4a5568", zorder=52,
                    bbox=dict(boxstyle="round,pad=0.20", fc="white",
                              ec="#e2e6ec", lw=0.4, alpha=0.96))

    if d.caption:
        ax.text(d.w / 2, d.h - 4, d.caption, ha="center", va="bottom",
                fontsize=6.8, color="#6b7280", style="italic")

    os.makedirs(os.path.dirname(path), exist_ok=True)
    # Suppress the embedded creation date. Without it every regeneration
    # rewrites all ten PDFs with identical content but a new timestamp, which
    # makes `git status` useless for spotting a real change.
    meta = {"CreationDate": None} if fmt == "pdf" else None
    fig.savefig(path, format=fmt, bbox_inches="tight", pad_inches=0.05,
                dpi=dpi, facecolor="white", metadata=meta)
    plt.close(fig)


def _label_point(pts, lpos: float, boxes=(), text: str = "") -> tuple[float, float]:
    """Pick a point on the polyline that is clear of the boxes.

    Sampling beats "midpoint of the longest segment": on these figures the
    midpoint very often lands on top of a box the edge merely passes, and a
    label drawn over a box title is worse than no label.
    """
    half_w = max(len(text), 1) * 6.2 * CHAR_W / PT_PER_UNIT / 2 + 3
    half_h = 6.2 * 1.4 / PT_PER_UNIT / 2 + 2

    samples: list[tuple[float, float, float]] = []
    total = sum(abs(q[0] - p[0]) + abs(q[1] - p[1]) for p, q in zip(pts, pts[1:])) or 1.0
    walked = 0.0
    for p, q in zip(pts, pts[1:]):
        seg = abs(q[0] - p[0]) + abs(q[1] - p[1])
        if seg < 1e-6:
            continue
        steps = max(int(seg / 6), 1)
        # unit normal to this segment, so a crowded line can push its label aside
        nx, ny = -(q[1] - p[1]) / seg, (q[0] - p[0]) / seg
        for i in range(steps + 1):
            t = i / steps
            x, y = p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t
            frac = (walked + seg * t) / total
            for off in (0.0, 9.0, -9.0, 16.0, -16.0):
                samples.append((x + nx * off, y + ny * off, frac + abs(off) * 0.002))
        walked += seg
    if not samples:
        return pts[0]

    def penalty(x, y, frac):
        worst = 0.0
        for bx in boxes:
            # a banner box is only "occupied" where its title is drawn
            top = bx.y + (30 if bx.kind in ("layer", "boundary") else bx.h)
            if (x + half_w > bx.x and x - half_w < bx.x + bx.w
                    and y + half_h > bx.y and y - half_h < top):
                worst += 100.0
        return worst + abs(frac - lpos) * 4.0

    x, y, _ = min(samples, key=lambda s: penalty(*s))
    return x, y


def _crosses(p, q, box: Box, margin: float = 3.0) -> bool:
    """Does the axis-aligned segment p->q pass through `box`?"""
    x0, x1 = sorted((p[0], q[0]))
    y0, y1 = sorted((p[1], q[1]))
    return not (x1 < box.x - margin or x0 > box.x + box.w + margin
                or y1 < box.y - margin or y0 > box.y + box.h + margin)


def _cost(pts, obstacles, a: Box, b: Box) -> float:
    """Penalise routes that run through boxes, then prefer shorter ones."""
    hits = 0
    for p, q in zip(pts, pts[1:]):
        for o in obstacles:
            if o is a or o is b:
                continue
            if _crosses(p, q, o):
                hits += 1
    length = sum(abs(q[0] - p[0]) + abs(q[1] - p[1]) for p, q in zip(pts, pts[1:]))
    bends = max(len(pts) - 2, 0)
    # A crossing is far worse than a detour, and a detour is slightly worse
    # than a bend; that ordering is what keeps the figures readable.
    return hits * 4000.0 + length + bends * 12.0


def _route(a: Box, b: Box, route: str, obstacles=(),
           page: tuple[float, float] | None = None) -> list[tuple[float, float]]:
    """Orthogonal polyline from `a` to `b`.

    Straight diagonal connectors are what made the first version of these
    figures unreadable: a line from one corner of the page to another passes
    through everything in between. This builds Manhattan routes instead --
    the same style the .drawio files use -- and picks whichever candidate
    crosses the fewest other boxes.
    """
    acx, acy = a.x + a.w / 2, a.y + a.h / 2
    bcx, bcy = b.x + b.w / 2, b.y + b.h / 2
    cands: list[list[tuple[float, float]]] = []

    # --- straight, when the boxes already line up ---
    if abs(bcx - acx) < min(a.w, b.w) / 2:
        x = (acx + bcx) / 2
        if bcy >= acy:
            cands.append([(x, a.y + a.h), (x, b.y)])
        else:
            cands.append([(x, a.y), (x, b.y + b.h)])
    if abs(bcy - acy) < min(a.h, b.h) / 2:
        y = (acy + bcy) / 2
        if bcx >= acx:
            cands.append([(a.x + a.w, y), (b.x, y)])
        else:
            cands.append([(a.x, y), (b.x + b.w, y)])

    # --- vertical first: leave top/bottom, cross on a horizontal channel ---
    # Several channel positions, not just the midpoint: with one candidate the
    # router has no way out when the midpoint happens to sit on a box, and the
    # line is drawn straight through it.
    if bcy >= acy:
        ay, by = a.y + a.h, b.y
    else:
        ay, by = a.y, b.y + b.h
    for t in (0.5, 0.25, 0.75, 0.12, 0.88):
        mid = ay + (by - ay) * t
        cands.append([(acx, ay), (acx, mid), (bcx, mid), (bcx, by)])

    # --- horizontal first: leave left/right, cross on a vertical channel ---
    if bcx >= acx:
        ax, bx = a.x + a.w, b.x
    else:
        ax, bx = a.x, b.x + b.w
    for t in (0.5, 0.25, 0.75, 0.12, 0.88):
        midx = ax + (bx - ax) * t
        cands.append([(ax, acy), (midx, acy), (midx, bcy), (bx, bcy)])

    # --- L shapes ---
    cands.append([(acx, ay), (acx, bcy), (bx, bcy)])
    cands.append([(ax, acy), (bcx, acy), (bcx, by)])

    # --- detours entirely outside both boxes ---
    # Needed when the two boxes share a row or a column: the channel "between"
    # them is then degenerate, every direct candidate runs through whatever
    # sits in the gap, and the router has no way out without going around.
    pw, ph = page or (max(a.x + a.w, b.x + b.w) + 200, max(a.y + a.h, b.y + b.h) + 200)
    below = min(max(a.y + a.h, b.y + b.h) + 38, ph - 12)
    above = max(min(a.y, b.y) - 38, 12)
    right = min(max(a.x + a.w, b.x + b.w) + 38, pw - 12)
    left = max(min(a.x, b.x) - 38, 12)
    cands.append([(acx, a.y + a.h), (acx, below), (bcx, below), (bcx, b.y + b.h)])
    cands.append([(acx, a.y), (acx, above), (bcx, above), (bcx, b.y)])
    cands.append([(a.x + a.w, acy), (right, acy), (right, bcy), (b.x + b.w, bcy)])
    cands.append([(a.x, acy), (left, acy), (left, bcy), (b.x, bcy)])

    # The route hint is a PREFERENCE, not a filter. Filtering meant that when a
    # box sat directly between the two endpoints, every surviving candidate had
    # to pass through it; a discount lets the router take the long way round
    # when the short way is blocked.
    def hinted(c) -> bool:
        if route == "v":
            return abs(c[0][0] - acx) < 1e-6
        if route == "h":
            return abs(c[0][1] - acy) < 1e-6
        return False

    best = min(cands, key=lambda c: _cost(c, obstacles, a, b) - (60.0 if hinted(c) else 0.0))
    return [p for i, p in enumerate(best) if i == 0 or p != best[i - 1]]


# --------------------------------------------------------------------------- #
def overlaps(d: Diagram) -> list[str]:
    """Boxes that intersect. A container (layer/boundary) is allowed to hold
    its children, and a note may sit inside a container; anything else
    overlapping is a layout mistake in the diagram source."""
    out = []
    solid = [b for b in d.boxes if b.kind not in ("layer", "boundary")]
    for i, a in enumerate(solid):
        for b in solid[i + 1:]:
            ox = min(a.x + a.w, b.x + b.w) - max(a.x, b.x)
            oy = min(a.y + a.h, b.y + b.h) - max(a.y, b.y)
            if ox <= 1 or oy <= 1:
                continue
            # full containment is deliberate nesting (a grouping box drawn
            # around its members), not a collision
            inside = (lambda p, q: p.x >= q.x - 2 and p.y >= q.y - 2
                      and p.x + p.w <= q.x + q.w + 2 and p.y + p.h <= q.y + q.h + 2)
            if inside(a, b) or inside(b, a):
                continue
            out.append(f"{a.id} <-> {b.id}  ({ox:.0f} x {oy:.0f} units)")
    return out


def out_of_bounds(d: Diagram) -> list[str]:
    bad = []
    for b in d.boxes:
        if b.x < 0 or b.y < 0 or b.x + b.w > d.w or b.y + b.h > d.h - 18:
            bad.append(f"{b.id} at ({b.x},{b.y},{b.w}x{b.h}) vs page {d.w}x{d.h}")
    return bad


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
    problems = 0
    for d in diagrams:
        with open(os.path.join(dio, f"{d.name}.drawio"), "w") as f:
            f.write(to_drawio(d))
        to_pdf(d, os.path.join(pdf, f"{d.name}.pdf"))
        to_pdf(d, os.path.join(png, f"{d.name}.png"), fmt="png", dpi=140)
        print(f"{d.name:28s} {len(d.boxes):6d} {len(d.edges):6d}      ok    ok   ok")
        for msg in overlaps(d):
            print(f"    OVERLAP  {msg}")
            problems += 1
        for msg in out_of_bounds(d):
            print(f"    BOUNDS   {msg}")
            problems += 1
    if problems:
        print(f"\n{problems} layout problem(s) -- fix the coordinates in "
              f"tools/make_diagrams.py")
