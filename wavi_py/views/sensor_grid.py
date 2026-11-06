"""The whisker sensors as a small grid laid out like the Dome view's plane map,
and the gauges' fixed colours.

``GridLayout.of(dome_map, nsensor)``: one cell per sensor (whisker slot, gauges
A and B) at the plane map's place (``whisker_link.dome_pose.PlaneMap``: port
on the left, starboard on the right, each side as seen from inside facing it,
its front column toward the centre line; rows by height, up on top), with a
gap column between the sides. The dome map's positions place the sensors
(``hole_channels``); without a map, the placeholder holes of the Dome view
(``PLACEHOLDER_HOLE_OF``). Sensors without a position go on the row(s) below.
``dead``: the gauges the map lists as dead or not scanned (without a map: the
slots that are not wired, ``LIVE_WHISKERS``).

``gauge_color(col)``: one colour per gauge for the whole app, by its index, so a
gauge keeps it whatever else is shown: the sensor's hue (16 hues, spread so
that neighbouring slots differ), A darker, B lighter, at a moderate saturation
so none reads as a cursor's saturated red / green / azure / magenta / black.

The picker has two views of the same cells (``SensorGrid.positions``): by
gauge (the halves A and B, keys ``wsk:<col>``) and by position (the halves
drag and lift of the dome position at that place, keys
``pos:<position>:<axis>``, read through each recording's dome map: the
Analysis window's Positions and the Events windows' default). In the position
view half ``2 s`` is the drag of sensor ``s``'s position and ``2 s + 1`` its
lift, whichever gauge is wired to it; a sensor without a position has none.

The plane map (``SensorGrid.set_map``, the Analysis window's Plots grid in
the position view): the cells grow into small plots, the name strip on top
with the D and L halves as chips at its right, and under it each position's
tip path over the selections (``daq/plane_overlay.py``, ``set_overlay``): drag
toward the side's front (``GridLayout.fwd``: right on the port side, left on
starboard, as the Dome view's plane map), lift up, every cell on one scale (the
largest bend, the title's ``±x°``), arrows from the root to the tip at the
cursors (numbered) or at each path's peak. A click on the name or the plot
toggles both axes, on a chip that axis. Rows without a position stay compact.

``paint_grid``: draws a grid into a rect (the picker, ``SensorGrid``, and the
waveform's legend, ``GridLegend``): each sensor a box with its name over its A
and B halves; a lit gauge is filled with its colour, an unlit one light grey, a
dead or absent one dimmed (hatched).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import numpy as np
from PyQt6.QtCore import QPointF, QRectF, QSize, Qt, pyqtSignal
from PyQt6.QtGui import QBrush, QColor, QFont, QPainter, QPainterPath, QPen, QPolygonF
from PyQt6.QtWidgets import QSizePolicy, QToolTip, QWidget

import pyqtgraph as pg

ASSET_DIR = Path(__file__).resolve().parents[1] / "assets" / "dome"
N_HUES = 16
_HUE_STEP = 7                      # sensor s gets hue slot (s * 7) % 16: neighbours far apart
_UNLIT = QColor(228, 228, 228)
_DIM = QColor(244, 244, 244)
_EDGE = QColor(150, 150, 150)
_TEXT = QColor(50, 50, 50)
MAP_NAME_H = 14.0                  # the plane map: a cell's name strip ...
MAP_MIN_H, MAP_MAX_H = 52.0, 120.0  # ... and its height (the cell's width + the strip, clipped)


def gauge_color(col: int) -> QColor:
    """The gauge's fixed colour (see the module doc): column ``col`` (W1 A = 0)."""
    s, b = divmod(int(col), 2)
    h = (((s * _HUE_STEP) % N_HUES) + 0.5) / N_HUES
    light = 0.33 if not b else 0.58
    if 0.10 <= h <= 0.22:                  # yellows / olives wash out on white: darker
        light -= 0.08
    return QColor.fromHslF(h, 0.62, light)


@dataclass
class GridLayout:
    """Cells (sensor -> (col, row)) on a ``ncols`` x ``nrows`` grid; ``names``
    per sensor; ``dead`` gauge columns; ``nch`` gauges there are."""
    cells: dict = field(default_factory=dict)
    names: list = field(default_factory=list)
    ncols: int = 1
    nrows: int = 1
    dead: set = field(default_factory=set)
    nch: int = 0
    fwd: dict = field(default_factory=dict)      # sensor -> screen x of forward (+1 port, -1 stbd)

    @classmethod
    def of(cls, dome_map, nsensor: int, nch: Optional[int] = None,
           names: Optional[list] = None) -> "GridLayout":
        from whisker_link.dome_pose import LIVE_WHISKERS, PLACEHOLDER_HOLE_OF, DomeLayout, PlaneMap
        nch = 2 * nsensor if nch is None else nch
        asset = DomeLayout.load(ASSET_DIR / "whiskers.json")
        if dome_map is not None:
            lay = dome_map.layout(asset.center, asset.dome_radius)
            hole_sensor = {}
            for h, chans in enumerate(dome_map.hole_channels()):
                ok = [int(c) for c in chans if c >= 0]
                if ok:
                    hole_sensor[h] = ok[0] // 2
            dead = {int(c) for c, why in dome_map.unused.items()
                    if str(why).lower().startswith(("dead", "not scanned"))}
            short = [p.label for p in dome_map.positions]
            label = {hole_sensor[h]: short[h] for h in hole_sensor}
        else:
            lay = asset
            hole_sensor = {h: w - 1 for w, h in PLACEHOLDER_HOLE_OF.items()}
            dead = {c for s in range(nsensor) if s + 1 not in LIVE_WHISKERS for c in (2 * s, 2 * s + 1)}
            label = {}
        pm = PlaneMap.of(lay)
        xy = pm.xy
        holes = [h for h in hole_sensor if h < len(xy) and hole_sensor[h] < nsensor]
        cells, fwd = {}, {}
        if holes:
            xs = np.array([xy[h, 0] for h in holes])
            ys = np.array([xy[h, 1] for h in holes])
            tol = 0.5 * pm.spacing
            ux = _levels(xs, tol)
            uy = _levels(-ys, tol)                      # up on top
            port = [x for x in ux if x < 0]
            cx = {x: i + (1 if x > 0 and port else 0) for i, x in enumerate(ux)}
            for h, x, y in zip(holes, xs, ys):
                cells[hole_sensor[h]] = (cx[_near(ux, x)], _index(uy, -y))
                fwd[hole_sensor[h]] = float(pm.fwd[h])
            ncols = len(ux) + (1 if port and len(port) < len(ux) else 0)
            nrows = len(uy)
        else:
            ncols, nrows = 4, 0
        rest = [s for s in range(nsensor) if s not in cells]
        if rest:                                         # no position: a row below
            per = max(ncols, 1)
            r0 = nrows
            for i, s in enumerate(rest):
                cells[s] = (i % per, r0 + i // per)
            nrows = r0 + (len(rest) + per - 1) // per
        nm = list(names) if names else [f"W{s + 1}" for s in range(nsensor)]
        short_names = [label.get(s, f"W{s + 1}") for s in range(nsensor)]
        return cls(cells, [short_names[s] if s < len(short_names) else nm[s] for s in range(nsensor)],
                   max(1, ncols), max(1, nrows), dead, nch, fwd)

    def absent(self, col: int) -> bool:
        return col >= self.nch


def _levels(v: np.ndarray, tol: float) -> list[float]:
    out: list[float] = []
    for x in np.sort(v):
        if not out or x - out[-1] > tol:
            out.append(float(x))
    return out


def _near(levels: list, x: float) -> float:
    return min(levels, key=lambda u: abs(u - x))


def _index(levels: list, x: float) -> int:
    return levels.index(_near(levels, x))


# ----------------------------------------------------------------------
# painting
# ----------------------------------------------------------------------

@dataclass
class GridState:
    """What a painted grid shows: ``lit`` {gauge col: QColor} (absent: unlit)."""
    lit: dict = field(default_factory=dict)


def cell_rects(grid: GridLayout, rect: QRectF, gap: float = 3.0):
    """{sensor: (cell rect, name rect, A rect, B rect)} fitting ``rect``."""
    cw = (rect.width() - gap * (grid.ncols - 1)) / grid.ncols
    ch = (rect.height() - gap * (grid.nrows - 1)) / grid.nrows
    out = {}
    for s, (c, r) in grid.cells.items():
        x, y = rect.left() + c * (cw + gap), rect.top() + r * (ch + gap)
        cell = QRectF(x, y, cw, ch)
        nh = min(max(ch * 0.42, 0.0), 14.0)
        name = QRectF(x, y, cw, nh)
        a = QRectF(x, y + nh, cw / 2, ch - nh)
        b = QRectF(x + cw / 2, y + nh, cw / 2, ch - nh)
        out[s] = (cell, name, a, b)
    return out


def paint_grid(p: QPainter, grid: GridLayout, rect: QRectF, state: GridState,
               font_px: float = 9.0, show_ab: bool = True, gap: float = 3.0,
               halves: str = "AB", off: Optional[Callable[[int], bool]] = None) -> dict:
    """Draw ``grid`` in ``rect`` (see the module doc); returns ``cell_rects``.
    ``halves``: the halves' letters; ``off(col)``: a half drawn dimmed (default:
    an absent or dead gauge)."""
    off_of = off or (lambda c: grid.absent(c) or c in grid.dead)
    rects = cell_rects(grid, rect, gap)
    f = QFont(p.font())
    f.setPixelSize(max(6, int(round(font_px))))
    p.setFont(f)
    for s, (cell, name, a, b) in rects.items():
        for col, half in ((2 * s, a), (2 * s + 1, b)):
            off = off_of(col)
            c = state.lit.get(col)
            if c is not None:
                p.setBrush(QBrush(c))
            elif off:
                p.setBrush(QBrush(_DIM, Qt.BrushStyle.BDiagPattern if half.height() > 6
                                  else Qt.BrushStyle.SolidPattern))
            else:
                p.setBrush(QBrush(_UNLIT))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawRect(half)
            if off and c is None:
                p.setBrush(QBrush(QColor(170, 170, 170), Qt.BrushStyle.BDiagPattern))
                p.drawRect(half)
            if show_ab and half.height() >= 10 and half.width() >= 10:
                lit_dark = c is not None and c.lightnessF() < 0.5
                p.setPen(QColor(255, 255, 255) if lit_dark else
                         QColor(160, 160, 160) if off else _TEXT)
                p.drawText(half, Qt.AlignmentFlag.AlignCenter, halves[col % 2])
        if name.height() >= 6:
            dead = all(off_of(c) for c in (2 * s, 2 * s + 1))
            p.setPen(QColor(165, 165, 165) if dead else _TEXT)
            p.drawText(name, Qt.AlignmentFlag.AlignCenter, grid.names[s] if s < len(grid.names)
                       else f"W{s + 1}")
        p.setPen(QPen(_EDGE, 1))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRect(cell)
        p.drawLine(QPointF(a.right(), a.top()), QPointF(a.right(), a.bottom()))
    return rects


class SensorGrid(QWidget):
    """The picker (see the module doc): ``toggled(keys, on)`` with the gauge keys
    (``wsk:<col>``) a click toggles: the name strip both, a half that gauge.
    ``checked`` is the set of checked columns; ``color`` gives a lit gauge's
    colour."""

    toggled = pyqtSignal(list, bool)
    AXES = ("drag", "lift")

    def __init__(self, title: str = "", color: Callable[[int], QColor] = gauge_color,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.title = title
        self.grid = GridLayout()
        self.checked: set[int] = set()
        self.color = color
        self.labels: dict[int, str] = {}                 # gauge col -> its full name (tooltips)
        self.positions = False                           # the by-position view (module doc)
        self.pos_off: set[int] = set()                   # position view: halves dimmed
        self.map = False                                 # the plane map (module doc)
        self.overlay = None                              # daq/plane_overlay.PlaneOverlay
        self.path_pen: Callable = lambda path: QPen(QColor(60, 60, 60), 1.2)
        self._rects: dict = {}
        self.setMouseTracking(True)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setMinimumHeight(60)

    def set_grid(self, grid: GridLayout) -> None:
        self.grid = grid
        self._fit_height()
        self.update()

    def set_map(self, on: bool) -> None:
        """The plane map (module doc) or the compact picker."""
        on = bool(on)
        if on != self.map:
            self.map = on
            self._fit_height()
            self.update()

    def set_overlay(self, overlay, path_pen: Optional[Callable] = None) -> None:
        """The paths and arrows (``PlaneOverlay``) and the pen of each path."""
        self.overlay = overlay
        if path_pen is not None:
            self.path_pen = path_pen
        self.update()

    def _map_rows(self) -> set:
        """The rows drawn as plots in the plane map: those with a position."""
        return {r for s, (_c, r) in self.grid.cells.items() if self.position_of(2 * s)}

    def _row_heights(self, width: Optional[float] = None) -> list[float]:
        if not (self.map and self.positions):
            return [28.0] * self.grid.nrows
        w = (self.width() if width is None else width) - 4
        cw = (w - 3 * (self.grid.ncols - 1)) / max(1, self.grid.ncols)
        tall = max(MAP_MIN_H, min(MAP_MAX_H, cw + MAP_NAME_H))
        rows = self._map_rows()
        return [tall if r in rows else 22.0 for r in range(self.grid.nrows)]

    def _fit_height(self) -> None:
        hs = self._row_heights()
        self.setFixedHeight(int(self._title_h() + sum(hs) + 3 * (len(hs) - 1) + 4))

    def resizeEvent(self, ev) -> None:  # noqa: N802 — Qt naming
        super().resizeEvent(ev)
        if self.map and ev.size().width() != ev.oldSize().width():
            self._fit_height()

    def set_checked(self, cols) -> None:
        cols = set(cols)
        if cols != self.checked:
            self.checked = cols
            self.update()

    def _title_h(self) -> int:
        return 16 if self.title else 0

    def sizeHint(self) -> QSize:  # noqa: N802 — Qt naming
        return QSize(260, self.height())

    def _area(self) -> QRectF:
        return QRectF(2, self._title_h() + 1, self.width() - 4, self.height() - self._title_h() - 3)

    def paintEvent(self, ev) -> None:  # noqa: N802 — Qt naming
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)
        if self.title:
            f = QFont(p.font())
            f.setBold(True)
            p.setFont(f)
            p.setPen(_TEXT)
            p.drawText(QRectF(2, 0, self.width(), 15), Qt.AlignmentFlag.AlignLeft
                       | Qt.AlignmentFlag.AlignVCenter, self.title)
        lit = {c: self.color(c) for c in self.checked}
        if self.map and self.positions:
            self._paint_map(p, lit)
        elif self.positions:
            self._rects = paint_grid(p, self.grid, self._area(), GridState(lit), font_px=10.0,
                                     halves="DL", off=lambda c: c in self.pos_off)
        else:
            self._rects = paint_grid(p, self.grid, self._area(), GridState(lit), font_px=10.0)
        p.end()

    # ----- the plane map

    def map_rects(self) -> dict:
        """{sensor: (cell, name strip, D chip, L chip, plot)} of the plane map."""
        area = self._area()
        hs = self._row_heights()
        cw = (area.width() - 3 * (self.grid.ncols - 1)) / max(1, self.grid.ncols)
        tops = [area.top()]
        for h in hs[:-1]:
            tops.append(tops[-1] + h + 3)
        out = {}
        for s, (c, r) in self.grid.cells.items():
            x, y, h = area.left() + c * (cw + 3), tops[r], hs[r]
            cell = QRectF(x, y, cw, h)
            nh = min(MAP_NAME_H, h)
            chip = min(MAP_NAME_H, cw / 4)
            name = QRectF(x, y, cw - 2 * chip, nh)
            d = QRectF(x + cw - 2 * chip, y, chip, nh)
            lchip = QRectF(x + cw - chip, y, chip, nh)
            out[s] = (cell, name, d, lchip, QRectF(x, y + nh, cw, h - nh))
        return out

    def _paint_map(self, p: QPainter, lit: dict) -> None:
        rects = self.map_rects()
        self._rects = {s: (cell, name, d, l) for s, (cell, name, d, l, _pl) in rects.items()}
        ov = self.overlay
        f = QFont(p.font())
        f.setPixelSize(10)
        p.setFont(f)
        if ov is not None and ov.scale > 0 and self.title:
            p.setPen(QColor(90, 90, 90))
            p.drawText(QRectF(2, 0, self.width() - 4, 15), Qt.AlignmentFlag.AlignRight
                       | Qt.AlignmentFlag.AlignVCenter, f"plane map  ±{ov.scale:.3g}°")
        for s, (cell, name, d, lchip, plot) in rects.items():
            pos = self.position_of(2 * s)
            for col, chip, letter in ((2 * s, d, "D"), (2 * s + 1, lchip, "L")):
                c = lit.get(col)
                off = col in self.pos_off or pos is None
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(QBrush(c) if c is not None else QBrush(_DIM if off else _UNLIT))
                p.drawRect(chip)
                p.setPen(QColor(255, 255, 255) if c is not None and c.lightnessF() < 0.5
                         else QColor(170, 170, 170) if off else _TEXT)
                p.drawText(chip, Qt.AlignmentFlag.AlignCenter, letter)
            p.setPen(_TEXT if pos else QColor(165, 165, 165))
            p.drawText(name.adjusted(3, 0, 0, 0), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                       self.grid.names[s] if s < len(self.grid.names) else f"W{s + 1}")
            if plot.height() > 8 and pos:
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(QBrush(QColor(252, 252, 252)))
                p.drawRect(plot)
                self._paint_cell(p, s, pos, plot)
            p.setPen(QPen(_EDGE, 1))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawRect(cell)
            p.drawLine(name.bottomLeft(), QPointF(cell.right(), name.bottom()))

    def _paint_cell(self, p: QPainter, s: int, pos: str, plot: QRectF) -> None:
        cx, cy = plot.center().x(), plot.center().y()
        half = 0.44 * min(plot.width(), plot.height())
        p.setPen(QPen(QColor(222, 222, 222), 1))
        p.drawLine(QPointF(plot.left() + 2, cy), QPointF(plot.right() - 2, cy))
        p.drawLine(QPointF(cx, plot.top() + 2), QPointF(cx, plot.bottom() - 2))
        fwd = self.grid.fwd.get(s, 1.0)
        # which way is forward: a small tick on the front edge
        p.drawLine(QPointF(cx + fwd * half, cy - 3), QPointF(cx + fwd * half, cy + 3))
        ov = self.overlay
        if ov is None or ov.scale <= 0:
            return
        k = half / ov.scale

        def at(xy) -> QPointF:
            return QPointF(cx + fwd * k * float(xy[0]), cy - k * float(xy[1]))
        p.save()
        p.setClipRect(plot)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        p.setBrush(Qt.BrushStyle.NoBrush)
        for path in ov.paths.get(pos, []):
            pen = QPen(self.path_pen(path))
            col = QColor(pen.color())
            col.setAlpha(150)
            pen.setColor(col)
            p.setPen(pen)
            pts = path.xy
            if len(pts) > 1:
                p.drawPolyline(QPolygonF([at(q) for q in pts]))
        o = QPointF(cx, cy)
        for ar in ov.arrows.get(pos, []):
            pen = QPen(self.path_pen(ar))
            pen.setStyle(Qt.PenStyle.SolidLine)
            pen.setWidthF(2.0)
            p.setPen(pen)
            tip = at(ar.xy)
            p.drawLine(o, tip)
            dx, dy = tip.x() - cx, tip.y() - cy
            r = math.hypot(dx, dy)
            if r > 4:
                ux, uy = dx / r, dy / r
                head = QPainterPath(tip)
                head.lineTo(tip.x() - 5 * ux + 3 * uy, tip.y() - 5 * uy - 3 * ux)
                head.lineTo(tip.x() - 5 * ux - 3 * uy, tip.y() - 5 * uy + 3 * ux)
                head.closeSubpath()
                p.fillPath(head, pen.color())
            if ar.label:
                p.drawText(QPointF(tip.x() + 3, tip.y() - 3), ar.label)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(QColor(90, 90, 90)))
        p.drawEllipse(o, 2.0, 2.0)
        p.restore()

    def hit(self, pos: QPointF) -> list[int]:
        """The gauge columns a click at ``pos`` toggles (absent ones left out)."""
        if self.map and self.positions:
            for s, (cell, _name, d, lchip, _plot) in self.map_rects().items():
                if cell.contains(pos):
                    cols = [2 * s] if d.contains(pos) else [2 * s + 1] if lchip.contains(pos) \
                        else [2 * s, 2 * s + 1]
                    return [c for c in cols if self.position_of(c) is not None]
            return []
        for s, (cell, name, a, b) in self._rects.items():
            if not cell.contains(pos):
                continue
            if name.contains(pos):
                cols = [2 * s, 2 * s + 1]
            else:
                cols = [2 * s] if a.contains(pos) else [2 * s + 1]
            if self.positions:
                return [c for c in cols if self.position_of(c) is not None]
            return [c for c in cols if not self.grid.absent(c)]
        return []

    def position_of(self, col: int) -> Optional[str]:
        """The position of half ``col`` in the position view (its sensor's place on
        the dome), None where the sensor has none."""
        s = col // 2
        name = self.grid.names[s] if s < len(self.grid.names) else ""
        return name if len(name) == 3 and name[:2] in ("LB", "LF", "RF", "RB") \
            and name[2] in "UML" else None

    def key_of(self, col: int) -> Optional[str]:
        """The variable key of half ``col`` in the current view."""
        if not self.positions:
            return f"wsk:{col}"
        p = self.position_of(col)
        return f"pos:{p}:{self.AXES[col % 2]}" if p else None

    def col_of(self, key: str) -> Optional[int]:
        """The half showing variable ``key`` in the current view (None: not here)."""
        if not self.positions:
            return int(key[4:]) if key.startswith("wsk:") else None
        if not key.startswith("pos:"):
            return None
        _, p, ax = key.split(":")
        for s, name in enumerate(self.grid.names):
            if name == p and s in self.grid.cells:
                return 2 * s + self.AXES.index(ax)
        return None

    def mousePressEvent(self, ev) -> None:  # noqa: N802 — Qt naming
        if ev.button() != Qt.MouseButton.LeftButton:
            return super().mousePressEvent(ev)
        cols = self.hit(ev.position())
        if cols:
            on = not all(c in self.checked for c in cols)     # all on -> off, else all on
            self.toggled.emit([self.key_of(c) for c in cols], on)
        ev.accept()

    def mouseMoveEvent(self, ev) -> None:  # noqa: N802 — Qt naming
        cols = self.hit(ev.position())
        if not cols:
            self.setToolTip("")
            return
        if self.positions:
            names = [f"{self.position_of(c)} {self.AXES[c % 2]}" for c in cols]
            dead = [c for c in cols if c in self.pos_off]
        else:
            names = [self.labels.get(c, f"W{c // 2 + 1} {'AB'[c % 2]}") for c in cols]
            dead = [c for c in cols if c in self.grid.dead]
        tip = " + ".join(names) + (" (dead / not scanned)" if dead else "")
        self.setToolTip(tip)
        QToolTip.showText(ev.globalPosition().toPoint(), tip, self)


class GridLegend(pg.GraphicsWidget, pg.GraphicsWidgetAnchor):
    """The waveform row's legend for its gauges: the grid in miniature, the
    plotted gauges lit in their trace colour (``set_lit``), anchored at the
    plot's top right as the text legend is."""

    CELL_W, CELL_H = 26.0, 19.0

    def __init__(self, grid: GridLayout):
        pg.GraphicsWidget.__init__(self)
        pg.GraphicsWidgetAnchor.__init__(self)
        self.grid = grid
        self.lit: dict = {}
        w = grid.ncols * self.CELL_W + 3 * (grid.ncols - 1) + 8
        h = grid.nrows * self.CELL_H + 3 * (grid.nrows - 1) + 8
        self.setGeometry(QRectF(0, 0, w, h))
        self.setZValue(40)

    def set_lit(self, lit: dict) -> None:
        if {k: c.rgba() for k, c in lit.items()} != {k: c.rgba() for k, c in self.lit.items()}:
            self.lit = dict(lit)
            self.update()

    def boundingRect(self) -> QRectF:  # noqa: N802 — Qt naming
        return QRectF(0, 0, self.width(), self.height())

    def paint(self, p: QPainter, *args) -> None:
        r = self.boundingRect()
        p.setPen(QPen(QColor(180, 180, 180), 1))
        p.setBrush(QBrush(QColor(255, 255, 255, 215)))
        p.drawRect(r.adjusted(0, 0, -1, -1))
        paint_grid(p, self.grid, r.adjusted(4, 4, -4, -4), GridState(self.lit), font_px=8.0,
                   show_ab=False, gap=3.0)
