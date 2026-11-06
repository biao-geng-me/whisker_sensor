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

``paint_grid``: draws a grid into a rect (the picker, ``SensorGrid``, and the
waveform's legend, ``GridLegend``): each sensor a box with its name over its A
and B halves; a lit gauge is filled with its colour, an unlit one light grey, a
dead or absent one dimmed (hatched).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import numpy as np
from PyQt6.QtCore import QPointF, QRectF, QSize, Qt, pyqtSignal
from PyQt6.QtGui import QBrush, QColor, QFont, QPainter, QPen
from PyQt6.QtWidgets import QSizePolicy, QToolTip, QWidget

import pyqtgraph as pg

ASSET_DIR = Path(__file__).resolve().parents[1] / "assets" / "dome"
N_HUES = 16
_HUE_STEP = 7                      # sensor s gets hue slot (s * 7) % 16: neighbours far apart
_UNLIT = QColor(228, 228, 228)
_DIM = QColor(244, 244, 244)
_EDGE = QColor(150, 150, 150)
_TEXT = QColor(50, 50, 50)


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
        cells = {}
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
                   max(1, ncols), max(1, nrows), dead, nch)

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
               font_px: float = 9.0, show_ab: bool = True, gap: float = 3.0) -> dict:
    """Draw ``grid`` in ``rect`` (see the module doc); returns ``cell_rects``."""
    rects = cell_rects(grid, rect, gap)
    f = QFont(p.font())
    f.setPixelSize(max(6, int(round(font_px))))
    p.setFont(f)
    for s, (cell, name, a, b) in rects.items():
        for col, half in ((2 * s, a), (2 * s + 1, b)):
            off = grid.absent(col) or col in grid.dead
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
                p.drawText(half, Qt.AlignmentFlag.AlignCenter, "AB"[col % 2])
        if name.height() >= 6:
            dead = all(grid.absent(c) or c in grid.dead for c in (2 * s, 2 * s + 1))
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

    def __init__(self, title: str = "", color: Callable[[int], QColor] = gauge_color,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.title = title
        self.grid = GridLayout()
        self.checked: set[int] = set()
        self.color = color
        self.labels: dict[int, str] = {}                 # gauge col -> its full name (tooltips)
        self._rects: dict = {}
        self.setMouseTracking(True)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setMinimumHeight(60)

    def set_grid(self, grid: GridLayout) -> None:
        self.grid = grid
        self.setFixedHeight(self._title_h() + 28 * grid.nrows + 3 * (grid.nrows - 1) + 4)
        self.update()

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
        self._rects = paint_grid(p, self.grid, self._area(), GridState(lit), font_px=10.0)
        p.end()

    def hit(self, pos: QPointF) -> list[int]:
        """The gauge columns a click at ``pos`` toggles (absent ones left out)."""
        for s, (cell, name, a, b) in self._rects.items():
            if not cell.contains(pos):
                continue
            if name.contains(pos):
                cols = [2 * s, 2 * s + 1]
            else:
                cols = [2 * s] if a.contains(pos) else [2 * s + 1]
            return [c for c in cols if not self.grid.absent(c)]
        return []

    def mousePressEvent(self, ev) -> None:  # noqa: N802 — Qt naming
        if ev.button() != Qt.MouseButton.LeftButton:
            return super().mousePressEvent(ev)
        cols = self.hit(ev.position())
        if cols:
            on = not all(c in self.checked for c in cols)     # all on -> off, else all on
            self.toggled.emit([f"wsk:{c}" for c in cols], on)
        ev.accept()

    def mouseMoveEvent(self, ev) -> None:  # noqa: N802 — Qt naming
        cols = self.hit(ev.position())
        if not cols:
            self.setToolTip("")
            return
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
