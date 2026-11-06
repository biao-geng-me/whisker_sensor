"""Hover and Probe readouts at the mouse for the Analysis window's line plots
(timeline, waveform, spectrum), which have no per-variable legends.

Hover (always): the trace nearest the mouse (within ``HOVER_PX``, the row
under it) is drawn again over itself, thicker (a halo curve: the trace's own
pen is left alone, so a redraw meanwhile does not fight it), and a small box
next to the mouse names it with the value at the mouse ("LFM drag #1 · t =
1.234 s · 0.52 mV"). In the spectrum (``peaks``) the hovered trace's highest
peaks (``N_PEAKS``, local maxima above 0 Hz at least ``PEAK_SEP`` bins apart)
get a dot and their frequency and amplitude (only those at least ``PEAK_REL`` of
the highest: the noise floor's ripples are not peaks).

Probe (P): the probe line at the mouse in every row of the cell, and in the row
under the mouse a dot where each trace crosses it (its nearest sample) with
its value next to it (labels pushed apart so they do not overlap), plus the x
at the top of the line. While a readout shows, the cell's legend boxes fade
(``LEGEND_FADE``): they sit above the data and would hide the labels.

A cell gives ``probe_items(plot)``: (x name, x unit, x from view, y from view,
[(label, unit, curve)]): the curves' data are in view coordinates (log10 on a
log axis), the two functions make them the data's.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor

HOVER_PX = 8                    # a trace this near the mouse is the hovered one
N_PEAKS = 3
PEAK_SEP = 3                    # bins between two marked peaks
PEAK_REL = 0.05                 # a marked peak is at least this fraction of the highest
LEGEND_FADE = 0.2               # the legends' opacity while a readout shows
MAX_PROBE = 16                  # probe labels per row at most
LABEL_GAP_PX = 15               # probe labels at least this far apart (vertically)
_FILL = pg.mkBrush(255, 255, 255, 245)
_BORDER = pg.mkPen(170, 170, 170)


def fmt(v: float) -> str:
    return f"{v:.4g}" if np.isfinite(v) else "–"


def _nearest_index(x: np.ndarray, at: float) -> int:
    i = int(np.clip(np.searchsorted(x, at), 0, len(x) - 1))
    if i > 0 and abs(x[i - 1] - at) < abs(x[i] - at):
        i -= 1
    return i


def peaks(x: np.ndarray, y: np.ndarray, n: int = N_PEAKS, sep: int = PEAK_SEP) -> list[int]:
    """Indices of the ``n`` highest local maxima of ``y`` (x > 0), ``sep`` apart, at
    least ``PEAK_REL`` of the highest (no noise ripples)."""
    y = np.asarray(y, dtype=float)
    if len(y) < 3:
        return []
    mid = y[1:-1]
    loc = np.flatnonzero((mid > y[:-2]) & (mid >= y[2:]) & np.isfinite(mid)) + 1
    loc = loc[np.asarray(x)[loc] > 0]
    out: list[int] = []
    if not len(loc):
        return out
    floor = PEAK_REL * float(np.nanmax(y[loc]))
    for i in loc[np.argsort(y[loc])[::-1]]:
        if y[i] < floor:
            break
        if all(abs(int(i) - j) >= sep for j in out):
            out.append(int(i))
            if len(out) == n:
                break
    return out


def nearest_curve(vb: pg.ViewBox, items: list, pt, px: float = HOVER_PX):
    """(item, index) of the curve nearest the view point ``pt`` within ``px`` pixels
    (a min / max trace covers its block's span), or None."""
    px_x, px_y = vb.viewPixelSize()
    best, best_d = None, (px + 1e-9, np.inf)
    for it in items:
        c = it[2]
        if not c.isVisible():
            continue
        x, y = c.getData()
        if x is None or not len(x):
            continue
        i0 = int(np.searchsorted(x, pt.x() - px * px_x))
        i1 = int(np.searchsorted(x, pt.x() + px * px_x, side="right"))
        i0, i1 = max(0, i0 - 1), min(len(x), max(i1, i0 + 1) + 1)
        seg = np.asarray(y[i0:i1], dtype=float)
        seg = seg[np.isfinite(seg)]
        if not len(seg):
            continue
        lo, hi = float(seg.min()), float(seg.max())
        d = 0.0 if lo <= pt.y() <= hi else min(abs(pt.y() - lo), abs(pt.y() - hi)) / px_y
        j = _nearest_index(x, pt.x())
        d = (d, abs(float(y[j]) - pt.y()) / px_y if np.isfinite(y[j]) else np.inf)
        if d < best_d:
            best, best_d = (it, j), d
    return best


class _RowItems:
    """The readout's items in one plot (made on first use)."""

    def __init__(self, plot: pg.PlotItem):
        self.plot = plot
        self.halo = pg.PlotCurveItem(connect="finite")
        self.halo.setZValue(40)
        self.dots = pg.ScatterPlotItem(size=7, pen=pg.mkPen("k", width=1))
        self.dots.setZValue(45)
        self.box = pg.TextItem(color=(20, 20, 20), fill=_FILL, border=_BORDER, anchor=(0, 1))
        self.box.setZValue(50)
        for it in (self.halo, self.dots, self.box):
            it.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            it.setAcceptHoverEvents(False)
            plot.addItem(it, ignoreBounds=True)
            it.hide()
        self.labels: list[pg.TextItem] = []

    def label(self, i: int) -> pg.TextItem:
        while len(self.labels) <= i:
            t = pg.TextItem(color=(20, 20, 20), fill=_FILL, anchor=(0, 0.5))
            t.setZValue(50)
            t.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            self.plot.addItem(t, ignoreBounds=True)
            self.labels.append(t)
        return self.labels[i]

    def hide(self) -> None:
        for it in (self.halo, self.dots, self.box, *self.labels):
            it.hide()


class HoverProbe:
    """The readouts of one cell (see the module doc). ``peaks``: mark the hovered
    trace's peaks (the spectrum)."""

    def __init__(self, cell, peaks: bool = False):
        self.cell, self.peaks = cell, peaks
        self._rows: dict[int, _RowItems] = {}
        self.hovered = None               # the hovered curve (tests)
        self.shown: list[str] = []        # the texts shown (tests)

    def _items(self, plot: pg.PlotItem) -> _RowItems:
        r = self._rows.get(id(plot))
        if r is None or r.plot is not plot or r.halo.scene() is not plot.scene():
            r = self._rows[id(plot)] = _RowItems(plot)
        return r

    def _legends(self) -> list:
        """The legend boxes over the plots (the selections', the grid): faded while a
        readout shows, as they sit above the data (siblings of the view's items)."""
        out = [p.legend for p in self.cell.nav_plots() if getattr(p, "legend", None) is not None]
        gl = getattr(self.cell, "_grid_legend", None)
        return out + ([gl] if gl is not None else [])

    def _fade(self, on: bool) -> None:
        for lg in self._legends():
            try:
                lg.setOpacity(LEGEND_FADE if on else 1.0)
            except RuntimeError:
                pass

    def clear(self) -> None:
        self._fade(False)
        for line in getattr(self.cell, "probes", {}).values():
            try:
                line.hide()
            except RuntimeError:
                pass
        for r in self._rows.values():
            try:
                r.hide()
            except RuntimeError:          # a row rebuilt: its items are gone
                pass
        self.hovered, self.shown = None, []

    def update(self, scene_pos, probe: bool) -> bool:
        """The readout at ``scene_pos``; False: not over a plot of the cell."""
        self.clear()
        plot = next((p for p in self.cell.nav_plots()
                     if p.getViewBox().sceneBoundingRect().contains(scene_pos)), None)
        if plot is None:
            return False
        got = self.cell.probe_items(plot)
        if got is None:
            return False
        xname, xunit, fx, fy, items = got
        vb = plot.getViewBox()
        pt = vb.mapSceneToView(scene_pos)
        r = self._items(plot)
        if probe:
            self._probe(r, vb, pt, xname, xunit, fx, fy, items)
        else:
            self._hover(r, vb, pt, xname, xunit, fx, fy, items)
        self._fade(bool(self.shown))
        return True

    def _hover(self, r: _RowItems, vb, pt, xname, xunit, fx, fy, items) -> None:
        hit = nearest_curve(vb, items, pt)
        if hit is None:
            return
        (label, unit, c), j = hit
        self.hovered = c
        x, y = c.getData()
        pen = c.opts.get("pen")
        color = QColor(pen.color()) if pen is not None else QColor(40, 40, 40)
        r.halo.setData(x, y)
        r.halo.setPen(pg.mkPen(color, width=3.5))
        r.halo.show()
        text = f"{label}\n{xname} = {fmt(fx(pt.x()))} {xunit} · {fmt(fy(float(y[j])))} {unit}"
        r.box.setText(text, color=(20, 20, 20))
        r.box.setAnchor((0, 1))
        px_x, _ = vb.viewPixelSize()
        r.box.setPos(pt.x() + 12 * px_x, pt.y())
        r.box.show()
        self.shown = [text]
        if self.peaks:
            with np.errstate(over="ignore", invalid="ignore"):
                fxs = np.asarray(fx(np.asarray(x, dtype=float)), dtype=float)
                ys = np.asarray(fy(np.asarray(y, dtype=float)), dtype=float)
            idx = peaks(fxs, ys)
            if idx:
                r.dots.setData(x[idx], y[idx], brush=pg.mkBrush(color))
                r.dots.show()
                for k, i in enumerate(idx):
                    t = r.label(k)
                    s = f"{fmt(fxs[i])} {xunit} · {fmt(ys[i])} {unit}"
                    t.setText(s, color=(20, 20, 20))
                    t.setPos(x[i] + 4 * px_x, y[i])
                    t.show()
                    self.shown.append(s)

    def _probe(self, r: _RowItems, vb, pt, xname, xunit, fx, fy, items) -> None:
        for line in getattr(self.cell, "probes", {}).values():
            line.setPos(pt.x())
            line.show()
        pts = []
        for label, unit, c in items:
            x, y = c.getData()
            if x is None or not len(x):
                continue
            j = _nearest_index(x, pt.x())
            if not np.isfinite(y[j]):
                continue
            pen = c.opts.get("pen")
            pts.append((float(x[j]), float(y[j]), label, unit,
                        QColor(pen.color()) if pen is not None else QColor(40, 40, 40)))
        pts = pts[:MAX_PROBE]
        px_x, px_y = vb.viewPixelSize()
        top = vb.viewRange()[1][1]
        r.box.setText(f"{xname} = {fmt(fx(pt.x()))} {xunit}", color=(20, 20, 20))
        r.box.setAnchor((0, 0))
        r.box.setPos(pt.x() + 4 * px_x, top)
        r.box.show()
        self.shown = [f"{xname} = {fmt(fx(pt.x()))} {xunit}"]
        if not pts:
            return
        r.dots.setData([p[0] for p in pts], [p[1] for p in pts],
                       brush=[pg.mkBrush(p[4]) for p in pts])
        r.dots.show()
        # labels by height, pushed apart by LABEL_GAP_PX (y may be inverted: depth)
        inv = vb.yInverted()
        order = sorted(range(len(pts)), key=lambda k: pts[k][1], reverse=not inv)
        gap = LABEL_GAP_PX * abs(px_y)
        placed: list[float] = []
        for n, k in enumerate(order):
            x, y, label, unit, color = pts[k]
            ly = y
            if placed:
                ly = min(ly, placed[-1] - gap) if not inv else max(ly, placed[-1] + gap)
            placed.append(ly)
            t = r.label(n)
            s = f"{label}: {fmt(fy(y))} {unit}"
            t.setText(s, color=color.darker(130))
            t.setPos(x + 6 * px_x, ly)
            t.show()
            self.shown.append(s)
