"""The Analysis window's timeline: the whole recording above the 2x2.

Absolute recording time, one row per unit group (mV, mg, mrad/s, deg, cm: the
waveform's) of the window's Timeline variables (their own checkbox column in
the Variables tree, independent of the Plots column), min / max decimated to
the plot width over the visible span (``Dataset.overview``: the store's
envelope, so the whole 13 min dive draws as cheaply as a minute). Depth goes
down, as on the strip. With no Timeline variable there is still one bare row,
so the selections and the playhead show.

The selections are drawn and adjusted with the shared region item
(``SelectionOverlay``, one per row: edges, and the body once the regions branch
lands); the playhead is a red line (not draggable). Mouse: a plain drag pans,
the wheel zooms time, Select (S) + drag makes a new selection (``selections``'
drag hook; the window includes it), a double-click seeks the replay there.
There is no click-seek, so a click on a region or a near miss does not move
the playhead. The view starts on the whole source and keeps a hand zoom
(``zoomed``) until Reset (R).

The rows are kept (a unit's plot and its overlay are made once and only moved
in the layout), so checking variables does not rebuild the regions.
"""

from __future__ import annotations

import math
from typing import Callable, Optional

import numpy as np
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import QWidget

import pyqtgraph as pg

from wavi_py.daq.analysis import UNIT_GROUPS, Dataset
from wavi_py.views.analysis_cells import AXIS_W, _Cell, hook_click, probe_line
from wavi_py.views.selections import SelectionModel, SelectionOverlay, _hook_drag
from wavi_py.views.whisker_plots import _show_grid

# the timeline's curves: dark, plain (the regions carry the selections' colours)
TIMELINE_COLORS = ((40, 90, 170), (70, 70, 70), (20, 120, 60), (150, 30, 30), (100, 60, 150),
                   (0, 110, 120))
_HEAD_PEN = pg.mkPen((210, 0, 0), width=2)
_BARE = ""                                       # the row shown with no variable


class TimelineCell(_Cell):
    """See the module doc. ``win``: the Analysis window (its status bar and Select
    action); ``replay``: the ``ReplayModel`` (playhead, seek) or None."""

    madeSelection = pyqtSignal(object)           # a Select drag here made this one

    def __init__(self, model: SelectionModel, win, select_act, replay=None,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.model, self._win, self._select, self.replay = model, win, select_act, replay
        self.glw.ci.setSpacing(0)
        self._pages.setCurrentIndex(1)
        self._plots: dict[str, pg.PlotItem] = {}         # unit -> its row (kept)
        self._overlays: dict[str, SelectionOverlay] = {}
        self._heads: dict[str, pg.InfiniteLine] = {}
        self.probes: dict[str, pg.InfiniteLine] = {}
        self.rows: dict[str, pg.PlotItem] = {}           # the rows shown, top to bottom
        self.curves: dict[str, pg.PlotCurveItem] = {}    # var key -> curve
        self._anchor: Optional[pg.PlotItem] = None       # the top row shown: the X link
        self.zoomed = False
        self._extent: Optional[tuple] = None
        self._show_rows([_BARE])

    # ----- rows

    def _row(self, unit: str) -> pg.PlotItem:
        p = self._plots.get(unit)
        if p is not None:
            return p
        p = pg.PlotItem()
        p.setMenuEnabled(False)
        p.hideButtons()
        _show_grid(p)
        p.setMouseEnabled(x=True, y=False)
        p.getAxis("left").enableAutoSIPrefix(False)
        p.getAxis("left").setWidth(AXIS_W)
        p.setLabel("left", unit)
        if unit == "cm":
            p.getViewBox().invertY(True)                 # depth goes down
        vb = p.getViewBox()
        vb.sigRangeChangedManually.connect(self._on_manual)
        vb.sigXRangeChanged.connect(lambda *_: self.changed.emit())   # re-decimate
        _hook_drag(self._win, vb, self.model, self._select)
        hooked = vb.mouseDragEvent

        def drag(ev, axis=None):
            # a Select drag here makes a selection: tell the window (it includes it)
            n = len(self.model)
            hooked(ev, axis)
            if len(self.model) > n:
                self.madeSelection.emit(self.model.selections[-1])
        vb.mouseDragEvent = drag
        hook_click(vb, self._click)
        head = pg.InfiniteLine(angle=90, movable=False, pen=_HEAD_PEN)
        head.setZValue(20)
        head.setVisible(False)
        p.addItem(head, ignoreBounds=True)
        self._heads[unit] = head
        self.probes[unit] = probe_line(p)
        self._plots[unit] = p
        self._overlays[unit] = SelectionOverlay(self.model, [p], None, self)
        return p

    def _show_rows(self, units: list[str]) -> None:
        if units == list(self.rows):
            return
        xr = self._anchor.getViewBox().viewRange()[0] if self._anchor is not None else None
        for p in self.rows.values():
            self.glw.ci.removeItem(p)
            p.setXLink(None)
        self.rows = {}
        # the rows shown link to the top one (a row out of the layout has no screen
        # geometry, and pyqtgraph would not pass a linked range through it)
        self._anchor = self._row(units[0])
        for i, u in enumerate(units):
            p = self._row(u)
            self.glw.ci.addItem(p, row=i, col=0)
            p.setLabel("bottom", None)
            p.showAxis("bottom", i == len(units) - 1)
            if i:
                p.setXLink(self._anchor)
            self.rows[u] = p
        if xr is not None:
            self._anchor.setXRange(*xr, padding=0)
        last = self.rows[units[-1]]
        last.setLabel("bottom", "Recording time", units="s")
        self.show_position()

    def _on_manual(self, *_a) -> None:
        self.zoomed = True

    def on_manual(self) -> None:
        self.zoomed = True

    def nav_plots(self) -> list[pg.PlotItem]:
        return list(self.rows.values())

    def x_plot(self) -> Optional[pg.PlotItem]:
        return self._anchor

    def reset_view(self) -> None:
        super().reset_view()
        self.zoomed = False
        for p in self.rows.values():
            p.enableAutoRange(y=True)
        self.fit()

    def fit(self) -> None:
        if self._extent is not None:
            a, b = self._extent
            self._anchor.setXRange(a, max(b, a + 1e-3), padding=0.01)

    # ----- mouse

    def _click(self, vb: pg.ViewBox, ev) -> bool:
        if ev.button() != Qt.MouseButton.LeftButton:
            return False
        ev.accept()                                  # no click-seek: a click does nothing
        if ev.double():
            t = float(vb.mapSceneToView(ev.scenePos()).x())
            if self.replay is not None and self.replay.active:
                self.replay.request_seek(max(0.0, t), True)
                self._win.statusBar().showMessage(f"Seek to {t:.2f} s", 3000)
        return True

    # ----- playhead

    def show_position(self, *_a) -> None:
        r = self.replay
        t = r.position if r is not None and r.source is not None else math.nan
        for u, head in self._heads.items():
            on = math.isfinite(t) and u in self.rows
            if on:
                head.setValue(t)
            head.setVisible(on)

    # ----- probe

    def probe_items(self, plot: pg.PlotItem):
        unit = next((u for u, p in self.rows.items() if p is plot), None)
        if unit is None:
            return None
        items = [(c.label, unit, c) for c in self.curves.values() if c.unit == unit]
        return "t", "s", (lambda x: x), (lambda y: y), items

    # ----- drawing

    def render(self, source, ds: Dataset) -> None:
        """``source``: a ``SelSource`` (the whole recording / the live buffers) or None;
        ``ds``: the Timeline variables with the display context (no selections)."""
        extent = source.extent() if source is not None else None
        depth_t = source.depth[0] if source is not None else np.empty(0)
        if extent is None and len(depth_t):
            extent = (float(depth_t[0]), float(depth_t[-1]))
        elif extent is not None and len(depth_t):
            extent = (min(extent[0], float(depth_t[0])), max(extent[1], float(depth_t[-1])))
        units = [u for u in UNIT_GROUPS if any(v.unit == u for v in ds.variables)] or [_BARE]
        self._show_rows(units)
        new_extent = extent != self._extent
        self._extent = extent
        if new_extent and not self.zoomed:
            self.fit()
        x0, x1 = self._anchor.getViewBox().viewRange()[0]
        n_px = max(50, int(self._anchor.getViewBox().width()) or 600)
        v0 = None if ds.v0 is None else np.asarray(ds.v0).tobytes()
        inputs = (id(source.series) if source is not None else None, extent, round(x0, 6),
                  round(x1, 6), n_px, tuple(v.key for v in ds.variables), ds.despike, v0, ds.scale)
        if inputs == self._last:
            return
        self._last = inputs
        self.renders += 1
        seen = set()
        traces = ds.overview(source, x0, x1, n_px) if source is not None else []
        order = [v.key for v in ds.variables]
        for tr in traces:
            v = tr.var
            c = self.curves.get(v.key)
            if c is None or c.unit != v.unit:
                if c is not None:
                    self._plots[c.unit].removeItem(c)
                c = pg.PlotCurveItem(connect="finite")
                c.unit, c.label = v.unit, v.label
                self._plots[v.unit].addItem(c)
                self.curves[v.key] = c
            c.setPen(pg.mkPen(TIMELINE_COLORS[order.index(v.key) % len(TIMELINE_COLORS)],
                              width=1.5 if v.unit == "cm" else 1))
            c.setData(tr.t, tr.y)
            seen.add(v.key)
        for k in [k for k in self.curves if k not in seen]:
            c = self.curves.pop(k)
            self._plots[c.unit].removeItem(c)
        self._legends(ds)
        self.glw.scene().prepareForPaint()

    def _legends(self, ds: Dataset) -> None:
        key = tuple(v.key for v in ds.variables)
        if getattr(self, "_legend_key", None) == key:
            return
        self._legend_key = key
        for u, p in self.rows.items():
            if p.legend is None:
                p.addLegend(offset=(-10, 2), labelTextColor="k", verSpacing=-6,
                            brush=pg.mkBrush(255, 255, 255, 200), pen=pg.mkPen(180, 180, 180))
            p.legend.clear()
            for v in ds.variables:
                c = self.curves.get(v.key)
                if v.unit == u and c is not None:
                    p.legend.addItem(c, v.label)

    def overlays(self) -> list[SelectionOverlay]:
        return list(self._overlays.values())


def timeline_source_of(fn: Optional[Callable]):
    """``fn()`` or None when it says there is none (a str)."""
    if fn is None:
        return None
    src = fn()
    return None if src is None or isinstance(src, str) else src
