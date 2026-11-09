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

The vehicle's mode / gain / armed (``rov:mode``) is a band row, not a curve:
one labelled band per flight-mode span (MANUAL blue, ALT_HOLD yellow, ...),
the armed spans as a green bar along its bottom, and the pilot gain as a step
line on its 0 .. 1 axis.

The activity row (the Timeline variable "Activity (card)", ``rec:card``, timeline
only, on top): the recording card's preview (``replay/card.py``: each gauge's RMS
about its mean per time bin, dB re 1 mV on the card's fixed -40..+15 dB scale,
inferno, W1 A on top) computed from the source for the span shown, a bin per 2 px
(``daq/analysis.card_image``), so it reads like the Recordings window's thumbnail
zoomed out and sharpens when zoomed in; live data too. Hover: gauge and dB.

Each row keeps at least ``ROW_MIN`` px, so its y labels don't overlap; the
cell's minimum height grows with the rows, and the window's splitter makes
room for it.

The rows are kept (a unit's plot and its overlay are made once and only moved
in the layout), so checking variables does not rebuild the regions.

The manoeuvre lane (when the recording has its sidecar, ``replay/manoeuvres.py``;
``set_manoeuvres``): a row under the mode band, above the variables' rows,
with the stick segments as bars, coloured by axis and sign (``LANE_COLORS``; the minus side a darker shade),
stronger the higher the level; rest is empty, the first ``COAST_S`` of a rest
after a release (the coast) hatched in the released axis' colour, combos grey.
Hover: class, level, duration. A click makes a selection of the event's
window (the transition starting the segment, else the hold; ``laneClicked``);
right-click: Compare similar... (``laneCompare``: the event, for an Events
window).
"""

from __future__ import annotations

import math
from typing import Callable, Optional

import numpy as np
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QBrush, QColor
from PyQt6.QtWidgets import QMenu, QWidget

import pyqtgraph as pg

from wavi_py.daq.analysis import CARD_KEY, UNIT_GROUPS, Dataset, card_image
from wavi_py.daq.series import WSK
from wavi_py.replay.rov import MODE_KEY
from wavi_py.daq.series import WSK
from wavi_py.views.analysis_cells import AXIS_W, SENSOR, SELECTION, _Cell, hook_click, probe_line
from wavi_py.views.plot_tools import SELECT
from wavi_py.views.sensor_grid import gauge_color
from wavi_py.views.selections import SelectionModel, SelectionOverlay, _hook_drag
from wavi_py.views.whisker_plots import _show_grid

# the timeline's curves: dark, plain (the regions carry the selections' colours)
TIMELINE_COLORS = ((40, 90, 170), (70, 70, 70), (20, 120, 60), (150, 30, 30), (100, 60, 150),
                   (0, 110, 120), (120, 110, 0), (190, 80, 150))
_HEAD_PEN = pg.mkPen((210, 0, 0), width=2)
_BARE = ""                                       # the row shown with no variable
ROW_MIN = 58                                     # px a row keeps (its y labels don't overlap)
_AXIS_H = 34                                     # the bottom row's time axis and label
LANE = "stick"                                   # the manoeuvre lane's row key
LANE_H = 34
CARD = "card"                                    # the activity row's key (its unit)
CARD_H = 72                                      # px: 32 gauges at ~2 px each
LANE_COLORS = {"surge": (31, 119, 180), "sway": (44, 160, 44), "heave": (148, 103, 189),
               "yaw": (230, 120, 0)}
COMBO_COLOR = (120, 120, 120)
COAST_S = 2.0
_AXES = ("surge", "sway", "heave", "yaw")


def lane_bars(doc: dict) -> list[tuple]:
    """[(t0, t1, QBrush, segment)] of a sidecar's segments (see the module doc)."""
    out = []
    segs = doc.get("segments") or []
    prev = None
    for sg in segs:
        cls = sg.get("class", "")
        t0, t1 = float(sg["t0"]), float(sg["t1"])
        if cls == "rest":
            if prev is not None and prev.get("class", "").startswith("single:"):
                ax = prev["class"].split(":")[1][:-1]
                c = QColor(*LANE_COLORS.get(ax, COMBO_COLOR))
                out.append((t0, min(t1, t0 + COAST_S), QBrush(c, Qt.BrushStyle.BDiagPattern), sg))
        elif cls == "combo":
            out.append((t0, t1, QBrush(QColor(*COMBO_COLOR, 150)), sg))
        elif cls.startswith("single:"):
            ax, sign = cls.split(":")[1][:-1], cls[-1]
            lv = sg.get("level") or [0, 0, 0, 0]
            a = abs(float(lv[_AXES.index(ax)] or 0)) if ax in _AXES else 0.5
            c = QColor(*LANE_COLORS.get(ax, COMBO_COLOR))
            if sign == "-":
                c = c.darker(160)
            c.setAlpha(int(70 + 185 * min(1.0, a)))
            out.append((t0, t1, QBrush(c), sg))
        if cls != "disarmed":
            prev = sg
    return out


def segment_text(sg: dict) -> str:
    lv = sg.get("level") or []
    parts = [f"{a} {float(v):+.2f}" for a, v in zip(_AXES, lv) if v]
    minor = sg.get("minor") or {}
    extra = (" · minor " + ", ".join(minor)) if minor else ""
    return (f"{sg.get('class', '')}  {', '.join(parts) or 'rest'} · "
            f"{float(sg['t1']) - float(sg['t0']):.2f} s ({float(sg['t0']):.2f} .. "
            f"{float(sg['t1']):.2f} s){extra}")
# the mode band: a light fill per ArduSub mode (none of the selections' colours)
MODE_COLORS = {"MANUAL": (200, 222, 255), "ALT_HOLD": (250, 240, 165), "STABILIZE": (225, 205, 245),
               "POSHOLD": (195, 238, 228), "SURFTRAK": (240, 215, 200)}
_MODE_OTHER = (225, 225, 225)
_ARMED_BRUSH = pg.mkBrush(60, 170, 60, 200)
_GAIN_PEN = pg.mkPen((40, 40, 40), width=1.5)


def _row_h(unit: str) -> int:
    return LANE_H if unit == LANE else CARD_H if unit == CARD else ROW_MIN


class TimelineCell(_Cell):
    """See the module doc. ``win``: the Analysis window (its status bar and Select
    action); ``replay``: the ``ReplayModel`` (playhead, seek) or None."""

    TOOLS = (SELECT,)                            # Select makes a selection; no Zoom box (X only)
    madeSelection = pyqtSignal(object)           # a Select drag here made this one
    laneClicked = pyqtSignal(float, float)       # the lane: a selection of this window
    laneCompare = pyqtSignal(object)             # the lane: Compare similar (the event)

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
        self._lane_doc: Optional[dict] = None
        self._lane_bars: list = []
        self._lane_item = None
        self.color_by = SELECTION                        # sensor: the gauges in their own colour
        self._mode_items: list = []                      # the mode band's spans and labels
        self._mode_key = None
        self._card_img = None                            # the activity row's image
        self._card = None                                # (q, a, b) drawn
        self._show_rows([_BARE])
        self.glw.scene().sigMouseMoved.connect(self._lane_hover)

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
        p.setLabel("left", "mode / gain" if unit == "mode" else unit)
        if unit == "mode":                               # 0 .. 1: the pilot gain's axis
            p.setYRange(0.0, 1.05, padding=0)
            p.getViewBox().setLimits(yMin=-0.02, yMax=1.1)
        if unit == LANE:
            p.setYRange(0, 1, padding=0)
            p.getAxis("left").setTicks([[]])
            p.showGrid(x=True, y=False)
        if unit == "cm":
            p.getViewBox().invertY(True)                 # depth goes down
        if unit == CARD:                                 # gauges top to bottom, no y ticks
            p.setLabel("left", "activity")
            p.getViewBox().invertY(True)
            p.getAxis("left").setTicks([[]])
            p.showGrid(x=True, y=False)
        vb = p.getViewBox()
        vb.sigRangeChangedManually.connect(self._on_manual)
        vb.sigXRangeChanged.connect(lambda *_: self.changed.emit())   # re-decimate
        _hook_drag(self._win, vb, self.model, self._select)
        vb.select_action = self._select      # as in Line: Select on, a body drag selects anew
        hooked = vb.mouseDragEvent

        def drag(ev, axis=None):
            # a Select drag here makes a selection: tell the window (it includes it)
            n = len(self.model)
            hooked(ev, axis)
            if len(self.model) > n:
                self.madeSelection.emit(self.model.selections[-1])
        vb.mouseDragEvent = drag
        hook_click(vb, self._lane_click if unit == LANE else self._click)
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
        for u, p in self.rows.items():
            base = _row_h(u)
            p.setMinimumHeight(base + (_AXIS_H if p is last else 0))
            if u == LANE:
                p.setMaximumHeight(base + (_AXIS_H if p is last else 0))
        # the splitter keeps the timeline at least this tall (it grows with the rows)
        self.setMinimumHeight(sum(_row_h(u) for u in units) + _AXIS_H + 6)
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
        for u, p in self.rows.items():
            if u == "mode":
                p.setYRange(0.0, 1.05, padding=0)
            else:
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
        more = [(float(depth_t[0]), float(depth_t[-1]))] if len(depth_t) else []
        rex = source.rov.extent() if source is not None else None
        more += [rex] if rex is not None else []
        for e in more:
            extent = e if extent is None else (min(extent[0], e[0]), max(extent[1], e[1]))
        units = [u for u in UNIT_GROUPS if any(v.unit == u for v in ds.variables)]
        # the activity row on top; with the stick lane: the mode band, the lane, then
        # the variables' rows
        top = [u for u in ((CARD, "mode") if self._lane_doc is not None else (CARD,)) if u in units]
        units = top + ([LANE] if self._lane_doc is not None else []) + \
            [u for u in units if u not in top]
        units = units or [_BARE]
        self._show_rows(units)
        new_extent = extent != self._extent
        self._extent = extent
        if new_extent and not self.zoomed:
            self.fit()
        x0, x1 = self._anchor.getViewBox().viewRange()[0]
        n_px = max(50, int(self._anchor.getViewBox().width()) or 600)
        v0 = None if ds.v0 is None else np.asarray(ds.v0).tobytes()
        inputs = (id(source.series) if source is not None else None, extent, round(x0, 6),
                  round(x1, 6), n_px, tuple(v.key for v in ds.variables), ds.despike, v0, ds.scale,
                  self.color_by)
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
            color = gauge_color(v.col) if self.color_by == SENSOR and v.source == WSK else \
                TIMELINE_COLORS[order.index(v.key) % len(TIMELINE_COLORS)]
            c.setPen(pg.mkPen(color, width=1.5 if v.unit == "cm" else 1))
            c.setData(tr.t, tr.y)
            seen.add(v.key)
        self._draw_card(source if any(v.key == CARD_KEY for v in ds.variables) else None,
                        x0, x1, n_px, ds.despike)
        if any(v.key == MODE_KEY for v in ds.variables) and source is not None:
            self._draw_modes(source.rov)
            seen.add(MODE_KEY)
        else:
            self._draw_modes(None)
        for k in [k for k in self.curves if k not in seen]:
            c = self.curves.pop(k)
            self._plots[c.unit].removeItem(c)
        self._legends(ds)
        self.glw.scene().prepareForPaint()

    def _draw_card(self, source, x0: float, x1: float, n_px: int, despike: bool) -> None:
        """The activity row: the recording card's preview (gauge rows, W1 A on top;
        inferno, the card's fixed dB scale) over the span shown, a column per 2 px."""
        got = card_image(source, x0, x1, max(16, n_px // 2), despike) if source is not None else None
        img = self._card_img
        if got is None:
            if img is not None:
                img.setVisible(False)
            self._card = None
            return
        q, a, b = got
        if img is None:
            from wavi_py.views.recordings_browser import _lut
            img = self._card_img = pg.ImageItem(axisOrder="row-major")
            img.setLookupTable(_lut())
            img.setZValue(-30)
            self._row(CARD).addItem(img, ignoreBounds=True)
        img.setImage(q, levels=(0, 255), autoLevels=False)
        img.setRect(a, 0.0, b - a, q.shape[0])
        img.setVisible(True)
        p = self._row(CARD)
        p.setYRange(0, q.shape[0], padding=0)
        p.getViewBox().setLimits(yMin=0, yMax=q.shape[0])
        self._card = (q, a, b)

    def card_at(self, t: float, y: float) -> Optional[tuple[int, float]]:
        """(gauge column, dB) of the activity row at (t, y), or None."""
        if self._card is None:
            return None
        q, a, b = self._card
        if not a <= t < b or not 0 <= y < q.shape[0]:
            return None
        from wavi_py.replay.card import HI_DB, LO_DB
        g, i = int(y), min(int((t - a) / (b - a) * q.shape[1]), q.shape[1] - 1)
        return g, LO_DB + q[g, i] * (HI_DB - LO_DB) / 255.0

    def _draw_modes(self, rov) -> None:
        """The mode band row (see the module doc); rebuilt only when the spans, the
        armed spans or the gain changed (live: they grow)."""
        gt, gy = rov.series("rov:gain") if rov else (np.empty(0), np.empty(0))
        key = None if not rov else (tuple(rov.modes), tuple(map(tuple, rov.armed)), len(gt))
        if key == self._mode_key:
            return
        self._mode_key = key
        p = self._row("mode")
        for it in self._mode_items:
            p.removeItem(it)
        self._mode_items = []
        if key is None:
            return
        from PyQt6.QtWidgets import QGraphicsRectItem
        for t0, t1, name in rov.modes:
            r = QGraphicsRectItem(t0, 0.0, max(t1 - t0, 1e-3), 1.05)
            r.setBrush(pg.mkBrush(*MODE_COLORS.get(name, _MODE_OTHER)))
            r.setPen(pg.mkPen(None))
            r.setZValue(-30)
            p.addItem(r, ignoreBounds=True)
            lab = pg.TextItem(name, color=(60, 60, 60), anchor=(0, 0))
            lab.setPos(t0, 1.05)
            lab.setZValue(-29)
            p.addItem(lab, ignoreBounds=True)
            self._mode_items += [r, lab]
        for a, b in rov.armed:
            r = QGraphicsRectItem(a, 0.0, max(b - a, 1e-3), 0.07)
            r.setBrush(_ARMED_BRUSH)
            r.setPen(pg.mkPen(None))
            r.setZValue(-28)
            r.setToolTip("armed")
            p.addItem(r, ignoreBounds=True)
            self._mode_items.append(r)
        c = self.curves.get(MODE_KEY)
        if c is None:
            c = self.curves[MODE_KEY] = pg.PlotCurveItem(pen=_GAIN_PEN)
            c.unit, c.label = "mode", "pilot gain"
            p.addItem(c)
        if len(gt):                                       # a step: each value until the next
            ts = np.r_[np.repeat(gt, 2)[1:], self._extent[1] if self._extent else gt[-1]]
            c.setData(ts, np.repeat(gy, 2))
        else:
            c.setData([], [])

    def _legends(self, ds: Dataset) -> None:
        """No legends (they crowd the short rows): hovering a trace names it at the
        mouse (``views/hover_probe.py``)."""
        for p in self.rows.values():
            if p.legend is not None:
                p.legend.clear()
                p.legend.setVisible(False)

    # ----- the manoeuvre lane

    def set_manoeuvres(self, doc: Optional[dict]) -> None:
        """The lane from a manoeuvres sidecar (None: no lane)."""
        if doc is self._lane_doc:
            return
        self._lane_doc = doc
        p = self._row(LANE)
        if self._lane_item is not None:
            p.removeItem(self._lane_item)
            self._lane_item = None
        self._lane_bars = lane_bars(doc) if doc is not None else []
        if self._lane_bars:
            x0 = np.array([b[0] for b in self._lane_bars])
            w = np.array([b[1] - b[0] for b in self._lane_bars])
            self._lane_item = pg.BarGraphItem(x0=x0, width=w, y0=0.12, height=0.76,
                                              brushes=[b[2] for b in self._lane_bars],
                                              pens=[pg.mkPen(None)] * len(self._lane_bars))
            self._lane_item.setZValue(-10)
            p.addItem(self._lane_item)
        self._last = None
        self.changed.emit()

    def lane_segment_at(self, t: float) -> Optional[dict]:
        if self._lane_doc is None:
            return None
        for sg in self._lane_doc.get("segments") or []:
            if float(sg["t0"]) <= t < float(sg["t1"]):
                return sg
        return None

    def lane_event_at(self, t: float) -> Optional[dict]:
        """The event of the segment at ``t``: the transition starting it, else its hold."""
        sg = self.lane_segment_at(t)
        if sg is None or sg.get("class") in ("rest", "disarmed"):
            return None
        t0 = float(sg["t0"])
        evs_ = [e for e in self._lane_doc.get("events") or [] if abs(float(e["t"]) - t0) < 1e-6]
        ev = next((e for e in evs_ if e.get("kind") == "transition"), None) or \
            next((e for e in evs_ if e.get("kind") == "hold"), None)
        if ev is not None:
            ev = dict(ev, _stem=self._lane_doc.get("stem", ""))
        return ev

    def _lane_hover(self, pos) -> None:
        c = self._plots.get(CARD)
        if c is not None and CARD in self.rows and c.sceneBoundingRect().contains(pos):
            pt = c.getViewBox().mapSceneToView(pos)
            got = self.card_at(float(pt.x()), float(pt.y()))
            names = {v.col: v.label for v in getattr(self._win, "_vars", []) if v.source == WSK}
            self.glw.setToolTip("" if got is None else
                                f"{names.get(got[0], f'gauge {got[0]}')}: {got[1]:.0f} dB re 1 mV RMS")
            return
        p = self._plots.get(LANE)
        if p is None or LANE not in self.rows or not p.sceneBoundingRect().contains(pos):
            return
        t = float(p.getViewBox().mapSceneToView(pos).x())
        sg = self.lane_segment_at(t)
        self.glw.setToolTip(segment_text(sg) if sg is not None else "")

    def _lane_click(self, vb: pg.ViewBox, ev) -> bool:
        t = float(vb.mapSceneToView(ev.scenePos()).x())
        if ev.button() == Qt.MouseButton.RightButton:
            e = self.lane_event_at(t)
            if e is None:
                return False
            ev.accept()
            m = QMenu(self)
            m.addAction(f"Compare similar... ({e.get('type')} {e.get('class')})").triggered.connect(
                lambda: self.laneCompare.emit(e))
            m.popup(ev.screenPos().toPoint())
            self._lane_menu = m
            return True
        if ev.button() != Qt.MouseButton.LeftButton:
            return False
        if ev.double():
            return self._click(vb, ev)
        ev.accept()
        e = self.lane_event_at(t)
        if e is not None:
            self.laneClicked.emit(float(e["t_start"]), float(e["t_end"]))
        return True

    def overlays(self) -> list[SelectionOverlay]:
        return list(self._overlays.values())


def timeline_source_of(fn: Optional[Callable]):
    """``fn()`` or None when it says there is none (a str)."""
    if fn is None:
        return None
    src = fn()
    return None if src is None or isinstance(src, str) else src
