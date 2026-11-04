"""The Analysis window's 2x2 cells and its cursors.

Each cell draws a ``daq/analysis.Dataset`` (the window's included selections x
the variables checked for Plots, its sub-range, the panel's Despike / V0 /
scale) handed to ``render``; the window decides when (``views/analysis_window.py``)
and every cell does no work when its inputs did not change (``renders`` /
``computed`` count it, for the tests).

Encoding (waveform and spectrum): colour is the selection (its region's
colour); the variables in a row are told apart by the shade of it, dark to
light in the order they are listed (``shade_levels``). The legends: each row's
variables in their shade (grey), the top row's also the selections in their
colour. Rows are unit groups (mV, mg, mrad/s, deg, cm).

- ``WaveCell`` (top left, the former Compare tab): relative time, min / max
  decimated to the plot width (``Dataset.traces``). With the window's Select on,
  a drag makes the sub-range (a shaded band in every row, edges draggable). A
  click on a trace (within ``PICK_PX``) picks that (variable, selection) for the
  spectrogram, which the window then marks (``set_picked``: drawn thick, on
  top); a click on the picked trace adds a cursor; elsewhere nothing. Its time
  range is copied to and from the spectrogram (``xChanged`` / ``set_x``: the
  same span, whatever the two widths).
- ``SpectrumCell`` (top right, the former Spectrum tab): ``Dataset.spectrum``
  per (variable, selection), of the whole selections or the sub-range. Options:
  window, detrend, log f, log amplitude (on by default), f range (0 .. the highest Nyquist until
  edited). A trace is recomputed only when its inputs change (the selection's
  span, the options, Despike, that gauge's V0 / scale).
- ``SpecgramCell`` (bottom left): ONE tile, the picked (variable, selection),
  full cell size, relative time; the picker is two combo boxes in its header.
  Colour range per unit group as in the Spec window (Auto: 1st to 99.5th
  percentile of the tile; or the min / max boxes, kept per unit and per lin /
  dB; ``spectral_base``): the controls show the picked variable's unit, so
  switching to accel and back keeps the gauges' range. A horizontal colour bar
  on top keeps the plot as wide as the waveform's rows.
- ``CursorCell`` (bottom right): the picked tile's spectrogram column at each
  cursor (``spectrogram_column``), one curve per cursor in the cursor's colour,
  legend "cursor number  t"; dB follows the spectrogram's dB box.

Cursors are numbered vertical lines at a relative time (``CursorLine``, kept
in sync per plot by ``CursorLines``), drawn in every waveform row and in the
spectrogram: drag to move, right-click to delete, Del deletes the hovered one.
Their colours (``CURSOR_COLORS``: saturated red, green, azure, magenta, black)
are none of the selections' muted ones, and solid (no dash styles); in the
spectrogram they get a white outline to stand out on its colours.

Every cell has a header with a collapse button at its right
(``collapseRequested``; ``TITLE`` names it on the window's restore bar).

Navigation (the window's keys and Probe) goes through ``nav_plots`` (the plots
a Y key may act on), ``x_plot`` (the one X keys scale), ``on_manual`` (a key
zoomed: stop following the data) and ``probe_items(plot)`` (the curves a
hover reads, with the axis conversions).
"""

from __future__ import annotations

from typing import Callable, Optional

import numpy as np
from PyQt6.QtCore import QPointF, Qt, pyqtSignal
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QHBoxLayout, QLabel,
                             QSizePolicy, QStackedWidget, QToolButton, QVBoxLayout, QWidget)

import pyqtgraph as pg

from wavi_py.daq.analysis import UNIT_GROUPS, Dataset, spectrogram_column
from wavi_py.daq.series import WSK
from wavi_py.views.render_sched import GraphicsLayoutWidget
from wavi_py.views.selections import sel_color
from wavi_py.views.spectral_base import (_DB_LEVELS, _LIN_LEVELS, auto_levels, show_level_boxes,
                                         to_db)
from wavi_py.views.whisker_plots import _show_grid

# a row's variables: HSL lightness of the selection's colour, dark to light (one
# variable keeps the colour itself); the light end stays readable as a 1 px line on white
_L_DARK, _L_LIGHT = 0.18, 0.66
_SUB_BRUSH = pg.mkBrush(60, 60, 60, 28)
_SUB_PEN = pg.mkPen((60, 60, 60), width=1, style=Qt.PenStyle.DashLine)
_PROBE_PEN = pg.mkPen((90, 90, 90), width=1)
AXIS_W = 56                                    # left axes: the cells' plots line up
SEGMENTS_S = (0.25, 0.5, 1.0, 2.0, 5.0)
OVERLAPS = (0.0, 0.5, 0.75, 0.9)
PICK_PX = 5                                    # a click this near a trace picks it
# cursors: saturated, none of the selections' colours (SEL_COLORS) nor the grey sub-range
CURSOR_COLORS = ((215, 25, 28), (0, 150, 0), (0, 110, 255), (210, 0, 210), (0, 0, 0))


def cursor_color(number: int) -> QColor:
    return QColor(*CURSOR_COLORS[(number - 1) % len(CURSOR_COLORS)])


def shade_levels(n: int) -> list[Optional[float]]:
    """Lightness of each of ``n`` variables in a row (None: the colour as it is)."""
    if n <= 1:
        return [None] * n
    return [_L_DARK + (_L_LIGHT - _L_DARK) * i / (n - 1) for i in range(n)]


def shade(color: QColor, lightness: Optional[float]) -> QColor:
    """``color`` at HSL ``lightness`` (hue and saturation kept)."""
    if lightness is None:
        return QColor(color)
    h, s_, _l, _a = color.getHslF()
    return QColor.fromHslF(max(h, 0.0), s_, lightness)


def shades_of(ds: Dataset) -> dict:
    """Each variable's lightness: dark to light within its row, in the listed order."""
    out = {}
    for u in UNIT_GROUPS:
        vs = [v for v in ds.variables if v.unit == u]
        out.update(zip((v.key for v in vs), shade_levels(len(vs))))
    return out


def sub_text(sub) -> str:
    """The spectral cells' header: what part of the selections they use."""
    if sub is None:
        return "Whole selections (waveform: Select (S) and drag for a sub-range)"
    return (f"Sub-range {sub[0]:.2f} .. {sub[1]:.2f} s from each selection's start, clipped to"
            " its length (Esc clears)")


def _input_key(ds: Dataset, sel, var, *opts) -> tuple:
    """Everything a spectrum / spectrogram of (sel, var) depends on."""
    v0 = (float(ds.v0[var.col]) if var.source == WSK and ds.v0 is not None
          and var.col < len(ds.v0) else 0.0)
    return (ds.span(sel), id(sel.source), ds.despike and var.source == WSK, v0,
            ds.scale if var.source == WSK else 1.0, *opts)


def _hook_drag(vb: pg.ViewBox, handler: Callable) -> None:
    """``handler(vb, ev)`` gets the ViewBox's drags first; False: the ViewBox's own."""
    def drag(ev, axis=None):
        if axis is None and handler(vb, ev):
            return
        type(vb).mouseDragEvent(vb, ev, axis)
    vb.mouseDragEvent = drag


def hook_click(vb: pg.ViewBox, handler: Callable) -> None:
    """``handler(vb, ev)`` gets the clicks that reach the ViewBox (no item above it
    took them: a cursor line or a region takes its own); False: the ViewBox's own."""
    def click(ev):
        if handler(vb, ev):
            return
        type(vb).mouseClickEvent(vb, ev)
    vb.mouseClickEvent = click


def probe_line(plot: pg.PlotItem) -> pg.InfiniteLine:
    line = pg.InfiniteLine(angle=90, movable=False, pen=_PROBE_PEN)
    line.setZValue(25)
    line.hide()
    plot.addItem(line, ignoreBounds=True)
    return line


# ----------------------------------------------------------------------
# cursors
# ----------------------------------------------------------------------

class CursorLine(pg.InfiniteLine):
    """One cursor in one plot (see the module doc). ``owner`` has ``move_cursor(n,
    t)``, ``delete_cursor(n)`` and ``hover_cursor(n or None)``."""

    def __init__(self, number: int, t: float, owner, halo: bool = False):
        c = cursor_color(number)
        super().__init__(pos=t, angle=90, movable=True, pen=pg.mkPen(c, width=2),
                         hoverPen=pg.mkPen(c, width=4), label=str(number),
                         labelOpts={"position": 0.94, "color": c,
                                    "fill": pg.mkBrush(255, 255, 255, 220)})
        self.number, self._owner, self._halo = number, owner, halo
        self.setZValue(30)
        self.sigDragged.connect(lambda line: owner.move_cursor(self.number, float(line.value())))

    def paint(self, p, *args) -> None:
        if self._halo:
            left, right = self._endPoints
            p.setPen(pg.mkPen((255, 255, 255), width=5))
            p.drawLine(QPointF(left, 0), QPointF(right, 0))
        super().paint(p, *args)

    def mouseClickEvent(self, ev) -> None:  # noqa: N802 — pyqtgraph naming
        if ev.button() == Qt.MouseButton.RightButton and not self.moving:
            ev.accept()
            self._owner.delete_cursor(self.number)
            return
        if ev.button() == Qt.MouseButton.LeftButton:
            ev.accept()                          # a click on a cursor adds no other
        super().mouseClickEvent(ev)

    def hoverEvent(self, ev) -> None:  # noqa: N802 — pyqtgraph naming
        super().hoverEvent(ev)
        self._owner.hover_cursor(None if ev.isExit() else self.number)


class CursorLines:
    """The cursors' lines in a set of plots, kept in sync with ``[(number, t)]``."""

    def __init__(self, owner, halo: bool = False):
        self._owner, self._halo = owner, halo
        self.lines: dict[tuple, tuple] = {}          # (id(plot), number) -> (plot, line)

    def sync(self, plots, cursors) -> None:
        want = {(id(p), n): (p, t) for p in plots for n, t in cursors}
        for k in [k for k in self.lines if k not in want]:
            p, line = self.lines.pop(k)
            p.removeItem(line)
        for k, (p, t) in want.items():
            got = self.lines.get(k)
            if got is None:
                line = CursorLine(k[1], t, self._owner, self._halo)
                p.addItem(line, ignoreBounds=True)
                self.lines[k] = (p, line)
            elif not got[1].moving and abs(got[1].value() - t) > 1e-12:
                got[1].blockSignals(True)
                got[1].setValue(t)
                got[1].blockSignals(False)


# ----------------------------------------------------------------------
# rows by unit: waveform and spectrum
# ----------------------------------------------------------------------

def _spin(lo: float, hi: float, value: float, decimals: int = 2, special: str = "") -> QDoubleSpinBox:
    b = QDoubleSpinBox()
    b.setRange(lo, hi)
    b.setDecimals(decimals)
    b.setValue(value)
    b.setKeyboardTracking(False)
    if special:
        b.setSpecialValueText(special)
    return b


class _Header(QWidget):
    """An options row (spectral cells) over the sub-range line."""

    def __init__(self, parent: Optional[QWidget] = None, sub_line: bool = True):
        super().__init__(parent)
        # no minimum width: the 2x2's borders move freely (a narrow cell clips its options)
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed)
        outer = QHBoxLayout(self)
        outer.setContentsMargins(4, 2, 2, 0)
        outer.setSpacing(4)
        v = QVBoxLayout()
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(1)
        outer.addLayout(v, 1)
        # the cell's collapse button, top right (the window restores it from its bar)
        self.collapse_btn = QToolButton()
        self.collapse_btn.setText("–")
        self.collapse_btn.setAutoRaise(True)
        self.collapse_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.collapse_btn.setToolTip("Collapse this cell (the bar above the plots brings it back)")
        outer.addWidget(self.collapse_btn, 0, Qt.AlignmentFlag.AlignTop)
        self.row = QHBoxLayout()
        self.row.setSpacing(6)
        v.addLayout(self.row)
        self._v = v
        self.sub = QLabel(sub_text(None))
        self.sub.setStyleSheet("color: #555;")
        self.sub.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        self.sub.setVisible(sub_line)
        v.addWidget(self.sub)

    def new_row(self) -> None:
        """Further options on a second line (a narrow cell)."""
        self.row.addStretch(1)
        self.row = QHBoxLayout()
        self.row.setSpacing(6)
        self._v.insertLayout(self._v.count() - 1, self.row)

    def add(self, label: str, w: QWidget) -> QWidget:
        if label:
            self.row.addWidget(QLabel(label))
        self.row.addWidget(w)
        return w


class _Cell(QWidget):
    """A cell: an optional header over a hint page / a GraphicsLayoutWidget."""

    changed = pyqtSignal()                       # an option: redraw
    collapseRequested = pyqtSignal()             # its header's collapse button
    TITLE = ""                                   # the window's restore bar names it so

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        self.header: Optional[_Header] = None
        self._lay = lay
        self._pages = QStackedWidget()
        lay.addWidget(self._pages, 1)
        self._hint = QLabel()
        self._hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._hint.setWordWrap(True)
        self._hint.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.glw = GraphicsLayoutWidget()
        self.glw.setMinimumSize(0, 0)
        self.glw.setBackground("w")
        self._pages.addWidget(self._hint)
        self._pages.addWidget(self.glw)
        self._last: Optional[tuple] = None           # the last render's inputs
        self.renders = 0                             # renders that did any work (tests)

    def _add_header(self, sub_line: bool = True) -> _Header:
        self.header = _Header(sub_line=sub_line)
        self.header.collapse_btn.clicked.connect(self.collapseRequested.emit)
        self._lay.insertWidget(0, self.header)
        return self.header

    def _add_title(self, text: str) -> QLabel:
        """A header with just a title (and the collapse button)."""
        h = self._add_header(sub_line=False)
        lab = QLabel(text)
        lab.setStyleSheet("color: #444;")
        lab.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        h.row.addWidget(lab, 1)
        return lab

    def set_sub_text(self, sub) -> None:
        if self.header is not None:
            self.header.sub.setText(sub_text(sub))

    def show_hint(self, text: str) -> None:
        self._hint.setText(text)
        self._pages.setCurrentIndex(0)
        self._last = None                            # redraw when there is something again

    @property
    def showing(self) -> bool:
        """The plots page is up (not the hint) and the cell is not collapsed."""
        return (self._pages.currentIndex() == 1 and not self.isHidden() and self.width() > 1
                and self.height() > 1)

    # navigation (the window's keys / Probe)
    def nav_plots(self) -> list[pg.PlotItem]:
        return []

    def x_plot(self) -> Optional[pg.PlotItem]:
        plots = self.nav_plots()
        return plots[0] if plots else None

    def on_manual(self) -> None:
        """A key zoomed / panned: stop following the data."""

    def reset_view(self) -> None:
        self._last = None

    def probe_items(self, plot: pg.PlotItem):
        """(x name, x unit, x from view, y from view, [(label, unit, curve)]) or None."""
        return None


class _RowsCell(_Cell):
    """Stacked rows by unit; curves keyed (selection, variable)."""

    X_LABEL = ("", "")

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.rows: dict[str, pg.PlotItem] = {}           # unit -> plot
        self.curves: dict[tuple, pg.PlotCurveItem] = {}  # (selection number, var key) -> curve
        self.probes: dict[str, pg.InfiniteLine] = {}     # unit -> its probe line
        self._legend_key: tuple = ()
        self._sel_labels: dict = {}                      # selection number -> its legend label
        self.picked: Optional[tuple] = None              # (sel number, var key) drawn thicker
        self._auto = True                                # X follows the data

    def nav_plots(self) -> list[pg.PlotItem]:
        return list(self.rows.values()) if self._pages.currentIndex() == 1 else []

    def on_manual(self) -> None:
        self._auto = False

    def reset_view(self) -> None:
        super().reset_view()
        self._auto = True
        for p in self.rows.values():
            p.enableAutoRange(y=True)

    def _rebuild_rows(self, units: list[str]) -> None:
        self.glw.clear()
        self.rows, self.curves, self.probes, self._legend_key = {}, {}, {}, ()
        first = None
        for i, u in enumerate(units):
            p = self.glw.addPlot(row=i, col=0)
            p.setLabel("left", u)
            _show_grid(p)
            p.getAxis("left").enableAutoSIPrefix(False)
            p.getAxis("left").setWidth(AXIS_W)            # the rows' plots line up
            p.setMenuEnabled(False)
            p.hideButtons()
            self._link_x(p, first)
            if first is None:
                first = p
            p.getViewBox().sigRangeChangedManually.connect(self._on_manual)
            self.rows[u] = p
            self.probes[u] = probe_line(p)
            self._row_added(p)
        if units:
            self.rows[units[-1]].setLabel("bottom", self.X_LABEL[0], units=self.X_LABEL[1])

    def _link_x(self, p: pg.PlotItem, first: Optional[pg.PlotItem]) -> None:
        if first is not None:
            p.setXLink(first)

    def _row_added(self, p: pg.PlotItem) -> None:
        """Hook: a row was made."""

    def _on_manual(self, *_a) -> None:
        self._auto = False

    def _units(self, ds: Dataset) -> list[str]:
        units = [u for u in UNIT_GROUPS if any(v.unit == u for v in ds.variables)]
        if units != list(self.rows):
            self._rebuild_rows(units)
        self._pages.setCurrentIndex(1)
        return units

    def _curve(self, sel, var, lightness) -> pg.PlotCurveItem:
        key = (sel.number, var.key)
        c = self.curves.get(key)
        if c is None:
            c = pg.PlotCurveItem(connect="finite")
            c.unit, c.look, c.label = var.unit, None, f"{var.label} #{sel.number}"
            self.rows[var.unit].addItem(c)
            self.curves[key] = c
        self._style(key, c, sel.number, lightness)
        return c

    def _style(self, key: tuple, c: pg.PlotCurveItem, number: int, lightness) -> None:
        """The pen: the selection's colour in the variable's shade (the checked order);
        the picked trace (waveform) thicker and on top."""
        look = (number, lightness, key == self.picked)
        if c.look != look:
            c.setPen(pg.mkPen(shade(sel_color(number), lightness), width=2.5 if look[2] else 1))
            c.setZValue(5 if look[2] else 0)
            c.look = look

    def _drop(self, seen: set) -> None:
        for key in [k for k in self.curves if k not in seen]:
            c = self.curves.pop(key)
            self.rows[c.unit].removeItem(c)

    @staticmethod
    def _sel_text(s) -> str:
        return f"#{s.number}  {s.t0:.2f} .. {s.t1:.2f} s"

    def _legends(self, ds: Dataset, shades: dict, units: list[str]) -> None:
        """Rebuilt when the variables or selections change; an edge drag only re-labels
        (a rebuild per drag step would cost more than the curves)."""
        key = (tuple(v.key for v in ds.variables), tuple(s.number for s in ds.selections),
               tuple(units))
        if key == self._legend_key:
            for s in ds.selections:
                lab = self._sel_labels.get(s.number)
                text = self._sel_text(s)
                if lab is not None and lab.text != text:
                    lab.setText(text)
            return
        self._legend_key = key
        self._sel_labels = {}
        for i, u in enumerate(units):
            p = self.rows[u]
            if p.legend is None:
                p.addLegend(offset=(-10, 5), labelTextColor="k",
                            brush=pg.mkBrush(255, 255, 255, 210), pen=pg.mkPen(180, 180, 180))
            p.legend.clear()
            for v in ds.variables:
                if v.unit == u:
                    grey = shade(QColor(128, 128, 128), shades[v.key])
                    p.legend.addItem(pg.PlotDataItem(pen=pg.mkPen(grey, width=3)), v.label)
            if i == 0:
                for s in ds.selections:
                    p.legend.addItem(pg.PlotDataItem(pen=pg.mkPen(sel_color(s.number), width=3)),
                                     self._sel_text(s))
                    self._sel_labels[s.number] = p.legend.items[-1][1]

    def probe_items(self, plot: pg.PlotItem):
        unit = next((u for u, p in self.rows.items() if p is plot), None)
        if unit is None:
            return None
        items = [(c.label, unit, c) for c in self.curves.values() if c.unit == unit]
        return (*self._probe_axes(), items)

    def _probe_axes(self) -> tuple:
        return "t", "s", (lambda x: x), (lambda y: y)


class WaveCell(_RowsCell):
    """The waveform (see the module doc). ``selecting()``: the window's Select is on;
    ``subChanged(sub or None)``; ``clicked(t, (sel number, var key) or None)``."""

    X_LABEL = ("Time from the selection's start", "s")
    TITLE = "Waveform"
    subChanged = pyqtSignal(object)
    clicked = pyqtSignal(float, object)
    xChanged = pyqtSignal(float, float)          # the time range shown (the spectrogram copies it)

    def __init__(self, selecting: Callable[[], bool] = lambda: False,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._selecting = selecting
        self.sub: Optional[tuple] = None
        self.bands: list[pg.LinearRegionItem] = []
        self.title = self._add_title(
            "Waveform · click a trace: the spectrogram shows it (drawn thick) · click the thick"
            " one again: a cursor there")
        self.glw.setToolTip("Click a trace to show it in the spectrogram (it is then drawn"
                            " thick); click the thick trace again to add a cursor there")

    def x_plot(self) -> Optional[pg.PlotItem]:
        return next(iter(self.rows.values()), None)

    def _rebuild_rows(self, units: list[str]) -> None:
        self.bands = []
        super()._rebuild_rows(units)
        first = self.x_plot()
        if first is not None:
            first.getViewBox().sigXRangeChanged.connect(
                lambda _vb, r: self.xChanged.emit(float(r[0]), float(r[1])))

    def set_x(self, lo: float, hi: float) -> None:
        """The time range from the spectrogram (the same span, whatever the widths)."""
        p = self.x_plot()
        if p is not None:
            x0, x1 = p.getViewBox().viewRange()[0]
            if abs(x0 - lo) > 1e-9 or abs(x1 - hi) > 1e-9:
                p.setXRange(lo, hi, padding=0)

    def set_picked(self, pick: Optional[tuple]) -> None:
        """The trace the spectrogram shows: drawn thick and on top."""
        if pick == self.picked:
            return
        self.picked = pick
        for key, c in self.curves.items():
            if c.look is not None:
                self._style(key, c, c.look[0], c.look[1])

    def _row_added(self, p: pg.PlotItem) -> None:
        _hook_drag(p.getViewBox(), self._drag)
        hook_click(p.getViewBox(), self._click)
        band = pg.LinearRegionItem((0, 1), movable=True, brush=_SUB_BRUSH, pen=_SUB_PEN,
                                   hoverPen=pg.mkPen((30, 30, 30), width=3))
        band.movable = False                          # edges only: the body pans
        band.setZValue(-20)
        band.setVisible(self.sub is not None)
        if self.sub is not None:
            band.setRegion(self.sub)
        band.sigRegionChanged.connect(self._band_dragged)
        p.addItem(band, ignoreBounds=True)
        self.bands.append(band)

    def _drag(self, vb: pg.ViewBox, ev) -> bool:
        if not self._selecting() or ev.button() != Qt.MouseButton.LeftButton:
            return False
        ev.accept()
        x0 = vb.mapSceneToView(ev.buttonDownScenePos()).x()
        x = vb.mapSceneToView(ev.scenePos()).x()
        if ev.isFinish() and abs(ev.scenePos().x() - ev.buttonDownScenePos().x()) < 3:
            return True                               # a click: nothing
        self.set_sub((x0, x), emit=True)
        return True

    def pick_at(self, vb: pg.ViewBox, scene_pos) -> Optional[tuple]:
        """(sel number, var key) of the trace within ``PICK_PX`` of ``scene_pos`` in
        ``vb``'s row (the nearest), or None. A min / max trace covers its block's
        whole span, so a point between a block's min and max is on it."""
        unit = next((u for u, p in self.rows.items() if p.getViewBox() is vb), None)
        if unit is None:
            return None
        pt = vb.mapSceneToView(scene_pos)
        px_x, px_y = vb.viewPixelSize()
        best, best_d = None, (PICK_PX + 1e-9, np.inf)
        for key, c in self.curves.items():
            if c.unit != unit or not c.isVisible():
                continue
            x, y = c.getData()
            if x is None or not len(x):
                continue
            i0 = int(np.searchsorted(x, pt.x() - PICK_PX * px_x))
            i1 = int(np.searchsorted(x, pt.x() + PICK_PX * px_x, side="right"))
            i0, i1 = max(0, i0 - 1), min(len(x), max(i1, i0 + 1) + 1)
            seg = y[i0:i1]
            seg = seg[np.isfinite(seg)]
            if not len(seg):
                continue
            lo, hi = float(seg.min()), float(seg.max())
            d = 0.0 if lo <= pt.y() <= hi else min(abs(pt.y() - lo), abs(pt.y() - hi)) / px_y
            # overlapping traces (both span the point): the one whose nearest sample is nearest
            j = int(min(max(0, np.searchsorted(x, pt.x())), len(x) - 1))
            d = (d, abs(float(y[j]) - pt.y()) / px_y if np.isfinite(y[j]) else np.inf)
            if d < best_d:
                best, best_d = key, d
        return best

    def _click(self, vb: pg.ViewBox, ev) -> bool:
        if ev.button() != Qt.MouseButton.LeftButton or ev.double():
            return False
        ev.accept()
        self.clicked.emit(float(vb.mapSceneToView(ev.scenePos()).x()), self.pick_at(vb, ev.scenePos()))
        return True

    def _band_dragged(self, band) -> None:
        self.set_sub(tuple(band.getRegion()), emit=True)

    def set_sub(self, sub, emit: bool = False) -> None:
        """The sub-range (a, b) seconds from each selection's start, or None."""
        if sub is not None:
            a, b = sorted(float(v) for v in sub)
            sub = (max(0.0, a), max(0.0, b))
            if sub[1] <= sub[0]:
                sub = None
        if sub == self.sub:
            return
        self.sub = sub
        for band in self.bands:
            band.blockSignals(True)
            if sub is not None:
                band.setRegion(sub)
            band.setVisible(sub is not None)
            band.blockSignals(False)
        if emit:
            self.subChanged.emit(sub)

    def fit_x(self, longest: float) -> None:
        """X from 0 to the longest selection, unless zoomed by hand."""
        if self._auto and longest > 0:
            p = self.x_plot()
            if p is not None:
                p.setXRange(0.0, longest, padding=0.01)

    def render(self, ds: Dataset, shades: dict) -> None:
        units = self._units(ds)
        first = self.rows[units[0]]
        n_px = max(50, int(first.getViewBox().width()) or 600)
        v0 = None if ds.v0 is None else np.asarray(ds.v0).tobytes()
        inputs = (n_px, tuple((s.number, s.t0, s.t1, id(s.source)) for s in ds.selections),
                  tuple(v.key for v in ds.variables), ds.despike, v0, ds.scale,
                  tuple(sorted(shades.items(), key=lambda kv: kv[0])), self._auto)
        if inputs == self._last:
            return
        self._last = inputs
        self.renders += 1
        seen = set()
        for tr in ds.traces(n_px):
            seen.add((tr.sel.number, tr.var.key))
            self._curve(tr.sel, tr.var, shades[tr.var.key]).setData(tr.t, tr.y)
        self._drop(seen)
        self._legends(ds, shades, units)
        self.glw.scene().prepareForPaint()


class SpectrumCell(_RowsCell):
    """Amplitude spectra per (variable, selection) (see the module doc)."""

    X_LABEL = ("Frequency", "Hz")
    TITLE = "Spectrum"

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        h = self._add_header()
        self.window_box = h.add("Window", QComboBox())
        self.window_box.addItems(["Hann", "Rectangular"])
        self.window_box.setToolTip("Hann (default) or rectangular (no window); either way a"
                                   " sine reads its amplitude")
        self.detrend_box = h.add("Detrend", QComboBox())
        self.detrend_box.addItems(["mean", "linear", "none"])
        self.detrend_box.setToolTip("Removed before the window: the mean (default), a"
                                    " fitted line, or nothing (the DC bin then shows the level)")
        h.new_row()
        self.log_f = h.add("", QCheckBox("log f"))
        self.log_a = h.add("", QCheckBox("log amp"))
        self.log_a.setChecked(True)                   # the default: spectra span decades
        self.log_a.setToolTip("Log amplitude axis")
        self.f_lo = h.add("f", _spin(0.0, 1e5, 0.0, 2))
        self.f_hi = h.add("..", _spin(0.0, 1e5, 0.0, 2, special="Nyquist"))
        for b in (self.f_lo, self.f_hi):
            b.setSuffix(" Hz")
            b.setToolTip("Frequency range shown (max at its lowest: up to the highest Nyquist)")
        h.row.addStretch(1)
        for w in (self.window_box, self.detrend_box):
            w.currentIndexChanged.connect(lambda _i: self.changed.emit())
        for w in (self.log_f, self.log_a):
            w.toggled.connect(lambda _on: self.changed.emit())
        for w in (self.f_lo, self.f_hi):
            w.valueChanged.connect(lambda _v: self.changed.emit())
        self._cache: dict[tuple, tuple] = {}          # (sel number, var key) -> (key, f, amp)
        self.computed = 0                             # spectra computed (tests)

    @property
    def options(self) -> tuple[str, str]:
        return (("hann", "rect")[self.window_box.currentIndex()],
                self.detrend_box.currentText())

    def _probe_axes(self) -> tuple:
        lf, la = self.log_f.isChecked(), self.log_a.isChecked()
        return ("f", "Hz", (lambda x: 10 ** x) if lf else (lambda x: x),
                (lambda y: 10 ** y) if la else (lambda y: y))

    def spectra(self, ds: Dataset) -> dict:
        """{(sel number, var key): (f, amp) or None}, from the cache where the inputs
        are unchanged."""
        win, det = self.options
        out, keep = {}, {}
        for sel, var in ds.pairs():
            k = (sel.number, var.key)
            key = _input_key(ds, sel, var, win, det)
            hit = self._cache.get(k)
            if hit is None or hit[0] != key:
                self.computed += 1
                got = ds.spectrum(sel, var, win, det)
                hit = (key, *(got if got is not None else (None, None)))
            keep[k] = hit
            out[k] = None if hit[1] is None else (hit[1], hit[2])
        self._cache = keep
        return out

    def render(self, ds: Dataset, shades: dict) -> None:
        win, det = self.options
        inputs = (tuple((s.number, _input_key(ds, s, v, win, det), v.key) for s, v in ds.pairs()),
                  tuple(sorted(shades.items(), key=lambda kv: kv[0])),
                  self.log_f.isChecked(), self.log_a.isChecked(),
                  self.f_lo.value(), self.f_hi.value(), self._auto)
        if inputs == self._last:
            return                                    # nothing changed: no work
        self._last = inputs
        self.renders += 1
        units = self._units(ds)
        lf, la = self.log_f.isChecked(), self.log_a.isChecked()
        seen, fmax, fmin_pos = set(), 0.0, np.inf
        for (num, key), got in self.spectra(ds).items():
            if got is None:
                continue
            sel = next(s for s in ds.selections if s.number == num)
            var = next(v for v in ds.variables if v.key == key)
            f, a = got
            if len(f):
                fmax = max(fmax, float(f[-1]))
                pos = f[f > 0]
                if len(pos):
                    fmin_pos = min(fmin_pos, float(pos[0]))
            # log axes: the curves get log10 of the data (setLogMode below only does the
            # ticks for bare curves); no 0 Hz, and zero amplitude is a gap
            with np.errstate(divide="ignore", invalid="ignore"):
                if lf:
                    f, a = np.log10(f[1:]), a[1:]
                if la:
                    a = np.where(a > 0, np.log10(a), np.nan)
            seen.add((num, key))
            self._curve(sel, var, shades[key]).setData(f, a)
        self._drop(seen)
        for p in self.rows.values():
            p.setLogMode(x=lf, y=la)
        self._legends(ds, shades, units)
        first = self.rows[units[0]]
        if self._auto and fmax > 0:
            lo, hi = self.f_lo.value(), self.f_hi.value() or fmax
            if hi > lo:
                if lf:
                    lo = max(lo, fmin_pos if np.isfinite(fmin_pos) else hi / 1e3)
                    first.setXRange(np.log10(lo), np.log10(hi), padding=0.01)
                else:
                    first.setXRange(lo, hi, padding=0.01)
        self.glw.scene().prepareForPaint()


# ----------------------------------------------------------------------
# the spectrogram (one tile) and the spectrum at the cursors
# ----------------------------------------------------------------------

class SpecgramCell(_Cell):
    """One tile, the picked (variable, selection) (see the module doc).
    ``clicked(t)``: a click in the tile (the window adds a cursor)."""

    clicked = pyqtSignal(float)
    TITLE = "Spectrogram"

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        h = self._add_header()
        self.var_box = h.add("", QComboBox())
        self.var_box.setToolTip("The variable shown (or click its trace in the waveform)")
        self.sel_box = h.add("", QComboBox())
        self.sel_box.setToolTip("The selection shown (or click its trace in the waveform)")
        self.seg_box = h.add("Seg", QComboBox())
        self.seg_box.addItems([f"{s:g} s" for s in SEGMENTS_S])
        self.seg_box.setCurrentIndex(SEGMENTS_S.index(1.0))
        self.seg_box.setToolTip("Segment length: frequency bins are 1 / length apart")
        self.ovl_box = h.add("Ovl", QComboBox())
        self.ovl_box.addItems([f"{int(o * 100)} %" for o in OVERLAPS])
        self.ovl_box.setCurrentIndex(OVERLAPS.index(0.75))
        self.ovl_box.setToolTip("Segment overlap")
        self.db_box = h.add("", QCheckBox("dB"))
        self.db_box.setToolTip("Colour by amplitude in dB re 1 unit instead of linear (and the"
                               " cursor spectra in dB)")
        self.auto_box = h.add("", QCheckBox("Auto"))
        self.auto_box.setChecked(True)
        self.auto_box.setToolTip("Colour range of this unit from the data (1st to 99.5th"
                                 " percentile of the tile); off: the min / max boxes. Each unit"
                                 " (mV, mg, ...) keeps its own")
        self.lo_box = h.add("", _spin(-1e6, 1e6, _LIN_LEVELS[0]))
        self.hi_box = h.add("..", _spin(-1e6, 1e6, _LIN_LEVELS[1]))
        for b in (self.lo_box, self.hi_box):
            b.setToolTip("Colour range min / max (typing one switches Auto off for this unit)")
        h.row.addStretch(1)
        self._ranges: dict[str, dict] = {}            # unit -> auto, and lin / dB fixed ranges
        for w in (self.seg_box, self.ovl_box):
            w.currentIndexChanged.connect(lambda _i: self.changed.emit())
        self.var_box.activated.connect(self._on_picker)
        self.sel_box.activated.connect(self._on_picker)
        self.db_box.toggled.connect(self._on_db)
        self.auto_box.toggled.connect(self._on_auto)
        for b in (self.lo_box, self.hi_box):
            b.valueChanged.connect(self._on_levels_edited)
        self._cmap = pg.colormap.get("viridis")
        self.cbar = pg.ColorBarItem(colorMap=self._cmap, interactive=False, width=12,
                                    orientation="horizontal")
        self.glw.addItem(self.cbar, row=0, col=0)
        self.plot = self.glw.addPlot(row=1, col=0)
        self.plot.setMenuEnabled(False)
        self.plot.hideButtons()
        self.plot.setLabel("left", "Hz")
        self.plot.getAxis("left").setWidth(AXIS_W)
        self.plot.setLabel("bottom", "Time from the selection's start", units="s")
        self.img = pg.ImageItem()
        self.img.setColorMap(self._cmap)
        self.plot.addItem(self.img)
        self.probe = probe_line(self.plot)
        hook_click(self.plot.getViewBox(), self._click)
        self.pick: Optional[tuple] = None             # (sel number, var key)
        self.var = None                               # the picked Variable / Selection
        self.sel = None
        self.levels: Optional[tuple] = None           # the range in use
        self.data = None                              # (tc, f, amp) shown, or None
        self._cache: dict[tuple, tuple] = {}
        self.computed = 0

    # ----- the picker

    def set_pick(self, number: int, key: str) -> None:
        if (number, key) != self.pick:
            self.pick = (number, key)
            self.changed.emit()

    def _on_picker(self, _i: int) -> None:
        num, key = self.sel_box.currentData(), self.var_box.currentData()
        if num is not None and key is not None:
            self.set_pick(num, key)

    def _sync_picker(self, ds: Dataset) -> None:
        """The combo boxes list the dataset's variables / selections; the pick falls
        back to the first of each when its own is gone."""
        nums = [s.number for s in ds.selections]
        keys = [v.key for v in ds.variables]
        num, key = self.pick if self.pick is not None else (None, None)
        num = num if num in nums else nums[0]
        key = key if key in keys else keys[0]
        self.pick = (num, key)
        for box, items, cur in ((self.var_box, [(v.label, v.key) for v in ds.variables], key),
                                (self.sel_box, [(f"#{s.number}", s.number) for s in ds.selections],
                                 num)):
            box.blockSignals(True)
            if [(box.itemText(i), box.itemData(i)) for i in range(box.count())] != items:
                box.clear()
                for text, data in items:
                    box.addItem(text, data)
            box.setCurrentIndex([d for _t, d in items].index(cur))
            box.blockSignals(False)
        for i, s in enumerate(ds.selections):
            self.sel_box.setItemData(i, sel_color(s.number), Qt.ItemDataRole.ForegroundRole)

    # ----- colour range (the Spec window's rules: spectral_base), per unit

    @property
    def db(self) -> bool:
        return self.db_box.isChecked()

    @property
    def unit(self) -> str:
        return self.var.unit if self.var is not None else ""

    def _range(self, unit: str) -> dict:
        return self._ranges.setdefault(unit, {"auto": True, False: _LIN_LEVELS, True: _DB_LEVELS})

    def _show_unit(self) -> None:
        st = self._range(self.unit)
        self.auto_box.blockSignals(True)
        self.auto_box.setChecked(st["auto"])
        self.auto_box.blockSignals(False)
        show_level_boxes((self.lo_box, self.hi_box),
                         self.levels if st["auto"] and self.levels else st[self.db])

    def _on_levels_edited(self, _v: float) -> None:
        st = self._range(self.unit)
        self.auto_box.blockSignals(True)
        self.auto_box.setChecked(False)              # typing a range means: use it
        self.auto_box.blockSignals(False)
        st["auto"] = False
        st[self.db] = (self.lo_box.value(), self.hi_box.value())
        self.changed.emit()

    def _on_auto(self, on: bool) -> None:
        st = self._range(self.unit)
        st["auto"] = on
        if not on:
            st[self.db] = (self.lo_box.value(), self.hi_box.value())
        self.changed.emit()

    def _on_db(self, _on: bool) -> None:
        if not self._range(self.unit)["auto"]:
            show_level_boxes((self.lo_box, self.hi_box), self._range(self.unit)[self.db])
        self.changed.emit()

    def set_levels(self, unit: str, lo_hi: Optional[tuple]) -> None:
        """A unit's fixed range (on the current scale), or None: Auto again."""
        st = self._range(unit)
        st["auto"] = lo_hi is None
        if lo_hi is not None:
            st[self.db] = tuple(lo_hi)
        if unit == self.unit:
            self._show_unit()
        self.changed.emit()

    # ----- navigation

    def nav_plots(self) -> list[pg.PlotItem]:
        return [self.plot] if self._pages.currentIndex() == 1 else []

    def reset_view(self) -> None:
        super().reset_view()

    def _click(self, vb: pg.ViewBox, ev) -> bool:
        if ev.button() != Qt.MouseButton.LeftButton or ev.double():
            return False
        ev.accept()
        self.clicked.emit(float(vb.mapSceneToView(ev.scenePos()).x()))
        return True

    def probe_text(self, x: float, y: float) -> Optional[str]:
        """t, f and the amplitude of the tile under the mouse."""
        if self.data is None:
            return None
        tc, f, amp = self.data
        col = spectrogram_column(self.data, x)
        if col is None or not len(f) or y < f[0] - 0.5 * (f[1] - f[0] if len(f) > 1 else 1) \
                or y > f[-1]:
            return None
        fi = int(np.argmin(np.abs(f - y)))
        a = col[1][fi]
        val = f"{float(to_db(np.array([a]))[0]):.1f} dB" if self.db else f"{a:.4g} {self.unit}"
        return f"t = {x:.3f} s · f = {f[fi]:.2f} Hz · {self.var.label} #{self.sel.number}: {val}"

    # ----- drawing

    def spectrogram(self, ds: Dataset):
        """The picked tile's ``Dataset.spectrogram`` (cached by its inputs)."""
        seg = SEGMENTS_S[self.seg_box.currentIndex()]
        ovl = OVERLAPS[self.ovl_box.currentIndex()]
        k = (self.sel.number, self.var.key)
        key = _input_key(ds, self.sel, self.var, seg, ovl)
        hit = self._cache.get(k)
        if hit is None or hit[0] != key:
            self.computed += 1
            hit = (key, ds.spectrogram(self.sel, self.var, seg, ovl))
            self._cache = {kk: vv for kk, vv in self._cache.items()
                           if any(kk == (s.number, v.key) for s, v in ds.pairs())}
            self._cache[k] = hit
        return hit[1]

    def render(self, ds: Dataset) -> None:
        self._sync_picker(ds)
        num, key = self.pick
        self.sel = next(s for s in ds.selections if s.number == num)
        self.var = next(v for v in ds.variables if v.key == key)
        seg = SEGMENTS_S[self.seg_box.currentIndex()]
        ovl = OVERLAPS[self.ovl_box.currentIndex()]
        st = self._range(self.unit)
        inputs = (self.pick, _input_key(ds, self.sel, self.var, seg, ovl), self.db,
                  st["auto"], None if st["auto"] else st[self.db])
        if inputs == self._last:
            return
        self._last = inputs
        self.renders += 1
        self._pages.setCurrentIndex(1)
        self.data = g = self.spectrogram(ds)
        if g is None:
            self.img.clear()
            self.levels = None
            self.plot.setTitle(f"{self.var.label}  #{self.sel.number}: shorter than one segment",
                               color=sel_color(self.sel.number).name(), size="9pt")
            self._show_unit()
            return
        tc, f, amp = g
        v = to_db(amp) if self.db else amp
        self.levels = auto_levels(v, self.db, st[self.db]) if st["auto"] else st[self.db]
        self._show_unit()
        dt = tc[1] - tc[0] if len(tc) > 1 else seg
        df = f[1] - f[0] if len(f) > 1 else 1.0
        rect = pg.QtCore.QRectF(tc[0] - dt / 2, -df / 2, dt * len(tc), df * len(f))
        self.img.setImage(v, autoLevels=False, levels=self.levels, rect=rect)
        self.plot.setTitle(f"{self.var.label}  #{self.sel.number}", color=sel_color(self.sel.number).name(),
                           size="9pt")
        self.plot.setYRange(0.0, float(rect.bottom()), padding=0)
        self.cbar.setLevels(self.levels)
        self.cbar.axis.setLabel(f"dB re 1 {self.unit}" if self.db else self.unit)
        self.glw.scene().prepareForPaint()


class CursorCell(_Cell):
    """The picked tile's spectrum at each cursor (see the module doc)."""

    TITLE = "At the cursors"

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.title = self._add_title("Spectrum at the cursors")
        self.plot = self.glw.addPlot(row=0, col=0)
        self.plot.setMenuEnabled(False)
        self.plot.hideButtons()
        _show_grid(self.plot)
        self.plot.getAxis("left").enableAutoSIPrefix(False)
        self.plot.setLabel("bottom", "Frequency", units="Hz")
        self.plot.addLegend(offset=(-10, 5), labelTextColor="k",
                            brush=pg.mkBrush(255, 255, 255, 210), pen=pg.mkPen(180, 180, 180))
        self.plot.getViewBox().sigRangeChangedManually.connect(lambda *_: self.on_manual())
        self.probe = probe_line(self.plot)
        self.curves: dict[int, pg.PlotCurveItem] = {}    # cursor number -> curve
        self.columns: dict[int, tuple] = {}              # cursor number -> (f, amp, centre)
        self._auto = True
        self._unit = ""
        self._legend_nums: list = []

    def nav_plots(self) -> list[pg.PlotItem]:
        return [self.plot] if self._pages.currentIndex() == 1 else []

    def on_manual(self) -> None:
        self._auto = False

    def reset_view(self) -> None:
        super().reset_view()
        self._auto = True
        self.plot.enableAutoRange(y=True)

    def probe_items(self, plot: pg.PlotItem):
        if plot is not self.plot:
            return None
        items = [(f"cursor {n}", self._unit, c) for n, c in self.curves.items()]
        return "f", "Hz", (lambda x: x), (lambda y: y), items

    def render(self, spec: SpecgramCell, cursors) -> None:
        """``spec``: the spectrogram cell (its tile and dB); ``cursors``: [(number, t)]."""
        if spec.data is None or not cursors:
            self.show_hint("The spectrogram tile is shorter than one segment." if spec.data is None
                           else "Click in the spectrogram or the waveform to add a cursor: the"
                           " spectrogram's column there shows here, one curve per cursor.")
            for c in self.curves.values():
                self.plot.removeItem(c)
            self.curves, self.columns = {}, {}
            self.plot.legend.clear()
            self._legend_nums = []
            return
        inputs = (id(spec.data), spec.db, tuple(cursors), self._auto)
        if inputs == self._last:
            return
        self._last = inputs
        self.renders += 1
        self._pages.setCurrentIndex(1)
        self._unit = f"dB re 1 {spec.unit}" if spec.db else spec.unit
        self.plot.setLabel("left", self._unit)
        self.title.setText(f"Spectrum at the cursors: <span style='color:"
                           f"{sel_color(spec.sel.number).name()}'>{spec.var.label}  #{spec.sel.number}</span>")
        nums = {n for n, _t in cursors}
        for n in [n for n in self.curves if n not in nums]:
            self.plot.removeItem(self.curves.pop(n))
        self.columns = {}
        relabel = sorted(nums) == self._legend_nums      # a drag: same cursors, new labels
        if not relabel:
            self.plot.legend.clear()
            self._legend_nums = sorted(nums)
        labels = {}
        for n, t in sorted(cursors):
            col = spectrogram_column(spec.data, t)
            c = self.curves.get(n)
            if c is None:
                c = self.curves[n] = pg.PlotCurveItem(pen=pg.mkPen(cursor_color(n), width=1.5))
                self.plot.addItem(c)
            if col is None:
                c.setData([], [])
                labels[n] = (c, f"{n}  t = {t:.2f} s: outside the tile")
                continue
            self.columns[n] = col
            f, a, centre = col
            c.setData(f, to_db(a) if spec.db else a)
            labels[n] = (c, f"{n}  t = {t:.2f} s (segment at {centre:.2f} s)")
        if relabel:
            for (_sample, lab), n in zip(self.plot.legend.items, sorted(labels)):
                if lab.text != labels[n][1]:
                    lab.setText(labels[n][1])
        else:
            for n in sorted(labels):
                self.plot.legend.addItem(*labels[n])
        if self._auto:
            f = spec.data[1]
            if len(f):
                self.plot.setXRange(0.0, float(f[-1]), padding=0.01)
            self.plot.enableAutoRange(y=True)
        self.glw.scene().prepareForPaint()
