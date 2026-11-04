"""The Analysis window's tabs: Compare, Spectrum, Spectrogram.

Each tab draws a ``daq/analysis.Dataset`` (the window's included selections x
checked variables, its Compare sub-range, the panel's Despike / V0 / scale)
handed to ``render``; the window decides when (``views/analysis_window.py``).

Encoding (all tabs): colour is the selection (its region's colour); the
variables in a Compare / Spectrum row are told apart by the shade of it, dark
to light in the order they are listed (``shade_levels``). The legends: each
row's variables in their shade (grey), the top row's also the selections in
their colour. Rows are unit groups (mV, mg, mrad/s, deg, cm).

- Compare (``CompareTab``): relative time, min / max decimated to the plot width
  (``Dataset.traces``). With the window's Select on, a drag makes the sub-range
  (a shaded band in every row, edges draggable); Spectrum and Spectrogram then
  use that part of every included selection.
- Spectrum (``SpectrumTab``): ``Dataset.spectrum`` per (variable, selection).
  Options in one row: window, detrend, log f, log amplitude, f range. A trace is
  recomputed only when its inputs change (the selection's span, the options,
  Despike, that gauge's V0 / scale): an edge drag recomputes the dragged
  selection's traces only, and a render with nothing changed does nothing
  (``computed`` / ``renders`` count it, for the tests).
- Spectrogram (``SpectrogramTab``): one tile per (variable, selection), rows =
  variables, columns = selections, relative time; one colour range for the
  tiles of a unit group (mV, mg, ...: a colour bar each), chosen as in the Spec
  window (Auto: 1st to 99.5th percentile of that unit's tiles; or the min / max
  boxes, kept per lin / dB; ``spectral_base``); the Colour box picks the unit
  the controls set. At most ``MAX_TILES``
  tiles and ``MAX_COLS`` columns: variables are cut first, with a note.
"""

from __future__ import annotations

from typing import Callable, Optional

import numpy as np
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QHBoxLayout, QLabel,
                             QStackedWidget, QVBoxLayout, QWidget)

import pyqtgraph as pg

from wavi_py.daq.analysis import UNIT_GROUPS, Dataset
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
MAX_TILES, MAX_COLS = 24, 6                    # spectrogram grid cap
SEGMENTS_S = (0.25, 0.5, 1.0, 2.0, 5.0)
OVERLAPS = (0.0, 0.5, 0.75, 0.9)


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
    """The header of the spectral tabs: what part of the selections they use."""
    if sub is None:
        return "Whole selections (Compare: Select (S) and drag for a sub-range)"
    return (f"Sub-range {sub[0]:.2f} .. {sub[1]:.2f} s from each selection's start, clipped to"
            " its length (Compare: Esc clears)")


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


class _Header(QWidget):
    """An options row (spectral tabs) over the sub-range line."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        v = QVBoxLayout(self)
        v.setContentsMargins(4, 2, 4, 0)
        v.setSpacing(1)
        self.row = QHBoxLayout()
        self.row.setSpacing(6)
        v.addLayout(self.row)
        self.sub = QLabel(sub_text(None))
        self.sub.setStyleSheet("color: #555;")
        v.addWidget(self.sub)

    def add(self, label: str, w: QWidget) -> QWidget:
        if label:
            self.row.addWidget(QLabel(label))
        self.row.addWidget(w)
        return w


class _RowsTab(QWidget):
    """Stacked rows by unit with a hint page; curves keyed (selection, variable)."""

    X_LABEL = ("", "")
    changed = pyqtSignal()                       # an option: redraw

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
        self.glw = GraphicsLayoutWidget()
        self.glw.setBackground("w")
        self._pages.addWidget(self._hint)
        self._pages.addWidget(self.glw)
        self.rows: dict[str, pg.PlotItem] = {}           # unit -> plot
        self.curves: dict[tuple, pg.PlotCurveItem] = {}  # (selection number, var key) -> curve
        self._legend_key: tuple = ()
        self._auto = True                                # X follows the data

    def _add_header(self) -> _Header:
        self.header = _Header()
        self._lay.insertWidget(0, self.header)
        return self.header

    def set_sub_text(self, sub) -> None:
        if self.header is not None:
            self.header.sub.setText(sub_text(sub))

    def reset_view(self) -> None:
        self._auto = True
        for p in self.rows.values():
            p.enableAutoRange(y=True)

    def _rebuild_rows(self, units: list[str]) -> None:
        self.glw.clear()
        self.rows, self.curves, self._legend_key = {}, {}, ()
        first = None
        for i, u in enumerate(units):
            p = self.glw.addPlot(row=i, col=0)
            p.setLabel("left", u)
            _show_grid(p)
            p.getAxis("left").enableAutoSIPrefix(False)
            p.getAxis("left").setWidth(56)                # the rows' plots line up
            p.setMenuEnabled(False)
            if first is None:
                first = p
            else:
                p.setXLink(first)
            p.getViewBox().sigRangeChangedManually.connect(self._on_manual)
            self.rows[u] = p
            self._row_added(p)
        if units:
            self.rows[units[-1]].setLabel("bottom", self.X_LABEL[0], units=self.X_LABEL[1])

    def _row_added(self, p: pg.PlotItem) -> None:
        """Hook: a row was made."""

    def _on_manual(self, *_a) -> None:
        self._auto = False

    def show_hint(self, text: str) -> None:
        self._hint.setText(text)
        self._pages.setCurrentIndex(0)

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
            c.unit, c.look = var.unit, None
            self.rows[var.unit].addItem(c)
            self.curves[key] = c
        look = (sel.number, lightness)
        if c.look != look:                  # the shade follows the checked order
            c.setPen(pg.mkPen(shade(sel_color(look[0]), look[1]), width=1))
            c.look = look
        return c

    def _drop(self, seen: set) -> None:
        for key in [k for k in self.curves if k not in seen]:
            c = self.curves.pop(key)
            self.rows[c.unit].removeItem(c)

    def _legends(self, ds: Dataset, shades: dict, units: list[str]) -> None:
        key = (tuple(v.key for v in ds.variables),
               tuple((s.number, round(s.t0, 2), round(s.t1, 2)) for s in ds.selections))
        if key == self._legend_key:
            return
        self._legend_key = key
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
                                     f"#{s.number}  {s.t0:.2f} .. {s.t1:.2f} s")


class CompareTab(_RowsTab):
    """Stacked rows by unit, relative time, and the sub-range (see the module doc).
    ``selecting()``: the window's Select is on; ``subChanged(sub or None)``."""

    X_LABEL = ("Time from the selection's start", "s")
    subChanged = pyqtSignal(object)

    def __init__(self, selecting: Callable[[], bool] = lambda: False,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._selecting = selecting
        self.sub: Optional[tuple] = None
        self.bands: list[pg.LinearRegionItem] = []
        self._drag_from: Optional[float] = None

    def _row_added(self, p: pg.PlotItem) -> None:
        _hook_drag(p.getViewBox(), self._drag)
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

    def _rebuild_rows(self, units: list[str]) -> None:
        self.bands = []
        super()._rebuild_rows(units)

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

    def render(self, ds: Dataset, shades: dict) -> None:
        units = self._units(ds)
        first = self.rows[units[0]]
        n_px = max(50, int(first.getViewBox().width()) or 600)
        seen = set()
        for tr in ds.traces(n_px):
            seen.add((tr.sel.number, tr.var.key))
            self._curve(tr.sel, tr.var, shades[tr.var.key]).setData(tr.t, tr.y)
        self._drop(seen)
        self._legends(ds, shades, units)
        if self._auto:
            longest = max((s.duration for s in ds.selections), default=0.0)
            if longest > 0:
                first.setXRange(0.0, longest, padding=0.01)
        self.glw.scene().prepareForPaint()


def _spin(lo: float, hi: float, value: float, decimals: int = 2, special: str = "") -> QDoubleSpinBox:
    b = QDoubleSpinBox()
    b.setRange(lo, hi)
    b.setDecimals(decimals)
    b.setValue(value)
    b.setKeyboardTracking(False)
    if special:
        b.setSpecialValueText(special)
    return b


class SpectrumTab(_RowsTab):
    """Amplitude spectra per (variable, selection) (see the module doc)."""

    X_LABEL = ("Frequency", "Hz")

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
        self.log_f = h.add("", QCheckBox("log f"))
        self.log_a = h.add("", QCheckBox("log amplitude"))
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
        self._last: Optional[tuple] = None            # the whole render's inputs
        self.computed = 0                             # spectra computed (tests)
        self.renders = 0                              # renders that did any work (tests)

    @property
    def options(self) -> tuple[str, str]:
        return (("hann", "rect")[self.window_box.currentIndex()],
                self.detrend_box.currentText())

    def reset_view(self) -> None:
        super().reset_view()
        self._last = None

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


class SpectrogramTab(QWidget):
    """One tile per (variable, selection) (see the module doc)."""

    changed = pyqtSignal()

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        self.header = h = _Header()
        lay.addWidget(h)
        self.seg_box = h.add("Segment", QComboBox())
        self.seg_box.addItems([f"{s:g} s" for s in SEGMENTS_S])
        self.seg_box.setCurrentIndex(SEGMENTS_S.index(1.0))
        self.seg_box.setToolTip("Segment length: frequency bins are 1 / length apart")
        self.ovl_box = h.add("Overlap", QComboBox())
        self.ovl_box.addItems([f"{int(o * 100)} %" for o in OVERLAPS])
        self.ovl_box.setCurrentIndex(OVERLAPS.index(0.75))
        self.db_box = h.add("", QCheckBox("dB"))
        self.db_box.setToolTip("Colour by amplitude in dB re 1 unit instead of linear")
        self.unit_box = h.add("Colour", QComboBox())
        self.unit_box.setToolTip("The unit group whose colour range Auto / min / max set (each"
                                 " unit has its own range and colour bar)")
        self.auto_box = h.add("", QCheckBox("Auto"))
        self.auto_box.setChecked(True)
        self.auto_box.setToolTip("Colour range of this unit from the data (1st to 99.5th"
                                 " percentile of its tiles); off: the min / max boxes")
        self.lo_box = h.add("min", _spin(-1e6, 1e6, _LIN_LEVELS[0]))
        self.hi_box = h.add("max", _spin(-1e6, 1e6, _LIN_LEVELS[1]))
        h.row.addStretch(1)
        # per unit group: auto, and the fixed range of each scale (lin False, dB True)
        self._ranges: dict[str, dict] = {}
        for w in (self.seg_box, self.ovl_box):
            w.currentIndexChanged.connect(lambda _i: self.changed.emit())
        self.unit_box.currentIndexChanged.connect(self._on_unit)
        self.db_box.toggled.connect(self._on_db)
        self.auto_box.toggled.connect(self._on_auto)
        for b in (self.lo_box, self.hi_box):
            b.valueChanged.connect(self._on_levels_edited)
        self.note = QLabel()
        self.note.setStyleSheet("color: #a05000;")
        self.note.hide()
        h.layout().addWidget(self.note)
        self._pages = QStackedWidget()
        lay.addWidget(self._pages, 1)
        self._hint = QLabel()
        self._hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._hint.setWordWrap(True)
        self.glw = GraphicsLayoutWidget()
        self.glw.setBackground("w")
        self._pages.addWidget(self._hint)
        self._pages.addWidget(self.glw)
        self._cmap = pg.colormap.get("viridis")
        self.tiles: dict[tuple, tuple] = {}           # (sel number, var key) -> (plot, image)
        self._layout_key: tuple = ()
        self.cbars: dict[str, pg.ColorBarItem] = {}   # unit -> its colour bar
        self.levels: dict[str, tuple] = {}            # unit -> the range in use
        self._cache: dict[tuple, tuple] = {}
        self._last: Optional[tuple] = None
        self.grid = (0, 0)                            # (rows, cols) shown
        self.computed = 0
        self.renders = 0

    # ----- colour range (the Spec window's rules: spectral_base)

    @property
    def db(self) -> bool:
        return self.db_box.isChecked()

    def _range(self, unit: str) -> dict:
        return self._ranges.setdefault(unit, {"auto": True, False: _LIN_LEVELS, True: _DB_LEVELS})

    @property
    def unit(self) -> str:
        """The unit group the Auto / min / max controls are showing."""
        return self.unit_box.currentText()

    def _show_unit(self) -> None:
        """The controls show the chosen unit's state."""
        st = self._range(self.unit)
        self.auto_box.blockSignals(True)
        self.auto_box.setChecked(st["auto"])
        self.auto_box.blockSignals(False)
        show_level_boxes((self.lo_box, self.hi_box),
                         self.levels.get(self.unit, st[self.db]) if st["auto"] else st[self.db])

    def _on_unit(self, _i: int) -> None:
        if self.unit:
            self._show_unit()
            self.changed.emit()

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
        if self.unit and not self._range(self.unit)["auto"]:
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

    def set_sub_text(self, sub) -> None:
        self.header.sub.setText(sub_text(sub))

    def show_hint(self, text: str) -> None:
        self._hint.setText(text)
        self._pages.setCurrentIndex(0)

    def reset_view(self) -> None:
        self._last = None
        for p, _img in self.tiles.values():
            p.enableAutoRange()

    # ----- grid

    def _rebuild(self, sels: list, vars_: list) -> None:
        self.glw.clear()
        self.tiles = {}
        first = None
        for r, var in enumerate(vars_):
            row_first = None
            for c, sel in enumerate(sels):
                p = self.glw.addPlot(row=r, col=c)
                p.setMenuEnabled(False)
                p.hideButtons()
                p.setTitle(f"{var.label}  #{sel.number}", color=sel_color(sel.number).name(),
                           size="9pt")
                img = pg.ImageItem()
                img.setColorMap(self._cmap)
                p.addItem(img)
                if first is None:
                    first = p
                else:
                    p.setXLink(first)
                if row_first is None:
                    row_first = p
                else:
                    p.setYLink(row_first)
                if c == 0:
                    p.setLabel("left", "Hz")
                if r == len(vars_) - 1:
                    p.setLabel("bottom", "Time from the selection's start", units="s")
                self.tiles[(sel.number, var.key)] = (p, img)
        # a colour bar per unit group, beside its rows (the tree lists a unit's
        # variables together, so they are consecutive rows)
        self.cbars = {}
        for u in dict.fromkeys(v.unit for v in vars_):
            rows = [r for r, v in enumerate(vars_) if v.unit == u]
            bar = pg.ColorBarItem(colorMap=self._cmap, interactive=False, width=14)
            self.glw.addItem(bar, row=rows[0], col=len(sels), rowspan=len(rows))
            self.cbars[u] = bar

    def spectrograms(self, ds: Dataset, sels: list, vars_: list) -> dict:
        seg = SEGMENTS_S[self.seg_box.currentIndex()]
        ovl = OVERLAPS[self.ovl_box.currentIndex()]
        out, keep = {}, {}
        for sel in sels:
            for var in vars_:
                k = (sel.number, var.key)
                key = _input_key(ds, sel, var, seg, ovl)
                hit = self._cache.get(k)
                if hit is None or hit[0] != key:
                    self.computed += 1
                    hit = (key, ds.spectrogram(sel, var, seg, ovl))
                keep[k] = hit
                out[k] = hit[1]
        self._cache = keep
        return out

    def render(self, ds: Dataset) -> None:
        sels = ds.selections[:MAX_COLS]
        nrows = max(1, MAX_TILES // max(1, len(sels)))
        vars_ = ds.variables[:nrows]
        cut = []
        if len(ds.variables) > len(vars_):
            cut.append(f"{len(vars_)} of {len(ds.variables)} variables")
        if len(ds.selections) > len(sels):
            cut.append(f"{len(sels)} of {len(ds.selections)} selections")
        self.note.setText(f"Showing {' and '.join(cut)} (at most {MAX_TILES} tiles,"
                          f" {MAX_COLS} columns): uncheck some to see the rest" if cut else "")
        self.note.setVisible(bool(cut))
        seg = SEGMENTS_S[self.seg_box.currentIndex()]
        ovl = OVERLAPS[self.ovl_box.currentIndex()]
        units = list(dict.fromkeys(v.unit for v in vars_))
        if units != [self.unit_box.itemText(i) for i in range(self.unit_box.count())]:
            keep = self.unit
            self.unit_box.blockSignals(True)
            self.unit_box.clear()
            self.unit_box.addItems(units)
            self.unit_box.setCurrentIndex(units.index(keep) if keep in units else 0)
            self.unit_box.blockSignals(False)
            self._show_unit()
        levels_in = (self.db, self.unit, tuple(
            (u, self._range(u)["auto"], None if self._range(u)["auto"] else self._range(u)[self.db])
            for u in units))
        inputs = (tuple((s.number, v.key, _input_key(ds, s, v, seg, ovl)) for s in sels for v in vars_),
                  levels_in)
        if inputs == self._last:
            return
        self._last = inputs
        self.renders += 1
        self._pages.setCurrentIndex(1)
        layout = (tuple(s.number for s in sels), tuple(v.key for v in vars_))
        if layout != self._layout_key:
            self._rebuild(sels, vars_)
            self._layout_key = layout
        self.grid = (len(vars_), len(sels))
        data = self.spectrograms(ds, sels, vars_)
        vals = {k: (None if g is None else (g, to_db(g[2]) if self.db else g[2]))
                for k, g in data.items()}
        unit_of = {v.key: v.unit for v in vars_}
        self.levels = {}
        for u in units:                               # each unit group its own range
            st = self._range(u)
            if st["auto"]:
                vs = [g[1].ravel() for k, g in vals.items() if g is not None and unit_of[k[1]] == u]
                self.levels[u] = auto_levels(np.concatenate(vs) if vs else np.empty(0), self.db,
                                             st[self.db])
            else:
                self.levels[u] = st[self.db]
        if self.unit in self.levels and self._range(self.unit)["auto"]:
            show_level_boxes((self.lo_box, self.hi_box), self.levels[self.unit])
        t_end, f_top = 0.0, {}
        for k, (p, img) in self.tiles.items():
            got = vals.get(k)
            if got is None:
                img.clear()
                continue
            (tc, f, _amp), v = got
            dt = tc[1] - tc[0] if len(tc) > 1 else 1.0
            df = f[1] - f[0] if len(f) > 1 else 1.0
            rect = pg.QtCore.QRectF(tc[0] - dt / 2, -df / 2, dt * len(tc), df * len(f))
            img.setImage(v, autoLevels=False, levels=self.levels[unit_of[k[1]]], rect=rect)
            t_end = max(t_end, float(rect.right()))
            f_top[k[1]] = max(f_top.get(k[1], 0.0), float(rect.bottom()))
        # the ranges set, as the Spec window does (an image's auto-range lags its rect):
        # X from 0 to the longest span's end, Y per variable up to its Nyquist
        if t_end > 0:
            for (num, key), (p, _img) in self.tiles.items():
                p.setXRange(0.0, t_end, padding=0)
                if key in f_top:
                    p.setYRange(0.0, f_top[key], padding=0)
        for u, bar in self.cbars.items():
            if u in self.levels:
                bar.setLevels(self.levels[u])
                bar.axis.setLabel(f"dB re 1 {u}" if self.db else u)
        self.glw.scene().prepareForPaint()
