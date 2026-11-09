"""The Analysis window's 2x2 cells and its cursors.

Each cell draws a ``daq/analysis.Dataset`` (the window's included selections x
the variables checked for Plots, its sub-range, the panel's Despike / V0 /
scale) handed to ``render``; the window decides when (``views/analysis_window.py``)
and every cell does no work when its inputs did not change (``renders`` /
``computed`` count it, for the tests).

Encoding (waveform and spectrum), by the window's Colour by:

- selection (default): colour is the selection (its region's colour); the
  variables in a row are told apart by the shade of it, dark to light in the
  order they are listed (``shade_levels``);
- sensor: each gauge has its fixed colour (``sensor_grid.gauge_color``, by its
  index: it keeps it whatever else is shown), the other variables a fixed dark
  colour by their place in their row (``OTHER_COLORS``), and the selections are
  told apart by the line style, solid, dash, dot, dash-dot in their order
  (``LINE_STYLES``).

The legends: the gauges' row (mV) has the plane-map grid in miniature
(``sensor_grid.GridLegend``), the plotted gauges lit in their shade (grey) or
their colour; the other rows list their variables; the top row's also lists
the selections in their colour (or line style). Rows are unit groups (mV, mg,
mrad/s, deg, cm, and the ROV's).

- ``WaveCell`` (top left, the former Compare tab): relative time, min / max
  decimated to the plot width (``Dataset.traces``): following the data, each
  whole selection; zoomed or panned by hand, only the span shown plus one width
  each side, re-read when the view leaves it or zooms ``RELOAD_ZOOM`` past it
  (``_view``), so a zoom gets the detail back down to the raw samples. F fits
  X back to the whole selections. With the window's Select on,
  a drag makes another sub-range (``SubBand``: a hatched band, numbered s1, s2,
  ... in every row, a different hatch each; its edges and its body drag,
  right-click or Del deletes it; the cell owns them: ``subs``,
  ``subsChanged``). A
  click on a trace (within ``PICK_PX``) picks that (variable, selection) for the
  spectrogram, which the window then marks (``set_picked``: drawn thick, on
  top); elsewhere nothing (cursors are the spectrogram's). Its time
  range is copied to and from the spectrogram (``xChanged`` / ``set_x``: the
  same span, whatever the two widths).
- ``SpectrumCell`` (top right, the former Spectrum tab): ``Dataset.spectrum``
  per (variable, selection, sub-range), of the whole selections or each
  sub-range (its own trace, told apart by line style; ``SUB_STYLES``). Options:
  window, detrend, log f, log amplitude (on by default), f range (0 .. the highest Nyquist until
  edited). A trace is recomputed only when its inputs change (the selection's
  span, the options, Despike, that gauge's V0 / scale).
- ``SpecgramCell`` (bottom left): ONE tile, the picked (variable, selection,
  sub-range), full cell size, relative time; the picker is three combo boxes in
  its header; the tile is the whole selection, its sub-ranges highlighted
  (``SpecBand``: a light veil, white dashed edges, the number).
  Colour range per unit group as in the Spec window (Auto: 1st to 99.5th
  percentile of the tile; or the min / max boxes, kept per unit and per lin /
  dB; ``spectral_base``): the controls show the picked variable's unit, so
  switching to accel and back keeps the gauges' range. A horizontal colour bar
  on top keeps the plot as wide as the waveform's rows.
- ``CursorCell`` (bottom right): the picked tile's spectrogram column at each
  cursor (``spectrogram_column``), one curve per cursor in the cursor's colour,
  legend "t=12.34 s (segment at ...)"; dB follows the spectrogram's dB box.

Cursors are vertical lines at a relative time, labelled with that time
(``t=12.34 s`` from the selection's start, ``cursor_text``; the label follows a
drag; the colour, kept per cursor, tells them apart) (``CursorLine``, kept
in sync per plot by ``CursorLines``), drawn in every waveform row and in the
spectrogram: drag to move, right-click to delete, Del deletes the hovered one.
Their colours (``CURSOR_COLORS``: saturated red, green, azure, magenta, black)
are none of the selections' muted ones, and solid (no dash styles); in the
spectrogram they get a white outline to stand out on its colours.

Every cell has a header with an expand and a collapse button at its right
(``expandRequested``: the cell takes the whole 2x2, again: back;
``collapseRequested``; ``TITLE`` names it on the window's restore bar).

Drag tools (the window's exclusive Select / Zoom box, ``views/plot_tools.py``):
``set_tool`` puts the cell's plots in the tool's mouse mode and cursor, for the
tools the cell takes (``TOOLS``: the waveform Select and Zoom box, the timeline
Select, the spectra and the spectrogram Zoom box); rows made later follow.

Navigation (the window's keys and Probe) goes through ``nav_plots`` (the plots
a Y key may act on), ``x_plot`` (the one X keys scale), ``on_manual`` (a key
zoomed: stop following the data) and ``probe_items(plot)`` (the curves a
hover reads, with the axis conversions).
"""

from __future__ import annotations

from typing import Callable, Optional

import numpy as np
from PyQt6.QtCore import QPointF, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QBrush, QColor
from PyQt6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QHBoxLayout, QLabel,
                             QSizePolicy, QStackedWidget, QToolButton, QVBoxLayout, QWidget)

import pyqtgraph as pg

from wavi_py.daq.analysis import UNIT_GROUPS, Dataset, spectrogram_column
from wavi_py.daq.series import WSK
from wavi_py.views.plot_tools import PAN, SELECT, ZOOM, apply_tool, effective, zoom_cursor  # noqa: F401
from wavi_py.views.render_sched import GraphicsLayoutWidget
from wavi_py.views.selections import sel_color
from wavi_py.views.sensor_grid import GridLegend, gauge_color
from wavi_py.views.spectral_base import (_DB_LEVELS, _LIN_LEVELS, auto_levels, show_level_boxes,
                                         to_db)
from wavi_py.views.whisker_plots import _show_grid

# a row's variables: HSL lightness of the selection's colour, dark to light (one
# variable keeps the colour itself); the light end stays readable as a 1 px line on white
_L_DARK, _L_LIGHT = 0.18, 0.66
_SUB_PEN = pg.mkPen((60, 60, 60), width=1, style=Qt.PenStyle.DashLine)
# sub-ranges: hatched (the selections are flat colours), one hatch each
SUB_HATCH = (Qt.BrushStyle.BDiagPattern, Qt.BrushStyle.FDiagPattern,
             Qt.BrushStyle.DiagCrossPattern, Qt.BrushStyle.CrossPattern)
_PROBE_PEN = pg.mkPen((90, 90, 90), width=1)
AXIS_W = 56                                    # left axes: the cells' plots line up
SEGMENTS_S = (0.25, 0.5, 1.0, 2.0, 5.0)
OVERLAPS = (0.0, 0.5, 0.75, 0.9)
PICK_PX = 5                                    # a click this near a trace picks it
# cursors: saturated, none of the selections' colours (SEL_COLORS) nor the grey sub-range
CURSOR_COLORS = ((215, 25, 28), (0, 150, 0), (0, 110, 255), (210, 0, 210), (0, 0, 0))
SELECTION, SENSOR = "selection", "sensor"            # the window's Colour by
LINE_STYLES = (Qt.PenStyle.SolidLine, Qt.PenStyle.DashLine, Qt.PenStyle.DotLine,
               Qt.PenStyle.DashDotLine)
# Colour by sensor: the non-gauge variables of a row, by their place in it
OTHER_COLORS = ((40, 90, 170), (70, 70, 70), (20, 120, 60), (150, 30, 30), (100, 60, 150),
                (0, 110, 120), (120, 110, 0), (190, 80, 150))


def cursor_color(number: int) -> QColor:
    return QColor(*CURSOR_COLORS[(number - 1) % len(CURSOR_COLORS)])


CURSOR_FMT = "t={value:.2f} s"


def cursor_text(t: float) -> str:
    """A cursor's label and legend name: its time from the selection's start."""
    return CURSOR_FMT.format(value=t)


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


def line_style(i: int) -> Qt.PenStyle:
    """The ``i``-th selection's line style (Colour by sensor)."""
    return LINE_STYLES[i % len(LINE_STYLES)]


def sub_name(n: int) -> str:
    return f"s{n}"


def sub_text(sub) -> str:
    """The spectral cells' header: what part of the selections they use (``sub``:
    None, one (a, b), or the numbered ((n, a, b), ...))."""
    if not sub:
        return "Whole selections (waveform: Select (S) and drag for a sub-range)"
    if not isinstance(sub[0], (tuple, list)):
        sub = [(1, *sub)]
    if len(sub) == 1:
        n, a, b = sub[0]
        return (f"Sub-range {sub_name(n)} {a:.2f} .. {b:.2f} s from each selection's start,"
                " clipped to its length (Esc clears)")
    parts = ", ".join(f"{sub_name(n)} {a:.2f} .. {b:.2f}" for n, a, b in sub)
    return f"Sub-ranges {parts} s from each selection's start, clipped (Esc clears)"


def sub_style(i: int) -> Qt.PenStyle:
    """The ``i``-th sub-range's line style in the spectrum (Colour by selection)."""
    return LINE_STYLES[i % len(LINE_STYLES)]


def _input_key(ds: Dataset, sel, var, *opts, sub=None) -> tuple:
    """Everything a spectrum / spectrogram of (sel, var, sub-range) depends on."""
    v0 = (float(ds.v0[var.col]) if var.source == WSK and ds.v0 is not None
          and var.col < len(ds.v0) else 0.0)
    gauge = var.source in (WSK, "pos")
    return (ds.span(sel, sub), id(sel.source), ds.despike and gauge, v0,
            ds.scale if gauge else 1.0, getattr(ds, "baseline", False), ds.gauge(sel, var)
            if var.source == "pos" else None, *opts)


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
                         hoverPen=pg.mkPen(c, width=4), label=CURSOR_FMT,
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
                got[1].label.valueChanged()          # its time label (the signal was blocked)


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
        # the cell's expand and collapse buttons, top right (the window restores it from its bar)
        self.expand_btn = QToolButton()
        self.expand_btn.setText("□")
        self.expand_btn.setAutoRaise(True)
        self.expand_btn.setCheckable(True)
        self.expand_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.expand_btn.setToolTip("Expand: this cell takes the whole 2x2 (again, or the bar"
                                   " above the plots: back to the layout before)")
        outer.addWidget(self.expand_btn, 0, Qt.AlignmentFlag.AlignTop)
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
    expandRequested = pyqtSignal()               # its header's expand button
    TITLE = ""                                   # the window's restore bar names it so
    TOOLS: tuple = (ZOOM,)                       # the drag tools it takes (set_tool)

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
        self.tool = PAN                              # the window's drag tool (set_tool)

    def _add_header(self, sub_line: bool = True) -> _Header:
        self.header = _Header(sub_line=sub_line)
        self.header.collapse_btn.clicked.connect(self.collapseRequested.emit)
        self.header.expand_btn.clicked.connect(lambda _on: self.expandRequested.emit())
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

    def set_tool(self, tool: str) -> None:
        """The window's drag tool (``views/plot_tools.py``); one this cell does not
        take (``TOOLS``) leaves it panning, with the arrow."""
        self.tool = tool
        apply_tool(tool, [p.getViewBox() for p in self._zoom_plots()], [self.glw], self.TOOLS)

    @property
    def zoom_box(self) -> bool:
        return effective(self.tool, self.TOOLS) == ZOOM

    def _apply_tool(self, p: pg.PlotItem) -> None:
        """A plot made after ``set_tool``: in step with it."""
        apply_tool(self.tool, [p.getViewBox()], (), self.TOOLS)

    def _zoom_plots(self) -> list[pg.PlotItem]:
        return self.nav_plots()

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
        self.color_by = SELECTION
        self.grid = None                                 # the sensors' GridLayout (legends)
        self._grid_legend: Optional[GridLegend] = None
        self._order: dict = {}                           # var key -> its place in its row
        self._sel_index: dict = {}                       # sel number -> its place

    def nav_plots(self) -> list[pg.PlotItem]:
        return list(self.rows.values()) if self._pages.currentIndex() == 1 else []

    def _zoom_plots(self) -> list[pg.PlotItem]:
        return list(self.rows.values())

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
        self._grid_legend = None
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
            self._apply_tool(p)
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

    def _index(self, ds: Dataset) -> None:
        """Each variable's place in its row and each selection's place (the styles)."""
        self._order, n = {}, {}
        for v in ds.variables:
            self._order[v.key] = n.get(v.unit, 0)
            n[v.unit] = self._order[v.key] + 1
        self._sel_index = {s.number: i for i, s in enumerate(ds.selections)}

    def var_color(self, var, lightness=None) -> QColor:
        """A variable's colour by sensor (a gauge's own; a position's, the colour of
        its half in the grid's position view; else by its place in its row)."""
        if var.source == WSK:
            return gauge_color(var.col)
        if var.source == "pos" and self.grid is not None and var.pos in self.grid.names:
            return gauge_color(2 * self.grid.names.index(var.pos) + (var.axis == "lift"))
        return QColor(*OTHER_COLORS[self._order.get(var.key, 0) % len(OTHER_COLORS)])

    def _pen_of(self, number: int, var, lightness, style_i: Optional[int] = None) -> tuple:
        """(colour, line style) of a trace (see the module doc)."""
        if self.color_by == SENSOR:
            i = self._sel_index.get(number, 0) if style_i is None else style_i
            return self.var_color(var), line_style(i)
        return shade(sel_color(number), lightness), (Qt.PenStyle.SolidLine if style_i is None
                                                      else line_style(style_i))

    def _curve(self, sel, var, lightness, key: Optional[tuple] = None,
               style_i: Optional[int] = None, label: str = "") -> pg.PlotCurveItem:
        key = key or (sel.number, var.key)
        c = self.curves.get(key)
        if c is None:
            c = pg.PlotCurveItem(connect="finite")
            c.unit, c.look, c.label = var.unit, None, label or f"{var.label} #{sel.number}"
            self.rows[var.unit].addItem(c)
            self.curves[key] = c
        color, style = self._pen_of(sel.number, var, lightness, style_i)
        self._style(key, c, (color.rgba(), style))
        return c

    def _style(self, key: tuple, c: pg.PlotCurveItem, pen: tuple) -> None:
        """The pen ``(rgba, style)``; the picked trace (waveform) thicker and on top."""
        look = (pen, key[:2] == self.picked)
        if c.look != look:
            c.setPen(pg.mkPen(QColor.fromRgba(pen[0]), width=2.5 if look[1] else 1,
                              style=pen[1]))
            c.setZValue(5 if look[1] else 0)
            c.look = look

    def _drop(self, seen: set) -> None:
        for key in [k for k in self.curves if k not in seen]:
            c = self.curves.pop(key)
            self.rows[c.unit].removeItem(c)

    @staticmethod
    def _sel_text(s) -> str:
        return f"#{s.number}  {s.t0:.2f} .. {s.t1:.2f} s"

    def _grid_col(self, var) -> Optional[int]:
        """The grid half a variable lights in the grid legend: a gauge its own, a
        position the half of its cell (drag left, lift right), whichever the
        picker's view; None: not on the grid."""
        if var.source == WSK:
            return var.col
        if var.source == "pos" and self.grid is not None and var.pos in self.grid.names:
            return 2 * self.grid.names.index(var.pos) + (var.axis == "lift")
        return None

    def _legend_color(self, var, shades: dict) -> QColor:
        """A variable's legend colour: its shade in grey (by selection), else its own."""
        if self.color_by == SENSOR:
            return self.var_color(var)
        return shade(QColor(128, 128, 128), shades[var.key])

    def _legends(self, ds: Dataset, shades: dict, units: list[str], subs: tuple = ()) -> None:
        """Rebuilt when the variables, the selections, the sub-ranges (``subs``: the
        spectrum's) or the colouring change; an edge drag only re-labels (a rebuild
        per drag step would cost more than the curves). The gauges' row has the grid
        legend instead of their names."""
        key = (tuple(v.key for v in ds.variables), tuple(s.number for s in ds.selections),
               tuple(units), self.color_by, id(self.grid), tuple(u[0] for u in subs),
               tuple(sorted((k, v) for k, v in shades.items() if v is not None)))
        if key == self._legend_key:
            for lab, text_of in self._sel_labels.values():
                text = text_of(ds)
                if lab.text != text:
                    lab.setText(text)
            return
        self._legend_key = key
        self._sel_labels = {}
        gauges = [v for v in ds.variables if self._grid_col(v) is not None]
        for i, u in enumerate(units):
            p = self.rows[u]
            if p.legend is None:
                p.addLegend(offset=(-10, 5), labelTextColor="k",
                            brush=pg.mkBrush(255, 255, 255, 210), pen=pg.mkPen(180, 180, 180))
            p.legend.clear()
            grid_here = u == "mV" and self.grid is not None and gauges
            gl_w = 0.0
            if grid_here:
                gl = self._grid_legend
                if gl is None or gl.grid is not self.grid or gl.parentItem() is not p.getViewBox():
                    if gl is not None and gl.scene() is not None:
                        gl.scene().removeItem(gl)
                    gl = self._grid_legend = GridLegend(self.grid)
                    gl.setParentItem(p.getViewBox())
                    gl.anchor(itemPos=(1, 0), parentPos=(1, 0), offset=(-10, 5))
                gl.set_lit({self._grid_col(v): self._legend_color(v, shades) for v in gauges})
                gl.setToolTip("\n".join(v.label for v in gauges))
                gl_w = gl.width() + 6
            # no per-variable legend: hovering a trace names it (views/hover_probe.py)
            if i == 0:
                self._top_legend(p, ds, subs)
            p.legend.setOffset((-10 - gl_w, 5))
            p.legend.setVisible(bool(p.legend.items))
        if self._grid_legend is not None and not (self.grid is not None and gauges and "mV" in units):
            if self._grid_legend.scene() is not None:
                self._grid_legend.scene().removeItem(self._grid_legend)
            self._grid_legend = None

    def _top_legend(self, p: pg.PlotItem, ds: Dataset, subs: tuple) -> None:
        """The selections (in their colour, or by sensor in their line style) and the
        sub-ranges (their line style; by sensor each (selection, sub-range) pair's)."""
        def add(pen, key, text_of):
            p.legend.addItem(pg.PlotDataItem(pen=pen), text_of(ds))
            self._sel_labels[key] = (p.legend.items[-1][1], text_of)

        def sel_text(num):
            return lambda d: next((self._sel_text(s) for s in d.selections if s.number == num), "")

        def sub_text_of(n, prefix=""):
            def f(d):
                ab = next(((a, b) for m, a, b in d.subs if m == n), None)
                return prefix + (f"{sub_name(n)} {ab[0]:.2f} .. {ab[1]:.2f} s" if ab else sub_name(n))
            return f
        grey = (40, 40, 40)
        if self.color_by == SENSOR and subs:
            for j, s in enumerate(ds.selections):
                for k, u in enumerate(subs):
                    add(pg.mkPen(grey, width=2, style=line_style(j * len(subs) + k)),
                        (s.number, u[0]), sub_text_of(u[0], f"#{s.number} · "))
            return
        for j, s in enumerate(ds.selections):
            pen = (pg.mkPen(grey, width=2, style=line_style(j)) if self.color_by == SENSOR
                   else pg.mkPen(sel_color(s.number), width=3))
            add(pen, s.number, sel_text(s.number))
        for k, u in enumerate(subs):
            add(pg.mkPen(grey, width=2, style=sub_style(k)), ("sub", u[0]), sub_text_of(u[0]))

    def probe_items(self, plot: pg.PlotItem):
        unit = next((u for u, p in self.rows.items() if p is plot), None)
        if unit is None:
            return None
        items = [(c.label, unit, c) for c in self.curves.values() if c.unit == unit]
        return (*self._probe_axes(), items)

    def _probe_axes(self) -> tuple:
        return "t", "s", (lambda x: x), (lambda y: y)


class SpecBand(pg.LinearRegionItem):
    """A sub-range on the spectrogram: display only, a light veil with white edges
    and its number (the tile stays the whole selection)."""

    def __init__(self, number: int, region: tuple):
        super().__init__(region, movable=False, pen=pg.mkPen((255, 255, 255), width=1.5,
                                                              style=Qt.PenStyle.DashLine))
        self.number = number
        self.setZValue(5)
        for it in (self, *self.lines):
            it.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            it.setAcceptHoverEvents(False)

    def paint(self, p, *args) -> None:
        dev = p.transform().mapRect(self.boundingRect())
        p.save()
        p.resetTransform()
        p.fillRect(dev, QBrush(QColor(255, 255, 255, 55)))
        p.setPen(QColor(255, 255, 255))
        p.drawText(dev.adjusted(4, 2, 0, 0), int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop),
                   sub_name(self.number))
        p.restore()


class SubBand(pg.LinearRegionItem):
    """One sub-range in one waveform row: hatched (``SUB_HATCH``) and labelled with
    its number in device pixels (a brush pattern would be stretched by the view's
    scale); edges and body drag; right-click deletes it; with the window's Select
    on, a drag on the body is left to the row (a new sub-range there). ``owner``
    is the ``WaveCell``."""

    def __init__(self, number: int, region: tuple, owner: "WaveCell"):
        super().__init__(region, movable=True, pen=_SUB_PEN,
                         hoverPen=pg.mkPen((30, 30, 30), width=3), bounds=(0.0, None))
        self.number, self._owner = number, owner
        self.setZValue(-20)

    def paint(self, p, *args) -> None:
        dev = p.transform().mapRect(self.boundingRect())
        p.save()
        p.resetTransform()
        a = 70 if self.mouseHovering or self.moving else 40
        p.fillRect(dev, QBrush(QColor(40, 40, 40, a),
                               SUB_HATCH[(self.number - 1) % len(SUB_HATCH)]))
        p.setPen(QColor(40, 40, 40))
        p.drawText(dev.adjusted(3, 1, 0, 0), int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop),
                   sub_name(self.number))
        p.restore()

    def mouseDragEvent(self, ev) -> None:  # noqa: N802 — pyqtgraph naming
        if self._owner._selecting() and ev.isStart():
            ev.ignore()                              # Select: a new one, made by the row
            return
        super().mouseDragEvent(ev)

    def mouseClickEvent(self, ev) -> None:  # noqa: N802 — pyqtgraph naming
        if ev.button() == Qt.MouseButton.RightButton and not self.moving:
            ev.accept()
            self._owner.delete_sub(self.number)
            return
        super().mouseClickEvent(ev)

    def hoverEvent(self, ev) -> None:  # noqa: N802 — pyqtgraph naming
        super().hoverEvent(ev)
        self._owner.hovered_sub = None if ev.isExit() else self.number


RELOAD_ZOOM = 1.5               # the waveform re-reads once zoomed this much past the read


class WaveCell(_RowsCell):
    """The waveform (see the module doc). ``selecting()``: the window's Select is on;
    ``subsChanged([(number, a, b), ...])``; ``subChanged(the first (a, b) or None)``
    (the single sub-range of before); ``clicked(t, (sel number, var key) or None)``."""

    X_LABEL = ("Time from the selection's start", "s")
    TITLE = "Waveform"
    TOOLS = (SELECT, ZOOM)
    subChanged = pyqtSignal(object)
    subsChanged = pyqtSignal(list)
    clicked = pyqtSignal(float, object)
    xChanged = pyqtSignal(float, float)          # the time range shown (the spectrogram copies it)

    def __init__(self, selecting: Callable[[], bool] = lambda: False,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._selecting = selecting
        self.subs: dict[int, tuple] = {}             # number -> (a, b), seconds from the start
        self._next_sub = 1
        self._making: Optional[int] = None           # the sub-range a Select drag is making
        self.hovered_sub: Optional[int] = None
        self._bands: dict[tuple, SubBand] = {}       # (row unit, number) -> its band
        self._loaded: Optional[tuple] = None         # zoomed: the span read (lo, hi, w)
        self.title = self._add_title(
            "Waveform · click a trace: the spectrogram shows it (drawn thick) · drag the teal"
            " line to scrub the plane map")
        self.glw.setToolTip("Click a trace to show it in the spectrogram (cursors: click in the"
                            " spectrogram); drag the teal line to scrub the plane map")

    def x_plot(self) -> Optional[pg.PlotItem]:
        return next(iter(self.rows.values()), None)

    def _rebuild_rows(self, units: list[str]) -> None:
        self._bands = {}
        super()._rebuild_rows(units)
        self._sync_bands()
        first = self.x_plot()
        if first is not None:
            first.getViewBox().sigXRangeChanged.connect(
                lambda _vb, r: self.xChanged.emit(float(r[0]), float(r[1])))
            first.getViewBox().sigXRangeChanged.connect(self._on_x)

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
                self._style(key, c, c.look[0])

    def _row_added(self, p: pg.PlotItem) -> None:
        _hook_drag(p.getViewBox(), self._drag)
        hook_click(p.getViewBox(), self._click)

    @property
    def bands(self) -> list[SubBand]:
        """Every band, by sub-range number then row."""
        return [b for _k, b in sorted(self._bands.items(), key=lambda kb: (kb[0][1],
                                      list(self.rows).index(kb[0][0])))]

    @property
    def sub(self) -> Optional[tuple]:
        """The first sub-range (a, b), or None (the single sub-range of before)."""
        return next(iter(self.subs.values()), None)

    def sub_list(self) -> list[tuple]:
        return [(n, a, b) for n, (a, b) in self.subs.items()]

    def _sync_bands(self) -> None:
        """A band per (row, sub-range), at its range."""
        want = {(u, n) for u in self.rows for n in self.subs}
        for k in [k for k in self._bands if k not in want]:
            b = self._bands.pop(k)
            if b.scene() is not None:
                self.rows[k[0]].removeItem(b) if k[0] in self.rows else b.scene().removeItem(b)
        for u, n in sorted(want, key=lambda k: k[1]):
            b = self._bands.get((u, n))
            if b is None:
                b = self._bands[(u, n)] = SubBand(n, self.subs[n], self)
                b.sigRegionChanged.connect(self._band_dragged)
                self.rows[u].addItem(b, ignoreBounds=True)
            elif not b.moving and not any(ln.moving for ln in b.lines) and \
                    tuple(b.getRegion()) != self.subs[n]:
                b.blockSignals(True)
                b.setRegion(self.subs[n])
                b.blockSignals(False)

    @staticmethod
    def _norm(a: float, b: float) -> Optional[tuple]:
        a, b = sorted((float(a), float(b)))
        a, b = max(0.0, a), max(0.0, b)
        return (a, b) if b > a else None

    def _changed(self) -> None:
        self._sync_bands()
        self.subsChanged.emit(self.sub_list())
        self.subChanged.emit(self.sub)

    def add_sub(self, a: float, b: float) -> Optional[int]:
        """A new sub-range (its number), or None if empty."""
        r = self._norm(a, b)
        if r is None:
            return None
        n = self._next_sub
        self._next_sub += 1
        self.subs[n] = r
        self._changed()
        return n

    def move_sub(self, n: int, a: float, b: float) -> None:
        r = self._norm(a, b)
        if n in self.subs and r is not None and r != self.subs[n]:
            self.subs[n] = r
            self._changed()

    def delete_sub(self, n: int) -> None:
        if self.subs.pop(n, None) is not None:
            if self.hovered_sub == n:
                self.hovered_sub = None
            self._changed()

    def clear_subs(self) -> None:
        if self.subs:
            self.subs = {}
            self.hovered_sub = None
            self._changed()
        self._next_sub = 1

    def _drag(self, vb: pg.ViewBox, ev) -> bool:
        """Select on: a drag makes a new sub-range and sets its range as it goes."""
        if not self._selecting() or ev.button() != Qt.MouseButton.LeftButton:
            return False
        ev.accept()
        x0 = vb.mapSceneToView(ev.buttonDownScenePos()).x()
        x = vb.mapSceneToView(ev.scenePos()).x()
        short = abs(ev.scenePos().x() - ev.buttonDownScenePos().x()) < 3
        if ev.isStart() or self._making not in self.subs:
            self._making = None if short else self.add_sub(x0, x)
        elif not short:
            self.move_sub(self._making, x0, x)
        if ev.isFinish():
            if short and self._making is not None:  # shrank back to a click: none
                self.delete_sub(self._making)
            self._making = None
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

    def _band_dragged(self, band: SubBand) -> None:
        self.move_sub(band.number, *band.getRegion())

    def set_sub(self, sub, emit: bool = False) -> None:
        """Exactly one sub-range (a, b) seconds from each selection's start, or None
        (none). The signals go out only with ``emit``."""
        if sub is not None and len(self.subs) == 1:
            r = self._norm(*sub)
            if r is not None:
                n = next(iter(self.subs))
                if r != self.subs[n]:
                    self.subs[n] = r
                    self._sync_bands()
                    if emit:
                        self.subsChanged.emit(self.sub_list())
                        self.subChanged.emit(self.sub)
                return
        self.blockSignals(not emit)
        try:
            self.clear_subs()
            if sub is not None:
                self.add_sub(*sub)
        finally:
            self.blockSignals(False)

    def reset_x(self) -> None:
        """X follows the data again (the whole selections, read whole)."""
        self._auto = True
        self._loaded = None

    def fit_x(self, longest: float) -> None:
        """X from 0 to the longest selection, unless zoomed by hand."""
        if self._auto and longest > 0:
            p = self.x_plot()
            if p is not None:
                p.setXRange(0.0, longest, padding=0.01)

    def _view(self) -> Optional[tuple[float, float, float]]:
        """What ``Dataset.traces`` reads (``view``): None while X follows the data
        (each whole selection); zoomed / panned by hand, the span shown plus one
        width each side (``_loaded``), kept while the view stays inside it and its
        width within ``RELOAD_ZOOM`` of the one read for (so a small pan reads
        nothing)."""
        if self._auto:
            self._loaded = None
            return None
        p = self.x_plot()
        if p is None:
            return None
        x0, x1 = p.getViewBox().viewRange()[0]
        w = max(x1 - x0, 1e-9)
        ld = self._loaded
        if ld is not None and ld[0] <= x0 and x1 <= ld[1] and \
                1 / RELOAD_ZOOM <= ld[2] / w <= RELOAD_ZOOM:
            return ld
        self._loaded = (x0 - w, x1 + w, w)
        return self._loaded

    def _on_x(self, *_a) -> None:
        """X moved: redraw if the span read no longer covers it (after the manual
        flag lands: pyqtgraph sends the range before ``sigRangeChangedManually``)."""
        QTimer.singleShot(0, self._check_view)

    def _check_view(self) -> None:
        if self._auto or not self.rows:
            return
        if self._view() != (self._last[-1] if self._last else None):
            self.changed.emit()

    def render(self, ds: Dataset, shades: dict) -> None:
        units = self._units(ds)
        first = self.rows[units[0]]
        n_px = max(50, int(first.getViewBox().width()) or 600)
        v0 = None if ds.v0 is None else np.asarray(ds.v0).tobytes()
        view = self._view()
        inputs = (n_px, tuple((s.number, s.t0, s.t1, id(s.source)) for s in ds.selections),
                  tuple(v.key for v in ds.variables), ds.despike, v0, ds.scale,
                  tuple(sorted(shades.items(), key=lambda kv: kv[0])), self._auto,
                  self.color_by, id(self.grid), view)
        if inputs == self._last:
            return
        self._last = inputs
        self.renders += 1
        self._index(ds)
        seen = set()
        for tr in ds.traces(n_px, view):
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
        self.log_a.setChecked(False)                  # the default: linear (peaks read directly)
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

    def _style_index(self, number: int, sub_i: int, n_subs: int) -> Optional[int]:
        """The line style of a (selection, sub-range) trace: by selection the
        sub-range's (solid with none); by sensor the (selection, sub-range) pair's
        place."""
        if self.color_by == SENSOR:
            return self._sel_index.get(number, 0) * max(1, n_subs) + sub_i
        return sub_i if n_subs else None

    @property
    def options(self) -> tuple[str, str]:
        return (("hann", "rect")[self.window_box.currentIndex()],
                self.detrend_box.currentText())

    def _probe_axes(self) -> tuple:
        lf, la = self.log_f.isChecked(), self.log_a.isChecked()
        return ("f", "Hz", (lambda x: 10 ** x) if lf else (lambda x: x),
                (lambda y: 10 ** y) if la else (lambda y: y))

    def spectra(self, ds: Dataset) -> dict:
        """{(sel number, var key[, sub number]): (f, amp) or None} (the sub-range's
        number only when there are sub-ranges), from the cache where the inputs are
        unchanged."""
        win, det = self.options
        out, keep = {}, {}
        for sel, var, sub in ds.triples():
            k = (sel.number, var.key, sub[0]) if sub else (sel.number, var.key)
            key = _input_key(ds, sel, var, win, det, sub=sub)
            hit = self._cache.get(k)
            if hit is None or hit[0] != key:
                self.computed += 1
                got = ds.spectrum(sel, var, win, det, sub=sub)
                hit = (key, *(got if got is not None else (None, None)))
            keep[k] = hit
            out[k] = None if hit[1] is None else (hit[1], hit[2])
        self._cache = keep
        return out

    def render(self, ds: Dataset, shades: dict) -> None:
        win, det = self.options
        inputs = (tuple((s.number, _input_key(ds, s, v, win, det, sub=u), v.key, u)
                        for s, v, u in ds.triples()),
                  tuple(sorted(shades.items(), key=lambda kv: kv[0])),
                  self.log_f.isChecked(), self.log_a.isChecked(),
                  self.f_lo.value(), self.f_hi.value(), self._auto, self.color_by, id(self.grid))
        if inputs == self._last:
            return                                    # nothing changed: no work
        self._last = inputs
        self.renders += 1
        self._index(ds)
        units = self._units(ds)
        lf, la = self.log_f.isChecked(), self.log_a.isChecked()
        seen, fmax, fmin_pos = set(), 0.0, np.inf
        subs = [u[0] for u in ds.subs]
        for k, got in self.spectra(ds).items():
            if got is None:
                continue
            num, key, sn = (*k, 0)[:3]
            sel = next(s for s in ds.selections if s.number == num)
            var = next(v for v in ds.variables if v.key == key)
            si = subs.index(sn) if sn in subs else 0
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
            seen.add(k)
            label = f"{var.label} #{num}" + (f" {sub_name(sn)}" if sn else "")
            self._curve(sel, var, shades[key], key=k, label=label,
                        style_i=self._style_index(num, si, len(subs))).setData(f, a)
        self._drop(seen)
        for p in self.rows.values():
            p.setLogMode(x=lf, y=la)
        self._legends(ds, shades, units, tuple(ds.subs))
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
        self.hprobe = pg.InfiniteLine(angle=0, movable=False, pen=pg.mkPen((255, 255, 255, 170)))
        self.pdots = pg.ScatterPlotItem(size=8, pen=pg.mkPen("k", width=1))
        self.ptext = pg.TextItem(color=(20, 20, 20), fill=pg.mkBrush(255, 255, 255, 225), anchor=(0, 1))
        self.peak_text = pg.TextItem(color=(150, 0, 0), fill=pg.mkBrush(255, 255, 255, 225),
                                     anchor=(0, 0.5))
        for it, z in ((self.hprobe, 25), (self.pdots, 26), (self.ptext, 27), (self.peak_text, 27)):
            it.setZValue(z)
            it.hide()
            self.plot.addItem(it, ignoreBounds=True)
        hook_click(self.plot.getViewBox(), self._click)
        self.pick: Optional[tuple] = None             # (sel number, var key)
        self.bands: list[SpecBand] = []               # the sub-ranges, highlighted
        self.var = None                               # the picked Variable / Selection
        self.sel = None
        self.levels: Optional[tuple] = None           # the range in use
        self.data = None                              # (tc, f, amp) shown, or None
        self._cache: dict[tuple, tuple] = {}
        self.computed = 0

    # ----- the picker

    def set_pick(self, number: int, key: str) -> None:
        """Show (selection, variable): the whole selection, its sub-ranges highlighted."""
        if (number, key) != self.pick:
            self.pick = (number, key)
            self.changed.emit()

    def _on_picker(self, _i: int) -> None:
        num, key = self.sel_box.currentData(), self.var_box.currentData()
        if num is not None and key is not None:
            self.set_pick(num, key)

    @property
    def tile_name(self) -> str:
        """``W1 A #2``."""
        if self.var is None or self.sel is None:
            return ""
        return f"{self.var.label} #{self.sel.number}"

    def _sync_bands(self, ds: Dataset) -> None:
        """The sub-ranges as bands (relative to the selection's zero, as the tile)."""
        want = [(n, (a, b)) for n, a, b in ds.subs]
        if [(b.number, b.getRegion()) for b in self.bands] == [(n, tuple(map(float, r)))
                                                                for n, r in want]:
            return
        for b in self.bands:
            self.plot.removeItem(b)
        self.bands = []
        for n, r in want:
            b = SpecBand(n, r)
            self.plot.addItem(b, ignoreBounds=True)
            self.bands.append(b)

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
            if not items:
                box.clear()
                continue
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

    def _zoom_plots(self) -> list[pg.PlotItem]:
        return [self.plot]

    def reset_view(self) -> None:
        super().reset_view()

    def _click(self, vb: pg.ViewBox, ev) -> bool:
        if ev.button() != Qt.MouseButton.LeftButton or ev.double():
            return False
        ev.accept()
        self.clicked.emit(float(vb.mapSceneToView(ev.scenePos()).x()))
        return True

    def probe_hide(self) -> None:
        for it in (self.probe, self.hprobe, self.pdots, self.ptext, self.peak_text):
            it.hide()

    def probe_at(self, x: float, y: float) -> Optional[tuple]:
        """Probe at view (t, f): a crosshair with t, f and the amplitude there next to
        it, and the segment's peak (above 0 Hz) marked with its f and amplitude.
        Returns ((f, amp) here, (f, amp) of the peak) or None off the tile."""
        self.probe_hide()
        if self.data is None:
            return None
        col = spectrogram_column(self.data, x)
        f = self.data[1]
        if col is None or not len(f) or y < f[0] - 0.5 * (f[1] - f[0] if len(f) > 1 else 1) \
                or y > f[-1]:
            return None
        cf, amp, centre = col
        fi = int(np.argmin(np.abs(f - y)))
        pos = np.flatnonzero(f > 0)
        pk = int(pos[np.argmax(amp[pos])]) if len(pos) else fi

        def val(a: float) -> str:
            return f"{float(to_db(np.array([a]))[0]):.1f} dB" if self.db else f"{a:.4g} {self.unit}"
        px_x, _px_y = self.plot.getViewBox().viewPixelSize()
        self.probe.setPos(x)
        self.hprobe.setPos(f[fi])
        self.pdots.setData([x, centre], [f[fi], f[pk]],
                           brush=[pg.mkBrush(255, 255, 255), pg.mkBrush(220, 30, 30)])
        self.ptext.setText(f"t = {x:.3f} s · f = {f[fi]:.2f} Hz · {val(amp[fi])}")
        self.ptext.setPos(x + 8 * px_x, f[fi])
        self.peak_text.setText(f"peak {f[pk]:.2f} Hz · {val(amp[pk])}")
        self.peak_text.setPos(centre + 8 * px_x, f[pk])
        for it in (self.probe, self.hprobe, self.pdots, self.ptext, self.peak_text):
            it.show()
        return (f[fi], amp[fi]), (f[pk], amp[pk])

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
        return f"t = {x:.3f} s · f = {f[fi]:.2f} Hz · {self.tile_name}: {val}"

    # ----- drawing

    def spectrogram(self, ds: Dataset):
        """The picked tile's ``Dataset.spectrogram`` (cached by its inputs)."""
        seg = SEGMENTS_S[self.seg_box.currentIndex()]
        ovl = OVERLAPS[self.ovl_box.currentIndex()]
        k = (self.sel.number, self.var.key)
        key = _input_key(ds, self.sel, self.var, seg, ovl, sub=None)
        hit = self._cache.get(k)
        if hit is None or hit[0] != key:
            self.computed += 1
            hit = (key, ds.spectrogram(self.sel, self.var, seg, ovl, sub=None))
            live = {(s.number, v.key) for s, v in ds.pairs()}
            self._cache = {kk: vv for kk, vv in self._cache.items() if kk in live}
            self._cache[k] = hit
        return hit[1]

    def render(self, ds: Dataset) -> None:
        self._sync_picker(ds)
        num, key = self.pick
        self.sel = next(s for s in ds.selections if s.number == num)
        self.var = next(v for v in ds.variables if v.key == key)
        self._sync_bands(ds)
        seg = SEGMENTS_S[self.seg_box.currentIndex()]
        ovl = OVERLAPS[self.ovl_box.currentIndex()]
        st = self._range(self.unit)
        inputs = (self.pick, _input_key(ds, self.sel, self.var, seg, ovl, sub=None),
                  self.db, st["auto"], None if st["auto"] else st[self.db])
        if inputs == self._last:
            return
        self._last = inputs
        self.renders += 1
        self._pages.setCurrentIndex(1)
        self.data = g = self.spectrogram(ds)
        if g is None:
            self.img.clear()
            self.levels = None
            self.plot.setTitle(f"{self.tile_name}: shorter than one segment",
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
        self.plot.setTitle(self.tile_name, color=sel_color(self.sel.number).name(),
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

    def _zoom_plots(self) -> list[pg.PlotItem]:
        return [self.plot]

    def on_manual(self) -> None:
        self._auto = False

    def reset_view(self) -> None:
        super().reset_view()
        self._auto = True
        self.plot.enableAutoRange(y=True)

    def probe_items(self, plot: pg.PlotItem):
        if plot is not self.plot:
            return None
        items = [(f"cursor at {cursor_text(c.t)}", self._unit, c) for c in self.curves.values()]
        return "f", "Hz", (lambda x: x), (lambda y: y), items

    def render(self, spec: SpecgramCell, cursors) -> None:
        """``spec``: the spectrogram cell (its tile and dB); ``cursors``: [(number, t)]."""
        if spec.data is None or not cursors:
            self.show_hint("The spectrogram tile is shorter than one segment." if spec.data is None
                           else "Click in the spectrogram to add a cursor: the"
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
                           f"{sel_color(spec.sel.number).name()}'>{spec.tile_name}</span>")
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
            c.t = t
            if col is None:
                c.setData([], [])
                labels[n] = (c, f"{cursor_text(t)}: outside the tile")
                continue
            self.columns[n] = col
            f, a, centre = col
            c.setData(f, to_db(a) if spec.db else a)
            labels[n] = (c, f"{cursor_text(t)} (segment at {centre:.2f} s)")
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
