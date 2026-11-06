"""The Analysis window: a timeline over a resizable 2x2 of plots.

Layout (each window keeps its own sizes)::

    side panel | timeline (whole recording, absolute time)
               | -------------------------------------------------
               | waveform (relative time)   | spectrum of the windows
               | spectrogram (one tile)     | spectrum at the cursors

The left column has the time axes (the same time span in both), the right one
the frequency axes. The 2x2 is a vertical splitter over two horizontal ones that
keep the same sizes while all four cells are open, so the left / right border is
one line across both rows (``Grid2x2``). Each cell collapses on its own (the
button at the right of its header, or its border dragged to the edge): its row's
other cell takes the width, a row with both collapsed gives the other row the
height, and a thin bar above the 2x2 lists the collapsed cells (click one to
bring it back). The expand button next to a cell's collapse button gives it the
whole 2x2 (the other three collapse); a second click, or the bar, puts back the
layout from before the expand. A double-click on a border shares its space
equally again. The timeline sits over the 2x2 in its own vertical splitter.

The side panel holds two lists:

- Selections: every selection of the app-wide model (``views/selections.py``),
  with an include box, its colour and number, t0 / t1 boxes (edits move it
  everywhere) and delete. Which are included is this window's own.
- Variables: the whisker gauges as two small grids laid out like the Dome
  view's plane map (``views/sensor_grid.py``), a Plots grid over a Timeline
  grid: a sensor's name strip toggles both its gauges, its A or B half that
  gauge; a checked gauge is lit in its colour, a dead or absent one dimmed.
  The grids' "by position" view (the combo above them) shows the same cells as
  the dome's positions, the halves drag and lift (``pos:<position>:<axis>``:
  read through each recording's dome map, polarity applied), so one choice
  reads the same place on the dome in recordings wired differently.
  "Plane map" (next to the combo; on by default, in either view: both views
  are the same cells, only the picker's labels differ) makes the Plots grid the
  Dome view's plane map in small, without taking a cell of the 2x2: every
  position at the plane head (a teal line on the timeline, absolute, dragged
  to scrub; ``_render_head``: the recording there and its last second, whatever
  is checked or included), under it each included selection's path (each
  sub-range its own, in its line style), numbered arrows at the cursors. Events
  (no timeline): a relative head in the waveform, the highlighted / picked
  events' paths (``daq/plane_overlay.py``; the cells still toggle the variables:
  name or plot both halves, the chips one).
  Below them a tree with the rest, IMU (accel x / y / z, gyro x / y / z,
  attitude roll / pitch / yaw, depth) and ROV (the vehicle telemetry, when the
  source has it, with the pilot's demand and the thrust projected per axis:
  intended against achieved motion), with the same two checkbox columns:
  Plots (the 2x2) and Timeline (the timeline's own rows, independent). The
  Timeline column starts on Depth when the recording has depth, else on the
  first variable checked for Plots, until it is edited.

The cells read one ``daq/analysis.Dataset`` (the included selections x the
Plots variables, with the panel's Despike / V0 / scale and the sub-range),
made per redraw; they are in ``views/analysis_cells.py`` and
``views/analysis_timeline.py``. A collapsed cell is not drawn, and every cell
skips its work when its inputs did not change.

Cursors: a click in the spectrogram adds a numbered vertical cursor at that
relative time, drawn there and in the waveform; drag it to move it, right-click
it or Del (hovered) to delete it. A click in the waveform adds one too, except on
a trace other than the picked one: that shows its (variable, selection) in the
spectrogram (the trace is then drawn thick).

Readouts (``views/hover_probe.py``): no per-variable legends in the timeline,
waveform and spectrum; hovering a trace draws it thicker and names it with its
value next to the mouse (the spectrum also marks its peaks). Probe (P) marks
every trace's value at the mouse in the row, labelled; in the spectrogram a
crosshair with its value and the segment's peak.
The bottom right shows the picked tile's spectrogram column at each cursor.

Toolbar: Select (S: a drag in the waveform makes the sub-range, in the
timeline a new selection), Clear sub-range (Esc), Zoom box (Z: a drag in the
waveform, the spectrum, the spectrogram or the cursor spectra zooms to the box
drawn, the waveform and spectrogram keeping one time span; R resets; the mouse is
a magnifier with a + while armed), Probe (P: hover readout in
the waveform, timeline, both spectra and the spectrogram), Reset view (R: the
cell under the mouse, or all), F (fit the plot under the mouse to its data), Keys,
Colour by (sensor by default in Selections mode). The Line window's zoom / pan keys act on
the cell under the mouse (= - X, + _ Y of the row under the mouse, . , Y of all
its rows, arrows move the data), matched by the character typed in an
application event filter while the window is shown, as the Line window does.

Events mode (``mode=EVENTS``, fixed for the window; titled "Events n"): the
source is an event set (``replay/eventset.py``: the manoeuvre events of the
sidecars in a scope of recordings, filtered, grouped, curated) instead of the
selections. The side panel's Selections list gives way to the ``EventPanel``
(``views/event_panel.py``: saved comparisons, scope, filter, group by; a
change re-runs the query, debounced) and Positions is checked by default. The
timeline gives way to the event browser (``views/event_cells.EventBrowserCell``:
a table and a feature scatter sharing one selection; hover or select draws the
event thick in the waveform; a checkbox / the context menu curate; a
double-click opens the recording in the replay at the event, ``openEvent``).
The waveform is ``EventWaveCell``: time from each event's onset, colour by
group, thin single events under a darker, outlined median with its 25-75 %
band per group, a "drawn N of M" cap, optionally the 3x4 dome grid; a click on
a trace selects that event. The spectrum and the spectrogram read the picked
event alone (the first selected, else the first drawn); there is no group
median spectrum (one FFT per event and variable per redraw: not cheap at a few
hundred events). The sidecars, the stores and the traces are fetched off the
GUI thread (``views/event_loader.py``, progress in the panel); the window only
draws what arrives. In Selections mode the timeline's manoeuvre lane (the open
recording's sidecar, ``manoeuvres_fn``) makes a selection on a click and opens
an Events window on similar events from its menu (``compareSimilar``).

Several Analysis windows may be open, each with its own included selections,
variables, sizes, cursors and options (the control panel keeps them; the
newest focused one takes a region's Compare). Model changes only mark the
window dirty: the render scheduler redraws it at most ``MAX_HZ`` a second, and
not at all while it is hidden or closed (closing only hides it; the panel
reuses it for the next new window).
"""

from __future__ import annotations

import math
import weakref
from pathlib import Path
from typing import Callable, Optional, Sequence

from PyQt6.QtCore import QEvent, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QAction, QColor, QCursor, QIcon, QKeySequence, QPen, QPixmap
from PyQt6.QtWidgets import (QAbstractItemView, QAbstractSpinBox, QApplication, QCheckBox, QComboBox,
                             QDoubleSpinBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMessageBox,
                             QSplitter, QSplitterHandle, QToolBar, QToolButton, QTreeWidget,
                             QTreeWidgetItem, QVBoxLayout, QWidget)

import numpy as np
import pyqtgraph as pg

from wavi_py.daq.analysis import DEPTH, POS, ROV, Dataset, Selection, Variable
from wavi_py.daq.plane_overlay import head_state, plane_overlay, recording_origin, set_arrows
from wavi_py.daq.series import IMU, WSK
from wavi_py.replay import eventset as evs
from wavi_py.views.analysis_cells import (SELECTION, SENSOR, CursorCell, CursorLines, line_style,
                                          SpecgramCell, SpectrumCell, WaveCell, shades_of)
from wavi_py.views.analysis_timeline import TimelineCell, timeline_source_of
from wavi_py.views.base import ViewWindow
from wavi_py.views.event_cells import EventBrowserCell, EventWaveCell, group_colors
from wavi_py.views.event_loader import EventLoader
from wavi_py.views.event_panel import EventPanel
from wavi_py.views.hover_probe import HoverProbe
from wavi_py.views.sensor_grid import GridLayout, SensorGrid
from wavi_py.views.render_sched import scheduler
from wavi_py.views.selections import SelectionModel, sel_color
from wavi_py.views.whisker_plots import _PAN_STEP, ZOOM_STEP

CELLS = ("Compare", "Spectrum", "Spectrogram", "Cursors")   # set_tab names (region menus)
SELECTIONS, EVENTS = "selections", "events"
GAUGES_VIEW, POSITIONS_VIEW = "by gauge (A / B)", "by position (drag / lift)"                 # the window's source mode
HEAD_COLOR = (0, 130, 110)                                  # the plane head: teal, no cursor's colour
PLOTS, TIMELINE = 1, 2                                      # the Variables tree's check columns
_KEY = Qt.ItemDataRole.UserRole
_LIVE_POLL_MS = 1000
TIMELINE_H = 190                                            # its default height (two rows)                                        # the timeline follows live data

KEYS_HELP = """\
<table cellspacing=4>
<tr><td><b>S</b></td><td>Select: drag in the waveform for another sub-range, in the timeline for a new selection</td></tr>
<tr><td><b>Esc</b></td><td>clear the sub-ranges (all)</td></tr>
<tr><td><b>Z</b></td><td>zoom box: drag a rectangle to zoom (waveform, spectra, spectrogram); off: a drag pans</td></tr>
<tr><td><b>P</b></td><td>probe: every trace's value at the mouse, marked and labelled (waveform, timeline, spectra); the spectrogram: t, f, amplitude and the segment's peak</td></tr>
<tr><td><b>R</b></td><td>reset view: the cell under the mouse (outside the cells: all)</td></tr>
<tr><td><b>F</b></td><td>fit: the plot under the mouse to its data (x and y)</td></tr>
<tr><td><b>= &nbsp;-</b></td><td>zoom X in / out (the cell under the mouse). Keypad + - too</td></tr>
<tr><td><b>+ &nbsp;_</b></td><td>zoom Y in / out: the row under the mouse</td></tr>
<tr><td><b>. &nbsp;,</b></td><td>zoom Y in / out: all rows of the cell</td></tr>
<tr><td><b>arrows</b></td><td>move the data (Left: traces slide left), the cell under the mouse</td></tr>
<tr><td><b>Del</b></td><td>delete the cursor (else the sub-range, else the selection) under the mouse</td></tr>
</table>
Mouse: a click in the spectrogram or the waveform adds a cursor; in the waveform a
click on another trace shows it in the spectrogram instead (drawn thick). Hover a
trace for its name and value (spectrum: its peaks too).
Drag a cursor to move it, right-click deletes it. Timeline: drag pans, wheel zooms,
double-click seeks. A cell's – button (or its border dragged to the edge) collapses
it; the bar above the plots brings it back. Double-click a border between cells to
share the space equally."""


def _swatch(color: QColor) -> QIcon:
    pm = QPixmap(12, 12)
    pm.fill(color)
    return QIcon(pm)


class _Handle(QSplitterHandle):
    """A splitter border: a double-click shares the space equally."""

    def mouseDoubleClickEvent(self, ev) -> None:  # noqa: N802 — Qt naming
        self.splitter().equalise()


class _Splitter(QSplitter):
    equalised = pyqtSignal()

    def createHandle(self) -> QSplitterHandle:  # noqa: N802 — Qt naming
        return _Handle(self.orientation(), self)

    def equalise(self) -> None:
        """The shown children share the space equally (a collapsed one keeps none)."""
        shown = [i for i in range(self.count()) if not self.widget(i).isHidden()]
        if not shown:
            return
        total = sum(self.sizes()) or (self.width() if self.orientation() == Qt.Orientation.Horizontal
                                      else self.height())
        self.setSizes([total // len(shown) if i in shown else 0 for i in range(self.count())])
        self.equalised.emit()


def _open(cell: QWidget) -> bool:
    """Shown and not squashed (a collapsed cell is not drawn)."""
    return cell.isVisibleTo(cell.window()) and cell.width() > 1 and cell.height() > 1


class Grid2x2(QWidget):
    """The 2x2 (see the module doc): ``top`` / ``bottom`` horizontal splitters in
    ``rows``. While all four cells are open, moving one row's border moves the
    other's, so the column border is one line. Each cell collapses on its own
    (``collapse``: its header's button, or its border dragged to the edge): it is
    hidden, its row's other cell takes the width, and a row with both collapsed
    is hidden (the other row takes the height). The collapsed cells are listed
    as buttons on a thin bar above the 2x2 (``bar``); a click restores one
    (``restore``) at the sizes it had."""

    resized = pyqtSignal()                       # a border moved, a cell collapsed / came back

    def __init__(self, tl: QWidget, tr: QWidget, bl: QWidget, br: QWidget,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        self.cells = (tl, tr, bl, br)
        self.bar = QWidget()
        bl_ = QHBoxLayout(self.bar)
        bl_.setContentsMargins(4, 1, 4, 1)
        bl_.setSpacing(4)
        lab = QLabel("Collapsed:")
        lab.setStyleSheet("color: #555;")
        bl_.addWidget(lab)
        self.restore_buttons: dict = {}
        for c in self.cells:
            b = QToolButton()
            b.setText(f"▸ {getattr(c, 'TITLE', '') or 'cell'}")
            b.setToolTip("Bring this cell back")
            b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            b.clicked.connect(lambda _=False, c=c: self.restore(c))
            b.hide()
            bl_.addWidget(b)
            self.restore_buttons[c] = b
        bl_.addStretch(1)
        self.bar.hide()
        lay.addWidget(self.bar)
        self.rows = _Splitter(Qt.Orientation.Vertical)
        self.top = _Splitter(Qt.Orientation.Horizontal)
        self.bottom = _Splitter(Qt.Orientation.Horizontal)
        self.top.addWidget(tl)
        self.top.addWidget(tr)
        self.bottom.addWidget(bl)
        self.bottom.addWidget(br)
        self.rows.addWidget(self.top)
        self.rows.addWidget(self.bottom)
        self._cols: dict = {}                        # row -> its column sizes, last seen both open
        self._row_sizes: Optional[list] = None       # row sizes, last seen both rows shown
        for s in (self.rows, self.top, self.bottom):
            s.setChildrenCollapsible(True)
            s.setHandleWidth(5)
            s.splitterMoved.connect(lambda *_, s=s: self._moved(s))
            s.equalised.connect(lambda s=s: self._moved(s))
        self.expanded: Optional[QWidget] = None      # the expanded cell
        self._pre_expand: Optional[tuple] = None     # (hidden cells, sizes) before it
        for c in self.cells:
            if hasattr(c, "collapseRequested"):
                c.collapseRequested.connect(lambda c=c: self.collapse(c))
            if hasattr(c, "expandRequested"):
                c.expandRequested.connect(lambda c=c: self.expand(c))
        lay.addWidget(self.rows, 1)
        self.top.installEventFilter(self)
        self.bottom.installEventFilter(self)

    # ----- collapse / restore

    def _row(self, cell: QWidget) -> QSplitter:
        return self.top if cell in self.cells[:2] else self.bottom

    def _partner(self, cell: QWidget) -> QWidget:
        return self.cells[self.cells.index(cell) ^ 1]

    def collapsed(self) -> list:
        return [c for c in self.cells if c.isHidden()]

    def all_open(self) -> bool:
        return not self.collapsed()

    def _remember(self) -> None:
        for row in (self.top, self.bottom):
            if not row.isHidden() and not any(row.widget(i).isHidden() for i in range(2)) \
                    and all(row.sizes()):
                self._cols[row] = row.sizes()
        if not self.top.isHidden() and not self.bottom.isHidden() and all(self.rows.sizes()):
            self._row_sizes = self.rows.sizes()

    def expand(self, cell: QWidget) -> None:
        """``cell`` takes the whole 2x2 (the others collapse); on the expanded cell,
        back to the layout before (``unexpand``)."""
        if self.expanded is cell:
            self.unexpand()
            return
        if self.expanded is not None:
            self.unexpand()
        self._remember()
        self._pre_expand = ([c for c in self.cells if c.isHidden()],
                            self.top.sizes(), self.bottom.sizes(), self.rows.sizes(),
                            dict(self._cols), self._row_sizes)
        self.expanded = cell
        for c in self.cells:
            c.setHidden(c is not cell)
        row = self._row(cell)
        other = self.bottom if row is self.top else self.top
        row.show()
        other.hide()
        self._set_expand_buttons()
        self._show_bar()
        self.resized.emit()

    def unexpand(self) -> None:
        """Back to the layout from before the expand."""
        if self.expanded is None:
            return
        hidden, top, bottom, rows, cols, row_sizes = self._pre_expand
        self.expanded, self._pre_expand = None, None
        for c in self.cells:
            c.setHidden(c in hidden)
        for row in (self.top, self.bottom):
            row.setHidden(all(row.widget(i).isHidden() for i in range(2)))
        self.top.setSizes(top)
        self.bottom.setSizes(bottom)
        self.rows.setSizes(rows)
        self._cols, self._row_sizes = cols, row_sizes
        self._set_expand_buttons()
        self._show_bar()
        self.resized.emit()

    def _set_expand_buttons(self) -> None:
        for c in self.cells:
            h = getattr(c, "header", None)
            if h is not None:
                h.expand_btn.setChecked(c is self.expanded)

    def collapse(self, cell: QWidget) -> None:
        if cell.isHidden():
            return
        if self.expanded is not None:                # a collapse ends the expand (kept as is)
            self.expanded, self._pre_expand = None, None
            self._set_expand_buttons()
        self._remember()
        cell.hide()
        if self._partner(cell).isHidden():
            self._row(cell).hide()                   # the whole row: the other takes the height
        self._show_bar()
        self.resized.emit()

    def restore(self, cell: QWidget) -> None:
        if self.expanded is not None:                # the bar while expanded: the layout before
            self.unexpand()
        if not cell.isHidden():
            return
        row = self._row(cell)
        other_row = self.bottom if row is self.top else self.top
        row_was_hidden = row.isHidden()
        cell.show()
        row.show()
        if row_was_hidden and not other_row.isHidden():
            h = sum(self.rows.sizes())
            self.rows.setSizes(self._row_sizes or [h // 2, h - h // 2])
        if not self._partner(cell).isHidden():       # the row has both again
            w = sum(row.sizes())
            if self.all_open():
                row.setSizes(other_row.sizes())     # one column border again
            else:
                row.setSizes(self._cols.get(row) or [w // 2, w - w // 2])
        self._show_bar()
        self.resized.emit()

    def _show_bar(self) -> None:
        for c, b in self.restore_buttons.items():
            b.setVisible(c.isHidden())
        self.bar.setVisible(bool(self.collapsed()))

    def _moved(self, s: QSplitter) -> None:
        if s in (self.top, self.bottom) and self.all_open():
            self._sync(s, self.bottom if s is self.top else self.top)
        # a border dragged to the edge collapses what it squashed
        for row in (self.top, self.bottom):
            if row.isHidden():
                continue
            if s is self.rows and self.rows.sizes()[self.rows.indexOf(row)] == 0:
                for i in range(2):
                    self.collapse(row.widget(i))
                continue
            for i, size in enumerate(row.sizes()):
                if size == 0 and not row.widget(i).isHidden() and not row.widget(i ^ 1).isHidden():
                    self.collapse(row.widget(i))
        self._remember()
        self.resized.emit()

    @staticmethod
    def _sync(src: QSplitter, dst: QSplitter) -> None:
        if dst.sizes() != src.sizes():
            dst.blockSignals(True)
            dst.setSizes(src.sizes())
            dst.blockSignals(False)

    def eventFilter(self, obj, ev) -> bool:  # noqa: N802 — Qt naming
        # each row splitter shares out a resize itself (rounding apart): one border again
        if ev.type() == QEvent.Type.Resize and obj in (self.top, self.bottom) and self.all_open():
            QTimer.singleShot(0, lambda: self.all_open() and self._sync(self.top, self.bottom))
        return False

    def state(self) -> dict:
        return {"cols": self.top.sizes(), "rows": self.rows.sizes(),
                "collapsed": [getattr(c, "TITLE", "") for c in self.collapsed()]}

    def set_state(self, cols: Sequence[int], rows: Sequence[int]) -> None:
        """All cells open, at these sizes."""
        self.expanded, self._pre_expand = None, None
        self._set_expand_buttons()
        for c in self.cells:
            self.restore(c)
        self.top.setSizes(list(cols))
        self.bottom.setSizes(list(cols))
        self.rows.setSizes(list(rows))
        self._remember()
        self.resized.emit()


class AnalysisWindow(ViewWindow):
    """See the module doc. ``variables_fn()`` lists the variables there are (the
    gauges by the Line window's sensor names, the IMU, depth, demand / thrust,
    the dome's positions); ``source_fn()`` is what the timeline shows (a
    ``SelSource``: the whole recording, the live buffers; or None / a str when
    there is none); ``replay``: the ``ReplayModel`` (playhead, seek);
    ``dome_map_fn()``: the active dome map (the grids' layout; the Positions of a
    selection without its own map, i.e. live data: a replayed recording's selection
    carries its recording's map, ``SelectionModel.dome_map_fn``); ``manoeuvres_fn()``: the open recording's manoeuvres sidecar (the
    timeline's lane) or None. ``mode``: SELECTIONS or EVENTS (fixed for the
    window); EVENTS starts on ``query`` and fetches stores through ``cache``."""

    MAX_HZ = 20.0
    activated = pyqtSignal(object)            # this window got the focus (newest-focused)
    openEvent = pyqtSignal(str, float)        # Events: open this recording in the replay at t
    compareSimilar = pyqtSignal(object)       # the lane's Compare similar: an EventQuery

    def __init__(self, model: SelectionModel, variables_fn: Callable[[], list], number: int = 1,
                 source_fn: Optional[Callable] = None, replay=None,
                 dome_map_fn: Optional[Callable] = None, mode: str = SELECTIONS,
                 manoeuvres_fn: Optional[Callable] = None,
                 query: Optional[evs.EventQuery] = None, cache: Optional[evs.StoreCache] = None):
        super().__init__()
        self.mode = mode
        self._dome_map_fn = dome_map_fn or (lambda: None)
        self._manoeuvres_fn = manoeuvres_fn or (lambda: None)
        self.model = model
        self._variables_fn = variables_fn
        self._grid_checked: dict[int, set] = {PLOTS: set(), TIMELINE: set()}  # wsk / pos keys
        self._default_map = None
        self._grid_key = None
        self._source_fn = source_fn
        self.replay = replay
        self._vars: list[Variable] = []
        self._included: list[Selection] = []
        self._last_included: Optional[Selection] = None
        self._lists_dirty = True
        self._tl_edited = False                   # the Timeline column was edited by hand
        self.number = number
        self.cursors: list[list] = []             # [number, t (relative)]
        self._next_cursor = 1
        self.hovered_cursor: Optional[int] = None
        self._hover_cell = None                   # the cell last under the mouse (keys)
        self.setWindowTitle(self._title(number))
        self.resize(1300, 860)
        ref = weakref.ref(self)                   # the scheduler must not keep it alive
        scheduler().register(self, lambda: (w := ref()) is not None and w._render(), self.MAX_HZ)
        self._build_toolbar()

        # side panel
        side = QWidget()
        sl = QVBoxLayout(side)
        sl.setContentsMargins(4, 4, 4, 4)
        self._sel_label = QLabel("<b>Selections</b>")
        sl.addWidget(self._sel_label)
        self.events_panel: Optional[EventPanel] = None
        if mode == EVENTS:
            self._sel_label.hide()
            self.events_panel = EventPanel()
            sl.addWidget(self.events_panel)
        self.sel_list = QTreeWidget()
        self.sel_list.setHeaderLabels(["#", "t0 (s)", "t1 (s)", ""])
        self.sel_list.setRootIsDecorated(False)
        self.sel_list.setToolTip("Include a selection (box); edit its times; ✕ deletes it")
        hdr = self.sel_list.header()
        hdr.setStretchLastSection(False)
        for c, rm in enumerate((QHeaderView.ResizeMode.ResizeToContents,
                                QHeaderView.ResizeMode.Stretch, QHeaderView.ResizeMode.Stretch,
                                QHeaderView.ResizeMode.ResizeToContents)):
            hdr.setSectionResizeMode(c, rm)
        self.sel_list.itemChanged.connect(self._on_sel_item)
        sl.addWidget(self.sel_list, 1)
        self.sel_list.setVisible(mode != EVENTS)
        sl.addWidget(QLabel("<b>Variables</b>"))
        self.grid_view = QComboBox()
        self.grid_view.addItems([GAUGES_VIEW, POSITIONS_VIEW])
        self.grid_view.setToolTip("The grids by gauge (A / B, as wired) or by dome position"
                                  " (drag / lift, through each recording's dome map)")
        self.grid_view.setCurrentText(POSITIONS_VIEW if mode == EVENTS else GAUGES_VIEW)
        self.grid_view.currentTextChanged.connect(lambda _t: self._rebuild_grids())
        self.plane_box = QCheckBox("Plane map")
        self.plane_box.setToolTip(
            "The Plots grid as the Dome view's plane map (either view): every position at the"
            " plane head (the teal line in the timeline: drag it), with its last second, in"
            " degrees of bend, drag toward the side's front, lift up, all cells on one scale;"
            " under it each included selection's path (sub-ranges: each its own), numbered"
            " arrows at the cursors. Without V0 the head is relative to the recording's first"
            " 2 s, a path to its selection's first 0.25 s")
        self.plane_box.toggled.connect(self._on_plane_toggled)
        gv = QHBoxLayout()
        gv.setContentsMargins(0, 0, 0, 0)
        gv.addWidget(self.grid_view, 1)
        gv.addWidget(self.plane_box)
        sl.addLayout(gv)
        self._plane_key = None                    # what the plane map's paths were read for
        self._plane_ov = None
        self._path_cache: dict = {}               # each selection's plane paths by its inputs
        self.plane_t: Optional[float] = None      # Events: the plane head in the waveform (relative s)
        self._head_key = None                     # Selections: the plane head's inputs ...
        self._head_state = None                   # ... and what it read (plane_overlay.HeadState)
        self._head_big = 0.0                      # the largest head bend seen (the map's scale)
        self._head_src = None
        self._origin = None                       # without V0: the recording's start per position
        self._head_lines: dict = {}               # id(row) -> (row, line)
        self.grids = {PLOTS: SensorGrid("Plots"), TIMELINE: SensorGrid("Timeline")}
        for col, g in self.grids.items():
            g.setToolTip("Click a sensor's name for both gauges, its A or B half for one")
            g.toggled.connect(lambda keys, on, col=col: self.set_checked(keys, on, col))
            sl.addWidget(g)
        self.var_tree = QTreeWidget()
        self.var_tree.setColumnCount(3)
        self.var_tree.setHeaderLabels(["", "Plots", "Timeline"])
        self.var_tree.headerItem().setToolTip(PLOTS, "Shown in the 2x2 (waveform, spectra)")
        self.var_tree.headerItem().setToolTip(TIMELINE, "Shown in the timeline (its own rows)")
        vh = self.var_tree.header()
        vh.setStretchLastSection(False)
        vh.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        for c in (PLOTS, TIMELINE):
            vh.setSectionResizeMode(c, QHeaderView.ResizeMode.ResizeToContents)
        self.var_tree.itemChanged.connect(self._on_var_item)
        sl.addWidget(self.var_tree, 2)

        # the cells
        self.spectrogram = SpecgramCell()
        self.compare = (EventWaveCell() if mode == EVENTS
                        else WaveCell(lambda: self._act_select.isChecked()))
        self.spectrum = SpectrumCell()
        self.at_cursors = CursorCell()
        self.timeline = TimelineCell(model, self, self._act_select, replay)
        self.grid = Grid2x2(self.compare, self.spectrum, self.spectrogram, self.at_cursors)
        self.compare.subsChanged.connect(self._on_subs)
        self.compare.clicked.connect(self._on_wave_click)
        self.spectrogram.clicked.connect(self.add_cursor)
        for cell in (self.spectrum, self.spectrogram, self.timeline):
            cell.changed.connect(self._request)
        self.grid.resized.connect(self._on_resized)
        self.spectrogram.plot.getViewBox().sigRangeChangedManually.connect(
            lambda *_: self.compare.on_manual())
        # the waveform and the spectrogram show the same time span (copied both ways:
        # with a collapsed neighbour their widths differ, and pyqtgraph's own link
        # would line them up on screen and crop the narrower one)
        self.compare.xChanged.connect(self._wave_x)
        self.spectrogram.plot.getViewBox().sigXRangeChanged.connect(
            lambda _vb, r: self.compare.set_x(float(r[0]), float(r[1])))
        if mode != EVENTS:
            self.set_color_by(SENSOR)                 # the default (each gauge its own colour)
        self.timeline.madeSelection.connect(self.include)
        self.timeline.planeMoved.connect(self._on_plane_moved)
        self.timeline.laneClicked.connect(self._lane_clicked)
        self.timeline.laneCompare.connect(self._lane_compare)
        self.subs: tuple = ()                         # the waveform's sub-ranges ((n, a, b), ...)
        self._wave_lines = CursorLines(self)
        self._spec_lines = CursorLines(self, halo=True)

        self.right = _Splitter(Qt.Orientation.Vertical)
        self.browser: Optional[EventBrowserCell] = None
        if mode == EVENTS:
            self.browser = EventBrowserCell()
            self.right.addWidget(self.browser)
            self.timeline.hide()
        else:
            self.right.addWidget(self.timeline)
        self.right.addWidget(self.grid)
        self.right.setChildrenCollapsible(True)
        self.right.setStretchFactor(1, 1)
        self.right.splitterMoved.connect(lambda *_: self._on_resized())
        self.split = QSplitter()
        self.split.addWidget(side)
        self.split.addWidget(self.right)
        self.split.setStretchFactor(1, 1)
        self.setCentralWidget(self.split)
        self.default_sizes()

        self._hovers = {c: HoverProbe(c, peaks=c in (self.spectrum, self.at_cursors))
                        for c in self.cells() if c is not self.spectrogram}
        for cell in self.cells():
            cell.glw.scene().sigMouseMoved.connect(lambda pos, c=cell: self._on_mouse(c, pos))
        self._live_timer = QTimer(self)
        self._live_timer.setInterval(_LIVE_POLL_MS)
        self._live_timer.timeout.connect(self._poll_source)
        if replay is not None:
            replay.positionChanged.connect(self._on_position)
            replay.sourceChanged.connect(self._on_data_changed)   # the ROV group comes and goes
        model.added.connect(self._on_added)
        model.removed.connect(self._on_removed)
        model.changed.connect(self._on_changed)
        model.dataChanged.connect(self._on_data_changed)
        self._rebuild_vars()
        if mode == EVENTS:
            self._init_events(query or evs.EventQuery(types=["onset"], group_by="run"), cache)
        self.plane_box.setChecked(True)            # the plane map: on by default

    def _build_toolbar(self) -> None:
        tb = QToolBar("Analysis")
        tb.setMovable(False)
        self.addToolBar(tb)
        self._act_select = QAction("Select", self, checkable=True)
        self._act_select.setShortcut(QKeySequence("S"))
        self._act_select.setToolTip("Drag in the waveform for a sub-range (seconds from each"
                                    " selection's start; the spectra use it), or in the timeline"
                                    " for a new selection [S]")
        self._act_select.toggled.connect(lambda on: self.statusBar().showMessage(
            "Select: drag in the waveform for a sub-range (Esc clears it), in the timeline for a"
            " new selection") if on else self.statusBar().clearMessage())
        self._act_clear_sub = QAction("Clear sub-range", self)
        self._act_clear_sub.setShortcut(QKeySequence(Qt.Key.Key_Escape))
        self._act_clear_sub.setToolTip("Clear every sub-range: back to the whole selections [Esc]")
        self._act_clear_sub.setEnabled(False)
        self._act_clear_sub.triggered.connect(lambda: self.set_sub(None))
        self._act_zoom = QAction("Zoom box", self, checkable=True)
        self._act_zoom.setShortcut(QKeySequence("Z"))
        self._act_zoom.setToolTip("Drag a box to zoom in the waveform, spectrum, spectrogram and"
                                  " cursor spectra (off: a drag pans). R resets. [Z]")
        self._act_zoom.toggled.connect(self._on_zoom_toggled)
        self._act_probe = QAction("Probe", self, checkable=True)
        self._act_probe.setShortcut(QKeySequence("P"))
        self._act_probe.setToolTip("Show the values under the mouse (waveform, timeline, spectra,"
                                   " spectrogram) [P]")
        self._act_probe.toggled.connect(self._on_probe_toggled)
        self._act_reset = QAction("Reset view", self)
        self._act_reset.setShortcut(QKeySequence("R"))
        self._act_reset.setToolTip("Fit the data: the cell under the mouse, or all [R]")
        self._act_reset.triggered.connect(self._reset_view)
        act_keys = QAction("Keys", self)
        act_keys.setToolTip("Keyboard and mouse shortcuts")
        act_keys.triggered.connect(self._show_keys)
        for a in (self._act_select, self._act_clear_sub, self._act_zoom, self._act_probe,
                  self._act_reset, act_keys):
            tb.addAction(a)
            w = tb.widgetForAction(a)
            if w is not None:
                w.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        tb.addSeparator()
        tb.addWidget(QLabel(" Colour by "))
        self.color_box = QComboBox()
        self.color_box.addItems([SELECTION, SENSOR])
        self.color_box.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.color_box.setToolTip("selection: a trace has its selection's colour, the variables"
                                  " of a row in its shades; sensor: each gauge its own fixed"
                                  " colour, the selections told apart by line style (solid,"
                                  " dash, dot, dash-dot)")
        self.color_box.currentTextChanged.connect(self.set_color_by)
        tb.addWidget(self.color_box)
        self.color_by = SELECTION
        delete = QAction("Delete cursor / selection", self)
        delete.setShortcuts([QKeySequence(Qt.Key.Key_Delete), QKeySequence(Qt.Key.Key_Backspace)])
        delete.triggered.connect(self._delete_hovered)
        self.addAction(delete)

    def cells(self) -> list:
        if self.mode == EVENTS:
            return [self.compare, self.spectrum, self.spectrogram, self.at_cursors]
        return [self.timeline, self.compare, self.spectrum, self.spectrogram, self.at_cursors]

    def _title(self, number: int) -> str:
        return f"Wavi - {'Events' if self.mode == EVENTS else 'Analysis'} {number}"

    def default_sizes(self) -> None:
        """The layout a new window starts with: panel 290 px (Events: 330), timeline
        150 px (Events: the browser, 30 %), the 2x2 at 62 / 38 % across and half /
        half down (Events: 70 / 30, the waveform's row 62 %: the spectrogram keeps
        room for its tile under its colour bar)."""
        w, h = max(800, self.width()), max(500, self.height())
        if self.mode == EVENTS:
            pw, th = 330, int(h * 0.30)
            self.split.setSizes([pw, w - pw])
            self.right.setSizes([th, h - th])
            cw, gh = w - pw, h - th
            self.grid.set_state([int(cw * 0.7), cw - int(cw * 0.7)], [int(gh * 0.62), gh - int(gh * 0.62)])
            return
        th = max(TIMELINE_H, self.timeline.minimumHeight())
        self.split.setSizes([290, w - 290])
        self.right.setSizes([th, h - th])
        cw = w - 290
        self.grid.set_state([int(cw * 0.62), cw - int(cw * 0.62)], [(h - th) // 2] * 2)

    # ----- state (the panel and the region menus use these)

    @property
    def included(self) -> list[Selection]:
        """The included selections still in the model, in model order."""
        return [s for s in self.model.selections if s in self._included]

    def include(self, sel: Selection, on: bool = True) -> None:
        if on and sel not in self._included:
            self._included.append(sel)
        elif not on and sel in self._included:
            self._included.remove(sel)
        if on:
            self._last_included = sel
        self._lists_dirty = True
        self._sync_lists()
        self._request()

    def checked_keys(self, column: int = PLOTS) -> list[str]:
        """The checked variables of a column, in the variables' order."""
        on = set(self._grid_checked[column]) | {
            it.data(0, _KEY) for it in self._var_leaves()
            if it.checkState(column) == Qt.CheckState.Checked}
        return [v.key for v in self._vars if v.key in on]

    def timeline_keys(self) -> list[str]:
        return self.checked_keys(TIMELINE)

    def set_checked(self, keys: Sequence[str], on: bool = True, column: int = PLOTS) -> None:
        keys = set(keys)
        if column == PLOTS:                           # timeline-only variables have no Plots box
            keys -= {v.key for v in self._vars if not v.plots}
        wsk = {v.key for v in self._vars if v.source in (WSK, POS)} & keys
        if on:
            self._grid_checked[column] |= wsk
        else:
            self._grid_checked[column] -= wsk
        self._sync_grids()
        self.var_tree.blockSignals(True)
        for it in self._var_leaves():
            if it.data(0, _KEY) in keys:
                it.setCheckState(column, Qt.CheckState.Checked if on else Qt.CheckState.Unchecked)
        self.var_tree.blockSignals(False)
        self._fix_parents()
        if column == TIMELINE:
            self._tl_edited = True
        self._request()

    def set_timeline(self, keys: Sequence[str]) -> None:
        """The Timeline column checks exactly ``keys``."""
        self.set_checked([v.key for v in self._vars], False, TIMELINE)
        self.set_checked(keys, True, TIMELINE)

    def set_tab(self, name: str) -> None:
        """A region menu's Compare / Spectrum / Spectrogram: that cell shows (a
        collapsed one gets its space back); Spectrogram also picks the selection
        last included, with the first Plots variable."""
        cell = {"Compare": self.compare, "Spectrum": self.spectrum,
                "Spectrogram": self.spectrogram, "Cursors": self.at_cursors}[name]
        self.grid.restore(cell)
        if cell.width() <= 1:                         # squashed without collapsing
            self.grid._row(cell).equalise()
        if cell.height() <= 1:
            self.grid.rows.equalise()
        if cell is self.spectrogram and self._last_included is not None:
            keys = self.checked_keys()
            if keys:
                self.spectrogram.set_pick(self._last_included.number, keys[0])
        self._request()

    def set_color_by(self, mode: str) -> None:
        """Colour by ``sensor`` (default) or ``selection`` (this window's own)."""
        mode = SENSOR if mode == SENSOR else SELECTION
        if self.color_box.currentText() != mode:
            self.color_box.setCurrentText(mode)          # comes back here
            return
        self.color_by = mode
        for cell in (self.compare, self.spectrum, self.timeline):
            cell.color_by = mode
        self._request()

    def reset_state(self, number: int) -> None:
        """A closed window reused as a new one: nothing included or checked, no
        cursors, the default sizes."""
        self.number = number
        self.setWindowTitle(self._title(number))
        self._included = []
        self.set_checked([v.key for v in self._vars], False)
        self.set_checked([v.key for v in self._vars], False, TIMELINE)
        self._tl_edited = False
        self.set_color_by(SENSOR if self.mode != EVENTS else SELECTION)
        self.set_sub(None)
        self.cursors = []
        self._next_cursor = 1
        self._sync_cursors()
        self.spectrogram.pick = None
        for cell in self.cells():
            cell.reset_view()
        self.default_sizes()
        self._lists_dirty = True
        self.plane_box.setChecked(True)            # the plane map: on by default

    def dataset(self) -> Dataset:
        keys = set(self.checked_keys())
        m = self.model
        if self.mode == EVENTS:
            pick = self.picked_event_selection()
            return Dataset([pick] if pick is not None else [],
                           [v for v in self._vars if v.key in keys], baseline=True)
        return Dataset(self.included, [v for v in self._vars if v.key in keys],
                       despike=m.despike, v0=m.v0, scale=m.scale, subs=self.subs,
                       dome_map=self._position_map())

    def timeline_dataset(self) -> Dataset:
        keys = set(self.timeline_keys())
        m = self.model
        rec_map = m.dome_map_fn()             # the replayed recording's own map, if any
        return Dataset([], [v for v in self._vars if v.key in keys],
                       despike=m.despike, v0=m.v0, scale=m.scale,
                       dome_map=rec_map if rec_map is not None else self._position_map())

    @property
    def sub(self) -> Optional[tuple]:
        """The first sub-range (a, b), or None."""
        return self.subs[0][1:] if self.subs else None

    def set_sub(self, sub) -> None:
        """Exactly one sub-range (a, b) in seconds from each selection's start, or
        None: none (the whole selections)."""
        self.compare.set_sub(sub, emit=True)

    def add_sub(self, a: float, b: float) -> Optional[int]:
        """Another sub-range (its number): the spectra use each of them."""
        return self.compare.add_sub(a, b)

    def delete_sub(self, n: int) -> None:
        self.compare.delete_sub(n)

    def _on_subs(self, subs: list) -> None:
        subs = tuple(tuple(u) for u in subs)
        if subs == self.subs:
            return
        self.subs = subs
        self._act_clear_sub.setEnabled(bool(subs))
        for cell in (self.spectrum, self.spectrogram):
            cell.set_sub_text(subs)
        self._request()

    # ----- cursors

    def add_cursor(self, t: float) -> int:
        n = self._next_cursor
        self._next_cursor += 1
        self.cursors.append([n, float(t)])
        self._sync_cursors()
        self._request()
        return n

    def move_cursor(self, number: int, t: float) -> None:
        for c in self.cursors:
            if c[0] == number and c[1] != t:
                c[1] = float(t)
                self._sync_cursors()
                self._request()

    def delete_cursor(self, number: int) -> None:
        self.cursors = [c for c in self.cursors if c[0] != number]
        if self.hovered_cursor == number:
            self.hovered_cursor = None
        self._sync_cursors()
        self._request()

    def hover_cursor(self, number: Optional[int]) -> None:
        self.hovered_cursor = number

    def _sync_cursors(self) -> None:
        cur = [(n, t) for n, t in self.cursors]
        self._wave_lines.sync(list(self.compare.rows.values()), cur)
        self._spec_lines.sync([self.spectrogram.plot], cur)

    def _wave_x(self, lo: float, hi: float) -> None:
        x0, x1 = self.spectrogram.plot.getViewBox().viewRange()[0]
        if abs(x0 - lo) > 1e-9 or abs(x1 - hi) > 1e-9:
            self.spectrogram.plot.setXRange(lo, hi, padding=0)

    def _on_wave_click(self, t: float, pick) -> None:
        """A click on a trace shows it in the spectrogram; on the one shown (drawn
        thick), it adds a cursor there; away from the traces, nothing."""
        if pick is None or pick == self.spectrogram.pick:
            self.add_cursor(t)
        else:
            self.spectrogram.set_pick(*pick)
            self.compare.set_picked(pick)

    def _delete_hovered(self) -> None:
        """Del: the cursor under the mouse, else the sub-range, else the selection."""
        if self.hovered_cursor is not None:
            self.delete_cursor(self.hovered_cursor)
        elif self.compare.hovered_sub is not None:
            self.delete_sub(self.compare.hovered_sub)
        else:
            self.model.remove(self.model.hovered)

    # ----- variables tree

    def _var_leaves(self) -> list[QTreeWidgetItem]:
        out = []

        def walk(it: QTreeWidgetItem) -> None:
            for i in range(it.childCount()):
                ch = it.child(i)
                if ch.data(0, _KEY) is not None:
                    out.append(ch)
                walk(ch)
        walk(self.var_tree.invisibleRootItem())
        return out

    def _fix_parents(self) -> None:
        """Tristate parents follow their children after a programmatic change."""
        self.var_tree.blockSignals(True)

        def walk(it: QTreeWidgetItem, col: int) -> Optional[Qt.CheckState]:
            if it.childCount() == 0:
                return it.checkState(col)
            states = {walk(it.child(i), col) for i in range(it.childCount())}
            st = (states.pop() if len(states) == 1 else Qt.CheckState.PartiallyChecked)
            if it is not self.var_tree.invisibleRootItem():
                it.setCheckState(col, st)
            return st
        for col in (PLOTS, TIMELINE):
            walk(self.var_tree.invisibleRootItem(), col)
        self.var_tree.blockSignals(False)

    def _on_var_item(self, _it: QTreeWidgetItem, col: int) -> None:
        if col == TIMELINE:
            self._tl_edited = True
        self._request()

    def _sync_grids(self) -> None:
        for col, g in self.grids.items():
            g.set_checked(c for k in self._grid_checked[col] if (c := g.col_of(k)) is not None)

    @property
    def grid_positions(self) -> bool:
        """The grids show the dome's positions (drag / lift), not the gauges."""
        return self.grid_view.currentText() == POSITIONS_VIEW

    def _position_map(self):
        """The map the Positions read when neither the selection nor the replayed
        recording has its own (live data): the window's dome map, else the default one."""
        m = self._dome_map_fn()
        if m is None:
            if self._default_map is None:
                from whisker_link.dome_map import DEFAULT_MAP, DomeMap
                self._default_map = DomeMap.load(DEFAULT_MAP)
            m = self._default_map
        return m

    def _rebuild_grids(self) -> None:
        """The grids' layout from the dome map and the gauges there are."""
        wsk = [v for v in self._vars if v.source == WSK]
        pos = self.grid_positions
        dm = self._position_map()          # both views: the same cells (only the labels differ)
        key = (id(dm), len(wsk), tuple(v.label for v in wsk), pos)
        if key == self._grid_key:
            return
        self._grid_key = key
        nsensor = max(1, (len(wsk) + 1) // 2) if wsk else 16
        grid = GridLayout.of(dm, nsensor, len(wsk) if wsk else 2 * nsensor,
                             [v.group for v in wsk[::2]] or None)
        off = set()
        if pos:                       # a position's axis without a live gauge is dimmed
            for sn, name in enumerate(grid.names):
                p = dm.position(name) if dm is not None else None
                for j, ax in enumerate(SensorGrid.AXES):
                    g = p.daq.get(ax) if p is not None else None
                    if g is None or g.ch is None or int(g.ch) in grid.dead:
                        off.add(2 * sn + j)
        self._plane_key = None
        for g in self.grids.values():
            g.positions, g.pos_off = pos, off
            g.set_grid(grid)
            g.labels = {v.col: v.label for v in wsk}
            g.setToolTip("Click a position's name for its drag and lift, a half for one"
                         if pos else "Click a sensor's name for both gauges, its A or B half"
                         " for one")
        self.compare.grid = self.spectrum.grid = grid
        self._sync_grids()
        self._request()

    # ----- the plane map (the Plots grid, either view)

    def _on_plane_toggled(self, on: bool) -> None:
        g = self.grids[PLOTS]
        g.set_map(on)
        if not on:
            g.set_overlay(None)
            self.timeline.set_plane_head(None)
        self._sync_head()
        self._plane_key = self._head_key = None
        self._request()

    def _plane_on(self) -> bool:
        return self.plane_box.isChecked()

    def _plane_selections(self) -> list:
        """What the plane map draws paths of: the included selections; Events: the
        highlighted events, else the picked one."""
        if self.mode != EVENTS:
            return list(self.included)
        ov = self.overlay
        if ov is None:
            return []
        by = ov.by_number
        got = [by[n] for n in self.highlighted() if n in by]
        pick = self.picked_event_selection()
        return got or ([pick] if pick is not None else [])

    def _plane_pen(self, sels: list, item) -> QPen:
        if self.mode == EVENTS:
            c = QColor(self.colors.get(item.sel.group, QColor(60, 60, 60)))
        else:
            c = QColor(sel_color(item.sel.number))
        nums = [n for n, _a, _b in self.subs]
        style = (line_style(nums.index(item.sub)) if item.sub in nums else Qt.PenStyle.SolidLine)
        return QPen(c, 1.3, style)

    def _plane_positions(self) -> list:
        g = self.grids[PLOTS]
        return [n for s, n in enumerate(g.grid.names) if g.position_of(2 * s)]

    def _render_plane(self) -> None:
        g = self.grids[PLOTS]
        if not (self._plane_on() and g.map):
            return
        sels = self._plane_selections()
        positions = self._plane_positions()
        m = self.model
        if self.mode == EVENTS:
            ds = Dataset(sels, [], baseline=True)
        else:
            ds = Dataset(sels, [], despike=m.despike, v0=m.v0, scale=m.scale, subs=self.subs,
                         dome_map=self._position_map())
        key = (tuple((id(s), s.t0, s.t1, s.align, id(s.source), id(s.dome_map)) for s in sels),
               tuple(positions), self.subs, m.despike, id(m.v0), m.scale, id(ds.dome_map),
               self.mode == EVENTS)
        cur = [(n, t) for n, t in self.cursors]
        if self.mode == EVENTS and self.plane_t is None and sels:   # relative head: mid-way
            x = self.compare.x_plot()
            lo, hi = x.getViewBox().viewRange()[0] if x is not None else (0.0, 0.0)
            self.plane_t = 0.5 * (lo + hi) if hi > lo else 0.0
        head = self.plane_t if self.mode == EVENTS else None
        if key != self._plane_key:
            self._plane_key = key
            self._plane_ov = plane_overlay(ds, positions, cur, head=head, cache=self._path_cache,
                                           peaks=self.mode == EVENTS)
        else:
            set_arrows(self._plane_ov, cur, head, peaks=self.mode == EVENTS)
        if self.mode != EVENTS:
            self._render_head(positions)
        self._plane_ov.scale = max(self._plane_ov.scale, self._head_big, 0.5)
        g.set_overlay(self._plane_ov, lambda item, sels=sels: self._plane_pen(sels, item))
        self._sync_head()

    def _render_head(self, positions: list) -> None:
        """The recording at the plane head (absolute, on the timeline): every position's
        tip and its trail, from the replayed recording or the live buffers."""
        src = timeline_source_of(self._source_fn)
        ov = self._plane_ov
        if src is None or ov is None:
            if ov is not None:
                ov.head, ov.trail = {}, {}
            self.timeline.set_plane_head(None)
            return
        ext = src.extent()
        if self.timeline.plane_t is None:          # start at the playhead, else mid-view
            r = self.replay
            t = r.position if r is not None and r.source is not None else math.nan
            if not math.isfinite(t):
                lo, hi = self.timeline.x_plot().getViewBox().viewRange()[0]
                t = 0.5 * (lo + hi)
            if ext is not None:
                t = min(max(t, ext[0]), ext[1])
            self.timeline.set_plane_head(t)
        t = self.timeline.plane_t
        m = self.model
        rec_map = m.dome_map_fn()
        dm = rec_map if rec_map is not None else self._position_map()
        ds = Dataset([], [], despike=m.despike, v0=m.v0, scale=m.scale, dome_map=dm)
        key = (t, id(src.series), tuple(positions), m.despike, id(m.v0), m.scale, id(dm))
        if key == self._head_key:
            ov.head, ov.trail = self._head_state.head, self._head_state.trail
            return
        if self._head_src != (id(src.series), id(dm)):
            self._head_src = (id(src.series), id(dm))
            self._head_big = 0.0                    # the running scale: a new recording
            self._origin = None
        origin = None
        if m.v0 is None and ext is not None:       # without V0: the recording's start at rest
            if self._origin is None:
                self._origin = recording_origin(ds, src, positions, ext)
            origin = self._origin
        st = head_state(ds, src, positions, t, origin)
        self._head_key, self._head_state = key, st
        self._head_big = max(self._head_big, st.big)
        ov.head, ov.trail = st.head, st.trail

    def _on_plane_moved(self, t: float) -> None:
        """The timeline's plane head was dragged: only the head is re-read (1 s)."""
        g = self.grids[PLOTS]
        if self._plane_ov is None or not g.map:
            return
        self._render_head(self._plane_positions())
        self._plane_ov.scale = max(self._plane_ov.scale, self._head_big, 0.5)
        g.update()

    def _sync_head(self) -> None:
        """Events (no timeline): the plane head's line in every waveform row, relative
        to each event's onset."""
        rows = list(self.compare.rows.values()) if self.mode == EVENTS and self._plane_on() \
            and self.plane_t is not None else []
        want = {id(r): r for r in rows}
        for k in [k for k in self._head_lines if k not in want]:
            r, line = self._head_lines.pop(k)
            r.removeItem(line)
        for k, r in want.items():
            got = self._head_lines.get(k)
            if got is None:
                line = pg.InfiniteLine(self.plane_t, angle=90, movable=True,
                                       pen=pg.mkPen(HEAD_COLOR, width=2, style=Qt.PenStyle.DashLine),
                                       hoverPen=pg.mkPen(HEAD_COLOR, width=4),
                                       label="plane", labelOpts={"position": 0.04, "color": HEAD_COLOR,
                                                                 "fill": (255, 255, 255, 200)})
                line.setZValue(30)
                line.setToolTip("The plane map's head: drag to scrub the arrows")
                line.sigDragged.connect(self._on_head_dragged)
                r.addItem(line, ignoreBounds=True)
                self._head_lines[k] = (r, line)
            elif not got[1].moving and abs(got[1].value() - self.plane_t) > 1e-12:
                got[1].blockSignals(True)
                got[1].setValue(self.plane_t)
                got[1].blockSignals(False)

    def set_plane_t(self, t: float) -> None:
        """Move the plane head: Selections mode on the timeline (absolute s), Events in
        the waveform (relative to the onsets). Only the head is re-read."""
        if self.mode != EVENTS:
            self.timeline.set_plane_head(float(t))
            self._on_plane_moved(float(t))
            return
        self.plane_t = float(t)
        self._sync_head()
        if self._plane_ov is not None:
            set_arrows(self._plane_ov, [(n, c) for n, c in self.cursors], self.plane_t)
            self.grids[PLOTS].update()

    def _on_head_dragged(self, line) -> None:
        self.set_plane_t(line.value())

    def _rebuild_vars(self) -> None:
        """The tree from ``variables_fn`` (kept checks by key); only when it changed."""
        new = list(self._variables_fn())
        if [v.key for v in new] == [v.key for v in self._vars] and \
                [v.label for v in new] == [v.label for v in self._vars]:
            self._rebuild_grids()
            return
        keep = {col: set(self.checked_keys(col)) for col in (PLOTS, TIMELINE)}
        self._vars = new
        have = {v.key for v in new if v.source in (WSK, POS)}
        for col in (PLOTS, TIMELINE):
            self._grid_checked[col] = keep[col] & have
        self._rebuild_grids()
        self.var_tree.blockSignals(True)
        self.var_tree.clear()
        flags = Qt.ItemFlag.ItemIsUserCheckable | Qt.ItemFlag.ItemIsEnabled
        tri = flags | Qt.ItemFlag.ItemIsAutoTristate

        def node(parent, text: str) -> QTreeWidgetItem:
            it = QTreeWidgetItem(parent, [text])
            it.setFlags(tri)
            for col in (PLOTS, TIMELINE):
                it.setCheckState(col, Qt.CheckState.Unchecked)
            return it
        tops = {IMU: node(self.var_tree, "IMU")}
        if any(v.source == ROV for v in new):
            tops[ROV] = node(self.var_tree, "ROV")
        groups: dict = {}
        for v in new:
            if v.source in (WSK, POS):
                continue                              # the grids
            side = v.source if v.source == ROV else IMU
            top = tops[side]
            if v.source == DEPTH:
                parent = top
            else:
                parent = groups.get((side, v.group))
                if parent is None:
                    parent = groups[(side, v.group)] = node(top, v.group)
            text = v.label if v.source == ROV else \
                v.label.split(" ")[-1] if v.source != DEPTH else v.label
            it = QTreeWidgetItem(parent, [text])
            it.setFlags(flags)
            it.setData(0, _KEY, v.key)
            it.setToolTip(0, f"{v.label} ({v.unit})" if v.plots else f"{v.label}: timeline only")
            for col in (PLOTS, TIMELINE) if v.plots else (TIMELINE,):
                it.setCheckState(col, Qt.CheckState.Checked if v.key in keep[col]
                                 else Qt.CheckState.Unchecked)
        for top in tops.values():
            top.setExpanded(True)
        self.var_tree.blockSignals(False)
        self._fix_parents()

    def _default_timeline(self, source) -> None:
        """Until the Timeline column is edited: Depth if the source has depth, else the
        first variable checked for Plots."""
        if self._tl_edited:
            return
        has_depth = source is not None and len(source.depth[0]) > 0
        plots = self.checked_keys()
        want = [DEPTH] if has_depth and any(v.key == DEPTH for v in self._vars) else plots[:1]
        if want and want != self.timeline_keys():
            self.set_checked([v.key for v in self._vars], False, TIMELINE)
            self.set_checked(want, True, TIMELINE)
            self._tl_edited = False                # still the default

    # ----- selections list

    def _on_added(self, sel: Selection) -> None:
        self._lists_dirty = True
        self._sync_lists()

    def _on_removed(self, sel: Selection) -> None:
        if sel in self._included:
            self._included.remove(sel)
        self._lists_dirty = True
        self._sync_lists()
        self._request()

    def _on_changed(self, sel: Selection) -> None:
        if not self.isVisible():
            return                                  # the lists are refreshed on show
        for i in range(self.sel_list.topLevelItemCount()):
            it = self.sel_list.topLevelItem(i)
            if it.data(0, _KEY) == sel.number:
                for col, v in ((1, sel.t0), (2, sel.t1)):
                    box = self.sel_list.itemWidget(it, col)
                    if box is not None and abs(box.value() - v) > 1e-9:
                        box.blockSignals(True)
                        box.setValue(v)
                        box.blockSignals(False)
        if sel in self._included:
            self._request()

    def _on_data_changed(self) -> None:
        self._rebuild_vars()
        self._request()

    def _sync_lists(self) -> None:
        """Rebuild the selections list (cheap; skipped while hidden)."""
        if not self._lists_dirty or not self.isVisible():
            return
        self._lists_dirty = False
        lst = self.sel_list
        lst.blockSignals(True)
        lst.clear()
        for sel in self.model.selections:
            it = QTreeWidgetItem(lst, [f"{sel.number}"])
            it.setFlags(Qt.ItemFlag.ItemIsUserCheckable | Qt.ItemFlag.ItemIsEnabled
                        | Qt.ItemFlag.ItemIsSelectable)
            it.setData(0, _KEY, sel.number)
            it.setIcon(0, _swatch(sel_color(sel.number)))
            it.setCheckState(0, Qt.CheckState.Checked if sel in self._included
                             else Qt.CheckState.Unchecked)
            lo, hi = sel.bounds or (-1e9, 1e9)
            for col, v in ((1, sel.t0), (2, sel.t1)):
                box = QDoubleSpinBox()
                box.setDecimals(3)
                box.setRange(lo, hi)
                box.setSingleStep(0.1)
                box.setKeyboardTracking(False)
                box.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
                box.setValue(v)
                box.valueChanged.connect(lambda val, s=sel, c=col: self.model.set_range(
                    s, val if c == 1 else s.t0, val if c == 2 else s.t1))
                lst.setItemWidget(it, col, box)
            btn = QToolButton()
            btn.setText("✕")
            btn.setToolTip(f"Delete selection {sel.number} (everywhere)")
            btn.clicked.connect(lambda _=False, s=sel: self.model.remove(s))
            lst.setItemWidget(it, 3, btn)
        lst.blockSignals(False)

    def _on_sel_item(self, it: QTreeWidgetItem, col: int) -> None:
        if col != 0:
            return
        sel = self.model.by_number(it.data(0, _KEY))
        if sel is not None:
            on = it.checkState(0) == Qt.CheckState.Checked
            if on and sel not in self._included:
                self._included.append(sel)
                self._last_included = sel
            elif not on and sel in self._included:
                self._included.remove(sel)
            self._request()

    # ----- keys, probe

    def _cell_at(self, global_pos) -> Optional[object]:
        w = QApplication.widgetAt(global_pos)
        while w is not None and w is not self:
            for cell in self.cells():
                if w is cell.glw or w is cell.glw.viewport():
                    return cell
            w = w.parentWidget()
        return None

    def _target(self):
        """(cell, plot under the mouse or None) the keys act on: the cell under the
        mouse, else the one last under it."""
        pos = QCursor.pos()
        cell = self._cell_at(pos) or self._hover_cell
        if cell is None:
            return None, None
        scene_pos = cell.glw.mapToScene(cell.glw.mapFromGlobal(pos))
        plot = next((p for p in cell.nav_plots() if p.sceneBoundingRect().contains(scene_pos)), None)
        return cell, plot

    def eventFilter(self, obj, ev) -> bool:  # noqa: N802 — Qt naming
        """Zoom / pan keys by the character typed, anywhere in this window; the mouse
        leaving a cell clears its readouts."""
        if ev.type() == QEvent.Type.Leave and self._hover_cell is not None and \
                obj is self._hover_cell.glw.viewport():
            self._clear_readouts()
            return False
        if ev.type() != QEvent.Type.KeyPress or not isinstance(obj, QWidget) or obj.window() is not self:
            return False
        if isinstance(obj, (QAbstractSpinBox, QLineEdit, QComboBox, QAbstractItemView)):
            return False                                   # typing / moving in a box or list
        mods = ev.modifiers() & ~(Qt.KeyboardModifier.ShiftModifier | Qt.KeyboardModifier.KeypadModifier)
        if mods:
            return False
        key = ev.key()
        arrows = {Qt.Key.Key_Left: (_PAN_STEP, 0), Qt.Key.Key_Right: (-_PAN_STEP, 0),
                  Qt.Key.Key_Up: (0, -_PAN_STEP), Qt.Key.Key_Down: (0, _PAN_STEP)}
        if key in arrows:
            self._pan(*arrows[key])
            return True
        text = ev.text()
        if not text:
            return False
        if ev.modifiers() & Qt.KeyboardModifier.KeypadModifier:
            action = {"+": lambda: self._zoom_x(1 / ZOOM_STEP), "-": lambda: self._zoom_x(ZOOM_STEP)}.get(text)
        else:
            action = {
                "=": lambda: self._zoom_x(1 / ZOOM_STEP), "-": lambda: self._zoom_x(ZOOM_STEP),
                "+": lambda: self._zoom_y(1 / ZOOM_STEP, False), "_": lambda: self._zoom_y(ZOOM_STEP, False),
                ".": lambda: self._zoom_y(1 / ZOOM_STEP, True), ",": lambda: self._zoom_y(ZOOM_STEP, True),
                "f": self._fit_plot, "F": self._fit_plot,
            }.get(text)
        if action is None:
            return False
        action()
        return True

    def _zoom_x(self, f: float, cell=None) -> None:
        cell = cell or self._target()[0]
        p = cell.x_plot() if cell is not None else None
        if p is not None:
            p.getViewBox().scaleBy(x=f)
            cell.on_manual()
            self._request()

    def _zoom_y(self, f: float, all_rows: bool, cell=None, plot=None) -> None:
        if cell is None:
            cell, plot = self._target()
        if cell is None:
            return
        plots = cell.nav_plots() if all_rows or plot is None else [plot]
        for p in plots:
            p.getViewBox().scaleBy(y=f)
        self._request()

    def _pan(self, dx: float, dy: float, cell=None, plot=None) -> None:
        if cell is None:
            cell, plot = self._target()
        if cell is None:
            return
        if dx and cell.x_plot() is not None:
            vb = cell.x_plot().getViewBox()
            x0, x1 = vb.viewRange()[0]
            vb.translateBy(x=dx * (x1 - x0))
            cell.on_manual()
        if dy:
            for p in ([plot] if plot is not None else cell.nav_plots()):
                vb = p.getViewBox()
                y0, y1 = vb.viewRange()[1]
                vb.translateBy(y=dy * (y1 - y0))
        self._request()

    def _fit_plot(self) -> None:
        """F: the plot under the mouse fits its data (x and y; outside a plot, the
        cell's plots). The waveform's rows and the spectrogram share x, so they follow."""
        cell, plot = self._target()
        if cell is None:
            return
        for p in ([plot] if plot is not None else cell.nav_plots()):
            p.getViewBox().autoRange(padding=0.02)
        cell.on_manual()
        self._request()

    def _reset_view(self) -> None:
        cell = self._cell_at(QCursor.pos())
        for c in ([cell] if cell is not None else self.cells()):
            c.reset_view()
        self._request()

    def _on_zoom_toggled(self, on: bool) -> None:
        for cell in (self.compare, self.spectrum, self.spectrogram, self.at_cursors):
            cell.set_zoom_box(on)
        self.statusBar().showMessage("Zoom box: drag a rectangle to zoom (R resets)" if on
                                     else "Pan: drag to move the view", 3000)

    def _show_keys(self) -> None:
        box = QMessageBox(QMessageBox.Icon.NoIcon, "Analysis keys", KEYS_HELP, parent=self)
        box.setTextFormat(Qt.TextFormat.RichText)
        box.setModal(False)
        box.show()

    def _probe_lines(self) -> list[pg.InfiniteLine]:
        return [*self.compare.probes.values(), *self.spectrum.probes.values(),
                *(self.timeline.probes.values() if self.mode != EVENTS else ()),
                self.spectrogram.probe, self.at_cursors.probe]

    def _on_probe_toggled(self, on: bool) -> None:
        self._clear_readouts()

    def _clear_readouts(self) -> None:
        for line in self._probe_lines():
            line.hide()
        for h in self._hovers.values():
            h.clear()
        self.spectrogram.probe_hide()

    def _on_mouse(self, cell, pos) -> None:
        """Hover (a trace highlighted and named at the mouse) or, with Probe, every
        trace's value at the mouse marked (``views/hover_probe.py``); the spectrogram:
        Probe's crosshair and the segment's peak."""
        if cell is not self._hover_cell:
            self._clear_readouts()
        self._hover_cell = cell
        probe = self._act_probe.isChecked()
        if cell is self.spectrogram:
            vb = self.spectrogram.plot.getViewBox()
            if probe and vb.sceneBoundingRect().contains(pos):
                pt = vb.mapSceneToView(pos)
                self.spectrogram.probe_at(pt.x(), pt.y())
            else:
                self.spectrogram.probe_hide()
            return
        h = self._hovers.get(cell)
        if h is not None:
            h.update(pos, probe)

    def probe_text(self, cell, scene_pos) -> Optional[str]:
        """The Probe readout at ``scene_pos`` in ``cell`` (and its probe lines moved
        there): x, then every curve of the row under the mouse at the nearest sample."""
        plots = [cell.plot] if cell is self.spectrogram else cell.nav_plots()
        plot = next((p for p in plots if p.getViewBox().sceneBoundingRect().contains(scene_pos)),
                    None)
        if plot is None:
            return None
        pt = plot.getViewBox().mapSceneToView(scene_pos)
        if cell is self.spectrogram:
            self.spectrogram.probe.setPos(pt.x())
            self.spectrogram.probe.show()
            return self.spectrogram.probe_text(pt.x(), pt.y())
        got = cell.probe_items(plot)
        if got is None:
            return None
        xname, xunit, fx, fy, items = got
        lines = cell.probes.values() if hasattr(cell, "probes") else [cell.probe]
        for line in lines:
            line.setPos(pt.x())
            line.show()
        parts = [f"{xname} = {fx(pt.x()):.4g} {xunit}"]
        for label, unit, c in items[:10]:
            x, y = c.getData()
            if x is None or not len(x):
                continue
            i = int(min(max(0, np.searchsorted(x, pt.x())), len(x) - 1))
            if i > 0 and abs(x[i - 1] - pt.x()) < abs(x[i] - pt.x()):
                i -= 1
            if np.isfinite(y[i]):
                parts.append(f"{label}: {fy(y[i]):.4g} {unit}")
        if len(items) > 10:
            parts.append(f"(+{len(items) - 10} more)")
        return "  ·  ".join(parts)

    # ----- drawing

    def _request(self) -> None:
        scheduler().request(self)

    def _on_resized(self) -> None:
        self._request()

    def _on_position(self, *_a) -> None:
        if self.isVisible():
            self.timeline.show_position()

    def _poll_source(self) -> None:
        """Live: the timeline follows the buffers (once a second, while shown)."""
        if self.replay is None or not self.replay.active:
            self._rebuild_vars()                    # live telemetry may have started
            self._request()

    # ----- Events mode

    def _init_events(self, query: evs.EventQuery, cache) -> None:
        self.loader = EventLoader(cache, self)
        self.eventset: Optional[evs.EventSet] = None
        self.overlay: Optional[evs.Overlay] = None
        self.colors: dict = {}
        self.sel_keys: list = []
        self.hover_key = None
        self._ov_vars: Optional[tuple] = None
        self._num_of: dict = {}                    # EventRef.key -> selection number
        self._extra: dict = {}                     # (numbers, var keys) -> {number: {key: (t, y)}}
        p, b = self.events_panel, self.browser
        p.queryChanged.connect(self._on_query)
        b.selectionChanged.connect(self._on_event_selection)
        b.hovered.connect(self._on_event_hover)
        b.curate.connect(self.curate)
        b.openEvent.connect(self._open_event_key)
        self.compare.eventClicked.connect(self._on_wave_event)
        self.loader.setReady.connect(self._on_set)
        self.loader.overlayReady.connect(self._on_overlay)
        self.loader.progress.connect(self._on_progress)
        self.loader.failed.connect(lambda m: self._on_progress(f"failed: {m}"))
        # no timeline: no Timeline grid / column; colour is the group; no sub-ranges
        # (the time axis is the onset's, the spectra read the picked event whole)
        self.grids[TIMELINE].hide()
        self.var_tree.setColumnHidden(TIMELINE, True)
        for w, tip in ((self.color_box, "Events: a trace has its group's colour"),
                       (self._act_select, "Events: no sub-ranges (the spectra read the picked"
                                          " event whole)"),
                       (self._act_clear_sub, "Events: no sub-ranges")):
            w.setEnabled(False)
            w.setToolTip(tip)
        for cell in (self.spectrum, self.spectrogram):
            if cell.header is not None:
                cell.header.sub.setText("The picked event's whole window (time from its start;"
                                        " no sub-ranges in Events)")
        if not self.checked_keys():
            self.set_checked([v.key for v in self._vars if v.source == POS])
        p.set_query(query, emit=False)
        self.query = p.query
        self._request_load()

    def set_query(self, q: evs.EventQuery) -> None:
        """Events: the panel, the set and the traces from ``q``."""
        self.events_panel.set_query(q)

    def _checked_vars(self) -> list:
        keys = set(self.checked_keys())
        return [v for v in self._vars if v.key in keys]

    def _request_load(self) -> None:
        vs = self._checked_vars()
        self._ov_vars = tuple(v.key for v in vs)
        self.loader.request(self.query, vs)

    def _on_query(self, q: evs.EventQuery) -> None:
        self.query = q
        self._request_load()

    def _on_set(self, es, gen: int) -> None:
        if gen != self.loader.gen:
            return
        self.eventset = es
        self.colors = group_colors(list(es.groups))
        self.browser.set_events(es, self.colors)
        self._request()

    def _on_overlay(self, ov, gen: int) -> None:
        if gen != self.loader.gen:
            return
        self.overlay = ov
        self._num_of = {s.event.key: s.number for s in ov.selections if s.event is not None}
        self._extra = {}
        self._request()

    def _on_progress(self, text: str) -> None:
        if self.events_panel is not None:
            self.events_panel.set_status(text)

    def _on_event_selection(self, keys: list) -> None:
        self.sel_keys = list(keys)
        self._request()

    def _on_event_hover(self, key) -> None:
        if key != self.hover_key:
            self.hover_key = key
            self._request()

    def _on_wave_event(self, number: int) -> None:
        key = next((k for k, n in self._num_of.items() if n == number), None)
        if key is not None:
            self.browser.set_selected([key], emit=True)

    def _ref(self, key) -> Optional[evs.EventRef]:
        es = self.eventset
        if es is None:
            return None
        return next((r for r in es.events + es.excluded if r.key == tuple(key)), None)

    def _open_event_key(self, key) -> None:
        r = self._ref(key)
        if r is not None:
            self.openEvent.emit(r.recording, float(r.t))

    def curate(self, action: str, keys: list) -> None:
        """The browser's Exclude / Include / Pin / Unpin / Show only these / Include
        all: into the query's curation (and so into a saved comparison)."""
        q = self.events_panel.query
        ex = {tuple(k) for k in q.excludes}
        pins = {tuple(k) for k in q.pins}
        keys = {tuple(k) for k in keys}
        if action == "exclude":
            ex |= keys
        elif action == "include":
            ex -= keys
        elif action == "pin":
            pins |= keys
        elif action == "unpin":
            pins -= keys
        elif action == "only" and self.eventset is not None:
            ex |= {r.key for r in self.eventset.events} - keys
        elif action == "include_all":
            ex = set()
        self.events_panel.set_curation(sorted(ex), sorted(pins))

    def highlighted(self) -> list:
        """Selection numbers drawn thick: the browser's selection, then the hovered one."""
        keys = list(self.sel_keys) + ([self.hover_key] if self.hover_key is not None else [])
        return list(dict.fromkeys(self._num_of[k] for k in keys if k in self._num_of))

    def picked_event_selection(self) -> Optional[Selection]:
        """The event the spectra read: the first selected, else the first drawn."""
        ov = self.overlay
        if ov is None or not ov.selections:
            return None
        by = ov.by_number
        for k in self.sel_keys:
            n = self._num_of.get(k)
            if n in by:
                return by[n]
        return ov.drawn[0] if ov.drawn else ov.selections[0]

    def _extra_traces(self, numbers: list, vs: list) -> dict:
        """Traces of highlighted events beyond the drawn ones (one event: cheap)."""
        ov = self.overlay
        drawn = {s.number for s in ov.drawn}
        want = tuple(n for n in numbers if n not in drawn)
        key = (want, tuple(v.key for v in vs))
        got = self._extra.get(key)
        if got is None:
            by = ov.by_number
            got = {n: evs.event_traces(by[n], vs, self.loader.n_px) for n in want if n in by}
            self._extra = {key: got}
        return got

    def _render_events(self) -> None:
        vs = self._checked_vars()
        if tuple(v.key for v in vs) != self._ov_vars:
            self._request_load()
        ov = self.overlay
        hl = self.highlighted() if ov is not None else []
        if _open(self.compare):
            self.compare.render(ov, vs, self.colors, hl,
                                self._extra_traces(hl, vs) if ov is not None else None)
            self._sync_cursors()
            self._sync_head()
        self._render_plane()
        ds = self.dataset()
        grid_cells = (self.spectrum, self.spectrogram, self.at_cursors)
        if not ds.selections or not ds.variables:
            hint = ("Loading the events ..." if self.loader.busy else
                    "No events: widen the filter or the scope on the left." if ov is not None
                    else "Check variables for Plots on the left.")
            for cell in grid_cells:
                cell.show_hint(hint)
            self.spectrogram.data = None
            return
        shades = shades_of(ds)
        if _open(self.spectrum):
            self.spectrum.render(ds, shades)
        if _open(self.spectrogram) or _open(self.at_cursors):
            self.spectrogram.render(ds)
        if _open(self.at_cursors):
            self.at_cursors.render(self.spectrogram, [(n, t) for n, t in self.cursors])

    # ----- the timeline's manoeuvre lane (Selections mode)

    def _lane_clicked(self, t0: float, t1: float) -> None:
        sel = self.model.create(t0, t1)
        if isinstance(sel, str):
            self.statusBar().showMessage(f"No selection: {sel}", 4000)
            return
        self.include(sel)

    def _lane_compare(self, ev: dict) -> None:
        """Compare similar: an Events query with the event's type, class, mode and
        |u| +- 0.1, over the recording's folder (its day, and what else is there)."""
        got = self._manoeuvres_fn()
        rec = got[0] if got else None
        lvl = abs(evs._event_level(ev))
        mode = (ev.get("context") or {}).get("mode")
        q = evs.EventQuery(recordings=[str(Path(rec).parent)] if rec else [],
                           types=[ev.get("type", "onset")], classes=[ev.get("class", "")],
                           level_min=round(max(0.0, lvl - 0.1), 3),
                           level_max=round(min(1.0, lvl + 0.1), 3) if lvl + 0.1 < 1.0 else None,
                           modes=[mode] if mode else [], group_by="run",
                           pins=[[ev.get("_stem", ""), int(ev.get("id", -1))]] if ev.get("_stem") else [])
        self.compareSimilar.emit(q)

    def _render(self) -> None:
        scheduler().done(self)
        if not self.isVisible():
            return
        if self.mode == EVENTS:
            self._render_events()
            return
        src = timeline_source_of(self._source_fn)
        self._default_timeline(src)
        if _open(self.timeline):
            got = self._manoeuvres_fn()
            self.timeline.set_manoeuvres(got[1] if got else None)
            self.timeline.render(src, self.timeline_dataset())
        self._render_plane()
        ds = self.dataset()
        grid_cells = (self.compare, self.spectrum, self.spectrogram, self.at_cursors)
        if not ds.selections or not ds.variables:
            hint = ("No selections here yet: turn on Select (S) and drag in the timeline above"
                    " (or in the Line or IMU window)." if not len(self.model) else
                    "Include a selection (its box on the left)." if not ds.selections else
                    "Check variables for Plots on the left (whiskers, IMU, depth).")
            for cell in grid_cells:
                cell.show_hint(hint)
            self.spectrogram.data = None
            return
        shades = shades_of(ds)
        if _open(self.compare):
            self.compare.render(ds, shades)
            self._sync_cursors()                    # rows may have been rebuilt
            self._sync_head()
        if _open(self.spectrum):
            self.spectrum.render(ds, shades)
        # the spectrogram feeds the cursor cell: drawn when either shows
        if _open(self.spectrogram) or _open(self.at_cursors):
            self.spectrogram.render(ds)
        self.compare.set_picked(self.spectrogram.pick)
        self._fit_x(max((s.duration for s in ds.selections), default=0.0))
        if _open(self.at_cursors):
            self.at_cursors.render(self.spectrogram, [(n, t) for n, t in self.cursors])

    def _fit_x(self, longest: float) -> None:
        """The time cells' X (linked) from 0 to the longest selection unless zoomed by
        hand; set on a cell that is open (a collapsed one has no width to line up)."""
        if not self.compare._auto or longest <= 0:
            return
        if _open(self.compare) and self.compare.rows:
            self.compare.fit_x(longest)
        elif _open(self.spectrogram):
            self.spectrogram.plot.setXRange(0.0, longest, padding=0.01)

    def showEvent(self, event) -> None:  # noqa: N802 — Qt naming
        super().showEvent(event)
        QApplication.instance().installEventFilter(self)     # the keys (app-wide: only while shown)
        self._live_timer.start()
        self._rebuild_vars()
        self._lists_dirty = True
        self._sync_lists()
        self.timeline.show_position()
        self._request()

    def hideEvent(self, event) -> None:  # noqa: N802 — Qt naming
        QApplication.instance().removeEventFilter(self)
        self._live_timer.stop()
        super().hideEvent(event)

    def changeEvent(self, event) -> None:  # noqa: N802 — Qt naming
        super().changeEvent(event)
        if event.type() == event.Type.ActivationChange and self.isActiveWindow():
            self.activated.emit(self)

