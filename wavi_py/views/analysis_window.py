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
  Below them a tree with the rest, IMU (accel x / y / z, gyro x / y / z,
  attitude roll / pitch / yaw, depth) and ROV (the vehicle telemetry, when the
  source has it), with the same two checkbox columns: Plots (the 2x2) and
  Timeline (the timeline's own rows, independent). The Timeline column starts
  on Depth when the recording has depth, else on the first variable checked
  for Plots, until it is edited.

The cells read one ``daq/analysis.Dataset`` (the included selections x the
Plots variables, with the panel's Despike / V0 / scale and the sub-range),
made per redraw; they are in ``views/analysis_cells.py`` and
``views/analysis_timeline.py``. A collapsed cell is not drawn, and every cell
skips its work when its inputs did not change.

Cursors: a click in the spectrogram adds a numbered vertical cursor at that
relative time, drawn there and in the waveform; drag it to move it, right-click
it or Del (hovered) to delete it. In the waveform a click on a trace shows that
(variable, selection) in the spectrogram (the trace is then drawn thick), a click
on the thick trace adds a cursor, and a click away from the traces does nothing.
The bottom right shows the picked tile's spectrogram column at each cursor.

Toolbar: Select (S: a drag in the waveform makes the sub-range, in the
timeline a new selection), Clear sub-range (Esc), Zoom box (Z: a drag in the
waveform, the spectrum, the spectrogram or the cursor spectra zooms to the box
drawn, the waveform and spectrogram keeping one time span; R resets), Probe (P: hover readout in
the waveform, timeline, both spectra and the spectrogram), Reset view (R: the
cell under the mouse, or all), Keys. The Line window's zoom / pan keys act on
the cell under the mouse (= - X, + _ Y of the row under the mouse, . , Y of all
its rows, arrows move the data), matched by the character typed in an
application event filter while the window is shown, as the Line window does.

Several Analysis windows may be open, each with its own included selections,
variables, sizes, cursors and options (the control panel keeps them; the
newest focused one takes a region's Compare). Model changes only mark the
window dirty: the render scheduler redraws it at most ``MAX_HZ`` a second, and
not at all while it is hidden or closed (closing only hides it; the panel
reuses it for the next new window).
"""

from __future__ import annotations

import weakref
from typing import Callable, Optional, Sequence

from PyQt6.QtCore import QEvent, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QAction, QColor, QCursor, QIcon, QKeySequence, QPixmap
from PyQt6.QtWidgets import (QAbstractItemView, QAbstractSpinBox, QApplication, QComboBox,
                             QDoubleSpinBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMessageBox,
                             QSplitter, QSplitterHandle, QToolBar, QToolButton, QTreeWidget,
                             QTreeWidgetItem, QVBoxLayout, QWidget)

import numpy as np
import pyqtgraph as pg

from wavi_py.daq.analysis import DEPTH, ROV, Dataset, Selection, Variable
from wavi_py.daq.series import IMU, WSK
from wavi_py.views.analysis_cells import (SELECTION, SENSOR, CursorCell, CursorLines,
                                          SpecgramCell, SpectrumCell, WaveCell, shades_of)
from wavi_py.views.analysis_timeline import TimelineCell, timeline_source_of
from wavi_py.views.base import ViewWindow
from wavi_py.views.sensor_grid import GridLayout, SensorGrid
from wavi_py.views.render_sched import scheduler
from wavi_py.views.selections import SelectionModel, sel_color
from wavi_py.views.whisker_plots import _PAN_STEP, ZOOM_STEP

CELLS = ("Compare", "Spectrum", "Spectrogram", "Cursors")   # set_tab names (region menus)
PLOTS, TIMELINE = 1, 2                                      # the Variables tree's check columns
_KEY = Qt.ItemDataRole.UserRole
_LIVE_POLL_MS = 1000
TIMELINE_H = 190                                            # its default height (two rows)                                        # the timeline follows live data

KEYS_HELP = """\
<table cellspacing=4>
<tr><td><b>S</b></td><td>Select: drag in the waveform for another sub-range, in the timeline for a new selection</td></tr>
<tr><td><b>Esc</b></td><td>clear the sub-ranges (all)</td></tr>
<tr><td><b>Z</b></td><td>zoom box: drag a rectangle to zoom (waveform, spectra, spectrogram); off: a drag pans</td></tr>
<tr><td><b>P</b></td><td>probe: values under the mouse (waveform, timeline, spectra, spectrogram)</td></tr>
<tr><td><b>R</b></td><td>reset view: the cell under the mouse (outside the cells: all)</td></tr>
<tr><td><b>= &nbsp;-</b></td><td>zoom X in / out (the cell under the mouse). Keypad + - too</td></tr>
<tr><td><b>+ &nbsp;_</b></td><td>zoom Y in / out: the row under the mouse</td></tr>
<tr><td><b>. &nbsp;,</b></td><td>zoom Y in / out: all rows of the cell</td></tr>
<tr><td><b>arrows</b></td><td>move the data (Left: traces slide left), the cell under the mouse</td></tr>
<tr><td><b>Del</b></td><td>delete the cursor (else the sub-range, else the selection) under the mouse</td></tr>
</table>
Mouse: a click in the spectrogram adds a cursor. In the waveform a click on a trace
shows it in the spectrogram (drawn thick); a click on the thick trace adds a cursor.
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
    gauges by the Line window's sensor names, the IMU, depth); ``source_fn()`` is
    what the timeline shows (a ``SelSource``: the whole recording, the live
    buffers; or None / a str when there is none); ``replay``: the ``ReplayModel``
    (playhead, seek)."""

    MAX_HZ = 20.0
    activated = pyqtSignal(object)            # this window got the focus (newest-focused)

    def __init__(self, model: SelectionModel, variables_fn: Callable[[], list], number: int = 1,
                 source_fn: Optional[Callable] = None, replay=None,
                 dome_map_fn: Optional[Callable] = None):
        super().__init__()
        self.model = model
        self._variables_fn = variables_fn
        self._dome_map_fn = dome_map_fn or (lambda: None)
        self._wsk_checked: dict[int, set] = {PLOTS: set(), TIMELINE: set()}   # gauge keys
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
        self.setWindowTitle(f"Wavi - Analysis {number}")
        self.resize(1300, 860)
        ref = weakref.ref(self)                   # the scheduler must not keep it alive
        scheduler().register(self, lambda: (w := ref()) is not None and w._render(), self.MAX_HZ)
        self._build_toolbar()

        # side panel
        side = QWidget()
        sl = QVBoxLayout(side)
        sl.setContentsMargins(4, 4, 4, 4)
        sl.addWidget(QLabel("<b>Selections</b>"))
        self.sel_list = QTreeWidget()
        self.sel_list.setHeaderLabels(["#", "t0 (s)", "t1 (s)", ""])
        self.sel_list.setRootIsDecorated(False)
        self.sel_list.setToolTip("Include a selection (box); edit its times; ✕ deletes it")
        hdr = self.sel_list.header()
        hdr.setStretchLastSection(False)
        for c, mode in enumerate((QHeaderView.ResizeMode.ResizeToContents,
                                  QHeaderView.ResizeMode.Stretch, QHeaderView.ResizeMode.Stretch,
                                  QHeaderView.ResizeMode.ResizeToContents)):
            hdr.setSectionResizeMode(c, mode)
        self.sel_list.itemChanged.connect(self._on_sel_item)
        sl.addWidget(self.sel_list, 1)
        sl.addWidget(QLabel("<b>Variables</b>"))
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
        self.compare = WaveCell(lambda: self._act_select.isChecked())
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
        self.timeline.madeSelection.connect(self.include)
        self.subs: tuple = ()                         # the waveform's sub-ranges ((n, a, b), ...)
        self._wave_lines = CursorLines(self)
        self._spec_lines = CursorLines(self, halo=True)

        self.right = _Splitter(Qt.Orientation.Vertical)
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
        return [self.timeline, self.compare, self.spectrum, self.spectrogram, self.at_cursors]

    def default_sizes(self) -> None:
        """The layout a new window starts with: panel 290 px, timeline 150 px, the
        2x2 at 62 / 38 % across and half / half down."""
        w, h = max(800, self.width()), max(500, self.height())
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
        on = set(self._wsk_checked[column]) | {
            it.data(0, _KEY) for it in self._var_leaves()
            if it.checkState(column) == Qt.CheckState.Checked}
        return [v.key for v in self._vars if v.key in on]

    def timeline_keys(self) -> list[str]:
        return self.checked_keys(TIMELINE)

    def set_checked(self, keys: Sequence[str], on: bool = True, column: int = PLOTS) -> None:
        keys = set(keys)
        if column == PLOTS:                           # timeline-only variables have no Plots box
            keys -= {v.key for v in self._vars if not v.plots}
        wsk = {v.key for v in self._vars if v.source == WSK} & keys
        if on:
            self._wsk_checked[column] |= wsk
        else:
            self._wsk_checked[column] -= wsk
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
        """Colour by ``selection`` (default) or ``sensor`` (this window's own)."""
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
        self.setWindowTitle(f"Wavi - Analysis {number}")
        self._included = []
        self.set_checked([v.key for v in self._vars], False)
        self.set_checked([v.key for v in self._vars], False, TIMELINE)
        self._tl_edited = False
        self.set_color_by(SELECTION)
        self.set_sub(None)
        self.cursors = []
        self._next_cursor = 1
        self._sync_cursors()
        self.spectrogram.pick = None
        for cell in self.cells():
            cell.reset_view()
        self.default_sizes()
        self._lists_dirty = True

    def dataset(self) -> Dataset:
        keys = set(self.checked_keys())
        m = self.model
        return Dataset(self.included, [v for v in self._vars if v.key in keys],
                       despike=m.despike, v0=m.v0, scale=m.scale, subs=self.subs)

    def timeline_dataset(self) -> Dataset:
        keys = set(self.timeline_keys())
        m = self.model
        return Dataset([], [v for v in self._vars if v.key in keys],
                       despike=m.despike, v0=m.v0, scale=m.scale)

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
        if pick is None:
            return
        if pick == self.spectrogram.pick:
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
            g.set_checked(int(k[4:]) for k in self._wsk_checked[col])

    def _rebuild_grids(self) -> None:
        """The grids' layout from the dome map and the gauges there are."""
        wsk = [v for v in self._vars if v.source == WSK]
        dm = self._dome_map_fn()
        key = (id(dm), len(wsk), tuple(v.label for v in wsk))
        if key == self._grid_key:
            return
        self._grid_key = key
        grid = GridLayout.of(dm, max(1, (len(wsk) + 1) // 2), len(wsk),
                             [v.group for v in wsk[::2]])
        for g in self.grids.values():
            g.set_grid(grid)
            g.labels = {v.col: v.label for v in wsk}
        self.compare.grid = self.spectrum.grid = grid
        self._sync_grids()
        self._request()

    def _rebuild_vars(self) -> None:
        """The tree from ``variables_fn`` (kept checks by key); only when it changed."""
        new = list(self._variables_fn())
        if [v.key for v in new] == [v.key for v in self._vars] and \
                [v.label for v in new] == [v.label for v in self._vars]:
            self._rebuild_grids()
            return
        keep = {col: set(self.checked_keys(col)) for col in (PLOTS, TIMELINE)}
        self._vars = new
        have = {v.key for v in new if v.source == WSK}
        for col in (PLOTS, TIMELINE):
            self._wsk_checked[col] = keep[col] & have
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
            if v.source == WSK:
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
        """Zoom / pan keys by the character typed, anywhere in this window."""
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
                *self.timeline.probes.values(), self.spectrogram.probe, self.at_cursors.probe]

    def _on_probe_toggled(self, on: bool) -> None:
        if not on:
            for line in self._probe_lines():
                line.hide()
            self.statusBar().clearMessage()

    def _on_mouse(self, cell, pos) -> None:
        if cell is not self._hover_cell:
            for line in self._probe_lines():
                line.hide()
        self._hover_cell = cell
        if self._act_probe.isChecked():
            text = self.probe_text(cell, pos)
            if text:
                self.statusBar().showMessage(text)

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

    def _render(self) -> None:
        scheduler().done(self)
        if not self.isVisible():
            return
        src = timeline_source_of(self._source_fn)
        self._default_timeline(src)
        if _open(self.timeline):
            self.timeline.render(src, self.timeline_dataset())
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

