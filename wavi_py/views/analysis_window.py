"""The Analysis window: selections x variables, compared (C1), spectra (C2).

A side panel with two lists shared by the tabs:

- Selections: every selection of the app-wide model (``views/selections.py``),
  with an include box, its colour and number, t0 / t1 boxes (edits move it
  everywhere) and delete. Which are included is this window's own.
- Variables: a tree, Whiskers (by sensor, then gauge; a sensor's box checks
  both) and IMU (accel x / y / z, gyro x / y / z, attitude roll / pitch / yaw,
  depth). Also this window's own.

Tabs read one ``daq/analysis.Dataset`` (the included selections x the checked
variables, with the panel's Despike / V0 / scale), made per redraw:

- Compare: stacked rows, one per unit group (mV, mg, mrad/s, deg, cm: the IMU
  window's units), X linked; X is the time from each selection's start, so the
  selections overlay at t = 0. Colour is the selection (its region's colour), and
  the variables in a row are told apart by the shade of it, dark to light in the
  order they are listed (``shade_levels``). The legends: each row's variables in
  their shade (grey), the top row's also the selections in their colour. The curves are min / max
  decimated to the plot's width (``Dataset.traces``), from the store's pyramid in
  replay, so a long selection redraws as cheaply as a short one. X follows the
  longest selection and Y fits the data until a mouse zoom; R (Reset view) goes
  back to that.
- Spectrum, Spectrogram: phase C2 (empty tabs here).

Several Analysis windows may be open, each with its own included selections,
variables and tab (the control panel keeps them; the newest focused one takes a
region's Compare). Model changes only mark the window dirty: the render
scheduler redraws it at most ``MAX_HZ`` a second, and not at all while it is
hidden or closed (closing only hides it; the panel reuses it for the next new
window).
"""

from __future__ import annotations

import weakref
from typing import Callable, Optional, Sequence

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QAction, QColor, QIcon, QKeySequence, QPixmap
from PyQt6.QtWidgets import (QAbstractSpinBox, QDoubleSpinBox, QHeaderView, QLabel, QSplitter,
                             QStackedWidget, QTabWidget, QToolBar, QToolButton, QTreeWidget,
                             QTreeWidgetItem, QVBoxLayout, QWidget)

import pyqtgraph as pg

from wavi_py.daq.analysis import DEPTH, UNIT_GROUPS, Dataset, Selection, Variable
from wavi_py.daq.series import IMU, WSK
from wavi_py.views.base import ViewWindow
from wavi_py.views.render_sched import GraphicsLayoutWidget, scheduler
from wavi_py.views.selections import SelectionModel, sel_color
from wavi_py.views.whisker_plots import _show_grid

# a row's variables: HSL lightness of the selection's colour, dark to light (one
# variable keeps the colour itself); the light end stays readable as a 1 px line on white
_L_DARK, _L_LIGHT = 0.18, 0.66
TABS = ("Compare", "Spectrum", "Spectrogram")
_KEY = Qt.ItemDataRole.UserRole


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


def _swatch(color: QColor) -> QIcon:
    pm = QPixmap(12, 12)
    pm.fill(color)
    return QIcon(pm)


class CompareTab(QWidget):
    """Stacked rows by unit, relative time (see the module doc)."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self._pages = QStackedWidget()
        lay.addWidget(self._pages)
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
        self._auto = True                                # X follows the longest selection

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
        if units:
            self.rows[units[-1]].setLabel("bottom", "Time from the selection's start", units="s")

    def _on_manual(self, *_a) -> None:
        self._auto = False

    def show_hint(self, text: str) -> None:
        self._hint.setText(text)
        self._pages.setCurrentIndex(0)

    def render(self, ds: Dataset, shades: dict) -> None:
        """``shades``: the lightness of each variable (``shade_levels`` per row)."""
        units = [u for u in UNIT_GROUPS if any(v.unit == u for v in ds.variables)]
        if units != list(self.rows):
            self._rebuild_rows(units)
        self._pages.setCurrentIndex(1)
        first = self.rows[units[0]]
        n_px = max(50, int(first.getViewBox().width()) or 600)
        seen = set()
        for tr in ds.traces(n_px):
            key = (tr.sel.number, tr.var.key)
            seen.add(key)
            c = self.curves.get(key)
            if c is None:
                c = pg.PlotCurveItem()
                c.unit, c.look = tr.var.unit, None
                self.rows[tr.var.unit].addItem(c)
                self.curves[key] = c
            look = (tr.sel.number, shades[tr.var.key])
            if c.look != look:                  # the shade follows the checked order
                c.setPen(pg.mkPen(shade(sel_color(look[0]), look[1]), width=1))
                c.look = look
            c.setData(tr.t, tr.y)
        for key in [k for k in self.curves if k not in seen]:
            c = self.curves.pop(key)
            self.rows[c.unit].removeItem(c)
        self._legends(ds, shades, units)
        if self._auto:
            longest = max((s.duration for s in ds.selections), default=0.0)
            if longest > 0:
                first.setXRange(0.0, longest, padding=0.01)
        self.glw.scene().prepareForPaint()

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


class AnalysisWindow(ViewWindow):
    """See the module doc. ``variables_fn()`` lists the variables there are (the
    gauges by the Line window's sensor names, the IMU, depth)."""

    MAX_HZ = 20.0
    activated = pyqtSignal(object)            # this window got the focus (newest-focused)

    def __init__(self, model: SelectionModel, variables_fn: Callable[[], list], number: int = 1):
        super().__init__()
        self.model = model
        self._variables_fn = variables_fn
        self._vars: list[Variable] = []
        self._included: list[Selection] = []
        self._lists_dirty = True
        self.number = number
        self.setWindowTitle(f"Wavi - Analysis {number}")
        self.resize(1100, 700)
        ref = weakref.ref(self)                   # the scheduler must not keep it alive
        scheduler().register(self, lambda: (w := ref()) is not None and w._render(), self.MAX_HZ)

        tb = QToolBar("Analysis")
        tb.setMovable(False)
        self.addToolBar(tb)
        self._act_reset = QAction("Reset view", self)
        self._act_reset.setShortcut(QKeySequence("R"))
        self._act_reset.setToolTip("X: the longest selection, Y: fit the data [R]")
        self._act_reset.triggered.connect(self._reset_view)
        tb.addAction(self._act_reset)

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
        self.var_tree = QTreeWidget()
        self.var_tree.setHeaderHidden(True)
        self.var_tree.itemChanged.connect(lambda *_: self._request())
        sl.addWidget(self.var_tree, 2)

        self.tabs = QTabWidget()
        self.compare = CompareTab()
        self.tabs.addTab(self.compare, "Compare")
        for name in TABS[1:]:
            lab = QLabel(f"{name}: coming in phase C2.")
            lab.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.tabs.addTab(lab, name)
        self.tabs.currentChanged.connect(lambda _i: self._request())

        split = QSplitter()
        split.addWidget(side)
        split.addWidget(self.tabs)
        split.setStretchFactor(1, 1)
        split.setSizes([290, 810])
        self.setCentralWidget(split)

        model.added.connect(self._on_added)
        model.removed.connect(self._on_removed)
        model.changed.connect(self._on_changed)
        model.dataChanged.connect(self._on_data_changed)
        self._rebuild_vars()

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
        self._lists_dirty = True
        self._sync_lists()
        self._request()

    def checked_keys(self) -> list[str]:
        return [it.data(0, _KEY) for it in self._var_leaves()
                if it.checkState(0) == Qt.CheckState.Checked]

    def set_checked(self, keys: Sequence[str], on: bool = True) -> None:
        keys = set(keys)
        self.var_tree.blockSignals(True)
        for it in self._var_leaves():
            if it.data(0, _KEY) in keys:
                it.setCheckState(0, Qt.CheckState.Checked if on else Qt.CheckState.Unchecked)
        self.var_tree.blockSignals(False)
        self._fix_parents()
        self._request()

    def set_tab(self, name: str) -> None:
        self.tabs.setCurrentIndex(TABS.index(name))

    def reset_state(self, number: int) -> None:
        """A closed window reused as a new one: nothing included or checked."""
        self.number = number
        self.setWindowTitle(f"Wavi - Analysis {number}")
        self._included = []
        self.set_checked([v.key for v in self._vars], False)
        self.set_tab("Compare")
        self.compare.reset_view()
        self._lists_dirty = True

    def dataset(self) -> Dataset:
        keys = set(self.checked_keys())
        m = self.model
        return Dataset(self.included, [v for v in self._vars if v.key in keys],
                       despike=m.despike, v0=m.v0, scale=m.scale)

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

        def walk(it: QTreeWidgetItem) -> Optional[Qt.CheckState]:
            if it.childCount() == 0:
                return it.checkState(0)
            states = {walk(it.child(i)) for i in range(it.childCount())}
            st = (states.pop() if len(states) == 1 else Qt.CheckState.PartiallyChecked)
            if it is not self.var_tree.invisibleRootItem():
                it.setCheckState(0, st)
            return st
        walk(self.var_tree.invisibleRootItem())
        self.var_tree.blockSignals(False)

    def _rebuild_vars(self) -> None:
        """The tree from ``variables_fn`` (kept checks by key); only when it changed."""
        new = list(self._variables_fn())
        if [v.key for v in new] == [v.key for v in self._vars] and \
                [v.label for v in new] == [v.label for v in self._vars]:
            return
        keep = set(self.checked_keys())
        self._vars = new
        self.var_tree.blockSignals(True)
        self.var_tree.clear()
        flags = Qt.ItemFlag.ItemIsUserCheckable | Qt.ItemFlag.ItemIsEnabled
        tri = flags | Qt.ItemFlag.ItemIsAutoTristate

        def node(parent, text: str) -> QTreeWidgetItem:
            it = QTreeWidgetItem(parent, [text])
            it.setFlags(tri)
            it.setCheckState(0, Qt.CheckState.Unchecked)
            return it
        tops = {WSK: node(self.var_tree, "Whiskers"), IMU: node(self.var_tree, "IMU")}
        groups: dict = {}
        for v in new:
            side = WSK if v.source == WSK else IMU
            top = tops[side]
            if v.source == DEPTH:
                parent = top
            else:
                parent = groups.get((side, v.group))
                if parent is None:
                    parent = groups[(side, v.group)] = node(top, v.group)
            it = QTreeWidgetItem(parent, [v.label.split(" ")[-1] if v.source != DEPTH else v.label])
            it.setFlags(flags)
            it.setData(0, _KEY, v.key)
            it.setToolTip(0, f"{v.label} ({v.unit})")
            it.setCheckState(0, Qt.CheckState.Checked if v.key in keep else Qt.CheckState.Unchecked)
        tops[IMU].setExpanded(True)
        tops[WSK].setExpanded(True)
        self.var_tree.blockSignals(False)
        self._fix_parents()

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
            elif not on and sel in self._included:
                self._included.remove(sel)
            self._request()

    # ----- drawing

    def _request(self) -> None:
        scheduler().request(self)

    def _reset_view(self) -> None:
        self.compare.reset_view()
        self._request()

    @staticmethod
    def _shades(ds: Dataset) -> dict:
        """Each variable's lightness: dark to light within its row, in the listed order."""
        out = {}
        for u in UNIT_GROUPS:
            vs = [v for v in ds.variables if v.unit == u]
            out.update(zip((v.key for v in vs), shade_levels(len(vs))))
        return out

    def _render(self) -> None:
        scheduler().done(self)
        if not self.isVisible():
            return
        if self.tabs.currentIndex() != 0:
            return                                  # C2's tabs
        ds = self.dataset()
        if not ds.selections or not ds.variables:
            self.compare.show_hint(
                "No selections here yet: in the Line or IMU window, turn on Select (S), drag "
                "over a time span, then right-click it → Compare." if not len(self.model) else
                "Include a selection (its box on the left)." if not ds.selections else
                "Check variables on the left (whiskers, IMU, depth).")
            return
        self.compare.render(ds, self._shades(ds))

    def showEvent(self, event) -> None:  # noqa: N802 — Qt naming
        super().showEvent(event)
        self._rebuild_vars()
        self._lists_dirty = True
        self._sync_lists()
        self._request()

    def changeEvent(self, event) -> None:  # noqa: N802 — Qt naming
        super().changeEvent(event)
        if event.type() == event.Type.ActivationChange and self.isActiveWindow():
            self.activated.emit(self)
