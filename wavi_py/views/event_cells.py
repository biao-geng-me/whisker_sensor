"""The Analysis window's Events mode: the event waveform and the event browser.

``EventWaveCell`` (the 2x2's top left in Events mode) draws an
``replay/eventset.Overlay`` (computed off the GUI thread): one panel per
variable, a position's drag (solid) and lift (dashed) sharing one panel; time
from each event's onset (t = 0, the stick, a dashed line). Colour is the group.
Each group's drawn events are thin lines (one curve per group and variable,
NaN between events; fainter the more events), its median over every event a
thick darker line with a white outline on top of them, and the 25-75 % band
of the solid variables a light fill. The highlighted events (the browser's
selection, a click here) are drawn thick in their group's colour on top. The
header says how many are drawn ("drawn 200 of 340": ``Dataset.drawn``, the
aggregates use all). Dome grid (the header's toggle, on by default): the
positions sit where they are on the dome as seen from behind (columns LB, LF,
RF, RB; rows U, M, L; one y range per row), the other variables in a row above;
off, the panels fill a grid in the checked order. A click on a thin trace
(within ``PICK_PX``) picks that event (``eventClicked``). Probe reads the
medians and the highlighted events of the panel under the mouse.

``EventBrowserCell`` (in place of the timeline): a table (one row per event:
group swatch with the include box, recording, t, type, class, level, mode,
depth, flags, and one feature chosen in the header) beside a scatter (x: a
field or a feature; y: the feature; colour: the group). The two share one
selection (``selectionChanged``): a click selects, Ctrl / Shift add, the
scatter's Lasso (header toggle) selects what a drawn outline encloses. Hover
(a row, a point) highlights (``hovered``). The context menu (both halves)
offers Exclude / Include / Pin / Unpin / Show only these / Include all
(``curate(action, keys)``; the window puts it into the query), and a
double-click opens the recording in the replay at the event (``openEvent``).
The header counts the events per group.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
from PyQt6.QtCore import QAbstractTableModel, QModelIndex, QPointF, Qt, pyqtSignal
from PyQt6.QtGui import QAction, QColor, QIcon, QPainter, QPixmap, QPolygonF
from PyQt6.QtWidgets import (QAbstractItemView, QCheckBox, QComboBox, QHBoxLayout, QHeaderView,
                             QLabel, QMenu, QSizePolicy, QSplitter, QTableView, QToolButton,
                             QVBoxLayout, QWidget)

import pyqtgraph as pg

from wavi_py.daq.analysis import DOME_COLS, DOME_ROWS, POS
from wavi_py.replay import eventset as evs
from wavi_py.views.analysis_cells import AXIS_W, PICK_PX, _Cell, _hook_drag, hook_click, probe_line
from wavi_py.views.whisker_plots import _show_grid

# group colours: distinct, none the playhead red
GROUP_COLORS = ((31, 119, 180), (255, 127, 14), (44, 160, 44), (148, 103, 189), (23, 190, 207),
                (140, 86, 75), (227, 119, 194), (127, 127, 127), (188, 189, 34), (214, 39, 40))
_ZERO_PEN = pg.mkPen((90, 90, 90), width=1, style=Qt.PenStyle.DashLine)
_OUTLINE = pg.mkPen((255, 255, 255, 230), width=5)
Z_THIN, Z_BAND, Z_OUTLINE, Z_MEDIAN, Z_HIGHLIGHT = 0, 2, 8, 9, 12
PANEL_MIN_H = 70


def group_color(i: int) -> QColor:
    return QColor(*GROUP_COLORS[i % len(GROUP_COLORS)])


def group_colors(groups) -> dict:
    """``{group: QColor}`` in the order given."""
    return {g: group_color(i) for i, g in enumerate(groups)}


def median_color(c: QColor) -> QColor:
    """The median's: the group colour, darker (it must stand out from its traces)."""
    h, s, _l, _a = c.getHslF()
    return QColor.fromHslF(max(h, 0.0), min(1.0, s * 1.05), 0.27)


def group_label(g: str) -> str:
    """A group's short name: a run (a stem) as "11-05 13:03 tag", else as it is."""
    from wavi_py.replay import recname
    rn = recname.parse(g) if isinstance(g, str) and g.startswith(("st_", "whisker_")) else None
    return f"{rn.start:%m-%d %H:%M} {rn.tag}".strip() if rn else str(g)


def thin_alpha(n: int) -> int:
    """The single traces' alpha for a group of ``n`` drawn events."""
    return int(min(150, max(22, 700 / max(n, 1))))


def _swatch(c: QColor, size: int = 11) -> QIcon:
    pm = QPixmap(size, size)
    pm.fill(c)
    return QIcon(pm)


def panel_key(v) -> str:
    """The panel a variable is drawn in: its position (drag and lift share one), else
    the variable itself."""
    return v.pos if v.source == POS else v.key


# ----------------------------------------------------------------------
# the waveform
# ----------------------------------------------------------------------

class EventWaveCell(_Cell):
    """See the module doc. ``eventClicked(number)``: a click picked that event's trace."""

    TITLE = "Waveform"
    eventClicked = pyqtSignal(int)
    clicked = pyqtSignal(float, object)          # as WaveCell's (the window's cursor code)
    subChanged = pyqtSignal(object)              # (no sub-range here; WaveCell's signals)
    xChanged = pyqtSignal(float, float)          # the time range shown (the spectrogram copies it)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        h = self._add_header(sub_line=False)
        self.title = QLabel("Events · t = 0 at the stick")
        self.title.setStyleSheet("color: #444;")
        h.row.addWidget(self.title)
        self.note = QLabel("")
        self.note.setStyleSheet("color: #666;")
        h.row.addWidget(self.note)
        h.row.addStretch(1)
        self.dome_box = h.add("", QCheckBox("Dome grid"))
        self.dome_box.setChecked(True)
        self.dome_box.setToolTip("Positions where they sit on the dome, seen from behind (LB LF RF"
                                 " RB across, U M L down; one y range per row)")
        self.band_box = h.add("", QCheckBox("25-75 %"))
        self.band_box.setChecked(True)
        self.band_box.setToolTip("Each group's 25-75 % band (solid variables)")
        self.thin_box = h.add("", QCheckBox("Events"))
        self.thin_box.setChecked(True)
        self.thin_box.setToolTip("The single events (thin); off: only the medians and bands")
        for b in (self.dome_box, self.band_box, self.thin_box):
            b.toggled.connect(lambda _on: (self._invalidate(), self.changed.emit()))
        self.glw.ci.setSpacing(4)
        self.panels: dict[str, pg.PlotItem] = {}
        self.probes: dict[str, pg.InfiniteLine] = {}
        self._layout_key: tuple = ()
        self._items: list = []                       # (plot, item) drawn for the overlay
        self._hl_items: list = []                    # (plot, item) of the highlight
        self._thin: dict = {}                        # panel -> [(curve, owner array)]
        self._medians: dict = {}                     # panel -> [(label, unit, curve)]
        self._hl_curves: dict = {}                   # panel -> [(label, unit, curve)]
        self.overlay: Optional[evs.Overlay] = None
        self.colors: dict = {}
        self.highlight: tuple = ()
        self._auto = True
        self._drawn_key = None

    # rows: the window's cursor lines go into every panel
    @property
    def rows(self) -> dict:
        return self.panels

    @property
    def picked(self):
        return None

    def set_picked(self, _pick) -> None:
        pass

    def set_sub(self, _sub, emit: bool = False) -> None:
        pass

    sub = None

    def nav_plots(self) -> list[pg.PlotItem]:
        return list(self.panels.values()) if self._pages.currentIndex() == 1 else []

    def x_plot(self) -> Optional[pg.PlotItem]:
        return next(iter(self.panels.values()), None)

    def on_manual(self) -> None:
        self._auto = False

    def reset_view(self) -> None:
        super().reset_view()
        self._auto = True
        self._drawn_key = None
        for p in self.panels.values():
            p.enableAutoRange(y=True)

    def fit_x(self, longest: float) -> None:
        pass

    def set_x(self, lo: float, hi: float) -> None:
        p = self.x_plot()
        if p is not None:
            x0, x1 = p.getViewBox().viewRange()[0]
            if abs(x0 - lo) > 1e-9 or abs(x1 - hi) > 1e-9:
                p.setXRange(lo, hi, padding=0)

    def _invalidate(self) -> None:
        self._last = None
        self._drawn_key = None

    # ----- layout

    def _grid(self, variables) -> list[tuple[str, int, int, str]]:
        """[(panel key, row, col, title)] for the checked variables."""
        keys, titles = [], {}
        for v in variables:
            k = panel_key(v)
            if k not in titles:
                keys.append(k)
                titles[k] = v.pos if v.source == POS else v.label
        pos = [k for k in keys if k in {c + r for r in DOME_ROWS for c in DOME_COLS}]
        other = [k for k in keys if k not in pos]
        out = []
        if self.dome_box.isChecked() and pos:
            ncol = len(DOME_COLS)
            for i, k in enumerate(other):
                out.append((k, i // ncol, i % ncol, titles[k]))
            r0 = -(-len(other) // ncol)
            for k in pos:
                out.append((k, r0 + DOME_ROWS.index(k[2]), DOME_COLS.index(k[:2]), titles[k]))
            return out
        n = len(keys)
        ncol = 1 if n <= 2 else min(4, math.ceil(math.sqrt(n)))
        return [(k, i // ncol, i % ncol, titles[k]) for i, k in enumerate(keys)]

    def _layout(self, variables) -> None:
        grid = self._grid(variables)
        key = (tuple(grid), self.dome_box.isChecked())
        if key == self._layout_key:
            return
        self._layout_key = key
        self.glw.clear()
        self.panels, self.probes, self._items, self._hl_items = {}, {}, [], []
        first = None
        row_first: dict = {}
        nrows = 1 + max((r for _k, r, _c, _t in grid), default=0)
        dome = self.dome_box.isChecked()
        for k, r, c, title in grid:
            p = self.glw.addPlot(row=r, col=c)
            p.setTitle(title, size="9pt", color="#333")
            _show_grid(p)
            p.setMenuEnabled(False)
            p.hideButtons()
            p.getAxis("left").enableAutoSIPrefix(False)
            p.getAxis("left").setWidth(AXIS_W if c == 0 else 38)
            p.setMinimumHeight(PANEL_MIN_H)
            if r == nrows - 1:
                p.setLabel("bottom", "s from the onset")
            if first is None:
                first = p
                p.getViewBox().sigXRangeChanged.connect(
                    lambda _vb, rg: self.xChanged.emit(float(rg[0]), float(rg[1])))
            else:
                p.setXLink(first)
            if dome and len(k) == 3 and k[2] in DOME_ROWS:
                if r in row_first:
                    p.setYLink(row_first[r])
                else:
                    row_first[r] = p
            zero = pg.InfiniteLine(0.0, angle=90, movable=False, pen=_ZERO_PEN)
            zero.setZValue(-5)
            p.addItem(zero, ignoreBounds=True)
            p.getViewBox().sigRangeChangedManually.connect(lambda *_: self.on_manual())
            hook_click(p.getViewBox(), self._click)
            self.panels[k] = p
            self.probes[k] = probe_line(p)
        self._drawn_key = None

    # ----- drawing

    def render(self, ov: Optional[evs.Overlay], variables, colors: dict, highlight=(),
               extra: Optional[dict] = None) -> None:
        """``ov``: the overlay (None: not loaded yet); ``variables``: the checked ones
        (the panels); ``colors``: group -> QColor; ``highlight``: selection numbers
        drawn thick; ``extra``: ``{number: {key: (t, y)}}`` of highlighted events
        beyond the drawn ones."""
        if not variables:
            self.show_hint("Check variables for Plots on the left (Positions, Demand / Thrust, IMU).")
            return
        if ov is None:
            self.show_hint("Loading the events' traces ...")
            return
        self._pages.setCurrentIndex(1)
        self._layout(variables)
        dkey = (id(ov), tuple(v.key for v in variables), tuple(colors.items()),
                self.band_box.isChecked(), self.thin_box.isChecked(), self._layout_key)
        if dkey != self._drawn_key:
            self._drawn_key = dkey
            self.renders += 1
            self._draw(ov, variables, colors)
        hkey = (dkey, tuple(highlight), id(extra))
        if hkey != self._last:
            self._last = hkey
            self._draw_highlight(ov, variables, colors, highlight, extra or {})
        self.glw.scene().prepareForPaint()

    def _add(self, plot, item, z) -> None:
        item.setZValue(z)
        plot.addItem(item)
        self._items.append((plot, item))

    def _draw(self, ov: evs.Overlay, variables, colors: dict) -> None:
        for p, it in self._items:
            p.removeItem(it)
        self._items, self._thin, self._medians = [], {}, {}
        self.overlay, self.colors = ov, colors
        lo, hi = math.inf, -math.inf
        for s in ov.selections:
            lo, hi = min(lo, s.t0 - s.zero), max(hi, s.t1 - s.zero)
        for v in variables:
            pk = panel_key(v)
            p = self.panels.get(pk)
            if p is None:
                continue
            dash = v.axis == "lift"
            style = Qt.PenStyle.DashLine if dash else Qt.PenStyle.SolidLine
            for g, c in colors.items():
                got = ov.thin.get((g, v.key))
                if got is not None and self.thin_box.isChecked():
                    t, y, owner = got
                    cc = QColor(c)
                    cc.setAlpha(thin_alpha(len(ov.drawn)))      # all groups overlap
                    curve = pg.PlotCurveItem(t, y, connect="finite",
                                             pen=pg.mkPen(cc, width=1, style=style))
                    self._add(p, curve, Z_THIN)
                    self._thin.setdefault(pk, []).append((curve, owner))
                for a in ov.aggregates.get(v.key, []):
                    if a.group != g:
                        continue
                    if self.band_box.isChecked() and not dash:
                        bc = QColor(c)
                        bc.setAlpha(55)
                        top = pg.PlotCurveItem(a.t, np.nan_to_num(a.q75, nan=0.0), pen=None)
                        bot = pg.PlotCurveItem(a.t, np.nan_to_num(a.q25, nan=0.0), pen=None)
                        band = pg.FillBetweenItem(top, bot, brush=pg.mkBrush(bc))
                        self._add(p, band, Z_BAND)
                    outline = pg.PlotCurveItem(a.t, a.median, connect="finite", pen=_OUTLINE)
                    self._add(p, outline, Z_OUTLINE)
                    med = pg.PlotCurveItem(a.t, a.median, connect="finite",
                                           pen=pg.mkPen(median_color(c), width=2.6, style=style))
                    self._add(p, med, Z_MEDIAN)
                    self._medians.setdefault(pk, []).append(
                        (f"{g} median {v.label}", v.unit, med))
        # no legend in the panels (it covers the traces): the browser's header and
        # the table's swatches give the groups' colours and counts
        first = self.x_plot()
        drawn, total = len(ov.drawn), len(ov.selections)
        self.note.setText(f"· {total} events" + (f", drawn {drawn} of {total}" if drawn < total
                                                  else "") + " · median thick, events thin")
        if self._auto and math.isfinite(lo) and first is not None:
            first.setXRange(lo, hi, padding=0.01)

    def _draw_highlight(self, ov, variables, colors, highlight, extra) -> None:
        for p, it in self._hl_items:
            p.removeItem(it)
        self._hl_items, self._hl_curves = [], {}
        self.highlight = tuple(highlight)
        by_num = ov.by_number
        for n in highlight:
            sel = by_num.get(n)
            if sel is None:
                continue
            c = colors.get(sel.group, QColor(0, 0, 0))
            for v in variables:
                p = self.panels.get(panel_key(v))
                got = ov.per_event.get((n, v.key)) or extra.get(n, {}).get(v.key)
                if p is None or got is None:
                    continue
                style = Qt.PenStyle.DashLine if v.axis == "lift" else Qt.PenStyle.SolidLine
                cc = QColor(c).darker(115)
                curve = pg.PlotCurveItem(got[0], got[1], connect="finite",
                                         pen=pg.mkPen(cc, width=2.4, style=style))
                curve.setZValue(Z_HIGHLIGHT)
                p.addItem(curve)
                self._hl_items.append((p, curve))
                self._hl_curves.setdefault(panel_key(v), []).append(
                    (f"{sel.label or '#' + str(n)} {v.label}", v.unit, curve))

    # ----- mouse

    def pick_at(self, vb: pg.ViewBox, scene_pos) -> Optional[int]:
        """The selection number of the thin trace within ``PICK_PX`` of ``scene_pos``."""
        pk = next((k for k, p in self.panels.items() if p.getViewBox() is vb), None)
        if pk is None:
            return None
        pt = vb.mapSceneToView(scene_pos)
        sx, sy = vb.viewPixelSize()
        best, best_d = None, PICK_PX + 1e-9
        for curve, owner in self._thin.get(pk, []):
            x, y = curve.getData()
            if x is None or not len(x):
                continue
            near = np.flatnonzero(np.abs(x - pt.x()) <= PICK_PX * sx * 1.5)
            if not len(near):
                continue
            d = np.hypot((x[near] - pt.x()) / sx, (y[near] - pt.y()) / sy)
            ok = np.isfinite(d)
            if not ok.any():
                continue
            j = int(np.nanargmin(np.where(ok, d, np.inf)))
            if d[j] < best_d:
                best, best_d = int(owner[near[j]]), float(d[j])
        return best

    def _click(self, vb: pg.ViewBox, ev) -> bool:
        if ev.button() != Qt.MouseButton.LeftButton or ev.double():
            return False
        ev.accept()
        n = self.pick_at(vb, ev.scenePos())
        if n is not None:
            self.eventClicked.emit(n)
        return True

    def probe_items(self, plot: pg.PlotItem):
        pk = next((k for k, p in self.panels.items() if p is plot), None)
        if pk is None:
            return None
        items = self._hl_curves.get(pk, []) + self._medians.get(pk, [])
        return "t", "s", (lambda x: x), (lambda y: y), items


# ----------------------------------------------------------------------
# the browser: table + scatter
# ----------------------------------------------------------------------

TABLE_COLS = ("", "recording", "t (s)", "type", "class", "level", "mode", "depth", "flags", "feature")
C_USE, C_REC, C_T, C_TYPE, C_CLASS, C_LEVEL, C_MODE, C_DEPTH, C_FLAGS, C_FEAT = range(len(TABLE_COLS))


def flags_text(r: evs.EventRef) -> str:
    out = []
    if r.pinned:
        out.append("pin")
    if r.baseline_clean:
        out.append("clean")
    if r.steady:
        out.append("steady")
    if r.minor:
        out.append("minor " + ",".join(f"{k}{float(v):+.2f}" for k, v in r.minor.items()))
    if r.no_reference:
        out.append("no ref")
    lost = [k for k, a in r.alive.items() if a is False]
    if lost:
        out.append("lost " + ",".join(lost))
    return " · ".join(out)


def rec_text(r: evs.EventRef) -> str:
    from wavi_py.replay import recname
    rn = recname.parse(r.stem)
    return f"{rn.start:%m-%d %H:%M} {rn.tag}" if rn else r.stem


class EventTableModel(QAbstractTableModel):
    """Rows: the set's events, then the excluded ones (unchecked, grey)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.refs: list[evs.EventRef] = []
        self.colors: dict = {}
        self.feature = ("LFM drag", "peak")
        self._icons: dict = {}
        self._feat: list = []
        self.toggled = None                         # callback(key, include)

    def set_rows(self, refs, colors, feature) -> None:
        self.beginResetModel()
        self.refs, self.colors, self.feature = list(refs), dict(colors), feature
        self._icons = {g: _swatch(c) for g, c in self.colors.items()}
        self._feat = [evs.feature(r, *feature) for r in self.refs]
        self.endResetModel()

    def rowCount(self, parent=QModelIndex()) -> int:  # noqa: N802 — Qt naming
        return 0 if parent.isValid() else len(self.refs)

    def columnCount(self, parent=QModelIndex()) -> int:  # noqa: N802 — Qt naming
        return len(TABLE_COLS)

    def headerData(self, section, orientation, role=Qt.ItemDataRole.DisplayRole):  # noqa: N802
        if orientation == Qt.Orientation.Horizontal and role == Qt.ItemDataRole.DisplayRole:
            if section == C_FEAT:
                return f"{self.feature[0]} {self.feature[1]}"
            return TABLE_COLS[section]
        return None

    def _value(self, r: evs.EventRef, row: int, col: int):
        return {C_USE: r.group, C_REC: rec_text(r), C_T: r.t, C_TYPE: r.type, C_CLASS: r.cls,
                C_LEVEL: r.level, C_MODE: r.mode or "", C_DEPTH: r.depth_cm,
                C_FLAGS: flags_text(r), C_FEAT: self._feat[row]}[col]

    def data(self, index, role=Qt.ItemDataRole.DisplayRole):
        if not index.isValid():
            return None
        r = self.refs[index.row()]
        col = index.column()
        if role == Qt.ItemDataRole.DisplayRole:
            v = self._value(r, index.row(), col)
            if col == C_USE:
                return ""
            if isinstance(v, float):
                return f"{v:.3g}" if col == C_FEAT else f"{v:.2f}" if col in (C_T, C_LEVEL) else f"{v:.0f}"
            return "" if v is None else str(v)
        if role == Qt.ItemDataRole.CheckStateRole and col == C_USE:
            return Qt.CheckState.Unchecked if r.excluded else Qt.CheckState.Checked
        if role == Qt.ItemDataRole.DecorationRole and col == C_USE:
            return self._icons.get(r.group)
        if role == Qt.ItemDataRole.ForegroundRole and r.excluded:
            return QColor(150, 150, 150)
        if role == Qt.ItemDataRole.ToolTipRole:
            return f"{r.stem}  #{r.id}\n{r.label}\n{flags_text(r)}"
        return None

    def flags(self, index):
        f = Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable
        if index.column() == C_USE:
            f |= Qt.ItemFlag.ItemIsUserCheckable
        return f

    def setData(self, index, value, role=Qt.ItemDataRole.EditRole):  # noqa: N802 — Qt naming
        if role == Qt.ItemDataRole.CheckStateRole and index.column() == C_USE:
            r = self.refs[index.row()]
            include = Qt.CheckState(value) == Qt.CheckState.Checked
            if self.toggled is not None:
                self.toggled(r.key, include)
            return True
        return False

    def sort(self, column: int, order=Qt.SortOrder.AscendingOrder) -> None:
        rev = order == Qt.SortOrder.DescendingOrder
        rows = list(range(len(self.refs)))

        def k(i):
            v = self._value(self.refs[i], i, column)
            if v is None or (isinstance(v, float) and not math.isfinite(v)):
                return (1, 0.0, "")
            return (0, v, "") if isinstance(v, (int, float)) else (0, 0.0, str(v))
        rows.sort(key=k, reverse=rev)
        self.layoutAboutToBeChanged.emit()
        self.refs = [self.refs[i] for i in rows]
        self._feat = [self._feat[i] for i in rows]
        self.layoutChanged.emit()


class _FeaturePicker(QWidget):
    """A feature: its source (a field, a position's axis, a motion channel) and quantity."""

    changed = pyqtSignal()

    def __init__(self, fields=(), source="LFM drag", quantity="steady", parent=None):
        super().__init__(parent)
        h = QHBoxLayout(self)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(2)
        self.src = QComboBox()
        self.src.addItems([*fields, *evs.feature_sources()])
        self.src.setCurrentText(source)
        self.qty = QComboBox()
        self.qty.addItems(list(evs.QUANTITIES))
        self.qty.setCurrentText(quantity)
        h.addWidget(self.src)
        h.addWidget(self.qty)
        self.src.currentIndexChanged.connect(self._on)
        self.qty.currentIndexChanged.connect(self._on)
        self._on()

    def _on(self, *_a) -> None:
        self.qty.setEnabled(self.src.currentText() not in evs.EVENT_FIELDS)
        self.changed.emit()

    def value(self) -> tuple[str, str]:
        return self.src.currentText(), self.qty.currentText()

    def label(self) -> str:
        s, q = self.value()
        return s if s in evs.EVENT_FIELDS else f"{s} {q}"


class EventBrowserCell(QWidget):
    """See the module doc. Keys are ``EventRef.key`` = (stem, id)."""

    selectionChanged = pyqtSignal(list)          # [key]
    hovered = pyqtSignal(object)                 # key or None
    curate = pyqtSignal(str, list)               # action, [key]
    openEvent = pyqtSignal(object)               # key
    TITLE = "Events"

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2)
        head = QHBoxLayout()
        head.setContentsMargins(4, 2, 4, 0)
        head.setSpacing(6)
        self.counts = QLabel("No events yet.")
        self.counts.setStyleSheet("color: #333;")
        self.counts.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        head.addWidget(self.counts, 1)
        head.addWidget(QLabel("feature (y):"))
        self.ypick = _FeaturePicker((), "LFM drag", "peak")
        head.addWidget(self.ypick)
        head.addWidget(QLabel("x:"))
        self.xpick = _FeaturePicker(evs.EVENT_FIELDS, "level", "steady")
        head.addWidget(self.xpick)
        self.lasso_btn = QToolButton()
        self.lasso_btn.setText("Lasso")
        self.lasso_btn.setCheckable(True)
        self.lasso_btn.setToolTip("Draw an outline in the scatter: the points inside are selected"
                                  " (right-click them: Exclude / Include / Pin / Show only these)")
        head.addWidget(self.lasso_btn)
        lay.addLayout(head)
        self.split = QSplitter(Qt.Orientation.Horizontal)
        self.model = EventTableModel(self)
        self.model.toggled = lambda key, inc: self.curate.emit("include" if inc else "exclude", [key])
        self.table = QTableView()
        self.table.setModel(self.model)
        self.table.setSortingEnabled(True)
        self.table.sortByColumn(C_REC, Qt.SortOrder.AscendingOrder)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self.table.setMouseTracking(True)
        self.table.verticalHeader().setVisible(False)
        self.table.verticalHeader().setDefaultSectionSize(20)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Interactive)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.table.customContextMenuRequested.connect(lambda pos: self._menu(
            self.table.viewport().mapToGlobal(pos)))
        self.table.entered.connect(lambda idx: self._hover(self.model.refs[idx.row()].key
                                                           if idx.isValid() else None))
        self.table.doubleClicked.connect(lambda idx: self.openEvent.emit(self.model.refs[idx.row()].key))
        self.table.selectionModel().selectionChanged.connect(self._table_selected)
        self.table.viewport().installEventFilter(self)
        self.plot = pg.PlotWidget(background="w")
        pi = self.plot.getPlotItem()
        pi.setMenuEnabled(False)
        pi.hideButtons()
        _show_grid(pi)
        pi.getAxis("left").enableAutoSIPrefix(False)
        pi.getAxis("bottom").enableAutoSIPrefix(False)
        self.scatter = pg.ScatterPlotItem(size=7, hoverable=True, hoverSize=11,
                                          hoverPen=pg.mkPen("k", width=1.5), tip=None)
        self.scatter.sigHovered.connect(self._scatter_hover)
        self.scatter.sigClicked.connect(self._scatter_click)
        pi.addItem(self.scatter)
        self.sel_marks = pg.ScatterPlotItem(size=12, pen=pg.mkPen("k", width=1.8), brush=None)
        self.sel_marks.setZValue(5)
        pi.addItem(self.sel_marks)
        self.lasso_curve = pg.PlotCurveItem(pen=pg.mkPen((0, 0, 0), width=1, style=Qt.PenStyle.DashLine))
        pi.addItem(self.lasso_curve, ignoreBounds=True)
        vb = pi.getViewBox()
        _hook_drag(vb, self._lasso_drag)
        hook_click(vb, self._plot_click)
        self.split.addWidget(self.table)
        self.split.addWidget(self.plot)
        self.split.setStretchFactor(0, 3)
        self.split.setStretchFactor(1, 2)
        lay.addWidget(self.split, 1)
        self.ypick.changed.connect(self._refeature)
        self.xpick.changed.connect(self._refeature)
        self.refs: list[evs.EventRef] = []
        self.colors: dict = {}
        self.selected: list = []
        self._xy = np.empty((0, 2))
        self._lasso: list = []
        self._syncing = False
        self.renders = 0

    # ----- data

    def set_events(self, es: Optional[evs.EventSet], colors: dict) -> None:
        refs = (es.events + es.excluded) if es is not None else []
        self.refs, self.colors = refs, colors
        keys = {r.key for r in refs}
        self.selected = [k for k in self.selected if k in keys]
        self.renders += 1
        if es is None:
            self.counts.setText("No events yet.")
        else:
            parts = [f"<span style='color:{colors[g].name()}'>■</span> {group_label(g)}: {n}"
                     for g, n in es.groups.items() if g in colors]
            more = f" · {len(es.excluded)} excluded" if es.excluded else ""
            miss = f" · {len(es.missing)} without a sidecar" if es.missing else ""
            self.counts.setText(f"<b>{len(es)}</b> events in {len(es.recordings)} recordings"
                                f"{more}{miss}: " + " &nbsp; ".join(parts))
            self.counts.setToolTip("<br>".join(parts))         # the label may be cut
        self._refeature()

    def _refeature(self) -> None:
        first = not self.model.refs
        self.model.set_rows(self.refs, self.colors, self.ypick.value())
        if first and self.refs:                   # widths once, from the first rows
            self.table.resizeColumnsToContents()
            hdr = self.table.horizontalHeader()
            hdr.resizeSection(C_FLAGS, min(hdr.sectionSize(C_FLAGS), 160))
        self.model.headerDataChanged.emit(Qt.Orientation.Horizontal, C_FEAT, C_FEAT)
        hdr = self.table.horizontalHeader()
        if hdr.sortIndicatorSection() >= 0 and self.table.isSortingEnabled():
            self.model.sort(hdr.sortIndicatorSection(), hdr.sortIndicatorOrder())
        xs, ys = [], []
        for r in self.refs:
            x, y = evs.feature(r, *self.xpick.value()), evs.feature(r, *self.ypick.value())
            xs.append(np.nan if x is None else float(x))
            ys.append(np.nan if y is None else float(y))
        self._xy = np.column_stack([xs, ys]) if self.refs else np.empty((0, 2))
        spots = []
        for i, r in enumerate(self.refs):
            if not (np.isfinite(self._xy[i]).all()):
                continue
            c = QColor(self.colors.get(r.group, QColor(0, 0, 0)))
            if r.excluded:
                c.setAlpha(120)
            spots.append({"pos": tuple(self._xy[i]), "data": i, "brush": pg.mkBrush(c),
                          "pen": pg.mkPen(255, 255, 255, 200) if not r.pinned else pg.mkPen("k", width=2),
                          "symbol": "o" if not r.excluded else "x"})
        self.scatter.setData(spots)
        pi = self.plot.getPlotItem()
        pi.setLabel("bottom", self.xpick.label())
        pi.setLabel("left", self.ypick.label())
        self._show_selected()

    # ----- selection (shared by the table and the scatter)

    def set_selected(self, keys, emit: bool = False) -> None:
        keys = list(dict.fromkeys(keys))
        self.selected = keys
        self._syncing = True
        sm = self.table.selectionModel()
        sm.clearSelection()
        want = set(keys)
        first = None
        for row, r in enumerate(self.model.refs):
            if r.key in want:
                sm.select(self.model.index(row, 0), sm.SelectionFlag.Select | sm.SelectionFlag.Rows)
                first = row if first is None else first
        if first is not None:
            self.table.scrollTo(self.model.index(first, 0))
        self._syncing = False
        self._show_selected()
        if emit:
            self.selectionChanged.emit(keys)

    def _show_selected(self) -> None:
        idx = {r.key: i for i, r in enumerate(self.refs)}
        pts = [tuple(self._xy[idx[k]]) for k in self.selected
               if k in idx and np.isfinite(self._xy[idx[k]]).all()]
        self.sel_marks.setData(pos=pts) if pts else self.sel_marks.clear()

    def _table_selected(self, *_a) -> None:
        if self._syncing:
            return
        rows = sorted({i.row() for i in self.table.selectionModel().selectedRows()})
        self.selected = [self.model.refs[r].key for r in rows]
        self._show_selected()
        self.selectionChanged.emit(self.selected)

    def _hover(self, key) -> None:
        self.hovered.emit(key)

    def eventFilter(self, obj, ev) -> bool:  # noqa: N802 — Qt naming
        if obj is self.table.viewport() and ev.type() == ev.Type.Leave:
            self.hovered.emit(None)
        return False

    def _scatter_hover(self, _item, points, _ev) -> None:
        if len(points):
            r = self.refs[points[0].data()]
            self.plot.setToolTip(f"{rec_text(r)}  t = {r.t:.2f} s  #{r.id}\n{r.type} {r.cls}"
                                 f" level {r.level:+.2f} · {r.mode or ''}\n{flags_text(r)}")
            self.hovered.emit(r.key)
        else:
            self.plot.setToolTip("")
            self.hovered.emit(None)

    def _scatter_click(self, _item, points, ev) -> None:
        if not len(points):
            return
        r = self.refs[points[0].data()]
        if ev.double():
            self.openEvent.emit(r.key)
            return
        mods = ev.modifiers()
        if mods & (Qt.KeyboardModifier.ControlModifier | Qt.KeyboardModifier.ShiftModifier):
            keys = [k for k in self.selected if k != r.key] if r.key in self.selected \
                else self.selected + [r.key]
        else:
            keys = [r.key]
        ev.accept()
        self.set_selected(keys, emit=True)

    def _plot_click(self, vb, ev) -> bool:
        if ev.button() == Qt.MouseButton.RightButton:
            ev.accept()
            self._menu(ev.screenPos().toPoint())
            return True
        return False

    def _lasso_drag(self, vb, ev) -> bool:
        if not self.lasso_btn.isChecked() or ev.button() != Qt.MouseButton.LeftButton:
            return False
        ev.accept()
        p = vb.mapSceneToView(ev.scenePos())
        if ev.isStart():
            p0 = vb.mapSceneToView(ev.buttonDownScenePos())
            self._lasso = [(p0.x(), p0.y())]
        self._lasso.append((p.x(), p.y()))
        xs, ys = zip(*(self._lasso + self._lasso[:1]))
        self.lasso_curve.setData(xs, ys)
        if ev.isFinish():
            self.select_in(self._lasso, add=bool(ev.modifiers() & Qt.KeyboardModifier.ShiftModifier))
            self._lasso = []
            self.lasso_curve.setData([], [])
        return True

    def select_in(self, outline, add: bool = False) -> list:
        """Select the points inside ``outline`` [(x, y)] (data coordinates)."""
        poly = QPolygonF([QPointF(x, y) for x, y in outline])
        keys = [r.key for i, r in enumerate(self.refs) if np.isfinite(self._xy[i]).all()
                and poly.containsPoint(QPointF(*self._xy[i]), Qt.FillRule.OddEvenFill)]
        self.set_selected((self.selected if add else []) + keys, emit=True)
        return keys

    def _menu(self, global_pos) -> None:
        keys = list(self.selected)
        m = QMenu(self)
        n = len(keys)
        for action, text in (("exclude", f"Exclude ({n})"), ("include", f"Include ({n})"),
                             ("pin", f"Pin ({n})"), ("unpin", f"Unpin ({n})"),
                             ("only", f"Show only these ({n})")):
            a = m.addAction(text)
            a.setEnabled(bool(keys))
            a.triggered.connect(lambda _=False, act=action: self.curate.emit(act, keys))
        m.addSeparator()
        m.addAction("Include all").triggered.connect(lambda: self.curate.emit("include_all", []))
        if n == 1:
            m.addAction("Open in the replay").triggered.connect(lambda: self.openEvent.emit(keys[0]))
        m.popup(global_pos)
        self._last_menu = m
