"""Shared base of the per-whisker plot windows (Line, Spectrum).

Each sensor uses two channels, gauge A then gauge B (red / blue, with a legend).
Three pages:

- Grid (the default): one tile per sensor (4x4 for 16 whiskers), X linked,
  Y per tile unless Link Y gives all tiles one Y range.
- Stacked: one axis, the two channels of sensor ``i`` drawn around Y-offset
  ``i``; the Y keys change a trace gain instead of the axis range.
- Single: one sensor filling the window. Double-click a tile (grid) or near a
  whisker's trace (stacked) to enter; double-click again, Esc or "All
  whiskers" goes back.

Keys (window-wide; the Keys button lists the subclass's ``KEYS_HELP``):
= / - zoom X, + / _ zoom Y of the framed tile (the one last under the mouse,
all tiles if none), . / , zoom Y of all tiles, arrows move the data (Up / Down: the
framed tile), Y link Y, Z zoom box, R reset view, Space pause, Esc back. The
punctuation keys are matched by the character typed (an application event
filter, installed while the window is shown), not as QAction shortcuts: Qt
can see Shift+= as both "+" and "Shift++" and then fire neither.

Pause (and the subclass's stream actions) mirror the control panel: the view
only emits ``pauseToggled`` / ``resetRequested`` and the panel pushes its state
back with ``set_paused`` / ``set_stream_controls_enabled``. The status bar's
right side shows the panel's stream status line (``set_stream_status``); its
left side keeps transient messages (probe readout, hints).

Y scale (toolbar "Y auto" / "Y fixed" and its value box): auto is pyqtgraph's
per-plot auto-range (Link Y: one range fitted over all tiles; stacked: a fixed
gain the Y keys change). Fixed gives every tile, the single plot and every
stacked row the same Y range, ``_fixed_range`` from the value box(es), so the
channels can be compared by eye; the Y keys then change that value, and Reset
view puts the fixed range back (it stays fixed). Typing a value switches to
fixed. Subclasses set the boxes' units and ``_fixed_range``.

Every plot has a value label in its corner (on the right of each row in the
stacked layout), refreshed at ~7 Hz; the subclass supplies the text.

Redraw cost: new data and mouse-driven range changes only mark the view dirty;
it is redrawn from a timer at most ~30 times a second, and not at all while
hidden. After a redraw the scene's deferred range / auto-range work is done at
once, so the paint that follows changes nothing and Qt paints once, not twice.
Axes (which also draw the grid lines across the plot) are cached as pixmaps,
so an axis whose ticks did not move is blitted instead of re-rendered.

Subclasses implement ``_has_data``, ``_plot_data``, ``_label_html`` and
``_probe_text``, and may override the other ``_``-hooks marked below.
"""

from __future__ import annotations

import math
import time

import numpy as np
from PyQt6.QtCore import QEvent, QPointF, QRectF, QSize, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QAction, QFont, QFontMetricsF, QKeySequence, QPainter, QPixmapCache
from PyQt6.QtWidgets import (QAbstractSpinBox, QApplication, QComboBox, QDoubleSpinBox,
                             QGraphicsRectItem, QLabel, QLineEdit, QMessageBox, QStackedWidget,
                             QToolBar, QWidget)

import pyqtgraph as pg

from wavi_py.views.base import ViewWindow


RED = (220, 40, 40)
BLUE = (40, 80, 220)
GAUGE_NAMES = ("A", "B")
HTML_COLOR = ("#dc2828", "#2850dc")
_PROBE_PEN = pg.mkPen((90, 90, 90), width=1, style=pg.QtCore.Qt.PenStyle.DashLine)
_PEN_W = 1                      # stacked + grid: wider pens cost 3-6x the paint time
_SINGLE_PEN_W = 2
LABEL_PERIOD_S = 0.15           # value labels refresh at ~7 Hz
_LABEL_FILL = pg.mkBrush(255, 255, 255, 225)
ZOOM_STEP = 1.25                # zoom keys
_PAN_STEP = 0.1                 # arrow keys: fraction of the visible span
_FRAME_S = 1 / 30               # redraw at most ~30 times a second
_FOCUS_PEN = pg.mkPen((255, 150, 0), width=2)
_PIXMAP_CACHE_KB = 128 * 1024

STACKED, GRID, SINGLE = 0, 1, 2                # pages of the QStackedWidget


def _add_gauge_legend(plot: pg.PlotItem) -> None:
    """A / B legend in the top-right corner; curves named afterwards join it."""
    plot.addLegend(offset=(-10, 10), labelTextColor="k",
                   brush=pg.mkBrush(255, 255, 255, 200), pen=pg.mkPen(180, 180, 180))


def _show_grid(plot: pg.PlotItem) -> None:
    """Major grid lines only: the minor ones cost as much to paint as the data."""
    plot.showGrid(x=True, y=True, alpha=0.2)
    for ax in ("left", "bottom"):
        plot.getAxis(ax).setStyle(maxTickLevel=0)
        _cache(plot.getAxis(ax))


def _cache(item: pg.GraphicsObject) -> None:
    """Blit ``item`` from a pixmap unless it changed. Axes with grid lines span the
    whole plot, so every curve update would re-render them otherwise."""
    item.setCacheMode(item.CacheMode.DeviceCoordinateCache)
    # those pixmaps are plot-sized (~20 MB each on a 2x screen); the default 10 MB
    # QPixmapCache would evict them every frame
    if QPixmapCache.cacheLimit() < _PIXMAP_CACHE_KB:
        QPixmapCache.setCacheLimit(_PIXMAP_CACHE_KB)


def _value_label() -> pg.TextItem:
    label = pg.TextItem(anchor=(0, 0), fill=_LABEL_FILL)
    label.setZValue(100)
    # text is only re-laid out at the label rate; blit it from a cache otherwise
    label.textItem.setCacheMode(label.textItem.CacheMode.DeviceCoordinateCache)
    return label


def _px_font(px: int, lo: int, hi: int) -> QFont:
    font = QFont()
    font.setPixelSize(int(min(max(px, lo), hi)))
    return font


def gauge_html(texts: tuple[str, str], unit: str = "") -> str:
    """``A <text>  B <text> [unit]`` in the gauge colours."""
    parts = [f"<span style='color:{HTML_COLOR[g]}'>{GAUGE_NAMES[g]}&nbsp;{texts[g]}</span>"
             for g in range(2)]
    html = "&nbsp;&nbsp;".join(parts)
    if unit:
        html += f"&nbsp;<span style='color:#666'>{unit}</span>"
    return html


class _ElidedLabel(QLabel):
    """One line of text that shrinks to any width, dropping text from the left
    (the error counters at the end of the status line are what matters)."""

    def minimumSizeHint(self) -> QSize:  # noqa: N802 — Qt naming
        return QSize(0, super().minimumSizeHint().height())

    def paintEvent(self, event) -> None:  # noqa: N802 — Qt naming
        r = self.contentsRect()
        text = self.fontMetrics().elidedText(self.text(), Qt.TextElideMode.ElideLeft, r.width())
        QPainter(self).drawText(r, int(self.alignment()), text)


class _ValueBox(QDoubleSpinBox):
    """A spin box that shows 10 as "10", not "10.000"."""

    def textFromValue(self, v: float) -> str:  # noqa: N802 — Qt naming
        loc = self.locale()
        text = loc.toString(float(v), "f", self.decimals()).replace(loc.groupSeparator(), "")
        dp = loc.decimalPoint()
        return text.rstrip("0").rstrip(dp) if dp in text else text


class WhiskerPlotsView(ViewWindow):
    pauseToggled = pyqtSignal(bool)
    resetRequested = pyqtSignal()

    TITLE = "Wavi"
    X_LABEL = ("X", "")              # (label, units) of the X axis
    KEYS_HELP = ""
    VALUE_AXIS_LABEL = ""            # title of the stacked layout's value-label column
    STACKED_CENTRE = 0.0             # fixed Y, stacked: the row offset sits this far up the range
    STACKED_LABEL_SAMPLE = "A -0000.0   B -0000.0"   # widest stacked value label

    def __init__(self, nsensor: int, nch: int):
        super().__init__()
        self.setWindowTitle(self.TITLE)
        self.resize(900, 600)

        self._nsensor = int(nsensor)
        self._nch = int(nch)
        self._gain = 1.0                           # stacked layout: trace height
        self._link_auto = True                     # Link Y: auto-range the shared Y
        self._single: int | None = None            # the maximized sensor, if any
        self._offsets = (np.arange(self._nch) // 2).astype(np.float32)   # stacked Y offsets
        self._labels_t = 0.0
        self._paused = False
        self._focus_tile: int | None = None        # grid: target of Up/Down, framed
        self._last_render = 0.0
        self._render_timer = QTimer(self)
        self._render_timer.setSingleShot(True)
        self._render_timer.timeout.connect(self._render)
        x_label, x_units = self.X_LABEL

        # --- stacked: one axis, sensor i at Y-offset i
        self._stacked = pg.PlotWidget()
        self._stacked.setBackground("w")
        self._stacked.setLabel("bottom", x_label, units=x_units)
        self._stacked.setLabel("left", "Whisker")
        self._stacked.getAxis("left").setTicks([[(s, f"W{s + 1}") for s in range(self._nsensor)]])
        _show_grid(self._stacked.getPlotItem())
        _add_gauge_legend(self._stacked.getPlotItem())
        self._stacked_curves: list[pg.PlotDataItem] = []
        for c in range(self._nch):
            color = RED if (c % 2) == 0 else BLUE
            name = GAUGE_NAMES[c] if c < 2 else None
            self._stacked_curves.append(self._stacked.plot(pen=pg.mkPen(color=color, width=_PEN_W), name=name))
        # value labels: a column right of the axis, one label per whisker row
        self._stacked.showAxis("right")
        self._value_axis = self._stacked.getAxis("right")
        self._value_axis.setStyle(showValues=False, tickLength=0)
        self._value_axis.setGrid(False)        # showGrid gave it a second copy of the left grid
        self._value_axis.setLabel(self.VALUE_AXIS_LABEL)
        _cache(self._value_axis)
        self._stacked_labels = [_value_label() for _ in range(self._nsensor)]
        for label in self._stacked_labels:
            label.setAnchor((0, 0.5))
            label.setParentItem(self._value_axis)
        vb = self._stacked.getPlotItem().getViewBox()
        vb.sigYRangeChanged.connect(self._place_stacked_labels)
        vb.sigResized.connect(self._place_stacked_labels)

        # --- grid: one tile per sensor
        self._grid = pg.GraphicsLayoutWidget()
        self._grid.setBackground("w")
        ncol = math.ceil(math.sqrt(self._nsensor))
        self._tiles: list[pg.PlotItem] = []
        self._grid_curves: list[pg.PlotDataItem] = []
        for s in range(self._nsensor):
            p = self._grid.addPlot(row=s // ncol, col=s % ncol, title=f"W{s + 1}")
            _show_grid(p)
            if self._tiles:
                p.setXLink(self._tiles[0])
            if not self._tiles:
                _add_gauge_legend(p)
            self._tiles.append(p)
            for g in range(2):
                color = RED if g == 0 else BLUE
                name = GAUGE_NAMES[g] if s == 0 else None
                self._grid_curves.append(p.plot(pen=pg.mkPen(color=color, width=_PEN_W), name=name))
        self._tile_labels = [self._corner_label(p) for p in self._tiles]
        self._focus_frames = [self._frame(p) for p in self._tiles]

        # --- single: one sensor filling the window (double-click to enter)
        self._single_plot = pg.PlotWidget()
        self._single_plot.setBackground("w")
        self._single_plot.setLabel("bottom", x_label, units=x_units)
        _show_grid(self._single_plot.getPlotItem())
        _add_gauge_legend(self._single_plot.getPlotItem())
        self._single_curves = [
            self._single_plot.plot(pen=pg.mkPen(color=color, width=_SINGLE_PEN_W), name=name)
            for color, name in ((RED, "A"), (BLUE, "B"))]
        self._single_label = self._corner_label(self._single_plot.getPlotItem())

        self._pages = QStackedWidget()
        self._pages.addWidget(self._stacked)
        self._pages.addWidget(self._grid)
        self._pages.addWidget(self._single_plot)
        self.setCentralWidget(self._pages)

        # --- probe: a vertical line per plot plus a status-bar readout
        self._probe_lines: list[pg.InfiniteLine] = []
        for p in self._all_plots():
            line = pg.InfiniteLine(angle=90, movable=False, pen=_PROBE_PEN)
            line.setVisible(False)
            p.addItem(line, ignoreBounds=True)
            self._probe_lines.append(line)
        for scene in (self._stacked.scene(), self._grid.scene(), self._single_plot.scene()):
            scene.sigMouseMoved.connect(self._on_mouse_moved)
            scene.sigMouseClicked.connect(self._on_mouse_clicked)

        for p in self._all_plots():
            p.getViewBox().sigRangeChangedManually.connect(self._on_manual_range)

        self._build_toolbar()
        self._stream_status = _ElidedLabel()
        self._stream_status.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.statusBar().addPermanentWidget(self._stream_status)
        self.statusBar().messageChanged.connect(self._fit_stream_status)
        # subclasses call self._act_grid.setChecked(True) at the end of their __init__
        # (grid by default; this also resets the view)

    # ------------------------------------------------------------------
    # toolbar
    # ------------------------------------------------------------------

    def _build_toolbar(self) -> None:
        tb = QToolBar("View")
        tb.setMovable(False)
        self.addToolBar(tb)

        self._act_grid = QAction("Grid", self, checkable=True)
        self._act_grid.setToolTip("One tile per sensor instead of one stacked axis")
        self._act_grid.toggled.connect(self._on_grid_toggled)

        self._act_zoom = QAction("Zoom box", self, checkable=True)
        self._act_zoom.setToolTip("Drag a box to zoom (off: drag pans). Wheel zooms either way. [Z]")
        self._act_zoom.setShortcut(QKeySequence("Z"))
        self._act_zoom.toggled.connect(self._on_zoom_toggled)

        self._act_reset = QAction("Reset view", self)
        self._act_reset.setShortcut(QKeySequence("R"))
        self._act_reset.triggered.connect(self._reset_view)

        self._act_link_y = QAction("Link Y", self, checkable=True)
        self._act_link_y.setToolTip("Grid: all tiles share one Y range [Y]")
        self._act_link_y.setShortcut(QKeySequence("Y"))
        self._act_link_y.toggled.connect(self._on_link_y)

        act_keys = QAction("Keys", self)
        act_keys.setToolTip("Keyboard and mouse shortcuts")
        act_keys.triggered.connect(self._show_keys)

        self._act_probe = QAction("Probe", self, checkable=True)
        self._act_probe.setToolTip("Show channel values under the cursor")
        self._act_probe.toggled.connect(self._on_probe_toggled)

        self._act_all = QAction("All whiskers", self)
        self._act_all.setToolTip("Back to all whiskers (or double-click the plot) [Esc]")
        self._act_all.setShortcut(QKeySequence("Esc"))
        self._act_all.setEnabled(False)
        self._act_all.triggered.connect(self._restore)

        # Pause (and the subclass's stream actions): the control panel owns the logic.
        self._act_pause = QAction("Pause", self, checkable=True)
        self._act_pause.setToolTip("Freeze the plots; data keeps flowing and recording continues [Space]")
        self._act_pause.setShortcut(QKeySequence(Qt.Key.Key_Space))
        self._act_pause.toggled.connect(self._on_pause_action)

        for a in (self._act_grid, self._act_link_y, self._act_zoom, self._act_reset,
                  *self._view_actions()):
            tb.addAction(a)
        self._build_y_scale(tb)
        for a in (self._act_probe, self._act_all):
            tb.addAction(a)
        tb.addSeparator()
        for a in (self._act_pause, *self._stream_actions()):
            tb.addAction(a)
        tb.addSeparator()
        tb.addAction(act_keys)
        # Shortcuts are window-wide (QAction's default WindowShortcut context), so they
        # work whichever plot has focus. A shortcut beats a focused button's own Space
        # handling; the toolbar buttons also take no focus, so Space never clicks one.
        # the arrows move the data, not the view: Left slides the traces left (later
        # times come into view), Up slides them up
        for key, dx, dy in ((Qt.Key.Key_Left, _PAN_STEP, 0), (Qt.Key.Key_Right, -_PAN_STEP, 0),
                            (Qt.Key.Key_Up, 0, -_PAN_STEP), (Qt.Key.Key_Down, 0, _PAN_STEP)):
            act = QAction(self)
            act.setShortcut(QKeySequence(key))
            act.triggered.connect(lambda _=False, dx=dx, dy=dy: self._pan(dx, dy))
            self.addAction(act)                    # window-wide, not on the toolbar
        for a in tb.actions():
            w = tb.widgetForAction(a)
            if w is not None and not isinstance(w, QDoubleSpinBox):
                w.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.set_stream_controls_enabled(False)

    def _build_y_scale(self, tb: QToolBar) -> None:
        self._y_mode = QComboBox()
        self._y_mode.addItems(["Y auto", "Y fixed"])
        self._y_mode.setToolTip("Y auto: each plot fits its data. Y fixed: the same Y range for "
                                "every plot, so the channels compare by eye")
        self._y_mode.currentIndexChanged.connect(self._on_y_mode)
        self._y_top = self._y_box()
        self._y_span = self._y_box()
        tb.addWidget(self._y_mode)
        tb.addWidget(self._y_top)
        self._act_y_span = tb.addWidget(self._y_span)
        self._act_y_span.setVisible(False)              # subclasses with a second value show it

    def _y_box(self) -> QDoubleSpinBox:
        """A value box of the fixed Y range: takes typing only while clicked into."""
        b = _ValueBox()
        b.setStepType(QAbstractSpinBox.StepType.AdaptiveDecimalStepType)
        b.setKeyboardTracking(False)
        b.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
        b.editingFinished.connect(lambda: self._pages.currentWidget().setFocus())
        b.valueChanged.connect(self._on_y_value)
        return b

    @staticmethod
    def _set_box(box: QDoubleSpinBox, value: float, lo: float, hi: float, decimals: int,
                 prefix: str = "", suffix: str = "") -> None:
        """Configure a value box without emitting ``valueChanged``."""
        box.blockSignals(True)
        box.setDecimals(decimals)
        box.setRange(lo, hi)
        box.setPrefix(prefix)
        box.setSuffix(suffix)
        box.setValue(value)
        box.blockSignals(False)

    def _view_actions(self) -> list[QAction]:
        """Hook: extra toolbar actions after Reset view."""
        return []

    def _stream_actions(self) -> list[QAction]:
        """Hook: extra toolbar actions after Pause, enabled only while connected."""
        return []

    def _all_plots(self) -> list[pg.PlotItem]:
        return [self._stacked.getPlotItem(), *self._tiles, self._single_plot.getPlotItem()]

    # ------------------------------------------------------------------
    # pause and stream status (state pushed from the control panel)
    # ------------------------------------------------------------------

    def _on_pause_action(self, on: bool) -> None:
        self._act_pause.setText("Resume" if on else "Pause")
        self.pauseToggled.emit(on)

    def set_paused(self, on: bool) -> None:
        """Show the pause state without emitting ``pauseToggled``."""
        self._act_pause.blockSignals(True)
        self._act_pause.setChecked(on)
        self._act_pause.blockSignals(False)
        self._act_pause.setText("Resume" if on else "Pause")
        was_paused, self._paused = self._paused, on
        if was_paused and not on:
            self._on_resumed()

    def _on_resumed(self) -> None:
        """Hook: Pause was released."""

    def set_stream_status(self, text: str) -> None:
        """The control panel's stream status line, shown on the right of the status bar."""
        self._stream_status.setText(text)
        self._stream_status.setToolTip(text)

    def _fit_stream_status(self, *_args) -> None:
        """A transient message on the left gets the room it needs; the status line
        on the right takes the rest."""
        msg = self.statusBar().currentMessage()
        if msg:
            room = self.statusBar().width() - self.fontMetrics().horizontalAdvance(msg) - 40
            self._stream_status.setMaximumWidth(max(0, room))
        else:
            self._stream_status.setMaximumWidth(16777215)          # QWIDGETSIZE_MAX

    def resizeEvent(self, event) -> None:  # noqa: N802 — Qt naming
        super().resizeEvent(event)
        self._fit_stream_status()

    def set_stream_controls_enabled(self, on: bool) -> None:
        """Pause and the stream actions only make sense while connected."""
        for a in (self._act_pause, *self._stream_actions()):
            a.setEnabled(on)

    def _idle_hint(self) -> str:
        """Hook: status-bar message to show when nothing else is (e.g. "not live")."""
        return ""

    # ------------------------------------------------------------------
    # layouts
    # ------------------------------------------------------------------

    def _on_grid_toggled(self, on: bool) -> None:
        self._leave_single()
        self._set_focus_tile(None)
        self._pages.setCurrentIndex(GRID if on else STACKED)
        self._reset_view()

    def _on_mouse_clicked(self, ev) -> None:
        if not ev.double() or ev.button() != pg.QtCore.Qt.MouseButton.LeftButton:
            return
        page = self._pages.currentIndex()
        pos = ev.scenePos()
        if page == SINGLE:
            self._restore()
        elif page == GRID:
            sensor = next((i for i, p in enumerate(self._tiles)
                           if p.sceneBoundingRect().contains(pos)), None)
            if sensor is not None:
                self._maximize(sensor)
        else:
            vb = self._stacked.getPlotItem().getViewBox()
            if vb.sceneBoundingRect().contains(pos):
                sensor = int(round(vb.mapSceneToView(pos).y()))
                self._maximize(int(np.clip(sensor, 0, self._nsensor - 1)))

    def _maximize(self, sensor: int) -> None:
        x = self._visible_x_range()
        self._single = sensor
        self._single_plot.setTitle(f"W{sensor + 1}")
        self.setWindowTitle(f"{self.TITLE} - W{sensor + 1}")
        self._act_all.setEnabled(True)
        self._pages.setCurrentIndex(SINGLE)
        self._init_y(self._single_plot.getPlotItem())
        self._show_x(x)
        self._render(labels=True)

    def _restore(self) -> None:
        if self._single is None:
            return
        x = self._visible_x_range()
        self._leave_single()
        self._pages.setCurrentIndex(GRID if self._act_grid.isChecked() else STACKED)
        self._show_x(x)
        self._render(labels=True)

    def _leave_single(self) -> None:
        self._single = None
        self._act_all.setEnabled(False)
        self.setWindowTitle(self.TITLE)

    def _visible_x_range(self) -> tuple[float, float]:
        x0, x1 = self._current_x_plot().getViewBox().viewRange()[0]
        return x0, x1

    def _show_x(self, x: tuple[float, float]) -> None:
        """Hook: carry the X view across a page switch."""
        self._current_x_plot().setXRange(*x, padding=0)

    def _current_x_plot(self) -> pg.PlotItem:
        page = self._pages.currentIndex()
        if page == STACKED:
            return self._stacked.getPlotItem()
        if page == GRID:
            return self._tiles[0]                         # the rest are X-linked
        return self._single_plot.getPlotItem()

    # ------------------------------------------------------------------
    # keyboard zoom / pan, link Y
    # ------------------------------------------------------------------

    def eventFilter(self, obj, ev) -> bool:  # noqa: N802 — Qt naming
        """Zoom keys by the character typed, anywhere in this window."""
        if ev.type() != QEvent.Type.KeyPress or not isinstance(obj, QWidget) or obj.window() is not self:
            return False
        if isinstance(obj, (QAbstractSpinBox, QLineEdit)):
            return False                                   # typing into a value box
        text = ev.text()
        mods = ev.modifiers() & ~(Qt.KeyboardModifier.ShiftModifier | Qt.KeyboardModifier.KeypadModifier)
        if mods or not text:
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

    def _zoom_x(self, f: float) -> None:
        """X zoom about the view centre, all plots (they are X-linked)."""
        self._current_x_plot().getViewBox().scaleBy(x=f)
        self._on_manual_range()

    def _y_targets(self, all_tiles: bool) -> list[pg.PlotItem]:
        """Grid plots a Y key acts on: the framed tile, or all (one if Y is linked)."""
        if self._act_link_y.isChecked():
            self._link_auto = False
            return [self._tiles[0]]
        if all_tiles or self._focus_tile is None:
            return self._tiles
        return [self._tiles[self._focus_tile]]

    def _zoom_y(self, f: float, all_tiles: bool) -> None:
        """Y zoom about the view centre; in the stacked layout, the trace gain."""
        if self.fixed_y:
            self._zoom_fixed(f)
            return
        page = self._pages.currentIndex()
        if page == STACKED:
            self._gain /= f
            self._show_gain()
            self._render()
            return
        plots = self._y_targets(all_tiles) if page == GRID else [self._single_plot.getPlotItem()]
        for p in plots:
            p.getViewBox().scaleBy(y=f)

    def _show_gain(self) -> None:
        if self.fixed_y:
            self._stacked.setLabel("left", f"Whisker (row: {self._fixed_text()})")
            return
        self._stacked.setLabel("left", "Whisker" if abs(self._gain - 1) < 1e-9
                               else f"Whisker (gain x{self._gain:.3g})")

    # ------------------------------------------------------------------
    # Y scale: auto / fixed
    # ------------------------------------------------------------------

    @property
    def fixed_y(self) -> bool:
        return self._y_mode.currentIndex() == 1

    def set_fixed_y(self, on: bool) -> None:
        self._y_mode.setCurrentIndex(1 if on else 0)

    def _fixed_range(self) -> tuple[float, float]:
        """Hook: the fixed Y range, in the plotted units. Default: +- the value box."""
        v = self._y_top.value()
        return -v, v

    def _fixed_text(self) -> str:
        lo, hi = self._fixed_range()
        return f"{lo:g} to {hi:g}"

    def _zoom_fixed(self, f: float) -> None:
        """Hook: a Y zoom key while fixed scales the value (all plots alike)."""
        self._y_top.setValue(self._y_top.value() * f)

    def _on_y_mode(self, _i: int) -> None:
        self._reset_y()
        self._show_gain()
        self._render(labels=True)

    def _on_y_value(self, _v: float) -> None:
        if self.fixed_y:
            self._on_y_mode(1)
        else:
            self.set_fixed_y(True)                         # typing a value means: use it

    def _init_y(self, p: pg.PlotItem) -> None:
        """Auto-range ``p``'s Y, or give it the fixed range."""
        if self.fixed_y:
            p.disableAutoRange(axis="y")
            p.setYRange(*self._fixed_range(), padding=0)
        else:
            p.enableAutoRange(axis="y")

    def _reset_y(self) -> None:
        """Y of the tiles and the single plot: auto (Link Y: fitted in ``_redraw``) or fixed."""
        self._link_auto = True
        linked = self._act_link_y.isChecked()
        for p in (*self._tiles, self._single_plot.getPlotItem()):
            if linked and not self.fixed_y and p in self._tiles:
                p.disableAutoRange(axis="y")
            else:
                self._init_y(p)

    def _pan(self, dx: float, dy: float) -> None:
        """Shift the view by ``dx`` / ``dy`` visible spans. X moves all plots
        (``_on_manual_range``); Y moves the framed tile (all if none)."""
        def shift(p: pg.PlotItem, x: bool, y: bool) -> None:
            vb = p.getViewBox()
            (x0, x1), (y0, y1) = vb.viewRange()
            vb.translateBy(x=dx * (x1 - x0) if x else None, y=dy * (y1 - y0) if y else None)
        page = self._pages.currentIndex()
        if dx:
            shift(self._current_x_plot(), True, False)
        if dy:
            plots = self._y_targets(False) if page == GRID else [self._current_x_plot()]
            for p in plots:
                shift(p, False, True)
        if dx:
            self._on_manual_range()

    def _on_link_y(self, on: bool) -> None:
        for p in self._tiles[1:]:
            p.setYLink(self._tiles[0] if on else None)
        self._reset_y()
        self._render()

    def _show_keys(self) -> None:
        box = QMessageBox(QMessageBox.Icon.NoIcon, f"{self.TITLE} keys", self.KEYS_HELP, parent=self)
        box.setTextFormat(Qt.TextFormat.RichText)
        box.setModal(False)
        box.show()

    def _set_focus_tile(self, sensor: int | None) -> None:
        if sensor == self._focus_tile:
            return
        if self._focus_tile is not None:
            self._focus_frames[self._focus_tile].hide()
        if sensor is not None:
            self._focus_frames[sensor].show()
        self._focus_tile = sensor

    @staticmethod
    def _frame(plot: pg.PlotItem) -> QGraphicsRectItem:
        """A hidden frame around the whole tile (title and axes included)."""
        frame = QGraphicsRectItem(plot)
        frame.setPen(_FOCUS_PEN)
        frame.setZValue(1000)
        frame.hide()

        def fit() -> None:
            frame.setRect(QRectF(plot.boundingRect()).adjusted(1, 1, -1, -1))
        plot.geometryChanged.connect(fit)
        fit()
        return frame

    def _on_zoom_toggled(self, on: bool) -> None:
        mode = pg.ViewBox.RectMode if on else pg.ViewBox.PanMode
        for p in self._all_plots():
            p.getViewBox().setMouseMode(mode)
        msg = "Zoom box: drag a rectangle to zoom" if on else "Pan: drag to move the view"
        hint = self._idle_hint()
        self.statusBar().showMessage(f"{msg}. {hint}" if hint else msg)

    def _on_manual_range(self, *_args) -> None:
        """A mouse zoom / pan, or a key that moves X. Subclasses extend it."""
        self._link_auto = False
        # re-draw for the new range (matters while paused); after the range lands
        self._request_render()

    def _reset_view(self) -> None:
        self._gain = 1.0
        self._link_auto = True
        self._show_gain()
        self.statusBar().clearMessage()
        self._stacked.setYRange(-0.5, self._nsensor)
        self._reset_y()
        self._reset_x()
        self._render(labels=True)

    def _reset_x(self) -> None:
        """Hook: the X part of Reset view."""

    # ------------------------------------------------------------------
    # probe
    # ------------------------------------------------------------------

    def _on_probe_toggled(self, on: bool) -> None:
        for line in self._probe_lines:
            line.setVisible(on)
        if not on:
            self.statusBar().showMessage(self._idle_hint())

    def _on_mouse_moved(self, pos) -> None:
        if self._pages.currentIndex() == GRID:
            hit = next((i for i, p in enumerate(self._tiles)
                        if p.sceneBoundingRect().contains(pos)), None)
            if hit is not None:
                self._set_focus_tile(hit)
        if not self._act_probe.isChecked() or not self._has_data():
            return
        page = self._pages.currentIndex()
        if page in (STACKED, SINGLE):
            vb = self._current_x_plot().getViewBox()
            if not vb.sceneBoundingRect().contains(pos):
                return
            pt = vb.mapSceneToView(pos)
            sensor = int(round(pt.y())) if page == STACKED else self._single
        else:
            sensor = next((i for i, p in enumerate(self._tiles)
                           if p.getViewBox().sceneBoundingRect().contains(pos)), None)
            if sensor is None:
                return
            pt = self._tiles[sensor].getViewBox().mapSceneToView(pos)
        for line in self._probe_lines:
            line.setPos(pt.x())
        self.statusBar().showMessage(
            self._probe_text(sensor if 0 <= sensor < self._nsensor else None, pt.x()))

    # ------------------------------------------------------------------
    # rendering: throttled, visible page only
    # ------------------------------------------------------------------

    def _request_render(self) -> None:
        """Redraw soon, at most every ``_FRAME_S``; repeated requests coalesce."""
        if not self._render_timer.isActive():
            wait = self._last_render + _FRAME_S - time.monotonic()
            self._render_timer.start(max(0, int(wait * 1000)))

    def _render(self, labels: bool = False) -> None:
        self._render_timer.stop()
        self._last_render = time.monotonic()
        if not self.isVisible():
            return                                         # showEvent renders
        self._before_redraw()
        self._redraw()
        if labels or self._last_render - self._labels_t >= LABEL_PERIOD_S:
            self._update_labels()
        # do the deferred range / auto-range / transform updates now, not in paintEvent:
        # changes made during a paint schedule a second paint
        self._pages.currentWidget().scene().prepareForPaint()

    def _before_redraw(self) -> None:
        """Hook: e.g. pin X to the live window."""

    def showEvent(self, event) -> None:  # noqa: N802 — Qt naming
        super().showEvent(event)
        # the zoom-key filter (eventFilter) is app-wide, so only while shown
        QApplication.instance().installEventFilter(self)
        self._request_render()

    def hideEvent(self, event) -> None:  # noqa: N802 — Qt naming
        QApplication.instance().removeEventFilter(self)
        super().hideEvent(event)

    def _stacked_y(self, y: np.ndarray) -> np.ndarray:
        """Hook: per-channel values -> height in the stacked layout (before the row offset).
        Fixed Y: the fixed range spans one row."""
        if self.fixed_y:
            lo, hi = self._fixed_range()
            return (y - lo) / (hi - lo) - self.STACKED_CENTRE
        return y * self._gain

    def _redraw(self) -> None:
        # Only the visible layout is drawn; switching layouts redraws the other.
        if not self._has_data():
            return
        page = self._pages.currentIndex()
        if page == STACKED:
            x, y = self._plot_data(self._stacked.getPlotItem(), slice(None))
            y = (self._stacked_y(y) + self._offsets).T.copy()      # contiguous rows, one per curve
            finite = bool(np.isfinite(y).all())
            for c, curve in enumerate(self._stacked_curves):
                curve.setData(x, y[c], skipFiniteCheck=finite)
        elif page == GRID:
            x, y = self._plot_data(self._tiles[0], slice(None))
            y = y.T.copy()
            finite = bool(np.isfinite(y).all())
            for c, curve in enumerate(self._grid_curves):
                curve.setData(x, y[c], skipFiniteCheck=finite)
            if self._act_link_y.isChecked() and self._link_auto and not self.fixed_y:
                self._fit_linked_y(y)
        elif self._single is not None:
            c0 = 2 * self._single
            x, y = self._plot_data(self._single_plot.getPlotItem(), slice(c0, min(c0 + 2, self._nch)))
            for g in range(y.shape[1]):
                self._single_curves[g].setData(x, y[:, g])

    def _fit_linked_y(self, y: np.ndarray) -> None:
        """Link Y, auto: one Y range over all tiles' data (pyqtgraph's per-tile
        auto-range would fight over the linked range)."""
        if not np.isfinite(y).any():
            return
        lo, hi = float(np.nanmin(y)), float(np.nanmax(y))
        pad = 0.05 * (hi - lo) if hi > lo else 0.5
        lo, hi = lo - pad, hi + pad
        y0, y1 = self._tiles[0].getViewBox().viewRange()[1]
        if abs(y0 - lo) + abs(y1 - hi) > 1e-3 * (hi - lo):
            self._tiles[0].setYRange(lo, hi, padding=0)

    # ------------------------------------------------------------------
    # data hooks
    # ------------------------------------------------------------------

    def _has_data(self) -> bool:
        raise NotImplementedError

    def _plot_data(self, plot: pg.PlotItem, cols: slice) -> tuple[np.ndarray, np.ndarray]:
        """``(x, y)`` for channels ``cols``: x (N,), y (N, ncols), as drawn in ``plot``."""
        raise NotImplementedError

    def _label_html(self, sensor: int, compact: bool) -> str:
        """Corner label of ``sensor`` (``compact``: the stacked layout's row label)."""
        raise NotImplementedError

    def _prepare_labels(self) -> None:
        """Hook: compute what ``_label_html`` shows, once per label refresh."""

    def _probe_text(self, sensor: int | None, x: float) -> str:
        raise NotImplementedError

    # ------------------------------------------------------------------
    # value labels
    # ------------------------------------------------------------------

    def _corner_label(self, plot: pg.PlotItem) -> pg.TextItem:
        """Top-left of ``plot``'s view box, font sized to the plot."""
        label = _value_label()
        vb = plot.getViewBox()
        label.setParentItem(vb)
        label.setPos(2, 2)

        def resize() -> None:
            label.setFont(_px_font(round(0.07 * vb.height()), 8, 16))
        vb.sigResized.connect(resize)
        resize()
        return label

    def _place_stacked_labels(self) -> None:
        vb = self._stacked.getPlotItem().getViewBox()
        y0, y1 = vb.viewRange()[1]
        row_px = vb.height() / max(y1 - y0, 1e-9)
        font = _px_font(round(0.45 * row_px), 7, 14)
        width = math.ceil(QFontMetricsF(font).horizontalAdvance(self.STACKED_LABEL_SAMPLE)) + 12
        if self._value_axis.width() != width:
            self._value_axis.setWidth(width)
        for s, label in enumerate(self._stacked_labels):
            y = self._value_axis.mapFromScene(vb.mapViewToScene(QPointF(0, s))).y()
            label.setVisible(0 <= y <= self._value_axis.height())
            label.setFont(font)
            label.setPos(4, y)

    def _update_labels(self) -> None:
        """Refresh the visible page's value labels."""
        self._labels_t = time.monotonic()
        self._prepare_labels()
        page = self._pages.currentIndex()
        if page == STACKED:
            for s, label in enumerate(self._stacked_labels):
                label.setHtml(self._label_html(s, compact=True))
        elif page == GRID:
            for s, label in enumerate(self._tile_labels):
                label.setHtml(self._label_html(s, compact=False))
        elif self._single is not None:
            self._single_label.setHtml(self._label_html(self._single, compact=False))
