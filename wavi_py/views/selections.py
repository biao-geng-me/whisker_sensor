"""Selections: the app-wide model, their regions in the plots, and Select mode.

``SelectionModel`` holds the selections (``daq/analysis.Selection``) for every
window: Line, IMU, every timeline strip and every Analysis window show the same
set, numbered and coloured by number. The control panel gives it the source a
new selection reads (``source_fn``: the whole-recording store in replay; live,
the copy of the buffers taken at Pause, ``freeze``; else a reason why not) and
the display context (Despike, V0, scale) the Analysis windows apply.

``SelectionOverlay`` draws each selection as a shaded region in a set of plots
(a ``LinearRegionItem`` per selection per plot, under the curves). An edge
drag changes that edge; a drag on the body (between the edges) moves the whole
selection, its length kept and clamped to the source's extent (``move``), so a
drag that starts on a body does not pan the plot (one elsewhere still does).
Overlapping regions: the newest is on top (Z by number) and takes the drag.
Cursors: open hand over a body, closed hand while it is dragged, the resize
cursor over an edge. In a plot with Select on, the body takes no drag (the drag
makes a new selection, which may overlap). Either drag sets the model, which
moves that region in every other plot and redraws the Analysis windows.
Right-click a region for its menu (Compare / Spectrum / Spectrogram / New
analysis window / Delete: ``model.requested``); the hovered one is
``model.hovered``, which Del deletes.

``attach_select`` gives a plot window the Select toggle (S) and Del: with Select
on, a left drag in a plot makes a new selection (a per-instance wrapper of the
ViewBox's ``mouseDragEvent``, so the view classes stay as they are).
"""

from __future__ import annotations

from typing import Callable, Optional, Sequence, Union

from PyQt6.QtCore import QObject, Qt, pyqtSignal
from PyQt6.QtGui import QAction, QColor, QKeySequence
from PyQt6.QtWidgets import QMainWindow, QMenu, QToolBar

import pyqtgraph as pg

from wavi_py.daq.analysis import Selection, SelSource

# selection colours by number (1 -> the first); distinct from the red playhead,
# the blue loop range and the green armed spans
SEL_COLORS = ((230, 140, 0), (140, 60, 200), (0, 150, 150), (200, 60, 120),
              (110, 130, 30), (60, 100, 200), (160, 90, 40), (90, 90, 90))
_MIN_PX = 3                       # a Select drag shorter than this makes nothing
_EDGE_PX = 3                      # a body's hover this near an edge leaves the drag to the edge
COMPARE, SPECTRUM, SPECTROGRAM, NEW_WINDOW = "compare", "spectrum", "spectrogram", "new"


def sel_color(number: int, alpha: int = 255) -> QColor:
    r, g, b = SEL_COLORS[(number - 1) % len(SEL_COLORS)]
    return QColor(r, g, b, alpha)


class SelectionModel(QObject):
    """The app-wide selections (see the module doc). Signals carry the Selection."""

    added = pyqtSignal(object)
    changed = pyqtSignal(object)            # t0 / t1 moved
    removed = pyqtSignal(object)
    hoverChanged = pyqtSignal(object)       # Selection or None
    dataChanged = pyqtSignal()              # Despike, V0 / scale or the variables changed
    # a region's menu: (action, selection, variable keys of the plot it was used in)
    requested = pyqtSignal(str, object, list)

    def __init__(self, parent: Optional[QObject] = None):
        super().__init__(parent)
        self._sels: list[Selection] = []
        self._next = 1
        self.hovered: Optional[Selection] = None
        # the source of a new selection: a SelSource, or a str saying why there is none
        self.source_fn: Callable[[], Union[SelSource, str]] = lambda: "no data"
        # the dome map of a new selection's recording (its Positions read through it);
        # None (live): the Analysis window's active map
        self.dome_map_fn: Callable[[], object] = lambda: None
        self._frozen: Optional[SelSource] = None
        self.despike = False
        self.v0 = None
        self.scale = 1.0

    # ----- the set

    @property
    def selections(self) -> list[Selection]:
        return list(self._sels)

    def __len__(self) -> int:
        return len(self._sels)

    def by_number(self, number: int) -> Optional[Selection]:
        return next((s for s in self._sels if s.number == number), None)

    def create(self, t0: float, t1: float) -> Union[Selection, str]:
        """A new selection over [t0, t1] on the current source (the frozen live copy
        while paused); a str (the reason) if there is none to select from."""
        src = self._frozen if self._frozen is not None else self.source_fn()
        if isinstance(src, str):
            return src
        bounds = src.extent()
        if bounds is None:
            return "no data to select"
        sel = Selection(self._next, 0.0, 0.0, src, bounds,
                        dome_map=None if self._frozen is not None else self.dome_map_fn())
        sel.t0, sel.t1 = sel.clamp(t0, t1)
        self._next += 1
        self._sels.append(sel)
        self.added.emit(sel)
        return sel

    def set_range(self, sel: Selection, t0: float, t1: float) -> None:
        t0, t1 = sel.clamp(t0, t1)
        if (t0, t1) != (sel.t0, sel.t1) and sel in self._sels:
            sel.t0, sel.t1 = t0, t1
            self.changed.emit(sel)

    def move(self, sel: Selection, t0: float) -> None:
        """Start ``sel`` at ``t0``, its length kept, inside its bounds (a body drag)."""
        d = sel.duration
        if sel.bounds is not None:
            lo, hi = sel.bounds
            t0 = max(lo, min(t0, hi - d))
        self.set_range(sel, t0, t0 + d)

    def remove(self, sel: Optional[Selection]) -> None:
        if sel is None or sel not in self._sels:
            return
        self._sels.remove(sel)
        if self.hovered is sel:
            self.set_hovered(None)
        self.removed.emit(sel)

    def clear(self) -> None:
        """All gone (a new recording / stream: their times meant another one)."""
        for sel in list(self._sels):
            self.remove(sel)
        self._next = 1
        self._frozen = None

    def set_hovered(self, sel: Optional[Selection]) -> None:
        if sel is not self.hovered:
            self.hovered = sel
            self.hoverChanged.emit(sel)

    # ----- the source and the display context

    def freeze(self, src: Optional[SelSource]) -> None:
        """Live paused: new selections read ``src`` (the buffers copied at Pause);
        None (resumed): back to ``source_fn``. Selections made meanwhile keep it."""
        self._frozen = src

    @property
    def frozen(self) -> bool:
        return self._frozen is not None

    def set_context(self, despike: Optional[bool] = None, v0=None,
                    scale: Optional[float] = None) -> None:
        if despike is not None:
            self.despike = bool(despike)
        if v0 is not None:
            self.v0 = v0
        if scale is not None:
            self.scale = float(scale)
        self.dataChanged.emit()

    def request(self, action: str, sel: Selection, var_keys: Sequence[str] = ()) -> None:
        self.requested.emit(action, sel, list(var_keys))


class _Region(pg.LinearRegionItem):
    """One selection in one plot: the edges and the body drag (the body moves the
    whole selection, ``SelectionModel.move``); the body takes hover and right-click."""

    def __init__(self, overlay: "SelectionOverlay", sel: Selection, plot: pg.PlotItem):
        c = sel_color(sel.number)
        super().__init__((sel.t0, sel.t1), movable=True, brush=pg.mkBrush(sel_color(sel.number, 45)),
                         hoverBrush=pg.mkBrush(sel_color(sel.number, 80)),
                         pen=pg.mkPen(c, width=1), hoverPen=pg.mkPen(c, width=3))
        self.movable = False                 # not the base class's body drag (see mouseDragEvent)
        self.setAcceptHoverEvents(True)
        # under the curves; the newest on top, so it takes hover and drags where they overlap
        self.setZValue(-10 + 1e-4 * min(sel.number, 9999))
        self._overlay, self.sel, self.plot = overlay, sel, plot
        self._drag_from: Optional[tuple[float, float]] = None   # body drag: (t0, x) at its start
        for line in self.lines:
            line.setCursor(Qt.CursorShape.SizeHorCursor)
        self._set_cursor()
        if sel.bounds is not None:
            self.setBounds(sel.bounds)
        # at the bottom: the top corners hold the Line tiles' value labels and legends
        pg.InfLineLabel(self.lines[0], text=str(sel.number), position=0.04, anchors=[(0, 1), (0, 1)],
                        color=c)
        self.sigRegionChanged.connect(self._dragged)

    def _dragged(self) -> None:
        t0, t1 = self.getRegion()
        self._overlay.model.set_range(self.sel, t0, t1)

    def body_drags(self) -> bool:
        """The body moves on a drag, unless its plot is in Select mode (``attach_select``)."""
        act = getattr(self.plot.getViewBox(), "select_action", None)
        return not (act is not None and act.isChecked())

    def _set_cursor(self) -> None:
        if self._drag_from is not None:
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
        elif self.body_drags():
            self.setCursor(Qt.CursorShape.OpenHandCursor)
        else:
            self.unsetCursor()

    def hoverEvent(self, ev) -> None:  # noqa: N802 — pyqtgraph naming
        self._overlay.model.set_hovered(None if ev.isExit() else self.sel)
        if not ev.isExit() and self.body_drags() and not self._near_edge(ev.pos().x()):
            ev.acceptDrags(Qt.MouseButton.LeftButton)    # (an edge claims its own drags)
        self._set_cursor()

    def _near_edge(self, x: float) -> bool:
        """Within _EDGE_PX of an edge: the hover can reach the region before its edge
        line (a point near, not on, the line), so it leaves the drag to the line."""
        px = self.plot.getViewBox().viewPixelSize()[0]
        return min(abs(x - v) for v in self.getRegion()) <= _EDGE_PX * px

    def mouseDragEvent(self, ev) -> None:  # noqa: N802 — pyqtgraph naming
        """A drag on the body: the selection moves by the mouse's travel in time."""
        if ev.button() != Qt.MouseButton.LeftButton or (
                self._drag_from is None and not (ev.isStart() and self.body_drags())):
            return                                       # not taken: the ViewBox pans / selects
        ev.accept()
        if ev.isStart():
            self._drag_from = (self.sel.t0, float(ev.buttonDownPos().x()))
            self._set_cursor()
        t0, x0 = self._drag_from
        self._overlay.model.move(self.sel, t0 + float(ev.pos().x()) - x0)
        if ev.isFinish():
            self._drag_from = None
            self._set_cursor()

    def mouseClickEvent(self, ev) -> None:  # noqa: N802 — pyqtgraph naming
        if ev.button() == Qt.MouseButton.RightButton:
            ev.accept()
            self._overlay.menu(self, ev)


class SelectionOverlay(QObject):
    """The model's selections drawn in ``plots`` (see the module doc).

    ``vars_at(plot, scene_pos)``: the variable keys a region's menu opened in that
    plot pre-checks (the sensor tile, the IMU row); ``menu_extra(menu)`` may add to
    the menu."""

    def __init__(self, model: SelectionModel, plots: Sequence[pg.PlotItem],
                 vars_at: Optional[Callable] = None, parent: Optional[QObject] = None):
        super().__init__(parent)
        self.model = model
        self._plots = list(plots)
        self._vars_at = vars_at
        self._items: dict[int, list[_Region]] = {}       # id(selection) -> its regions
        model.added.connect(self._add)
        model.changed.connect(self._move)
        model.removed.connect(self._remove)
        for sel in model.selections:
            self._add(sel)

    def regions(self, sel: Selection) -> list[_Region]:
        return list(self._items.get(id(sel), []))

    def _add(self, sel: Selection) -> None:
        items = []
        for p in self._plots:
            r = _Region(self, sel, p)
            p.addItem(r, ignoreBounds=True)
            items.append(r)
        self._items[id(sel)] = items

    def _move(self, sel: Selection) -> None:
        for r in self._items.get(id(sel), []):
            if tuple(r.getRegion()) != (sel.t0, sel.t1):
                r.blockSignals(True)
                r.setRegion((sel.t0, sel.t1))
                r.blockSignals(False)

    def _remove(self, sel: Selection) -> None:
        for r in self._items.pop(id(sel), []):
            r.plot.removeItem(r)

    def menu(self, region: _Region, ev) -> None:
        sel = region.sel
        keys = list(self._vars_at(region.plot, ev.scenePos())) if self._vars_at else []
        m = QMenu()
        m.setToolTipsVisible(True)
        title = m.addAction(f"Selection {sel.number}: {sel.t0:.2f} .. {sel.t1:.2f} s "
                            f"({sel.duration:.2f} s)")
        title.setEnabled(False)
        m.addSeparator()
        for action, text, tip, enabled in (
                (COMPARE, "Compare", "Show it in the Analysis window (the newest focused)", True),
                (SPECTRUM, "Spectrum", "Its amplitude spectrum in the Analysis window", True),
                (SPECTROGRAM, "Spectrogram", "Its spectrogram in the Analysis window", True),
                (NEW_WINDOW, "New analysis window", "Compare it in a new Analysis window", True)):
            a = m.addAction(text)
            a.setToolTip(tip)
            a.setEnabled(enabled)
            a.triggered.connect(lambda _=False, a_=action: self.model.request(a_, sel, keys))
        m.addSeparator()
        m.addAction("Delete [Del]", lambda: self.model.remove(sel))
        self.last_menu = m                       # (tests)
        m.exec(ev.screenPos().toPoint())


def attach_select(win: QMainWindow, model: SelectionModel, viewboxes: Sequence[pg.ViewBox],
                  after: Optional[QAction] = None) -> QAction:
    """The Select toggle (S) and Del in ``win``'s first toolbar (after ``after``), and
    the drag that makes a selection in each of ``viewboxes``. Returns the toggle."""
    act = QAction("Select", win, checkable=True)
    act.setShortcut(QKeySequence("S"))
    act.setToolTip("Drag in a plot to make a selection (time window) for the Analysis window;"
                   " drag its edges to change it (Select off: its body to move it),"
                   " right-click it for its menu [S]")
    tb = win.findChildren(QToolBar)[0]
    acts = tb.actions()
    nxt = acts[acts.index(after) + 1] if after in acts and acts.index(after) + 1 < len(acts) else None
    tb.insertAction(nxt, act) if nxt is not None else tb.addAction(act)
    w = tb.widgetForAction(act)
    if w is not None:
        w.setFocusPolicy(Qt.FocusPolicy.NoFocus)
    act.toggled.connect(lambda on: win.statusBar().showMessage(
        "Select: drag in a plot to make a selection (S again: off)") if on
        else win.statusBar().clearMessage())
    delete = QAction("Delete selection", win)
    delete.setShortcuts([QKeySequence(Qt.Key.Key_Delete), QKeySequence(Qt.Key.Key_Backspace)])
    delete.triggered.connect(lambda: model.remove(model.hovered))
    win.addAction(delete)
    for vb in viewboxes:
        vb.select_action = act               # the regions there leave drags to Select
        _hook_drag(win, vb, model, act)
    return act


def _hook_drag(win: QMainWindow, vb: pg.ViewBox, model: SelectionModel, act: QAction) -> None:
    state: dict = {"sel": None}

    def drag(ev, axis=None):
        if axis is not None or not act.isChecked() or ev.button() != Qt.MouseButton.LeftButton:
            return type(vb).mouseDragEvent(vb, ev, axis)        # the class's own: pan / zoom box
        ev.accept()
        x0 = vb.mapSceneToView(ev.buttonDownScenePos()).x()
        x = vb.mapSceneToView(ev.scenePos()).x()
        if ev.isStart():
            sel = model.create(x0, x)
            if isinstance(sel, str):
                win.statusBar().showMessage(f"Select: {sel}", 5000)
                sel = None
            state["sel"] = sel
        sel = state["sel"]
        if sel is None:
            return
        model.set_range(sel, x0, x)
        if ev.isFinish():
            state["sel"] = None
            if abs(ev.scenePos().x() - ev.buttonDownScenePos().x()) < _MIN_PX:
                model.remove(sel)
            else:
                win.statusBar().showMessage(
                    f"Selection {sel.number}: {sel.t0:.2f} .. {sel.t1:.2f} s; right-click it"
                    " for Compare", 5000)
    vb.mouseDragEvent = drag
