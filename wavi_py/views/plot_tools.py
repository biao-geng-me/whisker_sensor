"""Plot tools: the one drag tool armed in a plot window, the same in every window.

A window's drag tools are exclusive: Pan (nothing armed: a drag pans, the wheel
zooms), Select (S) and Zoom box (Z). Arming one disarms the other; its key (or
its button) again goes back to Pan. ``PlotTools`` owns the toolbar actions (a
``QActionGroup``, ExclusiveOptional) and keeps the plots it was given in step
(``add``): each ViewBox's mouse mode (RectMode for Zoom box) and each
GraphicsView's cursor (``tool_cursor``: a cross for Select, a magnifier with a +
for Zoom box, the arrow for Pan). A plot set only takes the tools it handles
(``supports``); under another tool it pans and shows the arrow, so the cursor
says where the armed tool acts. Plots made later: ``apply`` / ``refresh``.

What a Select drag makes is the window's own (a selection in Line, IMU and the
Analysis timeline; a sub-range in the Analysis waveform): its drag hooks read
the Select action (``tools.action(SELECT).isChecked()``) when the drag starts.

Probe is not one of these: it changes what a hover shows, not what a drag does,
so it stays a separate toggle that works under any tool.
"""

from __future__ import annotations

from typing import Callable, Iterable, Optional, Sequence, Union

from PyQt6.QtCore import QObject, QPointF, Qt, pyqtSignal
from PyQt6.QtGui import QAction, QActionGroup, QColor, QCursor, QKeySequence, QPainter, QPen, QPixmap

import pyqtgraph as pg

PAN, SELECT, ZOOM = "pan", "select", "zoom"
_SPEC = {SELECT: ("Select", "S"), ZOOM: ("Zoom box", "Z")}

_ZOOM_CURSOR: Optional[QCursor] = None


def zoom_cursor() -> QCursor:
    """A magnifier with a + in its lens (Zoom box armed); the hot spot is the lens' centre."""
    global _ZOOM_CURSOR
    if _ZOOM_CURSOR is None:
        pm = QPixmap(32, 32)
        pm.fill(Qt.GlobalColor.transparent)
        p = QPainter(pm)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        for color, w in ((QColor(255, 255, 255), 5.0), (QColor(20, 20, 20), 2.2)):
            p.setPen(QPen(color, w, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(QPointF(12, 12), 8.5, 8.5)
            p.drawLine(QPointF(18.5, 18.5), QPointF(28, 28))
            p.drawLine(QPointF(8, 12), QPointF(16, 12))
            p.drawLine(QPointF(12, 8), QPointF(12, 16))
        p.setPen(QPen(QColor(20, 20, 20), 4.0, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        p.drawLine(QPointF(20, 20), QPointF(27, 27))         # the handle, thicker
        p.end()
        _ZOOM_CURSOR = QCursor(pm, 12, 12)
    return _ZOOM_CURSOR


def tool_cursor(tool: str) -> Optional[QCursor]:
    """The plot cursor under ``tool`` (None: the arrow)."""
    if tool == ZOOM:
        return zoom_cursor()
    if tool == SELECT:
        return QCursor(Qt.CursorShape.CrossCursor)
    return None


def effective(tool: str, supports: Iterable[str]) -> str:
    return tool if tool in supports else PAN


def apply_tool(tool: str, viewboxes: Iterable[pg.ViewBox] = (), views: Iterable = (),
               supports: Iterable[str] = (SELECT, ZOOM)) -> None:
    """Put ``viewboxes`` in ``tool``'s mouse mode and ``views`` (GraphicsViews) in its
    cursor; a tool not in ``supports`` counts as Pan."""
    t = effective(tool, tuple(supports))
    mode = pg.ViewBox.RectMode if t == ZOOM else pg.ViewBox.PanMode
    for vb in viewboxes:
        if vb.state["mouseMode"] != mode:
            vb.setMouseMode(mode)
    c = tool_cursor(t)
    for v in views:
        vp = v.viewport()
        if c is None:
            vp.unsetCursor()
        else:
            vp.setCursor(c)


Plots = Union[Sequence, Callable[[], Sequence]]


class PlotTools(QObject):
    """A window's exclusive drag tools (see the module doc). ``changed(tool)``."""

    changed = pyqtSignal(str)

    def __init__(self, win):
        super().__init__(win)
        self._win = win
        self.group = QActionGroup(self)
        self.group.setExclusionPolicy(QActionGroup.ExclusionPolicy.ExclusiveOptional)
        self.actions: dict[str, QAction] = {}
        self._targets: list[tuple] = []       # (viewboxes, views, supports): lists or callables
        self.tool = PAN

    def action(self, tool: str, tip: str = "") -> QAction:
        """``tool``'s toggle (made on first use; its key: S / Z)."""
        a = self.actions.get(tool)
        if a is None:
            label, key = _SPEC[tool]
            a = QAction(label, self._win, checkable=True)
            a.setShortcut(QKeySequence(key))
            self.group.addAction(a)
            a.toggled.connect(self._sync)
            self.actions[tool] = a
        if tip:
            a.setToolTip(tip)
        return a

    def set_tool(self, tool: str) -> None:
        if tool in self.actions:
            self.actions[tool].setChecked(True)
        else:
            for a in self.actions.values():
                a.setChecked(False)

    def add(self, viewboxes: Plots = (), views: Plots = (),
            supports: Iterable[str] = (SELECT, ZOOM)) -> None:
        """Keep these plots in step with the tool (lists, or callables giving the
        current ones: they are read again on every change and ``refresh``)."""
        self._targets.append((viewboxes, views, tuple(supports)))
        self._apply_target(self._targets[-1])

    def apply(self, viewboxes: Iterable[pg.ViewBox] = (), views: Iterable = (),
              supports: Iterable[str] = (SELECT, ZOOM)) -> None:
        """Bring plots made after ``add`` in step (a rebuilt row)."""
        apply_tool(self.tool, viewboxes, views, supports)

    def refresh(self) -> None:
        for t in self._targets:
            self._apply_target(t)

    def _apply_target(self, t: tuple) -> None:
        vbs, views, supports = t
        apply_tool(self.tool, vbs() if callable(vbs) else vbs,
                   views() if callable(views) else views, supports)

    def _sync(self, *_a) -> None:
        # read the whole state: during a switch both may be checked for a moment
        checked = [t for t, a in self.actions.items() if a.isChecked()]
        tool = checked[0] if len(checked) == 1 else (PAN if not checked else self.tool)
        if tool == self.tool:
            return
        self.tool = tool
        self.refresh()
        self.changed.emit(tool)
