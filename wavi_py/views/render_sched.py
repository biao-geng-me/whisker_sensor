"""One render scheduler for all plot windows.

New data (or a zoom) only marks a window dirty (``request``); the scheduler
redraws it later from a timer:

- at most at the window's own rate (``register(win, max_hz)``);
- one window per event-loop pass, the most urgent first (waited longest
  relative to its ``min_hz`` interval: the Line window before the spectra),
  so the paints
  and other timers (the Dome view's frame timer, input) get in between
  instead of waiting for every window's redraw in one block;
- not while the window is hidden, minimised or not exposed (fully covered,
  where the platform reports it); it is redrawn when it shows again;
- with a global budget: when the windows together take more than
  ``BUDGET`` of the GUI thread (redraws plus their paints, averaged over
  about a second), the windows' intervals are stretched to fit, but none
  below its ``min_hz`` (the Line window keeps 20 fps; the spectra give way
  first), so the Dome view and input stay responsive however many windows
  are open.

Windows report their paint time with ``note_paint`` (the ``PlotWidget`` /
``GraphicsLayoutWidget`` here do it for every pyqtgraph view).
"""

from __future__ import annotations

import time
import weakref
from typing import Callable

from PyQt6.QtCore import QObject, QTimer
from PyQt6.QtWidgets import QWidget

import pyqtgraph as pg

BUDGET = 0.5                    # share of the GUI thread the windows may take together
_MAX_STRETCH = 4.0
_LOAD_PERIOD_S = 1.0
_POLL_S = 0.1                   # a shown window not exposed yet (or minimised): look again


class _Entry:
    __slots__ = ("render", "interval", "longest", "dirty", "last", "cost")

    def __init__(self, render: Callable[[], None], max_hz: float, min_hz: float):
        self.render = render
        self.interval = 1.0 / max_hz
        self.longest = 1.0 / min(min_hz, max_hz)
        self.dirty = False
        self.last = -1e9
        self.cost = 0.0                     # redraw + paint seconds in this load period


def exposed(win: QWidget) -> bool:
    """Shown, not minimised, and (where the platform says so) not fully covered."""
    if not win.isVisible() or win.isMinimized():
        return False
    handle = win.windowHandle()
    return handle is None or handle.isExposed()


class RenderScheduler(QObject):
    def __init__(self):
        super().__init__()
        self._entries: "weakref.WeakKeyDictionary[QWidget, _Entry]" = weakref.WeakKeyDictionary()
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._run)
        self.stretch = 1.0
        self._load_t0 = time.monotonic()
        self._busy = False
        self._current: QWidget | None = None       # being redrawn by _run

    def register(self, win: QWidget, render: Callable[[], None], max_hz: float,
                 min_hz: float | None = None) -> None:
        """``win`` is redrawn by ``render`` at most ``max_hz`` times a second, and the
        budget stretches that to no less than ``min_hz`` (default: a quarter)."""
        self._entries[win] = _Entry(render, max_hz, min_hz or max_hz / _MAX_STRETCH)

    def request(self, win: QWidget) -> None:
        """Redraw ``win`` soon; repeated requests coalesce."""
        e = self._entries.get(win)
        if e is None:
            return
        e.dirty = True
        self._arm()

    def done(self, win: QWidget) -> None:
        """``win`` was redrawn directly (a key, a layout switch): the next scheduled
        redraw keeps its interval from now."""
        e = self._entries.get(win)
        if e is not None and win is not self._current:
            e.dirty = False
            e.last = time.monotonic()

    def note_paint(self, win: QWidget, seconds: float) -> None:
        e = self._entries.get(win)
        if e is not None:
            e.cost += seconds

    def _interval(self, e: _Entry) -> float:
        return min(e.interval * self.stretch, e.longest)

    def _due(self, e: _Entry) -> float:
        return e.last + self._interval(e)

    def _arm(self) -> None:
        if self._busy:
            return                           # _run re-arms when it is done
        now = time.monotonic()
        waits = [self._due(e) if exposed(w) else now + _POLL_S
                 for w, e in self._entries.items() if e.dirty and w.isVisible()]
        if not waits:
            self._timer.stop()
            return
        wait = max(0.0, min(waits) - now)
        if not self._timer.isActive() or self._timer.remainingTime() > wait * 1000:
            self._timer.start(int(wait * 1000))

    def _run(self) -> None:
        now = time.monotonic()
        self._update_load(now)
        due = [(self._due(e), w, e) for w, e in self._entries.items()
               if e.dirty and self._due(e) <= now + 0.001 and exposed(w)]
        if due:
            _, w, e = max(due, key=lambda d: (now - d[2].last) / d[2].longest)
            e.dirty = False
            # the slot this redraw was planned for, not the time it ran: a late redraw
            # lets the next one come early, so a window whose interval equals its data
            # rate (the Line window at 20 Hz) does not drop every few frames
            iv = self._interval(e)
            e.last = max(e.last + iv, now - iv)
            self._busy, self._current = True, w
            try:
                e.render()
            finally:
                self._busy, self._current = False, None
                e.cost += time.monotonic() - now
        self._arm()

    def _update_load(self, now: float) -> None:
        dt = now - self._load_t0
        if dt < _LOAD_PERIOD_S:
            return
        load = sum(e.cost for e in self._entries.values()) / dt
        for e in self._entries.values():
            e.cost = 0.0
        self._load_t0 = now
        # load was measured at the current stretch; the unstretched load is load * stretch
        want = min(max(load * self.stretch / BUDGET, 1.0), _MAX_STRETCH)
        self.stretch = 0.5 * self.stretch + 0.5 * want


_SCHED: RenderScheduler | None = None


def scheduler() -> RenderScheduler:
    global _SCHED
    if _SCHED is None:
        _SCHED = RenderScheduler()
    return _SCHED


class _PaintTimed:
    """Reports each paint's duration to the scheduler, for the window it is in."""

    def paintEvent(self, ev) -> None:  # noqa: N802 — Qt naming
        t0 = time.perf_counter()
        super().paintEvent(ev)
        scheduler().note_paint(self.window(), time.perf_counter() - t0)


class PlotWidget(_PaintTimed, pg.PlotWidget):
    pass


class GraphicsLayoutWidget(_PaintTimed, pg.GraphicsLayoutWidget):
    pass
