"""Base class for the view windows."""

from __future__ import annotations

from typing import Callable, Optional

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (QDockWidget, QLabel, QMainWindow, QMenu, QProgressBar, QToolBar,
                             QToolButton, QWidget)

# the ESP32 / ADC alarm banner (see whisker_link/esp32_fault.py)
ALARM_STYLE = {
    "fault": "QLabel { background: #c00000; color: white; font-weight: bold; padding: 6px; }",
    "warn": "QLabel { background: #f0a000; color: black; padding: 4px; }",
}


def alarm_label() -> QLabel:
    lab = QLabel()
    lab.setWordWrap(True)
    lab.hide()
    return lab


def show_alarm(lab: QLabel, text: str, level: str) -> None:
    """``level`` ok hides the banner."""
    if level not in ALARM_STYLE or not text:
        lab.hide()
        return
    lab.setStyleSheet(ALARM_STYLE[level])
    lab.setText(text)
    lab.show()


def bring_to_front(win: QWidget) -> None:
    """Show ``win`` un-minimized on top and give it the focus (a window manager may
    still only flash it in the task bar instead; that is its policy)."""
    win.show()
    if win.isMinimized():
        win.setWindowState(win.windowState() & ~Qt.WindowState.WindowMinimized)
    win.raise_()
    win.activateWindow()


def fmt_secs(s: float) -> str:
    """42 s, 3:07, 1:02:03 (whole seconds)."""
    s = int(max(0.0, s))
    h, m = divmod(s // 60, 60)
    return f"{h}:{m:02d}:{s % 60:02d}" if h else (f"{m}:{s % 60:02d}" if m else f"{s} s")


# The recording indicator's states (RecIndicator.set_state); the control panel works
# them out from its Rec state and pushes them to every view window.
REC_OFF, REC_ON, REC_PENDING, REC_FOREIGN = "off", "on", "pending", "foreign"
_REC_STYLE = "QProgressBar {{ border: 1px solid {edge}; border-radius: 3px; background: {bg};" \
             " color: {fg}; text-align: center; }} QProgressBar::chunk {{ background: {chunk}; }}"
_REC_STYLES = {
    REC_OFF: _REC_STYLE.format(edge="#bbb", bg="#eee", fg="#888", chunk="#eee"),
    REC_PENDING: _REC_STYLE.format(edge="#bbb", bg="#eee", fg="#555", chunk="#eee"),
    REC_ON: _REC_STYLE.format(edge="#a00", bg="#f6dada", fg="black", chunk="#e04040"),
    REC_FOREIGN: _REC_STYLE.format(edge="#b06000", bg="#d07000", fg="white", chunk="#d07000"),
}


class RecIndicator(QProgressBar):
    """The recording state in a view window's status bar (display only; the control
    panel's Rec button and timer are the one source). Recording with auto-stop: the
    bar fills red toward the stop time, "REC 0:42 / 1:00"; without: a full red bar,
    "REC 0:42"; not recording: grey "not recording"; also "Starting..." while the
    server's reply is pending, and orange for a vehicle recording this client didn't
    start. A fixed width, so the status line doesn't shift as it counts."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setRange(0, 1000)
        self.setTextVisible(True)
        self.setFixedWidth(self.fontMetrics().horizontalAdvance("REC 10:00:00 / 10:00:00") + 16)
        self.setMaximumHeight(self.fontMetrics().height() + 4)
        self.state = ""
        self.set_state(REC_OFF)

    def set_state(self, state: str, elapsed: float = 0.0, total: float = 0.0,
                  tip: str = "") -> None:
        if state != self.state:
            self.state = state
            self.setStyleSheet(_REC_STYLES[state])
        if state == REC_ON and total > 0:
            self.setValue(int(round(1000 * min(1.0, elapsed / total))))
            text = f"REC {fmt_secs(elapsed)} / {fmt_secs(total)}"
        elif state == REC_ON:
            self.setValue(1000)
            text = f"REC {fmt_secs(elapsed)}"
        elif state == REC_FOREIGN:
            self.setValue(1000)
            text = f"ROV rec {fmt_secs(elapsed)}"
        else:
            self.setValue(0)
            text = "Starting…" if state == REC_PENDING else "not recording"
        self.setFormat(text)
        self.setToolTip(tip or {REC_ON: "Recording (Ctrl+R stops it)",
                                REC_OFF: "Not recording (Ctrl+R starts, as the panel's Rec)",
                                REC_PENDING: "Waiting for the server's reply",
                                REC_FOREIGN: "The ROV records a file this window didn't start"
                                }[state])


class ViewWindow(QMainWindow):
    """A view window that reports when the user closes it.

    Closing only hides the window (it is reused on the next toggle); ``closed``
    lets the control panel uncheck the matching view button. ``set_alarm``
    shows the ESP32 / ADC alarm banner above the window's toolbars.
    """

    closed = pyqtSignal()
    _alarm: Optional[QLabel] = None
    timeline_dock: Optional[QDockWidget] = None
    rec_indicator: Optional[RecIndicator] = None
    goto_button: Optional[QToolButton] = None

    def set_alarm(self, text: str, level: str) -> None:
        if self._alarm is None:
            if level not in ALARM_STYLE:
                return
            self._alarm = alarm_label()
            self.setMenuWidget(self._alarm)     # above the toolbars; no menu bar here
        show_alarm(self._alarm, text, level)

    def add_timeline(self, bar: QWidget) -> QDockWidget:
        """The replay / timeline bar as a dock at the bottom (movable, floatable,
        closable; the first toolbar gets a Timeline toggle)."""
        self.timeline_dock = add_timeline_dock(self, bar)
        return self.timeline_dock

    def add_chrome(self, fill_goto: Callable[[QMenu], None]) -> None:
        """The panel's additions to a plot window: a "Go to" menu at the left of the
        first toolbar (``fill_goto`` lists the views into it each time it opens) and
        the recording indicator at the right of the status bar."""
        tb = self.findChildren(QToolBar)[0]
        btn = QToolButton(tb)
        btn.setText("Go to")
        btn.setToolTip("Open another view, or bring it to the front (✓: open)")
        btn.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
        btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        menu = QMenu(btn)
        menu.aboutToShow.connect(lambda: (menu.clear(), fill_goto(menu)))
        btn.setMenu(menu)
        acts = tb.actions()
        if acts:
            tb.insertWidget(acts[0], btn)
            tb.insertSeparator(acts[0])
        else:
            tb.addWidget(btn)
        self.goto_button = btn
        self.rec_indicator = RecIndicator()
        self.statusBar().addPermanentWidget(self.rec_indicator)

    def closeEvent(self, event) -> None:  # noqa: N802 — Qt naming
        super().closeEvent(event)
        self.closed.emit()


def add_timeline_dock(win: QMainWindow, bar: QWidget) -> QDockWidget:
    dock = QDockWidget("Timeline", win)
    dock.setObjectName("timeline")
    dock.setWidget(bar)
    dock.setFeatures(QDockWidget.DockWidgetFeature.DockWidgetMovable
                     | QDockWidget.DockWidgetFeature.DockWidgetFloatable
                     | QDockWidget.DockWidgetFeature.DockWidgetClosable)
    win.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, dock)
    act = dock.toggleViewAction()
    act.setToolTip("Show / hide the timeline (replay controls, depth, events)")
    tbs = win.findChildren(QToolBar)
    if tbs:
        tbs[0].addAction(act)
    dock.hide()               # hidden by default; the panel shows its own with a source
    return dock
