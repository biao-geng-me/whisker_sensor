"""Base class for the view windows."""

from __future__ import annotations

from typing import Optional

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QLabel, QMainWindow

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


class ViewWindow(QMainWindow):
    """A view window that reports when the user closes it.

    Closing only hides the window (it is reused on the next toggle); ``closed``
    lets the control panel uncheck the matching view button. ``set_alarm``
    shows the ESP32 / ADC alarm banner above the window's toolbars.
    """

    closed = pyqtSignal()
    _alarm: Optional[QLabel] = None

    def set_alarm(self, text: str, level: str) -> None:
        if self._alarm is None:
            if level not in ALARM_STYLE:
                return
            self._alarm = alarm_label()
            self.setMenuWidget(self._alarm)     # above the toolbars; no menu bar here
        show_alarm(self._alarm, text, level)

    def closeEvent(self, event) -> None:  # noqa: N802 — Qt naming
        super().closeEvent(event)
        self.closed.emit()
