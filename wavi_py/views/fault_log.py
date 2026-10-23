"""The session's ESP32 / ADC fault events (onsets, clears, warnings), one per
line, newest last. Fed by the control panel from the ``mark`` events of the
fault model (``whisker_link/esp32_fault.py``)."""

from __future__ import annotations

from PyQt6.QtGui import QFontDatabase
from PyQt6.QtWidgets import QPlainTextEdit

from wavi_py.views.base import ViewWindow


class FaultLogView(ViewWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Wavi - ESP32 / ADC fault log")
        self.resize(900, 300)
        self._text = QPlainTextEdit()
        self._text.setReadOnly(True)
        self._text.setFont(QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont))
        self._text.setPlaceholderText("No ESP32 / ADC faults or warnings this session.")
        self.setCentralWidget(self._text)

    def append(self, line: str) -> None:
        self._text.appendPlainText(line)

    def text(self) -> str:
        return self._text.toPlainText()
