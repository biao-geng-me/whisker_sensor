"""Base class for the view windows."""

from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QMainWindow


class ViewWindow(QMainWindow):
    """A view window that reports when the user closes it.

    Closing only hides the window (it is reused on the next toggle); ``closed``
    lets the control panel uncheck the matching view button.
    """

    closed = pyqtSignal()

    def closeEvent(self, event) -> None:  # noqa: N802 — Qt naming
        super().closeEvent(event)
        self.closed.emit()
