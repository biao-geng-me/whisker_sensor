"""The notes of one recording in the GUI (``replay/notes.py``): the model the
control panel, the timelines and the notes list share, the Add note dialog,
and the notes list.

``NotesModel`` follows one recording (the open replay, the one recording now,
else the last one recorded): its ``.notes.jsonl``, re-read when the file
changes (another window, or the sync, may append; a 2 s stat poll), and its
``offset``: where the recording's time zero sits on the timeline shown (0 in
replay; live, the recording's start in the stream's session seconds; None
when the timeline shows no part of it, so no markers are drawn).

Every change appends a line, so nothing here rewrites the file: an edit is a
``del`` and a new note, Mark bad a ``flag``.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

from PyQt6.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt6.QtWidgets import (QCheckBox, QDialog, QDialogButtonBox, QHBoxLayout, QHeaderView,
                             QLabel, QMessageBox, QPlainTextEdit, QPushButton, QTreeWidget,
                             QTreeWidgetItem, QVBoxLayout, QWidget)

from wavi_py.replay import notes as nt
from wavi_py.replay.recname import stem_of

POLL_MS = 2000


def fmt_note_t(t: Optional[float]) -> str:
    if t is None:
        return "whole"
    m, s = divmod(max(0.0, float(t)), 60.0)
    return f"{int(m)}:{s:04.1f}"


class NotesModel(QObject):
    changed = pyqtSignal()                  # the file, the recording or the offset

    def __init__(self, who: str = "", parent: Optional[QObject] = None):
        super().__init__(parent)
        self.who = who
        self.recording: Optional[Path] = None
        self.offset: Optional[float] = None
        self.state = nt.NotesState()
        self._sig: Optional[tuple] = None
        self._timer = QTimer(self)
        self._timer.setInterval(POLL_MS)
        self._timer.timeout.connect(self.refresh)

    # ----- the recording

    @property
    def path(self) -> Optional[Path]:
        return nt.notes_path(self.recording) if self.recording is not None else None

    @property
    def stem(self) -> str:
        return stem_of(self.recording) if self.recording is not None else ""

    def set_recording(self, recording, offset: Optional[float] = None) -> None:
        self.recording = Path(recording) if recording else None
        self.offset = offset
        self._sig = None
        self.state = nt.NotesState()
        if self.recording is None:
            self._timer.stop()
        else:
            self._timer.start()
            self.refresh(force=True)
        self.changed.emit()

    def set_offset(self, offset: Optional[float]) -> None:
        if offset != self.offset:
            self.offset = offset
            self.changed.emit()

    def refresh(self, force: bool = False) -> None:
        """Re-read the file if it changed (size or mtime)."""
        p = self.path
        if p is None:
            return
        try:
            st = p.stat()
            sig = (st.st_size, st.st_mtime_ns)
        except OSError:
            sig = None
        if sig == self._sig and not force:
            return
        self._sig = sig
        self.state = nt.load(p) if sig is not None else nt.NotesState()
        self.changed.emit()

    # ----- changes (OSError goes to the caller: a read-only folder, say)

    def _append(self, line: dict) -> dict:
        assert self.path is not None
        nt.append(self.path, line)
        self.refresh(force=True)
        return line

    def add(self, text: str, t: Optional[float]) -> dict:
        return self._append(nt.note(text, t, by=self.who))

    def remove(self, note: dict) -> None:
        self._append(nt.delete(note["id"], by=self.who))

    def edit(self, note: dict, text: str, t: Optional[float]) -> dict:
        nt.append(self.path, nt.delete(note["id"], by=self.who))
        return self._append(nt.note(text, t, by=self.who))

    def set_flag(self, keep: bool, text: str = "") -> dict:
        return self._append(nt.flag(keep, text, by=self.who))


def ask_note(parent: QWidget, title: str, t: Optional[float], text: str = "",
             pinned: bool = True) -> Optional[tuple[str, Optional[float]]]:
    """(text, t or None) from a small dialog, or None if cancelled / empty. ``t``
    None: no time to pin to, so the note is on the whole recording."""
    dlg = QDialog(parent)
    dlg.setWindowTitle(title)
    v = QVBoxLayout(dlg)
    edit = QPlainTextEdit(text)
    edit.setPlaceholderText("note")
    edit.setMinimumSize(360, 90)
    v.addWidget(edit)
    pin = QCheckBox(f"Pin to {fmt_note_t(t)} into the recording" if t is not None
                    else "Whole recording (no time to pin to)")
    pin.setChecked(t is not None and pinned)
    pin.setEnabled(t is not None)
    pin.setToolTip("Unchecked: a note on the whole recording")
    v.addWidget(pin)
    bb = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
    bb.accepted.connect(dlg.accept)
    bb.rejected.connect(dlg.reject)
    v.addWidget(bb)
    edit.setFocus()
    if dlg.exec() != QDialog.DialogCode.Accepted:
        return None
    s = edit.toPlainText().strip()
    if not s:
        return None
    return s, (t if pin.isChecked() else None)


class NotesDialog(QDialog):
    """The notes list: time, text, who, when. Double-click a pinned note to go
    there; Edit (a del and a new note), Delete, Add."""

    def __init__(self, model: NotesModel, add_note: Callable[[], None],
                 seek: Callable[[float], None], parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.model = model
        self._add_note = add_note
        self._seek = seek
        self.setWindowTitle("Notes")
        self.resize(560, 300)
        v = QVBoxLayout(self)
        self.head = QLabel()
        self.head.setWordWrap(True)
        v.addWidget(self.head)
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["t", "note", "by", "when"])
        self.tree.setRootIsDecorated(False)
        self.tree.header().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        self.tree.itemDoubleClicked.connect(self._go)
        self.tree.itemSelectionChanged.connect(self._sel_changed)
        v.addWidget(self.tree, stretch=1)
        h = QHBoxLayout()
        v.addLayout(h)
        self.add_btn = QPushButton("Add...")
        self.add_btn.clicked.connect(lambda: self._add_note())
        self.edit_btn = QPushButton("Edit...")
        self.edit_btn.clicked.connect(self._edit)
        self.del_btn = QPushButton("Delete")
        self.del_btn.clicked.connect(self._delete)
        close = QPushButton("Close")
        close.clicked.connect(self.close)
        for b in (self.add_btn, self.edit_btn, self.del_btn):
            h.addWidget(b)
        h.addStretch(1)
        h.addWidget(close)
        model.changed.connect(self._fill)
        self._fill()

    def _fill(self) -> None:
        m = self.model
        st = m.state
        if m.recording is None:
            self.head.setText("No recording (open one, or record).")
        else:
            flag = (f"<b style='color:#b00'>marked bad</b>"
                    + (f": {st.reason}" if st.reason else "")) if st.bad else "not marked bad"
            self.head.setText(f"<b>{m.stem}</b> — {st.count} note(s), {flag}")
        self.tree.clear()
        for n in st.notes:
            when = datetime.fromtimestamp(float(n.get("unix") or 0)).strftime("%Y-%m-%d %H:%M")
            it = QTreeWidgetItem([fmt_note_t(n.get("t")), str(n.get("text", "")),
                                  str(n.get("by", "")), when])
            it.setData(0, Qt.ItemDataRole.UserRole, n)
            it.setToolTip(1, str(n.get("text", "")))
            self.tree.addTopLevelItem(it)
        for c in (0, 2, 3):
            self.tree.resizeColumnToContents(c)
        self.add_btn.setEnabled(m.recording is not None)
        self._sel_changed()

    def _current(self) -> Optional[dict]:
        it = self.tree.currentItem()
        return it.data(0, Qt.ItemDataRole.UserRole) if it is not None else None

    def _sel_changed(self) -> None:
        on = self._current() is not None and bool(self.tree.selectedItems())
        self.edit_btn.setEnabled(on)
        self.del_btn.setEnabled(on)

    def _go(self, item, _col=0) -> None:
        n = item.data(0, Qt.ItemDataRole.UserRole)
        if n and n.get("t") is not None:
            self._seek(float(n["t"]))

    def _edit(self) -> None:
        n = self._current()
        if n is None:
            return
        got = ask_note(self, "Edit note", n.get("t"), str(n.get("text", "")))
        if got is None:
            return
        try:
            self.model.edit(n, got[0], got[1])
        except OSError as exc:
            QMessageBox.critical(self, "Note not saved", str(exc))

    def _delete(self) -> None:
        n = self._current()
        if n is None:
            return
        if QMessageBox.question(self, "Delete note", f"Delete “{n.get('text', '')}”?") \
                != QMessageBox.StandardButton.Yes:
            return
        try:
            self.model.remove(n)
        except OSError as exc:
            QMessageBox.critical(self, "Note not deleted", str(exc))


def note_tooltip(n: dict) -> str:
    who = f" ({n['by']})" if n.get("by") else ""
    return f"{fmt_note_t(n.get('t'))}  {n.get('text', '')}{who}"

