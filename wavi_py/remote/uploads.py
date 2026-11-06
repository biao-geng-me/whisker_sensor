"""The Uploads window: every recording in the upload queue and the ones
finished this session, each expandable to its files, with the run's progress.

It draws ``SyncController.tracker`` (``remote/progress.py``) and redraws on the
controller's ``progress`` signal (the worker's ``SyncEvent``s, queued to the GUI
thread), so it never asks the worker anything and does no I/O: the persistent
queue is read on a thread (``SyncController.refresh_queue``) when it opens.
A one-second timer keeps the rate, the ETA and the retry time current.

One instance (``SyncController.show_uploads``), a top-level window with no
parent: the Cluster... dialog that opens it is modal to the control panel only,
and a window parented to the panel would be blocked with it.

Columns: recording (date, time, tag) or file; status (queued / waiting /
checking the cluster / hashing / uploading / verifying / done / skipped /
failed / offline / removed); a bar; size; when it finished, or why it failed.
Actions: Retry now (push at once instead of at the next 5-min retry),
Pause / Resume (the run stops after the file being sent; that file completes),
Remove from queue (dequeues, never deletes a file), Clear finished, and on a
done row "Copy cluster path".
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Optional

from PyQt6.QtCore import QModelIndex, Qt, QTimer
from PyQt6.QtGui import QAction, QBrush, QColor, QGuiApplication, QPalette
from PyQt6.QtWidgets import (QHBoxLayout, QHeaderView, QLabel, QMenu, QProgressBar,
                             QPushButton, QStyledItemDelegate, QTreeWidget, QTreeWidgetItem,
                             QVBoxLayout, QWidget)

from wavi_py.remote.progress import FINISHED, FileRow, RecRow, fmt_eta, fmt_mb

if TYPE_CHECKING:
    from wavi_py.remote.qt import SyncController

COLS = ("Recording / file", "Status", "Progress", "Size", "Finished / error")
C_NAME, C_STATUS, C_BAR, C_SIZE, C_END = range(5)
PCT_ROLE = Qt.ItemDataRole.UserRole + 1
STEM_ROLE = Qt.ItemDataRole.UserRole + 2

_COLOR = {"done": "#2a7a2a", "skipped": "#2a7a2a", "failed": "#c00", "offline": "#b06000",
          "waiting": "#b06000", "removed": "#777", "paused": "#b06000",
          "uploading": "#1f5fa8", "hashing": "#1f5fa8", "verifying": "#1f5fa8",
          "checking": "#1f5fa8"}


class _BarDelegate(QStyledItemDelegate):
    """The Progress column: a flat bar with the percentage beside it, from the
    item's ``PCT_ROLE`` (nothing when it is -1). Painted by hand: a style's
    progress bar squeezed into a tree row clips its own text."""

    def paint(self, painter, option, index: QModelIndex) -> None:
        super().paint(painter, option, index)
        pct = index.data(PCT_ROLE)
        if pct is None or pct < 0:
            return
        r = option.rect.adjusted(3, 4, -3, -4)
        tw = option.fontMetrics.horizontalAdvance("100%") + 6
        bar = r.adjusted(0, 0, -tw, 0)
        pal = option.palette
        painter.save()
        painter.setPen(pal.color(QPalette.ColorRole.Mid))
        painter.setBrush(pal.color(QPalette.ColorRole.Base))
        painter.drawRect(bar.adjusted(0, 0, -1, -1))
        fill = bar.adjusted(1, 1, -1, -1)
        fill.setWidth(int(fill.width() * max(0, min(100, pct)) / 100))
        painter.fillRect(fill, QColor("#3d7fd1"))
        painter.setPen(pal.color(QPalette.ColorRole.Text))
        painter.drawText(r.adjusted(r.width() - tw + 4, -4, 0, 4),
                         int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter), f"{pct}%")
        painter.restore()


def _when(t: Optional[float]) -> str:
    if not t:
        return ""
    lt = time.localtime(t)
    return time.strftime("%H:%M:%S" if time.time() - t < 20 * 3600 else "%m-%d %H:%M", lt)


class UploadsWindow(QWidget):
    def __init__(self, ctl: "SyncController"):
        super().__init__(None, Qt.WindowType.Window)
        self.setWindowTitle("Wavi - Uploads")
        self.resize(900, 480)
        self._ctl = ctl
        self._items: dict[str, QTreeWidgetItem] = {}
        self._version = -1

        self.overall = QLabel("")
        self.bar = QProgressBar()
        self.bar.setRange(0, 1000)
        self.bar.setTextVisible(False)
        self.bar.setMaximumHeight(10)
        self.conn = QLabel("")
        self.conn.setWordWrap(True)
        self.conn.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)

        self.tree = QTreeWidget()
        self.tree.setColumnCount(len(COLS))
        self.tree.setHeaderLabels(COLS)
        self.tree.setUniformRowHeights(True)
        self.tree.setAlternatingRowColors(True)
        self.tree.setSelectionMode(QTreeWidget.SelectionMode.ExtendedSelection)
        self.tree.setItemDelegateForColumn(C_BAR, _BarDelegate(self.tree))
        hdr = self.tree.header()
        hdr.setStretchLastSection(True)
        for c, w in ((C_NAME, 300), (C_STATUS, 150), (C_BAR, 120), (C_SIZE, 80)):
            hdr.resizeSection(c, w)
        hdr.setSectionResizeMode(C_NAME, QHeaderView.ResizeMode.Interactive)
        self.tree.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.tree.customContextMenuRequested.connect(self._menu)
        self.tree.itemSelectionChanged.connect(self._sync_buttons)

        self.retry_btn = QPushButton("Retry now")
        self.retry_btn.setToolTip("Test the connection and push the queue now"
                                  " (instead of at the next retry, every 5 min)")
        self.retry_btn.clicked.connect(ctl.retry_now)
        self.pause_btn = QPushButton("Pause")
        self.pause_btn.setToolTip("Stop uploading after the file being sent (it completes);"
                                  "\nnothing is uploaded until Resume. Not kept across restarts.")
        self.pause_btn.clicked.connect(lambda: ctl.set_paused(not ctl.paused))
        self.remove_btn = QPushButton("Remove from queue")
        self.remove_btn.setToolTip("Take the selected recordings out of the upload queue."
                                   "\nNo file is deleted; Upload existing... queues them again.")
        self.remove_btn.clicked.connect(self._remove_selected)
        self.clear_btn = QPushButton("Clear finished")
        self.clear_btn.clicked.connect(self._clear_finished)
        h = QHBoxLayout()
        for b in (self.retry_btn, self.pause_btn, self.remove_btn):
            h.addWidget(b)
        h.addStretch(1)
        h.addWidget(self.clear_btn)

        v = QVBoxLayout(self)
        v.addWidget(self.overall)
        v.addWidget(self.bar)
        v.addWidget(self.conn)
        v.addWidget(self.tree, stretch=1)
        v.addLayout(h)

        ctl.progress.connect(self._on_event)
        ctl.queue_loaded.connect(self.redraw)
        ctl.status.connect(lambda *a: self._header())
        self._timer = QTimer(self)
        self._timer.setInterval(1000)
        self._timer.timeout.connect(self._header)
        self._timer.start()
        self.redraw()

    # ----- drawing

    def _on_event(self, ev) -> None:
        t = self._ctl.tracker
        if t.version != self._version or (ev.stem and ev.stem not in self._items):
            self.redraw()
            return
        if ev.stem:
            r = t.recs.get(ev.stem)
            if r is not None:
                self._fill(self._items[ev.stem], r)
        self._header()

    def redraw(self) -> None:
        """Rows in the tracker's order; existing items are reused (their
        expansion and selection stay)."""
        t = self._ctl.tracker
        self._version = t.version
        order = t.order()
        for stem in [s for s in self._items if s not in t.recs]:
            it = self._items.pop(stem)
            self.tree.takeTopLevelItem(self.tree.indexOfTopLevelItem(it))
        current = [self.tree.topLevelItem(i).data(C_NAME, STEM_ROLE)
                   for i in range(self.tree.topLevelItemCount())]
        if current != order:
            expanded = {s for s, it in self._items.items() if it.isExpanded()}
            selected = {s for s, it in self._items.items() if it.isSelected()}
            while self.tree.topLevelItemCount():
                self.tree.takeTopLevelItem(0)
            for stem in order:
                it = self._items.get(stem)
                if it is None:
                    it = self._items[stem] = QTreeWidgetItem()
                    it.setData(C_NAME, STEM_ROLE, stem)
                self.tree.addTopLevelItem(it)
                it.setExpanded(stem in expanded)
                it.setSelected(stem in selected)
        for stem in order:
            self._fill(self._items[stem], t.recs[stem])
        self._header()
        self._sync_buttons()

    def _fill(self, it: QTreeWidgetItem, r: RecRow) -> None:
        status = r.status()
        if r.phase in FINISHED:
            end = _when(r.t_end) + (f"  {r.info or r.error}" if r.info or r.error else "")
        elif r.phase in ("failed", "offline"):
            end = f"{_when(r.t_end)}  {r.error}".strip()
        else:
            end = f"last try: {r.error}" if r.error else ""
        self._cells(it, r.label, status, r.pct, r.size, end,
                    r.phase if r.active is None else r.active.phase)
        tip = r.stem + (f"\n{self._ctl.cfg.host}:{r.rdir}" if r.rdir else "")
        if r.bytes_sent:
            tip += f"\nsent {fmt_mb(r.bytes_sent)} this push"
        if r.error:
            tip += f"\n{r.error}"
        it.setToolTip(C_NAME, tip)
        it.setToolTip(C_END, r.error or tip)
        names = list(r.files)
        kids = {it.child(i).data(C_NAME, STEM_ROLE): it.child(i) for i in range(it.childCount())}
        if list(kids) != names:
            it.takeChildren()
            kids = {}
            for n in names:
                c = kids[n] = QTreeWidgetItem()
                c.setData(C_NAME, STEM_ROLE, n)
                it.addChild(c)
        for n, f in r.files.items():
            self._file(kids[n], f, r.stem)

    def _file(self, it: QTreeWidgetItem, f: FileRow, stem: str) -> None:
        word = {"skipped": "skipped (on the cluster)"}.get(f.phase, f.phase)
        end = (f.error or (_when(f.t_end) + (f"  {f.info}" if f.info else ""))
               if f.phase in ("done", "skipped", "failed") else "")
        short = f.name[len(stem):] if f.name.startswith(stem) and f.name != stem else f.name
        self._cells(it, short, word, f.pct, f.size, end, f.phase)
        it.setToolTip(C_NAME, f.name)
        it.setToolTip(C_END, f.error or end)

    def _cells(self, it, name, status, pct, size, end, phase) -> None:
        it.setText(C_NAME, name)
        it.setText(C_STATUS, status)
        it.setData(C_BAR, PCT_ROLE, pct)
        it.setText(C_SIZE, fmt_mb(size) if size else "")
        it.setTextAlignment(C_SIZE, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        it.setText(C_END, end)
        col = _COLOR.get(phase)
        it.setForeground(C_STATUS, QBrush(QColor(col)) if col else
                         QGuiApplication.palette().text())
        it.setForeground(C_END, QBrush(QColor("#c00")) if phase in ("failed", "offline")
                         else QGuiApplication.palette().text())

    def _header(self) -> None:
        t = self._ctl.tracker
        s = t.totals()
        queued = sum(r.phase not in FINISHED for r in t.recs.values())
        if s["n"]:
            word = "Uploading" if s["running"] else "Last run"
            parts = [f"{word}: recording {s['k']} of {s['n']}" if s["running"] else
                     f"{word}: {s['n_done']} of {s['n']} recordings done",
                     f"{fmt_mb(s['done'])} of {fmt_mb(s['total'])}"]
            if s["running"]:
                if s["rate"]:
                    parts.append(f"{s['rate'] / 1e6:.1f} MB/s")
                if s["eta"] is not None:
                    parts.append(f"ETA {fmt_eta(s['eta'])}")
            elif s["secs"]:
                parts.append(f"in {fmt_eta(s['secs'])}")
            text = "   ·   ".join(parts)
            self.bar.setValue(int(1000 * s["done"] / s["total"]) if s["total"] else
                              (1000 if not s["running"] else 0))
        else:
            text = "No upload has run yet in this session."
            self.bar.setValue(0)
        self.overall.setText(f"<b>{text}</b>   ({queued} in the queue)")
        level, msg = self._ctl.connection_state()
        col = {"ok": "#2a7a2a", "offline": "#b06000", "error": "#c00", "paused": "#b06000",
               "busy": "#1f5fa8"}.get(level, "#777")
        self.conn.setText(f"<span style='color:{col}'>Connection: {msg}</span>")
        self.pause_btn.setText("Resume" if self._ctl.paused else "Pause")
        self.retry_btn.setEnabled(self._ctl.cfg.configured and not self._ctl.paused)

    def _sync_buttons(self) -> None:
        sel = self._selected_stems()
        t = self._ctl.tracker
        self.remove_btn.setEnabled(any(s in t.recs and t.recs[s].phase not in FINISHED
                                       for s in sel))

    # ----- actions

    def _selected_stems(self) -> list[str]:
        out = []
        for it in self.tree.selectedItems():
            top = it if it.parent() is None else it.parent()
            s = top.data(C_NAME, STEM_ROLE)
            if s and s not in out:
                out.append(s)
        return out

    def _remove_selected(self) -> None:
        t = self._ctl.tracker
        stems = [s for s in self._selected_stems() if s in t.recs
                 and t.recs[s].phase not in FINISHED]
        if stems:
            self._ctl.remove(stems)

    def _clear_finished(self) -> None:
        self._ctl.tracker.clear_finished()
        self.redraw()

    def _menu(self, pos) -> None:
        it = self.tree.itemAt(pos)
        if it is None:
            return
        top = it if it.parent() is None else it.parent()
        stem = top.data(C_NAME, STEM_ROLE)
        r = self._ctl.tracker.recs.get(stem)
        if r is None:
            return
        m = QMenu(self)
        if r.phase not in FINISHED:
            m.addAction("Remove from queue", lambda: self._ctl.remove([stem]))
        if r.rdir:
            path = f"{self._ctl.cfg.host}:{r.rdir}/{stem}.*"
            act = QAction(f"Copy cluster path ({path})", m)
            act.triggered.connect(lambda: QGuiApplication.clipboard().setText(path))
            m.addAction(act)
        m.addAction("Copy name", lambda: QGuiApplication.clipboard().setText(stem))
        m.exec(self.tree.viewport().mapToGlobal(pos))
