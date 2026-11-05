"""The Recordings window: local and cluster recordings, grouped, with previews.

Two tabs over one list of rows (``remote/rows.py``): **Local** (the output dir
and the download cache) and **Cluster** (the remote root, from the cached
catalogue, ``remote/catalog.py``). A tree: date -> tag -> recordings, newest
first; each row: #n in its date / tag group (by start, over every known row,
so the number is the same in both tabs and under any filter), the start time,
the duration, the card's preview strip (``replay/card.py``: gauges top to
bottom, time left to right, the inferno map as the cards' contact sheet), a
BAD badge (the reason in its tooltip), the comment count, and where it lives
(local / cluster / both, queued). A laptop recording and the Pi's file of the
same run are one row.

Selecting a row shows the card's numbers and the notes. A recording with a
local copy takes a new note here (an append, ``replay/notes.py``); a
cluster-only one's notes are read-only until it is opened (downloaded). Open
replays a local copy at once; a cluster-only one is downloaded into the cache
first (verified, with progress and Cancel; ``remote/cache.py``), then opened.
Make card builds a missing card of a local recording in the background (the
same child process as after Stop); a cluster-only recording without a card
says in its tooltip how to make it there.

Refresh lists the cluster in one ssh call (only the date folders whose mtime
changed are walked again); the label shows the catalogue's age, or the reason
the listing failed (the Local tab keeps working). Nothing here runs ssh, a
download or a folder scan of the cluster on the GUI thread.
"""

from __future__ import annotations

import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

import numpy as np
from PyQt6.QtCore import QSize, Qt, pyqtSignal
from PyQt6.QtGui import QColor, QFont, QImage, QPainter, QPixmap
from PyQt6.QtWidgets import (QCheckBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit,
                             QMessageBox, QProgressDialog, QPushButton, QSplitter, QTabWidget,
                             QTextBrowser, QTreeWidget, QTreeWidgetItem, QVBoxLayout, QWidget)

from wavi_py.remote import config as rconfig
from wavi_py.remote import rows as rrows
from wavi_py.remote.cache import Cache, download
from wavi_py.remote.catalog import Catalog
from wavi_py.remote.sync import Session, SyncError, SyncState
from wavi_py.replay import card as cardmod
from wavi_py.replay import notes as rnotes
from wavi_py.views.notes_panel import ask_note, fmt_note_t

PREVIEW_W, PREVIEW_H = 256, 32            # one pixel per gauge row at 32 gauges
COLS = ["#", "start", "duration", "preview", "", "notes", "where"]
C_N, C_START, C_DUR, C_PREV, C_BAD, C_NOTES, C_WHERE = range(len(COLS))
ROLE = Qt.ItemDataRole.UserRole

_LUT: Optional[np.ndarray] = None


def _lut() -> np.ndarray:
    global _LUT
    if _LUT is None:
        import pyqtgraph as pg
        _LUT = pg.colormap.get("inferno").getLookupTable(0, 1, 256).astype(np.uint8)[:, :3]
    return _LUT


def preview_pixmap(card: Optional[dict], w: int = PREVIEW_W, h: int = PREVIEW_H) -> QPixmap:
    """The card's preview as a small heatmap (gauge rows, time columns), or a
    grey "no card" placeholder."""
    pm = QPixmap(w, h)
    if card is None or not card.get("preview"):
        pm.fill(QColor(225, 225, 225))
        p = QPainter(pm)
        p.setPen(QColor(120, 120, 120))
        f = QFont()
        f.setPointSize(8)
        p.setFont(f)
        p.drawText(pm.rect(), Qt.AlignmentFlag.AlignCenter, "no card")
        p.end()
        return pm
    q = cardmod.decode_preview(card)                     # nch x nbins uint8
    rgb = _lut()[q]
    a = np.empty(q.shape + (4,), np.uint8)
    a[..., 0], a[..., 1], a[..., 2], a[..., 3] = rgb[..., 2], rgb[..., 1], rgb[..., 0], 255
    a = np.ascontiguousarray(a)
    # copy: the QImage only borrows ``a``'s buffer, which is gone once this returns
    img = QImage(a.data, a.shape[1], a.shape[0], a.shape[1] * 4,
                 QImage.Format.Format_RGB32).copy()
    return QPixmap.fromImage(img.scaled(w, h, Qt.AspectRatioMode.IgnoreAspectRatio,
                                        Qt.TransformationMode.FastTransformation))


def _fmt_dur(s: Optional[float]) -> str:
    if s is None:
        return "–"
    m, sec = divmod(int(round(s)), 60)
    return f"{m}:{sec:02d}"


def _age(s: Optional[float]) -> str:
    if s is None:
        return "never"
    if s < 90:
        return f"{s:.0f} s ago"
    if s < 5400:
        return f"{s / 60:.0f} min ago"
    if s < 172800:
        return f"{s / 3600:.0f} h ago"
    return f"{s / 86400:.0f} days ago"


class RecordingsBrowser(QWidget):
    _listed = pyqtSignal(object)               # (walked, error) from the listing thread
    _dl_progress = pyqtSignal(int, int, str)
    _dl_done = pyqtSignal(object, object, float)   # path or None, error or None, secs
    _card_done = pyqtSignal(str, object)

    def __init__(self, outdir: Path, open_fn: Callable[[str], None],
                 protect: Callable[[], list] = lambda: [], who: str = "",
                 state: Optional[SyncState] = None, catalog: Optional[Catalog] = None,
                 cache: Optional[Cache] = None, extra_dirs: tuple = (),
                 parent: Optional[QWidget] = None):
        super().__init__(parent, Qt.WindowType.Window)
        self.setWindowTitle("Wavi - Recordings")
        self.resize(1000, 600)
        self._outdir = Path(outdir)
        self._extra = [Path(d) for d in extra_dirs]     # more local folders to list
        self._open_fn, self._protect, self._who = open_fn, protect, who
        self._state = state or SyncState()
        self.catalog = catalog or Catalog()
        self.cache = cache or Cache()
        self.rows: list[rrows.Row] = []
        self._listing = False
        self._dl_sess: Optional[Session] = None
        self._dlg: Optional[QProgressDialog] = None
        self.last_download: Optional[tuple[str, float, int]] = None   # stem, secs, bytes

        top = QHBoxLayout()
        self.filter_edit = QLineEdit()
        self.filter_edit.setPlaceholderText("filter: tag or note text")
        self.filter_edit.setClearButtonEnabled(True)
        self.filter_edit.textChanged.connect(self._fill)
        self.hide_bad = QCheckBox("Hide bad")
        self.hide_bad.toggled.connect(self._fill)
        self.status = QLabel()
        self.status.setStyleSheet("QLabel { color: #555; }")
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.setToolTip("Rescan the local folders and list the cluster (one ssh call;"
                                    " only changed date folders are fetched)")
        self.refresh_btn.clicked.connect(self.refresh)
        top.addWidget(self.filter_edit, stretch=1)
        top.addWidget(self.hide_bad)
        top.addWidget(self.status, stretch=1)
        top.addWidget(self.refresh_btn)

        self.tabs = QTabWidget()
        self.local_tree, self.cluster_tree = self._make_tree(), self._make_tree()
        self.tabs.addTab(self.local_tree, "Local")
        self.tabs.addTab(self.cluster_tree, "Cluster")
        self.tabs.currentChanged.connect(lambda _i: self._show_details())

        # details: the card's numbers, the notes
        det = QWidget()
        dv = QVBoxLayout(det)
        dv.setContentsMargins(0, 0, 0, 0)
        # a text browser, not a wrapped QLabel: that keeps its first height and clips
        self.info = QTextBrowser()
        self.info.setOpenLinks(False)
        self.info.setMaximumHeight(190)
        self.info.setText("Select a recording.")
        self.notes_tree = QTreeWidget()
        self.notes_tree.setHeaderLabels(["t", "note", "by", "when"])
        self.notes_tree.setRootIsDecorated(False)
        self.notes_tree.header().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        self.notes_hint = QLabel()
        self.notes_hint.setStyleSheet("QLabel { color: #777; }")
        self.add_note_btn = QPushButton("Add note...")
        self.add_note_btn.clicked.connect(self._add_note)
        dv.addWidget(self.info)
        dv.addWidget(QLabel("Notes:"))
        dv.addWidget(self.notes_tree, stretch=1)
        dv.addWidget(self.notes_hint)
        hb = QHBoxLayout()
        hb.addWidget(self.add_note_btn)
        hb.addStretch(1)
        dv.addLayout(hb)

        split = QSplitter(Qt.Orientation.Horizontal)
        split.addWidget(self.tabs)
        split.addWidget(det)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 1)
        split.setSizes([700, 300])

        bottom = QHBoxLayout()
        self.open_btn = QPushButton("Open")
        self.open_btn.clicked.connect(self.open_selected)
        self.card_btn = QPushButton("Make card")
        self.card_btn.setToolTip("Build the missing card of a local recording (in the background)")
        self.card_btn.clicked.connect(self._make_card)
        self.cache_label = QLabel()
        self.cache_label.setStyleSheet("QLabel { color: #777; }")
        bottom.addWidget(self.open_btn)
        bottom.addWidget(self.card_btn)
        bottom.addStretch(1)
        bottom.addWidget(self.cache_label)

        v = QVBoxLayout(self)
        v.addLayout(top)
        v.addWidget(split, stretch=1)
        v.addLayout(bottom)

        self._listed.connect(self._on_listed)
        self._dl_progress.connect(self._on_dl_progress)
        self._dl_done.connect(self._on_dl_done)
        self._card_done.connect(lambda stem, err: (self.rebuild(),
                                                   self._say(f"card: {stem}: {err}" if err else
                                                             f"card written: {stem}")))
        self.rebuild()

    # ----- building

    def _make_tree(self) -> QTreeWidget:
        t = QTreeWidget()
        t.setHeaderLabels(COLS)
        t.setIconSize(QSize(PREVIEW_W, PREVIEW_H))
        t.setUniformRowHeights(False)
        t.setAlternatingRowColors(True)
        h = t.header()
        h.setStretchLastSection(False)
        for c in (C_N, C_START, C_DUR, C_BAD, C_NOTES, C_WHERE):
            h.setSectionResizeMode(c, QHeaderView.ResizeMode.ResizeToContents)
        h.setSectionResizeMode(C_PREV, QHeaderView.ResizeMode.Fixed)
        t.setColumnWidth(C_PREV, PREVIEW_W + 12)
        t.itemSelectionChanged.connect(self._show_details)
        t.itemDoubleClicked.connect(lambda it, _c: self.open_selected() if it.data(0, ROLE) else None)
        return t

    def local_dirs(self) -> list[Path]:
        return [self._outdir, *self._extra, self.cache.dir]

    def rebuild(self) -> None:
        """Rows from the local folders and the cached catalogue (no network)."""
        try:
            queued = list(self._state.queued())
        except SyncError:
            queued = []
        local = rrows.local_rows(self.local_dirs(), queued)
        cfg = rconfig.load()
        cl = (rrows.cluster_rows(self.catalog.folders)
              if self.catalog.matches(cfg.host.strip(), cfg.root) else {})
        self.rows = rrows.build_rows(local, cl)
        self._fill()
        self._update_status()

    def _fill(self) -> None:
        sel = self.selected()
        keep = sel.stem if sel else None
        text, hide = self.filter_edit.text(), self.hide_bad.isChecked()
        for tree, pick in ((self.local_tree, lambda r: r.local is not None),
                           (self.cluster_tree, lambda r: r.cluster is not None)):
            tree.clear()
            shown = [r for r in self.rows if pick(r) and rrows.matches(r, text, hide)]
            for date, tags in rrows.grouped(shown):
                n = sum(len(rs) for _, rs in tags)
                di = QTreeWidgetItem([f"{date}   ({n})"])
                f = di.font(0)
                f.setBold(True)
                di.setFont(0, f)
                tree.addTopLevelItem(di)
                di.setFirstColumnSpanned(True)
                for tag, rs in tags:
                    ti = QTreeWidgetItem([f"{tag or '(no tag)'}   ({len(rs)})"])
                    di.addChild(ti)
                    ti.setFirstColumnSpanned(True)
                    for r in rs:
                        it = self._row_item(r)
                        ti.addChild(it)
                        if r.stem == keep:
                            tree.setCurrentItem(it)
            tree.expandAll()
            if tree is self.cluster_tree and not shown:
                why = self._cluster_empty_reason()
                if why:
                    it = QTreeWidgetItem([why])
                    it.setFirstColumnSpanned(True)
                    tree.addTopLevelItem(it)
                    it.setFirstColumnSpanned(True)
        self._show_details()

    def _cluster_empty_reason(self) -> str:
        cfg = rconfig.load()
        if not cfg.configured:
            return "Cluster sync is not set up: Cluster... in the control panel's Record row."
        if self.catalog.data.get("error"):
            return f"Cluster not reachable: {self.catalog.data['error']}"
        if self.catalog.age_s() is None or not self.catalog.matches(cfg.host.strip(), cfg.root):
            return "Not listed yet: Refresh."
        if self.catalog.data.get("noroot"):
            return f"{cfg.root} does not exist on {cfg.host} yet (no upload so far)."
        return "No recordings" + (" match the filter." if self.filter_edit.text()
                                  or self.hide_bad.isChecked() else " on the cluster.")

    def _row_item(self, r: rrows.Row) -> QTreeWidgetItem:
        st = r.state
        it = QTreeWidgetItem([f"#{r.n}", f"{r.start:%H:%M:%S}" if r.start else "?",
                              _fmt_dur(r.duration), "", "BAD" if st.bad else "",
                              str(st.count) if st.count else "", r.where])
        it.setData(0, ROLE, r.stem)
        it.setIcon(C_PREV, _icon(preview_pixmap(r.card)))
        it.setSizeHint(C_PREV, QSize(PREVIEW_W, PREVIEW_H + 4))
        tip = " / ".join(r.stems)
        if r.card is None:
            if r.local is not None:
                tip += "\nno card: Make card builds it"
            else:
                d = (r.cluster or {}).get("date", "")
                tip += ("\nno card; on the cluster, from a wavi_py checkout:\n"
                        f"  ~/bin/onnode -- python -m wavi_py.tools.make_cards"
                        f" {rconfig.load().root}/{d}")
        it.setToolTip(C_PREV, tip)
        it.setToolTip(C_N, tip)
        if st.bad:
            it.setForeground(C_BAD, QColor("#c00"))
            f = it.font(C_BAD)
            f.setBold(True)
            it.setFont(C_BAD, f)
            it.setToolTip(C_BAD, f"marked bad{': ' + st.reason if st.reason else ''}")
        if st.count:
            it.setToolTip(C_NOTES, "\n".join(f"{fmt_note_t(n.get('t'))}  {n.get('text', '')}"
                                             for n in st.notes))
        return it

    # ----- selection and details

    def _tree(self) -> QTreeWidget:
        return self.local_tree if self.tabs.currentIndex() == 0 else self.cluster_tree

    def selected(self) -> Optional[rrows.Row]:
        it = self._tree().currentItem() if hasattr(self, "local_tree") else None
        stem = it.data(0, ROLE) if it is not None else None
        return next((r for r in self.rows if r.stem == stem), None) if stem else None

    def select(self, stem: str) -> bool:
        tree = self._tree()
        for it in _walk(tree):
            if it.data(0, ROLE) == stem:
                tree.setCurrentItem(it)
                return True
        return False

    def _show_details(self) -> None:
        r = self.selected()
        self.notes_tree.clear()
        self.open_btn.setEnabled(r is not None and not self._busy())
        self.card_btn.setEnabled(r is not None and r.local is not None and r.card is None)
        self.add_note_btn.setEnabled(r is not None and r.local is not None)
        if r is None:
            self.info.setText("Select a recording.")
            self.notes_hint.setText("")
            return
        c = r.card or {}
        lines = [f"<b>{' / '.join(r.stems)}</b>",
                 f"{r.start:%Y-%m-%d %H:%M:%S}" if r.start else "",
                 f"tag <b>{r.tag or '(none)'}</b>, #{r.n} of the day's tag group; {r.where}"]
        if c:
            f = c.get("faults")
            lines.append(f"{_fmt_dur(c.get('duration_s'))} ({c.get('duration_s', 0):.1f} s),"
                         f" {c.get('fs_hz')} Hz, {c.get('nch')} ch, {c.get('n_scans')} scans,"
                         f" {c.get('n_imu')} IMU rows, {c.get('spikes')} spikes")
            lines.append("faults: " + ("none recorded (.dat)" if f is None else
                                       f"{f.get('warn', 0)} warn, {f.get('fault', 0)} fault"))
            if c.get("depth_m"):
                lo, hi = c["depth_m"]
                lines.append(f"depth {lo:.2f} .. {hi:.2f} m")
            if c.get("armed_s") is not None:
                lines.append(f"armed {c['armed_s']:.0f} s")
        else:
            lines.append("<i>no card</i>")
        if r.cluster:
            mb = r.size / 1e6
            lines.append(f"on the cluster: {r.cluster['date']}/, {mb:.1f} MB")
        if r.local:
            lines.append(f"local: {r.local}")
        st = r.state
        if st.bad:
            lines.append(f"<b style='color:#c00'>marked bad</b>"
                         + (f": {st.reason}" if st.reason else ""))
        self.info.setText("<br>".join(x for x in lines if x))
        for n in st.notes:
            when = datetime.fromtimestamp(float(n.get("unix") or 0)).strftime("%Y-%m-%d %H:%M")
            it = QTreeWidgetItem([fmt_note_t(n.get("t")), str(n.get("text", "")),
                                  str(n.get("by", "")), when])
            it.setToolTip(1, str(n.get("text", "")))
            self.notes_tree.addTopLevelItem(it)
        for col in (0, 2, 3):
            self.notes_tree.resizeColumnToContents(col)
        self.notes_hint.setText("" if r.local is not None else
                                "Read-only (on the cluster only): Open it to add notes.")
        self.notes_hint.setToolTip("" if r.local is not None else
                                   "Opening downloads it into the cache; notes added then go"
                                   " up with the sync.")

    def _add_note(self) -> None:
        r = self.selected()
        if r is None or r.local is None:
            return
        got = ask_note(self, f"Note on {r.stem}", None)
        if got is None:
            return
        try:
            rnotes.append(rnotes.notes_path(r.local), rnotes.note(got[0], got[1], by=self._who))
        except OSError as exc:
            QMessageBox.critical(self, "Note not saved", str(exc))
            return
        self.rebuild()

    # ----- the cluster listing

    def refresh(self) -> None:
        self.rebuild()
        cfg = rconfig.load()
        if not cfg.configured or self._listing:
            return
        self._listing = True
        self.refresh_btn.setEnabled(False)
        self._say(f"listing {cfg.host}:{cfg.root} ...")

        def run():
            try:
                walked = self.catalog.refresh(Session(cfg, self._state))
                res = (walked, None)
            except SyncError as exc:
                res = (0, exc.reason)
            except Exception as exc:  # noqa: BLE001 — shown, never raised in a thread
                res = (0, f"{type(exc).__name__}: {exc}")
            try:
                self._listed.emit(res)
            except RuntimeError:
                pass
        threading.Thread(target=run, name="wavi-catalog", daemon=True).start()

    def _on_listed(self, res) -> None:
        self._listing = False
        self.refresh_btn.setEnabled(True)
        self.rebuild()
        if res[1]:
            self._say(f"cluster: {res[1]}" + (f" (catalogue from {_age(self.catalog.age_s())})"
                                              if self.catalog.age_s() is not None else ""))

    def _say(self, text: str) -> None:
        self.status.setText(text)
        self.status.setToolTip(text)

    def _update_status(self) -> None:
        if self._listing:
            return
        n_l = sum(1 for r in self.rows if r.local)
        n_c = sum(1 for r in self.rows if r.cluster)
        cfg = rconfig.load()
        if not cfg.configured:
            cl = "cluster: not set up"
        elif self.catalog.data.get("error"):
            cl = f"cluster: {self.catalog.data['error']}"
        else:
            cl = f"cluster: {n_c}, listed {_age(self.catalog.age_s())}"
        self._say(f"local: {n_l}   {cl}")
        try:
            self.cache_label.setText(f"download cache {self.cache.size() / 1e9:.2f} of"
                                     f" {self.cache.cap / 1e9:.0f} GB")
        except OSError:
            pass

    # ----- open, download, make card

    def _busy(self) -> bool:
        return self._dl_sess is not None

    def open_selected(self) -> None:
        r = self.selected()
        if r is None or self._busy():
            return
        if r.local is not None:
            if r.local.parent == self.cache.dir:
                self.cache.touch(r.local_stem)
            self._open_fn(str(r.local))
            return
        cfg = rconfig.load()
        if not cfg.configured:
            QMessageBox.information(self, "Open", "Cluster sync is not set up (Cluster...).")
            return
        self._download(r, cfg)

    def _download(self, r: rrows.Row, cfg: rconfig.SyncConfig) -> None:
        files = dict(r.cluster["files"])
        total = sum(int(v[0]) for v in files.values())
        self._dl_sess = sess = Session(cfg, self._state)
        dlg = QProgressDialog(f"Downloading {r.stem} ({total / 1e6:.1f} MB)...", "Cancel",
                              0, 1000, self)
        dlg.setWindowTitle("Download")
        dlg.setWindowModality(Qt.WindowModality.WindowModal)
        dlg.setMinimumDuration(0)
        dlg.canceled.connect(sess.cancel)
        dlg.setValue(0)
        self._dlg = dlg
        self._show_details()
        t0 = time.monotonic()
        date = r.cluster["date"]

        def run():
            path, err = None, None
            try:
                path = download(sess, self.cache, date, files,
                                progress=lambda d, tot, name: self._dl_progress.emit(d, tot, name))
            except SyncError as exc:
                err = exc.reason
            except OSError as exc:
                err = str(exc)
            try:
                self._dl_done.emit(path, err, time.monotonic() - t0)
            except RuntimeError:
                pass
        threading.Thread(target=run, name="wavi-download", daemon=True).start()

    def _on_dl_progress(self, done: int, total: int, name: str) -> None:
        dlg = self._dlg
        if dlg is not None:
            # the label first: setValue of a modal progress dialog runs the event loop,
            # where the download may finish and drop the dialog
            dlg.setLabelText(f"Downloading {name}\n{done / 1e6:.1f} of {total / 1e6:.1f} MB")
            dlg.setValue(int(1000 * done / max(total, 1)))

    def _on_dl_done(self, path, err, secs: float) -> None:
        self._dl_sess = None
        if self._dlg is not None:
            self._dlg.reset()
            self._dlg.deleteLater()
            self._dlg = None
        if err is not None:
            if err != "cancelled":
                QMessageBox.critical(self, "Download failed", err)
            self._say(f"download {err}")
            self.rebuild()
            return
        path = Path(path)
        size = sum(p.stat().st_size for p in path.parent.glob(path.name.rsplit(".", 1)[0] + ".*")
                   if p.is_file())
        self.last_download = (path.name, secs, size)
        gone = self.cache.evict(self._state, protect=[str(path), *self._protect()])
        self.rebuild()
        self._say(f"downloaded {path.name} in {secs:.1f} s"
                  + (f"; evicted {len(gone)} older from the cache" if gone else ""))
        self._open_fn(str(path))

    def _make_card(self) -> None:
        r = self.selected()
        if r is None or r.local is None:
            return
        self.card_btn.setEnabled(False)
        self._say(f"making the card of {r.stem} ...")
        stem = r.stem
        cardmod.write_card_in_background(
            r.local, on_done=lambda p, err: self._emit_card(stem, err))

    def _emit_card(self, stem: str, err) -> None:
        try:
            self._card_done.emit(stem, err)
        except RuntimeError:
            pass

    def showEvent(self, event) -> None:  # noqa: N802 — Qt naming
        super().showEvent(event)
        self.rebuild()
        age = self.catalog.age_s()
        if rconfig.load().configured and (age is None or age > 60):
            self.refresh()


def _icon(pm: QPixmap):
    from PyQt6.QtGui import QIcon
    icon = QIcon()
    icon.addPixmap(pm, QIcon.Mode.Normal)
    icon.addPixmap(pm, QIcon.Mode.Selected)            # no blue tint over the colours
    return icon


def _walk(tree: QTreeWidget):
    stack = [tree.topLevelItem(i) for i in range(tree.topLevelItemCount())]
    while stack:
        it = stack.pop()
        yield it
        stack.extend(it.child(i) for i in range(it.childCount()))
