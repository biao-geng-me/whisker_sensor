"""The GUI side of cluster sync: a worker thread, the Record row's status line
and the "Cluster..." dialog.

``SyncController`` owns one daemon thread that runs the jobs (test, push the
queue, plan a prune, prune) one at a time from a job queue; it reports through
Qt signals, queued to the GUI thread, so no ssh call, hash or file scan ever
runs on the GUI thread. main_window.py makes one, adds ``status_widget()`` to
the Record box, calls ``recording_stopped(path)`` after Stop and ``close()`` on
exit; ``protect`` is a callable giving the paths it must not prune or upload
(the recording being written, the one open in replay).

The flow: after Stop, with auto-upload on, the recording is queued and a push
runs. A push first tests the connection unless the last test passed; a failed
test leaves everything queued ("queued (n, offline)"). A timer retries every
``RETRY_S`` while anything is queued, and also queues recordings uploaded
before whose files changed since (a note added in replay, a card written after
Stop). With pruning on, a push that empties the queue plans a prune; the first
time, the GUI lists what would go and asks.

Status line: ``off`` / ``ok`` / ``uploading 2 of 5`` / ``queued (n, offline)`` /
``error: <reason>``; the tooltip has the details.
"""

from __future__ import annotations

import queue as _queue
import threading
import time
from pathlib import Path
from typing import Callable, Iterable, Optional

from PyQt6.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt6.QtWidgets import (QCheckBox, QDialog, QDialogButtonBox, QFormLayout, QHBoxLayout,
                             QLabel, QLineEdit, QMessageBox, QPushButton, QSpinBox,
                             QVBoxLayout, QWidget)

from wavi_py.remote import config as rconfig
from wavi_py.remote.sync import (PrunePlan, Session, SyncError, SyncState, prune_execute,
                                 prune_plan, push_queue, recording_of, requeue_changed)

RETRY_S = 300.0          # while anything is queued: test + push again
STOP_DELAY_MS = 1500     # after Stop: sidecars written right after it get in too

_STYLE = {"off": "color: #777;", "ok": "color: #2a7a2a;", "busy": "color: #1f5fa8;",
          "queued": "color: #b06000;", "error": "color: #c00;"}


class SyncController(QObject):
    status = pyqtSignal(str, str, str)          # level, text, tooltip
    note = pyqtSignal(str)                      # a one-off message for the panel label
    _plan_ready = pyqtSignal(object)            # PrunePlan, from the worker

    def __init__(self, parent: QWidget, local_dir: Path,
                 protect: Callable[[], Iterable[str]] = lambda: (),
                 state: Optional[SyncState] = None):
        super().__init__(parent)
        self._parent = parent
        self._local_dir = Path(local_dir)
        self._protect = protect
        self._state = state or SyncState()
        self.cfg = rconfig.load()
        if str(self._local_dir) != self.cfg.local_dir:
            self.cfg.local_dir = str(self._local_dir)   # the CLI's default from now on
            rconfig.save(self.cfg)
        self._jobs: _queue.Queue = _queue.Queue()
        self._sess: Optional[Session] = None
        self._sess_lock = threading.Lock()
        self._closed = False
        self.last_test: Optional[tuple[bool, str, bool]] = None   # (ok, reason, offline)
        self._label: Optional[QLabel] = None
        self.status.connect(self._show)
        self._plan_ready.connect(self._on_plan)
        self._thread = threading.Thread(target=self._loop, name="wavi-sync", daemon=True)
        self._thread.start()
        self._timer = QTimer(self)
        self._timer.setInterval(int(RETRY_S * 1000))
        self._timer.timeout.connect(self._retry)
        self._timer.start()
        self._idle_status()
        if self.cfg.auto_upload and self.cfg.configured:
            self.submit("push")                   # what was queued last session

    # ----- the GUI side

    def status_widget(self) -> QWidget:
        w = QWidget()
        h = QHBoxLayout(w)
        h.setContentsMargins(0, 0, 0, 0)
        self._label = QLabel("Cluster: off")
        self._label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        btn = QPushButton("Cluster...")
        btn.setToolTip("Cluster sync: host, folder, auto-upload, pruning, connection test")
        btn.clicked.connect(self.open_dialog)
        h.addWidget(self._label, stretch=1)
        h.addWidget(btn)
        self._last_status = getattr(self, "_last_status", ("off", "Cluster: off", ""))
        self._show(*self._last_status)
        return w

    def _show(self, level: str, text: str, tip: str) -> None:
        self._last_status = (level, text, tip)
        if self._label is not None:
            self._label.setText(text)
            self._label.setToolTip(tip or text)
            self._label.setStyleSheet(f"QLabel {{ {_STYLE.get(level, '')} }}")

    def _emit(self, level: str, text: str, tip: str = "") -> None:
        if not self._closed:
            self.status.emit(level, "Cluster: " + text, tip)

    def _idle_status(self, n_queued: Optional[int] = None) -> None:
        """The resting state, from the config and the last test (GUI thread or
        worker; reads the queue file unless ``n_queued`` is given)."""
        c = self.cfg
        if n_queued is None:
            try:
                n_queued = len(self._state.queued())
            except SyncError:
                n_queued = 0
        where = f"{c.host}:{c.root}" if c.configured else "not set up"
        if not (c.auto_upload and c.configured):
            tip = f"Auto-upload off ({where})" + (f"; {n_queued} queued" if n_queued else "")
            self._emit("off", "off" + (f" ({n_queued} queued)" if n_queued else ""), tip)
            return
        t = self.last_test
        if n_queued and t is not None and not t[0]:
            if t[2]:
                self._emit("queued", f"queued ({n_queued}, offline)", t[1])
            else:
                self._emit("error", f"error: {t[1]}" + f" ({n_queued} queued)", t[1])
        elif n_queued:
            self._emit("queued", f"queued ({n_queued})", f"{where}; retried every"
                       f" {RETRY_S / 60:.0f} min")
        else:
            self._emit("ok", "ok", f"{where}: everything uploaded"
                       + (f"\nlast test: {t[1]}" if t else ""))

    def recording_stopped(self, path) -> None:
        """After Stop (local or vehicle recording: the laptop's own files)."""
        if path is None or self._closed:
            return
        if self.cfg.auto_upload and self.cfg.configured:
            QTimer.singleShot(STOP_DELAY_MS, lambda: self.submit("enqueue", str(path)))

    def submit(self, job: str, arg=None) -> None:
        if self._closed:
            return
        protect = [str(p) for p in (self._protect() or ()) if p]
        self._jobs.put((job, arg, protect))

    def _retry(self) -> None:
        if self.cfg.auto_upload and self.cfg.configured and self._jobs.empty():
            self.submit("retry")

    def close(self) -> None:
        self._closed = True
        self._timer.stop()
        with self._sess_lock:
            if self._sess is not None:
                self._sess.cancel()
        self._jobs.put(None)

    def open_dialog(self) -> None:
        dlg = ClusterDialog(self.cfg, self._parent)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return
        old = self.cfg
        self.cfg = dlg.result_config()
        self.cfg.local_dir, self.cfg.prune_confirmed = str(self._local_dir), old.prune_confirmed
        if (self.cfg.host, self.cfg.root) != (old.host, old.root):
            self.last_test = None
        if dlg.tested is not None and (dlg.tested[0], dlg.tested[1]) == (self.cfg.host,
                                                                         self.cfg.root):
            self.last_test = dlg.tested[2]
        rconfig.save(self.cfg)
        self._idle_status()
        if self.cfg.auto_upload and self.cfg.configured:
            self.submit("push")

    def _on_plan(self, plan: PrunePlan) -> None:
        if not plan.delete or self._closed:
            return
        if not self.cfg.prune_confirmed:
            paths = plan.paths()
            mb = sum(p.stat().st_size for p in paths if p.exists()) / 1e6
            names = "\n".join(p.name for p in paths[:30])
            more = f"\n... and {len(paths) - 30} more" if len(paths) > 30 else ""
            text = (f"Pruning keeps the newest {self.cfg.keep_n} recordings in {self._local_dir}."
                    f" These {len(plan.delete)} older ones are verified on the cluster"
                    f" (size and sha256), and their local files ({mb:.1f} MB) would be"
                    f" deleted:\n\n{names}{more}\n\nDelete them now? (Asked only this once;"
                    " turn pruning off in Cluster... to stop it.)")
            if QMessageBox.question(self._parent, "Prune local recordings", text) != \
                    QMessageBox.StandardButton.Yes:
                self.cfg.prune = False
                rconfig.save(self.cfg)
                self.note.emit("Pruning turned off (Cluster... turns it on again).")
                return
            self.cfg.prune_confirmed = True
            rconfig.save(self.cfg)
        self.submit("prune", plan)

    # ----- the worker thread

    def _session(self) -> Session:
        with self._sess_lock:
            if self._sess is None or self._sess.cfg is not self.cfg:
                self._sess = Session(self.cfg, self._state)
            return self._sess

    def _loop(self) -> None:
        while True:
            item = self._jobs.get()
            if item is None or self._closed:
                return
            job, arg, protect = item
            try:
                self._do(job, arg, protect)
            except Exception as exc:  # noqa: BLE001 — the thread must live on
                self._emit("error", f"error: {exc}", repr(exc))

    def _do(self, job: str, arg, protect: list[str]) -> None:
        sess = self._session()
        if job == "enqueue":
            rec = recording_of(arg)
            if rec is not None:
                self._state.enqueue(rec, "recorded")
            job = "push"
        if job == "retry":
            requeue_changed(self._state, self._local_dir)
            if not self._state.queued():
                self._idle_status(0)
                return
            job = "push"
        if job == "push":
            self._push(sess, protect)
        elif job == "prune":
            self._prune(sess, arg, protect)

    def _push(self, sess: Session, protect: list[str]) -> None:
        if not self.cfg.configured:
            return
        n = len(self._state.queued())
        if not n:
            self._idle_status(0)
            return
        if self.last_test is None or not self.last_test[0]:
            self._emit("busy", "testing...", f"ssh {self.cfg.host}")
            self.last_test = sess.test()
            if not self.last_test[0]:
                self._idle_status(n)
                return

        def progress(stem, i, k, name):
            self._emit("busy", f"uploading {i} of {k}", f"{stem}\n{name}")
        t0, total = time.monotonic(), 0
        try:
            results, errors = push_queue(sess, protect, check_busy=False, progress=progress)
        except SyncError as exc:                   # the network went: stop, keep the queue
            self.last_test = (False, exc.reason, True)
            self._idle_status()
            return
        total = sum(r.bytes for r in results)
        left = len(self._state.queued())
        if errors:
            self._emit("error", f"error: {errors[-1]}" + (f" ({left} queued)" if left else ""),
                       "\n".join(errors))
        else:
            self._idle_status(left)
        if results and total:
            secs = time.monotonic() - t0
            self.note.emit(f"Cluster: uploaded {sum(len(r.uploaded) for r in results)} files"
                           f" ({total / 1e6:.1f} MB, {total / 1e6 / max(secs, 1e-3):.1f} MB/s)")
        if not left and self.cfg.prune:
            plan = prune_plan(sess, self._local_dir, self.cfg.keep_n, protect)
            self._plan_ready.emit(plan)

    def _prune(self, sess: Session, plan: PrunePlan, protect: list[str]) -> None:
        from wavi_py.remote import _stem
        busy = {_stem.stem_of(Path(p).name) for p in protect}
        plan = PrunePlan([r for r in plan.delete if r.stem not in busy], plan.kept_unverified)
        gone = prune_execute(sess, plan)
        if gone:
            self.note.emit(f"Cluster: pruned {len(plan.delete)} local recordings"
                           f" ({len(gone)} files), kept the newest {self.cfg.keep_n}")


class ClusterDialog(QDialog):
    """Host, root, the toggles and a Test button (its own thread)."""

    _tested_sig = pyqtSignal(str, str, object)

    def __init__(self, cfg: rconfig.SyncConfig, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Cluster sync")
        self._cfg = cfg
        self.tested: Optional[tuple[str, str, tuple]] = None
        form = QFormLayout()
        self.host_edit = QLineEdit(cfg.host)
        self.host_edit.setPlaceholderText("sporcsubmit.rc.rit.edu (an ~/.ssh/config alias)")
        self.root_edit = QLineEdit(cfg.root)
        self.auto_chk = QCheckBox("Upload each recording after Stop")
        self.auto_chk.setChecked(cfg.auto_upload)
        self.prune_chk = QCheckBox("Delete local copies beyond the newest")
        self.prune_chk.setChecked(cfg.prune)
        self.prune_chk.setToolTip("Only recordings whose every file is verified on the cluster"
                                  " (size and sha256);\nnever the one recording or in replay.")
        self.keep_spin = QSpinBox()
        self.keep_spin.setRange(1, 1000)
        self.keep_spin.setValue(cfg.keep_n)
        self.keep_spin.setSuffix(" recordings")
        ph = QHBoxLayout()
        ph.addWidget(self.prune_chk)
        ph.addWidget(self.keep_spin)
        ph.addStretch(1)
        form.addRow("Host:", self.host_edit)
        form.addRow("Folder:", self.root_edit)
        form.addRow("", self.auto_chk)
        form.addRow("", ph)
        self.test_btn = QPushButton("Test")
        self.test_btn.clicked.connect(self._test)
        self.result_label = QLabel("ssh with your own key and ~/.ssh/config; see the README"
                                   " (Cluster sync)")
        self.result_label.setWordWrap(True)
        th = QHBoxLayout()
        th.addWidget(self.test_btn)
        th.addWidget(self.result_label, stretch=1)
        bb = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok
                              | QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        v = QVBoxLayout(self)
        v.addLayout(form)
        v.addLayout(th)
        v.addWidget(bb)
        self.resize(520, self.sizeHint().height())
        self._tested_sig.connect(self._on_tested)

    def result_config(self) -> rconfig.SyncConfig:
        c = rconfig.SyncConfig(**vars(self._cfg))
        c.host = self.host_edit.text().strip()
        c.root = self.root_edit.text().strip().rstrip("/") or rconfig.DEFAULT_ROOT
        c.auto_upload = self.auto_chk.isChecked()
        c.prune = self.prune_chk.isChecked()
        c.keep_n = self.keep_spin.value()
        return c

    def _test(self) -> None:
        c = self.result_config()
        self.test_btn.setEnabled(False)
        self.result_label.setText(f"Testing ssh {c.host or '?'} ...")
        self.result_label.setStyleSheet("")

        def run():
            res = Session(c).test()
            try:
                self._tested_sig.emit(c.host, c.root, res)
            except RuntimeError:
                pass                              # the dialog was closed meanwhile
        threading.Thread(target=run, name="wavi-sync-test", daemon=True).start()

    def _on_tested(self, host: str, root: str, res: tuple) -> None:
        self.tested = (host, root, res)
        self.test_btn.setEnabled(True)
        self.result_label.setText(res[1] if res[0] else f"Failed: {res[1]}")
        self.result_label.setStyleSheet(f"QLabel {{ {_STYLE['ok' if res[0] else 'error']} }}")
