"""The Analysis window's side panel in Events mode: scope, filter, grouping, saved
comparisons (the Variables tree stays below it, the window's own).

- Scope: the recordings in the set (folders are expanded to their ``*.jsonl``),
  with a count; Add recordings... (a file dialog: recordings, or a folder with
  Add folder...) and Remove.
- Filter: the ``replay/eventset.EventQuery`` fields in compact controls: type,
  axis and sign (the class), |u| range, mode, minimum hold, clean baseline,
  steady, minor inputs (any / none / at most), depth range, alive (any lost
  excluded / every position known alive), whisker set. A change re-runs the
  query after ``DEBOUNCE_MS`` (sidecars only: ~0.25 s warm, off the GUI
  thread; the stores are not rebuilt).
- Group by: run, day, |u| bin (edges in the box beside it), mode, whisker set,
  none.
- Comparisons: the saved queries (``eventset.list_comparisons``, the user
  config dir); picking one loads it (scope, filter, grouping, curation); Save
  as..., Delete, Revert (back to the saved one).

``queryChanged(EventQuery)`` carries the whole query: the controls plus the scope
and the curation (excludes / pins, from the browser) the panel keeps.
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Optional

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout,
                             QGroupBox, QHBoxLayout, QInputDialog, QLabel, QLineEdit,
                             QListWidget, QListWidgetItem, QMessageBox, QPushButton, QVBoxLayout,
                             QWidget)

from wavi_py.replay import eventset as evs
from wavi_py.replay import recname
from wavi_py.replay.events import ARDUSUB_MODES

DEBOUNCE_MS = 250
TYPES = ("any", "onset", "release", "step", "reversal", "hold")
AXES = ("any", "surge", "sway", "heave", "yaw", "combo", "rest")
SIGNS = ("any", "+", "-")
WHISKER_SETS = ("any", "unknown", "3x", "2x", "3x-cut-120", "3x-cut-72")
GROUPS = (("run", "run"), ("day", "day"), ("|u| bin", "level"), ("mode", "mode"),
          ("whisker set", "whisker_set"), ("none", "none"))
_NONE = -1e9                                     # a spin box at its minimum: no limit


def _spin(lo, hi, step, decimals, special="") -> QDoubleSpinBox:
    b = QDoubleSpinBox()
    b.setRange(lo, hi)
    b.setSingleStep(step)
    b.setDecimals(decimals)
    b.setKeyboardTracking(False)
    if special:
        b.setSpecialValueText(special)
    b.setValue(lo)
    return b


def classes_for(axis: str, sign: str) -> list:
    """The ``EventQuery.classes`` of an axis / sign choice."""
    if axis == "any":
        return []
    if axis in ("combo", "rest"):
        return [axis]
    return [f"single:{axis}{s}" for s in ("+", "-") if sign in ("any", s)]


def axis_sign_of(classes) -> tuple[str, str]:
    """The axis / sign controls for ``classes`` (the reverse of ``classes_for``)."""
    classes = list(classes or [])
    if not classes:
        return "any", "any"
    if classes in (["combo"], ["rest"]):
        return classes[0], "any"
    axes = {c.split(":", 1)[-1][:-1] for c in classes}
    signs = {c[-1] for c in classes}
    if len(axes) == 1:
        return axes.pop(), (signs.pop() if len(signs) == 1 else "any")
    return "any", "any"


class EventPanel(QWidget):
    """See the module doc."""

    queryChanged = pyqtSignal(object)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._query = evs.EventQuery(types=["onset"], group_by="run")
        self._saved_name = ""
        self._loading = False
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(4)

        # comparisons
        cmp_row = QHBoxLayout()
        self.cmp_box = QComboBox()
        self.cmp_box.setToolTip("Saved comparisons (a query with its scope, grouping and curation)")
        self.cmp_box.activated.connect(self._on_cmp_picked)
        cmp_row.addWidget(QLabel("<b>Comparison</b>"))
        cmp_row.addWidget(self.cmp_box, 1)
        v.addLayout(cmp_row)
        btns = QHBoxLayout()
        self.save_btn = QPushButton("Save as...")
        self.save_btn.clicked.connect(self.save_as)
        self.del_btn = QPushButton("Delete")
        self.del_btn.clicked.connect(self._delete_cmp)
        self.revert_btn = QPushButton("Revert")
        self.revert_btn.setToolTip("Back to the saved comparison")
        self.revert_btn.clicked.connect(self._revert)
        for b in (self.save_btn, self.del_btn, self.revert_btn):
            btns.addWidget(b)
        v.addLayout(btns)

        # scope
        scope = QGroupBox("Scope")
        sl = QVBoxLayout(scope)
        sl.setContentsMargins(4, 4, 4, 4)
        self.scope_label = QLabel()
        sl.addWidget(self.scope_label)
        self.scope_list = QListWidget()
        self.scope_list.setMaximumHeight(90)
        self.scope_list.setSelectionMode(QListWidget.SelectionMode.ExtendedSelection)
        sl.addWidget(self.scope_list)
        sb = QHBoxLayout()
        self.add_btn = QPushButton("Add recordings...")
        self.add_btn.clicked.connect(self._add_recordings)
        self.add_dir_btn = QPushButton("Add folder...")
        self.add_dir_btn.clicked.connect(self._add_folder)
        self.rm_btn = QPushButton("Remove")
        self.rm_btn.clicked.connect(self._remove_selected)
        for b in (self.add_btn, self.add_dir_btn, self.rm_btn):
            sb.addWidget(b)
        sl.addLayout(sb)
        v.addWidget(scope)

        # filter
        filt = QGroupBox("Filter")
        f = QFormLayout(filt)
        f.setContentsMargins(4, 4, 4, 4)
        f.setVerticalSpacing(2)
        self.type_box = QComboBox()
        self.type_box.addItems(TYPES)
        f.addRow("type", self.type_box)
        cls = QHBoxLayout()
        self.axis_box = QComboBox()
        self.axis_box.addItems(AXES)
        self.sign_box = QComboBox()
        self.sign_box.addItems(SIGNS)
        cls.addWidget(self.axis_box, 1)
        cls.addWidget(self.sign_box)
        f.addRow("class", cls)
        lv = QHBoxLayout()
        self.level_lo = _spin(0.0, 1.0, 0.05, 2)
        self.level_hi = _spin(0.0, 1.0, 0.05, 2)
        self.level_hi.setValue(1.0)
        lv.addWidget(self.level_lo)
        lv.addWidget(QLabel(".."))
        lv.addWidget(self.level_hi)
        f.addRow("|u|", lv)
        self.mode_box = QComboBox()
        self.mode_box.addItems(["any", *sorted(set(ARDUSUB_MODES.values()))])
        f.addRow("mode", self.mode_box)
        self.hold_box = _spin(0.0, 60.0, 0.5, 1, special="any")
        self.hold_box.setSuffix(" s")
        self.hold_box.setToolTip("Minimum hold: a hold's length, a transition's time to the next"
                                 " stick change")
        f.addRow("min hold", self.hold_box)
        flags = QHBoxLayout()
        self.clean_box = QCheckBox("clean baseline")
        self.clean_box.setToolTip("A quiet rest of at least 2 s before the event")
        self.steady_box = QCheckBox("steady")
        self.steady_box.setToolTip("The hold lasts long enough for a steady value (3 s)")
        flags.addWidget(self.clean_box)
        flags.addWidget(self.steady_box)
        f.addRow("", flags)
        mn_row = QHBoxLayout()
        self.minor_box = QComboBox()
        self.minor_box.addItems(["any", "none", "at most"])
        self.minor_box.setToolTip("Minor inputs on other axes (|u| < 0.15 next to the event's axis)")
        self.minor_max = _spin(0.0, 0.15, 0.01, 2)
        self.minor_max.setValue(0.05)
        mn_row.addWidget(self.minor_box, 1)
        mn_row.addWidget(self.minor_max)
        f.addRow("minor", mn_row)
        dp = QHBoxLayout()
        self.depth_lo = _spin(-1.0, 5000.0, 10.0, 0, special="-")
        self.depth_hi = _spin(-1.0, 5000.0, 10.0, 0, special="-")
        for b in (self.depth_lo, self.depth_hi):
            b.setSuffix(" cm")
            b.setToolTip("Depth range; a limit drops runs whose depth is not trusted (armed"
                         " under water) and events without depth")
        dp.addWidget(self.depth_lo)
        dp.addWidget(QLabel(".."))
        dp.addWidget(self.depth_hi)
        f.addRow("depth", dp)
        self.alive_box = QComboBox()
        self.alive_box.addItems(["any", "no position lost", "all known alive"])
        self.alive_box.setToolTip("no position lost: undetermined (rest, no thrust) passes; all"
                                  " known alive: only events where every position is alive")
        f.addRow("alive", self.alive_box)
        self.wset_box = QComboBox()
        self.wset_box.addItems(WHISKER_SETS)
        f.addRow("whiskers", self.wset_box)
        gb = QHBoxLayout()
        self.group_box = QComboBox()
        for text, val in GROUPS:
            self.group_box.addItem(text, val)
        self.edges_edit = QLineEdit(", ".join(f"{e:g}" for e in evs.LEVEL_EDGES))
        self.edges_edit.setToolTip("|u| bin edges")
        gb.addWidget(self.group_box, 1)
        gb.addWidget(self.edges_edit, 1)
        f.addRow("group by", gb)
        v.addWidget(filt)
        self.note = ""                       # kept under the status (e.g. skipped recordings)
        self.status = QLabel("")
        self.status.setWordWrap(True)
        self.status.setStyleSheet("color: #555;")
        v.addWidget(self.status)

        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.setInterval(DEBOUNCE_MS)
        self._timer.timeout.connect(self._emit)
        for w in (self.type_box, self.axis_box, self.sign_box, self.mode_box, self.minor_box,
                  self.alive_box, self.wset_box, self.group_box):
            w.currentIndexChanged.connect(self._edited)
        for w in (self.level_lo, self.level_hi, self.hold_box, self.minor_max, self.depth_lo,
                  self.depth_hi):
            w.valueChanged.connect(self._edited)
        for w in (self.clean_box, self.steady_box):
            w.toggled.connect(self._edited)
        self.edges_edit.editingFinished.connect(self._edited)
        self.refresh_comparisons()
        self.set_query(self._query, emit=False)

    # ----- the query

    @property
    def query(self) -> evs.EventQuery:
        return copy.deepcopy(self._query)

    def set_query(self, q: evs.EventQuery, emit: bool = True) -> None:
        """The controls (and scope, curation) from ``q``."""
        self._loading = True
        self._query = copy.deepcopy(q)
        self.type_box.setCurrentText(q.types[0] if len(q.types) == 1 else "any")
        ax, sg = axis_sign_of(q.classes)
        self.axis_box.setCurrentText(ax)
        self.sign_box.setCurrentText(sg)
        self.level_lo.setValue(q.level_min or 0.0)
        self.level_hi.setValue(1.0 if q.level_max is None else q.level_max)
        self.mode_box.setCurrentText(q.modes[0] if len(q.modes) == 1 else "any")
        self.hold_box.setValue(q.hold_min_s or 0.0)
        self.clean_box.setChecked(bool(q.baseline_clean))
        self.steady_box.setChecked(bool(q.steady))
        if q.minor is None or q.minor is True:
            self.minor_box.setCurrentText("any")
        elif q.minor is False:
            self.minor_box.setCurrentText("none")
        else:
            self.minor_box.setCurrentText("at most")
            self.minor_max.setValue(float(q.minor))
        self.depth_lo.setValue(-1.0 if q.depth_min_cm is None else q.depth_min_cm)
        self.depth_hi.setValue(-1.0 if q.depth_max_cm is None else q.depth_max_cm)
        self.alive_box.setCurrentIndex(2 if q.alive_known else 1 if q.alive else 0)
        self.wset_box.setCurrentText(q.whisker_sets[0] if len(q.whisker_sets) == 1 else "any")
        i = self.group_box.findData(q.group_by)
        self.group_box.setCurrentIndex(max(0, i))
        self.edges_edit.setText(", ".join(f"{e:g}" for e in q.level_edges))
        self._fill_scope()
        self._enable()
        self._loading = False
        if emit:
            self._emit()

    def _from_controls(self) -> evs.EventQuery:
        q = copy.deepcopy(self._query)
        t = self.type_box.currentText()
        q.types = [] if t == "any" else [t]
        q.kinds = []
        q.classes = classes_for(self.axis_box.currentText(), self.sign_box.currentText())
        lo, hi = self.level_lo.value(), self.level_hi.value()
        q.level_min = lo if lo > 0 else None
        q.level_max = hi if hi < 1.0 else None
        m = self.mode_box.currentText()
        q.modes = [] if m == "any" else [m]
        q.hold_min_s = self.hold_box.value() or None
        q.baseline_clean = self.clean_box.isChecked()
        q.steady = self.steady_box.isChecked()
        q.minor = {"any": None, "none": False}.get(self.minor_box.currentText(),
                                                    self.minor_max.value())
        q.depth_min_cm = None if self.depth_lo.value() < 0 else self.depth_lo.value()
        q.depth_max_cm = None if self.depth_hi.value() < 0 else self.depth_hi.value()
        a = self.alive_box.currentIndex()
        q.alive, q.alive_known = ("all" if a else None), a == 2
        w = self.wset_box.currentText()
        q.whisker_sets = [] if w == "any" else [w]
        q.group_by = self.group_box.currentData()
        try:
            edges = sorted(float(x) for x in self.edges_edit.text().replace(";", ",").split(",")
                           if x.strip())
            if len(edges) >= 2:
                q.level_edges = edges
        except ValueError:
            pass
        return q

    def _enable(self) -> None:
        self.minor_max.setEnabled(self.minor_box.currentText() == "at most")
        self.edges_edit.setVisible(self.group_box.currentData() == "level")
        self.sign_box.setEnabled(self.axis_box.currentText() not in ("any", "combo", "rest"))
        has = bool(self._saved_name)
        self.del_btn.setEnabled(has)
        self.revert_btn.setEnabled(has)

    def _edited(self, *_a) -> None:
        if self._loading:
            return
        self._enable()
        self._timer.start()

    def _emit(self) -> None:
        self._timer.stop()
        self._query = self._from_controls()
        self.queryChanged.emit(self.query)

    # ----- scope and curation (from the window / browser)

    def set_scope(self, paths, emit: bool = True) -> None:
        self._query.recordings = [str(p) for p in paths]
        self._fill_scope()
        if emit:
            self._emit()

    def add_scope(self, paths) -> None:
        have = list(self._query.recordings)
        self.set_scope(have + [str(p) for p in paths if str(p) not in have])

    def _fill_scope(self) -> None:
        self.scope_list.clear()
        recs = evs.expand_recordings(self._query.recordings)
        for p in self._query.recordings:
            pp = Path(p)
            it = QListWidgetItem(f"{pp.name}/" if pp.is_dir() else recname.stem_of(pp.name))
            it.setToolTip(str(pp))
            it.setData(Qt.ItemDataRole.UserRole, str(p))
            self.scope_list.addItem(it)
        self.scope_label.setText(f"{len(recs)} recording{'s' if len(recs) != 1 else ''}")

    def set_curation(self, excludes, pins) -> None:
        self._query.excludes = [list(k) for k in excludes]
        self._query.pins = [list(k) for k in pins]
        self._emit()

    def _add_recordings(self) -> None:
        start = str(Path(self._query.recordings[0]).parent) if self._query.recordings else ""
        paths, _ = QFileDialog.getOpenFileNames(self, "Add recordings", start,
                                                "Recordings (*.jsonl);;All files (*)")
        if paths:
            self.add_scope(paths)

    def _add_folder(self) -> None:
        d = QFileDialog.getExistingDirectory(self, "Add a folder of recordings")
        if d:
            self.add_scope([d])

    def _remove_selected(self) -> None:
        gone = {it.data(Qt.ItemDataRole.UserRole) for it in self.scope_list.selectedItems()}
        if gone:
            self.set_scope([p for p in self._query.recordings if p not in gone])

    def set_status(self, text: str) -> None:
        self.status.setText(text + (f"\n{self.note}" if self.note else ""))

    # ----- saved comparisons

    def refresh_comparisons(self) -> None:
        self.cmp_box.blockSignals(True)
        self.cmp_box.clear()
        self.cmp_box.addItem("(not saved)", "")
        for name in evs.list_comparisons():
            self.cmp_box.addItem(name, name)
        i = self.cmp_box.findData(self._saved_name)
        self.cmp_box.setCurrentIndex(max(0, i))
        self.cmp_box.blockSignals(False)
        self._enable()

    def _on_cmp_picked(self, _i: int) -> None:
        name = self.cmp_box.currentData()
        if name:
            self.load_comparison(name)

    def load_comparison(self, name: str) -> None:
        try:
            q = evs.load_comparison(name)
        except (OSError, ValueError, KeyError) as exc:
            QMessageBox.warning(self, "Comparison", f"Could not load {name}: {exc}")
            return
        self._saved_name = name
        self.refresh_comparisons()
        self.set_query(q)

    def save_as(self, name: Optional[str] = None) -> Optional[str]:
        if name is None:
            name, ok = QInputDialog.getText(self, "Save comparison", "Name:",
                                            text=self._saved_name)
            if not ok or not name.strip():
                return None
        q = self._from_controls()
        self._query = q
        evs.save_comparison(name, q)
        self._saved_name = evs._safe(name)
        self.refresh_comparisons()
        return self._saved_name

    def _delete_cmp(self) -> None:
        if self._saved_name and QMessageBox.question(
                self, "Delete comparison", f"Delete {self._saved_name}?") == \
                QMessageBox.StandardButton.Yes:
            evs.delete_comparison(self._saved_name)
            self._saved_name = ""
            self.refresh_comparisons()

    def _revert(self) -> None:
        if self._saved_name:
            self.load_comparison(self._saved_name)
