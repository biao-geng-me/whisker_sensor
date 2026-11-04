"""Window chrome (offscreen): Ctrl+R records from any wavi_py window and respects
the Rec button's state; every plot window's recording indicator follows the panel;
a view turned on (panel button or a window's Go to menu) comes to the front.
Screenshots go to $WAVI_TEST_SHOTS if set.

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_chrome.py
"""

from __future__ import annotations

import os
import sys
import time
from datetime import datetime, timedelta
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from PyQt6.QtCore import Qt  # noqa: E402
from PyQt6.QtGui import QAction, QKeySequence, QShortcut  # noqa: E402
from PyQt6.QtTest import QTest  # noqa: E402
from PyQt6.QtWidgets import QApplication, QMenu, QMessageBox  # noqa: E402

from wavi_py.tests import replay_fixture as fx  # noqa: E402
from wavi_py.views.base import (REC_FOREIGN, REC_OFF, REC_ON, REC_PENDING,  # noqa: E402
                                RecIndicator, fmt_secs)

SHOTS = os.environ.get("WAVI_TEST_SHOTS")


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture(autouse=True)
def no_modal_errors(monkeypatch):
    def critical(parent, title, text, *a, **k):
        raise AssertionError(f"{title}: {text}")
    monkeypatch.setattr(QMessageBox, "critical", critical)


def _pump(app, secs: float, pred=lambda: False) -> bool:
    t_end = time.monotonic() + secs
    while time.monotonic() < t_end:
        app.processEvents()
        if pred():
            return True
        time.sleep(0.005)
    app.processEvents()
    return pred()


def _shot(widget, name: str) -> None:
    if SHOTS:
        Path(SHOTS).mkdir(parents=True, exist_ok=True)
        widget.grab().save(str(Path(SHOTS) / f"{name}.png"))


@pytest.fixture
def sim(app, tmp_path):
    from wavi_py.main_window import MainWindow
    w = MainWindow(transport="sim", nsensor=16, outpath=tmp_path)
    w.show()
    w._connect()
    assert _pump(app, 10, lambda: w._record_btn.isEnabled())
    yield w
    if w._is_recording:
        w._stop_recording()
    w.close()


@pytest.fixture
def replay(app, tmp_path):
    from wavi_py.main_window import MainWindow
    p = tmp_path / "rec.jsonl"
    fx.write_recording(p)
    w = MainWindow(transport="replay", nsensor=16, fs=80.0, t_buffer_s=2.0, replay_path=str(p))
    w.show()
    w.open_replay(str(p))
    assert _pump(app, 10, lambda: w._client is not None and w._replay.active)
    yield w
    w.close()


def _ctrl_r(app, target) -> None:
    target.activateWindow()
    _pump(app, 0.05)
    QTest.keyClick(target, Qt.Key.Key_R, Qt.KeyboardModifier.ControlModifier)
    _pump(app, 0.1)


# ----------------------------------------------------------------------
# Ctrl+R
# ----------------------------------------------------------------------

def test_ctrl_r_unique(app, sim):
    """Nothing else in wavi_py binds Ctrl+R (the view windows' plain R is Reset view)."""
    w = sim
    for b in (w._toggle_line, w._toggle_spectrum, w._toggle_fft, w._toggle_spec, w._toggle_imu):
        b.setChecked(True)
    seq = QKeySequence("Ctrl+R")
    owners = []
    # (closed windows of earlier tests linger as hidden top-levels: a MainWindow is never
    # freed; a hidden window's shortcuts don't fire, so only the shown ones count)
    for top in [t for t in QApplication.topLevelWidgets() if t.isVisible()]:
        for a in top.findChildren(QAction) + top.actions():
            if any(k == seq for k in a.shortcuts()):
                owners.append(("action", a.text()))
        for sc in top.findChildren(QShortcut):
            if sc.key() == seq:
                owners.append(("shortcut", sc.parent()))
    assert owners == [("shortcut", w)], owners
    sc = next(s for s in w.findChildren(QShortcut) if s.key() == seq)
    assert sc.context() == Qt.ShortcutContext.ApplicationShortcut


@pytest.mark.parametrize("where", ["panel", "line", "imu"])
def test_ctrl_r_records_from_any_window(app, sim, where):
    """Ctrl+R in the panel or a view window does what Rec does: start, then stop."""
    w = sim
    clicks = []
    w._record_btn.clicked.connect(lambda: clicks.append(1))
    target = {"panel": w, "line": w._line_view, "imu": w._imu_view}[where]
    if target is not w:
        {"line": w._toggle_line, "imu": w._toggle_imu}[where].setChecked(True)
    _ctrl_r(app, target)
    assert w._is_recording and clicks == [1]
    assert w._dat_writer is not None
    _ctrl_r(app, target)
    assert not w._is_recording and clicks == [1, 1]


def test_ctrl_r_replay_does_nothing(app, replay):
    """Replay: Rec is disabled, so Ctrl+R records nothing and says why."""
    w = replay
    w._toggle_line.setChecked(True)
    assert not w._record_btn.isEnabled()
    clicks = []
    w._record_btn.clicked.connect(lambda: clicks.append(1))
    _ctrl_r(app, w._line_view)
    assert not w._is_recording and not clicks
    assert "replay" in w._path_label.text()
    if QApplication.activeWindow() is w._line_view:          # the window's own note
        assert "replay" in w._line_view.statusBar().currentMessage()
    _ctrl_r(app, w)
    assert not w._is_recording and not clicks


# ----------------------------------------------------------------------
# the recording indicator
# ----------------------------------------------------------------------

def test_rec_indicator_unit(app):
    ind = RecIndicator()
    assert ind.state == REC_OFF and ind.value() == 0 and ind.format() == "not recording"
    ind.set_state(REC_ON, 30.0, 60.0)
    assert ind.value() == 500 and ind.format() == "REC 30 s / 1:00"
    ind.set_state(REC_ON, 75.0, 60.0)                       # past the time: stays full
    assert ind.value() == 1000
    ind.set_state(REC_ON, 3725.0, 0.0)                      # no auto-stop: full, elapsed
    assert ind.value() == 1000 and ind.format() == "REC 1:02:05"
    ind.set_state(REC_PENDING)
    assert ind.value() == 0 and ind.format() == "Starting…"
    ind.set_state(REC_FOREIGN, 12.0, 0.0, "tip")
    assert ind.value() == 1000 and "12 s" in ind.format() and ind.toolTip() == "tip"
    ind.set_state(REC_OFF)
    assert ind.value() == 0 and ind.format() == "not recording"
    assert [fmt_secs(s) for s in (0, 59.9, 60, 3600)] == ["0 s", "59 s", "1:00", "1:00:00"]


def _indicators(w) -> list:
    return [v.rec_indicator for v in (w._line_view, w._spectrum_view, w._fft_view,
                                      w._spec_view, w._imu_view)]


def test_rec_indicator_follows_panel(app, sim):
    """Every plot window shows the panel's Rec state: off, auto-stop progress,
    elapsed only, off again."""
    w = sim
    w._toggle_line.setChecked(True)
    w._toggle_imu.setChecked(True)
    inds = _indicators(w)
    assert all(i is not None and i.state == REC_OFF for i in inds)
    _shot(w._line_view, "line_rec_off")
    _shot(w._imu_view, "imu_rec_off")

    w._set_auto_stop(60)
    w._record_btn.click()
    assert w._is_recording
    w._record_start_dt = datetime.now() - timedelta(seconds=30)    # 30 s into it
    _pump(app, 0.2)                                                # the tick pushes it
    for i in inds:
        assert i.state == REC_ON and 480 <= i.value() <= 520, (i.value(), i.format())
        assert i.format().startswith("REC 30 s / 1:00") or i.format().startswith("REC 31 s")
    _shot(w._line_view, "line_rec_autostop")
    _shot(w._imu_view, "imu_rec_autostop")

    w._set_auto_stop(0)                                            # auto-stop off
    _pump(app, 0.2)
    for i in inds:
        assert i.state == REC_ON and i.value() == 1000 and "/" not in i.format()
    _shot(w._line_view, "line_rec_running")
    _shot(w._imu_view, "imu_rec_running")

    w._record_btn.click()
    assert not w._is_recording
    assert all(i.state == REC_OFF and i.value() == 0 for i in inds)


def test_rec_indicator_lazy_window(app, sim):
    """A window built later (as the Dome / ROV are) starts with the current state."""
    from wavi_py.views.base import ViewWindow
    from PyQt6.QtWidgets import QToolBar
    w = sim
    w._record_btn.click()
    v = ViewWindow()
    tb = QToolBar("t")
    v.addToolBar(tb)
    tb.addAction("x")
    w._dome_view = v                    # stands in for the GL window (built, then chrome)
    try:
        w._add_chrome(v)
        assert v.rec_indicator.state == REC_ON
        w._record_btn.click()
        assert v.rec_indicator.state == REC_OFF
    finally:
        w._dome_view = None
        v.close()


# ----------------------------------------------------------------------
# bring to front, Go to
# ----------------------------------------------------------------------

def test_panel_toggle_brings_to_front(app, sim, monkeypatch):
    import wavi_py.main_window as mw
    raised = []
    orig = mw.bring_to_front
    monkeypatch.setattr(mw, "bring_to_front", lambda win: (raised.append(win), orig(win)))
    w = sim
    w._toggle_spec.setChecked(True)
    assert w._spec_view.isVisible() and raised == [w._spec_view]
    w._spec_view.showMinimized()
    _pump(app, 0.05)
    w.show_view("Spec")                       # already open: un-minimized, raised again
    assert raised == [w._spec_view, w._spec_view]
    assert w._spec_view.isVisible() and not w._spec_view.isMinimized()
    assert w._toggle_spec.isChecked()


def _goto_menu(view) -> QMenu:
    menu = view.goto_button.menu()
    menu.aboutToShow.emit()                    # filled as it opens
    return menu


def test_goto_menu(app, sim, monkeypatch):
    import wavi_py.main_window as mw
    raised = []
    orig = mw.bring_to_front
    monkeypatch.setattr(mw, "bring_to_front", lambda win: (raised.append(win), orig(win)))
    w = sim
    w._toggle_line.setChecked(True)
    raised.clear()
    menu = _goto_menu(w._line_view)
    acts = {a.text(): a for a in menu.actions() if a.text()}
    assert list(acts) == ["Control panel", "Line", "Spectrum", "FFT", "Spec", "IMU", "Dome", "ROV",
                          "New analysis window"]
    assert acts["Line"].isChecked() and not acts["FFT"].isChecked()
    # pick FFT from the Line window: opened through its panel button, raised
    acts["FFT"].trigger()
    assert w._toggle_fft.isChecked() and w._fft_view.isVisible() and raised == [w._fft_view]
    # the menu marks it open now; picking it again only raises it
    acts = {a.text(): a for a in _goto_menu(w._line_view).actions() if a.text()}
    assert acts["FFT"].isChecked()
    acts["FFT"].trigger()
    assert raised == [w._fft_view, w._fft_view] and w._toggle_fft.isChecked()
    acts["Control panel"].trigger()
    assert raised[-1] is w
    # every plot window has one (and the indicator), at the left of its first toolbar
    for v in (w._line_view, w._spectrum_view, w._fft_view, w._spec_view, w._imu_view):
        assert v.goto_button is not None and v.rec_indicator is not None
    if SHOTS:
        m = w._line_view.goto_button.menu()
        m.aboutToShow.emit()
        m.popup(w._line_view.mapToGlobal(w._line_view.goto_button.geometry().bottomLeft()))
        _pump(app, 0.2)
        _shot(m, "goto_menu")
        _shot(w._line_view, "line_goto_window")
        m.hide()
