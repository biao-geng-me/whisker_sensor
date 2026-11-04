"""Toolbar defaults and value-box typing (offscreen): Link Y on by default, typing
into every value box while data flows (the FFT / Spec colour min / max boxes used
to be overwritten by each Auto redraw), the FFT lower plot's default mode and its
Box over time history on data time (a replay at 8x), and the view windows' timeline
hidden by default.

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_ui_defaults.py
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from PyQt6.QtCore import Qt  # noqa: E402
from PyQt6.QtTest import QTest  # noqa: E402
from PyQt6.QtWidgets import QApplication, QMessageBox, QToolBar  # noqa: E402

from wavi_py.tests import replay_fixture as fx  # noqa: E402


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


def _open_views(w) -> dict:
    for t in (w._toggle_line, w._toggle_spectrum, w._toggle_fft, w._toggle_spec):
        t.setChecked(True)
    return {"line": w._line_view, "spectrum": w._spectrum_view, "fft": w._fft_view,
            "spec": w._spec_view}


@pytest.fixture(scope="module")
def sim(app, tmp_path_factory):
    from wavi_py.main_window import MainWindow
    w = MainWindow(transport="sim", nsensor=16, outpath=tmp_path_factory.mktemp("sim"))
    w.show()
    w._connect()
    views = _open_views(w)
    assert _pump(app, 10, lambda: w._fft_view._fft is not None and w._spec_view.isVisible())
    yield w, views
    w.close()


# ----------------------------------------------------------------------
# Link Y
# ----------------------------------------------------------------------

def _linked(v) -> bool:
    vb0 = v._tiles[0].getViewBox()
    return all(p.getViewBox().linkedView(1) is vb0 for p in v._tiles[1:])


@pytest.mark.parametrize("name", ["line", "spectrum"])
def test_link_y_on_by_default(sim, name):
    _w, views = sim
    v = views[name]
    assert v._act_link_y.isChecked() and _linked(v)
    v._act_grid.setChecked(False)                  # stacked, then back to the grid
    v._act_grid.setChecked(True)
    v._act_reset.trigger()
    assert v._act_link_y.isChecked() and _linked(v)
    v._act_link_y.setChecked(False)                # still a toggle
    assert not _linked(v)
    v._act_link_y.setChecked(True)


def test_link_y_default_fresh_views(app):
    from wavi_py.views.line_view import LineView
    from wavi_py.views.spectrum_view import SpectrumView
    for cls in (LineView, SpectrumView):
        v = cls(16, 32)
        assert v._act_link_y.isChecked() and _linked(v)
        v.close()


# ----------------------------------------------------------------------
# value boxes: a normal text field while the data flows
# ----------------------------------------------------------------------

BOXES = [("line", "_y_top"), ("spectrum", "_y_top"), ("fft", "_lo"), ("fft", "_hi"),
         ("fft", "_fmin"), ("fft", "_fmax"), ("fft", "_history"), ("spec", "_lo"), ("spec", "_hi")]


def _click_into(app, view, box) -> None:
    view.activateWindow()
    QTest.mouseClick(box.lineEdit(), Qt.MouseButton.LeftButton)
    _pump(app, 0.05)
    assert QApplication.focusWidget() in (box, box.lineEdit())


@pytest.mark.parametrize("name,attr", BOXES)
def test_typing_into_value_box(app, sim, name, attr):
    w, views = sim
    v = views[name]
    box = getattr(v, attr)
    if name == "fft":
        v._mode.setCurrentIndex(2)                  # (history is only enabled in Box over time)
        v._act_auto.setChecked(True)
    if name == "spec":
        v._act_auto.setChecked(True)
    ed = box.lineEdit()
    _click_into(app, v, box)
    QTest.keyClick(ed, Qt.Key.Key_A, Qt.KeyboardModifier.ControlModifier)   # select all
    assert ed.selectedText() == box.text()
    QTest.keyClicks(ed, "12.5")
    _pump(app, 0.6)                                  # several redraws while the user thinks
    assert "12.5" in box.text(), box.text()
    QTest.keyClick(ed, Qt.Key.Key_Backspace)
    assert "12." in box.text() and "12.5" not in box.text()
    QTest.keyClick(ed, Qt.Key.Key_Backspace)
    QTest.keyClick(ed, Qt.Key.Key_Backspace)
    QTest.keyClicks(ed, "4")                         # "14"
    pos = ed.cursorPosition()
    QTest.keyClick(ed, Qt.Key.Key_Left)              # caret moves (not the window's pan key)
    assert ed.cursorPosition() == pos - 1
    QTest.keyClicks(ed, "0")                         # "104"; 0 is also a window shortcut
    _pump(app, 0.3)
    assert "104" in box.text(), box.text()
    QTest.keyClick(ed, Qt.Key.Key_Return)
    _pump(app, 0.2)
    want = min(104.0, box.maximum())
    assert box.value() == pytest.approx(want)
    if attr in ("_lo", "_hi"):
        assert not v._act_auto.isChecked()          # a typed range is used
        _pump(app, 0.3)
        assert box.value() == pytest.approx(want)   # and not overwritten by the next redraw


def test_value_box_applies_on_focus_out(app, sim):
    _w, views = sim
    v = views["fft"]
    v._act_auto.setChecked(True)
    box = v._hi
    _click_into(app, v, box)
    QTest.keyClick(box.lineEdit(), Qt.Key.Key_A, Qt.KeyboardModifier.ControlModifier)
    QTest.keyClicks(box.lineEdit(), "33")
    _pump(app, 0.3)
    v.centralWidget().setFocus()                    # click away, no Enter
    _pump(app, 0.2)
    assert box.value() == pytest.approx(33.0) and not v._act_auto.isChecked()
    v._act_auto.setChecked(True)


# ----------------------------------------------------------------------
# FFT lower plot
# ----------------------------------------------------------------------

def test_fft_lower_plot_default_is_box_over_time(app):
    from wavi_py.views.fft_view import FftView
    v = FftView(32, 51, 1.0)
    assert v._mode.currentText() == "Box over time" and v._history.isEnabled()
    v.close()


def test_fft_history_on_data_time(app):
    """Frames keyed on the times passed in; all frames of a call are kept; time going
    back starts over."""
    from wavi_py.views.fft_view import FftView
    v = FftView(4, 51, 1.0)
    v.show()
    frames = np.ones((300, 51, 4), dtype=np.float32)
    t = np.linspace(0, 29.9, 300)                    # 30 s of data in one wall-clock instant
    v.update_view(frames[-1], 100.0, frames, t)
    assert len(v._hist) == 300 and v._hist[-1][0] == pytest.approx(29.9)
    v._history.setValue(10)
    used = [p for tt, p in v._hist if v._hist[-1][0] - tt <= v._history.value()]
    assert len(used) == 101                          # 19.9 .. 29.9 s, not all 300
    v.update_view(frames[-1], 100.0, frames[:2], np.array([1.0, 1.1]))
    assert [tt for tt, _ in v._hist] == [1.0, 1.1]
    v.close()


def test_fft_history_replay_8x(app, tmp_path):
    """A replay at 8x: the Box over time history spans recording seconds."""
    from wavi_py.main_window import MainWindow
    from wavi_py.scan_mode import parse_mode
    p = tmp_path / "rec.jsonl"
    fx.write_recording(p)
    w = MainWindow(transport="replay", nsensor=16, fs=80.0, t_buffer_s=2.0, replay_speed=8.0,
                   replay_path=str(p), scan_mode=parse_mode("wide"))
    w.show()
    try:
        w.open_replay(str(p))
        assert _pump(app, 10, lambda: w._client is not None and w._replay.active)
        w._toggle_fft.setChecked(True)
        fv = w._fft_view
        w._pause_btn.setChecked(False)               # play
        t0 = time.monotonic()
        assert _pump(app, 10, lambda: len(fv._hist) > 1 and fv._hist[-1][0] - fv._hist[0][0] > 3.0)
        wall = time.monotonic() - t0
        span = fv._hist[-1][0] - fv._hist[0][0]
        assert span > 2 * wall                       # data time, not wall time
        assert abs(fv._hist[-1][0] - w._sig_t[-1]) < 0.2
        dt = np.diff([tt for tt, _ in fv._hist])
        assert np.all(dt > 0) and np.median(dt) < 0.1   # every frame, not one per UI tick
    finally:
        w.close()


def test_link_y_a_button_refits(app):
    """Link Y on: after a mouse zoom of a tile, its A button fits the shared Y again."""
    from wavi_py.views.line_view import LineView
    v = LineView(4, 8, t_buffer_s=2.0)
    v.resize(800, 600)
    v.show()
    t = np.arange(200) / 100.0
    sig = (250 + (1.0 + np.arange(8))[None, :] * np.sin(2 * np.pi * 2 * t)[:, None]).astype(np.float32)
    v.update_view(t, sig, np.full(8, 250, np.float32), 1.0, x_range=(0.0, t[-1]))
    v._render(labels=True)
    vb = v._tiles[3].getViewBox()
    fitted = tuple(vb.viewRange()[1])
    assert fitted[1] > 8                                  # the 8 mV channel fits in every tile
    vb.scaleBy(y=0.25)
    v._on_manual_range()
    v._render()
    assert tuple(vb.viewRange()[1]) != pytest.approx(fitted, rel=0.05)
    vb.enableAutoRange()                                  # the A button
    v._render()
    assert tuple(vb.viewRange()[1]) == pytest.approx(fitted, rel=0.05)
    assert tuple(v._tiles[0].getViewBox().viewRange()[1]) == pytest.approx(fitted, rel=0.05)
    v.close()


def test_level_box_follows_db_with_caret_parked(app, sim):
    """A focused box nobody typed into still shows the range in use: with Auto off,
    switching dB while the caret sits in the min box must not leave the linear value
    there (the heatmap reads the boxes)."""
    _w, views = sim
    v = views["spec"]
    v._act_auto.setChecked(False)
    _pump(app, 0.2)
    _click_into(app, v, v._lo)
    db0 = v._act_db.isChecked()
    v._act_db.setChecked(not db0)
    _pump(app, 0.2)
    lo, hi = v._levels_saved[v.db]
    assert v._lo.value() == pytest.approx(lo, abs=10 ** -v._lo.decimals())
    assert v._hi.value() == pytest.approx(hi, abs=10 ** -v._hi.decimals())
    v._act_db.setChecked(db0)
    v.centralWidget().setFocus()
    v._act_auto.setChecked(True)


def test_spectral_columns_wait_for_despike(app, sim):
    """Live Despike: the spectrogram / FFT columns end before the provisional
    (still raw) samples, so a spike fixed later never reaches them."""
    w, _views = sim
    assert w._despiker is not None and w._despiker.pending > 0
    seen = []
    orig = w._fft_view.update_view

    def spy(fft_map, fs, frames=None, t=None):
        if t is not None and len(t):
            seen.append((float(t[-1]), float(w._sig_t[-1])))
        return orig(fft_map, fs, frames, t)
    w._fft_view.update_view = spy
    try:
        _pump(app, 1.0, lambda: len(seen) > 3)
    finally:
        w._fft_view.update_view = orig
    lag = w._despiker.pending / w._fs
    assert seen and all(newest - last >= lag - 1.5 / w._fs for last, newest in seen)


# ----------------------------------------------------------------------
# timeline: the panel's shows with a source, the view windows' only on request
# ----------------------------------------------------------------------

def test_timeline_hidden_in_view_windows(app, tmp_path):
    from wavi_py.main_window import MainWindow
    p = tmp_path / "rec.jsonl"
    fx.write_recording(p)
    w = MainWindow(transport="replay", nsensor=16, fs=80.0, t_buffer_s=2.0, replay_path=str(p))
    w.show()
    try:
        w.open_replay(str(p))
        assert _pump(app, 10, lambda: w._client is not None and w._replay.active)
        w._toggle_line.setChecked(True)
        w._toggle_imu.setChecked(True)
        _pump(app, 0.1)
        assert w._timeline_docks[0].isVisible()                     # the panel keeps it
        for v in (w._line_view, w._imu_view):
            d = v.timeline_dock
            assert v.isVisible() and not d.isVisible()
            act = d.toggleViewAction()
            assert act in v.findChildren(QToolBar)[0].actions()      # on its toolbar
            act.trigger()                                           # the Timeline toggle
            assert d.isVisible()
        w._disconnect()                                             # no source: all go
        _pump(app, 0.1)
        assert not any(d.isVisible() for d in w._timeline_docks)
    finally:
        w.close()
