"""Analysis phase C2: the Spectrum and Spectrogram tabs and the Compare sub-range.

Spectrum values against a direct numpy computation (a known sine, both windows),
each selection its own frequency grid, the sub-range clipped to each selection,
recompute only on a change (none on a tick with nothing changed, none while
hidden), the spectrogram tile and its colour range per unit, the region menu's entries.
Screenshots go to $WAVI_TEST_SHOTS if set; $WAVI_ANALYSIS_REC replays a real
recording for them.

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_analysis_c2.py
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from PyQt6.QtCore import QPointF  # noqa: E402
from PyQt6.QtWidgets import QApplication, QMessageBox  # noqa: E402

from wavi_py.daq.analysis import Dataset, Selection, SelSource, whisker_variables  # noqa: E402
from wavi_py.daq.series import IMU, WSK, ArraySeries  # noqa: E402
from wavi_py.tests import replay_fixture as fx  # noqa: E402
from wavi_py.tests.test_analysis import _drag, _open, _pump, _shot, SHOTS  # noqa: E402

A, F0, FS = 3.0, 12.0, 250.0


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture(autouse=True)
def no_modal_errors(monkeypatch):
    def critical(parent, title, text, *a, **k):
        raise AssertionError(f"{title}: {text}")
    monkeypatch.setattr(QMessageBox, "critical", critical)


@pytest.fixture
def win(app, tmp_path):
    p = tmp_path / "rec.jsonl"
    fx.write_recording(p)
    w = _open(app, p, t_buffer_s=2.0)
    yield w
    w.close()


def _sine_source(dur: float = 20.0):
    t = np.arange(0, dur, 1 / FS)
    y = np.stack([A * np.sin(2 * np.pi * F0 * t) + 7.0,                  # + an offset
                  A * np.sin(2 * np.pi * F0 * t) + 0.5 * t], axis=1)     # + a ramp
    ti = np.arange(0, dur, 0.01)
    return SelSource(ArraySeries({WSK: (t, y.astype(np.float64)),
                                  IMU: (ti, np.zeros((len(ti), 9)))},
                                 {WSK: ["W1_A", "W1_B"], IMU: [f"i{c}" for c in range(9)]}))


def _direct(y: np.ndarray, window: str) -> np.ndarray:
    """The amplitude spectrum written out: mean removed, window, |rfft| * 2 / sum(w),
    DC and Nyquist not doubled."""
    n = len(y)
    w = np.hanning(n) if window == "hann" else np.ones(n)
    a = np.abs(np.fft.rfft((y - y.mean()) * w)) * 2 / w.sum()
    a[0] /= 2
    if n % 2 == 0:
        a[-1] /= 2
    return a


@pytest.mark.parametrize("window", ["hann", "rect"])
def test_spectrum_sine(window):
    src = _sine_source()
    sel = Selection(1, 2.0, 6.0, src, (0, 20))                          # 4 s: 0.25 Hz bins
    v = whisker_variables(2, ["W1"])
    ds = Dataset([sel], v)
    f, a = ds.spectrum(sel, v[0], window, "mean")
    t, y = ds.samples(sel, v[0])
    assert np.allclose(a[a > 1e-9], _direct(y, window)[a > 1e-9], atol=1e-9)
    assert len(f) == len(y) // 2 + 1 and abs(f[1] - FS / len(y)) < 1e-6
    k = int(np.argmax(a))
    assert abs(f[k] - F0) <= f[1] / 2 + 1e-9                           # the frequency
    assert abs(a[k] - A) < (0.02 if window == "rect" else 0.05) * A    # and the amplitude


def test_detrend_options():
    src = _sine_source()
    sel = Selection(1, 2.0, 6.0, src, (0, 20))
    v = whisker_variables(2, ["W1"])
    ds = Dataset([sel], v)
    lowbins = slice(1, 8)                                              # below 2 Hz
    _, mean = ds.spectrum(sel, v[1], "hann", "mean")
    _, lin = ds.spectrum(sel, v[1], "hann", "linear")
    assert lin[lowbins].max() < 0.1 * mean[lowbins].max()             # the ramp gone
    _, none = ds.spectrum(sel, v[0], "rect", "none")
    assert abs(none[0] - 7.0) < 0.05                                   # DC: the offset


def test_frequency_grids_and_sub_range():
    src = _sine_source()
    v = whisker_variables(2, ["W1"])
    s1, s2 = Selection(1, 1.0, 3.0, src, (0, 20)), Selection(2, 5.0, 13.0, src, (0, 20))
    ds = Dataset([s1, s2], v[:1])
    f1, _ = ds.spectrum(s1, v[0])
    f2, _ = ds.spectrum(s2, v[0])
    assert abs(f1[1] - 0.5) < 0.01 and abs(f2[1] - 0.125) < 0.01      # 1 / length each
    # sub-range (relative): clipped to each selection's length; none when shorter
    ds.sub = (1.0, 4.0)
    assert ds.span(s1) == (2.0, 3.0) and ds.span(s2) == (6.0, 9.0)
    ds.sub = (2.5, 6.0)
    assert ds.span(s1) is None and ds.spectrum(s1, v[0]) is None
    t, _ = ds.samples(s2, v[0], ds.span(s2))
    assert t[0] >= 7.5 - 1e-9 and t[-1] <= 11.0 + 1e-9
    # spectrogram: segment centres from the selection's start, inside the span
    tc, f, a = ds.spectrogram(s2, v[0], 1.0, 0.5)
    assert tc[0] >= 2.5 and tc[-1] <= 6.0 and a.shape == (len(tc), len(f))
    assert abs(f[np.argmax(a.mean(axis=0))] - F0) <= 1.0


# ----------------------------------------------------------------------
# the tabs in a replay
# ----------------------------------------------------------------------

def test_spectrum_tab_recompute_on_change(app, win):
    w = win
    m = w._selections
    sels = [m.create(1.0, 4.0), m.create(4.5, 7.5), m.create(7.0, 9.5)]
    keys = ["wsk:0", "wsk:1", "wsk:2", "wsk:3"]
    aw = w.new_analysis_window(sels[0], keys)
    for s in sels[1:]:
        aw.include(s)
    aw.set_tab("Spectrum")
    sp = aw.spectrum
    assert _pump(app, lambda: len(sp.curves) == 12, 3)
    n_comp, n_rend = sp.computed, sp.renders
    assert n_comp == 12
    # the replay plays (ticks, new data): nothing about the selections changed
    w._pause_btn.setChecked(False)
    _pump(app, lambda: False, 0.6)
    w._pause_btn.setChecked(True)
    aw._render()                                       # even a forced render: no work
    assert (sp.computed, sp.renders) == (n_comp, n_rend)
    # an edge of selection 2 moves: its 4 traces only
    m.set_range(sels[1], 4.5, 8.0)
    assert _pump(app, lambda: sp.computed == n_comp + 4, 2)
    _pump(app, lambda: False, 0.2)
    assert sp.computed == n_comp + 4
    # an option: all again
    sp.window_box.setCurrentIndex(1)
    assert _pump(app, lambda: sp.computed == n_comp + 16, 2)
    # hidden: no work at all
    aw.hide()
    c, r = sp.computed, sp.renders
    m.set_range(sels[0], 1.0, 3.5)
    sp.detrend_box.setCurrentIndex(1)
    _pump(app, lambda: False, 0.4)
    assert (sp.computed, sp.renders) == (c, r)
    aw.show()
    assert _pump(app, lambda: sp.computed == c + 12, 2)
    # different lengths: different grids; the curves are the dataset's spectra
    ds = aw.dataset()
    f, a = ds.spectrum(sels[0], ds.variables[0], *sp.options)
    x, y = sp.curves[(1, "wsk:0")].getData()                      # linear amplitude (the default)
    assert np.allclose(x, f) and np.allclose(y, a)
    x2, _ = sp.curves[(2, "wsk:0")].getData()                     # 2.5 s and 3.5 s long
    assert abs((x[1] - x[0]) - 1 / 2.5) < 0.01 and abs((x2[1] - x2[0]) - 1 / 3.5) < 0.01
    aw.close()


def test_sub_range_from_compare(app, win):
    w = win
    m = w._selections
    a, b = m.create(1.0, 4.0), m.create(5.0, 6.5)                       # 3 s and 1.5 s
    aw = w.new_analysis_window(a, ["wsk:0"])
    aw.include(b)
    assert _pump(app, lambda: bool(aw.compare.curves), 2)
    vb = next(iter(aw.compare.rows.values())).getViewBox()
    from wavi_py.tests.test_analysis import _Drag
    ev = _Drag(vb.mapViewToScene(QPointF(0.5, 0)), vb.mapViewToScene(QPointF(2, 0)), start=True)
    assert aw.compare._drag(vb, ev) is False and aw.sub is None        # Select off: the pan's
    aw._act_select.setChecked(True)
    _drag(vb, 0.5, 2.0)
    assert aw.sub is not None and abs(aw.sub[0] - 0.5) < 0.05 and abs(aw.sub[1] - 2.0) < 0.05
    assert all(band.isVisible() for band in aw.compare.bands)
    ds = aw.dataset()
    assert ds.span(a, ds.subs[0]) == pytest.approx((1.0 + aw.sub[0], 1.0 + aw.sub[1]))
    assert ds.span(b, ds.subs[0]) == pytest.approx((5.0 + aw.sub[0], 6.5))   # clipped to its length
    assert "Sub-range s1 0.50 .. 2.00 s" in aw.spectrum.header.sub.text()
    # the band's edge drags the sub-range
    aw.compare.bands[0].lines[1].setValue(1.5)
    assert aw.sub[1] == pytest.approx(1.5)
    # Spectrum uses it
    aw.set_tab("Spectrum")
    assert _pump(app, lambda: (1, "wsk:0", 1) in aw.spectrum.curves, 2)
    f, _ = aw.spectrum.curves[(1, "wsk:0", 1)].getData()
    assert abs(f[1] - f[0] - 1 / (aw.sub[1] - aw.sub[0])) < 0.05
    aw._act_clear_sub.trigger()                                         # Esc
    assert aw.sub is None and not any(b_.isVisible() for b_ in aw.compare.bands)
    assert aw.spectrum.header.sub.text().startswith("Whole selections")
    aw.close()


def test_spectrogram_tile_and_options(app, win):
    """One tile, the picked (variable, selection); options recompute; a typed range
    means fixed, as in the Spec window."""
    w = win
    m = w._selections
    sels = [m.create(1.0, 4.0), m.create(4.5, 7.5)]
    aw = w.new_analysis_window(sels[0], ["wsk:0", "imu:2"])
    aw.include(sels[1])
    sg = aw.spectrogram
    assert _pump(app, lambda: sg.pick == (1, "wsk:0") and sg.data is not None, 3)
    tc, f, amp = aw.dataset().spectrogram(sels[0], aw.dataset().variables[0], 1.0, 0.75)
    assert np.allclose(sg.img.image, amp) and sg.var_box.count() == 2 and sg.sel_box.count() == 2
    # nothing changed: no work; an option: the shown tile again (only it)
    c = sg.computed
    aw._render()
    assert sg.computed == c
    sg.seg_box.setCurrentIndex(0)
    assert _pump(app, lambda: sg.computed == c + 1, 3)
    sg.hi_box.setValue(1.0)
    assert not sg.auto_box.isChecked()
    _pump(app, lambda: False, 0.2)
    assert sg.cbar.levels() == pytest.approx((sg.lo_box.value(), 1.0))
    aw.close()


def test_spectrogram_colour_range_per_unit(app, win):
    from wavi_py.views.spectral_base import auto_levels
    w = win
    m = w._selections
    sels = [m.create(1.0, 4.0), m.create(4.5, 7.5)]
    aw = w.new_analysis_window(sels[0], ["wsk:0", "imu:2"])
    aw.include(sels[1])
    sg = aw.spectrogram
    assert _pump(app, lambda: sg.data is not None, 3)
    assert sg.levels == pytest.approx(auto_levels(sg.img.image, False, (0, 1)))
    mv = sg.levels
    # the controls set the picked unit's range; the other unit keeps its own
    sg.set_pick(2, "imu:2")
    assert _pump(app, lambda: sg.unit == "mg" and sg.levels != pytest.approx(mv), 3)
    assert sg.auto_box.isChecked() and sg.cbar.axis.labelText == "mg"
    sg.hi_box.setValue(99.0)
    _pump(app, lambda: False, 0.3)
    assert sg.levels[1] == 99.0
    sg.set_pick(1, "wsk:0")
    assert _pump(app, lambda: sg.unit == "mV", 3)
    _pump(app, lambda: False, 0.2)
    assert sg.auto_box.isChecked() and sg.levels == pytest.approx(mv)        # mV still auto
    sg.set_pick(1, "imu:2")
    assert _pump(app, lambda: sg.unit == "mg", 3)
    _pump(app, lambda: False, 0.2)
    assert not sg.auto_box.isChecked() and sg.hi_box.value() == 99.0
    aw.close()


def test_spectrum_frequency_range_follows_nyquist(app, win):
    w = win
    sel = w._selections.create(1.0, 5.0)
    aw = w.new_analysis_window(sel, ["imu:0"])
    aw.set_tab("Spectrum")
    sp = aw.spectrum

    def x_hi():
        if not sp.rows:
            return float("nan")                          # not drawn yet
        return next(iter(sp.rows.values())).getViewBox().viewRange()[0][1]
    assert sp.f_hi.value() == 0.0 and sp.f_hi.text() == "Nyquist"      # the default
    assert _pump(app, lambda: abs(x_hi() - 50.0) < 1.5, 3)            # IMU at 100 Hz
    aw.set_checked(["wsk:0"])                                          # + gauges at 250 Hz
    assert _pump(app, lambda: abs(x_hi() - 125.0) < 2.0, 3)           # the highest Nyquist
    # an edited range stays, whatever is shown; back at its lowest: follows again
    sp.f_hi.setValue(60.0)
    assert _pump(app, lambda: abs(x_hi() - 60.0) < 1.0, 3)
    aw.set_checked(["wsk:0"], False)
    aw.set_checked(["wsk:1"])
    _pump(app, lambda: False, 0.3)
    assert abs(x_hi() - 60.0) < 1.0
    sp.f_hi.setValue(0.0)
    assert _pump(app, lambda: abs(x_hi() - 125.0) < 2.0, 3)
    aw.close()


def test_region_menu_spectral_entries(app, win):
    w = win
    sel = w._selections.create(1.0, 4.0)
    w._selections.request("spectrum", sel, ["wsk:0", "wsk:1"])
    aw = w._analysis[-1]
    assert aw.included == [sel] and aw.spectrum.width() > 1
    aw.grid.set_state([600, 0], [400, 400])                 # the spectrum collapsed
    w._selections.request("spectrum", sel, [])
    assert aw.spectrum.width() > 1                          # back
    sel2 = w._selections.create(5.0, 7.0)
    w._selections.request("spectrogram", sel2, [])
    assert w._analysis[-1] is aw and aw.spectrogram.pick == (sel2.number, "wsk:0")
    # the menu's entries are enabled now
    w._toggle_line.setChecked(True)
    _pump(app, lambda: False, 0.2)
    ov = w._sel_overlays[0]
    region = ov.regions(sel)[1]
    from PyQt6.QtWidgets import QMenu
    shown = []
    orig = QMenu.exec
    QMenu.exec = lambda self, *a: shown.append(self)
    try:
        class Ev:
            def scenePos(self):
                return QPointF()

            def screenPos(self):
                return QPointF()
        ov.menu(region, Ev())
    finally:
        QMenu.exec = orig
    acts = {a.text(): a for a in shown[0].actions() if a.text()}
    assert acts["Spectrum"].isEnabled() and acts["Spectrogram"].isEnabled()
    aw.close()
