"""Y scale Auto / fixed in the Line, Spectrum and FFT windows (offscreen Qt).

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_amplitude.py
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from PyQt6.QtCore import QEvent, Qt  # noqa: E402
from PyQt6.QtGui import QKeyEvent  # noqa: E402
from PyQt6.QtWidgets import QApplication  # noqa: E402

from wavi_py.views.fft_view import FftView  # noqa: E402
from wavi_py.views.line_view import LineView  # noqa: E402
from wavi_py.views.spectrum_view import SpectrumView  # noqa: E402
from wavi_py.views.whisker_plots import GRID, SINGLE, STACKED  # noqa: E402

NS, NCH, FS = 4, 8, 100.0


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


def yrange(p):
    return tuple(p.getViewBox().viewRange()[1])


def auto_y(p) -> bool:
    return bool(p.getViewBox().autoRangeEnabled()[1])


def line_data(amps):
    """2 s of sines, channel c of amplitude amps[c], on DC offsets ~250 mV."""
    t = np.arange(int(2 * FS)) / FS
    v0 = 250.0 + 10 * np.arange(NCH)
    sig = v0 + np.asarray(amps)[None, :] * np.sin(2 * np.pi * 2 * t)[:, None]
    return t, sig.astype(np.float32), v0.astype(np.float32)


@pytest.fixture
def line(app):
    v = LineView(NS, NCH, t_buffer_s=2.0)
    v.show()
    t, sig, v0 = line_data(1.0 + np.arange(NCH))           # 1..8 mV
    v.set_value_format("mV", None)
    v.update_view(t, sig, v0, 1.0, x_range=(0.0, t[-1]))
    v._render(labels=True)
    yield v
    v.close()


def test_line_auto_is_per_tile(line):
    assert not line.fixed_y
    assert all(auto_y(p) for p in line._tiles)
    spans = [np.diff(yrange(p))[0] for p in line._tiles]
    # tile s holds channels 2s, 2s+1: amplitude up to 2s+2 mV, so the auto ranges differ
    assert spans[-1] > 2.5 * spans[0]
    assert all(abs(lo) < 12 and abs(hi) < 12 for lo, hi in map(yrange, line._tiles))


def test_line_fixed_same_range_everywhere(line):
    line._y_top.setValue(20.0)                              # typing a value switches to fixed
    assert line.fixed_y
    assert line._y_top.text().endswith("mV")
    for p in line._tiles:
        assert not auto_y(p)
        assert yrange(p) == pytest.approx((-20, 20))
    # new data does not move it
    t, sig, v0 = line_data(np.full(NCH, 50.0))
    line.update_view(t, sig, v0, 1.0, x_range=(0.0, t[-1]))
    line._render()
    assert all(yrange(p) == pytest.approx((-20, 20)) for p in line._tiles)
    # single whisker and Link Y keep it
    line._maximize(3)
    assert line._pages.currentIndex() == SINGLE
    assert yrange(line._single_plot.getPlotItem()) == pytest.approx((-20, 20))
    line._restore()
    line._act_link_y.setChecked(True)
    assert all(yrange(p) == pytest.approx((-20, 20)) for p in line._tiles)
    line._act_link_y.setChecked(False)
    assert all(yrange(p) == pytest.approx((-20, 20)) for p in line._tiles)


def test_line_fixed_stacked_rows(line):
    line._act_grid.setChecked(False)
    assert line._pages.currentIndex() == STACKED
    line._y_top.setValue(10.0)
    line._render()
    # channel c is a sine of amplitude c + 1 mV around V0: row offset c // 2, +-(c+1)/20 rows
    for c, curve in enumerate(line._stacked_curves):
        y = curve.getData()[1]
        assert y.mean() == pytest.approx(c // 2, abs=0.02)
        assert (y.max() - y.min()) / 2 == pytest.approx((c + 1) / 20, rel=0.05)
    assert "±10 mV" in line._stacked.getPlotItem().getAxis("left").labelText
    # a Y key scales the fixed value, not a gain
    line._zoom_y(1 / 1.25, True)
    assert line._y_top.value() == pytest.approx(8.0)
    assert line._gain == 1.0


def test_line_back_to_auto(line):
    line._y_top.setValue(20.0)
    line._act_reset.trigger()                                # Reset view keeps it fixed
    assert line.fixed_y and yrange(line._tiles[0]) == pytest.approx((-20, 20))
    line.set_fixed_y(False)
    assert all(auto_y(p) for p in line._tiles)
    line._render()
    assert np.diff(yrange(line._tiles[0]))[0] < 10            # fitted to the +-2 mV tile again
    line._act_grid.setChecked(False)
    line._render()
    y = line._stacked_curves[0].getData()[1]
    assert (y.max() - y.min()) / 2 == pytest.approx(1.0, rel=0.05)   # gain 1: 1 mV = 1 row


def test_line_typing_into_the_box_is_not_a_zoom_key(line):
    edit = line._y_top.lineEdit()
    ev = QKeyEvent(QEvent.Type.KeyPress, Qt.Key.Key_Period, Qt.KeyboardModifier.NoModifier, ".")
    assert line.eventFilter(edit, ev) is False
    assert line.eventFilter(line._tiles[0].getViewBox().scene().views()[0], ev) is True


@pytest.fixture
def spectrum(app):
    v = SpectrumView(NS, NCH)
    v.show()
    f = np.linspace(0, FS / 2, 51)
    amp = np.full((51, NCH), 1e-3)
    amp[10] = 0.5 * (1 + np.arange(NCH))                     # peaks 0.5..4
    v.set_units("mV")
    v.update_view(f, amp)
    v._render(labels=True)
    yield v
    v.close()


def test_spectrum_auto_then_fixed_linear(spectrum):
    assert all(auto_y(p) for p in spectrum._tiles)
    assert yrange(spectrum._tiles[-1])[1] > 2 * yrange(spectrum._tiles[0])[1]
    spectrum.set_fixed_y(True)
    assert all(yrange(p) == pytest.approx((0, 5.0)) for p in spectrum._tiles)
    assert not spectrum._act_y_span.isVisible()
    spectrum._y_top.setValue(2.0)
    assert all(yrange(p) == pytest.approx((0, 2.0)) for p in spectrum._tiles)
    spectrum.set_fixed_y(False)
    assert all(auto_y(p) for p in spectrum._tiles)


def test_spectrum_fixed_db_and_stacked(spectrum):
    spectrum.set_fixed_y(True)
    spectrum._y_top.setValue(2.0)
    spectrum._act_db.setChecked(True)
    assert spectrum._act_y_span.isVisible()
    assert all(yrange(p) == pytest.approx((-60, 20)) for p in spectrum._tiles)
    spectrum._y_top.setValue(10.0)
    spectrum._y_span.setValue(40.0)
    assert all(yrange(p) == pytest.approx((-30, 10)) for p in spectrum._tiles)
    spectrum._zoom_y(1 / 1.25, True)                          # a Y key narrows the dB span
    assert spectrum._fixed_range() == pytest.approx((-22, 10))
    spectrum._act_db.setChecked(False)                        # linear keeps its own top
    assert spectrum._y_top.value() == pytest.approx(2.0)
    spectrum._act_db.setChecked(True)
    assert spectrum._fixed_range() == pytest.approx((-22, 10))
    spectrum._act_grid.setChecked(False)
    spectrum._render()
    # channel 7 peaks at 4 mV = 12 dB, above the 10 dB top: just over a row; the floor sits on the row base
    y = spectrum._stacked_curves[7].getData()[1]
    assert y[10] == pytest.approx(3 + (20 * np.log10(4) + 22) / 32, rel=1e-3)
    assert y[0] == pytest.approx(3.0)
    # rate change (new frequency axis) keeps the fixed range
    spectrum._act_grid.setChecked(True)
    spectrum.update_view(np.linspace(0, 100, 51), spectrum._amp)
    assert all(yrange(p) == pytest.approx((-22, 10)) for p in spectrum._tiles)


@pytest.fixture
def fft(app):
    v = FftView(NCH, 51, 1.0)
    v.show()
    amp = np.full((51, NCH), 1e-3)
    amp[10] = 0.5 * (1 + np.arange(NCH))
    v.update_view(amp, FS)
    yield v
    v.close()


def test_fft_lower_plot_follows_the_colour_range(fft):
    lp = fft._low.getPlotItem()
    assert fft._act_auto.isChecked() and auto_y(lp)
    fft._lo.setValue(0.0)
    fft._hi.setValue(10.0)                                    # typing a range turns Auto off
    assert not fft._act_auto.isChecked()
    assert not auto_y(lp) and yrange(lp) == pytest.approx((0, 10))
    levels = fft._image.getLevels()
    assert tuple(levels) == pytest.approx((0, 10))
    fft.update_view(fft._fft * 3, FS)                         # new data: still fixed
    assert yrange(lp) == pytest.approx((0, 10))
    fft._act_db.setChecked(True)                              # dB has its own fixed range
    assert yrange(lp) == pytest.approx((-60, 20))
    bars = fft._boxes.opts
    assert np.all(np.asarray(bars["y0"]) == -60)              # peak bars rise from the min
    fft._act_auto.setChecked(True)
    assert auto_y(lp)


def _screen_x(widget, plot, x: float) -> float:
    from PyQt6.QtCore import QPointF
    pt = plot.getViewBox().mapViewToScene(QPointF(x, 0))
    return widget.mapToGlobal(widget.mapFromScene(pt)).x()


@pytest.mark.parametrize("db", [False, True])
def test_fft_columns_line_up(app, db):
    """Each channel's bar sits under its heatmap column, also after the labels change width."""
    v = FftView(32, 51, 1.0)
    v.resize(1100, 760)
    v.show()
    amp = np.full((51, 32), 1e-3)
    amp[10] = 100 * (1 + np.arange(32))                       # wide colour-bar / bar labels
    v.update_view(amp, 1000.0)
    v._act_db.setChecked(db)
    for _ in range(3):
        app.processEvents()
    hp, lp = v._heat.getPlotItem(), v._low.getPlotItem()
    for c in (0, 15, 31):
        assert _screen_x(v._heat, hp, c + 0.5) == pytest.approx(_screen_x(v._low, lp, c), abs=1.5)
    v.close()
