"""The cheap-redraw machinery: batched spectra, the spectrogram ring and its
column timing, the render scheduler, shared axis pictures, the auto-Y fit and
the vectorized outlier filler (offscreen Qt).

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_viz_perf.py
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from PyQt6.QtWidgets import QApplication, QWidget  # noqa: E402

import pyqtgraph as pg  # noqa: E402

from wavi_py.daq.outliers import fill_outliers_linear  # noqa: E402
from wavi_py.daq.spectra import SpecHistory, amp_spectra, amp_spectrum  # noqa: E402
from wavi_py.views.line_view import LineView  # noqa: E402
from wavi_py.views.render_sched import RenderScheduler  # noqa: E402
from wavi_py.views.spectral_base import to_db  # noqa: E402
from wavi_py.views.whisker_plots import _ViewBox, _fit_y  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


def pump(app, secs: float) -> None:
    t_end = time.monotonic() + secs
    while time.monotonic() < t_end:
        app.processEvents()
        time.sleep(0.001)


# ----------------------------------------------------------------- spectra

def test_amp_spectra_batch_matches_single_windows():
    fs, n = 100.0, 200
    t = np.arange(3 * n) / fs
    x = np.column_stack([3.0 * np.sin(2 * np.pi * 10 * t), 250 + 0 * t]).astype(np.float32)
    wins = np.stack([x[i:i + n] for i in (0, 50, 100)])
    batch = amp_spectra(wins)
    for k in range(3):
        assert np.allclose(batch[k], amp_spectrum(wins[k]))
    f = np.fft.rfftfreq(n, 1 / fs)
    assert f[np.argmax(batch[0][:, 0])] == pytest.approx(10.0)
    assert batch[0][:, 0].max() == pytest.approx(3.0, rel=0.02)      # a sine of amplitude a reads a
    assert not batch[0][:, 1].any()                                  # constant (railed): nothing


def test_amp_spectra_gap_window_is_zero():
    x = np.ones((2, 64, 3))
    x[1, 5, 2] = np.nan
    a = amp_spectra(x)
    assert not a[1].any()
    assert amp_spectrum(x[1]) is None


def test_spec_history_ring_order_resize_and_db():
    nfreq, nch = 3, 2
    h = SpecHistory(nfreq * nch, 4)
    frames = np.arange(6 * nfreq * nch, dtype=np.float32).reshape(6, nfreq, nch) + 1
    h.push(frames[:3])
    h.push(frames[3:])                                   # wraps: holds frames 2..5
    d = h.ordered()
    assert d.shape == (4, nfreq * nch)
    for j, k in enumerate(range(2, 6)):
        assert np.array_equal(d[j], frames[k].T.reshape(-1))         # channel-major rows
    assert np.array_equal(h.latest(nfreq), frames[5])
    assert np.allclose(h.ordered(db=True), to_db(d))
    h.push(frames[:1])                                   # the dB copy follows new columns
    assert np.allclose(h.ordered(db=True)[-1], to_db(frames[0].T.reshape(-1)))
    assert np.array_equal(h.ordered(rows=slice(2, 4))[-1], frames[0].T.reshape(-1)[2:4])
    h.resize(6)                                          # newest kept, zero-padded at the old end
    d = h.ordered()
    assert d.shape == (6, nfreq * nch) and not d[:2].any()
    assert np.array_equal(d[-1], frames[0].T.reshape(-1))
    h.resize(2)
    assert np.array_equal(h.ordered()[-1], frames[0].T.reshape(-1))


def test_spectrogram_columns_follow_the_hop(app):
    """One column per hop samples, each the spectrum of the window ending there,
    and the time axis says so (it used to add one column per GUI tick)."""
    from wavi_py.main_window import MainWindow
    mw = MainWindow(transport="sim", nsensor=16, fs=250.0, auto_fs=False)
    try:
        fs, hop = mw._fs, mw._spec_hop
        assert hop == 13 and mw._spec_cols == int(mw._t_spec_s * fs / hop)
        mw._spec_view.show()
        n_tot = 0
        version = mw._spec_hist.version
        for _ in range(40):                              # 40 ticks of 12-13 samples (~2 s)
            n = 12 + (_ % 2)
            t = (n_tot + np.arange(n)) / fs
            block = np.full((n, mw._nch), 250.0, dtype=np.float32)
            block[:, 0] += 5 * np.sin(2 * np.pi * 20 * t)
            mw._ingest(block, n_tot)
            n_tot += n
            if mw._samples_since_spec_update >= hop:
                mw._update_fft_and_spec()
        # columns only once a whole FFT window (250 samples) was buffered
        expected = sum(1 for e in range(hop, n_tot + 1, hop) if e >= mw._n_fft_window)
        assert mw._spec_hist.version - version == pytest.approx(expected, abs=1)
        f = np.fft.rfftfreq(mw._n_fft_window, 1 / fs)
        assert f[np.argmax(mw._fft_map[:, 0])] == pytest.approx(20.0)
        assert mw._spec_view._col_dt == pytest.approx(hop / fs)
        mw._spec_view._redraw()
        v, t0, _h = mw._spec_view._shown
        assert t0 == pytest.approx(-mw._spec_cols * hop / fs)
    finally:
        mw._spec_view.close()
        mw.deleteLater()


# --------------------------------------------------------------- scheduler

class _Counter:
    def __init__(self):
        self.n = 0

    def __call__(self):
        self.n += 1


def test_scheduler_caps_the_rate_and_coalesces(app):
    s = RenderScheduler()
    w = QWidget()
    w.show()
    c = _Counter()
    s.register(w, c, 20.0)
    t_end = time.monotonic() + 0.5
    while time.monotonic() < t_end:                     # requests far faster than 20 Hz
        s.request(w)
        app.processEvents()
        time.sleep(0.002)
    pump(app, 0.06)
    assert 8 <= c.n <= 12
    w.close()


def test_scheduler_keeps_up_with_data_at_its_rate(app):
    """Requests at exactly the window's interval: no frame dropped by timer jitter."""
    s = RenderScheduler()
    w = QWidget()
    w.show()
    c = _Counter()
    s.register(w, c, 20.0)
    for _ in range(20):
        s.request(w)
        pump(app, 0.05)
    assert c.n >= 19
    w.close()


def test_scheduler_skips_hidden_windows(app):
    s = RenderScheduler()
    w = QWidget()
    c = _Counter()
    s.register(w, c, 50.0)
    s.request(w)
    pump(app, 0.1)
    assert c.n == 0                                      # never shown: nothing drawn
    w.show()
    s.request(w)
    pump(app, 0.3)                                       # once exposed (re-checked every 0.1 s)
    assert c.n == 1
    w.close()


def test_scheduler_prefers_the_window_below_its_floor(app):
    s = RenderScheduler()
    fast, slow = QWidget(), QWidget()
    for w in (fast, slow):
        w.show()
    pump(app, 0.05)                                      # exposed
    order = []
    s.register(fast, lambda: order.append("fast"), 30.0, 20.0)
    s.register(slow, lambda: order.append("slow"), 10.0, 2.5)
    now = time.monotonic()
    s._entries[fast].last = now - 0.06                   # both overdue, fast past its floor
    s._entries[slow].last = now - 0.2
    s.request(fast)
    s.request(slow)
    pump(app, 0.02)
    assert order[:2] == ["fast", "slow"]
    for w in (fast, slow):
        w.close()


# ------------------------------------------------------------ line view

NS, NCH, FS = 4, 8, 100.0


@pytest.fixture
def line(app):
    v = LineView(NS, NCH, t_buffer_s=2.0)
    v.resize(800, 600)
    v.show()
    t = np.arange(int(2 * FS)) / FS
    amps = 1.0 + np.arange(NCH)
    sig = (250 + amps[None, :] * np.sin(2 * np.pi * 2 * t)[:, None]).astype(np.float32)
    v.update_view(t, sig, np.full(NCH, 250, np.float32), 1.0, x_range=(0.0, t[-1]))
    v._render(labels=True)
    v.grab()
    yield v
    v.close()


def test_grid_tiles_share_axis_pictures(app, line):
    for _ in range(2):                                   # the left axes align to the widths
        line._render()                                   # measured at the paint before, and
        line.grab()                                      # the layout follows on the next pass
        app.processEvents()
    line.grab()
    keys = [p.getAxis("bottom")._picture_key() for p in line._tiles]
    assert all(k == keys[0] for k in keys), keys
    bottoms = [p.getAxis("bottom").picture for p in line._tiles]
    assert bottoms[0] is not None and all(b is bottoms[0] for b in bottoms)
    lefts = [p.getAxis("left").picture for p in line._tiles]
    assert lefts[0] is not lefts[-1]                     # different Y ranges: own pictures
    line.set_fixed_y(True)
    line._render()
    line.grab()
    app.processEvents()
    lefts = [p.getAxis("left").picture for p in line._tiles]
    assert all(lp is lefts[0] for lp in lefts)           # one fixed range: shared


def test_auto_y_returns_after_a_zoom(line):
    vb = line._tiles[3].getViewBox()
    fitted = tuple(vb.viewRange()[1])
    vb.scaleBy(y=0.25)                                   # a mouse-style zoom: auto off
    assert not vb.autoRangeEnabled()[1]
    line._render()
    assert tuple(vb.viewRange()[1]) != pytest.approx(fitted, rel=0.05)
    vb.enableAutoRange(axis="y")                         # the A button
    line._render()
    assert tuple(vb.viewRange()[1]) == pytest.approx(fitted, rel=0.05)


def test_fit_y_ignores_jitter_but_never_clips(app):
    vb = _ViewBox()
    vb.resize(400, 300)
    vb.enableAutoRange(axis="y")
    _fit_y(vb, -0.95, 0.95)
    fitted = tuple(vb.viewRange()[1])
    assert fitted[0] < -0.95 and fitted[1] > 0.95
    _fit_y(vb, -0.94, 0.93)                              # a 1 % move: left alone
    assert tuple(vb.viewRange()[1]) == fitted
    _fit_y(vb, -0.95, 1.2)                               # data above the view: re-fit at once
    lo, hi = vb.viewRange()[1]
    assert hi >= 1.2 and lo < -0.95
    _fit_y(vb, -0.2, 0.2)                                # much smaller: shrinks
    lo, hi = vb.viewRange()[1]
    assert hi - lo < 0.5


# ------------------------------------------------------------- outliers

def _fill_outliers_loop(x: np.ndarray) -> np.ndarray:
    """The original per-column implementation."""
    out = np.array(x, copy=True)
    n = out.shape[0]
    if n < 3:
        return out
    idx = np.arange(n)
    for c in range(out.shape[1]):
        col = out[:, c]
        med = np.median(col)
        mad = np.median(np.abs(col - med))
        if mad <= 0:
            continue
        bad = np.abs(col - med) > 3.0 * 1.4826 * mad
        if not bad.any() or (~bad).sum() < 2:
            continue
        col[bad] = np.interp(idx[bad], idx[~bad], col[~bad])
    return out


def test_vectorized_outliers_match_the_loop():
    rng = np.random.default_rng(3)
    for trial in range(200):
        x = rng.normal(0, 1, (int(rng.integers(1, 30)), 6)).astype(np.float32)
        x[rng.random(x.shape) < 0.08] += 40
        x[:, 2] = 5.0                                    # constant: passed through
        if trial % 4 == 0:
            x[int(rng.integers(0, len(x))), 4] = np.nan  # a gap: column left alone
        assert np.array_equal(fill_outliers_linear(x), _fill_outliers_loop(x), equal_nan=True)


def test_tiles_show_only_left_and_bottom_axes(line):
    for p in line._tiles:
        assert p.getAxis("left").isVisible() and p.getAxis("bottom").isVisible()
        assert not p.getAxis("top").isVisible() and not p.getAxis("right").isVisible()


def test_curve_bounds_come_with_the_data(line):
    curve = line._grid_curves[5]
    x, y = curve.getData()
    assert curve.dataBounds(0) == pytest.approx((x[0], x[-1]))
    assert curve.dataBounds(1) == pytest.approx((np.nanmin(y), np.nanmax(y)))
    assert isinstance(curve, pg.PlotCurveItem)
