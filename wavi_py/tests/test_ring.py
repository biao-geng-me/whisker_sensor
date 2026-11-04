"""The live whisker buffer's ring (daq/ring.py) against a plain shifted buffer, and
the live ``MainWindow.series()`` snapshot over it.

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_ring.py
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

from wavi_py.daq.ring import Window  # noqa: E402
from wavi_py.daq.series import WSK  # noqa: E402


@pytest.mark.parametrize("slack", [None, 1, 7, 100])
def test_window_is_a_shifted_buffer(slack):
    rng = np.random.default_rng(0)
    n = 50
    ref = np.full((n, 3), np.nan, np.float32)
    ref_t = np.full(n, np.nan)
    r = Window(n, [((3,), np.float32), ((), np.float64)], slack=slack)
    i = 0
    for k in list(rng.integers(0, 20, 300)) + [n, n + 7, 3]:   # incl. blocks of >= n rows
        x = rng.normal(size=(k, 3)).astype(np.float32)
        t = np.arange(i, i + k, dtype=float)
        i += k
        if k >= n:
            ref[:], ref_t[:] = x[-n:], t[-n:]
        elif k:
            ref[:-k], ref[-k:] = ref[k:].copy(), x
            ref_t[:-k], ref_t[-k:] = ref_t[k:].copy(), t
        r.append(x, t)
        assert np.array_equal(r.window(0), ref, equal_nan=True)
        assert np.array_equal(r.window(1), ref_t, equal_nan=True)
        assert r.window(0).flags["C_CONTIGUOUS"]
    assert r.compactions > 0


def test_window_writes_through_and_fill():
    r = Window(10, [((2,), np.float32)])
    r.append(np.ones((4, 2), np.float32))
    r.window(0)[-2:] = 5.0                     # the despike patch writes into the window
    assert r.window(0)[-1, 0] == 5.0 and r.window(0)[-3, 0] == 1.0
    r.fill(np.nan)
    assert np.isnan(r.window(0)).all()


@pytest.fixture(scope="module")
def app():
    from PyQt6.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


def test_live_series_snapshot_and_buffers(app):
    """Sim, live: the buffers fill in order through compactions (a small buffer so
    they happen), series() is sorted, matches the buffer, and a kept snapshot does not
    change as the ring moves on."""
    from wavi_py.main_window import MainWindow
    w = MainWindow(transport="sim", nsensor=16, fs=250.0, t_buffer_s=1.0)
    w._transport_combo.setCurrentText("Sim")
    w._connect()
    try:
        t_end = time.monotonic() + 15
        while time.monotonic() < t_end and w._ring.compactions < 2:
            app.processEvents()
            time.sleep(0.005)
        assert w._ring.compactions >= 2               # (the 5 s minimum buffer: one per 5 s)
        n = w._samples_since_clear
        t = w._sig_t[-n:]
        assert n > 0 and np.all(np.diff(t) > 0)
        s = w.series()
        assert s is w.series()                      # one snapshot per tick
        ts, ys = s.get(WSK, slice(None), -1e9, 1e9)
        assert np.array_equal(ts, t) and np.array_equal(ys, w._sig[-n:], equal_nan=True)
        kept = ts.copy()
        t_end = time.monotonic() + 3
        while time.monotonic() < t_end and w._sig_t[-1] < kept[-1] + 1.5:
            app.processEvents()
            time.sleep(0.005)
        assert np.array_equal(s.get(WSK, slice(None), -1e9, 1e9)[0], kept)   # unchanged
        assert w.series() is not s and w.series().extent(WSK)[1] > kept[-1]
    finally:
        w._disconnect()
        w.close()
