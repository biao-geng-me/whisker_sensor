"""Despike (daq/despike.py): the batch test and the streaming one on synthetic
signals (spikes found, steps / ramps / a fast zigzag / plain noise left alone),
the index's spikes, the replay transport's despike switch, the GUI's checkbox,
and the live path (provisional samples patched, the .dat written once final).

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_despike.py
"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from wavi_py.daq.despike import HALF, LAG, Despiker, despike, find_spikes  # noqa: E402
from wavi_py.replay.index import K, RecordingIndex  # noqa: E402
from wavi_py.tests import replay_fixture as fx  # noqa: E402
from wavi_py.transports.replay_transport import ReplayTransport  # noqa: E402

SIG = 0.05                       # mV, the gauges' white noise


def signal(n=5000, seed=0):
    """(n, 4): noise on a slow sine; a step; a ramp; a ±20 sigma zigzag burst.
    Returns the signal and the spikes put on it {(row, col)}."""
    rng = np.random.default_rng(seed)
    t = np.arange(n) / 250.0
    x = rng.normal(0, SIG, (n, 4)) + 260.0
    x[:, 0] += 2.0 * np.sin(2 * np.pi * 0.5 * t)
    x[n // 2:, 1] += 3.0                                      # a 60 sigma step
    x[1000:1100, 2] += np.linspace(0, 5, 100)                 # a fast ramp (2 sigma / sample)
    x[1100:, 2] += 5.0
    x[3000:3200, 3] += 1.0 * (-1) ** np.arange(200)           # thrust zigzag, as in pilot2
    for (i, c), d in SPIKES.items():
        x[i, c] += d
    return x, set(SPIKES)


SPIKES = {(700, 0): 1.0, (701, 0): -0.8, (2200, 1): 0.6, (4000, 2): -1.5,
          (4100, 3): 0.7, (4101, 3): 0.7, (4102, 3): 0.7, (1500, 3): 0.8}


def test_batch_finds_the_spikes_only():
    x, spikes = signal()
    rows, cols, vals = find_spikes(x)
    assert set(zip(rows.tolist(), cols.tolist())) == spikes
    y = despike(x)
    ref = x.copy()
    for (i, c), d in SPIKES.items():
        ref[i, c] -= d                        # the spike-free signal
    for i, c in spikes:                       # back within a few sigma of it
        assert abs(y[i, c] - ref[i, c]) < 6 * SIG
    mask = np.ones_like(x, bool)
    for i, c in spikes:
        mask[i, c] = False
    assert np.array_equal(y[mask], x[mask])   # nothing else touched


def test_plain_noise_and_constant_untouched():
    rng = np.random.default_rng(3)
    x = rng.normal(0, SIG, (50_000, 3))
    x[:, 2] = 8388607 * 1.43e-4               # a railed (constant) slot
    assert len(find_spikes(x)[0]) == 0        # 8 sigma: none in 150 k Gaussian samples
    assert len(find_spikes(x[:5])[0]) == 0


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_streaming_matches(seed):
    """Blocks of 1..40 samples: every final value as the batch has it (the windows
    are the same; only the noise is the last second instead of a centred one),
    the newest `pending` provisional and raw. The zigzag burst's onset included."""
    x, spikes = signal(seed=seed)
    rng = np.random.default_rng(seed)
    d = Despiker()
    final = np.full_like(x, np.nan)
    shown = np.full_like(x, np.nan)            # the caller's buffer, patched as it goes
    i = 0
    while i < len(x):
        n = int(rng.integers(1, 41))
        y = d.push(x[i:i + n])
        n = len(x[i:i + n])
        n_back = len(y) - n
        shown[i - n_back:i + n] = y
        assert d.pending <= max(LAG, n_back + n)
        assert np.array_equal(y[len(y) - d.pending:], x[i + n - d.pending:i + n])  # raw
        final[i - n_back:i + n - d.pending] = y[:len(y) - d.pending]
        i += n
    assert d.pending == LAG and np.isnan(final[-LAG:]).all()
    assert not np.isnan(final[:-LAG]).any()
    ref = despike(x)
    assert np.array_equal(final[:-LAG], ref[:-LAG])
    assert np.array_equal(shown[:-LAG], ref[:-LAG])
    d.reset()
    assert d.pending == 0


def test_streaming_one_by_one():
    x = signal()[0][:1500]
    d = Despiker()
    final = np.full_like(x, np.nan)
    for k in range(len(x)):
        y = d.push(x[k:k + 1])
        nb = len(y) - 1
        final[k - nb:k + 1 - d.pending] = y[:len(y) - d.pending]
    assert np.array_equal(final[:-LAG], despike(x)[:-LAG])


# ----------------------------------------------------------------- replay

@pytest.fixture
def spk(tmp_path):
    p = tmp_path / "spk.jsonl"
    fx.write_recording(p, spikes=True)
    return RecordingIndex.build(p)


def _scans(tr: ReplayTransport) -> dict:
    """{seq: raw} of the scans served now."""
    buf = b""
    for _ in range(100000):
        b = tr.read_some(65536)
        if not b:
            break
        buf += b
    return {e["seq"]: e["raw"] for e in map(json.loads, buf.splitlines()) if e["k"] == "wsk"}


def near_sine(v: float, i: int, c: int) -> bool:
    """``v`` (mV) is within the fixture's sine over scans i-3..i+3 (the median of 7)."""
    ref = [fx.raw_of(5000 + j)[c] * fx.MV for j in range(i - HALF, i + HALF + 1)]
    return min(ref) - 0.01 <= v <= max(ref) + 0.01


def test_index_spikes(spk, tmp_path):
    assert spk.n_spikes == len(fx.SPIKES)
    assert np.all(spk.kind[spk.spk_line] == K["wsk"])
    assert set(spk.spk_ch.tolist()) == {c for _, c, _ in fx.SPIKES}
    # cached, and the summary says so
    p = spk.path
    again = RecordingIndex.open(p)
    again2 = RecordingIndex.open(p)
    assert again2.from_cache and np.array_equal(again2.spk_line, spk.spk_line)
    assert np.allclose(again2.spk_val, spk.spk_val)
    assert f"{len(fx.SPIKES)} spikes" in again.summary()


def test_transport_switch(spk):
    tr = ReplayTransport(spk, speed=None, playing=True, preroll_s=0.0)
    try:
        raw = _scans(tr)
    finally:
        tr.close()
    tr = ReplayTransport(spk, speed=None, playing=True, preroll_s=0.0, despike=True)
    try:
        dsp = _scans(tr)
        assert tr.despike
    finally:
        tr.close()
    assert raw.keys() == dsp.keys()
    diff = {(q - 5000, c) for q in raw for c in range(32) if raw[q][c] != dsp[q][c]}
    assert diff == {(i, c) for i, c, _ in fx.SPIKES}
    for i, c, _ in fx.SPIKES:
        assert isinstance(dsp[5000 + i][c], int)                  # still integer codes
        assert near_sine(dsp[5000 + i][c] * fx.MV, i, c)
    # switching re-serves from the playhead (a seek)
    tr = ReplayTransport(spk, speed=None, playing=False, start_s=2.0, preroll_s=1.0)
    try:
        _scans(tr)
        g = tr.gen
        tr.despike = True
        assert tr.gen == g + 1
        evs = []
        b = tr.read_some(65536)
        evs = [json.loads(x) for x in b.splitlines()]
        assert evs[0]["k"] == "replay" and evs[0]["op"] == "seek"
        assert abs(evs[0]["pos"] - 2.0) < 1e-6
        tr.despike = True                     # no change, no seek
        assert tr.gen == g + 1
    finally:
        tr.close()


# ----------------------------------------------------------------- GUI

@pytest.fixture(scope="module")
def app():
    from PyQt6.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


def test_gui_replay_checkbox(app, tmp_path):
    from wavi_py.main_window import MainWindow
    from wavi_py.tests.test_replay_gui import _pump

    p = tmp_path / "spk.jsonl"
    fx.write_recording(p, spikes=True)
    w = MainWindow(transport="replay", nsensor=16, fs=80.0, t_buffer_s=2.0, replay_speed=4.0,
                   replay_path=str(p))
    w.show()
    try:
        w.open_replay(str(p))
        assert _pump(app, lambda: w._client is not None and w._replay.active, 10)
        assert not w._despike_chk.isChecked() and w._despiker is None   # replay: raw
        assert not w._client.replay.despike
        w._despike_chk.setChecked(True)
        assert w._client.replay.despike
        out = w.export_range(str(tmp_path / "x.dat"), (0.0, 2.0))
        rows = out.read_text().splitlines()
        v = np.array([[float(x) for x in r[23:].split()] for r in rows])
        i, c, _ = fx.SPIKES[0]
        assert near_sine(v[i, c], i, c)                                  # despiked
        w._despike_chk.setChecked(False)
        assert not w._client.replay.despike
    finally:
        w.close()


def test_gui_live_patch_and_record(app, tmp_path):
    """Live path, driven directly: the buffer tail is patched a tick later, and the
    .dat gets every sample once, final, in order."""
    from wavi_py.io.dat_writer import DatWriter
    from wavi_py.main_window import MainWindow

    w = MainWindow(transport="sim", nsensor=16, fs=250.0, t_buffer_s=2.0)
    try:
        assert w._despike_chk.isChecked() and w._despiker is not None
        x, spikes = signal()
        x = np.concatenate([x[:3000]] * 8, axis=1).astype(np.float32)     # 32 gauges
        spikes = {(i, c) for i, c in spikes if i < 3000}
        assert w._nch == 32
        w._set_rate(250.0)
        w._t0_unix = 1.79e9
        w._dat_writer = DatWriter(tmp_path / "live.dat", 32)
        w._is_recording, w._rec_from = True, 0
        for k in range(0, 1500, 12):
            w._ingest(x[k:k + 12], k)
        w._despike_chk.setChecked(False)           # flushes the provisional ones to the .dat
        assert w._despiker is None
        for k in range(1500, 3000, 12):
            w._ingest(x[k:k + 12], k)
        w._dat_writer.close()
        vals = np.array([[float(v) for v in r[23:].split()]
                         for r in (tmp_path / "live.dat").read_text().splitlines()])
        assert len(vals) == 3000
        ref = despike(x[:1500].astype(np.float64))
        hit = [(i, c) for i, c in spikes if i < 1500 - LAG]
        for i, c in hit:
            assert abs(vals[i, c] - ref[i, c]) < 1e-3 and abs(vals[i, c] - x[i, c]) > 0.3
        late = [(i, c) for i, c in spikes if i >= 1500]
        for i, c in late:                          # despike off: raw
            assert abs(vals[i, c] - x[i, c]) < 1e-3
        assert np.allclose(w._sig[-1], x[-1], atol=1e-4)
    finally:
        w._dat_writer = None
        w._is_recording = False
        w.close()


def test_despike_with_skipped_line(tmp_path):
    """A line the index skips (no "k") right after a spike's scan rides in that scan's
    byte span; serving it despiked decodes the scan's line only."""
    p = tmp_path / "skip.jsonl"
    fx.write_recording(p, spikes=True, truncated=False)
    i0 = fx.SPIKES[0][0]
    out = []
    for line in p.read_bytes().splitlines(keepends=True):
        out.append(line)
        if line.startswith(b'{"k":"wsk"') and json.loads(line)["seq"] == 5000 + i0:
            out.append(b'{"note":"no kind"}\n')
    p.write_bytes(b"".join(out))
    idx = RecordingIndex.build(p)
    tr = ReplayTransport(idx, speed=None, playing=True, preroll_s=0.0, despike=True)
    try:
        buf = b""
        for _ in range(100000):
            b = tr.read_some(65536)
            if not b:
                break
            buf += b
    finally:
        tr.close()
    assert b'{"note":"no kind"}' in buf                       # passed on as it was
    dsp = {e["seq"]: e["raw"] for e in map(json.loads, buf.splitlines()) if e.get("k") == "wsk"}
    for i, c, _ in fx.SPIKES:
        assert near_sine(dsp[5000 + i][c] * fx.MV, i, c)
