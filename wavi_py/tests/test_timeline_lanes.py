"""The replay timeline's ROV lanes (views/replay_bar.py): the menu lists the
recording's telemetry, a choice adds one lane per unit under the strip (x linked,
playhead, values on hover), is kept for new timelines, and a recording without
the telemetry shows none.

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_timeline_lanes.py
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
from PyQt6.QtWidgets import QApplication  # noqa: E402

from wavi_py.replay.index import RecordingIndex  # noqa: E402
from wavi_py.tests import replay_fixture as fx  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


class _Src:
    """Just what the bar reads of a source."""

    def __init__(self, idx):
        self._idx = idx
        self.duration, self.events, self.armed_spans = idx.duration, idx.events, idx.armed_spans
        self.unix0 = 0

    def depth(self):
        return self._idx.depth()

    def rov(self):
        return self._idx.rov()


def _bar(app, idx):
    from wavi_py.views.replay_bar import ReplayModel, TimelineBar
    m = ReplayModel()
    bar = TimelineBar(m)
    m.set_live(_Src(idx) if idx is not None else None)
    return m, bar


def _menu_actions(menu):
    out = {}
    for a in menu.actions():
        if a.menu() is not None:
            for b in a.menu().actions():
                out[b.text()] = b
        else:
            out[a.text()] = a
    return out


def test_rov_lanes(app, tmp_path, monkeypatch):
    monkeypatch.setenv("WAVI_SETTINGS_INI", str(tmp_path / "settings.ini"))
    p = tmp_path / "rov.jsonl"
    fx.write_recording(p, rov=True)
    idx = RecordingIndex.build(p)
    m, bar = _bar(app, idx)
    assert not bar.lanes and bar.rov_btn.text() == "ROV ▾"
    acts = _menu_actions(bar.lanes_menu())
    assert "pilot forward" in acts and "flight mode / gain" in acts
    acts["pilot forward"].setChecked(True)                 # applied at once
    assert list(bar.lanes) == ["pilot"]
    lane = bar.lanes["pilot"]
    assert lane.curves[0][0] == "rov:fwd" and len(lane.curves[0][3]) > 10
    assert lane.getPlotItem().getViewBox().linkedView(0) is bar.strip.getPlotItem().getViewBox()
    bar.set_rov_keys(["rov:fwd", "rov:lat", "rov:current", "rov:mode"])
    assert list(bar.lanes) == ["mode", "pilot", "A"]      # one per unit, the band first
    assert [c[0] for c in bar.lanes["pilot"].curves] == ["rov:fwd", "rov:lat"]
    assert bar.rov_btn.text() == "ROV (3) ▾"
    t, fwd = idx.rov().series("rov:fwd")
    x = float(t[len(t) // 2])
    assert f"pilot forward: {fwd[len(t) // 2]:.3g}" in bar.lanes["pilot"].text_at(x)
    assert "mode" in bar.lanes["mode"].text_at(x)
    m.set_live_position(x)
    assert all(abs(ln.head.value() - x) < 1e-9 for ln in bar.lanes.values())
    if os.environ.get("WAVI_TEST_SHOTS"):
        bar.resize(1100, bar.sizeHint().height())
        bar.show()
        app.processEvents()
        bar.grab().save(str(Path(os.environ["WAVI_TEST_SHOTS"]) / "timeline_lanes.png"))
        bar.hide()
    # a new timeline starts on the last choice
    _m2, bar2 = _bar(app, idx)
    assert bar2.rov_keys == ["rov:fwd", "rov:lat", "rov:current", "rov:mode"]
    assert list(bar2.lanes) == ["mode", "pilot", "A"]
    # without the telemetry: no lanes, the choice kept
    q = tmp_path / "plain.jsonl"
    fx.write_recording(q)
    idx2 = RecordingIndex.build(q)
    m.set_live(_Src(idx2))
    assert not bar.lanes and bar.rov_keys
    assert "No ROV telemetry in this source" in _menu_actions(bar.lanes_menu())
    bar.set_rov_keys([])
    m.set_live(_Src(idx))
    assert not bar.lanes
