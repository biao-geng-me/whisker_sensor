"""Dome map in the main window: the plot views' whisker names follow it.

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_dome_map_labels.py
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from PyQt6.QtWidgets import QApplication, QMessageBox  # noqa: E402

from wavi_py.app import _parse_args  # noqa: E402
from whisker_link.dome_map import DEFAULT_MAP, DomeMap  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


def _ticks(axis):
    return [t for _, t in axis._tickLevels[0]] if axis._tickLevels else []


def test_main_window_names_follow_the_map(app, tmp_path, monkeypatch):
    from wavi_py.main_window import MainWindow

    mw = MainWindow(transport="sim", nsensor=16, outpath=tmp_path, dome_map=DomeMap.load(DEFAULT_MAP))
    try:
        lv, fv, sv = mw._line_view, mw._fft_view, mw._spec_view
        assert _ticks(lv._stacked.getAxis("left"))[6] == "LFU (W7)"
        assert lv._tiles[2].titleLabel.text == "W3"
        assert lv._tiles[0].titleLabel.text == "LBL (W1)"
        assert mw._spectrum_view._names[15] == "RBU (W16)"
        assert _ticks(fv._heat.getPlotItem().getAxis("bottom"))[6] == "LFU (W7)"
        assert sv._chan.itemText(14) == "LFU lift (-W7_B)"
        assert sv._chan.itemText(17) == "RFL drag (W9_A)"
        assert fv._gauge_names[12] == "LFU drag (W7_A)"
        assert mw._map_btn.text() == "Map: dome_map"
        assert "12 positions, 0 calibrated" in mw._map_btn.toolTip()
        lv._maximize(6)
        assert lv.windowTitle().endswith("LFU (W7)")
        mw.set_dome_map(None)
        assert _ticks(lv._stacked.getAxis("left"))[6] == "W7"
        assert sv._chan.itemText(14) == "W7 B" and mw._map_btn.text() == "Map"
        assert fv._gauge_names[12] == "W7 A"
        # a bad file: a warning, the map unchanged
        bad = tmp_path / "bad.json"
        bad.write_text('{"format": "wavi-dome-map/2", "positions": {}}')
        shown = []
        monkeypatch.setattr(QMessageBox, "warning", lambda *a: shown.append(a[2]))
        mw._load_dome_map(bad)
        assert shown and "non-empty" in shown[0] and mw._dome_map is None
        mw._load_dome_map(DEFAULT_MAP)
        assert mw._dome_map is not None and mw._dome_map.path == DEFAULT_MAP
    finally:
        mw.close()


def test_cli_dome_map(capsys):
    a = _parse_args(["--transport", "sim", "--dome-map", str(DEFAULT_MAP)])
    assert [p.key for p in a.dome_map.positions][:3] == ["LBL", "LBM", "LBU"]
    assert _parse_args([]).dome_map.path == DEFAULT_MAP          # the ROV dome by default
    assert _parse_args(["--dome-map", "none"]).dome_map is None
    with pytest.raises(SystemExit):
        _parse_args(["--dome-map", "/nonexistent/map.json"])
    assert "cannot read" in capsys.readouterr().err
