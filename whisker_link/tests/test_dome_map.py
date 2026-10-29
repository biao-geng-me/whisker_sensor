"""whisker_link.dome_map: loading, validation, and the pose model's inputs."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from whisker_link import dome_map as dm
from whisker_link import dome_pose as dp
from whisker_link.channels import CHANNEL_NAMES, NCH, OUT_OF

ASSET = Path(__file__).resolve().parents[2] / "wavi_py" / "assets" / "dome" / "whiskers.json"

# the ROV dome as the user gave it (channel -> position, X = no whisker)
USER = ("W1 LBL, W2 LBM, W3 X, W4 LBU, W5 LFL, W6 LFM, W7 LFU, W8 X, W9 RFL, W10 X, "
        "W11 RFM, W12 RFU, W13 RBL, W14 X, W15 RBM, W16 RBU")


def doc(**top):
    d = {"format": dm.MAP_FORMAT, "positions": {
        "LFU": {"channel": "W7", "sensor_id": "S1", "root": [40, 30, 145], "axis": [1, 0, 0]},
        "RBM": {"channel": "W15", "root": [-56, 0, 116], "axis": [-1, 0, 0]},
    }}
    d.update(top)
    return d


def write(tmp_path, d, cal=None, name="map.json"):
    if cal is not None:
        (tmp_path / "cal.json").write_text(json.dumps(cal))
        d = dict(d, calibration_file="cal.json")
    p = tmp_path / name
    p.write_text(json.dumps(d))
    return p


def test_default_map_is_the_users_table():
    m = dm.DomeMap.load(dm.DEFAULT_MAP)
    want = dict(s.split() for s in USER.split(", "))
    got = {p.channel: p.key for p in m.positions}
    assert got == {ch: pos for ch, pos in want.items() if pos != "X"}
    assert sorted(f"W{w}" for w in m.unused) == sorted(ch for ch, pos in want.items() if pos == "X")
    assert all(p.sensor_id is None for p in m.positions)
    assert m.uncalibrated() == [p.key for p in m.positions]
    names = m.whisker_names()
    assert names[6] == "LFU (W7)" and names[2] == "W3"


def test_default_map_unused_channels_are_the_select_6_whiskers():
    """The X channels are exactly the whiskers with a gauge on mux select 6
    (dead on mux 1, skipped by selects 0123457)."""
    sel6 = {CHANNEL_NAMES[OUT_OF[a][6] - 1].split("_")[0] for a in range(4)}
    m = dm.DomeMap.load(dm.DEFAULT_MAP)
    assert {f"W{w}" for w in m.unused} == sel6
    assert {p.whisker for p in m.positions} == set(dp.LIVE_WHISKERS)


def test_default_map_geometry_matches_the_cad_holes():
    """Position codes vs the CAD holes: L = port (+x), F = the column nearer
    the pole (higher z), U = +y; frames the asset's."""
    asset = dp.DomeLayout.load(ASSET)
    m = dm.DomeMap.load(dm.DEFAULT_MAP)
    lay = m.layout(asset.center, asset.dome_radius)
    assert len(lay) == 12 and lay.dome_radius == asset.dome_radius
    holes = set()
    for i, p in enumerate(m.positions):
        h = int(np.argmin(np.linalg.norm(asset.root - p.root, axis=1)))
        holes.add(h)
        assert np.allclose(asset.root[h], p.root, atol=0.01)
        assert np.allclose(lay.axis[i], asset.axis[h], atol=1e-3)
        assert np.allclose(lay.e1[i], asset.e1[h], atol=1e-3)
        assert np.allclose(lay.e2[i], asset.e2[h], atol=1e-3)
        side, fb, ud = p.key
        assert (p.root[0] > 0) == (side == "L")
        assert (p.root[2] > 130) == (fb == "F")
        assert np.sign(round(p.root[1])) == {"U": 1, "M": 0, "L": -1}[ud]
    assert holes == set(range(12))


def test_pose_uses_map_holes_and_sensor_calibration(tmp_path):
    cal = {"format": dm.CAL_FORMAT, "sensors": {
        "S1": {"deg_per_mv": [[2.0, 0.0], [0.0, -3.0]], "date": "2026-11-02", "source": "test"},
        "spare": {"deg_per_mv": [1.0, 1.0]}}}
    m = dm.DomeMap.load(write(tmp_path, doc(), cal))
    assert m.calibration_path == (tmp_path / "cal.json").resolve()
    assert m.uncalibrated() == ["RBM"]
    assert m.sensor_cal(m.positions[0]).date == "2026-11-02"
    lay = m.layout([0, 0, 119], 57.0)
    pose = dp.DomePose(lay, m.hole_of(), m.calibration(), tare_n=4, smooth_tau_s=0.0)
    assert pose.hole_of == {7: 0, 15: 1}
    base = np.full((8, NCH), 100.0)
    bend = base.copy()
    for ch in ("W7_A", "W7_B", "W15_A"):
        bend[:, CHANNEL_NAMES.index(ch)] += 1.0
    pose.feed(base, 250.0)
    pose.feed(bend, 250.0)
    th = np.degrees(pose.angles())
    assert np.allclose(th[0], [2.0, -3.0])                     # S1's matrix
    assert np.allclose(th[1], [dp.DEG_PER_MV, 0.0])            # no sensor: placeholder gain
    assert lay.labels == ["LFU (W7)", "RBM (W15)"]


def test_e1_default_and_override(tmp_path):
    d = doc()
    d["positions"]["LFU"]["e1"] = [1, 1, 0]                      # made normal to the axis
    d["positions"]["RBM"]["axis"] = [0, 0, 2]                    # along +z: default e1 = +y
    m = dm.DomeMap.from_dict(d)
    assert np.allclose(m.positions[0].e1, [0, 1, 0])
    assert np.allclose(m.positions[1].axis, [0, 0, 1])
    assert np.allclose(m.positions[1].e1, [0, 1, 0])


def test_labels_and_lengths(tmp_path):
    d = doc(whisker_length_mm=100)
    d["positions"]["LFU"].update(label="port top", length_mm=90)
    m = dm.DomeMap.from_dict(d)
    assert [p.length for p in m.positions] == [90.0, 100.0]
    assert m.positions[0].display == "port top (W7)"
    assert m.positions[1].label == "RBM"


def _bad(d, match):
    with pytest.raises(dm.DomeMapError, match=match):
        dm.DomeMap.from_dict(d)


def test_validation_errors():
    d = doc()
    d["positions"]["RBM"]["channel"] = "W7"
    _bad(d, r"channel W7 is also position 'LFU'")
    d = doc()
    d["positions"]["RBM"]["channel"] = "W17"
    _bad(d, r"'W17' is not W1 \.\. W16")
    d = doc()
    d["positions"]["RBM"]["channel"] = "W15_A"
    _bad(d, r"both gauges")
    d = doc()
    d["positions"]["RBM"]["sensor_id"] = "S1"
    _bad(d, r"sensor_id 'S1' is also at position 'LFU'")
    d = doc(unused_channels={"W15": "dead"})
    _bad(d, r"W15 is also listed in unused_channels")
    d = doc()
    d["positions"]["RBM"]["sensorid"] = "S2"
    _bad(d, r"unknown field\(s\) 'sensorid'")
    d = doc()
    del d["positions"]["RBM"]["axis"]
    _bad(d, r"position 'RBM' has no 'axis'")
    d = doc()
    d["positions"]["RBM"]["axis"] = [0, 0, 0]
    _bad(d, r"axis is zero")
    d = doc()
    d["positions"]["RBM"]["root"] = [0, 0]
    _bad(d, r"root must be \[x, y, z\]")
    d = doc()
    d["positions"]["RBM"]["e1"] = [-2, 0, 0]
    _bad(d, r"parallel")
    _bad(doc(format="other"), r"format must be")
    _bad(doc(positions={}), r"non-empty")
    _bad(doc(whisker_length_mm=-1), r"must be > 0")


def test_duplicate_keys_and_bad_files(tmp_path):
    text = json.dumps(doc())
    dup = text.replace('"RBM"', '"LFU"')
    p = tmp_path / "dup.json"
    p.write_text(dup)
    with pytest.raises(dm.DomeMapError, match=r"dup\.json: duplicate key 'LFU'"):
        dm.DomeMap.load(p)
    p.write_text(text[:-3])
    with pytest.raises(dm.DomeMapError, match="not valid JSON"):
        dm.DomeMap.load(p)
    with pytest.raises(dm.DomeMapError, match="cannot read"):
        dm.DomeMap.load(tmp_path / "missing.json")
    # a referenced calibration file must exist and be valid
    with pytest.raises(dm.DomeMapError, match=r"cal\.json: cannot read"):
        dm.DomeMap.load(write(tmp_path, dict(doc(), calibration_file="cal.json")))
    for cal, match in (
            ({"format": dm.CAL_FORMAT, "sensors": {"S1": {"date": "x"}}}, "no deg_per_mv"),
            ({"format": dm.CAL_FORMAT, "sensors": {"S1": {"deg_per_mv": [1, 2, 3]}}}, "deg_per_mv must"),
            ({"format": dm.CAL_FORMAT, "sensors": {"S1": {"deg_per_mv": [1, 1], "gain": 2}}}, "'gain'"),
            ({"format": "x", "sensors": {}}, "format must")):
        with pytest.raises(dm.DomeMapError, match=match):
            dm.DomeMap.load(write(tmp_path, doc(), cal))


def test_shipped_calibration_file_is_valid():
    assert dm.load_calibration(dm.DEFAULT_MAP.parent / "whisker_calibration.json") == {}
