"""whisker_link.dome_map: loading, validation, and the pose model's inputs."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from whisker_link import dome_map as dm
from whisker_link import dome_pose as dp
from whisker_link.channels import CHANNEL_NAMES, NCH, OUT_OF, channels_of_selects

ASSET = Path(__file__).resolve().parents[2] / "wavi_py" / "assets" / "dome" / "whiskers.json"

# the ROV dome as the user gave it (whisker slot -> position, X = no whisker)
USER = ("W1 LBL, W2 LBM, W3 X, W4 LBU, W5 LFL, W6 LFM, W7 LFU, W8 X, W9 RFL, W10 X, "
        "W11 RFM, W12 RFU, W13 RBL, W14 X, W15 RBM, W16 RBU")


def doc(**top):
    d = {"format": dm.MAP_FORMAT, "positions": {
        "LFU": {"daq": {"drag": "W7_A", "lift": {"ch": "W7_B", "polarity": -1}},
                "sensor_id": "S1", "root": [40, 30, 145], "axis": [1, 0, 0]},
        "RBM": {"daq": {"drag": "W15A", "lift": "W15_B"}, "root": [-56, 0, 116],
                "axis": [-1, 0, 0]},
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


def ch(name):
    return CHANNEL_NAMES.index(name)


def test_default_map_is_the_users_table():
    m = dm.DomeMap.load(dm.DEFAULT_MAP)
    want = dict(s.split() for s in USER.split(", "))
    got = {}
    for p in m.positions:
        assert p.slots.startswith("W")                    # both axes from one slot
        w = p.slots
        assert (p.daq["drag"].name, p.daq["lift"].name) == (f"{w}_A", f"{w}_B")
        got[w] = p.key
    assert got == {w: pos for w, pos in want.items() if pos != "X"}
    assert all(p.sensor_id is None for p in m.positions)
    assert m.uncalibrated() == [p.key for p in m.positions]
    names = m.whisker_names()
    assert names[6] == "LFU (W7)" and names[2] == "W3"
    g = m.gauge_names()
    assert g[ch("W7_A")] == "LFU drag (W7_A)" and g[ch("W7_B")] == "LFU lift (-W7_B)"
    assert g[ch("W9_B")] == "RFL lift (W9_B)" and g[ch("W3_A")] == "W3 A"


def test_default_map_unused_gauges_and_the_select_6_slots():
    """Every gauge is used or listed unused; the select-6 gauges (dead on mux 1,
    skipped by selects 0123457) are unused, and every used gauge is scanned."""
    m = dm.DomeMap.load(dm.DEFAULT_MAP)
    used = set(m.gauge_owner())
    assert used | set(m.unused) == set(range(NCH)) and not used & set(m.unused)
    sel6 = {OUT_OF[a][6] - 1 for a in range(4)}
    assert {CHANNEL_NAMES[c] for c in sel6} == {"W3_B", "W8_B", "W10_A", "W14_A"}
    assert sel6 <= set(m.unused)
    assert used <= set(channels_of_selects(0b10111111))
    alive = {CHANNEL_NAMES[c] for c in set(m.unused) - sel6}
    assert alive == {"W3_A", "W8_A", "W10_B", "W14_B"}
    assert all(m.unused[ch(n)].startswith("alive") for n in alive)
    assert {p.daq["drag"].ch // 2 + 1 for p in m.positions} == set(dp.LIVE_WHISKERS)


def test_default_map_geometry_matches_the_cad_holes():
    """Position codes vs the CAD holes in the ROV frame: L = port (+x), F = the
    column nearer the pole (higher z), U = +y; frames the asset's."""
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


def test_default_polarity_flips_port_lift():
    """The hole frame's e2 = axis x e1 points down on port and up on starboard;
    the port lift gauges are flipped, so with identical sensors (gauge B along
    e2 everywhere) the per-hole matrix comes out the same on both sides."""
    m = dm.DomeMap.load(dm.DEFAULT_MAP)
    for p in m.positions:
        port = p.key[0] == "L"
        assert m.lift_sign(p) == (-1 if port else 1)
        assert p.daq["lift"].polarity == (-1 if port else 1)
        assert p.daq["drag"].polarity == 1
    assert np.allclose(np.degrees(m.calibration().matrix), np.eye(2) * dp.DEG_PER_MV)


def test_pipeline_polarity_sensor_matrix_and_frame(tmp_path):
    """raw -> tare -> polarity -> sensor 2x2 -> (drag, lift) -> dome frame."""
    cal = {"format": dm.CAL_FORMAT, "sensors": {
        "S1": {"deg_per_mv": [[2.0, 0.5], [0.0, 3.0]], "date": "2026-11-02", "source": "test"},
        "spare": {"deg_per_mv": [1.0, 1.0]}}}
    m = dm.DomeMap.load(write(tmp_path, doc(), cal))
    assert m.calibration_path == (tmp_path / "cal.json").resolve()
    assert m.uncalibrated() == ["RBM"]
    assert m.sensor_cal(m.positions[0]).date == "2026-11-02"
    lay = m.layout([0, 0, 119], 57.0)
    assert lay.labels == ["LFU (W7)", "RBM (W15)"]
    hc = m.hole_channels()
    assert hc.tolist() == [[ch("W7_A"), ch("W7_B")], [ch("W15_A"), ch("W15_B")]]
    pose = dp.DomePose(lay, calibration=m.calibration(), hole_channels=hc, tare_n=4,
                       smooth_tau_s=0.0)
    base = np.full((8, NCH), 100.0)
    bend = base.copy()
    bend[:, ch("W7_A")] += 1.0
    bend[:, ch("W7_B")] += 2.0
    bend[:, ch("W15_B")] += 1.0
    pose.feed(base, 250.0)
    pose.feed(bend, 250.0)
    th = np.degrees(pose.angles())
    # LFU: polarity (1, -1) -> (1, -2); S1 -> drag 2 - 1 = 1, lift -6 (down);
    # port: e2 points down (lift_sign -1), so theta_e2 = +6
    assert m.lift_sign(m.positions[0]) == -1
    assert np.allclose(th[0], [1.0, 6.0])
    assert np.allclose(lay.e2[0], [0, -1, 0])
    # RBM (starboard, no sensor): lift +1 mV -> placeholder gain, e2 up
    assert m.lift_sign(m.positions[1]) == 1
    assert np.allclose(th[1], [0.0, dp.DEG_PER_MV])


def test_gauges_from_different_slots_and_a_dead_axis():
    d = doc()
    d["positions"]["LFU"]["daq"] = {"drag": "W1_B", "lift": "W2A"}
    d["positions"]["RBM"]["daq"] = {"drag": "W2_B", "lift": {"ch": None, "reason": "gauge dead"}}
    m = dm.DomeMap.from_dict(d)
    assert m.positions[0].display == "LFU (W1_B/W2_A)"
    assert m.positions[1].display == "RBM (W2)"
    assert m.whisker_names()[:3] == ["LFU (W1)", "LFU/RBM (W2)", "W3"]
    assert m.positions[1].daq["lift"].reason == "gauge dead"
    assert m.hole_channels()[1].tolist() == [ch("W2_B"), -1]
    lay = m.layout([0, 0, 119], 57.0)
    pose = dp.DomePose(lay, calibration=m.calibration(), hole_channels=m.hole_channels(),
                       tare_n=2, smooth_tau_s=0.0)
    base = np.full((4, NCH), 50.0)
    bend = base.copy()
    bend[:, ch("W2_B")] += 1.0
    pose.feed(base, 250.0)
    pose.feed(bend, 250.0)
    th = np.degrees(pose.angles())
    assert np.allclose(th[1], [dp.DEG_PER_MV, 0.0])      # dead lift reads as zero


def test_e1_default_and_override():
    d = doc()
    d["positions"]["LFU"]["e1"] = [1, 1, 0]                      # made normal to the axis
    d["positions"]["RBM"]["axis"] = [0, 0, 2]                    # along +z: default e1 = +y
    m = dm.DomeMap.from_dict(d)
    assert np.allclose(m.positions[0].e1, [0, 1, 0])
    assert np.allclose(m.positions[1].axis, [0, 0, 1])
    assert np.allclose(m.positions[1].e1, [0, 1, 0])


def test_labels_and_lengths():
    d = doc(whisker_length_mm=100)
    d["positions"]["LFU"].update(label="port top", length_mm=90)
    m = dm.DomeMap.from_dict(d)
    assert [p.length for p in m.positions] == [90.0, 100.0]
    assert m.positions[0].display == "port top (W7)"
    assert m.positions[0].gauge_display("lift") == "port top lift (-W7_B)"
    assert m.positions[1].label == "RBM"


def _bad(d, match):
    with pytest.raises(dm.DomeMapError, match=match):
        dm.DomeMap.from_dict(d)


def _set(axis_or_key, value, pos="RBM"):
    d = doc()
    if axis_or_key in dm.AXES:
        d["positions"][pos]["daq"][axis_or_key] = value
    else:
        d["positions"][pos][axis_or_key] = value
    return d


def test_validation_errors():
    _bad(_set("drag", "W7_B"), r"'RBM' drag: gauge W7_B is also LFU lift")
    _bad(_set("drag", "W17_A"), r"'W17_A' is not a gauge channel")
    _bad(_set("drag", "W15"), r"name one gauge")
    _bad(_set("lift", {"ch": "W15_B", "polarity": 2}), r"polarity must be 1 or -1")
    _bad(_set("lift", {"ch": "W15_B", "polarity": True}), r"polarity must be 1 or -1")
    _bad(_set("lift", {"ch": None}), r"needs a 'reason'")
    _bad(_set("lift", None), r"a gauge name or")
    _bad(_set("daq", {"drag": "W15_A"}), r"'RBM' lift: missing")
    _bad(_set("daq", {"drag": "W15_A", "lift": "W15_B", "yaw": "W16_A"}), r"'yaw'")
    _bad(_set("daq", {"drag": {"ch": None, "reason": "x"}, "lift": {"ch": None, "reason": "y"}}),
         r"both axes have no channel")
    _bad(_set("sensor_id", "S1"), r"sensor_id 'S1' is also at position 'LFU'")
    _bad(doc(unused_gauges={"W15_B": "dead"}), r"W15_B is also listed in unused_gauges")
    _bad(doc(unused_gauges={"W3_B": "dead", "W3B": "dead"}), r"listed twice")
    _bad(_set("sensorid", "S2"), r"unknown field\(s\) 'sensorid'")
    d = doc()
    del d["positions"]["RBM"]["axis"]
    _bad(d, r"position 'RBM' has no 'axis'")
    _bad(_set("axis", [0, 0, 0]), r"axis is zero")
    _bad(_set("root", [0, 0]), r"root must be \[x, y, z\]")
    _bad(_set("e1", [-2, 0, 0]), r"parallel")
    _bad(doc(format="wavi-dome-map/1"), r"format must be")
    _bad(doc(positions={}), r"non-empty")
    _bad(doc(whisker_length_mm=-1), r"must be > 0")


def test_duplicate_keys_and_bad_files(tmp_path):
    text = json.dumps(doc())
    p = tmp_path / "dup.json"
    p.write_text(text.replace('"RBM"', '"LFU"'))
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
