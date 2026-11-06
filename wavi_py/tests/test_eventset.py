"""Event sets across recordings: the dome map per recording (replay/dome_maps.py),
the cmd source of the store, EventQuery / EventSet / saved comparisons / StoreCache /
event_selections (replay/eventset.py), and the Dataset's event additions
(daq/analysis.py: alignment, baseline and reference drift, positions, aggregation,
the trace cap).

Run: python -m pytest wavi_py/tests/test_eventset.py
"""

from __future__ import annotations

import copy
import json
import sys
import threading
import types
from datetime import datetime
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from test_manoeuvres import BASE_MV, write_rec  # noqa: E402
from wavi_py.daq import analysis as an  # noqa: E402
from wavi_py.daq.series import IMU, WSK, ArraySeries  # noqa: E402
from wavi_py.replay import dome_maps as dm  # noqa: E402
from wavi_py.replay import eventset as evs  # noqa: E402
from wavi_py.replay import manoeuvres as mn  # noqa: E402
from wavi_py.replay.index import RecordingIndex  # noqa: E402
from wavi_py.replay.store import RecordingStore  # noqa: E402
from whisker_link.channels import CHANNEL_NAMES  # noqa: E402
from whisker_link.dome_map import DEFAULT_MAP, DomeMap, load_calibration  # noqa: E402


# ----- dome maps

def test_history_and_lookup_by_time():
    ids = [m.id for m in dm.history()]
    assert ids == ["2026-10-29", "2026-11-03-3x", "2026-11-03-2x", "2026-11-04-3x-cut-120",
                   "2026-11-04-3x-cut-72", "2026-11-05"]
    at = {s: dm.dome_map_for(f"/x/{s}_run.jsonl").id for s in (
        "st_2026-10-30_1200_00.00", "st_2026-11-03_1000_00.00", "st_2026-11-03_1616_00.00",
        "st_2026-11-04_1430_00.00", "st_2026-11-04_1503_00.00", "st_2026-11-05_0900_00.00")}
    assert list(at.values()) == ["2026-10-29", "2026-11-03-3x", "2026-11-03-2x",
                                 "2026-11-04-3x-cut-120", "2026-11-04-3x-cut-72", "2026-11-05"]
    assert dm.map_at(datetime(2026, 1, 1)).id == "2026-10-29"       # before the history
    # a Pi name
    assert dm.dome_map_for("/x/whisker_20261103_170000.jsonl").whisker_set == "2x"
    m = dm.map_at(datetime(2026, 11, 3, 17))
    assert m.position("LFU").length == 70.0 and m.position("LBL").length == 80.0
    assert m.position("LBU").length == 102.0 and m.reference is None
    old = dm.map_at(datetime(2026, 10, 30))
    assert old.whisker_set == "unknown" and old.position("LFM").length_known is None


def test_sensor_ids_and_the_reference():
    ids = {"LFU": 1, "LFM": 2, "LFL": 3, "LBU": 4, "LBM": 5, "LBL": 6,
           "RFU": 7, "RFM": 8, "RFL": 9, "RBU": 10, "RBM": 11, "RBL": 12}
    for m in dm.history():
        want = dict(ids, RFU=14) if m.id == "2026-11-05" else ids
        assert {p.key: int(p.sensor_id) for p in m.positions} == want
        wiring = {p.key: {a: (g.ch, g.polarity) for a, g in p.daq.items()} for p in m.positions}
        assert wiring == {p.key: {a: (g.ch, g.polarity) for a, g in p.daq.items()}
                          for p in dm.history()[-1].positions}               # same wiring
    ref = dm.history()[-1].reference
    assert sorted(CHANNEL_NAMES[c] for c in ref.gauges) == ["W10_B", "W14_B"]


def test_live_map_is_the_newest_history_entry():
    live = json.loads(DEFAULT_MAP.read_text())
    newest = json.loads(sorted((DEFAULT_MAP.parent / "maps").glob("*.json"))[-1].read_text())
    for d in (live, newest):
        d.pop("description", None)
        d.pop("calibration_file", None)
    assert live == newest


def _rec_with_map(path: Path, m: DomeMap) -> None:
    write_rec(path)
    lines = path.read_bytes().split(b"\n", 1)
    path.write_bytes(lines[0] + b"\n" + dm.dome_map_line(m) + lines[1])


def test_dome_map_line_round_trip_and_embedded_wins(tmp_path):
    other = dm.map_at(datetime(2026, 11, 3, 12))          # 3x, not the 11-05 entry
    line = dm.dome_map_line(other)
    ev = json.loads(line)
    assert ev["k"] == "dome_map" and ev["id"] == other.id and "t" not in ev
    back = dm.from_line(ev)
    assert back.id == other.id and back.whisker_set == "3x"
    assert [(p.key, p.sensor_id, p.length) for p in back.positions] == \
           [(p.key, p.sensor_id, p.length) for p in other.positions]
    plain = tmp_path / "st_2026-11-05_1200_00.00_plain.jsonl"
    withm = tmp_path / "st_2026-11-05_1200_00.00_map.jsonl"
    write_rec(plain)
    _rec_with_map(withm, other)
    assert dm.dome_map_for(plain).id == "2026-11-05"          # by time
    assert dm.dome_map_for(withm).id == other.id              # embedded wins
    # the readers ignore the line: the same session zero, scans and store
    a, b = RecordingIndex.build(plain), RecordingIndex.build(withm)
    assert a.t0 == b.t0 and a.n_wsk == b.n_wsk and a.duration == b.duration
    sa, sb = RecordingStore.build(a), RecordingStore.build(b)
    assert np.array_equal(sa.wsk, sb.wsk) and np.array_equal(sa.imu, sb.imu)
    assert a.rov().series("rov:fwd")[1].tolist() == b.rov().series("rov:fwd")[1].tolist()
    from whisker_link import convert
    ca, cb = convert.load(plain), convert.load(withm)
    assert len(ca["wsk"]["t"]) == len(cb["wsk"]["t"]) and cb["hello"]["k"] == "hello"


def test_start_tee_writes_the_map_after_the_hello(tmp_path):
    from wavi_py.daq.server_reader import ServerReader
    obj = types.SimpleNamespace(_tee_lock=threading.Lock(), _tee=None, _last_t=500.0,
                                hello={"k": "hello", "t": 1.0, "channels": CHANNEL_NAMES})
    p = tmp_path / "tee.jsonl"
    m = DomeMap.load(DEFAULT_MAP)
    ServerReader.start_tee(obj, str(p), m)
    obj._tee.close()
    lines = p.read_bytes().splitlines()
    assert json.loads(lines[0])["k"] == "hello" and json.loads(lines[0])["t"] == 500.0
    assert json.loads(lines[1])["k"] == "dome_map" and json.loads(lines[1])["id"] == m.id


# ----- the per-axis ROV variables (demand, thrust)

def test_rov_axis_variables(tmp_path):
    """The pilot's demand per axis and the thrust projected from the thrusters are
    ROV variables (``replay/rov.py``, from the index), the same keys everywhere."""
    rec = tmp_path / "st_2026-11-05_1200_00.00_cmd.jsonl"
    first = write_rec(rec)
    idx = RecordingIndex.build(rec)
    rov = idx.rov()
    keys = [k for k, *_ in rov.variables()]
    for k in ("rov:fwd", "rov:th_surge", "rov:th_sway", "rov:th_heave", "rov:th_yaw"):
        assert k in keys
    assert [v.key for v in an.rov_axis_variables()] == [
        "rov:fwd", "rov:lat", "rov:vert", "rov:yaw",
        "rov:th_surge", "rov:th_sway", "rov:th_heave", "rov:th_yaw"]
    on = lambda t: (t > 7.0 - first) & (t < 11.0 - first)            # noqa: E731
    t, u = rov.series("rov:fwd")
    off = (t > 1.0) & (t < 5.0 - first)
    assert np.allclose(u[on(t)], 0.5) and np.allclose(u[off], 0.0)
    t, th = rov.series("rov:th_surge")
    assert np.allclose(th[on(t)], 0.5, atol=0.01)
    assert np.allclose(rov.series("rov:th_yaw")[1][on(t)], 0.0, atol=0.01)
    # the Dataset reads them as ROV variables (unit groups pilot / thrust)
    v = an.variable_of("rov:fwd")
    assert v.source == an.ROV and v.unit == "pilot"
    assert an.variable_of("rov:th_surge").unit == "thrust"
    st = RecordingStore.build(idx)
    ds = an.Dataset([an.Selection(1, 0.0, st.duration, an.SelSource(st, rov=rov))], [v])
    tr = ds.traces(200)
    assert len(tr) == 1 and np.nanmax(tr[0].y) == pytest.approx(0.5)


# ----- the query

def _two_days(tmp_path):
    recs = [tmp_path / "rec" / "st_2026-11-03_1200_00.00_a.jsonl",
            tmp_path / "rec" / "st_2026-11-05_1200_00.00_b.jsonl"]
    recs[0].parent.mkdir()
    for r in recs:
        write_rec(r)
    return recs, tmp_path / "side"


def test_query_filters_groups_and_curation(tmp_path):
    recs, side = _two_days(tmp_path)
    q = evs.EventQuery(recordings=[str(recs[0].parent)], sidecar_dir=str(side),
                       types=["onset"], classes=["single:surge+"], level_min=0.5,
                       modes=["ALT_HOLD"], baseline_clean=True, group_by="whisker_set")
    es = evs.EventSet.from_query(q)                  # builds the sidecars into side/
    assert sorted(p.name for p in side.glob("*.manoeuvres.json")) == \
           [r.name.replace(".jsonl", ".manoeuvres.json") for r in recs]
    assert not list(recs[0].parent.glob("*.npz"))    # no caches next to them
    assert list(es.groups.items()) == [("3x", 1), ("3x-cut-72", 1)]
    e = es.events[0]
    assert e.type == "onset" and e.level == 0.5 and e.mode == "ALT_HOLD" and e.baseline_clean
    assert e.map_id == "2026-11-03-3x" and es.events[1].map_id == "2026-11-05"
    assert e.hold_s == pytest.approx(6.0, abs=0.1) and e.baseline_t is not None
    # filters that drop them
    for kw in ({"level_min": 0.6}, {"modes": ["MANUAL"]}, {"hold_min_s": 7.0},
               {"whisker_sets": ["2x"]}, {"depth_min_cm": 60.0}, {"classes": ["single:surge-"]}):
        assert len(evs.EventSet.from_query(evs.EventQuery(**{**q.to_dict(), **kw}))) == 0, kw
    assert len(evs.EventSet.from_query(evs.EventQuery(**{**q.to_dict(), "map_ids": ["2026-11-05"]}))) == 1
    assert len(evs.EventSet.from_query(evs.EventQuery(**{**q.to_dict(), "depth_min_cm": 40.0,
                                                         "alive": "all", "minor": False}))) == 2
    # curation
    q.exclude(e.stem, e.id)
    q.pin(es.events[1].stem, es.events[1].id)
    es2 = evs.EventSet.from_query(evs.EventQuery.from_json(q.to_json()))
    assert [r.key for r in es2.excluded] == [e.key] and len(es2) == 1
    assert es2.events[0].pinned and list(es2.groups.items()) == [("3x", 0), ("3x-cut-72", 1)]
    rows = es2.rows()
    assert len(rows) == 2 and {r["excluded"] for r in rows} == {True, False}
    assert rows[0]["LBM_drag_steady"] is not None or rows[1]["LBM_drag_steady"] is not None
    q.exclude(e.stem, e.id, False)
    assert q.excludes == []
    # other groupings
    for g, want in (("run", 2), ("day", 2), ("level", 1), ("mode", 1), ("none", 1)):
        assert len(evs.EventSet.from_query(evs.EventQuery(**{**q.to_dict(), "group_by": g})).groups) == want
    with pytest.raises(ValueError):
        evs.EventQuery.from_dict({"group_by": "colour"})


def test_saved_comparisons(tmp_path, monkeypatch):
    monkeypatch.setenv("WAVI_COMPARISONS_DIR", str(tmp_path / "cmp"))
    q = evs.EventQuery(recordings=["/a"], types=["onset"], level_min=0.5, pins=[["s", 3]])
    assert evs.list_comparisons() == []
    evs.save_comparison("surge onsets / clean", q)
    assert evs.list_comparisons() == ["surge onsets _ clean"]
    assert evs.load_comparison("surge onsets / clean") == q
    assert evs.delete_comparison("surge onsets / clean") and evs.list_comparisons() == []
    assert not evs.delete_comparison("nope")


def test_store_cache_lru(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    monkeypatch.setenv("LOCALAPPDATA", str(tmp_path / "cache"))
    recs, side = _two_days(tmp_path)
    c = evs.StoreCache(cap_bytes=1, disk=None)
    a = c.get(recs[0])
    assert c.get(recs[0]) is a and c.opened == 1
    c.get(recs[1])
    assert len(c) == 1 and recs[1] in c and recs[0] not in c     # over the cap: the older out
    big = evs.StoreCache(disk="user")
    big.get(recs[0])
    big.get(recs[1])
    assert len(big) == 2 and big.nbytes > 0
    assert not list(recs[0].parent.glob("*.npz"))                 # the user cache only
    assert len(list((tmp_path / "cache").rglob("*.store.npz"))) == 2


# ----- event selections and the Dataset

def test_two_recordings_aligned(tmp_path):
    recs, side = _two_days(tmp_path)
    q = evs.EventQuery(recordings=[str(r) for r in recs], sidecar_dir=str(side),
                       types=["onset"], classes=["single:surge+"], group_by="run")
    es = evs.EventSet.from_query(q)
    sels = evs.event_selections(es, evs.StoreCache(disk=None))
    assert len(sels) == 2 and sels[0].source is not sels[1].source
    assert [s.number for s in sels] == [1, 2] and sels[0].group != sels[1].group
    for s, r in zip(sels, es.events):
        assert s.zero == pytest.approx(r.t) and s.align == pytest.approx(r.t - s.t0)
        assert s.dome_map.id == r.map_id
    u = an.variable_of("rov:fwd")
    lbm = an.variable_of("pos:LBM:drag")
    ds = an.Dataset(sels, [u, lbm], baseline=True)
    tr = {(t.sel.number, t.var.key): t for t in ds.traces(400)}
    for n in (1, 2):
        t, y = tr[(n, u.key)].t, tr[(n, u.key)].y
        assert t[0] == pytest.approx(-2.0, abs=0.05)
        assert y[t < -0.1].max() == 0.0 and y[t > 0.1].min() == 0.5      # the step at 0
        t, y = tr[(n, lbm.key)].t, tr[(n, lbm.key)].y
        assert abs(y[t < -0.1]).max() < 0.01 and y[(t > 0.1) & (t < 4)].min() == pytest.approx(1.0, abs=0.01)
    ag = ds.aggregate(lbm)
    assert len(ag) == 2 and all(a.events == 1 for a in ag)
    for a in ds.aggregate(u):                 # RC: 20 /s on a 50 /s grid, held between
        inside = (a.t > -1.9) & (a.t < 4.9)
        assert np.all(a.n[inside] == 1)
        assert np.all(a.median[inside & (a.t > 0.1)] == 0.5)


def _gauge_series(dur=10.0, fs=250.0):
    """32 gauges: column c holds 100 + c mV."""
    t = np.arange(0, dur, 1 / fs)
    y = np.tile(100.0 + np.arange(32), (len(t), 1))
    return t, y


def test_positions_resolve_through_each_map():
    t, y = _gauge_series()
    src = an.SelSource(ArraySeries({WSK: (t, y)}, {WSK: CHANNEL_NAMES}))
    m1 = DomeMap.load(DEFAULT_MAP)
    doc = copy.deepcopy(m1.doc)
    # the other map: LBM's sensor moved to RBU's slots (a slot swap), drag flipped
    lbm, rbu = doc["positions"]["LBM"]["daq"], doc["positions"]["RBU"]["daq"]
    doc["positions"]["LBM"]["daq"] = {"drag": {"ch": rbu["drag"] if isinstance(rbu["drag"], str)
                                               else rbu["drag"]["ch"], "polarity": -1},
                                      "lift": rbu["lift"]}
    doc["positions"]["RBU"]["daq"] = lbm
    m2 = DomeMap.from_dict(doc, load_calibration(DEFAULT_MAP.parent / "whisker_calibration.json"))
    v = an.variable_of("pos:LBM:drag")
    s1 = an.Selection(1, 1.0, 5.0, src, dome_map=m1)
    s2 = an.Selection(2, 1.0, 5.0, src, dome_map=m2)
    ds = an.Dataset([s1, s2], [v])
    c1, p1 = ds.gauge(s1, v)
    c2, p2 = ds.gauge(s2, v)
    assert CHANNEL_NAMES[c1] == CHANNEL_NAMES[m1.position("LBM").daq["drag"].ch]
    assert CHANNEL_NAMES[c2] == CHANNEL_NAMES[m1.position("RBU").daq["drag"].ch] and p2 == -1
    y1 = ds.samples(s1, v)[1]
    y2 = ds.samples(s2, v)[1]
    assert np.allclose(y1, p1 * (100.0 + c1)) and np.allclose(y2, -(100.0 + c2))
    # without a selection map: the dataset's; neither: no gauge
    s3 = an.Selection(3, 1.0, 5.0, src)
    assert ds.gauge(s3, v) is None
    assert an.Dataset([s3], [v], dome_map=m2).gauge(s3, v) == (c2, -1)


def test_baseline_and_reference_drift():
    fs = 250.0
    t = np.arange(0, 20, 1 / fs)
    y = np.full((len(t), 32), 200.0)
    m = DomeMap.load(DEFAULT_MAP)
    drift = 0.05 * t                                    # mV, on every gauge
    y += drift[:, None]
    g = m.position("LFM").daq["drag"]
    y[:, g.ch] += np.where(t >= 10.0, 2.0, 0.0)         # the response
    src = an.SelSource(ArraySeries({WSK: (t, y)}, {WSK: CHANNEL_NAMES}))
    sel = an.Selection(1, 8.0, 15.0, src, align=2.0, baseline=(9.0, 10.0), dome_map=m)
    v = an.variable_of("pos:LFM:drag")
    tt, yy = an.Dataset([sel], [v], baseline=True).samples(sel, v)
    rel = tt - sel.zero
    # baseline gone, the drift removed by the reference (a ramp: the low-pass keeps it)
    assert abs(np.median(yy[(rel > -1) & (rel < 0)])) < 0.01
    assert np.median(yy[(rel > 2) & (rel < 4)]) == pytest.approx(2.0 * g.polarity, abs=0.03)
    # without the reference: the drift stays
    m0 = dm.map_at(datetime(2026, 11, 4, 16))
    sel0 = an.Selection(1, 8.0, 15.0, src, align=2.0, baseline=(9.0, 10.0), dome_map=m0)
    _, y0 = an.Dataset([sel0], [v], baseline=True).samples(sel0, v)
    assert np.median(y0[(rel > 2) & (rel < 4)]) == pytest.approx(g.polarity * (2.0 + 0.05 * 3.5), abs=0.03)
    # baseline off: V0 and scale as before
    ds = an.Dataset([sel], [an.variable_of("wsk:0")], v0=np.full(32, 200.0), scale=2.0)
    assert ds.samples(sel, ds.variables[0])[1][0] == pytest.approx(2.0 * 0.05 * 8.0, abs=0.01)


def test_aggregate_known_responses():
    fs = 250.0
    t = np.arange(0, 60, 1 / fs)
    amps = {"A": [1.0, 2.0, 3.0], "B": [10.0, 30.0]}
    y = np.zeros((len(t), 2))
    sels = []
    onset = 5.0
    for g, aa in amps.items():
        for a in aa:
            y[:, 0] += np.where(t >= onset, a * (1 - np.exp(-(t - onset) / 0.2)), 0.0) * (t < onset + 5)
            sels.append((g, onset))
            onset += 10.0 + 0.013 * len(sels)          # off the grid
    src = an.SelSource(ArraySeries({WSK: (t, y)}, {WSK: ["W1_A", "W1_B"]}))
    ss = [an.Selection(i + 1, o - 2, o + 4, src, align=2.0, group=g) for i, (g, o) in enumerate(sels)]
    v = an.variable_of("wsk:0")
    ds = an.Dataset(ss, [v], baseline=True, max_traces=2)
    ag = {a.group: a for a in ds.aggregate(v, dt=0.05)}
    assert list(ag) == ["A", "B"] and ag["A"].events == 3 and ag["B"].events == 2
    late = ag["A"].t > 2.0
    assert np.allclose(ag["A"].median[late], 2.0, atol=0.02)
    assert np.allclose(ag["A"].q25[late], 1.5, atol=0.02) and np.allclose(ag["A"].q75[late], 2.5, atol=0.02)
    assert np.allclose(ag["B"].median[late], 20.0, atol=0.2)
    assert np.allclose(ag["A"].median[ag["A"].t < -0.05], 0.0, atol=1e-6)
    i = np.searchsorted(ag["A"].t, 0.2)                 # one tau: 63 %
    assert ag["A"].median[i] == pytest.approx(2.0 * (1 - np.exp(-ag["A"].t[i] / 0.2)), abs=0.1)
    assert ag["A"].n.max() == 3
    # drawn: capped, aggregates still on all
    assert len({t.sel for t in ds.traces(100)}) == 2


def test_drawn_cap_pinned_first_then_spread():
    src = an.SelSource(ArraySeries({WSK: _gauge_series(30.0)}, {WSK: CHANNEL_NAMES}))
    ss = [an.Selection(i + 1, i, i + 1, src, pinned=(i == 7)) for i in range(20)]
    ds = an.Dataset(ss, [an.variable_of("wsk:0")], max_traces=5)
    got = [s.number for s in ds.drawn()]
    assert len(got) == 5 and 8 in got and got == sorted(got)
    assert got[0] == 1 and got[-1] == 20
    assert len(an.Dataset(ss, [], max_traces=None).drawn()) == 20


def test_variable_keys():
    assert an.variable_of("imu:0").source == IMU
    assert an.variable_of("pos:RFU:lift").pos == "RFU" and an.variable_of("pos:RFU:lift").axis == "lift"
    assert an.variable_of("depth").source == an.DEPTH
    assert an.variable_of("pos:RFU") is None and an.variable_of("rov:nope") is None
    assert [v.key for v in an.position_variables(["LFU"])] == ["pos:LFU:drag", "pos:LFU:lift"]
    assert an.variable_of("rov:th_heave").label == "thrust heave"
