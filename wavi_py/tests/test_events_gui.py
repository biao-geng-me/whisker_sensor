"""The Analysis window's Events mode (views/event_panel.py, event_cells.py,
event_loader.py, analysis_window.py), the timeline's manoeuvre lane, the
recordings browser's Events..., and the stage-1 follow-ups (the reference drift
over the whole recording, alive_known, no_reference, the sidecar fallback dir).

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_events_gui.py
"""

from __future__ import annotations

import os
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from PyQt6.QtWidgets import QApplication, QMessageBox  # noqa: E402

from test_manoeuvres import write_rec  # noqa: E402
from wavi_py.daq import analysis as an  # noqa: E402
from wavi_py.daq.series import WSK, ArraySeries  # noqa: E402
from wavi_py.replay import dome_maps as dm  # noqa: E402
from wavi_py.replay import eventset as evs  # noqa: E402
from wavi_py.replay import manoeuvres as mn  # noqa: E402
from wavi_py.replay import recname  # noqa: E402
from whisker_link.channels import CHANNEL_NAMES  # noqa: E402
from whisker_link.dome_map import DEFAULT_MAP, DomeMap  # noqa: E402

SHOTS = os.environ.get("WAVI_TEST_SHOTS")


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture(autouse=True)
def no_modal_errors(monkeypatch, tmp_path):
    def critical(parent, title, text, *a, **k):
        raise AssertionError(f"{title}: {text}")
    monkeypatch.setattr(QMessageBox, "critical", critical)
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    monkeypatch.setenv("LOCALAPPDATA", str(tmp_path / "cache"))
    monkeypatch.setenv("WAVI_COMPARISONS_DIR", str(tmp_path / "cmp"))


def _pump(app, pred, secs: float) -> bool:
    t_end = time.monotonic() + secs
    while time.monotonic() < t_end:
        app.processEvents()
        if pred():
            return True
        time.sleep(0.01)
    app.processEvents()
    return pred()


def _shot(widget, name: str) -> None:
    if SHOTS:
        Path(SHOTS).mkdir(parents=True, exist_ok=True)
        widget.grab().save(str(Path(SHOTS) / f"{name}.png"))


def _recs(tmp_path):
    """Two recordings on days with different dome maps (11-03 3x, 11-05)."""
    d = tmp_path / "rec"
    d.mkdir(exist_ok=True)
    recs = [d / "st_2026-11-03_1200_00.00_a.jsonl", d / "st_2026-11-05_1200_00.00_b.jsonl"]
    for r in recs:
        if not r.exists():
            write_rec(r)
    return recs, tmp_path / "side"


def _variables():
    return (an.whisker_variables(32, [f"S{i // 2 + 1}" for i in range(32)]) + an.imu_variables()
            + an.rov_axis_variables() + an.position_variables(an.DOME_ORDER))


def _events_window(app, query, cache=None):
    from wavi_py.views.analysis_window import EVENTS, AnalysisWindow
    from wavi_py.views.selections import SelectionModel
    model = SelectionModel()
    w = AnalysisWindow(model, _variables, 1, None, None, mode=EVENTS, query=query,
                       cache=cache if cache is not None else evs.StoreCache(disk=None))
    w._test_model = model
    w.show()
    assert _pump(app, lambda: w.overlay is not None and not w.loader.busy, 60), \
        w.events_panel.status.text()
    app.processEvents()
    return w


def _query(recs, side, **kw):
    return evs.EventQuery(**{"recordings": [str(r) for r in recs], "sidecar_dir": str(side),
                             "types": ["onset"], "classes": ["single:surge+"],
                             "group_by": "run", **kw})


# ----- stage-1 follow-ups

def test_reference_drift_is_over_the_whole_recording():
    """The overlay's reference drift is the whole recording's low-passed mean: the
    same for any window over it (the sidecar's numbers), cached per series."""
    fs = 250.0
    t = np.arange(0, 40, 1 / fs)
    y = np.full((len(t), 32), 200.0)
    m = DomeMap.load(DEFAULT_MAP)
    y += (0.05 * t + 0.4 * np.sin(2 * np.pi * 0.05 * t))[:, None]       # slow drift
    ser = ArraySeries({WSK: (t, y)}, {WSK: CHANNEL_NAMES})
    cols = tuple(m.reference.gauges)
    a = an.reference_drift(ser, cols)
    assert an.reference_drift(ser, cols) is a                          # cached
    tt, dd = a
    assert len(tt) == len(t)
    v = an.variable_of("pos:LFM:drag")
    out = []
    for t0, t1 in ((8.0, 15.0), (5.0, 25.0)):
        sel = an.Selection(1, t0, t1, an.SelSource(ser), align=10.0 - t0, baseline=(9.0, 10.0),
                           dome_map=m)
        tt2, yy = an.Dataset([sel], [v], baseline=True).samples(sel, v)
        k = np.searchsorted(tt2, 12.0)
        out.append(yy[k])
    assert out[0] == pytest.approx(out[1], abs=1e-6)                   # window-independent
    assert abs(out[0]) < 0.02


def test_alive_known_and_no_reference(tmp_path):
    recs, side = _recs(tmp_path)
    q = _query(recs, side)
    es = evs.EventSet.from_query(q)
    assert len(es) == 2
    by = {r.map_id: r for r in es.events}
    assert by["2026-11-03-3x"].no_reference and not by["2026-11-05"].no_reference
    rows = {r["map_id"]: r for r in es.rows()}
    assert rows["2026-11-03-3x"]["no_reference"] is True and "lost" in rows["2026-11-05"]
    # undetermined alive: in by default and with "no position lost"; out with alive_known
    docs = {r.recording: mn.read_doc(mn.manoeuvres_path(r.recording, side)) for r in es.events}
    alive = {k: [v["drag"].get("alive") for v in e["positions"].values()]
             for k, d in docs.items() for e in d["events"] if e["type"] == "onset"}
    assert len(evs.EventSet.from_query(_query(recs, side, alive="all"))) == 2
    known = evs.EventSet.from_query(_query(recs, side, alive="all", alive_known=True))
    want = sum(all(a is True for a in v) for v in alive.values())
    assert len(known) == want
    assert evs.EventQuery.from_json(_query(recs, side, alive_known=True).to_json()).alive_known


def test_sidecar_dir_fallback_and_suffix(tmp_path, monkeypatch):
    rec = tmp_path / "st_2026-11-05_1200_00.00_x.jsonl"
    write_rec(rec)
    assert mn.sidecar_dir_for(rec) is None                           # writable: next to it
    monkeypatch.setattr(os, "access", lambda p, mode: False)
    got = mn.sidecar_dir_for(rec)
    assert got is not None and str(got).startswith(str(tmp_path / "cache"))
    (tmp_path / "st_2026-11-05_1200_00.00_x.manoeuvres.json").write_text("{}")
    assert mn.sidecar_dir_for(rec) is None                           # one exists next to it
    assert recname.stem_of("st_2026-11-05_1200_00.00_x.manoeuvres.json") == \
        "st_2026-11-05_1200_00.00_x"


# ----- the window

def test_events_mode_window(app, tmp_path):
    from wavi_py.views.event_cells import C_USE
    recs, side = _recs(tmp_path)
    cache = evs.StoreCache(disk=None)
    w = _events_window(app, _query(recs, side), cache)
    try:
        assert "Events 1" in w.windowTitle()
        assert not w.timeline.isVisible() and w.browser.isVisible()
        assert not w.sel_list.isVisible() and w.events_panel.isVisible()
        # Positions checked by default, drag and lift of all 12
        assert sorted(w.checked_keys()) == sorted(v.key for v in an.position_variables(an.DOME_ORDER))
        ov = w.overlay
        assert len(ov.selections) == 2 and {s.dome_map.id for s in ov.selections} == \
            {"2026-11-03-3x", "2026-11-05"}
        assert w.browser.model.rowCount() == 2
        assert len(w.compare.rows) == 12                                # the dome grid
        assert "2 events" in w.compare.note.text()
        _shot(w, "events_mode")
        # a query change (level filter) re-runs without opening the stores again
        opened = cache.opened
        renders = w.browser.renders
        q = w.events_panel.query
        q.level_min = 0.4
        w.set_query(q)
        assert _pump(app, lambda: w.browser.renders > renders and not w.loader.busy, 30)
        assert cache.opened == opened == 2 and len(w.eventset) == 2
        # selection sync: table -> scatter + waveform; scatter -> table
        k0 = w.browser.model.refs[0].key
        w.browser.table.selectRow(0)
        app.processEvents()
        assert w.sel_keys == [k0] and w.highlighted() == [w._num_of[k0]]
        w._render()
        assert w.compare._hl_items and len(w.browser.sel_marks.data) == 1
        k1 = w.browser.model.refs[1].key
        w.browser.select_in([(-10, -1e6), (10, -1e6), (10, 1e6), (-10, 1e6)])  # everything
        assert sorted(w.sel_keys) == sorted([k0, k1])
        rows = {i.row() for i in w.browser.table.selectionModel().selectedRows()}
        assert rows == {0, 1}
        w._on_wave_event(w._num_of[k1])                                  # a click in the waveform
        assert w.browser.selected == [k1] and w.sel_keys == [k1]
        assert w.picked_event_selection().event.key == k1
        # hover highlights (and leaving clears)
        w.browser.hovered.emit(k0)
        assert set(w.highlighted()) == {w._num_of[k1], w._num_of[k0]}
        w.browser.hovered.emit(None)
        assert w.highlighted() == [w._num_of[k1]]
        # curation: the checkbox excludes; into a saved comparison and back
        row = next(i for i, r in enumerate(w.browser.model.refs) if r.key == k0)
        w.browser.model.setData(w.browser.model.index(row, C_USE), 0, 10)       # CheckStateRole
        assert _pump(app, lambda: not w.loader.busy and len(w.eventset) == 1, 30)
        assert [list(k0)] == w.events_panel.query.excludes
        w.curate("pin", [k1])
        assert _pump(app, lambda: not w.loader.busy and w.eventset.events[0].pinned, 30)
        name = w.events_panel.save_as("two runs / surge")
        back = evs.load_comparison(name)
        assert back.excludes == [list(k0)] and back.pins == [list(k1)] and back.level_min == 0.4
        w.curate("include_all", [])
        assert _pump(app, lambda: not w.loader.busy and len(w.eventset) == 2, 30)
        w.events_panel.load_comparison(name)
        assert _pump(app, lambda: not w.loader.busy and len(w.eventset) == 1, 30)
        # open in the replay: the recording and t
        got = []
        w.openEvent.connect(lambda p, t: got.append((p, t)))
        w.browser.openEvent.emit(k1)
        r1 = w._ref(k1)
        assert got == [(r1.recording, pytest.approx(r1.t))]
        # spectra on the picked event
        ds = w.dataset()
        assert len(ds.selections) == 1 and ds.baseline
    finally:
        w.close()


def test_selections_mode_is_unchanged(app, tmp_path):
    """A Selections window: the timeline, the selections list, no events panel;
    the grids by gauge; the ROV group in the tree; nothing checked by them."""
    from wavi_py.views.analysis_window import AnalysisWindow
    from wavi_py.views.selections import SelectionModel
    model = SelectionModel()
    w = AnalysisWindow(model, _variables, 1)
    w.show()
    app.processEvents()
    try:
        assert w.mode == "selections" and w.events_panel is None and w.browser is None
        assert w.timeline.isVisible() and w.sel_list.isVisible()
        assert "Analysis 1" in w.windowTitle()
        tops = [w.var_tree.topLevelItem(i).text(0) for i in range(w.var_tree.topLevelItemCount())]
        assert tops == ["IMU", "ROV"] and not w.grid_positions
        assert not any(k.startswith(("pos:", "rov:")) for k in w.checked_keys())
    finally:
        w.close()


def test_background_loading_does_not_block(app, tmp_path):
    recs, side = _recs(tmp_path)
    release = threading.Event()

    class Slow(evs.StoreCache):
        def _open(self, rec):
            release.wait(20)
            return super()._open(rec)

    from wavi_py.views.analysis_window import EVENTS, AnalysisWindow
    from wavi_py.views.selections import SelectionModel
    w = AnalysisWindow(SelectionModel(), _variables, 1, None, None, mode=EVENTS,
                       query=_query(recs, side), cache=Slow(disk=None))
    w.show()
    try:
        # the GUI thread keeps running while the stores load
        t0 = time.monotonic()
        ticks = 0
        while time.monotonic() - t0 < 0.6:
            app.processEvents()
            ticks += 1
            time.sleep(0.005)
        assert ticks > 30 and w.loader.busy
        assert _pump(app, lambda: "loading" in w.events_panel.status.text(), 5)
        assert w.eventset is not None and w.browser.model.rowCount() == 2   # the table first
        assert w.overlay is None
        release.set()
        assert _pump(app, lambda: w.overlay is not None and not w.loader.busy, 30)
    finally:
        release.set()
        w.close()


# ----- the lane and Compare similar

def test_lane_from_a_sidecar_and_compare_similar(app, tmp_path):
    from wavi_py.views.analysis_timeline import lane_bars
    from wavi_py.views.analysis_window import AnalysisWindow
    from wavi_py.views.selections import SelectionModel
    recs, side = _recs(tmp_path)
    rec = recs[1]
    path, doc, _ = mn.write_manoeuvres_for(rec, side, cache=False)
    bars = lane_bars(doc)
    segs = [s for s in doc["segments"] if s["class"] != "rest"]
    assert len(bars) >= len([s for s in segs if s["class"] != "disarmed"])
    surge = [b for b in bars if b[3]["class"] == "single:surge+"]
    assert len(surge) == 1 and surge[0][2].color().alpha() == int(70 + 185 * 0.5)
    coast = [b for b in bars if b[3]["class"] == "rest"]            # the release coast, hatched
    assert coast and coast[0][0] == pytest.approx(surge[0][1])
    model = SelectionModel()
    w = AnalysisWindow(model, _variables, 1, manoeuvres_fn=lambda: (str(rec), doc))
    w.show()
    app.processEvents()
    try:
        w._render()
        assert w.timeline._lane_item is not None
        sg = next(s for s in doc["segments"] if s["class"] == "single:surge+")
        ev = w.timeline.lane_event_at(0.5 * (sg["t0"] + sg["t1"]))
        assert ev is not None and ev["type"] == "onset" and ev["_stem"] == doc["stem"]
        got = []
        w.compareSimilar.connect(got.append)
        w._lane_compare(ev)
        q = got[0]
        assert q.types == ["onset"] and q.classes == ["single:surge+"]
        assert q.level_min == pytest.approx(0.4) and q.level_max == pytest.approx(0.6)
        assert q.modes == ["ALT_HOLD"] and q.recordings == [str(rec.parent)]
        assert q.pins == [[doc["stem"], ev["id"]]]
        # its result: both recordings' surge onsets
        q.sidecar_dir = str(side)
        assert len(evs.EventSet.from_query(q)) == 2
    finally:
        w.close()


def test_lane_click_makes_a_selection(app, tmp_path):
    from wavi_py.views.analysis_window import AnalysisWindow
    from wavi_py.views.selections import SelectionModel
    from wavi_py.daq.analysis import SelSource
    from wavi_py.replay.index import RecordingIndex
    from wavi_py.replay.store import RecordingStore
    recs, side = _recs(tmp_path)
    st = RecordingStore.build(RecordingIndex.build(recs[1]))
    model = SelectionModel()
    model.source_fn = lambda: SelSource(st)
    w = AnalysisWindow(model, _variables, 1)
    w.show()
    try:
        w._lane_clicked(4.0, 9.0)
        assert len(model) == 1 and w.included[0].t0 == pytest.approx(4.0)
    finally:
        w.close()


# ----- the recordings browser's Events...

def test_browser_events_on_two_recordings_with_different_maps(app, tmp_path):
    from wavi_py.views.recordings_browser import RecordingsBrowser
    recs, side = _recs(tmp_path)
    got = []
    b = RecordingsBrowser(recs[0].parent, lambda p: None, events_fn=lambda p, n: got.append((p, n)))
    b.show()
    try:
        app.processEvents()
        assert b.events_btn.isVisible()
        t = b._tree()
        t.selectAll()
        rows = b.selected_rows()
        assert sorted(r.stem for r in rows) == sorted(recname.stem_of(r.name) for r in recs)
        b.events_selected()
        assert len(got) == 1 and sorted(got[0][0]) == sorted(str(r) for r in recs) and got[0][1] == ""
    finally:
        b.close()
    # the window on them: two maps, both events
    w = _events_window(app, _query(recs, side))
    try:
        assert {s.dome_map.id for s in w.overlay.selections} == {"2026-11-03-3x", "2026-11-05"}
        assert dm.dome_map_for(recs[0]).id != dm.dome_map_for(recs[1]).id
    finally:
        w.close()


# ----- the main window

def test_main_window_lane_events_and_open_event(app, tmp_path, monkeypatch):
    """Replay open builds the sidecar in the background (a child) and the lane shows;
    Events... opens on the open recording; an event opens seeked."""
    from wavi_py.main_window import MainWindow
    monkeypatch.setenv("WAVI_SIDECAR_BUILD", "1")
    recs, _side = _recs(tmp_path)
    rec = recs[1]
    w = MainWindow(transport="replay", nsensor=16, fs=80.0, replay_path=str(rec), t_buffer_s=2.0)
    w.show()
    w.open_replay(str(rec))
    try:
        assert _pump(app, lambda: w._store is not None, 60)
        assert _pump(app, lambda: w.manoeuvres_doc() is not None, 120)        # the child
        assert mn.manoeuvres_path(rec).exists()                              # next to it
        a = w.new_analysis_window()
        assert _pump(app, lambda: a.timeline._lane_item is not None, 10)
        ev = w.show_events()
        assert ev.mode == "events" and ev.query.recordings == [str(rec)]
        assert _pump(app, lambda: ev.overlay is not None and not ev.loader.busy, 60)
        assert len(ev.eventset) >= 1
        seeks = []
        monkeypatch.setattr(w._replay, "request_seek", lambda t, *a: seeks.append(t))
        r = ev.eventset.events[0]
        ev.openEvent.emit(r.recording, r.t)
        assert seeks == [pytest.approx(max(0.0, r.t - 2.0))]
        ev.close()
        a.close()
        assert w._last_event_scope == [str(rec)]
    finally:
        w.close()


def test_minor_inputs_of_the_round_2_sidecar():
    """Round-2 sidecars give minor inputs as {axis: {max, share}}: the event keeps the
    largest |u| per axis, the "at most" filter and the table's flags read it."""
    from wavi_py.views.event_cells import flags_text
    assert evs._minor_levels({"yaw": {"max": 0.07, "share": 0.1}, "sway": 0.12}) == \
        {"yaw": 0.07, "sway": 0.12}
    assert evs._minor_levels(None) == {}
    r = evs.EventRef.__new__(evs.EventRef)
    for k, v in dict(pinned=False, baseline_clean=True, steady=False, minor={"yaw": 0.07},
                     no_reference=True, alive={"LFM": False}).items():
        object.__setattr__(r, k, v)
    assert flags_text(r) == "clean · minor yaw+0.07 · no ref · lost LFM"


# ----- merged with the Analysis round 3 (grid picker, ROV, mode band)

def test_grid_position_view(app):
    """The grid picker's by-position view: the same cells, halves drag / lift, keys
    pos:<position>:<axis>; gauge and position checks are kept apart."""
    from PyQt6.QtCore import QPointF
    from wavi_py.views.analysis_window import POSITIONS_VIEW, PLOTS, AnalysisWindow
    from wavi_py.views.selections import SelectionModel
    from wavi_py.views.sensor_grid import cell_rects
    m = DomeMap.load(DEFAULT_MAP)
    w = AnalysisWindow(SelectionModel(), _variables, 1, dome_map_fn=lambda: m)
    w.show()
    app.processEvents()
    try:
        g = w.grids[PLOTS]
        w.set_checked(["wsk:10"])
        w.grid_view.setCurrentText(POSITIONS_VIEW)
        assert w.grid_positions and g.positions
        s = g.grid.names.index("LFM")
        assert g.key_of(2 * s) == "pos:LFM:drag" and g.key_of(2 * s + 1) == "pos:LFM:lift"
        assert g.col_of("pos:LFM:lift") == 2 * s + 1 and g.checked == set()
        g.grab()
        _cell, name, _a, b = cell_rects(g.grid, g._area())[s]
        g.toggled.emit([g.key_of(c) for c in g.hit(QPointF(name.center()))], True)
        assert {"pos:LFM:drag", "pos:LFM:lift", "wsk:10"} <= set(w.checked_keys())
        assert g.checked == {2 * s, 2 * s + 1}
        assert g.hit(QPointF(b.center())) == [2 * s + 1]
        # a position whose gauge the map lists dead is dimmed in this view
        dead = [p for p in m.positions if any(int(x.ch) in g.grid.dead for x in p.daq.values())]
        for p in dead:
            sn = g.grid.names.index(p.key)
            assert any(c in g.pos_off for c in (2 * sn, 2 * sn + 1))
        # the dataset reads the positions through the window's map
        ds = w.dataset()
        assert ds.dome_map is m
    finally:
        w.close()


def test_events_mode_switches_and_dome_places(app, tmp_path):
    from wavi_py.views.analysis_window import TIMELINE
    recs, side = _recs(tmp_path)
    w = _events_window(app, _query(recs, side))
    try:
        assert w.grid_positions and not w.grids[TIMELINE].isVisible()
        assert w.var_tree.isColumnHidden(TIMELINE)
        assert not w.color_box.isEnabled() and not w._act_select.isEnabled()
        tops = [w.var_tree.topLevelItem(i).text(0) for i in range(w.var_tree.topLevelItemCount())]
        assert tops == ["IMU", "ROV"]
        # the dome grid's places are the grid picker's (plane map): LB LF | RF RB, U M L
        pl = w.compare._places()
        assert pl["LBU"][0] < pl["LFU"][0] < pl["RFU"][0] < pl["RBU"][0]
        assert pl["LFU"][1] < pl["LFM"][1] < pl["LFL"][1]
        cols = {k: c for k, _r, c, _t in w.compare._grid(w._checked_vars())}
        assert sorted(set(cols.values())) == [0, 1, 2, 3]                # no gap column
        # Zoom box reaches the event panels; demand and thrust per axis plot per event
        w._act_zoom.setChecked(True)
        assert all(p.getViewBox().state["mouseMode"] == p.getViewBox().RectMode
                   for p in w.compare.panels.values()) and w.compare.zoom_box
        w.set_checked(["rov:fwd", "rov:th_surge"])
        assert _pump(app, lambda: not w.loader.busy and w.overlay is not None and
                     "rov:th_surge" in [v.key for v in w.overlay.variables], 30)
        ag = {a.group: a for a in w.overlay.aggregates["rov:fwd"]}
        for a in ag.values():
            assert np.nanmax(a.median[a.t > 0.2]) == pytest.approx(0.5)
    finally:
        w.close()


def test_timeline_order_mode_band_lane_variables(app, tmp_path):
    from wavi_py.views.analysis_timeline import LANE
    from wavi_py.views.analysis_window import AnalysisWindow
    from wavi_py.views.selections import SelectionModel
    from wavi_py.daq.analysis import SelSource, rov_variables
    from wavi_py.replay.index import RecordingIndex
    from wavi_py.replay.store import RecordingStore
    recs, side = _recs(tmp_path)
    rec = recs[1]
    _p, doc, _ = mn.write_manoeuvres_for(rec, side, cache=False)
    idx = RecordingIndex.build(rec)
    src = SelSource(RecordingStore.build(idx), idx.depth(), rov=idx.rov())
    model = SelectionModel()
    w = AnalysisWindow(model, lambda: _variables()[:-24] + rov_variables(idx.rov())
                       + an.position_variables(an.DOME_ORDER), 1, lambda: src,
                       manoeuvres_fn=lambda: (str(rec), doc))
    w.show()
    app.processEvents()
    try:
        w.set_timeline(["rov:mode", "depth", "rov:fwd"])
        w._render()
        units = list(w.timeline.rows)
        assert units[:2] == ["mode", LANE] and set(units[2:]) == {"cm", "pilot"}
    finally:
        w.close()


def test_events_mode_spectra_header(app, tmp_path):
    recs, side = _recs(tmp_path)
    w = _events_window(app, _query(recs, side))
    try:
        assert "no sub-ranges" in w.spectrum.header.sub.text()
        assert "Select (S)" not in w.spectrogram.header.sub.text()
    finally:
        w.close()
