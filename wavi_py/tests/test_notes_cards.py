"""Recording names (replay/recname.py), notes (replay/notes.py), cards
(replay/card.py) and the make_cards tool.

Run: python -m pytest wavi_py/tests/test_notes_cards.py
"""

from __future__ import annotations

import json
import multiprocessing as mp
import os
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from wavi_py.io.dat_writer import build_datalog_filename  # noqa: E402
from wavi_py.replay import card as cardmod  # noqa: E402
from wavi_py.replay import notes, recname  # noqa: E402
from wavi_py.replay.index import RecordingIndex  # noqa: E402
from wavi_py.replay.store import RecordingStore  # noqa: E402
from wavi_py.tests import replay_fixture as fx  # noqa: E402

DIVE = Path("/scratch/bigeme/whisker_dev/pilot2/st_2026-11-02_1644_57.91_in_water_pilot2.jsonl")


# ----- recname

@pytest.mark.parametrize("name,stem,start,tag,source", [
    ("st_2026-11-02_1644_57.91_in_water_pilot2.jsonl", "st_2026-11-02_1644_57.91_in_water_pilot2",
     datetime(2026, 11, 2, 16, 44, 57, 910000), "in_water_pilot2", "laptop"),
    ("st_2026-11-02_1644_57.91_a-b_c.dat", "st_2026-11-02_1644_57.91_a-b_c",
     datetime(2026, 11, 2, 16, 44, 57, 910000), "a-b_c", "laptop"),
    ("st_2026-11-02_0900_05.00_.faults.txt", "st_2026-11-02_0900_05.00_",
     datetime(2026, 11, 2, 9, 0, 5), "", "laptop"),
    ("st_2026-11-02_0900_05.00_v1.2.notes.jsonl", "st_2026-11-02_0900_05.00_v1.2",
     datetime(2026, 11, 2, 9, 0, 5), "v1.2", "laptop"),
    ("/x/y/st_2026-11-02_0900_05.00_t.jsonl.idx.npz", "st_2026-11-02_0900_05.00_t",
     datetime(2026, 11, 2, 9, 0, 5), "t", "laptop"),
    ("whisker_20261102_164455.jsonl", "whisker_20261102_164455",
     datetime(2026, 11, 2, 16, 44, 55), "", "pi"),
    ("whisker_20261102_164455_pool-run_2.card.json", "whisker_20261102_164455_pool-run_2",
     datetime(2026, 11, 2, 16, 44, 55), "pool-run_2", "pi"),
    ("whisker_20261102_164455_x.jsonl.part", "whisker_20261102_164455_x",
     datetime(2026, 11, 2, 16, 44, 55), "x", "pi"),
])
def test_parse(name, stem, start, tag, source):
    r = recname.parse(name)
    assert r is not None
    assert (r.stem, r.start, r.tag, r.source) == (stem, start, tag, source)
    assert recname.stem_of(name) == stem
    assert r.date == "2026-11-02"


@pytest.mark.parametrize("name", ["series.npz", "quick.txt", "trials.jsonl", "st_2026-13-02_0900_05.00_x.dat",
                                  "whisker_2026110_164455.jsonl", "st_2026-11-02_0900_5.00_x.dat", ""])
def test_parse_not_a_recording(name):
    assert recname.parse(name) is None


def test_roundtrip_dat_writer_name():
    dt = datetime(2026, 11, 3, 7, 5, 9, 123456)
    for tag in ("", "pool_run-3", "a b/c"):
        r = recname.parse(build_datalog_filename(dt, tag))
        assert r is not None and r.source == "laptop"
        assert abs((r.start - dt).total_seconds()) < 0.01


def test_same_recording():
    lap = "st_2026-11-02_1644_57.91_in_water_pilot2.jsonl"
    assert recname.same_recording(lap, "whisker_20261102_164455_in_water_pilot2.jsonl")
    assert recname.same_recording("whisker_20261102_164500.jsonl", lap)      # 2.09 s later
    assert not recname.same_recording(lap, "whisker_20261102_164453.jsonl")  # 4.9 s before
    assert recname.same_recording(lap, lap.replace(".jsonl", ".dat"))
    # two of the laptop's own recordings: different names are different recordings
    assert not recname.same_recording(lap, "st_2026-11-02_1644_58.50_other.jsonl")
    assert not recname.same_recording(lap, "notes.txt")
    assert not recname.same_recording(None, lap)


def test_sidecar():
    p = Path("/d/st_2026-11-02_1644_57.91_t.jsonl")
    assert notes.notes_path(p) == Path("/d/st_2026-11-02_1644_57.91_t.notes.jsonl")
    assert cardmod.card_path(p.with_suffix(".dat")) == Path("/d/st_2026-11-02_1644_57.91_t.card.json")


# ----- notes

def test_notes_append_read_state(tmp_path):
    p = tmp_path / "st_2026-11-02_1644_57.91_t.notes.jsonl"
    assert notes.read(p) == [] and notes.load(p).count == 0 and not notes.load(p).bad
    a = notes.append(p, notes.note("touch on W3", t=12.5, by="me"))
    b = notes.append(p, notes.note("whole thing: pool test", by="me"))
    notes.append(p, notes.flag(False, "W7 loose", by="me"))
    lines = notes.read(p)
    assert [ln["ev"] for ln in lines] == ["note", "note", "flag"]
    assert all(len(ln["id"]) == 12 and ln["v"] == 1 for ln in lines)
    st = notes.load(p)
    assert st.bad and st.reason == "W7 loose" and st.count == 2
    assert [n["id"] for n in st.notes] == [a["id"], b["id"]]          # pinned first
    assert [n["id"] for n in st.pinned] == [a["id"]]
    # edit = del + new; unmark = keep true
    notes.append(p, notes.delete(a["id"]))
    c = notes.append(p, notes.note("touch on W3, hard", t=12.5))
    notes.append(p, notes.flag(True))
    st = notes.load(p)
    assert not st.bad and st.flag["keep"] is True
    assert {n["id"] for n in st.notes} == {b["id"], c["id"]}
    # utf-8 text
    notes.append(p, notes.note("Δt ≈ 0.2 s — ok"))
    assert notes.load(p).notes[-1]["text"] == "Δt ≈ 0.2 s — ok"
    # one JSON object per line, nothing else
    raw = p.read_bytes().decode("utf-8").splitlines()
    assert all(json.loads(s)["id"] for s in raw)


def test_notes_torn_last_line(tmp_path):
    p = tmp_path / "n.notes.jsonl"
    notes.append(p, notes.note("one"))
    with open(p, "ab") as f:
        f.write(b'{"v":1,"id":"abc","ev":"note","te')         # a crash mid-write
    assert [n["text"] for n in notes.read(p)] == ["one"]
    notes.append(p, notes.note("two"))                         # starts on a new line
    assert [n["text"] for n in notes.read(p)] == ["one", "two"]


def test_flag_newest_by_unix_wins():
    old = dict(notes.flag(False, "bad"), unix=200.0)
    new = dict(notes.flag(True), unix=300.0)
    assert not notes.state([new, old]).bad           # file order does not matter
    assert notes.state([dict(new, unix=100.0), old]).bad


def test_merge():
    a1, a2 = dict(notes.note("a1"), unix=1.0), dict(notes.flag(False), unix=3.0)
    b1 = dict(notes.note("b1"), unix=2.0)
    d = dict(notes.delete(a1["id"]), unix=4.0)
    laptop, cluster = [a1, a2, d], [a1, b1]
    m = notes.merge(laptop, cluster)
    assert [x["id"] for x in m] == [a1["id"], b1["id"], a2["id"], d["id"]]
    assert notes.merge(cluster, laptop) == m                 # symmetric
    assert notes.merge(m, laptop) == m                       # idempotent
    st = notes.state(m)
    assert st.bad and [n["text"] for n in st.notes] == ["b1"]


def test_write_lines(tmp_path):
    p = tmp_path / "n.notes.jsonl"
    notes.append(p, notes.note("x"))
    lines = notes.merge(notes.read(p), [notes.note("y")])
    notes.write_lines(p, lines)
    assert [n["text"] for n in notes.read(p)] == ["x", "y"]
    assert not list(tmp_path.glob("*.tmp"))


def _writer(path, who, n):
    from wavi_py.replay import notes as nt
    for i in range(n):
        nt.append(path, nt.note(f"{who}-{i}-" + "x" * 3000, t=float(i), by=who))


def test_concurrent_appends(tmp_path):
    """Two processes (two windows) appending to one file: every line whole."""
    p = tmp_path / "n.notes.jsonl"
    ctx = mp.get_context("spawn")
    procs = [ctx.Process(target=_writer, args=(str(p), w, 200)) for w in ("a", "b", "c")]
    for pr in procs:
        pr.start()
    for pr in procs:
        pr.join(60)
        assert pr.exitcode == 0
    raw = p.read_bytes().decode("utf-8").splitlines()
    assert len(raw) == 600
    texts = {json.loads(s)["text"].split("-x")[0] for s in raw}
    assert len(texts) == 600


# ----- cards

def test_rms_db_and_quantise():
    t = np.arange(1000) / 100.0                      # 10 s at 100 Hz
    y = np.zeros((1000, 3), np.float32) + 250.0
    y[:, 1] += np.where(np.arange(1000) % 2, 1.0, -1.0)    # 1 mV RMS
    y[500:, 2] += np.sin(np.arange(500)) * 0.1
    db = cardmod.rms_db(t, y, 10.0, nbins=10)
    assert db.shape == (3, 10)
    assert np.all(np.isneginf(db[0]))                # constant: no variance
    assert np.allclose(db[1], 0.0, atol=0.01)        # 1 mV: 0 dB
    assert np.all(np.isneginf(db[2, :5]) | (db[2, :5] < -100)) and np.all(db[2, 5:] > -30)
    q = cardmod.quantise(db)
    assert q.dtype == np.uint8 and q[0].max() == 0
    assert abs(int(q[1, 0]) - round(40 / 55 * 255)) <= 1
    # empty bins (a gap in the scans) read as NaN, then 0
    db = cardmod.rms_db(np.r_[t[:300], t[700:]], np.r_[y[:300], y[700:]], 10.0, nbins=10)
    assert np.isnan(db[1, 3:7]).all() and np.isfinite(db[1, :3]).all()
    assert np.allclose(db[1, 7:], 0.0, atol=0.01)


def test_card_from_fixture(tmp_path):
    rec = tmp_path / "st_2026-11-02_1644_57.91_pool-a_b.jsonl"
    fx.write_recording(rec, spikes=True)
    out, card, tm = cardmod.write_card_for(rec)
    assert out == tmp_path / "st_2026-11-02_1644_57.91_pool-a_b.card.json"
    assert out.stat().st_size < 20_000
    assert not (tmp_path / (rec.name + ".store.npz")).exists()     # not kept by default
    back = cardmod.read_card(out)
    assert back == card
    assert card["stem"] == rec.name[:-6] and card["tag"] == "pool-a_b"
    assert card["start"] == "2026-11-02T16:44:57"
    assert card["nch"] == 32 and card["n_scans"] > 2000 and card["n_imu"] > 500
    assert card["fs_hz"] == pytest.approx(250, rel=1e-3)
    assert card["duration_s"] == pytest.approx(fx.DUR_S, abs=0.3)
    assert card["spikes"] == len(fx.SPIKES)
    assert card["faults"] == {"warn": 0, "fault": 1} or sum(card["faults"].values()) == 1
    assert card["armed_s"] == pytest.approx(fx.DISARM_AT - fx.ARM_AT, abs=0.5)
    lo, hi = card["depth_m"]
    assert hi == pytest.approx(fx.MAX_DEPTH_CM / 100, abs=0.03) and abs(lo) < 0.03
    d = cardmod.decode_depth(card)
    assert d.shape == (cardmod.NBINS,) and d.max() == pytest.approx(hi, abs=0.01)
    q = cardmod.decode_preview(card)
    assert q.shape == (32, cardmod.NBINS) and q.dtype == np.uint8
    assert q[list(fx.RAILED)].max() == 0            # railed gauges: dark rows
    live = [c for c in range(32) if c not in fx.RAILED]
    assert q[live].min() > 0
    assert cardmod.is_current(rec)
    os.utime(rec, ns=(out.stat().st_mtime_ns + 10**9,) * 2)
    assert not cardmod.is_current(rec)               # older than the recording


def test_make_cards_tool(tmp_path, capsys):
    from wavi_py.tools import make_cards
    sub = tmp_path / "2026-11-02"
    sub.mkdir()
    a = sub / "st_2026-11-02_1644_57.91_a.jsonl"
    b = sub / "whisker_20261102_170000_b.jsonl"
    fx.write_recording(a)
    fx.write_recording(b)
    notes.append(notes.notes_path(a), notes.note("x"))     # a .notes.jsonl is not a recording
    (sub / "junk.jsonl").write_text("not json\n", encoding="utf-8")
    assert make_cards.main([str(tmp_path)]) == 1           # junk fails, the rest are made
    err = capsys.readouterr().err
    assert "3 recordings, 3 cards to write" in err and "FAILED" in err
    assert cardmod.read_card(cardmod.card_path(a)) and cardmod.read_card(cardmod.card_path(b))
    (sub / "junk.jsonl").unlink()
    assert make_cards.main([str(tmp_path)]) == 0
    assert "2 recordings, 0 cards to write" in capsys.readouterr().err
    assert make_cards.main([str(tmp_path), "--force", "--quiet"]) == 0
    assert capsys.readouterr().err == ""


@pytest.mark.skipif(not DIVE.exists(), reason="pilot2 recording not on this machine")
def test_card_build_time_real_dive():
    """The card on top of the store: well under a second for the 13 min dive."""
    import time
    idx = RecordingIndex.open(DIVE)
    st = RecordingStore.open(idx)
    t0 = time.perf_counter()
    card = cardmod.build_card(st, idx, name=DIVE)
    dt = time.perf_counter() - t0
    print(f"card build {dt:.3f} s")
    assert dt < 0.5
    assert len(json.dumps(card)) < 20_000
