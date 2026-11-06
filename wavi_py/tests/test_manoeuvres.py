"""Manoeuvre extraction (replay/manoeuvres.py) and tools/make_manoeuvres.py, on
synthetic stick / servo / whisker traces.

Run: python -m pytest wavi_py/tests/test_manoeuvres.py
"""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from wavi_py.replay import manoeuvres as mn  # noqa: E402
from whisker_link import wire  # noqa: E402
from whisker_link.channels import CHANNEL_NAMES  # noqa: E402
from whisker_link.dome_map import DEFAULT_MAP, DomeMap  # noqa: E402

FS = mn.FS_HZ
BASE_MV = 270.0
DT = 1.0 / FS


def ch(name: str) -> int:
    return CHANNEL_NAMES.index(name)


def flight(dur: float, pieces, armed_from: float = 0.0, mode=None, **kw) -> mn.Flight:
    """``pieces``: [(t_from, (surge, sway, heave, yaw)), ...], each held until the next."""
    n = int(round(dur * FS))
    t = (np.arange(n) + 0.5) / FS
    u = np.zeros((n, 4))
    for t0, lev in pieces:
        u[t >= t0] = lev
    wsk = kw.pop("wsk", np.full((n, 32), BASE_MV))
    return mn.Flight(t=t, armed=t >= armed_from, u=u, wsk=wsk, names=list(CHANNEL_NAMES),
                     mode=np.full(n, 2) if mode is None else mode, stem="synthetic", **kw)


def classes(segs):
    return [(round(s.t0, 2), s.cls) for s in segs]


# ----- segments

def test_segment_boundaries_and_classes():
    fl = flight(20, [(0, (0, 0, 0, 0)), (3, (0.5, 0, 0, 0)), (8, (0, 0, 0, 0)),
                     (11, (-0.75, 0, 0, 0)), (12, (0, 0, 0, 0)), (13, (0.5, 0, 0, 0.25)),
                     (16, (0, 0, 0, 0)), (17, (0, 0, -1.0, 0))])
    segs = mn.segment(fl)
    assert classes(segs) == [(0.0, "rest"), (3.0, "single:surge+"), (8.0, "rest"),
                             (11.0, "single:surge-"), (12.0, "rest"), (13.0, "combo"),
                             (16.0, "rest"), (17.0, "single:heave-")]
    assert segs[5].axes == "surge+,yaw+"
    assert segs[1].level[0] == pytest.approx(0.5) and segs[3].level[0] == pytest.approx(-0.75)
    assert segs[-1].t1 == pytest.approx(20.0)
    # contiguous, covering the flight
    assert all(a.i1 == b.i0 for a, b in zip(segs, segs[1:]))


def test_rest_tolerance_and_small_wobble():
    """RC at 1500 +- REST_TOL is rest; a 0.05 wobble on a held level is no new
    segment (under STEP); a lone small input (0.05, nothing larger) is its own
    single-axis segment."""
    fl = flight(14, [(0, (0.01, 0, 0, -0.01)), (4, (0.5, 0, 0, 0)), (6, (0.55, 0, 0, 0)),
                     (8, (0.5, 0, 0, 0)), (10, (0, 0, 0, 0)), (12, (0, 0, 0, 0.05))])
    segs = mn.segment(fl)
    assert classes(segs) == [(0.0, "rest"), (4.0, "single:surge+"), (10.0, "rest"),
                             (12.0, "single:yaw+")]
    assert segs[-1].level[3] == pytest.approx(0.05)


def test_minor_inputs():
    """A yaw under MINOR during a surge hold: no new segment, no combo, recorded as
    minor on the hold and its onset; at MINOR and above: a combo."""
    fl = flight(20, [(0, (0, 0, 0, 0)), (3, (0.75, 0, 0, 0)), (5, (0.75, 0, 0, 0.1)),
                     (6, (0.75, 0, 0, -0.12)), (7, (0.75, 0, 0, 0)), (10, (0, 0, 0, 0)),
                     (13, (0.5, 0, 0, 0)), (15, (0.5, 0, 0, 0.3)), (17, (0, 0, 0, 0))])
    segs = mn.segment(fl)
    assert classes(segs) == [(0.0, "rest"), (3.0, "single:surge+"), (10.0, "rest"),
                             (13.0, "single:surge+"), (15.0, "combo"), (17.0, "rest")]
    assert segs[1].minor == {"yaw": {"max": 0.12, "share": 0.286}}     # 2 s of 7 s
    assert segs[3].minor == {} and segs[1].level[3] == 0.0
    doc = mn.extract(fl)
    hold = next(e for e in doc["events"] if e["kind"] == "hold" and e["t"] == 3.0)
    on = next(e for e in doc["events"] if e["type"] == "onset" and e["t"] == 3.0)
    assert hold["minor"] == on["minor"] == {"yaw": {"max": 0.12, "share": 0.286}}
    assert doc["segments"][1]["minor"]["yaw"]["max"] == 0.12
    assert np.array_equal(mn.effective_demand(np.array([[0.1, 0, 0, 0.05], [0.5, 0, 0, 0.1]])),
                          [[0.1, 0, 0, 0.05], [0.5, 0, 0, 0]])


def test_debounce_ramp_and_blips():
    """A ramp 0 -> 0.75 over 0.1 s is one onset at its start; a 0.1 s yaw blip
    disappears into the rest; a 0.3 s yaw tap stays."""
    fl = flight(12, [(0, (0, 0, 0, 0)), (2.0, (0.25, 0, 0, 0)), (2.04, (0.5, 0, 0, 0)),
                     (2.08, (0.75, 0, 0, 0)), (5, (0, 0, 0, 0)), (7, (0, 0, 0, 0.5)),
                     (7.1, (0, 0, 0, 0)), (9, (0, 0, 0, -0.5)), (9.3, (0, 0, 0, 0))])
    segs = mn.segment(fl)
    assert classes(segs) == [(0.0, "rest"), (2.0, "single:surge+"), (5.0, "rest"),
                             (9.0, "single:yaw-"), (9.3, "rest")]
    assert segs[1].level[0] == pytest.approx(0.75)


def test_disarmed_is_its_own_segment_without_events():
    fl = flight(10, [(0, (0.5, 0, 0, 0)), (6, (0, 0, 0, 0))], armed_from=4.0)
    segs = mn.segment(fl)
    assert classes(segs) == [(0.0, "disarmed"), (4.0, "single:surge+"), (6.0, "rest")]
    doc = mn.extract(fl)
    assert [(e["type"], e["t"]) for e in doc["events"]] == [
        ("hold", 4.0), ("release", 6.0), ("hold", 6.0)]


def test_transition_types_and_windows():
    fl = flight(30, [(0, (0, 0, 0, 0)), (5, (0.5, 0, 0, 0)), (10, (1.0, 0, 0, 0)),
                     (15, (-0.5, 0, 0, 0)), (20, (0, 0, 0, 0)), (24, (0, 0, 0.5, 0)),
                     (26, (0, 0, 0.5, 0.5))])
    doc = mn.extract(fl)
    tr = [e for e in doc["events"] if e["kind"] == "transition"]
    got = [(e["t"], e["type"], e["class"], e["axis"], e["sign"]) for e in tr]
    assert got == [(5.0, "onset", "single:surge+", "surge", 1),
                   (10.0, "step", "single:surge+", "surge", 1),
                   (15.0, "reversal", "single:surge-", "surge", -1),
                   (20.0, "release", "single:surge-", "surge", -1),
                   (24.0, "onset", "single:heave+", "heave", 1),
                   (26.0, "step", "combo", "yaw", 1)]
    e = tr[0]
    assert (e["t_start"], e["t_end"]) == (5.0 - mn.PRE_S, 5.0 + mn.POST_S)
    assert tr[-1]["t_end"] == pytest.approx(30.0)            # clipped to the flight
    holds = [e for e in doc["events"] if e["kind"] == "hold"]
    assert len(holds) == 7 and holds[0]["class"] == "rest"
    assert holds[1]["level"] == [0.5, 0.0, 0.0, 0.0] and holds[1]["duration_s"] == 5.0
    # quiet time and baseline_clean: 5 s of rest before the first onset; the
    # heave onset at 24 s follows 4 s of rest; the step at 26 none
    assert tr[0]["context"]["quiet_before_s"] == 5.0 and tr[0]["context"]["baseline_clean"]
    assert tr[4]["context"]["quiet_before_s"] == 4.0
    assert tr[5]["context"]["quiet_before_s"] == 0.0
    assert tr[1]["context"]["baseline_t"] == [4.0, 5.0]      # the rest before the onset


def test_short_quiet_is_not_a_clean_baseline():
    fl = flight(14, [(0, (0, 0, 0, 0)), (3, (0.5, 0, 0, 0)), (6, (0, 0, 0, 0)),
                     (7, (0.5, 0, 0, 0))])
    tr = [e for e in mn.extract(fl)["events"] if e["type"] == "onset"]
    assert [e["context"]["baseline_clean"] for e in tr] == [True, False]
    assert tr[1]["context"]["quiet_before_s"] == 1.0
    assert not tr[1]["quality"]["baseline_clean"]


def test_mode_tagging():
    n = int(20 * FS)
    t = (np.arange(n) + 0.5) / FS
    mode = np.where(t < 10, 19, 2)
    fl = flight(20, [(0, (0, 0, 0, 0)), (4, (0, 0, 0.5, 0)), (6, (0, 0, 0, 0)),
                     (14, (0, 0, 0.5, 0)), (16, (0, 0, 0, 0))], mode=mode)
    on = [e for e in mn.extract(fl)["events"] if e["type"] == "onset"]
    assert [e["context"]["mode"] for e in on] == ["MANUAL", "ALT_HOLD"]
    assert [e["context"]["mode_id"] for e in on] == [19, 2]
    hold = [e for e in mn.extract(fl)["events"] if e["kind"] == "hold" and e["t"] == 6.0][0]
    assert hold["context"]["mode_changes"]                    # 6..14 spans the switch


# ----- measurements

def first_order(t, t_on, t_off, amp, tau_on, tau_off):
    """0 before t_on, a first-order rise to amp, from t_off a first-order decay."""
    y = np.where(t >= t_on, amp * (1 - np.exp(-(t - t_on) / tau_on)), 0.0)
    y_off = amp * (1 - np.exp(-(t_off - t_on) / tau_on))
    return np.where(t >= t_off, y_off * np.exp(-(t - t_off) / tau_off), y)


def response_flight(drift_mv_s: float = 0.0, ref: bool = True):
    dur = 30.0
    n = int(dur * FS)
    t = (np.arange(n) + 0.5) / FS
    wsk = np.full((n, 32), BASE_MV)
    resp = first_order(t, 5.0, 15.0, 2.0, 0.4, 0.8)
    wsk[:, ch("W2_A")] += resp                     # LBM drag, polarity +1
    wsk[:, ch("W2_B")] += 0.5 * resp               # LBM lift, polarity -1 -> -0.5 x
    drift = drift_mv_s * t
    if ref:
        wsk += drift[:, None]                      # every gauge, the reference's too
    else:
        wsk[:, [c for c in range(32) if c not in (ch("W10_B"), ch("W14_B"))]] += drift[:, None]
    gyro = np.zeros((n, 3))
    gyro[:, 2] = np.radians(first_order(t, 5.0, 15.0, 30.0, 0.3, 0.6))   # yaw dps
    return flight(dur, [(0, (0, 0, 0, 0)), (5, (0.5, 0, 0, 0)), (15, (0, 0, 0, 0))],
                  wsk=wsk, gyro=gyro)


def ev_of(doc, typ):
    return next(e for e in doc["events"] if e["type"] == typ)


def test_baseline_steady_decay_on_a_first_order_response():
    doc = mn.extract(response_flight())
    on, rel = ev_of(doc, "onset"), ev_of(doc, "release")
    d, li = on["positions"]["LBM"]["drag"], on["positions"]["LBM"]["lift"]
    assert d["base"] == pytest.approx(BASE_MV, abs=1e-3)
    assert d["steady"] == pytest.approx(2.0, rel=1e-3)
    assert li["steady"] == pytest.approx(-1.0, rel=1e-3)     # polarity applied
    assert d["peak"] == pytest.approx(2.0 * (1 - np.exp(-mn.PEAK_S / 0.4)), rel=0.02)
    assert on["positions"]["LFM"]["drag"]["steady"] == pytest.approx(0.0, abs=1e-6)
    # release: decays from ~2 mV with tau 0.8 s, over the rest after it
    assert rel["positions"]["LBM"]["drag"]["tau_s"] == pytest.approx(0.8, rel=0.05)
    assert rel["positions"]["LBM"]["drag"]["peak"] == pytest.approx(2.0, rel=0.02)
    # motion: yaw rate steady 30 dps, rise to 63 % in ~tau (0.3 s), decay 0.6 s
    y = on["motion"]["yaw_rate_dps"]
    assert y["steady"] == pytest.approx(30.0, rel=1e-3)
    assert y["rise63_s"] == pytest.approx(0.3, abs=2 * DT)
    assert rel["motion"]["yaw_rate_dps"]["tau_s"] == pytest.approx(0.6, rel=0.05)
    assert on["motion"]["roll_rate_dps"]["rise63_s"] is None   # no response: no rise time
    assert on["quality"]["steady"] and not doc["events"][0]["quality"]["steady"] is None


def test_reference_removes_the_drift():
    """A 0.02 mV/s common drift on every gauge: with the reference gauges drifting
    too it is removed; without (they stay flat) the steady value carries the
    drift since the baseline (5..15 s later, ~0.2 mV)."""
    on = ev_of(mn.extract(response_flight(0.02)), "onset")
    assert on["positions"]["LBM"]["drag"]["steady"] == pytest.approx(2.0, abs=0.01)
    assert on["reference"]["gauges"] == ["W10_B", "W14_B"]
    assert on["reference"]["shift_mv"] > 0.05
    on2 = ev_of(mn.extract(response_flight(0.02, ref=False)), "onset")
    assert on2["positions"]["LBM"]["drag"]["steady"] == pytest.approx(2.0 + 0.02 * 8.3, abs=0.03)
    # a map without a reference: no correction
    doc = json.loads(Path(DEFAULT_MAP).read_text())
    doc.pop("reference")
    on3 = ev_of(mn.extract(response_flight(0.02), DomeMap.from_dict(doc)), "onset")
    assert on3["reference"]["gauges"] == [] and on3["reference"]["shift_mv"] is None
    assert on3["positions"]["LBM"]["drag"]["steady"] > 2.1


def test_alive_flags():
    """Band noise (1.5-10 Hz, ~200 uV) on every position's gauges during a hold
    but LFL's: LFL is flagged lost; at rest nothing is flagged lost."""
    dur = 20.0
    n = int(dur * FS)
    t = (np.arange(n) + 0.5) / FS
    rng = np.random.default_rng(3)
    wsk = np.full((n, 32), BASE_MV) + rng.normal(0, 0.003, (n, 32))   # 3 uV floor
    on = (t >= 5) & (t < 15)
    noise = 0.2 * np.sin(2 * np.pi * 4.0 * t)[:, None] * np.ones(32)
    wsk[on] += noise[on]
    wsk[:, [ch("W5_A"), ch("W5_B")]] = BASE_MV + rng.normal(0, 0.003, (n, 2))
    fl = flight(dur, [(0, (0, 0, 0, 0)), (5, (0.5, 0, 0, 0)), (15, (0, 0, 0, 0))], wsk=wsk)
    doc = mn.extract(fl)
    hold = next(e for e in doc["events"] if e["kind"] == "hold" and e["class"] != "rest")
    assert hold["positions"]["LFL"]["alive"] is False and hold["quality"]["lost"] == ["LFL"]
    assert hold["positions"]["LBM"]["alive"] is True
    assert hold["positions"]["LBM"]["band_uv"] == pytest.approx(200, rel=0.1)
    rest = doc["events"][0]
    assert rest["class"] == "rest" and rest["positions"]["LFL"]["alive"] is None


def test_depth_reference_check():
    """Depth below -DEPTH_NEG_CM (a surface pressure taken under water): no
    near-surface flag."""
    n = int(12 * FS)
    t = (np.arange(n) + 0.5) / FS
    pieces = [(0, (0, 0, 0, 0)), (5, (0.5, 0, 0, 0))]
    e = ev_of(mn.extract(flight(12, pieces, depth_cm=np.full(n, 10.0))), "onset")
    assert e["context"]["near_surface"] is True
    doc = mn.extract(flight(12, pieces, depth_cm=-30.0 + t))
    assert not doc["recording"]["depth_ok"] and doc["recording"]["depth_min_cm"] < -29
    assert ev_of(doc, "onset")["context"]["near_surface"] is None


def test_railed_and_flagged_quality():
    n = int(12 * FS)
    wsk = np.full((n, 32), BASE_MV)
    wsk[300:310, ch("W9_A")] = 1200.0                          # RFL drag railed at 6 s
    fl = flight(12, [(0, (0, 0, 0, 0)), (5, (0.5, 0, 0, 0))], wsk=wsk,
                flag_t=np.array([5.5, 5.6, 11.0]))
    e = ev_of(mn.extract(fl), "onset")
    assert e["quality"]["railed"] == ["RFL"] and e["quality"]["adc_flagged_scans"] == 2


def test_thrust_mix_round_trip_and_check():
    """Servos made from the ArduSub mix with this vehicle's directions project back
    onto the demand; mix_check sees the directions and a unit slope."""
    fl = flight(24, [(0, (0, 0, 0, 0)), (2, (0.5, 0, 0, 0)), (5, (0, 0, 0, 0)),
                     (6, (1.0, 0, 0, 0)), (9, (0, 0, 0, 0)), (10, (0, -0.5, 0, 0)),
                     (13, (0, -1.0, 0, 0)), (16, (0, 0, 0.75, 0)), (18, (0, 0, -0.5, 0)),
                     (20, (0, 0, 0, 0.5)), (22, (0, 0, 0, -0.25))])
    cmd = np.zeros((len(fl.t), 6))
    cmd[:, 4], cmd[:, 5], cmd[:, 3], cmd[:, 2] = fl.u[:, 0], fl.u[:, 1], fl.u[:, 2], fl.u[:, 3]
    th = (cmd @ mn.MIX.T) * np.asarray(mn.MOTOR_DIRECTION)
    fl.servo = mn.NEUTRAL_US + mn.FULL_US * th
    rec = fl.thrust()
    assert np.allclose(rec, cmd, atol=1e-9)
    chk = mn.mix_check(fl, mn.segment(fl))
    assert chk["direction_ok"] and chk["servo_sign"] == list(mn.MOTOR_DIRECTION)
    for a in mn.AXES:
        assert chk["axes"][a]["holds"] == 2
        assert chk["axes"][a]["slope"] == pytest.approx(1.0) and chk["axes"][a]["leak_max"] < 1e-9
    on = [e for e in mn.extract(fl)["events"] if e["type"] == "onset"][0]
    assert on["context"]["thrust"]["surge"] == pytest.approx(0.5)


def test_fit_decay_and_rise_time_helpers():
    t = np.arange(0, 4, DT)
    assert mn.fit_decay(1.5 + 3 * np.exp(-t / 0.7), DT) == pytest.approx(0.7, rel=1e-3)
    assert mn.fit_decay(np.full(len(t), 2.0), DT) is None
    assert mn.fit_decay(np.exp(-t[:10] / 0.7), DT) is None          # too short
    x = 5 * (1 - np.exp(-t / 0.5))
    assert mn.rise_time(x, 0.0, 5.0, DT) == pytest.approx(0.5, abs=DT)
    assert mn.rise_time(x, 0.0, float("nan"), DT) is None


# ----- the sidecar, from a recording

def write_rec(path: Path, dur: float = 20.0) -> float:
    """A small daq_server recording: 250 Hz scans, RAW_IMU / ATTITUDE 50 Hz, RC and
    SERVO 20 Hz (surge 0.5 from 6 s to 12 s), HEARTBEAT 2 Hz (armed from 2 s, ALT_HOLD),
    SCALED_PRESSURE2 20 Hz. Returns the receive time of the first line (session zero)."""
    lines = []
    t_first = 500.0

    def add(t, ev):
        lines.append((t, wire.dumps({"k": ev.pop("k"), "t": t_first + t, **ev})))

    for i in range(int(dur * 250)):
        s = i / 250
        raw = [int(BASE_MV / mn.MV_PER_CODE)] * 32
        if 6.0 <= s < 12.0:
            raw[ch("W2_A")] += int(1.0 / mn.MV_PER_CODE)
        add(s + 0.001, {"k": "wsk", "seq": 1000 + i, "tus": 9_000_000 + int(s * 1e6),
                        "dur": 4000, "rail": 0, "flg": 2 if abs(s - 7.0) < 0.002 else 0,
                        "raw": raw})
    for i in range(int(dur * 50)):
        s = i / 50
        add(s + 0.002, {"k": "mav", "type": "RAW_IMU", "src": [1, 1],
                        "d": {"time_usec": 70_000_000 + int(s * 1e6), "xacc": 0, "yacc": 0,
                              "zacc": -1000, "xgyro": 0, "ygyro": 0, "zgyro": 0, "id": 0}})
        add(s + 0.003, {"k": "mav", "type": "ATTITUDE", "src": [1, 1],
                        "d": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}})
    for i in range(int(dur * 20)):
        s = i / 20
        u = 1700 if 6.0 <= s < 12.0 else 1500
        add(s + 0.004, {"k": "mav", "type": "RC_CHANNELS", "src": [1, 1],
                        "d": {f"chan{c}_raw": (u if c == 5 else 1500) for c in range(1, 12)}})
        th = mn.MIX[:, 4] * np.asarray(mn.MOTOR_DIRECTION) * (u - 1500) / 400
        add(s + 0.005, {"k": "mav", "type": "SERVO_OUTPUT_RAW", "src": [1, 1],
                        "d": {f"servo{m + 1}_raw": int(round(1500 + 400 * th[m]))
                              for m in range(8)}})
        add(s + 0.006, {"k": "mav", "type": "SCALED_PRESSURE2", "src": [1, 1],
                        "d": {"press_abs": 1000.0 + (0.981 * 50 if s > 3 else 0.0)}})
    for i in range(int(dur * 2)):
        s = i / 2
        add(s + 0.007, {"k": "mav", "type": "HEARTBEAT", "src": [1, 1],
                        "d": {"base_mode": 81 + 128 * (s >= 2.0), "custom_mode": 2}})
        add(s + 0.008, {"k": "mav", "type": "HEARTBEAT", "src": [1, 194],
                        "d": {"base_mode": 128, "custom_mode": 0}})
    lines.sort(key=lambda x: x[0])
    hello = {"k": "hello", "t": t_first - 0.5, "mono": 1.0, "unix": 1.79e9, "wire": 1,
             "server": "0.2", "channels": CHANNEL_NAMES, "mv_per_code": mn.MV_PER_CODE}
    with open(path, "wb") as f:
        f.write((json.dumps(hello) + "\n").encode())
        for _, b in lines:
            f.write(b)
    return lines[0][0]


def test_sidecar_from_a_recording(tmp_path):
    rec = tmp_path / "st_2026-11-05_1200_00.00_test.jsonl"
    first = write_rec(rec)
    out = tmp_path / "out"
    assert not mn.is_current(rec, out)
    path, doc, _ = mn.write_manoeuvres_for(rec, out, cache=False)
    assert path == out / "st_2026-11-05_1200_00.00_test.manoeuvres.json"
    assert mn.is_current(rec, out) and not mn.is_current(rec)
    assert not list(tmp_path.glob("*.npz"))                   # no caches next to it
    assert mn.read_doc(path)["v"] == mn.VERSION and doc["constants"]["minor"] == mn.MINOR
    assert doc["stem"] == "st_2026-11-05_1200_00.00_test" and doc["tag"] == "test"
    r = doc["recording"]
    assert r["armed_spans"][0][0] == pytest.approx(2.0 - first + 0.007, abs=0.01)
    assert r["mix_check"]["direction_ok"]
    on = ev_of(doc, "onset")
    # session seconds: the RC change at 6.004 s receive, session zero at the first line
    assert on["t"] == pytest.approx(6.004 - first, abs=1.5 / FS)
    assert on["class"] == "single:surge+" and on["level_to"][0] == 0.5
    assert on["context"]["mode"] == "ALT_HOLD"
    assert on["context"]["depth_cm"] == pytest.approx(50.0, abs=1.0)
    assert on["context"]["near_surface"] is False and r["depth_ok"]
    assert on["context"]["thrust"]["surge"] == pytest.approx(0.5, abs=0.01)
    assert on["positions"]["LBM"]["drag"]["steady"] == pytest.approx(1.0, abs=0.01)
    assert on["quality"]["adc_flagged_scans"] == 1
    rows = list(mn.csv_rows(doc))
    assert len(rows) == 12 * len(doc["events"]) and set(rows[0]) == set(mn.CSV_COLUMNS)


def test_make_manoeuvres_tool(tmp_path):
    from wavi_py.tools import make_manoeuvres as tool
    d = tmp_path / "recs"
    d.mkdir()
    write_rec(d / "st_2026-11-05_1200_00.00_a.jsonl")
    write_rec(d / "st_2026-11-05_1210_00.00_b.jsonl")
    (d / "st_2026-11-05_1200_00.00_a.notes.jsonl").write_text("")
    out = tmp_path / "out"
    assert tool.main([str(d), "--out", str(out), "--quiet"]) == 0
    assert sorted(p.name for p in out.iterdir()) == [
        "events.csv", "st_2026-11-05_1200_00.00_a.manoeuvres.json",
        "st_2026-11-05_1210_00.00_b.manoeuvres.json"]
    with open(out / "events.csv", newline="") as f:
        rows = list(csv.DictReader(f))
    assert {r["stem"] for r in rows} == {"st_2026-11-05_1200_00.00_a", "st_2026-11-05_1210_00.00_b"}
    assert not list(d.glob("*.npz")) and not list(d.glob("*.manoeuvres.json"))
    # current: nothing to do; next to the recordings (no --out) with caches
    m = (out / "st_2026-11-05_1200_00.00_a.manoeuvres.json").stat().st_mtime_ns
    assert tool.main([str(d), "--out", str(out), "--quiet"]) == 0
    assert (out / "st_2026-11-05_1200_00.00_a.manoeuvres.json").stat().st_mtime_ns == m
    assert tool.main([str(d / "st_2026-11-05_1200_00.00_a.jsonl"), "--quiet", "--no-csv"]) == 0
    assert (d / "st_2026-11-05_1200_00.00_a.manoeuvres.json").exists()
    assert list(d.glob("*.idx.npz")) and not (d / "events.csv").exists()
