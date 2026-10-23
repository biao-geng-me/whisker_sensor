"""Run whisker_daq.ino's locked scan in the host simulation (sim.cpp) and check
the decoded stream. Skips without arduino-cli (esp32 core) or a working g++.

Run: python -m pytest msd/esp32/host_sim/test_host_sim.py
     (CXX=/path/to/g++ if the default one isn't usable)
"""

from __future__ import annotations

import collections
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT))
from whisker_link import esp32_frame as ef  # noqa: E402
from whisker_link.channels import OUT_OF  # noqa: E402


sys.path.insert(0, str(HERE))
from cxx_probe import find_cxx  # noqa: E402


def _cxx() -> str | None:
    return find_cxx("-O0")


@pytest.fixture(scope="module")
def sim(tmp_path_factory):
    if not shutil.which("arduino-cli"):
        pytest.skip("arduino-cli not found")
    cxx = _cxx()
    if not cxx:
        pytest.skip("no working C++ compiler (set CXX)")
    out = tmp_path_factory.mktemp("sim")
    r = subprocess.run([str(HERE / "run_sim.sh"), str(out)], capture_output=True, text=True,
                       env={**os.environ, "CXX": cxx})
    assert r.returncode == 0, r.stderr[-3000:]
    return out / "sim"


LAST = {}                              # the last run: decoder junk bytes, sim summary


def run(sim, out: Path, *args: str):
    """Run the sim; every run must be free of task-watchdog errors and raw
    (unframed) ESP-IDF log lines (bench r3)."""
    r = subprocess.run([str(sim), "--out", str(out), *args], capture_output=True, text=True,
                       timeout=300)
    assert r.returncode == 0, r.stderr
    assert "twdt errors 0, raw log lines 0" in r.stderr, r.stderr[-500:]
    dec = ef.Decoder()
    evs = list(dec.feed(out.read_bytes()))
    LAST.update(junk=dec.n_junk, stderr=r.stderr)
    return ([e for e in evs if isinstance(e, ef.Scan)], [e for e in evs if isinstance(e, ef.Info)],
            [e for e in evs if isinstance(e, ef.Health)], [e.text for e in evs if isinstance(e, ef.Text)])


def expected(o: int) -> int:          # the simulated input of OUT o (sim.cpp outValue)
    return -1600000 + 100003 * o


SKIPPED6 = {OUT_OF[a][6] for a in range(4)}


@pytest.mark.parametrize("mcmd,mode,period,osr,tol", [
    ("M2;", 2, 4000, 128, 300), ("M1;", 1, 16000, 512, 60), ("M1 200 256;", 1, 16000, 256, 60),
    ("M1 200 256 8000;", 1, 8000, 256, 60)])
def test_locked_modes(sim, tmp_path, mcmd, mode, period, osr, tol):
    scans, infos, health, _ = run(sim, tmp_path / "o.bin", "--end", "3",
                                  "--cmd", "1.0:S0123457;", "--cmd", f"1.0:{mcmd}",
                                  "--cmd", "1.3:B")
    cfg = ef.info_cfg(dict(infos[-1].__dict__))
    assert cfg["mode"] == mode and cfg["selects"] == "0123457" and cfg["clkin_hz"] == 8_000_000
    assert cfg["osr"] == osr
    assert cfg["scan_hz"] == 1e6 / period
    assert len(scans) > 1.4e6 / period
    assert {b.t_us - a.t_us for a, b in zip(scans, scans[1:])} == {period}
    assert all(b.seq == a.seq + 1 for a, b in zip(scans, scans[1:]))
    assert all(s.flags == 0 and s.dur_us == period for s in scans[1:])
    for s in scans[2:]:
        for o in range(1, 33):
            if o in SKIPPED6:
                assert s.railmask >> (o - 1) & 1
            else:
                assert abs(s.raw[o - 1] - expected(o)) < tol, (o, s.raw[o - 1] - expected(o))
    h = health[-1]
    assert h.mode == mode and h.overrun == 0 and h.missed == 0 and h.tx_drop == 0
    assert h.empty_slots == 0 and h.conversions > 0.95 * cfg["fdata_sps"]
    assert sum(x.resync for x in health) <= 1          # the one at stream start


def test_period_needs_whole_conversions(sim, tmp_path):
    _, infos, _, text = run(sim, tmp_path / "o.bin", "--end", "1.5",
                            "--cmd", "1.0:M1 200 512 8000;", "--cmd", "1.2:B")
    assert any("whole number of conversions" in t for t in text)
    assert infos[-1].mode == 1                          # still the boot default (quiet)


def test_stall_flags_one_scan_and_keeps_grid(sim, tmp_path):
    scans, _, health, _ = run(sim, tmp_path / "o.bin", "--end", "3", "--cmd", "1.0:M2;",
                              "--cmd", "1.3:B", "--stall", "2.0:300")
    flagged = [s for s in scans if s.flags]
    assert len(flagged) == 1 and flagged[0].flags & ef.FLAG_SYNC
    assert {b.t_us - a.t_us for a, b in zip(scans, scans[1:])} == {4000}
    assert sum(h.overrun for h in health) >= 1 and sum(h.missed for h in health) >= 5


def test_adc_reset_reinit(sim, tmp_path):
    scans, _, health, text = run(sim, tmp_path / "o.bin", "--end", "3", "--cmd", "1.0:M2;",
                                 "--cmd", "1.3:B", "--adc-reset", "2.0")
    assert any("adc reset" in t for t in text)
    assert any(s.flags & ef.FLAG_REINIT for s in scans)
    after = [s for s in scans if s.t_us > scans[-1].t_us - 500_000]
    assert all(s.flags == 0 for s in after)
    assert collections.Counter(b.t_us - a.t_us for a, b in zip(after, after[1:])) == \
        collections.Counter({4000: len(after) - 1})


def test_quiet_boot_default_wide_and_legacy(sim, tmp_path):
    scans, infos, health, _ = run(sim, tmp_path / "o.bin", "--end", "3", "--cmd", "0.9:B",
                                  "--cmd", "1.5:M2;", "--cmd", "2.2:M0;")
    assert [i.mode for i in infos] == [1, 2, 0]           # quiet is the boot default
    quiet = [s for s in scans if s.dur_us == 16000]
    assert len(quiet) > 20 and {b.t_us - a.t_us for a, b in zip(quiet, quiet[1:])} == {16000}
    legacy = [s for s in scans if s.dur_us not in (4000, 16000)]
    assert len(legacy) > 40 and len([s for s in scans if s.dur_us == 4000]) > 100


# An overrun (a stall of 1.1-4 conversions, at phases spread over a scan): the
# reader reads the older FIFO entry, and the narrow DRDY pulse after it is an
# extra edge for an index already read. Bench 2026-10-23 (quiet OSR 256): one
# startup overrun left every later index one ahead, and resync counted every
# conversion (the old firmware does the same in this sim: ~13500 resyncs/s).
@pytest.mark.parametrize("mcmd,T,period", [("M1 200 256;", 64, 16000), ("M1;", 128, 16000),
                                           ("M2;", 32, 4000), ("M1 200 256 8000;", 64, 8000)])
def test_overrun_realigns(sim, tmp_path, mcmd, T, period):
    for i, mult in enumerate((1.1, 1.5, 2.0, 2.6, 4.0, 1.5, 2.6)):
        at = 1.5 + (i * period / 7 + i * T * 0.37) * 1e-6
        scans, _, health, _ = run(sim, tmp_path / "o.bin", "--end", "2.7",
                                  "--cmd", "1.0:S0123457;", "--cmd", f"1.0:{mcmd}",
                                  "--cmd", "1.3:B", "--stall", f"{at}:{mult * T}")
        what = (mult, at)
        assert {b.t_us - a.t_us for a, b in zip(scans, scans[1:])} == {period}, what
        assert all(b.seq == a.seq + 1 for a, b in zip(scans, scans[1:])), what
        flagged = [s.seq for s in scans if s.flags]
        # one scan, or two when the stall straddles a scan end
        assert len(flagged) <= 2 and all(b == a + 1 for a, b in zip(flagged, flagged[1:])), what
        # (wide on one core, with the DRDY ISR on the reader's core: a 2T stall
        # can cost a second overrun while it catches up)
        assert sum(h.overrun for h in health) <= (2 if T == 32 else 1), what
        assert sum(h.resync for h in health) <= 3, (what, [h.resync for h in health])
        tail = [s for s in scans if s.t_us > scans[-1].t_us - 300_000]
        assert all(s.flags == 0 for s in tail), what
        for s in tail:
            for o in range(1, 33):
                if o not in SKIPPED6:
                    assert abs(s.raw[o - 1] - expected(o)) < (300 if period == 4000 else 60)


# Bench 2026-10-23, OSR 256 on one core: one overrun per HEALTH frame. A write
# while the UART FIFO still held data deferred the driver's refill ISR to a
# random point of the grid (here: --board-costs, calibrated to the bench).
@pytest.mark.parametrize("mcmd", ["M1 200 256;", "M1;", "M1 200 256 8000;"])
def test_quiet_board_costs_no_overrun(sim, tmp_path, mcmd):
    scans, _, health, _ = run(sim, tmp_path / "o.bin", "--end", "5.4", "--board-costs",
                              "--cmd", "1.0:S0123457;", "--cmd", f"1.0:{mcmd}", "--cmd", "1.3:B")
    assert len(health) >= 3
    assert sum(h.overrun for h in health) == 0 and sum(h.missed for h in health) == 0
    assert max(h.read_max_us for h in health) < 40   # no UART refill ISR (~60 us) inside a read
    # bench r3: C0 read on core 0 while the DRDY ISR (core 1) waited for core 1's
    # tick: one edgeless + stale read per LCM(1 ms, T). C0 reads on core 1 again.
    assert sum(h.edgeless for h in health) == 0 and sum(h.stale for h in health) == 0
    assert not [s for s in scans if s.flags] and LAST["junk"] == 0


# ---------------------------------------------------------------- dual core (C1)
def _health_ok(health, lat_limit):
    tail = health[1:]                                     # the first window covers the start
    assert tail and all(h.core == 1 for h in tail)
    assert sum(h.overrun for h in tail) == 0 and sum(h.missed for h in tail) == 0
    assert max(h.lat_max_us for h in tail) < lat_limit, [h.lat_max_us for h in tail]
    assert sum(h.tx_drop for h in tail) == 0 and sum(h.empty_slots for h in tail) == 0


@pytest.mark.parametrize("scale", ["1.0", "1.5"])
@pytest.mark.parametrize("ccmd", ["C1;", "C1 100;"])
def test_c1_wide_board_costs(sim, tmp_path, scale, ccmd):
    """Wide on its own core with the bench-calibrated costs: no overrun, lat < T = 32 us."""
    scans, infos, health, _ = run(sim, tmp_path / "o.bin", "--end", "4", "--board-costs",
                                  "--cost-scale", scale, "--cmd", "1.0:S0123457;",
                                  "--cmd", f"1.0:{ccmd}", "--cmd", "1.0:M2;", "--cmd", "1.3:B")
    # lat < T at the bench costs; at 1.5x the 12 us tick on core 1 can push one read
    # just past the next conversion (handled: its narrow DRDY pulse, no overrun)
    _health_ok(health, 32 if scale == "1.0" else 40)
    assert all(h.tx_win_us == (100 if "100" in ccmd else 0) and h.tx_late == 0 for h in health[1:])
    assert {b.t_us - a.t_us for a, b in zip(scans, scans[1:])} == {4000}
    assert all(b.seq == a.seq + 1 for a, b in zip(scans, scans[1:]))
    assert not [s for s in scans if s.flags]
    for s in scans[2:]:
        for o in range(1, 33):
            if o not in SKIPPED6:
                assert abs(s.raw[o - 1] - expected(o)) < 300


def test_c1_quiet_windowed(sim, tmp_path):
    scans, _, health, _ = run(sim, tmp_path / "o.bin", "--end", "4", "--board-costs",
                              "--cmd", "1.0:S0123457;", "--cmd", "1.0:C1 2000;", "--cmd", "1.3:B")
    _health_ok(health, 64)
    assert {b.t_us - a.t_us for a, b in zip(scans, scans[1:])} == {16000}
    assert not [s for s in scans if s.flags] and all(h.tx_late == 0 for h in health)


def test_c1_stalls_each_core(sim, tmp_path):
    """Core 1 (the reader) stalled 300 us: one flagged scan. Core 0 (the sender)
    stalled 30 ms: the queue absorbs it, nothing is flagged or lost."""
    scans, _, health, _ = run(sim, tmp_path / "o.bin", "--end", "4", "--board-costs",
                              "--cmd", "1.0:S0123457;", "--cmd", "1.0:C1;", "--cmd", "1.0:M2;",
                              "--cmd", "1.3:B", "--stall", "2.0021:300:1", "--stall", "2.5:30000:0")
    assert len([s for s in scans if s.flags]) == 1
    assert {b.t_us - a.t_us for a, b in zip(scans, scans[1:])} == {4000}
    assert all(b.seq == a.seq + 1 for a, b in zip(scans, scans[1:]))
    assert max(h.q_hwm for h in health) >= 7 and sum(h.tx_drop for h in health) == 0


def test_c1_queue_full_keeps_health(sim, tmp_path):
    """The sender stalled 300 ms (> 32 queued scans): scans are dropped and
    counted, HEALTH frames still arrive, the stream recovers."""
    scans, _, health, _ = run(sim, tmp_path / "o.bin", "--end", "5", "--board-costs",
                              "--cmd", "1.0:C1;", "--cmd", "1.0:M2;", "--cmd", "1.3:B",
                              "--stall", "2.5:300000:0")
    assert sum(h.tx_drop for h in health) > 0
    assert len(health) >= 3 and all(h.overrun == 0 for h in health[1:])
    gaps = [b.seq - a.seq for a, b in zip(scans, scans[1:]) if b.seq != a.seq + 1]
    assert len(gaps) == 1 and gaps[0] > 1                 # one hole, seq still counts scans
    tail = [s for s in scans if s.t_us > scans[-1].t_us - 500_000]
    assert {b.t_us - a.t_us for a, b in zip(tail, tail[1:])} == {4000}


def test_c1_adc_reset(sim, tmp_path):
    scans, _, health, text = run(sim, tmp_path / "o.bin", "--end", "4", "--board-costs",
                                 "--cmd", "1.0:C1;", "--cmd", "1.0:M2;", "--cmd", "1.3:B",
                                 "--adc-reset", "2.0")
    assert any("adc reset" in t for t in text)              # the message crossed to core 0
    assert any(s.flags & ef.FLAG_REINIT for s in scans)
    after = [s for s in scans if s.t_us > scans[-1].t_us - 500_000]
    assert all(s.flags == 0 for s in after)
    assert {b.t_us - a.t_us for a, b in zip(after, after[1:])} == {4000}


def test_c1_command_spam_and_park(sim, tmp_path):
    """S/M/text/legacy commands while core 1 streams: each one parks the reader
    and hands it back; nothing hangs and every mode streams cleanly after."""
    cmds = []
    t = 1.3
    for i in range(12):
        cmds += ["--cmd", f"{t:.3f}:{['S012345;', 'S0123457;', 'M1;', 'M2;'][i % 4]}"]
        t += 0.15
    cmds += ["--cmd", f"{t:.3f}:g", "--cmd", f"{t + 0.2:.3f}:M0;", "--cmd", f"{t + 0.3:.3f}:B",
             "--cmd", f"{t + 0.6:.3f}:M2;", "--cmd", f"{t + 0.65:.3f}:B"]
    end = t + 2.0
    scans, infos, health, text = run(sim, tmp_path / "o.bin", "--end", f"{end:.3f}",
                                     "--board-costs", "--cmd", "1.0:C1;", "--cmd", "1.0:M2;",
                                     "--cmd", "1.3:B", *cmds)
    assert not any("park timeout" in x for x in text)
    assert any("GRID" in x or "W01A" in x for x in text)          # the text mode ran
    assert {i.mode for i in infos} >= {0, 1, 2}
    assert health[-1].parks >= 10 and health[-1].park_timeouts == 0
    tail = [s for s in scans if s.t_us > scans[-1].t_us - 1_000_000]
    assert {b.t_us - a.t_us for a, b in zip(tail, tail[1:])} == {4000}
    assert not [s for s in tail if s.flags]
    _health_ok(health[-1:] * 2, 32)


def test_c0_c1_switch(sim, tmp_path):
    """C1 -> C0 -> C1 while streaming quiet: HEALTH reports the core."""
    _, _, health, _ = run(sim, tmp_path / "o.bin", "--end", "6", "--cmd", "1.0:C1;",
                          "--cmd", "1.3:B", "--cmd", "3.0:C0;", "--cmd", "3.05:B",
                          "--cmd", "4.5:C1;", "--cmd", "4.55:B")
    cores = [h.core for h in health]
    assert 1 in cores[:2] and 0 in cores and cores[-1] == 1
    assert all(h.overrun == 0 for h in health)


@pytest.mark.parametrize("mcmd", ["M1;", "M1 200 256;"])
def test_c0_after_c1_like_fresh_boot(sim, tmp_path, mcmd):
    """C1 streaming, then C0 without a reboot (as on the bench): no stale or
    edgeless reads, no flags, nothing outside frames."""
    scans, _, health, _ = run(sim, tmp_path / "o.bin", "--end", "6", "--board-costs",
                              "--cmd", "1.0:S0123457;", "--cmd", "1.0:C1;", "--cmd", "1.0:M2;",
                              "--cmd", "1.3:B", "--cmd", "2.5:C0;", "--cmd", f"2.5:{mcmd}",
                              "--cmd", "2.55:B")
    c0 = [h for h in health if h.core == 0]
    assert len(c0) >= 2 and all(h.parks >= 1 for h in c0)
    assert sum(h.edgeless + h.stale + h.overrun + h.missed for h in c0) == 0
    quiet = [s for s in scans if s.dur_us == 16000]
    assert len(quiet) > 150 and not [s for s in quiet[1:] if s.flags]
    assert LAST["junk"] == 0


@pytest.mark.parametrize("setup", [["--cmd", "1.0:M1;"],
                                   ["--cmd", "1.0:C1;", "--cmd", "1.0:M2;"],
                                   ["--cmd", "1.0:C1 2000;", "--cmd", "1.0:M1;"]])
def test_idf_logs_are_framed(sim, tmp_path, setup):
    """An ESP-IDF component logging while streaming (C0 and C1): the line arrives
    as a framed TEXT record, a flood is rate-limited with a drop count, and no
    byte lands outside a frame."""
    logs = ["--idf-log", "2.0:task_wdt: test line"]
    for i in range(40):                                  # a flood: 40 lines in 4 ms
        logs += ["--idf-log", f"{2.5 + i * 0.0001:.4f}:flood {i}"]
    scans, _, health, text = run(sim, tmp_path / "o.bin", "--end", "4.5", "--board-costs",
                                 "--cmd", "1.0:S0123457;", *setup, "--cmd", "1.3:B", *logs)
    assert any("[idf] E (2000) task_wdt: test line" in t for t in text)
    assert any("log lines dropped" in t for t in text)
    assert 5 <= len([t for t in text if "flood" in t]) <= 20
    assert LAST["junk"] == 0
    assert sum(h.overrun for h in health[1:]) == 0


def test_twdt_mock_flags_unsubscribed_reset(sim, tmp_path):
    """The fake task watchdog is live: the firmware no longer calls it, so this
    only checks the sim reports zero (run() asserts it for every run)."""
    run(sim, tmp_path / "o.bin", "--end", "2", "--cmd", "1.0:C1;", "--cmd", "1.0:M2;",
        "--cmd", "1.3:B")
    assert "twdt errors 0" in LAST["stderr"]
