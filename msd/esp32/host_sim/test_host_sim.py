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


def run_events(sim, out: Path, *args: str) -> list:
    """Run the sim; every run must be free of task-watchdog errors and raw
    (unframed) ESP-IDF log lines (bench r3). The decoded events, in order."""
    r = subprocess.run([str(sim), "--out", str(out), *args], capture_output=True, text=True,
                       timeout=300)
    assert r.returncode == 0, r.stderr
    assert "twdt errors 0, raw log lines 0, reader kernel calls 0" in r.stderr, r.stderr[-500:]
    dec = ef.Decoder()
    evs = list(dec.feed(out.read_bytes()))
    LAST.update(junk=dec.n_junk, stderr=r.stderr, beats=[e for e in evs if isinstance(e, ef.Beat)])
    return evs


def run(sim, out: Path, *args: str):
    evs = run_events(sim, out, *args)
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
    assert infos[-1].mode == 2                          # still the boot default (wide)


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


def test_wide_boot_default_quiet_and_legacy(sim, tmp_path):
    """Since 2026-10-28: wide 250 Hz with selects 0123457 (mask 0xBF) after boot,
    no command but B; M1 / M0 / S; still switch."""
    scans, infos, health, _ = run(sim, tmp_path / "o.bin", "--end", "3.6", "--cmd", "0.9:B",
                                  "--cmd", "1.5:M1;", "--cmd", "2.2:S;M0;")
    assert [i.mode for i in infos][:2] == [2, 1] and infos[-1].mode == 0
    assert infos[0].sel_mask == 0xBF and infos[-1].sel_mask == 0xFF
    cfg = ef.info_cfg(dict(infos[0].__dict__))
    assert cfg["mode_name"] == "wide" and cfg["osr"] == 128 and cfg["scan_hz"] == 250.0
    assert cfg["selects"] == "0123457" and cfg["avg_per_sel"][6] == 0
    wide = [s for s in scans if s.dur_us == 4000]
    assert len(wide) > 100 and {b.t_us - a.t_us for a, b in zip(wide, wide[1:])} == {4000}
    assert all(b.seq == a.seq + 1 for a, b in zip(wide, wide[1:]))
    for s in wide[2:]:
        for o in range(1, 33):
            if o in SKIPPED6:
                assert s.railmask >> (o - 1) & 1
            else:
                assert abs(s.raw[o - 1] - expected(o)) < 300, (o, s.raw[o - 1] - expected(o))
    assert health[0].mode == 2 and health[0].overrun == 0 and health[0].missed == 0
    quiet = [s for s in scans if s.dur_us == 16000]
    assert len(quiet) > 20 and {b.t_us - a.t_us for a, b in zip(quiet, quiet[1:])} == {16000}
    legacy = [s for s in scans if s.dur_us not in (4000, 16000)]
    assert len(legacy) > 40
    assert all(not s.railmask >> (o - 1) & 1 for s in legacy[2:] for o in SKIPPED6)  # S; = all 8


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


@pytest.mark.parametrize("phase", ["0", "137", "437", "731"])
@pytest.mark.parametrize("scale", ["1.0", "1.5"])
@pytest.mark.parametrize("ccmd", ["C1;", "C1 100;"])
def test_c1_wide_board_costs(sim, tmp_path, scale, ccmd, phase):
    """Wide on its own core with the bench-calibrated costs: no overrun, lat < T = 32 us.
    Core 1's tick hits the same conversions every 4 ms scan; its phase (random per
    boot on the board) is swept: a tick on a slot end must not flag every scan."""
    scans, infos, health, _ = run(sim, tmp_path / "o.bin", "--end", "4", "--board-costs",
                                  "--tick-phase", phase,
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
                              "--cmd", "1.0:S0123457;", "--cmd", "1.0:M1;", "--cmd", "1.0:C1 2000;",
                              "--cmd", "1.3:B")
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


# ---------------------------------------------------------------- bench r4
def _adc_diag_cmd(selects, core, win, mode, osr=None):
    """The exact bytes adc_diag.py stream sends (its own builders)."""
    sys.path.insert(0, str(HERE.parent))
    import adc_diag as ad
    return ("S" + selects + ";" + ad.core_command(core, win)
            + ad.mode_command(mode, None, osr, None, None) + "B")


@pytest.mark.parametrize("core,win,mode", [(0, None, "quiet"), (1, None, "quiet"),
                                           (1, 100, "quiet"), (1, None, "wide")])
def test_adc_diag_sequence_late_start(sim, tmp_path, core, win, mode):
    """Bench r4 (6d90c76): every run sent S..;C..;M..;B more than 3 s after boot,
    got one INFO and no SCAN/HEALTH, then '[reader watchdog]' and a reboot: the
    watchdog's clock started at boot, not at the RUN. Start late, as the bench."""
    cmd = _adc_diag_cmd("0123457", core, win, mode)
    scans, infos, health, text = run(sim, tmp_path / "o.bin", "--end", "9", "--board-costs",
                                     "--cmd", f"5.0:{cmd}")
    assert not any("watchdog" in t for t in text), [t for t in text if "watchdog" in t]
    period = 4000 if mode == "wide" else 16000
    assert len(scans) > 0.7 * 3.5e6 / period and len(health) >= 2
    assert all(h.core == core for h in health)
    assert sum(h.overrun + h.stale + h.edgeless for h in health[1:]) == 0
    assert {b.t_us - a.t_us for a, b in zip(scans, scans[1:])} == {period}
    assert LAST["junk"] == 0


def test_second_run_after_late_restart_of_stream(sim, tmp_path):
    """Stop (text mode) and start streaming again much later: re-armed, no watchdog."""
    cmd = _adc_diag_cmd("0123457", 1, None, "quiet")
    scans, _, health, text = run(sim, tmp_path / "o.bin", "--end", "16", "--cmd", f"4.0:{cmd}",
                                 "--cmd", "6.0:g", "--cmd", f"11.0:{cmd}")
    assert not any("watchdog" in t for t in text)
    assert len([s for s in scans if s.t_us > 11.3e6]) > 200


def test_reader_watchdog_reports_state(sim, tmp_path):
    """A reader that really stops (core 1 stalled 4 s): the watchdog sends its
    state as framed TEXT, then restarts (the sim exits with code 4)."""
    out = tmp_path / "o.bin"
    r = subprocess.run([str(sim), "--out", str(out), "--end", "12", "--cmd", "1.0:C1;",
                        "--cmd", "1.3:B", "--stall", "3.0:4000000:1"],
                       capture_output=True, text=True, timeout=300)
    assert r.returncode == 4 and "esp_restart" in r.stderr
    text = [e.text for e in ef.Decoder().feed(out.read_bytes()) if isinstance(e, ef.Text)]
    wd = [t for t in text if "[reader watchdog]" in t]
    assert wd and "req" in wd[0] and "beat" in wd[0] and "drdy edges" in wd[0]


def test_reader_task_creation_fails(sim, tmp_path):
    """xTaskCreatePinnedToCore fails for the reader: reported, and locked modes
    still stream from the host core."""
    scans, _, health, text = run(sim, tmp_path / "o.bin", "--end", "5", "--fail-task", "lkReader",
                                 "--cmd", "4.0:S0123457;M1;B")
    assert any("reader task NOT created" in t for t in text)
    assert len([s for s in scans if s.dur_us == 16000]) > 40


def test_stress_resend_late_start_keeps_health(sim, tmp_path):
    """adc_diag --resend-every 1, first command 5 s after boot (bench r4 stress_c1):
    streams from the first B, and HEALTH still arrives although no 1 s window
    completes between re-sends (the partial window goes out at each park)."""
    cmd = _adc_diag_cmd("0123457", 1, None, "wide")
    cmds = []
    for i in range(6):
        cmds += ["--cmd", f"{5.0 + i:.1f}:{cmd}"]
    scans, _, health, text = run(sim, tmp_path / "o.bin", "--end", "11.5", "--board-costs", *cmds)
    assert not any("watchdog" in t for t in text)
    assert len(health) >= 5 and all(h.core == 1 for h in health)
    assert all(200 <= h.window_ms <= 1100 for h in health)
    assert len([s for s in scans if s.t_us < 6.0e6]) > 150      # the first B streams


def test_c1_is_boot_default(sim, tmp_path):
    """Since 2026-10-23 r5: C1 (core 0 sends) after boot, no C command; the boot
    mode is wide since 2026-10-28."""
    scans, infos, health, _ = run(sim, tmp_path / "o.bin", "--end", "7", "--board-costs",
                                  "--cmd", "4.0:S0123457;B")
    assert ef.info_cfg(dict(infos[-1].__dict__))["mode_name"] == "wide"
    assert health and all(h.core == 1 for h in health)
    assert len(scans) > 150 and not [s for s in scans[1:] if s.flags]


def test_c0_pending_scan_not_lost_on_command(sim, tmp_path):
    """Bench r5 audit: in C0 a closed scan waits (pendingSend) for idle time to be
    packed; a command that parks the reader then must send it, not drop it."""
    cmds = ["--cmd", "4.0:S0123457;C0;B"]
    for i in range(10):
        cmds += ["--cmd", f"{4.5 + i * 0.2371:.4f}:S0123457;"]
    scans, _, health, _ = run(sim, tmp_path / "o.bin", "--end", "7.5", "--board-costs", *cmds)
    gaps = [(a.seq, b.seq) for a, b in zip(scans, scans[1:]) if b.seq != a.seq + 1]
    lost = sum(b - a - 1 for a, b in gaps)
    # each S re-arms on the same grid: the partial scan it interrupts is skipped and
    # counted as a resync; every scan that was closed must arrive
    assert lost <= sum(h.resync for h in health) + 1, (gaps, [h.resync for h in health])
    assert sum(h.tx_drop for h in health) == 0


# ---------------------------------------------------------------- dead ADC, heartbeat (brief 6)
import re  # noqa: E402

from whisker_link.esp32_fault import FaultMonitor, classify  # noqa: E402

UNREC_RE = re.compile(r"^\[halt #\d+\] UNRECOVERED  ID=0x2405 CLOCKrb=0x00FF crcbad=\d+"
                      r"  -> power-cycle the PCB with the ESP32 already running$")


def timeline(evs):
    """(ESP32 time in s, event): from the heartbeat uptime and the scan times."""
    t, out = 0.0, []
    for e in evs:
        if isinstance(e, ef.Beat):
            t = e.uptime_ms / 1e3
        elif isinstance(e, ef.Scan):
            t = max(t, e.t_us / 1e6)
        out.append((t, e))
    return out


def feed_monitor(tl):
    """The host's fault model over the stream, on the ESP32's clock."""
    now = [0.0]
    mon = FaultMonitor(lambda: now[0], wall=lambda: 1.7e9 + now[0])
    mon.link(True, 0.0)
    for t, e in tl:
        while now[0] < t:
            now[0] = min(t, now[0] + 0.1)
            mon.poll()
        if isinstance(e, ef.Scan):
            mon.on_scan(e.flags)
        elif isinstance(e, ef.Text):
            mon.on_text(e.text)
        elif isinstance(e, ef.Beat):
            mon.on_beat(e)
    return mon


def _beat_gaps(beats, t0=0.0, t1=1e9):
    return [b.uptime_ms - a.uptime_ms for a, b in zip(beats, beats[1:]) if t0 <= a.uptime_ms / 1e3 < t1]


@pytest.mark.parametrize("setup,period", [("S0123457;M1;B", 16000), ("S0123457;M1;C0;B", 16000),
                                          ("S0123457;B", 4000), ("S0123457;M2;B", 4000)])
def test_dead_adc_locked_unrecovered_backoff_heartbeat(sim, tmp_path, setup, period):
    """The ADC locks up while a locked mode streams (no DRDY, the CLOCK write
    doesn't stick), the PCB is power-cycled 12 s later. Before: '[halt] no
    conversion' ~10x/s forever. Now: 5 of them, then the legacy UNRECOVERED line,
    then a retry (and the line) every 5 s; a heartbeat every second throughout."""
    evs = run_events(sim, tmp_path / "o.bin", "--end", "18", "--board-costs",
                     "--cmd", f"2.0:{setup}", "--adc-dead", "3.0:15.0")
    tl = timeline(evs)
    text = [(t, e.text) for t, e in tl if isinstance(e, ef.Text) and t > 2.0]
    halts = [x for x in text if x[1].startswith("[halt")]
    assert [x[1].startswith("[halt] no conversion") for x in halts] == [True] * 5 + [False] * 2, halts
    unrec = [x for x in halts if "UNRECOVERED" in x[1]]
    assert all(UNREC_RE.match(x) and classify(x)[1] == "adc_unrecovered" for _, x in unrec)
    assert 4.0 < unrec[0][0] < 6.5 and 4.5 < unrec[1][0] - unrec[0][0] < 5.6, unrec
    assert any("[adc ok]" in x for t, x in text if t > 14.0)
    beats = LAST["beats"]
    assert max(_beat_gaps(beats, 2.0)) <= 1350, _beat_gaps(beats, 2.0)
    dead = [b for b in beats if 4.5 < b.uptime_ms / 1e3 < 15.0]
    assert len(dead) >= 9 and all(b.drdy_edges == 0 and b.host_mode == "B" for b in dead)
    assert all(b.adc_down == (b.uptime_ms / 1e3 > unrec[0][0]) for b in dead)
    assert dead[-1].unrecovered == 2 and dead[-1].clock_rb == 0x00FF
    after = [b for b in beats if b.uptime_ms > 16_000]
    assert after and all(not b.adc_down and b.drdy_edges > 1000 for b in after)
    scans = [e for t, e in tl if isinstance(e, ef.Scan) and t > 16.0]
    assert len(scans) > 1.5e6 / period and {b.t_us - a.t_us for a, b in zip(scans, scans[1:])} == {period}
    assert LAST["junk"] == 0
    mon = feed_monitor(tl)
    assert [e["code"] for e in mon.take_events() if e["event"] == "fault"][-1] == "adc_unrecovered"


def test_dead_adc_legacy_heartbeat(sim, tmp_path):
    """Legacy scan: UNRECOVERED every failed sweep (~1.4 s, as before), beats keep coming."""
    evs = run_events(sim, tmp_path / "o.bin", "--end", "14", "--board-costs",
                     "--cmd", "2.0:S0123457;M0;B", "--adc-dead", "3.0:12.0")
    text = [e.text for e in evs if isinstance(e, ef.Text)]
    assert 3 <= len([x for x in text if UNREC_RE.match(x)]) <= 7
    assert any("recovered at L" in x for x in text)
    beats = LAST["beats"]
    assert max(_beat_gaps(beats, 2.0)) <= 1700
    assert any(b.adc_down for b in beats) and not beats[-1].adc_down and beats[-1].drdy_edges > 1000


def test_dead_adc_at_boot(sim, tmp_path):
    """Dead from power-up: the boot gives up on the baseline after 2 scans (not 16
    x 1-3 s), text-mode #BEAT lines carry the boot result, B streams the beat."""
    evs = run_events(sim, tmp_path / "o.bin", "--end", "16", "--adc-dead", "0.0001",
                     "--cmd", "13:B")
    text = [e.text for e in evs if isinstance(e, ef.Text)]
    assert any("Baseline FAILED" in x for x in text) and any("Boot recovery: FAILED" in x for x in text)
    beats = LAST["beats"]
    boot = [b for b in beats if b.host_mode != "B"]
    assert boot and boot[0].uptime_ms < 2500 and max(_beat_gaps(beats)) < 2500
    late = [b for b in beats if b.host_mode == "B"]
    assert late and all(b.flags & ef.BEAT_BOOT_REC_FAIL and not b.flags & ef.BEAT_BOOT_CLOCK_OK
                        and b.boot_clock_rb == 0x00FF and b.drdy_edges == 0 and b.adc_down
                        for b in late)
    assert any(isinstance(e, ef.Info) and e.mode == 0 for e in evs)      # back to legacy


def test_dead_adc_with_host_kicks(sim, tmp_path):
    """daq_server re-sends S..;M1;B every 2 s while nothing streams: no park
    timeout (a recovery blocks the reader ~0.4 s), it recovers at the power cycle."""
    cmds = ["--cmd", "2.0:S0123457;B"]
    for i in range(6):
        cmds += ["--cmd", f"{3.5 + 2 * i:.1f}:S0123457;M1;B"]
    evs = run_events(sim, tmp_path / "o.bin", "--end", "17", "--board-costs", *cmds,
                     "--adc-dead", "3.0:14.2")
    text = [e.text for e in evs if isinstance(e, ef.Text)]
    assert not any("park timeout" in x for x in text)
    assert any(UNREC_RE.match(x) for x in text)
    # (an M with a dead ADC holds the host ~0.9 s: a failed init and the one back;
    # no beat inside a command, which could land in a diagnostic's dump)
    assert max(_beat_gaps(LAST["beats"], 2.0)) <= 2100
    scans = [e for e in evs if isinstance(e, ef.Scan) and e.t_us > 15.5e6]
    assert len(scans) > 60 and not [s for s in scans if s.flags]


@pytest.mark.parametrize("cmd,hmode,mode,conv", [("S0123457;B", "B", 2, 31250),   # boot: wide
                                                 ("S0123457;M1;B", "B", 1, 7812),
                                                 ("S0123457;M0;B", "B", 0, 3125), ("g", "g", 2, 31250)])
def test_heartbeat_healthy(sim, tmp_path, cmd, hmode, mode, conv):
    """What the board check looks at: one beat a second in every mode (a #BEAT line
    in a text mode), DRDY edges at the conversion rate, the boot result."""
    evs = run_events(sim, tmp_path / "o.bin", "--end", "7", "--board-costs", "--cmd", f"2.0:{cmd}")
    beats = [b for b in LAST["beats"] if b.uptime_ms > 3000]
    assert len(beats) >= 3 and all(900 <= g <= 1250 for g in _beat_gaps(beats))
    for b in beats:
        assert b.host_mode == hmode and b.mode == mode and b.core == 1 and b.reset_reason == 1
        assert b.flags == ef.BEAT_BOOT_CLOCK_OK and b.boot_clock_rb == 0xFF02 and b.device_id == 0x2405
        assert abs(b.conv_hz - conv) < 0.03 * conv and b.halts == b.unrecovered == 0
    if hmode == "B":
        assert LAST["junk"] == 0
        scans = [e for e in evs if isinstance(e, ef.Scan)]
        assert scans and not [s for s in scans[2:] if s.flags]


def test_previous_watchdog_note_is_framed_once(sim, tmp_path):
    """After a reader-watchdog restart: the banner prints the note raw, the beat
    flags it, and the first B sends it once as framed TEXT (a host that connects
    after the boot sees it)."""
    note = "no progress for 3001 ms: req 2/RUN state 1/RUNNING beat 77"
    evs = run_events(sim, tmp_path / "o.bin", "--end", "6", "--prev-wd", note,
                     "--cmd", "2.0:B", "--cmd", "3.0:g", "--cmd", "4.0:B")
    lines = [e.text for e in evs if isinstance(e, ef.Text) and note in e.text]
    assert lines == [f"previous reset: [reader watchdog] {note}"] * 2      # raw banner + framed
    assert all(b.flags & ef.BEAT_PREV_WD for b in LAST["beats"])
    mon = feed_monitor(timeline(evs))
    assert [e["code"] for e in mon.take_events()].count("reader_watchdog") == 1
