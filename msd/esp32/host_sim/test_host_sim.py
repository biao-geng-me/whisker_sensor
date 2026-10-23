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


def _cxx() -> str | None:
    for c in (os.environ.get("CXX"), "g++", "c++"):
        if c and shutil.which(c):
            try:
                if subprocess.run([c, "--version"], capture_output=True, timeout=20).returncode == 0:
                    return c
            except (OSError, subprocess.TimeoutExpired):
                pass
    return None


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


def run(sim, out: Path, *args: str):
    r = subprocess.run([str(sim), "--out", str(out), *args], capture_output=True, text=True,
                       timeout=300)
    assert r.returncode == 0, r.stderr
    evs = list(ef.Decoder().feed(out.read_bytes()))
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
        assert sum(h.overrun for h in health) <= 1, what
        assert sum(h.resync for h in health) <= 3, (what, [h.resync for h in health])
        tail = [s for s in scans if s.t_us > scans[-1].t_us - 300_000]
        assert all(s.flags == 0 for s in tail), what
        for s in tail:
            for o in range(1, 33):
                if o not in SKIPPED6:
                    assert abs(s.raw[o - 1] - expected(o)) < (300 if period == 4000 else 60)
