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


@pytest.mark.parametrize("mode,period,tol", [(2, 4000, 300), (1, 16000, 60)])
def test_locked_modes(sim, tmp_path, mode, period, tol):
    scans, infos, health, _ = run(sim, tmp_path / "o.bin", "--end", "3",
                                  "--cmd", "1.0:S0123457;", "--cmd", f"1.0:M{mode};",
                                  "--cmd", "1.3:B")
    cfg = ef.info_cfg(dict(infos[-1].__dict__))
    assert cfg["mode"] == mode and cfg["selects"] == "0123457" and cfg["clkin_hz"] == 8_000_000
    assert cfg["n_conv"] * 1e6 / cfg["fdata_sps"] == period
    assert len(scans) > 1.6e6 / period
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


def test_legacy_default_and_back(sim, tmp_path):
    scans, infos, health, _ = run(sim, tmp_path / "o.bin", "--end", "3", "--cmd", "0.9:B",
                                  "--cmd", "1.5:M2;", "--cmd", "2.2:M0;")
    assert [i.mode for i in infos] == [0, 2, 0]
    legacy = [s for s in scans if s.dur_us != 4000]
    assert len(legacy) > 50 and len([s for s in scans if s.dur_us == 4000]) > 100
