"""Tests for burst_spectrum.py on synthetic dumps (no hardware).

Run: python -m pytest msd/esp32/test_burst_spectrum.py
"""

import os
import sys
import threading
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import burst_spectrum as bs  # noqa: E402

TONE_COMMON = 456.0     # on all 4 channels, 0.2 mV
TONE_CH0 = 60.0         # CH0 only, 0.5 mV


def fake_dump(osr=128, n=8192, clkin=800_000, sel=2, seed=0, bad=(100,), railed_ch=None) -> bytes:
    """A dump laid out exactly as burst_capture() prints it."""
    rng = np.random.default_rng(seed)
    fs = clkin / 2 / osr
    t = np.arange(n) / fs
    mv = np.empty((n, 4))
    for c in range(4):
        mv[:, c] = (5.0 + c + 0.2 * np.sin(2 * np.pi * TONE_COMMON * t + c)
                    + 0.02 * rng.standard_normal(n))
    mv[:, 0] += 0.5 * np.sin(2 * np.pi * TONE_CH0 * t)
    codes = np.round(mv / bs.MV_PER_CODE).astype(np.int64)
    if railed_ch is not None:
        codes[:, railed_ch] = 8388607
    flags = np.zeros(n, int)
    for i in bad:
        flags[i] = bs.BF_CRC
        codes[i] = [999999, -999999, 999999, -999999]   # garbage the analysis must drop
    t_us = np.round(t * 1e6).astype(np.int64)
    head = (f"#BURST v1 sel={sel} n={n} osr={osr} clkin_hz={clkin} fdata_sps={fs:.3f}"
            f" spi_hz=600000 clock_rb=0x0F02 out=5,7,3,1 mv_per_code={bs.MV_PER_CODE:.12g}"
            f" crc_bad={len(bad)} stale=0 overrun=0 edgeless=0 adc_reset=0\n").encode()
    body = b"".join(f"{t_us[i]} {flags[i]} {codes[i,0]} {codes[i,1]} {codes[i,2]} {codes[i,3]}\n"
                    .encode() for i in range(n))
    end = f"#END lines={n} crc={bs.crc16_ccitt(body):04X}\n".encode()
    return b"\x00\xa5\x5agrid junk\r\n" + head + body + end + b"\r\n-- GRID --\r\n"


def test_parse_roundtrip(tmp_path):
    (b,) = bs.parse_dump(fake_dump(n=1000))
    assert b.meta["osr"] == 128 and b.meta["out"] == [5, 7, 3, 1] and b.codes.shape == (1000, 4)
    bs.save_npz(tmp_path / "x.npz", [b])
    (b2,) = bs.load(tmp_path / "x.npz")
    assert b2.meta == b.meta and np.array_equal(b2.codes, b.codes)
    assert np.array_equal(b2.t_us, b.t_us) and np.array_equal(b2.flags, b.flags)


def test_damage_detected():
    d = bytearray(fake_dump(n=500))
    i = d.find(b"\n", d.find(b"#BURST")) + 5
    d[i] = ord("9") if d[i] != ord("9") else ord("8")
    with pytest.raises(ValueError, match="CRC"):
        bs.parse_dump(bytes(d))
    with pytest.raises(ValueError, match="no #END"):
        bs.parse_dump(fake_dump(n=500).split(b"#END")[0])
    with pytest.raises(ValueError, match="firmware"):
        bs.parse_dump(b"junk\n#BURSTERR bad select\n")


@pytest.mark.parametrize("osr", [128, 512])
def test_tones_found(osr):
    (b,) = bs.parse_dump(fake_dump(osr=osr))
    s = bs.spectrum(b, None, 6.0, 8)
    want = bs.alias(TONE_COMMON, s.fs)          # 456 at 3125 SPS, 325.25 at 781.25 SPS
    df = s.f[1] - s.f[0]
    common = [f for f, k in bs.common_peaks(s) if k == 4]
    assert any(abs(f - want) < df for f in common), (common, want)
    for c in range(4):
        f, a = min(s.peaks[c], key=lambda p: abs(p[0] - want))
        assert abs(f - want) < 0.5 * df and a == pytest.approx(0.2, rel=0.15)
    f, a = min(s.peaks[0], key=lambda p: abs(p[0] - TONE_CH0))
    assert abs(f - TONE_CH0) < 0.5 * df and a == pytest.approx(0.5, rel=0.15)
    assert not any(abs(f - TONE_CH0) < 2 * df for f, _ in s.peaks[1])
    assert any("interpolated" in n for n in s.notes)


def test_alias_matches_scan():
    # what the 76.2 Hz scan would show: 456 = 6 x 76.2 - 1.2
    assert bs.alias(456.0, 76.2) == pytest.approx(1.2)
    assert bs.alias(1000.0, 76.2) == pytest.approx(1000.0 - 13 * 76.2)


def test_report_files(tmp_path):
    pytest.importorskip("matplotlib")
    bursts = bs.parse_dump(fake_dump(osr=128, railed_ch=3) + fake_dump(osr=512, n=4096))
    specs = bs.report(bursts, tmp_path / "rep", 76.2, None, 6.0, 8)
    assert 3 not in specs[0].amp and any("railed" in n for n in specs[0].notes)
    assert (tmp_path / "rep.png").stat().st_size > 10_000
    html = (tmp_path / "rep.html").read_text()
    assert "data:image/png;base64," in html and "455.9" in html


def test_capture_over_pty(tmp_path):
    """capture() against a fake firmware on a pseudo-terminal."""
    pytest.importorskip("serial")
    master, slave = os.openpty()
    cmds = []

    def firmware():
        buf = b""
        while len(cmds) < 2:
            buf += os.read(master, 256)
            while b";" in buf:
                line, buf = buf.split(b";", 1)
                line = line[line.rfind(b"r") + 1:]
                sel, n, osr, clk, spi = map(int, line.split())
                cmds.append((sel, n, osr, clk, spi))
                os.write(master, b"\x00binary junk\n" + fake_dump(osr=osr, n=n, sel=sel, clkin=clk))

    th = threading.Thread(target=firmware, daemon=True)
    th.start()
    plan = bs.parse_plan(["128", "1000000:1024"])
    assert plan == [(800_000, 128), (1_000_000, 1024)]
    bursts, raw = bs.capture(os.ttyname(slave), 3, 2048, plan, 600_000)
    assert cmds == [(3, 2048, 128, 800000, 600000), (3, 2048, 1024, 1000000, 600000)]
    assert [b.meta["osr"] for b in bursts] == [128, 1024] and raw.count(b"#END") == 2
    assert bursts[1].meta["clkin_hz"] == 1_000_000
    assert bs.main(["analyze", str(_save(tmp_path, raw)), "--out", str(tmp_path / "r")]) == 0


def _save(tmp_path, raw):
    p = tmp_path / "raw.txt"
    p.write_bytes(raw)
    return p


def test_resolve_true_frequency():
    """Two CLKINs pin the tone down; OSR-only bursts at one CLKIN cannot."""
    mixed = bs.parse_dump(fake_dump(clkin=800_000, osr=128) + fake_dump(clkin=1_000_000, osr=128)
                          + fake_dump(clkin=8_000_000, osr=2048, n=4096))
    specs = [bs.spectrum(b, None, 6.0, 8) for b in mixed]
    found = [f for f, _, _ in bs.resolve(specs, 5000.0)]
    assert any(abs(f - TONE_COMMON) < 1.0 for f in found)
    assert any(abs(f - TONE_CH0) < 1.0 for f in found)
    assert all(min(abs(f - TONE_COMMON), abs(f - TONE_CH0)) < 1.0 for f in found), found

    nested = bs.parse_dump(fake_dump(osr=128) + fake_dump(osr=512))
    specs = [bs.spectrum(b, None, 6.0, 8) for b in nested]
    assert "NOTE all rates are integer multiples" in bs._resolve_text(specs, bs.resolve(specs, 5000))
