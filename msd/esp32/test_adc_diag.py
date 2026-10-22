"""Tests for adc_diag.py on synthetic firmware output (no hardware).

Run: python -m pytest msd/esp32/test_adc_diag.py
"""

import os
import sys
import threading
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import adc_diag as ad  # noqa: E402

# per ADC channel: (pre mV, final mV); CH2 has no step
LEVELS = [(1199.0, 26.0), (300.0, -50.0), (5.0, 5.0), (-800.0, 400.0)]
KICK, TAU, DELAY = 3e-4, 100.0, 3.0


def fake_settle(osr=256, clkin=8_000_000, reps=64, dither=4, window=2000, noise=0.03,
                seed=0, bad_rows=(10, 500)) -> bytes:
    """'D' output with a charge-sharing tail of KICK x (pre - final), tau TAU."""
    rng = np.random.default_rng(seed)
    period = 2e6 * osr / clkin
    filt = ad.Sinc3(osr, clkin)
    rows = []
    for r in range(reps):
        phi = 20.0 + (r % dither) * period / dither          # last pre conversion ended at -phi
        k = np.arange(-3, int(window / period) + 3)
        te = np.round(-phi + k * period)
        te = te[te <= window + period]
        te = te[: np.searchsorted(te, window, side="right") + 1]
        vals = []
        for pre, fin in LEVELS:
            v = (pre + (fin - pre) * filt.step(te - DELAY)
                 + KICK * (pre - fin) * filt.exp_tail(te - DELAY, TAU)
                 + noise * rng.standard_normal(len(te)))
            vals.append(np.round(v / ad.MV_PER_CODE).astype(np.int64))
        for i, t in enumerate(te):
            rows.append([r, int(t), 0] + [int(vals[c][i]) for c in range(4)])
    bad_rows = [i for i in bad_rows if i < len(rows)]
    for i in bad_rows:
        rows[i][2] = ad.BF_CRC
        rows[i][3:] = [999999, -999999, 999999, -999999]
    body = b"".join((" ".join(map(str, x)) + "\n").encode() for x in rows)
    head = (f"#SETTLE v1 from=6 to=2 out_from=27,19,6,16 out_to=28,22,3,11 clkin_hz={clkin}"
            f" osr={osr} fdata_sps={clkin / 2 / osr:.3f} period_us={period:.3f} spi_hz=4000000"
            f" spi_actual=4000000 reps={reps} window_us={window} dither={dither} dwell_us=3000"
            f" mux_us=2 clock_rb=0x0F06 mv_per_code={ad.MV_PER_CODE:.12g}\n").encode()
    end = (f"#END lines={len(rows)} crc={ad.crc16_ccitt(body):04X} crc_bad={len(bad_rows)}"
           f" stale=0 overrun=0 missed=0 edgeless=0 reset=0 read_us_max=9 lat_us_max=17\n").encode()
    return b"grid junk\r\n" + head + body + end


SPI_OUT = b"""junk
#SPISWEEP v1 clkin_hz=8000000 osr=256 fdata_sps=15625.000 period_us=64.000 secs=2 steps=5
#SPI spi_hz=600000 spi_actual=600000 clock_ok=1 frames=31000 expected=31250 crc_bad=0 stale=0 overrun=120 missed=250 edgeless=40 reset=0 timeout=0 read_us_avg=246.0 read_us_max=250 lat_us_avg=260.0 lat_us_max=300
#SPI spi_hz=2000000 spi_actual=2000000 clock_ok=1 frames=31250 expected=31250 crc_bad=0 stale=0 overrun=0 missed=0 edgeless=0 reset=0 timeout=0 read_us_avg=76.0 read_us_max=80 lat_us_avg=50.0 lat_us_max=60
#SPI spi_hz=4000000 spi_actual=4000000 clock_ok=1 frames=31250 expected=31250 crc_bad=0 stale=0 overrun=0 missed=0 edgeless=0 reset=0 timeout=0 read_us_avg=40.0 read_us_max=44 lat_us_avg=48.0 lat_us_max=55
#SPI spi_hz=6000000 spi_actual=6153846 clock_ok=1 frames=31250 expected=31250 crc_bad=0 stale=0 overrun=0 missed=0 edgeless=0 reset=0 timeout=0 read_us_avg=28.0 read_us_max=30 lat_us_avg=36.0 lat_us_max=40
#SPI spi_hz=8000000 spi_actual=8000000 clock_ok=0 frames=31250 expected=31250 crc_bad=812 stale=3 overrun=0 missed=0 edgeless=0 reset=0 timeout=0 read_us_avg=23.0 read_us_max=25 lat_us_avg=30.0 lat_us_max=33
#SPIEND
"""


def test_settle_parse_and_damage():
    s = ad.parse_settle(fake_settle(reps=4))
    assert s.meta["from"] == 6 and s.meta["out_to"] == [28, 22, 3, 11] and s.codes.shape[1] == 4
    d = bytearray(fake_settle(reps=4))
    i = d.find(b"\n", d.find(b"#SETTLE")) + 3
    d[i] = ord("7") if d[i] != ord("7") else ord("8")
    with pytest.raises(ValueError, match="CRC"):
        ad.parse_settle(bytes(d))
    with pytest.raises(ValueError, match="firmware"):
        ad.parse_settle(b"x\n#SETTLEERR bad select\n")


def test_sinc3_step():
    f = ad.Sinc3(256, 8e6)
    assert f.length_us == pytest.approx(3 * 64, rel=0.02)           # 3 conversions
    assert f.step(np.array([-1.0, 0.0]))[0] == 0 and f.step(np.array([200.0]))[0] == 1
    assert f.step(np.array([96.0]))[0] == pytest.approx(0.5, abs=0.01)   # symmetric
    assert f.exp_tail(np.array([5000.0]), 100.0)[0] == pytest.approx(0, abs=1e-12)


def test_settle_fit_recovers_tail(tmp_path):
    s = ad.parse_settle(fake_settle())
    fits, notes = ad.analyze(s)
    assert any("2 of" in n for n in notes)                     # CRC rows dropped
    for f, (pre, fin) in zip(fits, LEVELS):
        assert f.pre == pytest.approx(pre, abs=0.02) and f.final == pytest.approx(fin, abs=0.02)
        assert f.delay_us == pytest.approx(DELAY, abs=1.5)
        want = KICK * (pre - fin)
        if abs(want) > 0.05:
            assert f.significant
            assert f.amp_mv == pytest.approx(want, rel=0.15)
            assert f.tau_us == pytest.approx(TAU, rel=0.2)
            assert 300 < f.t_settle_us < 1100
        else:
            assert not f.significant and f.t_settle_us < 250
    ad.save_settle_npz(tmp_path / "s.npz", s)
    s2 = ad.load_settle(tmp_path / "s.npz")
    assert s2.meta == s.meta and np.array_equal(s2.codes, s.codes)
    ad.settle_report(s2, tmp_path / "rep")
    html = (tmp_path / "rep.html").read_text(encoding="utf-8")
    assert "Mux settle" in html and "tau_us" in html


def test_spi_summary():
    head, steps = ad.parse_spi(SPI_OUT)
    assert head["period_us"] == 64.0 and len(steps) == 5
    text, rec = ad.spi_summary(head, steps)
    assert "fastest clean SPI clock: 6 MHz" in text and rec == 4_000_000
    assert "FALLS BEHIND" in text.splitlines()[2]           # 600 kHz can't keep up
    with pytest.raises(ValueError, match="firmware"):
        ad.parse_spi(b"#SPIERR secs out of 1-60\n")


def test_commands_over_pty(tmp_path, capsys):
    """spi, settle and selects against a fake firmware on a pseudo-terminal."""
    pytest.importorskip("serial")
    master, slave = os.openpty()
    got = []

    def firmware():
        buf = b""
        while len(got) < 3:
            buf += os.read(master, 256)
            while b";" in buf:
                line, buf = buf.split(b";", 1)
                line = line.lstrip(b"g")
                got.append(line.decode())
                if line.startswith(b"p"):
                    os.write(master, SPI_OUT)
                elif line.startswith(b"D"):
                    os.write(master, fake_settle(reps=16))
                elif line.startswith(b"S"):
                    os.write(master, b"selects: 0123457 (7 of 8; skipped outputs read as railed/open)\r\n")

    threading.Thread(target=firmware, daemon=True).start()
    port = os.ttyname(slave)
    assert ad.main(["spi", "--port", port, "--secs", "1", "--out", str(tmp_path / "sp")]) == 0
    assert ad.main(["settle", "--port", port, "--from", "6", "--to", "2", "--spi", "4000000",
                    "--reps", "16", "--out", str(tmp_path / "st")]) == 0
    assert ad.main(["selects", "--port", port, "7012345"]) == 0
    assert got[0] == "p8000000 256 1 600000 1000000 2000000 3000000 4000000 5000000 6000000 8000000"
    assert got[1] == "D6 2 8000000 256 4000000 16 2000 4 3000"
    assert got[2] == "S0123457"
    assert (tmp_path / "sp.csv").read_text(encoding="utf-8").startswith("spi_hz,")
    assert (tmp_path / "st.npz").exists() and (tmp_path / "st.html").exists()
    out = capsys.readouterr().out
    assert "one step of margin: 4 MHz" in out and "selects: 0123457" in out
