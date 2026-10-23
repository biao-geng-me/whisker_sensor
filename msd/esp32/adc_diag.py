#!/usr/bin/env python3
"""ADC link and mux-settling diagnostics for whisker_daq.ino (fast-scan prep).

Subcommands (the ESP32 on a serial port, e.g. COM5 or /dev/ttyUSB0):

``spi``      SPI speed sweep (firmware ``p``): per SPI clock, read every
             conversion for N s at CLKIN 8 MHz / OSR 256 and report frames, CRC
             errors, stale / overrun / edgeless reads and the read time. Prints a
             table, saves ``<out>.txt`` (raw) and ``<out>.csv``.
``settle``   Fine mux settle diagnostic (firmware ``D``): switch from-select ->
             to-select many times, read every conversion, save ``<out>.txt`` /
             ``<out>.npz`` and analyze (below).
``analyze``  Analyze a saved settle capture (.txt or .npz), e.g. on the cluster.
``selects``  Set the scanned mux selects (firmware ``S``), e.g. ``0123457`` or
             ``all``. The ESP32 forgets it on reboot.
``mode``     Set the scan mode (firmware ``M``): quiet (62.5 Hz locked, OSR 512,
             the boot default), legacy (~76 Hz) or wide (250 Hz locked), with
             optional --settle-us / --osr / --period-us / --spi. The ESP32
             forgets it on reboot.
``stream``   Optionally set selects and mode, stream for N s and print each
             1 s HEALTH record (busy %, DRDY-to-read latency, overruns, ...) and a
             summary of the scans (rate, t_us spacing, seq gaps, flags).

Settle analysis, per ADC channel with a step: ``pre`` = mean of the conversions
that ended before the switch, ``final`` = mean over the end of the window. The
analog input after the switch (u = t - d > 0) is modelled as a fast
first-order response carrying the whole step plus a slow tail,
pre + step*(1 - exp(-u/tau_a)) + A2*exp(-u/tau2), and seen through the
ADC's sinc3 filter (computed for the same OSR and CLKIN). d is the latency
(mux, DRDY edge, ISR). Each channel is fitted with |A2| bounded to 5 % of the
step. The weights include DRDY timestamp jitter (--sigma-t-us), which
dominates while the filtered signal moves by mV/us. The report gives d,
tau_a, A2, tau2, the analog time at which the slow tail (and the fast part)
fall below 3e-4 and 1e-4 of the step, and a residual table by time window
(data minus the fast-only model, mean +- sem, beside the slow-tail fit), as
text, ``<out>.html`` and (with matplotlib) ``<out>.png``.

Needs numpy and scipy, pyserial for the commands that talk to the ESP32, and
matplotlib only for the plot. All files are written as UTF-8.
"""

from __future__ import annotations

import argparse
import base64
import csv
import html
import io
import json
import re
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

MV_PER_CODE = 1.2 * 1000.0 / (1 << 23)      # 1.430511474609375e-4 mV/code
BF_CRC, BF_STALE, BF_OVERRUN, BF_EDGELESS, BF_RESET = 0x01, 0x02, 0x04, 0x08, 0x10
BAD = BF_CRC | BF_STALE | BF_RESET | BF_OVERRUN | BF_EDGELESS
DEFAULT_SPIS = [600_000, 1_000_000, 2_000_000, 3_000_000, 4_000_000, 5_000_000,
                6_000_000, 8_000_000]


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def parse_kv(line: str, skip: int = 1) -> dict:
    """``#TAG v1 a=1 b=0x0F02 c=5,7,3,1 d=3.5`` -> dict (lists for comma values)."""
    out = {}
    for tok in line.split()[skip:]:
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        out[k] = [_num(x) for x in v.split(",")] if "," in v else _num(v)
    return out


def _num(v: str):
    try:
        return int(v, 16) if v.startswith("0x") else int(v)
    except ValueError:
        try:
            return float(v)
        except ValueError:
            return v


# -------------------------------------------------------------------- serial

def open_port(port: str, baud: int = 921600):
    import serial

    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, baud, 0.2
    ser.dtr = False          # don't pulse EN: the ESP32 would reboot
    ser.rts = False
    ser.open()
    return ser


def run_command(port: str, cmd: str, end_tags: tuple[bytes, ...], budget_s: float,
                quiet_first: bool = True) -> bytes:
    """Send ``cmd``; collect bytes until a line starting with one of end_tags."""
    ser = open_port(port)
    try:
        if quiet_first:
            ser.write(b"g")                      # leave binary mode, if in it
            time.sleep(0.3)
        ser.reset_input_buffer()
        ser.write(cmd.encode())
        raw, t0 = b"", time.monotonic()
        while time.monotonic() - t0 < budget_s:
            raw += ser.read(ser.in_waiting or 1)
            j = max(raw.rfind(t) for t in end_tags)
            if j >= 0 and raw.find(b"\n", j) >= 0:
                return raw
        raise TimeoutError(f"no {end_tags[0].decode()} within {budget_s:.0f} s; got"
                           f" {len(raw)} bytes: {raw[-200:]!r}")
    finally:
        ser.close()


def _lines(raw: bytes) -> list[bytes]:
    return [ln[:-1] if ln.endswith(b"\r") else ln for ln in raw.split(b"\n")]


def _check_err(raw: bytes, tag: bytes) -> None:
    for ln in _lines(raw):
        i = ln.find(tag)
        if i >= 0:
            raise ValueError("firmware: " + ln[i:].decode(errors="replace"))


# ----------------------------------------------------------------- SPI sweep

def parse_spi(raw: bytes) -> tuple[dict, list[dict]]:
    _check_err(raw, b"#SPIERR")
    head, steps, done = None, [], False
    for ln in _lines(raw):
        s = ln.decode(errors="replace")
        i = s.find("#SPISWEEP")
        if i >= 0:
            head = parse_kv(s[i:], 2)
        elif s.startswith("#SPI "):
            steps.append(parse_kv(s))
        elif s.startswith("#SPIEND"):
            done = True
    if head is None or not done:
        raise ValueError("SPI sweep output incomplete (no #SPISWEEP / #SPIEND)")
    return head, steps


def step_clean(st: dict) -> bool:
    return (st.get("clock_ok") == 1 and st.get("timeout") == 0 and st.get("crc_bad") == 0
            and st.get("stale") == 0 and st.get("reset") == 0 and st.get("frames", 0) > 0)


def step_keeps_up(st: dict, period_us: float) -> bool:
    return (st.get("overrun") == 0 and st.get("missed") == 0 and st.get("edgeless") == 0
            and st.get("lat_us_max", 1e9) < period_us)


def spi_summary(head: dict, steps: list[dict]) -> tuple[str, int | None]:
    p = float(head["period_us"])
    cols = ["spi_hz", "spi_actual", "frames", "expected", "crc_bad", "stale", "overrun", "missed",
            "edgeless", "reset", "read_us_avg", "read_us_max", "lat_us_avg", "lat_us_max"]
    lines = [f"CLKIN {head['clkin_hz']/1e6:g} MHz, OSR {head['osr']}: {head['fdata_sps']:.1f} SPS,"
             f" period {p:.1f} us, {head['secs']} s per step",
             "  ".join(f"{c:>10}" for c in cols) + "  verdict"]
    for st in steps:
        verdict = ("CLEAN" if step_clean(st) else "ERRORS") + (
            ", keeps up" if step_keeps_up(st, p) else ", FALLS BEHIND")
        if st.get("clock_ok") == 0:
            verdict += ", CLOCK readback wrong"
        lines.append("  ".join(f"{st.get(c, ''):>10}" for c in cols) + "  " + verdict)
    # Fastest SPI clock that is clean, with every slower step clean as well.
    best = None
    for st in sorted(steps, key=lambda s: s["spi_hz"]):
        if not step_clean(st):
            break
        best = st
    rec = None
    if best is None:
        lines.append("no clean step: check the wiring / ground before going faster")
    else:
        clean = [s for s in sorted(steps, key=lambda s: s["spi_hz"]) if s["spi_hz"] <= best["spi_hz"]]
        rec = clean[-2]["spi_hz"] if len(clean) >= 2 else best["spi_hz"]
        lines.append(f"fastest clean SPI clock: {best['spi_hz']/1e6:g} MHz"
                     f" ({'keeps up' if step_keeps_up(best, p) else 'but reads FALL BEHIND'}"
                     f" at {p:.1f} us); one step of margin: {rec/1e6:g} MHz")
        ok = [s for s in clean if s["spi_hz"] <= rec and step_keeps_up(s, p)]
        if not ok:
            lines.append(f"no clean step keeps up with a {p:.0f} us period: use a higher OSR")
    return "\n".join(lines), rec


def cmd_spi(a) -> int:
    spis = a.spi or DEFAULT_SPIS
    cmd = f"p{a.clkin} {a.osr} {a.secs} " + " ".join(str(s) for s in spis) + ";"
    print(f"SPI sweep: {cmd}  (~{len(spis) * (a.secs + 1)} s)", flush=True)
    raw = run_command(a.port, cmd, (b"#SPIEND", b"#SPIERR"), len(spis) * (a.secs + 3) + 20)
    a.out.with_suffix(".txt").write_bytes(raw)
    head, steps = parse_spi(raw)
    text, rec = spi_summary(head, steps)
    print(text)
    with open(a.out.with_suffix(".csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(steps[0].keys()) if steps else ["spi_hz"])
        w.writeheader()
        w.writerows(steps)
    print(f"saved {a.out.with_suffix('.txt')} and {a.out.with_suffix('.csv')}")
    if rec:
        print(f"next: python adc_diag.py settle --port {a.port} --from 6 --to 2 --spi {rec}"
              f" --out settle_6to2")
    return 0


# ------------------------------------------------------------ settle capture

@dataclass
class Settle:
    meta: dict
    end: dict
    rep: np.ndarray        # (n,) rep index
    t_us: np.ndarray       # (n,) DRDY edge minus switch, us (negative = before)
    flags: np.ndarray      # (n,)
    codes: np.ndarray      # (n, 4)
    notes: list = field(default_factory=list)


def parse_settle(raw: bytes) -> Settle:
    _check_err(raw, b"#SETTLEERR")
    meta, rows, crc = None, [], 0xFFFF
    for ln in _lines(raw):
        if meta is None:
            i = ln.find(b"#SETTLE ")
            if i >= 0:
                meta = parse_kv(ln[i:].decode(), 2)
            continue
        if ln.startswith(b"#END"):
            end = parse_kv(ln.decode())
            if end.get("lines") != len(rows):
                raise ValueError(f"settle: {len(rows)} lines, #END says {end.get('lines')}")
            want = re.search(rb"crc=([0-9A-Fa-f]{1,4})\b", ln)
            if not want or int(want.group(1), 16) != crc:
                raise ValueError(f"settle: data CRC {crc:04X} != {end.get('crc')} (serial damage)")
            a = np.array(rows, dtype=np.int64).reshape(-1, 7)
            return Settle(meta, end, a[:, 0], a[:, 1], a[:, 2], a[:, 3:7])
        crc = crc16_ccitt(ln + b"\n", crc)
        rows.append([int(x) for x in ln.split()])
    raise ValueError("settle: no #SETTLE header / #END (capture cut short?)")


def save_settle_npz(path: Path, s: Settle) -> None:
    np.savez_compressed(path, meta=np.array(json.dumps(s.meta)), end=np.array(json.dumps(s.end)),
                        rep=s.rep, t_us=s.t_us, flags=s.flags, codes=s.codes)


def load_settle(path: Path) -> Settle:
    if path.suffix == ".npz":
        z = np.load(path)
        return Settle(json.loads(str(z["meta"])), json.loads(str(z["end"])), z["rep"], z["t_us"],
                      z["flags"], z["codes"])
    return parse_settle(path.read_bytes())


# ------------------------------------------------------------ settle analysis

def sinc3_taps(osr: int) -> np.ndarray:
    """sinc3 impulse response (3 boxes of OSR modulator samples), normalized."""
    box = np.ones(osr)
    h = np.convolve(np.convolve(box, box), box)
    return h / h.sum()


class Sinc3:
    """Step and filtered-exponential responses of the ADC's sinc3 filter.

    Times in us relative to the analog event; a conversion that ENDS at t has
    weights h[n] on the modulator sample at t - n * tmod.
    """

    def __init__(self, osr: int, clkin_hz: float, max_lags: int = 1536):
        self.tmod = 2e6 / clkin_hz                 # us per modulator sample
        h = sinc3_taps(osr)
        self.stride = max(1, int(np.ceil(len(h) / max_lags)))
        if self.stride > 1:                        # coarser grid for long filters
            pad = (-len(h)) % self.stride
            h = np.pad(h, (0, pad)).reshape(-1, self.stride).sum(1)
        self.h = h
        self.lag = (np.arange(len(h)) * self.stride + 0.5 * (self.stride - 1)) * self.tmod
        self.cdf = np.cumsum(h)
        self.length_us = len(h) * self.stride * self.tmod

    def step(self, t: np.ndarray) -> np.ndarray:
        """Fraction of a unit step at t=0 seen by the conversion ending at t."""
        t = np.asarray(t, float)
        idx = np.floor(t / (self.tmod * self.stride)).astype(int)
        out = np.where(idx < 0, 0.0, self.cdf[np.clip(idx, 0, len(self.cdf) - 1)])
        return np.where(t >= self.length_us, 1.0, out)

    def exp_tail(self, t: np.ndarray, tau: float) -> np.ndarray:
        """sinc3-filtered exp(-u/tau) for u > 0 (unit amplitude).

        sum over lags with u_n = t - lag[n] > 0 of h[n] exp(-u_n / tau). With
        lag[n] = n*ts + off and i = the last included lag this is
        exp(-(t - lag[i]) / tau) * G[i], G[i] = sum_{n<=i} h[n] r^(i-n),
        r = exp(-ts/tau): one recursive filter pass, O(N + L), no overflow.
        """
        from scipy.signal import lfilter

        t = np.asarray(t, float)
        ts = self.tmod * self.stride
        off = self.lag[0]
        g = lfilter([1.0], [1.0, -np.exp(-ts / tau)], self.h)
        i = np.ceil((t - off) / ts).astype(np.int64) - 1          # lag[i] < t
        i = np.minimum(i, len(self.h) - 1)
        ok = i >= 0
        out = np.zeros_like(t)
        ii = i[ok]
        out[ok] = np.exp(-(t[ok] - self.lag[ii]) / tau) * g[ii]
        return out


WINDOWS_US = ((150, 250), (250, 350), (350, 500), (500, 700), (700, 1000), (1000, 1400), (1400, 2000))
TAIL_FRACS = (3e-4, 1e-4)
SIGMA_T_US = 2.0          # DRDY timestamp jitter assumed by the fit weights
TAU_A_MIN = 0.2


@dataclass
class ChanFit:
    ch: int
    out_from: int
    out_to: int
    pre: float
    final: float
    noise: float            # sd of single conversions over the final part
    fitted: bool            # False: no step big enough to fit
    delay_us: float = float("nan")
    tau_a_us: float = float("nan")      # fast analog first-order response (100 % of the step)
    a2_mv: float = float("nan")         # slow tail amplitude at the switch
    tau2_us: float = float("nan")
    err: dict = field(default_factory=dict)      # 1-sigma of the above
    rms: float = float("nan")                    # weighted-fit residual rms, mV
    t_slow_us: dict = field(default_factory=dict)  # frac -> analog time (from the switch)
    t_fast_us: dict = field(default_factory=dict)  #   when that part is below frac x step
    windows: list = field(default_factory=list)    # (lo, hi, mean, sem, n, slow-fit mean)
    bins: dict = field(default_factory=dict)

    @property
    def step(self) -> float:
        return self.final - self.pre


def _fast(filt, pre, step, t, d, ta):
    """Analog first-order step (time constant ta, delay d) through the sinc3."""
    return pre + step * (filt.step(t - d) - filt.exp_tail(t - d, ta))


def analyze(s: Settle, final_from_us: float | None = None, bin_us: float | None = None,
            sigma_t_us: float = SIGMA_T_US) -> tuple[list[ChanFit], list[str]]:
    """Fit pre + step * (1 - exp(-u/tau_a)) + A2 * exp(-u/tau2), u = t - d > 0,
    through the sinc3 filter, per channel with a step.

    Weights: 1 / sqrt(noise^2 + (slope * sigma_t)^2). Around the step the
    filtered signal moves by several mV/us, so the DRDY timestamp jitter, not
    the ADC noise, sets the error there. Soft-L1 loss against outliers.
    """
    from scipy.optimize import least_squares

    m = s.meta
    notes = list(s.notes)
    osr, clk, period = int(m["osr"]), float(m["clkin_hz"]), float(m["period_us"])
    window = float(m["window_us"])
    filt = Sinc3(osr, clk)
    mv = s.codes.astype(float) * float(m.get("mv_per_code", MV_PER_CODE))
    good = (s.flags & BAD) == 0
    nbad = int((~good).sum())
    if nbad:
        notes.append(f"{nbad} of {len(good)} conversions dropped (CRC/stale/reset/overrun/edgeless)")
    e = s.end
    for k in ("crc_bad", "stale", "overrun", "missed", "edgeless", "reset"):
        if e.get(k):
            notes.append(f"firmware counted {k} = {e[k]}")
    if e.get("truncated"):
        notes.append("WARNING capture truncated (DRDY timeout)")
    notes.append(f"read_us_max {e.get('read_us_max')} lat_us_max {e.get('lat_us_max')}"
                 f" (conversion period {period:.1f} us)")
    if "jitter_us" not in m:
        notes.append("rep spacing not randomized (firmware before the jitter_us option):"
                     " a board line at a multiple of 1/dwell adds up coherently across reps")
    t = s.t_us.astype(float)
    if final_from_us is None:
        final_from_us = 0.6 * window
    pre_m = good & (t < 0)
    fin_m = good & (t >= final_from_us) & (t <= window + period)
    post_m = good & (t >= 0) & (t <= window + period)
    if pre_m.sum() < 2 or fin_m.sum() < 2:
        raise ValueError("not enough clean conversions before the switch / at the end of the window")
    if bin_us is None:
        bin_us = period / max(1, int(m.get("dither", 1)))
    pre_all, fin_all = mv[pre_m].mean(0), mv[fin_m].mean(0)
    noise_all = mv[fin_m].std(0)
    tp = t[post_m]
    hi_d = max(2 * period, 100.0)
    of, ot = m.get("out_from", [0] * 4), m.get("out_to", [0] * 4)

    fits = []
    for c in range(4):
        pre, fin, noise = float(pre_all[c]), float(fin_all[c]), float(noise_all[c])
        step = fin - pre
        cf = ChanFit(c, of[c], ot[c], pre, fin, noise, False)
        fits.append(cf)
        if abs(step) < max(5.0, 20 * noise):
            continue
        y = mv[post_m, c]
        sn = max(noise, 0.005)
        lim = 0.05 * abs(step)

        def model(p, tt=tp):
            d, ta, a2, t2 = p
            return _fast(filt, pre, step, tt, d, ta) + a2 * filt.exp_tail(tt - d, t2)

        best = None
        for d0 in (5.0, 20.0, 40.0):
            for ta0 in (3.0, 10.0, 30.0):
                slope = (_fast(filt, pre, step, tp + 0.5, d0, ta0)
                         - _fast(filt, pre, step, tp - 0.5, d0, ta0))
                w = 1.0 / np.sqrt(sn ** 2 + (slope * sigma_t_us) ** 2)
                for t20 in (40.0, 100.0, 300.0):
                    r = least_squares(lambda p: (model(p) - y) * w,
                                      [min(d0, hi_d), ta0, -0.003 * step, t20],
                                      bounds=([-period, TAU_A_MIN, -lim, 15.0], [hi_d, 60.0, lim, 3000.0]),
                                      x_scale=[5.0, 5.0, max(0.003 * abs(step), 0.01), 50.0],
                                      loss="soft_l1", f_scale=3.0)
                    if best is None or r.cost < best.cost:
                        best, best_w = r, w
        d, ta, a2, t2 = (float(x) for x in best.x)
        try:
            jac = best.jac
            cov = np.linalg.pinv(jac.T @ jac) * (2 * best.cost / max(1, len(y) - 4))
            errs = np.sqrt(np.clip(np.diag(cov), 0, None))
        except np.linalg.LinAlgError:
            errs = np.full(4, np.nan)
        cf.fitted = True
        cf.delay_us, cf.tau_a_us, cf.a2_mv, cf.tau2_us = d, ta, a2, t2
        cf.err = dict(zip(("delay_us", "tau_a_us", "a2_mv", "tau2_us"), map(float, errs)))
        cf.rms = float(np.sqrt(np.mean(((model(best.x) - y)) ** 2)))
        for frac in TAIL_FRACS:
            lvl = frac * abs(step)
            cf.t_slow_us[frac] = d + (t2 * np.log(abs(a2) / lvl) if abs(a2) > lvl else 0.0)
            cf.t_fast_us[frac] = d + ta * np.log(1.0 / frac)
        fast_all = _fast(filt, pre, step, t, d, ta)
        diff = mv[:, c] - fast_all
        slow = a2 * filt.exp_tail(t - d, t2)
        for lo, hi in WINDOWS_US:
            k = good & (t >= lo) & (t < hi)
            n = int(k.sum())
            if n:
                cf.windows.append((lo, hi, float(diff[k].mean()),
                                   float(diff[k].std() / np.sqrt(n)) if n > 1 else float("nan"),
                                   n, float(slow[k].mean())))
        sel = good & (t > -3 * period) & (t <= window + period)
        edges = np.arange(np.floor(t[sel].min() / bin_us) * bin_us, t[sel].max() + bin_us, bin_us)
        idx = np.digitize(t[sel], edges) - 1
        b = {k: [] for k in ("t_us", "n", "mv", "model_mv", "diff_mv", "diff_sem", "slow_mv")}
        full = fast_all + slow
        for i in np.unique(idx):
            k = idx == i
            b["t_us"].append(float(t[sel][k].mean()))
            b["n"].append(int(k.sum()))
            b["mv"].append(float(mv[sel, c][k].mean()))
            b["model_mv"].append(float(full[sel][k].mean()))
            b["diff_mv"].append(float(diff[sel][k].mean()))
            b["diff_sem"].append(float(diff[sel][k].std() / np.sqrt(k.sum())) if k.sum() > 1 else np.nan)
            b["slow_mv"].append(float(slow[sel][k].mean()))
        cf.bins = b
    if not any(f.fitted for f in fits):
        notes.append("no channel has a step to fit (need |final - pre| > max(5 mV, 20 x noise))")
    return fits, notes


def settle_text(s: Settle, fits: list[ChanFit], notes: list[str]) -> str:
    m = s.meta
    filt_us = 3 * float(m["period_us"])
    lines = [f"mux settle: select {m['from']} -> {m['to']}, CLKIN {m['clkin_hz']/1e6:g} MHz,"
             f" OSR {m['osr']} ({m['fdata_sps']:.0f} SPS, {float(m['period_us']):.1f} us),"
             f" SPI {m.get('spi_actual', m.get('spi_hz', 0))/1e6:g} MHz, {m['reps']} reps,"
             f" dither {m['dither']}, rep jitter {m.get('jitter_us', 0)} us;"
             f" sinc3 window {filt_us:.0f} us"]
    lines += [f"  {n}" for n in notes]
    lines.append("model (analog, u = t - d > 0): pre + step*(1 - exp(-u/tau_a)) + A2*exp(-u/tau2),"
                 " seen through the sinc3")
    lines.append("   ch  OUT from->to     pre_mV   final_mV    step_mV  noise_mV     d_us        tau_a_us"
                 "        A2_mV     A2/step    tau2_us   rms_mV")
    for f in fits:
        head = (f"  CH{f.ch}  OUT{f.out_from:>2}->OUT{f.out_to:<2} {f.pre:+10.3f} {f.final:+10.3f}"
                f" {f.step:+10.3f} {f.noise:9.4f}")
        if not f.fitted:
            lines.append(head + "   no step to fit")
            continue
        e = f.err
        ta = (f"<={TAU_A_MIN:g} (bound)" if f.tau_a_us <= TAU_A_MIN * 1.01
              else f"{f.tau_a_us:5.1f}±{e.get('tau_a_us', np.nan):<5.1f}")
        lines.append(head + f" {f.delay_us:5.1f}±{e.get('delay_us', np.nan):<4.1f} {ta:>14}"
                     f" {f.a2_mv:+8.3f}±{e.get('a2_mv', np.nan):<6.3f} {f.a2_mv / f.step:+.2e}"
                     f" {f.tau2_us:6.0f}±{e.get('tau2_us', np.nan):<4.0f} {f.rms:7.3f}")
    for f in fits:
        if not f.fitted:
            continue
        lines.append(f"  CH{f.ch} (step {f.step:+.1f} mV), analog time from the switch when each part is"
                     " below a fraction of the step:")
        for frac in TAIL_FRACS:
            lines.append(f"     {frac:.0e} x step ({frac * abs(f.step):.3f} mV): slow tail at"
                         f" {f.t_slow_us[frac]:5.0f} us, fast part at {f.t_fast_us[frac]:5.0f} us"
                         f"  -> a conversion must END >= {max(f.t_slow_us[frac], f.t_fast_us[frac]) + filt_us:.0f} us")
        if f.tau_a_us <= TAU_A_MIN * 1.01:
            lines.append("     (tau_a at its lower bound: the fast part is not resolved from the delay"
                         " d at this rate; d is the effective delay)")
        lines.append("      window_us     data - fast-only model, mV        slow-tail fit, mV      n")
        for lo, hi, mean, sem, n, slow in f.windows:
            lines.append(f"     {lo:5d}-{hi:<5d}   {mean:+9.4f} ± {sem:.4f}            {slow:+9.4f}        {n:5d}")
    return "\n".join(lines)


def settle_report(s: Settle, out: Path, final_from_us=None, bin_us=None,
                  sigma_t_us: float = SIGMA_T_US) -> list[ChanFit]:
    fits, notes = analyze(s, final_from_us, bin_us, sigma_t_us)
    text = settle_text(s, fits, notes)
    print(text)
    png = _settle_plot(s, fits, out.with_suffix(".png"))
    out.with_suffix(".html").write_text(_settle_html(s, fits, text, png), encoding="utf-8")
    print(f"wrote {out.with_suffix('.html')}" + (f" and {out.with_suffix('.png')}" if png else
          " (no plot: matplotlib not installed)"))
    return fits


def _settle_plot(s: Settle, fits: list[ChanFit], path: Path) -> bytes | None:
    fitted = [f for f in fits if f.fitted]
    if not fitted:
        return None
    try:
        import matplotlib
    except ImportError:
        return None
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    m = s.meta
    fig, axs = plt.subplots(len(fitted), 2, figsize=(12, 3.4 * len(fitted)), squeeze=False)
    for row, f in zip(axs, fitted):
        b = {k: np.asarray(v) for k, v in f.bins.items()}
        ax = row[0]
        ax.plot(b["t_us"], b["mv"], ".", ms=3, label="measured (bin mean)")
        ax.plot(b["t_us"], b["model_mv"], "-", lw=1, label="fit")
        ax.axvline(0, color="0.6", lw=0.6)
        ax.set_title(f"CH{f.ch}: OUT{f.out_from} -> OUT{f.out_to}, step {f.step:+.1f} mV", fontsize=9)
        ax.set_ylabel("mV")
        ax.legend(fontsize=7)
        ax = row[1]
        ax.errorbar(b["t_us"], b["diff_mv"], yerr=b["diff_sem"], fmt=".", ms=3, lw=0.6,
                    label="data - fast-only fit")
        ax.plot(b["t_us"], b["slow_mv"], "r-", lw=1,
                label=f"slow tail A2={f.a2_mv:+.3f} mV, tau2={f.tau2_us:.0f} us")
        for frac, ls in zip(TAIL_FRACS, ("--", ":")):
            for sg in (1, -1):
                ax.axhline(sg * frac * abs(f.step), color="0.6", lw=0.7, ls=ls)
        lim = max(4 * 3e-4 * abs(f.step), 0.05)
        ax.set_ylim(-lim, lim)
        ax.set_xlim(left=0)
        ax.set_ylabel("mV (lines: 3e-4, 1e-4 x step)")
        ax.legend(fontsize=7)
    for ax in axs[-1]:
        ax.set_xlabel("us since mux switch (conversion end)")
    fig.suptitle(f"select {m['from']} -> {m['to']}, CLKIN {m['clkin_hz']/1e6:g} MHz, OSR {m['osr']}",
                 fontsize=10)
    fig.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=100)
    plt.close(fig)
    path.write_bytes(buf.getvalue())
    return buf.getvalue()


def _settle_html(s: Settle, fits: list[ChanFit], text: str, png: bytes | None) -> str:
    img = (f'<img src="data:image/png;base64,{base64.b64encode(png).decode()}" alt="settle plot">'
           if png else "<p><i>No plot (matplotlib not installed, or no channel with a step).</i></p>")
    tables = []
    for f in fits:
        if not f.fitted:
            continue
        keys = ("t_us", "n", "mv", "model_mv", "diff_mv", "diff_sem", "slow_mv")
        rows = "".join(f"<tr><td>{t:.1f}</td><td>{n}</td><td>{v:+.4f}</td><td>{mo:+.4f}</td>"
                       f"<td>{d:+.4f}</td><td>{e:.4f}</td><td>{sl:+.4f}</td></tr>"
                       for t, n, v, mo, d, e, sl in zip(*(f.bins[k] for k in keys)))
        tables.append(f"<details><summary>CH{f.ch} binned data</summary><table><tr><th>t, us</th>"
                      f"<th>n</th><th>mV</th><th>fit mV</th><th>data - fast mV</th><th>sem</th>"
                      f"<th>slow fit mV</th></tr>{rows}</table></details>")
    return f"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Mux settle</title>
<style>
:root {{ --bg:#fff; --fg:#1b1b1b; --line:#ddd; }}
@media (prefers-color-scheme: dark) {{ :root {{ --bg:#16181c; --fg:#e6e6e6; --line:#3a3d44; }} }}
body {{ background:var(--bg); color:var(--fg); font:14px/1.45 system-ui,sans-serif;
       margin:0 auto; max-width:1150px; padding:16px; }}
img {{ max-width:100%; background:#fff; }}
pre {{ overflow-x:auto; font-size:12px; }}
table {{ border-collapse:collapse; font-variant-numeric:tabular-nums; }}
td,th {{ border-bottom:1px solid var(--line); padding:1px 8px; text-align:right; }}
</style></head><body>
<h1 style="font-size:18px">Mux settle diagnostic</h1>
<pre>{html.escape(text)}</pre>
{img}
{''.join(tables)}
<details><summary>capture header</summary><pre>{html.escape(json.dumps(s.meta, indent=1))}</pre></details>
</body></html>
"""


def cmd_settle(a) -> int:
    cmd = (f"D{a.from_sel} {a.to_sel} {a.clkin} {a.osr} {a.spi} {a.reps} {a.window_us}"
           f" {a.dither} {a.dwell_us} {a.jitter_us};")
    lines_est = a.reps * (4 + a.window_us / (2e6 * a.osr / a.clkin) + 2)
    budget = a.reps * (a.dwell_us + a.jitter_us + a.window_us) / 1e6 + lines_est * 50 * 10 / 921600 + 20
    print(f"settle diag: {cmd}  (up to {budget:.0f} s)", flush=True)
    raw = run_command(a.port, cmd, (b"#END", b"#SETTLEERR"), budget)
    a.out.with_suffix(".txt").write_bytes(raw)
    s = parse_settle(raw)
    save_settle_npz(a.out.with_suffix(".npz"), s)
    print(f"saved {a.out.with_suffix('.txt')} and {a.out.with_suffix('.npz')}")
    settle_report(s, a.out, a.final_from_us, a.bin_us, a.sigma_t_us)
    return 0


def cmd_analyze(a) -> int:
    s = load_settle(a.file)
    settle_report(s, a.out or a.file.with_suffix(""), a.final_from_us, a.bin_us, a.sigma_t_us)
    return 0


def cmd_selects(a) -> int:
    sel = "" if a.selects.lower() == "all" else "".join(sorted(set(a.selects)))
    if any(c not in "01234567" for c in sel):
        print("selects: digits 0-7, or 'all'", file=sys.stderr)
        return 2
    ser = open_port(a.port)
    try:
        ser.reset_input_buffer()
        ser.write(f"S{sel};".encode())
        raw, t0 = b"", time.monotonic()
        while time.monotonic() - t0 < 1.5:
            raw += ser.read(ser.in_waiting or 1)
    finally:
        ser.close()
    i = raw.find(b"selects:")
    if i < 0:
        print(f"no confirmation from the ESP32; got {raw[-200:]!r}", file=sys.stderr)
        return 1
    print(raw[i:raw.find(b"\n", i) if raw.find(b"\n", i) > 0 else None].decode(errors="replace").strip())
    print("note: the ESP32 forgets this on reboot (daq_server.py --selects re-sends it)")
    return 0


# ------------------------------------------------------------- scan modes

MODES = {"legacy": 0, "quiet": 1, "wide": 2, "0": 0, "1": 1, "2": 2}


def core_command(core: int | None, tx_window_us: int | None) -> str:
    """The C command: C<0|1> [tx_window_us]; ("" = leave the core as it is)."""
    if core is None:
        return ""
    if core not in (0, 1):
        raise ValueError("core: 0 or 1")
    return f"C1 {tx_window_us};" if core == 1 and tx_window_us else f"C{core};"


def mode_command(mode: str, settle: int | None, osr: int | None, spi: int | None,
                 period: int | None = None) -> str:
    """The M command: M<mode> [settle_us] [osr] [period_us] [spi_hz]; (0 = default)."""
    m = MODES[mode.lower()]
    if m == 0:
        return "M0;"
    args = [str(m), str(settle if settle is not None else 200), str(osr or 0), str(period or 0),
            str(spi or 0)]
    while args[-1] == "0":               # 0 = the mode's default
        args.pop()
    return "M" + " ".join(args) + ";"


def _send_and_echo(port: str, cmd: str, keys: tuple[bytes, ...], secs: float = 2.0) -> bytes:
    ser = open_port(port)
    try:
        ser.reset_input_buffer()
        ser.write(cmd.encode())
        raw, t0 = b"", time.monotonic()
        while time.monotonic() - t0 < secs:
            raw += ser.read(ser.in_waiting or 1)
    finally:
        ser.close()
    for ln in raw.replace(b"\r", b"").split(b"\n"):
        if any(k in ln for k in keys):
            print(ln[max(0, min(ln.find(k) for k in keys if k in ln)):].decode(errors="replace"))
    return raw


def cmd_mode(a) -> int:
    if a.mode.lower() not in MODES:
        print("mode: legacy|quiet|wide (or 0|1|2)", file=sys.stderr)
        return 2
    cmd = mode_command(a.mode, a.settle_us, a.osr, a.spi, a.period_us)
    raw = _send_and_echo(a.port, cmd, (b"mode ", b"M: "))
    if b"mode " not in raw and b"M: " not in raw:
        print(f"no confirmation from the ESP32 (sent {cmd}); in binary mode the reply is a framed"
              " TEXT record: use 'stream' to see it", file=sys.stderr)
        return 1
    print("note: the ESP32 forgets this on reboot (daq_server.py --mode re-sends it)")
    return 0


# Lines that mean the run is not what it looks like (bench r4: the ESP32 had
# rebooted and streamed the text-mode grid, which looked like a working run).
STREAM_FAIL_MARKS = ("[reader watchdog]", "whisker_daq \u2014", "Commands: B=", "previous reset:",
                     "Guru Meditation", "abort()", "Backtrace:", "rst:0x", "[park timeout]",
                     "reader task NOT created")
NO_SCAN_TIMEOUT_S = 2.0


class StreamWatch:
    """Fails a stream run loudly: no SCAN within NO_SCAN_TIMEOUT_S of B (of each
    re-send), or a reboot / watchdog / crash line anywhere."""

    def __init__(self) -> None:
        self.problems: list[str] = []
        self.t_kick = None
        self.first_scan_after_kick = True

    def kicked(self, t: float) -> None:
        self.t_kick = t
        self.first_scan_after_kick = False

    def scan(self) -> None:
        self.first_scan_after_kick = True

    def text(self, line: str) -> None:
        for m in STREAM_FAIL_MARKS:
            if m in line:
                self.problems.append(f"the ESP32 printed {line.strip()[:120]!r}")
                break

    def no_scan(self, t: float) -> bool:
        """True (and records it) if the last B got no SCAN in time."""
        if self.first_scan_after_kick or self.t_kick is None or t - self.t_kick < NO_SCAN_TIMEOUT_S:
            return False
        self.problems.append(f"no SCAN frame within {NO_SCAN_TIMEOUT_S:.0f} s of B")
        self.first_scan_after_kick = True            # report once per kick
        return True


def cmd_stream(a) -> int:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from whisker_link import esp32_frame as ef

    cmd = ""
    if a.selects:
        cmd += "S" + ("" if a.selects.lower() == "all" else a.selects) + ";"
    cmd += core_command(a.core, a.tx_window_us)
    if a.mode:
        if a.mode.lower() not in MODES:
            print("mode: legacy|quiet|wide (or 0|1|2)", file=sys.stderr)
            return 2
        cmd += mode_command(a.mode, a.settle_us, a.osr, a.spi, a.period_us)
    cmd += "B"
    resends = 0
    ser = open_port(a.port)
    dec, raw = ef.Decoder(), bytearray()
    scans, infos, health = [], [], []
    watch = StreamWatch()
    try:
        ser.reset_input_buffer()
        ser.write(cmd.encode())
        t0 = time.monotonic()
        watch.kicked(t0)
        t_resend = t0 + (a.resend_every or 1e9)
        while time.monotonic() - t0 < a.secs:
            if time.monotonic() >= t_resend:          # stress: the same setup again
                ser.write(cmd.encode())
                resends += 1
                t_resend += a.resend_every
                watch.kicked(time.monotonic())
            if watch.no_scan(time.monotonic()) and not a.resend_every:
                print(f"ERROR: {watch.problems[-1]}; stopping", file=sys.stderr, flush=True)
                break
            data = ser.read(ser.in_waiting or 1)
            raw += data
            for ev in dec.feed(data):
                if isinstance(ev, ef.Scan):
                    scans.append(ev)
                    watch.scan()
                elif isinstance(ev, ef.Info):
                    infos.append(ev)
                    print("INFO", json.dumps(ef.info_cfg(dict(ev.__dict__))))
                elif isinstance(ev, ef.Health):
                    health.append(ev)
                    h = ev.summary()
                    print(f"HEALTH busy {h['busy_pct']:5.1f}%  lat avg {h['lat_avg_us']:5.1f} max"
                          f" {h['lat_max_us']:3d} us  read avg {h['read_avg_us']:5.1f} max"
                          f" {h['read_max_us']:3d} us  tx max {h['tx_max_us']:3d} us | conv"
                          f" {h['conversions']} overrun {h['overrun']} missed {h['missed']}"
                          f" edgeless {h['edgeless']} crc {h['crc']} stale {h['stale']}"
                          f" late_sw {h['late_switch']} tx_drop {h['tx_drop']} empty"
                          f" {h['empty_slots']} resync {h['resync']} reinit {h['reinit']}"
                          f" recov {h['recov']}" + (
                              f" | core {h['core']} q_hwm {h['q_hwm']} tx_win {h['tx_win_us']}"
                              f" host_tx_max {h['host_tx_max_us']} det_max {h['det_max_us']}"
                              f" tx_late {h['tx_late']} parks {h['parks']}"
                              f" park_timeouts {h['park_timeouts']}" if "core" in h else ""),
                          flush=True)
                elif isinstance(ev, ef.Text):
                    print("TEXT", ev.text)
                    n = len(watch.problems)
                    watch.text(ev.text)
                    if len(watch.problems) > n:
                        print(f"ERROR: {watch.problems[-1]}", file=sys.stderr, flush=True)
        wall = time.monotonic() - t0
        ser.write(b"g")                      # leave binary mode
    finally:
        ser.close()
    if a.out:
        a.out.write_bytes(bytes(raw))
        print(f"saved the raw stream to {a.out}")
    if not scans:
        print("ERROR: no scans received", file=sys.stderr)
        return 1
    import collections
    dts = collections.Counter(b.t_us - c.t_us for c, b in zip(scans, scans[1:]))
    gaps = sum(1 for c, b in zip(scans, scans[1:]) if b.seq != c.seq + 1)
    flags = collections.Counter()
    for sc in scans:
        for bit, name in ((ef.FLAG_RECOVERED, "recovered"), (ef.FLAG_CRC_RETRY, "crc"),
                          (ef.FLAG_STALE, "stale"), (ef.FLAG_REINIT, "reinit"),
                          (ef.FLAG_SYNC, "sync"), (ef.FLAG_EMPTY, "empty")):
            if sc.flags & bit:
                flags[name] += 1
    print(f"{len(scans)} scans in {wall:.1f} s ({len(scans) / wall:.1f}/s on the wire); t_us"
          f" spacing (us: count) {dict(dts.most_common(5))}; seq gaps {gaps};"
          f" flagged scans {dict(flags) or 0}; link CRC failures {dec.n_crc_fail}")
    if health:
        hs = [h.summary() for h in health]
        tot = {k: sum(h[k] for h in hs) for k in ("overrun", "missed", "late_switch", "empty_slots",
                                                 "tx_drop", "resync", "reinit", "recov")}
        print(f"HEALTH totals over {len(hs)} frames: {tot}; lat max {max(h['lat_max_us'] for h in hs)}"
              f" us; busy {min(h['busy_pct'] for h in hs)}-{max(h['busy_pct'] for h in hs)} %"
              + (f"; core {sorted({h['core'] for h in hs if 'core' in h})}, parks"
                 f" {hs[-1]['parks']}, park timeouts {hs[-1]['park_timeouts']}" if "core" in hs[-1]
                 else ""))
    if resends:
        print(f"stress: the setup was re-sent {resends} times ({len([x for x in scans if x.flags])}"
              f" flagged scans in total)")
    if dec.n_junk:
        print(f"WARNING: {dec.n_junk} bytes arrived outside frames and {gaps} seq gap(s): bytes"
              " were lost or corrupted on the link (bench r5: 64 x 0xFF replaced a frame's"
              " sync bytes; the firmware had sent it)", file=sys.stderr)
    if watch.problems:
        print(f"ERROR: this run is NOT valid ({len(watch.problems)} problem(s)):", file=sys.stderr)
        for p in dict.fromkeys(watch.problems):
            print(f"  - {p}", file=sys.stderr)
        return 3
    return 0


# ------------------------------------------------------------------------ CLI

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")

    p = sub.add_parser("spi", help="SPI speed sweep")
    p.add_argument("--port", required=True)
    p.add_argument("--clkin", type=int, default=8_000_000)
    p.add_argument("--osr", type=int, default=256)
    p.add_argument("--secs", type=int, default=2, help="seconds per SPI clock")
    p.add_argument("--spi", type=int, nargs="+", help=f"SPI clocks, Hz (default {DEFAULT_SPIS})")
    p.add_argument("--out", type=Path, default=Path(f"spi_sweep_{stamp}"))
    p.set_defaults(func=cmd_spi)

    an = argparse.ArgumentParser(add_help=False)
    an.add_argument("--final-from-us", type=float, help="final value = mean after this (default 0.6 x window)")
    an.add_argument("--bin-us", type=float, help="bin width (default period / dither)")
    an.add_argument("--sigma-t-us", type=float, default=SIGMA_T_US,
                    help="DRDY timestamp jitter for the fit weights (default %(default)s us)")

    p = sub.add_parser("settle", parents=[an], help="fine mux settle diagnostic")
    p.add_argument("--port", required=True)
    p.add_argument("--from", dest="from_sel", type=int, required=True, help="select before the switch")
    p.add_argument("--to", dest="to_sel", type=int, required=True, help="select after the switch")
    p.add_argument("--clkin", type=int, default=8_000_000)
    p.add_argument("--osr", type=int, default=256)
    p.add_argument("--spi", type=int, default=4_000_000, help="use the best safe value from 'spi'")
    p.add_argument("--reps", type=int, default=64)
    p.add_argument("--window-us", type=int, default=2000)
    p.add_argument("--dither", type=int, default=4, help="switch phases per conversion period")
    p.add_argument("--dwell-us", type=int, default=3000)
    p.add_argument("--jitter-us", type=int, default=1000,
                   help="random 0..N us added to each rep's dwell (breaks coherence with board lines)")
    p.add_argument("--out", type=Path, default=Path(f"settle_{stamp}"))
    p.set_defaults(func=cmd_settle)

    p = sub.add_parser("analyze", parents=[an], help="analyze a saved settle capture")
    p.add_argument("file", type=Path)
    p.add_argument("--out", type=Path)
    p.set_defaults(func=cmd_analyze)

    p = sub.add_parser("selects", help="set the scanned mux selects")
    p.add_argument("--port", required=True)
    p.add_argument("selects", help="digits 0-7, e.g. 0123457, or 'all'")
    p.set_defaults(func=cmd_selects)

    mo = argparse.ArgumentParser(add_help=False)
    mo.add_argument("--settle-us", type=int, help="locked modes: settle after a switch (default 200)")
    mo.add_argument("--osr", type=int, help="locked modes: OSR (quiet 512 default or 256, wide 128)")
    mo.add_argument("--period-us", type=int,
                    help="quiet mode: scan period, a multiple of 8000 us (default 16000; 8000 ="
                         " 125 Hz, needs --osr 256 or lower)")
    mo.add_argument("--spi", type=int, help="locked modes: SPI clock, Hz (quiet 8 MHz, wide 12 MHz)")

    p = sub.add_parser("mode", parents=[mo], help="set the scan mode")
    p.add_argument("--port", required=True)
    p.add_argument("mode", help="legacy | quiet | wide (or 0 | 1 | 2)")
    p.set_defaults(func=cmd_mode)

    p = sub.add_parser("stream", parents=[mo], help="stream and print HEALTH + scan stats")
    p.add_argument("--core", type=int, choices=(0, 1),
                   help="who sends while streaming: 1 core 0 (boot default), 0 the reader itself")
    p.add_argument("--tx-window-us", type=int,
                   help="with --core 1: send each scan frame this long after the scan start"
                        " (100 us .. the scan period; default: as soon as it is ready)")
    p.add_argument("--resend-every", type=float, default=0,
                   help="stress: re-send the S/C/M/B setup every this many seconds")
    p.add_argument("--port", required=True)
    p.add_argument("--mode", help="set this mode first: legacy | quiet | wide")
    p.add_argument("--selects", help="set these selects first, e.g. 0123457 or all")
    p.add_argument("--secs", type=float, default=10.0)
    p.add_argument("--out", type=Path, help="save the raw byte stream here")
    p.set_defaults(func=cmd_stream)

    a = ap.parse_args(argv)
    return a.func(a)


if __name__ == "__main__":
    sys.exit(main())
