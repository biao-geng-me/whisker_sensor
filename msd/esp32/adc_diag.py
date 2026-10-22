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

Settle analysis, per ADC channel: ``pre`` = mean of the conversions that ended
before the switch, ``final`` = mean over the end of the window. The ideal
response is what an instantaneous analog step does through the ADC's sinc3
filter (computed for the same OSR and CLKIN): pre + (final - pre) * S(t - d),
where S is the cumulative sinc3 impulse response and d a fitted latency (DRDY
edge + ISR timing; fitted on the channel with the largest step and shared).
What is left over is the analog tail. It is fitted as a decaying exponential
A*exp(-t/tau) passed through the same sinc3 filter (the expectation is charge
sharing: A ~ 3e-4 x (pre - final), tau ~ 100 us). The report gives A, tau, the
time at which the tail is below 0.01 mV, and the binned data, as text,
``<out>.html`` and (with matplotlib) ``<out>.png``.

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
SETTLED_MV = 0.01


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
        """sinc3-filtered exp(-u/tau) for u > 0 (unit amplitude)."""
        u = np.asarray(t, float)[:, None] - self.lag[None, :]
        e = np.where(u > 0, np.exp(-np.clip(u, 0, None) / tau), 0.0)
        return e @ self.h


@dataclass
class ChanFit:
    ch: int
    out_from: int
    out_to: int
    pre: float
    final: float
    noise: float            # sd of single conversions over the final part
    delay_us: float
    amp_mv: float
    tau_us: float
    amp_err: float
    tau_err: float
    significant: bool       # the tail stands out of the fit noise (|A| > 3 sigma, > SETTLED_MV
                            # once the filter has seen the whole step)
    t_settle_us: float | None   # tail (fit) below SETTLED_MV, from the switch
    t_settle_data_us: float | None
    bins: dict


def analyze(s: Settle, final_from_us: float | None = None, bin_us: float | None = None,
            fit_delay: bool = True) -> tuple[list[ChanFit], list[str]]:
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
    pre = mv[pre_m].mean(0)
    fin = mv[fin_m].mean(0)
    noise = mv[fin_m].std(0)
    step = fin - pre

    def model(p, c, delay_fixed=None):
        if delay_fixed is None:
            d, amp, tau = p
        else:
            (amp, tau), d = p, delay_fixed
        tt = t[post_m] - d
        return pre[c] + step[c] * filt.step(tt) + amp * filt.exp_tail(tt, tau)

    def fit(c, delay_fixed=None):
        y = mv[post_m, c]
        a0 = 3e-4 * (pre[c] - fin[c]) or 1e-3
        if delay_fixed is None:
            p0, lo, hi = [0.0, a0, 100.0], [-period, -50.0, 1.0], [period, 50.0, 1e4]
            r = least_squares(lambda p: model(p, c) - y, p0, bounds=(lo, hi), x_scale=[5, 0.01, 50])
        else:
            p0, lo, hi = [a0, 100.0], [-50.0, 1.0], [50.0, 1e4]
            r = least_squares(lambda p: model(p, c, delay_fixed) - y, p0, bounds=(lo, hi),
                              x_scale=[0.01, 50])
        # 1-sigma from the Jacobian
        try:
            dof = max(1, len(y) - len(r.x))
            cov = np.linalg.inv(r.jac.T @ r.jac) * (2 * r.cost / dof)
            err = np.sqrt(np.clip(np.diag(cov), 0, None))
        except np.linalg.LinAlgError:
            err = np.full(len(r.x), np.nan)
        return r.x, err

    big = int(np.argmax(np.abs(step)))
    if fit_delay and abs(step[big]) > 20 * max(noise[big], 1e-6):
        (delay, _, _), _ = fit(big)
        notes.append(f"latency d = {delay:.2f} us, fitted on CH{big} (step {step[big]:+.1f} mV)")
    else:
        delay = 0.0
        notes.append("latency d fixed at 0 (no channel with a large enough step)")

    fits = []
    of, ot = m.get("out_from", [0] * 4), m.get("out_to", [0] * 4)
    for c in range(4):
        (amp, tau), (ea, et) = fit(c, delay)
        ideal = pre[c] + step[c] * filt.step(t - delay)
        diff = mv[:, c] - ideal
        sel = good & (t > -3 * period) & (t <= window + period)
        edges = np.arange(np.floor(t[sel].min() / bin_us) * bin_us, t[sel].max() + bin_us, bin_us)
        idx = np.digitize(t[sel], edges) - 1
        b = {"t_us": [], "n": [], "mv": [], "ideal_mv": [], "diff_mv": [], "diff_sem": []}
        for i in np.unique(idx):
            k = idx == i
            if k.sum() == 0:
                continue
            b["t_us"].append(float(t[sel][k].mean()))
            b["n"].append(int(k.sum()))
            b["mv"].append(float(mv[sel, c][k].mean()))
            b["ideal_mv"].append(float(ideal[sel][k].mean()))
            b["diff_mv"].append(float(diff[sel][k].mean()))
            b["diff_sem"].append(float(diff[sel][k].std() / np.sqrt(k.sum())) if k.sum() > 1 else np.nan)
        # Without a real tail A and tau trade off (tiny tau, any A); judge it by
        # what the fitted tail leaves once the filter has seen the whole step.
        visible = abs(amp * filt.exp_tail(np.array([filt.length_us]), tau)[0])
        sig = bool(abs(amp) > 3 * ea and visible > SETTLED_MV)
        ts = delay + filt.length_us
        if sig:
            ts += tau * np.log(visible / SETTLED_MV)
        # From the data: after the filter has seen the whole step, the last
        # point where the tail, smoothed over one conversion period, is above
        # SETTLED_MV by more than 3 sigma.
        td = None
        bt, bd, bs = map(np.asarray, (b["t_us"], b["diff_mv"], b["diff_sem"]))
        w = max(1, int(round(period / bin_us)))
        if len(bd) >= w:
            ker = np.ones(w) / w
            bd = np.convolve(bd, ker, mode="same")
            bs = np.sqrt(np.convolve(np.nan_to_num(bs) ** 2, ker, mode="same") / w)
        after = bt >= delay + filt.length_us
        loud = after & (np.abs(bd) - 3 * bs > SETTLED_MV)
        if after.any():
            td = float(bt[loud].max()) if loud.any() else float(bt[after].min())
        fits.append(ChanFit(c, of[c], ot[c], float(pre[c]), float(fin[c]), float(noise[c]),
                            float(delay), float(amp), float(tau), float(ea), float(et), sig,
                            float(ts), td, b))
    return fits, notes


def settle_text(s: Settle, fits: list[ChanFit], notes: list[str]) -> str:
    m = s.meta
    filt_us = 3 * float(m["period_us"])
    lines = [f"mux settle: select {m['from']} -> {m['to']}, CLKIN {m['clkin_hz']/1e6:g} MHz,"
             f" OSR {m['osr']} ({m['fdata_sps']:.0f} SPS, {float(m['period_us']):.1f} us),"
             f" SPI {m.get('spi_actual', m.get('spi_hz', 0))/1e6:g} MHz, {m['reps']} reps,"
             f" dither {m['dither']}; sinc3 sees the whole step after {filt_us:.0f} us"]
    lines += [f"  {n}" for n in notes]
    lines.append("   ch  OUT from->to     pre_mV   final_mV    step_mV  noise_mV    tail_A_mV"
                 "     tau_us  A*tau_mVus  A/(pre-final)  tail<0.01mV_at_us (fit / data)")
    for f in fits:
        rel = f.amp_mv / (f.pre - f.final) if abs(f.pre - f.final) > 1e-9 else float("nan")
        tail = (f"{f.amp_mv:+8.4f}±{f.amp_err:.4f} {f.tau_us:7.1f}±{f.tau_err:<5.1f}"
                f" {f.amp_mv * f.tau_us:+10.3f} {rel:+12.2e}" if f.significant
                else f"{'no significant tail':>48}")
        lines.append(f"  CH{f.ch}  OUT{f.out_from:>2}->OUT{f.out_to:<2} {f.pre:+10.3f} {f.final:+10.3f}"
                     f" {f.final - f.pre:+10.3f} {f.noise:9.4f} {tail}   "
                     f"{_fmt(f.t_settle_us)} / {_fmt(f.t_settle_data_us)}")
    return "\n".join(lines)


def _fmt(v) -> str:
    return "-" if v is None else f"{v:.0f}"


def settle_report(s: Settle, out: Path, final_from_us=None, bin_us=None) -> list[ChanFit]:
    fits, notes = analyze(s, final_from_us, bin_us)
    text = settle_text(s, fits, notes)
    print(text)
    png = _settle_plot(s, fits, out.with_suffix(".png"))
    out.with_suffix(".html").write_text(_settle_html(s, fits, text, png), encoding="utf-8")
    print(f"wrote {out.with_suffix('.html')}" + (f" and {out.with_suffix('.png')}" if png else
          " (no plot: matplotlib not installed)"))
    return fits


def _settle_plot(s: Settle, fits: list[ChanFit], path: Path) -> bytes | None:
    try:
        import matplotlib
    except ImportError:
        return None
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    m = s.meta
    filt = Sinc3(int(m["osr"]), float(m["clkin_hz"]))
    fig, axs = plt.subplots(4, 2, figsize=(12, 13), squeeze=False)
    for f in fits:
        b = {k: np.asarray(v) for k, v in f.bins.items()}
        ax = axs[f.ch, 0]
        ax.plot(b["t_us"], b["mv"], ".", ms=3, label="measured (bin mean)")
        ax.plot(b["t_us"], b["ideal_mv"], "-", lw=1, label="ideal sinc3 step")
        ax.axvline(0, color="0.6", lw=0.6)
        ax.set_title(f"CH{f.ch}: OUT{f.out_from} -> OUT{f.out_to}", fontsize=9)
        ax.set_ylabel("mV")
        ax.legend(fontsize=7)
        ax = axs[f.ch, 1]
        ax.errorbar(b["t_us"], b["diff_mv"], yerr=b["diff_sem"], fmt=".", ms=3, lw=0.6,
                    label="measured - ideal")
        tt = np.linspace(0, b["t_us"].max(), 400)
        ax.plot(tt, f.amp_mv * filt.exp_tail(tt - f.delay_us, f.tau_us), "r-", lw=1,
                label=f"fit A={f.amp_mv:+.4f} mV, tau={f.tau_us:.0f} us")
        for y in (SETTLED_MV, -SETTLED_MV):
            ax.axhline(y, color="0.7", lw=0.6, ls="--")
        lim = max(5 * abs(f.amp_mv), 4 * SETTLED_MV)
        ax.set_ylim(-lim, lim)
        ax.set_ylabel("analog tail, mV")
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
           if png else "<p><i>No plot: matplotlib was not installed.</i></p>")
    tables = []
    for f in fits:
        rows = "".join(f"<tr><td>{t:.1f}</td><td>{n}</td><td>{v:+.4f}</td><td>{i:+.4f}</td>"
                       f"<td>{d:+.4f}</td><td>{e:.4f}</td></tr>"
                       for t, n, v, i, d, e in zip(*(f.bins[k] for k in
                                                     ("t_us", "n", "mv", "ideal_mv", "diff_mv", "diff_sem"))))
        tables.append(f"<details><summary>CH{f.ch} binned data</summary><table><tr><th>t, us</th>"
                      f"<th>n</th><th>mV</th><th>ideal mV</th><th>diff mV</th><th>sem</th></tr>"
                      f"{rows}</table></details>")
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
           f" {a.dither} {a.dwell_us};")
    lines_est = a.reps * (4 + a.window_us / (2e6 * a.osr / a.clkin) + 2)
    budget = a.reps * (a.dwell_us + a.window_us) / 1e6 + lines_est * 50 * 10 / 921600 + 20
    print(f"settle diag: {cmd}  (up to {budget:.0f} s)", flush=True)
    raw = run_command(a.port, cmd, (b"#END", b"#SETTLEERR"), budget)
    a.out.with_suffix(".txt").write_bytes(raw)
    s = parse_settle(raw)
    save_settle_npz(a.out.with_suffix(".npz"), s)
    print(f"saved {a.out.with_suffix('.txt')} and {a.out.with_suffix('.npz')}")
    settle_report(s, a.out, a.final_from_us, a.bin_us)
    return 0


def cmd_analyze(a) -> int:
    s = load_settle(a.file)
    settle_report(s, a.out or a.file.with_suffix(""), a.final_from_us, a.bin_us)
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

    a = ap.parse_args(argv)
    return a.func(a)


if __name__ == "__main__":
    sys.exit(main())
