#!/usr/bin/env python3
"""Raw ADC burst capture + spectrum, to find the true frequency of interference.

The normal scan samples each gauge at ~76 Hz, so faster interference folds
down to a low alias (the 6-7 Hz common tone, the 16.25 / 27.5 Hz ROV tones).
The firmware's ``r`` command (``whisker_daq.ino``, see ``burst_capture``)
holds one mux select and keeps every conversion of the 4 ADC channels at
fDATA = CLKIN / 2 / OSR (3125 SPS at the default 0.8 MHz / OSR 128). A real
tone stays put across bursts at different rates, an alias moves.

Changing only the OSR does NOT resolve aliases: OSR steps are powers of 2, so
the rates nest (3125 = 4 x 781.25) and every frequency that fits the fastest
burst fits the slower ones too. Bursts at a second CLKIN give an unrelated
rate; the default set is 0.8 MHz/OSR 128 (3125 SPS), 1 MHz/OSR 128
(3906.25 SPS) and 8 MHz/OSR 2048 (1953.125 SPS). The analysis then lists the
true-frequency candidates (up to --f-max) that explain the peaks of every
burst. CLKIN must be 80 MHz / integer (LEDC); fDATA must stay below ~3.5 kSPS
at the default 600 kHz SPI or conversions are missed (overrun warning).

Capture (laptop, ESP32 on USB; saves the raw text, an .npz and the report)::

    python burst_spectrum.py capture --port /dev/ttyUSB0 --sel 2 --out bench_sel2
    python burst_spectrum.py capture --port COM5 --sel 2 --burst 128 512 1000000:128

Analyze offline (cluster; one or more saved .npz or raw .txt dumps)::

    python burst_spectrum.py analyze bench_sel2.npz --out bench_sel2

Writes ``<out>.png`` (amplitude spectra) and ``<out>.html`` (the plot plus
peak tables, standalone), and prints the peaks. Amplitudes are peak mV of a
sinusoid at the ADC input (``MV_PER_CODE`` = 1.2 V / 2^23, as in
``whisker_link/esp32_frame.py``). Each peak also lists where it would land
in the ~76 Hz scan (``--scan-hz``), so it can be matched to what wavi_py shows.

Needs numpy and scipy, pyserial for capture, and matplotlib for the plot (without it
the peaks and the HTML tables are still written).
"""

from __future__ import annotations

import argparse
import base64
import html
import io
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

MV_PER_CODE = 1.2 * 1000.0 / (1 << 23)      # 1.430511474609375e-4 mV/code
SAT_CODE = 8_380_000                         # railed: nothing plugged in
BF_CRC, BF_STALE, BF_OVERRUN, BF_EDGELESS, BF_RESET = 0x01, 0x02, 0x04, 0x08, 0x10
BAD = BF_CRC | BF_STALE | BF_RESET


# ----------------------------------------------------------------- dump format

def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass
class Burst:
    meta: dict
    t_us: np.ndarray                 # (n,) u32, relative to the first conversion
    flags: np.ndarray                # (n,) u8, BF_* bits
    codes: np.ndarray                # (n, 4) i32, ADC CH0..CH3
    warnings: list = field(default_factory=list)


def _header(line: str) -> dict:
    meta = {}
    for tok in line.split()[2:]:
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        if k == "out":
            meta[k] = [int(x) for x in v.split(",")]
        elif v.startswith("0x"):
            meta[k] = int(v, 16)
        else:
            try:
                meta[k] = int(v)
            except ValueError:
                meta[k] = float(v)
    return meta


def parse_dump(raw: bytes) -> list[Burst]:
    """All bursts in a byte stream; anything outside #BURST..#END is ignored.

    Raises ValueError on a #BURSTERR or a damaged block (CRC or line count).
    """
    bursts, cur, rows, crc = [], None, [], 0xFFFF
    for bline in raw.split(b"\n"):
        if bline.endswith(b"\r"):
            bline = bline[:-1]
        if cur is None:
            i = bline.find(b"#BURSTERR")
            if i >= 0:
                raise ValueError("firmware: " + bline[i:].decode(errors="replace"))
            i = bline.find(b"#BURST ")
            if i >= 0:      # may follow binary/text junk on the same line
                cur, rows, crc = _header(bline[i:].decode()), [], 0xFFFF
            continue
        if bline.startswith(b"#END"):
            end = dict(t.split("=", 1) for t in bline.decode().split()[1:] if "=" in t)
            if int(end.get("lines", -1)) != len(rows):
                raise ValueError(f"burst: {len(rows)} lines, #END says {end.get('lines')}")
            want = int(end.get("crc", "-1"), 16)
            if want != crc:
                raise ValueError(f"burst: data CRC {crc:04X} != {want:04X} (serial damage)")
            a = np.array(rows, dtype=np.int64).reshape(-1, 6)
            bursts.append(_mk_burst(cur, a))
            cur = None
            continue
        crc = crc16_ccitt(bline + b"\n", crc)
        rows.append([int(x) for x in bline.split()])
    if cur is not None:
        raise ValueError("burst: no #END (capture cut short?)")
    return bursts


def _mk_burst(meta: dict, a: np.ndarray) -> Burst:
    return Burst(meta, a[:, 0].astype(np.uint32), a[:, 1].astype(np.uint8),
                 a[:, 2:6].astype(np.int32))


def save_npz(path: Path, bursts: list[Burst]) -> None:
    arrs = {}
    for i, b in enumerate(bursts):
        arrs[f"b{i}_t_us"], arrs[f"b{i}_flags"], arrs[f"b{i}_codes"] = b.t_us, b.flags, b.codes
        arrs[f"b{i}_meta"] = np.array(json.dumps(b.meta))
    np.savez_compressed(path, n_bursts=len(bursts), **arrs)


def load(path: Path) -> list[Burst]:
    if path.suffix == ".npz":
        z = np.load(path)
        return [Burst(json.loads(str(z[f"b{i}_meta"])), z[f"b{i}_t_us"], z[f"b{i}_flags"],
                      z[f"b{i}_codes"]) for i in range(int(z["n_bursts"]))]
    return parse_dump(path.read_bytes())


# -------------------------------------------------------------------- capture

DEFAULT_BURSTS = ["800000:128", "1000000:128", "8000000:2048"]


def parse_plan(items: list[str]) -> list[tuple[int, int]]:
    """'OSR' or 'CLKIN:OSR' -> [(clkin_hz, osr)]; a bare OSR uses 0.8 MHz."""
    plan = []
    for it in items:
        clk, _, osr = it.rpartition(":")
        plan.append((int(float(clk)) if clk else 800_000, int(osr)))
    return plan


def capture(port: str, sel: int, n: int, plan: list[tuple[int, int]], spi_hz: int,
            baud: int = 921600) -> tuple[list[Burst], bytes]:
    import serial

    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, baud, 0.2
    ser.dtr = False          # don't pulse EN (the ESP32 would reboot)
    ser.rts = False
    ser.open()
    bursts, raw_all = [], b""
    try:
        for clkin, osr in plan:
            ser.reset_input_buffer()
            ser.write(b"g")                    # leave binary mode, if in it
            time.sleep(0.3)
            ser.reset_input_buffer()
            ser.write(f"r{sel} {n} {osr} {clkin} {spi_hz};".encode())
            fdata = clkin / 2 / osr
            budget = n / fdata + n * 48 * 10 / baud + 15
            print(f"burst sel {sel} CLKIN {clkin/1e6:g} MHz OSR {osr}: {fdata:.1f} SPS, {n} conversions,"
                  f" up to {budget:.0f} s ...", flush=True)
            raw, t0 = b"", time.monotonic()
            while time.monotonic() - t0 < budget:
                raw += ser.read(ser.in_waiting or 1)
                j = max(raw.rfind(b"#END"), raw.rfind(b"#BURSTERR"))
                if j >= 0 and raw.find(b"\n", j) >= 0:
                    break
            raw_all += raw
            got = parse_dump(raw)
            if not got:
                raise TimeoutError(f"no burst within {budget:.0f} s (osr {osr})")
            bursts += got
    finally:
        ser.close()
    return bursts, raw_all


# ------------------------------------------------------------------- analysis

@dataclass
class Spectrum:
    label: str
    fs: float
    f: np.ndarray
    amp: dict                   # ADC channel -> peak-mV amplitude spectrum
    dc: dict                    # ADC channel -> mean level, mV
    peaks: dict                 # ADC channel -> [(f_hz, amp_mv)]
    notes: list


def _cleaned(b: Burst) -> tuple[np.ndarray, list]:
    """mV per channel with CRC-bad / stale samples interpolated over."""
    notes = []
    mv = b.codes.astype(np.float64) * float(b.meta.get("mv_per_code", MV_PER_CODE))
    bad = (b.flags & BAD) != 0
    if bad.any():
        good = np.flatnonzero(~bad)
        if len(good) < 2:
            raise ValueError("no clean samples")
        for c in range(4):
            mv[bad, c] = np.interp(np.flatnonzero(bad), good, mv[good, c])
        notes.append(f"{bad.sum()} CRC-bad/stale/reset samples interpolated")
    over = int(((b.flags & BF_OVERRUN) != 0).sum())
    if over:
        notes.append(f"WARNING {over} overruns: conversions were missed, the time grid"
                     " has holes; use a higher OSR or spi_hz")
    edg = int(((b.flags & BF_EDGELESS) != 0).sum())
    if edg:
        notes.append(f"{edg} edgeless claims (read could not keep up with DRDY)")
    return mv, notes


def spectrum(b: Burst, nperseg: int | None, k_noise: float, n_peaks: int) -> Spectrum:
    from scipy.signal import find_peaks, welch

    mv, notes = _cleaned(b)
    fs = float(b.meta.get("fdata_sps") or b.meta["clkin_hz"] / 2 / b.meta["osr"])
    ok = (b.flags & (BF_OVERRUN | BF_EDGELESS)) == 0
    dt = np.diff(b.t_us.astype(np.int64))[ok[1:] & ok[:-1]]
    if len(dt) > 10:
        fs_meas = 1e6 / np.median(dt)
        if abs(fs_meas / fs - 1) > 0.02:
            notes.append(f"WARNING measured rate {fs_meas:.1f} SPS vs nominal {fs:.1f}")
        notes.append(f"measured {fs_meas:.2f} SPS from DRDY edge times (nominal {fs:.2f})")
    n = len(mv)
    if nperseg is None:
        nperseg = max(256, 1 << int(np.floor(np.log2(max(n // 4, 1)))))
    nperseg = min(nperseg, n)
    amp, peaks, dc = {}, {}, {}
    for c in range(4):
        if np.median(np.abs(b.codes[:, c])) >= SAT_CODE:
            notes.append(f"CH{c} railed (nothing plugged in), skipped")
            continue
        f, p = welch(mv[:, c], fs=fs, window="hann", nperseg=nperseg, detrend="linear",
                     scaling="spectrum")
        a = np.sqrt(2.0 * p)                     # peak amplitude of a sinusoid, mV
        amp[c] = a
        dc[c] = float(mv[:, c].mean())
        df = f[1] - f[0]
        floor = np.median(a[f > 2 * df])
        idx, _ = find_peaks(a, height=k_noise * floor, prominence=0.5 * k_noise * floor)
        idx = [i for i in idx if f[i] > 2 * df]
        idx = sorted(idx, key=lambda i: -a[i])[:n_peaks]
        peaks[c] = [(_refine(f, a, i), float(a[i])) for i in sorted(idx)]
    label = (f"sel {b.meta.get('sel')} OSR {b.meta.get('osr')} CLKIN {b.meta.get('clkin_hz', 0)/1e6:g} MHz"
             f" fs {fs:.1f} SPS n {n}")
    return Spectrum(label, fs, f, amp, dc, peaks, notes)


def _refine(f: np.ndarray, a: np.ndarray, i: int) -> float:
    """Parabolic interpolation of a peak on log amplitude (sub-bin frequency)."""
    if 0 < i < len(a) - 1 and min(a[i - 1], a[i], a[i + 1]) > 0:
        l, m, r = np.log(a[i - 1 : i + 2])
        d = l - 2 * m + r
        if d < 0:
            return float(f[i] + 0.5 * (l - r) / d * (f[1] - f[0]))
    return float(f[i])


def alias(f: float, fs: float) -> float:
    """Where a tone at f lands when sampled at fs (0..fs/2)."""
    r = f % fs
    return min(r, fs - r)


def common_peaks(s: Spectrum, tol_bins: float = 2.0) -> list[tuple[float, int]]:
    """Peak frequencies seen on at least 3 of the live channels."""
    df = s.f[1] - s.f[0]
    allf = sorted(f for pk in s.peaks.values() for f, _ in pk)
    out = []
    for f in allf:
        if any(abs(f - g) <= tol_bins * df for g, _ in out):
            continue
        k = sum(any(abs(f - g) <= tol_bins * df for g, _ in pk) for pk in s.peaks.values())
        if k >= min(3, len(s.peaks)) and k >= 2:
            out.append((f, k))
    return out


def resolve(specs: list[Spectrum], f_max: float, tol_bins: float = 1.5):
    """True frequencies (<= f_max) whose alias matches a peak in every burst.

    Seeds: each peak of the fastest burst, unfolded to k*fs +/- f. A candidate
    is kept when every other burst has a peak (any channel) within tol_bins of
    where the candidate would land. Returns [(f_true, seed f, [alias per burst])].
    """
    if len(specs) < 2:
        return []
    rates = {round(s.fs, 6) for s in specs}
    ref = max(specs, key=lambda s: s.fs)
    peaks = [sorted({round(f, 3) for pk in s.peaks.values() for f, _ in pk}) for s in specs]
    out = []
    for f0 in peaks[specs.index(ref)]:
        cands = {f0}
        k = 1
        while k * ref.fs - f0 <= f_max:
            cands |= {k * ref.fs - f0, k * ref.fs + f0}
            k += 1
        for f in sorted(c for c in cands if c <= f_max):
            al = [alias(f, s.fs) for s in specs]
            if all(any(abs(a - g) <= tol_bins * (s.f[1] - s.f[0]) for g in pk)
                   for a, s, pk in zip(al, specs, peaks)):
                out.append((f, f0, al))
    if len(rates) < 2:
        out = [c for c in out if c[0] == c[1]]   # nothing to discriminate with
    return out


def _resolve_text(specs, cands) -> str:
    if len(specs) < 2:
        return "true-frequency candidates: need bursts at 2+ sample rates"
    rates = sorted({s.fs for s in specs})
    nested = all(abs(r / rates[0] - round(r / rates[0])) < 1e-9 for r in rates)
    lines = ["true-frequency candidates (alias of each matches a peak in every burst):"]
    if nested:
        lines.append("   NOTE all rates are integer multiples of the slowest (same CLKIN, OSR"
                     " only): frequencies are only known modulo the fastest rate."
                     " Add a burst at another CLKIN.")
    for f, f0, al in cands:
        lines.append(f"   {f:10.3f} Hz   (seed {f0:.3f}; aliases "
                     + ", ".join(f"{a:.2f}" for a in al) + ")")
    if not cands:
        lines.append("   none")
    return "\n".join(lines)


def _track_text(specs: list[Spectrum], f0: float, half_bw: float = 2.0) -> str:
    """Per burst and live channel: DC level and the largest amplitude within f0 +/- half_bw."""
    lines = [f"tracked tone {f0:g} Hz (largest bin within +/-{half_bw:g} Hz) vs channel DC level:"]
    for s in specs:
        lines.append(f"   {s.label}")
        win = (s.f >= f0 - half_bw) & (s.f <= f0 + half_bw)
        for c in sorted(s.amp):
            if not win.any():
                lines.append(f"      CH{c}  DC {s.dc[c]:8.1f} mV   (tone above fs/2)")
                continue
            i = np.flatnonzero(win)[np.argmax(s.amp[c][win])]
            lines.append(f"      CH{c}  DC {s.dc[c]:8.1f} mV   {s.amp[c][i]:.4f} mV at {s.f[i]:.2f} Hz")
    return "\n".join(lines)


def report(bursts: list[Burst], out: Path, scan_hz: float, nperseg: int | None,
           k_noise: float, n_peaks: int, f_max: float = 5000.0,
           track: float | None = None) -> list[Spectrum]:
    specs = [spectrum(b, nperseg, k_noise, n_peaks) for b in bursts]
    text = _peak_text(specs, bursts, scan_hz)
    cands = resolve(specs, f_max)
    text += "\n" + _resolve_text(specs, cands)
    if track:
        text += "\n" + _track_text(specs, track)
    print(text)
    png = _plot(specs, bursts, out.with_suffix(".png"))
    out.with_suffix(".html").write_text(_html(specs, bursts, scan_hz, png, cands), encoding="utf-8")
    print(f"wrote {out.with_suffix('.html')}" + (f" and {out.with_suffix('.png')}" if png else
          " (no plot: matplotlib not installed; peak tables only)"))
    return specs


def _plot(specs, bursts, png_path: Path) -> bytes | None:
    try:
        import matplotlib
    except ImportError:
        return None
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(len(specs), 1, figsize=(11, 3.4 * len(specs)), squeeze=False)
    for ax, s, b in zip(axs[:, 0], specs, bursts):
        outs = b.meta.get("out", [None] * 4)
        for c, a in s.amp.items():
            ax.semilogy(s.f, a, lw=0.8, label=f"CH{c} OUT{outs[c]}")
            for f, v in s.peaks[c]:
                ax.plot(f, v, "kv", ms=4)
        for f, k in common_peaks(s):
            ax.axvline(f, color="0.6", lw=0.6, ls="--")
            ax.annotate(f"{f:.2f}", (f, ax.get_ylim()[1]), fontsize=7, ha="center",
                        va="top", rotation=90, color="0.3")
        ax.set_title(s.label, fontsize=9)
        ax.set_ylabel("amplitude, mV peak")
        ax.set_xlim(0, s.fs / 2)
        ax.grid(True, which="both", alpha=0.3)
        ax.legend(fontsize=7, loc="upper right")
    axs[-1, 0].set_xlabel("frequency, Hz")
    fig.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110)
    plt.close(fig)
    png_path.write_bytes(buf.getvalue())
    return buf.getvalue()


def _peak_text(specs, bursts, scan_hz) -> str:
    lines = []
    for s, b in zip(specs, bursts):
        lines.append(f"== {s.label}")
        lines += [f"   {n}" for n in s.notes]
        for f, k in common_peaks(s):
            lines.append(f"   common ({k} ch): {f:9.3f} Hz -> {alias(f, scan_hz):6.2f} Hz in the"
                         f" {scan_hz:g} Hz scan")
        for c, pk in s.peaks.items():
            for f, a in pk:
                lines.append(f"   CH{c} OUT{b.meta.get('out', [0]*4)[c]:<3} {f:9.3f} Hz  {a:8.4f} mV"
                             f"  -> {alias(f, scan_hz):6.2f} Hz in scan")
    return "\n".join(lines)


def _html(specs, bursts, scan_hz, png: bytes | None, cands=None) -> str:
    rows = ["<h2>True-frequency candidates</h2><pre>"
            + html.escape(_resolve_text(specs, cands or [])) + "</pre>"]
    for s, b in zip(specs, bursts):
        rows.append(f"<h2>{html.escape(s.label)}</h2>")
        if s.notes:
            rows.append("<ul>" + "".join(f"<li>{html.escape(n)}</li>" for n in s.notes) + "</ul>")
        cp = common_peaks(s)
        if cp:
            rows.append("<p><b>Common to ≥3 channels:</b> " + ", ".join(
                f"{f:.3f} Hz (→ {alias(f, scan_hz):.2f} Hz in scan)" for f, _ in cp) + "</p>")
        rows.append("<table><tr><th>ch</th><th>OUT</th><th>f, Hz</th><th>mV peak</th>"
                    f"<th>alias in {scan_hz:g} Hz scan</th></tr>")
        outs = b.meta.get("out", [None] * 4)
        for c, pk in s.peaks.items():
            for f, a in pk:
                rows.append(f"<tr><td>CH{c}</td><td>{outs[c]}</td><td>{f:.3f}</td>"
                            f"<td>{a:.4f}</td><td>{alias(f, scan_hz):.2f}</td></tr>")
        rows.append("</table>")
        rows.append("<details><summary>burst header</summary><pre>"
                    + html.escape(json.dumps(b.meta, indent=1)) + "</pre></details>")
    img = (f'<img src="data:image/png;base64,{base64.b64encode(png).decode()}" alt="spectra">'
           if png else "<p><i>No plot: matplotlib was not installed.</i></p>")
    return f"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Burst spectrum</title>
<style>
:root {{ --bg:#fff; --fg:#1b1b1b; --line:#ddd; }}
@media (prefers-color-scheme: dark) {{ :root {{ --bg:#16181c; --fg:#e6e6e6; --line:#3a3d44; }} }}
body {{ background:var(--bg); color:var(--fg); font:14px/1.45 system-ui,sans-serif;
       margin:0 auto; max-width:1100px; padding:16px; }}
img {{ max-width:100%; background:#fff; }}
table {{ border-collapse:collapse; font-variant-numeric:tabular-nums; margin:6px 0 14px; }}
td,th {{ border-bottom:1px solid var(--line); padding:2px 10px; text-align:right; }}
h2 {{ font-size:15px; margin-top:22px; }}
</style></head><body>
<h1 style="font-size:18px">Raw ADC burst spectra</h1>
<p>Amplitude = peak mV of a sinusoid at the ADC input (Welch, Hann). A real tone keeps its
frequency across bursts at different sample rates; an alias moves.</p>
{img}
{''.join(rows)}
</body></html>
"""


# ------------------------------------------------------------------------ CLI

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    an = argparse.ArgumentParser(add_help=False)
    an.add_argument("--scan-hz", type=float, default=76.2, help="normal scan rate for alias column")
    an.add_argument("--nperseg", type=int, help="Welch segment (default n/4, power of 2)")
    an.add_argument("--k-noise", type=float, default=6.0, help="peak threshold x median floor")
    an.add_argument("--n-peaks", type=int, default=8, help="peaks listed per channel")
    an.add_argument("--f-max", type=float, default=5000.0,
                    help="highest true frequency to consider when unfolding aliases")
    an.add_argument("--track", type=float, metavar="HZ",
                    help="also print each channel's DC level and its amplitude near HZ (e.g. 845)")
    c = sub.add_parser("capture", parents=[an], help="trigger burst(s) on the ESP32, save, analyze")
    c.add_argument("--port", required=True)
    c.add_argument("--sel", type=int, default=0, help="mux select 0-7")
    c.add_argument("--n", type=int, default=8192, help="conversions per burst (RAM-capped)")
    c.add_argument("--burst", nargs="+", default=DEFAULT_BURSTS, metavar="[CLKIN:]OSR",
                   help="bursts to take, e.g. 128 512 1000000:128 8000000:2048"
                        f" (default {' '.join(DEFAULT_BURSTS)})")
    c.add_argument("--spi-hz", type=int, default=600000)
    c.add_argument("--out", type=Path, default=Path(time.strftime("burst_%Y%m%d_%H%M%S")))
    a = sub.add_parser("analyze", parents=[an], help="analyze saved .npz / raw .txt dumps")
    a.add_argument("files", type=Path, nargs="+")
    a.add_argument("--out", type=Path)
    args = ap.parse_args(argv)

    if args.cmd == "capture":
        bursts, raw = capture(args.port, args.sel, args.n, parse_plan(args.burst), args.spi_hz)
        args.out.with_suffix(".txt").write_bytes(raw)
        save_npz(args.out.with_suffix(".npz"), bursts)
        print(f"saved {args.out.with_suffix('.npz')} and raw {args.out.with_suffix('.txt')}")
    else:
        bursts = [b for p in args.files for b in load(p)]
        args.out = args.out or args.files[0].with_suffix("")
    report(bursts, args.out, args.scan_hz, args.nperseg, args.k_noise, args.n_peaks, args.f_max, args.track)
    return 0


if __name__ == "__main__":
    sys.exit(main())
