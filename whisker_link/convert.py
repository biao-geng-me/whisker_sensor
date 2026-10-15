"""Convert a ``daq_server`` recording (.jsonl) into arrays on one time base.

    python -m whisker_link.convert rec.jsonl            # -> rec.npz
    python -m whisker_link.convert rec.jsonl --mat      # -> rec.mat as well

Time base: seconds on the Pi's monotonic clock, zero at the recording start.

Every event carries ``t``, the moment the server *received* it. That includes
USB/UDP latency and scheduling jitter (~ms). Each source also has its own
sample clock (ESP32 ``tus``, autopilot ``time_boot_ms``/``time_usec``), which
is precise but on a different oscillator. For each source we fit

    t_receive ≈ a + b * t_source

along the *lower envelope* of the receive times (receive can be late, never
early), and report ``a + b * t_source`` as the sample time. ``b`` absorbs
crystal drift (tens of ppm = tens of ms per hour on the ESP32).

Output keys (npz; the .mat has the same names):
    wsk_t, wsk_t_rx, wsk_seq, wsk_raw (N,32 int32), wsk_mv (N,32), wsk_rail,
    channels, mv_per_code
    <TYPE>_t, <TYPE>_t_rx, <TYPE>_<field>     for each MAVLink message type
    mark_t, mark_text
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

from whisker_link.wire import iter_file

# MAVLink fields that carry the sender's own sample clock, in priority order.
_MAV_CLOCKS = (("time_usec", 1e-6), ("time_boot_ms", 1e-3))


def envelope_fit(src_s: np.ndarray, rx_s: np.ndarray, win_s: float = 5.0) -> tuple[float, float]:
    """Fit rx ≈ a + b*src on the lower envelope. Returns (a, b)."""
    if len(src_s) < 2:
        return (float(rx_s[0] - src_s[0]) if len(src_s) else 0.0), 1.0
    lag = rx_s - src_s
    bins = np.floor((src_s - src_s[0]) / win_s).astype(np.int64)
    xs, ys = [], []
    for b in np.unique(bins):
        idx = np.nonzero(bins == b)[0]
        j = idx[np.argmin(lag[idx])]
        xs.append(src_s[j])
        ys.append(rx_s[j])
    xs, ys = np.asarray(xs), np.asarray(ys)
    if len(xs) >= 3:
        b, a = np.polyfit(xs, ys, 1)
    else:
        b = 1.0
        a = float(np.min(lag))
    # Shift so the line sits on (not through) the envelope.
    a += float(np.min(rx_s - (a + b * src_s)))
    return float(a), float(b)


def _unwrap_u32_us(t_us: np.ndarray) -> np.ndarray:
    t = t_us.astype(np.int64)
    jumps = np.diff(t) < -(1 << 31)
    return t + np.concatenate([[0], np.cumsum(jumps)]) * (1 << 32)


def load(path: str | Path) -> dict:
    hello = None
    wsk = defaultdict(list)
    mav: dict[str, list[dict]] = defaultdict(list)
    mav_t: dict[str, list[float]] = defaultdict(list)
    marks, texts = [], []
    with open(path, encoding="utf-8") as f:
        for ev in iter_file(f):
            k = ev.get("k")
            if k == "wsk":
                for key in ("t", "seq", "tus", "rail", "flg", "raw"):
                    wsk[key].append(ev[key])
            elif k == "mav":
                mav[ev["type"]].append(ev["d"])
                mav_t[ev["type"]].append(ev["t"])
            elif k == "mark":
                marks.append((ev["t"], ev.get("text", "")))
            elif k == "txt":
                texts.append((ev["t"], ev.get("src", ""), ev.get("msg", "")))
            elif k == "hello" and hello is None:
                hello = ev
    if hello is None:
        raise ValueError(f"{path}: no hello line — not a daq_server recording?")
    return {"hello": hello, "wsk": wsk, "mav": mav, "mav_t": mav_t,
            "marks": marks, "texts": texts}


def to_arrays(rec: dict) -> dict[str, np.ndarray]:
    hello = rec["hello"]
    t0 = float(hello["t"])
    out: dict[str, np.ndarray] = {
        "t0_mono": np.float64(t0),
        "t0_unix": np.float64(hello.get("unix", np.nan) - (hello.get("mono", t0) - t0)),
        "channels": np.asarray(hello["channels"]),
        "mv_per_code": np.float64(hello["mv_per_code"]),
    }
    fits = {}

    w = rec["wsk"]
    if w["t"]:
        t_rx = np.asarray(w["t"]) - t0
        tus = _unwrap_u32_us(np.asarray(w["tus"]))   # server already unwraps; harmless
        src = (tus - tus[0]) * 1e-6
        a, b = envelope_fit(src, t_rx)
        raw = np.asarray(w["raw"], dtype=np.int32)
        out.update(wsk_t=a + b * src, wsk_t_rx=t_rx, wsk_seq=np.asarray(w["seq"]),
                   wsk_raw=raw, wsk_mv=raw * out["mv_per_code"],
                   wsk_rail=np.asarray(w["rail"], dtype=np.uint32),
                   wsk_flags=np.asarray(w["flg"], dtype=np.uint8))
        fits["wsk"] = (a, b)

    for mtype, rows in rec["mav"].items():
        t_rx = np.asarray(rec["mav_t"][mtype]) - t0
        out[f"{mtype}_t_rx"] = t_rx
        t = t_rx
        for field, scale in _MAV_CLOCKS:
            if field in rows[0]:
                src = np.asarray([r.get(field, 0) for r in rows], dtype=np.float64) * scale
                if len(src) > 1 and np.all(np.diff(src) >= 0):
                    a, b = envelope_fit(src - src[0], t_rx)
                    t = a + b * (src - src[0])
                    fits[mtype] = (a, b)
                break
        out[f"{mtype}_t"] = t
        for field in rows[0]:
            vals = [r.get(field) for r in rows]
            if all(isinstance(v, (int, float)) for v in vals):
                out[f"{mtype}_{field}"] = np.asarray(vals)
            elif all(isinstance(v, list) and v and all(isinstance(x, (int, float)) for x in v)
                     for v in vals) and len({len(v) for v in vals}) == 1:
                out[f"{mtype}_{field}"] = np.asarray(vals)

    if rec["marks"]:
        out["mark_t"] = np.asarray([m[0] for m in rec["marks"]]) - t0
        out["mark_text"] = np.asarray([m[1] for m in rec["marks"]])
    out["clock_fits"] = np.asarray(json.dumps({k: {"a": a, "ppm": (b - 1) * 1e6}
                                               for k, (a, b) in fits.items()}))
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("files", nargs="+")
    ap.add_argument("--mat", action="store_true", help="also write a MATLAB .mat")
    args = ap.parse_args(argv)
    for fn in args.files:
        arrays = to_arrays(load(fn))
        stem = Path(fn).with_suffix("")
        np.savez_compressed(f"{stem}.npz", **arrays)
        msg = f"{fn} -> {stem}.npz"
        if args.mat:
            from scipy.io import savemat
            savemat(f"{stem}.mat", {k.replace(".", "_"): v for k, v in arrays.items()},
                    do_compression=True)
            msg += f", {stem}.mat"
        n = len(arrays.get("wsk_t", []))
        dur = float(arrays["wsk_t"][-1] - arrays["wsk_t"][0]) if n > 1 else 0.0
        print(f"{msg}  ({n} scans, {dur:.1f} s, fits {arrays['clock_fits']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
