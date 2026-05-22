#!/usr/bin/env python3
"""
Plot lab-style 32-channel .dat files.

Usage:
    python3 plot_32ch_dat.py data_32ch/whisker16_32ch_voltage_YYYYMMDD_HHMMSS.dat

Creates:
    <input>_X_channels.png
    <input>_Y_channels.png
"""

from __future__ import annotations

import sys
from pathlib import Path
from datetime import datetime

import matplotlib.pyplot as plt


def load_dat(path: Path):
    times = []
    data = []

    with path.open() as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 34:
                continue

            date_str, time_str = parts[0], parts[1]
            values = [float(x) for x in parts[2:34]]
            dt = datetime.strptime(date_str + " " + time_str, "%Y-%m-%d %H:%M:%S.%f")
            times.append(dt)
            data.append(values)

    if not data:
        raise SystemExit(f"No valid rows found in {path}")

    t0 = times[0]
    t = [(x - t0).total_seconds() for x in times]
    signals = list(zip(*data))
    return t, signals


def main():
    if len(sys.argv) != 2:
        raise SystemExit("Usage: python3 plot_32ch_dat.py <file.dat>")

    path = Path(sys.argv[1])
    t, signals = load_dat(path)

    x_names = [f"Ch{i+1:02d}" for i in range(16)]
    y_names = [f"Ch{i+17:02d}" for i in range(16)]

    out_x = path.with_name(path.stem + "_X_channels.png")
    out_y = path.with_name(path.stem + "_Y_channels.png")

    plt.figure(figsize=(14, 8))
    for i in range(16):
        plt.plot(t, signals[i], label=x_names[i])
    plt.xlabel("Time (s)")
    plt.ylabel("Voltage (V) or raw count")
    plt.title("16 Whisker Array: Channels 1-16")
    plt.legend(ncol=4, fontsize=8)
    plt.tight_layout()
    plt.savefig(out_x, dpi=200)

    plt.figure(figsize=(14, 8))
    for i in range(16, 32):
        plt.plot(t, signals[i], label=y_names[i - 16])
    plt.xlabel("Time (s)")
    plt.ylabel("Voltage (V) or raw count")
    plt.title("16 Whisker Array: Channels 17-32")
    plt.legend(ncol=4, fontsize=8)
    plt.tight_layout()
    plt.savefig(out_y, dpi=200)

    print(f"Loaded {len(t)} samples from {path}")
    print(f"Saved {out_x}")
    print(f"Saved {out_y}")


if __name__ == "__main__":
    main()
