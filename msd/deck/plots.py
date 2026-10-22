"""Figures for the DAQ PCB troubleshooting deck (from the bench logs in data/)."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import welch
from load import load_dat, load_jsonl

plt.rcParams.update({"font.size": 11, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.grid": True, "grid.color": "#e4e4e4", "savefig.dpi": 200,
                     "axes.titlesize": 12, "axes.titleweight": "bold"})
C1, C2, C3, GREY = "#1f4e9c", "#c0392b", "#2e8b57", "#888888"
D = "data/"


def hp(x, fs, win_s=1.0):
    """Remove slow drift: subtract a running median over win_s."""
    n = max(3, int(win_s * fs) | 1)
    pad = np.pad(x, n // 2, mode="edge")
    med = np.array([np.median(pad[i:i + n]) for i in range(len(x))])
    return x - med


def seg(t, x, t0, dur):
    m = (t >= t[0] + t0) & (t < t[0] + t0 + dur)
    return t[m] - t[m][0], x[m]


# 1. DRDY fix: W2_A before / after --------------------------------------------------------
tb, vb = load_dat(D + "sensor_log.dat")
ta, va = load_dat(D + "st_2026-10-20_1050_25.70_.dat")
fig, ax = plt.subplots(1, 2, figsize=(11, 3.8), sharey=True)
for a, (t, v, lab, col) in zip(ax, ((tb, vb, "before (2026-10-19, 83 Hz)", C2),
                                   (ta, va, "after the DRDY fix (2026-10-20, 77 Hz)", C1))):
    fs = 1 / np.median(np.diff(t))
    x = hp(v[:, 2], fs)
    tt, xx = seg(t, x, 0, 5)
    a.plot(tt, xx, color=col, lw=0.9)
    d = np.diff(v[:, 2])
    a.set_title(lab)
    a.text(0.02, 0.95, f"sample-to-sample noise {d.std() / 2 ** .5:.2f} mV\nlargest jump {abs(d).max():.0f} mV",
           transform=a.transAxes, va="top", fontsize=10, bbox=dict(fc="white", ec="none", alpha=.8))
    a.set_xlabel("time (s)")
ax[0].set_ylabel("W2_A (OUT3), mV\n(slow drift removed)")
fig.tight_layout(); fig.savefig("fig_drdy.png"); plt.close(fig)

# 2. floating REF: slow drift of every live channel ------------------------------------------
t, v = load_dat(D + "st_2026-10-20_1618_57.40_w2_still_then_bend.dat")
fig, ax = plt.subplots(figsize=(11, 3.6))
for i, col, lab in ((2, C1, "W2_A"), (3, C2, "W2_B")):
    ax.plot(t - t[0], v[:, i], color=col, lw=0.9, label=lab)
ax.set_xlabel("time (s)"); ax.set_ylabel("output, mV")
ax.set_title("2026-10-20: W2 (gauge + 22 Ω) rests at ~1.09 V; expected ~0.5 V with REF ≈ 0")
ax.legend(frameon=False, loc="upper right")
fig.tight_layout(); fig.savefig("fig_floatref.png"); plt.close(fig)

# 3. ADC power-on default: 12.2 Hz vs 76 Hz ---------------------------------------------------
fig, ax = plt.subplots(1, 2, figsize=(11, 3.8), gridspec_kw={"width_ratios": [1.1, 1]})
rows = []
for f, lab, col in (("st_2026-10-21_1637_21.79_rov_power_5v", "board powered after ESP32", C2),
                    ("st_2026-10-21_1638_41.13_rov_power_5v_reset", "after ESP32 reset", C1)):
    W, S = load_jsonl(D + f + ".jsonl")
    tus = np.array([w["tus"] for w in W], float)
    dt = np.diff(tus) / 1000
    ax[0].plot(np.arange(len(dt))[:300], dt[:300], ".", ms=3, color=col, label=f"{lab}: {np.median(dt):.2f} ms")
    t, v = load_dat(D + f + ".dat")
    rows.append([v[:, i].std() for i in (1, 10, 11)])
ax[0].set_ylim(0, 95); ax[0].set_ylabel("time between scans (ms)"); ax[0].set_xlabel("scan #")
ax[0].legend(frameon=False, fontsize=9, loc="center right"); ax[0].set_title("scan period")
x = np.arange(3); w = 0.38
ax[1].bar(x - w / 2, rows[0], w, color=C2, label="OSR 1024 (12.2 Hz)")
ax[1].bar(x + w / 2, rows[1], w, color=C1, label="OSR 128 (76.2 Hz)")
ax[1].set_xticks(x, ["W1_B (370 Ω)", "W6_A", "W6_B"]); ax[1].set_ylabel("std, mV")
ax[1].legend(frameon=False, fontsize=9); ax[1].set_title("noise: longer averaging is quieter")
fig.tight_layout(); fig.savefig("fig_scan12.png"); plt.close(fig)

# 4. ground wiring: two twisted wires vs single wire -----------------------------------------
fig, ax = plt.subplots(1, 2, figsize=(11, 3.6), sharey=True)
for a, (f, lab, col) in zip(ax, (("st_2026-10-21_1456_17.68_w6_connected_still.dat",
                                  "two ground wires (twisted with CLKIN, SCLK)", C2),
                                 ("st_2026-10-21_1546_19.08_old_gnd_w6_still.dat",
                                  "one ground wire", C1))):
    t, v = load_dat(D + f)
    fs = 1 / np.median(np.diff(t))
    x = v[:, 10] - v[:, 10].mean()
    tt, xx = seg(t, x, 2, 4)
    a.plot(tt, xx, color=col, lw=0.9)
    d = np.diff(v[:, 10])
    a.set_title(lab, fontsize=11)
    a.text(0.02, 0.95, f"{fs:.1f} scans/s\nsample-to-sample {d.std() / 2 ** .5:.2f} mV",
           transform=a.transAxes, va="top", fontsize=10, bbox=dict(fc="white", ec="none", alpha=.8))
    a.set_xlabel("time (s)")
ax[0].set_ylabel("W6_A − mean, mV")
fig.tight_layout(); fig.savefig("fig_ground.png"); plt.close(fig)

# 5. the 6-7 Hz tone in the scan data ----------------------------------------------------------
t, v = load_dat(D + "st_2026-10-21_1605_14.63_old_gnd_w6_w1b_370ohm_still.dat")
fs = 1 / np.median(np.diff(t))
fig, ax = plt.subplots(1, 2, figsize=(11, 3.8), gridspec_kw={"width_ratios": [1.3, 1]})
for i, col, lab in ((10, C1, "W6_A"), (11, C3, "W6_B"), (1, C2, "W1_B (370 Ω resistor)")):
    x = v[:, i] - v[:, i].mean()
    tt, xx = seg(t, x, 5, 2)
    ax[0].plot(tt, xx, color=col, lw=1, label=lab)
    f, p = welch(x, fs=fs, nperseg=1024, scaling="spectrum")
    ax[1].plot(f, np.sqrt(2 * p), color=col, lw=1, label=lab)
ax[0].set_xlabel("time (s)"); ax[0].set_ylabel("output − mean, mV"); ax[0].set_ylim(-0.45, 0.6)
ax[0].legend(frameon=False, fontsize=9, ncol=3, loc="upper left", bbox_to_anchor=(0, 1.13))
ax[1].set_xlabel("frequency (Hz)"); ax[1].set_ylabel("amplitude, mV"); ax[1].set_xlim(0, 38)
ax[1].set_title("spectrum: one tone near 6.6 Hz")
fig.tight_layout(); fig.savefig("fig_tone.png"); plt.close(fig)

# 6. cable swap bursts ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(11, 3.6))
labels = ["burst 1\nW2 direct · W6 on cable+22 Ω", "burst 2 (cable swapped)\nW2 on cable+22 Ω · W6 direct",
          "burst 3\nW2 on other cable, no R · W6 direct"]
w2 = [0.0339, 0.3114, 0.0249]; w6 = [0.3116, 0.0310, 0.0307]
x = np.arange(3); w = 0.36
b1 = ax.bar(x - w / 2, w2, w, color=C2, label="W2_A slot (CH2)")
b2 = ax.bar(x + w / 2, w6, w, color=C1, label="W6_A slot (CH3)")
for b in list(b1) + list(b2):
    ax.text(b.get_x() + b.get_width() / 2, b.get_height() + 0.005, f"{b.get_height():.3f}", ha="center", fontsize=9)
ax.set_xticks(x, labels, fontsize=10); ax.set_ylabel("845 Hz amplitude, mV")
ax.legend(frameon=False); ax.set_title("the 845 Hz follows the cable with the inline resistors")
fig.tight_layout(); fig.savefig("fig_cable.png"); plt.close(fig)

# 7. imbalance line ---------------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(7.4, 5.4))
pts = [(30, 0.0199, "330+22 Ω (burst 4)"), (400, 0.2516, "369 Ω (burst 4)"),
       (762 - 294, 0.3116, "W6 + 22 Ω (burst 1; DC from the 10-21 log)")]
k = 0.2516 / 400
xx = np.linspace(0, 900, 50)
ax.plot(xx, k * xx, color=GREY, lw=1, ls="--", label=f"line through REF: {k * 1000:.2f} mV per V of offset")
for (o, a, lab), col in zip(pts, (C3, C1, C2)):
    ax.plot(o, a, "o", ms=8, color=col, label=lab)
for R in (360, 383, 390):
    o = 1000 * 33 * (0.5 - 352 / (R + 352))
    ax.plot(o, k * o, "o", ms=8, mfc="white", mec=GREY)
    ax.annotate(f"{R} Ω\n(predicted)", (o, k * o), textcoords="offset points", xytext=(6, -22), fontsize=9, color=GREY)
ax.set_xlabel("channel DC level − REF (mV)"); ax.set_ylabel("845 Hz amplitude, mV")
ax.set_xlim(0, 900); ax.set_ylim(0, 0.62)
ax.legend(frameon=False, fontsize=9, loc="upper left")
ax.set_title("845 Hz amplitude vs bridge offset")
fig.tight_layout(); fig.savefig("fig_imbalance.png"); plt.close(fig)

# 8. dead mux position: W3_B / W8_B across logs -------------------------------------------------
fig, ax = plt.subplots(figsize=(7.4, 5.0))
logs = ["st_2026-10-20_1557_33.13_still", "st_2026-10-20_1618_57.40_w2_still_then_bend",
        "st_2026-10-21_1546_19.08_old_gnd_w6_still", "st_2026-10-21_1605_14.63_old_gnd_w6_w1b_370ohm_still"]
m6 = []; m16 = []
for f in logs:
    t, v = load_dat(D + f + ".dat")
    m6.append(v[:, 5].mean()); m16.append(v[:, 15].mean())
x = np.arange(len(logs))
ax.axhline(1200, color=GREY, ls=":", lw=1); ax.text(3.4, 1150, "an empty slot reads 1200 mV (railed)", ha="right", fontsize=9, color=GREY)
b1 = ax.bar(x - 0.18, m6, 0.36, color=C2, label="W3_B (mux 1 X6, pin 3)")
b2 = ax.bar(x + 0.18, m16, 0.36, color=C1, label="W8_B (mux 1 Y6, pin 16)")
for b in list(b1) + list(b2):
    h = b.get_height()
    ax.text(b.get_x() + b.get_width() / 2, (h - 60) if h < 0 else (h + 20), f"{h:.0f}", ha="center", fontsize=9)
ax.set_xticks(x, ["10-20 15:57", "10-20 16:18", "10-21 15:46", "10-21 16:05"])
ax.set_ylabel("mean, mV (no gauge)"); ax.set_ylim(-200, 1300); ax.legend(frameon=False, loc="center left")
ax.set_title("mux 1 position 6: empty slots read ≈ 0, not 1200")
fig.tight_layout(); fig.savefig("fig_mux6.png"); plt.close(fig)
print("ok")
