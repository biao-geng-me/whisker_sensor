#!/usr/bin/env python3
"""
whisker_scope.py — live time-series scope for the 16-whisker array (32 gauges)

Voltage on Y, time on X, one coloured trace per gauge. Both gauges of a whisker
start overlapped on 0 mV and separate up or down according to the direction the
whisker is deflected.

This is the only program you need at runtime. Flash whisker_reader.ino once,
then run this — it listens to the board, puts the firmware into stream mode,
plots, and can command the board (re-zero, etc.) without the Arduino IDE.

The ESP32 lives inside the ROV, plugged into the BlueOS computer as
/dev/ttyUSB0. The Fathom tether is Ethernet, so there is no serial port on this
machine; BlueOS bridges that port to UDP and we read the UDP stream.

SETUP (one time):
    pip3 install matplotlib numpy        # pyserial is no longer needed

    Create the bridge on the vehicle, once per BlueOS reboot:
        curl -X POST http://192.168.2.2/bridget/v1.0/bridges \\
             -H 'Content-Type: application/json' \\
             -d '{"serial_path":"/dev/ttyUSB0","baud":115200,
                  "ip":"192.168.2.1","udp_target_port":27000,
                  "udp_listen_port":27001}'

RUN:
    python3 whisker_scope.py
    python3 whisker_scope.py --focus 3          # start zoomed on whisker 3
    python3 whisker_scope.py --window 60        # 60 s of history on screen
    python3 whisker_scope.py --ip 192.168.2.2 --rx-port 27000 --tx-port 27001

Only one program can bind the rx port, so whisker_plot.py and whisker_scope.py
cannot run at the same time on the same port.

VIEWS
    GRID  (default)  4x4 time-series panels, all 16 whiskers at once
    FOCUS            one whisker filling the window, like a lab scope

KEYS (with the plot window focused)
    f            switch GRID <-> FOCUS
    left/right   previous / next whisker (FOCUS)
    1..9, 0      jump to whisker 1..9, 10   (FOCUS; 0 = whisker 10)
    b            tell the board to re-capture its baseline (hold whiskers still)
    z            software re-zero right now: force every trace to 0
    d            drift tracking on/off (see BASELINE below)
    a            cycle y-axis: fixed +/-25 mV -> shared autoscale -> per-panel
    [  ]         shrink / grow the fixed y range
    +  -         longer / shorter time window
    space        pause / resume
    r            start/stop recording to CSV
    s            save a PNG snapshot
    q            quit

BASELINE / DRIFT
    Every trace is displayed as a DIFFERENCE from its own tracked baseline, so
    all 32 start at exactly 0 mV the moment the first sample arrives.

    Strain gauges and their amplifiers drift slowly (thermal, gauge creep), so
    a fixed baseline captured once at boot walks away over minutes. A slow
    baseline tracker follows that drift and subtracts it, with a hold: while a
    channel is more than HOLD_MV away from its baseline the tracker nearly
    freezes, so a real deflection is not eaten. Time constant is --tau
    (default 20 s). Press 'd' to freeze the tracker, 'z' to re-zero now.

    Consequence worth knowing: a deflection HELD for much longer than tau will
    slowly decay toward 0, because that is what "remove slow drift" means.
    Press 'd' before a static-load measurement.

CHANNELS THAT ARE NOT PLUGGED IN
    The firmware flags a channel as "open" only when its amplifier output is
    railed. An unconnected input can also float mid-range, in which case it
    behaves as an antenna and shows mains pickup rather than a flat line — real
    looking noise from a gauge that is not there. Use --only to say which
    gauges are actually connected; everything else is greyed out and ignored.
        --only W3A                 just that one gauge
        --only W3A,W3B             both gauges of whisker 3
        --only W3                  same as above
        --only 5,31                by OUT number

STREAM FORMAT (emitted by whisker_reader.ino in 'p' mode)
    $,<scan>,<railmask-hex>,<mv1>,...,<mv32>
    mV deviation from the board's baseline, in OUT order 1..32.
    OUT(2W-1) = whisker W gauge A,  OUT(2W) = whisker W gauge B.
    A set railmask bit means that channel is at full scale — nothing plugged
    in — and its trace is hidden.

THROUGHPUT NOTE
    At 115200 baud a 32-value line is ~230 bytes, capping the stream near
    45 samples/s. The serial leg between the ESP32 and the BlueOS computer is
    still the bottleneck, not the tether. To go faster, raise the baud in
    Serial.begin() in whisker_reader.ino AND in the bridge definition above
    (921600 works well on the ESP32).

TWO SOCKETS, ON PURPOSE
    Bridget adds the source endpoint of anything it receives on udp_listen_port
    to its list of output targets. Sending commands from the socket we listen
    on would register this machine's rx port as a dynamic client on top of the
    configured target - the same address twice - and every packet would arrive
    twice. So rx only listens and tx only sends. Do not merge them.
"""

import argparse
import csv
import socket
import sys
import time
from collections import deque
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

N_OUT, N_WHISKER = 32, 16
MAX_BUF = 1 << 16       # cap the line-reassembly buffer if newlines stop

# Gauge A / gauge B colours — the two channels of a whisker are always these
# two, so the pairing reads the same in every panel.
COL_A, COL_B = "#2ca02c", "#a6a03c"
Y_MIN_MV = 2.0          # never zoom tighter than +/- this, so idle looks flat
HOLD_MV  = 3.0          # above this deviation the drift tracker nearly freezes


def parse_only(spec):
    """'W3A,W16' or '5,31' -> set of OUT numbers (1..32)."""
    outs = set()
    for tok in spec.split(","):
        tok = tok.strip().upper()
        if not tok:
            continue
        if tok.startswith("W"):
            body = tok[1:]
            if body and body[-1] in "AB":
                w, g = int(body[:-1]), body[-1]
                outs.add(2 * w - 1 if g == "A" else 2 * w)
            else:
                w = int(body)
                outs.update((2 * w - 1, 2 * w))
        else:
            outs.add(int(tok))
    bad = [o for o in outs if not 1 <= o <= N_OUT]
    if bad:
        raise ValueError(f"--only: OUT number(s) out of range 1..{N_OUT}: {bad}")
    return outs


# ─────────────────────────────────────────────────────────────────────────────
#  UDP reader (BlueOS serial bridge)
# ─────────────────────────────────────────────────────────────────────────────
class Board:
    def __init__(self, ip, rx_port, tx_port):
        self.ip, self.tx_port = ip, tx_port
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind(("0.0.0.0", rx_port))      # listen only, never send
        self.rx.setblocking(False)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # send only
        self.buf = bytearray()                  # partial line between packets
        self.last_scan = None                   # drop stale/duplicate scans
        print(f"Listening on UDP {rx_port}, commands to {ip}:{tx_port}")
        time.sleep(0.3)                # no reset here: UDP cannot reboot the ESP32
        self.send("p")                 # firmware -> stream mode
        print("Streaming. Press 'f' to switch view, 'q' to quit.")

    def send(self, ch):
        try:
            self.tx.sendto(ch.encode(), (self.ip, self.tx_port))
        except OSError as e:
            print(f"[udp] write failed: {e}")

    def poll(self, max_pkts=600):
        """Return a list of (values[32], railed[32]) parsed since last call."""
        for _ in range(max_pkts):
            try:
                self.buf += self.rx.recv(65535)
            except BlockingIOError:
                break
            except OSError:
                break
        if len(self.buf) > MAX_BUF:             # no newline in sight: resync
            del self.buf[:-MAX_BUF]

        # The bridge splits on packet boundaries, not lines, so reassemble.
        chunks = self.buf.split(b"\n")
        self.buf = bytearray(chunks.pop())      # trailing fragment waits
        out = []
        for raw in chunks:
            line = raw.decode("ascii", errors="ignore").strip("\r")
            if not line.startswith("$,"):
                continue
            parts = line.strip().split(",")
            if len(parts) != 3 + N_OUT:
                continue
            try:
                scan = int(parts[1])
                mask = int(parts[2], 16)
                vals = np.array([float(v) for v in parts[3:]], dtype=float)
            except ValueError:
                continue
            if self.last_scan is not None and scan <= self.last_scan:
                continue                        # late or duplicate packet
            self.last_scan = scan
            railed = np.array([bool(mask & (1 << o)) for o in range(N_OUT)])
            out.append((vals, railed))
        return out

    def close(self):
        self.send("g")                 # leave firmware in its readable grid view
        self.tx.close()
        self.rx.close()


# ─────────────────────────────────────────────────────────────────────────────
#  Scope
# ─────────────────────────────────────────────────────────────────────────────
class Scope:
    def __init__(self, board, window_s, focus, start_grid, ylim, tau, only):
        self.board = board
        self.window = window_s
        self.focus = focus
        self.grid = start_grid
        self.ylim = ylim            # fixed half-range, mV
        self.ymode = "fixed"        # fixed | shared | per
        self.paused = False
        self.t0 = None
        self.tprev = None
        self.tau = tau              # baseline tracker time constant, seconds
        self.track = True
        self.dc = None              # tracked baseline, mV, per OUT
        self.railed = np.ones(N_OUT, dtype=bool)
        # Channels the user says are physically connected. None = trust the
        # firmware's rail flag alone.
        self.wired = None
        if only:
            self.wired = np.zeros(N_OUT, dtype=bool)
            for o in only:
                self.wired[o - 1] = True
        self.csv_w = None
        self.csv_f = None

        maxlen = 20000
        self.t = deque(maxlen=maxlen)
        self.y = [deque(maxlen=maxlen) for _ in range(N_OUT)]

        self.fig = plt.figure(figsize=(14, 8))
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.build()

    # ── figure construction ──────────────────────────────────────────────────
    def build(self):
        self.fig.clf()
        self.lines = {}
        if self.grid:
            self.axes = self.fig.subplots(4, 4, sharex=True).ravel()
            self.fig.suptitle("Whisker array — time series (all 16 whiskers)",
                              fontweight="bold")
            for w in range(N_WHISKER):
                ax = self.axes[w]
                oa, ob = 2 * w, 2 * w + 1
                self.lines[oa], = ax.plot([], [], color=COL_A, lw=1.0)
                self.lines[ob], = ax.plot([], [], color=COL_B, lw=1.0)
                ax.axhline(0, color="0.75", lw=0.7, zorder=0)
                ax.set_title(f"W{w + 1}", fontsize=9, pad=2)
                ax.tick_params(labelsize=7)
                ax.grid(True, alpha=0.25)
            self.axes[12].set_xlabel("Time (s)", fontsize=8)
            self.axes[0].set_ylabel("Voltage (mV)", fontsize=8)
            self.fig.legend([self.lines[0], self.lines[1]],
                            ["gauge A", "gauge B"], loc="upper right", ncol=2)
        else:
            ax = self.fig.add_subplot(111)
            self.axes = [ax]
            w = self.focus
            oa, ob = 2 * (w - 1), 2 * (w - 1) + 1
            self.lines[oa], = ax.plot([], [], color=COL_A, lw=1.2,
                                      label=f"Whisker {w} — gauge A (OUT{oa + 1})")
            self.lines[ob], = ax.plot([], [], color=COL_B, lw=1.2,
                                      label=f"Whisker {w} — gauge B (OUT{ob + 1})")
            ax.axhline(0, color="0.75", lw=0.8, zorder=0)
            ax.set_title("Time-series", fontweight="bold")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Voltage (mV)")
            ax.grid(True, alpha=0.3)
            ax.legend(loc="upper right", fontsize=9)
        self.fig.tight_layout(rect=[0, 0, 1, 0.95])

    def visible_outs(self):
        if self.grid:
            return range(N_OUT)
        w = self.focus
        return (2 * (w - 1), 2 * (w - 1) + 1)

    # ── data ─────────────────────────────────────────────────────────────────
    def dead(self, o):
        """True if channel o should not be drawn: railed, or not in --only."""
        if self.wired is not None and not self.wired[o]:
            return True
        return bool(self.railed[o])

    def ingest(self):
        for vals, railed in self.board.poll():
            if self.t0 is None:
                self.t0 = time.time()
            now = time.time() - self.t0
            self.railed = railed

            if self.dc is None:
                # First sample defines the baseline, so every trace starts at 0.
                self.dc = vals.copy()
            elif self.track and self.tau > 0:
                dt = max(0.0, now - self.tprev)
                alpha = min(1.0, dt / self.tau)
                delta = vals - self.dc
                # Hold: while a channel is far from baseline it is deflecting,
                # not drifting, so track it 20x slower and don't eat the signal.
                rate = np.where(np.abs(delta) < HOLD_MV, alpha, alpha * 0.05)
                self.dc += rate * delta
            self.tprev = now

            y = vals - self.dc
            self.t.append(now)
            for o in range(N_OUT):
                self.y[o].append(np.nan if self.dead(o) else y[o])
            if self.csv_w:
                self.csv_w.writerow([f"{now:.4f}"] +
                                    ["" if self.dead(o) else f"{y[o]:.3f}"
                                     for o in range(N_OUT)])

    # ── draw ─────────────────────────────────────────────────────────────────
    def update(self, _frame):
        if not self.paused:
            self.ingest()
        if not self.t:
            return list(self.lines.values())

        t = np.fromiter(self.t, dtype=float)
        tmax = t[-1]
        tmin = max(0.0, tmax - self.window)
        keep = t >= tmin
        tv = t[keep]

        peak = 0.0
        cache = {}
        for o in self.visible_outs():
            yv = np.fromiter(self.y[o], dtype=float)[keep]
            cache[o] = yv
            if not self.dead(o) and yv.size:
                m = np.nanmax(np.abs(yv)) if np.any(~np.isnan(yv)) else 0.0
                peak = max(peak, 0.0 if np.isnan(m) else m)
        lim = max(Y_MIN_MV, peak * 1.25)

        for o in self.visible_outs():
            self.lines[o].set_data(tv, cache[o])

        for i, ax in enumerate(self.axes):
            # Always show exactly the last `window` seconds, even before the
            # buffer has filled, so the x-axis width never changes.
            ax.set_xlim(tmax - self.window, tmax)
            if self.ymode == "fixed":
                ax.set_ylim(-self.ylim, self.ylim)
            elif self.ymode == "shared" or not self.grid:
                ax.set_ylim(-lim, lim)
            else:
                outs = (2 * i, 2 * i + 1)
                pk = 0.0
                for o in outs:
                    yv = cache.get(o)
                    if yv is not None and yv.size and not self.dead(o):
                        m = np.nanmax(np.abs(yv)) if np.any(~np.isnan(yv)) else 0.0
                        pk = max(pk, 0.0 if np.isnan(m) else m)
                ax.set_ylim(-max(Y_MIN_MV, pk * 1.25), max(Y_MIN_MV, pk * 1.25))
            if self.grid:
                oa, ob = 2 * i, 2 * i + 1
                both_dead = self.dead(oa) and self.dead(ob)
                ax.set_facecolor("0.94" if both_dead else "white")
                tag = ""
                if both_dead:
                    tag = "  (n/c)" if (self.wired is not None and
                                        not self.wired[oa] and not self.wired[ob]) else "  (open)"
                ax.set_title(f"W{i + 1}" + tag, fontsize=9, pad=2)

        if not self.grid:
            oa, ob = self.visible_outs()
            state = []
            for o, name in ((oa, "A"), (ob, "B")):
                state.append(f"{name}: --" if self.dead(o)
                             else f"{name}: {self.y[o][-1]:+.2f} mV")
            self.axes[0].set_title(
                f"Time-series — whisker {self.focus}    " + "    ".join(state),
                fontweight="bold")
        return list(self.lines.values())

    # ── keys ─────────────────────────────────────────────────────────────────
    def on_key(self, e):
        k = e.key
        if k == "f":
            self.grid = not self.grid
            self.build()
        elif k in ("right", "left") and not self.grid:
            self.focus = (self.focus % N_WHISKER) + 1 if k == "right" \
                         else (self.focus - 2) % N_WHISKER + 1
            self.build()
        elif k and k.isdigit():
            w = 10 if k == "0" else int(k)
            if 1 <= w <= N_WHISKER:
                self.focus, self.grid = w, False
                self.build()
        elif k == "b":
            self.board.send("b")
            print("-> board re-baselining; keep whiskers still")
        elif k == "z":
            if self.dc is not None:
                for o in range(N_OUT):
                    if self.y[o] and not np.isnan(self.y[o][-1]):
                        self.dc[o] += self.y[o][-1]
            print("-> re-zeroed now")
        elif k == "d":
            self.track = not self.track
            print(f"-> drift tracking {'ON' if self.track else 'OFF (baseline frozen)'}")
        elif k == "a":
            self.ymode = {"fixed": "shared", "shared": "per", "per": "fixed"}[self.ymode]
            print(f"-> y-axis: {self.ymode}"
                  + (f" (+/-{self.ylim:.0f} mV)" if self.ymode == "fixed" else ""))
        elif k == "[":
            self.ymode = "fixed"
            self.ylim = max(1.0, self.ylim / 2)
            print(f"-> y-axis fixed +/-{self.ylim:.1f} mV")
        elif k == "]":
            self.ymode = "fixed"
            self.ylim = min(1200.0, self.ylim * 2)
            print(f"-> y-axis fixed +/-{self.ylim:.1f} mV")
        elif k in ("+", "="):
            self.window = min(600, self.window * 1.5)
            print(f"-> window {self.window:.0f} s")
        elif k == "-":
            self.window = max(2, self.window / 1.5)
            print(f"-> window {self.window:.0f} s")
        elif k == " ":
            self.paused = not self.paused
            print("-> paused" if self.paused else "-> running")
        elif k == "r":
            self.toggle_record()
        elif k == "s":
            name = f"whisker_scope_{datetime.now():%Y%m%d_%H%M%S}.png"
            self.fig.savefig(name, dpi=150)
            print(f"-> saved {name}")
        elif k == "q":
            plt.close(self.fig)

    def toggle_record(self):
        if self.csv_w is None:
            name = f"whisker_rec_{datetime.now():%Y%m%d_%H%M%S}.csv"
            self.csv_f = open(name, "w", newline="")
            self.csv_w = csv.writer(self.csv_f)
            self.csv_w.writerow(["time_s"] +
                                [f"W{o // 2 + 1}_{'AB'[o % 2]}"
                                 for o in range(N_OUT)])
            print(f"-> recording to {name}")
        else:
            self.csv_w = None
            self.csv_f.close()
            print("-> recording stopped")

    def close(self):
        if self.csv_f:
            self.csv_f.close()


# ─────────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="Live whisker-array scope")
    ap.add_argument("--ip", default="192.168.2.2",
                    help="BlueOS computer at the far end of the tether")
    ap.add_argument("--rx-port", type=int, default=27000,
                    help="UDP port this machine listens on (bridge target)")
    ap.add_argument("--tx-port", type=int, default=27001,
                    help="UDP port the bridge listens on for commands")
    ap.add_argument("--window", type=float, default=10.0, help="seconds on screen")
    ap.add_argument("--ylim", type=float, default=25.0,
                    help="fixed y half-range in mV (default 25 => -25..+25)")
    ap.add_argument("--focus", type=int, default=0,
                    help="start focused on this whisker (1-16); 0 = grid view")
    ap.add_argument("--tau", type=float, default=20.0,
                    help="baseline drift tracker time constant in seconds; "
                         "0 disables tracking (still zeroed at start)")
    ap.add_argument("--only", default="",
                    help="gauges actually connected, e.g. 'W3A' or 'W3A,W16A' "
                         "or '5,31'. Everything else is greyed out.")
    a = ap.parse_args()

    try:
        only = parse_only(a.only) if a.only else None
    except ValueError as e:
        sys.exit(str(e))
    if only:
        print("Treating as connected: " +
              ", ".join(f"W{(o + 1) // 2}{'A' if o % 2 else 'B'}"
                        for o in sorted(only)))

    try:
        board = Board(a.ip, a.rx_port, a.tx_port)
    except OSError as e:
        sys.exit(f"Cannot bind UDP {a.rx_port}: {e}\n"
                 "Something else is already listening - whisker_plot.py or a "
                 "second copy of this script. Close it and retry.")

    scope = Scope(board, a.window,
                  focus=a.focus if a.focus else 1,
                  start_grid=(a.focus == 0),
                  ylim=a.ylim, tau=a.tau, only=only)
    ani = FuncAnimation(scope.fig, scope.update, interval=50,
                        blit=False, cache_frame_data=False)
    scope._ani = ani            # keep a reference alive
    plt.show()

    scope.close()
    board.close()


if __name__ == "__main__":
    main()
