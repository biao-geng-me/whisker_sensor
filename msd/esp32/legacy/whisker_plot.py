#!/usr/bin/env python3
"""
whisker_plot.py — live 4x4 grid of all 16 whiskers (32 gauges), over the tether

Reads the plot stream from whisker_reader.ino and draws one small plot per
whisker, with two traces each (gauge A solid, gauge B dashed). Everything sits
flat on 0 mV until a whisker is deflected, then that whisker's trace moves and
its panel is highlighted.

The ESP32 lives inside the ROV, plugged into the BlueOS computer as
/dev/ttyUSB0. The Fathom tether is Ethernet, so there is no serial port on this
machine any more — BlueOS bridges that port to UDP and we read the UDP stream.

SETUP (one time):
    pip3 install matplotlib          # pyserial is no longer needed

    Create the bridge on the vehicle, once per BlueOS reboot:
        curl -X POST http://192.168.2.2/bridget/v1.0/bridges \\
             -H 'Content-Type: application/json' \\
             -d '{"serial_path":"/dev/ttyUSB0","baud":115200,
                  "ip":"192.168.2.1","udp_target_port":27000,
                  "udp_listen_port":27001}'

    Check it is there:   curl -s http://192.168.2.2/bridget/v1.0/bridges

RUN:
    python3 whisker_plot.py
    python3 whisker_plot.py --log                      # log from the start
    python3 whisker_plot.py 192.168.2.2 27000 27001    # ip, rx port, tx port

--log starts CSV logging automatically once the zero capture finishes, so an
unattended run records everything without anyone pressing a key. 'l' still
toggles it by hand.

Nothing else may be bound to the rx port while this runs.

KEYS (with the plot window focused):
    z   re-zero here (hold all whiskers still first) - host side, instant
    b   ask the firmware to redo its own baseline
    a   toggle shared y-axis  <->  per-whisker autoscale
    l   start/stop CSV logging to whisker_log_<timestamp>.csv
    q   quit

ZERO
    Panels show deviation from rest, not absolute level. A channel can idle on
    a large constant offset and still respond normally to deflection, so on
    startup we average each connected channel's first AUTOZERO scans and
    subtract that. Every trace therefore begins at 0 and only movement shows.
    The offsets are printed to the console and written as the first row of any
    CSV, so absolute firmware readings stay recoverable.

CSV LAYOUT
    # zero_mV , <32 rest values, same column order as below>
    scan , W1_A , W1_B , W2_A , W2_B , ... , W16_A , W16_B
    Values are mV deviation from rest. Unconnected gauges log exactly 0.

STREAM FORMAT (from the firmware):
    $,<scan>,<railmask-hex>,<mv1>,...,<mv32>
    mV deviation from baseline, in OUT order 1..32.
    OUT(2W-1) = whisker W gauge A,  OUT(2W) = whisker W gauge B.
    railmask bit (o-1) set => that channel is at full scale, i.e. nothing
    plugged in; its panel is greyed out and labelled "open".

A NOTE ON UDP
    The bridge splits the serial stream on packet boundaries, not line
    boundaries, so we reassemble lines from a byte buffer and validate every
    field before trusting a line. We also drop any scan whose number is not
    newer than the last one accepted. Counters for rejected and out-of-order
    lines are in the window title; if they climb steadily something is wrong.

    TWO SOCKETS, ON PURPOSE. Bridget adds the source endpoint of anything it
    receives on udp_listen_port to its list of output targets. If we sent
    commands from the same socket we listen on, it would register
    192.168.2.1:<RX_PORT> as a dynamic client on top of the statically
    configured target - the same address twice - and deliver every packet
    twice. Measured: 261 duplicate pairs out of 524 packets one way, 0 the
    other. So RX only ever listens and TX only ever sends. Do not merge them.
"""

import sys
import csv
import socket
import time
from collections import deque
from datetime import datetime

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ── config ───────────────────────────────────────────────────────────────────
_flags    = [a for a in sys.argv[1:] if a.startswith("-")]
_pos      = [a for a in sys.argv[1:] if not a.startswith("-")]
AUTO_LOG  = "--log" in _flags        # start logging as soon as zeroing is done
ROV_IP    = _pos[0] if len(_pos) > 0 else "192.168.2.2"
RX_PORT   = int(_pos[1]) if len(_pos) > 1 else 27000   # we listen here
TX_PORT   = int(_pos[2]) if len(_pos) > 2 else 27001   # bridge listens here
WINDOW    = 300      # samples shown per panel
N_OUT     = 32
N_WHISKER = 16
Y_MIN_MV  = 2.0      # never zoom tighter than +/- this, so idle looks flat
Y_MAX_MV  = 50.0     # never zoom wider than +/- this, so one large offset
                     # cannot flatten every other panel. A trace beyond it
                     # just leaves the view; its value is still in the label.
AUTOZERO  = 40       # scans averaged to find each channel's rest value

# OUT(2W-1) is whisker W gauge A, OUT(2W) is gauge B, so the stream order
# already pairs up per whisker: W1_A, W1_B, W2_A, W2_B, ... W16_A, W16_B.
CH_NAME   = [f"W{o // 2 + 1}_{'AB'[o % 2]}" for o in range(32)]
MAX_PKTS  = 600      # packets drained per frame
MAX_BUF   = 1 << 16  # cap the reassembly buffer if newlines stop arriving

# ── sockets ──────────────────────────────────────────────────────────────────
# rx listens and never sends; tx sends and is never read. Keeping them separate
# is what stops the bridge duplicating every packet - see the note above.
rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    rx.bind(("0.0.0.0", RX_PORT))
except OSError as e:
    sys.exit(f"Cannot bind UDP {RX_PORT}: {e}\n"
             f"Something else is already listening - close it and retry.")
rx.setblocking(False)

tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)   # ephemeral source port


def send(cmd: bytes) -> None:
    """Send a firmware command up the tether to the ESP32."""
    try:
        tx.sendto(cmd, (ROV_IP, TX_PORT))
    except OSError as e:
        print(f"!! send failed: {e}")


print(f"Listening on UDP {RX_PORT}, commands to {ROV_IP}:{TX_PORT}")
time.sleep(0.3)                 # no reset here: UDP does not reboot the ESP32
send(b"p")                      # switch firmware into stream mode
print("Sent 'p' (plot stream). Keys in the plot window: z=re-zero "
      "b=firmware baseline a=autoscale l=log q=quit")

# ── state ────────────────────────────────────────────────────────────────────
bufs   = [deque([0.0] * WINDOW, maxlen=WINDOW) for _ in range(N_OUT)]
railed = [False] * N_OUT
raw    = [0.0] * N_OUT          # what the firmware sent
latest = [0.0] * N_OUT          # raw minus the rest value: what we plot
shared_y = True
log_writer = None
log_file = None

# Host-side zero. A channel can sit on a large constant offset at rest (W4B
# idles near -1188 mV) while still responding normally to deflection, so we
# average each channel's first AUTOZERO scans and subtract that. Everything
# then starts at 0 and the panels show change, not absolute level. Press 'z'
# to re-zero; 'b' still asks the firmware to redo its own baseline.
zero    = [0.0] * N_OUT
z_sum   = [0.0] * N_OUT
z_count = [0] * N_OUT
zeroing = True

rx_buf = bytearray()            # partial line carried between packets
last_scan = None                # highest scan number accepted so far
n_good = n_bad = n_oos = 0      # accepted / rejected / out-of-order lines

# ── figure: 4x4 grid, one panel per whisker ──────────────────────────────────
fig, axes = plt.subplots(4, 4, figsize=(14, 9), sharex=True)
fig.suptitle("Seal whisker array — 16 whiskers x 2 gauges", fontsize=13)
axes = axes.ravel()

COL_A, COL_B = "#1f77b4", "#d62728"
lines, labels = [], []
for w in range(N_WHISKER):
    ax = axes[w]
    la, = ax.plot(range(WINDOW), bufs[2 * w],     color=COL_A, lw=1.2, label="A")
    lb, = ax.plot(range(WINDOW), bufs[2 * w + 1], color=COL_B, lw=1.2, ls="--", label="B")
    lines.append((la, lb))
    ax.axhline(0, color="0.7", lw=0.7, zorder=0)
    ax.set_title(f"W{w + 1}", fontsize=9, pad=2)
    ax.tick_params(labelsize=7)
    ax.grid(True, alpha=0.25)
    ax.set_xlim(0, WINDOW)
    labels.append(ax.text(0.02, 0.04, "", transform=ax.transAxes,
                          fontsize=7, family="monospace"))
axes[12].set_xlabel("sample", fontsize=8)
axes[0].set_ylabel("mV", fontsize=8)
fig.legend(handles=list(lines[0]), labels=["gauge A", "gauge B"],
           loc="upper right", fontsize=9, ncol=2)


def parse(line: str) -> bool:
    """Consume one '$,...' stream line. Returns True if it was a data line."""
    global last_scan, n_oos
    if not line.startswith("$,"):
        return False
    parts = line.strip().split(",")
    if len(parts) != 3 + N_OUT:
        return False
    try:
        scan = int(parts[1])
        mask = int(parts[2], 16)
        vals = [float(v) for v in parts[3:]]
    except ValueError:
        return False

    # UDP can deliver a packet late or twice; only ever move forward.
    if last_scan is not None and scan <= last_scan:
        n_oos += 1
        return True
    last_scan = scan

    global zeroing
    if zeroing:
        for o in range(N_OUT):
            if not (mask & (1 << o)):        # only average connected channels
                z_sum[o] += vals[o]
                z_count[o] += 1
        if max(z_count) >= AUTOZERO:
            finish_zero()

    for o in range(N_OUT):
        railed[o] = bool(mask & (1 << o))
        raw[o] = vals[o]
        latest[o] = 0.0 if railed[o] else vals[o] - zero[o]
        bufs[o].append(latest[o])
    if log_writer:
        log_writer.writerow([parts[1]] + [f"{latest[o]:.3f}" for o in range(N_OUT)])
    return True


def finish_zero() -> None:
    """Average what we collected into per-channel rest values."""
    global zeroing
    for o in range(N_OUT):
        zero[o] = z_sum[o] / z_count[o] if z_count[o] else 0.0
    zeroing = False
    for b in bufs:                           # drop the pre-zero history
        b.clear(); b.extend([0.0] * WINDOW)
    offs = [f"{CH_NAME[o]}={zero[o]:+.2f}"
            for o in range(N_OUT) if z_count[o]]
    print("-> zeroed. rest values subtracted: " + ", ".join(offs))
    if AUTO_LOG:                             # --log: begin as soon as we can
        start_log()


def start_zero() -> None:
    """Begin a fresh zero capture; whiskers should be at rest."""
    global zeroing
    for o in range(N_OUT):
        z_sum[o] = 0.0
        z_count[o] = 0
    zeroing = True
    print(f"-> zeroing over the next {AUTOZERO} scans, hold everything still")


def drain() -> None:
    """Pull every waiting packet, reassemble whole lines, parse them."""
    global rx_buf, n_good, n_bad
    for _ in range(MAX_PKTS):
        try:
            rx_buf += rx.recv(65535)
        except BlockingIOError:
            break
        except OSError:
            break

    if len(rx_buf) > MAX_BUF:           # no newline in sight: resync
        del rx_buf[:-MAX_BUF]

    chunks = rx_buf.split(b"\n")
    rx_buf = bytearray(chunks.pop())    # trailing fragment waits for more
    for raw in chunks:
        text = raw.decode("ascii", errors="ignore").strip("\r")
        if not text.strip():
            continue
        if parse(text):
            n_good += 1
        elif text.startswith("$"):      # looked like data but did not validate
            n_bad += 1


def update(_frame):
    drain()
    state = "ZEROING, hold still" if zeroing else "deviation from rest"
    fig.suptitle(
        f"Whisker array · {state} · scan "
        f"{last_scan if last_scan is not None else '—'} · "
        f"ok {n_good}  rej {n_bad}  oos {n_oos}",
        fontsize=11, x=0.42)

    # y-limits: symmetric, clamped into [Y_MIN_MV, Y_MAX_MV]
    if shared_y:
        peak = max((max(abs(v) for v in bufs[o]) for o in range(N_OUT)
                    if not railed[o]), default=0.0)
        lim = min(Y_MAX_MV, max(Y_MIN_MV, peak * 1.25))

    for w in range(N_WHISKER):
        ax = axes[w]
        oa, ob = 2 * w, 2 * w + 1
        lines[w][0].set_ydata(bufs[oa])
        lines[w][1].set_ydata(bufs[ob])

        both_open = railed[oa] and railed[ob]
        ax.set_facecolor("0.93" if both_open else "white")

        if shared_y:
            ax.set_ylim(-lim, lim)
        else:
            pk = max((max(abs(v) for v in bufs[o]) for o in (oa, ob)
                      if not railed[o]), default=0.0)
            lim_w = min(Y_MAX_MV, max(Y_MIN_MV, pk * 1.25))
            ax.set_ylim(-lim_w, lim_w)

        def fmt(o):
            return "open" if railed[o] else f"{latest[o]:+7.2f}"
        labels[w].set_text(f"A{fmt(oa)}  B{fmt(ob)}")

        # highlight the panel of anything actually moving
        active = any(not railed[o] and abs(latest[o]) > max(Y_MIN_MV, 1.0)
                     for o in (oa, ob))
        for s in ax.spines.values():
            s.set_color("#e07b00" if active else "0.6")
            s.set_linewidth(2.0 if active else 0.8)

    return [ln for pair in lines for ln in pair]


def start_log() -> None:
    """Open a CSV and write the offset row. Refuses while still zeroing."""
    global log_writer, log_file
    if log_writer is not None:
        return
    if zeroing:
        # starting now would stamp an all-zero offset row and log raw values
        # until the capture finishes - refuse instead
        print("-> still zeroing, wait a second then press 'l' again")
        return
    name = f"whisker_log_{datetime.now():%Y%m%d_%H%M%S}.csv"
    log_file = open(name, "w", newline="")
    log_writer = csv.writer(log_file)
    # values are deviation from rest; keep the offsets so the absolute
    # firmware readings can be reconstructed later
    log_writer.writerow(["# zero_mV"] + [f"{z:.3f}" for z in zero])
    log_writer.writerow(["scan"] + CH_NAME)
    print(f"-> logging to {name}")


def stop_log() -> None:
    global log_writer, log_file
    if log_writer is None:
        return
    log_writer = None
    log_file.close()
    log_file = None
    print("-> logging stopped")


def on_key(event):
    global shared_y
    if event.key == "z":
        start_zero()
    elif event.key == "b":
        send(b"b")
        print("-> firmware rebaseline (keep whiskers still)")
    elif event.key == "a":
        shared_y = not shared_y
        print(f"-> y-axis: {'shared' if shared_y else 'per-whisker autoscale'}")
    elif event.key == "l":
        stop_log() if log_writer else start_log()
    elif event.key == "q":
        plt.close(fig)


fig.canvas.mpl_connect("key_press_event", on_key)
ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.show()

stop_log()
send(b"s")             # leave the firmware back in human-readable scan mode
tx.close()
rx.close()
print(f"Done. accepted {n_good}, rejected {n_bad}, out-of-order {n_oos}")
