# Whisker array — how to run it

MSD Seal Whisker Project · RIT · Flow Physics and Modeling Lab
Updated 5 Oct 2026 — rewritten for the BlueROV2 tether setup.

---

## How the pieces connect

```
whisker PCB + ESP32          Raspberry Pi (BlueOS)            your Mac
  32 gauges -> ADC   --USB-->  /dev/ttyUSB0                     en6
                               BlueOS "bridge"  --Fathom tether (Ethernet)-->
                               UDP 27000 (data) / 27001 (commands)
```

**The tether is Ethernet, not USB.** There is no `/dev/cu.usbserial-*` on your
Mac any more and there never will be while the ESP32 is inside the ROV. BlueOS
forwards the serial port over UDP and the Python scripts read that.

| | |
|---|---|
| Your Mac (topside) | `192.168.2.1` on `en6` |
| BlueOS computer | `192.168.2.2` |
| Data: ROV → Mac | UDP `27000` |
| Commands: Mac → ROV | UDP `27001` |

---

## Do I need the Arduino IDE?

**No — not to run anything.** The Python scripts talk to the board by
themselves: they put the firmware into stream mode, read it, and send it
commands. You never need to open the IDE or the Serial Monitor to collect data.

The Arduino IDE is only for **changing the firmware** (`whisker_reader.ino`),
and even then you cannot flash over the tether — the upload needs DTR/RTS
auto-reset, which UDP does not carry. To reflash you must either:

- unplug the ESP32 and connect it to your Mac by USB, or
- `ssh` into the BlueOS computer and flash it there with `esptool`.

Running the commands below from the VS Code terminal is sufficient for
everything else.

> The old advice *"close the Arduino Serial Monitor first"* no longer applies —
> nothing on your Mac holds a serial port. The modern equivalent is that only
> one program can bind UDP 27000, so **`whisker_plot.py` and `whisker_scope.py`
> cannot run at the same time.**

---

## One-time setup

**1. Check Python exists at that exact path**

```bash
ls /opt/homebrew/bin/python3.11
```

**2. Install the packages into that exact Python**

```bash
/opt/homebrew/bin/python3.11 -m pip install matplotlib numpy
```

`pyserial` is no longer needed — both scripts use UDP now. Installing it does
no harm if it is already there.

**3. Verify**

```bash
/opt/homebrew/bin/python3.11 -c "import matplotlib, numpy; print('both OK')"
```

The key point: steps 2 and 3 both start with `/opt/homebrew/bin/python3.11`.
Never type bare `python3` or `pip3` — those pick a different Python.

---

## Every session

**1. Check the bridge exists**

```bash
curl -s http://192.168.2.2/bridget/v1.0/bridges
```

If that prints `[]`, create it — **the bridge does not survive a BlueOS
reboot:**

```bash
curl -X POST http://192.168.2.2/bridget/v1.0/bridges -H 'Content-Type: application/json' -d '{"serial_path":"/dev/ttyUSB0","baud":115200,"ip":"192.168.2.1","udp_target_port":27000,"udp_listen_port":27001}'
```

**2. Go to the folder**

```bash
cd ~/Library/CloudStorage/OneDrive-rit.edu/Desktop/"Seal Whisker Project"/"MSD Project"/whisker_reader
```

**3. Run one of the two programs**

```bash
/opt/homebrew/bin/python3.11 whisker_plot.py --log
```

---

## The two programs

### `whisker_plot.py` — 4×4 grid, all 16 whiskers

Best for watching the whole array and for logging runs.

```bash
/opt/homebrew/bin/python3.11 whisker_plot.py          # just watch
/opt/homebrew/bin/python3.11 whisker_plot.py --log    # record from the start
```

Averages each channel's first 40 scans and subtracts that, so every trace
starts at 0 and the panels show **change**, not absolute level.

| key | does |
|---|---|
| `z` | re-zero now (hold whiskers still) |
| `b` | ask the firmware to redo its own baseline |
| `a` | shared y-axis ↔ per-whisker autoscale |
| `l` | start/stop CSV logging |
| `q` | quit |

### `whisker_scope.py` — lab-scope view

Best for looking closely at one or two whiskers. Has drift tracking and a
focus mode.

```bash
/opt/homebrew/bin/python3.11 whisker_scope.py
/opt/homebrew/bin/python3.11 whisker_scope.py --focus 3        # zoom whisker 3
/opt/homebrew/bin/python3.11 whisker_scope.py --window 60      # 60 s history
/opt/homebrew/bin/python3.11 whisker_scope.py --only W3B,W4B,W8B
```

`--only` tells it which gauges are physically connected; everything else is
greyed out and labelled `(n/c)`. Worth using, because an unconnected input can
float mid-range and show mains pickup that looks like a real signal.

| key | does |
|---|---|
| `f` | grid ↔ focus |
| `←` `→` / `1`–`9` `0` | pick whisker (focus mode) |
| `z` | software re-zero |
| `b` | firmware re-baseline |
| `d` | drift tracking on/off |
| `a` | cycle y-axis mode |
| `[` `]` | shrink / grow y range |
| `+` `-` | longer / shorter time window |
| `space` | pause |
| `r` | start/stop CSV recording |
| `s` | save PNG |
| `q` | quit |

Both default to `--ip 192.168.2.2 --rx-port 27000 --tx-port 27001`.

---

## CSV format

`whisker_plot.py` writes `whisker_log_<timestamp>.csv`:

```
# zero_mV , <32 rest values, same column order as below>
scan , W1_A , W1_B , W2_A , W2_B , ... , W16_A , W16_B
```

`whisker_scope.py` writes `whisker_rec_<timestamp>.csv` with `time_s` instead
of `scan` and the same 32 gauge columns.

Values are **mV deviation from rest**. Unconnected gauges log `0`. The
`# zero_mV` row keeps the offsets that were subtracted, so absolute firmware
readings stay recoverable.

Column order follows the firmware: `OUT(2W-1)` is whisker *W* gauge A,
`OUT(2W)` is gauge B — so the stream is already grouped per whisker.

---

## Troubleshooting

**Nothing on screen, `accepted 0`, no data at all.**
Work outward from your Mac:

```bash
ping -c 2 192.168.2.2                                   # tether up?
curl -s http://192.168.2.2/bridget/v1.0/bridges         # bridge there?
curl -s http://192.168.2.2/bridget/v1.0/serial_ports    # ttyUSB0 listed?
```

If all three are fine and it is still silent, the ESP32 itself has stopped
talking. **Press its EN/reset button.** This happened on 5 Oct 2026 after the
board had been silent for days, and the reset fixed it immediately.

Note that `/dev/ttyUSB0` appearing in the list only proves the USB-serial chip
has power. It says nothing about whether the ESP32 is running.

**`Cannot bind UDP 27000`.**
The other script, or a second copy of the same one, is already running. Close
it. Only one program can hold the port.

**Every value arrives twice / lines look spliced together.**
A bridge can end up with a doubled sender. Delete and recreate it:

```bash
curl -X DELETE http://192.168.2.2/bridget/v1.0/bridges -H 'Content-Type: application/json' -d '{"serial_path":"/dev/ttyUSB0","baud":115200,"ip":"192.168.2.1","udp_target_port":27000,"udp_listen_port":27001}'
```

then POST it again. Both scripts already avoid the usual cause by sending
commands from a different socket than they listen on.

**A channel sits on a huge constant offset.**
W4_B once idled at −1189 mV and an ESP32 reset cleared it, so try that before
assuming hardware. The host-side zero removes the offset either way.

---

## Hardware reference

### P1 — ADC / SPI (to ADS131M04 U4)

| P1 | Signal | ESP32 | U4 pin |
|---|---|---|---|
| 1 | SCLK | GPIO18 | 14 |
| 2 | DRDY | GPIO4 | 13 |
| 3 | CS | GPIO21 | 12 |
| 4 | DOUT / MISO | GPIO19 | 15 |
| 5 | DIN / MOSI | GPIO23 | 16 |
| 6 | CLKIN | GPIO16 | 17 |

### P2 — MUX selects (CD4097)

| P2 | Signal | ESP32 |
|---|---|---|
| 1 | SELC_1 | GPIO27 |
| 2 | SELB_1 | GPIO26 |
| 3 | SELA_1 | GPIO25 |
| 4 | SELA_2 | GPIO32 |
| 5 | SELB_2 | GPIO33 |
| 6 | SELC_2 | GPIO14 |

### Known channel notes

- **No signal from OUT 4 (CONN7/CONN8) and OUT 10 (CONN19/CONN20)** — carried
  over from the original guide. In the new column naming these are **W2_B** and
  **W5_B**. Not re-confirmed since; no whisker has been attached to either.
- **W3 and W8** were shorted through Aug 2026 and reworked with Prof. Barrios
  on 26 Aug. Both read normally as of 5 Oct.
- Only **W3_B, W4_B and W8_B** have had whiskers attached in recent sessions.
  Every other channel reads `open` for want of a connection, not necessarily
  for want of a working channel.
