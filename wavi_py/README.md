# wavi_py

Python port of [`wavi/wavi.m`](../wavi/wavi.m) — the whisker-sensor DAQ client. Same wire format, same `.dat` recording format, same three views (line / FFT / spectrogram). No MATLAB dependency.

The architectural difference from the MATLAB version: **data acquisition is decoupled from visualization** via a thread-safe `SampleBus`. The GUI is one subscriber; an agent control loop on another thread is another. Slow GUI redraws can never starve a tight control loop, and vice versa.

## Install

```bash
pip install -r wavi_py/requirements.txt
```

Dependencies: PyQt6, pyqtgraph, numpy, scipy, pyserial.

## Run (GUI)

Against the Pi bridge (`rpi/daq_bridge.py` running on the Pi):

```bash
python -m wavi_py --transport tcp --host 192.168.2.3 --port 5555 --nsensor 9 --fs 80
```

Against the BlueROV2 whisker array + IMU (`rpi/daq_server.py` on the vehicle; see [rpi/README.md](../rpi/README.md#bluerov2-whisker-array--imu-server-daq_serverpy)):

```bash
python -m wavi_py --transport server --host 192.168.2.2 --scale 0.2
```

This adds an **IMU** view (accel, gyro and attitude on the IMU's own time axis, over the same span as the whisker plots and independent of the whisker scan rate, with the stream status line and the alarm banner), and **Rec** records on the vehicle as well as locally (`.dat` + full `.jsonl`).

Against the ESP32 whisker DAQ ([`msd/esp32/whisker_daq`](../msd/esp32/whisker_daq/)) plugged directly into the PC by USB, for bench work without the ROV:

```bash
python -m wavi_py --transport esp32 --serial-port COM5
```

wavi_py decodes the ESP32's binary frames itself (921600 baud, sends `B` to start streaming), so everything behaves as with `--transport server` except that there is no IMU and **Rec** writes only the local `.dat` + `.jsonl`. The status line shows the scan rate and the same error counters. wavi_py deasserts DTR / RTS before opening the port so as not to reset the ESP32, but if it does reset (some USB-UART drivers toggle DTR anyway, or a watchdog) it comes back in its boot default, quiet 62.5 Hz, OSR 512 with all 8 selects; see **Scan mode** below.

**Scan mode** (second row of the Sensor array box; Server, ESP32 and Sim transports) switches the ESP32's scan mode without a reflash: **legacy** (`M0;`; its rate depends on the selects, ~89 Hz with 7), **quiet 62.5 Hz, OSR 512** (`M1;`, the firmware's boot default), **quiet 62.5 Hz, OSR 256** (`M1 200 256;`), **quiet 125 Hz, OSR 256** (`M1 200 256 8000;`) and **wide 250 Hz, OSR 128** (`M2;`); see [Locked scan modes](../msd/esp32/README.md#locked-scan-modes-quiet-625-hz-wide-250-hz-2026-10-22). Picking one sends `S<selects>;M<args>;B` through the `esp32` command (with `--transport server`, daq_server passes it on to the ESP32); **Selects** (default `0123457`, empty = all 8) goes with it. **as is** (the default) sends nothing. Next to it, and at the end of the status line, is what the ESP32's INFO frame reports, not what was asked for: "quiet 125 Hz OSR 256 confirmed", or in red "NOT SET ..." with the reason when no INFO matches within 2 s (no reply, or the firmware refused it, e.g. `M: scan period not a whole number of conversions at this OSR`). The ESP32 boots in quiet 62.5 Hz, OSR 512 (core C1) with all 8 selects and forgets any mode setting on reboot, so when an INFO later reports another mode or other selects, the choice is re-sent once and the note says what the ESP32 came back in ("re-applied HH:MM:SS after the ESP32 came back in quiet 62.5 Hz OSR 512 selects 01234567"); if that doesn't stick either, it is reported and not sent again until you pick a mode (picking the same one again retries). `--mode` sets the startup choice, with the same spelling as daq_server's `--mode` (`legacy`, `quiet`, `wide`, `quiet,200,256`, `quiet,200,256,8000`), and `--selects` the selects; on connect it is sent only if the ESP32 isn't already in that mode. A daq_server started with its own `--mode` re-sends that one on every stream start; the GUI then re-applies its own choice once, as after a reboot.

**ADC fault alarm** (Server, ESP32 and Sim transports). On the ROV the whisker PCB runs from vehicle power and its ADC can lock up so that only a PCB power cycle clears it. The fault model (`whisker_link/esp32_fault.py`, run by daq_server or by the ESP32 / Sim transport) turns the firmware's signals into a state in the status record, and wavi_py shows it as a banner on top of the panel and of every plot window. The banner does not block plotting. **Red (fault, latched):** `[halt #N] UNRECOVERED`, CLOCK config not accepted (at boot, on `M`, after an ADC reset), ADC not answering, repeated `[halt] no conversion` in a locked mode with no scans for 1.5 s, or no scans for 5 s while connected. It says what to do: "Power-cycle the whisker PCB (vehicle power)"; for no scans, check the ESP32's USB link. A later INFO doesn't clear it. It clears only after 10 s of clean streaming: after a PCB power cycle with the ESP32 running, the firmware recovers by itself. **Amber (warning, for 60 s):** recovered halts, ADC resets, reader-watchdog and other unexpected ESP32 reboots, `[idf]` errors, and more than 1 % of scans flagged SYNC/EMPTY over 10 s. **Fault log** (Status box, counts the events, red or amber while active) lists every onset, warning and clear of the session. While recording, each event also goes into `<recording>.faults.txt` next to the `.dat`, with its wall time and the `.dat` row it falls on; a fault already active at record start is written first. The `.jsonl` gets it as a `mark` event (`text` + `fault`), which `whisker_link.convert` lists under `mark_t` / `mark_text`. The status line starts with `FAULT <code>` while a fault is active. With firmware 2e9d8dc or later the status line also ends with the ESP32's 1 Hz heartbeat (`esp32.beat`) as `ADC 7812 conv/s` (DRDY edges per second: 7812 in quiet OSR 512, 31250 in wide, 3125 in legacy), in red as `ADC NOT converting` when the ADC stops; after a reboot it shows `ESP32 rebooted HH:MM:SS (reset: <reason>)` for 30 s (`booted` if the ESP32 came up less than 30 s before connecting), and the Fault log gets that line once.

After a switch the rate follows the INFO: scan_hz for the locked modes, and for legacy the rate measured earlier for the same selects, else an estimate from the select count (1.6 ms per select), replaced by the measured rate ~2 s later. The rolling buffers start over at the sample where the INFO arrived, so no plot or FFT / Spec window mixes two rates, and the time axis and the `.dat` timestamps continue from there at the new rate (a measured-rate correction is now applied while recording too, for the same reason). With `--fs` given, the rate stays fixed. The `.dat` timestamps count samples, so a stretch with no scans (a reboot, a dropout) takes no time in them.

Without any hardware, against a simulated 16-whisker array (any OS; no server, no serial port):

```bash
python -m wavi_py --transport sim
```

or pick **Sim** in the transport list. It sends the same event stream as the ESP32, so the plots, V0 / Re-zero, the measured rate, the status line and **Rec** (`.dat` + `.jsonl`) all work as with the real array; there is no IMU. **Signals** (next to Connect) opens a panel that changes the signals live, also before connecting: per gauge live or open (railed), a sine (amplitude, frequency) and white noise (RMS; **Noise (all)** sets every gauge at once); scan rate in the legacy scan mode (40 to 1000 Hz; the GUI adopts a new rate in one step), drift, and mains hum (amplitude at 50 or 60 Hz, picked up by each gauge with its own fixed coupling 0.3 to 1.5; evaluated at the scan times, so below 2× the mains frequency it aliases, e.g. 50 Hz at 80 Hz scans shows at 30 Hz); presets (demo, all live at 1 to 16 Hz by whisker, all live at 2 Hz, all open, noise floor with every gauge live at 0.05 to 0.5 mV of noise, noisy gauges, hum 50 Hz, hum 60 Hz, and bench: small sines, spread noise, 1 mV of 50 Hz hum and a slow drift; hover the preset list for details); taps, a decaying ring on one whisker (by hand or every few seconds; "any" picks a whisker with a live gauge); and faults (ADC recovered / CRC retry / stale / re-init, a sequence gap, a link CRC error, a 3 s dropout) that show up in the status line's counters, **Reboot ESP32** (like the firmware it comes back in quiet 62.5 Hz with all selects ~2 s later, so Scan mode re-applies its choice; the array also starts in that mode, so the scan-rate setting applies only after picking legacy), and **ADC lock-up** / **Power-cycle PCB**. The lock-up stops the scans and makes the firmware report `[halt #N] UNRECOVERED` every 1.5 s in legacy, or five `[halt] no conversion` and then `[halt #N] UNRECOVERED` every 5 s in a locked mode; a reboot meanwhile shows CLOCK not accepted. The Sim also sends the firmware's 1 Hz heartbeat (0 conversions/s while locked up). The power cycle recovers it, as with the real board. Use these to see the ADC fault alarm. The Sim transport also takes the Scan mode commands like the firmware (`S`, `M` checked as the firmware does, `B`): the locked modes run at 62.5 / 125 / 250 Hz with a HEALTH record, and the outputs of skipped selects read open. Switching a gauge from open to live mid-stream puts a step in its trace until **Re-zero**, as plugging in a real gauge would. Scripts can use it too: `WaviClient(transport="sim", nsensor=16)` and edit `client.sim` (a `SimSignals`: `preset()`, `set_channel()`, `tap()`, `fault()`, `rate_hz`, `noise_mv` (per gauge), `drift_mv_s`, `hum_mv`, `hum_hz`).

Against an Arduino plugged directly into the PC:

```bash
python -m wavi_py --transport serial --serial-port COM3 --nsensor 9 --fs 80
```

Click **Connect**. Toggle **Line / FFT / Spec** to bring up plot windows. **Pause** freezes plot updates while data keeps flowing (and recording continues). **Rec** writes a `.dat` file using the exact MATLAB filename / line format, so existing tools under [`data_processing/`](../data_processing/) keep working unchanged.

The **Line** window has its own **Pause** and **Re-zero** buttons, which are the control panel's Pause and Reset (re-capture the V0 baseline) and stay in sync with them. It opens in the **Grid** layout (one tile per whisker); untick Grid for the stacked layout (all whiskers on one axis). Double-click a tile (or, in the stacked layout, near a whisker's trace) to show that whisker alone, and double-click again, press Esc or click **All whiskers** to go back.

Zoom and live: time (X) is always shared by all plots; Y is per tile unless **Link Y** gives all grid tiles one Y range. While live, the plots show the newest part of the buffer, and zooming time keeps them live (the right edge stays at now). Panning time or zooming / panning with the mouse leaves live; **Live** (L) goes back to live keeping the zoom (time span, Y ranges, gain), and so does Resume. **Reset view** (R) also restores the whole buffer, auto Y and gain 1. Keys (the **Keys** button lists them): **Space** pause / resume, **L** live, **R** reset view, **=** / **-** zoom time in / out (keypad + / - too), **+** / **_** zoom Y of the framed tile (the one last under the mouse, with an orange frame; all tiles if none), **.** / **,** zoom Y of all tiles, **arrows** move the data (Left slides the traces left, Up slides them up; Left / Right all plots, Up / Down the framed tile), **Y** link Y, **Z** zoom box, **0** re-zero, **Esc** back to all whiskers. In the stacked layout the Y keys change the trace gain (shown on the axis label) instead, so small signals grow while all rows stay in view. The right side of its status bar shows the same stream status line as the control panel (scan rate, error counters), also while recording and when the control panel is hidden; when a message is shown on the left, the status line is shortened from its start.

**Spectrum** opens a live amplitude spectrum per whisker in the same layouts and with the same keys (grid by default, Link Y, framed tile, double-click for one whisker, Pause in sync with the control panel); X is frequency, linear, 0 to fs/2. Each update takes the newest 1 s of every channel (W cycles 0.5 / 1 / 2 / 5 s; the bin spacing is 1 / window), removes the mean, applies a Hann window and scales the rFFT so a sine of amplitude a reads a, in the Line window's units (mV at the ADC times `--scale`). The mean is removed rather than a linear trend: with a 1 s window a 1 Hz sine is a single cycle, and a fitted line would take about 45 % of it. **dB** (D) shows dB re 1 unit, leaving out constant (railed) gauges. Each tile's corner shows the dominant frequency (largest bin above DC) and its amplitude per gauge; in the stacked layout the Y keys set a gain (in dB, one row spans -60 to +20 dB times the gain). It is computed only while the Spectrum window is shown and not paused, at the 20 Hz UI tick.

**FFT** and **Spec** use the same spectra (newest FFT-window seconds, mean removed, Hann window, amplitude-scaled) and share a toolbar: **dB** (D), colour range **Auto** (A; 1st to 99.5th percentile of what is shown) or typed **min** / **max** (typing switches Auto off; linear and dB each keep their own range), **FFT** window length 0.5 / 1 / 2 / 5 s (W; one setting for both windows, so the bins are 1 / window apart), **Reset view** (R), **Pause** (Space, in sync with the control panel), **Keys**; = / - zoom the frequency axis from 0 Hz, the mouse reads channel, frequency and amplitude into the status bar, whose right side shows the stream status line. The **FFT** window shows a heatmap of channel by frequency with a colour bar, and below it, in a closed frame, a per-channel summary of the frequency band set right above it: **Peak bars** (the largest amplitude in the band), **Box over frequency** (the spread of the channel's amplitudes across the band: box 25 to 75 %, line the median, roughly the noise floor, whiskers 5 to 95 %, dot the peak) or **Box over time** (the spread of the band peak over the last *history* seconds; dot the newest frame); M cycles them. Open gauges are left out. The **Spec** window shows time by frequency for all channels stacked (each channel a row band, 0 Hz at its bottom) or one gauge with its frequency axis in Hz ([ and ] step through them), over a 10 / 20 / 60 s span.

Each plot also shows the current absolute value of both gauges (raw mV at the ADC, before the V0 subtraction and the `--scale`; mean of the last 0.1 s, refreshed ~7 times a second) in its corner, or on the right of each row in the stacked layout; a railed gauge (nothing plugged in) reads **open**. With the TCP / Serial (Arduino) transports the values are shown without a unit and never as open. Curves are min/max decimated to about two points per pixel column and drawn with 1 px pens, so the redraw cost does not grow with the sample rate. The window redraws at most ~30 times a second (new data and mouse zoom / pan requests are merged) and not at all while hidden; the FFT and Spec windows likewise skip their redraw while hidden. On the cluster test (80 Hz, 16 whiskers, 1400×900, offscreen) this cut the main-thread load of the live line window from about 55 % to 13 % of a core (stacked), 65 % to 32 % (grid) and 17 % to 10 % (one whisker).

## Run (library / headless)

```python
from wavi_py import WaviClient

client = WaviClient(transport="tcp", host="192.168.2.3", port=5555,
                    nsensor=9, fs=80)
client.start()
client.wait_ready(timeout=10)

last_seq = 0
while running:
    samples, last_seq = client.bus.read_since(last_seq, block=True, timeout=0.1)
    if samples is None:
        continue
    # samples: np.ndarray, shape (N_new, nch), dtype float32, raw post-parse
    action = my_agent.act(samples[-1])
    motor.send(action)

client.stop()
```

Or run the included example:

```bash
python wavi_py/examples/headless_control_loop.py --host 192.168.2.3
```

The GUI and a headless control loop can run **simultaneously** in separate processes — each is just another subscriber on the bus inside its own `WaviClient`.

## Package layout

```
wavi_py/
  app.py             CLI / Qt entry point
  client.py          WaviClient facade (no Qt)
  main_window.py     control panel (PyQt6) — one subscriber
  config.py          defaults, wire-format constants
  scan_mode.py       ESP32 scan modes: presets, --mode, INFO check, re-apply (no Qt)
  (whisker_link/esp32_fault.py: the ADC / ESP32 fault model, shared with daq_server)
  daq/
    bus.py             SampleBus — single-writer/many-readers ring buffer
    reader.py          DAQ thread: bytes → parse → bus.publish
    sig_filter.py      port of wavi/SigFilter.m (scipy.signal.butter + lfilter)
    outliers.py        port of filloutliers('linear')
  transports/
    base.py            abstract Transport
    tcp_transport.py   socket-based, TCP_NODELAY, mirrors connection_manager.py
    serial_transport.py
                       pyserial-based, reuses hx711_array/arduino_reader.py defaults
    esp32_transport.py ESP32 binary frames -> the server's event stream
    sim_transport.py   simulated array (SimSignals) -> the same event stream
  views/
    whisker_plots.py   shared base of the Line / Spectrum windows (layouts, keys)
    line_view.py       per-whisker time series
    spectrum_view.py   per-whisker live amplitude spectrum
    sim_panel.py       signal panel for the sim transport
    fft_view.py        FFT heatmap + per-channel bar
    spectrogram_view.py
                       frequency-stacked-by-channel ImageItem
  io/
    dat_writer.py      writes the exact MATLAB-compatible .dat format
  examples/
    headless_control_loop.py
  tests/
    test_scan_mode.py  scan-mode logic and the Sim transport's firmware emulation
```

## Architecture (one diagram)

```
[Transport]  ──bytes──►  [DAQ thread]  ──samples──►  [SampleBus]  ──┬──►  [Qt UI subscriber]   plot + record
   TCP/serial                parse                    ring buffer    │       (this process)
                             align V0                 cond var       │
                                                                     ├──►  [agent control loop]
                                                                     │       (this process,
                                                                     │        own thread)
                                                                     │
                                                                     └──►  [recorder, ...]
                                                                            (anywhere)
```

Each subscriber tracks its own `last_seq`; slow ones can't backpressure the producer (the bus is bounded, oldest data dropped if a reader falls behind by more than `t_buffer × Fs` samples — detect via `new_seq - last_seq > len(samples)`).

## Verification checklist

1. **GUI vs MATLAB parity**: open the same recording (`.dat`) in both MATLAB ([data_processing/plot_sensor_signal.m](../data_processing/plot_sensor_signal.m)) and Python — outputs should match byte-for-byte.
2. **Decoupling**: run `wavi_py/examples/headless_control_loop.py` and `python -m wavi_py` simultaneously against the same Pi. Neither should affect the other; latencies from `bus.publish` to control-loop's `read_since` return should stay under a few ms even when the GUI is being dragged around.
3. **Disconnect/reconnect**: in the GUI, click Disconnect mid-stream then Connect — clean resume. Headless: `client.stop()` then `client.start()` does the same.
