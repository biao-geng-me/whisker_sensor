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

This adds an **IMU** view, and **Rec** records on the vehicle as well as locally (`.dat` + full `.jsonl`).

Against the ESP32 whisker DAQ ([`msd/esp32/whisker_daq`](../msd/esp32/whisker_daq/)) plugged directly into the PC by USB, for bench work without the ROV:

```bash
python -m wavi_py --transport esp32 --serial-port COM5
```

wavi_py decodes the ESP32's binary frames itself (921600 baud, sends `B` to start streaming), so everything behaves as with `--transport server` except that there is no IMU and **Rec** writes only the local `.dat` + `.jsonl`. The status line shows the scan rate and the same error counters. Opening the port does not reset the ESP32.

Against an Arduino plugged directly into the PC:

```bash
python -m wavi_py --transport serial --serial-port COM3 --nsensor 9 --fs 80
```

Click **Connect**. Toggle **Line / FFT / Spec** to bring up plot windows. **Pause** freezes plot updates while data keeps flowing (and recording continues). **Rec** writes a `.dat` file using the exact MATLAB filename / line format, so existing tools under [`data_processing/`](../data_processing/) keep working unchanged.

The **Line** window has its own **Pause** and **Re-zero** buttons, which are the control panel's Pause and Reset (re-capture the V0 baseline) and stay in sync with them; **Reset view** only resets zoom and live following. **Grid** shows one tile per whisker; double-click a tile (or, in the stacked layout, near a whisker's trace) to show that whisker alone, and double-click again, press Esc or click **All whiskers** to go back.

Keys in the Line window (whichever plot has focus): **Space** pause / resume, **Z** zoom box on / off (off = drag pans), **+** (or **=**, no Shift needed) / **-** zoom in / out by 1.25 about the view centre (keypad keys too), **R** reset view, **0** re-zero, **Esc** back to all whiskers. The right side of its status bar shows the same stream status line as the control panel (scan rate, error counters), also while recording and when the control panel is hidden.

Each plot also shows the current absolute value of both gauges (raw mV at the ADC, before the V0 subtraction and the `--scale`; mean of the last 0.1 s, refreshed ~7 times a second) in its corner, or on the right of each row in the stacked layout; a railed gauge (nothing plugged in) reads **open**. With the TCP / Serial (Arduino) transports the values are shown without a unit and never as open. Curves are min/max decimated to about two points per pixel column and drawn with 1 px pens, so the redraw cost does not grow with the sample rate.

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
  views/
    line_view.py       multi-channel offset plot
    fft_view.py        FFT heatmap + per-channel bar
    spectrogram_view.py
                       frequency-stacked-by-channel ImageItem
  io/
    dat_writer.py      writes the exact MATLAB-compatible .dat format
  examples/
    headless_control_loop.py
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
