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

Against an Arduino plugged directly into the PC:

```bash
python -m wavi_py --transport serial --serial-port COM3 --nsensor 9 --fs 80
```

Click **Connect**. Toggle **Line / FFT / Spec** to bring up plot windows. **Pause** freezes plot updates while data keeps flowing (and recording continues). **Rec** writes a `.dat` file using the exact MATLAB filename / line format, so existing tools under [`data_processing/`](../data_processing/) keep working unchanged.

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
