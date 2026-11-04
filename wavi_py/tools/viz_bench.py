"""Stress benchmark of the view windows: the full GUI on the Sim transport.

Opens the control panel on the Sim transport (16 whiskers, locked 250 Hz
scan, the 'dome' signal preset unless ``--preset``), opens the chosen
windows, runs for ``--secs`` after ``--warmup`` and prints one JSON line:

- per window: frames painted per second (its busiest plot widget), the
  Python-side redraw (setData & co.) and the paint (Qt rasterising the scene)
  per call, the worst redraw + paint, and their share of wall and CPU time;
- the main tick's cost broken down (bus read + despike + ring append,
  the Line copy, the spectra);
- the GUI thread's and the process's CPU share, and the longest event-loop
  stall (how late a 4 ms timer fired): what the user sees as a hitch.

Nothing in the views is changed: the bench wraps their redraw / paint methods.

    python -m wavi_py.tools.viz_bench --windows line,spectrum,fft,spec,imu,dome \
        [--layout grid|stacked|single] [--secs 14] [--warmup 4] [--tag NAME] \
        [--preset "all live"] [--dome-fps 60] [--opengl] [--shots DIR] [--profile FILE]

Windows: line, spectrum, fft, spec, imu, dome, rov. The Dome and ROV views
need a GL context (on the cluster: Xvfb + Mesa llvmpipe, xrun.sh). The Sim
transport has no IMU, so with ``imu`` the bench feeds the IMU window the way the
control panel does for the Server transport: 100 Hz rows, 30 s kept, every tick.

``--replay FILE.jsonl [--speed 4|max] [--start S]`` runs a recording instead
(the real IMU, MAVLink and status included) and adds a ``replay`` record: the
playhead's speed over the wall clock, the whisker / IMU rows reaching the
buses per second, and how far the newest plotted sample lags the playhead.
``--opengl`` turns on pyqtgraph's OpenGL viewports (the app: WAVI_PG_OPENGL=1).
"""
from __future__ import annotations

import argparse
import collections
import cProfile
import json
import os
import pstats
import resource
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from PyQt6.QtCore import QAbstractEventDispatcher, QTimer, Qt  # noqa: E402
from PyQt6.QtWidgets import QApplication  # noqa: E402

import pyqtgraph as pg  # noqa: E402

WINDOWS = ("line", "spectrum", "fft", "spec", "imu", "dome", "rov")


class Meter:
    """Calls / time / worst of one kind of work, since the last ``reset``."""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.n = 0
        self.s = 0.0
        self.cpu = 0.0
        self.worst = 0.0

    def add(self, dt: float, cpu: float = 0.0) -> None:
        self.n += 1
        self.s += dt
        self.cpu += cpu
        self.worst = max(self.worst, dt)


METERS: dict[str, Meter] = collections.defaultdict(Meter)
_owner: dict[int, str] = {}          # id(top-level window) -> name


def _timed(cls, name: str, key):
    """Wrap ``cls.name`` so each call adds to METERS[key(self)] (None: not counted)."""
    orig = getattr(cls, name)

    def wrapper(self, *a, **kw):
        t0, c0 = time.perf_counter(), time.thread_time()
        try:
            return orig(self, *a, **kw)
        finally:
            k = key(self)
            if k is not None:
                METERS[k].add(time.perf_counter() - t0, time.thread_time() - c0)
    setattr(cls, name, wrapper)


def _win(self) -> str | None:
    return _owner.get(id(self.window()))


def _key(self, kind: str) -> str | None:
    w = _win(self)
    return None if w is None else f"{w}.{kind}"


def instrument() -> None:
    from wavi_py.main_window import MainWindow
    from wavi_py.views.fft_view import FftView
    from wavi_py.views.imu_view import ImuView
    from wavi_py.views.line_view import LineView
    from wavi_py.views.spectrogram_view import SpectrogramView
    from wavi_py.views.whisker_plots import WhiskerPlotsView

    # paint: every pyqtgraph view (PlotWidget, GraphicsLayoutWidget), by its window;
    # frames: the paints of the window's busiest widget (FFT has two)
    _timed(pg.GraphicsView, "paintEvent", lambda s: _key(s, "paint"))
    _timed(pg.GraphicsView, "paintEvent", lambda s: _key(s, f"frames.{id(s)}"))
    # render: the views' Python-side redraw
    _timed(WhiskerPlotsView, "_render", lambda s: _key(s, "render"))
    _timed(FftView, "_redraw", lambda s: _key(s, "render"))
    _timed(SpectrogramView, "_redraw", lambda s: _key(s, "render"))
    # (before the render scheduler, update_view drew at once)
    _timed(ImuView, "_render" if hasattr(ImuView, "_render") else "update_view",
           lambda s: _key(s, "render"))
    try:                                   # the Dome view needs PyOpenGL
        from wavi_py.views.dome_view import DomeView, _GLView
        _timed(_GLView, "paintGL", lambda s: _key(s, "paint"))
        _timed(DomeView, "_render_pose", lambda s: _key(s, "render"))
        from wavi_py.views.rov_view import RovView
        _timed(RovView, "_render", lambda s: _key(s, "render"))
    except Exception:
        pass
    # the main tick and its parts
    _timed(MainWindow, "_on_tick", lambda s: "tick.total")
    _timed(MainWindow, "_ingest", lambda s: "tick.ingest")
    _timed(MainWindow, "_update_spectrum", lambda s: "tick.spectrum")
    _timed(MainWindow, "_update_fft_and_spec", lambda s: "tick.fft_spec")
    _timed(LineView, "update_view", lambda s: "tick.line_copy")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--windows", default="line", help=f"comma list of {','.join(WINDOWS)}, or none")
    ap.add_argument("--layout", default="grid", choices=("grid", "stacked", "single"))
    ap.add_argument("--secs", type=float, default=14.0)
    ap.add_argument("--warmup", type=float, default=4.0)
    ap.add_argument("--tag", default="")
    ap.add_argument("--shots", default="", help="directory for window screenshots at the end")
    ap.add_argument("--profile", default="", help="cProfile the GUI thread after warm-up, to FILE")
    ap.add_argument("--dome-fps", type=int, default=60)
    ap.add_argument("--fs-mode", type=int, default=2, help="ESP32 locked scan mode (2: 250 Hz)")
    ap.add_argument("--beat-ms", type=int, default=4, help="stall probe period (0: off)")
    ap.add_argument("--switch", type=float, default=0.0, help="sys.setswitchinterval (s), 0: default")
    ap.add_argument("--opengl", action="store_true", help="pyqtgraph useOpenGL (GL viewports)")
    ap.add_argument("--preset", default="dome", help="Sim signal preset (dome, all live, ...)")
    ap.add_argument("--replay", default="", metavar="FILE.jsonl",
                    help="replay this recording instead of the Sim transport (real IMU)")
    ap.add_argument("--speed", default="1", help="replay speed (0.25 .. 8, or max)")
    ap.add_argument("--start", type=float, default=0.0, help="replay: start here (s)")
    ap.add_argument("--full-span", action="store_true",
                    help="Line: zoomed out to the whole buffer (replay: the recording)")
    ap.add_argument("--t-buffer", type=float, default=0.0,
                    help="live plot buffer seconds (default: the app's, DEFAULT_T_BUFFER_S)")
    args = ap.parse_args()
    wins = [w for w in args.windows.split(",") if w and w != "none"]
    for w in wins:
        if w not in WINDOWS:
            ap.error(f"unknown window {w}")

    if args.switch:
        sys.setswitchinterval(args.switch)
    if args.opengl:
        pg.setConfigOptions(useOpenGL=True)
    # one GL context family: pyqtgraph compiles each shader once, so the Dome and ROV
    # windows (and GL plot viewports) must share contexts; set before the app exists
    QApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
    app = QApplication(sys.argv[:1])
    instrument()
    from wavi_py.main_window import MainWindow

    speed = None if args.speed == "max" else float(args.speed)
    buf = {"t_buffer_s": args.t_buffer} if args.t_buffer else {}
    if args.replay:
        mw = MainWindow(transport="replay", nsensor=16, replay_path=args.replay,
                        replay_speed=speed, replay_start_s=args.start, **buf)
        mw.show()
        mw.open_replay(args.replay)
        t_end = time.monotonic() + 60
        while not mw._replay.active and time.monotonic() < t_end:
            app.processEvents()
            time.sleep(0.005)
        if not mw._replay.active:
            raise SystemExit(f"cannot open {args.replay}")
        mw._replay.set_speed(speed)
        mw._replay.play()
    else:
        mw = MainWindow(transport="sim", nsensor=16, fs=62.5, auto_fs=True,
                        scan_mode=(args.fs_mode,), selects="0123457", **buf)
        mw.show()
        mw._sim.preset(args.preset)
        mw._transport_combo.setCurrentText("Sim")
        mw._connect()

    views = {"line": mw._line_view, "spectrum": mw._spectrum_view, "fft": mw._fft_view,
             "spec": mw._spec_view, "imu": mw._imu_view}
    toggles = {"line": mw._toggle_line, "spectrum": mw._toggle_spectrum, "fft": mw._toggle_fft,
               "spec": mw._toggle_spec, "imu": mw._toggle_imu, "dome": mw._toggle_dome,
               "rov": mw._toggle_rov}
    sizes = {"line": (1400, 900), "spectrum": (1000, 700), "fft": (1000, 760),
             "spec": (1000, 700), "imu": (900, 700), "dome": (1280, 800), "rov": (900, 700)}
    for w in wins:
        toggles[w].setChecked(True)
        if w == "dome":
            views["dome"] = mw._dome_view
            mw._dome_view._fps_box.setValue(args.dome_fps)
        if w == "rov":
            views["rov"] = mw._rov_view
        views[w].resize(*sizes[w])
    for name, v in views.items():
        _owner[id(v)] = name
    if "line" in wins:
        lv = mw._line_view
        if args.layout == "stacked":
            lv._act_grid.setChecked(False)
        elif args.layout == "single":
            lv._maximize(0)
        if args.full_span:
            QTimer.singleShot(int(args.warmup * 500), lambda: lv._zoom_x(1e6))

    # IMU: what the control panel does for the Server transport, 100 Hz, every tick
    imu_rows = collections.deque(maxlen=int(mw._t_buffer_s * 100))
    imu_state = np.zeros(9)

    def feed_imu() -> None:
        nonlocal imu_state
        for _ in range(5):
            imu_state = imu_state + np.random.normal(0, [5, 5, 5, 2, 2, 2, .002, .002, .002])
            imu_rows.append(imu_state.astype(np.float32))
        rows = np.array(imu_rows)
        n = len(rows)
        # on the whisker time axis, so the IMU window linked to the Line window shows them
        end = float(mw._sig_t[-1]) if np.isfinite(mw._sig_t[-1]) else 0.0
        if not mw._is_paused and mw._imu_view.isVisible():
            mw._imu_view.update_view(end + np.arange(-n + 1, 1) / 100.0, rows)

    if "imu" in wins and not args.replay:
        imu_timer = QTimer()
        imu_timer.timeout.connect(feed_imu)
        imu_timer.start(50)

    # event-loop stall: how late a 4 ms timer fires
    stall = {"last": time.perf_counter(), "worst": 0.0, "sum_late": 0.0, "n": 0}

    def beat() -> None:
        now = time.perf_counter()
        late = now - stall["last"] - args.beat_ms / 1000
        stall["last"] = now
        stall["worst"] = max(stall["worst"], late)
        stall["n"] += 1
        if late > 0.0:
            stall["sum_late"] += late
    beat_timer = QTimer()
    beat_timer.setTimerType(Qt.TimerType.PreciseTimer)
    beat_timer.timeout.connect(beat)
    if args.beat_ms:
        beat_timer.start(args.beat_ms)

    # event-loop idle: time between the dispatcher blocking and waking up
    idle = {"t": None, "s": 0.0}

    def blocking() -> None:
        idle["t"] = time.perf_counter()

    def awake() -> None:
        if idle["t"] is not None:
            idle["s"] += time.perf_counter() - idle["t"]
            idle["t"] = None
    disp = QAbstractEventDispatcher.instance()
    disp.aboutToBlock.connect(blocking)
    disp.awake.connect(awake)

    prof = cProfile.Profile() if args.profile else None
    mark: dict = {}

    def start_measuring() -> None:
        for m in METERS.values():
            m.reset()
        stall.update(worst=0.0, sum_late=0.0, n=0, last=time.perf_counter())
        idle["s"] = 0.0
        mark.update(t=time.perf_counter(), cpu=time.process_time(), main=time.thread_time())
        if args.replay:
            mw._replay.poll()
            mark.update(pos=mw._replay.position, n_bus=mw._client.bus.total_published,
                        n_imu=mw._client.imu_bus.total_published)
        if prof:
            prof.enable()

    def finish() -> None:
        if prof:
            prof.disable()
        dt = time.perf_counter() - mark["t"]
        res = {"tag": args.tag, "windows": wins, "layout": args.layout, "preset": args.preset,
               "line_span_s": round(float(np.subtract(*mw._line_view._visible_x_range()[::-1])), 1),
               "fs": mw._fs,
               "secs": round(dt, 2),
               "gui_cpu_pct": round(100 * (time.thread_time() - mark["main"]) / dt, 1),
               "proc_cpu_pct": round(100 * (time.process_time() - mark["cpu"]) / dt, 1),
               "idle_pct": round(100 * idle["s"] / dt, 1),
               "stall_worst_ms": round(1e3 * stall["worst"], 1),
               "stall_late_pct": round(100 * stall["sum_late"] / dt, 1),
               "t_buffer_s": mw._t_buffer_s,
               "maxrss_mb": round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024, 1)}
        per = {}
        for w in wins:
            r, p = METERS[w + ".render"], METERS[w + ".paint"]
            frames = max([m.n for k, m in METERS.items() if k.startswith(w + ".frames.")] or [p.n])
            per[w] = {"fps": round(frames / dt, 1), "renders_per_s": round(r.n / dt, 1),
                      "render_ms": round(1e3 * r.s / max(r.n, 1), 2),
                      "paint_ms": round(1e3 * p.s / max(p.n, 1), 2),
                      "worst_ms": round(1e3 * (r.worst + p.worst), 1),
                      "wall_pct": round(100 * (r.s + p.s) / dt, 1),
                      "cpu_pct": round(100 * (r.cpu + p.cpu) / dt, 1)}
        res["views"] = per
        if args.replay:
            # the replay kept up: playhead speed (wall), what reached the buses, the lag of
            # the newest plotted sample behind the playhead
            mw._replay.poll()
            c = mw._client
            res["replay"] = {
                "speed_set": args.speed,
                "speed_got": round((mw._replay.position - mark["pos"]) / dt, 2),
                "position": round(mw._replay.position, 1),
                "wsk_per_s": round((c.bus.total_published - mark["n_bus"]) / dt, 1),
                "imu_per_s": round((c.imu_bus.total_published - mark["n_imu"]) / dt, 1),
                "lag_s": round(mw._replay.position - float(mw._sig_t[-1]), 3)}
        res["tick"] = {k.split(".", 1)[1]: {"per_s": round(m.n / dt, 1),
                                           "ms": round(1e3 * m.s / max(m.n, 1), 3),
                                           "cpu_pct": round(100 * m.s / dt, 2)}
                       for k, m in sorted(METERS.items()) if k.startswith("tick.")}
        if args.shots:
            os.makedirs(args.shots, exist_ok=True)
            for w in wins:
                views[w].grab().save(f"{args.shots}/{args.tag or 'bench'}_{w}.png")
        if prof:
            prof.dump_stats(args.profile)
            st = pstats.Stats(prof, stream=sys.stderr)
            st.sort_stats("tottime").print_stats(35)
            st.sort_stats("cumulative").print_stats(45)
        print(json.dumps(res), flush=True)
        app.quit()

    QTimer.singleShot(int(args.warmup * 1000), start_measuring)
    QTimer.singleShot(int((args.warmup + args.secs) * 1000), finish)
    app.exec()
    os._exit(0)


if __name__ == "__main__":
    main()
