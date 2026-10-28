"""GUI entry point. Run with ``python -m wavi_py [args]``.

For headless / library usage, import ``WaviClient`` directly — this module
is only the Qt-based front door.
"""

from __future__ import annotations

import argparse
import os
import sys

from wavi_py.config import (
    DEFAULT_FS_HZ,
    DEFAULT_NS_READ,
    DEFAULT_NSENSOR,
    DEFAULT_T_BUFFER_S,
    DEFAULT_T_FFT_S,
    DEFAULT_T_SPEC_S,
    DEFAULT_TCP_HOST,
    DEFAULT_TCP_PORT,
)
from wavi_py.scan_mode import DEFAULT_SELECTS, parse_mode, parse_selects
from whisker_link.channels import NUM_WHISKER
from whisker_link.wire import DEFAULT_PORT as SERVER_PORT


def _arg(parse):
    """An argparse type from a parser that raises ValueError."""
    def f(s: str):
        try:
            return parse(s)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(str(exc)) from None
    f.__name__ = parse.__name__
    return f


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="wavi_py",
        description="Python whisker-sensor DAQ client (GUI).",
    )
    p.add_argument("--transport", choices=["server", "esp32", "sim", "tcp", "serial"], default="tcp",
                   help="server = rpi/daq_server.py (ESP32 whisker array + ROV IMU); "
                        "esp32 = the ESP32 DAQ on a local serial port (no IMU); "
                        "sim = a simulated array with a signal panel (no hardware)")
    p.add_argument("--host", default=DEFAULT_TCP_HOST, help="TCP host (Pi IP or hostname)")
    p.add_argument("--port", type=int, default=None,
                   help=f"TCP port (default {DEFAULT_TCP_PORT}; {SERVER_PORT} for server)")
    p.add_argument("--serial-port", default=None, help="e.g. COM3 or /dev/ttyACM0")
    p.add_argument("--nsensor", type=int, default=None,
                   help=f"default {DEFAULT_NSENSOR}; 16 for server / esp32 / sim")
    p.add_argument("--fs", type=float, default=None,
                   help=f"sample rate Hz (default {DEFAULT_FS_HZ:g}; server / esp32: measured "
                        "from the stream unless given)")
    p.add_argument("--ns-read", type=int, default=DEFAULT_NS_READ,
                   help="samples per DAQ batch")
    p.add_argument("--t-fft", type=float, default=DEFAULT_T_FFT_S, help="FFT window seconds")
    p.add_argument("--t-spec", type=float, default=DEFAULT_T_SPEC_S, help="Spectrogram seconds")
    p.add_argument("--t-buffer", type=float, default=DEFAULT_T_BUFFER_S, help="signal buffer seconds")
    p.add_argument("--scale", type=float, default=1.0, help="line view amplitude scale")
    p.add_argument("--mode", type=_arg(parse_mode), default=None,
                   help="ESP32 scan mode to set on connect and keep (server / esp32 / sim; same "
                        "spelling as daq_server --mode): legacy, quiet (62.5 Hz, OSR 512), wide "
                        "(250 Hz), optionally ,settle_us,osr,period_us,spi_hz, e.g. quiet,200,256 "
                        "or quiet,200,256,8000 (125 Hz); default: leave as is (the firmware boots in "
                        "wide 250 Hz, OSR 128, selects 0123457; before 2026-10-28 quiet "
                        "62.5 Hz, OSR 512, all selects). The Scan mode "
                        "dropdown changes it later")
    p.add_argument("--selects", type=_arg(parse_selects), default=DEFAULT_SELECTS,
                   help=f"mux selects sent with the scan mode (default {DEFAULT_SELECTS}; "
                        "'' = all 8)")
    a = p.parse_args(argv)
    server = a.transport == "server"
    array = a.transport in ("server", "esp32", "sim")
    if a.port is None:
        a.port = SERVER_PORT if server else DEFAULT_TCP_PORT
    if a.nsensor is None:
        a.nsensor = NUM_WHISKER if array else DEFAULT_NSENSOR
    a.auto_fs = a.fs is None
    if a.fs is None:
        a.fs = DEFAULT_FS_HZ      # server: a starting guess until the rate is measured
    return a


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)

    # Qt imports are deferred so help / parsing errors don't pull in PyQt6.
    from PyQt6.QtWidgets import QApplication

    from wavi_py.main_window import MainWindow

    if os.environ.get("WAVI_PG_OPENGL", "") not in ("", "0"):
        # plot windows drawn through OpenGL viewports: slower than the default raster
        # path on software GL (llvmpipe); try it on a real GPU with tools/viz_bench --opengl
        import pyqtgraph as pg
        pg.setConfigOptions(useOpenGL=True)
    app = QApplication(sys.argv)
    win = MainWindow(
        transport=args.transport,
        host=args.host,
        port=args.port,
        serial_port=args.serial_port,
        nsensor=args.nsensor,
        fs=args.fs,
        auto_fs=args.auto_fs,
        ns_read=args.ns_read,
        t_fft_s=args.t_fft,
        t_spec_s=args.t_spec,
        t_buffer_s=args.t_buffer,
        scale=args.scale,
        scan_mode=args.mode,
        selects=args.selects,
    )
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
