"""GUI entry point. Run with ``python -m wavi_py [args]``.

For headless / library usage, import ``WaviClient`` directly — this module
is only the Qt-based front door.
"""

from __future__ import annotations

import argparse
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


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="wavi_py",
        description="Python whisker-sensor DAQ client (GUI).",
    )
    p.add_argument("--transport", choices=["tcp", "serial"], default="tcp")
    p.add_argument("--host", default=DEFAULT_TCP_HOST, help="TCP host (Pi IP or hostname)")
    p.add_argument("--port", type=int, default=DEFAULT_TCP_PORT, help="TCP port")
    p.add_argument("--serial-port", default=None, help="e.g. COM3 or /dev/ttyACM0")
    p.add_argument("--nsensor", type=int, default=DEFAULT_NSENSOR)
    p.add_argument("--fs", type=float, default=DEFAULT_FS_HZ, help="sample rate Hz")
    p.add_argument("--ns-read", type=int, default=DEFAULT_NS_READ,
                   help="samples per DAQ batch")
    p.add_argument("--t-fft", type=float, default=DEFAULT_T_FFT_S, help="FFT window seconds")
    p.add_argument("--t-spec", type=float, default=DEFAULT_T_SPEC_S, help="Spectrogram seconds")
    p.add_argument("--t-buffer", type=float, default=DEFAULT_T_BUFFER_S, help="signal buffer seconds")
    p.add_argument("--scale", type=float, default=1.0, help="line view amplitude scale")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)

    # Qt imports are deferred so help / parsing errors don't pull in PyQt6.
    from PyQt6.QtWidgets import QApplication

    from wavi_py.main_window import MainWindow

    app = QApplication(sys.argv)
    win = MainWindow(
        transport=args.transport,
        host=args.host,
        port=args.port,
        serial_port=args.serial_port,
        nsensor=args.nsensor,
        fs=args.fs,
        ns_read=args.ns_read,
        t_fft_s=args.t_fft,
        t_spec_s=args.t_spec,
        t_buffer_s=args.t_buffer,
        scale=args.scale,
    )
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
