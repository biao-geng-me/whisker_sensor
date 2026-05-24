"""High-level facade — the single public entry point.

``WaviClient`` wires up the transport, the ``SampleBus``, and the DAQ
``Reader``. It's deliberately Qt-free so headless library users (agent
control loops, batch capture, tests) don't pay the PyQt import cost.

GUI mode (``python -m wavi_py``) instantiates a ``WaviClient`` and then
``MainWindow`` from ``wavi_py.main_window``; the GUI attaches as one more
subscriber on the same bus.

Typical headless usage::

    from wavi_py import WaviClient

    client = WaviClient(transport="tcp", host="192.168.2.3", port=5555,
                        nsensor=9, fs=80)
    client.start()
    client.wait_ready(timeout=10)
    last_seq = 0
    while running:
        samples, last_seq = client.bus.read_since(
            last_seq, block=True, timeout=0.1
        )
        if samples is not None:
            do_something(samples)
    client.stop()
"""

from __future__ import annotations

from typing import Optional

from wavi_py.config import (
    DEFAULT_FS_HZ,
    DEFAULT_NS_READ,
    DEFAULT_NSENSOR,
    DEFAULT_T_BUFFER_S,
    DEFAULT_TCP_HOST,
    DEFAULT_TCP_PORT,
    DEFAULT_BAUDRATE,
)
from wavi_py.daq.bus import SampleBus
from wavi_py.daq.reader import Reader
from wavi_py.transports.base import Transport
from wavi_py.transports.serial_transport import SerialTransport
from wavi_py.transports.tcp_transport import TcpTransport


class WaviClient:
    """Owns transport + DAQ thread + SampleBus.

    Construction is cheap — no I/O. Call ``start()`` to actually connect and
    begin streaming, ``stop()`` to tear down cleanly. ``bus`` is the public
    pub/sub surface: pass it to subscribers or call ``bus.read_since()`` /
    ``bus.subscribe()`` directly.
    """

    def __init__(
        self,
        *,
        transport: str = "tcp",
        host: str = DEFAULT_TCP_HOST,
        port: int = DEFAULT_TCP_PORT,
        serial_port: Optional[str] = None,
        baudrate: int = DEFAULT_BAUDRATE,
        nsensor: int = DEFAULT_NSENSOR,
        fs: float = DEFAULT_FS_HZ,
        ns_read: int = DEFAULT_NS_READ,
        t_buffer_s: float = DEFAULT_T_BUFFER_S,
    ):
        transport = transport.lower()
        if transport not in ("tcp", "serial"):
            raise ValueError(f"transport must be 'tcp' or 'serial', got {transport!r}")
        if transport == "serial" and not serial_port:
            raise ValueError("serial transport requires serial_port (e.g. 'COM3', '/dev/ttyACM0')")

        # connection params (deferred until start())
        self._transport_kind = transport
        self._tcp_host = host
        self._tcp_port = int(port)
        self._serial_port = serial_port
        self._baudrate = int(baudrate)

        # acquisition params
        self.nsensor = int(nsensor)
        self.nch = self.nsensor * 2
        self.fs = float(fs)
        self.ns_read = int(ns_read)
        self.t_buffer_s = float(t_buffer_s)

        capacity = max(self.ns_read * 2, int(round(self.t_buffer_s * self.fs)))
        self.bus = SampleBus(capacity=capacity, nch=self.nch)

        self._transport: Optional[Transport] = None
        self._reader: Optional[Reader] = None

    # ----- lifecycle

    def start(self) -> None:
        """Open the transport and start the DAQ thread.

        Returns immediately; call ``wait_ready()`` if you need to block until
        the V0 offset has been computed and the first samples are flowing.
        """
        if self._reader is not None and self._reader.is_running:
            return

        self._transport = self._build_transport()

        self._reader = Reader(
            transport=self._transport,
            bus=self.bus,
            fs=self.fs,
            nch=self.nch,
            nsensor=self.nsensor,
            ns_read=self.ns_read,
        )
        self._reader.start()

    def stop(self) -> None:
        """Stop the DAQ thread and close the transport. Idempotent.

        Order matters: we set the reader's stop flag first, then close the
        transport (which unblocks any in-flight read_exact), then join. This
        way the reader returns silently instead of printing an error.
        """
        if self._reader is not None:
            self._reader.request_stop()
        if self._transport is not None:
            # Best-effort: tell the Arduino we're done recording before we drop the line.
            try:
                self._transport.write(b"E")
            except Exception:
                pass
            self._transport.close()
            self._transport = None
        if self._reader is not None:
            self._reader.stop()
            self._reader = None

    def wait_ready(self, timeout: Optional[float] = None) -> bool:
        """Block until the reader has aligned + computed V0. Returns True on success."""
        if self._reader is None:
            return False
        return self._reader.wait_ready(timeout=timeout)

    # ----- record-indicator passthrough

    def send_record_start(self) -> bool:
        """Send 'S' so the Arduino raises its CONTROL_PIN. Returns True if sent."""
        return self._send_cmd(b"S")

    def send_record_stop(self) -> bool:
        """Send 'E' so the Arduino drops CONTROL_PIN. Returns True if sent."""
        return self._send_cmd(b"E")

    def _send_cmd(self, payload: bytes) -> bool:
        if self._transport is None or not self._transport.is_open:
            return False
        try:
            self._transport.write(payload)
            return True
        except Exception:
            return False

    # ----- status

    @property
    def is_running(self) -> bool:
        return self._reader is not None and self._reader.is_running

    @property
    def transport_kind(self) -> str:
        return self._transport_kind

    @property
    def endpoint_str(self) -> str:
        if self._transport_kind == "tcp":
            return f"tcp://{self._tcp_host}:{self._tcp_port}"
        return f"serial://{self._serial_port}@{self._baudrate}"

    # ----- internals

    def _build_transport(self) -> Transport:
        if self._transport_kind == "tcp":
            return TcpTransport(self._tcp_host, self._tcp_port)
        assert self._serial_port is not None
        return SerialTransport(self._serial_port, baudrate=self._baudrate)
