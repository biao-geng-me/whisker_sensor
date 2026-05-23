"""Transparent USB-serial <-> TCP bridge for the hx711_array Arduino.

Phase 1 of the Pi-side stack: byte pump only. No parsing, no recording.

The Pi opens the Arduino serial port once at startup and keeps it open
across client connect/disconnect cycles so the firmware is not reset on
every reconnect (Arduino reset happens only on serial port open). A
single TCP client is served at a time; bytes flow verbatim in both
directions.

Wire format flowing through the bridge is the Arduino's existing one
(see hx711_array/hx711_array.ino):

    Arduino -> host: [nch x float32][0x0A][float32 = 2024.0] repeating
    host -> Arduino: ASCII "N=<num>\\n" once, then single bytes 'S' / 'E'

The MATLAB client (wavi.m with transport='tcp') handles framing and
alignment exactly as it does over a direct serial connection.

Run from the repo root:
    python3 -u rpi/daq_bridge.py --port /dev/ttyACM0 --bind 0.0.0.0:5555
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import socket
import sys
import threading

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_THIS_DIR)
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from hx711_array.arduino_reader import open_serial, DEFAULT_BAUD  # noqa: E402

log = logging.getLogger("daq_bridge")


class Bridge:
    def __init__(self, serial_port: str, baud: int, host: str, port: int):
        self.ser = open_serial(serial_port, baudrate=baud)
        log.info("Opened serial %s @ %d baud", serial_port, baud)

        self._listen_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listen_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
        self._listen_sock.bind((host, port))
        self._listen_sock.listen(1)
        self._listen_sock.settimeout(0.5)
        log.info("Listening on %s:%d", host, port)

        self._stop = threading.Event()

    def run(self) -> None:
        while not self._stop.is_set():
            try:
                conn, addr = self._listen_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            log.info("Client connected: %s", addr)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._drain_serial()
            try:
                self._serve_client(conn)
            finally:
                try:
                    conn.close()
                except OSError:
                    pass
            log.info("Client disconnected; serial stays open")

    def _drain_serial(self) -> None:
        n = self.ser.in_waiting
        if n:
            self.ser.read(n)
            log.info("Drained %d stale serial bytes before new client", n)

    def _serve_client(self, conn: socket.socket) -> None:
        client_stop = threading.Event()
        t_in = threading.Thread(
            target=self._serial_to_sock, args=(conn, client_stop), daemon=True
        )
        t_out = threading.Thread(
            target=self._sock_to_serial, args=(conn, client_stop), daemon=True
        )
        t_in.start()
        t_out.start()

        while not client_stop.is_set() and not self._stop.is_set():
            client_stop.wait(timeout=0.5)

        try:
            conn.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        t_in.join(timeout=1.0)
        t_out.join(timeout=1.0)

    def _serial_to_sock(self, conn: socket.socket, client_stop: threading.Event) -> None:
        try:
            while not client_stop.is_set() and not self._stop.is_set():
                # read all available, or block up to the serial timeout for 1 byte
                n = self.ser.in_waiting or 1
                data = self.ser.read(n)
                if data:
                    conn.sendall(data)
        except (OSError, ConnectionError) as e:
            log.info("serial->sock ended: %s", e)
        finally:
            client_stop.set()

    def _sock_to_serial(self, conn: socket.socket, client_stop: threading.Event) -> None:
        try:
            conn.settimeout(0.5)
            while not client_stop.is_set() and not self._stop.is_set():
                try:
                    data = conn.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    break
                self.ser.write(data)
        except (OSError, ConnectionError) as e:
            log.info("sock->serial ended: %s", e)
        finally:
            client_stop.set()

    def stop(self) -> None:
        log.info("Stopping bridge")
        self._stop.set()
        try:
            self._listen_sock.close()
        except OSError:
            pass
        try:
            self.ser.close()
        except OSError:
            pass


def _parse_bind(s: str) -> tuple[str, int]:
    host, _, port = s.rpartition(":")
    if not host or not port:
        raise argparse.ArgumentTypeError(f"--bind must be host:port, got {s!r}")
    return host, int(port)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyACM0", help="Arduino serial device path")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--bind", type=_parse_bind, default=("0.0.0.0", 5555),
                    help="host:port to listen on (default 0.0.0.0:5555)")
    ap.add_argument("--log-level", default="INFO")
    args = ap.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(message)s",
    )

    host, port = args.bind
    bridge = Bridge(args.port, args.baud, host, port)

    def _signal_stop(*_: object) -> None:
        bridge.stop()

    signal.signal(signal.SIGINT, _signal_stop)
    signal.signal(signal.SIGTERM, _signal_stop)

    try:
        bridge.run()
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()
