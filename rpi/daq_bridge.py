"""Transparent USB-serial <-> TCP fan-out bridge for the hx711_array Arduino.

Multi-client successor to the single-client byte pump. The Pi opens the
Arduino serial port once at startup and keeps it open across the entire
session. A single background thread continuously reads bytes from the
Arduino and broadcasts each chunk to every connected TCP client's send
queue, so every client receives the same raw byte stream the Arduino
emits.

Wire format flowing through the bridge is the Arduino's existing one
(see hx711_array/hx711_array.ino):

    Arduino -> host: [nch x float32][\\r\\n][float32 = 2024.0] repeating
    host -> Arduino: ASCII "N=<num>\\n" once, then single bytes 'S' / 'E'

Each client also gets its own reader thread that forwards bytes from the
socket back to the Arduino (serialized by a write lock). Any client can
send N=, S, or E; the Arduino's loop() only acts on S/E and ignores
everything else, so two clients fighting over the record-indicator pin
can race — coordinate at the application level if it matters.

Slow-client policy: each client has a bounded send queue. If it fills
(client can't keep up with 80 Hz x ~78 byte frames for ~16 s), the
bridge logs a warning and disconnects that client. Other clients are
unaffected.

The MATLAB / Python clients (wavi.m, wavi_py) do byte-by-byte
marker-search alignment, so a client joining mid-stream finds the next
frame boundary on its own — the bridge doesn't drain or pad anything.

Run from the repo root:
    python3 -u rpi/daq_bridge.py --port /dev/ttyACM0 --bind 0.0.0.0:5555
"""

from __future__ import annotations

import argparse
import logging
import os
import queue
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


_DEFAULT_LISTEN_BACKLOG = 8
_DEFAULT_CLIENT_QUEUE_MAX = 1024     # chunks; ~16 s of buffering at 80 Hz batches
_DEFAULT_RECV_CHUNK = 4096


class _Client:
    """One connected TCP client. Owns its socket + a writer/reader thread pair."""

    def __init__(
        self,
        conn: socket.socket,
        addr,
        ser,
        ser_write_lock: threading.Lock,
        on_close,
        queue_max: int = _DEFAULT_CLIENT_QUEUE_MAX,
    ):
        self.conn = conn
        self.addr = addr
        self.ser = ser
        self.ser_write_lock = ser_write_lock
        self.on_close = on_close
        self.send_q: queue.Queue = queue.Queue(maxsize=queue_max)
        self._stop = threading.Event()
        self._closed = False
        self._writer_thread = threading.Thread(
            target=self._writer, name=f"client-{addr}-w", daemon=True
        )
        self._reader_thread = threading.Thread(
            target=self._reader, name=f"client-{addr}-r", daemon=True
        )

    def start(self) -> None:
        self._writer_thread.start()
        self._reader_thread.start()

    def enqueue(self, data: bytes) -> bool:
        """Try to add data to the send queue. Returns False if the queue is full."""
        try:
            self.send_q.put_nowait(data)
            return True
        except queue.Full:
            return False

    def stop(self) -> None:
        self._close()

    def _writer(self) -> None:
        try:
            while not self._stop.is_set():
                try:
                    data = self.send_q.get(timeout=0.5)
                except queue.Empty:
                    continue
                self.conn.sendall(data)
        except (OSError, ConnectionError) as exc:
            log.info("client %s writer exit: %s", self.addr, exc)
        finally:
            self._close()

    def _reader(self) -> None:
        try:
            self.conn.settimeout(0.5)
            while not self._stop.is_set():
                try:
                    data = self.conn.recv(_DEFAULT_RECV_CHUNK)
                except socket.timeout:
                    continue
                if not data:
                    break
                with self.ser_write_lock:
                    self.ser.write(data)
        except (OSError, ConnectionError) as exc:
            log.info("client %s reader exit: %s", self.addr, exc)
        finally:
            self._close()

    def _close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        try:
            self.conn.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.conn.close()
        except OSError:
            pass
        try:
            self.on_close(self)
        except Exception as exc:  # noqa: BLE001
            log.warning("client %s on_close raised: %s", self.addr, exc)


class Bridge:
    def __init__(self, serial_port: str, baud: int, host: str, port: int):
        self.ser = open_serial(serial_port, baudrate=baud)
        log.info("Opened serial %s @ %d baud", serial_port, baud)

        self._ser_write_lock = threading.Lock()

        self._listen_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listen_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
        self._listen_sock.bind((host, port))
        self._listen_sock.listen(_DEFAULT_LISTEN_BACKLOG)
        self._listen_sock.settimeout(0.5)
        log.info("Listening on %s:%d (multi-client fan-out)", host, port)

        self._stop = threading.Event()
        self._clients: list[_Client] = []
        self._clients_lock = threading.Lock()

        self._fanout_thread = threading.Thread(
            target=self._serial_to_clients, name="serial-fanout", daemon=True
        )

    # ----- lifecycle

    def run(self) -> None:
        self._fanout_thread.start()
        while not self._stop.is_set():
            try:
                conn, addr = self._listen_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            client = _Client(
                conn, addr, self.ser, self._ser_write_lock, self._on_client_closed
            )
            with self._clients_lock:
                self._clients.append(client)
                n = len(self._clients)
            log.info("Client connected: %s (now %d total)", addr, n)
            client.start()

    def stop(self) -> None:
        log.info("Stopping bridge")
        self._stop.set()
        # Close all clients first so their threads exit before we close serial.
        with self._clients_lock:
            clients = list(self._clients)
        for c in clients:
            c.stop()
        try:
            self._listen_sock.close()
        except OSError:
            pass
        try:
            self.ser.close()
        except OSError:
            pass

    # ----- callbacks / workers

    def _on_client_closed(self, client: _Client) -> None:
        with self._clients_lock:
            if client in self._clients:
                self._clients.remove(client)
            n = len(self._clients)
        log.info("Client disconnected: %s (now %d total)", client.addr, n)

    def _serial_to_clients(self) -> None:
        """Background thread: read Arduino bytes and broadcast to every client.

        Runs from bridge startup, regardless of client count. Bytes are
        silently dropped when no client is connected — the serial buffer
        never accumulates, so newcomers always see live data.
        """
        try:
            while not self._stop.is_set():
                try:
                    n = self.ser.in_waiting or 1
                    data = self.ser.read(n)
                except (OSError, ConnectionError) as exc:
                    log.warning("serial read failed: %s", exc)
                    break
                if not data:
                    continue   # serial read timed out; loop and check stop flag

                with self._clients_lock:
                    clients = list(self._clients)
                slow: list[_Client] = []
                for c in clients:
                    if not c.enqueue(data):
                        slow.append(c)
                for c in slow:
                    log.warning(
                        "dropping slow client %s (send queue full, can't keep up)",
                        c.addr,
                    )
                    c.stop()
        finally:
            log.info("serial fan-out exiting")


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
