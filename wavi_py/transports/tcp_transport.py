"""TCP transport — talks to ``rpi/daq_bridge.py``.

Style follows ``gantry_control/python/connection_manager.py``: raw socket,
``TCP_NODELAY``, bumped ``SO_RCVBUF``, and a ``_recvall``-style exact-read.
"""

from __future__ import annotations

import socket

from wavi_py.config import TCP_CONNECT_TIMEOUT_S
from wavi_py.transports.base import Transport


class TcpTransport(Transport):
    def __init__(self, host: str, port: int, connect_timeout_s: float = TCP_CONNECT_TIMEOUT_S):
        self._sock: socket.socket | None = None
        self._host = host
        self._port = port
        self._connect_timeout_s = connect_timeout_s
        self._open()

    def _open(self) -> None:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self._connect_timeout_s)
        s.connect((self._host, self._port))
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
        s.settimeout(None)  # block on subsequent reads
        self._sock = s

    @property
    def is_open(self) -> bool:
        return self._sock is not None

    def read_exact(self, n: int) -> bytes:
        if self._sock is None:
            raise ConnectionError("transport closed")
        out = bytearray()
        while len(out) < n:
            chunk = self._sock.recv(n - len(out))
            if not chunk:
                raise ConnectionError(f"transport closed after {len(out)}/{n} bytes")
            out.extend(chunk)
        return bytes(out)

    def read_some(self, max_n: int) -> bytes:
        if self._sock is None:
            raise ConnectionError("transport closed")
        chunk = self._sock.recv(max_n)
        if not chunk:
            raise ConnectionError("transport closed")
        return chunk

    def write(self, data: bytes) -> None:
        if self._sock is None:
            raise ConnectionError("transport closed")
        self._sock.sendall(data)

    def bytes_available(self) -> int:
        # No portable non-blocking peek on a stream socket; the DAQ thread
        # uses read_exact / read_some which block as needed.
        return 0

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
