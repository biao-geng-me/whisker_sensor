"""Abstract byte transport.

The DAQ thread depends only on this interface, not on the concrete serial /
socket object. Matches the MATLAB pattern where ``wavi.m`` calls
``read(obj.s, ...)`` regardless of whether ``obj.s`` is a ``serialport`` or a
``tcpclient``.
"""

from __future__ import annotations

import abc


class Transport(abc.ABC):
    """Byte-level read/write to the data source."""

    @abc.abstractmethod
    def read_exact(self, n: int) -> bytes:
        """Block until exactly ``n`` bytes are read, or raise on EOF/close."""

    @abc.abstractmethod
    def read_some(self, max_n: int) -> bytes:
        """Read up to ``max_n`` bytes, return whatever is available. May block."""

    @abc.abstractmethod
    def write(self, data: bytes) -> None:
        """Write all of ``data``. Blocks until done."""

    @abc.abstractmethod
    def bytes_available(self) -> int:
        """Best-effort count of bytes ready to read without blocking."""

    @abc.abstractmethod
    def close(self) -> None:
        """Tear down the underlying handle. Safe to call twice."""

    @property
    @abc.abstractmethod
    def is_open(self) -> bool: ...
