"""Binary frame format between the ESP32 firmware and the host.

Mirrors ``msd/esp32/whisker_daq/whisker_daq.ino`` (binary stream mode,
entered with the ``B`` command). Every frame is::

    0xA5 0x5A | type u8 | len u8 | payload[len] | crc16 (LE)

The CRC is CRC-16-CCITT (poly 0x1021, init 0xFFFF) over ``type``, ``len`` and
``payload`` — the same polynomial the ADS131M04 uses for its own frames.
All multi-byte fields are little-endian.

Frame types:

``SCAN`` (0x01) — one full sweep of the 32 gauges::

    u32 seq        scan counter
    u32 t_us       ESP32 micros() at the start of the sweep (wraps ~71.6 min)
    u16 dur_us     sweep duration
    u32 railmask   bit (o-1) set => OUT o is at full scale (nothing plugged in)
    u8  flags      bit0: an ADC halt was recovered during this sweep
                   bit1: an ADC SPI read failed CRC and was re-read
                   bit2: an ADC read carried no new conversion (STATUS DRDY
                         bits clear) and was dropped; older firmware never sets it
    u8  nch        number of codes that follow (32)
    nch x i24      raw ADC codes, OUT order 1..nch

``TEXT`` (0x02) — a UTF-8 log line from the firmware (halt reports etc.),
so diagnostics never corrupt the binary stream.

``INFO`` (0x03) — sent on entering binary mode::

    u8  proto      protocol version (PROTO_VERSION)
    u8  nch
    u16 device_id  ADS131M04 ID register
    u16 clock_rb   CLOCK register read-back
    u32 clkin_hz
    u32 fdata_sps  per-channel ADC data rate
    u32 baud

This module is pure stdlib so it runs on the BlueOS Pi without numpy.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterator, Union

SYNC = b"\xA5\x5A"
PROTO_VERSION = 1

T_SCAN = 0x01
T_TEXT = 0x02
T_INFO = 0x03

NCH = 32
_SCAN_HDR = struct.Struct("<IIHIBB")       # seq, t_us, dur_us, railmask, flags, nch
_INFO = struct.Struct("<BBHHIII")
FLAG_RECOVERED = 0x01
FLAG_CRC_RETRY = 0x02
FLAG_STALE = 0x04

# Physical conversion: code -> volts at the ADC input (PGA gain 1).
VREF_V = 1.2
ADC_FS = 1 << 23
MV_PER_CODE = VREF_V * 1000.0 / ADC_FS


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


# Table-driven variant for the hot path (~120 bytes x 80 Hz on a Pi).
_CRC_TABLE = []
for _i in range(256):
    _c = _i << 8
    for _ in range(8):
        _c = ((_c << 1) ^ 0x1021) & 0xFFFF if _c & 0x8000 else (_c << 1) & 0xFFFF
    _CRC_TABLE.append(_c)


def crc16_fast(data: bytes, crc: int = 0xFFFF) -> int:
    tbl = _CRC_TABLE
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ tbl[((crc >> 8) ^ b) & 0xFF]
    return crc


@dataclass
class Scan:
    seq: int
    t_us: int
    dur_us: int
    railmask: int
    flags: int
    raw: list[int]          # OUT order 1..nch


@dataclass
class Text:
    text: str


@dataclass
class Info:
    proto: int
    nch: int
    device_id: int
    clock_rb: int
    clkin_hz: int
    fdata_sps: int
    baud: int


Event = Union[Scan, Text, Info]


# ----- encoding (firmware does this in C; Python copy is for simulators/tests)

def _frame(ftype: int, payload: bytes) -> bytes:
    if len(payload) > 255:
        raise ValueError("payload too long")
    body = bytes((ftype, len(payload))) + payload
    return SYNC + body + struct.pack("<H", crc16_fast(body))


def encode_scan(seq: int, t_us: int, dur_us: int, railmask: int, flags: int,
                raw: list[int]) -> bytes:
    hdr = _SCAN_HDR.pack(seq & 0xFFFFFFFF, t_us & 0xFFFFFFFF, dur_us & 0xFFFF,
                         railmask & 0xFFFFFFFF, flags, len(raw))
    codes = b"".join((c & 0xFFFFFF).to_bytes(3, "little") for c in raw)
    return _frame(T_SCAN, hdr + codes)


def encode_text(text: str) -> bytes:
    return _frame(T_TEXT, text.encode("utf-8")[:255])


def encode_info(info: Info) -> bytes:
    return _frame(T_INFO, _INFO.pack(info.proto, info.nch, info.device_id,
                                     info.clock_rb, info.clkin_hz,
                                     info.fdata_sps, info.baud))


# ----- decoding

def _i24(b: bytes, off: int) -> int:
    v = b[off] | (b[off + 1] << 8) | (b[off + 2] << 16)
    return v - (1 << 24) if v & 0x800000 else v


def _parse(ftype: int, p: bytes) -> Event | None:
    if ftype == T_SCAN:
        if len(p) < _SCAN_HDR.size:
            return None
        seq, t_us, dur_us, rail, flags, nch = _SCAN_HDR.unpack_from(p)
        if len(p) != _SCAN_HDR.size + 3 * nch:
            return None
        off = _SCAN_HDR.size
        return Scan(seq, t_us, dur_us, rail, flags,
                    [_i24(p, off + 3 * i) for i in range(nch)])
    if ftype == T_TEXT:
        return Text(p.decode("utf-8", errors="replace"))
    if ftype == T_INFO and len(p) == _INFO.size:
        return Info(*_INFO.unpack(p))
    return None


class Decoder:
    """Incremental decoder: ``feed(bytes)`` yields events, resyncing on garbage.

    Bytes outside frames (the boot banner, text-mode output before ``B`` takes
    effect) are collected and returned as ``Text`` lines, so nothing the
    firmware prints is silently lost.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self._loose = bytearray()     # non-frame bytes, emitted as Text per line
        self.n_frames = 0
        self.n_crc_fail = 0
        self.n_skipped = 0

    def feed(self, data: bytes) -> Iterator[Event]:
        buf = self._buf
        buf += data
        while True:
            i = buf.find(SYNC)
            if i < 0:
                # keep a trailing 0xA5 in case the 0x5A arrives next
                keep = 1 if buf.endswith(SYNC[:1]) else 0
                yield from self._loose_bytes(bytes(buf[:len(buf) - keep]))
                del buf[:len(buf) - keep]
                return
            if i > 0:
                yield from self._loose_bytes(bytes(buf[:i]))
                del buf[:i]
            if len(buf) < 4:
                return
            plen = buf[3]
            total = 4 + plen + 2
            if len(buf) < total:
                return
            body = bytes(buf[2:4 + plen])
            rx_crc = buf[4 + plen] | (buf[5 + plen] << 8)
            if crc16_fast(body) != rx_crc:
                self.n_crc_fail += 1
                self.n_skipped += 1
                del buf[:1]                 # false sync; slide one byte
                continue
            del buf[:total]
            ev = _parse(body[0], body[2:])
            if ev is not None:
                self.n_frames += 1
                yield ev

    def _loose_bytes(self, b: bytes) -> Iterator[Event]:
        if not b:
            return
        self.n_skipped += len(b)
        self._loose += b
        while True:
            j = self._loose.find(b"\n")
            if j < 0:
                break
            line = bytes(self._loose[:j]).rstrip(b"\r").decode("utf-8", errors="replace")
            del self._loose[:j + 1]
            if line.strip():
                yield Text(line)
        if len(self._loose) > 4096:          # binary noise with no newline
            self._loose.clear()


class TimeUnwrapper:
    """Unwraps the ESP32's 32-bit micros() counter into a monotonic int."""

    def __init__(self) -> None:
        self._last: int | None = None
        self._offset = 0

    def __call__(self, t_us: int) -> int:
        if self._last is not None and t_us < self._last and self._last - t_us > (1 << 31):
            self._offset += 1 << 32
        self._last = t_us
        return t_us + self._offset
