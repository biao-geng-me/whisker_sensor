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
                   bit3: the ADC had reset (STATUS RESET bit) and was re-initialised;
                         the sweep was restarted. Older firmware never sets it
                   bit4: locked scan: conversions missed / mux switched late /
                         resynced to the conversion grid in this scan (SYNC)
                   bit5: locked scan: a slot had no usable conversion; its outputs
                         repeat the previous value (EMPTY)
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
    -- appended in PROTO_VERSION 2 (absent from version-1 firmware) --
    u16 osr              ADC oversampling ratio of the normal scan
    u16 settle_us        mux settle time per select
    u8  discard_samples  conversions dropped per select after the settle
    -- appended in PROTO_VERSION 3 --
    u8  sel_mask         mux selects in the scan (bit s = select s). The 4
                         outputs of a skipped select carry +full scale, so the
                         SCAN railmask marks them railed ("open")
    -- appended in PROTO_VERSION 4 (scan modes) --
    u8  mode             0 legacy (~76 Hz), 1 quiet (62.5 Hz locked), 2 wide (250 Hz)
    u32 spi_hz           SPI clock in effect
    u16 n_conv           conversions per scan (locked modes; 0 = legacy)
    u8 x 8 avg_per_sel   conversions averaged per select (0 = not scanned)
    In the locked modes clkin_hz, fdata_sps, osr, settle_us and discard_samples
    describe the locked scan (discard = conversions dropped at each slot start).

``HEALTH`` (0x04) — about once a second in the locked modes::

    u8 version, u8 mode, u16 busy_permille (reader + sender CPU time),
    u16 lat_max_us, u16 lat_avg_x10 (DRDY edge -> end of read),
    u16 read_avg_x10, u16 read_max_us (one 18-byte SPI read), u16 tx_max_us,
    u32 conversions, u16 overrun, u16 missed, u16 edgeless, u16 crc, u16 stale,
    u16 late_switch, u16 tx_drop, u16 empty_slots, u16 resync, u16 reinit,
    u16 recov, u16 window_ms

This module is pure stdlib so it runs on the BlueOS Pi without numpy.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterator, Union

SYNC = b"\xA5\x5A"
PROTO_VERSION = 4

T_SCAN = 0x01
T_TEXT = 0x02
T_INFO = 0x03
T_HEALTH = 0x04

NCH = 32
_SCAN_HDR = struct.Struct("<IIHIBB")       # seq, t_us, dur_us, railmask, flags, nch
_INFO = struct.Struct("<BBHHIII")          # version 1
_INFO_EXT = struct.Struct("<HHB")          # appended in version 2
_INFO_EXT3 = struct.Struct("<B")           # appended in version 3
_INFO_EXT4 = struct.Struct("<BIH8B")       # appended in version 4
_HEALTH = struct.Struct("<BBHHHHHHI" + "H" * 12)
MODE_NAMES = {0: "legacy", 1: "quiet", 2: "wide"}
FLAG_RECOVERED = 0x01
FLAG_CRC_RETRY = 0x02
FLAG_STALE = 0x04
FLAG_REINIT = 0x08
FLAG_SYNC = 0x10
FLAG_EMPTY = 0x20

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
    osr: int | None = None              # None: version-1 firmware didn't send it
    settle_us: int | None = None
    discard_samples: int | None = None
    sel_mask: int | None = None         # None: firmware before version 3 (all 8 scanned)
    mode: int | None = None             # None: firmware before version 4 (legacy scan)
    spi_hz: int | None = None
    n_conv: int | None = None
    avg_per_sel: tuple | None = None    # conversions averaged per select 0..7


@dataclass
class Health:
    version: int
    mode: int
    busy_permille: int
    lat_max_us: int
    lat_avg_x10: int
    read_avg_x10: int
    read_max_us: int
    tx_max_us: int
    conversions: int
    overrun: int
    missed: int
    edgeless: int
    crc: int
    stale: int
    late_switch: int
    tx_drop: int
    empty_slots: int
    resync: int
    reinit: int
    recov: int
    window_ms: int

    def summary(self) -> dict:
        """The fields a status record carries (busy %, latencies in us)."""
        d = dict(self.__dict__)
        d["busy_pct"] = round(self.busy_permille / 10, 1)
        d["lat_avg_us"] = round(self.lat_avg_x10 / 10, 1)
        d["read_avg_us"] = round(self.read_avg_x10 / 10, 1)
        for k in ("busy_permille", "lat_avg_x10", "read_avg_x10", "version"):
            d.pop(k)
        return d


Event = Union[Scan, Text, Info, Health]


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
    p = _INFO.pack(info.proto, info.nch, info.device_id, info.clock_rb,
                   info.clkin_hz, info.fdata_sps, info.baud)
    if info.osr is not None:
        p += _INFO_EXT.pack(info.osr, info.settle_us or 0, info.discard_samples or 0)
        if info.sel_mask is not None:
            p += _INFO_EXT3.pack(info.sel_mask)
            if info.mode is not None:
                p += _INFO_EXT4.pack(info.mode, info.spi_hz or 0, info.n_conv or 0,
                                     *(info.avg_per_sel or (0,) * 8))
    return _frame(T_INFO, p)


def info_cfg(info: dict) -> dict:
    """The active ADC/scan config from an INFO event dict, for status records.

    Keys a version-1 INFO doesn't carry are left out rather than guessed.
    """
    keys = ("clkin_hz", "fdata_sps", "osr", "settle_us", "discard_samples", "sel_mask",
            "mode", "spi_hz", "n_conv")
    cfg = {k: info[k] for k in keys if info.get(k) is not None}
    if "sel_mask" in cfg:
        cfg["selects"] = "".join(str(s) for s in range(8) if cfg["sel_mask"] >> s & 1)
    if "mode" in cfg:
        cfg["mode_name"] = MODE_NAMES.get(cfg["mode"], str(cfg["mode"]))
        # fdata_sps is a whole number on the wire (7812 at OSR 512): use CLKIN/(2 OSR)
        fdata = (cfg["clkin_hz"] / 2 / cfg["osr"] if cfg.get("osr") and cfg.get("clkin_hz")
                 else cfg.get("fdata_sps"))
        if cfg["n_conv"] and fdata:
            cfg["scan_hz"] = round(fdata / cfg["n_conv"], 3)
    if info.get("avg_per_sel") is not None:
        cfg["avg_per_sel"] = list(info["avg_per_sel"])
    return cfg


def encode_health(h: Health) -> bytes:
    return _frame(T_HEALTH, _HEALTH.pack(*(getattr(h, f) for f in Health.__dataclass_fields__)))


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
    if ftype == T_INFO and len(p) == _INFO.size + _INFO_EXT.size:
        return Info(*_INFO.unpack_from(p), *_INFO_EXT.unpack_from(p, _INFO.size))
    if ftype == T_INFO and len(p) == _INFO.size + _INFO_EXT.size + _INFO_EXT3.size:
        return Info(*_INFO.unpack_from(p), *_INFO_EXT.unpack_from(p, _INFO.size),
                    *_INFO_EXT3.unpack_from(p, _INFO.size + _INFO_EXT.size))
    n3 = _INFO.size + _INFO_EXT.size + _INFO_EXT3.size
    if ftype == T_INFO and len(p) == n3 + _INFO_EXT4.size:
        mode, spi, n_conv, *avg = _INFO_EXT4.unpack_from(p, n3)
        return Info(*_INFO.unpack_from(p), *_INFO_EXT.unpack_from(p, _INFO.size),
                    *_INFO_EXT3.unpack_from(p, _INFO.size + _INFO_EXT.size),
                    mode, spi, n_conv, tuple(avg))
    if ftype == T_HEALTH and len(p) >= _HEALTH.size:      # longer = newer firmware
        return Health(*_HEALTH.unpack_from(p))
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
