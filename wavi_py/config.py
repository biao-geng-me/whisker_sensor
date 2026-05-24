"""Defaults and wire-format constants shared across modules.

The wire format mirrors what the Arduino in ``hx711_array/hx711_array.ino``
emits per sample:

    [nch x float32 LE][CR LF][float32 marker = 2024.0]   total = nch*4 + 6

A marker of 2024.0 stored little-endian as float32 is the byte sequence
``0x00 0x40 0xFD 0x44`` — see ``FRAME_MARKER_BYTES`` below.
"""

from __future__ import annotations

import struct


# Sampling and transport defaults
DEFAULT_FS_HZ = 80.0
DEFAULT_NSENSOR = 9
DEFAULT_NS_READ = 4              # samples per DAQ-thread batch
DEFAULT_T_BUFFER_S = 30.0        # ring-buffer depth on the SampleBus
DEFAULT_T_FFT_S = 1.0
DEFAULT_T_SPEC_S = 20.0
DEFAULT_BAUDRATE = 2_000_000

# TCP transport defaults
DEFAULT_TCP_HOST = "127.0.0.1"
DEFAULT_TCP_PORT = 5555
TCP_CONNECT_TIMEOUT_S = 5.0

# Frame layout — derive concrete byte sizes from nch at runtime via frame_bytes()
FRAME_MARKER_VALUE = 2024.0
FRAME_MARKER_BYTES = struct.pack("<f", FRAME_MARKER_VALUE)  # b"\x00\x40\xfd\x44"
FRAME_LINE_TERMINATOR = b"\r\n"                              # Arduino Serial.println()
FRAME_TAIL_BYTES = len(FRAME_LINE_TERMINATOR) + len(FRAME_MARKER_BYTES)  # = 6


def frame_bytes(nch: int) -> int:
    """Total bytes per sample frame for a given channel count."""
    return nch * 4 + FRAME_TAIL_BYTES


# Outlier removal default window (matches wavi.m ns_fill)
DEFAULT_NS_FILL = 6

# Minimum recording length for auto-stop (matches wavi.m MIN_REC_LEN)
MIN_REC_LEN_S = 5
