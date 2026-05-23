"""USB-serial access for the hx711_array Arduino firmware.

The firmware in this folder streams little-endian binary frames at ~80 Hz:

    [nch x float32 voltages][0x0A '\\n'][float32 marker = 2024.0]

with `nch = nsensor * 2` and `nbytes_per_sample = nch * 4 + 5`. The host
sends an ASCII `N=<num>\\n` line once at startup so the Arduino can size
its HX711 array, then single bytes `S` / `E` toggle the recording-indicator
output pin.

Phase 1 only needs to open the serial port; the Pi-side bridge forwards
raw bytes both ways. Frame helpers (alignment + parsed reads) will be
added here in Phase 2 when the Pi takes ownership of parsing.
"""

from __future__ import annotations

import time

import serial


DEFAULT_BAUD = 2_000_000
DEFAULT_POST_OPEN_SLEEP = 1.5  # seconds; matches the wait in wavi.m onSerialConnected


def open_serial(
    port: str,
    baudrate: int = DEFAULT_BAUD,
    post_open_sleep: float = DEFAULT_POST_OPEN_SLEEP,
    read_timeout: float = 1.0,
    write_timeout: float = 1.0,
) -> serial.Serial:
    """Open the Arduino USB serial port.

    Opening the port toggles DTR and resets the Arduino. The caller should
    expect the firmware to be in its `N=` wait state for ~1.5 s afterward.

    `dsrdtr=False` keeps pyserial from asserting DTR again after open on
    some platforms; the firmware's reset on open is unavoidable but at
    least repeated resets are not stacked.
    """
    s = serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=read_timeout,
        write_timeout=write_timeout,
        dsrdtr=False,
        rtscts=False,
        xonxoff=False,
    )
    if post_open_sleep > 0:
        time.sleep(post_open_sleep)
    return s
