"""DAQ thread.

Owns the byte stream. Does only:

1. Send the Arduino's ``N=<nsensor>\\n`` configuration line.
2. Align to a frame boundary by scanning for the 2024.0 marker.
3. Read a chunk of frames at startup, average → V0 offset, publish as metadata.
4. Loop: read ``ns_read`` frames, parse to ``(ns_read, nch)`` float32, publish.

No filter, no outlier removal, no recording, no Qt. Subscribers handle that.
This keeps DAQ latency low and makes the bus the single source of truth.

Equivalent MATLAB code:
- ``wavi.m:align_data_read`` ([wavi.m:799-826](wavi/wavi.m#L799-L826))
- ``wavi.m:average_signal_as_offset`` ([wavi.m:828-839](wavi/wavi.m#L828-L839))
- ``wavi.m:read_serial_data`` ([wavi.m:854-885](wavi/wavi.m#L854-L885))
- ``wavi.m:print_sig_vals`` ([wavi.m:916-973](wavi/wavi.m#L916-L973))
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np

from wavi_py.config import (
    DEFAULT_NS_READ,
    FRAME_MARKER_BYTES,
)
from wavi_py.config import frame_bytes as frame_size_for
from wavi_py.daq.bus import SampleBus
from wavi_py.daq.outliers import fill_outliers_linear
from wavi_py.transports.base import Transport


class Reader:
    """Background-thread DAQ reader. One per ``WaviClient``."""

    def __init__(
        self,
        transport: Transport,
        bus: SampleBus,
        *,
        fs: float,
        nch: int,
        nsensor: int,
        ns_read: int = DEFAULT_NS_READ,
        stats_interval_s: float = 1.0,
    ):
        self._transport = transport
        self._bus = bus
        self._fs = float(fs)
        self._nch = int(nch)
        self._nsensor = int(nsensor)
        self._ns_read = int(ns_read)
        self._stats_interval_s = float(stats_interval_s)

        self._stop = threading.Event()
        self._reset_pending = threading.Event()  # set by request_reset()
        self._thread: Optional[threading.Thread] = None
        self._ready = threading.Event()    # set once V0 is computed and streaming starts

        # diagnostic counters
        self._total_samples = 0
        self._run_start: Optional[float] = None
        self._window_start: Optional[float] = None
        self._window_samples_at_start = 0

    # ----- lifecycle

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._ready.clear()
        self._thread = threading.Thread(target=self._run, name="wavi-daq", daemon=True)
        self._thread.start()

    def request_stop(self) -> None:
        """Set the stop flag without joining. Use before closing the transport
        so the reader's pending blocking read returns silently."""
        self._stop.set()

    def request_reset(self) -> None:
        """Ask the stream loop to recompute V0 on the next iteration. Cheap
        and idempotent; multiple requests collapse into one reset."""
        self._reset_pending.set()

    def stop(self, join_timeout_s: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=join_timeout_s)
            self._thread = None

    def wait_ready(self, timeout: Optional[float] = None) -> bool:
        """Block until the V0 offset is computed and streaming has started."""
        return self._ready.wait(timeout=timeout)

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ----- main thread function

    def _run(self) -> None:
        try:
            self._send_nsensor()
            self._align()
            v0 = self._compute_v0()
            self._bus.set_metadata(
                V0=v0,
                v0_version=1,
                fs=self._fs,
                nsensor=self._nsensor,
                nch=self._nch,
                t0_unix=time.time(),
            )
            print(
                f"[Reader] V0 offset: "
                + " ".join(f"{v:7.3f}" for v in v0)
            )

            self._run_start = time.perf_counter()
            self._window_start = self._run_start
            self._ready.set()

            self._stream_loop()
        except Exception as exc:  # noqa: BLE001
            if not self._stop.is_set():
                print(f"[Reader] terminating: {exc}")
        finally:
            self._bus.close()

    # ----- protocol pieces

    def _send_nsensor(self) -> None:
        """Send ``N=<n>\\n`` to the Arduino. Harmless if already configured."""
        try:
            self._transport.write(f"N={self._nsensor}\n".encode("ascii"))
        except Exception as exc:  # noqa: BLE001
            raise ConnectionError(f"failed to send N=: {exc}") from exc

    def _align(self) -> None:
        """Scan bytes for the 2024.0 marker, leaving the cursor at a frame start."""
        # Read one byte at a time, maintain a 4-byte sliding window.
        # Slow but only runs at startup (a few hundred bytes typically).
        window = bytearray(b"\x00\x00\x00\x00")
        n_scanned = 0
        deadline = time.monotonic() + 30.0  # give up after 30 s of bad data
        while not self._stop.is_set():
            b = self._transport.read_exact(1)
            window[0:3] = window[1:4]
            window[3] = b[0]
            n_scanned += 1
            if bytes(window) == FRAME_MARKER_BYTES:
                print(f"[Reader] aligned after {n_scanned} bytes")
                return
            if time.monotonic() > deadline:
                raise ConnectionError(
                    f"alignment timed out after scanning {n_scanned} bytes"
                )
        raise ConnectionError("alignment interrupted")

    def _compute_v0(self) -> np.ndarray:
        """Read ~Fs/4 samples (verified), replace outliers, average per channel."""
        n = max(1, int(round(self._fs / 4.0)))
        buf = self._read_verified_block(n)
        cleaned = fill_outliers_linear(buf)
        return cleaned.mean(axis=0).astype(np.float32)

    # ----- frame reading

    def _read_verified_block(self, n: int, max_realigns: int = 5) -> np.ndarray:
        """Read ``n`` frames with marker verification; re-align on failure.

        Used by V0 computation at startup. Guards against ``_align()`` having
        locked onto a false marker — without this, a single chance match in
        the float data would poison V0 (and downstream FFT etc.) for the
        whole session.
        """
        sample_bytes = self._nch * 4
        frame_size = frame_size_for(self._nch)
        for attempt in range(max_realigns + 1):
            raw = self._transport.read_exact(n * frame_size)
            if self._verify_markers(raw, n, sample_bytes, frame_size):
                return self._parse_frames(raw, n)
            if attempt < max_realigns:
                print(
                    f"[Reader] startup block of {n} frames misaligned "
                    f"(attempt {attempt + 1}); re-aligning"
                )
                self._align()
        raise ConnectionError(
            f"failed to verify a {n}-frame block after {max_realigns} re-aligns"
        )

    def _parse_frames(self, raw: bytes, n: int) -> np.ndarray:
        """Parse ``n`` consecutive frames from ``raw``. Returns (n, nch) float32.

        Each frame is ``[nch * float32 LE][CR LF][float32 marker]`` — we just
        slice the float region and reinterpret as little-endian float32.
        """
        sample_bytes = self._nch * 4
        frame_size = frame_size_for(self._nch)
        if len(raw) != n * frame_size:
            raise ValueError(
                f"frame parse: expected {n * frame_size} bytes, got {len(raw)}"
            )
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(n, frame_size)
        # Take only the float-byte region of each frame; contiguous copy then view.
        float_bytes = np.ascontiguousarray(arr[:, :sample_bytes]).tobytes()
        return np.frombuffer(float_bytes, dtype="<f4").reshape(n, self._nch).copy()

    def _stream_loop(self) -> None:
        ns_read = self._ns_read
        sample_bytes = self._nch * 4
        frame_size = frame_size_for(self._nch)
        frame_total = ns_read * frame_size
        misaligns = 0
        while not self._stop.is_set():
            if self._reset_pending.is_set():
                self._reset_pending.clear()
                self._do_reset()
                continue
            raw = self._transport.read_exact(frame_total)
            if not self._verify_markers(raw, ns_read, sample_bytes, frame_size):
                misaligns += 1
                print(
                    f"[Reader] frame misalignment #{misaligns} "
                    f"(after {self._total_samples} samples); re-aligning"
                )
                self._align()
                continue
            samples = self._parse_frames(raw, ns_read)
            self._bus.publish(samples)
            self._total_samples += ns_read
            self._maybe_emit_stats(samples)

    def _do_reset(self) -> None:
        """Recompute V0 on demand. Called between batches on the DAQ thread."""
        print("[Reader] reset requested; recomputing V0")
        v0 = self._compute_v0()
        # Bump v0_version so subscribers can detect the refresh without
        # bytewise-comparing the array every tick.
        meta = self._bus.get_metadata()
        version = int(meta.get("v0_version", 0)) + 1
        self._bus.set_metadata(V0=v0, v0_version=version)
        print(
            f"[Reader] new V0 (v{version}): "
            + " ".join(f"{v:7.3f}" for v in v0)
        )
        # Reset the rate-measurement window so the next stats line doesn't
        # show a spurious "Fs avg" drop caused by the V0 read pause.
        now = time.perf_counter()
        self._window_start = now
        self._window_samples_at_start = self._total_samples

    @staticmethod
    def _verify_markers(raw: bytes, n: int, sample_bytes: int, frame_size: int) -> bool:
        """Confirm every frame in the batch ends with the 2024.0 marker.

        Cheap sanity check that catches the rare case where ``_align`` locked
        onto a false marker (the marker bytes appearing inside the float data
        by chance), or where bytes were dropped mid-stream. If any frame
        fails, the caller re-runs ``_align`` and the batch is discarded.
        """
        marker_offset = sample_bytes + 2  # skip floats + CR LF
        for i in range(n):
            start = i * frame_size + marker_offset
            if raw[start:start + 4] != FRAME_MARKER_BYTES:
                return False
        return True

    # ----- diagnostics

    def _maybe_emit_stats(self, last_batch: np.ndarray) -> None:
        if self._window_start is None or self._run_start is None:
            return
        now = time.perf_counter()
        if (now - self._window_start) < self._stats_interval_s:
            return

        elapsed_total = now - self._run_start
        avg_fs = self._total_samples / elapsed_total if elapsed_total > 0 else 0.0
        window_elapsed = now - self._window_start
        window_n = self._total_samples - self._window_samples_at_start
        window_fs = window_n / window_elapsed if window_elapsed > 0 else avg_fs

        print(
            f"[Reader] {self._total_samples:>8d} samples | "
            f"Fs avg {avg_fs:6.1f} Hz, window {window_fs:6.1f} Hz "
            f"(target {self._fs:.1f}) | last "
            + " ".join(f"{v:7.3f}" for v in last_batch[-1])
        )

        self._window_start = now
        self._window_samples_at_start = self._total_samples
