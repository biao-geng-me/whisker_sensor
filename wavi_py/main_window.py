"""GUI subscriber — the control panel + the visualization pipeline.

This is one subscriber on the SampleBus among many. It owns:

- The control panel (transport selector, record, pause, view toggles, tag, auto-stop).
- The three view windows (line, fft, spectrogram).
- The local processing chain: outlier removal → recording → (optional) filter → plot updates.

External agent loops can subscribe to ``WaviClient.bus`` independently without
touching this module. The GUI's slow plot redraws can never starve them — they
poll the bus on their own thread.
"""

from __future__ import annotations

import os
from datetime import datetime, timedelta
from pathlib import Path
from typing import Optional

import numpy as np
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from scipy.signal import detrend

from wavi_py.client import WaviClient
from wavi_py.config import (
    DEFAULT_FS_HZ,
    DEFAULT_NS_READ,
    DEFAULT_NSENSOR,
    DEFAULT_T_BUFFER_S,
    DEFAULT_T_FFT_S,
    DEFAULT_T_SPEC_S,
    DEFAULT_TCP_HOST,
    DEFAULT_TCP_PORT,
    MIN_REC_LEN_S,
)
from wavi_py.daq.outliers import fill_outliers_linear
from wavi_py.io.dat_writer import DatWriter, build_datalog_filename
from wavi_py.views import FftView, LineView, SpectrogramView


def _default_outpath() -> Path:
    home = os.environ.get("USERPROFILE") or os.environ.get("HOME") or "."
    p = Path(home) / "wavi_data"
    p.mkdir(parents=True, exist_ok=True)
    return p


class MainWindow(QMainWindow):
    """The wavi GUI control panel."""

    def __init__(
        self,
        *,
        transport: str = "tcp",
        host: str = DEFAULT_TCP_HOST,
        port: int = DEFAULT_TCP_PORT,
        serial_port: Optional[str] = None,
        nsensor: int = DEFAULT_NSENSOR,
        fs: float = DEFAULT_FS_HZ,
        ns_read: int = DEFAULT_NS_READ,
        t_fft_s: float = DEFAULT_T_FFT_S,
        t_spec_s: float = DEFAULT_T_SPEC_S,
        t_buffer_s: float = DEFAULT_T_BUFFER_S,
        scale: float = 1.0,
        outpath: Optional[Path] = None,
    ):
        super().__init__()
        self.setWindowTitle("Wavi (Python)")
        self.resize(560, 320)

        # acquisition params
        self._initial_transport = transport
        self._initial_host = host
        self._initial_port = int(port)
        self._initial_serial_port = serial_port
        self._nsensor = int(nsensor)
        self._nch = self._nsensor * 2
        self._fs = float(fs)
        self._ns_read = int(ns_read)
        self._scale = float(scale)
        self._t_fft_s = float(t_fft_s)
        self._t_spec_s = float(t_spec_s)
        self._t_buffer_s = float(t_buffer_s)

        # derived sizes
        self._n_fft_window = max(2, int(round(self._fs * self._t_fft_s)))
        self._nfreq = self._n_fft_window // 2 + 1
        self._ns_tot = max(self._n_fft_window, int(self._t_buffer_s * self._fs))
        self._ns_spec = self._ns_read
        self._spec_cols = max(1, int(self._t_spec_s * self._fs / self._ns_spec))

        # rolling buffers (filled by timer)
        self._sig = np.full((self._ns_tot, self._nch), np.nan, dtype=np.float32)
        self._t_axis = np.arange(self._ns_tot, dtype=np.float64) / self._fs  # seconds since stream start
        self._v0 = np.zeros(self._nch, dtype=np.float32)
        self._spec_data = np.zeros((self._nfreq * self._nch, self._spec_cols), dtype=np.float32)
        self._fft_map = np.zeros((self._nfreq, self._nch), dtype=np.float32)

        # subscriber cursor + counters
        self._last_seq = 0
        self._samples_since_spec_update = 0
        self._last_v0_version = 0

        # recording state
        self._outpath: Path = outpath if outpath else _default_outpath()
        self._is_recording = False
        self._is_paused = False
        self._dat_writer: Optional[DatWriter] = None
        self._record_start_dt: Optional[datetime] = None
        self._auto_stop_s = 0.0
        self._t0_unix: Optional[float] = None    # stream start in unix time

        # client (None until Connect pressed)
        self._client: Optional[WaviClient] = None

        # build UI before constructing views (views are created lazily — only on first toggle)
        self._build_ui()

        # construct views up front but hidden, mirroring MATLAB behavior
        self._line_view = LineView(self._nsensor, self._nch, self._t_buffer_s)
        self._fft_view = FftView(self._nch, self._nfreq, self._t_fft_s)
        self._spec_view = SpectrogramView(
            self._nch, self._nfreq, self._fs, self._ns_spec, self._t_spec_s
        )
        for v in (self._line_view, self._fft_view, self._spec_view):
            v.hide()

        # main-thread timer that drains the bus
        self._timer = QTimer(self)
        self._timer.setInterval(50)  # 20 Hz UI cadence
        self._timer.timeout.connect(self._on_tick)

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        central = QWidget()
        outer = QVBoxLayout(central)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(6)
        self.setCentralWidget(central)

        outer.addWidget(self._build_transport_group())
        outer.addWidget(self._build_view_buttons())
        outer.addWidget(self._build_record_row())
        outer.addWidget(self._build_status_row())

    def _build_transport_group(self) -> QGroupBox:
        box = QGroupBox("Sensor array")
        h = QHBoxLayout(box)

        self._transport_combo = QComboBox()
        self._transport_combo.addItems(["TCP", "Serial"])
        self._transport_combo.setCurrentText(self._initial_transport.upper())
        self._transport_combo.currentTextChanged.connect(self._on_transport_changed)

        self._host_edit = QLineEdit(self._initial_host)
        self._host_edit.setPlaceholderText("host")

        self._port_spin = QSpinBox()
        self._port_spin.setRange(1, 65535)
        self._port_spin.setValue(self._initial_port)

        self._serial_port_edit = QLineEdit(self._initial_serial_port or "")
        self._serial_port_edit.setPlaceholderText("COM3 or /dev/ttyACM0")

        self._connect_btn = QPushButton("Connect")
        self._connect_btn.clicked.connect(self._on_connect_clicked)

        h.addWidget(QLabel("Transport:"))
        h.addWidget(self._transport_combo)
        h.addWidget(QLabel("Host:"))
        h.addWidget(self._host_edit, stretch=2)
        h.addWidget(QLabel("Port:"))
        h.addWidget(self._port_spin)
        h.addWidget(QLabel("Serial:"))
        h.addWidget(self._serial_port_edit, stretch=1)
        h.addWidget(self._connect_btn)

        self._on_transport_changed(self._initial_transport.upper())
        return box

    def _build_view_buttons(self) -> QGroupBox:
        box = QGroupBox("Views")
        h = QHBoxLayout(box)

        self._toggle_line = QPushButton("Line")
        self._toggle_line.setCheckable(True)
        self._toggle_line.toggled.connect(lambda on: self._toggle_window(self._line_view, on))

        self._toggle_fft = QPushButton("FFT")
        self._toggle_fft.setCheckable(True)
        self._toggle_fft.toggled.connect(lambda on: self._toggle_window(self._fft_view, on))

        self._toggle_spec = QPushButton("Spec")
        self._toggle_spec.setCheckable(True)
        self._toggle_spec.toggled.connect(lambda on: self._toggle_window(self._spec_view, on))

        self._pause_btn = QPushButton("Pause")
        self._pause_btn.setCheckable(True)
        self._pause_btn.toggled.connect(self._on_pause_toggled)
        self._pause_btn.setEnabled(False)

        self._reset_btn = QPushButton("Reset")
        self._reset_btn.setToolTip("Clear the plots and recompute the V0 offset")
        self._reset_btn.clicked.connect(self._on_reset_clicked)
        self._reset_btn.setEnabled(False)

        h.addWidget(self._toggle_line)
        h.addWidget(self._toggle_fft)
        h.addWidget(self._toggle_spec)
        h.addStretch(1)
        h.addWidget(self._pause_btn)
        h.addWidget(self._reset_btn)
        return box

    def _build_record_row(self) -> QGroupBox:
        box = QGroupBox("Record")
        h = QHBoxLayout(box)

        self._tag_edit = QLineEdit()
        self._tag_edit.setPlaceholderText("tag (alphanumerics, _ -)")

        self._auto_stop_spin = QDoubleSpinBox()
        self._auto_stop_spin.setRange(0.0, 36000.0)
        self._auto_stop_spin.setSuffix(" s")
        self._auto_stop_spin.setSpecialValueText("off")
        self._auto_stop_spin.setDecimals(1)
        self._auto_stop_spin.setSingleStep(5.0)
        self._auto_stop_spin.valueChanged.connect(self._on_auto_stop_changed)

        self._record_btn = QPushButton("● Rec")
        self._record_btn.setStyleSheet("QPushButton { color: #c00; font-weight: bold; }")
        self._record_btn.clicked.connect(self._on_record_clicked)
        self._record_btn.setEnabled(False)

        h.addWidget(QLabel("Tag:"))
        h.addWidget(self._tag_edit, stretch=2)
        h.addWidget(QLabel("Auto-stop:"))
        h.addWidget(self._auto_stop_spin)
        h.addWidget(self._record_btn)
        return box

    def _build_status_row(self) -> QGroupBox:
        box = QGroupBox("Status")
        v = QVBoxLayout(box)

        self._progress = QProgressBar()
        self._progress.setRange(0, 1000)
        self._progress.setValue(0)
        self._progress.setTextVisible(False)
        self._progress.setMaximumHeight(8)

        self._path_label = QLabel(str(self._outpath))
        self._path_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self._path_label.setStyleSheet("QLabel { color: #555; }")

        v.addWidget(self._progress)
        v.addWidget(self._path_label)
        return box

    # ------------------------------------------------------------------
    # callbacks: connection
    # ------------------------------------------------------------------

    def _on_transport_changed(self, kind: str) -> None:
        is_tcp = kind.upper() == "TCP"
        self._host_edit.setEnabled(is_tcp)
        self._port_spin.setEnabled(is_tcp)
        self._serial_port_edit.setEnabled(not is_tcp)

    def _on_connect_clicked(self) -> None:
        if self._client is not None and self._client.is_running:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        try:
            kind = self._transport_combo.currentText().lower()
            client = WaviClient(
                transport=kind,
                host=self._host_edit.text().strip() or DEFAULT_TCP_HOST,
                port=self._port_spin.value(),
                serial_port=(self._serial_port_edit.text().strip() or None),
                nsensor=self._nsensor,
                fs=self._fs,
                ns_read=self._ns_read,
                t_buffer_s=self._t_buffer_s,
            )
            client.start()
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "Connect failed", str(exc))
            return

        self._client = client
        self._last_seq = 0
        self._samples_since_spec_update = 0
        self._sig.fill(np.nan)
        self._spec_data.fill(0)
        self._fft_map.fill(0)
        # Force the metadata-latching path in _on_tick to re-run; otherwise
        # the new bus's V0 is ignored and FFT/plots use stale offsets.
        self._v0 = np.zeros(self._nch, dtype=np.float32)
        self._t0_unix = None
        self._last_v0_version = 0

        self._connect_btn.setText("Disconnect")
        self._pause_btn.setEnabled(True)
        self._reset_btn.setEnabled(True)
        self._record_btn.setEnabled(True)
        self._set_transport_inputs_enabled(False)
        self._path_label.setText(f"Connected to {client.endpoint_str} — output dir: {self._outpath}")
        self._timer.start()

    def _disconnect(self) -> None:
        if self._is_recording:
            self._stop_recording()
        self._timer.stop()
        if self._client is not None:
            self._client.stop()
            self._client = None

        self._connect_btn.setText("Connect")
        self._pause_btn.setEnabled(False)
        self._pause_btn.setChecked(False)
        self._reset_btn.setEnabled(False)
        self._record_btn.setEnabled(False)
        self._set_transport_inputs_enabled(True)
        self._path_label.setText(f"Disconnected. Output dir: {self._outpath}")
        self._progress.setValue(0)

    def _set_transport_inputs_enabled(self, enabled: bool) -> None:
        self._transport_combo.setEnabled(enabled)
        kind = self._transport_combo.currentText().upper()
        self._host_edit.setEnabled(enabled and kind == "TCP")
        self._port_spin.setEnabled(enabled and kind == "TCP")
        self._serial_port_edit.setEnabled(enabled and kind != "TCP")

    # ------------------------------------------------------------------
    # callbacks: view toggle / pause / record
    # ------------------------------------------------------------------

    def _toggle_window(self, win: QMainWindow, show: bool) -> None:
        if show:
            win.show()
            win.raise_()
        else:
            win.hide()

    def _on_pause_toggled(self, on: bool) -> None:
        self._is_paused = on
        self._pause_btn.setText("Resume" if on else "Pause")

    def _on_reset_clicked(self) -> None:
        if self._client is None or not self._client.is_running:
            return
        self._client.reset()  # DAQ thread will recompute V0 on its next tick
        # Skip whatever is already buffered on the bus — those samples were
        # collected under the old V0 and would briefly render with a jump
        # before the new V0 lands.
        self._last_seq = self._client.bus.total_published
        self._sig.fill(np.nan)
        self._spec_data.fill(0)
        self._fft_map.fill(0)
        self._samples_since_spec_update = 0
        self._path_label.setText("Resetting V0...")

    def _on_auto_stop_changed(self, v: float) -> None:
        if 0 < v < MIN_REC_LEN_S:
            self._auto_stop_spin.blockSignals(True)
            self._auto_stop_spin.setValue(MIN_REC_LEN_S)
            self._auto_stop_spin.blockSignals(False)
            v = MIN_REC_LEN_S
        self._auto_stop_s = float(v)

    def _on_record_clicked(self) -> None:
        if self._is_recording:
            self._stop_recording()
        else:
            self._start_recording()

    def _start_recording(self) -> None:
        if self._client is None:
            return
        tag = self._tag_edit.text().strip()
        start_dt = datetime.now()
        fname = build_datalog_filename(start_dt, tag)
        fullpath = self._outpath / fname
        try:
            self._dat_writer = DatWriter(fullpath, self._nch)
        except OSError as exc:
            QMessageBox.critical(self, "Record failed", f"{exc}")
            return
        self._record_start_dt = start_dt
        self._is_recording = True
        self._client.send_record_start()

        self._record_btn.setText("■ Stop")
        self._record_btn.setStyleSheet(
            "QPushButton { background-color: #c00; color: white; font-weight: bold; }"
        )
        self._path_label.setText(f"Recording → {fullpath}")
        self._progress.setValue(0)

    def _stop_recording(self) -> None:
        self._is_recording = False
        if self._client is not None:
            self._client.send_record_stop()
        if self._dat_writer is not None:
            try:
                self._dat_writer.close()
            finally:
                p = self._dat_writer.path
                self._dat_writer = None
                self._path_label.setText(f"Saved {p}")
        self._record_btn.setText("● Rec")
        self._record_btn.setStyleSheet("QPushButton { color: #c00; font-weight: bold; }")
        self._progress.setValue(0)

    # ------------------------------------------------------------------
    # the data tick
    # ------------------------------------------------------------------

    def _on_tick(self) -> None:
        if self._client is None or not self._client.is_running:
            return

        samples, new_seq = self._client.bus.read_since(self._last_seq, block=False)
        if samples is None:
            return
        n_new_published = new_seq - self._last_seq
        self._last_seq = new_seq

        # Sync bus metadata.
        #   t0_unix latches once per connection.
        #   V0 can refresh on Reset; v0_version lets us cheaply detect that.
        meta = self._client.bus.get_metadata()
        if self._t0_unix is None and "t0_unix" in meta:
            self._t0_unix = float(meta["t0_unix"])
        v0_version = int(meta.get("v0_version", 0))
        if v0_version != self._last_v0_version and "V0" in meta:
            self._v0 = np.asarray(meta["V0"], dtype=np.float32)
            self._last_v0_version = v0_version
            if v0_version > 1:
                self._path_label.setText(f"Reset complete (V0 v{v0_version}).")

        n_new = samples.shape[0]
        if n_new_published > n_new:
            # caller fell behind — log it once in a while
            print(f"[MainWindow] bus dropped {n_new_published - n_new} samples")

        # Outlier removal on the trailing block, mirroring wavi.m's remove_outliers.
        cleaned = fill_outliers_linear(samples)

        # Shift the rolling buffer and append cleaned samples.
        if n_new >= self._ns_tot:
            self._sig[:] = cleaned[-self._ns_tot:]
        else:
            self._sig[:-n_new] = self._sig[n_new:]
            self._sig[-n_new:] = cleaned

        # Recording path: write the cleaned samples with their wall-clock timestamps.
        if self._is_recording and self._dat_writer is not None:
            self._write_record(cleaned, new_seq)
            self._maybe_auto_stop()

        # Time axis: most recent sample is at t = (new_seq - 1)/fs since stream start.
        # We render the whole buffer with x = (i - (ns_tot-1)) / fs + latest_time.
        latest_t = (new_seq - 1) / self._fs
        t_axis = latest_t + np.arange(-(self._ns_tot - 1), 1) / self._fs

        # Plot updates (skipped while paused, but data still buffered + recorded).
        if not self._is_paused:
            self._line_view.update_view(t_axis, self._sig, self._v0, self._scale)

            # FFT / spectrogram: update every batch (mirrors n_update=1 default).
            self._samples_since_spec_update += n_new
            if self._samples_since_spec_update >= self._ns_spec:
                self._update_fft_and_spec()
                self._samples_since_spec_update = 0

    def _update_fft_and_spec(self) -> None:
        """Recompute the current FFT (matches ``do_fft`` in wavi.m:901-914)."""
        window = self._sig[-self._n_fft_window:]
        if not np.isfinite(window).all():
            return

        x = detrend(window - self._v0, axis=0)
        Y = np.fft.rfft(x, axis=0)
        amp = np.abs(Y) / window.shape[0]
        if amp.shape[0] > 2:
            amp[1:-1] *= 2
        self._fft_map = amp.astype(np.float32)

        # Spectrogram: shift columns left, drop new frame at right edge.
        self._spec_data[:, :-1] = self._spec_data[:, 1:]
        self._spec_data[:, -1] = self._fft_map.reshape(-1, order="F")

        self._fft_view.update_view(self._fft_map, self._fs)
        self._spec_view.update_view(self._spec_data)

    def _write_record(self, cleaned: np.ndarray, new_seq: int) -> None:
        assert self._dat_writer is not None
        n = cleaned.shape[0]
        if n == 0 or self._t0_unix is None:
            return
        first_idx = new_seq - n
        timestamps = [
            datetime.fromtimestamp(self._t0_unix + (first_idx + i) / self._fs)
            for i in range(n)
        ]
        self._dat_writer.write_batch(timestamps, cleaned)

    def _maybe_auto_stop(self) -> None:
        if self._auto_stop_s <= 0 or self._record_start_dt is None:
            return
        elapsed = (datetime.now() - self._record_start_dt).total_seconds()
        frac = min(1.0, elapsed / self._auto_stop_s)
        self._progress.setValue(int(round(frac * 1000)))
        if elapsed >= self._auto_stop_s:
            print(f"[MainWindow] auto-stop after {elapsed:.1f}s")
            self._stop_recording()

    # ------------------------------------------------------------------
    # window lifecycle
    # ------------------------------------------------------------------

    def closeEvent(self, event) -> None:  # noqa: N802 — Qt naming
        try:
            self._disconnect()
        finally:
            for v in (self._line_view, self._fft_view, self._spec_view):
                v.close()
        super().closeEvent(event)
