"""GUI subscriber — the control panel + the visualization pipeline.

This is one subscriber on the SampleBus among many. It owns:

- The control panel (transport selector, record, pause, view toggles, tag, auto-stop).
- The view windows (line, spectrum, fft, spectrogram, imu).
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
from PyQt6.QtCore import QRegularExpression, QTimer, Qt
from PyQt6.QtGui import QRegularExpressionValidator
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
    QSizePolicy,
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
from wavi_py.scan_mode import (DEFAULT_SELECTS, PRESETS, ModeKeeper, ScanRequest, describe_cfg,
                               parse_selects)
from wavi_py.transports.sim_transport import SimSignals
from wavi_py.views import FftView, ImuView, LineView, SpectrogramView, SpectrumView
from wavi_py.views.base import alarm_label, show_alarm
from wavi_py.views.fault_log import FaultLogView
from wavi_py.views.sim_panel import SimPanel
from wavi_py.views.spectral_base import FFT_WINDOWS_S
from whisker_link.channels import SAT_CODE
from whisker_link.esp32_frame import MV_PER_CODE, info_cfg


LEGACY_NOMINAL_HZ = 76.0     # the legacy scan with 7 selects; the measured rate replaces it


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
        auto_fs: bool = True,
        ns_read: int = DEFAULT_NS_READ,
        t_fft_s: float = DEFAULT_T_FFT_S,
        t_spec_s: float = DEFAULT_T_SPEC_S,
        t_buffer_s: float = DEFAULT_T_BUFFER_S,
        scale: float = 1.0,
        outpath: Optional[Path] = None,
        scan_mode: Optional[tuple[int, ...]] = None,
        selects: str = DEFAULT_SELECTS,
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
        # Server transport with auto_fs: fs is only a starting guess, replaced
        # by the rate measured from the ESP32's scan clock (fs_measured).
        self._auto_fs = bool(auto_fs)
        self._ns_read = int(ns_read)
        self._scale = float(scale)
        self._t_fft_s = float(t_fft_s)
        self._t_spec_s = float(t_spec_s)
        self._t_buffer_s = float(t_buffer_s)
        self._ns_spec = self._ns_read
        self._v0 = np.zeros(self._nch, dtype=np.float32)

        # subscriber cursor + counters
        self._last_seq = 0
        self._last_v0_version = 0
        # Time of sample index i since the stream start: anchor_t + (i - anchor_idx) / fs.
        # A rate change moves the anchor, so the time axis and the .dat timestamps
        # stay continuous across a scan-mode switch.
        self._anchor_idx = 0
        self._anchor_t = 0.0
        self._set_rate(float(fs))   # derived sizes + rolling buffers

        # ESP32 scan mode: what the user picked (kept and re-applied by the keeper),
        # the INFO frames seen, and rate switches waiting for their sample index
        self._keeper = ModeKeeper()
        if scan_mode is not None:
            self._keeper.set_wanted(ScanRequest(tuple(scan_mode), selects), connected=False)
        self._initial_scan_mode = scan_mode
        self._initial_selects = selects
        self._esp32_cfg: Optional[dict] = None
        self._cfg_key: Optional[tuple] = None
        self._info_seen = 0
        self._txt_seen = 0
        self._pending_switches: list[tuple[int, float]] = []    # (sample index, fs)
        self._legacy_fs: dict[int, float] = {}                  # measured, per select mask
        self._last_status_rec: dict = {}

        # ESP32 / ADC faults (whisker_link/esp32_fault.py, run by the transport or
        # daq_server): the current state from the status record, the events for
        # the session's log, and a <recording>.faults.txt next to the .dat
        self._fault_state: dict = {}
        self._fault_marks_seen = 0
        self._n_fault_events = 0
        self._fault_log = FaultLogView()
        self._fault_sidecar = None
        self._fault_sidecar_path: Optional[Path] = None

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
        # the Sim transport's signals: kept across connects, edited in the signal panel
        self._sim = SimSignals()
        self._sim_panel = SimPanel(self._sim)

        # build UI before constructing views (views are created lazily — only on first toggle)
        self._build_ui()

        # construct views up front but hidden, mirroring MATLAB behavior
        self._line_view = LineView(self._nsensor, self._nch, self._t_buffer_s)
        self._spectrum_view = SpectrumView(self._nsensor, self._nch)
        self._fft_view = FftView(self._nch, self._nfreq, self._t_fft_s)
        self._spec_view = SpectrogramView(
            self._nch, self._nfreq, self._fs, self._ns_spec, self._t_spec_s, self._t_fft_s
        )
        self._imu_view = ImuView(self._t_buffer_s)
        for v, btn in ((self._line_view, self._toggle_line), (self._spectrum_view, self._toggle_spectrum),
                       (self._fft_view, self._toggle_fft),
                       (self._spec_view, self._toggle_spec), (self._imu_view, self._toggle_imu)):
            v.hide()
            # closing a view window unchecks its button (setChecked is a no-op if already off)
            v.closed.connect(lambda b=btn: b.setChecked(False))
        # the plot windows' Pause (and Re-zero) drive the panel's buttons (state flows
        # back through _on_pause_toggled / _set_stream_buttons_enabled); they all show
        # the stream status line
        self._stream_views = (self._line_view, self._spectrum_view, self._fft_view, self._spec_view)
        for v in self._stream_views:
            v.pauseToggled.connect(self._pause_btn.setChecked)
        for v in (self._line_view, self._spectrum_view):
            v.resetRequested.connect(self._on_reset_clicked)
        # FFT window length (shared by FFT and Spec) and the spectrogram span live here
        for v in (self._fft_view, self._spec_view):
            v.fftWindowChanged.connect(self._set_fft_window)
        self._spec_view.spanChanged.connect(self._set_spec_span)
        # IMU rolling buffer (server transport only)
        self._imu_last_seq = 0
        self._imu_rows = np.empty((0, 9), dtype=np.float32)
        self._last_status_t = 0.0

        # main-thread timer that drains the bus
        self._timer = QTimer(self)
        self._timer.setInterval(50)  # 20 Hz UI cadence
        self._timer.timeout.connect(self._on_tick)

    def _set_rate(self, fs: float) -> None:
        """Size the rolling buffers for sample rate ``fs`` and clear them."""
        self._fs = float(fs)
        self._ns_tot = max(int(round(self._fs * FFT_WINDOWS_S[-1])), int(self._t_buffer_s * self._fs))
        self._sig = np.full((self._ns_tot, self._nch), np.nan, dtype=np.float32)
        self._size_spectral()
        # Track how many real samples are sitting in `_sig` since the last
        # connect/reset. Used to feed only the populated portion to the line
        # view; otherwise pyqtgraph's auto-range latches onto the full t-axis
        # the moment any sample lands and gaps the NaN-filled leading slots.
        self._samples_since_clear = 0

    def _size_spectral(self) -> None:
        """Size (and clear) the FFT frame and the spectrogram history."""
        self._n_fft_window = max(2, int(round(self._fs * self._t_fft_s)))
        self._nfreq = self._n_fft_window // 2 + 1
        self._spec_cols = max(1, int(self._t_spec_s * self._fs / self._ns_spec))
        self._spec_data = np.zeros((self._nfreq * self._nch, self._spec_cols), dtype=np.float32)
        self._fft_map = np.zeros((self._nfreq, self._nch), dtype=np.float32)
        self._samples_since_spec_update = 0
        for v in (getattr(self, "_fft_view", None), getattr(self, "_spec_view", None)):
            if v is not None:
                v.set_nfreq(self._nfreq)

    def _set_fft_window(self, secs: float) -> None:
        """FFT window length of the FFT and Spec windows (from either one)."""
        self._t_fft_s = float(secs)
        self._size_spectral()
        for v in (self._fft_view, self._spec_view):
            v.set_fft_window(self._t_fft_s)

    def _set_spec_span(self, secs: float) -> None:
        """Spectrogram history length; keeps the newest columns."""
        self._t_spec_s = float(secs)
        cols = max(1, int(self._t_spec_s * self._fs / self._ns_spec))
        new = np.zeros((self._spec_data.shape[0], cols), dtype=np.float32)
        keep = min(cols, self._spec_cols)
        new[:, -keep:] = self._spec_data[:, -keep:]
        self._spec_data, self._spec_cols = new, cols
        self._spec_view.set_span(self._t_spec_s)

    def _t_of(self, idx: int) -> float:
        """Seconds since the stream start of sample index ``idx``."""
        return self._anchor_t + (idx - self._anchor_idx) / self._fs

    def _switch_rate(self, fs: float, at: int) -> None:
        """From sample index ``at`` on the rate is ``fs``: move the time anchor
        there and start the buffers over (no window mixes two rates)."""
        at = max(at, self._anchor_idx)
        self._anchor_t = self._t_of(at)
        self._anchor_idx = at
        self._set_rate(fs)

    def _maybe_apply_measured_rate(self, fs_measured: Optional[float]) -> None:
        """Adopt the measured whisker rate when it differs by more than 1%
        (also while recording: the time anchor keeps the .dat timestamps continuous)."""
        if not self._auto_fs or not fs_measured or self._pending_switches:
            return
        if self._esp32_cfg is not None and self._esp32_cfg.get("mode") == 0:
            self._legacy_fs[self._esp32_cfg.get("sel_mask", 0xFF)] = fs_measured
        if abs(fs_measured - self._fs) <= 0.01 * self._fs:
            return
        self._switch_rate(fs_measured, self._last_seq)
        print(f"[MainWindow] whisker rate measured from the ESP32 clock: {fs_measured:.2f} Hz")

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        central = QWidget()
        outer = QVBoxLayout(central)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(6)
        self.setCentralWidget(central)

        self._fault_banner = alarm_label()
        outer.addWidget(self._fault_banner)
        outer.addWidget(self._build_transport_group())
        outer.addWidget(self._build_view_buttons())
        outer.addWidget(self._build_record_row())
        outer.addWidget(self._build_status_row())

    def _build_transport_group(self) -> QGroupBox:
        box = QGroupBox("Sensor array")
        rows = QVBoxLayout(box)
        h = QHBoxLayout()
        rows.addLayout(h)
        rows.addLayout(self._build_scan_mode_row())

        self._transport_combo = QComboBox()
        self._transport_combo.addItems(["Server", "ESP32", "Sim", "TCP", "Serial"])
        names = [self._transport_combo.itemText(i).lower()
                 for i in range(self._transport_combo.count())]
        if self._initial_transport.lower() in names:
            self._transport_combo.setCurrentIndex(names.index(self._initial_transport.lower()))
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

        self._sim_btn = QPushButton("Signals")
        self._sim_btn.setCheckable(True)
        self._sim_btn.setToolTip("Sim transport: the simulated signals (gauges, taps, faults)")
        self._sim_btn.toggled.connect(lambda on: self._toggle_window(self._sim_panel, on))
        self._sim_panel.closed.connect(lambda: self._sim_btn.setChecked(False))

        h.addWidget(QLabel("Transport:"))
        h.addWidget(self._transport_combo)
        h.addWidget(QLabel("Host:"))
        h.addWidget(self._host_edit, stretch=2)
        h.addWidget(QLabel("Port:"))
        h.addWidget(self._port_spin)
        h.addWidget(QLabel("Serial:"))
        h.addWidget(self._serial_port_edit, stretch=1)
        h.addWidget(self._connect_btn)
        h.addWidget(self._sim_btn)

        self._on_transport_changed(self._transport_combo.currentText())
        return box

    def _build_scan_mode_row(self) -> QHBoxLayout:
        h = QHBoxLayout()
        self._mode_combo = QComboBox()
        self._mode_combo.addItem("as is (don't set)", None)
        for label, args in PRESETS:
            self._mode_combo.addItem(label, args)
        init = self._initial_scan_mode
        if init is not None:
            i = self._mode_combo.findData(tuple(init))
            if i < 0:
                self._mode_combo.addItem(ScanRequest(tuple(init)).label, tuple(init))
                i = self._mode_combo.count() - 1
            self._mode_combo.setCurrentIndex(i)
        self._mode_combo.setToolTip(
            "ESP32 scan mode, sent as S<selects>;M<mode>;B (Server, ESP32 and Sim transports).\n"
            "The ESP32 boots in legacy and forgets the mode on reboot: the mode shown next to\n"
            "this is what its INFO reports, and if it comes back in another mode the choice is\n"
            "re-sent once. 'as is' sends nothing. Wide 250 Hz overruns on the current\n"
            "single-core firmware.")
        self._mode_combo.activated.connect(lambda _i: self._on_scan_mode_changed(force=True))
        self._selects_edit = QLineEdit(self._initial_selects)
        self._selects_edit.setPlaceholderText("all 8")
        self._selects_edit.setMaxLength(8)
        self._selects_edit.setFixedWidth(80)
        self._selects_edit.setValidator(QRegularExpressionValidator(QRegularExpression("[0-7]{0,8}")))
        self._selects_edit.setToolTip("Mux selects the ESP32 scans, sent with the scan mode "
                                      "(empty = all 8; the outputs of a skipped select read open)")
        self._selects_edit.editingFinished.connect(lambda: self._on_scan_mode_changed(force=False))
        self._mode_label = QLabel()
        # long notes are cut at the right instead of widening the window (full text: tooltip)
        self._mode_label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        h.addWidget(QLabel("Scan mode:"))
        h.addWidget(self._mode_combo)
        h.addWidget(QLabel("Selects:"))
        h.addWidget(self._selects_edit)
        h.addWidget(self._mode_label, stretch=1)
        self._show_mode_note()
        return h

    def _scan_request(self) -> Optional[ScanRequest]:
        args = self._mode_combo.currentData()
        if args is None:
            return None
        return ScanRequest(tuple(args), parse_selects(self._selects_edit.text()))

    def _on_scan_mode_changed(self, force: bool) -> None:
        """The user picked a mode (``force``: send even if it is the same, to retry)
        or edited the selects."""
        req = self._scan_request()
        if not force and req == self._keeper.want:
            return
        connected = (self._client is not None and self._client.is_running
                     and self._client.is_event_stream)
        self._send_esp32(self._keeper.set_wanted(req, connected))
        self._show_mode_note()

    def _send_esp32(self, data: Optional[str]) -> None:
        if data and self._client is not None:
            print(f"[MainWindow] to the ESP32: {data}")
            self._client.send_command({"cmd": "esp32", "data": data})

    def _show_mode_note(self) -> None:
        """The keeper's note (or the reported mode) next to the dropdown and in the
        stream status line."""
        note = self._keeper.note
        if not note and self._esp32_cfg is not None:
            note = f"ESP32 reports {describe_cfg(self._esp32_cfg)}"
        if note == self._mode_label.text():
            return
        self._mode_label.setText(note)
        self._mode_label.setToolTip(note)
        bad = self._keeper.state == "failed"
        self._mode_label.setStyleSheet("QLabel { color: #c00; }" if bad else "QLabel { color: #555; }")
        if self._last_status_rec and self._client is not None:
            self._show_status(self._last_status_rec)

    def _build_view_buttons(self) -> QGroupBox:
        box = QGroupBox("Views")
        h = QHBoxLayout(box)

        self._toggle_line = QPushButton("Line")
        self._toggle_line.setCheckable(True)
        self._toggle_line.toggled.connect(lambda on: self._toggle_window(self._line_view, on))

        self._toggle_spectrum = QPushButton("Spectrum")
        self._toggle_spectrum.setCheckable(True)
        self._toggle_spectrum.setToolTip("Live amplitude spectrum per whisker (Hann window, mean removed)")
        self._toggle_spectrum.toggled.connect(lambda on: self._toggle_window(self._spectrum_view, on))

        self._toggle_fft = QPushButton("FFT")
        self._toggle_fft.setCheckable(True)
        self._toggle_fft.toggled.connect(lambda on: self._toggle_window(self._fft_view, on))

        self._toggle_spec = QPushButton("Spec")
        self._toggle_spec.setCheckable(True)
        self._toggle_spec.toggled.connect(lambda on: self._toggle_window(self._spec_view, on))

        self._toggle_imu = QPushButton("IMU")
        self._toggle_imu.setCheckable(True)
        self._toggle_imu.setToolTip("Vehicle IMU (Server transport only)")
        self._toggle_imu.toggled.connect(lambda on: self._toggle_window(self._imu_view, on))

        self._pause_btn = QPushButton("Pause")
        self._pause_btn.setCheckable(True)
        self._pause_btn.toggled.connect(self._on_pause_toggled)
        self._pause_btn.setEnabled(False)

        self._reset_btn = QPushButton("Reset")
        self._reset_btn.setToolTip("Clear the plots and recompute the V0 offset")
        self._reset_btn.clicked.connect(self._on_reset_clicked)
        self._reset_btn.setEnabled(False)

        h.addWidget(self._toggle_line)
        h.addWidget(self._toggle_spectrum)
        h.addWidget(self._toggle_fft)
        h.addWidget(self._toggle_spec)
        h.addWidget(self._toggle_imu)
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

        self._fault_log_btn = QPushButton("Fault log")
        self._fault_log_btn.setToolTip("ESP32 / ADC faults and warnings this session")
        self._fault_log_btn.clicked.connect(self._show_fault_log)
        h = QHBoxLayout()
        h.addWidget(self._path_label, stretch=1)
        h.addWidget(self._fault_log_btn)
        v.addWidget(self._progress)
        v.addLayout(h)
        return box

    # ------------------------------------------------------------------
    # callbacks: connection
    # ------------------------------------------------------------------

    def _on_transport_changed(self, kind: str) -> None:
        kind = kind.upper()
        self._host_edit.setEnabled(kind in ("TCP", "SERVER"))
        self._port_spin.setEnabled(kind in ("TCP", "SERVER"))
        self._serial_port_edit.setEnabled(kind in ("SERIAL", "ESP32"))
        array = kind in ("SERVER", "ESP32", "SIM")
        self._mode_combo.setEnabled(array)
        self._selects_edit.setEnabled(array)
        self._sim_btn.setEnabled(kind == "SIM")
        if kind != "SIM":
            self._sim_btn.setChecked(False)

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
                sim=self._sim,
            )
            client.start()
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "Connect failed", str(exc))
            return

        self._client = client
        self._last_seq = 0
        self._samples_since_spec_update = 0
        self._samples_since_clear = 0
        self._sig.fill(np.nan)
        self._spec_data.fill(0)
        self._fft_map.fill(0)
        # Force the metadata-latching path in _on_tick to re-run; otherwise
        # the new bus's V0 is ignored and FFT/plots use stale offsets.
        self._v0 = np.zeros(self._nch, dtype=np.float32)
        self._t0_unix = None
        self._last_v0_version = 0
        self._imu_last_seq = 0
        self._imu_rows = np.empty((0, 9), dtype=np.float32)
        self._anchor_idx, self._anchor_t = 0, 0.0
        self._esp32_cfg, self._cfg_key = None, None
        self._info_seen = self._txt_seen = 0
        self._pending_switches = []
        self._last_status_rec = {}
        self._fault_marks_seen = 0
        self._set_fault_state({})
        if client.is_event_stream:
            self._keeper.connected()
        self._show_mode_note()

        self._connect_btn.setText("Disconnect")
        self._set_stream_buttons_enabled(True)
        # server / esp32 samples are mV at the ADC; a railed (open) gauge reads ~ full scale
        if client.is_event_stream:
            self._line_view.set_value_format("mV", SAT_CODE * MV_PER_CODE)
            for v in (self._spectrum_view, self._fft_view, self._spec_view):
                v.set_units("mV")
        else:
            self._line_view.set_value_format("", None)
            for v in (self._spectrum_view, self._fft_view, self._spec_view):
                v.set_units("")
        self._record_btn.setEnabled(True)
        self._set_transport_inputs_enabled(False)
        self._path_label.setText(f"Connected to {client.endpoint_str} — output dir: {self._outpath}")
        for v in self._stream_views:
            v.set_stream_status(f"Connected to {client.endpoint_str}")
        self._last_status_t = 0.0
        self._timer.start()

    def _disconnect(self) -> None:
        if self._is_recording:
            self._stop_recording()
        self._timer.stop()
        if self._client is not None:
            self._client.stop()
            self._client = None

        self._keeper.set_wanted(self._keeper.want, connected=False)
        self._esp32_cfg = None
        self._set_fault_state({})
        self._show_mode_note()
        self._connect_btn.setText("Connect")
        self._pause_btn.setChecked(False)
        self._set_stream_buttons_enabled(False)
        self._record_btn.setEnabled(False)
        self._set_transport_inputs_enabled(True)
        self._path_label.setText(f"Disconnected. Output dir: {self._outpath}")
        for v in self._stream_views:
            v.set_stream_status("Disconnected")
        self._progress.setValue(0)

    def _on_link_lost(self) -> None:
        """The DAQ thread ended on its own (server dropped us, cable, timeout)."""
        reason = (self._client.error if self._client else None) or "stream ended"
        was_recording = self._is_recording
        self._disconnect()
        msg = f"Connection lost: {reason}. Click Connect to reconnect."
        for v in self._stream_views:
            v.set_stream_status(f"Connection lost: {reason}")
        if was_recording:
            msg += " The local recording was saved; the ROV-side recording may still be running."
        self._path_label.setText(msg)

    def _set_transport_inputs_enabled(self, enabled: bool) -> None:
        self._transport_combo.setEnabled(enabled)
        kind = self._transport_combo.currentText().upper()
        self._host_edit.setEnabled(enabled and kind in ("TCP", "SERVER"))
        self._port_spin.setEnabled(enabled and kind in ("TCP", "SERVER"))
        self._serial_port_edit.setEnabled(enabled and kind in ("SERIAL", "ESP32"))

    # ------------------------------------------------------------------
    # callbacks: view toggle / pause / record
    # ------------------------------------------------------------------

    def _toggle_window(self, win: QMainWindow, show: bool) -> None:
        if show:
            win.show()
            win.raise_()
        else:
            win.hide()

    def _set_stream_buttons_enabled(self, on: bool) -> None:
        self._pause_btn.setEnabled(on)
        self._reset_btn.setEnabled(on)
        for v in self._stream_views:
            v.set_stream_controls_enabled(on)

    def _on_pause_toggled(self, on: bool) -> None:
        self._is_paused = on
        self._pause_btn.setText("Resume" if on else "Pause")
        for v in self._stream_views:
            v.set_paused(on)

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
        self._samples_since_clear = 0
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
        # Server transport: also starts the recording on the vehicle, and tees
        # the full whisker+IMU stream to a local .jsonl next to the .dat.
        self._client.send_record_start(tag, local_copy=str(fullpath.with_suffix(".jsonl")))

        self._record_btn.setText("■ Stop")
        self._record_btn.setStyleSheet(
            "QPushButton { background-color: #c00; color: white; font-weight: bold; }"
        )
        self._path_label.setText(f"Recording → {fullpath}")
        self._progress.setValue(0)
        self._fault_sidecar_path = fullpath.with_suffix(".faults.txt")
        f = self._fault_state
        if f.get("level") in ("fault", "warn"):     # from before the recording
            since = datetime.fromtimestamp(f.get("since", 0)).strftime("%H:%M:%S")
            self._write_fault_sidecar(f"at record start: {f['level']} {f.get('code')}:"
                                      f" {f.get('msg')} (since {since})")

    def _stop_recording(self) -> None:
        self._is_recording = False
        if self._fault_sidecar is not None:
            self._fault_sidecar.close()
            self._fault_sidecar = None
        self._fault_sidecar_path = None
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
        if self._client is None:
            return
        if not self._client.is_running:
            self._on_link_lost()
            return
        if self._client.is_event_stream:
            self._tick_server_extras()

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

        # A scan-mode switch inside this block: the samples before it go into the old
        # buffers, then the buffers start over at the new rate.
        first = new_seq - n_new
        pos = 0
        while self._pending_switches and self._pending_switches[0][0] < new_seq:
            at, fs = self._pending_switches.pop(0)
            cut = min(max(at - first, pos), n_new)
            if cut > pos:
                self._ingest(samples[pos:cut], first + pos)
                pos = cut
            self._switch_rate(fs, at)
        if pos < n_new:
            self._ingest(samples[pos:], first + pos)
        if self._is_recording:
            self._maybe_auto_stop()

        # Time axis: the most recent sample is at t_of(new_seq - 1) since the stream start.
        # We render the whole buffer with x = (i - (ns_tot-1)) / fs + latest_time.
        latest_t = self._t_of(new_seq - 1)
        t_axis = latest_t + np.arange(-(self._ns_tot - 1), 1) / self._fs

        # Plot updates (skipped while paused, but data still buffered + recorded).
        if not self._is_paused:
            # Only feed the populated tail to the line view — passing the
            # NaN-prefixed full buffer makes pyqtgraph gap everything after
            # the auto-range has already locked onto the full t-axis.
            # Pin the X range to a fixed-width window ending at latest_t so
            # the time axis doesn't shrink while the buffer is filling.
            n_plot = self._samples_since_clear
            if n_plot > 0:
                self._line_view.update_view(
                    t_axis[-n_plot:],
                    self._sig[-n_plot:],
                    self._v0,
                    self._scale,
                    x_range=(latest_t - self._t_buffer_s, latest_t),
                )

            if self._spectrum_view.isVisible():
                self._update_spectrum()

            # FFT / spectrogram: update every batch (mirrors n_update=1 default).
            if self._samples_since_spec_update >= self._ns_spec:
                self._update_fft_and_spec()
                self._samples_since_spec_update = 0

    def _ingest(self, samples: np.ndarray, first_idx: int) -> None:
        """Samples ``first_idx``... (all at the current rate) into the rolling buffer
        and the recording."""
        n_new = samples.shape[0]
        # Outlier removal on the trailing block, mirroring wavi.m's remove_outliers.
        cleaned = fill_outliers_linear(samples)

        # Shift the rolling buffer and append cleaned samples.
        if n_new >= self._ns_tot:
            self._sig[:] = cleaned[-self._ns_tot:]
        else:
            self._sig[:-n_new] = self._sig[n_new:]
            self._sig[-n_new:] = cleaned
        self._samples_since_clear = min(self._samples_since_clear + n_new, self._ns_tot)
        self._samples_since_spec_update += n_new

        # Recording path: write the cleaned samples with their wall-clock timestamps.
        if self._is_recording and self._dat_writer is not None:
            self._write_record(cleaned, first_idx)

    def _tick_esp32_mode(self, meta: dict) -> None:
        """New INFO frames: the reported scan config, a rate switch at the sample
        where it arrived (scan_hz from INFO; legacy: the rate measured before for
        these selects, else ~76 Hz, until the measured rate comes ~2 s later),
        and the keeper."""
        for n, at, info in meta.get("esp32_infos") or ():
            if n <= self._info_seen:
                continue
            self._info_seen = n
            cfg = info_cfg(info)
            self._esp32_cfg = cfg
            key = (cfg.get("mode"), cfg.get("scan_hz"), cfg.get("sel_mask"))
            if self._auto_fs and key != self._cfg_key:
                was_locked = bool(self._cfg_key and self._cfg_key[1])
                fs = (cfg.get("scan_hz")
                      or (self._legacy_fs.get(cfg.get("sel_mask", 0xFF)) if cfg.get("mode") == 0
                          else None)
                      or (LEGACY_NOMINAL_HZ if was_locked else self._fs))
                if self._cfg_key is not None or abs(fs - self._fs) > 1e-6 * fs:
                    self._pending_switches.append((int(at), float(fs)))
                    print(f"[MainWindow] ESP32 scan config {describe_cfg(cfg)}: "
                          f"{fs:.3f} Hz from sample {at}")
            self._cfg_key = key
            self._send_esp32(self._keeper.on_info(cfg))
        n_txt, msg = meta.get("esp32_txt") or (0, "")
        if n_txt > self._txt_seen:
            self._txt_seen = n_txt
            self._keeper.on_text(msg)
        self._send_esp32(self._keeper.tick())
        self._show_mode_note()

    def _tick_server_extras(self) -> None:
        """IMU view + a one-line server status (rates, vehicle-side recording)."""
        assert self._client is not None
        rows, seq = self._client.imu_bus.read_since(self._imu_last_seq, block=False)
        meta = self._client.bus.get_metadata()
        self._tick_esp32_mode(meta)
        self._tick_faults(meta)
        self._maybe_apply_measured_rate(meta.get("fs_measured"))
        st = meta.get("server_status") or {}
        if rows is not None:
            self._imu_last_seq = seq
            keep = int(self._t_buffer_s * 400)
            self._imu_rows = np.vstack([self._imu_rows, rows])[-keep:]
            hz = float(st.get("hz", {}).get("RAW_IMU", 0)) or 100.0
            n = len(self._imu_rows)
            if not self._is_paused and self._imu_view.isVisible():
                self._imu_view.update_view(np.arange(-n + 1, 1) / hz, self._imu_rows)
        if st and st.get("t", 0) != self._last_status_t:
            self._last_status_t = st.get("t", 0)
            self._last_status_rec = st
            self._set_fault_state(st.get("esp32", {}).get("fault") or {})
            self._show_status(st)

    # ------------------------------------------------------------------
    # ESP32 / ADC faults
    # ------------------------------------------------------------------

    def _tick_faults(self, meta: dict) -> None:
        """New fault events: the session log, and the recording's sidecar."""
        for n, mark in meta.get("fault_marks") or ():
            if n <= self._fault_marks_seen:
                continue
            self._fault_marks_seen = n
            fev = mark.get("fault", {})
            self._n_fault_events += 1
            t = datetime.fromtimestamp(fev.get("unix") or datetime.now().timestamp())
            self._fault_log.append(f"{t:%Y-%m-%d %H:%M:%S}  {fev.get('event', '?'):5s}  "
                                   f"{mark.get('text', '')}")
            if self._is_recording:
                self._write_fault_sidecar(mark.get("text", ""), t)
        self._update_fault_button()

    def _write_fault_sidecar(self, text: str, t: Optional[datetime] = None) -> None:
        """One line in <recording>.faults.txt: wall time, the .dat row it falls
        on (the rows written so far), the event."""
        if self._fault_sidecar_path is None:
            return
        if self._fault_sidecar is None:
            self._fault_sidecar = open(self._fault_sidecar_path, "a", buffering=1)
            self._fault_sidecar.write("# ESP32 / ADC fault events during "
                                      f"{self._fault_sidecar_path.with_suffix('').stem}.dat:"
                                      " wall time, .dat row (0-based), event\n")
        row = self._dat_writer.n_written if self._dat_writer is not None else 0
        t = t or datetime.now()
        self._fault_sidecar.write(f"{t:%Y-%m-%d %H:%M:%S.%f}"[:-3] + f"  row {row}  {text}\n")

    def _set_fault_state(self, f: dict) -> None:
        """The banner on the panel and on every view window."""
        self._fault_state = f
        level = f.get("level", "ok")
        text = ""
        if level in ("fault", "warn"):
            since = datetime.fromtimestamp(f.get("since", 0)).strftime("%H:%M:%S")
            if level == "fault":
                action = f.get("action") or ""
                text = (f"ADC / ESP32 FAULT since {since}: {f.get('msg', '')}. "
                        f"{action[:1].upper() + action[1:]}.")
                if f.get("detail"):
                    text += f"\nESP32: {f['detail']}"
                text += ("\nNo valid whisker data while this lasts. The alarm clears after"
                         " 10 s of clean streaming; the Fault log keeps it.")
            else:
                text = (f"ESP32 warning at {since}: {f.get('msg', '')} "
                        f"(warnings this session: {sum((f.get('warns') or {}).values())};"
                        " see the Fault log)")
        show_alarm(self._fault_banner, text, level)
        for v in (*self._stream_views, self._imu_view):
            v.set_alarm(text, level)
        self._update_fault_button()

    def _update_fault_button(self) -> None:
        n = self._n_fault_events
        level = self._fault_state.get("level", "ok")
        if (n, level) == getattr(self, "_fault_btn_shown", None):
            return
        self._fault_btn_shown = (n, level)
        self._fault_log_btn.setText(f"Fault log ({n})" if n else "Fault log")
        self._fault_log_btn.setStyleSheet(
            {"fault": "QPushButton { background: #c00000; color: white; font-weight: bold; }",
             "warn": "QPushButton { background: #f0a000; }"}.get(level, ""))

    def _show_fault_log(self) -> None:
        self._fault_log.show()
        self._fault_log.raise_()

    def _show_status(self, st: dict) -> None:
        text = self._format_status(st)
        # the line window's status bar always shows it; the panel's label
        # keeps "Recording -> path" while recording
        for v in self._stream_views:
            v.set_stream_status(text)
        if not self._is_recording:
            self._path_label.setText(text)

    def _format_status(self, st: dict) -> str:
        """One line from the server / ESP32 status record."""
        assert self._client is not None
        hz = st.get("hz", {})
        rec = st.get("rec", {})
        if self._client.transport_kind == "server":
            pi_rec = f"ROV rec {rec.get('secs', 0):.0f}s" if rec.get("on") else "ROV rec off"
        else:
            pi_rec = "streaming" if st.get("esp32", {}).get("streaming") else "NOT streaming"
        e = st.get("esp32", {})
        # running totals since daq_server started; adc-crc = SPI reads re-read,
        # adc-stale = scans where a read had no new conversion (dropped)
        errs = (f"errs: gap {e.get('seq_gaps', 0)} link-crc {e.get('crc_fail', 0)} "
                f"adc-crc {e.get('adc_crc', 0)} adc-stale {e.get('adc_stale', 0)} "
                f"adc-recov {e.get('adc_recov', 0)} adc-reinit {e.get('adc_reinit', 0)}")
        sels = e.get("cfg", {}).get("selects")
        if sels and sels != "01234567":
            errs += f" | selects {sels}"         # skipped ones show as open
        mode = e.get("cfg", {}).get("mode_name")
        if mode and mode != "legacy":
            h = e.get("health")
            errs += f" | {mode}" + (f": busy {h['busy_pct']}% lat {h['lat_max_us']} us"
                                    f" ovr {h['overrun']}" if h else "")
            if e.get("adc_sync"):
                errs += f" sync {e['adc_sync']}"
        if self._keeper.want is not None and self._keeper.note:
            errs += f" | mode: {self._keeper.note}"
        f = e.get("fault") or {}
        if f.get("level") == "fault":
            errs = f"FAULT {f.get('code')} | " + errs
        imu = (f"IMU {hz.get('RAW_IMU', 0):g} Hz | att {hz.get('ATTITUDE', 0):g} Hz | "
               if self._client.transport_kind == "server" else "")
        return (f"{self._client.endpoint_str} | whisker {self._fs:.2f} Hz | {imu}{pi_rec}"
                f" | {errs}")

    def _update_spectrum(self) -> None:
        """Amplitude spectra of the newest ``window_s`` seconds: mean removed, Hann
        window, scaled so a sine of amplitude a reads a (units as the line view)."""
        n = int(round(self._spectrum_view.window_s * self._fs))
        if n < 8 or self._samples_since_clear < n:
            return
        amp = self._amp_spectrum(self._sig[-n:])
        if amp is not None:
            self._spectrum_view.update_view(np.fft.rfftfreq(n, 1 / self._fs), amp)

    def _amp_spectrum(self, x: np.ndarray) -> Optional[np.ndarray]:
        """(nfreq, nch) amplitude of the samples ``x`` (n, nch): mean removed, Hann
        window, scaled so a sine of amplitude a reads a; None if ``x`` has gaps."""
        # float64: a railed gauge (~1200 mV, constant) leaves ~1e-4 of float32 rounding
        # after the mean removal, which would read as a tiny peak instead of nothing
        x = x.astype(np.float64)
        if not np.isfinite(x).all():
            return None
        n = x.shape[0]
        w = np.hanning(n)
        # Mean removal, not a linear detrend: with a 1 s window a 1 Hz sine is one cycle,
        # and a line fitted to one cycle takes ~45 % of its amplitude (5 mV read as 2.7).
        # Slow drift is kept out of the higher bins by the Hann window.
        amp = np.abs(np.fft.rfft(detrend(x, axis=0, type="constant") * w[:, None], axis=0))
        amp *= 2 * self._scale / w.sum()
        amp[0] /= 2
        if n % 2 == 0:
            amp[-1] /= 2
        amp[amp < 1e-9] = 0                    # round-off of a constant (railed) gauge
        return amp

    def _update_fft_and_spec(self) -> None:
        """Recompute the current FFT (``do_fft`` in wavi.m:901-914, but with the
        Spectrum window's method: mean removed, Hann window, amplitude-scaled)."""
        if self._samples_since_clear < self._n_fft_window:
            return
        amp = self._amp_spectrum(self._sig[-self._n_fft_window:])
        if amp is None:
            return
        self._fft_map = amp.astype(np.float32)

        # Spectrogram: shift columns left, drop new frame at right edge.
        self._spec_data[:, :-1] = self._spec_data[:, 1:]
        self._spec_data[:, -1] = self._fft_map.reshape(-1, order="F")

        # the spectrogram history above is kept regardless; only redraw what is shown
        if self._fft_view.isVisible():
            self._fft_view.update_view(self._fft_map, self._fs)
        if self._spec_view.isVisible():
            self._spec_view.update_view(self._spec_data, self._fs, self._ns_spec / self._fs)

    def _write_record(self, cleaned: np.ndarray, first_idx: int) -> None:
        assert self._dat_writer is not None
        n = cleaned.shape[0]
        if n == 0 or self._t0_unix is None:
            return
        t0 = self._t0_unix + self._t_of(first_idx)
        timestamps = [datetime.fromtimestamp(t0 + i / self._fs) for i in range(n)]
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
            for v in (self._line_view, self._spectrum_view, self._fft_view, self._spec_view,
                      self._imu_view, self._sim_panel, self._fault_log):
                v.close()
        super().closeEvent(event)
