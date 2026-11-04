"""GUI subscriber — the control panel + the visualization pipeline.

This is one subscriber on the SampleBus among many. It owns:

- The control panel (transport selector, record, pause, view toggles, tag, auto-stop).
- The view windows (line, spectrum, fft, spectrogram, imu, dome).
- The local processing chain: despike → recording → (optional) filter → plot updates.
  Despike (``daq/despike.py``, the panel's **Despike**): live, the same centred
  test judged 28 samples late (112 ms at 250 Hz), so the newest samples are
  shown raw and patched over the next ticks, and the .dat gets them that late;
  replay, the transport serves the scans with the spikes the index found over
  the whole file (off by default: replay shows the recording as it is).
- Replay of a recording (.jsonl, transport "Replay"): the file is indexed in the
  background, then played through the same pipeline as live
  (``transports/replay_transport.py``). Pause pauses playback; a seek is a
  discontinuity the buffers start over at (``replay_marks``); Rec, the scan
  mode, Reset ESP32 and the mode keeper are off (nothing to command).

Sample times: event streams carry each scan's and IMU row's time (the device
clocks mapped onto the Pi's receive clock, ``daq/clock.py``; fitted over the
whole file in replay), so the Line and IMU windows share one time axis (session
seconds) and the IMU window can follow the Line window's time range.

Plot data: the Line and IMU windows read through the series API
(``daq/series.py``). Live, that is the rolling buffers (``t_buffer_s``, 90 s by
default; the windows open on the newest ``DEFAULT_VIEW_SPAN_S``); in replay,
once it is built in the background after the index (``_store_job``), the whole
recording (``replay/store.py``), drawn for any span and following the
playhead, which a timeline drag moves at mouse rate (``ReplayModel.scrubbed``)
without waiting for the transport. ``series()`` gives the same API to other
readers (pick-and-compare, select-for-FFT).

External agent loops can subscribe to ``WaviClient.bus`` independently without
touching this module. The GUI's slow plot redraws can never starve them — they
poll the bus on their own thread.
"""

from __future__ import annotations

import math
import os
import socket
import threading
import time
from datetime import datetime, timedelta
from pathlib import Path
from typing import Optional

import numpy as np
from PyQt6.QtCore import QRegularExpression, QTimer, Qt
from PyQt6.QtGui import QGuiApplication, QKeySequence, QRegularExpressionValidator, QShortcut
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMenu,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

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
    DEFAULT_VIEW_SPAN_S,
    MIN_REC_LEN_S,
)
from wavi_py.daq.despike import Despiker
from wavi_py.daq.series import IMU, WSK, ArraySeries, Series
from wavi_py.daq.server_reader import IMU_CHANNELS
from wavi_py.daq.spectra import SPEC_COL_HZ, SpecHistory, amp_spectra, amp_spectrum
from wavi_py.io.dat_writer import DatWriter, build_datalog_filename
from wavi_py.replay.index import RecordingIndex
from wavi_py.replay.store import RecordingStore
from wavi_py.scan_mode import (DEFAULT_SELECTS, PRESETS, ModeKeeper, ScanRequest, describe_cfg,
                               legacy_nominal_hz, parse_selects)
from wavi_py.transports.sim_transport import SimSignals
from wavi_py.views import FftView, ImuView, LineView, SpectrogramView, SpectrumView
from wavi_py.views.base import add_timeline_dock, alarm_label, show_alarm
from wavi_py.views.fault_log import FaultLogView
from wavi_py.views.replay_bar import FINE_STEP_S, STEP_S, ReplayModel, TimelineBar, fmt_t
from wavi_py.views.sim_panel import SimPanel
from wavi_py.views.spectral_base import FFT_WINDOWS_S
from whisker_link.channels import NUM_WHISKER, SAT_CODE
from whisker_link.dome_map import DEFAULT_MAP, DomeMap, DomeMapError
from whisker_link.esp32_frame import MV_PER_CODE, info_cfg

RESET_ACK_TIMEOUT_S = 3.0     # esp32_reset's ack (the RTS pulse takes 0.1 s)
REC_ACK_TIMEOUT_S = 5.0       # rec_start / rec_stop's ack from daq_server
SERVER_STOP_GRACE_S = 3.0     # auto-stop: the server's own stop (rec_start duration) gets
                              # this long to show up in the status before the client stops
REBOOT_NOTE_S = 30.0         # an ESP32 reboot (with its reset reason) stays in the status line
NOTE_HOLD_S = 5.0            # a reply / result stays in the panel label before the status


def _default_outpath() -> Path:
    home = os.environ.get("USERPROFILE") or os.environ.get("HOME") or "."
    p = Path(home) / "wavi_data"
    p.mkdir(parents=True, exist_ok=True)
    return p


class _WrapLabel(QLabel):
    """Word-wrapped text that never widens the window: it takes the width it is
    given and grows in height to fit (a plain wrapped QLabel in a layout keeps the
    height it had at the first layout and clips)."""

    def __init__(self, text: str = ""):
        super().__init__(text)
        self.setWordWrap(True)
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)

    def setText(self, text: str) -> None:  # noqa: N802 — Qt naming
        super().setText(text)
        self._fit()

    def resizeEvent(self, event) -> None:  # noqa: N802 — Qt naming
        super().resizeEvent(event)
        self._fit()

    def _fit(self) -> None:
        h = self.heightForWidth(max(1, self.width()))
        if h > 0 and h != self.minimumHeight():
            self.setMinimumHeight(h)


def _same_rec(a: Optional[str], b: Optional[str]) -> bool:
    """Two names of the vehicle's recording (a path, or just the file name)."""
    return bool(a) and bool(b) and (a == b or os.path.basename(a) == os.path.basename(b))


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
        dome_map: Optional[DomeMap] = None,
        replay_path: Optional[str] = None,
        replay_speed: Optional[float] = 1.0,
        replay_start_s: float = 0.0,
    ):
        super().__init__()
        self.setWindowTitle("Wavi (Python)")
        self.resize(560, 320)

        # acquisition params
        self._initial_transport = transport
        self._initial_host = host
        self._initial_port = int(port)
        self._initial_serial_port = serial_port
        # replay: the file, and where / how fast the first open starts
        self._initial_replay_path = replay_path or ""
        self._replay_speed = replay_speed
        self._replay_start_s = float(replay_start_s)
        self._replay = ReplayModel()
        self._index: Optional[RecordingIndex] = None
        self._index_job: Optional[dict] = None       # the background indexing
        self._replay_gen = 0                         # the newest seek handled
        self._replay_seek_at: Optional[int] = None   # its bus index, until "live"
        self._status_shown_t = 0.0
        self._note_until = 0.0                 # a reply / result in the panel label until then
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
        self._reset_sent_at: Optional[float] = None    # esp32_reset sent, ack pending
        self._reset_ack_seen = 0

        # ESP32 / ADC faults (whisker_link/esp32_fault.py, run by the transport or
        # daq_server): the current state from the status record, the events for
        # the session's log, and a <recording>.faults.txt next to the .dat
        self._fault_state: dict = {}
        self._fault_marks_seen = 0
        self._n_fault_events = 0
        self._beat_uptime: Optional[int] = None      # the last heartbeat's ESP32 uptime (ms)
        self._reboot_note = ""
        self._reboot_note_until = 0.0
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
        # the vehicle's recording (server transport): ours is the one this window
        # started (_rov_path, from the ack); one running that isn't ours (another
        # client's, or ours from before a crash / reconnect) is adopted from the
        # status record: the Rec button then only stops it, never starts over it
        self._rov_path: Optional[str] = None
        self._rov_confirmed = False              # a status record showed ours running
        self._rov_duration = False               # the server stops ours on its own
        self._foreign_rec: Optional[dict] = None  # status["rec"] of one not ours
        self._rec_cmd: Optional[dict] = None     # rec_start / rec_stop sent, ack pending
        self._flag_cmd: Optional[dict] = None    # rec_flag sent, ack pending
        self._who = f"wavi_py@{socket.gethostname()}"
        self._t0_unix: Optional[float] = None    # stream start in unix time
        # live despike: the newest `pending` samples are provisional (raw) until judged,
        # 28 samples later; the .dat gets each sample once final, from index _rec_from on
        self._despiker: Optional[Despiker] = Despiker()
        self._next_idx = 0                       # index of the next sample to ingest
        self._rec_from = 0

        # client (None until Connect pressed)
        self._client: Optional[WaviClient] = None
        # the Sim transport's signals: kept across connects, edited in the signal panel
        self._sim = SimSignals()
        self._sim_panel = SimPanel(self._sim)

        # build UI before constructing views (views are created lazily — only on first toggle)
        self._build_ui()

        # construct views up front but hidden, mirroring MATLAB behavior
        self._line_view = LineView(self._nsensor, self._nch, self._t_buffer_s, DEFAULT_VIEW_SPAN_S)
        self._spectrum_view = SpectrumView(self._nsensor, self._nch)
        self._fft_view = FftView(self._nch, self._nfreq, self._t_fft_s)
        self._spec_view = SpectrogramView(
            self._nch, self._nfreq, self._fs, self._spec_hop, self._t_spec_s, self._t_fft_s
        )
        self._imu_view = ImuView(self._t_buffer_s, DEFAULT_VIEW_SPAN_S)
        # the Dome and ROV views need PyOpenGL and a GL context: built on first use
        self._dome_view = None
        self._rov_view = None
        # whisker positions / sensors / calibration (None: W1 .. W16, placeholder dome holes)
        self._dome_map: Optional[DomeMap] = None
        self.set_dome_map(dome_map)
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
        self._imu_t = np.empty(0)
        self._last_status_t = 0.0
        # the IMU window follows the Line window's time range (and drives it)
        self._line_view.timeWindowChanged.connect(self._on_line_time_window)
        self._line_view.closed.connect(lambda: self._imu_view.set_time_window(None))
        self._imu_view.timeWindowChanged.connect(self._line_view.set_time_window)
        # replay: playback state drives the Pause buttons
        self._replay.stateChanged.connect(self._on_replay_state)
        self._replay.seeked.connect(lambda _t: self._line_view.go_live())
        # replay: the playhead in the Line / IMU windows (drawn from the store), at mouse
        # rate while the timeline's playhead is dragged
        self._store: Optional[RecordingStore] = None
        self._store_job: Optional[dict] = None
        self._replay.positionChanged.connect(self._on_playhead)
        self._replay.scrubbed.connect(lambda _t: self._line_view.go_live())
        # the timeline (replay controls, depth, events): a dock in the panel and in
        # every view window, all on the one model; shown while there is a source
        self._timeline_docks = [add_timeline_dock(self, TimelineBar(self._replay))]
        for v in (self._line_view, self._spectrum_view, self._fft_view, self._spec_view,
                  self._imu_view):
            self._timeline_docks.append(v.add_timeline(TimelineBar(self._replay)))
        self._replay.sourceChanged.connect(self._show_timelines)
        self._replay.liveSeekRequested.connect(self._show_time_from_buffer)
        self._replay.exportRequested.connect(self.export_range)
        self._live_events_t = 0.0
        self._line_view.zeroAtRequested.connect(self.zero_at)
        self._build_keys()

        # main-thread timer that drains the bus
        self._timer = QTimer(self)
        self._timer.setInterval(50)  # 20 Hz UI cadence
        self._timer.timeout.connect(self._on_tick)

    def _set_rate(self, fs: float) -> None:
        """Size the rolling buffers for sample rate ``fs`` and clear them."""
        self._fs = float(fs)
        self._ns_tot = max(int(round(self._fs * FFT_WINDOWS_S[-1])), int(self._t_buffer_s * self._fs))
        self._sig = np.full((self._ns_tot, self._nch), np.nan, dtype=np.float32)
        self._sig_t = np.full(self._ns_tot, np.nan)      # the samples' times (session s)
        self._size_spectral()
        # Track how many real samples are sitting in `_sig` since the last
        # connect/reset. Used to feed only the populated portion to the line
        # view; otherwise pyqtgraph's auto-range latches onto the full t-axis
        # the moment any sample lands and gaps the NaN-filled leading slots.
        self._samples_since_clear = 0
        if getattr(self, "_dome_view", None) is not None:
            self._dome_view.set_rate(self._fs)

    def _size_spectral(self) -> None:
        """Size (and clear) the FFT frame and the spectrogram history."""
        self._n_fft_window = max(2, int(round(self._fs * self._t_fft_s)))
        self._nfreq = self._n_fft_window // 2 + 1
        # a spectrogram column every ``hop`` samples (ns_spec, but at most SPEC_COL_HZ a second)
        self._spec_hop = max(self._ns_spec, math.ceil(self._fs / SPEC_COL_HZ))
        self._spec_cols = max(1, int(self._t_spec_s * self._fs / self._spec_hop))
        self._spec_hist = SpecHistory(self._nfreq * self._nch, self._spec_cols)
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
        self._spec_cols = max(1, int(self._t_spec_s * self._fs / self._spec_hop))
        self._spec_hist.resize(self._spec_cols)
        self._spec_view.set_span(self._t_spec_s)

    def _t_of(self, idx: int) -> float:
        """Seconds since the stream start of sample index ``idx``."""
        return self._anchor_t + (idx - self._anchor_idx) / self._fs

    def _switch_rate(self, fs: float, at: int) -> None:
        """From sample index ``at`` on the rate is ``fs``: move the time anchor
        there and start the buffers over (no window mixes two rates)."""
        self._flush_despike()
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
        self._transport_combo.addItems(["Server", "ESP32", "Sim", "TCP", "Serial", "Replay"])
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

        self._file_edit = QLineEdit(self._initial_replay_path)
        self._file_edit.setPlaceholderText("recording (.jsonl, or .dat)")
        self._file_edit.setToolTip("A daq_server recording: the Pi's whisker_rec/*.jsonl, or the\n"
                                   "local copy next to a .dat (also the ESP32 / Sim tees).\n"
                                   "A .dat alone replays the whiskers only (no IMU, events,\n"
                                   "status; the times count samples)")
        self._browse_btn = QPushButton("Browse...")
        self._browse_btn.clicked.connect(self._choose_recording)

        self._connect_btn = QPushButton("Connect")
        self._connect_btn.clicked.connect(self._on_connect_clicked)

        self._sim_btn = QPushButton("Signals")
        self._sim_btn.setCheckable(True)
        self._sim_btn.setToolTip("Sim transport: the simulated signals (gauges, taps, faults)")
        self._sim_btn.toggled.connect(lambda on: self._toggle_window(self._sim_panel, on))
        self._sim_panel.closed.connect(lambda: self._sim_btn.setChecked(False))

        self._live_inputs = [QLabel("Host:"), self._host_edit, QLabel("Port:"), self._port_spin,
                             QLabel("Serial:"), self._serial_port_edit]
        self._replay_inputs = [QLabel("File:"), self._file_edit, self._browse_btn]
        h.addWidget(QLabel("Transport:"))
        h.addWidget(self._transport_combo)
        for w, stretch in zip(self._live_inputs, (0, 2, 0, 0, 0, 1)):
            h.addWidget(w, stretch=stretch)
        for w, stretch in zip(self._replay_inputs, (0, 3, 0)):
            h.addWidget(w, stretch=stretch)
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
            "The ESP32 boots in wide 250 Hz, OSR 128 (core C1) with selects 0123457 and\n"
            "forgets any mode setting on reboot: the mode shown next to this is what its INFO\n"
            "reports, and if it comes back in another mode the choice is re-sent once.\n"
            "'as is' sends nothing. The legacy rate depends on the selects (~89 Hz for 7);\n"
            "the measured rate is shown.")
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
        self._esp32_reset_btn = QPushButton("Reset ESP32")
        self._esp32_reset_btn.setToolTip(
            "Reboot the ESP32 (RTS pulse = pressing EN), through daq_server (Server) or the\n"
            "serial port (ESP32). Streaming stops for a few seconds; a recording gets a gap.\n"
            "Doesn't power-cycle the whisker PCB, so it won't clear an ADC lock-up.")
        self._esp32_reset_btn.clicked.connect(self._on_esp32_reset_clicked)
        self._esp32_reset_btn.setEnabled(False)
        h.addWidget(self._esp32_reset_btn)
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
        if data and self._client is not None and not self._replaying:
            print(f"[MainWindow] to the ESP32: {data}")
            self._client.send_command({"cmd": "esp32", "data": data})

    def _on_esp32_reset_clicked(self) -> None:
        """Ask, then send esp32_reset; _tick_esp32_reset shows the ack."""
        if (self._client is None or not self._client.is_running
                or not self._client.is_event_stream):
            return
        text = "Reset the ESP32 (EN pulse)?\n\nStreaming stops for a few seconds while it reboots."
        if self._is_recording:
            text += "\nThe recording keeps running and gets a gap."
        if QMessageBox.question(self, "Reset ESP32", text) != QMessageBox.StandardButton.Yes:
            return
        acks = self._client.bus.get_metadata().get("server_acks") or {}
        self._reset_ack_seen = acks.get("esp32_reset", (0, None))[0]
        if not self._client.send_command({"cmd": "esp32_reset"}):
            self._note("ESP32 reset: could not send the command.")
            return
        self._reset_sent_at = time.monotonic()
        self._path_label.setText("ESP32 reset sent, waiting for the reply...")

    def _tick_esp32_reset(self, meta: dict) -> None:
        if self._reset_sent_at is None:
            return
        n, ack = (meta.get("server_acks") or {}).get("esp32_reset", (0, None))
        if n > self._reset_ack_seen and ack is not None:
            self._reset_sent_at = None
            msg = ("ESP32 reset: EN pulsed, it is rebooting (streaming resumes in a few s)."
                   if ack.get("ok") else f"ESP32 reset failed: {ack.get('error', 'no reason given')}")
            print(f"[MainWindow] {msg}")
            self._note(msg)
        elif time.monotonic() - self._reset_sent_at > RESET_ACK_TIMEOUT_S:
            self._reset_sent_at = None
            self._note("ESP32 reset: no reply (is the server up to date?).")

    def _show_mode_note(self) -> None:
        """The keeper's note (or the reported mode) next to the dropdown and in the
        stream status line."""
        note = self._keeper.note if not self._replaying else ""
        if not note and self._esp32_cfg is not None:
            note = (f"{'recorded' if self._replaying else 'ESP32 reports'} "
                    f"{describe_cfg(self._esp32_cfg)}")
        if note == self._mode_label.text():
            return
        self._mode_label.setText(note)
        self._mode_label.setToolTip(note)
        bad = self._keeper.state == "failed" and not self._replaying
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

        self._despike_chk = QCheckBox("Despike")
        self._despike_chk.setChecked(True)
        self._despike_chk.setToolTip(
            "Replace isolated spikes (1-3 samples, > 8x the local noise) by the median of\n"
            "the 7 samples around them (daq/despike.py). Live: on by default, judged 28\n"
            "samples late (the newest ~0.1 s is patched as it comes); off: raw samples.\n"
            "Replay: off by default (the recording as it is); on uses the spikes found\n"
            "over the whole file, and the .dat export follows it.")
        self._despike_chk.toggled.connect(self._on_despike_toggled)

        h.addWidget(self._toggle_line)
        h.addWidget(self._toggle_spectrum)
        h.addWidget(self._toggle_fft)
        h.addWidget(self._toggle_spec)
        self._toggle_dome = QPushButton("Dome")
        self._toggle_dome.setCheckable(True)
        self._toggle_dome.setToolTip("3D whiskers bending on the dome (placeholder mapping and"
                                     " calibration; needs PyOpenGL)")
        self._toggle_dome.toggled.connect(self._toggle_dome_view)
        self._toggle_rov = QPushButton("ROV")
        self._toggle_rov.setCheckable(True)
        self._toggle_rov.setToolTip("The ROV's attitude and translational acceleration from the"
                                    " IMU, on a BlueROV2 model (Server / Replay; needs PyOpenGL)")
        self._toggle_rov.toggled.connect(self._toggle_rov_view)

        self._map_btn = QToolButton()
        self._map_btn.setText("Map")
        self._map_btn.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
        menu = QMenu(self._map_btn)
        menu.addAction("Load dome map...", self._choose_dome_map)
        menu.addAction("ROV dome (default map)", lambda: self._load_dome_map(DEFAULT_MAP))
        menu.addAction("None (W1 .. W16)", lambda: self.set_dome_map(None))
        self._map_btn.setMenu(menu)

        h.addWidget(self._toggle_imu)
        h.addWidget(self._toggle_dome)
        h.addWidget(self._toggle_rov)
        h.addWidget(self._map_btn)
        h.addStretch(1)
        h.addWidget(self._despike_chk)
        h.addWidget(self._pause_btn)
        h.addWidget(self._reset_btn)
        return box

    def _build_record_row(self) -> QGroupBox:
        box = QGroupBox("Record")
        h = QHBoxLayout(box)

        self._tag_edit = QLineEdit()
        self._tag_edit.setPlaceholderText("tag (alphanumerics, _ -)")

        # Auto-stop: the toggle says whether, the box how long (whole seconds); the box
        # keeps its value while the toggle is off, and typing a time (or the arrows)
        # turns it on, as the FFT / Spec colour range's Auto + min / max do
        self._auto_stop_chk = QCheckBox("Auto-stop")
        self._auto_stop_chk.setToolTip(
            "Stop each recording after the time in the box. With daq_server the server\n"
            "stops it itself, so it also stops if this window is gone.")
        self._auto_stop_chk.toggled.connect(self._on_auto_stop_toggled)
        self._auto_stop_spin = QSpinBox()
        self._auto_stop_spin.setRange(math.ceil(MIN_REC_LEN_S), 36000)
        self._auto_stop_spin.setSuffix(" s")
        self._auto_stop_spin.setSingleStep(5)
        self._auto_stop_spin.setValue(60)
        self._auto_stop_spin.setKeyboardTracking(False)    # one value when typed, not per digit
        self._auto_stop_spin.setToolTip(f"Auto-stop time, {math.ceil(MIN_REC_LEN_S)} s to 10 h;"
                                        " typing a time turns Auto-stop on")
        self._auto_stop_spin.valueChanged.connect(self._on_auto_stop_edited)

        self._record_btn = QPushButton("● Rec")
        self._record_btn.setStyleSheet("QPushButton { color: #c00; font-weight: bold; }")
        self._record_btn.clicked.connect(self._on_record_clicked)
        self._record_btn.setEnabled(False)
        # wide enough for "Stop ROV rec <secs>", so the row doesn't shift as it counts
        self._record_btn.setMinimumWidth(
            self._record_btn.fontMetrics().horizontalAdvance("■ Stop ROV rec 99999s") + 24)

        h.addWidget(QLabel("Tag:"))
        h.addWidget(self._tag_edit, stretch=2)
        h.addWidget(self._auto_stop_chk)
        h.addWidget(self._auto_stop_spin)
        h.addWidget(self._record_btn)

        # the vehicle's recording index (trials.jsonl on the Pi): flag the newest
        # recording bad, or good again; the file stays, convert skips bad ones
        self._flag_btn = QPushButton("Mark bad")
        self._flag_btn.clicked.connect(self._on_flag_clicked)
        self._flag_btn.setEnabled(False)
        self._flag_btn.setMinimumWidth(
            self._flag_btn.fontMetrics().horizontalAdvance("Unmark bad") + 24)
        h.addWidget(self._flag_btn)
        return box

    def _build_status_row(self) -> QGroupBox:
        box = QGroupBox("Status")
        v = QVBoxLayout(box)

        self._progress = QProgressBar()
        self._progress.setRange(0, 1000)
        self._progress.setValue(0)
        self._progress.setTextVisible(False)
        self._progress.setMaximumHeight(8)

        self._path_label = _WrapLabel(str(self._outpath))
        self._path_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self._path_label.setStyleSheet("QLabel { color: #555; }")
        # the status lines (and long paths) wrap instead of widening the window

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
        replay = kind == "REPLAY"
        for w in self._live_inputs:
            w.setVisible(not replay)
        for w in self._replay_inputs:
            w.setVisible(replay)
        if self._client is None:
            self._connect_btn.setText("Open" if replay else "Connect")
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
        elif self._index_job is not None:
            return                                   # still indexing
        elif self._transport_combo.currentText().lower() == "replay":
            self.open_replay(self._file_edit.text().strip())
        else:
            self._connect()

    # ----- replay: index the file in the background, then connect to it

    def _choose_recording(self) -> None:
        cur = self._file_edit.text().strip()
        start = str(Path(cur).parent) if cur else str(self._outpath)
        path, _ = QFileDialog.getOpenFileName(self, "Open a recording", start,
                                              "Recordings (*.jsonl *.dat);;All files (*)")
        if path:
            self._file_edit.setText(path)
            if self._client is None and self._index_job is None:
                self.open_replay(path)

    def open_replay(self, path: str) -> None:
        """Index ``path`` (a thread; the progress bar shows it), then play it paused
        at the start (or ``--start`` the first time)."""
        if not path or not Path(path).is_file():
            QMessageBox.critical(self, "Open failed", f"No such recording: {path or '(none)'}")
            return
        i = self._transport_combo.findText("Replay")
        if self._transport_combo.currentIndex() != i:
            self._transport_combo.setCurrentIndex(i)
        self._file_edit.setText(path)
        job: dict = {"path": path, "frac": 0.0, "index": None, "error": None}

        def run() -> None:
            try:
                job["index"] = RecordingIndex.open(
                    path, progress=lambda f: job.__setitem__("frac", f))
            except Exception as exc:  # noqa: BLE001 — not a recording, unreadable
                job["error"] = str(exc) or type(exc).__name__
        self._index_job = job
        self._set_transport_inputs_enabled(False)
        self._connect_btn.setEnabled(False)
        self._path_label.setText(f"Indexing {Path(path).name}...")
        threading.Thread(target=run, name="wavi-index", daemon=True).start()
        QTimer.singleShot(50, self._poll_index_job)

    def _poll_index_job(self) -> None:
        job = self._index_job
        if job is None:
            return
        if job["index"] is None and job["error"] is None:
            self._progress.setValue(int(job["frac"] * 1000))
            QTimer.singleShot(50, self._poll_index_job)
            return
        self._index_job = None
        self._progress.setValue(0)
        self._connect_btn.setEnabled(True)
        self._set_transport_inputs_enabled(True)
        if job["error"] is not None:
            QMessageBox.critical(self, "Open failed", f"{job['path']}: {job['error']}")
            self._path_label.setText(f"Could not open {job['path']}")
            return
        self._index = job["index"]
        self._connect(index=self._index)
        if self._replaying:
            self._start_store(self._index)

    def _start_store(self, index: RecordingIndex) -> None:
        """The whole recording for the Line / IMU windows (replay/store.py; ~3 s for a
        13 min dive, cached next to the index): a thread, then ``_poll_store_job``.
        Until it is in, the windows draw the pipeline's buffer as live."""
        job: dict = {"index": index, "frac": 0.0, "store": None, "error": None}

        def run() -> None:
            try:
                job["store"] = RecordingStore.open(
                    index, progress=lambda f: job.__setitem__("frac", f))
            except Exception as exc:  # noqa: BLE001 — a plot aid: replay goes on without it
                job["error"] = str(exc) or type(exc).__name__
        self._store_job = job
        threading.Thread(target=run, name="wavi-store", daemon=True).start()
        QTimer.singleShot(50, self._poll_store_job)

    def _poll_store_job(self) -> None:
        job = self._store_job
        if job is None:
            return
        if job["store"] is None and job["error"] is None:
            if not self._is_recording:
                self._progress.setValue(int(job["frac"] * 1000))
            QTimer.singleShot(50, self._poll_store_job)
            return
        self._store_job = None
        if not self._is_recording:
            self._progress.setValue(0)
        if job["index"] is not self._index or not self._replaying:
            return                                   # closed (or another file) meanwhile
        if job["error"] is not None or not job["store"].has(WSK):
            self._note(f"Whole-recording plots unavailable ({job['error'] or 'no scans'}); "
                       "the Line window shows the playback buffer.")
            return
        self._set_store(job["store"])
        print(f"[MainWindow] {self._store.summary()}")

    def _set_store(self, store: Optional[RecordingStore]) -> None:
        self._store = store
        despike = bool(self._despike_chk.isChecked())
        self._line_view.set_store(store, despike)
        self._imu_view.set_store(store if store is not None and store.has(IMU) else None)
        if store is not None:
            self._on_playhead(self._replay.position)

    def _on_playhead(self, t: float) -> None:
        """Replay: the playhead (poll, a seek, or a scrub at mouse rate) in Line / IMU."""
        if self._store is not None and math.isfinite(t):
            self._line_view.set_playhead(t)
            self._imu_view.set_playhead(t)

    def series(self) -> Series:
        """The plot data through the series API (``daq/series.py``): the whole
        recording in replay (once built), else the live rolling buffers, whisker
        (``t_buffer_s``) and IMU. The live arrays move on the next tick: read at once
        (reads copy)."""
        if self._store is not None:
            return self._store
        n = self._samples_since_clear
        names = [f"W{i // 2 + 1}_{'AB'[i % 2]}" for i in range(self._nch)]
        return ArraySeries({WSK: (self._sig_t[len(self._sig_t) - n:], self._sig[len(self._sig) - n:]),
                            IMU: (self._imu_t, self._imu_rows)},
                           {WSK: names, IMU: IMU_CHANNELS})

    def _connect(self, index: Optional[RecordingIndex] = None) -> None:
        try:
            kind = self._transport_combo.currentText().lower()
            if kind == "replay" and index is not None and index.fs > 0:
                self._set_rate(index.fs)             # the scan rate measured from the file
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
                index=index,
                replay_speed=self._replay_speed,
                replay_start_s=self._replay_start_s,
                replay_playing=False,
            )
            client.start()
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "Connect failed", str(exc))
            return
        self._replay_start_s = 0.0                   # --start: the first open only

        self._client = client
        self._last_seq = 0
        self._next_idx = 0
        self._despike_chk.blockSignals(True)
        self._despike_chk.setChecked(not self._replaying)   # live on, replay raw
        self._despike_chk.blockSignals(False)
        self._despiker = None if self._replaying else Despiker()
        self._samples_since_spec_update = 0
        self._samples_since_clear = 0
        self._sig.fill(np.nan)
        self._spec_hist.clear()
        self._fft_map.fill(0)
        # Force the metadata-latching path in _on_tick to re-run; otherwise
        # the new bus's V0 is ignored and FFT/plots use stale offsets.
        self._v0 = np.zeros(self._nch, dtype=np.float32)
        self._t0_unix = None
        self._last_v0_version = 0
        self._imu_last_seq = 0
        self._imu_rows = np.empty((0, 9), dtype=np.float32)
        self._imu_t = np.empty(0)
        if self._rov_view is not None:
            self._rov_view.clear()
        if self._dome_view is not None:
            self._dome_view.clear_imu()
        self._sig_t.fill(np.nan)
        self._replay_gen, self._replay_seek_at = 0, None
        self._anchor_idx, self._anchor_t = 0, 0.0
        self._esp32_cfg, self._cfg_key = None, None
        self._info_seen = self._txt_seen = 0
        self._pending_switches = []
        self._last_status_rec = {}
        self._fault_marks_seen = 0
        self._beat_uptime, self._reboot_note = None, ""
        self._set_fault_state({})
        self._foreign_rec, self._rec_cmd, self._flag_cmd = None, None, None
        if client.is_event_stream and not self._replaying:
            self._keeper.connected()
        self._show_mode_note()

        self._connect_btn.setText("Close" if self._replaying else "Disconnect")
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
        # a replay records nothing (the file is the recording)
        self._update_record_button()
        self._set_transport_inputs_enabled(False)
        if not self._replaying and client.has_vehicle:
            self._replay.set_live(client.timeline)      # live: the events and depth so far
        if self._replaying:
            assert index is not None
            self._replay.attach(client.replay, index)
            self.setWindowTitle(f"Wavi (Python) - replay {index.path.name}")
            self._path_label.setText(f"Replaying {index.summary()}")
            self._fault_log.append(f"---- replay of {index.path.name} "
                                   f"({datetime.fromtimestamp(index.unix0):%Y-%m-%d %H:%M:%S}, "
                                   f"{fmt_t(index.duration)}) ----")
        else:
            self._path_label.setText(f"Connected to {client.endpoint_str} — output dir: {self._outpath}")
        for v in self._stream_views:
            v.set_stream_status(f"Connected to {client.endpoint_str}")
        self._last_status_t = 0.0
        if self._dome_view is not None:
            self._dome_view.set_bus(client.bus, self._fs)
        self._timer.start()

    def _disconnect(self) -> None:
        self._store_job = None
        if self._store is not None:
            self._set_store(None)
        if self._is_recording:
            self._stop_recording()
        # one not ours keeps running (the status line said so); a rec_start still
        # waiting for its ack is left to the server (the next connect adopts it)
        self._foreign_rec, self._rec_cmd, self._flag_cmd = None, None, None
        self._timer.stop()
        if self._client is not None:
            self._client.stop()
            self._client = None
        if self._dome_view is not None:
            self._dome_view.set_bus(None)
        if self._replay.source is not None:
            self._replay.detach()
            self.setWindowTitle("Wavi (Python)")

        self._keeper.set_wanted(self._keeper.want, connected=False)
        self._esp32_cfg = None
        self._set_fault_state({})
        self._show_mode_note()
        self._connect_btn.setText(
            "Open" if self._transport_combo.currentText().upper() == "REPLAY" else "Connect")
        self._pause_btn.setChecked(False)
        self._set_stream_buttons_enabled(False)
        self._update_record_button()
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
        server_stops = self._rov_duration
        self._disconnect()
        msg = f"Connection lost: {reason}. Click Connect to reconnect."
        for v in self._stream_views:
            v.set_stream_status(f"Connection lost: {reason}")
        if was_recording:
            msg += (" The local recording was saved; the ROV-side recording "
                    + ("stops by itself at its auto-stop time." if server_stops
                       else "may still be running (Connect shows it)."))
        self._path_label.setText(msg)

    @property
    def _replaying(self) -> bool:
        return self._client is not None and self._client.transport_kind == "replay"

    def _set_transport_inputs_enabled(self, enabled: bool) -> None:
        self._file_edit.setEnabled(enabled)
        self._browse_btn.setEnabled(enabled)
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

    def _toggle_dome_view(self, show: bool) -> None:
        if show and self._dome_view is None:
            try:
                if QGuiApplication.platformName() in ("offscreen", "minimal"):
                    raise RuntimeError(f"no OpenGL on the Qt platform '{QGuiApplication.platformName()}'")
                self._check_gl_sharing(self._rov_view)
                from wavi_py.views.dome_view import DomeView
                self._dome_view = DomeView(self._fs, dome_map=self._dome_map)
            except Exception as exc:  # noqa: BLE001 — no PyOpenGL / GL context
                QMessageBox.warning(self, "Dome view", f"Cannot open the Dome view: {exc}\n"
                                    "(pip install PyOpenGL)")
                self._toggle_dome.setChecked(False)
                return
            self._dome_view.closed.connect(lambda: self._toggle_dome.setChecked(False))
            self._timeline_docks.append(self._dome_view.add_timeline(TimelineBar(self._replay)))
            self._timeline_docks[-1].setVisible(self._replay.source is not None)
            self._dome_view.set_paused(self._is_paused)
            self._dome_view.update_imu(self._imu_t, self._imu_rows)
            if self._client is not None:
                self._dome_view.set_bus(self._client.bus, self._fs)
        if self._dome_view is not None:
            self._toggle_window(self._dome_view, show)

    @staticmethod
    def _check_gl_sharing(other) -> None:
        """A second GL window draws pyqtgraph's shaders compiled in the first one's
        context: only valid with shared contexts (app.py sets the attribute)."""
        if other is not None and not QGuiApplication.testAttribute(
                Qt.ApplicationAttribute.AA_ShareOpenGLContexts):
            raise RuntimeError("a second OpenGL window needs Qt.AA_ShareOpenGLContexts set"
                               " before the QApplication is made (as wavi_py.app does)")

    def _toggle_rov_view(self, show: bool) -> None:
        if show and self._rov_view is None:
            try:
                if QGuiApplication.platformName() in ("offscreen", "minimal"):
                    raise RuntimeError(f"no OpenGL on the Qt platform '{QGuiApplication.platformName()}'")
                self._check_gl_sharing(self._dome_view)
                from wavi_py.views.rov_view import RovView
                self._rov_view = RovView()
            except Exception as exc:  # noqa: BLE001 — no PyOpenGL / GL context / mesh
                QMessageBox.warning(self, "ROV view", f"Cannot open the ROV view: {exc}\n"
                                    "(pip install PyOpenGL)")
                self._toggle_rov.setChecked(False)
                return
            self._rov_view.closed.connect(lambda: self._toggle_rov.setChecked(False))
            self._timeline_docks.append(self._rov_view.add_timeline(TimelineBar(self._replay)))
            self._timeline_docks[-1].setVisible(self._replay.source is not None)
            self._rov_view.update_view(self._imu_t, self._imu_rows)
        if self._rov_view is not None:
            self._toggle_window(self._rov_view, show)

    # ----- dome map: whisker position labels, the Dome view's holes and calibration
    def set_dome_map(self, dome_map: Optional[DomeMap]) -> None:
        self._dome_map = dome_map
        if dome_map is not None:
            names, gauges = dome_map.whisker_names(NUM_WHISKER), dome_map.gauge_names(2 * NUM_WHISKER)
        else:
            names, gauges = [f"W{w + 1}" for w in range(NUM_WHISKER)], None
        for v in (self._line_view, self._spectrum_view, self._fft_view, self._spec_view):
            v.set_whisker_names(names, gauges)
        if self._dome_view is not None:
            self._dome_view.set_dome_map(dome_map)
        if dome_map is None:
            tip = "Dome map: none (channels W1 .. W16, placeholder dome holes)"
        else:
            tip = f"Dome map: {dome_map.path or ''}\n{dome_map.summary()}"
            if dome_map.uncalibrated():
                tip += f"\nplaceholder gain: {', '.join(dome_map.uncalibrated())}"
        self._map_btn.setToolTip(tip + "\n(whisker positions, sensors and calibration)")
        name = dome_map.path.stem if dome_map is not None and dome_map.path else "set"
        self._map_btn.setText("Map" if dome_map is None else f"Map: {name}")

    def _load_dome_map(self, path) -> None:
        try:
            self.set_dome_map(DomeMap.load(path))
        except DomeMapError as exc:
            QMessageBox.warning(self, "Dome map", str(exc))

    def _choose_dome_map(self) -> None:
        start = self._dome_map.path if self._dome_map is not None and self._dome_map.path \
            else DEFAULT_MAP
        path, _ = QFileDialog.getOpenFileName(self, "Dome map", str(start.parent),
                                              "Dome map (*.json);;All files (*)")
        if path:
            self._load_dome_map(path)

    def _side_views(self) -> tuple:
        """The views besides the stream views that show the status line and alarm."""
        return tuple(v for v in (self._imu_view, self._dome_view, self._rov_view) if v is not None)

    def _set_stream_buttons_enabled(self, on: bool) -> None:
        self._pause_btn.setEnabled(on)
        self._reset_btn.setEnabled(on)
        self._esp32_reset_btn.setEnabled(
            on and self._client is not None and self._client.is_event_stream
            and not self._replaying)
        array = self._transport_combo.currentText().upper() in ("SERVER", "ESP32", "SIM")
        self._mode_combo.setEnabled(array)
        self._selects_edit.setEnabled(array)
        if not on:
            self._reset_sent_at = None
        for v in self._stream_views:
            v.set_stream_controls_enabled(on)

    def _on_despike_toggled(self, on: bool) -> None:
        if self._replaying:
            assert self._client is not None and self._client.replay is not None
            self._client.replay.despike = on          # re-serves from the playhead
            self._line_view.set_despike(on)           # the store's despiked scans
            return
        if on:
            self._despiker = Despiker()
        else:
            self._flush_despike()
            self._despiker = None

    def _flush_despike(self) -> None:
        """A discontinuity (or Despike off): the provisional samples stay as they are
        (raw); the .dat gets them now. The next sample starts a new stream."""
        d = self._despiker
        if d is None:
            return
        if d.pending and self._is_recording and self._dat_writer is not None:
            self._record_final(self._sig[-d.pending:], self._next_idx - d.pending)
        d.reset()

    def _record_final(self, rows: np.ndarray, first_idx: int) -> None:
        """Final samples ``first_idx``... into the .dat, from ``_rec_from`` on."""
        skip = max(0, self._rec_from - first_idx)
        if skip < len(rows):
            self._write_record(rows[skip:], first_idx + skip)
            self._rec_from = first_idx + len(rows)

    def _on_pause_toggled(self, on: bool) -> None:
        self._pause_btn.setText("Resume" if on else "Pause")
        for v in self._stream_views:
            v.set_paused(on)
        if self._replay.active:
            # replay: Pause pauses playback; the views keep drawing (a seek while
            # paused shows the new position)
            self._is_paused = False
            if on:
                self._replay.pause()
            elif self._replay.state != "playing":
                self._replay.play()
            return
        self._is_paused = on
        if self._dome_view is not None:
            self._dome_view.set_paused(on)

    def _show_timelines(self) -> None:
        on = self._replay.source is not None
        for d in self._timeline_docks:
            d.setVisible(on)
        if on and self.height() < 480:
            self.resize(max(self.width(), 760), self.height() + 170)

    def _build_keys(self) -> None:
        """Panel keys: Space pause / play; replay: Left / Right -+5 s (Shift: 1 s),
        , / . previous / next event, [ / ] slower / faster. (The view windows keep
        their own keys; their Space is Pause too.)"""
        m = self._replay

        def key(seq, fn) -> None:
            QShortcut(QKeySequence(seq), self, activated=fn)
        key(Qt.Key.Key_Space, lambda: self._pause_btn.isEnabled() and self._pause_btn.toggle())
        key(Qt.Key.Key_Left, lambda: m.step(-STEP_S))
        key(Qt.Key.Key_Right, lambda: m.step(STEP_S))
        key("Shift+Left", lambda: m.step(-FINE_STEP_S))
        key("Shift+Right", lambda: m.step(FINE_STEP_S))
        key(",", lambda: m.event_step(False))
        key(".", lambda: m.event_step(True))
        key("[", lambda: m.speed_step(False))
        key("]", lambda: m.speed_step(True))

    def _show_time_from_buffer(self, t: float) -> None:
        """Live: a time picked on the timeline is shown from the plot buffers (the
        newest t_buffer_s), centred, at the Line window's current span."""
        n = self._samples_since_clear
        if n <= 0 or not (self._sig_t[-n] <= t <= self._sig_t[-1]):
            self._note(f"{fmt_t(t)} is not in the buffer (the last "
                                     f"{self._t_buffer_s:g} s): pause sooner, or replay the "
                                     "recording.")
            return
        x0, x1 = self._line_view._visible_x_range()
        span = min(x1 - x0, self._t_buffer_s)
        self._line_view.set_time_window(t - span / 2, t + span / 2)
        if self._rov_view is not None:
            self._rov_view.show_at(t)
        if self._dome_view is not None:
            self._dome_view.show_imu_at(t)

    def export_range(self, path: Optional[str] = None,
                     rng: Optional[tuple[float, float]] = None) -> Optional[Path]:
        """Export... (replay): ``rng`` (session seconds; default the loop range, else the
        Line window's time span, else the 10 s before the playhead) saved as a .jsonl
        clip or, by the extension, a .dat (replay/export.py). Asks for the file unless
        ``path`` is given. Returns where it wrote, None if not."""
        idx = self._index if self._replay.active else None
        if idx is None:
            return None
        if rng is None:
            rng = self._replay.loop
        if rng is None and self._line_view.isVisible():
            rng = self._line_view._visible_x_range()
        if rng is None:
            rng = (self._replay.position - 10.0, self._replay.position)
        t0, t1 = max(0.0, float(rng[0])), min(idx.duration, float(rng[1]))
        # the file opened (a .dat replays from its converted copy in the cache)
        opened = Path(self._file_edit.text().strip() or idx.path)
        if path is None:
            stem = f"{opened.stem}_{int(t0)}-{int(math.ceil(t1))}s"
            path, _ = QFileDialog.getSaveFileName(
                self, f"Export {fmt_t(t0)} .. {fmt_t(t1)}", str(opened.with_name(stem + ".jsonl")),
                "Recording clip (*.jsonl);;Whisker .dat (*.dat)")
            if not path:
                return None
        from wavi_py.replay.export import export_dat, export_jsonl
        out = Path(path)
        try:
            same = any(out.resolve() == p.resolve() for p in (idx.path, opened))
        except OSError:
            same = False
        if same:
            QMessageBox.critical(self, "Export failed",
                                 f"{out} is the recording being replayed; choose another file.")
            return None
        try:
            if out.suffix.lower() == ".dat":
                despike = bool(self._client and self._client.replay and self._client.replay.despike)
                what = f"{export_dat(idx, t0, t1, out, despike)} scans" + (
                    " despiked" if despike else "")
            else:
                what = f"{export_jsonl(idx, t0, t1, out)} lines"
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, "Export failed", f"{out}: {exc}")
            return None
        self._note(f"Exported {fmt_t(t0)} .. {fmt_t(t1)} ({what}) to {out}")
        return out

    def _on_line_time_window(self, x0: float, x1: float) -> None:
        """The Line window's visible time range: the IMU window follows it."""
        if self._line_view.isVisible():
            self._imu_view.set_time_window((x0, x1))

    def _on_replay_state(self, state: str) -> None:
        """Playback started / stopped (any control, or the end): the Pause buttons."""
        if state in ("playing", "paused", "eof"):
            paused = state != "playing"
            if self._pause_btn.isChecked() != paused:
                self._pause_btn.setChecked(paused)
        if state == "eof":
            self._note("End of the recording (Play starts over).")

    def zero_at(self, t: Optional[float] = None, window_s: float = 0.25) -> bool:
        """Re-zero (V0, and the Dome's tare) from the buffered samples: the mean over
        ``window_s`` centred on time ``t`` (session seconds; None = the newest
        ``window_s``). Local only, so it works while paused, live or in replay.
        Returns False if there is no data there."""
        src = self.series()                   # replay: the whole recording; live: the buffer
        ext = src.extent(WSK)
        if ext is None:
            return False
        despike = self._store is not None and self._despike_chk.isChecked()
        if t is None:                         # the newest: before the playhead in replay
            end = min(self._replay.position, ext[1]) if self._store is not None else ext[1]
            _, sig = src.get(WSK, slice(None), end - window_s, end, despike)
        else:
            _, sig = src.get(WSK, slice(None), t - window_s / 2, t + window_s / 2, despike)
        if not len(sig):
            return False
        with np.errstate(invalid="ignore"):
            v0 = np.nanmean(sig, axis=0)
        self._v0 = np.where(np.isfinite(v0), v0, self._v0).astype(np.float32)
        if self._dome_view is not None:
            self._dome_view.set_zero(self._v0)
        where = "the newest samples" if t is None else f"t = {t:.2f} s"
        self._note(f"Re-zeroed at {where} ({len(sig)} samples).")
        self._fault_log.append(f"{self._now():%Y-%m-%d %H:%M:%S}  info   re-zero at {where}")
        self._request_redraw()
        return True

    def _request_redraw(self) -> None:
        """Redraw the Line window from the buffer (e.g. a new V0 while paused)."""
        n = self._samples_since_clear
        if n > 0:
            latest = float(self._sig_t[-1])
            self._line_view.update_view(self._sig_t[-n:], self._sig[-n:], self._v0, self._scale,
                                        x_range=(latest - self._t_buffer_s, latest))

    def _now(self) -> datetime:
        """Wall clock: the recording's at the playhead in replay, else now."""
        if self._replay.active:
            wall = self._replay.wall_clock(self._replay.position)
            if wall is not None:
                return wall
        return datetime.now()

    def _on_reset_clicked(self) -> None:
        if self._client is None or not self._client.is_running:
            return
        if self._replaying:
            self.zero_at(None)        # from the buffer: works while playback is paused
            return
        self._client.reset()  # DAQ thread will recompute V0 on its next tick
        if self._dome_view is not None:
            self._dome_view.tare()
        # Skip whatever is already buffered on the bus — those samples were
        # collected under the old V0 and would briefly render with a jump
        # before the new V0 lands.
        self._last_seq = self._client.bus.total_published
        self._flush_despike()                      # before the buffer it reads is cleared
        self._sig.fill(np.nan)
        self._spec_hist.clear()
        self._fft_map.fill(0)
        self._samples_since_spec_update = 0
        self._samples_since_clear = 0
        self._path_label.setText("Resetting V0...")

    def _set_auto_stop(self, secs: float) -> None:
        """Auto-stop after ``secs`` (0: off, the box keeps its time)."""
        if secs > 0:
            self._auto_stop_spin.setValue(int(math.ceil(secs)))      # turns it on
            self._auto_stop_chk.setChecked(True)
        else:
            self._auto_stop_chk.setChecked(False)

    def _on_auto_stop_edited(self, _v: int) -> None:
        self._auto_stop_chk.setChecked(True)          # typing a time means: use it
        self._apply_auto_stop()

    def _on_auto_stop_toggled(self, _on: bool) -> None:
        self._apply_auto_stop()

    def _apply_auto_stop(self) -> None:
        self._auto_stop_s = (float(self._auto_stop_spin.value())
                             if self._auto_stop_chk.isChecked() else 0.0)
        if self._auto_stop_s <= 0:
            self._progress.setValue(0)

    def _on_record_clicked(self) -> None:
        if self._is_recording:
            self._stop_recording()
        elif self._foreign_rec is not None:
            self._stop_foreign_recording()
        else:
            self._start_recording()

    @property
    def _vehicle_live(self) -> bool:
        """Connected to daq_server: the recording is the vehicle's, with acks and a
        status record to follow (not the ESP32 / Sim transports' local tee)."""
        return (self._client is not None and not self._replaying
                and self._client.transport_kind == "server")

    def _start_recording(self, force: bool = False) -> None:
        if self._client is None or self._replaying or self._is_recording or self._rec_cmd:
            return
        tag = self._tag_edit.text().strip()
        if not self._vehicle_live:
            self._begin_local_recording(tag)
            return
        if self._foreign_rec is not None and not force:
            return                                   # the button stops that one instead
        # daq_server: the local .dat / .jsonl start once the server says it started
        # (it refuses while another recording runs, unless forced)
        try:
            self._outpath.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            QMessageBox.critical(self, "Record failed", f"{exc}")
            return
        acks = self._client.bus.get_metadata().get("server_acks") or {}
        dur = self._auto_stop_s if self._auto_stop_s > 0 else None
        if not self._client.send_record_start(tag, duration=dur, force=force, who=self._who):
            self._note("Rec: could not send the command.")
            return
        self._rec_cmd = {"cmd": "rec_start", "tag": tag, "sent": time.monotonic(),
                         "seen": acks.get("rec_start", (0, None))[0]}
        self._path_label.setText("Starting the ROV recording...")
        self._update_record_button()

    def _begin_local_recording(self, tag: str) -> bool:
        """The local .dat (and, event streams, the .jsonl tee) from now on."""
        assert self._client is not None
        start_dt = datetime.now()
        fname = build_datalog_filename(start_dt, tag)
        fullpath = self._outpath / fname
        try:
            self._dat_writer = DatWriter(fullpath, self._nch)
        except OSError as exc:
            QMessageBox.critical(self, "Record failed", f"{exc}")
            return False
        self._record_start_dt = start_dt
        self._is_recording = True
        # from the oldest sample not yet final (the provisional ones come later)
        self._rec_from = self._next_idx - (self._despiker.pending if self._despiker else 0)
        # the full whisker+IMU stream to a local .jsonl next to the .dat
        if self._vehicle_live:
            self._client.start_local_copy(str(fullpath.with_suffix(".jsonl")))
        else:
            self._client.send_record_start(tag, local_copy=str(fullpath.with_suffix(".jsonl")))

        self._update_record_button()
        self._path_label.setText(f"Recording → {fullpath}"
                                 + (f"\nROV: {self._rov_path}" if self._rov_path else ""))
        self._progress.setValue(0)
        self._fault_sidecar_path = fullpath.with_suffix(".faults.txt")
        f = self._fault_state
        if f.get("level") in ("fault", "warn"):     # from before the recording
            since = datetime.fromtimestamp(f.get("since", 0)).strftime("%H:%M:%S")
            self._write_fault_sidecar(f"at record start: {f['level']} {f.get('code')}:"
                                      f" {f.get('msg')} (since {since})")
        return True

    def _stop_recording(self, server_stopped: str = "") -> None:
        """Stop ours: the local files, and (unless ``server_stopped`` says the
        server already did, and why) the vehicle's."""
        d = self._despiker
        if d is not None and d.pending and self._dat_writer is not None:
            self._record_final(self._sig[-d.pending:], self._next_idx - d.pending)
        self._is_recording = False
        if self._fault_sidecar is not None:
            self._fault_sidecar.close()
            self._fault_sidecar = None
        self._fault_sidecar_path = None
        if self._client is not None:
            if server_stopped:
                self._client.stop_local_copy()
            elif self._vehicle_live:
                acks = self._client.bus.get_metadata().get("server_acks") or {}
                if self._client.send_record_stop(path=self._rov_path, who=self._who):
                    # until the ack, a status in flight may still show it running
                    self._rec_cmd = {"cmd": "rec_stop", "own": True, "sent": time.monotonic(),
                                     "seen": acks.get("rec_stop", (0, None))[0]}
            else:
                self._client.send_record_stop()
        rov = self._rov_path
        self._rov_path, self._rov_confirmed, self._rov_duration = None, False, False
        if self._dat_writer is not None:
            try:
                self._dat_writer.close()
            finally:
                p = self._dat_writer.path
                self._dat_writer = None
                text = f"Saved {p}"
                if server_stopped:
                    text += f"\nROV recording {os.path.basename(rov or '')} {server_stopped}"
                self._note(text) if server_stopped else self._path_label.setText(text)
        self._update_record_button()
        self._progress.setValue(0)

    def _stop_foreign_recording(self) -> None:
        """Stop the vehicle's recording that this window didn't start (after asking)."""
        rec = self._foreign_rec
        if rec is None or self._client is None or self._rec_cmd:
            return
        name = os.path.basename(rec.get("path") or "?")
        text = (f"The ROV is recording {name} ({rec.get('secs', 0):.0f} s"
                + (f", started by {rec['by']}" if rec.get("by") else "")
                + ").\n\nThis window didn't start it (another client, or this one before a"
                  " reconnect). Stop it?")
        if QMessageBox.question(self, "Stop ROV recording", text) != QMessageBox.StandardButton.Yes:
            return
        acks = self._client.bus.get_metadata().get("server_acks") or {}
        if not self._client.send_record_stop(path=rec.get("path"), who=self._who, local=False):
            self._note("Stop: could not send the command.")
            return
        self._rec_cmd = {"cmd": "rec_stop", "sent": time.monotonic(),
                         "seen": acks.get("rec_stop", (0, None))[0]}
        self._update_record_button()

    def _tick_rec_ack(self, meta: dict) -> None:
        """The ack of the rec_start / rec_stop this window sent."""
        pend = self._rec_cmd
        if pend is None:
            return
        n, ack = (meta.get("server_acks") or {}).get(pend["cmd"], (0, None))
        if n <= pend["seen"] or ack is None:
            if time.monotonic() - pend["sent"] > REC_ACK_TIMEOUT_S:
                self._rec_cmd = None
                self._note(f"{pend['cmd']}: no reply from the server (the status line shows"
                           " whether the ROV records).")
                self._update_record_button()
            return
        self._rec_cmd = None
        if pend["cmd"] == "rec_stop":
            self._foreign_rec = None
            if pend.get("own") and ack.get("ok", True):
                pass                               # the label says "Saved ..." already
            elif ack.get("ok", True):
                self._note(f"ROV recording stopped: {ack.get('path') or '(none was running)'}")
            else:
                self._note(f"ROV recording not stopped: {ack.get('error', '?')}")
            rec = ack.get("rec")
            if isinstance(rec, dict) and rec.get("on"):
                self._foreign_rec = rec
            self._update_record_button()
            return
        if ack.get("ok", True) and ack.get("path"):
            self._rov_path = str(ack["path"])
            self._rov_confirmed = False
            self._rov_duration = ack.get("duration") is not None
            self._foreign_rec = None
            if not self._begin_local_recording(pend["tag"]):
                # the ROV records, here nothing does: show it as one to stop
                self._foreign_rec = {"on": True, "path": self._rov_path, "secs": 0,
                                     "by": self._who}
                self._rov_path = None
                self._update_record_button()
                return
            if ack.get("replaced"):
                self._note(f"Recording; closed the ROV's previous one, {ack['replaced']}")
            return
        if ack.get("busy"):
            # another recording runs (started since the last status): ask, outside this tick
            rec = ack.get("rec") if isinstance(ack.get("rec"), dict) else {}
            self._foreign_rec = rec or None
            self._update_record_button()
            QTimer.singleShot(0, lambda: self._ask_replace(rec, pend["tag"]))
            return
        self._note(f"ROV recording not started: {ack.get('error', 'no path in the reply')}")
        self._update_record_button()

    def _ask_replace(self, rec: dict, tag: str) -> None:
        name = os.path.basename(rec.get("path") or "?")
        box = QMessageBox(QMessageBox.Icon.Question, "ROV already recording",
                          f"The ROV is already recording {name} ({rec.get('secs', 0):.0f} s"
                          + (f", started by {rec['by']}" if rec.get("by") else "") + ").\n\n"
                          "Replace closes that file and starts yours; Keep leaves it running"
                          " (the Rec button then stops it).", parent=self)
        replace = box.addButton("Replace", QMessageBox.ButtonRole.DestructiveRole)
        box.addButton("Keep", QMessageBox.ButtonRole.RejectRole)
        box.exec()
        if box.clickedButton() is replace and self._client is not None:
            self._foreign_rec = None
            self._tag_edit.setText(tag)
            self._start_recording(force=True)

    def _on_rec_status(self, rec: dict) -> None:
        """The vehicle's recording in a status record: adopt one not ours; follow
        ours (the server may stop it: its duration, another client, a restart)."""
        if not self._vehicle_live or self._rec_cmd is not None:
            return
        on = bool(rec.get("on"))
        if self._is_recording and self._rov_path:
            ours_on = on and _same_rec(rec.get("path"), self._rov_path)
            if ours_on:
                self._rov_confirmed = True
                return
            last = rec.get("last") or {}
            if _same_rec(last.get("path"), self._rov_path):
                why = f"stopped by the server: {last.get('why', '?')}" + (
                    f" ({last['by']})" if last.get("by") and last.get("by") != "server" else "")
            elif self._rov_confirmed:
                why = "is no longer running"
            else:
                return                             # a status from before ours started
            print(f"[MainWindow] ROV recording {why}")
            self._stop_recording(server_stopped=why)
            if on:                                 # and another one runs now
                self._foreign_rec = rec
                self._update_record_button()
            return
        if self._is_recording:
            return
        foreign = rec if on else None
        if (foreign is None) != (self._foreign_rec is None) or foreign is not None:
            self._foreign_rec = foreign
            self._update_record_button()

    def _flag_target(self) -> Optional[tuple[str, bool]]:
        """(the newest vehicle recording's file name, flagged bad) from the status:
        the running one, else the last; None if there is none (or no vehicle)."""
        if not self._vehicle_live:
            return None
        rec = self._last_status_rec.get("rec") or {}
        if rec.get("on") and rec.get("path"):
            return os.path.basename(rec["path"]), rec.get("keep", True) is False
        last = rec.get("last") or {}
        if last.get("path"):
            return os.path.basename(last["path"]), last.get("keep", True) is False
        return None

    def _update_flag_button(self) -> None:
        b, target = self._flag_btn, self._flag_target()
        if target is None:
            b.setText("Mark bad")
            b.setEnabled(False)
            b.setToolTip("Flag the newest ROV recording bad in the Pi's index (trials.jsonl)")
            return
        name, bad = target
        b.setText("Unmark bad" if bad else "Mark bad")
        b.setEnabled(self._flag_cmd is None)
        b.setToolTip(f"{name} is flagged bad; click to keep it again" if bad else
                     f"Flag {name} bad in the Pi's index (trials.jsonl). The file stays;"
                     "\nwhisker_link.convert skips it.")

    def _on_flag_clicked(self) -> None:
        target = self._flag_target()
        if target is None or self._client is None or self._flag_cmd is not None:
            return
        name, bad = target
        if bad:
            note, ok = "", QMessageBox.question(
                self, "Keep recording", f"Keep {name} again (remove the bad flag)?"
            ) == QMessageBox.StandardButton.Yes
        else:
            note, ok = QInputDialog.getText(
                self, "Mark recording bad",
                f"Mark {name} bad?\nThe file stays on the Pi; convert skips it.\n\nNote (optional):")
        if not ok:
            return
        acks = self._client.bus.get_metadata().get("server_acks") or {}
        cmd = {"cmd": "rec_flag", "file": name, "keep": bad, "who": self._who}
        if note.strip():
            cmd["note"] = note.strip()
        if not self._client.send_command(cmd):
            self._note("Mark bad: could not send the command.")
            return
        self._flag_cmd = {"sent": time.monotonic(), "file": name, "keep": bad,
                          "seen": acks.get("rec_flag", (0, None))[0]}
        self._update_flag_button()

    def _tick_flag_ack(self, meta: dict) -> None:
        pend = self._flag_cmd
        if pend is None:
            return
        n, ack = (meta.get("server_acks") or {}).get("rec_flag", (0, None))
        if n <= pend["seen"] or ack is None:
            if time.monotonic() - pend["sent"] > REC_ACK_TIMEOUT_S:
                self._flag_cmd = None
                self._note("Mark bad: no reply from the server.")
                self._update_flag_button()
            return
        self._flag_cmd = None
        if ack.get("ok", True):
            self._note(f"{ack.get('file', pend['file'])} "
                       + ("kept again" if pend["keep"] else "marked bad (the file stays)"))
            # until the next status says so
            rec = self._last_status_rec.get("rec") or {}
            for r in (rec, rec.get("last") or {}):
                if r.get("path") and os.path.basename(r["path"]) == pend["file"]:
                    r["keep"] = pend["keep"]
        else:
            err = str(ack.get("error", "?"))
            if "unknown command" in err:
                err = "this daq_server has no recording index (needs 0.3)"
            self._note(f"Mark bad failed: {err}")
        self._update_flag_button()

    def _update_record_button(self) -> None:
        self._update_flag_button()
        b = self._record_btn
        if self._is_recording:
            b.setText("■ Stop")
            b.setStyleSheet("QPushButton { background-color: #c00; color: white; font-weight: bold; }")
            b.setToolTip("Stop this recording (local files and the ROV's)")
        elif self._rec_cmd is not None:
            b.setText("Starting…" if self._rec_cmd["cmd"] == "rec_start" else "Stopping…")
            b.setStyleSheet("QPushButton { color: #888; font-weight: bold; }")
            b.setToolTip("Waiting for the server's reply")
        elif self._foreign_rec is not None:
            r = self._foreign_rec
            b.setText(f"■ Stop ROV rec {r.get('secs', 0):.0f}s")
            b.setStyleSheet("QPushButton { background-color: #d07000; color: white; font-weight: bold; }")
            b.setToolTip(f"The ROV is recording {r.get('path') or '?'}"
                         + (f"\nstarted by {r['by']}" if r.get("by") else "")
                         + (f", stops in {r['left']:.0f} s" if r.get("left") is not None else "")
                         + "\nThis window didn't start it, so nothing is recorded locally."
                           "\nClick to stop it (asks first).")
        else:
            b.setText("● Rec")
            b.setStyleSheet("QPushButton { color: #c00; font-weight: bold; }")
            b.setToolTip("")
        b.setEnabled(self._client is not None and not self._replaying and self._rec_cmd is None)
        # the server took the duration at rec_start: changing it here would only move
        # the client's fallback, so it is fixed until that recording ends
        locked = self._is_recording and self._rov_duration
        for w in (self._auto_stop_chk, self._auto_stop_spin):
            w.setEnabled(not locked)
        self._auto_stop_chk.setToolTip(
            "Fixed while this recording runs: the ROV stops it at the time it was started with."
            if locked else
            "Stop each recording after the time in the box. With daq_server the server\n"
            "stops it itself, so it also stops if this window is gone.")

    # ------------------------------------------------------------------
    # the data tick
    # ------------------------------------------------------------------

    def _on_tick(self) -> None:
        if self._client is None:
            return
        if not self._client.is_running:
            self._on_link_lost()
            return
        # the buses before the metadata: a replay seek mark published after a bus read
        # then refers to an index at or after what was read (pending, cut later),
        # never to samples already ingested into the old buffers
        samples, times, new_seq = self._client.bus.read_since_t(self._last_seq, block=False)
        if self._client.is_event_stream:
            self._tick_server_extras()
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
        # A scan-mode switch (or a replay seek) inside this block: the samples before
        # it go into the old buffers, then the buffers start over (at the new rate).
        first = new_seq - n_new
        pos = 0
        while self._pending_switches and self._pending_switches[0][0] < new_seq:
            at, fs = self._pending_switches.pop(0)
            cut = min(max(at - first, pos), n_new)
            if cut > pos:
                self._ingest(samples[pos:cut], first + pos, times[pos:cut])
                pos = cut
            self._switch_rate(fs, at)
        if pos < n_new:
            self._ingest(samples[pos:], first + pos, times[pos:])
        if self._is_recording:
            self._maybe_auto_stop()

        # Time axis: each buffered sample's time (session seconds; from the stream's
        # clocks, else the sample index since the stream start / fs).
        latest_t = float(self._sig_t[-1])
        t_axis = self._sig_t
        if not self._replay.active and self._replay.source is not None:
            self._replay.set_live_position(latest_t)
            if time.monotonic() - self._live_events_t > 1.0:
                self._live_events_t = time.monotonic()
                self._replay.sourceUpdated.emit()           # more events / depth

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

            # FFT / spectrogram: a column every hop samples
            if self._samples_since_spec_update >= self._spec_hop:
                self._update_fft_and_spec()

    def _ingest(self, samples: np.ndarray, first_idx: int,
                times: Optional[np.ndarray] = None) -> None:
        """Samples ``first_idx``... (all at the current rate; their times, NaN where
        the stream has none) into the rolling buffer and the recording."""
        n_new = samples.shape[0]
        if first_idx != self._next_idx:                # skipped / dropped samples: a gap
            self._flush_despike()
        # Despike (live): `out` also re-gives earlier provisional samples, now final
        if self._despiker is not None:
            out = self._despiker.push(samples).astype(np.float32)
            n_final = len(out) - self._despiker.pending
        else:
            out, n_final = samples, n_new
        n_back = len(out) - n_new
        cleaned = out[n_back:]
        t_idx = self._t_of(first_idx) + np.arange(n_new) / self._fs
        t = t_idx if times is None else np.where(np.isfinite(times), times, t_idx)

        # Shift the rolling buffer and append cleaned samples.
        if n_new >= self._ns_tot:
            self._sig[:] = cleaned[-self._ns_tot:]
            self._sig_t[:] = t[-self._ns_tot:]
        else:
            self._sig[:-n_new] = self._sig[n_new:]
            self._sig[-n_new:] = cleaned
            self._sig_t[:-n_new] = self._sig_t[n_new:]
            self._sig_t[-n_new:] = t
        if n_back:                                   # the provisional ones, patched
            m = min(n_back, self._ns_tot - n_new)
            if m > 0:
                self._sig[-n_new - m:-n_new] = out[n_back - m:n_back]
        self._next_idx = first_idx + n_new
        self._samples_since_clear = min(self._samples_since_clear + n_new, self._ns_tot)
        self._samples_since_spec_update += n_new

        # Recording path: the samples final now, with their wall-clock timestamps.
        if self._is_recording and self._dat_writer is not None:
            self._record_final(out[:n_final], first_idx - n_back)

    def _tick_esp32_mode(self, meta: dict) -> None:
        """New INFO frames: the reported scan config, a rate switch at the sample
        where it arrived (scan_hz from INFO; legacy: the rate measured before for
        these selects, else an estimate from the select count, until the measured
        rate comes ~2 s later),
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
                mask = cfg.get("sel_mask", 0xFF)
                fs = (cfg.get("scan_hz")
                      or (self._legacy_fs.get(mask) if cfg.get("mode") == 0 else None)
                      or (legacy_nominal_hz(mask) if was_locked else self._fs))
                if self._cfg_key is not None or abs(fs - self._fs) > 1e-6 * fs:
                    self._pending_switches.append((int(at), float(fs)))
                    print(f"[MainWindow] ESP32 scan config {describe_cfg(cfg)}: "
                          f"{fs:.3f} Hz from sample {at}")
            self._cfg_key = key
            if not self._replaying:              # a recording is not kept in a mode
                self._send_esp32(self._keeper.on_info(cfg))
        n_txt, msg = meta.get("esp32_txt") or (0, "")
        if n_txt > self._txt_seen:
            self._txt_seen = n_txt
            if not self._replaying:
                self._keeper.on_text(msg)
        if not self._replaying:
            self._send_esp32(self._keeper.tick())
        self._show_mode_note()

    def _tick_server_extras(self) -> None:
        """IMU view + a one-line server status (rates, vehicle-side recording)."""
        assert self._client is not None
        rows, t, seq = self._client.imu_bus.read_since_t(self._imu_last_seq, block=False)
        meta = self._client.bus.get_metadata()                  # after the reads (_on_tick)
        if self._replay.active:
            self._replay.poll()
            self._tick_replay(meta)
            if rows is not None:                # rows before a seek just seen: dropped
                drop = min(len(rows), max(0, self._imu_last_seq - (seq - len(rows))))
                rows, t = rows[drop:], t[drop:]
                if not len(rows):
                    rows = None
                    self._imu_last_seq = max(self._imu_last_seq, seq)
        self._tick_esp32_mode(meta)
        self._tick_esp32_reset(meta)
        self._tick_rec_ack(meta)
        self._tick_flag_ack(meta)
        self._tick_faults(meta)
        self._maybe_apply_measured_rate(meta.get("fs_measured"))
        st = meta.get("server_status") or {}
        if rows is not None:
            self._imu_last_seq = max(self._imu_last_seq, seq)
            # the IMU's own times (session seconds, as the Line window's); the newest
            # t_buffer_s of them. Rows without a time: the RAW_IMU rate.
            hz = min(float(st.get("hz", {}).get("RAW_IMU", 0)) or 100.0, 400.0)
            if not np.isfinite(t).all():
                last = self._imu_t[-1] if len(self._imu_t) else 0.0
                t = np.where(np.isfinite(t), t, last + np.arange(1, len(rows) + 1) / hz)
            self._imu_rows = np.vstack([self._imu_rows, rows])
            self._imu_t = np.concatenate([self._imu_t, t])
            keep = int(np.searchsorted(self._imu_t, self._imu_t[-1] - self._t_buffer_s))
            keep = max(keep, len(self._imu_t) - int(self._t_buffer_s * 400))   # 400 Hz at most
            if keep:
                self._imu_rows, self._imu_t = self._imu_rows[keep:], self._imu_t[keep:]
            if not self._is_paused and self._imu_view.isVisible():
                self._imu_view.update_view(self._imu_t, self._imu_rows)
            if not self._is_paused and self._rov_view is not None and self._rov_view.isVisible():
                self._rov_view.update_view(self._imu_t, self._imu_rows)
            if not self._is_paused and self._dome_view is not None and self._dome_view.isVisible():
                self._dome_view.update_imu(self._imu_t, self._imu_rows)
        if st and st.get("t", 0) != self._last_status_t:
            self._last_status_t = st.get("t", 0)
            self._last_status_rec = st
            if isinstance(st.get("rec"), dict):
                self._on_rec_status(st["rec"])
                self._update_flag_button()
            self._set_fault_state(st.get("esp32", {}).get("fault") or {})
            self._note_beat(st.get("esp32", {}).get("beat"))
            self._show_status(st)
        elif (self._replay.active and self._last_status_rec
              and time.monotonic() - self._status_shown_t > 0.25):
            self._show_status(self._last_status_rec)        # the REPLAY position moves

    def _tick_replay(self, meta: dict) -> None:
        """Replay seeks (``replay_marks``): the buffers start over at the marked bus
        index, the IMU rows at the marked IMU index; the ESP32 state is forgotten
        (a backward seek is not a reboot); the Dome reads on from the trail's worth
        before the new position."""
        for gen, op, at, imu_at, pos in meta.get("replay_marks") or ():
            if op == "seek" and gen > self._replay_gen:
                self._replay_gen, self._replay_seek_at = gen, at
                self._pending_switches.append((int(at), self._fs))
                self._pending_switches.sort(key=lambda x: x[0])
                self._imu_rows = np.empty((0, 9), dtype=np.float32)
                self._imu_t = np.empty(0)
                self._imu_last_seq = max(self._imu_last_seq, int(imu_at))
                self._beat_uptime, self._reboot_note = None, ""
                self._last_status_t, self._last_status_rec = 0.0, {}
                self._set_fault_state({})
                if gen > 1:
                    wall = self._replay.wall_clock(pos)
                    self._fault_log.append(f"{wall or datetime.now():%Y-%m-%d %H:%M:%S}  "
                                           f"----   seek to {fmt_t(pos)}")
            elif op == "live" and gen == self._replay_gen and self._replay_seek_at is not None:
                if self._dome_view is not None:
                    self._dome_view.discontinuity(max(self._replay_seek_at, int(at)))
                self._replay_seek_at = None

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
        for v in (*self._stream_views, *self._side_views()):
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

    def _note_beat(self, beat: Optional[dict]) -> None:
        """The heartbeat: a reboot (its uptime went back, or the first beat after
        connecting is from a fresh boot) is reported once, with its reset reason,
        in the Fault log and for ``REBOOT_NOTE_S`` in the status line."""
        if not beat:
            return
        up = beat.get("uptime_ms", 0)
        prev, self._beat_uptime = self._beat_uptime, up
        if (prev is None and up < REBOOT_NOTE_S * 1000) or (prev is not None and up + 1000 < prev):
            t = self._now() - timedelta(milliseconds=up)
            self._reboot_note = (f"ESP32 {'booted' if prev is None else 'rebooted'} {t:%H:%M:%S}"
                                 f" (reset: {beat.get('reset_name', '?')})")
            self._reboot_note_until = time.monotonic() + REBOOT_NOTE_S
            self._fault_log.append(f"{t:%Y-%m-%d %H:%M:%S}  info   {self._reboot_note}")

    def _show_status(self, st: dict) -> None:
        self._status_shown_t = time.monotonic()
        link, adc = self._format_status(st)
        text = f"{link} | {adc}"
        beat = st.get("esp32", {}).get("beat")
        style = ("QLabel { color: #c00000; font-weight: bold; }"
                 if beat and not beat.get("converting", True) else "")
        # the line window's status bar always shows it; the panel's label
        # keeps "Recording -> path" while recording
        for v in (*self._stream_views, *self._side_views()):
            v.set_stream_status(text)
            if v._stream_status.styleSheet() != style:
                v._stream_status.setStyleSheet(style)
        if not self._is_recording and time.monotonic() >= self._note_until:
            self._path_label.setText(f"{link}\n{adc}")
            self._path_label.setStyleSheet(style or "QLabel { color: #555; }")

    def _note(self, text: str, hold_s: float = NOTE_HOLD_S) -> None:
        """A command's reply or an action's result in the panel label, kept there for
        ``hold_s`` before the stream status line (1 Hz) takes the label back."""
        self._path_label.setText(text)
        self._path_label.setStyleSheet("")
        self._note_until = time.monotonic() + hold_s

    def _format_status(self, st: dict) -> tuple[str, str]:
        """The server / ESP32 status record as two lines: the link (where, rates,
        recording, fault) and the ADC (error counters, scan mode, heartbeat). The
        view windows show them as one line."""
        assert self._client is not None
        hz = st.get("hz", {})
        rec = st.get("rec", {})
        if self._client.has_vehicle:
            pi_rec = f"ROV rec {rec.get('secs', 0):.0f}s" if rec.get("on") else "ROV rec off"
            if rec.get("on") and rec.get("keep") is False:
                pi_rec += " (marked bad)"
            if rec.get("on") and not self._replaying:
                if not (self._rov_path and _same_rec(rec.get("path"), self._rov_path)):
                    pi_rec += (f" {os.path.basename(rec.get('path') or '')}"
                               + (f" by {rec['by']}" if rec.get("by") else "") + " (not ours)")
                if rec.get("left") is not None:
                    pi_rec += f", stops in {rec['left']:.0f}s"
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
        if self._keeper.want is not None and self._keeper.note and not self._replaying:
            errs += f" | mode: {self._keeper.note}"
        # the heartbeat and a recent reboot go last: the line is cut from the left
        b = e.get("beat")
        if b:                                    # the firmware's 1 Hz heartbeat
            errs += (f" | ADC {b.get('conv_hz', 0):.0f} conv/s" if b.get("converting", True)
                     else f" | ADC NOT converting ({b.get('conv_hz', 0):.0f} conv/s)")
        if self._reboot_note and time.monotonic() < self._reboot_note_until:
            errs += f" | {self._reboot_note}"
        f = e.get("fault") or {}
        fault = f" | FAULT {f.get('code')}" if f.get("level") == "fault" else ""
        imu = (f"IMU {hz.get('RAW_IMU', 0):g} Hz | att {hz.get('ATTITUDE', 0):g} Hz | "
               if self._client.has_vehicle else "")
        where = self._client.endpoint_str
        if self._replay.active:
            # so a screenshot can't pass for live
            sp = self._replay.speed
            where = (f"REPLAY {'max' if sp is None else f'{sp:g}x'} "
                     f"{self._replay.position:.1f}/{self._replay.duration:.1f} s | {where}")
        return (f"{where} | whisker {self._fs:.2f} Hz | {imu}{pi_rec}{fault}", errs)

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
        return amp_spectrum(x, self._scale)

    def _update_fft_and_spec(self) -> None:
        """The spectrogram columns that fell due (one per ``hop`` samples, each the
        spectrum of the ``fft_window`` seconds ending there; ``do_fft`` in
        wavi.m:901-914, with the Spectrum window's method) and the FFT frame."""
        hop, n = self._spec_hop, self._n_fft_window
        k = self._samples_since_spec_update // hop
        self._samples_since_spec_update -= k * hop
        # column j ends this many samples before the newest one
        back = self._samples_since_spec_update + hop * np.arange(min(k, self._spec_cols) - 1, -1, -1)
        # live Despike: the newest `pending` samples are still raw (provisional); the
        # columns end before them, so a spike fixed later is not kept in a column or in
        # the FFT's Box over time history (the spectral views run Despike's lag behind)
        back = back + (self._despiker.pending if self._despiker is not None else 0)
        back = back[back + n <= self._samples_since_clear]          # whole window buffered
        if back.size == 0:
            return
        ends = self._ns_tot - back
        frames = amp_spectra(self._sig[ends[:, None] - n + np.arange(n)], self._scale)
        frames = frames.astype(np.float32)
        self._spec_hist.push(frames)
        self._fft_map = frames[-1]
        # the spectrogram history above is kept regardless; the windows only mark
        # themselves dirty here and read the data when they redraw
        if self._fft_view.isVisible():
            # every frame of this tick, keyed on its last sample's (data) time
            self._fft_view.update_view(self._fft_map, self._fs, frames, self._sig_t[ends - 1])
        if self._spec_view.isVisible():
            self._spec_view.update_view(self._spec_hist, self._fs, hop / self._fs)

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
        # a server that took the duration stops it itself (its status ends ours here);
        # this is the fallback, for an older server or a lost status
        limit = self._auto_stop_s + (SERVER_STOP_GRACE_S if self._rov_duration else 0.0)
        if elapsed >= limit:
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
            for v in (self._dome_view, self._rov_view):
                if v is not None:
                    v.close()
        super().closeEvent(event)
