"""Live visualization window for viz-mode server.

VizWindow  — interactive matplotlib figure (runs in its own subprocess)
VizProcess — spawns a VizWindow subprocess and forwards messages via a Queue
_viz_worker — subprocess entry point
"""

from __future__ import annotations

import multiprocessing as mp
import queue as _queue_mod

import numpy as np

# ── constants ────────────────────────────────────────────────────────────────

_TAIL_FRAMES   = 200    # recent-trajectory tail length (frames, not viz-frames)
_HISTORY_S     = 5.0    # rolling signal window (seconds)
_QUEUE_MAXSIZE = 500
_PAUSE_S       = 0.02   # ~50 Hz GUI

_WHISKER_ROWS  = 3
_WHISKER_COLS  = 3
_WHISKER_COUNT = _WHISKER_ROWS * _WHISKER_COLS   # 9

# Panel index → 1-indexed whisker number.
# top row: 3 2 1 | middle: 6 5 4 | bottom: 9 8 7
_PANEL_TO_WHISKER = [3, 2, 1, 6, 5, 4, 9, 8, 7]

_CARRIAGE_SZ_MM = 50    # half-side of the whisker-array square (mm)


# ── helpers ──────────────────────────────────────────────────────────────────

def _choose_signal_limit(frames: list[np.ndarray], n_ch_total: int) -> float:
    arr = np.stack(frames)
    whisker_data = arr[:, 5:]
    peak = float(np.max(np.abs(whisker_data)))
    return max(peak, 1e-6)


# ── VizWindow ────────────────────────────────────────────────────────────────

class VizWindow:
    """Interactive matplotlib figure. Must be created inside the viz subprocess."""

    def __init__(self, config: dict):
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        self._plt = plt

        self._n_rl = config.get("n_rl_interval", 4)
        self._n_ch = config.get("n_ch_total", 23)
        self._path_xy_default = (config.get("path_data") or [[]])[0]

        # episode buffers (one entry per individual data frame, not per viz frame)
        self._x_buf:   list[float]      = []
        self._y_buf:   list[float]      = []
        self._t_buf:   list[float]      = []   # elapsed seconds
        self._frames:  list[np.ndarray] = []
        self._t0_ms:   float | None     = None
        self._signal_limit: float | None = None
        self._episode_ended: bool        = False
        self._val_texts: list            = []
        self._dragging:  bool            = False
        self._slider        = None
        self._slider_ax     = None
        self._xrange_slider = None
        self._xrange_ax     = None
        self._ylim_slider   = None
        self._ylim_ax       = None

        self._build_figure()

    # ── figure setup ─────────────────────────────────────────────────────────

    def _build_figure(self):
        plt = self._plt
        from matplotlib.gridspec import GridSpec
        from matplotlib.patches import Rectangle, Ellipse
        from matplotlib.transforms import blended_transform_factory, Affine2D
        import matplotlib.cm as cm
        import matplotlib.colors as mcolors

        self._fig = plt.figure(figsize=(18, 10), dpi=100)
        try:
            self._fig.canvas.manager.set_window_title("Live Sensor Visualization")
        except Exception:
            pass

        gs = GridSpec(4, 3, figure=self._fig,
                      height_ratios=[2, 1, 1, 1],
                      hspace=0.30, wspace=0.28)

        # ── trajectory panel ─────────────────────────────────────────────────
        ax_t = self._fig.add_subplot(gs[0, 0:2])
        ax_t.set_title("Trajectory", fontsize=9)
        ax_t.set_xlabel("X (mm)", fontsize=8)
        ax_t.set_ylabel("Y (mm)", fontsize=8)
        ax_t.set_xlim([-500, 4500])
        ax_t.set_ylim([-300, 1200])
        ax_t.set_aspect("equal", adjustable="box")
        ax_t.invert_xaxis()
        ax_t.invert_yaxis()
        ax_t.xaxis.tick_top()
        ax_t.xaxis.set_label_position("top")
        ax_t.yaxis.tick_right()
        ax_t.yaxis.set_label_position("right")
        ax_t.tick_params(labelsize=7)
        ax_t.add_patch(Rectangle((0, 0), 3800, 850,
                                  fill=False, edgecolor="red", linewidth=1.5))
        self._path_line, = ax_t.plot([], [], color="0.55", linestyle="--",
                                      linewidth=1.5, label="Target path")
        self._full_traj_line, = ax_t.plot([], [], color="0.80", linewidth=1.0,
                                           zorder=1, label="_nolegend_")
        self._traj_line, = ax_t.plot([], [], color="tab:blue",
                                      linewidth=1.2, label="Trajectory")
        self._tail_line, = ax_t.plot([], [], color="#4DBEEE",
                                      linewidth=2.0, label="Recent tail")

        # rotatable square representing the whisker array
        sz = _CARRIAGE_SZ_MM
        self._carriage_patch = Rectangle((-sz, -sz), 2 * sz, 2 * sz,
                                          facecolor="tab:orange", alpha=0.55,
                                          edgecolor="k", linewidth=1.5, zorder=4)
        ax_t.add_patch(self._carriage_patch)
        self._Affine2D = Affine2D

        self._info_text = ax_t.text(
            0.01, 0.03, "", transform=ax_t.transAxes,
            ha="left", va="bottom", fontsize=8, fontweight="bold",
            bbox={"facecolor": "white", "edgecolor": "none", "pad": 2})
        ax_t.legend(loc="lower center", ncol=3, fontsize=7)
        self._ax_traj = ax_t

        # ── whisker layout panel ─────────────────────────────────────────────
        ax_l = self._fig.add_subplot(gs[0, 2])
        ax_l.set_title("Whisker Layout", fontsize=9)
        ax_l.set_xlim(-0.8, 3.0)
        ax_l.set_ylim(-0.8, 3.0)
        ax_l.set_aspect("equal")
        ax_l.axis("off")

        self._layout_cmap = cm.viridis
        self._layout_norm = mcolors.Normalize(vmin=0.0, vmax=1.0)
        self._layout_patches: list = []

        ew, eh = 0.80, 0.80 / 3.0
        qx, qy = [], []
        for pi in range(_WHISKER_COUNT):
            col_pos = pi % _WHISKER_COLS
            row_pos = 2 - pi // _WHISKER_COLS
            qx.append(col_pos)
            qy.append(row_pos)
            wn = _PANEL_TO_WHISKER[pi]
            e = Ellipse((col_pos, row_pos), width=ew, height=eh,
                        facecolor=self._layout_cmap(0.0),
                        edgecolor="k", linewidth=1.2, zorder=3)
            ax_l.add_patch(e)
            ax_l.text(col_pos, row_pos, str(wn),
                      ha="center", va="center",
                      fontsize=9, fontweight="bold", color="white", zorder=4)
            self._layout_patches.append(e)

        # quiver arrows: MD (drag) as x-component, ML (lift) as y-component
        self._layout_quiver = ax_l.quiver(
            qx, qy, [0.0] * _WHISKER_COUNT, [0.0] * _WHISKER_COUNT,
            color=(0.2, 0.2, 0.2), scale=800.0, scale_units="xy",
            width=0.03, headwidth=1.6, headlength=1.6, headaxislength=1.6, zorder=5)
        self._ax_layout = ax_l

        # ── signal panels (9 whiskers) ────────────────────────────────────────
        self._ax_w      = []
        self._ml_lines  = []
        self._md_lines  = []
        self._cur_lines = []
        self._val_texts = []

        for pi in range(_WHISKER_COUNT):
            grid_row = 1 + pi // _WHISKER_COLS
            grid_col = pi % _WHISKER_COLS
            wn = _PANEL_TO_WHISKER[pi]

            ax = self._fig.add_subplot(gs[grid_row, grid_col])
            ax.grid(True, alpha=0.2)
            ax.tick_params(labelsize=6)

            if grid_row == 3:
                ax.set_xlabel("Time (s)", fontsize=7)
            else:
                ax.tick_params(labelbottom=False)
            if grid_col == 0:
                ax.set_ylabel("Moment", fontsize=7)

            ax.text(0.02, 0.93, f"W{wn}",
                    transform=ax.transAxes, fontsize=8, fontweight="bold",
                    va="top", ha="left",
                    bbox={"facecolor": "white", "edgecolor": "none",
                          "alpha": 0.7, "pad": 1})

            ml, = ax.plot([], [], color="tab:blue",   linewidth=1.2, label="ML")
            md, = ax.plot([], [], color="tab:orange",  linewidth=1.2, label="MD")
            cur  = ax.axvline(0.0, color="k", linestyle=":", linewidth=0.8)

            if pi == 0:
                ax.legend(loc="upper right", fontsize=6)

            trans = blended_transform_factory(ax.transData, ax.transAxes)
            vt = ax.text(0.0, 0.96, "", transform=trans, fontsize=9,
                         va="top", ha="left", zorder=5,
                         bbox={"facecolor": "white", "edgecolor": "none",
                               "alpha": 0.75, "pad": 1})

            self._ax_w.append(ax)
            self._ml_lines.append(ml)
            self._md_lines.append(md)
            self._cur_lines.append(cur)
            self._val_texts.append(vt)

        self._fig.subplots_adjust(bottom=0.05)
        self._fig.canvas.mpl_connect("button_press_event",  self._on_press)
        self._fig.canvas.mpl_connect("button_release_event", self._on_release)
        self._fig.canvas.mpl_connect("motion_notify_event",  self._on_motion)
        plt.show(block=False)
        plt.pause(0.05)

    # ── public API ────────────────────────────────────────────────────────────

    def start_episode(self, state: list[float], path_xy: list | None = None):
        self._x_buf.clear()
        self._y_buf.clear()
        self._t_buf.clear()
        self._frames.clear()
        self._t0_ms         = None
        self._signal_limit  = None
        self._episode_ended = False

        for ax in self._ax_w:
            ax.set_autoscaley_on(True)
            ax.relim()

        for ax_attr in ('_slider_ax', '_xrange_ax', '_ylim_ax'):
            ax = getattr(self, ax_attr, None)
            if ax is not None:
                try:
                    ax.remove()
                except Exception:
                    pass
                setattr(self, ax_attr, None)
        self._slider = self._xrange_slider = self._ylim_slider = None
        self._fig.subplots_adjust(bottom=0.05)

        for line in (self._full_traj_line, self._traj_line, self._tail_line):
            line.set_data([], [])
        self._info_text.set_text("")
        for i in range(_WHISKER_COUNT):
            self._ml_lines[i].set_data([], [])
            self._md_lines[i].set_data([], [])
            self._val_texts[i].set_text("")

        raw_path = path_xy if path_xy else self._path_xy_default
        if raw_path:
            self._path_line.set_data([p[0] for p in raw_path],
                                      [p[1] for p in raw_path])
        else:
            self._path_line.set_data([], [])

        self._ingest_state(state)
        self._draw_frame(len(self._frames) - 1)

    def ingest_only(self, state: list[float]):
        """Ingest state into buffers without redrawing (for batch updates)."""
        self._ingest_state(state)

    def update_frame(self, state: list[float]):
        self._ingest_state(state)
        self._draw_frame(len(self._frames) - 1)

    def end_episode(self):
        if not self._frames:
            return

        if self._signal_limit is None:
            self._signal_limit = _choose_signal_limit(self._frames, self._n_ch)
        if self._signal_limit:
            for ax in self._ax_w:
                ax.set_autoscaley_on(False)
                ax.set_ylim(-self._signal_limit, self._signal_limit)

        self._episode_ended = True
        self._full_traj_line.set_data(self._x_buf, self._y_buf)
        t_total = self._t_buf[-1] if self._t_buf else 1.0
        for ax in self._ax_w:
            ax.set_xlim(0.0, max(t_total, 0.1))

        self._fig.subplots_adjust(bottom=0.11)
        from matplotlib.widgets import Slider, RangeSlider

        # frame navigation slider (full width, very bottom)
        self._slider_ax = self._fig.add_axes([0.10, 0.01, 0.80, 0.018])
        n = max(1, len(self._frames) - 1)
        self._slider = Slider(self._slider_ax, "Frame", 0, n,
                              valinit=n, valstep=1)
        self._slider.on_changed(lambda v: self._draw_frame(int(round(v))))

        # force layout so get_position() is accurate
        self._fig.canvas.draw()
        pos0 = self._ax_w[0].get_position()   # first signal panel (Whisker 3)
        t_total = self._t_buf[-1] if self._t_buf else 1.0
        lim     = self._signal_limit or 1.0
        sh = 0.018   # slider height

        # x-range (time) slider — same horizontal span as ax_w[0]
        self._xrange_ax = self._fig.add_axes(
            [pos0.x0, 0.055, pos0.width, sh])
        self._xrange_slider = RangeSlider(
            self._xrange_ax, "t (s)", 0.0, t_total,
            valinit=(0.0, t_total))
        self._xrange_slider.on_changed(self._on_xrange)

        # y-limit slider — vertical, left of signal panels
        pos6   = self._ax_w[6].get_position()   # bottom-left panel
        v_bot  = pos6.y0
        v_top  = pos6.y0 + pos6.height
        self._ylim_ax = self._fig.add_axes(
            [pos0.x0 - 0.05, v_bot, 0.015, v_top - v_bot])
        self._ylim_slider = Slider(
            self._ylim_ax, "Y", lim * 0.05, lim * 2.0,
            valinit=lim, orientation="vertical")
        self._ylim_slider.on_changed(self._on_ylim)

        self._plt.draw()

    # ── internal helpers ──────────────────────────────────────────────────────

    def _ingest_state(self, state: list[float]):
        """Append all n_rl_interval rows from a viz frame to the buffers."""
        mat = np.array(state, dtype=np.float64).reshape(self._n_rl, self._n_ch)
        for row in mat:
            t_ms = float(row[0])
            if self._t0_ms is None:
                self._t0_ms = t_ms
            self._t_buf.append((t_ms - self._t0_ms) / 1000.0)
            self._x_buf.append(float(row[1]))
            self._y_buf.append(float(row[2]))
            self._frames.append(row)

    def _draw_frame(self, frame_idx: int):
        if not self._frames:
            return
        frame_idx = max(0, min(frame_idx, len(self._frames) - 1))

        x_arr  = np.array(self._x_buf[:frame_idx + 1])
        y_arr  = np.array(self._y_buf[:frame_idx + 1])
        t_now  = self._t_buf[frame_idx] if self._t_buf else 0.0
        frame  = self._frames[frame_idx]

        # ── trajectory ────────────────────────────────────────────────────────
        self._traj_line.set_data(x_arr, y_arr)
        tail_start = max(0, frame_idx + 1 - _TAIL_FRAMES)
        self._tail_line.set_data(x_arr[tail_start:], y_arr[tail_start:])

        # rotated carriage square
        vx, vy = float(frame[3]), float(frame[4])
        angle_deg = float(np.degrees(np.arctan2(vy, vx))) if (vx**2 + vy**2) > 1e-6 else 0.0
        t_patch = (self._Affine2D()
                   .rotate_deg(angle_deg)
                   .translate(x_arr[-1], y_arr[-1])
                   + self._ax_traj.transData)
        self._carriage_patch.set_transform(t_patch)

        self._info_text.set_text(
            f"t={t_now:.1f}s  x={x_arr[-1]:.0f}mm  y={y_arr[-1]:.0f}mm"
        )

        # ── whisker layout: ellipse colours + quiver arrows ───────────────────
        lim = self._signal_limit or 1.0
        u_arr, v_arr = [], []
        for pi in range(_WHISKER_COUNT):
            w     = _PANEL_TO_WHISKER[pi]
            ml_v  = float(frame[5 + (w - 1) * 2])
            md_v  = float(frame[5 + (w - 1) * 2 + 1])
            norm  = float(np.clip(max(abs(ml_v), abs(md_v)) / lim, 0.0, 1.0))
            self._layout_patches[pi].set_facecolor(
                self._layout_cmap(self._layout_norm(norm)))
            u_arr.append(md_v)   # drag → x direction
            v_arr.append(ml_v)   # lift → y direction
        self._layout_quiver.set_UVC(u_arr, v_arr)

        # ── signal traces ─────────────────────────────────────────────────────
        if self._episode_ended:
            t_win    = np.array(self._t_buf)
            data_arr = np.array(self._frames)
            mask     = np.ones(len(t_win), dtype=bool)
        else:
            t_arr_all = np.array(self._t_buf[:frame_idx + 1])
            t_win_lo  = max(0.0, t_now - _HISTORY_S)
            mask      = t_arr_all >= t_win_lo
            t_win     = t_arr_all[mask]
            data_arr  = np.array(self._frames[:frame_idx + 1])

        for pi in range(_WHISKER_COUNT):
            w      = _PANEL_TO_WHISKER[pi]
            ml_ch  = 5 + (w - 1) * 2
            md_ch  = 5 + (w - 1) * 2 + 1
            ml_win = data_arr[mask, ml_ch]
            md_win = data_arr[mask, md_ch]
            self._ml_lines[pi].set_data(t_win, ml_win)
            self._md_lines[pi].set_data(t_win, md_win)
            self._cur_lines[pi].set_xdata([t_now, t_now])

            if not self._episode_ended:
                t_win_lo = max(0.0, t_now - _HISTORY_S)
                self._ax_w[pi].set_xlim(t_win_lo, t_win_lo + _HISTORY_S)
                if self._signal_limit is None:
                    self._ax_w[pi].relim()
                    self._ax_w[pi].autoscale_view(scalex=False)

            # value annotation at cursor
            if len(t_win) > 0:
                nearest = int(np.argmin(np.abs(t_win - t_now)))
                ml_val, md_val = float(ml_win[nearest]), float(md_win[nearest])
                xlim = self._ax_w[pi].get_xlim()
                ha   = "right" if t_now > xlim[0] + 0.75 * (xlim[1] - xlim[0]) else "left"
                self._val_texts[pi].set_text(f"ML:{ml_val:.1f}\nMD:{md_val:.1f}")
                self._val_texts[pi].set_x(t_now)
                self._val_texts[pi].set_ha(ha)
            else:
                self._val_texts[pi].set_text("")

        self._fig.canvas.draw_idle()

    def _on_xrange(self, val):
        lo, hi = val
        for ax in self._ax_w:
            ax.set_xlim(lo, hi)
        self._fig.canvas.draw_idle()

    def _on_ylim(self, val):
        for ax in self._ax_w:
            ax.set_ylim(-val, val)
        self._fig.canvas.draw_idle()

    def _navigate_to(self, event):
        """Resolve event position to a frame index and update display."""
        if not self._episode_ended or event.xdata is None:
            return
        if event.inaxes is self._ax_traj:
            x_arr = np.array(self._x_buf)
            y_arr = np.array(self._y_buf)
            frame_idx = int(np.argmin(
                (x_arr - event.xdata) ** 2 + (y_arr - event.ydata) ** 2))
        elif event.inaxes in self._ax_w:
            frame_idx = int(np.argmin(
                np.abs(np.array(self._t_buf) - event.xdata)))
        else:
            return
        if self._slider is not None:
            self._slider.set_val(frame_idx)
        else:
            self._draw_frame(frame_idx)

    def _on_press(self, event):
        self._dragging = True
        self._navigate_to(event)

    def _on_release(self, _event):
        self._dragging = False

    def _on_motion(self, event):
        if self._dragging:
            self._navigate_to(event)


# ── subprocess worker ─────────────────────────────────────────────────────────

def _viz_worker(q: mp.Queue, config: dict):
    try:
        viz = VizWindow(config)
    except Exception as ex:
        print(f"[VizProcess] Failed to create VizWindow: {ex}")
        return

    import matplotlib.pyplot as plt

    while True:
        pending_frames: list = []
        try:
            while True:
                msg  = q.get_nowait()
                kind = msg.get("type")
                if kind == "frame":
                    pending_frames.append(msg["state"])
                elif kind == "start":
                    pending_frames.clear()
                    viz.start_episode(msg["state"], msg.get("path_xy"))
                elif kind == "end":
                    viz.end_episode()
                elif kind == "shutdown":
                    plt.close("all")
                    return
        except _queue_mod.Empty:
            pass

        # Ingest every queued frame so no data is lost; redraw only once.
        if pending_frames:
            for state in pending_frames[:-1]:
                viz.ingest_only(state)
            viz.update_frame(pending_frames[-1])

        try:
            plt.pause(_PAUSE_S)
        except Exception:
            break


# ── VizProcess ────────────────────────────────────────────────────────────────

class VizProcess:
    """Non-blocking viz frontend. Spawns a subprocess that owns the matplotlib window."""

    def __init__(self, config: dict):
        self._q: mp.Queue = mp.Queue(maxsize=_QUEUE_MAXSIZE)
        self._proc = mp.Process(target=_viz_worker, args=(self._q, config), daemon=True)
        self._proc.start()

    def start_episode(self, state: list[float], path_xy: list | None = None):
        try:
            self._q.put_nowait({"type": "start", "state": state, "path_xy": path_xy or []})
        except _queue_mod.Full:
            pass

    def update_frame(self, state: list[float]):
        try:
            self._q.put_nowait({"type": "frame", "state": state})
        except _queue_mod.Full:
            pass

    def end_episode(self):
        try:
            self._q.put_nowait({"type": "end"})
        except _queue_mod.Full:
            pass

    def shutdown(self):
        try:
            self._q.put_nowait({"type": "shutdown"})
        except _queue_mod.Full:
            pass
        self._proc.join(timeout=3)
        if self._proc.is_alive():
            self._proc.terminate()
