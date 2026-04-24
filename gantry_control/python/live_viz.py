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

_TAIL_FRAMES   = 50     # recent-trajectory tail length
_HISTORY_S     = 5.0    # rolling signal window (seconds)
_QUEUE_MAXSIZE = 500    # drop frames silently when viz falls behind
_PAUSE_S       = 0.02   # plt.pause interval in subprocess (~50 Hz GUI)

_WHISKER_ROWS  = 3
_WHISKER_COLS  = 3
_WHISKER_COUNT = _WHISKER_ROWS * _WHISKER_COLS   # 9

# Panel index → 1-indexed whisker number.
# Panels fill row-major (left to right, top to bottom):
#   row 1: panels 0,1,2  → whiskers 3,2,1
#   row 2: panels 3,4,5  → whiskers 6,5,4
#   row 3: panels 6,7,8  → whiskers 9,8,7
_PANEL_TO_WHISKER = [3, 2, 1, 6, 5, 4, 9, 8, 7]


# ── helpers ──────────────────────────────────────────────────────────────────

def _decode_newest_frame(state: list[float], n_rl_interval: int, n_ch_total: int) -> np.ndarray:
    """Return the newest (last) row of the state matrix as shape (n_ch_total,)."""
    mat = np.array(state, dtype=np.float64).reshape(n_rl_interval, n_ch_total)
    return mat[-1]


def _choose_signal_limit(frames: list[np.ndarray], n_ch_total: int) -> float:
    """99.5th-percentile absolute value of all whisker channels across frames."""
    arr = np.stack(frames)          # (N, n_ch_total)
    whisker_data = arr[:, 5:]       # channels 5+ are ML/MD pairs
    mags = np.sort(np.abs(whisker_data.ravel()))
    if mags.size == 0:
        return 1.0
    idx = max(0, int(round(0.995 * (mags.size - 1))))
    return max(float(mags[idx]), 1e-6)


# ── VizWindow ────────────────────────────────────────────────────────────────

class VizWindow:
    """Interactive matplotlib figure. Must be created inside the viz subprocess."""

    def __init__(self, config: dict):
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        self._plt = plt

        self._n_rl  = config.get("n_rl_interval", 4)
        self._n_ch  = config.get("n_ch_total", 23)
        self._path_xy_default = (config.get("path_data") or [[]])[0]

        # episode buffers
        self._x_buf:   list[float]      = []
        self._y_buf:   list[float]      = []
        self._t_buf:   list[float]      = []   # elapsed time in seconds per frame
        self._frames:  list[np.ndarray] = []   # newest frame per received state
        self._t0_ms:   float | None     = None  # absolute controller time at episode start
        self._signal_limit: float | None = None
        self._episode_ended: bool        = False
        self._val_texts: list            = []
        self._slider     = None
        self._slider_ax  = None

        self._build_figure()

    # ── figure setup ─────────────────────────────────────────────────────────

    def _build_figure(self):
        plt = self._plt
        from matplotlib.gridspec import GridSpec
        from matplotlib.patches import Rectangle, Ellipse
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
        self._traj_line, = ax_t.plot([], [], color="tab:blue",
                                      linewidth=1.5, label="Trajectory")
        self._tail_line, = ax_t.plot([], [], color="#4DBEEE",
                                      linewidth=2.5, label="Recent tail")
        self._pos_dot, = ax_t.plot([], [], "o", markersize=7,
                                    markerfacecolor="tab:orange",
                                    markeredgecolor="k", label="Current")
        self._info_text = ax_t.text(
            0.01, 0.03, "", transform=ax_t.transAxes,
            ha="left", va="bottom", fontsize=8, fontweight="bold",
            bbox={"facecolor": "white", "edgecolor": "none", "pad": 2})
        ax_t.legend(loc="lower center", ncol=4, fontsize=7)
        self._ax_traj = ax_t

        # ── whisker layout panel ─────────────────────────────────────────────
        ax_l = self._fig.add_subplot(gs[0, 2])
        ax_l.set_title("Whisker Layout", fontsize=9)
        ax_l.set_xlim(-0.5, 2.5)
        ax_l.set_ylim(-0.5, 2.5)
        ax_l.set_aspect("equal")
        ax_l.axis("off")

        self._layout_cmap  = cm.viridis
        self._layout_norm  = mcolors.Normalize(vmin=0.0, vmax=1.0)
        self._layout_patches: list = []

        ew, eh = 0.80, 0.80 / 3.0   # ellipse width/height — 3:1 aspect in data space
        for panel_idx in range(_WHISKER_COUNT):
            col_pos = panel_idx % _WHISKER_COLS
            row_pos = 2 - panel_idx // _WHISKER_COLS   # row 2 = top, 0 = bottom
            whisker_num = _PANEL_TO_WHISKER[panel_idx]
            e = Ellipse((col_pos, row_pos), width=ew, height=eh,
                        facecolor=self._layout_cmap(0.0),
                        edgecolor="k", linewidth=1.2, zorder=3)
            ax_l.add_patch(e)
            ax_l.text(col_pos, row_pos, str(whisker_num),
                      ha="center", va="center",
                      fontsize=9, fontweight="bold", color="white", zorder=4)
            self._layout_patches.append(e)
        self._ax_layout = ax_l

        # ── signal panels (9 whiskers, ordered by _PANEL_TO_WHISKER) ─────────
        self._ax_w     = []
        self._ml_lines = []
        self._md_lines = []
        self._cur_lines = []

        for panel_idx in range(_WHISKER_COUNT):
            grid_row = 1 + panel_idx // _WHISKER_COLS
            grid_col = panel_idx % _WHISKER_COLS
            whisker_num = _PANEL_TO_WHISKER[panel_idx]

            ax = self._fig.add_subplot(gs[grid_row, grid_col])
            ax.grid(True, alpha=0.2)
            # no fixed ylim — autoscale until signal_limit is known
            ax.tick_params(labelsize=6)

            # x-axis label only on bottom row
            if grid_row == 3:
                ax.set_xlabel("Time (s)", fontsize=7)
            else:
                ax.tick_params(labelbottom=False)

            # y-axis label on left column only
            if grid_col == 0:
                ax.set_ylabel("Moment", fontsize=7)

            # whisker label inside top-left of panel instead of title
            ax.text(0.02, 0.93, f"W{whisker_num}",
                    transform=ax.transAxes, fontsize=8, fontweight="bold",
                    va="top", ha="left",
                    bbox={"facecolor": "white", "edgecolor": "none",
                          "alpha": 0.7, "pad": 1})

            ml, = ax.plot([], [], color="tab:blue",   linewidth=1.2, label="ML")
            md, = ax.plot([], [], color="tab:orange",  linewidth=1.2, label="MD")
            cur  = ax.axvline(0.0, color="k", linestyle=":", linewidth=0.8)

            if panel_idx == 0:
                ax.legend(loc="upper right", fontsize=6)

            from matplotlib.transforms import blended_transform_factory
            trans = blended_transform_factory(ax.transData, ax.transAxes)
            vt = ax.text(0.0, 0.96, "", transform=trans, fontsize=6,
                         va="top", ha="left", zorder=5,
                         bbox={"facecolor": "white", "edgecolor": "none",
                               "alpha": 0.75, "pad": 1})

            self._ax_w.append(ax)
            self._ml_lines.append(ml)
            self._md_lines.append(md)
            self._cur_lines.append(cur)
            self._val_texts.append(vt)

        self._fig.subplots_adjust(bottom=0.05)
        plt.show(block=False)
        plt.pause(0.05)

    # ── public API ────────────────────────────────────────────────────────────

    def start_episode(self, state: list[float], path_xy: list | None = None):
        """Reset buffers and draw static path for a new episode."""
        self._x_buf.clear()
        self._y_buf.clear()
        self._t_buf.clear()
        self._frames.clear()
        self._t0_ms        = None
        self._signal_limit = None
        self._episode_ended = False
        for vt in self._val_texts:
            vt.set_text("")

        # re-enable autoscale on signal panels (cleared fixed ylim from last episode)
        for ax in self._ax_w:
            ax.set_autoscaley_on(True)
            ax.relim()

        # remove slider from previous episode
        if self._slider_ax is not None:
            try:
                self._slider_ax.remove()
            except Exception:
                pass
            self._slider    = None
            self._slider_ax = None
            self._fig.subplots_adjust(bottom=0.05)

        # clear live lines
        for line in (self._traj_line, self._tail_line):
            line.set_data([], [])
        self._pos_dot.set_data([], [])
        self._info_text.set_text("")
        for i in range(_WHISKER_COUNT):
            self._ml_lines[i].set_data([], [])
            self._md_lines[i].set_data([], [])

        # draw target path
        raw_path = path_xy if path_xy else self._path_xy_default
        if raw_path:
            self._path_line.set_data([p[0] for p in raw_path],
                                      [p[1] for p in raw_path])
        else:
            self._path_line.set_data([], [])

        self._ingest_frame(state)
        self._draw_frame(0)

    def update_frame(self, state: list[float]):
        """Append a new state frame and refresh live plots."""
        self._ingest_frame(state)
        self._draw_frame(len(self._frames) - 1)

    def end_episode(self):
        """Fix y-limits and attach a review slider."""
        if not self._frames:
            return

        # fix signal y-limits to episode range
        if self._signal_limit is None and self._frames:
            self._signal_limit = _choose_signal_limit(self._frames, self._n_ch)
        if self._signal_limit:
            for ax in self._ax_w:
                ax.set_autoscaley_on(False)
                ax.set_ylim(-self._signal_limit, self._signal_limit)

        self._episode_ended = True
        t_total = self._t_buf[-1] if self._t_buf else 1.0
        for ax in self._ax_w:
            ax.set_xlim(0.0, t_total)

        self._fig.subplots_adjust(bottom=0.07)
        self._slider_ax = self._fig.add_axes([0.1, 0.01, 0.8, 0.02])

        from matplotlib.widgets import Slider
        n = max(1, len(self._frames) - 1)
        self._slider = Slider(self._slider_ax, "Frame", 0, n,
                              valinit=n, valstep=1)
        self._slider.on_changed(lambda v: self._draw_frame(int(round(v))))
        self._plt.draw()

    # ── internal helpers ──────────────────────────────────────────────────────

    def _ingest_frame(self, state: list[float]):
        frame = _decode_newest_frame(state, self._n_rl, self._n_ch)
        t_ms = float(frame[0])
        if self._t0_ms is None:
            self._t0_ms = t_ms
        self._t_buf.append((t_ms - self._t0_ms) / 1000.0)
        self._x_buf.append(float(frame[1]))
        self._y_buf.append(float(frame[2]))
        self._frames.append(frame)

    def _draw_frame(self, frame_idx: int):
        if not self._frames:
            return
        frame_idx = max(0, min(frame_idx, len(self._frames) - 1))

        x_arr = np.array(self._x_buf[:frame_idx + 1])
        y_arr = np.array(self._y_buf[:frame_idx + 1])
        t_now = self._t_buf[frame_idx] if self._t_buf else 0.0

        # ── trajectory ────────────────────────────────────────────────────────
        self._traj_line.set_data(x_arr, y_arr)
        tail_start = max(0, frame_idx + 1 - _TAIL_FRAMES)
        self._tail_line.set_data(x_arr[tail_start:], y_arr[tail_start:])
        self._pos_dot.set_data([x_arr[-1]], [y_arr[-1]])
        self._info_text.set_text(
            f"t = {t_now:.1f} s   x = {x_arr[-1]:.0f} mm   y = {y_arr[-1]:.0f} mm"
        )

        # ── whisker layout ellipse colours ────────────────────────────────────
        frame = self._frames[frame_idx]
        lim   = self._signal_limit or 1.0
        for panel_idx in range(_WHISKER_COUNT):
            w    = _PANEL_TO_WHISKER[panel_idx]        # 1-indexed whisker
            ml_v = frame[5 + (w - 1) * 2]
            md_v = frame[5 + (w - 1) * 2 + 1]
            norm_mag = float(np.clip(max(abs(ml_v), abs(md_v)) / lim, 0.0, 1.0))
            self._layout_patches[panel_idx].set_facecolor(
                self._layout_cmap(self._layout_norm(norm_mag))
            )

        # ── signal traces ─────────────────────────────────────────────────────
        # in review mode show full episode; live mode uses rolling window
        all_frames = np.array(self._frames[:frame_idx + 1])
        t_arr      = np.array(self._t_buf[:frame_idx + 1])

        if self._episode_ended:
            t_full   = np.array(self._t_buf)
            all_data = np.array(self._frames)
            mask     = np.ones(len(t_full), dtype=bool)
            t_win    = t_full
            data_arr = all_data
        else:
            t_win_lo = max(0.0, t_now - _HISTORY_S)
            mask     = t_arr >= t_win_lo
            t_win    = t_arr[mask]
            data_arr = all_frames

        for panel_idx in range(_WHISKER_COUNT):
            w     = _PANEL_TO_WHISKER[panel_idx]
            ml_ch = 5 + (w - 1) * 2
            md_ch = 5 + (w - 1) * 2 + 1
            ml_win = data_arr[mask, ml_ch]
            md_win = data_arr[mask, md_ch]
            self._ml_lines[panel_idx].set_data(t_win, ml_win)
            self._md_lines[panel_idx].set_data(t_win, md_win)
            self._cur_lines[panel_idx].set_xdata([t_now, t_now])

            if not self._episode_ended:
                t_win_lo = max(0.0, t_now - _HISTORY_S)
                self._ax_w[panel_idx].set_xlim(t_win_lo, t_win_lo + _HISTORY_S)
                if self._signal_limit is None:
                    self._ax_w[panel_idx].relim()
                    self._ax_w[panel_idx].autoscale_view(scalex=False)

            # annotate ML/MD values at cursor
            if len(t_win) > 0:
                nearest = int(np.argmin(np.abs(t_win - t_now)))
                ml_val  = float(ml_win[nearest])
                md_val  = float(md_win[nearest])
                xlim    = self._ax_w[panel_idx].get_xlim()
                ha      = "right" if t_now > xlim[0] + 0.75 * (xlim[1] - xlim[0]) else "left"
                self._val_texts[panel_idx].set_text(f"ML:{ml_val:.3f}\nMD:{md_val:.3f}")
                self._val_texts[panel_idx].set_x(t_now)
                self._val_texts[panel_idx].set_ha(ha)
            else:
                self._val_texts[panel_idx].set_text("")

        self._fig.canvas.draw_idle()


# ── subprocess worker ─────────────────────────────────────────────────────────

def _viz_worker(q: mp.Queue, config: dict):
    """Entry point for the visualization subprocess."""
    try:
        viz = VizWindow(config)
    except Exception as ex:
        print(f"[VizProcess] Failed to create VizWindow: {ex}")
        return

    import matplotlib.pyplot as plt

    while True:
        # Drain queue, but keep only the latest "frame" message to avoid lag.
        last_frame: dict | None = None
        try:
            while True:
                msg = q.get_nowait()
                kind = msg.get("type")
                if kind == "frame":
                    last_frame = msg          # supersedes older frames
                elif kind == "start":
                    last_frame = None         # episode reset clears pending frame
                    viz.start_episode(msg["state"], msg.get("path_xy"))
                elif kind == "end":
                    viz.end_episode()
                elif kind == "shutdown":
                    plt.close("all")
                    return
        except _queue_mod.Empty:
            pass

        if last_frame is not None:
            viz.update_frame(last_frame["state"])

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
