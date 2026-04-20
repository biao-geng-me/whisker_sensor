"""Render a saved hardware episode as a signal visualization video.

Layout:
- top panel: target path + actual trajectory inside the tank domain
- bottom 3x3 grid: one whisker per subplot with ML/MD traces over time

Inputs are the per-episode CSVs written by main_server_loop.py:
- traj_YYYYMMDD_HHMMSS_epXXXX.csv
- path_YYYYMMDD_HHMMSS_epXXXX.csv
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import re
import traceback

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import gridspec
from matplotlib.patches import Rectangle
import numpy as np


REPO_ROOT = Path(__file__).resolve().parent.parent
EPISODE_TRAJ_DIR = REPO_ROOT / "episode_trajectories"
SIGNALS_DIR = EPISODE_TRAJ_DIR / "signals"
WHISKER_COUNT = 9


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize one saved hardware episode.")
    parser.add_argument(
        "traj_csv",
        nargs="?",
        default="",
        help="Path to one traj_*.csv file. Defaults to the newest saved trajectory.",
    )
    parser.add_argument(
        "--path-csv",
        default="",
        help="Optional explicit path_*.csv file. Defaults to matching the traj filename.",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Output video path. Defaults next to the trajectory CSV as signals_*.mp4.",
    )
    parser.add_argument(
        "--history-seconds",
        type=float,
        default=5.0,
        help="Rolling history window shown in each whisker subplot.",
    )
    parser.add_argument(
        "--frame-stride",
        type=int,
        default=2,
        help="Use every Nth sample as one video frame.",
    )
    parser.add_argument(
        "--frame-rate",
        type=float,
        default=12.0,
        help="Output video frame rate.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Optional frame cap for quick previews. 0 means full episode.",
    )
    parser.add_argument(
        "--tail-seconds",
        type=float,
        default=1.5,
        help="Recent trajectory tail shown in the top panel.",
    )
    parser.add_argument(
        "--ensure-gif-in-signals",
        action="store_true",
        help=(
            "Use episode_trajectories/signals as a GIF cache: if the matching GIF already exists "
            "there, print its path and exit; otherwise render and save the GIF there."
        ),
    )
    parser.add_argument(
        "--ep",
        type=int,
        default=None,
        help=(
            "Render all trajectory files whose filename episode number matches this value "
            "(for example --ep 39 matches *_ep0039.csv). Matching GIFs are written into "
            "episode_trajectories/signals and existing ones are skipped."
        ),
    )
    return parser.parse_args()


def resolve_traj_csv(traj_csv: str) -> Path:
    if traj_csv:
        path = Path(traj_csv).expanduser().resolve()
        if not path.exists():
            raise FileNotFoundError(f"Trajectory CSV not found: {path}")
        return path

    files = sorted(EPISODE_TRAJ_DIR.glob("traj_*.csv"), key=lambda p: p.stat().st_mtime)
    if not files:
        raise FileNotFoundError(f"No traj_*.csv files found under {EPISODE_TRAJ_DIR}")
    return files[-1]


def extract_episode_number(path: Path) -> int | None:
    match = re.search(r"_ep(\d+)$", path.stem)
    if not match:
        return None
    return int(match.group(1))


def iter_traj_csvs(ep: int | None = None) -> list[Path]:
    files = sorted(EPISODE_TRAJ_DIR.glob("traj_*.csv"))
    if ep is not None:
        files = [p for p in files if extract_episode_number(p) == ep]
    if not files:
        if ep is None:
            raise FileNotFoundError(f"No traj_*.csv files found under {EPISODE_TRAJ_DIR}")
        raise FileNotFoundError(
            f"No traj_*.csv files with episode number {ep} found under {EPISODE_TRAJ_DIR}"
        )
    return files


def resolve_path_csv(traj_csv: Path, path_csv: str) -> Path:
    if path_csv:
        path = Path(path_csv).expanduser().resolve()
        if not path.exists():
            raise FileNotFoundError(f"Path CSV not found: {path}")
        return path

    candidate = traj_csv.with_name(traj_csv.name.replace("traj_", "path_", 1))
    if not candidate.exists():
        raise FileNotFoundError(f"Matching target-path CSV not found: {candidate}")
    return candidate


def default_output_path(traj_csv: Path) -> Path:
    return traj_csv.with_name(traj_csv.name.replace("traj_", "signals_", 1)).with_suffix(".mp4")


def default_signals_gif_path(traj_csv: Path) -> Path:
    return SIGNALS_DIR / traj_csv.name.replace("traj_", "signals_", 1).replace(".csv", ".gif")


def load_traj_table(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"Trajectory CSV is empty: {path}")

    required = ["t", "x", "y"] + [f"{prefix}{idx}" for idx in range(1, WHISKER_COUNT + 1) for prefix in ("ML", "MD")]
    missing = [name for name in required if name not in rows[0]]
    if missing:
        raise ValueError(f"Missing required trajectory columns in {path}: {missing}")

    data: dict[str, np.ndarray] = {}
    for name in required:
        data[name] = np.array([float(r[name]) for r in rows], dtype=np.float64)
    return data


def load_path_table(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"Path CSV is empty: {path}")
    for name in ("x_mm", "y_mm"):
        if name not in rows[0]:
            raise ValueError(f"Missing required path columns in {path}: x_mm, y_mm")
    x = np.array([float(r["x_mm"]) for r in rows], dtype=np.float64)
    y = np.array([float(r["y_mm"]) for r in rows], dtype=np.float64)
    return x, y


def choose_signal_limit(traj: dict[str, np.ndarray]) -> float:
    all_values = np.concatenate([traj[f"{prefix}{idx}"] for idx in range(1, WHISKER_COUNT + 1) for prefix in ("ML", "MD")])
    magnitudes = np.sort(np.abs(all_values))
    if magnitudes.size == 0:
        return 1.0
    idx = max(0, int(round(0.995 * (magnitudes.size - 1))))
    return max(float(magnitudes[idx]), 1.0)


def pick_writer(output_path: Path, frame_rate: float):
    suffix = output_path.suffix.lower()
    if suffix not in {".mp4", ".gif"}:
        output_path = output_path.with_suffix(".mp4")

    if output_path.suffix.lower() == ".gif":
        return animation.PillowWriter(fps=frame_rate), output_path

    if animation.writers.is_available("ffmpeg"):
        writer = animation.FFMpegWriter(fps=frame_rate, codec="libx264", bitrate=1800)
        return writer, output_path

    if animation.writers.is_available("pillow"):
        fallback = output_path.with_suffix(".gif")
        return animation.PillowWriter(fps=frame_rate), fallback

    raise RuntimeError(
        "Neither ffmpeg nor pillow animation writers are available. "
        "Install ffmpeg for MP4 output or pillow for GIF output."
    )


def render_episode(args: argparse.Namespace, traj_csv: Path, path_csv: Path, output_path: Path) -> None:
    traj = load_traj_table(traj_csv)
    path_x, path_y = load_path_table(path_csv)

    t_sec = traj["t"] / 1000.0
    x_mm = traj["x"]
    y_mm = traj["y"]
    frame_count = len(t_sec)
    if args.max_frames and args.max_frames > 0:
        frame_count = min(frame_count, int(args.max_frames))

    frame_indices = list(range(0, frame_count, max(1, args.frame_stride)))
    if frame_indices[-1] != frame_count - 1:
        frame_indices.append(frame_count - 1)

    dt = float(np.median(np.diff(t_sec))) if len(t_sec) > 1 else 0.05
    tail_samples = max(5, int(round(args.tail_seconds / max(dt, 1e-6))))
    signal_limit = choose_signal_limit(traj)

    writer, output_path = pick_writer(output_path, args.frame_rate)

    fig = plt.figure(figsize=(15, 10), dpi=140)
    gs = gridspec.GridSpec(4, 3, height_ratios=[1.55, 1.0, 1.0, 1.0], hspace=0.35, wspace=0.22)

    ax_top = fig.add_subplot(gs[0, :])
    ax_top.set_title(f"Episode Signals: {traj_csv.stem}")
    ax_top.set_xlabel("X (mm)")
    ax_top.set_ylabel("Y (mm)")
    ax_top.set_xlim([-500, 4500])
    ax_top.set_ylim([-300, 1200])
    ax_top.set_aspect("equal", adjustable="box")
    ax_top.invert_xaxis()
    ax_top.invert_yaxis()
    ax_top.xaxis.tick_top()
    ax_top.xaxis.set_label_position("top")
    ax_top.yaxis.tick_right()
    ax_top.yaxis.set_label_position("right")
    ax_top.add_patch(Rectangle((0, 0), 3800, 850, fill=False, edgecolor="red", linewidth=2))
    ax_top.plot(path_x, path_y, color="0.55", linestyle="--", linewidth=2.0, label="Target path")
    traj_line, = ax_top.plot([], [], color="tab:blue", linewidth=2.0, label="Actual trajectory")
    tail_line, = ax_top.plot([], [], color="#4DBEEE", linewidth=3.0, label="Recent tail")
    current_marker, = ax_top.plot([], [], "o", markersize=8, markeredgecolor="k",
                                  markerfacecolor="tab:orange", label="Current position")
    info_text = ax_top.text(
        0.01, 0.02, "", transform=ax_top.transAxes, ha="left", va="bottom",
        fontweight="bold", bbox={"facecolor": "white", "edgecolor": "none", "pad": 3.0}
    )
    ax_top.legend(loc="lower center", ncol=4)

    axes = []
    ml_lines = []
    md_lines = []
    cursors = []
    for whisker_idx in range(1, WHISKER_COUNT + 1):
        ax = fig.add_subplot(gs[1 + (whisker_idx - 1) // 3, (whisker_idx - 1) % 3])
        ax.set_title(f"Whisker {whisker_idx}")
        ax.grid(True, alpha=0.2)
        ax.set_ylim([-signal_limit, signal_limit])
        if whisker_idx > 6:
            ax.set_xlabel("Time (s)")
        if whisker_idx in {1, 4, 7}:
            ax.set_ylabel("Moment")
        ml_line, = ax.plot([], [], color="tab:blue", linewidth=1.4, label="ML")
        md_line, = ax.plot([], [], color="tab:orange", linewidth=1.4, label="MD")
        cursor = ax.axvline(0.0, color="k", linestyle=":", linewidth=1.0)
        if whisker_idx == 1:
            ax.legend(loc="upper left")
        axes.append(ax)
        ml_lines.append(ml_line)
        md_lines.append(md_line)
        cursors.append(cursor)

    def update(frame_idx: int):
        t_now = t_sec[frame_idx]
        hist_mask = (t_sec >= max(0.0, t_now - args.history_seconds)) & (t_sec <= t_now)

        traj_line.set_data(x_mm[: frame_idx + 1], y_mm[: frame_idx + 1])
        tail_start = max(0, frame_idx - tail_samples + 1)
        tail_line.set_data(x_mm[tail_start: frame_idx + 1], y_mm[tail_start: frame_idx + 1])
        current_marker.set_data([x_mm[frame_idx]], [y_mm[frame_idx]])
        info_text.set_text(f"t = {t_now:.2f} s   x = {x_mm[frame_idx]:.1f} mm   y = {y_mm[frame_idx]:.1f} mm")

        for whisker_idx in range(1, WHISKER_COUNT + 1):
            ml_name = f"ML{whisker_idx}"
            md_name = f"MD{whisker_idx}"
            ml_lines[whisker_idx - 1].set_data(t_sec[hist_mask], traj[ml_name][hist_mask])
            md_lines[whisker_idx - 1].set_data(t_sec[hist_mask], traj[md_name][hist_mask])
            cursors[whisker_idx - 1].set_xdata([t_now, t_now])
            if t_now <= args.history_seconds:
                axes[whisker_idx - 1].set_xlim(0.0, args.history_seconds)
            else:
                axes[whisker_idx - 1].set_xlim(t_now - args.history_seconds, t_now)

        return [traj_line, tail_line, current_marker, info_text, *ml_lines, *md_lines, *cursors]

    ani = animation.FuncAnimation(fig, update, frames=frame_indices, blit=False, interval=1000.0 / args.frame_rate)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    ani.save(str(output_path), writer=writer)
    plt.close(fig)

    print(f"Trajectory CSV: {traj_csv}")
    print(f"Path CSV: {path_csv}")
    print(f"Wrote: {output_path}")


def ensure_gifs_in_signals(args: argparse.Namespace) -> None:
    if args.path_csv or args.output:
        raise ValueError("--path-csv and --output are only supported when rendering one specific trajectory.")

    rendered = 0
    skipped = 0
    failed = 0

    for traj_csv in iter_traj_csvs(args.ep):
        output_path = default_signals_gif_path(traj_csv)
        if output_path.exists():
            print(f"Existing GIF: {output_path}")
            skipped += 1
            continue

        try:
            path_csv = resolve_path_csv(traj_csv, "")
            render_episode(args, traj_csv, path_csv, output_path)
            rendered += 1
        except Exception as ex:
            print(f"Failed: {traj_csv} -> {type(ex).__name__}: {ex}")
            traceback.print_exc()
            failed += 1

    print(f"Signals cache summary: rendered={rendered} skipped={skipped} failed={failed}")


def main() -> None:
    args = parse_args()
    if args.ep is not None:
        if args.traj_csv:
            raise ValueError("--ep cannot be combined with a positional traj_csv argument.")
        ensure_gifs_in_signals(args)
        return

    if args.ensure_gif_in_signals and not args.traj_csv:
        ensure_gifs_in_signals(args)
        return

    traj_csv = resolve_traj_csv(args.traj_csv)
    path_csv = resolve_path_csv(traj_csv, args.path_csv)
    if args.ensure_gif_in_signals:
        output_path = default_signals_gif_path(traj_csv)
        if output_path.exists():
            print(f"Trajectory CSV: {traj_csv}")
            print(f"Path CSV: {path_csv}")
            print(f"Existing GIF: {output_path}")
            return
    else:
        output_path = Path(args.output).expanduser().resolve() if args.output else default_output_path(traj_csv)

    render_episode(args, traj_csv, path_csv, output_path)


if __name__ == "__main__":
    main()
