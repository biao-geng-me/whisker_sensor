"""Plot helpers for episode-level monitoring in hardware training."""

from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from .config import SACV2PathblindConfig


def save_rollout_plots(rows: list[dict], path_xy: np.ndarray, outdir: Path, cfg: SACV2PathblindConfig) -> None:
    if not rows:
        return

    step = [r['step'] for r in rows]
    x = [r['x_mm'] for r in rows]
    y = [r['y_mm'] for r in rows]
    reward = [r['reward'] for r in rows]
    lateral = [r.get('signed_lateral_error_mm', np.nan) for r in rows]
    action = [r['action'] for r in rows]

    plt.figure(figsize=(12, 10))
    ax1 = plt.subplot(2, 1, 1)
    ax1.plot(path_xy[:, 0], path_xy[:, 1], color='0.75', lw=2, label='target path')
    ax1.plot(x, y, color='tab:blue', lw=2, label='array trajectory')
    ax1.scatter([x[0]], [y[0]], color='tab:green', s=70, label='start', zorder=3)
    ax1.scatter([x[-1]], [y[-1]], color='tab:red', s=70, label='end', zorder=3)
    ax1.axvline(cfg.finish_line_mm, color='tab:purple', ls='--', alpha=0.6, label='finish line')
    ax1.set_title('Episode Trajectory')
    ax1.set_xlabel('x (mm)')
    ax1.set_ylabel('y (mm)')
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    ax2 = plt.subplot(2, 1, 2)
    ax2.plot(step, lateral, label='signed lateral error (mm)', color='tab:orange')
    ax2.axhline(cfg.reward_corridor_half_width_mm, color='tab:orange', ls='--', alpha=0.5)
    ax2.axhline(-cfg.reward_corridor_half_width_mm, color='tab:orange', ls='--', alpha=0.5)
    ax2.axhline(cfg.terminate_corridor_half_width_mm, color='tab:red', ls='--', alpha=0.5)
    ax2.axhline(-cfg.terminate_corridor_half_width_mm, color='tab:red', ls='--', alpha=0.5)
    ax2.set_title('Lateral Tracking Error')
    ax2.set_xlabel('RL step')
    ax2.set_ylabel('Signed lateral error (mm)')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(outdir / 'trajectory.png', dpi=180)
    plt.close()

    plt.figure(figsize=(12, 8))
    ax1 = plt.subplot(2, 1, 1)
    ax1.plot(step, reward, color='tab:green')
    ax1.set_title('Reward per Step')
    ax1.set_xlabel('RL step')
    ax1.set_ylabel('Reward')
    ax1.grid(True, alpha=0.3)

    ax2 = plt.subplot(2, 1, 2)
    ax2.plot(step, action, color='tab:purple')
    ax2.axhline(1.0, color='0.7', ls='--')
    ax2.axhline(-1.0, color='0.7', ls='--')
    ax2.set_title('Policy Action')
    ax2.set_xlabel('RL step')
    ax2.set_ylabel('Action')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(outdir / 'metrics.png', dpi=180)
    plt.close()
