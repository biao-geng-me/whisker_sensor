"""Generate reusable analysis plots for one hardware training run."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import statistics as stats

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


COLORS = {
    'y_boundary': 'tab:brown',
    'too_far': 'tab:red',
    'too_close': 'tab:orange',
    'time_limit': 'tab:blue',
    'runtime_done': 'tab:green',
    'runtime_truncated': 'tab:cyan',
    'command_failed': 'tab:brown',
    'sensor_timeout': 'tab:purple',
    'state_timeout': 'tab:pink',
    'hardware_error': 'tab:gray',
    'done': 'tab:olive',
}


def parse_args():
    parser = argparse.ArgumentParser(description='Analyze one rl_sac_v4_pathblind_hardware run directory.')
    parser.add_argument('run_dir', type=str, help='Path to one rl_sac_v4_pathblind_hardware run directory.')
    return parser.parse_args()


def load_csv_rows(path: Path) -> list[dict]:
    with path.open() as handle:
        return list(csv.DictReader(handle))


def ensure_positive(xs: list[int], ys: list[float]) -> tuple[list[int], list[float]]:
    filtered = [(x, y) for x, y in zip(xs, ys) if y > 0.0]
    if not filtered:
        return [], []
    x_out, y_out = zip(*filtered)
    return list(x_out), list(y_out)


def make_training_curves(train_rows: list[dict], outdir: Path):
    steps = [int(r['total_env_steps']) for r in train_rows]
    mean_reward = [float(r['mean_reward_this_episode']) for r in train_rows]
    mean_lat = [float(r['mean_lateral_error_mm']) for r in train_rows]
    actor_steps = [int(r['total_env_steps']) for r in train_rows if r['actor_loss'] not in ('', None)]
    alpha = [float(r['alpha']) for r in train_rows if r['alpha'] not in ('', None)]

    plt.figure(figsize=(12, 9))

    ax1 = plt.subplot(2, 2, 1)
    ax1.plot(steps, mean_reward, lw=2)
    ax1.set_title('Mean Reward vs Env Steps')
    ax1.set_xlabel('Env steps')
    ax1.set_ylabel('Mean reward')
    ax1.grid(True, alpha=0.3)

    ax2 = plt.subplot(2, 2, 2)
    ax2.plot(steps, mean_lat, lw=2, color='tab:orange')
    ax2.axhline(180, ls='--', color='tab:orange', alpha=0.5)
    ax2.axhline(-180, ls='--', color='tab:orange', alpha=0.5)
    ax2.axhline(240, ls='--', color='tab:red', alpha=0.5)
    ax2.axhline(-240, ls='--', color='tab:red', alpha=0.5)
    ax2.set_title('Mean Signed Lateral Error vs Env Steps')
    ax2.set_xlabel('Env steps')
    ax2.set_ylabel('Signed lateral error (mm)')
    ax2.grid(True, alpha=0.3)

    ax3 = plt.subplot(2, 2, 3)
    ax3.plot(steps, [float(r['episode_return']) for r in train_rows], lw=2, color='tab:green')
    ax3.set_title('Episode Return vs Env Steps')
    ax3.set_xlabel('Env steps')
    ax3.set_ylabel('Episode return')
    ax3.grid(True, alpha=0.3)

    ax4 = plt.subplot(2, 2, 4)
    ax4.plot(actor_steps, alpha, color='tab:purple')
    ax4.set_title('Entropy Temperature (alpha)')
    ax4.set_xlabel('Env steps')
    ax4.set_ylabel('alpha')
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(outdir / 'training_curves.png', dpi=180)
    plt.close()


def make_loss_curves_log(train_rows: list[dict], outdir: Path):
    def _pick_metric(row: dict, primary: str, fallback: str):
        value = row.get(primary, '')
        if value in ('', None):
            value = row.get(fallback, '')
        return value

    valid_rows = [
        r for r in train_rows
        if _pick_metric(r, 'actor_loss', 'actor_loss_last') not in ('', None)
    ]
    actor_steps = [int(r['total_env_steps']) for r in valid_rows]
    actor_loss = [float(_pick_metric(r, 'actor_loss', 'actor_loss_last')) for r in valid_rows]
    q1_loss = [float(_pick_metric(r, 'q1_loss', 'q1_loss_last')) for r in valid_rows]
    q2_loss = [float(_pick_metric(r, 'q2_loss', 'q2_loss_last')) for r in valid_rows]

    q1_x, q1_y = ensure_positive(actor_steps, q1_loss)
    q2_x, q2_y = ensure_positive(actor_steps, q2_loss)

    plt.figure(figsize=(10, 8))

    ax1 = plt.subplot(2, 1, 1)
    if q1_x:
        ax1.semilogy(q1_x, q1_y, label='q1_loss_mean')
    if q2_x:
        ax1.semilogy(q2_x, q2_y, label='q2_loss_mean')
    ax1.set_title('Critic Loss Curves (episode-mean, log scale)')
    ax1.set_xlabel('Env steps')
    ax1.set_ylabel('Critic loss')
    ax1.grid(True, which='both', alpha=0.3)
    ax1.legend()

    ax2 = plt.subplot(2, 1, 2)
    if actor_steps:
        ax2.plot(actor_steps, actor_loss, label='actor_loss_mean')
    ax2.set_yscale('symlog', linthresh=1e-2)
    ax2.axhline(0.0, color='0.5', ls='--', alpha=0.6)
    ax2.set_title('Actor Loss Curve (episode-mean, symlog scale)')
    ax2.set_xlabel('Env steps')
    ax2.set_ylabel('Actor loss')
    ax2.grid(True, which='both', alpha=0.3)
    ax2.legend()

    plt.tight_layout()
    plt.savefig(outdir / 'loss_curves_log.png', dpi=180)
    plt.close()


def make_episode_curves(episode_rows: list[dict], outdir: Path):
    episode_idx = list(range(1, len(episode_rows) + 1))
    episode_returns = [float(r['episode_return']) for r in episode_rows]
    episode_lats = [float(r['signed_lateral_error_mm']) for r in episode_rows]
    reasons = [r['termination_reason'] for r in episode_rows]

    plt.figure(figsize=(12, 9))

    ax1 = plt.subplot(2, 1, 1)
    for idx, ret, reason in zip(episode_idx, episode_returns, reasons):
        ax1.scatter(idx, ret, color=COLORS.get(reason, 'tab:gray'), s=40)
    ax1.plot(episode_idx, episode_returns, color='0.6', alpha=0.6)
    ax1.set_title('Episode Return by Episode')
    ax1.set_xlabel('Episode index')
    ax1.set_ylabel('Episode return')
    ax1.grid(True, alpha=0.3)

    ax2 = plt.subplot(2, 1, 2)
    for idx, lat, reason in zip(episode_idx, episode_lats, reasons):
        ax2.scatter(idx, lat, color=COLORS.get(reason, 'tab:gray'), s=40)
    ax2.axhline(180, ls='--', color='tab:orange', alpha=0.5)
    ax2.axhline(-180, ls='--', color='tab:orange', alpha=0.5)
    ax2.axhline(240, ls='--', color='tab:red', alpha=0.5)
    ax2.axhline(-240, ls='--', color='tab:red', alpha=0.5)
    ax2.plot(episode_idx, episode_lats, color='0.6', alpha=0.6)
    ax2.set_title('Episode-End Signed Lateral Error')
    ax2.set_xlabel('Episode index')
    ax2.set_ylabel('Signed lateral error (mm)')
    ax2.grid(True, alpha=0.3)

    present = [name for name in COLORS.keys() if name in set(reasons)]
    if present:
        handles = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor=COLORS[name], markersize=8, label=name)
            for name in present
        ]
        ax1.legend(handles=handles, title='termination reason', loc='best')

    plt.tight_layout()
    plt.savefig(outdir / 'episode_curves.png', dpi=180)
    plt.close()


def write_summary(train_rows: list[dict], episode_rows: list[dict], outdir: Path):
    steps = [int(r['total_env_steps']) for r in train_rows]
    mean_reward = [float(r['mean_reward_this_episode']) for r in train_rows]
    mean_lat = [float(r['mean_lateral_error_mm']) for r in train_rows]
    actor_loss = [float(r['actor_loss']) for r in train_rows if r['actor_loss'] not in ('', None)]
    alpha = [float(r['alpha']) for r in train_rows if r['alpha'] not in ('', None)]
    episode_returns = [float(r['episode_return']) for r in episode_rows]
    reasons = [r['termination_reason'] for r in episode_rows]

    summary = []
    summary.append(f'train_rows={len(train_rows)}')
    summary.append(f'episode_rows={len(episode_rows)}')
    summary.append(f'first_step={steps[0]}')
    summary.append(f'last_step={steps[-1]}')
    summary.append(f'mean_mean_reward={stats.fmean(mean_reward):.4f}')
    summary.append(f'first_mean_reward={mean_reward[0]:.4f}')
    summary.append(f'last_mean_reward={mean_reward[-1]:.4f}')
    summary.append(f'mean_signed_lateral={stats.fmean(mean_lat):.2f}')
    summary.append(f'first_signed_lateral={mean_lat[0]:.2f}')
    summary.append(f'last_signed_lateral={mean_lat[-1]:.2f}')
    if actor_loss:
        summary.append(f'first_actor_loss={actor_loss[0]:.6f}')
        summary.append(f'last_actor_loss={actor_loss[-1]:.6f}')
    if alpha:
        summary.append(f'first_alpha={alpha[0]:.6f}')
        summary.append(f'last_alpha={alpha[-1]:.6f}')
    summary.append(f'episode_return_mean={stats.fmean(episode_returns):.4f}')
    summary.append(f'episode_return_min={min(episode_returns):.4f}')
    summary.append(f'episode_return_max={max(episode_returns):.4f}')
    summary.append('termination_counts=' + str({k: reasons.count(k) for k in sorted(set(reasons))}))
    (outdir / 'summary.txt').write_text('\n'.join(summary) + '\n')


def main():
    args = parse_args()
    run = Path(args.run_dir).resolve()
    outdir = run / 'analysis'
    outdir.mkdir(exist_ok=True)

    train_rows = load_csv_rows(run / 'train_log.csv')
    episode_rows = load_csv_rows(run / 'episode_log.csv')

    make_training_curves(train_rows, outdir)
    make_loss_curves_log(train_rows, outdir)
    make_episode_curves(episode_rows, outdir)
    write_summary(train_rows, episode_rows, outdir)

    print(outdir / 'training_curves.png')
    print(outdir / 'loss_curves_log.png')
    print(outdir / 'episode_curves.png')
    print(outdir / 'summary.txt')


if __name__ == '__main__':
    main()
