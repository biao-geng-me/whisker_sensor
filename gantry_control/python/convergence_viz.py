"""Convergence visualization subprocess.

ConvergenceVizProcess — spawns a subprocess that owns three persistent matplotlib
windows (episode curves, training curves, loss curves), CSV writing, and PNG saving.
The main control loop just fire-and-forgets row dicts through a queue.
"""

from __future__ import annotations

import csv
import multiprocessing as mp
import queue as _queue_mod
from pathlib import Path

_QUEUE_MAXSIZE = 200
_PAUSE_S = 0.05   # ~20 Hz GUI update

_TERM_COLORS = {
    'y_boundary':         'tab:brown',
    'too_far':            'tab:red',
    'too_close':          'tab:orange',
    'time_limit':         'tab:blue',
    'runtime_done':       'tab:green',
    'runtime_truncated':  'tab:cyan',
    'done':               'tab:olive',
}


def _write_log_csv(path: Path, rows: list, fieldnames: list):
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


# ── per-figure update functions ───────────────────────────────────────────────

def _update_episode_curves(fig, ax_ret, ax_lat, episode_log_rows: list, out_dir: Path | None):
    ep_idx = list(range(1, len(episode_log_rows) + 1))
    try:
        returns  = [float(r['episode_return'])          for r in episode_log_rows]
        laterals = [float(r['signed_lateral_error_mm']) for r in episode_log_rows]
        reasons  = [r['termination_reason']             for r in episode_log_rows]
        colors   = [_TERM_COLORS.get(r, 'tab:gray')     for r in reasons]
    except (KeyError, ValueError):
        return

    for ax in (ax_ret, ax_lat):
        ax.cla()

    ax_ret.set_title('Episode Return')
    ax_ret.set_xlabel('Episode')
    ax_ret.set_ylabel('Return')
    ax_ret.grid(True, alpha=0.3)
    ax_ret.plot(ep_idx, returns, color='0.6', alpha=0.5, lw=1)
    ax_ret.scatter(ep_idx, returns, c=colors, s=35, zorder=3)

    ax_lat.set_title('Episode-End Signed Lateral Error')
    ax_lat.set_xlabel('Episode')
    ax_lat.set_ylabel('Error (mm)')
    for level, color in [(180, 'tab:orange'), (-180, 'tab:orange'),
                         (240, 'tab:red'),    (-240, 'tab:red')]:
        ax_lat.axhline(level, ls='--', color=color, alpha=0.5)
    ax_lat.grid(True, alpha=0.3)
    ax_lat.plot(ep_idx, laterals, color='0.6', alpha=0.5, lw=1)
    ax_lat.scatter(ep_idx, laterals, c=colors, s=35, zorder=3)

    from matplotlib.lines import Line2D
    present = [k for k in _TERM_COLORS if k in set(reasons)]
    if present:
        handles = [Line2D([0], [0], marker='o', color='w',
                          markerfacecolor=_TERM_COLORS[k], markersize=8, label=k)
                   for k in present]
        ax_ret.legend(handles=handles, title='termination', loc='best', fontsize=7)

    fig.suptitle(f'Episode Curves — episode {len(episode_log_rows)}')
    fig.canvas.draw_idle()
    if out_dir is not None:
        try:
            fig.savefig(out_dir / 'episode_curves.png', dpi=150)
        except Exception:
            pass


def _update_training_curves(fig, axes, train_log_rows: list, out_dir: Path | None):
    for ax in axes.flat:
        ax.cla()

    steps      = [int(r['total_env_steps'])            for r in train_log_rows]
    mean_rwd   = [float(r['mean_reward_this_episode']) for r in train_log_rows]
    mean_lat   = [float(r['mean_lateral_error_mm'])    for r in train_log_rows]
    ep_return  = [float(r['episode_return'])           for r in train_log_rows]
    alpha_rows = [(int(r['total_env_steps']), float(r['alpha']))
                  for r in train_log_rows if r.get('alpha') not in ('', None)]
    a_steps, alphas = zip(*alpha_rows) if alpha_rows else ([], [])

    ax1, ax2, ax3, ax4 = axes.flat
    ax1.plot(steps, mean_rwd,  lw=2)
    ax1.set_title('Mean Reward vs Env Steps')
    ax1.set_xlabel('Env steps')
    ax1.set_ylabel('Mean reward')
    ax1.grid(True, alpha=0.3)

    ax2.plot(steps, mean_lat, lw=2, color='tab:orange')
    for level, color in [(180, 'tab:orange'), (-180, 'tab:orange'),
                         (240, 'tab:red'),    (-240, 'tab:red')]:
        ax2.axhline(level, ls='--', color=color, alpha=0.5)
    ax2.set_title('Mean Signed Lateral Error vs Env Steps')
    ax2.set_xlabel('Env steps')
    ax2.set_ylabel('Signed lateral error (mm)')
    ax2.grid(True, alpha=0.3)

    ax3.plot(steps, ep_return, lw=2, color='tab:green')
    ax3.set_title('Episode Return vs Env Steps')
    ax3.set_xlabel('Env steps')
    ax3.set_ylabel('Episode return')
    ax3.grid(True, alpha=0.3)

    if alphas:
        ax4.plot(list(a_steps), list(alphas), color='tab:purple')
    ax4.set_title('Entropy Temperature (alpha)')
    ax4.set_xlabel('Env steps')
    ax4.set_ylabel('alpha')
    ax4.grid(True, alpha=0.3)

    fig.suptitle(f'Training Curves — {steps[-1]} env steps')
    fig.canvas.draw_idle()
    if out_dir is not None:
        try:
            fig.savefig(out_dir / 'training_curves.png', dpi=150)
        except Exception:
            pass


def _update_loss_curves(fig, ax_critic, ax_actor, train_log_rows: list, out_dir: Path | None):
    ax_critic.cla()
    ax_actor.cla()

    def _pick(row, primary, fallback):
        v = row.get(primary, '')
        return v if v not in ('', None) else row.get(fallback, '')

    valid = [r for r in train_log_rows
             if _pick(r, 'actor_loss', 'actor_loss_last') not in ('', None)]
    if not valid:
        fig.canvas.draw_idle()
        return

    steps      = [int(r['total_env_steps'])                              for r in valid]
    actor_loss = [float(_pick(r, 'actor_loss',  'actor_loss_last'))      for r in valid]
    q1_loss    = [float(_pick(r, 'q1_loss',     'q1_loss_last'))         for r in valid]
    q2_loss    = [float(_pick(r, 'q2_loss',     'q2_loss_last'))         for r in valid]

    q1_pos = [(x, y) for x, y in zip(steps, q1_loss) if y > 0]
    q2_pos = [(x, y) for x, y in zip(steps, q2_loss) if y > 0]

    if q1_pos:
        ax_critic.semilogy(*zip(*q1_pos), label='q1_loss')
    if q2_pos:
        ax_critic.semilogy(*zip(*q2_pos), label='q2_loss')
    ax_critic.set_title('Critic Loss (log scale)')
    ax_critic.set_xlabel('Env steps')
    ax_critic.set_ylabel('Critic loss')
    ax_critic.grid(True, which='both', alpha=0.3)
    ax_critic.legend()

    ax_actor.plot(steps, actor_loss, label='actor_loss')
    ax_actor.set_yscale('symlog', linthresh=1e-2)
    ax_actor.axhline(0.0, color='0.5', ls='--', alpha=0.6)
    ax_actor.set_title('Actor Loss (symlog scale)')
    ax_actor.set_xlabel('Env steps')
    ax_actor.set_ylabel('Actor loss')
    ax_actor.grid(True, which='both', alpha=0.3)
    ax_actor.legend()

    fig.suptitle(f'Loss Curves — {steps[-1]} env steps')
    fig.canvas.draw_idle()
    if out_dir is not None:
        try:
            fig.savefig(out_dir / 'loss_curves_log.png', dpi=150)
        except Exception:
            pass


# ── subprocess worker ─────────────────────────────────────────────────────────

def _convergence_worker(q: mp.Queue, ckpt_output_dir: str,
                        initial_train_rows: list, initial_episode_rows: list):
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt

    import time

    # Create all three persistent figures up front
    fig_ep,    (ax_ret,    ax_lat)    = plt.subplots(2, 1, figsize=(10, 7))
    fig_train,  axes_train             = plt.subplots(2, 2, figsize=(12, 9))
    fig_loss,  (ax_critic, ax_actor)  = plt.subplots(2, 1, figsize=(10, 8))

    for fig, title in [(fig_ep,    'Episode Curves'),
                       (fig_train, 'Training Curves'),
                       (fig_loss,  'Loss Curves')]:
        fig.tight_layout()
        try:
            fig.canvas.manager.set_window_title(title)
        except Exception:
            pass

    plt.show(block=False)   # display all windows once; never called again in the loop

    train_log_rows   = list(initial_train_rows)
    episode_log_rows = list(initial_episode_rows)
    out_dir = Path(ckpt_output_dir)

    # Prime with any pre-loaded history
    if episode_log_rows:
        _update_episode_curves(fig_ep, ax_ret, ax_lat, episode_log_rows, None)
    if train_log_rows:
        _update_training_curves(fig_train, axes_train, train_log_rows, None)
        _update_loss_curves(fig_loss, ax_critic, ax_actor, train_log_rows, None)
    if episode_log_rows or train_log_rows:
        for fig in (fig_ep, fig_train, fig_loss):
            fig.canvas.flush_events()

    while True:
        # Drain all pending messages without blocking
        new_data = False
        while True:
            try:
                msg = q.get_nowait()
            except _queue_mod.Empty:
                break
            if msg["type"] == "shutdown":
                plt.close('all')
                return
            train_row = msg.get("train_row")
            ep_row    = msg.get("ep_row")
            if train_row:
                train_log_rows.append(train_row)
            if ep_row:
                episode_log_rows.append(ep_row)
            new_data = True

        if new_data:
            try:
                out_dir.mkdir(parents=True, exist_ok=True)
                if train_log_rows:
                    _write_log_csv(out_dir / 'train_log.csv', train_log_rows,
                                   list(train_log_rows[0].keys()))
                if episode_log_rows:
                    _write_log_csv(out_dir / 'episode_log.csv', episode_log_rows,
                                   list(episode_log_rows[0].keys()))
            except Exception as ex:
                print(f'[ConvergenceViz] CSV write error: {ex}')

            if episode_log_rows:
                _update_episode_curves(fig_ep, ax_ret, ax_lat,
                                       episode_log_rows, out_dir)
            if train_log_rows:
                _update_training_curves(fig_train, axes_train,
                                        train_log_rows, out_dir)
                _update_loss_curves(fig_loss, ax_critic, ax_actor,
                                    train_log_rows, out_dir)

        # Process GUI events without raising windows to the front.
        # plt.pause() calls plt.show() internally which steals focus — avoid it.
        try:
            for fig in (fig_ep, fig_train, fig_loss):
                fig.canvas.flush_events()
        except Exception:
            return
        time.sleep(_PAUSE_S)


# ── public API ────────────────────────────────────────────────────────────────

class ConvergenceVizProcess:
    """Non-blocking convergence viz. Spawns a subprocess that owns the three
    persistent plot windows, CSV writing, and PNG saving."""

    def __init__(self, ckpt_output_dir: Path,
                 initial_train_rows: list | None = None,
                 initial_episode_rows: list | None = None):
        self._q: mp.Queue = mp.Queue(maxsize=_QUEUE_MAXSIZE)
        self._proc = mp.Process(
            target=_convergence_worker,
            args=(self._q, str(ckpt_output_dir),
                  initial_train_rows or [], initial_episode_rows or []),
            daemon=True,
        )
        self._proc.start()

    def update(self, train_row: dict, ep_row: dict):
        try:
            self._q.put_nowait({"type": "row", "train_row": train_row, "ep_row": ep_row})
        except _queue_mod.Full:
            pass

    def shutdown(self):
        try:
            self._q.put_nowait({"type": "shutdown"})
        except _queue_mod.Full:
            pass
        self._proc.join(timeout=5)
        if self._proc.is_alive():
            self._proc.terminate()
