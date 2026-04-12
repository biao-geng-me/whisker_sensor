"""Hardware-only training entry point for path-blind SAC.

This file is intentionally organized around the hardware workflow rather than a
fully generic RL loop:
1. run one episode with a fixed policy
2. store all valid transitions in replay
3. update the networks during between-episode idle time
4. checkpoint, log, and continue

For a first-time reader, the most important thing to know is that policy
weights do not change mid-episode. Learning happens only after an episode ends.
"""

from __future__ import annotations

import argparse
import csv
import json
import random
import shutil
import sys
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np
import torch

from .config import SACV2PathblindConfig
from .hardware_adapter import create_training_env
from .models import SACAgent
from .plot_utils import save_rollout_plots
from .replay_buffer import ReplayBuffer

def _configure_unbuffered_output() -> None:
    """Request line-buffered stdout/stderr when supported by the runtime."""
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if not callable(reconfigure):
            continue
        try:
            reconfigure(line_buffering=True, write_through=True)
        except TypeError:
            reconfigure(line_buffering=True)
        except Exception:
            pass

_configure_unbuffered_output()

def set_seed(seed: int) -> None:
    """Seed Python, NumPy, and PyTorch for repeatable training runs."""
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)

def parse_path_list(raw_value: str | None):
    """Accept comma- or colon-separated path lists from CLI or shell variables."""
    if raw_value is None:
        return None
    cleaned = raw_value.replace(':', ',')
    items = tuple(part.strip() for part in cleaned.split(',') if part.strip())
    return items or None

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Train path-blind SAC on hardware (single-tank episode mode).'
    )
    parser.add_argument('--total-env-steps', type=int, default=None)
    parser.add_argument('--max-runtime-seconds', type=float, default=None)
    parser.add_argument('--path-subs', type=str, default=None)
    parser.add_argument('--eval-path-subs', type=str, default=None)
    parser.add_argument('--device', type=str, default=None)
    parser.add_argument('--io-timeout', '--timeout-get', dest='io_timeout', type=int, default=None)
    parser.add_argument('--checkpoint-every-episodes', type=int, default=None)
    parser.add_argument('--fixed-x-speed', type=float, default=None)
    parser.add_argument('--object-tangential-speed', type=float, default=None)
    parser.add_argument('--initial-object-gap', type=float, default=None)
    parser.add_argument('--min-object-x-gap-terminate', type=float, default=None)
    parser.add_argument('--terminate-corridor-half-width', type=float, default=None)
    parser.add_argument('--xloc-start', type=float, default=None)
    parser.add_argument('--yloc-start', type=float, default=None)
    parser.add_argument('--episode-time-ms', type=float, default=None)
    parser.add_argument('--replay-size', type=int, default=None)
    parser.add_argument('--batch-size', type=int, default=None)
    parser.add_argument('--start-steps', type=int, default=None)
    parser.add_argument('--update-after', type=int, default=None)
    parser.add_argument('--target-update-to-data-ratio', type=float, default=None)
    parser.add_argument('--max-updates-per-episode', type=int, default=None)
    parser.add_argument('--settle-time-seconds', type=float, default=None)
    parser.add_argument('--runtime-factory', type=str, default=None)
    parser.add_argument('--target-paths-root', type=str, default=None)
    parser.add_argument('--path-file-template', type=str, default=None)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--resume-path', type=str, default=None)
    parser.add_argument('--keep-checkpoints', type=int, default=5)
    parser.add_argument('--start-on-path-initial-point', action='store_true')
    parser.add_argument('--disable-episode-plots', action='store_true')
    parser.add_argument('--output-dir', type=str, default='rl_sac_v4_pathblind_hardware_runs/default')
    return parser.parse_args()

def config_to_json_dict(cfg: SACV2PathblindConfig) -> dict:
    """Serialize config to JSON-friendly values for run snapshots."""
    data = asdict(cfg)
    for key, value in list(data.items()):
        if isinstance(value, Path):
            data[key] = str(value)
    return data

def _move_optimizer_to_device(optim: torch.optim.Optimizer, device: str) -> None:
    """Move optimizer state tensors after loading a checkpoint on a new device."""
    for state in optim.state.values():
        for key, value in state.items():
            if torch.is_tensor(value):
                state[key] = value.to(device)

def _build_checkpoint(
    agent: SACAgent,
    total_env_steps: int,
    episodes_completed: int,
    replay_skipped_total: int,
    elapsed_seconds: float,
    cfg: SACV2PathblindConfig,
    replay_path: str | None,
) -> dict:
    """Package model, optimizer, replay, and RNG state into one resume snapshot."""
    return {
        'actor': agent.actor.state_dict(),
        'q1': agent.q1.state_dict(),
        'q2': agent.q2.state_dict(),
        'q1_target': agent.q1_target.state_dict(),
        'q2_target': agent.q2_target.state_dict(),
        'log_alpha': agent.log_alpha.detach().cpu(),
        'actor_optim': agent.actor_optim.state_dict(),
        'q1_optim': agent.q1_optim.state_dict(),
        'q2_optim': agent.q2_optim.state_dict(),
        'alpha_optim': agent.alpha_optim.state_dict(),
        'counters': {
            'total_env_steps': int(total_env_steps),
            'episodes_completed': int(episodes_completed),
            'replay_skipped_total': int(replay_skipped_total),
        },
        'elapsed_seconds': float(elapsed_seconds),
        'rng': {
            'python': random.getstate(),
            'numpy': np.random.get_state(),
            'torch': torch.get_rng_state(),
            'cuda': torch.cuda.get_rng_state_all() if torch.cuda.is_available() else None,
        },
        'cfg': config_to_json_dict(cfg),
        'replay_path': replay_path,
    }

def _checkpoint_paths(output_dir: Path, total_env_steps: int) -> tuple[Path, Path]:
    ckpt = output_dir / f'checkpoint_{total_env_steps:07d}.pt'
    replay = output_dir / f'replay_{total_env_steps:07d}.npz'
    return ckpt, replay

def _prune_checkpoints(output_dir: Path, keep: int) -> None:
    if keep <= 0:
        return
    ckpts = sorted(output_dir.glob('checkpoint_*.pt'))
    if len(ckpts) <= keep:
        return
    for ckpt in ckpts[:-keep]:
        step = ckpt.stem.split('_', 1)[1]
        replay = output_dir / f'replay_{step}.npz'
        ckpt.unlink(missing_ok=True)
        if replay.exists():
            replay.unlink()

def _save_checkpoint(
    agent: SACAgent,
    replay: ReplayBuffer,
    output_dir: Path,
    total_env_steps: int,
    episodes_completed: int,
    replay_skipped_total: int,
    elapsed_seconds: float,
    cfg: SACV2PathblindConfig,
    keep_checkpoints: int,
) -> Path:
    ckpt_path, replay_path = _checkpoint_paths(output_dir, total_env_steps)
    checkpoint = _build_checkpoint(
        agent=agent,
        total_env_steps=total_env_steps,
        episodes_completed=episodes_completed,
        replay_skipped_total=replay_skipped_total,
        elapsed_seconds=elapsed_seconds,
        cfg=cfg,
        replay_path=str(replay_path),
    )
    torch.save(checkpoint, ckpt_path)
    replay.save(str(replay_path))
    shutil.copyfile(ckpt_path, output_dir / 'latest_checkpoint.pt')
    shutil.copyfile(replay_path, output_dir / 'latest_replay.npz')
    _prune_checkpoints(output_dir, keep_checkpoints)
    return ckpt_path

def _save_final_checkpoint(
    agent: SACAgent,
    replay: ReplayBuffer,
    output_dir: Path,
    total_env_steps: int,
    episodes_completed: int,
    replay_skipped_total: int,
    elapsed_seconds: float,
    cfg: SACV2PathblindConfig,
    keep_checkpoints: int,
) -> Path:
    final_ckpt = output_dir / 'final_model.pt'
    final_replay = output_dir / 'final_replay.npz'
    checkpoint = _build_checkpoint(
        agent=agent,
        total_env_steps=total_env_steps,
        episodes_completed=episodes_completed,
        replay_skipped_total=replay_skipped_total,
        elapsed_seconds=elapsed_seconds,
        cfg=cfg,
        replay_path=str(final_replay),
    )
    torch.save(checkpoint, final_ckpt)
    replay.save(str(final_replay))
    shutil.copyfile(final_ckpt, output_dir / 'latest_checkpoint.pt')
    shutil.copyfile(final_replay, output_dir / 'latest_replay.npz')
    _prune_checkpoints(output_dir, keep_checkpoints)
    return final_ckpt

def _load_checkpoint(
    agent: SACAgent,
    replay: ReplayBuffer,
    checkpoint_path: Path,
    output_dir: Path,
    device: str,
):
    """Restore model, optimizer, replay, and RNG state from one checkpoint."""
    checkpoint = torch.load(checkpoint_path, map_location=device)
    agent.actor.load_state_dict(checkpoint['actor'])
    agent.q1.load_state_dict(checkpoint['q1'])
    agent.q2.load_state_dict(checkpoint['q2'])
    agent.q1_target.load_state_dict(checkpoint['q1_target'])
    agent.q2_target.load_state_dict(checkpoint['q2_target'])
    agent.log_alpha.data.copy_(checkpoint['log_alpha'].to(device))

    agent.actor_optim.load_state_dict(checkpoint['actor_optim'])
    agent.q1_optim.load_state_dict(checkpoint['q1_optim'])
    agent.q2_optim.load_state_dict(checkpoint['q2_optim'])
    agent.alpha_optim.load_state_dict(checkpoint['alpha_optim'])

    _move_optimizer_to_device(agent.actor_optim, device)
    _move_optimizer_to_device(agent.q1_optim, device)
    _move_optimizer_to_device(agent.q2_optim, device)
    _move_optimizer_to_device(agent.alpha_optim, device)

    agent.actor.train()
    agent.q1.train()
    agent.q2.train()

    replay_path = checkpoint.get('replay_path')
    rp = Path(replay_path) if replay_path else (output_dir / 'latest_replay.npz')
    if rp.exists():
        replay.load(str(rp))
    else:
        print(f'warning: replay file not found for resume: {rp}')
        latest = output_dir / 'latest_replay.npz'
        if rp != latest and latest.exists():
            replay.load(str(latest))
            print(f'warning: resumed replay from latest file: {latest}')

    rng = checkpoint.get('rng')
    if rng:
        random.setstate(rng['python'])
        np.random.set_state(rng['numpy'])
        torch.set_rng_state(rng['torch'])
        if torch.cuda.is_available() and rng.get('cuda') is not None:
            torch.cuda.set_rng_state_all(rng['cuda'])

    return checkpoint

def termination_reason(info: dict, truncated: bool) -> str:
    """Collapse many boolean flags into one human-readable episode ending label."""
    if info.get('command_failed', False):
        return 'command_failed'
    if info.get('sensor_timeout', False):
        return 'sensor_timeout'
    if info.get('state_timeout', False):
        return 'state_timeout'
    if info.get('hardware_error', False):
        return 'hardware_error'
    if info.get('too_close', False):
        return 'too_close'
    if info.get('too_far', False):
        return 'too_far'
    if info.get('finish_line_reached', False):
        return 'finish_line'
    if truncated or info.get('time_limit_reached', False):
        return 'time_limit'
    if info.get('runtime_done', False):
        return 'runtime_done'
    if info.get('runtime_truncated', False):
        return 'runtime_truncated'
    return 'done'

def is_infrastructure_failure(info: dict) -> bool:
    """Mark steps that should be excluded from replay because hardware failed."""
    return any([
        bool(info.get('infrastructure_failure', False)),
        bool(info.get('command_failed', False)),
        bool(info.get('sensor_timeout', False)),
        bool(info.get('state_timeout', False)),
        bool(info.get('hardware_error', False)),
    ])

def safe_reset_env(env, cfg: SACV2PathblindConfig):
    """Retry reset a few times so transient hardware hiccups do not kill the run."""
    last_exc = None
    total_attempts = cfg.reset_retry_attempts + 1
    for attempt in range(total_attempts):
        try:
            return env.reset()
        except Exception as exc:  # pragma: no cover - defensive recovery path
            last_exc = exc
            print(f'reset attempt {attempt + 1}/{total_attempts} failed: {exc}')
            if attempt < total_attempts - 1:
                time.sleep(cfg.reset_retry_delay_s)
    raise RuntimeError(f'Failed to reset env after {total_attempts} attempts.') from last_exc

def episode_update_budget(cfg: SACV2PathblindConfig, episode_length: int) -> int:
    """Convert the configured update-to-data ratio into a per-episode update count."""
    target = int(round(float(cfg.target_update_to_data_ratio) * max(int(episode_length), 0)))
    if cfg.target_update_to_data_ratio > 0.0 and episode_length > 0:
        target = max(target, 1)
    if cfg.max_updates_per_episode is not None:
        target = min(target, int(cfg.max_updates_per_episode))
    return max(target, 0)

def run_between_episode_updates(
    agent: SACAgent,
    replay: ReplayBuffer,
    cfg: SACV2PathblindConfig,
    episode_length: int,
) -> tuple[int, float, dict | None]:
    """Run SAC gradient steps only after an episode has finished collecting data."""
    warm_threshold = max(int(cfg.update_after), int(cfg.batch_size))
    if replay.size < warm_threshold:
        return 0, 0.0, None

    updates_to_run = episode_update_budget(cfg, episode_length)
    if updates_to_run <= 0:
        return 0, 0.0, None

    metrics = None
    t0 = time.monotonic()
    for _ in range(updates_to_run):
        metrics = agent.update(replay.sample(cfg.batch_size, cfg.device))
    update_seconds = time.monotonic() - t0
    return updates_to_run, update_seconds, metrics

def make_episode_step_row(
    step_idx: int,
    action: float,
    reward: float,
    kin: np.ndarray,
    info: dict,
    done: bool,
    truncated: bool,
    cfg: SACV2PathblindConfig,
) -> dict:
    """Build one rollout row for the per-episode CSV and trajectory plots."""
    x_mm = float(kin[0, 0])
    y_mm = float(kin[0, 1])
    vx = float(kin[0, 2])
    vy = float(kin[0, 3])
    time_ms = float(kin[0, 5] * cfg.episode_time_ms)

    return {
        'step': int(step_idx),
        'time_ms': time_ms,
        'path_sub': info.get('path_sub', ''),
        'x_mm': x_mm,
        'y_mm': y_mm,
        'vx_mm_per_ms': vx,
        'vy_mm_per_ms': vy,
        'action': float(action),
        'reward': float(reward),
        'signed_lateral_error_mm': float(info.get('signed_lateral_error_mm', np.nan)),
        'target_path_x_mm': float(info.get('target_path_x_mm', np.nan)),
        'target_path_y_mm': float(info.get('target_path_y_mm', np.nan)),
        'path_progress_mm': float(info.get('path_progress_mm', np.nan)),
        'object_x_mm': float(info.get('object_x_mm', np.nan)),
        'object_x_gap_mm': float(info.get('object_x_gap_mm', np.nan)),
        'too_far': bool(info.get('too_far', False)),
        'too_close': bool(info.get('too_close', False)),
        'finish_line_reached': bool(info.get('finish_line_reached', False)),
        'time_limit_reached': bool(info.get('time_limit_reached', False)),
        'runtime_done': bool(info.get('runtime_done', False)),
        'runtime_truncated': bool(info.get('runtime_truncated', False)),
        'command_failed': bool(info.get('command_failed', False)),
        'sensor_timeout': bool(info.get('sensor_timeout', False)),
        'state_timeout': bool(info.get('state_timeout', False)),
        'hardware_error': bool(info.get('hardware_error', False)),
        'infrastructure_failure': bool(info.get('infrastructure_failure', False)),
        'done': bool(done),
        'truncated': bool(truncated),
    }

def save_episode_artifacts(
    output_dir: Path,
    episode_idx: int,
    rows: list[dict],
    path_xy: np.ndarray,
    cfg: SACV2PathblindConfig,
) -> Path | None:
    """Write per-episode CSV and plots so a failed run still leaves breadcrumbs."""
    if not rows:
        return None

    episode_dir = output_dir / 'episodes' / f'episode_{episode_idx:06d}'
    episode_dir.mkdir(parents=True, exist_ok=True)

    csv_path = episode_dir / 'rollout.csv'
    with csv_path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    if cfg.save_episode_plots and (episode_idx % max(1, int(cfg.plot_every_episodes)) == 0):
        save_rollout_plots(rows, path_xy, episode_dir, cfg)

    return episode_dir

def main() -> None:
    args = parse_args()
    cfg = SACV2PathblindConfig()

    if args.total_env_steps is not None:
        cfg.total_env_steps = args.total_env_steps
    if args.max_runtime_seconds is not None:
        cfg.max_runtime_seconds = args.max_runtime_seconds

    parsed_paths = parse_path_list(args.path_subs)
    if parsed_paths is not None:
        cfg.path_subs = parsed_paths
        cfg.path_sub = parsed_paths[0]

    parsed_eval_paths = parse_path_list(args.eval_path_subs)
    if parsed_eval_paths is not None:
        cfg.eval_path_subs = parsed_eval_paths

    if args.device is not None:
        cfg.device = args.device
    if args.io_timeout is not None:
        cfg.io_timeout_s = args.io_timeout
    if args.checkpoint_every_episodes is not None:
        cfg.checkpoint_every_episodes = args.checkpoint_every_episodes
    if args.fixed_x_speed is not None:
        cfg.fixed_x_speed_mm_per_ms = args.fixed_x_speed
    if args.object_tangential_speed is not None:
        cfg.object_tangential_speed_mm_per_ms = float(args.object_tangential_speed)
    if args.initial_object_gap is not None:
        cfg.initial_object_gap_mm = float(args.initial_object_gap)
    if args.min_object_x_gap_terminate is not None:
        cfg.min_object_x_gap_terminate_mm = float(args.min_object_x_gap_terminate)
    if args.terminate_corridor_half_width is not None:
        cfg.terminate_corridor_half_width_mm = args.terminate_corridor_half_width
    if args.xloc_start is not None or args.yloc_start is not None:
        cfg.start_on_path_initial_point = False
        if args.xloc_start is not None:
            cfg.xloc_start_mm = args.xloc_start
        if args.yloc_start is not None:
            cfg.yloc_start_mm = args.yloc_start
    if args.start_on_path_initial_point:
        cfg.start_on_path_initial_point = True
    if args.episode_time_ms is not None:
        cfg.episode_time_ms = float(args.episode_time_ms)
    if args.replay_size is not None:
        cfg.replay_size = int(args.replay_size)
    if args.batch_size is not None:
        cfg.batch_size = int(args.batch_size)
    if args.start_steps is not None:
        cfg.start_steps = int(args.start_steps)
    if args.update_after is not None:
        cfg.update_after = int(args.update_after)
    if args.target_update_to_data_ratio is not None:
        cfg.target_update_to_data_ratio = float(args.target_update_to_data_ratio)
    if args.max_updates_per_episode is not None:
        cfg.max_updates_per_episode = int(args.max_updates_per_episode)
    if args.settle_time_seconds is not None:
        cfg.settle_time_seconds = float(args.settle_time_seconds)
    if args.runtime_factory is not None:
        cfg.hardware_runtime_factory = str(args.runtime_factory)
    if args.target_paths_root is not None:
        cfg.target_paths_root = Path(args.target_paths_root)
    if args.path_file_template is not None:
        cfg.path_file_template = str(args.path_file_template)
    if args.disable_episode_plots:
        cfg.save_episode_plots = False

    keep_checkpoints = max(1, int(args.keep_checkpoints))
    resume = bool(args.resume)
    resume_path = Path(args.resume_path) if args.resume_path else None

    set_seed(cfg.seed)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'config.json').write_text(json.dumps(config_to_json_dict(cfg), indent=2))

    print(f'training_paths={cfg.training_paths()}')
    eval_paths = tuple(cfg.eval_path_subs) if cfg.eval_path_subs else ()
    print(f'eval_paths={eval_paths}')
    print(f'episode_time_ms={cfg.episode_time_ms}')
    print(f'fixed_x_speed_mm_per_ms={cfg.fixed_x_speed_mm_per_ms}')
    print(f'object_tangential_speed_mm_per_ms={cfg.object_tangential_speed_mm_per_ms}')
    print(f'initial_object_gap_mm={cfg.initial_object_gap_mm}')
    print(f'min_object_x_gap_terminate_mm={cfg.min_object_x_gap_terminate_mm}')
    print(f'terminate_corridor_half_width_mm={cfg.terminate_corridor_half_width_mm}')
    print(f'target_update_to_data_ratio={cfg.target_update_to_data_ratio}')
    print(f'max_updates_per_episode={cfg.max_updates_per_episode}')
    print(f'settle_time_seconds={cfg.settle_time_seconds}')
    print(f'hardware_runtime_factory={cfg.hardware_runtime_factory}')
    print(f'target_paths_root={cfg.target_paths_root}')

    env = create_training_env(cfg)
    agent = SACAgent(kin_dim=cfg.kin_dim, action_dim=cfg.action_dim, cfg=cfg)
    replay = ReplayBuffer(
        sensor_shape=cfg.sensor_shape,
        kin_dim=cfg.kin_dim,
        action_dim=cfg.action_dim,
        capacity=cfg.replay_size,
    )

    episode_log_path = output_dir / 'episode_log.csv'
    train_log_path = output_dir / 'train_log.csv'

    episodes_completed = 0
    total_env_steps = 0
    replay_skipped_total = 0
    train_start_time = None

    episode_fields = [
        'total_env_steps', 'episodes_completed', 'path_sub', 'episode_return',
        'episode_length', 'updates_run', 'update_seconds',
        'signed_lateral_error_mm', 'path_progress_mm', 'object_x_gap_mm',
        'too_far', 'too_close', 'finish_line_reached', 'time_limit_reached',
        'runtime_done', 'runtime_truncated',
        'command_failed', 'sensor_timeout', 'state_timeout', 'hardware_error',
        'infrastructure_failure', 'termination_reason',
    ]

    train_fields = [
        'total_env_steps', 'episodes_completed', 'replay_size', 'replay_skipped_total',
        'elapsed_hours', 'episode_return', 'episode_length', 'path_sub',
        'mean_reward_this_episode', 'mean_lateral_error_mm', 'mean_object_x_gap_mm',
        'updates_run', 'update_seconds', 'actor_loss', 'q1_loss', 'q2_loss', 'alpha',
    ]

    stop_reason = None

    if resume:
        ckpt_path = resume_path or (output_dir / 'latest_checkpoint.pt')
        if not ckpt_path.exists():
            raise FileNotFoundError(f'Resume requested but checkpoint not found: {ckpt_path}')
        ckpt = _load_checkpoint(agent, replay, ckpt_path, output_dir, cfg.device)
        counters = ckpt.get('counters', {})
        total_env_steps = int(counters.get('total_env_steps', total_env_steps))
        episodes_completed = int(counters.get('episodes_completed', episodes_completed))
        replay_skipped_total = int(counters.get('replay_skipped_total', replay_skipped_total))
        elapsed_seconds = ckpt.get('elapsed_seconds')
        train_start_time = time.monotonic() - float(elapsed_seconds) if elapsed_seconds is not None else time.monotonic()
        print(f'resumed from {ckpt_path} at env_steps={total_env_steps} episodes={episodes_completed}')
    else:
        train_start_time = time.monotonic()

    try:
        episode_mode = 'a' if resume and episode_log_path.exists() else 'w'
        train_mode = 'a' if resume and train_log_path.exists() else 'w'
        with episode_log_path.open(episode_mode, newline='') as episode_f, train_log_path.open(train_mode, newline='') as train_f:
            episode_writer = csv.DictWriter(episode_f, fieldnames=episode_fields)
            train_writer = csv.DictWriter(train_f, fieldnames=train_fields)
            if episode_mode == 'w':
                episode_writer.writeheader()
            if train_mode == 'w':
                train_writer.writeheader()
            episode_f.flush()
            train_f.flush()

            (sensor, kin), _ = safe_reset_env(env, cfg)

            episode_return = 0.0
            episode_length = 0
            episode_rows: list[dict] = []
            episode_rewards: list[float] = []
            episode_infos: list[dict] = []

            while total_env_steps < cfg.total_env_steps:
                elapsed_seconds = time.monotonic() - train_start_time
                if cfg.max_runtime_seconds is not None and elapsed_seconds >= cfg.max_runtime_seconds:
                    stop_reason = 'runtime_limit'
                    print(f'stopping due to runtime limit after {elapsed_seconds:.1f}s at env_steps={total_env_steps}')
                    break

                if total_env_steps < cfg.start_steps:
                    action = np.random.uniform(-1.0, 1.0, size=(1, cfg.action_dim)).astype(np.float32)
                else:
                    action = agent.act(sensor, kin, deterministic=False).astype(np.float32)

                # One hardware control interval: command action, receive the next
                # batch of sensor frames, and decide whether this episode ended.
                (next_sensor, next_kin), reward, done, truncated, infos = env.step(action)
                done_or_truncated = np.logical_or(done, truncated)

                info0 = infos[0]
                valid_replay_mask = np.array([not is_infrastructure_failure(info0)], dtype=bool)
                replay_skipped_total += int((~valid_replay_mask).sum())

                replay.store_batch(
                    sensor=sensor,
                    kin=kin,
                    action=action,
                    reward=reward,
                    next_sensor=next_sensor,
                    next_kin=next_kin,
                    done=done_or_truncated.astype(np.float32),
                    valid_mask=valid_replay_mask,
                )

                sensor = next_sensor
                kin = next_kin
                total_env_steps += 1

                reward0 = float(reward[0])
                done0 = bool(done[0])
                truncated0 = bool(truncated[0])

                episode_return += reward0
                episode_length += 1
                episode_rewards.append(reward0)
                episode_infos.append(info0)
                episode_rows.append(
                    make_episode_step_row(
                        step_idx=episode_length,
                        action=float(action[0, 0]),
                        reward=reward0,
                        kin=kin,
                        info=info0,
                        done=done0,
                        truncated=truncated0,
                        cfg=cfg,
                    )
                )

                if not (done0 or truncated0):
                    continue

                episodes_completed += 1
                # Training updates happen only here, after the episode is over.
                reason = termination_reason(info0, truncated0)
                updates_run, update_seconds, metrics = run_between_episode_updates(
                    agent=agent,
                    replay=replay,
                    cfg=cfg,
                    episode_length=episode_length,
                )

                episode_writer.writerow({
                    'total_env_steps': int(total_env_steps),
                    'episodes_completed': int(episodes_completed),
                    'path_sub': info0.get('path_sub', ''),
                    'episode_return': float(episode_return),
                    'episode_length': int(episode_length),
                    'updates_run': int(updates_run),
                    'update_seconds': float(update_seconds),
                    'signed_lateral_error_mm': float(info0.get('signed_lateral_error_mm', np.nan)),
                    'path_progress_mm': float(info0.get('path_progress_mm', np.nan)),
                    'object_x_gap_mm': float(info0.get('object_x_gap_mm', np.nan)),
                    'too_far': bool(info0.get('too_far', False)),
                    'too_close': bool(info0.get('too_close', False)),
                    'finish_line_reached': bool(info0.get('finish_line_reached', False)),
                    'time_limit_reached': bool(info0.get('time_limit_reached', False)),
                    'runtime_done': bool(info0.get('runtime_done', False)),
                    'runtime_truncated': bool(info0.get('runtime_truncated', False)),
                    'command_failed': bool(info0.get('command_failed', False)),
                    'sensor_timeout': bool(info0.get('sensor_timeout', False)),
                    'state_timeout': bool(info0.get('state_timeout', False)),
                    'hardware_error': bool(info0.get('hardware_error', False)),
                    'infrastructure_failure': bool(info0.get('infrastructure_failure', False)),
                    'termination_reason': reason,
                })
                episode_f.flush()

                mean_reward_episode = float(np.mean(episode_rewards)) if episode_rewards else float('nan')
                mean_lateral_error = float(np.mean([i.get('signed_lateral_error_mm', np.nan) for i in episode_infos]))
                mean_object_x_gap = float(np.mean([i.get('object_x_gap_mm', np.nan) for i in episode_infos]))
                elapsed_hours = (time.monotonic() - train_start_time) / 3600.0

                train_writer.writerow({
                    'total_env_steps': int(total_env_steps),
                    'episodes_completed': int(episodes_completed),
                    'replay_size': int(replay.size),
                    'replay_skipped_total': int(replay_skipped_total),
                    'elapsed_hours': float(elapsed_hours),
                    'episode_return': float(episode_return),
                    'episode_length': int(episode_length),
                    'path_sub': info0.get('path_sub', ''),
                    'mean_reward_this_episode': mean_reward_episode,
                    'mean_lateral_error_mm': mean_lateral_error,
                    'mean_object_x_gap_mm': mean_object_x_gap,
                    'updates_run': int(updates_run),
                    'update_seconds': float(update_seconds),
                    'actor_loss': '' if metrics is None else float(metrics['actor_loss']),
                    'q1_loss': '' if metrics is None else float(metrics['q1_loss']),
                    'q2_loss': '' if metrics is None else float(metrics['q2_loss']),
                    'alpha': '' if metrics is None else float(metrics['alpha']),
                })
                train_f.flush()

                episode_dir = save_episode_artifacts(
                    output_dir=output_dir,
                    episode_idx=episodes_completed,
                    rows=episode_rows,
                    path_xy=env.path_xy,
                    cfg=cfg,
                )

                msg = (
                    f'episode={episodes_completed} path={info0.get("path_sub", "")} '
                    f'return={episode_return:.3f} len={episode_length} '
                    f'mean_lateral={mean_lateral_error:.1f}mm '
                    f'mean_gap={mean_object_x_gap:.1f}mm '
                    f'updates={updates_run} update_s={update_seconds:.1f} '
                    f'reason={reason}'
                )
                if episode_dir is not None:
                    msg += f' artifacts={episode_dir}'
                if metrics is not None:
                    msg += (
                        f' actor_loss={metrics["actor_loss"]:.3f}'
                        f' q1_loss={metrics["q1_loss"]:.3f}'
                        f' q2_loss={metrics["q2_loss"]:.3f}'
                        f' alpha={metrics["alpha"]:.3f}'
                    )
                print(msg)

                if episodes_completed % max(1, int(cfg.checkpoint_every_episodes)) == 0:
                    ckpt = _save_checkpoint(
                        agent=agent,
                        replay=replay,
                        output_dir=output_dir,
                        total_env_steps=total_env_steps,
                        episodes_completed=episodes_completed,
                        replay_skipped_total=replay_skipped_total,
                        elapsed_seconds=(time.monotonic() - train_start_time),
                        cfg=cfg,
                        keep_checkpoints=keep_checkpoints,
                    )
                    print(f'saved {ckpt}')

                if cfg.settle_time_seconds > 0.0:
                    print(f'settling for {cfg.settle_time_seconds:.1f}s before next episode...')
                    time.sleep(cfg.settle_time_seconds)

                (sensor, kin), _ = safe_reset_env(env, cfg)
                episode_return = 0.0
                episode_length = 0
                episode_rows = []
                episode_rewards = []
                episode_infos = []

            if stop_reason is None and total_env_steps >= cfg.total_env_steps:
                stop_reason = 'env_step_limit'

            final_ckpt = _save_final_checkpoint(
                agent=agent,
                replay=replay,
                output_dir=output_dir,
                total_env_steps=total_env_steps,
                episodes_completed=episodes_completed,
                replay_skipped_total=replay_skipped_total,
                elapsed_seconds=(time.monotonic() - train_start_time),
                cfg=cfg,
                keep_checkpoints=keep_checkpoints,
            )
            print(f'training complete ({stop_reason}), saved {final_ckpt}')
    finally:
        env.close()

if __name__ == '__main__':
    main()
