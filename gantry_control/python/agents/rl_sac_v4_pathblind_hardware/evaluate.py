"""Evaluate one trained path-blind SAC policy on hardware."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np
import torch

from .config import SACV2PathblindConfig
from .hardware_adapter import create_training_env
from .models import SACAgent
from .plot_utils import save_rollout_plots

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


PATH_FIELDS = {'target_paths_root'}


def parse_args():
    parser = argparse.ArgumentParser(description='Evaluate one trained path-blind SAC policy on hardware.')
    parser.add_argument('checkpoint', type=str)
    parser.add_argument('--steps', type=int, default=250)
    parser.add_argument('--output-dir', type=str, default=None)
    parser.add_argument('--io-timeout', '--timeout-get', dest='io_timeout', type=int, default=None)
    parser.add_argument('--path-sub', type=str, default=None)
    parser.add_argument('--xloc-start', type=float, default=None)
    parser.add_argument('--yloc-start', type=float, default=None)
    parser.add_argument('--runtime-factory', type=str, default=None)
    parser.add_argument('--target-paths-root', type=str, default=None)
    parser.add_argument('--path-file-template', type=str, default=None)
    return parser.parse_args()


def load_cfg_for_checkpoint(checkpoint: Path) -> SACV2PathblindConfig:
    cfg = SACV2PathblindConfig()
    cfg.device = 'cuda' if torch.cuda.is_available() else 'cpu'

    config_path = checkpoint.parent / 'config.json'
    if not config_path.exists():
        return cfg

    data = json.loads(config_path.read_text())
    for key, value in data.items():
        if key == 'timeout_get_s':
            cfg.io_timeout_s = value
            continue
        if not hasattr(cfg, key):
            continue
        if key in PATH_FIELDS:
            setattr(cfg, key, Path(value))
        else:
            setattr(cfg, key, value)
    return cfg


def main():
    args = parse_args()
    checkpoint = Path(args.checkpoint).resolve()
    cfg = load_cfg_for_checkpoint(checkpoint)

    if args.io_timeout is not None:
        cfg.io_timeout_s = args.io_timeout
    if args.runtime_factory is not None:
        cfg.hardware_runtime_factory = args.runtime_factory
    if args.target_paths_root is not None:
        cfg.target_paths_root = Path(args.target_paths_root)
    if args.path_file_template is not None:
        cfg.path_file_template = args.path_file_template

    if args.path_sub is not None:
        cfg.path_sub = args.path_sub
        cfg.path_subs = (args.path_sub,)
        cfg.eval_path_subs = (args.path_sub,)
    elif cfg.eval_path_subs:
        default_eval_path = tuple(path for path in cfg.eval_path_subs if path)[0]
        cfg.path_sub = default_eval_path
        cfg.path_subs = (default_eval_path,)
        cfg.eval_path_subs = (default_eval_path,)
    elif len(cfg.training_paths()) > 1:
        default_eval_path = cfg.training_paths()[0]
        cfg.path_sub = default_eval_path
        cfg.path_subs = (default_eval_path,)

    if args.xloc_start is not None or args.yloc_start is not None:
        cfg.start_on_path_initial_point = False
        if args.xloc_start is not None:
            cfg.xloc_start_mm = float(args.xloc_start)
        if args.yloc_start is not None:
            cfg.yloc_start_mm = float(args.yloc_start)

    if args.output_dir is None:
        output_dir = checkpoint.parent / f'eval_{checkpoint.stem}'
    else:
        output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    env = create_training_env(cfg)
    agent = SACAgent(kin_dim=cfg.kin_dim, action_dim=cfg.action_dim, cfg=cfg)
    agent.load(str(checkpoint))

    rows = []
    total_return = 0.0

    try:
        (sensor, kin), _ = env.reset()
        for step_idx in range(args.steps):
            action = agent.act(sensor, kin, deterministic=True)
            (sensor, kin), reward, done, truncated, infos = env.step(action)
            info = infos[0]
            total_return += float(reward[0])

            x_mm = float(kin[0, 0])
            y_mm = float(kin[0, 1])
            vx = float(kin[0, 2])
            vy = float(kin[0, 3])
            time_ms = float(kin[0, 5] * cfg.episode_time_ms)

            row = {
                'step': step_idx + 1,
                'time_ms': time_ms,
                'path_sub': info.get('path_sub', cfg.path_sub),
                'x_mm': x_mm,
                'y_mm': y_mm,
                'vx_mm_per_ms': vx,
                'vy_mm_per_ms': vy,
                'action': float(action[0, 0]),
                'reward': float(reward[0]),
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
                'done': bool(done[0]),
                'truncated': bool(truncated[0]),
            }
            rows.append(row)

            print(
                f'step={step_idx + 1} path={row["path_sub"]} action={action[0,0]:+.3f} '
                f'reward={reward[0]:+.3f} lateral={row["signed_lateral_error_mm"]:+.1f}mm '
                f'gap={row["object_x_gap_mm"]:+.1f}mm '
                f'done={done[0]} trunc={truncated[0]}'
            )
            if done[0] or truncated[0]:
                break
    finally:
        env.close()

    if not rows:
        raise RuntimeError('No rollout rows were collected during evaluation.')

    csv_path = output_dir / 'rollout.csv'
    with csv_path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    save_rollout_plots(rows, env.path_xy, output_dir, cfg)

    summary = [
        f'checkpoint={checkpoint}',
        f'path_sub={rows[-1]["path_sub"]}',
        f'num_steps={len(rows)}',
        f'total_return={total_return:.6f}',
        f'final_x_mm={rows[-1]["x_mm"]:.3f}',
        f'final_y_mm={rows[-1]["y_mm"]:.3f}',
        f'final_lateral_error_mm={rows[-1]["signed_lateral_error_mm"]:.3f}',
        f'final_object_x_gap_mm={rows[-1]["object_x_gap_mm"]:.3f}',
        f'final_too_far={rows[-1]["too_far"]}',
        f'final_too_close={rows[-1]["too_close"]}',
        f'final_done={rows[-1]["done"]}',
        f'final_truncated={rows[-1]["truncated"]}',
    ]
    (output_dir / 'summary.txt').write_text('\n'.join(summary) + '\n')

    print(f'Wrote {csv_path}')
    print(f'Wrote {output_dir / "trajectory.png"}')
    print(f'Wrote {output_dir / "metrics.png"}')
    print(f'Wrote {output_dir / "summary.txt"}')


if __name__ == '__main__':
    main()
