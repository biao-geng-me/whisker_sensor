from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np
import torch


PACKAGE_DIR = Path(__file__).resolve().parent
REPO_ROOT = PACKAGE_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from policy_model import load_actor


PATH_FIELDS = {"flow_data_root", "wageng", "ccx", "eval_job_script"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run one deterministic simulator rollout using the v2 paths16 hardware handoff package."
    )
    parser.add_argument("--package-dir", type=str, default=".")
    parser.add_argument(
        "--weights",
        type=str,
        default=None,
        help="Optional weights file inside the package. Defaults to actor-only weights.",
    )
    parser.add_argument("--steps", type=int, default=250)
    parser.add_argument("--output-dir", type=str, default=None)
    parser.add_argument("--timeout-get", type=int, default=None)
    parser.add_argument("--path-sub", type=str, default=None)
    parser.add_argument("--xloc-start", type=float, default=None)
    parser.add_argument("--yloc-start", type=float, default=None)
    parser.add_argument("--device", type=str, default="cpu")
    return parser.parse_args()


def load_cfg_from_package(package_dir: Path, device: str):
    from rl_sac_v2_pathblind.config import SACV2PathblindConfig

    cfg = SACV2PathblindConfig()
    cfg.device = device

    config_path = package_dir / "source_config.json"
    if not config_path.exists():
        return cfg

    data = json.loads(config_path.read_text())
    for key, value in data.items():
        if not hasattr(cfg, key):
            continue
        if key in PATH_FIELDS:
            setattr(cfg, key, Path(value))
        else:
            setattr(cfg, key, value)

    cfg.num_envs = 1
    cfg.auto_submit_eval_jobs = False
    return cfg


def select_weights_file(package_dir: Path, requested_name: str | None) -> tuple[Path, dict]:
    ckpt_info = json.loads((package_dir / "checkpoint_info.json").read_text())
    if requested_name:
        weights_path = package_dir / requested_name
    else:
        default_name = ckpt_info.get("actor_only_weights", ckpt_info["selected_checkpoint"])
        weights_path = package_dir / default_name
    return weights_path, ckpt_info


def actor_action(actor, sensor: np.ndarray, kin: np.ndarray, device: str) -> np.ndarray:
    sensor_t = torch.as_tensor(sensor, dtype=torch.float32, device=device)
    kin_t = torch.as_tensor(kin, dtype=torch.float32, device=device)
    with torch.no_grad():
        action = actor(sensor_t, kin_t).cpu().numpy().astype(np.float32)
    return action


def main() -> None:
    args = parse_args()

    from rl_sac_v2_pathblind.env_adapter import WhiskerWakeTrackingVecEnv
    from rl_sac_v2_pathblind.evaluate import save_rollout_plots

    package_dir = Path(args.package_dir).resolve()
    cfg = load_cfg_from_package(package_dir, device=args.device)

    if args.timeout_get is not None:
        cfg.timeout_get_s = args.timeout_get
    cfg.num_envs = 1

    if args.path_sub is not None:
        cfg.path_sub = args.path_sub
        cfg.path_subs = (args.path_sub,)
        cfg.eval_path_subs = (args.path_sub,)
    elif len(cfg.training_paths()) > 1:
        default_eval_path = cfg.resolved_eval_paths()[0]
        cfg.path_sub = default_eval_path
        cfg.path_subs = (default_eval_path,)
        cfg.eval_path_subs = (default_eval_path,)

    if args.xloc_start is not None or args.yloc_start is not None:
        cfg.start_on_path_initial_point = False
        if args.xloc_start is not None:
            cfg.xloc_start_mm = float(args.xloc_start)
        if args.yloc_start is not None:
            cfg.yloc_start_mm = float(args.yloc_start)

    weights_path, ckpt_info = select_weights_file(package_dir, args.weights)
    actor = load_actor(weights_path, device=args.device, kin_dim=cfg.kin_dim, action_dim=cfg.action_dim)

    if args.output_dir is None:
        output_dir = package_dir / f"sim_eval_{weights_path.stem}"
    else:
        output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    env = WhiskerWakeTrackingVecEnv(cfg)
    rows: list[dict] = []
    total_return = 0.0

    try:
        (sensor, kin), _ = env.reset()
        for step_idx in range(args.steps):
            action = actor_action(actor, sensor, kin, args.device)
            (sensor, kin), reward, done, truncated, infos = env.step(action)
            info = infos[0]
            total_return += float(reward[0])

            x_mm = float(kin[0, 0])
            y_mm = float(kin[0, 1])
            vx = float(kin[0, 2])
            vy = float(kin[0, 3])
            time_ms = float(kin[0, 5] * cfg.episode_time_ms)

            row = {
                "step": step_idx + 1,
                "time_ms": time_ms,
                "path_sub": info.get("path_sub", cfg.path_sub),
                "x_mm": x_mm,
                "y_mm": y_mm,
                "vx_mm_per_ms": vx,
                "vy_mm_per_ms": vy,
                "action": float(action[0, 0]),
                "reward": float(reward[0]),
                "signed_lateral_error_mm": float(info["signed_lateral_error_mm"]),
                "object_x_gap_mm": float(info["object_x_gap_mm"]),
                "object_x_mm": float(info.get("object_x_mm", np.nan)),
                "object_y_mm": float(info.get("object_y_mm", np.nan)),
                "path_progress_mm": float(info["path_progress_mm"]),
                "object_progress_mm": float(info["object_progress_mm"]),
                "too_far": bool(info["too_far"]),
                "too_close": bool(info["too_close"]),
                "base_done": bool(info["base_done"]),
                "base_truncated": bool(info["base_truncated"]),
                "finish_line_reached": bool(info.get("finish_line_reached", False)),
                "time_limit_reached": bool(info.get("time_limit_reached", False)),
                "send_action_failed": bool(info.get("send_action_failed", False)),
                "recv_state_failed": bool(info.get("recv_state_failed", False)),
                "worker_connection_lost": bool(info.get("worker_connection_lost", False)),
                "state_timeout": bool(info.get("state_timeout", False)),
                "base_termination_reason": info.get("base_termination_reason", ""),
                "done": bool(done[0]),
                "truncated": bool(truncated[0]),
            }
            rows.append(row)

            print(
                f"step={step_idx + 1} path={row['path_sub']} action={action[0,0]:+.3f} "
                f"reward={reward[0]:+.3f} lateral={info['signed_lateral_error_mm']:+.1f}mm "
                f"x_gap={info['object_x_gap_mm']:+.1f}mm done={done[0]} trunc={truncated[0]}"
            )
            if done[0] or truncated[0]:
                break
    finally:
        env.close()

    if not rows:
        raise RuntimeError("No rollout rows were collected.")

    csv_path = output_dir / "rollout.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    save_rollout_plots(rows, env.path_xy, output_dir, cfg)

    summary = [
        f"package_dir={package_dir}",
        f"weights={weights_path}",
        f"selection_basis={ckpt_info.get('selection_basis', '')}",
        f"path_sub={rows[-1]['path_sub']}",
        f"num_steps={len(rows)}",
        f"total_return={total_return:.6f}",
        f"final_x_mm={rows[-1]['x_mm']:.3f}",
        f"final_y_mm={rows[-1]['y_mm']:.3f}",
        f"final_lateral_error_mm={rows[-1]['signed_lateral_error_mm']:.3f}",
        f"final_object_x_gap_mm={rows[-1]['object_x_gap_mm']:.3f}",
        f"final_done={rows[-1]['done']}",
        f"final_truncated={rows[-1]['truncated']}",
        f"base_done={rows[-1]['base_done']}",
        f"base_truncated={rows[-1]['base_truncated']}",
        f"finish_line_reached={rows[-1].get('finish_line_reached', False)}",
        f"worker_connection_lost={rows[-1].get('worker_connection_lost', False)}",
        f"state_timeout={rows[-1].get('state_timeout', False)}",
        f"base_termination_reason={rows[-1].get('base_termination_reason', '')}",
    ]
    (output_dir / "summary.txt").write_text("\n".join(summary) + "\n")

    print(csv_path)
    print(output_dir / "trajectory.png")
    print(output_dir / "metrics.png")
    print(output_dir / "summary.txt")


if __name__ == "__main__":
    main()
