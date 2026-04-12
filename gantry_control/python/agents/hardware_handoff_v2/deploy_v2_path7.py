from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch

from policy_model import load_actor


class V2Path7HardwareRunner:
    """Minimal inference wrapper for the v2 path7 path-blind hardware handoff.

    This class does only the deployment-side work needed for closed-loop control:
    - keep a rolling 16-frame whisker history
    - build the 6D kinematics vector expected by the actor
    - run deterministic actor inference
    - convert the global-y action into a physical command

    It does not contain any SAC training logic, replay buffer, or critic networks.
    """

    def __init__(self, package_dir: str | Path, device: str = 'cpu'):
        self.package_dir = Path(package_dir)
        self.cfg = json.loads((self.package_dir / 'source_config.json').read_text())
        self.ckpt_info = json.loads((self.package_dir / 'checkpoint_info.json').read_text())
        actor_path = self.package_dir / self.ckpt_info['actor_only_weights']
        self.actor = load_actor(actor_path, device=device, kin_dim=6, action_dim=1)
        print(f'[Model] Loaded actor weights from {actor_path}')
        self.device = torch.device(device)
        self.history_steps = int(round(self.cfg['history_ms'] / self.cfg['sim_step_ms']))
        self.rl_interval = int(self.cfg['rl_interval'])
        self.fixed_x_speed = float(self.cfg['fixed_x_speed_mm_per_ms'])
        self.y_speed_limit = float(self.cfg['y_speed_limit_mm_per_ms'])
        self.vel_max = float(self.cfg['vel_max_mm_per_ms'])
        self.reset_episode()

    def reset_episode(self) -> None:
        """Reset all rolling state kept on the deployment side for a new trial."""
        self.sensor_history = np.zeros((self.history_steps, 3, 3, 2), dtype=np.float32)
        self.history_initialized = False
        self.frames_since_action = 0
        self.prev_y_velocity = 0.0

    def push_sensor_frame(self, sensor_frame: np.ndarray) -> None:
        """Append one new 80 Hz whisker frame into the rolling 16-frame history."""
        frame = np.asarray(sensor_frame, dtype=np.float32).reshape(3, 3, 2)
        if not self.history_initialized:
            self.sensor_history[:] = frame
            self.history_initialized = True
        else:
            self.sensor_history = np.concatenate([self.sensor_history, frame[None, ...]], axis=0)[-self.history_steps:]
        self.frames_since_action += 1

    def ready_for_action(self) -> bool:
        """Return True when a new 20 Hz control decision is due."""
        return self.history_initialized and self.frames_since_action >= self.rl_interval

    def _build_kinematics(self, x_mm: float, y_mm: float, vx_mm_per_ms: float, vy_mm_per_ms: float, time_ms: float) -> np.ndarray:
        """Build the 6D kinematic feature vector in the exact training-time field order."""
        time_norm = float(time_ms) / max(float(self.cfg['episode_time_ms']), 1.0)
        return np.array([
            float(x_mm),
            float(y_mm),
            float(vx_mm_per_ms),
            float(vy_mm_per_ms),
            float(self.prev_y_velocity),
            float(time_norm),
        ], dtype=np.float32)

    def compute_command(self, x_mm: float, y_mm: float, vx_mm_per_ms: float, vy_mm_per_ms: float, time_ms: float):
        """Run one actor inference step when enough new frames have arrived."""
        if not self.ready_for_action():
            return None

        kin = self._build_kinematics(x_mm, y_mm, vx_mm_per_ms, vy_mm_per_ms, time_ms)
        sensor_t = torch.as_tensor(self.sensor_history[None, ...], dtype=torch.float32, device=self.device)
        kin_t = torch.as_tensor(kin[None, ...], dtype=torch.float32, device=self.device)
        with torch.no_grad():
            action = self.actor(sensor_t, kin_t).cpu().numpy()[0, 0]

        y_vel = float(np.clip(action, -1.0, 1.0) * self.y_speed_limit)
        velocity_xy = np.array([self.fixed_x_speed, y_vel], dtype=np.float32)
        speed = float(np.linalg.norm(velocity_xy))
        speed_ratio = min(speed / self.vel_max, 1.0)
        direction_radian = float(np.arctan2(velocity_xy[1], velocity_xy[0]))

        self.prev_y_velocity = y_vel
        self.frames_since_action = 0

        return {
            'normalized_action': float(action),
            'y_velocity_mm_per_ms': y_vel,
            'command_vx_mm_per_ms': float(velocity_xy[0]),
            'command_vy_mm_per_ms': float(velocity_xy[1]),
            'speed_ratio': speed_ratio,
            'direction_radian': direction_radian,
        }


def main() -> None:
    parser = argparse.ArgumentParser(description='Minimal hardware-side inference wrapper for the v2 path7 checkpoint.')
    parser.add_argument('--package-dir', type=str, default='.')
    parser.add_argument('--device', type=str, default='cpu')
    args = parser.parse_args()

    runner = V2Path7HardwareRunner(args.package_dir, device=args.device)
    print('Runner initialized successfully.')
    print(f"Selected checkpoint: {runner.ckpt_info['selected_checkpoint']}")
    print('Call push_sensor_frame(...) every 12.5 ms and compute_command(...) every 4 frames.')


if __name__ == '__main__':
    main()
