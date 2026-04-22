"""Replay buffer for the path-blind SAC baseline.

The buffer stores the same two-part observation interface used by the new path-blind policy:
- a spatiotemporal whisker tensor
- a small kinematic feature vector

The implementation is intentionally plain NumPy for readability.  There is no
fancy prioritization or memory compression yet.
"""

from __future__ import annotations

import os
from pathlib import Path
import tempfile

import numpy as np
import torch


# FIFO replay buffer storing (sensor_history, kin, action, reward, next_*) tuples.
class ReplayBuffer:
    """Simple FIFO replay buffer for off-policy learning."""

    def __init__(self, sensor_shape, kin_dim: int, action_dim: int, capacity: int):
        # Every array stores one field from the transition tuple:
        # (sensor_t, kin_t, action_t, reward_t, sensor_t+1, kin_t+1, done_t).
        #
        # Shapes:
        # - sensor:      [capacity, *sensor_shape]
        # - kin:         [capacity, kin_dim]
        # - actions:     [capacity, action_dim]
        # - rewards:     [capacity, 1]
        # - next_sensor: [capacity, *sensor_shape]
        # - next_kin:    [capacity, kin_dim]
        # - dones:       [capacity, 1]
        self.capacity = int(capacity)
        self.sensor = np.zeros((self.capacity, *sensor_shape), dtype=np.float32)
        self.kin = np.zeros((self.capacity, kin_dim), dtype=np.float32)
        self.actions = np.zeros((self.capacity, action_dim), dtype=np.float32)
        self.rewards = np.zeros((self.capacity, 1), dtype=np.float32)
        self.next_sensor = np.zeros((self.capacity, *sensor_shape), dtype=np.float32)
        self.next_kin = np.zeros((self.capacity, kin_dim), dtype=np.float32)
        self.dones = np.zeros((self.capacity, 1), dtype=np.float32)
        self.ptr = 0
        self.size = 0

    # Insert a vectorized step into replay, skipping invalid envs.
    def store_batch(self, sensor, kin, action, reward, next_sensor, next_kin, done, valid_mask=None):
        """Insert one vectorized environment step into replay.

        Parameters are expected to have a leading batch dimension equal to the
        number of parallel environments used during data collection.

        ``valid_mask`` lets the trainer skip entries that came from hardware/I-O infrastructure failures (timeouts, command failures, sensor timeouts).
        We do not want those failure-driven transitions to pollute replay and
        confuse the SAC updates.
        """
        sensor = np.asarray(sensor, dtype=np.float32)
        kin = np.asarray(kin, dtype=np.float32)
        action = np.asarray(action, dtype=np.float32)
        reward = np.asarray(reward, dtype=np.float32).reshape(-1, 1)
        next_sensor = np.asarray(next_sensor, dtype=np.float32)
        next_kin = np.asarray(next_kin, dtype=np.float32)
        done = np.asarray(done, dtype=np.float32).reshape(-1, 1)

        batch_size = sensor.shape[0]
        if valid_mask is None:
            valid_mask = np.ones((batch_size,), dtype=bool)
        for i in range(batch_size):
            if not bool(valid_mask[i]):
                continue

            self.sensor[self.ptr] = sensor[i]
            self.kin[self.ptr] = kin[i]
            self.actions[self.ptr] = action[i]
            self.rewards[self.ptr, 0] = float(reward[i, 0])
            self.next_sensor[self.ptr] = next_sensor[i]
            self.next_kin[self.ptr] = next_kin[i]
            self.dones[self.ptr, 0] = float(done[i, 0])

            self.ptr = (self.ptr + 1) % self.capacity
            self.size = min(self.size + 1, self.capacity)

    # Random mini-batch for SAC updates.
    def sample(self, batch_size: int, device: str):
        """Sample a random mini-batch and return torch tensors.

        The sampled tensors have shapes:
        - sensor: ``[batch_size, *sensor_shape]``
        - kin: ``[batch_size, kin_dim]``
        - action: ``[batch_size, action_dim]``
        - reward: ``[batch_size, 1]``
        - done: ``[batch_size, 1]``
        """
        idx = np.random.randint(0, self.size, size=batch_size)
        batch = {
            'sensor': torch.as_tensor(self.sensor[idx], device=device),
            'kin': torch.as_tensor(self.kin[idx], device=device),
            'action': torch.as_tensor(self.actions[idx], device=device),
            'reward': torch.as_tensor(self.rewards[idx], device=device),
            'next_sensor': torch.as_tensor(self.next_sensor[idx], device=device),
            'next_kin': torch.as_tensor(self.next_kin[idx], device=device),
            'done': torch.as_tensor(self.dones[idx], device=device),
        }
        return batch

    # Persist replay buffer to disk for resume/debug.
    def save(self, path: str):
        """Save replay contents and pointers to a NumPy archive atomically."""
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        tmp_path: Path | None = None
        try:
            with tempfile.NamedTemporaryFile(dir=target.parent, suffix='.npz', delete=False) as handle:
                tmp_path = Path(handle.name)
            np.savez(
                tmp_path,
                sensor=self.sensor,
                kin=self.kin,
                actions=self.actions,
                rewards=self.rewards,
                next_sensor=self.next_sensor,
                next_kin=self.next_kin,
                dones=self.dones,
                ptr=np.array([self.ptr], dtype=np.int64),
                size=np.array([self.size], dtype=np.int64),
            )
            os.replace(tmp_path, target)
        finally:
            if tmp_path is not None and tmp_path.exists():
                tmp_path.unlink()

    # Restore replay buffer from disk for resume.
    def load(self, path: str):
        """Load replay contents from a NumPy archive.

        The on-disk archive may have been created with a different replay
        capacity. When that happens, load the newest transitions that fit in the
        current buffer rather than failing on a shape mismatch.
        """
        data = np.load(path)
        stored_sensor = data['sensor']
        stored_kin = data['kin']
        stored_actions = data['actions']
        stored_rewards = data['rewards']
        stored_next_sensor = data['next_sensor']
        stored_next_kin = data['next_kin']
        stored_dones = data['dones']

        expected_shapes = {
            'sensor': self.sensor.shape[1:],
            'kin': self.kin.shape[1:],
            'actions': self.actions.shape[1:],
            'rewards': self.rewards.shape[1:],
            'next_sensor': self.next_sensor.shape[1:],
            'next_kin': self.next_kin.shape[1:],
            'dones': self.dones.shape[1:],
        }
        actual_shapes = {
            'sensor': stored_sensor.shape[1:],
            'kin': stored_kin.shape[1:],
            'actions': stored_actions.shape[1:],
            'rewards': stored_rewards.shape[1:],
            'next_sensor': stored_next_sensor.shape[1:],
            'next_kin': stored_next_kin.shape[1:],
            'dones': stored_dones.shape[1:],
        }
        mismatches = [
            key for key in expected_shapes
            if actual_shapes[key] != expected_shapes[key]
        ]
        if mismatches:
            details = ', '.join(
                f'{key}: stored={actual_shapes[key]} current={expected_shapes[key]}'
                for key in mismatches
            )
            raise ValueError(f'Replay buffer feature shape mismatch: {details}')

        stored_capacity = int(stored_sensor.shape[0])
        stored_size = max(0, min(int(data['size'][0]), stored_capacity))
        stored_ptr = int(data['ptr'][0]) % max(stored_capacity, 1)

        # Reset first so partially loaded state never leaks through.
        self.sensor.fill(0.0)
        self.kin.fill(0.0)
        self.actions.fill(0.0)
        self.rewards.fill(0.0)
        self.next_sensor.fill(0.0)
        self.next_kin.fill(0.0)
        self.dones.fill(0.0)

        if stored_size <= 0:
            self.ptr = 0
            self.size = 0
            return

        if stored_size < stored_capacity:
            ordered_idx = np.arange(stored_size, dtype=np.int64)
        else:
            ordered_idx = np.concatenate([
                np.arange(stored_ptr, stored_capacity, dtype=np.int64),
                np.arange(0, stored_ptr, dtype=np.int64),
            ])

        load_count = min(stored_size, self.capacity)
        take_idx = ordered_idx[-load_count:]

        self.sensor[:load_count] = stored_sensor[take_idx]
        self.kin[:load_count] = stored_kin[take_idx]
        self.actions[:load_count] = stored_actions[take_idx]
        self.rewards[:load_count] = stored_rewards[take_idx]
        self.next_sensor[:load_count] = stored_next_sensor[take_idx]
        self.next_kin[:load_count] = stored_next_kin[take_idx]
        self.dones[:load_count] = stored_dones[take_idx]
        self.ptr = load_count % self.capacity
        self.size = load_count
