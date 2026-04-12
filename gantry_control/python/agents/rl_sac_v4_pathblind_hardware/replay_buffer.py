"""Replay buffer for the path-blind SAC baseline.

The buffer stores the same two-part observation interface used by the new path-blind policy:
- a spatiotemporal whisker tensor
- a small kinematic feature vector

The implementation is intentionally plain NumPy for readability.  There is no
fancy prioritization or memory compression yet.
"""

from __future__ import annotations

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
        """Save replay contents and pointers to a NumPy archive."""
        np.savez(
            path,
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

    # Restore replay buffer from disk for resume.
    def load(self, path: str):
        """Load replay contents from a NumPy archive."""
        data = np.load(path)
        if data['sensor'].shape != self.sensor.shape:
            raise ValueError('Replay buffer shape mismatch; cannot resume with different capacity or shapes.')
        self.sensor[:] = data['sensor']
        self.kin[:] = data['kin']
        self.actions[:] = data['actions']
        self.rewards[:] = data['rewards']
        self.next_sensor[:] = data['next_sensor']
        self.next_kin[:] = data['next_kin']
        self.dones[:] = data['dones']
        self.ptr = int(data['ptr'][0])
        self.size = int(data['size'][0])
