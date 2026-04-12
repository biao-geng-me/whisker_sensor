from __future__ import annotations

from pathlib import Path

import torch
import torch.nn as nn


class TemporalSpatialEncoder(nn.Module):
    """Actor-side encoder copied from rl_sac_v1 for standalone deployment."""

    def __init__(self, hidden_dim: int = 64):
        super().__init__()
        # Per-frame spatial encoder for one 3x3x2 whisker snapshot.
        self.frame_encoder = nn.Sequential(
            nn.Conv2d(2, 16, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.Conv2d(16, 16, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.Flatten(),
            nn.Linear(16 * 3 * 3, hidden_dim),
            nn.ReLU(),
        )
        # Temporal model over the 16-frame history window.
        self.gru = nn.GRU(hidden_dim, hidden_dim, batch_first=True)

    def forward(self, sensor_history: torch.Tensor) -> torch.Tensor:
        """Encode a batch of sensor histories with shape [B, T, 3, 3, 2]."""
        bsz, steps = sensor_history.shape[:2]
        # Convert to [B*T, C, H, W] so the CNN sees each frame independently.
        x = sensor_history.permute(0, 1, 4, 2, 3).reshape(bsz * steps, 2, 3, 3)
        x = self.frame_encoder(x)
        # Restore the time dimension for the GRU.
        x = x.reshape(bsz, steps, -1)
        out, _ = self.gru(x)
        # Use the final GRU output as the summary of the recent 200 ms history.
        return out[:, -1, :]


class Actor(nn.Module):
    """Standalone actor network for deterministic hardware inference.

    The training-time SAC actor had both ``mean`` and ``log_std`` heads.
    We keep the same parameter structure here so the deployment package can
    load the original actor weights directly, but deterministic inference only
    uses the squashed mean action.
    """

    def __init__(self, kin_dim: int = 6, hidden_dim: int = 64, action_dim: int = 1):
        super().__init__()
        self.encoder = TemporalSpatialEncoder(hidden_dim=hidden_dim)
        self.policy = nn.Sequential(
            nn.Linear(hidden_dim + kin_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
        )
        self.mean = nn.Linear(hidden_dim, action_dim)
        self.log_std = nn.Linear(hidden_dim, action_dim)

    def forward(self, sensor_history: torch.Tensor, kin: torch.Tensor) -> torch.Tensor:
        """Return the deterministic tanh-squashed mean action in [-1, 1]."""
        z = self.encoder(sensor_history)
        z = torch.cat([z, kin], dim=-1)
        h = self.policy(z)
        return torch.tanh(self.mean(h))


def load_actor(weights_path: str | Path, device: str = 'cpu', kin_dim: int = 6, action_dim: int = 1) -> Actor:
    """Load either an actor-only state dict or a full SAC checkpoint."""
    device_t = torch.device(device)
    payload = torch.load(weights_path, map_location=device_t)
    # Support both:
    # 1. the smaller actor-only export used for deployment
    # 2. a full SAC checkpoint that contains an 'actor' entry
    state_dict = payload['actor'] if isinstance(payload, dict) and 'actor' in payload else payload
    actor = Actor(kin_dim=kin_dim, action_dim=action_dim).to(device_t)
    actor.load_state_dict(state_dict)
    actor.eval()
    return actor
