"""Neural-network components for the path-blind SAC baseline.

The architecture here is intentionally modest:
- encode each 3x3 whisker frame with a small convolutional network
- summarize the frame sequence with a GRU
- fuse that sensory summary with low-dimensional kinematics
- use standard SAC actor / twin-critic heads

This is designed to preserve both spatial structure and short temporal history
without making the model so large that debugging becomes difficult.
"""

from __future__ import annotations

import copy
import math

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions.normal import Normal


LOG_STD_MIN = -5.0
LOG_STD_MAX = 2.0


# Encode 16-step whisker history into one latent vector.
class TemporalSpatialEncoder(nn.Module):
    """Encode whisker history into one compact latent vector."""

    def __init__(self, hidden_dim: int = 64):
        super().__init__()

        # The whisker array is tiny (3x3), so a small convolutional stack is
        # enough to capture local spatial patterns before the GRU models time.
        self.frame_encoder = nn.Sequential(
            nn.Conv2d(2, 16, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.Conv2d(16, 16, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.Flatten(),
            nn.Linear(16 * 3 * 3, hidden_dim),
            nn.ReLU(),
        )
        self.gru = nn.GRU(hidden_dim, hidden_dim, batch_first=True)

    def forward(self, sensor_history: torch.Tensor) -> torch.Tensor:
        """Encode ``[B, T, 3, 3, 2]`` sensor history into ``[B, H]``.

        ``B`` is batch size and ``T`` is the number of stored history frames.
        The last dimension is the two whisker signal channels used here.
        """
        bsz, steps = sensor_history.shape[:2]

        # Move channels to the position expected by Conv2d and merge batch/time so
        # every frame can be encoded with the same CNN.
        x = sensor_history.permute(0, 1, 4, 2, 3).reshape(bsz * steps, 2, 3, 3)
        x = self.frame_encoder(x)
        x = x.reshape(bsz, steps, -1)

        out, _ = self.gru(x)
        return out[:, -1, :]


# Actor outputs a Gaussian policy; actions are tanh-squashed.
class Actor(nn.Module):
    """Gaussian actor used by SAC."""

    def __init__(self, kin_dim: int, hidden_dim: int = 64, action_dim: int = 1):
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

    def forward(self, sensor_history: torch.Tensor, kin: torch.Tensor):
        """Return Gaussian policy parameters before action sampling.

        Inputs:
        - ``sensor_history``: ``[B, T, 3, 3, 2]``
        - ``kin``: ``[B, kin_dim]``

        Outputs:
        - ``mean``: ``[B, action_dim]``
        - ``log_std``: ``[B, action_dim]``
        """
        z = self.encoder(sensor_history)
        z = torch.cat([z, kin], dim=-1)
        h = self.policy(z)
        mean = self.mean(h)
        log_std = self.log_std(h).clamp(LOG_STD_MIN, LOG_STD_MAX)
        return mean, log_std

    def sample(self, sensor_history: torch.Tensor, kin: torch.Tensor):
        """Sample a tanh-squashed action and its log probability.

        Returns
        -------
        tuple
            ``(sampled_action, log_prob, mean_action)`` with each action shaped
            ``[B, action_dim]`` and ``log_prob`` shaped ``[B, 1]``.
        """
        mean, log_std = self(sensor_history, kin)
        std = log_std.exp()
        normal = Normal(mean, std)

        # Reparameterization trick for SAC.
        x_t = normal.rsample()
        y_t = torch.tanh(x_t)
        log_prob = normal.log_prob(x_t) - torch.log(1 - y_t.pow(2) + 1e-6)
        log_prob = log_prob.sum(dim=-1, keepdim=True)
        mean_action = torch.tanh(mean)
        return y_t, log_prob, mean_action


# Critic estimates Q(s, a) for one of the twin heads.
class Critic(nn.Module):
    """Single Q-function used inside the twin-critic setup."""

    def __init__(self, kin_dim: int, hidden_dim: int = 64, action_dim: int = 1):
        super().__init__()
        self.encoder = TemporalSpatialEncoder(hidden_dim=hidden_dim)
        self.q = nn.Sequential(
            nn.Linear(hidden_dim + kin_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1),
        )

    def forward(self, sensor_history: torch.Tensor, kin: torch.Tensor, action: torch.Tensor):
        """Evaluate Q(s, a).

        Inputs:
        - ``sensor_history``: ``[B, T, 3, 3, 2]``
        - ``kin``: ``[B, kin_dim]``
        - ``action``: ``[B, action_dim]``
        """
        z = self.encoder(sensor_history)
        z = torch.cat([z, kin, action], dim=-1)
        return self.q(z)


# Full SAC wrapper (actor, twin critics, target critics, optimizers).
class SACAgent:
    """Minimal Soft Actor-Critic implementation for the path-blind baseline."""

    def __init__(self, kin_dim: int, action_dim: int, cfg):
        self.device = torch.device(cfg.device)

        self.actor = Actor(kin_dim=kin_dim, action_dim=action_dim).to(self.device)
        self.q1 = Critic(kin_dim=kin_dim, action_dim=action_dim).to(self.device)
        self.q2 = Critic(kin_dim=kin_dim, action_dim=action_dim).to(self.device)
        self.q1_target = copy.deepcopy(self.q1).to(self.device)
        self.q2_target = copy.deepcopy(self.q2).to(self.device)

        self.actor_optim = torch.optim.Adam(self.actor.parameters(), lr=cfg.actor_lr)
        self.q1_optim = torch.optim.Adam(self.q1.parameters(), lr=cfg.critic_lr)
        self.q2_optim = torch.optim.Adam(self.q2.parameters(), lr=cfg.critic_lr)

        self.log_alpha = torch.tensor(
            math.log(cfg.init_temperature),
            device=self.device,
            requires_grad=True,
        )
        self.alpha_optim = torch.optim.Adam([self.log_alpha], lr=cfg.alpha_lr)
        self.target_entropy = -float(action_dim)

        self.gamma = cfg.gamma
        self.tau = cfg.tau

    @property
    def alpha(self):
        """Current entropy temperature."""
        return self.log_alpha.exp()

    @torch.no_grad()
    def act(self, sensor_history: np.ndarray, kin: np.ndarray, deterministic: bool = False) -> np.ndarray:
        """Compute one action batch from NumPy observations.

        Parameters are NumPy arrays with batch dimension:
        - ``sensor_history``: ``[B, T, 3, 3, 2]``
        - ``kin``: ``[B, kin_dim]``

        Returns
        -------
        np.ndarray
            Action batch shaped ``[B, action_dim]``.
        """
        sensor = torch.as_tensor(sensor_history, device=self.device, dtype=torch.float32)
        kin_t = torch.as_tensor(kin, device=self.device, dtype=torch.float32)
        was_training = self.actor.training
        self.actor.eval()
        try:
            if deterministic:
                _, _, action = self.actor.sample(sensor, kin_t)
            else:
                action, _, _ = self.actor.sample(sensor, kin_t)
            return action.cpu().numpy()
        finally:
            if was_training:
                self.actor.train()

    def update(self, batch: dict) -> dict:
        """Run one SAC update step and return logging scalars.

        ``batch`` is the dictionary produced by :class:`ReplayBuffer.sample`.
        Every tensor inside it already carries a leading mini-batch dimension.
        """
        self.actor.train()
        self.q1.train()
        self.q2.train()
        self.q1_target.train()
        self.q2_target.train()

        sensor = batch['sensor']
        kin = batch['kin']
        action = batch['action']
        reward = batch['reward']
        next_sensor = batch['next_sensor']
        next_kin = batch['next_kin']
        done = batch['done']

        with torch.no_grad():
            # Target Q for SAC uses entropy-regularized target value.
            next_action, next_log_prob, _ = self.actor.sample(next_sensor, next_kin)
            target_q1 = self.q1_target(next_sensor, next_kin, next_action)
            target_q2 = self.q2_target(next_sensor, next_kin, next_action)
            target_q = torch.min(target_q1, target_q2) - self.alpha.detach() * next_log_prob
            q_target = reward + (1.0 - done) * self.gamma * target_q

        q1_pred = self.q1(sensor, kin, action)
        q2_pred = self.q2(sensor, kin, action)
        q1_loss = F.mse_loss(q1_pred, q_target)
        q2_loss = F.mse_loss(q2_pred, q_target)

        self.q1_optim.zero_grad()
        q1_loss.backward()
        self.q1_optim.step()

        self.q2_optim.zero_grad()
        q2_loss.backward()
        self.q2_optim.step()

        new_action, log_prob, _ = self.actor.sample(sensor, kin)
        q_new = torch.min(
            self.q1(sensor, kin, new_action),
            self.q2(sensor, kin, new_action),
        )
        actor_loss = (self.alpha.detach() * log_prob - q_new).mean()

        self.actor_optim.zero_grad()
        actor_loss.backward()
        self.actor_optim.step()

        alpha_loss = -(self.log_alpha * (log_prob + self.target_entropy).detach()).mean()
        self.alpha_optim.zero_grad()
        alpha_loss.backward()
        self.alpha_optim.step()

        self._soft_update(self.q1, self.q1_target)
        self._soft_update(self.q2, self.q2_target)

        return {
            'q1_loss': float(q1_loss.item()),
            'q2_loss': float(q2_loss.item()),
            'actor_loss': float(actor_loss.item()),
            'alpha_loss': float(alpha_loss.item()),
            'alpha': float(self.alpha.item()),
        }

    def _soft_update(self, source: nn.Module, target: nn.Module):
        """Polyak-average target-network parameters."""
        for p, tp in zip(source.parameters(), target.parameters()):
            tp.data.mul_(1.0 - self.tau)
            tp.data.add_(self.tau * p.data)

    def save(self, path: str):
        """Save enough state to evaluate or resume the policy later."""
        torch.save({
            'actor': self.actor.state_dict(),
            'q1': self.q1.state_dict(),
            'q2': self.q2.state_dict(),
            'q1_target': self.q1_target.state_dict(),
            'q2_target': self.q2_target.state_dict(),
            'log_alpha': self.log_alpha.detach().cpu(),
        }, path)

    def load(self, path: str):
        """Load a checkpoint created by :meth:`save`."""
        checkpoint = torch.load(path, map_location=self.device)
        self.actor.load_state_dict(checkpoint['actor'])
        self.q1.load_state_dict(checkpoint['q1'])
        self.q2.load_state_dict(checkpoint['q2'])
        self.q1_target.load_state_dict(checkpoint['q1_target'])
        self.q2_target.load_state_dict(checkpoint['q2_target'])
        self.log_alpha.data.copy_(checkpoint['log_alpha'].to(self.device))
