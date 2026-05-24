"""Streaming multi-channel IIR/FIR filter — port of ``wavi/SigFilter.m``.

State (``zi``) persists across ``apply()`` calls so high-order filters get
correct history from the moment the stream starts. Designed with
``scipy.signal.butter`` and applied with ``scipy.signal.lfilter`` along axis 0.

Used as a utility by subscribers (GUI applies, recording subscribers can
choose to). The DAQ thread does NOT filter — the bus carries raw samples.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.signal import butter, lfilter, lfilter_zi


@dataclass
class SigFilterConfig:
    filter_type: str = "lowpass-iir"   # 'lowpass-iir', 'highpass-iir', 'moving-average'
    fs: float = 80.0
    order: int = 2
    cutoff_hz: float = 10.0
    n_channels: int = 18
    gain: float = 1.0


class SigFilter:
    def __init__(self, cfg: Optional[SigFilterConfig] = None, **overrides):
        self.cfg = cfg or SigFilterConfig()
        for k, v in overrides.items():
            setattr(self.cfg, k, v)
        self._design()
        self._init_state()

    # ----- public API

    def apply(self, x: np.ndarray) -> np.ndarray:
        """Filter (N, C) input with persistent state. Returns (N, C) same dtype."""
        if x.ndim != 2:
            raise ValueError(f"expected (N, C) array, got shape {x.shape}")
        if x.shape[1] != self.cfg.n_channels:
            raise ValueError(
                f"channel mismatch: cfg.n_channels={self.cfg.n_channels}, "
                f"input has {x.shape[1]}"
            )
        if x.shape[0] == 0:
            return x

        y, self._zi = lfilter(self._b, self._a, x, axis=0, zi=self._zi)
        if self.cfg.gain != 1.0:
            y = y * self.cfg.gain
        return y.astype(x.dtype, copy=False)

    def reset(self) -> None:
        """Re-initialize filter state to zero. Useful on stream restart."""
        self._init_state()

    def configure(self, **kwargs) -> None:
        """Update config fields in-place, redesign, and reset state."""
        for k, v in kwargs.items():
            if hasattr(self.cfg, k):
                setattr(self.cfg, k, v)
            else:
                raise ValueError(f"unknown SigFilterConfig field: {k}")
        self._design()
        self._init_state()

    # ----- internals

    def _design(self) -> None:
        cfg = self.cfg
        nyq = cfg.fs / 2.0
        ft = cfg.filter_type
        if ft == "lowpass-iir":
            self._b, self._a = butter(cfg.order, cfg.cutoff_hz / nyq, btype="low")
        elif ft == "highpass-iir":
            self._b, self._a = butter(cfg.order, cfg.cutoff_hz / nyq, btype="high")
        elif ft == "moving-average":
            n = max(1, cfg.order)
            self._b = np.ones(n, dtype=np.float64) / n
            self._a = np.array([1.0])
        else:
            raise ValueError(f"unsupported filter_type: {ft!r}")

    def _init_state(self) -> None:
        # lfilter_zi gives the steady-state for unit step; we want zero state
        # so the filter starts at zero and warms up with real samples.
        zi_single = lfilter_zi(self._b, self._a)
        n_state = zi_single.shape[0]
        # zi shape for axis=0 multi-channel filtering: (n_state, n_channels)
        self._zi = np.zeros((n_state, self.cfg.n_channels), dtype=np.float64)
