"""Outlier removal — Python port of MATLAB ``filloutliers(x, 'linear')``.

MATLAB's default detection is MAD-based (3 * 1.4826 * MAD ≈ 3-sigma assuming
normal distribution), per column. Outliers are replaced with linear
interpolation from neighboring non-outlier samples.

Used by ``main_window.py`` (the GUI subscriber) to clean signals before
plotting and recording — mirrors ``wavi.m:remove_outliers`` ([wavi.m:870-877](wavi/wavi.m#L870-L877)).
The DAQ thread does NOT call this; it's per-subscriber to keep raw samples on
the bus.
"""

from __future__ import annotations

import numpy as np


_MAD_SCALE = 1.4826   # so 1*MAD ≈ 1 standard deviation for normal data
_MAD_K = 3.0          # 3-sigma equivalent — MATLAB default


def fill_outliers_linear(x: np.ndarray) -> np.ndarray:
    """Per-column outlier replacement via linear interpolation.

    Input ``x`` is shape ``(N, C)``. Returns a new array of the same shape;
    the input is not modified.

    Columns with zero MAD (constant signal) are passed through unchanged.
    Columns with fewer than two non-outlier samples are also passed through.
    """
    if x.ndim != 2:
        raise ValueError(f"expected (N, C) array, got shape {x.shape}")

    out = np.array(x, copy=True)
    n = out.shape[0]
    if n < 3:
        return out

    indices = np.arange(n)
    for c in range(out.shape[1]):
        col = out[:, c]
        med = np.median(col)
        mad = np.median(np.abs(col - med))
        if mad <= 0:
            continue
        threshold = _MAD_K * _MAD_SCALE * mad
        bad = np.abs(col - med) > threshold
        if not bad.any():
            continue
        good = ~bad
        if good.sum() < 2:
            continue
        col[bad] = np.interp(indices[bad], indices[good], col[good])
    return out
