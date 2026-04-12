"""Helpers for reasoning about the object path.

The path-blind controller does not use the path to construct actions, but the
training reward and evaluation code still need the path as hidden ground truth.
This module provides the small geometry helpers used for that hidden side:

- cumulative arc length ``s`` along the trajectory
- nearest sampled-path lookup
- local tangent / normal frame utilities for cross-track error
- interpolation of path position as a function of arc length or global x
"""

from __future__ import annotations

import numpy as np


# Add arc-length column to a (x,y) path.
def calc_path_data(path_xy: np.ndarray) -> np.ndarray:
    """Append cumulative arc length to a ``(x, y)`` path array."""
    ds = np.linalg.norm(np.diff(path_xy, axis=0), axis=1)
    s = np.pad(np.cumsum(ds), (1, 0))
    return np.concatenate((path_xy, s[:, None]), axis=1)


# Find closest sampled point by Euclidean distance.
def nearest_path_index(path_xy: np.ndarray, position_xy: np.ndarray) -> int:
    """Return the index of the nearest sampled path point."""
    d = np.linalg.norm(path_xy - position_xy[None, :], axis=1)
    return int(np.argmin(d))


# Compute local tangent and normal vectors.
def tangent_normal(path_xy: np.ndarray, idx: int) -> tuple[np.ndarray, np.ndarray]:
    """Compute a local tangent / normal pair around one sampled path point."""
    if idx <= 0:
        tangent = path_xy[1] - path_xy[0]
    elif idx >= len(path_xy) - 1:
        tangent = path_xy[-1] - path_xy[-2]
    else:
        tangent = path_xy[idx + 1] - path_xy[idx - 1]

    tangent_norm = np.linalg.norm(tangent)
    if tangent_norm < 1e-12:
        tangent = np.array([1.0, 0.0], dtype=np.float64)
    else:
        tangent = tangent / tangent_norm

    normal = np.array([-tangent[1], tangent[0]], dtype=np.float64)
    return tangent, normal


# Express a point in the local path frame (error + s).
def local_path_frame(path_data: np.ndarray, position_xy: np.ndarray) -> dict:
    """Describe the whisker-array position in the local path frame."""
    path_xy = path_data[:, :2]
    idx = nearest_path_index(path_xy, position_xy)
    point = path_xy[idx]
    tangent, normal = tangent_normal(path_xy, idx)
    offset = position_xy - point
    signed_lateral_error = float(np.dot(offset, normal))
    return {
        'index': idx,
        'point': point,
        'tangent': tangent,
        'normal': normal,
        's': float(path_data[idx, 2]),
        'signed_lateral_error': signed_lateral_error,
    }


# Interpolate path position at arc length s.
def path_point_at_s(path_data: np.ndarray, s_mm: float) -> np.ndarray:
    """Interpolate the path point ``[x, y]`` at a given arc-length position."""
    s = path_data[:, 2]
    x = np.interp(float(s_mm), s, path_data[:, 0], left=path_data[0, 0], right=path_data[-1, 0])
    y = np.interp(float(s_mm), s, path_data[:, 1], left=path_data[0, 1], right=path_data[-1, 1])
    return np.array([x, y], dtype=np.float64)


# Sort path samples for x-based interpolation.
def _path_sorted_by_x(path_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return path samples sorted by x with duplicate x-values removed."""
    order = np.argsort(path_xy[:, 0], kind='mergesort')
    xs = path_xy[order, 0]
    ys = path_xy[order, 1]
    unique_xs, unique_idx = np.unique(xs, return_index=True)
    unique_ys = ys[unique_idx]
    return unique_xs.astype(np.float64), unique_ys.astype(np.float64)


# Interpolate y as a function of global x.
def path_y_at_x(path_xy: np.ndarray, x_mm: float) -> float:
    """Interpolate the path's y-position at a given global x-position."""
    xs, ys = _path_sorted_by_x(path_xy)
    return float(np.interp(float(x_mm), xs, ys, left=ys[0], right=ys[-1]))
