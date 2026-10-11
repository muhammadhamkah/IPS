"""Evaluation grid over the flight volume."""
from __future__ import annotations

import math

import numpy as np


def axis_samples(lo: float, hi: float, spacing: float, inset: float) -> np.ndarray:
    a, b = lo + inset, hi - inset
    if b <= a:
        return np.array([(lo + hi) / 2])
    n = max(2, math.ceil((b - a) / spacing) + 1)
    return np.linspace(a, b, n)


def volume_grid(vmin, vmax, spacing, inset=0.5, z_min_abs=0.2):
    """Voxel-centre grid that includes the boundary (walls, floor, ceiling).

    spacing may be a scalar or (sx, sy, sz). The bottom layer is lowered to
    z_min + z_min_abs (take-off height) so it is always evaluated, whatever
    the inset.
    Returns (points (V, 3), (xs, ys, zs)).
    """
    sx, sy, sz = (spacing,) * 3 if np.isscalar(spacing) else spacing
    xs = axis_samples(vmin[0], vmax[0], sx, inset)
    ys = axis_samples(vmin[1], vmax[1], sy, inset)
    zs = axis_samples(vmin[2], vmax[2], sz, inset)
    zs[0] = max(min(zs[0], vmin[2] + z_min_abs), vmin[2])
    X, Y, Z = np.meshgrid(xs, ys, zs, indexing="ij")
    return np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=-1), (xs, ys, zs)
