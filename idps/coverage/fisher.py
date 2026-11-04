"""Vectorised visibility and triangulation-uncertainty maths.

Each camera that sees a point contributes Fisher information J^T J / var to
the point's 3x3 information matrix, where J is the 2x3 Jacobian of the
normalised image coordinates with respect to the point, and var the angular
variance from the noise budget. The predicted covariance is the inverse of
the summed information. Information matrices are stored packed as
(xx, xy, xz, yy, yz, zz) along the last axis.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from idps.core.camera import Sensor
from idps.core.noise import NoiseBudget


@dataclass(frozen=True)
class VisibilityLimits:
    max_range_m: float = 120.0     # marker detection range
    min_range_m: float = 0.5
    edge_margin: float = 0.05      # ignore the outer 5 % of the image (distortion, vignetting)


def contributions(pos: np.ndarray, R: np.ndarray, f_px: np.ndarray, points: np.ndarray,
                  sensor: Sensor, noise: NoiseBudget, limits: VisibilityLimits):
    """Visibility and packed information for C cameras and V points.

    pos (C, 3), R (C, 3, 3), f_px (C,), points (V, 3).
    Returns visible (C, V) bool and info (C, V, 6) float64 (zero where not visible).
    """
    pos = np.atleast_2d(pos)
    R = R.reshape(-1, 3, 3)
    f_px = np.broadcast_to(np.asarray(f_px, float), (len(pos),))
    d = points[None, :, :] - pos[:, None, :]                       # (C, V, 3)
    Xc = np.einsum("cij,cvj->cvi", R, d)                            # (C, V, 3)
    z = Xc[..., 2]
    rng = np.linalg.norm(d, axis=-1)
    zs = np.where(z > 1e-6, z, 1.0)
    u = f_px[:, None] * Xc[..., 0] / zs
    v = f_px[:, None] * Xc[..., 1] / zs
    half_w = sensor.width_px / 2 * (1 - limits.edge_margin)
    half_h = sensor.height_px / 2 * (1 - limits.edge_margin)
    visible = ((z > 1e-6) & (np.abs(u) <= half_w) & (np.abs(v) <= half_h)
               & (rng <= limits.max_range_m) & (rng >= limits.min_range_m))

    # Jacobian of (x/z, y/z) wrt camera-frame point, then rotate to world.
    inv_z = 1.0 / zs
    a = Xc[..., 0] * inv_z ** 2
    b = Xc[..., 1] * inv_z ** 2
    r0, r1, r2 = R[:, 0, :], R[:, 1, :], R[:, 2, :]                 # (C, 3)
    J0 = inv_z[..., None] * r0[:, None, :] - a[..., None] * r2[:, None, :]   # (C, V, 3)
    J1 = inv_z[..., None] * r1[:, None, :] - b[..., None] * r2[:, None, :]
    w = np.where(visible, 1.0 / noise.angular_var(f_px[:, None], rng), 0.0)
    info = np.empty(z.shape + (6,))
    for k, (i, j) in enumerate(((0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2))):
        info[..., k] = (J0[..., i] * J0[..., j] + J1[..., i] * J1[..., j]) * w
    return visible, info


def trace_inverse(info: np.ndarray) -> np.ndarray:
    """trace(inv(A)) for packed symmetric 3x3 matrices; inf where singular."""
    a, b, c, d, e, f = np.moveaxis(info, -1, 0)
    m_aa = d * f - e * e
    m_dd = a * f - c * c
    m_ff = a * d - b * b
    det = a * m_aa - b * (b * f - c * e) + c * (b * e - c * d)
    scale = (a + d + f) ** 3
    ok = det > 1e-12 * np.maximum(scale, 1e-300)
    with np.errstate(divide="ignore", invalid="ignore"):
        tr = np.where(ok, (m_aa + m_dd + m_ff) / np.where(ok, det, 1.0), np.inf)
    return tr


def rms_mm(info: np.ndarray) -> np.ndarray:
    """Predicted 3D RMS error in millimetres (sqrt of covariance trace)."""
    return np.sqrt(trace_inverse(info)) * 1000.0


def unpack(info6: np.ndarray) -> np.ndarray:
    a, b, c, d, e, f = np.moveaxis(info6, -1, 0)
    return np.stack([np.stack([a, b, c], -1), np.stack([b, d, e], -1), np.stack([c, e, f], -1)], -2)


def guaranteed_visible(pos: np.ndarray, R: np.ndarray, f_px: np.ndarray, points: np.ndarray, sensor: Sensor,
                       limits: VisibilityLimits, radius: float) -> np.ndarray:
    """Cameras that see the *whole ball* of `radius` around each point (C, V) bool.

    A camera's usable view is a pyramid (image edges minus the edge margin) cut by the
    detection range. The ball is inside it when its centre is at least `radius` from every
    side plane and from the near and far limits. If every evaluation point's ball is fully
    seen by N cameras, then every point in the space between the samples is seen by at
    least N cameras too, at any resolution: a guarantee, not a sample.
    """
    pos = np.atleast_2d(pos)
    R = R.reshape(-1, 3, 3)
    f_px = np.broadcast_to(np.asarray(f_px, float), (len(pos),))
    d = points[None, :, :] - pos[:, None, :]
    Xc = np.einsum("cij,cvj->cvi", R, d)
    x, y, z = Xc[..., 0], Xc[..., 1], Xc[..., 2]
    tw = (sensor.width_px / 2 * (1 - limits.edge_margin)) / f_px[:, None]     # tan of half-angles
    th = (sensor.height_px / 2 * (1 - limits.edge_margin)) / f_px[:, None]
    rng = np.linalg.norm(d, axis=-1)
    ok = (z * tw - np.abs(x)) / np.sqrt(1 + tw ** 2) >= radius
    ok &= (z * th - np.abs(y)) / np.sqrt(1 + th ** 2) >= radius
    ok &= (rng + radius <= limits.max_range_m) & (rng - radius >= limits.min_range_m) & (z > radius)
    return ok
