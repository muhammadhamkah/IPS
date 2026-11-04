"""Triangulation: the 3D point where several cameras' lines of sight (nearly) cross.

Each camera contributes a line from its centre along the direction of the marker. The
point minimises the sum of squared *angular* misses (perpendicular distance divided by
range), which is what the pixel noise is. Closed form, then one re-weighting with the ranges.

Moving markers: cameras expose at slightly different instants. With a velocity estimate
``v``, a ray taken ``dt`` seconds after the reference time is moved back by ``v * dt`` so
that every ray describes the marker at the same instant.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Point3D:
    X: np.ndarray               # (3,) position
    cov: np.ndarray             # (3, 3)
    resid_px: np.ndarray        # (k,) miss per camera, pixels
    used: np.ndarray            # (k,) bool, cameras kept after outlier rejection

    @property
    def n(self) -> int:
        return int(self.used.sum())


def _solve(o, d, w, shift):
    P = np.eye(3)[None] - d[:, :, None] * d[:, None, :]          # projectors onto each ray's normal plane
    A = np.einsum("k,kij->ij", w, P)
    b = np.einsum("k,kij,kj->i", w, P, o - shift)
    return np.linalg.solve(A, b), A


def triangulate(origins, dirs, f_px, sigma_ang: float, dt=None, vel=None, max_resid_px: float = 3.0,
                min_cams: int = 2, X0=None) -> Point3D | None:
    """origins/dirs (k, 3) per camera, f_px (k,) focal lengths (for pixel residuals),
    sigma_ang one-ray angular noise (rad). Drops the worst camera while any misses by more
    than max_resid_px and enough cameras remain. None if fewer than min_cams or degenerate.
    X0: a good guess (a track's prediction): the range weights come from it, one solve."""
    o = np.asarray(origins, float)
    d = np.asarray(dirs, float)
    f = np.broadcast_to(np.asarray(f_px, float), (len(o),))
    k = len(o)
    shift = np.zeros((k, 3)) if dt is None or vel is None else np.outer(dt, vel)
    used = np.ones(k, bool)
    while used.sum() >= min_cams:
        idx = np.nonzero(used)[0]
        try:
            if X0 is None:
                X, _ = _solve(o[idx], d[idx], np.ones(len(idx)), shift[idx])
                rounds = 2
            else:
                X, rounds = np.asarray(X0, float), 1
            for _ in range(rounds):
                r = np.linalg.norm(X + shift[idx] - o[idx], axis=1)
                X, A = _solve(o[idx], d[idx], 1.0 / np.maximum(r, 0.1) ** 2, shift[idx])
        except np.linalg.LinAlgError:
            return None
        v = X + shift - o
        rng = np.linalg.norm(v, axis=1)
        cosang = np.clip(np.einsum("ij,ij->i", v, d) / np.maximum(rng, 1e-9), -1, 1)
        resid = np.arccos(cosang) * f                          # pixels
        resid[cosang <= 0] = np.inf                            # behind the camera
        worst = idx[np.argmax(resid[idx])]
        if resid[worst] > max_resid_px and used.sum() > min_cams:
            used[worst] = False
            continue
        if resid[worst] > max_resid_px:
            return None
        if X0 is None and np.linalg.cond(A) > 1e8:          # near-parallel rays (new tracks only)
            return None
        return Point3D(X, sigma_ang ** 2 * np.linalg.inv(A), resid, used)
    return None


def closest_approach(o1, d1, o2, d2):
    """Midpoint and gap of the closest approach of two lines, and the distances along each
    (negative = behind). Vectorised over leading axes."""
    w0 = o1 - o2
    b = np.sum(d1 * d2, axis=-1)
    dd = np.sum(d1 * w0, axis=-1)
    e = np.sum(d2 * w0, axis=-1)
    den = np.maximum(1 - b * b, 1e-12)
    s = (b * e - dd) / den
    t = (e - b * dd) / den
    p1 = o1 + s[..., None] * d1
    p2 = o2 + t[..., None] * d2
    return (p1 + p2) / 2, np.linalg.norm(p1 - p2, axis=-1), s, t


def triangulate_many(tix, O, D, F, X0, shift, n: int, sigma_ang: float, max_resid_px: float = 3.0,
                     min_cams: int = 2, max_drops: int = 3):
    """Triangulate many markers at once, each seeded with a prediction (tracked markers).

    tix (N,) marker index of each observation; O, D (N, 3) camera centres and rays; F (N,)
    focal lengths; X0 (n, 3) predictions; shift (N, 3) motion correction (velocity * dt).
    Like ``triangulate`` with X0: range weights from X0, one solve, and the worst camera of a
    marker is dropped while it misses by more than max_resid_px (up to max_drops times).
    Returns X (n, 3), cov (n, 3, 3), ok (n,), keep (N,) and the number of cameras used (n,).
    """
    N = len(tix)
    keep = np.ones(N, bool)
    Pm = np.eye(3)[None] - D[:, :, None] * D[:, None, :]
    Os = O - shift
    w = 1.0 / np.maximum(np.linalg.norm(X0[tix] - Os, axis=1), 0.1) ** 2
    X = X0.copy()
    A = np.zeros((n, 3, 3))
    ok = np.zeros(n, bool)
    for _ in range(max_drops + 1):
        A[:] = 0
        b = np.zeros((n, 3))
        wk = np.where(keep, w, 0.0)
        np.add.at(A, tix, wk[:, None, None] * Pm)
        np.add.at(b, tix, wk[:, None] * np.einsum("nij,nj->ni", Pm, Os))
        cnt = np.bincount(tix, keep.astype(int), minlength=n)
        ok = cnt >= min_cams
        if not ok.any():
            break
        X[ok] = np.linalg.solve(A[ok], b[ok][..., None])[..., 0]
        v = X[tix] - Os
        cosang = np.clip(np.einsum("ij,ij->i", v, D) / np.maximum(np.linalg.norm(v, axis=1), 1e-9), -1, 1)
        resid = np.where(cosang > 0, np.arccos(cosang) * F, np.inf)
        resid = np.where(keep, resid, -1.0)
        worst = np.full(n, -1.0)
        np.maximum.at(worst, tix, resid)
        bad = ok & (worst > max_resid_px)
        if not bad.any():
            break
        drop_ok = bad & (cnt > min_cams)
        if not drop_ok.any():
            ok &= ~bad
            break
        # drop each such marker's worst observation
        is_worst = keep & drop_ok[tix] & (resid >= worst[tix])
        first = np.zeros(n, bool)
        for i in np.nonzero(is_worst)[0]:          # one per marker
            if not first[tix[i]]:
                first[tix[i]] = True
                keep[i] = False
        ok &= ~(bad & ~drop_ok)
    cov = np.zeros((n, 3, 3))
    if ok.any():
        cov[ok] = sigma_ang ** 2 * np.linalg.inv(A[ok])
    ncam = np.bincount(tix, keep.astype(int), minlength=n)
    return X, cov, ok, keep, ncam
