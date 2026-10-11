"""Flight volume (floor polygon extruded in Z) and its evaluation grid.

The volume is drawn as XY vertices (any simple polygon, convex or not) and
extruded from z_min to z_max. Evaluation points are a regular grid inside the
polygon plus a ring of points just inside every edge and vertex, so walls and
corners are always evaluated whatever the grid spacing.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def _close(poly: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return poly, np.roll(poly, -1, axis=0)


def points_in_polygon(xy: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Even-odd rule, vectorised over points. Points exactly on an edge may go either way."""
    xy = np.atleast_2d(xy)
    x, y = xy[:, 0:1], xy[:, 1:2]
    a, b = _close(poly)
    xa, ya, xb, yb = a[:, 0], a[:, 1], b[:, 0], b[:, 1]
    crosses = (ya > y) != (yb > y)
    with np.errstate(divide="ignore", invalid="ignore"):
        x_int = xa + (y - ya) * (xb - xa) / (yb - ya)
    return np.sum(crosses & (x < x_int), axis=1) % 2 == 1


def distance_to_polygon(xy: np.ndarray, poly: np.ndarray):
    """Distance from each point to the polygon boundary, plus the nearest
    edge index and the distance along that edge from its first vertex."""
    xy = np.atleast_2d(xy)
    a, b = _close(poly)
    ab = b - a                                                     # (E, 2)
    L2 = np.maximum((ab ** 2).sum(1), 1e-12)
    ap = xy[:, None, :] - a[None]                                  # (P, E, 2)
    t = np.clip((ap * ab[None]).sum(-1) / L2[None], 0, 1)
    d = np.linalg.norm(ap - t[..., None] * ab[None], axis=-1)      # (P, E)
    e = d.argmin(1)
    rows = np.arange(len(xy))
    return d[rows, e], e, t[rows, e] * np.sqrt(L2[e])


def signed_area(poly: np.ndarray) -> float:
    a, b = _close(poly)
    return 0.5 * float(np.sum(a[:, 0] * b[:, 1] - b[:, 0] * a[:, 1]))


def outward_normals(poly: np.ndarray) -> np.ndarray:
    a, b = _close(poly)
    ab = b - a
    n = np.stack([ab[:, 1], -ab[:, 0]], 1) / np.linalg.norm(ab, axis=1)[:, None]
    return n if signed_area(poly) > 0 else -n      # CCW polygon: (dy, -dx) points outward


def offset_ring(poly: np.ndarray, offset: float, spacing: float) -> np.ndarray:
    """Points along every edge, shifted by `offset` along the outward normal
    (negative = inward), at most `spacing` apart, plus offset vertex points."""
    a, b = _close(poly)
    n = outward_normals(poly)
    out = []
    for k in range(len(poly)):
        L = float(np.linalg.norm(b[k] - a[k]))
        m = max(1, math.ceil(L / spacing - 1e-9))
        for t in (np.arange(m) + 0.5) / m:
            out.append(a[k] + t * (b[k] - a[k]) + offset * n[k])
        # vertex: offset along the bisector of the two adjacent normals
        nb = n[k] + n[k - 1]
        nn = np.linalg.norm(nb)
        if nn > 1e-9:
            cos_half = max(0.3, float(np.dot(nb / nn, n[k])))
            out.append(a[k] + offset * (nb / nn) / cos_half)
    return np.array(out)


@dataclass
class Volume:
    polygon: np.ndarray       # (K, 2) floor vertices, metres
    z_min: float
    z_max: float

    def __post_init__(self):
        self.polygon = np.asarray(self.polygon, float)
        if len(self.polygon) < 3:
            raise ValueError("volume polygon needs at least 3 vertices")
        if abs(signed_area(self.polygon)) < 1e-6:
            raise ValueError("volume polygon has zero area")
        if self.z_max <= self.z_min:
            raise ValueError("volume z_max must be above z_min")

    @classmethod
    def box(cls, vmin, vmax) -> "Volume":
        (x0, y0, z0), (x1, y1, z1) = vmin, vmax
        return cls(np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], float), float(z0), float(z1))

    @property
    def bbox_min(self) -> np.ndarray:
        return np.array([*self.polygon.min(0), self.z_min])

    @property
    def bbox_max(self) -> np.ndarray:
        return np.array([*self.polygon.max(0), self.z_max])

    @property
    def floor_area_m2(self) -> float:
        return abs(signed_area(self.polygon))

    @property
    def volume_m3(self) -> float:
        return self.floor_area_m2 * (self.z_max - self.z_min)

    def contains_xy(self, xy) -> np.ndarray:
        return points_in_polygon(np.asarray(xy, float), self.polygon)

    def edge_lengths(self) -> np.ndarray:
        a, b = _close(self.polygon)
        return np.linalg.norm(b - a, axis=1)


def axis_samples(lo: float, hi: float, spacing: float, inset: float) -> np.ndarray:
    a, b = lo + inset, hi - inset
    if b <= a:
        return np.array([(lo + hi) / 2])
    n = max(2, math.ceil((b - a) / spacing) + 1)
    return np.linspace(a, b, n)


@dataclass
class EvalGrid:
    points: np.ndarray        # (V, 3): grid points first, then boundary-ring points
    xs: np.ndarray            # grid axes over the bounding box
    ys: np.ndarray
    zs: np.ndarray
    inside: np.ndarray        # (nx, ny) bool: bbox grid cells inside the polygon
    n_grid: int               # number of grid points (the rest are ring points)
    ring_xy: np.ndarray       # (R, 2)

    def level_of(self) -> np.ndarray:
        """Index into zs for every point."""
        return np.searchsorted(self.zs, self.points[:, 2])


def volume_grid(volume: Volume, spacing, inset: float = 0.5, z_min_abs: float = 0.2) -> EvalGrid:
    """spacing: scalar or (sx, sy, sz). The bottom layer is lowered to
    z_min + z_min_abs (take-off height) so it is always evaluated."""
    sx, sy, sz = (spacing,) * 3 if np.isscalar(spacing) else spacing
    lo, hi = volume.bbox_min, volume.bbox_max
    xs = axis_samples(lo[0], hi[0], sx, inset)
    ys = axis_samples(lo[1], hi[1], sy, inset)
    zs = axis_samples(lo[2], hi[2], sz, inset)
    zs[0] = max(min(zs[0], lo[2] + z_min_abs), lo[2])
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    xy = np.stack([X.ravel(), Y.ravel()], 1)
    d, _, _ = distance_to_polygon(xy, volume.polygon)
    inside = (volume.contains_xy(xy) & (d >= inset * 0.999)).reshape(X.shape)
    grid_xy = xy[inside.ravel()]
    ring_xy = offset_ring(volume.polygon, -inset, min(sx, sy))
    ring_xy = ring_xy[volume.contains_xy(ring_xy)]
    pts = [np.column_stack([grid_xy, np.full(len(grid_xy), z)]) for z in zs]
    pts += [np.column_stack([ring_xy, np.full(len(ring_xy), z)]) for z in zs]
    n_grid = len(grid_xy) * len(zs)
    return EvalGrid(np.concatenate(pts), xs, ys, zs, inside, n_grid, ring_xy)
