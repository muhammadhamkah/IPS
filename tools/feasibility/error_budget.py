#!/usr/bin/env python3
"""First-order positioning error budget for IDPS camera layouts.

Not production code: a feasibility tool used to size the architecture
(docs/RISKS.md, docs/ARCHITECTURE.md). It propagates per-camera angular noise
through linearised multi-view triangulation (Fisher information) and reports
the expected 3D error over a grid of points in the flight volume.

Per-camera angular noise (radians) is the quadrature sum of:
  * centroid noise           sigma_px / f_px
  * extrinsic rotation error sigma_rot
  * extrinsic position error sigma_pos / range
  * timing error             v * sigma_t / range (worst case: motion across the ray)

Calibration and timing errors are *systematic* per camera, so they are not
averaged away over time; treating them as independent per camera is the
optimistic assumption made here.

Usage: python3 tools/feasibility/error_budget.py
"""
from __future__ import annotations

import itertools
from dataclasses import dataclass

import numpy as np

PIXEL_PITCH_M = 3.0e-6          # OV9281
SENSOR_PX = (1280, 800)


@dataclass
class Camera:
    pos: np.ndarray             # (3,) world position, metres
    R: np.ndarray               # (3,3) world->camera rotation, camera looks along +z
    f_px: float


def look_at(pos, target, up=(0.0, 0.0, 1.0)) -> np.ndarray:
    z = np.asarray(target, float) - pos
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    if np.linalg.norm(x) < 1e-9:
        x = np.cross(z, (0.0, 1.0, 0.0))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z])


@dataclass
class Noise:
    sigma_px: float = 0.1       # centroid noise, pixels (1-sigma)
    sigma_rot: float = 1e-4     # extrinsic rotation, rad (1-sigma)
    sigma_pos: float = 2e-3     # extrinsic position, m (1-sigma)
    sigma_t: float = 1e-4       # timing, s (1-sigma)
    speed: float = 4.0          # marker speed, m/s


def visible(cam: Camera, X: np.ndarray, max_range: float):
    """Boolean mask of points inside the image and within range."""
    Xc = (X - cam.pos) @ cam.R.T
    z = Xc[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        u = cam.f_px * Xc[:, 0] / z
        v = cam.f_px * Xc[:, 1] / z
    w, h = SENSOR_PX
    rng = np.linalg.norm(X - cam.pos, axis=1)
    return (z > 0.1) & (np.abs(u) < w / 2) & (np.abs(v) < h / 2) & (rng < max_range)


def point_cov(cams: list[Camera], X: np.ndarray, noise: Noise):
    """3x3 covariance of the triangulated point X from cameras that see it."""
    info = np.zeros((3, 3))
    for cam in cams:
        d = X - cam.pos
        rng = np.linalg.norm(d)
        Xc = cam.R @ d
        # Jacobian of normalised image coords (x/z, y/z) wrt X, in radians/m.
        J_c = np.array([[1 / Xc[2], 0, -Xc[0] / Xc[2] ** 2],
                        [0, 1 / Xc[2], -Xc[1] / Xc[2] ** 2]])
        J = J_c @ cam.R
        sig_ang2 = ((noise.sigma_px / cam.f_px) ** 2 + noise.sigma_rot ** 2
                    + (noise.sigma_pos / rng) ** 2
                    + (noise.speed * noise.sigma_t / rng) ** 2)
        info += J.T @ J / sig_ang2
    return np.linalg.inv(info)


def evaluate(name, cams, grid, noise, max_range, min_cams=2):
    n_vis = np.zeros(len(grid), int)
    masks = [visible(c, grid, max_range) for c in cams]
    for m in masks:
        n_vis += m
    err = np.full(len(grid), np.nan)
    for i, X in enumerate(grid):
        seen = [c for c, m in zip(cams, masks) if m[i]]
        if len(seen) >= min_cams:
            C = point_cov(seen, X, noise)
            err[i] = np.sqrt(np.trace(C))          # 3D RMS error, metres
    ok = ~np.isnan(err)
    cov_frac = ok.mean()
    if ok.any():
        e = err[ok] * 1000
        stats = f"3D RMS err mm: median {np.median(e):6.1f}  p95 {np.percentile(e, 95):6.1f}  max {e.max():6.1f}"
    else:
        stats = "no coverage"
    print(f"  {name:<44s} cover>={min_cams}: {cov_frac*100:5.1f}%  "
          f"median cams {np.median(n_vis):4.1f}  {stats}")


def prototype_rig(f_mm=3.6):
    """3-4 cameras around a 5 x 5 x 3 m lab volume."""
    f_px = f_mm * 1e-3 / PIXEL_PITCH_M
    target = np.array([0.0, 0.0, 1.0])
    corners = [(-3.5, -3.5), (3.5, -3.5), (3.5, 3.5), (-3.5, 3.5)]
    return [Camera(np.array([x, y, 2.8]), look_at(np.array([x, y, 2.8]), target), f_px)
            for x, y in corners], f_px


def hall_rig(spacing=25.0, heights=(10.0, 28.0), f_mm=4.0, aim_inward=30.0):
    """Cameras on the perimeter of a 100 x 100 m hall, aimed inward and down."""
    f_px = f_mm * 1e-3 / PIXEL_PITCH_M
    cams = []
    L = 100.0
    pts = []
    for s in np.arange(0, L, spacing):
        pts += [(s, 0.0), (L, s), (L - s, L), (0.0, L - s)]
    for (x, y), h in itertools.product(pts, heights):
        pos = np.array([x - L / 2, y - L / 2, h])
        inward = -pos[:2] / (np.linalg.norm(pos[:2]) + 1e-9)
        target = np.array([*(pos[:2] + inward * aim_inward), 8.0])
        cams.append(Camera(pos, look_at(pos, target), f_px))
    return cams, f_px


def ceiling_rig(spacing=20.0, height=30.0, f_mm=2.8, tilt_m=12.0):
    """Interior ceiling grid; neighbouring cameras tilted in alternating
    directions so every point is seen from several converging angles."""
    f_px = f_mm * 1e-3 / PIXEL_PITCH_M
    cams = []
    offs = [(tilt_m, 0), (0, tilt_m), (-tilt_m, 0), (0, -tilt_m)]
    coords = np.arange(-50 + spacing / 2, 50, spacing)
    for k, (x, y) in enumerate(itertools.product(coords, coords)):
        pos = np.array([x, y, height])
        dx, dy = offs[k % 4]
        cams.append(Camera(pos, look_at(pos, np.array([x + dx, y + dy, 0.0]), up=(1.0, 0.3, 0.0)), f_px))
    return cams, f_px


def grid(xs, ys, zs):
    return np.array(list(itertools.product(xs, ys, zs)), float)


def main():
    np.set_printoptions(precision=3)
    print("== Prototype lab: 5 x 5 x 3 m, 4 cameras, 3.6 mm lens ==")
    cams, f_px = prototype_rig()
    g = grid(np.linspace(-2.5, 2.5, 11), np.linspace(-2.5, 2.5, 11), np.linspace(0.2, 2.5, 6))
    print(f"  pixel IFOV {1e3 / f_px:.3f} mrad")
    for label, nz in [
        ("good: 0.1px, 0.1mrad, 1mm, 0.1ms", Noise(0.1, 1e-4, 1e-3, 1e-4)),
        ("wifi timing: 0.1px, 0.1mrad, 1mm, 2ms", Noise(0.1, 1e-4, 1e-3, 2e-3)),
        ("3 cams only (good)", None),
    ]:
        if nz is None:
            evaluate(label, cams[:3], g, Noise(0.1, 1e-4, 1e-3, 1e-4), 15)
        else:
            evaluate(label, cams, g, nz, 15)

    print("\n== Full hall: 100 x 100 x 30 m ==")
    g = grid(np.linspace(-45, 45, 10), np.linspace(-45, 45, 10), [2.0, 10.0, 20.0, 28.0])
    for spacing, f_mm, heights in [(25, 4.0, (10, 28)), (12.5, 4.0, (10, 28)),
                                   (12.5, 8.0, (10, 28)), (12.5, 4.0, (5, 15, 28))]:
        cams, f_px = hall_rig(spacing, heights, f_mm)
        print(f" layout: {len(cams)} cams, spacing {spacing} m, f {f_mm} mm "
              f"(IFOV {1e3 / f_px:.2f} mrad, HFOV {np.degrees(2*np.arctan(640/f_px)):.0f} deg), heights {heights}")
        for label, nz in [
            ("optimistic: 0.05px, 0.05mrad, 1mm, 0.05ms", Noise(0.05, 5e-5, 1e-3, 5e-5)),
            ("realistic: 0.1px, 0.1mrad, 3mm, 0.1ms", Noise(0.1, 1e-4, 3e-3, 1e-4)),
            ("degraded: 0.2px, 0.3mrad, 5mm, 1ms", Noise(0.2, 3e-4, 5e-3, 1e-3)),
        ]:
            evaluate(label, cams, g, nz, max_range=150)

    for spacing, f_mm in [(20.0, 2.8), (12.5, 2.8)]:
        cams, f_px = ceiling_rig(spacing, f_mm=f_mm)
        cams += hall_rig(12.5, (5, 15), 4.0)[0]
        print(f" layout: {len(cams)} cams = ceiling grid {spacing} m (f {f_mm} mm, "
              f"HFOV {np.degrees(2*np.arctan(640/f_px)):.0f} deg) + perimeter 12.5 m x 2 rings")
        for label, nz in [
            ("optimistic: 0.05px, 0.05mrad, 1mm, 0.05ms", Noise(0.05, 5e-5, 1e-3, 5e-5)),
            ("realistic: 0.1px, 0.1mrad, 3mm, 0.1ms", Noise(0.1, 1e-4, 3e-3, 1e-4)),
            ("degraded: 0.2px, 0.3mrad, 5mm, 1ms", Noise(0.2, 3e-4, 5e-3, 1e-3)),
        ]:
            evaluate(label, cams, g, nz, max_range=150)


if __name__ == "__main__":
    main()
