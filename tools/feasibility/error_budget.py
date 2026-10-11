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

Usage: python3 tools/feasibility/error_budget.py [--sweep] [--beacons] [--all]
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




# ---------------------------------------------------------------------------
# Full-volume requirement sweep and beacon-based camera pose precision.
# ---------------------------------------------------------------------------

# For an isotropic 3D Gaussian, the 95 % error radius is ~2.80 sigma per axis
# = 1.62 x 3D RMS.  "Within 1 cm" read as p95 <= 10 mm therefore needs
# 3D RMS <= 6.2 mm at *every* location in the volume.
RMS_LIMIT_MM = 10.0 / 1.62


def worst_case(cams, g, noise, max_range=150.0):
    """Per-point 3D RMS error (mm); NaN where < 2 cameras see the point."""
    masks = np.array([visible(c, g, max_range) for c in cams])
    err = np.full(len(g), np.nan)
    for i, X in enumerate(g):
        seen = [c for c, m in zip(cams, masks[:, i]) if m]
        if len(seen) >= 2:
            err[i] = np.sqrt(np.trace(point_cov(seen, X, noise))) * 1000
    return err


def hybrid_rig(ceiling_spacing, perim_spacing, perim_heights, f_ceiling=2.8, f_perim=4.0):
    cams = ceiling_rig(ceiling_spacing, f_mm=f_ceiling)[0]
    cams += hall_rig(perim_spacing, perim_heights, f_perim)[0]
    return cams


def full_volume_sweep():
    """Which layout / noise combinations meet p95 <= 10 mm everywhere?"""
    # Flight volume inset 1 m from walls and ceiling; from 0.2 m above floor.
    g = grid(np.linspace(-49, 49, 9), np.linspace(-49, 49, 9), [0.2, 5, 10, 15, 20, 25, 29])
    print(f"\n== Full-volume requirement: 3D RMS <= {RMS_LIMIT_MM:.1f} mm at every point ==")
    print("   (grid inset 1 m from walls/ceiling, 0.2-29 m altitude, includes corners)")
    layouts = [
        ("ceil 20 m + perim 12.5 m x (5,15,28)", hybrid_rig(20, 12.5, (5, 15, 28))),
        ("ceil 12.5 m + perim 12.5 m x (5,15,28)", hybrid_rig(12.5, 12.5, (5, 15, 28))),
        ("ceil 10 m + perim 10 m x (2,10,20,29)", hybrid_rig(10, 10, (2, 10, 20, 29))),
    ]
    noises = [
        ("0.10px 0.10mrad 3mm 0.10ms", Noise(0.1, 1e-4, 3e-3, 1e-4)),
        ("0.05px 0.05mrad 2mm 0.05ms", Noise(0.05, 5e-5, 2e-3, 5e-5)),
        ("0.05px 0.03mrad 1mm 0.02ms", Noise(0.05, 3e-5, 1e-3, 2e-5)),
    ]
    for lname, cams in layouts:
        print(f" {lname}: {len(cams)} cameras")
        for nname, nz in noises:
            e = worst_case(cams, g, nz)
            ok = ~np.isnan(e)
            meets = np.mean(e[ok] <= RMS_LIMIT_MM) * 100 if ok.any() else 0
            print(f"   {nname:<30s} cover {ok.mean()*100:5.1f}%  RMS median {np.nanmedian(e):5.1f}"
                  f"  max {np.nanmax(e):5.1f} mm  points meeting spec {meets:5.1f}%")


def pose_precision(n_beacons, ranges, f_px, sigma_px, sigma_beacon_m, n_frames=1, seed=0,
                   rotation_only=False):
    """1-sigma camera rotation / position error from PnP on fixed beacons.

    Beacons are spread over the central 80 % of the image at the given ranges.
    Survey error of each beacon is folded in as extra image noise.
    rotation_only=True assumes the camera position is known (servo gimbal with
    a calibrated kinematic model) and estimates only the 3 rotation angles.
    """
    rng = np.random.default_rng(seed)
    w, h = SENSOR_PX
    info = np.zeros((6, 6))
    for k in range(n_beacons):
        u = rng.uniform(-0.4, 0.4) * w
        v = rng.uniform(-0.4, 0.4) * h
        z = ranges[k % len(ranges)]
        Xc = np.array([u / f_px * z, v / f_px * z, z])
        # d(normalised image)/d(pose): small rotation dtheta and translation dt
        # in the camera frame, Xc' = Xc + dtheta x Xc + dt.
        J_c = np.array([[1 / z, 0, -Xc[0] / z ** 2], [0, 1 / z, -Xc[1] / z ** 2]])
        skew = np.array([[0, Xc[2], -Xc[1]], [-Xc[2], 0, Xc[0]], [Xc[1], -Xc[0], 0]])
        J = np.hstack([J_c @ skew, J_c])
        # Survey error is systematic (not reduced by averaging frames).
        s2_meas = (sigma_px / f_px) ** 2 / n_frames
        s2_survey = (sigma_beacon_m / z) ** 2
        info += J.T @ J / (s2_meas + s2_survey)
    if rotation_only:
        return np.sqrt(np.trace(np.linalg.inv(info[:3, :3]))), 0.0
    C = np.linalg.inv(info)
    rot = np.sqrt(np.trace(C[:3, :3]))
    pos = np.sqrt(np.trace(C[3:, 3:]))
    return rot, pos


def beacon_study():
    print("\n== Camera pose from fixed beacons (live re-pose for servo gimbals) ==")
    print("   rotation / position 1-sigma; beacons spread over 80 % of the image")
    f_px = 4.0e-3 / PIXEL_PITCH_M
    for n in (4, 6, 9, 16):
        for ranges in ((20, 40, 60), (40, 70, 100)):
            for frames, label in ((1, "1 frame"), (12, "12 frames (0.1 s)")):
                rot, pos = pose_precision(n, ranges, f_px, 0.1, 2e-3, frames)
                print(f"   {n:2d} beacons @ {str(ranges):<13s} {label:<18s}"
                      f" rot {rot*1e3:6.3f} mrad   pos {pos*1e3:6.1f} mm")
    print("   -- rotation only (camera position known from gimbal kinematics) --")
    for n in (4, 9, 16):
        for ranges in ((20, 40, 60), (40, 70, 100)):
            rots = [pose_precision(n, ranges, f_px, 0.1, 2e-3, fr, rotation_only=True)[0]
                    for fr in (1, 12, 120)]
            print(f"   {n:2d} beacons @ {str(ranges):<13s} rot 1 frame {rots[0]*1e3:5.3f}"
                  f"  0.1 s {rots[1]*1e3:5.3f}  1 s {rots[2]*1e3:5.3f} mrad")


if __name__ == "__main__":
    import sys
    flags = set(sys.argv[1:])
    if not flags or "--all" in flags:
        main()
    if flags & {"--sweep", "--all"}:
        full_volume_sweep()
    if flags & {"--beacons", "--all"}:
        beacon_study()
