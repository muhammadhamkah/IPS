"""Camera calibration as the engine uses it: pixel -> line of sight, and world point -> pixel.

A camera's calibration has two parts (docs/ARCHITECTURE.md section 6):

* **intrinsics**, from the lab: focal length in pixels, principal point and lens distortion
  (Brown-Conrady k1, k2, p1, p2, k3);
* **extrinsics**, from the wand and calibration square: where the camera is and which way it
  points (``R`` turns world directions into camera directions).

World frame here is the planner's drawing frame (metres, Z up); ``idps.core.frames.Frame``
turns results into the venue's NED frame.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


@dataclass
class CameraCal:
    station_id: int
    position: np.ndarray        # (3,) camera centre, world
    R: np.ndarray               # (3, 3) world -> camera (rows: camera x right, y down, z forward)
    f_px: float
    cx: float
    cy: float
    distortion: np.ndarray      # k1, k2, p1, p2, k3
    width: int = 1280
    height: int = 800
    plan_id: str = ""

    def normalised(self, x, y, iterations: int = 8):
        """Distorted pixel(s) -> undistorted normalised image coordinates (x/z, y/z)."""
        k1, k2, p1, p2, k3 = self.distortion
        xd = (np.asarray(x, float) - self.cx) / self.f_px
        yd = (np.asarray(y, float) - self.cy) / self.f_px
        xn, yn = xd.copy(), yd.copy()
        for _ in range(iterations):             # fixed-point inversion of the distortion model
            r2 = xn * xn + yn * yn
            radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
            dx = 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn)
            dy = p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn
            xn = (xd - dx) / radial
            yn = (yd - dy) / radial
        return xn, yn

    def rays(self, x, y) -> np.ndarray:
        """Distorted pixel(s) -> unit line-of-sight directions in the world frame, (N, 3)."""
        xn, yn = self.normalised(x, y)
        d_cam = np.stack([xn, yn, np.ones_like(xn)], axis=-1)
        d = d_cam @ self.R                       # camera -> world (R is orthonormal)
        return d / np.linalg.norm(d, axis=-1, keepdims=True)

    def project(self, X) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """World point(s) (N, 3) -> distorted pixel x, y and depth z (z <= 0: behind)."""
        Xc = (np.atleast_2d(np.asarray(X, float)) - self.position) @ self.R.T
        z = Xc[:, 2]
        zs = np.where(z > 1e-9, z, 1e-9)
        xn, yn = Xc[:, 0] / zs, Xc[:, 1] / zs
        k1, k2, p1, p2, k3 = self.distortion
        r2 = xn * xn + yn * yn
        radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
        xd = xn * radial + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn)
        yd = yn * radial + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn
        return self.cx + self.f_px * xd, self.cy + self.f_px * yd, z

    def in_image(self, x, y, margin: float = 0.0):
        return (x >= -margin) & (x <= self.width - 1 + margin) & (y >= -margin) & (y <= self.height - 1 + margin)


def rays_batch(cams: list[CameraCal], xy: np.ndarray, iterations: int = 8) -> np.ndarray:
    """Lines of sight for dots from several cameras at once: ``cams[i]`` took dot ``xy[i]``.
    Same result as each camera's ``rays``, in one vectorised pass."""
    if not len(cams):
        return np.zeros((0, 3))
    f = np.array([c.f_px for c in cams])
    cx = np.array([c.cx for c in cams])
    cy = np.array([c.cy for c in cams])
    k1, k2, p1, p2, k3 = np.array([c.distortion for c in cams]).T
    xd = (xy[:, 0] - cx) / f
    yd = (xy[:, 1] - cy) / f
    xn, yn = xd.copy(), yd.copy()
    for _ in range(iterations):
        r2 = xn * xn + yn * yn
        radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
        dx = 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn)
        dy = p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn
        xn, yn = (xd - dx) / radial, (yd - dy) / radial
    d_cam = np.stack([xn, yn, np.ones_like(xn)], axis=-1)
    R = np.array([c.R for c in cams])
    d = np.einsum("ni,nij->nj", d_cam, R)
    return d / np.linalg.norm(d, axis=-1, keepdims=True)


def load_calibration(path, which: str = "estimate") -> dict[int, CameraCal]:
    """Read an ``idps-calibration/1`` file. ``which`` = "estimate" (what a real calibration
    gives, the default) or "true" (simulator only: the exact cameras, for diagnosis)."""
    doc = yaml.safe_load(Path(path).read_text())
    if doc.get("format") != "idps-calibration/1":
        raise ValueError(f"{path}: not an idps-calibration/1 file")
    out = {}
    for c in doc["cameras"]:
        ext = c.get(which) or c.get("estimate") or c
        out[int(c["station_id"])] = CameraCal(
            int(c["station_id"]), np.asarray(ext["position_m"], float), np.asarray(ext["R_world_to_cam"], float),
            float(c["f_px"]), float(c["cx"]), float(c["cy"]), np.asarray(c.get("distortion", [0] * 5), float),
            int(doc.get("width_px", 1280)), int(doc.get("height_px", 800)), str(c.get("plan_camera_id", "")))
    return out
