"""Pinhole camera model shared by the planner, simulator and engine.

Conventions (docs/ARCHITECTURE.md section 3):
  * World frame: right-handed, Z up, metres.
  * Camera frame: OpenCV - +X right, +Y down, +Z along the optical axis.
  * Aim is given as azimuth (degrees from world +X towards +Y) and elevation
    (degrees above horizontal; negative looks down). Camera roll is zero: the
    image x axis stays horizontal.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Sensor:
    name: str
    width_px: int
    height_px: int
    pixel_pitch_m: float

    def f_px(self, focal_mm: float) -> float:
        return focal_mm * 1e-3 / self.pixel_pitch_m

    def hfov_deg(self, focal_mm: float) -> float:
        return float(np.degrees(2 * np.arctan(self.width_px / 2 / self.f_px(focal_mm))))

    def vfov_deg(self, focal_mm: float) -> float:
        return float(np.degrees(2 * np.arctan(self.height_px / 2 / self.f_px(focal_mm))))


SENSORS = {
    "OV9281": Sensor("OV9281", 1280, 800, 3.0e-6),
}


def direction_from_az_el(az_deg, el_deg) -> np.ndarray:
    """Unit viewing direction(s) in the world frame. Broadcasts over inputs."""
    az = np.radians(az_deg)
    el = np.radians(el_deg)
    return np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=-1)


def rotation_from_az_el(az_deg, el_deg) -> np.ndarray:
    """World->camera rotation matrix (rows are camera x, y, z axes in world).

    Broadcasts: scalar inputs give (3, 3), arrays of shape S give S + (3, 3).
    The camera x axis is (sin az, -cos az, 0), which stays well defined when
    looking straight up or down.
    """
    az = np.radians(np.asarray(az_deg, float))
    fwd = direction_from_az_el(az_deg, el_deg)
    right = np.stack([np.sin(az), -np.cos(az), np.zeros_like(az)], axis=-1)
    down = np.cross(fwd, right)
    return np.stack([right, down, fwd], axis=-2)


def az_el_from_direction(d) -> tuple[float, float]:
    d = np.asarray(d, float)
    d = d / np.linalg.norm(d)
    el = float(np.degrees(np.arcsin(np.clip(d[2], -1, 1))))
    az = float(np.degrees(np.arctan2(d[1], d[0])))
    return az, el


def project(R: np.ndarray, pos: np.ndarray, f_px: float, points: np.ndarray):
    """Project world points; returns (u, v, z) with (u, v) relative to the
    principal point, in pixels, and z the depth along the optical axis."""
    Xc = (np.asarray(points, float) - pos) @ R.T
    z = Xc[..., 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        u = f_px * Xc[..., 0] / z
        v = f_px * Xc[..., 1] / z
    return u, v, z
