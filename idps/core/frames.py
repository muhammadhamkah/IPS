"""Venue coordinate frames.

Geometry is stored and computed in the *drawing frame*: the plan as drawn, X right and Y
up on the plan, Z up from the floor, metres. Everything shown to people and sent to
drones uses **North-East-Down (NED)** relative to the **origin**: the corner of the
calibration square on the floor. North is the direction of the square's long arm (and
of a drone's yaw 0), East is 90° clockwise from it seen from above, Down is down.
This is the local frame flight controllers (PX4, ArduPilot) use.

The origin and the direction of North are set on the plan (drag the origin, turn its N
arrow) and stored in the site file as ``origin: {xy: [x, y], north_deg: a}``, where `a`
is the direction of North in the drawing, measured counter-clockwise from the drawing's
+X axis (90 = North points up the plan).

Headings are compass style: degrees from North, clockwise (East = 90).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Frame:
    origin_xy: tuple = (0.0, 0.0)
    north_deg: float = 0.0               # direction of North in the drawing, CCW from drawing +X
    floor_z: float = 0.0                 # the calibration square lies on the floor

    @classmethod
    def from_dict(cls, d: dict | None) -> "Frame":
        d = d or {}
        xy = d.get("xy", (0.0, 0.0))
        return cls((float(xy[0]), float(xy[1])), float(d.get("north_deg", 0.0)), float(d.get("floor_z", 0.0)))

    def to_dict(self) -> dict:
        return {"xy": [float(self.origin_xy[0]), float(self.origin_xy[1])], "north_deg": float(self.north_deg)}

    # unit vectors of North and East in the drawing's XY plane
    @property
    def n_hat(self) -> np.ndarray:
        a = math.radians(self.north_deg)
        return np.array([math.cos(a), math.sin(a)])

    @property
    def e_hat(self) -> np.ndarray:
        a = math.radians(self.north_deg)
        return np.array([math.sin(a), -math.cos(a)])          # North turned 90° clockwise

    # ---------------------------------------------------------------- points
    def ne(self, xy) -> np.ndarray:
        """Drawing XY -> (N, E). Accepts (2,) or (K, 2)."""
        d = np.asarray(xy, float) - np.asarray(self.origin_xy, float)
        return np.stack([d @ self.n_hat, d @ self.e_hat], -1)

    def xy(self, ne) -> np.ndarray:
        """(N, E) -> drawing XY."""
        ne = np.asarray(ne, float)
        return np.asarray(self.origin_xy, float) + ne[..., :1] * self.n_hat + ne[..., 1:2] * self.e_hat

    def ned(self, p) -> np.ndarray:
        """Drawing XYZ -> NED. Accepts (3,) or (K, 3)."""
        p = np.asarray(p, float)
        ne = self.ne(p[..., :2])
        return np.concatenate([ne, -(p[..., 2:3] - self.floor_z)], -1)

    def from_ned(self, ned) -> np.ndarray:
        ned = np.asarray(ned, float)
        return np.concatenate([self.xy(ned[..., :2]), self.floor_z - ned[..., 2:3]], -1)

    # -------------------------------------------------------------- headings
    def heading(self, az_deg: float) -> float:
        """Drawing azimuth (CCW from drawing +X) -> compass heading (clockwise from North)."""
        return (self.north_deg - az_deg) % 360.0

    def azimuth(self, heading_deg: float) -> float:
        return (self.north_deg - heading_deg) % 360.0

    # ----------------------------------------------------------------- text
    def fmt_ne(self, xy, nd: int = 2) -> str:
        n, e = self.ne(np.asarray(xy, float)[:2])
        return f"N {n:.{nd}f}, E {e:.{nd}f}"

    def fmt_ned(self, p, nd: int = 2) -> str:
        n, e, d = self.ned(np.asarray(p, float)[:3])
        return f"N {n:.{nd}f}, E {e:.{nd}f}, D {d:.{nd}f}"


def pan_display(pan_deg: float) -> float:
    """Servo pan as shown to people: clockwise positive (like a heading)."""
    return -pan_deg
