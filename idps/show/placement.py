"""Where a show sits in the venue: from the show's axes to the venue's NED frame (D18).

Blender (and Skybrush Studio) use X, Y and Z up. The default mapping, ``axes="ENU"``, is the
natural one in Blender's top view with North up the screen: **Blender +X = East, +Y = North,
+Z = up**, and Blender's origin = the venue origin (the calibration square). The IDPS venue
script for Blender (``idps-show blender-venue``) builds the venue in exactly this mapping,
so a show designed inside it needs no adjustment. A rotation about the vertical and an
offset can move a show designed elsewhere.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# show axes -> NED, as a 3x3 matrix applied to (x, y, z)
AXES = {
    "ENU": np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float),   # x East, y North, z up (default)
    "NWU": np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float),  # x North, y West, z up
    "NED": np.eye(3),
}


@dataclass
class Placement:
    axes: str = "ENU"
    rotate_deg: float = 0.0           # turn the show clockwise (seen from above) about the origin
    offset_ned: tuple = (0.0, 0.0, 0.0)

    @property
    def matrix(self) -> np.ndarray:
        a = math.radians(self.rotate_deg)
        Rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
        return Rz @ AXES[self.axes]

    def to_ned(self, p) -> np.ndarray:
        """Show-frame positions (or, for vectors, use vec_to_ned) -> venue NED."""
        return np.asarray(p, float) @ self.matrix.T + np.asarray(self.offset_ned, float)

    def vec_to_ned(self, v) -> np.ndarray:
        return np.asarray(v, float) @ self.matrix.T

    def from_ned(self, ned) -> np.ndarray:
        return (np.asarray(ned, float) - np.asarray(self.offset_ned, float)) @ self.matrix

    @classmethod
    def from_dict(cls, d: dict | None) -> "Placement":
        d = d or {}
        return cls(str(d.get("axes", "ENU")).upper(), float(d.get("rotate_deg", 0.0)),
                   tuple(float(x) for x in d.get("offset_ned", (0, 0, 0))))
