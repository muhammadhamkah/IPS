"""Plan data model and YAML/CSV serialisation."""
from __future__ import annotations

import csv
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import yaml

from idps.core.camera import rotation_from_az_el


@dataclass
class PlannedCamera:
    id: str
    mount_type: str               # "truss" | "tripod"
    mount_id: str                 # truss id or tripod zone id
    position_m: list[float]       # world XYZ of the camera
    azimuth_deg: float            # aim direction in the world frame
    elevation_deg: float
    lens_mm: float
    along_truss_m: float | None = None   # distance from the truss start (truss mounts)
    # Installer instructions: the gimbal base is set to base_heading_deg
    # (pan-zero direction, world azimuth); the servos then go to pan/tilt.
    base_heading_deg: float = 0.0
    pan_deg: float = 0.0
    tilt_deg: float = 0.0
    install_note: str = ""
    neighbour_spacing_m: float | None = None

    @property
    def pos(self) -> np.ndarray:
        return np.array(self.position_m, float)

    @property
    def R(self) -> np.ndarray:
        return rotation_from_az_el(self.azimuth_deg, self.elevation_deg)


@dataclass
class Plan:
    site_name: str
    site_file: str | None
    cameras: list[PlannedCamera]
    summary: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "format": "idps-plan/1",
            "site_name": self.site_name,
            "site_file": self.site_file,
            "summary": self.summary,
            "cameras": [_clean(asdict(c)) for c in self.cameras],
        }

    def save(self, path: str | Path) -> None:
        with open(path, "w") as f:
            yaml.safe_dump(self.to_dict(), f, sort_keys=False, width=120)

    @classmethod
    def load(cls, path: str | Path) -> "Plan":
        with open(path) as f:
            d = yaml.safe_load(f)
        if d.get("format") != "idps-plan/1":
            raise ValueError(f"{path}: not an idps-plan/1 file")
        cams = [PlannedCamera(**c) for c in d["cameras"]]
        return cls(d["site_name"], d.get("site_file"), cams, d.get("summary", {}))

    def write_csv(self, path: str | Path) -> None:
        cols = ["id", "mount_type", "mount_id", "along_truss_m", "x_m", "y_m", "z_m",
                "base_heading_deg", "pan_deg", "tilt_deg", "lens_mm", "neighbour_spacing_m",
                "install_note"]
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(cols)
            for c in self.cameras:
                x, y, z = c.position_m
                w.writerow([c.id, c.mount_type, c.mount_id, _fmt(c.along_truss_m), f"{x:.2f}",
                            f"{y:.2f}", f"{z:.2f}", f"{c.base_heading_deg:.0f}", f"{c.pan_deg:.1f}",
                            f"{c.tilt_deg:.1f}", f"{c.lens_mm:g}", _fmt(c.neighbour_spacing_m),
                            c.install_note])


def _fmt(v):
    return "" if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:.2f}"


def _clean(d: dict) -> dict:
    out = {}
    for k, v in d.items():
        if isinstance(v, float) and math.isnan(v):
            v = None
        elif isinstance(v, (np.floating, float)):
            v = round(float(v), 4)
        elif isinstance(v, list):
            v = [round(float(x), 4) for x in v]
        out[k] = v
    return out


def install_angles(az_deg: float, el_deg: float, heading_step_deg: float = 45.0):
    """Split a world aim into a gimbal base heading (rounded so installers can
    set it with a compass/laser square) plus a small servo pan, and a tilt."""
    base = (round(az_deg / heading_step_deg) * heading_step_deg) % 360.0
    pan = (az_deg - base + 180.0) % 360.0 - 180.0
    return base, pan, el_deg
