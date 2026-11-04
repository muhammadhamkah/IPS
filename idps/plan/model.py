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
    frame: dict | None = None     # origin + North of the venue ({xy, north_deg}); see idps.core.frames

    def _frame(self):
        from idps.core.frames import Frame
        return Frame.from_dict(self.frame)

    def ned_fields(self, c: PlannedCamera) -> dict:
        """What installers and the drones use: North-East-Down from the origin, compass headings."""
        from idps.core.frames import pan_display
        f = self._frame()
        n, e, d = f.ned(c.position_m)
        return {"ned_m": [round(float(n), 4), round(float(e), 4), round(float(d), 4)],
                "view_heading_deg": round(f.heading(c.azimuth_deg), 2),
                "base_heading_from_north_deg": round(f.heading(c.base_heading_deg), 2),
                "pan_clockwise_deg": round(pan_display(c.pan_deg), 2)}

    def to_dict(self) -> dict:
        return {
            "format": "idps-plan/1",
            "site_name": self.site_name,
            "site_file": self.site_file,
            "frame": {**self._frame().to_dict(), "convention": "NED from the origin; drawing-frame fields "
                      "(position_m, azimuth_deg, base_heading_deg, pan_deg) are for the software"},
            "summary": self.summary,
            "cameras": [{**_clean(asdict(c)), **self.ned_fields(c)} for c in self.cameras],
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
        derived = ("ned_m", "view_heading_deg", "base_heading_from_north_deg", "pan_clockwise_deg")
        cams = [PlannedCamera(**{k: v for k, v in c.items() if k not in derived}) for c in d["cameras"]]
        fr = d.get("frame")
        fr = {"xy": fr["xy"], "north_deg": fr["north_deg"]} if fr else None
        return cls(d["site_name"], d.get("site_file"), cams, d.get("summary", {}), fr)

    def write_csv(self, path: str | Path) -> None:
        cols = ["id", "mount_type", "mount_id", "along_truss_m", "north_m", "east_m", "down_m", "height_m",
                "base_heading_from_north_deg", "pan_clockwise_deg", "tilt_deg", "view_heading_deg", "lens_mm",
                "neighbour_spacing_m", "install_note"]
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(cols)
            for c in self.cameras:
                nf = self.ned_fields(c)
                n, e, d = nf["ned_m"]
                w.writerow([c.id, c.mount_type, c.mount_id, _fmt(c.along_truss_m), f"{n:.2f}", f"{e:.2f}",
                            f"{d:.2f}", f"{c.position_m[2]:.2f}", f"{nf['base_heading_from_north_deg']:.0f}",
                            f"{nf['pan_clockwise_deg']:.1f}", f"{c.tilt_deg:.1f}", f"{nf['view_heading_deg']:.1f}",
                            f"{c.lens_mm:g}", _fmt(c.neighbour_spacing_m), c.install_note])


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


def install_angles(az_deg: float, el_deg: float, heading_step_deg: float = 45.0, north_deg: float = 0.0):
    """Split a world aim into a gimbal base heading (rounded to compass steps from North, so
    installers can set it with a compass or laser square) plus a small servo pan, and a tilt.
    Angles in and out are drawing-frame azimuths; north_deg is where North points in it."""
    heading = (north_deg - az_deg) % 360.0
    base = (north_deg - round(heading / heading_step_deg) * heading_step_deg) % 360.0
    pan = (az_deg - base + 180.0) % 360.0 - 180.0
    return base, pan, el_deg
