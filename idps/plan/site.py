"""Site description loaded from YAML (see examples/sites/*.yaml).

All coordinates are in the tracking world frame: origin at the calibration
square, +X along its X arm, Z up, metres.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from idps.core.camera import SENSORS, Sensor
from idps.core.noise import NoiseBudget, noise_from_config
from idps.coverage.fisher import VisibilityLimits


@dataclass
class Truss:
    id: str
    start: np.ndarray            # (2,) floor-plan XY of one end
    end: np.ndarray              # (2,)
    height_m: float              # height of the truss (camera hangs below it)
    usable: list[tuple[float, float]]   # usable segments, metres along the truss from `start`

    @property
    def length(self) -> float:
        return float(np.linalg.norm(self.end - self.start))

    def point_at(self, s: float) -> np.ndarray:
        return self.start + (self.end - self.start) * (s / self.length)


@dataclass
class TripodZone:
    id: str
    rect: tuple[float, float, float, float]   # xmin, ymin, xmax, ymax
    height_range_m: tuple[float, float]


@dataclass
class CandidateSettings:
    truss_step_m: float = 5.0
    truss_drop_m: float = 0.3
    truss_elevations_deg: list[float] = field(default_factory=lambda: [-90.0, -60.0, -35.0])
    tripod_step_m: float = 5.0
    tripod_heights_m: list[float] = field(default_factory=lambda: [4.0, 12.0, 20.0])
    tripod_elevations_deg: list[float] = field(default_factory=lambda: [0.0, 25.0, 50.0, 75.0])
    azimuth_step_deg: float = 30.0
    max_cameras_per_mount: int = 2


@dataclass
class Requirements:
    min_cameras: int = 3
    p95_mm: float = 10.0
    noise: NoiseBudget = field(default_factory=NoiseBudget)


@dataclass
class Site:
    name: str
    volume_min: np.ndarray
    volume_max: np.ndarray
    sensor: Sensor
    lenses_mm: list[float]
    requirements: Requirements
    limits: VisibilityLimits
    eval_spacing_m: tuple[float, float, float]
    eval_inset_m: float
    trusses: list[Truss]
    tripod_zones: list[TripodZone]
    no_go_zones: list[tuple[float, float, float, float]]
    candidates: CandidateSettings
    max_cameras: int
    notes: str = ""
    source_path: str | None = None

    def warnings(self) -> list[str]:
        out = []
        top = self.volume_max[2]
        for t in self.trusses:
            if t.height_m - self.candidates.truss_drop_m < top:
                out.append(f"truss {t.id}: cameras would hang at {t.height_m - self.candidates.truss_drop_m:.1f} m,"
                           f" inside the flight volume (top {top:.1f} m)")
            for s0, s1 in t.usable:
                if s0 < 0 or s1 > t.length + 1e-6 or s1 <= s0:
                    out.append(f"truss {t.id}: usable segment [{s0}, {s1}] outside 0..{t.length:.1f} m")
        for z in self.tripod_zones:
            x0, y0, x1, y1 = z.rect
            if (x0 < self.volume_max[0] and x1 > self.volume_min[0]
                    and y0 < self.volume_max[1] and y1 > self.volume_min[1]):
                out.append(f"tripod zone {z.id} overlaps the flight volume footprint")
            if z.height_range_m[1] > 20.0:
                out.append(f"tripod zone {z.id}: height above 20 m")
        if not self.trusses and not self.tripod_zones:
            out.append("no trusses or tripod zones: nowhere to mount cameras")
        return out


def _rect(v) -> tuple[float, float, float, float]:
    x0, y0, x1, y1 = (float(a) for a in v)
    return (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))


def site_from_dict(d: dict, source_path: str | None = None) -> Site:
    vol = d["volume"]
    sensor_name = d.get("sensor", "OV9281")
    if sensor_name not in SENSORS:
        raise ValueError(f"unknown sensor {sensor_name!r}; known: {sorted(SENSORS)}")
    req = d.get("requirements", {})
    requirements = Requirements(
        min_cameras=int(req.get("min_cameras", 3)),
        p95_mm=float(req.get("p95_mm", 10.0)),
        noise=noise_from_config(req.get("noise", "tight")),
    )
    vis = d.get("visibility", {})
    limits = VisibilityLimits(
        max_range_m=float(vis.get("max_range_m", 120.0)),
        min_range_m=float(vis.get("min_range_m", 0.5)),
        edge_margin=float(vis.get("edge_margin", 0.05)),
    )
    ev = d.get("evaluation", {})
    sp = ev.get("spacing_m", 5.0)
    spacing = tuple(float(x) for x in (sp if isinstance(sp, (list, tuple)) else (sp, sp, sp)))

    trusses = []
    for t in d.get("trusses", []) or []:
        start = np.array(t["start"], float)
        end = np.array(t["end"], float)
        length = float(np.linalg.norm(end - start))
        usable = t.get("usable")
        segs = [(0.0, length)] if usable in (None, "all") else [(float(a), float(b)) for a, b in usable]
        trusses.append(Truss(str(t["id"]), start, end, float(t["height_m"]), segs))

    zones = []
    for z in d.get("tripod_zones", []) or []:
        h = z.get("height_m", [2.0, 20.0])
        h = (float(h), float(h)) if np.isscalar(h) else (float(h[0]), float(h[1]))
        zones.append(TripodZone(str(z["id"]), _rect(z["rect"]), h))

    c = d.get("candidates", {}) or {}
    defaults = CandidateSettings()
    cand = CandidateSettings(**{k: c.get(k, getattr(defaults, k)) for k in defaults.__dataclass_fields__})

    return Site(
        name=str(d.get("name", "site")),
        volume_min=np.array(vol["min"], float),
        volume_max=np.array(vol["max"], float),
        sensor=SENSORS[sensor_name],
        lenses_mm=[float(x) for x in d.get("lenses_mm", [2.8])],
        requirements=requirements,
        limits=limits,
        eval_spacing_m=spacing,
        eval_inset_m=float(ev.get("inset_m", 0.5)),
        trusses=trusses,
        tripod_zones=zones,
        no_go_zones=[_rect(r) for r in d.get("no_go_zones", []) or []],
        candidates=cand,
        max_cameras=int(d.get("optimiser", {}).get("max_cameras", 400)),
        notes=str(d.get("notes", "")),
        source_path=source_path,
    )


def load_site(path: str | Path) -> Site:
    path = Path(path)
    with open(path) as f:
        return site_from_dict(yaml.safe_load(f), str(path))
