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
from idps.core.frames import Frame
from idps.core.noise import NoiseBudget, noise_from_config
from idps.coverage.fisher import VisibilityLimits
from idps.coverage.volume import Volume, distance_to_polygon, points_in_polygon


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
    """Floor area where tripods may stand. Exactly one of:
      * polygon: explicit floor polygon (a `rect` is converted to one);
      * offset_m: a strip outside the volume edge, between offset_m[0] and
        offset_m[1] metres from it, optionally only along `edges` (indices;
        edge k runs from volume vertex k to vertex k+1).
    """
    id: str
    height_range_m: tuple[float, float]
    polygon: np.ndarray | None = None
    offset_m: tuple[float, float] | None = None
    edges: list[int] | None = None


@dataclass
class CandidateSettings:
    truss_step_m: float = 5.0
    truss_drop_m: float = 0.3
    # shallow tilts matter: cameras hang just above the drones (D17), and only a camera
    # looking nearly level sees the top of the volume across the hall
    truss_elevations_deg: list[float] = field(default_factory=lambda: [-90.0, -60.0, -35.0, -15.0, 0.0])
    tripod_step_m: float = 5.0
    tripod_heights_m: list[float] = field(default_factory=lambda: [4.0, 12.0, 20.0])
    tripod_elevations_deg: list[float] = field(default_factory=lambda: [0.0, -15.0, -30.0, -50.0])
    azimuth_step_deg: float = 30.0
    max_cameras_per_mount: int = 1
    # Drones always fly below the cameras (decision D17): no camera is placed lower than the
    # top of the flight volume plus this margin, and cameras only look level or down.
    min_height_above_volume_m: float = 0.0
    min_camera_spacing_m: float = 1.5      # between cameras on different mounts


@dataclass
class Requirements:
    min_cameras: int = 3
    p95_mm: float = 10.0
    noise: NoiseBudget = field(default_factory=NoiseBudget)


@dataclass
class DesignMargin:
    """What the optimiser aims for. Stricter than the requirement so the plan
    survives occlusion and lost cameras (docs/RISKS.md section 0.1)."""
    min_cameras: int
    p95_mm: float


@dataclass
class Site:
    name: str
    volume: Volume
    sensor: Sensor
    lenses_mm: list[float]
    requirements: Requirements
    design: DesignMargin
    limits: VisibilityLimits
    eval_spacing_m: tuple[float, float, float]
    eval_inset_m: float
    trusses: list[Truss]
    tripod_zones: list[TripodZone]
    no_go_zones: list[np.ndarray]          # floor polygons
    candidates: CandidateSettings
    max_cameras: int
    notes: str = ""
    source_path: str | None = None
    frame: Frame = field(default_factory=Frame)    # origin + North: everything shown in NED

    @property
    def volume_min(self) -> np.ndarray:
        return self.volume.bbox_min

    @property
    def volume_max(self) -> np.ndarray:
        return self.volume.bbox_max

    def warnings(self) -> list[str]:
        out = []
        top = self.volume.z_max
        for t in self.trusses:
            if t.height_m - self.candidates.truss_drop_m < top + self.candidates.min_height_above_volume_m:
                out.append(f"truss {t.id}: cameras would hang at {t.height_m - self.candidates.truss_drop_m:.1f} m,"
                           f" below the flight ceiling ({top:.1f} m); cameras must be above the drones, so this "
                           f"truss is not used")
            for s0, s1 in t.usable:
                if s0 < 0 or s1 > t.length + 1e-6 or s1 <= s0:
                    out.append(f"truss {t.id}: usable segment [{s0}, {s1}] outside 0..{t.length:.1f} m")
        n_edges = len(self.volume.polygon)
        for z in self.tripod_zones:
            if z.polygon is not None:
                inside = points_in_polygon(z.polygon, self.volume.polygon)
                if inside.any() or points_in_polygon(self.volume.polygon, z.polygon).any():
                    out.append(f"tripod zone {z.id} overlaps the flight volume; tripods inside the"
                               f" volume are skipped")
            if z.edges and max(z.edges) >= n_edges:
                out.append(f"tripod zone {z.id}: edge index out of range (volume has {n_edges} edges)")
            if z.height_range_m[1] > 20.0:
                out.append(f"tripod zone {z.id}: height above 20 m")
            need = top + self.candidates.min_height_above_volume_m
            if z.height_range_m[1] < need - 1e-9:
                out.append(f"tripod zone {z.id}: masts reach only {z.height_range_m[1]:g} m, below the flight "
                           f"ceiling ({need:g} m); cameras must be above the drones, so this zone is not used")
        if not self.trusses and not self.tripod_zones:
            out.append("no trusses or tripod zones: nowhere to mount cameras")
        return out


def _rect(v) -> tuple[float, float, float, float]:
    x0, y0, x1, y1 = (float(a) for a in v)
    return (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))


def _rect_polygon(v) -> np.ndarray:
    x0, y0, x1, y1 = _rect(v)
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], float)


def _volume(vol: dict) -> Volume:
    """Either {polygon: [[x, y], ...], z: [z_min, z_max]} or {min: [...], max: [...]}."""
    if "polygon" in vol:
        z = vol.get("z", vol.get("height_m"))
        z = (0.0, float(z)) if np.isscalar(z) else (float(z[0]), float(z[1]))
        return Volume(np.array(vol["polygon"], float), z[0], z[1])
    return Volume.box(vol["min"], vol["max"])


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
    des = d.get("design", {}) or {}
    design = DesignMargin(min_cameras=int(des.get("min_cameras", requirements.min_cameras)),
                          p95_mm=float(des.get("p95_mm", requirements.p95_mm)))
    if design.min_cameras < requirements.min_cameras or design.p95_mm > requirements.p95_mm:
        raise ValueError("design margin must be at least as strict as the requirements")
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
        if "offset_m" in z:
            off = z["offset_m"]
            off = (float(off), float(off)) if np.isscalar(off) else (float(off[0]), float(off[1]))
            edges = z.get("edges")
            zones.append(TripodZone(str(z["id"]), h, offset_m=off,
                                    edges=None if edges is None else [int(e) for e in edges]))
        elif "polygon" in z:
            zones.append(TripodZone(str(z["id"]), h, polygon=np.array(z["polygon"], float)))
        elif "rect" in z:
            zones.append(TripodZone(str(z["id"]), h, polygon=_rect_polygon(z["rect"])))
        else:
            raise ValueError(f"tripod zone {z.get('id')}: needs offset_m, polygon or rect")

    c = d.get("candidates", {}) or {}
    defaults = CandidateSettings()
    cand = CandidateSettings(**{k: c.get(k, getattr(defaults, k)) for k in defaults.__dataclass_fields__})

    return Site(
        name=str(d.get("name", "site")),
        volume=_volume(vol),
        sensor=SENSORS[sensor_name],
        lenses_mm=[float(x) for x in d.get("lenses_mm", [2.8])],
        requirements=requirements,
        design=design,
        limits=limits,
        eval_spacing_m=spacing,
        eval_inset_m=float(ev.get("inset_m", 0.5)),
        trusses=trusses,
        tripod_zones=zones,
        no_go_zones=[np.array(r, float) if np.ndim(r) == 2 else _rect_polygon(r)
                     for r in d.get("no_go_zones", []) or []],
        candidates=cand,
        max_cameras=int(d.get("optimiser", {}).get("max_cameras", 400)),
        notes=str(d.get("notes", "")),
        source_path=source_path,
        frame=Frame.from_dict(d.get("origin")),
    )


def load_site(path: str | Path) -> Site:
    path = Path(path)
    with open(path) as f:
        return site_from_dict(yaml.safe_load(f), str(path))
