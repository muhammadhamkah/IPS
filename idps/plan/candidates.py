"""Candidate camera placements: every allowed mount x aim x lens."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from idps.core.camera import rotation_from_az_el
from idps.plan.site import Site


@dataclass
class CandidateSet:
    pos: np.ndarray          # (N, 3)
    az: np.ndarray           # (N,) degrees
    el: np.ndarray           # (N,) degrees
    lens_mm: np.ndarray      # (N,)
    f_px: np.ndarray         # (N,)
    mount_key: np.ndarray    # (N,) str - identical for candidates sharing a physical mount
    mount_type: np.ndarray   # (N,) "truss" | "tripod"
    mount_id: np.ndarray     # (N,) truss id or tripod zone id
    along_m: np.ndarray      # (N,) distance along the truss from its start; NaN for tripods

    def __len__(self) -> int:
        return len(self.az)

    @property
    def R(self) -> np.ndarray:
        return rotation_from_az_el(self.az, self.el)


def _in_rect(x, y, r) -> bool:
    return r[0] <= x <= r[2] and r[1] <= y <= r[3]


def _aims(elevations, az_step):
    azs = np.arange(0.0, 360.0, az_step)
    out = []
    for el in elevations:
        if abs(abs(el) - 90.0) < 1e-6:
            out.append((0.0, float(el)))          # straight up/down: azimuth irrelevant
        else:
            out += [(float(a), float(el)) for a in azs]
    return out


def _samples(a: float, b: float, step: float) -> np.ndarray:
    """Evenly spaced samples from a to b inclusive, spacing <= step."""
    if b - a < 1e-9:
        return np.array([a])
    n = int(np.ceil((b - a) / step - 1e-9)) + 1
    return np.linspace(a, b, n)


def generate_candidates(site: Site) -> CandidateSet:
    cs = site.candidates
    rows = []

    def blocked(x, y):
        return any(_in_rect(x, y, r) for r in site.no_go_zones)

    truss_aims = _aims(cs.truss_elevations_deg, cs.azimuth_step_deg)
    for t in site.trusses:
        for s0, s1 in t.usable:
            for s in _samples(s0, s1, cs.truss_step_m):
                xy = t.point_at(s)
                if blocked(*xy):
                    continue
                pos = (xy[0], xy[1], t.height_m - cs.truss_drop_m)
                key = f"{t.id}@{s:.2f}"
                for az, el in truss_aims:
                    for lens in site.lenses_mm:
                        rows.append((pos, az, el, lens, key, "truss", t.id, s))

    tripod_aims = _aims(cs.tripod_elevations_deg, cs.azimuth_step_deg)
    for z in site.tripod_zones:
        x0, y0, x1, y1 = z.rect
        heights = [h for h in cs.tripod_heights_m if z.height_range_m[0] <= h <= z.height_range_m[1]]
        if not heights:
            heights = sorted({z.height_range_m[0], z.height_range_m[1]})
        for x in _samples(x0, x1, cs.tripod_step_m):
            for y in _samples(y0, y1, cs.tripod_step_m):
                if blocked(x, y):
                    continue
                for h in heights:
                    key = f"{z.id}@{x:.2f},{y:.2f},{h:.1f}"
                    for az, el in tripod_aims:
                        for lens in site.lenses_mm:
                            rows.append(((x, y, h), az, el, lens, key, "tripod", z.id, np.nan))

    if not rows:
        raise ValueError("no candidate mounts: add trusses or tripod zones to the site file")
    pos, az, el, lens, key, mtype, mid, along = zip(*rows)
    lens = np.array(lens, float)
    return CandidateSet(
        pos=np.array(pos, float), az=np.array(az), el=np.array(el), lens_mm=lens,
        f_px=lens * 1e-3 / site.sensor.pixel_pitch_m, mount_key=np.array(key),
        mount_type=np.array(mtype), mount_id=np.array(mid), along_m=np.array(along, float))
