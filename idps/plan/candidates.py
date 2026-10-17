"""Candidate camera placements: every allowed mount x aim x lens."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from idps.core.camera import rotation_from_az_el
from idps.coverage.volume import distance_to_polygon, offset_ring, points_in_polygon
from idps.plan.site import Site, TripodZone


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


def _aims(elevations, az_step):
    """Aims at or below horizontal: drones are always below the cameras, so looking up sees nothing."""
    azs = np.arange(0.0, 360.0, az_step)
    out = []
    for el in elevations:
        if el > 1e-6:
            continue
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


def tripod_positions(site: Site, zone: TripodZone) -> np.ndarray:
    """Candidate tripod floor positions for a zone, about tripod_step_m apart."""
    step = site.candidates.tripod_step_m
    if zone.offset_m is not None:
        poly = site.volume.polygon
        pts = []
        for d in _samples(zone.offset_m[0], zone.offset_m[1], step):
            ring = offset_ring(poly, float(d), step)
            dist, e, _ = distance_to_polygon(ring, poly)
            # In concave corners the offset strips of neighbouring edges overlap,
            # bringing points closer to the volume than allowed: drop those.
            keep = dist >= zone.offset_m[0] - 1e-6
            if zone.edges is not None:
                keep &= np.isin(e, zone.edges)   # only along the chosen edges
            pts.append(ring[keep])
        return np.concatenate(pts) if pts else np.zeros((0, 2))
    poly = zone.polygon
    (x0, y0), (x1, y1) = poly.min(0), poly.max(0)
    X, Y = np.meshgrid(_samples(x0, x1, step), _samples(y0, y1, step), indexing="ij")
    xy = np.stack([X.ravel(), Y.ravel()], 1)
    return xy[points_in_polygon(xy, poly) | _on_bbox_edge(xy, poly)]


def _on_bbox_edge(xy, poly):
    """Rectangular zones: keep samples on the zone's own edges (the even-odd
    test is ambiguous exactly on the boundary)."""
    if len(poly) != 4:
        return np.zeros(len(xy), bool)
    (x0, y0), (x1, y1) = poly.min(0), poly.max(0)
    is_rect = np.all(np.isclose(poly[:, 0], x0) | np.isclose(poly[:, 0], x1)) and \
        np.all(np.isclose(poly[:, 1], y0) | np.isclose(poly[:, 1], y1))
    if not is_rect:
        return np.zeros(len(xy), bool)
    return (xy[:, 0] >= x0 - 1e-9) & (xy[:, 0] <= x1 + 1e-9) & (xy[:, 1] >= y0 - 1e-9) & (xy[:, 1] <= y1 + 1e-9)


def generate_candidates(site: Site) -> CandidateSet:
    cs = site.candidates
    rows = []

    def blocked(x, y):
        p = np.array([[x, y]])
        return any(points_in_polygon(p, r)[0] or distance_to_polygon(p, r)[0][0] < 1e-6
                   for r in site.no_go_zones)

    ceiling = site.volume.z_max + cs.min_height_above_volume_m     # cameras never below the drones
    truss_aims = _aims(cs.truss_elevations_deg, cs.azimuth_step_deg)
    for t in site.trusses:
        if t.height_m - cs.truss_drop_m < ceiling - 1e-9:
            continue
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
        lo, hi = max(z.height_range_m[0], ceiling), z.height_range_m[1]
        if lo > hi + 1e-9:
            continue                                   # masts too short to reach above the drones
        # the lowest allowed height (at the ceiling: closest to the drones) plus any listed heights above it
        heights = sorted({round(lo, 3)} | {h for h in cs.tripod_heights_m if lo <= h <= hi})
        for x, y in tripod_positions(site, z):
            if blocked(x, y) or site.volume.contains_xy(np.array([[x, y]]))[0]:
                continue
            # One tripod per floor spot; it may carry cameras at several heights
            # on its mast (the mount capacity is max_cameras_per_mount).
            key = f"{z.id}@{x:.2f},{y:.2f}"
            for h in heights:
                for az, el in tripod_aims:
                    for lens in site.lenses_mm:
                        rows.append(((x, y, h), az, el, lens, key, "tripod", z.id, np.nan))

    if not rows:
        if site.trusses or site.tripod_zones:
            raise ValueError(f"no mount reaches above the flight ceiling ({ceiling:g} m): cameras must be above "
                             f"the drones. Raise the trusses or the tripod mast heights, or lower the volume top.")
        raise ValueError("no candidate mounts: add trusses or tripod zones to the site file")
    pos, az, el, lens, key, mtype, mid, along = zip(*rows)
    lens = np.array(lens, float)
    return CandidateSet(
        pos=np.array(pos, float), az=np.array(az), el=np.array(el), lens_mm=lens,
        f_px=lens * 1e-3 / site.sensor.pixel_pitch_m, mount_key=np.array(key),
        mount_type=np.array(mtype), mount_id=np.array(mid), along_m=np.array(along, float))
