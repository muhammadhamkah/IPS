"""Turn optimiser output into an install plan, and evaluate any plan."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from idps.core.noise import P95_PER_RMS
from idps.coverage.fisher import contributions, rms_mm
from idps.coverage.volume import volume_grid
from idps.plan.candidates import CandidateSet, generate_candidates
from idps.plan.model import Plan, PlannedCamera, install_angles
from idps.plan.optimizer import Optimiser
from idps.plan.site import Site


@dataclass
class Evaluation:
    points: np.ndarray        # (V, 3)
    axes: tuple               # (xs, ys, zs)
    n_cams: np.ndarray        # (V,)
    p95_mm: np.ndarray        # (V,) predicted p95 3D error; inf if < 2 cameras
    meets: np.ndarray         # (V,) bool
    critical: np.ndarray      # (C,) points that would fail without each camera
    robust: np.ndarray        # (V,) bool: still meets requirements if any one camera is lost
    summary: dict


def site_grid(site: Site):
    return volume_grid(site.volume_min, site.volume_max, site.eval_spacing_m, site.eval_inset_m)


def evaluate(site: Site, cameras: list[PlannedCamera]) -> Evaluation:
    pts, axes = site_grid(site)
    req = site.requirements
    V = len(pts)
    if cameras:
        pos = np.array([c.position_m for c in cameras])
        R = np.array([c.R for c in cameras])
        f_px = np.array([site.sensor.f_px(c.lens_mm) for c in cameras])
        vis, inf = contributions(pos, R, f_px, pts, site.sensor, req.noise, site.limits)
    else:
        vis, inf = np.zeros((0, V), bool), np.zeros((0, V, 6))
    n = vis.sum(0)
    info = inf.sum(0)
    p95 = rms_mm(info) * P95_PER_RMS
    meets = (n >= req.min_cameras) & (p95 <= req.p95_mm)

    critical = np.zeros(len(cameras), int)
    robust = meets.copy()
    for k in range(len(cameras)):
        p95_k = rms_mm(info - inf[k]) * P95_PER_RMS
        meets_k = ((n - vis[k]) >= req.min_cameras) & (p95_k <= req.p95_mm)
        critical[k] = int((meets & ~meets_k).sum())
        robust &= meets_k

    vol = float(np.prod(site.volume_max - site.volume_min))
    finite = p95[np.isfinite(p95)]
    by_count = {str(k): float(np.mean(np.minimum(n, 3) == k)) for k in range(3)}
    by_count["3+"] = float(np.mean(n >= 3))
    summary = {
        "cameras": len(cameras),
        "cameras_on_trusses": sum(c.mount_type == "truss" for c in cameras),
        "cameras_on_tripods": sum(c.mount_type == "tripod" for c in cameras),
        "tripods": len({tuple(np.round(c.position_m, 2)) for c in cameras if c.mount_type == "tripod"}),
        "evaluation_points": int(V),
        "volume_m3": vol,
        "fraction_meeting_all_requirements": float(meets.mean()),
        "fraction_meeting_with_any_one_camera_lost": float(robust.mean()),
        "fraction_with_min_cameras": float(np.mean(n >= req.min_cameras)),
        "fraction_by_camera_count": by_count,
        "volume_below_min_cameras_m3": float(np.mean(n < req.min_cameras) * vol),
        "volume_over_accuracy_limit_m3": float(np.mean(~(p95 <= req.p95_mm)) * vol),
        "p95_error_mm_median": float(np.median(finite)) if finite.size else None,
        "p95_error_mm_worst": float(p95.max()) if np.isfinite(p95).all() else None,
        "requirements": {"min_cameras": req.min_cameras, "p95_mm": req.p95_mm,
                         "noise": req.noise.to_dict()},
    }
    return Evaluation(pts, axes, n, p95, meets, critical, robust, summary)


def _outside_note(site: Site, x: float, y: float) -> str:
    (x0, y0), (x1, y1) = site.volume_min[:2], site.volume_max[:2]
    parts = []
    if x < x0:
        parts.append(f"{x0 - x:.2f} m beyond the -X boundary")
    if x > x1:
        parts.append(f"{x - x1:.2f} m beyond the +X boundary")
    if y < y0:
        parts.append(f"{y0 - y:.2f} m beyond the -Y boundary")
    if y > y1:
        parts.append(f"{y - y1:.2f} m beyond the +Y boundary")
    return ", ".join(parts) if parts else "inside the volume footprint"


def build_cameras(site: Site, cands: CandidateSet, selected: list[int]) -> list[PlannedCamera]:
    trusses = {t.id: t for t in site.trusses}
    cams = []
    for i in selected:
        base, pan, tilt = install_angles(float(cands.az[i]), float(cands.el[i]))
        x, y, z = (float(v) for v in cands.pos[i])
        if cands.mount_type[i] == "truss":
            t = trusses[cands.mount_id[i]]
            s = float(cands.along_m[i])
            note = (f"Truss {t.id}: {s:.2f} m from its start end ({t.start[0]:.1f}, {t.start[1]:.1f}),"
                    f" camera hung {site.candidates.truss_drop_m:.2f} m below the truss")
            along = s
        else:
            note = (f"Tripod (zone {cands.mount_id[i]}) at X {x:.2f}, Y {y:.2f};"
                    f" {_outside_note(site, x, y)}; camera height {z:.2f} m")
            along = None
        note += f"; gimbal pan-zero towards azimuth {base:.0f} deg"
        cams.append(PlannedCamera(
            id="", mount_type=str(cands.mount_type[i]), mount_id=str(cands.mount_id[i]),
            position_m=[x, y, z], azimuth_deg=float(cands.az[i]), elevation_deg=float(cands.el[i]),
            lens_mm=float(cands.lens_mm[i]), along_truss_m=along, base_heading_deg=base,
            pan_deg=pan, tilt_deg=tilt, install_note=note))

    def sort_key(c):
        if c.mount_type == "truss":
            return (0, c.mount_id, c.along_truss_m, c.azimuth_deg, c.elevation_deg)
        return (1, c.mount_id, c.position_m[0], c.position_m[1], c.position_m[2], c.azimuth_deg)

    cams.sort(key=sort_key)
    for k, c in enumerate(cams, 1):
        c.id = f"C{k:03d}"
    _neighbour_spacing(cams)
    return cams


def _neighbour_spacing(cams: list[PlannedCamera]) -> None:
    """Distance from each camera's mount to the nearest *other* mount of the
    same kind (same truss for truss mounts; any tripod for tripods)."""
    for c in cams:
        if c.mount_type == "truss":
            others = [o.along_truss_m for o in cams if o.mount_type == "truss" and o.mount_id == c.mount_id
                      and abs(o.along_truss_m - c.along_truss_m) > 1e-6]
            c.neighbour_spacing_m = min((abs(o - c.along_truss_m) for o in others), default=None)
        else:
            p = np.array(c.position_m[:2])
            others = [np.array(o.position_m[:2]) for o in cams if o.mount_type == "tripod"
                      and np.linalg.norm(np.array(o.position_m[:2]) - p) > 1e-6]
            c.neighbour_spacing_m = (float(min(np.linalg.norm(o - p) for o in others))
                                     if others else None)


def optimise(site: Site, log=print, max_cameras: int | None = None):
    for w in site.warnings():
        log(f"warning: {w}")
    cands = generate_candidates(site)
    pts, _ = site_grid(site)
    result = Optimiser(site, cands, pts, log=log).run(max_cameras=max_cameras)
    cams = build_cameras(site, cands, result.selected)
    ev = evaluate(site, cams)
    summary = dict(ev.summary)
    summary["optimiser"] = {"candidates": len(cands), "seconds": round(result.seconds, 1),
                            "stopped_because": result.stopped_because}
    plan = Plan(site.name, site.source_path, cams, summary)
    return plan, ev
